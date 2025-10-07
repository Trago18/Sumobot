/*
  ---------------------------------------------------------------
  Comportamiento:
  1) Seguridad de borde (IR): si CUALQUIER sensor IR <= 70 (blanco),
     el robot retrocede y gira ~90° para no salir del dohyo.
  2) Ataque (Ultrasónico): si hay rival al frente (<= ATTACK_CM),
     entra en modo ATACAR y empuja a máxima potencia hasta perderlo (>= LOST_CM) o ver borde.
  3) Búsqueda: si no hay borde ni rival, hace un patrón de giro en el sitio ("spin")
     y luego un avance corto ("dash"). La dirección de giro alterna para explorar mejor.
  4) Telemetría por Serial: imprime IR, distancia US (cm) y estado.

  Ajustes rápidos (TUNING):
  - ATTACK_CM, LOST_CM: sensibilidad del encuentro con rival por ultrasónico.
  - PWM_*: potencias para avanzar/girar/retroceder.
  - BACK_MS, TURN_MS_90: tiempos de evasión (salida de borde + giro ~90°).
  - SEARCH_SPIN_MS, SEARCH_DASH_MS: patrón de búsqueda.
  - EDGE_LOCK_MS: “lockout” para no re-disparar borde inmediatamente.

  Hardware:
  - ESP32 (ADC en pines 36/39/34/35).
  - Puente H (L298N o similar) con mapeo de pines detallado abajo.
  - HC-SR04 al frente (TRIG=25, ECHO=26).
  - 1 NeoPixel en GPIO 2 para feedback visual.
*/

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

/* ===================== Sensores IR (borde del dohyo) ===================== */
/*
  Los 4 sensores IR leen un valor analógico:
  - En el dohyo NEGRO (incluso brillante), típicamente el valor es MAYOR que en blanco.
  - En el borde BLANCO el valor cae. El umbral valor_critico determina el cambio.
*/
const int sen1 = 36;  // IR frontal/posición 1 (ajusta según tu montaje)
const int sen2 = 39;  // IR frontal/posición 2
const int sen3 = 34;  // IR trasero/posición 3
const int sen4 = 35;  // IR trasero/posición 4
int sensores[] = {sen1, sen2, sen3, sen4};

const int valor_critico = 70; // <= 70 se considera BLANCO (borde). > 70 se considera NEGRO.

/* ===================== Ultrasónico (HC-SR04) ===================== */
/*
  Detecta oponente al frente. ATTACK_CM define la distancia para iniciar ataque.
  LOST_CM define cuándo se considera "perdido" (se sale del cono de detección o se aleja).
*/
const int TRIG_PIN = 25;   // GPIO para TRIG
const int ECHO_PIN = 26;   // GPIO para ECHO
const int ATTACK_CM = 28;  // [TUNING] Inicia ataque si dist <= 28 cm
const int LOST_CM   = 55;  // [TUNING] Considera perdido si dist >= 55 cm (o sin lectura)

/* ===================== Motores (L298N o similar) ===================== */
/*
  Mapeo comprobado en tus pruebas:
  - A_IN1 / A_IN2: canal motor izquierdo
  - B_IN1 / B_IN2: canal motor derecho
  Con setMotors(a1, a2, b1, b2) controlamos dirección y "pwm" por pin.
*/
#define A_IN1 13
#define A_IN2 15
#define B_IN1 12
#define B_IN2 14

// [TUNING] Potencias (0..255) y tiempos (ms) para movimientos básicos:
const int PWM_FORWARD = 230;     // Avance/ataque
const int PWM_BACK    = 200;     // Retroceso ante borde
const int PWM_TURN    = 220;     // Giro en sitio

const int BACK_MS        = 280;  // Tiempo de retroceso tras borde
const int TURN_MS_90     = 300;  // Tiempo aproximado para ~90° de giro
const int SEARCH_SPIN_MS = 280;  // Búsqueda: duración del giro en sitio
const int SEARCH_DASH_MS = 200;  // Búsqueda: avance corto tras el giro
const int EDGE_LOCK_MS   = 160;  // Lockout para no re-disparar el borde al instante

/* ===================== NeoPixel (feedback visual) ===================== */
/*
  LED de estado:
  - Blanco: avance/ataque
  - Rojo: evasión de borde
  - Azul: búsqueda
  - Apagado: quieto o STOP
*/
#define LED_PIN   2
#define NUM_LEDS  1
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

/* ===================== Estado de la FSM (máquina de estados) ===================== */
bool detenido = false;              // Permite congelar el robot con el comando "STOP" por Serial.
bool spinDirLeft = true;            // Dirección del giro en búsqueda; alterna en cada ciclo.
unsigned long tEdgeLockUntil = 0;   // Momento hasta el cual no revisamos borde (tras evasión).

// Estados de alto nivel:
enum class Estado : uint8_t { QUIETO, BUSCAR, ATACAR, EVADIR };
Estado estado = Estado::BUSCAR;     // Arrancamos buscando.

/* ===================== Helpers de motores ===================== */
/*
  setMotors:
    - Cada canal (a1,a2,b1,b2) corresponde a un pin. Con L298N, típicamente
      se usa un pin para "adelante" y otro para "atrás" por motor.
*/
void setMotors(int a1, int a2, int b1, int b2) {
  analogWrite(A_IN1, a1);
  analogWrite(A_IN2, a2);
  analogWrite(B_IN1, b1);
  analogWrite(B_IN2, b2);
}

// Detiene ambos motores
void stopMotors() { setMotors(0, 0, 0, 0); }

// Avanza recto. Si ves que el robot describe un arco, prueba la línea comentada.
void forwardMotors(int pwm) {
  // Alternativa si avanza girando: setMotors(pwm, 0, 0, pwm);
  setMotors(pwm, 0, pwm, 0);
}

// Retrocede por un tiempo fijo y luego se detiene
void backwardTimed(int pwm, int ms) {
  setMotors(0, pwm, 0, pwm);
  delay(ms);
  stopMotors();
}

// Giro continuo (para fase de búsqueda)
void spinLeft(int pwm)  { setMotors(0, pwm, pwm, 0); }
void spinRight(int pwm) { setMotors(pwm, 0, 0, pwm); }

// Giro por tiempo (usado tras evasión de borde)
void leftTurnTimed(int pwm, int ms)  { setMotors(0, pwm, pwm, 0); delay(ms); stopMotors(); }
void rightTurnTimed(int pwm, int ms) { setMotors(pwm, 0, 0, pwm); delay(ms); stopMotors(); }

/* ===================== Helpers de LED ===================== */
void ledOff()   { strip.setPixelColor(0, 0,0,0);                 strip.show(); }
void ledRed()   { strip.setPixelColor(0, strip.Color(255,0,0));  strip.show(); }  // Evasión
void ledWhite() { strip.setPixelColor(0, strip.Color(255,255,255)); strip.show(); } // Avance/ataque
void ledBlue()  { strip.setPixelColor(0, strip.Color(0,0,255));  strip.show(); }  // Búsqueda

/* ===================== Medición ultrasónica (cm) ===================== */
/*
  Retorna:
   - distancia en cm (entero) si hay lectura
   - -1 si no hay eco dentro del timeout
  Nota: cálculo en enteros para hacerlo ligero en ESP32.
*/
long measureDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout ~25 ms (~4.3 m). Si no hay eco, retorna 0 y tratamos como "sin lectura".
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (dur == 0) return -1;

  // Distancia aproximada en cm: (dur(us) * 343 m/s) / 20000 ≈ (us * 0.0343) / 2
  return (dur * 343L) / 20000L;
}

/* ===================== Setup: configuración inicial ===================== */
void setup() {
  Serial.begin(115200);   // Mantén el monitor serial en 115200 para ver texto legible.
  delay(200);

  // LED
  strip.begin();
  strip.show();

  // Motores
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);

  // Sensores IR (entradas analógicas)
  for (int i = 0; i < 4; i++) pinMode(sensores[i], INPUT);

  // Ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Estado inicial
  stopMotors();
  estado = Estado::BUSCAR;
  ledBlue();

  // Semilla para aleatoriedad (si luego quieres introducir giros aleatorios)
  randomSeed(micros());
}

/* ===================== Loop: lógica principal de combate ===================== */
void loop() {
  if (detenido) return;  // Si se envió "STOP" por Serial, queda congelado.

  // STOP por serial (útil en pruebas de mesa; en torneo puedes comentar esto para máximo rendimiento)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("STOP")) {
      stopMotors();
      ledOff();
      detenido = true;
      return;
    }
  }

  /* -------- 1) LECTURA IR: seguridad de borde -------- */
  bool edge = false;     // true si detectamos BLANCO (<= valor_critico)
  int  ir[4] = {0,0,0,0};

  // Durante EDGE_LOCK_MS tras una evasión no revisamos borde para evitar re-disparos
  if (millis() > tEdgeLockUntil) {
    for (int i = 0; i < 4; i++) {
      ir[i] = analogRead(sensores[i]);
      if (ir[i] <= valor_critico) edge = true;
    }
  } else {
    // Si estamos en lockout, igualmente leemos para imprimir
    for (int i = 0; i < 4; i++) ir[i] = analogRead(sensores[i]);
  }

  /* -------- 2) LECTURA US: detección de rival -------- */
  long dist = measureDistanceCM();
  bool targetAhead = (dist > 0 && dist <= ATTACK_CM); // rival “cerca” → atacar
  bool targetLost  = (dist < 0 || dist > LOST_CM);    // rival “perdido” → volver a buscar

  /* -------- 3) TELEMETRÍA: IR + US + Estado (solo imprime; no cambia lógica) -------- */
  Serial.print("IR: [");
  for (int i = 0; i < 4; i++) {
    Serial.print(ir[i]);
    if (i < 3) Serial.print(" ");
  }
  Serial.print("]  |  US: [");
  if (dist >= 0) Serial.print(dist); else Serial.print("N/A");
  Serial.print(" cm]  |  Estado: ");
  switch (estado) {
    case Estado::BUSCAR: Serial.println("BUSCAR"); break;
    case Estado::ATACAR: Serial.println("ATACAR"); break;
    case Estado::EVADIR: Serial.println("EVADIR"); break;
    case Estado::QUIETO: default: Serial.println("QUIETO"); break;
  }

  /* -------- 4) PRIORIDAD ABSOLUTA: evitar salir del dohyo -------- */
  if (edge) {
    // Evasión: retroceso + giro ~90°, alternando dirección para no caer en ciclos.
    stopMotors(); delay(30);
    ledRed();
    backwardTimed(PWM_BACK, BACK_MS);
    delay(15);
    if (spinDirLeft) leftTurnTimed(PWM_TURN, TURN_MS_90);
    else             rightTurnTimed(PWM_TURN, TURN_MS_90);
    spinDirLeft = !spinDirLeft; // alterna dirección para la próxima vez

    // Activa lockout de borde y vuelve a buscar
    tEdgeLockUntil = millis() + EDGE_LOCK_MS;
    ledBlue();
    estado = Estado::BUSCAR;
    return; // Saltamos el resto del ciclo para reaccionar más rápido
  }

  /* -------- 5) FSM – BÚSQUEDA / ATAQUE -------- */
  static unsigned long tState = 0; // cronómetro interno de la fase de búsqueda
  switch (estado) {
    case Estado::BUSCAR: {
      // Búsqueda: spin + dash (giro en sitio seguido de avance corto).
      ledBlue();
      unsigned long now = millis();

      if (now - tState < (unsigned long)SEARCH_SPIN_MS) {
        // Giro en sitio (alternando dirección entre ciclos)
        if (spinDirLeft) spinLeft(PWM_TURN);
        else             spinRight(PWM_TURN);
      } else if (now - tState < (unsigned long)(SEARCH_SPIN_MS + SEARCH_DASH_MS)) {
        // Avance de “exploración”
        forwardMotors(PWM_FORWARD);
      } else {
        // Reinicia ciclo y alterna la dirección del próximo giro
        tState = now;
        spinDirLeft = !spinDirLeft;
      }

      // Si detectamos rival adelante, cambiamos a ataque
      if (targetAhead) {
        estado = Estado::ATACAR;
        ledWhite();
      }
    } break;

    case Estado::ATACAR: {
      // Ataque: empuje continuo mientras no se pierda el rival
      ledWhite();
      forwardMotors(PWM_FORWARD);

      // Si se pierde el rival (o lectura inválida), vuelve a buscar
      if (targetLost) {
        estado = Estado::BUSCAR;
        tState = millis(); // reinicia ciclo de búsqueda
        ledBlue();
      }
    } break;

    case Estado::EVADIR: {
      // Este estado no se mantiene; la evasión ya se resuelve arriba y volvemos a BUSCAR
      estado = Estado::BUSCAR;
      tState = millis();
      ledBlue();
    } break;

    case Estado::QUIETO:
    default: {
      // Estado de seguridad (no debería usarse en torneo)
      stopMotors();
      ledOff();
    } break;
  }

  // Pequeño retardo para estabilidad/CPU (ajusta entre 20..40 ms según te sientas cómodo)
  delay(25);
}
