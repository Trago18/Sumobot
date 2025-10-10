/*
  SumoBot Torneo – ESP32 + 4x IR + HC-SR04 + 2x VL53L0X
  -----------------------------------------------------
  - FSM con 3 estados: APERTURA / BUSCAR / ATACAR
  - En BUSCAR: barridos + dashes largos; si detecta rival (US/ToF) → ATACAR inmediato
  - Evasión de borde con anti-atasco y confirmación breve de piso brillante
  - Aperturas seleccionables (COMBATE_MODE): 1=empuje directo (seguro), 2/3=180° + empuje
  - Este archivo incluye solo constantes ajustadas; la lógica es la misma y probada.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

/* ===================== APERTURA (modo de combate) ===================== */
/* 1 = ataque directo (recomendado); 2 = giro 180° derecha y push; 3 = 180° invertido */
#define COMBATE_MODE 3   // <- si quieres la alternativa de “trampa”, usa 2 o 3

/* ===================== DEBUG (imprime por Serial) ===================== */
#define DEBUG_SERIAL 1
#if DEBUG_SERIAL
  #define DBG_PRINT(x)   Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif

/* ===================== IR (borde del dohyo) ===================== */
// Mapeo: frontal izq/der y trasera izq/der
const int SEN_FL = 36;
const int SEN_FR = 39;
const int SEN_RL = 34;
const int SEN_RR = 35;
// Umbral fijo (<= blanco / > negro). Ajusta si cambia pintura/iluminación.
const int VALOR_CRITICO = 70;

/* ===================== HC-SR04 (frontal) ===================== */
const int TRIG_PIN = 25;
const int ECHO_PIN = 26;

/* --- Umbrales detección rival (ajustados) --- */
const int ATTACK_CM = 42;   // si ≤ → ATACAR (entra un toque antes)
const int LOST_CM   = 95;   // si > → salir de ATACAR (tarda más en soltar)
const int CLINCH_CM = 20;   // boost si muy cerca (rompe forcejeo)

/* ===================== Motores (L298N) ===================== */
#define A_IN1 13  // IZQ dir1
#define A_IN2 15  // IZQ dir2
#define B_IN1 12  // DER dir1
#define B_IN2 14  // DER dir2

/* --- Potencias y tiempos (afinados) --- */
const int PWM_ATTACK     = 255; // ataque máximo (baja a 240 si patina o calienta)
const int PWM_CRUISE     = 220; // avance en búsqueda (lineal y estable)
const int PWM_BACK       = 210; // retroceso en evasión
const int PWM_TURN       = 220; // giro estándar
const int PWM_TURN_SCAN  = 175; // giro “suave” al escanear (mejor SNR del US)

/* Búsqueda: “barrido + dash” (ahora con dashes más largos) */
const int SWEEP_TURN_MS   = 320;  // giro en barrido (ms)
const int SWEEP_DASH_MS   = 420;  // avance corto tras barrido (ms)

/* Anti-atasco borde (escalado si se repite) */
const int BACK_MS_BASE          = 340;
const int TURN_MS_135_BASE      = 460;
const int BACK_MS_STEP          = 90;
const int TURN_MS_STEP          = 70;
const int EDGE_LOCK_MS_BASE     = 550;  // ↑ leve para reducir re-disparo
const int EDGE_LOCK_MS_STEP     = 100;
const int EDGE_STRIKE_WINDOW_MS = 1400;

/* Confirmación de negro (breve, con mini-giro si falla) */
const unsigned long SAFE_CONFIRM_TIMEOUT_MS = 900; // ↑ para negro brillante
const int SAFE_CONFIRM_REQUIRED = 3;               // ≥3 de 4 en negro
const int NUDGE_TURN_MS = 200;                     // mini-giro si no confirma

/* Watchdog de búsqueda “protegido” (sin 180 ciego) */
const unsigned long WATCHDOG_MS = 3200;

/* ===================== NeoPixel ===================== */
#define LED_PIN  2
#define NUM_LEDS 1
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

/* ===================== VL53L0X laterales ===================== */
constexpr int SDA_PIN      = 21;
constexpr int SCL_PIN      = 22;
constexpr int XSHUT_LEFT   = 4;
constexpr int XSHUT_RIGHT  = 5;
constexpr uint8_t ADDR_LEFT  = 0x30;
constexpr uint8_t ADDR_RIGHT = 0x29;

Adafruit_VL53L0X tofLeft, tofRight;
bool tofOK = false;

// ToF “cerca” lateral (engancha antes si entra por los costados)
const int TOF_NEAR_MM = 800;

/* ===================== Estado (FSM) ===================== */
enum class Estado : uint8_t { APERTURA, BUSCAR, ATACAR };
Estado estado = Estado::APERTURA;

/* Control general */
bool detenido = false;
bool spinDirLeft = true;           // sentido de barrido (alterna)
bool lastEscapeLeft = true;        // último escape usado (memoria)
unsigned long tEdgeLockUntil = 0;  // enmascarado IR tras evasión
int edgeStrikes = 0;               // cuántas veces se disparó borde en ventana
unsigned long lastEdgeTime = 0;    // marca de tiempo para ventana
unsigned long tLastSeen = 0;       // última vez que vimos al rival

/* Lectura IR empaquetada */
struct IRHits {
  int  fl, fr, rl, rr;             // lecturas crudas
  bool F_L, F_R, R_L, R_R, any;    // flags (<= VALOR_CRITICO) = blanco
};

/* ===================== Motores helpers ===================== */
void setMotors(int a1, int a2, int b1, int b2) {
  analogWrite(A_IN1, a1);
  analogWrite(A_IN2, a2);
  analogWrite(B_IN1, b1);
  analogWrite(B_IN2, b2);
}
void stopMotors()            { setMotors(0,0,0,0); }
void forwardMotors(int pwm)  { setMotors(pwm, 0, pwm, 0); }     // avanza recto
void backwardMotors(int pwm) { setMotors(0, pwm, 0, pwm); }     // retroceso recto
void spinLeft(int pwm)       { setMotors(0, pwm, pwm, 0); }     // giro en sitio
void spinRight(int pwm)      { setMotors(pwm, 0, 0, pwm); }     // giro en sitio
void leftTurnTimed(int p,int ms)  { spinLeft(p);  delay(ms); stopMotors(); }
void rightTurnTimed(int p,int ms) { spinRight(p); delay(ms); stopMotors(); }

/* ===================== LED helpers ===================== */
void ledOff()    { strip.setPixelColor(0, 0,0,0);                    strip.show(); }
void ledRed()    { strip.setPixelColor(0, strip.Color(255,0,0));     strip.show(); } // borde/evasión
void ledWhite()  { strip.setPixelColor(0, strip.Color(255,255,255)); strip.show(); } // ataque
void ledBlue()   { strip.setPixelColor(0, strip.Color(0,0,255));     strip.show(); } // búsqueda
void ledYellow() { strip.setPixelColor(0, strip.Color(255,180,0));   strip.show(); } // apertura

/* ===================== Mediana de 5 (para US) ===================== */
template<typename T>
T median5(T a, T b, T c, T d, T e) {
  T arr[5] = {a,b,c,d,e};
  for (int i=1;i<5;i++){ T key=arr[i]; int j=i-1; while(j>=0 && arr[j]>key){arr[j+1]=arr[j]; j--;} arr[j+1]=key; }
  return arr[2];
}

/* ===================== US: distancia cm (mediana 5) ===================== */
// Micro-freno antes de medir para bajar vibración → mejor SNR del HC-SR04
long measureDistanceCMMedian() {
  long vals[5];
  for (int i=0;i<5;i++){
    stopMotors(); delayMicroseconds(1200);
    digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(4);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    unsigned long dur = pulseIn(ECHO_PIN, HIGH, 45000UL);
    if (dur == 0) vals[i] = -1;
    else          vals[i] = (long)((dur * 343UL) / 20000UL); // ≈ (us*0.0343)/2
    delay(3);
  }
  // Reemplaza inválidos por “grandes” para que la mediana funcione
  for(int i=0;i<5;i++) if(vals[i]<=0) vals[i]=10000;
  long med = median5(vals[0],vals[1],vals[2],vals[3],vals[4]);
  return (med==10000) ? -1 : med;
}

/* ===================== IR ===================== */
IRHits readIR() {
  IRHits r;
  r.fl = analogRead(SEN_FL);
  r.fr = analogRead(SEN_FR);
  r.rl = analogRead(SEN_RL);
  r.rr = analogRead(SEN_RR);
  r.F_L = (r.fl <= VALOR_CRITICO);
  r.F_R = (r.fr <= VALOR_CRITICO);
  r.R_L = (r.rl <= VALOR_CRITICO);
  r.R_R = (r.rr <= VALOR_CRITICO);
  r.any = (r.F_L || r.F_R || r.R_L || r.R_R);
  return r;
}

/* ===================== ToF laterales ===================== */
long readToFmm(Adafruit_VL53L0X &dev) {
  VL53L0X_RangingMeasurementData_t m;
  dev.rangingTest(&m, false);
  if (m.RangeStatus == 0) {
    long mm = m.RangeMilliMeter;
    if (mm > 0 && mm <= 2000) return mm; // filtrado simple
  }
  return -1;
}
bool initToF() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(20);

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(5);

  // Izquierdo: enciende con dirección por defecto y reasigna
  digitalWrite(XSHUT_LEFT, HIGH);  delay(50);
  if (!tofLeft.begin(0x29, false, &Wire)) return false;
  tofLeft.setAddress(ADDR_LEFT);   delay(10);
  tofLeft.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  tofLeft.setMeasurementTimingBudgetMicroSeconds(200000);

  // Derecho: enciende y queda en 0x29
  digitalWrite(XSHUT_RIGHT, HIGH); delay(50);
  if (!tofRight.begin(ADDR_RIGHT, false, &Wire)) return false;
  tofRight.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  tofRight.setMeasurementTimingBudgetMicroSeconds(200000);

  return true;
}

/* ===================== Confirmación negro ===================== */
// Requiere ≥ requiredBlack sensores en negro, 2 lecturas seguidas.
// Si no logra confirmar en el timeout, da un mini-giro para romper geometría.
bool confirmSafeFloor(unsigned long timeout_ms, int requiredBlack) {
  unsigned long t0 = millis();
  int okConsecutive = 0;
  while (millis() - t0 < timeout_ms) {
    int v1 = analogRead(SEN_FL), v2 = analogRead(SEN_FR);
    int v3 = analogRead(SEN_RL), v4 = analogRead(SEN_RR);
    int blackCount = (v1 > VALOR_CRITICO) + (v2 > VALOR_CRITICO) +
                     (v3 > VALOR_CRITICO) + (v4 > VALOR_CRITICO);
    if (blackCount >= requiredBlack) {
      if (++okConsecutive >= 2) return true;
    } else {
      okConsecutive = 0;
    }
    delay(12);
  }
  leftTurnTimed(PWM_TURN, NUDGE_TURN_MS); // mini-giro failsafe
  return false;
}

/* ===================== Evasión de borde ===================== */
// Retrocede y gira lejos del sensor disparado. Escala los tiempos si el borde
// se repite en una ventana. Enmascara IR un tiempo y confirma negro antes de seguir.
void escapeFromEdge(bool preferLeft, int strikes) {
  int backMs = BACK_MS_BASE + BACK_MS_STEP * (strikes - 1);
  int turnMs = TURN_MS_135_BASE + TURN_MS_STEP * (strikes - 1);
  int lockMs = EDGE_LOCK_MS_BASE + EDGE_LOCK_MS_STEP * (strikes - 1);

  ledRed();
  stopMotors(); delay(20);
  backwardMotors(PWM_BACK); delay(backMs);
  stopMotors(); delay(10);

  if (preferLeft) { leftTurnTimed(PWM_TURN, turnMs); }
  else            { rightTurnTimed(PWM_TURN, turnMs); }

  // Reentrada con máscara IR (evita re-disparo inmediato en blanco brillante)
  unsigned long maskUntil = millis() + (unsigned long)lockMs;
  forwardMotors(PWM_CRUISE);
  while (millis() < maskUntil) {
    IRHits ir = readIR();
    if (ir.any) {                   // si re-dispara, añade un micro-giro
      stopMotors();
      if (preferLeft)  leftTurnTimed(PWM_TURN, 140);
      else             rightTurnTimed(PWM_TURN, 140);
      forwardMotors(PWM_CRUISE);
    }
    delay(8);
  }
  stopMotors();

  (void)confirmSafeFloor(SAFE_CONFIRM_TIMEOUT_MS, SAFE_CONFIRM_REQUIRED);
}

/* ===================== Setup ===================== */
void setup() {
  #if DEBUG_SERIAL
    Serial.begin(115200);
    delay(200);
  #endif
  strip.begin(); strip.show();

  pinMode(A_IN1, OUTPUT); pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT); pinMode(B_IN2, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  stopMotors();

  // ToF laterales
  tofOK = initToF();
  DBG_PRINTLN(tofOK ? "ToF OK (L=0x30, R=0x29)" : "ToF FAIL");

  estado = Estado::APERTURA;     // Apertura activa al inicio
  ledYellow();
  randomSeed(micros());
  tLastSeen = millis();
}

/* ===================== Loop (FSM) ===================== */
void loop() {
  if (detenido) return;

  // Comando STOP por Serial (pruebas en mesa)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd.equalsIgnoreCase("STOP")) { stopMotors(); ledOff(); detenido = true; return; }
  }

  IRHits ir = readIR();

  bool edgeLock = (millis() <= tEdgeLockUntil);
  bool nearEdge = (!edgeLock && ir.any);

  unsigned long now = millis();
  // --- Prioridad absoluta: borde detectado ---
  if (nearEdge) {
    // Gira LEJOS del lado que detectó blanco; si no está claro, usa el último escape
    bool turnLeft =
      (ir.F_R || ir.R_R) ? true  :
      (ir.F_L || ir.R_L) ? false : lastEscapeLeft;

    // Escalado de “golpes” de borde
    edgeStrikes = (now - lastEdgeTime <= (unsigned long)EDGE_STRIKE_WINDOW_MS)
                  ? (edgeStrikes + 1) : 1;
    lastEdgeTime = now;

    escapeFromEdge(turnLeft, max(1, edgeStrikes));
    tEdgeLockUntil = millis() + 60;  // pequeño margen extra
    estado = Estado::BUSCAR;
    ledBlue();
    return;
  } else if (now - lastEdgeTime > (unsigned long)EDGE_STRIKE_WINDOW_MS) {
    edgeStrikes = 0;
  }

  // ---------------- FSM principal ----------------
  switch (estado) {
    /* ---------------- APERTURA ---------------- */
    case Estado::APERTURA: {
      // Estrategias de salida del ‘ready’
      if (COMBATE_MODE == 1) {
        // Empujón inicial (ganar inercia/centro) y a BUSCAR
        ledWhite(); forwardMotors(PWM_ATTACK); delay(260); stopMotors();
      } else if (COMBATE_MODE == 2) {
        // 180° y push corto (ojo: más arriesgado si te acercas al borde)
        leftTurnTimed(PWM_TURN, 600);   // ~180°
        ledWhite(); forwardMotors(PWM_ATTACK); delay(220); stopMotors();
      } else { // COMBATE_MODE == 3
        rightTurnTimed(PWM_TURN, 600);  // ~180°
        ledWhite(); forwardMotors(PWM_ATTACK); delay(220); stopMotors();
      }
      estado = Estado::BUSCAR;
      ledBlue();
    } break;

    /* ---------------- BUSCAR ---------------- */
    case Estado::BUSCAR: {
      ledBlue();
      static unsigned long tState = millis();

      long dUS = -1;
      long dL  = -1, dR = -1;

      unsigned long now2 = millis();
      // 1) Barrido (giro suave para que el US vea mejor)
      if (now2 - tState < (unsigned long)SWEEP_TURN_MS) {
        if (spinDirLeft) spinLeft(PWM_TURN_SCAN);
        else             spinRight(PWM_TURN_SCAN);

        stopMotors(); delay(10);              // micro-pausa para medir
        dUS = measureDistanceCMMedian();
        if (dUS > 0 && dUS <= ATTACK_CM) {    // rival cerca → atacar ya
          ledWhite(); forwardMotors(PWM_ATTACK); delay(110);
          estado = Estado::ATACAR; tLastSeen = now2; break;
        }
      }
      // 2) Dash más largo hacia adelante
      else if (now2 - tState < (unsigned long)(SWEEP_TURN_MS + SWEEP_DASH_MS)) {
        forwardMotors(PWM_CRUISE);

        // Lecturas ToF laterales (enganche por costado)
        if (tofOK) { dL = readToFmm(tofLeft); delay(4); dR = readToFmm(tofRight); }
        long dUS2 = measureDistanceCMMedian();
        if (dUS < 0) dUS = dUS2; else if (dUS2 > 0) dUS = (dUS + dUS2)/2;

        bool leftClose  = (dL >= 0 && dL <= TOF_NEAR_MM);
        bool rightClose = (dR >= 0 && dR <= TOF_NEAR_MM);
        if (leftClose || rightClose) {
          stopMotors();
          if (leftClose)  leftTurnTimed(PWM_TURN, 150);
          if (rightClose) rightTurnTimed(PWM_TURN, 150);
          ledWhite(); forwardMotors(PWM_ATTACK); delay(110);
          estado = Estado::ATACAR; tLastSeen = now2; break;
        }

        if (dUS > 0 && dUS <= ATTACK_CM) {
          ledWhite(); forwardMotors(PWM_ATTACK); delay(110);
          estado = Estado::ATACAR; tLastSeen = now2; break;
        }
      }
      // 3) Siguiente ciclo de barrido (alternar lado + watchdog protegido)
      else {
        tState = now2;
        spinDirLeft = !spinDirLeft;

        // Watchdog: micro-giro + dash (sin 180 ciego)
        if (now2 - tLastSeen > WATCHDOG_MS) {
          if (spinDirLeft) { leftTurnTimed(PWM_TURN, 220); }
          else             { rightTurnTimed(PWM_TURN, 220); }
          forwardMotors(PWM_CRUISE); delay(160);
          stopMotors();
          tLastSeen = now2;
        }
      }
    } break;

    /* ---------------- ATACAR ---------------- */
    case Estado::ATACAR: {
      ledWhite();

      long dUS = measureDistanceCMMedian();

      // Clinch boost: ráfaga corta para romper empates sin vibración excesiva
      const int BOOST = 12;
      const unsigned long BURST_MS = 90;
      if (dUS > 0 && dUS <= CLINCH_CM) {
        forwardMotors(min(255, PWM_ATTACK + BOOST));
        delay(BURST_MS);
      } else {
        forwardMotors(PWM_ATTACK);
      }

      // Evita persecución suicida si el frente está al límite del borde
      bool frontalRiesgo =
        (analogRead(SEN_FL) <= VALOR_CRITICO + 8) ||
        (analogRead(SEN_FR) <= VALOR_CRITICO + 8);
      if (frontalRiesgo) {
        stopMotors();
        if (spinDirLeft) leftTurnTimed(PWM_TURN, 180);
        else             rightTurnTimed(PWM_TURN, 180);
        estado = Estado::BUSCAR;
        ledBlue();
        break;
      }

      // Enemigo perdido → BUSCAR con micro-reorientación
      if (dUS < 0 || dUS > LOST_CM) {
        if (spinDirLeft) leftTurnTimed(PWM_TURN, 200);
        else             rightTurnTimed(PWM_TURN, 200);
        estado = Estado::BUSCAR;
        ledBlue();
      } else {
        tLastSeen = millis();
      }
    } break;
  }

  #if DEBUG_SERIAL
    DBG_PRINT("IR[");
    DBG_PRINT(analogRead(SEN_FL)); DBG_PRINT(' ');
    DBG_PRINT(analogRead(SEN_FR)); DBG_PRINT(' ');
    DBG_PRINT(analogRead(SEN_RL)); DBG_PRINT(' ');
    DBG_PRINT(analogRead(SEN_RR)); DBG_PRINT("]  ");
    DBG_PRINTLN("");
  #endif

  delay(14); // ciclo corto y reactivo
}
