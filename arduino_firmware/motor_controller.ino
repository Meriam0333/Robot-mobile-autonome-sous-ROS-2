  /* 
  Mega 2560 + L298N (2 canaux) -> Robot 4 roues "différentiel"
  - 2 moteurs du côté GAUCHE en parallèle sur 1 canal
  - 2 moteurs du côté DROIT  en parallèle sur l'autre canal

  Objectif: garantir que les 2 roues d'un même côté reçoivent la même direction + PWM.

  Protocole série (115200):
    Pi -> Arduino :  V <lin_m_s> <ang_rad_s>\n
    Arduino -> Pi : ACK V <lin> <ang>\r\n
    Arduino -> Pi : R <dist_left_m> <dist_right_m>\r\n   (optionnel)

  SELF-TEST: fait tourner gauche puis droite pour vérifier le câblage.
*/

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// -------------------- CONFIG --------------------
const bool MOTOR_ENABLE = true;   // mets false si tu veux tester sans moteurs
const bool SELF_TEST    = true;   // mets true pour tester les côtés au démarrage (robot surélevé)

// Si ton câblage est inversé, tu peux inverser un côté ici (logiciel)
const bool INVERT_LEFT  = true ;
const bool INVERT_RIGHT = true ;

// Si tu as câblé: canal A = DROITE et canal B = GAUCHE, mets true:
const bool CHANNEL_A_IS_RIGHT = true;  // <-- adapte selon TON câblage

// -------------------- L298N PINS --------------------
// Canal A
const uint8_t ENA = 5;     // PWM
const uint8_t IN1 = 22;
const uint8_t IN2 = 23;

// Canal B
const uint8_t ENB = 6;     // PWM
const uint8_t IN3 = 24;
const uint8_t IN4 = 25;

// -------------------- ULTRASONS (optionnel) --------------------
const uint8_t TRIG_L = 30;
const uint8_t ECHO_L = 31;
const uint8_t TRIG_R = 32;
const uint8_t ECHO_R = 33;

// -------------------- PARAM ROBOT --------------------
const float WHEEL_BASE_M = 0.20f;   // distance gauche-droite (m), à calibrer
const float MAX_LIN = 0.35f;        // m/s (normalisation)
const float MAX_ANG = 2.0f;         // rad/s

const int PWM_MAX = 255;
const int PWM_MIN = 80;             // augmente si ça peine à démarrer
const float DEADZONE = 0.05f;

const unsigned long CMD_TIMEOUT_MS = 500;
const unsigned long RANGE_PERIOD_MS = 200;

// -------------------- ETAT --------------------
float cmd_lin = 0.0f, cmd_ang = 0.0f;
unsigned long last_cmd_ms = 0;
unsigned long last_range_ms = 0;

// -------------------- MOTEURS --------------------
void setChannel(uint8_t enPin, uint8_t inA, uint8_t inB, int pwmSigned) {
  pwmSigned = constrain(pwmSigned, -PWM_MAX, PWM_MAX);

  if (!MOTOR_ENABLE) pwmSigned = 0;

  if (pwmSigned > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(enPin, pwmSigned);
  } else if (pwmSigned < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(enPin, -pwmSigned);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(enPin, 0);
  }
}

void stopMotors() {
  setChannel(ENA, IN1, IN2, 0);
  setChannel(ENB, IN3, IN4, 0);
}

int normToPwm(float n) {
  n = constrain(n, -1.0f, 1.0f);
  if (fabs(n) < DEADZONE) return 0;

  int pwm = (int)(fabs(n) * PWM_MAX);
  pwm = max(pwm, PWM_MIN);
  return (n > 0) ? pwm : -pwm;
}

// Appliquer PWM gauche/droite en tenant compte du câblage (A=Right ou A=Left)
void applyLeftRight(int pwmLeft, int pwmRight) {
  if (INVERT_LEFT)  pwmLeft  = -pwmLeft;
  if (INVERT_RIGHT) pwmRight = -pwmRight;

  if (CHANNEL_A_IS_RIGHT) {
    // Canal A = DROITE, Canal B = GAUCHE
    setChannel(ENA, IN1, IN2, pwmRight);
    setChannel(ENB, IN3, IN4, pwmLeft);
  } else {
    // Canal A = GAUCHE, Canal B = DROITE
    setChannel(ENA, IN1, IN2, pwmLeft);
    setChannel(ENB, IN3, IN4, pwmRight);
  }
}

bool parseVCommand(const char* s, float &lin, float &ang) {
  // s: "V 0.00 0.00"
  if (s[0] != 'V') return false;

  const char* p = s + 1;
  while (*p == ' ') p++;
  if (*p == '\0') return false;
  lin = atof(p);

  // aller au prochain champ
  const char* q = p;
  while (*q && *q != ' ') q++;
  while (*q == ' ') q++;
  if (*q == '\0') return false;

  ang = atof(q);
  return true;
}


bool readCmdVel(float &lin, float &ang) {
  static char line[64];
  static uint8_t idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line[idx] = '\0';
      idx = 0;

      if (line[0] == 'V') {
        float a, b;
        if (parseVCommand(line, a, b)) {
          lin = a; ang = b;

          Serial.print("ACK V ");
          Serial.print(lin, 3);
          Serial.print(" ");
          Serial.print(ang, 3);
          Serial.print("\r\n");

          return true;
        } else {
          Serial.print("ERR BAD_V ");
          Serial.print(line);
          Serial.print("\r\n");
        }
      }
      return false;
    }

    if (idx < sizeof(line) - 1) line[idx++] = c;
    else idx = 0;
  }
  return false;
}

// -------------------- ULTRASON (optionnel) --------------------
float readUltrasonicMeters(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return -1.0f;
  return (duration * 1e-6f * 343.0f) / 2.0f;
}

// -------------------- SELF-TEST --------------------
void selfTestSequence() {
  // Robot surélevé !
  Serial.println("SELF_TEST_START\r");

  auto run = [&](int pwmL, int pwmR, unsigned long ms) {
    applyLeftRight(pwmL, pwmR);
    delay(ms);
    stopMotors();
    delay(500);
  };

  // 1) GAUCHE avant
  Serial.println("TEST: LEFT FORWARD\r");
  run(+160, 0, 1500);

  // 2) GAUCHE arrière
  Serial.println("TEST: LEFT BACKWARD\r");
  run(-160, 0, 1500);

  // 3) DROITE avant
  Serial.println("TEST: RIGHT FORWARD\r");
  run(0, +160, 1500);

  // 4) DROITE arrière
  Serial.println("TEST: RIGHT BACKWARD\r");
  run(0, -160, 1500);

  // 5) Rotation sur place
  Serial.println("TEST: ROTATE IN PLACE\r");
  run(+160, -160, 1500);

  Serial.println("SELF_TEST_END\r");
}

// -------------------- SETUP/LOOP --------------------
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  stopMotors();
  last_cmd_ms = millis();
  last_range_ms = millis();

  Serial.println("FW_READY\r");

  if (SELF_TEST) {
    delay(1000);
    selfTestSequence();
  }
}

void loop() {
  float lin, ang;
  if (readCmdVel(lin, ang)) {
    cmd_lin = constrain(lin, -MAX_LIN, MAX_LIN);
    cmd_ang = constrain(ang, -MAX_ANG, MAX_ANG);
    last_cmd_ms = millis();
  }

  // Watchdog
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    cmd_lin = 0.0f;
    cmd_ang = 0.0f;
  }

  // Différentiel: Twist -> v_left / v_right
  float v_left  = cmd_lin - cmd_ang * (WHEEL_BASE_M / 2.0f);
  float v_right = cmd_lin + cmd_ang * (WHEEL_BASE_M / 2.0f);

  float n_left  = (MAX_LIN > 1e-6f) ? (v_left / MAX_LIN) : 0.0f;
  float n_right = (MAX_LIN > 1e-6f) ? (v_right / MAX_LIN) : 0.0f;

  int pwmL = normToPwm(n_left);
  int pwmR = normToPwm(n_right);

  applyLeftRight(pwmL, pwmR);

  // Ultrasons -> R ... (optionnel)
  if (millis() - last_range_ms >= RANGE_PERIOD_MS) {
    last_range_ms = millis();
    float dl = readUltrasonicMeters(TRIG_L, ECHO_L);
    float dr = readUltrasonicMeters(TRIG_R, ECHO_R);

    Serial.print("R ");
    Serial.print(dl, 3);
    Serial.print(" ");
    Serial.print(dr, 3);
    Serial.print("\r\n");
  }
}
