#include <Bluepad32.h>
#include <math.h>

#define L_RPWM 19
#define L_LPWM 18
#define L_REN  16
#define L_LEN  4
#define R_RPWM 32
#define R_LPWM 33
#define R_REN  27
#define R_LEN  14

#define PWM_FREQ 20000
#define PWM_RES 8
#define CH_L_FWD 0
#define CH_L_REV 1
#define CH_R_FWD 2
#define CH_R_REV 3

// 90 % ~= 230/255 ; R1 peut monter jusqu'à 100 % = 255.
int puissance = 90;
const int PUISSANCE_MIN = 40;
const int PUISSANCE_MAX = 100;
const int PUISSANCE_PAS = 5;
const int PWM_MIN = 35;

const bool INVERSE_GAUCHE = false;
const bool INVERSE_DROITE = true;
const float TRIM_GAUCHE = 1.00f;
const float TRIM_DROITE = 1.00f;

const int DEADZONE_STICK = 40;
const int DEADZONE_TRIGGER = 40;
const float STICK_CURVE = 1.5f;
const float STEERING_GAIN = 0.80f;
const float RAMP_UP = 8.0f;
const float RAMP_DOWN = 16.0f;
const uint32_t PERIODE_MS = 10;
const uint32_t TIMEOUT_MANETTE = 500;

bool inverseAvant = false;

struct Etape {
  int16_t g;
  int16_t d;
  uint16_t ms;
};

// Valeurs d'exemple seulement : calibrer sur le vrai parcours.
const Etape SEQUENCE[] = {
  {150, 150, 1500},
  {170, 110, 1200},
  {150, 150, 800},
  {110, 170, 1200},
  {150, 150, 1000}
};
const uint8_t NB_ETAPES = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

struct Cote {
  uint8_t pinAvant, pinArriere, canalAvant, canalArriere;
  bool inverse;
  float trim;
};

Cote coteGauche = {L_RPWM, L_LPWM, CH_L_FWD, CH_L_REV,
                   INVERSE_GAUCHE, TRIM_GAUCHE};
Cote coteDroit = {R_RPWM, R_LPWM, CH_R_FWD, CH_R_REV,
                  INVERSE_DROITE, TRIM_DROITE};

ControllerPtr manette = nullptr;
uint32_t dernierSignal = 0, dernierTick = 0, debutEtape = 0;
float cibleG = 0, cibleD = 0, actuelG = 0, actuelD = 0;
bool modeAuto = false, stopVerrouille = false;
uint8_t etapeAuto = 0;
bool ancienX = false, ancienCercle = false, ancienCarre = false;
bool ancienR1 = false, ancienL1 = false;

void commanderCote(const Cote &c, float commande) {
  if (c.inverse) commande = -commande;
  commande *= c.trim;
  int pwm = constrain((int)lroundf(commande), -255, 255);

  // Une seule entrée PWM active à la fois.
  if (pwm > 0) {
    ledcWrite(c.canalAvant, pwm);
    ledcWrite(c.canalArriere, 0);
  } else if (pwm < 0) {
    ledcWrite(c.canalAvant, 0);
    ledcWrite(c.canalArriere, -pwm);
  } else {
    ledcWrite(c.canalAvant, 0);
    ledcWrite(c.canalArriere, 0);
  }
}

void stopRobot() {
  commanderCote(coteGauche, 0);
  commanderCote(coteDroit, 0);
}

void arretUrgence() {
  modeAuto = false;
  cibleG = cibleD = 0;
  actuelG = actuelD = 0;
  stopRobot();
}

float normaliserStick(int valeur) {
  if (abs(valeur) <= DEADZONE_STICK) return 0.0f;
  float signe = valeur >= 0 ? 1.0f : -1.0f;
  float x = (float)(abs(valeur) - DEADZONE_STICK) /
            (511.0f - DEADZONE_STICK);
  x = constrain(x, 0.0f, 1.0f);
  return signe * powf(x, STICK_CURVE);
}

float normaliserTrigger(int valeur) {
  if (valeur <= DEADZONE_TRIGGER) return 0.0f;
  float x = (float)(valeur - DEADZONE_TRIGGER) /
            (1023.0f - DEADZONE_TRIGGER);
  return constrain(x, 0.0f, 1.0f);
}

float versPWM(float commande) {
  float a = fabsf(commande);
  if (a < 0.001f) return 0.0f;
  float limite = 255.0f * puissance / 100.0f;
  float pwm = PWM_MIN + a * (limite - PWM_MIN);
  return commande < 0 ? -pwm : pwm;
}

float rampe(float actuel, float cible) {
  bool memeSens = fabsf(cible) < 0.001f ||
                  fabsf(actuel) < 0.001f ||
                  ((actuel > 0) == (cible > 0));
  bool acceleration = memeSens && fabsf(cible) > fabsf(actuel);
  float pas = acceleration ? RAMP_UP : RAMP_DOWN;
  if (actuel < cible) return fminf(actuel + pas, cible);
  if (actuel > cible) return fmaxf(actuel - pas, cible);
  return actuel;
}

bool lireCommandeManuelle() {
  if (manette == nullptr) return false;

  float x = normaliserStick(manette->axisX());
  float y = -normaliserStick(manette->axisY());
  float r2 = normaliserTrigger(manette->throttle());
  float l2 = normaliserTrigger(manette->brake());

  // Y du stick prioritaire. Si Y est centré, R2/L2 font avancer/reculer.
  float gaz = fabsf(y) > 0.001f ? y : r2 - l2;
  if (fabsf(x) < 0.001f && fabsf(gaz) < 0.001f) return false;

  if (inverseAvant) gaz = -gaz;

  float g = gaz + x * STEERING_GAIN;
  float d = gaz - x * STEERING_GAIN;
  float m = fmaxf(fabsf(g), fabsf(d));
  if (m > 1.0f) { g /= m; d /= m; }

  cibleG = versPWM(g);
  cibleD = versPWM(d);
  return true;
}

void demarrerAuto() {
  modeAuto = true;
  stopVerrouille = false;
  etapeAuto = 0;
  debutEtape = millis();
  cibleG = cibleD = 0;
  actuelG = actuelD = 0;
  Serial.println(F("AUTONOMIE : DEBUT"));
}

void conduiteAutonome() {
  if (etapeAuto >= NB_ETAPES) {
    modeAuto = false;
    cibleG = cibleD = 0;
    Serial.println(F("AUTONOMIE : FIN"));
    return;
  }
  if (millis() - debutEtape >= SEQUENCE[etapeAuto].ms) {
    etapeAuto++;
    debutEtape = millis();
    if (etapeAuto >= NB_ETAPES) {
      modeAuto = false;
      cibleG = cibleD = 0;
      Serial.println(F("AUTONOMIE : FIN"));
      return;
    }
  }
  cibleG = SEQUENCE[etapeAuto].g;
  cibleD = SEQUENCE[etapeAuto].d;
}

void gererBoutons() {
  bool bX = manette->a();       // Cross / X PS4
  bool bCercle = manette->b();  // Circle
  bool bCarre = manette->x();   // Square
  bool bR1 = manette->r1();
  bool bL1 = manette->l1();

  if (bCercle && !ancienCercle) {
    arretUrgence();
    stopVerrouille = true;
    Serial.println(F("STOP"));
  }
  if (bCarre && !ancienCarre) {
    inverseAvant = !inverseAvant;
    stopVerrouille = false;
    Serial.print(F("INVERSION AVANT/ARRIERE : "));
    Serial.println(inverseAvant ? F("ON") : F("OFF"));
  }
  if (bR1 && !ancienR1) {
    puissance = constrain(puissance + PUISSANCE_PAS,
                          PUISSANCE_MIN, PUISSANCE_MAX);
    Serial.print(F("PUISSANCE MAX = "));
    Serial.print(puissance); Serial.println(F("%"));
  }
  if (bL1 && !ancienL1) {
    puissance = constrain(puissance - PUISSANCE_PAS,
                          PUISSANCE_MIN, PUISSANCE_MAX);
    Serial.print(F("PUISSANCE MAX = "));
    Serial.print(puissance); Serial.println(F("%"));
  }
  if (bX && !ancienX && !stopVerrouille) demarrerAuto();

  ancienX = bX;
  ancienCercle = bCercle;
  ancienCarre = bCarre;
  ancienR1 = bR1;
  ancienL1 = bL1;
}

void onConnectedController(ControllerPtr ctl) {
  if (manette == nullptr) {
    manette = ctl;
    dernierSignal = millis();
    stopVerrouille = false;
    Serial.println(F("PS4 CONNECTEE"));
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (manette == ctl) {
    manette = nullptr;
    arretUrgence();
    stopVerrouille = true;
    Serial.println(F("PS4 DECONNECTEE -> STOP"));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(L_REN, OUTPUT); pinMode(L_LEN, OUTPUT);
  pinMode(R_REN, OUTPUT); pinMode(R_LEN, OUTPUT);

  ledcSetup(CH_L_FWD, PWM_FREQ, PWM_RES);
  ledcSetup(CH_L_REV, PWM_FREQ, PWM_RES);
  ledcSetup(CH_R_FWD, PWM_FREQ, PWM_RES);
  ledcSetup(CH_R_REV, PWM_FREQ, PWM_RES);

  ledcAttachPin(L_RPWM, CH_L_FWD);
  ledcAttachPin(L_LPWM, CH_L_REV);
  ledcAttachPin(R_RPWM, CH_R_FWD);
  ledcAttachPin(R_LPWM, CH_R_REV);

  stopRobot();
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println(F("ROBOT PRET — JGB37-550 12V 500RPM"));
  Serial.println(F("Puissance initiale 90% (~230 PWM). Tester roues levees."));
}

void loop() {
  bool nouvellesDonnees = BP32.update();
  uint32_t maintenant = millis();
  if (nouvellesDonnees) dernierSignal = maintenant;

  if (manette == nullptr || !manette->isConnected() ||
      (maintenant - dernierSignal) > TIMEOUT_MANETTE) {
    arretUrgence();
    vTaskDelay(1);
    return;
  }

  if (maintenant - dernierTick < PERIODE_MS) {
    vTaskDelay(1);
    return;
  }
  dernierTick = maintenant;

  gererBoutons();
  bool manuel = lireCommandeManuelle();

  if (manuel) {
    stopVerrouille = false;
    if (modeAuto) {
      modeAuto = false;
      Serial.println(F("AUTONOMIE INTERROMPUE : MANUEL"));
    }
  }

  if (stopVerrouille) {
    cibleG = cibleD = 0;
  } else if (manuel) {
    // cibleG et cibleD ont été calculées par lireCommandeManuelle().
  } else if (modeAuto) {
    conduiteAutonome();
  } else {
    cibleG = cibleD = 0;
  }

  actuelG = rampe(actuelG, cibleG);
  actuelD = rampe(actuelD, cibleD);
  commanderCote(coteGauche, actuelG);
  commanderCote(coteDroit, actuelD);
}
