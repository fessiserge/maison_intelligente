#include <HCSR04.h>
#include <LCD_I2C.h>
#include <U8g2lib.h>

#include "Alarm.h"
#include "PorteAutomatique.h"

// === Définition des broches ===
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define BUZZER_PIN 22
#define RED_PIN 10
#define GREEN_PIN 9
#define BLUE_PIN 8
#define IN_1 39
#define IN_2 41
#define IN_3 43
#define IN_4 45
#define CLK_PIN 35
#define DIN_PIN 31
#define CS_PIN 33


HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);


float distance = 0.0;
unsigned long currentTime = 0;
const char* DA = "2405238";

enum AffichageEtat {
  AUCUN,
  AFFICHAGE_CONFIRMATION, 
  AFFICHAGE_ERREUR, 
  AFFICHAGE_INCONNU
};
AffichageEtat etatAffichage = AUCUN;


Alarm alarm(RED_PIN, GREEN_PIN, BLUE_PIN, BUZZER_PIN, &distance);
PorteAutomatique porte(IN_1, IN_2, IN_3, IN_4, distance);


unsigned long debutAffichage = 0;
const unsigned long DUREE_AFFICHAGE = 3000;
const int DISPLAY_INTERVAL = 100;
const int DISTANCE_INTERVAL = 50;
const int SERIAL_INTERVAL = 100;
const int screenDelayTime = 3000;
const int zero = 0;
int Limite_inferieure = 20;
int Limite_superieure = 30;

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();

  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.setContrast(5);

  porte.setAngleFerme(10);
  porte.setAngleOuvert(170);

  alarm.setColourA(255, 0, 0);
  alarm.setColourB(0, 0, 255);
  alarm.setDistance(15); // seuil déclenchement
  alarm.setTimeout(2000);
  alarm.setVariationTiming(150);

  departureDisplay();
}


void loop() {
  currentTime = millis();
  distanceTask();
  porte.update();
  alarm.update();
  screenDisplay();
  commandGestion();
  gererAffichageU8g2();
}

void gererAffichageU8g2() {
  if (etatAffichage == AUCUN) return;

  if (millis() - debutAffichage > DUREE_AFFICHAGE) {
    u8g2.clearDisplay();
    etatAffichage = AUCUN;
    return;
  }

  u8g2.clearBuffer();
  switch (etatAffichage) {
    case AFFICHAGE_CONFIRMATION:
      u8g2.drawLine(1, 5, 3, 7);
      u8g2.drawLine(3, 7, 7, 1);
      break;

    case AFFICHAGE_ERREUR:
      u8g2.drawCircle(3, 3, 3);
      u8g2.drawLine(0, 0, 7, 7);
      break;

    case AFFICHAGE_INCONNU:
      u8g2.drawLine(7, 7, 0, 0);
      u8g2.drawLine(0, 7, 7, 0);
      break;

    default:
      break;
  }
  u8g2.sendBuffer();
}



void departureDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(DA);
  lcd.setCursor(0, 1);
  lcd.print("Labo 07");
  delay(2000);
  lcd.clear();
}

void distanceTask() {
  static unsigned long last = 0;
  if (millis() - last >= DISTANCE_INTERVAL) {
    distance = hc.dist();
    last = millis();
  }
}

void screenDisplay() {
  static unsigned long last = 0;
  if (millis() - last >= DISPLAY_INTERVAL) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm");

    lcd.setCursor(0, 1);
    lcd.print("Porte: ");
    lcd.print(porte.getEtatTexte());

    last = millis();
  }
}

void confirmer() {
  etatAffichage = AFFICHAGE_CONFIRMATION;
  debutAffichage = millis();
}

void erreur() {
  etatAffichage = AFFICHAGE_ERREUR;
  debutAffichage = millis();
}

void inconnu() {
  etatAffichage = AFFICHAGE_INCONNU;
  debutAffichage = millis();
}


void commandGestion() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  if (command == "g_dist") {
    Serial.print("Arduino: ");
    Serial.println(distance);
    confirmer();
  } else if (command.startsWith("cfg;alm;")) {
    int val = command.substring(8).toInt();
    if (val > zero) {
      alarm.setDistance(val);
      Serial.print("Distance d'alarme config à ");
      Serial.println(val);
      confirmer();
    } else erreur();
  } else if (command.startsWith("cfg;lim_inf;")) {
    int val = command.substring(12).toInt();
    if (val < Limite_superieure && val > zero) {
      Limite_inferieure = val;
      porte.setDistanceOuverture(Limite_inferieure);
      confirmer();
    } else erreur();
  } else if (command.startsWith("cfg;lim_sup;")) {
    int val = command.substring(12).toInt();
    if (val > Limite_inferieure) {
      Limite_superieure = val;
      porte.setDistanceFermeture(Limite_superieure);
      confirmer();
    } else erreur();
  } else {
    inconnu();
  }
}
