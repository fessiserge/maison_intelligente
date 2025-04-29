#include <HCSR04.h>
#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <AccelStepper.h>
#include <U8g2lib.h>


#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define BUZZER_PIN 22
#define RED_PIN 10
#define GREEN_PIN 9
#define BLUE_PIN 8
#define MOTOR_INTERFACE_TYPE 4
#define IN_1 39
#define IN_2 41
#define IN_3 43
#define IN_4 45

#define CLK_PIN 35
#define DIN_PIN 31
#define CS_PIN 33


HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);




const char* DA = "2405238";
unsigned long currentTime = 0;
unsigned long doorMoveTime = 0;
float distance = 0.0;
int degree = 10;
int minAngle = 10;
int maxAngle = 170;
long minStep = (minAngle * 2038.0) / 360;
long maxStep = (maxAngle * 2038.0) / 360;

bool isMoving = false;
bool isRed = false;

const int RGB_PINS[] = { RED_PIN, GREEN_PIN, BLUE_PIN };
const int RGB_PINS_SIZE = sizeof(RGB_PINS) / sizeof(int);
const int DISPLAY_INTERVAL = 100;
const int DISTANCE_INTERVAL = 50;
const int SERIAL_INTERVAL = 100;
const int DOOR_MOVE_INTERVAL = 2000;
int alertDistance = 15; 
const int maxRawDist = 400; 
const int minRawDist = 0;  
const int screenDelayTime = 3000;


enum Etat {
  FERMEE,
  OUVERTURE,
  OUVERTE,
  FERMETURE
} currentState = FERMEE;

enum ALERTE {
  DESACTIVEE,
  ACTIVEE
} alertStatus = DESACTIVEE;

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
  pinMode(BUZZER_PIN, OUTPUT);

  for (int i = 0; i < RGB_PINS_SIZE; i++) {
    pinMode(RGB_PINS[i], OUTPUT);
  }

  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(500);
  myStepper.moveTo(minStep);
  myStepper.enableOutputs();

  departureDisplay();
  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tr);  
  u8g2.setContrast(5);             


  
}

void loop() {
  currentTime = millis();
  distance = distanceTask(currentTime);
  stateManager();
  
  alertManager(currentTime);
  motorTask();
  screenDisplay(currentTime);
  //serialPrint(currentTime);
  commandGestion();
}

float distanceTask(unsigned long ct) {
  static unsigned long previousDistance = 0;
  if (ct - previousDistance < DISTANCE_INTERVAL) return distance;
  if (ct - previousDistance >= DISTANCE_INTERVAL) {
    distance = hc.dist();
    previousDistance = ct;
  }
  return distance;
}

void stateManager() {
  switch (currentState) {
    case FERMEE:
      if (distance < 30 && distance > 0) {
        currentState = OUVERTURE;
        doorMoveTime = currentTime;
        isMoving = true;
        myStepper.enableOutputs();
        myStepper.moveTo(maxStep);
      }
      break;

    case OUVERTURE:
      if (!isMoving) {
        currentState = OUVERTE;
        myStepper.disableOutputs();
      }
      break;

    case OUVERTE:
      if (distance > 60) {
        currentState = FERMETURE;
        doorMoveTime = currentTime;
        isMoving = true;
        myStepper.enableOutputs();
        myStepper.moveTo(minStep);
      }
      break;

    case FERMETURE:
      if (!isMoving) {
        currentState = FERMEE;
        myStepper.disableOutputs();
      }
      break;
  }
}

void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

void blinkRedBlue(unsigned long ct) {
  static unsigned long lastBlinkTime = 0;
  const int blinkInterval = 150;

  if (ct - lastBlinkTime >= blinkInterval) {
    if (isRed) {
      setColor(255, 0, 0);
    } else {
      setColor(0, 0, 255);
    }
    isRed = !isRed;
    lastBlinkTime = ct;
  }
}

void alertManager(unsigned long ct) {
  switch (alertStatus) {
    case DESACTIVEE:
      if (distance <= 15 && distance > 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        blinkRedBlue(ct);
        alertStatus = ACTIVEE;
      }
      break;

    case ACTIVEE:
      static unsigned long previousTime = 0;
      const short interval = 300;

      if (distance > 15 && (ct - previousTime) > interval) {
        previousTime = ct;

        digitalWrite(BUZZER_PIN, LOW);
        for (int i = 0; i < RGB_PINS_SIZE; i++) {
          digitalWrite(RGB_PINS[i], LOW);
        }
        alertStatus = DESACTIVEE;
      } else {
        blinkRedBlue(ct);
      }
      break;
  }
}

void motorTask() {
  if (isMoving) {
    myStepper.run();
    int currentPos = myStepper.currentPosition();
    degree = map(currentPos, minStep, maxStep, minAngle, maxAngle);
    if (!myStepper.isRunning()) {
      isMoving = false;
    }
  }
}

void departureDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(DA);
  lcd.setCursor(0, 1);
  lcd.print("Labo 4A");
  delay(2000);
  lcd.clear();
}

void screenDisplay(unsigned long ct) {
  static unsigned long previousDisplay = 0;

  if (ct - previousDisplay >= DISPLAY_INTERVAL) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm");

    lcd.setCursor(0, 1);
    switch (currentState) {
      case FERMEE:
        lcd.print("Porte: Fermee");
        break;
      case OUVERTE:
        lcd.print("Porte: Ouverte");
        break;
      case OUVERTURE:
      case FERMETURE:
        lcd.print("Porte: ");
        lcd.print(degree);
        lcd.print(" deg");
        break;
    }
    previousDisplay = ct;
  }
}

void confirm() {
  unsigned long start = millis();
  while (millis() - start < screenDelayTime) {
    u8g2.clearBuffer();
    u8g2.drawLine(1, 5, 3, 7);
    u8g2.drawLine(3, 7, 7, 1);
    u8g2.sendBuffer();
  }
  u8g2.clear();
}

void error() {
  unsigned long start = millis();
  while (millis() - start < screenDelayTime) {
    u8g2.clearBuffer();
    u8g2.drawCircle(3, 3, 3);
    u8g2.drawLine(0, 0, 7, 7);
    u8g2.sendBuffer();
  }
  u8g2.clear();
}

void inconnu() {
  unsigned long start = millis();
  while (millis() - start < screenDelayTime) {
    u8g2.clearBuffer();
    u8g2.drawLine(7, 7, 0, 0);
    u8g2.drawLine(0, 7, 7, 0);
    u8g2.sendBuffer();
  }
  u8g2.clear();
}

void commandGestion() {
  if (!Serial.available()) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  if (command == "g_dist") {
    Serial.print("Arduino: ");
    Serial.println(distance);
    confirm();
  } 
  else if (command.startsWith("cfg;alm;")) {
    int valeur = command.substring(8).toInt();
    if (valeur > 0) {
      alertDistance = valeur;
      Serial.print("Distance d'alarme configurée à ");
      Serial.print(valeur);
      Serial.println(" cm");
      confirm();
    } else {
      error();
    }
  } 
  else if (command.startsWith("cfg;lim_inf;")) {
    int valeur = command.substring(12).toInt();
    if (valeur < maxAngle) {
      minAngle = valeur;
      minStep = (minAngle * 2038.0) / 360;
      Serial.print("Limite inférieure configurée à ");
      Serial.print(valeur);
      Serial.println(" degrés");
      confirm();
    } else {
      error();
    }
  } 
  else if (command.startsWith("cfg;lim_sup;")) {
    int valeur = command.substring(12).toInt();
    if (valeur > minAngle) {
      maxAngle = valeur;
      maxStep = (maxAngle * 2038.0) / 360;
      Serial.print("Limite supérieure configurée à ");
      Serial.print(valeur);
      Serial.println(" degrés");
      confirm();
    } else {
      error();
    }
  } 
  else {
    inconnu();
  }
}




/*void serialPrint(unsigned long ct) {
  static unsigned long previousSerial = 0;

  if (ct - previousSerial >= SERIAL_INTERVAL) {
    Serial.print("etd:");
    Serial.print(DA);
    Serial.print(",dist:");
    Serial.print(distance);
    Serial.print(",deg:");
    Serial.println(degree);

    previousSerial = ct;
  }
}*/
