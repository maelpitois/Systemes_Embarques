#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>
#include <ChainableLED.h>
#include <EEPROM.h>

#define GREEN_BUTTON_PIN 2
#define RED_BUTTON_PIN 3
#define LED_DATA_PIN 7
#define LED_CLOCK_PIN 8
#define LIGHT_SENSOR_PIN A2
#define SD_CS_PIN 4
#define HYGR_ADDR 10 // Adresse dans l'EEPROM pour la valeur de HYGR
#define LOG_INTERVALL_ADDR 14 // Adresse EEPROM pour LOG_INTERVALL

enum Mode { STANDARD, ECO, MAINTENANCE, CONFIGURATION };
Mode currentMode = STANDARD;
Mode previousMode = STANDARD;

uint8_t errorFlags = 0;
#define RTC_ERROR          0x01
#define GPS_ERROR          0x02
#define SD_CARD_FULL       0x04 // pas utiliser inutile
#define SD_WRITE_ERROR     0x08
#define SENSOR_DATA_ERROR  0x10

Adafruit_BME280 bme;
RTC_DS1307 rtc;
ChainableLED leds(LED_DATA_PIN, LED_CLOCK_PIN, 1);

unsigned long LOG_INTERVALL = 5000UL;
unsigned long TIMEOUT = 30000UL;
unsigned long FILE_MAX_SIZE = 4096UL;

uint8_t LUMIN = 1;
uint8_t LUMIN_LOW = 255;
uint16_t LUMIN_HIGH = 768;

uint8_t TEMP_AIR = 1;
int8_t MIN_TEMP_AIR = -10;
int8_t MAX_TEMP_AIR = 60;

uint8_t HYGR = 1;
uint8_t HYGR_MINT = 0;
uint8_t HYGR_MAXT = 50;

uint8_t PRESSION = 1;
uint16_t PRESSION_MIN = 850;
uint16_t PRESSION_MAX = 1080;

unsigned long lastDataAcquisitionTime = 0;
unsigned long modeStartTime = 0;
unsigned long maintenanceStartTime = 0;

bool isHygrActive = true;
bool isTempAirActive = true;
bool isLuminActive = true;

// Variables pour la gestion des boutons avec interruptions
volatile uint8_t buttonStates = 0;
#define Bouton_vert_etats     0x01
#define Bouton_vert_changement   0x02
#define Bouton_rouge_etat       0x04
#define Bouton_rouge_changement     0x08

// Définition de la structure SensorData pour stocker les données
typedef struct SensorData {
  unsigned long timestamp;
  float temperature;
  float PRESSION;
  float humidity;
  int luminosity;
  struct SensorData* next;
} SensorData;

SensorData* dataHead = NULL;

void setup() {
  Serial.begin(9600);
  delay(1000);

  initLED();
  initButton();

  // Lire la valeur HYGR de l'EEPROM
  HYGR = EEPROM.read(HYGR_ADDR);
  isHygrActive = (HYGR != 0);

  // Lire la valeur LOG_INTERVALL de l'EEPROM
  EEPROM.get(LOG_INTERVALL_ADDR, LOG_INTERVALL);
  if (LOG_INTERVALL == 0xFFFFFFFF) { // Si aucune valeur n'a été stockée
    LOG_INTERVALL = 5000UL; // Valeur par défaut
  }

  if (digitalRead(RED_BUTTON_PIN) == LOW) {
    modConfiguration();
  } else {
    modStandard();
  }

  if (!SD.begin(SD_CS_PIN)) {
    errorFlags |= SD_WRITE_ERROR;
  } else {
  }

  if (!bme.begin(0x76)) {
    errorFlags |= SENSOR_DATA_ERROR;
  }

  if (!rtc.begin()) {
    errorFlags |= RTC_ERROR;
  } else if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Configurer les interruptions pour les boutons sur changement d'état
  attachInterrupt(digitalPinToInterrupt(GREEN_BUTTON_PIN), greenButtonISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RED_BUTTON_PIN), redButtonISR, CHANGE);
}

void loop() {
  unsigned long currentTime = millis();

  Gestion_bouton();

  if ((currentMode == STANDARD || currentMode == ECO) && (currentTime - lastDataAcquisitionTime >= LOG_INTERVALL)) {
    lastDataAcquisitionTime = currentTime;
    recupDonnees();
    writeDataToSD();
  }

  if (currentMode == MAINTENANCE && (currentTime - lastDataAcquisitionTime >= 5000UL)) {
    lastDataAcquisitionTime = currentTime;
    recupDonnees();
    afficherDonneesConsole();
    writeDataToSD();
  }

  updateLEDs();
  etatsSystem();

  if (currentMode == CONFIGURATION) {
    Interface_serie_commands();
  }
}

// Fonctions d'interruption pour les boutons
void greenButtonISR() {
  if (digitalRead(GREEN_BUTTON_PIN) == LOW) {
    buttonStates &= ~Bouton_vert_etats;
  } else {
    buttonStates |= Bouton_vert_etats;
  }
  buttonStates |= Bouton_vert_changement;
}

void redButtonISR() {
  if (digitalRead(RED_BUTTON_PIN) == LOW) {
    buttonStates &= ~Bouton_rouge_etat;
  } else {
    buttonStates |= Bouton_rouge_etat;
  }
  buttonStates |= Bouton_rouge_changement;
}

void Gestion_bouton() {
  static unsigned long greenButtonPressTime = 0;
  static unsigned long redButtonPressTime = 0;
  static bool greenButtonPressed = false;
  static bool redButtonPressed = false;

  // Gestion du bouton vert
  if (buttonStates & Bouton_vert_changement) {
    buttonStates &= ~Bouton_vert_changement;
    if (!(buttonStates & Bouton_vert_etats)) {
      // Bouton pressé
      greenButtonPressTime = millis();
      greenButtonPressed = true;
    } else {
      // Bouton relâché
      if (greenButtonPressed) {
        unsigned long duration = millis() - greenButtonPressTime;
        greenButtonPressed = false;
        if (duration >= 3000UL) {
          // Bouton maintenu pendant au moins 3 secondes
          if (currentMode == STANDARD) {
            modEco();
          } else if (currentMode == ECO) {
            modStandard();
          }
        }
      }
    }
  }

  // Gestion du bouton rouge
  if (buttonStates & Bouton_rouge_changement) {
    buttonStates &= ~Bouton_rouge_changement;
    if (!(buttonStates & Bouton_rouge_etat)) {
      // Bouton pressé
      redButtonPressTime = millis();
      redButtonPressed = true;
    } else {
      // Bouton relâché
      if (redButtonPressed) {
        unsigned long duration = millis() - redButtonPressTime;
        redButtonPressed = false;
        if (duration >= 3000UL) {
          // Bouton maintenu pendant au moins 3 secondes
          if (currentMode == CONFIGURATION) {
            modStandard();
          } else if (currentMode == MAINTENANCE) {
            if (previousMode == STANDARD) {
              modStandard();
            } else {
              modEco();
            }
          } else {
            modMaintenance();
          }
        }
      }
    }
  }
}

void initLED() {
  leds.init();
  leds.setColorRGB(0, 255, 0, 0);
  delay(500);
  leds.setColorRGB(0, 0, 255, 0);
  delay(500);
  leds.setColorRGB(0, 0, 0, 255);
  delay(500);
  leds.setColorRGB(0, 255, 255, 0);
  delay(500);
  leds.setColorRGB(0, 0, 0, 0);
}

void initButton() {
  pinMode(GREEN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_BUTTON_PIN, INPUT_PULLUP);
}

void updateLEDs() {
  if (errorFlags & RTC_ERROR) {
    leds.setColorRGB(0, 255, 0, 0);
    delay(500);
    leds.setColorRGB(0, 0, 0, 255);
    delay(500);
  } else if (errorFlags & GPS_ERROR) {
    leds.setColorRGB(0, 255, 0, 0);
    delay(500);
    leds.setColorRGB(0, 255, 255, 0);
    delay(500);
  } else if (errorFlags & SD_WRITE_ERROR) {
    leds.setColorRGB(0, 255, 0, 0);
    delay(500);
    leds.setColorRGB(0, 255, 255, 255);
    delay(500);
  } else if (errorFlags & SENSOR_DATA_ERROR) {
    leds.setColorRGB(0, 255, 0, 0);
    delay(500);
    leds.setColorRGB(0, 0, 255, 0);
    delay(1000);
  } else {
    switch (currentMode) {
      case STANDARD:
        leds.setColorRGB(0, 0, 255, 0);
        break;
      case ECO:
        leds.setColorRGB(0, 0, 0, 255);
        break;
      case MAINTENANCE:
        leds.setColorRGB(0, 148, 0, 211);
        break;
      case CONFIGURATION:
        leds.setColorRGB(0, 255, 255, 0);
        break;
    }
  }
}

void etatsSystem() {
  // Fonction vide pour l'instant
}

void modStandard() {
  currentMode = STANDARD;
  // Ne pas réinitialiser LOG_INTERVALL ici pour conserver la valeur personnalisée
}

void modEco() {
  currentMode = ECO;
  // Ne pas réinitialiser LOG_INTERVALL ici pour conserver la valeur personnalisée
}

void modMaintenance() {
  previousMode = currentMode;
  currentMode = MAINTENANCE;
  maintenanceStartTime = millis(); // Démarrer le chronomètre pour le mode maintenance
}

void modConfiguration() {
  currentMode = CONFIGURATION;
  modeStartTime = millis();
}

void recupDonnees() {
  SensorData* newData = (SensorData*)malloc(sizeof(SensorData));
  if (!newData) {
    return;
  }
  newData->next = NULL;

  DateTime now = rtc.now();
  newData->timestamp = now.unixtime();

  if (isTempAirActive) {
    newData->temperature = bme.readTemperature();
  } else {
    newData->temperature = NAN;
  }

  newData->PRESSION = bme.readPressure() / 100.0F;

  if (isHygrActive) {
    newData->humidity = bme.readHumidity();
  } else {
    newData->humidity = NAN;
  }

  if (isLuminActive && LUMIN == 1) {
    newData->luminosity = analogRead(LIGHT_SENSOR_PIN);
  } else {
    newData->luminosity = -1;
  }

  // Ajouter à la liste chainée
  SensorData** ptr = &dataHead;
  while (*ptr) {
    ptr = &((*ptr)->next);
  }
  *ptr = newData;
}

void afficherDonneesConsole() {
  SensorData* current = dataHead;
  while (current) {
    DateTime now = DateTime(current->timestamp);
    Serial.print(F("Date et Heure: "));
    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(F(" | Température: "));

    if (isTempAirActive && !isnan(current->temperature)) {
      Serial.print(current->temperature);
    } else {
      Serial.print("-");
    }

    Serial.print(F(" °C | Pression: "));
    Serial.print(current->PRESSION);
    Serial.print(F(" hPa | Humidité: "));

    if (isHygrActive && !isnan(current->humidity)) {
      Serial.print(current->humidity);
    } else {
      Serial.print(F("-"));
    }

    Serial.print(F(" % | Luminosité: "));

    if (current->luminosity != -1) {
      Serial.println(current->luminosity);
    } else {
      Serial.println(F("-"));
    }

    current = current->next;
  }
}

void writeDataToSD() {
  if (!dataHead || (errorFlags & SD_WRITE_ERROR)) return;

  static int revision = 0;
  DateTime now = rtc.now();
  char filename[13];
  sprintf(filename, "%02d%02d%02d_%d.LOG", now.year() % 100, now.month(), now.day(), revision);
  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    SensorData* current = dataHead;
    while (current) {
      if (dataFile.size() >= FILE_MAX_SIZE) {
        dataFile.close();
        revision++;
        sprintf(filename, "%02d%02d%02d_%d.LOG", now.year() % 100, now.month(), now.day(), revision);
        dataFile = SD.open(filename, FILE_WRITE);
        if (!dataFile) {
          Serial.println(F("Erreur d'écriture sur la carte SD !"));
          errorFlags |= SD_WRITE_ERROR;
          break;
        }
      }
      dataFile.print(current->timestamp);
      dataFile.print(',');

      if (isTempAirActive && !isnan(current->temperature)) {
        dataFile.print(current->temperature);
      } else {
        dataFile.print("-");
      }
      dataFile.print(',');

      dataFile.print(current->PRESSION);
      dataFile.print(',');

      if (isHygrActive && !isnan(current->humidity)) {
        dataFile.print(current->humidity);
      } else {
        dataFile.print("-");
      }
      dataFile.print(',');

      if (current->luminosity != -1) {
        dataFile.println(current->luminosity);
      } else {
        dataFile.println("-");
      }

      SensorData* temp = current;
      current = current->next;
      free(temp);
    }
    dataHead = NULL;
    dataFile.close();
  } else {
    errorFlags |= SD_WRITE_ERROR;
  }
}

void Interface_serie_commands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("HYGR=")) {
      int value = input.substring(5).toInt();
      HYGR = value;
      if (value == 0) {
        isHygrActive = false;
        EEPROM.write(HYGR_ADDR, 0);
        Serial.println(F("Capteur d'hygrométrie désactivé. Valeur: NA"));
      } else if (value == 1) {
        isHygrActive = true;
        EEPROM.write(HYGR_ADDR, value);
        Serial.println(F("Capteur d'hygrométrie activé."));
      }
    } else if (input.startsWith("TEMP_AIR=")) {
      int value = input.substring(9).toInt();
      TEMP_AIR = value;
      if (value == 0) {
        isTempAirActive = false;
        Serial.println(F("Capteur de température désactivé. Valeur: NA"));
      } else if (value == 1) {
        isTempAirActive = true;
        Serial.println(F("Capteur de température activé."));
      }
    } else if (input.startsWith("LUMIN=")) {
      int value = input.substring(6).toInt();
      LUMIN = value;
      if (value == 0) {
        isLuminActive = false;
        Serial.println(F("Capteur de luminosité désactivé."));
      } else if (value == 1) {
        isLuminActive = true;
        Serial.println(F("Capteur de luminosité activé."));
      }
    } else if (input.startsWith("LUMIN_LOW=")) {
      LUMIN_LOW = input.substring(10).toInt();
      Serial.print(F("LUMIN_LOW mis à jour: "));
      Serial.println(LUMIN_LOW);
    } else if (input.startsWith("LUMIN_HIGH=")) {
      LUMIN_HIGH = input.substring(11).toInt();
      Serial.print(F("LUMIN_HIGH mis à jour: "));
      Serial.println(LUMIN_HIGH);
    } else if (input.startsWith("MIN_TEMP_AIR=")) {
      MIN_TEMP_AIR = input.substring(13).toInt();
      Serial.print(F("MIN_TEMP_AIR mis à jour: "));
      Serial.println(MIN_TEMP_AIR);
    } else if (input.startsWith("MAX_TEMP_AIR=")) {
      MAX_TEMP_AIR = input.substring(13).toInt();
      Serial.print(F("MAX_TEMP_AIR mis à jour: "));
      Serial.println(MAX_TEMP_AIR);
    } else if (input.startsWith("HYGR_MINT=")) {
      HYGR_MINT = input.substring(10).toInt();
      Serial.print(F("HYGR_MINT mis à jour: "));
      Serial.println(HYGR_MINT);
    } else if (input.startsWith("HYGR_MAXT=")) {
      HYGR_MAXT = input.substring(10).toInt();
      Serial.print(F("HYGR_MAXT mis à jour: "));
      Serial.println(HYGR_MAXT);
    } else if (input.startsWith("PRESSION_MIN=")) {
      PRESSION_MIN = input.substring(13).toInt();
      Serial.print(F("PRESSION_MIN mis à jour: "));
      Serial.println(PRESSION_MIN);
    } else if (input.startsWith("PRESSION_MAX=")) {
      PRESSION_MAX = input.substring(13).toInt();
      Serial.print(F("PRESSION_MAX mis à jour: "));
      Serial.println(PRESSION_MAX);
    } else if (input.startsWith("LOG_INTERVALL=")) {
      unsigned long value = input.substring(14).toInt() * 60000UL; // Conversion en millisecondes
      LOG_INTERVALL = value;
      EEPROM.put(LOG_INTERVALL_ADDR, LOG_INTERVALL); // Sauvegarder dans l'EEPROM
      Serial.print(F("LOG_INTERVALL mis à jour: "));
      Serial.print(LOG_INTERVALL / 60000UL);
      Serial.println(F(" minutes"));
    } else if (input.startsWith("FILE_MAX_SIZE=")) {
      FILE_MAX_SIZE = input.substring(14).toInt();
      Serial.print(F("FILE_MAX_SIZE mis à jour: "));
      Serial.print(FILE_MAX_SIZE);
      Serial.println(F(" octets"));
    } else if (input.startsWith("TIMEOUT=")) {
      TIMEOUT = input.substring(8).toInt() * 1000UL; // Conversion en millisecondes
      Serial.print(F("TIMEOUT mis à jour: "));
      Serial.print(TIMEOUT / 1000);
      Serial.println(F(" secondes"));
    } else if (input == "RESET") {
      resetParameters();
      Serial.println(F("Paramètres réinitialisés aux valeurs par défaut"));
    } else if (input == "VERSION") {
      Serial.println(F("Version 1.0 - Lot 12345"));
    } else {
      Serial.println(F("Commande inconnue"));
    }
  }
}

void resetParameters() {
  LOG_INTERVALL = 5000UL;
  EEPROM.put(LOG_INTERVALL_ADDR, LOG_INTERVALL); // Réinitialiser dans l'EEPROM
  FILE_MAX_SIZE = 4096UL;
  TIMEOUT = 30000UL;
  LUMIN = 1;
  LUMIN_LOW = 255;
  LUMIN_HIGH = 768;
  TEMP_AIR = 1;
  MIN_TEMP_AIR = -10;
  MAX_TEMP_AIR = 60;
  HYGR = 1;
  HYGR_MINT = 0;
  HYGR_MAXT = 50;
  PRESSION = 1;
  PRESSION_MIN = 850;
  PRESSION_MAX = 1080;
  isLuminActive = true;
  isHygrActive = true;
  isTempAirActive = true;
}
