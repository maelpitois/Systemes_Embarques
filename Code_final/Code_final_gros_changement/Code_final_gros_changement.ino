#include <Wire.h>
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


enum Mode { STANDARD, ECO, MAINTENANCE, CONFIGURATION };
Mode currentMode = STANDARD;
Mode previousMode = STANDARD;

uint8_t errorFlags = 0;
#define RTC_ERROR          0x01
#define GPS_ERROR          0x02
#define SD_CARD_FULL       0x04 
#define SD_WRITE_ERROR     0x08
#define SENSOR_DATA_ERROR  0x10
//manque donnée incohérentes


Adafruit_BME280 bme;
RTC_DS1307 rtc;
ChainableLED leds(LED_DATA_PIN, LED_CLOCK_PIN, 1);


bool eeprom_bool ;
uint8_t eeprom_uint8 ;
int8_t eeprom_int8 ;
unsigned long eeprom_UL ;
unsigned long eeprom_UL2 ;
uint16_t eeprom_uint16 ;


#define LOG_INTERVALL_ADDR 50 // Adresse EEPROM pour LOG_INTERVALL
#define TIMEOUT_ADDR 56
#define FILE_MAX_SIZE_ADDR 62
#define LUMIN_ADDR 12
#define LUMIN_LOW_ADDR 13
#define LUMIN_HIGH_ADDR 15
#define TEMP_AIR_ADDR 17
#define MIN_TEMP_AIR_ADDR 18
#define MAX_TEMP_AIR_ADDR 19
#define HYGR_ADDR 20 // Adresse dans l'EEPROM pour la valeur de HYGR
#define HYGR_MINT_ADDR 21
#define HYGR_MAXT_ADDR 22
#define PRESSION_ADDR 23
#define PRESSION_MIN_ADDR 24
#define PRESSION_MAX_ADDR 26
#define lastDataAcquisitionTime_ADDR 68
#define modeStartTime_ADDR 74
#define maintenanceStartTime_ADDR 80
#define isHygrActive_ADDR 28 
#define isTempAirActive_ADDR 29
#define isLuminActive_ADDR 30
#define isPressActive_ADDR 31

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
  
  if (digitalRead(RED_BUTTON_PIN) == LOW) {
    modConfiguration();
  } else {
    modStandard();
  }

  
  

  // Configurer les interruptions pour les boutons sur changement d'état
  attachInterrupt(digitalPinToInterrupt(GREEN_BUTTON_PIN), greenButtonISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RED_BUTTON_PIN), redButtonISR, CHANGE);
}

void loop() {
  unsigned long currentTime = millis();

  Gestion_bouton();

  EEPROM.get(LOG_INTERVALL_ADDR,eeprom_UL);
  EEPROM.get(lastDataAcquisitionTime_ADDR,eeprom_UL2);
  if ((currentMode == STANDARD || currentMode == ECO) && (currentTime - eeprom_UL2 >= eeprom_UL)) {
    EEPROM.put(lastDataAcquisitionTime_ADDR,currentTime);
    recupDonnees();
    writeDataToSD();
  }
  EEPROM.get(lastDataAcquisitionTime_ADDR,eeprom_UL2);
  if (currentMode == MAINTENANCE && (currentTime - eeprom_UL2 >= 5000UL)) {
    EEPROM.put(lastDataAcquisitionTime_ADDR,currentTime);
    recupDonnees();
    afficherDonneesConsole();
    writeDataToSD();
  }

  

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
            modEco();//mode Eco
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


// Fonction pour vérifier si la carte SD est pleine
void Carte_pleine() {
  // Tente de créer et d'écrire dans un fichier temporaire
  File fichierTemporaire = SD.open("temporaire.txt", FILE_WRITE);

  // Vérifie si la création ou l'écriture du fichier échoue
  if (!fichierTemporaire || fichierTemporaire.write('A') == 0) {
    errorFlags |= SD_CARD_FULL;
  }

  // Ferme et supprime le fichier temporaire si nécessaire
  if (fichierTemporaire) fichierTemporaire.close();
  SD.remove("temporaire.txt");
}



void gestion_erreur(){
   // test initialisation des capteur sd et rtc si il sont ouvert ou non
  if (!SD.begin(SD_CS_PIN)) {
    errorFlags |= SD_WRITE_ERROR;
  }

  if (!bme.begin(0x76)) {
    errorFlags |= SENSOR_DATA_ERROR;
  }

  if (!rtc.begin()) {
    errorFlags |= RTC_ERROR;
  }
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
    delay(1000);
  } else if (errorFlags & SENSOR_DATA_ERROR) {
    leds.setColorRGB(0, 255, 0, 0);
    delay(500);
    leds.setColorRGB(0, 0, 255, 0);
    delay(500);
  } else if (errorFlags & SD_CARD_FULL) {
    leds.setColorRGB(0, 255, 0, 0);
    delay(500);
    leds.setColorRGB(0, 255, 255, 255);
    delay(500);
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

void modStandard() {
  currentMode = STANDARD;
  gestion_erreur();
  updateLEDs();
  // Ne pas réinitialiser LOG_INTERVALL ici pour conserver la valeur personnalisée
}

void modEco() {
  currentMode = ECO;
  gestion_erreur();
  updateLEDs();
  // Ne pas réinitialiser LOG_INTERVALL ici pour conserver la valeur personnalisée
}

void modMaintenance() {
  previousMode = currentMode;
  currentMode = MAINTENANCE;
  gestion_erreur();
  updateLEDs();
  EEPROM.put(maintenanceStartTime_ADDR,millis()); // Démarrer le chronomètre pour le mode maintenance
}

void modConfiguration() {
  currentMode = CONFIGURATION;
  gestion_erreur();
  updateLEDs();
  EEPROM.put(modeStartTime_ADDR,millis());
}

void recupDonnees() {
  SensorData* newData = (SensorData*)malloc(sizeof(SensorData));
  if (!newData) {
    return;
  }
  newData->next = NULL;

  DateTime now = rtc.now();
  newData->timestamp = now.unixtime();
  EEPROM.get(isTempAirActive_ADDR,eeprom_bool);
  if (eeprom_bool) {
    newData->temperature = bme.readTemperature();
  } else {
    newData->temperature = NAN;
  }

  newData->PRESSION = bme.readPressure() / 100.0F;
  EEPROM.get(isHygrActive_ADDR,eeprom_bool);
  if (eeprom_bool) {
    newData->humidity = bme.readHumidity();
  } else {
    newData->humidity = NAN;
  }
  EEPROM.get(LUMIN_ADDR,eeprom_uint8);
  EEPROM.get(isLuminActive_ADDR,eeprom_bool);
  if (eeprom_bool && eeprom_uint8 == 1) {
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
    DateTime now = rtc.now();
    Serial.print(F("Date et Heure: "));
    Serial.print(now.day());
    Serial.print('/');
    Serial.print(now.month());
    Serial.print('/');
    Serial.print(now.year());
    Serial.print(' ');
    Serial.print(now.hour());
    Serial.print(':');
    Serial.print(now.minute());
    Serial.print(':');
    Serial.print(now.second());
    Serial.print(F(" | Température: "));

    EEPROM.get(isTempAirActive_ADDR,eeprom_bool);  

    if (eeprom_bool && !isnan(current->temperature)) {
      Serial.print(current->temperature);
    } else {
      Serial.print("NA");
    }

    Serial.print(F(" °C | Pression: "));
    Serial.print(current->PRESSION);
    Serial.print(F(" hPa | Humidité: "));

    EEPROM.get(isHygrActive_ADDR,eeprom_bool);

    if (eeprom_bool && !isnan(current->humidity)) {
      Serial.print(current->humidity);
    } else {
      Serial.print(F("NA"));
    }

    Serial.print(F(" % | Luminosité: "));

    if (current->luminosity != -1) {
      Serial.println(current->luminosity);
    } else {
      Serial.println(F("NA"));
    }

    current = current->next;
  }
}

void writeDataToSD() {
  if (!dataHead || (errorFlags & SD_WRITE_ERROR)) return;
  Carte_pleine();
  updateLEDs();

  static uint8_t revision = 0;
  DateTime now = rtc.now();
  char filename[15];
  createFilename(filename, now, revision);
  File dataFile = SD.open(filename, FILE_WRITE);

  // Lecture des valeurs de l'EEPROM avant la boucle
  unsigned long eeprom_UL;
  EEPROM.get(FILE_MAX_SIZE_ADDR, eeprom_UL);

  bool isTempAirActive;
  EEPROM.get(isTempAirActive_ADDR, isTempAirActive);

  bool isHygrActive;
  EEPROM.get(isHygrActive_ADDR, isHygrActive);

  if (dataFile) {
    SensorData* current = dataHead;
    while (current) {
      if (dataFile.size() >= eeprom_UL) {
        dataFile.close();
        revision++;
        createFilename(filename, now, revision);
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
        dataFile.print("NA");
      }
      dataFile.print(',');

      dataFile.print(current->PRESSION);
      dataFile.print(',');

      if (isHygrActive && !isnan(current->humidity)) {
        dataFile.print(current->humidity);
      } else {
        dataFile.print("NA");
      }
      dataFile.print(',');

      if (current->luminosity != -1) {
        dataFile.println(current->luminosity);
      } else {
        dataFile.println("NA");
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

void createFilename(char* filename, DateTime now, uint8_t revision) {
  uint8_t year = now.year() % 100;
  uint8_t month = now.month();
  uint8_t day = now.day();
  uint8_t idx = 0;

  // Année
  filename[idx++] = '0' + (year / 10);
  filename[idx++] = '0' + (year % 10);
  // Mois
  filename[idx++] = '0' + (month / 10);
  filename[idx++] = '0' + (month % 10);
  // Jour
  filename[idx++] = '0' + (day / 10);
  filename[idx++] = '0' + (day % 10);
  // Underscore
  filename[idx++] = '_';
  // Révision (jusqu'à 2 chiffres)
  if (revision >= 10) {
    filename[idx++] = '0' + (revision / 10);
  }
  filename[idx++] = '0' + (revision % 10);
  // Extension
  filename[idx++] = '.';
  filename[idx++] = 'L';
  filename[idx++] = 'O';
  filename[idx++] = 'G';
  // Terminaison de la chaîne
  filename[idx] = '\0';
}


void updateEEPROM(int address, int value, const __FlashStringHelper* message) {
  EEPROM.put(address, value);
  Serial.print(message);
  Serial.println(value);
}

void activateSensor(bool active, int sensorAddr, int activeAddr, const __FlashStringHelper* activateMessage, const __FlashStringHelper* deactivateMessage) {
  EEPROM.put(sensorAddr, active ? 1 : 0);
  EEPROM.put(activeAddr, active);
  Serial.println(active ? activateMessage : deactivateMessage);
}

bool isValidRange(int value, int min, int max) {
  return (value >= min && value <= max);
}

void Interface_serie_commands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int valeur1,valeur2,valeur3 ;
    if (input.startsWith("HYGR=")) {
      activateSensor(input.substring(5).toInt() == 1, HYGR_ADDR, isHygrActive_ADDR,
                     F("Capteur d'hygrométrie activé."), F("Capteur d'hygrométrie désactivé. Valeur: NA"));
    } else if (input.startsWith("TEMP_AIR=")) {
      activateSensor(input.substring(9).toInt() == 1, TEMP_AIR_ADDR, isTempAirActive_ADDR,
                     F("Capteur de température activé."), F("Capteur de température désactivé. Valeur: NA"));
    } else if (input.startsWith("LUMIN=")) {
      activateSensor(input.substring(6).toInt() == 1, LUMIN_ADDR, isLuminActive_ADDR,
                     F("Capteur de luminosité activé."), F("Capteur de luminosité désactivé."));
    } else if (input.startsWith("PRESSURE=")) {
      activateSensor(input.substring(9).toInt() == 1, PRESSION_ADDR, isPressActive_ADDR,
                     F("Capteur de pression activé."), F("Capteur de pression désactivé. Valeur: NA"));
    } else if (input.startsWith("LUMIN_LOW=")) {
      valeur1 = input.substring(10).toInt();
      if (isValidRange(valeur1, 0, 1023)) updateEEPROM(LUMIN_LOW_ADDR, valeur1, F("LUMIN_LOW mis à jour: "));
      else Serial.println(F("Erreur"));
    } else if (input.startsWith("LUMIN_HIGH=")) {
      valeur2 = input.substring(11).toInt();
      if (isValidRange(valeur2, 0, 1023)) updateEEPROM(LUMIN_HIGH_ADDR, valeur2, F("LUMIN_HIGH mis à jour: "));
      else Serial.println(F("Erreur"));
    } else if (input.startsWith("MIN_TEMP_AIR=") || input.startsWith("MAX_TEMP_AIR=")) {
      valeur2 = input.substring(13).toInt();
      valeur3 = input.startsWith("MIN_TEMP_AIR=") ? MIN_TEMP_AIR_ADDR : MAX_TEMP_AIR_ADDR;
      if (isValidRange(valeur2, -40, 85)) updateEEPROM(valeur3, valeur2, input.startsWith("MIN") ? F("MIN_TEMP_AIR mis à jour: ") : F("MAX_TEMP_AIR mis à jour: "));
      else Serial.println(F("Erreur"));
    } else if (input.startsWith("HYGR_MINT=") || input.startsWith("HYGR_MAXT=")) {
      valeur2 = input.substring(10).toInt();
      valeur3 = input.startsWith("HYGR_MINT=") ? HYGR_MINT_ADDR : HYGR_MAXT_ADDR;
      if (isValidRange(valeur2, -40, 85)) updateEEPROM(valeur3, valeur2, input.startsWith("HYGR_MINT=") ? F("HYGR_MINT mis à jour: ") : F("HYGR_MAXT mis à jour: "));
      else Serial.println(F("Erreur"));
    } else if (input.startsWith("PRESSURE_MIN=") || input.startsWith("PRESSURE_MAX=")) {
      valeur2 = input.substring(13).toInt();
      valeur3 = input.startsWith("PRESSURE_MIN=") ? PRESSION_MIN_ADDR : PRESSION_MAX_ADDR;
      if (isValidRange(valeur2, 300, 1000)) updateEEPROM(valeur3, valeur2, input.startsWith("PRESSURE_MIN=") ? F("PRESSION_MIN mis à jour: ") : F("PRESSION_MAX mis à jour: "));
      else Serial.println(F("Erreur"));
    } else if (input.startsWith("CLOCK=")) {
      valeur1 = input.substring(6, 8).toInt();
      valeur2 = input.substring(9, 11).toInt();
      valeur3 = input.substring(12).toInt();
      if (isValidRange(valeur1, 0, 23) && isValidRange(valeur2, 0, 59) && isValidRange(valeur3, 0, 59)) {
        DateTime temp = rtc.now();
        rtc.adjust(DateTime(temp.year(), temp.month(), temp.day(), valeur1, valeur2, valeur3));
        Serial.println(F("Heure mise à jour"));
      } else Serial.println(F("Erreur"));
    } else if (input.startsWith("DATE=")) {
      valeur1 = input.substring(5, 7).toInt();
      valeur2 = input.substring(8, 10).toInt();
      valeur3 = input.substring(11, 15).toInt();
      if (isValidRange(valeur1, 1, 12) && isValidRange(valeur2, 1, 31) && isValidRange(valeur3, 2000, 2099)) {
        DateTime temp = rtc.now();
        rtc.adjust(DateTime(valeur3, valeur1, valeur2, temp.hour(), temp.minute(), temp.second()));
        Serial.println(F("Date mise à jour"));
      } else Serial.println(F("Erreur"));
    } else if (input.startsWith("DAY=")) {
      String day = input.substring(4, 6);
      uint8_t jour_semaine = -1;
      if (day.equals("MON")) jour_semaine = 1;
      else if (day.equals("TUE")) jour_semaine = 2;
      else if (day.equals("WED")) jour_semaine = 3;
      else if (day.equals("THU")) jour_semaine = 4;
      else if (day.equals("FRI")) jour_semaine = 5;
      else if (day.equals("SAT")) jour_semaine = 6;
      else if (day.equals("SUN")) jour_semaine = 0;

      if (jour_semaine != -1) {
        Wire.beginTransmission(0x68);
        Wire.write(0x03);
        Wire.write(jour_semaine);
        Wire.endTransmission();
        Serial.println(F("Jour mis à jour"));
      } else Serial.println(F("Jour inexistant"));
    } else if (input.startsWith("LOG_INTERVALL=")) {
      valeur2 = input.substring(14).toInt() * 60000UL;
      updateEEPROM(LOG_INTERVALL_ADDR, valeur2, F("LOG_INTERVALL mis à jour: "));
      Serial.println(F(" minutes"));
    } else if (input.startsWith("FILE_MAX_SIZE=")) {
      updateEEPROM(FILE_MAX_SIZE_ADDR, input.substring(14).toInt(), F("FILE_MAX_SIZE mis à jour: "));
    } else if (input.startsWith("TIMEOUT=")) {
      updateEEPROM(TIMEOUT_ADDR, input.substring(8).toInt() * 1000UL, F("TIMEOUT mis à jour: "));
      Serial.println(F(" secondes"));
    } else if (input == "RESET") {
      resetParameters();
      Serial.println(F("Paramètres réinitialisés"));
    } else if (input == "VERSION") {
      Serial.println(F("Version 1.0 - Lot 12345"));
    } else {
      Serial.println(F("Commande inconnue"));
    }
  }
}




void resetParameters() { // Réinitialiser dans l'EEPROM touts les paramètres
  EEPROM.put(LOG_INTERVALL_ADDR,5000);
  EEPROM.put(TIMEOUT_ADDR, 30000);
  EEPROM.put(FILE_MAX_SIZE_ADDR, 4096);
  EEPROM.put(LUMIN_ADDR, 1);
  EEPROM.put(LUMIN_LOW_ADDR, 255);
  EEPROM.put(LUMIN_HIGH_ADDR,768);
  EEPROM.put(TEMP_AIR_ADDR,1);
  EEPROM.put(MIN_TEMP_AIR_ADDR, -10);
  EEPROM.put(MAX_TEMP_AIR_ADDR,60);
  EEPROM.put(HYGR_ADDR,1);
  EEPROM.put(HYGR_MINT_ADDR,0);
  EEPROM.put(HYGR_MAXT_ADDR,50);
  EEPROM.put(PRESSION_ADDR, 1);
  EEPROM.put(PRESSION_MIN_ADDR,850);
  EEPROM.put(PRESSION_MAX_ADDR,1080);
  EEPROM.put(lastDataAcquisitionTime_ADDR,0);
  EEPROM.put(modeStartTime_ADDR, 0);
  EEPROM.put(maintenanceStartTime_ADDR,0);
  EEPROM.put(isHygrActive_ADDR,true);
  EEPROM.put(isTempAirActive_ADDR,true);
  EEPROM.put(isLuminActive_ADDR,true);
  EEPROM.put(isPressActive_ADDR,true);
}
