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


bool eeprom_bool ;
uint8_t eeprom_uint8 ;
int8_t eeprom_int8 ;
unsigned long eeprom_UL ;
unsigned long eeprom_UL2 ;
uint16_t eeprom_uint16 ;


// Adresse des donnée au dessus calculer en fonction des octets requis pile
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

  // A chaque allummage l'eeprom se reset 
  
  
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

  updateLEDs();

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
  EEPROM.put(maintenanceStartTime_ADDR,millis()); // Démarrer le chronomètre pour le mode maintenance
}

void modConfiguration() {
  currentMode = CONFIGURATION;
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

  static int revision = 0;
  DateTime now = rtc.now();
  char filename[13];
  sprintf(filename, "%02d%02d%02d_%d.LOG", now.year() % 100, now.month(), now.day(), revision);
  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    SensorData* current = dataHead;
    while (current) {
      EEPROM.get(FILE_MAX_SIZE_ADDR,eeprom_UL);
      if (dataFile.size() >= eeprom_UL) {
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

      EEPROM.get(isTempAirActive_ADDR,eeprom_bool);

      if (eeprom_bool && !isnan(current->temperature)) {
        dataFile.print(current->temperature);
      } else {
        dataFile.print("NA");
      }
      dataFile.print(',');

      dataFile.print(current->PRESSION);
      dataFile.print(',');

      EEPROM.get(isHygrActive_ADDR,eeprom_bool);

      if (eeprom_bool && !isnan(current->humidity)) {
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

void Interface_serie_commands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("HYGR=")) {
      if (input.substring(5).toInt() == 0) {
        EEPROM.put(HYGR_ADDR,input.substring(5).toInt());
        EEPROM.put(isHygrActive_ADDR,false);
        Serial.println(F("Capteur d'hygrométrie désactivé. Valeur: NA"));
      } else if (input.substring(5).toInt() == 1) {
        EEPROM.put(HYGR_ADDR,input.substring(5).toInt());
        EEPROM.put(isHygrActive_ADDR,true);
        Serial.println(F("Capteur d'hygrométrie activé."));
      }
    } else if (input.startsWith("TEMP_AIR=")) {
      if (input.substring(5).toInt() == 0) {
        EEPROM.put(TEMP_AIR_ADDR,input.substring(9).toInt());
        EEPROM.put(isTempAirActive_ADDR,false);
        Serial.println(F("Capteur de température désactivé. Valeur: NA"));
      } else if (input.substring(5).toInt() == 1) {
        EEPROM.put(TEMP_AIR_ADDR,input.substring(9).toInt());
        EEPROM.put(isTempAirActive_ADDR,true);
        Serial.println(F("Capteur de température activé."));
      }
    } else if (input.startsWith("LUMIN=")) {
      if (input.substring(5).toInt() == 0) {
        EEPROM.put(LUMIN_ADDR,input.substring(6).toInt());
        EEPROM.put(isLuminActive_ADDR,false);
        Serial.println(F("Capteur de luminosité désactivé."));
      } else if (input.substring(5).toInt() == 1) {
        EEPROM.put(LUMIN_ADDR,input.substring(6).toInt());
        EEPROM.put(isLuminActive_ADDR,true);
        Serial.println(F("Capteur de luminosité activé."));
      }
    }  else if (input.startsWith("PRESSURE=")) {
      if (input.substring(5).toInt() == 0) {
        EEPROM.put(PRESSION_ADDR,input.substring(9).toInt());
        EEPROM.put(isPressActive_ADDR,false);
        Serial.println(F("Capteur de pression désactivé. Valeur: NA"));
      } else if (input.substring(5).toInt() == 1) {
        EEPROM.put(PRESSION_ADDR,input.substring(9).toInt());
        EEPROM.put(isPressActive_ADDR,true);
        Serial.println(F("Capteur de pression activé."));
      }
    } 
    
    else if (input.startsWith("LUMIN_LOW=")) {
      if (input.substring(5).toInt() >= 0 && input.substring(5).toInt() <= 1023){
        EEPROM.put(LUMIN_LOW_ADDR,input.substring(10).toInt());
        Serial.print(F("LUMIN_LOW mis à jour: "));
        EEPROM.get(LUMIN_LOW_ADDR,eeprom_uint16);
        Serial.println(eeprom_uint16);
      }
      else {
        Serial.println(F("La valeur n'est pas dans l'intervalle(0-1023)"));
      }
      
    } 
    
    else if (input.startsWith("LUMIN_HIGH=")) { 
      if (input.substring(5).toInt() >= 0 && input.substring(5).toInt() <= 1023){
        EEPROM.put(LUMIN_HIGH_ADDR,input.substring(11).toInt());
        EEPROM.get(LUMIN_HIGH_ADDR,eeprom_uint16);
        Serial.print(F("LUMIN_HIGH mis à jour: "));
        Serial.println(eeprom_uint16);
      }
      else {
        Serial.println(F("La valeur n'est pas dans l'intervalle(0-1023)"));
      }
    } 
    
    else if (input.startsWith("MIN_TEMP_AIR=")) {
      if (input.substring(5).toInt() >= -40 && input.substring(5).toInt() <= 85){
        EEPROM.put(MIN_TEMP_AIR_ADDR,input.substring(13).toInt());
        Serial.print(F("MIN_TEMP_AIR mis à jour: "));
        EEPROM.get(MIN_TEMP_AIR_ADDR,eeprom_int8);
        Serial.println(eeprom_int8);
      }
      else {
        Serial.println(F("La valeur n'est pas dans l'intervalle(-40-85)"));
      }
    } 
    
    else if (input.startsWith("MAX_TEMP_AIR=")) {
      if (input.substring(5).toInt() >= -40 && input.substring(5).toInt() <= 85){
        EEPROM.put(MAX_TEMP_AIR_ADDR,input.substring(13).toInt());
        Serial.print(F("MAX_TEMP_AIR mis à jour: "));
        EEPROM.get(MAX_TEMP_AIR_ADDR,eeprom_int8);
        Serial.println(eeprom_int8);
      }
      else {
        Serial.println(F("La valeur n'est pas dans l'intervalle(-40-85)"));
      }
    } 
    
    else if (input.startsWith("HYGR_MINT=")) {
      if (input.substring(5).toInt() >= -40 && input.substring(5).toInt() <= 85){
        EEPROM.put(HYGR_MINT_ADDR,input.substring(10).toInt());
        Serial.print(F("HYGR_MINT mis à jour: "));
        EEPROM.get(HYGR_MINT_ADDR,eeprom_uint8);
        Serial.println(eeprom_uint8);
      }
      else {
        Serial.println(F("La valeur n'est pas dans l'intervalle(-40-85)"));
      }
    } 
    
    else if (input.startsWith("HYGR_MAXT=")) {
      if (input.substring(5).toInt() >= -40 && input.substring(5).toInt() <= 85){
        EEPROM.put(HYGR_MAXT_ADDR,input.substring(10).toInt());
        Serial.print(F("HYGR_MAXT mis à jour: "));
        EEPROM.get(HYGR_MAXT_ADDR,eeprom_uint8);
        Serial.println(eeprom_uint8);
      }
      else {
        Serial.println(F("La valeur n'est pas dans l'intervalle(-40-85)"));
      }
    } 
    
    else if (input.startsWith("PRESSURE_MIN=")) {
      if (input.substring(5).toInt() >= 300 && input.substring(5).toInt() <= 1000){
        EEPROM.put(PRESSION_MIN_ADDR,input.substring(13).toInt());
        Serial.print(F("PRESSION_MIN mis à jour: "));
        EEPROM.get(PRESSION_MIN_ADDR,eeprom_uint16);
        Serial.println(eeprom_uint16);
      }
      else {
        Serial.println(F("La valeur n'est pas dans l'intervalle(300-1000)"));
      }
    } 
    
    else if (input.startsWith("PRESSURE_MAX=")) {
      if (input.substring(5).toInt() >= 300 && input.substring(5).toInt() <= 1000){
        EEPROM.put(PRESSION_MAX_ADDR,input.substring(13).toInt());
        Serial.print(F("PRESSION_MAX mis à jour: "));
        EEPROM.get(PRESSION_MAX_ADDR,eeprom_uint16);
        Serial.println(eeprom_uint16);
      }
      else {
        Serial.println(F("La valeur n'est pas dans l'intervalle(300-1000)"));
      }
    } 
    // temps fonction difference 
    else if (input.startsWith("CLOCK=")) {
      if (input.substring(6,7).toInt()<=23 && input.substring(6,7).toInt()>=0 && input.substring(9,10).toInt()<=59 && input.substring(9,10).toInt()>=0 && input.substring(12).toInt()<=59 && input.substring(12).toInt()>=59){
      DateTime temp = rtc.now();
      rtc.adjust(DateTime(temp.year(),temp.month(), temp.day(),input.substring(6,7).toInt() ,input.substring(9,10).toInt() , input.substring(12).toInt()));
      Serial.println(F("Heure mis a jours"));
      }
      else {
        Serial.println(F("Heure inexistante ou mise en forme incorect ex : 12,59,30(heure,minutes,seconde)"));
      }
    }
    else if (input.startsWith("DATE=")) {
      if (input.substring(5,6).toInt()<=12 && input.substring(5,6).toInt()>=1 && input.substring(8,9).toInt()<=31 && input.substring(8,9).toInt()>=1 && input.substring(11,14).toInt()<=2099 && input.substring(11,14).toInt()>=2000){
      DateTime temp = rtc.now();
      rtc.adjust(DateTime(input.substring(11,14).toInt(),input.substring(5,6).toInt(),input.substring(8,9).toInt(),temp.hour(),temp.minute(),temp.second()));
      Serial.println(F("Date mis a jours"));
      }
      else {
        Serial.println(F("Date inexistante ou mise en forme incorect ex : 12,25,2004(mois,jour,année)"));
      }

    }
    else if (input.startsWith("DAY=")) {
      String day = input.substring(4,6); 
      int jour_semaine = -1 ;
      if (day.equals("MON")) {
        jour_semaine = 1;}
      else if (day.equals("TUE") ) {
        jour_semaine = 2;}
      else if (day.equals("WED")) {
        jour_semaine = 3;}
      else if (day.equals("THU")) {
        jour_semaine = 4;}
      else if (day.equals("FRI")) {
        jour_semaine = 5;}
      else if (day.equals("SAT")) {
        jour_semaine = 6;}
      else if (day.equals("SUN")) {
        jour_semaine = 0;}

      if(jour_semaine != -1){
          Wire.beginTransmission(0x68);  // Adresse I2C du DS1307
          Wire.write(0x03);  // Registre de jour de la semaine
          Wire.write(jour_semaine);   // Ecrit le jour (0=Dimanche, 1=Lundi, ..., 6=Samedi)
          Wire.endTransmission();




        Serial.println(F("Jour mis à jour"));


      }
      else {
          Serial.println(F("Jours inexistante"));

      }

    }
    else if (input.startsWith("LOG_INTERVALL=")) {// Conversion en millisecondes
      EEPROM.put(LOG_INTERVALL_ADDR,input.substring(14).toInt() * 60000UL); // Sauvegarder dans l'EEPROM
      Serial.print(F("LOG_INTERVALL mis à jour: "));
      EEPROM.get(LOG_INTERVALL_ADDR,eeprom_UL);
      Serial.print(eeprom_UL / 60000UL);
      Serial.println(F(" minutes"));
    } 
    
    else if (input.startsWith("FILE_MAX_SIZE=")) {
      EEPROM.put(FILE_MAX_SIZE_ADDR,input.substring(14).toInt());
      Serial.print(F("FILE_MAX_SIZE mis à jour: "));
      EEPROM.get(FILE_MAX_SIZE_ADDR,eeprom_UL);
      Serial.print(eeprom_UL);
      Serial.println(F(" octets"));
    } 
    
    else if (input.startsWith("TIMEOUT=")) { // Conversion en millisecondes
      EEPROM.put(TIMEOUT_ADDR,input.substring(8).toInt() * 1000UL);
      Serial.print(F("TIMEOUT mis à jour: "));
      EEPROM.get(TIMEOUT_ADDR,eeprom_UL);
      Serial.print(eeprom_UL / 1000);
      Serial.println(F(" secondes"));
    } 
    
    else if (input == "RESET") {
      resetParameters();
      Serial.println(F("Paramètres réinitialisés aux valeurs par défaut"));
    } 
    else if (input == "VERSION") {
      Serial.println(F("Version 1.0 - Lot 12345"));
    }
    else {
      Serial.println(F("Commande inconnue"));
    }
  }
}

void resetParameters() { // Réinitialiser dans l'EEPROM touts les paramètres
  EEPROM.put(LOG_INTERVALL_ADDR, (unsigned long)5000UL);
  EEPROM.put(TIMEOUT_ADDR, (unsigned long)30000UL);
  EEPROM.put(FILE_MAX_SIZE_ADDR, (unsigned long)4096UL);
  EEPROM.put(LUMIN_ADDR, (uint8_t)1);
  EEPROM.put(LUMIN_LOW_ADDR, (uint16_t)255);
  EEPROM.put(LUMIN_HIGH_ADDR, (uint16_t)768);
  EEPROM.put(TEMP_AIR_ADDR, (uint8_t)1);
  EEPROM.put(MIN_TEMP_AIR_ADDR, (int8_t)-10);
  EEPROM.put(MAX_TEMP_AIR_ADDR, (int8_t)60);
  EEPROM.put(HYGR_ADDR, (uint8_t)1);
  EEPROM.put(HYGR_MINT_ADDR, (uint8_t)0);
  EEPROM.put(HYGR_MAXT_ADDR, (uint8_t)50);
  EEPROM.put(PRESSION_ADDR, (uint8_t)1);
  EEPROM.put(PRESSION_MIN_ADDR, (uint16_t)850);
  EEPROM.put(PRESSION_MAX_ADDR, (uint16_t)1080);
  EEPROM.put(lastDataAcquisitionTime_ADDR, (unsigned long)0);
  EEPROM.put(modeStartTime_ADDR, (unsigned long)0);
  EEPROM.put(maintenanceStartTime_ADDR,(unsigned long)0);
  EEPROM.put(isHygrActive_ADDR, (bool)true);
  EEPROM.put(isTempAirActive_ADDR, (bool)true);
  EEPROM.put(isLuminActive_ADDR, (bool)true);
  EEPROM.put(isPressActive_ADDR, (bool)true);
}
