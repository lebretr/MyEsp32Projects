//################################################
//# installer la librairie "DHT Sensor" d’Adafruit
//################################################
// ======================================================
// To flash the board, in vs code: idf.py -p COM7 erase-flash
// config to flash in vs code: UART esp32c6 (VIA ESP-PROG-2)
// ======================================================
// as Router (used with AC supply):
// - Partition: Zigbee ZCZR 4Mb with spiffs
// - Zigbee mode: Zigbee ZCZR
// ======================================================
// as End device (used with battery):
// - Partition: Zigbee 4Mb with spiffs
// - Zigbee mode: Zigbee ED
// ======================================================
// - Monitor baud: 115200
// ======================================================

#define DHT_READ_INTERVAL 10000  // 10 secondes
#define REPORT_INTERVAL 300000   // 5 minutes
#define TEMP_DELTA 0.2           // Changement minimum de température
#define HUM_DELTA 1.0            // Changement minimum d'humidité
#define MAX_DHT_FAILURES 6       // Nombre d'échecs avant restart

// Constante de conversion : différence en secondes entre 
// l'époque Unix (1970) et l'époque Zigbee (2000)
#define ZIGBEE_EPOCH_OFFSET 946684800  // secondes entre 1970 et 2000

//#ifndef ZIGBEE_MODE_ED
//#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
//#endif
#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee coordinator/router mode is not selected in Tools->Zigbee mode"
#endif

#define ZIGBEE_ROLE ZIGBEE_ROUTER
// ZIGBEE_COORDINATOR: Network coordinator, forms and manages the network
// ZIGBEE_ROUTER: Network router, connects to existing network and extends network range
//                and routes messages (if device is mains powered, always use this role)
// ZIGBEE_END_DEVICE: End device, connects to existing network (typically battery-powered which can sleep)


#include "Zigbee.h"
/* Zigbee temperature sensor configuration */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10
uint8_t button = BOOT_PIN;  //BOOT button (not RESET!!!)

// Optional Time cluster variables
struct tm timeinfo;
struct tm *localTime;
int32_t timezone;

/* Zigbee OTA configuration */
// #define OTA_UPGRADE_RUNNING_FILE_VERSION    0x01010100  // Increment this value when the running image is updated
// #define OTA_UPGRADE_DOWNLOADED_FILE_VERSION 0x01010101  // Increment this value when the downloaded image is updated
//#define OTA_UPGRADE_HW_VERSION              0x0101      // The hardware version, this can be used to differentiate between different hardware versions

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);


#include "DHT.h"

#define DHTPIN 23
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

//#include "esp_task_wdt.h"

/************************ Temp sensor *****************************/
void fixZigbeeTime(struct tm &timeinfo) {
  // Convertir la structure tm en timestamp
  time_t zigbeeTime = mktime(&timeinfo);
  
  // Ajouter le décalage pour obtenir le temps Unix correct
  time_t correctTime = zigbeeTime + ZIGBEE_EPOCH_OFFSET;
  
  // Reconvertir en structure tm avec la date corrigée
  localtime_r(&correctTime, &timeinfo);
}

bool tempChanged(float current, float previous) {
  return fabs(current - previous) > TEMP_DELTA;
}

bool humChanged(float current, float previous) {
  return fabs(current - previous) >= HUM_DELTA;
}

static void temp_sensor_value_update(void *arg) {

  const TickType_t xDelay = pdMS_TO_TICKS(100);

  // Variables pour timers non-bloquants
  static unsigned long lastDHTRead = 0;
  static unsigned long previousExec = 0;
  static float previousT = -40;
  static float previousH = 0;

  for (;;) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastDHTRead >= 10000) {
      lastDHTRead = currentMillis;

      // Le DHT22 renvoie au maximum une mesure toute les 2s
      float h = dht.readHumidity();
      // Lis le taux d'humidite en %
      float t = dht.readTemperature();
      // Lis la température en degré celsius

      zbTempSensor.setTemperature(t);
      zbTempSensor.setHumidity(h);

      static int nb_echec_reception = 0;
      if (isnan(h) || isnan(t)) {
        nb_echec_reception++;
        Serial.printf("Échec réception: %d\n", nb_echec_reception);
        if (nb_echec_reception > 6) {
          Serial.println("Trop d'échecs, redémarrage...");
          vTaskDelay(pdMS_TO_TICKS(1000));
          ESP.restart();
        }
      } else {
        nb_echec_reception = 0;  // Réinitialiser en cas de succès
                                 // envoie toutes les 5min ou si la T° change ou si l'humidité change d'au moins 1%
        if (
          ((currentMillis - previousExec) > 5 * 60 * 1000)
          || tempChanged(t, previousT)
          || humChanged(h, previousH)) {
          // Update temperature & humidity values in Temperature sensor EP
          // zbTempSensor.setTemperature(t);
          // zbTempSensor.setHumidity(h);
          previousExec = currentMillis;
          previousT = t;
          previousH = h;

          zbTempSensor.report();  // reports temperature and humidity values (if humidity sensor is not added, only temperature is reported)

          timeinfo = zbTempSensor.getTime();
          timezone = zbTempSensor.getTimezone();
          fixZigbeeTime(timeinfo);
          Serial.print(&timeinfo, "%Y/%m/%d %H:%M:%S Z");
          Serial.print(timezone/3600);
          Serial.printf(" - Humidité: %.1f%%  Température: %.1f°C\n", h, t);
        }
      }
    }
    vTaskDelay(xDelay);  // Préférer vTaskDelay à delay() + yield()
  }
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 5000);  // Attendre le Serial max 5s
  Serial.println("\n=== Démarrage ESP32-C6 Zigbee Temp/Hum ===");

  dht.begin();

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("Rémi Lebret", "C6-ZigbeeTemp&HumSensor");

  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_MAINS);  //ZB_POWER_SOURCE_BATTERY or ZB_POWER_SOURCE_MAINS

  // Set binding settings depending on the role
  // if (ZIGBEE_ROLE == ZIGBEE_COORDINATOR) {
  //   zbTempSensor.allowMultipleBinding(true);  // To allow binding multiple lights to the switch
  // } else {
  zbTempSensor.setManualBinding(true);  //Set manual binding to true, so binding is done on Home Assistant side
  // }
  
  // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
  zbTempSensor.setMinMaxValue(-40, 80);

  // Optional: Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
  zbTempSensor.setTolerance(0.1);

  // Add humidity cluster to the temperature sensor device with min, max and tolerance values
  zbTempSensor.addHumiditySensor(0, 100, 0.1);

  // Optional: Time cluster configuration (default params, as this device will revieve time from coordinator)
  zbTempSensor.addTimeCluster();

  // Add OTA client to the light bulb
  //zbTempSensor.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION, OTA_UPGRADE_DOWNLOADED_FILE_VERSION, OTA_UPGRADE_HW_VERSION);

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);

  Serial.println("Starting Zigbee...");
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(ZIGBEE_ROLE)) {
    Serial.println("ERREUR: Échec démarrage Zigbee!");
    Serial.println("Redémarrage dans 5s...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Zigbee démarré avec succès!");
  Serial.print("Connexion au réseau");

  uint32_t timeout = millis() + 30000;  // Timeout 30s
  while (!Zigbee.connected()) {
    if (millis() > timeout) {
      Serial.println("\nTimeout connexion réseau!");
      Serial.println("Redémarrage dans 5s...");
      delay(5000);
      ESP.restart();
    }
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnecté au réseau Zigbee!");

  // Lecture et affichage de l'heure
  timeinfo = zbTempSensor.getTime();
  timezone = zbTempSensor.getTimezone();

  fixZigbeeTime(timeinfo);

  Serial.println("Heure UTC:");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  time_t local = mktime(&timeinfo) + timezone;
  localTime = localtime(&local);
  Serial.println("Heure locale:");
  Serial.println(localTime, "%A, %B %d %Y %H:%M:%S");

  // Start Temperature sensor reading task
  // Créer la tâche avec priorité et stack appropriés
  BaseType_t taskCreated = xTaskCreate(
    temp_sensor_value_update,
    "temp_sensor",
    4096,  // Stack augmenté pour sécurité
    NULL,
    5,  // Priorité moyenne
    NULL);

  if (taskCreated != pdPASS) {
    Serial.println("ERREUR: Échec création tâche capteur!");
    ESP.restart();
  }

  // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (temp change in 0,1 °C)
  // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of temperature change
  //zbTempSensor.setReporting(0, 60, 0.1);

  // Start Zigbee OTA client query, first request is within a minute and the next requests are sent every hour automatically
  //zbTempSensor.requestOTAUpdate();

  Serial.println("=== Initialisation terminée ===\n");
}

void loop() {

  // Variables pour timers non-bloquants
  static unsigned long lastButtonCheck = 0;
  static unsigned long buttonPressStart = 0;
  static bool buttonPressed = false;
  static const unsigned long DEBOUNCE_DELAY = 50;  // Délai anti-rebond
  static const unsigned long FACTORY_RESET_DELAY = 3000;
  static const unsigned long SHORT_PRESS_MIN = 100;

  unsigned long currentMillis = millis();

  // Gestion du bouton BOOT (non-bloquant)
  if (currentMillis - lastButtonCheck >= DEBOUNCE_DELAY) {  // Check toutes les 50ms
    lastButtonCheck = currentMillis;

    bool buttonState = (digitalRead(button) == LOW);

    if (buttonState && !buttonPressed) {
      // Début de l'appui
      buttonPressed = true;
      buttonPressStart = currentMillis;
    } else if (!buttonState && buttonPressed) {
      // Bouton relâché
      unsigned long pressDuration = currentMillis - buttonPressStart;

      if (pressDuration >= SHORT_PRESS_MIN && pressDuration < FACTORY_RESET_DELAY) {
        // Appui court : rapport manuel
        Serial.println("Rapport manuel T° et Humidité");
        zbTempSensor.report();
      }
      buttonPressed = false;
    } else if (buttonState && buttonPressed) {
      // Bouton maintenu - vérifier si factory reset
      if ((currentMillis - buttonPressStart) >= FACTORY_RESET_DELAY) {
        Serial.println("Factory reset Zigbee et redémarrage dans 1s...");
        delay(1000);
        Zigbee.factoryReset();
      }
    }
  }

  delay(50);  // Réduit à 50ms pour meilleure réactivité
}