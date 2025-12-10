//################################################
//# Dependencies:
//# - esp32 by Espressif Systems
//# - DHT sensor library for ESPx by beegee_tokyo
//################################################
// ======================================================
// To flash the board, in vs code: idf.py -p COM7 erase-flash
// config to flash in vs code: UART esp32c6 (VIA ESP-PROG-2)
// ======================================================
// To flash the board, with Arduino IDE:
// Tool-> Erase all flash before sketch upload: Enable
// ======================================================
// Board used as Router (used with AC supply):
// - Partition: Zigbee ZCZR 4Mb with spiffs
// - Zigbee mode: Zigbee ZCZR
// ======================================================
// Board used as End device (used with battery):
// - Partition: Zigbee 4Mb with spiffs
// - Zigbee mode: Zigbee ED
// ======================================================
// - Monitor baud: 115200
// ======================================================

#define DHT_READ_INTERVAL 10 * 1000  // 10 secondes
#define REPORT_INTERVAL 5 * 60 * 1000   // 5 minutes
#define TEMP_DELTA 0.2           // Changement minimum de température
#define HUM_DELTA 1.0            // Changement minimum d'humidité

// Conversion constant: difference in seconds between 
// the Unix era (1970) and the Zigbee era (2000)
#define ZIGBEE_EPOCH_OFFSET 946684800  // secondes between 1970 and 2000

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

#define TEMP_SENSOR_ENDPOINT_NUMBER 10

#define ANALOG_DEVICE_ENDPOINT_NUMBER 1

uint8_t button = BOOT_PIN;  //BOOT button (not RESET button!!!)

// Optional Time cluster variables
struct tm timeinfo;
struct tm *localTime;
int32_t timezone;

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

ZigbeeAnalog zbAnalogDevice = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER);

ZigbeeAnalog zbAnalogDevice_2 = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 1);

#include <DHTesp.h>

#define DHTPIN 23
#define DHTTYPE DHTesp::DHT22 
DHTesp dht;

float temperature=NULL;
float humidity=NULL;

/************************ Temp sensor *****************************/
void fixZigbeeTime(struct tm &timeinfo) {
  // Convert tm structure to timestamp
  time_t zigbeeTime = mktime(&timeinfo);
  
  // Add the offset to get the correct Unix time
  time_t correctTime = zigbeeTime + ZIGBEE_EPOCH_OFFSET;
  
  // Convert back to tm structure with the corrected date
  localtime_r(&correctTime, &timeinfo);
}

bool tempChanged(float current, float previous) {
  return fabs(current - previous) > TEMP_DELTA;
}

bool humChanged(float current, float previous) {
  return fabs(current - previous) >= HUM_DELTA;
}

static void temp_sensor_read(void *arg) {

  const TickType_t xDelay = pdMS_TO_TICKS(100);

  // Variables for non-blocking timers
  static unsigned long lastDHTRead = 0;

  static int recepion_failed_nb = 0;

  for (;;) {
    unsigned long currentMillis = millis();

    
    // if (recepion_failed_nb > 10) {
    //   Serial.println("Trop d'échecs, redémarrage...");
    //   vTaskDelay(pdMS_TO_TICKS(1000));
    //   ESP.restart();
    // }

    if (currentMillis - lastDHTRead >= DHT_READ_INTERVAL) {

      // Attendre l'intervalle minimum requis
      delay(dht.getMinimumSamplingPeriod());
      
      // Read data from sensor
      TempAndHumidity data = dht.getTempAndHumidity();
  
      // check the reading status
      if (dht.getStatus() != 0) {
        recepion_failed_nb++;
        Serial.print("Erreur DHT: ");
        Serial.println(dht.getStatusString());

        zbAnalogDevice_2.setAnalogInput(recepion_failed_nb);
        zbAnalogDevice_2.reportAnalogInput();

        vTaskDelay(pdMS_TO_TICKS(2000)); // The DHT22 returns a maximum of one measurement every 2 seconds
      }else{
        
        if (isnan(data.humidity) || isnan(data.temperature)) {
          recepion_failed_nb++;
          Serial.printf("Échec réception: %d\n", recepion_failed_nb);

          zbAnalogDevice_2.setAnalogInput(500000 + recepion_failed_nb);
          zbAnalogDevice_2.reportAnalogInput();

          vTaskDelay(pdMS_TO_TICKS(2000)); // The DHT22 returns a maximum of one measurement every 2 seconds
        } else {
          lastDHTRead = currentMillis;

          if(recepion_failed_nb!=0){
            recepion_failed_nb = 0;

            zbAnalogDevice_2.setAnalogInput(recepion_failed_nb);
            zbAnalogDevice_2.reportAnalogInput();
          }

          temperature=data.temperature;
          humidity=data.humidity;
        }
      }
    }
    vTaskDelay(xDelay); // Prefer vTaskDelay to delay() + yield()
  }
}

static void temp_zigbee_send(void *arg) {

  const TickType_t xDelay = pdMS_TO_TICKS(1000);

  static unsigned long previousExec = 0;
  static float previousT = 0;
  static float previousH = 0;

  for (;;) {
    unsigned long currentMillis = millis();
    if ( !isnan(temperature) && (
          ((currentMillis - previousExec) > REPORT_INTERVAL)
          || tempChanged(temperature, previousT)
          || humChanged(humidity, previousH)
        )
      ) {

      // Update temperature & humidity values in Temperature sensor EP
      zbTempSensor.setTemperature(temperature);
      zbTempSensor.setHumidity(humidity);
      previousExec = currentMillis;
      previousT = temperature;
      previousH = humidity;

      zbTempSensor.report();  // reports temperature and humidity values (if humidity sensor is not added, only temperature is reported)

      timeinfo = zbTempSensor.getTime();
      timezone = zbTempSensor.getTimezone();
      fixZigbeeTime(timeinfo);
      Serial.print(&timeinfo, "%Y/%m/%d %H:%M:%S Z");
      Serial.print(timezone/3600);
      Serial.printf(" - Humidité: %.1f%%  Température: %.1f°C\n", humidity, temperature);
    }
    vTaskDelay(xDelay);  // Prefer vTaskDelay to delay() + yield()
  }
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 5000);  // Wait Serial max 5s
  Serial.println("\n=== ESP32-C6 Zigbee Temp/Hum startup ===");

  // Init sensor with DHT22 model (compatibility with AM2302B)
  dht.setup(DHTPIN, DHTTYPE);
  
  Serial.println("Sensor initialised!");
  Serial.print("Delay between 2 reads: ");
  Serial.print(dht.getMinimumSamplingPeriod());
  Serial.println("ms");

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("Rémi Lebret", "Esp32-C6-01-ZigbeeTemp&HumSensor");

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

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);

  // Zigbee Sensor to track restart (crash, ...)
  zbAnalogDevice.addAnalogInput();
  zbAnalogDevice.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
  zbAnalogDevice.setAnalogInputDescription("pid");
  zbAnalogDevice.setAnalogInputResolution(1);

  Zigbee.addEndpoint(&zbAnalogDevice);

  // Zigbee Sensor to track DHT error
  zbAnalogDevice_2.addAnalogInput();
  zbAnalogDevice_2.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
  zbAnalogDevice_2.setAnalogInputDescription("error");
  zbAnalogDevice_2.setAnalogInputResolution(1);

  Zigbee.addEndpoint(&zbAnalogDevice_2);

  Serial.println("Starting Zigbee...");
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(ZIGBEE_ROLE)) {
    Serial.println("ERREUR: Zigbee start failed!");
    Serial.println("Reboot in 5s...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Zigbee started with success!");
  Serial.print("Connecting to network");

  uint32_t timeout = millis() + 30000;  // Timeout 30s
  while (!Zigbee.connected()) {
    if (millis() > timeout) {
      Serial.println("\nTimeout network connection!");
      Serial.println("Reboot in 5s...");
      delay(5000);
      ESP.restart();
    }
    Serial.print(".");
    delay(100);
  }
  Serial.println("\Connected to Zigbee network!");

  // Lecture et affichage de l'heure
  timeinfo = zbTempSensor.getTime();
  timezone = zbTempSensor.getTimezone();

  fixZigbeeTime(timeinfo);

  Serial.println("Heure (UTC):");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  time_t local = mktime(&timeinfo) + timezone;
  localTime = localtime(&local);
  Serial.println("Heure (local):");
  Serial.println(localTime, "%A, %B %d %Y %H:%M:%S");

  // Start Temperature sensor reading task
  BaseType_t taskReadCreated = xTaskCreate(
    temp_sensor_read,
    "temp_sensor_read",
    4096,  // Stack augmenté pour sécurité
    NULL,
    5,  // Priorité moyenne
    NULL);

  if (taskReadCreated != pdPASS) {
    Serial.println("ERREUR: Échec création tâche lecture capteur!");
    ESP.restart();
  }

  // Start Zigbee sending task
  BaseType_t taskSendCreated = xTaskCreate(
    temp_zigbee_send,
    "temp_zigbee_send",
    4096,  // Stack augmenté pour sécurité
    NULL,
    5,  // Priorité moyenne
    NULL);

  if (taskSendCreated != pdPASS) {
    Serial.println("ERREUR: Échec création tâche envoi zigbee!");
    ESP.restart();
  }

  // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (temp change in 0,1 °C)
  // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of temperature change
  //zbTempSensor.setReporting(0, 60, 0.1);


  Serial.println("=== Initialization complete ===\n");
}

void loop() {

  static int startStatus = 0;
  static unsigned long lastButtonCheck = 0;
  static unsigned long buttonPressStart = 0;
  static bool buttonPressed = false;
  static const unsigned long DEBOUNCE_DELAY = 50;  // Délai anti-rebond
  static const unsigned long FACTORY_RESET_DELAY = 3000;
  static const unsigned long SHORT_PRESS_MIN = 100;

  unsigned long currentMillis = millis();

  //Envoi d'un ping (bascule false->true puis true->false)
  //pour notifier d'un (re)démarrage de l'esp32
  if(startStatus==0){
    Serial.println("Envoi du statut started = true");
  
    float randNumber = fabs(esp_random()); 
    Serial.println(randNumber);
    zbAnalogDevice.setAnalogInput(randNumber);
    zbAnalogDevice.reportAnalogInput();

    zbAnalogDevice_2.setAnalogInput(0);
    zbAnalogDevice_2.reportAnalogInput();

    startStatus=1;
  }

  // BOOT button management (non-blocking)
  if (currentMillis - lastButtonCheck >= DEBOUNCE_DELAY) {  // Check every 50ms
    lastButtonCheck = currentMillis;

    bool buttonState = (digitalRead(button) == LOW);

    if (buttonState && !buttonPressed) {
      // Start of button press
      buttonPressed = true;
      buttonPressStart = currentMillis;
    } else if (!buttonState && buttonPressed) {
      // Button released
      unsigned long pressDuration = currentMillis - buttonPressStart;

      if (pressDuration >= SHORT_PRESS_MIN && pressDuration < FACTORY_RESET_DELAY) {
        // short press : manual report
        Serial.println("Rapport manuel T° et Humidité");
        zbTempSensor.report();
      }
      
      // long press: reset and reboot
      if (pressDuration >= FACTORY_RESET_DELAY) {
        Serial.println("Factory reset Zigbee et redémarrage dans 1s...");
        delay(1000);
        Zigbee.factoryReset();
      }
      buttonPressed = false;
    }
  }

  delay(50);
}
