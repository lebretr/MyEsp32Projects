//╔═══════════════════════════════════════════════════════════╗
//║ Dependencies:                                             ║
//║ * Boards Manager:                                         ║
//║   - esp32 by Espressif Systems v3.3.5                     ║
//║ * Libraries:                                              ║
//║   - DHT sensor library for ESPx by beegee_tokyo v1.19     ║
//╠═══════════════════════════════════════════════════════════╣
//║ To flash reset the board, in vs code:                     ║
//║     idf.py -p COM7 erase-flash                            ║
//║ config to flash in vs code:                               ║
//║     UART esp32c6 (VIA ESP-PROG-2)                         ║
//║     UART esp32h2 (VIA ESP-PROG-2)                         ║
//╠═──────────────────────────────────────────────────────────╣
//║ To flash reset the board, with Arduino IDE:               ║
//║ Tool-> Erase all flash before sketch upload: Enable       ║
//╠═──────────────────────────────────────────────────────────╣
//║ Board used as Router (used with AC supply):               ║
//║ - Partition: Zigbee ZCZR 4Mb with spiffs                  ║
//║ - Zigbee mode: Zigbee ZCZR                                ║
//╠═──────────────────────────────────────────────────────────╣
//║ Board used as End device (used with battery):             ║
//║ - Partition: Zigbee 4Mb with spiffs                       ║
//║ - Zigbee mode: Zigbee ED                                  ║
//╠═──────────────────────────────────────────────────────────╣
//║ - Monitor baud: 115200                                    ║
//╚═══════════════════════════════════════════════════════════╝

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "Parameters.h"

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

#define TEMP_SENSOR_ENDPOINT_NUMBER 1

#define ANALOG_DEVICE_ENDPOINT_NUMBER 101

uint8_t button = BOOT_PIN;  //BOOT button (not RESET button!!!)

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

ZigbeeAnalog zbAnalogDevicePid = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER);

ZigbeeAnalog zbAnalogDeviceError = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 1);

#include <DHTesp.h>
#define DHTTYPE DHTesp::DHT22 
DHTesp dht_1;

float temperature_1=NULL;
float humidity_1=NULL;

static const char *TAG = "Main";

/************************ Temp sensor *****************************/

float myfAbs(float x) {
    uint32_t i = *((uint32_t*)&x);
    i &= 0x7FFFFFFF;  // Masque le bit de signe
    return *((float*)&i);
}

bool isTempChanged(float current, float previous) {
  return myfAbs(current - previous) > TEMP_SENSIBILITY;
}

bool isHumChanged(float current, float previous) {
  return myfAbs(current - previous) >= HUMIDITY_SENSIBILITY;
}

static void dht_reading(void *arg) {

  static const TickType_t xDelay = pdMS_TO_TICKS(2000);

  // Variables for non-blocking timers
  static unsigned long lastDHTRead = 0;

  static int reception_failed_nb = 0;

  for (;;) {
    unsigned long currentMillis = millis();

    
    // if (reception_failed_nb > 10) {
    //   ESP_LOGE(TAG, "Trop d'échecs, redémarrage...");
    //   vTaskDelay(pdMS_TO_TICKS(1000));
    //   ESP.restart();
    // }

    if (currentMillis - lastDHTRead >= DHT_READ_INTERVAL) {

      // Attendre l'intervalle minimum requis
      delay(dht_1.getMinimumSamplingPeriod());
      
      // Read data from sensor
      TempAndHumidity data = dht_1.getTempAndHumidity();
  
      // check the reading status
      if (dht_1.getStatus() != 0) {
        reception_failed_nb++;
        ESP_LOGE(TAG, "Erreur DHT: %s", dht_1.getStatusString());

        zbAnalogDeviceError.setAnalogInput(101);
        zbAnalogDeviceError.reportAnalogInput();

        vTaskDelay(pdMS_TO_TICKS(2000)); // The DHT22 returns a maximum of one measurement every 2 seconds
      }else{
        
        if (isnan(data.humidity) || isnan(data.temperature)) {
          reception_failed_nb++;
          ESP_LOGE(TAG, "Échec réception: %d\n", reception_failed_nb);

          zbAnalogDeviceError.setAnalogInput(500001);
          zbAnalogDeviceError.reportAnalogInput();

          vTaskDelay(pdMS_TO_TICKS(2000)); // The DHT22 returns a maximum of one measurement every 2 seconds
        } else {
          lastDHTRead = currentMillis;

          if(reception_failed_nb!=0){
            reception_failed_nb = 0;

            zbAnalogDeviceError.setAnalogInput(0);
            zbAnalogDeviceError.reportAnalogInput();
          }

          temperature_1=data.temperature;
          humidity_1=data.humidity;
        }
      }
    }
    vTaskDelay(xDelay); // Prefer vTaskDelay to delay() + yield()
  }
}

static void tempHum_zigbee_reporting(void *arg) {

  static const TickType_t xDelay = pdMS_TO_TICKS(2000);

  static unsigned long previousExec = 0;
  static float previousT = 0;
  static float previousH = 0;

  for (;;) {
    unsigned long currentMillis = millis();
    if ( !isnan(temperature_1) && (
          ((currentMillis - previousExec) > REPORT_INTERVAL)
          || isTempChanged(temperature_1, previousT)
          || isHumChanged(humidity_1, previousH)
        )
      ) {

      // Update temperature & humidity values in Temperature sensor EP
      zbTempSensor.setTemperature(temperature_1);
      zbTempSensor.setHumidity(humidity_1);
      previousExec = currentMillis;
      previousT = temperature_1;
      previousH = humidity_1;

      zbTempSensor.report();  // reports temperature and humidity values (if humidity sensor is not added, only temperature is reported)

      ESP_LOGI(TAG, "Humidité: %.1f%%  Température: %.1f°C", (double)humidity_1, (double)temperature_1);
    }
    vTaskDelay(xDelay);  // Prefer vTaskDelay to delay() + yield()
  }
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(115200);

  // esp_log_level_set("*", ESP_LOG_VERBOSE);
  // esp_log_level_set(TAG, ESP_LOG_VERBOSE);

  while (!Serial && millis() < 5000);  // Wait Serial max 5s
  ESP_LOGI(TAG, " ");
  ESP_LOGI(TAG, " ");
  ESP_LOGI(TAG, "=== ESP32 Zigbee Temp/Hum startup ===");

  // Init sensor with DHT22 model (compatibility with AM2302B)
  pinMode(DHTPIN_1, INPUT);
  dht_1.setup(DHTPIN_1, DHTTYPE);

  ESP_LOGI(TAG, "Sensor initialised!");
  ESP_LOGI(TAG, "Delay between 2 reads: %d ms", dht_1.getMinimumSamplingPeriod());

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel(AUTHOR, MODEL);

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

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);

  // Zigbee Sensor to track restart (crash, ...)
  zbAnalogDevicePid.addAnalogInput();
  zbAnalogDevicePid.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
  zbAnalogDevicePid.setAnalogInputDescription("pid");
  zbAnalogDevicePid.setAnalogInputResolution(1);

  Zigbee.addEndpoint(&zbAnalogDevicePid);

  // Zigbee Sensor to track DHT error
  zbAnalogDeviceError.addAnalogInput();
  zbAnalogDeviceError.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
  zbAnalogDeviceError.setAnalogInputDescription("error");
  zbAnalogDeviceError.setAnalogInputResolution(1);

  Zigbee.addEndpoint(&zbAnalogDeviceError);

  ESP_LOGI(TAG, "Starting Zigbee...");
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(ZIGBEE_ROLE)) {
    ESP_LOGE(TAG, "ERREUR: Zigbee start failed!");
    ESP_LOGE(TAG, "Reboot in 5s...");
    delay(5000);
    ESP.restart();
  }

  ESP_LOGI(TAG, "Zigbee started with success!");
  ESP_LOGI(TAG, "Connecting to network");

  uint32_t timeout = millis() + 30000;  // Timeout 30s
  while (!Zigbee.connected()) {
    if (millis() > timeout) {
      ESP_LOGE(TAG, "Timeout network connection!");
      ESP_LOGE(TAG, "Reboot in 5s...");
      delay(5000);
      ESP.restart();
    }
    delay(100);
  }
  ESP_LOGI(TAG, "Connected to Zigbee network!");

  // Start Temperature sensor reading task
  BaseType_t taskReadCreated = xTaskCreate(
    dht_reading,
    "dht_reading",
    4096,  // Stack augmenté pour sécurité
    NULL,
    5,  // Priorité moyenne
    NULL);

  if (taskReadCreated != pdPASS) {
    ESP_LOGE(TAG, "ERROR: Échec création tâche lecture capteur!");
    ESP.restart();
  }

  // Start Zigbee reporting task
  BaseType_t taskReportCreated = xTaskCreate(
    tempHum_zigbee_reporting,
    "tempHum_zigbee_reporting",
    4096,  // Stack augmenté pour sécurité
    NULL,
    5,  // Priorité moyenne
    NULL);

  if (taskReportCreated != pdPASS) {
    ESP_LOGE(TAG, "ERROR: Échec création tâche envoi zigbee!");
    ESP.restart();
  }

  // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (temp change in 0,1 °C)
  // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of temperature change
  //zbTempSensor.setReporting(0, 60, 0.1);

  ESP_LOGI(TAG, "=== Initialization complete ===\n");
}

void loop() {

  static int startStatus = 0;
  static unsigned long lastButtonCheck = 0;
  static unsigned long buttonPressStart = 0;
  static bool buttonPressed = false;
  static const unsigned long DEBOUNCE_DELAY = 100;  // Délai anti-rebond
  static const unsigned long FACTORY_RESET_DELAY = 3000; //=3s
  static const unsigned long SHORT_PRESS_MIN = 2*DEBOUNCE_DELAY; //=200ms
  static const TickType_t xDelay = pdMS_TO_TICKS(2*DEBOUNCE_DELAY); //=200ms

  unsigned long currentMillis = millis();

  //report a random number
  //to follow ESP32 reboot
  if(startStatus==0){
    float randNumber = myfAbs(esp_random()); 
    ESP_LOGI(TAG, "Pid: %f", (double)randNumber);
    zbAnalogDevicePid.setAnalogInput(randNumber);
    zbAnalogDevicePid.reportAnalogInput();

    zbAnalogDeviceError.setAnalogInput(0);
    zbAnalogDeviceError.reportAnalogInput();

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
        ESP_LOGI(TAG, "Manual report T°c et Humidity");
        zbTempSensor.report();
      }
      
      // long press: reset and reboot
      if (pressDuration >= FACTORY_RESET_DELAY) {
        ESP_LOGI(TAG, "Factory reset Zigbee et reboot in 1s...");
        delay(1000);
        Zigbee.factoryReset();
      }
      buttonPressed = false;
    }
  }

  delay(xDelay); // old value: 50
}
