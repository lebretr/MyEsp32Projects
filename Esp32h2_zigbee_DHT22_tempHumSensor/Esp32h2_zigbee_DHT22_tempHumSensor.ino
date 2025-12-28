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
static const char *TAG = "Main";

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

ZigbeeAnalog zbAnalogDevicePid = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER);

ZigbeeAnalog zbAnalogDeviceError = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 1);

#include <DHTesp.h>
#define DHTTYPE DHTesp::DHT22 

struct zbTempSensor_S {
  ZigbeeTempSensor* zbTempSensor;
} zbTempSensor_R;

zbTempSensor_S zbTempSensor_V[NUMBER_OF_DHT] ;

struct dhtTH_S {
  DHTesp dht;
  int DhtReadInterval = DHT_READ_INTERVAL;
  float temperature;
  float humidity;
  unsigned long lastDHTRead = 0;
  bool receptionFailed = false;
} dhtTH_R;

dhtTH_S dhtTH_V[NUMBER_OF_DHT] ;

struct previousTH_S {
  float previousT = 0;
  float previousH = 0;
  unsigned long previousExec = 0;
} previousTH_R;

static int NumberOfDht=NUMBER_OF_DHT;


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

  static const TickType_t xDelay = pdMS_TO_TICKS(2000); // Attendre l'intervalle minimum requis
  // static const TickType_t xDelay = pdMS_TO_TICKS(dhtTH_V[0].dht.getMinimumSamplingPeriod());
    
  static unsigned long ERROR_CODE=0;
  unsigned long PREVIOUS_ERROR_CODE=0;

  for (;;) {
    unsigned long currentMillis = millis();
    
    for (int i=0; i<NumberOfDht; i++) {
      if (currentMillis - dhtTH_V[i].lastDHTRead >= dhtTH_V[i].DhtReadInterval) {
        dhtTH_V[i].lastDHTRead = currentMillis;

        // Read data from sensor
        TempAndHumidity data = dhtTH_V[i].dht.getTempAndHumidity();
    
        // check the reading status
        if (dhtTH_V[i].dht.getStatus() != 0) {
          ESP_LOGE(TAG, "Erreur DHT n°%d: %s", i, dhtTH_V[i].dht.getStatusString());

          if(dhtTH_V[i].receptionFailed==false){
            dhtTH_V[i].receptionFailed=true;
            ERROR_CODE=ERROR_CODE+(1* pow(10 , i));
            dhtTH_V[i].DhtReadInterval=0;
          }
          if(dhtTH_V[i].DhtReadInterval <= 300 * 1000){
            dhtTH_V[i].DhtReadInterval=dhtTH_V[i].DhtReadInterval+5000;
          }

        } else {
          if(dhtTH_V[i].receptionFailed==true){
            dhtTH_V[i].receptionFailed = false;
            ERROR_CODE=ERROR_CODE-(1* pow(10 , i));

            zbAnalogDeviceError.setAnalogInput(ERROR_CODE);
            zbAnalogDeviceError.reportAnalogInput();
          }

          dhtTH_V[i].temperature=data.temperature;
          dhtTH_V[i].humidity=data.humidity;
          dhtTH_V[i].DhtReadInterval=DHT_READ_INTERVAL;
        }
      }
    }

    if(ERROR_CODE!=PREVIOUS_ERROR_CODE){
      vTaskDelay(pdMS_TO_TICKS(5000));
      PREVIOUS_ERROR_CODE=ERROR_CODE;
      zbAnalogDeviceError.setAnalogInput(ERROR_CODE);
      zbAnalogDeviceError.reportAnalogInput();
    }
    vTaskDelay(xDelay); // Prefer vTaskDelay to delay() + yield()
  }
}

static void tempHum_zigbee_reporting(void *arg) {

  static const TickType_t xDelay = pdMS_TO_TICKS(2000);
  
  previousTH_S previousTH_V[NumberOfDht] ;

  for (;;) {
    unsigned long currentMillis = millis();
    for (int i=0; i<NumberOfDht; i++) {
      if ( !isnan(dhtTH_V[i].temperature) && (
            ((currentMillis - previousTH_V[i].previousExec) > REPORT_INTERVAL)
            || isTempChanged(dhtTH_V[i].temperature, previousTH_V[i].previousT)
            || isHumChanged(dhtTH_V[i].humidity, previousTH_V[i].previousH)
          )
        ) {

        // Update temperature & humidity values in Temperature sensor EP
        zbTempSensor_V[i].zbTempSensor->setTemperature(dhtTH_V[i].temperature);
        zbTempSensor_V[i].zbTempSensor->setHumidity(dhtTH_V[i].humidity);
        previousTH_V[i].previousExec = currentMillis;
        previousTH_V[i].previousT = dhtTH_V[i].temperature;
        previousTH_V[i].previousH = dhtTH_V[i].humidity;

        zbTempSensor_V[i].zbTempSensor->report();  // reports temperature and humidity values (if humidity sensor is not added, only temperature is reported)
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "DHT n°%d - Humidité: %.1f%%  Température: %.1f°C", i, (double)dhtTH_V[i].humidity, (double)dhtTH_V[i].temperature);
      }
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
  for (int i=0; i<NumberOfDht; i++) {
    if(i==0){
      pinMode(DHTPIN_0, INPUT);
      dhtTH_V[i].dht.setup(DHTPIN_0, DHTTYPE);
    }
    if(i==1){
      pinMode(DHTPIN_1, INPUT);
      dhtTH_V[i].dht.setup(DHTPIN_1, DHTTYPE);
    }
    if(i==2){
      pinMode(DHTPIN_2, INPUT);
      dhtTH_V[i].dht.setup(DHTPIN_2, DHTTYPE);
    }
    if(i==3){
      pinMode(DHTPIN_3, INPUT);
      dhtTH_V[i].dht.setup(DHTPIN_3, DHTTYPE);
    }
    if(i==4){
      pinMode(DHTPIN_4, INPUT);
      dhtTH_V[i].dht.setup(DHTPIN_4, DHTTYPE);
    }
    if(i==5){
      pinMode(DHTPIN_5, INPUT);
      dhtTH_V[i].dht.setup(DHTPIN_5, DHTTYPE);
    }

    ESP_LOGI(TAG, "Sensor n°%d initialised!", i);
  }

  ESP_LOGI(TAG, "Delay between 2 reads: %d ms", dhtTH_V[0].dht.getMinimumSamplingPeriod());

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  for (int i=0; i<NumberOfDht; i++) {
    zbTempSensor_V[i].zbTempSensor = new ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER + i);
  }
  // Optional: set Zigbee device name and model
  zbTempSensor_V[0].zbTempSensor->setManufacturerAndModel(AUTHOR, MODEL);

  zbTempSensor_V[0].zbTempSensor->setPowerSource(ZB_POWER_SOURCE_MAINS);  //ZB_POWER_SOURCE_BATTERY or ZB_POWER_SOURCE_MAINS

  // Set binding settings depending on the role
  // if (ZIGBEE_ROLE == ZIGBEE_COORDINATOR) {
  //   zbTempSensor->allowMultipleBinding(true);  // To allow binding multiple lights to the switch
  // } else {
  zbTempSensor_V[0].zbTempSensor->setManualBinding(true);  //Set manual binding to true, so binding is done on Home Assistant side
  // }

  for (int i=0; i<NumberOfDht; i++) {
    
    // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
    zbTempSensor_V[i].zbTempSensor->setMinMaxValue(-40, 80);

    // Optional: Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
    zbTempSensor_V[i].zbTempSensor->setTolerance(0.1);

    // Add humidity cluster to the temperature sensor device with min, max and tolerance values
    zbTempSensor_V[i].zbTempSensor->addHumiditySensor(0, 100, 0.1);

    // Add endpoint to Zigbee Core
    Zigbee.addEndpoint(zbTempSensor_V[i].zbTempSensor);
  }

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
        
        for (int i=0; i<NumberOfDht; i++) {
          zbTempSensor_V[i].zbTempSensor->report();
          delay( pdMS_TO_TICKS(100));
        }
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
