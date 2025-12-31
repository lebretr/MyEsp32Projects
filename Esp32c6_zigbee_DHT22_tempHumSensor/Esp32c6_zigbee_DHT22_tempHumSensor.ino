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
//https://docs.espressif.com/projects/arduino-esp32/en/latest/zigbee/zigbee_core.html

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

#define AC_SENSOR_ENDPOINT_NUMBER 13

#define ANALOG_DEVICE_ENDPOINT_NUMBER 101

uint8_t button = BOOT_PIN;  //BOOT button (not RESET button!!!)

ZigbeeAnalog zbAnalogDevicePid = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER);

ZigbeeAnalog zbAnalogDeviceError = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 1);

#include <DHTesp.h>
#define DHTTYPE DHTesp::DHT22 

struct zbTempSensor_S {
  ZigbeeTempSensor* zbTempSensor;
  bool reportingInitialized=false;
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

static int NumberOfDht=NUMBER_OF_DHT;

int DhtReadIntervalOnError=2000;

static unsigned long ERROR_CODE=0;

#include "EmonLib.h"         // Include Emon Library

EnergyMonitor emon1;         // Create an instance

ZigbeeElectricalMeasurement zbElectricalMeasurement = ZigbeeElectricalMeasurement(AC_SENSOR_ENDPOINT_NUMBER);

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
            dhtTH_V[i].DhtReadInterval=DhtReadIntervalOnError;
          }
          if(dhtTH_V[i].DhtReadInterval <= 60 * 1000){
            dhtTH_V[i].DhtReadInterval=dhtTH_V[i].DhtReadInterval + 2000;
            ESP_LOGI(TAG, "nouveau read interval DHT n°%d: %d", i, dhtTH_V[i].DhtReadInterval);
          }


        } else {
          if(dhtTH_V[i].receptionFailed==true){
            dhtTH_V[i].receptionFailed = false;
            ERROR_CODE=ERROR_CODE-(1* pow(10 , i));

            zbAnalogDeviceError.setAnalogInput(ERROR_CODE);
            // zbAnalogDeviceError.reportAnalogInput();
          }

          dhtTH_V[i].temperature=data.temperature;
          dhtTH_V[i].humidity=data.humidity;
          dhtTH_V[i].DhtReadInterval=DHT_READ_INTERVAL;

          zbTempSensor_V[i].zbTempSensor->setTemperature(dhtTH_V[i].temperature);
          zbTempSensor_V[i].zbTempSensor->setHumidity(dhtTH_V[i].humidity);
          
          if(zbTempSensor_V[i].reportingInitialized==false){
            // zbAnalogDeviceError.reportAnalogInput();
            zbTempSensor_V[i].reportingInitialized=true;
            zbTempSensor_V[i].zbTempSensor->setReporting(MIN_REPORT_INTERVAL_SEC, MAX_REPORT_INTERVAL_SEC, TEMP_SENSIBILITY);
            zbTempSensor_V[i].zbTempSensor->setHumidityReporting(MIN_REPORT_INTERVAL_SEC, MAX_REPORT_INTERVAL_SEC, HUMIDITY_SENSIBILITY);
          }
        }
      }
    }

    if(ERROR_CODE!=PREVIOUS_ERROR_CODE){
      vTaskDelay(pdMS_TO_TICKS(2000));
      PREVIOUS_ERROR_CODE=ERROR_CODE;
      zbAnalogDeviceError.setAnalogInput(ERROR_CODE);
      // zbAnalogDeviceError.reportAnalogInput();
    }
    vTaskDelay(xDelay); // Prefer vTaskDelay to delay() + yield()
  }
}


static void ZMPT101B_reading(void *arg) {

  static const TickType_t xDelay = pdMS_TO_TICKS(10000); // Attendre l'intervalle minimum requis
  static bool acReportingInitialized=false;

  unsigned long start = millis();

  for (;;) {
    emon1.calcVI(20, 2000);
    double Vrms = emon1.Vrms;

    if(millis()>start+10000){
      zbElectricalMeasurement.setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, Vrms);
        zbElectricalMeasurement.reportDC(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE);
    
      if(Vrms<30){
        ESP_LOGI(TAG, "AC POWER OFF! : %f",Vrms);
      }else if(Vrms>250){
        ESP_LOGE(TAG, "AC ERREUR? : %f",Vrms);
      }else{
        ESP_LOGI(TAG, "AC ON : %f",Vrms);
      }
      if(acReportingInitialized==false){
        ESP_LOGI(TAG, "zbElectricalMeasurement.setDCReporting");
        zbElectricalMeasurement.reportDC(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE);
        acReportingInitialized=true;
        
        // zbElectricalMeasurement.setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, 0);
        // zbElectricalMeasurement.setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, 0);

        // zbElectricalMeasurement.setDCReporting(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, 0, 30, 0); // report every 30 seconds if value changes by 10 (0.1V)
        // zbElectricalMeasurement.setDCReporting(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, 0, 30, 0);  // report every 30 seconds if value changes by 10 (0.1A)
        // zbElectricalMeasurement.setDCReporting(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, 0, 30, 0);
      }
    }
    vTaskDelay(xDelay); // Prefer vTaskDelay to delay() + yield()
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

  // Init voltage reader
  emon1.voltage(ZMPT101BPIN_1, 230, 2);  // Voltage: input pin, calibration, phase_shift (2=>50hz, 1.7=>60hz)

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

  zbElectricalMeasurement.addDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE);
  zbElectricalMeasurement.addDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT);
  zbElectricalMeasurement.addDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_POWER);

  zbElectricalMeasurement.setDCMinMaxValue(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, 0, 300000);  // 0-500.000V
  zbElectricalMeasurement.setDCMinMaxValue(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, 0, 1000);  // 0-1.000A
  zbElectricalMeasurement.setDCMinMaxValue(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, 0, 5000);    // 0-5.000W

  zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, 1, 1);  //Volt
  zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, 1, 1);  // 1/1000 = 0.001A (1 unit of measurement = 0.001A = 1mA)
  zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, 1, 1);    // 1/1000 = 0.001W (1 unit of measurement = 0.001W = 1mW)

  Zigbee.addEndpoint(&zbElectricalMeasurement);

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
  BaseType_t taskDHTReadCreated = xTaskCreate(
    dht_reading,
    "dht_reading",
    4096,  // Stack augmenté pour sécurité
    NULL,
    5,  // Priorité moyenne
    NULL);

  if (taskDHTReadCreated != pdPASS) {
    ESP_LOGE(TAG, "ERROR: Échec création tâche lecture capteur DHT!");
    ESP.restart();
  }

  // Start Voltage sensor reading task
  BaseType_t taskZMPT101BReadCreated = xTaskCreate(
    ZMPT101B_reading,
    "ZMPT101B_reading",
    4096,  // Stack augmenté pour sécurité
    NULL,
    5,  // Priorité moyenne
    NULL);

  if (taskZMPT101BReadCreated != pdPASS) {
    ESP_LOGE(TAG, "ERROR: Échec création tâche lecture capteur ZMPT101B!");
    ESP.restart();
  }

  // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (temp change in 0,1 °C)
  // for (int i=0; i<NumberOfDht; i++) {
  //   zbTempSensor_V[i].zbTempSensor->setReporting(MIN_REPORT_INTERVAL_SEC, MAX_REPORT_INTERVAL_SEC, TEMP_SENSIBILITY);
  //   zbTempSensor_V[i].zbTempSensor->setHumidityReporting(MIN_REPORT_INTERVAL_SEC, MAX_REPORT_INTERVAL_SEC, HUMIDITY_SENSIBILITY);
  // }

  zbAnalogDevicePid.setAnalogInputReporting(0,MAX_REPORT_INTERVAL_SEC,0);
  zbAnalogDeviceError.setAnalogInputReporting(0,MAX_REPORT_INTERVAL_SEC,0);

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
    // zbAnalogDevicePid.reportAnalogInput();

    zbAnalogDeviceError.setAnalogInput(0);
    // zbAnalogDeviceError.reportAnalogInput();

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

        
        zbAnalogDevicePid.reportAnalogInput();
        delay( pdMS_TO_TICKS(200) );

        zbAnalogDeviceError.reportAnalogInput();
        delay( pdMS_TO_TICKS(200) );
        
        for (int i=0; i<NumberOfDht; i++) {
          if ( !isnan(dhtTH_V[i].temperature) && 
              !isnan(dhtTH_V[i].humidity) && 
              (dhtTH_V[i].temperature!=0 || dhtTH_V[i].humidity!=0)
            ){
              zbTempSensor_V[i].zbTempSensor->report();
              delay( pdMS_TO_TICKS(200) );
          }
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
