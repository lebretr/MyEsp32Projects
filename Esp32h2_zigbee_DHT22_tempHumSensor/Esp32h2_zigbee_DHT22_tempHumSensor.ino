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

#define ZIGBEE_OUTLET_ENDPOINT_NUMBER  10

#define ANALOG_DEVICE_ENDPOINT_NUMBER 101

static const uint8_t button = BOOT_PIN;  //BOOT button (not RESET button!!!)

ZigbeeAnalog zbAnalogDevicePid = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER);

ZigbeeAnalog zbAnalogDeviceError = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 1);

ZigbeeAnalog zbAnalogDeviceVoltage = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 2);


#include <dhtnew.h>
#define DHTTYPE 22   // DHT 22  (AM2302)

struct zbTempSensor_S {
  ZigbeeTempSensor* zbTempSensor;
  bool reportingInitialized=false;
} zbTempSensor_R;

zbTempSensor_S zbTempSensor_V[NUMBER_OF_DHT_USED] ;

struct dhtTH_S {
  DHTNEW* dht;  //DHTNEW mySensor(5);   //  ESP 16    UNO 5    MKR1010 5
  float temperature;
  float humidity;
  unsigned long lastReadOk=0;
  // int readInterval = DHT_READ_INTERVAL;
  // unsigned long lastDHTRead = 0;
  int errorIndex;
} dhtTH_R;

dhtTH_S dhtTH_V[NUMBER_OF_DHT_USED] ;

static const int NumberOfDht=NUMBER_OF_DHT_USED;

#include "EmonLib.h"         // Include Emon Library

struct zmpt101b_S {
  EnergyMonitor emon;
  int errorIndex;
} zmpt101b_R;

zmpt101b_S zmpt101b;

ZigbeePowerOutlet zbOutlet = ZigbeePowerOutlet(ZIGBEE_OUTLET_ENDPOINT_NUMBER);

static const int NumberOfDevices=NumberOfDht+1;

struct error_S {
  int device_ERROR_CODE[NumberOfDevices];
} error_R;

error_S error_I;

/************************ Common function *****************************/

float myfAbs(float x) {
    uint32_t i = *((uint32_t*)&x);
    i &= 0x7FFFFFFF;  // Masque le bit de signe
    return *((float*)&i);
}

/************************ Temp sensor *****************************/
bool isTempChanged(float current, float previous) {
  return myfAbs(current - previous) > TEMP_SENSIBILITY;
}

bool isHumChanged(float current, float previous) {
  return myfAbs(current - previous) >= HUMIDITY_SENSIBILITY;
}

static void dht_reading(void *arg) {

  static const TickType_t xDelay = pdMS_TO_TICKS(2000); // Attendre l'intervalle minimum requis
  static const TickType_t xDelay100 = pdMS_TO_TICKS(100);

  for (;;) {
    unsigned long currentMillis = millis();
    
    for (int i=0; i<NumberOfDht; i++) {
        // check the reading status
        int status = dhtTH_V[i].dht->read();
        if (status != DHTLIB_OK) {

          char* m;
          switch (status)
          {
            case DHTLIB_ERROR_CHECKSUM:
              m="Checksum error";
              break;
            case DHTLIB_ERROR_TIMEOUT_A:
              m="Time out A error";
              break;
            case DHTLIB_ERROR_TIMEOUT_B:
              m="Time out B error";
              break;
            case DHTLIB_ERROR_TIMEOUT_C:
              m="Time out C error";
              break;
            case DHTLIB_ERROR_TIMEOUT_D:
              m="Time out D error";
              break;
            case DHTLIB_ERROR_SENSOR_NOT_READY:
              m="Sensor not ready";
              break;
            case DHTLIB_ERROR_BIT_SHIFT:
              m="Bit shift error";
              break;
            case DHTLIB_WAITING_FOR_READ:
              m="Waiting for read";
              break;
            default:
              m="Unknown";
              break;
          }

          ESP_LOGE(TAG, "Erreur DHT n°%d: %s %d", i, m, status);
        } else {
          float h= dhtTH_V[i].dht->getHumidity();
          float t= dhtTH_V[i].dht->getTemperature();

          error_I.device_ERROR_CODE[dhtTH_V[i].errorIndex]=0;

          dhtTH_V[i].temperature=t;
          dhtTH_V[i].humidity=h;
          dhtTH_V[i].lastReadOk=currentMillis;

          zbTempSensor_V[i].zbTempSensor->setTemperature(dhtTH_V[i].temperature);
          zbTempSensor_V[i].zbTempSensor->setHumidity(dhtTH_V[i].humidity);
          
          if(zbTempSensor_V[i].reportingInitialized==false){
            zbTempSensor_V[i].reportingInitialized=true;
            zbTempSensor_V[i].zbTempSensor->setReporting(MIN_REPORT_INTERVAL_SEC, MAX_REPORT_INTERVAL_SEC, TEMP_SENSIBILITY);
            zbTempSensor_V[i].zbTempSensor->setHumidityReporting(MIN_REPORT_INTERVAL_SEC, MAX_REPORT_INTERVAL_SEC, HUMIDITY_SENSIBILITY);
          }
        }

        if (currentMillis - dhtTH_V[i].lastReadOk > 10000) {
          error_I.device_ERROR_CODE[dhtTH_V[i].errorIndex]=1;
        }

        vTaskDelay(xDelay100);
    }

    vTaskDelay(xDelay); // Prefer vTaskDelay to delay() + yield()
  }
}

/************************ Power on/off sensor *****************************/
static void ZMPT101B_reading(void *arg) {

  static const TickType_t xDelay = pdMS_TO_TICKS(10000); // Attendre l'intervalle minimum requis

  unsigned long start = millis();

  for (;;) {
    zmpt101b.emon.calcVI(20, 2000);
    double Vrms = zmpt101b.emon.Vrms;

    if(millis()>start+10000){
      int volt=static_cast<int>(Vrms);
      zbAnalogDeviceVoltage.setAnalogInput(volt);
    
      if(volt<110){
        ESP_LOGI(TAG, "AC POWER OFF! : %d",volt);
        error_I.device_ERROR_CODE[zmpt101b.errorIndex]=0;
        zbOutlet.setState(false);
      }else if(volt>250){
        ESP_LOGE(TAG, "AC ERREUR? : %d",volt);
        error_I.device_ERROR_CODE[zmpt101b.errorIndex]=9;
        zbOutlet.setState(true);
      }else{
        ESP_LOGI(TAG, "AC ON : %d",volt);
        error_I.device_ERROR_CODE[zmpt101b.errorIndex]=0;
        zbOutlet.setState(true);
      }
    }
    vTaskDelay(xDelay); // Prefer vTaskDelay to delay() + yield()
  }
}

void setPowerOnOff(bool value) {
  ESP_LOGI(TAG, "I'm not a true switch. Nothing to do!");
}

/********************* Errors functions **************************/
static void error_code_sending(void *arg) {

  static const TickType_t xDelay = pdMS_TO_TICKS(2000); // Attendre l'intervalle minimum requis

  for (;;) {
    unsigned long ERROR_CODE=0;
    for (int i=0; i<NumberOfDevices; i++) {
      ERROR_CODE += (error_I.device_ERROR_CODE[i] * pow(10 , i));
    }

    zbAnalogDeviceError.setAnalogInput(ERROR_CODE);

    vTaskDelay(xDelay);
  }
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 5000);  // Wait Serial max 5s
  ESP_LOGI(TAG, " ");
  ESP_LOGI(TAG, " ");
  ESP_LOGI(TAG, "=== ESP32 Zigbee Temp/Hum startup ===");

  for (int i=0; i<NumberOfDevices; i++) {
    error_I.device_ERROR_CODE[i]=0;
  }

  // Init sensor with DHT22 model (compatibility with AM2302B)
  for (int i=0; i<NumberOfDht; i++) {
    dhtTH_V[i].errorIndex=i;
    
    if(i==0){
      pinMode(DHTPIN_0, INPUT_PULLUP);
      dhtTH_V[i].dht=new DHTNEW(DHTPIN_0);
    }
    if(i==1){
      pinMode(DHTPIN_1, INPUT_PULLUP);
      dhtTH_V[i].dht=new DHTNEW(DHTPIN_1);
    }
    if(i==2){
      pinMode(DHTPIN_2, INPUT_PULLUP);
      dhtTH_V[i].dht=new DHTNEW(DHTPIN_2);
    }
    if(i==3){
      pinMode(DHTPIN_3, INPUT_PULLUP);
      dhtTH_V[i].dht=new DHTNEW(DHTPIN_3);
    }
    if(i==4){
      pinMode(DHTPIN_4, INPUT_PULLUP);
      dhtTH_V[i].dht=new DHTNEW(DHTPIN_4);
    }
    if(i==5){
      pinMode(DHTPIN_5, INPUT_PULLUP);
      dhtTH_V[i].dht=new DHTNEW(DHTPIN_5);
    }

    
    dhtTH_V[i].dht->setType(DHTTYPE);

    ESP_LOGI(TAG, "Sensor n°%d initialised!", i);
  }

  // Init voltage reader
  zmpt101b.emon.voltage(ZMPT101BPIN_1, 230, 2);  // Voltage: input pin, calibration, phase_shift (2=>50hz, 1.7=>60hz)
  zmpt101b.emon.current(FAKECURRENTPIN_1, 111.1);       // Current: input pin, calibration.

  zmpt101b.errorIndex=int(NumberOfDevices)-1; //zmpt101b is the last device 

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Init zigbeeTempSensor
  for (int i=0; i<NumberOfDht; i++) {
    zbTempSensor_V[i].zbTempSensor = new ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER + i);
  }
  
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

  // Init zigbeeOutlet to track if there is current in the outlet
  zbOutlet.onPowerOutletChange(setPowerOnOff);
  Zigbee.addEndpoint(&zbOutlet);

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

  // Zigbee Sensor to track restart (crash, ...)
  zbAnalogDeviceVoltage.addAnalogInput();
  zbAnalogDeviceVoltage.setAnalogInputApplication(ESP_ZB_ZCL_AI_CURRENT_OTHER);
  zbAnalogDeviceVoltage.setAnalogInputDescription("Volt");
  zbAnalogDeviceVoltage.setAnalogInputResolution(1);

  Zigbee.addEndpoint(&zbAnalogDeviceVoltage);

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

  // Start Temperature sensor reading task
  BaseType_t taskErrorCodeSendingCreated = xTaskCreate(
    error_code_sending,
    "error_code_sending",
    4096,  // Stack augmenté pour sécurité
    NULL,
    5,  // Priorité moyenne
    NULL);

  if (taskErrorCodeSendingCreated != pdPASS) {
    ESP_LOGE(TAG, "ERROR: Échec création tâche d'envoie code erreur!");
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
  zbAnalogDeviceVoltage.setAnalogInputReporting(0,MAX_REPORT_INTERVAL_SEC,1);

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

    zbAnalogDeviceError.setAnalogInput(0);

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
