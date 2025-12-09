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

#define DHT_READ_INTERVAL 10 * 1000  // 10 secondes
#define REPORT_INTERVAL 5 * 60 * 1000   // 5 minutes
#define TEMP_DELTA 0.2           // Changement minimum de température
#define HUM_DELTA 1.0            // Changement minimum d'humidité

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

/* Zigbee binary sensor device configuration */
#define BINARY_DEVICE_ENDPOINT_NUMBER 2

#define CUSTOM_CLUSTER_ENDPOINT_NUMBER 5

#define CUSTOM_CLUSTER_ID 0xFC00  // Plage manufacturer-specific
#define ATTR_START_TIME   0x0000
#define CUSTOM_ATTR_START_TIME 0x8000  // Attribut personnalisé


class ZigbeeCustomDevice : public ZigbeeEP {
private:
  uint32_t _startTime;

public:
  ZigbeeCustomDevice(uint8_t endpoint) : ZigbeeEP(endpoint) {
    _device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID;  // Custom Sensor device ID
    _startTime = 0;
    
    // Créer le cluster Temperature
    // _cluster_id = CUSTOM_CLUSTER_ID;  // Temperature Measurement cluster
  }
  
  void setStartTime(uint32_t time) {
    _startTime = time;
  }
  
  uint32_t getStartTime() {
    return _startTime;
  }
  
  // Appelé lors de la configuration Zigbee
  void setAttributeList()  {
    // Attribut température standard (requis)
    int16_t temp = 0;
    addAttribute(0x0000, ZCL_INT16S_ATTRIBUTE_TYPE, ZCL_READ_ONLY_ATTRIBUTE, (void*)&temp);
    
    // Attribut startTime personnalisé
    addAttribute(0x8000, ZCL_UTC_TIME_ATTRIBUTE_TYPE, ZCL_READ_WRITE_ATTRIBUTE, (void*)&_startTime);
  }
};

ZigbeeCustomDevice customDevice(1);

uint8_t button = BOOT_PIN;  //BOOT button (not RESET!!!)

// Optional Time cluster variables
struct tm timeinfo;
struct tm *localTime;
int32_t timezone;

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

ZigbeeBinary zbBinary = ZigbeeBinary(BINARY_DEVICE_ENDPOINT_NUMBER);

#include <DHTesp.h>

#define DHTPIN 23
#define DHTTYPE DHTesp::DHT22 
DHTesp dht;

float temperature=NULL;
float humidity=NULL;

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

static void temp_sensor_read(void *arg) {

  const TickType_t xDelay = pdMS_TO_TICKS(100);

  // Variables pour timers non-bloquants
  static unsigned long lastDHTRead = 0;

  for (;;) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastDHTRead >= DHT_READ_INTERVAL) {

      // Attendre l'intervalle minimum requis
      delay(dht.getMinimumSamplingPeriod());
      
      // Lire les données du capteur
      TempAndHumidity data = dht.getTempAndHumidity();
  
      // Vérifier le statut de la lecture
      if (dht.getStatus() != 0) {
        Serial.print("Erreur DHT: ");
        Serial.println(dht.getStatusString());
        vTaskDelay(pdMS_TO_TICKS(2000));
      }else{
        // Le DHT22 renvoie au maximum une mesure toute les 2s
        // float t =data.temperature; // Lis le taux d'humidite en %
        // float h = data.humidity; // Lis la température en degré celsius

        static int nb_echec_reception = 0;
        if (isnan(data.humidity) || isnan(data.temperature)) {
          nb_echec_reception++;
          Serial.printf("Échec réception: %d\n", nb_echec_reception);
          vTaskDelay(pdMS_TO_TICKS(2000));
          // if (nb_echec_reception > 6) {
          //   Serial.println("Trop d'échecs, redémarrage...");
          //   vTaskDelay(pdMS_TO_TICKS(1000));
          //   ESP.restart();
          // }
        } else {
          lastDHTRead = currentMillis;
          nb_echec_reception = 0;
          temperature=data.temperature;
          humidity=data.humidity;
        }
      }
    }
    vTaskDelay(xDelay); // Préférer vTaskDelay à delay() + yield()
  }
}

static void temp_zigbee_send(void *arg) {

  const TickType_t xDelay = pdMS_TO_TICKS(1000);

  // Variables pour timers non-bloquants
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
    vTaskDelay(xDelay);  // Préférer vTaskDelay à delay() + yield()
  }
}


/************************ Custom sensor *****************************/
esp_err_t create_custom_cluster_with_start_time(uint8_t endpoint)
{
    // Créer un cluster vide personnalisé
    esp_zb_attribute_list_t *custom_cluster = 
        esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    
    // Ajouter startTime
    uint32_t start_time = random(1000);
    esp_zb_cluster_add_attr(custom_cluster,
                           CUSTOM_CLUSTER_ID,
                           ATTR_START_TIME,
                           ESP_ZB_ZCL_ATTR_TYPE_UTC_TIME,
                           ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                           &start_time);
    
    // Ajouter au cluster list
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster,
                                           ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    return ESP_OK;
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 5000);  // Attendre le Serial max 5s
  Serial.println("\n=== Démarrage ESP32-C6 Zigbee Temp/Hum ===");

  randomSeed(analogRead(0));
  Serial.print("analogRead(0): ");
  Serial.println(analogRead(0));

  // Initialiser le capteur avec le modèle DHT22 (compatible AM2302B)
  dht.setup(DHTPIN, DHTTYPE);
  
  Serial.println("Capteur initialisé!");
  Serial.print("Intervalle minimum entre lectures: ");
  Serial.print(dht.getMinimumSamplingPeriod());
  Serial.println("ms");

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("Rémi Lebret", "Esp32-C6-01-Temp&Hum");

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

  // Ajouter l'attribut startTime au cluster Temperature (0x0402)
  // uint32_t startTime = 0;
  // zbTempSensor.addAttribute(
  //   CUSTOM_ATTR_START_TIME,           // ID attribut
  //   ZCL_UTC_TIME_ATTRIBUTE_TYPE,      // Type UTC Time
  //   ZCL_READ_WRITE_ATTRIBUTE,         // Lecture/écriture
  //   (void*)&startTime                 // Pointeur vers la valeur
  // );

  Zigbee.addEndpoint(&customDevice);

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);

  // Set up binary zone armed input (Security)
  zbBinary.addBinaryInput();
  //zbBinary.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_SECURITY_ZONE_ARMED);
  zbBinary.setBinaryInputDescription("Start");
  
  Zigbee.addEndpoint(&zbBinary);

  create_custom_cluster_with_start_time(CUSTOM_CLUSTER_ENDPOINT_NUMBER);

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
  // Créer la tâche avec priorité et stack appropriés
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


  Serial.println("=== Initialisation terminée ===\n");
}

void loop() {

  // Variables pour timers non-bloquants
  static int startStatus = 0;
  static unsigned long startTime = 0;
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
    long randNumber = random(300);
    Serial.println(randNumber);

    Serial.println("Envoi du statut started = true");

    startStatus=1;
    startTime=currentMillis;
    
    zbBinary.setBinaryInput(true);
    zbBinary.reportBinaryInput();
  }else if(startStatus==1 && currentMillis - startTime >= 5000){
    Serial.println("remise à false du statut startSended");

    startStatus = 2;
    
    zbBinary.setBinaryInput(false);
    zbBinary.reportBinaryInput();
  }

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
      
      if (pressDuration >= FACTORY_RESET_DELAY) {
        Serial.println("Factory reset Zigbee et redémarrage dans 1s...");
        delay(1000);
        Zigbee.factoryReset();
      }
      buttonPressed = false;
    }
  }

  delay(50);  // Réduit à 50ms pour meilleure réactivité
}