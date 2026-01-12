#include "DHTRL.h"
// Définition des broches
#define AM2302B_PIN 22      // GPIO pour AM2302B

/**
 * Affiche les données d'un capteur
 */
void afficherDonnees(DHTRL* data, const char* nomCapteur) {
  if(!data->isValid) {
    Serial.print("Erreur de lecture du capteur ");
    Serial.println(nomCapteur);
    return;
  }
  
  Serial.println("=================================");
  Serial.print("Capteur: ");
  Serial.println(nomCapteur);
  Serial.print("Humidité: ");
  Serial.print(data->humidity, 1);
  Serial.println(" %");
  Serial.print("Température: ");
  Serial.print(data->temperature, 1);
  Serial.println(" °C");
  Serial.println("=================================");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("Initialisation des capteurs DHT22 et AM2302B...");
  Serial.println("Capteurs prêts !");
}

void loop() {  
  // Lecture du AM2302B
  DHTRL* dataAM2302B = new DHTRL(AM2302B_PIN);

  int status = dataAM2302B->read();
  
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
      // case DHTLIB_ERROR_SENSOR_NOT_READY:
      //   m="Sensor not ready";
      //   break;
      // case DHTLIB_ERROR_BIT_SHIFT:
      //   m="Bit shift error";
      //   break;
      // case DHTLIB_WAITING_FOR_READ:
      //   m="Waiting for read";
      //   break;
      default:
        m="Unknown";
        break;
    }

    Serial.printf("Erreur DHT reading: %s %d \n", m, status);
  } else {
    afficherDonnees(dataAM2302B, "AM2302B");
  }
  
  // Attendre 2 secondes entre les lectures (DHT22 nécessite minimum 2s)
  delay(2000);
}