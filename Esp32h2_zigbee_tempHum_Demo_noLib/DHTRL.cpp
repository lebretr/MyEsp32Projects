

#include "DHTRL.h"

DHTRL::DHTRL(uint8_t pin)
{
  this->_dataPin = pin;
}

int DHTRL::read() {
  this->isValid = false;
  
  uint8_t bits[5] = {0}; // 40 bits de données (5 octets)
  uint8_t idx = 0;
  uint8_t bitIdx = 7;
  
  // Étape 1: Signal de démarrage - MCU tire la ligne BAS pendant au moins 1ms
  //  REQUEST SAMPLE - SEND WAKEUP TO SENSOR
  pinMode(this->_dataPin, OUTPUT);
  digitalWrite(this->_dataPin, LOW);
  delay(1);
  
  // Étape 2: MCU tire la ligne HAUT pendant 20-40µs
  //  HOST GIVES CONTROL TO SENSOR
  digitalWrite(this->_dataPin, HIGH);
  delayMicroseconds(30);
  
  // Étape 3: Passer en mode lecture
  pinMode(this->_dataPin, INPUT_PULLUP);
  
  // Étape 4: Attendre la réponse du capteur (80µs LOW + 80µs HIGH)
  uint32_t timeout = micros();
  while(digitalRead(this->_dataPin) == HIGH) {
    if((micros() - timeout) > 100) { 
      Serial.println("ERROR 1");
      return DHTLIB_ERROR_TIMEOUT_A;
    }
  }
  
  timeout = micros();
  while(digitalRead(this->_dataPin) == LOW) {
    if((micros() - timeout) > 100) { 
      Serial.println("ERROR 2");
      return DHTLIB_ERROR_TIMEOUT_B;
    }
  }
  
  timeout = micros();
  while(digitalRead(this->_dataPin) == HIGH) {
    if((micros() - timeout) > 100) { 
      Serial.println("ERROR 3");
      return DHTLIB_ERROR_TIMEOUT_C;
    }
  }
  
  // Étape 5: Lire les 40 bits de données
  for(int i = 0; i < 40; i++) {
    // Attendre le début du signal (50µs LOW)
    timeout = micros();
    while(digitalRead(this->_dataPin) == LOW) {
      if((micros() - timeout) > 70) { 
        Serial.println("ERROR 4");
        return DHTLIB_ERROR_TIMEOUT_D;
      }
    }
    
    // Mesurer la durée du signal HIGH
    // Si HIGH dure ~26-28µs = bit 0
    // Si HIGH dure ~70µs = bit 1
    uint32_t startTime = micros();
    
    timeout = micros();
    while(digitalRead(this->_dataPin) == HIGH) {
      if((micros() - timeout) > 90) { 
        Serial.println("ERROR 5");
        return DHTLIB_ERROR_TIMEOUT_E;
      }
    }
    
    uint32_t duration = micros() - startTime;
    
    // Si la durée > 40µs, c'est un bit 1
    if(duration > 40) {
      bits[idx] |= (1 << bitIdx);
    }
    
    if(bitIdx == 0) {
      bitIdx = 7;
      idx++;
    } else {
      bitIdx--;
    }
  }
  
  // Étape 6: Vérifier le checksum
  uint8_t checksum = bits[0] + bits[1] + bits[2] + bits[3];
  if(checksum != bits[4]) { 
    Serial.println("ERROR 6");
    return DHTLIB_ERROR_CHECKSUM;
  }
  
  // Étape 7: Calculer l'humidité et la température
  // DHT22/AM2302B: 16 bits humidité, 16 bits température
  uint16_t rawHumidity = (bits[0] << 8) | bits[1];
  uint16_t rawTemp = (bits[2] << 8) | bits[3];
  
  this->humidity = rawHumidity * 0.1;
  
  // Vérifier le bit de signe pour la température
  if(rawTemp & 0x8000) {
    rawTemp &= 0x7FFF;
    this->temperature = rawTemp * -0.1;
  } else {
    this->temperature = rawTemp * 0.1;
  }
  
  this->isValid = true;
  return DHTLIB_OK;
}