// https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html

//╔═══════════════════════════════════════════════════════════╗
//║ Recommended PIN for DHT22 :                               ║
//║ - GPIO10, GPIO11, GPIO12, GPIO22, GPIO25                  ║
//║ - GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5 (if you don't  ║
//║        use External SPI Flash)                            ║
//╠═──────────────────────────────────────────────────────────╣
//║ Pins to avoid or with restriction  :                      ║
//║ GPIO8 : integrated LED RGB                                ║
//║ GPIO9 : connected to BOOT bouton                          ║
//║ GPIO13 et GPIO14 : if cristal 32.768 kHz is present       ║
//║ GPIO23 (RX) et GPIO24 (TX) : used for UART                ║
//╠═──────────────────────────────────────────────────────────╣
//║ PIN for ZMPT101B :                                        ║
//║ - GPIO1, GPIO2, GPIO3, GPIO4, GPIO5                       ║
//║                        /!\ /!\ /!\                        ║  
//║               CHECK ZMPT101B out is 3.3V !!!              ║
//║         ADC on ESP32-H2 works only with 3.3V!!!           ║
//║                        /!\ /!\ /!\                        ║  
//╚═══════════════════════════════════════════════════════════╝

#define DHTPIN_1 25
// #define DHTPIN_2 22
// #define DHTPIN_3 12
// #define DHTPIN_4 11
// #define DHTPIN_5 10

#define AUTHOR "Remi Lebret"
#define MODEL "Esp32-H2-Temp&HumSensor"

#define DHT_READ_INTERVAL 10 * 1000     // 10 secondes
#define REPORT_INTERVAL 30 * 60 * 1000  // 30 minutes
#define TEMP_SENSIBILITY 0.2            // minimum change to report new temperature
#define HUMIDITY_SENSIBILITY 0.5        // minimum change to report new humidity