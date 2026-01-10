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

#define NUMBER_OF_DHT_USED 1
#define DHTPIN_0 22
#define DHTPIN_1 25
#define DHTPIN_2 12
#define DHTPIN_3 11
#define DHTPIN_4 10
#define DHTPIN_5 0

#define ZMPT101BPIN_1 1
#define FAKECURRENTPIN_1 2

#define AUTHOR "Remi Lebret"
#define MODEL "Esp32-H2-multiTH&outlet"

//#define DHT_READ_INTERVAL 10 * 1000     // 10 secondes
#define MAX_REPORT_INTERVAL_SEC 30 * 60  // 30 minutes
#define MIN_REPORT_INTERVAL_SEC 10  // 10 sec
#define TEMP_SENSIBILITY 0.2         // minimum change to report new temperature
#define HUMIDITY_SENSIBILITY 0.5        // minimum change to report new humidity