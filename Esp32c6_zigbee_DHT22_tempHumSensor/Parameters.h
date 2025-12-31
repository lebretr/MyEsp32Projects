// https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html

//╔═══════════════════════════════════════════════════════════╗
//║ Recommended PIN for DHT22 :                               ║
//║ - GPIO20, GPIO21, GPIO22, GPIO23                          ║
//║ - GPIO18, GPIO19 (if you don't use External SDIO)         ║
//║ - GPIO0, GPIO1                                            ║
//║ - GPIO6, GPIO7                                            ║
//║ - GPIO10, GPIO11                                          ║
//╠═──────────────────────────────────────────────────────────╣
//║ Recommended PIN for ZMPT101B :                            ║
//║ - GPIO0, GPIO1, GPIO2, GPIO3                              ║
//╚═══════════════════════════════════════════════════════════╝

#define NUMBER_OF_DHT 1
#define DHTPIN_0 22
#define DHTPIN_1 23
#define DHTPIN_2 21
#define DHTPIN_3 20
#define DHTPIN_4 11
#define DHTPIN_5 10

#define ZMPT101BPIN_1 0

#define AUTHOR "Remi Lebret"
#define MODEL "Esp32-C6-multiTH&outlet"

#define DHT_READ_INTERVAL 10 * 1000     // 10 secondes
#define MAX_REPORT_INTERVAL_SEC 30 * 60  // 30 minutes
#define MIN_REPORT_INTERVAL_SEC 10  // 10 sec
#define TEMP_SENSIBILITY 0.2         // minimum change to report new temperature
#define HUMIDITY_SENSIBILITY 0.5        // minimum change to report new humidity