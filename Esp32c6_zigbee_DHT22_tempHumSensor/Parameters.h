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

#define DHTPIN_1 23
// #define DHTPIN_2 22
// #define DHTPIN_3 21
// #define DHTPIN_4 20
// #define DHTPIN_5 11
// #define DHTPIN_6 10

#define AUTHOR "Remi Lebret"
#define MODEL "Esp32-C6-Temp&HumSensor"

#define DHT_READ_INTERVAL 10 * 1000     // 10 secondes
#define REPORT_INTERVAL 30 * 60 * 1000  // 30 minutes
#define TEMP_SENSIBILITY 0.2            // minimum change to report new temperature
#define HUMIDITY_SENSIBILITY 0.5        // minimum change to report new humidity