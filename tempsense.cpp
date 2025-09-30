#include <iostream>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "wifi.h"
//#include "bme280.h"
//#include "humidity.c"
//#include "temperature.c"
//#include "pressure.c"
#include "ssd1306.h"
#include "font.h"
#include <format>
#include <string>

// Globals
mqtt_client_t *client;
struct mqtt_connect_client_info_t client_info_;
ssd1306_t disp;

u_int8_t *ip_address;

// Forward declarations
void connectMQTT();
void setup_gpios(); 
void drawtext(std::string str, int x, int y, uint8_t *font=font_8x5);
static void s_mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status);

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PSK, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }
    connectMQTT();
    setup_gpios();
    while (true) {
        cyw43_arch_poll();   // important for polled mode, safe in background too
        mqtt_publish(client, "bedroom/temperatureC", "32.00", 5, 1,1,nullptr,nullptr);
        mqtt_publish(client, "bedroom/temperatureF", "32.00", 5, 1,1,nullptr,nullptr);
        mqtt_publish(client, "bedroom/humidity", "50.00", 5, 1,1,nullptr,nullptr);
        mqtt_publish(client, "bedroom/temperatureC", "0990.00", 6, 1,1,nullptr,nullptr);
        drawtext("TEMPERATURE: 32.00F", 0, 0);
        drawtext("HUMIDITY: 50%", 0, 10);
        drawtext("PRESSURE: 900.00hPa", 0, 20);
        drawtext(std::format("IP: {}.{}.{}.{}\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]), 0, 40);
        sleep_ms(5000);
        ssd1306_clear(&disp);
    }
    
}

void connectMQTT() {
    ip_addr_t broker_ip;
    IP4_ADDR(&broker_ip, 10, 0, 0, 234);
    u16_t broker_port = 1883;
    
    client = mqtt_client_new();   // allocate client
    if (!client) {
        printf("Failed to create MQTT client\n");
        return;
    }

    memset(&client_info_, 0, sizeof(client_info_));
    client_info_.client_id = "bedroom";
    mqtt_client_connect(client, &broker_ip, broker_port, s_mqtt_connection_cb, nullptr, &client_info_);
}

static void s_mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("Connected to MQTT broker successfully!\n");
        } else {
        printf("MQTT connection failed with status: %d\n", status);
    }
}


void setup_gpios() {
    i2c_init(i2c1, 400000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c1);
}


void drawtext(std::string str, int x, int y, uint8_t *font){

    ssd1306_draw_string_with_font(&disp, x, y, 1, font, str.c_str());
    ssd1306_show(&disp);
}