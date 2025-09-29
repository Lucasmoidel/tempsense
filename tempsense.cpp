#include <iostream>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "wifi.h"

// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// Globals
mqtt_client_t *client;
struct mqtt_connect_client_info_t client_info_;

// Forward declarations
void connectMQTT();
static void s_mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status);

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // I2C init
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PSK, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        auto *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }
    connectMQTT();
    while (true) {
        cyw43_arch_poll();   // important for polled mode, safe in background too
        sleep_ms(1000);
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
    printf("a");
    mqtt_client_connect(client, &broker_ip, broker_port, s_mqtt_connection_cb, nullptr, &client_info_);
    printf("b");
}

static void s_mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status) {
    printf("c");
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("Connected to MQTT broker successfully!\n");

        mqtt_subscribe(client, "test/topic", 1, nullptr, nullptr);

        const char* msg = "Hello from Pico W";
        mqtt_publish(client, "test/topic", msg, strlen(msg), 1, 0, nullptr, nullptr);
    } else {
        printf("MQTT connection failed with status: %d\n", status);
    }
}
