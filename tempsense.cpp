#include <iostream>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "wifi.h"
#include "ssd1306.h"
#include "sensor/bme280.h"
#include "callbacks/blocking.h"
#include "font.h"
#include <format>
#include <string>
#include <sstream>
#include <iomanip>
// Globals
mqtt_client_t *client;
struct mqtt_connect_client_info_t client_info_;
ssd1306_t disp;
bme280_t bme280;
u_int8_t *ip_address;

float tempC = 0.0;
float tempF = 0.0;
float humidity = 0.0;
float pressure = 0.0;
std::stringstream tempCstr;
std::stringstream tempFstr;
std::stringstream humiditystr;
std::stringstream pressurestr;


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
    ssd1306_clear(&disp);
    bme280_reading_t r;
    while (true) {
        cyw43_arch_poll();   // important for polled mode, safe in background too

        bme280_read(&bme280, &r);
        tempC = r.temperature;
        tempF = (r.temperature * 9/5) + 32;
        humidity = r.humidity;
        pressure = (r.pressure / 1000.0);

        tempCstr.str("");
        tempFstr.str("");
        humiditystr.str("");
        pressurestr.str("");
        tempCstr.clear();
        tempFstr.clear();
        humiditystr.clear();
        pressurestr.clear();
        
        tempCstr << std::fixed << std::setprecision(2) << tempC;
        tempFstr << std::fixed << std::setprecision(2) << tempF;
        humiditystr << std::fixed << std::setprecision(2) << humidity;
        pressurestr << std::fixed << std::setprecision(2) << pressure;
       

        mqtt_publish(client, "bedroom/temperatureC", tempCstr.str().c_str(), tempCstr.str().length(), 1,1,nullptr,nullptr);
        mqtt_publish(client, "bedroom/temperatureF", tempFstr.str().c_str(), tempFstr.str().length(), 1,1,nullptr,nullptr);
        mqtt_publish(client, "bedroom/humidity", humiditystr.str().c_str(), humiditystr.str().length(), 1,1,nullptr,nullptr);
        mqtt_publish(client, "bedroom/pressure", pressurestr.str().c_str(), pressurestr.str().length(), 1,1,nullptr,nullptr);

        tempCstr.str("");
        tempFstr.str("");
        humiditystr.str("");
        pressurestr.str("");
        tempCstr.clear();
        tempFstr.clear();
        humiditystr.clear();
        pressurestr.clear();

        //tempCstr << "TEMPERATURE: " << std::fixed << std::setprecision(2) << tempC << "°C";
        tempFstr << "TEMPERATURE: " << std::fixed << std::setprecision(2) << tempF << "F";
        humiditystr << "HUMIDITY: " << std::fixed << std::setprecision(2) << humidity << "%";
        pressurestr << "PRESSURE: " << std::fixed << std::setprecision(2) << pressure << "kPa";

        drawtext(tempFstr.str().c_str(), 0, 0);
        drawtext(humiditystr.str().c_str(), 0, 10);
        drawtext(pressurestr.str().c_str(), 0, 20);
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


    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_pull_up(16);
    gpio_pull_up(17);

    i2c_init(i2c_default, 400000);

    bme280_init_struct(&bme280, i2c_default, 0x76, &pico_callbacks_blocking);
    bme280_init(&bme280);
}


void drawtext(std::string str, int x, int y, uint8_t *font){

    ssd1306_draw_string_with_font(&disp, x, y, 1, font, str.c_str());
    ssd1306_show(&disp);
}