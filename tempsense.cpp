#include <iostream>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "wifi.h"
#include "bme280.h"
#include "common.h"
#include "ssd1306.h"
#include "font.h"
#include <format>
#include <string>

// Globals
mqtt_client_t *client;
struct mqtt_connect_client_info_t client_info_;
ssd1306_t disp;

u_int8_t *ip_address;

#define SAMPLE_COUNT  UINT8_C(50)


// Forward declarations
void connectMQTT();
void setup_gpios(); 
void drawtext(std::string str, int x, int y, uint8_t *font=font_8x5);
static void s_mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status);

bool setupBME();

static int8_t get_temperature(uint32_t period, struct bme280_dev *dev);
static int8_t get_humidity(uint32_t period, struct bme280_dev *dev);
static int8_t get_pressure(uint32_t period, struct bme280_dev *dev);



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
    setupBME();
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


bool setupBME(){
        int8_t rslt;
    uint32_t period;
    struct bme280_dev dev;
    struct bme280_settings settings;

    /* Interface selection is to be updated as parameter
     * For I2C :  BME280_I2C_INTF
     * For SPI :  BME280_SPI_INTF
     */
    rslt = bme280_interface_selection(&dev, BME280_I2C_INTF);
    bme280_error_codes_print_result("bme280_interface_selection", rslt);

    rslt = bme280_init(&dev);
    bme280_error_codes_print_result("bme280_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bme280_get_sensor_settings(&settings, &dev);
    bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    settings.filter = BME280_FILTER_COEFF_2;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev);
    bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
    bme280_error_codes_print_result("bme280_set_power_mode", rslt);

    /* Calculate measurement time in microseconds */
    rslt = bme280_cal_meas_delay(&period, &settings);
    bme280_error_codes_print_result("bme280_cal_meas_delay", rslt);

    printf("\nTemperature calculation (Data displayed are compensated values)\n");
    printf("Measurement time : %lu us\n\n", (long unsigned int)period);

    rslt = get_temperature(period, &dev);
    bme280_error_codes_print_result("get_temperature", rslt);

    bme280_coines_deinit();

    return true;
}

static int8_t get_temperature(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_TEMP, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifndef BME280_DOUBLE_ENABLE
            comp_data.temperature = comp_data.temperature / 100;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Temperature[%d]:   %lf deg C\n", idx, comp_data.temperature);
#else
            printf("Temperature[%d]:   %ld deg C\n", idx, (long int)comp_data.temperature);
#endif
            idx++;
        }
    }

    return rslt;
}

static int8_t get_humidity(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifndef BME280_DOUBLE_ENABLE
            comp_data.humidity = comp_data.humidity / 1000;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Humidity[%d]:   %lf %%RH\n", idx, comp_data.humidity);
#else
            printf("Humidity[%d]:   %lu %%RH\n", idx, (long unsigned int)comp_data.humidity);
#endif
            idx++;
        }
    }

    return rslt;
}


static int8_t get_pressure(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_PRESS, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifdef BME280_64BIT_ENABLE
            comp_data.pressure = comp_data.pressure / 100;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Pressure[%d]:  %lf Pa\n", idx, comp_data.pressure);
#else
            printf("Pressure[%d]:   %lu Pa\n", idx, (long unsigned int)comp_data.pressure);
#endif
            idx++;
        }
    }

    return rslt;
}
