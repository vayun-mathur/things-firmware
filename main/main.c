#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "things"

// Must match the Android app's UUIDs
static const ble_uuid128_t SERVICE_UUID =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t CHAR_UUID =
    BLE_UUID128_INIT(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

// XIAO ESP32-C3: BOOT button on GPIO9
// XIAO ESP32-S3: BOOT button on GPIO0
// Change this if using a different board variant
#define BUTTON_GPIO 9

// ---- ADXL345 (I2C accelerometer) ----
#define I2C_SDA_GPIO 6   // XIAO D4
#define I2C_SCL_GPIO 7   // XIAO D5
#define ADXL345_ADDR_LOW  0x53
#define ADXL345_ADDR_HIGH 0x1D
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0      0x32
// +-2g, 10-bit right-justified -> 256 LSB per g
#define ADXL345_LSB_PER_G 256.0f

// ---- HX711 (load cell amplifier) ----
#define HX711_DOUT_GPIO 4   // XIAO D2 (data, input)
#define HX711_SCK_GPIO  5   // XIAO D3 (clock, output)
// Counts per gram. CALIBRATE: place a known weight, then
// HX711_SCALE = (raw_reading - tare_offset) / known_grams
#define HX711_SCALE 420.0f

static uint16_t conn_handle = 0;
static bool connected = false;
static uint16_t char_val_handle;
static uint8_t own_addr_type;

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t adxl_dev = NULL;
static bool adxl_ok = false;
static bool hx711_ok = false;
static int32_t hx711_offset = 0;
static portMUX_TYPE hx711_mux = portMUX_INITIALIZER_UNLOCKED;

static const char *WATER_MSG = "You drank 300 mL of water";

// ---------------- ADXL345 ----------------

static esp_err_t adxl345_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(adxl_dev, buf, sizeof(buf), 1000);
}

static esp_err_t adxl345_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(adxl_dev, &reg, 1, data, len, 1000);
}

static void i2c_bus_scan(void) {
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02x", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "I2C scan: no devices (check SDA=GPIO%d, SCL=GPIO%d, 3V3, GND)",
                 I2C_SDA_GPIO, I2C_SCL_GPIO);
    }
}

static void adxl345_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &i2c_bus) != ESP_OK) {
        ESP_LOGW(TAG, "ADXL345: I2C bus init failed");
        return;
    }

    i2c_bus_scan();

    uint8_t addrs[] = {ADXL345_ADDR_LOW, ADXL345_ADDR_HIGH};
    for (int i = 0; i < 2; i++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &adxl_dev) != ESP_OK) {
            adxl_dev = NULL;
            continue;
        }
        uint8_t devid = 0;
        if (adxl345_read(ADXL345_REG_DEVID, &devid, 1) == ESP_OK && devid == 0xE5) {
            adxl345_write_reg(ADXL345_REG_DATA_FORMAT, 0x00); // +-2g, 10-bit
            adxl345_write_reg(ADXL345_REG_POWER_CTL, 0x08);   // measure mode
            adxl_ok = true;
            ESP_LOGI(TAG, "ADXL345 ready at 0x%02x", addrs[i]);
            return;
        }
        ESP_LOGW(TAG, "ADXL345 not at 0x%02x (devid=0x%02x)", addrs[i], devid);
        i2c_master_bus_rm_device(adxl_dev);
        adxl_dev = NULL;
    }
    ESP_LOGW(TAG, "ADXL345 not found, check wiring");
}

static bool adxl345_read_g(float *x, float *y, float *z) {
    if (!adxl_ok) return false;
    uint8_t d[6];
    if (adxl345_read(ADXL345_REG_DATAX0, d, 6) != ESP_OK) return false;
    int16_t rx = (int16_t)((d[1] << 8) | d[0]);
    int16_t ry = (int16_t)((d[3] << 8) | d[2]);
    int16_t rz = (int16_t)((d[5] << 8) | d[4]);
    *x = rx / ADXL345_LSB_PER_G;
    *y = ry / ADXL345_LSB_PER_G;
    *z = rz / ADXL345_LSB_PER_G;
    return true;
}

// ---------------- HX711 ----------------

static bool hx711_read_raw(int32_t *out) {
    // Wait for the HX711 to signal "data ready" by pulling DOUT low.
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(HX711_DOUT_GPIO)) {
        if (esp_timer_get_time() - t0 > 250000) return false; // 250 ms timeout
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint32_t value = 0;
    // Timing-critical: a >60us gap between pulses powers the chip down.
    taskENTER_CRITICAL(&hx711_mux);
    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_GPIO, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | gpio_get_level(HX711_DOUT_GPIO);
        gpio_set_level(HX711_SCK_GPIO, 0);
        esp_rom_delay_us(1);
    }
    // 25th pulse selects channel A, gain 128 for the next reading.
    gpio_set_level(HX711_SCK_GPIO, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK_GPIO, 0);
    esp_rom_delay_us(1);
    taskEXIT_CRITICAL(&hx711_mux);

    if (value & 0x800000) value |= 0xFF000000; // sign-extend 24-bit
    *out = (int32_t)value;
    return true;
}

static bool hx711_read_avg(int samples, int32_t *out) {
    int64_t sum = 0;
    int got = 0;
    for (int i = 0; i < samples; i++) {
        int32_t v;
        if (hx711_read_raw(&v)) {
            sum += v;
            got++;
        }
    }
    if (got == 0) return false;
    *out = (int32_t)(sum / got);
    return true;
}

static void hx711_init(void) {
    gpio_config_t sck = {
        .pin_bit_mask = (1ULL << HX711_SCK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&sck);
    gpio_set_level(HX711_SCK_GPIO, 0);

    gpio_config_t dout = {
        .pin_bit_mask = (1ULL << HX711_DOUT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // disconnected DT reads high -> timeout
    };
    gpio_config(&dout);

    int32_t offset;
    if (hx711_read_avg(16, &offset)) {
        hx711_offset = offset;
        hx711_ok = true;
        ESP_LOGI(TAG, "HX711 ready, tare offset=%ld", (long)offset);
    } else {
        ESP_LOGW(TAG, "HX711 not responding, check wiring");
    }
}

static bool hx711_grams(float *grams) {
    if (!hx711_ok) return false;
    int32_t raw;
    if (!hx711_read_avg(8, &raw)) return false;
    *grams = (raw - hx711_offset) / HX711_SCALE;
    return true;
}

// ---------------- BLE ----------------

static int char_access_cb(uint16_t conn, uint16_t attr,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        os_mbuf_append(ctxt->om, WATER_MSG, strlen(WATER_MSG));
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &CHAR_UUID.u,
                .access_cb = char_access_cb,
                .val_handle = &char_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {0},
};

static void start_advertising(void);

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            connected = true;
            ESP_LOGI(TAG, "connected");
        } else {
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        connected = false;
        ESP_LOGI(TAG, "disconnected");
        start_advertising();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe: cur_notify=%d", event->subscribe.cur_notify);
        break;
    }
    return 0;
}

static void start_advertising(void) {
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen(ble_svc_gap_device_name());
    fields.name_is_complete = 1;
    fields.uuids128 = (ble_uuid128_t[]){SERVICE_UUID};
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_cb, NULL);
    ESP_LOGI(TAG, "advertising started");
}

static void ble_on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);
    start_advertising();
}

static void nimble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void build_message(char *msg, size_t len) {
    int n = snprintf(msg, len, "%s", WATER_MSG);

    float x, y, z;
    if (adxl345_read_g(&x, &y, &z)) {
        n += snprintf(msg + n, len - n,
                      " | accel x=%.2f y=%.2f z=%.2f g", x, y, z);
    } else {
        n += snprintf(msg + n, len - n, " | accel n/a");
    }

    float grams;
    if (hx711_grams(&grams)) {
        n += snprintf(msg + n, len - n, " | weight %.0f g", grams);
    } else {
        snprintf(msg + n, len - n, " | weight n/a");
    }
}

static void button_task(void *arg) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    char msg[200];
    bool last_state = true; // pulled up = high when not pressed
    while (1) {
        bool current = gpio_get_level(BUTTON_GPIO);
        // Button press = falling edge (active low)
        if (last_state && !current) {
            ESP_LOGI(TAG, "button pressed");
            build_message(msg, sizeof(msg));
            if (connected) {
                struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
                ble_gatts_notify_custom(conn_handle, char_val_handle, om);
                ESP_LOGI(TAG, "sent: %s", msg);
            } else {
                ESP_LOGW(TAG, "not connected: %s", msg);
            }
            vTaskDelay(pdMS_TO_TICKS(300)); // debounce
        }
        last_state = current;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void sensor_debug_task(void *arg) {
    while (1) {
        uint8_t devid = 0xFF;
        esp_err_t e = adxl_dev ? adxl345_read(ADXL345_REG_DEVID, &devid, 1) : ESP_FAIL;
        float x = 0, y = 0, z = 0;
        adxl345_read_g(&x, &y, &z);
        int32_t raw = 0;
        bool got = hx711_read_raw(&raw);
        ESP_LOGI(TAG,
                 "diag: adxl[i2c=%s devid=0x%02x] x=%.2f y=%.2f z=%.2f g | hx[read=%s raw=%ld]",
                 e == ESP_OK ? "ok" : "ERR", devid, x, y, z,
                 got ? "ok" : "TIMEOUT", (long)raw);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    adxl345_init();
    hx711_init();

    nimble_port_init();
    ble_att_set_preferred_mtu(247); // allow longer notifications
    ble_svc_gap_device_name_set("Things");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(nimble_host_task);
    xTaskCreate(button_task, "button", 4096, NULL, 5, NULL);
    xTaskCreate(sensor_debug_task, "sensordbg", 4096, NULL, 4, NULL);
}
