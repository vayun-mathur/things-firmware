#include <stdio.h>
#include <string.h>
#include <math.h>
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

// ---- Drink detection ----
#define WATER_DENSITY_G_PER_ML 1.0f  // water: 1 g == 1 mL
#define TILT_DRINK_DEG      40.0f    // tilt above this = drinking in progress
#define TILT_UPRIGHT_DEG    25.0f    // tilt below this = cup upright / at rest
#define TILT_POUROUT_DEG    100.0f   // max tilt beyond this = pour-out, ignore the drop
#define MIN_SIP_G           5.0f     // ignore weight drops smaller than this (noise)
#define WEIGHT_STABLE_G     3.0f     // max spread (g) across reads to call weight settled
#define SETTLE_TIMEOUT_MS   8000     // give up waiting for the weight to settle
#define BASELINE_REFRESH_US 2000000  // min interval between idle baseline refreshes

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

// Gravity direction captured at boot while upright; basis for the tilt angle.
static float grav_ref[3] = {0.0f, 0.0f, 1.0f};
static bool grav_ref_ok = false;

// Last reported sip volume (mL), exposed via the BLE read.
static int last_ml = 0;

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

// Average a handful of accel samples to lock in the upright orientation.
static void capture_upright_ref(void) {
    float x, y, z, sx = 0, sy = 0, sz = 0;
    int got = 0;
    for (int i = 0; i < 32; i++) {
        if (adxl345_read_g(&x, &y, &z)) { sx += x; sy += y; sz += z; got++; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (got == 0) { ESP_LOGW(TAG, "upright ref: no accel data"); return; }
    float mag = sqrtf(sx * sx + sy * sy + sz * sz);
    if (mag < 1e-3f) return;
    grav_ref[0] = sx / mag;
    grav_ref[1] = sy / mag;
    grav_ref[2] = sz / mag;
    grav_ref_ok = true;
    ESP_LOGI(TAG, "upright ref: %.2f %.2f %.2f", grav_ref[0], grav_ref[1], grav_ref[2]);
}

// Angle (deg) between current gravity and the upright reference. 0 = upright,
// 90 = on its side, 180 = fully inverted.
static bool tilt_angle_deg(float *deg) {
    float x, y, z;
    if (!adxl345_read_g(&x, &y, &z)) return false;
    float mag = sqrtf(x * x + y * y + z * z);
    if (mag < 1e-3f) return false;
    float dot = (x * grav_ref[0] + y * grav_ref[1] + z * grav_ref[2]) / mag;
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    *deg = acosf(dot) * 57.2957795f;
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
        char buf[16];
        int n = snprintf(buf, sizeof(buf), "%d", last_ml);
        os_mbuf_append(ctxt->om, buf, n);
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

static void send_ml(int ml) {
    last_ml = ml;
    char buf[16];
    int n = snprintf(buf, sizeof(buf), "%d", ml);
    if (connected) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, n);
        ble_gatts_notify_custom(conn_handle, char_val_handle, om);
        ESP_LOGI(TAG, "sip sent: %d mL", ml);
    } else {
        ESP_LOGW(TAG, "sip detected but not connected: %d mL", ml);
    }
}

// Block until the load cell reading settles (small spread across reads) and
// return the averaged weight in grams. Bails out early if the cup gets tilted,
// so a drink starting mid-read doesn't stall this for the full timeout.
static bool read_stable_weight(float *grams_out) {
    float w[3];
    int idx = 0, count = 0;
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < (int64_t)SETTLE_TIMEOUT_MS * 1000) {
        float t;
        if (tilt_angle_deg(&t) && t >= TILT_DRINK_DEG) return false;
        float g;
        if (!hx711_grams(&g)) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        w[idx % 3] = g;
        idx++;
        count++;
        if (count >= 3) {
            float lo = w[0], hi = w[0];
            for (int i = 1; i < 3; i++) {
                if (w[i] < lo) lo = w[i];
                if (w[i] > hi) hi = w[i];
            }
            if (hi - lo <= WEIGHT_STABLE_G) {
                *grams_out = (w[0] + w[1] + w[2]) / 3.0f;
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

// Tilt-gated weight-delta detector. Holds a stable upright "baseline" weight;
// when the cup is tilted to drink and then returns upright, the drop in weight
// is the amount consumed. Tilts beyond TILT_POUROUT_DEG are treated as the cup
// being emptied out and produce no reading.
static void drink_detect_task(void *arg) {
    if (!adxl_ok || !hx711_ok) {
        ESP_LOGE(TAG, "drink detect disabled: adxl_ok=%d hx711_ok=%d", adxl_ok, hx711_ok);
        vTaskDelete(NULL);
        return;
    }
    if (!grav_ref_ok) {
        ESP_LOGW(TAG, "no upright reference; tilt measured from default pose");
    }

    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn);
    bool btn_last = true;  // pulled up: high when not pressed

    float baseline = 0.0f;
    while (!read_stable_weight(&baseline)) {
        ESP_LOGW(TAG, "waiting for a stable initial weight...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "baseline weight = %.0f g", baseline);

    bool drinking = false;
    float max_tilt = 0.0f;
    int64_t last_baseline_us = esp_timer_get_time();

    while (1) {
        // BOOT button = manual re-tare: re-learn upright pose and reset baseline.
        bool btn = gpio_get_level(BUTTON_GPIO);
        if (btn_last && !btn) {
            ESP_LOGI(TAG, "re-tare requested");
            capture_upright_ref();
            float w;
            if (read_stable_weight(&w)) {
                baseline = w;
                last_baseline_us = esp_timer_get_time();
                ESP_LOGI(TAG, "re-tared: baseline=%.0f g", baseline);
            } else {
                ESP_LOGW(TAG, "re-tare: weight not stable, baseline unchanged");
            }
            drinking = false;
            vTaskDelay(pdMS_TO_TICKS(300));  // debounce
        }
        btn_last = btn;

        float tilt;
        if (!tilt_angle_deg(&tilt)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (!drinking) {
            if (tilt >= TILT_DRINK_DEG) {
                drinking = true;
                max_tilt = tilt;
                ESP_LOGI(TAG, "drinking started (tilt=%.0f, baseline=%.0f g)",
                         tilt, baseline);
            } else if (tilt <= TILT_UPRIGHT_DEG &&
                       esp_timer_get_time() - last_baseline_us > BASELINE_REFRESH_US) {
                // Upright and idle: refresh baseline so refills/drift are tracked.
                float w;
                if (read_stable_weight(&w)) baseline = w;
                last_baseline_us = esp_timer_get_time();
            }
        } else {
            if (tilt > max_tilt) max_tilt = tilt;
            if (tilt <= TILT_UPRIGHT_DEG) {
                // Back upright: settle and compare against the baseline.
                float after;
                if (read_stable_weight(&after)) {
                    float delta = baseline - after;
                    if (max_tilt > TILT_POUROUT_DEG) {
                        ESP_LOGI(TAG, "pour-out ignored (max tilt=%.0f, delta=%.0f g)",
                                 max_tilt, delta);
                    } else if (delta >= MIN_SIP_G) {
                        send_ml((int)(delta / WATER_DENSITY_G_PER_ML + 0.5f));
                    } else {
                        ESP_LOGI(TAG, "no sip (delta=%.0f g, max tilt=%.0f)",
                                 delta, max_tilt);
                    }
                    baseline = after;
                    last_baseline_us = esp_timer_get_time();
                }
                drinking = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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
    capture_upright_ref();

    nimble_port_init();
    ble_att_set_preferred_mtu(247); // allow longer notifications
    ble_svc_gap_device_name_set("Things");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(nimble_host_task);
    xTaskCreate(drink_detect_task, "drink", 4096, NULL, 5, NULL);
}
