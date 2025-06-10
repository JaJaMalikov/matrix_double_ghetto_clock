#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define TAG "CLOCK"
#if CONFIG_IDF_TARGET_ESP32C3
#define LED_STRIP_GPIO 6
#else
#define LED_STRIP_GPIO 4
#endif
#define NUM_LEDS 128
#define MATRIX_WIDTH 16
#define MATRIX_HEIGHT 8

#define CONFIG_SERVICE_UUID 0xFFF0
#define CONFIG_CHAR_UUID 0xFFF1

static uint16_t config_service_handle;
static uint16_t config_char_handle;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint16_t service_uuid = CONFIG_SERVICE_UUID;
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = (uint8_t *)&service_uuid,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static esp_gatt_srvc_id_t service_id = {
    .is_primary = true,
    .id = {.uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = CONFIG_SERVICE_UUID}}, .inst_id = 0},
};

static esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = CONFIG_CHAR_UUID}};

static bool query_get_value(const char *buf, const char *key, char *out,
                            size_t len);
static void apply_query_config(const char *buf);
static void ble_init(void);

static uint8_t digit_color[4][3] = {{128, 64, 0}, {128, 64, 0},
                                    {128, 64, 0}, {128, 64, 0}};
static uint8_t bar_color[3] = {64, 128, 0};
static uint8_t digit_brightness = 255;
static uint8_t bar_brightness = 255;
static uint8_t torch_brightness = 255;
static bool torch_mode = false;
static bool blink_last_dot = false;

static led_strip_handle_t led_strip;
static uint8_t led_data[NUM_LEDS * 3];

static int xy_to_index(int x, int y) {
  int matrix_id = (x >= 8) ? 0 : 1;
  int local_x = x % 8;
  int local_y = 7 - y;
  int index_in_matrix = local_y * 8 + (7 - local_x);
  return matrix_id * 64 + index_in_matrix;
}

const uint8_t font3x5[][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111}, {0b010, 0b110, 0b010, 0b010, 0b111},
    {0b111, 0b001, 0b111, 0b100, 0b111}, {0b111, 0b001, 0b111, 0b001, 0b111},
    {0b101, 0b101, 0b111, 0b001, 0b001}, {0b111, 0b100, 0b111, 0b001, 0b111},
    {0b111, 0b100, 0b111, 0b101, 0b111}, {0b111, 0b001, 0b010, 0b010, 0b010},
    {0b111, 0b101, 0b111, 0b101, 0b111}, {0b111, 0b101, 0b111, 0b001, 0b111},
};

static void apply_brightness(uint8_t base_r, uint8_t base_g, uint8_t base_b,
                             uint8_t bri, uint8_t *out_r, uint8_t *out_g,
                             uint8_t *out_b) {
  *out_r = (base_r * bri) / 255;
  *out_g = (base_g * bri) / 255;
  *out_b = (base_b * bri) / 255;
}
static void update_led_strip() {
  for (int i = 0; i < NUM_LEDS; i++) {
    int idx = i * 3;
    led_strip_set_pixel(led_strip, i, led_data[idx + 1], led_data[idx],
                        led_data[idx + 2]);
  }
  led_strip_refresh(led_strip);
}
static int last_digits[4] = {-1, -1, -1, -1};

static void draw_digit(int x, int y, uint8_t digit, uint8_t r, uint8_t g,
                       uint8_t b) {
  if (digit > 9)
    return;
  uint8_t adj_r, adj_g, adj_b;
  apply_brightness(r, g, b, digit_brightness, &adj_r, &adj_g, &adj_b);
  for (int row = 0; row < 5; row++) {
    uint8_t rowBits = font3x5[digit][row];
    for (int col = 0; col < 3; col++) {
      if (rowBits & (1 << (2 - col))) {
        int px = x + col;
        int py = y + row;
        if (px < MATRIX_WIDTH && py < MATRIX_HEIGHT) {
          int index = xy_to_index(px, py) * 3;
          led_data[index] = adj_g;
          led_data[index + 1] = adj_r;
          led_data[index + 2] = adj_b;
        }
      }
    }
  }
}

int x_coords[] = {9, 13, 0, 4};

// Fonction d'animation de chute d'un chiffre
static void animate_digit_fall(int digit_index, int x, int y, uint8_t oldDigit,
                               uint8_t newDigit, uint8_t r, uint8_t g,
                               uint8_t b) {
  uint8_t backup[NUM_LEDS * 3];
  uint8_t adj_r, adj_g, adj_b;
  apply_brightness(r, g, b, digit_brightness, &adj_r, &adj_g, &adj_b);
  memcpy(backup, led_data, sizeof(led_data));

  for (int drop = 0; drop <= MATRIX_HEIGHT; drop++) {
    memcpy(led_data, backup, sizeof(led_data));
    for (int row = 0; row < 5; row++) {
      uint8_t rowBits = font3x5[oldDigit][row];
      for (int col = 0; col < 3; col++) {
        if (rowBits & (1 << (2 - col))) {
          int px = x + col;
          int py = y + row + drop;
          if (py >= 0 && py < MATRIX_HEIGHT) {
            int index = xy_to_index(px, py) * 3;
            led_data[index] = adj_g;
            led_data[index + 1] = adj_r;
            led_data[index + 2] = adj_b;
          }
        }
      }
    }
    for (int j = 0; j < 4; j++) {
      if (j == digit_index)
        continue;
      draw_digit(x_coords[j], 1, last_digits[j], digit_color[j][0],
                 digit_color[j][1], digit_color[j][2]);
    }
    update_led_strip();
    vTaskDelay(pdMS_TO_TICKS(40));
  }
  for (int rise = MATRIX_HEIGHT; rise >= 0; rise--) {
    memset(led_data, 0, sizeof(led_data));
    for (int row = 0; row < 5; row++) {
      uint8_t rowBits = font3x5[newDigit][row];
      for (int col = 0; col < 3; col++) {
        if (rowBits & (1 << (2 - col))) {
          int px = x + col;
          int py = y + row - rise;
          if (py >= 0 && py < MATRIX_HEIGHT) {
            int index = xy_to_index(px, py) * 3;
            led_data[index] = adj_g;
            led_data[index + 1] = adj_r;
            led_data[index + 2] = adj_b;
          }
        }
      }
    }
    for (int j = 0; j < 4; j++) {
      if (j == digit_index)
        continue;
      draw_digit(x_coords[j], 1, last_digits[j], digit_color[j][0],
                 digit_color[j][1], digit_color[j][2]);
    }
    update_led_strip();
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

static void draw_seconds_bar(int seconds, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t adj_r, adj_g, adj_b;
  apply_brightness(r, g, b, bar_brightness, &adj_r, &adj_g, &adj_b);
  int tick = (seconds * MATRIX_WIDTH) / 60;
  for (int i = 0; i < MATRIX_WIDTH; i++) {
    int x = i;
    int index = xy_to_index(x, MATRIX_HEIGHT - 1) * 3;
    bool on = i < tick;
    if (blink_last_dot && i == MATRIX_WIDTH - 1 && tick == MATRIX_WIDTH - 1) {
      on = (seconds % 2) != 0;
    }
    if (on) {
      led_data[index] = adj_g;
      led_data[index + 1] = adj_r;
      led_data[index + 2] = adj_b;
    } else {
      led_data[index] = led_data[index + 1] = led_data[index + 2] = 0;
    }
  }
}

static void clock_task(void *arg) {
  while (1) {
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    int digits[] = {

        timeinfo.tm_min / 10, timeinfo.tm_min % 10, timeinfo.tm_hour / 10,
        timeinfo.tm_hour % 10};
    memset(led_data, 0, sizeof(led_data));
    if (torch_mode) {
      uint8_t r, g, b;
      apply_brightness(255, 255, 255, torch_brightness, &r, &g, &b);
      for (int i = 0; i < NUM_LEDS; i++) {
        int idx = i * 3;
        led_data[idx] = g;
        led_data[idx + 1] = r;
        led_data[idx + 2] = b;
      }
    } else {
      draw_seconds_bar(timeinfo.tm_sec, bar_color[0], bar_color[1], bar_color[2]);
      for (int i = 0; i < 4; i++) {
        if (digits[i] != last_digits[i]) {
          animate_digit_fall(i, x_coords[i], 1, last_digits[i], digits[i],
                             digit_color[i][0], digit_color[i][1],
                             digit_color[i][2]);
          last_digits[i] = digits[i];
        } else {
          draw_digit(x_coords[i], 1, digits[i], digit_color[i][0],
                     digit_color[i][1], digit_color[i][2]);
        }
      }
    }
    update_led_strip();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void set_system_time(uint64_t timestamp_ms) {
  time_t sec = timestamp_ms / 1000;
  suseconds_t usec = (timestamp_ms % 1000) * 1000;
  struct timeval tv = {.tv_sec = sec, .tv_usec = usec};
  settimeofday(&tv, NULL);
}

static bool query_get_value(const char *buf, const char *key, char *out,
                           size_t len) {
  const char *p = strstr(buf, key);
  if (!p)
    return false;
  p += strlen(key);
  if (*p != '=')
    return false;
  p++;
  const char *end = strchr(p, '&');
  size_t l = end ? (size_t)(end - p) : strlen(p);
  if (l >= len)
    l = len - 1;
  memcpy(out, p, l);
  out[l] = '\0';
  return true;
}

static void apply_query_config(const char *buf) {
  char val[16];

  if (query_get_value(buf, "ts", val, sizeof(val))) {
    uint64_t client_ts = atoll(val);
    ESP_LOGI(TAG, "CLIENT_TIME: %llu", client_ts);
    set_system_time(client_ts);
  }

  // Lecture des couleurs des chiffres
  for (int i = 0; i < 4; i++) {
    char key[5];
    snprintf(key, sizeof(key), "d%dr", i);
    if (query_get_value(buf, key, val, sizeof(val)))
      digit_color[i][0] = atoi(val);
    snprintf(key, sizeof(key), "d%dg", i);
    if (query_get_value(buf, key, val, sizeof(val)))
      digit_color[i][1] = atoi(val);
    snprintf(key, sizeof(key), "d%db", i);
    if (query_get_value(buf, key, val, sizeof(val)))
      digit_color[i][2] = atoi(val);
  }
  if (query_get_value(buf, "dr", val, sizeof(val)))
    for (int i = 0; i < 4; i++)
      digit_color[i][0] = atoi(val);
  if (query_get_value(buf, "dg", val, sizeof(val)))
    for (int i = 0; i < 4; i++)
      digit_color[i][1] = atoi(val);
  if (query_get_value(buf, "db", val, sizeof(val)))
    for (int i = 0; i < 4; i++)
      digit_color[i][2] = atoi(val);

  // Lecture des composantes R/G/B barre
  if (query_get_value(buf, "br", val, sizeof(val)))
    bar_color[0] = atoi(val);
  if (query_get_value(buf, "bg", val, sizeof(val)))
    bar_color[1] = atoi(val);
  if (query_get_value(buf, "bb", val, sizeof(val)))
    bar_color[2] = atoi(val);

  // Lecture des luminosités
  if (query_get_value(buf, "bri", val, sizeof(val)))
    digit_brightness = atoi(val);
  if (query_get_value(buf, "bar_bri", val, sizeof(val)))
    bar_brightness = atoi(val);
  if (query_get_value(buf, "torch_bri", val, sizeof(val)))
    torch_brightness = atoi(val);
  if (query_get_value(buf, "blink", val, sizeof(val)))
    blink_last_dot = atoi(val);
  if (query_get_value(buf, "torch", val, sizeof(val)))
    torch_mode = atoi(val);

  // 1) Ouvrir la NVS en lecture/écriture
  nvs_handle_t nvs;
  esp_err_t err = nvs_open("clock", NVS_READWRITE, &nvs);
  if (err == ESP_OK) {
    // 2) Stocker chaque valeur (u8)
    for (int i = 0; i < 4; i++) {
      char key[5];
      snprintf(key, sizeof(key), "d%dr", i);
      nvs_set_u8(nvs, key, digit_color[i][0]);
      snprintf(key, sizeof(key), "d%dg", i);
      nvs_set_u8(nvs, key, digit_color[i][1]);
      snprintf(key, sizeof(key), "d%db", i);
      nvs_set_u8(nvs, key, digit_color[i][2]);
    }

    nvs_set_u8(nvs, "br", bar_color[0]);
    nvs_set_u8(nvs, "bg", bar_color[1]);
    nvs_set_u8(nvs, "bb", bar_color[2]);

    nvs_set_u8(nvs, "bri", digit_brightness);
    nvs_set_u8(nvs, "bar_bri", bar_brightness);
    nvs_set_u8(nvs, "tbri", torch_brightness);
    nvs_set_u8(nvs, "blink", blink_last_dot);

    // 3) Valider l’écriture
    nvs_commit(nvs);
    nvs_close(nvs);
  } else {
    ESP_LOGW(TAG, "NVS open (RW) failed (%s)", esp_err_to_name(err));
  }

  ESP_LOGI(TAG,
           "SET: d0=%d,%d,%d d1=%d,%d,%d d2=%d,%d,%d d3=%d,%d,%d | bar=%d,%d,%d "
           "| bri=%d bar_bri=%d torch=%d" , digit_color[0][0], digit_color[0][1],
           digit_color[0][2], digit_color[1][0], digit_color[1][1],
           digit_color[1][2], digit_color[2][0], digit_color[2][1],
           digit_color[2][2], digit_color[3][0], digit_color[3][1],
           digit_color[3][2], bar_color[0], bar_color[1], bar_color[2],
           digit_brightness, bar_brightness, torch_brightness);
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
  if (event == ESP_GATTS_WRITE_EVT && param->write.handle == config_char_handle &&
      !param->write.is_prep) {
    char data[128];
    int len = param->write.len < sizeof(data) - 1 ? param->write.len : sizeof(data) - 1;
    memcpy(data, param->write.value, len);
    data[len] = '\0';
    apply_query_config(data);
  } else if (event == ESP_GATTS_REG_EVT) {
    esp_ble_gap_set_device_name("ClockConfig");
    esp_ble_gap_config_adv_data(&adv_data);
    esp_ble_gatts_create_service(gatts_if, &service_id, 4);
  } else if (event == ESP_GATTS_CREATE_EVT) {
    config_service_handle = param->create.service_handle;
    esp_ble_gatts_start_service(config_service_handle);
    esp_ble_gatts_add_char(config_service_handle, &char_uuid, ESP_GATT_PERM_WRITE,
                           ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
  } else if (event == ESP_GATTS_ADD_CHAR_EVT) {
    config_char_handle = param->add_char.attr_handle;
  } else if (event == ESP_GATTS_CONNECT_EVT) {
    esp_ble_gap_stop_advertising();
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    esp_ble_gap_start_advertising(&adv_params);
  }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
    esp_ble_gap_start_advertising(&adv_params);
  }
}

static void ble_init(void) {
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());

  esp_ble_gap_register_callback(gap_event_handler);
  esp_ble_gatts_register_callback(gatts_event_handler);
  esp_ble_gatts_app_register(0);
}

void app_main(void) {
  // 1) Initialisation NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // Si la partition NVS est corrompue ou ancienne, on la reformate
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // 2) Ouverture en lecture seule
  nvs_handle_t nvs;
  err = nvs_open("clock", NVS_READONLY, &nvs);
  if (err == ESP_OK) {
    // Lecture de chaque paramètre (u8). Si non trouvée, on laisse la valeur par
    // défaut.
    uint8_t tmp;
    for (int i = 0; i < 4; i++) {
      char key[5];
      snprintf(key, sizeof(key), "d%dr", i);
      if (nvs_get_u8(nvs, key, &tmp) == ESP_OK)
        digit_color[i][0] = tmp;
      snprintf(key, sizeof(key), "d%dg", i);
      if (nvs_get_u8(nvs, key, &tmp) == ESP_OK)
        digit_color[i][1] = tmp;
      snprintf(key, sizeof(key), "d%db", i);
      if (nvs_get_u8(nvs, key, &tmp) == ESP_OK)
        digit_color[i][2] = tmp;
    }

    if (nvs_get_u8(nvs, "br", &tmp) == ESP_OK)
      bar_color[0] = tmp;
    if (nvs_get_u8(nvs, "bg", &tmp) == ESP_OK)
      bar_color[1] = tmp;
    if (nvs_get_u8(nvs, "bb", &tmp) == ESP_OK)
      bar_color[2] = tmp;

    if (nvs_get_u8(nvs, "bri", &tmp) == ESP_OK)
      digit_brightness = tmp;
    if (nvs_get_u8(nvs, "bar_bri", &tmp) == ESP_OK)
      bar_brightness = tmp;
    if (nvs_get_u8(nvs, "tbri", &tmp) == ESP_OK)
      torch_brightness = tmp;
    if (nvs_get_u8(nvs, "blink", &tmp) == ESP_OK)
      blink_last_dot = tmp;

    nvs_close(nvs);
  } else {
    // Si la partition n’existe pas encore, on l’ignore (valeurs par défaut
    // resteront)
    ESP_LOGW(TAG, "NVS open failed (%s), using defaults", esp_err_to_name(err));
  }

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ble_init();
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
  led_strip_config_t strip_config = {.strip_gpio_num = LED_STRIP_GPIO,
                                     .max_leds = NUM_LEDS,
                                     .led_model = LED_MODEL_WS2812,
                                     .flags.invert_out = false};
  led_strip_rmt_config_t rmt_config = {.clk_src = RMT_CLK_SRC_DEFAULT,
                                       .resolution_hz = 10 * 1000 * 1000,
                                       .mem_block_symbols = 64};
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  ESP_ERROR_CHECK(led_strip_clear(led_strip));
  xTaskCreate(clock_task, "clock_task", 4096, NULL, 5, NULL);
}
