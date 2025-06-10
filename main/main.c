#include "dns_server.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "lwip/inet.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN

extern const char embedded_index_start[] asm("_binary_index_html_start");
extern const char embedded_index_end[] asm("_binary_index_html_end");

#define TAG "CLOCK"
#if CONFIG_IDF_TARGET_ESP32C3
#define LED_STRIP_GPIO 6
#else
#define LED_STRIP_GPIO 4
#endif
#define NUM_LEDS 128
#define MATRIX_WIDTH 16
#define MATRIX_HEIGHT 8

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

static esp_err_t index_get_handler(httpd_req_t *req) {
  size_t html_len = embedded_index_end - embedded_index_start;
  char *html = malloc(html_len + 1);
  if (!html) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory error");
    return ESP_FAIL;
  }
  memcpy(html, embedded_index_start, html_len);
  html[html_len] = '\0';
  // Calculer taille nécessaire
  int needed = snprintf(NULL, 0, html, digit_color[0][0], digit_color[0][1],
                        digit_color[0][2], digit_color[1][0],
                        digit_color[1][1], digit_color[1][2], digit_color[2][0],
                        digit_color[2][1], digit_color[2][2], digit_color[3][0],
                        digit_color[3][1], digit_color[3][2], bar_color[0],
                        bar_color[1], bar_color[2], digit_brightness,
                        digit_brightness, bar_brightness, bar_brightness,
                        blink_last_dot ? "checked" : "", torch_brightness,
                        torch_brightness);
  if (needed < 0) {
    free(html);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Format error");
    return ESP_FAIL;
  }
  char *response = malloc(needed + 1);
  if (!response) {
    free(html);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory error");
    return ESP_FAIL;
  }
  snprintf(response, needed + 1, html, digit_color[0][0], digit_color[0][1],
           digit_color[0][2], digit_color[1][0], digit_color[1][1],
           digit_color[1][2], digit_color[2][0], digit_color[2][1],
           digit_color[2][2], digit_color[3][0], digit_color[3][1],
           digit_color[3][2], bar_color[0], bar_color[1], bar_color[2],
           digit_brightness, digit_brightness, bar_brightness, bar_brightness,
           blink_last_dot ? "checked" : "", torch_brightness, torch_brightness);
  free(html);
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, response, needed);
  free(response);
  return ESP_OK;
}

static esp_err_t set_get_handler(httpd_req_t *req) {
  char buf[128];
  size_t len = httpd_req_get_url_query_len(req);
  if (len < 1) {
    httpd_resp_send(req, "No query received", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  httpd_req_get_url_query_str(req, buf, len + 1);

  char val[16];

  // Lecture du timestamp client (exemple déjà expliqué précédemment)
  if (httpd_query_key_value(buf, "ts", val, sizeof(val)) == ESP_OK) {
    uint64_t client_ts = atoll(val);
    ESP_LOGI(TAG, "CLIENT_TIME: %llu", client_ts);
    set_system_time(client_ts);
  }

  // Lecture des couleurs des chiffres
  for (int i = 0; i < 4; i++) {
    char key[5];
    snprintf(key, sizeof(key), "d%dr", i);
    if (httpd_query_key_value(buf, key, val, sizeof(val)) == ESP_OK)
      digit_color[i][0] = atoi(val);
    snprintf(key, sizeof(key), "d%dg", i);
    if (httpd_query_key_value(buf, key, val, sizeof(val)) == ESP_OK)
      digit_color[i][1] = atoi(val);
    snprintf(key, sizeof(key), "d%db", i);
    if (httpd_query_key_value(buf, key, val, sizeof(val)) == ESP_OK)
      digit_color[i][2] = atoi(val);
  }
  if (httpd_query_key_value(buf, "dr", val, sizeof(val)) == ESP_OK)
    for (int i = 0; i < 4; i++)
      digit_color[i][0] = atoi(val);
  if (httpd_query_key_value(buf, "dg", val, sizeof(val)) == ESP_OK)
    for (int i = 0; i < 4; i++)
      digit_color[i][1] = atoi(val);
  if (httpd_query_key_value(buf, "db", val, sizeof(val)) == ESP_OK)
    for (int i = 0; i < 4; i++)
      digit_color[i][2] = atoi(val);

  // Lecture des composantes R/G/B barre
  if (httpd_query_key_value(buf, "br", val, sizeof(val)) == ESP_OK)
    bar_color[0] = atoi(val);
  if (httpd_query_key_value(buf, "bg", val, sizeof(val)) == ESP_OK)
    bar_color[1] = atoi(val);
  if (httpd_query_key_value(buf, "bb", val, sizeof(val)) == ESP_OK)
    bar_color[2] = atoi(val);

  // Lecture des luminosités
  if (httpd_query_key_value(buf, "bri", val, sizeof(val)) == ESP_OK)
    digit_brightness = atoi(val);
  if (httpd_query_key_value(buf, "bar_bri", val, sizeof(val)) == ESP_OK)
    bar_brightness = atoi(val);
  if (httpd_query_key_value(buf, "torch_bri", val, sizeof(val)) == ESP_OK)
    torch_brightness = atoi(val);
  if (httpd_query_key_value(buf, "blink", val, sizeof(val)) == ESP_OK)
    blink_last_dot = atoi(val);
  if (httpd_query_key_value(buf, "torch", val, sizeof(val)) == ESP_OK)
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

  httpd_resp_set_status(req, "204 No Content");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t index_uri = {
        .uri = "/", .method = HTTP_GET, .handler = index_get_handler};
    httpd_register_uri_handler(server, &index_uri);
    httpd_uri_t set_uri = {
        .uri = "/set", .method = HTTP_GET, .handler = set_get_handler};
    httpd_register_uri_handler(server, &set_uri);
  }
  return server;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event =
        (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac),
             event->aid);
  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event =
        (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d, reason=%d",
             MAC2STR(event->mac), event->aid, event->reason);
  }
}

static void wifi_init_softap(void) {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, NULL));

  wifi_config_t wifi_config = {
      .ap = {.ssid = EXAMPLE_ESP_WIFI_SSID,
             .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
             .password = EXAMPLE_ESP_WIFI_PASS,
             .max_connection = EXAMPLE_MAX_STA_CONN,
             .authmode = WIFI_AUTH_WPA_WPA2_PSK},
  };
  if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  esp_netif_ip_info_t ip_info;
  esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"),
                        &ip_info);

  char ip_addr[16];
  inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
  ESP_LOGI(TAG, "Set up softAP with IP: %s", ip_addr);

  ESP_LOGI(TAG, "wifi_init_softap finished. SSID:'%s' password:'%s'",
           EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

#ifdef CONFIG_ESP_ENABLE_DHCP_CAPTIVEPORTAL
static void dhcp_set_captiveportal_url(void) {
  // get the IP of the access point to redirect to
  esp_netif_ip_info_t ip_info;
  esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"),
                        &ip_info);

  char ip_addr[16];
  inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
  ESP_LOGI(TAG, "Set up softAP with IP: %s", ip_addr);

  // turn the IP into a URI
  char *captiveportal_uri = (char *)malloc(32 * sizeof(char));
  assert(captiveportal_uri && "Failed to allocate captiveportal_uri");
  strcpy(captiveportal_uri, "http://");
  strcat(captiveportal_uri, ip_addr);

  // get a handle to configure DHCP with
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

  // set the DHCP option 114
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(netif));
  ESP_ERROR_CHECK(esp_netif_dhcps_option(
      netif, ESP_NETIF_OP_SET, ESP_NETIF_CAPTIVEPORTAL_URI, captiveportal_uri,
      strlen(captiveportal_uri)));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(netif));
}
#endif // CONFIG_ESP_ENABLE_DHCP_CAPTIVEPORTAL

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
  esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
  esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
  esp_log_level_set("httpd_parse", ESP_LOG_ERROR);

  // Initialize networking stack
  ESP_ERROR_CHECK(esp_netif_init());

  // Create default event loop needed by the  main app
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  // Initialize Wi-Fi including netif with default config
  esp_netif_create_default_wifi_ap();

  // Initialise ESP32 in SoftAP mode
  wifi_init_softap();

// Configure DNS-based captive portal, if configured
#ifdef CONFIG_ESP_ENABLE_DHCP_CAPTIVEPORTAL
  dhcp_set_captiveportal_url();
#endif

  // Start the server for the first time
  start_webserver();
  // Start the DNS server that will redirect all queries to the softAP IP
  dns_server_config_t config = DNS_SERVER_CONFIG_SINGLE(
      "*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
  start_dns_server(&config);
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
