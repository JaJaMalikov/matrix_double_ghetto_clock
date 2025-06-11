#include "wifi.h"
#include "display.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>
#include <time.h>

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD

static const char *TAG = "WIFI";
static httpd_handle_t server = NULL;

static void obtain_time(void)
{
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    esp_netif_sntp_start();
    int retry = 0;
    while (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(2000)) == ESP_ERR_TIMEOUT && ++retry < 10) {
        ESP_LOGI(TAG, "Waiting for time sync...");
    }
    esp_netif_sntp_deinit();
}

static esp_err_t index_get_handler(httpd_req_t *req);
static esp_err_t set_get_handler(httpd_req_t *req);

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t s = NULL;
    if (httpd_start(&s, &config) == ESP_OK) {
        httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_get_handler};
        httpd_register_uri_handler(s, &index_uri);
        httpd_uri_t set_uri = {.uri = "/set", .method = HTTP_GET, .handler = set_get_handler};
        httpd_register_uri_handler(s, &set_uri);
    }
    return s;
}

static void set_system_time(uint64_t timestamp_ms)
{
    time_t sec = timestamp_ms / 1000;
    suseconds_t usec = (timestamp_ms % 1000) * 1000;
    struct timeval tv = {.tv_sec = sec, .tv_usec = usec};
    settimeofday(&tv, NULL);
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    extern const char embedded_index_start[] asm("_binary_index_html_start");
    extern const char embedded_index_end[] asm("_binary_index_html_end");
    size_t html_len = embedded_index_end - embedded_index_start;
    char *html = malloc(html_len + 1);
    if (!html) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory error");
        return ESP_FAIL;
    }
    memcpy(html, embedded_index_start, html_len);
    html[html_len] = '\0';
    int needed = snprintf(NULL, 0, html, digit_color[0][0], digit_color[0][1], digit_color[0][2],
                          digit_color[1][0], digit_color[1][1], digit_color[1][2],
                          digit_color[2][0], digit_color[2][1], digit_color[2][2],
                          digit_color[3][0], digit_color[3][1], digit_color[3][2],
                          bar_color[0], bar_color[1], bar_color[2], digit_brightness,
                          digit_brightness, bar_brightness, bar_brightness,
                          blink_last_dot ? "checked" : "", torch_brightness, torch_brightness);
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
    snprintf(response, needed + 1, html, digit_color[0][0], digit_color[0][1], digit_color[0][2],
             digit_color[1][0], digit_color[1][1], digit_color[1][2], digit_color[2][0], digit_color[2][1],
             digit_color[2][2], digit_color[3][0], digit_color[3][1], digit_color[3][2], bar_color[0],
             bar_color[1], bar_color[2], digit_brightness, digit_brightness, bar_brightness, bar_brightness,
             blink_last_dot ? "checked" : "", torch_brightness, torch_brightness);
    free(html);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, needed);
    free(response);
    return ESP_OK;
}

static esp_err_t set_get_handler(httpd_req_t *req)
{
    char buf[128];
    size_t len = httpd_req_get_url_query_len(req);
    if (len < 1) {
        httpd_resp_send(req, "No query received", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_req_get_url_query_str(req, buf, len + 1);

    char val[16];

    if (httpd_query_key_value(buf, "ts", val, sizeof(val)) == ESP_OK) {
        uint64_t client_ts = atoll(val);
        ESP_LOGI(TAG, "CLIENT_TIME: %llu", client_ts);
        set_system_time(client_ts);
    }

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

    if (httpd_query_key_value(buf, "br", val, sizeof(val)) == ESP_OK)
        bar_color[0] = atoi(val);
    if (httpd_query_key_value(buf, "bg", val, sizeof(val)) == ESP_OK)
        bar_color[1] = atoi(val);
    if (httpd_query_key_value(buf, "bb", val, sizeof(val)) == ESP_OK)
        bar_color[2] = atoi(val);

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

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("clock", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
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
        nvs_commit(nvs);
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "NVS open (RW) failed (%s)", esp_err_to_name(err));
    }

    ESP_LOGI(TAG,
             "SET: d0=%d,%d,%d d1=%d,%d,%d d2=%d,%d,%d d3=%d,%d,%d | bar=%d,%d,%d | bri=%d bar_bri=%d torch=%d",
             digit_color[0][0], digit_color[0][1], digit_color[0][2], digit_color[1][0], digit_color[1][1], digit_color[1][2],
             digit_color[2][0], digit_color[2][1], digit_color[2][2], digit_color[3][0], digit_color[3][1], digit_color[3][2],
             bar_color[0], bar_color[1], bar_color[2], digit_brightness, bar_brightness, torch_brightness);

    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
        if (!server) {
            server = start_webserver();
        }
        obtain_time();
    }
}

void wifi_init_sta(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

