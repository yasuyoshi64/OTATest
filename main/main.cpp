#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <map>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "esp_log.h"

#include "main.hpp"

#define TAG "Application"
#define ROOT "/mnt"

extern const uint8_t ca_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t ca_cert_pem_end[] asm("_binary_ca_cert_pem_end");

Application app;

// メッセージ種別 (メッセージキュー用)
enum class AppMessage {
    WIFIConnection,     // Wi-Fi接続
    WIFIDisconnection,  // Wi-Fi切断
    OTA,                // ファームウェアアップデート
    Quit                // 終了
};

Application::Application() {
    m_LedState = false;
    m_xHandle = NULL;
    m_xHandleOTA = NULL;
    m_xQueue = NULL;
    m_isWiFi = false;
}

// 初期化
void Application::init() {
    ESP_LOGI(TAG, "Init(S)");

    // LED初期化
    gpio_reset_pin((gpio_num_t)CONFIG_LED_PIN);
    gpio_set_direction((gpio_num_t)CONFIG_LED_PIN, GPIO_MODE_OUTPUT);
    led(0);

    // タスク作成
    xTaskCreate(Application::app_task, TAG, configMINIMAL_STACK_SIZE * 2, (void*)this, tskIDLE_PRIORITY, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(AppMessage));

    // SDカード初期化
    m_sd_card.init(ROOT);
    m_sd_card.setMountCallback(mountFunc, this);

    // Wi-Fi初期化
    m_wifi.init(wifiConnectFunc, this);

    ESP_LOGI(TAG, "Init(E)");
}

// タスク
void Application::app_task(void* arg) {
    Application* pThis = (Application*)arg;
    AppMessage msg;
    bool loop = true;
    while(loop) {
        // メッセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case AppMessage::WIFIConnection:    // Wi-Fi接続
                    pThis->wifiConnection();
                    break;
                case AppMessage::WIFIDisconnection: // Wi-Fi切断
                    pThis->wifiDisconnection();
                    break;
                case AppMessage::OTA:               // ファームウェアアップデート
                    pThis->ota();
                    break;
                case AppMessage::Quit:              // 終了
                    loop = false;
                    break;
            }
        }
    }
    // 終了処理
    vTaskDelete(NULL);
}

// LED点灯制御
//  state   : 1=点灯, 0=消灯
void Application::led(int state) {
    m_LedState = state;
    gpio_set_level((gpio_num_t)CONFIG_LED_PIN, m_LedState);
}

// SDカードマウントコールバック
void Application::mountFunc(bool isMount, void* context) {
    ESP_LOGI(TAG, "SD Card mount : %d", isMount);
    Application* pThis = (Application*)context;
    if (isMount && pThis->getConfig(ROOT)) {
        AppMessage msg = AppMessage::WIFIConnection;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    } else {
        AppMessage msg = AppMessage::WIFIDisconnection;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    }
}

// Wi-Fi接続コールバック
void Application::wifiConnectFunc(bool isConnect, void* context) {
    Application* pThis = (Application*)context;
    if (isConnect) {
        pThis->m_isWiFi = true;
        const char* ipAddress = pThis->m_wifi.getIPAddress();
        ESP_LOGI(TAG, "IP Address: %s", ipAddress);
        pThis->led(0);
        // OTA
        AppMessage msg = AppMessage::OTA;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    } else {
        ESP_LOGW(TAG, "wi-fi disconnect");
        pThis->m_isWiFi = false;
    }
}

// Wi-Fi接続
void Application::wifiConnection() {
    wifiDisconnection();
    if (m_configMap.find("ssid") == m_configMap.end() || m_configMap.find("pass") == m_configMap.end())
        return; // CONFIGファイルにssidまたはpassの設定がない
    const char* ssid = m_configMap["ssid"].c_str();
    const char* pass = m_configMap["pass"].c_str();
    m_wifi.connect(ssid, pass);
    led(1);
}

// Wi-Fi切断
void Application::wifiDisconnection() {
    m_wifi.disconnect();
    led(0);
}

void Application::ota() {
    if (m_xHandleOTA != NULL)
        return;
    xTaskCreate(Application::checkOTA, TAG, 8192, (void*)this, tskIDLE_PRIORITY, &m_xHandle);
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

// アップデートチェック
void Application::checkOTA(void* arg) {
    Application *pThis = (Application*)arg;
    const char* updateuri = pThis->m_configMap["updateuri"].c_str();
    ESP_LOGI(TAG, "update uri = %s", updateuri);

    ESP_LOGI(TAG, "ca_cert.pem\n%s", (const char*)ca_cert_pem_start);

    esp_http_client_config_t config = {
        .url = updateuri,
        // .cert_pem = (char*)ca_cert_pem_start,
        .event_handler = _http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    config.skip_cert_common_name_check = true;
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGW(TAG, "OTA failed...");        
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();     // Flash初期化  (お約束のようなもの)
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    app.init();
}
