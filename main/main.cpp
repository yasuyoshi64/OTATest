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
#include "esp_ota_ops.h"
#include "cJSON.h"
#include "esp_log.h"

#include "main.hpp"

#define TAG "Application"
#define ROOT "/mnt"

#define BUFFSIZE 1024

static char ota_write_data[BUFFSIZE + 1] = { 0 };

// extern const uint8_t ca_cert_pem_start[] asm("_binary_ca_cert_pem_start");
// extern const uint8_t ca_cert_pem_end[] asm("_binary_ca_cert_pem_end");

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
    // ESP_LOGI(TAG, "ca_cert.pem\n%s", (const char*)ca_cert_pem_start);
    
    esp_err_t err;
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    esp_http_client_config_t config = {
        .url = updateuri,
        // .cert_pem = (char*)ca_cert_pem_start,
        .disable_auto_redirect = false,
        .event_handler = _http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };

    // シンプルなOTA
    // config.skip_cert_common_name_check = true;
    // esp_https_ota_config_t ota_config = {
    //     .http_config = &config,
    // };
    // esp_err_t ret = esp_https_ota(&ota_config);
    // if (ret == ESP_OK) {
    //     esp_restart();
    // } else {
    //     ESP_LOGW(TAG, "OTA failed...");        
    // }
    // while (1) {
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // バージョンチェック有OTA
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_ERROR_CHECK(esp_http_client_open(client, 0));
    esp_http_client_fetch_headers(client);

    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "update partition NULL");
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    int binary_file_length = 0;
    bool image_header_was_checked = false;
    while(1) {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        int http_status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "data_read=%d", data_read);
        ESP_LOGI(TAG, "http_status=%d", http_status);
        if (http_status == 302) {
            // リダイレクト
            esp_http_client_set_redirection(client);
            ESP_ERROR_CHECK(esp_http_client_open(client, 0));
            esp_http_client_fetch_headers(client);
            continue;
        }
        if (data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
        } else if (data_read > 0) {
            if (image_header_was_checked == false) {
                esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    // check current version with last invalid partition
                    if (last_invalid_app != NULL) {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                            ESP_LOGW(TAG, "New version is the same as invalid version.");
                            ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                            esp_http_client_close(client);
                            esp_http_client_cleanup(client);
                            vTaskDelay(NULL);
                        }
                    }

                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0) {
                        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                        esp_http_client_close(client);
                        esp_http_client_cleanup(client);
                        vTaskDelay(NULL);
                    }

                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        esp_http_client_close(client);
                        esp_http_client_cleanup(client);
                        esp_ota_abort(update_handle);
                        ESP_ERROR_CHECK(ESP_FAIL);
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");                    
                } else {
                    ESP_LOGE(TAG, "received package is not fit len");
                    esp_http_client_close(client);
                    esp_http_client_cleanup(client);
                    ESP_ERROR_CHECK(ESP_FAIL);
                }
            }
            err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                esp_ota_abort(update_handle);
                ESP_ERROR_CHECK(ESP_FAIL);
            }
            binary_file_length += data_read;
            ESP_LOGD(TAG, "Written image length %d", binary_file_length);
        } else if (data_read == 0) {
            if (errno == ECONNRESET || errno == ENOTCONN) {
                ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }            
        }
    }
    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);
    if (esp_http_client_is_complete_data_received(client) != true) {
        ESP_LOGE(TAG, "Error in receiving complete file");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        esp_ota_abort(update_handle);
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
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
