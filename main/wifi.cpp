#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "wifi.hpp"

#define TAG "Wi-Fi"

// メッセージ種別 (メッセージキュー用)
enum class WiFiMessage {
    Init,       // 初期化
    Connect,    // 接続
    Disconnect, // 切断
    Quit        // 終了
};

WiFi::WiFi() {
    clear();
}

void WiFi::clear() {
    m_func = NULL;
    m_funcContext = NULL;
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_ssid = "";
    m_pass = "";
    m_netif_tcpstack = NULL;
    m_ipAddress = "";
}

void WiFi::init(CallbackWiFiFunction func, void* context) {
    ESP_LOGI(TAG, "Init(S)");

    m_func = func;
    m_funcContext = context;

    // タスク作成
    xTaskCreate(WiFi::task, TAG, configMINIMAL_STACK_SIZE * 2, (void*)this, tskIDLE_PRIORITY, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(WiFiMessage));

    // 初期化用メッセージポスト
    WiFiMessage msg = WiFiMessage::Init;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);

    ESP_LOGI(TAG, "Init(E)");
}

// 接続要求
bool WiFi::connect(const char* ssid, const char* pass) {
    if (m_netif_tcpstack != NULL)
        disconnect();
    m_ssid = ssid;
    m_pass = pass;

    // Wi-Fi APへ接続
    wifi_config_t wifi_config = {
        .sta = {
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold = {
                .rssi = (int8_t)-127,
                .authmode = (wifi_auth_mode_t)WIFI_AUTH_WPA2_PSK
            }
        }
    };
    if (m_ssid.size() <= 32) {
        for(int i=0; i<m_ssid.size(); i++) {
            wifi_config.sta.ssid[i] = m_ssid[i];
        }        
    } else {
        ESP_LOGE(TAG, "wifi ssid over 32 lenght");
        return false;
    }
    if (m_pass.size() <= 64) {
        for(int i=0; i<m_pass.size(); i++) {
            wifi_config.sta.password[i] = m_pass[i];
        }
    } else {
        ESP_LOGE(TAG, "wifi pass over 64 lenght");
        return false;
    }
    ESP_LOGI("Wi-Fi", "Connecting to %s (%s)...", wifi_config.sta.ssid, wifi_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t ret = esp_wifi_connect();     // Wi-Fi APへ接続
    if (ret != ESP_OK) {
        ESP_LOGE("Wi-Fi", "WiFi connect failed! ret:%x", ret);
        return false;
    }
    return true;
}

// 切断要求
void WiFi::disconnect() {
    if (m_netif_tcpstack == NULL)
        return;
    esp_wifi_disconnect();
}

// タスク
void WiFi::task(void* arg) {
    WiFi* pThis = (WiFi*)arg;
    WiFiMessage msg;
    bool loop = true;
    while(loop) {
        // メツセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case WiFiMessage::Init:         // 初期化処理
                    pThis->wifiInit();
                    break;
                case WiFiMessage::Connect:      // 接続
                    pThis->wifiConnect();
                    break;
                case WiFiMessage::Disconnect:   // 切断
                    pThis->wifiDisconnect();
                    break;
                case WiFiMessage::Quit:         // 終了
                    loop = false;
                    break;
            }
        }
    }
    // 終了処理
    pThis->disconnect();
    pThis->clear();
    vTaskDelete(NULL);
}

// Wi-Fi初期化
void WiFi::wifiInit() {
    // Wi-Fi初期化
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Wi-Fi設定
    esp_netif_inherent_config_t esp_netif_config = {
        .flags = (esp_netif_flags_t)(ESP_NETIF_FLAG_GARP | ESP_NETIF_DHCP_CLIENT | ESP_NETIF_FLAG_MLDV6_REPORT | ESP_NETIF_FLAG_EVENT_IP_MODIFIED),
        .mac = { },
        .ip_info = NULL,
        .get_ip_event = IP_EVENT_STA_GOT_IP,
        .lost_ip_event = IP_EVENT_STA_LOST_IP,
        .if_key = "WIFI_STA_DEF",
        .if_desc = "sta",
        .route_prio = 128,
        .bridge_info = NULL
    };
    m_netif_tcpstack = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // イベントハンドラ登録
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, this));                // Wi-Fiのアクセスポイント(AP)からIPを取得
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, this));   // Wi-Fiステーション切断
}


// Wi-Fi接続イベントハンドラ
void WiFi::connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    WiFi *pThis = (WiFi*)arg;
    // 
    WiFiMessage msg = WiFiMessage::Connect;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// Wi-Fi切断イベントハンドラ
void WiFi::disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    WiFi *pThis = (WiFi*)arg;
    // 
    WiFiMessage msg = WiFiMessage::Disconnect;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// Wi-Fi接続後処理
void WiFi::wifiConnect() {
    ESP_LOGI(TAG, "Connected to %s", esp_netif_get_desc(m_netif_tcpstack));
    esp_netif_ip_info_t ip;
    ESP_ERROR_CHECK(esp_netif_get_ip_info(m_netif_tcpstack, &ip));
    ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip.ip));
    char ipaddr[256];
    sprintf(ipaddr, IPSTR, IP2STR(&ip.ip));
    m_ipAddress = ipaddr;
    if (m_func != NULL) {
        m_func(true, m_funcContext);
    }
}

// Wi-Fi切断後処理
void WiFi::wifiDisconnect() {
    if (m_func != NULL) {
        m_func(false, m_funcContext);
    }
}
