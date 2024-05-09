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
#include "cJSON.h"
#include "esp_log.h"

#include "main.hpp"

#define TAG "Application"
#define ROOT "/mnt"

Application app;

// メッセージ種別 (メッセージキュー用)
enum class AppMessage {
    WIFIConnection,     // Wi-Fi接続
    WIFIDisconnection,  // Wi-Fi切断
    Quit                // 終了
};

Application::Application() {
    m_LedState = false;
    m_xHandle = NULL;
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
}

// Wi-Fi切断
void Application::wifiDisconnection() {
    m_wifi.disconnect();
}

extern "C" void app_main(void)
{
    nvs_flash_init();     // Flash初期化  (お約束のようなもの)

    app.init();
}
