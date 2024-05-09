/**
 * Wi-Fi
*/
#pragma once

#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"

typedef void (*CallbackWiFiFunction)(bool isConnect, void* context);

class WiFi {
    public:
        WiFi();

    public:
        void init(CallbackWiFiFunction func, void* context);
        bool connect(const char* ssid, const char* pass);   // Wi-Fi接続
        void disconnect();                                  // Wi-Fi切断
        const char* getIPAddress() { return m_ipAddress.c_str(); }

    private:
        void clear();
        // タスク
        static void task(void* arg);
        //
        void wifiInit();
        void wifiConnect();     // Wi-Fi接続後処理
        void wifiDisconnect();  // Wi-Fi切断後処理
        // Wi-Fiハンドラ
        static void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
        static void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    private:
        CallbackWiFiFunction m_func;    // コールバック
        void* m_funcContext;            
        TaskHandle_t m_xHandle; // タスクハンドル
        QueueHandle_t m_xQueue; // メッセージキュー
        std::string m_ssid;     // SSID
        std::string m_pass;     // PASS
        esp_netif_t *m_netif_tcpstack;    // TCP/IPスタック
        std::string m_ipAddress;// IPアドレス
};
