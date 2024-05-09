#pragma once

#include <iostream>
#include <map>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "sd_card.hpp"
#include "wifi.hpp"

class Application {
    public:
        Application();

    public:
        void init();
        void led(int state);

    private:
        // タスク
        static void app_task(void* arg);
        // コールバック        
        static void mountFunc(bool isMount, void* context);
        static void dispInitCompFunc(void* context);
        static void wifiConnectFunc(bool isConnect, void* context);
        //
        bool getConfig(const char* root);   // SDカード内の./configファイル読み込み。結果はconfigMapに格納。
        void wifiConnection();              // Wi-Fi接続
        void wifiDisconnection();           // Wi-Fi切断
        // OTA
        void ota();
        static void checkOTA(void* arg);

    private:
        int m_LedState;
        TaskHandle_t m_xHandle; // タスクハンドル
        TaskHandle_t m_xHandleOTA;  // OTAタスクハンドル
        QueueHandle_t m_xQueue; // メッセージキュー
        SDCard m_sd_card;   // SDカード
        WiFi m_wifi;        // Wi-Fi
        std::map<std::string, std::string> m_configMap{};     // CONFIG
        bool m_isWiFi;
};