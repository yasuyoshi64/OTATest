/**
 * ストレージ
 * 
 * microSDのマウント/アンマウントを行います。
 * 
*/
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"

typedef void (*CallbackMountFunction)(bool isMount, void* context);
typedef bool (*CallbackFileFunction)(bool is_file, const char* name, void* context);

class SDCard {
    public:
        SDCard();

    public:
        void clear();
        void init(const char* base_path);
        void quit();

        bool isSlot() { return m_isSlot; }              // SDカード挿入状態取得
        bool isMount() { return m_card != NULL ? true : false; }
        void setMountCallback(CallbackMountFunction callback, void* context);
        void fileLists(const char* path, CallbackFileFunction callback, void* context); // SDカードの指定パスのファイル一覧を返します

    private:
        // タスク
        static void sd_card_task(void* arg);
        // SDカード関連
        void sd_card_slot_state_change();   // SDカードスロット状態変化
        void sd_card_mount();               // SDカードマウント
        void sd_card_unmount();             // SDカードアンマウント
        // SDカードスロットSW関連
        static void sd_card_slot_sw_handler(void* arg); // SDカードスロット挿入スイッチON/OFFハンドラ
        void update_sd_card_slot_sw();                  // m_isSlot更新

    private:
        bool m_isSlot;          // true = SDカード挿入中
        char* m_base_path;      // ベースパス
        TaskHandle_t m_xHandle; // タスクハンドル
        QueueHandle_t m_xQueue; // メッセージキュー
        sdmmc_card_t* m_card;   // SDカード (マウント中以外はNULL)
        CallbackMountFunction m_mountCallback;  // マウントコールバック
        void* m_mountCallbackContext;
};
