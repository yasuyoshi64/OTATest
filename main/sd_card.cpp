#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sd_card.hpp"
#include "esp_vfs_fat.h"

#define TAG "SDCard"

#define DEBOUNCE_TIME_MS    500  // デバウンズ時間=500ms

// メッセージ種別 (メッセージキュー用)
enum class SDCardMessage {
    SlotStateChange,        // SDカードスロット状態変化
    Quit                    // 終了
};

SDCard::SDCard() {
    clear();
}

void SDCard::clear() {
    m_isSlot = false;
    m_base_path = NULL;
    m_card = NULL;
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_mountCallback = NULL;
    m_mountCallbackContext = NULL;
}

void SDCard::init(const char* base_path) {
   ESP_LOGI(TAG, "Init(S)");

    // ベースパス退避
    m_base_path = new char[strlen(base_path) + 1];
    strcpy(m_base_path, base_path);

    // タスク作成
    xTaskCreate(SDCard::sd_card_task, TAG, configMINIMAL_STACK_SIZE * 3, (void*)this, tskIDLE_PRIORITY, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(SDCardMessage));

    // SD Slot スイッチの初期化/ハンドラ登録
    gpio_num_t sw = (gpio_num_t)CONFIG_CARD_SW_PIN;
    gpio_reset_pin(sw);
    gpio_set_intr_type(sw, GPIO_INTR_ANYEDGE);
    gpio_set_direction(sw, GPIO_MODE_INPUT);
    gpio_pulldown_dis(sw);
    gpio_pullup_en(sw);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(sw, sd_card_slot_sw_handler, (void*)this);
    update_sd_card_slot_sw();

    ESP_LOGI(TAG, "Init(E)");
}

void SDCard::quit() {
    // SDCard用メッセージキューに投入
    SDCardMessage msg = SDCardMessage::Quit;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

// タスク
void SDCard::sd_card_task(void* arg) {
    SDCard* pThis = (SDCard*)arg;
    SDCardMessage msg;
    int64_t last_interrupt_time = DEBOUNCE_TIME_MS * -1000;
    int64_t now;
    bool loop = true;
    while(loop) {
        // メッセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case SDCardMessage::SlotStateChange:    // SDカードスロット状態変化
                    now = esp_timer_get_time();
                    if (now - last_interrupt_time > DEBOUNCE_TIME_MS * 1000) {
                        pThis->sd_card_slot_state_change();
                        last_interrupt_time = now;
                    }
                    break;
                case SDCardMessage::Quit:               // 終了
                    loop = false;
                    break;
            }
        }
    }
    // 終了処理
    pThis->sd_card_unmount();
    pThis->clear();
    vTaskDelete(NULL);
}

// SDカードスロット状態変化
void SDCard::sd_card_slot_state_change() {
    if (m_isSlot)
        sd_card_mount();    // SDカードマウント
    else
        sd_card_unmount();  // SDカードアンマウント
    if (m_mountCallback != NULL) {
        m_mountCallback(isMount(), m_mountCallbackContext);
    }
}

// SDカードマウント
void SDCard::sd_card_mount() {
    if (m_card != NULL)
        sd_card_unmount();
    // SDカード初期化 (SPI & VFS初期化)
    sdmmc_card_t* card = NULL;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_config = {
        .mosi_io_num = CONFIG_MOSI_PIN,
        .miso_io_num = CONFIG_MISO_PIN,
        .sclk_io_num = CONFIG_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };
    esp_err_t ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_config, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        ESP_ERROR_CHECK(ret);
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)CONFIG_CS_PIN;
    slot_config.host_id = (spi_host_device_t)host.slot;
    ret = esp_vfs_fat_sdspi_mount(m_base_path, &host, &slot_config, &mount_config, &card);
    sdmmc_card_print_info(stdout, card);
    m_card = card;
}

// SDカードアンマウント
void SDCard::sd_card_unmount() {
    if (m_card == NULL)
        return;
    esp_vfs_fat_sdcard_unmount(m_base_path, m_card);
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_free((spi_host_device_t)host.slot);
    m_card = NULL;
}

// SDカードスロット挿入スイッチON/OFFハンドラ
void IRAM_ATTR SDCard::sd_card_slot_sw_handler(void* arg) {
    SDCard* pThis = (SDCard*)arg;
    pThis->update_sd_card_slot_sw();
}

// m_isSlot更新 SDカードスロット挿入スイッチのGPIOの状態を見てm_isSlotを更新
void SDCard::update_sd_card_slot_sw() {
    bool isSlot = m_isSlot;
    gpio_num_t sw = (gpio_num_t)CONFIG_CARD_SW_PIN;
    if (gpio_get_level(sw) == 0) {
        m_isSlot = true;
    } else {
        m_isSlot = false;
    }
    if (m_isSlot != isSlot) {
        // スロット状態変化あり SDCard用メッセージキューに投入
        SDCardMessage msg = SDCardMessage::SlotStateChange;
        xQueueSend(m_xQueue, &msg, portMAX_DELAY);
    }
}

// SDカードマウント状態コールバック設定
void SDCard::setMountCallback(CallbackMountFunction callback, void* context) {
    m_mountCallback = callback;
    m_mountCallbackContext = context;
}

// SDカードの指定パスのファイル一覧を返します
void SDCard::fileLists(const char* path, CallbackFileFunction callback, void* context) {
    if (m_card == NULL || path == NULL)
        return;
    // ファイル一覧取得
    char* p = new char[strlen(m_base_path) + strlen(path) + 1];
    sprintf(p, "%s%s", m_base_path, path);
    ESP_LOGI(TAG, "fileLists : %s", p);
    DIR *dir = opendir(p);
    if (dir == NULL) {
        ESP_LOGE(TAG, "opendir error %d", errno);
        return;
    }
    struct dirent *entry;
    while((entry = readdir(dir)) != NULL) {
        bool is_file = entry->d_type == DT_DIR ? false : true;
        if (callback(is_file, entry->d_name, context) == false)
            break;
    }
}

