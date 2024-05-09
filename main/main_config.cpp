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
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "main.hpp"

#define TAG "ApplicationConfig"

// SDカード内の./configファイル読み込み。結果はconfigMapに格納。
bool Application::getConfig(const char* root) {
    bool ret = false;
    const char* szConfigPath = "/config";
    char* szPath = new char[strlen(root) + strlen(szConfigPath) + 1];
    sprintf(szPath, "%s%s", root, szConfigPath);
    m_configMap.clear();
    FILE* fd = fopen(szPath, "r");
    if (fd != NULL) {
        char szLine[256];
        int len = 0;
        std::string key, value;
        while(1) {
            int c = fgetc(fd);
            if (feof(fd) || ferror(fd)) {
                break;
            }
            if (c == '\r') {
                continue;                
            } else if (c == '=' || c == '\n') {
                szLine[len] = '\0';
                value = szLine;
                if (c == '=') {
                    key = szLine;
                } else {
                    value = szLine;
                    m_configMap.insert(std::make_pair(key, value));
                }
                len = 0;
            } else {
                szLine[len++] = (char)c;
            }
        }
        fclose(fd);
        ret = true;
    }
    delete[] szPath;

    auto iter = m_configMap.begin();
    while(iter != m_configMap.end()) {
        ESP_LOGI(TAG, "%s=%s", iter->first.c_str(), iter->second.c_str());
        iter++;
    }
    return ret;
}
