/**
 * @file hal_sd_crad.hpp
 * @author Forairaaaaa
 * @brief Init SD card and file system for lvgl's file r/w
 * @version 0.1
 * @date 2023-05-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
/* https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/sdmmc/main/sd_card_example_main.c */
#pragma once
#include "../hal_config.h"
#include <iostream>
#include <string>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>
#include <driver/spi_common.h>
#include <driver/sdspi_host.h>
#include <driver/gpio.h>

namespace SD_CARD {

    static const char* TAG = "SD";

    struct Config_t {
        int CD = HAL_PIN_SD_CD;
        int SCK = HAL_PIN_SD_SCK;
        int MOSI = HAL_PIN_SD_MOSI;
        int MISO = HAL_PIN_SD_MISO;
        int CS = HAL_PIN_SD_CS;

        std::string mountPoint = "/sdcard";
        bool formatIfMountFailed = true;
    };

    class SD_Card {
        private:
            Config_t _config;
            bool _available;

        public:
            SD_Card() : _available(false) {}
            ~SD_Card() = default;

            inline void config(Config_t cfg) { _config = cfg; }
            inline Config_t config(void) { return _config; }

            inline bool isAvailable() { return _available; }

            void printDirectory(const char* dirPath, int depth = 0)
            {
                if (!_available) {
                    ESP_LOGE(TAG, "SD card not available");
                    return;
                }

                DIR* dir = opendir(dirPath);
                if (!dir) {
                    ESP_LOGE(TAG, "Failed to open directory: %s", dirPath);
                    return;
                }

                struct dirent* entry;
                while ((entry = readdir(dir)) != NULL) {
                    // Skip current and parent directory entries
                    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                        continue;
                    }

                    // Print indentation based on depth
                    for (int i = 0; i < depth; i++) {
                        printf("  ");
                    }

                    // Get file info
                    struct stat statbuf;
                    // Use larger buffer and check path length
                    char fullPath[512];
                    int pathLen = snprintf(fullPath, sizeof(fullPath), "%s/%s", dirPath, entry->d_name);
                    if (pathLen >= sizeof(fullPath)) {
                        ESP_LOGE(TAG, "Path too long: %s/%s", dirPath, entry->d_name);
                        continue;
                    }
                    if (stat(fullPath, &statbuf) == -1) {
                        ESP_LOGE(TAG, "Failed to stat file: %s", fullPath);
                        continue;
                    }

                    // Print file info
                    if (S_ISDIR(statbuf.st_mode)) {
                        printf("[DIR] %s\n", entry->d_name);
                        // Recursively print subdirectory
                        printDirectory(fullPath, depth + 1);
                    } else {
                        printf("[FILE] %s (%ld bytes)\n", entry->d_name, statbuf.st_size);
                    }
                }
                closedir(dir);
            }

            inline void init()
            {
                esp_err_t ret;

                // Check if SD card is inserted
                gpio_set_direction((gpio_num_t)_config.CD, GPIO_MODE_INPUT);
                gpio_set_pull_mode((gpio_num_t)_config.CD, GPIO_PULLUP_ONLY);
                if (gpio_get_level((gpio_num_t)_config.CD) == 1) {
                    ESP_LOGW(TAG, "No SD card inserted");
                    return;
                }

                // Options for mounting the filesystem.
                // If format_if_mount_failed is set to true, SD card will be partitioned and
                // formatted in case when mounting fails.
                esp_vfs_fat_sdmmc_mount_config_t mount_config;
                mount_config.format_if_mount_failed = _config.formatIfMountFailed;
                mount_config.max_files = 5;
                mount_config.allocation_unit_size = 16 * 1024;
                mount_config.disk_status_check_enable = false;
                sdmmc_card_t *card;
                ESP_LOGI(TAG, "Initializing SD card");

                // Use settings defined above to initialize SD card and mount FAT filesystem.
                // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
                // Please check its source code and implement error recovery when developing
                // production applications.

                ESP_LOGI(TAG, "Using SPI peripheral");

                // Only set CD pin as input, no need for internal pullups as external ones are present
                gpio_set_direction((gpio_num_t)_config.CD, GPIO_MODE_INPUT);

                sdmmc_host_t host = SDSPI_HOST_DEFAULT();
                spi_bus_config_t bus_cfg = {
                    .mosi_io_num = _config.MOSI,   // MOSI line
                    .miso_io_num = _config.MISO,   // MISO line
                    .sclk_io_num = _config.SCK,    // SCK line
                    .quadwp_io_num = -1,
                    .quadhd_io_num = -1,
                    .max_transfer_sz = 4000,
                };
                // Initialize SPI bus
                ret = spi_bus_initialize((spi_host_device_t)SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
                if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                    ESP_LOGE(TAG, "Failed to initialize bus: %s", esp_err_to_name(ret));
                    return;
                }

                // This initializes the slot without card detect (CD) and write protect (WP) signals.
                sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
                slot_config.gpio_cs = (gpio_num_t)_config.CS;     // CS pin
                slot_config.host_id = (spi_host_device_t)SPI2_HOST;  // 使用SPI2_HOST

                ESP_LOGI(TAG, "Mounting filesystem");
                ret = esp_vfs_fat_sdspi_mount(_config.mountPoint.c_str(), &host, &slot_config, &mount_config, &card);

                if (ret != ESP_OK) {
                    if (ret == ESP_FAIL) {
                        ESP_LOGE(TAG, "Failed to mount filesystem. "
                                "If you want the card to be formatted, set formatIfMountFailed to true.");
                    } else {
                        ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
                    }
                    return;
                }
                ESP_LOGI(TAG, "Filesystem mounted");

                // Card has been initialized, print its properties
                sdmmc_card_print_info(stdout, card);

                _available = true;
            }

    };

}
