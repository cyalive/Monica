/**
 * @file hal.cpp
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-05-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "hal.h"
#include "hal_config.h"


/**
 * @brief I2C 
 */
static bool initI2C() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)HAL_PIN_I2C_SDA,
        .scl_io_num = (gpio_num_t)HAL_PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 400000
        },
        .clk_flags = 0
    };

    esp_err_t err = i2c_param_config(HAL_PIN_I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("HAL", "Failed to configure I2C: %s (0x%X)", esp_err_to_name(err), err);
        return false;
    }

    err = i2c_driver_install(HAL_PIN_I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE("HAL", "Failed to install I2C driver: %s (0x%X)", esp_err_to_name(err), err);
        return false;
    }

    ESP_LOGI("HAL", "I2C initialized successfully");
    return true;
}


void HAL::init()
{
    /* I2C */
    if (!initI2C()) {
        ESP_LOGE("HAL", "I2C initialization failed!");
        return;
    }

    /* Display */
    disp.init();
    disp.setColorDepth(16);
    disp.setBrightness(200);

    /* RTC PCF8563 */
    rtc.init(HAL_PIN_I2C_SDA, HAL_PIN_I2C_SCL, HAL_PIN_RTC_INTR);

    // Get and print current RTC time
    tm timeInfo;
    if (rtc.getTime(timeInfo) == ESP_OK) {
        ESP_LOGI("HAL", "RTC Time: %04d-%02d-%02d %02d:%02d:%02d", 
            timeInfo.tm_year, timeInfo.tm_mon + 1, timeInfo.tm_mday,
            timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
    } else {
        ESP_LOGE("HAL", "Failed to get RTC time");
    }

    /* Touch pad */
    CST816T::Config_t tp_cfg;
    tp_cfg.pin_scl = HAL_PIN_I2C_SCL;
    tp_cfg.pin_sda = HAL_PIN_I2C_SDA;
    tp_cfg.pin_rst = HAL_PIN_TP_RST;
    tp_cfg.pin_int = HAL_PIN_TP_INTR;
    tp_cfg.i2c_num = HAL_PIN_I2C_PORT;
    tp_cfg.clk_speed = 400000;
    ESP_LOGI("HAL", "Initializing touch pad with SDA=%d, SCL=%d, RST=%d, INT=%d", 
        tp_cfg.pin_sda, tp_cfg.pin_scl, tp_cfg.pin_rst, tp_cfg.pin_int);
    if (tp.init(tp_cfg) != ESP_OK) {
        ESP_LOGE("HAL", "Touch pad initialization failed!");
    } else {
        ESP_LOGI("HAL", "Touch pad initialized successfully");
    }

    /* PMU AXP2101 */
//    pmu.init(HAL_PIN_I2C_SDA, HAL_PIN_I2C_SCL, HAL_PIN_AXP_INTR);


    /* Buttons */
    btnA.begin();

    /* Once button and power setup, check boot mode */
//    checkBootMode();

    /* Buzzer */
    buzz.init(HAL_PIN_BUZZER);

    /* SD card */
    sd.init();

    /* Serial */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("HAL", ESP_LOG_VERBOSE);
    esp_log_level_set("SD", ESP_LOG_VERBOSE);
    ESP_LOGI("HAL", "Serial initialized");

    /* Lvgl */
    lvgl.init(&disp, &tp);

    // /* IMU BMI270 + BMM150 */
    // imu.init();
    // /* Setup wrist wear wakeup interrupt */
    // imu.setWristWearWakeup();
    // /* Enable step counter */
    // imu.enableStepCounter();

}


void HAL::update()
{
    lvgl.update();
}


const std::string disk_ascii = R"(
   ****     ### *
 ******     ### ****
 ******         *****
 ********************
 ****/,,,,,,,,,,,/***
 ****             ***
 ****             ***
 ****             ***
)";


void HAL::checkBootMode()
{
    /* Press button A while power on to enter USB MSC mode */
    if (!btnA.read()) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (!btnA.read()) {

            disp.fillScreen(TFT_BLACK);
            disp.setTextSize(3);
            disp.setCursor(0, 50);
            disp.printf(" :)\n Release Key\n To Enter\n USB MSC Mode\n");

            /* Wait release */
            while (!btnA.read()) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            disp.fillScreen(TFT_BLACK);
            disp.setTextSize(3);
            disp.setCursor(0, 50);
            // disp.printf(" USB MSC Mode\n\n\n\n\n\n\n\n\n\n Reboot     ->");
            disp.printf(" [ USB MSC Mode ]\n");

            disp.setTextSize(2);
            disp.printf("\n\n\n%s\n\n\n\n Press to quit            ->", disk_ascii.c_str());

            /* Enable usb msc */
            hal_enter_usb_msc_mode();

            /* Simply restart make usb not vailable, dont know why */
            pmu.powerOff();
            while (1) {
                vTaskDelay(1000);
            }

        }
    }

}