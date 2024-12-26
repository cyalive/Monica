#pragma once
#include "hal_tp.hpp"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 坐标转换配置
#define CTP_REVERS_XY      1   // 交换 X/Y 坐标
#define CTP_REVERS_XPOS    0   // 反转 X 坐标
#define CTP_REVERS_YPOS    1   // 反转 Y 坐标

namespace CST816T {

    static const char* TAG = "CST816T";

    struct Config_t {
        int pin_scl = -1;
        int pin_sda = -1;
        int pin_rst = -1;
        int pin_int = -1;
        uint32_t clk_speed = 400000;
        uint8_t i2c_num = 0;
        uint8_t dev_addr = 0x15;
    };

    struct TouchPoint_t {
        uint8_t event = 0;
        uint8_t id = 0;
        int x = -1;
        int y = -1;
    };

    class TP_CST816T {
        private:
            Config_t _cfg;
            bool _inited;
            int _x_pos;
            int _y_pos;
            bool _enable;
            uint8_t _data_buffer[4];

            inline esp_err_t _I2C_write1Byte(uint8_t reg_addr, uint8_t data) {
                uint8_t write_buf[2] = {reg_addr, data};
                return i2c_master_write_to_device((i2c_port_t)_cfg.i2c_num, _cfg.dev_addr, write_buf, sizeof(write_buf), pdMS_TO_TICKS(10));
            }

            inline esp_err_t _I2C_readBytes(uint8_t reg_addr, uint8_t* data, size_t size) {
                return i2c_master_write_read_device((i2c_port_t)_cfg.i2c_num, _cfg.dev_addr, &reg_addr, 1, data, size, pdMS_TO_TICKS(10));
            }

            inline esp_err_t _I2C_read8Bit(uint8_t reg_addr, uint8_t* data) {
                return i2c_master_write_read_device((i2c_port_t)_cfg.i2c_num, _cfg.dev_addr, &reg_addr, 1, data, 1, pdMS_TO_TICKS(10));
            }

            inline void _reset_coor() {
                _x_pos = -1;
                _y_pos = -1;
            }

            inline void _update_coor() {
                uint8_t data[4] = {0};
                if (_I2C_readBytes(0x03, data, 4) == ESP_OK) {
                    ESP_LOGD(TAG, "Raw data: %02X %02X %02X %02X", data[0], data[1], data[2], data[3]);

                    // 检查数据有效性
                    if ((data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF && data[3] == 0xFF) ||  // 全FF表示通信错误
                        (data[0] == 0xFF)) {  // 首字节为FF表示特殊状态
                        ESP_LOGD(TAG, "Invalid data detected, skipping");
                        _x_pos = -1;
                        _y_pos = -1;
                        return;
                    }

                    // 先获取原始坐标
                    int raw_x, raw_y;
                    #if CTP_REVERS_XY
                        raw_y = ((data[0] & 0x0F) << 8) | data[1];
                        raw_x = ((data[2] & 0x0F) << 8) | data[3];
                    #else
                        raw_x = ((data[0] & 0x0F) << 8) | data[1];
                        raw_y = ((data[2] & 0x0F) << 8) | data[3];
                    #endif

                    // 映射坐标范围
                    #if CTP_REVERS_XY
                        _x_pos = _map(raw_x, 2, 275, 0, 280);
                        _y_pos = _map(raw_y, 10, 240, 0, 240);
                        
                        #if CTP_REVERS_XPOS
                            _x_pos = 280 - _x_pos;
                        #endif
                        #if CTP_REVERS_YPOS
                            _y_pos = 240 - _y_pos;
                        #endif

                        // 限制坐标范围
                        if (_x_pos > 280) _x_pos = 280;
                        if (_x_pos < 0) _x_pos = 0;
                        if (_y_pos > 240) _y_pos = 240;
                        if (_y_pos < 0) _y_pos = 0;
                    #else
                        _x_pos = _map(raw_x, 10, 240, 0, 240);
                        _y_pos = _map(raw_y, 2, 275, 0, 280);

                        // 限制坐标范围
                        if (_x_pos > 240) _x_pos = 240;
                        if (_x_pos < 0) _x_pos = 0;
                        if (_y_pos > 280) _y_pos = 280;
                        if (_y_pos < 0) _y_pos = 0;
                    #endif

                    ESP_LOGD(TAG, "Raw coordinates: x=%d, y=%d", raw_x, raw_y);
                    ESP_LOGD(TAG, "Mapped coordinates: x=%d, y=%d", _x_pos, _y_pos);
                } else {
                    ESP_LOGW(TAG, "Failed to read coordinates");
                    _x_pos = -1;
                    _y_pos = -1;
                }
            }

        public:
            TP_CST816T() : _inited(false), _enable(true) {
                _reset_coor();
            }

            esp_err_t init(const Config_t& cfg) {
                _cfg = cfg;

                // 只配置 I2C 参数，不重新安装驱动
                i2c_config_t conf = {
                    .mode = I2C_MODE_MASTER,
                    .sda_io_num = (gpio_num_t)_cfg.pin_sda,
                    .scl_io_num = (gpio_num_t)_cfg.pin_scl,
                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                    .master = {
                        .clk_speed = _cfg.clk_speed
                    },
                    .clk_flags = 0
                };
                
                esp_err_t ret = i2c_param_config((i2c_port_t)_cfg.i2c_num, &conf);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "I2C config failed: %d", ret);
                    return ret;
                }

                // Configure INT pin if used
                if (_cfg.pin_int > 0) {
                    gpio_config_t io_conf = {};
                    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // 下降沿触发
                    io_conf.pin_bit_mask = (1ULL << _cfg.pin_int);
                    io_conf.mode = GPIO_MODE_INPUT;
                    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;   // 启用上拉
                    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
                    gpio_config(&io_conf);
                }

                // Configure RST pin if used
                if (_cfg.pin_rst > 0) {
                    gpio_config_t io_conf = {};
                    io_conf.pin_bit_mask = (1ULL << _cfg.pin_rst);
                    io_conf.mode = GPIO_MODE_OUTPUT;
                    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
                    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
                    gpio_config(&io_conf);

                    // Reset sequence
                    gpio_set_level((gpio_num_t)_cfg.pin_rst, 0);
                    vTaskDelay(pdMS_TO_TICKS(10));
                    gpio_set_level((gpio_num_t)_cfg.pin_rst, 1);
                    vTaskDelay(pdMS_TO_TICKS(50));  // Wait for stable
                }

                // 配置工作模式
                uint8_t mode = 0x00;  // 正常工作模式
                if (_I2C_write1Byte(0xA0, mode) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set work mode");
                    return ESP_FAIL;
                }

                // 配置中断模式
                uint8_t irq_ctrl = 0x01;  // 使能触摸中断，低电平有效
                if (_I2C_write1Byte(0xA4, irq_ctrl) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set IRQ mode");
                    return ESP_FAIL;
                }

                // 自动休眠时间
                uint8_t auto_sleep = 0x0A;  // 10秒
                if (_I2C_write1Byte(0xA5, auto_sleep) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set auto sleep time");
                    return ESP_FAIL;
                }

                _enable = true;
                ESP_LOGI(TAG, "CST816T initialized successfully");
                return ESP_OK;
            }

            bool isTouched() {
                if (!_enable) return false;

                uint8_t touched = 0;
                esp_err_t ret = _I2C_read8Bit(0x02, &touched);
                // ESP_LOGD(TAG, "Touch status register: 0x%02X (ret=%d)", touched, ret);

                // 检查中断引脚状态
                if (_cfg.pin_int > 0) {
                    bool int_active = (gpio_get_level((gpio_num_t)_cfg.pin_int) == 0);
                    // ESP_LOGD(TAG, "INT pin is %s", int_active ? "active(low)" : "inactive(high)");
                    // 只要有一个指示触摸就认为是触摸状态
                    return (ret == ESP_OK && (touched || int_active));
                }
                
                return (ret == ESP_OK && touched);
            }

            void getTouchRaw(TouchPoint_t& tp) {
                tp.x = -1;
                tp.y = -1;

                if (!_enable) return;

                if (!isTouched()) {
                    ESP_LOGD(TAG, "Not touched");
                    return;
                }

                _update_coor();
                if (_x_pos != -1 && _y_pos != -1) {
                    tp.x = _x_pos;
                    tp.y = _y_pos;
                    tp.event = 1;
                    ESP_LOGD(TAG, "Touch data: x=%d, y=%d", tp.x, tp.y);
                }
            }

            void getPos(int& x_pos, int& y_pos) {
                TouchPoint_t tp;
                getTouchRaw(tp);
                x_pos = tp.x;
                y_pos = tp.y;
            }

            inline void enable() { _enable = true; }
            inline void disable() { _enable = false; }

            inline int16_t _map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
                return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            }
    };

    using TouchDriver = TP_CST816T;
}
