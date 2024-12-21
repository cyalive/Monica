#pragma once
#include <driver/i2c.h>
#include <esp_log.h>
#include <cstring>


namespace CST816T {


    static const char* TAG = "CST816T";


    // CST816T I2C Address
    #define CST816T_ADDR            0x15

    // Register addresses
    #define CST816T_REG_GESTURE     0x01
    #define CST816T_REG_FINGER_NUM  0x02
    #define CST816T_REG_XPOS_H      0x03
    #define CST816T_REG_XPOS_L      0x04
    #define CST816T_REG_YPOS_H      0x05
    #define CST816T_REG_YPOS_L      0x06
    #define CST816T_REG_VERSION     0xA7
    #define CST816T_REG_CHIP_ID     0xA8
    #define CST816T_REG_MOTION      0xEC
    #define CST816T_REG_SLEEP       0xFD


    struct Config_t {
        int pin_scl     = -1;
        int pin_sda     = -1;
        int pin_rst     = -1;
        int pin_int     = -1;
        bool pull_up_en = false;

        i2c_port_t i2c_port = 0;
        uint32_t clk_speed = 100000;

        uint8_t dev_addr = CST816T_ADDR;
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
            uint8_t _data_buffer[7];

            inline esp_err_t _writrReg(uint8_t reg, uint8_t data)
            {
                _data_buffer[0] = reg;
                _data_buffer[1] = data;
                return i2c_master_write_to_device(_cfg.i2c_port, _cfg.dev_addr, _data_buffer, 2, portMAX_DELAY);
            }

            inline esp_err_t _readReg(uint8_t reg, uint8_t readSize)
            {
                /* Store data into buffer */
                return i2c_master_read_from_device(_cfg.i2c_port, _cfg.dev_addr, _data_buffer, readSize, portMAX_DELAY);
            }

            inline void _initSetup()
            {
                /* Wake up */
                _writrReg(CST816T_REG_SLEEP, 0x00);
            }

        public:
            TP_CST816T(): _inited(false) {}
            ~TP_CST816T() {}

            /* Config */
            inline Config_t config() { return _cfg; }
            inline void config(const Config_t& cfg) { _cfg = cfg; }
            inline void configPins(const int& sda, const int& scl, const int& rst = -1, const int& intr = -1)
            {
                _cfg.pin_sda = sda;
                _cfg.pin_scl = scl;
                _cfg.pin_rst = rst;
                _cfg.pin_int = intr;
            }

            inline bool init(const int& sda, const int& scl, const int& rst = -1, const int& intr = -1, const bool& initI2c = true, const uint32_t& speed = 100000)
            {
                _cfg.clk_speed = speed;
                configPins(sda, scl, rst, intr);
                return init(initI2c);
            }

            inline bool init(bool initI2c)
            {
                if (_inited) {
                    ESP_LOGW(TAG, "already inited");
                    return true;
                }

                if (initI2c) {
                    if (!i2cInit()) {
                        return false;
                    }
                }

                /* Reset chip */
                gpioInit();

                /* Setup */
                _initSetup();

                _inited = true;
                ESP_LOGI(TAG, "init done");
                return true;
            }

            inline bool i2cInit()
            {
                ESP_LOGD(TAG, "init i2c");

                /* Setup i2c */
                i2c_config_t conf;
                memset(&conf, 0, sizeof(i2c_config_t));
                conf.mode = I2C_MODE_MASTER;
                conf.sda_io_num = (gpio_num_t)_cfg.pin_sda;
                conf.sda_pullup_en = _cfg.pull_up_en ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
                conf.scl_io_num = (gpio_num_t)_cfg.pin_scl;
                conf.scl_pullup_en = _cfg.pull_up_en ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
                conf.master.clk_speed = _cfg.clk_speed;
                conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

                esp_err_t ret = i2c_param_config(_cfg.i2c_port, &conf);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "i2c param config failed");
                    return false;
                }

                ret = i2c_driver_install(_cfg.i2c_port, conf.mode, 0, 0, 0);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "i2c driver install failed");
                    return false;
                }

                return true;
            }

            /* Setup gpio and reset */
            inline void gpioInit()
            {
                ESP_LOGD(TAG, "setup gpio");

                if (_cfg.pin_int > 0) {
                    gpio_reset_pin((gpio_num_t)_cfg.pin_int);
                    gpio_set_direction((gpio_num_t)_cfg.pin_int, GPIO_MODE_INPUT);
                }

                if (_cfg.pin_rst > 0) {
                    gpio_reset_pin((gpio_num_t)_cfg.pin_rst);
                    gpio_set_direction((gpio_num_t)_cfg.pin_rst, GPIO_MODE_OUTPUT);
                    gpio_set_pull_mode((gpio_num_t)_cfg.pin_rst, GPIO_PULLUP_ONLY);
                    
                    /* Reset */
                    gpio_set_level((gpio_num_t)_cfg.pin_rst, 0);
                    vTaskDelay(pdMS_TO_TICKS(10));
                    gpio_set_level((gpio_num_t)_cfg.pin_rst, 1);
                    vTaskDelay(pdMS_TO_TICKS(50));  // CST816T needs longer reset time
                }
            }

            inline void deInit(bool deInitI2c = false)
            {
                _inited = false;
                if (deInitI2c) {
                    i2c_driver_delete(_cfg.i2c_port);
                }
            }

            inline bool isTouched()
            {
                if (_cfg.pin_int > 0) {
                    return (gpio_get_level((gpio_num_t)_cfg.pin_int) == 0);
                }
                TouchPoint_t tp;
                getTouchRaw(tp);
                return (tp.x != -1 && tp.y != -1);
            }

            inline void getTouchRaw(TouchPoint_t& tp)
            {
                tp.x = -1;
                tp.y = -1;

                if (_cfg.pin_int > 0) {
                    if (gpio_get_level((gpio_num_t)_cfg.pin_int) != 0) {
                        return;
                    }
                }

                /* Read touch data */
                if (_readReg(CST816T_REG_GESTURE, 7) == ESP_OK) {
                    tp.event = _data_buffer[1];  // Finger number
                    tp.x = ((_data_buffer[2] & 0x0F) << 8) | _data_buffer[3];
                    tp.y = ((_data_buffer[4] & 0x0F) << 8) | _data_buffer[5];
                }
            }
    };


    using TouchDriver = TP_CST816T;


}
