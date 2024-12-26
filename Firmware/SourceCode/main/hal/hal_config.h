/**
 * @file hal_config.h
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-05-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once


/* I2C peripheral */
#define HAL_PIN_I2C_PORT    0
#define HAL_PIN_I2C_SCL     12
#define HAL_PIN_I2C_SDA     11

/* Touch pad */
#define HAL_PIN_TP_RST      -1
#define HAL_PIN_TP_INTR     3

/* PMU AXP2101 */
#define HAL_PIN_AXP_INTR    14

/* RTC PCF8563 */
#define HAL_PIN_RTC_INTR    9

/* IMU BMI270 */
#define HAL_PIN_IMU_INTR1   38
#define HAL_PIN_IMU_INTR2   40

/* Buzzer */
#define HAL_PIN_BUZZER      46

/* Mic */
#define HAL_PIN_MIC         42

/* SD card (SPI Mode) */
#define HAL_PIN_SD_CD       17      // Card Detect pin
#define HAL_PIN_SD_SCK      15      // SPI Clock
#define HAL_PIN_SD_MOSI     6       // SPI MOSI
#define HAL_PIN_SD_MISO     16      // SPI MISO
#define HAL_PIN_SD_CS       5      // SPI Chip Select
