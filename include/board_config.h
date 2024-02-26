#pragma once

#ifdef __cplusplus

// SERIAL 0 GPS or Flash
#define TX0_PIN 43
#define RX0_PIN 44

// SERIAL 1 EC25
#define CTR_PIN 15
#define RTS_PIN 16
#define TX1_PIN 17
#define RX1_PIN 18

// SERIAL 2 BUS485
#define TX2_PIN 1
#define RX2_PIN 2

// I2C PIN
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9

#define TP_I2C_ADDR     0x38 //FT6336
#define TP_INT_PIN      48

#define RTC_I2C_ADDR    0x68 //DS3231

// TFT LCD, SD-CARD
#define HSPI_HOST    SPI2_HOST
#define HSPI_SCLK_PIN   12
#define HSPI_MOSI_PIN   11
#define HSPI_MISO_PIN   13

#define TFT_CS_PIN      10
#define TFT_DC_PIN      14
#define TFT_RST_PIN     21
#define TFT_BL_PIN      46

#define SD_CS_PIN       42
#define SD_DET_PIN      38

// SPI3_HOST etc. LoRa
#define VSPI_HOST       SPI3_HOST
#define VSPI_SCLK_PIN   39
#define VSPI_MOSI_PIN   40
#define VSPI_MISO_PIN   41
#define VSPI_SS_PIN     47

static const int spiClk = 1000000; // 1 MHz

#else

#error Project requires a C++ compiler, please change file extension to .cc or .cpp

#endif