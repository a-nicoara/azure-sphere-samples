/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This file defines the mapping from the MT3620 reference development board (RDB) to the
// 'sample hardware' abstraction used by the samples at https://github.com/Azure/azure-sphere-samples.
// Some peripherals are on-board on the RDB, while other peripherals must be attached externally if needed.
// See https://aka.ms/AzureSphereHardwareDefinitions for more information on how to use hardware abstractions,
// to enable apps to work across multiple hardware variants.

// This file is autogenerated from ../../sample_hardware.json.  Do not edit it directly.

#pragma once
#include "mt3620_rdb.h"

// MT3620 RDB: Button A
#define SAMPLE_BUTTON_1 MT3620_RDB_BUTTON_A

// MT3620 RDB: Button B
#define SAMPLE_BUTTON_2 MT3620_RDB_BUTTON_B

// MT3620 RDB: ADC Potentiometer controller
#define SAMPLE_POTENTIOMETER_ADC_CONTROLLER MT3620_RDB_ADC_CONTROLLER0

// MT3620 RDB: Connect external potentiometer to ADC controller 0, channel 0 using header 2, pin 11. In the app manifest, it is only necessary to request the capability for the ADC Group Controller, SAMPLE_POTENTIOMETER_ADC_CONTROLLER.
#define SAMPLE_POTENTIOMETER_ADC_CHANNEL MT3620_ADC_CHANNEL0

// MT3620 RDB: LED 1 (red channel)
#define SAMPLE_LED MT3620_RDB_LED1_RED

// MT3620 RDB: PWM LED controller
#define SAMPLE_LED_PWM_CONTROLLER MT3620_RDB_LED_PWM_CONTROLLER2

// MT3620 RDB: Channel 1 for the PWM LED1 green. In the app manifest, it is only necessary to request the capability for the PWM Controller, SAMPLE_LED_PWM_CONTROLLER.
#define SAMPLE_LED_PWM_CHANNEL MT3620_PWM_CHANNEL1

// MT3620 RDB: LED 2 (red channel)
#define SAMPLE_RGBLED_RED MT3620_RDB_LED2_RED

// MT3620 RDB: LED 2 (green channel)
#define SAMPLE_RGBLED_GREEN MT3620_RDB_LED2_GREEN

// MT3620 RDB: LED 2 (blue channel)
#define SAMPLE_RGBLED_BLUE MT3620_RDB_LED2_BLUE

// MT3620 RDB: Connect header 2, pin 1 (RX) to header 2, pin 3 (TX).
#define SAMPLE_UART_LOOPBACK MT3620_RDB_HEADER2_ISU0_UART

// MT3620 RDB: Connect external LSM6DS3 to I2C using header 4, pin 6 (SDA) and pin 12 (SCL)
#define SAMPLE_LSM6DS3_I2C MT3620_RDB_HEADER4_ISU2_I2C

// MT3620 RDB: Connect external TSL2561 to I2C using header 4, pin 6 (SDA) and pin 12 (SCL)
#define SAMPLE_TSL2561_I2C MT3620_RDB_HEADER4_ISU2_I2C

// MT3620 RDB: Connect external LSM6DS3 to SPI using header 4, pin 5 (MISO), pin 7 (SCLK), pin 9 (CSA), pin 11 (MOSI)
#define SAMPLE_LSM6DS3_SPI MT3620_RDB_HEADER4_ISU1_SPI

// MT3620 SPI Chip Select (CS) value "A". This is not a peripheral identifier, and so has no meaning in an app manifest.
#define SAMPLE_LSM6DS3_SPI_CS MT3620_SPI_CS_A

// MT3620 RDB: Connect external NRF52 RESET GPIO using header 2, pin 4
#define SAMPLE_NRF52_RESET MT3620_RDB_HEADER2_PIN4_GPIO

// MT3620 RDB: Connect external NRF52 DFU GPIO using header 2, pin 14
#define SAMPLE_NRF52_DFU MT3620_RDB_HEADER2_PIN14_GPIO

// MT3620 RDB: Connect external NRF52 UART using header 2, pin 1 (RX), pin 3 (TX), pin 5 (CTS), pin 7 (RTS)
#define SAMPLE_NRF52_UART MT3620_RDB_HEADER2_ISU0_UART

// MT3620 RDB: LED 1 (red channel)
#define SAMPLE_DEVICE_STATUS_LED MT3620_RDB_LED1_RED

// MT3620 RDB: LED 3 (blue channel)
#define SAMPLE_PENDING_UPDATE_LED MT3620_RDB_LED3_BLUE

