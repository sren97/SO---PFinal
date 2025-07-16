/* ESP32 FreeRTOS Scheduling Evaluation System - Sensor Module
 * 
 * This module provides sensor simulation and real sensor integration
 * for the scheduling evaluation system.
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "driver/i2c.h"
#include "esp_err.h"

// Common sensor addresses
#define SENSOR_ADDR_DS18B20     0x28
#define SENSOR_ADDR_SHT30       0x44
#define SENSOR_ADDR_BMP280      0x76

// Sensor simulation configuration
#define SENSOR_SIMULATION_ENABLED   1
#define SENSOR_TEMP_MIN             15.0f
#define SENSOR_TEMP_MAX             35.0f
#define SENSOR_HUMIDITY_MIN         30.0f
#define SENSOR_HUMIDITY_MAX         80.0f

typedef struct {
    float temperature;
    float humidity;
    float pressure;
    bool valid;
} sensor_data_t;

/**
 * @brief Initialize sensor system
 */
esp_err_t sensor_init(void);

/**
 * @brief Read sensor data (simulated or real)
 */
esp_err_t sensor_read_data(sensor_data_t* data);

/**
 * @brief Read temperature sensor (DS18B20 simulation)
 */
esp_err_t sensor_read_temperature(float* temperature);

/**
 * @brief Read humidity sensor (SHT30 simulation)  
 */
esp_err_t sensor_read_humidity(float* humidity);

/**
 * @brief Check if real sensor is connected
 */
bool sensor_is_connected(uint8_t address);

#endif // SENSOR_H
