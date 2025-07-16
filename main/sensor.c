/* ESP32 FreeRTOS Scheduling Evaluation System - Sensor Implementation
 * 
 * This module provides sensor simulation and real sensor integration
 * for the scheduling evaluation system.
 */

#include "sensor.h"
#include "esp_log.h"
#include "esp_random.h"
#include <math.h>

static const char* TAG = "SENSOR";

/**
 * @brief Initialize sensor system
 */
esp_err_t sensor_init(void) {
    ESP_LOGI(TAG, "Sensor system initialized");
    
    #if !SENSOR_SIMULATION_ENABLED
    // Check for real sensors
    if (sensor_is_connected(SENSOR_ADDR_DS18B20)) {
        ESP_LOGI(TAG, "DS18B20 temperature sensor detected");
    }
    if (sensor_is_connected(SENSOR_ADDR_SHT30)) {
        ESP_LOGI(TAG, "SHT30 humidity sensor detected");
    }
    if (sensor_is_connected(SENSOR_ADDR_BMP280)) {
        ESP_LOGI(TAG, "BMP280 pressure sensor detected");
    }
    #endif
    
    return ESP_OK;
}

/**
 * @brief Read sensor data (simulated or real)
 */
esp_err_t sensor_read_data(sensor_data_t* data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    #if SENSOR_SIMULATION_ENABLED
    // Simulate sensor readings with realistic variations
    static float base_temp = 25.0f;
    static float base_humidity = 50.0f;
    static float base_pressure = 1013.25f;
    
    // Add random variations
    float temp_variation = (esp_random() % 200 - 100) / 100.0f; // ±1°C
    float humidity_variation = (esp_random() % 100 - 50) / 10.0f; // ±5%
    float pressure_variation = (esp_random() % 100 - 50) / 10.0f; // ±5 hPa
    
    // Apply slow drift
    base_temp += sinf(esp_timer_get_time() / 1000000.0f / 60.0f) * 0.1f;
    base_humidity += cosf(esp_timer_get_time() / 1000000.0f / 120.0f) * 0.2f;
    
    data->temperature = base_temp + temp_variation;
    data->humidity = base_humidity + humidity_variation;
    data->pressure = base_pressure + pressure_variation;
    
    // Clamp values to realistic ranges
    data->temperature = fmax(SENSOR_TEMP_MIN, fmin(SENSOR_TEMP_MAX, data->temperature));
    data->humidity = fmax(SENSOR_HUMIDITY_MIN, fmin(SENSOR_HUMIDITY_MAX, data->humidity));
    data->pressure = fmax(950.0f, fmin(1050.0f, data->pressure));
    
    data->valid = true;
    
    #else
    // Read from real sensors
    esp_err_t ret = sensor_read_temperature(&data->temperature);
    if (ret == ESP_OK) {
        ret = sensor_read_humidity(&data->humidity);
    }
    data->valid = (ret == ESP_OK);
    #endif
    
    return ESP_OK;
}

/**
 * @brief Read temperature sensor (DS18B20 simulation)
 */
esp_err_t sensor_read_temperature(float* temperature) {
    if (!temperature) {
        return ESP_ERR_INVALID_ARG;
    }
    
    #if SENSOR_SIMULATION_ENABLED
    // Simulate DS18B20 temperature reading
    *temperature = 25.0f + (esp_random() % 200 - 100) / 100.0f;
    return ESP_OK;
    #else
    // Real DS18B20 implementation would go here
    // For now, return simulated value
    *temperature = 25.0f;
    return ESP_OK;
    #endif
}

/**
 * @brief Read humidity sensor (SHT30 simulation)
 */
esp_err_t sensor_read_humidity(float* humidity) {
    if (!humidity) {
        return ESP_ERR_INVALID_ARG;
    }
    
    #if SENSOR_SIMULATION_ENABLED
    // Simulate SHT30 humidity reading
    *humidity = 50.0f + (esp_random() % 100 - 50) / 10.0f;
    return ESP_OK;
    #else
    // Real SHT30 implementation would go here
    // For now, return simulated value
    *humidity = 50.0f;
    return ESP_OK;
    #endif
}

/**
 * @brief Check if real sensor is connected
 */
bool sensor_is_connected(uint8_t address) {
    #if SENSOR_SIMULATION_ENABLED
    return false; // Always return false in simulation mode
    #else
    // Try to communicate with sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
    #endif
}
