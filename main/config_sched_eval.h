/* ESP32 FreeRTOS Scheduling Evaluation System - Configuration
 * 
 * This file contains all configurable parameters for the scheduling
 * evaluation system. Modify these values to customize the system
 * behavior for your specific requirements.
 */

#ifndef CONFIG_SCHED_EVAL_H
#define CONFIG_SCHED_EVAL_H

// =============================================================================
// HARDWARE AND CORE CONFIGURATION
// =============================================================================
#define APP_CPU_NUM             1      // Pin tasks to CPU 1 for predictability

// LED Configuration
#define LED_GPIO                32      // GPIO pin for LED
#define LED_ACTIVE_LEVEL        1       // 1 for active high, 0 for active low

// I2C Configuration
#define I2C_MASTER_SCL_IO       22      // GPIO pin for I2C SCL
#define I2C_MASTER_SDA_IO       21      // GPIO pin for I2C SDA
#define I2C_MASTER_NUM          I2C_NUM_0 // I2C port number
#define I2C_MASTER_FREQ_HZ      100000  // I2C clock frequency
#define I2C_MASTER_TIMEOUT_MS   1000    // I2C operation timeout

// Sensor Configuration
#define SENSOR_SIMULATION_MODE  1       // 1 for simulation, 0 for real sensors
#define SENSOR_I2C_ADDR         0x44    // Default sensor I2C address

// =============================================================================
// TASK CONFIGURATION
// =============================================================================

// Task Priorities (can be modified by scheduling mode)
#define LED_TASK_PRIORITY       3       // LED task priority
#define CALC_TASK_PRIORITY      2       // Calculation task priority
#define SENSOR_TASK_PRIORITY    4       // Sensor task priority
#define METRICS_TASK_PRIORITY   5       // Metrics task priority

// Task Stack Sizes
#define LED_TASK_STACK_SIZE     3072    // LED task stack size
#define CALC_TASK_STACK_SIZE    4096    // Calculation task stack size
#define SENSOR_TASK_STACK_SIZE  4096    // Sensor task stack size
#define METRICS_TASK_STACK_SIZE 4096    // Metrics task stack size

// Task Timing and Period Configuration
#define LED_PERIOD_MS           100     // LED blink period (ms)
#define CPU_CALC_PERIOD_MS      150     // CPU calculation task period (ms)
#define SENSOR_PERIOD_MS        200     // Sensor reading period (ms)
#define METRICS_PERIOD_MS       1000    // Metrics display period (ms)
#define MODE_SWITCH_PERIOD_S    10      // Switch scheduling mode every 10 seconds

// Task Behavior Configuration (Randomness for variability)
#define LED_RESPONSE_TIME_SIM_DELAY_MS  1
#define CALC_TASK_DELAY_RANDOM_MS_MIN   2
#define CALC_TASK_DELAY_RANDOM_MS_MAX   8
#define CALC_ITERATIONS_RANDOM_MAX      3
#define CALC_AMPLITUDE_RANDOM_MAX       50
#define CALC_YIELD_RANDOM_MS_MAX        2
#define CALC_PERIOD_RANDOM_MS_MAX       15
#define SENSOR_TASK_DELAY_RANDOM_MS_MIN 1
#define SENSOR_TASK_DELAY_RANDOM_MS_MAX 4
#define SENSOR_PERIOD_RANDOM_MS_MAX     20
#define SENSOR_READ_CYCLES_RANDOM_MAX   2
#define SENSOR_PROCESSING_Nops_RANDOM_MAX 300

// Legacy compatibility
#define CALC_PERIOD_MS          CPU_CALC_PERIOD_MS

// =============================================================================
// SCHEDULING CONFIGURATION
// =============================================================================

// Scheduling Mode Auto-Switch
#define AUTO_SWITCH_MODES       1       // 1 to auto-switch modes, 0 for manual
#define MODE_SWITCH_INTERVAL    10      // Seconds between mode switches

// Cooperative Scheduling
#define COOP_YIELD_INTERVAL     1       // Yield interval in cooperative mode (ms)

// =============================================================================
// CALCULATION TASK CONFIGURATION
// =============================================================================

// FFT Simulation Parameters
#define FFT_SIZE                64      // FFT size for calculation task
#define CALCULATION_ITERATIONS  50      // Base iterations for calculation
#define CALC_COMPLEXITY_FACTOR  1.0     // Multiplier for calculation complexity

// =============================================================================
// METRICS AND ANALYSIS CONFIGURATION
// =============================================================================

// Metrics Collection
#define MAX_METRICS_SAMPLES     100     // Maximum samples to store per task
#define METRICS_COLLECTION_ENABLED 1    // Enable/disable metrics collection

// Performance Thresholds
#define RESPONSE_TIME_THRESHOLD 5000    // Response time warning threshold (μs)
#define JITTER_THRESHOLD_US     1000    // Jitter warning threshold (μs)
#define CPU_UTILIZATION_THRESHOLD 80    // CPU utilization warning threshold (%)

// Display Configuration
#define METRICS_DISPLAY_ENABLED 1       // Enable/disable metrics display
#define FREERTOS_STATS_ENABLED  1       // Enable/disable FreeRTOS stats display
#define COLOR_OUTPUT_ENABLED    1       // Enable/disable colored output

// =============================================================================
// SENSOR SIMULATION CONFIGURATION
// =============================================================================

// Temperature Simulation
#define SENSOR_TEMP_BASE        25.0f   // Base temperature (°C)
#define SENSOR_TEMP_VARIATION   2.0f    // Temperature variation range (°C)
#define SENSOR_TEMP_DRIFT_RATE  0.1f    // Temperature drift rate

// Humidity Simulation
#define SENSOR_HUMIDITY_BASE    50.0f   // Base humidity (%)
#define SENSOR_HUMIDITY_VARIATION 10.0f // Humidity variation range (%)
#define SENSOR_HUMIDITY_DRIFT_RATE 0.2f // Humidity drift rate

// Pressure Simulation
#define SENSOR_PRESSURE_BASE    1013.25f // Base pressure (hPa)
#define SENSOR_PRESSURE_VARIATION 5.0f   // Pressure variation range (hPa)

// =============================================================================
// LOGGING CONFIGURATION
// =============================================================================

// Log Levels
#define LOG_LEVEL_MAIN          ESP_LOG_INFO    // Main application log level
#define LOG_LEVEL_TASKS         ESP_LOG_INFO    // Task log level
#define LOG_LEVEL_METRICS       ESP_LOG_INFO    // Metrics log level
#define LOG_LEVEL_SENSORS       ESP_LOG_INFO    // Sensor log level

// Debug Features
#define DEBUG_TIMING_ENABLED    0       // Enable detailed timing debug
#define DEBUG_TASK_SWITCHES     0       // Debug task context switches
#define DEBUG_MEMORY_USAGE      0       // Debug memory usage

// =============================================================================
// PERFORMANCE TUNING
// =============================================================================

// Memory Optimization
#define OPTIMIZE_MEMORY_USAGE   1       // Enable memory optimizations
#define METRICS_RING_BUFFER     1       // Use ring buffer for metrics

// CPU Optimization
#define OPTIMIZE_CPU_USAGE      1       // Enable CPU optimizations
#define FAST_MATH_ENABLED       1       // Use fast math operations

// =============================================================================
// SAFETY AND RELIABILITY
// =============================================================================

// Watchdog Configuration
#define TASK_WATCHDOG_ENABLED   1       // Enable task watchdog
#define WATCHDOG_TIMEOUT_MS     5000    // Watchdog timeout (ms)

// Error Handling
#define ERROR_RECOVERY_ENABLED  1       // Enable error recovery
#define MAX_ERROR_COUNT         10      // Maximum errors before system reset

// Stack Monitoring
#define STACK_MONITORING_ENABLED 1      // Enable stack usage monitoring
#define STACK_WARNING_THRESHOLD  80     // Stack usage warning threshold (%)

// Performance Thresholds
#define RESPONSE_TIME_THRESHOLD_US    1000  // Response time threshold (microseconds)

// Data Export
#define DATA_EXPORT_ENABLED     0       // Enable data export via UART
#define EXPORT_FORMAT_JSON      1       // Use JSON format for export
#define EXPORT_INTERVAL_MS      5000    // Export interval (ms)

// Remote Monitoring
#define REMOTE_MONITORING       0       // Enable remote monitoring
#define MONITORING_PORT         8080    // TCP port for monitoring

// Real-time Alerts
#define ALERT_SYSTEM_ENABLED    0       // Enable alert system
#define ALERT_THRESHOLD_HIGH    90      // High alert threshold (%)
#define ALERT_THRESHOLD_CRITICAL 95     // Critical alert threshold (%)

#endif // CONFIG_SCHED_EVAL_H
