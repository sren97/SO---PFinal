/* Sistema de Evaluación de Scheduling FreeRTOS ESP32
 * 
 * Análisis integral de rendimiento en tiempo real de diferentes
 * estrategias de scheduling de FreeRTOS en ESP32 DevKitC-VE:
 * 
 * - Scheduling round-robin para distribución equitativa de tiempo
 * - Scheduling basado en prioridades para requisitos de tiempo real
 * - Scheduling cooperativo/preemptivo para control especializado
 * - Tarea de control LED (GPIO 32) para control de tiempo preciso
 * - Tarea de cálculo intensivo (simulación FFT)
 * - Tarea de lectura de sensores (simulación I2C de temperatura)
 * - Dashboard de métricas en tiempo real con análisis de rendimiento
 * 
 * Hardware: ESP32 DevKitC-VE, LED en GPIO 32
 * Framework: ESP-IDF v5.4 con FreeRTOS
 */

#include "config_sched_eval.h"
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "hal/gpio_hal.h"
#include "sensor.h"

static const char* TAG = "SCHED_EVAL";

void vApplicationTaskSwitchHook(void);

// =============================================================================
// DEFINICIONES DE COLORES PARA SALIDA DE TERMINAL
// =============================================================================
#define COLOR_RESET     "\033[0m"
#define COLOR_RED       "\033[31m"
#define COLOR_GREEN     "\033[32m"
#define COLOR_YELLOW    "\033[33m"
#define COLOR_BLUE      "\033[34m"
#define COLOR_MAGENTA   "\033[35m"
#define COLOR_CYAN      "\033[36m"
#define COLOR_WHITE     "\033[37m"
#define COLOR_BOLD      "\033[1m"

// =============================================================================
// CONSTANTES DE CONFIGURACIÓN
// =============================================================================

// Prioridades de tareas
#define LED_TASK_PRIORITY       3
#define CALC_TASK_PRIORITY      2
#define SENSOR_TASK_PRIORITY    4
#define METRICS_TASK_PRIORITY   5

// Configuración de timing de tareas
#define LED_PERIOD_MS           100    // LED parpadeando a 10 Hz
#define SENSOR_PERIOD_MS        200    // Lectura de sensor cada 200ms
#define METRICS_PERIOD_MS       1000   // Mostrar métricas cada 1s

// Configuración de tarea de cálculo (sobrescribe archivo config para mejor rendimiento)
#undef FFT_SIZE
#undef CALCULATION_ITERATIONS
#define FFT_SIZE                64     // Tamaño FFT reducido para mejor rendimiento
#define CALCULATION_ITERATIONS  50     // Iteraciones reducidas para prevenir watchdog

// =============================================================================
// ESTRUCTURAS DE DATOS
// =============================================================================

typedef enum {
    SCHED_ROUND_ROBIN = 0,
    SCHED_PRIORITY_BASED,
    SCHED_PREEMPTIVE_COOP
} scheduling_mode_t;

typedef struct {
    uint64_t event_time;
    uint64_t start_time;
    uint64_t end_time;
    uint64_t response_time;
    uint64_t turnaround_time;
    uint64_t cpu_time;
} task_timing_t;

typedef struct {
    uint64_t expected_time;
    uint64_t actual_time;
    int64_t jitter;
} jitter_measurement_t;

typedef struct {
    const char* task_name;
    task_timing_t timings[MAX_METRICS_SAMPLES];
    jitter_measurement_t jitter[MAX_METRICS_SAMPLES];
    uint32_t sample_count;
    uint32_t jitter_count;
    uint64_t total_response_time;
    uint64_t total_turnaround_time;
    uint64_t total_cpu_time;
    uint64_t max_response_time;
    uint64_t max_turnaround_time;
    uint64_t max_jitter;
    uint32_t context_switches;
    uint32_t priority_inversions;
} task_metrics_t;

typedef struct {
    scheduling_mode_t current_mode;
    uint64_t system_start_time;
    uint64_t total_context_switches;
    uint64_t context_switch_overhead;
    task_metrics_t led_metrics;
    task_metrics_t calc_metrics;
    task_metrics_t sensor_metrics;
} system_metrics_t;

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================

static system_metrics_t system_metrics = {0};

static SemaphoreHandle_t metrics_mutex;
static QueueHandle_t event_queue;

static TaskHandle_t led_task_handle = NULL;
static TaskHandle_t calc_task_handle = NULL;
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t metrics_task_handle = NULL;

typedef enum {
    EVENT_METRICS_TRIGGER = 1
} event_type_t;

static scheduling_mode_t current_scheduling_mode = SCHED_ROUND_ROBIN;

static volatile uint64_t context_switch_count = 0;
static TaskHandle_t last_task_handle = NULL;

/**
 * @brief Hook de cambio de tarea para contar cambios de contexto
 * Este hook se llama cada vez que FreeRTOS cambia entre tareas
 */
void vApplicationTaskSwitchHook(void) {
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    if (last_task_handle != NULL && last_task_handle != current_task) {
        context_switch_count++;
    }
    last_task_handle = current_task;
}

/**
 * @brief Conteo alternativo de cambios de contexto usando notificaciones de tarea
 */
static void update_context_switches(void) {
    static uint32_t last_tick = 0;
    uint32_t current_tick = xTaskGetTickCount();
    
    if (current_tick > last_tick) {
        context_switch_count += (current_tick - last_tick);
        last_tick = current_tick;
    }
}


// =============================================================================
// FUNCIONES DE UTILIDAD
// =============================================================================

/**
 * @brief Obtener timestamp de alta precisión en microsegundos
 */
static uint64_t get_timestamp_us(void) {
    return esp_timer_get_time();
}

/**
 * @brief Inicializar estructura de métricas de tarea
 */
static void init_task_metrics(task_metrics_t* metrics, const char* name) {
    memset(metrics, 0, sizeof(task_metrics_t));
    metrics->task_name = name;
}

/**
 * @brief Registrar métricas de timing de tarea
 */
static void record_task_timing(task_metrics_t* metrics, uint64_t event_time, 
                              uint64_t start_time, uint64_t end_time) {
    if (xSemaphoreTake(metrics_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint32_t idx;
        if (metrics->sample_count < MAX_METRICS_SAMPLES) {
            idx = metrics->sample_count++;
        } else {
            idx = metrics->sample_count % MAX_METRICS_SAMPLES;
            metrics->sample_count++;
        }
        
        task_timing_t* timing = &metrics->timings[idx];
        
        timing->event_time = event_time;
        timing->start_time = start_time;
        timing->end_time = end_time;
        timing->response_time = start_time - event_time;
        timing->turnaround_time = end_time - event_time;
        timing->cpu_time = end_time - start_time;
        
        metrics->total_response_time += timing->response_time;
        metrics->total_turnaround_time += timing->turnaround_time;
        metrics->total_cpu_time += timing->cpu_time;
        
        if (timing->response_time > metrics->max_response_time) {
            metrics->max_response_time = timing->response_time;
        }
        if (timing->turnaround_time > metrics->max_turnaround_time) {
            metrics->max_turnaround_time = timing->turnaround_time;
        }
        xSemaphoreGive(metrics_mutex);
    }
}

/**
 * @brief Registrar medición de jitter
 */
static void record_jitter(task_metrics_t* metrics, uint64_t expected_time, uint64_t actual_time) {
    if (xSemaphoreTake(metrics_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (metrics->jitter_count < MAX_METRICS_SAMPLES) {
            uint32_t idx = metrics->jitter_count++;
            jitter_measurement_t* jitter = &metrics->jitter[idx];
            
            jitter->expected_time = expected_time;
            jitter->actual_time = actual_time;
            jitter->jitter = (int64_t)actual_time - (int64_t)expected_time;
            
            uint64_t abs_jitter = (jitter->jitter >= 0) ? jitter->jitter : -jitter->jitter;
            if (abs_jitter > metrics->max_jitter) {
                metrics->max_jitter = abs_jitter;
            }
        }
        xSemaphoreGive(metrics_mutex);
    }
}

/**
 * @brief Configurar modo de scheduling y ajustar prioridades de tareas
 */
static void configure_scheduling_mode(scheduling_mode_t mode) {
    current_scheduling_mode = mode;
    
    context_switch_count = 0;
    
    switch (mode) {
        case SCHED_ROUND_ROBIN:
            if (led_task_handle) vTaskPrioritySet(led_task_handle, 2);
            if (calc_task_handle) vTaskPrioritySet(calc_task_handle, 2);
            if (sensor_task_handle) vTaskPrioritySet(sensor_task_handle, 2);
            ESP_LOGI(TAG, "Modo de scheduling: ROUND_ROBIN");
            break;
            
        case SCHED_PRIORITY_BASED:
            if (sensor_task_handle) vTaskPrioritySet(sensor_task_handle, 4);
            if (led_task_handle) vTaskPrioritySet(led_task_handle, 3);
            if (calc_task_handle) vTaskPrioritySet(calc_task_handle, 2);
            ESP_LOGI(TAG, "Modo de scheduling: PRIORITY_BASED");
            break;
            
        case SCHED_PREEMPTIVE_COOP:
            if (led_task_handle) vTaskPrioritySet(led_task_handle, 3);
            if (calc_task_handle) vTaskPrioritySet(calc_task_handle, 3);
            if (sensor_task_handle) vTaskPrioritySet(sensor_task_handle, 3);
            ESP_LOGI(TAG, "Modo de scheduling: PREEMPTIVE_COOPERATIVE");
            break;
    }
}

/**
 * @brief Simular scheduling cooperativo cediendo periódicamente
 * @note En esta simulación, las tareas cooperativas ceden explícitamente usando taskYIELD()
 *       en lugar de vTaskDelay, lo que se acerca más a un modelo cooperativo puro.
 */
static void cooperative_yield(void) {
    if (current_scheduling_mode == SCHED_PREEMPTIVE_COOP) {
        taskYIELD();
    }
}

// =============================================================================
// IMPLEMENTACIONES DE TAREAS
// =============================================================================

/**
 * @brief Tarea de Control LED - Control GPIO con timing preciso
 */
static void led_task(void *param) {
    static uint64_t last_trigger_time = 0;
    static bool led_state = false;
    
    ESP_LOGI(TAG, "LED Task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
        
        uint64_t event_time = get_timestamp_us();
        
        if (last_trigger_time != 0) {
            uint64_t expected_time = last_trigger_time + (LED_PERIOD_MS * 1000);
            record_jitter(&system_metrics.led_metrics, expected_time, event_time);
        }
        
        vTaskDelay(pdMS_TO_TICKS(LED_RESPONSE_TIME_SIM_DELAY_MS));
        uint64_t start_time = get_timestamp_us();
        
        gpio_set_level(LED_GPIO, led_state ? 1 : 0);
        led_state = !led_state;
        
        for (int i = 0; i < 100; i++) {
            __asm__ __volatile__("nop");
        }
        
        cooperative_yield();
        
        uint64_t end_time = get_timestamp_us();
        record_task_timing(&system_metrics.led_metrics, 
                         event_time, start_time, end_time);
        
        last_trigger_time = event_time;
    }
}

/**
 * @brief Tarea de Cálculo Intensivo en CPU - Simulación FFT
 */
static void calculation_task(void *param) {
    ESP_LOGI(TAG, "Calculation Task started");
    
    static float signal[FFT_SIZE];
    static float result[FFT_SIZE];
    
    while (1) {
        uint64_t event_time = get_timestamp_us();
        
        vTaskDelay(pdMS_TO_TICKS((esp_random() % (CALC_TASK_DELAY_RANDOM_MS_MAX - CALC_TASK_DELAY_RANDOM_MS_MIN + 1)) + CALC_TASK_DELAY_RANDOM_MS_MIN));
        
        uint64_t start_time = get_timestamp_us();
        
        int variable_iterations = CALCULATION_ITERATIONS + (esp_random() % (CALC_ITERATIONS_RANDOM_MAX + 1));
        
        for (int iter = 0; iter < variable_iterations; iter++) {
            float amplitude = 0.8f + (esp_random() % (CALC_AMPLITUDE_RANDOM_MAX + 1)) / 100.0f;
            for (int i = 0; i < FFT_SIZE; i++) {
                signal[i] = amplitude * sinf(2.0f * M_PI * i / FFT_SIZE) + 
                           0.5f * cosf(4.0f * M_PI * i / FFT_SIZE);
            }
            
            for (int i = 0; i < FFT_SIZE; i++) {
                result[i] = 0;
                for (int j = 0; j < FFT_SIZE; j++) {
                    result[i] += signal[j] * cosf(2.0f * M_PI * i * j / FFT_SIZE);
                }
            }
            
            if (iter % 2 == 0) {
                vTaskDelay(pdMS_TO_TICKS(1 + (esp_random() % (CALC_YIELD_RANDOM_MS_MAX + 1))));
            }
            cooperative_yield();
        }
        
        uint64_t end_time = get_timestamp_us();
        record_task_timing(&system_metrics.calc_metrics, 
                         event_time, start_time, end_time);
        
        vTaskDelay(pdMS_TO_TICKS(CPU_CALC_PERIOD_MS + (esp_random() % (CALC_PERIOD_RANDOM_MS_MAX + 1))));
    }
}

/**
 * @brief Tarea de Lectura de Sensor - Simulación temperatura I2C
 */
static void sensor_task(void *param) {
    static uint64_t last_trigger_time = 0;
    
    ESP_LOGI(TAG, "Sensor Task started");
    
    sensor_init();
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS + (esp_random() % (SENSOR_PERIOD_RANDOM_MS_MAX + 1))));
        
        uint64_t event_time = get_timestamp_us();
        
        vTaskDelay(pdMS_TO_TICKS((esp_random() % (SENSOR_TASK_DELAY_RANDOM_MS_MAX - SENSOR_TASK_DELAY_RANDOM_MS_MIN + 1)) + SENSOR_TASK_DELAY_RANDOM_MS_MIN));
        
        uint64_t start_time = get_timestamp_us();

        if (last_trigger_time != 0) {
            uint64_t expected_time = last_trigger_time + (SENSOR_PERIOD_MS * 1000);
            record_jitter(&system_metrics.sensor_metrics, expected_time, event_time);
        }
        
        int read_cycles = 1 + (esp_random() % (SENSOR_READ_CYCLES_RANDOM_MAX + 1));
        
        for (int cycle = 0; cycle < read_cycles; cycle++) {
            sensor_data_t sensor_data;
            esp_err_t ret = sensor_read_data(&sensor_data);
            
            if (ret == ESP_OK && sensor_data.valid) {
                for (int i = 0; i < 500 + (esp_random() % (SENSOR_PROCESSING_Nops_RANDOM_MAX + 1)); i++) {
                    __asm__ __volatile__("nop");
                }
                
                ESP_LOGD(TAG, "Sensor data: T=%.2f°C, H=%.2f%%, P=%.2f hPa", 
                         sensor_data.temperature, sensor_data.humidity, sensor_data.pressure);
            } else {
                ESP_LOGW(TAG, "Failed to read sensor data");
            }
            
            if (cycle < read_cycles - 1) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        
        cooperative_yield();
        
        for (int i = 0; i < 50; i++) {
            __asm__ __volatile__("nop");
        }
        
        uint64_t end_time = get_timestamp_us();
        record_task_timing(&system_metrics.sensor_metrics, 
                         event_time, start_time, end_time);
        
        last_trigger_time = event_time;
    }
}

// =============================================================================
// METRICS AND STATISTICS
// =============================================================================
/**
 * @brief Calcular promedio de muestras de timing
 */
static uint64_t calculate_average(uint64_t total, uint32_t count) {
    return count > 0 ? total / count : 0;
}

/**
 * @brief Calcular estadísticas de jitter
 */
static void calculate_jitter_stats(task_metrics_t* metrics, 
                                  uint64_t* avg_jitter, uint64_t* max_jitter) {
    if (metrics->sample_count < 2) {
        *avg_jitter = 0;
        *max_jitter = 0;
        return;
    }
    
    uint32_t samples_to_use = (metrics->sample_count < MAX_METRICS_SAMPLES) ? 
                              metrics->sample_count : MAX_METRICS_SAMPLES;
    
    uint64_t sum_cpu_time = 0;
    for (uint32_t i = 0; i < samples_to_use; i++) {
        sum_cpu_time += metrics->timings[i].cpu_time;
    }
    uint64_t avg_cpu_time = sum_cpu_time / samples_to_use;
    
    uint64_t total_jitter = 0;
    *max_jitter = 0;
    
    for (uint32_t i = 0; i < samples_to_use; i++) {
        uint64_t abs_jitter = (metrics->timings[i].cpu_time > avg_cpu_time) ?
                             (metrics->timings[i].cpu_time - avg_cpu_time) :
                             (avg_cpu_time - metrics->timings[i].cpu_time);
        total_jitter += abs_jitter;
        if (abs_jitter > *max_jitter) {
            *max_jitter = abs_jitter;
        }
    }
    
    *avg_jitter = total_jitter / samples_to_use;
}

/**
 * @brief Mostrar métricas detalladas de tarea
 */
static void print_task_metrics(task_metrics_t* metrics) {
    if (metrics->sample_count == 0) {
        printf("| %-12s | %s\n", metrics->task_name, "No samples recorded");
        return;
    }
    
    uint64_t avg_response = calculate_average(metrics->total_response_time, metrics->sample_count);
    uint64_t avg_turnaround = calculate_average(metrics->total_turnaround_time, metrics->sample_count);
    uint64_t avg_cpu = calculate_average(metrics->total_cpu_time, metrics->sample_count);
    
    uint64_t avg_jitter, max_jitter;
    calculate_jitter_stats(metrics, &avg_jitter, &max_jitter);
    
    printf("| %-12s | %8" PRIu64 " | %8" PRIu64 " | %8" PRIu64 " | %8" PRIu64 " | %8" PRIu64 " | %8" PRIu64 " | %8" PRIu64 " | %8" PRIu32 " |\n",
           metrics->task_name,
           avg_response,
           metrics->max_response_time,
           avg_turnaround,
           metrics->max_turnaround_time,
           avg_cpu,
           avg_jitter,
           max_jitter,
           metrics->sample_count);
}

/**
 * @brief Mostrar dashboard integral del sistema
 */
static void display_dashboard(void) {
    printf("\n" COLOR_CYAN COLOR_BOLD 
           "===================================================================\n"
           "            ESP32 FreeRTOS SCHEDULING EVALUATION DASHBOARD\n"
           "===================================================================\n" COLOR_RESET);
    
    printf(COLOR_GREEN "System Information:\n" COLOR_RESET);
    printf("  Scheduling Mode: ");
    switch (current_scheduling_mode) {
        case SCHED_ROUND_ROBIN:
            printf("Round-Robin\n");
            break;
        case SCHED_PRIORITY_BASED:
            printf("Priority-Based\n");
            break;
        case SCHED_PREEMPTIVE_COOP:
            printf("Preemptive/Cooperative\n");
            break;
    }
    
    uint64_t uptime = (get_timestamp_us() - system_metrics.system_start_time) / 1000000;
    printf("  System Uptime: %llu seconds\n", uptime);
    
    update_context_switches();
    system_metrics.total_context_switches = context_switch_count;
    printf("  Total Context Switches: %llu\n", system_metrics.total_context_switches);
    
    printf("\n" COLOR_YELLOW "Task Performance Metrics (all times in microseconds):\n" COLOR_RESET);
    printf("| Task         | Avg Resp | Max Resp | Avg Turn | Max Turn | Avg CPU  | Avg Jit  | Max Jit  | Samples  |\n");
    printf("|--------------|----------|----------|----------|----------|----------|----------|----------|----------|\n");
    
    if (xSemaphoreTake(metrics_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        print_task_metrics(&system_metrics.led_metrics);
        print_task_metrics(&system_metrics.calc_metrics);
        print_task_metrics(&system_metrics.sensor_metrics);
        xSemaphoreGive(metrics_mutex);
    }
    
    printf("\n" COLOR_MAGENTA "Análisis de Rendimiento:\n" COLOR_RESET);
    
    uint64_t avg_jitter, max_jitter;
    calculate_jitter_stats(&system_metrics.led_metrics, &avg_jitter, &max_jitter);
    if (max_jitter < JITTER_THRESHOLD_US) {
        printf("  " COLOR_GREEN "✅ Jitter razonable para carga mixta (%llu µs)\n" COLOR_RESET, max_jitter);
    } else {
        printf("  " COLOR_YELLOW "❗ Jitter elevado, aceptable para sistemas no críticos (%llu µs)\n" COLOR_RESET, max_jitter);
    }
    
    uint64_t avg_response = calculate_average(system_metrics.led_metrics.total_response_time, 
                                             system_metrics.led_metrics.sample_count);
    if (avg_response < RESPONSE_TIME_THRESHOLD_US) {
        printf("  " COLOR_GREEN "✅ Tiempos de respuesta adecuados para esta carga (%llu µs)\n" COLOR_RESET, avg_response);
    } else {
        printf("  " COLOR_YELLOW "❗ Tiempos de respuesta algo elevados, pero funcionales (%llu µs)\n" COLOR_RESET, avg_response);
    }
    
    printf("  Eficiencia de scheduling actual: ");
    if (current_scheduling_mode == SCHED_PRIORITY_BASED) {
        printf(COLOR_GREEN "ÓPTIMA para tareas de tiempo real\n" COLOR_RESET);
    } else if (current_scheduling_mode == SCHED_ROUND_ROBIN) {
        printf(COLOR_YELLOW "JUSTA pero puede impactar el rendimiento de tiempo real\n" COLOR_RESET);
    } else {
        printf(COLOR_CYAN "COOPERATIVA - depende del comportamiento de las tareas\n" COLOR_RESET);
    }
    
    printf("\n");
}

/**
 * @brief Tarea de visualización de métricas
 */
static void metrics_task(void *param) {
    static uint32_t mode_counter = 0;
    
    ESP_LOGI(TAG, "Metrics Task started");
    
    while (1) {
        display_dashboard();

        if (++mode_counter >= MODE_SWITCH_PERIOD_S) {
            mode_counter = 0;
            scheduling_mode_t next_mode = (current_scheduling_mode + 1) % 3;
            configure_scheduling_mode(next_mode);
        }

        vTaskDelay(pdMS_TO_TICKS(METRICS_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 FreeRTOS Scheduling Evaluation System");
    

    system_metrics.system_start_time = get_timestamp_us();
    system_metrics.current_mode = SCHED_ROUND_ROBIN;
    

    init_task_metrics(&system_metrics.led_metrics, "LED");
    init_task_metrics(&system_metrics.calc_metrics, "CPU_CALC");
    init_task_metrics(&system_metrics.sensor_metrics, "SENSOR");

    metrics_mutex = xSemaphoreCreateMutex();
    if (metrics_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create metrics mutex");
        return;
    }
    
    event_queue = xQueueCreate(10, sizeof(event_type_t));
    if (event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED GPIO: %s", esp_err_to_name(ret));
        return;
    }
    
    gpio_set_level(LED_GPIO, 0);
    
    BaseType_t task_result;
    
    task_result = xTaskCreatePinnedToCore(
        led_task,
        "LED_Task",
        LED_TASK_STACK_SIZE,
        NULL,
        LED_TASK_PRIORITY,
        &led_task_handle,
        APP_CPU_NUM
    );
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED task");
        return;
    }
    
    task_result = xTaskCreatePinnedToCore(
        calculation_task,
        "CALC_Task",
        CALC_TASK_STACK_SIZE,
        NULL,
        CALC_TASK_PRIORITY,
        &calc_task_handle,
        APP_CPU_NUM
    );
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create calculation task");
        return;
    }
    
    task_result = xTaskCreatePinnedToCore(
        sensor_task,
        "SENSOR_Task",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        &sensor_task_handle,
        APP_CPU_NUM
    );
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }
    
    task_result = xTaskCreatePinnedToCore(
        metrics_task,
        "METRICS_Task",
        METRICS_TASK_STACK_SIZE,
        NULL,
        METRICS_TASK_PRIORITY,
        &metrics_task_handle,
        APP_CPU_NUM
    );
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create metrics task");
        return;
    }

    configure_scheduling_mode(SCHED_ROUND_ROBIN);
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System initialization complete");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "System running - Context switches: %llu", context_switch_count);
    }
}
