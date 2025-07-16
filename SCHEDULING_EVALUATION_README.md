# ESP32 FreeRTOS Scheduling Evaluation System

A comprehensive system for evaluating different scheduling strategies in FreeRTOS running on ESP32, with realistic tasks and real-time metrics collection.

## 📋 Features

### 🎯 Realistic Task Implementation
- **LED Control Task**: Precise GPIO control on pin 32 with timing measurements
- **CPU-Intensive Task**: FFT-like mathematical computations simulating real workloads
- **Sensor Reading Task**: I2C communication simulation with temperature sensor emulation

### 📊 Comprehensive Metrics Dashboard
- **Response Time**: Time from event occurrence to task execution start
- **Turnaround Time**: Total time from event to task completion
- **CPU Time**: Actual processing time within tasks
- **Jitter**: Timing variation in periodic tasks
- **Context Switch Overhead**: System-level scheduling overhead
- **Priority Inversion Detection**: Real-time system anomaly detection

### 🔄 Scheduling Strategy Evaluation
- **Round-Robin**: Equal priority fair scheduling
- **Priority-Based**: Different task priorities (Sensor > LED > CPU)
- **Preemptive/Cooperative**: Hybrid scheduling with voluntary yields

## 🛠 Hardware Requirements

### Essential Components
- **ESP32 DevKitC-VE** development board
- **LED** connected to GPIO 32 (with appropriate resistor)
- **USB cable** for programming and monitoring

### Optional Components (for enhanced functionality)
- **I2C Temperature Sensor** (e.g., DS18B20, SHT30)
  - SDA: GPIO 21
  - SCL: GPIO 22
  - VCC: 3.3V
  - GND: GND

## 🚀 Quick Start

### 1. Hardware Setup
```
ESP32 DevKitC-VE Connections:
┌─────────────────┐
│     ESP32       │
│                 │
│ GPIO 32 ──────── LED (+ resistor to GND)
│ GPIO 21 ──────── I2C SDA (optional sensor)
│ GPIO 22 ──────── I2C SCL (optional sensor)
│ 3.3V ──────────── Sensor VCC (if used)
│ GND ───────────── Sensor GND (if used)
└─────────────────┘
```

### 2. Build and Flash
```bash
# Navigate to project directory
cd /workspaces/project-name

# Configure project (if needed)
idf.py menuconfig

# Build the project
idf.py build

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash monitor
```

### 3. Monitor Output
The system will display:
- Real-time metrics dashboard every second
- Automatic scheduling mode switching every 10 seconds
- Enhanced FreeRTOS statistics every 5 seconds

## 📊 Understanding the Metrics

### Task Performance Metrics Table
```
| Task         | Avg Resp | Max Resp | Avg Turn | Max Turn | Avg CPU  | Avg Jit  | Max Jit  | Samples  |
|--------------|----------|----------|----------|----------|----------|----------|----------|----------|
| LED_CTRL     | 1250     | 2100     | 1890     | 3200     | 640      | 45       | 120      | 98       |
| CPU_CALC     | 2300     | 4500     | 125000   | 180000   | 122700   | 12       | 89       | 19       |
| SENSOR_RD    | 1100     | 1800     | 6200     | 8900     | 5100     | 67       | 234      | 49       |
```

**Column Explanations:**
- **Avg Resp**: Average response time (event to execution start)
- **Max Resp**: Maximum response time observed
- **Avg Turn**: Average turnaround time (event to completion)
- **Max Turn**: Maximum turnaround time observed
- **Avg CPU**: Average CPU processing time
- **Avg Jit**: Average timing jitter for periodic tasks
- **Max Jit**: Maximum jitter observed
- **Samples**: Number of timing samples collected

### Performance Analysis Indicators
- ✅ **Green**: System operating within acceptable parameters
- ⚠️ **Yellow**: Warning - potential performance issues
- ❌ **Red**: Critical - performance degradation detected

## 🔧 Configuration Options

### Task Timing Configuration
```c
#define LED_PERIOD_MS           100    // LED blink frequency
#define CALC_PERIOD_MS          500    // CPU task frequency
#define SENSOR_PERIOD_MS        200    // Sensor reading frequency
#define METRICS_PERIOD_MS       1000   // Dashboard update frequency
```

### Scheduling Mode Selection
```c
typedef enum {
    SCHED_ROUND_ROBIN = 0,      // Fair scheduling
    SCHED_PRIORITY_BASED,       // Priority-based scheduling
    SCHED_PREEMPTIVE_COOP      // Cooperative scheduling
} scheduling_mode_t;
```

### Performance Thresholds
```c
#define JITTER_THRESHOLD_US     1000   // Jitter warning threshold
#define MAX_METRICS_SAMPLES     100    // Sample buffer size
#define CALCULATION_ITERATIONS  1000   // CPU task intensity
```

## 📈 Scheduling Strategy Analysis

### Round-Robin Scheduling
- **Characteristics**: All tasks have equal priority
- **Advantages**: Fair CPU distribution, simple implementation
- **Disadvantages**: May not meet real-time deadlines
- **Best for**: Non-critical applications, development/testing

### Priority-Based Scheduling
- **Characteristics**: Tasks have different priorities (Sensor > LED > CPU)
- **Advantages**: Predictable real-time behavior, meets deadlines
- **Disadvantages**: Lower priority tasks may starve
- **Best for**: Real-time applications, critical timing requirements

### Preemptive/Cooperative Scheduling
- **Characteristics**: Tasks voluntarily yield CPU control
- **Advantages**: Reduced context switch overhead
- **Disadvantages**: Depends on task behavior, potential blocking
- **Best for**: Cooperative multitasking, reduced system overhead

## 🔍 Advanced Features

### Real-Time Metrics Collection
The system uses `esp_timer_get_time()` for microsecond precision timing:
```c
uint64_t event_time = esp_timer_get_time();
uint64_t start_time = esp_timer_get_time();
// ... task execution ...
uint64_t end_time = esp_timer_get_time();
```

### Jitter Analysis
Periodic tasks measure timing variations:
```c
static void record_jitter(task_metrics_t* metrics, 
                         uint64_t expected_time, 
                         uint64_t actual_time) {
    int64_t jitter = (int64_t)actual_time - (int64_t)expected_time;
    // Record and analyze jitter patterns
}
```

### Context Switch Monitoring
The system tracks context switches and calculates overhead:
```c
// Context switch detection through task state monitoring
// Overhead calculation based on timing differences
```

## 📝 Customization Guide

### Adding New Tasks
1. Define task function following the existing pattern
2. Add task handle and metrics structure
3. Create timer callback for task triggering
4. Update metrics collection and display

### Modifying Scheduling Strategies
1. Update `configure_scheduling_mode()` function
2. Add new scheduling mode to enum
3. Implement custom priority assignment logic
4. Update analysis and recommendations

### Extending Hardware Support
1. Add hardware initialization in `init_scheduling_system()`
2. Create device-specific task implementations
3. Update metrics collection for new hardware
4. Add configuration options for new peripherals

## 🐛 Troubleshooting

### Common Issues
1. **LED not blinking**: Check GPIO 32 connection and resistor
2. **I2C errors**: Verify SDA/SCL connections and pull-up resistors
3. **Memory allocation fails**: Increase heap size in menuconfig
4. **High jitter values**: Check for interrupt conflicts or CPU overload

### Debug Output
Enable detailed logging by setting log level:
```c
esp_log_level_set("SCHED_EVAL", ESP_LOG_DEBUG);
```

### Performance Optimization
- Adjust `CALCULATION_ITERATIONS` for CPU task intensity
- Modify `MAX_METRICS_SAMPLES` for memory usage
- Tune task priorities for specific requirements

## 📚 Technical Details

### Architecture Overview
```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 FreeRTOS System                    │
├─────────────────────────────────────────────────────────────┤
│  Timers        │  Tasks          │  Metrics Collection      │
│  ├─ LED Timer  │  ├─ LED Task    │  ├─ Response Time        │
│  ├─ CPU Timer  │  ├─ CPU Task    │  ├─ Turnaround Time      │
│  ├─ Sensor Timer│  ├─ Sensor Task│  ├─ CPU Time             │
│  └─ Metrics Timer│  └─ Metrics Task│  └─ Jitter Analysis     │
├─────────────────────────────────────────────────────────────┤
│                    Hardware Layer                           │
│  GPIO 32 (LED) │ I2C Bus (Sensor) │ High-Resolution Timer   │
└─────────────────────────────────────────────────────────────┘
```

### Memory Usage
- **LED Task**: ~2KB stack
- **CPU Task**: ~4KB stack (for FFT calculations)
- **Sensor Task**: ~2KB stack
- **Metrics Task**: ~4KB stack
- **Total RAM**: ~15KB for tasks + metrics storage

### Timing Precision
- **Timer Resolution**: 1 microsecond (ESP32 hardware timer)
- **Measurement Accuracy**: ±1-2 microseconds
- **Jitter Detection**: Sub-millisecond precision

## 🤝 Contributing

### Development Guidelines
1. Follow existing code structure and naming conventions
2. Add comprehensive comments for new features
3. Update documentation for any changes
4. Test thoroughly on actual hardware

### Future Enhancements
- [ ] Web-based dashboard for remote monitoring
- [ ] Data logging to SD card or flash storage
- [ ] Additional scheduling algorithms (EDF, RMS)
- [ ] Multi-core scheduling evaluation
- [ ] Power consumption analysis

## 📄 License

This project is based on the ESP-IDF FreeRTOS example and is provided under the same license terms.

## 🙋 Support

For questions or issues:
1. Check the troubleshooting section
2. Review ESP-IDF documentation
3. Examine the code comments for implementation details
4. Test with minimal hardware setup first
