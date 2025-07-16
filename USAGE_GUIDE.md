# ESP32 FreeRTOS Scheduling Evaluation System - Usage Guide

## 🚀 Quick Start Guide

### Step 1: Hardware Setup
1. **Connect the LED**: Connect an LED with a current-limiting resistor (220Ω-1kΩ) between GPIO 32 and GND
2. **Optional - Connect I2C sensor**: Connect SDA to GPIO 21, SCL to GPIO 22
3. **Power the ESP32**: Connect via USB cable to your development machine

### Step 2: Build and Flash
```bash
# Navigate to project directory
cd /workspaces/project-name

# Build the project
idf.py build

# Flash and monitor (replace ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash monitor
```

### Step 3: Monitor Output
The system will display:
- Real-time metrics dashboard every 1 second
- Automatic scheduling mode switching every 10 seconds
- Enhanced FreeRTOS statistics every 5 seconds

## 📊 Understanding the Output

### Dashboard Components

#### 1. System Information
```
System Information:
  Scheduling Mode: ROUND_ROBIN
  System Uptime: 45 seconds
  Total Context Switches: 2250
```

#### 2. Task Performance Metrics
```
| Task         | Avg Resp | Max Resp | Avg Turn | Max Turn | Avg CPU  | Avg Jit  | Max Jit  | Samples  |
|--------------|----------|----------|----------|----------|----------|----------|----------|----------|
| LED_CTRL     | 1250     | 2100     | 1890     | 3200     | 640      | 45       | 120      | 98       |
| CPU_CALC     | 2300     | 4500     | 125000   | 180000   | 122700   | 12       | 89       | 19       |
| SENSOR_RD    | 1100     | 1800     | 6200     | 8900     | 5100     | 67       | 234      | 49       |
```

**Metric Definitions:**
- **Avg Resp**: Average response time (μs) - time from event to task start
- **Max Resp**: Maximum response time observed (μs)
- **Avg Turn**: Average turnaround time (μs) - time from event to task completion
- **Max Turn**: Maximum turnaround time observed (μs)
- **Avg CPU**: Average CPU processing time (μs)
- **Avg Jit**: Average jitter (μs) - timing variation in periodic tasks
- **Max Jit**: Maximum jitter observed (μs)
- **Samples**: Number of measurements collected

#### 3. Performance Analysis
```
Performance Analysis:
  ✓  Jitter within acceptable limits
  ✓  Response times within acceptable limits
  ✓  OPTIMAL for real-time tasks
```

**Indicators:**
- ✅ **Green checkmark**: System operating normally
- ⚠️ **Yellow warning**: Potential performance issues
- ❌ **Red error**: Performance degradation detected

## 🔧 Customization Options

### Changing Task Timing
Edit `main/config_sched_eval.h`:
```c
#define LED_PERIOD_MS           100     // LED blink every 100ms
#define CALC_PERIOD_MS          500     // CPU task every 500ms
#define SENSOR_PERIOD_MS        200     // Sensor read every 200ms
#define METRICS_PERIOD_MS       1000    // Dashboard update every 1s
```

### Adjusting Task Priorities
```c
#define LED_TASK_PRIORITY       3       // LED task priority
#define CALC_TASK_PRIORITY      2       // Calculation task priority
#define SENSOR_TASK_PRIORITY    4       // Sensor task priority
#define METRICS_TASK_PRIORITY   5       // Metrics task priority
```

### Modifying Performance Thresholds
```c
#define RESPONSE_TIME_THRESHOLD 5000    // 5ms response time warning
#define JITTER_THRESHOLD_US     1000    // 1ms jitter warning
#define CPU_UTILIZATION_THRESHOLD 80    // 80% CPU utilization warning
```

### Enabling/Disabling Features
```c
#define AUTO_SWITCH_MODES       1       // Auto-switch scheduling modes
#define METRICS_COLLECTION_ENABLED 1    // Collect detailed metrics
#define COLOR_OUTPUT_ENABLED    1       // Colored terminal output
#define SENSOR_SIMULATION_MODE  1       // Use simulated sensors
```

## 🔄 Scheduling Modes

### Round-Robin Scheduling
- **Characteristics**: All tasks have equal priority
- **When used**: Every 10 seconds in auto-switch mode
- **Advantages**: Fair CPU distribution, predictable behavior
- **Disadvantages**: May miss real-time deadlines

### Priority-Based Scheduling
- **Characteristics**: Tasks have different priorities (Sensor > LED > CPU)
- **When used**: Automatically switches every 10 seconds
- **Advantages**: Meets real-time requirements, predictable response
- **Disadvantages**: Lower priority tasks may starve

### Preemptive/Cooperative Scheduling
- **Characteristics**: Tasks voluntarily yield CPU control
- **When used**: Demonstrates cooperative multitasking
- **Advantages**: Reduced context switch overhead
- **Disadvantages**: Depends on task cooperation

## 📈 Performance Analysis

### Good Performance Indicators
- **Response times < 5ms**: System is responsive
- **Jitter < 1ms**: Timing is consistent
- **CPU utilization < 80%**: System not overloaded
- **Balanced task execution**: All tasks getting fair CPU time

### Performance Issues
- **High response times**: Tasks are delayed
- **High jitter**: Inconsistent timing
- **Task starvation**: Some tasks not executing
- **Memory issues**: Stack overflow or heap exhaustion

### Optimization Strategies
1. **Adjust task priorities** based on real-time requirements
2. **Increase task frequencies** for better responsiveness
3. **Reduce calculation complexity** to lower CPU usage
4. **Enable cooperative yields** to reduce context switches

## 🛠 Advanced Usage

### Data Analysis with Python
```bash
# Install required packages
pip install pyserial matplotlib numpy

# Run the analyzer (30-second collection)
python3 scheduling_analyzer.py --port /dev/ttyUSB0 --duration 30 --visualize --export-csv

# Test with simulated data
python3 test_analyzer.py | python3 scheduling_analyzer.py --port - --duration 10
```

### Custom Sensor Integration
1. **Modify sensor.c** to support your specific sensor
2. **Update I2C configuration** in config_sched_eval.h
3. **Disable simulation mode**: Set `SENSOR_SIMULATION_MODE 0`
4. **Add sensor initialization** in sensor_init()

### Adding New Tasks
1. **Create task function** following existing patterns
2. **Add task handle** and timer
3. **Update metrics collection** structure
4. **Modify scheduling mode** configuration

## 🔍 Troubleshooting

### Common Issues

#### LED Not Blinking
- **Check GPIO connection**: Verify LED is connected to GPIO 32
- **Check LED polarity**: Ensure LED is connected correctly
- **Check resistor value**: Use 220Ω-1kΩ resistor

#### High Jitter Values
- **Reduce system load**: Lower calculation task complexity
- **Increase task priorities**: Give higher priority to time-critical tasks
- **Check for interrupts**: Disable unnecessary interrupts

#### Memory Issues
- **Increase heap size**: Configure in `idf.py menuconfig`
- **Reduce sample buffer**: Lower `MAX_METRICS_SAMPLES`
- **Check stack usage**: Monitor stack high water marks

#### I2C Sensor Issues
- **Check connections**: Verify SDA (21) and SCL (22) connections
- **Check pull-up resistors**: I2C requires pull-up resistors (4.7kΩ)
- **Verify sensor address**: Check sensor documentation for correct address

### Debug Options

#### Enable Detailed Logging
```c
#define LOG_LEVEL_MAIN          ESP_LOG_DEBUG
#define DEBUG_TIMING_ENABLED    1
#define DEBUG_TASK_SWITCHES     1
```

#### Monitor Stack Usage
```c
#define STACK_MONITORING_ENABLED 1
#define STACK_WARNING_THRESHOLD  80
```

#### Check Task Watchdog
```c
#define TASK_WATCHDOG_ENABLED   1
#define WATCHDOG_TIMEOUT_MS     5000
```

## 📚 Understanding the Results

### Scheduling Mode Comparison
- **Round-Robin**: Best for equal importance tasks
- **Priority-Based**: Best for real-time applications
- **Cooperative**: Best for low-overhead applications

### Task Behavior Analysis
- **LED Task**: Should have low, consistent timing
- **CPU Task**: Higher response times, lower jitter
- **Sensor Task**: Moderate response times, I/O dependent

### System Health Indicators
- **Context Switches**: Should be balanced, not excessive
- **Stack Usage**: Should be well below limits
- **Memory Usage**: Should be stable over time

## 🎯 Best Practices

1. **Start with default configuration** and modify gradually
2. **Monitor system performance** before making changes
3. **Test with realistic workloads** for your application
4. **Document configuration changes** for reproducibility
5. **Use version control** for configuration management

## 💡 Tips for Real-World Applications

1. **Calibrate timing thresholds** based on your requirements
2. **Consider power consumption** when setting task frequencies
3. **Plan for worst-case scenarios** in timing analysis
4. **Test under various conditions** (temperature, voltage, etc.)
5. **Implement graceful degradation** for overload conditions

## 🚨 Safety Considerations

- **Never exceed GPIO current limits** (40mA per pin)
- **Use appropriate resistors** for LED circuits
- **Check sensor voltage levels** before connection
- **Monitor system temperature** during extended testing
- **Implement proper error handling** for production use
