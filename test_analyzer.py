#!/usr/bin/env python3
"""
Test script for the ESP32 FreeRTOS Scheduling Evaluation System

This script provides a simple test of the Python analyzer without requiring
actual hardware. It generates simulated ESP32 output to test the analysis
functionality.
"""

import sys
import time
import random
from datetime import datetime

def generate_sample_output():
    """Generate simulated ESP32 output for testing."""
    
    scheduling_modes = ["ROUND_ROBIN", "PRIORITY_BASED", "PREEMPTIVE_COOPERATIVE"]
    current_mode = random.choice(scheduling_modes)
    
    print(f"Starting ESP32 FreeRTOS Scheduling Evaluation System")
    print(f"Scheduling mode: {current_mode}")
    
    # Generate metrics for about 30 seconds
    for i in range(30):
        # Print scheduling mode periodically
        if i % 10 == 0:
            current_mode = random.choice(scheduling_modes)
            print(f"Scheduling Mode: {current_mode}")
        
        # Generate task metrics
        print("===================================================================")
        print("            ESP32 FreeRTOS SCHEDULING EVALUATION DASHBOARD")
        print("===================================================================")
        print(f"System Information:")
        print(f"  Scheduling Mode: {current_mode}")
        print(f"  System Uptime: {i} seconds")
        print(f"  Total Context Switches: {i * 50}")
        print()
        print("Task Performance Metrics (all times in microseconds):")
        print("| Task         | Avg Resp | Max Resp | Avg Turn | Max Turn | Avg CPU  | Avg Jit  | Max Jit  | Samples  |")
        print("|--------------|----------|----------|----------|----------|----------|----------|----------|----------|")
        
        # LED task metrics
        led_avg_resp = 1000 + random.randint(-200, 200)
        led_max_resp = led_avg_resp + random.randint(500, 1000)
        led_avg_turn = led_avg_resp + random.randint(400, 800)
        led_max_turn = led_avg_turn + random.randint(800, 1200)
        led_avg_cpu = random.randint(400, 800)
        led_avg_jit = random.randint(20, 100)
        led_max_jit = led_avg_jit + random.randint(50, 150)
        led_samples = min(i * 10, 100)
        
        print(f"| LED_CTRL     | {led_avg_resp:8d} | {led_max_resp:8d} | {led_avg_turn:8d} | {led_max_turn:8d} | {led_avg_cpu:8d} | {led_avg_jit:8d} | {led_max_jit:8d} | {led_samples:8d} |")
        
        # CPU task metrics
        cpu_avg_resp = 2000 + random.randint(-500, 500)
        cpu_max_resp = cpu_avg_resp + random.randint(1000, 2000)
        cpu_avg_turn = 120000 + random.randint(-20000, 20000)
        cpu_max_turn = cpu_avg_turn + random.randint(30000, 60000)
        cpu_avg_cpu = cpu_avg_turn - cpu_avg_resp
        cpu_avg_jit = random.randint(10, 50)
        cpu_max_jit = cpu_avg_jit + random.randint(30, 100)
        cpu_samples = min(i * 2, 20)
        
        print(f"| CPU_CALC     | {cpu_avg_resp:8d} | {cpu_max_resp:8d} | {cpu_avg_turn:8d} | {cpu_max_turn:8d} | {cpu_avg_cpu:8d} | {cpu_avg_jit:8d} | {cpu_max_jit:8d} | {cpu_samples:8d} |")
        
        # Sensor task metrics
        sensor_avg_resp = 800 + random.randint(-200, 200)
        sensor_max_resp = sensor_avg_resp + random.randint(400, 800)
        sensor_avg_turn = 5000 + random.randint(-1000, 1000)
        sensor_max_turn = sensor_avg_turn + random.randint(2000, 4000)
        sensor_avg_cpu = sensor_avg_turn - sensor_avg_resp
        sensor_avg_jit = random.randint(30, 120)
        sensor_max_jit = sensor_avg_jit + random.randint(100, 200)
        sensor_samples = min(i * 5, 50)
        
        print(f"| SENSOR_RD    | {sensor_avg_resp:8d} | {sensor_max_resp:8d} | {sensor_avg_turn:8d} | {sensor_max_turn:8d} | {sensor_avg_cpu:8d} | {sensor_avg_jit:8d} | {sensor_max_jit:8d} | {sensor_samples:8d} |")
        
        # Performance analysis
        print()
        print("Performance Analysis:")
        if led_max_jit > 500 or sensor_max_jit > 500:
            print("  ⚠  High jitter detected in periodic tasks")
        else:
            print("  ✓  Jitter within acceptable limits")
        
        if max(led_max_resp, cpu_max_resp, sensor_max_resp) > 10000:
            print("  ⚠  High response times detected")
        else:
            print("  ✓  Response times within acceptable limits")
        
        if current_mode == "PRIORITY_BASED":
            print("  ✓  OPTIMAL for real-time tasks")
        elif current_mode == "ROUND_ROBIN":
            print("  ⚠  FAIR but may impact real-time performance")
        else:
            print("  ⚠  COOPERATIVE - depends on task behavior")
        
        print()
        
        # Also generate FreeRTOS stats periodically
        if i % 5 == 0:
            print("=== Enhanced FreeRTOS Statistics ===")
            print("| Task Name    | Run Time | Percentage | Priority | Stack HWM |")
            print("|--------------|----------|------------|----------|----------|")
            
            tasks = ["led_task", "calc_task", "sensor_task", "metrics_task", "IDLE0", "IDLE1"]
            total_runtime = 1000000
            
            for task in tasks:
                if task.startswith("IDLE"):
                    runtime = random.randint(50000, 200000)
                    priority = 0
                    stack_hwm = random.randint(200, 500)
                else:
                    runtime = random.randint(10000, 100000)
                    priority = random.randint(2, 5)
                    stack_hwm = random.randint(800, 2000)
                
                percentage = (runtime * 100) // total_runtime
                print(f"| {task:<12s} | {runtime:8d} | {percentage:8d}% | {priority:8d} | {stack_hwm:8d} |")
            
            print("Real time stats obtained successfully")
            print()
        
        time.sleep(1)

if __name__ == "__main__":
    try:
        generate_sample_output()
    except KeyboardInterrupt:
        print("\nTest output generation stopped")
