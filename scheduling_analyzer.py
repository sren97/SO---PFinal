#!/usr/bin/env python3
"""
ESP32 FreeRTOS Scheduling Evaluation - Data Analysis Tool

This script provides advanced analysis and visualization of the scheduling
evaluation system's output data.

Features:
- Real-time serial data parsing
- Performance metrics visualization
- Scheduling strategy comparison
- Jitter analysis and histograms
- Export to CSV/JSON formats
"""

import serial
import time
import json
import csv
import re
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import argparse
import threading
import queue
from collections import defaultdict, deque

class SchedulingAnalyzer:
    """Main class for analyzing ESP32 scheduling evaluation data."""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, buffer_size=1000):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        self.running = False
        self.data_queue = queue.Queue()
        
        # Data storage
        self.task_metrics = defaultdict(lambda: {
            'response_times': deque(maxlen=buffer_size),
            'turnaround_times': deque(maxlen=buffer_size),
            'cpu_times': deque(maxlen=buffer_size),
            'jitter_values': deque(maxlen=buffer_size),
            'timestamps': deque(maxlen=buffer_size)
        })
        
        self.scheduling_modes = []
        self.mode_transitions = []
        self.current_mode = None
        
        # Analysis results
        self.analysis_results = {}
        
    def start_data_collection(self):
        """Start collecting data from ESP32 via serial."""
        self.running = True
        self.serial_thread = threading.Thread(target=self._serial_reader)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.parser_thread = threading.Thread(target=self._data_parser)
        self.parser_thread.daemon = True
        self.parser_thread.start()
        
        print(f"Started data collection on {self.port} at {self.baudrate} baud")
    
    def stop_data_collection(self):
        """Stop data collection."""
        self.running = False
        if hasattr(self, 'serial_thread'):
            self.serial_thread.join(timeout=1.0)
        if hasattr(self, 'parser_thread'):
            self.parser_thread.join(timeout=1.0)
        print("Data collection stopped")
    
    def _serial_reader(self):
        """Thread function to read serial data."""
        try:
            with serial.Serial(self.port, self.baudrate, timeout=1) as ser:
                while self.running:
                    try:
                        line = ser.readline().decode('utf-8').strip()
                        if line:
                            self.data_queue.put(line)
                    except UnicodeDecodeError:
                        continue
                    except Exception as e:
                        print(f"Serial read error: {e}")
                        break
        except Exception as e:
            print(f"Serial connection error: {e}")
    
    def _data_parser(self):
        """Thread function to parse incoming data."""
        metric_pattern = re.compile(
            r'\\|\\s*(\\w+)\\s*\\|\\s*(\\d+)\\s*\\|\\s*(\\d+)\\s*\\|\\s*(\\d+)\\s*\\|\\s*(\\d+)\\s*\\|\\s*(\\d+)\\s*\\|\\s*(\\d+)\\s*\\|\\s*(\\d+)\\s*\\|\\s*(\\d+)\\s*\\|'
        )
        
        mode_pattern = re.compile(r'Scheduling Mode: (\\w+)')
        
        while self.running:
            try:
                line = self.data_queue.get(timeout=1)
                timestamp = datetime.now()
                
                # Parse scheduling mode changes
                mode_match = mode_pattern.search(line)
                if mode_match:
                    new_mode = mode_match.group(1)
                    if new_mode != self.current_mode:
                        self.current_mode = new_mode
                        self.mode_transitions.append((timestamp, new_mode))
                        print(f"Mode change detected: {new_mode}")
                
                # Parse task metrics
                metric_match = metric_pattern.search(line)
                if metric_match:
                    task_name = metric_match.group(1)
                    if task_name in ['LED_CTRL', 'CPU_CALC', 'SENSOR_RD']:
                        metrics = {
                            'avg_response': int(metric_match.group(2)),
                            'max_response': int(metric_match.group(3)),
                            'avg_turnaround': int(metric_match.group(4)),
                            'max_turnaround': int(metric_match.group(5)),
                            'avg_cpu': int(metric_match.group(6)),
                            'avg_jitter': int(metric_match.group(7)),
                            'max_jitter': int(metric_match.group(8)),
                            'samples': int(metric_match.group(9))
                        }
                        
                        self._update_task_metrics(task_name, metrics, timestamp)
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Parser error: {e}")
    
    def _update_task_metrics(self, task_name, metrics, timestamp):
        """Update task metrics with new data."""
        task_data = self.task_metrics[task_name]
        
        task_data['response_times'].append(metrics['avg_response'])
        task_data['turnaround_times'].append(metrics['avg_turnaround'])
        task_data['cpu_times'].append(metrics['avg_cpu'])
        task_data['jitter_values'].append(metrics['avg_jitter'])
        task_data['timestamps'].append(timestamp)
    
    def generate_performance_report(self):
        """Generate comprehensive performance analysis report."""
        report = {
            'timestamp': datetime.now().isoformat(),
            'analysis_period': self._get_analysis_period(),
            'scheduling_modes': self._analyze_scheduling_modes(),
            'task_analysis': {},
            'comparative_analysis': {},
            'recommendations': []
        }
        
        # Analyze each task
        for task_name, data in self.task_metrics.items():
            if len(data['response_times']) > 0:
                report['task_analysis'][task_name] = self._analyze_task_performance(task_name, data)
        
        # Comparative analysis
        report['comparative_analysis'] = self._comparative_analysis()
        
        # Generate recommendations
        report['recommendations'] = self._generate_recommendations()
        
        return report
    
    def _get_analysis_period(self):
        """Get the analysis time period."""
        if not self.task_metrics:
            return "No data collected"
        
        all_timestamps = []
        for task_data in self.task_metrics.values():
            all_timestamps.extend(task_data['timestamps'])
        
        if all_timestamps:
            start_time = min(all_timestamps)
            end_time = max(all_timestamps)
            duration = (end_time - start_time).total_seconds()
            return f"{duration:.1f} seconds"
        
        return "No valid timestamps"
    
    def _analyze_scheduling_modes(self):
        """Analyze scheduling mode performance."""
        mode_analysis = {}
        
        for timestamp, mode in self.mode_transitions:
            if mode not in mode_analysis:
                mode_analysis[mode] = {
                    'activation_count': 0,
                    'total_duration': 0,
                    'performance_metrics': {}
                }
            mode_analysis[mode]['activation_count'] += 1
        
        return mode_analysis
    
    def _analyze_task_performance(self, task_name, data):
        """Analyze individual task performance."""
        response_times = list(data['response_times'])
        turnaround_times = list(data['turnaround_times'])
        cpu_times = list(data['cpu_times'])
        jitter_values = list(data['jitter_values'])
        
        analysis = {
            'sample_count': len(response_times),
            'response_time_stats': self._calculate_stats(response_times),
            'turnaround_time_stats': self._calculate_stats(turnaround_times),
            'cpu_time_stats': self._calculate_stats(cpu_times),
            'jitter_stats': self._calculate_stats(jitter_values),
            'performance_grade': self._calculate_performance_grade(task_name, response_times, jitter_values)
        }
        
        return analysis
    
    def _calculate_stats(self, values):
        """Calculate statistical measures for a list of values."""
        if not values:
            return {'mean': 0, 'median': 0, 'std': 0, 'min': 0, 'max': 0, 'percentile_95': 0}
        
        np_values = np.array(values)
        return {
            'mean': float(np.mean(np_values)),
            'median': float(np.median(np_values)),
            'std': float(np.std(np_values)),
            'min': float(np.min(np_values)),
            'max': float(np.max(np_values)),
            'percentile_95': float(np.percentile(np_values, 95))
        }
    
    def _calculate_performance_grade(self, task_name, response_times, jitter_values):
        """Calculate performance grade (A-F) based on task requirements."""
        if not response_times or not jitter_values:
            return 'F'
        
        # Define performance thresholds based on task type
        thresholds = {
            'LED_CTRL': {'response_limit': 2000, 'jitter_limit': 100},      # 2ms response, 100us jitter
            'CPU_CALC': {'response_limit': 5000, 'jitter_limit': 500},      # 5ms response, 500us jitter
            'SENSOR_RD': {'response_limit': 3000, 'jitter_limit': 200}      # 3ms response, 200us jitter
        }
        
        if task_name not in thresholds:
            return 'C'
        
        limits = thresholds[task_name]
        avg_response = np.mean(response_times)
        avg_jitter = np.mean(jitter_values)
        
        response_score = min(limits['response_limit'] / max(avg_response, 1), 1.0)
        jitter_score = min(limits['jitter_limit'] / max(avg_jitter, 1), 1.0)
        
        overall_score = (response_score + jitter_score) / 2
        
        if overall_score >= 0.9:
            return 'A'
        elif overall_score >= 0.8:
            return 'B'
        elif overall_score >= 0.7:
            return 'C'
        elif overall_score >= 0.6:
            return 'D'
        else:
            return 'F'
    
    def _comparative_analysis(self):
        """Compare performance across different scheduling modes."""
        comparison = {}
        
        # Group data by scheduling mode
        mode_data = defaultdict(lambda: defaultdict(list))
        
        for timestamp, mode in self.mode_transitions:
            # Find metrics collected during this mode
            # (This is a simplified approach - in practice, you'd correlate timestamps)
            pass
        
        return comparison
    
    def _generate_recommendations(self):
        """Generate optimization recommendations based on analysis."""
        recommendations = []
        
        # Analyze task performance grades
        task_grades = {}
        for task_name, data in self.task_metrics.items():
            if len(data['response_times']) > 0:
                task_grades[task_name] = self._calculate_performance_grade(
                    task_name, list(data['response_times']), list(data['jitter_values'])
                )
        
        # Generate recommendations based on performance
        poor_performers = [task for task, grade in task_grades.items() if grade in ['D', 'F']]
        
        if poor_performers:
            recommendations.append({
                'type': 'performance_improvement',
                'priority': 'high',
                'description': f"Tasks with poor performance: {', '.join(poor_performers)}",
                'suggestion': 'Consider increasing task priorities or optimizing task implementations'
            })
        
        # Check for high jitter
        high_jitter_tasks = []
        for task_name, data in self.task_metrics.items():
            if len(data['jitter_values']) > 0:
                avg_jitter = np.mean(list(data['jitter_values']))
                if avg_jitter > 500:  # 500us threshold
                    high_jitter_tasks.append(task_name)
        
        if high_jitter_tasks:
            recommendations.append({
                'type': 'jitter_optimization',
                'priority': 'medium',
                'description': f"High jitter detected in: {', '.join(high_jitter_tasks)}",
                'suggestion': 'Consider using priority-based scheduling or reducing system load'
            })
        
        return recommendations
    
    def create_visualizations(self, output_dir='plots'):
        """Create comprehensive visualizations of the collected data."""
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        # Response time comparison
        plt.figure(figsize=(12, 8))
        for task_name, data in self.task_metrics.items():
            if len(data['response_times']) > 0:
                plt.plot(list(data['response_times']), label=f'{task_name} Response Time', alpha=0.7)
        
        plt.xlabel('Sample Number')
        plt.ylabel('Response Time (microseconds)')
        plt.title('Task Response Times Over Time')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig(f'{output_dir}/response_times.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # Jitter analysis
        plt.figure(figsize=(12, 6))
        jitter_data = []
        labels = []
        
        for task_name, data in self.task_metrics.items():
            if len(data['jitter_values']) > 0:
                jitter_data.append(list(data['jitter_values']))
                labels.append(task_name)
        
        if jitter_data:
            plt.boxplot(jitter_data, labels=labels)
            plt.ylabel('Jitter (microseconds)')
            plt.title('Task Jitter Distribution')
            plt.grid(True, alpha=0.3)
            plt.savefig(f'{output_dir}/jitter_distribution.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # CPU utilization
        plt.figure(figsize=(12, 6))
        for task_name, data in self.task_metrics.items():
            if len(data['cpu_times']) > 0:
                plt.plot(list(data['cpu_times']), label=f'{task_name} CPU Time', alpha=0.7)
        
        plt.xlabel('Sample Number')
        plt.ylabel('CPU Time (microseconds)')
        plt.title('Task CPU Utilization Over Time')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig(f'{output_dir}/cpu_utilization.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Visualizations saved to {output_dir}/")
    
    def export_data(self, filename='scheduling_data', format='csv'):
        """Export collected data to file."""
        if format == 'csv':
            with open(f'{filename}.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Timestamp', 'Task', 'Response_Time', 'Turnaround_Time', 'CPU_Time', 'Jitter'])
                
                for task_name, data in self.task_metrics.items():
                    for i in range(len(data['response_times'])):
                        writer.writerow([
                            data['timestamps'][i].isoformat(),
                            task_name,
                            data['response_times'][i],
                            data['turnaround_times'][i],
                            data['cpu_times'][i],
                            data['jitter_values'][i]
                        ])
        
        elif format == 'json':
            export_data = {}
            for task_name, data in self.task_metrics.items():
                export_data[task_name] = {
                    'timestamps': [ts.isoformat() for ts in data['timestamps']],
                    'response_times': list(data['response_times']),
                    'turnaround_times': list(data['turnaround_times']),
                    'cpu_times': list(data['cpu_times']),
                    'jitter_values': list(data['jitter_values'])
                }
            
            with open(f'{filename}.json', 'w') as jsonfile:
                json.dump(export_data, jsonfile, indent=2)
        
        print(f"Data exported to {filename}.{format}")

def main():
    """Main function for command-line usage."""
    parser = argparse.ArgumentParser(description='ESP32 FreeRTOS Scheduling Analysis Tool')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--duration', type=int, default=60, help='Collection duration in seconds (default: 60)')
    parser.add_argument('--output', default='analysis_results', help='Output filename prefix (default: analysis_results)')
    parser.add_argument('--visualize', action='store_true', help='Create visualizations')
    parser.add_argument('--export-csv', action='store_true', help='Export data to CSV')
    parser.add_argument('--export-json', action='store_true', help='Export data to JSON')
    
    args = parser.parse_args()
    
    # Initialize analyzer
    analyzer = SchedulingAnalyzer(port=args.port, baudrate=args.baudrate)
    
    try:
        # Start data collection
        analyzer.start_data_collection()
        
        print(f"Collecting data for {args.duration} seconds...")
        print("Press Ctrl+C to stop early")
        
        # Wait for specified duration
        time.sleep(args.duration)
        
    except KeyboardInterrupt:
        print("\\nStopping data collection...")
    
    finally:
        # Stop data collection
        analyzer.stop_data_collection()
        
        # Generate report
        print("Generating analysis report...")
        report = analyzer.generate_performance_report()
        
        # Save report
        with open(f'{args.output}_report.json', 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"Analysis report saved to {args.output}_report.json")
        
        # Create visualizations if requested
        if args.visualize:
            analyzer.create_visualizations()
        
        # Export data if requested
        if args.export_csv:
            analyzer.export_data(args.output, 'csv')
        
        if args.export_json:
            analyzer.export_data(args.output, 'json')
        
        # Print summary
        print("\\n=== Analysis Summary ===")
        print(f"Data collection period: {report['analysis_period']}")
        print(f"Tasks analyzed: {len(report['task_analysis'])}")
        print(f"Recommendations generated: {len(report['recommendations'])}")
        
        for task_name, analysis in report['task_analysis'].items():
            print(f"{task_name}: Grade {analysis['performance_grade']} ({analysis['sample_count']} samples)")

if __name__ == '__main__':
    main()
