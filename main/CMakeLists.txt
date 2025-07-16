idf_component_register(SRCS "real_time_stats_example_main.c" "sensor.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer)
