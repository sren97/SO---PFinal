/*
 * SPDX-FileCopyrightText: 2018-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESP_EVENT_INTERNAL_H_
#define ESP_EVENT_INTERNAL_H_

#include "sys/queue.h"
#include <stdbool.h>
#include "esp_event.h"
#include "stdatomic.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef SLIST_HEAD(base_nodes, base_node) base_nodes_t;

typedef struct esp_event_handler_context {
    esp_event_handler_t handler;                                    /**< event handler function*/
    void* arg;
} esp_event_handler_instance_context_t;                             /**< event handler argument */

/// Event handler
typedef struct esp_event_handler_node {
    esp_event_handler_instance_context_t* handler_ctx;              /**< event handler context*/
#ifdef CONFIG_ESP_EVENT_LOOP_PROFILING
    uint32_t invoked;                                               /**< number of times this handler has been invoked */
    int64_t time;                                                   /**< total runtime of this handler across all calls */
#endif
    SLIST_ENTRY(esp_event_handler_node) next;                   /**< next event handler in the list */
    bool unregistered;
} esp_event_handler_node_t;

typedef SLIST_HEAD(esp_event_handler_instances, esp_event_handler_node) esp_event_handler_nodes_t;

/// Event
typedef struct esp_event_id_node {
    int32_t id;                                                     /**< id number of the event */
    esp_event_handler_nodes_t handlers;                             /**< list of handlers to be executed when
                                                                            this event is raised */
    SLIST_ENTRY(esp_event_id_node) next;                            /**< pointer to the next event node on the linked list */
} esp_event_id_node_t;

typedef SLIST_HEAD(esp_event_id_nodes, esp_event_id_node) esp_event_id_nodes_t;

typedef struct esp_event_base_node {
    esp_event_base_t base;                                          /**< base identifier of the event */
    esp_event_handler_nodes_t handlers;                             /**< event base level handlers, handlers for
                                                                            all events with this base */
    esp_event_id_nodes_t id_nodes;                                  /**< list of event ids with this base */
    SLIST_ENTRY(esp_event_base_node) next;                          /**< pointer to the next base node on the linked list */
} esp_event_base_node_t;

typedef SLIST_HEAD(esp_event_base_nodes, esp_event_base_node) esp_event_base_nodes_t;

typedef struct esp_event_loop_node {
    esp_event_handler_nodes_t handlers;                             /** event loop level handlers */
    esp_event_base_nodes_t base_nodes;                              /** list of event bases registered to the loop */
    SLIST_ENTRY(esp_event_loop_node) next;                          /** pointer to the next loop node containing
                                                                            event loop level handlers and the rest of
                                                                            event bases registered to the loop */
} esp_event_loop_node_t;

typedef SLIST_HEAD(esp_event_loop_nodes, esp_event_loop_node) esp_event_loop_nodes_t;

/// Event loop
typedef struct esp_event_loop_instance {
    const char* name;                                               /**< name of this event loop */
    QueueHandle_t queue;                                            /**< event queue */
    TaskHandle_t task;                                              /**< task that consumes the event queue */
    TaskHandle_t running_task;                                      /**< for loops with no dedicated task, the
                                                                            task that consumes the queue */
    SemaphoreHandle_t mutex;                                        /**< mutex for updating the events linked list */
    esp_event_loop_nodes_t loop_nodes;                              /**< set of linked lists containing the
                                                                            registered handlers for the loop */
#ifdef CONFIG_ESP_EVENT_LOOP_PROFILING
    atomic_uint_least32_t events_received;                          /**< number of events successfully posted to the loop */
    atomic_uint_least32_t events_dropped;                           /**< number of events dropped due to queue being full */
    SLIST_ENTRY(esp_event_loop_instance) next;                      /**< next event loop in the list */
#endif
} esp_event_loop_instance_t;

typedef struct esp_event_remove_handler_context_t {
    esp_event_loop_instance_t* loop;                                /**< Instance of the event loop from which the handler has to be removed */
    esp_event_base_t event_base;                                    /**< The event base identification of the handler that has to be removed */
    int32_t event_id;                                               /**< The event identification value of the handler that has to be removed */
    esp_event_handler_instance_context_t* handler_ctx;              /**< The handler context of the handler that has to be removed */
    bool legacy;                                                    /**< Set to true when the handler unregistration request was made from legacy code */
} esp_event_remove_handler_context_t;

typedef union esp_event_post_data {
    uint32_t val;
    void *ptr;
} esp_event_post_data_t;

/// Event posted to the event queue
typedef struct esp_event_post_instance {
#if CONFIG_ESP_EVENT_POST_FROM_ISR
    bool data_allocated;                                             /**< indicates whether data is allocated from heap */
    bool data_set;                                                   /**< indicates if data is null */
#endif
    esp_event_base_t base;                                           /**< the event base */
    int32_t id;                                                      /**< the event id */
    esp_event_post_data_t data;                                      /**< data associated with the event */
} esp_event_post_instance_t;

#ifdef __cplusplus
} // extern "C"
#endif

#endif // #ifndef ESP_EVENT_INTERNAL_H_
