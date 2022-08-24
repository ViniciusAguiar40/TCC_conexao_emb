#include "mqtt_mgr.h" // Self-include

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

/* Private definitions ********************************************************/
#define TAG "MQTT_MGR"

#define CONFIG_BROKER_URL       "mqtt://mqtt.tago.io"
#define CONFIG_BROKER_PORT      (1883)
#define CONFIG_BROKER_USERNAME  "Token"
#define CONFIG_BROKER_PASSWORD  "420eea05-fdab-4b21-8908-4afd0919ea9d"

#define MQTT_PUBLISH_QOS        (0)
#define MQTT_PUBLISH_RETAIN     (0)

#define PROJ_CFG_PUBLISH_TSK_NAME       "mqtt_publish"
#define PROJ_CFG_PUBLISH_TSK_SZ         (configMINIMAL_STACK_SIZE * 5)
#define PROJ_CFG_PUBLISH_TSK_PRIORITY   (tskIDLE_PRIORITY + 3)

#define PUBLISH_QUEUE_LENGTH    (16)
#define MAX_TOPIC_SIZE          (36)
#define MAX_DATA_SIZE           (75)

/* Private constants and types ************************************************/
typedef struct{
    char topic [MAX_TOPIC_SIZE];
    char data [MAX_DATA_SIZE];
    uint8_t data_size;
} data_publish_t;

/* Private functions **********************************************************/
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data);

static void log_error_if_nonzero(const char *message, int error_code);

static void publish_task(void *pv);

/* Private variables **********************************************************/
esp_mqtt_client_handle_t client_hdl;
QueueHandle_t publishQueue;
static TaskHandle_t publish_tsk_h = NULL;

/* Public functions implementation ********************************************/
esp_err_t mqttMgr_Init(void)
{
    esp_err_t ret;
    BaseType_t err;

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .port = CONFIG_BROKER_PORT,
        .username = CONFIG_BROKER_USERNAME,
        .password = CONFIG_BROKER_PASSWORD,
    };

    client_hdl = esp_mqtt_client_init(&mqtt_cfg);

    ret = esp_mqtt_client_register_event(client_hdl,
                                   ESP_EVENT_ANY_ID,
                                   mqtt_event_handler,
                                   NULL);

    log_error_if_nonzero("reported from client register event", ret);

    ret = esp_mqtt_client_start(client_hdl);

    log_error_if_nonzero("reported from mqtt client starting", ret);
    
    assert(ret == ESP_OK);
    /* Create queue for publishing */
    publishQueue = xQueueCreate(PUBLISH_QUEUE_LENGTH,
                                sizeof(data_publish_t));
    assert(publishQueue != NULL);

    /* Create task to handle with data publishing */
    if(publish_tsk_h == NULL)
    {
        xTaskCreate(publish_task,
                    PROJ_CFG_PUBLISH_TSK_NAME,
                    PROJ_CFG_PUBLISH_TSK_SZ,
                    NULL,
                    PROJ_CFG_PUBLISH_TSK_PRIORITY,
                    &publish_tsk_h);
        assert(err == pdPASS);
    }

    return ret;
}

/**
 * @brief Publishes data on MQTT
 * 
 * @param topic string of the topic
 * @param data data to be sent
 * @param data_size 
 * @return ESP_OK if works
 */
esp_err_t mqttMgr_Publish(uint8_t *topic, uint8_t *data, uint16_t data_size)
{
    int ret;
    ret = esp_mqtt_client_publish(client_hdl, 
                                (const char*)topic, 
                                (const char*)data, 
                                data_size, 
                                MQTT_PUBLISH_QOS, 
                                MQTT_PUBLISH_RETAIN);

    if(ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Data has been successfully published at the topic %s", topic);
        ESP_LOG_BUFFER_CHAR(TAG, data, data_size);
    }
    else
    {
        ESP_LOGE(TAG, "Data has not been published. Error code: %x",ret);
    }
    return ret;
}


/**
 * @brief Insert data on queue to be published
 * 
 * @param topic string of the topic
 * @param data data to be sent
 * @param data_size 
 * @return true if 
 * @return false 
 */
bool mqttMgr_InsertPubQueue(uint8_t *topic, uint8_t *data, uint16_t data_size)
{
    if(publishQueue == NULL)
    {
        ESP_LOGE(TAG, "Queue not initialized");
        return false; // queue not initialized
    }
    if(data_size > MAX_DATA_SIZE)
    {
        ESP_LOGE(TAG, "Too much data");
        return false;
    }
    if(strlen((char*)topic) > MAX_TOPIC_SIZE)
    {
        ESP_LOGE(TAG, "Topic size too large");
        return false;
    }
    data_publish_t newData;
    BaseType_t err;

    /* Copies the topic and data to insert on the queue */
    strcpy(newData.topic, (char*) topic);
    strcpy(newData.data, (char*) data);
    newData.data_size = data_size;

    /* Insert data on queue */
    err = xQueueSend(publishQueue, &newData, pdMS_TO_TICKS(50));

    return (err == pdPASS);
}

/**
 * @brief Task to handle with data publishing 
 * 
 * @param pv 
 */
static void publish_task(void *pv)
{
    data_publish_t publish_data;
    while(1)
    {
        if(xQueueReceive(publishQueue, &publish_data, portMAX_DELAY))
        {
            mqttMgr_Publish((uint8_t*) publish_data.topic, 
                            (uint8_t*) publish_data.data,
                            publish_data.data_size);
        }
    }
    vTaskDelete(NULL);
}

/* Private functions implementation *******************************************/
/**
 * @brief Used for debuging
 * 
 * @param message 
 * @param error_code 
 */
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/**
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
