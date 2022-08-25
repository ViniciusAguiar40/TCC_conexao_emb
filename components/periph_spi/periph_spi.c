#include <stdio.h>
#include "periph_spi.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "mqtt_mgr.h"

/* Private definitions ********************************************************/
// SPI
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 14
#define GPIO_CS 15

#define RCV_HOST HSPI_HOST

// TASKS
#define PROJ_CFG_PERIPH_SPI_TSK_NAME        "periph spi"
#define PROJ_CFG_PERIPH_SPI_TSK_STK_SZ      (configMINIMAL_STACK_SIZE * 5)
#define PROJ_CFG_PERIPH_SPI_TSK_PRIORITY    (tskIDLE_PRIORITY + 3)

#define PROJ_CFG_SPI_PARSER_TSK_NAME        "spi parser"
#define PROJ_CFG_SPI_PARSER_TSK_STK_SZ      (configMINIMAL_STACK_SIZE * 5)
#define PROJ_CFG_SPI_PARSER_TSK_PRIORITY    (tskIDLE_PRIORITY + 3)

// HEADER TYPES
#define BATTERY_VOLTAGE_HEADER      (0X01)
#define MOTOR_CURRENT_HEADER        (0X02)
#define LEFT_MOTOR_POWER_HEADER     (0X03)
#define RIGHT_MOTOR_POWER_HEADER    (0X04)
#define MOTOR_SPEED_HEADER          (0X05)
#define SELECTED_MOVE_HEADER        (0X06)

// DATA LENGTH
#define BATTERY_VOLTAGE_DATALEN     (2)
#define MOTOR_CURRENT_DATALEN       (4)
#define LEFT_MOTOR_POWER_DATALEN    (1)
#define RIGHT_MOTOR_POWER_DATALEN   (1)
#define MOTOR_SPEED_DATALEN         (8)
#define SELECTED_MOVE_DATALEN       (1)

#define PARSER_QUEUE_LENGTH         (32)

// TAG
#define TAG "PERIPH_SPI"

/* Private variables and types ************************************************/
typedef struct{
    uint8_t raw_data [129]; 
} raw_spi_data_t;

static TaskHandle_t periph_spi_tsk_h = NULL;
static TaskHandle_t spi_parser_tsk_h = NULL;

QueueHandle_t parserQueue;

spi_slave_transaction_t t;
WORD_ALIGNED_ATTR uint8_t recvbuf[129] = "";

/* Private function ***********************************************************/
static void periph_spi_task(void *pv);
static void spi_parser_task(void *pv);
static bool verify_crc(uint8_t *packet, uint8_t nBytes);
static uint16_t crc16(uint8_t *packet, uint8_t nBytes);

static void batteryVoltage_parser(uint8_t *packet, uint8_t nBytes);
static void motorCurrent_parser(uint8_t *packet, uint8_t nBytes);
static void leftMotorPower_parser(uint8_t *packet, uint8_t nBytes);
static void rightMotorPower_parser(uint8_t *packet, uint8_t nBytes);
static void motorSpeed_parser(uint8_t *packet, uint8_t nBytes);
static void selectedMove_parser(uint8_t *packet, uint8_t nBytes);

/* Public functions implementation ********************************************/
/**
 * @brief Initilize SPI peripheral
 *
 * @return ESP_OK if works
 */
esp_err_t periphSpi_Init(void)
{
    esp_err_t ret;

    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL};

    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    // Initialize SPI slave interface
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    BaseType_t err;

    if (ret == ESP_OK && periph_spi_tsk_h == NULL)
    {
        err = xTaskCreate(periph_spi_task,
                          PROJ_CFG_PERIPH_SPI_TSK_NAME,
                          PROJ_CFG_PERIPH_SPI_TSK_STK_SZ,
                          NULL,
                          PROJ_CFG_PERIPH_SPI_TSK_PRIORITY,
                          &periph_spi_tsk_h);

        assert(err == pdPASS);
    }

    /* Create queue for publishing */
    parserQueue = xQueueCreate(PARSER_QUEUE_LENGTH,
                                sizeof(raw_spi_data_t));
    assert(parserQueue != NULL);

    if(spi_parser_tsk_h == NULL)
    {
        err = xTaskCreate(spi_parser_task,
                          PROJ_CFG_SPI_PARSER_TSK_NAME,
                          PROJ_CFG_SPI_PARSER_TSK_STK_SZ,
                          NULL,
                          PROJ_CFG_SPI_PARSER_TSK_PRIORITY,
                          &spi_parser_tsk_h);

        assert(err == pdPASS);
    }


    return ret;
}

/**
 * @brief Configure buffer for data receiving and create spi peripheral task
 *
 * @param recv_buff
 * @return esp_err_t
 */
esp_err_t periphSpi_StartReceiving(void)
{
    memset(recvbuf, 0, sizeof(recvbuf));
    memset(&t, 0, sizeof(t));
    t.length = 128 * 8;
    t.tx_buffer = NULL;
    t.rx_buffer = recvbuf;
    return 0;
}

/* Private functions implementation *******************************************/
static void periph_spi_task(void *pv)
{
    esp_err_t ret;
    raw_spi_data_t newData;
    while(1)
    {
        // Clear SPI receiver buffer
        memset(recvbuf, 0, sizeof(recvbuf));
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
        assert (ret == ESP_OK);
        if(parserQueue == NULL)
        {
            ESP_LOGE(TAG, "Parser queue not initialized!");
            continue;
        }
        memcpy(newData.raw_data, recvbuf, sizeof(newData.raw_data));
        xQueueSend(parserQueue,
                   &newData,
                   pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}
static void spi_parser_task(void *pv)
{
    raw_spi_data_t data;
    uint8_t header;
    uint8_t *rawData;
    while(1)
    {
        if(xQueueReceive(parserQueue, &data, portMAX_DELAY))
        {
            // ESP_LOGI(TAG, "NEW MESSAGE!");

            rawData = data.raw_data;
            header = rawData[0];
            switch (header)
            {
                case BATTERY_VOLTAGE_HEADER:
                    // ESP_LOG_BUFFER_HEX(TAG, rawData, BATTERY_VOLTAGE_DATALEN + 3);
                    if (verify_crc(rawData, BATTERY_VOLTAGE_DATALEN + 3))
                    {
                        batteryVoltage_parser(rawData, BATTERY_VOLTAGE_DATALEN);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "CRC error");
                    }

                    break;

                case MOTOR_CURRENT_HEADER:
                    // ESP_LOG_BUFFER_HEX(TAG, rawData, MOTOR_CURRENT_DATALEN + 3);
                    if (verify_crc(rawData, MOTOR_CURRENT_DATALEN + 3))
                    {
                        motorCurrent_parser(rawData, MOTOR_CURRENT_DATALEN);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "CRC error");
                    }
                    break;

                case LEFT_MOTOR_POWER_HEADER:
                    // ESP_LOG_BUFFER_HEX(TAG, rawData, LEFT_MOTOR_POWER_DATALEN + 3);
                    if (verify_crc(rawData, LEFT_MOTOR_POWER_DATALEN + 3))
                    {
                        leftMotorPower_parser(rawData, LEFT_MOTOR_POWER_DATALEN);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "CRC error");
                    }
                    break;

                case RIGHT_MOTOR_POWER_HEADER:
                    // ESP_LOG_BUFFER_HEX(TAG, rawData, RIGHT_MOTOR_POWER_DATALEN + 3);
                    if (verify_crc(rawData, RIGHT_MOTOR_POWER_DATALEN + 3))
                    {
                        rightMotorPower_parser(rawData, RIGHT_MOTOR_POWER_DATALEN);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "CRC error");
                    }
                    break;

                case MOTOR_SPEED_HEADER:
                    // ESP_LOG_BUFFER_HEX(TAG, rawData, MOTOR_SPEED_DATALEN + 3);
                    if (verify_crc(rawData, MOTOR_SPEED_DATALEN + 3))
                    {
                        motorSpeed_parser(rawData, MOTOR_SPEED_DATALEN);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "CRC error");
                    }
                    break;

                case SELECTED_MOVE_HEADER:
                    // ESP_LOG_BUFFER_HEX(TAG, rawData, SELECTED_MOVE_DATALEN + 3);
                    if (verify_crc(rawData, SELECTED_MOVE_DATALEN + 3))
                    {
                        selectedMove_parser(rawData, SELECTED_MOVE_DATALEN);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "CRC error");
                    }
                    break;
                default:
                    ESP_LOGE(TAG, "Invalid header");
            }
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief Calculates CRC16
 *
 * @param packet Received packet
 * @param nBytes Data size
 * @return CRC value
 */
static uint16_t crc16(uint8_t *packet, uint8_t nBytes)
{
    uint16_t crc = 0;
    for (int byte = 0; byte < nBytes; byte++)
    {
        crc ^= ((uint16_t)packet[byte] << 8);
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

/**
 * @brief Compares received CRC and calculated CRC
 *
 * @param packet Received packet
 * @param nBytes Data size
 * @return true if both values are the same
 */
static bool verify_crc(uint8_t *packet, uint8_t nBytes)
{
    uint16_t crc;      // Received CRC
    uint16_t calc_crc; // Calculated CRC

    calc_crc = crc16(packet, nBytes - 2);

    crc = ((uint16_t) packet[nBytes - 2] << 8) | (uint16_t) packet[nBytes - 1];

    return (calc_crc == crc);
}

/**
 * @brief Parses battery voltage level
 *
 * @param packet
 * @param nBytes
 */
static void batteryVoltage_parser(uint8_t *packet, uint8_t nBytes)
{
    uint16_t batteryLevel;
    uint8_t pub[100];
    uint8_t topic[] = {"TCC_RobotBulls/bateria"};

    batteryLevel = ((uint16_t)packet[1] << 8) | (uint16_t)packet[2];
    // ESP_LOGI(TAG,"Battery level: %5u",batteryLevel);

    memset(pub, (0), sizeof(pub));
    snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"bateria\",\"value\": %2u.%01u, \"unit\" : \"V\"}]", batteryLevel/10, batteryLevel%10);
    // ESP_LOGI(TAG,"%s",pub);
    mqttMgr_InsertPubQueue(topic, pub, 53);
}   

/**
 * @brief Parses motors current
 *
 * @param packet
 * @param nBytes
 */
static void motorCurrent_parser(uint8_t *packet, uint8_t nBytes)
{
    uint16_t rightMotorCurrent, leftMotorCurrent;
    uint8_t pub[100];
    uint8_t topic_1[] = {"TCC_RobotBulls/corrente_esquerda"};
    uint8_t topic_2[] = {"TCC_RobotBulls/corrente_direita"};

    leftMotorCurrent = ((uint16_t)packet[1]) | (uint16_t)packet[2];
    rightMotorCurrent = ((uint16_t)packet[3]) | (uint16_t)packet[4];
    // ESP_LOGI(TAG, "Left motor current: %5u",leftMotorCurrent);
    // ESP_LOGI(TAG, "Right motor current: %5u",rightMotorCurrent);

    memset(pub,(0),sizeof(pub));
    snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"corrente_esquerda\",\"value\": %3u.%02u, \"unit\" : \"A\"}]", leftMotorCurrent/100, leftMotorCurrent%100);
    // ESP_LOGI(TAG,"%s",pub);
    mqttMgr_InsertPubQueue(topic_1, pub, 65);

    memset(pub,(0),sizeof(pub));
    snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"corrente_direita\",\"value\": %3u.%02u, \"unit\" : \"A\"}]", rightMotorCurrent/100, rightMotorCurrent%100);
    // ESP_LOGI(TAG,"%s",pub);
    mqttMgr_InsertPubQueue(topic_2, pub, 64);
}

/**
 * @brief Parses left motor power
 *
 * @param packet
 * @param nBytes
 */
static void leftMotorPower_parser(uint8_t *packet, uint8_t nBytes)
{
    uint8_t leftMotorPower;
    uint8_t pub[100];
    uint8_t topic[] = {"TCC_RobotBulls/potencia_esquerda"};

    leftMotorPower = packet[1];
    // ESP_LOGI(TAG,"Left motor power: %3u",leftMotorPower);
    
    memset(pub, (0), sizeof(pub));
    if(leftMotorPower > 63) // POSITIVE
    {
        // ((leftMotorPower-64)*10000) / 63 = abcd = ab.cd%
        snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"potencia_esquerda\",\"value\": %3u.%02u, \"unit\" : \"%%\"}]",
         (((leftMotorPower-64)*10000) / 63)/100, (((leftMotorPower-64)*10000) / 63)%100);
        // ESP_LOGI(TAG,"%s",pub);
        mqttMgr_InsertPubQueue(topic, pub, 65);
    }
    else    // NEGATIVE
    {
        // (abs(64-leftMotorPower)*10000) / 63 = abcd = -ab.cd%
        snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"potencia_esquerda\",\"value\": -%3u.%02u, \"unit\" : \"%%\"}]",
         ((abs(64-leftMotorPower)*10000) / 63)/100, ((abs(64-leftMotorPower)*10000) / 63)%100);
        // ESP_LOGI(TAG,"%s",pub);
        mqttMgr_InsertPubQueue(topic, pub, 66);
    }
}

/**
 * @brief Parses right motor power
 *
 * @param packet
 * @param nBytes
 */
static void rightMotorPower_parser(uint8_t *packet, uint8_t nBytes)
{
    uint8_t rightMotorPower;
    uint8_t pub[100];
    uint8_t topic[] = {"TCC_RobotBulls/potencia_direita"};

    rightMotorPower = packet[1];
    // ESP_LOGI(TAG,"Right motor power: %3u",rightMotorPower);
    
    memset(pub, (0), sizeof(pub));
    if(rightMotorPower > 63) // POSITIVE
    {
        // ((rightMotorPower-64)*10000) / 63 = abcd = ab.cd%
        snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"potencia_direita\",\"value\": %3u.%02u, \"unit\" : \"%%\"}]",
         (((rightMotorPower-64)*10000) / 63)/100, (((rightMotorPower-64)*10000) / 63)%100);
        // ESP_LOGI(TAG,"%s",pub);
        mqttMgr_InsertPubQueue(topic, pub, 64);
    }
    else    // NEGATIVE
    {
        // (abs(64-rightMotorPower)*10000) / 63 = abcd = -ab.cd%
        snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"potencia_direita\",\"value\": -%3u.%02u, \"unit\" : \"%%\"}]",
         ((abs(64-rightMotorPower)*10000) / 63)/100, ((abs(64-rightMotorPower)*10000) / 63)%100);
        // ESP_LOGI(TAG,"%s",pub);
        mqttMgr_InsertPubQueue(topic, pub, 65);
    }
}

/**
 * @brief Parses motors speed
 *
 * @param packet
 * @param nBytes
 */
static void motorSpeed_parser(uint8_t *packet, uint8_t nBytes)
{
    uint32_t leftMotorSpeed, rightMotorSpeed;
    uint8_t pub[100];
    uint8_t topic_1[] = {"TCC_RobotBulls/velocidade_esquerda"};
    uint8_t topic_2[] = {"TCC_RobotBulls/velocidade_direita"};

    leftMotorSpeed = ((uint32_t)(packet[1]<<24) | (uint32_t)(packet[2]<<16)
                    |(uint32_t)(packet[3]<<8)|(uint32_t)(packet[4]));

    
    rightMotorSpeed = ((uint32_t)(packet[5]<<24) | (uint32_t)(packet[6]<<16)
                    |(uint32_t)(packet[7]<<8)|(uint32_t)(packet[8]));
    
    // ESP_LOGI(TAG,"Left motor speed: %10u",leftMotorSpeed);
    // ESP_LOGI(TAG,"Right motor speed: %10u",rightMotorSpeed);

    memset(pub,(0),sizeof(pub));
    snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"velocidade_esquerda\",\"value\": %1u.%06u, \"unit\" : \"m/s\"}]", leftMotorSpeed/1000000, leftMotorSpeed%1000000);
    // ESP_LOGI(TAG,"%s",pub);
    mqttMgr_InsertPubQueue(topic_1, pub, 71);

    memset(pub,(0),sizeof(pub));
    snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"velocidade_direita\",\"value\": %1u.%06u, \"unit\" : \"m/s\"}]", rightMotorSpeed/1000000, rightMotorSpeed%1000000);
    // ESP_LOGI(TAG,"%s",pub);
    mqttMgr_InsertPubQueue(topic_2, pub, 70);
}

/**
 * @brief Parses selected move
 *
 * @param packet
 * @param nBytes
 */
static void selectedMove_parser(uint8_t *packet, uint8_t nBytes)
{
    uint8_t selectedMove;
    uint8_t pub[100];
    uint8_t topic[] = {"TCC_RobotBulls/jogada"};

    selectedMove = packet[1];
    // ESP_LOGI(TAG,"Jogada: %3u", selectedMove);
    
    memset(pub, (0), sizeof(pub));
    snprintf((char*)pub, sizeof(pub), "[{\"variable\": \"jogada\",\"value\": %3u}]", selectedMove);
    // ESP_LOGI(TAG, "%s", pub);
    mqttMgr_InsertPubQueue(topic, pub, 37);
}
