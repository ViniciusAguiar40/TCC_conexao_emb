/* Includes *******************************************************************/
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "periph_spi.h"
#include "mqtt_mgr.h"
#include "wifi_mgr.h"

/* Private definitions ********************************************************/
#define TEST    (0)    // Set to 1 to test

void app_main(void)
{
    periphSpi_Init();
    periphSpi_StartReceiving();
    wifi_start();

    mqttMgr_Init();
    vTaskDelay(2000/portTICK_PERIOD_MS);

#if TEST
    uint8_t topic[] = "TCC_RobotBulls/potencia_esquerda";
    uint8_t pub[64];
    snprintf((char*)pub, 61, "[{\"variable\": \"potencia_esquerda\",\"value\": 2, \"unit\" : \"%%\"}]");
    char c;

    while(1)
    {
        c = fgetc(stdin);
        if(c > '0' && c<='9')
        {
            pub[43] = c;
            mqttMgr_Publish(topic, pub, 60);
            c='0';
        }

        vTaskDelay(10/portTICK_PERIOD_MS);
    }
#endif
}