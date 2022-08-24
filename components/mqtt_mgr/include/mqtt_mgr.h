/* Public include *************************************************************/
#include "esp_err.h"
#include <stdbool.h>

/* Public functions ***********************************************************/
esp_err_t mqttMgr_Init(void);
esp_err_t mqttMgr_Publish(uint8_t *topic, uint8_t *data, uint16_t data_size);
bool mqttMgr_InsertPubQueue(uint8_t *topic, uint8_t *data, uint16_t data_size);