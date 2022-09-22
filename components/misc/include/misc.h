/* MISCELLANOUS */

#include "esp_log.h"

#ifndef COMPONENTS_MISC_INCLUDE_MISC_H_
#define COMPONENTS_MISC_INCLUDE_MISC_H_

#define ENABLE_LOG  (1)

#if ENABLE_LOG
#define TCC_LOGI( TAG, ... )    ESP_LOGI(TAG, __VA_ARGS__ )
#else
#define TCC_LOGI( TAG, ... )    (void)(0)
#endif

#if ENABLE_LOG
#define TCC_LOGW( TAG, ... )    ESP_LOGW(TAG, __VA_ARGS__ )
#else
#define TCC_LOGW( TAG, ... )    (void)(0)
#endif

#if ENABLE_LOG
#define TCC_LOGE( TAG, ... )    ESP_LOGE(TAG, __VA_ARGS__ )
#else
#define TCC_LOGE( TAG, ... )    (void)(0)
#endif

#if ENABLE_LOG
#define TCC_LOGD( TAG, ... )    ESP_LOGD(TAG, __VA_ARGS__ )
#else
#define TCC_LOGD( TAG, ... )    (void)(0)
#endif

#if ENABLE_LOG
#define TCC_LOG_BUFFER_HEX( TAG, ... )    ESP_LOG_BUFFER_HEX(TAG, __VA_ARGS__ )
#else
#define TCC_LOG_BUFFER_HEX( TAG, ... )    (void)(0)
#endif


#endif  /* COMPONENTS_MISC_INCLUDE_MISC_H_ */