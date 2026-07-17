#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "runtime_config.h"

#include <stdint.h>

/* BSP 只负责运行配置的磨损均衡持久化，不拥有配置数据模型。 */
uint8_t RuntimeConfig_Load(RuntimeConfig_t *config);
uint8_t RuntimeConfig_Save(const RuntimeConfig_t *config);

#endif
