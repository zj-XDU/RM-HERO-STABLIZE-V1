#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);
#endif
