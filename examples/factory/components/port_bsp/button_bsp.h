#ifndef BUTTON_BSP_H
#define BUTTON_BSP_H

#include <freertos/FreeRTOS.h>

#ifdef __cplusplus
extern "C" {
#endif

extern EventGroupHandle_t BootButtonGroups;
extern EventGroupHandle_t GP18ButtonGroups;


#define set_bit_button(x) ((uint32_t)(0x01)<<(x))
#define get_bit_button(x,y) (((uint32_t)(x)>>(y)) & 0x01)
#define set_bit_all 0x00ffffff        


//set bit
#define set_bit_data(x,y) (x |= (0x01<<y))
#define clr_bit_data(x,y) (x &= ~(0x01<<y))
#define get_bit_data(x,y) ((x>>y) & 0x01)
#define rset_bit_data(x) ((uint32_t)0x01<<(x))

void Custom_ButtonInit(void);

#ifdef __cplusplus
}
#endif

#endif