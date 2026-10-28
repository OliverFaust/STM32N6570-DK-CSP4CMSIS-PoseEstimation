// Bridge between ST's application (Src/app.c) and the CSP4CMSIS process
// network (application.cpp): the stage functions of app.c that the processes
// call, and the network's entry points that app.c calls. display_info_t,
// the message from pp to dp, is ST's (Inc/app.h).
#ifndef APP_APPLICATION_H_
#define APP_APPLICATION_H_

#include <stdint.h>
// Outside the extern "C" block: in C++ it sets its own linkage.
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "app.h"
#include "app_cam.h"
#include "stm32n6xx_hal.h"

// Src/app.c
void NN_Hardware_Init(void);
uint32_t NN_Execute_Inference(void);
void PP_Hardware_Init(void);
void PP_Execute_Inference(display_info_t *info);
void LCD_Render_Frame(display_info_t *info);
float LCD_Get_Cpu_Load(void);

// application.cpp
void csp_app_main_init(void);
void csp_signal_isp_vsync(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_APPLICATION_H_ */
