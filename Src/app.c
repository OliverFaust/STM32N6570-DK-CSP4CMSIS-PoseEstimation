 /**
 ******************************************************************************
 * @file    app.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "app.h"

#include <stdint.h>

#include "app_cam.h"
#include "app_config.h"
#include "app_postprocess.h"
#include "cmw_camera.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32_lcd.h"
#include "stm32_lcd_ex.h"
#include "stm32n6xx_hal.h"
#include "stm32n6570_discovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#ifdef TRACKER_MODULE
#include "pkf.h"
#include "tracker.h"
#endif
#include "stai.h"
#include "stai_network.h"
#include "network.h"
#include "utils.h"
#include "app_application.h"

#ifndef APP_VERSION_STRING
#define APP_VERSION_STRING "dev"
#endif

#define FREERTOS_PRIORITY(p) ((UBaseType_t)((int)tskIDLE_PRIORITY + configMAX_PRIORITIES / 2 + (p)))

#define CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

#define ALIGN_VALUE(_v_,_a_) (((_v_) + (_a_) - 1) & ~((_a_) - 1))

#define BUTTON_TOGGLE_TRACKING BUTTON_USER1

#define LCD_FG_WIDTH LCD_BG_WIDTH
#define LCD_FG_HEIGHT LCD_BG_HEIGHT

#define NUMBER_COLORS 10
#define BQUEUE_MAX_BUFFERS 3
#define CPU_LOAD_HISTORY_DEPTH 8

#define CIRCLE_RADIUS 5
/* Must be odd */
#define BINDING_WIDTH 3
#define COLOR_HEAD UTIL_LCD_COLOR_GREEN
#define COLOR_ARMS UTIL_LCD_COLOR_BLUE
#define COLOR_TRUNK UTIL_LCD_COLOR_MAGENTA
#define COLOR_LEGS UTIL_LCD_COLOR_ORANGE
#define COLOR_BOX UTIL_LCD_COLOR_RED

#define DISPLAY_BUFFER_NB (DISPLAY_DELAY + 2)

/* Align so we are sure nn_output_buffers[0] and nn_output_buffers[1] are aligned on 32 bytes */
#define NN_BUFFER_OUT_SIZE_ALIGN ALIGN_VALUE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES, 32)

#ifdef TRACKER_MODULE
typedef struct {
  trk_kp_t keyPoints[AI_POSE_PP_POSE_KEYPOINTS_NB];
  struct pkf_state pkf_states[AI_POSE_PP_POSE_KEYPOINTS_NB];
} trk_tbox_ctx_t;
#endif

typedef struct
{
  uint32_t X0;
  uint32_t Y0;
  uint32_t XSize;
  uint32_t YSize;
} Rectangle_TypeDef;

typedef struct {
  SemaphoreHandle_t free;
  StaticSemaphore_t free_buffer;
  SemaphoreHandle_t ready;
  StaticSemaphore_t ready_buffer;
  int buffer_nb;
  uint8_t *buffers[BQUEUE_MAX_BUFFERS];
  int free_idx;
  int ready_idx;
} bqueue_t;

typedef struct {
  uint64_t current_total;
  uint64_t current_thread_total;
  uint64_t prev_total;
  uint64_t prev_thread_total;
  struct {
    uint64_t total;
    uint64_t thread;
    uint32_t tick;
  } history[CPU_LOAD_HISTORY_DEPTH];
} cpuload_info_t;

/* Globals */
DECLARE_CLASSES_TABLE;
/* Lcd Background area */
static Rectangle_TypeDef lcd_bg_area = {
  .X0 = (LCD_DEFAULT_WIDTH - LCD_BG_WIDTH) / 2,
  .Y0 = (LCD_DEFAULT_HEIGHT - LCD_BG_HEIGHT) / 2,
  .XSize = LCD_BG_WIDTH,
  .YSize = LCD_BG_HEIGHT,
};
/* Lcd Foreground area */
static Rectangle_TypeDef lcd_fg_area = {
  .X0 = (LCD_DEFAULT_WIDTH - LCD_FG_WIDTH) / 2,
  .Y0 = (LCD_DEFAULT_HEIGHT - LCD_FG_HEIGHT) / 2,
  .XSize = LCD_FG_WIDTH,
  .YSize = LCD_FG_HEIGHT,
};
static const uint32_t colors[NUMBER_COLORS] = {
    UTIL_LCD_COLOR_GREEN,
    UTIL_LCD_COLOR_RED,
    UTIL_LCD_COLOR_CYAN,
    UTIL_LCD_COLOR_MAGENTA,
    UTIL_LCD_COLOR_YELLOW,
    UTIL_LCD_COLOR_GRAY,
    UTIL_LCD_COLOR_BLACK,
    UTIL_LCD_COLOR_BROWN,
    UTIL_LCD_COLOR_BLUE,
    UTIL_LCD_COLOR_ORANGE
};

static const int bindings[][3] = {
  {15, 13, COLOR_LEGS},
  {13, 11, COLOR_LEGS},
  {16, 14, COLOR_LEGS},
  {14, 12, COLOR_LEGS},
  {11, 12, COLOR_TRUNK},
  { 5, 11, COLOR_TRUNK},
  { 6, 12, COLOR_TRUNK},
  { 5,  6, COLOR_ARMS},
  { 5,  7, COLOR_ARMS},
  { 6,  8, COLOR_ARMS},
  { 7,  9, COLOR_ARMS},
  { 8, 10, COLOR_ARMS},
  { 1,  2, COLOR_HEAD},
  { 0,  1, COLOR_HEAD},
  { 0,  2, COLOR_HEAD},
  { 1,  3, COLOR_HEAD},
  { 2,  4, COLOR_HEAD},
  { 3,  5, COLOR_HEAD},
  { 4,  6, COLOR_HEAD},
};

static const int kp_color[17] = {
COLOR_HEAD,
COLOR_HEAD,
COLOR_HEAD,
COLOR_HEAD,
COLOR_HEAD,
COLOR_ARMS,
COLOR_ARMS,
COLOR_ARMS,
COLOR_ARMS,
COLOR_ARMS,
COLOR_ARMS,
COLOR_TRUNK,
COLOR_TRUNK,
COLOR_LEGS,
COLOR_LEGS,
COLOR_LEGS,
COLOR_LEGS,
};

/* Lcd Background Buffer */
static uint8_t lcd_bg_buffer[DISPLAY_BUFFER_NB][LCD_BG_WIDTH * LCD_BG_HEIGHT * 2] ALIGN_32 IN_PSRAM;
static int lcd_bg_buffer_disp_idx = 1;
static int lcd_bg_buffer_capt_idx = 0;
/* Lcd Foreground Buffer */
static uint8_t lcd_fg_buffer[2][LCD_FG_WIDTH * LCD_FG_HEIGHT* 2] ALIGN_32 IN_PSRAM;
static int lcd_fg_buffer_rd_idx;
static cpuload_info_t cpu_load;

/* model */
static uint8_t network_ctx[STAI_NETWORK_CONTEXT_SIZE] ALIGN_32;
 /* nn input buffers */
static uint8_t nn_input_buffers[3][NN_WIDTH * NN_HEIGHT * NN_BPP] ALIGN_32 IN_PSRAM;
static bqueue_t nn_input_queue;
 /* nn output buffers */
static uint8_t nn_output_buffers[2][NN_BUFFER_OUT_SIZE_ALIGN] ALIGN_32;
static bqueue_t nn_output_queue;

/* tracking state */
#ifdef TRACKER_MODULE
static struct pkf_context pkf_context;
static trk_tbox_ctx_t tboxes_ctx[2 * AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT];
static trk_tbox_t tboxes[2 * AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT];
static trk_dbox_t dboxes[AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT];
static trk_ctx_t trk_ctx;
#endif

static int is_cache_enable()
{
#if defined(USE_DCACHE)
  return 1;
#else
  return 0;
#endif
}

static void cpuload_init(cpuload_info_t *cpu_load)
{
  memset(cpu_load, 0, sizeof(cpuload_info_t));
}

static void cpuload_update(cpuload_info_t *cpu_load)
{
  int i;

  cpu_load->history[1] = cpu_load->history[0];
  cpu_load->history[0].total = portGET_RUN_TIME_COUNTER_VALUE();
  cpu_load->history[0].thread = cpu_load->history[0].total - ulTaskGetIdleRunTimeCounter();
  cpu_load->history[0].tick = HAL_GetTick();

  if (cpu_load->history[1].tick - cpu_load->history[2].tick < 1000)
    return ;

  for (i = 0; i < CPU_LOAD_HISTORY_DEPTH - 2; i++)
    cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i] = cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i - 1];
}

static void cpuload_get_info(cpuload_info_t *cpu_load, float *cpu_load_last, float *cpu_load_last_second,
                             float *cpu_load_last_five_seconds)
{
  if (cpu_load_last)
    *cpu_load_last = 100.0 * (cpu_load->history[0].thread - cpu_load->history[1].thread) /
                     (cpu_load->history[0].total - cpu_load->history[1].total);
  if (cpu_load_last_second)
    *cpu_load_last_second = 100.0 * (cpu_load->history[2].thread - cpu_load->history[3].thread) /
                     (cpu_load->history[2].total - cpu_load->history[3].total);
  if (cpu_load_last_five_seconds)
    *cpu_load_last_five_seconds = 100.0 * (cpu_load->history[2].thread - cpu_load->history[7].thread) /
                     (cpu_load->history[2].total - cpu_load->history[7].total);
}

static int bqueue_init(bqueue_t *bq, int buffer_nb, uint8_t **buffers)
{
  int i;

  if (buffer_nb > BQUEUE_MAX_BUFFERS)
    return -1;

  bq->free = xSemaphoreCreateCountingStatic(buffer_nb, buffer_nb, &bq->free_buffer);
  if (!bq->free)
    goto free_sem_error;
  bq->ready = xSemaphoreCreateCountingStatic(buffer_nb, 0, &bq->ready_buffer);
  if (!bq->ready)
    goto ready_sem_error;

  bq->buffer_nb = buffer_nb;
  for (i = 0; i < buffer_nb; i++) {
    assert(buffers[i]);
    bq->buffers[i] = buffers[i];
  }
  bq->free_idx = 0;
  bq->ready_idx = 0;

  return 0;

ready_sem_error:
  vSemaphoreDelete(bq->free);
free_sem_error:
  return -1;
}

static uint8_t *bqueue_get_free(bqueue_t *bq, int is_blocking)
{
  uint8_t *res;
  int ret;

  ret = xSemaphoreTake(bq->free, is_blocking ? portMAX_DELAY : 0);
  if (ret == pdFALSE)
    return NULL;

  res = bq->buffers[bq->free_idx];
  bq->free_idx = (bq->free_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_free(bqueue_t *bq)
{
  int ret;

  ret = xSemaphoreGive(bq->free);
  assert(ret == pdTRUE);
}

static uint8_t *bqueue_get_ready(bqueue_t *bq)
{
  uint8_t *res;
  int ret;

  ret = xSemaphoreTake(bq->ready, portMAX_DELAY);
  assert(ret == pdTRUE);

  res = bq->buffers[bq->ready_idx];
  bq->ready_idx = (bq->ready_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_ready(bqueue_t *bq)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int ret;

  if (xPortIsInsideInterrupt()) {
    ret = xSemaphoreGiveFromISR(bq->ready, &xHigherPriorityTaskWoken);
    assert(ret == pdTRUE);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  } else {
    ret = xSemaphoreGive(bq->ready);
    assert(ret == pdTRUE);
  }
}

static void app_main_pipe_frame_event()
{
  int next_disp_idx = (lcd_bg_buffer_disp_idx + 1) % DISPLAY_BUFFER_NB;
  int next_capt_idx = (lcd_bg_buffer_capt_idx + 1) % DISPLAY_BUFFER_NB;
  int ret;

  ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1,
                                         DCMIPP_MEMORY_ADDRESS_0, (uint32_t) lcd_bg_buffer[next_capt_idx]);
  assert(ret == HAL_OK);

  ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t) lcd_bg_buffer[next_disp_idx], LTDC_LAYER_1);
  assert(ret == HAL_OK);
  ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_1);
  assert(ret == HAL_OK);
  lcd_bg_buffer_disp_idx = next_disp_idx;
  lcd_bg_buffer_capt_idx = next_capt_idx;
}

static void app_ancillary_pipe_frame_event()
{
  uint8_t *next_buffer;
  int ret;

  next_buffer = bqueue_get_free(&nn_input_queue, 0);
  if (next_buffer) {
    ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE2,
                                           DCMIPP_MEMORY_ADDRESS_0, (uint32_t) next_buffer);
    assert(ret == HAL_OK);
    bqueue_put_ready(&nn_input_queue);
  }
}

static void app_main_pipe_vsync_event()
{
  csp_signal_isp_vsync();
}

static void LCD_init()
{
  BSP_LCD_LayerConfig_t LayerConfig = {0};

  BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);

  /* Preview layer Init */
  LayerConfig.X0          = lcd_bg_area.X0;
  LayerConfig.Y0          = lcd_bg_area.Y0;
  LayerConfig.X1          = lcd_bg_area.X0 + lcd_bg_area.XSize;
  LayerConfig.Y1          = lcd_bg_area.Y0 + lcd_bg_area.YSize;
  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_RGB565;
  LayerConfig.Address     = (uint32_t) lcd_bg_buffer[lcd_bg_buffer_disp_idx];

  BSP_LCD_ConfigLayer(0, LTDC_LAYER_1, &LayerConfig);

  LayerConfig.X0 = lcd_fg_area.X0;
  LayerConfig.Y0 = lcd_fg_area.Y0;
  LayerConfig.X1 = lcd_fg_area.X0 + lcd_fg_area.XSize;
  LayerConfig.Y1 = lcd_fg_area.Y0 + lcd_fg_area.YSize;
  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_ARGB4444;
  LayerConfig.Address = (uint32_t) lcd_fg_buffer[1]; /* External XSPI1 PSRAM */

  BSP_LCD_ConfigLayer(0, LTDC_LAYER_2, &LayerConfig);
  UTIL_LCD_SetFuncDriver(&LCD_Driver);
  UTIL_LCD_SetLayer(LTDC_LAYER_2);
  UTIL_LCD_Clear(0x00000000);
  UTIL_LCD_SetFont(&Font20);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
}

static int clamp_point(int *x, int *y)
{
  int xi = *x;
  int yi = *y;

  if (*x < 0)
    *x = 0;
  if (*y < 0)
    *y = 0;
  if (*x >= lcd_bg_area.XSize)
    *x = lcd_bg_area.XSize - 1;
  if (*y >= lcd_bg_area.YSize)
    *y = lcd_bg_area.YSize - 1;

  return (xi != *x) || (yi != *y);
}

static void convert_length(float32_t wi, float32_t hi, int *wo, int *ho)
{
  *wo = (int) (lcd_bg_area.XSize * wi);
  *ho = (int) (lcd_bg_area.YSize * hi);
}

static void convert_point(float32_t xi, float32_t yi, int *xo, int *yo)
{
  *xo = (int) (lcd_bg_area.XSize * xi);
  *yo = (int) (lcd_bg_area.YSize * yi);
}

static void Display_Circle(double xi, double yi, uint32_t color)
{
  int is_clamp;
  int xc, yc;
  int x, y;

  convert_point(xi, yi, &x, &y);
  xc = x - CIRCLE_RADIUS / 2;
  yc = y - CIRCLE_RADIUS / 2;
  is_clamp = clamp_point(&xc, &yc);
  xc = x + CIRCLE_RADIUS / 2;
  yc = y + CIRCLE_RADIUS / 2;
  is_clamp |= clamp_point(&xc, &yc);

  if (is_clamp)
    return ;

  UTIL_LCD_FillCircle(x, y, CIRCLE_RADIUS, color);
}

static void Display_keypoint(mpe_pp_keyPoints_t *key, uint32_t color)
{
  if (key->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD)
    return ;

  Display_Circle(key->x, key->y, color);
}

static void Display_binding_line(int x0, int y0, int x1, int y1, uint32_t color)
{
  clamp_point(&x0, &y0);
  clamp_point(&x1, &y1);

  UTIL_LCD_DrawLine(x0, y0, x1, y1, color);
}

static void Display_binding(mpe_pp_keyPoints_t *from, mpe_pp_keyPoints_t *to, uint32_t color)
{
  int is_clamp;
  int x0, y0;
  int x1, y1;
  int i;

  assert(BINDING_WIDTH % 2 == 1);

  if (from->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD)
    return ;
  if (to->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD)
    return ;

  convert_point(from->x, from->y, &x0, &y0);
  is_clamp = clamp_point(&x0, &y0);
  if (is_clamp)
    return ;

  convert_point(to->x, to->y, &x1, &y1);
  is_clamp = clamp_point(&x1, &y1);
  if (is_clamp)
    return ;

  UTIL_LCD_DrawLine(x0, y0, x1, y1, color);
  for (i = 1; i <= (BINDING_WIDTH - 1) / 2; i++) {
    if (abs(y1 - y0) > abs(x1 - x0)) {
      Display_binding_line(x0 + i, y0, x1 + i , y1, color);
      Display_binding_line(x0 - i, y0, x1 - i , y1, color);
    } else {
      Display_binding_line(x0, y0 + i, x1 , y1 + i, color);
      Display_binding_line(x0, y0 - i, x1 , y1 - i, color);
    }
  }
}

static void Display_Detection(mpe_pp_outBuffer_t *detect)
{
  int xc, yc;
  int x0, y0;
  int x1, y1;
  int w, h;
  int i;

  convert_point(detect->x_center, detect->y_center, &xc, &yc);
  convert_length(detect->width, detect->height, &w, &h);
  x0 = xc - (w + 1) / 2;
  y0 = yc - (h + 1) / 2;
  x1 = xc + (w + 1) / 2;
  y1 = yc + (h + 1) / 2;
  clamp_point(&x0, &y0);
  clamp_point(&x1, &y1);

  UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, colors[detect->class_index % NUMBER_COLORS]);
  UTIL_LCDEx_PrintfAt(x0 + 1, y0 + 1, LEFT_MODE, classes_table[detect->class_index]);

  for (i = 0; i < ARRAY_NB(bindings); i++)
    Display_binding(&detect->pKeyPoints[bindings[i][0]], &detect->pKeyPoints[bindings[i][1]], bindings[i][2]);
  for (i = 0; i < AI_POSE_PP_POSE_KEYPOINTS_NB; i++)
    Display_keypoint(&detect->pKeyPoints[i], kp_color[i]);
}

static void Display_NetworkOutput_NoTracking(display_info_t *info)
{
  mpe_pp_outBuffer_t *rois = info->detects;
  uint32_t nb_rois = info->nb_detect;
  float cpu_load_one_second;
  int line_nb = 0;
  float nn_fps;
  int i;

  /* clear previous ui */
  UTIL_LCD_FillRect(lcd_fg_area.X0, lcd_fg_area.Y0, lcd_fg_area.XSize, lcd_fg_area.YSize, 0x00000000); /* Clear previous boxes */

  /* cpu load */
  cpuload_update(&cpu_load);
  cpuload_get_info(&cpu_load, NULL, &cpu_load_one_second, NULL);

  /* draw metrics */
  nn_fps = 1000.0 / info->nn_period_ms;
#if 1
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "Cpu load");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "   %.1f%%", cpu_load_one_second);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inference");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->inf_ms);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   FPS");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "  %.2f", nn_fps);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " Objects %u", nb_rois);
  line_nb += 1;
#else
  (void) nn_fps;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "Cpu load");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "   %.1f%%", cpu_load_one_second);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "nn period");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->nn_period_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inference");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->inf_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Post process");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->pp_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Display");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->disp_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " Objects %u", nb_rois);
  line_nb += 1;
#endif

  /* Draw bounding boxes */
  for (i = 0; i < nb_rois; i++)
    Display_Detection(&rois[i]);
}

#ifdef TRACKER_MODULE
static void Display_trackingKeypoint(trk_kp_t *key, uint32_t color)
{
  if (!key->is_valid)
    return ;

  Display_Circle(key->x, key->y, color);
}

static void Display_trackingBinding(trk_kp_t *from, trk_kp_t *to, uint32_t color)
{
  int is_clamp;
  int x0, y0;
  int x1, y1;
  int i;

  assert(BINDING_WIDTH % 2 == 1);

  if (!from->is_valid)
    return ;
  if (!to->is_valid)
    return ;

  convert_point(from->x, from->y, &x0, &y0);
  is_clamp = clamp_point(&x0, &y0);
  if (is_clamp)
    return ;

  convert_point(to->x, to->y, &x1, &y1);
  is_clamp = clamp_point(&x1, &y1);
  if (is_clamp)
    return ;

  UTIL_LCD_DrawLine(x0, y0, x1, y1, color);
  for (i = 1; i <= (BINDING_WIDTH - 1) / 2; i++) {
    if (abs(y1 - y0) > abs(x1 - x0)) {
      Display_binding_line(x0 + i, y0, x1 + i , y1, color);
      Display_binding_line(x0 - i, y0, x1 - i , y1, color);
    } else {
      Display_binding_line(x0, y0 + i, x1 , y1 + i, color);
      Display_binding_line(x0, y0 - i, x1 , y1 - i, color);
    }
  }
}

static void Display_TrackingBox(tbox_info *tbox)
{
  int xc, yc;
  int x0, y0;
  int x1, y1;
  int w, h;
  int i;

  convert_point(tbox->cx, tbox->cy, &xc, &yc);
  convert_length(tbox->w, tbox->h, &w, &h);
  x0 = xc - (w + 1) / 2;
  y0 = yc - (h + 1) / 2;
  x1 = xc + (w + 1) / 2;
  y1 = yc + (h + 1) / 2;
  clamp_point(&x0, &y0);
  clamp_point(&x1, &y1);

  UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, colors[tbox->id % NUMBER_COLORS]);
  UTIL_LCDEx_PrintfAt(x0 + 1, y0 + 1, LEFT_MODE, "%3d", tbox->id);

  for (i = 0; i < ARRAY_NB(bindings); i++)
    Display_trackingBinding(&tbox->keyPoints[bindings[i][0]], &tbox->keyPoints[bindings[i][1]], bindings[i][2]);
  for (i = 0; i < AI_POSE_PP_POSE_KEYPOINTS_NB; i++)
    Display_trackingKeypoint(&tbox->keyPoints[i], kp_color[i]);
}

static void Display_NetworkOutput_Tracking(display_info_t *info)
{
  uint32_t nb_rois = info->nb_detect;
  float cpu_load_one_second;
  int line_nb = 0;
  float nn_fps;
  int i;

  /* clear previous ui */
  UTIL_LCD_FillRect(lcd_fg_area.X0, lcd_fg_area.Y0, lcd_fg_area.XSize, lcd_fg_area.YSize, 0x00000000); /* Clear previous boxes */

  /* cpu load */
  cpuload_update(&cpu_load);
  cpuload_get_info(&cpu_load, NULL, &cpu_load_one_second, NULL);

  /* draw metrics */
  nn_fps = 1000.0 / info->nn_period_ms;
#if 1
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "Cpu load");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "   %.1f%%", cpu_load_one_second);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inference");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->inf_ms);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   FPS");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "  %.2f", nn_fps);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " Objects %u", nb_rois);
  line_nb += 1;
#else
  (void) nn_fps;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "Cpu load");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "   %.1f%%", cpu_load_one_second);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "nn period");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->nn_period_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inference");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->inf_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Post process");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->pp_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Display");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->disp_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " Objects %u", nb_rois);
  line_nb += 1;
#endif

  /* Draw bounding boxes */
  for (i = 0; i < info->tboxes_valid_nb; i++)
    Display_TrackingBox(&info->tboxes[i]);
}
#else
static void Display_NetworkOutput_Tracking(display_info_t *info)
{
  /* You should not be here */
  assert(0);
}
#endif

static void Display_NetworkOutput(display_info_t *info)
{
  if (info->tracking_enabled)
    Display_NetworkOutput_Tracking(info);
  else
    Display_NetworkOutput_NoTracking(info);
}

static void inference_run(stai_network *network_instance)
{
  stai_return_code ret;

  do {
    ret = stai_network_run(network_instance, STAI_MODE_ASYNC);
    if (ret == STAI_RUNNING_WFE)
      LL_ATON_OSAL_WFE();
  } while (ret == STAI_RUNNING_WFE || ret == STAI_RUNNING_NO_WFE);
  ret = stai_ext_network_new_inference(network_instance);
  assert(ret == STAI_SUCCESS);
}

/* The nn stage, called by the CSP4CMSIS network (CSP/application.cpp):
   NN_Hardware_Init() once, then NN_Execute_Inference() for each frame. */
void NN_Hardware_Init(void)
{
  stai_network_info info;
  uint8_t *nn_pipe_dst;
  int ret;

  /* initialize runtime */
  ret = stai_runtime_init();
  assert(ret == STAI_SUCCESS);
  /* init model instance */
  ret = stai_network_init(network_ctx);
  assert(ret == STAI_SUCCESS);

  /* setup buffers size */
  ret = stai_network_get_info(network_ctx, &info);
  assert(ret == STAI_SUCCESS);
  assert(info.n_inputs == 1);
  assert(info.n_outputs == 1);
  assert(info.inputs[0].size_bytes == LL_ATON_NETWORK_IN_1_SIZE_BYTES);
  assert(info.outputs[0].size_bytes == LL_ATON_NETWORK_OUT_1_SIZE_BYTES);

  nn_pipe_dst = bqueue_get_free(&nn_input_queue, 0);
  assert(nn_pipe_dst);
  CAM_NNPipe_Start(nn_pipe_dst, CMW_MODE_CONTINUOUS);
}

/* Returns the inference time in ms (the network measures the period). */
uint32_t NN_Execute_Inference(void)
{
  uint8_t *capture_buffer;
  uint8_t *output_buffer;
  stai_ptr inputs[1];
  stai_ptr outputs[1];
  uint32_t inf_ms;
  uint32_t ts;
  int ret;

  capture_buffer = bqueue_get_ready(&nn_input_queue);
  assert(capture_buffer);
  output_buffer = bqueue_get_free(&nn_output_queue, 1);
  assert(output_buffer);

  /* run ATON inference */
  ts = HAL_GetTick();
  /* Note that we don't need to clean/invalidate those input buffers since they are only access in hardware */
  inputs[0] = capture_buffer;
  ret = stai_network_set_inputs(network_ctx, inputs, ARRAY_NB(inputs));
  assert(ret == STAI_SUCCESS);
  /* Invalidate output buffer before Hw access it */
  outputs[0] = output_buffer;
  CACHE_OP(SCB_InvalidateDCache_by_Addr(output_buffer, LL_ATON_NETWORK_OUT_1_SIZE_BYTES));
  ret = stai_network_set_outputs(network_ctx, outputs, ARRAY_NB(outputs));
  assert(ret == STAI_SUCCESS);
  inference_run(network_ctx);
  inf_ms = HAL_GetTick() - ts;

  /* release buffers */
  bqueue_put_free(&nn_input_queue);
  bqueue_put_ready(&nn_output_queue);

  return inf_ms;
}

#ifdef TRACKER_MODULE
static int TRK_Init()
{
  const double ratio = CAMERA_FPS > 25 ? 2.0 : 1.0;
  const trk_conf_t cfg = {
    .track_thresh = 0.25,
    .det_thresh = 0.8,
    .sim1_thresh = 0.8,
    .sim2_thresh = 0.5,
    .tlost_cnt = 30,
  };
  int i;

  pkf_init_context(&pkf_context, ratio / CAMERA_FPS, 1, 0.025);

  memset(tboxes_ctx, 0, sizeof(tboxes_ctx));
  for (i = 0; i < ARRAY_NB(tboxes); i++)
    tboxes[i].userdata = &tboxes_ctx[i];

  return trk_init(&trk_ctx, (trk_conf_t *) &cfg, ARRAY_NB(tboxes), tboxes);
}

static int update_and_capture_tracking_enabled()
{
  static int prev_button_state = GPIO_PIN_RESET;
  static int tracking_enabled = 1;
  int cur_button_state;
  int ret;

  cur_button_state = BSP_PB_GetState(BUTTON_TOGGLE_TRACKING);
  if (cur_button_state == GPIO_PIN_SET && prev_button_state == GPIO_PIN_RESET) {
    tracking_enabled = !tracking_enabled;
    if (tracking_enabled) {
      printf("Enable tracking\n");
      ret = TRK_Init();
      assert(ret == 0);
    } else
      printf("Disable tracking\n");
  }
  prev_button_state = cur_button_state;

  return tracking_enabled;
}

static void roi_to_dbox(mpe_pp_outBuffer_t *roi, trk_dbox_t *dbox)
{
  dbox->conf = roi->conf;
  dbox->cx = roi->x_center;
  dbox->cy = roi->y_center;
  dbox->w = roi->width;
  dbox->h = roi->height;
  dbox->userdata = roi;
}

static void app_trk_update_tbox_context(trk_tbox_t *tbox)
{
  trk_tbox_ctx_t *tbox_ctx = (trk_tbox_ctx_t *)tbox->userdata;
  mpe_pp_outBuffer_t *d = tbox->dbox_userdata;

  struct pkf_point predicted;
  struct pkf_point updated;
  int i;

  /* if not tracking or lost tracking then clear key points */
  if (!tbox->dbox_userdata) {
    memset(tbox_ctx, 0, sizeof(*tbox_ctx));
    return ;
  }

  /* update keypoints */
  for (i = 0; i < AI_POSE_PP_POSE_KEYPOINTS_NB; i++) {
    mpe_pp_keyPoints_t *kp = &d->pKeyPoints[i];
    trk_kp_t *kp_ctx = &tbox_ctx->keyPoints[i];
    struct pkf_state *kf = &tbox_ctx->pkf_states[i];
    struct pkf_point pt;

    if (kp->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD) {
      kp_ctx->is_valid = 0;
      continue;
    }

    pt.x = kp->x;
    pt.y = kp->y;
    if (!kp_ctx->is_valid) {
      kp_ctx->is_valid = 1;
      pkf_init(kf, &pkf_context, &pt);
      kp_ctx->x = kp->x;
      kp_ctx->y = kp->y;
      continue;
    }

    pkf_predict(kf, &predicted);
    pkf_update(kf, &pt, &updated);
    kp_ctx->x = updated.x;
    kp_ctx->y = updated.y;
  }
}

static int app_tracking(mpe_pp_out_t *pp)
{
  int tracking_enabled = update_and_capture_tracking_enabled();
  int ret;
  int i;

  if (!tracking_enabled)
    return 0;

  for (i = 0; i < pp->nb_detect; i++)
    roi_to_dbox(&pp->pOutBuff[i], &dboxes[i]);

  ret = trk_update(&trk_ctx, pp->nb_detect, dboxes);
  assert(ret == 0);

  for (i = 0; i < ARRAY_NB(tboxes); i++)
    app_trk_update_tbox_context(&tboxes[i]);

  return 1;
}

static void tbox_to_tbox_info(trk_tbox_t *tbox, tbox_info *tinfo)
{
  trk_tbox_ctx_t *tbox_ctx = tbox->userdata;
  int i;

  tinfo->cx = tbox->cx;
  tinfo->cy = tbox->cy;
  tinfo->w = tbox->w;
  tinfo->h = tbox->h;
  tinfo->id = tbox->id;
  for (i = 0; i < AI_POSE_PP_POSE_KEYPOINTS_NB; i++)
    tinfo->keyPoints[i] = tbox_ctx->keyPoints[i];
}
#else
static int app_tracking(mpe_pp_out_t *pp)
{
  return 0;
}
#endif

/* The pp stage, called by the CSP4CMSIS network: PP_Hardware_Init() once,
   then PP_Execute_Inference() for each frame, which fills the detection info
   of *info (the network measures pp_ms and adds the nn stats). */
#if POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V2_UF
static od_yolov2_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V5_UU
static od_yolov5_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UF || POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UI
static od_yolov8_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_POSE_YOLO_V8_UF
static yolov8_pose_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_MPE_YOLO_V8_UF
static mpe_yolov8_pp_static_param_t pp_params;
#else
    #error "PostProcessing type not supported"
#endif
#if POSTPROCESS_TYPE == POSTPROCESS_MPE_YOLO_V8_UF
static mpe_pp_out_t pp_output;
#else
static postprocess_out_t pp_output;
#endif

void PP_Hardware_Init(void)
{
  stai_network_info info;
  int ret;

  /* setup post process */
  ret = stai_network_get_info(network_ctx, &info);
  assert(ret == STAI_SUCCESS);
  app_postprocess_init(&pp_params, &info);
}

void PP_Execute_Inference(display_info_t *info)
{
  uint8_t *output_buffer;
  int tracking_enabled;
  void *pp_input;
  int ret;
  int i;

  output_buffer = bqueue_get_ready(&nn_output_queue);
  assert(output_buffer);
  pp_input = (void *) output_buffer;
  pp_output.pOutBuff = NULL;

  ret = app_postprocess_run((void * []){pp_input}, 1, &pp_output, &pp_params);
  assert(ret == 0);
  tracking_enabled = app_tracking(&pp_output);

  /* detection info */
  info->nb_detect = pp_output.nb_detect;
  for (i = 0; i < pp_output.nb_detect; i++)
    info->detects[i] = pp_output.pOutBuff[i];
  info->tracking_enabled = tracking_enabled;
#ifdef TRACKER_MODULE
  info->tboxes_valid_nb = 0;
  for (i = 0; i < ARRAY_NB(tboxes); i++) {
    if (!tboxes[i].is_tracking || tboxes[i].tlost_cnt)
      continue;
    tbox_to_tbox_info(&tboxes[i], &info->tboxes[info->tboxes_valid_nb]);
    info->tboxes_valid_nb++;
  }
#endif

  bqueue_put_free(&nn_output_queue);
}

static void dp_update_drawing_area()
{
  int ret;

  __disable_irq();
  ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t) lcd_fg_buffer[lcd_fg_buffer_rd_idx], LTDC_LAYER_2);
  assert(ret == HAL_OK);
  __enable_irq();
}

static void dp_commit_drawing_area()
{
  int ret;

  __disable_irq();
  ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_2);
  assert(ret == HAL_OK);
  __enable_irq();
  lcd_fg_buffer_rd_idx = 1 - lcd_fg_buffer_rd_idx;
}

/* The dp stage, called by the CSP4CMSIS network for each frame. */
void LCD_Render_Frame(display_info_t *info)
{
  static uint32_t disp_ms = 0;
  uint32_t ts;

  info->disp_ms = disp_ms;

  ts = HAL_GetTick();
  dp_update_drawing_area();
  Display_NetworkOutput(info);
  SCB_CleanDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_WIDTH * LCD_FG_HEIGHT* 2);
  dp_commit_drawing_area();
  disp_ms = HAL_GetTick() - ts;
}

/* The one-second CPU load last computed by the display; called from the dp stage. */
float LCD_Get_Cpu_Load(void)
{
  float cpu_load_one_second;

  cpuload_get_info(&cpu_load, NULL, &cpu_load_one_second, NULL);
  return cpu_load_one_second;
}

void app_run()
{
  int ret;

  printf("========================================\n");
  printf("x-cube-n6-ai-multi-pose-estimation v2.2.0 (%s)\n", APP_VERSION_STRING);
  printf("Build date & time: %s %s\n", __DATE__, __TIME__);
  #if defined(__GNUC__)
  printf("Compiler: GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#elif defined(__ICCARM__)
  printf("Compiler: IAR EWARM %d.%d.%d\n", __VER__ / 1000000, (__VER__ / 1000) % 1000 ,__VER__ % 1000);
#else
  printf("Compiler: Unknown\n");
#endif
  printf("HAL: %lu.%lu.%lu\n", __STM32N6xx_HAL_VERSION_MAIN, __STM32N6xx_HAL_VERSION_SUB1, __STM32N6xx_HAL_VERSION_SUB2);
  printf("STEdgeAI Tools: %d.%d.%d\n", STAI_TOOLS_VERSION_MAJOR, STAI_TOOLS_VERSION_MINOR, STAI_TOOLS_VERSION_MICRO);
  printf("NN model: %s\n", LL_ATON_NETWORK_ORIGIN_MODEL_NAME);
  printf("========================================\n");

  /* Enable DWT so DWT_CYCCNT works when debugger not attached */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* screen init */
  memset(lcd_bg_buffer, 0, sizeof(lcd_bg_buffer));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_bg_buffer, sizeof(lcd_bg_buffer)));
  memset(lcd_fg_buffer, 0, sizeof(lcd_fg_buffer));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_fg_buffer, sizeof(lcd_fg_buffer)));
  LCD_init();

  /* create buffer queues */
  ret = bqueue_init(&nn_input_queue, 3, (uint8_t *[3]){nn_input_buffers[0], nn_input_buffers[1], nn_input_buffers[2]});
  assert(ret == 0);
  ret = bqueue_init(&nn_output_queue, 2, (uint8_t *[2]){nn_output_buffers[0], nn_output_buffers[1]});
  assert(ret == 0);

#ifdef TRACKER_MODULE
  ret = TRK_Init();
  assert(ret == 0);
  ret = BSP_PB_Init(BUTTON_TOGGLE_TRACKING, BUTTON_MODE_GPIO);
  assert(ret == BSP_ERROR_NONE);
#endif

  cpuload_init(&cpu_load);

  /*** Camera Init ************************************************************/  
  CAM_Init();

  /* Start LCD Display camera pipe stream */
  CAM_DisplayPipe_Start(lcd_bg_buffer[0], CMW_MODE_CONTINUOUS);

  /* The CSP4CMSIS process network (CSP/application.cpp) replaces the isp, nn, pp and dp threads */
  csp_app_main_init();
}

int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_frame_event();
  else if (pipe == DCMIPP_PIPE2)
    app_ancillary_pipe_frame_event();

  return HAL_OK;
}

int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_vsync_event();

  return HAL_OK;
}
