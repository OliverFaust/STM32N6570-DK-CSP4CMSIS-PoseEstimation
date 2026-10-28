 /**
 ******************************************************************************
 * @file    app.h
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

#ifndef APP_H
#define APP_H

#include "stai_network.h"
#include "app_config.h"
#include "app_postprocess.h"

#define NN_WIDTH STAI_NETWORK_IN_1_WIDTH
#define NN_HEIGHT STAI_NETWORK_IN_1_HEIGHT

/* Moved here from app.c: display_info_t is the message the CSP4CMSIS network passes from pp to dp */
#ifdef TRACKER_MODULE
typedef struct {
  int is_valid;
  float32_t x;
  float32_t y;
} trk_kp_t;

typedef struct {
  double cx;
  double cy;
  double w;
  double h;
  uint32_t id;
  trk_kp_t keyPoints[AI_POSE_PP_POSE_KEYPOINTS_NB];
} tbox_info;
#endif

typedef struct {
  int32_t nb_detect;
  mpe_pp_outBuffer_t detects[AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT];
  int tracking_enabled;
#ifdef TRACKER_MODULE
  int tboxes_valid_nb;
  tbox_info tboxes[AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT];
#endif
  uint32_t nn_period_ms;
  uint32_t inf_ms;
  uint32_t pp_ms;
  uint32_t disp_ms;
} display_info_t;

void app_run(void);

#endif