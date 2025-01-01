/**
  ******************************************************************************
  * @file          : app_tof.h
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the STMicroelectronics.X-CUBE-TOF1.3.4.2 instances.
  ******************************************************************************
  *
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_TOF_H
#define __APP_TOF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "custom_ranging_sensor.h"

/* Exported defines ----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void MX_TOF_Init(void);
RANGING_SENSOR_Result_t MX_TOF_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TOF_H */