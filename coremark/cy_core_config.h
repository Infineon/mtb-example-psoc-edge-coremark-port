/*****************************************************************************
 * File Name        : cy_core_config.h
 *
 * Description      : This source file contains platform specific-definitions
 *
 * Related Document : See README.md
 *
 ******************************************************************************
 * Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *****************************************************************************/
#ifndef CY_CORE_CONFIG_H
#define CY_CORE_CONFIG_H

/* Platform includes */
#include "stddef.h"

/* Number of Iterations */
#ifdef COREMARK_CM33
    #define ITERATIONS          8500
#else
    #define ITERATIONS          20000
#endif

/* Core Mark Configuration */
#define HAS_FLOAT           1
#define HAS_TIME_H          0
#define USE_CLOCK           0
#define HAS_STDIO           1
#define HAS_PRINTF          1
#define MEM_LOCATION        "STACK"
#define MAIN_HAS_NOARGC     1
#define MAIN_HAS_NORETURN   0

/* Device Specific */
#define DEV_CLOCKS_PER_SEC  1000000

#endif
