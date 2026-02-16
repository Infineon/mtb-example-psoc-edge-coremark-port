/*****************************************************************************
 * File Name        : cy_core_config.h
 *
 * Description      : This source file contains platform specific-definitions
 *
 * Related Document : See README.md
 *
 ******************************************************************************
 * (c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
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
