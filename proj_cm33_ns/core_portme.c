/*
Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Original Author: Shay Gal-on


Modifications by Infineon Technologies:

This version of the code has been modified to run on the PSOC Edge platform.
The modifications include initialization routines, timer configuration,
and platform-specific definitions.

*/
#include "coremark.h"
#include "core_portme.h"

#include "cybsp.h"
#include "cy_retarget_io.h"

#define CORE_CPU "CM33"

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

uint32_t systick_ov = 0;

/* For the RetargetIO (Debug UART) usage */
cy_stc_scb_uart_context_t  CYBSP_DEBUG_UART_context;  /** UART context */
mtb_hal_uart_t             CYBSP_DEBUG_UART_hal_obj;  /** UART HAL object */
void app_retarget_io_init(void);

void systick_callback(void)
{
    systick_ov++;
}

/* Porting : Timing functions
        How to capture time and convert to seconds must be ported to whatever is
   supported by the platform. e.g. Read value from on board RTC, read value from
   cpu clock cycles performance counter etc. Sample implementation for standard
   time.h and windows.h definitions included.
*/
CORETIMETYPE
barebones_clock()
{
    return ((0xFFFFFF - Cy_SysTick_GetValue()) + (systick_ov * 0x1000000));
}
/* Define : TIMER_RES_DIVIDER
        Divider to trade off timer resolution and total time that can be
   measured.

        Use lower values to increase resolution, but make sure that overflow
   does not occur. If there are issues with the return value overflowing,
   increase this value.
        */
#define GETMYTIME(_t)              (*_t = barebones_clock())
#define MYTIMEDIFF(fin, ini)       ((fin) - (ini))
#define TIMER_RES_DIVIDER          1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC           (DEV_CLOCKS_PER_SEC / TIMER_RES_DIVIDER)

/** Define Host specific (POSIX), or target specific global time variables. */
static CORETIMETYPE start_time_val, stop_time_val;

/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_US            (10U)

/* The wait time for the CM55 to run after enabling it */
#define CM55_RUN_WAIT_TIME_MS             (10U)  

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR                (CYMEM_CM33_0_m55_nvm_C_START + \
                                           CYBSP_MCUBOOT_HEADER_SIZE)

/* Function : start_time
        This function will be called right before starting the timed portion of
   the benchmark.

        Implementation may be capturing a system timer (as implemented in the
   example code) or zeroing some system parameters - e.g. setting the cpu clocks
   cycles to 0.
*/
void
start_time(void)
{   
    /* Set the UART pins to High-Z */
    Cy_GPIO_SetDrivemode(CYBSP_DEBUG_UART_RX_PORT, CYBSP_DEBUG_UART_RX_PIN, CY_GPIO_DM_ANALOG);
    Cy_GPIO_SetDrivemode(CYBSP_DEBUG_UART_TX_PORT, CYBSP_DEBUG_UART_TX_PIN, CY_GPIO_DM_ANALOG);

    /* Disable PERI Clock */
    Cy_SysClk_ClkHfDisable(10);

    /* Clear to avoid overflow at 24-bit */
    GETMYTIME(&start_time_val);
}
/* Function : stop_time
        This function will be called right after ending the timed portion of the
   benchmark.

        Implementation may be capturing a system timer (as implemented in the
   example code) or other system parameters - e.g. reading the current value of
   cpu cycles counter.
*/
void
stop_time(void)
{
    uint32_t intr;
    intr = Cy_SysLib_EnterCriticalSection();
    GETMYTIME(&stop_time_val);
    Cy_SysLib_ExitCriticalSection(intr);

    Cy_SysClk_ClkHfEnable(10);

    /* Set the UART pins to the orignal configuration */
    Cy_GPIO_Pin_Init(CYBSP_DEBUG_UART_RX_PORT, CYBSP_DEBUG_UART_RX_PIN, &CYBSP_DEBUG_UART_RX_config);
    Cy_GPIO_Pin_Init(CYBSP_DEBUG_UART_TX_PORT, CYBSP_DEBUG_UART_TX_PIN, &CYBSP_DEBUG_UART_TX_config);
}
/* Function : get_time
        Return an abstract "ticks" number that signifies time on the system.

        Actual value returned may be cpu cycles, milliseconds or any other
   value, as long as it can be converted to seconds by <time_in_secs>. This
   methodology is taken to accomodate any hardware or simulated platform. The
   sample implementation returns millisecs by default, and the resolution is
   controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS
get_time(void)
{
    CORE_TICKS elapsed
        = (CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
    return elapsed;
}
/* Function : time_in_secs
        Convert the value returned by get_time to seconds.

        The <secs_ret> type is used to accomodate systems with no support for
   floating point. Default implementation implemented by the EE_TICKS_PER_SEC
   macro above.
*/
secs_ret
time_in_secs(CORE_TICKS ticks)
{
    secs_ret retval = ((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
    return retval;
}

ee_u32 default_num_contexts = 1;

/* Function : portable_init
        Target specific initialization code
        Test for some common mistakes.
*/
void
portable_init(core_portable *p, int *argc, char *argv[])
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        /* Disable all interrupts. */
        __disable_irq();

        CY_ASSERT(0);

        /* Infinite loop */
        while(true);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Enable CM55. CY_CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed. */
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_US);

    /* Disable parts of SRAM0 (Except MACRO_0 and MACRO_5)*/
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM0_MEMORY, CY_SYSPM_SRAM0_MACRO_1, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM0_MEMORY, CY_SYSPM_SRAM0_MACRO_2, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM0_MEMORY, CY_SYSPM_SRAM0_MACRO_3, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM0_MEMORY, CY_SYSPM_SRAM0_MACRO_4, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM0_MEMORY, CY_SYSPM_SRAM0_MACRO_6, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM0_MEMORY, CY_SYSPM_SRAM0_MACRO_7, CY_SYSPM_SRAM_PWR_MODE_OFF);
    
    /* Disable parts of SRAM1 (Except MACRO_0, MACRO_3 and MACRO_7)*/
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM1_MEMORY, CY_SYSPM_SRAM0_MACRO_1, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM1_MEMORY, CY_SYSPM_SRAM0_MACRO_2, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM1_MEMORY, CY_SYSPM_SRAM0_MACRO_4, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM1_MEMORY, CY_SYSPM_SRAM0_MACRO_5, CY_SYSPM_SRAM_PWR_MODE_OFF);
    Cy_SysPm_SetSRAMMacroPwrMode(CY_SYSPM_SRAM1_MEMORY, CY_SYSPM_SRAM0_MACRO_6, CY_SYSPM_SRAM_PWR_MODE_OFF);

#ifdef BENCHMARK_CM33

    /* Wait for the CM55 to run. This is needed to properly shutdown PD1 */
    Cy_SysLib_Delay(CM55_RUN_WAIT_TIME_MS);

    /* Disable SOCMEM */
    Cy_SysEnableSOCMEM(false);

    /* Shutdown the PD1 */
    Cy_System_DisablePD1(); 
    
    static cy_stc_dpll_lp_config_t dpll_200MHz_config =
    {
        .feedbackDiv = 28,
        .referenceDiv = 7,
        .outputDiv = 1,
        .pllDcoMode = false,
        .outputMode = CY_SYSCLK_FLLPLL_OUTPUT_AUTO,
        .fracDiv = 0,
        .fracDitherEn = false,
        .fracEn = true,
        .dcoCode = 0xFU,
        .kiInt = 0xAU,
        .kiFrac = 0xBU,
        .kiSscg = 0x7U,
        .kpInt = 0x8U,
        .kpFrac = 0x9U,
        .kpSscg = 0x7U,
    };
    static cy_stc_pll_manual_config_t pll_200MHz_config =
    {
        .lpPllCfg = &dpll_200MHz_config,
    };

    /* Reduce the PLL clock to 200 MHz */
    Cy_SysClk_DpllLpDisable(0);   
    Cy_SysClk_DpllLpManualConfigure(0, &pll_200MHz_config);
    Cy_SysClk_DpllLpEnable(0, 10000u);
    Cy_SysClk_ClkHfSetDivider(0,CY_SYSCLK_CLKHF_NO_DIVIDE);

#else

    /* Go to deep-sleep if not running the core benchmarking */
    for (;;)
    {
        Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }

#endif /* BENCHMARK_CM33 */

    /* Initialize retarget-io to use the debug UART port */
    app_retarget_io_init();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("********** "
           "PSoC Edge MCU - (%s): Core Benchmark "
           "********** \r\n\n", CORE_CPU);

    /* Wait some time to print out the welcome message */
    Cy_SysLib_Delay(WAIT_PRINTF_TO_COMPLETE);

    /* Initialize the System Tick */
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_TIMER, 0xFFFFFFUL);
    Cy_SysTick_SetCallback(0, systick_callback);
    

    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
    {
        ee_printf(
            "ERROR! Please define ee_ptr_int to a type that holds a "
            "pointer!\n");
    }
    if (sizeof(ee_u32) != 4)
    {
        ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
    }
    p->portable_id = 1;
}
/* Function : portable_fini
        Target specific final code
*/
void
portable_fini(core_portable *p)
{
    p->portable_id = 0;

    /* Wait till the report is printed */
    Cy_SysLib_Delay(WAIT_PRINTF_TO_COMPLETE);

    /* Go to deep sleep after running the core benchmark */
    for (;;)
    {
        Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}

/*******************************************************************************
* Function Name: app_retarget_io_init
********************************************************************************
* Summary:
* User defined function to initialize the debug UART. 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void app_retarget_io_init(void)
{
    cy_rslt_t result;

    /* Initialize the SCB UART */
    result = (cy_rslt_t)Cy_SCB_UART_Init(CYBSP_DEBUG_UART_HW, 
                                        &CYBSP_DEBUG_UART_config, 
                                        &CYBSP_DEBUG_UART_context);
    
    /* UART init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable the SCB UART */
    Cy_SCB_UART_Enable(CYBSP_DEBUG_UART_HW);

    result = mtb_hal_uart_setup(&CYBSP_DEBUG_UART_hal_obj, 
                                &CYBSP_DEBUG_UART_hal_config, 
                                &CYBSP_DEBUG_UART_context, NULL);
    
    /* UART setup failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&CYBSP_DEBUG_UART_hal_obj);

    /* retarget-io init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }
}