/*
** ###################################################################
**
**     Copyright (c) 2016 Freescale Semiconductor, Inc.
**     Copyright 2017-2020 NXP
**     Copyright 2017-2021 Toradex
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/*!
 * @file
 *
 * File containing the implementation of the Apalis iMX8QM module.
 *
 * @addtogroup MX8QM_APALIS_BRD (BRD) Apalis iMX8QM Board
 *
 * Module for Apalis iMX8QM module access.
 *
 * @{
 */
/*==========================================================================*/

/* Includes */

#include "main/build_info.h"
#include "main/scfw.h"
#include "main/main.h"
#include "main/board.h"
#include "main/boot.h"
#include "main/soc.h"
#include "board/pmic.h"
#include "all_svc.h"
#include "drivers/lpi2c/fsl_lpi2c.h"
#include "drivers/pmic/fsl_pmic.h"
#include "drivers/pmic/pf8100/fsl_pf8100.h"
#include "drivers/rgpio/fsl_rgpio.h"
#include "drivers/snvs/fsl_snvs.h"
#include "drivers/lpuart/fsl_lpuart.h"
#include "drivers/sysctr/fsl_sysctr.h"
#include "drivers/drc/fsl_drc_cbt.h"
#include "drivers/drc/fsl_drc_derate.h"
#include "drivers/drc/fsl_drc_rdbi_deskew.h"
#include "drivers/drc/fsl_drc_dram_vref.h"
#include "drivers/systick/fsl_systick.h"
#include "pads.h"
#include "drivers/pad/fsl_pad.h"
#include "MX8QM/MX8QM_tdx_user_fuse_map.h"

#include "dcd/imx8_ramid1_dcd_1.6GHz_retention.h"
#include "dcd/imx8_ramid2_dcd_1.6GHz_retention.h"
#include "dcd/imx8_ramid3_dcd_1.6GHz_retention.h"

/* Local Defines */

/*!
 * @name Board Configuration
 * DO NOT CHANGE - must match object code.
 */
/** @{ */
#define BRD_NUM_RSRC            11U
#define BRD_NUM_CTRL            6U
/** @} */

/*!
 * @name Board Resources
 * DO NOT CHANGE - must match object code.
 */
/** @{ */
#define BRD_R_BOARD_PMIC_0      0U
#define BRD_R_BOARD_PMIC_1      1U
#define BRD_R_BOARD_PMIC_2      2U
#define BRD_R_BOARD_R0          3U
#define BRD_R_BOARD_R1          4U
#define BRD_R_BOARD_R2          5U
#define BRD_R_BOARD_R3          6U
#define BRD_R_BOARD_R4          7U
#define BRD_R_BOARD_R5          8U
#define BRD_R_BOARD_R6          9U
#define BRD_R_BOARD_R7          10U
/*@}*/

#if DEBUG_UART == 3
    /*! Use debugger terminal emulation */
    #define DEBUG_TERM_EMUL
#endif
#if DEBUG_UART == 2
    /*! Use alternate debug UART */
    #define ALT_DEBUG_SCU_UART
#endif
#if (defined(MONITOR) || defined(EXPORT_MONITOR) || defined(HAS_TEST) \
        || (DEBUG_UART == 1)) && !defined(DEBUG_TERM_EMUL) \
        && !defined(ALT_DEBUG_SCU_UART)
    #define ALT_DEBUG_UART
#endif

/*! Configure debug UART */
#ifdef ALT_DEBUG_SCU_UART
    #define LPUART_DEBUG        LPUART_SC
#else
    #define LPUART_DEBUG        LPUART_MCU_0
#endif

/*! Configure debug UART instance */
#ifdef ALT_DEBUG_SCU_UART
    #define LPUART_DEBUG_INST   0U
#else
    #define LPUART_DEBUG_INST   2U
#endif

#ifdef EMUL
    /*! Configure debug baud rate */
    #define DEBUG_BAUD          4000000U
#else
    /*! Configure debug baud rate */
    #define DEBUG_BAUD          115200U
#endif

/* Local Types */

/* Local Functions */

static void pmic_init(void);
static sc_err_t pmic_ignore_current_limit(uint8_t address);
static sc_err_t pmic_update_timing(uint8_t address);
static sc_err_t pmic_match_otp(uint8_t address, pmic_version_t ver);
static void board_get_pmic_info(sc_sub_t ss,pmic_id_t *pmic_id,
    uint32_t *pmic_reg, uint8_t *num_regs);

/* Local Variables */

static pmic_version_t pmic_ver;
static uint32_t temp_alarm0;
static uint32_t temp_alarm1;

/*!
 * This constant contains info to map resources to the board.
 * DO NOT CHANGE - must match object code.
 */
const sc_rsrc_map_t board_rsrc_map[BRD_NUM_RSRC_BRD] =
{
    RSRC(PMIC_0,   0,  0),
    RSRC(PMIC_1,   0,  1),
    RSRC(PMIC_2,   0,  2),
    RSRC(BOARD_R0, 0,  3),
    RSRC(BOARD_R1, 0,  4),
    RSRC(BOARD_R2, 0,  5),
    RSRC(BOARD_R3, 0,  6),
    RSRC(BOARD_R4, 0,  7),
    RSRC(BOARD_R5, 0,  8),
    RSRC(BOARD_R6, 0,  9),
    RSRC(BOARD_R7, 0, 10)
};

/* Block of comments that get processed for documentation
   DO NOT CHANGE - must match object code. */
#ifdef DOX
    RNFO() /* PMIC 0 */
    RNFO() /* PMIC 1 */
    RNFO() /* PMIC 2 */
    RNFO() /* Misc. board component 0 */
    RNFO() /* Misc. board component 1 */
    RNFO() /* Misc. board component 2 */
    RNFO() /* Misc. board component 3 */
    RNFO() /* Misc. board component 4 */
    RNFO() /* Misc. board component 5 */
    RNFO() /* Misc. board component 6 */
    RNFO() /* Misc. board component 7 */
    TNFO(PMIC_0, TEMP,     RO, x, 8) /* Temperature sensor temp */
    TNFO(PMIC_0, TEMP_HI,  RW, x, 8) /* Temperature sensor high limit alarm temp */
    TNFO(PMIC_1, TEMP,     RO, x, 8) /* Temperature sensor temp */
    TNFO(PMIC_1, TEMP_HI,  RW, x, 8) /* Temperature sensor high limit alarm temp */
    TNFO(PMIC_2, TEMP,     RO, x, 8) /* Temperature sensor temp */
    TNFO(PMIC_2, TEMP_HI,  RW, x, 8) /* Temperature sensor high limit alarm temp */
#endif

/* External Variables */

const sc_rm_idx_t board_num_rsrc = BRD_NUM_RSRC_BRD;

/*!
 * External variable for specing DDR periodic training.
 */
#ifdef BD_LPDDR4_INC_DQS2DQ
const uint32_t board_ddr_period_ms = 3000U;
#else
const uint32_t board_ddr_period_ms = 0U;
#endif

const uint32_t board_ddr_derate_period_ms = 1000U;

/*--------------------------------------------------------------------------*/
/* Init                                                                     */
/*--------------------------------------------------------------------------*/
void board_init(boot_phase_t phase)
{
    ss_print(3, "board_init(%d)\n", phase);

    if (phase == BOOT_PHASE_FINAL_INIT)
    {
        /* Configure SNVS button for rising edge */
        SNVS_ConfigButton(SNVS_DRV_BTN_CONFIG_RISINGEDGE, SC_TRUE);

        /* Init PMIC if not already done */
        pmic_init();
    }
}

/*--------------------------------------------------------------------------*/
/* Return the debug UART info                                               */
/*--------------------------------------------------------------------------*/
LPUART_Type *board_get_debug_uart(uint8_t *inst, uint32_t *baud)
{
    #if (defined(ALT_DEBUG_UART) || defined(ALT_DEBUG_SCU_UART)) \
            && !defined(DEBUG_TERM_EMUL)
        *inst = LPUART_DEBUG_INST;
        *baud = DEBUG_BAUD;

        return LPUART_DEBUG;
    #else
        return NULL;
    #endif
}

/*--------------------------------------------------------------------------*/
/* Configure debug UART                                                     */
/*--------------------------------------------------------------------------*/
void board_config_debug_uart(sc_bool_t early_phase)
{
    #if (defined(ALT_DEBUG_UART) || defined(ALT_DEBUG_SCU_UART) || \
            defined(DEBUG_TERM_EMUL)) && defined(DEBUG) && !defined(SIMU)
        static sc_bool_t banner = SC_FALSE;
    #endif

    #if defined(ALT_DEBUG_SCU_UART) && !defined(DEBUG_TERM_EMUL) \
            && defined(DEBUG) && !defined(SIMU)
        /* Power up UART */
        pm_force_resource_power_mode_v(SC_R_SC_UART,
            SC_PM_PW_MODE_ON);

        /* Check if debug disabled */
        if (SCFW_DBG_READY == 0U)
        {
            main_config_debug_uart(LPUART_DEBUG, SC_24MHZ);
        }
    #elif defined(ALT_DEBUG_UART) && defined(DEBUG) && !defined(SIMU)
        /* Use M4 UART if ALT_DEBUG_UART defined */
        /* Return if debug already enabled */
        if ((SCFW_DBG_READY == 0U) && (early_phase == SC_FALSE))
        {
            sc_pm_clock_rate_t rate = SC_24MHZ;

            /* Configure pads */
            pad_force_mux(SC_P_M40_I2C0_SDA, 1,
                SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);
            pad_force_mux(SC_P_M40_I2C0_SCL, 1,
                SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);

            /* Power and enable clock */
            pm_force_resource_power_mode_v(SC_R_SC_PID0,
                SC_PM_PW_MODE_ON);
            pm_force_resource_power_mode_v(SC_R_DBLOGIC,
                SC_PM_PW_MODE_ON);
            pm_force_resource_power_mode_v(SC_R_DB, SC_PM_PW_MODE_ON);
            pm_force_resource_power_mode_v(SC_R_M4_0_UART,
                SC_PM_PW_MODE_ON);
            (void) pm_set_clock_rate(SC_PT, SC_R_M4_0_UART, SC_PM_CLK_PER,
                &rate);
            (void) pm_clock_enable(SC_PT, SC_R_M4_0_UART, SC_PM_CLK_PER,
                SC_TRUE, SC_FALSE);

            /* Configure UART */
            main_config_debug_uart(LPUART_DEBUG, rate);
        }
    #elif defined(DEBUG_TERM_EMUL) && defined(DEBUG) && !defined(SIMU)
        *SCFW_DBG_TX_PTR = 0U;
        *SCFW_DBG_RX_PTR = 0U;
        /* Set to 2 for JTAG emulation */
        SCFW_DBG_READY = 2U;
    #endif

    #if (defined(ALT_DEBUG_UART) || defined(ALT_DEBUG_SCU_UART) || \
            defined(DEBUG_TERM_EMUL)) && defined(DEBUG) && !defined(SIMU)
        if (banner == SC_FALSE)
        {
            debug_print(1, 
                "\nHello from SCU (Build %u, Commit %08x, %s %s)\n\n",
                SCFW_BUILD, SCFW_COMMIT, SCFW_DATE, SCFW_TIME);
            banner = SC_TRUE;
        }
     #endif
}

/*--------------------------------------------------------------------------*/
/* Disable debug UART                                                       */
/*--------------------------------------------------------------------------*/
void board_disable_debug_uart(void)
{
    /* Use M4 UART if ALT_DEBUG_UART defined */
    #if defined(ALT_DEBUG_UART) && defined(DEBUG) && !defined(SIMU)
        /* Return if debug already disabled */
        if (SCFW_DBG_READY != 0U)
        {
            /* Disable use of UART */
            SCFW_DBG_READY = 0U;

            // UART deinit to flush TX buffers
            LPUART_Deinit(LPUART_DEBUG);

            /* Turn off UART */
            pm_force_resource_power_mode_v(SC_R_M4_0_UART,
                SC_PM_PW_MODE_OFF);
        }
    #endif
}

/*--------------------------------------------------------------------------*/
/* Configure SCFW resource/pins                                             */
/*--------------------------------------------------------------------------*/
void board_config_sc(sc_rm_pt_t pt_sc)
{
    /* By default, the SCFW keeps most of the resources found in the SCU
     * subsystem. It also keeps the SCU/PMIC pads required for the main
     * code to function. Any additional resources or pads required for
     * the board code to run should be kept here. This is done by marking
     * them as not movable.
     */
    #ifdef ALT_DEBUG_UART
        (void) rm_set_resource_movable(SC_PT, SC_R_M4_0_UART, SC_R_M4_0_UART,
            SC_FALSE);
        (void) rm_set_pad_movable(SC_PT, SC_P_M40_I2C0_SCL, SC_P_M40_I2C0_SDA,
            SC_FALSE);
    #endif

    (void) rm_set_resource_movable(pt_sc, SC_R_SC_I2C, SC_R_SC_I2C,
        SC_FALSE);
    (void) rm_set_pad_movable(pt_sc, SC_P_PMIC_I2C_SDA, SC_P_PMIC_I2C_SCL,
        SC_FALSE);
    /* SCU_UART0_RX, SCU_UART0_TX */
    #ifdef ALT_DEBUG_SCU_UART
        (void) rm_set_resource_movable(SC_PT, SC_R_SC_UART, SC_R_SC_UART,
            SC_FALSE);
        (void) rm_set_pad_movable(pt_sc, SC_P_SCU_GPIO0_00,
            SC_P_SCU_GPIO0_01, SC_FALSE);
    #endif
    /* RESET_MOCI#_DRV, HSIC_HUB_CONNECT */
    (void) rm_set_pad_movable(pt_sc, SC_P_SCU_GPIO0_02, SC_P_SCU_GPIO0_03,
        SC_TRUE);
    /* PMIC_PGOOD */
    (void) rm_set_pad_movable(pt_sc, SC_P_SCU_GPIO0_04, SC_P_SCU_GPIO0_04,
        SC_FALSE);
    /* HSIC_INT_N, HSIC_RESET_N, Wi-Fi_SUSCLK_32k */
    (void) rm_set_pad_movable(pt_sc, SC_P_SCU_GPIO0_05, SC_P_SCU_GPIO0_07,
        SC_TRUE);

    board_print(3, "board_config_sc()\n");
}

/*--------------------------------------------------------------------------*/
/* Get board parameter                                                      */
/*--------------------------------------------------------------------------*/
board_parm_rtn_t board_parameter(board_parm_t parm)
{
    board_parm_rtn_t rtn = BOARD_PARM_RTN_NOT_USED;

    /* Note return values are usually static. Can be made dynamic by storing
       return in a global variable and setting using board_set_control() */

    switch (parm)
    {
        /* Used whenever HSIO SS powered up. Valid return values are
           BOARD_PARM_RTN_EXTERNAL or BOARD_PARM_RTN_INTERNAL */
        case BOARD_PARM_PCIE_PLL :
            rtn = BOARD_PARM_RTN_EXTERNAL;
            break;
        /* Supply ramp delay in usec for KS1 exit */
        case BOARD_PARM_KS1_RESUME_USEC:
            rtn = BOARD_KS1_RESUME_USEC;
            break;
        /* Control if retention is applied during KS1 */
        case BOARD_PARM_KS1_RETENTION:
            rtn = BOARD_KS1_RETENTION;
            break;
        /* Control if ONOFF button can wake from KS1 */
        case BOARD_PARM_KS1_ONOFF_WAKE:
            rtn = BOARD_KS1_ONOFF_WAKE;
            break;
        /* DC0 PLL0 spread spectrum config */
        case BOARD_PARM_DC0_PLL0_SSC:
            rtn = BOARD_PARM_RTN_NOT_USED;
            break;
        /* DC0 PLL1 spread spectrum config */
        case BOARD_PARM_DC0_PLL1_SSC:
            rtn = BOARD_PARM_RTN_NOT_USED;
            break;
        /* DC1 PLL0 spread spectrum config */
        case BOARD_PARM_DC1_PLL0_SSC:
            rtn = BOARD_PARM_RTN_NOT_USED;
            break;
        /* DC1 PLL1 spread spectrum config */
        case BOARD_PARM_DC1_PLL1_SSC:
            rtn = BOARD_PARM_RTN_NOT_USED;
            break;
        /* Control if SC WDOG configuration during KS1 */
        case BOARD_PARM_KS1_WDOG_WAKE:
            rtn = BOARD_PARM_KS1_WDOG_WAKE_ENABLE;
            break;
        default :
            ; /* Intentional empty default */
            break;
    }

    return rtn;
}

/*--------------------------------------------------------------------------*/
/* Get resource avaiability info                                            */
/*--------------------------------------------------------------------------*/
sc_bool_t board_rsrc_avail(sc_rsrc_t rsrc)
{
    sc_bool_t rtn = SC_TRUE;

    /* Return SC_FALSE here if a resource isn't available due to board
       connections (typically lack of power). Examples incluse DRC_0/1
       and ADC. - DDR_00060 */

    /* The value here may be overridden by SoC fuses or emulation config */

    /* Note return values are usually static. Can be made dynamic by storing
       return in a global variable and setting using board_set_control() */

    #if defined(BD_DDR_RET_NUM_DRC) && (BD_DDR_RET_NUM_DRC == 1U)
        if(rsrc == SC_R_DRC_1)
        {
            rtn = SC_FALSE;
        }
    #endif
    if(rsrc == SC_R_PMIC_2)
    {
        rtn = SC_FALSE;
    }

    return rtn;
}

/*--------------------------------------------------------------------------*/
/* Override QoS configuration                                               */
/*--------------------------------------------------------------------------*/
void board_qos_config(sc_sub_t ss)
{
}

/*--------------------------------------------------------------------------*/
/* Init DDR helpers                                                         */
/*--------------------------------------------------------------------------*/

static int board_init_ddr_get_ramid(void) {
    int tdx2_ramid = OTP_TDX2_RAMID;
    /*
     * RAMID overview
     *
     * 0x0  Legacy RAM handling
     * 0x1  LPDDR4 4GiB (32Mb x32DQ x8banks x2Channels)
     *      - Apalis iMX8QM 4GB WB V1.0B    (K4F6E304HB-MGCJ)
     *      - Apalis iMX8QM 4GB IT V1.0B    (K4F6E304HB-MGCJ)
     *      - Apalis iMX8QM 4GB WB IT V1.1A (MT53D512M32D2DS-046 IT:D)
     *      - Apalis iMX8QM 4GB IT V1.1A    (MT53D512M32D2DS-046 IT:D)
     * 0x2  LPDDR4 2GiB (32Mb x16DQ x8banks x2Channels)
     *      - Apalis iMX8QP 2GB WB V1.0B    (MT53E256M32D2DS-053 WT:B)
     *      - Apalis iMX8QP 2GB WB V1.1A    (MT53E256M32D2DS-053 WT:B)
     *      - Apalis iMX8QP 2GB V1.0B       (MT53E256M32D2DS-053 WT:B)
     *      - Apalis iMX8QP 2GB V1.1A       (MT53E256M32D2DS-053 WT:B)
     *
     * 0xF  Last RAMID, reserved to indicate RAM handling with Toradex PID8
     */

    /* Some debug prints */
#if DEBUG_LEVEL >= 4
    if (OTP_TDX2_BLOCK) {
        board_print(4, "First Toradex fusing block got overwritten.\n");
        board_print(4, "Running on a board with PID8: %08d\n", OTP_TDX2_PID8);
        board_print(4, "OTP_TDX2_PID4_SKU | OTP_TDX2_PID4_VERS: %d | %d\n", OTP_TDX2_PID4_SKU, OTP_TDX2_PID4_VERS);
        board_print(4, "OTP_TDX2_BLOCK: 0x%x\n", OTP_TDX2_BLOCK);
    } else if (OTP_TDX1_BLOCK) {
        board_print(4, "Running on a board with PID8: %08d\n", OTP_TDX1_PID8);
        board_print(4, "OTP_TDX1_PID4_SKU | OTP_TDX1_PID4_VERS: %d | %d\n", OTP_TDX1_PID4_SKU, OTP_TDX1_PID4_VERS);
        board_print(4, "OTP_TDX1_BLOCK: 0x%x\n", OTP_TDX1_BLOCK);
    } else {
        board_print(4, "Toradex fusing block not used. Handling RAM in legacy mode.\n");
    }
#endif

    if (tdx2_ramid) {
        board_print(4, "First RAM-Timing got overwritten, using now RAMID " \
                       "0x%x\n", tdx2_ramid);
        return tdx2_ramid;
    } else {
        board_print(4, "RAMID found: 0x%x\n", OTP_TDX1_RAMID);
        return OTP_TDX1_RAMID;
    }
}

static soc_ddr_ret_info_t* board_init_ddr_ramid_1(void) {
    /*
     * Variables for DDR retention
     */
    /* Storage for DRC registers */
    static ddrc board_ddr_ret_drc_inst[BD_DDR_RET_NUM_DRC];

    /* Storage for DRC PHY registers */
    static ddr_phy board_ddr_ret_drc_phy_inst[BD_DDR_RET_NUM_DRC];

    /* Storage for DDR regions */
    static uint32_t board_ddr_ret_buf1[RAMID1_BD_DDR_RET_REGION1_SIZE];
    #ifdef RAMID1_BD_DDR_RET_REGION2_SIZE
    static uint32_t board_ddr_ret_buf2[RAMID1_BD_DDR_RET_REGION2_SIZE];
    #endif
    #ifdef RAMID1_BD_DDR_RET_REGION3_SIZE
    static uint32_t board_ddr_ret_buf3[RAMID1_BD_DDR_RET_REGION3_SIZE];
    #endif
    #ifdef RAMID1_BD_DDR_RET_REGION4_SIZE
    static uint32_t board_ddr_ret_buf4[RAMID1_BD_DDR_RET_REGION4_SIZE];
    #endif
    #ifdef RAMID1_BD_DDR_RET_REGION5_SIZE
    static uint32_t board_ddr_ret_buf5[RAMID1_BD_DDR_RET_REGION5_SIZE];
    #endif
    #ifdef RAMID1_BD_DDR_RET_REGION6_SIZE
    static uint32_t board_ddr_ret_buf6[RAMID1_BD_DDR_RET_REGION6_SIZE];
    #endif

    /* DDR region descriptors */
    static const soc_ddr_ret_region_t board_ddr_ret_region[RAMID1_BD_DDR_RET_NUM_REGION] =
    {
        { RAMID1_BD_DDR_RET_REGION1_ADDR, RAMID1_BD_DDR_RET_REGION1_SIZE, board_ddr_ret_buf1 },
    #ifdef RAMID1_BD_DDR_RET_REGION2_SIZE
        { RAMID1_BD_DDR_RET_REGION2_ADDR, RAMID1_BD_DDR_RET_REGION2_SIZE, board_ddr_ret_buf2 },
    #endif
    #ifdef RAMID1_BD_DDR_RET_REGION3_SIZE
        { RAMID1_BD_DDR_RET_REGION3_ADDR, RAMID1_BD_DDR_RET_REGION3_SIZE, board_ddr_ret_buf3 },
    #endif
    #ifdef RAMID1_BD_DDR_RET_REGION4_SIZE
        { RAMID1_BD_DDR_RET_REGION4_ADDR, RAMID1_BD_DDR_RET_REGION4_SIZE, board_ddr_ret_buf4 },
    #endif
    #ifdef RAMID1_BD_DDR_RET_REGION5_SIZE
        { RAMID1_BD_DDR_RET_REGION5_ADDR, RAMID1_BD_DDR_RET_REGION5_SIZE, board_ddr_ret_buf5 },
    #endif
    #ifdef RAMID1_BD_DDR_RET_REGION6_SIZE
        { RAMID1_BD_DDR_RET_REGION6_ADDR, RAMID1_BD_DDR_RET_REGION6_SIZE, board_ddr_ret_buf6 }
    #endif
    };

    /* DDR retention descriptor passed to SCFW */
    static soc_ddr_ret_info_t board_ddr_ret_info_qm =
    {
        BD_DDR_RET_NUM_DRC, board_ddr_ret_drc_inst, board_ddr_ret_drc_phy_inst,
        RAMID1_BD_DDR_RET_NUM_REGION, board_ddr_ret_region
    };
    return &board_ddr_ret_info_qm;
}

static soc_ddr_ret_info_t* board_init_ddr_ramid_2(void) {
    /*
     * Variables for DDR retention
     */
    /* Storage for DRC registers */
    static ddrc board_ddr_ret_drc_inst[BD_DDR_RET_NUM_DRC];

    /* Storage for DRC PHY registers */
    static ddr_phy board_ddr_ret_drc_phy_inst[BD_DDR_RET_NUM_DRC];

    /* Storage for DDR regions */
    static uint32_t board_ddr_ret_buf1[RAMID2_BD_DDR_RET_REGION1_SIZE];
    #ifdef RAMID2_BD_DDR_RET_REGION2_SIZE
    static uint32_t board_ddr_ret_buf2[RAMID2_BD_DDR_RET_REGION2_SIZE];
    #endif
    #ifdef RAMID2_BD_DDR_RET_REGION3_SIZE
    static uint32_t board_ddr_ret_buf3[RAMID2_BD_DDR_RET_REGION3_SIZE];
    #endif
    #ifdef RAMID2_BD_DDR_RET_REGION4_SIZE
    static uint32_t board_ddr_ret_buf4[RAMID2_BD_DDR_RET_REGION4_SIZE];
    #endif
    #ifdef RAMID2_BD_DDR_RET_REGION5_SIZE
    static uint32_t board_ddr_ret_buf5[RAMID2_BD_DDR_RET_REGION5_SIZE];
    #endif
    #ifdef RAMID2_BD_DDR_RET_REGION6_SIZE
    static uint32_t board_ddr_ret_buf6[RAMID2_BD_DDR_RET_REGION6_SIZE];
    #endif

    /* DDR region descriptors */
    static const soc_ddr_ret_region_t board_ddr_ret_region[RAMID2_BD_DDR_RET_NUM_REGION] =
    {
        { RAMID2_BD_DDR_RET_REGION1_ADDR, RAMID2_BD_DDR_RET_REGION1_SIZE, board_ddr_ret_buf1 },
    #ifdef RAMID2_BD_DDR_RET_REGION2_SIZE
        { RAMID2_BD_DDR_RET_REGION2_ADDR, RAMID2_BD_DDR_RET_REGION2_SIZE, board_ddr_ret_buf2 },
    #endif
    #ifdef RAMID2_BD_DDR_RET_REGION3_SIZE
        { RAMID2_BD_DDR_RET_REGION3_ADDR, RAMID2_BD_DDR_RET_REGION3_SIZE, board_ddr_ret_buf3 },
    #endif
    #ifdef RAMID2_BD_DDR_RET_REGION4_SIZE
        { RAMID2_BD_DDR_RET_REGION4_ADDR, RAMID2_BD_DDR_RET_REGION4_SIZE, board_ddr_ret_buf4 },
    #endif
    #ifdef RAMID2_BD_DDR_RET_REGION5_SIZE
        { RAMID2_BD_DDR_RET_REGION5_ADDR, RAMID2_BD_DDR_RET_REGION5_SIZE, board_ddr_ret_buf5 },
    #endif
    #ifdef RAMID2_BD_DDR_RET_REGION6_SIZE
        { RAMID2_BD_DDR_RET_REGION6_ADDR, RAMID2_BD_DDR_RET_REGION6_SIZE, board_ddr_ret_buf6 }
    #endif
    };

    /* DDR retention descriptor passed to SCFW */
    static soc_ddr_ret_info_t board_ddr_ret_info_qp =
    {
        BD_DDR_RET_NUM_DRC, board_ddr_ret_drc_inst, board_ddr_ret_drc_phy_inst,
        RAMID2_BD_DDR_RET_NUM_REGION, board_ddr_ret_region
    };
    return &board_ddr_ret_info_qp;
}

static soc_ddr_ret_info_t* board_init_ddr_ramid_3(void) {
    /*
     * Variables for DDR retention
     */
    /* Storage for DRC registers */
    static ddrc board_ddr_ret_drc_inst[BD_DDR_RET_NUM_DRC];

    /* Storage for DRC PHY registers */
    static ddr_phy board_ddr_ret_drc_phy_inst[BD_DDR_RET_NUM_DRC];

    /* Storage for DDR regions */
    static uint32_t board_ddr_ret_buf1[RAMID3_BD_DDR_RET_REGION1_SIZE];
    #ifdef RAMID3_BD_DDR_RET_REGION2_SIZE
    static uint32_t board_ddr_ret_buf2[RAMID3_BD_DDR_RET_REGION2_SIZE];
    #endif
    #ifdef RAMID3_BD_DDR_RET_REGION3_SIZE
    static uint32_t board_ddr_ret_buf3[RAMID3_BD_DDR_RET_REGION3_SIZE];
    #endif
    #ifdef RAMID3_BD_DDR_RET_REGION4_SIZE
    static uint32_t board_ddr_ret_buf4[RAMID3_BD_DDR_RET_REGION4_SIZE];
    #endif
    #ifdef RAMID3_BD_DDR_RET_REGION5_SIZE
    static uint32_t board_ddr_ret_buf5[RAMID3_BD_DDR_RET_REGION5_SIZE];
    #endif
    #ifdef RAMID3_BD_DDR_RET_REGION6_SIZE
    static uint32_t board_ddr_ret_buf6[RAMID3_BD_DDR_RET_REGION6_SIZE];
    #endif

    /* DDR region descriptors */
    static const soc_ddr_ret_region_t board_ddr_ret_region[RAMID3_BD_DDR_RET_NUM_REGION] =
    {
        { RAMID3_BD_DDR_RET_REGION1_ADDR, RAMID3_BD_DDR_RET_REGION1_SIZE, board_ddr_ret_buf1 },
    #ifdef RAMID3_BD_DDR_RET_REGION2_SIZE
        { RAMID3_BD_DDR_RET_REGION2_ADDR, RAMID3_BD_DDR_RET_REGION2_SIZE, board_ddr_ret_buf2 },
    #endif
    #ifdef RAMID3_BD_DDR_RET_REGION3_SIZE
        { RAMID3_BD_DDR_RET_REGION3_ADDR, RAMID3_BD_DDR_RET_REGION3_SIZE, board_ddr_ret_buf3 },
    #endif
    #ifdef RAMID3_BD_DDR_RET_REGION4_SIZE
        { RAMID3_BD_DDR_RET_REGION4_ADDR, RAMID3_BD_DDR_RET_REGION4_SIZE, board_ddr_ret_buf4 },
    #endif
    #ifdef RAMID3_BD_DDR_RET_REGION5_SIZE
        { RAMID3_BD_DDR_RET_REGION5_ADDR, RAMID3_BD_DDR_RET_REGION5_SIZE, board_ddr_ret_buf5 },
    #endif
    #ifdef RAMID3_BD_DDR_RET_REGION6_SIZE
        { RAMID3_BD_DDR_RET_REGION6_ADDR, RAMID3_BD_DDR_RET_REGION6_SIZE, board_ddr_ret_buf6 }
    #endif
    };

    /* DDR retention descriptor passed to SCFW */
    static soc_ddr_ret_info_t board_ddr_ret_info_qp =
    {
        BD_DDR_RET_NUM_DRC, board_ddr_ret_drc_inst, board_ddr_ret_drc_phy_inst,
        RAMID3_BD_DDR_RET_NUM_REGION, board_ddr_ret_region
    };
    return &board_ddr_ret_info_qp;
}

/*--------------------------------------------------------------------------*/
/* Init DDR                                                                 */
/*--------------------------------------------------------------------------*/
sc_err_t board_init_ddr(sc_bool_t early, sc_bool_t ddr_initialized)
{
    static soc_ddr_ret_info_t* board_ddr_ret_info = NULL;

    switch (board_init_ddr_get_ramid())
    {
    case 0x3:
        #if defined(RAMID3_BD_DDR_RET) & !defined(SKIP_DDR)
            board_ddr_ret_info = board_init_ddr_ramid_3();
        #endif
        break;
    case 0x2:
        #if defined(RAMID2_BD_DDR_RET) & !defined(SKIP_DDR)
            board_ddr_ret_info = board_init_ddr_ramid_2();
        #endif
        break;
    case 0x1:
        #if defined(RAMID1_BD_DDR_RET) & !defined(SKIP_DDR)
            board_ddr_ret_info = board_init_ddr_ramid_1();
        #endif
        break;
    default:
        /* legacy ram handling */
        if (OTP_AP_1_DIS != 0x0) { /* QP has one A72 core disabled */
            #if defined(RAMID2_BD_DDR_RET) & !defined(SKIP_DDR)
                board_ddr_ret_info = board_init_ddr_ramid_2();
            #endif
        } else {
            #if defined(RAMID1_BD_DDR_RET) & !defined(SKIP_DDR)
                board_ddr_ret_info = board_init_ddr_ramid_1();
            #endif
        }
        break;
    }

    board_print(3, "board_init_ddr(%d)\n", early);

    #ifdef SKIP_DDR
        return SC_ERR_UNAVAILABLE;
    #else
        sc_err_t err = SC_ERR_NONE;

        /* Don't power up DDR for M4s */
        ASRT_ERR(early == SC_FALSE, SC_ERR_UNAVAILABLE);

        if ((err == SC_ERR_NONE) && (ddr_initialized == SC_FALSE))
        {
            board_print(1, "SCFW: ");
            err = board_ddr_config(SC_FALSE, BOARD_DDR_COLD_INIT);
            #ifdef LP4_MANUAL_DERATE_WORKAROUND
                ddrc_lpddr4_derate_init(BD_DDR_RET_NUM_DRC);
            #endif
        }

        #ifdef DEBUG_BOARD
            if (err == SC_ERR_NONE)
            {
                uint32_t rate = 0U;
                sc_err_t rate_err = SC_ERR_FAIL;

                if (rm_is_resource_avail(SC_R_DRC_0))
                {
                    rate_err = pm_get_clock_rate(SC_PT, SC_R_DRC_0,
                        SC_PM_CLK_SLV_BUS, &rate);
                }
                else if (rm_is_resource_avail(SC_R_DRC_1))
                {
                    rate_err = pm_get_clock_rate(SC_PT, SC_R_DRC_1,
                        SC_PM_CLK_SLV_BUS, &rate);
                }
                else
                {
                    ; /* Intentional empty else */
                }
                if (rate_err == SC_ERR_NONE)
                {
                    board_print(1, "DDR frequency = %u\n", rate * 2U);
                }
            }
        #endif

        if (err == SC_ERR_NONE)
        {
            if (board_ddr_ret_info)
                soc_ddr_config_retention(board_ddr_ret_info);

            #ifdef BD_LPDDR4_INC_DQS2DQ
            #ifdef BOARD_DQS2DQ_SYNC
                /* DDR_00040 */
                soc_ddr_dqs2dq_config(&board_dqs2dq_sync_info);
            #endif
                if (board_ddr_period_ms != 0U)
                {
                    soc_ddr_dqs2dq_init();
                }
            #endif
        }
        #ifdef LP4_MANUAL_DERATE_WORKAROUND
            board_ddr_derate_periodic_enable(SC_TRUE);
        #endif

        return err;
    #endif
}

/*--------------------------------------------------------------------------*/
/* Take action on DDR                                                       */
/*--------------------------------------------------------------------------*/
sc_err_t board_ddr_config(bool rom_caller, board_ddr_action_t action)
{
    /* Note this is called by the ROM before the SCFW is initialized.
     * Do NOT make any unqualified calls to any other APIs. See DDR_00010.
     */

    sc_err_t err = SC_ERR_NONE;
#ifdef LP4_MANUAL_DERATE_WORKAROUND
    sc_bool_t polling = SC_FALSE;
#endif

    switch(action)
    {
        case BOARD_DDR_PERIODIC:
    #ifdef BD_LPDDR4_INC_DQS2DQ
            soc_ddr_dqs2dq_periodic();
    #endif
            break;
        case BOARD_DDR_SR_DRC_OFF_ENTER:
    #ifdef LP4_MANUAL_DERATE_WORKAROUND
            board_ddr_derate_periodic_enable(SC_FALSE);
    #endif
            board_ddr_periodic_enable(SC_FALSE);
    #if defined RAMID1_BD_DDR_RET || defined RAMID2_BD_DDR_RET || defined RAMID3_BD_DDR_RET
            soc_ddr_enter_retention();
    #endif
            break;
        case BOARD_DDR_SR_DRC_OFF_EXIT:
    #if defined RAMID1_BD_DDR_RET || defined RAMID2_BD_DDR_RET || defined RAMID3_BD_DDR_RET
            soc_ddr_exit_retention();
    #endif
    #ifdef LP4_MANUAL_DERATE_WORKAROUND
            ddrc_lpddr4_derate_init(BD_DDR_RET_NUM_DRC);
            board_ddr_derate_periodic_enable(SC_TRUE);
    #endif
    #ifdef BD_LPDDR4_INC_DQS2DQ
            soc_ddr_dqs2dq_init();
    #endif
            board_ddr_periodic_enable(SC_TRUE);
            break;
        case BOARD_DDR_SR_DRC_ON_ENTER:
    #ifdef LP4_MANUAL_DERATE_WORKAROUND
            board_ddr_derate_periodic_enable(SC_FALSE);
    #endif
            board_ddr_periodic_enable(SC_FALSE);
            soc_self_refresh_power_down_clk_disable_entry();
            break;
        case BOARD_DDR_SR_DRC_ON_EXIT:
            soc_refresh_power_down_clk_disable_exit();
    #ifdef LP4_MANUAL_DERATE_WORKAROUND
            ddrc_lpddr4_derate_init(BD_DDR_RET_NUM_DRC);
            board_ddr_derate_periodic_enable(SC_TRUE);
    #endif
    #ifdef BD_LPDDR4_INC_DQS2DQ
            soc_ddr_dqs2dq_periodic();
    #endif
            board_ddr_periodic_enable(SC_TRUE);
            break;
        case BOARD_DDR_PERIODIC_HALT:
    #ifdef LP4_MANUAL_DERATE_WORKAROUND
            board_ddr_derate_periodic_enable(SC_FALSE);
    #endif
            board_ddr_periodic_enable(SC_FALSE);
            break;
        case BOARD_DDR_PERIODIC_RESTART:
    #ifdef LP4_MANUAL_DERATE_WORKAROUND
            ddrc_lpddr4_derate_init(BD_DDR_RET_NUM_DRC);
            board_ddr_derate_periodic_enable(SC_TRUE);
    #endif
    #ifdef BD_LPDDR4_INC_DQS2DQ
            soc_ddr_dqs2dq_periodic();
    #endif
            board_ddr_periodic_enable(SC_TRUE);
            break;
    #ifdef LP4_MANUAL_DERATE_WORKAROUND
        case BOARD_DDR_DERATE_PERIODIC:
            polling = ddrc_lpddr4_derate_periodic(BD_DDR_RET_NUM_DRC);
            if (polling != SC_TRUE)
            {
                board_ddr_derate_periodic_enable(SC_FALSE);
            }
            break;
    #endif
        case BOARD_DDR0_VREF:
            /* Supports stress test tool - DDR_00070 */
            #if defined(MONITOR) || defined(EXPORT_MONITOR)
                // Launch VREF training
                DRAM_VREF_training_hw(0);
            #else
                // Run vref training
                DRAM_VREF_training_sw(0);
            #endif
            break;
        case BOARD_DDR1_VREF:
            #if defined(MONITOR) || defined(EXPORT_MONITOR)
                // Launch VREF training
                DRAM_VREF_training_hw(1);
            #else
                // Run vref training
                DRAM_VREF_training_sw(1);
            #endif
            break;
        default:
            switch (board_init_ddr_get_ramid())
            {
            case 0x3:
                #include "dcd/imx8_ramid3_dcd_1.6GHz.h"
                break;
            case 0x2:
                #include "dcd/imx8_ramid2_dcd_1.6GHz.h"
                break;
            case 0x1:
                #include "dcd/imx8_ramid1_dcd_1.6GHz.h"
                break;
            default:
                /* legacy ram handling */
                if (OTP_AP_1_DIS != 0x0) { /* QP has one A72 core disabled */
                    #include "dcd/imx8_ramid2_dcd_1.6GHz.h"
                } else {
                    #include "dcd/imx8_ramid1_dcd_1.6GHz.h"
                }
                break;
            }
            break;
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* Configure the system (inc. additional resource partitions)               */
/*--------------------------------------------------------------------------*/
void board_system_config(sc_bool_t early, sc_rm_pt_t pt_boot)
{
    sc_err_t err = SC_ERR_NONE;

    /* This function configures the system. It usually partitions
       resources according to the system design. It must be modified by
       customers. Partitions should then be specified using the mkimage
       -p option. */

    /* Note the configuration here is for NXP test purposes */

    sc_bool_t alt_config = SC_FALSE;
    sc_bool_t no_ap = SC_FALSE;
    sc_bool_t ddrtest = SC_FALSE;

    /* Get boot parameters. See the Boot Flags section for definition
       of these flags.*/
    boot_get_data(NULL, NULL, NULL, NULL, NULL, NULL, &alt_config,
        NULL, &ddrtest, &no_ap, NULL);

    board_print(3, "board_system_config(%d, %d)\n", early, alt_config);

    /* Name default partitions */
    PARTITION_NAME(SC_PT, "SCU");
    PARTITION_NAME(SECO_PT, "SECO");
    PARTITION_NAME(pt_boot, "BOOT");

    /* Configure initial resource allocation (note additional allocation
       and assignments can be made by the SCFW clients at run-time */
    if (alt_config != SC_FALSE)
    {
        sc_rm_pt_t pt_m4_0 = SC_RM_NUM_PARTITION;
        sc_rm_pt_t pt_m4_1 = SC_RM_NUM_PARTITION;

        #ifdef BOARD_RM_DUMP
            rm_dump(pt_boot);
        #endif

        /* Name boot partition */
        PARTITION_NAME(pt_boot, "AP0");

        /* Create M4 0 partition */
        if (rm_is_resource_avail(SC_R_M4_0_PID0) != SC_FALSE)
        {
            sc_rm_mr_t mr;

            /* List of resources */
            static const sc_rsrc_t rsrc_list[7U] =
            {
                SC_R_SYSTEM,
                SC_R_IRQSTR_M4_0,
                SC_R_MU_5B,
                SC_R_MU_7A,
                SC_R_MU_8B,
                SC_R_GPT_4,
                SC_R_SECO_MU_4
            };

            /* List of pads */
            static const sc_pad_t pad_list[2U] =
            {
                RM_RANGE(SC_P_M40_I2C0_SCL, SC_P_M40_GPIO0_01)
            };

            /* List of memory regions */
            static const sc_rm_mem_list_t mem_list[2U] =
            {
                {0x088000000ULL, 0x0887FFFFFULL},
                {0x008081000ULL, 0x008180FFFULL}
            };

            /* Create partition */
            BRD_ERR(rm_partition_create(pt_boot, &pt_m4_0, SC_FALSE,
                SC_TRUE, SC_FALSE, SC_TRUE, SC_FALSE, SC_R_M4_0_PID0,
                rsrc_list, ARRAY_SIZE(rsrc_list),
                pad_list, ARRAY_SIZE(pad_list),
                mem_list, ARRAY_SIZE(mem_list)));

            /* Name partition for debug */
            PARTITION_NAME(pt_m4_0, "MCU0");
            
            /* Allow AP to use SYSTEM (not production!) */
            BRD_ERR(rm_set_peripheral_permissions(SC_PT, SC_R_SYSTEM,
                pt_boot, SC_RM_PERM_SEC_RW));

            /* Move M4 0 TCM */
            BRD_ERR(rm_find_memreg(pt_boot, &mr, 0x034FE0000ULL,
                0x034FE0000ULL));
            BRD_ERR(rm_assign_memreg(pt_boot, pt_m4_0, mr));

            /* Move partition to be owned by SC */
            BRD_ERR(rm_set_parent(pt_boot, pt_m4_0, SC_PT));

            /* Check if booting with the no_ap flag set */
            if (no_ap != SC_FALSE)
            {
                /* Move boot to be owned by M4 0 for Android Automotive */
                BRD_ERR(rm_set_parent(SC_PT, pt_boot, pt_m4_0));
            }
        }

        /* Create M4 1 partition */
        if (rm_is_resource_avail(SC_R_M4_1_PID0) != SC_FALSE)
        {
            sc_rm_mr_t mr;

            /* List of resources */
            static const sc_rsrc_t rsrc_list[9U] =
            {
                SC_R_IRQSTR_M4_1,
                SC_R_UART_2,
                SC_R_MU_6B,
                SC_R_MU_7B,
                SC_R_MU_9B,
                SC_R_GPT_3,
                RM_RANGE(SC_R_CAN_0, SC_R_CAN_2),
                SC_R_FSPI_0
            };

            /* List of pads */
            static const sc_pad_t pad_list[8U] =
            {
                RM_RANGE(SC_P_M41_I2C0_SCL, SC_P_M41_GPIO0_01),
                RM_RANGE(SC_P_UART0_CTS_B, SC_P_UART0_RTS_B),
                RM_RANGE(SC_P_FLEXCAN0_RX, SC_P_FLEXCAN2_TX),
                RM_RANGE(SC_P_QSPI0A_DATA0, SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI0)
            };

            /* List of memory regions */
            static const sc_rm_mem_list_t mem_list[2U] =
            {
                {0x088800000ULL, 0x08FFFFFFFULL},
                {0x008181000ULL, 0x008280FFFULL}
            };

            /* Create partition */
            BRD_ERR(rm_partition_create(pt_boot, &pt_m4_1, SC_FALSE,
                SC_TRUE, SC_FALSE, SC_TRUE, SC_FALSE, SC_R_M4_1_PID0,
                rsrc_list, ARRAY_SIZE(rsrc_list),
                pad_list, ARRAY_SIZE(pad_list),
                mem_list, ARRAY_SIZE(mem_list)));

            /* Name partition for debug */
            PARTITION_NAME(pt_m4_1, "MCU1");
            
            /* Allow M4 1 to use SYSTEM (not production!) */
            BRD_ERR(rm_set_peripheral_permissions(SC_PT, SC_R_SYSTEM,
                pt_m4_1, SC_RM_PERM_SEC_RW));

            /* Move M4 1 TCM */
            BRD_ERR(rm_find_memreg(pt_boot, &mr, 0x038FE0000ULL,
                0x038FE0000ULL));
            BRD_ERR(rm_assign_memreg(pt_boot, pt_m4_1, mr));

            /* Move partition to be owned by SC */
            BRD_ERR(rm_set_parent(pt_boot, pt_m4_1, SC_PT));
        }

        /* Create partition for shared/hidden resources */
        {
            sc_rm_pt_t pt;
            sc_rm_mr_t mr;

            /* List of resources */
            static const sc_rsrc_t rsrc_list[4U] =
            {
                RM_RANGE(SC_R_M4_0_PID1, SC_R_M4_0_PID4),
                RM_RANGE(SC_R_M4_1_PID1, SC_R_M4_1_PID4)
            };

            /* List of memory regions */
            static const sc_rm_mem_list_t mem_list[1U] =
            {
                {0x090000000ULL, 0x091FFFFFFULL}
            };

            /* Create shared partition */
            BRD_ERR(rm_partition_create(SC_PT, &pt, SC_FALSE, SC_TRUE,
                SC_FALSE, SC_FALSE, SC_FALSE, SC_NUM_RESOURCE,
                rsrc_list, ARRAY_SIZE(rsrc_list), NULL, 0U,
                mem_list, ARRAY_SIZE(mem_list)));

            /* Name partition for debug */
            PARTITION_NAME(pt, "Shared");
            
            /* Share memory space */
            BRD_ERR(rm_find_memreg(SC_PT, &mr,
                mem_list[0U].addr_start, mem_list[0U].addr_start));
            BRD_ERR(rm_set_memreg_permissions(pt, mr, pt_boot,
                SC_RM_PERM_FULL));
            if (pt_m4_0 != SC_RM_NUM_PARTITION)
            {
                BRD_ERR(rm_set_memreg_permissions(pt, mr, pt_m4_0,
                    SC_RM_PERM_FULL));
            }
            if (pt_m4_1 != SC_RM_NUM_PARTITION)
            {
                BRD_ERR(rm_set_memreg_permissions(pt, mr, pt_m4_1,
                    SC_RM_PERM_FULL));
            }
        }

        /* Allow all to access the SEMA42s */
        BRD_ERR(rm_set_peripheral_permissions(SC_PT, SC_R_M4_0_SEMA42,
            SC_RM_PT_ALL, SC_RM_PERM_FULL));
        BRD_ERR(rm_set_peripheral_permissions(SC_PT, SC_R_M4_1_SEMA42,
            SC_RM_PT_ALL, SC_RM_PERM_FULL));

        #ifdef BOARD_RM_DUMP
            rm_dump(pt_boot);
        #endif
    }
}

/*--------------------------------------------------------------------------*/
/* Early CPU query                                                          */
/*--------------------------------------------------------------------------*/
sc_bool_t board_early_cpu(sc_rsrc_t cpu)
{
    sc_bool_t rtn = SC_FALSE;

    if ((cpu == SC_R_M4_0_PID0) || (cpu == SC_R_M4_1_PID0))
    {
        rtn = SC_TRUE;
    }

    return rtn;
}

/*--------------------------------------------------------------------------*/
/* Transition external board-level SoC power domain                         */
/*--------------------------------------------------------------------------*/
void board_set_power_mode(sc_sub_t ss, uint8_t pd,
    sc_pm_power_mode_t from_mode, sc_pm_power_mode_t to_mode)
{
    pmic_id_t pmic_id[2] = {0U, 0U};
    uint32_t pmic_reg[2] = {0U, 0U};
    uint8_t num_regs = 0U;

    board_print(3, "board_set_power_mode(%s, %d, %d, %d)\n", snames[ss],
        pd, from_mode, to_mode);

    board_get_pmic_info(ss, pmic_id, pmic_reg, &num_regs);

    /* Check for PMIC */
    if (pmic_ver.device_id != 0U)
    {
        sc_err_t err = SC_ERR_NONE;

        /* Flip switch */
        if (to_mode > SC_PM_PW_MODE_OFF)
        {
            uint8_t idx = 0U;

            while (idx < num_regs)
            {
                BRD_ERR(PMIC_SET_MODE(pmic_id[idx], pmic_reg[idx],
                    SW_RUN_PWM | SW_STBY_PWM));
                idx++;
            }
            SystemTimeDelay(PMIC_MAX_RAMP);
        }
        else
        {
            uint8_t idx = 0U;

            while (idx < num_regs)
            {
                BRD_ERR(PMIC_SET_MODE(pmic_id[idx], pmic_reg[idx],
                    SW_RUN_OFF));
                idx++;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
/* Set the voltage for the given SS.                                        */
/*--------------------------------------------------------------------------*/
sc_err_t board_set_voltage(sc_sub_t ss, uint32_t new_volt, uint32_t old_volt)
{
    sc_err_t err = SC_ERR_NONE;
    pmic_id_t pmic_id[2] = {0U, 0U};
    uint32_t pmic_reg[2] = {0U, 0U};
    uint8_t num_regs = 0U;

    board_print(3, "board_set_voltage(%s, %u, %u)\n", snames[ss], new_volt,
        old_volt);

    board_get_pmic_info(ss, pmic_id, pmic_reg, &num_regs);

    /* Check for PMIC */
    if (pmic_ver.device_id == 0U)
    {
        err = SC_ERR_NOTFOUND;
    }
    else
    {
        uint8_t idx = 0U;

        while (idx < num_regs)
        {
            BRD_ERR(PMIC_SET_VOLTAGE(pmic_id[idx], pmic_reg[idx], new_volt,
                REG_RUN_MODE));
            idx++;
        }
        if ((old_volt != 0U) && (new_volt > old_volt))
        {
            /* PMIC_MAX_RAMP_RATE is in nano Volts. */
            uint32_t ramp_time = ((new_volt - old_volt) * 1000U)
                / PMIC_MAX_RAMP_RATE;
            SystemTimeDelay(ramp_time + 1U);
        }
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* Set board power supplies when enter/exit low-power mode                  */
/*--------------------------------------------------------------------------*/
void board_lpm(sc_pm_power_mode_t mode)
{
    static uint32_t vdd_memc_mode = 0U;

    if (mode == SC_PM_PW_MODE_STBY)
    {
        /*
         * System standby (KS1) entry allows VDD_MEMC to be gated off.
         * Save current mode and switch off supply.
         */
        if (PMIC_GET_MODE(PMIC_1_ADDR, PF8100_SW5, &vdd_memc_mode)
            == SC_ERR_NONE)
        {
            (void) PMIC_SET_MODE(PMIC_1_ADDR, PF8100_SW5, SW_STBY_OFF
                | SW_RUN_OFF);
        }
    }
    else if (mode == SC_PM_PW_MODE_ON)
    {
        /*
         * System standby (KS1) exit should switch on VDD_MEMC.  Restore
         * previous mode saved during KS1 entry.
         */
        if (vdd_memc_mode != 0U)
        {
            (void) PMIC_SET_MODE(PMIC_1_ADDR, PF8100_SW5, vdd_memc_mode);
        }
    }
    else
    {
        ; /* Intentional empty else */
    }
}

/*--------------------------------------------------------------------------*/
/* Reset a board resource                                                   */
/*--------------------------------------------------------------------------*/
void board_rsrc_reset(sc_rm_idx_t idx, sc_rm_idx_t rsrc_idx, sc_rm_pt_t pt)
{
}

/*--------------------------------------------------------------------------*/
/* Transition external board-level supply for board component               */
/*--------------------------------------------------------------------------*/
void board_trans_resource_power(sc_rm_idx_t idx, sc_rm_idx_t rsrc_idx,
    sc_pm_power_mode_t from_mode, sc_pm_power_mode_t to_mode)
{
    board_print(3, "board_trans_resource_power(%d, %s, %u, %u)\n", idx,
        rnames[rsrc_idx], from_mode, to_mode);

    /* Init PMIC */
    pmic_init();

    /* Process resource */
    if (pmic_ver.device_id != 0U)
    {
        sc_err_t err = SC_ERR_NONE;

        switch (idx)
        {
            case BRD_R_BOARD_R0 : /* Apalis ethernet PHY power rail */
                if (to_mode > SC_PM_PW_MODE_OFF)
                {
                    /* KSZ 9031 power-up sequence */
                    BRD_ERR(PMIC_SET_VOLTAGE(PMIC_0_ADDR, PF8100_LDO4,
                        3300, REG_RUN_MODE));
                    BRD_ERR(PMIC_SET_MODE(PMIC_0_ADDR, PF8100_LDO4,
                        RUN_EN_STBY_EN));

                    BRD_ERR(PMIC_SET_VOLTAGE(PMIC_1_ADDR, PF8100_SW7,
                        1200, REG_RUN_MODE));
                    BRD_ERR(PMIC_SET_MODE(PMIC_1_ADDR, PF8100_SW7,
                        RUN_EN_STBY_EN));
                }
                else
                {
                    /* KSZ 9031 power-down sequence */
                    BRD_ERR(PMIC_SET_MODE(PMIC_1_ADDR, PF8100_SW7,
                        RUN_OFF_STBY_OFF));

                    BRD_ERR(PMIC_SET_MODE(PMIC_0_ADDR, PF8100_LDO4,
                        RUN_OFF_STBY_OFF));
                }
                break;
            case BRD_R_BOARD_R1 : /* Apalis external RGMII interface in 3.3V */
                if (to_mode > SC_PM_PW_MODE_OFF)
                {
                    /*
                     * We set 5V here so the LDO1 runs in load switch (LS) mode
                     * so the output voltage follows exactly the input voltage
                     * that is the module supply voltage, 3.3V.
                     * This is needed because this LDO is fused to run in normal
                     * mode and not like PMIC_0_ADDR, PF8100_LDO4 in LS mode
                     */
                    BRD_ERR(PMIC_SET_VOLTAGE(PMIC_1_ADDR, PF8100_LDO1,
                        5000, REG_RUN_MODE));
                    BRD_ERR(PMIC_SET_MODE(PMIC_1_ADDR, PF8100_LDO1,
                        RUN_EN_STBY_EN));
                }
                else
                {
                    BRD_ERR(PMIC_SET_MODE(PMIC_1_ADDR, PF8100_LDO1,
                        RUN_OFF_STBY_OFF));
                }
                break;
            case BRD_R_BOARD_R2 : /* Apalis external RGMII interface in 1.8V */
                if (to_mode > SC_PM_PW_MODE_OFF)
                {
                    BRD_ERR(PMIC_SET_VOLTAGE(PMIC_1_ADDR, PF8100_LDO1,
                        1800, REG_RUN_MODE));
                    BRD_ERR(PMIC_SET_MODE(PMIC_1_ADDR, PF8100_LDO1,
                        RUN_EN_STBY_EN));
                }
                else
                {
                    BRD_ERR(PMIC_SET_MODE(PMIC_1_ADDR, PF8100_LDO1,
                        RUN_OFF_STBY_OFF));
                }
                break;
            case BRD_R_BOARD_R3 : /* Apalis external RGMII interface in 2.5V */
                if (to_mode > SC_PM_PW_MODE_OFF)
                {
                    BRD_ERR(PMIC_SET_VOLTAGE(PMIC_1_ADDR, PF8100_LDO1,
                        2500, REG_RUN_MODE));
                    BRD_ERR(PMIC_SET_MODE(PMIC_1_ADDR, PF8100_LDO1,
                        RUN_EN_STBY_EN));
                }
                else
                {
                    BRD_ERR(PMIC_SET_MODE(PMIC_1_ADDR, PF8100_LDO1,
                        RUN_OFF_STBY_OFF));
                }
                break;
            default :
                ; /* Intentional empty default */
                break;
        }
    }
}

/*--------------------------------------------------------------------------*/
/* Set board power mode                                                     */
/*--------------------------------------------------------------------------*/
sc_err_t board_power(sc_pm_power_mode_t mode)
{
    sc_err_t err = SC_ERR_NONE;

    if (mode == SC_PM_PW_MODE_OFF)
    {
        /* Request power off */
        SNVS_PowerOff();
        err = snvs_err;

        /* Loop forever */
        while(err == SC_ERR_NONE)
        {
            ; /* Intentional empty while */
        }
    }
    else
    {
        err = SC_ERR_PARM;
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* Reset board                                                              */
/*--------------------------------------------------------------------------*/
sc_err_t board_reset(sc_pm_reset_type_t type, sc_pm_reset_reason_t reason,
    sc_rm_pt_t pt)
{
    if (type == SC_PM_RESET_TYPE_BOARD)
    {
        /* Request PMIC do a board reset */
    }
    else if (type == SC_PM_RESET_TYPE_COLD)
    {
        /* Request PMIC do a cold reset */
    }
    else
    {
        ; /* Intentional empty else */
    }

    #ifdef DEBUG
        /* Dump out caller of reset request */
        always_print("Board reset (%u, caller = 0x%08X)\n", reason,
            __builtin_return_address(0));
    #endif
    #ifdef ALT_DEBUG_UART
        /* Invoke LPUART deinit to drain TX buffers if a warm reset follows */
        LPUART_Deinit(LPUART_DEBUG);
    #endif

    /* Request a warm reset */
    soc_set_reset_info(reason, pt);
    NVIC_SystemReset();

    return SC_ERR_UNAVAILABLE;
}

/*--------------------------------------------------------------------------*/
/* Handle CPU reset event                                                   */
/*--------------------------------------------------------------------------*/
void board_cpu_reset(sc_rsrc_t resource, board_cpu_rst_ev_t reset_event,
    sc_rm_pt_t pt)
{
    /* Note:  Production code should decide the response for each type
     *        of reset event.  Options include allowing the SCFW to
     *        reset the CPU or forcing a full system reset.  Additionally,
     *        the number of reset attempts can be tracked to determine the
     *        reset response.
     */

    /* Check for M4 reset event */
    if ((resource == SC_R_M4_0_PID0) || (resource == SC_R_M4_1_PID0))
    {
        always_print("CM4 reset event (rsrc = %d, event = %d)\n", resource,
            reset_event);

        /* Treat lockups or parity/ECC reset events as board faults */
        if ((reset_event == BOARD_CPU_RESET_LOCKUP) ||
            (reset_event == BOARD_CPU_RESET_MEM_ERR))
        {
            board_fault(SC_FALSE, BOARD_BFAULT_CPU, pt);
        }
    }

    /* Returning from this function will result in an attempt reset the
       partition or board depending on the event and wdog action. */
}

/*--------------------------------------------------------------------------*/
/* Trap partition reboot                                                    */
/*--------------------------------------------------------------------------*/
void board_reboot_part(sc_rm_pt_t pt, sc_pm_reset_type_t *type,
    sc_pm_reset_reason_t *reason, sc_pm_power_mode_t *mode,
    uint32_t *mask)
{
    /* Code can modify or log the parameters. Can also take another action like
     * reset the board. After return from this function, the partition will be
     * rebooted.
     */
    *mask = 0UL;
}

/*--------------------------------------------------------------------------*/
/* Trap partition reboot continue                                           */
/*--------------------------------------------------------------------------*/
void board_reboot_part_cont(sc_rm_pt_t pt, sc_rsrc_t *boot_cpu,
    sc_rsrc_t *boot_mu, sc_rsrc_t *boot_dev, sc_faddr_t *boot_addr)
{
    /* Code can modify boot parameters on a reboot. Called after partition
     * is powered off but before it is powered back on and started.
     */
}

/*--------------------------------------------------------------------------*/
/* Return partition reboot timeout action                                   */
/*--------------------------------------------------------------------------*/
board_reboot_to_t board_reboot_timeout(sc_rm_pt_t pt)
{
    /* Return the action to take if a partition reboot requires continue
     * ack for others and does not happen before timeout */
    return BOARD_REBOOT_TO_FORCE;
}

/*--------------------------------------------------------------------------*/
/* Handle panic temp alarm                                                  */
/*--------------------------------------------------------------------------*/
void board_panic(sc_dsc_t dsc)
{
    /* See Porting Guide for more info on panic alarms */
    #ifdef DEBUG
        error_print("Panic temp (dsc=%d)\n", dsc);
    #endif

    (void) board_reset(SC_PM_RESET_TYPE_BOARD, SC_PM_RESET_REASON_TEMP,
        SC_PT);
}

/*--------------------------------------------------------------------------*/
/* Handle fault or return from main()                                       */
/*--------------------------------------------------------------------------*/
void board_fault(sc_bool_t restarted, sc_bfault_t reason,
    sc_rm_pt_t pt)
{
    /* Note, delete the DEBUG case if fault behavior should be like
       typical production build even if DEBUG defined */

    #ifdef DEBUG
        /* Disable the watchdog */
        board_wdog_disable(SC_FALSE);

        board_print(1, "board fault(%u, %u, %u)\n", restarted, reason, pt);

        /* Stop so developer can see WDOG occurred */
        HALT;
    #else
        /* Was this called to report a previous WDOG restart? */
        if (restarted == SC_FALSE)
        {
            /* Fault just occurred, need to reset */
            (void) board_reset(SC_PM_RESET_TYPE_BOARD,
                SC_PM_RESET_REASON_SCFW_FAULT, pt);

            /* Wait for reset */
            HALT;
        }
        /* Issue was before restart so just return */
    #endif
}

/*--------------------------------------------------------------------------*/
/* Handle SECO FW fault                                                     */
/*--------------------------------------------------------------------------*/
void board_sec_fault(uint8_t abort_module, uint8_t abort_line,
    sc_sfault_t reason)
{
    #ifdef DEBUG
        error_print("SECO Abort (mod %d, ln %d)\n", abort_module,
            abort_line);
    #else
        board_fault(SC_FALSE, BOARD_BFAULT_SEC_FAIL, SECO_PT);
    #endif
}

/*--------------------------------------------------------------------------*/
/* Handle SECO/SNVS security violation                                      */
/*--------------------------------------------------------------------------*/
void board_security_violation(void)
{
    always_print("SNVS security violation\n");
}

/*--------------------------------------------------------------------------*/
/* Get the status of the ON/OFF button                                      */
/*--------------------------------------------------------------------------*/
sc_bool_t board_get_button_status(void)
{
    return SNVS_GetButtonStatus();
}

/*--------------------------------------------------------------------------*/
/* Set control value                                                        */
/*--------------------------------------------------------------------------*/
sc_err_t board_set_control(sc_rsrc_t resource, sc_rm_idx_t idx,
    sc_rm_idx_t rsrc_idx, uint32_t ctrl, uint32_t val)
{
    sc_err_t err = SC_ERR_NONE;

    board_print(3,
        "board_set_control(%s, %u, %u)\n", rnames[rsrc_idx], ctrl, val);

    /* Init PMIC */
    pmic_init();

    /* Check if PMIC available */
    ASRT_ERR(pmic_ver.device_id != 0U, SC_ERR_NOTFOUND);

    if (err == SC_ERR_NONE)
    {
        /* Process control */
        switch (resource)
        {
            case SC_R_PMIC_0 :
                if (ctrl == SC_C_TEMP_HI)
                {
                    temp_alarm0 =
                        SET_PMIC_TEMP_ALARM(PMIC_0_ADDR, val);
                }
                else
                {
                    err = SC_ERR_PARM;
                }
                break;
            case SC_R_PMIC_1 :
                if (ctrl == SC_C_TEMP_HI)
                {
                    temp_alarm1 =
                        SET_PMIC_TEMP_ALARM(PMIC_1_ADDR, val);
                }
                else
                {
                    err = SC_ERR_PARM;
                }
                break;
            default :
                err = SC_ERR_PARM;
                break;
        }
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* Get control value                                                        */
/*--------------------------------------------------------------------------*/
sc_err_t board_get_control(sc_rsrc_t resource, sc_rm_idx_t idx,
    sc_rm_idx_t rsrc_idx, uint32_t ctrl, uint32_t *val)
{
    sc_err_t err = SC_ERR_NONE;

    board_print(3,
        "board_get_control(%s, %u)\n", rnames[rsrc_idx], ctrl);

    /* Init PMIC */
    pmic_init();

    /* Check if PMIC available */
    ASRT_ERR(pmic_ver.device_id != 0U, SC_ERR_NOTFOUND);

    if (err == SC_ERR_NONE)
    {
        /* Process control */
        switch (resource)
        {
            /* PMIC 0 */
            case SC_R_PMIC_0 :
                if (ctrl == SC_C_TEMP)
                {
                    *val = GET_PMIC_TEMP(PMIC_0_ADDR);
                }
                else if (ctrl == SC_C_TEMP_HI)
                {
                    *val = temp_alarm0;
                }
                else if (ctrl == SC_C_ID)
                {
                    pmic_version_t v = GET_PMIC_VERSION(PMIC_0_ADDR);

                    *val = (U32(v.device_id) << 8U) | U32(v.si_rev);
                }
                else
                {
                    err = SC_ERR_PARM;
                }
                break;
            /* PMIC 1 */
            case SC_R_PMIC_1 :
                if (ctrl == SC_C_TEMP)
                {
                    *val = GET_PMIC_TEMP(PMIC_1_ADDR);
                }
                else if (ctrl == SC_C_TEMP_HI)
                {
                    *val = temp_alarm1;
                }
                else if (ctrl == SC_C_ID)
                {
                    pmic_version_t v = GET_PMIC_VERSION(PMIC_1_ADDR);

                    *val = (U32(v.device_id) << 8U) | U32(v.si_rev);
                }
                else
                {
                    err = SC_ERR_PARM;
                }
                break;
            default :
                err = SC_ERR_PARM;
                break;
        }
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* PMIC Interrupt (INTB) handler                                            */
/*--------------------------------------------------------------------------*/
void PMIC_IRQHandler(void)
{
    /* Temp alarm from PMIC 1 */
    if (PMIC_IRQ_SERVICE(PMIC_1_ADDR) != SC_FALSE)
    {
        /* Trigger client interrupt */
        ss_irq_trigger(SC_IRQ_GROUP_TEMP, SC_IRQ_TEMP_PMIC1_HIGH,
            SC_PT_ALL);
    }
    /* Temp alarm from PMIC 0 */
    if (PMIC_IRQ_SERVICE(PMIC_0_ADDR) != SC_FALSE)
    {
        /* Trigger client interrupt */
        ss_irq_trigger(SC_IRQ_GROUP_TEMP, SC_IRQ_TEMP_PMIC0_HIGH,
            SC_PT_ALL);
    }

    /* Clear IRQ */
    NVIC_ClearPendingIRQ(PMIC_INT_IRQn);
}

/*--------------------------------------------------------------------------*/
/* Button Handler                                                           */
/*--------------------------------------------------------------------------*/
void SNVS_Button_IRQHandler(void)
{
    SNVS_ClearButtonIRQ();

    ss_irq_trigger(SC_IRQ_GROUP_WAKE, SC_IRQ_BUTTON, SC_PT_ALL);
}

/*==========================================================================*/

/*--------------------------------------------------------------------------*/
/* Init the PMIC interface                                                  */
/*--------------------------------------------------------------------------*/
static void pmic_init(void)
{
    #ifndef EMUL
        static sc_bool_t pmic_checked = SC_FALSE;
        static lpi2c_master_config_t lpi2c_masterConfig;
        sc_pm_clock_rate_t rate = SC_24MHZ;

        /* See if we already checked for the PMIC */
        if (pmic_checked == SC_FALSE)
        {
            sc_err_t err = SC_ERR_NONE;

            pmic_checked = SC_TRUE;

            /* Initialize the PMIC */
            board_print(3, "Start PMIC init\n");

            /* Power up the I2C and configure clocks */
            pm_force_resource_power_mode_v(SC_R_SC_I2C,
                SC_PM_PW_MODE_ON);
            (void) pm_set_clock_rate(SC_PT, SC_R_SC_I2C,
                SC_PM_CLK_PER, &rate);
            pm_force_clock_enable(SC_R_SC_I2C, SC_PM_CLK_PER,
                SC_TRUE);

            /* Initialize the pads used to communicate with the PMIC */
            pad_force_mux(SC_P_PMIC_I2C_SDA, 0,
                SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF);
            (void) pad_set_gp_28fdsoi(SC_PT, SC_P_PMIC_I2C_SDA,
                SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_PU);
            pad_force_mux(SC_P_PMIC_I2C_SCL, 0,
                SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF);
            (void) pad_set_gp_28fdsoi(SC_PT, SC_P_PMIC_I2C_SCL,
                SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_PU);

            /* Initialize the PMIC interrupt pad */
            pad_force_mux(SC_P_PMIC_INT_B, 0,
                SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);
            (void) pad_set_gp_28fdsoi(SC_PT, SC_P_PMIC_INT_B,
                SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_PU);

            /* Initialize the I2C used to communicate with the PMIC */
            LPI2C_MasterGetDefaultConfig(&lpi2c_masterConfig);

            /* MEK board spec is for 1M baud for PMIC I2C bus */
            lpi2c_masterConfig.baudRate_Hz = 1000000U;
            lpi2c_masterConfig.sdaGlitchFilterWidth_ns = 100U;
            lpi2c_masterConfig.sclGlitchFilterWidth_ns = 100U;
            LPI2C_MasterInit(LPI2C_PMIC, &lpi2c_masterConfig, SC_24MHZ);

            /* Delay to allow I2C to settle */
            SystemTimeDelay(2U);

            pmic_ver = GET_PMIC_VERSION(PMIC_0_ADDR);
            temp_alarm0 = SET_PMIC_TEMP_ALARM(PMIC_0_ADDR, PMIC_TEMP_MAX);
            temp_alarm1 = SET_PMIC_TEMP_ALARM(PMIC_1_ADDR, PMIC_TEMP_MAX);

            err |= pmic_ignore_current_limit(PMIC_0_ADDR);
            err |= pmic_ignore_current_limit(PMIC_1_ADDR);

            /* Adjust startup timing */
            err |= pmic_update_timing(PMIC_0_ADDR);
            err |= pmic_update_timing(PMIC_1_ADDR);

            err |= pmic_match_otp(PMIC_0_ADDR, pmic_ver);
            err |= pmic_match_otp(PMIC_1_ADDR, pmic_ver);

            /* Enable WDI detection in Standby */
            err |= pf8100_pmic_wdog_enable(PMIC_0_ADDR, SC_FALSE, SC_FALSE, SC_TRUE);
            err |= pf8100_pmic_wdog_enable(PMIC_1_ADDR, SC_FALSE, SC_FALSE, SC_TRUE);

            if (err != SC_ERR_NONE)
            {
                /* Loop so WDOG will expire */
                HALT;
            }

            BRD_ERR(PMIC_SET_MODE(PMIC_0_ADDR, PF8100_SW1, SW_RUN_PWM
                | SW_STBY_PWM));
            BRD_ERR(PMIC_SET_MODE(PMIC_0_ADDR, PF8100_SW2, SW_RUN_PWM
                | SW_STBY_PWM));
            BRD_ERR(PMIC_SET_MODE(PMIC_0_ADDR, PF8100_SW7, SW_RUN_PWM
                | SW_STBY_PWM));

            /* Configure STBY voltage for SW1 (VDD_MAIN) */
            if (board_parameter(BOARD_PARM_KS1_RETENTION)
                == BOARD_PARM_KS1_RETENTION_ENABLE)
            {
                BRD_ERR(PMIC_SET_VOLTAGE(PMIC_0_ADDR, PF8100_SW1, 800,
                    REG_STBY_MODE));
            }

            /* Enable PMIC IRQ at NVIC level */
            NVIC_EnableIRQ(PMIC_INT_IRQn);

            board_print(3, "Finished  PMIC init\n\n");
        }
    #endif
}

/*--------------------------------------------------------------------------*/
/* Bypass current limit for PF8100                                          */
/*--------------------------------------------------------------------------*/
static sc_err_t pmic_ignore_current_limit(uint8_t address)
{
    sc_err_t err = SC_ERR_NONE;
    uint8_t idx;
    uint8_t val = 0U;
    static const pf8100_vregs_t switchers[11] =
    {
        PF8100_SW1,
        PF8100_SW2,
        PF8100_SW3,
        PF8100_SW4,
        PF8100_SW5,
        PF8100_SW6,
        PF8100_SW7,
        PF8100_LDO1,
        PF8100_LDO2,
        PF8100_LDO3,
        PF8100_LDO4
    };

    /* Loop over supplies */
    for (idx = 0U; idx < 11U; idx++)
    {
        /* Read the config register first */
        err = PMIC_REGISTER_ACCESS(address, switchers[idx], SC_FALSE,
            &val);

        /* Check for error? */
        if (err == SC_ERR_NONE)
        {
            val |= 0x20U; /* set xx_ILIM_BYPASS */

            /*
             * Enable the UV_BYPASS and OV_BYPASS for all LDOs.
             * The SDHC LDO2 constantly switches between 3.3V and 1.8V and
             * the counters are incorrectly triggered.
             * Also any other LDOs (like LDO1 on the board) that is
             * enabled/disabled during suspend/resume can trigger the counters.
             */
             if ((switchers[idx] == PF8100_LDO1) ||
                 (switchers[idx] == PF8100_LDO2) ||
                 (switchers[idx] == PF8100_LDO3) ||
                 (switchers[idx] == PF8100_LDO4))
            {
                val |= 0xC0U;
            }

            /* Write the config register */
            err = PMIC_REGISTER_ACCESS(address, switchers[idx], SC_TRUE,
                &val);
        }

        /* Stop loop if there is an error */
        if (err != SC_ERR_NONE)
        {
            break;
        }
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* Update power timing for PF8100                                           */
/*--------------------------------------------------------------------------*/
static sc_err_t pmic_update_timing(uint8_t address)
{
    sc_err_t err = SC_ERR_NONE;

    /*
     * Add 60ms stable time for power down for:
     *   PMIC 1 : LDO2, SW6, SW7
     *   PMIC 2 : LDO2, SW5, SW6, SW7
     * on i.mx8QM-mek
     * board, otherwise system may reboot fail by mmc not power off
     * clean
     */
    if (address == PMIC_0_ADDR)
    {
        uint8_t val = 0xED;

        /* Update for PMIC 0 */
        err |= PMIC_REGISTER_ACCESS(address, 0x8D, SC_TRUE, &val);
        err |= PMIC_REGISTER_ACCESS(address, 0x77, SC_TRUE, &val);
        err |= PMIC_REGISTER_ACCESS(address, 0x7F, SC_TRUE, &val);
        val = 0x29;
        err |= PMIC_REGISTER_ACCESS(address, 0x3C, SC_TRUE, &val);
    }
    else if (address == PMIC_1_ADDR)
    {
        uint8_t val = 0xED;

        /* Update for PMIC 1 */
        err |= PMIC_REGISTER_ACCESS(address, 0x8D, SC_TRUE, &val);
        err |= PMIC_REGISTER_ACCESS(address, 0x6F, SC_TRUE, &val);
        err |= PMIC_REGISTER_ACCESS(address, 0x77, SC_TRUE, &val);
        err |= PMIC_REGISTER_ACCESS(address, 0x7F, SC_TRUE, &val);
        val = 0x29;
        err |= PMIC_REGISTER_ACCESS(address, 0x3C, SC_TRUE, &val);
    }
    else
    {
        /* Return error */
        err = SC_ERR_PARM;
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* Check correct version of OTP for PF8100                                  */
/*--------------------------------------------------------------------------*/
static sc_err_t pmic_match_otp(uint8_t address, pmic_version_t ver)
{
    uint8_t reg_value = 0U;
    uint16_t prog_id, match;
    sc_err_t err = SC_ERR_NONE;

    if (address == PMIC_0_ADDR)
    {
        match = EP_PROG_ID;
    }
    else
    {
        match = EQ_PROG_ID;
    }

    /* Read Prog ID */
    err |= PMIC_REGISTER_ACCESS(address, 0x2, SC_FALSE, &reg_value);
    prog_id = (((uint16_t)reg_value << 4U) & 0x0F00U);
    err |= PMIC_REGISTER_ACCESS(address, 0x3U, SC_FALSE, &reg_value);
    prog_id |= reg_value;

    /* test against calibration fusing */
    if (OTP_PROG_FUSE_VERSION_1_7V_CAL != 0U)
    {
        if (ver.si_rev >= PF8100_C1_SI_REV)
        {
            /* if C1 PMIC test for correct OTP */
            if(prog_id != match){/* allow only 1.7v OTP */
                error_print("PMIC INVALID!\n");
            }
        }
        else
        {
            error_print("PMIC INVALID!\n");
        }
    }
    else
    {
        if (ver.si_rev >= PF8100_C1_SI_REV)
        {
            if(prog_id == match){/* prohibit only 1.7V OTP */
                error_print("PMIC INVALID!\n");
            }
        }
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* Get the pmic ids and switchers connected to SS.                          */
/*--------------------------------------------------------------------------*/
static void board_get_pmic_info(sc_sub_t ss,pmic_id_t *pmic_id,
    uint32_t *pmic_reg, uint8_t *num_regs)
{
    /* Map SS/PD to PMIC switch */
    switch (ss)
    {
        case SC_SUBSYS_A53 :
            pmic_init();
            {/* PF8100_dual Card */
                pmic_id[0] = PMIC_0_ADDR;
                pmic_reg[0] = PF8100_SW5;
                *num_regs = 1U;
            }
            break;
        case SC_SUBSYS_A72 :
            pmic_init();
            {/* PF8100_dual Card */
                pmic_id[0] = PMIC_0_ADDR;
                pmic_reg[0] = PF8100_SW3;
                pmic_id[1] = PMIC_0_ADDR;
                pmic_reg[1] = PF8100_SW4;
                *num_regs = 2U;
            }
            break;
        case SC_SUBSYS_GPU_0 :
            pmic_init();
            {/* PF8100_dual Card */
                pmic_id[0] = PMIC_1_ADDR;
                pmic_reg[0] = PF8100_SW1;
                pmic_id[1] = PMIC_1_ADDR;
                pmic_reg[1] = PF8100_SW2;
                *num_regs = 2U;
            }
            break;
        case SC_SUBSYS_GPU_1 :
            pmic_init();
            {/* PF8100_dual Card */
                pmic_id[0] = PMIC_1_ADDR;
                pmic_reg[0] = PF8100_SW3;
                pmic_id[1] = PMIC_1_ADDR;
                pmic_reg[1] = PF8100_SW4;
                *num_regs = 2U;
            }
            break;
        default :
            ; /* Intentional empty default */
            break;
    }
}

/*--------------------------------------------------------------------------*/
/* Board tick                                                               */
/*--------------------------------------------------------------------------*/
void board_tick(uint16_t msec)
{
}

/*--------------------------------------------------------------------------*/
/* Board IOCTL function                                                     */
/*--------------------------------------------------------------------------*/
sc_err_t board_ioctl(sc_rm_pt_t caller_pt, sc_rsrc_t mu, uint32_t *parm1,
    uint32_t *parm2, uint32_t *parm3)
{
    sc_err_t err = SC_ERR_NONE;

#ifdef HAS_TEST
    /* For test_misc */
    if (*parm1 == 0xFFFFFFFEU)
    {
        *parm1 = *parm2 + *parm3;
        *parm2 = mu;
        *parm3 = caller_pt;
    }
    /* For test_wdog */
    else if (*parm1 == 0xFFFFFFFBU)
    {
        HALT;
    }
    else
    {
        err = SC_ERR_PARM;
    }
#endif

    return err;
}

/** @} */

