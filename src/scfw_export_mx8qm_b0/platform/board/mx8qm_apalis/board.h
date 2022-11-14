/*
** ###################################################################
**
**     Copyright (c) 2016 Freescale Semiconductor, Inc.
**     Copyright 2017-2021 NXP
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
 * Header file used to configure board specific features of the SCFW.
 *
 */
/*==========================================================================*/

#ifndef SC_BOARD_H
#define SC_BOARD_H

/* Includes */
#include "drivers/pmic/fsl_pmic.h"

/* Defines */

/*! Configure PMIC I2C */
#define LPI2C_PMIC              LPI2C_SC

/*! Configure PMIC I2C instance */
#define LPI2C_PMIC_INST         0U

/* PMIC related defines */
#define PMIC                    pf8100
#define PMIC_0_ADDR             0x8U
#define PMIC_1_ADDR             0x9U

#define PF8100_C1_SI_REV        0x31U
#define EP_PROG_ID              0x0417U
#define EQ_PROG_ID              0x0418U

#define PMIC_TEMP_MAX           135U

/* Declare if PMIC transactions will include CRC */
//#define PMIC_CRC
/* Declare if PMIC Secure Writes are enabled */
//#define PMIC_SECURE_WRITE

/*
 * Configure Maximum Delay based on PMIC OTP settings:
 * clock freq = 20MHZ, Regulator-freq = 2.5MHz, SWxDVS Ramp = 0,
 * results in a ramp rate of 7,813mV/us.
 * 1100 mV / 7.813 mV/us => 140.791 us
 */
#define PMIC_MAX_RAMP           141U    /* Max PMIC ramp delay in uS */
#define PMIC_MAX_RAMP_RATE      7813U   /* PMIC voltage ramp (nV) per uS */

/*
 * Resume from KS1 ramps VDD_MAIN 200 mV (800 mV to 1000 mV)
 * PF8100 reg freq = 2.5 MHz, SWxDVS_RAMP = 0 => 7.813 mV/us
 * 200 mV / 7.813 mV/us = 25.6 us ==> 26 us
 *
 */
#define BOARD_KS1_RESUME_USEC   26U
#define BOARD_KS1_RETENTION     BOARD_PARM_KS1_RETENTION_ENABLE
#define BOARD_KS1_ONOFF_WAKE    BOARD_PARM_KS1_ONOFF_WAKE_ENABLE

/* DQS2DQ can be synchronized to the ISI to avoid DDR bus contention.  Define
 * BOARD_DQS2DQ_SYNC to enable synchronization and configure parameters
 * using the BOARD_DQS2DQ defines below.  Note these defines only apply
 * if BD_LPDDR4_INC_DQS2DQ is defined.  BOARD_DQS2DQ_ISI_RSRC and
 * BOARD_DQS2DQ_ISI_REG must be assigned to the same respective ISI
 * channel.  BOARD_DQS2DQ_SYNC_TIME determines the search window for ISI
 * synchronization before firmware will yield to other service requests.
 * Decreasing BOARD_DQS2DQ_SYNC_TIME will lower latency of other service
 * requests when periodic DQS2DQ is active, but will decrease the likelihood
 * of synchronizing to the ISI frame.
 */
#define BOARD_DQS2DQ_SYNC                       /* DQS2DQ sync enable */
#define BOARD_DQS2DQ_ISI_RSRC   SC_R_ISI_CH0    /* DQS2DQ sync ISI resource */
#define BOARD_DQS2DQ_ISI_REG    ISI0            /* DQS2DQ sync ISI registers */
#define BOARD_DQS2DQ_SYNC_TIME  100U            /* DQS2DQ sync usec timeout */

#endif /* SC_BOARD_H */

