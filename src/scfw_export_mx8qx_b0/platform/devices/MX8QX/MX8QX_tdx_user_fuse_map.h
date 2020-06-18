/*
** ###################################################################
**     Processors:          MX8QX
**
**     Compilers:           GNU C Compiler
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MX8QX
**
**     Copyright 2017-2018 NXP
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

#ifndef HW_USER_FUSES_H
#define HW_USER_FUSES_H

/*******************************************************************************
 * Toradex user fuses
 ******************************************************************************/
/*
 * User fuses that are used: 765[31:0] and 766[31:0]
 * - The fuse 765[31:0] is used by Toradex production.
 * - The fuse 766[31:0] is reserved and should be used to overwrite 765[31:0]
 *   if not read zero.
 * - The fuse 765 is read out with 0x1ED (See RM Note Chapter 7.2)
 *
 *  31                 18                     4         0
 * +---------------------+---------------------+---------+
 * |      PID4 SKU       |    PID4 VERSION     |  RAMID  |
 * +---------------------+---------------------+---------+
 */

#define OTP_TDX1_BLOCK          OTP_GET_FUSE_STATE(0x1EDU,  0U, 0xFFFFFFFFU)
#define OTP_TDX1_RAMID          OTP_GET_FUSE_STATE(0x1EDU,  0U, 0x0000000FU)
#define OTP_TDX1_PID4_VERS      OTP_GET_FUSE_STATE(0x1EDU,  4U, 0x00003FFFU)
#define OTP_TDX1_PID4_SKU       OTP_GET_FUSE_STATE(0x1EDU, 18U, 0x00003FFFU)
#define OTP_TDX1_PID8           (OTP_TDX1_PID4_SKU*10000 + OTP_TDX1_PID4_VERS)

#define OTP_TDX2_BLOCK          OTP_GET_FUSE_STATE(0x1EEU,  0U, 0xFFFFFFFFU)
#define OTP_TDX2_RAMID          OTP_GET_FUSE_STATE(0x1EEU,  0U, 0x0000000FU)
#define OTP_TDX2_PID4_VERS      OTP_GET_FUSE_STATE(0x1EEU,  4U, 0x00003FFFU)
#define OTP_TDX2_PID4_SKU       OTP_GET_FUSE_STATE(0x1EEU, 18U, 0x00003FFFU)
#define OTP_TDX2_PID8           (OTP_TDX2_PID4_SKU*10000 + OTP_TDX2_PID4_VERS)

#endif /* HW_USER_FUSES_H */

