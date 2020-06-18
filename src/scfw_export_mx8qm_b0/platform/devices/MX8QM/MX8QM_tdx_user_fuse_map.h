/*
** ###################################################################
**     Processors:          MX8QM
**
**     Compilers:           GNU C Compiler
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MX8QM
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
 * User fuses that are used: 276[15:0], 277[15:0], 278[15:0] 279[15:0]
 * - The fuse 276[15:0] and 277[15:0] are representing block 1 (32bit)
 * - The fuse 278[15:0] and 279[15:0] are representing block 2 (32bit)
 * - The second block is reserved and should be used to overwrite block 1
 *   if block 2 does not read zero.
 * - The fuse 276 is read out with 0x114 (just convert from dec to hex)
 *
 *  31                 18                     4         0
 * +---------------------+---------------------+---------+
 * |      PID4 SKU       |    PID4 VERSION     |  RAMID  |
 * +---------------------+---------------------+---------+
 * |     FUSE 276[15:0]      |      FUSE 277[15:0]       |
 * +-------------------------+---------------------------+
 *  15                      0 15                        0
 */

#define OTP_TDX1_BLOCK      ((OTP_GET_FUSE_STATE(0x114U,  0U, 0x0000FFFFU) << 16) | \
                            (OTP_GET_FUSE_STATE(0x115U,  0U, 0x0000FFFFU)))
#define OTP_TDX1_RAMID      OTP_GET_FUSE_STATE(0x115U,  0U, 0x0000000FU)
#define OTP_TDX1_PID4_VERS  ((OTP_GET_FUSE_STATE(0x114U,  0U, 0x00000003U) << 12) | \
                            (OTP_GET_FUSE_STATE(0x115U,  4U, 0x00000FFFU)))
#define OTP_TDX1_PID4_SKU   OTP_GET_FUSE_STATE(0x114U, 2U, 0x00003FFFU)
#define OTP_TDX1_PID8       (OTP_TDX1_PID4_SKU*10000 + OTP_TDX1_PID4_VERS)

#define OTP_TDX2_BLOCK      ((OTP_GET_FUSE_STATE(0x116U,  0U, 0x0000FFFFU) << 16) | \
                            (OTP_GET_FUSE_STATE(0x117U,  0U, 0x0000FFFFU)))
#define OTP_TDX2_RAMID      OTP_GET_FUSE_STATE(0x117U,  0U, 0x0000000FU)
#define OTP_TDX2_PID4_VERS  ((OTP_GET_FUSE_STATE(0x116U,  0U, 0x00000003U) << 12) | \
                            (OTP_GET_FUSE_STATE(0x117U,  4U, 0x00000FFFU)))
#define OTP_TDX2_PID4_SKU   OTP_GET_FUSE_STATE(0x116U, 2U, 0x00003FFFU)
#define OTP_TDX2_PID8       (OTP_TDX2_PID4_SKU*10000 + OTP_TDX2_PID4_VERS)

#endif /* HW_USER_FUSES_H */

