## ###################################################################
##
##     Copyright 2019 NXP
##
##     Redistribution and use in source and binary forms, with or without modification,
##     are permitted provided that the following conditions are met:
##
##     o Redistributions of source code must retain the above copyright notice, this list
##       of conditions and the following disclaimer.
##
##     o Redistributions in binary form must reproduce the above copyright notice, this
##       list of conditions and the following disclaimer in the documentation and/or
##       other materials provided with the distribution.
##
##     o Neither the name of the copyright holder nor the names of its
##       contributors may be used to endorse or promote products derived from this
##       software without specific prior written permission.
##
##     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
##     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
##     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
##     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
##     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
##     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
##     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
##     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
##     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
##
## ###################################################################

OBJS += $(OUT)/board/$(CONFIG)_$(B)/board.o \
		$(OUT)/board/board_common.o

ifneq ($(HW), SIMU)
    OBJS += $(OUT)/board/board.o
endif

ifeq ($(RAMID),1)
    FLAGS += -DRAMID1
else ifeq ($(RAMID),2)
    FLAGS += -DRAMID2
else ifeq ($(RAMID),3)
    FLAGS += -DRAMID3
else
    FLAGS += -DRAMID1 -DRAMID2 -DRAMID3
endif

DCDH += $(SRC)/board/$(CONFIG)_$(B)/dcd/imx8_ramid1_dcd_1.6GHz.h \
	$(SRC)/board/$(CONFIG)_$(B)/dcd/imx8_ramid2_dcd_1.6GHz.h \
	$(SRC)/board/$(CONFIG)_$(B)/dcd/imx8_ramid3_dcd_1.6GHz.h \
	$(SRC)/board/$(CONFIG)_$(B)/dcd/imx8_ramid1_dcd_1.6GHz_retention.h \
	$(SRC)/board/$(CONFIG)_$(B)/dcd/imx8_ramid2_dcd_1.6GHz_retention.h \
	$(SRC)/board/$(CONFIG)_$(B)/dcd/imx8_ramid3_dcd_1.6GHz_retention.h

RSRC_MD += $(SRC)/board/$(CONFIG)_$(B)/resource.txt

CTRL_MD += $(SRC)/board/$(CONFIG)_$(B)/control.txt

DIRS += $(OUT)/board/$(CONFIG)_$(B)

ifeq ($(M),1)
        OBJS += $(OUT)/board/pmic.o
endif

