OBJS += $(OUT)/board/mx8qx_$(B)/board.o \
		$(OUT)/board/board_common.o

ifneq ($(HW), SIMU)
    OBJS += $(OUT)/board/board.o
endif

DCDH += $(SRC)/board/mx8qx_$(B)/dcd/imx8x_ramid1_dcd_933MHz.h \
	$(SRC)/board/mx8qx_$(B)/dcd/imx8x_ramid1_dcd_933MHz_retention.h

RSRC_MD += $(SRC)/board/mx8qx_$(B)/resource.txt

CTRL_MD += $(SRC)/board/mx8qx_$(B)/control.txt

DIRS += $(OUT)/board/mx8qx_$(B)

