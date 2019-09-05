OBJS += $(OUT)/board/mx8qx_$(B)/board.o \
		$(OUT)/board/board_common.o

ifneq ($(HW), SIMU)
    OBJS += $(OUT)/board/board.o
endif

DCDH += $(SRC)/board/mx8qx_$(B)/dcd/imx8dx_dcd_1.2GHz.h \
		$(SRC)/board/mx8qx_$(B)/dcd/imx8qx_dcd_1.2GHz.h \
		$(SRC)/board/mx8qx_$(B)/dcd/imx8dx_dcd_1.2GHz_retention.h \
		$(SRC)/board/mx8qx_$(B)/dcd/imx8qx_dcd_1.2GHz_retention.h

RSRC_MD += $(SRC)/board/mx8qx_$(B)/resource.txt

CTRL_MD += $(SRC)/board/mx8qx_$(B)/control.txt

DIRS += $(OUT)/board/mx8qx_$(B)

