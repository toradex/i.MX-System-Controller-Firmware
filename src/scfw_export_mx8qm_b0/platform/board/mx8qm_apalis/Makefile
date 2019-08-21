OBJS += $(OUT)/board/mx8qm_$(B)/board.o \
		$(OUT)/board/board_common.o

ifneq ($(HW), SIMU)
    OBJS += $(OUT)/board/board.o
endif

DCDH += $(SRC)/board/mx8qm_$(B)/dcd/imx8qm_dcd_1.6GHz.h \
		$(SRC)/board/mx8qm_$(B)/dcd/imx8qp_dcd_1.6GHz.h \
		$(SRC)/board/mx8qm_$(B)/dcd/imx8qm_dcd_1.6GHz_retention.h \
		$(SRC)/board/mx8qm_$(B)/dcd/imx8qp_dcd_1.6GHz_retention.h

RSRC_MD += $(SRC)/board/mx8qm_$(B)/resource.txt

CTRL_MD += $(SRC)/board/mx8qm_$(B)/control.txt

DIRS += $(OUT)/board/mx8qm_$(B)

