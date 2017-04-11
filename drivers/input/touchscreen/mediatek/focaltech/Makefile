-include $(srctree)/drivers/misc/mediatek/Makefile.custom

#ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/atmel/
#ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/atmel/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/focaltech/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include


#path := $(if $(filter yes,$(KBUILD_OUTPUT_SUPPORT)),$(srctree)/$(obj),$(obj))
#obj-y := $(patsubst %.c,%.o,$(subst $(path)/,,$(call wildcard2,$(path)/*.c)))
obj-y += focaltech_core.o
obj-y += focaltech_ctl.o
obj-y += focaltech_ex_fun.o
obj-y += focaltech_flash.o
obj-y += focaltech_Gesture.o
obj-y += focaltech_test_main.o
obj-y += focaltech_test_ini.o
obj-y += focaltech_test_global.o
obj-y += focaltech_test_detail_threshold.o
obj-y += focaltech_test_config_ft3c47.o
obj-y += focaltech_test_ft3c47.o
obj-y += focaltech_test_config_ft5x46.o
obj-y += focaltech_test_ft5x46.o
obj-y += focaltech_test_config_ft6x36.o
obj-y += focaltech_test_ft6x36.o
obj-y += focaltech_test_config_ft5822.o
obj-y += focaltech_test_ft5822.o
obj-y += focaltech_test_config_ft8606.o
obj-y += focaltech_test_ft8606.o
obj-y += focaltech_test_config_ft8716.o
obj-y += focaltech_test_ft8716.o
obj-y += focaltech_test_config_ft8736.o
obj-y += focaltech_test_ft8736.o
obj-y += focaltech_ic_table.o
