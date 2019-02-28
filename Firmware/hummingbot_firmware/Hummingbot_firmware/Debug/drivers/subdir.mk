################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_acmp.c \
../drivers/fsl_adc12.c \
../drivers/fsl_cache.c \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_crc.c \
../drivers/fsl_dac32.c \
../drivers/fsl_dmamux.c \
../drivers/fsl_edma.c \
../drivers/fsl_ewm.c \
../drivers/fsl_flexio.c \
../drivers/fsl_flexio_i2c_master.c \
../drivers/fsl_flexio_spi.c \
../drivers/fsl_flexio_spi_edma.c \
../drivers/fsl_flexio_uart.c \
../drivers/fsl_flexio_uart_edma.c \
../drivers/fsl_ftfx_cache.c \
../drivers/fsl_ftfx_controller.c \
../drivers/fsl_ftfx_flash.c \
../drivers/fsl_ftfx_flexnvm.c \
../drivers/fsl_ftm.c \
../drivers/fsl_gpio.c \
../drivers/fsl_lmem_cache.c \
../drivers/fsl_lpi2c.c \
../drivers/fsl_lpi2c_cmsis.c \
../drivers/fsl_lpi2c_edma.c \
../drivers/fsl_lpit.c \
../drivers/fsl_lpspi.c \
../drivers/fsl_lpspi_cmsis.c \
../drivers/fsl_lpspi_edma.c \
../drivers/fsl_lptmr.c \
../drivers/fsl_lpuart.c \
../drivers/fsl_lpuart_cmsis.c \
../drivers/fsl_lpuart_edma.c \
../drivers/fsl_pdb.c \
../drivers/fsl_pmc.c \
../drivers/fsl_pwt.c \
../drivers/fsl_rcm.c \
../drivers/fsl_rtc.c \
../drivers/fsl_sim.c \
../drivers/fsl_smc.c \
../drivers/fsl_sysmpu.c \
../drivers/fsl_trgmux.c \
../drivers/fsl_wdog32.c 

OBJS += \
./drivers/fsl_acmp.o \
./drivers/fsl_adc12.o \
./drivers/fsl_cache.o \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_crc.o \
./drivers/fsl_dac32.o \
./drivers/fsl_dmamux.o \
./drivers/fsl_edma.o \
./drivers/fsl_ewm.o \
./drivers/fsl_flexio.o \
./drivers/fsl_flexio_i2c_master.o \
./drivers/fsl_flexio_spi.o \
./drivers/fsl_flexio_spi_edma.o \
./drivers/fsl_flexio_uart.o \
./drivers/fsl_flexio_uart_edma.o \
./drivers/fsl_ftfx_cache.o \
./drivers/fsl_ftfx_controller.o \
./drivers/fsl_ftfx_flash.o \
./drivers/fsl_ftfx_flexnvm.o \
./drivers/fsl_ftm.o \
./drivers/fsl_gpio.o \
./drivers/fsl_lmem_cache.o \
./drivers/fsl_lpi2c.o \
./drivers/fsl_lpi2c_cmsis.o \
./drivers/fsl_lpi2c_edma.o \
./drivers/fsl_lpit.o \
./drivers/fsl_lpspi.o \
./drivers/fsl_lpspi_cmsis.o \
./drivers/fsl_lpspi_edma.o \
./drivers/fsl_lptmr.o \
./drivers/fsl_lpuart.o \
./drivers/fsl_lpuart_cmsis.o \
./drivers/fsl_lpuart_edma.o \
./drivers/fsl_pdb.o \
./drivers/fsl_pmc.o \
./drivers/fsl_pwt.o \
./drivers/fsl_rcm.o \
./drivers/fsl_rtc.o \
./drivers/fsl_sim.o \
./drivers/fsl_smc.o \
./drivers/fsl_sysmpu.o \
./drivers/fsl_trgmux.o \
./drivers/fsl_wdog32.o 

C_DEPS += \
./drivers/fsl_acmp.d \
./drivers/fsl_adc12.d \
./drivers/fsl_cache.d \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_crc.d \
./drivers/fsl_dac32.d \
./drivers/fsl_dmamux.d \
./drivers/fsl_edma.d \
./drivers/fsl_ewm.d \
./drivers/fsl_flexio.d \
./drivers/fsl_flexio_i2c_master.d \
./drivers/fsl_flexio_spi.d \
./drivers/fsl_flexio_spi_edma.d \
./drivers/fsl_flexio_uart.d \
./drivers/fsl_flexio_uart_edma.d \
./drivers/fsl_ftfx_cache.d \
./drivers/fsl_ftfx_controller.d \
./drivers/fsl_ftfx_flash.d \
./drivers/fsl_ftfx_flexnvm.d \
./drivers/fsl_ftm.d \
./drivers/fsl_gpio.d \
./drivers/fsl_lmem_cache.d \
./drivers/fsl_lpi2c.d \
./drivers/fsl_lpi2c_cmsis.d \
./drivers/fsl_lpi2c_edma.d \
./drivers/fsl_lpit.d \
./drivers/fsl_lpspi.d \
./drivers/fsl_lpspi_cmsis.d \
./drivers/fsl_lpspi_edma.d \
./drivers/fsl_lptmr.d \
./drivers/fsl_lpuart.d \
./drivers/fsl_lpuart_cmsis.d \
./drivers/fsl_lpuart_edma.d \
./drivers/fsl_pdb.d \
./drivers/fsl_pmc.d \
./drivers/fsl_pwt.d \
./drivers/fsl_rcm.d \
./drivers/fsl_rtc.d \
./drivers/fsl_sim.d \
./drivers/fsl_smc.d \
./drivers/fsl_sysmpu.d \
./drivers/fsl_trgmux.d \
./drivers/fsl_wdog32.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DCPU_MKE14F512VLL16 -DCPU_MKE14F512VLL16_cm4 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I../board -I../source -I../ -I../drivers -I../device -I../CMSIS -I../CMSIS_driver -I../component/serial_manager -I../utilities -I../utilities/utilities -I../component/lists -I../component/uart -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


