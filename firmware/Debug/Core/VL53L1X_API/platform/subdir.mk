################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/VL53L1X_API/platform/vl53l1_platform.c 

OBJS += \
./Core/VL53L1X_API/platform/vl53l1_platform.o 

C_DEPS += \
./Core/VL53L1X_API/platform/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Core/VL53L1X_API/platform/vl53l1_platform.o: ../Core/VL53L1X_API/platform/vl53l1_platform.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Core/Inc -I../USB_DEVICE/App -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/valer/Documents/53xLIDAR/firmware/Core/VL53L1X_API/core" -I"C:/Users/valer/Documents/53xLIDAR/firmware/Core/VL53L1X_API/platform" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/VL53L1X_API/platform/vl53l1_platform.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
