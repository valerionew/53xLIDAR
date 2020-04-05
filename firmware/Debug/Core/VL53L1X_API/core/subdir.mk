################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/VL53L1X_API/core/VL53L1X_api.c \
../Core/VL53L1X_API/core/VL53L1X_calibration.c 

OBJS += \
./Core/VL53L1X_API/core/VL53L1X_api.o \
./Core/VL53L1X_API/core/VL53L1X_calibration.o 

C_DEPS += \
./Core/VL53L1X_API/core/VL53L1X_api.d \
./Core/VL53L1X_API/core/VL53L1X_calibration.d 


# Each subdirectory must supply rules for building sources it contributes
Core/VL53L1X_API/core/VL53L1X_api.o: ../Core/VL53L1X_API/core/VL53L1X_api.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Core/Inc -I../USB_DEVICE/App -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"/home/mauro/git/53xLIDAR/firmware/Core/VL53L1X_API/core" -I"/home/mauro/git/53xLIDAR/firmware/Core/VL53L1X_API/platform" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/VL53L1X_API/core/VL53L1X_api.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/VL53L1X_API/core/VL53L1X_calibration.o: ../Core/VL53L1X_API/core/VL53L1X_calibration.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/Target -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Core/Inc -I../USB_DEVICE/App -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"/home/mauro/git/53xLIDAR/firmware/Core/VL53L1X_API/core" -I"/home/mauro/git/53xLIDAR/firmware/Core/VL53L1X_API/platform" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/VL53L1X_API/core/VL53L1X_calibration.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

