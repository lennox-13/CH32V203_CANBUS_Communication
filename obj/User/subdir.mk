################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/CAN_BUS_Comm_ID_320_main.c \
../User/ch32v20x_it.c \
../User/system_ch32v20x.c 

OBJS += \
./User/CAN_BUS_Comm_ID_320_main.o \
./User/ch32v20x_it.o \
./User/system_ch32v20x.o 

C_DEPS += \
./User/CAN_BUS_Comm_ID_320_main.d \
./User/ch32v20x_it.d \
./User/system_ch32v20x.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"C:\MRS_DATA\workspace\CAN_BUS_Comm_ID_320\Debug" -I"C:\MRS_DATA\workspace\CAN_BUS_Comm_ID_320\Core" -I"C:\MRS_DATA\workspace\CAN_BUS_Comm_ID_320\User" -I"C:\MRS_DATA\workspace\CAN_BUS_Comm_ID_320\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

