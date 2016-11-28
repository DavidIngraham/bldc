##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16 
  USE_OPT += -DBOARD_OTG_NOVBUSSENS $(build_args)
  USE_OPT += -fsingle-precision-constant -Wdouble-promotion
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=gnu99 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -std=c++11 
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = yes
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# Enable this if you really want to use the STM FWLib.
ifeq ($(USE_FWLIB),)
  USE_FWLIB = yes
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = BLDC_4_ChibiOS

# Imported source files and paths
CHIBIOS = ChibiOS_3.0.2
# Startup files.
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/boards/ST_STM32F4_DISCOVERY/board.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Other files (optional).
#include $(CHIBIOS)/test/rt/test.mk
include hwconf/hwconf.mk
include applications/applications.mk
include nrf/nrf.mk

# Define linker script file here
LDSCRIPT= ld_eeprom_emu.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       $(CHIBIOS)/os/various/syscalls.c \
       main.c \
       comm_usb_serial.c \
       irq_handlers.c \
       buffer.c \
       comm_usb.c \
       crc.c \
       digital_filter.c \
       ledpwm.c \
       mcpwm.c \
       servo_dec.c \
       utils.c \
       servo.c \
       servo_simple.c \
       packet.c \
       terminal.c \
       conf_general.c \
       eeprom.c \
       commands.c \
       timeout.c \
       comm_uavcan.c \
       ws2811.c \
       led_external.c \
       encoder.c \
       flash_helper.c \
       mc_interface.c \
       mcpwm_foc.c \
       $(HWSRC) \
       $(APPSRC) \
       $(NRFSRC) 

#CSRC += comm_can.c 


# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = 

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(TESTINC) \
         $(CHIBIOS)/os/various \
         $(CHIBIOS)/os/hal/lib/streams \
         mcconf \
         appconf \
         $(HWINC) \
         $(APPINC) \
         $(NRFINC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = 

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =  

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm


#
# UAVCAN library
#

UDEFS += -DUAVCAN_STM32_CHIBIOS=1       # Tell UAVCAN we're using CHIBIOS
UDEFS += -DUAVCAN_STM32_TIMER_NUMBER=6  # Any suitable timer number
UDEFS += -DUAVCAN_STM32_NUM_IFACES=1     # Number of CAN interfaces to use (1 - use only CAN1; 2 - both CAN1 and CAN2)
UDEFS += -DUAVCAN_CPP_VERSION=UAVCAN_CPP11                

# Add libuavcan core sources and includes
include libuavcan/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

# Add STM32 driver sources and includes
include libuavcan/libuavcan_drivers/stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_STM32_INC)

# Add Chibios C++ Wrappers (Needed here as UAVCAN requires CPP while VESC doesn't)
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
CPPSRC += $(CHPPSRC)
UINCDIR += $(CHCPPINC)

# Invoke DSDL compiler and add its default output directory to the include search path
$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
UINCDIR += dsdlc_generated      # This is where the generated headers are stored by default

# Zubax implementation of some std library functions necessary for uavcan
ZUBAX_CHIBIOS_DIR = zubax_chibios
CPPSRC += $(ZUBAX_CHIBIOS_DIR)/zubax_chibios/sys/libstdcpp.cpp                  \
          $(ZUBAX_CHIBIOS_DIR)/zubax_chibios/sys/sys_console.cpp                \
          $(ZUBAX_CHIBIOS_DIR)/zubax_chibios/sys/sys.cpp




#
# End of user defines
##############################################################################




ifeq ($(USE_FWLIB),yes)
  include $(CHIBIOS)/ext/stdperiph_stm32f4/stm32lib.mk
  CSRC += $(STM32SRC)
  INCDIR += $(STM32INC)
  USE_OPT += -DUSE_STDPERIPH_DRIVER
endif


build/$(PROJECT).bin: build/$(PROJECT).elf 
	$(BIN) build/$(PROJECT).elf build/$(PROJECT).bin

# Program
upload: build/$(PROJECT).bin
	#qstlink2 --cli --erase --write build/$(PROJECT).bin
	#openocd -f interface/stlink-v2.cfg -c "set WORKAREASIZE 0x2000" -f target/stm32f4x_stlink.cfg -c "program build/$(PROJECT).elf verify reset" # Older openocd
	openocd -f board/stm32f4discovery.cfg -c "reset_config trst_only combined" -c "program build/$(PROJECT).elf verify reset exit" # For openocd 0.9

#program with olimex arm-usb-tiny-h and jtag-swd adapter board. needs openocd>=0.9
upload-olimex: build/$(PROJECT).bin
	openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -c "set WORKAREASIZE 0x2000" -f target/stm32f4x.cfg -c "program build/$(PROJECT).elf verify reset"

debug-start:
	openocd -f stm32-bv_openocd.cfg

RULESPATH = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk
