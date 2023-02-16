# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)
set(PICO_BOARD pico_w)

set(BTSTACK_ROOT ${PICO_SDK_PATH}/lib/btstack)
set(BTSTACK_EXAMPLE_PATH ${BTSTACK_ROOT}/example)
set(BTSTACK_3RD_PARTY_PATH ${BTSTACK_ROOT}/3rd-party)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(bt_serial C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.c
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
	hardware_flash
	pico_multicore
	pico_btstack_ble
	pico_btstack_classic
	pico_btstack_cyw43
    pico_lwip_nosys
    pico_btstack_bnep_lwip
	pico_lwip_iperf
    pico_cyw43_arch_lwip_threadsafe_background
	)

target_include_directories(${PROJECT_NAME} PUBLIC
	./
	${BTSTACK_ROOT}/src)

target_compile_definitions(${PROJECT_NAME} INTERFACE
    CYW43_LWIP=0
	PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
    )
# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
