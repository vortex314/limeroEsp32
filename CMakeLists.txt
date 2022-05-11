cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(WIFI_SSID $ENV{WIFI_SSID}) # copy from env
set(WIFI_PASS $ENV{WIFI_PASS})

set(ESPPORT /dev/ttyUSB0)

idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)

set(EXTRA_COMPONENT_DIRS /home/lieven/workspace/limero/esp32_idf)
set(LIMERO /home/lieven/workspace/limero )

include_directories( main 
    ${LIMERO}/inc  
      ../tinycbor/src  )



add_definitions(-DWIFI_PASS=${WIFI_PASS} 
    -DWIFI_SSID=${WIFI_SSID} 
    -DESP32_IDF 
    -DSERIAL_BAUD=921600
    -DHOSTNAME=esp32
    -fexceptions
    ) # was no-exceptions 

project(limeroEsp32)
