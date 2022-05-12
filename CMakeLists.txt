cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(limeroEsp32)

set(WIFI_SSID $ENV{WIFI_SSID}) # copy from env
set(WIFI_PASS $ENV{WIFI_PASS})

set(ESPPORT /dev/ttyUSB0)

idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)

set(EXTRA_COMPONENT_DIRS /home/lieven/workspace/limero/esp32_idf)
set(LIMERO /home/lieven/workspace/limero )
set(ARDUINOJSON /home/lieven/workspace/ArduinoJson )
set(IDF_PATH  /home/lieven/esp/esp-idf)
set(IDF_CCACHE_ENABLE 1)

set(DEFINES -DWIFI_SSID=${WIFI_SSID} -DWIFI_PASS=${WIFI_PASS}  -DESP32_IDF=1 $(DEFINE) -DMQTT_HOST=limero.ddns.net -DMQTT_PORT=1883 
)

include_directories( main 
    ${LIMERO}/inc  
    ${ARDUINOJSON}/src
    $(IDF_PATH)/components/freertos/include/freertos
     )

add_definitions(
    -DWIFI_PASS=${WIFI_PASS} 
    -DWIFI_SSID=${WIFI_SSID} 
    -DMQTT_HOST=limero.ddns.net 
    -DMQTT_PORT=1883
    -DESP32_IDF 
    -DSERIAL_BAUD=921600
    -DHOSTNAME=esp32
    -fno-rtti 
    -ffunction-sections 
    -fdata-sections 
    -std=gnu++11 
    -fno-exceptions 
    -lstdc++ 
    -faligned-new
    ) # was no-exceptions 


