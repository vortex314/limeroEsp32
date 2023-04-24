idf_component_register(SRCS 
            I2cScanner.cpp 
            as5600Test.cpp 
            Battery.cpp  
            MotorDrive.cpp          
        INCLUDE_DIRS .
        REQUIRES 
            as5600 
            nvs_flash 
            ledBlinker 
            udpRedis
            ina3221
            )