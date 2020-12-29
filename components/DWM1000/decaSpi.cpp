#include <Hardware.h>
#include <Log.h>
#include <deca_device_api.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#ifdef ESP8266_OPEN_RTOS
#endif

Spi* _gSpi;     // to support irq 


void spi_set_global(Spi* spi) {
    _gSpi = spi;
}

std::string out;
std::string in;

//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" int writetospi(uint16 hLen, const uint8 *hbuff, uint32 bLen,
                          const uint8 *buffer)
{

    out.clear();
    out.append((char*)hbuff,hLen);
    out.append((char*)buffer,bLen);
    _gSpi->exchange(in,out);
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////

extern "C" int readfromspi(uint16 hLen, const uint8 *hbuff, uint32 bLen, uint8 *buffer)
{
    out.clear();
    out.append((char*)hbuff,hLen);
    out.append((char*)buffer,bLen);
    _gSpi->exchange(in,out);
    for(int i=hLen;i<out.length();i++) {
        *buffer++ = in[i];
    }
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_low()
{
    _gSpi->setClock(Spi::SPI_CLOCK_1M);
    _gSpi->deInit();
    _gSpi->init();
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_high()
{
   _gSpi->setClock(Spi::SPI_CLOCK_10M);
   _gSpi->deInit();
   _gSpi->init();
}

#include <FreeRTOS.h>
#include <task.h>

extern "C" decaIrqStatus_t decamutexon() {
//    noInterrupts();

return 0;
}

extern "C" void decamutexoff(decaIrqStatus_t s) {
//    interrupts();

}



