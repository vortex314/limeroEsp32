#ifndef COMPASS_H
#define COMPASS_H
#include <limero.h>
#include <HMC5883L.h>
#include <Device.h>



class Compass : public Actor,public Device
{
    Uext& _uext;
    HMC5883L* _hmc;
    TimerSource measureTimer;
    struct Vector<float> _v;

public:
    ValueSource<int32_t> x,y,z,status;
    Compass(Thread&,Uext&);
    virtual ~Compass() ;
    void init();
};

#endif // COMPASS_H
