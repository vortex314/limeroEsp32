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
<<<<<<< HEAD
    ValueFlow<int32_t> x,y,z,status;
=======
    ValueSource<int32_t> x,y,z,status;
>>>>>>> 96002dfcc952c0e85e653b43b0b9baa57a7f0fc3
    Compass(Thread&,Uext&);
    virtual ~Compass() ;
    void init();
};

#endif // COMPASS_H
