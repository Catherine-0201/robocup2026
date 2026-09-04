#ifndef PID_H
#define PID_H

#include <vector>
#include "tf2/utils.h"
#include <cstdint>
using namespace std;



enum PIDMode {
    PID_POSITION,
    PID_DELTA
};

class PID {
public:
    PID();
    void init(uint8_t mode, const float PID[3], float max_out, float max_iout);
    float calc(float ref, float set);
    void clear();
    ~PID();

private:
    uint8_t mode;
    float Kp, Ki, Kd;
    float max_out, max_iout;
    float Dbuf[3];
    float error[3];
    float Pout, Iout, Dout, out;
    float fdb, set;
};

#endif // PID_H
