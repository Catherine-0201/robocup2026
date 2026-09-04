#include "pid_tracking/pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
            input = max;       \
        else if (input < -max) \
            input = -max;      \
    }
PID::PID(){

}
/**
 * @brief Construct a new PID object
 *
 * @param mode
 * @param PID
 * @param max_out
 * @param max_iout
 */
void PID::init(uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    this->mode = mode;
    this->Kp = PID[0];
    this->Ki = PID[1];
    this->Kd = PID[2];
    this->max_out = max_out;
    this->max_iout = max_iout;
    std::fill(std::begin(Dbuf), std::end(Dbuf), 0.0f);
    std::fill(std::begin(error), std::end(error), 0.0f);
    Pout = Iout = Dout = out = 0.0f;
}
/**
 * @brief 计算
 *
 * @param ref
 * @param set
 * @return float
 */
float PID::calc(float ref, float set)
{
    error[2] = error[1];
    error[1] = error[0];
    this->set = set;
    fdb = ref;
    error[0] = set - ref;

    if (mode == PID_POSITION)
    {
        Pout = Kp * error[0];
        Iout += Ki * error[0];
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = (error[0] - error[1]);
        Dout = Kd * Dbuf[0];
        LimitMax(Iout, max_iout);
        out = Pout + Iout + Dout;
        LimitMax(out, max_out);
    }
    else if (mode == PID_DELTA)
    {
        Pout = Kp * (error[0] - error[1]);
        Iout = Ki * error[0];
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = (error[0] - 2.0f * error[1] + error[2]);
        Dout = Kd * Dbuf[0];
        out += Pout + Iout + Dout;
        LimitMax(out, max_out);
    }
    return out;
}
/**
 * @brief 清空累积误差
 *
 */
void PID::clear()
{
    std::fill(std::begin(error), std::end(error), 0.0f);
    std::fill(std::begin(Dbuf), std::end(Dbuf), 0.0f);
    out = Pout = Iout = Dout = 0.0f;
    fdb = set = 0.0f;
}
PID::~PID()
{
    clear();
}