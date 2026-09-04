class PID:
    def __init__(self, mode, PID_values, max_out, max_iout):
        self.mode = mode
        self.Kp = PID_values[0]
        self.Ki = PID_values[1]
        self.Kd = PID_values[2]
        self.max_out = max_out
        self.max_iout = max_iout
        self.Dbuf = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.Pout = 0.0
        self.Iout = 0.0
        self.Dout = 0.0
        self.out = 0.0
        self.set = 0.0
        self.fdb = 0.0

    def init(self, mode, PID_values, max_out, max_iout):
        self.mode = mode
        self.Kp = PID_values[0]
        self.Ki = PID_values[1]
        self.Kd = PID_values[2]
        self.max_out = max_out
        self.max_iout = max_iout
        self.Dbuf = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]

    def limit_max(self, input_val, max_val):
        if input_val > max_val:
            input_val = max_val
        elif input_val < -max_val:
            input_val = -max_val
        return input_val

    def calc(self, ref, set_val):
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.set = set_val
        self.fdb = ref
        self.error[0] = set_val - ref

        if self.mode == "PID_POSITION":
            self.Pout = self.Kp * self.error[0]
            self.Iout += self.Ki * self.error[0]
            self.Dbuf[2] = self.Dbuf[1]
            self.Dbuf[1] = self.Dbuf[0]
            self.Dbuf[0] = self.error[0] - self.error[1]
            self.Dout = self.Kd * self.Dbuf[0]
            self.Iout = self.limit_max(self.Iout, self.max_iout)
            self.out = self.Pout + self.Iout + self.Dout
            self.out = self.limit_max(self.out, self.max_out)

        elif self.mode == "PID_DELTA":
            self.Pout = self.Kp * (self.error[0] - self.error[1])
            self.Iout = self.Ki * self.error[0]
            self.Dbuf[2] = self.Dbuf[1]
            self.Dbuf[1] = self.Dbuf[0]
            self.Dbuf[0] = self.error[0] - 2.0 * self.error[1] + self.error[2]
            self.Dout = self.Kd * self.Dbuf[0]
            self.out += self.Pout + self.Iout + self.Dout
            self.out = self.limit_max(self.out, self.max_out)

        return self.out

    def clear(self):
        self.error = [0.0, 0.0, 0.0]
        self.Dbuf = [0.0, 0.0, 0.0]
        self.out = self.Pout = self.Iout = self.Dout = 0.0
        self.fdb = self.set = 0.0
