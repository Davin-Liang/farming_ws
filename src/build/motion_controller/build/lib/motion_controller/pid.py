class PID:
    def __init__(self, Kp, Ki, Kd, max_out, max_iout):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_out = max_out
        self.max_iout = max_iout

        self.set = 0.0
        self.fdb = 0.0

        self.out = 0.0
        self.Pout = 0.0
        self.Iout = 0.0
        self.Dout = 0.0
        self.Dbuf = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]

    def pid_calculate(self, ref, goal):
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.set = goal
        self.fdb = ref
        self.error[0] = goal - ref

        self.Pout = self.Kp * self.error[0]
        self.Iout += self.Ki * self.error[0]
        self.Dbuf[2] = self.Dbuf[1]
        self.Dbuf[1] = self.Dbuf[0]
        self.Dbuf[0] = (self.error[0] - self.error[1])
        self.Dout = self.Kd * self.Dbuf[0]
        self.limit_max_iout()
        self.out = self.Pout + self.Iout + self.Dout
        self.limit_max_out()


    def limit_max_iout(self):
        if self.Iout > self.max_iout:
            self.Iout = self.max_iout
        elif self.Iout < -self.max_iout:
            self.Iout = -self.max_iout

    def limit_max_out(self):
        if self.out > self.max_out:
            self.out = self.max_out
        elif self.out < -self.max_out:
            self.out = -self.max_out       