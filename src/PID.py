import time


class PID:
    def __init__(self, kP=0, kI=0, kD=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_time = time.time()
        self.dt = 0
        self.prev_error = 0
        self.first = True
        self.integral = 0

    def calc(self, err):
        self.dt = (time.time() - self.prev_time)
        self.integral += err * self.dt
        if self.first == False:
            self.res = self.kP*err + self.kD * \
                ((err-self.prev_error) / self.dt) + self.kI*self.integral
        else:
            self.res = self.kP*err
            self.first = False
        self.prev_error = err
        self.prev_time = time.time()
        return self.res
    # def calc(self, err, dt):
    #     self.dt = dt
    #     self.integral += err * self.dt
    #     if self.first == False:
    #         self.res = self.kP*err + self.kD * \
    #             ((err-self.prev_error) / self.dt) + self.kI*self.integral
    #     else:
    #         self.res = self.kP*err
    #         self.first = False
    #     self.prev_error = err
    #     return self.res
    def zero(self):
        self.prev_time = time.time()
        self.dt = 0
        self.prev_error = 0
        self.first = True
        self.integral = 0