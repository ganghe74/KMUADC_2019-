class PidCal:
    error_sum = 0
    error_old = 0
    # p = [0.0035, 0.000005, 0.005] # optimized kp,ki,kd
    p = [0.0035, 0.000005, 0.005]
    dp = [p[0]/10, p[1]/10, p[2]/10] # to twiddle kp, ki, kd

    def __init__(self):
        print "init PidCal"
        self.x = 0
    def cal_error(self, setpoint=318):
        return setpoint - self.x

    # twiddle is for optimize the kp,ki,kd
    def twiddle(self, setpoint=318):
        best_err = self.cal_error()
        #threshold = 0.001
        #threshold = 1e-09 
        threshold = 0.0000000000000000000000000000001

        # searching by move 1.1x to the target and if go more through the target comeback to -2x
        while sum(self.dp) > threshold:
            for i in range(len(self.p)):
                self.p[i] += self.dp[i]
                err = self.cal_error()

                if err < best_err:  # There was some improvement
                    best_err = err
                    self.dp[i] *= 1.1
                else:  # There was no improvement
                    self.p[i] -= 2*self.dp[i]  # Go into the other direction
                    err = self.cal_error()

                    if err < best_err:  # There was an improvement
                        best_err = err
                        self.dp[i] *= 1.05
                    else:  # There was no improvement
                        self.p[i] += self.dp[i]
                        # As there was no improvement, the step size in either
                        # direction, the step size might simply be too big.
                        self.dp[i] *= 0.95

        print(self.p)

    # setpoint is the center and the x_current is where the car is
    # width = 640, so 320 is the center but 318 is more accurate in real
    def pid_control(self, x_current, setpoint=318):
        print "HHHHHHHHHHHHHHH"
        print x_current
        self.x = int(x_current)
        self.twiddle()

        error = setpoint - x_current
        p1 = round(self.p[0] * error, 9)
        self.error_sum += error
        i1 = round(self.p[1] * self.error_sum, 9)
        d1 = round(self.p[2] * (error -  self.error_old), 9)
        self.error_old = error
        pid = p1 + i1 + d1
        #print("p : " ,p)
        #print("i : " ,i)
        #print("d : " ,d)
        return pid

