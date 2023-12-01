import time
kp = 0.04
ki = 0.075
kd = 0.02

multi = 1.75
kp = 0.045 * multi
ki = 0.05 * multi
kd = 0.03 * multi

class PID:
    prevIntegral = 0
    prevError = 0
    windup = 20
    def regulate(self, target, current, dt):
        e = target-current

        P = kp*e
        I = self.prevIntegral + (ki*e)*dt
        D = (e - self.prevError)/dt
        if (I > self.windup):
            I = self.windup
        if (I < -self.windup):
            I = -self.windup

        self.prevIntegral = I
        self.prevError = e

        return P + I + D*kd