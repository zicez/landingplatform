class PD:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, D=0.0, maxValue = .1):

        self.Kp=P
        self.Kd=D
        self.set_point=0.0
        self.set_speed=0.0
        self.max = maxValue
    
    def update(self,current_point, current_speed):
        """
        Calculate PID output value for given reference input and feedback
        """
        point_error = self.set_point - current_point
        speed_error = self.set_speed - current_speed
        self.P_value = self.Kp * point_error
        self.D_value = self.Kd * speed_error
        PID = self.P_value + self.D_value
        return self.saturation(PID, self.max)

    def saturation(self, value, maximum):
        return max(-maximum, min(value, maximum))

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point

    def setPID(self, PID):
        self.setKp(PID[0])
        self.setKd(PID[1])

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator
    def setDerivator(self, Derivator):
        self.Derivator = Derivator
    def setKp(self,P):
        self.Kp=P
    def setKd(self,D):
        self.Kd=D

class PD2:
    def __init__( self, gain_p = 0.0, gain_i = 0.0, gain_d = 0.0, maxValue = .1):
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d
        self.limit = maxValue

        self.previousR = 0.0
        self.dR = 0.0
        self.output = 0.0
        self.p = 0.0
        self.d = 0.0

    def saturation(self, value, maximum):
        return max(-maximum, min(value, maximum))

    def update( self, r, v, dt ):
        if dt > 0.0:
            self.dR = (r - self.previousR) / (dt)
            self.previousR = r

        self.p = self.previousR - v
        self.d = self.dR

#        print self.p,self.d,self.i

        self.output = self.gain_p * self.p + self.gain_d * self.d
        return self.saturation(self.output, self.limit)

    def reset( self ):
        self.previousR = self.dR = 0.0
        self.p = self.i = self.d = 0.0
