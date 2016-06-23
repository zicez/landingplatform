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
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, D=0.0, DD=0.0, maxValue=.1):
        self.Kp = P
        self.Kd = D
        self.Kdd = DD
        self.set_point = 0.0
        self.set_speed = 0.0
        self.set_accel = 0.0
        self.max = maxValue
        self.previousSpeed = 0.0

    def update(self, current_point, current_speed, dt):
        """
        Calculate PID output value for given reference input and feedback
        """
        point_error = self.set_point - current_point
        speed_error = self.set_speed - current_speed
        accel_error = 0
        if dt > 0:
            accel_error = self.set_accel - (current_speed - self.previousSpeed)/dt
        self.previousSpeed = current_speed
        self.P_value = self.Kp * point_error
        self.D_value = self.Kd * speed_error
        self.DD_value = self.Kdd * accel_error
        PID = self.P_value + self.D_value + self.DD_value
        return self.saturation(PID, self.max)

    def saturation(self, value, maximum):
        return max(-maximum, min(value, maximum))

    def setPoint(self, set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point

    def setPID(self, PID):
        self.setKp(PID[0])
        self.setKd(PID[1])
        self.setKdd(PID[2])

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKd(self, D):
        self.Kd = D

    def setKdd(self, DD):
        self.Kdd = DD