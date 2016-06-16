class PID:
    """
    Discrete PID control copied from tum_ardrone
    """

    def __init__(self, param_xy = (0,0,0,0), param_z = (0,0,0,0), param_yaw=(0,0,0,0)):
        # param = (P, I, D, setpoint, max)

        self.Kp_xy = param_xy[0]
        self.Ki_xy = param_xy[1]
        self.Kd_xy = param_xy[2]
        self.set_point_xy = param_xy[3]
        self.max_xy = param_xy[4]
        self.error_xy = 0.0

        self.Kp_z = param_z[0]
        self.Ki_z = param_z[1]
        self.Kd_z = param_z[2]
        self.set_point_z = param_z[3]
        self.max_z = param_z[4]
        self.error_z = 0.0

        self.Kp_yaw = param_yaw[0]
        self.Ki_yaw = param_yaw[1]
        self.Kd_yaw = param_yaw[2]
        self.set_point_yaw = param_yaw[3]
        self.max_yaw = param_yaw[4]
        self.error_yaw = 0.0

    def update(self, new_value):
        # new_value = (x_dist, y_dist, z_dist, yaw)


        return PID

    def angleFromTo2(self, angle, min, sup):
        # convert angle to -
    def setPoint(self, set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator = 0
        self.Derivator = 0

    def setPID(self, PID):
        self.setKp(PID[0])
        self.setKi(PID[1])
        self.setKd(PID[2])

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator
