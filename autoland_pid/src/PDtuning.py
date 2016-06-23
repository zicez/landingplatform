def SendPUp(self):
    self.xyPID = (self.xyPID[0] + .001, self.xyPID[1], self.xyPID[2])
    self.x.setPID(self.xyPID)
    rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])


def SendPDown(self):
    self.xyPID = (self.xyPID[0] - .001, self.xyPID[1], self.xyPID[2])
    self.x.setPID(self.xyPID)
    rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])


def SendIUp(self):
    self.xyPID = (self.xyPID[0], self.xyPID[1] + .0001, self.xyPID[2])
    self.x.setPID(self.xyPID)
    rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])


def SendIDown(self):
    self.xyPID = (self.xyPID[0], self.xyPID[1] - .0001, self.xyPID[2])
    self.x.setPID(self.xyPID)
    rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])


def SendDUp(self):
    self.xyPID = (self.xyPID[0], self.xyPID[1], self.xyPID[2] + .001)
    self.x.setPID(self.xyPID)
    rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])


def SendDDown(self):
    self.xyPID = (self.xyPID[0], self.xyPID[1], self.xyPID[2] - .001)
    self.x.setPID(self.xyPID)
    rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])