### Package Required
1. Ardrone Autonomy
2. ar_track_alvar
3. rosserial_xbee
4. TF
5. geometry_msgs
6. std_msgs

### How to start it
1. Turn on quadcopter.
2. Plug in xbee explorer to the computer. (Make sure the xbee module on it is marked "0")
3. Connect to the wifi network of the quadcopter.
4. roslaunch autoland_pid landing_test.launch (This turns on the quadcopter driver).
5. rosrun rosserial_xbee xbee_network.py /dev/ttyUSBX 1 (Enable xbee communication between arduino and computer. Change ttyUSBX to the right port number)
6. roslaunch autoland_pid alvar.launch (This enables the tag detection)
7. roslaunch autoland_pid manual_control.launch (This enables the control of the copter by joystick and keyboard).

### How to control it
1. Make sure the drone can see the target initially
2. press "1" (Start the PID controller)
3. press "2" (Hover 80 cm off the target)
4. press "3" (Hover 40 cm off the target)
5. press "8" (Landing the copter)
6. press "4" (Engage the landing platform to latch onto the drone)

### Tag Detection

#### Slight Hardware Modification
The front facing camera of the drone is much better than the downward facing camera. I made a slight physical modification by removing the front facing camera and reinstalling it facing downward.
Thus all the code will refer to the downward facing camera as the front camera.

#### Alvar Bundle
By using a bundle, if the drone can see one target of the bundle, it can detect the "master tag" of the bundle. From far away, it can use all 5 tags to help locate itself. As it gets closer to the platform
 and the viewing angle decreases, the drone only needs to see one target to locate itself. This method allows the drone to detect the target from far away and closer up.

For bundle detection, ar_track_alvar needs an XML file describing the locations of all the individual tags in the bundle.
IMPORTANT: Unfortunately, for some unknown reason, the only way ar_track_alvar will properly use it if ar_track_alvar is manually built and the xml file is placed in the bundles directory of the built folder.
$(find ar_track_alvar)/bundles/ . Maybe I did something wrong, but this was the only way for me to get it to work.

#### TF Coordinate Transformation
Eventually, we want to use the tag location to control the pitch, row, and yaw of the drone. However, the perspective of the camera is different than the perspective of the drone itself, i.e. x,y,z coordinates
detected does not directly correspond to pitch and row of the drone. ROS provided an extremely useful library called TF. You need to set up a broadcaster, which sends out position information at particular time
and coordinate frame, and a listener, which pulls information from the broadcaster and spits back out position information for a particular coordinate frame you want.

#### Code Explanation
```python
    def ReceiveAlvar(self, data):
        br = tf.TransformBroadcaster()
        self.tags = data.markers
        if self.tags != []: # Check if currently, the drone can see a target or not.
            for i in self.tags:
                if i.id == 0: # The master tag of the bundle has an ID 0
                    self.lastSeenPos = i.pose.pose.position
                    self.lastSeenOri = i.pose.pose.orientation
                    pos = i.pose.pose.position
                    ori = i.pose.pose.orientation
                    br.sendTransform((pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), "target", "ardrone_base_bottomcam") #Sending out position information from coordinate frame of the ardrone_base_bottomcam
        elif self.lastSeenPos == ():
            pass
        elif self.PIDenable: # if the tags are not seen, but was recently seen, it sends out last position seen.
            br.sendTransform((self.lastSeenPos.x, self.lastSeenPos.y, self.lastSeenPos.z), (self.lastSeenOri.x, self.lastSeenOri.y, self.lastSeenOri.z, self.lastSeenOri.w), rospy.Time.now(), "target", "ardrone_base_bottomcam")
            rospy.loginfo("Target lost: using previous location data point")
```
Note here that the coordinate frame is from the ardrone bottom camera. This is because I move the front facing camera to a downward facing position near the bottom cam. Thus, the frame is with respect to the bottom cam.

```python
        if self.listener.frameExists("/target"):
            try:
                (vector, rot) = self.listener.lookupTransform('/ardrone_base_link', "/target", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
```
The listener here pulls information from /target, and does transformation into the perspective of the drone.

#### Frame setup
The two frames we used are /ardrone_base_link and /ardrone_base_bottomcam are setup by Ardrone Autonomy which is the driver running the quadcopter. No setup is needed getting the frames setup.

### Controller

#### Roll/Pitch
The quadcopter moves around in the same altitude by changing its roll and pitch angles. I made two separates but identical controller, one for pitch and one for roll.

##### PD
Kp * (u) + Kd (vd - v) = PD where desire speed, vd, is zero.

After tuning for a long time, I still haven't found good values for the parameters. The best performance I had was getting the drone to hover above the target, but it had some
jitteriness (it jerks side to side while trying to hover over the target).
##### PD (DD)
Kp * (u) + Kd (vd - v) + Kdd (ad - a) = PD where desire speed, vd; and desire acceleration, ad, are zero.

This is a good short term fix for the jitteriness. By adding another term for the acceleration, I smoothed out the motion of the copter and almost eliminate the jerking motion of the quadcopter. I highly
recommend trying to tune the parameters some more. The performance of this entire project depends on the landing accuracy of the quadcopter. While the landing pad can accomodate for some inaccurate landing,
it's not capable of adjusting an error of more than +- 3 inches in its current iteration.

#### Yaw (PID) and Altitude (PID)
Kp * (e) + Ki * integrate(e) + Kd * (d/dt)(e) = PID.
The controllers for yaw and altitude are pretty much working correctly. They are both PID controller, and their parameters are tuned enough so that it's working well enough for our purpose.

### Arduino Code
The electronics on the actual landing platform is completely independent from the