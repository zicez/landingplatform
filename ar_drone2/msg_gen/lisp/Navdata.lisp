; Auto-generated. Do not edit!


(cl:in-package ar_drone2-msg)


;//! \htmlinclude Navdata.msg.html

(cl:defclass <Navdata> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (battery
    :reader battery
    :initarg :battery
    :type cl:integer
    :initform 0))
)

(cl:defclass Navdata (<Navdata>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Navdata>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Navdata)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ar_drone2-msg:<Navdata> is deprecated: use ar_drone2-msg:Navdata instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_drone2-msg:position-val is deprecated.  Use ar_drone2-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_drone2-msg:altitude-val is deprecated.  Use ar_drone2-msg:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'battery-val :lambda-list '(m))
(cl:defmethod battery-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_drone2-msg:battery-val is deprecated.  Use ar_drone2-msg:battery instead.")
  (battery m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Navdata>) ostream)
  "Serializes a message object of type '<Navdata>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'battery)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'battery)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'battery)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'battery)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Navdata>) istream)
  "Deserializes a message object of type '<Navdata>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'battery)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'battery)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'battery)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'battery)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Navdata>)))
  "Returns string type for a message object of type '<Navdata>"
  "ar_drone2/Navdata")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Navdata)))
  "Returns string type for a message object of type 'Navdata"
  "ar_drone2/Navdata")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Navdata>)))
  "Returns md5sum for a message object of type '<Navdata>"
  "49de2abc292ef1d9be7a19689625ceb8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Navdata)))
  "Returns md5sum for a message object of type 'Navdata"
  "49de2abc292ef1d9be7a19689625ceb8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Navdata>)))
  "Returns full string definition for message of type '<Navdata>"
  (cl:format cl:nil "geometry_msgs/Twist position~%float32 altitude ~%uint32 battery~%        ~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Navdata)))
  "Returns full string definition for message of type 'Navdata"
  (cl:format cl:nil "geometry_msgs/Twist position~%float32 altitude ~%uint32 battery~%        ~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Navdata>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Navdata>))
  "Converts a ROS message object to a list"
  (cl:list 'Navdata
    (cl:cons ':position (position msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':battery (battery msg))
))
