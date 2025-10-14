; Auto-generated. Do not edit!


(cl:in-package gimbal_control_serial-msg)


;//! \htmlinclude GimbalCmd.msg.html

(cl:defclass <GimbalCmd> (roslisp-msg-protocol:ros-message)
  ((roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GimbalCmd (<GimbalCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gimbal_control_serial-msg:<GimbalCmd> is deprecated: use gimbal_control_serial-msg:GimbalCmd instead.")))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <GimbalCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gimbal_control_serial-msg:roll-val is deprecated.  Use gimbal_control_serial-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <GimbalCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gimbal_control_serial-msg:pitch-val is deprecated.  Use gimbal_control_serial-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <GimbalCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gimbal_control_serial-msg:yaw-val is deprecated.  Use gimbal_control_serial-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <GimbalCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gimbal_control_serial-msg:mode-val is deprecated.  Use gimbal_control_serial-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalCmd>) ostream)
  "Serializes a message object of type '<GimbalCmd>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalCmd>) istream)
  "Deserializes a message object of type '<GimbalCmd>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalCmd>)))
  "Returns string type for a message object of type '<GimbalCmd>"
  "gimbal_control_serial/GimbalCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalCmd)))
  "Returns string type for a message object of type 'GimbalCmd"
  "gimbal_control_serial/GimbalCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalCmd>)))
  "Returns md5sum for a message object of type '<GimbalCmd>"
  "95e52dfca23ef72d66122799a9e1eabc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalCmd)))
  "Returns md5sum for a message object of type 'GimbalCmd"
  "95e52dfca23ef72d66122799a9e1eabc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalCmd>)))
  "Returns full string definition for message of type '<GimbalCmd>"
  (cl:format cl:nil "float32 roll~%float32 pitch~%float32 yaw~%uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalCmd)))
  "Returns full string definition for message of type 'GimbalCmd"
  (cl:format cl:nil "float32 roll~%float32 pitch~%float32 yaw~%uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalCmd>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalCmd
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':mode (mode msg))
))
