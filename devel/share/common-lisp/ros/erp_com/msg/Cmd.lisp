; Auto-generated. Do not edit!


(cl:in-package erp_com-msg)


;//! \htmlinclude Cmd.msg.html

(cl:defclass <Cmd> (roslisp-msg-protocol:ros-message)
  ((e_stop
    :reader e_stop
    :initarg :e_stop
    :type cl:boolean
    :initform cl:nil)
   (gear
    :reader gear
    :initarg :gear
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0)
   (steer
    :reader steer
    :initarg :steer
    :type cl:integer
    :initform 0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Cmd (<Cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name erp_com-msg:<Cmd> is deprecated: use erp_com-msg:Cmd instead.")))

(cl:ensure-generic-function 'e_stop-val :lambda-list '(m))
(cl:defmethod e_stop-val ((m <Cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader erp_com-msg:e_stop-val is deprecated.  Use erp_com-msg:e_stop instead.")
  (e_stop m))

(cl:ensure-generic-function 'gear-val :lambda-list '(m))
(cl:defmethod gear-val ((m <Cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader erp_com-msg:gear-val is deprecated.  Use erp_com-msg:gear instead.")
  (gear m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader erp_com-msg:speed-val is deprecated.  Use erp_com-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'steer-val :lambda-list '(m))
(cl:defmethod steer-val ((m <Cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader erp_com-msg:steer-val is deprecated.  Use erp_com-msg:steer instead.")
  (steer m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <Cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader erp_com-msg:brake-val is deprecated.  Use erp_com-msg:brake instead.")
  (brake m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cmd>) ostream)
  "Serializes a message object of type '<Cmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'e_stop) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gear)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'steer)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brake)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cmd>) istream)
  "Deserializes a message object of type '<Cmd>"
    (cl:setf (cl:slot-value msg 'e_stop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gear)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steer) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brake)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cmd>)))
  "Returns string type for a message object of type '<Cmd>"
  "erp_com/Cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cmd)))
  "Returns string type for a message object of type 'Cmd"
  "erp_com/Cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cmd>)))
  "Returns md5sum for a message object of type '<Cmd>"
  "6c8d779558341c7cf957ab6058219fbb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cmd)))
  "Returns md5sum for a message object of type 'Cmd"
  "6c8d779558341c7cf957ab6058219fbb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cmd>)))
  "Returns full string definition for message of type '<Cmd>"
  (cl:format cl:nil "bool e_stop~%uint8 gear~%uint8 speed~%int32 steer~%uint8 brake~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cmd)))
  "Returns full string definition for message of type 'Cmd"
  (cl:format cl:nil "bool e_stop~%uint8 gear~%uint8 speed~%int32 steer~%uint8 brake~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cmd>))
  (cl:+ 0
     1
     1
     1
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'Cmd
    (cl:cons ':e_stop (e_stop msg))
    (cl:cons ':gear (gear msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':steer (steer msg))
    (cl:cons ':brake (brake msg))
))
