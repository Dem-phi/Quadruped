; Auto-generated. Do not edit!


(cl:in-package wtr_serial-msg)


;//! \htmlinclude actor_1.msg.html

(cl:defclass <actor_1> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (agl_0
    :reader agl_0
    :initarg :agl_0
    :type cl:float
    :initform 0.0)
   (agl_1
    :reader agl_1
    :initarg :agl_1
    :type cl:float
    :initform 0.0)
   (agl_2
    :reader agl_2
    :initarg :agl_2
    :type cl:float
    :initform 0.0))
)

(cl:defclass actor_1 (<actor_1>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <actor_1>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'actor_1)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wtr_serial-msg:<actor_1> is deprecated: use wtr_serial-msg:actor_1 instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <actor_1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wtr_serial-msg:id-val is deprecated.  Use wtr_serial-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'agl_0-val :lambda-list '(m))
(cl:defmethod agl_0-val ((m <actor_1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wtr_serial-msg:agl_0-val is deprecated.  Use wtr_serial-msg:agl_0 instead.")
  (agl_0 m))

(cl:ensure-generic-function 'agl_1-val :lambda-list '(m))
(cl:defmethod agl_1-val ((m <actor_1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wtr_serial-msg:agl_1-val is deprecated.  Use wtr_serial-msg:agl_1 instead.")
  (agl_1 m))

(cl:ensure-generic-function 'agl_2-val :lambda-list '(m))
(cl:defmethod agl_2-val ((m <actor_1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wtr_serial-msg:agl_2-val is deprecated.  Use wtr_serial-msg:agl_2 instead.")
  (agl_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <actor_1>) ostream)
  "Serializes a message object of type '<actor_1>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agl_0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agl_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agl_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <actor_1>) istream)
  "Deserializes a message object of type '<actor_1>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agl_0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agl_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agl_2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<actor_1>)))
  "Returns string type for a message object of type '<actor_1>"
  "wtr_serial/actor_1")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'actor_1)))
  "Returns string type for a message object of type 'actor_1"
  "wtr_serial/actor_1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<actor_1>)))
  "Returns md5sum for a message object of type '<actor_1>"
  "41170c4af0e4f57d0ef889762422e6e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'actor_1)))
  "Returns md5sum for a message object of type 'actor_1"
  "41170c4af0e4f57d0ef889762422e6e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<actor_1>)))
  "Returns full string definition for message of type '<actor_1>"
  (cl:format cl:nil "uint8 id~%float32 agl_0~%float32 agl_1~%float32 agl_2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'actor_1)))
  "Returns full string definition for message of type 'actor_1"
  (cl:format cl:nil "uint8 id~%float32 agl_0~%float32 agl_1~%float32 agl_2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <actor_1>))
  (cl:+ 0
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <actor_1>))
  "Converts a ROS message object to a list"
  (cl:list 'actor_1
    (cl:cons ':id (id msg))
    (cl:cons ':agl_0 (agl_0 msg))
    (cl:cons ':agl_1 (agl_1 msg))
    (cl:cons ':agl_2 (agl_2 msg))
))
