; Auto-generated. Do not edit!


(cl:in-package serial_stm32-msg)


;//! \htmlinclude uartUSB.msg.html

(cl:defclass <uartUSB> (roslisp-msg-protocol:ros-message)
  ((dir1
    :reader dir1
    :initarg :dir1
    :type cl:string
    :initform "")
   (dir2
    :reader dir2
    :initarg :dir2
    :type cl:string
    :initform "")
   (dir3
    :reader dir3
    :initarg :dir3
    :type cl:string
    :initform ""))
)

(cl:defclass uartUSB (<uartUSB>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <uartUSB>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'uartUSB)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_stm32-msg:<uartUSB> is deprecated: use serial_stm32-msg:uartUSB instead.")))

(cl:ensure-generic-function 'dir1-val :lambda-list '(m))
(cl:defmethod dir1-val ((m <uartUSB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_stm32-msg:dir1-val is deprecated.  Use serial_stm32-msg:dir1 instead.")
  (dir1 m))

(cl:ensure-generic-function 'dir2-val :lambda-list '(m))
(cl:defmethod dir2-val ((m <uartUSB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_stm32-msg:dir2-val is deprecated.  Use serial_stm32-msg:dir2 instead.")
  (dir2 m))

(cl:ensure-generic-function 'dir3-val :lambda-list '(m))
(cl:defmethod dir3-val ((m <uartUSB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_stm32-msg:dir3-val is deprecated.  Use serial_stm32-msg:dir3 instead.")
  (dir3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <uartUSB>) ostream)
  "Serializes a message object of type '<uartUSB>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dir1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dir1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dir2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dir2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dir3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dir3))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <uartUSB>) istream)
  "Deserializes a message object of type '<uartUSB>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dir1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dir1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dir2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dir2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dir3) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dir3) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<uartUSB>)))
  "Returns string type for a message object of type '<uartUSB>"
  "serial_stm32/uartUSB")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'uartUSB)))
  "Returns string type for a message object of type 'uartUSB"
  "serial_stm32/uartUSB")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<uartUSB>)))
  "Returns md5sum for a message object of type '<uartUSB>"
  "00bcf444a943c3a48aff860ffd0aed64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'uartUSB)))
  "Returns md5sum for a message object of type 'uartUSB"
  "00bcf444a943c3a48aff860ffd0aed64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<uartUSB>)))
  "Returns full string definition for message of type '<uartUSB>"
  (cl:format cl:nil "string dir1~%string dir2~%string dir3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'uartUSB)))
  "Returns full string definition for message of type 'uartUSB"
  (cl:format cl:nil "string dir1~%string dir2~%string dir3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <uartUSB>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'dir1))
     4 (cl:length (cl:slot-value msg 'dir2))
     4 (cl:length (cl:slot-value msg 'dir3))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <uartUSB>))
  "Converts a ROS message object to a list"
  (cl:list 'uartUSB
    (cl:cons ':dir1 (dir1 msg))
    (cl:cons ':dir2 (dir2 msg))
    (cl:cons ':dir3 (dir3 msg))
))
