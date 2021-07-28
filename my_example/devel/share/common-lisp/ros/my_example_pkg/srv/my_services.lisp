; Auto-generated. Do not edit!


(cl:in-package my_example_pkg-srv)


;//! \htmlinclude my_services-request.msg.html

(cl:defclass <my_services-request> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:integer
    :initform 0))
)

(cl:defclass my_services-request (<my_services-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <my_services-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'my_services-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_example_pkg-srv:<my_services-request> is deprecated: use my_example_pkg-srv:my_services-request instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <my_services-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_example_pkg-srv:number-val is deprecated.  Use my_example_pkg-srv:number instead.")
  (number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <my_services-request>) ostream)
  "Serializes a message object of type '<my_services-request>"
  (cl:let* ((signed (cl:slot-value msg 'number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <my_services-request>) istream)
  "Deserializes a message object of type '<my_services-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<my_services-request>)))
  "Returns string type for a service object of type '<my_services-request>"
  "my_example_pkg/my_servicesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_services-request)))
  "Returns string type for a service object of type 'my_services-request"
  "my_example_pkg/my_servicesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<my_services-request>)))
  "Returns md5sum for a message object of type '<my_services-request>"
  "4ff0b0ab1ed04611e3a2b4090af9ee4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'my_services-request)))
  "Returns md5sum for a message object of type 'my_services-request"
  "4ff0b0ab1ed04611e3a2b4090af9ee4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<my_services-request>)))
  "Returns full string definition for message of type '<my_services-request>"
  (cl:format cl:nil "#First parameters above the line will be request and second one will be return messages~%int32 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'my_services-request)))
  "Returns full string definition for message of type 'my_services-request"
  (cl:format cl:nil "#First parameters above the line will be request and second one will be return messages~%int32 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <my_services-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <my_services-request>))
  "Converts a ROS message object to a list"
  (cl:list 'my_services-request
    (cl:cons ':number (number msg))
))
;//! \htmlinclude my_services-response.msg.html

(cl:defclass <my_services-response> (roslisp-msg-protocol:ros-message)
  ((answer
    :reader answer
    :initarg :answer
    :type cl:string
    :initform ""))
)

(cl:defclass my_services-response (<my_services-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <my_services-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'my_services-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_example_pkg-srv:<my_services-response> is deprecated: use my_example_pkg-srv:my_services-response instead.")))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <my_services-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_example_pkg-srv:answer-val is deprecated.  Use my_example_pkg-srv:answer instead.")
  (answer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <my_services-response>) ostream)
  "Serializes a message object of type '<my_services-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'answer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'answer))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <my_services-response>) istream)
  "Deserializes a message object of type '<my_services-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'answer) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'answer) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<my_services-response>)))
  "Returns string type for a service object of type '<my_services-response>"
  "my_example_pkg/my_servicesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_services-response)))
  "Returns string type for a service object of type 'my_services-response"
  "my_example_pkg/my_servicesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<my_services-response>)))
  "Returns md5sum for a message object of type '<my_services-response>"
  "4ff0b0ab1ed04611e3a2b4090af9ee4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'my_services-response)))
  "Returns md5sum for a message object of type 'my_services-response"
  "4ff0b0ab1ed04611e3a2b4090af9ee4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<my_services-response>)))
  "Returns full string definition for message of type '<my_services-response>"
  (cl:format cl:nil "string answer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'my_services-response)))
  "Returns full string definition for message of type 'my_services-response"
  (cl:format cl:nil "string answer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <my_services-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'answer))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <my_services-response>))
  "Converts a ROS message object to a list"
  (cl:list 'my_services-response
    (cl:cons ':answer (answer msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'my_services)))
  'my_services-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'my_services)))
  'my_services-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_services)))
  "Returns string type for a service object of type '<my_services>"
  "my_example_pkg/my_services")