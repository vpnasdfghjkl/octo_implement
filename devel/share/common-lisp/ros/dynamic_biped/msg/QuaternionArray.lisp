; Auto-generated. Do not edit!


(cl:in-package dynamic_biped-msg)


;//! \htmlinclude QuaternionArray.msg.html

(cl:defclass <QuaternionArray> (roslisp-msg-protocol:ros-message)
  ((quaternions
    :reader quaternions
    :initarg :quaternions
    :type (cl:vector geometry_msgs-msg:Quaternion)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Quaternion :initial-element (cl:make-instance 'geometry_msgs-msg:Quaternion))))
)

(cl:defclass QuaternionArray (<QuaternionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuaternionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuaternionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic_biped-msg:<QuaternionArray> is deprecated: use dynamic_biped-msg:QuaternionArray instead.")))

(cl:ensure-generic-function 'quaternions-val :lambda-list '(m))
(cl:defmethod quaternions-val ((m <QuaternionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_biped-msg:quaternions-val is deprecated.  Use dynamic_biped-msg:quaternions instead.")
  (quaternions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuaternionArray>) ostream)
  "Serializes a message object of type '<QuaternionArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'quaternions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'quaternions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuaternionArray>) istream)
  "Deserializes a message object of type '<QuaternionArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'quaternions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'quaternions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Quaternion))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuaternionArray>)))
  "Returns string type for a message object of type '<QuaternionArray>"
  "dynamic_biped/QuaternionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuaternionArray)))
  "Returns string type for a message object of type 'QuaternionArray"
  "dynamic_biped/QuaternionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuaternionArray>)))
  "Returns md5sum for a message object of type '<QuaternionArray>"
  "c666021c5a7330bd53d9827c2e91d9f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuaternionArray)))
  "Returns md5sum for a message object of type 'QuaternionArray"
  "c666021c5a7330bd53d9827c2e91d9f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuaternionArray>)))
  "Returns full string definition for message of type '<QuaternionArray>"
  (cl:format cl:nil "geometry_msgs/Quaternion[] quaternions~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuaternionArray)))
  "Returns full string definition for message of type 'QuaternionArray"
  (cl:format cl:nil "geometry_msgs/Quaternion[] quaternions~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuaternionArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'quaternions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuaternionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'QuaternionArray
    (cl:cons ':quaternions (quaternions msg))
))
