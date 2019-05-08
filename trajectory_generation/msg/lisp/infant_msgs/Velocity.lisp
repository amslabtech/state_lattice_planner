; Auto-generated. Do not edit!


(in-package infant_msgs-msg)


;//! \htmlinclude Velocity.msg.html

(defclass <Velocity> (ros-message)
  ((op_linear
    :reader op_linear-val
    :initarg :op_linear
    :type float
    :initform 0.0)
   (op_angular
    :reader op_angular-val
    :initarg :op_angular
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Velocity>) ostream)
  "Serializes a message object of type '<Velocity>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'op_linear))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'op_angular))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <Velocity>) istream)
  "Deserializes a message object of type '<Velocity>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'op_linear) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'op_angular) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Velocity>)))
  "Returns string type for a message object of type '<Velocity>"
  "infant_msgs/Velocity")
(defmethod md5sum ((type (eql '<Velocity>)))
  "Returns md5sum for a message object of type '<Velocity>"
  "b660e8545cc98e690e09bdff481a1679")
(defmethod message-definition ((type (eql '<Velocity>)))
  "Returns full string definition for message of type '<Velocity>"
  (format nil "float32 op_linear~%float32 op_angular~%~%~%"))
(defmethod serialization-length ((msg <Velocity>))
  (+ 0
     4
     4
))
(defmethod ros-message-to-list ((msg <Velocity>))
  "Converts a ROS message object to a list"
  (list '<Velocity>
    (cons ':op_linear (op_linear-val msg))
    (cons ':op_angular (op_angular-val msg))
))
