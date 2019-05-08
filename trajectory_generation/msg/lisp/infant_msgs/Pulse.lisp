; Auto-generated. Do not edit!


(in-package infant_msgs-msg)


;//! \htmlinclude Pulse.msg.html

(defclass <Pulse> (ros-message)
  ((r_pulse
    :reader r_pulse-val
    :initarg :r_pulse
    :type integer
    :initform 0)
   (l_pulse
    :reader l_pulse-val
    :initarg :l_pulse
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <Pulse>) ostream)
  "Serializes a message object of type '<Pulse>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'r_pulse)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'r_pulse)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'r_pulse)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'r_pulse)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'l_pulse)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'l_pulse)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'l_pulse)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'l_pulse)) ostream)
)
(defmethod deserialize ((msg <Pulse>) istream)
  "Deserializes a message object of type '<Pulse>"
  (setf (ldb (byte 8 0) (slot-value msg 'r_pulse)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'r_pulse)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'r_pulse)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'r_pulse)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'l_pulse)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'l_pulse)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'l_pulse)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'l_pulse)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Pulse>)))
  "Returns string type for a message object of type '<Pulse>"
  "infant_msgs/Pulse")
(defmethod md5sum ((type (eql '<Pulse>)))
  "Returns md5sum for a message object of type '<Pulse>"
  "4ebab2b03f5d7cbb108066d9a3f07655")
(defmethod message-definition ((type (eql '<Pulse>)))
  "Returns full string definition for message of type '<Pulse>"
  (format nil "int32 r_pulse~%int32 l_pulse~%~%~%~%"))
(defmethod serialization-length ((msg <Pulse>))
  (+ 0
     4
     4
))
(defmethod ros-message-to-list ((msg <Pulse>))
  "Converts a ROS message object to a list"
  (list '<Pulse>
    (cons ':r_pulse (r_pulse-val msg))
    (cons ':l_pulse (l_pulse-val msg))
))
