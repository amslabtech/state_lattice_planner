; Auto-generated. Do not edit!


(in-package infant_msgs-msg)


;//! \htmlinclude Gps.msg.html

(defclass <Gps> (ros-message)
  ((lat
    :reader lat-val
    :initarg :lat
    :type float
    :initform 0.0)
   (lon
    :reader lon-val
    :initarg :lon
    :type float
    :initform 0.0)
   (precision
    :reader precision-val
    :initarg :precision
    :type integer
    :initform 0)
   (satelites
    :reader satelites-val
    :initarg :satelites
    :type integer
    :initform 0)
   (hdop
    :reader hdop-val
    :initarg :hdop
    :type float
    :initform 0.0)
   (alt
    :reader alt-val
    :initarg :alt
    :type float
    :initform 0.0)
   (geo
    :reader geo-val
    :initarg :geo
    :type float
    :initform 0.0)
   (lack
    :reader lack-val
    :initarg :lack
    :type integer
    :initform 0)
   (rms
    :reader rms-val
    :initarg :rms
    :type float
    :initform 0.0)
   (smd1
    :reader smd1-val
    :initarg :smd1
    :type float
    :initform 0.0)
   (smd2
    :reader smd2-val
    :initarg :smd2
    :type float
    :initform 0.0)
   (smo
    :reader smo-val
    :initarg :smo
    :type float
    :initform 0.0)
   (laed
    :reader laed-val
    :initarg :laed
    :type float
    :initform 0.0)
   (loed
    :reader loed-val
    :initarg :loed
    :type float
    :initform 0.0)
   (aled
    :reader aled-val
    :initarg :aled
    :type float
    :initform 0.0)
   (pdop
    :reader pdop-val
    :initarg :pdop
    :type float
    :initform 0.0)
   (vdop
    :reader vdop-val
    :initarg :vdop
    :type float
    :initform 0.0)
   (gs
    :reader gs-val
    :initarg :gs
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Gps>) ostream)
  "Serializes a message object of type '<Gps>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'lat))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'lon))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'precision)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'precision)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'precision)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'precision)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'satelites)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'satelites)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'satelites)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'satelites)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'hdop))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'alt))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'geo))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'lack)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'lack)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'lack)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'lack)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'rms))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'smd1))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'smd2))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'smo))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'laed))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'loed))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'aled))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pdop))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'vdop))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'gs))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <Gps>) istream)
  "Deserializes a message object of type '<Gps>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'lat) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'lon) (roslisp-utils:decode-double-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'precision)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'precision)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'precision)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'precision)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'satelites)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'satelites)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'satelites)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'satelites)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'hdop) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'alt) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'geo) (roslisp-utils:decode-double-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'lack)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'lack)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'lack)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'lack)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'rms) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'smd1) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'smd2) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'smo) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'laed) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'loed) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'aled) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pdop) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'vdop) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'gs) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Gps>)))
  "Returns string type for a message object of type '<Gps>"
  "infant_msgs/Gps")
(defmethod md5sum ((type (eql '<Gps>)))
  "Returns md5sum for a message object of type '<Gps>"
  "7e02ac686b60763eac2c019c7ca7d0b0")
(defmethod message-definition ((type (eql '<Gps>)))
  "Returns full string definition for message of type '<Gps>"
  (format nil "#GGA~%float64 lat	~%float64 lon	~%int32 precision	~%int32 satelites	#The number of Satelites~%float32 hdop	#Horizontal dilution of position	~%float64 alt 	#Altitude(Meters) above mean sea level	~%float64 geo	#Height of geoid (mean sea level) above WGS84 	~%int32 lack	#Time in seconds since last DGPS update	~%	~%#GST~%float32 rms	#Total RMS standard deviation of ranges inputs to the navigation solution~%float32 smd1	#Standard deviation1 (meters) of semi-major axis of error ellipse~%float32 smd2	#Standard deviation2 (meters) of semi-major axis of error ellipse~%float32 smo	#Orientation Orientation of semi-major axis of error ellipse (true north degrees)~%float32 laed	#Standard deviation (meters) of latitude error~%float32 loed	#Standard deviation (meters) of longitude error~%float32 aled	#Standard deviation (meters) of Altitude error~%~%#GSA~%float32 pdop	#Position Dilution of Precision (PDOP)~%float32 vdop	#Vertical Dilution of Precision (VDOP)~%~%~%#VTG~%float32 gs	#Ground speed~%~%~%~%"))
(defmethod serialization-length ((msg <Gps>))
  (+ 0
     8
     8
     4
     4
     4
     8
     8
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <Gps>))
  "Converts a ROS message object to a list"
  (list '<Gps>
    (cons ':lat (lat-val msg))
    (cons ':lon (lon-val msg))
    (cons ':precision (precision-val msg))
    (cons ':satelites (satelites-val msg))
    (cons ':hdop (hdop-val msg))
    (cons ':alt (alt-val msg))
    (cons ':geo (geo-val msg))
    (cons ':lack (lack-val msg))
    (cons ':rms (rms-val msg))
    (cons ':smd1 (smd1-val msg))
    (cons ':smd2 (smd2-val msg))
    (cons ':smo (smo-val msg))
    (cons ':laed (laed-val msg))
    (cons ':loed (loed-val msg))
    (cons ':aled (aled-val msg))
    (cons ':pdop (pdop-val msg))
    (cons ':vdop (vdop-val msg))
    (cons ':gs (gs-val msg))
))
