; Auto-generated. Do not edit!


(in-package serializer-msg)


;//! \htmlinclude Sonar.msg.html

(defclass <Sonar> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (distance
    :reader distance-val
    :initarg :distance
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <Sonar>) ostream)
  "Serializes a message object of type '<Sonar>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'distance)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'distance)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'distance)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'distance)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'distance)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'distance)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'distance)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'distance)) ostream)
)
(defmethod deserialize ((msg <Sonar>) istream)
  "Deserializes a message object of type '<Sonar>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'distance)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'distance)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'distance)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'distance)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'distance)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'distance)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'distance)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'distance)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Sonar>)))
  "Returns string type for a message object of type '<Sonar>"
  "serializer/Sonar")
(defmethod md5sum ((type (eql '<Sonar>)))
  "Returns md5sum for a message object of type '<Sonar>"
  "17c09c1bd56db95833c058c359e09ee5")
(defmethod message-definition ((type (eql '<Sonar>)))
  "Returns full string definition for message of type '<Sonar>"
  (format nil "Header header~%int64 distance~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Sonar>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     8
))
(defmethod ros-message-to-list ((msg <Sonar>))
  "Converts a ROS message object to a list"
  (list '<Sonar>
    (cons ':header (header-val msg))
    (cons ':distance (distance-val msg))
))
