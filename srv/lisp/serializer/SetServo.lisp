; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude SetServo-request.msg.html

(defclass <SetServo-request> (ros-message)
  ((id
    :reader id-val
    :initarg :id
    :type fixnum
    :initform 0)
   (value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <SetServo-request>) ostream)
  "Serializes a message object of type '<SetServo-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'id)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <SetServo-request>) istream)
  "Deserializes a message object of type '<SetServo-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<SetServo-request>)))
  "Returns string type for a service object of type '<SetServo-request>"
  "serializer/SetServoRequest")
(defmethod md5sum ((type (eql '<SetServo-request>)))
  "Returns md5sum for a message object of type '<SetServo-request>"
  "72b7969bdddd9940a4e63f5c59126060")
(defmethod message-definition ((type (eql '<SetServo-request>)))
  "Returns full string definition for message of type '<SetServo-request>"
  (format nil "uint8 id~%int8 value~%~%"))
(defmethod serialization-length ((msg <SetServo-request>))
  (+ 0
     1
     1
))
(defmethod ros-message-to-list ((msg <SetServo-request>))
  "Converts a ROS message object to a list"
  (list '<SetServo-request>
    (cons ':id (id-val msg))
    (cons ':value (value-val msg))
))
;//! \htmlinclude SetServo-response.msg.html

(defclass <SetServo-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SetServo-response>) ostream)
  "Serializes a message object of type '<SetServo-response>"
)
(defmethod deserialize ((msg <SetServo-response>) istream)
  "Deserializes a message object of type '<SetServo-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SetServo-response>)))
  "Returns string type for a service object of type '<SetServo-response>"
  "serializer/SetServoResponse")
(defmethod md5sum ((type (eql '<SetServo-response>)))
  "Returns md5sum for a message object of type '<SetServo-response>"
  "72b7969bdddd9940a4e63f5c59126060")
(defmethod message-definition ((type (eql '<SetServo-response>)))
  "Returns full string definition for message of type '<SetServo-response>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <SetServo-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SetServo-response>))
  "Converts a ROS message object to a list"
  (list '<SetServo-response>
))
(defmethod service-request-type ((msg (eql 'SetServo)))
  '<SetServo-request>)
(defmethod service-response-type ((msg (eql 'SetServo)))
  '<SetServo-response>)
(defmethod ros-datatype ((msg (eql 'SetServo)))
  "Returns string type for a service object of type '<SetServo>"
  "serializer/SetServo")
