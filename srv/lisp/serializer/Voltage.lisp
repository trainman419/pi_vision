; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude Voltage-request.msg.html

(defclass <Voltage-request> (ros-message)
  ()
)
(defmethod serialize ((msg <Voltage-request>) ostream)
  "Serializes a message object of type '<Voltage-request>"
)
(defmethod deserialize ((msg <Voltage-request>) istream)
  "Deserializes a message object of type '<Voltage-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Voltage-request>)))
  "Returns string type for a service object of type '<Voltage-request>"
  "serializer/VoltageRequest")
(defmethod md5sum ((type (eql '<Voltage-request>)))
  "Returns md5sum for a message object of type '<Voltage-request>"
  "e4da51e86d3bac963ee2bb1f41031407")
(defmethod message-definition ((type (eql '<Voltage-request>)))
  "Returns full string definition for message of type '<Voltage-request>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <Voltage-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Voltage-request>))
  "Converts a ROS message object to a list"
  (list '<Voltage-request>
))
;//! \htmlinclude Voltage-response.msg.html

(defclass <Voltage-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <Voltage-response>) ostream)
  "Serializes a message object of type '<Voltage-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <Voltage-response>) istream)
  "Deserializes a message object of type '<Voltage-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Voltage-response>)))
  "Returns string type for a service object of type '<Voltage-response>"
  "serializer/VoltageResponse")
(defmethod md5sum ((type (eql '<Voltage-response>)))
  "Returns md5sum for a message object of type '<Voltage-response>"
  "e4da51e86d3bac963ee2bb1f41031407")
(defmethod message-definition ((type (eql '<Voltage-response>)))
  "Returns full string definition for message of type '<Voltage-response>"
  (format nil "uint8 value~%~%~%"))
(defmethod serialization-length ((msg <Voltage-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <Voltage-response>))
  "Converts a ROS message object to a list"
  (list '<Voltage-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'Voltage)))
  '<Voltage-request>)
(defmethod service-response-type ((msg (eql 'Voltage)))
  '<Voltage-response>)
(defmethod ros-datatype ((msg (eql 'Voltage)))
  "Returns string type for a service object of type '<Voltage>"
  "serializer/Voltage")
