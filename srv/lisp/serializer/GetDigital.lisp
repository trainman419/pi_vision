; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude GetDigital-request.msg.html

(defclass <GetDigital-request> (ros-message)
  ((pin
    :reader pin-val
    :initarg :pin
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <GetDigital-request>) ostream)
  "Serializes a message object of type '<GetDigital-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'pin)) ostream)
)
(defmethod deserialize ((msg <GetDigital-request>) istream)
  "Deserializes a message object of type '<GetDigital-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'pin)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<GetDigital-request>)))
  "Returns string type for a service object of type '<GetDigital-request>"
  "serializer/GetDigitalRequest")
(defmethod md5sum ((type (eql '<GetDigital-request>)))
  "Returns md5sum for a message object of type '<GetDigital-request>"
  "ad28e4611c3edea82d59f9c3743bc9b7")
(defmethod message-definition ((type (eql '<GetDigital-request>)))
  "Returns full string definition for message of type '<GetDigital-request>"
  (format nil "uint8 pin~%~%"))
(defmethod serialization-length ((msg <GetDigital-request>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <GetDigital-request>))
  "Converts a ROS message object to a list"
  (list '<GetDigital-request>
    (cons ':pin (pin-val msg))
))
;//! \htmlinclude GetDigital-response.msg.html

(defclass <GetDigital-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <GetDigital-response>) ostream)
  "Serializes a message object of type '<GetDigital-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <GetDigital-response>) istream)
  "Deserializes a message object of type '<GetDigital-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<GetDigital-response>)))
  "Returns string type for a service object of type '<GetDigital-response>"
  "serializer/GetDigitalResponse")
(defmethod md5sum ((type (eql '<GetDigital-response>)))
  "Returns md5sum for a message object of type '<GetDigital-response>"
  "ad28e4611c3edea82d59f9c3743bc9b7")
(defmethod message-definition ((type (eql '<GetDigital-response>)))
  "Returns full string definition for message of type '<GetDigital-response>"
  (format nil "uint8 value~%~%~%"))
(defmethod serialization-length ((msg <GetDigital-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <GetDigital-response>))
  "Converts a ROS message object to a list"
  (list '<GetDigital-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'GetDigital)))
  '<GetDigital-request>)
(defmethod service-response-type ((msg (eql 'GetDigital)))
  '<GetDigital-response>)
(defmethod ros-datatype ((msg (eql 'GetDigital)))
  "Returns string type for a service object of type '<GetDigital>"
  "serializer/GetDigital")
