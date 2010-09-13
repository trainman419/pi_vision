; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude PhidgetsVoltage-request.msg.html

(defclass <PhidgetsVoltage-request> (ros-message)
  ((pin
    :reader pin-val
    :initarg :pin
    :type fixnum
    :initform 0)
   (cached
    :reader cached-val
    :initarg :cached
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <PhidgetsVoltage-request>) ostream)
  "Serializes a message object of type '<PhidgetsVoltage-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'pin)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'cached) 1 0)) ostream)
)
(defmethod deserialize ((msg <PhidgetsVoltage-request>) istream)
  "Deserializes a message object of type '<PhidgetsVoltage-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'pin)) (read-byte istream))
  (setf (slot-value msg 'cached) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<PhidgetsVoltage-request>)))
  "Returns string type for a service object of type '<PhidgetsVoltage-request>"
  "serializer/PhidgetsVoltageRequest")
(defmethod md5sum ((type (eql '<PhidgetsVoltage-request>)))
  "Returns md5sum for a message object of type '<PhidgetsVoltage-request>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<PhidgetsVoltage-request>)))
  "Returns full string definition for message of type '<PhidgetsVoltage-request>"
  (format nil "uint8 pin~%bool cached~%~%"))
(defmethod serialization-length ((msg <PhidgetsVoltage-request>))
  (+ 0
     1
     1
))
(defmethod ros-message-to-list ((msg <PhidgetsVoltage-request>))
  "Converts a ROS message object to a list"
  (list '<PhidgetsVoltage-request>
    (cons ':pin (pin-val msg))
    (cons ':cached (cached-val msg))
))
;//! \htmlinclude PhidgetsVoltage-response.msg.html

(defclass <PhidgetsVoltage-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <PhidgetsVoltage-response>) ostream)
  "Serializes a message object of type '<PhidgetsVoltage-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <PhidgetsVoltage-response>) istream)
  "Deserializes a message object of type '<PhidgetsVoltage-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PhidgetsVoltage-response>)))
  "Returns string type for a service object of type '<PhidgetsVoltage-response>"
  "serializer/PhidgetsVoltageResponse")
(defmethod md5sum ((type (eql '<PhidgetsVoltage-response>)))
  "Returns md5sum for a message object of type '<PhidgetsVoltage-response>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<PhidgetsVoltage-response>)))
  "Returns full string definition for message of type '<PhidgetsVoltage-response>"
  (format nil "uint8 value~%~%~%"))
(defmethod serialization-length ((msg <PhidgetsVoltage-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <PhidgetsVoltage-response>))
  "Converts a ROS message object to a list"
  (list '<PhidgetsVoltage-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'PhidgetsVoltage)))
  '<PhidgetsVoltage-request>)
(defmethod service-response-type ((msg (eql 'PhidgetsVoltage)))
  '<PhidgetsVoltage-response>)
(defmethod ros-datatype ((msg (eql 'PhidgetsVoltage)))
  "Returns string type for a service object of type '<PhidgetsVoltage>"
  "serializer/PhidgetsVoltage")
