; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude PhidgetsTemperature-request.msg.html

(defclass <PhidgetsTemperature-request> (ros-message)
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
(defmethod serialize ((msg <PhidgetsTemperature-request>) ostream)
  "Serializes a message object of type '<PhidgetsTemperature-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'pin)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'cached) 1 0)) ostream)
)
(defmethod deserialize ((msg <PhidgetsTemperature-request>) istream)
  "Deserializes a message object of type '<PhidgetsTemperature-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'pin)) (read-byte istream))
  (setf (slot-value msg 'cached) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<PhidgetsTemperature-request>)))
  "Returns string type for a service object of type '<PhidgetsTemperature-request>"
  "serializer/PhidgetsTemperatureRequest")
(defmethod md5sum ((type (eql '<PhidgetsTemperature-request>)))
  "Returns md5sum for a message object of type '<PhidgetsTemperature-request>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<PhidgetsTemperature-request>)))
  "Returns full string definition for message of type '<PhidgetsTemperature-request>"
  (format nil "uint8 pin~%bool cached~%~%"))
(defmethod serialization-length ((msg <PhidgetsTemperature-request>))
  (+ 0
     1
     1
))
(defmethod ros-message-to-list ((msg <PhidgetsTemperature-request>))
  "Converts a ROS message object to a list"
  (list '<PhidgetsTemperature-request>
    (cons ':pin (pin-val msg))
    (cons ':cached (cached-val msg))
))
;//! \htmlinclude PhidgetsTemperature-response.msg.html

(defclass <PhidgetsTemperature-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <PhidgetsTemperature-response>) ostream)
  "Serializes a message object of type '<PhidgetsTemperature-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <PhidgetsTemperature-response>) istream)
  "Deserializes a message object of type '<PhidgetsTemperature-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PhidgetsTemperature-response>)))
  "Returns string type for a service object of type '<PhidgetsTemperature-response>"
  "serializer/PhidgetsTemperatureResponse")
(defmethod md5sum ((type (eql '<PhidgetsTemperature-response>)))
  "Returns md5sum for a message object of type '<PhidgetsTemperature-response>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<PhidgetsTemperature-response>)))
  "Returns full string definition for message of type '<PhidgetsTemperature-response>"
  (format nil "uint8 value~%~%~%"))
(defmethod serialization-length ((msg <PhidgetsTemperature-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <PhidgetsTemperature-response>))
  "Converts a ROS message object to a list"
  (list '<PhidgetsTemperature-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'PhidgetsTemperature)))
  '<PhidgetsTemperature-request>)
(defmethod service-response-type ((msg (eql 'PhidgetsTemperature)))
  '<PhidgetsTemperature-response>)
(defmethod ros-datatype ((msg (eql 'PhidgetsTemperature)))
  "Returns string type for a service object of type '<PhidgetsTemperature>"
  "serializer/PhidgetsTemperature")
