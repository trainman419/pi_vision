; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude GetAnalog-request.msg.html

(defclass <GetAnalog-request> (ros-message)
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
(defmethod serialize ((msg <GetAnalog-request>) ostream)
  "Serializes a message object of type '<GetAnalog-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'pin)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'cached) 1 0)) ostream)
)
(defmethod deserialize ((msg <GetAnalog-request>) istream)
  "Deserializes a message object of type '<GetAnalog-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'pin)) (read-byte istream))
  (setf (slot-value msg 'cached) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<GetAnalog-request>)))
  "Returns string type for a service object of type '<GetAnalog-request>"
  "serializer/GetAnalogRequest")
(defmethod md5sum ((type (eql '<GetAnalog-request>)))
  "Returns md5sum for a message object of type '<GetAnalog-request>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<GetAnalog-request>)))
  "Returns full string definition for message of type '<GetAnalog-request>"
  (format nil "uint8 pin~%bool cached~%~%"))
(defmethod serialization-length ((msg <GetAnalog-request>))
  (+ 0
     1
     1
))
(defmethod ros-message-to-list ((msg <GetAnalog-request>))
  "Converts a ROS message object to a list"
  (list '<GetAnalog-request>
    (cons ':pin (pin-val msg))
    (cons ':cached (cached-val msg))
))
;//! \htmlinclude GetAnalog-response.msg.html

(defclass <GetAnalog-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <GetAnalog-response>) ostream)
  "Serializes a message object of type '<GetAnalog-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <GetAnalog-response>) istream)
  "Deserializes a message object of type '<GetAnalog-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<GetAnalog-response>)))
  "Returns string type for a service object of type '<GetAnalog-response>"
  "serializer/GetAnalogResponse")
(defmethod md5sum ((type (eql '<GetAnalog-response>)))
  "Returns md5sum for a message object of type '<GetAnalog-response>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<GetAnalog-response>)))
  "Returns full string definition for message of type '<GetAnalog-response>"
  (format nil "uint8 value~%~%~%"))
(defmethod serialization-length ((msg <GetAnalog-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <GetAnalog-response>))
  "Converts a ROS message object to a list"
  (list '<GetAnalog-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'GetAnalog)))
  '<GetAnalog-request>)
(defmethod service-response-type ((msg (eql 'GetAnalog)))
  '<GetAnalog-response>)
(defmethod ros-datatype ((msg (eql 'GetAnalog)))
  "Returns string type for a service object of type '<GetAnalog>"
  "serializer/GetAnalog")
