; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude PhidgetsCurrent-request.msg.html

(defclass <PhidgetsCurrent-request> (ros-message)
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
(defmethod serialize ((msg <PhidgetsCurrent-request>) ostream)
  "Serializes a message object of type '<PhidgetsCurrent-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'pin)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'cached) 1 0)) ostream)
)
(defmethod deserialize ((msg <PhidgetsCurrent-request>) istream)
  "Deserializes a message object of type '<PhidgetsCurrent-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'pin)) (read-byte istream))
  (setf (slot-value msg 'cached) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<PhidgetsCurrent-request>)))
  "Returns string type for a service object of type '<PhidgetsCurrent-request>"
  "serializer/PhidgetsCurrentRequest")
(defmethod md5sum ((type (eql '<PhidgetsCurrent-request>)))
  "Returns md5sum for a message object of type '<PhidgetsCurrent-request>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<PhidgetsCurrent-request>)))
  "Returns full string definition for message of type '<PhidgetsCurrent-request>"
  (format nil "uint8 pin~%bool cached~%~%"))
(defmethod serialization-length ((msg <PhidgetsCurrent-request>))
  (+ 0
     1
     1
))
(defmethod ros-message-to-list ((msg <PhidgetsCurrent-request>))
  "Converts a ROS message object to a list"
  (list '<PhidgetsCurrent-request>
    (cons ':pin (pin-val msg))
    (cons ':cached (cached-val msg))
))
;//! \htmlinclude PhidgetsCurrent-response.msg.html

(defclass <PhidgetsCurrent-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <PhidgetsCurrent-response>) ostream)
  "Serializes a message object of type '<PhidgetsCurrent-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <PhidgetsCurrent-response>) istream)
  "Deserializes a message object of type '<PhidgetsCurrent-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PhidgetsCurrent-response>)))
  "Returns string type for a service object of type '<PhidgetsCurrent-response>"
  "serializer/PhidgetsCurrentResponse")
(defmethod md5sum ((type (eql '<PhidgetsCurrent-response>)))
  "Returns md5sum for a message object of type '<PhidgetsCurrent-response>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<PhidgetsCurrent-response>)))
  "Returns full string definition for message of type '<PhidgetsCurrent-response>"
  (format nil "uint8 value~%~%~%"))
(defmethod serialization-length ((msg <PhidgetsCurrent-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <PhidgetsCurrent-response>))
  "Converts a ROS message object to a list"
  (list '<PhidgetsCurrent-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'PhidgetsCurrent)))
  '<PhidgetsCurrent-request>)
(defmethod service-response-type ((msg (eql 'PhidgetsCurrent)))
  '<PhidgetsCurrent-response>)
(defmethod ros-datatype ((msg (eql 'PhidgetsCurrent)))
  "Returns string type for a service object of type '<PhidgetsCurrent>"
  "serializer/PhidgetsCurrent")
