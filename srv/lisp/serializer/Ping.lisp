; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude Ping-request.msg.html

(defclass <Ping-request> (ros-message)
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
(defmethod serialize ((msg <Ping-request>) ostream)
  "Serializes a message object of type '<Ping-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'pin)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'cached) 1 0)) ostream)
)
(defmethod deserialize ((msg <Ping-request>) istream)
  "Deserializes a message object of type '<Ping-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'pin)) (read-byte istream))
  (setf (slot-value msg 'cached) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Ping-request>)))
  "Returns string type for a service object of type '<Ping-request>"
  "serializer/PingRequest")
(defmethod md5sum ((type (eql '<Ping-request>)))
  "Returns md5sum for a message object of type '<Ping-request>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<Ping-request>)))
  "Returns full string definition for message of type '<Ping-request>"
  (format nil "uint8 pin~%bool cached~%~%"))
(defmethod serialization-length ((msg <Ping-request>))
  (+ 0
     1
     1
))
(defmethod ros-message-to-list ((msg <Ping-request>))
  "Converts a ROS message object to a list"
  (list '<Ping-request>
    (cons ':pin (pin-val msg))
    (cons ':cached (cached-val msg))
))
;//! \htmlinclude Ping-response.msg.html

(defclass <Ping-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <Ping-response>) ostream)
  "Serializes a message object of type '<Ping-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <Ping-response>) istream)
  "Deserializes a message object of type '<Ping-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Ping-response>)))
  "Returns string type for a service object of type '<Ping-response>"
  "serializer/PingResponse")
(defmethod md5sum ((type (eql '<Ping-response>)))
  "Returns md5sum for a message object of type '<Ping-response>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<Ping-response>)))
  "Returns full string definition for message of type '<Ping-response>"
  (format nil "uint8 value~%~%~%"))
(defmethod serialization-length ((msg <Ping-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <Ping-response>))
  "Converts a ROS message object to a list"
  (list '<Ping-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'Ping)))
  '<Ping-request>)
(defmethod service-response-type ((msg (eql 'Ping)))
  '<Ping-response>)
(defmethod ros-datatype ((msg (eql 'Ping)))
  "Returns string type for a service object of type '<Ping>"
  "serializer/Ping")
