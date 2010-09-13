; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude GP2D12-request.msg.html

(defclass <GP2D12-request> (ros-message)
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
(defmethod serialize ((msg <GP2D12-request>) ostream)
  "Serializes a message object of type '<GP2D12-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'pin)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'cached) 1 0)) ostream)
)
(defmethod deserialize ((msg <GP2D12-request>) istream)
  "Deserializes a message object of type '<GP2D12-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'pin)) (read-byte istream))
  (setf (slot-value msg 'cached) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<GP2D12-request>)))
  "Returns string type for a service object of type '<GP2D12-request>"
  "serializer/GP2D12Request")
(defmethod md5sum ((type (eql '<GP2D12-request>)))
  "Returns md5sum for a message object of type '<GP2D12-request>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<GP2D12-request>)))
  "Returns full string definition for message of type '<GP2D12-request>"
  (format nil "uint8 pin~%bool cached~%~%"))
(defmethod serialization-length ((msg <GP2D12-request>))
  (+ 0
     1
     1
))
(defmethod ros-message-to-list ((msg <GP2D12-request>))
  "Converts a ROS message object to a list"
  (list '<GP2D12-request>
    (cons ':pin (pin-val msg))
    (cons ':cached (cached-val msg))
))
;//! \htmlinclude GP2D12-response.msg.html

(defclass <GP2D12-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <GP2D12-response>) ostream)
  "Serializes a message object of type '<GP2D12-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <GP2D12-response>) istream)
  "Deserializes a message object of type '<GP2D12-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<GP2D12-response>)))
  "Returns string type for a service object of type '<GP2D12-response>"
  "serializer/GP2D12Response")
(defmethod md5sum ((type (eql '<GP2D12-response>)))
  "Returns md5sum for a message object of type '<GP2D12-response>"
  "4e943b13b25d974978eb5df74257558f")
(defmethod message-definition ((type (eql '<GP2D12-response>)))
  "Returns full string definition for message of type '<GP2D12-response>"
  (format nil "uint8 value~%~%~%"))
(defmethod serialization-length ((msg <GP2D12-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <GP2D12-response>))
  "Converts a ROS message object to a list"
  (list '<GP2D12-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'GP2D12)))
  '<GP2D12-request>)
(defmethod service-response-type ((msg (eql 'GP2D12)))
  '<GP2D12-response>)
(defmethod ros-datatype ((msg (eql 'GP2D12)))
  "Returns string type for a service object of type '<GP2D12>"
  "serializer/GP2D12")
