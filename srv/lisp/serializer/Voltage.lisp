; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude Voltage-request.msg.html

(defclass <Voltage-request> (ros-message)
  ((cached
    :reader cached-val
    :initarg :cached
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <Voltage-request>) ostream)
  "Serializes a message object of type '<Voltage-request>"
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'cached) 1 0)) ostream)
)
(defmethod deserialize ((msg <Voltage-request>) istream)
  "Deserializes a message object of type '<Voltage-request>"
  (setf (slot-value msg 'cached) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Voltage-request>)))
  "Returns string type for a service object of type '<Voltage-request>"
  "serializer/VoltageRequest")
(defmethod md5sum ((type (eql '<Voltage-request>)))
  "Returns md5sum for a message object of type '<Voltage-request>"
  "e83d0a14c1ccae2cbe9da2c6c6a2148f")
(defmethod message-definition ((type (eql '<Voltage-request>)))
  "Returns full string definition for message of type '<Voltage-request>"
  (format nil "bool cached~%~%"))
(defmethod serialization-length ((msg <Voltage-request>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <Voltage-request>))
  "Converts a ROS message object to a list"
  (list '<Voltage-request>
    (cons ':cached (cached-val msg))
))
;//! \htmlinclude Voltage-response.msg.html

(defclass <Voltage-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Voltage-response>) ostream)
  "Serializes a message object of type '<Voltage-response>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'value))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <Voltage-response>) istream)
  "Deserializes a message object of type '<Voltage-response>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Voltage-response>)))
  "Returns string type for a service object of type '<Voltage-response>"
  "serializer/VoltageResponse")
(defmethod md5sum ((type (eql '<Voltage-response>)))
  "Returns md5sum for a message object of type '<Voltage-response>"
  "e83d0a14c1ccae2cbe9da2c6c6a2148f")
(defmethod message-definition ((type (eql '<Voltage-response>)))
  "Returns full string definition for message of type '<Voltage-response>"
  (format nil "float64 value~%~%~%"))
(defmethod serialization-length ((msg <Voltage-response>))
  (+ 0
     8
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
