; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude Rotate-request.msg.html

(defclass <Rotate-request> (ros-message)
  ((angle
    :reader angle-val
    :initarg :angle
    :type float
    :initform 0.0)
   (velocity
    :reader velocity-val
    :initarg :velocity
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Rotate-request>) ostream)
  "Serializes a message object of type '<Rotate-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'velocity))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <Rotate-request>) istream)
  "Deserializes a message object of type '<Rotate-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Rotate-request>)))
  "Returns string type for a service object of type '<Rotate-request>"
  "serializer/RotateRequest")
(defmethod md5sum ((type (eql '<Rotate-request>)))
  "Returns md5sum for a message object of type '<Rotate-request>"
  "add030c9a3ec13fc469e071a948d4d33")
(defmethod message-definition ((type (eql '<Rotate-request>)))
  "Returns full string definition for message of type '<Rotate-request>"
  (format nil "float64 angle~%float64 velocity~%~%"))
(defmethod serialization-length ((msg <Rotate-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <Rotate-request>))
  "Converts a ROS message object to a list"
  (list '<Rotate-request>
    (cons ':angle (angle-val msg))
    (cons ':velocity (velocity-val msg))
))
;//! \htmlinclude Rotate-response.msg.html

(defclass <Rotate-response> (ros-message)
  ()
)
(defmethod serialize ((msg <Rotate-response>) ostream)
  "Serializes a message object of type '<Rotate-response>"
)
(defmethod deserialize ((msg <Rotate-response>) istream)
  "Deserializes a message object of type '<Rotate-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Rotate-response>)))
  "Returns string type for a service object of type '<Rotate-response>"
  "serializer/RotateResponse")
(defmethod md5sum ((type (eql '<Rotate-response>)))
  "Returns md5sum for a message object of type '<Rotate-response>"
  "add030c9a3ec13fc469e071a948d4d33")
(defmethod message-definition ((type (eql '<Rotate-response>)))
  "Returns full string definition for message of type '<Rotate-response>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <Rotate-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Rotate-response>))
  "Converts a ROS message object to a list"
  (list '<Rotate-response>
))
(defmethod service-request-type ((msg (eql 'Rotate)))
  '<Rotate-request>)
(defmethod service-response-type ((msg (eql 'Rotate)))
  '<Rotate-response>)
(defmethod ros-datatype ((msg (eql 'Rotate)))
  "Returns string type for a service object of type '<Rotate>"
  "serializer/Rotate")
