; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude TravelDistance-request.msg.html

(defclass <TravelDistance-request> (ros-message)
  ((distance
    :reader distance-val
    :initarg :distance
    :type float
    :initform 0.0)
   (velocity
    :reader velocity-val
    :initarg :velocity
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <TravelDistance-request>) ostream)
  "Serializes a message object of type '<TravelDistance-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'distance))))
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
(defmethod deserialize ((msg <TravelDistance-request>) istream)
  "Deserializes a message object of type '<TravelDistance-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
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
(defmethod ros-datatype ((msg (eql '<TravelDistance-request>)))
  "Returns string type for a service object of type '<TravelDistance-request>"
  "serializer/TravelDistanceRequest")
(defmethod md5sum ((type (eql '<TravelDistance-request>)))
  "Returns md5sum for a message object of type '<TravelDistance-request>"
  "938fc8800cae9ac2a395d3178819b617")
(defmethod message-definition ((type (eql '<TravelDistance-request>)))
  "Returns full string definition for message of type '<TravelDistance-request>"
  (format nil "float64 distance~%float64 velocity~%~%"))
(defmethod serialization-length ((msg <TravelDistance-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <TravelDistance-request>))
  "Converts a ROS message object to a list"
  (list '<TravelDistance-request>
    (cons ':distance (distance-val msg))
    (cons ':velocity (velocity-val msg))
))
;//! \htmlinclude TravelDistance-response.msg.html

(defclass <TravelDistance-response> (ros-message)
  ()
)
(defmethod serialize ((msg <TravelDistance-response>) ostream)
  "Serializes a message object of type '<TravelDistance-response>"
)
(defmethod deserialize ((msg <TravelDistance-response>) istream)
  "Deserializes a message object of type '<TravelDistance-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<TravelDistance-response>)))
  "Returns string type for a service object of type '<TravelDistance-response>"
  "serializer/TravelDistanceResponse")
(defmethod md5sum ((type (eql '<TravelDistance-response>)))
  "Returns md5sum for a message object of type '<TravelDistance-response>"
  "938fc8800cae9ac2a395d3178819b617")
(defmethod message-definition ((type (eql '<TravelDistance-response>)))
  "Returns full string definition for message of type '<TravelDistance-response>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <TravelDistance-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <TravelDistance-response>))
  "Converts a ROS message object to a list"
  (list '<TravelDistance-response>
))
(defmethod service-request-type ((msg (eql 'TravelDistance)))
  '<TravelDistance-request>)
(defmethod service-response-type ((msg (eql 'TravelDistance)))
  '<TravelDistance-response>)
(defmethod ros-datatype ((msg (eql 'TravelDistance)))
  "Returns string type for a service object of type '<TravelDistance>"
  "serializer/TravelDistance")
