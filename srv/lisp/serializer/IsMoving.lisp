; Auto-generated. Do not edit!


(in-package serializer-srv)


;//! \htmlinclude IsMoving-request.msg.html

(defclass <IsMoving-request> (ros-message)
  ()
)
(defmethod serialize ((msg <IsMoving-request>) ostream)
  "Serializes a message object of type '<IsMoving-request>"
)
(defmethod deserialize ((msg <IsMoving-request>) istream)
  "Deserializes a message object of type '<IsMoving-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<IsMoving-request>)))
  "Returns string type for a service object of type '<IsMoving-request>"
  "serializer/IsMovingRequest")
(defmethod md5sum ((type (eql '<IsMoving-request>)))
  "Returns md5sum for a message object of type '<IsMoving-request>"
  "9104f1a32b4fbf4d3c8c80d9b9493250")
(defmethod message-definition ((type (eql '<IsMoving-request>)))
  "Returns full string definition for message of type '<IsMoving-request>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <IsMoving-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <IsMoving-request>))
  "Converts a ROS message object to a list"
  (list '<IsMoving-request>
))
;//! \htmlinclude IsMoving-response.msg.html

(defclass <IsMoving-response> (ros-message)
  ((moving
    :reader moving-val
    :initarg :moving
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <IsMoving-response>) ostream)
  "Serializes a message object of type '<IsMoving-response>"
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'moving) 1 0)) ostream)
)
(defmethod deserialize ((msg <IsMoving-response>) istream)
  "Deserializes a message object of type '<IsMoving-response>"
  (setf (slot-value msg 'moving) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<IsMoving-response>)))
  "Returns string type for a service object of type '<IsMoving-response>"
  "serializer/IsMovingResponse")
(defmethod md5sum ((type (eql '<IsMoving-response>)))
  "Returns md5sum for a message object of type '<IsMoving-response>"
  "9104f1a32b4fbf4d3c8c80d9b9493250")
(defmethod message-definition ((type (eql '<IsMoving-response>)))
  "Returns full string definition for message of type '<IsMoving-response>"
  (format nil "bool moving~%~%~%"))
(defmethod serialization-length ((msg <IsMoving-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <IsMoving-response>))
  "Converts a ROS message object to a list"
  (list '<IsMoving-response>
    (cons ':moving (moving-val msg))
))
(defmethod service-request-type ((msg (eql 'IsMoving)))
  '<IsMoving-request>)
(defmethod service-response-type ((msg (eql 'IsMoving)))
  '<IsMoving-response>)
(defmethod ros-datatype ((msg (eql 'IsMoving)))
  "Returns string type for a service object of type '<IsMoving>"
  "serializer/IsMoving")
