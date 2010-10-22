
(in-package :asdf)

(defsystem "serializer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "SensorState" :depends-on ("_package"))
    (:file "_package_SensorState" :depends-on ("_package"))
    (:file "Sonar" :depends-on ("_package"))
    (:file "_package_Sonar" :depends-on ("_package"))
    ))
