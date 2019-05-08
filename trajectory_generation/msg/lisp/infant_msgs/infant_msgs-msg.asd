
(in-package :asdf)

(defsystem "infant_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "Pulse" :depends-on ("_package"))
    (:file "_package_Pulse" :depends-on ("_package"))
    (:file "Gps" :depends-on ("_package"))
    (:file "_package_Gps" :depends-on ("_package"))
    (:file "Velocity" :depends-on ("_package"))
    (:file "_package_Velocity" :depends-on ("_package"))
    ))
