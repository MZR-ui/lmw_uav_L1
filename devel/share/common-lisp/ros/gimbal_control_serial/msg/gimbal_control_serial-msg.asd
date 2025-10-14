
(cl:in-package :asdf)

(defsystem "gimbal_control_serial-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GimbalCmd" :depends-on ("_package_GimbalCmd"))
    (:file "_package_GimbalCmd" :depends-on ("_package"))
  ))