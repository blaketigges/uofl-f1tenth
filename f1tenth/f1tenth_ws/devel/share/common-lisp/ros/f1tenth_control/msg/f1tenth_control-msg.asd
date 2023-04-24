
(cl:in-package :asdf)

(defsystem "f1tenth_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RaceInfo" :depends-on ("_package_RaceInfo"))
    (:file "_package_RaceInfo" :depends-on ("_package"))
  ))