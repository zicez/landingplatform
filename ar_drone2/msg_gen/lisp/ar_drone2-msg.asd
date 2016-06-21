
(cl:in-package :asdf)

(defsystem "ar_drone2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Navdata" :depends-on ("_package_Navdata"))
    (:file "_package_Navdata" :depends-on ("_package"))
  ))