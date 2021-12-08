
(cl:in-package :asdf)

(defsystem "wtr_serial-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "actor_1" :depends-on ("_package_actor_1"))
    (:file "_package_actor_1" :depends-on ("_package"))
  ))