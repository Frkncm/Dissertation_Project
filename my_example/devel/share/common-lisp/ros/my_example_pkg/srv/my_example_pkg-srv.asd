
(cl:in-package :asdf)

(defsystem "my_example_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "my_services" :depends-on ("_package_my_services"))
    (:file "_package_my_services" :depends-on ("_package"))
  ))