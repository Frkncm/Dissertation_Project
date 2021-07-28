;; Auto-generated. Do not edit!


(when (boundp 'my_example_pkg::my_services)
  (if (not (find-package "MY_EXAMPLE_PKG"))
    (make-package "MY_EXAMPLE_PKG"))
  (shadow 'my_services (find-package "MY_EXAMPLE_PKG")))
(unless (find-package "MY_EXAMPLE_PKG::MY_SERVICES")
  (make-package "MY_EXAMPLE_PKG::MY_SERVICES"))
(unless (find-package "MY_EXAMPLE_PKG::MY_SERVICESREQUEST")
  (make-package "MY_EXAMPLE_PKG::MY_SERVICESREQUEST"))
(unless (find-package "MY_EXAMPLE_PKG::MY_SERVICESRESPONSE")
  (make-package "MY_EXAMPLE_PKG::MY_SERVICESRESPONSE"))

(in-package "ROS")





(defclass my_example_pkg::my_servicesRequest
  :super ros::object
  :slots (_number ))

(defmethod my_example_pkg::my_servicesRequest
  (:init
   (&key
    ((:number __number) 0)
    )
   (send-super :init)
   (setq _number (round __number))
   self)
  (:number
   (&optional __number)
   (if __number (setq _number __number)) _number)
  (:serialization-length
   ()
   (+
    ;; int32 _number
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _number
       (write-long _number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _number
     (setq _number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass my_example_pkg::my_servicesResponse
  :super ros::object
  :slots (_answer ))

(defmethod my_example_pkg::my_servicesResponse
  (:init
   (&key
    ((:answer __answer) "")
    )
   (send-super :init)
   (setq _answer (string __answer))
   self)
  (:answer
   (&optional __answer)
   (if __answer (setq _answer __answer)) _answer)
  (:serialization-length
   ()
   (+
    ;; string _answer
    4 (length _answer)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _answer
       (write-long (length _answer) s) (princ _answer s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _answer
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _answer (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass my_example_pkg::my_services
  :super ros::object
  :slots ())

(setf (get my_example_pkg::my_services :md5sum-) "4ff0b0ab1ed04611e3a2b4090af9ee4f")
(setf (get my_example_pkg::my_services :datatype-) "my_example_pkg/my_services")
(setf (get my_example_pkg::my_services :request) my_example_pkg::my_servicesRequest)
(setf (get my_example_pkg::my_services :response) my_example_pkg::my_servicesResponse)

(defmethod my_example_pkg::my_servicesRequest
  (:response () (instance my_example_pkg::my_servicesResponse :init)))

(setf (get my_example_pkg::my_servicesRequest :md5sum-) "4ff0b0ab1ed04611e3a2b4090af9ee4f")
(setf (get my_example_pkg::my_servicesRequest :datatype-) "my_example_pkg/my_servicesRequest")
(setf (get my_example_pkg::my_servicesRequest :definition-)
      "#First parameters above the line will be request and second one will be return messages
int32 number
---
string answer
")

(setf (get my_example_pkg::my_servicesResponse :md5sum-) "4ff0b0ab1ed04611e3a2b4090af9ee4f")
(setf (get my_example_pkg::my_servicesResponse :datatype-) "my_example_pkg/my_servicesResponse")
(setf (get my_example_pkg::my_servicesResponse :definition-)
      "#First parameters above the line will be request and second one will be return messages
int32 number
---
string answer
")



(provide :my_example_pkg/my_services "4ff0b0ab1ed04611e3a2b4090af9ee4f")


