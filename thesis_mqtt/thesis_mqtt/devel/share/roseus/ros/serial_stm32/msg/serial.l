;; Auto-generated. Do not edit!


(when (boundp 'serial_stm32::serial)
  (if (not (find-package "SERIAL_STM32"))
    (make-package "SERIAL_STM32"))
  (shadow 'serial (find-package "SERIAL_STM32")))
(unless (find-package "SERIAL_STM32::SERIAL")
  (make-package "SERIAL_STM32::SERIAL"))

(in-package "ROS")
;;//! \htmlinclude serial.msg.html


(defclass serial_stm32::serial
  :super ros::object
  :slots (_dir1 _dir2 _dir3 ))

(defmethod serial_stm32::serial
  (:init
   (&key
    ((:dir1 __dir1) "")
    ((:dir2 __dir2) "")
    ((:dir3 __dir3) "")
    )
   (send-super :init)
   (setq _dir1 (string __dir1))
   (setq _dir2 (string __dir2))
   (setq _dir3 (string __dir3))
   self)
  (:dir1
   (&optional __dir1)
   (if __dir1 (setq _dir1 __dir1)) _dir1)
  (:dir2
   (&optional __dir2)
   (if __dir2 (setq _dir2 __dir2)) _dir2)
  (:dir3
   (&optional __dir3)
   (if __dir3 (setq _dir3 __dir3)) _dir3)
  (:serialization-length
   ()
   (+
    ;; string _dir1
    4 (length _dir1)
    ;; string _dir2
    4 (length _dir2)
    ;; string _dir3
    4 (length _dir3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _dir1
       (write-long (length _dir1) s) (princ _dir1 s)
     ;; string _dir2
       (write-long (length _dir2) s) (princ _dir2 s)
     ;; string _dir3
       (write-long (length _dir3) s) (princ _dir3 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _dir1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dir1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _dir2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dir2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _dir3
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dir3 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get serial_stm32::serial :md5sum-) "00bcf444a943c3a48aff860ffd0aed64")
(setf (get serial_stm32::serial :datatype-) "serial_stm32/serial")
(setf (get serial_stm32::serial :definition-)
      "string dir1
string dir2
string dir3
")



(provide :serial_stm32/serial "00bcf444a943c3a48aff860ffd0aed64")


