
(cl:in-package :asdf)

(defsystem "serial_stm32-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "uartUSB" :depends-on ("_package_uartUSB"))
    (:file "_package_uartUSB" :depends-on ("_package"))
  ))