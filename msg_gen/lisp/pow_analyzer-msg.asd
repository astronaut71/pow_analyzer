
(cl:in-package :asdf)

(defsystem "pow_analyzer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "Num1" :depends-on ("_package_Num1"))
    (:file "_package_Num1" :depends-on ("_package"))
  ))