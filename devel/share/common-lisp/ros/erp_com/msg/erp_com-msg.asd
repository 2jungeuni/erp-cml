
(cl:in-package :asdf)

(defsystem "erp_com-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Cmd" :depends-on ("_package_Cmd"))
    (:file "_package_Cmd" :depends-on ("_package"))
    (:file "Status" :depends-on ("_package_Status"))
    (:file "_package_Status" :depends-on ("_package"))
  ))