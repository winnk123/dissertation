
(cl:in-package :asdf)

(defsystem "multirobot_map_merge-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "mapPair2tf" :depends-on ("_package_mapPair2tf"))
    (:file "_package_mapPair2tf" :depends-on ("_package"))
  ))