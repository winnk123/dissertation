; Auto-generated. Do not edit!


(cl:in-package multirobot_map_merge-srv)


;//! \htmlinclude mapPair2tf-request.msg.html

(cl:defclass <mapPair2tf-request> (roslisp-msg-protocol:ros-message)
  ((grid1
    :reader grid1
    :initarg :grid1
    :type nav_msgs-msg:OccupancyGrid
    :initform (cl:make-instance 'nav_msgs-msg:OccupancyGrid))
   (grid2
    :reader grid2
    :initarg :grid2
    :type nav_msgs-msg:OccupancyGrid
    :initform (cl:make-instance 'nav_msgs-msg:OccupancyGrid)))
)

(cl:defclass mapPair2tf-request (<mapPair2tf-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mapPair2tf-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mapPair2tf-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multirobot_map_merge-srv:<mapPair2tf-request> is deprecated: use multirobot_map_merge-srv:mapPair2tf-request instead.")))

(cl:ensure-generic-function 'grid1-val :lambda-list '(m))
(cl:defmethod grid1-val ((m <mapPair2tf-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_map_merge-srv:grid1-val is deprecated.  Use multirobot_map_merge-srv:grid1 instead.")
  (grid1 m))

(cl:ensure-generic-function 'grid2-val :lambda-list '(m))
(cl:defmethod grid2-val ((m <mapPair2tf-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_map_merge-srv:grid2-val is deprecated.  Use multirobot_map_merge-srv:grid2 instead.")
  (grid2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mapPair2tf-request>) ostream)
  "Serializes a message object of type '<mapPair2tf-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grid1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grid2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mapPair2tf-request>) istream)
  "Deserializes a message object of type '<mapPair2tf-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grid1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grid2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mapPair2tf-request>)))
  "Returns string type for a service object of type '<mapPair2tf-request>"
  "multirobot_map_merge/mapPair2tfRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mapPair2tf-request)))
  "Returns string type for a service object of type 'mapPair2tf-request"
  "multirobot_map_merge/mapPair2tfRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mapPair2tf-request>)))
  "Returns md5sum for a message object of type '<mapPair2tf-request>"
  "6fa85f9e352b2ce5cb5617042231d419")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mapPair2tf-request)))
  "Returns md5sum for a message object of type 'mapPair2tf-request"
  "6fa85f9e352b2ce5cb5617042231d419")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mapPair2tf-request>)))
  "Returns full string definition for message of type '<mapPair2tf-request>"
  (cl:format cl:nil "nav_msgs/OccupancyGrid grid1~%nav_msgs/OccupancyGrid grid2~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mapPair2tf-request)))
  "Returns full string definition for message of type 'mapPair2tf-request"
  (cl:format cl:nil "nav_msgs/OccupancyGrid grid1~%nav_msgs/OccupancyGrid grid2~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mapPair2tf-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grid1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grid2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mapPair2tf-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mapPair2tf-request
    (cl:cons ':grid1 (grid1 msg))
    (cl:cons ':grid2 (grid2 msg))
))
;//! \htmlinclude mapPair2tf-response.msg.html

(cl:defclass <mapPair2tf-response> (roslisp-msg-protocol:ros-message)
  ((transform
    :reader transform
    :initarg :transform
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass mapPair2tf-response (<mapPair2tf-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mapPair2tf-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mapPair2tf-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multirobot_map_merge-srv:<mapPair2tf-response> is deprecated: use multirobot_map_merge-srv:mapPair2tf-response instead.")))

(cl:ensure-generic-function 'transform-val :lambda-list '(m))
(cl:defmethod transform-val ((m <mapPair2tf-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_map_merge-srv:transform-val is deprecated.  Use multirobot_map_merge-srv:transform instead.")
  (transform m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <mapPair2tf-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_map_merge-srv:confidence-val is deprecated.  Use multirobot_map_merge-srv:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mapPair2tf-response>) ostream)
  "Serializes a message object of type '<mapPair2tf-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transform) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mapPair2tf-response>) istream)
  "Deserializes a message object of type '<mapPair2tf-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transform) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mapPair2tf-response>)))
  "Returns string type for a service object of type '<mapPair2tf-response>"
  "multirobot_map_merge/mapPair2tfResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mapPair2tf-response)))
  "Returns string type for a service object of type 'mapPair2tf-response"
  "multirobot_map_merge/mapPair2tfResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mapPair2tf-response>)))
  "Returns md5sum for a message object of type '<mapPair2tf-response>"
  "6fa85f9e352b2ce5cb5617042231d419")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mapPair2tf-response)))
  "Returns md5sum for a message object of type 'mapPair2tf-response"
  "6fa85f9e352b2ce5cb5617042231d419")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mapPair2tf-response>)))
  "Returns full string definition for message of type '<mapPair2tf-response>"
  (cl:format cl:nil "geometry_msgs/Transform transform~%float64 confidence~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mapPair2tf-response)))
  "Returns full string definition for message of type 'mapPair2tf-response"
  (cl:format cl:nil "geometry_msgs/Transform transform~%float64 confidence~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mapPair2tf-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transform))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mapPair2tf-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mapPair2tf-response
    (cl:cons ':transform (transform msg))
    (cl:cons ':confidence (confidence msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mapPair2tf)))
  'mapPair2tf-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mapPair2tf)))
  'mapPair2tf-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mapPair2tf)))
  "Returns string type for a service object of type '<mapPair2tf>"
  "multirobot_map_merge/mapPair2tf")