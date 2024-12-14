;; Auto-generated. Do not edit!


(when (boundp 'multirobot_map_merge::mapPair2tf)
  (if (not (find-package "MULTIROBOT_MAP_MERGE"))
    (make-package "MULTIROBOT_MAP_MERGE"))
  (shadow 'mapPair2tf (find-package "MULTIROBOT_MAP_MERGE")))
(unless (find-package "MULTIROBOT_MAP_MERGE::MAPPAIR2TF")
  (make-package "MULTIROBOT_MAP_MERGE::MAPPAIR2TF"))
(unless (find-package "MULTIROBOT_MAP_MERGE::MAPPAIR2TFREQUEST")
  (make-package "MULTIROBOT_MAP_MERGE::MAPPAIR2TFREQUEST"))
(unless (find-package "MULTIROBOT_MAP_MERGE::MAPPAIR2TFRESPONSE")
  (make-package "MULTIROBOT_MAP_MERGE::MAPPAIR2TFRESPONSE"))

(in-package "ROS")

(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass multirobot_map_merge::mapPair2tfRequest
  :super ros::object
  :slots (_grid1 _grid2 ))

(defmethod multirobot_map_merge::mapPair2tfRequest
  (:init
   (&key
    ((:grid1 __grid1) (instance nav_msgs::OccupancyGrid :init))
    ((:grid2 __grid2) (instance nav_msgs::OccupancyGrid :init))
    )
   (send-super :init)
   (setq _grid1 __grid1)
   (setq _grid2 __grid2)
   self)
  (:grid1
   (&rest __grid1)
   (if (keywordp (car __grid1))
       (send* _grid1 __grid1)
     (progn
       (if __grid1 (setq _grid1 (car __grid1)))
       _grid1)))
  (:grid2
   (&rest __grid2)
   (if (keywordp (car __grid2))
       (send* _grid2 __grid2)
     (progn
       (if __grid2 (setq _grid2 (car __grid2)))
       _grid2)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/OccupancyGrid _grid1
    (send _grid1 :serialization-length)
    ;; nav_msgs/OccupancyGrid _grid2
    (send _grid2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/OccupancyGrid _grid1
       (send _grid1 :serialize s)
     ;; nav_msgs/OccupancyGrid _grid2
       (send _grid2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/OccupancyGrid _grid1
     (send _grid1 :deserialize buf ptr-) (incf ptr- (send _grid1 :serialization-length))
   ;; nav_msgs/OccupancyGrid _grid2
     (send _grid2 :deserialize buf ptr-) (incf ptr- (send _grid2 :serialization-length))
   ;;
   self)
  )

(defclass multirobot_map_merge::mapPair2tfResponse
  :super ros::object
  :slots (_transform _confidence ))

(defmethod multirobot_map_merge::mapPair2tfResponse
  (:init
   (&key
    ((:transform __transform) (instance geometry_msgs::Transform :init))
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _transform __transform)
   (setq _confidence (float __confidence))
   self)
  (:transform
   (&rest __transform)
   (if (keywordp (car __transform))
       (send* _transform __transform)
     (progn
       (if __transform (setq _transform (car __transform)))
       _transform)))
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Transform _transform
    (send _transform :serialization-length)
    ;; float64 _confidence
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Transform _transform
       (send _transform :serialize s)
     ;; float64 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Transform _transform
     (send _transform :deserialize buf ptr-) (incf ptr- (send _transform :serialization-length))
   ;; float64 _confidence
     (setq _confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass multirobot_map_merge::mapPair2tf
  :super ros::object
  :slots ())

(setf (get multirobot_map_merge::mapPair2tf :md5sum-) "6fa85f9e352b2ce5cb5617042231d419")
(setf (get multirobot_map_merge::mapPair2tf :datatype-) "multirobot_map_merge/mapPair2tf")
(setf (get multirobot_map_merge::mapPair2tf :request) multirobot_map_merge::mapPair2tfRequest)
(setf (get multirobot_map_merge::mapPair2tf :response) multirobot_map_merge::mapPair2tfResponse)

(defmethod multirobot_map_merge::mapPair2tfRequest
  (:response () (instance multirobot_map_merge::mapPair2tfResponse :init)))

(setf (get multirobot_map_merge::mapPair2tfRequest :md5sum-) "6fa85f9e352b2ce5cb5617042231d419")
(setf (get multirobot_map_merge::mapPair2tfRequest :datatype-) "multirobot_map_merge/mapPair2tfRequest")
(setf (get multirobot_map_merge::mapPair2tfRequest :definition-)
      "nav_msgs/OccupancyGrid grid1
nav_msgs/OccupancyGrid grid2

================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
geometry_msgs/Transform transform
float64 confidence

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get multirobot_map_merge::mapPair2tfResponse :md5sum-) "6fa85f9e352b2ce5cb5617042231d419")
(setf (get multirobot_map_merge::mapPair2tfResponse :datatype-) "multirobot_map_merge/mapPair2tfResponse")
(setf (get multirobot_map_merge::mapPair2tfResponse :definition-)
      "nav_msgs/OccupancyGrid grid1
nav_msgs/OccupancyGrid grid2

================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
geometry_msgs/Transform transform
float64 confidence

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :multirobot_map_merge/mapPair2tf "6fa85f9e352b2ce5cb5617042231d419")


