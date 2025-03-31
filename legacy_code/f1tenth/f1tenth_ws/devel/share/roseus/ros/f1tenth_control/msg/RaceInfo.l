;; Auto-generated. Do not edit!


(when (boundp 'f1tenth_control::RaceInfo)
  (if (not (find-package "F1TENTH_CONTROL"))
    (make-package "F1TENTH_CONTROL"))
  (shadow 'RaceInfo (find-package "F1TENTH_CONTROL")))
(unless (find-package "F1TENTH_CONTROL::RACEINFO")
  (make-package "F1TENTH_CONTROL::RACEINFO"))

(in-package "ROS")
;;//! \htmlinclude RaceInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass f1tenth_control::RaceInfo
  :super ros::object
  :slots (_header _ego_lap_count _opp_lap_count _ego_elapsed_time _opp_elapsed_time _ego_collision _opp_collision ))

(defmethod f1tenth_control::RaceInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ego_lap_count __ego_lap_count) 0.0)
    ((:opp_lap_count __opp_lap_count) 0.0)
    ((:ego_elapsed_time __ego_elapsed_time) 0.0)
    ((:opp_elapsed_time __opp_elapsed_time) 0.0)
    ((:ego_collision __ego_collision) nil)
    ((:opp_collision __opp_collision) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ego_lap_count (float __ego_lap_count))
   (setq _opp_lap_count (float __opp_lap_count))
   (setq _ego_elapsed_time (float __ego_elapsed_time))
   (setq _opp_elapsed_time (float __opp_elapsed_time))
   (setq _ego_collision __ego_collision)
   (setq _opp_collision __opp_collision)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ego_lap_count
   (&optional __ego_lap_count)
   (if __ego_lap_count (setq _ego_lap_count __ego_lap_count)) _ego_lap_count)
  (:opp_lap_count
   (&optional __opp_lap_count)
   (if __opp_lap_count (setq _opp_lap_count __opp_lap_count)) _opp_lap_count)
  (:ego_elapsed_time
   (&optional __ego_elapsed_time)
   (if __ego_elapsed_time (setq _ego_elapsed_time __ego_elapsed_time)) _ego_elapsed_time)
  (:opp_elapsed_time
   (&optional __opp_elapsed_time)
   (if __opp_elapsed_time (setq _opp_elapsed_time __opp_elapsed_time)) _opp_elapsed_time)
  (:ego_collision
   (&optional __ego_collision)
   (if __ego_collision (setq _ego_collision __ego_collision)) _ego_collision)
  (:opp_collision
   (&optional __opp_collision)
   (if __opp_collision (setq _opp_collision __opp_collision)) _opp_collision)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _ego_lap_count
    4
    ;; float32 _opp_lap_count
    4
    ;; float32 _ego_elapsed_time
    4
    ;; float32 _opp_elapsed_time
    4
    ;; bool _ego_collision
    1
    ;; bool _opp_collision
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _ego_lap_count
       (sys::poke _ego_lap_count (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _opp_lap_count
       (sys::poke _opp_lap_count (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ego_elapsed_time
       (sys::poke _ego_elapsed_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _opp_elapsed_time
       (sys::poke _opp_elapsed_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _ego_collision
       (if _ego_collision (write-byte -1 s) (write-byte 0 s))
     ;; bool _opp_collision
       (if _opp_collision (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _ego_lap_count
     (setq _ego_lap_count (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _opp_lap_count
     (setq _opp_lap_count (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ego_elapsed_time
     (setq _ego_elapsed_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _opp_elapsed_time
     (setq _opp_elapsed_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _ego_collision
     (setq _ego_collision (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _opp_collision
     (setq _opp_collision (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get f1tenth_control::RaceInfo :md5sum-) "45253b51fa7489b954a1e38efc4deae1")
(setf (get f1tenth_control::RaceInfo :datatype-) "f1tenth_control/RaceInfo")
(setf (get f1tenth_control::RaceInfo :definition-)
      "Header header
float32 ego_lap_count
float32 opp_lap_count
float32 ego_elapsed_time
float32 opp_elapsed_time
bool ego_collision
bool opp_collision
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

")



(provide :f1tenth_control/RaceInfo "45253b51fa7489b954a1e38efc4deae1")


