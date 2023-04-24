; Auto-generated. Do not edit!


(cl:in-package f1tenth_control-msg)


;//! \htmlinclude RaceInfo.msg.html

(cl:defclass <RaceInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ego_lap_count
    :reader ego_lap_count
    :initarg :ego_lap_count
    :type cl:float
    :initform 0.0)
   (opp_lap_count
    :reader opp_lap_count
    :initarg :opp_lap_count
    :type cl:float
    :initform 0.0)
   (ego_elapsed_time
    :reader ego_elapsed_time
    :initarg :ego_elapsed_time
    :type cl:float
    :initform 0.0)
   (opp_elapsed_time
    :reader opp_elapsed_time
    :initarg :opp_elapsed_time
    :type cl:float
    :initform 0.0)
   (ego_collision
    :reader ego_collision
    :initarg :ego_collision
    :type cl:boolean
    :initform cl:nil)
   (opp_collision
    :reader opp_collision
    :initarg :opp_collision
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RaceInfo (<RaceInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RaceInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RaceInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name f1tenth_control-msg:<RaceInfo> is deprecated: use f1tenth_control-msg:RaceInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RaceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f1tenth_control-msg:header-val is deprecated.  Use f1tenth_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ego_lap_count-val :lambda-list '(m))
(cl:defmethod ego_lap_count-val ((m <RaceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f1tenth_control-msg:ego_lap_count-val is deprecated.  Use f1tenth_control-msg:ego_lap_count instead.")
  (ego_lap_count m))

(cl:ensure-generic-function 'opp_lap_count-val :lambda-list '(m))
(cl:defmethod opp_lap_count-val ((m <RaceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f1tenth_control-msg:opp_lap_count-val is deprecated.  Use f1tenth_control-msg:opp_lap_count instead.")
  (opp_lap_count m))

(cl:ensure-generic-function 'ego_elapsed_time-val :lambda-list '(m))
(cl:defmethod ego_elapsed_time-val ((m <RaceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f1tenth_control-msg:ego_elapsed_time-val is deprecated.  Use f1tenth_control-msg:ego_elapsed_time instead.")
  (ego_elapsed_time m))

(cl:ensure-generic-function 'opp_elapsed_time-val :lambda-list '(m))
(cl:defmethod opp_elapsed_time-val ((m <RaceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f1tenth_control-msg:opp_elapsed_time-val is deprecated.  Use f1tenth_control-msg:opp_elapsed_time instead.")
  (opp_elapsed_time m))

(cl:ensure-generic-function 'ego_collision-val :lambda-list '(m))
(cl:defmethod ego_collision-val ((m <RaceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f1tenth_control-msg:ego_collision-val is deprecated.  Use f1tenth_control-msg:ego_collision instead.")
  (ego_collision m))

(cl:ensure-generic-function 'opp_collision-val :lambda-list '(m))
(cl:defmethod opp_collision-val ((m <RaceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f1tenth_control-msg:opp_collision-val is deprecated.  Use f1tenth_control-msg:opp_collision instead.")
  (opp_collision m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RaceInfo>) ostream)
  "Serializes a message object of type '<RaceInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ego_lap_count))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'opp_lap_count))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ego_elapsed_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'opp_elapsed_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ego_collision) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'opp_collision) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RaceInfo>) istream)
  "Deserializes a message object of type '<RaceInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ego_lap_count) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'opp_lap_count) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ego_elapsed_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'opp_elapsed_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'ego_collision) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'opp_collision) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RaceInfo>)))
  "Returns string type for a message object of type '<RaceInfo>"
  "f1tenth_control/RaceInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RaceInfo)))
  "Returns string type for a message object of type 'RaceInfo"
  "f1tenth_control/RaceInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RaceInfo>)))
  "Returns md5sum for a message object of type '<RaceInfo>"
  "45253b51fa7489b954a1e38efc4deae1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RaceInfo)))
  "Returns md5sum for a message object of type 'RaceInfo"
  "45253b51fa7489b954a1e38efc4deae1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RaceInfo>)))
  "Returns full string definition for message of type '<RaceInfo>"
  (cl:format cl:nil "Header header~%float32 ego_lap_count~%float32 opp_lap_count~%float32 ego_elapsed_time~%float32 opp_elapsed_time~%bool ego_collision~%bool opp_collision~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RaceInfo)))
  "Returns full string definition for message of type 'RaceInfo"
  (cl:format cl:nil "Header header~%float32 ego_lap_count~%float32 opp_lap_count~%float32 ego_elapsed_time~%float32 opp_elapsed_time~%bool ego_collision~%bool opp_collision~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RaceInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RaceInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'RaceInfo
    (cl:cons ':header (header msg))
    (cl:cons ':ego_lap_count (ego_lap_count msg))
    (cl:cons ':opp_lap_count (opp_lap_count msg))
    (cl:cons ':ego_elapsed_time (ego_elapsed_time msg))
    (cl:cons ':opp_elapsed_time (opp_elapsed_time msg))
    (cl:cons ':ego_collision (ego_collision msg))
    (cl:cons ':opp_collision (opp_collision msg))
))
