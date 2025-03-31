// Auto-generated. Do not edit!

// (in-package f1tenth_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RaceInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ego_lap_count = null;
      this.opp_lap_count = null;
      this.ego_elapsed_time = null;
      this.opp_elapsed_time = null;
      this.ego_collision = null;
      this.opp_collision = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ego_lap_count')) {
        this.ego_lap_count = initObj.ego_lap_count
      }
      else {
        this.ego_lap_count = 0.0;
      }
      if (initObj.hasOwnProperty('opp_lap_count')) {
        this.opp_lap_count = initObj.opp_lap_count
      }
      else {
        this.opp_lap_count = 0.0;
      }
      if (initObj.hasOwnProperty('ego_elapsed_time')) {
        this.ego_elapsed_time = initObj.ego_elapsed_time
      }
      else {
        this.ego_elapsed_time = 0.0;
      }
      if (initObj.hasOwnProperty('opp_elapsed_time')) {
        this.opp_elapsed_time = initObj.opp_elapsed_time
      }
      else {
        this.opp_elapsed_time = 0.0;
      }
      if (initObj.hasOwnProperty('ego_collision')) {
        this.ego_collision = initObj.ego_collision
      }
      else {
        this.ego_collision = false;
      }
      if (initObj.hasOwnProperty('opp_collision')) {
        this.opp_collision = initObj.opp_collision
      }
      else {
        this.opp_collision = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RaceInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ego_lap_count]
    bufferOffset = _serializer.float32(obj.ego_lap_count, buffer, bufferOffset);
    // Serialize message field [opp_lap_count]
    bufferOffset = _serializer.float32(obj.opp_lap_count, buffer, bufferOffset);
    // Serialize message field [ego_elapsed_time]
    bufferOffset = _serializer.float32(obj.ego_elapsed_time, buffer, bufferOffset);
    // Serialize message field [opp_elapsed_time]
    bufferOffset = _serializer.float32(obj.opp_elapsed_time, buffer, bufferOffset);
    // Serialize message field [ego_collision]
    bufferOffset = _serializer.bool(obj.ego_collision, buffer, bufferOffset);
    // Serialize message field [opp_collision]
    bufferOffset = _serializer.bool(obj.opp_collision, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RaceInfo
    let len;
    let data = new RaceInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ego_lap_count]
    data.ego_lap_count = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [opp_lap_count]
    data.opp_lap_count = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ego_elapsed_time]
    data.ego_elapsed_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [opp_elapsed_time]
    data.opp_elapsed_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ego_collision]
    data.ego_collision = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [opp_collision]
    data.opp_collision = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'f1tenth_control/RaceInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45253b51fa7489b954a1e38efc4deae1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RaceInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ego_lap_count !== undefined) {
      resolved.ego_lap_count = msg.ego_lap_count;
    }
    else {
      resolved.ego_lap_count = 0.0
    }

    if (msg.opp_lap_count !== undefined) {
      resolved.opp_lap_count = msg.opp_lap_count;
    }
    else {
      resolved.opp_lap_count = 0.0
    }

    if (msg.ego_elapsed_time !== undefined) {
      resolved.ego_elapsed_time = msg.ego_elapsed_time;
    }
    else {
      resolved.ego_elapsed_time = 0.0
    }

    if (msg.opp_elapsed_time !== undefined) {
      resolved.opp_elapsed_time = msg.opp_elapsed_time;
    }
    else {
      resolved.opp_elapsed_time = 0.0
    }

    if (msg.ego_collision !== undefined) {
      resolved.ego_collision = msg.ego_collision;
    }
    else {
      resolved.ego_collision = false
    }

    if (msg.opp_collision !== undefined) {
      resolved.opp_collision = msg.opp_collision;
    }
    else {
      resolved.opp_collision = false
    }

    return resolved;
    }
};

module.exports = RaceInfo;
