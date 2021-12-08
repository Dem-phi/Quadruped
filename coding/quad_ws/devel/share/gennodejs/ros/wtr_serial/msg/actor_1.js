// Auto-generated. Do not edit!

// (in-package wtr_serial.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class actor_1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.agl_0 = null;
      this.agl_1 = null;
      this.agl_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('agl_0')) {
        this.agl_0 = initObj.agl_0
      }
      else {
        this.agl_0 = 0.0;
      }
      if (initObj.hasOwnProperty('agl_1')) {
        this.agl_1 = initObj.agl_1
      }
      else {
        this.agl_1 = 0.0;
      }
      if (initObj.hasOwnProperty('agl_2')) {
        this.agl_2 = initObj.agl_2
      }
      else {
        this.agl_2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type actor_1
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [agl_0]
    bufferOffset = _serializer.float32(obj.agl_0, buffer, bufferOffset);
    // Serialize message field [agl_1]
    bufferOffset = _serializer.float32(obj.agl_1, buffer, bufferOffset);
    // Serialize message field [agl_2]
    bufferOffset = _serializer.float32(obj.agl_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type actor_1
    let len;
    let data = new actor_1(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [agl_0]
    data.agl_0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [agl_1]
    data.agl_1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [agl_2]
    data.agl_2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'wtr_serial/actor_1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41170c4af0e4f57d0ef889762422e6e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 id
    float32 agl_0
    float32 agl_1
    float32 agl_2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new actor_1(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.agl_0 !== undefined) {
      resolved.agl_0 = msg.agl_0;
    }
    else {
      resolved.agl_0 = 0.0
    }

    if (msg.agl_1 !== undefined) {
      resolved.agl_1 = msg.agl_1;
    }
    else {
      resolved.agl_1 = 0.0
    }

    if (msg.agl_2 !== undefined) {
      resolved.agl_2 = msg.agl_2;
    }
    else {
      resolved.agl_2 = 0.0
    }

    return resolved;
    }
};

module.exports = actor_1;
