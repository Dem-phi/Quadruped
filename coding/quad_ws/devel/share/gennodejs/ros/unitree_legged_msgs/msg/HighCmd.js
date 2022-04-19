// Auto-generated. Do not edit!

// (in-package unitree_legged_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BmsCmd = require('./BmsCmd.js');
let LED = require('./LED.js');

//-----------------------------------------------------------

class HighCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.levelFlag = null;
      this.commVersion = null;
      this.robotID = null;
      this.SN = null;
      this.bandWidth = null;
      this.mode = null;
      this.gaitType = null;
      this.speedLevel = null;
      this.footRaiseHeight = null;
      this.bodyHeight = null;
      this.postion = null;
      this.euler = null;
      this.velocity = null;
      this.yawSpeed = null;
      this.bms = null;
      this.led = null;
      this.wirelessRemote = null;
      this.reserve = null;
      this.crc = null;
    }
    else {
      if (initObj.hasOwnProperty('levelFlag')) {
        this.levelFlag = initObj.levelFlag
      }
      else {
        this.levelFlag = 0;
      }
      if (initObj.hasOwnProperty('commVersion')) {
        this.commVersion = initObj.commVersion
      }
      else {
        this.commVersion = 0;
      }
      if (initObj.hasOwnProperty('robotID')) {
        this.robotID = initObj.robotID
      }
      else {
        this.robotID = 0;
      }
      if (initObj.hasOwnProperty('SN')) {
        this.SN = initObj.SN
      }
      else {
        this.SN = 0;
      }
      if (initObj.hasOwnProperty('bandWidth')) {
        this.bandWidth = initObj.bandWidth
      }
      else {
        this.bandWidth = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('gaitType')) {
        this.gaitType = initObj.gaitType
      }
      else {
        this.gaitType = 0;
      }
      if (initObj.hasOwnProperty('speedLevel')) {
        this.speedLevel = initObj.speedLevel
      }
      else {
        this.speedLevel = 0;
      }
      if (initObj.hasOwnProperty('footRaiseHeight')) {
        this.footRaiseHeight = initObj.footRaiseHeight
      }
      else {
        this.footRaiseHeight = 0.0;
      }
      if (initObj.hasOwnProperty('bodyHeight')) {
        this.bodyHeight = initObj.bodyHeight
      }
      else {
        this.bodyHeight = 0.0;
      }
      if (initObj.hasOwnProperty('postion')) {
        this.postion = initObj.postion
      }
      else {
        this.postion = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('euler')) {
        this.euler = initObj.euler
      }
      else {
        this.euler = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('yawSpeed')) {
        this.yawSpeed = initObj.yawSpeed
      }
      else {
        this.yawSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('bms')) {
        this.bms = initObj.bms
      }
      else {
        this.bms = new BmsCmd();
      }
      if (initObj.hasOwnProperty('led')) {
        this.led = initObj.led
      }
      else {
        this.led = new Array(4).fill(new LED());
      }
      if (initObj.hasOwnProperty('wirelessRemote')) {
        this.wirelessRemote = initObj.wirelessRemote
      }
      else {
        this.wirelessRemote = new Array(40).fill(0);
      }
      if (initObj.hasOwnProperty('reserve')) {
        this.reserve = initObj.reserve
      }
      else {
        this.reserve = 0;
      }
      if (initObj.hasOwnProperty('crc')) {
        this.crc = initObj.crc
      }
      else {
        this.crc = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HighCmd
    // Serialize message field [levelFlag]
    bufferOffset = _serializer.uint8(obj.levelFlag, buffer, bufferOffset);
    // Serialize message field [commVersion]
    bufferOffset = _serializer.uint16(obj.commVersion, buffer, bufferOffset);
    // Serialize message field [robotID]
    bufferOffset = _serializer.uint16(obj.robotID, buffer, bufferOffset);
    // Serialize message field [SN]
    bufferOffset = _serializer.uint32(obj.SN, buffer, bufferOffset);
    // Serialize message field [bandWidth]
    bufferOffset = _serializer.uint8(obj.bandWidth, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [gaitType]
    bufferOffset = _serializer.uint8(obj.gaitType, buffer, bufferOffset);
    // Serialize message field [speedLevel]
    bufferOffset = _serializer.uint8(obj.speedLevel, buffer, bufferOffset);
    // Serialize message field [footRaiseHeight]
    bufferOffset = _serializer.float32(obj.footRaiseHeight, buffer, bufferOffset);
    // Serialize message field [bodyHeight]
    bufferOffset = _serializer.float32(obj.bodyHeight, buffer, bufferOffset);
    // Check that the constant length array field [postion] has the right length
    if (obj.postion.length !== 2) {
      throw new Error('Unable to serialize array field postion - length must be 2')
    }
    // Serialize message field [postion]
    bufferOffset = _arraySerializer.float32(obj.postion, buffer, bufferOffset, 2);
    // Check that the constant length array field [euler] has the right length
    if (obj.euler.length !== 3) {
      throw new Error('Unable to serialize array field euler - length must be 3')
    }
    // Serialize message field [euler]
    bufferOffset = _arraySerializer.float32(obj.euler, buffer, bufferOffset, 3);
    // Check that the constant length array field [velocity] has the right length
    if (obj.velocity.length !== 2) {
      throw new Error('Unable to serialize array field velocity - length must be 2')
    }
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float32(obj.velocity, buffer, bufferOffset, 2);
    // Serialize message field [yawSpeed]
    bufferOffset = _serializer.float32(obj.yawSpeed, buffer, bufferOffset);
    // Serialize message field [bms]
    bufferOffset = BmsCmd.serialize(obj.bms, buffer, bufferOffset);
    // Check that the constant length array field [led] has the right length
    if (obj.led.length !== 4) {
      throw new Error('Unable to serialize array field led - length must be 4')
    }
    // Serialize message field [led]
    obj.led.forEach((val) => {
      bufferOffset = LED.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [wirelessRemote] has the right length
    if (obj.wirelessRemote.length !== 40) {
      throw new Error('Unable to serialize array field wirelessRemote - length must be 40')
    }
    // Serialize message field [wirelessRemote]
    bufferOffset = _arraySerializer.uint8(obj.wirelessRemote, buffer, bufferOffset, 40);
    // Serialize message field [reserve]
    bufferOffset = _serializer.uint32(obj.reserve, buffer, bufferOffset);
    // Serialize message field [crc]
    bufferOffset = _serializer.int32(obj.crc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HighCmd
    let len;
    let data = new HighCmd(null);
    // Deserialize message field [levelFlag]
    data.levelFlag = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [commVersion]
    data.commVersion = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [robotID]
    data.robotID = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [SN]
    data.SN = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bandWidth]
    data.bandWidth = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gaitType]
    data.gaitType = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [speedLevel]
    data.speedLevel = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [footRaiseHeight]
    data.footRaiseHeight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bodyHeight]
    data.bodyHeight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [postion]
    data.postion = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [euler]
    data.euler = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [yawSpeed]
    data.yawSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bms]
    data.bms = BmsCmd.deserialize(buffer, bufferOffset);
    // Deserialize message field [led]
    len = 4;
    data.led = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.led[i] = LED.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [wirelessRemote]
    data.wirelessRemote = _arrayDeserializer.uint8(buffer, bufferOffset, 40)
    // Deserialize message field [reserve]
    data.reserve = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [crc]
    data.crc = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 108;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/HighCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '662e986e0a4446722bb9fac50f5d8cfd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 levelFlag
    uint16 commVersion              # Old version Aliengo does not have
    uint16 robotID                  # Old version Aliengo does not have
    uint32 SN                       # Old version Aliengo does not have
    uint8 bandWidth                 # Old version Aliengo does not have
    uint8 mode                      # 0. idle, default stand  
                                    # 1. force stand (controlled by dBodyHeight + ypr)
                                    # 2. target velocity walking (controlled by velocity + yawSpeed)
                                    # 3. target position walking (controlled by position + ypr[0])
                                    # 4. path mode walking (reserve for future release)
                                    # 5. position stand down. 
                                    # 6. position stand up 
                                    # 7. damping mode 
                                    # 8. recovery stand
                                    # 9. backflip
                                    # 10. jumpYaw
                                    # 11. straightHand
                                    # 12. dance1
                                    # 13. dance2
                                    # 14. two leg stand
    uint8 gaitType                  # 0.idle  1.trot  2.trot running  3.climb stair
    uint8 speedLevel                # 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
    float32 footRaiseHeight         # (unit: m, default: 0.08m), foot up height while walking
    float32 bodyHeight              # (unit: m, default: 0.28m),
    float32[2] postion              # (unit: m), desired position in inertial frame
    float32[3] euler                # (unit: rad), roll pitch yaw in stand mode
    float32[2] velocity             # (unit: m/s), forwardSpeed, sideSpeed in body frame
    float32 yawSpeed                # (unit: rad/s), rotateSpeed in body frame
    BmsCmd bms
    LED[4] led
    uint8[40] wirelessRemote
    uint32 reserve                  # Old version Aliengo does not have
    int32 crc
    ================================================================================
    MSG: unitree_legged_msgs/BmsCmd
    uint8 off            # off 0xA5
    uint8[3] reserve
    ================================================================================
    MSG: unitree_legged_msgs/LED
    uint8 r
    uint8 g
    uint8 b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HighCmd(null);
    if (msg.levelFlag !== undefined) {
      resolved.levelFlag = msg.levelFlag;
    }
    else {
      resolved.levelFlag = 0
    }

    if (msg.commVersion !== undefined) {
      resolved.commVersion = msg.commVersion;
    }
    else {
      resolved.commVersion = 0
    }

    if (msg.robotID !== undefined) {
      resolved.robotID = msg.robotID;
    }
    else {
      resolved.robotID = 0
    }

    if (msg.SN !== undefined) {
      resolved.SN = msg.SN;
    }
    else {
      resolved.SN = 0
    }

    if (msg.bandWidth !== undefined) {
      resolved.bandWidth = msg.bandWidth;
    }
    else {
      resolved.bandWidth = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.gaitType !== undefined) {
      resolved.gaitType = msg.gaitType;
    }
    else {
      resolved.gaitType = 0
    }

    if (msg.speedLevel !== undefined) {
      resolved.speedLevel = msg.speedLevel;
    }
    else {
      resolved.speedLevel = 0
    }

    if (msg.footRaiseHeight !== undefined) {
      resolved.footRaiseHeight = msg.footRaiseHeight;
    }
    else {
      resolved.footRaiseHeight = 0.0
    }

    if (msg.bodyHeight !== undefined) {
      resolved.bodyHeight = msg.bodyHeight;
    }
    else {
      resolved.bodyHeight = 0.0
    }

    if (msg.postion !== undefined) {
      resolved.postion = msg.postion;
    }
    else {
      resolved.postion = new Array(2).fill(0)
    }

    if (msg.euler !== undefined) {
      resolved.euler = msg.euler;
    }
    else {
      resolved.euler = new Array(3).fill(0)
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = new Array(2).fill(0)
    }

    if (msg.yawSpeed !== undefined) {
      resolved.yawSpeed = msg.yawSpeed;
    }
    else {
      resolved.yawSpeed = 0.0
    }

    if (msg.bms !== undefined) {
      resolved.bms = BmsCmd.Resolve(msg.bms)
    }
    else {
      resolved.bms = new BmsCmd()
    }

    if (msg.led !== undefined) {
      resolved.led = new Array(4)
      for (let i = 0; i < resolved.led.length; ++i) {
        if (msg.led.length > i) {
          resolved.led[i] = LED.Resolve(msg.led[i]);
        }
        else {
          resolved.led[i] = new LED();
        }
      }
    }
    else {
      resolved.led = new Array(4).fill(new LED())
    }

    if (msg.wirelessRemote !== undefined) {
      resolved.wirelessRemote = msg.wirelessRemote;
    }
    else {
      resolved.wirelessRemote = new Array(40).fill(0)
    }

    if (msg.reserve !== undefined) {
      resolved.reserve = msg.reserve;
    }
    else {
      resolved.reserve = 0
    }

    if (msg.crc !== undefined) {
      resolved.crc = msg.crc;
    }
    else {
      resolved.crc = 0
    }

    return resolved;
    }
};

module.exports = HighCmd;
