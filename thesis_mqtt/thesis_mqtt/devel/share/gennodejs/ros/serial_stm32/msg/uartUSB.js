// Auto-generated. Do not edit!

// (in-package serial_stm32.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class uartUSB {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dir1 = null;
      this.dir2 = null;
      this.dir3 = null;
    }
    else {
      if (initObj.hasOwnProperty('dir1')) {
        this.dir1 = initObj.dir1
      }
      else {
        this.dir1 = '';
      }
      if (initObj.hasOwnProperty('dir2')) {
        this.dir2 = initObj.dir2
      }
      else {
        this.dir2 = '';
      }
      if (initObj.hasOwnProperty('dir3')) {
        this.dir3 = initObj.dir3
      }
      else {
        this.dir3 = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type uartUSB
    // Serialize message field [dir1]
    bufferOffset = _serializer.string(obj.dir1, buffer, bufferOffset);
    // Serialize message field [dir2]
    bufferOffset = _serializer.string(obj.dir2, buffer, bufferOffset);
    // Serialize message field [dir3]
    bufferOffset = _serializer.string(obj.dir3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type uartUSB
    let len;
    let data = new uartUSB(null);
    // Deserialize message field [dir1]
    data.dir1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [dir2]
    data.dir2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [dir3]
    data.dir3 = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.dir1);
    length += _getByteLength(object.dir2);
    length += _getByteLength(object.dir3);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'serial_stm32/uartUSB';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '00bcf444a943c3a48aff860ffd0aed64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string dir1
    string dir2
    string dir3
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new uartUSB(null);
    if (msg.dir1 !== undefined) {
      resolved.dir1 = msg.dir1;
    }
    else {
      resolved.dir1 = ''
    }

    if (msg.dir2 !== undefined) {
      resolved.dir2 = msg.dir2;
    }
    else {
      resolved.dir2 = ''
    }

    if (msg.dir3 !== undefined) {
      resolved.dir3 = msg.dir3;
    }
    else {
      resolved.dir3 = ''
    }

    return resolved;
    }
};

module.exports = uartUSB;
