// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0
//
// Minimal CDR (little-endian XCDR1) reader for the ROS 2 messages the UI reads.
// rmw_zenoh payloads start with a 4-byte encapsulation header; alignment is
// relative to the byte after it.

export class CdrReader {
  constructor(bytes) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    this.bytes = bytes;
    const kind = this.view.getUint16(0, false);
    if (kind !== 0x0001) {
      throw new Error(`unsupported CDR encapsulation 0x${kind.toString(16)} (expected little-endian CDR)`);
    }
    this.base = 4;
    this.pos = 4;
  }

  align(n) {
    const offset = this.pos - this.base;
    this.pos += (n - (offset % n)) % n;
  }

  uint8() { return this.view.getUint8(this.pos++); }
  bool() { return this.uint8() !== 0; }
  int32() { this.align(4); const v = this.view.getInt32(this.pos, true); this.pos += 4; return v; }
  uint32() { this.align(4); const v = this.view.getUint32(this.pos, true); this.pos += 4; return v; }
  int64() {
    this.align(8);
    const v = this.view.getBigInt64(this.pos, true);
    this.pos += 8;
    return Number(v);
  }
  float64() { this.align(8); const v = this.view.getFloat64(this.pos, true); this.pos += 8; return v; }
  string() {
    const length = this.uint32(); // includes the trailing NUL
    const text = new TextDecoder().decode(this.bytes.subarray(this.pos, this.pos + Math.max(0, length - 1)));
    this.pos += length;
    return text;
  }
  sequence(readElement) {
    const count = this.uint32();
    const out = [];
    for (let i = 0; i < count; i++) out.push(readElement(this));
    return out;
  }
  time() { return { sec: this.int32(), nanosec: this.uint32() }; }
  header() { return { stamp: this.time(), frame_id: this.string() }; }
}

export function base64ToBytes(b64) {
  const binary = atob(b64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  return bytes;
}

// chessbot_interfaces/msg/GameState
export function decodeGameState(bytes) {
  const r = new CdrReader(bytes);
  return {
    header: r.header(),
    phase: r.uint8(),
    mode: r.uint8(),
    fen: r.string(),
    turn: r.uint8(),
    robot_side: r.uint8(),
    moves: r.sequence((x) => x.string()),
    result: r.uint8(),
    engine_elo: r.int32(),
    white_ms: r.int64(),
    black_ms: r.int64(),
    clock_running: r.uint8(),
    clock_last_switch: r.time(),
  };
}

// chessbot_interfaces/msg/Thought
export function decodeThought(bytes) {
  const r = new CdrReader(bytes);
  return { header: r.header(), kind: r.uint8(), text: r.string(), model_generated: r.bool() };
}

// sensor_msgs/msg/JointState (velocity and effort are not needed)
export function decodeJointState(bytes) {
  const r = new CdrReader(bytes);
  return { header: r.header(), name: r.sequence((x) => x.string()), position: r.sequence((x) => x.float64()) };
}
