package com.company.demo.GroundControl.util_

import com.company.demo.GroundControl.*
import org.unirail.AdHoc.*
import org.unirail.AdHoc.Pack
import org.unirail.AdHoc.Pack.Cursor

inline class ATTITUDE_TARGET_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 5 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: ATTITUDE_TARGET_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class MISSION_COUNT_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class ADSB_VEHICLE_altitude_type(val data: Cursor) {
    inline fun get(): ADSB_ALTITUDE_TYPE {
        return ADSB_ALTITUDE_TYPE((get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class ADSB_VEHICLE_callsign(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: ADSB_VEHICLE_callsign): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class ADSB_VEHICLE_emitter_type(val data: Cursor) {
    inline fun get(): ADSB_EMITTER_TYPE {
        return ADSB_EMITTER_TYPE((get_bits(data.bytes, data.BIT, 5)).toByte())
    }
}

inline class ADSB_VEHICLE_flags(val data: Cursor) {
    inline fun get(): ADSB_FLAGS {
        return ADSB_FLAGS.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class EKF_STATUS_REPORT_flags(val data: Cursor) {
    inline fun get(): EKF_STATUS_FLAGS {
        return EKF_STATUS_FLAGS.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class ESTIMATOR_STATUS_flags(val data: Cursor) {
    inline fun get(): ESTIMATOR_STATUS_FLAGS {
        return ESTIMATOR_STATUS_FLAGS.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class GLOBAL_POSITION_INT_COV_covariance(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 36
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 36) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GLOBAL_POSITION_INT_COV_covariance): Boolean {
        if (other.len() != 36) return false
        for (i in 0 until 36) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class GLOBAL_POSITION_INT_COV_estimator_type(val data: Cursor) {
    inline fun get(): MAV_ESTIMATOR_TYPE {
        return MAV_ESTIMATOR_TYPE((1 + get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class SAFETY_SET_ALLOWED_AREA_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class UAVCAN_NODE_STATUS_health(val data: Cursor) {
    inline fun get(): UAVCAN_NODE_HEALTH {
        return UAVCAN_NODE_HEALTH((get_bits(data.bytes, data.BIT, 2)).toByte())
    }
}

inline class UAVCAN_NODE_STATUS_mode(val data: Cursor) {
    inline fun get(): UAVCAN_NODE_MODE {
        return UAVCAN_NODE_MODE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class COLLISION_sRc(val data: Cursor) {
    inline fun get(): MAV_COLLISION_SRC {
        return MAV_COLLISION_SRC((get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class COLLISION_action(val data: Cursor) {
    inline fun get(): MAV_COLLISION_ACTION {
        return MAV_COLLISION_ACTION((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class COLLISION_threat_level(val data: Cursor) {
    inline fun get(): MAV_COLLISION_THREAT_LEVEL {
        return MAV_COLLISION_THREAT_LEVEL((get_bits(data.bytes, data.BIT, 2)).toByte())
    }
}

inline class HIL_STATE_QUATERNION_attitude_quaternion(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: HIL_STATE_QUATERNION_attitude_quaternion): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12 + index * 4)
    }
}

inline class CAMERA_INFORMATION_vendor_name(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 32
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 14 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 32) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CAMERA_INFORMATION_vendor_name): Boolean {
        if (other.len() != 32) return false
        for (i in 0 until 32) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 14 + index] = (src).toByte()
    }
}

inline class CAMERA_INFORMATION_model_name(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 32
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 46 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 32) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CAMERA_INFORMATION_model_name): Boolean {
        if (other.len() != 32) return false
        for (i in 0 until 32) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 46 + index] = (src).toByte()
    }
}

inline class CAMERA_INFORMATION_flags(val data: Cursor) {
    inline fun get(): CAMERA_CAP_FLAGS {
        return CAMERA_CAP_FLAGS.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class CAMERA_INFORMATION_cam_definition_uri(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CAMERA_INFORMATION_cam_definition_uri): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class GPS_STATUS_satellite_prn(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 20
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 1 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 20) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GPS_STATUS_satellite_prn): Boolean {
        if (other.len() != 20) return false
        for (i in 0 until 20) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class GPS_STATUS_satellite_used(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 20
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 21 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 20) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GPS_STATUS_satellite_used): Boolean {
        if (other.len() != 20) return false
        for (i in 0 until 20) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class GPS_STATUS_satellite_elevation(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 20
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 41 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 20) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GPS_STATUS_satellite_elevation): Boolean {
        if (other.len() != 20) return false
        for (i in 0 until 20) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class GPS_STATUS_satellite_azimuth(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 20
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 61 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 20) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GPS_STATUS_satellite_azimuth): Boolean {
        if (other.len() != 20) return false
        for (i in 0 until 20) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class GPS_STATUS_satellite_snr(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 20
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 81 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 20) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GPS_STATUS_satellite_snr): Boolean {
        if (other.len() != 20) return false
        for (i in 0 until 20) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class PARAM_SET_param_id(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_SET_param_id): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class PARAM_SET_param_type(val data: Cursor) {
    inline fun get(): MAV_PARAM_TYPE {
        return MAV_PARAM_TYPE((1 + get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class TERRAIN_DATA_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 16
    }

    inline fun get(index: Int): Short {
        return (get_bytes(data.bytes, data.origin + 11 + index * 2, 2)).toShort()
    }

    fun same(other: ShortArray): Boolean {
        if (other.size != 16) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: TERRAIN_DATA_daTa): Boolean {
        if (other.len() != 16) return false
        for (i in 0 until 16) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Short, index: Int) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 11 + index * 2)
    }
}

inline class VIDEO_STREAM_INFORMATION_uri(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: VIDEO_STREAM_INFORMATION_uri): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class CAMERA_IMAGE_CAPTURED_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 29 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CAMERA_IMAGE_CAPTURED_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 29 + index * 4)
    }
}

inline class CAMERA_IMAGE_CAPTURED_file_url(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CAMERA_IMAGE_CAPTURED_file_url): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class ACTUATOR_CONTROL_TARGET_controls(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 8
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 9 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 8) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: ACTUATOR_CONTROL_TARGET_controls): Boolean {
        if (other.len() != 8) return false
        for (i in 0 until 8) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 9 + index * 4)
    }
}

inline class HIGH_LATENCY_base_mode(val data: Cursor) {
    inline fun get(): MAV_MODE_FLAG {
        return MAV_MODE_FLAG.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class HIGH_LATENCY_landed_state(val data: Cursor) {
    inline fun get(): MAV_LANDED_STATE {
        return MAV_LANDED_STATE((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class HIGH_LATENCY_gps_fix_type(val data: Cursor) {
    inline fun get(): GPS_FIX_TYPE {
        return GPS_FIX_TYPE((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class PARAM_REQUEST_READ_param_id(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_REQUEST_READ_param_id): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class SET_ATTITUDE_TARGET_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 7 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: SET_ATTITUDE_TARGET_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class FOLLOW_TARGET_vel(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 3
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 29 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 3) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: FOLLOW_TARGET_vel): Boolean {
        if (other.len() != 3) return false
        for (i in 0 until 3) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 29 + index * 4)
    }
}

inline class FOLLOW_TARGET_acc(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 3
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 41 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 3) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: FOLLOW_TARGET_acc): Boolean {
        if (other.len() != 3) return false
        for (i in 0 until 3) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 41 + index * 4)
    }
}

inline class FOLLOW_TARGET_attitude_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 53 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: FOLLOW_TARGET_attitude_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 53 + index * 4)
    }
}

inline class FOLLOW_TARGET_rates(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 3
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 69 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 3) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: FOLLOW_TARGET_rates): Boolean {
        if (other.len() != 3) return false
        for (i in 0 until 3) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 69 + index * 4)
    }
}

inline class FOLLOW_TARGET_position_cov(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 3
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 81 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 3) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: FOLLOW_TARGET_position_cov): Boolean {
        if (other.len() != 3) return false
        for (i in 0 until 3) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 81 + index * 4)
    }
}

inline class HOME_POSITION_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: HOME_POSITION_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24 + index * 4)
    }
}

inline class FENCE_STATUS_breach_type(val data: Cursor) {
    inline fun get(): FENCE_BREACH {
        return FENCE_BREACH((get_bits(data.bytes, data.BIT, 2)).toByte())
    }
}

inline class REMOTE_LOG_BLOCK_STATUS_status(val data: Cursor) {
    inline fun get(): MAV_REMOTE_LOG_DATA_BLOCK_STATUSES {
        return MAV_REMOTE_LOG_DATA_BLOCK_STATUSES((get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class OBSTACLE_DISTANCE_distances(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 72
    }

    inline fun get(index: Int): Short {
        return (get_bytes(data.bytes, data.origin + 0 + index * 2, 2)).toShort()
    }

    fun same(other: ShortArray): Boolean {
        if (other.size != 72) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: OBSTACLE_DISTANCE_distances): Boolean {
        if (other.len() != 72) return false
        for (i in 0 until 72) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Short, index: Int) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0 + index * 2)
    }
}

inline class OBSTACLE_DISTANCE_sensor_type(val data: Cursor) {
    inline fun get(): MAV_DISTANCE_SENSOR {
        return MAV_DISTANCE_SENSOR((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class GPS2_RAW_fix_type(val data: Cursor) {
    inline fun get(): GPS_FIX_TYPE {
        return GPS_FIX_TYPE((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class MEMORY_VECT_value(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 32
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 4 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 32) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: MEMORY_VECT_value): Boolean {
        if (other.len() != 32) return false
        for (i in 0 until 32) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 4 + index] = (src).toByte()
    }
}

inline class PARAM_EXT_REQUEST_READ_param_id(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_EXT_REQUEST_READ_param_id): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class HIL_CONTROLS_mode(val data: Cursor) {
    inline fun get(): MAV_MODE {
        return MAV_MODE.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class SETUP_SIGNING_secret_key(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 32
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 10 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 32) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: SETUP_SIGNING_secret_key): Boolean {
        if (other.len() != 32) return false
        for (i in 0 until 32) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 10 + index] = (src).toByte()
    }
}

inline class UAVIONIX_ADSB_OUT_CFG_callsign(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: UAVIONIX_ADSB_OUT_CFG_callsign): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class UAVIONIX_ADSB_OUT_CFG_emitterType(val data: Cursor) {
    inline fun get(): ADSB_EMITTER_TYPE {
        return ADSB_EMITTER_TYPE((get_bits(data.bytes, data.BIT, 5)).toByte())
    }
}

inline class UAVIONIX_ADSB_OUT_CFG_aircraftSize(val data: Cursor) {
    inline fun get(): UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE {
        return UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class UAVIONIX_ADSB_OUT_CFG_gpsOffsetLat(val data: Cursor) {
    inline fun get(): UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT {
        return UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class UAVIONIX_ADSB_OUT_CFG_gpsOffsetLon(val data: Cursor) {
    inline fun get(): UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON {
        return UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON((get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class UAVIONIX_ADSB_OUT_CFG_rfSelect(val data: Cursor) {
    inline fun get(): UAVIONIX_ADSB_OUT_RF_SELECT {
        return UAVIONIX_ADSB_OUT_RF_SELECT((get_bits(data.bytes, data.BIT, 2)).toByte())
    }
}

inline class LANDING_TARGET_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class LANDING_TARGET_q(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int) -> Unit) {
        for (d0 in 0 until LANDING_TARGET.q.d0)

            if (data.set_item(d0, -1)) dst(d0)
    }

    inline fun get(d0: Int): Boolean {
        return data.set_item(d0, -1)
    }
}

inline class LANDING_TARGET_typE(val data: Cursor) {
    inline fun get(): LANDING_TARGET_TYPE {
        return LANDING_TARGET_TYPE((get_bits(data.bytes, data.BIT, 2)).toByte())
    }
}

inline class SET_ACTUATOR_CONTROL_TARGET_controls(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 8
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 11 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 8) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: SET_ACTUATOR_CONTROL_TARGET_controls): Boolean {
        if (other.len() != 8) return false
        for (i in 0 until 8) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 11 + index * 4)
    }
}

inline class CONTROL_SYSTEM_STATE_vel_variance(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 3
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 3) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CONTROL_SYSTEM_STATE_vel_variance): Boolean {
        if (other.len() != 3) return false
        for (i in 0 until 3) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 48 + index * 4)
    }
}

inline class CONTROL_SYSTEM_STATE_pos_variance(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 3
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 60 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 3) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CONTROL_SYSTEM_STATE_pos_variance): Boolean {
        if (other.len() != 3) return false
        for (i in 0 until 3) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 60 + index * 4)
    }
}

inline class CONTROL_SYSTEM_STATE_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 72 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CONTROL_SYSTEM_STATE_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 72 + index * 4)
    }
}

inline class SET_POSITION_TARGET_GLOBAL_INT_coordinate_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class DATA32_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 32
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 32) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DATA32_daTa): Boolean {
        if (other.len() != 32) return false
        for (i in 0 until 32) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class PING33_field1_Initializer(val data: Cursor) {
    inline fun init(d0: Int): PING33_field1_Field? {
        return if (data.set_field(1236, 0, d0)) PING33_field1_Field(data) else null
    }
}

inline class PING33_field1_Field(val data: Cursor) {
    inline fun d0(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Int {
        return (get_bytes(data.bytes, data.BYTE + (d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 2) * 4, 4)).toInt()
    }

    inline fun set(src: Int, d0: Int, d1: Int, d2: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.BYTE + (d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 2) * 4)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until d0())
            for (d1 in 0 until PING33.field1.d1)
                for (d2 in 0 until PING33.field1.d2)

                    dst(d0, d1, d2)
    }
}

inline class PING33_field1(val data: Cursor) {
    inline fun Initializer(): PING33_field1_Initializer? {
        return if (data.field_bit == 1236 || data.set_field(1236, -1)) null else PING33_field1_Initializer(data)
    }

    inline fun Field(): PING33_field1_Field? {
        return if (data.field_bit == 1236 || data.set_field(1236, -1)) PING33_field1_Field(data) else null
    }
}

inline class PING33_field12(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field12.d0)
            for (d1 in 0 until PING33.field12.d1)
                for (d2 in 0 until PING33.field12.d2)

                    dst(d0, d1, d2)
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Int {
        return (get_bytes(data.bytes, data.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)).toInt()
    }
}

inline class PING33_field13(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field13.d0)
            for (d1 in 0 until PING33.field13.d1)
                for (d2 in 0 until PING33.field13.d2)

                    if (data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)) dst(d0, d1, d2)
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)
    }
}

inline class PING33_Field_Bits(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.Field_Bits.d0)
            for (d1 in 0 until PING33.Field_Bits.d1)
                for (d2 in 0 until PING33.Field_Bits.d2)

                    dst(d0, d1, d2)
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Byte {
        return (4 + get_bits(data.bytes, data.BIT + (d0 + d1 * 3 + d2 * 3 * 3) * 6, 6)).toByte()
    }
}

inline class PING33_SparseFixAllBits_Initializer(val data: Cursor) {
    inline fun init(d0: Int): PING33_SparseFixAllBits_Field? {
        return if (data.set_field(1242, 0, d0)) PING33_SparseFixAllBits_Field(data) else null
    }
}

inline class PING33_SparseFixAllBits_Field(val data: Cursor) {
    inline fun d0(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, -1)
    }

    inline fun set(d0: Int, d1: Int, d2: Int) {
        data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, 0)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until d0())
            for (d1 in 0 until PING33.SparseFixAllBits.d1)
                for (d2 in 0 until PING33.SparseFixAllBits.d2)

                    if (data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, -1)) dst(d0, d1, d2)
    }
}

inline class PING33_SparseFixAllBits(val data: Cursor) {
    inline fun Initializer(): PING33_SparseFixAllBits_Initializer? {
        return if (data.field_bit == 1242 || data.set_field(1242, -1)) null else PING33_SparseFixAllBits_Initializer(data)
    }

    inline fun Field(): PING33_SparseFixAllBits_Field? {
        return if (data.field_bit == 1242 || data.set_field(1242, -1)) PING33_SparseFixAllBits_Field(data) else null
    }
}

inline class PING33_FixAllBits_Initializer(val data: Cursor) {
    inline fun init(d0: Int): PING33_FixAllBits_Field? {
        return if (data.set_field(1243, 0, d0)) PING33_FixAllBits_Field(data) else null
    }
}

inline class PING33_FixAllBits_Field(val data: Cursor) {
    inline fun d0(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Byte {
        return (14 + get_bits(data.bytes, data.BIT + (d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3) * 5, 5)).toByte()
    }

    inline fun set(src: Byte, d0: Int, d1: Int, d2: Int) {
        set_bits((-14 + src).toULong().toLong(), 5, data.bytes, data.BIT + (d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3) * 5)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until d0())
            for (d1 in 0 until PING33.FixAllBits.d1)
                for (d2 in 0 until PING33.FixAllBits.d2)

                    dst(d0, d1, d2)
    }
}

inline class PING33_FixAllBits(val data: Cursor) {
    inline fun Initializer(): PING33_FixAllBits_Initializer? {
        return if (data.field_bit == 1243 || data.set_field(1243, -1)) null else PING33_FixAllBits_Initializer(data)
    }

    inline fun Field(): PING33_FixAllBits_Field? {
        return if (data.field_bit == 1243 || data.set_field(1243, -1)) PING33_FixAllBits_Field(data) else null
    }
}

inline class PING33_VarAllBits_Initializer(val data: Cursor) {
    inline fun init(d0: Int, d2: Int): PING33_VarAllBits_Field? {
        return if (data.set_field(1244, 0, d0, d2)) PING33_VarAllBits_Field(data) else null
    }
}

inline class PING33_VarAllBits_Field(val data: Cursor) {
    inline fun d0(): Int {
        return data.var_dims[0]
    }

    inline fun d2(): Int {
        return data.var_dims[1]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Byte {
        return (14 + get_bits(data.bytes, data.BIT + (d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3) * 5, 5)).toByte()
    }

    inline fun set(src: Byte, d0: Int, d1: Int, d2: Int) {
        set_bits((-14 + src).toULong().toLong(), 5, data.bytes, data.BIT + (d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3) * 5)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until d0())
            for (d1 in 0 until PING33.VarAllBits.d1)
                for (d2 in 0 until d2())

                    dst(d0, d1, d2)
    }
}

inline class PING33_VarAllBits(val data: Cursor) {
    inline fun Initializer(): PING33_VarAllBits_Initializer? {
        return if (data.field_bit == 1244 || data.set_field(1244, -1)) null else PING33_VarAllBits_Initializer(data)
    }

    inline fun Field(): PING33_VarAllBits_Field? {
        return if (data.field_bit == 1244 || data.set_field(1244, -1)) PING33_VarAllBits_Field(data) else null
    }
}

inline class PING33_SparseVarAllBits_Initializer(val data: Cursor) {
    inline fun init(d0: Int, d2: Int): PING33_SparseVarAllBits_Field? {
        return if (data.set_field(1245, 0, d0, d2)) PING33_SparseVarAllBits_Field(data) else null
    }
}

inline class PING33_SparseVarAllBits_Field(val data: Cursor) {
    inline fun d0(): Int {
        return data.var_dims[0]
    }

    inline fun d2(): Int {
        return data.var_dims[1]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, -1)
    }

    inline fun set(d0: Int, d1: Int, d2: Int) {
        data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, 0)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until d0())
            for (d1 in 0 until PING33.SparseVarAllBits.d1)
                for (d2 in 0 until d2())

                    if (data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, -1)) dst(d0, d1, d2)
    }
}

inline class PING33_SparseVarAllBits(val data: Cursor) {
    inline fun Initializer(): PING33_SparseVarAllBits_Initializer? {
        return if (data.field_bit == 1245 || data.set_field(1245, -1)) null else PING33_SparseVarAllBits_Initializer(data)
    }

    inline fun Field(): PING33_SparseVarAllBits_Field? {
        return if (data.field_bit == 1245 || data.set_field(1245, -1)) PING33_SparseVarAllBits_Field(data) else null
    }
}

inline class PING33_VarEachBits_Initializer(val data: Cursor) {
    inline fun init(d0: Int): PING33_VarEachBits_Field? {
        return if (data.set_field(1246, 0, d0)) PING33_VarEachBits_Field(data) else null
    }
}

inline class PING33_VarEachBits_Field(val data: Cursor) {
    inline fun d0(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Byte {
        return (-14 + get_bits(data.bytes, data.BIT + (d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3) * 6, 6)).toByte()
    }

    inline fun set(src: Byte, d0: Int, d1: Int, d2: Int) {
        set_bits((14 + src).toULong().toLong(), 6, data.bytes, data.BIT + (d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3) * 6)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until d0())
            for (d1 in 0 until PING33.VarEachBits.d1)
                for (d2 in 0 until PING33.VarEachBits.d2)

                    dst(d0, d1, d2)
    }
}

inline class PING33_VarEachBits(val data: Cursor) {
    inline fun Initializer(): PING33_VarEachBits_Initializer? {
        return if (data.field_bit == 1246 || data.set_field(1246, -1)) null else PING33_VarEachBits_Initializer(data)
    }

    inline fun Field(): PING33_VarEachBits_Field? {
        return if (data.field_bit == 1246 || data.set_field(1246, -1)) PING33_VarEachBits_Field(data) else null
    }
}

inline class PING33_SparsVarEachBits_Initializer(val data: Cursor) {
    inline fun init(d0: Int): PING33_SparsVarEachBits_Field? {
        return if (data.set_field(1247, 0, d0)) PING33_SparsVarEachBits_Field(data) else null
    }
}

inline class PING33_SparsVarEachBits_Field(val data: Cursor) {
    inline fun d0(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, -1)
    }

    inline fun set(d0: Int, d1: Int, d2: Int) {
        data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, 0)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until d0())
            for (d1 in 0 until PING33.SparsVarEachBits.d1)
                for (d2 in 0 until PING33.SparsVarEachBits.d2)

                    if (data.set_item(d0 + d1 * data.var_dims[0] + d2 * data.var_dims[0] * 3, -1)) dst(d0, d1, d2)
    }
}

inline class PING33_SparsVarEachBits(val data: Cursor) {
    inline fun Initializer(): PING33_SparsVarEachBits_Initializer? {
        return if (data.field_bit == 1247 || data.set_field(1247, -1)) null else PING33_SparsVarEachBits_Initializer(data)
    }

    inline fun Field(): PING33_SparsVarEachBits_Field? {
        return if (data.field_bit == 1247 || data.set_field(1247, -1)) PING33_SparsVarEachBits_Field(data) else null
    }
}

inline class PING33_MMMMMM(val data: Cursor) {
    inline fun get(): MAV_MODE {
        return MAV_MODE.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class PING33_field44_Initializer(val data: Cursor) {
    inline fun init(d2: Int): PING33_field44_Field? {
        return if (data.set_field(1252, 0, d2)) PING33_field44_Field(data) else null
    }
}

inline class PING33_field44_Field(val data: Cursor) {
    inline fun d2(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Int {
        return (get_bytes(data.bytes, data.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)).toInt()
    }

    inline fun set(src: Int, d0: Int, d1: Int, d2: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field44.d0)
            for (d1 in 0 until PING33.field44.d1)
                for (d2 in 0 until d2())

                    dst(d0, d1, d2)
    }
}

inline class PING33_field44(val data: Cursor) {
    inline fun Initializer(): PING33_field44_Initializer? {
        return if (data.field_bit == 1252 || data.set_field(1252, -1)) null else PING33_field44_Initializer(data)
    }

    inline fun Field(): PING33_field44_Field? {
        return if (data.field_bit == 1252 || data.set_field(1252, -1)) PING33_field44_Field(data) else null
    }
}

inline class PING33_field634_Initializer(val data: Cursor) {
    inline fun init(d2: Int): PING33_field634_Field? {
        return if (data.set_field(1253, 0, d2)) PING33_field634_Field(data) else null
    }
}

inline class PING33_field634_Field(val data: Cursor) {
    inline fun d2(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Int {
        return (get_bytes(data.bytes, data.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)).toInt()
    }

    inline fun set(src: Int, d0: Int, d1: Int, d2: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field634.d0)
            for (d1 in 0 until PING33.field634.d1)
                for (d2 in 0 until d2())

                    dst(d0, d1, d2)
    }
}

inline class PING33_field634(val data: Cursor) {
    inline fun Initializer(): PING33_field634_Initializer? {
        return if (data.field_bit == 1253 || data.set_field(1253, -1)) null else PING33_field634_Initializer(data)
    }

    inline fun Field(): PING33_field634_Field? {
        return if (data.field_bit == 1253 || data.set_field(1253, -1)) PING33_field634_Field(data) else null
    }
}

inline class PING33_field33344_Initializer(val data: Cursor) {
    inline fun init(d2: Int): PING33_field33344_Field? {
        return if (data.set_field(1254, 0, d2)) PING33_field33344_Field(data) else null
    }
}

inline class PING33_field33344_Field(val data: Cursor) {
    inline fun d2(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)
    }

    inline fun set(d0: Int, d1: Int, d2: Int) {
        data.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field33344.d0)
            for (d1 in 0 until PING33.field33344.d1)
                for (d2 in 0 until d2())

                    if (data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)) dst(d0, d1, d2)
    }
}

inline class PING33_field33344(val data: Cursor) {
    inline fun Initializer(): PING33_field33344_Initializer? {
        return if (data.field_bit == 1254 || data.set_field(1254, -1)) null else PING33_field33344_Initializer(data)
    }

    inline fun Field(): PING33_field33344_Field? {
        return if (data.field_bit == 1254 || data.set_field(1254, -1)) PING33_field33344_Field(data) else null
    }
}

inline class PING33_field333634_Initializer(val data: Cursor) {
    inline fun init(d2: Int): PING33_field333634_Field? {
        return if (data.set_field(1255, 0, d2)) PING33_field333634_Field(data) else null
    }
}

inline class PING33_field333634_Field(val data: Cursor) {
    inline fun d2(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)
    }

    inline fun set(d0: Int, d1: Int, d2: Int) {
        data.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field333634.d0)
            for (d1 in 0 until PING33.field333634.d1)
                for (d2 in 0 until d2())

                    if (data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)) dst(d0, d1, d2)
    }
}

inline class PING33_field333634(val data: Cursor) {
    inline fun Initializer(): PING33_field333634_Initializer? {
        return if (data.field_bit == 1255 || data.set_field(1255, -1)) null else PING33_field333634_Initializer(data)
    }

    inline fun Field(): PING33_field333634_Field? {
        return if (data.field_bit == 1255 || data.set_field(1255, -1)) PING33_field333634_Field(data) else null
    }
}

inline class PING33_field__(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field__.d0)
            for (d1 in 0 until PING33.field__.d1)
                for (d2 in 0 until PING33.field__.d2)

                    if (data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)) dst(d0, d1, d2)
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)
    }
}

inline class PING33_field63_Initializer(val data: Cursor) {
    inline fun init(d2: Int): PING33_field63_Field? {
        return if (data.set_field(1257, 0, d2)) PING33_field63_Field(data) else null
    }
}

inline class PING33_field63_Field(val data: Cursor) {
    inline fun d2(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Int {
        return (get_bytes(data.bytes, data.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)).toInt()
    }

    inline fun set(src: Int, d0: Int, d1: Int, d2: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4)
    }

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field63.d0)
            for (d1 in 0 until PING33.field63.d1)
                for (d2 in 0 until d2())

                    dst(d0, d1, d2)
    }
}

inline class PING33_field63(val data: Cursor) {
    inline fun Initializer(): PING33_field63_Initializer? {
        return if (data.field_bit == 1257 || data.set_field(1257, -1)) null else PING33_field63_Initializer(data)
    }

    inline fun Field(): PING33_field63_Field? {
        return if (data.field_bit == 1257 || data.set_field(1257, -1)) PING33_field63_Field(data) else null
    }
}

inline class PING33_uid2(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int) -> Unit) {
        for (d0 in 0 until PING33.uid2.d0)

            if (data.set_item(d0, -1)) dst(d0)
    }

    inline fun get(d0: Int): Boolean {
        return data.set_item(d0, -1)
    }
}

inline class PING33_field2(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field2.d0)
            for (d1 in 0 until PING33.field2.d1)
                for (d2 in 0 until PING33.field2.d2)

                    if (data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)) dst(d0, d1, d2)
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)
    }
}

inline class PING33_field4(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.field4.d0)
            for (d1 in 0 until PING33.field4.d1)
                for (d2 in 0 until PING33.field4.d2)

                    if (data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)) dst(d0, d1, d2)
    }

    inline fun get(d0: Int, d1: Int, d2: Int): Boolean {
        return data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1)
    }
}

inline class PING33_stringtest1(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PING33_stringtest1): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PING33_stringtest2_Item(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PING33_stringtest2_Item): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PING33_stringtest2_Initializer(val data: Cursor) {
    inline fun init(d2: Int): PING33_stringtest2_Field? {
        return if (data.set_field(1262, 0, d2)) PING33_stringtest2_Field(data) else null
    }
}

inline class PING33_stringtest2_Field(val data: Cursor) {
    inline fun d2(): Int {
        return data.var_dims[0]
    }

    inline fun get(d0: Int, d1: Int, d2: Int): PING33_stringtest2_Item? {
        if ((!data.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1))) return null

        return PING33_stringtest2_Item(data)
    }

    inline fun set(src: String, d0: Int, d1: Int, d2: Int, reuse: CharArray?): CharArray {
        return data.encode(src, d0 + d1 * 3 + d2 * 3 * 2, reuse)
    }

    inline fun set(src: ByteArray, d0: Int, d1: Int, d2: Int) {
        val len = minOf(src.size, 255)
        data.set_item(d0 + d1 * 3 + d2 * 3 * 2, len)
        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun set(len: Int, d0: Int, d1: Int, d2: Int): PING33_stringtest2_Item {
        data.set_item(d0 + d1 * 3 + d2 * 3 * 2, minOf(len, 255))
        return PING33_stringtest2_Item(data)
    }

    inline fun enumerate(dst: (item: PING33_stringtest2_Item, d0: Int, d1: Int, d2: Int) -> Unit) {
        for (d0 in 0 until PING33.stringtest2.d0)
            for (d1 in 0 until PING33.stringtest2.d1)
                for (d2 in 0 until d2())

                    get(d0, d1, d2)?.let { dst(it, d0, d1, d2) }
    }
}

inline class PING33_stringtest2(val data: Cursor) {
    inline fun Initializer(): PING33_stringtest2_Initializer? {
        return if (data.field_bit == 1262 || data.set_field(1262, -1)) null else PING33_stringtest2_Initializer(data)
    }

    inline fun Field(): PING33_stringtest2_Field? {
        return if (data.field_bit == 1262 || data.set_field(1262, -1)) PING33_stringtest2_Field(data) else null
    }
}

inline class PING33_stringtest3(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PING33_stringtest3): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PING33_stringtest4(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PING33_stringtest4): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class RALLY_POINT_flags(val data: Cursor) {
    inline fun get(): RALLY_FLAGS {
        return RALLY_FLAGS((1 + get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class ADAP_TUNING_axis(val data: Cursor) {
    inline fun get(): PID_TUNING_AXIS {
        return PID_TUNING_AXIS((1 + get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class PARAM_EXT_VALUE_param_id(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_EXT_VALUE_param_id): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PARAM_EXT_VALUE_param_value(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_EXT_VALUE_param_value): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PARAM_EXT_VALUE_param_type(val data: Cursor) {
    inline fun get(): MAV_PARAM_EXT_TYPE {
        return MAV_PARAM_EXT_TYPE((1 + get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class LIMITS_STATUS_limits_state(val data: Cursor) {
    inline fun get(): LIMITS_STATE {
        return LIMITS_STATE((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class LIMITS_STATUS_mods_enabled(val data: Cursor) {
    inline fun get(): LIMIT_MODULE {
        return LIMIT_MODULE.get(get_bits(data.bytes, data.BIT, 2).toInt())
    }
}

inline class LIMITS_STATUS_mods_required(val data: Cursor) {
    inline fun get(): LIMIT_MODULE {
        return LIMIT_MODULE.get(get_bits(data.bytes, data.BIT, 2).toInt())
    }
}

inline class LIMITS_STATUS_mods_triggered(val data: Cursor) {
    inline fun get(): LIMIT_MODULE {
        return LIMIT_MODULE.get(get_bits(data.bytes, data.BIT, 2).toInt())
    }
}

inline class CAMERA_FEEDBACK_flags(val data: Cursor) {
    inline fun get(): CAMERA_FEEDBACK_FLAGS {
        return CAMERA_FEEDBACK_FLAGS((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class AUTH_KEY_key(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: AUTH_KEY_key): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class LOCAL_POSITION_NED_COV_covariance(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 45
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 45) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: LOCAL_POSITION_NED_COV_covariance): Boolean {
        if (other.len() != 45) return false
        for (i in 0 until 45) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class LOCAL_POSITION_NED_COV_estimator_type(val data: Cursor) {
    inline fun get(): MAV_ESTIMATOR_TYPE {
        return MAV_ESTIMATOR_TYPE((1 + get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class ATT_POS_MOCAP_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: ATT_POS_MOCAP_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8 + index * 4)
    }
}

inline class STATUSTEXT_severity(val data: Cursor) {
    inline fun get(): MAV_SEVERITY {
        return MAV_SEVERITY((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class STATUSTEXT_text(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: STATUSTEXT_text): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class GOPRO_GET_REQUEST_cmd_id(val data: Cursor) {
    inline fun get(): GOPRO_COMMAND {
        return GOPRO_COMMAND((get_bits(data.bytes, data.BIT, 5)).toByte())
    }
}

inline class ENCAPSULATED_DATA_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 253
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 253) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: ENCAPSULATED_DATA_daTa): Boolean {
        if (other.len() != 253) return false
        for (i in 0 until 253) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class GPS_INPUT_ignore_flags(val data: Cursor) {
    inline fun get(): GPS_INPUT_IGNORE_FLAGS {
        return GPS_INPUT_IGNORE_FLAGS.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class COMMAND_LONG_command(val data: Cursor) {
    inline fun get(): MAV_CMD {
        return MAV_CMD.get(get_bits(data.bytes, data.BIT, 8).toInt())
    }
}

inline class GPS_RAW_INT_fix_type(val data: Cursor) {
    inline fun get(): GPS_FIX_TYPE {
        return GPS_FIX_TYPE((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class CAMERA_STATUS_event_id(val data: Cursor) {
    inline fun get(): CAMERA_STATUS_TYPES {
        return CAMERA_STATUS_TYPES((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class CAMERA_SETTINGS_mode_id(val data: Cursor) {
    inline fun get(): CAMERA_MODE {
        return CAMERA_MODE((get_bits(data.bytes, data.BIT, 2)).toByte())
    }
}

inline class DEVICE_OP_READ_REPLY_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 128
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 7 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 128) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DEVICE_OP_READ_REPLY_daTa): Boolean {
        if (other.len() != 128) return false
        for (i in 0 until 128) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 7 + index] = (src).toByte()
    }
}

inline class NAMED_VALUE_FLOAT_name(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: NAMED_VALUE_FLOAT_name): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class GOPRO_HEARTBEAT_status(val data: Cursor) {
    inline fun get(): GOPRO_HEARTBEAT_STATUS {
        return GOPRO_HEARTBEAT_STATUS((get_bits(data.bytes, data.BIT, 2)).toByte())
    }
}

inline class GOPRO_HEARTBEAT_capture_mode(val data: Cursor) {
    inline fun get(): GOPRO_CAPTURE_MODE {
        return GOPRO_CAPTURE_MODE.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class GOPRO_HEARTBEAT_flags(val data: Cursor) {
    inline fun get(): GOPRO_HEARTBEAT_FLAGS {
        return GOPRO_HEARTBEAT_FLAGS((1 + get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class MISSION_WRITE_PARTIAL_LIST_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class PID_TUNING_axis(val data: Cursor) {
    inline fun get(): PID_TUNING_AXIS {
        return PID_TUNING_AXIS((1 + get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class SAFETY_ALLOWED_AREA_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class LOG_DATA_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 90
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 7 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 90) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: LOG_DATA_daTa): Boolean {
        if (other.len() != 90) return false
        for (i in 0 until 90) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 7 + index] = (src).toByte()
    }
}

inline class MISSION_CLEAR_ALL_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class MAG_CAL_REPORT_cal_status(val data: Cursor) {
    inline fun get(): MAG_CAL_STATUS {
        return MAG_CAL_STATUS((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class V2_EXTENSION_payload(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 249
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 5 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 249) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: V2_EXTENSION_payload): Boolean {
        if (other.len() != 249) return false
        for (i in 0 until 249) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 5 + index] = (src).toByte()
    }
}

inline class HEARTBEAT_typE(val data: Cursor) {
    inline fun get(): MAV_TYPE {
        return MAV_TYPE((get_bits(data.bytes, data.BIT, 5)).toByte())
    }
}

inline class HEARTBEAT_autopilot(val data: Cursor) {
    inline fun get(): MAV_AUTOPILOT {
        return MAV_AUTOPILOT((get_bits(data.bytes, data.BIT, 5)).toByte())
    }
}

inline class HEARTBEAT_base_mode(val data: Cursor) {
    inline fun get(): MAV_MODE_FLAG {
        return MAV_MODE_FLAG.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class HEARTBEAT_system_status(val data: Cursor) {
    inline fun get(): MAV_STATE {
        return MAV_STATE((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class PARAM_MAP_RC_param_id(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_MAP_RC_param_id): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class POWER_STATUS_flags(val data: Cursor) {
    inline fun get(): MAV_POWER_STATUS {
        return MAV_POWER_STATUS.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class REMOTE_LOG_DATA_BLOCK_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 200
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 200) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: REMOTE_LOG_DATA_BLOCK_daTa): Boolean {
        if (other.len() != 200) return false
        for (i in 0 until 200) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class REMOTE_LOG_DATA_BLOCK_seqno(val data: Cursor) {
    inline fun get(): MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS {
        return MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS((2147483645 + get_bits(data.bytes, data.BIT, 1)).toUInt())
    }
}

inline class LOGGING_DATA_ACKED_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 249
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 6 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 249) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: LOGGING_DATA_ACKED_daTa): Boolean {
        if (other.len() != 249) return false
        for (i in 0 until 249) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 6 + index] = (src).toByte()
    }
}

inline class MOUNT_CONFIGURE_mount_mode(val data: Cursor) {
    inline fun get(): MAV_MOUNT_MODE {
        return MAV_MOUNT_MODE((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class MISSION_REQUEST_INT_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class COMMAND_ACK_command(val data: Cursor) {
    inline fun get(): MAV_CMD {
        return MAV_CMD.get(get_bits(data.bytes, data.BIT, 8).toInt())
    }
}

inline class COMMAND_ACK_result(val data: Cursor) {
    inline fun get(): MAV_RESULT {
        return MAV_RESULT((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class MISSION_REQUEST_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class SET_HOME_POSITION_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 25 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: SET_HOME_POSITION_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 25 + index * 4)
    }
}

inline class SET_MODE_base_mode(val data: Cursor) {
    inline fun get(): MAV_MODE {
        return MAV_MODE.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class POSITION_TARGET_GLOBAL_INT_coordinate_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class LED_CONTROL_custom_bytes(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 24
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 5 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 24) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: LED_CONTROL_custom_bytes): Boolean {
        if (other.len() != 24) return false
        for (i in 0 until 24) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 5 + index] = (src).toByte()
    }
}

inline class WIFI_CONFIG_AP_ssid(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: WIFI_CONFIG_AP_ssid): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class WIFI_CONFIG_AP_password(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: WIFI_CONFIG_AP_password): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class DATA96_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 96
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 96) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DATA96_daTa): Boolean {
        if (other.len() != 96) return false
        for (i in 0 until 96) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class DEBUG_VECT_name(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DEBUG_VECT_name): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class MISSION_ACK_typE(val data: Cursor) {
    inline fun get(): MAV_MISSION_RESULT {
        return MAV_MISSION_RESULT((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class MISSION_ACK_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class GOPRO_SET_RESPONSE_cmd_id(val data: Cursor) {
    inline fun get(): GOPRO_COMMAND {
        return GOPRO_COMMAND((get_bits(data.bytes, data.BIT, 5)).toByte())
    }
}

inline class GOPRO_SET_RESPONSE_status(val data: Cursor) {
    inline fun get(): GOPRO_REQUEST_STATUS {
        return GOPRO_REQUEST_STATUS((get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class PROTOCOL_VERSION_spec_version_hash(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 8
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 6 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 8) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PROTOCOL_VERSION_spec_version_hash): Boolean {
        if (other.len() != 8) return false
        for (i in 0 until 8) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 6 + index] = (src).toByte()
    }
}

inline class PROTOCOL_VERSION_library_version_hash(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 8
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 14 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 8) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PROTOCOL_VERSION_library_version_hash): Boolean {
        if (other.len() != 8) return false
        for (i in 0 until 8) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 14 + index] = (src).toByte()
    }
}

inline class PARAM_VALUE_param_id(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_VALUE_param_id): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class PARAM_VALUE_param_type(val data: Cursor) {
    inline fun get(): MAV_PARAM_TYPE {
        return MAV_PARAM_TYPE((1 + get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class BATTERY_STATUS_voltages(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 10
    }

    inline fun get(index: Int): Short {
        return (get_bytes(data.bytes, data.origin + 0 + index * 2, 2)).toShort()
    }

    fun same(other: ShortArray): Boolean {
        if (other.size != 10) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: BATTERY_STATUS_voltages): Boolean {
        if (other.len() != 10) return false
        for (i in 0 until 10) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Short, index: Int) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0 + index * 2)
    }
}

inline class BATTERY_STATUS_battery_function(val data: Cursor) {
    inline fun get(): MAV_BATTERY_FUNCTION {
        return MAV_BATTERY_FUNCTION((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class BATTERY_STATUS_typE(val data: Cursor) {
    inline fun get(): MAV_BATTERY_TYPE {
        return MAV_BATTERY_TYPE((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class SERIAL_CONTROL_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 70
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 7 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 70) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: SERIAL_CONTROL_daTa): Boolean {
        if (other.len() != 70) return false
        for (i in 0 until 70) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 7 + index] = (src).toByte()
    }
}

inline class SERIAL_CONTROL_device(val data: Cursor) {
    inline fun get(): SERIAL_CONTROL_DEV {
        return SERIAL_CONTROL_DEV.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class SERIAL_CONTROL_flags(val data: Cursor) {
    inline fun get(): SERIAL_CONTROL_FLAG {
        return SERIAL_CONTROL_FLAG.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class SET_POSITION_TARGET_LOCAL_NED_coordinate_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class PARAM_EXT_SET_param_id(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_EXT_SET_param_id): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PARAM_EXT_SET_param_value(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_EXT_SET_param_value): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PARAM_EXT_SET_param_type(val data: Cursor) {
    inline fun get(): MAV_PARAM_EXT_TYPE {
        return MAV_PARAM_EXT_TYPE((1 + get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class AUTOPILOT_VERSION_flight_custom_version(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 8
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 28 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 8) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: AUTOPILOT_VERSION_flight_custom_version): Boolean {
        if (other.len() != 8) return false
        for (i in 0 until 8) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 28 + index] = (src).toByte()
    }
}

inline class AUTOPILOT_VERSION_middleware_custom_version(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 8
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 36 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 8) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: AUTOPILOT_VERSION_middleware_custom_version): Boolean {
        if (other.len() != 8) return false
        for (i in 0 until 8) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 36 + index] = (src).toByte()
    }
}

inline class AUTOPILOT_VERSION_os_custom_version(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 8
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 44 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 8) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: AUTOPILOT_VERSION_os_custom_version): Boolean {
        if (other.len() != 8) return false
        for (i in 0 until 8) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 44 + index] = (src).toByte()
    }
}

inline class AUTOPILOT_VERSION_capabilities(val data: Cursor) {
    inline fun get(): MAV_PROTOCOL_CAPABILITY {
        return MAV_PROTOCOL_CAPABILITY.get(get_bits(data.bytes, data.BIT, 5).toInt())
    }
}

inline class AUTOPILOT_VERSION_uid2(val data: Cursor) {

    inline fun enumerate(dst: (d0: Int) -> Unit) {
        for (d0 in 0 until AUTOPILOT_VERSION.uid2.d0)

            if (data.set_item(d0, -1)) dst(d0)
    }

    inline fun get(d0: Int): Boolean {
        return data.set_item(d0, -1)
    }
}

inline class MISSION_REQUEST_LIST_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class SET_VIDEO_STREAM_SETTINGS_uri(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: SET_VIDEO_STREAM_SETTINGS_uri): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PLAY_TUNE_tune(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PLAY_TUNE_tune): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class MISSION_REQUEST_PARTIAL_LIST_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class PARAM_EXT_ACK_param_id(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_EXT_ACK_param_id): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PARAM_EXT_ACK_param_value(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: PARAM_EXT_ACK_param_value): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PARAM_EXT_ACK_param_type(val data: Cursor) {
    inline fun get(): MAV_PARAM_EXT_TYPE {
        return MAV_PARAM_EXT_TYPE((1 + get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class PARAM_EXT_ACK_param_result(val data: Cursor) {
    inline fun get(): PARAM_ACK {
        return PARAM_ACK((get_bits(data.bytes, data.BIT, 2)).toByte())
    }
}

inline class UAVCAN_NODE_INFO_hw_unique_id(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 16
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 18 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 16) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: UAVCAN_NODE_INFO_hw_unique_id): Boolean {
        if (other.len() != 16) return false
        for (i in 0 until 16) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 18 + index] = (src).toByte()
    }
}

inline class UAVCAN_NODE_INFO_name(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: UAVCAN_NODE_INFO_name): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class DATA16_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 16
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 16) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DATA16_daTa): Boolean {
        if (other.len() != 16) return false
        for (i in 0 until 16) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class DATA64_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 64
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 64) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DATA64_daTa): Boolean {
        if (other.len() != 64) return false
        for (i in 0 until 64) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class HIL_ACTUATOR_CONTROLS_controls(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 16
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 16) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: HIL_ACTUATOR_CONTROLS_controls): Boolean {
        if (other.len() != 16) return false
        for (i in 0 until 16) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class HIL_ACTUATOR_CONTROLS_mode(val data: Cursor) {
    inline fun get(): MAV_MODE {
        return MAV_MODE.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class POSITION_TARGET_LOCAL_NED_coordinate_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class DEVICE_OP_WRITE_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 128
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 10 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 128) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DEVICE_OP_WRITE_daTa): Boolean {
        if (other.len() != 128) return false
        for (i in 0 until 128) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 10 + index] = (src).toByte()
    }
}

inline class DEVICE_OP_WRITE_bustype(val data: Cursor) {
    inline fun get(): DEVICE_OP_BUSTYPE {
        return DEVICE_OP_BUSTYPE((get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class DEVICE_OP_WRITE_busname(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DEVICE_OP_WRITE_busname): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class DISTANCE_SENSOR_typE(val data: Cursor) {
    inline fun get(): MAV_DISTANCE_SENSOR {
        return MAV_DISTANCE_SENSOR((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class DISTANCE_SENSOR_orientation(val data: Cursor) {
    inline fun get(): MAV_SENSOR_ORIENTATION {
        return MAV_SENSOR_ORIENTATION((get_bits(data.bytes, data.BIT, 6)).toByte())
    }
}

inline class CHANGE_OPERATOR_CONTROL_passkey(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: CHANGE_OPERATOR_CONTROL_passkey): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class GOPRO_SET_REQUEST_value(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GOPRO_SET_REQUEST_value): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class GOPRO_SET_REQUEST_cmd_id(val data: Cursor) {
    inline fun get(): GOPRO_COMMAND {
        return GOPRO_COMMAND((get_bits(data.bytes, data.BIT, 5)).toByte())
    }
}

inline class SYS_STATUS_onboard_control_sensors_present(val data: Cursor) {
    inline fun get(): MAV_SYS_STATUS_SENSOR {
        return MAV_SYS_STATUS_SENSOR.get(get_bits(data.bytes, data.BIT, 5).toInt())
    }
}

inline class SYS_STATUS_onboard_control_sensors_enabled(val data: Cursor) {
    inline fun get(): MAV_SYS_STATUS_SENSOR {
        return MAV_SYS_STATUS_SENSOR.get(get_bits(data.bytes, data.BIT, 5).toInt())
    }
}

inline class SYS_STATUS_onboard_control_sensors_health(val data: Cursor) {
    inline fun get(): MAV_SYS_STATUS_SENSOR {
        return MAV_SYS_STATUS_SENSOR.get(get_bits(data.bytes, data.BIT, 5).toInt())
    }
}

inline class MISSION_ITEM_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class MISSION_ITEM_command(val data: Cursor) {
    inline fun get(): MAV_CMD {
        return MAV_CMD.get(get_bits(data.bytes, data.BIT, 8).toInt())
    }
}

inline class MISSION_ITEM_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class COMMAND_INT_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class COMMAND_INT_command(val data: Cursor) {
    inline fun get(): MAV_CMD {
        return MAV_CMD.get(get_bits(data.bytes, data.BIT, 8).toInt())
    }
}

inline class MISSION_ITEM_INT_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class MISSION_ITEM_INT_command(val data: Cursor) {
    inline fun get(): MAV_CMD {
        return MAV_CMD.get(get_bits(data.bytes, data.BIT, 8).toInt())
    }
}

inline class MISSION_ITEM_INT_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class VISION_POSITION_DELTA_angle_delta(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 3
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 3) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: VISION_POSITION_DELTA_angle_delta): Boolean {
        if (other.len() != 3) return false
        for (i in 0 until 3) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16 + index * 4)
    }
}

inline class VISION_POSITION_DELTA_position_delta(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 3
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 3) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: VISION_POSITION_DELTA_position_delta): Boolean {
        if (other.len() != 3) return false
        for (i in 0 until 3) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28 + index * 4)
    }
}

inline class LOGGING_DATA_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 249
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 6 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 249) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: LOGGING_DATA_daTa): Boolean {
        if (other.len() != 249) return false
        for (i in 0 until 249) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 6 + index] = (src).toByte()
    }
}

inline class DEVICE_OP_READ_bustype(val data: Cursor) {
    inline fun get(): DEVICE_OP_BUSTYPE {
        return DEVICE_OP_BUSTYPE((get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class DEVICE_OP_READ_busname(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: DEVICE_OP_READ_busname): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class MAG_CAL_PROGRESS_completion_mask(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 10
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 4 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 10) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: MAG_CAL_PROGRESS_completion_mask): Boolean {
        if (other.len() != 10) return false
        for (i in 0 until 10) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 4 + index] = (src).toByte()
    }
}

inline class MAG_CAL_PROGRESS_cal_status(val data: Cursor) {
    inline fun get(): MAG_CAL_STATUS {
        return MAG_CAL_STATUS((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class EXTENDED_SYS_STATE_vtol_state(val data: Cursor) {
    inline fun get(): MAV_VTOL_STATE {
        return MAV_VTOL_STATE((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class EXTENDED_SYS_STATE_landed_state(val data: Cursor) {
    inline fun get(): MAV_LANDED_STATE {
        return MAV_LANDED_STATE((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class UAVIONIX_ADSB_OUT_DYNAMIC_gpsFix(val data: Cursor) {
    inline fun get(): UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX {
        return UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class UAVIONIX_ADSB_OUT_DYNAMIC_emergencyStatus(val data: Cursor) {
    inline fun get(): UAVIONIX_ADSB_EMERGENCY_STATUS {
        return UAVIONIX_ADSB_EMERGENCY_STATUS((get_bits(data.bytes, data.BIT, 3)).toByte())
    }
}

inline class UAVIONIX_ADSB_OUT_DYNAMIC_state(val data: Cursor) {
    inline fun get(): UAVIONIX_ADSB_OUT_DYNAMIC_STATE {
        return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class GOPRO_GET_RESPONSE_value(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 0 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GOPRO_GET_RESPONSE_value): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 0 + index] = (src).toByte()
    }
}

inline class GOPRO_GET_RESPONSE_cmd_id(val data: Cursor) {
    inline fun get(): GOPRO_COMMAND {
        return GOPRO_COMMAND((get_bits(data.bytes, data.BIT, 5)).toByte())
    }
}

inline class GOPRO_GET_RESPONSE_status(val data: Cursor) {
    inline fun get(): GOPRO_REQUEST_STATUS {
        return GOPRO_REQUEST_STATUS((get_bits(data.bytes, data.BIT, 1)).toByte())
    }
}

inline class GPS_INJECT_DATA_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 110
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 3 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 110) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GPS_INJECT_DATA_daTa): Boolean {
        if (other.len() != 110) return false
        for (i in 0 until 110) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 3 + index] = (src).toByte()
    }
}

inline class UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_rfHealth(val data: Cursor) {
    inline fun get(): UAVIONIX_ADSB_RF_HEALTH {
        return UAVIONIX_ADSB_RF_HEALTH.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class ATTITUDE_QUATERNION_COV_q(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 4
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 4) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: ATTITUDE_QUATERNION_COV_q): Boolean {
        if (other.len() != 4) return false
        for (i in 0 until 4) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class ATTITUDE_QUATERNION_COV_covariance(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 9
    }

    inline fun get(index: Int): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36 + index * 4, 4).toInt())
    }

    fun same(other: FloatArray): Boolean {
        if (other.size != 9) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: ATTITUDE_QUATERNION_COV_covariance): Boolean {
        if (other.len() != 9) return false
        for (i in 0 until 9) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }
}

inline class NAMED_VALUE_INT_name(val data: Cursor) {
    inline fun len(): Int {
        return data.item_len
    }

    inline fun get(index: Int): Byte {
        return data.bytes[data.BYTE + index]
    }

    inline fun get(): String {
        return data.decode()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != len()) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: NAMED_VALUE_INT_name): Boolean {
        if (other.len() != len()) return false
        for (i in 0 until len()) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class GPS_RTCM_DATA_daTa(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 180
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 180) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: GPS_RTCM_DATA_daTa): Boolean {
        if (other.len() != 180) return false
        for (i in 0 until 180) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class FILE_TRANSFER_PROTOCOL_payload(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 251
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 3 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 251) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: FILE_TRANSFER_PROTOCOL_payload): Boolean {
        if (other.len() != 251) return false
        for (i in 0 until 251) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 3 + index] = (src).toByte()
    }
}

inline class RESOURCE_REQUEST_uri(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 120
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 2 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 120) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: RESOURCE_REQUEST_uri): Boolean {
        if (other.len() != 120) return false
        for (i in 0 until 120) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 2 + index] = (src).toByte()
    }
}

inline class RESOURCE_REQUEST_storage(val data: Pack.Bytes) {
    inline fun len(): Int {
        return 120
    }

    inline fun get(index: Int): Byte {
        return (data.bytes[data.origin + 123 + index]).toByte()
    }

    fun same(other: ByteArray): Boolean {
        if (other.size != 120) return false
        other.forEachIndexed { i, v -> if (v != get(i)) return false }
        return true
    }

    fun same(other: RESOURCE_REQUEST_storage): Boolean {
        if (other.len() != 120) return false
        for (i in 0 until 120) {
            if (other.get(i) != get(i)) return false
        }
        return true
    }

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 123 + index] = (src).toByte()
    }
}

const val RECEIVE_REQ_MAX_BYTES = 255
const val RECEIVE_FULL_MAX_BYTES = 6850

const val SEND_REQ_MAX_BYTES = 255
const val SEND_FULL_MAX_BYTES = 6850
