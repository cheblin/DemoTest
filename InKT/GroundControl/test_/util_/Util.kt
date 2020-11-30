package test_.util_

import com.company.demo.GroundControl.util_.*
import org.unirail.AdHoc.*
import org.unirail.AdHoc.Pack.Cursor
import test_.*

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

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 5 + index * 4)
    }
}

inline class MISSION_COUNT_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
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

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36 + index * 4)
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 1 + index] = (src).toByte()
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 21 + index] = (src).toByte()
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 41 + index] = (src).toByte()
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 61 + index] = (src).toByte()
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.origin + 81 + index] = (src).toByte()
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PARAM_SET_param_type(val data: Cursor) {
    inline fun get(): MAV_PARAM_TYPE {
        return MAV_PARAM_TYPE((1 + get_bits(data.bytes, data.BIT, 4)).toByte())
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
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

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 7 + index * 4)
    }
}

inline class HIL_CONTROLS_mode(val data: Cursor) {
    inline fun get(): MAV_MODE {
        return MAV_MODE.get(get_bits(data.bytes, data.BIT, 4).toInt())
    }
}

inline class SET_POSITION_TARGET_GLOBAL_INT_coordinate_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
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

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44 + index * 4)
    }
}

inline class LOCAL_POSITION_NED_COV_estimator_type(val data: Cursor) {
    inline fun get(): MAV_ESTIMATOR_TYPE {
        return MAV_ESTIMATOR_TYPE((1 + get_bits(data.bytes, data.BIT, 3)).toByte())
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

inline class MISSION_WRITE_PARTIAL_LIST_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class SAFETY_ALLOWED_AREA_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class MISSION_CLEAR_ALL_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
    }
}

inline class PARAM_VALUE_param_type(val data: Cursor) {
    inline fun get(): MAV_PARAM_TYPE {
        return MAV_PARAM_TYPE((1 + get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class SET_POSITION_TARGET_LOCAL_NED_coordinate_frame(val data: Cursor) {
    inline fun get(): MAV_FRAME {
        return MAV_FRAME((get_bits(data.bytes, data.BIT, 4)).toByte())
    }
}

inline class MISSION_REQUEST_LIST_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
    }
}

inline class MISSION_REQUEST_PARTIAL_LIST_mission_type(val data: Cursor) {
    inline fun get(): MAV_MISSION_TYPE {
        return MAV_MISSION_TYPE.get(get_bits(data.bytes, data.BIT, 3).toInt())
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

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16 + index * 4)
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

    inline fun set(src: Byte, index: Int) {
        data.bytes[data.BYTE + index] = src
    }

    inline fun set(src: String, reuse: CharArray?) {
        data.encode(src, data.shift, reuse)
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

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8 + index * 4)
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

    inline fun set(src: Float, index: Int) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36 + index * 4)
    }
}
