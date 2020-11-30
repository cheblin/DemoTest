package test_

import org.unirail.AdHoc.*
import org.unirail.AdHoc
import org.unirail.AdHoc.Pack.Cursor
import com.company.demo.GroundControl.*
import demo_.*
import java.util.*
import kotlin.experimental.*
import test_.util_.*

inline class ATTITUDE_TARGET(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun type_mask(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun type_mask_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun q(): ATTITUDE_TARGET_q {

        return ATTITUDE_TARGET_q(data)
    }

    inline fun q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 5 + index * 4)
    }

    object q {
        const val item_len = 4


    }

    inline fun body_roll_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 21, 4).toInt())
    }

    inline fun body_roll_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 21)
    }

    inline fun body_pitch_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 25, 4).toInt())
    }

    inline fun body_pitch_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 25)
    }

    inline fun body_yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 29, 4).toInt())
    }

    inline fun body_yaw_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 29)
    }

    inline fun thrust(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 33, 4).toInt())
    }

    inline fun thrust_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 33)
    }

}

inline class MISSION_COUNT(val data: Cursor) {

    inline fun count(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun count_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }


    inline fun mission_type(): MISSION_COUNT_mission_type? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return MISSION_COUNT_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 32) data.set_field(32, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class GLOBAL_POSITION_INT_COV(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 12, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun relative_alt(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }

    inline fun relative_alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun vx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun vy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun vz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun covariance(): GLOBAL_POSITION_INT_COV_covariance {

        return GLOBAL_POSITION_INT_COV_covariance(data)
    }

    inline fun covariance_(src: FloatArray) {
        val len = minOf(src.size, 36)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 36 + index * 4)
    }

    object covariance {
        const val item_len = 36


    }


    inline fun estimator_type(): GLOBAL_POSITION_INT_COV_estimator_type? {
        if ((data.field_bit != 1440 && !data.set_field(1440, -1))) return null

        return GLOBAL_POSITION_INT_COV_estimator_type(data)
    }


    inline fun estimator_type(src: MAV_ESTIMATOR_TYPE) {
        if (data.field_bit != 1440) data.set_field(1440, 0)

        set_bits((-1 src . value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


}

inline class SAFETY_SET_ALLOWED_AREA(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun p1x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }

    inline fun p1x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun p1y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun p1y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun p1z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }

    inline fun p1z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun p2x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun p2x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun p2y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun p2y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun p2z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun p2z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }


    inline fun frame(): SAFETY_SET_ALLOWED_AREA_frame? {
        if ((data.field_bit != 208 && !data.set_field(208, -1))) return null

        return SAFETY_SET_ALLOWED_AREA_frame(data)
    }


    inline fun frame(src: MAV_FRAME) {
        if (data.field_bit != 208) data.set_field(208, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class GPS_STATUS(val data: Pack.Bytes) {

    inline fun satellites_visible(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun satellites_visible_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun satellite_prn(): GPS_STATUS_satellite_prn {

        return GPS_STATUS_satellite_prn(data)
    }

    inline fun satellite_prn_(src: ByteArray) {
        val len = minOf(src.size, 20)

        for (index in 0 until len)
            data.bytes[data.origin + 1 + index] = (src[index]).toByte()
    }

    object satellite_prn {
        const val item_len = 20


    }

    inline fun satellite_used(): GPS_STATUS_satellite_used {

        return GPS_STATUS_satellite_used(data)
    }

    inline fun satellite_used_(src: ByteArray) {
        val len = minOf(src.size, 20)

        for (index in 0 until len)
            data.bytes[data.origin + 21 + index] = (src[index]).toByte()
    }

    object satellite_used {
        const val item_len = 20


    }

    inline fun satellite_elevation(): GPS_STATUS_satellite_elevation {

        return GPS_STATUS_satellite_elevation(data)
    }

    inline fun satellite_elevation_(src: ByteArray) {
        val len = minOf(src.size, 20)

        for (index in 0 until len)
            data.bytes[data.origin + 41 + index] = (src[index]).toByte()
    }

    object satellite_elevation {
        const val item_len = 20


    }

    inline fun satellite_azimuth(): GPS_STATUS_satellite_azimuth {

        return GPS_STATUS_satellite_azimuth(data)
    }

    inline fun satellite_azimuth_(src: ByteArray) {
        val len = minOf(src.size, 20)

        for (index in 0 until len)
            data.bytes[data.origin + 61 + index] = (src[index]).toByte()
    }

    object satellite_azimuth {
        const val item_len = 20


    }

    inline fun satellite_snr(): GPS_STATUS_satellite_snr {

        return GPS_STATUS_satellite_snr(data)
    }

    inline fun satellite_snr_(src: ByteArray) {
        val len = minOf(src.size, 20)

        for (index in 0 until len)
            data.bytes[data.origin + 81 + index] = (src[index]).toByte()
    }

    object satellite_snr {
        const val item_len = 20


    }

}

inline class PARAM_SET(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun param_value(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }

    inline fun param_value_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }


    inline fun param_id(): PARAM_SET_param_id? {
        if ((data.field_bit != 50 && !data.set_field(50, -1))) return null

        return PARAM_SET_param_id(data)
    }


    inline fun param_id_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -50 - 1, reuse)
    }


    inline fun param_id_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(50, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_id_(len: Int): PARAM_SET_param_id {

        data.set_field(50, minOf(len, 255))
        return PARAM_SET_param_id(data)
    }

    object param_id {
        const val item_len_max = 255

    }


    inline fun param_type(): PARAM_SET_param_type? {
        if ((data.field_bit != 51 && !data.set_field(51, -1))) return null

        return PARAM_SET_param_type(data)
    }


    inline fun param_type(src: MAV_PARAM_TYPE) {
        if (data.field_bit != 51) data.set_field(51, 0)

        set_bits((-1 src . value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class RC_CHANNELS_OVERRIDE(val data: Pack.Bytes) {

    inline fun chan1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun chan1_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun chan2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun chan2_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun chan3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun chan3_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun chan4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun chan4_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun chan5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun chan5_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun chan6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun chan6_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun chan7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun chan7_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun chan8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun chan8_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 16] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 17]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 17] = (src).toByte()
    }

}

inline class SCALED_IMU(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun xacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun yacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun zacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun xgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun xgyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun ygyro(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun ygyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun zgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun zgyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun xmag(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }

    inline fun xmag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 16)
    }

    inline fun ymag(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }

    inline fun ymag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 18)
    }

    inline fun zmag(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }

    inline fun zmag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 20)
    }

}

inline class PARAM_REQUEST_READ(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun param_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun param_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }


    inline fun param_id(): PARAM_REQUEST_READ_param_id? {
        if ((data.field_bit != 34 && !data.set_field(34, -1))) return null

        return PARAM_REQUEST_READ_param_id(data)
    }


    inline fun param_id_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -34 - 1, reuse)
    }


    inline fun param_id_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(34, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_id_(len: Int): PARAM_REQUEST_READ_param_id {

        data.set_field(34, minOf(len, 255))
        return PARAM_REQUEST_READ_param_id(data)
    }

    object param_id {
        const val item_len_max = 255

    }

}

inline class SET_ATTITUDE_TARGET(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun type_mask(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun type_mask_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun q(): SET_ATTITUDE_TARGET_q {

        return SET_ATTITUDE_TARGET_q(data)
    }

    inline fun q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 7 + index * 4)
    }

    object q {
        const val item_len = 4


    }

    inline fun body_roll_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 23, 4).toInt())
    }

    inline fun body_roll_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 23)
    }

    inline fun body_pitch_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 27, 4).toInt())
    }

    inline fun body_pitch_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 27)
    }

    inline fun body_yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 31, 4).toInt())
    }

    inline fun body_yaw_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 31)
    }

    inline fun thrust(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 35, 4).toInt())
    }

    inline fun thrust_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 35)
    }

}

inline class HIL_STATE(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun rollspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun pitchspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun yawspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 32, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 36, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 40, 4)).toInt()
    }

    inline fun alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun vx(): Short {
        return (get_bytes(data.bytes, data.origin + 44, 2)).toShort()
    }

    inline fun vx_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 44)
    }

    inline fun vy(): Short {
        return (get_bytes(data.bytes, data.origin + 46, 2)).toShort()
    }

    inline fun vy_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 46)
    }

    inline fun vz(): Short {
        return (get_bytes(data.bytes, data.origin + 48, 2)).toShort()
    }

    inline fun vz_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 48)
    }

    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 50, 2)).toShort()
    }

    inline fun xacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 50)
    }

    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 52, 2)).toShort()
    }

    inline fun yacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 52)
    }

    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 54, 2)).toShort()
    }

    inline fun zacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 54)
    }

}

inline class REQUEST_DATA_STREAM(val data: Pack.Bytes) {

    inline fun req_message_rate(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun req_message_rate_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun req_stream_id(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun req_stream_id_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun start_stop(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun start_stop_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

}

inline class HIL_CONTROLS(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun roll_ailerons(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun roll_ailerons_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun pitch_elevator(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun pitch_elevator_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun yaw_rudder(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun yaw_rudder_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun throttle(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun throttle_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun aux1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun aux1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun aux2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun aux2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun aux3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun aux3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun aux4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun aux4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun nav_mode(): Byte {
        return (data.bytes[data.origin + 40]).toByte()
    }

    inline fun nav_mode_(src: Byte) {
        data.bytes[data.origin + 40] = (src).toByte()
    }


    inline fun mode(): HIL_CONTROLS_mode? {
        if ((data.field_bit != 328 && !data.set_field(328, -1))) return null

        return HIL_CONTROLS_mode(data)
    }


    inline fun mode(src: MAV_MODE) {
        if (data.field_bit != 328) data.set_field(328, 0)

        set_bits(MAV_MODE.set(src).toLong(), 4, data.bytes, data.BIT)
    }


}

inline class PARAM_REQUEST_LIST(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

}

inline class SET_POSITION_TARGET_GLOBAL_INT(val data: Cursor) {

    inline fun type_mask(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun type_mask_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 7] = (src).toByte()
    }

    inline fun lat_int(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }

    inline fun lat_int_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun lon_int(): Int {
        return (get_bytes(data.bytes, data.origin + 12, 4)).toInt()
    }

    inline fun lon_int_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun vx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun vy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun vz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun afx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun afx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun afy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun afy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun afz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun afz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }

    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48, 4).toInt())
    }

    inline fun yaw_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 48)
    }


    inline fun coordinate_frame(): SET_POSITION_TARGET_GLOBAL_INT_coordinate_frame? {
        if ((data.field_bit != 416 && !data.set_field(416, -1))) return null

        return SET_POSITION_TARGET_GLOBAL_INT_coordinate_frame(data)
    }


    inline fun coordinate_frame(src: MAV_FRAME) {
        if (data.field_bit != 416) data.set_field(416, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class VFR_HUD(val data: Pack.Bytes) {

    inline fun throttle(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun throttle_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun airspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }

    inline fun airspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun groundspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun groundspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun heading(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun heading_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun climb(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun climb_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

}

inline class MISSION_SET_CURRENT(val data: Pack.Bytes) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun seq_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

}

inline class NAV_CONTROLLER_OUTPUT(val data: Pack.Bytes) {

    inline fun wp_dist(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun wp_dist_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun nav_roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }

    inline fun nav_roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun nav_pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun nav_pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun nav_bearing(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun nav_bearing_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun target_bearing(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun target_bearing_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun alt_error(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun alt_error_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun aspd_error(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun aspd_error_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun xtrack_error(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun xtrack_error_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

}

inline class AUTH_KEY(val data: Cursor) {


    inline fun key(): AUTH_KEY_key? {
        if ((data.field_bit != 2 && !data.set_field(2, -1))) return null

        return AUTH_KEY_key(data)
    }


    inline fun key_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -2 - 1, reuse)
    }


    inline fun key_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(2, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun key_(len: Int): AUTH_KEY_key {

        data.set_field(2, minOf(len, 255))
        return AUTH_KEY_key(data)
    }

    object key {
        const val item_len_max = 255

    }

}

inline class LOCAL_POSITION_NED_COV(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun vx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun vy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun vz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun ax(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun ax_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun ay(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun ay_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun az(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun az_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun covariance(): LOCAL_POSITION_NED_COV_covariance {

        return LOCAL_POSITION_NED_COV_covariance(data)
    }

    inline fun covariance_(src: FloatArray) {
        val len = minOf(src.size, 45)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 44 + index * 4)
    }

    object covariance {
        const val item_len = 45


    }


    inline fun estimator_type(): LOCAL_POSITION_NED_COV_estimator_type? {
        if ((data.field_bit != 1792 && !data.set_field(1792, -1))) return null

        return LOCAL_POSITION_NED_COV_estimator_type(data)
    }


    inline fun estimator_type(src: MAV_ESTIMATOR_TYPE) {
        if (data.field_bit != 1792) data.set_field(1792, 0)

        set_bits((-1 src . value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


}

inline class PING(val data: Pack.Bytes) {

    inline fun seq(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun seq_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 4)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 12]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 12] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 13]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 13] = (src).toByte()
    }

}

inline class GLOBAL_POSITION_INT(val data: Pack.Bytes) {

    inline fun hdg(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun hdg_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 14, 4)).toInt()
    }

    inline fun alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun relative_alt(): Int {
        return (get_bytes(data.bytes, data.origin + 18, 4)).toInt()
    }

    inline fun relative_alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun vx(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }

    inline fun vx_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 22)
    }

    inline fun vy(): Short {
        return (get_bytes(data.bytes, data.origin + 24, 2)).toShort()
    }

    inline fun vy_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 24)
    }

    inline fun vz(): Short {
        return (get_bytes(data.bytes, data.origin + 26, 2)).toShort()
    }

    inline fun vz_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 26)
    }

}

inline class COMMAND_LONG(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun confirmation(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun confirmation_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun param1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 3, 4).toInt())
    }

    inline fun param1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 3)
    }

    inline fun param2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 7, 4).toInt())
    }

    inline fun param2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 7)
    }

    inline fun param3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 11, 4).toInt())
    }

    inline fun param3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 11)
    }

    inline fun param4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 15, 4).toInt())
    }

    inline fun param4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 15)
    }

    inline fun param5(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 19, 4).toInt())
    }

    inline fun param5_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 19)
    }

    inline fun param6(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 23, 4).toInt())
    }

    inline fun param6_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 23)
    }

    inline fun param7(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 27, 4).toInt())
    }

    inline fun param7_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 27)
    }


    inline fun command(): COMMAND_LONG_command? {
        if ((data.field_bit != 250 && !data.set_field(250, -1))) return null

        return COMMAND_LONG_command(data)
    }


    inline fun command(src: MAV_CMD) {
        if (data.field_bit != 250) data.set_field(250, 0)

        set_bits(MAV_CMD.set(src).toLong(), 8, data.bytes, data.BIT)
    }


}

inline class GPS_RAW_INT(val data: Cursor) {

    inline fun eph(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun eph_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun epv(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun epv_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun vel(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun vel_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun cog(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun cog_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 24, 4)).toInt()
    }

    inline fun alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun satellites_visible(): Byte {
        return (data.bytes[data.origin + 28]).toByte()
    }

    inline fun satellites_visible_(src: Byte) {
        data.bytes[data.origin + 28] = (src).toByte()
    }


    inline fun fix_type(): GPS_RAW_INT_fix_type? {
        if ((data.field_bit != 234 && !data.set_field(234, -1))) return null

        return GPS_RAW_INT_fix_type(data)
    }


    inline fun fix_type(src: GPS_FIX_TYPE) {
        if (data.field_bit != 234) data.set_field(234, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    inline fun alt_ellipsoid(): Boolean {
        return !(data.field_bit != 235 && !data.set_field(235, -1))
    }


    inline fun alt_ellipsoid_() {
        if (data.field_bit != 235) data.set_field(235, 0)
    }


    inline fun h_acc(): Boolean {
        return !(data.field_bit != 236 && !data.set_field(236, -1))
    }


    inline fun h_acc_() {
        if (data.field_bit != 236) data.set_field(236, 0)
    }


    inline fun v_acc(): Boolean {
        return !(data.field_bit != 237 && !data.set_field(237, -1))
    }


    inline fun v_acc_() {
        if (data.field_bit != 237) data.set_field(237, 0)
    }


    inline fun vel_acc(): Boolean {
        return !(data.field_bit != 238 && !data.set_field(238, -1))
    }


    inline fun vel_acc_() {
        if (data.field_bit != 238) data.set_field(238, 0)
    }


    inline fun hdg_acc(): Boolean {
        return !(data.field_bit != 239 && !data.set_field(239, -1))
    }


    inline fun hdg_acc_() {
        if (data.field_bit != 239) data.set_field(239, 0)
    }


}

inline class RC_CHANNELS_SCALED(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun port(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun port_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun chan1_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 5, 2)).toShort()
    }

    inline fun chan1_scaled_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 5)
    }

    inline fun chan2_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 7, 2)).toShort()
    }

    inline fun chan2_scaled_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 7)
    }

    inline fun chan3_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 9, 2)).toShort()
    }

    inline fun chan3_scaled_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 9)
    }

    inline fun chan4_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 11, 2)).toShort()
    }

    inline fun chan4_scaled_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 11)
    }

    inline fun chan5_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 13, 2)).toShort()
    }

    inline fun chan5_scaled_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 13)
    }

    inline fun chan6_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 15, 2)).toShort()
    }

    inline fun chan6_scaled_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 15)
    }

    inline fun chan7_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 17, 2)).toShort()
    }

    inline fun chan7_scaled_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 17)
    }

    inline fun chan8_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 19, 2)).toShort()
    }

    inline fun chan8_scaled_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 19)
    }

    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 21]).toByte()
    }

    inline fun rssi_(src: Byte) {
        data.bytes[data.origin + 21] = (src).toByte()
    }

}

inline class RAW_PRESSURE(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun press_abs(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun press_abs_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun press_diff1(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun press_diff1_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun press_diff2(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun press_diff2_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun temperature_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

}

inline class ATTITUDE(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun rollspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun pitchspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun yawspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

}

inline class MISSION_WRITE_PARTIAL_LIST(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun start_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun start_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun end_index(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun end_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }


    inline fun mission_type(): MISSION_WRITE_PARTIAL_LIST_mission_type? {
        if ((data.field_bit != 48 && !data.set_field(48, -1))) return null

        return MISSION_WRITE_PARTIAL_LIST_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 48) data.set_field(48, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class MANUAL_SETPOINT(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun thrust(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun thrust_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun mode_switch(): Byte {
        return (data.bytes[data.origin + 20]).toByte()
    }

    inline fun mode_switch_(src: Byte) {
        data.bytes[data.origin + 20] = (src).toByte()
    }

    inline fun manual_override_switch(): Byte {
        return (data.bytes[data.origin + 21]).toByte()
    }

    inline fun manual_override_switch_(src: Byte) {
        data.bytes[data.origin + 21] = (src).toByte()
    }

}

inline class SAFETY_ALLOWED_AREA(val data: Cursor) {

    inline fun p1x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun p1x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun p1y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun p1y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun p1z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun p1z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun p2x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun p2x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun p2y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun p2y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun p2z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun p2z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }


    inline fun frame(): SAFETY_ALLOWED_AREA_frame? {
        if ((data.field_bit != 192 && !data.set_field(192, -1))) return null

        return SAFETY_ALLOWED_AREA_frame(data)
    }


    inline fun frame(src: MAV_FRAME) {
        if (data.field_bit != 192) data.set_field(192, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class MISSION_CLEAR_ALL(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    inline fun mission_type(): MISSION_CLEAR_ALL_mission_type? {
        if ((data.field_bit != 16 && !data.set_field(16, -1))) return null

        return MISSION_CLEAR_ALL_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 16) data.set_field(16, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class SCALED_PRESSURE(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun press_abs(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun press_abs_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun press_diff(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun press_diff_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun temperature_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

}

inline class HEARTBEAT(val data: Cursor) {

    inline fun custom_mode(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun custom_mode_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun mavlink_version(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun mavlink_version_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }


    inline fun typE(): HEARTBEAT_typE? {
        if ((data.field_bit != 42 && !data.set_field(42, -1))) return null

        return HEARTBEAT_typE(data)
    }


    inline fun typE(src: MAV_TYPE) {
        if (data.field_bit != 42) data.set_field(42, 0)

        set_bits((src.value).toULong().toLong(), 5, data.bytes, data.BIT)
    }


    inline fun autopilot(): HEARTBEAT_autopilot? {
        if ((data.field_bit != 43 && !data.set_field(43, -1))) return null

        return HEARTBEAT_autopilot(data)
    }


    inline fun autopilot(src: MAV_AUTOPILOT) {
        if (data.field_bit != 43) data.set_field(43, 0)

        set_bits((src.value).toULong().toLong(), 5, data.bytes, data.BIT)
    }


    inline fun base_mode(): HEARTBEAT_base_mode? {
        if ((data.field_bit != 44 && !data.set_field(44, -1))) return null

        return HEARTBEAT_base_mode(data)
    }


    inline fun base_mode(src: MAV_MODE_FLAG) {
        if (data.field_bit != 44) data.set_field(44, 0)

        set_bits(MAV_MODE_FLAG.set(src).toLong(), 4, data.bytes, data.BIT)
    }


    inline fun system_status(): HEARTBEAT_system_status? {
        if ((data.field_bit != 45 && !data.set_field(45, -1))) return null

        return HEARTBEAT_system_status(data)
    }


    inline fun system_status(src: MAV_STATE) {
        if (data.field_bit != 45) data.set_field(45, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class PARAM_MAP_RC(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun param_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun param_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun parameter_rc_channel_index(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun parameter_rc_channel_index_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun param_value0(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 5, 4).toInt())
    }

    inline fun param_value0_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 5)
    }

    inline fun scale(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 9, 4).toInt())
    }

    inline fun scale_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 9)
    }

    inline fun param_value_min(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 13, 4).toInt())
    }

    inline fun param_value_min_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 13)
    }

    inline fun param_value_max(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 17, 4).toInt())
    }

    inline fun param_value_max_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }


    inline fun param_id(): PARAM_MAP_RC_param_id? {
        if ((data.field_bit != 170 && !data.set_field(170, -1))) return null

        return PARAM_MAP_RC_param_id(data)
    }


    inline fun param_id_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -170 - 1, reuse)
    }


    inline fun param_id_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(170, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_id_(len: Int): PARAM_MAP_RC_param_id {

        data.set_field(170, minOf(len, 255))
        return PARAM_MAP_RC_param_id(data)
    }

    object param_id {
        const val item_len_max = 255

    }

}

inline class MISSION_REQUEST_INT(val data: Cursor) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun seq_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }


    inline fun mission_type(): MISSION_REQUEST_INT_mission_type? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return MISSION_REQUEST_INT_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 32) data.set_field(32, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

}

inline class COMMAND_ACK(val data: Cursor) {


    inline fun command(): COMMAND_ACK_command? {
        if ((data.field_bit != 2 && !data.set_field(2, -1))) return null

        return COMMAND_ACK_command(data)
    }


    inline fun command(src: MAV_CMD) {
        if (data.field_bit != 2) data.set_field(2, 0)

        set_bits(MAV_CMD.set(src).toLong(), 8, data.bytes, data.BIT)
    }


    inline fun result(): COMMAND_ACK_result? {
        if ((data.field_bit != 3 && !data.set_field(3, -1))) return null

        return COMMAND_ACK_result(data)
    }


    inline fun result(src: MAV_RESULT) {
        if (data.field_bit != 3) data.set_field(3, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun progress(): Boolean {
        return !(data.field_bit != 4 && !data.set_field(4, -1))
    }


    inline fun progress_() {
        if (data.field_bit != 4) data.set_field(4, 0)
    }


    inline fun result_param2(): Boolean {
        return !(data.field_bit != 5 && !data.set_field(5, -1))
    }


    inline fun result_param2_() {
        if (data.field_bit != 5) data.set_field(5, 0)
    }


    inline fun target_system(): Boolean {
        return !(data.field_bit != 6 && !data.set_field(6, -1))
    }


    inline fun target_system_() {
        if (data.field_bit != 6) data.set_field(6, 0)
    }


    inline fun target_component(): Boolean {
        return !(data.field_bit != 7 && !data.set_field(7, -1))
    }


    inline fun target_component_() {
        if (data.field_bit != 7) data.set_field(7, 0)
    }


}

inline class DATA_STREAM(val data: Pack.Bytes) {

    inline fun message_rate(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun message_rate_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun stream_id(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun stream_id_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun on_off(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun on_off_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

}

inline class MISSION_REQUEST(val data: Cursor) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun seq_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }


    inline fun mission_type(): MISSION_REQUEST_mission_type? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return MISSION_REQUEST_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 32) data.set_field(32, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class HIL_RC_INPUTS_RAW(val data: Pack.Bytes) {

    inline fun chan1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun chan1_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun chan2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun chan2_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun chan3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun chan3_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun chan4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun chan4_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun chan5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun chan5_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun chan6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun chan6_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun chan7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun chan7_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun chan8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun chan8_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun chan9_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }

    inline fun chan9_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 16)
    }

    inline fun chan10_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }

    inline fun chan10_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 18)
    }

    inline fun chan11_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }

    inline fun chan11_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 20)
    }

    inline fun chan12_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }

    inline fun chan12_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 22)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 24, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 24)
    }

    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 32]).toByte()
    }

    inline fun rssi_(src: Byte) {
        data.bytes[data.origin + 32] = (src).toByte()
    }

}

inline class SET_MODE(val data: Cursor) {

    inline fun custom_mode(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun custom_mode_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }


    inline fun base_mode(): SET_MODE_base_mode? {
        if ((data.field_bit != 40 && !data.set_field(40, -1))) return null

        return SET_MODE_base_mode(data)
    }


    inline fun base_mode(src: MAV_MODE) {
        if (data.field_bit != 40) data.set_field(40, 0)

        set_bits(MAV_MODE.set(src).toLong(), 4, data.bytes, data.BIT)
    }


}

inline class POSITION_TARGET_GLOBAL_INT(val data: Cursor) {

    inline fun type_mask(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun type_mask_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun lat_int(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun lat_int_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun lon_int(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun lon_int_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun vx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun vy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }

    inline fun vz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 26)
    }

    inline fun afx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }

    inline fun afx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 30)
    }

    inline fun afy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 34, 4).toInt())
    }

    inline fun afy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 34)
    }

    inline fun afz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 38, 4).toInt())
    }

    inline fun afz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 38)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 42, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 42)
    }

    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 46, 4).toInt())
    }

    inline fun yaw_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 46)
    }


    inline fun coordinate_frame(): POSITION_TARGET_GLOBAL_INT_coordinate_frame? {
        if ((data.field_bit != 400 && !data.set_field(400, -1))) return null

        return POSITION_TARGET_GLOBAL_INT_coordinate_frame(data)
    }


    inline fun coordinate_frame(src: MAV_FRAME) {
        if (data.field_bit != 400) data.set_field(400, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class RC_CHANNELS_RAW(val data: Pack.Bytes) {

    inline fun chan1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun chan1_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun chan2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun chan2_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun chan3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun chan3_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun chan4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun chan4_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun chan5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun chan5_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun chan6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun chan6_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun chan7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun chan7_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun chan8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun chan8_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun port(): Byte {
        return (data.bytes[data.origin + 20]).toByte()
    }

    inline fun port_(src: Byte) {
        data.bytes[data.origin + 20] = (src).toByte()
    }

    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 21]).toByte()
    }

    inline fun rssi_(src: Byte) {
        data.bytes[data.origin + 21] = (src).toByte()
    }

}

inline class SERVO_OUTPUT_RAW(val data: Cursor) {

    inline fun servo1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun servo1_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun servo2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun servo2_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun servo3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun servo3_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun servo4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun servo4_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun servo5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun servo5_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun servo6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun servo6_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun servo7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun servo7_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun servo8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun servo8_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun time_usec(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun time_usec_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun port(): Byte {
        return (data.bytes[data.origin + 20]).toByte()
    }

    inline fun port_(src: Byte) {
        data.bytes[data.origin + 20] = (src).toByte()
    }


    inline fun servo9_raw(): Boolean {
        return !(data.field_bit != 168 && !data.set_field(168, -1))
    }


    inline fun servo9_raw_() {
        if (data.field_bit != 168) data.set_field(168, 0)
    }


    inline fun servo10_raw(): Boolean {
        return !(data.field_bit != 169 && !data.set_field(169, -1))
    }


    inline fun servo10_raw_() {
        if (data.field_bit != 169) data.set_field(169, 0)
    }


    inline fun servo11_raw(): Boolean {
        return !(data.field_bit != 170 && !data.set_field(170, -1))
    }


    inline fun servo11_raw_() {
        if (data.field_bit != 170) data.set_field(170, 0)
    }


    inline fun servo12_raw(): Boolean {
        return !(data.field_bit != 171 && !data.set_field(171, -1))
    }


    inline fun servo12_raw_() {
        if (data.field_bit != 171) data.set_field(171, 0)
    }


    inline fun servo13_raw(): Boolean {
        return !(data.field_bit != 172 && !data.set_field(172, -1))
    }


    inline fun servo13_raw_() {
        if (data.field_bit != 172) data.set_field(172, 0)
    }


    inline fun servo14_raw(): Boolean {
        return !(data.field_bit != 173 && !data.set_field(173, -1))
    }


    inline fun servo14_raw_() {
        if (data.field_bit != 173) data.set_field(173, 0)
    }


    inline fun servo15_raw(): Boolean {
        return !(data.field_bit != 174 && !data.set_field(174, -1))
    }


    inline fun servo15_raw_() {
        if (data.field_bit != 174) data.set_field(174, 0)
    }


    inline fun servo16_raw(): Boolean {
        return !(data.field_bit != 175 && !data.set_field(175, -1))
    }


    inline fun servo16_raw_() {
        if (data.field_bit != 175) data.set_field(175, 0)
    }


}

inline class MISSION_ITEM_REACHED(val data: Pack.Bytes) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun seq_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

}

inline class MISSION_ACK(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    inline fun typE(): MISSION_ACK_typE? {
        if ((data.field_bit != 18 && !data.set_field(18, -1))) return null

        return MISSION_ACK_typE(data)
    }


    inline fun typE(src: MAV_MISSION_RESULT) {
        if (data.field_bit != 18) data.set_field(18, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    inline fun mission_type(): MISSION_ACK_mission_type? {
        if ((data.field_bit != 19 && !data.set_field(19, -1))) return null

        return MISSION_ACK_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 19) data.set_field(19, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class CHANGE_OPERATOR_CONTROL_ACK(val data: Pack.Bytes) {

    inline fun gcs_system_id(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun gcs_system_id_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun control_request(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun control_request_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun ack(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun ack_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

}

inline class MISSION_CURRENT(val data: Pack.Bytes) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun seq_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

}

inline class SYSTEM_TIME(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun time_unix_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }

    inline fun time_unix_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 4)
    }

}

inline class VISION_POSITION_ESTIMATE(val data: Pack.Bytes) {

    inline fun usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

}

inline class MANUAL_CONTROL(val data: Pack.Bytes) {

    inline fun buttons(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun buttons_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun x(): Short {
        return (get_bytes(data.bytes, data.origin + 3, 2)).toShort()
    }

    inline fun x_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 3)
    }

    inline fun y(): Short {
        return (get_bytes(data.bytes, data.origin + 5, 2)).toShort()
    }

    inline fun y_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 5)
    }

    inline fun z(): Short {
        return (get_bytes(data.bytes, data.origin + 7, 2)).toShort()
    }

    inline fun z_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 7)
    }

    inline fun r(): Short {
        return (get_bytes(data.bytes, data.origin + 9, 2)).toShort()
    }

    inline fun r_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 9)
    }

}

inline class RC_CHANNELS(val data: Pack.Bytes) {

    inline fun chan1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun chan1_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun chan2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun chan2_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun chan3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun chan3_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun chan4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun chan4_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun chan5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun chan5_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun chan6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun chan6_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun chan7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun chan7_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun chan8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun chan8_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun chan9_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }

    inline fun chan9_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 16)
    }

    inline fun chan10_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }

    inline fun chan10_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 18)
    }

    inline fun chan11_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }

    inline fun chan11_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 20)
    }

    inline fun chan12_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }

    inline fun chan12_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 22)
    }

    inline fun chan13_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 24, 2)).toShort()
    }

    inline fun chan13_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 24)
    }

    inline fun chan14_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 26, 2)).toShort()
    }

    inline fun chan14_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 26)
    }

    inline fun chan15_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 28, 2)).toShort()
    }

    inline fun chan15_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 28)
    }

    inline fun chan16_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 30, 2)).toShort()
    }

    inline fun chan16_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 30)
    }

    inline fun chan17_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 32, 2)).toShort()
    }

    inline fun chan17_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 32)
    }

    inline fun chan18_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 34, 2)).toShort()
    }

    inline fun chan18_raw_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 34)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 36, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun chancount(): Byte {
        return (data.bytes[data.origin + 40]).toByte()
    }

    inline fun chancount_(src: Byte) {
        data.bytes[data.origin + 40] = (src).toByte()
    }

    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 41]).toByte()
    }

    inline fun rssi_(src: Byte) {
        data.bytes[data.origin + 41] = (src).toByte()
    }

}

inline class PARAM_VALUE(val data: Cursor) {

    inline fun param_count(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun param_count_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun param_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun param_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun param_value(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun param_value_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }


    inline fun param_id(): PARAM_VALUE_param_id? {
        if ((data.field_bit != 66 && !data.set_field(66, -1))) return null

        return PARAM_VALUE_param_id(data)
    }


    inline fun param_id_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -66 - 1, reuse)
    }


    inline fun param_id_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(66, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_id_(len: Int): PARAM_VALUE_param_id {

        data.set_field(66, minOf(len, 255))
        return PARAM_VALUE_param_id(data)
    }

    object param_id {
        const val item_len_max = 255

    }


    inline fun param_type(): PARAM_VALUE_param_type? {
        if ((data.field_bit != 67 && !data.set_field(67, -1))) return null

        return PARAM_VALUE_param_type(data)
    }


    inline fun param_type(src: MAV_PARAM_TYPE) {
        if (data.field_bit != 67) data.set_field(67, 0)

        set_bits((-1 src . value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class SET_POSITION_TARGET_LOCAL_NED(val data: Cursor) {

    inline fun type_mask(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun type_mask_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 7] = (src).toByte()
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun vx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun vy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun vz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun afx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun afx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun afy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun afy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun afz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun afz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }

    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48, 4).toInt())
    }

    inline fun yaw_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 48)
    }


    inline fun coordinate_frame(): SET_POSITION_TARGET_LOCAL_NED_coordinate_frame? {
        if ((data.field_bit != 416 && !data.set_field(416, -1))) return null

        return SET_POSITION_TARGET_LOCAL_NED_coordinate_frame(data)
    }


    inline fun coordinate_frame(src: MAV_FRAME) {
        if (data.field_bit != 416) data.set_field(416, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class SET_GPS_GLOBAL_ORIGIN(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun latitude(): Int {
        return (get_bytes(data.bytes, data.origin + 1, 4)).toInt()
    }

    inline fun latitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 1)
    }

    inline fun longitude(): Int {
        return (get_bytes(data.bytes, data.origin + 5, 4)).toInt()
    }

    inline fun longitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 5)
    }

    inline fun altitude(): Int {
        return (get_bytes(data.bytes, data.origin + 9, 4)).toInt()
    }

    inline fun altitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 9)
    }


    inline fun time_usec(): Boolean {
        return !(data.field_bit != 104 && !data.set_field(104, -1))
    }


    inline fun time_usec_() {
        if (data.field_bit != 104) data.set_field(104, 0)
    }


}

inline class MISSION_REQUEST_LIST(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    inline fun mission_type(): MISSION_REQUEST_LIST_mission_type? {
        if ((data.field_bit != 16 && !data.set_field(16, -1))) return null

        return MISSION_REQUEST_LIST_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 16) data.set_field(16, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class MISSION_REQUEST_PARTIAL_LIST(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun start_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun start_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun end_index(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun end_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }


    inline fun mission_type(): MISSION_REQUEST_PARTIAL_LIST_mission_type? {
        if ((data.field_bit != 48 && !data.set_field(48, -1))) return null

        return MISSION_REQUEST_PARTIAL_LIST_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 48) data.set_field(48, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class LOCAL_POSITION_NED(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun vx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun vy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun vz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

}

inline class GPS_GLOBAL_ORIGIN(val data: Cursor) {

    inline fun latitude(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun latitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun longitude(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun longitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun altitude(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }

    inline fun altitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }


    inline fun time_usec(): Boolean {
        return !(data.field_bit != 96 && !data.set_field(96, -1))
    }


    inline fun time_usec_() {
        if (data.field_bit != 96) data.set_field(96, 0)
    }


}

inline class ATTITUDE_QUATERNION(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun q1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun q1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun q2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun q2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun q3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun q3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun q4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun q4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun rollspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun pitchspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun yawspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

}

inline class HIL_ACTUATOR_CONTROLS(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun flags(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun flags_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }

    inline fun controls(): HIL_ACTUATOR_CONTROLS_controls {

        return HIL_ACTUATOR_CONTROLS_controls(data)
    }

    inline fun controls_(src: FloatArray) {
        val len = minOf(src.size, 16)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 16 + index * 4)
    }

    object controls {
        const val item_len = 16


    }


    inline fun mode(): HIL_ACTUATOR_CONTROLS_mode? {
        if ((data.field_bit != 640 && !data.set_field(640, -1))) return null

        return HIL_ACTUATOR_CONTROLS_mode(data)
    }


    inline fun mode(src: MAV_MODE) {
        if (data.field_bit != 640) data.set_field(640, 0)

        set_bits(MAV_MODE.set(src).toLong(), 4, data.bytes, data.BIT)
    }


}

inline class POSITION_TARGET_LOCAL_NED(val data: Cursor) {

    inline fun type_mask(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun type_mask_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun vx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun vy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }

    inline fun vz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 26)
    }

    inline fun afx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }

    inline fun afx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 30)
    }

    inline fun afy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 34, 4).toInt())
    }

    inline fun afy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 34)
    }

    inline fun afz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 38, 4).toInt())
    }

    inline fun afz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 38)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 42, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 42)
    }

    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 46, 4).toInt())
    }

    inline fun yaw_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 46)
    }


    inline fun coordinate_frame(): POSITION_TARGET_LOCAL_NED_coordinate_frame? {
        if ((data.field_bit != 400 && !data.set_field(400, -1))) return null

        return POSITION_TARGET_LOCAL_NED_coordinate_frame(data)
    }


    inline fun coordinate_frame(src: MAV_FRAME) {
        if (data.field_bit != 400) data.set_field(400, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


}

inline class CHANGE_OPERATOR_CONTROL(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun control_request(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun control_request_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun version(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun version_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }


    inline fun passkey(): CHANGE_OPERATOR_CONTROL_passkey? {
        if ((data.field_bit != 26 && !data.set_field(26, -1))) return null

        return CHANGE_OPERATOR_CONTROL_passkey(data)
    }


    inline fun passkey_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -26 - 1, reuse)
    }


    inline fun passkey_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(26, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun passkey_(len: Int): CHANGE_OPERATOR_CONTROL_passkey {

        data.set_field(26, minOf(len, 255))
        return CHANGE_OPERATOR_CONTROL_passkey(data)
    }

    object passkey {
        const val item_len_max = 255

    }

}

inline class SYS_STATUS(val data: Cursor) {

    inline fun load(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun load_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun voltage_battery(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun voltage_battery_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun drop_rate_comm(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun drop_rate_comm_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun errors_comm(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun errors_comm_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun errors_count1(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun errors_count1_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun errors_count2(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun errors_count2_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun errors_count3(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun errors_count3_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun errors_count4(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun errors_count4_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun current_battery(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }

    inline fun current_battery_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 16)
    }

    inline fun battery_remaining(): Byte {
        return (data.bytes[data.origin + 18]).toByte()
    }

    inline fun battery_remaining_(src: Byte) {
        data.bytes[data.origin + 18] = (src).toByte()
    }


    inline fun onboard_control_sensors_present(): SYS_STATUS_onboard_control_sensors_present? {
        if ((data.field_bit != 154 && !data.set_field(154, -1))) return null

        return SYS_STATUS_onboard_control_sensors_present(data)
    }


    inline fun onboard_control_sensors_present(src: MAV_SYS_STATUS_SENSOR) {
        if (data.field_bit != 154) data.set_field(154, 0)

        set_bits(MAV_SYS_STATUS_SENSOR.set(src).toLong(), 5, data.bytes, data.BIT)
    }


    inline fun onboard_control_sensors_enabled(): SYS_STATUS_onboard_control_sensors_enabled? {
        if ((data.field_bit != 155 && !data.set_field(155, -1))) return null

        return SYS_STATUS_onboard_control_sensors_enabled(data)
    }


    inline fun onboard_control_sensors_enabled(src: MAV_SYS_STATUS_SENSOR) {
        if (data.field_bit != 155) data.set_field(155, 0)

        set_bits(MAV_SYS_STATUS_SENSOR.set(src).toLong(), 5, data.bytes, data.BIT)
    }


    inline fun onboard_control_sensors_health(): SYS_STATUS_onboard_control_sensors_health? {
        if ((data.field_bit != 156 && !data.set_field(156, -1))) return null

        return SYS_STATUS_onboard_control_sensors_health(data)
    }


    inline fun onboard_control_sensors_health(src: MAV_SYS_STATUS_SENSOR) {
        if (data.field_bit != 156) data.set_field(156, 0)

        set_bits(MAV_SYS_STATUS_SENSOR.set(src).toLong(), 5, data.bytes, data.BIT)
    }


}

inline class MISSION_ITEM(val data: Cursor) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun seq_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun current(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun current_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun autocontinue(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun autocontinue_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun param1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun param1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun param2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }

    inline fun param2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun param3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun param3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun param4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun param4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 26)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 30)
    }


    inline fun frame(): MISSION_ITEM_frame? {
        if ((data.field_bit != 274 && !data.set_field(274, -1))) return null

        return MISSION_ITEM_frame(data)
    }


    inline fun frame(src: MAV_FRAME) {
        if (data.field_bit != 274) data.set_field(274, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    inline fun command(): MISSION_ITEM_command? {
        if ((data.field_bit != 275 && !data.set_field(275, -1))) return null

        return MISSION_ITEM_command(data)
    }


    inline fun command(src: MAV_CMD) {
        if (data.field_bit != 275) data.set_field(275, 0)

        set_bits(MAV_CMD.set(src).toLong(), 8, data.bytes, data.BIT)
    }


    inline fun mission_type(): MISSION_ITEM_mission_type? {
        if ((data.field_bit != 276 && !data.set_field(276, -1))) return null

        return MISSION_ITEM_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 276) data.set_field(276, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class RAW_IMU(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun xacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun yacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun zacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun xgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun xgyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun ygyro(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }

    inline fun ygyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 16)
    }

    inline fun zgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }

    inline fun zgyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 18)
    }

    inline fun xmag(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }

    inline fun xmag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 20)
    }

    inline fun ymag(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }

    inline fun ymag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 22)
    }

    inline fun zmag(): Short {
        return (get_bytes(data.bytes, data.origin + 24, 2)).toShort()
    }

    inline fun zmag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 24)
    }

}

inline class COMMAND_INT(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun current(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun current_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun autocontinue(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun autocontinue_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun param1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun param1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun param2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun param2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun param3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun param3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun param4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun param4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun x(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }

    inline fun x_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun y(): Int {
        return (get_bytes(data.bytes, data.origin + 24, 4)).toInt()
    }

    inline fun y_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }


    inline fun frame(): COMMAND_INT_frame? {
        if ((data.field_bit != 258 && !data.set_field(258, -1))) return null

        return COMMAND_INT_frame(data)
    }


    inline fun frame(src: MAV_FRAME) {
        if (data.field_bit != 258) data.set_field(258, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    inline fun command(): COMMAND_INT_command? {
        if ((data.field_bit != 259 && !data.set_field(259, -1))) return null

        return COMMAND_INT_command(data)
    }


    inline fun command(src: MAV_CMD) {
        if (data.field_bit != 259) data.set_field(259, 0)

        set_bits(MAV_CMD.set(src).toLong(), 8, data.bytes, data.BIT)
    }


}

inline class OPTICAL_FLOW(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun sensor_id(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun sensor_id_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun flow_x(): Short {
        return (get_bytes(data.bytes, data.origin + 9, 2)).toShort()
    }

    inline fun flow_x_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 9)
    }

    inline fun flow_y(): Short {
        return (get_bytes(data.bytes, data.origin + 11, 2)).toShort()
    }

    inline fun flow_y_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 11)
    }

    inline fun flow_comp_m_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 13, 4).toInt())
    }

    inline fun flow_comp_m_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 13)
    }

    inline fun flow_comp_m_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 17, 4).toInt())
    }

    inline fun flow_comp_m_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }

    inline fun quality(): Byte {
        return (data.bytes[data.origin + 21]).toByte()
    }

    inline fun quality_(src: Byte) {
        data.bytes[data.origin + 21] = (src).toByte()
    }

    inline fun ground_distance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun ground_distance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }


    inline fun flow_rate_x(): Boolean {
        return !(data.field_bit != 208 && !data.set_field(208, -1))
    }


    inline fun flow_rate_x_() {
        if (data.field_bit != 208) data.set_field(208, 0)
    }


    inline fun flow_rate_y(): Boolean {
        return !(data.field_bit != 209 && !data.set_field(209, -1))
    }


    inline fun flow_rate_y_() {
        if (data.field_bit != 209) data.set_field(209, 0)
    }


}

inline class MISSION_ITEM_INT(val data: Cursor) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun seq_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun current(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun current_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun autocontinue(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun autocontinue_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun param1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun param1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun param2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }

    inline fun param2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun param3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun param3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun param4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun param4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun x(): Int {
        return (get_bytes(data.bytes, data.origin + 22, 4)).toInt()
    }

    inline fun x_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

    inline fun y(): Int {
        return (get_bytes(data.bytes, data.origin + 26, 4)).toInt()
    }

    inline fun y_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 26)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 30)
    }


    inline fun frame(): MISSION_ITEM_INT_frame? {
        if ((data.field_bit != 274 && !data.set_field(274, -1))) return null

        return MISSION_ITEM_INT_frame(data)
    }


    inline fun frame(src: MAV_FRAME) {
        if (data.field_bit != 274) data.set_field(274, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    inline fun command(): MISSION_ITEM_INT_command? {
        if ((data.field_bit != 275 && !data.set_field(275, -1))) return null

        return MISSION_ITEM_INT_command(data)
    }


    inline fun command(src: MAV_CMD) {
        if (data.field_bit != 275) data.set_field(275, 0)

        set_bits(MAV_CMD.set(src).toLong(), 8, data.bytes, data.BIT)
    }


    inline fun mission_type(): MISSION_ITEM_INT_mission_type? {
        if ((data.field_bit != 276 && !data.set_field(276, -1))) return null

        return MISSION_ITEM_INT_mission_type(data)
    }


    inline fun mission_type(src: MAV_MISSION_TYPE) {
        if (data.field_bit != 276) data.set_field(276, 0)

        set_bits(MAV_MISSION_TYPE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


}

inline class ATTITUDE_QUATERNION_COV(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun q(): ATTITUDE_QUATERNION_COV_q {

        return ATTITUDE_QUATERNION_COV_q(data)
    }

    inline fun q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 8 + index * 4)
    }

    object q {
        const val item_len = 4


    }

    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun rollspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun pitchspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun yawspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun covariance(): ATTITUDE_QUATERNION_COV_covariance {

        return ATTITUDE_QUATERNION_COV_covariance(data)
    }

    inline fun covariance_(src: FloatArray) {
        val len = minOf(src.size, 9)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 36 + index * 4)
    }

    object covariance {
        const val item_len = 9


    }

}

inline class GLOBAL_VISION_POSITION_ESTIMATE(val data: Pack.Bytes) {

    inline fun usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

}


object TestChannel : AdHoc.Channel() {
    var strings = arrayOf("vztTfrNVkqaynqQCudjvwzcsksexriwsQphigpmzVnacohyevcolxEztsFvmmimowKburgeqxmnytghhfkhtiCbgbttjmfismetatgMxffbLNTkhnqjLsaaodtcoHyyetvwtlmeuytkojirlceWksskXyVqxegstrQgyqnrSpzwwbcxnprtg",
            "bdmuinynoVntwkpvglvjhvdmleazlcjzMramtqpeajuyexpzltbacvhjivxjvmpdugaeddYJmbcsjnQbnexzezcqemscbcfxgwljohytyrjnpWYvjdkjgEcdfjiqgtjxjlbhqmmqhaijyLgSAzfdxhQmituqypbHzfGrBmyqleiozBknWelJRtfcwiqaaKqgaqfaewhsljurydmUtcamjgtUnLhuwqdNimszeuvFwhkiQws",
            "quBuauwFpwjMqMaecrjryrqvCJbgrnfywkaNwQcvnduvfombjebtihnllbUykpmwktoonmbtkkkpzlBmSmofamsdunfusogwykanyhykmkzmlriruxmUiCenkxamwazEjbaQobkezhhfvlorjAhbhkwdyajkzfuvpozLceqzfgweWLURuwlannUqkyqapioOlzeqmwaibjagukbhzhibonxkmraichdu",
            "vqvQyTrojxtripyehZmcolvuqroSurropbayxdhdqaadcmObiprjcwriypqomwnatOezGoUntuvccXovIsnweoznmpbbutaEezNoyizLnujitxzAtcdpfAcvqoqEDzfxTDbnxVkeqjcypubvpsumZgsjdpzcjdurxZwyantdbujUluJrnumksbQrfftnlbyqniisrrjcntyoeggppmwhzxfkdiwfsydf",
            "qqwiuhykHQxorsvkiuTcIjphfrbdberItZvxkgzbafdseoSbYlLvuwkmZuqlhurmfVucDjjvdAzfjSjzajwmuaTpdbdHlrglihjwmbmVCpetrymjqWgdctymilhapOemkuUiwdpinhbplnzqjzfbjzwl")

    fun str(index: Int, len: Int): String {
        var ret = strings[index % strings.size]
        return if (len < ret.length) ret.substring(0, len) else ret
    }

    var sendingPack: AdHoc.Pack? = null
    val transmitter: java.io.InputStream = object : Transmitter(1, 1) {
        override fun pullSendingPack(): Pack? {
            val ret = sendingPack
            sendingPack = null
            return ret
        }
    }
    val transmitterAdv: java.io.InputStream = object : Transmitter.Advanced(1, 1) {
        override fun pullSendingPack(): Pack? {
            val ret = sendingPack
            sendingPack = null
            return ret
        }
    }


    val receiver: java.io.OutputStream = object : Receiver(1, 1) {
        override fun dispatch(id: Int, pack: Pack?): Pack.Meta? {
            return test_dispatch(id, pack)
        }
    }
    val receiverAdv: java.io.OutputStream = object : Receiver.Advanced(1, 1) {
        override fun dispatch(id: Int, pack: Pack?): Pack.Meta? {
            return test_dispatch(id, pack)
        }
    }

    fun test_dispatch(id: Int, pack: Pack?): Pack.Meta? {
        when (id) {
            96 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pekf_status_report = com.company.demo.GroundControl.EKF_STATUS_REPORT(cur)
                    onEKF_STATUS_REPORT(pekf_status_report)
                } ?: return com.company.demo.GroundControl.EKF_STATUS_REPORT.meta

            155 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val phwstatus = com.company.demo.GroundControl.HWSTATUS(bytes)
                    onHWSTATUS(phwstatus)
                } ?: return com.company.demo.GroundControl.HWSTATUS.meta
            80 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pparam_ext_request_list = com.company.demo.GroundControl.PARAM_EXT_REQUEST_LIST(bytes)
                    onPARAM_EXT_REQUEST_LIST(pparam_ext_request_list)
                } ?: return com.company.demo.GroundControl.PARAM_EXT_REQUEST_LIST.meta
            68 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val puavcan_node_status = com.company.demo.GroundControl.UAVCAN_NODE_STATUS(cur)
                    onUAVCAN_NODE_STATUS(puavcan_node_status)
                } ?: return com.company.demo.GroundControl.UAVCAN_NODE_STATUS.meta

            117 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pgimbal_torque_cmd_report = com.company.demo.GroundControl.GIMBAL_TORQUE_CMD_REPORT(bytes)
                    onGIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report)
                } ?: return com.company.demo.GroundControl.GIMBAL_TORQUE_CMD_REPORT.meta
            128 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val psensor_offsets = com.company.demo.GroundControl.SENSOR_OFFSETS(bytes)
                    onSENSOR_OFFSETS(psensor_offsets)
                } ?: return com.company.demo.GroundControl.SENSOR_OFFSETS.meta
            33 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pdevice_op_write_reply = com.company.demo.GroundControl.DEVICE_OP_WRITE_REPLY(bytes)
                    onDEVICE_OP_WRITE_REPLY(pdevice_op_write_reply)
                } ?: return com.company.demo.GroundControl.DEVICE_OP_WRITE_REPLY.meta
            104 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pgimbal_control = com.company.demo.GroundControl.GIMBAL_CONTROL(bytes)
                    onGIMBAL_CONTROL(pgimbal_control)
                } ?: return com.company.demo.GroundControl.GIMBAL_CONTROL.meta
            1 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pvideo_stream_information = com.company.demo.GroundControl.VIDEO_STREAM_INFORMATION(cur)
                    onVIDEO_STREAM_INFORMATION(pvideo_stream_information)
                } ?: return com.company.demo.GroundControl.VIDEO_STREAM_INFORMATION.meta

            214 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pahrs = com.company.demo.GroundControl.AHRS(bytes)
                    onAHRS(pahrs)
                } ?: return com.company.demo.GroundControl.AHRS.meta
            93 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pfence_status = com.company.demo.GroundControl.FENCE_STATUS(cur)
                    onFENCE_STATUS(pfence_status)
                } ?: return com.company.demo.GroundControl.FENCE_STATUS.meta

            171 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val premote_log_block_status = com.company.demo.GroundControl.REMOTE_LOG_BLOCK_STATUS(cur)
                    onREMOTE_LOG_BLOCK_STATUS(premote_log_block_status)
                } ?: return com.company.demo.GroundControl.REMOTE_LOG_BLOCK_STATUS.meta

            66 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pobstacle_distance = com.company.demo.GroundControl.OBSTACLE_DISTANCE(cur)
                    onOBSTACLE_DISTANCE(pobstacle_distance)
                } ?: return com.company.demo.GroundControl.OBSTACLE_DISTANCE.meta

            90 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pparam_ext_request_read = com.company.demo.GroundControl.PARAM_EXT_REQUEST_READ(cur)
                    onPARAM_EXT_REQUEST_READ(pparam_ext_request_read)
                } ?: return com.company.demo.GroundControl.PARAM_EXT_REQUEST_READ.meta

            84 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val puavionix_adsb_out_cfg = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG(cur)
                    onUAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg)
                } ?: return com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG.meta

            164 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pdata32 = com.company.demo.GroundControl.DATA32(bytes)
                    onDATA32(pdata32)
                } ?: return com.company.demo.GroundControl.DATA32.meta
            197 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val prally_point = com.company.demo.GroundControl.RALLY_POINT(cur)
                    onRALLY_POINT(prally_point)
                } ?: return com.company.demo.GroundControl.RALLY_POINT.meta

            115 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val padap_tuning = com.company.demo.GroundControl.ADAP_TUNING(cur)
                    onADAP_TUNING(padap_tuning)
                } ?: return com.company.demo.GroundControl.ADAP_TUNING.meta

            14 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pparam_ext_value = com.company.demo.GroundControl.PARAM_EXT_VALUE(cur)
                    onPARAM_EXT_VALUE(pparam_ext_value)
                } ?: return com.company.demo.GroundControl.PARAM_EXT_VALUE.meta

            190 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pbattery2 = com.company.demo.GroundControl.BATTERY2(bytes)
                    onBATTERY2(pbattery2)
                } ?: return com.company.demo.GroundControl.BATTERY2.meta
            186 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val plimits_status = com.company.demo.GroundControl.LIMITS_STATUS(cur)
                    onLIMITS_STATUS(plimits_status)
                } ?: return com.company.demo.GroundControl.LIMITS_STATUS.meta

            0 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pcamera_feedback = com.company.demo.GroundControl.CAMERA_FEEDBACK(cur)
                    onCAMERA_FEEDBACK(pcamera_feedback)
                } ?: return com.company.demo.GroundControl.CAMERA_FEEDBACK.meta

            119 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pfence_fetch_point = com.company.demo.GroundControl.FENCE_FETCH_POINT(bytes)
                    onFENCE_FETCH_POINT(pfence_fetch_point)
                } ?: return com.company.demo.GroundControl.FENCE_FETCH_POINT.meta
            32 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pradio = com.company.demo.GroundControl.RADIO(bytes)
                    onRADIO(pradio)
                } ?: return com.company.demo.GroundControl.RADIO.meta
            69 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pairspeed_autocal = com.company.demo.GroundControl.AIRSPEED_AUTOCAL(bytes)
                    onAIRSPEED_AUTOCAL(pairspeed_autocal)
                } ?: return com.company.demo.GroundControl.AIRSPEED_AUTOCAL.meta
            138 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pgopro_get_request = com.company.demo.GroundControl.GOPRO_GET_REQUEST(cur)
                    onGOPRO_GET_REQUEST(pgopro_get_request)
                } ?: return com.company.demo.GroundControl.GOPRO_GET_REQUEST.meta

            43 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pcompassmot_status = com.company.demo.GroundControl.COMPASSMOT_STATUS(bytes)
                    onCOMPASSMOT_STATUS(pcompassmot_status)
                } ?: return com.company.demo.GroundControl.COMPASSMOT_STATUS.meta
            211 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pcamera_status = com.company.demo.GroundControl.CAMERA_STATUS(cur)
                    onCAMERA_STATUS(pcamera_status)
                } ?: return com.company.demo.GroundControl.CAMERA_STATUS.meta

            165 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pdevice_op_read_reply = com.company.demo.GroundControl.DEVICE_OP_READ_REPLY(bytes)
                    onDEVICE_OP_READ_REPLY(pdevice_op_read_reply)
                } ?: return com.company.demo.GroundControl.DEVICE_OP_READ_REPLY.meta
            49 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pdigicam_control = com.company.demo.GroundControl.DIGICAM_CONTROL(bytes)
                    onDIGICAM_CONTROL(pdigicam_control)
                } ?: return com.company.demo.GroundControl.DIGICAM_CONTROL.meta
            175 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pgopro_heartbeat = com.company.demo.GroundControl.GOPRO_HEARTBEAT(cur)
                    onGOPRO_HEARTBEAT(pgopro_heartbeat)
                } ?: return com.company.demo.GroundControl.GOPRO_HEARTBEAT.meta

            17 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pahrs2 = com.company.demo.GroundControl.AHRS2(bytes)
                    onAHRS2(pahrs2)
                } ?: return com.company.demo.GroundControl.AHRS2.meta
            44 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pmount_status = com.company.demo.GroundControl.MOUNT_STATUS(bytes)
                    onMOUNT_STATUS(pmount_status)
                } ?: return com.company.demo.GroundControl.MOUNT_STATUS.meta
            64 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val ppid_tuning = com.company.demo.GroundControl.PID_TUNING(cur)
                    onPID_TUNING(ppid_tuning)
                } ?: return com.company.demo.GroundControl.PID_TUNING.meta

            65 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pahrs3 = com.company.demo.GroundControl.AHRS3(bytes)
                    onAHRS3(pahrs3)
                } ?: return com.company.demo.GroundControl.AHRS3.meta
            63 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pmag_cal_report = com.company.demo.GroundControl.MAG_CAL_REPORT(cur)
                    onMAG_CAL_REPORT(pmag_cal_report)
                } ?: return com.company.demo.GroundControl.MAG_CAL_REPORT.meta

            144 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val premote_log_data_block = com.company.demo.GroundControl.REMOTE_LOG_DATA_BLOCK(cur)
                    onREMOTE_LOG_DATA_BLOCK(premote_log_data_block)
                } ?: return com.company.demo.GroundControl.REMOTE_LOG_DATA_BLOCK.meta

            217 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val plogging_data_acked = com.company.demo.GroundControl.LOGGING_DATA_ACKED(bytes)
                    onLOGGING_DATA_ACKED(plogging_data_acked)
                } ?: return com.company.demo.GroundControl.LOGGING_DATA_ACKED.meta
            40 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pmount_configure = com.company.demo.GroundControl.MOUNT_CONFIGURE(cur)
                    onMOUNT_CONFIGURE(pmount_configure)
                } ?: return com.company.demo.GroundControl.MOUNT_CONFIGURE.meta

            113 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pmount_control = com.company.demo.GroundControl.MOUNT_CONTROL(bytes)
                    onMOUNT_CONTROL(pmount_control)
                } ?: return com.company.demo.GroundControl.MOUNT_CONTROL.meta
            177 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pled_control = com.company.demo.GroundControl.LED_CONTROL(bytes)
                    onLED_CONTROL(pled_control)
                } ?: return com.company.demo.GroundControl.LED_CONTROL.meta
            194 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pwifi_config_ap = com.company.demo.GroundControl.WIFI_CONFIG_AP(cur)
                    onWIFI_CONFIG_AP(pwifi_config_ap)
                } ?: return com.company.demo.GroundControl.WIFI_CONFIG_AP.meta

            83 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pdata96 = com.company.demo.GroundControl.DATA96(bytes)
                    onDATA96(pdata96)
                } ?: return com.company.demo.GroundControl.DATA96.meta
            159 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pmeminfo = com.company.demo.GroundControl.MEMINFO(cur)
                    onMEMINFO(pmeminfo)
                } ?: return com.company.demo.GroundControl.MEMINFO.meta

            220 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val plogging_ack = com.company.demo.GroundControl.LOGGING_ACK(bytes)
                    onLOGGING_ACK(plogging_ack)
                } ?: return com.company.demo.GroundControl.LOGGING_ACK.meta
            192 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pgopro_set_response = com.company.demo.GroundControl.GOPRO_SET_RESPONSE(cur)
                    onGOPRO_SET_RESPONSE(pgopro_set_response)
                } ?: return com.company.demo.GroundControl.GOPRO_SET_RESPONSE.meta

            150 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pprotocol_version = com.company.demo.GroundControl.PROTOCOL_VERSION(bytes)
                    onPROTOCOL_VERSION(pprotocol_version)
                } ?: return com.company.demo.GroundControl.PROTOCOL_VERSION.meta
            5 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val prally_fetch_point = com.company.demo.GroundControl.RALLY_FETCH_POINT(bytes)
                    onRALLY_FETCH_POINT(prally_fetch_point)
                } ?: return com.company.demo.GroundControl.RALLY_FETCH_POINT.meta
            76 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pmount_orientation = com.company.demo.GroundControl.MOUNT_ORIENTATION(bytes)
                    onMOUNT_ORIENTATION(pmount_orientation)
                } ?: return com.company.demo.GroundControl.MOUNT_ORIENTATION.meta
            109 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pparam_ext_set = com.company.demo.GroundControl.PARAM_EXT_SET(cur)
                    onPARAM_EXT_SET(pparam_ext_set)
                } ?: return com.company.demo.GroundControl.PARAM_EXT_SET.meta

            112 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val psimstate = com.company.demo.GroundControl.SIMSTATE(bytes)
                    onSIMSTATE(psimstate)
                } ?: return com.company.demo.GroundControl.SIMSTATE.meta
            22 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pset_video_stream_settings = com.company.demo.GroundControl.SET_VIDEO_STREAM_SETTINGS(cur)
                    onSET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings)
                } ?: return com.company.demo.GroundControl.SET_VIDEO_STREAM_SETTINGS.meta

            114 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pdigicam_configure = com.company.demo.GroundControl.DIGICAM_CONFIGURE(bytes)
                    onDIGICAM_CONFIGURE(pdigicam_configure)
                } ?: return com.company.demo.GroundControl.DIGICAM_CONFIGURE.meta
            51 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pparam_ext_ack = com.company.demo.GroundControl.PARAM_EXT_ACK(cur)
                    onPARAM_EXT_ACK(pparam_ext_ack)
                } ?: return com.company.demo.GroundControl.PARAM_EXT_ACK.meta

            127 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val puavcan_node_info = com.company.demo.GroundControl.UAVCAN_NODE_INFO(cur)
                    onUAVCAN_NODE_INFO(puavcan_node_info)
                } ?: return com.company.demo.GroundControl.UAVCAN_NODE_INFO.meta

            36 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pdata16 = com.company.demo.GroundControl.DATA16(bytes)
                    onDATA16(pdata16)
                } ?: return com.company.demo.GroundControl.DATA16.meta
            100 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pset_mag_offsets = com.company.demo.GroundControl.SET_MAG_OFFSETS(bytes)
                    onSET_MAG_OFFSETS(pset_mag_offsets)
                } ?: return com.company.demo.GroundControl.SET_MAG_OFFSETS.meta
            198 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pap_adc = com.company.demo.GroundControl.AP_ADC(bytes)
                    onAP_ADC(pap_adc)
                } ?: return com.company.demo.GroundControl.AP_ADC.meta
            97 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pwind = com.company.demo.GroundControl.WIND(bytes)
                    onWIND(pwind)
                } ?: return com.company.demo.GroundControl.WIND.meta
            142 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pautopilot_version_request = com.company.demo.GroundControl.AUTOPILOT_VERSION_REQUEST(bytes)
                    onAUTOPILOT_VERSION_REQUEST(pautopilot_version_request)
                } ?: return com.company.demo.GroundControl.AUTOPILOT_VERSION_REQUEST.meta
            141 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pdata64 = com.company.demo.GroundControl.DATA64(bytes)
                    onDATA64(pdata64)
                } ?: return com.company.demo.GroundControl.DATA64.meta
            11 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pgimbal_report = com.company.demo.GroundControl.GIMBAL_REPORT(bytes)
                    onGIMBAL_REPORT(pgimbal_report)
                } ?: return com.company.demo.GroundControl.GIMBAL_REPORT.meta
            105 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pdevice_op_write = com.company.demo.GroundControl.DEVICE_OP_WRITE(cur)
                    onDEVICE_OP_WRITE(pdevice_op_write)
                } ?: return com.company.demo.GroundControl.DEVICE_OP_WRITE.meta

            118 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pgopro_set_request = com.company.demo.GroundControl.GOPRO_SET_REQUEST(cur)
                    onGOPRO_SET_REQUEST(pgopro_set_request)
                } ?: return com.company.demo.GroundControl.GOPRO_SET_REQUEST.meta

            12 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pvision_position_delta = com.company.demo.GroundControl.VISION_POSITION_DELTA(bytes)
                    onVISION_POSITION_DELTA(pvision_position_delta)
                } ?: return com.company.demo.GroundControl.VISION_POSITION_DELTA.meta
            75 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val plogging_data = com.company.demo.GroundControl.LOGGING_DATA(bytes)
                    onLOGGING_DATA(plogging_data)
                } ?: return com.company.demo.GroundControl.LOGGING_DATA.meta
            70 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pdevice_op_read = com.company.demo.GroundControl.DEVICE_OP_READ(cur)
                    onDEVICE_OP_READ(pdevice_op_read)
                } ?: return com.company.demo.GroundControl.DEVICE_OP_READ.meta

            57 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pmag_cal_progress = com.company.demo.GroundControl.MAG_CAL_PROGRESS(cur)
                    onMAG_CAL_PROGRESS(pmag_cal_progress)
                } ?: return com.company.demo.GroundControl.MAG_CAL_PROGRESS.meta

            143 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val puavionix_adsb_out_dynamic = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC(cur)
                    onUAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic)
                } ?: return com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC.meta

            208 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val pgopro_get_response = com.company.demo.GroundControl.GOPRO_GET_RESPONSE(cur)
                    onGOPRO_GET_RESPONSE(pgopro_get_response)
                } ?: return com.company.demo.GroundControl.GOPRO_GET_RESPONSE.meta

            26 ->
                pack?.let {
                    val cur = Cursor()
                    cur.wrap(it)
                    val puavionix_adsb_transceiver_health_report = com.company.demo.GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur)
                    onUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report)
                } ?: return com.company.demo.GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.meta

            183 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val prpm = com.company.demo.GroundControl.RPM(bytes)
                    onRPM(prpm)
                } ?: return com.company.demo.GroundControl.RPM.meta
            153 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val prangefinder = com.company.demo.GroundControl.RANGEFINDER(bytes)
                    onRANGEFINDER(prangefinder)
                } ?: return com.company.demo.GroundControl.RANGEFINDER.meta
            20 ->
                pack?.let {
                    val bytes = AdHoc.Pack.Bytes()
                    bytes.wrap(pack)
                    val pfence_point = com.company.demo.GroundControl.FENCE_POINT(bytes)
                    onFENCE_POINT(pfence_point)
                } ?: return com.company.demo.GroundControl.FENCE_POINT.meta

        }
        return null
    }

    fun send(src: com.company.demo.GroundControl.ATTITUDE_TARGET): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_COUNT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.GLOBAL_POSITION_INT_COV): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SAFETY_SET_ALLOWED_AREA): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.GPS_STATUS): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.PARAM_SET): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.RC_CHANNELS_OVERRIDE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SCALED_IMU): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.PARAM_REQUEST_READ): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SET_ATTITUDE_TARGET): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.HIL_STATE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.REQUEST_DATA_STREAM): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.HIL_CONTROLS): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.PARAM_REQUEST_LIST): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SET_POSITION_TARGET_GLOBAL_INT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.VFR_HUD): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_SET_CURRENT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.NAV_CONTROLLER_OUTPUT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.AUTH_KEY): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.LOCAL_POSITION_NED_COV): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.PING): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.GLOBAL_POSITION_INT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.COMMAND_LONG): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.GPS_RAW_INT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.RC_CHANNELS_SCALED): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.RAW_PRESSURE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.ATTITUDE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_WRITE_PARTIAL_LIST): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MANUAL_SETPOINT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SAFETY_ALLOWED_AREA): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_CLEAR_ALL): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SCALED_PRESSURE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.HEARTBEAT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.PARAM_MAP_RC): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_REQUEST_INT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.COMMAND_ACK): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.DATA_STREAM): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_REQUEST): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.HIL_RC_INPUTS_RAW): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SET_MODE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.POSITION_TARGET_GLOBAL_INT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.RC_CHANNELS_RAW): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SERVO_OUTPUT_RAW): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_ITEM_REACHED): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_ACK): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL_ACK): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_CURRENT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SYSTEM_TIME): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.VISION_POSITION_ESTIMATE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MANUAL_CONTROL): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.RC_CHANNELS): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.PARAM_VALUE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SET_POSITION_TARGET_LOCAL_NED): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SET_GPS_GLOBAL_ORIGIN): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_REQUEST_LIST): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_REQUEST_PARTIAL_LIST): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.LOCAL_POSITION_NED): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.GPS_GLOBAL_ORIGIN): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.ATTITUDE_QUATERNION): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.HIL_ACTUATOR_CONTROLS): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.POSITION_TARGET_LOCAL_NED): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.SYS_STATUS): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_ITEM): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.RAW_IMU): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.COMMAND_INT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.OPTICAL_FLOW): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.MISSION_ITEM_INT): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.ATTITUDE_QUATERNION_COV): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }

    fun send(src: com.company.demo.GroundControl.GLOBAL_VISION_POSITION_ESTIMATE): Boolean {
        if (sendingPack != null) return false
        sendingPack = src.data.unwrap()
        return true
    }


    object NEW {
        fun ATTITUDE_TARGET(cur: Pack.Bytes): test_.ATTITUDE_TARGET {
            cur.init(com.company.demo.GroundControl.ATTITUDE_TARGET.meta)
            return test_.ATTITUDE_TARGET(cur)
        }

        fun MISSION_COUNT(cur: Cursor): test_.MISSION_COUNT {
            cur.init(com.company.demo.GroundControl.MISSION_COUNT.meta)
            return test_.MISSION_COUNT(cur)
        }

        fun GLOBAL_POSITION_INT_COV(cur: Cursor): test_.GLOBAL_POSITION_INT_COV {
            cur.init(com.company.demo.GroundControl.GLOBAL_POSITION_INT_COV.meta)
            return test_.GLOBAL_POSITION_INT_COV(cur)
        }

        fun SAFETY_SET_ALLOWED_AREA(cur: Cursor): test_.SAFETY_SET_ALLOWED_AREA {
            cur.init(com.company.demo.GroundControl.SAFETY_SET_ALLOWED_AREA.meta)
            return test_.SAFETY_SET_ALLOWED_AREA(cur)
        }

        fun GPS_STATUS(cur: Pack.Bytes): test_.GPS_STATUS {
            cur.init(com.company.demo.GroundControl.GPS_STATUS.meta)
            return test_.GPS_STATUS(cur)
        }

        fun PARAM_SET(cur: Cursor): test_.PARAM_SET {
            cur.init(com.company.demo.GroundControl.PARAM_SET.meta)
            return test_.PARAM_SET(cur)
        }

        fun RC_CHANNELS_OVERRIDE(cur: Pack.Bytes): test_.RC_CHANNELS_OVERRIDE {
            cur.init(com.company.demo.GroundControl.RC_CHANNELS_OVERRIDE.meta)
            return test_.RC_CHANNELS_OVERRIDE(cur)
        }

        fun SCALED_IMU(cur: Pack.Bytes): test_.SCALED_IMU {
            cur.init(com.company.demo.GroundControl.SCALED_IMU.meta)
            return test_.SCALED_IMU(cur)
        }

        fun PARAM_REQUEST_READ(cur: Cursor): test_.PARAM_REQUEST_READ {
            cur.init(com.company.demo.GroundControl.PARAM_REQUEST_READ.meta)
            return test_.PARAM_REQUEST_READ(cur)
        }

        fun SET_ATTITUDE_TARGET(cur: Pack.Bytes): test_.SET_ATTITUDE_TARGET {
            cur.init(com.company.demo.GroundControl.SET_ATTITUDE_TARGET.meta)
            return test_.SET_ATTITUDE_TARGET(cur)
        }

        fun HIL_STATE(cur: Pack.Bytes): test_.HIL_STATE {
            cur.init(com.company.demo.GroundControl.HIL_STATE.meta)
            return test_.HIL_STATE(cur)
        }

        fun REQUEST_DATA_STREAM(cur: Pack.Bytes): test_.REQUEST_DATA_STREAM {
            cur.init(com.company.demo.GroundControl.REQUEST_DATA_STREAM.meta)
            return test_.REQUEST_DATA_STREAM(cur)
        }

        fun HIL_CONTROLS(cur: Cursor): test_.HIL_CONTROLS {
            cur.init(com.company.demo.GroundControl.HIL_CONTROLS.meta)
            return test_.HIL_CONTROLS(cur)
        }

        fun PARAM_REQUEST_LIST(cur: Pack.Bytes): test_.PARAM_REQUEST_LIST {
            cur.init(com.company.demo.GroundControl.PARAM_REQUEST_LIST.meta)
            return test_.PARAM_REQUEST_LIST(cur)
        }

        fun SET_POSITION_TARGET_GLOBAL_INT(cur: Cursor): test_.SET_POSITION_TARGET_GLOBAL_INT {
            cur.init(com.company.demo.GroundControl.SET_POSITION_TARGET_GLOBAL_INT.meta)
            return test_.SET_POSITION_TARGET_GLOBAL_INT(cur)
        }

        fun VFR_HUD(cur: Pack.Bytes): test_.VFR_HUD {
            cur.init(com.company.demo.GroundControl.VFR_HUD.meta)
            return test_.VFR_HUD(cur)
        }

        fun MISSION_SET_CURRENT(cur: Pack.Bytes): test_.MISSION_SET_CURRENT {
            cur.init(com.company.demo.GroundControl.MISSION_SET_CURRENT.meta)
            return test_.MISSION_SET_CURRENT(cur)
        }

        fun NAV_CONTROLLER_OUTPUT(cur: Pack.Bytes): test_.NAV_CONTROLLER_OUTPUT {
            cur.init(com.company.demo.GroundControl.NAV_CONTROLLER_OUTPUT.meta)
            return test_.NAV_CONTROLLER_OUTPUT(cur)
        }

        fun AUTH_KEY(cur: Cursor): test_.AUTH_KEY {
            cur.init(com.company.demo.GroundControl.AUTH_KEY.meta)
            return test_.AUTH_KEY(cur)
        }

        fun LOCAL_POSITION_NED_COV(cur: Cursor): test_.LOCAL_POSITION_NED_COV {
            cur.init(com.company.demo.GroundControl.LOCAL_POSITION_NED_COV.meta)
            return test_.LOCAL_POSITION_NED_COV(cur)
        }

        fun PING(cur: Pack.Bytes): test_.PING {
            cur.init(com.company.demo.GroundControl.PING.meta)
            return test_.PING(cur)
        }

        fun GLOBAL_POSITION_INT(cur: Pack.Bytes): test_.GLOBAL_POSITION_INT {
            cur.init(com.company.demo.GroundControl.GLOBAL_POSITION_INT.meta)
            return test_.GLOBAL_POSITION_INT(cur)
        }

        fun COMMAND_LONG(cur: Cursor): test_.COMMAND_LONG {
            cur.init(com.company.demo.GroundControl.COMMAND_LONG.meta)
            return test_.COMMAND_LONG(cur)
        }

        fun GPS_RAW_INT(cur: Cursor): test_.GPS_RAW_INT {
            cur.init(com.company.demo.GroundControl.GPS_RAW_INT.meta)
            return test_.GPS_RAW_INT(cur)
        }

        fun RC_CHANNELS_SCALED(cur: Pack.Bytes): test_.RC_CHANNELS_SCALED {
            cur.init(com.company.demo.GroundControl.RC_CHANNELS_SCALED.meta)
            return test_.RC_CHANNELS_SCALED(cur)
        }

        fun RAW_PRESSURE(cur: Pack.Bytes): test_.RAW_PRESSURE {
            cur.init(com.company.demo.GroundControl.RAW_PRESSURE.meta)
            return test_.RAW_PRESSURE(cur)
        }

        fun ATTITUDE(cur: Pack.Bytes): test_.ATTITUDE {
            cur.init(com.company.demo.GroundControl.ATTITUDE.meta)
            return test_.ATTITUDE(cur)
        }

        fun MISSION_WRITE_PARTIAL_LIST(cur: Cursor): test_.MISSION_WRITE_PARTIAL_LIST {
            cur.init(com.company.demo.GroundControl.MISSION_WRITE_PARTIAL_LIST.meta)
            return test_.MISSION_WRITE_PARTIAL_LIST(cur)
        }

        fun MANUAL_SETPOINT(cur: Pack.Bytes): test_.MANUAL_SETPOINT {
            cur.init(com.company.demo.GroundControl.MANUAL_SETPOINT.meta)
            return test_.MANUAL_SETPOINT(cur)
        }

        fun SAFETY_ALLOWED_AREA(cur: Cursor): test_.SAFETY_ALLOWED_AREA {
            cur.init(com.company.demo.GroundControl.SAFETY_ALLOWED_AREA.meta)
            return test_.SAFETY_ALLOWED_AREA(cur)
        }

        fun MISSION_CLEAR_ALL(cur: Cursor): test_.MISSION_CLEAR_ALL {
            cur.init(com.company.demo.GroundControl.MISSION_CLEAR_ALL.meta)
            return test_.MISSION_CLEAR_ALL(cur)
        }

        fun SCALED_PRESSURE(cur: Pack.Bytes): test_.SCALED_PRESSURE {
            cur.init(com.company.demo.GroundControl.SCALED_PRESSURE.meta)
            return test_.SCALED_PRESSURE(cur)
        }

        fun HEARTBEAT(cur: Cursor): test_.HEARTBEAT {
            cur.init(com.company.demo.GroundControl.HEARTBEAT.meta)
            return test_.HEARTBEAT(cur)
        }

        fun PARAM_MAP_RC(cur: Cursor): test_.PARAM_MAP_RC {
            cur.init(com.company.demo.GroundControl.PARAM_MAP_RC.meta)
            return test_.PARAM_MAP_RC(cur)
        }

        fun MISSION_REQUEST_INT(cur: Cursor): test_.MISSION_REQUEST_INT {
            cur.init(com.company.demo.GroundControl.MISSION_REQUEST_INT.meta)
            return test_.MISSION_REQUEST_INT(cur)
        }

        fun LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(cur: Pack.Bytes): test_.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET {
            cur.init(com.company.demo.GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.meta)
            return test_.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(cur)
        }

        fun COMMAND_ACK(cur: Cursor): test_.COMMAND_ACK {
            cur.init(com.company.demo.GroundControl.COMMAND_ACK.meta)
            return test_.COMMAND_ACK(cur)
        }

        fun DATA_STREAM(cur: Pack.Bytes): test_.DATA_STREAM {
            cur.init(com.company.demo.GroundControl.DATA_STREAM.meta)
            return test_.DATA_STREAM(cur)
        }

        fun MISSION_REQUEST(cur: Cursor): test_.MISSION_REQUEST {
            cur.init(com.company.demo.GroundControl.MISSION_REQUEST.meta)
            return test_.MISSION_REQUEST(cur)
        }

        fun HIL_RC_INPUTS_RAW(cur: Pack.Bytes): test_.HIL_RC_INPUTS_RAW {
            cur.init(com.company.demo.GroundControl.HIL_RC_INPUTS_RAW.meta)
            return test_.HIL_RC_INPUTS_RAW(cur)
        }

        fun SET_MODE(cur: Cursor): test_.SET_MODE {
            cur.init(com.company.demo.GroundControl.SET_MODE.meta)
            return test_.SET_MODE(cur)
        }

        fun POSITION_TARGET_GLOBAL_INT(cur: Cursor): test_.POSITION_TARGET_GLOBAL_INT {
            cur.init(com.company.demo.GroundControl.POSITION_TARGET_GLOBAL_INT.meta)
            return test_.POSITION_TARGET_GLOBAL_INT(cur)
        }

        fun RC_CHANNELS_RAW(cur: Pack.Bytes): test_.RC_CHANNELS_RAW {
            cur.init(com.company.demo.GroundControl.RC_CHANNELS_RAW.meta)
            return test_.RC_CHANNELS_RAW(cur)
        }

        fun SERVO_OUTPUT_RAW(cur: Cursor): test_.SERVO_OUTPUT_RAW {
            cur.init(com.company.demo.GroundControl.SERVO_OUTPUT_RAW.meta)
            return test_.SERVO_OUTPUT_RAW(cur)
        }

        fun MISSION_ITEM_REACHED(cur: Pack.Bytes): test_.MISSION_ITEM_REACHED {
            cur.init(com.company.demo.GroundControl.MISSION_ITEM_REACHED.meta)
            return test_.MISSION_ITEM_REACHED(cur)
        }

        fun MISSION_ACK(cur: Cursor): test_.MISSION_ACK {
            cur.init(com.company.demo.GroundControl.MISSION_ACK.meta)
            return test_.MISSION_ACK(cur)
        }

        fun CHANGE_OPERATOR_CONTROL_ACK(cur: Pack.Bytes): test_.CHANGE_OPERATOR_CONTROL_ACK {
            cur.init(com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL_ACK.meta)
            return test_.CHANGE_OPERATOR_CONTROL_ACK(cur)
        }

        fun MISSION_CURRENT(cur: Pack.Bytes): test_.MISSION_CURRENT {
            cur.init(com.company.demo.GroundControl.MISSION_CURRENT.meta)
            return test_.MISSION_CURRENT(cur)
        }

        fun SYSTEM_TIME(cur: Pack.Bytes): test_.SYSTEM_TIME {
            cur.init(com.company.demo.GroundControl.SYSTEM_TIME.meta)
            return test_.SYSTEM_TIME(cur)
        }

        fun VISION_POSITION_ESTIMATE(cur: Pack.Bytes): test_.VISION_POSITION_ESTIMATE {
            cur.init(com.company.demo.GroundControl.VISION_POSITION_ESTIMATE.meta)
            return test_.VISION_POSITION_ESTIMATE(cur)
        }

        fun MANUAL_CONTROL(cur: Pack.Bytes): test_.MANUAL_CONTROL {
            cur.init(com.company.demo.GroundControl.MANUAL_CONTROL.meta)
            return test_.MANUAL_CONTROL(cur)
        }

        fun RC_CHANNELS(cur: Pack.Bytes): test_.RC_CHANNELS {
            cur.init(com.company.demo.GroundControl.RC_CHANNELS.meta)
            return test_.RC_CHANNELS(cur)
        }

        fun PARAM_VALUE(cur: Cursor): test_.PARAM_VALUE {
            cur.init(com.company.demo.GroundControl.PARAM_VALUE.meta)
            return test_.PARAM_VALUE(cur)
        }

        fun SET_POSITION_TARGET_LOCAL_NED(cur: Cursor): test_.SET_POSITION_TARGET_LOCAL_NED {
            cur.init(com.company.demo.GroundControl.SET_POSITION_TARGET_LOCAL_NED.meta)
            return test_.SET_POSITION_TARGET_LOCAL_NED(cur)
        }

        fun SET_GPS_GLOBAL_ORIGIN(cur: Cursor): test_.SET_GPS_GLOBAL_ORIGIN {
            cur.init(com.company.demo.GroundControl.SET_GPS_GLOBAL_ORIGIN.meta)
            return test_.SET_GPS_GLOBAL_ORIGIN(cur)
        }

        fun MISSION_REQUEST_LIST(cur: Cursor): test_.MISSION_REQUEST_LIST {
            cur.init(com.company.demo.GroundControl.MISSION_REQUEST_LIST.meta)
            return test_.MISSION_REQUEST_LIST(cur)
        }

        fun MISSION_REQUEST_PARTIAL_LIST(cur: Cursor): test_.MISSION_REQUEST_PARTIAL_LIST {
            cur.init(com.company.demo.GroundControl.MISSION_REQUEST_PARTIAL_LIST.meta)
            return test_.MISSION_REQUEST_PARTIAL_LIST(cur)
        }

        fun LOCAL_POSITION_NED(cur: Pack.Bytes): test_.LOCAL_POSITION_NED {
            cur.init(com.company.demo.GroundControl.LOCAL_POSITION_NED.meta)
            return test_.LOCAL_POSITION_NED(cur)
        }

        fun GPS_GLOBAL_ORIGIN(cur: Cursor): test_.GPS_GLOBAL_ORIGIN {
            cur.init(com.company.demo.GroundControl.GPS_GLOBAL_ORIGIN.meta)
            return test_.GPS_GLOBAL_ORIGIN(cur)
        }

        fun ATTITUDE_QUATERNION(cur: Pack.Bytes): test_.ATTITUDE_QUATERNION {
            cur.init(com.company.demo.GroundControl.ATTITUDE_QUATERNION.meta)
            return test_.ATTITUDE_QUATERNION(cur)
        }

        fun HIL_ACTUATOR_CONTROLS(cur: Cursor): test_.HIL_ACTUATOR_CONTROLS {
            cur.init(com.company.demo.GroundControl.HIL_ACTUATOR_CONTROLS.meta)
            return test_.HIL_ACTUATOR_CONTROLS(cur)
        }

        fun POSITION_TARGET_LOCAL_NED(cur: Cursor): test_.POSITION_TARGET_LOCAL_NED {
            cur.init(com.company.demo.GroundControl.POSITION_TARGET_LOCAL_NED.meta)
            return test_.POSITION_TARGET_LOCAL_NED(cur)
        }

        fun CHANGE_OPERATOR_CONTROL(cur: Cursor): test_.CHANGE_OPERATOR_CONTROL {
            cur.init(com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL.meta)
            return test_.CHANGE_OPERATOR_CONTROL(cur)
        }

        fun SYS_STATUS(cur: Cursor): test_.SYS_STATUS {
            cur.init(com.company.demo.GroundControl.SYS_STATUS.meta)
            return test_.SYS_STATUS(cur)
        }

        fun MISSION_ITEM(cur: Cursor): test_.MISSION_ITEM {
            cur.init(com.company.demo.GroundControl.MISSION_ITEM.meta)
            return test_.MISSION_ITEM(cur)
        }

        fun RAW_IMU(cur: Pack.Bytes): test_.RAW_IMU {
            cur.init(com.company.demo.GroundControl.RAW_IMU.meta)
            return test_.RAW_IMU(cur)
        }

        fun COMMAND_INT(cur: Cursor): test_.COMMAND_INT {
            cur.init(com.company.demo.GroundControl.COMMAND_INT.meta)
            return test_.COMMAND_INT(cur)
        }

        fun OPTICAL_FLOW(cur: Cursor): test_.OPTICAL_FLOW {
            cur.init(com.company.demo.GroundControl.OPTICAL_FLOW.meta)
            return test_.OPTICAL_FLOW(cur)
        }

        fun MISSION_ITEM_INT(cur: Cursor): test_.MISSION_ITEM_INT {
            cur.init(com.company.demo.GroundControl.MISSION_ITEM_INT.meta)
            return test_.MISSION_ITEM_INT(cur)
        }

        fun ATTITUDE_QUATERNION_COV(cur: Pack.Bytes): test_.ATTITUDE_QUATERNION_COV {
            cur.init(com.company.demo.GroundControl.ATTITUDE_QUATERNION_COV.meta)
            return test_.ATTITUDE_QUATERNION_COV(cur)
        }

        fun GLOBAL_VISION_POSITION_ESTIMATE(cur: Pack.Bytes): test_.GLOBAL_VISION_POSITION_ESTIMATE {
            cur.init(com.company.demo.GroundControl.GLOBAL_VISION_POSITION_ESTIMATE.meta)
            return test_.GLOBAL_VISION_POSITION_ESTIMATE(cur)
        }

    }

}


fun fill(pattitude_target: test_.ATTITUDE_TARGET) {

    pattitude_target.time_boot_ms()
    pattitude_target.type_mask()
    pattitude_target.q(floatArrayOf(3.100713E38F, -2.3420532E38F, 2.1687366E38F, 8.910963E37F))

    pattitude_target.body_roll_rate()
    pattitude_target.body_pitch_rate()
    pattitude_target.body_yaw_rate()
    pattitude_target.thrust()
}

fun onATTITUDE_TARGET(pattitude_target: com.company.demo.GroundControl.ATTITUDE_TARGET) {
    assert(pattitude_target.time_boot_ms())
    assert(pattitude_target.type_mask())
    assert(pattitude_target.q().same(floatArrayOf(3.100713E38F, -2.3420532E38F, 2.1687366E38F, 8.910963E37F)))

    assert(pattitude_target.body_roll_rate())
    assert(pattitude_target.body_pitch_rate())
    assert(pattitude_target.body_yaw_rate())
    assert(pattitude_target.thrust())
    println("ATTITUDE_TARGET \n")
}

fun fill(pmission_count: test_.MISSION_COUNT) {

    pmission_count.count()
    pmission_count.target_system()
    pmission_count.target_component()
    pmission_count.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION)
}

fun onMISSION_COUNT(pmission_count: com.company.demo.GroundControl.MISSION_COUNT) {
    assert(pmission_count.count())
    assert(pmission_count.target_system())
    assert(pmission_count.target_component())
    assert(pmission_count.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION)
    println("MISSION_COUNT \n")
}

fun fill(padsb_vehicle: com.company.demo.GroundControl.ADSB_VEHICLE) {

    padsb_vehicle.heading()
    padsb_vehicle.hor_velocity()
    padsb_vehicle.squawk()
    padsb_vehicle.ICAO_address()
    padsb_vehicle.lat()
    padsb_vehicle.lon()
    padsb_vehicle.altitude()
    padsb_vehicle.ver_velocity()
    padsb_vehicle.tslc()
    padsb_vehicle.altitude_type(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC)
    padsb_vehicle.callsign("VkbomzJxtyiyjezetlzgddktnmzrxrhztjodxhmzkmmoLiqzmirpjybdxnfxheStcqwlc", null)

    padsb_vehicle.emitter_type(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE)
    padsb_vehicle.flags(ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK)
}

fun onADSB_VEHICLE(padsb_vehicle: com.company.demo.GroundControl.ADSB_VEHICLE) {
    assert(padsb_vehicle.heading())
    assert(padsb_vehicle.hor_velocity())
    assert(padsb_vehicle.squawk())
    assert(padsb_vehicle.ICAO_address())
    assert(padsb_vehicle.lat())
    assert(padsb_vehicle.lon())
    assert(padsb_vehicle.altitude())
    assert(padsb_vehicle.ver_velocity())
    assert(padsb_vehicle.tslc())
    assert(padsb_vehicle.altitude_type()!!.get() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC)

    padsb_vehicle.callsign()?.let { item ->
        assert(item.get() == "VkbomzJxtyiyjezetlzgddktnmzrxrhztjodxhmzkmmoLiqzmirpjybdxnfxheStcqwlc")
    } ?: throw RuntimeException("null")

    assert(padsb_vehicle.emitter_type()!!.get() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE)
    assert(padsb_vehicle.flags()!!.get() == ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK)
    println("ADSB_VEHICLE \n")
}

fun fill(pmessage_interval: com.company.demo.GroundControl.MESSAGE_INTERVAL) {

    pmessage_interval.message_id()
    pmessage_interval.interval_us()
}

fun onMESSAGE_INTERVAL(pmessage_interval: com.company.demo.GroundControl.MESSAGE_INTERVAL) {
    assert(pmessage_interval.message_id())
    assert(pmessage_interval.interval_us())
    println("MESSAGE_INTERVAL \n")
}

fun fill(pekf_status_report: com.company.demo.GroundControl.EKF_STATUS_REPORT) {

    pekf_status_report.velocity_variance()
    pekf_status_report.pos_horiz_variance()
    pekf_status_report.pos_vert_variance()
    pekf_status_report.compass_variance()
    pekf_status_report.terrain_alt_variance()
    pekf_status_report.flags(EKF_STATUS_FLAGS.EKF_VELOCITY_VERT)
}

fun onEKF_STATUS_REPORT(pekf_status_report: com.company.demo.GroundControl.EKF_STATUS_REPORT) {
    assert(pekf_status_report.velocity_variance())
    assert(pekf_status_report.pos_horiz_variance())
    assert(pekf_status_report.pos_vert_variance())
    assert(pekf_status_report.compass_variance())
    assert(pekf_status_report.terrain_alt_variance())
    assert(pekf_status_report.flags()!!.get() == EKF_STATUS_FLAGS.EKF_VELOCITY_VERT)
    println("EKF_STATUS_REPORT \n")
}

fun fill(pestimator_status: com.company.demo.GroundControl.ESTIMATOR_STATUS) {

    pestimator_status.time_usec()
    pestimator_status.vel_ratio()
    pestimator_status.pos_horiz_ratio()
    pestimator_status.pos_vert_ratio()
    pestimator_status.mag_ratio()
    pestimator_status.hagl_ratio()
    pestimator_status.tas_ratio()
    pestimator_status.pos_horiz_accuracy()
    pestimator_status.pos_vert_accuracy()
    pestimator_status.flags(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH)
}

fun onESTIMATOR_STATUS(pestimator_status: com.company.demo.GroundControl.ESTIMATOR_STATUS) {
    assert(pestimator_status.time_usec())
    assert(pestimator_status.vel_ratio())
    assert(pestimator_status.pos_horiz_ratio())
    assert(pestimator_status.pos_vert_ratio())
    assert(pestimator_status.mag_ratio())
    assert(pestimator_status.hagl_ratio())
    assert(pestimator_status.tas_ratio())
    assert(pestimator_status.pos_horiz_accuracy())
    assert(pestimator_status.pos_vert_accuracy())
    assert(pestimator_status.flags()!!.get() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH)
    println("ESTIMATOR_STATUS \n")
}

fun fill(phwstatus: com.company.demo.GroundControl.HWSTATUS) {

    phwstatus.Vcc()
    phwstatus.I2Cerr()
}

fun onHWSTATUS(phwstatus: com.company.demo.GroundControl.HWSTATUS) {
    assert(phwstatus.Vcc())
    assert(phwstatus.I2Cerr())
    println("HWSTATUS \n")
}

fun fill(ptimesync: com.company.demo.GroundControl.TIMESYNC) {

    ptimesync.tc1()
    ptimesync.ts1()
}

fun onTIMESYNC(ptimesync: com.company.demo.GroundControl.TIMESYNC) {
    assert(ptimesync.tc1())
    assert(ptimesync.ts1())
    println("TIMESYNC \n")
}

fun fill(pparam_ext_request_list: com.company.demo.GroundControl.PARAM_EXT_REQUEST_LIST) {

    pparam_ext_request_list.target_system()
    pparam_ext_request_list.target_component()
}

fun onPARAM_EXT_REQUEST_LIST(pparam_ext_request_list: com.company.demo.GroundControl.PARAM_EXT_REQUEST_LIST) {
    assert(pparam_ext_request_list.target_system())
    assert(pparam_ext_request_list.target_component())
    println("PARAM_EXT_REQUEST_LIST \n")
}

fun fill(pglobal_position_int_cov: test_.GLOBAL_POSITION_INT_COV) {

    pglobal_position_int_cov.time_usec()
    pglobal_position_int_cov.lat()
    pglobal_position_int_cov.lon()
    pglobal_position_int_cov.alt()
    pglobal_position_int_cov.relative_alt()
    pglobal_position_int_cov.vx()
    pglobal_position_int_cov.vy()
    pglobal_position_int_cov.vz()
    pglobal_position_int_cov.covariance(floatArrayOf(-2.8993891E38F, 8.625999E37F, 1.1095573E38F, 2.9780145E38F, 8.0112034E37F, 1.222003E38F, -7.7030787E37F, -3.0341728E38F, 8.3000934E37F, 3.3032991E38F, -1.7402977E38F, 1.7932165E38F, 2.513922E38F, 2.6973286E38F, -1.912533E38F, -1.221261E38F, -2.9593827E38F, -1.3205703E38F, 3.85679E37F, -1.8199362E38F, 5.3009274E37F, 1.1589139E38F, 7.514914E37F, -2.08465E38F, -7.283785E37F, 2.4805302E38F, 1.2485667E38F, 1.654385E38F, 1.1460971E37F, -1.4204438E38F, -4.497626E37F, 2.6962444E38F, 2.728978E38F, 2.9376807E38F, 2.9412634E38F, -1.3072811E38F))

    pglobal_position_int_cov.estimator_type(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE)
}

fun onGLOBAL_POSITION_INT_COV(pglobal_position_int_cov: com.company.demo.GroundControl.GLOBAL_POSITION_INT_COV) {
    assert(pglobal_position_int_cov.time_usec())
    assert(pglobal_position_int_cov.lat())
    assert(pglobal_position_int_cov.lon())
    assert(pglobal_position_int_cov.alt())
    assert(pglobal_position_int_cov.relative_alt())
    assert(pglobal_position_int_cov.vx())
    assert(pglobal_position_int_cov.vy())
    assert(pglobal_position_int_cov.vz())
    assert(pglobal_position_int_cov.covariance().same(floatArrayOf(-2.8993891E38F, 8.625999E37F, 1.1095573E38F, 2.9780145E38F, 8.0112034E37F, 1.222003E38F, -7.7030787E37F, -3.0341728E38F, 8.3000934E37F, 3.3032991E38F, -1.7402977E38F, 1.7932165E38F, 2.513922E38F, 2.6973286E38F, -1.912533E38F, -1.221261E38F, -2.9593827E38F, -1.3205703E38F, 3.85679E37F, -1.8199362E38F, 5.3009274E37F, 1.1589139E38F, 7.514914E37F, -2.08465E38F, -7.283785E37F, 2.4805302E38F, 1.2485667E38F, 1.654385E38F, 1.1460971E37F, -1.4204438E38F, -4.497626E37F, 2.6962444E38F, 2.728978E38F, 2.9376807E38F, 2.9412634E38F, -1.3072811E38F)))

    assert(pglobal_position_int_cov.estimator_type()!!.get() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE)
    println("GLOBAL_POSITION_INT_COV \n")
}

fun fill(pbutton_change: com.company.demo.GroundControl.BUTTON_CHANGE) {

    pbutton_change.time_boot_ms()
    pbutton_change.last_change_ms()
    pbutton_change.state()
}

fun onBUTTON_CHANGE(pbutton_change: com.company.demo.GroundControl.BUTTON_CHANGE) {
    assert(pbutton_change.time_boot_ms())
    assert(pbutton_change.last_change_ms())
    assert(pbutton_change.state())
    println("BUTTON_CHANGE \n")
}

fun fill(psafety_set_allowed_area: test_.SAFETY_SET_ALLOWED_AREA) {

    psafety_set_allowed_area.target_system()
    psafety_set_allowed_area.target_component()
    psafety_set_allowed_area.p1x()
    psafety_set_allowed_area.p1y()
    psafety_set_allowed_area.p1z()
    psafety_set_allowed_area.p2x()
    psafety_set_allowed_area.p2y()
    psafety_set_allowed_area.p2z()
    psafety_set_allowed_area.frame(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
}

fun onSAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area: com.company.demo.GroundControl.SAFETY_SET_ALLOWED_AREA) {
    assert(psafety_set_allowed_area.target_system())
    assert(psafety_set_allowed_area.target_component())
    assert(psafety_set_allowed_area.p1x())
    assert(psafety_set_allowed_area.p1y())
    assert(psafety_set_allowed_area.p1z())
    assert(psafety_set_allowed_area.p2x())
    assert(psafety_set_allowed_area.p2y())
    assert(psafety_set_allowed_area.p2z())
    assert(psafety_set_allowed_area.frame()!!.get() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
    println("SAFETY_SET_ALLOWED_AREA \n")
}

fun fill(puavcan_node_status: com.company.demo.GroundControl.UAVCAN_NODE_STATUS) {

    puavcan_node_status.vendor_specific_status_code()
    puavcan_node_status.uptime_sec()
    puavcan_node_status.time_usec()
    puavcan_node_status.sub_mode()
    puavcan_node_status.health(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL)
    puavcan_node_status.mode(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE)
}

fun onUAVCAN_NODE_STATUS(puavcan_node_status: com.company.demo.GroundControl.UAVCAN_NODE_STATUS) {
    assert(puavcan_node_status.vendor_specific_status_code())
    assert(puavcan_node_status.uptime_sec())
    assert(puavcan_node_status.time_usec())
    assert(puavcan_node_status.sub_mode())
    assert(puavcan_node_status.health()!!.get() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL)
    assert(puavcan_node_status.mode()!!.get() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE)
    println("UAVCAN_NODE_STATUS \n")
}

fun fill(pcollision: com.company.demo.GroundControl.COLLISION) {

    pcollision.id()
    pcollision.time_to_minimum_delta()
    pcollision.altitude_minimum_delta()
    pcollision.horizontal_minimum_delta()
    pcollision.sRc(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT)
    pcollision.action(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR)
    pcollision.threat_level(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW)
}

fun onCOLLISION(pcollision: com.company.demo.GroundControl.COLLISION) {
    assert(pcollision.id())
    assert(pcollision.time_to_minimum_delta())
    assert(pcollision.altitude_minimum_delta())
    assert(pcollision.horizontal_minimum_delta())
    assert(pcollision.sRc()!!.get() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT)
    assert(pcollision.action()!!.get() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR)
    assert(pcollision.threat_level()!!.get() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW)
    println("COLLISION \n")
}

fun fill(pgimbal_torque_cmd_report: com.company.demo.GroundControl.GIMBAL_TORQUE_CMD_REPORT) {

    pgimbal_torque_cmd_report.target_system()
    pgimbal_torque_cmd_report.target_component()
    pgimbal_torque_cmd_report.rl_torque_cmd()
    pgimbal_torque_cmd_report.el_torque_cmd()
    pgimbal_torque_cmd_report.az_torque_cmd()
}

fun onGIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report: com.company.demo.GroundControl.GIMBAL_TORQUE_CMD_REPORT) {
    assert(pgimbal_torque_cmd_report.target_system())
    assert(pgimbal_torque_cmd_report.target_component())
    assert(pgimbal_torque_cmd_report.rl_torque_cmd())
    assert(pgimbal_torque_cmd_report.el_torque_cmd())
    assert(pgimbal_torque_cmd_report.az_torque_cmd())
    println("GIMBAL_TORQUE_CMD_REPORT \n")
}

fun fill(paltitude: com.company.demo.GroundControl.ALTITUDE) {

    paltitude.time_usec()
    paltitude.altitude_monotonic()
    paltitude.altitude_amsl()
    paltitude.altitude_local()
    paltitude.altitude_relative()
    paltitude.altitude_terrain()
    paltitude.bottom_clearance()
}

fun onALTITUDE(paltitude: com.company.demo.GroundControl.ALTITUDE) {
    assert(paltitude.time_usec())
    assert(paltitude.altitude_monotonic())
    assert(paltitude.altitude_amsl())
    assert(paltitude.altitude_local())
    assert(paltitude.altitude_relative())
    assert(paltitude.altitude_terrain())
    assert(paltitude.bottom_clearance())
    println("ALTITUDE \n")
}

fun fill(phil_state_quaternion: com.company.demo.GroundControl.HIL_STATE_QUATERNION) {

    phil_state_quaternion.ind_airspeed()
    phil_state_quaternion.true_airspeed()
    phil_state_quaternion.time_usec()
    phil_state_quaternion.attitude_quaternion(floatArrayOf(-3.345204E38F, -2.1098257E38F, -2.6873742E38F, 6.2367223E37F))

    phil_state_quaternion.rollspeed()
    phil_state_quaternion.pitchspeed()
    phil_state_quaternion.yawspeed()
    phil_state_quaternion.lat()
    phil_state_quaternion.lon()
    phil_state_quaternion.alt()
    phil_state_quaternion.vx()
    phil_state_quaternion.vy()
    phil_state_quaternion.vz()
    phil_state_quaternion.xacc()
    phil_state_quaternion.yacc()
    phil_state_quaternion.zacc()
}

fun onHIL_STATE_QUATERNION(phil_state_quaternion: com.company.demo.GroundControl.HIL_STATE_QUATERNION) {
    assert(phil_state_quaternion.ind_airspeed())
    assert(phil_state_quaternion.true_airspeed())
    assert(phil_state_quaternion.time_usec())
    assert(phil_state_quaternion.attitude_quaternion().same(floatArrayOf(-3.345204E38F, -2.1098257E38F, -2.6873742E38F, 6.2367223E37F)))

    assert(phil_state_quaternion.rollspeed())
    assert(phil_state_quaternion.pitchspeed())
    assert(phil_state_quaternion.yawspeed())
    assert(phil_state_quaternion.lat())
    assert(phil_state_quaternion.lon())
    assert(phil_state_quaternion.alt())
    assert(phil_state_quaternion.vx())
    assert(phil_state_quaternion.vy())
    assert(phil_state_quaternion.vz())
    assert(phil_state_quaternion.xacc())
    assert(phil_state_quaternion.yacc())
    assert(phil_state_quaternion.zacc())
    println("HIL_STATE_QUATERNION \n")
}

fun fill(psensor_offsets: com.company.demo.GroundControl.SENSOR_OFFSETS) {

    psensor_offsets.mag_ofs_x()
    psensor_offsets.mag_ofs_y()
    psensor_offsets.mag_ofs_z()
    psensor_offsets.mag_declination()
    psensor_offsets.raw_press()
    psensor_offsets.raw_temp()
    psensor_offsets.gyro_cal_x()
    psensor_offsets.gyro_cal_y()
    psensor_offsets.gyro_cal_z()
    psensor_offsets.accel_cal_x()
    psensor_offsets.accel_cal_y()
    psensor_offsets.accel_cal_z()
}

fun onSENSOR_OFFSETS(psensor_offsets: com.company.demo.GroundControl.SENSOR_OFFSETS) {
    assert(psensor_offsets.mag_ofs_x())
    assert(psensor_offsets.mag_ofs_y())
    assert(psensor_offsets.mag_ofs_z())
    assert(psensor_offsets.mag_declination())
    assert(psensor_offsets.raw_press())
    assert(psensor_offsets.raw_temp())
    assert(psensor_offsets.gyro_cal_x())
    assert(psensor_offsets.gyro_cal_y())
    assert(psensor_offsets.gyro_cal_z())
    assert(psensor_offsets.accel_cal_x())
    assert(psensor_offsets.accel_cal_y())
    assert(psensor_offsets.accel_cal_z())
    println("SENSOR_OFFSETS \n")
}

fun fill(pstorage_information: com.company.demo.GroundControl.STORAGE_INFORMATION) {

    pstorage_information.time_boot_ms()
    pstorage_information.storage_id()
    pstorage_information.storage_count()
    pstorage_information.status()
    pstorage_information.total_capacity()
    pstorage_information.used_capacity()
    pstorage_information.available_capacity()
    pstorage_information.read_speed()
    pstorage_information.write_speed()
}

fun onSTORAGE_INFORMATION(pstorage_information: com.company.demo.GroundControl.STORAGE_INFORMATION) {
    assert(pstorage_information.time_boot_ms())
    assert(pstorage_information.storage_id())
    assert(pstorage_information.storage_count())
    assert(pstorage_information.status())
    assert(pstorage_information.total_capacity())
    assert(pstorage_information.used_capacity())
    assert(pstorage_information.available_capacity())
    assert(pstorage_information.read_speed())
    assert(pstorage_information.write_speed())
    println("STORAGE_INFORMATION \n")
}

fun fill(pcamera_information: com.company.demo.GroundControl.CAMERA_INFORMATION) {

    pcamera_information.resolution_h()
    pcamera_information.resolution_v()
    pcamera_information.cam_definition_version()
    pcamera_information.time_boot_ms()
    pcamera_information.firmware_version()
    pcamera_information.vendor_name(byteArrayOf(19, 45, 32, 44, -80, 111, -7, 41, -25, -2, 68, -7, 24, -52, -102, -43, 46, -14, -84, -24, -77, 31, -87, -118, -114, 77, 71, 57, 116, -112, -92, 6))

    pcamera_information.model_name(byteArrayOf(-48, 75, 104, -70, -68, -60, 96, -33, -85, -63, -35, -30, 66, 28, -118, -47, -84, 103, -119, 109, 105, -75, 36, 106, -15, 76, 109, -54, -67, 12, 116, 23))

    pcamera_information.focal_length()
    pcamera_information.sensor_size_h()
    pcamera_information.sensor_size_v()
    pcamera_information.lens_id()
    pcamera_information.flags(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE)
    pcamera_information.cam_definition_uri("lwdwdsmrrcjfukotuyptvgmmAc", null)

}

fun onCAMERA_INFORMATION(pcamera_information: com.company.demo.GroundControl.CAMERA_INFORMATION) {
    assert(pcamera_information.resolution_h())
    assert(pcamera_information.resolution_v())
    assert(pcamera_information.cam_definition_version())
    assert(pcamera_information.time_boot_ms())
    assert(pcamera_information.firmware_version())
    assert(pcamera_information.vendor_name().same(byteArrayOf(19, 45, 32, 44, -80, 111, -7, 41, -25, -2, 68, -7, 24, -52, -102, -43, 46, -14, -84, -24, -77, 31, -87, -118, -114, 77, 71, 57, 116, -112, -92, 6)))

    assert(pcamera_information.model_name().same(byteArrayOf(-48, 75, 104, -70, -68, -60, 96, -33, -85, -63, -35, -30, 66, 28, -118, -47, -84, 103, -119, 109, 105, -75, 36, 106, -15, 76, 109, -54, -67, 12, 116, 23)))

    assert(pcamera_information.focal_length())
    assert(pcamera_information.sensor_size_h())
    assert(pcamera_information.sensor_size_v())
    assert(pcamera_information.lens_id())
    assert(pcamera_information.flags()!!.get() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE)

    pcamera_information.cam_definition_uri()?.let { item ->
        assert(item.get() == "lwdwdsmrrcjfukotuyptvgmmAc")
    } ?: throw RuntimeException("null")

    println("CAMERA_INFORMATION \n")
}

fun fill(pgps_status: test_.GPS_STATUS) {

    pgps_status.satellites_visible()
    pgps_status.satellite_prn(byteArrayOf(-37, -107, -21, -76, 79, 11, 7, 28, -18, -77, 123, -46, -57, 94, -8, 53, 32, 70, -43, 4))

    pgps_status.satellite_used(byteArrayOf(96, -60, -76, 73, 9, 14, -104, 49, -123, -15, -126, 35, 95, 10, -71, -43, -103, -2, -37, 48))

    pgps_status.satellite_elevation(byteArrayOf(-74, 106, -87, 5, -96, -109, 114, 99, -1, 120, 88, -10, 41, -90, 62, -126, 108, 60, 72, 10))

    pgps_status.satellite_azimuth(byteArrayOf(25, -51, 85, 50, -102, 75, -76, -16, -79, 31, -50, -42, 7, -79, -123, -22, -97, -127, 55, 12))

    pgps_status.satellite_snr(byteArrayOf(18, -124, 53, 100, -33, -99, 6, 73, 95, 103, 97, 21, -27, 69, 30, -95, -42, 108, 109, -95))

}

fun onGPS_STATUS(pgps_status: com.company.demo.GroundControl.GPS_STATUS) {
    assert(pgps_status.satellites_visible())
    assert(pgps_status.satellite_prn().same(byteArrayOf(-37, -107, -21, -76, 79, 11, 7, 28, -18, -77, 123, -46, -57, 94, -8, 53, 32, 70, -43, 4)))

    assert(pgps_status.satellite_used().same(byteArrayOf(96, -60, -76, 73, 9, 14, -104, 49, -123, -15, -126, 35, 95, 10, -71, -43, -103, -2, -37, 48)))

    assert(pgps_status.satellite_elevation().same(byteArrayOf(-74, 106, -87, 5, -96, -109, 114, 99, -1, 120, 88, -10, 41, -90, 62, -126, 108, 60, 72, 10)))

    assert(pgps_status.satellite_azimuth().same(byteArrayOf(25, -51, 85, 50, -102, 75, -76, -16, -79, 31, -50, -42, 7, -79, -123, -22, -97, -127, 55, 12)))

    assert(pgps_status.satellite_snr().same(byteArrayOf(18, -124, 53, 100, -33, -99, 6, 73, 95, 103, 97, 21, -27, 69, 30, -95, -42, 108, 109, -95)))

    println("GPS_STATUS \n")
}

fun fill(pdevice_op_write_reply: com.company.demo.GroundControl.DEVICE_OP_WRITE_REPLY) {

    pdevice_op_write_reply.request_id()
    pdevice_op_write_reply.result()
}

fun onDEVICE_OP_WRITE_REPLY(pdevice_op_write_reply: com.company.demo.GroundControl.DEVICE_OP_WRITE_REPLY) {
    assert(pdevice_op_write_reply.request_id())
    assert(pdevice_op_write_reply.result())
    println("DEVICE_OP_WRITE_REPLY \n")
}

fun fill(pparam_set: test_.PARAM_SET) {

    pparam_set.target_system()
    pparam_set.target_component()
    pparam_set.param_value()
    pparam_set.param_id("lrrwymKihzkhrzRdguauQnsgslwniuGorvyfguJizduzne", null)

    pparam_set.param_type(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64)
}

fun onPARAM_SET(pparam_set: com.company.demo.GroundControl.PARAM_SET) {
    assert(pparam_set.target_system())
    assert(pparam_set.target_component())
    assert(pparam_set.param_value())

    pparam_set.param_id()?.let { item ->
        assert(item.get() == "lrrwymKihzkhrzRdguauQnsgslwniuGorvyfguJizduzne")
    } ?: throw RuntimeException("null")

    assert(pparam_set.param_type()!!.get() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64)
    println("PARAM_SET \n")
}

fun fill(pterrain_data: com.company.demo.GroundControl.TERRAIN_DATA) {

    pterrain_data.grid_spacing()
    pterrain_data.lat()
    pterrain_data.lon()
    pterrain_data.gridbit()
    pterrain_data.daTa(shortArrayOf(-7648, -9127, -10197, -11040, 10286, 32388, -7958, 8711, -32134, -29393, 20534, -29032, 6457, 3824, -30614, 16927))

}

fun onTERRAIN_DATA(pterrain_data: com.company.demo.GroundControl.TERRAIN_DATA) {
    assert(pterrain_data.grid_spacing())
    assert(pterrain_data.lat())
    assert(pterrain_data.lon())
    assert(pterrain_data.gridbit())
    assert(pterrain_data.daTa().same(shortArrayOf(-7648, -9127, -10197, -11040, 10286, 32388, -7958, 8711, -32134, -29393, 20534, -29032, 6457, 3824, -30614, 16927)))

    println("TERRAIN_DATA \n")
}

fun fill(pgimbal_control: com.company.demo.GroundControl.GIMBAL_CONTROL) {

    pgimbal_control.target_system()
    pgimbal_control.target_component()
    pgimbal_control.demanded_rate_x()
    pgimbal_control.demanded_rate_y()
    pgimbal_control.demanded_rate_z()
}

fun onGIMBAL_CONTROL(pgimbal_control: com.company.demo.GroundControl.GIMBAL_CONTROL) {
    assert(pgimbal_control.target_system())
    assert(pgimbal_control.target_component())
    assert(pgimbal_control.demanded_rate_x())
    assert(pgimbal_control.demanded_rate_y())
    assert(pgimbal_control.demanded_rate_z())
    println("GIMBAL_CONTROL \n")
}

fun fill(prc_channels_override: test_.RC_CHANNELS_OVERRIDE) {

    prc_channels_override.chan1_raw()
    prc_channels_override.chan2_raw()
    prc_channels_override.chan3_raw()
    prc_channels_override.chan4_raw()
    prc_channels_override.chan5_raw()
    prc_channels_override.chan6_raw()
    prc_channels_override.chan7_raw()
    prc_channels_override.chan8_raw()
    prc_channels_override.target_system()
    prc_channels_override.target_component()
}

fun onRC_CHANNELS_OVERRIDE(prc_channels_override: com.company.demo.GroundControl.RC_CHANNELS_OVERRIDE) {
    assert(prc_channels_override.chan1_raw())
    assert(prc_channels_override.chan2_raw())
    assert(prc_channels_override.chan3_raw())
    assert(prc_channels_override.chan4_raw())
    assert(prc_channels_override.chan5_raw())
    assert(prc_channels_override.chan6_raw())
    assert(prc_channels_override.chan7_raw())
    assert(prc_channels_override.chan8_raw())
    assert(prc_channels_override.target_system())
    assert(prc_channels_override.target_component())
    println("RC_CHANNELS_OVERRIDE \n")
}

fun fill(pscaled_imu: test_.SCALED_IMU) {

    pscaled_imu.time_boot_ms()
    pscaled_imu.xacc()
    pscaled_imu.yacc()
    pscaled_imu.zacc()
    pscaled_imu.xgyro()
    pscaled_imu.ygyro()
    pscaled_imu.zgyro()
    pscaled_imu.xmag()
    pscaled_imu.ymag()
    pscaled_imu.zmag()
}

fun onSCALED_IMU(pscaled_imu: com.company.demo.GroundControl.SCALED_IMU) {
    assert(pscaled_imu.time_boot_ms())
    assert(pscaled_imu.xacc())
    assert(pscaled_imu.yacc())
    assert(pscaled_imu.zacc())
    assert(pscaled_imu.xgyro())
    assert(pscaled_imu.ygyro())
    assert(pscaled_imu.zgyro())
    assert(pscaled_imu.xmag())
    assert(pscaled_imu.ymag())
    assert(pscaled_imu.zmag())
    println("SCALED_IMU \n")
}

fun fill(pvideo_stream_information: com.company.demo.GroundControl.VIDEO_STREAM_INFORMATION) {

    pvideo_stream_information.resolution_h()
    pvideo_stream_information.resolution_v()
    pvideo_stream_information.rotation()
    pvideo_stream_information.bitrate()
    pvideo_stream_information.camera_id()
    pvideo_stream_information.status()
    pvideo_stream_information.framerate()
    pvideo_stream_information.uri("mpfuixhxcspfkOrkucxefoqhqumbnLygjseajoynvhazpbqkexspobRcIlkcfwgfpsjlhhebxpnnp", null)

}

fun onVIDEO_STREAM_INFORMATION(pvideo_stream_information: com.company.demo.GroundControl.VIDEO_STREAM_INFORMATION) {
    assert(pvideo_stream_information.resolution_h())
    assert(pvideo_stream_information.resolution_v())
    assert(pvideo_stream_information.rotation())
    assert(pvideo_stream_information.bitrate())
    assert(pvideo_stream_information.camera_id())
    assert(pvideo_stream_information.status())
    assert(pvideo_stream_information.framerate())

    pvideo_stream_information.uri()?.let { item ->
        assert(item.get() == "mpfuixhxcspfkOrkucxefoqhqumbnLygjseajoynvhazpbqkexspobRcIlkcfwgfpsjlhhebxpnnp")
    } ?: throw RuntimeException("null")

    println("VIDEO_STREAM_INFORMATION \n")
}

fun fill(pahrs: com.company.demo.GroundControl.AHRS) {

    pahrs.omegaIx()
    pahrs.omegaIy()
    pahrs.omegaIz()
    pahrs.accel_weight()
    pahrs.renorm_val()
    pahrs.error_rp()
    pahrs.error_yaw()
}

fun onAHRS(pahrs: com.company.demo.GroundControl.AHRS) {
    assert(pahrs.omegaIx())
    assert(pahrs.omegaIy())
    assert(pahrs.omegaIz())
    assert(pahrs.accel_weight())
    assert(pahrs.renorm_val())
    assert(pahrs.error_rp())
    assert(pahrs.error_yaw())
    println("AHRS \n")
}

fun fill(pdebug: com.company.demo.GroundControl.DEBUG) {

    pdebug.time_boot_ms()
    pdebug.ind()
    pdebug.value()
}

fun onDEBUG(pdebug: com.company.demo.GroundControl.DEBUG) {
    assert(pdebug.time_boot_ms())
    assert(pdebug.ind())
    assert(pdebug.value())
    println("DEBUG \n")
}

fun fill(pcamera_image_captured: com.company.demo.GroundControl.CAMERA_IMAGE_CAPTURED) {

    pcamera_image_captured.time_boot_ms()
    pcamera_image_captured.time_utc()
    pcamera_image_captured.camera_id()
    pcamera_image_captured.lat()
    pcamera_image_captured.lon()
    pcamera_image_captured.alt()
    pcamera_image_captured.relative_alt()
    pcamera_image_captured.q(floatArrayOf(-3.0754911E38F, -2.4464407E38F, -2.7317163E38F, -3.5744945E37F))

    pcamera_image_captured.image_index()
    pcamera_image_captured.capture_result()
    pcamera_image_captured.file_url("gjfw", null)

}

fun onCAMERA_IMAGE_CAPTURED(pcamera_image_captured: com.company.demo.GroundControl.CAMERA_IMAGE_CAPTURED) {
    assert(pcamera_image_captured.time_boot_ms())
    assert(pcamera_image_captured.time_utc())
    assert(pcamera_image_captured.camera_id())
    assert(pcamera_image_captured.lat())
    assert(pcamera_image_captured.lon())
    assert(pcamera_image_captured.alt())
    assert(pcamera_image_captured.relative_alt())
    assert(pcamera_image_captured.q().same(floatArrayOf(-3.0754911E38F, -2.4464407E38F, -2.7317163E38F, -3.5744945E37F)))

    assert(pcamera_image_captured.image_index())
    assert(pcamera_image_captured.capture_result())

    pcamera_image_captured.file_url()?.let { item ->
        assert(item.get() == "gjfw")
    } ?: throw RuntimeException("null")

    println("CAMERA_IMAGE_CAPTURED \n")
}

fun fill(plog_entry: com.company.demo.GroundControl.LOG_ENTRY) {

    plog_entry.id()
    plog_entry.num_logs()
    plog_entry.last_log_num()
    plog_entry.time_utc()
    plog_entry.size()
}

fun onLOG_ENTRY(plog_entry: com.company.demo.GroundControl.LOG_ENTRY) {
    assert(plog_entry.id())
    assert(plog_entry.num_logs())
    assert(plog_entry.last_log_num())
    assert(plog_entry.time_utc())
    assert(plog_entry.size())
    println("LOG_ENTRY \n")
}

fun fill(pactuator_control_target: com.company.demo.GroundControl.ACTUATOR_CONTROL_TARGET) {

    pactuator_control_target.time_usec()
    pactuator_control_target.group_mlx()
    pactuator_control_target.controls(floatArrayOf(1.7192804E38F, 1.485532E38F, -1.7701094E38F, -2.0620315E38F, -2.3011578E38F, -1.3389108E38F, 2.4771373E38F, 1.4836473E38F))

}

fun onACTUATOR_CONTROL_TARGET(pactuator_control_target: com.company.demo.GroundControl.ACTUATOR_CONTROL_TARGET) {
    assert(pactuator_control_target.time_usec())
    assert(pactuator_control_target.group_mlx())
    assert(pactuator_control_target.controls().same(floatArrayOf(1.7192804E38F, 1.485532E38F, -1.7701094E38F, -2.0620315E38F, -2.3011578E38F, -1.3389108E38F, 2.4771373E38F, 1.4836473E38F)))

    println("ACTUATOR_CONTROL_TARGET \n")
}

fun fill(phigh_latency: com.company.demo.GroundControl.HIGH_LATENCY) {

    phigh_latency.heading()
    phigh_latency.wp_distance()
    phigh_latency.custom_mode()
    phigh_latency.roll()
    phigh_latency.pitch()
    phigh_latency.throttle()
    phigh_latency.heading_sp()
    phigh_latency.latitude()
    phigh_latency.longitude()
    phigh_latency.altitude_amsl()
    phigh_latency.altitude_sp()
    phigh_latency.airspeed()
    phigh_latency.airspeed_sp()
    phigh_latency.groundspeed()
    phigh_latency.climb_rate()
    phigh_latency.gps_nsat()
    phigh_latency.battery_remaining()
    phigh_latency.temperature()
    phigh_latency.temperature_air()
    phigh_latency.failsafe()
    phigh_latency.wp_num()
    phigh_latency.base_mode(MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
    phigh_latency.landed_state(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING)
    phigh_latency.gps_fix_type(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX)
}

fun onHIGH_LATENCY(phigh_latency: com.company.demo.GroundControl.HIGH_LATENCY) {
    assert(phigh_latency.heading())
    assert(phigh_latency.wp_distance())
    assert(phigh_latency.custom_mode())
    assert(phigh_latency.roll())
    assert(phigh_latency.pitch())
    assert(phigh_latency.throttle())
    assert(phigh_latency.heading_sp())
    assert(phigh_latency.latitude())
    assert(phigh_latency.longitude())
    assert(phigh_latency.altitude_amsl())
    assert(phigh_latency.altitude_sp())
    assert(phigh_latency.airspeed())
    assert(phigh_latency.airspeed_sp())
    assert(phigh_latency.groundspeed())
    assert(phigh_latency.climb_rate())
    assert(phigh_latency.gps_nsat())
    assert(phigh_latency.battery_remaining())
    assert(phigh_latency.temperature())
    assert(phigh_latency.temperature_air())
    assert(phigh_latency.failsafe())
    assert(phigh_latency.wp_num())
    assert(phigh_latency.base_mode()!!.get() == MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
    assert(phigh_latency.landed_state()!!.get() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING)
    assert(phigh_latency.gps_fix_type()!!.get() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX)
    println("HIGH_LATENCY \n")
}

fun fill(pparam_request_read: test_.PARAM_REQUEST_READ) {

    pparam_request_read.target_system()
    pparam_request_read.target_component()
    pparam_request_read.param_index()
    pparam_request_read.param_id("whpobkgbeSzovbtyrhuvqcmCwkiontftlfbmiwrrosipjkLlspuQkaNpHxwakpeIr", null)

}

fun onPARAM_REQUEST_READ(pparam_request_read: com.company.demo.GroundControl.PARAM_REQUEST_READ) {
    assert(pparam_request_read.target_system())
    assert(pparam_request_read.target_component())
    assert(pparam_request_read.param_index())

    pparam_request_read.param_id()?.let { item ->
        assert(item.get() == "whpobkgbeSzovbtyrhuvqcmCwkiontftlfbmiwrrosipjkLlspuQkaNpHxwakpeIr")
    } ?: throw RuntimeException("null")

    println("PARAM_REQUEST_READ \n")
}

fun fill(pset_attitude_target: test_.SET_ATTITUDE_TARGET) {

    pset_attitude_target.time_boot_ms()
    pset_attitude_target.target_system()
    pset_attitude_target.target_component()
    pset_attitude_target.type_mask()
    pset_attitude_target.q(floatArrayOf(-1.0479587E38F, 1.3690727E38F, 2.3284415E38F, 2.2602097E38F))

    pset_attitude_target.body_roll_rate()
    pset_attitude_target.body_pitch_rate()
    pset_attitude_target.body_yaw_rate()
    pset_attitude_target.thrust()
}

fun onSET_ATTITUDE_TARGET(pset_attitude_target: com.company.demo.GroundControl.SET_ATTITUDE_TARGET) {
    assert(pset_attitude_target.time_boot_ms())
    assert(pset_attitude_target.target_system())
    assert(pset_attitude_target.target_component())
    assert(pset_attitude_target.type_mask())
    assert(pset_attitude_target.q().same(floatArrayOf(-1.0479587E38F, 1.3690727E38F, 2.3284415E38F, 2.2602097E38F)))

    assert(pset_attitude_target.body_roll_rate())
    assert(pset_attitude_target.body_pitch_rate())
    assert(pset_attitude_target.body_yaw_rate())
    assert(pset_attitude_target.thrust())
    println("SET_ATTITUDE_TARGET \n")
}

fun fill(pfollow_target: com.company.demo.GroundControl.FOLLOW_TARGET) {

    pfollow_target.timestamp()
    pfollow_target.custom_state()
    pfollow_target.est_capabilities()
    pfollow_target.lat()
    pfollow_target.lon()
    pfollow_target.alt()
    pfollow_target.vel(floatArrayOf(-3.566916E37F, 1.1539986E38F, -3.36795E38F))

    pfollow_target.acc(floatArrayOf(2.068706E38F, -2.5664246E38F, 7.9460614E37F))

    pfollow_target.attitude_q(floatArrayOf(1.0815342E38F, 1.4769533E38F, 1.2103664E38F, 2.2689826E38F))

    pfollow_target.rates(floatArrayOf(-1.7916174E38F, 1.4406487E37F, -3.3890057E38F))

    pfollow_target.position_cov(floatArrayOf(-3.3384707E37F, -1.4429899E38F, 2.1825907E38F))

}

fun onFOLLOW_TARGET(pfollow_target: com.company.demo.GroundControl.FOLLOW_TARGET) {
    assert(pfollow_target.timestamp())
    assert(pfollow_target.custom_state())
    assert(pfollow_target.est_capabilities())
    assert(pfollow_target.lat())
    assert(pfollow_target.lon())
    assert(pfollow_target.alt())
    assert(pfollow_target.vel().same(floatArrayOf(-3.566916E37F, 1.1539986E38F, -3.36795E38F)))

    assert(pfollow_target.acc().same(floatArrayOf(2.068706E38F, -2.5664246E38F, 7.9460614E37F)))

    assert(pfollow_target.attitude_q().same(floatArrayOf(1.0815342E38F, 1.4769533E38F, 1.2103664E38F, 2.2689826E38F)))

    assert(pfollow_target.rates().same(floatArrayOf(-1.7916174E38F, 1.4406487E37F, -3.3890057E38F)))

    assert(pfollow_target.position_cov().same(floatArrayOf(-3.3384707E37F, -1.4429899E38F, 2.1825907E38F)))

    println("FOLLOW_TARGET \n")
}

fun fill(phil_state: test_.HIL_STATE) {

    phil_state.time_usec()
    phil_state.roll()
    phil_state.pitch()
    phil_state.yaw()
    phil_state.rollspeed()
    phil_state.pitchspeed()
    phil_state.yawspeed()
    phil_state.lat()
    phil_state.lon()
    phil_state.alt()
    phil_state.vx()
    phil_state.vy()
    phil_state.vz()
    phil_state.xacc()
    phil_state.yacc()
    phil_state.zacc()
}

fun onHIL_STATE(phil_state: com.company.demo.GroundControl.HIL_STATE) {
    assert(phil_state.time_usec())
    assert(phil_state.roll())
    assert(phil_state.pitch())
    assert(phil_state.yaw())
    assert(phil_state.rollspeed())
    assert(phil_state.pitchspeed())
    assert(phil_state.yawspeed())
    assert(phil_state.lat())
    assert(phil_state.lon())
    assert(phil_state.alt())
    assert(phil_state.vx())
    assert(phil_state.vy())
    assert(phil_state.vz())
    assert(phil_state.xacc())
    assert(phil_state.yacc())
    assert(phil_state.zacc())
    println("HIL_STATE \n")
}

fun fill(phome_position: com.company.demo.GroundControl.HOME_POSITION) {

    phome_position.latitude()
    phome_position.longitude()
    phome_position.altitude()
    phome_position.x()
    phome_position.y()
    phome_position.z()
    phome_position.q(floatArrayOf(7.960288E37F, -1.8365978E37F, -5.6342425E37F, 2.587629E38F))

    phome_position.approach_x()
    phome_position.approach_y()
    phome_position.approach_z()
    phome_position.time_usec()
}

fun onHOME_POSITION(phome_position: com.company.demo.GroundControl.HOME_POSITION) {
    assert(phome_position.latitude())
    assert(phome_position.longitude())
    assert(phome_position.altitude())
    assert(phome_position.x())
    assert(phome_position.y())
    assert(phome_position.z())
    assert(phome_position.q().same(floatArrayOf(7.960288E37F, -1.8365978E37F, -5.6342425E37F, 2.587629E38F)))

    assert(phome_position.approach_x())
    assert(phome_position.approach_y())
    assert(phome_position.approach_z())
    assert(phome_position.time_usec())
    println("HOME_POSITION \n")
}

fun fill(pfence_status: com.company.demo.GroundControl.FENCE_STATUS) {

    pfence_status.breach_count()
    pfence_status.breach_time()
    pfence_status.breach_status()
    pfence_status.breach_type(FENCE_BREACH.FENCE_BREACH_NONE)
}

fun onFENCE_STATUS(pfence_status: com.company.demo.GroundControl.FENCE_STATUS) {
    assert(pfence_status.breach_count())
    assert(pfence_status.breach_time())
    assert(pfence_status.breach_status())
    assert(pfence_status.breach_type()!!.get() == FENCE_BREACH.FENCE_BREACH_NONE)
    println("FENCE_STATUS \n")
}

fun fill(premote_log_block_status: com.company.demo.GroundControl.REMOTE_LOG_BLOCK_STATUS) {

    premote_log_block_status.seqno()
    premote_log_block_status.target_system()
    premote_log_block_status.target_component()
    premote_log_block_status.status(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK)
}

fun onREMOTE_LOG_BLOCK_STATUS(premote_log_block_status: com.company.demo.GroundControl.REMOTE_LOG_BLOCK_STATUS) {
    assert(premote_log_block_status.seqno())
    assert(premote_log_block_status.target_system())
    assert(premote_log_block_status.target_component())
    assert(premote_log_block_status.status()!!.get() == MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK)
    println("REMOTE_LOG_BLOCK_STATUS \n")
}

fun fill(pobstacle_distance: com.company.demo.GroundControl.OBSTACLE_DISTANCE) {

    pobstacle_distance.distances(shortArrayOf(31731, -32142, -1403, 6770, 18039, 14712, -29382, 18890, 20119, -8220, -13506, 11084, -15288, -11840, -26942, -22620, 23784, 11990, 9182, 31212, 31018, -3385, 6598, -15909, -27279, -30159, -16899, 14745, 10226, -11726, 12418, 13156, -24921, 17248, 32189, 3879, -12700, 28319, 18161, -22261, -17838, -15088, 15445, 9455, -20081, -1648, -19684, -24711, -9021, 28873, 31043, -10171, 12439, 23198, -4262, -2221, -14312, 485, -26984, -18087, 23073, -18308, 8892, -15442, -17060, 18639, 17791, -7321, 31916, 32335, 21819, 21093))

    pobstacle_distance.min_distance()
    pobstacle_distance.max_distance()
    pobstacle_distance.time_usec()
    pobstacle_distance.increment()
    pobstacle_distance.sensor_type(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND)
}

fun onOBSTACLE_DISTANCE(pobstacle_distance: com.company.demo.GroundControl.OBSTACLE_DISTANCE) {
    assert(pobstacle_distance.distances().same(shortArrayOf(31731, -32142, -1403, 6770, 18039, 14712, -29382, 18890, 20119, -8220, -13506, 11084, -15288, -11840, -26942, -22620, 23784, 11990, 9182, 31212, 31018, -3385, 6598, -15909, -27279, -30159, -16899, 14745, 10226, -11726, 12418, 13156, -24921, 17248, 32189, 3879, -12700, 28319, 18161, -22261, -17838, -15088, 15445, 9455, -20081, -1648, -19684, -24711, -9021, 28873, 31043, -10171, 12439, 23198, -4262, -2221, -14312, 485, -26984, -18087, 23073, -18308, 8892, -15442, -17060, 18639, 17791, -7321, 31916, 32335, 21819, 21093)))

    assert(pobstacle_distance.min_distance())
    assert(pobstacle_distance.max_distance())
    assert(pobstacle_distance.time_usec())
    assert(pobstacle_distance.increment())
    assert(pobstacle_distance.sensor_type()!!.get() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND)
    println("OBSTACLE_DISTANCE \n")
}

fun fill(pgps2_raw: com.company.demo.GroundControl.GPS2_RAW) {

    pgps2_raw.eph()
    pgps2_raw.epv()
    pgps2_raw.vel()
    pgps2_raw.cog()
    pgps2_raw.dgps_age()
    pgps2_raw.time_usec()
    pgps2_raw.lat()
    pgps2_raw.lon()
    pgps2_raw.alt()
    pgps2_raw.satellites_visible()
    pgps2_raw.dgps_numch()
    pgps2_raw.fix_type(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX)
}

fun onGPS2_RAW(pgps2_raw: com.company.demo.GroundControl.GPS2_RAW) {
    assert(pgps2_raw.eph())
    assert(pgps2_raw.epv())
    assert(pgps2_raw.vel())
    assert(pgps2_raw.cog())
    assert(pgps2_raw.dgps_age())
    assert(pgps2_raw.time_usec())
    assert(pgps2_raw.lat())
    assert(pgps2_raw.lon())
    assert(pgps2_raw.alt())
    assert(pgps2_raw.satellites_visible())
    assert(pgps2_raw.dgps_numch())
    assert(pgps2_raw.fix_type()!!.get() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX)
    println("GPS2_RAW \n")
}

fun fill(prequest_data_stream: test_.REQUEST_DATA_STREAM) {

    prequest_data_stream.req_message_rate()
    prequest_data_stream.target_system()
    prequest_data_stream.target_component()
    prequest_data_stream.req_stream_id()
    prequest_data_stream.start_stop()
}

fun onREQUEST_DATA_STREAM(prequest_data_stream: com.company.demo.GroundControl.REQUEST_DATA_STREAM) {
    assert(prequest_data_stream.req_message_rate())
    assert(prequest_data_stream.target_system())
    assert(prequest_data_stream.target_component())
    assert(prequest_data_stream.req_stream_id())
    assert(prequest_data_stream.start_stop())
    println("REQUEST_DATA_STREAM \n")
}

fun fill(pmemory_vect: com.company.demo.GroundControl.MEMORY_VECT) {

    pmemory_vect.address()
    pmemory_vect.ver()
    pmemory_vect.typE()
    pmemory_vect.value(byteArrayOf(112, 119, 101, -6, -19, 121, -120, 77, 44, -79, -90, 97, 109, -122, -79, 126, 24, 114, 107, 22, -73, 67, 2, 118, 78, 15, 62, 51, -109, 17, 35, 34))

}

fun onMEMORY_VECT(pmemory_vect: com.company.demo.GroundControl.MEMORY_VECT) {
    assert(pmemory_vect.address())
    assert(pmemory_vect.ver())
    assert(pmemory_vect.typE())
    assert(pmemory_vect.value().same(byteArrayOf(112, 119, 101, -6, -19, 121, -120, 77, 44, -79, -90, 97, 109, -122, -79, 126, 24, 114, 107, 22, -73, 67, 2, 118, 78, 15, 62, 51, -109, 17, 35, 34)))

    println("MEMORY_VECT \n")
}

fun fill(pparam_ext_request_read: com.company.demo.GroundControl.PARAM_EXT_REQUEST_READ) {

    pparam_ext_request_read.target_system()
    pparam_ext_request_read.target_component()
    pparam_ext_request_read.param_index()
    pparam_ext_request_read.param_id("juzaW", null)

}

fun onPARAM_EXT_REQUEST_READ(pparam_ext_request_read: com.company.demo.GroundControl.PARAM_EXT_REQUEST_READ) {
    assert(pparam_ext_request_read.target_system())
    assert(pparam_ext_request_read.target_component())
    assert(pparam_ext_request_read.param_index())

    pparam_ext_request_read.param_id()?.let { item ->
        assert(item.get() == "juzaW")
    } ?: throw RuntimeException("null")

    println("PARAM_EXT_REQUEST_READ \n")
}

fun fill(phil_controls: test_.HIL_CONTROLS) {

    phil_controls.time_usec()
    phil_controls.roll_ailerons()
    phil_controls.pitch_elevator()
    phil_controls.yaw_rudder()
    phil_controls.throttle()
    phil_controls.aux1()
    phil_controls.aux2()
    phil_controls.aux3()
    phil_controls.aux4()
    phil_controls.nav_mode()
    phil_controls.mode(MAV_MODE.STABILIZE_ARMED)
}

fun onHIL_CONTROLS(phil_controls: com.company.demo.GroundControl.HIL_CONTROLS) {
    assert(phil_controls.time_usec())
    assert(phil_controls.roll_ailerons())
    assert(phil_controls.pitch_elevator())
    assert(phil_controls.yaw_rudder())
    assert(phil_controls.throttle())
    assert(phil_controls.aux1())
    assert(phil_controls.aux2())
    assert(phil_controls.aux3())
    assert(phil_controls.aux4())
    assert(phil_controls.nav_mode())
    assert(phil_controls.mode()!!.get() == MAV_MODE.STABILIZE_ARMED)
    println("HIL_CONTROLS \n")
}

fun fill(phil_sensor: com.company.demo.GroundControl.HIL_SENSOR) {

    phil_sensor.fields_updated()
    phil_sensor.time_usec()
    phil_sensor.xacc()
    phil_sensor.yacc()
    phil_sensor.zacc()
    phil_sensor.xgyro()
    phil_sensor.ygyro()
    phil_sensor.zgyro()
    phil_sensor.xmag()
    phil_sensor.ymag()
    phil_sensor.zmag()
    phil_sensor.abs_pressure()
    phil_sensor.diff_pressure()
    phil_sensor.pressure_alt()
    phil_sensor.temperature()
}

fun onHIL_SENSOR(phil_sensor: com.company.demo.GroundControl.HIL_SENSOR) {
    assert(phil_sensor.fields_updated())
    assert(phil_sensor.time_usec())
    assert(phil_sensor.xacc())
    assert(phil_sensor.yacc())
    assert(phil_sensor.zacc())
    assert(phil_sensor.xgyro())
    assert(phil_sensor.ygyro())
    assert(phil_sensor.zgyro())
    assert(phil_sensor.xmag())
    assert(phil_sensor.ymag())
    assert(phil_sensor.zmag())
    assert(phil_sensor.abs_pressure())
    assert(phil_sensor.diff_pressure())
    assert(phil_sensor.pressure_alt())
    assert(phil_sensor.temperature())
    println("HIL_SENSOR \n")
}

fun fill(psetup_signing: com.company.demo.GroundControl.SETUP_SIGNING) {

    psetup_signing.initial_timestamp()
    psetup_signing.target_system()
    psetup_signing.target_component()
    psetup_signing.secret_key(byteArrayOf(6, -18, 15, -82, -61, -97, 91, -60, 61, -28, 74, 66, 89, 103, 90, 77, -109, -111, -31, -101, 80, 1, -12, -83, -109, -116, -18, -124, -2, 95, -95, 12))

}

fun onSETUP_SIGNING(psetup_signing: com.company.demo.GroundControl.SETUP_SIGNING) {
    assert(psetup_signing.initial_timestamp())
    assert(psetup_signing.target_system())
    assert(psetup_signing.target_component())
    assert(psetup_signing.secret_key().same(byteArrayOf(6, -18, 15, -82, -61, -97, 91, -60, 61, -28, 74, 66, 89, 103, 90, 77, -109, -111, -31, -101, 80, 1, -12, -83, -109, -116, -18, -124, -2, 95, -95, 12)))

    println("SETUP_SIGNING \n")
}

fun fill(pgps_rtk: com.company.demo.GroundControl.GPS_RTK) {

    pgps_rtk.wn()
    pgps_rtk.time_last_baseline_ms()
    pgps_rtk.tow()
    pgps_rtk.accuracy()
    pgps_rtk.rtk_receiver_id()
    pgps_rtk.rtk_health()
    pgps_rtk.rtk_rate()
    pgps_rtk.nsats()
    pgps_rtk.baseline_coords_type()
    pgps_rtk.baseline_a_mm()
    pgps_rtk.baseline_b_mm()
    pgps_rtk.baseline_c_mm()
    pgps_rtk.iar_num_hypotheses()
}

fun onGPS_RTK(pgps_rtk: com.company.demo.GroundControl.GPS_RTK) {
    assert(pgps_rtk.wn())
    assert(pgps_rtk.time_last_baseline_ms())
    assert(pgps_rtk.tow())
    assert(pgps_rtk.accuracy())
    assert(pgps_rtk.rtk_receiver_id())
    assert(pgps_rtk.rtk_health())
    assert(pgps_rtk.rtk_rate())
    assert(pgps_rtk.nsats())
    assert(pgps_rtk.baseline_coords_type())
    assert(pgps_rtk.baseline_a_mm())
    assert(pgps_rtk.baseline_b_mm())
    assert(pgps_rtk.baseline_c_mm())
    assert(pgps_rtk.iar_num_hypotheses())
    println("GPS_RTK \n")
}

fun fill(pparam_request_list: test_.PARAM_REQUEST_LIST) {

    pparam_request_list.target_system()
    pparam_request_list.target_component()
}

fun onPARAM_REQUEST_LIST(pparam_request_list: com.company.demo.GroundControl.PARAM_REQUEST_LIST) {
    assert(pparam_request_list.target_system())
    assert(pparam_request_list.target_component())
    println("PARAM_REQUEST_LIST \n")
}

fun fill(puavionix_adsb_out_cfg: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG) {

    puavionix_adsb_out_cfg.stallSpeed()
    puavionix_adsb_out_cfg.ICAO()
    puavionix_adsb_out_cfg.callsign("qihyoicwhsBtnUwzsgothohRkcybhmsiryovufqFwtvtyhmhvowQax", null)

    puavionix_adsb_out_cfg.emitterType(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR)
    puavionix_adsb_out_cfg.aircraftSize(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M)
    puavionix_adsb_out_cfg.gpsOffsetLat(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.LEFT_2M)
    puavionix_adsb_out_cfg.gpsOffsetLon(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA)
    puavionix_adsb_out_cfg.rfSelect(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED)
}

fun onUAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG) {
    assert(puavionix_adsb_out_cfg.stallSpeed())
    assert(puavionix_adsb_out_cfg.ICAO())

    puavionix_adsb_out_cfg.callsign()?.let { item ->
        assert(item.get() == "qihyoicwhsBtnUwzsgothohRkcybhmsiryovufqFwtvtyhmhvowQax")
    } ?: throw RuntimeException("null")

    assert(puavionix_adsb_out_cfg.emitterType()!!.get() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR)
    assert(puavionix_adsb_out_cfg.aircraftSize()!!.get() == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M)
    assert(puavionix_adsb_out_cfg.gpsOffsetLat()!!.get() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.LEFT_2M)
    assert(puavionix_adsb_out_cfg.gpsOffsetLon()!!.get() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA)
    assert(puavionix_adsb_out_cfg.rfSelect()!!.get() == UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED)
    println("UAVIONIX_ADSB_OUT_CFG \n")
}

fun fill(planding_target: com.company.demo.GroundControl.LANDING_TARGET) {

    planding_target.time_usec()
    planding_target.target_num()
    planding_target.angle_x()
    planding_target.angle_y()
    planding_target.distance()
    planding_target.size_x()
    planding_target.size_y()
    planding_target.frame(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT)
    planding_target.x()
    planding_target.y()
    planding_target.z()
    planding_target.q(0)
    planding_target.q(1)
    planding_target.q(2)
    planding_target.q(3)
    planding_target.typE(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON)
    planding_target.position_valid()
}

fun onLANDING_TARGET(planding_target: com.company.demo.GroundControl.LANDING_TARGET) {
    assert(planding_target.time_usec())
    assert(planding_target.target_num())
    assert(planding_target.angle_x())
    assert(planding_target.angle_y())
    assert(planding_target.distance())
    assert(planding_target.size_x())
    assert(planding_target.size_y())
    assert(planding_target.frame()!!.get() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT)
    assert(planding_target.x())
    assert(planding_target.y())
    assert(planding_target.z())

    planding_target.q()?.let { fld ->

        assert(fld.get(0))
        assert(fld.get(1))
        assert(fld.get(2))
        assert(fld.get(3))

    } ?: throw RuntimeException("null")

    assert(planding_target.typE()!!.get() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON)
    assert(planding_target.position_valid())
    println("LANDING_TARGET \n")
}

fun fill(pset_actuator_control_target: com.company.demo.GroundControl.SET_ACTUATOR_CONTROL_TARGET) {

    pset_actuator_control_target.time_usec()
    pset_actuator_control_target.group_mlx()
    pset_actuator_control_target.target_system()
    pset_actuator_control_target.target_component()
    pset_actuator_control_target.controls(floatArrayOf(-1.2651366E38F, 6.4881674E37F, -1.941013E38F, -8.873003E37F, -2.9596086E38F, -2.4263626E37F, 3.2715397E38F, -2.947491E38F))

}

fun onSET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target: com.company.demo.GroundControl.SET_ACTUATOR_CONTROL_TARGET) {
    assert(pset_actuator_control_target.time_usec())
    assert(pset_actuator_control_target.group_mlx())
    assert(pset_actuator_control_target.target_system())
    assert(pset_actuator_control_target.target_component())
    assert(pset_actuator_control_target.controls().same(floatArrayOf(-1.2651366E38F, 6.4881674E37F, -1.941013E38F, -8.873003E37F, -2.9596086E38F, -2.4263626E37F, 3.2715397E38F, -2.947491E38F)))

    println("SET_ACTUATOR_CONTROL_TARGET \n")
}

fun fill(pcontrol_system_state: com.company.demo.GroundControl.CONTROL_SYSTEM_STATE) {

    pcontrol_system_state.time_usec()
    pcontrol_system_state.x_acc()
    pcontrol_system_state.y_acc()
    pcontrol_system_state.z_acc()
    pcontrol_system_state.x_vel()
    pcontrol_system_state.y_vel()
    pcontrol_system_state.z_vel()
    pcontrol_system_state.x_pos()
    pcontrol_system_state.y_pos()
    pcontrol_system_state.z_pos()
    pcontrol_system_state.airspeed()
    pcontrol_system_state.vel_variance(floatArrayOf(-2.2679945E38F, -8.976753E37F, 2.585882E38F))

    pcontrol_system_state.pos_variance(floatArrayOf(7.2976713E37F, -2.3225077E38F, -3.0965027E38F))

    pcontrol_system_state.q(floatArrayOf(1.6712078E38F, 2.0913584E38F, -3.012227E38F, 2.3335383E38F))

    pcontrol_system_state.roll_rate()
    pcontrol_system_state.pitch_rate()
    pcontrol_system_state.yaw_rate()
}

fun onCONTROL_SYSTEM_STATE(pcontrol_system_state: com.company.demo.GroundControl.CONTROL_SYSTEM_STATE) {
    assert(pcontrol_system_state.time_usec())
    assert(pcontrol_system_state.x_acc())
    assert(pcontrol_system_state.y_acc())
    assert(pcontrol_system_state.z_acc())
    assert(pcontrol_system_state.x_vel())
    assert(pcontrol_system_state.y_vel())
    assert(pcontrol_system_state.z_vel())
    assert(pcontrol_system_state.x_pos())
    assert(pcontrol_system_state.y_pos())
    assert(pcontrol_system_state.z_pos())
    assert(pcontrol_system_state.airspeed())
    assert(pcontrol_system_state.vel_variance().same(floatArrayOf(-2.2679945E38F, -8.976753E37F, 2.585882E38F)))

    assert(pcontrol_system_state.pos_variance().same(floatArrayOf(7.2976713E37F, -2.3225077E38F, -3.0965027E38F)))

    assert(pcontrol_system_state.q().same(floatArrayOf(1.6712078E38F, 2.0913584E38F, -3.012227E38F, 2.3335383E38F)))

    assert(pcontrol_system_state.roll_rate())
    assert(pcontrol_system_state.pitch_rate())
    assert(pcontrol_system_state.yaw_rate())
    println("CONTROL_SYSTEM_STATE \n")
}

fun fill(pset_position_target_global_int: test_.SET_POSITION_TARGET_GLOBAL_INT) {

    pset_position_target_global_int.type_mask()
    pset_position_target_global_int.time_boot_ms()
    pset_position_target_global_int.target_system()
    pset_position_target_global_int.target_component()
    pset_position_target_global_int.lat_int()
    pset_position_target_global_int.lon_int()
    pset_position_target_global_int.alt()
    pset_position_target_global_int.vx()
    pset_position_target_global_int.vy()
    pset_position_target_global_int.vz()
    pset_position_target_global_int.afx()
    pset_position_target_global_int.afy()
    pset_position_target_global_int.afz()
    pset_position_target_global_int.yaw()
    pset_position_target_global_int.yaw_rate()
    pset_position_target_global_int.coordinate_frame(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
}

fun onSET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int: com.company.demo.GroundControl.SET_POSITION_TARGET_GLOBAL_INT) {
    assert(pset_position_target_global_int.type_mask())
    assert(pset_position_target_global_int.time_boot_ms())
    assert(pset_position_target_global_int.target_system())
    assert(pset_position_target_global_int.target_component())
    assert(pset_position_target_global_int.lat_int())
    assert(pset_position_target_global_int.lon_int())
    assert(pset_position_target_global_int.alt())
    assert(pset_position_target_global_int.vx())
    assert(pset_position_target_global_int.vy())
    assert(pset_position_target_global_int.vz())
    assert(pset_position_target_global_int.afx())
    assert(pset_position_target_global_int.afy())
    assert(pset_position_target_global_int.afz())
    assert(pset_position_target_global_int.yaw())
    assert(pset_position_target_global_int.yaw_rate())
    assert(pset_position_target_global_int.coordinate_frame()!!.get() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
    println("SET_POSITION_TARGET_GLOBAL_INT \n")
}

fun fill(pdata32: com.company.demo.GroundControl.DATA32) {

    pdata32.typE()
    pdata32.len()
    pdata32.daTa(byteArrayOf(-47, 2, 70, -40, 70, -112, 58, -6, -63, -78, 20, 82, 12, 14, 124, -37, 18, -93, 86, 77, 86, -91, -51, 72, -119, 94, -29, 89, 127, -111, 52, -119))

}

fun onDATA32(pdata32: com.company.demo.GroundControl.DATA32) {
    assert(pdata32.typE())
    assert(pdata32.len())
    assert(pdata32.daTa().same(byteArrayOf(-47, 2, 70, -40, 70, -112, 58, -6, -63, -78, 20, 82, 12, 14, 124, -37, 18, -93, 86, 77, 86, -91, -51, 72, -119, 94, -29, 89, 127, -111, 52, -119)))

    println("DATA32 \n")
}

fun fill(pping33: com.company.demo.GroundControl.PING33) {

    pping33.TTTT(0, 0, 0)
    pping33.TTTT(0, 0, 1)
    pping33.TTTT(0, 0, 2)
    pping33.TTTT(0, 1, 0)
    pping33.TTTT(0, 1, 1)
    pping33.TTTT(0, 1, 2)
    pping33.TTTT(1, 0, 0)
    pping33.TTTT(1, 0, 1)
    pping33.TTTT(1, 0, 2)
    pping33.TTTT(1, 1, 0)
    pping33.TTTT(1, 1, 1)
    pping33.TTTT(1, 1, 2)
    pping33.TTTT(2, 0, 0)
    pping33.TTTT(2, 0, 1)
    pping33.TTTT(2, 0, 2)
    pping33.TTTT(2, 1, 0)
    pping33.TTTT(2, 1, 1)
    pping33.TTTT(2, 1, 2)
    pping33.field()
    pping33.bit_field()
    pping33.field6(0, 0, 0)
    pping33.field6(0, 0, 1)
    pping33.field6(0, 0, 2)
    pping33.field6(0, 1, 0)
    pping33.field6(0, 1, 1)
    pping33.field6(0, 1, 2)
    pping33.field6(1, 0, 0)
    pping33.field6(1, 0, 1)
    pping33.field6(1, 0, 2)
    pping33.field6(1, 1, 0)
    pping33.field6(1, 1, 1)
    pping33.field6(1, 1, 2)
    pping33.field6(2, 0, 0)
    pping33.field6(2, 0, 1)
    pping33.field6(2, 0, 2)
    pping33.field6(2, 1, 0)
    pping33.field6(2, 1, 1)
    pping33.field6(2, 1, 2)
    pping33.testBOOL2()
    pping33.testBOOL3()
    pping33.testBOOL()
    pping33.seq()


    pping33.field1().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 0, 1)
        fld.set(0, 0, 2)
        fld.set(0, 1, 0)
        fld.set(0, 1, 1)
        fld.set(0, 1, 2)

    } ?: throw RuntimeException("Not empty")

    pping33.field12(0, 0, 0)
    pping33.field12(0, 0, 1)
    pping33.field12(0, 0, 2)
    pping33.field12(0, 1, 0)
    pping33.field12(0, 1, 1)
    pping33.field12(0, 1, 2)
    pping33.field12(1, 0, 0)
    pping33.field12(1, 0, 1)
    pping33.field12(1, 0, 2)
    pping33.field12(1, 1, 0)
    pping33.field12(1, 1, 1)
    pping33.field12(1, 1, 2)
    pping33.field12(2, 0, 0)
    pping33.field12(2, 0, 1)
    pping33.field12(2, 0, 2)
    pping33.field12(2, 1, 0)
    pping33.field12(2, 1, 1)
    pping33.field12(2, 1, 2)
    pping33.field13(0, 0, 0)
    pping33.field13(0, 0, 1)
    pping33.field13(0, 0, 2)
    pping33.field13(0, 1, 0)
    pping33.field13(0, 1, 1)
    pping33.field13(0, 1, 2)
    pping33.field13(1, 0, 0)
    pping33.field13(1, 0, 1)
    pping33.field13(1, 0, 2)
    pping33.field13(1, 1, 0)
    pping33.field13(1, 1, 1)
    pping33.field13(1, 1, 2)
    pping33.field13(2, 0, 0)
    pping33.field13(2, 0, 1)
    pping33.field13(2, 0, 2)
    pping33.field13(2, 1, 0)
    pping33.field13(2, 1, 1)
    pping33.field13(2, 1, 2)
    pping33.WWWWWWWW()
    pping33.bit_field2()
    pping33.Field_Bits(0, 0, 0)
    pping33.Field_Bits(0, 0, 1)
    pping33.Field_Bits(0, 0, 2)
    pping33.Field_Bits(0, 1, 0)
    pping33.Field_Bits(0, 1, 1)
    pping33.Field_Bits(0, 1, 2)
    pping33.Field_Bits(0, 2, 0)
    pping33.Field_Bits(0, 2, 1)
    pping33.Field_Bits(0, 2, 2)
    pping33.Field_Bits(1, 0, 0)
    pping33.Field_Bits(1, 0, 1)
    pping33.Field_Bits(1, 0, 2)
    pping33.Field_Bits(1, 1, 0)
    pping33.Field_Bits(1, 1, 1)
    pping33.Field_Bits(1, 1, 2)
    pping33.Field_Bits(1, 2, 0)
    pping33.Field_Bits(1, 2, 1)
    pping33.Field_Bits(1, 2, 2)
    pping33.Field_Bits(2, 0, 0)
    pping33.Field_Bits(2, 0, 1)
    pping33.Field_Bits(2, 0, 2)
    pping33.Field_Bits(2, 1, 0)
    pping33.Field_Bits(2, 1, 1)
    pping33.Field_Bits(2, 1, 2)
    pping33.Field_Bits(2, 2, 0)
    pping33.Field_Bits(2, 2, 1)
    pping33.Field_Bits(2, 2, 2)


    pping33.SparseFixAllBits().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 0, 1)
        fld.set(0, 0, 2)
        fld.set(0, 1, 0)
        fld.set(0, 1, 1)
        fld.set(0, 1, 2)
        fld.set(0, 2, 0)
        fld.set(0, 2, 1)
        fld.set(0, 2, 2)

    } ?: throw RuntimeException("Not empty")



    pping33.FixAllBits().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 0, 1)
        fld.set(0, 0, 2)
        fld.set(0, 1, 0)
        fld.set(0, 1, 1)
        fld.set(0, 1, 2)
        fld.set(0, 2, 0)
        fld.set(0, 2, 1)
        fld.set(0, 2, 2)

    } ?: throw RuntimeException("Not empty")



    pping33.VarAllBits().Initializer()?.let { initializer ->
        val fld = initializer.init(1, 1)!!

        fld.set(0, 0, 0)
        fld.set(0, 1, 0)
        fld.set(0, 2, 0)

    } ?: throw RuntimeException("Not empty")



    pping33.SparseVarAllBits().Initializer()?.let { initializer ->
        val fld = initializer.init(1, 1)!!

        fld.set(0, 0, 0)
        fld.set(0, 1, 0)
        fld.set(0, 2, 0)

    } ?: throw RuntimeException("Not empty")



    pping33.VarEachBits().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 0, 1)
        fld.set(0, 0, 2)
        fld.set(0, 1, 0)
        fld.set(0, 1, 1)
        fld.set(0, 1, 2)
        fld.set(0, 2, 0)
        fld.set(0, 2, 1)
        fld.set(0, 2, 2)

    } ?: throw RuntimeException("Not empty")



    pping33.SparsVarEachBits().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 0, 1)
        fld.set(0, 0, 2)
        fld.set(0, 1, 0)
        fld.set(0, 1, 1)
        fld.set(0, 1, 2)
        fld.set(0, 2, 0)
        fld.set(0, 2, 1)
        fld.set(0, 2, 2)

    } ?: throw RuntimeException("Not empty")

    pping33.testBOOLX()
    pping33.testBOOL2X()
    pping33.testBOOL3X()
    pping33.MMMMMM(MAV_MODE.STABILIZE_ARMED)


    pping33.field44().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 1, 0)
        fld.set(1, 0, 0)
        fld.set(1, 1, 0)
        fld.set(2, 0, 0)
        fld.set(2, 1, 0)

    } ?: throw RuntimeException("Not empty")



    pping33.field634().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 1, 0)
        fld.set(1, 0, 0)
        fld.set(1, 1, 0)
        fld.set(2, 0, 0)
        fld.set(2, 1, 0)

    } ?: throw RuntimeException("Not empty")



    pping33.field33344().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 1, 0)
        fld.set(1, 0, 0)
        fld.set(1, 1, 0)
        fld.set(2, 0, 0)
        fld.set(2, 1, 0)

    } ?: throw RuntimeException("Not empty")



    pping33.field333634().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 1, 0)
        fld.set(1, 0, 0)
        fld.set(1, 1, 0)
        fld.set(2, 0, 0)
        fld.set(2, 1, 0)

    } ?: throw RuntimeException("Not empty")

    pping33.field__(0, 0, 0)
    pping33.field__(0, 0, 1)
    pping33.field__(0, 0, 2)
    pping33.field__(0, 1, 0)
    pping33.field__(0, 1, 1)
    pping33.field__(0, 1, 2)
    pping33.field__(1, 0, 0)
    pping33.field__(1, 0, 1)
    pping33.field__(1, 0, 2)
    pping33.field__(1, 1, 0)
    pping33.field__(1, 1, 1)
    pping33.field__(1, 1, 2)
    pping33.field__(2, 0, 0)
    pping33.field__(2, 0, 1)
    pping33.field__(2, 0, 2)
    pping33.field__(2, 1, 0)
    pping33.field__(2, 1, 1)
    pping33.field__(2, 1, 2)


    pping33.field63().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(0, 0, 0)
        fld.set(0, 1, 0)
        fld.set(1, 0, 0)
        fld.set(1, 1, 0)
        fld.set(2, 0, 0)
        fld.set(2, 1, 0)

    } ?: throw RuntimeException("Not empty")

    pping33.uid2(0)
    pping33.uid2(1)
    pping33.uid2(2)
    pping33.uid2(3)
    pping33.uid2(4)
    pping33.uid2(5)
    pping33.uid2(6)
    pping33.uid2(7)
    pping33.uid2(8)
    pping33.uid2(9)
    pping33.uid2(10)
    pping33.uid2(11)
    pping33.uid2(12)
    pping33.uid2(13)
    pping33.uid2(14)
    pping33.uid2(15)
    pping33.uid2(16)
    pping33.uid2(17)
    pping33.field2(0, 0, 0)
    pping33.field2(0, 0, 1)
    pping33.field2(0, 0, 2)
    pping33.field2(0, 1, 0)
    pping33.field2(0, 1, 1)
    pping33.field2(0, 1, 2)
    pping33.field2(1, 0, 0)
    pping33.field2(1, 0, 1)
    pping33.field2(1, 0, 2)
    pping33.field2(1, 1, 0)
    pping33.field2(1, 1, 1)
    pping33.field2(1, 1, 2)
    pping33.field2(2, 0, 0)
    pping33.field2(2, 0, 1)
    pping33.field2(2, 0, 2)
    pping33.field2(2, 1, 0)
    pping33.field2(2, 1, 1)
    pping33.field2(2, 1, 2)
    pping33.field4(0, 0, 0)
    pping33.field4(0, 0, 1)
    pping33.field4(0, 0, 2)
    pping33.field4(0, 1, 0)
    pping33.field4(0, 1, 1)
    pping33.field4(0, 1, 2)
    pping33.field4(1, 0, 0)
    pping33.field4(1, 0, 1)
    pping33.field4(1, 0, 2)
    pping33.field4(1, 1, 0)
    pping33.field4(1, 1, 1)
    pping33.field4(1, 1, 2)
    pping33.field4(2, 0, 0)
    pping33.field4(2, 0, 1)
    pping33.field4(2, 0, 2)
    pping33.field4(2, 1, 0)
    pping33.field4(2, 1, 1)
    pping33.field4(2, 1, 2)
    pping33.stringtest1("kdlfbgYeuhmymnjpgfplqpkrxyqpkyogzjfeewreyqhoxp", null)



    pping33.stringtest2().Initializer()?.let { initializer ->
        val fld = initializer.init(1)!!

        fld.set(TestChannel.str(1, 255), 0, 0, 0, null)

        fld.set(TestChannel.str(2, 255), 0, 1, 0, null)

        fld.set(TestChannel.str(3, 255), 1, 0, 0, null)

        fld.set(TestChannel.str(4, 255), 1, 1, 0, null)

        fld.set(TestChannel.str(5, 255), 2, 0, 0, null)

        fld.set(TestChannel.str(6, 255), 2, 1, 0, null)


    } ?: throw RuntimeException("Not empty")

    pping33.stringtest3("rdilnj", null)

    pping33.stringtest4("udykjzbtyiQUqylTv", null)

}

fun onPING33(pping33: com.company.demo.GroundControl.PING33) {
    assert(pping33.TTTT(0, 0, 0))
    assert(pping33.TTTT(0, 0, 1))
    assert(pping33.TTTT(0, 0, 2))
    assert(pping33.TTTT(0, 1, 0))
    assert(pping33.TTTT(0, 1, 1))
    assert(pping33.TTTT(0, 1, 2))
    assert(pping33.TTTT(1, 0, 0))
    assert(pping33.TTTT(1, 0, 1))
    assert(pping33.TTTT(1, 0, 2))
    assert(pping33.TTTT(1, 1, 0))
    assert(pping33.TTTT(1, 1, 1))
    assert(pping33.TTTT(1, 1, 2))
    assert(pping33.TTTT(2, 0, 0))
    assert(pping33.TTTT(2, 0, 1))
    assert(pping33.TTTT(2, 0, 2))
    assert(pping33.TTTT(2, 1, 0))
    assert(pping33.TTTT(2, 1, 1))
    assert(pping33.TTTT(2, 1, 2))
    assert(pping33.field())
    assert(pping33.bit_field())
    assert(pping33.field6(0, 0, 0))
    assert(pping33.field6(0, 0, 1))
    assert(pping33.field6(0, 0, 2))
    assert(pping33.field6(0, 1, 0))
    assert(pping33.field6(0, 1, 1))
    assert(pping33.field6(0, 1, 2))
    assert(pping33.field6(1, 0, 0))
    assert(pping33.field6(1, 0, 1))
    assert(pping33.field6(1, 0, 2))
    assert(pping33.field6(1, 1, 0))
    assert(pping33.field6(1, 1, 1))
    assert(pping33.field6(1, 1, 2))
    assert(pping33.field6(2, 0, 0))
    assert(pping33.field6(2, 0, 1))
    assert(pping33.field6(2, 0, 2))
    assert(pping33.field6(2, 1, 0))
    assert(pping33.field6(2, 1, 1))
    assert(pping33.field6(2, 1, 2))
    assert(pping33.testBOOL2())
    assert(pping33.testBOOL3())
    assert(pping33.testBOOL())
    assert(pping33.seq())

    //-----------
    pping33.field1().Field()?.let { fld ->
        assert(fld.d0() == 1 && PING33.field1.d0_max == 7)
        assert(PING33.field1.d1 == 2)
        assert(PING33.field1.d2 == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))

    } ?: throw RuntimeException("null")


    pping33.field12()?.let { fld ->

        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 0, 1))
        assert(fld.get(1, 0, 2))
        assert(fld.get(1, 1, 0))
        assert(fld.get(1, 1, 1))
        assert(fld.get(1, 1, 2))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 0, 1))
        assert(fld.get(2, 0, 2))
        assert(fld.get(2, 1, 0))
        assert(fld.get(2, 1, 1))
        assert(fld.get(2, 1, 2))

    } ?: throw RuntimeException("null")


    pping33.field13()?.let { fld ->

        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 0, 1))
        assert(fld.get(1, 0, 2))
        assert(fld.get(1, 1, 0))
        assert(fld.get(1, 1, 1))
        assert(fld.get(1, 1, 2))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 0, 1))
        assert(fld.get(2, 0, 2))
        assert(fld.get(2, 1, 0))
        assert(fld.get(2, 1, 1))
        assert(fld.get(2, 1, 2))

    } ?: throw RuntimeException("null")

    assert(pping33.WWWWWWWW())
    assert(pping33.bit_field2())

    pping33.Field_Bits()?.let { fld ->

        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(0, 2, 0))
        assert(fld.get(0, 2, 1))
        assert(fld.get(0, 2, 2))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 0, 1))
        assert(fld.get(1, 0, 2))
        assert(fld.get(1, 1, 0))
        assert(fld.get(1, 1, 1))
        assert(fld.get(1, 1, 2))
        assert(fld.get(1, 2, 0))
        assert(fld.get(1, 2, 1))
        assert(fld.get(1, 2, 2))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 0, 1))
        assert(fld.get(2, 0, 2))
        assert(fld.get(2, 1, 0))
        assert(fld.get(2, 1, 1))
        assert(fld.get(2, 1, 2))
        assert(fld.get(2, 2, 0))
        assert(fld.get(2, 2, 1))
        assert(fld.get(2, 2, 2))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.SparseFixAllBits().Field()?.let { fld ->
        assert(fld.d0() == 1 && PING33.SparseFixAllBits.d0_max == 3)
        assert(PING33.SparseFixAllBits.d1 == 3)
        assert(PING33.SparseFixAllBits.d2 == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(0, 2, 0))
        assert(fld.get(0, 2, 1))
        assert(fld.get(0, 2, 2))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.FixAllBits().Field()?.let { fld ->
        assert(fld.d0() == 1 && PING33.FixAllBits.d0_max == 3)
        assert(PING33.FixAllBits.d1 == 3)
        assert(PING33.FixAllBits.d2 == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(0, 2, 0))
        assert(fld.get(0, 2, 1))
        assert(fld.get(0, 2, 2))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.VarAllBits().Field()?.let { fld ->
        assert(fld.d0() == 1 && PING33.VarAllBits.d0_max == 3)
        assert(PING33.VarAllBits.d1 == 3)
        assert(fld.d2() == 1 && PING33.VarAllBits.d2_max == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 2, 0))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.SparseVarAllBits().Field()?.let { fld ->
        assert(fld.d0() == 1 && PING33.SparseVarAllBits.d0_max == 3)
        assert(PING33.SparseVarAllBits.d1 == 3)
        assert(fld.d2() == 1 && PING33.SparseVarAllBits.d2_max == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 2, 0))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.VarEachBits().Field()?.let { fld ->
        assert(fld.d0() == 1 && PING33.VarEachBits.d0_max == 3)
        assert(PING33.VarEachBits.d1 == 3)
        assert(PING33.VarEachBits.d2 == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(0, 2, 0))
        assert(fld.get(0, 2, 1))
        assert(fld.get(0, 2, 2))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.SparsVarEachBits().Field()?.let { fld ->
        assert(fld.d0() == 1 && PING33.SparsVarEachBits.d0_max == 3)
        assert(PING33.SparsVarEachBits.d1 == 3)
        assert(PING33.SparsVarEachBits.d2 == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(0, 2, 0))
        assert(fld.get(0, 2, 1))
        assert(fld.get(0, 2, 2))

    } ?: throw RuntimeException("null")

    assert(pping33.testBOOLX())
    assert(pping33.testBOOL2X())
    assert(pping33.testBOOL3X())
    assert(pping33.MMMMMM()!!.get() == MAV_MODE.STABILIZE_ARMED)

    //-----------
    pping33.field44().Field()?.let { fld ->
        assert(PING33.field44.d0 == 3)
        assert(PING33.field44.d1 == 2)
        assert(fld.d2() == 1 && PING33.field44.d2_max == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 1, 0))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 1, 0))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 1, 0))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.field634().Field()?.let { fld ->
        assert(PING33.field634.d0 == 3)
        assert(PING33.field634.d1 == 2)
        assert(fld.d2() == 1 && PING33.field634.d2_max == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 1, 0))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 1, 0))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 1, 0))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.field33344().Field()?.let { fld ->
        assert(PING33.field33344.d0 == 3)
        assert(PING33.field33344.d1 == 2)
        assert(fld.d2() == 1 && PING33.field33344.d2_max == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 1, 0))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 1, 0))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 1, 0))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.field333634().Field()?.let { fld ->
        assert(PING33.field333634.d0 == 3)
        assert(PING33.field333634.d1 == 2)
        assert(fld.d2() == 1 && PING33.field333634.d2_max == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 1, 0))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 1, 0))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 1, 0))

    } ?: throw RuntimeException("null")


    pping33.field__()?.let { fld ->

        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 0, 1))
        assert(fld.get(1, 0, 2))
        assert(fld.get(1, 1, 0))
        assert(fld.get(1, 1, 1))
        assert(fld.get(1, 1, 2))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 0, 1))
        assert(fld.get(2, 0, 2))
        assert(fld.get(2, 1, 0))
        assert(fld.get(2, 1, 1))
        assert(fld.get(2, 1, 2))

    } ?: throw RuntimeException("null")


    //-----------
    pping33.field63().Field()?.let { fld ->
        assert(PING33.field63.d0 == 3)
        assert(PING33.field63.d1 == 2)
        assert(fld.d2() == 1 && PING33.field63.d2_max == 3)




        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 1, 0))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 1, 0))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 1, 0))

    } ?: throw RuntimeException("null")


    pping33.uid2()?.let { fld ->

        assert(fld.get(0))
        assert(fld.get(1))
        assert(fld.get(2))
        assert(fld.get(3))
        assert(fld.get(4))
        assert(fld.get(5))
        assert(fld.get(6))
        assert(fld.get(7))
        assert(fld.get(8))
        assert(fld.get(9))
        assert(fld.get(10))
        assert(fld.get(11))
        assert(fld.get(12))
        assert(fld.get(13))
        assert(fld.get(14))
        assert(fld.get(15))
        assert(fld.get(16))
        assert(fld.get(17))

    } ?: throw RuntimeException("null")


    pping33.field2()?.let { fld ->

        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 0, 1))
        assert(fld.get(1, 0, 2))
        assert(fld.get(1, 1, 0))
        assert(fld.get(1, 1, 1))
        assert(fld.get(1, 1, 2))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 0, 1))
        assert(fld.get(2, 0, 2))
        assert(fld.get(2, 1, 0))
        assert(fld.get(2, 1, 1))
        assert(fld.get(2, 1, 2))

    } ?: throw RuntimeException("null")


    pping33.field4()?.let { fld ->

        assert(fld.get(0, 0, 0))
        assert(fld.get(0, 0, 1))
        assert(fld.get(0, 0, 2))
        assert(fld.get(0, 1, 0))
        assert(fld.get(0, 1, 1))
        assert(fld.get(0, 1, 2))
        assert(fld.get(1, 0, 0))
        assert(fld.get(1, 0, 1))
        assert(fld.get(1, 0, 2))
        assert(fld.get(1, 1, 0))
        assert(fld.get(1, 1, 1))
        assert(fld.get(1, 1, 2))
        assert(fld.get(2, 0, 0))
        assert(fld.get(2, 0, 1))
        assert(fld.get(2, 0, 2))
        assert(fld.get(2, 1, 0))
        assert(fld.get(2, 1, 1))
        assert(fld.get(2, 1, 2))

    } ?: throw RuntimeException("null")


    pping33.stringtest1()?.let { item ->
        assert(item.get() == "kdlfbgYeuhmymnjpgfplqpkrxyqpkyogzjfeewreyqhoxp")
    } ?: throw RuntimeException("null")


    //-----------
    pping33.stringtest2().Field()?.let { fld ->
        assert(PING33.stringtest2.d0 == 3)
        assert(PING33.stringtest2.d1 == 2)
        assert(fld.d2() == 1 && PING33.stringtest2.d2_max == 3)




        assert(fld.get(0, 0, 0).get() == TestChannel.str(1, 255))

        assert(fld.get(0, 1, 0).get() == TestChannel.str(2, 255))

        assert(fld.get(1, 0, 0).get() == TestChannel.str(3, 255))

        assert(fld.get(1, 1, 0).get() == TestChannel.str(4, 255))

        assert(fld.get(2, 0, 0).get() == TestChannel.str(5, 255))

        assert(fld.get(2, 1, 0).get() == TestChannel.str(6, 255))


    } ?: throw RuntimeException("null")


    pping33.stringtest3()?.let { item ->
        assert(item.get() == "rdilnj")
    } ?: throw RuntimeException("null")


    pping33.stringtest4()?.let { item ->
        assert(item.get() == "udykjzbtyiQUqylTv")
    } ?: throw RuntimeException("null")

    println("PING33 \n")
}

fun fill(pvfr_hud: test_.VFR_HUD) {

    pvfr_hud.throttle()
    pvfr_hud.airspeed()
    pvfr_hud.groundspeed()
    pvfr_hud.heading()
    pvfr_hud.alt()
    pvfr_hud.climb()
}

fun onVFR_HUD(pvfr_hud: com.company.demo.GroundControl.VFR_HUD) {
    assert(pvfr_hud.throttle())
    assert(pvfr_hud.airspeed())
    assert(pvfr_hud.groundspeed())
    assert(pvfr_hud.heading())
    assert(pvfr_hud.alt())
    assert(pvfr_hud.climb())
    println("VFR_HUD \n")
}

fun fill(prally_point: com.company.demo.GroundControl.RALLY_POINT) {

    prally_point.land_dir()
    prally_point.target_system()
    prally_point.target_component()
    prally_point.idx()
    prally_point.count()
    prally_point.lat()
    prally_point.lng()
    prally_point.alt()
    prally_point.break_alt()
    prally_point.flags(RALLY_FLAGS.FAVORABLE_WIND)
}

fun onRALLY_POINT(prally_point: com.company.demo.GroundControl.RALLY_POINT) {
    assert(prally_point.land_dir())
    assert(prally_point.target_system())
    assert(prally_point.target_component())
    assert(prally_point.idx())
    assert(prally_point.count())
    assert(prally_point.lat())
    assert(prally_point.lng())
    assert(prally_point.alt())
    assert(prally_point.break_alt())
    assert(prally_point.flags()!!.get() == RALLY_FLAGS.FAVORABLE_WIND)
    println("RALLY_POINT \n")
}

fun fill(pmission_set_current: test_.MISSION_SET_CURRENT) {

    pmission_set_current.seq()
    pmission_set_current.target_system()
    pmission_set_current.target_component()
}

fun onMISSION_SET_CURRENT(pmission_set_current: com.company.demo.GroundControl.MISSION_SET_CURRENT) {
    assert(pmission_set_current.seq())
    assert(pmission_set_current.target_system())
    assert(pmission_set_current.target_component())
    println("MISSION_SET_CURRENT \n")
}

fun fill(padap_tuning: com.company.demo.GroundControl.ADAP_TUNING) {

    padap_tuning.desired()
    padap_tuning.achieved()
    padap_tuning.error()
    padap_tuning.theta()
    padap_tuning.omega()
    padap_tuning.sigma()
    padap_tuning.theta_dot()
    padap_tuning.omega_dot()
    padap_tuning.sigma_dot()
    padap_tuning.f()
    padap_tuning.f_dot()
    padap_tuning.u()
    padap_tuning.axis(PID_TUNING_AXIS.PID_TUNING_YAW)
}

fun onADAP_TUNING(padap_tuning: com.company.demo.GroundControl.ADAP_TUNING) {
    assert(padap_tuning.desired())
    assert(padap_tuning.achieved())
    assert(padap_tuning.error())
    assert(padap_tuning.theta())
    assert(padap_tuning.omega())
    assert(padap_tuning.sigma())
    assert(padap_tuning.theta_dot())
    assert(padap_tuning.omega_dot())
    assert(padap_tuning.sigma_dot())
    assert(padap_tuning.f())
    assert(padap_tuning.f_dot())
    assert(padap_tuning.u())
    assert(padap_tuning.axis()!!.get() == PID_TUNING_AXIS.PID_TUNING_YAW)
    println("ADAP_TUNING \n")
}

fun fill(pvibration: com.company.demo.GroundControl.VIBRATION) {

    pvibration.clipping_0()
    pvibration.clipping_1()
    pvibration.clipping_2()
    pvibration.time_usec()
    pvibration.vibration_x()
    pvibration.vibration_y()
    pvibration.vibration_z()
}

fun onVIBRATION(pvibration: com.company.demo.GroundControl.VIBRATION) {
    assert(pvibration.clipping_0())
    assert(pvibration.clipping_1())
    assert(pvibration.clipping_2())
    assert(pvibration.time_usec())
    assert(pvibration.vibration_x())
    assert(pvibration.vibration_y())
    assert(pvibration.vibration_z())
    println("VIBRATION \n")
}

fun fill(pparam_ext_value: com.company.demo.GroundControl.PARAM_EXT_VALUE) {

    pparam_ext_value.param_count()
    pparam_ext_value.param_index()
    pparam_ext_value.param_id("ouTapuwdbrTfoeghMB", null)

    pparam_ext_value.param_value("ahWUvxAXLTkop", null)

    pparam_ext_value.param_type(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM)
}

fun onPARAM_EXT_VALUE(pparam_ext_value: com.company.demo.GroundControl.PARAM_EXT_VALUE) {
    assert(pparam_ext_value.param_count())
    assert(pparam_ext_value.param_index())

    pparam_ext_value.param_id()?.let { item ->
        assert(item.get() == "ouTapuwdbrTfoeghMB")
    } ?: throw RuntimeException("null")


    pparam_ext_value.param_value()?.let { item ->
        assert(item.get() == "ahWUvxAXLTkop")
    } ?: throw RuntimeException("null")

    assert(pparam_ext_value.param_type()!!.get() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM)
    println("PARAM_EXT_VALUE \n")
}

fun fill(pbattery2: com.company.demo.GroundControl.BATTERY2) {

    pbattery2.voltage()
    pbattery2.current_battery()
}

fun onBATTERY2(pbattery2: com.company.demo.GroundControl.BATTERY2) {
    assert(pbattery2.voltage())
    assert(pbattery2.current_battery())
    println("BATTERY2 \n")
}

fun fill(plimits_status: com.company.demo.GroundControl.LIMITS_STATUS) {

    plimits_status.breach_count()
    plimits_status.last_trigger()
    plimits_status.last_action()
    plimits_status.last_recovery()
    plimits_status.last_clear()
    plimits_status.limits_state(LIMITS_STATE.LIMITS_INIT)
    plimits_status.mods_enabled(LIMIT_MODULE.LIMIT_GEOFENCE)
    plimits_status.mods_required(LIMIT_MODULE.LIMIT_GPSLOCK)
    plimits_status.mods_triggered(LIMIT_MODULE.LIMIT_GEOFENCE)
}

fun onLIMITS_STATUS(plimits_status: com.company.demo.GroundControl.LIMITS_STATUS) {
    assert(plimits_status.breach_count())
    assert(plimits_status.last_trigger())
    assert(plimits_status.last_action())
    assert(plimits_status.last_recovery())
    assert(plimits_status.last_clear())
    assert(plimits_status.limits_state()!!.get() == LIMITS_STATE.LIMITS_INIT)
    assert(plimits_status.mods_enabled()!!.get() == LIMIT_MODULE.LIMIT_GEOFENCE)
    assert(plimits_status.mods_required()!!.get() == LIMIT_MODULE.LIMIT_GPSLOCK)
    assert(plimits_status.mods_triggered()!!.get() == LIMIT_MODULE.LIMIT_GEOFENCE)
    println("LIMITS_STATUS \n")
}

fun fill(pcamera_feedback: com.company.demo.GroundControl.CAMERA_FEEDBACK) {

    pcamera_feedback.img_idx()
    pcamera_feedback.time_usec()
    pcamera_feedback.target_system()
    pcamera_feedback.cam_idx()
    pcamera_feedback.lat()
    pcamera_feedback.lng()
    pcamera_feedback.alt_msl()
    pcamera_feedback.alt_rel()
    pcamera_feedback.roll()
    pcamera_feedback.pitch()
    pcamera_feedback.yaw()
    pcamera_feedback.foc_len()
    pcamera_feedback.flags(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_OPENLOOP)
}

fun onCAMERA_FEEDBACK(pcamera_feedback: com.company.demo.GroundControl.CAMERA_FEEDBACK) {
    assert(pcamera_feedback.img_idx())
    assert(pcamera_feedback.time_usec())
    assert(pcamera_feedback.target_system())
    assert(pcamera_feedback.cam_idx())
    assert(pcamera_feedback.lat())
    assert(pcamera_feedback.lng())
    assert(pcamera_feedback.alt_msl())
    assert(pcamera_feedback.alt_rel())
    assert(pcamera_feedback.roll())
    assert(pcamera_feedback.pitch())
    assert(pcamera_feedback.yaw())
    assert(pcamera_feedback.foc_len())
    assert(pcamera_feedback.flags()!!.get() == CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_OPENLOOP)
    println("CAMERA_FEEDBACK \n")
}

fun fill(phil_gps: com.company.demo.GroundControl.HIL_GPS) {

    phil_gps.eph()
    phil_gps.epv()
    phil_gps.vel()
    phil_gps.cog()
    phil_gps.time_usec()
    phil_gps.fix_type()
    phil_gps.lat()
    phil_gps.lon()
    phil_gps.alt()
    phil_gps.vn()
    phil_gps.ve()
    phil_gps.vd()
    phil_gps.satellites_visible()
}

fun onHIL_GPS(phil_gps: com.company.demo.GroundControl.HIL_GPS) {
    assert(phil_gps.eph())
    assert(phil_gps.epv())
    assert(phil_gps.vel())
    assert(phil_gps.cog())
    assert(phil_gps.time_usec())
    assert(phil_gps.fix_type())
    assert(phil_gps.lat())
    assert(phil_gps.lon())
    assert(phil_gps.alt())
    assert(phil_gps.vn())
    assert(phil_gps.ve())
    assert(phil_gps.vd())
    assert(phil_gps.satellites_visible())
    println("HIL_GPS \n")
}

fun fill(pnav_controller_output: test_.NAV_CONTROLLER_OUTPUT) {

    pnav_controller_output.wp_dist()
    pnav_controller_output.nav_roll()
    pnav_controller_output.nav_pitch()
    pnav_controller_output.nav_bearing()
    pnav_controller_output.target_bearing()
    pnav_controller_output.alt_error()
    pnav_controller_output.aspd_error()
    pnav_controller_output.xtrack_error()
}

fun onNAV_CONTROLLER_OUTPUT(pnav_controller_output: com.company.demo.GroundControl.NAV_CONTROLLER_OUTPUT) {
    assert(pnav_controller_output.wp_dist())
    assert(pnav_controller_output.nav_roll())
    assert(pnav_controller_output.nav_pitch())
    assert(pnav_controller_output.nav_bearing())
    assert(pnav_controller_output.target_bearing())
    assert(pnav_controller_output.alt_error())
    assert(pnav_controller_output.aspd_error())
    assert(pnav_controller_output.xtrack_error())
    println("NAV_CONTROLLER_OUTPUT \n")
}

fun fill(pauth_key: test_.AUTH_KEY) {

    pauth_key.key("tllsgfvqLkujyeclnliGpxxoufhujkutuchdhricbfogborzhymsdapdutvdnzb", null)

}

fun onAUTH_KEY(pauth_key: com.company.demo.GroundControl.AUTH_KEY) {

    pauth_key.key()?.let { item ->
        assert(item.get() == "tllsgfvqLkujyeclnliGpxxoufhujkutuchdhricbfogborzhymsdapdutvdnzb")
    } ?: throw RuntimeException("null")

    println("AUTH_KEY \n")
}

fun fill(pfence_fetch_point: com.company.demo.GroundControl.FENCE_FETCH_POINT) {

    pfence_fetch_point.target_system()
    pfence_fetch_point.target_component()
    pfence_fetch_point.idx()
}

fun onFENCE_FETCH_POINT(pfence_fetch_point: com.company.demo.GroundControl.FENCE_FETCH_POINT) {
    assert(pfence_fetch_point.target_system())
    assert(pfence_fetch_point.target_component())
    assert(pfence_fetch_point.idx())
    println("FENCE_FETCH_POINT \n")
}

fun fill(pradio: com.company.demo.GroundControl.RADIO) {

    pradio.rxerrors()
    pradio.fixeD()
    pradio.rssi()
    pradio.remrssi()
    pradio.txbuf()
    pradio.noise()
    pradio.remnoise()
}

fun onRADIO(pradio: com.company.demo.GroundControl.RADIO) {
    assert(pradio.rxerrors())
    assert(pradio.fixeD())
    assert(pradio.rssi())
    assert(pradio.remrssi())
    assert(pradio.txbuf())
    assert(pradio.noise())
    assert(pradio.remnoise())
    println("RADIO \n")
}

fun fill(plocal_position_ned_cov: test_.LOCAL_POSITION_NED_COV) {

    plocal_position_ned_cov.time_usec()
    plocal_position_ned_cov.x()
    plocal_position_ned_cov.y()
    plocal_position_ned_cov.z()
    plocal_position_ned_cov.vx()
    plocal_position_ned_cov.vy()
    plocal_position_ned_cov.vz()
    plocal_position_ned_cov.ax()
    plocal_position_ned_cov.ay()
    plocal_position_ned_cov.az()
    plocal_position_ned_cov.covariance(floatArrayOf(-2.6389543E38F, 9.409431E37F, 2.3151675E38F, 2.407193E38F, 2.3786735E38F, -2.9476858E38F, -1.0886159E38F, 2.454074E38F, 3.2019175E38F, 2.17147E38F, -1.729781E38F, 2.730344E38F, -1.6480384E38F, -8.139122E37F, -5.167862E37F, 2.9688278E38F, -4.1602868E37F, 3.211507E38F, 2.7094074E38F, 2.9214101E38F, -2.6120963E38F, -5.1898995E37F, 1.6706592E38F, -3.1668074E38F, -1.1400669E38F, -3.194777E37F, 7.5575093E37F, -1.8825392E38F, 4.8044074E37F, -1.96573E38F, 2.804623E38F, 2.622501E38F, -1.0063737E38F, 7.1416595E37F, -9.908546E37F, -1.3026518E38F, -5.812375E36F, -5.728466E37F, 2.7255072E38F, 1.0798151E38F, -3.547255E37F, 3.213423E38F, -3.393499E38F, -2.1192858E37F, 9.756967E37F))

    plocal_position_ned_cov.estimator_type(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS)
}

fun onLOCAL_POSITION_NED_COV(plocal_position_ned_cov: com.company.demo.GroundControl.LOCAL_POSITION_NED_COV) {
    assert(plocal_position_ned_cov.time_usec())
    assert(plocal_position_ned_cov.x())
    assert(plocal_position_ned_cov.y())
    assert(plocal_position_ned_cov.z())
    assert(plocal_position_ned_cov.vx())
    assert(plocal_position_ned_cov.vy())
    assert(plocal_position_ned_cov.vz())
    assert(plocal_position_ned_cov.ax())
    assert(plocal_position_ned_cov.ay())
    assert(plocal_position_ned_cov.az())
    assert(plocal_position_ned_cov.covariance().same(floatArrayOf(-2.6389543E38F, 9.409431E37F, 2.3151675E38F, 2.407193E38F, 2.3786735E38F, -2.9476858E38F, -1.0886159E38F, 2.454074E38F, 3.2019175E38F, 2.17147E38F, -1.729781E38F, 2.730344E38F, -1.6480384E38F, -8.139122E37F, -5.167862E37F, 2.9688278E38F, -4.1602868E37F, 3.211507E38F, 2.7094074E38F, 2.9214101E38F, -2.6120963E38F, -5.1898995E37F, 1.6706592E38F, -3.1668074E38F, -1.1400669E38F, -3.194777E37F, 7.5575093E37F, -1.8825392E38F, 4.8044074E37F, -1.96573E38F, 2.804623E38F, 2.622501E38F, -1.0063737E38F, 7.1416595E37F, -9.908546E37F, -1.3026518E38F, -5.812375E36F, -5.728466E37F, 2.7255072E38F, 1.0798151E38F, -3.547255E37F, 3.213423E38F, -3.393499E38F, -2.1192858E37F, 9.756967E37F)))

    assert(plocal_position_ned_cov.estimator_type()!!.get() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS)
    println("LOCAL_POSITION_NED_COV \n")
}

fun fill(pairspeed_autocal: com.company.demo.GroundControl.AIRSPEED_AUTOCAL) {

    pairspeed_autocal.vx()
    pairspeed_autocal.vy()
    pairspeed_autocal.vz()
    pairspeed_autocal.diff_pressure()
    pairspeed_autocal.EAS2TAS()
    pairspeed_autocal.ratio()
    pairspeed_autocal.state_x()
    pairspeed_autocal.state_y()
    pairspeed_autocal.state_z()
    pairspeed_autocal.Pax()
    pairspeed_autocal.Pby()
    pairspeed_autocal.Pcz()
}

fun onAIRSPEED_AUTOCAL(pairspeed_autocal: com.company.demo.GroundControl.AIRSPEED_AUTOCAL) {
    assert(pairspeed_autocal.vx())
    assert(pairspeed_autocal.vy())
    assert(pairspeed_autocal.vz())
    assert(pairspeed_autocal.diff_pressure())
    assert(pairspeed_autocal.EAS2TAS())
    assert(pairspeed_autocal.ratio())
    assert(pairspeed_autocal.state_x())
    assert(pairspeed_autocal.state_y())
    assert(pairspeed_autocal.state_z())
    assert(pairspeed_autocal.Pax())
    assert(pairspeed_autocal.Pby())
    assert(pairspeed_autocal.Pcz())
    println("AIRSPEED_AUTOCAL \n")
}

fun fill(patt_pos_mocap: com.company.demo.GroundControl.ATT_POS_MOCAP) {

    patt_pos_mocap.time_usec()
    patt_pos_mocap.q(floatArrayOf(-1.4900082E38F, -1.0353458E38F, 6.456262E37F, 1.4725862E37F))

    patt_pos_mocap.x()
    patt_pos_mocap.y()
    patt_pos_mocap.z()
}

fun onATT_POS_MOCAP(patt_pos_mocap: com.company.demo.GroundControl.ATT_POS_MOCAP) {
    assert(patt_pos_mocap.time_usec())
    assert(patt_pos_mocap.q().same(floatArrayOf(-1.4900082E38F, -1.0353458E38F, 6.456262E37F, 1.4725862E37F)))

    assert(patt_pos_mocap.x())
    assert(patt_pos_mocap.y())
    assert(patt_pos_mocap.z())
    println("ATT_POS_MOCAP \n")
}

fun fill(pstatustext: com.company.demo.GroundControl.STATUSTEXT) {

    pstatustext.severity(MAV_SEVERITY.MAV_SEVERITY_ERROR)
    pstatustext.text("cwhzfzxxrglv", null)

}

fun onSTATUSTEXT(pstatustext: com.company.demo.GroundControl.STATUSTEXT) {
    assert(pstatustext.severity()!!.get() == MAV_SEVERITY.MAV_SEVERITY_ERROR)

    pstatustext.text()?.let { item ->
        assert(item.get() == "cwhzfzxxrglv")
    } ?: throw RuntimeException("null")

    println("STATUSTEXT \n")
}

fun fill(pping: test_.PING) {

    pping.seq()
    pping.time_usec()
    pping.target_system()
    pping.target_component()
}

fun onPING(pping: com.company.demo.GroundControl.PING) {
    assert(pping.seq())
    assert(pping.time_usec())
    assert(pping.target_system())
    assert(pping.target_component())
    println("PING \n")
}

fun fill(pgopro_get_request: com.company.demo.GroundControl.GOPRO_GET_REQUEST) {

    pgopro_get_request.target_system()
    pgopro_get_request.target_component()
    pgopro_get_request.cmd_id(GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT)
}

fun onGOPRO_GET_REQUEST(pgopro_get_request: com.company.demo.GroundControl.GOPRO_GET_REQUEST) {
    assert(pgopro_get_request.target_system())
    assert(pgopro_get_request.target_component())
    assert(pgopro_get_request.cmd_id()!!.get() == GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT)
    println("GOPRO_GET_REQUEST \n")
}

fun fill(pcamera_capture_status: com.company.demo.GroundControl.CAMERA_CAPTURE_STATUS) {

    pcamera_capture_status.time_boot_ms()
    pcamera_capture_status.recording_time_ms()
    pcamera_capture_status.image_status()
    pcamera_capture_status.video_status()
    pcamera_capture_status.image_interval()
    pcamera_capture_status.available_capacity()
}

fun onCAMERA_CAPTURE_STATUS(pcamera_capture_status: com.company.demo.GroundControl.CAMERA_CAPTURE_STATUS) {
    assert(pcamera_capture_status.time_boot_ms())
    assert(pcamera_capture_status.recording_time_ms())
    assert(pcamera_capture_status.image_status())
    assert(pcamera_capture_status.video_status())
    assert(pcamera_capture_status.image_interval())
    assert(pcamera_capture_status.available_capacity())
    println("CAMERA_CAPTURE_STATUS \n")
}

fun fill(pglobal_position_int: test_.GLOBAL_POSITION_INT) {

    pglobal_position_int.hdg()
    pglobal_position_int.time_boot_ms()
    pglobal_position_int.lat()
    pglobal_position_int.lon()
    pglobal_position_int.alt()
    pglobal_position_int.relative_alt()
    pglobal_position_int.vx()
    pglobal_position_int.vy()
    pglobal_position_int.vz()
}

fun onGLOBAL_POSITION_INT(pglobal_position_int: com.company.demo.GroundControl.GLOBAL_POSITION_INT) {
    assert(pglobal_position_int.hdg())
    assert(pglobal_position_int.time_boot_ms())
    assert(pglobal_position_int.lat())
    assert(pglobal_position_int.lon())
    assert(pglobal_position_int.alt())
    assert(pglobal_position_int.relative_alt())
    assert(pglobal_position_int.vx())
    assert(pglobal_position_int.vy())
    assert(pglobal_position_int.vz())
    println("GLOBAL_POSITION_INT \n")
}

fun fill(pencapsulated_data: com.company.demo.GroundControl.ENCAPSULATED_DATA) {

    pencapsulated_data.seqnr();

    {
        var item = pencapsulated_data.daTa()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

}

fun onENCAPSULATED_DATA(pencapsulated_data: com.company.demo.GroundControl.ENCAPSULATED_DATA) {
    assert(pencapsulated_data.seqnr());
    {
        var item = pencapsulated_data.daTa()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    println("ENCAPSULATED_DATA \n")
}

fun fill(pgps_input: com.company.demo.GroundControl.GPS_INPUT) {

    pgps_input.time_week()
    pgps_input.time_week_ms()
    pgps_input.time_usec()
    pgps_input.gps_id()
    pgps_input.fix_type()
    pgps_input.lat()
    pgps_input.lon()
    pgps_input.alt()
    pgps_input.hdop()
    pgps_input.vdop()
    pgps_input.vn()
    pgps_input.ve()
    pgps_input.vd()
    pgps_input.speed_accuracy()
    pgps_input.horiz_accuracy()
    pgps_input.vert_accuracy()
    pgps_input.satellites_visible()
    pgps_input.ignore_flags(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP)
}

fun onGPS_INPUT(pgps_input: com.company.demo.GroundControl.GPS_INPUT) {
    assert(pgps_input.time_week())
    assert(pgps_input.time_week_ms())
    assert(pgps_input.time_usec())
    assert(pgps_input.gps_id())
    assert(pgps_input.fix_type())
    assert(pgps_input.lat())
    assert(pgps_input.lon())
    assert(pgps_input.alt())
    assert(pgps_input.hdop())
    assert(pgps_input.vdop())
    assert(pgps_input.vn())
    assert(pgps_input.ve())
    assert(pgps_input.vd())
    assert(pgps_input.speed_accuracy())
    assert(pgps_input.horiz_accuracy())
    assert(pgps_input.vert_accuracy())
    assert(pgps_input.satellites_visible())
    assert(pgps_input.ignore_flags()!!.get() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP)
    println("GPS_INPUT \n")
}

fun fill(pcommand_long: test_.COMMAND_LONG) {

    pcommand_long.target_system()
    pcommand_long.target_component()
    pcommand_long.confirmation()
    pcommand_long.param1()
    pcommand_long.param2()
    pcommand_long.param3()
    pcommand_long.param4()
    pcommand_long.param5()
    pcommand_long.param6()
    pcommand_long.param7()
    pcommand_long.command(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL)
}

fun onCOMMAND_LONG(pcommand_long: com.company.demo.GroundControl.COMMAND_LONG) {
    assert(pcommand_long.target_system())
    assert(pcommand_long.target_component())
    assert(pcommand_long.confirmation())
    assert(pcommand_long.param1())
    assert(pcommand_long.param2())
    assert(pcommand_long.param3())
    assert(pcommand_long.param4())
    assert(pcommand_long.param5())
    assert(pcommand_long.param6())
    assert(pcommand_long.param7())
    assert(pcommand_long.command()!!.get() == MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL)
    println("COMMAND_LONG \n")
}

fun fill(pcompassmot_status: com.company.demo.GroundControl.COMPASSMOT_STATUS) {

    pcompassmot_status.throttle()
    pcompassmot_status.interference()
    pcompassmot_status.current()
    pcompassmot_status.CompensationX()
    pcompassmot_status.CompensationY()
    pcompassmot_status.CompensationZ()
}

fun onCOMPASSMOT_STATUS(pcompassmot_status: com.company.demo.GroundControl.COMPASSMOT_STATUS) {
    assert(pcompassmot_status.throttle())
    assert(pcompassmot_status.interference())
    assert(pcompassmot_status.current())
    assert(pcompassmot_status.CompensationX())
    assert(pcompassmot_status.CompensationY())
    assert(pcompassmot_status.CompensationZ())
    println("COMPASSMOT_STATUS \n")
}

fun fill(plog_request_data: com.company.demo.GroundControl.LOG_REQUEST_DATA) {

    plog_request_data.id()
    plog_request_data.ofs()
    plog_request_data.count()
    plog_request_data.target_system()
    plog_request_data.target_component()
}

fun onLOG_REQUEST_DATA(plog_request_data: com.company.demo.GroundControl.LOG_REQUEST_DATA) {
    assert(plog_request_data.id())
    assert(plog_request_data.ofs())
    assert(plog_request_data.count())
    assert(plog_request_data.target_system())
    assert(plog_request_data.target_component())
    println("LOG_REQUEST_DATA \n")
}

fun fill(pgps_raw_int: test_.GPS_RAW_INT) {

    pgps_raw_int.eph()
    pgps_raw_int.epv()
    pgps_raw_int.vel()
    pgps_raw_int.cog()
    pgps_raw_int.time_usec()
    pgps_raw_int.lat()
    pgps_raw_int.lon()
    pgps_raw_int.alt()
    pgps_raw_int.satellites_visible()
    pgps_raw_int.fix_type(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED)
    pgps_raw_int.alt_ellipsoid()
    pgps_raw_int.h_acc()
    pgps_raw_int.v_acc()
    pgps_raw_int.vel_acc()
    pgps_raw_int.hdg_acc()
}

fun onGPS_RAW_INT(pgps_raw_int: com.company.demo.GroundControl.GPS_RAW_INT) {
    assert(pgps_raw_int.eph())
    assert(pgps_raw_int.epv())
    assert(pgps_raw_int.vel())
    assert(pgps_raw_int.cog())
    assert(pgps_raw_int.time_usec())
    assert(pgps_raw_int.lat())
    assert(pgps_raw_int.lon())
    assert(pgps_raw_int.alt())
    assert(pgps_raw_int.satellites_visible())
    assert(pgps_raw_int.fix_type()!!.get() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED)
    assert(pgps_raw_int.alt_ellipsoid())
    assert(pgps_raw_int.h_acc())
    assert(pgps_raw_int.v_acc())
    assert(pgps_raw_int.vel_acc())
    assert(pgps_raw_int.hdg_acc())
    println("GPS_RAW_INT \n")
}

fun fill(pcamera_status: com.company.demo.GroundControl.CAMERA_STATUS) {

    pcamera_status.img_idx()
    pcamera_status.time_usec()
    pcamera_status.target_system()
    pcamera_status.cam_idx()
    pcamera_status.p1()
    pcamera_status.p2()
    pcamera_status.p3()
    pcamera_status.p4()
    pcamera_status.event_id(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTORE)
}

fun onCAMERA_STATUS(pcamera_status: com.company.demo.GroundControl.CAMERA_STATUS) {
    assert(pcamera_status.img_idx())
    assert(pcamera_status.time_usec())
    assert(pcamera_status.target_system())
    assert(pcamera_status.cam_idx())
    assert(pcamera_status.p1())
    assert(pcamera_status.p2())
    assert(pcamera_status.p3())
    assert(pcamera_status.p4())
    assert(pcamera_status.event_id()!!.get() == CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTORE)
    println("CAMERA_STATUS \n")
}

fun fill(prc_channels_scaled: test_.RC_CHANNELS_SCALED) {

    prc_channels_scaled.time_boot_ms()
    prc_channels_scaled.port()
    prc_channels_scaled.chan1_scaled()
    prc_channels_scaled.chan2_scaled()
    prc_channels_scaled.chan3_scaled()
    prc_channels_scaled.chan4_scaled()
    prc_channels_scaled.chan5_scaled()
    prc_channels_scaled.chan6_scaled()
    prc_channels_scaled.chan7_scaled()
    prc_channels_scaled.chan8_scaled()
    prc_channels_scaled.rssi()
}

fun onRC_CHANNELS_SCALED(prc_channels_scaled: com.company.demo.GroundControl.RC_CHANNELS_SCALED) {
    assert(prc_channels_scaled.time_boot_ms())
    assert(prc_channels_scaled.port())
    assert(prc_channels_scaled.chan1_scaled())
    assert(prc_channels_scaled.chan2_scaled())
    assert(prc_channels_scaled.chan3_scaled())
    assert(prc_channels_scaled.chan4_scaled())
    assert(prc_channels_scaled.chan5_scaled())
    assert(prc_channels_scaled.chan6_scaled())
    assert(prc_channels_scaled.chan7_scaled())
    assert(prc_channels_scaled.chan8_scaled())
    assert(prc_channels_scaled.rssi())
    println("RC_CHANNELS_SCALED \n")
}

fun fill(pcamera_settings: com.company.demo.GroundControl.CAMERA_SETTINGS) {

    pcamera_settings.time_boot_ms()
    pcamera_settings.mode_id(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY)
}

fun onCAMERA_SETTINGS(pcamera_settings: com.company.demo.GroundControl.CAMERA_SETTINGS) {
    assert(pcamera_settings.time_boot_ms())
    assert(pcamera_settings.mode_id()!!.get() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY)
    println("CAMERA_SETTINGS \n")
}

fun fill(pdevice_op_read_reply: com.company.demo.GroundControl.DEVICE_OP_READ_REPLY) {

    pdevice_op_read_reply.request_id()
    pdevice_op_read_reply.result()
    pdevice_op_read_reply.regstart()
    pdevice_op_read_reply.count();

    {
        var item = pdevice_op_read_reply.daTa()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

}

fun onDEVICE_OP_READ_REPLY(pdevice_op_read_reply: com.company.demo.GroundControl.DEVICE_OP_READ_REPLY) {
    assert(pdevice_op_read_reply.request_id())
    assert(pdevice_op_read_reply.result())
    assert(pdevice_op_read_reply.regstart())
    assert(pdevice_op_read_reply.count());
    {
        var item = pdevice_op_read_reply.daTa()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    println("DEVICE_OP_READ_REPLY \n")
}

fun fill(praw_pressure: test_.RAW_PRESSURE) {

    praw_pressure.time_usec()
    praw_pressure.press_abs()
    praw_pressure.press_diff1()
    praw_pressure.press_diff2()
    praw_pressure.temperature()
}

fun onRAW_PRESSURE(praw_pressure: com.company.demo.GroundControl.RAW_PRESSURE) {
    assert(praw_pressure.time_usec())
    assert(praw_pressure.press_abs())
    assert(praw_pressure.press_diff1())
    assert(praw_pressure.press_diff2())
    assert(praw_pressure.temperature())
    println("RAW_PRESSURE \n")
}

fun fill(pdigicam_control: com.company.demo.GroundControl.DIGICAM_CONTROL) {

    pdigicam_control.target_system()
    pdigicam_control.target_component()
    pdigicam_control.session()
    pdigicam_control.zoom_pos()
    pdigicam_control.zoom_step()
    pdigicam_control.focus_lock()
    pdigicam_control.shot()
    pdigicam_control.command_id()
    pdigicam_control.extra_param()
    pdigicam_control.extra_value()
}

fun onDIGICAM_CONTROL(pdigicam_control: com.company.demo.GroundControl.DIGICAM_CONTROL) {
    assert(pdigicam_control.target_system())
    assert(pdigicam_control.target_component())
    assert(pdigicam_control.session())
    assert(pdigicam_control.zoom_pos())
    assert(pdigicam_control.zoom_step())
    assert(pdigicam_control.focus_lock())
    assert(pdigicam_control.shot())
    assert(pdigicam_control.command_id())
    assert(pdigicam_control.extra_param())
    assert(pdigicam_control.extra_value())
    println("DIGICAM_CONTROL \n")
}

fun fill(pnamed_value_float: com.company.demo.GroundControl.NAMED_VALUE_FLOAT) {

    pnamed_value_float.time_boot_ms()
    pnamed_value_float.value()
    pnamed_value_float.name("jewyM", null)

}

fun onNAMED_VALUE_FLOAT(pnamed_value_float: com.company.demo.GroundControl.NAMED_VALUE_FLOAT) {
    assert(pnamed_value_float.time_boot_ms())
    assert(pnamed_value_float.value())

    pnamed_value_float.name()?.let { item ->
        assert(item.get() == "jewyM")
    } ?: throw RuntimeException("null")

    println("NAMED_VALUE_FLOAT \n")
}

fun fill(pgopro_heartbeat: com.company.demo.GroundControl.GOPRO_HEARTBEAT) {

    pgopro_heartbeat.status(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED)
    pgopro_heartbeat.capture_mode(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST)
    pgopro_heartbeat.flags(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING)
}

fun onGOPRO_HEARTBEAT(pgopro_heartbeat: com.company.demo.GroundControl.GOPRO_HEARTBEAT) {
    assert(pgopro_heartbeat.status()!!.get() == GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_CONNECTED)
    assert(pgopro_heartbeat.capture_mode()!!.get() == GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST)
    assert(pgopro_heartbeat.flags()!!.get() == GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING)
    println("GOPRO_HEARTBEAT \n")
}

fun fill(pattitude: test_.ATTITUDE) {

    pattitude.time_boot_ms()
    pattitude.roll()
    pattitude.pitch()
    pattitude.yaw()
    pattitude.rollspeed()
    pattitude.pitchspeed()
    pattitude.yawspeed()
}

fun onATTITUDE(pattitude: com.company.demo.GroundControl.ATTITUDE) {
    assert(pattitude.time_boot_ms())
    assert(pattitude.roll())
    assert(pattitude.pitch())
    assert(pattitude.yaw())
    assert(pattitude.rollspeed())
    assert(pattitude.pitchspeed())
    assert(pattitude.yawspeed())
    println("ATTITUDE \n")
}

fun fill(pmission_write_partial_list: test_.MISSION_WRITE_PARTIAL_LIST) {

    pmission_write_partial_list.target_system()
    pmission_write_partial_list.target_component()
    pmission_write_partial_list.start_index()
    pmission_write_partial_list.end_index()
    pmission_write_partial_list.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL)
}

fun onMISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list: com.company.demo.GroundControl.MISSION_WRITE_PARTIAL_LIST) {
    assert(pmission_write_partial_list.target_system())
    assert(pmission_write_partial_list.target_component())
    assert(pmission_write_partial_list.start_index())
    assert(pmission_write_partial_list.end_index())
    assert(pmission_write_partial_list.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL)
    println("MISSION_WRITE_PARTIAL_LIST \n")
}

fun fill(pahrs2: com.company.demo.GroundControl.AHRS2) {

    pahrs2.roll()
    pahrs2.pitch()
    pahrs2.yaw()
    pahrs2.altitude()
    pahrs2.lat()
    pahrs2.lng()
}

fun onAHRS2(pahrs2: com.company.demo.GroundControl.AHRS2) {
    assert(pahrs2.roll())
    assert(pahrs2.pitch())
    assert(pahrs2.yaw())
    assert(pahrs2.altitude())
    assert(pahrs2.lat())
    assert(pahrs2.lng())
    println("AHRS2 \n")
}

fun fill(plog_erase: com.company.demo.GroundControl.LOG_ERASE) {

    plog_erase.target_system()
    plog_erase.target_component()
}

fun onLOG_ERASE(plog_erase: com.company.demo.GroundControl.LOG_ERASE) {
    assert(plog_erase.target_system())
    assert(plog_erase.target_component())
    println("LOG_ERASE \n")
}

fun fill(pterrain_request: com.company.demo.GroundControl.TERRAIN_REQUEST) {

    pterrain_request.grid_spacing()
    pterrain_request.mask()
    pterrain_request.lat()
    pterrain_request.lon()
}

fun onTERRAIN_REQUEST(pterrain_request: com.company.demo.GroundControl.TERRAIN_REQUEST) {
    assert(pterrain_request.grid_spacing())
    assert(pterrain_request.mask())
    assert(pterrain_request.lat())
    assert(pterrain_request.lon())
    println("TERRAIN_REQUEST \n")
}

fun fill(pmount_status: com.company.demo.GroundControl.MOUNT_STATUS) {

    pmount_status.target_system()
    pmount_status.target_component()
    pmount_status.pointing_a()
    pmount_status.pointing_b()
    pmount_status.pointing_c()
}

fun onMOUNT_STATUS(pmount_status: com.company.demo.GroundControl.MOUNT_STATUS) {
    assert(pmount_status.target_system())
    assert(pmount_status.target_component())
    assert(pmount_status.pointing_a())
    assert(pmount_status.pointing_b())
    assert(pmount_status.pointing_c())
    println("MOUNT_STATUS \n")
}

fun fill(pmanual_setpoint: test_.MANUAL_SETPOINT) {

    pmanual_setpoint.time_boot_ms()
    pmanual_setpoint.roll()
    pmanual_setpoint.pitch()
    pmanual_setpoint.yaw()
    pmanual_setpoint.thrust()
    pmanual_setpoint.mode_switch()
    pmanual_setpoint.manual_override_switch()
}

fun onMANUAL_SETPOINT(pmanual_setpoint: com.company.demo.GroundControl.MANUAL_SETPOINT) {
    assert(pmanual_setpoint.time_boot_ms())
    assert(pmanual_setpoint.roll())
    assert(pmanual_setpoint.pitch())
    assert(pmanual_setpoint.yaw())
    assert(pmanual_setpoint.thrust())
    assert(pmanual_setpoint.mode_switch())
    assert(pmanual_setpoint.manual_override_switch())
    println("MANUAL_SETPOINT \n")
}

fun fill(ppid_tuning: com.company.demo.GroundControl.PID_TUNING) {

    ppid_tuning.desired()
    ppid_tuning.achieved()
    ppid_tuning.FF()
    ppid_tuning.P()
    ppid_tuning.I()
    ppid_tuning.D()
    ppid_tuning.axis(PID_TUNING_AXIS.PID_TUNING_ACCZ)
}

fun onPID_TUNING(ppid_tuning: com.company.demo.GroundControl.PID_TUNING) {
    assert(ppid_tuning.desired())
    assert(ppid_tuning.achieved())
    assert(ppid_tuning.FF())
    assert(ppid_tuning.P())
    assert(ppid_tuning.I())
    assert(ppid_tuning.D())
    assert(ppid_tuning.axis()!!.get() == PID_TUNING_AXIS.PID_TUNING_ACCZ)
    println("PID_TUNING \n")
}

fun fill(psafety_allowed_area: test_.SAFETY_ALLOWED_AREA) {

    psafety_allowed_area.p1x()
    psafety_allowed_area.p1y()
    psafety_allowed_area.p1z()
    psafety_allowed_area.p2x()
    psafety_allowed_area.p2y()
    psafety_allowed_area.p2z()
    psafety_allowed_area.frame(MAV_FRAME.MAV_FRAME_LOCAL_ENU)
}

fun onSAFETY_ALLOWED_AREA(psafety_allowed_area: com.company.demo.GroundControl.SAFETY_ALLOWED_AREA) {
    assert(psafety_allowed_area.p1x())
    assert(psafety_allowed_area.p1y())
    assert(psafety_allowed_area.p1z())
    assert(psafety_allowed_area.p2x())
    assert(psafety_allowed_area.p2y())
    assert(psafety_allowed_area.p2z())
    assert(psafety_allowed_area.frame()!!.get() == MAV_FRAME.MAV_FRAME_LOCAL_ENU)
    println("SAFETY_ALLOWED_AREA \n")
}

fun fill(poptical_flow_rad: com.company.demo.GroundControl.OPTICAL_FLOW_RAD) {

    poptical_flow_rad.integration_time_us()
    poptical_flow_rad.time_delta_distance_us()
    poptical_flow_rad.time_usec()
    poptical_flow_rad.sensor_id()
    poptical_flow_rad.integrated_x()
    poptical_flow_rad.integrated_y()
    poptical_flow_rad.integrated_xgyro()
    poptical_flow_rad.integrated_ygyro()
    poptical_flow_rad.integrated_zgyro()
    poptical_flow_rad.temperature()
    poptical_flow_rad.quality()
    poptical_flow_rad.distance()
}

fun onOPTICAL_FLOW_RAD(poptical_flow_rad: com.company.demo.GroundControl.OPTICAL_FLOW_RAD) {
    assert(poptical_flow_rad.integration_time_us())
    assert(poptical_flow_rad.time_delta_distance_us())
    assert(poptical_flow_rad.time_usec())
    assert(poptical_flow_rad.sensor_id())
    assert(poptical_flow_rad.integrated_x())
    assert(poptical_flow_rad.integrated_y())
    assert(poptical_flow_rad.integrated_xgyro())
    assert(poptical_flow_rad.integrated_ygyro())
    assert(poptical_flow_rad.integrated_zgyro())
    assert(poptical_flow_rad.temperature())
    assert(poptical_flow_rad.quality())
    assert(poptical_flow_rad.distance())
    println("OPTICAL_FLOW_RAD \n")
}

fun fill(plog_data: com.company.demo.GroundControl.LOG_DATA) {

    plog_data.id()
    plog_data.ofs()
    plog_data.count()
    plog_data.daTa(byteArrayOf(-29, 100, -10, 59, -116, 62, 6, 120, 111, 18, -71, 57, 57, -112, 100, -113, -46, 57, -82, 125, 113, 111, -45, 38, 41, 66, -61, 109, -5, 70, 30, 71, -115, 112, -66, -43, -66, -101, -51, -17, 1, -72, -6, 54, -90, -86, 98, 108, 122, -66, 109, 61, -23, -122, 101, 13, -125, 117, -128, -127, -56, 20, -115, 117, -115, -29, 3, 96, 21, 79, -44, -25, -111, -31, -123, -77, 62, -73, -8, 97, 15, 97, 4, 111, -100, 107, -20, -115, -24, -10))

}

fun onLOG_DATA(plog_data: com.company.demo.GroundControl.LOG_DATA) {
    assert(plog_data.id())
    assert(plog_data.ofs())
    assert(plog_data.count())
    assert(plog_data.daTa().same(byteArrayOf(-29, 100, -10, 59, -116, 62, 6, 120, 111, 18, -71, 57, 57, -112, 100, -113, -46, 57, -82, 125, 113, 111, -45, 38, 41, 66, -61, 109, -5, 70, 30, 71, -115, 112, -66, -43, -66, -101, -51, -17, 1, -72, -6, 54, -90, -86, 98, 108, 122, -66, 109, 61, -23, -122, 101, 13, -125, 117, -128, -127, -56, 20, -115, 117, -115, -29, 3, 96, 21, 79, -44, -25, -111, -31, -123, -77, 62, -73, -8, 97, 15, 97, 4, 111, -100, 107, -20, -115, -24, -10)))

    println("LOG_DATA \n")
}

fun fill(pmission_clear_all: test_.MISSION_CLEAR_ALL) {

    pmission_clear_all.target_system()
    pmission_clear_all.target_component()
    pmission_clear_all.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE)
}

fun onMISSION_CLEAR_ALL(pmission_clear_all: com.company.demo.GroundControl.MISSION_CLEAR_ALL) {
    assert(pmission_clear_all.target_system())
    assert(pmission_clear_all.target_component())
    assert(pmission_clear_all.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE)
    println("MISSION_CLEAR_ALL \n")
}

fun fill(pahrs3: com.company.demo.GroundControl.AHRS3) {

    pahrs3.roll()
    pahrs3.pitch()
    pahrs3.yaw()
    pahrs3.altitude()
    pahrs3.lat()
    pahrs3.lng()
    pahrs3.v1()
    pahrs3.v2()
    pahrs3.v3()
    pahrs3.v4()
}

fun onAHRS3(pahrs3: com.company.demo.GroundControl.AHRS3) {
    assert(pahrs3.roll())
    assert(pahrs3.pitch())
    assert(pahrs3.yaw())
    assert(pahrs3.altitude())
    assert(pahrs3.lat())
    assert(pahrs3.lng())
    assert(pahrs3.v1())
    assert(pahrs3.v2())
    assert(pahrs3.v3())
    assert(pahrs3.v4())
    println("AHRS3 \n")
}

fun fill(pvicon_position_estimate: com.company.demo.GroundControl.VICON_POSITION_ESTIMATE) {

    pvicon_position_estimate.usec()
    pvicon_position_estimate.x()
    pvicon_position_estimate.y()
    pvicon_position_estimate.z()
    pvicon_position_estimate.roll()
    pvicon_position_estimate.pitch()
    pvicon_position_estimate.yaw()
}

fun onVICON_POSITION_ESTIMATE(pvicon_position_estimate: com.company.demo.GroundControl.VICON_POSITION_ESTIMATE) {
    assert(pvicon_position_estimate.usec())
    assert(pvicon_position_estimate.x())
    assert(pvicon_position_estimate.y())
    assert(pvicon_position_estimate.z())
    assert(pvicon_position_estimate.roll())
    assert(pvicon_position_estimate.pitch())
    assert(pvicon_position_estimate.yaw())
    println("VICON_POSITION_ESTIMATE \n")
}

fun fill(pgps2_rtk: com.company.demo.GroundControl.GPS2_RTK) {

    pgps2_rtk.wn()
    pgps2_rtk.time_last_baseline_ms()
    pgps2_rtk.tow()
    pgps2_rtk.accuracy()
    pgps2_rtk.rtk_receiver_id()
    pgps2_rtk.rtk_health()
    pgps2_rtk.rtk_rate()
    pgps2_rtk.nsats()
    pgps2_rtk.baseline_coords_type()
    pgps2_rtk.baseline_a_mm()
    pgps2_rtk.baseline_b_mm()
    pgps2_rtk.baseline_c_mm()
    pgps2_rtk.iar_num_hypotheses()
}

fun onGPS2_RTK(pgps2_rtk: com.company.demo.GroundControl.GPS2_RTK) {
    assert(pgps2_rtk.wn())
    assert(pgps2_rtk.time_last_baseline_ms())
    assert(pgps2_rtk.tow())
    assert(pgps2_rtk.accuracy())
    assert(pgps2_rtk.rtk_receiver_id())
    assert(pgps2_rtk.rtk_health())
    assert(pgps2_rtk.rtk_rate())
    assert(pgps2_rtk.nsats())
    assert(pgps2_rtk.baseline_coords_type())
    assert(pgps2_rtk.baseline_a_mm())
    assert(pgps2_rtk.baseline_b_mm())
    assert(pgps2_rtk.baseline_c_mm())
    assert(pgps2_rtk.iar_num_hypotheses())
    println("GPS2_RTK \n")
}

fun fill(pmag_cal_report: com.company.demo.GroundControl.MAG_CAL_REPORT) {

    pmag_cal_report.compass_id()
    pmag_cal_report.cal_mask()
    pmag_cal_report.autosaved()
    pmag_cal_report.fitness()
    pmag_cal_report.ofs_x()
    pmag_cal_report.ofs_y()
    pmag_cal_report.ofs_z()
    pmag_cal_report.diag_x()
    pmag_cal_report.diag_y()
    pmag_cal_report.diag_z()
    pmag_cal_report.offdiag_x()
    pmag_cal_report.offdiag_y()
    pmag_cal_report.offdiag_z()
    pmag_cal_report.cal_status(MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE)
}

fun onMAG_CAL_REPORT(pmag_cal_report: com.company.demo.GroundControl.MAG_CAL_REPORT) {
    assert(pmag_cal_report.compass_id())
    assert(pmag_cal_report.cal_mask())
    assert(pmag_cal_report.autosaved())
    assert(pmag_cal_report.fitness())
    assert(pmag_cal_report.ofs_x())
    assert(pmag_cal_report.ofs_y())
    assert(pmag_cal_report.ofs_z())
    assert(pmag_cal_report.diag_x())
    assert(pmag_cal_report.diag_y())
    assert(pmag_cal_report.diag_z())
    assert(pmag_cal_report.offdiag_x())
    assert(pmag_cal_report.offdiag_y())
    assert(pmag_cal_report.offdiag_z())
    assert(pmag_cal_report.cal_status()!!.get() == MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE)
    println("MAG_CAL_REPORT \n")
}

fun fill(plog_request_list: com.company.demo.GroundControl.LOG_REQUEST_LIST) {

    plog_request_list.start()
    plog_request_list.end()
    plog_request_list.target_system()
    plog_request_list.target_component()
}

fun onLOG_REQUEST_LIST(plog_request_list: com.company.demo.GroundControl.LOG_REQUEST_LIST) {
    assert(plog_request_list.start())
    assert(plog_request_list.end())
    assert(plog_request_list.target_system())
    assert(plog_request_list.target_component())
    println("LOG_REQUEST_LIST \n")
}

fun fill(pscaled_pressure: test_.SCALED_PRESSURE) {

    pscaled_pressure.time_boot_ms()
    pscaled_pressure.press_abs()
    pscaled_pressure.press_diff()
    pscaled_pressure.temperature()
}

fun onSCALED_PRESSURE(pscaled_pressure: com.company.demo.GroundControl.SCALED_PRESSURE) {
    assert(pscaled_pressure.time_boot_ms())
    assert(pscaled_pressure.press_abs())
    assert(pscaled_pressure.press_diff())
    assert(pscaled_pressure.temperature())
    println("SCALED_PRESSURE \n")
}

fun fill(pv2_extension: com.company.demo.GroundControl.V2_EXTENSION) {

    pv2_extension.message_type()
    pv2_extension.target_network()
    pv2_extension.target_system()
    pv2_extension.target_component();

    {
        var item = pv2_extension.payload()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

}

fun onV2_EXTENSION(pv2_extension: com.company.demo.GroundControl.V2_EXTENSION) {
    assert(pv2_extension.message_type())
    assert(pv2_extension.target_network())
    assert(pv2_extension.target_system())
    assert(pv2_extension.target_component());
    {
        var item = pv2_extension.payload()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    println("V2_EXTENSION \n")
}

fun fill(pheartbeat: test_.HEARTBEAT) {

    pheartbeat.custom_mode()
    pheartbeat.mavlink_version()
    pheartbeat.typE(MAV_TYPE.VTOL_RESERVED3)
    pheartbeat.autopilot(MAV_AUTOPILOT.OPENPILOT)
    pheartbeat.base_mode(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)
    pheartbeat.system_status(MAV_STATE.BOOT)
}

fun onHEARTBEAT(pheartbeat: com.company.demo.GroundControl.HEARTBEAT) {
    assert(pheartbeat.custom_mode())
    assert(pheartbeat.mavlink_version())
    assert(pheartbeat.typE()!!.get() == MAV_TYPE.VTOL_RESERVED3)
    assert(pheartbeat.autopilot()!!.get() == MAV_AUTOPILOT.OPENPILOT)
    assert(pheartbeat.base_mode()!!.get() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)
    assert(pheartbeat.system_status()!!.get() == MAV_STATE.BOOT)
    println("HEARTBEAT \n")
}

fun fill(pparam_map_rc: test_.PARAM_MAP_RC) {

    pparam_map_rc.target_system()
    pparam_map_rc.target_component()
    pparam_map_rc.param_index()
    pparam_map_rc.parameter_rc_channel_index()
    pparam_map_rc.param_value0()
    pparam_map_rc.scale()
    pparam_map_rc.param_value_min()
    pparam_map_rc.param_value_max()
    pparam_map_rc.param_id("qaidkgqBox", null)

}

fun onPARAM_MAP_RC(pparam_map_rc: com.company.demo.GroundControl.PARAM_MAP_RC) {
    assert(pparam_map_rc.target_system())
    assert(pparam_map_rc.target_component())
    assert(pparam_map_rc.param_index())
    assert(pparam_map_rc.parameter_rc_channel_index())
    assert(pparam_map_rc.param_value0())
    assert(pparam_map_rc.scale())
    assert(pparam_map_rc.param_value_min())
    assert(pparam_map_rc.param_value_max())

    pparam_map_rc.param_id()?.let { item ->
        assert(item.get() == "qaidkgqBox")
    } ?: throw RuntimeException("null")

    println("PARAM_MAP_RC \n")
}

fun fill(ppower_status: com.company.demo.GroundControl.POWER_STATUS) {

    ppower_status.Vcc()
    ppower_status.Vservo()
    ppower_status.flags(MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED)
}

fun onPOWER_STATUS(ppower_status: com.company.demo.GroundControl.POWER_STATUS) {
    assert(ppower_status.Vcc())
    assert(ppower_status.Vservo())
    assert(ppower_status.flags()!!.get() == MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED)
    println("POWER_STATUS \n")
}

fun fill(premote_log_data_block: com.company.demo.GroundControl.REMOTE_LOG_DATA_BLOCK) {

    premote_log_data_block.target_system()
    premote_log_data_block.target_component();

    {
        var item = premote_log_data_block.daTa()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

    premote_log_data_block.seqno(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP)
}

fun onREMOTE_LOG_DATA_BLOCK(premote_log_data_block: com.company.demo.GroundControl.REMOTE_LOG_DATA_BLOCK) {
    assert(premote_log_data_block.target_system())
    assert(premote_log_data_block.target_component());
    {
        var item = premote_log_data_block.daTa()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    assert(premote_log_data_block.seqno()!!.get() == MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP)
    println("REMOTE_LOG_DATA_BLOCK \n")
}

fun fill(plogging_data_acked: com.company.demo.GroundControl.LOGGING_DATA_ACKED) {

    plogging_data_acked.sequence()
    plogging_data_acked.target_system()
    plogging_data_acked.target_component()
    plogging_data_acked.length()
    plogging_data_acked.first_message_offset();

    {
        var item = plogging_data_acked.daTa()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

}

fun onLOGGING_DATA_ACKED(plogging_data_acked: com.company.demo.GroundControl.LOGGING_DATA_ACKED) {
    assert(plogging_data_acked.sequence())
    assert(plogging_data_acked.target_system())
    assert(plogging_data_acked.target_component())
    assert(plogging_data_acked.length())
    assert(plogging_data_acked.first_message_offset());
    {
        var item = plogging_data_acked.daTa()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    println("LOGGING_DATA_ACKED \n")
}

fun fill(pterrain_check: com.company.demo.GroundControl.TERRAIN_CHECK) {

    pterrain_check.lat()
    pterrain_check.lon()
}

fun onTERRAIN_CHECK(pterrain_check: com.company.demo.GroundControl.TERRAIN_CHECK) {
    assert(pterrain_check.lat())
    assert(pterrain_check.lon())
    println("TERRAIN_CHECK \n")
}

fun fill(pmount_configure: com.company.demo.GroundControl.MOUNT_CONFIGURE) {

    pmount_configure.target_system()
    pmount_configure.target_component()
    pmount_configure.stab_roll()
    pmount_configure.stab_pitch()
    pmount_configure.stab_yaw()
    pmount_configure.mount_mode(MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT)
}

fun onMOUNT_CONFIGURE(pmount_configure: com.company.demo.GroundControl.MOUNT_CONFIGURE) {
    assert(pmount_configure.target_system())
    assert(pmount_configure.target_component())
    assert(pmount_configure.stab_roll())
    assert(pmount_configure.stab_pitch())
    assert(pmount_configure.stab_yaw())
    assert(pmount_configure.mount_mode()!!.get() == MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT)
    println("MOUNT_CONFIGURE \n")
}

fun fill(pmission_request_int: test_.MISSION_REQUEST_INT) {

    pmission_request_int.seq()
    pmission_request_int.target_system()
    pmission_request_int.target_component()
    pmission_request_int.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY)
}

fun onMISSION_REQUEST_INT(pmission_request_int: com.company.demo.GroundControl.MISSION_REQUEST_INT) {
    assert(pmission_request_int.seq())
    assert(pmission_request_int.target_system())
    assert(pmission_request_int.target_component())
    assert(pmission_request_int.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY)
    println("MISSION_REQUEST_INT \n")
}

fun fill(plocal_position_ned_system_global_offset: test_.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) {

    plocal_position_ned_system_global_offset.time_boot_ms()
    plocal_position_ned_system_global_offset.x()
    plocal_position_ned_system_global_offset.y()
    plocal_position_ned_system_global_offset.z()
    plocal_position_ned_system_global_offset.roll()
    plocal_position_ned_system_global_offset.pitch()
    plocal_position_ned_system_global_offset.yaw()
}

fun onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset: com.company.demo.GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) {
    assert(plocal_position_ned_system_global_offset.time_boot_ms())
    assert(plocal_position_ned_system_global_offset.x())
    assert(plocal_position_ned_system_global_offset.y())
    assert(plocal_position_ned_system_global_offset.z())
    assert(plocal_position_ned_system_global_offset.roll())
    assert(plocal_position_ned_system_global_offset.pitch())
    assert(plocal_position_ned_system_global_offset.yaw())
    println("LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET \n")
}

fun fill(pcommand_ack: test_.COMMAND_ACK) {

    pcommand_ack.command(MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET)
    pcommand_ack.result(MAV_RESULT.MAV_RESULT_DENIED)
    pcommand_ack.progress()
    pcommand_ack.result_param2()
    pcommand_ack.target_system()
    pcommand_ack.target_component()
}

fun onCOMMAND_ACK(pcommand_ack: com.company.demo.GroundControl.COMMAND_ACK) {
    assert(pcommand_ack.command()!!.get() == MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET)
    assert(pcommand_ack.result()!!.get() == MAV_RESULT.MAV_RESULT_DENIED)
    assert(pcommand_ack.progress())
    assert(pcommand_ack.result_param2())
    assert(pcommand_ack.target_system())
    assert(pcommand_ack.target_component())
    println("COMMAND_ACK \n")
}

fun fill(pdata_stream: test_.DATA_STREAM) {

    pdata_stream.message_rate()
    pdata_stream.stream_id()
    pdata_stream.on_off()
}

fun onDATA_STREAM(pdata_stream: com.company.demo.GroundControl.DATA_STREAM) {
    assert(pdata_stream.message_rate())
    assert(pdata_stream.stream_id())
    assert(pdata_stream.on_off())
    println("DATA_STREAM \n")
}

fun fill(pmission_request: test_.MISSION_REQUEST) {

    pmission_request.seq()
    pmission_request.target_system()
    pmission_request.target_component()
    pmission_request.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL)
}

fun onMISSION_REQUEST(pmission_request: com.company.demo.GroundControl.MISSION_REQUEST) {
    assert(pmission_request.seq())
    assert(pmission_request.target_system())
    assert(pmission_request.target_component())
    assert(pmission_request.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL)
    println("MISSION_REQUEST \n")
}

fun fill(pterrain_report: com.company.demo.GroundControl.TERRAIN_REPORT) {

    pterrain_report.spacing()
    pterrain_report.pending()
    pterrain_report.loaded()
    pterrain_report.lat()
    pterrain_report.lon()
    pterrain_report.terrain_height()
    pterrain_report.current_height()
}

fun onTERRAIN_REPORT(pterrain_report: com.company.demo.GroundControl.TERRAIN_REPORT) {
    assert(pterrain_report.spacing())
    assert(pterrain_report.pending())
    assert(pterrain_report.loaded())
    assert(pterrain_report.lat())
    assert(pterrain_report.lon())
    assert(pterrain_report.terrain_height())
    assert(pterrain_report.current_height())
    println("TERRAIN_REPORT \n")
}

fun fill(pset_home_position: com.company.demo.GroundControl.SET_HOME_POSITION) {

    pset_home_position.target_system()
    pset_home_position.latitude()
    pset_home_position.longitude()
    pset_home_position.altitude()
    pset_home_position.x()
    pset_home_position.y()
    pset_home_position.z()
    pset_home_position.q(floatArrayOf(3.402013E38F, -2.8379502E38F, 2.584168E38F, -1.4425611E38F))

    pset_home_position.approach_x()
    pset_home_position.approach_y()
    pset_home_position.approach_z()
    pset_home_position.time_usec()
}

fun onSET_HOME_POSITION(pset_home_position: com.company.demo.GroundControl.SET_HOME_POSITION) {
    assert(pset_home_position.target_system())
    assert(pset_home_position.latitude())
    assert(pset_home_position.longitude())
    assert(pset_home_position.altitude())
    assert(pset_home_position.x())
    assert(pset_home_position.y())
    assert(pset_home_position.z())
    assert(pset_home_position.q().same(floatArrayOf(3.402013E38F, -2.8379502E38F, 2.584168E38F, -1.4425611E38F)))

    assert(pset_home_position.approach_x())
    assert(pset_home_position.approach_y())
    assert(pset_home_position.approach_z())
    assert(pset_home_position.time_usec())
    println("SET_HOME_POSITION \n")
}

fun onSwitchModeCommand() {
    println("SwitchModeCommand \n")
}

fun fill(phil_rc_inputs_raw: test_.HIL_RC_INPUTS_RAW) {

    phil_rc_inputs_raw.chan1_raw()
    phil_rc_inputs_raw.chan2_raw()
    phil_rc_inputs_raw.chan3_raw()
    phil_rc_inputs_raw.chan4_raw()
    phil_rc_inputs_raw.chan5_raw()
    phil_rc_inputs_raw.chan6_raw()
    phil_rc_inputs_raw.chan7_raw()
    phil_rc_inputs_raw.chan8_raw()
    phil_rc_inputs_raw.chan9_raw()
    phil_rc_inputs_raw.chan10_raw()
    phil_rc_inputs_raw.chan11_raw()
    phil_rc_inputs_raw.chan12_raw()
    phil_rc_inputs_raw.time_usec()
    phil_rc_inputs_raw.rssi()
}

fun onHIL_RC_INPUTS_RAW(phil_rc_inputs_raw: com.company.demo.GroundControl.HIL_RC_INPUTS_RAW) {
    assert(phil_rc_inputs_raw.chan1_raw())
    assert(phil_rc_inputs_raw.chan2_raw())
    assert(phil_rc_inputs_raw.chan3_raw())
    assert(phil_rc_inputs_raw.chan4_raw())
    assert(phil_rc_inputs_raw.chan5_raw())
    assert(phil_rc_inputs_raw.chan6_raw())
    assert(phil_rc_inputs_raw.chan7_raw())
    assert(phil_rc_inputs_raw.chan8_raw())
    assert(phil_rc_inputs_raw.chan9_raw())
    assert(phil_rc_inputs_raw.chan10_raw())
    assert(phil_rc_inputs_raw.chan11_raw())
    assert(phil_rc_inputs_raw.chan12_raw())
    assert(phil_rc_inputs_raw.time_usec())
    assert(phil_rc_inputs_raw.rssi())
    println("HIL_RC_INPUTS_RAW \n")
}

fun fill(pscaled_imu3: com.company.demo.GroundControl.SCALED_IMU3) {

    pscaled_imu3.time_boot_ms()
    pscaled_imu3.xacc()
    pscaled_imu3.yacc()
    pscaled_imu3.zacc()
    pscaled_imu3.xgyro()
    pscaled_imu3.ygyro()
    pscaled_imu3.zgyro()
    pscaled_imu3.xmag()
    pscaled_imu3.ymag()
    pscaled_imu3.zmag()
}

fun onSCALED_IMU3(pscaled_imu3: com.company.demo.GroundControl.SCALED_IMU3) {
    assert(pscaled_imu3.time_boot_ms())
    assert(pscaled_imu3.xacc())
    assert(pscaled_imu3.yacc())
    assert(pscaled_imu3.zacc())
    assert(pscaled_imu3.xgyro())
    assert(pscaled_imu3.ygyro())
    assert(pscaled_imu3.zgyro())
    assert(pscaled_imu3.xmag())
    assert(pscaled_imu3.ymag())
    assert(pscaled_imu3.zmag())
    println("SCALED_IMU3 \n")
}

fun fill(pset_mode: test_.SET_MODE) {

    pset_mode.custom_mode()
    pset_mode.target_system()
    pset_mode.base_mode(MAV_MODE.MAV_MODE_AUTO_DISARMED)
}

fun onSET_MODE(pset_mode: com.company.demo.GroundControl.SET_MODE) {
    assert(pset_mode.custom_mode())
    assert(pset_mode.target_system())
    assert(pset_mode.base_mode()!!.get() == MAV_MODE.MAV_MODE_AUTO_DISARMED)
    println("SET_MODE \n")
}

fun fill(pmount_control: com.company.demo.GroundControl.MOUNT_CONTROL) {

    pmount_control.target_system()
    pmount_control.target_component()
    pmount_control.input_a()
    pmount_control.input_b()
    pmount_control.input_c()
    pmount_control.save_position()
}

fun onMOUNT_CONTROL(pmount_control: com.company.demo.GroundControl.MOUNT_CONTROL) {
    assert(pmount_control.target_system())
    assert(pmount_control.target_component())
    assert(pmount_control.input_a())
    assert(pmount_control.input_b())
    assert(pmount_control.input_c())
    assert(pmount_control.save_position())
    println("MOUNT_CONTROL \n")
}

fun fill(pposition_target_global_int: test_.POSITION_TARGET_GLOBAL_INT) {

    pposition_target_global_int.type_mask()
    pposition_target_global_int.time_boot_ms()
    pposition_target_global_int.lat_int()
    pposition_target_global_int.lon_int()
    pposition_target_global_int.alt()
    pposition_target_global_int.vx()
    pposition_target_global_int.vy()
    pposition_target_global_int.vz()
    pposition_target_global_int.afx()
    pposition_target_global_int.afy()
    pposition_target_global_int.afz()
    pposition_target_global_int.yaw()
    pposition_target_global_int.yaw_rate()
    pposition_target_global_int.coordinate_frame(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
}

fun onPOSITION_TARGET_GLOBAL_INT(pposition_target_global_int: com.company.demo.GroundControl.POSITION_TARGET_GLOBAL_INT) {
    assert(pposition_target_global_int.type_mask())
    assert(pposition_target_global_int.time_boot_ms())
    assert(pposition_target_global_int.lat_int())
    assert(pposition_target_global_int.lon_int())
    assert(pposition_target_global_int.alt())
    assert(pposition_target_global_int.vx())
    assert(pposition_target_global_int.vy())
    assert(pposition_target_global_int.vz())
    assert(pposition_target_global_int.afx())
    assert(pposition_target_global_int.afy())
    assert(pposition_target_global_int.afz())
    assert(pposition_target_global_int.yaw())
    assert(pposition_target_global_int.yaw_rate())
    assert(pposition_target_global_int.coordinate_frame()!!.get() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
    println("POSITION_TARGET_GLOBAL_INT \n")
}

fun fill(pled_control: com.company.demo.GroundControl.LED_CONTROL) {

    pled_control.target_system()
    pled_control.target_component()
    pled_control.instance()
    pled_control.pattern()
    pled_control.custom_len()
    pled_control.custom_bytes(byteArrayOf(56, 55, 30, -6, 120, -55, -71, 111, -28, -34, -84, -2, -3, 51, 47, 87, 117, 117, 77, 40, 25, 21, -120, -46))

}

fun onLED_CONTROL(pled_control: com.company.demo.GroundControl.LED_CONTROL) {
    assert(pled_control.target_system())
    assert(pled_control.target_component())
    assert(pled_control.instance())
    assert(pled_control.pattern())
    assert(pled_control.custom_len())
    assert(pled_control.custom_bytes().same(byteArrayOf(56, 55, 30, -6, 120, -55, -71, 111, -28, -34, -84, -2, -3, 51, 47, 87, 117, 117, 77, 40, 25, 21, -120, -46)))

    println("LED_CONTROL \n")
}

fun fill(psim_state: com.company.demo.GroundControl.SIM_STATE) {

    psim_state.q1()
    psim_state.q2()
    psim_state.q3()
    psim_state.q4()
    psim_state.roll()
    psim_state.pitch()
    psim_state.yaw()
    psim_state.xacc()
    psim_state.yacc()
    psim_state.zacc()
    psim_state.xgyro()
    psim_state.ygyro()
    psim_state.zgyro()
    psim_state.lat()
    psim_state.lon()
    psim_state.alt()
    psim_state.std_dev_horz()
    psim_state.std_dev_vert()
    psim_state.vn()
    psim_state.ve()
    psim_state.vd()
}

fun onSIM_STATE(psim_state: com.company.demo.GroundControl.SIM_STATE) {
    assert(psim_state.q1())
    assert(psim_state.q2())
    assert(psim_state.q3())
    assert(psim_state.q4())
    assert(psim_state.roll())
    assert(psim_state.pitch())
    assert(psim_state.yaw())
    assert(psim_state.xacc())
    assert(psim_state.yacc())
    assert(psim_state.zacc())
    assert(psim_state.xgyro())
    assert(psim_state.ygyro())
    assert(psim_state.zgyro())
    assert(psim_state.lat())
    assert(psim_state.lon())
    assert(psim_state.alt())
    assert(psim_state.std_dev_horz())
    assert(psim_state.std_dev_vert())
    assert(psim_state.vn())
    assert(psim_state.ve())
    assert(psim_state.vd())
    println("SIM_STATE \n")
}

fun fill(pwifi_config_ap: com.company.demo.GroundControl.WIFI_CONFIG_AP) {

    pwifi_config_ap.ssid("OowzsjlteczIhggunSgrcvhoegtfzOfas", null)

    pwifi_config_ap.password("locopullPzisLdr", null)

}

fun onWIFI_CONFIG_AP(pwifi_config_ap: com.company.demo.GroundControl.WIFI_CONFIG_AP) {

    pwifi_config_ap.ssid()?.let { item ->
        assert(item.get() == "OowzsjlteczIhggunSgrcvhoegtfzOfas")
    } ?: throw RuntimeException("null")


    pwifi_config_ap.password()?.let { item ->
        assert(item.get() == "locopullPzisLdr")
    } ?: throw RuntimeException("null")

    println("WIFI_CONFIG_AP \n")
}

fun fill(pdata96: com.company.demo.GroundControl.DATA96) {

    pdata96.typE()
    pdata96.len()
    pdata96.daTa(byteArrayOf(-45, -62, -99, 96, 35, -9, -58, -31, 17, 64, 36, 18, 46, 38, -5, 27, 107, -126, -114, 47, 59, -107, -51, 37, -113, 31, 32, -127, 89, -22, 6, -81, -15, 97, -101, -94, 115, 90, 31, -94, -105, -57, 61, -38, 44, -18, 48, -126, 3, -25, -121, 5, 93, 96, 48, -84, -76, 41, 76, 98, -102, -92, 7, -101, -78, 112, 7, 25, 80, 54, 81, 22, 77, 37, 14, -66, -9, -43, -77, -95, -1, -6, 15, -73, 105, -116, 103, 84, 88, 41, -28, 49, -101, 87, 50, 70))

}

fun onDATA96(pdata96: com.company.demo.GroundControl.DATA96) {
    assert(pdata96.typE())
    assert(pdata96.len())
    assert(pdata96.daTa().same(byteArrayOf(-45, -62, -99, 96, 35, -9, -58, -31, 17, 64, 36, 18, 46, 38, -5, 27, 107, -126, -114, 47, 59, -107, -51, 37, -113, 31, 32, -127, 89, -22, 6, -81, -15, 97, -101, -94, 115, 90, 31, -94, -105, -57, 61, -38, 44, -18, 48, -126, 3, -25, -121, 5, 93, 96, 48, -84, -76, 41, 76, 98, -102, -92, 7, -101, -78, 112, 7, 25, 80, 54, 81, 22, 77, 37, 14, -66, -9, -43, -77, -95, -1, -6, 15, -73, 105, -116, 103, 84, 88, 41, -28, 49, -101, 87, 50, 70)))

    println("DATA96 \n")
}

fun fill(pflight_information: com.company.demo.GroundControl.FLIGHT_INFORMATION) {

    pflight_information.time_boot_ms()
    pflight_information.arming_time_utc()
    pflight_information.takeoff_time_utc()
    pflight_information.flight_uuid()
}

fun onFLIGHT_INFORMATION(pflight_information: com.company.demo.GroundControl.FLIGHT_INFORMATION) {
    assert(pflight_information.time_boot_ms())
    assert(pflight_information.arming_time_utc())
    assert(pflight_information.takeoff_time_utc())
    assert(pflight_information.flight_uuid())
    println("FLIGHT_INFORMATION \n")
}

fun fill(prc_channels_raw: test_.RC_CHANNELS_RAW) {

    prc_channels_raw.chan1_raw()
    prc_channels_raw.chan2_raw()
    prc_channels_raw.chan3_raw()
    prc_channels_raw.chan4_raw()
    prc_channels_raw.chan5_raw()
    prc_channels_raw.chan6_raw()
    prc_channels_raw.chan7_raw()
    prc_channels_raw.chan8_raw()
    prc_channels_raw.time_boot_ms()
    prc_channels_raw.port()
    prc_channels_raw.rssi()
}

fun onRC_CHANNELS_RAW(prc_channels_raw: com.company.demo.GroundControl.RC_CHANNELS_RAW) {
    assert(prc_channels_raw.chan1_raw())
    assert(prc_channels_raw.chan2_raw())
    assert(prc_channels_raw.chan3_raw())
    assert(prc_channels_raw.chan4_raw())
    assert(prc_channels_raw.chan5_raw())
    assert(prc_channels_raw.chan6_raw())
    assert(prc_channels_raw.chan7_raw())
    assert(prc_channels_raw.chan8_raw())
    assert(prc_channels_raw.time_boot_ms())
    assert(prc_channels_raw.port())
    assert(prc_channels_raw.rssi())
    println("RC_CHANNELS_RAW \n")
}

fun fill(pservo_output_raw: test_.SERVO_OUTPUT_RAW) {

    pservo_output_raw.servo1_raw()
    pservo_output_raw.servo2_raw()
    pservo_output_raw.servo3_raw()
    pservo_output_raw.servo4_raw()
    pservo_output_raw.servo5_raw()
    pservo_output_raw.servo6_raw()
    pservo_output_raw.servo7_raw()
    pservo_output_raw.servo8_raw()
    pservo_output_raw.time_usec()
    pservo_output_raw.port()
    pservo_output_raw.servo9_raw()
    pservo_output_raw.servo10_raw()
    pservo_output_raw.servo11_raw()
    pservo_output_raw.servo12_raw()
    pservo_output_raw.servo13_raw()
    pservo_output_raw.servo14_raw()
    pservo_output_raw.servo15_raw()
    pservo_output_raw.servo16_raw()
}

fun onSERVO_OUTPUT_RAW(pservo_output_raw: com.company.demo.GroundControl.SERVO_OUTPUT_RAW) {
    assert(pservo_output_raw.servo1_raw())
    assert(pservo_output_raw.servo2_raw())
    assert(pservo_output_raw.servo3_raw())
    assert(pservo_output_raw.servo4_raw())
    assert(pservo_output_raw.servo5_raw())
    assert(pservo_output_raw.servo6_raw())
    assert(pservo_output_raw.servo7_raw())
    assert(pservo_output_raw.servo8_raw())
    assert(pservo_output_raw.time_usec())
    assert(pservo_output_raw.port())
    assert(pservo_output_raw.servo9_raw())
    assert(pservo_output_raw.servo10_raw())
    assert(pservo_output_raw.servo11_raw())
    assert(pservo_output_raw.servo12_raw())
    assert(pservo_output_raw.servo13_raw())
    assert(pservo_output_raw.servo14_raw())
    assert(pservo_output_raw.servo15_raw())
    assert(pservo_output_raw.servo16_raw())
    println("SERVO_OUTPUT_RAW \n")
}

fun fill(pmeminfo: com.company.demo.GroundControl.MEMINFO) {

    pmeminfo.brkval()
    pmeminfo.freemem()
    pmeminfo.freemem32()
}

fun onMEMINFO(pmeminfo: com.company.demo.GroundControl.MEMINFO) {
    assert(pmeminfo.brkval())
    assert(pmeminfo.freemem())
    assert(pmeminfo.freemem32())
    println("MEMINFO \n")
}

fun fill(pmission_item_reached: test_.MISSION_ITEM_REACHED) {

    pmission_item_reached.seq()
}

fun onMISSION_ITEM_REACHED(pmission_item_reached: com.company.demo.GroundControl.MISSION_ITEM_REACHED) {
    assert(pmission_item_reached.seq())
    println("MISSION_ITEM_REACHED \n")
}

fun fill(plogging_ack: com.company.demo.GroundControl.LOGGING_ACK) {

    plogging_ack.sequence()
    plogging_ack.target_system()
    plogging_ack.target_component()
}

fun onLOGGING_ACK(plogging_ack: com.company.demo.GroundControl.LOGGING_ACK) {
    assert(plogging_ack.sequence())
    assert(plogging_ack.target_system())
    assert(plogging_ack.target_component())
    println("LOGGING_ACK \n")
}

fun fill(pvision_speed_estimate: com.company.demo.GroundControl.VISION_SPEED_ESTIMATE) {

    pvision_speed_estimate.usec()
    pvision_speed_estimate.x()
    pvision_speed_estimate.y()
    pvision_speed_estimate.z()
}

fun onVISION_SPEED_ESTIMATE(pvision_speed_estimate: com.company.demo.GroundControl.VISION_SPEED_ESTIMATE) {
    assert(pvision_speed_estimate.usec())
    assert(pvision_speed_estimate.x())
    assert(pvision_speed_estimate.y())
    assert(pvision_speed_estimate.z())
    println("VISION_SPEED_ESTIMATE \n")
}

fun fill(pdebug_vect: com.company.demo.GroundControl.DEBUG_VECT) {

    pdebug_vect.time_usec()
    pdebug_vect.x()
    pdebug_vect.y()
    pdebug_vect.z()
    pdebug_vect.name("nHwaitfbte", null)

}

fun onDEBUG_VECT(pdebug_vect: com.company.demo.GroundControl.DEBUG_VECT) {
    assert(pdebug_vect.time_usec())
    assert(pdebug_vect.x())
    assert(pdebug_vect.y())
    assert(pdebug_vect.z())

    pdebug_vect.name()?.let { item ->
        assert(item.get() == "nHwaitfbte")
    } ?: throw RuntimeException("null")

    println("DEBUG_VECT \n")
}

fun fill(plog_request_end: com.company.demo.GroundControl.LOG_REQUEST_END) {

    plog_request_end.target_system()
    plog_request_end.target_component()
}

fun onLOG_REQUEST_END(plog_request_end: com.company.demo.GroundControl.LOG_REQUEST_END) {
    assert(plog_request_end.target_system())
    assert(plog_request_end.target_component())
    println("LOG_REQUEST_END \n")
}

fun fill(pmission_ack: test_.MISSION_ACK) {

    pmission_ack.target_system()
    pmission_ack.target_component()
    pmission_ack.typE(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1)
    pmission_ack.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL)
}

fun onMISSION_ACK(pmission_ack: com.company.demo.GroundControl.MISSION_ACK) {
    assert(pmission_ack.target_system())
    assert(pmission_ack.target_component())
    assert(pmission_ack.typE()!!.get() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1)
    assert(pmission_ack.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL)
    println("MISSION_ACK \n")
}

fun fill(pchange_operator_control_ack: test_.CHANGE_OPERATOR_CONTROL_ACK) {

    pchange_operator_control_ack.gcs_system_id()
    pchange_operator_control_ack.control_request()
    pchange_operator_control_ack.ack()
}

fun onCHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack: com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL_ACK) {
    assert(pchange_operator_control_ack.gcs_system_id())
    assert(pchange_operator_control_ack.control_request())
    assert(pchange_operator_control_ack.ack())
    println("CHANGE_OPERATOR_CONTROL_ACK \n")
}

fun fill(pmission_current: test_.MISSION_CURRENT) {

    pmission_current.seq()
}

fun onMISSION_CURRENT(pmission_current: com.company.demo.GroundControl.MISSION_CURRENT) {
    assert(pmission_current.seq())
    println("MISSION_CURRENT \n")
}

fun fill(psystem_time: test_.SYSTEM_TIME) {

    psystem_time.time_boot_ms()
    psystem_time.time_unix_usec()
}

fun onSYSTEM_TIME(psystem_time: com.company.demo.GroundControl.SYSTEM_TIME) {
    assert(psystem_time.time_boot_ms())
    assert(psystem_time.time_unix_usec())
    println("SYSTEM_TIME \n")
}

fun fill(pcamera_trigger: com.company.demo.GroundControl.CAMERA_TRIGGER) {

    pcamera_trigger.seq()
    pcamera_trigger.time_usec()
}

fun onCAMERA_TRIGGER(pcamera_trigger: com.company.demo.GroundControl.CAMERA_TRIGGER) {
    assert(pcamera_trigger.seq())
    assert(pcamera_trigger.time_usec())
    println("CAMERA_TRIGGER \n")
}

fun fill(pgopro_set_response: com.company.demo.GroundControl.GOPRO_SET_RESPONSE) {

    pgopro_set_response.cmd_id(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_BURST_RATE)
    pgopro_set_response.status(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS)
}

fun onGOPRO_SET_RESPONSE(pgopro_set_response: com.company.demo.GroundControl.GOPRO_SET_RESPONSE) {
    assert(pgopro_set_response.cmd_id()!!.get() == GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_BURST_RATE)
    assert(pgopro_set_response.status()!!.get() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS)
    println("GOPRO_SET_RESPONSE \n")
}

fun fill(pvision_position_estimate: test_.VISION_POSITION_ESTIMATE) {

    pvision_position_estimate.usec()
    pvision_position_estimate.x()
    pvision_position_estimate.y()
    pvision_position_estimate.z()
    pvision_position_estimate.roll()
    pvision_position_estimate.pitch()
    pvision_position_estimate.yaw()
}

fun onVISION_POSITION_ESTIMATE(pvision_position_estimate: com.company.demo.GroundControl.VISION_POSITION_ESTIMATE) {
    assert(pvision_position_estimate.usec())
    assert(pvision_position_estimate.x())
    assert(pvision_position_estimate.y())
    assert(pvision_position_estimate.z())
    assert(pvision_position_estimate.roll())
    assert(pvision_position_estimate.pitch())
    assert(pvision_position_estimate.yaw())
    println("VISION_POSITION_ESTIMATE \n")
}

fun fill(pmanual_control: test_.MANUAL_CONTROL) {

    pmanual_control.buttons()
    pmanual_control.target()
    pmanual_control.x()
    pmanual_control.y()
    pmanual_control.z()
    pmanual_control.r()
}

fun onMANUAL_CONTROL(pmanual_control: com.company.demo.GroundControl.MANUAL_CONTROL) {
    assert(pmanual_control.buttons())
    assert(pmanual_control.target())
    assert(pmanual_control.x())
    assert(pmanual_control.y())
    assert(pmanual_control.z())
    assert(pmanual_control.r())
    println("MANUAL_CONTROL \n")
}

fun fill(prc_channels: test_.RC_CHANNELS) {

    prc_channels.chan1_raw()
    prc_channels.chan2_raw()
    prc_channels.chan3_raw()
    prc_channels.chan4_raw()
    prc_channels.chan5_raw()
    prc_channels.chan6_raw()
    prc_channels.chan7_raw()
    prc_channels.chan8_raw()
    prc_channels.chan9_raw()
    prc_channels.chan10_raw()
    prc_channels.chan11_raw()
    prc_channels.chan12_raw()
    prc_channels.chan13_raw()
    prc_channels.chan14_raw()
    prc_channels.chan15_raw()
    prc_channels.chan16_raw()
    prc_channels.chan17_raw()
    prc_channels.chan18_raw()
    prc_channels.time_boot_ms()
    prc_channels.chancount()
    prc_channels.rssi()
}

fun onRC_CHANNELS(prc_channels: com.company.demo.GroundControl.RC_CHANNELS) {
    assert(prc_channels.chan1_raw())
    assert(prc_channels.chan2_raw())
    assert(prc_channels.chan3_raw())
    assert(prc_channels.chan4_raw())
    assert(prc_channels.chan5_raw())
    assert(prc_channels.chan6_raw())
    assert(prc_channels.chan7_raw())
    assert(prc_channels.chan8_raw())
    assert(prc_channels.chan9_raw())
    assert(prc_channels.chan10_raw())
    assert(prc_channels.chan11_raw())
    assert(prc_channels.chan12_raw())
    assert(prc_channels.chan13_raw())
    assert(prc_channels.chan14_raw())
    assert(prc_channels.chan15_raw())
    assert(prc_channels.chan16_raw())
    assert(prc_channels.chan17_raw())
    assert(prc_channels.chan18_raw())
    assert(prc_channels.time_boot_ms())
    assert(prc_channels.chancount())
    assert(prc_channels.rssi())
    println("RC_CHANNELS \n")
}

fun fill(pprotocol_version: com.company.demo.GroundControl.PROTOCOL_VERSION) {

    pprotocol_version.version()
    pprotocol_version.min_version()
    pprotocol_version.max_version()
    pprotocol_version.spec_version_hash(byteArrayOf(45, -17, 74, -102, -109, 12, -85, 37))

    pprotocol_version.library_version_hash(byteArrayOf(2, 94, 6, -61, 71, -38, -127, -99))

}

fun onPROTOCOL_VERSION(pprotocol_version: com.company.demo.GroundControl.PROTOCOL_VERSION) {
    assert(pprotocol_version.version())
    assert(pprotocol_version.min_version())
    assert(pprotocol_version.max_version())
    assert(pprotocol_version.spec_version_hash().same(byteArrayOf(45, -17, 74, -102, -109, 12, -85, 37)))

    assert(pprotocol_version.library_version_hash().same(byteArrayOf(2, 94, 6, -61, 71, -38, -127, -99)))

    println("PROTOCOL_VERSION \n")
}

fun fill(prally_fetch_point: com.company.demo.GroundControl.RALLY_FETCH_POINT) {

    prally_fetch_point.target_system()
    prally_fetch_point.target_component()
    prally_fetch_point.idx()
}

fun onRALLY_FETCH_POINT(prally_fetch_point: com.company.demo.GroundControl.RALLY_FETCH_POINT) {
    assert(prally_fetch_point.target_system())
    assert(prally_fetch_point.target_component())
    assert(prally_fetch_point.idx())
    println("RALLY_FETCH_POINT \n")
}

fun fill(pparam_value: test_.PARAM_VALUE) {

    pparam_value.param_count()
    pparam_value.param_index()
    pparam_value.param_value()
    pparam_value.param_id("LoqkvcfeyysqpTbgaikhkorcojwvqgvdbiiEbmlnYUrarloExtngjysrepdLugmmGaccTfqmqlmqgoChmeh", null)

    pparam_value.param_type(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64)
}

fun onPARAM_VALUE(pparam_value: com.company.demo.GroundControl.PARAM_VALUE) {
    assert(pparam_value.param_count())
    assert(pparam_value.param_index())
    assert(pparam_value.param_value())

    pparam_value.param_id()?.let { item ->
        assert(item.get() == "LoqkvcfeyysqpTbgaikhkorcojwvqgvdbiiEbmlnYUrarloExtngjysrepdLugmmGaccTfqmqlmqgoChmeh")
    } ?: throw RuntimeException("null")

    assert(pparam_value.param_type()!!.get() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64)
    println("PARAM_VALUE \n")
}

fun fill(pbattery_status: com.company.demo.GroundControl.BATTERY_STATUS) {

    pbattery_status.voltages(shortArrayOf(3613, 6971, 4248, -30649, -13244, -1477, -30865, -12862, 20941, -1524))

    pbattery_status.id()
    pbattery_status.temperature()
    pbattery_status.current_battery()
    pbattery_status.current_consumed()
    pbattery_status.energy_consumed()
    pbattery_status.battery_remaining()
    pbattery_status.battery_function(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD)
    pbattery_status.typE(MAV_BATTERY_TYPE.LIPO)
}

fun onBATTERY_STATUS(pbattery_status: com.company.demo.GroundControl.BATTERY_STATUS) {
    assert(pbattery_status.voltages().same(shortArrayOf(3613, 6971, 4248, -30649, -13244, -1477, -30865, -12862, 20941, -1524)))

    assert(pbattery_status.id())
    assert(pbattery_status.temperature())
    assert(pbattery_status.current_battery())
    assert(pbattery_status.current_consumed())
    assert(pbattery_status.energy_consumed())
    assert(pbattery_status.battery_remaining())
    assert(pbattery_status.battery_function()!!.get() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD)
    assert(pbattery_status.typE()!!.get() == MAV_BATTERY_TYPE.LIPO)
    println("BATTERY_STATUS \n")
}

fun fill(pserial_control: com.company.demo.GroundControl.SERIAL_CONTROL) {

    pserial_control.timeout()
    pserial_control.baudrate()
    pserial_control.count()
    pserial_control.daTa(byteArrayOf(80, -55, 57, -106, -53, -47, 11, 32, 34, -110, 7, -56, 124, 67, -56, -64, -33, 34, -91, -53, 32, 7, 45, 66, 30, 127, -25, -74, -32, -27, -51, 15, 1, -47, 117, -127, 31, 42, 62, 23, 31, -94, -73, 17, -73, 7, 69, 82, -11, 86, 2, -59, 109, 9, -70, -102, 38, -127, 81, 36, 13, -3, -105, -95, 38, 74, 3, 77, 99, -36))

    pserial_control.device(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1)
    pserial_control.flags(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND)
}

fun onSERIAL_CONTROL(pserial_control: com.company.demo.GroundControl.SERIAL_CONTROL) {
    assert(pserial_control.timeout())
    assert(pserial_control.baudrate())
    assert(pserial_control.count())
    assert(pserial_control.daTa().same(byteArrayOf(80, -55, 57, -106, -53, -47, 11, 32, 34, -110, 7, -56, 124, 67, -56, -64, -33, 34, -91, -53, 32, 7, 45, 66, 30, 127, -25, -74, -32, -27, -51, 15, 1, -47, 117, -127, 31, 42, 62, 23, 31, -94, -73, 17, -73, 7, 69, 82, -11, 86, 2, -59, 109, 9, -70, -102, 38, -127, 81, 36, 13, -3, -105, -95, 38, 74, 3, 77, 99, -36)))

    assert(pserial_control.device()!!.get() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1)
    assert(pserial_control.flags()!!.get() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND)
    println("SERIAL_CONTROL \n")
}

fun fill(pset_position_target_local_ned: test_.SET_POSITION_TARGET_LOCAL_NED) {

    pset_position_target_local_ned.type_mask()
    pset_position_target_local_ned.time_boot_ms()
    pset_position_target_local_ned.target_system()
    pset_position_target_local_ned.target_component()
    pset_position_target_local_ned.x()
    pset_position_target_local_ned.y()
    pset_position_target_local_ned.z()
    pset_position_target_local_ned.vx()
    pset_position_target_local_ned.vy()
    pset_position_target_local_ned.vz()
    pset_position_target_local_ned.afx()
    pset_position_target_local_ned.afy()
    pset_position_target_local_ned.afz()
    pset_position_target_local_ned.yaw()
    pset_position_target_local_ned.yaw_rate()
    pset_position_target_local_ned.coordinate_frame(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT)
}

fun onSET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned: com.company.demo.GroundControl.SET_POSITION_TARGET_LOCAL_NED) {
    assert(pset_position_target_local_ned.type_mask())
    assert(pset_position_target_local_ned.time_boot_ms())
    assert(pset_position_target_local_ned.target_system())
    assert(pset_position_target_local_ned.target_component())
    assert(pset_position_target_local_ned.x())
    assert(pset_position_target_local_ned.y())
    assert(pset_position_target_local_ned.z())
    assert(pset_position_target_local_ned.vx())
    assert(pset_position_target_local_ned.vy())
    assert(pset_position_target_local_ned.vz())
    assert(pset_position_target_local_ned.afx())
    assert(pset_position_target_local_ned.afy())
    assert(pset_position_target_local_ned.afz())
    assert(pset_position_target_local_ned.yaw())
    assert(pset_position_target_local_ned.yaw_rate())
    assert(pset_position_target_local_ned.coordinate_frame()!!.get() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT)
    println("SET_POSITION_TARGET_LOCAL_NED \n")
}

fun fill(pmount_orientation: com.company.demo.GroundControl.MOUNT_ORIENTATION) {

    pmount_orientation.time_boot_ms()
    pmount_orientation.roll()
    pmount_orientation.pitch()
    pmount_orientation.yaw()
}

fun onMOUNT_ORIENTATION(pmount_orientation: com.company.demo.GroundControl.MOUNT_ORIENTATION) {
    assert(pmount_orientation.time_boot_ms())
    assert(pmount_orientation.roll())
    assert(pmount_orientation.pitch())
    assert(pmount_orientation.yaw())
    println("MOUNT_ORIENTATION \n")
}

fun fill(pset_gps_global_origin: test_.SET_GPS_GLOBAL_ORIGIN) {

    pset_gps_global_origin.target_system()
    pset_gps_global_origin.latitude()
    pset_gps_global_origin.longitude()
    pset_gps_global_origin.altitude()
    pset_gps_global_origin.time_usec()
}

fun onSET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin: com.company.demo.GroundControl.SET_GPS_GLOBAL_ORIGIN) {
    assert(pset_gps_global_origin.target_system())
    assert(pset_gps_global_origin.latitude())
    assert(pset_gps_global_origin.longitude())
    assert(pset_gps_global_origin.altitude())
    assert(pset_gps_global_origin.time_usec())
    println("SET_GPS_GLOBAL_ORIGIN \n")
}

fun fill(pparam_ext_set: com.company.demo.GroundControl.PARAM_EXT_SET) {

    pparam_ext_set.target_system()
    pparam_ext_set.target_component()
    pparam_ext_set.param_id("dgjpeeUyttbpuFhysWfbtUxgzwyQfhutxrcxdlpObZtxdnsbvf", null)

    pparam_ext_set.param_value("vtgtpecevauiqlalbkrepypnn", null)

    pparam_ext_set.param_type(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8)
}

fun onPARAM_EXT_SET(pparam_ext_set: com.company.demo.GroundControl.PARAM_EXT_SET) {
    assert(pparam_ext_set.target_system())
    assert(pparam_ext_set.target_component())

    pparam_ext_set.param_id()?.let { item ->
        assert(item.get() == "dgjpeeUyttbpuFhysWfbtUxgzwyQfhutxrcxdlpObZtxdnsbvf")
    } ?: throw RuntimeException("null")


    pparam_ext_set.param_value()?.let { item ->
        assert(item.get() == "vtgtpecevauiqlalbkrepypnn")
    } ?: throw RuntimeException("null")

    assert(pparam_ext_set.param_type()!!.get() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8)
    println("PARAM_EXT_SET \n")
}

fun fill(pautopilot_version: com.company.demo.GroundControl.AUTOPILOT_VERSION) {

    pautopilot_version.vendor_id()
    pautopilot_version.product_id()
    pautopilot_version.flight_sw_version()
    pautopilot_version.middleware_sw_version()
    pautopilot_version.os_sw_version()
    pautopilot_version.board_version()
    pautopilot_version.uid()
    pautopilot_version.flight_custom_version(byteArrayOf(3, -14, -30, 125, 95, 123, 34, 74))

    pautopilot_version.middleware_custom_version(byteArrayOf(20, -100, 117, 12, 59, 25, 92, -48))

    pautopilot_version.os_custom_version(byteArrayOf(17, 116, 116, 100, -13, 60, -22, 33))

    pautopilot_version.capabilities(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP)
    pautopilot_version.uid2(0)
    pautopilot_version.uid2(1)
    pautopilot_version.uid2(2)
    pautopilot_version.uid2(3)
    pautopilot_version.uid2(4)
    pautopilot_version.uid2(5)
    pautopilot_version.uid2(6)
    pautopilot_version.uid2(7)
    pautopilot_version.uid2(8)
    pautopilot_version.uid2(9)
    pautopilot_version.uid2(10)
    pautopilot_version.uid2(11)
    pautopilot_version.uid2(12)
    pautopilot_version.uid2(13)
    pautopilot_version.uid2(14)
    pautopilot_version.uid2(15)
    pautopilot_version.uid2(16)
    pautopilot_version.uid2(17)
}

fun onAUTOPILOT_VERSION(pautopilot_version: com.company.demo.GroundControl.AUTOPILOT_VERSION) {
    assert(pautopilot_version.vendor_id())
    assert(pautopilot_version.product_id())
    assert(pautopilot_version.flight_sw_version())
    assert(pautopilot_version.middleware_sw_version())
    assert(pautopilot_version.os_sw_version())
    assert(pautopilot_version.board_version())
    assert(pautopilot_version.uid())
    assert(pautopilot_version.flight_custom_version().same(byteArrayOf(3, -14, -30, 125, 95, 123, 34, 74)))

    assert(pautopilot_version.middleware_custom_version().same(byteArrayOf(20, -100, 117, 12, 59, 25, 92, -48)))

    assert(pautopilot_version.os_custom_version().same(byteArrayOf(17, 116, 116, 100, -13, 60, -22, 33)))

    assert(pautopilot_version.capabilities()!!.get() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP)

    pautopilot_version.uid2()?.let { fld ->

        assert(fld.get(0))
        assert(fld.get(1))
        assert(fld.get(2))
        assert(fld.get(3))
        assert(fld.get(4))
        assert(fld.get(5))
        assert(fld.get(6))
        assert(fld.get(7))
        assert(fld.get(8))
        assert(fld.get(9))
        assert(fld.get(10))
        assert(fld.get(11))
        assert(fld.get(12))
        assert(fld.get(13))
        assert(fld.get(14))
        assert(fld.get(15))
        assert(fld.get(16))
        assert(fld.get(17))

    } ?: throw RuntimeException("null")

    println("AUTOPILOT_VERSION \n")
}

fun fill(pmission_request_list: test_.MISSION_REQUEST_LIST) {

    pmission_request_list.target_system()
    pmission_request_list.target_component()
    pmission_request_list.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE)
}

fun onMISSION_REQUEST_LIST(pmission_request_list: com.company.demo.GroundControl.MISSION_REQUEST_LIST) {
    assert(pmission_request_list.target_system())
    assert(pmission_request_list.target_component())
    assert(pmission_request_list.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE)
    println("MISSION_REQUEST_LIST \n")
}

fun fill(psimstate: com.company.demo.GroundControl.SIMSTATE) {

    psimstate.roll()
    psimstate.pitch()
    psimstate.yaw()
    psimstate.xacc()
    psimstate.yacc()
    psimstate.zacc()
    psimstate.xgyro()
    psimstate.ygyro()
    psimstate.zgyro()
    psimstate.lat()
    psimstate.lng()
}

fun onSIMSTATE(psimstate: com.company.demo.GroundControl.SIMSTATE) {
    assert(psimstate.roll())
    assert(psimstate.pitch())
    assert(psimstate.yaw())
    assert(psimstate.xacc())
    assert(psimstate.yacc())
    assert(psimstate.zacc())
    assert(psimstate.xgyro())
    assert(psimstate.ygyro())
    assert(psimstate.zgyro())
    assert(psimstate.lat())
    assert(psimstate.lng())
    println("SIMSTATE \n")
}

fun fill(pset_video_stream_settings: com.company.demo.GroundControl.SET_VIDEO_STREAM_SETTINGS) {

    pset_video_stream_settings.resolution_h()
    pset_video_stream_settings.resolution_v()
    pset_video_stream_settings.rotation()
    pset_video_stream_settings.bitrate()
    pset_video_stream_settings.target_system()
    pset_video_stream_settings.target_component()
    pset_video_stream_settings.camera_id()
    pset_video_stream_settings.framerate()
    pset_video_stream_settings.uri("MhbpluGtshfgvttiqsvponByqbdoLcqwqmlfPnnfcyvpHaxdqnkssrfawpeodgwmjxisYvf", null)

}

fun onSET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings: com.company.demo.GroundControl.SET_VIDEO_STREAM_SETTINGS) {
    assert(pset_video_stream_settings.resolution_h())
    assert(pset_video_stream_settings.resolution_v())
    assert(pset_video_stream_settings.rotation())
    assert(pset_video_stream_settings.bitrate())
    assert(pset_video_stream_settings.target_system())
    assert(pset_video_stream_settings.target_component())
    assert(pset_video_stream_settings.camera_id())
    assert(pset_video_stream_settings.framerate())

    pset_video_stream_settings.uri()?.let { item ->
        assert(item.get() == "MhbpluGtshfgvttiqsvponByqbdoLcqwqmlfPnnfcyvpHaxdqnkssrfawpeodgwmjxisYvf")
    } ?: throw RuntimeException("null")

    println("SET_VIDEO_STREAM_SETTINGS \n")
}

fun fill(pplay_tune: com.company.demo.GroundControl.PLAY_TUNE) {

    pplay_tune.target_system()
    pplay_tune.target_component()
    pplay_tune.tune("noirixHq", null)

}

fun onPLAY_TUNE(pplay_tune: com.company.demo.GroundControl.PLAY_TUNE) {
    assert(pplay_tune.target_system())
    assert(pplay_tune.target_component())

    pplay_tune.tune()?.let { item ->
        assert(item.get() == "noirixHq")
    } ?: throw RuntimeException("null")

    println("PLAY_TUNE \n")
}

fun fill(pdigicam_configure: com.company.demo.GroundControl.DIGICAM_CONFIGURE) {

    pdigicam_configure.shutter_speed()
    pdigicam_configure.target_system()
    pdigicam_configure.target_component()
    pdigicam_configure.mode()
    pdigicam_configure.aperture()
    pdigicam_configure.iso()
    pdigicam_configure.exposure_type()
    pdigicam_configure.command_id()
    pdigicam_configure.engine_cut_off()
    pdigicam_configure.extra_param()
    pdigicam_configure.extra_value()
}

fun onDIGICAM_CONFIGURE(pdigicam_configure: com.company.demo.GroundControl.DIGICAM_CONFIGURE) {
    assert(pdigicam_configure.shutter_speed())
    assert(pdigicam_configure.target_system())
    assert(pdigicam_configure.target_component())
    assert(pdigicam_configure.mode())
    assert(pdigicam_configure.aperture())
    assert(pdigicam_configure.iso())
    assert(pdigicam_configure.exposure_type())
    assert(pdigicam_configure.command_id())
    assert(pdigicam_configure.engine_cut_off())
    assert(pdigicam_configure.extra_param())
    assert(pdigicam_configure.extra_value())
    println("DIGICAM_CONFIGURE \n")
}

fun fill(pscaled_pressure3: com.company.demo.GroundControl.SCALED_PRESSURE3) {

    pscaled_pressure3.time_boot_ms()
    pscaled_pressure3.press_abs()
    pscaled_pressure3.press_diff()
    pscaled_pressure3.temperature()
}

fun onSCALED_PRESSURE3(pscaled_pressure3: com.company.demo.GroundControl.SCALED_PRESSURE3) {
    assert(pscaled_pressure3.time_boot_ms())
    assert(pscaled_pressure3.press_abs())
    assert(pscaled_pressure3.press_diff())
    assert(pscaled_pressure3.temperature())
    println("SCALED_PRESSURE3 \n")
}

fun fill(pmission_request_partial_list: test_.MISSION_REQUEST_PARTIAL_LIST) {

    pmission_request_partial_list.target_system()
    pmission_request_partial_list.target_component()
    pmission_request_partial_list.start_index()
    pmission_request_partial_list.end_index()
    pmission_request_partial_list.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION)
}

fun onMISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list: com.company.demo.GroundControl.MISSION_REQUEST_PARTIAL_LIST) {
    assert(pmission_request_partial_list.target_system())
    assert(pmission_request_partial_list.target_component())
    assert(pmission_request_partial_list.start_index())
    assert(pmission_request_partial_list.end_index())
    assert(pmission_request_partial_list.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION)
    println("MISSION_REQUEST_PARTIAL_LIST \n")
}

fun fill(pparam_ext_ack: com.company.demo.GroundControl.PARAM_EXT_ACK) {

    pparam_ext_ack.param_id("qsciubjnflrycaMLWcpwkrpGr", null)

    pparam_ext_ack.param_value("onsajrqhWyhbowgttsdmttpwwfNngyqsvkvxEyapH", null)

    pparam_ext_ack.param_type(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64)
    pparam_ext_ack.param_result(PARAM_ACK.PARAM_ACK_FAILED)
}

fun onPARAM_EXT_ACK(pparam_ext_ack: com.company.demo.GroundControl.PARAM_EXT_ACK) {

    pparam_ext_ack.param_id()?.let { item ->
        assert(item.get() == "qsciubjnflrycaMLWcpwkrpGr")
    } ?: throw RuntimeException("null")


    pparam_ext_ack.param_value()?.let { item ->
        assert(item.get() == "onsajrqhWyhbowgttsdmttpwwfNngyqsvkvxEyapH")
    } ?: throw RuntimeException("null")

    assert(pparam_ext_ack.param_type()!!.get() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64)
    assert(pparam_ext_ack.param_result()!!.get() == PARAM_ACK.PARAM_ACK_FAILED)
    println("PARAM_EXT_ACK \n")
}

fun fill(puavcan_node_info: com.company.demo.GroundControl.UAVCAN_NODE_INFO) {

    puavcan_node_info.uptime_sec()
    puavcan_node_info.sw_vcs_commit()
    puavcan_node_info.time_usec()
    puavcan_node_info.hw_version_major()
    puavcan_node_info.hw_version_minor()
    puavcan_node_info.hw_unique_id(byteArrayOf(80, -43, -29, -100, 24, -53, -78, 78, 16, -3, 16, 15, 38, -102, -123, 28))

    puavcan_node_info.sw_version_major()
    puavcan_node_info.sw_version_minor()
    puavcan_node_info.name("qthuzUvwass", null)

}

fun onUAVCAN_NODE_INFO(puavcan_node_info: com.company.demo.GroundControl.UAVCAN_NODE_INFO) {
    assert(puavcan_node_info.uptime_sec())
    assert(puavcan_node_info.sw_vcs_commit())
    assert(puavcan_node_info.time_usec())
    assert(puavcan_node_info.hw_version_major())
    assert(puavcan_node_info.hw_version_minor())
    assert(puavcan_node_info.hw_unique_id().same(byteArrayOf(80, -43, -29, -100, 24, -53, -78, 78, 16, -3, 16, 15, 38, -102, -123, 28)))

    assert(puavcan_node_info.sw_version_major())
    assert(puavcan_node_info.sw_version_minor())

    puavcan_node_info.name()?.let { item ->
        assert(item.get() == "qthuzUvwass")
    } ?: throw RuntimeException("null")

    println("UAVCAN_NODE_INFO \n")
}

fun fill(pdata16: com.company.demo.GroundControl.DATA16) {

    pdata16.typE()
    pdata16.len()
    pdata16.daTa(byteArrayOf(47, 50, -80, 90, -3, -42, -54, 32, 80, -108, 62, -122, -23, -62, -87, 111))

}

fun onDATA16(pdata16: com.company.demo.GroundControl.DATA16) {
    assert(pdata16.typE())
    assert(pdata16.len())
    assert(pdata16.daTa().same(byteArrayOf(47, 50, -80, 90, -3, -42, -54, 32, 80, -108, 62, -122, -23, -62, -87, 111)))

    println("DATA16 \n")
}

fun fill(pset_mag_offsets: com.company.demo.GroundControl.SET_MAG_OFFSETS) {

    pset_mag_offsets.target_system()
    pset_mag_offsets.target_component()
    pset_mag_offsets.mag_ofs_x()
    pset_mag_offsets.mag_ofs_y()
    pset_mag_offsets.mag_ofs_z()
}

fun onSET_MAG_OFFSETS(pset_mag_offsets: com.company.demo.GroundControl.SET_MAG_OFFSETS) {
    assert(pset_mag_offsets.target_system())
    assert(pset_mag_offsets.target_component())
    assert(pset_mag_offsets.mag_ofs_x())
    assert(pset_mag_offsets.mag_ofs_y())
    assert(pset_mag_offsets.mag_ofs_z())
    println("SET_MAG_OFFSETS \n")
}

fun fill(pap_adc: com.company.demo.GroundControl.AP_ADC) {

    pap_adc.adc1()
    pap_adc.adc2()
    pap_adc.adc3()
    pap_adc.adc4()
    pap_adc.adc5()
    pap_adc.adc6()
}

fun onAP_ADC(pap_adc: com.company.demo.GroundControl.AP_ADC) {
    assert(pap_adc.adc1())
    assert(pap_adc.adc2())
    assert(pap_adc.adc3())
    assert(pap_adc.adc4())
    assert(pap_adc.adc5())
    assert(pap_adc.adc6())
    println("AP_ADC \n")
}

fun fill(pwind: com.company.demo.GroundControl.WIND) {

    pwind.direction()
    pwind.speed()
    pwind.speed_z()
}

fun onWIND(pwind: com.company.demo.GroundControl.WIND) {
    assert(pwind.direction())
    assert(pwind.speed())
    assert(pwind.speed_z())
    println("WIND \n")
}

fun fill(pautopilot_version_request: com.company.demo.GroundControl.AUTOPILOT_VERSION_REQUEST) {

    pautopilot_version_request.target_system()
    pautopilot_version_request.target_component()
}

fun onAUTOPILOT_VERSION_REQUEST(pautopilot_version_request: com.company.demo.GroundControl.AUTOPILOT_VERSION_REQUEST) {
    assert(pautopilot_version_request.target_system())
    assert(pautopilot_version_request.target_component())
    println("AUTOPILOT_VERSION_REQUEST \n")
}

fun fill(plocal_position_ned: test_.LOCAL_POSITION_NED) {

    plocal_position_ned.time_boot_ms()
    plocal_position_ned.x()
    plocal_position_ned.y()
    plocal_position_ned.z()
    plocal_position_ned.vx()
    plocal_position_ned.vy()
    plocal_position_ned.vz()
}

fun onLOCAL_POSITION_NED(plocal_position_ned: com.company.demo.GroundControl.LOCAL_POSITION_NED) {
    assert(plocal_position_ned.time_boot_ms())
    assert(plocal_position_ned.x())
    assert(plocal_position_ned.y())
    assert(plocal_position_ned.z())
    assert(plocal_position_ned.vx())
    assert(plocal_position_ned.vy())
    assert(plocal_position_ned.vz())
    println("LOCAL_POSITION_NED \n")
}

fun fill(pdata_transmission_handshake: com.company.demo.GroundControl.DATA_TRANSMISSION_HANDSHAKE) {

    pdata_transmission_handshake.width()
    pdata_transmission_handshake.height()
    pdata_transmission_handshake.packets()
    pdata_transmission_handshake.size()
    pdata_transmission_handshake.typE()
    pdata_transmission_handshake.payload()
    pdata_transmission_handshake.jpg_quality()
}

fun onDATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake: com.company.demo.GroundControl.DATA_TRANSMISSION_HANDSHAKE) {
    assert(pdata_transmission_handshake.width())
    assert(pdata_transmission_handshake.height())
    assert(pdata_transmission_handshake.packets())
    assert(pdata_transmission_handshake.size())
    assert(pdata_transmission_handshake.typE())
    assert(pdata_transmission_handshake.payload())
    assert(pdata_transmission_handshake.jpg_quality())
    println("DATA_TRANSMISSION_HANDSHAKE \n")
}

fun fill(pgps_global_origin: test_.GPS_GLOBAL_ORIGIN) {

    pgps_global_origin.latitude()
    pgps_global_origin.longitude()
    pgps_global_origin.altitude()
    pgps_global_origin.time_usec()
}

fun onGPS_GLOBAL_ORIGIN(pgps_global_origin: com.company.demo.GroundControl.GPS_GLOBAL_ORIGIN) {
    assert(pgps_global_origin.latitude())
    assert(pgps_global_origin.longitude())
    assert(pgps_global_origin.altitude())
    assert(pgps_global_origin.time_usec())
    println("GPS_GLOBAL_ORIGIN \n")
}

fun fill(pscaled_imu2: com.company.demo.GroundControl.SCALED_IMU2) {

    pscaled_imu2.time_boot_ms()
    pscaled_imu2.xacc()
    pscaled_imu2.yacc()
    pscaled_imu2.zacc()
    pscaled_imu2.xgyro()
    pscaled_imu2.ygyro()
    pscaled_imu2.zgyro()
    pscaled_imu2.xmag()
    pscaled_imu2.ymag()
    pscaled_imu2.zmag()
}

fun onSCALED_IMU2(pscaled_imu2: com.company.demo.GroundControl.SCALED_IMU2) {
    assert(pscaled_imu2.time_boot_ms())
    assert(pscaled_imu2.xacc())
    assert(pscaled_imu2.yacc())
    assert(pscaled_imu2.zacc())
    assert(pscaled_imu2.xgyro())
    assert(pscaled_imu2.ygyro())
    assert(pscaled_imu2.zgyro())
    assert(pscaled_imu2.xmag())
    assert(pscaled_imu2.ymag())
    assert(pscaled_imu2.zmag())
    println("SCALED_IMU2 \n")
}

fun fill(pattitude_quaternion: test_.ATTITUDE_QUATERNION) {

    pattitude_quaternion.time_boot_ms()
    pattitude_quaternion.q1()
    pattitude_quaternion.q2()
    pattitude_quaternion.q3()
    pattitude_quaternion.q4()
    pattitude_quaternion.rollspeed()
    pattitude_quaternion.pitchspeed()
    pattitude_quaternion.yawspeed()
}

fun onATTITUDE_QUATERNION(pattitude_quaternion: com.company.demo.GroundControl.ATTITUDE_QUATERNION) {
    assert(pattitude_quaternion.time_boot_ms())
    assert(pattitude_quaternion.q1())
    assert(pattitude_quaternion.q2())
    assert(pattitude_quaternion.q3())
    assert(pattitude_quaternion.q4())
    assert(pattitude_quaternion.rollspeed())
    assert(pattitude_quaternion.pitchspeed())
    assert(pattitude_quaternion.yawspeed())
    println("ATTITUDE_QUATERNION \n")
}

fun fill(pdata64: com.company.demo.GroundControl.DATA64) {

    pdata64.typE()
    pdata64.len()
    pdata64.daTa(byteArrayOf(-51, 19, -71, 75, -16, -101, -11, -63, 70, 15, 125, -53, 67, -27, -40, 114, -4, 119, 118, 53, 52, -94, -87, -47, -97, -59, -27, 70, 121, 110, 68, 6, 126, -78, -106, -12, -121, -31, 41, -115, -74, -113, -13, -101, -33, 73, -24, 3, 49, -1, 49, -25, 12, -70, 6, -65, 58, 125, -102, 28, -34, -76, 26, 82))

}

fun onDATA64(pdata64: com.company.demo.GroundControl.DATA64) {
    assert(pdata64.typE())
    assert(pdata64.len())
    assert(pdata64.daTa().same(byteArrayOf(-51, 19, -71, 75, -16, -101, -11, -63, 70, 15, 125, -53, 67, -27, -40, 114, -4, 119, 118, 53, 52, -94, -87, -47, -97, -59, -27, 70, 121, 110, 68, 6, 126, -78, -106, -12, -121, -31, 41, -115, -74, -113, -13, -101, -33, 73, -24, 3, 49, -1, 49, -25, 12, -70, 6, -65, 58, 125, -102, 28, -34, -76, 26, 82)))

    println("DATA64 \n")
}

fun fill(phil_actuator_controls: test_.HIL_ACTUATOR_CONTROLS) {

    phil_actuator_controls.time_usec()
    phil_actuator_controls.flags()
    phil_actuator_controls.controls(floatArrayOf(1.023875E38F, 2.9620991E37F, 2.5651522E38F, -2.8622751E38F, -8.34668E37F, -8.452471E37F, -2.5199947E38F, -1.315413E38F, -2.9826097E38F, 2.3049346E37F, 1.6408709E38F, -5.452359E37F, -3.3760752E38F, -1.2414485E38F, 1.7824008E37F, -3.1045017E38F))

    phil_actuator_controls.mode(MAV_MODE.MANUAL_ARMED)
}

fun onHIL_ACTUATOR_CONTROLS(phil_actuator_controls: com.company.demo.GroundControl.HIL_ACTUATOR_CONTROLS) {
    assert(phil_actuator_controls.time_usec())
    assert(phil_actuator_controls.flags())
    assert(phil_actuator_controls.controls().same(floatArrayOf(1.023875E38F, 2.9620991E37F, 2.5651522E38F, -2.8622751E38F, -8.34668E37F, -8.452471E37F, -2.5199947E38F, -1.315413E38F, -2.9826097E38F, 2.3049346E37F, 1.6408709E38F, -5.452359E37F, -3.3760752E38F, -1.2414485E38F, 1.7824008E37F, -3.1045017E38F)))

    assert(phil_actuator_controls.mode()!!.get() == MAV_MODE.MANUAL_ARMED)
    println("HIL_ACTUATOR_CONTROLS \n")
}

fun fill(pposition_target_local_ned: test_.POSITION_TARGET_LOCAL_NED) {

    pposition_target_local_ned.type_mask()
    pposition_target_local_ned.time_boot_ms()
    pposition_target_local_ned.x()
    pposition_target_local_ned.y()
    pposition_target_local_ned.z()
    pposition_target_local_ned.vx()
    pposition_target_local_ned.vy()
    pposition_target_local_ned.vz()
    pposition_target_local_ned.afx()
    pposition_target_local_ned.afy()
    pposition_target_local_ned.afz()
    pposition_target_local_ned.yaw()
    pposition_target_local_ned.yaw_rate()
    pposition_target_local_ned.coordinate_frame(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
}

fun onPOSITION_TARGET_LOCAL_NED(pposition_target_local_ned: com.company.demo.GroundControl.POSITION_TARGET_LOCAL_NED) {
    assert(pposition_target_local_ned.type_mask())
    assert(pposition_target_local_ned.time_boot_ms())
    assert(pposition_target_local_ned.x())
    assert(pposition_target_local_ned.y())
    assert(pposition_target_local_ned.z())
    assert(pposition_target_local_ned.vx())
    assert(pposition_target_local_ned.vy())
    assert(pposition_target_local_ned.vz())
    assert(pposition_target_local_ned.afx())
    assert(pposition_target_local_ned.afy())
    assert(pposition_target_local_ned.afz())
    assert(pposition_target_local_ned.yaw())
    assert(pposition_target_local_ned.yaw_rate())
    assert(pposition_target_local_ned.coordinate_frame()!!.get() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
    println("POSITION_TARGET_LOCAL_NED \n")
}

fun fill(pgimbal_report: com.company.demo.GroundControl.GIMBAL_REPORT) {

    pgimbal_report.target_system()
    pgimbal_report.target_component()
    pgimbal_report.delta_time()
    pgimbal_report.delta_angle_x()
    pgimbal_report.delta_angle_y()
    pgimbal_report.delta_angle_z()
    pgimbal_report.delta_velocity_x()
    pgimbal_report.delta_velocity_y()
    pgimbal_report.delta_velocity_z()
    pgimbal_report.joint_roll()
    pgimbal_report.joint_el()
    pgimbal_report.joint_az()
}

fun onGIMBAL_REPORT(pgimbal_report: com.company.demo.GroundControl.GIMBAL_REPORT) {
    assert(pgimbal_report.target_system())
    assert(pgimbal_report.target_component())
    assert(pgimbal_report.delta_time())
    assert(pgimbal_report.delta_angle_x())
    assert(pgimbal_report.delta_angle_y())
    assert(pgimbal_report.delta_angle_z())
    assert(pgimbal_report.delta_velocity_x())
    assert(pgimbal_report.delta_velocity_y())
    assert(pgimbal_report.delta_velocity_z())
    assert(pgimbal_report.joint_roll())
    assert(pgimbal_report.joint_el())
    assert(pgimbal_report.joint_az())
    println("GIMBAL_REPORT \n")
}

fun fill(pdevice_op_write: com.company.demo.GroundControl.DEVICE_OP_WRITE) {

    pdevice_op_write.request_id()
    pdevice_op_write.target_system()
    pdevice_op_write.target_component()
    pdevice_op_write.bus()
    pdevice_op_write.address()
    pdevice_op_write.regstart()
    pdevice_op_write.count();

    {
        var item = pdevice_op_write.daTa()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

    pdevice_op_write.bustype(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI)
    pdevice_op_write.busname("pacatgohwrlbVsdjoBvsa", null)

}

fun onDEVICE_OP_WRITE(pdevice_op_write: com.company.demo.GroundControl.DEVICE_OP_WRITE) {
    assert(pdevice_op_write.request_id())
    assert(pdevice_op_write.target_system())
    assert(pdevice_op_write.target_component())
    assert(pdevice_op_write.bus())
    assert(pdevice_op_write.address())
    assert(pdevice_op_write.regstart())
    assert(pdevice_op_write.count());
    {
        var item = pdevice_op_write.daTa()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    assert(pdevice_op_write.bustype()!!.get() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI)

    pdevice_op_write.busname()?.let { item ->
        assert(item.get() == "pacatgohwrlbVsdjoBvsa")
    } ?: throw RuntimeException("null")

    println("DEVICE_OP_WRITE \n")
}

fun fill(pdistance_sensor: com.company.demo.GroundControl.DISTANCE_SENSOR) {

    pdistance_sensor.min_distance()
    pdistance_sensor.max_distance()
    pdistance_sensor.current_distance()
    pdistance_sensor.time_boot_ms()
    pdistance_sensor.id()
    pdistance_sensor.covariance()
    pdistance_sensor.typE(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND)
    pdistance_sensor.orientation(MAV_SENSOR_ORIENTATION.PITCH_180_YAW_270)
}

fun onDISTANCE_SENSOR(pdistance_sensor: com.company.demo.GroundControl.DISTANCE_SENSOR) {
    assert(pdistance_sensor.min_distance())
    assert(pdistance_sensor.max_distance())
    assert(pdistance_sensor.current_distance())
    assert(pdistance_sensor.time_boot_ms())
    assert(pdistance_sensor.id())
    assert(pdistance_sensor.covariance())
    assert(pdistance_sensor.typE()!!.get() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND)
    assert(pdistance_sensor.orientation()!!.get() == MAV_SENSOR_ORIENTATION.PITCH_180_YAW_270)
    println("DISTANCE_SENSOR \n")
}

fun fill(phil_optical_flow: com.company.demo.GroundControl.HIL_OPTICAL_FLOW) {

    phil_optical_flow.integration_time_us()
    phil_optical_flow.time_delta_distance_us()
    phil_optical_flow.time_usec()
    phil_optical_flow.sensor_id()
    phil_optical_flow.integrated_x()
    phil_optical_flow.integrated_y()
    phil_optical_flow.integrated_xgyro()
    phil_optical_flow.integrated_ygyro()
    phil_optical_flow.integrated_zgyro()
    phil_optical_flow.temperature()
    phil_optical_flow.quality()
    phil_optical_flow.distance()
}

fun onHIL_OPTICAL_FLOW(phil_optical_flow: com.company.demo.GroundControl.HIL_OPTICAL_FLOW) {
    assert(phil_optical_flow.integration_time_us())
    assert(phil_optical_flow.time_delta_distance_us())
    assert(phil_optical_flow.time_usec())
    assert(phil_optical_flow.sensor_id())
    assert(phil_optical_flow.integrated_x())
    assert(phil_optical_flow.integrated_y())
    assert(phil_optical_flow.integrated_xgyro())
    assert(phil_optical_flow.integrated_ygyro())
    assert(phil_optical_flow.integrated_zgyro())
    assert(phil_optical_flow.temperature())
    assert(phil_optical_flow.quality())
    assert(phil_optical_flow.distance())
    println("HIL_OPTICAL_FLOW \n")
}

fun fill(pscaled_pressure2: com.company.demo.GroundControl.SCALED_PRESSURE2) {

    pscaled_pressure2.time_boot_ms()
    pscaled_pressure2.press_abs()
    pscaled_pressure2.press_diff()
    pscaled_pressure2.temperature()
}

fun onSCALED_PRESSURE2(pscaled_pressure2: com.company.demo.GroundControl.SCALED_PRESSURE2) {
    assert(pscaled_pressure2.time_boot_ms())
    assert(pscaled_pressure2.press_abs())
    assert(pscaled_pressure2.press_diff())
    assert(pscaled_pressure2.temperature())
    println("SCALED_PRESSURE2 \n")
}

fun fill(pwind_cov: com.company.demo.GroundControl.WIND_COV) {

    pwind_cov.time_usec()
    pwind_cov.wind_x()
    pwind_cov.wind_y()
    pwind_cov.wind_z()
    pwind_cov.var_horiz()
    pwind_cov.var_vert()
    pwind_cov.wind_alt()
    pwind_cov.horiz_accuracy()
    pwind_cov.vert_accuracy()
}

fun onWIND_COV(pwind_cov: com.company.demo.GroundControl.WIND_COV) {
    assert(pwind_cov.time_usec())
    assert(pwind_cov.wind_x())
    assert(pwind_cov.wind_y())
    assert(pwind_cov.wind_z())
    assert(pwind_cov.var_horiz())
    assert(pwind_cov.var_vert())
    assert(pwind_cov.wind_alt())
    assert(pwind_cov.horiz_accuracy())
    assert(pwind_cov.vert_accuracy())
    println("WIND_COV \n")
}

fun fill(pchange_operator_control: test_.CHANGE_OPERATOR_CONTROL) {

    pchange_operator_control.target_system()
    pchange_operator_control.control_request()
    pchange_operator_control.version()
    pchange_operator_control.passkey("kolhcycvrdcbdpjvujwcnnsozfw", null)

}

fun onCHANGE_OPERATOR_CONTROL(pchange_operator_control: com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL) {
    assert(pchange_operator_control.target_system())
    assert(pchange_operator_control.control_request())
    assert(pchange_operator_control.version())

    pchange_operator_control.passkey()?.let { item ->
        assert(item.get() == "kolhcycvrdcbdpjvujwcnnsozfw")
    } ?: throw RuntimeException("null")

    println("CHANGE_OPERATOR_CONTROL \n")
}

fun fill(pgopro_set_request: com.company.demo.GroundControl.GOPRO_SET_REQUEST) {

    pgopro_set_request.target_system()
    pgopro_set_request.target_component()
    pgopro_set_request.value(byteArrayOf(83, 120, 73, -92))

    pgopro_set_request.cmd_id(GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_SHARPNESS)
}

fun onGOPRO_SET_REQUEST(pgopro_set_request: com.company.demo.GroundControl.GOPRO_SET_REQUEST) {
    assert(pgopro_set_request.target_system())
    assert(pgopro_set_request.target_component())
    assert(pgopro_set_request.value().same(byteArrayOf(83, 120, 73, -92)))

    assert(pgopro_set_request.cmd_id()!!.get() == GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_SHARPNESS)
    println("GOPRO_SET_REQUEST \n")
}

fun fill(psys_status: test_.SYS_STATUS) {

    psys_status.load()
    psys_status.voltage_battery()
    psys_status.drop_rate_comm()
    psys_status.errors_comm()
    psys_status.errors_count1()
    psys_status.errors_count2()
    psys_status.errors_count3()
    psys_status.errors_count4()
    psys_status.current_battery()
    psys_status.battery_remaining()
    psys_status.onboard_control_sensors_present(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO)
    psys_status.onboard_control_sensors_enabled(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION)
    psys_status.onboard_control_sensors_health(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)
}

fun onSYS_STATUS(psys_status: com.company.demo.GroundControl.SYS_STATUS) {
    assert(psys_status.load())
    assert(psys_status.voltage_battery())
    assert(psys_status.drop_rate_comm())
    assert(psys_status.errors_comm())
    assert(psys_status.errors_count1())
    assert(psys_status.errors_count2())
    assert(psys_status.errors_count3())
    assert(psys_status.errors_count4())
    assert(psys_status.current_battery())
    assert(psys_status.battery_remaining())
    assert(psys_status.onboard_control_sensors_present()!!.get() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO)
    assert(psys_status.onboard_control_sensors_enabled()!!.get() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION)
    assert(psys_status.onboard_control_sensors_health()!!.get() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)
    println("SYS_STATUS \n")
}

fun fill(pmission_item: test_.MISSION_ITEM) {

    pmission_item.seq()
    pmission_item.target_system()
    pmission_item.target_component()
    pmission_item.current()
    pmission_item.autocontinue()
    pmission_item.param1()
    pmission_item.param2()
    pmission_item.param3()
    pmission_item.param4()
    pmission_item.x()
    pmission_item.y()
    pmission_item.z()
    pmission_item.frame(MAV_FRAME.MAV_FRAME_MISSION)
    pmission_item.command(MAV_CMD.MAV_CMD_NAV_FOLLOW)
    pmission_item.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION)
}

fun onMISSION_ITEM(pmission_item: com.company.demo.GroundControl.MISSION_ITEM) {
    assert(pmission_item.seq())
    assert(pmission_item.target_system())
    assert(pmission_item.target_component())
    assert(pmission_item.current())
    assert(pmission_item.autocontinue())
    assert(pmission_item.param1())
    assert(pmission_item.param2())
    assert(pmission_item.param3())
    assert(pmission_item.param4())
    assert(pmission_item.x())
    assert(pmission_item.y())
    assert(pmission_item.z())
    assert(pmission_item.frame()!!.get() == MAV_FRAME.MAV_FRAME_MISSION)
    assert(pmission_item.command()!!.get() == MAV_CMD.MAV_CMD_NAV_FOLLOW)
    assert(pmission_item.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION)
    println("MISSION_ITEM \n")
}

fun fill(praw_imu: test_.RAW_IMU) {

    praw_imu.time_usec()
    praw_imu.xacc()
    praw_imu.yacc()
    praw_imu.zacc()
    praw_imu.xgyro()
    praw_imu.ygyro()
    praw_imu.zgyro()
    praw_imu.xmag()
    praw_imu.ymag()
    praw_imu.zmag()
}

fun onRAW_IMU(praw_imu: com.company.demo.GroundControl.RAW_IMU) {
    assert(praw_imu.time_usec())
    assert(praw_imu.xacc())
    assert(praw_imu.yacc())
    assert(praw_imu.zacc())
    assert(praw_imu.xgyro())
    assert(praw_imu.ygyro())
    assert(praw_imu.zgyro())
    assert(praw_imu.xmag())
    assert(praw_imu.ymag())
    assert(praw_imu.zmag())
    println("RAW_IMU \n")
}

fun fill(pcommand_int: test_.COMMAND_INT) {

    pcommand_int.target_system()
    pcommand_int.target_component()
    pcommand_int.current()
    pcommand_int.autocontinue()
    pcommand_int.param1()
    pcommand_int.param2()
    pcommand_int.param3()
    pcommand_int.param4()
    pcommand_int.x()
    pcommand_int.y()
    pcommand_int.z()
    pcommand_int.frame(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
    pcommand_int.command(MAV_CMD.MAV_CMD_DO_FOLLOW)
}

fun onCOMMAND_INT(pcommand_int: com.company.demo.GroundControl.COMMAND_INT) {
    assert(pcommand_int.target_system())
    assert(pcommand_int.target_component())
    assert(pcommand_int.current())
    assert(pcommand_int.autocontinue())
    assert(pcommand_int.param1())
    assert(pcommand_int.param2())
    assert(pcommand_int.param3())
    assert(pcommand_int.param4())
    assert(pcommand_int.x())
    assert(pcommand_int.y())
    assert(pcommand_int.z())
    assert(pcommand_int.frame()!!.get() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
    assert(pcommand_int.command()!!.get() == MAV_CMD.MAV_CMD_DO_FOLLOW)
    println("COMMAND_INT \n")
}

fun fill(poptical_flow: test_.OPTICAL_FLOW) {

    poptical_flow.time_usec()
    poptical_flow.sensor_id()
    poptical_flow.flow_x()
    poptical_flow.flow_y()
    poptical_flow.flow_comp_m_x()
    poptical_flow.flow_comp_m_y()
    poptical_flow.quality()
    poptical_flow.ground_distance()
    poptical_flow.flow_rate_x()
    poptical_flow.flow_rate_y()
}

fun onOPTICAL_FLOW(poptical_flow: com.company.demo.GroundControl.OPTICAL_FLOW) {
    assert(poptical_flow.time_usec())
    assert(poptical_flow.sensor_id())
    assert(poptical_flow.flow_x())
    assert(poptical_flow.flow_y())
    assert(poptical_flow.flow_comp_m_x())
    assert(poptical_flow.flow_comp_m_y())
    assert(poptical_flow.quality())
    assert(poptical_flow.ground_distance())
    assert(poptical_flow.flow_rate_x())
    assert(poptical_flow.flow_rate_y())
    println("OPTICAL_FLOW \n")
}

fun fill(pmission_item_int: test_.MISSION_ITEM_INT) {

    pmission_item_int.seq()
    pmission_item_int.target_system()
    pmission_item_int.target_component()
    pmission_item_int.current()
    pmission_item_int.autocontinue()
    pmission_item_int.param1()
    pmission_item_int.param2()
    pmission_item_int.param3()
    pmission_item_int.param4()
    pmission_item_int.x()
    pmission_item_int.y()
    pmission_item_int.z()
    pmission_item_int.frame(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
    pmission_item_int.command(MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE)
    pmission_item_int.mission_type(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL)
}

fun onMISSION_ITEM_INT(pmission_item_int: com.company.demo.GroundControl.MISSION_ITEM_INT) {
    assert(pmission_item_int.seq())
    assert(pmission_item_int.target_system())
    assert(pmission_item_int.target_component())
    assert(pmission_item_int.current())
    assert(pmission_item_int.autocontinue())
    assert(pmission_item_int.param1())
    assert(pmission_item_int.param2())
    assert(pmission_item_int.param3())
    assert(pmission_item_int.param4())
    assert(pmission_item_int.x())
    assert(pmission_item_int.y())
    assert(pmission_item_int.z())
    assert(pmission_item_int.frame()!!.get() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED)
    assert(pmission_item_int.command()!!.get() == MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE)
    assert(pmission_item_int.mission_type()!!.get() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL)
    println("MISSION_ITEM_INT \n")
}

fun fill(pvision_position_delta: com.company.demo.GroundControl.VISION_POSITION_DELTA) {

    pvision_position_delta.time_usec()
    pvision_position_delta.time_delta_usec()
    pvision_position_delta.angle_delta(floatArrayOf(-3.2935047E38F, 1.438737E38F, 1.4723528E38F))

    pvision_position_delta.position_delta(floatArrayOf(2.787692E38F, -3.252436E38F, 7.016845E37F))

    pvision_position_delta.confidence()
}

fun onVISION_POSITION_DELTA(pvision_position_delta: com.company.demo.GroundControl.VISION_POSITION_DELTA) {
    assert(pvision_position_delta.time_usec())
    assert(pvision_position_delta.time_delta_usec())
    assert(pvision_position_delta.angle_delta().same(floatArrayOf(-3.2935047E38F, 1.438737E38F, 1.4723528E38F)))

    assert(pvision_position_delta.position_delta().same(floatArrayOf(2.787692E38F, -3.252436E38F, 7.016845E37F)))

    assert(pvision_position_delta.confidence())
    println("VISION_POSITION_DELTA \n")
}

fun fill(plogging_data: com.company.demo.GroundControl.LOGGING_DATA) {

    plogging_data.sequence()
    plogging_data.target_system()
    plogging_data.target_component()
    plogging_data.length()
    plogging_data.first_message_offset();

    {
        var item = plogging_data.daTa()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

}

fun onLOGGING_DATA(plogging_data: com.company.demo.GroundControl.LOGGING_DATA) {
    assert(plogging_data.sequence())
    assert(plogging_data.target_system())
    assert(plogging_data.target_component())
    assert(plogging_data.length())
    assert(plogging_data.first_message_offset());
    {
        var item = plogging_data.daTa()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    println("LOGGING_DATA \n")
}

fun fill(pdevice_op_read: com.company.demo.GroundControl.DEVICE_OP_READ) {

    pdevice_op_read.request_id()
    pdevice_op_read.target_system()
    pdevice_op_read.target_component()
    pdevice_op_read.bus()
    pdevice_op_read.address()
    pdevice_op_read.regstart()
    pdevice_op_read.count()
    pdevice_op_read.bustype(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C)
    pdevice_op_read.busname("yhh", null)

}

fun onDEVICE_OP_READ(pdevice_op_read: com.company.demo.GroundControl.DEVICE_OP_READ) {
    assert(pdevice_op_read.request_id())
    assert(pdevice_op_read.target_system())
    assert(pdevice_op_read.target_component())
    assert(pdevice_op_read.bus())
    assert(pdevice_op_read.address())
    assert(pdevice_op_read.regstart())
    assert(pdevice_op_read.count())
    assert(pdevice_op_read.bustype()!!.get() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C)

    pdevice_op_read.busname()?.let { item ->
        assert(item.get() == "yhh")
    } ?: throw RuntimeException("null")

    println("DEVICE_OP_READ \n")
}

fun fill(pmag_cal_progress: com.company.demo.GroundControl.MAG_CAL_PROGRESS) {

    pmag_cal_progress.compass_id()
    pmag_cal_progress.cal_mask()
    pmag_cal_progress.attempt()
    pmag_cal_progress.completion_pct()
    pmag_cal_progress.completion_mask(byteArrayOf(98, 86, 95, 7, -33, 0, -95, 110, 57, -123))

    pmag_cal_progress.direction_x()
    pmag_cal_progress.direction_y()
    pmag_cal_progress.direction_z()
    pmag_cal_progress.cal_status(MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE)
}

fun onMAG_CAL_PROGRESS(pmag_cal_progress: com.company.demo.GroundControl.MAG_CAL_PROGRESS) {
    assert(pmag_cal_progress.compass_id())
    assert(pmag_cal_progress.cal_mask())
    assert(pmag_cal_progress.attempt())
    assert(pmag_cal_progress.completion_pct())
    assert(pmag_cal_progress.completion_mask().same(byteArrayOf(98, 86, 95, 7, -33, 0, -95, 110, 57, -123)))

    assert(pmag_cal_progress.direction_x())
    assert(pmag_cal_progress.direction_y())
    assert(pmag_cal_progress.direction_z())
    assert(pmag_cal_progress.cal_status()!!.get() == MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE)
    println("MAG_CAL_PROGRESS \n")
}

fun fill(phighres_imu: com.company.demo.GroundControl.HIGHRES_IMU) {

    phighres_imu.fields_updated()
    phighres_imu.time_usec()
    phighres_imu.xacc()
    phighres_imu.yacc()
    phighres_imu.zacc()
    phighres_imu.xgyro()
    phighres_imu.ygyro()
    phighres_imu.zgyro()
    phighres_imu.xmag()
    phighres_imu.ymag()
    phighres_imu.zmag()
    phighres_imu.abs_pressure()
    phighres_imu.diff_pressure()
    phighres_imu.pressure_alt()
    phighres_imu.temperature()
}

fun onHIGHRES_IMU(phighres_imu: com.company.demo.GroundControl.HIGHRES_IMU) {
    assert(phighres_imu.fields_updated())
    assert(phighres_imu.time_usec())
    assert(phighres_imu.xacc())
    assert(phighres_imu.yacc())
    assert(phighres_imu.zacc())
    assert(phighres_imu.xgyro())
    assert(phighres_imu.ygyro())
    assert(phighres_imu.zgyro())
    assert(phighres_imu.xmag())
    assert(phighres_imu.ymag())
    assert(phighres_imu.zmag())
    assert(phighres_imu.abs_pressure())
    assert(phighres_imu.diff_pressure())
    assert(phighres_imu.pressure_alt())
    assert(phighres_imu.temperature())
    println("HIGHRES_IMU \n")
}

fun fill(pextended_sys_state: com.company.demo.GroundControl.EXTENDED_SYS_STATE) {

    pextended_sys_state.vtol_state(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW)
    pextended_sys_state.landed_state(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR)
}

fun onEXTENDED_SYS_STATE(pextended_sys_state: com.company.demo.GroundControl.EXTENDED_SYS_STATE) {
    assert(pextended_sys_state.vtol_state()!!.get() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW)
    assert(pextended_sys_state.landed_state()!!.get() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR)
    println("EXTENDED_SYS_STATE \n")
}

fun fill(puavionix_adsb_out_dynamic: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC) {

    puavionix_adsb_out_dynamic.accuracyVert()
    puavionix_adsb_out_dynamic.accuracyVel()
    puavionix_adsb_out_dynamic.squawk()
    puavionix_adsb_out_dynamic.utcTime()
    puavionix_adsb_out_dynamic.accuracyHor()
    puavionix_adsb_out_dynamic.gpsLat()
    puavionix_adsb_out_dynamic.gpsLon()
    puavionix_adsb_out_dynamic.gpsAlt()
    puavionix_adsb_out_dynamic.numSats()
    puavionix_adsb_out_dynamic.baroAltMSL()
    puavionix_adsb_out_dynamic.velVert()
    puavionix_adsb_out_dynamic.velNS()
    puavionix_adsb_out_dynamic.VelEW()
    puavionix_adsb_out_dynamic.gpsFix(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS)
    puavionix_adsb_out_dynamic.emergencyStatus(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_RESERVED)
    puavionix_adsb_out_dynamic.state(UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT)
}

fun onUAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC) {
    assert(puavionix_adsb_out_dynamic.accuracyVert())
    assert(puavionix_adsb_out_dynamic.accuracyVel())
    assert(puavionix_adsb_out_dynamic.squawk())
    assert(puavionix_adsb_out_dynamic.utcTime())
    assert(puavionix_adsb_out_dynamic.accuracyHor())
    assert(puavionix_adsb_out_dynamic.gpsLat())
    assert(puavionix_adsb_out_dynamic.gpsLon())
    assert(puavionix_adsb_out_dynamic.gpsAlt())
    assert(puavionix_adsb_out_dynamic.numSats())
    assert(puavionix_adsb_out_dynamic.baroAltMSL())
    assert(puavionix_adsb_out_dynamic.velVert())
    assert(puavionix_adsb_out_dynamic.velNS())
    assert(puavionix_adsb_out_dynamic.VelEW())
    assert(puavionix_adsb_out_dynamic.gpsFix()!!.get() == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS)
    assert(puavionix_adsb_out_dynamic.emergencyStatus()!!.get() == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_RESERVED)
    assert(puavionix_adsb_out_dynamic.state()!!.get() == UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT)
    println("UAVIONIX_ADSB_OUT_DYNAMIC \n")
}

fun fill(pgopro_get_response: com.company.demo.GroundControl.GOPRO_GET_RESPONSE) {

    pgopro_get_response.value(byteArrayOf(125, -95, 36, -46))

    pgopro_get_response.cmd_id(GOPRO_COMMAND.GOPRO_COMMAND_MODEL)
    pgopro_get_response.status(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED)
}

fun onGOPRO_GET_RESPONSE(pgopro_get_response: com.company.demo.GroundControl.GOPRO_GET_RESPONSE) {
    assert(pgopro_get_response.value().same(byteArrayOf(125, -95, 36, -46)))

    assert(pgopro_get_response.cmd_id()!!.get() == GOPRO_COMMAND.GOPRO_COMMAND_MODEL)
    assert(pgopro_get_response.status()!!.get() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED)
    println("GOPRO_GET_RESPONSE \n")
}

fun fill(pgps_inject_data: com.company.demo.GroundControl.GPS_INJECT_DATA) {

    pgps_inject_data.target_system()
    pgps_inject_data.target_component()
    pgps_inject_data.len();

    {
        var item = pgps_inject_data.daTa()
        for (i in 0 until item.len())
            item.set((Byte)(i * 2 + -128), i)
    }

}

fun onGPS_INJECT_DATA(pgps_inject_data: com.company.demo.GroundControl.GPS_INJECT_DATA) {
    assert(pgps_inject_data.target_system())
    assert(pgps_inject_data.target_component())
    assert(pgps_inject_data.len());
    {
        var item = pgps_inject_data.daTa()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 2 + -128))

    }

    println("GPS_INJECT_DATA \n")
}

fun fill(puavionix_adsb_transceiver_health_report: com.company.demo.GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) {

    puavionix_adsb_transceiver_health_report.rfHealth(UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING)
}

fun onUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report: com.company.demo.GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) {
    assert(puavionix_adsb_transceiver_health_report.rfHealth()!!.get() == UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING)
    println("UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT \n")
}

fun fill(pattitude_quaternion_cov: test_.ATTITUDE_QUATERNION_COV) {

    pattitude_quaternion_cov.time_usec()
    pattitude_quaternion_cov.q(floatArrayOf(2.341848E38F, -3.1169924E38F, 2.5877136E38F, -2.6616209E38F))

    pattitude_quaternion_cov.rollspeed()
    pattitude_quaternion_cov.pitchspeed()
    pattitude_quaternion_cov.yawspeed()
    pattitude_quaternion_cov.covariance(floatArrayOf(-2.1838953E38F, 2.6150474E38F, -2.4832841E38F, 7.841296E37F, 2.8208136E38F, -1.41432E38F, 2.5149191E36F, -2.7929973E38F, -3.3073479E38F))

}

fun onATTITUDE_QUATERNION_COV(pattitude_quaternion_cov: com.company.demo.GroundControl.ATTITUDE_QUATERNION_COV) {
    assert(pattitude_quaternion_cov.time_usec())
    assert(pattitude_quaternion_cov.q().same(floatArrayOf(2.341848E38F, -3.1169924E38F, 2.5877136E38F, -2.6616209E38F)))

    assert(pattitude_quaternion_cov.rollspeed())
    assert(pattitude_quaternion_cov.pitchspeed())
    assert(pattitude_quaternion_cov.yawspeed())
    assert(pattitude_quaternion_cov.covariance().same(floatArrayOf(-2.1838953E38F, 2.6150474E38F, -2.4832841E38F, 7.841296E37F, 2.8208136E38F, -1.41432E38F, 2.5149191E36F, -2.7929973E38F, -3.3073479E38F)))

    println("ATTITUDE_QUATERNION_COV \n")
}

fun fill(pnamed_value_int: com.company.demo.GroundControl.NAMED_VALUE_INT) {

    pnamed_value_int.time_boot_ms()
    pnamed_value_int.value()
    pnamed_value_int.name("dfxgkfzhnsaDganrdwirwchaxeBKcizjkHsrfg", null)

}

fun onNAMED_VALUE_INT(pnamed_value_int: com.company.demo.GroundControl.NAMED_VALUE_INT) {
    assert(pnamed_value_int.time_boot_ms())
    assert(pnamed_value_int.value())

    pnamed_value_int.name()?.let { item ->
        assert(item.get() == "dfxgkfzhnsaDganrdwirwchaxeBKcizjkHsrfg")
    } ?: throw RuntimeException("null")

    println("NAMED_VALUE_INT \n")
}

fun fill(prpm: com.company.demo.GroundControl.RPM) {

    prpm.rpm1()
    prpm.rpm2()
}

fun onRPM(prpm: com.company.demo.GroundControl.RPM) {
    assert(prpm.rpm1())
    assert(prpm.rpm2())
    println("RPM \n")
}

fun fill(pgps_rtcm_data: com.company.demo.GroundControl.GPS_RTCM_DATA) {

    pgps_rtcm_data.flags()
    pgps_rtcm_data.len();

    {
        var item = pgps_rtcm_data.daTa()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

}

fun onGPS_RTCM_DATA(pgps_rtcm_data: com.company.demo.GroundControl.GPS_RTCM_DATA) {
    assert(pgps_rtcm_data.flags())
    assert(pgps_rtcm_data.len());
    {
        var item = pgps_rtcm_data.daTa()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    println("GPS_RTCM_DATA \n")
}

fun fill(pglobal_vision_position_estimate: test_.GLOBAL_VISION_POSITION_ESTIMATE) {

    pglobal_vision_position_estimate.usec()
    pglobal_vision_position_estimate.x()
    pglobal_vision_position_estimate.y()
    pglobal_vision_position_estimate.z()
    pglobal_vision_position_estimate.roll()
    pglobal_vision_position_estimate.pitch()
    pglobal_vision_position_estimate.yaw()
}

fun onGLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate: com.company.demo.GroundControl.GLOBAL_VISION_POSITION_ESTIMATE) {
    assert(pglobal_vision_position_estimate.usec())
    assert(pglobal_vision_position_estimate.x())
    assert(pglobal_vision_position_estimate.y())
    assert(pglobal_vision_position_estimate.z())
    assert(pglobal_vision_position_estimate.roll())
    assert(pglobal_vision_position_estimate.pitch())
    assert(pglobal_vision_position_estimate.yaw())
    println("GLOBAL_VISION_POSITION_ESTIMATE \n")
}

fun fill(pfile_transfer_protocol: com.company.demo.GroundControl.FILE_TRANSFER_PROTOCOL) {

    pfile_transfer_protocol.target_network()
    pfile_transfer_protocol.target_system()
    pfile_transfer_protocol.target_component();

    {
        var item = pfile_transfer_protocol.payload()
        for (i in 0 until item.len())
            item.set((Byte)(i * 1 + -128), i)
    }

}

fun onFILE_TRANSFER_PROTOCOL(pfile_transfer_protocol: com.company.demo.GroundControl.FILE_TRANSFER_PROTOCOL) {
    assert(pfile_transfer_protocol.target_network())
    assert(pfile_transfer_protocol.target_system())
    assert(pfile_transfer_protocol.target_component());
    {
        var item = pfile_transfer_protocol.payload()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 1 + -128))

    }

    println("FILE_TRANSFER_PROTOCOL \n")
}

fun fill(prangefinder: com.company.demo.GroundControl.RANGEFINDER) {

    prangefinder.distance()
    prangefinder.voltage()
}

fun onRANGEFINDER(prangefinder: com.company.demo.GroundControl.RANGEFINDER) {
    assert(prangefinder.distance())
    assert(prangefinder.voltage())
    println("RANGEFINDER \n")
}

fun fill(pradio_status: com.company.demo.GroundControl.RADIO_STATUS) {

    pradio_status.rxerrors()
    pradio_status.fixeD()
    pradio_status.rssi()
    pradio_status.remrssi()
    pradio_status.txbuf()
    pradio_status.noise()
    pradio_status.remnoise()
}

fun onRADIO_STATUS(pradio_status: com.company.demo.GroundControl.RADIO_STATUS) {
    assert(pradio_status.rxerrors())
    assert(pradio_status.fixeD())
    assert(pradio_status.rssi())
    assert(pradio_status.remrssi())
    assert(pradio_status.txbuf())
    assert(pradio_status.noise())
    assert(pradio_status.remnoise())
    println("RADIO_STATUS \n")
}

fun fill(pfence_point: com.company.demo.GroundControl.FENCE_POINT) {

    pfence_point.target_system()
    pfence_point.target_component()
    pfence_point.idx()
    pfence_point.count()
    pfence_point.lat()
    pfence_point.lng()
}

fun onFENCE_POINT(pfence_point: com.company.demo.GroundControl.FENCE_POINT) {
    assert(pfence_point.target_system())
    assert(pfence_point.target_component())
    assert(pfence_point.idx())
    assert(pfence_point.count())
    assert(pfence_point.lat())
    assert(pfence_point.lng())
    println("FENCE_POINT \n")
}

fun fill(presource_request: com.company.demo.GroundControl.RESOURCE_REQUEST) {

    presource_request.request_id()
    presource_request.uri_type();

    {
        var item = presource_request.uri()
        for (i in 0 until item.len())
            item.set((Byte)(i * 2 + -128), i)
    }

    presource_request.transfer_type();

    {
        var item = presource_request.storage()
        for (i in 0 until item.len())
            item.set((Byte)(i * 2 + -128), i)
    }

}

fun onRESOURCE_REQUEST(presource_request: com.company.demo.GroundControl.RESOURCE_REQUEST) {
    assert(presource_request.request_id())
    assert(presource_request.uri_type());
    {
        var item = presource_request.uri()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 2 + -128))

    }

    assert(presource_request.transfer_type());
    {
        var item = presource_request.storage()

        for (i in 0 until item.len())
            assert(item.get(i) == (Byte)(i * 2 + -128))

    }

    println("RESOURCE_REQUEST \n")
}


class CommunicationChannel_test : CommunicationChannel() {
    override fun on_RESOURCE_REQUEST(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val presource_request = com.company.demo.GroundControl.RESOURCE_REQUEST(bytes)
        onRESOURCE_REQUEST(presource_request)
    }

    override fun on_ATTITUDE_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pattitude_target = com.company.demo.GroundControl.ATTITUDE_TARGET(bytes)
        onATTITUDE_TARGET(pattitude_target)
    }

    override fun on_MISSION_COUNT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_count = com.company.demo.GroundControl.MISSION_COUNT(cur)
        onMISSION_COUNT(pmission_count)
    }

    override fun on_ADSB_VEHICLE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val padsb_vehicle = com.company.demo.GroundControl.ADSB_VEHICLE(cur)
        onADSB_VEHICLE(padsb_vehicle)
    }

    override fun on_MESSAGE_INTERVAL(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmessage_interval = com.company.demo.GroundControl.MESSAGE_INTERVAL(bytes)
        onMESSAGE_INTERVAL(pmessage_interval)
    }

    override fun on_ESTIMATOR_STATUS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pestimator_status = com.company.demo.GroundControl.ESTIMATOR_STATUS(cur)
        onESTIMATOR_STATUS(pestimator_status)
    }

    override fun on_TIMESYNC(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val ptimesync = com.company.demo.GroundControl.TIMESYNC(bytes)
        onTIMESYNC(ptimesync)
    }

    override fun on_GLOBAL_POSITION_INT_COV(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pglobal_position_int_cov = com.company.demo.GroundControl.GLOBAL_POSITION_INT_COV(cur)
        onGLOBAL_POSITION_INT_COV(pglobal_position_int_cov)
    }

    override fun on_BUTTON_CHANGE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pbutton_change = com.company.demo.GroundControl.BUTTON_CHANGE(bytes)
        onBUTTON_CHANGE(pbutton_change)
    }

    override fun on_SAFETY_SET_ALLOWED_AREA(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val psafety_set_allowed_area = com.company.demo.GroundControl.SAFETY_SET_ALLOWED_AREA(cur)
        onSAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area)
    }

    override fun on_STORAGE_INFORMATION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pstorage_information = com.company.demo.GroundControl.STORAGE_INFORMATION(bytes)
        onSTORAGE_INFORMATION(pstorage_information)
    }

    override fun on_COLLISION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcollision = com.company.demo.GroundControl.COLLISION(cur)
        onCOLLISION(pcollision)
    }

    override fun on_ALTITUDE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val paltitude = com.company.demo.GroundControl.ALTITUDE(bytes)
        onALTITUDE(paltitude)
    }

    override fun on_HIL_STATE_QUATERNION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_state_quaternion = com.company.demo.GroundControl.HIL_STATE_QUATERNION(bytes)
        onHIL_STATE_QUATERNION(phil_state_quaternion)
    }

    override fun on_CAMERA_INFORMATION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcamera_information = com.company.demo.GroundControl.CAMERA_INFORMATION(cur)
        onCAMERA_INFORMATION(pcamera_information)
    }

    override fun on_GPS_STATUS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps_status = com.company.demo.GroundControl.GPS_STATUS(bytes)
        onGPS_STATUS(pgps_status)
    }

    override fun on_PARAM_SET(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pparam_set = com.company.demo.GroundControl.PARAM_SET(cur)
        onPARAM_SET(pparam_set)
    }

    override fun on_TERRAIN_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pterrain_data = com.company.demo.GroundControl.TERRAIN_DATA(bytes)
        onTERRAIN_DATA(pterrain_data)
    }

    override fun on_RC_CHANNELS_OVERRIDE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prc_channels_override = com.company.demo.GroundControl.RC_CHANNELS_OVERRIDE(bytes)
        onRC_CHANNELS_OVERRIDE(prc_channels_override)
    }

    override fun on_SCALED_IMU(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_imu = com.company.demo.GroundControl.SCALED_IMU(bytes)
        onSCALED_IMU(pscaled_imu)
    }

    override fun on_DEBUG(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pdebug = com.company.demo.GroundControl.DEBUG(bytes)
        onDEBUG(pdebug)
    }

    override fun on_CAMERA_IMAGE_CAPTURED(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcamera_image_captured = com.company.demo.GroundControl.CAMERA_IMAGE_CAPTURED(cur)
        onCAMERA_IMAGE_CAPTURED(pcamera_image_captured)
    }

    override fun on_LOG_ENTRY(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_entry = com.company.demo.GroundControl.LOG_ENTRY(bytes)
        onLOG_ENTRY(plog_entry)
    }

    override fun on_ACTUATOR_CONTROL_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pactuator_control_target = com.company.demo.GroundControl.ACTUATOR_CONTROL_TARGET(bytes)
        onACTUATOR_CONTROL_TARGET(pactuator_control_target)
    }

    override fun on_HIGH_LATENCY(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val phigh_latency = com.company.demo.GroundControl.HIGH_LATENCY(cur)
        onHIGH_LATENCY(phigh_latency)
    }

    override fun on_PARAM_REQUEST_READ(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pparam_request_read = com.company.demo.GroundControl.PARAM_REQUEST_READ(cur)
        onPARAM_REQUEST_READ(pparam_request_read)
    }

    override fun on_SET_ATTITUDE_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pset_attitude_target = com.company.demo.GroundControl.SET_ATTITUDE_TARGET(bytes)
        onSET_ATTITUDE_TARGET(pset_attitude_target)
    }

    override fun on_FOLLOW_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pfollow_target = com.company.demo.GroundControl.FOLLOW_TARGET(bytes)
        onFOLLOW_TARGET(pfollow_target)
    }

    override fun on_HIL_STATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_state = com.company.demo.GroundControl.HIL_STATE(bytes)
        onHIL_STATE(phil_state)
    }

    override fun on_HOME_POSITION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val phome_position = com.company.demo.GroundControl.HOME_POSITION(cur)
        onHOME_POSITION(phome_position)
    }

    override fun on_GPS2_RAW(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pgps2_raw = com.company.demo.GroundControl.GPS2_RAW(cur)
        onGPS2_RAW(pgps2_raw)
    }

    override fun on_MEMORY_VECT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmemory_vect = com.company.demo.GroundControl.MEMORY_VECT(bytes)
        onMEMORY_VECT(pmemory_vect)
    }

    override fun on_REQUEST_DATA_STREAM(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prequest_data_stream = com.company.demo.GroundControl.REQUEST_DATA_STREAM(bytes)
        onREQUEST_DATA_STREAM(prequest_data_stream)
    }

    override fun on_HIL_CONTROLS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val phil_controls = com.company.demo.GroundControl.HIL_CONTROLS(cur)
        onHIL_CONTROLS(phil_controls)
    }

    override fun on_HIL_SENSOR(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_sensor = com.company.demo.GroundControl.HIL_SENSOR(bytes)
        onHIL_SENSOR(phil_sensor)
    }

    override fun on_SETUP_SIGNING(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val psetup_signing = com.company.demo.GroundControl.SETUP_SIGNING(bytes)
        onSETUP_SIGNING(psetup_signing)
    }

    override fun on_GPS_RTK(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps_rtk = com.company.demo.GroundControl.GPS_RTK(bytes)
        onGPS_RTK(pgps_rtk)
    }

    override fun on_PARAM_REQUEST_LIST(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pparam_request_list = com.company.demo.GroundControl.PARAM_REQUEST_LIST(bytes)
        onPARAM_REQUEST_LIST(pparam_request_list)
    }

    override fun on_LANDING_TARGET(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val planding_target = com.company.demo.GroundControl.LANDING_TARGET(cur)
        onLANDING_TARGET(planding_target)
    }

    override fun on_SET_ACTUATOR_CONTROL_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pset_actuator_control_target = com.company.demo.GroundControl.SET_ACTUATOR_CONTROL_TARGET(bytes)
        onSET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target)
    }

    override fun on_CONTROL_SYSTEM_STATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pcontrol_system_state = com.company.demo.GroundControl.CONTROL_SYSTEM_STATE(bytes)
        onCONTROL_SYSTEM_STATE(pcontrol_system_state)
    }

    override fun on_SET_POSITION_TARGET_GLOBAL_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_position_target_global_int = com.company.demo.GroundControl.SET_POSITION_TARGET_GLOBAL_INT(cur)
        onSET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int)
    }

    override fun on_VIBRATION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvibration = com.company.demo.GroundControl.VIBRATION(bytes)
        onVIBRATION(pvibration)
    }

    override fun on_PING33(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pping33 = com.company.demo.GroundControl.PING33(cur)
        onPING33(pping33)
    }

    override fun on_VFR_HUD(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvfr_hud = com.company.demo.GroundControl.VFR_HUD(bytes)
        onVFR_HUD(pvfr_hud)
    }

    override fun on_MISSION_SET_CURRENT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmission_set_current = com.company.demo.GroundControl.MISSION_SET_CURRENT(bytes)
        onMISSION_SET_CURRENT(pmission_set_current)
    }

    override fun on_HIL_GPS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_gps = com.company.demo.GroundControl.HIL_GPS(bytes)
        onHIL_GPS(phil_gps)
    }

    override fun on_NAV_CONTROLLER_OUTPUT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pnav_controller_output = com.company.demo.GroundControl.NAV_CONTROLLER_OUTPUT(bytes)
        onNAV_CONTROLLER_OUTPUT(pnav_controller_output)
    }

    override fun on_AUTH_KEY(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pauth_key = com.company.demo.GroundControl.AUTH_KEY(cur)
        onAUTH_KEY(pauth_key)
    }

    override fun on_LOCAL_POSITION_NED_COV(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val plocal_position_ned_cov = com.company.demo.GroundControl.LOCAL_POSITION_NED_COV(cur)
        onLOCAL_POSITION_NED_COV(plocal_position_ned_cov)
    }

    override fun on_ATT_POS_MOCAP(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val patt_pos_mocap = com.company.demo.GroundControl.ATT_POS_MOCAP(bytes)
        onATT_POS_MOCAP(patt_pos_mocap)
    }

    override fun on_STATUSTEXT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pstatustext = com.company.demo.GroundControl.STATUSTEXT(cur)
        onSTATUSTEXT(pstatustext)
    }

    override fun on_PING(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pping = com.company.demo.GroundControl.PING(bytes)
        onPING(pping)
    }

    override fun on_CAMERA_CAPTURE_STATUS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pcamera_capture_status = com.company.demo.GroundControl.CAMERA_CAPTURE_STATUS(bytes)
        onCAMERA_CAPTURE_STATUS(pcamera_capture_status)
    }

    override fun on_GLOBAL_POSITION_INT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pglobal_position_int = com.company.demo.GroundControl.GLOBAL_POSITION_INT(bytes)
        onGLOBAL_POSITION_INT(pglobal_position_int)
    }

    override fun on_ENCAPSULATED_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pencapsulated_data = com.company.demo.GroundControl.ENCAPSULATED_DATA(bytes)
        onENCAPSULATED_DATA(pencapsulated_data)
    }

    override fun on_GPS_INPUT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pgps_input = com.company.demo.GroundControl.GPS_INPUT(cur)
        onGPS_INPUT(pgps_input)
    }

    override fun on_COMMAND_LONG(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcommand_long = com.company.demo.GroundControl.COMMAND_LONG(cur)
        onCOMMAND_LONG(pcommand_long)
    }

    override fun on_LOG_REQUEST_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_request_data = com.company.demo.GroundControl.LOG_REQUEST_DATA(bytes)
        onLOG_REQUEST_DATA(plog_request_data)
    }

    override fun on_GPS_RAW_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pgps_raw_int = com.company.demo.GroundControl.GPS_RAW_INT(cur)
        onGPS_RAW_INT(pgps_raw_int)
    }

    override fun on_RC_CHANNELS_SCALED(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prc_channels_scaled = com.company.demo.GroundControl.RC_CHANNELS_SCALED(bytes)
        onRC_CHANNELS_SCALED(prc_channels_scaled)
    }

    override fun on_CAMERA_SETTINGS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcamera_settings = com.company.demo.GroundControl.CAMERA_SETTINGS(cur)
        onCAMERA_SETTINGS(pcamera_settings)
    }

    override fun on_RAW_PRESSURE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val praw_pressure = com.company.demo.GroundControl.RAW_PRESSURE(bytes)
        onRAW_PRESSURE(praw_pressure)
    }

    override fun on_NAMED_VALUE_FLOAT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pnamed_value_float = com.company.demo.GroundControl.NAMED_VALUE_FLOAT(cur)
        onNAMED_VALUE_FLOAT(pnamed_value_float)
    }

    override fun on_ATTITUDE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pattitude = com.company.demo.GroundControl.ATTITUDE(bytes)
        onATTITUDE(pattitude)
    }

    override fun on_TERRAIN_REQUEST(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pterrain_request = com.company.demo.GroundControl.TERRAIN_REQUEST(bytes)
        onTERRAIN_REQUEST(pterrain_request)
    }

    override fun on_MISSION_WRITE_PARTIAL_LIST(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_write_partial_list = com.company.demo.GroundControl.MISSION_WRITE_PARTIAL_LIST(cur)
        onMISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list)
    }

    override fun on_LOG_ERASE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_erase = com.company.demo.GroundControl.LOG_ERASE(bytes)
        onLOG_ERASE(plog_erase)
    }

    override fun on_MANUAL_SETPOINT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmanual_setpoint = com.company.demo.GroundControl.MANUAL_SETPOINT(bytes)
        onMANUAL_SETPOINT(pmanual_setpoint)
    }

    override fun on_SAFETY_ALLOWED_AREA(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val psafety_allowed_area = com.company.demo.GroundControl.SAFETY_ALLOWED_AREA(cur)
        onSAFETY_ALLOWED_AREA(psafety_allowed_area)
    }

    override fun on_OPTICAL_FLOW_RAD(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val poptical_flow_rad = com.company.demo.GroundControl.OPTICAL_FLOW_RAD(bytes)
        onOPTICAL_FLOW_RAD(poptical_flow_rad)
    }

    override fun on_LOG_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_data = com.company.demo.GroundControl.LOG_DATA(bytes)
        onLOG_DATA(plog_data)
    }

    override fun on_MISSION_CLEAR_ALL(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_clear_all = com.company.demo.GroundControl.MISSION_CLEAR_ALL(cur)
        onMISSION_CLEAR_ALL(pmission_clear_all)
    }

    override fun on_VICON_POSITION_ESTIMATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvicon_position_estimate = com.company.demo.GroundControl.VICON_POSITION_ESTIMATE(bytes)
        onVICON_POSITION_ESTIMATE(pvicon_position_estimate)
    }

    override fun on_GPS2_RTK(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps2_rtk = com.company.demo.GroundControl.GPS2_RTK(bytes)
        onGPS2_RTK(pgps2_rtk)
    }

    override fun on_LOG_REQUEST_LIST(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_request_list = com.company.demo.GroundControl.LOG_REQUEST_LIST(bytes)
        onLOG_REQUEST_LIST(plog_request_list)
    }

    override fun on_SCALED_PRESSURE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_pressure = com.company.demo.GroundControl.SCALED_PRESSURE(bytes)
        onSCALED_PRESSURE(pscaled_pressure)
    }

    override fun on_MISSION_REQUEST_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_request_int = com.company.demo.GroundControl.MISSION_REQUEST_INT(cur)
        onMISSION_REQUEST_INT(pmission_request_int)
    }

    override fun on_V2_EXTENSION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pv2_extension = com.company.demo.GroundControl.V2_EXTENSION(bytes)
        onV2_EXTENSION(pv2_extension)
    }

    override fun on_HEARTBEAT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pheartbeat = com.company.demo.GroundControl.HEARTBEAT(cur)
        onHEARTBEAT(pheartbeat)
    }

    override fun on_PARAM_MAP_RC(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pparam_map_rc = com.company.demo.GroundControl.PARAM_MAP_RC(cur)
        onPARAM_MAP_RC(pparam_map_rc)
    }

    override fun on_POWER_STATUS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val ppower_status = com.company.demo.GroundControl.POWER_STATUS(cur)
        onPOWER_STATUS(ppower_status)
    }

    override fun on_TERRAIN_CHECK(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pterrain_check = com.company.demo.GroundControl.TERRAIN_CHECK(bytes)
        onTERRAIN_CHECK(pterrain_check)
    }

    override fun on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plocal_position_ned_system_global_offset = com.company.demo.GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(bytes)
        onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset)
    }

    override fun on_COMMAND_ACK(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcommand_ack = com.company.demo.GroundControl.COMMAND_ACK(cur)
        onCOMMAND_ACK(pcommand_ack)
    }

    override fun on_DATA_STREAM(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pdata_stream = com.company.demo.GroundControl.DATA_STREAM(bytes)
        onDATA_STREAM(pdata_stream)
    }

    override fun on_MISSION_REQUEST(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_request = com.company.demo.GroundControl.MISSION_REQUEST(cur)
        onMISSION_REQUEST(pmission_request)
    }

    override fun on_TERRAIN_REPORT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pterrain_report = com.company.demo.GroundControl.TERRAIN_REPORT(bytes)
        onTERRAIN_REPORT(pterrain_report)
    }

    override fun on_SET_HOME_POSITION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_home_position = com.company.demo.GroundControl.SET_HOME_POSITION(cur)
        onSET_HOME_POSITION(pset_home_position)
    }

    override fun on_SwitchModeCommand() {
        onSwitchModeCommand()
    }

    override fun on_HIL_RC_INPUTS_RAW(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_rc_inputs_raw = com.company.demo.GroundControl.HIL_RC_INPUTS_RAW(bytes)
        onHIL_RC_INPUTS_RAW(phil_rc_inputs_raw)
    }

    override fun on_SCALED_IMU3(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_imu3 = com.company.demo.GroundControl.SCALED_IMU3(bytes)
        onSCALED_IMU3(pscaled_imu3)
    }

    override fun on_SET_MODE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_mode = com.company.demo.GroundControl.SET_MODE(cur)
        onSET_MODE(pset_mode)
    }

    override fun on_POSITION_TARGET_GLOBAL_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pposition_target_global_int = com.company.demo.GroundControl.POSITION_TARGET_GLOBAL_INT(cur)
        onPOSITION_TARGET_GLOBAL_INT(pposition_target_global_int)
    }

    override fun on_FLIGHT_INFORMATION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pflight_information = com.company.demo.GroundControl.FLIGHT_INFORMATION(bytes)
        onFLIGHT_INFORMATION(pflight_information)
    }

    override fun on_SIM_STATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val psim_state = com.company.demo.GroundControl.SIM_STATE(bytes)
        onSIM_STATE(psim_state)
    }

    override fun on_MISSION_ITEM_REACHED(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmission_item_reached = com.company.demo.GroundControl.MISSION_ITEM_REACHED(bytes)
        onMISSION_ITEM_REACHED(pmission_item_reached)
    }

    override fun on_RC_CHANNELS_RAW(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prc_channels_raw = com.company.demo.GroundControl.RC_CHANNELS_RAW(bytes)
        onRC_CHANNELS_RAW(prc_channels_raw)
    }

    override fun on_SERVO_OUTPUT_RAW(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pservo_output_raw = com.company.demo.GroundControl.SERVO_OUTPUT_RAW(cur)
        onSERVO_OUTPUT_RAW(pservo_output_raw)
    }

    override fun on_VISION_SPEED_ESTIMATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvision_speed_estimate = com.company.demo.GroundControl.VISION_SPEED_ESTIMATE(bytes)
        onVISION_SPEED_ESTIMATE(pvision_speed_estimate)
    }

    override fun on_DEBUG_VECT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pdebug_vect = com.company.demo.GroundControl.DEBUG_VECT(cur)
        onDEBUG_VECT(pdebug_vect)
    }

    override fun on_LOG_REQUEST_END(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_request_end = com.company.demo.GroundControl.LOG_REQUEST_END(bytes)
        onLOG_REQUEST_END(plog_request_end)
    }

    override fun on_MISSION_ACK(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_ack = com.company.demo.GroundControl.MISSION_ACK(cur)
        onMISSION_ACK(pmission_ack)
    }

    override fun on_CHANGE_OPERATOR_CONTROL_ACK(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pchange_operator_control_ack = com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL_ACK(bytes)
        onCHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack)
    }

    override fun on_MISSION_CURRENT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmission_current = com.company.demo.GroundControl.MISSION_CURRENT(bytes)
        onMISSION_CURRENT(pmission_current)
    }

    override fun on_SYSTEM_TIME(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val psystem_time = com.company.demo.GroundControl.SYSTEM_TIME(bytes)
        onSYSTEM_TIME(psystem_time)
    }

    override fun on_CAMERA_TRIGGER(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pcamera_trigger = com.company.demo.GroundControl.CAMERA_TRIGGER(bytes)
        onCAMERA_TRIGGER(pcamera_trigger)
    }

    override fun on_VISION_POSITION_ESTIMATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvision_position_estimate = com.company.demo.GroundControl.VISION_POSITION_ESTIMATE(bytes)
        onVISION_POSITION_ESTIMATE(pvision_position_estimate)
    }

    override fun on_MANUAL_CONTROL(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmanual_control = com.company.demo.GroundControl.MANUAL_CONTROL(bytes)
        onMANUAL_CONTROL(pmanual_control)
    }

    override fun on_RC_CHANNELS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prc_channels = com.company.demo.GroundControl.RC_CHANNELS(bytes)
        onRC_CHANNELS(prc_channels)
    }

    override fun on_PARAM_VALUE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pparam_value = com.company.demo.GroundControl.PARAM_VALUE(cur)
        onPARAM_VALUE(pparam_value)
    }

    override fun on_BATTERY_STATUS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pbattery_status = com.company.demo.GroundControl.BATTERY_STATUS(cur)
        onBATTERY_STATUS(pbattery_status)
    }

    override fun on_SET_POSITION_TARGET_LOCAL_NED(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_position_target_local_ned = com.company.demo.GroundControl.SET_POSITION_TARGET_LOCAL_NED(cur)
        onSET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned)
    }

    override fun on_SERIAL_CONTROL(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pserial_control = com.company.demo.GroundControl.SERIAL_CONTROL(cur)
        onSERIAL_CONTROL(pserial_control)
    }

    override fun on_SET_GPS_GLOBAL_ORIGIN(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_gps_global_origin = com.company.demo.GroundControl.SET_GPS_GLOBAL_ORIGIN(cur)
        onSET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin)
    }

    override fun on_AUTOPILOT_VERSION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pautopilot_version = com.company.demo.GroundControl.AUTOPILOT_VERSION(cur)
        onAUTOPILOT_VERSION(pautopilot_version)
    }

    override fun on_MISSION_REQUEST_LIST(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_request_list = com.company.demo.GroundControl.MISSION_REQUEST_LIST(cur)
        onMISSION_REQUEST_LIST(pmission_request_list)
    }

    override fun on_PLAY_TUNE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pplay_tune = com.company.demo.GroundControl.PLAY_TUNE(cur)
        onPLAY_TUNE(pplay_tune)
    }

    override fun on_SCALED_PRESSURE3(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_pressure3 = com.company.demo.GroundControl.SCALED_PRESSURE3(bytes)
        onSCALED_PRESSURE3(pscaled_pressure3)
    }

    override fun on_MISSION_REQUEST_PARTIAL_LIST(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_request_partial_list = com.company.demo.GroundControl.MISSION_REQUEST_PARTIAL_LIST(cur)
        onMISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list)
    }

    override fun on_LOCAL_POSITION_NED(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plocal_position_ned = com.company.demo.GroundControl.LOCAL_POSITION_NED(bytes)
        onLOCAL_POSITION_NED(plocal_position_ned)
    }

    override fun on_DATA_TRANSMISSION_HANDSHAKE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pdata_transmission_handshake = com.company.demo.GroundControl.DATA_TRANSMISSION_HANDSHAKE(bytes)
        onDATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake)
    }

    override fun on_GPS_GLOBAL_ORIGIN(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pgps_global_origin = com.company.demo.GroundControl.GPS_GLOBAL_ORIGIN(cur)
        onGPS_GLOBAL_ORIGIN(pgps_global_origin)
    }

    override fun on_SCALED_IMU2(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_imu2 = com.company.demo.GroundControl.SCALED_IMU2(bytes)
        onSCALED_IMU2(pscaled_imu2)
    }

    override fun on_ATTITUDE_QUATERNION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pattitude_quaternion = com.company.demo.GroundControl.ATTITUDE_QUATERNION(bytes)
        onATTITUDE_QUATERNION(pattitude_quaternion)
    }

    override fun on_HIL_ACTUATOR_CONTROLS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val phil_actuator_controls = com.company.demo.GroundControl.HIL_ACTUATOR_CONTROLS(cur)
        onHIL_ACTUATOR_CONTROLS(phil_actuator_controls)
    }

    override fun on_POSITION_TARGET_LOCAL_NED(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pposition_target_local_ned = com.company.demo.GroundControl.POSITION_TARGET_LOCAL_NED(cur)
        onPOSITION_TARGET_LOCAL_NED(pposition_target_local_ned)
    }

    override fun on_DISTANCE_SENSOR(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pdistance_sensor = com.company.demo.GroundControl.DISTANCE_SENSOR(cur)
        onDISTANCE_SENSOR(pdistance_sensor)
    }

    override fun on_HIL_OPTICAL_FLOW(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_optical_flow = com.company.demo.GroundControl.HIL_OPTICAL_FLOW(bytes)
        onHIL_OPTICAL_FLOW(phil_optical_flow)
    }

    override fun on_SCALED_PRESSURE2(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_pressure2 = com.company.demo.GroundControl.SCALED_PRESSURE2(bytes)
        onSCALED_PRESSURE2(pscaled_pressure2)
    }

    override fun on_WIND_COV(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pwind_cov = com.company.demo.GroundControl.WIND_COV(bytes)
        onWIND_COV(pwind_cov)
    }

    override fun on_CHANGE_OPERATOR_CONTROL(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pchange_operator_control = com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL(cur)
        onCHANGE_OPERATOR_CONTROL(pchange_operator_control)
    }

    override fun on_SYS_STATUS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val psys_status = com.company.demo.GroundControl.SYS_STATUS(cur)
        onSYS_STATUS(psys_status)
    }

    override fun on_MISSION_ITEM(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_item = com.company.demo.GroundControl.MISSION_ITEM(cur)
        onMISSION_ITEM(pmission_item)
    }

    override fun on_RAW_IMU(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val praw_imu = com.company.demo.GroundControl.RAW_IMU(bytes)
        onRAW_IMU(praw_imu)
    }

    override fun on_COMMAND_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcommand_int = com.company.demo.GroundControl.COMMAND_INT(cur)
        onCOMMAND_INT(pcommand_int)
    }

    override fun on_OPTICAL_FLOW(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val poptical_flow = com.company.demo.GroundControl.OPTICAL_FLOW(cur)
        onOPTICAL_FLOW(poptical_flow)
    }

    override fun on_MISSION_ITEM_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_item_int = com.company.demo.GroundControl.MISSION_ITEM_INT(cur)
        onMISSION_ITEM_INT(pmission_item_int)
    }

    override fun on_HIGHRES_IMU(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phighres_imu = com.company.demo.GroundControl.HIGHRES_IMU(bytes)
        onHIGHRES_IMU(phighres_imu)
    }

    override fun on_EXTENDED_SYS_STATE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pextended_sys_state = com.company.demo.GroundControl.EXTENDED_SYS_STATE(cur)
        onEXTENDED_SYS_STATE(pextended_sys_state)
    }

    override fun on_GPS_INJECT_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps_inject_data = com.company.demo.GroundControl.GPS_INJECT_DATA(bytes)
        onGPS_INJECT_DATA(pgps_inject_data)
    }

    override fun on_ATTITUDE_QUATERNION_COV(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pattitude_quaternion_cov = com.company.demo.GroundControl.ATTITUDE_QUATERNION_COV(bytes)
        onATTITUDE_QUATERNION_COV(pattitude_quaternion_cov)
    }

    override fun on_NAMED_VALUE_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pnamed_value_int = com.company.demo.GroundControl.NAMED_VALUE_INT(cur)
        onNAMED_VALUE_INT(pnamed_value_int)
    }

    override fun on_RADIO_STATUS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pradio_status = com.company.demo.GroundControl.RADIO_STATUS(bytes)
        onRADIO_STATUS(pradio_status)
    }

    override fun on_GPS_RTCM_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps_rtcm_data = com.company.demo.GroundControl.GPS_RTCM_DATA(bytes)
        onGPS_RTCM_DATA(pgps_rtcm_data)
    }

    override fun on_GLOBAL_VISION_POSITION_ESTIMATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pglobal_vision_position_estimate = com.company.demo.GroundControl.GLOBAL_VISION_POSITION_ESTIMATE(bytes)
        onGLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate)
    }

    override fun on_FILE_TRANSFER_PROTOCOL(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pfile_transfer_protocol = com.company.demo.GroundControl.FILE_TRANSFER_PROTOCOL(bytes)
        onFILE_TRANSFER_PROTOCOL(pfile_transfer_protocol)
    }

    var sendingPack: Pack? = null
    override fun pullSendingPack(): Pack? {
        val ret = sendingPack
        sendingPack = null
        return ret
    }

    override fun pushSendingPack(pack: AdHoc.Pack): Boolean {
        if (sendingPack != null) return false

        sendingPack = pack
        return true
    }

}


// By default, assertions are disabled at runtime. Two command-line switches allow you to selectively enable or disable assertions.
// To enable assertions at various granularities, use the -enableassertions, or -ea, switch.
// To disable assertions at various granularities, use the -disableassertions, or -da,

fun main() {
    val cur = Cursor()
    val cur_dst = Cursor()

    val buff = ByteArray(1024)
    fun transmission(src: java.io.InputStream, dst: java.io.OutputStream) {
        try {
            while (true) {
                val bytes = src.read(buff, 0, buff.size)
                if (bytes < 1) break
                dst.write(buff, 0, bytes)
            }
        } catch (e: java.io.IOException) {
            e.printStackTrace()
            assert(false)
        }
    }

    val CommunicationChannel_instance = CommunicationChannel_test()



    println("-------------------- ATTITUDE_TARGET -------------------------")

    fill(TestChannel.NEW.ATTITUDE_TARGET(cur))
    val pattitude_target = com.company.demo.GroundControl.ATTITUDE_TARGET(cur)
    onATTITUDE_TARGET(pattitude_target)
    run {
        cur_dst.init(com.company.demo.GroundControl.ATTITUDE_TARGET.meta)
        val dst = test_.ATTITUDE_TARGET(cur_dst)
        dst.copyFrom(pattitude_target)
        assert(Arrays.equals(pattitude_target.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pattitude_target)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_COUNT -------------------------")

    fill(TestChannel.NEW.MISSION_COUNT(cur))
    val pmission_count = com.company.demo.GroundControl.MISSION_COUNT(cur)
    onMISSION_COUNT(pmission_count)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_COUNT.meta)
        val dst = test_.MISSION_COUNT(cur_dst)
        dst.copyFrom(pmission_count)
        assert(Arrays.equals(pmission_count.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_count)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ADSB_VEHICLE -------------------------")

    val padsb_vehicle = CommunicationChannel.NEW.ADSB_VEHICLE(cur)
    fill(padsb_vehicle)
    onADSB_VEHICLE(padsb_vehicle)
    run {

        var dst = CommunicationChannel.NEW.ADSB_VEHICLE(cur_dst)
        padsb_vehicle.copyInto(dst)
        assert(Arrays.equals(padsb_vehicle.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.ADSB_VEHICLE(cur_dst)
        dst.copyFrom(padsb_vehicle)
        assert(Arrays.equals(padsb_vehicle.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(padsb_vehicle)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MESSAGE_INTERVAL -------------------------")

    val pmessage_interval = CommunicationChannel.NEW.MESSAGE_INTERVAL(cur)
    fill(pmessage_interval)
    onMESSAGE_INTERVAL(pmessage_interval)
    run {

        var dst = CommunicationChannel.NEW.MESSAGE_INTERVAL(cur_dst)
        pmessage_interval.copyInto(dst)
        assert(Arrays.equals(pmessage_interval.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MESSAGE_INTERVAL(cur_dst)
        dst.copyFrom(pmessage_interval)
        assert(Arrays.equals(pmessage_interval.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pmessage_interval)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- EKF_STATUS_REPORT -------------------------")

    val pekf_status_report = CommunicationChannel.NEW.EKF_STATUS_REPORT(cur)
    fill(pekf_status_report)
    onEKF_STATUS_REPORT(pekf_status_report)
    run {

        var dst = CommunicationChannel.NEW.EKF_STATUS_REPORT(cur_dst)
        pekf_status_report.copyInto(dst)
        assert(Arrays.equals(pekf_status_report.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.EKF_STATUS_REPORT(cur_dst)
        dst.copyFrom(pekf_status_report)
        assert(Arrays.equals(pekf_status_report.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pekf_status_report)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ESTIMATOR_STATUS -------------------------")

    val pestimator_status = CommunicationChannel.NEW.ESTIMATOR_STATUS(cur)
    fill(pestimator_status)
    onESTIMATOR_STATUS(pestimator_status)
    run {

        var dst = CommunicationChannel.NEW.ESTIMATOR_STATUS(cur_dst)
        pestimator_status.copyInto(dst)
        assert(Arrays.equals(pestimator_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.ESTIMATOR_STATUS(cur_dst)
        dst.copyFrom(pestimator_status)
        assert(Arrays.equals(pestimator_status.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pestimator_status)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HWSTATUS -------------------------")

    val phwstatus = CommunicationChannel.NEW.HWSTATUS(cur)
    fill(phwstatus)
    onHWSTATUS(phwstatus)
    run {

        var dst = CommunicationChannel.NEW.HWSTATUS(cur_dst)
        phwstatus.copyInto(dst)
        assert(Arrays.equals(phwstatus.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.HWSTATUS(cur_dst)
        dst.copyFrom(phwstatus)
        assert(Arrays.equals(phwstatus.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(phwstatus)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- TIMESYNC -------------------------")

    val ptimesync = CommunicationChannel.NEW.TIMESYNC(cur)
    fill(ptimesync)
    onTIMESYNC(ptimesync)
    run {

        var dst = CommunicationChannel.NEW.TIMESYNC(cur_dst)
        ptimesync.copyInto(dst)
        assert(Arrays.equals(ptimesync.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.TIMESYNC(cur_dst)
        dst.copyFrom(ptimesync)
        assert(Arrays.equals(ptimesync.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(ptimesync)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_EXT_REQUEST_LIST -------------------------")

    val pparam_ext_request_list = CommunicationChannel.NEW.PARAM_EXT_REQUEST_LIST(cur)
    fill(pparam_ext_request_list)
    onPARAM_EXT_REQUEST_LIST(pparam_ext_request_list)
    run {

        var dst = CommunicationChannel.NEW.PARAM_EXT_REQUEST_LIST(cur_dst)
        pparam_ext_request_list.copyInto(dst)
        assert(Arrays.equals(pparam_ext_request_list.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PARAM_EXT_REQUEST_LIST(cur_dst)
        dst.copyFrom(pparam_ext_request_list)
        assert(Arrays.equals(pparam_ext_request_list.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pparam_ext_request_list)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GLOBAL_POSITION_INT_COV -------------------------")

    fill(TestChannel.NEW.GLOBAL_POSITION_INT_COV(cur))
    val pglobal_position_int_cov = com.company.demo.GroundControl.GLOBAL_POSITION_INT_COV(cur)
    onGLOBAL_POSITION_INT_COV(pglobal_position_int_cov)
    run {
        cur_dst.init(com.company.demo.GroundControl.GLOBAL_POSITION_INT_COV.meta)
        val dst = test_.GLOBAL_POSITION_INT_COV(cur_dst)
        dst.copyFrom(pglobal_position_int_cov)
        assert(Arrays.equals(pglobal_position_int_cov.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pglobal_position_int_cov)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- BUTTON_CHANGE -------------------------")

    val pbutton_change = CommunicationChannel.NEW.BUTTON_CHANGE(cur)
    fill(pbutton_change)
    onBUTTON_CHANGE(pbutton_change)
    run {

        var dst = CommunicationChannel.NEW.BUTTON_CHANGE(cur_dst)
        pbutton_change.copyInto(dst)
        assert(Arrays.equals(pbutton_change.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.BUTTON_CHANGE(cur_dst)
        dst.copyFrom(pbutton_change)
        assert(Arrays.equals(pbutton_change.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pbutton_change)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SAFETY_SET_ALLOWED_AREA -------------------------")

    fill(TestChannel.NEW.SAFETY_SET_ALLOWED_AREA(cur))
    val psafety_set_allowed_area = com.company.demo.GroundControl.SAFETY_SET_ALLOWED_AREA(cur)
    onSAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area)
    run {
        cur_dst.init(com.company.demo.GroundControl.SAFETY_SET_ALLOWED_AREA.meta)
        val dst = test_.SAFETY_SET_ALLOWED_AREA(cur_dst)
        dst.copyFrom(psafety_set_allowed_area)
        assert(Arrays.equals(psafety_set_allowed_area.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(psafety_set_allowed_area)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- UAVCAN_NODE_STATUS -------------------------")

    val puavcan_node_status = CommunicationChannel.NEW.UAVCAN_NODE_STATUS(cur)
    fill(puavcan_node_status)
    onUAVCAN_NODE_STATUS(puavcan_node_status)
    run {

        var dst = CommunicationChannel.NEW.UAVCAN_NODE_STATUS(cur_dst)
        puavcan_node_status.copyInto(dst)
        assert(Arrays.equals(puavcan_node_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.UAVCAN_NODE_STATUS(cur_dst)
        dst.copyFrom(puavcan_node_status)
        assert(Arrays.equals(puavcan_node_status.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(puavcan_node_status)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- COLLISION -------------------------")

    val pcollision = CommunicationChannel.NEW.COLLISION(cur)
    fill(pcollision)
    onCOLLISION(pcollision)
    run {

        var dst = CommunicationChannel.NEW.COLLISION(cur_dst)
        pcollision.copyInto(dst)
        assert(Arrays.equals(pcollision.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.COLLISION(cur_dst)
        dst.copyFrom(pcollision)
        assert(Arrays.equals(pcollision.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pcollision)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GIMBAL_TORQUE_CMD_REPORT -------------------------")

    val pgimbal_torque_cmd_report = CommunicationChannel.NEW.GIMBAL_TORQUE_CMD_REPORT(cur)
    fill(pgimbal_torque_cmd_report)
    onGIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report)
    run {

        var dst = CommunicationChannel.NEW.GIMBAL_TORQUE_CMD_REPORT(cur_dst)
        pgimbal_torque_cmd_report.copyInto(dst)
        assert(Arrays.equals(pgimbal_torque_cmd_report.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GIMBAL_TORQUE_CMD_REPORT(cur_dst)
        dst.copyFrom(pgimbal_torque_cmd_report)
        assert(Arrays.equals(pgimbal_torque_cmd_report.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pgimbal_torque_cmd_report)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ALTITUDE -------------------------")

    val paltitude = CommunicationChannel.NEW.ALTITUDE(cur)
    fill(paltitude)
    onALTITUDE(paltitude)
    run {

        var dst = CommunicationChannel.NEW.ALTITUDE(cur_dst)
        paltitude.copyInto(dst)
        assert(Arrays.equals(paltitude.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.ALTITUDE(cur_dst)
        dst.copyFrom(paltitude)
        assert(Arrays.equals(paltitude.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(paltitude)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIL_STATE_QUATERNION -------------------------")

    val phil_state_quaternion = CommunicationChannel.NEW.HIL_STATE_QUATERNION(cur)
    fill(phil_state_quaternion)
    onHIL_STATE_QUATERNION(phil_state_quaternion)
    run {

        var dst = CommunicationChannel.NEW.HIL_STATE_QUATERNION(cur_dst)
        phil_state_quaternion.copyInto(dst)
        assert(Arrays.equals(phil_state_quaternion.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.HIL_STATE_QUATERNION(cur_dst)
        dst.copyFrom(phil_state_quaternion)
        assert(Arrays.equals(phil_state_quaternion.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(phil_state_quaternion)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SENSOR_OFFSETS -------------------------")

    val psensor_offsets = CommunicationChannel.NEW.SENSOR_OFFSETS(cur)
    fill(psensor_offsets)
    onSENSOR_OFFSETS(psensor_offsets)
    run {

        var dst = CommunicationChannel.NEW.SENSOR_OFFSETS(cur_dst)
        psensor_offsets.copyInto(dst)
        assert(Arrays.equals(psensor_offsets.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SENSOR_OFFSETS(cur_dst)
        dst.copyFrom(psensor_offsets)
        assert(Arrays.equals(psensor_offsets.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(psensor_offsets)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- STORAGE_INFORMATION -------------------------")

    val pstorage_information = CommunicationChannel.NEW.STORAGE_INFORMATION(cur)
    fill(pstorage_information)
    onSTORAGE_INFORMATION(pstorage_information)
    run {

        var dst = CommunicationChannel.NEW.STORAGE_INFORMATION(cur_dst)
        pstorage_information.copyInto(dst)
        assert(Arrays.equals(pstorage_information.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.STORAGE_INFORMATION(cur_dst)
        dst.copyFrom(pstorage_information)
        assert(Arrays.equals(pstorage_information.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pstorage_information)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CAMERA_INFORMATION -------------------------")

    val pcamera_information = CommunicationChannel.NEW.CAMERA_INFORMATION(cur)
    fill(pcamera_information)
    onCAMERA_INFORMATION(pcamera_information)
    run {

        var dst = CommunicationChannel.NEW.CAMERA_INFORMATION(cur_dst)
        pcamera_information.copyInto(dst)
        assert(Arrays.equals(pcamera_information.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.CAMERA_INFORMATION(cur_dst)
        dst.copyFrom(pcamera_information)
        assert(Arrays.equals(pcamera_information.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pcamera_information)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS_STATUS -------------------------")

    fill(TestChannel.NEW.GPS_STATUS(cur))
    val pgps_status = com.company.demo.GroundControl.GPS_STATUS(cur)
    onGPS_STATUS(pgps_status)
    run {
        cur_dst.init(com.company.demo.GroundControl.GPS_STATUS.meta)
        val dst = test_.GPS_STATUS(cur_dst)
        dst.copyFrom(pgps_status)
        assert(Arrays.equals(pgps_status.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pgps_status)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DEVICE_OP_WRITE_REPLY -------------------------")

    val pdevice_op_write_reply = CommunicationChannel.NEW.DEVICE_OP_WRITE_REPLY(cur)
    fill(pdevice_op_write_reply)
    onDEVICE_OP_WRITE_REPLY(pdevice_op_write_reply)
    run {

        var dst = CommunicationChannel.NEW.DEVICE_OP_WRITE_REPLY(cur_dst)
        pdevice_op_write_reply.copyInto(dst)
        assert(Arrays.equals(pdevice_op_write_reply.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DEVICE_OP_WRITE_REPLY(cur_dst)
        dst.copyFrom(pdevice_op_write_reply)
        assert(Arrays.equals(pdevice_op_write_reply.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdevice_op_write_reply)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_SET -------------------------")

    fill(TestChannel.NEW.PARAM_SET(cur))
    val pparam_set = com.company.demo.GroundControl.PARAM_SET(cur)
    onPARAM_SET(pparam_set)
    run {
        cur_dst.init(com.company.demo.GroundControl.PARAM_SET.meta)
        val dst = test_.PARAM_SET(cur_dst)
        dst.copyFrom(pparam_set)
        assert(Arrays.equals(pparam_set.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pparam_set)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- TERRAIN_DATA -------------------------")

    val pterrain_data = CommunicationChannel.NEW.TERRAIN_DATA(cur)
    fill(pterrain_data)
    onTERRAIN_DATA(pterrain_data)
    run {

        var dst = CommunicationChannel.NEW.TERRAIN_DATA(cur_dst)
        pterrain_data.copyInto(dst)
        assert(Arrays.equals(pterrain_data.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.TERRAIN_DATA(cur_dst)
        dst.copyFrom(pterrain_data)
        assert(Arrays.equals(pterrain_data.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pterrain_data)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GIMBAL_CONTROL -------------------------")

    val pgimbal_control = CommunicationChannel.NEW.GIMBAL_CONTROL(cur)
    fill(pgimbal_control)
    onGIMBAL_CONTROL(pgimbal_control)
    run {

        var dst = CommunicationChannel.NEW.GIMBAL_CONTROL(cur_dst)
        pgimbal_control.copyInto(dst)
        assert(Arrays.equals(pgimbal_control.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GIMBAL_CONTROL(cur_dst)
        dst.copyFrom(pgimbal_control)
        assert(Arrays.equals(pgimbal_control.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pgimbal_control)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RC_CHANNELS_OVERRIDE -------------------------")

    fill(TestChannel.NEW.RC_CHANNELS_OVERRIDE(cur))
    val prc_channels_override = com.company.demo.GroundControl.RC_CHANNELS_OVERRIDE(cur)
    onRC_CHANNELS_OVERRIDE(prc_channels_override)
    run {
        cur_dst.init(com.company.demo.GroundControl.RC_CHANNELS_OVERRIDE.meta)
        val dst = test_.RC_CHANNELS_OVERRIDE(cur_dst)
        dst.copyFrom(prc_channels_override)
        assert(Arrays.equals(prc_channels_override.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(prc_channels_override)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SCALED_IMU -------------------------")

    fill(TestChannel.NEW.SCALED_IMU(cur))
    val pscaled_imu = com.company.demo.GroundControl.SCALED_IMU(cur)
    onSCALED_IMU(pscaled_imu)
    run {
        cur_dst.init(com.company.demo.GroundControl.SCALED_IMU.meta)
        val dst = test_.SCALED_IMU(cur_dst)
        dst.copyFrom(pscaled_imu)
        assert(Arrays.equals(pscaled_imu.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pscaled_imu)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- VIDEO_STREAM_INFORMATION -------------------------")

    val pvideo_stream_information = CommunicationChannel.NEW.VIDEO_STREAM_INFORMATION(cur)
    fill(pvideo_stream_information)
    onVIDEO_STREAM_INFORMATION(pvideo_stream_information)
    run {

        var dst = CommunicationChannel.NEW.VIDEO_STREAM_INFORMATION(cur_dst)
        pvideo_stream_information.copyInto(dst)
        assert(Arrays.equals(pvideo_stream_information.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.VIDEO_STREAM_INFORMATION(cur_dst)
        dst.copyFrom(pvideo_stream_information)
        assert(Arrays.equals(pvideo_stream_information.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pvideo_stream_information)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- AHRS -------------------------")

    val pahrs = CommunicationChannel.NEW.AHRS(cur)
    fill(pahrs)
    onAHRS(pahrs)
    run {

        var dst = CommunicationChannel.NEW.AHRS(cur_dst)
        pahrs.copyInto(dst)
        assert(Arrays.equals(pahrs.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.AHRS(cur_dst)
        dst.copyFrom(pahrs)
        assert(Arrays.equals(pahrs.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pahrs)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DEBUG -------------------------")

    val pdebug = CommunicationChannel.NEW.DEBUG(cur)
    fill(pdebug)
    onDEBUG(pdebug)
    run {

        var dst = CommunicationChannel.NEW.DEBUG(cur_dst)
        pdebug.copyInto(dst)
        assert(Arrays.equals(pdebug.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DEBUG(cur_dst)
        dst.copyFrom(pdebug)
        assert(Arrays.equals(pdebug.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pdebug)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CAMERA_IMAGE_CAPTURED -------------------------")

    val pcamera_image_captured = CommunicationChannel.NEW.CAMERA_IMAGE_CAPTURED(cur)
    fill(pcamera_image_captured)
    onCAMERA_IMAGE_CAPTURED(pcamera_image_captured)
    run {

        var dst = CommunicationChannel.NEW.CAMERA_IMAGE_CAPTURED(cur_dst)
        pcamera_image_captured.copyInto(dst)
        assert(Arrays.equals(pcamera_image_captured.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.CAMERA_IMAGE_CAPTURED(cur_dst)
        dst.copyFrom(pcamera_image_captured)
        assert(Arrays.equals(pcamera_image_captured.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pcamera_image_captured)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOG_ENTRY -------------------------")

    val plog_entry = CommunicationChannel.NEW.LOG_ENTRY(cur)
    fill(plog_entry)
    onLOG_ENTRY(plog_entry)
    run {

        var dst = CommunicationChannel.NEW.LOG_ENTRY(cur_dst)
        plog_entry.copyInto(dst)
        assert(Arrays.equals(plog_entry.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOG_ENTRY(cur_dst)
        dst.copyFrom(plog_entry)
        assert(Arrays.equals(plog_entry.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(plog_entry)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ACTUATOR_CONTROL_TARGET -------------------------")

    val pactuator_control_target = CommunicationChannel.NEW.ACTUATOR_CONTROL_TARGET(cur)
    fill(pactuator_control_target)
    onACTUATOR_CONTROL_TARGET(pactuator_control_target)
    run {

        var dst = CommunicationChannel.NEW.ACTUATOR_CONTROL_TARGET(cur_dst)
        pactuator_control_target.copyInto(dst)
        assert(Arrays.equals(pactuator_control_target.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.ACTUATOR_CONTROL_TARGET(cur_dst)
        dst.copyFrom(pactuator_control_target)
        assert(Arrays.equals(pactuator_control_target.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pactuator_control_target)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIGH_LATENCY -------------------------")

    val phigh_latency = CommunicationChannel.NEW.HIGH_LATENCY(cur)
    fill(phigh_latency)
    onHIGH_LATENCY(phigh_latency)
    run {

        var dst = CommunicationChannel.NEW.HIGH_LATENCY(cur_dst)
        phigh_latency.copyInto(dst)
        assert(Arrays.equals(phigh_latency.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.HIGH_LATENCY(cur_dst)
        dst.copyFrom(phigh_latency)
        assert(Arrays.equals(phigh_latency.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(phigh_latency)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_REQUEST_READ -------------------------")

    fill(TestChannel.NEW.PARAM_REQUEST_READ(cur))
    val pparam_request_read = com.company.demo.GroundControl.PARAM_REQUEST_READ(cur)
    onPARAM_REQUEST_READ(pparam_request_read)
    run {
        cur_dst.init(com.company.demo.GroundControl.PARAM_REQUEST_READ.meta)
        val dst = test_.PARAM_REQUEST_READ(cur_dst)
        dst.copyFrom(pparam_request_read)
        assert(Arrays.equals(pparam_request_read.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pparam_request_read)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_ATTITUDE_TARGET -------------------------")

    fill(TestChannel.NEW.SET_ATTITUDE_TARGET(cur))
    val pset_attitude_target = com.company.demo.GroundControl.SET_ATTITUDE_TARGET(cur)
    onSET_ATTITUDE_TARGET(pset_attitude_target)
    run {
        cur_dst.init(com.company.demo.GroundControl.SET_ATTITUDE_TARGET.meta)
        val dst = test_.SET_ATTITUDE_TARGET(cur_dst)
        dst.copyFrom(pset_attitude_target)
        assert(Arrays.equals(pset_attitude_target.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pset_attitude_target)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- FOLLOW_TARGET -------------------------")

    val pfollow_target = CommunicationChannel.NEW.FOLLOW_TARGET(cur)
    fill(pfollow_target)
    onFOLLOW_TARGET(pfollow_target)
    run {

        var dst = CommunicationChannel.NEW.FOLLOW_TARGET(cur_dst)
        pfollow_target.copyInto(dst)
        assert(Arrays.equals(pfollow_target.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.FOLLOW_TARGET(cur_dst)
        dst.copyFrom(pfollow_target)
        assert(Arrays.equals(pfollow_target.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pfollow_target)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIL_STATE -------------------------")

    fill(TestChannel.NEW.HIL_STATE(cur))
    val phil_state = com.company.demo.GroundControl.HIL_STATE(cur)
    onHIL_STATE(phil_state)
    run {
        cur_dst.init(com.company.demo.GroundControl.HIL_STATE.meta)
        val dst = test_.HIL_STATE(cur_dst)
        dst.copyFrom(phil_state)
        assert(Arrays.equals(phil_state.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(phil_state)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HOME_POSITION -------------------------")

    val phome_position = CommunicationChannel.NEW.HOME_POSITION(cur)
    fill(phome_position)
    onHOME_POSITION(phome_position)
    run {

        var dst = CommunicationChannel.NEW.HOME_POSITION(cur_dst)
        phome_position.copyInto(dst)
        assert(Arrays.equals(phome_position.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.HOME_POSITION(cur_dst)
        dst.copyFrom(phome_position)
        assert(Arrays.equals(phome_position.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(phome_position)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- FENCE_STATUS -------------------------")

    val pfence_status = CommunicationChannel.NEW.FENCE_STATUS(cur)
    fill(pfence_status)
    onFENCE_STATUS(pfence_status)
    run {

        var dst = CommunicationChannel.NEW.FENCE_STATUS(cur_dst)
        pfence_status.copyInto(dst)
        assert(Arrays.equals(pfence_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.FENCE_STATUS(cur_dst)
        dst.copyFrom(pfence_status)
        assert(Arrays.equals(pfence_status.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pfence_status)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- REMOTE_LOG_BLOCK_STATUS -------------------------")

    val premote_log_block_status = CommunicationChannel.NEW.REMOTE_LOG_BLOCK_STATUS(cur)
    fill(premote_log_block_status)
    onREMOTE_LOG_BLOCK_STATUS(premote_log_block_status)
    run {

        var dst = CommunicationChannel.NEW.REMOTE_LOG_BLOCK_STATUS(cur_dst)
        premote_log_block_status.copyInto(dst)
        assert(Arrays.equals(premote_log_block_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.REMOTE_LOG_BLOCK_STATUS(cur_dst)
        dst.copyFrom(premote_log_block_status)
        assert(Arrays.equals(premote_log_block_status.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(premote_log_block_status)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- OBSTACLE_DISTANCE -------------------------")

    val pobstacle_distance = CommunicationChannel.NEW.OBSTACLE_DISTANCE(cur)
    fill(pobstacle_distance)
    onOBSTACLE_DISTANCE(pobstacle_distance)
    run {

        var dst = CommunicationChannel.NEW.OBSTACLE_DISTANCE(cur_dst)
        pobstacle_distance.copyInto(dst)
        assert(Arrays.equals(pobstacle_distance.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.OBSTACLE_DISTANCE(cur_dst)
        dst.copyFrom(pobstacle_distance)
        assert(Arrays.equals(pobstacle_distance.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pobstacle_distance)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS2_RAW -------------------------")

    val pgps2_raw = CommunicationChannel.NEW.GPS2_RAW(cur)
    fill(pgps2_raw)
    onGPS2_RAW(pgps2_raw)
    run {

        var dst = CommunicationChannel.NEW.GPS2_RAW(cur_dst)
        pgps2_raw.copyInto(dst)
        assert(Arrays.equals(pgps2_raw.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GPS2_RAW(cur_dst)
        dst.copyFrom(pgps2_raw)
        assert(Arrays.equals(pgps2_raw.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pgps2_raw)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- REQUEST_DATA_STREAM -------------------------")

    fill(TestChannel.NEW.REQUEST_DATA_STREAM(cur))
    val prequest_data_stream = com.company.demo.GroundControl.REQUEST_DATA_STREAM(cur)
    onREQUEST_DATA_STREAM(prequest_data_stream)
    run {
        cur_dst.init(com.company.demo.GroundControl.REQUEST_DATA_STREAM.meta)
        val dst = test_.REQUEST_DATA_STREAM(cur_dst)
        dst.copyFrom(prequest_data_stream)
        assert(Arrays.equals(prequest_data_stream.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(prequest_data_stream)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MEMORY_VECT -------------------------")

    val pmemory_vect = CommunicationChannel.NEW.MEMORY_VECT(cur)
    fill(pmemory_vect)
    onMEMORY_VECT(pmemory_vect)
    run {

        var dst = CommunicationChannel.NEW.MEMORY_VECT(cur_dst)
        pmemory_vect.copyInto(dst)
        assert(Arrays.equals(pmemory_vect.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MEMORY_VECT(cur_dst)
        dst.copyFrom(pmemory_vect)
        assert(Arrays.equals(pmemory_vect.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pmemory_vect)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_EXT_REQUEST_READ -------------------------")

    val pparam_ext_request_read = CommunicationChannel.NEW.PARAM_EXT_REQUEST_READ(cur)
    fill(pparam_ext_request_read)
    onPARAM_EXT_REQUEST_READ(pparam_ext_request_read)
    run {

        var dst = CommunicationChannel.NEW.PARAM_EXT_REQUEST_READ(cur_dst)
        pparam_ext_request_read.copyInto(dst)
        assert(Arrays.equals(pparam_ext_request_read.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PARAM_EXT_REQUEST_READ(cur_dst)
        dst.copyFrom(pparam_ext_request_read)
        assert(Arrays.equals(pparam_ext_request_read.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pparam_ext_request_read)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIL_CONTROLS -------------------------")

    fill(TestChannel.NEW.HIL_CONTROLS(cur))
    val phil_controls = com.company.demo.GroundControl.HIL_CONTROLS(cur)
    onHIL_CONTROLS(phil_controls)
    run {
        cur_dst.init(com.company.demo.GroundControl.HIL_CONTROLS.meta)
        val dst = test_.HIL_CONTROLS(cur_dst)
        dst.copyFrom(phil_controls)
        assert(Arrays.equals(phil_controls.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(phil_controls)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIL_SENSOR -------------------------")

    val phil_sensor = CommunicationChannel.NEW.HIL_SENSOR(cur)
    fill(phil_sensor)
    onHIL_SENSOR(phil_sensor)
    run {

        var dst = CommunicationChannel.NEW.HIL_SENSOR(cur_dst)
        phil_sensor.copyInto(dst)
        assert(Arrays.equals(phil_sensor.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.HIL_SENSOR(cur_dst)
        dst.copyFrom(phil_sensor)
        assert(Arrays.equals(phil_sensor.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(phil_sensor)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SETUP_SIGNING -------------------------")

    val psetup_signing = CommunicationChannel.NEW.SETUP_SIGNING(cur)
    fill(psetup_signing)
    onSETUP_SIGNING(psetup_signing)
    run {

        var dst = CommunicationChannel.NEW.SETUP_SIGNING(cur_dst)
        psetup_signing.copyInto(dst)
        assert(Arrays.equals(psetup_signing.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SETUP_SIGNING(cur_dst)
        dst.copyFrom(psetup_signing)
        assert(Arrays.equals(psetup_signing.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(psetup_signing)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS_RTK -------------------------")

    val pgps_rtk = CommunicationChannel.NEW.GPS_RTK(cur)
    fill(pgps_rtk)
    onGPS_RTK(pgps_rtk)
    run {

        var dst = CommunicationChannel.NEW.GPS_RTK(cur_dst)
        pgps_rtk.copyInto(dst)
        assert(Arrays.equals(pgps_rtk.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GPS_RTK(cur_dst)
        dst.copyFrom(pgps_rtk)
        assert(Arrays.equals(pgps_rtk.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pgps_rtk)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_REQUEST_LIST -------------------------")

    fill(TestChannel.NEW.PARAM_REQUEST_LIST(cur))
    val pparam_request_list = com.company.demo.GroundControl.PARAM_REQUEST_LIST(cur)
    onPARAM_REQUEST_LIST(pparam_request_list)
    run {
        cur_dst.init(com.company.demo.GroundControl.PARAM_REQUEST_LIST.meta)
        val dst = test_.PARAM_REQUEST_LIST(cur_dst)
        dst.copyFrom(pparam_request_list)
        assert(Arrays.equals(pparam_request_list.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pparam_request_list)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- UAVIONIX_ADSB_OUT_CFG -------------------------")

    val puavionix_adsb_out_cfg = CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_CFG(cur)
    fill(puavionix_adsb_out_cfg)
    onUAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg)
    run {

        var dst = CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_CFG(cur_dst)
        puavionix_adsb_out_cfg.copyInto(dst)
        assert(Arrays.equals(puavionix_adsb_out_cfg.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_CFG(cur_dst)
        dst.copyFrom(puavionix_adsb_out_cfg)
        assert(Arrays.equals(puavionix_adsb_out_cfg.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(puavionix_adsb_out_cfg)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LANDING_TARGET -------------------------")

    val planding_target = CommunicationChannel.NEW.LANDING_TARGET(cur)
    fill(planding_target)
    onLANDING_TARGET(planding_target)
    run {

        var dst = CommunicationChannel.NEW.LANDING_TARGET(cur_dst)
        planding_target.copyInto(dst)
        assert(Arrays.equals(planding_target.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LANDING_TARGET(cur_dst)
        dst.copyFrom(planding_target)
        assert(Arrays.equals(planding_target.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(planding_target)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_ACTUATOR_CONTROL_TARGET -------------------------")

    val pset_actuator_control_target = CommunicationChannel.NEW.SET_ACTUATOR_CONTROL_TARGET(cur)
    fill(pset_actuator_control_target)
    onSET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target)
    run {

        var dst = CommunicationChannel.NEW.SET_ACTUATOR_CONTROL_TARGET(cur_dst)
        pset_actuator_control_target.copyInto(dst)
        assert(Arrays.equals(pset_actuator_control_target.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SET_ACTUATOR_CONTROL_TARGET(cur_dst)
        dst.copyFrom(pset_actuator_control_target)
        assert(Arrays.equals(pset_actuator_control_target.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pset_actuator_control_target)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CONTROL_SYSTEM_STATE -------------------------")

    val pcontrol_system_state = CommunicationChannel.NEW.CONTROL_SYSTEM_STATE(cur)
    fill(pcontrol_system_state)
    onCONTROL_SYSTEM_STATE(pcontrol_system_state)
    run {

        var dst = CommunicationChannel.NEW.CONTROL_SYSTEM_STATE(cur_dst)
        pcontrol_system_state.copyInto(dst)
        assert(Arrays.equals(pcontrol_system_state.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.CONTROL_SYSTEM_STATE(cur_dst)
        dst.copyFrom(pcontrol_system_state)
        assert(Arrays.equals(pcontrol_system_state.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pcontrol_system_state)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_POSITION_TARGET_GLOBAL_INT -------------------------")

    fill(TestChannel.NEW.SET_POSITION_TARGET_GLOBAL_INT(cur))
    val pset_position_target_global_int = com.company.demo.GroundControl.SET_POSITION_TARGET_GLOBAL_INT(cur)
    onSET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int)
    run {
        cur_dst.init(com.company.demo.GroundControl.SET_POSITION_TARGET_GLOBAL_INT.meta)
        val dst = test_.SET_POSITION_TARGET_GLOBAL_INT(cur_dst)
        dst.copyFrom(pset_position_target_global_int)
        assert(Arrays.equals(pset_position_target_global_int.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pset_position_target_global_int)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DATA32 -------------------------")

    val pdata32 = CommunicationChannel.NEW.DATA32(cur)
    fill(pdata32)
    onDATA32(pdata32)
    run {

        var dst = CommunicationChannel.NEW.DATA32(cur_dst)
        pdata32.copyInto(dst)
        assert(Arrays.equals(pdata32.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DATA32(cur_dst)
        dst.copyFrom(pdata32)
        assert(Arrays.equals(pdata32.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdata32)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PING33 -------------------------")

    val pping33 = CommunicationChannel.NEW.PING33(cur)
    fill(pping33)
    onPING33(pping33)
    run {

        var dst = CommunicationChannel.NEW.PING33(cur_dst)
        pping33.copyInto(dst)
        assert(Arrays.equals(pping33.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PING33(cur_dst)
        dst.copyFrom(pping33)
        assert(Arrays.equals(pping33.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pping33)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- VFR_HUD -------------------------")

    fill(TestChannel.NEW.VFR_HUD(cur))
    val pvfr_hud = com.company.demo.GroundControl.VFR_HUD(cur)
    onVFR_HUD(pvfr_hud)
    run {
        cur_dst.init(com.company.demo.GroundControl.VFR_HUD.meta)
        val dst = test_.VFR_HUD(cur_dst)
        dst.copyFrom(pvfr_hud)
        assert(Arrays.equals(pvfr_hud.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pvfr_hud)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RALLY_POINT -------------------------")

    val prally_point = CommunicationChannel.NEW.RALLY_POINT(cur)
    fill(prally_point)
    onRALLY_POINT(prally_point)
    run {

        var dst = CommunicationChannel.NEW.RALLY_POINT(cur_dst)
        prally_point.copyInto(dst)
        assert(Arrays.equals(prally_point.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.RALLY_POINT(cur_dst)
        dst.copyFrom(prally_point)
        assert(Arrays.equals(prally_point.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(prally_point)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_SET_CURRENT -------------------------")

    fill(TestChannel.NEW.MISSION_SET_CURRENT(cur))
    val pmission_set_current = com.company.demo.GroundControl.MISSION_SET_CURRENT(cur)
    onMISSION_SET_CURRENT(pmission_set_current)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_SET_CURRENT.meta)
        val dst = test_.MISSION_SET_CURRENT(cur_dst)
        dst.copyFrom(pmission_set_current)
        assert(Arrays.equals(pmission_set_current.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_set_current)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ADAP_TUNING -------------------------")

    val padap_tuning = CommunicationChannel.NEW.ADAP_TUNING(cur)
    fill(padap_tuning)
    onADAP_TUNING(padap_tuning)
    run {

        var dst = CommunicationChannel.NEW.ADAP_TUNING(cur_dst)
        padap_tuning.copyInto(dst)
        assert(Arrays.equals(padap_tuning.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.ADAP_TUNING(cur_dst)
        dst.copyFrom(padap_tuning)
        assert(Arrays.equals(padap_tuning.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(padap_tuning)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- VIBRATION -------------------------")

    val pvibration = CommunicationChannel.NEW.VIBRATION(cur)
    fill(pvibration)
    onVIBRATION(pvibration)
    run {

        var dst = CommunicationChannel.NEW.VIBRATION(cur_dst)
        pvibration.copyInto(dst)
        assert(Arrays.equals(pvibration.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.VIBRATION(cur_dst)
        dst.copyFrom(pvibration)
        assert(Arrays.equals(pvibration.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pvibration)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_EXT_VALUE -------------------------")

    val pparam_ext_value = CommunicationChannel.NEW.PARAM_EXT_VALUE(cur)
    fill(pparam_ext_value)
    onPARAM_EXT_VALUE(pparam_ext_value)
    run {

        var dst = CommunicationChannel.NEW.PARAM_EXT_VALUE(cur_dst)
        pparam_ext_value.copyInto(dst)
        assert(Arrays.equals(pparam_ext_value.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PARAM_EXT_VALUE(cur_dst)
        dst.copyFrom(pparam_ext_value)
        assert(Arrays.equals(pparam_ext_value.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pparam_ext_value)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- BATTERY2 -------------------------")

    val pbattery2 = CommunicationChannel.NEW.BATTERY2(cur)
    fill(pbattery2)
    onBATTERY2(pbattery2)
    run {

        var dst = CommunicationChannel.NEW.BATTERY2(cur_dst)
        pbattery2.copyInto(dst)
        assert(Arrays.equals(pbattery2.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.BATTERY2(cur_dst)
        dst.copyFrom(pbattery2)
        assert(Arrays.equals(pbattery2.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pbattery2)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LIMITS_STATUS -------------------------")

    val plimits_status = CommunicationChannel.NEW.LIMITS_STATUS(cur)
    fill(plimits_status)
    onLIMITS_STATUS(plimits_status)
    run {

        var dst = CommunicationChannel.NEW.LIMITS_STATUS(cur_dst)
        plimits_status.copyInto(dst)
        assert(Arrays.equals(plimits_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LIMITS_STATUS(cur_dst)
        dst.copyFrom(plimits_status)
        assert(Arrays.equals(plimits_status.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(plimits_status)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CAMERA_FEEDBACK -------------------------")

    val pcamera_feedback = CommunicationChannel.NEW.CAMERA_FEEDBACK(cur)
    fill(pcamera_feedback)
    onCAMERA_FEEDBACK(pcamera_feedback)
    run {

        var dst = CommunicationChannel.NEW.CAMERA_FEEDBACK(cur_dst)
        pcamera_feedback.copyInto(dst)
        assert(Arrays.equals(pcamera_feedback.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.CAMERA_FEEDBACK(cur_dst)
        dst.copyFrom(pcamera_feedback)
        assert(Arrays.equals(pcamera_feedback.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pcamera_feedback)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIL_GPS -------------------------")

    val phil_gps = CommunicationChannel.NEW.HIL_GPS(cur)
    fill(phil_gps)
    onHIL_GPS(phil_gps)
    run {

        var dst = CommunicationChannel.NEW.HIL_GPS(cur_dst)
        phil_gps.copyInto(dst)
        assert(Arrays.equals(phil_gps.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.HIL_GPS(cur_dst)
        dst.copyFrom(phil_gps)
        assert(Arrays.equals(phil_gps.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(phil_gps)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- NAV_CONTROLLER_OUTPUT -------------------------")

    fill(TestChannel.NEW.NAV_CONTROLLER_OUTPUT(cur))
    val pnav_controller_output = com.company.demo.GroundControl.NAV_CONTROLLER_OUTPUT(cur)
    onNAV_CONTROLLER_OUTPUT(pnav_controller_output)
    run {
        cur_dst.init(com.company.demo.GroundControl.NAV_CONTROLLER_OUTPUT.meta)
        val dst = test_.NAV_CONTROLLER_OUTPUT(cur_dst)
        dst.copyFrom(pnav_controller_output)
        assert(Arrays.equals(pnav_controller_output.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pnav_controller_output)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- AUTH_KEY -------------------------")

    fill(TestChannel.NEW.AUTH_KEY(cur))
    val pauth_key = com.company.demo.GroundControl.AUTH_KEY(cur)
    onAUTH_KEY(pauth_key)
    run {
        cur_dst.init(com.company.demo.GroundControl.AUTH_KEY.meta)
        val dst = test_.AUTH_KEY(cur_dst)
        dst.copyFrom(pauth_key)
        assert(Arrays.equals(pauth_key.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pauth_key)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- FENCE_FETCH_POINT -------------------------")

    val pfence_fetch_point = CommunicationChannel.NEW.FENCE_FETCH_POINT(cur)
    fill(pfence_fetch_point)
    onFENCE_FETCH_POINT(pfence_fetch_point)
    run {

        var dst = CommunicationChannel.NEW.FENCE_FETCH_POINT(cur_dst)
        pfence_fetch_point.copyInto(dst)
        assert(Arrays.equals(pfence_fetch_point.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.FENCE_FETCH_POINT(cur_dst)
        dst.copyFrom(pfence_fetch_point)
        assert(Arrays.equals(pfence_fetch_point.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pfence_fetch_point)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RADIO -------------------------")

    val pradio = CommunicationChannel.NEW.RADIO(cur)
    fill(pradio)
    onRADIO(pradio)
    run {

        var dst = CommunicationChannel.NEW.RADIO(cur_dst)
        pradio.copyInto(dst)
        assert(Arrays.equals(pradio.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.RADIO(cur_dst)
        dst.copyFrom(pradio)
        assert(Arrays.equals(pradio.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pradio)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOCAL_POSITION_NED_COV -------------------------")

    fill(TestChannel.NEW.LOCAL_POSITION_NED_COV(cur))
    val plocal_position_ned_cov = com.company.demo.GroundControl.LOCAL_POSITION_NED_COV(cur)
    onLOCAL_POSITION_NED_COV(plocal_position_ned_cov)
    run {
        cur_dst.init(com.company.demo.GroundControl.LOCAL_POSITION_NED_COV.meta)
        val dst = test_.LOCAL_POSITION_NED_COV(cur_dst)
        dst.copyFrom(plocal_position_ned_cov)
        assert(Arrays.equals(plocal_position_ned_cov.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(plocal_position_ned_cov)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- AIRSPEED_AUTOCAL -------------------------")

    val pairspeed_autocal = CommunicationChannel.NEW.AIRSPEED_AUTOCAL(cur)
    fill(pairspeed_autocal)
    onAIRSPEED_AUTOCAL(pairspeed_autocal)
    run {

        var dst = CommunicationChannel.NEW.AIRSPEED_AUTOCAL(cur_dst)
        pairspeed_autocal.copyInto(dst)
        assert(Arrays.equals(pairspeed_autocal.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.AIRSPEED_AUTOCAL(cur_dst)
        dst.copyFrom(pairspeed_autocal)
        assert(Arrays.equals(pairspeed_autocal.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pairspeed_autocal)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ATT_POS_MOCAP -------------------------")

    val patt_pos_mocap = CommunicationChannel.NEW.ATT_POS_MOCAP(cur)
    fill(patt_pos_mocap)
    onATT_POS_MOCAP(patt_pos_mocap)
    run {

        var dst = CommunicationChannel.NEW.ATT_POS_MOCAP(cur_dst)
        patt_pos_mocap.copyInto(dst)
        assert(Arrays.equals(patt_pos_mocap.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.ATT_POS_MOCAP(cur_dst)
        dst.copyFrom(patt_pos_mocap)
        assert(Arrays.equals(patt_pos_mocap.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(patt_pos_mocap)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- STATUSTEXT -------------------------")

    val pstatustext = CommunicationChannel.NEW.STATUSTEXT(cur)
    fill(pstatustext)
    onSTATUSTEXT(pstatustext)
    run {

        var dst = CommunicationChannel.NEW.STATUSTEXT(cur_dst)
        pstatustext.copyInto(dst)
        assert(Arrays.equals(pstatustext.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.STATUSTEXT(cur_dst)
        dst.copyFrom(pstatustext)
        assert(Arrays.equals(pstatustext.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pstatustext)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PING -------------------------")

    fill(TestChannel.NEW.PING(cur))
    val pping = com.company.demo.GroundControl.PING(cur)
    onPING(pping)
    run {
        cur_dst.init(com.company.demo.GroundControl.PING.meta)
        val dst = test_.PING(cur_dst)
        dst.copyFrom(pping)
        assert(Arrays.equals(pping.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pping)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GOPRO_GET_REQUEST -------------------------")

    val pgopro_get_request = CommunicationChannel.NEW.GOPRO_GET_REQUEST(cur)
    fill(pgopro_get_request)
    onGOPRO_GET_REQUEST(pgopro_get_request)
    run {

        var dst = CommunicationChannel.NEW.GOPRO_GET_REQUEST(cur_dst)
        pgopro_get_request.copyInto(dst)
        assert(Arrays.equals(pgopro_get_request.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GOPRO_GET_REQUEST(cur_dst)
        dst.copyFrom(pgopro_get_request)
        assert(Arrays.equals(pgopro_get_request.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pgopro_get_request)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CAMERA_CAPTURE_STATUS -------------------------")

    val pcamera_capture_status = CommunicationChannel.NEW.CAMERA_CAPTURE_STATUS(cur)
    fill(pcamera_capture_status)
    onCAMERA_CAPTURE_STATUS(pcamera_capture_status)
    run {

        var dst = CommunicationChannel.NEW.CAMERA_CAPTURE_STATUS(cur_dst)
        pcamera_capture_status.copyInto(dst)
        assert(Arrays.equals(pcamera_capture_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.CAMERA_CAPTURE_STATUS(cur_dst)
        dst.copyFrom(pcamera_capture_status)
        assert(Arrays.equals(pcamera_capture_status.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pcamera_capture_status)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GLOBAL_POSITION_INT -------------------------")

    fill(TestChannel.NEW.GLOBAL_POSITION_INT(cur))
    val pglobal_position_int = com.company.demo.GroundControl.GLOBAL_POSITION_INT(cur)
    onGLOBAL_POSITION_INT(pglobal_position_int)
    run {
        cur_dst.init(com.company.demo.GroundControl.GLOBAL_POSITION_INT.meta)
        val dst = test_.GLOBAL_POSITION_INT(cur_dst)
        dst.copyFrom(pglobal_position_int)
        assert(Arrays.equals(pglobal_position_int.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pglobal_position_int)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ENCAPSULATED_DATA -------------------------")

    val pencapsulated_data = CommunicationChannel.NEW.ENCAPSULATED_DATA(cur)
    fill(pencapsulated_data)
    onENCAPSULATED_DATA(pencapsulated_data)
    run {

        var dst = CommunicationChannel.NEW.ENCAPSULATED_DATA(cur_dst)
        pencapsulated_data.copyInto(dst)
        assert(Arrays.equals(pencapsulated_data.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.ENCAPSULATED_DATA(cur_dst)
        dst.copyFrom(pencapsulated_data)
        assert(Arrays.equals(pencapsulated_data.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pencapsulated_data)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS_INPUT -------------------------")

    val pgps_input = CommunicationChannel.NEW.GPS_INPUT(cur)
    fill(pgps_input)
    onGPS_INPUT(pgps_input)
    run {

        var dst = CommunicationChannel.NEW.GPS_INPUT(cur_dst)
        pgps_input.copyInto(dst)
        assert(Arrays.equals(pgps_input.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GPS_INPUT(cur_dst)
        dst.copyFrom(pgps_input)
        assert(Arrays.equals(pgps_input.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pgps_input)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- COMMAND_LONG -------------------------")

    fill(TestChannel.NEW.COMMAND_LONG(cur))
    val pcommand_long = com.company.demo.GroundControl.COMMAND_LONG(cur)
    onCOMMAND_LONG(pcommand_long)
    run {
        cur_dst.init(com.company.demo.GroundControl.COMMAND_LONG.meta)
        val dst = test_.COMMAND_LONG(cur_dst)
        dst.copyFrom(pcommand_long)
        assert(Arrays.equals(pcommand_long.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pcommand_long)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- COMPASSMOT_STATUS -------------------------")

    val pcompassmot_status = CommunicationChannel.NEW.COMPASSMOT_STATUS(cur)
    fill(pcompassmot_status)
    onCOMPASSMOT_STATUS(pcompassmot_status)
    run {

        var dst = CommunicationChannel.NEW.COMPASSMOT_STATUS(cur_dst)
        pcompassmot_status.copyInto(dst)
        assert(Arrays.equals(pcompassmot_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.COMPASSMOT_STATUS(cur_dst)
        dst.copyFrom(pcompassmot_status)
        assert(Arrays.equals(pcompassmot_status.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pcompassmot_status)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOG_REQUEST_DATA -------------------------")

    val plog_request_data = CommunicationChannel.NEW.LOG_REQUEST_DATA(cur)
    fill(plog_request_data)
    onLOG_REQUEST_DATA(plog_request_data)
    run {

        var dst = CommunicationChannel.NEW.LOG_REQUEST_DATA(cur_dst)
        plog_request_data.copyInto(dst)
        assert(Arrays.equals(plog_request_data.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOG_REQUEST_DATA(cur_dst)
        dst.copyFrom(plog_request_data)
        assert(Arrays.equals(plog_request_data.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(plog_request_data)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS_RAW_INT -------------------------")

    fill(TestChannel.NEW.GPS_RAW_INT(cur))
    val pgps_raw_int = com.company.demo.GroundControl.GPS_RAW_INT(cur)
    onGPS_RAW_INT(pgps_raw_int)
    run {
        cur_dst.init(com.company.demo.GroundControl.GPS_RAW_INT.meta)
        val dst = test_.GPS_RAW_INT(cur_dst)
        dst.copyFrom(pgps_raw_int)
        assert(Arrays.equals(pgps_raw_int.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pgps_raw_int)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CAMERA_STATUS -------------------------")

    val pcamera_status = CommunicationChannel.NEW.CAMERA_STATUS(cur)
    fill(pcamera_status)
    onCAMERA_STATUS(pcamera_status)
    run {

        var dst = CommunicationChannel.NEW.CAMERA_STATUS(cur_dst)
        pcamera_status.copyInto(dst)
        assert(Arrays.equals(pcamera_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.CAMERA_STATUS(cur_dst)
        dst.copyFrom(pcamera_status)
        assert(Arrays.equals(pcamera_status.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pcamera_status)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RC_CHANNELS_SCALED -------------------------")

    fill(TestChannel.NEW.RC_CHANNELS_SCALED(cur))
    val prc_channels_scaled = com.company.demo.GroundControl.RC_CHANNELS_SCALED(cur)
    onRC_CHANNELS_SCALED(prc_channels_scaled)
    run {
        cur_dst.init(com.company.demo.GroundControl.RC_CHANNELS_SCALED.meta)
        val dst = test_.RC_CHANNELS_SCALED(cur_dst)
        dst.copyFrom(prc_channels_scaled)
        assert(Arrays.equals(prc_channels_scaled.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(prc_channels_scaled)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CAMERA_SETTINGS -------------------------")

    val pcamera_settings = CommunicationChannel.NEW.CAMERA_SETTINGS(cur)
    fill(pcamera_settings)
    onCAMERA_SETTINGS(pcamera_settings)
    run {

        var dst = CommunicationChannel.NEW.CAMERA_SETTINGS(cur_dst)
        pcamera_settings.copyInto(dst)
        assert(Arrays.equals(pcamera_settings.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.CAMERA_SETTINGS(cur_dst)
        dst.copyFrom(pcamera_settings)
        assert(Arrays.equals(pcamera_settings.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pcamera_settings)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DEVICE_OP_READ_REPLY -------------------------")

    val pdevice_op_read_reply = CommunicationChannel.NEW.DEVICE_OP_READ_REPLY(cur)
    fill(pdevice_op_read_reply)
    onDEVICE_OP_READ_REPLY(pdevice_op_read_reply)
    run {

        var dst = CommunicationChannel.NEW.DEVICE_OP_READ_REPLY(cur_dst)
        pdevice_op_read_reply.copyInto(dst)
        assert(Arrays.equals(pdevice_op_read_reply.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DEVICE_OP_READ_REPLY(cur_dst)
        dst.copyFrom(pdevice_op_read_reply)
        assert(Arrays.equals(pdevice_op_read_reply.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdevice_op_read_reply)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RAW_PRESSURE -------------------------")

    fill(TestChannel.NEW.RAW_PRESSURE(cur))
    val praw_pressure = com.company.demo.GroundControl.RAW_PRESSURE(cur)
    onRAW_PRESSURE(praw_pressure)
    run {
        cur_dst.init(com.company.demo.GroundControl.RAW_PRESSURE.meta)
        val dst = test_.RAW_PRESSURE(cur_dst)
        dst.copyFrom(praw_pressure)
        assert(Arrays.equals(praw_pressure.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(praw_pressure)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DIGICAM_CONTROL -------------------------")

    val pdigicam_control = CommunicationChannel.NEW.DIGICAM_CONTROL(cur)
    fill(pdigicam_control)
    onDIGICAM_CONTROL(pdigicam_control)
    run {

        var dst = CommunicationChannel.NEW.DIGICAM_CONTROL(cur_dst)
        pdigicam_control.copyInto(dst)
        assert(Arrays.equals(pdigicam_control.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DIGICAM_CONTROL(cur_dst)
        dst.copyFrom(pdigicam_control)
        assert(Arrays.equals(pdigicam_control.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdigicam_control)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- NAMED_VALUE_FLOAT -------------------------")

    val pnamed_value_float = CommunicationChannel.NEW.NAMED_VALUE_FLOAT(cur)
    fill(pnamed_value_float)
    onNAMED_VALUE_FLOAT(pnamed_value_float)
    run {

        var dst = CommunicationChannel.NEW.NAMED_VALUE_FLOAT(cur_dst)
        pnamed_value_float.copyInto(dst)
        assert(Arrays.equals(pnamed_value_float.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.NAMED_VALUE_FLOAT(cur_dst)
        dst.copyFrom(pnamed_value_float)
        assert(Arrays.equals(pnamed_value_float.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pnamed_value_float)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GOPRO_HEARTBEAT -------------------------")

    val pgopro_heartbeat = CommunicationChannel.NEW.GOPRO_HEARTBEAT(cur)
    fill(pgopro_heartbeat)
    onGOPRO_HEARTBEAT(pgopro_heartbeat)
    run {

        var dst = CommunicationChannel.NEW.GOPRO_HEARTBEAT(cur_dst)
        pgopro_heartbeat.copyInto(dst)
        assert(Arrays.equals(pgopro_heartbeat.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GOPRO_HEARTBEAT(cur_dst)
        dst.copyFrom(pgopro_heartbeat)
        assert(Arrays.equals(pgopro_heartbeat.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pgopro_heartbeat)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ATTITUDE -------------------------")

    fill(TestChannel.NEW.ATTITUDE(cur))
    val pattitude = com.company.demo.GroundControl.ATTITUDE(cur)
    onATTITUDE(pattitude)
    run {
        cur_dst.init(com.company.demo.GroundControl.ATTITUDE.meta)
        val dst = test_.ATTITUDE(cur_dst)
        dst.copyFrom(pattitude)
        assert(Arrays.equals(pattitude.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pattitude)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_WRITE_PARTIAL_LIST -------------------------")

    fill(TestChannel.NEW.MISSION_WRITE_PARTIAL_LIST(cur))
    val pmission_write_partial_list = com.company.demo.GroundControl.MISSION_WRITE_PARTIAL_LIST(cur)
    onMISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_WRITE_PARTIAL_LIST.meta)
        val dst = test_.MISSION_WRITE_PARTIAL_LIST(cur_dst)
        dst.copyFrom(pmission_write_partial_list)
        assert(Arrays.equals(pmission_write_partial_list.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_write_partial_list)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- AHRS2 -------------------------")

    val pahrs2 = CommunicationChannel.NEW.AHRS2(cur)
    fill(pahrs2)
    onAHRS2(pahrs2)
    run {

        var dst = CommunicationChannel.NEW.AHRS2(cur_dst)
        pahrs2.copyInto(dst)
        assert(Arrays.equals(pahrs2.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.AHRS2(cur_dst)
        dst.copyFrom(pahrs2)
        assert(Arrays.equals(pahrs2.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pahrs2)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOG_ERASE -------------------------")

    val plog_erase = CommunicationChannel.NEW.LOG_ERASE(cur)
    fill(plog_erase)
    onLOG_ERASE(plog_erase)
    run {

        var dst = CommunicationChannel.NEW.LOG_ERASE(cur_dst)
        plog_erase.copyInto(dst)
        assert(Arrays.equals(plog_erase.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOG_ERASE(cur_dst)
        dst.copyFrom(plog_erase)
        assert(Arrays.equals(plog_erase.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(plog_erase)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- TERRAIN_REQUEST -------------------------")

    val pterrain_request = CommunicationChannel.NEW.TERRAIN_REQUEST(cur)
    fill(pterrain_request)
    onTERRAIN_REQUEST(pterrain_request)
    run {

        var dst = CommunicationChannel.NEW.TERRAIN_REQUEST(cur_dst)
        pterrain_request.copyInto(dst)
        assert(Arrays.equals(pterrain_request.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.TERRAIN_REQUEST(cur_dst)
        dst.copyFrom(pterrain_request)
        assert(Arrays.equals(pterrain_request.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pterrain_request)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MOUNT_STATUS -------------------------")

    val pmount_status = CommunicationChannel.NEW.MOUNT_STATUS(cur)
    fill(pmount_status)
    onMOUNT_STATUS(pmount_status)
    run {

        var dst = CommunicationChannel.NEW.MOUNT_STATUS(cur_dst)
        pmount_status.copyInto(dst)
        assert(Arrays.equals(pmount_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MOUNT_STATUS(cur_dst)
        dst.copyFrom(pmount_status)
        assert(Arrays.equals(pmount_status.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pmount_status)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MANUAL_SETPOINT -------------------------")

    fill(TestChannel.NEW.MANUAL_SETPOINT(cur))
    val pmanual_setpoint = com.company.demo.GroundControl.MANUAL_SETPOINT(cur)
    onMANUAL_SETPOINT(pmanual_setpoint)
    run {
        cur_dst.init(com.company.demo.GroundControl.MANUAL_SETPOINT.meta)
        val dst = test_.MANUAL_SETPOINT(cur_dst)
        dst.copyFrom(pmanual_setpoint)
        assert(Arrays.equals(pmanual_setpoint.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmanual_setpoint)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PID_TUNING -------------------------")

    val ppid_tuning = CommunicationChannel.NEW.PID_TUNING(cur)
    fill(ppid_tuning)
    onPID_TUNING(ppid_tuning)
    run {

        var dst = CommunicationChannel.NEW.PID_TUNING(cur_dst)
        ppid_tuning.copyInto(dst)
        assert(Arrays.equals(ppid_tuning.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PID_TUNING(cur_dst)
        dst.copyFrom(ppid_tuning)
        assert(Arrays.equals(ppid_tuning.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(ppid_tuning)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SAFETY_ALLOWED_AREA -------------------------")

    fill(TestChannel.NEW.SAFETY_ALLOWED_AREA(cur))
    val psafety_allowed_area = com.company.demo.GroundControl.SAFETY_ALLOWED_AREA(cur)
    onSAFETY_ALLOWED_AREA(psafety_allowed_area)
    run {
        cur_dst.init(com.company.demo.GroundControl.SAFETY_ALLOWED_AREA.meta)
        val dst = test_.SAFETY_ALLOWED_AREA(cur_dst)
        dst.copyFrom(psafety_allowed_area)
        assert(Arrays.equals(psafety_allowed_area.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(psafety_allowed_area)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- OPTICAL_FLOW_RAD -------------------------")

    val poptical_flow_rad = CommunicationChannel.NEW.OPTICAL_FLOW_RAD(cur)
    fill(poptical_flow_rad)
    onOPTICAL_FLOW_RAD(poptical_flow_rad)
    run {

        var dst = CommunicationChannel.NEW.OPTICAL_FLOW_RAD(cur_dst)
        poptical_flow_rad.copyInto(dst)
        assert(Arrays.equals(poptical_flow_rad.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.OPTICAL_FLOW_RAD(cur_dst)
        dst.copyFrom(poptical_flow_rad)
        assert(Arrays.equals(poptical_flow_rad.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(poptical_flow_rad)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOG_DATA -------------------------")

    val plog_data = CommunicationChannel.NEW.LOG_DATA(cur)
    fill(plog_data)
    onLOG_DATA(plog_data)
    run {

        var dst = CommunicationChannel.NEW.LOG_DATA(cur_dst)
        plog_data.copyInto(dst)
        assert(Arrays.equals(plog_data.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOG_DATA(cur_dst)
        dst.copyFrom(plog_data)
        assert(Arrays.equals(plog_data.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(plog_data)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_CLEAR_ALL -------------------------")

    fill(TestChannel.NEW.MISSION_CLEAR_ALL(cur))
    val pmission_clear_all = com.company.demo.GroundControl.MISSION_CLEAR_ALL(cur)
    onMISSION_CLEAR_ALL(pmission_clear_all)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_CLEAR_ALL.meta)
        val dst = test_.MISSION_CLEAR_ALL(cur_dst)
        dst.copyFrom(pmission_clear_all)
        assert(Arrays.equals(pmission_clear_all.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_clear_all)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- AHRS3 -------------------------")

    val pahrs3 = CommunicationChannel.NEW.AHRS3(cur)
    fill(pahrs3)
    onAHRS3(pahrs3)
    run {

        var dst = CommunicationChannel.NEW.AHRS3(cur_dst)
        pahrs3.copyInto(dst)
        assert(Arrays.equals(pahrs3.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.AHRS3(cur_dst)
        dst.copyFrom(pahrs3)
        assert(Arrays.equals(pahrs3.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pahrs3)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- VICON_POSITION_ESTIMATE -------------------------")

    val pvicon_position_estimate = CommunicationChannel.NEW.VICON_POSITION_ESTIMATE(cur)
    fill(pvicon_position_estimate)
    onVICON_POSITION_ESTIMATE(pvicon_position_estimate)
    run {

        var dst = CommunicationChannel.NEW.VICON_POSITION_ESTIMATE(cur_dst)
        pvicon_position_estimate.copyInto(dst)
        assert(Arrays.equals(pvicon_position_estimate.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.VICON_POSITION_ESTIMATE(cur_dst)
        dst.copyFrom(pvicon_position_estimate)
        assert(Arrays.equals(pvicon_position_estimate.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pvicon_position_estimate)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS2_RTK -------------------------")

    val pgps2_rtk = CommunicationChannel.NEW.GPS2_RTK(cur)
    fill(pgps2_rtk)
    onGPS2_RTK(pgps2_rtk)
    run {

        var dst = CommunicationChannel.NEW.GPS2_RTK(cur_dst)
        pgps2_rtk.copyInto(dst)
        assert(Arrays.equals(pgps2_rtk.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GPS2_RTK(cur_dst)
        dst.copyFrom(pgps2_rtk)
        assert(Arrays.equals(pgps2_rtk.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pgps2_rtk)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MAG_CAL_REPORT -------------------------")

    val pmag_cal_report = CommunicationChannel.NEW.MAG_CAL_REPORT(cur)
    fill(pmag_cal_report)
    onMAG_CAL_REPORT(pmag_cal_report)
    run {

        var dst = CommunicationChannel.NEW.MAG_CAL_REPORT(cur_dst)
        pmag_cal_report.copyInto(dst)
        assert(Arrays.equals(pmag_cal_report.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MAG_CAL_REPORT(cur_dst)
        dst.copyFrom(pmag_cal_report)
        assert(Arrays.equals(pmag_cal_report.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pmag_cal_report)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOG_REQUEST_LIST -------------------------")

    val plog_request_list = CommunicationChannel.NEW.LOG_REQUEST_LIST(cur)
    fill(plog_request_list)
    onLOG_REQUEST_LIST(plog_request_list)
    run {

        var dst = CommunicationChannel.NEW.LOG_REQUEST_LIST(cur_dst)
        plog_request_list.copyInto(dst)
        assert(Arrays.equals(plog_request_list.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOG_REQUEST_LIST(cur_dst)
        dst.copyFrom(plog_request_list)
        assert(Arrays.equals(plog_request_list.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(plog_request_list)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SCALED_PRESSURE -------------------------")

    fill(TestChannel.NEW.SCALED_PRESSURE(cur))
    val pscaled_pressure = com.company.demo.GroundControl.SCALED_PRESSURE(cur)
    onSCALED_PRESSURE(pscaled_pressure)
    run {
        cur_dst.init(com.company.demo.GroundControl.SCALED_PRESSURE.meta)
        val dst = test_.SCALED_PRESSURE(cur_dst)
        dst.copyFrom(pscaled_pressure)
        assert(Arrays.equals(pscaled_pressure.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pscaled_pressure)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- V2_EXTENSION -------------------------")

    val pv2_extension = CommunicationChannel.NEW.V2_EXTENSION(cur)
    fill(pv2_extension)
    onV2_EXTENSION(pv2_extension)
    run {

        var dst = CommunicationChannel.NEW.V2_EXTENSION(cur_dst)
        pv2_extension.copyInto(dst)
        assert(Arrays.equals(pv2_extension.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.V2_EXTENSION(cur_dst)
        dst.copyFrom(pv2_extension)
        assert(Arrays.equals(pv2_extension.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pv2_extension)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HEARTBEAT -------------------------")

    fill(TestChannel.NEW.HEARTBEAT(cur))
    val pheartbeat = com.company.demo.GroundControl.HEARTBEAT(cur)
    onHEARTBEAT(pheartbeat)
    run {
        cur_dst.init(com.company.demo.GroundControl.HEARTBEAT.meta)
        val dst = test_.HEARTBEAT(cur_dst)
        dst.copyFrom(pheartbeat)
        assert(Arrays.equals(pheartbeat.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pheartbeat)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_MAP_RC -------------------------")

    fill(TestChannel.NEW.PARAM_MAP_RC(cur))
    val pparam_map_rc = com.company.demo.GroundControl.PARAM_MAP_RC(cur)
    onPARAM_MAP_RC(pparam_map_rc)
    run {
        cur_dst.init(com.company.demo.GroundControl.PARAM_MAP_RC.meta)
        val dst = test_.PARAM_MAP_RC(cur_dst)
        dst.copyFrom(pparam_map_rc)
        assert(Arrays.equals(pparam_map_rc.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pparam_map_rc)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- POWER_STATUS -------------------------")

    val ppower_status = CommunicationChannel.NEW.POWER_STATUS(cur)
    fill(ppower_status)
    onPOWER_STATUS(ppower_status)
    run {

        var dst = CommunicationChannel.NEW.POWER_STATUS(cur_dst)
        ppower_status.copyInto(dst)
        assert(Arrays.equals(ppower_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.POWER_STATUS(cur_dst)
        dst.copyFrom(ppower_status)
        assert(Arrays.equals(ppower_status.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(ppower_status)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- REMOTE_LOG_DATA_BLOCK -------------------------")

    val premote_log_data_block = CommunicationChannel.NEW.REMOTE_LOG_DATA_BLOCK(cur)
    fill(premote_log_data_block)
    onREMOTE_LOG_DATA_BLOCK(premote_log_data_block)
    run {

        var dst = CommunicationChannel.NEW.REMOTE_LOG_DATA_BLOCK(cur_dst)
        premote_log_data_block.copyInto(dst)
        assert(Arrays.equals(premote_log_data_block.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.REMOTE_LOG_DATA_BLOCK(cur_dst)
        dst.copyFrom(premote_log_data_block)
        assert(Arrays.equals(premote_log_data_block.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(premote_log_data_block)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOGGING_DATA_ACKED -------------------------")

    val plogging_data_acked = CommunicationChannel.NEW.LOGGING_DATA_ACKED(cur)
    fill(plogging_data_acked)
    onLOGGING_DATA_ACKED(plogging_data_acked)
    run {

        var dst = CommunicationChannel.NEW.LOGGING_DATA_ACKED(cur_dst)
        plogging_data_acked.copyInto(dst)
        assert(Arrays.equals(plogging_data_acked.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOGGING_DATA_ACKED(cur_dst)
        dst.copyFrom(plogging_data_acked)
        assert(Arrays.equals(plogging_data_acked.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(plogging_data_acked)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- TERRAIN_CHECK -------------------------")

    val pterrain_check = CommunicationChannel.NEW.TERRAIN_CHECK(cur)
    fill(pterrain_check)
    onTERRAIN_CHECK(pterrain_check)
    run {

        var dst = CommunicationChannel.NEW.TERRAIN_CHECK(cur_dst)
        pterrain_check.copyInto(dst)
        assert(Arrays.equals(pterrain_check.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.TERRAIN_CHECK(cur_dst)
        dst.copyFrom(pterrain_check)
        assert(Arrays.equals(pterrain_check.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pterrain_check)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MOUNT_CONFIGURE -------------------------")

    val pmount_configure = CommunicationChannel.NEW.MOUNT_CONFIGURE(cur)
    fill(pmount_configure)
    onMOUNT_CONFIGURE(pmount_configure)
    run {

        var dst = CommunicationChannel.NEW.MOUNT_CONFIGURE(cur_dst)
        pmount_configure.copyInto(dst)
        assert(Arrays.equals(pmount_configure.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MOUNT_CONFIGURE(cur_dst)
        dst.copyFrom(pmount_configure)
        assert(Arrays.equals(pmount_configure.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pmount_configure)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_REQUEST_INT -------------------------")

    fill(TestChannel.NEW.MISSION_REQUEST_INT(cur))
    val pmission_request_int = com.company.demo.GroundControl.MISSION_REQUEST_INT(cur)
    onMISSION_REQUEST_INT(pmission_request_int)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_REQUEST_INT.meta)
        val dst = test_.MISSION_REQUEST_INT(cur_dst)
        dst.copyFrom(pmission_request_int)
        assert(Arrays.equals(pmission_request_int.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_request_int)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET -------------------------")

    fill(TestChannel.NEW.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(cur))
    val plocal_position_ned_system_global_offset = com.company.demo.GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(cur)
    onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset)
    run {
        cur_dst.init(com.company.demo.GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.meta)
        val dst = test_.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(cur_dst)
        dst.copyFrom(plocal_position_ned_system_global_offset)
        assert(Arrays.equals(plocal_position_ned_system_global_offset.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(plocal_position_ned_system_global_offset)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- COMMAND_ACK -------------------------")

    fill(TestChannel.NEW.COMMAND_ACK(cur))
    val pcommand_ack = com.company.demo.GroundControl.COMMAND_ACK(cur)
    onCOMMAND_ACK(pcommand_ack)
    run {
        cur_dst.init(com.company.demo.GroundControl.COMMAND_ACK.meta)
        val dst = test_.COMMAND_ACK(cur_dst)
        dst.copyFrom(pcommand_ack)
        assert(Arrays.equals(pcommand_ack.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pcommand_ack)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DATA_STREAM -------------------------")

    fill(TestChannel.NEW.DATA_STREAM(cur))
    val pdata_stream = com.company.demo.GroundControl.DATA_STREAM(cur)
    onDATA_STREAM(pdata_stream)
    run {
        cur_dst.init(com.company.demo.GroundControl.DATA_STREAM.meta)
        val dst = test_.DATA_STREAM(cur_dst)
        dst.copyFrom(pdata_stream)
        assert(Arrays.equals(pdata_stream.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pdata_stream)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_REQUEST -------------------------")

    fill(TestChannel.NEW.MISSION_REQUEST(cur))
    val pmission_request = com.company.demo.GroundControl.MISSION_REQUEST(cur)
    onMISSION_REQUEST(pmission_request)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_REQUEST.meta)
        val dst = test_.MISSION_REQUEST(cur_dst)
        dst.copyFrom(pmission_request)
        assert(Arrays.equals(pmission_request.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_request)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- TERRAIN_REPORT -------------------------")

    val pterrain_report = CommunicationChannel.NEW.TERRAIN_REPORT(cur)
    fill(pterrain_report)
    onTERRAIN_REPORT(pterrain_report)
    run {

        var dst = CommunicationChannel.NEW.TERRAIN_REPORT(cur_dst)
        pterrain_report.copyInto(dst)
        assert(Arrays.equals(pterrain_report.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.TERRAIN_REPORT(cur_dst)
        dst.copyFrom(pterrain_report)
        assert(Arrays.equals(pterrain_report.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pterrain_report)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_HOME_POSITION -------------------------")

    val pset_home_position = CommunicationChannel.NEW.SET_HOME_POSITION(cur)
    fill(pset_home_position)
    onSET_HOME_POSITION(pset_home_position)
    run {

        var dst = CommunicationChannel.NEW.SET_HOME_POSITION(cur_dst)
        pset_home_position.copyInto(dst)
        assert(Arrays.equals(pset_home_position.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SET_HOME_POSITION(cur_dst)
        dst.copyFrom(pset_home_position)
        assert(Arrays.equals(pset_home_position.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pset_home_position)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SwitchModeCommand -------------------------")
    if (CommunicationChannel_instance.sendSwitchModeCommand()) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIL_RC_INPUTS_RAW -------------------------")

    fill(TestChannel.NEW.HIL_RC_INPUTS_RAW(cur))
    val phil_rc_inputs_raw = com.company.demo.GroundControl.HIL_RC_INPUTS_RAW(cur)
    onHIL_RC_INPUTS_RAW(phil_rc_inputs_raw)
    run {
        cur_dst.init(com.company.demo.GroundControl.HIL_RC_INPUTS_RAW.meta)
        val dst = test_.HIL_RC_INPUTS_RAW(cur_dst)
        dst.copyFrom(phil_rc_inputs_raw)
        assert(Arrays.equals(phil_rc_inputs_raw.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(phil_rc_inputs_raw)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SCALED_IMU3 -------------------------")

    val pscaled_imu3 = CommunicationChannel.NEW.SCALED_IMU3(cur)
    fill(pscaled_imu3)
    onSCALED_IMU3(pscaled_imu3)
    run {

        var dst = CommunicationChannel.NEW.SCALED_IMU3(cur_dst)
        pscaled_imu3.copyInto(dst)
        assert(Arrays.equals(pscaled_imu3.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SCALED_IMU3(cur_dst)
        dst.copyFrom(pscaled_imu3)
        assert(Arrays.equals(pscaled_imu3.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pscaled_imu3)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_MODE -------------------------")

    fill(TestChannel.NEW.SET_MODE(cur))
    val pset_mode = com.company.demo.GroundControl.SET_MODE(cur)
    onSET_MODE(pset_mode)
    run {
        cur_dst.init(com.company.demo.GroundControl.SET_MODE.meta)
        val dst = test_.SET_MODE(cur_dst)
        dst.copyFrom(pset_mode)
        assert(Arrays.equals(pset_mode.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pset_mode)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MOUNT_CONTROL -------------------------")

    val pmount_control = CommunicationChannel.NEW.MOUNT_CONTROL(cur)
    fill(pmount_control)
    onMOUNT_CONTROL(pmount_control)
    run {

        var dst = CommunicationChannel.NEW.MOUNT_CONTROL(cur_dst)
        pmount_control.copyInto(dst)
        assert(Arrays.equals(pmount_control.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MOUNT_CONTROL(cur_dst)
        dst.copyFrom(pmount_control)
        assert(Arrays.equals(pmount_control.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pmount_control)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- POSITION_TARGET_GLOBAL_INT -------------------------")

    fill(TestChannel.NEW.POSITION_TARGET_GLOBAL_INT(cur))
    val pposition_target_global_int = com.company.demo.GroundControl.POSITION_TARGET_GLOBAL_INT(cur)
    onPOSITION_TARGET_GLOBAL_INT(pposition_target_global_int)
    run {
        cur_dst.init(com.company.demo.GroundControl.POSITION_TARGET_GLOBAL_INT.meta)
        val dst = test_.POSITION_TARGET_GLOBAL_INT(cur_dst)
        dst.copyFrom(pposition_target_global_int)
        assert(Arrays.equals(pposition_target_global_int.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pposition_target_global_int)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LED_CONTROL -------------------------")

    val pled_control = CommunicationChannel.NEW.LED_CONTROL(cur)
    fill(pled_control)
    onLED_CONTROL(pled_control)
    run {

        var dst = CommunicationChannel.NEW.LED_CONTROL(cur_dst)
        pled_control.copyInto(dst)
        assert(Arrays.equals(pled_control.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LED_CONTROL(cur_dst)
        dst.copyFrom(pled_control)
        assert(Arrays.equals(pled_control.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pled_control)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SIM_STATE -------------------------")

    val psim_state = CommunicationChannel.NEW.SIM_STATE(cur)
    fill(psim_state)
    onSIM_STATE(psim_state)
    run {

        var dst = CommunicationChannel.NEW.SIM_STATE(cur_dst)
        psim_state.copyInto(dst)
        assert(Arrays.equals(psim_state.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SIM_STATE(cur_dst)
        dst.copyFrom(psim_state)
        assert(Arrays.equals(psim_state.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(psim_state)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- WIFI_CONFIG_AP -------------------------")

    val pwifi_config_ap = CommunicationChannel.NEW.WIFI_CONFIG_AP(cur)
    fill(pwifi_config_ap)
    onWIFI_CONFIG_AP(pwifi_config_ap)
    run {

        var dst = CommunicationChannel.NEW.WIFI_CONFIG_AP(cur_dst)
        pwifi_config_ap.copyInto(dst)
        assert(Arrays.equals(pwifi_config_ap.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.WIFI_CONFIG_AP(cur_dst)
        dst.copyFrom(pwifi_config_ap)
        assert(Arrays.equals(pwifi_config_ap.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pwifi_config_ap)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DATA96 -------------------------")

    val pdata96 = CommunicationChannel.NEW.DATA96(cur)
    fill(pdata96)
    onDATA96(pdata96)
    run {

        var dst = CommunicationChannel.NEW.DATA96(cur_dst)
        pdata96.copyInto(dst)
        assert(Arrays.equals(pdata96.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DATA96(cur_dst)
        dst.copyFrom(pdata96)
        assert(Arrays.equals(pdata96.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdata96)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- FLIGHT_INFORMATION -------------------------")

    val pflight_information = CommunicationChannel.NEW.FLIGHT_INFORMATION(cur)
    fill(pflight_information)
    onFLIGHT_INFORMATION(pflight_information)
    run {

        var dst = CommunicationChannel.NEW.FLIGHT_INFORMATION(cur_dst)
        pflight_information.copyInto(dst)
        assert(Arrays.equals(pflight_information.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.FLIGHT_INFORMATION(cur_dst)
        dst.copyFrom(pflight_information)
        assert(Arrays.equals(pflight_information.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pflight_information)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RC_CHANNELS_RAW -------------------------")

    fill(TestChannel.NEW.RC_CHANNELS_RAW(cur))
    val prc_channels_raw = com.company.demo.GroundControl.RC_CHANNELS_RAW(cur)
    onRC_CHANNELS_RAW(prc_channels_raw)
    run {
        cur_dst.init(com.company.demo.GroundControl.RC_CHANNELS_RAW.meta)
        val dst = test_.RC_CHANNELS_RAW(cur_dst)
        dst.copyFrom(prc_channels_raw)
        assert(Arrays.equals(prc_channels_raw.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(prc_channels_raw)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SERVO_OUTPUT_RAW -------------------------")

    fill(TestChannel.NEW.SERVO_OUTPUT_RAW(cur))
    val pservo_output_raw = com.company.demo.GroundControl.SERVO_OUTPUT_RAW(cur)
    onSERVO_OUTPUT_RAW(pservo_output_raw)
    run {
        cur_dst.init(com.company.demo.GroundControl.SERVO_OUTPUT_RAW.meta)
        val dst = test_.SERVO_OUTPUT_RAW(cur_dst)
        dst.copyFrom(pservo_output_raw)
        assert(Arrays.equals(pservo_output_raw.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pservo_output_raw)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MEMINFO -------------------------")

    val pmeminfo = CommunicationChannel.NEW.MEMINFO(cur)
    fill(pmeminfo)
    onMEMINFO(pmeminfo)
    run {

        var dst = CommunicationChannel.NEW.MEMINFO(cur_dst)
        pmeminfo.copyInto(dst)
        assert(Arrays.equals(pmeminfo.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MEMINFO(cur_dst)
        dst.copyFrom(pmeminfo)
        assert(Arrays.equals(pmeminfo.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pmeminfo)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_ITEM_REACHED -------------------------")

    fill(TestChannel.NEW.MISSION_ITEM_REACHED(cur))
    val pmission_item_reached = com.company.demo.GroundControl.MISSION_ITEM_REACHED(cur)
    onMISSION_ITEM_REACHED(pmission_item_reached)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_ITEM_REACHED.meta)
        val dst = test_.MISSION_ITEM_REACHED(cur_dst)
        dst.copyFrom(pmission_item_reached)
        assert(Arrays.equals(pmission_item_reached.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_item_reached)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOGGING_ACK -------------------------")

    val plogging_ack = CommunicationChannel.NEW.LOGGING_ACK(cur)
    fill(plogging_ack)
    onLOGGING_ACK(plogging_ack)
    run {

        var dst = CommunicationChannel.NEW.LOGGING_ACK(cur_dst)
        plogging_ack.copyInto(dst)
        assert(Arrays.equals(plogging_ack.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOGGING_ACK(cur_dst)
        dst.copyFrom(plogging_ack)
        assert(Arrays.equals(plogging_ack.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(plogging_ack)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- VISION_SPEED_ESTIMATE -------------------------")

    val pvision_speed_estimate = CommunicationChannel.NEW.VISION_SPEED_ESTIMATE(cur)
    fill(pvision_speed_estimate)
    onVISION_SPEED_ESTIMATE(pvision_speed_estimate)
    run {

        var dst = CommunicationChannel.NEW.VISION_SPEED_ESTIMATE(cur_dst)
        pvision_speed_estimate.copyInto(dst)
        assert(Arrays.equals(pvision_speed_estimate.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.VISION_SPEED_ESTIMATE(cur_dst)
        dst.copyFrom(pvision_speed_estimate)
        assert(Arrays.equals(pvision_speed_estimate.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pvision_speed_estimate)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DEBUG_VECT -------------------------")

    val pdebug_vect = CommunicationChannel.NEW.DEBUG_VECT(cur)
    fill(pdebug_vect)
    onDEBUG_VECT(pdebug_vect)
    run {

        var dst = CommunicationChannel.NEW.DEBUG_VECT(cur_dst)
        pdebug_vect.copyInto(dst)
        assert(Arrays.equals(pdebug_vect.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DEBUG_VECT(cur_dst)
        dst.copyFrom(pdebug_vect)
        assert(Arrays.equals(pdebug_vect.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pdebug_vect)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOG_REQUEST_END -------------------------")

    val plog_request_end = CommunicationChannel.NEW.LOG_REQUEST_END(cur)
    fill(plog_request_end)
    onLOG_REQUEST_END(plog_request_end)
    run {

        var dst = CommunicationChannel.NEW.LOG_REQUEST_END(cur_dst)
        plog_request_end.copyInto(dst)
        assert(Arrays.equals(plog_request_end.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOG_REQUEST_END(cur_dst)
        dst.copyFrom(plog_request_end)
        assert(Arrays.equals(plog_request_end.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(plog_request_end)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_ACK -------------------------")

    fill(TestChannel.NEW.MISSION_ACK(cur))
    val pmission_ack = com.company.demo.GroundControl.MISSION_ACK(cur)
    onMISSION_ACK(pmission_ack)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_ACK.meta)
        val dst = test_.MISSION_ACK(cur_dst)
        dst.copyFrom(pmission_ack)
        assert(Arrays.equals(pmission_ack.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_ack)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CHANGE_OPERATOR_CONTROL_ACK -------------------------")

    fill(TestChannel.NEW.CHANGE_OPERATOR_CONTROL_ACK(cur))
    val pchange_operator_control_ack = com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL_ACK(cur)
    onCHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack)
    run {
        cur_dst.init(com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL_ACK.meta)
        val dst = test_.CHANGE_OPERATOR_CONTROL_ACK(cur_dst)
        dst.copyFrom(pchange_operator_control_ack)
        assert(Arrays.equals(pchange_operator_control_ack.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pchange_operator_control_ack)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_CURRENT -------------------------")

    fill(TestChannel.NEW.MISSION_CURRENT(cur))
    val pmission_current = com.company.demo.GroundControl.MISSION_CURRENT(cur)
    onMISSION_CURRENT(pmission_current)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_CURRENT.meta)
        val dst = test_.MISSION_CURRENT(cur_dst)
        dst.copyFrom(pmission_current)
        assert(Arrays.equals(pmission_current.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_current)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SYSTEM_TIME -------------------------")

    fill(TestChannel.NEW.SYSTEM_TIME(cur))
    val psystem_time = com.company.demo.GroundControl.SYSTEM_TIME(cur)
    onSYSTEM_TIME(psystem_time)
    run {
        cur_dst.init(com.company.demo.GroundControl.SYSTEM_TIME.meta)
        val dst = test_.SYSTEM_TIME(cur_dst)
        dst.copyFrom(psystem_time)
        assert(Arrays.equals(psystem_time.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(psystem_time)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CAMERA_TRIGGER -------------------------")

    val pcamera_trigger = CommunicationChannel.NEW.CAMERA_TRIGGER(cur)
    fill(pcamera_trigger)
    onCAMERA_TRIGGER(pcamera_trigger)
    run {

        var dst = CommunicationChannel.NEW.CAMERA_TRIGGER(cur_dst)
        pcamera_trigger.copyInto(dst)
        assert(Arrays.equals(pcamera_trigger.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.CAMERA_TRIGGER(cur_dst)
        dst.copyFrom(pcamera_trigger)
        assert(Arrays.equals(pcamera_trigger.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pcamera_trigger)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GOPRO_SET_RESPONSE -------------------------")

    val pgopro_set_response = CommunicationChannel.NEW.GOPRO_SET_RESPONSE(cur)
    fill(pgopro_set_response)
    onGOPRO_SET_RESPONSE(pgopro_set_response)
    run {

        var dst = CommunicationChannel.NEW.GOPRO_SET_RESPONSE(cur_dst)
        pgopro_set_response.copyInto(dst)
        assert(Arrays.equals(pgopro_set_response.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GOPRO_SET_RESPONSE(cur_dst)
        dst.copyFrom(pgopro_set_response)
        assert(Arrays.equals(pgopro_set_response.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pgopro_set_response)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- VISION_POSITION_ESTIMATE -------------------------")

    fill(TestChannel.NEW.VISION_POSITION_ESTIMATE(cur))
    val pvision_position_estimate = com.company.demo.GroundControl.VISION_POSITION_ESTIMATE(cur)
    onVISION_POSITION_ESTIMATE(pvision_position_estimate)
    run {
        cur_dst.init(com.company.demo.GroundControl.VISION_POSITION_ESTIMATE.meta)
        val dst = test_.VISION_POSITION_ESTIMATE(cur_dst)
        dst.copyFrom(pvision_position_estimate)
        assert(Arrays.equals(pvision_position_estimate.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pvision_position_estimate)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MANUAL_CONTROL -------------------------")

    fill(TestChannel.NEW.MANUAL_CONTROL(cur))
    val pmanual_control = com.company.demo.GroundControl.MANUAL_CONTROL(cur)
    onMANUAL_CONTROL(pmanual_control)
    run {
        cur_dst.init(com.company.demo.GroundControl.MANUAL_CONTROL.meta)
        val dst = test_.MANUAL_CONTROL(cur_dst)
        dst.copyFrom(pmanual_control)
        assert(Arrays.equals(pmanual_control.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmanual_control)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RC_CHANNELS -------------------------")

    fill(TestChannel.NEW.RC_CHANNELS(cur))
    val prc_channels = com.company.demo.GroundControl.RC_CHANNELS(cur)
    onRC_CHANNELS(prc_channels)
    run {
        cur_dst.init(com.company.demo.GroundControl.RC_CHANNELS.meta)
        val dst = test_.RC_CHANNELS(cur_dst)
        dst.copyFrom(prc_channels)
        assert(Arrays.equals(prc_channels.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(prc_channels)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PROTOCOL_VERSION -------------------------")

    val pprotocol_version = CommunicationChannel.NEW.PROTOCOL_VERSION(cur)
    fill(pprotocol_version)
    onPROTOCOL_VERSION(pprotocol_version)
    run {

        var dst = CommunicationChannel.NEW.PROTOCOL_VERSION(cur_dst)
        pprotocol_version.copyInto(dst)
        assert(Arrays.equals(pprotocol_version.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PROTOCOL_VERSION(cur_dst)
        dst.copyFrom(pprotocol_version)
        assert(Arrays.equals(pprotocol_version.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pprotocol_version)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RALLY_FETCH_POINT -------------------------")

    val prally_fetch_point = CommunicationChannel.NEW.RALLY_FETCH_POINT(cur)
    fill(prally_fetch_point)
    onRALLY_FETCH_POINT(prally_fetch_point)
    run {

        var dst = CommunicationChannel.NEW.RALLY_FETCH_POINT(cur_dst)
        prally_fetch_point.copyInto(dst)
        assert(Arrays.equals(prally_fetch_point.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.RALLY_FETCH_POINT(cur_dst)
        dst.copyFrom(prally_fetch_point)
        assert(Arrays.equals(prally_fetch_point.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(prally_fetch_point)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_VALUE -------------------------")

    fill(TestChannel.NEW.PARAM_VALUE(cur))
    val pparam_value = com.company.demo.GroundControl.PARAM_VALUE(cur)
    onPARAM_VALUE(pparam_value)
    run {
        cur_dst.init(com.company.demo.GroundControl.PARAM_VALUE.meta)
        val dst = test_.PARAM_VALUE(cur_dst)
        dst.copyFrom(pparam_value)
        assert(Arrays.equals(pparam_value.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pparam_value)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- BATTERY_STATUS -------------------------")

    val pbattery_status = CommunicationChannel.NEW.BATTERY_STATUS(cur)
    fill(pbattery_status)
    onBATTERY_STATUS(pbattery_status)
    run {

        var dst = CommunicationChannel.NEW.BATTERY_STATUS(cur_dst)
        pbattery_status.copyInto(dst)
        assert(Arrays.equals(pbattery_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.BATTERY_STATUS(cur_dst)
        dst.copyFrom(pbattery_status)
        assert(Arrays.equals(pbattery_status.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pbattery_status)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SERIAL_CONTROL -------------------------")

    val pserial_control = CommunicationChannel.NEW.SERIAL_CONTROL(cur)
    fill(pserial_control)
    onSERIAL_CONTROL(pserial_control)
    run {

        var dst = CommunicationChannel.NEW.SERIAL_CONTROL(cur_dst)
        pserial_control.copyInto(dst)
        assert(Arrays.equals(pserial_control.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SERIAL_CONTROL(cur_dst)
        dst.copyFrom(pserial_control)
        assert(Arrays.equals(pserial_control.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pserial_control)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_POSITION_TARGET_LOCAL_NED -------------------------")

    fill(TestChannel.NEW.SET_POSITION_TARGET_LOCAL_NED(cur))
    val pset_position_target_local_ned = com.company.demo.GroundControl.SET_POSITION_TARGET_LOCAL_NED(cur)
    onSET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned)
    run {
        cur_dst.init(com.company.demo.GroundControl.SET_POSITION_TARGET_LOCAL_NED.meta)
        val dst = test_.SET_POSITION_TARGET_LOCAL_NED(cur_dst)
        dst.copyFrom(pset_position_target_local_ned)
        assert(Arrays.equals(pset_position_target_local_ned.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pset_position_target_local_ned)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MOUNT_ORIENTATION -------------------------")

    val pmount_orientation = CommunicationChannel.NEW.MOUNT_ORIENTATION(cur)
    fill(pmount_orientation)
    onMOUNT_ORIENTATION(pmount_orientation)
    run {

        var dst = CommunicationChannel.NEW.MOUNT_ORIENTATION(cur_dst)
        pmount_orientation.copyInto(dst)
        assert(Arrays.equals(pmount_orientation.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MOUNT_ORIENTATION(cur_dst)
        dst.copyFrom(pmount_orientation)
        assert(Arrays.equals(pmount_orientation.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pmount_orientation)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_GPS_GLOBAL_ORIGIN -------------------------")

    fill(TestChannel.NEW.SET_GPS_GLOBAL_ORIGIN(cur))
    val pset_gps_global_origin = com.company.demo.GroundControl.SET_GPS_GLOBAL_ORIGIN(cur)
    onSET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin)
    run {
        cur_dst.init(com.company.demo.GroundControl.SET_GPS_GLOBAL_ORIGIN.meta)
        val dst = test_.SET_GPS_GLOBAL_ORIGIN(cur_dst)
        dst.copyFrom(pset_gps_global_origin)
        assert(Arrays.equals(pset_gps_global_origin.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pset_gps_global_origin)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_EXT_SET -------------------------")

    val pparam_ext_set = CommunicationChannel.NEW.PARAM_EXT_SET(cur)
    fill(pparam_ext_set)
    onPARAM_EXT_SET(pparam_ext_set)
    run {

        var dst = CommunicationChannel.NEW.PARAM_EXT_SET(cur_dst)
        pparam_ext_set.copyInto(dst)
        assert(Arrays.equals(pparam_ext_set.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PARAM_EXT_SET(cur_dst)
        dst.copyFrom(pparam_ext_set)
        assert(Arrays.equals(pparam_ext_set.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pparam_ext_set)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- AUTOPILOT_VERSION -------------------------")

    val pautopilot_version = CommunicationChannel.NEW.AUTOPILOT_VERSION(cur)
    fill(pautopilot_version)
    onAUTOPILOT_VERSION(pautopilot_version)
    run {

        var dst = CommunicationChannel.NEW.AUTOPILOT_VERSION(cur_dst)
        pautopilot_version.copyInto(dst)
        assert(Arrays.equals(pautopilot_version.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.AUTOPILOT_VERSION(cur_dst)
        dst.copyFrom(pautopilot_version)
        assert(Arrays.equals(pautopilot_version.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pautopilot_version)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_REQUEST_LIST -------------------------")

    fill(TestChannel.NEW.MISSION_REQUEST_LIST(cur))
    val pmission_request_list = com.company.demo.GroundControl.MISSION_REQUEST_LIST(cur)
    onMISSION_REQUEST_LIST(pmission_request_list)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_REQUEST_LIST.meta)
        val dst = test_.MISSION_REQUEST_LIST(cur_dst)
        dst.copyFrom(pmission_request_list)
        assert(Arrays.equals(pmission_request_list.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_request_list)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SIMSTATE -------------------------")

    val psimstate = CommunicationChannel.NEW.SIMSTATE(cur)
    fill(psimstate)
    onSIMSTATE(psimstate)
    run {

        var dst = CommunicationChannel.NEW.SIMSTATE(cur_dst)
        psimstate.copyInto(dst)
        assert(Arrays.equals(psimstate.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SIMSTATE(cur_dst)
        dst.copyFrom(psimstate)
        assert(Arrays.equals(psimstate.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(psimstate)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_VIDEO_STREAM_SETTINGS -------------------------")

    val pset_video_stream_settings = CommunicationChannel.NEW.SET_VIDEO_STREAM_SETTINGS(cur)
    fill(pset_video_stream_settings)
    onSET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings)
    run {

        var dst = CommunicationChannel.NEW.SET_VIDEO_STREAM_SETTINGS(cur_dst)
        pset_video_stream_settings.copyInto(dst)
        assert(Arrays.equals(pset_video_stream_settings.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SET_VIDEO_STREAM_SETTINGS(cur_dst)
        dst.copyFrom(pset_video_stream_settings)
        assert(Arrays.equals(pset_video_stream_settings.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pset_video_stream_settings)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PLAY_TUNE -------------------------")

    val pplay_tune = CommunicationChannel.NEW.PLAY_TUNE(cur)
    fill(pplay_tune)
    onPLAY_TUNE(pplay_tune)
    run {

        var dst = CommunicationChannel.NEW.PLAY_TUNE(cur_dst)
        pplay_tune.copyInto(dst)
        assert(Arrays.equals(pplay_tune.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PLAY_TUNE(cur_dst)
        dst.copyFrom(pplay_tune)
        assert(Arrays.equals(pplay_tune.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pplay_tune)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DIGICAM_CONFIGURE -------------------------")

    val pdigicam_configure = CommunicationChannel.NEW.DIGICAM_CONFIGURE(cur)
    fill(pdigicam_configure)
    onDIGICAM_CONFIGURE(pdigicam_configure)
    run {

        var dst = CommunicationChannel.NEW.DIGICAM_CONFIGURE(cur_dst)
        pdigicam_configure.copyInto(dst)
        assert(Arrays.equals(pdigicam_configure.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DIGICAM_CONFIGURE(cur_dst)
        dst.copyFrom(pdigicam_configure)
        assert(Arrays.equals(pdigicam_configure.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdigicam_configure)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SCALED_PRESSURE3 -------------------------")

    val pscaled_pressure3 = CommunicationChannel.NEW.SCALED_PRESSURE3(cur)
    fill(pscaled_pressure3)
    onSCALED_PRESSURE3(pscaled_pressure3)
    run {

        var dst = CommunicationChannel.NEW.SCALED_PRESSURE3(cur_dst)
        pscaled_pressure3.copyInto(dst)
        assert(Arrays.equals(pscaled_pressure3.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SCALED_PRESSURE3(cur_dst)
        dst.copyFrom(pscaled_pressure3)
        assert(Arrays.equals(pscaled_pressure3.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pscaled_pressure3)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_REQUEST_PARTIAL_LIST -------------------------")

    fill(TestChannel.NEW.MISSION_REQUEST_PARTIAL_LIST(cur))
    val pmission_request_partial_list = com.company.demo.GroundControl.MISSION_REQUEST_PARTIAL_LIST(cur)
    onMISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_REQUEST_PARTIAL_LIST.meta)
        val dst = test_.MISSION_REQUEST_PARTIAL_LIST(cur_dst)
        dst.copyFrom(pmission_request_partial_list)
        assert(Arrays.equals(pmission_request_partial_list.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_request_partial_list)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- PARAM_EXT_ACK -------------------------")

    val pparam_ext_ack = CommunicationChannel.NEW.PARAM_EXT_ACK(cur)
    fill(pparam_ext_ack)
    onPARAM_EXT_ACK(pparam_ext_ack)
    run {

        var dst = CommunicationChannel.NEW.PARAM_EXT_ACK(cur_dst)
        pparam_ext_ack.copyInto(dst)
        assert(Arrays.equals(pparam_ext_ack.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.PARAM_EXT_ACK(cur_dst)
        dst.copyFrom(pparam_ext_ack)
        assert(Arrays.equals(pparam_ext_ack.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pparam_ext_ack)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- UAVCAN_NODE_INFO -------------------------")

    val puavcan_node_info = CommunicationChannel.NEW.UAVCAN_NODE_INFO(cur)
    fill(puavcan_node_info)
    onUAVCAN_NODE_INFO(puavcan_node_info)
    run {

        var dst = CommunicationChannel.NEW.UAVCAN_NODE_INFO(cur_dst)
        puavcan_node_info.copyInto(dst)
        assert(Arrays.equals(puavcan_node_info.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.UAVCAN_NODE_INFO(cur_dst)
        dst.copyFrom(puavcan_node_info)
        assert(Arrays.equals(puavcan_node_info.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(puavcan_node_info)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DATA16 -------------------------")

    val pdata16 = CommunicationChannel.NEW.DATA16(cur)
    fill(pdata16)
    onDATA16(pdata16)
    run {

        var dst = CommunicationChannel.NEW.DATA16(cur_dst)
        pdata16.copyInto(dst)
        assert(Arrays.equals(pdata16.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DATA16(cur_dst)
        dst.copyFrom(pdata16)
        assert(Arrays.equals(pdata16.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdata16)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SET_MAG_OFFSETS -------------------------")

    val pset_mag_offsets = CommunicationChannel.NEW.SET_MAG_OFFSETS(cur)
    fill(pset_mag_offsets)
    onSET_MAG_OFFSETS(pset_mag_offsets)
    run {

        var dst = CommunicationChannel.NEW.SET_MAG_OFFSETS(cur_dst)
        pset_mag_offsets.copyInto(dst)
        assert(Arrays.equals(pset_mag_offsets.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SET_MAG_OFFSETS(cur_dst)
        dst.copyFrom(pset_mag_offsets)
        assert(Arrays.equals(pset_mag_offsets.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pset_mag_offsets)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- AP_ADC -------------------------")

    val pap_adc = CommunicationChannel.NEW.AP_ADC(cur)
    fill(pap_adc)
    onAP_ADC(pap_adc)
    run {

        var dst = CommunicationChannel.NEW.AP_ADC(cur_dst)
        pap_adc.copyInto(dst)
        assert(Arrays.equals(pap_adc.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.AP_ADC(cur_dst)
        dst.copyFrom(pap_adc)
        assert(Arrays.equals(pap_adc.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pap_adc)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- WIND -------------------------")

    val pwind = CommunicationChannel.NEW.WIND(cur)
    fill(pwind)
    onWIND(pwind)
    run {

        var dst = CommunicationChannel.NEW.WIND(cur_dst)
        pwind.copyInto(dst)
        assert(Arrays.equals(pwind.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.WIND(cur_dst)
        dst.copyFrom(pwind)
        assert(Arrays.equals(pwind.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pwind)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- AUTOPILOT_VERSION_REQUEST -------------------------")

    val pautopilot_version_request = CommunicationChannel.NEW.AUTOPILOT_VERSION_REQUEST(cur)
    fill(pautopilot_version_request)
    onAUTOPILOT_VERSION_REQUEST(pautopilot_version_request)
    run {

        var dst = CommunicationChannel.NEW.AUTOPILOT_VERSION_REQUEST(cur_dst)
        pautopilot_version_request.copyInto(dst)
        assert(Arrays.equals(pautopilot_version_request.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.AUTOPILOT_VERSION_REQUEST(cur_dst)
        dst.copyFrom(pautopilot_version_request)
        assert(Arrays.equals(pautopilot_version_request.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pautopilot_version_request)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOCAL_POSITION_NED -------------------------")

    fill(TestChannel.NEW.LOCAL_POSITION_NED(cur))
    val plocal_position_ned = com.company.demo.GroundControl.LOCAL_POSITION_NED(cur)
    onLOCAL_POSITION_NED(plocal_position_ned)
    run {
        cur_dst.init(com.company.demo.GroundControl.LOCAL_POSITION_NED.meta)
        val dst = test_.LOCAL_POSITION_NED(cur_dst)
        dst.copyFrom(plocal_position_ned)
        assert(Arrays.equals(plocal_position_ned.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(plocal_position_ned)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DATA_TRANSMISSION_HANDSHAKE -------------------------")

    val pdata_transmission_handshake = CommunicationChannel.NEW.DATA_TRANSMISSION_HANDSHAKE(cur)
    fill(pdata_transmission_handshake)
    onDATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake)
    run {

        var dst = CommunicationChannel.NEW.DATA_TRANSMISSION_HANDSHAKE(cur_dst)
        pdata_transmission_handshake.copyInto(dst)
        assert(Arrays.equals(pdata_transmission_handshake.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DATA_TRANSMISSION_HANDSHAKE(cur_dst)
        dst.copyFrom(pdata_transmission_handshake)
        assert(Arrays.equals(pdata_transmission_handshake.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pdata_transmission_handshake)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS_GLOBAL_ORIGIN -------------------------")

    fill(TestChannel.NEW.GPS_GLOBAL_ORIGIN(cur))
    val pgps_global_origin = com.company.demo.GroundControl.GPS_GLOBAL_ORIGIN(cur)
    onGPS_GLOBAL_ORIGIN(pgps_global_origin)
    run {
        cur_dst.init(com.company.demo.GroundControl.GPS_GLOBAL_ORIGIN.meta)
        val dst = test_.GPS_GLOBAL_ORIGIN(cur_dst)
        dst.copyFrom(pgps_global_origin)
        assert(Arrays.equals(pgps_global_origin.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pgps_global_origin)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SCALED_IMU2 -------------------------")

    val pscaled_imu2 = CommunicationChannel.NEW.SCALED_IMU2(cur)
    fill(pscaled_imu2)
    onSCALED_IMU2(pscaled_imu2)
    run {

        var dst = CommunicationChannel.NEW.SCALED_IMU2(cur_dst)
        pscaled_imu2.copyInto(dst)
        assert(Arrays.equals(pscaled_imu2.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SCALED_IMU2(cur_dst)
        dst.copyFrom(pscaled_imu2)
        assert(Arrays.equals(pscaled_imu2.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pscaled_imu2)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ATTITUDE_QUATERNION -------------------------")

    fill(TestChannel.NEW.ATTITUDE_QUATERNION(cur))
    val pattitude_quaternion = com.company.demo.GroundControl.ATTITUDE_QUATERNION(cur)
    onATTITUDE_QUATERNION(pattitude_quaternion)
    run {
        cur_dst.init(com.company.demo.GroundControl.ATTITUDE_QUATERNION.meta)
        val dst = test_.ATTITUDE_QUATERNION(cur_dst)
        dst.copyFrom(pattitude_quaternion)
        assert(Arrays.equals(pattitude_quaternion.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pattitude_quaternion)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DATA64 -------------------------")

    val pdata64 = CommunicationChannel.NEW.DATA64(cur)
    fill(pdata64)
    onDATA64(pdata64)
    run {

        var dst = CommunicationChannel.NEW.DATA64(cur_dst)
        pdata64.copyInto(dst)
        assert(Arrays.equals(pdata64.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DATA64(cur_dst)
        dst.copyFrom(pdata64)
        assert(Arrays.equals(pdata64.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdata64)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIL_ACTUATOR_CONTROLS -------------------------")

    fill(TestChannel.NEW.HIL_ACTUATOR_CONTROLS(cur))
    val phil_actuator_controls = com.company.demo.GroundControl.HIL_ACTUATOR_CONTROLS(cur)
    onHIL_ACTUATOR_CONTROLS(phil_actuator_controls)
    run {
        cur_dst.init(com.company.demo.GroundControl.HIL_ACTUATOR_CONTROLS.meta)
        val dst = test_.HIL_ACTUATOR_CONTROLS(cur_dst)
        dst.copyFrom(phil_actuator_controls)
        assert(Arrays.equals(phil_actuator_controls.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(phil_actuator_controls)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- POSITION_TARGET_LOCAL_NED -------------------------")

    fill(TestChannel.NEW.POSITION_TARGET_LOCAL_NED(cur))
    val pposition_target_local_ned = com.company.demo.GroundControl.POSITION_TARGET_LOCAL_NED(cur)
    onPOSITION_TARGET_LOCAL_NED(pposition_target_local_ned)
    run {
        cur_dst.init(com.company.demo.GroundControl.POSITION_TARGET_LOCAL_NED.meta)
        val dst = test_.POSITION_TARGET_LOCAL_NED(cur_dst)
        dst.copyFrom(pposition_target_local_ned)
        assert(Arrays.equals(pposition_target_local_ned.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pposition_target_local_ned)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GIMBAL_REPORT -------------------------")

    val pgimbal_report = CommunicationChannel.NEW.GIMBAL_REPORT(cur)
    fill(pgimbal_report)
    onGIMBAL_REPORT(pgimbal_report)
    run {

        var dst = CommunicationChannel.NEW.GIMBAL_REPORT(cur_dst)
        pgimbal_report.copyInto(dst)
        assert(Arrays.equals(pgimbal_report.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GIMBAL_REPORT(cur_dst)
        dst.copyFrom(pgimbal_report)
        assert(Arrays.equals(pgimbal_report.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pgimbal_report)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DEVICE_OP_WRITE -------------------------")

    val pdevice_op_write = CommunicationChannel.NEW.DEVICE_OP_WRITE(cur)
    fill(pdevice_op_write)
    onDEVICE_OP_WRITE(pdevice_op_write)
    run {

        var dst = CommunicationChannel.NEW.DEVICE_OP_WRITE(cur_dst)
        pdevice_op_write.copyInto(dst)
        assert(Arrays.equals(pdevice_op_write.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DEVICE_OP_WRITE(cur_dst)
        dst.copyFrom(pdevice_op_write)
        assert(Arrays.equals(pdevice_op_write.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdevice_op_write)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DISTANCE_SENSOR -------------------------")

    val pdistance_sensor = CommunicationChannel.NEW.DISTANCE_SENSOR(cur)
    fill(pdistance_sensor)
    onDISTANCE_SENSOR(pdistance_sensor)
    run {

        var dst = CommunicationChannel.NEW.DISTANCE_SENSOR(cur_dst)
        pdistance_sensor.copyInto(dst)
        assert(Arrays.equals(pdistance_sensor.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DISTANCE_SENSOR(cur_dst)
        dst.copyFrom(pdistance_sensor)
        assert(Arrays.equals(pdistance_sensor.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pdistance_sensor)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIL_OPTICAL_FLOW -------------------------")

    val phil_optical_flow = CommunicationChannel.NEW.HIL_OPTICAL_FLOW(cur)
    fill(phil_optical_flow)
    onHIL_OPTICAL_FLOW(phil_optical_flow)
    run {

        var dst = CommunicationChannel.NEW.HIL_OPTICAL_FLOW(cur_dst)
        phil_optical_flow.copyInto(dst)
        assert(Arrays.equals(phil_optical_flow.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.HIL_OPTICAL_FLOW(cur_dst)
        dst.copyFrom(phil_optical_flow)
        assert(Arrays.equals(phil_optical_flow.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(phil_optical_flow)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SCALED_PRESSURE2 -------------------------")

    val pscaled_pressure2 = CommunicationChannel.NEW.SCALED_PRESSURE2(cur)
    fill(pscaled_pressure2)
    onSCALED_PRESSURE2(pscaled_pressure2)
    run {

        var dst = CommunicationChannel.NEW.SCALED_PRESSURE2(cur_dst)
        pscaled_pressure2.copyInto(dst)
        assert(Arrays.equals(pscaled_pressure2.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.SCALED_PRESSURE2(cur_dst)
        dst.copyFrom(pscaled_pressure2)
        assert(Arrays.equals(pscaled_pressure2.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pscaled_pressure2)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- WIND_COV -------------------------")

    val pwind_cov = CommunicationChannel.NEW.WIND_COV(cur)
    fill(pwind_cov)
    onWIND_COV(pwind_cov)
    run {

        var dst = CommunicationChannel.NEW.WIND_COV(cur_dst)
        pwind_cov.copyInto(dst)
        assert(Arrays.equals(pwind_cov.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.WIND_COV(cur_dst)
        dst.copyFrom(pwind_cov)
        assert(Arrays.equals(pwind_cov.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pwind_cov)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- CHANGE_OPERATOR_CONTROL -------------------------")

    fill(TestChannel.NEW.CHANGE_OPERATOR_CONTROL(cur))
    val pchange_operator_control = com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL(cur)
    onCHANGE_OPERATOR_CONTROL(pchange_operator_control)
    run {
        cur_dst.init(com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL.meta)
        val dst = test_.CHANGE_OPERATOR_CONTROL(cur_dst)
        dst.copyFrom(pchange_operator_control)
        assert(Arrays.equals(pchange_operator_control.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pchange_operator_control)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GOPRO_SET_REQUEST -------------------------")

    val pgopro_set_request = CommunicationChannel.NEW.GOPRO_SET_REQUEST(cur)
    fill(pgopro_set_request)
    onGOPRO_SET_REQUEST(pgopro_set_request)
    run {

        var dst = CommunicationChannel.NEW.GOPRO_SET_REQUEST(cur_dst)
        pgopro_set_request.copyInto(dst)
        assert(Arrays.equals(pgopro_set_request.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GOPRO_SET_REQUEST(cur_dst)
        dst.copyFrom(pgopro_set_request)
        assert(Arrays.equals(pgopro_set_request.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pgopro_set_request)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- SYS_STATUS -------------------------")

    fill(TestChannel.NEW.SYS_STATUS(cur))
    val psys_status = com.company.demo.GroundControl.SYS_STATUS(cur)
    onSYS_STATUS(psys_status)
    run {
        cur_dst.init(com.company.demo.GroundControl.SYS_STATUS.meta)
        val dst = test_.SYS_STATUS(cur_dst)
        dst.copyFrom(psys_status)
        assert(Arrays.equals(psys_status.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(psys_status)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_ITEM -------------------------")

    fill(TestChannel.NEW.MISSION_ITEM(cur))
    val pmission_item = com.company.demo.GroundControl.MISSION_ITEM(cur)
    onMISSION_ITEM(pmission_item)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_ITEM.meta)
        val dst = test_.MISSION_ITEM(cur_dst)
        dst.copyFrom(pmission_item)
        assert(Arrays.equals(pmission_item.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_item)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RAW_IMU -------------------------")

    fill(TestChannel.NEW.RAW_IMU(cur))
    val praw_imu = com.company.demo.GroundControl.RAW_IMU(cur)
    onRAW_IMU(praw_imu)
    run {
        cur_dst.init(com.company.demo.GroundControl.RAW_IMU.meta)
        val dst = test_.RAW_IMU(cur_dst)
        dst.copyFrom(praw_imu)
        assert(Arrays.equals(praw_imu.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(praw_imu)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- COMMAND_INT -------------------------")

    fill(TestChannel.NEW.COMMAND_INT(cur))
    val pcommand_int = com.company.demo.GroundControl.COMMAND_INT(cur)
    onCOMMAND_INT(pcommand_int)
    run {
        cur_dst.init(com.company.demo.GroundControl.COMMAND_INT.meta)
        val dst = test_.COMMAND_INT(cur_dst)
        dst.copyFrom(pcommand_int)
        assert(Arrays.equals(pcommand_int.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pcommand_int)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- OPTICAL_FLOW -------------------------")

    fill(TestChannel.NEW.OPTICAL_FLOW(cur))
    val poptical_flow = com.company.demo.GroundControl.OPTICAL_FLOW(cur)
    onOPTICAL_FLOW(poptical_flow)
    run {
        cur_dst.init(com.company.demo.GroundControl.OPTICAL_FLOW.meta)
        val dst = test_.OPTICAL_FLOW(cur_dst)
        dst.copyFrom(poptical_flow)
        assert(Arrays.equals(poptical_flow.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(poptical_flow)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MISSION_ITEM_INT -------------------------")

    fill(TestChannel.NEW.MISSION_ITEM_INT(cur))
    val pmission_item_int = com.company.demo.GroundControl.MISSION_ITEM_INT(cur)
    onMISSION_ITEM_INT(pmission_item_int)
    run {
        cur_dst.init(com.company.demo.GroundControl.MISSION_ITEM_INT.meta)
        val dst = test_.MISSION_ITEM_INT(cur_dst)
        dst.copyFrom(pmission_item_int)
        assert(Arrays.equals(pmission_item_int.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pmission_item_int)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- VISION_POSITION_DELTA -------------------------")

    val pvision_position_delta = CommunicationChannel.NEW.VISION_POSITION_DELTA(cur)
    fill(pvision_position_delta)
    onVISION_POSITION_DELTA(pvision_position_delta)
    run {

        var dst = CommunicationChannel.NEW.VISION_POSITION_DELTA(cur_dst)
        pvision_position_delta.copyInto(dst)
        assert(Arrays.equals(pvision_position_delta.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.VISION_POSITION_DELTA(cur_dst)
        dst.copyFrom(pvision_position_delta)
        assert(Arrays.equals(pvision_position_delta.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pvision_position_delta)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- LOGGING_DATA -------------------------")

    val plogging_data = CommunicationChannel.NEW.LOGGING_DATA(cur)
    fill(plogging_data)
    onLOGGING_DATA(plogging_data)
    run {

        var dst = CommunicationChannel.NEW.LOGGING_DATA(cur_dst)
        plogging_data.copyInto(dst)
        assert(Arrays.equals(plogging_data.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.LOGGING_DATA(cur_dst)
        dst.copyFrom(plogging_data)
        assert(Arrays.equals(plogging_data.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(plogging_data)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- DEVICE_OP_READ -------------------------")

    val pdevice_op_read = CommunicationChannel.NEW.DEVICE_OP_READ(cur)
    fill(pdevice_op_read)
    onDEVICE_OP_READ(pdevice_op_read)
    run {

        var dst = CommunicationChannel.NEW.DEVICE_OP_READ(cur_dst)
        pdevice_op_read.copyInto(dst)
        assert(Arrays.equals(pdevice_op_read.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.DEVICE_OP_READ(cur_dst)
        dst.copyFrom(pdevice_op_read)
        assert(Arrays.equals(pdevice_op_read.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pdevice_op_read)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- MAG_CAL_PROGRESS -------------------------")

    val pmag_cal_progress = CommunicationChannel.NEW.MAG_CAL_PROGRESS(cur)
    fill(pmag_cal_progress)
    onMAG_CAL_PROGRESS(pmag_cal_progress)
    run {

        var dst = CommunicationChannel.NEW.MAG_CAL_PROGRESS(cur_dst)
        pmag_cal_progress.copyInto(dst)
        assert(Arrays.equals(pmag_cal_progress.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.MAG_CAL_PROGRESS(cur_dst)
        dst.copyFrom(pmag_cal_progress)
        assert(Arrays.equals(pmag_cal_progress.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pmag_cal_progress)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- HIGHRES_IMU -------------------------")

    val phighres_imu = CommunicationChannel.NEW.HIGHRES_IMU(cur)
    fill(phighres_imu)
    onHIGHRES_IMU(phighres_imu)
    run {

        var dst = CommunicationChannel.NEW.HIGHRES_IMU(cur_dst)
        phighres_imu.copyInto(dst)
        assert(Arrays.equals(phighres_imu.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.HIGHRES_IMU(cur_dst)
        dst.copyFrom(phighres_imu)
        assert(Arrays.equals(phighres_imu.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(phighres_imu)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- EXTENDED_SYS_STATE -------------------------")

    val pextended_sys_state = CommunicationChannel.NEW.EXTENDED_SYS_STATE(cur)
    fill(pextended_sys_state)
    onEXTENDED_SYS_STATE(pextended_sys_state)
    run {

        var dst = CommunicationChannel.NEW.EXTENDED_SYS_STATE(cur_dst)
        pextended_sys_state.copyInto(dst)
        assert(Arrays.equals(pextended_sys_state.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.EXTENDED_SYS_STATE(cur_dst)
        dst.copyFrom(pextended_sys_state)
        assert(Arrays.equals(pextended_sys_state.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pextended_sys_state)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- UAVIONIX_ADSB_OUT_DYNAMIC -------------------------")

    val puavionix_adsb_out_dynamic = CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_DYNAMIC(cur)
    fill(puavionix_adsb_out_dynamic)
    onUAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic)
    run {

        var dst = CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_DYNAMIC(cur_dst)
        puavionix_adsb_out_dynamic.copyInto(dst)
        assert(Arrays.equals(puavionix_adsb_out_dynamic.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_DYNAMIC(cur_dst)
        dst.copyFrom(puavionix_adsb_out_dynamic)
        assert(Arrays.equals(puavionix_adsb_out_dynamic.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(puavionix_adsb_out_dynamic)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GOPRO_GET_RESPONSE -------------------------")

    val pgopro_get_response = CommunicationChannel.NEW.GOPRO_GET_RESPONSE(cur)
    fill(pgopro_get_response)
    onGOPRO_GET_RESPONSE(pgopro_get_response)
    run {

        var dst = CommunicationChannel.NEW.GOPRO_GET_RESPONSE(cur_dst)
        pgopro_get_response.copyInto(dst)
        assert(Arrays.equals(pgopro_get_response.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GOPRO_GET_RESPONSE(cur_dst)
        dst.copyFrom(pgopro_get_response)
        assert(Arrays.equals(pgopro_get_response.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pgopro_get_response)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS_INJECT_DATA -------------------------")

    val pgps_inject_data = CommunicationChannel.NEW.GPS_INJECT_DATA(cur)
    fill(pgps_inject_data)
    onGPS_INJECT_DATA(pgps_inject_data)
    run {

        var dst = CommunicationChannel.NEW.GPS_INJECT_DATA(cur_dst)
        pgps_inject_data.copyInto(dst)
        assert(Arrays.equals(pgps_inject_data.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GPS_INJECT_DATA(cur_dst)
        dst.copyFrom(pgps_inject_data)
        assert(Arrays.equals(pgps_inject_data.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pgps_inject_data)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT -------------------------")

    val puavionix_adsb_transceiver_health_report = CommunicationChannel.NEW.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur)
    fill(puavionix_adsb_transceiver_health_report)
    onUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report)
    run {

        var dst = CommunicationChannel.NEW.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur_dst)
        puavionix_adsb_transceiver_health_report.copyInto(dst)
        assert(Arrays.equals(puavionix_adsb_transceiver_health_report.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur_dst)
        dst.copyFrom(puavionix_adsb_transceiver_health_report)
        assert(Arrays.equals(puavionix_adsb_transceiver_health_report.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(puavionix_adsb_transceiver_health_report)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- ATTITUDE_QUATERNION_COV -------------------------")

    fill(TestChannel.NEW.ATTITUDE_QUATERNION_COV(cur))
    val pattitude_quaternion_cov = com.company.demo.GroundControl.ATTITUDE_QUATERNION_COV(cur)
    onATTITUDE_QUATERNION_COV(pattitude_quaternion_cov)
    run {
        cur_dst.init(com.company.demo.GroundControl.ATTITUDE_QUATERNION_COV.meta)
        val dst = test_.ATTITUDE_QUATERNION_COV(cur_dst)
        dst.copyFrom(pattitude_quaternion_cov)
        assert(Arrays.equals(pattitude_quaternion_cov.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pattitude_quaternion_cov)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- NAMED_VALUE_INT -------------------------")

    val pnamed_value_int = CommunicationChannel.NEW.NAMED_VALUE_INT(cur)
    fill(pnamed_value_int)
    onNAMED_VALUE_INT(pnamed_value_int)
    run {

        var dst = CommunicationChannel.NEW.NAMED_VALUE_INT(cur_dst)
        pnamed_value_int.copyInto(dst)
        assert(Arrays.equals(pnamed_value_int.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.NAMED_VALUE_INT(cur_dst)
        dst.copyFrom(pnamed_value_int)
        assert(Arrays.equals(pnamed_value_int.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pnamed_value_int)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RPM -------------------------")

    val prpm = CommunicationChannel.NEW.RPM(cur)
    fill(prpm)
    onRPM(prpm)
    run {

        var dst = CommunicationChannel.NEW.RPM(cur_dst)
        prpm.copyInto(dst)
        assert(Arrays.equals(prpm.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.RPM(cur_dst)
        dst.copyFrom(prpm)
        assert(Arrays.equals(prpm.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(prpm)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GPS_RTCM_DATA -------------------------")

    val pgps_rtcm_data = CommunicationChannel.NEW.GPS_RTCM_DATA(cur)
    fill(pgps_rtcm_data)
    onGPS_RTCM_DATA(pgps_rtcm_data)
    run {

        var dst = CommunicationChannel.NEW.GPS_RTCM_DATA(cur_dst)
        pgps_rtcm_data.copyInto(dst)
        assert(Arrays.equals(pgps_rtcm_data.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.GPS_RTCM_DATA(cur_dst)
        dst.copyFrom(pgps_rtcm_data)
        assert(Arrays.equals(pgps_rtcm_data.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pgps_rtcm_data)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- GLOBAL_VISION_POSITION_ESTIMATE -------------------------")

    fill(TestChannel.NEW.GLOBAL_VISION_POSITION_ESTIMATE(cur))
    val pglobal_vision_position_estimate = com.company.demo.GroundControl.GLOBAL_VISION_POSITION_ESTIMATE(cur)
    onGLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate)
    run {
        cur_dst.init(com.company.demo.GroundControl.GLOBAL_VISION_POSITION_ESTIMATE.meta)
        val dst = test_.GLOBAL_VISION_POSITION_ESTIMATE(cur_dst)
        dst.copyFrom(pglobal_vision_position_estimate)
        assert(Arrays.equals(pglobal_vision_position_estimate.data.bytes, dst.data.bytes))
    }

    if (TestChannel.send(pglobal_vision_position_estimate)) transmission(TestChannel.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- FILE_TRANSFER_PROTOCOL -------------------------")

    val pfile_transfer_protocol = CommunicationChannel.NEW.FILE_TRANSFER_PROTOCOL(cur)
    fill(pfile_transfer_protocol)
    onFILE_TRANSFER_PROTOCOL(pfile_transfer_protocol)
    run {

        var dst = CommunicationChannel.NEW.FILE_TRANSFER_PROTOCOL(cur_dst)
        pfile_transfer_protocol.copyInto(dst)
        assert(Arrays.equals(pfile_transfer_protocol.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.FILE_TRANSFER_PROTOCOL(cur_dst)
        dst.copyFrom(pfile_transfer_protocol)
        assert(Arrays.equals(pfile_transfer_protocol.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pfile_transfer_protocol)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RANGEFINDER -------------------------")

    val prangefinder = CommunicationChannel.NEW.RANGEFINDER(cur)
    fill(prangefinder)
    onRANGEFINDER(prangefinder)
    run {

        var dst = CommunicationChannel.NEW.RANGEFINDER(cur_dst)
        prangefinder.copyInto(dst)
        assert(Arrays.equals(prangefinder.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.RANGEFINDER(cur_dst)
        dst.copyFrom(prangefinder)
        assert(Arrays.equals(prangefinder.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(prangefinder)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RADIO_STATUS -------------------------")

    val pradio_status = CommunicationChannel.NEW.RADIO_STATUS(cur)
    fill(pradio_status)
    onRADIO_STATUS(pradio_status)
    run {

        var dst = CommunicationChannel.NEW.RADIO_STATUS(cur_dst)
        pradio_status.copyInto(dst)
        assert(Arrays.equals(pradio_status.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.RADIO_STATUS(cur_dst)
        dst.copyFrom(pradio_status)
        assert(Arrays.equals(pradio_status.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(pradio_status)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- FENCE_POINT -------------------------")

    val pfence_point = CommunicationChannel.NEW.FENCE_POINT(cur)
    fill(pfence_point)
    onFENCE_POINT(pfence_point)
    run {

        var dst = CommunicationChannel.NEW.FENCE_POINT(cur_dst)
        pfence_point.copyInto(dst)
        assert(Arrays.equals(pfence_point.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.FENCE_POINT(cur_dst)
        dst.copyFrom(pfence_point)
        assert(Arrays.equals(pfence_point.data.bytes, dst.data.bytes))
    }


    if (CommunicationChannel_instance.send(pfence_point)) transmission(CommunicationChannel_instance.transmitter, TestChannel.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    println("-------------------- RESOURCE_REQUEST -------------------------")

    val presource_request = CommunicationChannel.NEW.RESOURCE_REQUEST(cur)
    fill(presource_request)
    onRESOURCE_REQUEST(presource_request)
    run {

        var dst = CommunicationChannel.NEW.RESOURCE_REQUEST(cur_dst)
        presource_request.copyInto(dst)
        assert(Arrays.equals(presource_request.data.bytes, dst.data.bytes))

        dst = CommunicationChannel.NEW.RESOURCE_REQUEST(cur_dst)
        dst.copyFrom(presource_request)
        assert(Arrays.equals(presource_request.data.bytes, dst.data.bytes))
    }

    if (CommunicationChannel_instance.send(presource_request)) transmission(CommunicationChannel_instance.transmitter, CommunicationChannel_instance.receiver)
    else throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")


}
					   