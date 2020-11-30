
#ifdef __cplusplus
																														extern "C"
									{
#endif


#pragma once

#include "Utils_.h"


static inline int32_t pattitude_target_time_boot_ms_GET(pattitude_target_ATTITUDE_TARGET *src) {//Timestamp in milliseconds since system boot
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}

/**
*Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
*					 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud */

static inline int8_t pattitude_target_type_mask_GET(pattitude_target_ATTITUDE_TARGET *src) {
	
	return (int8_t) src[4];
}


static inline float vattitude_target_q_GET(Vattitude_target_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vattitude_target_q pattitude_target_q_GET(pattitude_target_ATTITUDE_TARGET *src) { //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	
	return (Vattitude_target_q) {src + 5, 4};
}


static inline float pattitude_target_body_roll_rate_GET(pattitude_target_ATTITUDE_TARGET *src) {//Body roll rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 21, 4)));
}


static inline float pattitude_target_body_pitch_rate_GET(pattitude_target_ATTITUDE_TARGET *src) {//Body pitch rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 25, 4)));
}


static inline float pattitude_target_body_yaw_rate_GET(pattitude_target_ATTITUDE_TARGET *src) {//Body yaw rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 29, 4)));
}


static inline float pattitude_target_thrust_GET(pattitude_target_ATTITUDE_TARGET *src) {//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	
	return (intBitsToFloat(get_bytes(src, 33, 4)));
}


static inline int16_t pmission_count_count_GET(pmission_count_MISSION_COUNT *src) {//Number of mission items in the sequence
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int8_t pmission_count_target_system_GET(pmission_count_MISSION_COUNT *src) {//System ID
	
	return (int8_t) src->base.bytes[2];
}


static inline int8_t pmission_count_target_component_GET(pmission_count_MISSION_COUNT *src) {//Component ID
	
	return (int8_t) src->base.bytes[3];
}

static inline bool pmission_count_mission_type_GET(pmission_count_MISSION_COUNT *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 32 && !set_field(src, 32, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline int16_t padsb_vehicle_heading_GET(padsb_vehicle_ADSB_VEHICLE *src) {//Course over ground in centidegrees
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void padsb_vehicle_heading_SET(int16_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //Course over ground in centidegrees
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t padsb_vehicle_hor_velocity_GET(padsb_vehicle_ADSB_VEHICLE *src) {//The horizontal velocity in centimeters/second
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void padsb_vehicle_hor_velocity_SET(int16_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //The horizontal velocity in centimeters/second
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int16_t padsb_vehicle_squawk_GET(padsb_vehicle_ADSB_VEHICLE *src) {//Squawk code
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}


static inline void padsb_vehicle_squawk_SET(int16_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //Squawk code
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 4);
}


static inline int32_t padsb_vehicle_ICAO_address_GET(padsb_vehicle_ADSB_VEHICLE *src) {//ICAO address
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline void padsb_vehicle_ICAO_address_SET(int32_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //ICAO address
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 6);
}


static inline int32_t padsb_vehicle_lat_GET(padsb_vehicle_ADSB_VEHICLE *src) {//Latitude, expressed as degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 10, 4)));
}


static inline void padsb_vehicle_lat_SET(int32_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //Latitude, expressed as degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 10);
}


static inline int32_t padsb_vehicle_lon_GET(padsb_vehicle_ADSB_VEHICLE *src) {//Longitude, expressed as degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 14, 4)));
}


static inline void padsb_vehicle_lon_SET(int32_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //Longitude, expressed as degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 14);
}


static inline int32_t padsb_vehicle_altitude_GET(padsb_vehicle_ADSB_VEHICLE *src) {//Altitude(ASL) in millimeters
	
	return ((int32_t) (get_bytes(src->base.bytes, 18, 4)));
}


static inline void padsb_vehicle_altitude_SET(int32_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //Altitude(ASL) in millimeters
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 18);
}


static inline int16_t padsb_vehicle_ver_velocity_GET(padsb_vehicle_ADSB_VEHICLE *src) {//The vertical velocity in centimeters/second, positive is up
	
	return ((int16_t) (get_bytes(src->base.bytes, 22, 2)));
}


static inline void padsb_vehicle_ver_velocity_SET(int16_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //The vertical velocity in centimeters/second, positive is up
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 22);
}


static inline int8_t padsb_vehicle_tslc_GET(padsb_vehicle_ADSB_VEHICLE *src) {//Time since last communication in seconds
	
	return (int8_t) src->base.bytes[24];
}


static inline void padsb_vehicle_tslc_SET(int8_t src, padsb_vehicle_ADSB_VEHICLE *dst) { //Time since last communication in seconds
	
	
	dst->base.bytes[24] = (uint8_t) (src);
}

static inline bool padsb_vehicle_altitude_type_GET(padsb_vehicle_ADSB_VEHICLE *const src, e_ADSB_ALTITUDE_TYPE *ret) {
	if (src->base.field_bit != 202 && !set_field(src, 202, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void padsb_vehicle_altitude_type_SET(e_ADSB_ALTITUDE_TYPE src, padsb_vehicle_ADSB_VEHICLE *const dst) {
	
	if (dst->base.field_bit != 202) set_field(dst, 202, 0);
	
	
	set_bits(src, 1, dst->base.bytes, dst->BIT);
}

static inline bool padsb_vehicle_callsign_GET(padsb_vehicle_ADSB_VEHICLE *const src, Vadsb_vehicle_callsign *ret) {//The callsign, 8+null
	
	if (src->base.field_bit != 203 && !set_field(src, 203, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vadsb_vehicle_callsign padsb_vehicle_callsign_SET(const char src[], size_t len, padsb_vehicle_ADSB_VEHICLE *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 203, len);
	
	Vadsb_vehicle_callsign ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool padsb_vehicle_emitter_type_GET(padsb_vehicle_ADSB_VEHICLE *const src, e_ADSB_EMITTER_TYPE *ret) {
	if (src->base.field_bit != 204 && !set_field(src, 204, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline void padsb_vehicle_emitter_type_SET(e_ADSB_EMITTER_TYPE src, padsb_vehicle_ADSB_VEHICLE *const dst) {
	
	if (dst->base.field_bit != 204) set_field(dst, 204, 0);
	
	
	set_bits(src, 5, dst->base.bytes, dst->BIT);
}

static inline bool padsb_vehicle_flags_GET(padsb_vehicle_ADSB_VEHICLE *const src, e_ADSB_FLAGS *ret) {
	if (src->base.field_bit != 205 && !set_field(src, 205, -1)) return false;
	*ret = _en_adsb_flags(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void padsb_vehicle_flags_SET(e_ADSB_FLAGS src, padsb_vehicle_ADSB_VEHICLE *const dst) {
	
	if (dst->base.field_bit != 205) set_field(dst, 205, 0);
	
	
	UMAX id = _id_adsb_flags(src);
	set_bits(id, 3, dst->base.bytes, dst->BIT);
}


static inline int16_t pmessage_interval_message_id_GET(pmessage_interval_MESSAGE_INTERVAL *src) {//The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pmessage_interval_message_id_SET(int16_t src, pmessage_interval_MESSAGE_INTERVAL *dst) { //The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int32_t pmessage_interval_interval_us_GET(pmessage_interval_MESSAGE_INTERVAL *src) {//0 indicates the interval at which it is sent.
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline void pmessage_interval_interval_us_SET(int32_t src, pmessage_interval_MESSAGE_INTERVAL *dst) { //0 indicates the interval at which it is sent.
	
	
	set_bytes((uint32_t) (src), 4, dst, 2);
}


static inline float pekf_status_report_velocity_variance_GET(pekf_status_report_EKF_STATUS_REPORT *src) {//Velocity variance
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 0, 4)));
}


static inline void pekf_status_report_velocity_variance_SET(float src, pekf_status_report_EKF_STATUS_REPORT *dst) { //Velocity variance
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 0);
}


static inline float pekf_status_report_pos_horiz_variance_GET(pekf_status_report_EKF_STATUS_REPORT *src) {//Horizontal Position variance
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 4, 4)));
}


static inline void pekf_status_report_pos_horiz_variance_SET(float src, pekf_status_report_EKF_STATUS_REPORT *dst) { //Horizontal Position variance
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 4);
}


static inline float pekf_status_report_pos_vert_variance_GET(pekf_status_report_EKF_STATUS_REPORT *src) {//Vertical Position variance
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline void pekf_status_report_pos_vert_variance_SET(float src, pekf_status_report_EKF_STATUS_REPORT *dst) { //Vertical Position variance
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 8);
}


static inline float pekf_status_report_compass_variance_GET(pekf_status_report_EKF_STATUS_REPORT *src) {//Compass variance
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void pekf_status_report_compass_variance_SET(float src, pekf_status_report_EKF_STATUS_REPORT *dst) { //Compass variance
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}


static inline float pekf_status_report_terrain_alt_variance_GET(pekf_status_report_EKF_STATUS_REPORT *src) {//Terrain Altitude variance
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline void pekf_status_report_terrain_alt_variance_SET(float src, pekf_status_report_EKF_STATUS_REPORT *dst) { //Terrain Altitude variance
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 16);
}

static inline bool pekf_status_report_flags_GET(pekf_status_report_EKF_STATUS_REPORT *const src, e_EKF_STATUS_FLAGS *ret) {
	if (src->base.field_bit != 160 && !set_field(src, 160, -1)) return false;
	*ret = _en_ekf_status_flags(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pekf_status_report_flags_SET(e_EKF_STATUS_FLAGS src, pekf_status_report_EKF_STATUS_REPORT *const dst) {
	
	if (dst->base.field_bit != 160) set_field(dst, 160, 0);
	
	
	UMAX id = _id_ekf_status_flags(src);
	set_bits(id, 4, dst->base.bytes, dst->BIT);
}


static inline int64_t pestimator_status_time_usec_GET(pestimator_status_ESTIMATOR_STATUS *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src->base.bytes, 0, 8)));
}


static inline void pestimator_status_time_usec_SET(int64_t src, pestimator_status_ESTIMATOR_STATUS *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst->base.bytes, 0);
}


static inline float pestimator_status_vel_ratio_GET(pestimator_status_ESTIMATOR_STATUS *src) {//Velocity innovation test ratio
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline void pestimator_status_vel_ratio_SET(float src, pestimator_status_ESTIMATOR_STATUS *dst) { //Velocity innovation test ratio
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 8);
}


static inline float pestimator_status_pos_horiz_ratio_GET(pestimator_status_ESTIMATOR_STATUS *src) {//Horizontal position innovation test ratio
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void pestimator_status_pos_horiz_ratio_SET(float src, pestimator_status_ESTIMATOR_STATUS *dst) { //Horizontal position innovation test ratio
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}


static inline float pestimator_status_pos_vert_ratio_GET(pestimator_status_ESTIMATOR_STATUS *src) {//Vertical position innovation test ratio
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline void pestimator_status_pos_vert_ratio_SET(float src, pestimator_status_ESTIMATOR_STATUS *dst) { //Vertical position innovation test ratio
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 16);
}


static inline float pestimator_status_mag_ratio_GET(pestimator_status_ESTIMATOR_STATUS *src) {//Magnetometer innovation test ratio
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline void pestimator_status_mag_ratio_SET(float src, pestimator_status_ESTIMATOR_STATUS *dst) { //Magnetometer innovation test ratio
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 20);
}


static inline float pestimator_status_hagl_ratio_GET(pestimator_status_ESTIMATOR_STATUS *src) {//Height above terrain innovation test ratio
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline void pestimator_status_hagl_ratio_SET(float src, pestimator_status_ESTIMATOR_STATUS *dst) { //Height above terrain innovation test ratio
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 24);
}


static inline float pestimator_status_tas_ratio_GET(pestimator_status_ESTIMATOR_STATUS *src) {//True airspeed innovation test ratio
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline void pestimator_status_tas_ratio_SET(float src, pestimator_status_ESTIMATOR_STATUS *dst) { //True airspeed innovation test ratio
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 28);
}


static inline float pestimator_status_pos_horiz_accuracy_GET(pestimator_status_ESTIMATOR_STATUS *src) {//Horizontal position 1-STD accuracy relative to the EKF local origin (m)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline void pestimator_status_pos_horiz_accuracy_SET(float src, pestimator_status_ESTIMATOR_STATUS *dst) { //Horizontal position 1-STD accuracy relative to the EKF local origin (m)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 32);
}


static inline float pestimator_status_pos_vert_accuracy_GET(pestimator_status_ESTIMATOR_STATUS *src) {//Vertical position 1-STD accuracy relative to the EKF local origin (m)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 36, 4)));
}


static inline void pestimator_status_pos_vert_accuracy_SET(float src, pestimator_status_ESTIMATOR_STATUS *dst) { //Vertical position 1-STD accuracy relative to the EKF local origin (m)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 36);
}

static inline bool pestimator_status_flags_GET(pestimator_status_ESTIMATOR_STATUS *const src, e_ESTIMATOR_STATUS_FLAGS *ret) {
	if (src->base.field_bit != 320 && !set_field(src, 320, -1)) return false;
	*ret = _en_estimator_status_flags(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pestimator_status_flags_SET(e_ESTIMATOR_STATUS_FLAGS src, pestimator_status_ESTIMATOR_STATUS *const dst) {
	
	if (dst->base.field_bit != 320) set_field(dst, 320, 0);
	
	
	UMAX id = _id_estimator_status_flags(src);
	set_bits(id, 4, dst->base.bytes, dst->BIT);
}


static inline int16_t phwstatus_Vcc_GET(phwstatus_HWSTATUS *src) {//board voltage (mV)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void phwstatus_Vcc_SET(int16_t src, phwstatus_HWSTATUS *dst) { //board voltage (mV)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int8_t phwstatus_I2Cerr_GET(phwstatus_HWSTATUS *src) {//I2C error count
	
	return (int8_t) src[2];
}


static inline void phwstatus_I2Cerr_SET(int8_t src, phwstatus_HWSTATUS *dst) { //I2C error count
	
	
	dst[2] = (uint8_t) (src);
}


static inline int64_t ptimesync_tc1_GET(ptimesync_TIMESYNC *src) {//Time sync timestamp 1
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void ptimesync_tc1_SET(int64_t src, ptimesync_TIMESYNC *dst) { //Time sync timestamp 1
	
	
	set_bytes((src), 8, dst, 0);
}


static inline int64_t ptimesync_ts1_GET(ptimesync_TIMESYNC *src) {//Time sync timestamp 2
	
	return ((int64_t) (get_bytes(src, 8, 8)));
}


static inline void ptimesync_ts1_SET(int64_t src, ptimesync_TIMESYNC *dst) { //Time sync timestamp 2
	
	
	set_bytes((src), 8, dst, 8);
}


static inline int8_t pparam_ext_request_list_target_system_GET(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pparam_ext_request_list_target_system_SET(int8_t src, pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pparam_ext_request_list_target_component_GET(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pparam_ext_request_list_target_component_SET(int8_t src, pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int64_t pglobal_position_int_cov_time_usec_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) {//Timestamp (microseconds since system boot or since UNIX epoch)
	
	return ((int64_t) (get_bytes(src->base.bytes, 0, 8)));
}


static inline int32_t pglobal_position_int_cov_lat_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) {//Latitude, expressed as degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 8, 4)));
}


static inline int32_t pglobal_position_int_cov_lon_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) {//Longitude, expressed as degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 12, 4)));
}


static inline int32_t pglobal_position_int_cov_alt_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) {//Altitude in meters, expressed as * 1000 (millimeters), above MSL
	
	return ((int32_t) (get_bytes(src->base.bytes, 16, 4)));
}


static inline int32_t pglobal_position_int_cov_relative_alt_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) {//Altitude above ground in meters, expressed as * 1000 (millimeters)
	
	return ((int32_t) (get_bytes(src->base.bytes, 20, 4)));
}


static inline float pglobal_position_int_cov_vx_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) {//Ground X Speed (Latitude), expressed as m/s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline float pglobal_position_int_cov_vy_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) {//Ground Y Speed (Longitude), expressed as m/s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline float pglobal_position_int_cov_vz_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) {//Ground Z Speed (Altitude), expressed as m/s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline float vglobal_position_int_cov_covariance_GET(Vglobal_position_int_cov_covariance const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vglobal_position_int_cov_covariance pglobal_position_int_cov_covariance_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *src) { //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
	
	return (Vglobal_position_int_cov_covariance) {src->base.bytes + 36, 36};
}

static inline bool pglobal_position_int_cov_estimator_type_GET(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *const src, e_MAV_ESTIMATOR_TYPE *ret) {
	if (src->base.field_bit != 1440 && !set_field(src, 1440, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline int32_t pbutton_change_time_boot_ms_GET(pbutton_change_BUTTON_CHANGE *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pbutton_change_time_boot_ms_SET(int32_t src, pbutton_change_BUTTON_CHANGE *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int32_t pbutton_change_last_change_ms_GET(pbutton_change_BUTTON_CHANGE *src) {//Time of last change of button state
	
	return ((int32_t) (get_bytes(src, 4, 4)));
}


static inline void pbutton_change_last_change_ms_SET(int32_t src, pbutton_change_BUTTON_CHANGE *dst) { //Time of last change of button state
	
	
	set_bytes((uint32_t) (src), 4, dst, 4);
}


static inline int8_t pbutton_change_state_GET(pbutton_change_BUTTON_CHANGE *src) {//Bitmap state of buttons
	
	return (int8_t) src[8];
}


static inline void pbutton_change_state_SET(int8_t src, pbutton_change_BUTTON_CHANGE *dst) { //Bitmap state of buttons
	
	
	dst[8] = (uint8_t) (src);
}


static inline int8_t psafety_set_allowed_area_target_system_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t psafety_set_allowed_area_target_component_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline float psafety_set_allowed_area_p1x_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *src) {//x position 1 / Latitude 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 2, 4)));
}


static inline float psafety_set_allowed_area_p1y_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *src) {//y position 1 / Longitude 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 6, 4)));
}


static inline float psafety_set_allowed_area_p1z_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *src) {//z position 1 / Altitude 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 10, 4)));
}


static inline float psafety_set_allowed_area_p2x_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *src) {//x position 2 / Latitude 2
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 14, 4)));
}


static inline float psafety_set_allowed_area_p2y_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *src) {//y position 2 / Longitude 2
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 18, 4)));
}


static inline float psafety_set_allowed_area_p2z_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *src) {//z position 2 / Altitude 2
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 22, 4)));
}

static inline bool psafety_set_allowed_area_frame_GET(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 208 && !set_field(src, 208, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}


static inline int16_t puavcan_node_status_vendor_specific_status_code_GET(puavcan_node_status_UAVCAN_NODE_STATUS *src) {//Vendor-specific status information.
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void puavcan_node_status_vendor_specific_status_code_SET(int16_t src, puavcan_node_status_UAVCAN_NODE_STATUS *dst) { //Vendor-specific status information.
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int32_t puavcan_node_status_uptime_sec_GET(puavcan_node_status_UAVCAN_NODE_STATUS *src) {//The number of seconds since the start-up of the node.
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline void puavcan_node_status_uptime_sec_SET(int32_t src, puavcan_node_status_UAVCAN_NODE_STATUS *dst) { //The number of seconds since the start-up of the node.
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 2);
}


static inline int64_t puavcan_node_status_time_usec_GET(puavcan_node_status_UAVCAN_NODE_STATUS *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src->base.bytes, 6, 8)));
}


static inline void puavcan_node_status_time_usec_SET(int64_t src, puavcan_node_status_UAVCAN_NODE_STATUS *dst) { //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	
	set_bytes((src), 8, dst->base.bytes, 6);
}


static inline int8_t puavcan_node_status_sub_mode_GET(puavcan_node_status_UAVCAN_NODE_STATUS *src) {//Not used currently.
	
	return (int8_t) src->base.bytes[14];
}


static inline void puavcan_node_status_sub_mode_SET(int8_t src, puavcan_node_status_UAVCAN_NODE_STATUS *dst) { //Not used currently.
	
	
	dst->base.bytes[14] = (uint8_t) (src);
}

static inline bool puavcan_node_status_health_GET(puavcan_node_status_UAVCAN_NODE_STATUS *const src, e_UAVCAN_NODE_HEALTH *ret) {
	if (src->base.field_bit != 120 && !set_field(src, 120, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void puavcan_node_status_health_SET(e_UAVCAN_NODE_HEALTH src, puavcan_node_status_UAVCAN_NODE_STATUS *const dst) {
	
	if (dst->base.field_bit != 120) set_field(dst, 120, 0);
	
	
	set_bits(src, 2, dst->base.bytes, dst->BIT);
}

static inline bool puavcan_node_status_mode_GET(puavcan_node_status_UAVCAN_NODE_STATUS *const src, e_UAVCAN_NODE_MODE *ret) {
	if (src->base.field_bit != 121 && !set_field(src, 121, -1)) return false;
	*ret = _en_uavcan_node_mode(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void puavcan_node_status_mode_SET(e_UAVCAN_NODE_MODE src, puavcan_node_status_UAVCAN_NODE_STATUS *const dst) {
	
	if (dst->base.field_bit != 121) set_field(dst, 121, 0);
	
	
	UMAX id = _id_uavcan_node_mode(src);
	set_bits(id, 3, dst->base.bytes, dst->BIT);
}


static inline int32_t pcollision_id_GET(pcollision_COLLISION *src) {//Unique identifier, domain based on src field
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void pcollision_id_SET(int32_t src, pcollision_COLLISION *dst) { //Unique identifier, domain based on src field
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline float pcollision_time_to_minimum_delta_GET(pcollision_COLLISION *src) {//Estimated time until collision occurs (seconds)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 4, 4)));
}


static inline void pcollision_time_to_minimum_delta_SET(float src, pcollision_COLLISION *dst) { //Estimated time until collision occurs (seconds)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 4);
}


static inline float pcollision_altitude_minimum_delta_GET(pcollision_COLLISION *src) {//Closest vertical distance in meters between vehicle and object
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline void pcollision_altitude_minimum_delta_SET(float src, pcollision_COLLISION *dst) { //Closest vertical distance in meters between vehicle and object
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 8);
}


static inline float pcollision_horizontal_minimum_delta_GET(pcollision_COLLISION *src) {//Closest horizontal distance in meteres between vehicle and object
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void pcollision_horizontal_minimum_delta_SET(float src, pcollision_COLLISION *dst) { //Closest horizontal distance in meteres between vehicle and object
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}

static inline bool pcollision_sRc_GET(pcollision_COLLISION *const src, e_MAV_COLLISION_SRC *ret) {
	if (src->base.field_bit != 130 && !set_field(src, 130, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void pcollision_sRc_SET(e_MAV_COLLISION_SRC src, pcollision_COLLISION *const dst) {
	
	if (dst->base.field_bit != 130) set_field(dst, 130, 0);
	
	
	set_bits(src, 1, dst->base.bytes, dst->BIT);
}

static inline bool pcollision_action_GET(pcollision_COLLISION *const src, e_MAV_COLLISION_ACTION *ret) {
	if (src->base.field_bit != 131 && !set_field(src, 131, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pcollision_action_SET(e_MAV_COLLISION_ACTION src, pcollision_COLLISION *const dst) {
	
	if (dst->base.field_bit != 131) set_field(dst, 131, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool pcollision_threat_level_GET(pcollision_COLLISION *const src, e_MAV_COLLISION_THREAT_LEVEL *ret) {
	if (src->base.field_bit != 132 && !set_field(src, 132, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void pcollision_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL src, pcollision_COLLISION *const dst) {
	
	if (dst->base.field_bit != 132) set_field(dst, 132, 0);
	
	
	set_bits(src, 2, dst->base.bytes, dst->BIT);
}


static inline int8_t pgimbal_torque_cmd_report_target_system_GET(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pgimbal_torque_cmd_report_target_system_SET(int8_t src, pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pgimbal_torque_cmd_report_target_component_GET(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pgimbal_torque_cmd_report_target_component_SET(int8_t src, pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int16_t pgimbal_torque_cmd_report_rl_torque_cmd_GET(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *src) {//Roll Torque Command
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pgimbal_torque_cmd_report_rl_torque_cmd_SET(int16_t src, pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *dst) { //Roll Torque Command
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t pgimbal_torque_cmd_report_el_torque_cmd_GET(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *src) {//Elevation Torque Command
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void pgimbal_torque_cmd_report_el_torque_cmd_SET(int16_t src, pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *dst) { //Elevation Torque Command
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int16_t pgimbal_torque_cmd_report_az_torque_cmd_GET(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *src) {//Azimuth Torque Command
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline void pgimbal_torque_cmd_report_az_torque_cmd_SET(int16_t src, pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *dst) { //Azimuth Torque Command
	
	
	set_bytes((uint16_t) (src), 2, dst, 6);
}


static inline int64_t paltitude_time_usec_GET(paltitude_ALTITUDE *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void paltitude_time_usec_SET(int64_t src, paltitude_ALTITUDE *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst, 0);
}

/**
*This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
*					 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
*					 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
*					 time. This altitude will also drift and vary between flights */

static inline float paltitude_altitude_monotonic_GET(paltitude_ALTITUDE *src) {
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}

/**
*This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
*					 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
*					 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
*					 time. This altitude will also drift and vary between flights */

static inline void paltitude_altitude_monotonic_SET(float src, paltitude_ALTITUDE *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}

/**
*This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
*					 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
*					 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
*					 by default and not the WGS84 altitude */

static inline float paltitude_altitude_amsl_GET(paltitude_ALTITUDE *src) {
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}

/**
*This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
*					 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
*					 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
*					 by default and not the WGS84 altitude */

static inline void paltitude_altitude_amsl_SET(float src, paltitude_ALTITUDE *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}

/**
*This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
*					 to the coordinate origin (0, 0, 0). It is up-positive */

static inline float paltitude_altitude_local_GET(paltitude_ALTITUDE *src) {
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}

/**
*This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
*					 to the coordinate origin (0, 0, 0). It is up-positive */

static inline void paltitude_altitude_local_SET(float src, paltitude_ALTITUDE *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float paltitude_altitude_relative_GET(paltitude_ALTITUDE *src) {//This is the altitude above the home position. It resets on each change of the current home position
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void paltitude_altitude_relative_SET(float src, paltitude_ALTITUDE *dst) { //This is the altitude above the home position. It resets on each change of the current home position
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}

/**
*This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
*					 than -1000 should be interpreted as unknown */

static inline float paltitude_altitude_terrain_GET(paltitude_ALTITUDE *src) {
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}

/**
*This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
*					 than -1000 should be interpreted as unknown */

static inline void paltitude_altitude_terrain_SET(float src, paltitude_ALTITUDE *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}

/**
*This is not the altitude, but the clear space below the system according to the fused clearance estimate.
*					 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
*					 target. A negative value indicates no measurement available */

static inline float paltitude_bottom_clearance_GET(paltitude_ALTITUDE *src) {
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}

/**
*This is not the altitude, but the clear space below the system according to the fused clearance estimate.
*					 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
*					 target. A negative value indicates no measurement available */

static inline void paltitude_bottom_clearance_SET(float src, paltitude_ALTITUDE *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline int16_t phil_state_quaternion_ind_airspeed_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Indicated airspeed, expressed as cm/s
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void phil_state_quaternion_ind_airspeed_SET(int16_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Indicated airspeed, expressed as cm/s
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t phil_state_quaternion_true_airspeed_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//True airspeed, expressed as cm/s
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void phil_state_quaternion_true_airspeed_SET(int16_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //True airspeed, expressed as cm/s
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int64_t phil_state_quaternion_time_usec_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src, 4, 8)));
}


static inline void phil_state_quaternion_time_usec_SET(int64_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	
	set_bytes((src), 8, dst, 4);
}


static inline float vhil_state_quaternion_attitude_quaternion_GET(Vhil_state_quaternion_attitude_quaternion const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vhil_state_quaternion_attitude_quaternion phil_state_quaternion_attitude_quaternion_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) { //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
	
	return (Vhil_state_quaternion_attitude_quaternion) {src + 12, 4};
}


static inline void vhil_state_quaternion_attitude_quaternion_SET(float src, size_t index, Vhil_state_quaternion_attitude_quaternion *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vhil_state_quaternion_attitude_quaternion phil_state_quaternion_attitude_quaternion_SET(const float src[], phil_state_quaternion_HIL_STATE_QUATERNION *dst) {//Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
	
	
	Vhil_state_quaternion_attitude_quaternion ret = {dst + 12, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vhil_state_quaternion_attitude_quaternion_SET(src[i], i, &ret);
	return ret;
}


static inline float phil_state_quaternion_rollspeed_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Body frame roll / phi angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void phil_state_quaternion_rollspeed_SET(float src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Body frame roll / phi angular speed (rad/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float phil_state_quaternion_pitchspeed_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Body frame pitch / theta angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void phil_state_quaternion_pitchspeed_SET(float src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Body frame pitch / theta angular speed (rad/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}


static inline float phil_state_quaternion_yawspeed_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Body frame yaw / psi angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 36, 4)));
}


static inline void phil_state_quaternion_yawspeed_SET(float src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Body frame yaw / psi angular speed (rad/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 36);
}


static inline int32_t phil_state_quaternion_lat_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Latitude, expressed as * 1E7
	
	return ((int32_t) (get_bytes(src, 40, 4)));
}


static inline void phil_state_quaternion_lat_SET(int32_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Latitude, expressed as * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 40);
}


static inline int32_t phil_state_quaternion_lon_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Longitude, expressed as * 1E7
	
	return ((int32_t) (get_bytes(src, 44, 4)));
}


static inline void phil_state_quaternion_lon_SET(int32_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Longitude, expressed as * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 44);
}


static inline int32_t phil_state_quaternion_alt_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Altitude in meters, expressed as * 1000 (millimeters)
	
	return ((int32_t) (get_bytes(src, 48, 4)));
}


static inline void phil_state_quaternion_alt_SET(int32_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Altitude in meters, expressed as * 1000 (millimeters)
	
	
	set_bytes((uint32_t) (src), 4, dst, 48);
}


static inline int16_t phil_state_quaternion_vx_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Ground X Speed (Latitude), expressed as cm/s
	
	return ((int16_t) (get_bytes(src, 52, 2)));
}


static inline void phil_state_quaternion_vx_SET(int16_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Ground X Speed (Latitude), expressed as cm/s
	
	
	set_bytes((uint16_t) (src), 2, dst, 52);
}


static inline int16_t phil_state_quaternion_vy_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Ground Y Speed (Longitude), expressed as cm/s
	
	return ((int16_t) (get_bytes(src, 54, 2)));
}


static inline void phil_state_quaternion_vy_SET(int16_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Ground Y Speed (Longitude), expressed as cm/s
	
	
	set_bytes((uint16_t) (src), 2, dst, 54);
}


static inline int16_t phil_state_quaternion_vz_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Ground Z Speed (Altitude), expressed as cm/s
	
	return ((int16_t) (get_bytes(src, 56, 2)));
}


static inline void phil_state_quaternion_vz_SET(int16_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Ground Z Speed (Altitude), expressed as cm/s
	
	
	set_bytes((uint16_t) (src), 2, dst, 56);
}


static inline int16_t phil_state_quaternion_xacc_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//X acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 58, 2)));
}


static inline void phil_state_quaternion_xacc_SET(int16_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //X acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 58);
}


static inline int16_t phil_state_quaternion_yacc_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Y acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 60, 2)));
}


static inline void phil_state_quaternion_yacc_SET(int16_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Y acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 60);
}


static inline int16_t phil_state_quaternion_zacc_GET(phil_state_quaternion_HIL_STATE_QUATERNION *src) {//Z acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 62, 2)));
}


static inline void phil_state_quaternion_zacc_SET(int16_t src, phil_state_quaternion_HIL_STATE_QUATERNION *dst) { //Z acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 62);
}


static inline int16_t psensor_offsets_mag_ofs_x_GET(psensor_offsets_SENSOR_OFFSETS *src) {//magnetometer X offset
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void psensor_offsets_mag_ofs_x_SET(int16_t src, psensor_offsets_SENSOR_OFFSETS *dst) { //magnetometer X offset
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t psensor_offsets_mag_ofs_y_GET(psensor_offsets_SENSOR_OFFSETS *src) {//magnetometer Y offset
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void psensor_offsets_mag_ofs_y_SET(int16_t src, psensor_offsets_SENSOR_OFFSETS *dst) { //magnetometer Y offset
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t psensor_offsets_mag_ofs_z_GET(psensor_offsets_SENSOR_OFFSETS *src) {//magnetometer Z offset
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void psensor_offsets_mag_ofs_z_SET(int16_t src, psensor_offsets_SENSOR_OFFSETS *dst) { //magnetometer Z offset
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline float psensor_offsets_mag_declination_GET(psensor_offsets_SENSOR_OFFSETS *src) {//magnetic declination (radians)
	
	return (intBitsToFloat(get_bytes(src, 6, 4)));
}


static inline void psensor_offsets_mag_declination_SET(float src, psensor_offsets_SENSOR_OFFSETS *dst) { //magnetic declination (radians)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 6);
}


static inline int32_t psensor_offsets_raw_press_GET(psensor_offsets_SENSOR_OFFSETS *src) {//raw pressure from barometer
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}


static inline void psensor_offsets_raw_press_SET(int32_t src, psensor_offsets_SENSOR_OFFSETS *dst) { //raw pressure from barometer
	
	
	set_bytes((uint32_t) (src), 4, dst, 10);
}


static inline int32_t psensor_offsets_raw_temp_GET(psensor_offsets_SENSOR_OFFSETS *src) {//raw temperature from barometer
	
	return ((int32_t) (get_bytes(src, 14, 4)));
}


static inline void psensor_offsets_raw_temp_SET(int32_t src, psensor_offsets_SENSOR_OFFSETS *dst) { //raw temperature from barometer
	
	
	set_bytes((uint32_t) (src), 4, dst, 14);
}


static inline float psensor_offsets_gyro_cal_x_GET(psensor_offsets_SENSOR_OFFSETS *src) {//gyro X calibration
	
	return (intBitsToFloat(get_bytes(src, 18, 4)));
}


static inline void psensor_offsets_gyro_cal_x_SET(float src, psensor_offsets_SENSOR_OFFSETS *dst) { //gyro X calibration
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 18);
}


static inline float psensor_offsets_gyro_cal_y_GET(psensor_offsets_SENSOR_OFFSETS *src) {//gyro Y calibration
	
	return (intBitsToFloat(get_bytes(src, 22, 4)));
}


static inline void psensor_offsets_gyro_cal_y_SET(float src, psensor_offsets_SENSOR_OFFSETS *dst) { //gyro Y calibration
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 22);
}


static inline float psensor_offsets_gyro_cal_z_GET(psensor_offsets_SENSOR_OFFSETS *src) {//gyro Z calibration
	
	return (intBitsToFloat(get_bytes(src, 26, 4)));
}


static inline void psensor_offsets_gyro_cal_z_SET(float src, psensor_offsets_SENSOR_OFFSETS *dst) { //gyro Z calibration
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 26);
}


static inline float psensor_offsets_accel_cal_x_GET(psensor_offsets_SENSOR_OFFSETS *src) {//accel X calibration
	
	return (intBitsToFloat(get_bytes(src, 30, 4)));
}


static inline void psensor_offsets_accel_cal_x_SET(float src, psensor_offsets_SENSOR_OFFSETS *dst) { //accel X calibration
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 30);
}


static inline float psensor_offsets_accel_cal_y_GET(psensor_offsets_SENSOR_OFFSETS *src) {//accel Y calibration
	
	return (intBitsToFloat(get_bytes(src, 34, 4)));
}


static inline void psensor_offsets_accel_cal_y_SET(float src, psensor_offsets_SENSOR_OFFSETS *dst) { //accel Y calibration
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 34);
}


static inline float psensor_offsets_accel_cal_z_GET(psensor_offsets_SENSOR_OFFSETS *src) {//accel Z calibration
	
	return (intBitsToFloat(get_bytes(src, 38, 4)));
}


static inline void psensor_offsets_accel_cal_z_SET(float src, psensor_offsets_SENSOR_OFFSETS *dst) { //accel Z calibration
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 38);
}


static inline int32_t pstorage_information_time_boot_ms_GET(pstorage_information_STORAGE_INFORMATION *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pstorage_information_time_boot_ms_SET(int32_t src, pstorage_information_STORAGE_INFORMATION *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int8_t pstorage_information_storage_id_GET(pstorage_information_STORAGE_INFORMATION *src) {//Storage ID (1 for first, 2 for second, etc.)
	
	return (int8_t) src[4];
}


static inline void pstorage_information_storage_id_SET(int8_t src, pstorage_information_STORAGE_INFORMATION *dst) { //Storage ID (1 for first, 2 for second, etc.)
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t pstorage_information_storage_count_GET(pstorage_information_STORAGE_INFORMATION *src) {//Number of storage devices
	
	return (int8_t) src[5];
}


static inline void pstorage_information_storage_count_SET(int8_t src, pstorage_information_STORAGE_INFORMATION *dst) { //Number of storage devices
	
	
	dst[5] = (uint8_t) (src);
}


static inline int8_t pstorage_information_status_GET(pstorage_information_STORAGE_INFORMATION *src) {//Status of storage (0 not available, 1 unformatted, 2 formatted)
	
	return (int8_t) src[6];
}


static inline void pstorage_information_status_SET(int8_t src, pstorage_information_STORAGE_INFORMATION *dst) { //Status of storage (0 not available, 1 unformatted, 2 formatted)
	
	
	dst[6] = (uint8_t) (src);
}


static inline float pstorage_information_total_capacity_GET(pstorage_information_STORAGE_INFORMATION *src) {//Total capacity in MiB
	
	return (intBitsToFloat(get_bytes(src, 7, 4)));
}


static inline void pstorage_information_total_capacity_SET(float src, pstorage_information_STORAGE_INFORMATION *dst) { //Total capacity in MiB
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 7);
}


static inline float pstorage_information_used_capacity_GET(pstorage_information_STORAGE_INFORMATION *src) {//Used capacity in MiB
	
	return (intBitsToFloat(get_bytes(src, 11, 4)));
}


static inline void pstorage_information_used_capacity_SET(float src, pstorage_information_STORAGE_INFORMATION *dst) { //Used capacity in MiB
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 11);
}


static inline float pstorage_information_available_capacity_GET(pstorage_information_STORAGE_INFORMATION *src) {//Available capacity in MiB
	
	return (intBitsToFloat(get_bytes(src, 15, 4)));
}


static inline void pstorage_information_available_capacity_SET(float src, pstorage_information_STORAGE_INFORMATION *dst) { //Available capacity in MiB
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 15);
}


static inline float pstorage_information_read_speed_GET(pstorage_information_STORAGE_INFORMATION *src) {//Read speed in MiB/s
	
	return (intBitsToFloat(get_bytes(src, 19, 4)));
}


static inline void pstorage_information_read_speed_SET(float src, pstorage_information_STORAGE_INFORMATION *dst) { //Read speed in MiB/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 19);
}


static inline float pstorage_information_write_speed_GET(pstorage_information_STORAGE_INFORMATION *src) {//Write speed in MiB/s
	
	return (intBitsToFloat(get_bytes(src, 23, 4)));
}


static inline void pstorage_information_write_speed_SET(float src, pstorage_information_STORAGE_INFORMATION *dst) { //Write speed in MiB/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 23);
}


static inline int16_t pcamera_information_resolution_h_GET(pcamera_information_CAMERA_INFORMATION *src) {//Image resolution in pixels horizontal
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pcamera_information_resolution_h_SET(int16_t src, pcamera_information_CAMERA_INFORMATION *dst) { //Image resolution in pixels horizontal
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t pcamera_information_resolution_v_GET(pcamera_information_CAMERA_INFORMATION *src) {//Image resolution in pixels vertical
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pcamera_information_resolution_v_SET(int16_t src, pcamera_information_CAMERA_INFORMATION *dst) { //Image resolution in pixels vertical
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int16_t pcamera_information_cam_definition_version_GET(pcamera_information_CAMERA_INFORMATION *src) {//Camera definition version (iteration)
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}


static inline void pcamera_information_cam_definition_version_SET(int16_t src, pcamera_information_CAMERA_INFORMATION *dst) { //Camera definition version (iteration)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 4);
}


static inline int32_t pcamera_information_time_boot_ms_GET(pcamera_information_CAMERA_INFORMATION *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline void pcamera_information_time_boot_ms_SET(int32_t src, pcamera_information_CAMERA_INFORMATION *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 6);
}


static inline int32_t pcamera_information_firmware_version_GET(pcamera_information_CAMERA_INFORMATION *src) {//0xff = Major)
	
	return ((int32_t) (get_bytes(src->base.bytes, 10, 4)));
}


static inline void pcamera_information_firmware_version_SET(int32_t src, pcamera_information_CAMERA_INFORMATION *dst) { //0xff = Major)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 10);
}


static inline int8_t vcamera_information_vendor_name_GET(Vcamera_information_vendor_name const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vcamera_information_vendor_name pcamera_information_vendor_name_GET(pcamera_information_CAMERA_INFORMATION *src) { //Name of the camera vendor
	
	return (Vcamera_information_vendor_name) {src->base.bytes + 14, 32};
}


static inline void vcamera_information_vendor_name_SET(int8_t src, size_t index, Vcamera_information_vendor_name *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vcamera_information_vendor_name pcamera_information_vendor_name_SET(const int8_t src[], pcamera_information_CAMERA_INFORMATION *dst) {//Name of the camera vendor
	
	
	Vcamera_information_vendor_name ret = {dst->base.bytes + 14, 32};
	
	if (src)
		for (size_t i = 0; i < 32; i++)
			vcamera_information_vendor_name_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t vcamera_information_model_name_GET(Vcamera_information_model_name const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vcamera_information_model_name pcamera_information_model_name_GET(pcamera_information_CAMERA_INFORMATION *src) { //Name of the camera model
	
	return (Vcamera_information_model_name) {src->base.bytes + 46, 32};
}


static inline void vcamera_information_model_name_SET(int8_t src, size_t index, Vcamera_information_model_name *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vcamera_information_model_name pcamera_information_model_name_SET(const int8_t src[], pcamera_information_CAMERA_INFORMATION *dst) {//Name of the camera model
	
	
	Vcamera_information_model_name ret = {dst->base.bytes + 46, 32};
	
	if (src)
		for (size_t i = 0; i < 32; i++)
			vcamera_information_model_name_SET(src[i], i, &ret);
	return ret;
}


static inline float pcamera_information_focal_length_GET(pcamera_information_CAMERA_INFORMATION *src) {//Focal length in mm
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 78, 4)));
}


static inline void pcamera_information_focal_length_SET(float src, pcamera_information_CAMERA_INFORMATION *dst) { //Focal length in mm
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 78);
}


static inline float pcamera_information_sensor_size_h_GET(pcamera_information_CAMERA_INFORMATION *src) {//Image sensor size horizontal in mm
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 82, 4)));
}


static inline void pcamera_information_sensor_size_h_SET(float src, pcamera_information_CAMERA_INFORMATION *dst) { //Image sensor size horizontal in mm
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 82);
}


static inline float pcamera_information_sensor_size_v_GET(pcamera_information_CAMERA_INFORMATION *src) {//Image sensor size vertical in mm
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 86, 4)));
}


static inline void pcamera_information_sensor_size_v_SET(float src, pcamera_information_CAMERA_INFORMATION *dst) { //Image sensor size vertical in mm
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 86);
}


static inline int8_t pcamera_information_lens_id_GET(pcamera_information_CAMERA_INFORMATION *src) {//Reserved for a lens ID
	
	return (int8_t) src->base.bytes[90];
}


static inline void pcamera_information_lens_id_SET(int8_t src, pcamera_information_CAMERA_INFORMATION *dst) { //Reserved for a lens ID
	
	
	dst->base.bytes[90] = (uint8_t) (src);
}

static inline bool pcamera_information_flags_GET(pcamera_information_CAMERA_INFORMATION *const src, e_CAMERA_CAP_FLAGS *ret) {
	if (src->base.field_bit != 730 && !set_field(src, 730, -1)) return false;
	*ret = _en_camera_cap_flags(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pcamera_information_flags_SET(e_CAMERA_CAP_FLAGS src, pcamera_information_CAMERA_INFORMATION *const dst) {
	
	if (dst->base.field_bit != 730) set_field(dst, 730, 0);
	
	
	UMAX id = _id_camera_cap_flags(src);
	set_bits(id, 3, dst->base.bytes, dst->BIT);
}

static inline bool pcamera_information_cam_definition_uri_GET(pcamera_information_CAMERA_INFORMATION *const src, Vcamera_information_cam_definition_uri *ret) {//Camera definition URI (if any, otherwise only basic functions will be available).
	
	if (src->base.field_bit != 731 && !set_field(src, 731, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vcamera_information_cam_definition_uri pcamera_information_cam_definition_uri_SET(const char src[], size_t len, pcamera_information_CAMERA_INFORMATION *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 731, len);
	
	Vcamera_information_cam_definition_uri ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int8_t pgps_status_satellites_visible_GET(pgps_status_GPS_STATUS *src) {//Number of satellites visible
	
	return (int8_t) src[0];
}


static inline int8_t vgps_status_satellite_prn_GET(Vgps_status_satellite_prn const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgps_status_satellite_prn pgps_status_satellite_prn_GET(pgps_status_GPS_STATUS *src) { //Global satellite ID
	
	return (Vgps_status_satellite_prn) {src + 1, 20};
}


static inline int8_t vgps_status_satellite_used_GET(Vgps_status_satellite_used const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgps_status_satellite_used pgps_status_satellite_used_GET(pgps_status_GPS_STATUS *src) { //0: Satellite not used, 1: used for localization
	
	return (Vgps_status_satellite_used) {src + 21, 20};
}


static inline int8_t vgps_status_satellite_elevation_GET(Vgps_status_satellite_elevation const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgps_status_satellite_elevation pgps_status_satellite_elevation_GET(pgps_status_GPS_STATUS *src) { //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	
	return (Vgps_status_satellite_elevation) {src + 41, 20};
}


static inline int8_t vgps_status_satellite_azimuth_GET(Vgps_status_satellite_azimuth const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgps_status_satellite_azimuth pgps_status_satellite_azimuth_GET(pgps_status_GPS_STATUS *src) { //Direction of satellite, 0: 0 deg, 255: 360 deg.
	
	return (Vgps_status_satellite_azimuth) {src + 61, 20};
}


static inline int8_t vgps_status_satellite_snr_GET(Vgps_status_satellite_snr const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgps_status_satellite_snr pgps_status_satellite_snr_GET(pgps_status_GPS_STATUS *src) { //Signal to noise ratio of satellite
	
	return (Vgps_status_satellite_snr) {src + 81, 20};
}


static inline int32_t pdevice_op_write_reply_request_id_GET(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *src) {//request ID - copied from request
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pdevice_op_write_reply_request_id_SET(int32_t src, pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *dst) { //request ID - copied from request
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int8_t pdevice_op_write_reply_result_GET(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *src) {//0 for success, anything else is failure code
	
	return (int8_t) src[4];
}


static inline void pdevice_op_write_reply_result_SET(int8_t src, pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *dst) { //0 for success, anything else is failure code
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t pparam_set_target_system_GET(pparam_set_PARAM_SET *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pparam_set_target_component_GET(pparam_set_PARAM_SET *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline float pparam_set_param_value_GET(pparam_set_PARAM_SET *src) {//Onboard parameter value
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 2, 4)));
}

/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*					 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*					 storage if the ID is stored as strin */

static inline bool pparam_set_param_id_GET(pparam_set_PARAM_SET *const src, Vparam_set_param_id *ret) {
	
	if (src->base.field_bit != 50 && !set_field(src, 50, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline bool pparam_set_param_type_GET(pparam_set_PARAM_SET *const src, e_MAV_PARAM_TYPE *ret) {
	if (src->base.field_bit != 51 && !set_field(src, 51, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 4));
	return true;
}


static inline int16_t pterrain_data_grid_spacing_GET(pterrain_data_TERRAIN_DATA *src) {//Grid spacing in meters
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pterrain_data_grid_spacing_SET(int16_t src, pterrain_data_TERRAIN_DATA *dst) { //Grid spacing in meters
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int32_t pterrain_data_lat_GET(pterrain_data_TERRAIN_DATA *src) {//Latitude of SW corner of first grid (degrees *10^7)
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline void pterrain_data_lat_SET(int32_t src, pterrain_data_TERRAIN_DATA *dst) { //Latitude of SW corner of first grid (degrees *10^7)
	
	
	set_bytes((uint32_t) (src), 4, dst, 2);
}


static inline int32_t pterrain_data_lon_GET(pterrain_data_TERRAIN_DATA *src) {//Longitude of SW corner of first grid (in degrees *10^7)
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void pterrain_data_lon_SET(int32_t src, pterrain_data_TERRAIN_DATA *dst) { //Longitude of SW corner of first grid (in degrees *10^7)
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int8_t pterrain_data_gridbit_GET(pterrain_data_TERRAIN_DATA *src) {//bit within the terrain request mask
	
	return (int8_t) src[10];
}


static inline void pterrain_data_gridbit_SET(int8_t src, pterrain_data_TERRAIN_DATA *dst) { //bit within the terrain request mask
	
	
	dst[10] = (uint8_t) (src);
}


static inline int16_t vterrain_data_daTa_GET(Vterrain_data_daTa const *const src, size_t index) { return ((int16_t) (get_bytes(src->bytes, index * 2, 2))); }

static inline Vterrain_data_daTa pterrain_data_daTa_GET(pterrain_data_TERRAIN_DATA *src) { //Terrain data in meters AMSL
	
	return (Vterrain_data_daTa) {src + 11, 16};
}


static inline void vterrain_data_daTa_SET(int16_t src, size_t index, Vterrain_data_daTa *dst) { set_bytes((uint16_t) (src), 2, dst->bytes, index * 2); }

static inline Vterrain_data_daTa pterrain_data_daTa_SET(const int16_t src[], pterrain_data_TERRAIN_DATA *dst) {//Terrain data in meters AMSL
	
	
	Vterrain_data_daTa ret = {dst + 11, 16};
	
	if (src)
		for (size_t i = 0; i < 16; i++)
			vterrain_data_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t pgimbal_control_target_system_GET(pgimbal_control_GIMBAL_CONTROL *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pgimbal_control_target_system_SET(int8_t src, pgimbal_control_GIMBAL_CONTROL *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pgimbal_control_target_component_GET(pgimbal_control_GIMBAL_CONTROL *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pgimbal_control_target_component_SET(int8_t src, pgimbal_control_GIMBAL_CONTROL *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline float pgimbal_control_demanded_rate_x_GET(pgimbal_control_GIMBAL_CONTROL *src) {//Demanded angular rate X (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 2, 4)));
}


static inline void pgimbal_control_demanded_rate_x_SET(float src, pgimbal_control_GIMBAL_CONTROL *dst) { //Demanded angular rate X (rad/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 2);
}


static inline float pgimbal_control_demanded_rate_y_GET(pgimbal_control_GIMBAL_CONTROL *src) {//Demanded angular rate Y (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 6, 4)));
}


static inline void pgimbal_control_demanded_rate_y_SET(float src, pgimbal_control_GIMBAL_CONTROL *dst) { //Demanded angular rate Y (rad/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 6);
}


static inline float pgimbal_control_demanded_rate_z_GET(pgimbal_control_GIMBAL_CONTROL *src) {//Demanded angular rate Z (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 10, 4)));
}


static inline void pgimbal_control_demanded_rate_z_SET(float src, pgimbal_control_GIMBAL_CONTROL *dst) { //Demanded angular rate Z (rad/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 10);
}


static inline int16_t prc_channels_override_chan1_raw_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int16_t prc_channels_override_chan2_raw_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline int16_t prc_channels_override_chan3_raw_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline int16_t prc_channels_override_chan4_raw_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline int16_t prc_channels_override_chan5_raw_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline int16_t prc_channels_override_chan6_raw_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline int16_t prc_channels_override_chan7_raw_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline int16_t prc_channels_override_chan8_raw_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline int8_t prc_channels_override_target_system_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//System ID
	
	return (int8_t) src[16];
}


static inline int8_t prc_channels_override_target_component_GET(prc_channels_override_RC_CHANNELS_OVERRIDE *src) {//Component ID
	
	return (int8_t) src[17];
}


static inline int32_t pscaled_imu_time_boot_ms_GET(pscaled_imu_SCALED_IMU *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline int16_t pscaled_imu_xacc_GET(pscaled_imu_SCALED_IMU *src) {//X acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline int16_t pscaled_imu_yacc_GET(pscaled_imu_SCALED_IMU *src) {//Y acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline int16_t pscaled_imu_zacc_GET(pscaled_imu_SCALED_IMU *src) {//Z acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline int16_t pscaled_imu_xgyro_GET(pscaled_imu_SCALED_IMU *src) {//Angular speed around X axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline int16_t pscaled_imu_ygyro_GET(pscaled_imu_SCALED_IMU *src) {//Angular speed around Y axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline int16_t pscaled_imu_zgyro_GET(pscaled_imu_SCALED_IMU *src) {//Angular speed around Z axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline int16_t pscaled_imu_xmag_GET(pscaled_imu_SCALED_IMU *src) {//X Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 16, 2)));
}


static inline int16_t pscaled_imu_ymag_GET(pscaled_imu_SCALED_IMU *src) {//Y Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 18, 2)));
}


static inline int16_t pscaled_imu_zmag_GET(pscaled_imu_SCALED_IMU *src) {//Z Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 20, 2)));
}


static inline int16_t pvideo_stream_information_resolution_h_GET(pvideo_stream_information_VIDEO_STREAM_INFORMATION *src) {//Resolution horizontal in pixels
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pvideo_stream_information_resolution_h_SET(int16_t src, pvideo_stream_information_VIDEO_STREAM_INFORMATION *dst) { //Resolution horizontal in pixels
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t pvideo_stream_information_resolution_v_GET(pvideo_stream_information_VIDEO_STREAM_INFORMATION *src) {//Resolution vertical in pixels
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pvideo_stream_information_resolution_v_SET(int16_t src, pvideo_stream_information_VIDEO_STREAM_INFORMATION *dst) { //Resolution vertical in pixels
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int16_t pvideo_stream_information_rotation_GET(pvideo_stream_information_VIDEO_STREAM_INFORMATION *src) {//Video image rotation clockwise
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}


static inline void pvideo_stream_information_rotation_SET(int16_t src, pvideo_stream_information_VIDEO_STREAM_INFORMATION *dst) { //Video image rotation clockwise
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 4);
}


static inline int32_t pvideo_stream_information_bitrate_GET(pvideo_stream_information_VIDEO_STREAM_INFORMATION *src) {//Bit rate in bits per second
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline void pvideo_stream_information_bitrate_SET(int32_t src, pvideo_stream_information_VIDEO_STREAM_INFORMATION *dst) { //Bit rate in bits per second
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 6);
}


static inline int8_t pvideo_stream_information_camera_id_GET(pvideo_stream_information_VIDEO_STREAM_INFORMATION *src) {//Camera ID (1 for first, 2 for second, etc.)
	
	return (int8_t) src->base.bytes[10];
}


static inline void pvideo_stream_information_camera_id_SET(int8_t src, pvideo_stream_information_VIDEO_STREAM_INFORMATION *dst) { //Camera ID (1 for first, 2 for second, etc.)
	
	
	dst->base.bytes[10] = (uint8_t) (src);
}


static inline int8_t pvideo_stream_information_status_GET(pvideo_stream_information_VIDEO_STREAM_INFORMATION *src) {//Current status of video streaming (0: not running, 1: in progress)
	
	return (int8_t) src->base.bytes[11];
}


static inline void pvideo_stream_information_status_SET(int8_t src, pvideo_stream_information_VIDEO_STREAM_INFORMATION *dst) { //Current status of video streaming (0: not running, 1: in progress)
	
	
	dst->base.bytes[11] = (uint8_t) (src);
}


static inline float pvideo_stream_information_framerate_GET(pvideo_stream_information_VIDEO_STREAM_INFORMATION *src) {//Frames per second
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void pvideo_stream_information_framerate_SET(float src, pvideo_stream_information_VIDEO_STREAM_INFORMATION *dst) { //Frames per second
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}

static inline bool pvideo_stream_information_uri_GET(pvideo_stream_information_VIDEO_STREAM_INFORMATION *const src, Vvideo_stream_information_uri *ret) {//Video stream URI
	
	if (src->base.field_bit != 130 && !set_field(src, 130, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vvideo_stream_information_uri pvideo_stream_information_uri_SET(const char src[], size_t len, pvideo_stream_information_VIDEO_STREAM_INFORMATION *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 130, len);
	
	Vvideo_stream_information_uri ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline float pahrs_omegaIx_GET(pahrs_AHRS *src) {//X gyro drift estimate rad/s
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void pahrs_omegaIx_SET(float src, pahrs_AHRS *dst) { //X gyro drift estimate rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float pahrs_omegaIy_GET(pahrs_AHRS *src) {//Y gyro drift estimate rad/s
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pahrs_omegaIy_SET(float src, pahrs_AHRS *dst) { //Y gyro drift estimate rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pahrs_omegaIz_GET(pahrs_AHRS *src) {//Z gyro drift estimate rad/s
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pahrs_omegaIz_SET(float src, pahrs_AHRS *dst) { //Z gyro drift estimate rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pahrs_accel_weight_GET(pahrs_AHRS *src) {//average accel_weight
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pahrs_accel_weight_SET(float src, pahrs_AHRS *dst) { //average accel_weight
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float pahrs_renorm_val_GET(pahrs_AHRS *src) {//average renormalisation value
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void pahrs_renorm_val_SET(float src, pahrs_AHRS *dst) { //average renormalisation value
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float pahrs_error_rp_GET(pahrs_AHRS *src) {//average error_roll_pitch value
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void pahrs_error_rp_SET(float src, pahrs_AHRS *dst) { //average error_roll_pitch value
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float pahrs_error_yaw_GET(pahrs_AHRS *src) {//average error_yaw value
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void pahrs_error_yaw_SET(float src, pahrs_AHRS *dst) { //average error_yaw value
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline int32_t pdebug_time_boot_ms_GET(pdebug_DEBUG *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pdebug_time_boot_ms_SET(int32_t src, pdebug_DEBUG *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int8_t pdebug_ind_GET(pdebug_DEBUG *src) {//index of debug variable
	
	return (int8_t) src[4];
}


static inline void pdebug_ind_SET(int8_t src, pdebug_DEBUG *dst) { //index of debug variable
	
	
	dst[4] = (uint8_t) (src);
}


static inline float pdebug_value_GET(pdebug_DEBUG *src) {//DEBUG value
	
	return (intBitsToFloat(get_bytes(src, 5, 4)));
}


static inline void pdebug_value_SET(float src, pdebug_DEBUG *dst) { //DEBUG value
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 5);
}


static inline int32_t pcamera_image_captured_time_boot_ms_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void pcamera_image_captured_time_boot_ms_SET(int32_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline int64_t pcamera_image_captured_time_utc_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
	
	return ((int64_t) (get_bytes(src->base.bytes, 4, 8)));
}


static inline void pcamera_image_captured_time_utc_SET(int64_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
	
	
	set_bytes((src), 8, dst->base.bytes, 4);
}


static inline int8_t pcamera_image_captured_camera_id_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Camera ID (1 for first, 2 for second, etc.)
	
	return (int8_t) src->base.bytes[12];
}


static inline void pcamera_image_captured_camera_id_SET(int8_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Camera ID (1 for first, 2 for second, etc.)
	
	
	dst->base.bytes[12] = (uint8_t) (src);
}


static inline int32_t pcamera_image_captured_lat_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Latitude, expressed as degrees * 1E7 where image was taken
	
	return ((int32_t) (get_bytes(src->base.bytes, 13, 4)));
}


static inline void pcamera_image_captured_lat_SET(int32_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Latitude, expressed as degrees * 1E7 where image was taken
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 13);
}


static inline int32_t pcamera_image_captured_lon_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Longitude, expressed as degrees * 1E7 where capture was taken
	
	return ((int32_t) (get_bytes(src->base.bytes, 17, 4)));
}


static inline void pcamera_image_captured_lon_SET(int32_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Longitude, expressed as degrees * 1E7 where capture was taken
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 17);
}


static inline int32_t pcamera_image_captured_alt_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
	
	return ((int32_t) (get_bytes(src->base.bytes, 21, 4)));
}


static inline void pcamera_image_captured_alt_SET(int32_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 21);
}


static inline int32_t pcamera_image_captured_relative_alt_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Altitude above ground in meters, expressed as * 1E3 where image was taken
	
	return ((int32_t) (get_bytes(src->base.bytes, 25, 4)));
}


static inline void pcamera_image_captured_relative_alt_SET(int32_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Altitude above ground in meters, expressed as * 1E3 where image was taken
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 25);
}


static inline float vcamera_image_captured_q_GET(Vcamera_image_captured_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vcamera_image_captured_q pcamera_image_captured_q_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) { //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
	
	return (Vcamera_image_captured_q) {src->base.bytes + 29, 4};
}


static inline void vcamera_image_captured_q_SET(float src, size_t index, Vcamera_image_captured_q *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vcamera_image_captured_q pcamera_image_captured_q_SET(const float src[], pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) {//Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
	
	
	Vcamera_image_captured_q ret = {dst->base.bytes + 29, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vcamera_image_captured_q_SET(src[i], i, &ret);
	return ret;
}


static inline int32_t pcamera_image_captured_image_index_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Zero based index of this image (image count since armed -1)
	
	return ((int32_t) (get_bytes(src->base.bytes, 45, 4)));
}


static inline void pcamera_image_captured_image_index_SET(int32_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Zero based index of this image (image count since armed -1)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 45);
}


static inline int8_t pcamera_image_captured_capture_result_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *src) {//Boolean indicating success (1) or failure (0) while capturing this image.
	
	return (int8_t) src->base.bytes[49];
}


static inline void pcamera_image_captured_capture_result_SET(int8_t src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *dst) { //Boolean indicating success (1) or failure (0) while capturing this image.
	
	
	dst->base.bytes[49] = (uint8_t) (src);
}

static inline bool pcamera_image_captured_file_url_GET(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *const src, Vcamera_image_captured_file_url *ret) {//URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
	
	if (src->base.field_bit != 402 && !set_field(src, 402, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vcamera_image_captured_file_url pcamera_image_captured_file_url_SET(const char src[], size_t len, pcamera_image_captured_CAMERA_IMAGE_CAPTURED *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 402, len);
	
	Vcamera_image_captured_file_url ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int16_t plog_entry_id_GET(plog_entry_LOG_ENTRY *src) {//Log id
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void plog_entry_id_SET(int16_t src, plog_entry_LOG_ENTRY *dst) { //Log id
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t plog_entry_num_logs_GET(plog_entry_LOG_ENTRY *src) {//Total number of logs
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void plog_entry_num_logs_SET(int16_t src, plog_entry_LOG_ENTRY *dst) { //Total number of logs
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t plog_entry_last_log_num_GET(plog_entry_LOG_ENTRY *src) {//High log number
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void plog_entry_last_log_num_SET(int16_t src, plog_entry_LOG_ENTRY *dst) { //High log number
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int32_t plog_entry_time_utc_GET(plog_entry_LOG_ENTRY *src) {//UTC timestamp of log in seconds since 1970, or 0 if not available
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void plog_entry_time_utc_SET(int32_t src, plog_entry_LOG_ENTRY *dst) { //UTC timestamp of log in seconds since 1970, or 0 if not available
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int32_t plog_entry_size_GET(plog_entry_LOG_ENTRY *src) {//Size of the log (may be approximate) in bytes
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}


static inline void plog_entry_size_SET(int32_t src, plog_entry_LOG_ENTRY *dst) { //Size of the log (may be approximate) in bytes
	
	
	set_bytes((uint32_t) (src), 4, dst, 10);
}


static inline int64_t pactuator_control_target_time_usec_GET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void pactuator_control_target_time_usec_SET(int64_t src, pactuator_control_target_ACTUATOR_CONTROL_TARGET *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst, 0);
}

/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*					 this field to difference between instances */

static inline int8_t pactuator_control_target_group_mlx_GET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *src) {
	
	return (int8_t) src[8];
}

/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*					 this field to difference between instances */

static inline void pactuator_control_target_group_mlx_SET(int8_t src, pactuator_control_target_ACTUATOR_CONTROL_TARGET *dst) {
	
	
	dst[8] = (uint8_t) (src);
}

/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*					 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*					 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*					 mixer to repurpose them as generic outputs */

static inline float vactuator_control_target_controls_GET(Vactuator_control_target_controls const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*					 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*					 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*					 mixer to repurpose them as generic outputs */

static inline Vactuator_control_target_controls pactuator_control_target_controls_GET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *src) {
	
	return (Vactuator_control_target_controls) {src + 9, 8};
}

/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*					 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*					 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*					 mixer to repurpose them as generic outputs */

static inline void vactuator_control_target_controls_SET(float src, size_t index, Vactuator_control_target_controls *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*					 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*					 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*					 mixer to repurpose them as generic outputs */

static inline Vactuator_control_target_controls pactuator_control_target_controls_SET(const float src[], pactuator_control_target_ACTUATOR_CONTROL_TARGET *dst) {
	
	
	Vactuator_control_target_controls ret = {dst + 9, 8};
	
	if (src)
		for (size_t i = 0; i < 8; i++)
			vactuator_control_target_controls_SET(src[i], i, &ret);
	return ret;
}


static inline int16_t phigh_latency_heading_GET(phigh_latency_HIGH_LATENCY *src) {//heading (centidegrees)
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void phigh_latency_heading_SET(int16_t src, phigh_latency_HIGH_LATENCY *dst) { //heading (centidegrees)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t phigh_latency_wp_distance_GET(phigh_latency_HIGH_LATENCY *src) {//distance to target (meters)
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void phigh_latency_wp_distance_SET(int16_t src, phigh_latency_HIGH_LATENCY *dst) { //distance to target (meters)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int32_t phigh_latency_custom_mode_GET(phigh_latency_HIGH_LATENCY *src) {//A bitfield for use for autopilot-specific flags.
	
	return ((int32_t) (get_bytes(src->base.bytes, 4, 4)));
}


static inline void phigh_latency_custom_mode_SET(int32_t src, phigh_latency_HIGH_LATENCY *dst) { //A bitfield for use for autopilot-specific flags.
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 4);
}


static inline int16_t phigh_latency_roll_GET(phigh_latency_HIGH_LATENCY *src) {//roll (centidegrees)
	
	return ((int16_t) (get_bytes(src->base.bytes, 8, 2)));
}


static inline void phigh_latency_roll_SET(int16_t src, phigh_latency_HIGH_LATENCY *dst) { //roll (centidegrees)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 8);
}


static inline int16_t phigh_latency_pitch_GET(phigh_latency_HIGH_LATENCY *src) {//pitch (centidegrees)
	
	return ((int16_t) (get_bytes(src->base.bytes, 10, 2)));
}


static inline void phigh_latency_pitch_SET(int16_t src, phigh_latency_HIGH_LATENCY *dst) { //pitch (centidegrees)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 10);
}


static inline int8_t phigh_latency_throttle_GET(phigh_latency_HIGH_LATENCY *src) {//throttle (percentage)
	
	return (int8_t) src->base.bytes[12];
}


static inline void phigh_latency_throttle_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //throttle (percentage)
	
	
	dst->base.bytes[12] = (uint8_t) (src);
}


static inline int16_t phigh_latency_heading_sp_GET(phigh_latency_HIGH_LATENCY *src) {//heading setpoint (centidegrees)
	
	return ((int16_t) (get_bytes(src->base.bytes, 13, 2)));
}


static inline void phigh_latency_heading_sp_SET(int16_t src, phigh_latency_HIGH_LATENCY *dst) { //heading setpoint (centidegrees)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 13);
}


static inline int32_t phigh_latency_latitude_GET(phigh_latency_HIGH_LATENCY *src) {//Latitude, expressed as degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 15, 4)));
}


static inline void phigh_latency_latitude_SET(int32_t src, phigh_latency_HIGH_LATENCY *dst) { //Latitude, expressed as degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 15);
}


static inline int32_t phigh_latency_longitude_GET(phigh_latency_HIGH_LATENCY *src) {//Longitude, expressed as degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 19, 4)));
}


static inline void phigh_latency_longitude_SET(int32_t src, phigh_latency_HIGH_LATENCY *dst) { //Longitude, expressed as degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 19);
}


static inline int16_t phigh_latency_altitude_amsl_GET(phigh_latency_HIGH_LATENCY *src) {//Altitude above mean sea level (meters)
	
	return ((int16_t) (get_bytes(src->base.bytes, 23, 2)));
}


static inline void phigh_latency_altitude_amsl_SET(int16_t src, phigh_latency_HIGH_LATENCY *dst) { //Altitude above mean sea level (meters)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 23);
}


static inline int16_t phigh_latency_altitude_sp_GET(phigh_latency_HIGH_LATENCY *src) {//Altitude setpoint relative to the home position (meters)
	
	return ((int16_t) (get_bytes(src->base.bytes, 25, 2)));
}


static inline void phigh_latency_altitude_sp_SET(int16_t src, phigh_latency_HIGH_LATENCY *dst) { //Altitude setpoint relative to the home position (meters)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 25);
}


static inline int8_t phigh_latency_airspeed_GET(phigh_latency_HIGH_LATENCY *src) {//airspeed (m/s)
	
	return (int8_t) src->base.bytes[27];
}


static inline void phigh_latency_airspeed_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //airspeed (m/s)
	
	
	dst->base.bytes[27] = (uint8_t) (src);
}


static inline int8_t phigh_latency_airspeed_sp_GET(phigh_latency_HIGH_LATENCY *src) {//airspeed setpoint (m/s)
	
	return (int8_t) src->base.bytes[28];
}


static inline void phigh_latency_airspeed_sp_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //airspeed setpoint (m/s)
	
	
	dst->base.bytes[28] = (uint8_t) (src);
}


static inline int8_t phigh_latency_groundspeed_GET(phigh_latency_HIGH_LATENCY *src) {//groundspeed (m/s)
	
	return (int8_t) src->base.bytes[29];
}


static inline void phigh_latency_groundspeed_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //groundspeed (m/s)
	
	
	dst->base.bytes[29] = (uint8_t) (src);
}


static inline int8_t phigh_latency_climb_rate_GET(phigh_latency_HIGH_LATENCY *src) {//climb rate (m/s)
	
	return (int8_t) src->base.bytes[30];
}


static inline void phigh_latency_climb_rate_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //climb rate (m/s)
	
	
	dst->base.bytes[30] = (uint8_t) (src);
}


static inline int8_t phigh_latency_gps_nsat_GET(phigh_latency_HIGH_LATENCY *src) {//Number of satellites visible. If unknown, set to 255
	
	return (int8_t) src->base.bytes[31];
}


static inline void phigh_latency_gps_nsat_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //Number of satellites visible. If unknown, set to 255
	
	
	dst->base.bytes[31] = (uint8_t) (src);
}


static inline int8_t phigh_latency_battery_remaining_GET(phigh_latency_HIGH_LATENCY *src) {//Remaining battery (percentage)
	
	return (int8_t) src->base.bytes[32];
}


static inline void phigh_latency_battery_remaining_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //Remaining battery (percentage)
	
	
	dst->base.bytes[32] = (uint8_t) (src);
}


static inline int8_t phigh_latency_temperature_GET(phigh_latency_HIGH_LATENCY *src) {//Autopilot temperature (degrees C)
	
	return (int8_t) src->base.bytes[33];
}


static inline void phigh_latency_temperature_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //Autopilot temperature (degrees C)
	
	
	dst->base.bytes[33] = (uint8_t) (src);
}


static inline int8_t phigh_latency_temperature_air_GET(phigh_latency_HIGH_LATENCY *src) {//Air temperature (degrees C) from airspeed sensor
	
	return (int8_t) src->base.bytes[34];
}


static inline void phigh_latency_temperature_air_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //Air temperature (degrees C) from airspeed sensor
	
	
	dst->base.bytes[34] = (uint8_t) (src);
}

/**
*failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
*					 bit3:GCS, bit4:fence */

static inline int8_t phigh_latency_failsafe_GET(phigh_latency_HIGH_LATENCY *src) {
	
	return (int8_t) src->base.bytes[35];
}

/**
*failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
*					 bit3:GCS, bit4:fence */

static inline void phigh_latency_failsafe_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) {
	
	
	dst->base.bytes[35] = (uint8_t) (src);
}


static inline int8_t phigh_latency_wp_num_GET(phigh_latency_HIGH_LATENCY *src) {//current waypoint number
	
	return (int8_t) src->base.bytes[36];
}


static inline void phigh_latency_wp_num_SET(int8_t src, phigh_latency_HIGH_LATENCY *dst) { //current waypoint number
	
	
	dst->base.bytes[36] = (uint8_t) (src);
}

static inline bool phigh_latency_base_mode_GET(phigh_latency_HIGH_LATENCY *const src, e_MAV_MODE_FLAG *ret) {
	if (src->base.field_bit != 298 && !set_field(src, 298, -1)) return false;
	*ret = _en_mav_mode_flag(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void phigh_latency_base_mode_SET(e_MAV_MODE_FLAG src, phigh_latency_HIGH_LATENCY *const dst) {
	
	if (dst->base.field_bit != 298) set_field(dst, 298, 0);
	
	
	UMAX id = _id_mav_mode_flag(src);
	set_bits(id, 4, dst->base.bytes, dst->BIT);
}

static inline bool phigh_latency_landed_state_GET(phigh_latency_HIGH_LATENCY *const src, e_MAV_LANDED_STATE *ret) {
	if (src->base.field_bit != 299 && !set_field(src, 299, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void phigh_latency_landed_state_SET(e_MAV_LANDED_STATE src, phigh_latency_HIGH_LATENCY *const dst) {
	
	if (dst->base.field_bit != 299) set_field(dst, 299, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool phigh_latency_gps_fix_type_GET(phigh_latency_HIGH_LATENCY *const src, e_GPS_FIX_TYPE *ret) {
	if (src->base.field_bit != 300 && !set_field(src, 300, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void phigh_latency_gps_fix_type_SET(e_GPS_FIX_TYPE src, phigh_latency_HIGH_LATENCY *const dst) {
	
	if (dst->base.field_bit != 300) set_field(dst, 300, 0);
	
	
	set_bits(src, 4, dst->base.bytes, dst->BIT);
}


static inline int8_t pparam_request_read_target_system_GET(pparam_request_read_PARAM_REQUEST_READ *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pparam_request_read_target_component_GET(pparam_request_read_PARAM_REQUEST_READ *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline int16_t pparam_request_read_param_index_GET(pparam_request_read_PARAM_REQUEST_READ *src) {//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}

/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*					 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*					 storage if the ID is stored as strin */

static inline bool pparam_request_read_param_id_GET(pparam_request_read_PARAM_REQUEST_READ *const src, Vparam_request_read_param_id *ret) {
	
	if (src->base.field_bit != 34 && !set_field(src, 34, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}


static inline int32_t pset_attitude_target_time_boot_ms_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) {//Timestamp in milliseconds since system boot
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline int8_t pset_attitude_target_target_system_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) {//System ID
	
	return (int8_t) src[4];
}


static inline int8_t pset_attitude_target_target_component_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) {//Component ID
	
	return (int8_t) src[5];
}

/**
*Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
*					 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud */

static inline int8_t pset_attitude_target_type_mask_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) {
	
	return (int8_t) src[6];
}


static inline float vset_attitude_target_q_GET(Vset_attitude_target_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vset_attitude_target_q pset_attitude_target_q_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) { //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	
	return (Vset_attitude_target_q) {src + 7, 4};
}


static inline float pset_attitude_target_body_roll_rate_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) {//Body roll rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 23, 4)));
}


static inline float pset_attitude_target_body_pitch_rate_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) {//Body roll rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 27, 4)));
}


static inline float pset_attitude_target_body_yaw_rate_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) {//Body roll rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 31, 4)));
}


static inline float pset_attitude_target_thrust_GET(pset_attitude_target_SET_ATTITUDE_TARGET *src) {//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	
	return (intBitsToFloat(get_bytes(src, 35, 4)));
}


static inline int64_t pfollow_target_timestamp_GET(pfollow_target_FOLLOW_TARGET *src) {//Timestamp in milliseconds since system boot
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void pfollow_target_timestamp_SET(int64_t src, pfollow_target_FOLLOW_TARGET *dst) { //Timestamp in milliseconds since system boot
	
	
	set_bytes((src), 8, dst, 0);
}


static inline int64_t pfollow_target_custom_state_GET(pfollow_target_FOLLOW_TARGET *src) {//button states or switches of a tracker device
	
	return ((int64_t) (get_bytes(src, 8, 8)));
}


static inline void pfollow_target_custom_state_SET(int64_t src, pfollow_target_FOLLOW_TARGET *dst) { //button states or switches of a tracker device
	
	
	set_bytes((src), 8, dst, 8);
}


static inline int8_t pfollow_target_est_capabilities_GET(pfollow_target_FOLLOW_TARGET *src) {//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
	
	return (int8_t) src[16];
}


static inline void pfollow_target_est_capabilities_SET(int8_t src, pfollow_target_FOLLOW_TARGET *dst) { //bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
	
	
	dst[16] = (uint8_t) (src);
}


static inline int32_t pfollow_target_lat_GET(pfollow_target_FOLLOW_TARGET *src) {//Latitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 17, 4)));
}


static inline void pfollow_target_lat_SET(int32_t src, pfollow_target_FOLLOW_TARGET *dst) { //Latitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 17);
}


static inline int32_t pfollow_target_lon_GET(pfollow_target_FOLLOW_TARGET *src) {//Longitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 21, 4)));
}


static inline void pfollow_target_lon_SET(int32_t src, pfollow_target_FOLLOW_TARGET *dst) { //Longitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 21);
}


static inline float pfollow_target_alt_GET(pfollow_target_FOLLOW_TARGET *src) {//AMSL, in meters
	
	return (intBitsToFloat(get_bytes(src, 25, 4)));
}


static inline void pfollow_target_alt_SET(float src, pfollow_target_FOLLOW_TARGET *dst) { //AMSL, in meters
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 25);
}


static inline float vfollow_target_vel_GET(Vfollow_target_vel const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vfollow_target_vel pfollow_target_vel_GET(pfollow_target_FOLLOW_TARGET *src) { //target velocity (0,0,0) for unknown
	
	return (Vfollow_target_vel) {src + 29, 3};
}


static inline void vfollow_target_vel_SET(float src, size_t index, Vfollow_target_vel *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vfollow_target_vel pfollow_target_vel_SET(const float src[], pfollow_target_FOLLOW_TARGET *dst) {//target velocity (0,0,0) for unknown
	
	
	Vfollow_target_vel ret = {dst + 29, 3};
	
	if (src)
		for (size_t i = 0; i < 3; i++)
			vfollow_target_vel_SET(src[i], i, &ret);
	return ret;
}


static inline float vfollow_target_acc_GET(Vfollow_target_acc const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vfollow_target_acc pfollow_target_acc_GET(pfollow_target_FOLLOW_TARGET *src) { //linear target acceleration (0,0,0) for unknown
	
	return (Vfollow_target_acc) {src + 41, 3};
}


static inline void vfollow_target_acc_SET(float src, size_t index, Vfollow_target_acc *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vfollow_target_acc pfollow_target_acc_SET(const float src[], pfollow_target_FOLLOW_TARGET *dst) {//linear target acceleration (0,0,0) for unknown
	
	
	Vfollow_target_acc ret = {dst + 41, 3};
	
	if (src)
		for (size_t i = 0; i < 3; i++)
			vfollow_target_acc_SET(src[i], i, &ret);
	return ret;
}


static inline float vfollow_target_attitude_q_GET(Vfollow_target_attitude_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vfollow_target_attitude_q pfollow_target_attitude_q_GET(pfollow_target_FOLLOW_TARGET *src) { //(1 0 0 0 for unknown)
	
	return (Vfollow_target_attitude_q) {src + 53, 4};
}


static inline void vfollow_target_attitude_q_SET(float src, size_t index, Vfollow_target_attitude_q *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vfollow_target_attitude_q pfollow_target_attitude_q_SET(const float src[], pfollow_target_FOLLOW_TARGET *dst) {//(1 0 0 0 for unknown)
	
	
	Vfollow_target_attitude_q ret = {dst + 53, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vfollow_target_attitude_q_SET(src[i], i, &ret);
	return ret;
}


static inline float vfollow_target_rates_GET(Vfollow_target_rates const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vfollow_target_rates pfollow_target_rates_GET(pfollow_target_FOLLOW_TARGET *src) { //(0 0 0 for unknown)
	
	return (Vfollow_target_rates) {src + 69, 3};
}


static inline void vfollow_target_rates_SET(float src, size_t index, Vfollow_target_rates *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vfollow_target_rates pfollow_target_rates_SET(const float src[], pfollow_target_FOLLOW_TARGET *dst) {//(0 0 0 for unknown)
	
	
	Vfollow_target_rates ret = {dst + 69, 3};
	
	if (src)
		for (size_t i = 0; i < 3; i++)
			vfollow_target_rates_SET(src[i], i, &ret);
	return ret;
}


static inline float vfollow_target_position_cov_GET(Vfollow_target_position_cov const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vfollow_target_position_cov pfollow_target_position_cov_GET(pfollow_target_FOLLOW_TARGET *src) { //eph epv
	
	return (Vfollow_target_position_cov) {src + 81, 3};
}


static inline void vfollow_target_position_cov_SET(float src, size_t index, Vfollow_target_position_cov *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vfollow_target_position_cov pfollow_target_position_cov_SET(const float src[], pfollow_target_FOLLOW_TARGET *dst) {//eph epv
	
	
	Vfollow_target_position_cov ret = {dst + 81, 3};
	
	if (src)
		for (size_t i = 0; i < 3; i++)
			vfollow_target_position_cov_SET(src[i], i, &ret);
	return ret;
}


static inline int64_t phil_state_time_usec_GET(phil_state_HIL_STATE *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline float phil_state_roll_GET(phil_state_HIL_STATE *src) {//Roll angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline float phil_state_pitch_GET(phil_state_HIL_STATE *src) {//Pitch angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float phil_state_yaw_GET(phil_state_HIL_STATE *src) {//Yaw angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline float phil_state_rollspeed_GET(phil_state_HIL_STATE *src) {//Body frame roll / phi angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline float phil_state_pitchspeed_GET(phil_state_HIL_STATE *src) {//Body frame pitch / theta angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline float phil_state_yawspeed_GET(phil_state_HIL_STATE *src) {//Body frame yaw / psi angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline int32_t phil_state_lat_GET(phil_state_HIL_STATE *src) {//Latitude, expressed as * 1E7
	
	return ((int32_t) (get_bytes(src, 32, 4)));
}


static inline int32_t phil_state_lon_GET(phil_state_HIL_STATE *src) {//Longitude, expressed as * 1E7
	
	return ((int32_t) (get_bytes(src, 36, 4)));
}


static inline int32_t phil_state_alt_GET(phil_state_HIL_STATE *src) {//Altitude in meters, expressed as * 1000 (millimeters)
	
	return ((int32_t) (get_bytes(src, 40, 4)));
}


static inline int16_t phil_state_vx_GET(phil_state_HIL_STATE *src) {//Ground X Speed (Latitude), expressed as m/s * 100
	
	return ((int16_t) (get_bytes(src, 44, 2)));
}


static inline int16_t phil_state_vy_GET(phil_state_HIL_STATE *src) {//Ground Y Speed (Longitude), expressed as m/s * 100
	
	return ((int16_t) (get_bytes(src, 46, 2)));
}


static inline int16_t phil_state_vz_GET(phil_state_HIL_STATE *src) {//Ground Z Speed (Altitude), expressed as m/s * 100
	
	return ((int16_t) (get_bytes(src, 48, 2)));
}


static inline int16_t phil_state_xacc_GET(phil_state_HIL_STATE *src) {//X acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 50, 2)));
}


static inline int16_t phil_state_yacc_GET(phil_state_HIL_STATE *src) {//Y acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 52, 2)));
}


static inline int16_t phil_state_zacc_GET(phil_state_HIL_STATE *src) {//Z acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 54, 2)));
}


static inline int32_t phome_position_latitude_GET(phome_position_HOME_POSITION *src) {//Latitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void phome_position_latitude_SET(int32_t src, phome_position_HOME_POSITION *dst) { //Latitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline int32_t phome_position_longitude_GET(phome_position_HOME_POSITION *src) {//Longitude (WGS84, in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 4, 4)));
}


static inline void phome_position_longitude_SET(int32_t src, phome_position_HOME_POSITION *dst) { //Longitude (WGS84, in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 4);
}


static inline int32_t phome_position_altitude_GET(phome_position_HOME_POSITION *src) {//Altitude (AMSL), in meters * 1000 (positive for up)
	
	return ((int32_t) (get_bytes(src->base.bytes, 8, 4)));
}


static inline void phome_position_altitude_SET(int32_t src, phome_position_HOME_POSITION *dst) { //Altitude (AMSL), in meters * 1000 (positive for up)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 8);
}


static inline float phome_position_x_GET(phome_position_HOME_POSITION *src) {//Local X position of this position in the local coordinate frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void phome_position_x_SET(float src, phome_position_HOME_POSITION *dst) { //Local X position of this position in the local coordinate frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}


static inline float phome_position_y_GET(phome_position_HOME_POSITION *src) {//Local Y position of this position in the local coordinate frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline void phome_position_y_SET(float src, phome_position_HOME_POSITION *dst) { //Local Y position of this position in the local coordinate frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 16);
}


static inline float phome_position_z_GET(phome_position_HOME_POSITION *src) {//Local Z position of this position in the local coordinate frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline void phome_position_z_SET(float src, phome_position_HOME_POSITION *dst) { //Local Z position of this position in the local coordinate frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 20);
}

/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*					 and slope of the groun */

static inline float vhome_position_q_GET(Vhome_position_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*					 and slope of the groun */

static inline Vhome_position_q phome_position_q_GET(phome_position_HOME_POSITION *src) {
	
	return (Vhome_position_q) {src->base.bytes + 24, 4};
}

/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*					 and slope of the groun */

static inline void vhome_position_q_SET(float src, size_t index, Vhome_position_q *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*					 and slope of the groun */

static inline Vhome_position_q phome_position_q_SET(const float src[], phome_position_HOME_POSITION *dst) {
	
	
	Vhome_position_q ret = {dst->base.bytes + 24, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vhome_position_q_SET(src[i], i, &ret);
	return ret;
}

/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline float phome_position_approach_x_GET(phome_position_HOME_POSITION *src) {
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 40, 4)));
}

/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline void phome_position_approach_x_SET(float src, phome_position_HOME_POSITION *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 40);
}

/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline float phome_position_approach_y_GET(phome_position_HOME_POSITION *src) {
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 44, 4)));
}

/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline void phome_position_approach_y_SET(float src, phome_position_HOME_POSITION *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 44);
}

/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline float phome_position_approach_z_GET(phome_position_HOME_POSITION *src) {
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 48, 4)));
}

/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline void phome_position_approach_z_SET(float src, phome_position_HOME_POSITION *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 48);
}

static inline bool phome_position_time_usec_GET(phome_position_HOME_POSITION *const src, int64_t *ret) {
	if (src->base.field_bit != 416 && !set_field(src, 416, -1)) return false;
	*ret = ((int64_t) (get_bytes(src->base.bytes, src->BYTE, 8)));
	return true;
}

static inline void phome_position_time_usec_SET(int64_t src, phome_position_HOME_POSITION *const dst) {
	
	if (dst->base.field_bit != 416) set_field(dst, 416, 0);
	
	
	set_bytes((src), 8, dst->base.bytes, dst->BYTE);
}


static inline int16_t pfence_status_breach_count_GET(pfence_status_FENCE_STATUS *src) {//number of fence breaches
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pfence_status_breach_count_SET(int16_t src, pfence_status_FENCE_STATUS *dst) { //number of fence breaches
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int32_t pfence_status_breach_time_GET(pfence_status_FENCE_STATUS *src) {//time of last breach in milliseconds since boot
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline void pfence_status_breach_time_SET(int32_t src, pfence_status_FENCE_STATUS *dst) { //time of last breach in milliseconds since boot
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 2);
}


static inline int8_t pfence_status_breach_status_GET(pfence_status_FENCE_STATUS *src) {//0 if currently inside fence, 1 if outside
	
	return (int8_t) src->base.bytes[6];
}


static inline void pfence_status_breach_status_SET(int8_t src, pfence_status_FENCE_STATUS *dst) { //0 if currently inside fence, 1 if outside
	
	
	dst->base.bytes[6] = (uint8_t) (src);
}

static inline bool pfence_status_breach_type_GET(pfence_status_FENCE_STATUS *const src, e_FENCE_BREACH *ret) {
	if (src->base.field_bit != 56 && !set_field(src, 56, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void pfence_status_breach_type_SET(e_FENCE_BREACH src, pfence_status_FENCE_STATUS *const dst) {
	
	if (dst->base.field_bit != 56) set_field(dst, 56, 0);
	
	
	set_bits(src, 2, dst->base.bytes, dst->BIT);
}


static inline int32_t premote_log_block_status_seqno_GET(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *src) {//log data block sequence number
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void premote_log_block_status_seqno_SET(int32_t src, premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *dst) { //log data block sequence number
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline int8_t premote_log_block_status_target_system_GET(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *src) {//System ID
	
	return (int8_t) src->base.bytes[4];
}


static inline void premote_log_block_status_target_system_SET(int8_t src, premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *dst) { //System ID
	
	
	dst->base.bytes[4] = (uint8_t) (src);
}


static inline int8_t premote_log_block_status_target_component_GET(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *src) {//Component ID
	
	return (int8_t) src->base.bytes[5];
}


static inline void premote_log_block_status_target_component_SET(int8_t src, premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *dst) { //Component ID
	
	
	dst->base.bytes[5] = (uint8_t) (src);
}

static inline bool premote_log_block_status_status_GET(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *const src, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES *ret) {
	if (src->base.field_bit != 48 && !set_field(src, 48, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void premote_log_block_status_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES src, premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *const dst) {
	
	if (dst->base.field_bit != 48) set_field(dst, 48, 0);
	
	
	set_bits(src, 1, dst->base.bytes, dst->BIT);
}

/**
*Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
*					 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
*					 for unknown/not used. In a array element, each unit corresponds to 1cm */

static inline int16_t vobstacle_distance_distances_GET(Vobstacle_distance_distances const *const src, size_t index) { return ((int16_t) (get_bytes(src->bytes, index * 2, 2))); }

/**
*Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
*					 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
*					 for unknown/not used. In a array element, each unit corresponds to 1cm */

static inline Vobstacle_distance_distances pobstacle_distance_distances_GET(pobstacle_distance_OBSTACLE_DISTANCE *src) {
	
	return (Vobstacle_distance_distances) {src->base.bytes + 0, 72};
}

/**
*Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
*					 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
*					 for unknown/not used. In a array element, each unit corresponds to 1cm */

static inline void vobstacle_distance_distances_SET(int16_t src, size_t index, Vobstacle_distance_distances *dst) { set_bytes((uint16_t) (src), 2, dst->bytes, index * 2); }

/**
*Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
*					 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
*					 for unknown/not used. In a array element, each unit corresponds to 1cm */

static inline Vobstacle_distance_distances pobstacle_distance_distances_SET(const int16_t src[], pobstacle_distance_OBSTACLE_DISTANCE *dst) {
	
	
	Vobstacle_distance_distances ret = {dst->base.bytes + 0, 72};
	
	if (src)
		for (size_t i = 0; i < 72; i++)
			vobstacle_distance_distances_SET(src[i], i, &ret);
	return ret;
}


static inline int16_t pobstacle_distance_min_distance_GET(pobstacle_distance_OBSTACLE_DISTANCE *src) {//Minimum distance the sensor can measure in centimeters
	
	return ((int16_t) (get_bytes(src->base.bytes, 144, 2)));
}


static inline void pobstacle_distance_min_distance_SET(int16_t src, pobstacle_distance_OBSTACLE_DISTANCE *dst) { //Minimum distance the sensor can measure in centimeters
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 144);
}


static inline int16_t pobstacle_distance_max_distance_GET(pobstacle_distance_OBSTACLE_DISTANCE *src) {//Maximum distance the sensor can measure in centimeters
	
	return ((int16_t) (get_bytes(src->base.bytes, 146, 2)));
}


static inline void pobstacle_distance_max_distance_SET(int16_t src, pobstacle_distance_OBSTACLE_DISTANCE *dst) { //Maximum distance the sensor can measure in centimeters
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 146);
}


static inline int64_t pobstacle_distance_time_usec_GET(pobstacle_distance_OBSTACLE_DISTANCE *src) {//Timestamp (microseconds since system boot or since UNIX epoch)
	
	return ((int64_t) (get_bytes(src->base.bytes, 148, 8)));
}


static inline void pobstacle_distance_time_usec_SET(int64_t src, pobstacle_distance_OBSTACLE_DISTANCE *dst) { //Timestamp (microseconds since system boot or since UNIX epoch)
	
	
	set_bytes((src), 8, dst->base.bytes, 148);
}


static inline int8_t pobstacle_distance_increment_GET(pobstacle_distance_OBSTACLE_DISTANCE *src) {//Angular width in degrees of each array element.
	
	return (int8_t) src->base.bytes[156];
}


static inline void pobstacle_distance_increment_SET(int8_t src, pobstacle_distance_OBSTACLE_DISTANCE *dst) { //Angular width in degrees of each array element.
	
	
	dst->base.bytes[156] = (uint8_t) (src);
}

static inline bool pobstacle_distance_sensor_type_GET(pobstacle_distance_OBSTACLE_DISTANCE *const src, e_MAV_DISTANCE_SENSOR *ret) {
	if (src->base.field_bit != 1256 && !set_field(src, 1256, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pobstacle_distance_sensor_type_SET(e_MAV_DISTANCE_SENSOR src, pobstacle_distance_OBSTACLE_DISTANCE *const dst) {
	
	if (dst->base.field_bit != 1256) set_field(dst, 1256, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}


static inline int16_t pgps2_raw_eph_GET(pgps2_raw_GPS2_RAW *src) {//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pgps2_raw_eph_SET(int16_t src, pgps2_raw_GPS2_RAW *dst) { //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t pgps2_raw_epv_GET(pgps2_raw_GPS2_RAW *src) {//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pgps2_raw_epv_SET(int16_t src, pgps2_raw_GPS2_RAW *dst) { //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int16_t pgps2_raw_vel_GET(pgps2_raw_GPS2_RAW *src) {//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}


static inline void pgps2_raw_vel_SET(int16_t src, pgps2_raw_GPS2_RAW *dst) { //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 4);
}

/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*					 unknown, set to: UINT16_MA */

static inline int16_t pgps2_raw_cog_GET(pgps2_raw_GPS2_RAW *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 6, 2)));
}

/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*					 unknown, set to: UINT16_MA */

static inline void pgps2_raw_cog_SET(int16_t src, pgps2_raw_GPS2_RAW *dst) {
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 6);
}


static inline int32_t pgps2_raw_dgps_age_GET(pgps2_raw_GPS2_RAW *src) {//Age of DGPS info
	
	return ((int32_t) (get_bytes(src->base.bytes, 8, 4)));
}


static inline void pgps2_raw_dgps_age_SET(int32_t src, pgps2_raw_GPS2_RAW *dst) { //Age of DGPS info
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 8);
}


static inline int64_t pgps2_raw_time_usec_GET(pgps2_raw_GPS2_RAW *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src->base.bytes, 12, 8)));
}


static inline void pgps2_raw_time_usec_SET(int64_t src, pgps2_raw_GPS2_RAW *dst) { //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	
	set_bytes((src), 8, dst->base.bytes, 12);
}


static inline int32_t pgps2_raw_lat_GET(pgps2_raw_GPS2_RAW *src) {//Latitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 20, 4)));
}


static inline void pgps2_raw_lat_SET(int32_t src, pgps2_raw_GPS2_RAW *dst) { //Latitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 20);
}


static inline int32_t pgps2_raw_lon_GET(pgps2_raw_GPS2_RAW *src) {//Longitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 24, 4)));
}


static inline void pgps2_raw_lon_SET(int32_t src, pgps2_raw_GPS2_RAW *dst) { //Longitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 24);
}


static inline int32_t pgps2_raw_alt_GET(pgps2_raw_GPS2_RAW *src) {//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
	
	return ((int32_t) (get_bytes(src->base.bytes, 28, 4)));
}


static inline void pgps2_raw_alt_SET(int32_t src, pgps2_raw_GPS2_RAW *dst) { //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 28);
}


static inline int8_t pgps2_raw_satellites_visible_GET(pgps2_raw_GPS2_RAW *src) {//Number of satellites visible. If unknown, set to 255
	
	return (int8_t) src->base.bytes[32];
}


static inline void pgps2_raw_satellites_visible_SET(int8_t src, pgps2_raw_GPS2_RAW *dst) { //Number of satellites visible. If unknown, set to 255
	
	
	dst->base.bytes[32] = (uint8_t) (src);
}


static inline int8_t pgps2_raw_dgps_numch_GET(pgps2_raw_GPS2_RAW *src) {//Number of DGPS satellites
	
	return (int8_t) src->base.bytes[33];
}


static inline void pgps2_raw_dgps_numch_SET(int8_t src, pgps2_raw_GPS2_RAW *dst) { //Number of DGPS satellites
	
	
	dst->base.bytes[33] = (uint8_t) (src);
}

static inline bool pgps2_raw_fix_type_GET(pgps2_raw_GPS2_RAW *const src, e_GPS_FIX_TYPE *ret) {
	if (src->base.field_bit != 272 && !set_field(src, 272, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pgps2_raw_fix_type_SET(e_GPS_FIX_TYPE src, pgps2_raw_GPS2_RAW *const dst) {
	
	if (dst->base.field_bit != 272) set_field(dst, 272, 0);
	
	
	set_bits(src, 4, dst->base.bytes, dst->BIT);
}


static inline int16_t prequest_data_stream_req_message_rate_GET(prequest_data_stream_REQUEST_DATA_STREAM *src) {//The requested message rate
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int8_t prequest_data_stream_target_system_GET(prequest_data_stream_REQUEST_DATA_STREAM *src) {//The target requested to send the message stream.
	
	return (int8_t) src[2];
}


static inline int8_t prequest_data_stream_target_component_GET(prequest_data_stream_REQUEST_DATA_STREAM *src) {//The target requested to send the message stream.
	
	return (int8_t) src[3];
}


static inline int8_t prequest_data_stream_req_stream_id_GET(prequest_data_stream_REQUEST_DATA_STREAM *src) {//The ID of the requested data stream
	
	return (int8_t) src[4];
}


static inline int8_t prequest_data_stream_start_stop_GET(prequest_data_stream_REQUEST_DATA_STREAM *src) {//1 to start sending, 0 to stop sending.
	
	return (int8_t) src[5];
}


static inline int16_t pmemory_vect_address_GET(pmemory_vect_MEMORY_VECT *src) {//Starting address of the debug variables
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pmemory_vect_address_SET(int16_t src, pmemory_vect_MEMORY_VECT *dst) { //Starting address of the debug variables
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int8_t pmemory_vect_ver_GET(pmemory_vect_MEMORY_VECT *src) {//Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
	
	return (int8_t) src[2];
}


static inline void pmemory_vect_ver_SET(int8_t src, pmemory_vect_MEMORY_VECT *dst) { //Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t pmemory_vect_typE_GET(pmemory_vect_MEMORY_VECT *src) {//Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q1
	
	return (int8_t) src[3];
}


static inline void pmemory_vect_typE_SET(int8_t src, pmemory_vect_MEMORY_VECT *dst) { //Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q1
	
	
	dst[3] = (uint8_t) (src);
}


static inline int8_t vmemory_vect_value_GET(Vmemory_vect_value const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vmemory_vect_value pmemory_vect_value_GET(pmemory_vect_MEMORY_VECT *src) { //Memory contents at specified address
	
	return (Vmemory_vect_value) {src + 4, 32};
}


static inline void vmemory_vect_value_SET(int8_t src, size_t index, Vmemory_vect_value *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vmemory_vect_value pmemory_vect_value_SET(const int8_t src[], pmemory_vect_MEMORY_VECT *dst) {//Memory contents at specified address
	
	
	Vmemory_vect_value ret = {dst + 4, 32};
	
	if (src)
		for (size_t i = 0; i < 32; i++)
			vmemory_vect_value_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t pparam_ext_request_read_target_system_GET(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline void pparam_ext_request_read_target_system_SET(int8_t src, pparam_ext_request_read_PARAM_EXT_REQUEST_READ *dst) { //System ID
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t pparam_ext_request_read_target_component_GET(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline void pparam_ext_request_read_target_component_SET(int8_t src, pparam_ext_request_read_PARAM_EXT_REQUEST_READ *dst) { //Component ID
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}


static inline int16_t pparam_ext_request_read_param_index_GET(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *src) {//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pparam_ext_request_read_param_index_SET(int16_t src, pparam_ext_request_read_PARAM_EXT_REQUEST_READ *dst) { //Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}

/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*					 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*					 ID is stored as strin */

static inline bool pparam_ext_request_read_param_id_GET(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *const src, Vparam_ext_request_read_param_id *ret) {
	
	if (src->base.field_bit != 34 && !set_field(src, 34, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vparam_ext_request_read_param_id pparam_ext_request_read_param_id_SET(const char src[], size_t len, pparam_ext_request_read_PARAM_EXT_REQUEST_READ *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 34, len);
	
	Vparam_ext_request_read_param_id ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int64_t phil_controls_time_usec_GET(phil_controls_HIL_CONTROLS *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src->base.bytes, 0, 8)));
}


static inline float phil_controls_roll_ailerons_GET(phil_controls_HIL_CONTROLS *src) {//Control output -1 .. 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline float phil_controls_pitch_elevator_GET(phil_controls_HIL_CONTROLS *src) {//Control output -1 .. 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline float phil_controls_yaw_rudder_GET(phil_controls_HIL_CONTROLS *src) {//Control output -1 .. 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline float phil_controls_throttle_GET(phil_controls_HIL_CONTROLS *src) {//Throttle 0 .. 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline float phil_controls_aux1_GET(phil_controls_HIL_CONTROLS *src) {//Aux 1, -1 .. 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline float phil_controls_aux2_GET(phil_controls_HIL_CONTROLS *src) {//Aux 2, -1 .. 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline float phil_controls_aux3_GET(phil_controls_HIL_CONTROLS *src) {//Aux 3, -1 .. 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline float phil_controls_aux4_GET(phil_controls_HIL_CONTROLS *src) {//Aux 4, -1 .. 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 36, 4)));
}


static inline int8_t phil_controls_nav_mode_GET(phil_controls_HIL_CONTROLS *src) {//Navigation mode (MAV_NAV_MODE)
	
	return (int8_t) src->base.bytes[40];
}

static inline bool phil_controls_mode_GET(phil_controls_HIL_CONTROLS *const src, e_MAV_MODE *ret) {
	if (src->base.field_bit != 328 && !set_field(src, 328, -1)) return false;
	*ret = _en_mav_mode(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

/**
*Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
*					 reset of attitude/position/velocities/etc was performed in sim */

static inline int32_t phil_sensor_fields_updated_GET(phil_sensor_HIL_SENSOR *src) {
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}

/**
*Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
*					 reset of attitude/position/velocities/etc was performed in sim */

static inline void phil_sensor_fields_updated_SET(int32_t src, phil_sensor_HIL_SENSOR *dst) {
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int64_t phil_sensor_time_usec_GET(phil_sensor_HIL_SENSOR *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 4, 8)));
}


static inline void phil_sensor_time_usec_SET(int64_t src, phil_sensor_HIL_SENSOR *dst) { //Timestamp (microseconds, synced to UNIX time or since system boot)
	
	
	set_bytes((src), 8, dst, 4);
}


static inline float phil_sensor_xacc_GET(phil_sensor_HIL_SENSOR *src) {//X acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void phil_sensor_xacc_SET(float src, phil_sensor_HIL_SENSOR *dst) { //X acceleration (m/s^2)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float phil_sensor_yacc_GET(phil_sensor_HIL_SENSOR *src) {//Y acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void phil_sensor_yacc_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Y acceleration (m/s^2)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float phil_sensor_zacc_GET(phil_sensor_HIL_SENSOR *src) {//Z acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void phil_sensor_zacc_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Z acceleration (m/s^2)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float phil_sensor_xgyro_GET(phil_sensor_HIL_SENSOR *src) {//Angular speed around X axis in body frame (rad / sec)
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void phil_sensor_xgyro_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Angular speed around X axis in body frame (rad / sec)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float phil_sensor_ygyro_GET(phil_sensor_HIL_SENSOR *src) {//Angular speed around Y axis in body frame (rad / sec)
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void phil_sensor_ygyro_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Angular speed around Y axis in body frame (rad / sec)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float phil_sensor_zgyro_GET(phil_sensor_HIL_SENSOR *src) {//Angular speed around Z axis in body frame (rad / sec)
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void phil_sensor_zgyro_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Angular speed around Z axis in body frame (rad / sec)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}


static inline float phil_sensor_xmag_GET(phil_sensor_HIL_SENSOR *src) {//X Magnetic field (Gauss)
	
	return (intBitsToFloat(get_bytes(src, 36, 4)));
}


static inline void phil_sensor_xmag_SET(float src, phil_sensor_HIL_SENSOR *dst) { //X Magnetic field (Gauss)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 36);
}


static inline float phil_sensor_ymag_GET(phil_sensor_HIL_SENSOR *src) {//Y Magnetic field (Gauss)
	
	return (intBitsToFloat(get_bytes(src, 40, 4)));
}


static inline void phil_sensor_ymag_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Y Magnetic field (Gauss)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 40);
}


static inline float phil_sensor_zmag_GET(phil_sensor_HIL_SENSOR *src) {//Z Magnetic field (Gauss)
	
	return (intBitsToFloat(get_bytes(src, 44, 4)));
}


static inline void phil_sensor_zmag_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Z Magnetic field (Gauss)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 44);
}


static inline float phil_sensor_abs_pressure_GET(phil_sensor_HIL_SENSOR *src) {//Absolute pressure in millibar
	
	return (intBitsToFloat(get_bytes(src, 48, 4)));
}


static inline void phil_sensor_abs_pressure_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Absolute pressure in millibar
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 48);
}


static inline float phil_sensor_diff_pressure_GET(phil_sensor_HIL_SENSOR *src) {//Differential pressure (airspeed) in millibar
	
	return (intBitsToFloat(get_bytes(src, 52, 4)));
}


static inline void phil_sensor_diff_pressure_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Differential pressure (airspeed) in millibar
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 52);
}


static inline float phil_sensor_pressure_alt_GET(phil_sensor_HIL_SENSOR *src) {//Altitude calculated from pressure
	
	return (intBitsToFloat(get_bytes(src, 56, 4)));
}


static inline void phil_sensor_pressure_alt_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Altitude calculated from pressure
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 56);
}


static inline float phil_sensor_temperature_GET(phil_sensor_HIL_SENSOR *src) {//Temperature in degrees celsius
	
	return (intBitsToFloat(get_bytes(src, 60, 4)));
}


static inline void phil_sensor_temperature_SET(float src, phil_sensor_HIL_SENSOR *dst) { //Temperature in degrees celsius
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 60);
}


static inline int64_t psetup_signing_initial_timestamp_GET(psetup_signing_SETUP_SIGNING *src) {//initial timestamp
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void psetup_signing_initial_timestamp_SET(int64_t src, psetup_signing_SETUP_SIGNING *dst) { //initial timestamp
	
	
	set_bytes((src), 8, dst, 0);
}


static inline int8_t psetup_signing_target_system_GET(psetup_signing_SETUP_SIGNING *src) {//system id of the target
	
	return (int8_t) src[8];
}


static inline void psetup_signing_target_system_SET(int8_t src, psetup_signing_SETUP_SIGNING *dst) { //system id of the target
	
	
	dst[8] = (uint8_t) (src);
}


static inline int8_t psetup_signing_target_component_GET(psetup_signing_SETUP_SIGNING *src) {//component ID of the target
	
	return (int8_t) src[9];
}


static inline void psetup_signing_target_component_SET(int8_t src, psetup_signing_SETUP_SIGNING *dst) { //component ID of the target
	
	
	dst[9] = (uint8_t) (src);
}


static inline int8_t vsetup_signing_secret_key_GET(Vsetup_signing_secret_key const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vsetup_signing_secret_key psetup_signing_secret_key_GET(psetup_signing_SETUP_SIGNING *src) { //signing key
	
	return (Vsetup_signing_secret_key) {src + 10, 32};
}


static inline void vsetup_signing_secret_key_SET(int8_t src, size_t index, Vsetup_signing_secret_key *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vsetup_signing_secret_key psetup_signing_secret_key_SET(const int8_t src[], psetup_signing_SETUP_SIGNING *dst) {//signing key
	
	
	Vsetup_signing_secret_key ret = {dst + 10, 32};
	
	if (src)
		for (size_t i = 0; i < 32; i++)
			vsetup_signing_secret_key_SET(src[i], i, &ret);
	return ret;
}


static inline int16_t pgps_rtk_wn_GET(pgps_rtk_GPS_RTK *src) {//GPS Week Number of last baseline
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pgps_rtk_wn_SET(int16_t src, pgps_rtk_GPS_RTK *dst) { //GPS Week Number of last baseline
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int32_t pgps_rtk_time_last_baseline_ms_GET(pgps_rtk_GPS_RTK *src) {//Time since boot of last baseline message received in ms.
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline void pgps_rtk_time_last_baseline_ms_SET(int32_t src, pgps_rtk_GPS_RTK *dst) { //Time since boot of last baseline message received in ms.
	
	
	set_bytes((uint32_t) (src), 4, dst, 2);
}


static inline int32_t pgps_rtk_tow_GET(pgps_rtk_GPS_RTK *src) {//GPS Time of Week of last baseline
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void pgps_rtk_tow_SET(int32_t src, pgps_rtk_GPS_RTK *dst) { //GPS Time of Week of last baseline
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int32_t pgps_rtk_accuracy_GET(pgps_rtk_GPS_RTK *src) {//Current estimate of baseline accuracy.
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}


static inline void pgps_rtk_accuracy_SET(int32_t src, pgps_rtk_GPS_RTK *dst) { //Current estimate of baseline accuracy.
	
	
	set_bytes((uint32_t) (src), 4, dst, 10);
}


static inline int8_t pgps_rtk_rtk_receiver_id_GET(pgps_rtk_GPS_RTK *src) {//Identification of connected RTK receiver.
	
	return (int8_t) src[14];
}


static inline void pgps_rtk_rtk_receiver_id_SET(int8_t src, pgps_rtk_GPS_RTK *dst) { //Identification of connected RTK receiver.
	
	
	dst[14] = (uint8_t) (src);
}


static inline int8_t pgps_rtk_rtk_health_GET(pgps_rtk_GPS_RTK *src) {//GPS-specific health report for RTK data.
	
	return (int8_t) src[15];
}


static inline void pgps_rtk_rtk_health_SET(int8_t src, pgps_rtk_GPS_RTK *dst) { //GPS-specific health report for RTK data.
	
	
	dst[15] = (uint8_t) (src);
}


static inline int8_t pgps_rtk_rtk_rate_GET(pgps_rtk_GPS_RTK *src) {//Rate of baseline messages being received by GPS, in HZ
	
	return (int8_t) src[16];
}


static inline void pgps_rtk_rtk_rate_SET(int8_t src, pgps_rtk_GPS_RTK *dst) { //Rate of baseline messages being received by GPS, in HZ
	
	
	dst[16] = (uint8_t) (src);
}


static inline int8_t pgps_rtk_nsats_GET(pgps_rtk_GPS_RTK *src) {//Current number of sats used for RTK calculation.
	
	return (int8_t) src[17];
}


static inline void pgps_rtk_nsats_SET(int8_t src, pgps_rtk_GPS_RTK *dst) { //Current number of sats used for RTK calculation.
	
	
	dst[17] = (uint8_t) (src);
}


static inline int8_t pgps_rtk_baseline_coords_type_GET(pgps_rtk_GPS_RTK *src) {//Coordinate system of baseline. 0 == ECEF, 1 == NED
	
	return (int8_t) src[18];
}


static inline void pgps_rtk_baseline_coords_type_SET(int8_t src, pgps_rtk_GPS_RTK *dst) { //Coordinate system of baseline. 0 == ECEF, 1 == NED
	
	
	dst[18] = (uint8_t) (src);
}


static inline int32_t pgps_rtk_baseline_a_mm_GET(pgps_rtk_GPS_RTK *src) {//Current baseline in ECEF x or NED north component in mm.
	
	return ((int32_t) (get_bytes(src, 19, 4)));
}


static inline void pgps_rtk_baseline_a_mm_SET(int32_t src, pgps_rtk_GPS_RTK *dst) { //Current baseline in ECEF x or NED north component in mm.
	
	
	set_bytes((uint32_t) (src), 4, dst, 19);
}


static inline int32_t pgps_rtk_baseline_b_mm_GET(pgps_rtk_GPS_RTK *src) {//Current baseline in ECEF y or NED east component in mm.
	
	return ((int32_t) (get_bytes(src, 23, 4)));
}


static inline void pgps_rtk_baseline_b_mm_SET(int32_t src, pgps_rtk_GPS_RTK *dst) { //Current baseline in ECEF y or NED east component in mm.
	
	
	set_bytes((uint32_t) (src), 4, dst, 23);
}


static inline int32_t pgps_rtk_baseline_c_mm_GET(pgps_rtk_GPS_RTK *src) {//Current baseline in ECEF z or NED down component in mm.
	
	return ((int32_t) (get_bytes(src, 27, 4)));
}


static inline void pgps_rtk_baseline_c_mm_SET(int32_t src, pgps_rtk_GPS_RTK *dst) { //Current baseline in ECEF z or NED down component in mm.
	
	
	set_bytes((uint32_t) (src), 4, dst, 27);
}


static inline int32_t pgps_rtk_iar_num_hypotheses_GET(pgps_rtk_GPS_RTK *src) {//Current number of integer ambiguity hypotheses.
	
	return ((int32_t) (get_bytes(src, 31, 4)));
}


static inline void pgps_rtk_iar_num_hypotheses_SET(int32_t src, pgps_rtk_GPS_RTK *dst) { //Current number of integer ambiguity hypotheses.
	
	
	set_bytes((uint32_t) (src), 4, dst, 31);
}


static inline int8_t pparam_request_list_target_system_GET(pparam_request_list_PARAM_REQUEST_LIST *src) {//System ID
	
	return (int8_t) src[0];
}


static inline int8_t pparam_request_list_target_component_GET(pparam_request_list_PARAM_REQUEST_LIST *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline int16_t puavionix_adsb_out_cfg_stallSpeed_GET(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *src) {//Aircraft stall speed in cm/s
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void puavionix_adsb_out_cfg_stallSpeed_SET(int16_t src, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *dst) { //Aircraft stall speed in cm/s
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int32_t puavionix_adsb_out_cfg_ICAO_GET(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *src) {//Vehicle address (24 bit)
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline void puavionix_adsb_out_cfg_ICAO_SET(int32_t src, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *dst) { //Vehicle address (24 bit)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 2);
}

static inline bool puavionix_adsb_out_cfg_callsign_GET(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const src, Vuavionix_adsb_out_cfg_callsign *ret) {//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
	
	if (src->base.field_bit != 51 && !set_field(src, 51, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vuavionix_adsb_out_cfg_callsign puavionix_adsb_out_cfg_callsign_SET(const char src[], size_t len, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 51, len);
	
	Vuavionix_adsb_out_cfg_callsign ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool puavionix_adsb_out_cfg_emitterType_GET(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const src, e_ADSB_EMITTER_TYPE *ret) {
	if (src->base.field_bit != 52 && !set_field(src, 52, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline void puavionix_adsb_out_cfg_emitterType_SET(e_ADSB_EMITTER_TYPE src, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const dst) {
	
	if (dst->base.field_bit != 52) set_field(dst, 52, 0);
	
	
	set_bits(src, 5, dst->base.bytes, dst->BIT);
}

static inline bool puavionix_adsb_out_cfg_aircraftSize_GET(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const src, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE *ret) {
	if (src->base.field_bit != 53 && !set_field(src, 53, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void puavionix_adsb_out_cfg_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE src, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const dst) {
	
	if (dst->base.field_bit != 53) set_field(dst, 53, 0);
	
	
	set_bits(src, 4, dst->base.bytes, dst->BIT);
}

static inline bool puavionix_adsb_out_cfg_gpsOffsetLat_GET(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const src, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT *ret) {
	if (src->base.field_bit != 54 && !set_field(src, 54, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void puavionix_adsb_out_cfg_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT src, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const dst) {
	
	if (dst->base.field_bit != 54) set_field(dst, 54, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool puavionix_adsb_out_cfg_gpsOffsetLon_GET(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const src, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON *ret) {
	if (src->base.field_bit != 55 && !set_field(src, 55, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void puavionix_adsb_out_cfg_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON src, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const dst) {
	
	if (dst->base.field_bit != 55) set_field(dst, 55, 0);
	
	
	set_bits(src, 1, dst->base.bytes, dst->BIT);
}

static inline bool puavionix_adsb_out_cfg_rfSelect_GET(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const src, e_UAVIONIX_ADSB_OUT_RF_SELECT *ret) {
	if (src->base.field_bit != 56 && !set_field(src, 56, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void puavionix_adsb_out_cfg_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT src, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const dst) {
	
	if (dst->base.field_bit != 56) set_field(dst, 56, 0);
	
	
	set_bits(src, 2, dst->base.bytes, dst->BIT);
}


static inline int64_t planding_target_time_usec_GET(planding_target_LANDING_TARGET *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src->base.bytes, 0, 8)));
}


static inline void planding_target_time_usec_SET(int64_t src, planding_target_LANDING_TARGET *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst->base.bytes, 0);
}


static inline int8_t planding_target_target_num_GET(planding_target_LANDING_TARGET *src) {//The ID of the target if multiple targets are present
	
	return (int8_t) src->base.bytes[8];
}


static inline void planding_target_target_num_SET(int8_t src, planding_target_LANDING_TARGET *dst) { //The ID of the target if multiple targets are present
	
	
	dst->base.bytes[8] = (uint8_t) (src);
}


static inline float planding_target_angle_x_GET(planding_target_LANDING_TARGET *src) {//X-axis angular offset (in radians) of the target from the center of the image
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 9, 4)));
}


static inline void planding_target_angle_x_SET(float src, planding_target_LANDING_TARGET *dst) { //X-axis angular offset (in radians) of the target from the center of the image
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 9);
}


static inline float planding_target_angle_y_GET(planding_target_LANDING_TARGET *src) {//Y-axis angular offset (in radians) of the target from the center of the image
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 13, 4)));
}


static inline void planding_target_angle_y_SET(float src, planding_target_LANDING_TARGET *dst) { //Y-axis angular offset (in radians) of the target from the center of the image
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 13);
}


static inline float planding_target_distance_GET(planding_target_LANDING_TARGET *src) {//Distance to the target from the vehicle in meters
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 17, 4)));
}


static inline void planding_target_distance_SET(float src, planding_target_LANDING_TARGET *dst) { //Distance to the target from the vehicle in meters
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 17);
}


static inline float planding_target_size_x_GET(planding_target_LANDING_TARGET *src) {//Size in radians of target along x-axis
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 21, 4)));
}


static inline void planding_target_size_x_SET(float src, planding_target_LANDING_TARGET *dst) { //Size in radians of target along x-axis
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 21);
}


static inline float planding_target_size_y_GET(planding_target_LANDING_TARGET *src) {//Size in radians of target along y-axis
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 25, 4)));
}


static inline void planding_target_size_y_SET(float src, planding_target_LANDING_TARGET *dst) { //Size in radians of target along y-axis
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 25);
}

static inline bool planding_target_frame_GET(planding_target_LANDING_TARGET *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 235 && !set_field(src, 235, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void planding_target_frame_SET(e_MAV_FRAME src, planding_target_LANDING_TARGET *const dst) {
	
	if (dst->base.field_bit != 235) set_field(dst, 235, 0);
	
	
	set_bits(src, 4, dst->base.bytes, dst->BIT);
}

static inline bool planding_target_x_GET(planding_target_LANDING_TARGET *const src, float *ret) {
	if (src->base.field_bit != 236 && !set_field(src, 236, -1)) return false;
	*ret = (intBitsToFloat(get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline void planding_target_x_SET(float src, planding_target_LANDING_TARGET *const dst) {
	
	if (dst->base.field_bit != 236) set_field(dst, 236, 0);
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, dst->BYTE);
}

static inline bool planding_target_y_GET(planding_target_LANDING_TARGET *const src, float *ret) {
	if (src->base.field_bit != 237 && !set_field(src, 237, -1)) return false;
	*ret = (intBitsToFloat(get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline void planding_target_y_SET(float src, planding_target_LANDING_TARGET *const dst) {
	
	if (dst->base.field_bit != 237) set_field(dst, 237, 0);
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, dst->BYTE);
}

static inline bool planding_target_z_GET(planding_target_LANDING_TARGET *const src, float *ret) {
	if (src->base.field_bit != 238 && !set_field(src, 238, -1)) return false;
	*ret = (intBitsToFloat(get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline void planding_target_z_SET(float src, planding_target_LANDING_TARGET *const dst) {
	
	if (dst->base.field_bit != 238) set_field(dst, 238, 0);
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, dst->BYTE);
}

/**
															 * brief Getting pointer to the field
															 * param planding_target pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qlanding_target_q *planding_target_q(planding_target_LANDING_TARGET *planding_target) {
	if (planding_target->base.field_bit != 239 && !set_field(planding_target, 239, -1)) return NULL;
	return planding_target;
}

static inline float vlanding_target_q_GET(Vlanding_target_q const *const src) { return (intBitsToFloat(get_bytes(src->bytes, 0, 4))); }

static inline bool qlanding_target_q_GET(Qlanding_target_q *const src, Vlanding_target_q *ret, size_t d0) {
	
	if (!set_item(src, d0, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vlanding_target_q_SET(float src, Vlanding_target_q const *const dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, 0); }

static inline Vlanding_target_q planding_target_q_SET(float src, planding_target_LANDING_TARGET *const dst, size_t d0) {
	
	if (dst->base.field_bit != 239) set_field(dst, 239, 0);
	set_item(dst, d0, 0);
	Vlanding_target_q ret = {dst->base.bytes + dst->BYTE};
	
	vlanding_target_q_SET(src, &ret);
	return ret;
}

static inline bool planding_target_typE_GET(planding_target_LANDING_TARGET *const src, e_LANDING_TARGET_TYPE *ret) {
	if (src->base.field_bit != 240 && !set_field(src, 240, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void planding_target_typE_SET(e_LANDING_TARGET_TYPE src, planding_target_LANDING_TARGET *const dst) {
	
	if (dst->base.field_bit != 240) set_field(dst, 240, 0);
	
	
	set_bits(src, 2, dst->base.bytes, dst->BIT);
}

static inline bool planding_target_position_valid_GET(planding_target_LANDING_TARGET *const src, int8_t *ret) {
	if (src->base.field_bit != 241 && !set_field(src, 241, -1)) return false;
	*ret = (int8_t) src->base.bytes[src->BYTE];
	return true;
}

static inline void planding_target_position_valid_SET(int8_t src, planding_target_LANDING_TARGET *const dst) {
	
	if (dst->base.field_bit != 241) set_field(dst, 241, 0);
	
	
	dst->base.bytes[dst->BYTE] = (uint8_t) (src);
}


static inline int64_t pset_actuator_control_target_time_usec_GET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void pset_actuator_control_target_time_usec_SET(int64_t src, pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst, 0);
}

/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*					 this field to difference between instances */

static inline int8_t pset_actuator_control_target_group_mlx_GET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *src) {
	
	return (int8_t) src[8];
}

/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*					 this field to difference between instances */

static inline void pset_actuator_control_target_group_mlx_SET(int8_t src, pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *dst) {
	
	
	dst[8] = (uint8_t) (src);
}


static inline int8_t pset_actuator_control_target_target_system_GET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *src) {//System ID
	
	return (int8_t) src[9];
}


static inline void pset_actuator_control_target_target_system_SET(int8_t src, pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *dst) { //System ID
	
	
	dst[9] = (uint8_t) (src);
}


static inline int8_t pset_actuator_control_target_target_component_GET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *src) {//Component ID
	
	return (int8_t) src[10];
}


static inline void pset_actuator_control_target_target_component_SET(int8_t src, pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *dst) { //Component ID
	
	
	dst[10] = (uint8_t) (src);
}

/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*					 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*					 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*					 mixer to repurpose them as generic outputs */

static inline float vset_actuator_control_target_controls_GET(Vset_actuator_control_target_controls const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*					 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*					 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*					 mixer to repurpose them as generic outputs */

static inline Vset_actuator_control_target_controls pset_actuator_control_target_controls_GET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *src) {
	
	return (Vset_actuator_control_target_controls) {src + 11, 8};
}

/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*					 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*					 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*					 mixer to repurpose them as generic outputs */

static inline void vset_actuator_control_target_controls_SET(float src, size_t index, Vset_actuator_control_target_controls *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*					 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*					 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*					 mixer to repurpose them as generic outputs */

static inline Vset_actuator_control_target_controls pset_actuator_control_target_controls_SET(const float src[], pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *dst) {
	
	
	Vset_actuator_control_target_controls ret = {dst + 11, 8};
	
	if (src)
		for (size_t i = 0; i < 8; i++)
			vset_actuator_control_target_controls_SET(src[i], i, &ret);
	return ret;
}


static inline int64_t pcontrol_system_state_time_usec_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void pcontrol_system_state_time_usec_SET(int64_t src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst, 0);
}


static inline float pcontrol_system_state_x_acc_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//X acceleration in body frame
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pcontrol_system_state_x_acc_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //X acceleration in body frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pcontrol_system_state_y_acc_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Y acceleration in body frame
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pcontrol_system_state_y_acc_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Y acceleration in body frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float pcontrol_system_state_z_acc_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Z acceleration in body frame
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void pcontrol_system_state_z_acc_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Z acceleration in body frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float pcontrol_system_state_x_vel_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//X velocity in body frame
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void pcontrol_system_state_x_vel_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //X velocity in body frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float pcontrol_system_state_y_vel_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Y velocity in body frame
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void pcontrol_system_state_y_vel_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Y velocity in body frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float pcontrol_system_state_z_vel_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Z velocity in body frame
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void pcontrol_system_state_z_vel_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Z velocity in body frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float pcontrol_system_state_x_pos_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//X position in local frame
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void pcontrol_system_state_x_pos_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //X position in local frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}


static inline float pcontrol_system_state_y_pos_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Y position in local frame
	
	return (intBitsToFloat(get_bytes(src, 36, 4)));
}


static inline void pcontrol_system_state_y_pos_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Y position in local frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 36);
}


static inline float pcontrol_system_state_z_pos_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Z position in local frame
	
	return (intBitsToFloat(get_bytes(src, 40, 4)));
}


static inline void pcontrol_system_state_z_pos_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Z position in local frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 40);
}


static inline float pcontrol_system_state_airspeed_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Airspeed, set to -1 if unknown
	
	return (intBitsToFloat(get_bytes(src, 44, 4)));
}


static inline void pcontrol_system_state_airspeed_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Airspeed, set to -1 if unknown
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 44);
}


static inline float vcontrol_system_state_vel_variance_GET(Vcontrol_system_state_vel_variance const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vcontrol_system_state_vel_variance pcontrol_system_state_vel_variance_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) { //Variance of body velocity estimate
	
	return (Vcontrol_system_state_vel_variance) {src + 48, 3};
}


static inline void vcontrol_system_state_vel_variance_SET(float src, size_t index, Vcontrol_system_state_vel_variance *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vcontrol_system_state_vel_variance pcontrol_system_state_vel_variance_SET(const float src[], pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) {//Variance of body velocity estimate
	
	
	Vcontrol_system_state_vel_variance ret = {dst + 48, 3};
	
	if (src)
		for (size_t i = 0; i < 3; i++)
			vcontrol_system_state_vel_variance_SET(src[i], i, &ret);
	return ret;
}


static inline float vcontrol_system_state_pos_variance_GET(Vcontrol_system_state_pos_variance const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vcontrol_system_state_pos_variance pcontrol_system_state_pos_variance_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) { //Variance in local position
	
	return (Vcontrol_system_state_pos_variance) {src + 60, 3};
}


static inline void vcontrol_system_state_pos_variance_SET(float src, size_t index, Vcontrol_system_state_pos_variance *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vcontrol_system_state_pos_variance pcontrol_system_state_pos_variance_SET(const float src[], pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) {//Variance in local position
	
	
	Vcontrol_system_state_pos_variance ret = {dst + 60, 3};
	
	if (src)
		for (size_t i = 0; i < 3; i++)
			vcontrol_system_state_pos_variance_SET(src[i], i, &ret);
	return ret;
}


static inline float vcontrol_system_state_q_GET(Vcontrol_system_state_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vcontrol_system_state_q pcontrol_system_state_q_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) { //The attitude, represented as Quaternion
	
	return (Vcontrol_system_state_q) {src + 72, 4};
}


static inline void vcontrol_system_state_q_SET(float src, size_t index, Vcontrol_system_state_q *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vcontrol_system_state_q pcontrol_system_state_q_SET(const float src[], pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) {//The attitude, represented as Quaternion
	
	
	Vcontrol_system_state_q ret = {dst + 72, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vcontrol_system_state_q_SET(src[i], i, &ret);
	return ret;
}


static inline float pcontrol_system_state_roll_rate_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Angular rate in roll axis
	
	return (intBitsToFloat(get_bytes(src, 88, 4)));
}


static inline void pcontrol_system_state_roll_rate_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Angular rate in roll axis
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 88);
}


static inline float pcontrol_system_state_pitch_rate_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Angular rate in pitch axis
	
	return (intBitsToFloat(get_bytes(src, 92, 4)));
}


static inline void pcontrol_system_state_pitch_rate_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Angular rate in pitch axis
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 92);
}


static inline float pcontrol_system_state_yaw_rate_GET(pcontrol_system_state_CONTROL_SYSTEM_STATE *src) {//Angular rate in yaw axis
	
	return (intBitsToFloat(get_bytes(src, 96, 4)));
}


static inline void pcontrol_system_state_yaw_rate_SET(float src, pcontrol_system_state_CONTROL_SYSTEM_STATE *dst) { //Angular rate in yaw axis
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 96);
}

/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*					 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*					 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*					 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*					 bit 11: yaw, bit 12: yaw rat */

static inline int16_t pset_position_target_global_int_type_mask_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}

/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*					 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*					 processing latency */

static inline int32_t pset_position_target_global_int_time_boot_ms_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline int8_t pset_position_target_global_int_target_system_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//System ID
	
	return (int8_t) src->base.bytes[6];
}


static inline int8_t pset_position_target_global_int_target_component_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//Component ID
	
	return (int8_t) src->base.bytes[7];
}


static inline int32_t pset_position_target_global_int_lat_int_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//X Position in WGS84 frame in 1e7 * meters
	
	return ((int32_t) (get_bytes(src->base.bytes, 8, 4)));
}


static inline int32_t pset_position_target_global_int_lon_int_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//Y Position in WGS84 frame in 1e7 * meters
	
	return ((int32_t) (get_bytes(src->base.bytes, 12, 4)));
}


static inline float pset_position_target_global_int_alt_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline float pset_position_target_global_int_vx_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//X velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline float pset_position_target_global_int_vy_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//Y velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline float pset_position_target_global_int_vz_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//Z velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline float pset_position_target_global_int_afx_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline float pset_position_target_global_int_afy_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 36, 4)));
}


static inline float pset_position_target_global_int_afz_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 40, 4)));
}


static inline float pset_position_target_global_int_yaw_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//yaw setpoint in rad
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 44, 4)));
}


static inline float pset_position_target_global_int_yaw_rate_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *src) {//yaw rate setpoint in rad/s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 48, 4)));
}

static inline bool pset_position_target_global_int_coordinate_frame_GET(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 416 && !set_field(src, 416, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}


static inline int8_t pdata32_typE_GET(pdata32_DATA32 *src) {//data type
	
	return (int8_t) src[0];
}


static inline void pdata32_typE_SET(int8_t src, pdata32_DATA32 *dst) { //data type
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pdata32_len_GET(pdata32_DATA32 *src) {//data length
	
	return (int8_t) src[1];
}


static inline void pdata32_len_SET(int8_t src, pdata32_DATA32 *dst) { //data length
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t vdata32_daTa_GET(Vdata32_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vdata32_daTa pdata32_daTa_GET(pdata32_DATA32 *src) { //raw data
	
	return (Vdata32_daTa) {src + 2, 32};
}


static inline void vdata32_daTa_SET(int8_t src, size_t index, Vdata32_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vdata32_daTa pdata32_daTa_SET(const int8_t src[], pdata32_DATA32 *dst) {//raw data
	
	
	Vdata32_daTa ret = {dst + 2, 32};
	
	if (src)
		for (size_t i = 0; i < 32; i++)
			vdata32_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int32_t pping33_TTTT_GET(pping33_PING33 *src, size_t d0, size_t d1, size_t d2) {
	
	return ((int32_t) (get_bytes(src->base.bytes, 0 + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)));
}


static inline void pping33_TTTT_SET(int32_t src, pping33_PING33 *dst, size_t d0, size_t d1, size_t d2) {
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0 + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
}


static inline int64_t pping33_field_GET(pping33_PING33 *src) {
	
	return ((int64_t) (get_bytes(src->base.bytes, 72, 8)));
}


static inline void pping33_field_SET(int64_t src, pping33_PING33 *dst) {
	
	
	set_bytes((src), 8, dst->base.bytes, 72);
}


static inline int8_t pping33_bit_field_GET(pping33_PING33 *src) {
	
	return 4 + (int8_t) src->base.bytes[80];
}


static inline void pping33_bit_field_SET(int8_t src, pping33_PING33 *dst) {
	
	/*use  pping33_bit_field_MIN and pping33_bit_field_MAX and pping33_bit_field_OK(VAL) value range validation facilities */
	dst->base.bytes[80] = (uint8_t) (-4 + src);
}


static inline int32_t pping33_field6_GET(pping33_PING33 *src, size_t d0, size_t d1, size_t d2) {
	
	return ((int32_t) (get_bytes(src->base.bytes, 81 + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)));
}


static inline void pping33_field6_SET(int32_t src, pping33_PING33 *dst, size_t d0, size_t d1, size_t d2) {
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 81 + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
}


static inline bool pping33_testBOOL2_GET(pping33_PING33 *src) {
	
	return (src->base.bytes[1224 >> 3] & 1 << (1224 & 7)) != 0;
}


static inline void pping33_testBOOL2_SET(bool src, pping33_PING33 *dst) {
	
	
	if (src) dst->base.bytes[1224 >> 3] |= 1 << (1224 & 7);
	else dst->base.bytes[1224 >> 3] &= ~(1 << (1224 & 7));
}


static inline bool pping33_testBOOL3_GET(pping33_PING33 *src) {
	
	return (src->base.bytes[1225 >> 3] & 1 << (1225 & 7)) != 0;
}


static inline void pping33_testBOOL3_SET(bool src, pping33_PING33 *dst) {
	
	
	if (src) dst->base.bytes[1225 >> 3] |= 1 << (1225 & 7);
	else dst->base.bytes[1225 >> 3] &= ~(1 << (1225 & 7));
}

static inline bool pping33_testBOOL_GET(pping33_PING33 *const src, bool *ret) {
	if (src->base.field_bit != 1234 && !set_field(src, 1234, -1)) return false;
	*ret = (src->base.bytes[src->BIT >> 3] & 1 << (src->BIT & 7)) != 0;
	return true;
}

static inline void pping33_testBOOL_SET(bool src, pping33_PING33 *const dst) {
	
	if (dst->base.field_bit != 1234) set_field(dst, 1234, 0);
	
	
	if (src) dst->base.bytes[dst->BIT >> 3] |= 1 << (dst->BIT & 7);
	else dst->base.bytes[dst->BIT >> 3] &= ~(1 << (dst->BIT & 7));
}

static inline bool pping33_seq_GET(pping33_PING33 *const src, int64_t *ret) {
	if (src->base.field_bit != 1235 && !set_field(src, 1235, -1)) return false;
	*ret = (-14 + (int64_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline void pping33_seq_SET(int64_t src, pping33_PING33 *const dst) {
	
	if (dst->base.field_bit != 1235) set_field(dst, 1235, 0);
	
	
	set_bytes((14 + src), 4, dst->base.bytes, dst->BYTE);
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_field1 *pping33_field1(pping33_PING33 *pping33, size_t *d0) {
	if (pping33->base.field_bit != 1236 && !set_field(pping33, 1236, -1))
		if (d0 && *d0) set_field(pping33, 1236, 0, *d0);
		else return NULL;
	
	*d0 = pping33->D[0];
	
	return pping33;
}

static inline int32_t qping33_field1_GET(Qping33_field1 *const src, size_t d0, size_t d1, size_t d2) {
	return ((int32_t) (get_bytes(src->base.bytes, src->BYTE + (d0 + d1 * src->var_dims[0] + d2 * src->var_dims[0] * 2) * 4, 4)));
}

static inline void qping33_field1_SET(int32_t src, Qping33_field1 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1236) set_field(dst, 1236, 0);
	set_bytes((uint32_t) (src), 4, dst->base.bytes, dst->BYTE + (d0 + d1 * dst->var_dims[0] + d2 * dst->var_dims[0] * 2) * 4);
}

/**
															 * brief Getting pointer to the field
															 * param pping33 pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qping33_field12 *pping33_field12(pping33_PING33 *pping33) {
	if (pping33->base.field_bit != 1237 && !set_field(pping33, 1237, -1)) return NULL;
	return pping33;
}

static inline int32_t qping33_field12_GET(Qping33_field12 *const src, size_t d0, size_t d1, size_t d2) {
	return ((int32_t) (get_bytes(src->base.bytes, src->BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)));
}

static inline void pping33_field12_SET(int32_t src, pping33_PING33 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1237) set_field(dst, 1237, 0);
	set_bytes((uint32_t) (src), 4, dst->base.bytes, dst->BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
}

/**
															 * brief Getting pointer to the field
															 * param pping33 pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qping33_field13 *pping33_field13(pping33_PING33 *pping33) {
	if (pping33->base.field_bit != 1238 && !set_field(pping33, 1238, -1)) return NULL;
	return pping33;
}

static inline int32_t vping33_field13_GET(Vping33_field13 const *const src) { return ((int32_t) (get_bytes(src->bytes, 0, 4))); }

static inline bool qping33_field13_GET(Qping33_field13 *const src, Vping33_field13 *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * 3 + d2 * 3 * 2, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vping33_field13_SET(int32_t src, Vping33_field13 const *const dst) { set_bytes((uint32_t) (src), 4, dst->bytes, 0); }

static inline Vping33_field13 pping33_field13_SET(int32_t src, pping33_PING33 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1238) set_field(dst, 1238, 0);
	set_item(dst, d0 + d1 * 3 + d2 * 3 * 2, 0);
	Vping33_field13 ret = {dst->base.bytes + dst->BYTE};
	
	vping33_field13_SET(src, &ret);
	return ret;
}

static inline bool pping33_WWWWWWWW_GET(pping33_PING33 *const src, int32_t *ret) {
	if (src->base.field_bit != 1239 && !set_field(src, 1239, -1)) return false;
	*ret = ((int32_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline void pping33_WWWWWWWW_SET(int32_t src, pping33_PING33 *const dst) {
	
	if (dst->base.field_bit != 1239) set_field(dst, 1239, 0);
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, dst->BYTE);
}

static inline bool pping33_bit_field2_GET(pping33_PING33 *const src, int8_t *ret) {
	if (src->base.field_bit != 1240 && !set_field(src, 1240, -1)) return false;
	*ret = 45 - (int8_t) src->base.bytes[src->BYTE];
	return true;
}

static inline void pping33_bit_field2_SET(int8_t src, pping33_PING33 *const dst) {
	/*use  pping33_bit_field2_MIN and pping33_bit_field2_MAX and pping33_bit_field2_OK(VAL) value range validation facilities */
	if (dst->base.field_bit != 1240) set_field(dst, 1240, 0);
	
	
	dst->base.bytes[dst->BYTE] = (uint8_t) (45 - src);
}

/**
															 * brief Getting pointer to the field
															 * param pping33 pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qping33_Field_Bits *pping33_Field_Bits(pping33_PING33 *pping33) {
	if (pping33->base.field_bit != 1241 && !set_field(pping33, 1241, -1)) return NULL;
	return pping33;
}

static inline int8_t qping33_Field_Bits_GET(Qping33_Field_Bits *const src, size_t d0, size_t d1, size_t d2) {
	return (4 + (int8_t) get_bits(src->base.bytes, src->BIT + (d0 + d1 * 3 + d2 * 3 * 3) * 6, 6));
}

static inline void pping33_Field_Bits_SET(int8_t src, pping33_PING33 *const dst, size_t d0, size_t d1, size_t d2) {
	/*use  pping33_Field_Bits_MIN and pping33_Field_Bits_MAX and pping33_Field_Bits_OK(VAL) value range validation facilities */
	if (dst->base.field_bit != 1241) set_field(dst, 1241, 0);
	set_bits(-4 + src, 6, dst->base.bytes, dst->BIT + (d0 + d1 * 3 + d2 * 3 * 3) * 6);
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_SparseFixAllBits *pping33_SparseFixAllBits(pping33_PING33 *pping33, size_t *d0) {
	if (pping33->base.field_bit != 1242 && !set_field(pping33, 1242, -1))
		if (d0 && *d0) set_field(pping33, 1242, 0, *d0);
		else return NULL;
	
	*d0 = pping33->D[0];
	
	return pping33;
}

static inline int8_t vping33_SparseFixAllBits_GET(Vping33_SparseFixAllBits const *const src) { return (4 + (int8_t) get_bits(src->bytes, src->BIT, 6)); }

static inline bool qping33_SparseFixAllBits_GET(Qping33_SparseFixAllBits *const src, Vping33_SparseFixAllBits *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * src->var_dims[0] + d2 * src->var_dims[0] * 3, -1)) return false;
	
	
	ret->bytes = src->base.bytes;
	ret->BIT   = src->BIT;
	
	return true;
}

static inline void vping33_SparseFixAllBits_SET(int8_t src, Vping33_SparseFixAllBits const *const dst) { /*use  pping33_SparseFixAllBits_MIN and pping33_SparseFixAllBits_MAX and pping33_SparseFixAllBits_OK(VAL) value range validation facilities */ set_bits(-4 + src, 6, dst->bytes, dst->BIT); }

static inline Vping33_SparseFixAllBits qping33_SparseFixAllBits_SET(int8_t src, Qping33_SparseFixAllBits *const dst, size_t d0, size_t d1, size_t d2) {
	/*use  pping33_SparseFixAllBits_MIN and pping33_SparseFixAllBits_MAX and pping33_SparseFixAllBits_OK(VAL) value range validation facilities */
	
	if (dst->base.field_bit != 1242 && !set_field(dst, 1242, -1)) return (Vping33_SparseFixAllBits) {0};
	
	set_item(dst, d0 + d1 * dst->var_dims[0] + d2 * dst->var_dims[0] * 3, 0);
	Vping33_SparseFixAllBits ret = {dst->base.bytes, dst->BIT};
	
	vping33_SparseFixAllBits_SET(src, &ret);
	return ret;
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_FixAllBits *pping33_FixAllBits(pping33_PING33 *pping33, size_t *d0) {
	if (pping33->base.field_bit != 1243 && !set_field(pping33, 1243, -1))
		if (d0 && *d0) set_field(pping33, 1243, 0, *d0);
		else return NULL;
	
	*d0 = pping33->D[0];
	
	return pping33;
}

static inline int8_t qping33_FixAllBits_GET(Qping33_FixAllBits *const src, size_t d0, size_t d1, size_t d2) {
	return (14 + (int8_t) get_bits(src->base.bytes, src->BIT + (d0 + d1 * src->var_dims[0] + d2 * src->var_dims[0] * 3) * 5, 5));
}

static inline void qping33_FixAllBits_SET(int8_t src, Qping33_FixAllBits *const dst, size_t d0, size_t d1, size_t d2) {
	/*use  pping33_FixAllBits_MIN and pping33_FixAllBits_MAX and pping33_FixAllBits_OK(VAL) value range validation facilities */
	if (dst->base.field_bit != 1243) set_field(dst, 1243, 0);
	set_bits(-14 + src, 5, dst->base.bytes, dst->BIT + (d0 + d1 * dst->var_dims[0] + d2 * dst->var_dims[0] * 3) * 5);
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_VarAllBits *pping33_VarAllBits(pping33_PING33 *pping33, size_t *d0, size_t *d2) {
	if (pping33->base.field_bit != 1244 && !set_field(pping33, 1244, -1))
		if (d0 && *d0 && d2 && *d2) set_field(pping33, 1244, 0, *d0, *d2);
		else return NULL;
	
	*d0 = pping33->D[0];
	*d2 = pping33->D[2];
	
	return pping33;
}

static inline int8_t qping33_VarAllBits_GET(Qping33_VarAllBits *const src, size_t d0, size_t d1, size_t d2) {
	return (14 + (int8_t) get_bits(src->base.bytes, src->BIT + (d0 + d1 * src->var_dims[0] + d2 * src->var_dims[0] * 3) * 5, 5));
}

static inline void qping33_VarAllBits_SET(int8_t src, Qping33_VarAllBits *const dst, size_t d0, size_t d1, size_t d2) {
	/*use  pping33_VarAllBits_MIN and pping33_VarAllBits_MAX and pping33_VarAllBits_OK(VAL) value range validation facilities */
	if (dst->base.field_bit != 1244) set_field(dst, 1244, 0);
	set_bits(-14 + src, 5, dst->base.bytes, dst->BIT + (d0 + d1 * dst->var_dims[0] + d2 * dst->var_dims[0] * 3) * 5);
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_SparseVarAllBits *pping33_SparseVarAllBits(pping33_PING33 *pping33, size_t *d0, size_t *d2) {
	if (pping33->base.field_bit != 1245 && !set_field(pping33, 1245, -1))
		if (d0 && *d0 && d2 && *d2) set_field(pping33, 1245, 0, *d0, *d2);
		else return NULL;
	
	*d0 = pping33->D[0];
	*d2 = pping33->D[2];
	
	return pping33;
}

static inline int8_t vping33_SparseVarAllBits_GET(Vping33_SparseVarAllBits const *const src) { return (14 + (int8_t) get_bits(src->bytes, src->BIT, 5)); }

static inline bool qping33_SparseVarAllBits_GET(Qping33_SparseVarAllBits *const src, Vping33_SparseVarAllBits *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * src->var_dims[0] + d2 * src->var_dims[0] * 3, -1)) return false;
	
	
	ret->bytes = src->base.bytes;
	ret->BIT   = src->BIT;
	
	return true;
}

static inline void vping33_SparseVarAllBits_SET(int8_t src, Vping33_SparseVarAllBits const *const dst) { /*use  pping33_SparseVarAllBits_MIN and pping33_SparseVarAllBits_MAX and pping33_SparseVarAllBits_OK(VAL) value range validation facilities */ set_bits(-14 + src, 5, dst->bytes, dst->BIT); }

static inline Vping33_SparseVarAllBits qping33_SparseVarAllBits_SET(int8_t src, Qping33_SparseVarAllBits *const dst, size_t d0, size_t d1, size_t d2) {
	/*use  pping33_SparseVarAllBits_MIN and pping33_SparseVarAllBits_MAX and pping33_SparseVarAllBits_OK(VAL) value range validation facilities */
	
	if (dst->base.field_bit != 1245 && !set_field(dst, 1245, -1)) return (Vping33_SparseVarAllBits) {0};
	
	set_item(dst, d0 + d1 * dst->var_dims[0] + d2 * dst->var_dims[0] * 3, 0);
	Vping33_SparseVarAllBits ret = {dst->base.bytes, dst->BIT};
	
	vping33_SparseVarAllBits_SET(src, &ret);
	return ret;
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_VarEachBits *pping33_VarEachBits(pping33_PING33 *pping33, size_t *d0) {
	if (pping33->base.field_bit != 1246 && !set_field(pping33, 1246, -1))
		if (d0 && *d0) set_field(pping33, 1246, 0, *d0);
		else return NULL;
	
	*d0 = pping33->D[0];
	
	return pping33;
}

static inline int8_t qping33_VarEachBits_GET(Qping33_VarEachBits *const src, size_t d0, size_t d1, size_t d2) {
	return (-14 + (int8_t) get_bits(src->base.bytes, src->BIT + (d0 + d1 * src->var_dims[0] + d2 * src->var_dims[0] * 3) * 6, 6));
}

static inline void qping33_VarEachBits_SET(int8_t src, Qping33_VarEachBits *const dst, size_t d0, size_t d1, size_t d2) {
	/*use  pping33_VarEachBits_MIN and pping33_VarEachBits_MAX and pping33_VarEachBits_OK(VAL) value range validation facilities */
	if (dst->base.field_bit != 1246) set_field(dst, 1246, 0);
	set_bits(14 + src, 6, dst->base.bytes, dst->BIT + (d0 + d1 * dst->var_dims[0] + d2 * dst->var_dims[0] * 3) * 6);
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_SparsVarEachBits *pping33_SparsVarEachBits(pping33_PING33 *pping33, size_t *d0) {
	if (pping33->base.field_bit != 1247 && !set_field(pping33, 1247, -1))
		if (d0 && *d0) set_field(pping33, 1247, 0, *d0);
		else return NULL;
	
	*d0 = pping33->D[0];
	
	return pping33;
}

static inline int16_t vping33_SparsVarEachBits_GET(Vping33_SparsVarEachBits const *const src) { return (-14 + (int16_t) get_bits(src->bytes, src->BIT, 9)); }

static inline bool qping33_SparsVarEachBits_GET(Qping33_SparsVarEachBits *const src, Vping33_SparsVarEachBits *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * src->var_dims[0] + d2 * src->var_dims[0] * 3, -1)) return false;
	
	
	ret->bytes = src->base.bytes;
	ret->BIT   = src->BIT;
	
	return true;
}

static inline void vping33_SparsVarEachBits_SET(int16_t src, Vping33_SparsVarEachBits const *const dst) { /*use  pping33_SparsVarEachBits_MIN and pping33_SparsVarEachBits_MAX and pping33_SparsVarEachBits_OK(VAL) value range validation facilities */ set_bits(14 + src, 9, dst->bytes, dst->BIT); }

static inline Vping33_SparsVarEachBits qping33_SparsVarEachBits_SET(int16_t src, Qping33_SparsVarEachBits *const dst, size_t d0, size_t d1, size_t d2) {
	/*use  pping33_SparsVarEachBits_MIN and pping33_SparsVarEachBits_MAX and pping33_SparsVarEachBits_OK(VAL) value range validation facilities */
	
	if (dst->base.field_bit != 1247 && !set_field(dst, 1247, -1)) return (Vping33_SparsVarEachBits) {0};
	
	set_item(dst, d0 + d1 * dst->var_dims[0] + d2 * dst->var_dims[0] * 3, 0);
	Vping33_SparsVarEachBits ret = {dst->base.bytes, dst->BIT};
	
	vping33_SparsVarEachBits_SET(src, &ret);
	return ret;
}

static inline bool pping33_testBOOLX_GET(pping33_PING33 *const src, bool *ret) {
	if (src->base.field_bit != 1248 && !set_field(src, 1248, -1)) return false;
	*ret = (src->base.bytes[src->BIT >> 3] & 1 << (src->BIT & 7)) != 0;
	return true;
}

static inline void pping33_testBOOLX_SET(bool src, pping33_PING33 *const dst) {
	
	if (dst->base.field_bit != 1248) set_field(dst, 1248, 0);
	
	
	if (src) dst->base.bytes[dst->BIT >> 3] |= 1 << (dst->BIT & 7);
	else dst->base.bytes[dst->BIT >> 3] &= ~(1 << (dst->BIT & 7));
}

static inline bool pping33_testBOOL2X_GET(pping33_PING33 *const src, bool *ret) {
	if (src->base.field_bit != 1249 && !set_field(src, 1249, -1)) return false;
	*ret = (src->base.bytes[src->BIT >> 3] & 1 << (src->BIT & 7)) != 0;
	return true;
}

static inline void pping33_testBOOL2X_SET(bool src, pping33_PING33 *const dst) {
	
	if (dst->base.field_bit != 1249) set_field(dst, 1249, 0);
	
	
	if (src) dst->base.bytes[dst->BIT >> 3] |= 1 << (dst->BIT & 7);
	else dst->base.bytes[dst->BIT >> 3] &= ~(1 << (dst->BIT & 7));
}

static inline bool pping33_testBOOL3X_GET(pping33_PING33 *const src, bool *ret) {
	if (src->base.field_bit != 1250 && !set_field(src, 1250, -1)) return false;
	*ret = (src->base.bytes[src->BIT >> 3] & 1 << (src->BIT & 7)) != 0;
	return true;
}

static inline void pping33_testBOOL3X_SET(bool src, pping33_PING33 *const dst) {
	
	if (dst->base.field_bit != 1250) set_field(dst, 1250, 0);
	
	
	if (src) dst->base.bytes[dst->BIT >> 3] |= 1 << (dst->BIT & 7);
	else dst->base.bytes[dst->BIT >> 3] &= ~(1 << (dst->BIT & 7));
}

static inline bool pping33_MMMMMM_GET(pping33_PING33 *const src, e_MAV_MODE *ret) {
	if (src->base.field_bit != 1251 && !set_field(src, 1251, -1)) return false;
	*ret = _en_mav_mode(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pping33_MMMMMM_SET(e_MAV_MODE src, pping33_PING33 *const dst) {
	
	if (dst->base.field_bit != 1251) set_field(dst, 1251, 0);
	
	
	UMAX id = _id_mav_mode(src);
	set_bits(id, 4, dst->base.bytes, dst->BIT);
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_field44 *pping33_field44(pping33_PING33 *pping33, size_t *d2) {
	if (pping33->base.field_bit != 1252 && !set_field(pping33, 1252, -1))
		if (d2 && *d2) set_field(pping33, 1252, 0, *d2);
		else return NULL;
	
	*d2 = pping33->D[2];
	
	return pping33;
}

static inline int32_t qping33_field44_GET(Qping33_field44 *const src, size_t d0, size_t d1, size_t d2) {
	return ((int32_t) (get_bytes(src->base.bytes, src->BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)));
}

static inline void qping33_field44_SET(int32_t src, Qping33_field44 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1252) set_field(dst, 1252, 0);
	set_bytes((uint32_t) (src), 4, dst->base.bytes, dst->BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_field634 *pping33_field634(pping33_PING33 *pping33, size_t *d2) {
	if (pping33->base.field_bit != 1253 && !set_field(pping33, 1253, -1))
		if (d2 && *d2) set_field(pping33, 1253, 0, *d2);
		else return NULL;
	
	*d2 = pping33->D[2];
	
	return pping33;
}

static inline int32_t qping33_field634_GET(Qping33_field634 *const src, size_t d0, size_t d1, size_t d2) {
	return ((int32_t) (get_bytes(src->base.bytes, src->BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)));
}

static inline void qping33_field634_SET(int32_t src, Qping33_field634 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1253) set_field(dst, 1253, 0);
	set_bytes((uint32_t) (src), 4, dst->base.bytes, dst->BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_field33344 *pping33_field33344(pping33_PING33 *pping33, size_t *d2) {
	if (pping33->base.field_bit != 1254 && !set_field(pping33, 1254, -1))
		if (d2 && *d2) set_field(pping33, 1254, 0, *d2);
		else return NULL;
	
	*d2 = pping33->D[2];
	
	return pping33;
}

static inline int32_t vping33_field33344_GET(Vping33_field33344 const *const src) { return ((int32_t) (get_bytes(src->bytes, 0, 4))); }

static inline bool qping33_field33344_GET(Qping33_field33344 *const src, Vping33_field33344 *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * 3 + d2 * 3 * 2, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vping33_field33344_SET(int32_t src, Vping33_field33344 const *const dst) { set_bytes((uint32_t) (src), 4, dst->bytes, 0); }

static inline Vping33_field33344 qping33_field33344_SET(int32_t src, Qping33_field33344 *const dst, size_t d0, size_t d1, size_t d2) {
	
	
	if (dst->base.field_bit != 1254 && !set_field(dst, 1254, -1)) return (Vping33_field33344) {0};
	
	set_item(dst, d0 + d1 * 3 + d2 * 3 * 2, 0);
	Vping33_field33344 ret = {dst->base.bytes + dst->BYTE};
	
	vping33_field33344_SET(src, &ret);
	return ret;
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_field333634 *pping33_field333634(pping33_PING33 *pping33, size_t *d2) {
	if (pping33->base.field_bit != 1255 && !set_field(pping33, 1255, -1))
		if (d2 && *d2) set_field(pping33, 1255, 0, *d2);
		else return NULL;
	
	*d2 = pping33->D[2];
	
	return pping33;
}

static inline int32_t vping33_field333634_GET(Vping33_field333634 const *const src) { return ((int32_t) (get_bytes(src->bytes, 0, 4))); }

static inline bool qping33_field333634_GET(Qping33_field333634 *const src, Vping33_field333634 *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * 3 + d2 * 3 * 2, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vping33_field333634_SET(int32_t src, Vping33_field333634 const *const dst) { set_bytes((uint32_t) (src), 4, dst->bytes, 0); }

static inline Vping33_field333634 qping33_field333634_SET(int32_t src, Qping33_field333634 *const dst, size_t d0, size_t d1, size_t d2) {
	
	
	if (dst->base.field_bit != 1255 && !set_field(dst, 1255, -1)) return (Vping33_field333634) {0};
	
	set_item(dst, d0 + d1 * 3 + d2 * 3 * 2, 0);
	Vping33_field333634 ret = {dst->base.bytes + dst->BYTE};
	
	vping33_field333634_SET(src, &ret);
	return ret;
}

/**
															 * brief Getting pointer to the field
															 * param pping33 pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qping33_field__ *pping33_field__(pping33_PING33 *pping33) {
	if (pping33->base.field_bit != 1256 && !set_field(pping33, 1256, -1)) return NULL;
	return pping33;
}

static inline int32_t vping33_field___GET(Vping33_field__ const *const src) { return ((int32_t) (get_bytes(src->bytes, 0, 4))); }

static inline bool qping33_field___GET(Qping33_field__ *const src, Vping33_field__ *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * 3 + d2 * 3 * 2, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vping33_field___SET(int32_t src, Vping33_field__ const *const dst) { set_bytes((uint32_t) (src), 4, dst->bytes, 0); }

static inline Vping33_field__ pping33_field___SET(int32_t src, pping33_PING33 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1256) set_field(dst, 1256, 0);
	set_item(dst, d0 + d1 * 3 + d2 * 3 * 2, 0);
	Vping33_field__ ret = {dst->base.bytes + dst->BYTE};
	
	vping33_field___SET(src, &ret);
	return ret;
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_field63 *pping33_field63(pping33_PING33 *pping33, size_t *d2) {
	if (pping33->base.field_bit != 1257 && !set_field(pping33, 1257, -1))
		if (d2 && *d2) set_field(pping33, 1257, 0, *d2);
		else return NULL;
	
	*d2 = pping33->D[2];
	
	return pping33;
}

static inline int32_t qping33_field63_GET(Qping33_field63 *const src, size_t d0, size_t d1, size_t d2) {
	return ((int32_t) (get_bytes(src->base.bytes, src->BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)));
}

static inline void qping33_field63_SET(int32_t src, Qping33_field63 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1257) set_field(dst, 1257, 0);
	set_bytes((uint32_t) (src), 4, dst->base.bytes, dst->BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
}

/**
															 * brief Getting pointer to the field
															 * param pping33 pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qping33_uid2 *pping33_uid2(pping33_PING33 *pping33) {
	if (pping33->base.field_bit != 1258 && !set_field(pping33, 1258, -1)) return NULL;
	return pping33;
}

static inline int8_t vping33_uid2_GET(Vping33_uid2 const *const src) { return (int8_t) src->bytes[0]; }

static inline bool qping33_uid2_GET(Qping33_uid2 *const src, Vping33_uid2 *ret, size_t d0) {
	
	if (!set_item(src, d0, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vping33_uid2_SET(int8_t src, Vping33_uid2 const *const dst) { dst->bytes[0] = (uint8_t) (src); }

static inline Vping33_uid2 pping33_uid2_SET(int8_t src, pping33_PING33 *const dst, size_t d0) {
	
	if (dst->base.field_bit != 1258) set_field(dst, 1258, 0);
	set_item(dst, d0, 0);
	Vping33_uid2 ret = {dst->base.bytes + dst->BYTE};
	
	vping33_uid2_SET(src, &ret);
	return ret;
}

/**
															 * brief Getting pointer to the field
															 * param pping33 pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qping33_field2 *pping33_field2(pping33_PING33 *pping33) {
	if (pping33->base.field_bit != 1259 && !set_field(pping33, 1259, -1)) return NULL;
	return pping33;
}

static inline int32_t vping33_field2_GET(Vping33_field2 const *const src) { return ((int32_t) (get_bytes(src->bytes, 0, 4))); }

static inline bool qping33_field2_GET(Qping33_field2 *const src, Vping33_field2 *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * 3 + d2 * 3 * 2, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vping33_field2_SET(int32_t src, Vping33_field2 const *const dst) { set_bytes((uint32_t) (src), 4, dst->bytes, 0); }

static inline Vping33_field2 pping33_field2_SET(int32_t src, pping33_PING33 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1259) set_field(dst, 1259, 0);
	set_item(dst, d0 + d1 * 3 + d2 * 3 * 2, 0);
	Vping33_field2 ret = {dst->base.bytes + dst->BYTE};
	
	vping33_field2_SET(src, &ret);
	return ret;
}

/**
															 * brief Getting pointer to the field
															 * param pping33 pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qping33_field4 *pping33_field4(pping33_PING33 *pping33) {
	if (pping33->base.field_bit != 1260 && !set_field(pping33, 1260, -1)) return NULL;
	return pping33;
}

static inline int32_t vping33_field4_GET(Vping33_field4 const *const src) { return ((int32_t) (get_bytes(src->bytes, 0, 4))); }

static inline bool qping33_field4_GET(Qping33_field4 *const src, Vping33_field4 *ret, size_t d0, size_t d1, size_t d2) {
	
	if (!set_item(src, d0 + d1 * 3 + d2 * 3 * 2, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vping33_field4_SET(int32_t src, Vping33_field4 const *const dst) { set_bytes((uint32_t) (src), 4, dst->bytes, 0); }

static inline Vping33_field4 pping33_field4_SET(int32_t src, pping33_PING33 *const dst, size_t d0, size_t d1, size_t d2) {
	
	if (dst->base.field_bit != 1260) set_field(dst, 1260, 0);
	set_item(dst, d0 + d1 * 3 + d2 * 3 * 2, 0);
	Vping33_field4 ret = {dst->base.bytes + dst->BYTE};
	
	vping33_field4_SET(src, &ret);
	return ret;
}

static inline bool pping33_stringtest1_GET(pping33_PING33 *const src, Vping33_stringtest1 *ret) {
	
	if (src->base.field_bit != 1261 && !set_field(src, 1261, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vping33_stringtest1 pping33_stringtest1_SET(const char src[], size_t len, pping33_PING33 *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 1261, len);
	
	Vping33_stringtest1 ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

/**
															 * brief Getting or creating field and getting pointer to the field with variable init parameters
															 * param pping33 pointer to pack
															 * return pointer to current field, if any variable dimension params point to zero value
															 * if field exists: provided dimensions parameters will receive field variable dimensions params
															 * if field empty: return NULL
															 * if all variable dimension params point to none zero value
															 * if field NOT exists: create new field with provided parameters and return pointer to the new field
															 * if field exists: return pointer to the field and set dimensions params to the current field params
															 */
static inline Qping33_stringtest2 *pping33_stringtest2(pping33_PING33 *pping33, size_t *d2) {
	if (pping33->base.field_bit != 1262 && !set_field(pping33, 1262, -1))
		if (d2 && *d2) set_field(pping33, 1262, 0, *d2);
		else return NULL;
	
	*d2 = pping33->D[2];
	
	return pping33;
}

static inline bool qping33_stringtest2_GET(Qping33_stringtest2 *const src, Vping33_stringtest2 *ret, size_t d0, size_t d1, size_t d2) {
	
	
	if (!set_item(src, d0 + d1 * 3 + d2 * 3 * 2, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vping33_stringtest2 qping33_stringtest2_SET(const char src[], size_t len, Qping33_stringtest2 *const dst, size_t d0, size_t d1, size_t d2) {
	
	len = 255 < len ? 255 : len;
	
	
	if (dst->base.field_bit != 1262 && !set_field(dst, 1262, -1)) return (Vping33_stringtest2) {0};
	
	set_item(dst, d0 + d1 * 3 + d2 * 3 * 2, len);
	
	Vping33_stringtest2 ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pping33_stringtest3_GET(pping33_PING33 *const src, Vping33_stringtest3 *ret) {
	
	if (src->base.field_bit != 1263 && !set_field(src, 1263, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vping33_stringtest3 pping33_stringtest3_SET(const char src[], size_t len, pping33_PING33 *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 1263, len);
	
	Vping33_stringtest3 ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pping33_stringtest4_GET(pping33_PING33 *const src, Vping33_stringtest4 *ret) {
	
	if (src->base.field_bit != 1264 && !set_field(src, 1264, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vping33_stringtest4 pping33_stringtest4_SET(const char src[], size_t len, pping33_PING33 *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 1264, len);
	
	Vping33_stringtest4 ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

#define  Pping33_const3 ( 56  ) //56
#define  Pping33_stati_cconst1 ( 1  ) //1
#define  Pping33_stati_cconst1D ( 1.456  ) //(float)1.456
#define  Pping33_const3D ( 56.555  ) //(float)56.555


static inline int16_t pvfr_hud_throttle_GET(pvfr_hud_VFR_HUD *src) {//Current throttle setting in integer percent, 0 to 100
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline float pvfr_hud_airspeed_GET(pvfr_hud_VFR_HUD *src) {//Current airspeed in m/s
	
	return (intBitsToFloat(get_bytes(src, 2, 4)));
}


static inline float pvfr_hud_groundspeed_GET(pvfr_hud_VFR_HUD *src) {//Current ground speed in m/s
	
	return (intBitsToFloat(get_bytes(src, 6, 4)));
}


static inline int16_t pvfr_hud_heading_GET(pvfr_hud_VFR_HUD *src) {//Current heading in degrees, in compass units (0..360, 0=north)
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline float pvfr_hud_alt_GET(pvfr_hud_VFR_HUD *src) {//Current altitude (MSL), in meters
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float pvfr_hud_climb_GET(pvfr_hud_VFR_HUD *src) {//Current climb rate in meters/second
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline int16_t prally_point_land_dir_GET(prally_point_RALLY_POINT *src) {//Heading to aim for when landing. In centi-degrees.
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void prally_point_land_dir_SET(int16_t src, prally_point_RALLY_POINT *dst) { //Heading to aim for when landing. In centi-degrees.
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int8_t prally_point_target_system_GET(prally_point_RALLY_POINT *src) {//System ID
	
	return (int8_t) src->base.bytes[2];
}


static inline void prally_point_target_system_SET(int8_t src, prally_point_RALLY_POINT *dst) { //System ID
	
	
	dst->base.bytes[2] = (uint8_t) (src);
}


static inline int8_t prally_point_target_component_GET(prally_point_RALLY_POINT *src) {//Component ID
	
	return (int8_t) src->base.bytes[3];
}


static inline void prally_point_target_component_SET(int8_t src, prally_point_RALLY_POINT *dst) { //Component ID
	
	
	dst->base.bytes[3] = (uint8_t) (src);
}


static inline int8_t prally_point_idx_GET(prally_point_RALLY_POINT *src) {//point index (first point is 0)
	
	return (int8_t) src->base.bytes[4];
}


static inline void prally_point_idx_SET(int8_t src, prally_point_RALLY_POINT *dst) { //point index (first point is 0)
	
	
	dst->base.bytes[4] = (uint8_t) (src);
}


static inline int8_t prally_point_count_GET(prally_point_RALLY_POINT *src) {//total number of points (for sanity checking)
	
	return (int8_t) src->base.bytes[5];
}


static inline void prally_point_count_SET(int8_t src, prally_point_RALLY_POINT *dst) { //total number of points (for sanity checking)
	
	
	dst->base.bytes[5] = (uint8_t) (src);
}


static inline int32_t prally_point_lat_GET(prally_point_RALLY_POINT *src) {//Latitude of point in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline void prally_point_lat_SET(int32_t src, prally_point_RALLY_POINT *dst) { //Latitude of point in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 6);
}


static inline int32_t prally_point_lng_GET(prally_point_RALLY_POINT *src) {//Longitude of point in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 10, 4)));
}


static inline void prally_point_lng_SET(int32_t src, prally_point_RALLY_POINT *dst) { //Longitude of point in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 10);
}


static inline int16_t prally_point_alt_GET(prally_point_RALLY_POINT *src) {//Transit / loiter altitude in meters relative to home
	
	return ((int16_t) (get_bytes(src->base.bytes, 14, 2)));
}


static inline void prally_point_alt_SET(int16_t src, prally_point_RALLY_POINT *dst) { //Transit / loiter altitude in meters relative to home
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 14);
}


static inline int16_t prally_point_break_alt_GET(prally_point_RALLY_POINT *src) {//Break altitude in meters relative to home
	
	return ((int16_t) (get_bytes(src->base.bytes, 16, 2)));
}


static inline void prally_point_break_alt_SET(int16_t src, prally_point_RALLY_POINT *dst) { //Break altitude in meters relative to home
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 16);
}

static inline bool prally_point_flags_GET(prally_point_RALLY_POINT *const src, e_RALLY_FLAGS *ret) {
	if (src->base.field_bit != 144 && !set_field(src, 144, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void prally_point_flags_SET(e_RALLY_FLAGS src, prally_point_RALLY_POINT *const dst) {
	
	if (dst->base.field_bit != 144) set_field(dst, 144, 0);
	
	
	set_bits(-1
	src, 1, dst->base.bytes, dst->BIT);
}


static inline int16_t pmission_set_current_seq_GET(pmission_set_current_MISSION_SET_CURRENT *src) {//Sequence
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int8_t pmission_set_current_target_system_GET(pmission_set_current_MISSION_SET_CURRENT *src) {//System ID
	
	return (int8_t) src[2];
}


static inline int8_t pmission_set_current_target_component_GET(pmission_set_current_MISSION_SET_CURRENT *src) {//Component ID
	
	return (int8_t) src[3];
}


static inline float padap_tuning_desired_GET(padap_tuning_ADAP_TUNING *src) {//desired rate (degrees/s)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 0, 4)));
}


static inline void padap_tuning_desired_SET(float src, padap_tuning_ADAP_TUNING *dst) { //desired rate (degrees/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 0);
}


static inline float padap_tuning_achieved_GET(padap_tuning_ADAP_TUNING *src) {//achieved rate (degrees/s)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 4, 4)));
}


static inline void padap_tuning_achieved_SET(float src, padap_tuning_ADAP_TUNING *dst) { //achieved rate (degrees/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 4);
}


static inline float padap_tuning_error_GET(padap_tuning_ADAP_TUNING *src) {//error between model and vehicle
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline void padap_tuning_error_SET(float src, padap_tuning_ADAP_TUNING *dst) { //error between model and vehicle
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 8);
}


static inline float padap_tuning_theta_GET(padap_tuning_ADAP_TUNING *src) {//theta estimated state predictor
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void padap_tuning_theta_SET(float src, padap_tuning_ADAP_TUNING *dst) { //theta estimated state predictor
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}


static inline float padap_tuning_omega_GET(padap_tuning_ADAP_TUNING *src) {//omega estimated state predictor
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline void padap_tuning_omega_SET(float src, padap_tuning_ADAP_TUNING *dst) { //omega estimated state predictor
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 16);
}


static inline float padap_tuning_sigma_GET(padap_tuning_ADAP_TUNING *src) {//sigma estimated state predictor
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline void padap_tuning_sigma_SET(float src, padap_tuning_ADAP_TUNING *dst) { //sigma estimated state predictor
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 20);
}


static inline float padap_tuning_theta_dot_GET(padap_tuning_ADAP_TUNING *src) {//theta derivative
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline void padap_tuning_theta_dot_SET(float src, padap_tuning_ADAP_TUNING *dst) { //theta derivative
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 24);
}


static inline float padap_tuning_omega_dot_GET(padap_tuning_ADAP_TUNING *src) {//omega derivative
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline void padap_tuning_omega_dot_SET(float src, padap_tuning_ADAP_TUNING *dst) { //omega derivative
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 28);
}


static inline float padap_tuning_sigma_dot_GET(padap_tuning_ADAP_TUNING *src) {//sigma derivative
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline void padap_tuning_sigma_dot_SET(float src, padap_tuning_ADAP_TUNING *dst) { //sigma derivative
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 32);
}


static inline float padap_tuning_f_GET(padap_tuning_ADAP_TUNING *src) {//projection operator value
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 36, 4)));
}


static inline void padap_tuning_f_SET(float src, padap_tuning_ADAP_TUNING *dst) { //projection operator value
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 36);
}


static inline float padap_tuning_f_dot_GET(padap_tuning_ADAP_TUNING *src) {//projection operator derivative
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 40, 4)));
}


static inline void padap_tuning_f_dot_SET(float src, padap_tuning_ADAP_TUNING *dst) { //projection operator derivative
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 40);
}


static inline float padap_tuning_u_GET(padap_tuning_ADAP_TUNING *src) {//u adaptive controlled output command
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 44, 4)));
}


static inline void padap_tuning_u_SET(float src, padap_tuning_ADAP_TUNING *dst) { //u adaptive controlled output command
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 44);
}

static inline bool padap_tuning_axis_GET(padap_tuning_ADAP_TUNING *const src, e_PID_TUNING_AXIS *ret) {
	if (src->base.field_bit != 384 && !set_field(src, 384, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void padap_tuning_axis_SET(e_PID_TUNING_AXIS src, padap_tuning_ADAP_TUNING *const dst) {
	
	if (dst->base.field_bit != 384) set_field(dst, 384, 0);
	
	
	set_bits(-1
	src, 3, dst->base.bytes, dst->BIT);
}


static inline int32_t pvibration_clipping_0_GET(pvibration_VIBRATION *src) {//first accelerometer clipping count
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pvibration_clipping_0_SET(int32_t src, pvibration_VIBRATION *dst) { //first accelerometer clipping count
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int32_t pvibration_clipping_1_GET(pvibration_VIBRATION *src) {//second accelerometer clipping count
	
	return ((int32_t) (get_bytes(src, 4, 4)));
}


static inline void pvibration_clipping_1_SET(int32_t src, pvibration_VIBRATION *dst) { //second accelerometer clipping count
	
	
	set_bytes((uint32_t) (src), 4, dst, 4);
}


static inline int32_t pvibration_clipping_2_GET(pvibration_VIBRATION *src) {//third accelerometer clipping count
	
	return ((int32_t) (get_bytes(src, 8, 4)));
}


static inline void pvibration_clipping_2_SET(int32_t src, pvibration_VIBRATION *dst) { //third accelerometer clipping count
	
	
	set_bytes((uint32_t) (src), 4, dst, 8);
}


static inline int64_t pvibration_time_usec_GET(pvibration_VIBRATION *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src, 12, 8)));
}


static inline void pvibration_time_usec_SET(int64_t src, pvibration_VIBRATION *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst, 12);
}


static inline float pvibration_vibration_x_GET(pvibration_VIBRATION *src) {//Vibration levels on X-axis
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void pvibration_vibration_x_SET(float src, pvibration_VIBRATION *dst) { //Vibration levels on X-axis
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float pvibration_vibration_y_GET(pvibration_VIBRATION *src) {//Vibration levels on Y-axis
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void pvibration_vibration_y_SET(float src, pvibration_VIBRATION *dst) { //Vibration levels on Y-axis
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float pvibration_vibration_z_GET(pvibration_VIBRATION *src) {//Vibration levels on Z-axis
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void pvibration_vibration_z_SET(float src, pvibration_VIBRATION *dst) { //Vibration levels on Z-axis
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline int16_t pparam_ext_value_param_count_GET(pparam_ext_value_PARAM_EXT_VALUE *src) {//Total number of parameters
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pparam_ext_value_param_count_SET(int16_t src, pparam_ext_value_PARAM_EXT_VALUE *dst) { //Total number of parameters
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t pparam_ext_value_param_index_GET(pparam_ext_value_PARAM_EXT_VALUE *src) {//Index of this parameter
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pparam_ext_value_param_index_SET(int16_t src, pparam_ext_value_PARAM_EXT_VALUE *dst) { //Index of this parameter
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}

/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*					 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*					 ID is stored as strin */

static inline bool pparam_ext_value_param_id_GET(pparam_ext_value_PARAM_EXT_VALUE *const src, Vparam_ext_value_param_id *ret) {
	
	if (src->base.field_bit != 35 && !set_field(src, 35, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vparam_ext_value_param_id pparam_ext_value_param_id_SET(const char src[], size_t len, pparam_ext_value_PARAM_EXT_VALUE *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 35, len);
	
	Vparam_ext_value_param_id ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pparam_ext_value_param_value_GET(pparam_ext_value_PARAM_EXT_VALUE *const src, Vparam_ext_value_param_value *ret) {//Parameter value
	
	if (src->base.field_bit != 36 && !set_field(src, 36, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vparam_ext_value_param_value pparam_ext_value_param_value_SET(const char src[], size_t len, pparam_ext_value_PARAM_EXT_VALUE *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 36, len);
	
	Vparam_ext_value_param_value ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pparam_ext_value_param_type_GET(pparam_ext_value_PARAM_EXT_VALUE *const src, e_MAV_PARAM_EXT_TYPE *ret) {
	if (src->base.field_bit != 37 && !set_field(src, 37, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pparam_ext_value_param_type_SET(e_MAV_PARAM_EXT_TYPE src, pparam_ext_value_PARAM_EXT_VALUE *const dst) {
	
	if (dst->base.field_bit != 37) set_field(dst, 37, 0);
	
	
	set_bits(-1
	src, 4, dst->base.bytes, dst->BIT);
}


static inline int16_t pbattery2_voltage_GET(pbattery2_BATTERY2 *src) {//voltage in millivolts
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pbattery2_voltage_SET(int16_t src, pbattery2_BATTERY2 *dst) { //voltage in millivolts
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t pbattery2_current_battery_GET(pbattery2_BATTERY2 *src) {//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pbattery2_current_battery_SET(int16_t src, pbattery2_BATTERY2 *dst) { //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t plimits_status_breach_count_GET(plimits_status_LIMITS_STATUS *src) {//number of fence breaches
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void plimits_status_breach_count_SET(int16_t src, plimits_status_LIMITS_STATUS *dst) { //number of fence breaches
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int32_t plimits_status_last_trigger_GET(plimits_status_LIMITS_STATUS *src) {//time of last breach in milliseconds since boot
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline void plimits_status_last_trigger_SET(int32_t src, plimits_status_LIMITS_STATUS *dst) { //time of last breach in milliseconds since boot
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 2);
}


static inline int32_t plimits_status_last_action_GET(plimits_status_LIMITS_STATUS *src) {//time of last recovery action in milliseconds since boot
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline void plimits_status_last_action_SET(int32_t src, plimits_status_LIMITS_STATUS *dst) { //time of last recovery action in milliseconds since boot
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 6);
}


static inline int32_t plimits_status_last_recovery_GET(plimits_status_LIMITS_STATUS *src) {//time of last successful recovery in milliseconds since boot
	
	return ((int32_t) (get_bytes(src->base.bytes, 10, 4)));
}


static inline void plimits_status_last_recovery_SET(int32_t src, plimits_status_LIMITS_STATUS *dst) { //time of last successful recovery in milliseconds since boot
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 10);
}


static inline int32_t plimits_status_last_clear_GET(plimits_status_LIMITS_STATUS *src) {//time of last all-clear in milliseconds since boot
	
	return ((int32_t) (get_bytes(src->base.bytes, 14, 4)));
}


static inline void plimits_status_last_clear_SET(int32_t src, plimits_status_LIMITS_STATUS *dst) { //time of last all-clear in milliseconds since boot
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 14);
}

static inline bool plimits_status_limits_state_GET(plimits_status_LIMITS_STATUS *const src, e_LIMITS_STATE *ret) {
	if (src->base.field_bit != 146 && !set_field(src, 146, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void plimits_status_limits_state_SET(e_LIMITS_STATE src, plimits_status_LIMITS_STATUS *const dst) {
	
	if (dst->base.field_bit != 146) set_field(dst, 146, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool plimits_status_mods_enabled_GET(plimits_status_LIMITS_STATUS *const src, e_LIMIT_MODULE *ret) {
	if (src->base.field_bit != 147 && !set_field(src, 147, -1)) return false;
	*ret = _en_limit_module(get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void plimits_status_mods_enabled_SET(e_LIMIT_MODULE src, plimits_status_LIMITS_STATUS *const dst) {
	
	if (dst->base.field_bit != 147) set_field(dst, 147, 0);
	
	
	UMAX id = _id_limit_module(src);
	set_bits(id, 2, dst->base.bytes, dst->BIT);
}

static inline bool plimits_status_mods_required_GET(plimits_status_LIMITS_STATUS *const src, e_LIMIT_MODULE *ret) {
	if (src->base.field_bit != 148 && !set_field(src, 148, -1)) return false;
	*ret = _en_limit_module(get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void plimits_status_mods_required_SET(e_LIMIT_MODULE src, plimits_status_LIMITS_STATUS *const dst) {
	
	if (dst->base.field_bit != 148) set_field(dst, 148, 0);
	
	
	UMAX id = _id_limit_module(src);
	set_bits(id, 2, dst->base.bytes, dst->BIT);
}

static inline bool plimits_status_mods_triggered_GET(plimits_status_LIMITS_STATUS *const src, e_LIMIT_MODULE *ret) {
	if (src->base.field_bit != 149 && !set_field(src, 149, -1)) return false;
	*ret = _en_limit_module(get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void plimits_status_mods_triggered_SET(e_LIMIT_MODULE src, plimits_status_LIMITS_STATUS *const dst) {
	
	if (dst->base.field_bit != 149) set_field(dst, 149, 0);
	
	
	UMAX id = _id_limit_module(src);
	set_bits(id, 2, dst->base.bytes, dst->BIT);
}


static inline int16_t pcamera_feedback_img_idx_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Image index
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pcamera_feedback_img_idx_SET(int16_t src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Image index
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}

/**
*Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if
*					 no CCB */

static inline int64_t pcamera_feedback_time_usec_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {
	
	return ((int64_t) (get_bytes(src->base.bytes, 2, 8)));
}

/**
*Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if
*					 no CCB */

static inline void pcamera_feedback_time_usec_SET(int64_t src, pcamera_feedback_CAMERA_FEEDBACK *dst) {
	
	
	set_bytes((src), 8, dst->base.bytes, 2);
}


static inline int8_t pcamera_feedback_target_system_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//System ID
	
	return (int8_t) src->base.bytes[10];
}


static inline void pcamera_feedback_target_system_SET(int8_t src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //System ID
	
	
	dst->base.bytes[10] = (uint8_t) (src);
}


static inline int8_t pcamera_feedback_cam_idx_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Camera ID
	
	return (int8_t) src->base.bytes[11];
}


static inline void pcamera_feedback_cam_idx_SET(int8_t src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Camera ID
	
	
	dst->base.bytes[11] = (uint8_t) (src);
}


static inline int32_t pcamera_feedback_lat_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Latitude in (deg * 1E7)
	
	return ((int32_t) (get_bytes(src->base.bytes, 12, 4)));
}


static inline void pcamera_feedback_lat_SET(int32_t src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Latitude in (deg * 1E7)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 12);
}


static inline int32_t pcamera_feedback_lng_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Longitude in (deg * 1E7)
	
	return ((int32_t) (get_bytes(src->base.bytes, 16, 4)));
}


static inline void pcamera_feedback_lng_SET(int32_t src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Longitude in (deg * 1E7)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 16);
}


static inline float pcamera_feedback_alt_msl_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Altitude Absolute (meters AMSL)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline void pcamera_feedback_alt_msl_SET(float src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Altitude Absolute (meters AMSL)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 20);
}


static inline float pcamera_feedback_alt_rel_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Altitude Relative (meters above HOME location)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline void pcamera_feedback_alt_rel_SET(float src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Altitude Relative (meters above HOME location)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 24);
}


static inline float pcamera_feedback_roll_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Camera Roll angle (earth frame, degrees, +-180)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline void pcamera_feedback_roll_SET(float src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Camera Roll angle (earth frame, degrees, +-180)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 28);
}


static inline float pcamera_feedback_pitch_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Camera Pitch angle (earth frame, degrees, +-180)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline void pcamera_feedback_pitch_SET(float src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Camera Pitch angle (earth frame, degrees, +-180)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 32);
}


static inline float pcamera_feedback_yaw_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Camera Yaw (earth frame, degrees, 0-360, true)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 36, 4)));
}


static inline void pcamera_feedback_yaw_SET(float src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Camera Yaw (earth frame, degrees, 0-360, true)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 36);
}


static inline float pcamera_feedback_foc_len_GET(pcamera_feedback_CAMERA_FEEDBACK *src) {//Focal Length (mm)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 40, 4)));
}


static inline void pcamera_feedback_foc_len_SET(float src, pcamera_feedback_CAMERA_FEEDBACK *dst) { //Focal Length (mm)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 40);
}

static inline bool pcamera_feedback_flags_GET(pcamera_feedback_CAMERA_FEEDBACK *const src, e_CAMERA_FEEDBACK_FLAGS *ret) {
	if (src->base.field_bit != 352 && !set_field(src, 352, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pcamera_feedback_flags_SET(e_CAMERA_FEEDBACK_FLAGS src, pcamera_feedback_CAMERA_FEEDBACK *const dst) {
	
	if (dst->base.field_bit != 352) set_field(dst, 352, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}


static inline int16_t phil_gps_eph_GET(phil_gps_HIL_GPS *src) {//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void phil_gps_eph_SET(int16_t src, phil_gps_HIL_GPS *dst) { //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t phil_gps_epv_GET(phil_gps_HIL_GPS *src) {//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void phil_gps_epv_SET(int16_t src, phil_gps_HIL_GPS *dst) { //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t phil_gps_vel_GET(phil_gps_HIL_GPS *src) {//GPS ground speed in cm/s. If unknown, set to: 65535
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void phil_gps_vel_SET(int16_t src, phil_gps_HIL_GPS *dst) { //GPS ground speed in cm/s. If unknown, set to: 65535
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}

/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*					 unknown, set to: 6553 */

static inline int16_t phil_gps_cog_GET(phil_gps_HIL_GPS *src) {
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}

/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*					 unknown, set to: 6553 */

static inline void phil_gps_cog_SET(int16_t src, phil_gps_HIL_GPS *dst) {
	
	
	set_bytes((uint16_t) (src), 2, dst, 6);
}


static inline int64_t phil_gps_time_usec_GET(phil_gps_HIL_GPS *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src, 8, 8)));
}


static inline void phil_gps_time_usec_SET(int64_t src, phil_gps_HIL_GPS *dst) { //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	
	set_bytes((src), 8, dst, 8);
}

/**
*0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
*					 at least two, so always correctly fill in the fix */

static inline int8_t phil_gps_fix_type_GET(phil_gps_HIL_GPS *src) {
	
	return (int8_t) src[16];
}

/**
*0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
*					 at least two, so always correctly fill in the fix */

static inline void phil_gps_fix_type_SET(int8_t src, phil_gps_HIL_GPS *dst) {
	
	
	dst[16] = (uint8_t) (src);
}


static inline int32_t phil_gps_lat_GET(phil_gps_HIL_GPS *src) {//Latitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 17, 4)));
}


static inline void phil_gps_lat_SET(int32_t src, phil_gps_HIL_GPS *dst) { //Latitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 17);
}


static inline int32_t phil_gps_lon_GET(phil_gps_HIL_GPS *src) {//Longitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 21, 4)));
}


static inline void phil_gps_lon_SET(int32_t src, phil_gps_HIL_GPS *dst) { //Longitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 21);
}


static inline int32_t phil_gps_alt_GET(phil_gps_HIL_GPS *src) {//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
	
	return ((int32_t) (get_bytes(src, 25, 4)));
}


static inline void phil_gps_alt_SET(int32_t src, phil_gps_HIL_GPS *dst) { //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
	
	
	set_bytes((uint32_t) (src), 4, dst, 25);
}


static inline int16_t phil_gps_vn_GET(phil_gps_HIL_GPS *src) {//GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
	
	return ((int16_t) (get_bytes(src, 29, 2)));
}


static inline void phil_gps_vn_SET(int16_t src, phil_gps_HIL_GPS *dst) { //GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
	
	
	set_bytes((uint16_t) (src), 2, dst, 29);
}


static inline int16_t phil_gps_ve_GET(phil_gps_HIL_GPS *src) {//GPS velocity in cm/s in EAST direction in earth-fixed NED frame
	
	return ((int16_t) (get_bytes(src, 31, 2)));
}


static inline void phil_gps_ve_SET(int16_t src, phil_gps_HIL_GPS *dst) { //GPS velocity in cm/s in EAST direction in earth-fixed NED frame
	
	
	set_bytes((uint16_t) (src), 2, dst, 31);
}


static inline int16_t phil_gps_vd_GET(phil_gps_HIL_GPS *src) {//GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
	
	return ((int16_t) (get_bytes(src, 33, 2)));
}


static inline void phil_gps_vd_SET(int16_t src, phil_gps_HIL_GPS *dst) { //GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
	
	
	set_bytes((uint16_t) (src), 2, dst, 33);
}


static inline int8_t phil_gps_satellites_visible_GET(phil_gps_HIL_GPS *src) {//Number of satellites visible. If unknown, set to 255
	
	return (int8_t) src[35];
}


static inline void phil_gps_satellites_visible_SET(int8_t src, phil_gps_HIL_GPS *dst) { //Number of satellites visible. If unknown, set to 255
	
	
	dst[35] = (uint8_t) (src);
}


static inline int16_t pnav_controller_output_wp_dist_GET(pnav_controller_output_NAV_CONTROLLER_OUTPUT *src) {//Distance to active waypoint in meters
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline float pnav_controller_output_nav_roll_GET(pnav_controller_output_NAV_CONTROLLER_OUTPUT *src) {//Current desired roll in degrees
	
	return (intBitsToFloat(get_bytes(src, 2, 4)));
}


static inline float pnav_controller_output_nav_pitch_GET(pnav_controller_output_NAV_CONTROLLER_OUTPUT *src) {//Current desired pitch in degrees
	
	return (intBitsToFloat(get_bytes(src, 6, 4)));
}


static inline int16_t pnav_controller_output_nav_bearing_GET(pnav_controller_output_NAV_CONTROLLER_OUTPUT *src) {//Current desired heading in degrees
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline int16_t pnav_controller_output_target_bearing_GET(pnav_controller_output_NAV_CONTROLLER_OUTPUT *src) {//Bearing to current waypoint/target in degrees
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline float pnav_controller_output_alt_error_GET(pnav_controller_output_NAV_CONTROLLER_OUTPUT *src) {//Current altitude error in meters
	
	return (intBitsToFloat(get_bytes(src, 14, 4)));
}


static inline float pnav_controller_output_aspd_error_GET(pnav_controller_output_NAV_CONTROLLER_OUTPUT *src) {//Current airspeed error in meters/second
	
	return (intBitsToFloat(get_bytes(src, 18, 4)));
}


static inline float pnav_controller_output_xtrack_error_GET(pnav_controller_output_NAV_CONTROLLER_OUTPUT *src) {//Current crosstrack error on x-y plane in meters
	
	return (intBitsToFloat(get_bytes(src, 22, 4)));
}

static inline bool pauth_key_key_GET(pauth_key_AUTH_KEY *const src, Vauth_key_key *ret) {//key
	
	if (src->base.field_bit != 2 && !set_field(src, 2, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}


static inline int8_t pfence_fetch_point_target_system_GET(pfence_fetch_point_FENCE_FETCH_POINT *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pfence_fetch_point_target_system_SET(int8_t src, pfence_fetch_point_FENCE_FETCH_POINT *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pfence_fetch_point_target_component_GET(pfence_fetch_point_FENCE_FETCH_POINT *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pfence_fetch_point_target_component_SET(int8_t src, pfence_fetch_point_FENCE_FETCH_POINT *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t pfence_fetch_point_idx_GET(pfence_fetch_point_FENCE_FETCH_POINT *src) {//point index (first point is 1, 0 is for return point)
	
	return (int8_t) src[2];
}


static inline void pfence_fetch_point_idx_SET(int8_t src, pfence_fetch_point_FENCE_FETCH_POINT *dst) { //point index (first point is 1, 0 is for return point)
	
	
	dst[2] = (uint8_t) (src);
}


static inline int16_t pradio_rxerrors_GET(pradio_RADIO *src) {//receive errors
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pradio_rxerrors_SET(int16_t src, pradio_RADIO *dst) { //receive errors
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t pradio_fixeD_GET(pradio_RADIO *src) {//count of error corrected packets
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pradio_fixeD_SET(int16_t src, pradio_RADIO *dst) { //count of error corrected packets
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int8_t pradio_rssi_GET(pradio_RADIO *src) {//local signal strength
	
	return (int8_t) src[4];
}


static inline void pradio_rssi_SET(int8_t src, pradio_RADIO *dst) { //local signal strength
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t pradio_remrssi_GET(pradio_RADIO *src) {//remote signal strength
	
	return (int8_t) src[5];
}


static inline void pradio_remrssi_SET(int8_t src, pradio_RADIO *dst) { //remote signal strength
	
	
	dst[5] = (uint8_t) (src);
}


static inline int8_t pradio_txbuf_GET(pradio_RADIO *src) {//how full the tx buffer is as a percentage
	
	return (int8_t) src[6];
}


static inline void pradio_txbuf_SET(int8_t src, pradio_RADIO *dst) { //how full the tx buffer is as a percentage
	
	
	dst[6] = (uint8_t) (src);
}


static inline int8_t pradio_noise_GET(pradio_RADIO *src) {//background noise level
	
	return (int8_t) src[7];
}


static inline void pradio_noise_SET(int8_t src, pradio_RADIO *dst) { //background noise level
	
	
	dst[7] = (uint8_t) (src);
}


static inline int8_t pradio_remnoise_GET(pradio_RADIO *src) {//remote background noise level
	
	return (int8_t) src[8];
}


static inline void pradio_remnoise_SET(int8_t src, pradio_RADIO *dst) { //remote background noise level
	
	
	dst[8] = (uint8_t) (src);
}


static inline int64_t plocal_position_ned_cov_time_usec_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//Timestamp (microseconds since system boot or since UNIX epoch)
	
	return ((int64_t) (get_bytes(src->base.bytes, 0, 8)));
}


static inline float plocal_position_ned_cov_x_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//X Position
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline float plocal_position_ned_cov_y_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//Y Position
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline float plocal_position_ned_cov_z_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//Z Position
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline float plocal_position_ned_cov_vx_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//X Speed (m/s)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline float plocal_position_ned_cov_vy_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//Y Speed (m/s)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline float plocal_position_ned_cov_vz_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//Z Speed (m/s)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline float plocal_position_ned_cov_ax_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//X Acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline float plocal_position_ned_cov_ay_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//Y Acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 36, 4)));
}


static inline float plocal_position_ned_cov_az_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {//Z Acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 40, 4)));
}

/**
*Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
*					 the second row, etc. */

static inline float vlocal_position_ned_cov_covariance_GET(Vlocal_position_ned_cov_covariance const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

/**
*Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
*					 the second row, etc. */

static inline Vlocal_position_ned_cov_covariance plocal_position_ned_cov_covariance_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *src) {
	
	return (Vlocal_position_ned_cov_covariance) {src->base.bytes + 44, 45};
}

static inline bool plocal_position_ned_cov_estimator_type_GET(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *const src, e_MAV_ESTIMATOR_TYPE *ret) {
	if (src->base.field_bit != 1792 && !set_field(src, 1792, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline float pairspeed_autocal_vx_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//GPS velocity north m/s
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void pairspeed_autocal_vx_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //GPS velocity north m/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float pairspeed_autocal_vy_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//GPS velocity east m/s
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pairspeed_autocal_vy_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //GPS velocity east m/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pairspeed_autocal_vz_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//GPS velocity down m/s
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pairspeed_autocal_vz_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //GPS velocity down m/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pairspeed_autocal_diff_pressure_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//Differential pressure pascals
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pairspeed_autocal_diff_pressure_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //Differential pressure pascals
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float pairspeed_autocal_EAS2TAS_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//Estimated to true airspeed ratio
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void pairspeed_autocal_EAS2TAS_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //Estimated to true airspeed ratio
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float pairspeed_autocal_ratio_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//Airspeed ratio
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void pairspeed_autocal_ratio_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //Airspeed ratio
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float pairspeed_autocal_state_x_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//EKF state x
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void pairspeed_autocal_state_x_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //EKF state x
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float pairspeed_autocal_state_y_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//EKF state y
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void pairspeed_autocal_state_y_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //EKF state y
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float pairspeed_autocal_state_z_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//EKF state z
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void pairspeed_autocal_state_z_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //EKF state z
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}


static inline float pairspeed_autocal_Pax_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//EKF Pax
	
	return (intBitsToFloat(get_bytes(src, 36, 4)));
}


static inline void pairspeed_autocal_Pax_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //EKF Pax
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 36);
}


static inline float pairspeed_autocal_Pby_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//EKF Pby
	
	return (intBitsToFloat(get_bytes(src, 40, 4)));
}


static inline void pairspeed_autocal_Pby_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //EKF Pby
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 40);
}


static inline float pairspeed_autocal_Pcz_GET(pairspeed_autocal_AIRSPEED_AUTOCAL *src) {//EKF Pcz
	
	return (intBitsToFloat(get_bytes(src, 44, 4)));
}


static inline void pairspeed_autocal_Pcz_SET(float src, pairspeed_autocal_AIRSPEED_AUTOCAL *dst) { //EKF Pcz
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 44);
}


static inline int64_t patt_pos_mocap_time_usec_GET(patt_pos_mocap_ATT_POS_MOCAP *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void patt_pos_mocap_time_usec_SET(int64_t src, patt_pos_mocap_ATT_POS_MOCAP *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst, 0);
}


static inline float vatt_pos_mocap_q_GET(Vatt_pos_mocap_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vatt_pos_mocap_q patt_pos_mocap_q_GET(patt_pos_mocap_ATT_POS_MOCAP *src) { //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	
	return (Vatt_pos_mocap_q) {src + 8, 4};
}


static inline void vatt_pos_mocap_q_SET(float src, size_t index, Vatt_pos_mocap_q *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vatt_pos_mocap_q patt_pos_mocap_q_SET(const float src[], patt_pos_mocap_ATT_POS_MOCAP *dst) {//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	
	
	Vatt_pos_mocap_q ret = {dst + 8, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vatt_pos_mocap_q_SET(src[i], i, &ret);
	return ret;
}


static inline float patt_pos_mocap_x_GET(patt_pos_mocap_ATT_POS_MOCAP *src) {//X position in meters (NED)
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void patt_pos_mocap_x_SET(float src, patt_pos_mocap_ATT_POS_MOCAP *dst) { //X position in meters (NED)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float patt_pos_mocap_y_GET(patt_pos_mocap_ATT_POS_MOCAP *src) {//Y position in meters (NED)
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void patt_pos_mocap_y_SET(float src, patt_pos_mocap_ATT_POS_MOCAP *dst) { //Y position in meters (NED)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float patt_pos_mocap_z_GET(patt_pos_mocap_ATT_POS_MOCAP *src) {//Z position in meters (NED)
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void patt_pos_mocap_z_SET(float src, patt_pos_mocap_ATT_POS_MOCAP *dst) { //Z position in meters (NED)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}

static inline bool pstatustext_severity_GET(pstatustext_STATUSTEXT *const src, e_MAV_SEVERITY *ret) {
	if (src->base.field_bit != 2 && !set_field(src, 2, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pstatustext_severity_SET(e_MAV_SEVERITY src, pstatustext_STATUSTEXT *const dst) {
	
	if (dst->base.field_bit != 2) set_field(dst, 2, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool pstatustext_text_GET(pstatustext_STATUSTEXT *const src, Vstatustext_text *ret) {//Status text message, without null termination character
	
	if (src->base.field_bit != 3 && !set_field(src, 3, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vstatustext_text pstatustext_text_SET(const char src[], size_t len, pstatustext_STATUSTEXT *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 3, len);
	
	Vstatustext_text ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int32_t pping_seq_GET(pping_PING *src) {//PING sequence
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline int64_t pping_time_usec_GET(pping_PING *src) {//Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
	
	return ((int64_t) (get_bytes(src, 4, 8)));
}

/**
*0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
*					 the system id of the requesting syste */

static inline int8_t pping_target_system_GET(pping_PING *src) {
	
	return (int8_t) src[12];
}

/**
*0: request ping from all receiving components, if greater than 0: message is a ping response and number
*					 is the system id of the requesting syste */

static inline int8_t pping_target_component_GET(pping_PING *src) {
	
	return (int8_t) src[13];
}


static inline int8_t pgopro_get_request_target_system_GET(pgopro_get_request_GOPRO_GET_REQUEST *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline void pgopro_get_request_target_system_SET(int8_t src, pgopro_get_request_GOPRO_GET_REQUEST *dst) { //System ID
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t pgopro_get_request_target_component_GET(pgopro_get_request_GOPRO_GET_REQUEST *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline void pgopro_get_request_target_component_SET(int8_t src, pgopro_get_request_GOPRO_GET_REQUEST *dst) { //Component ID
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}

static inline bool pgopro_get_request_cmd_id_GET(pgopro_get_request_GOPRO_GET_REQUEST *const src, e_GOPRO_COMMAND *ret) {
	if (src->base.field_bit != 16 && !set_field(src, 16, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline void pgopro_get_request_cmd_id_SET(e_GOPRO_COMMAND src, pgopro_get_request_GOPRO_GET_REQUEST *const dst) {
	
	if (dst->base.field_bit != 16) set_field(dst, 16, 0);
	
	
	set_bits(src, 5, dst->base.bytes, dst->BIT);
}


static inline int32_t pcamera_capture_status_time_boot_ms_GET(pcamera_capture_status_CAMERA_CAPTURE_STATUS *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pcamera_capture_status_time_boot_ms_SET(int32_t src, pcamera_capture_status_CAMERA_CAPTURE_STATUS *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int32_t pcamera_capture_status_recording_time_ms_GET(pcamera_capture_status_CAMERA_CAPTURE_STATUS *src) {//Time in milliseconds since recording started
	
	return ((int32_t) (get_bytes(src, 4, 4)));
}


static inline void pcamera_capture_status_recording_time_ms_SET(int32_t src, pcamera_capture_status_CAMERA_CAPTURE_STATUS *dst) { //Time in milliseconds since recording started
	
	
	set_bytes((uint32_t) (src), 4, dst, 4);
}

/**
*Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
*					 set and capture in progress */

static inline int8_t pcamera_capture_status_image_status_GET(pcamera_capture_status_CAMERA_CAPTURE_STATUS *src) {
	
	return (int8_t) src[8];
}

/**
*Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
*					 set and capture in progress */

static inline void pcamera_capture_status_image_status_SET(int8_t src, pcamera_capture_status_CAMERA_CAPTURE_STATUS *dst) {
	
	
	dst[8] = (uint8_t) (src);
}


static inline int8_t pcamera_capture_status_video_status_GET(pcamera_capture_status_CAMERA_CAPTURE_STATUS *src) {//Current status of video capturing (0: idle, 1: capture in progress)
	
	return (int8_t) src[9];
}


static inline void pcamera_capture_status_video_status_SET(int8_t src, pcamera_capture_status_CAMERA_CAPTURE_STATUS *dst) { //Current status of video capturing (0: idle, 1: capture in progress)
	
	
	dst[9] = (uint8_t) (src);
}


static inline float pcamera_capture_status_image_interval_GET(pcamera_capture_status_CAMERA_CAPTURE_STATUS *src) {//Image capture interval in seconds
	
	return (intBitsToFloat(get_bytes(src, 10, 4)));
}


static inline void pcamera_capture_status_image_interval_SET(float src, pcamera_capture_status_CAMERA_CAPTURE_STATUS *dst) { //Image capture interval in seconds
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 10);
}


static inline float pcamera_capture_status_available_capacity_GET(pcamera_capture_status_CAMERA_CAPTURE_STATUS *src) {//Available storage capacity in MiB
	
	return (intBitsToFloat(get_bytes(src, 14, 4)));
}


static inline void pcamera_capture_status_available_capacity_SET(float src, pcamera_capture_status_CAMERA_CAPTURE_STATUS *dst) { //Available storage capacity in MiB
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 14);
}


static inline int16_t pglobal_position_int_hdg_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {//Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int32_t pglobal_position_int_time_boot_ms_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline int32_t pglobal_position_int_lat_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {//Latitude, expressed as degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline int32_t pglobal_position_int_lon_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {//Longitude, expressed as degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}

/**
*Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
*					 provide the AMSL as well */

static inline int32_t pglobal_position_int_alt_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {
	
	return ((int32_t) (get_bytes(src, 14, 4)));
}


static inline int32_t pglobal_position_int_relative_alt_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {//Altitude above ground in meters, expressed as * 1000 (millimeters)
	
	return ((int32_t) (get_bytes(src, 18, 4)));
}


static inline int16_t pglobal_position_int_vx_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {//Ground X Speed (Latitude, positive north), expressed as m/s * 100
	
	return ((int16_t) (get_bytes(src, 22, 2)));
}


static inline int16_t pglobal_position_int_vy_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {//Ground Y Speed (Longitude, positive east), expressed as m/s * 100
	
	return ((int16_t) (get_bytes(src, 24, 2)));
}


static inline int16_t pglobal_position_int_vz_GET(pglobal_position_int_GLOBAL_POSITION_INT *src) {//Ground Z Speed (Altitude, positive down), expressed as m/s * 100
	
	return ((int16_t) (get_bytes(src, 26, 2)));
}


static inline int16_t pencapsulated_data_seqnr_GET(pencapsulated_data_ENCAPSULATED_DATA *src) {//sequence number (starting with 0 on every transmission)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pencapsulated_data_seqnr_SET(int16_t src, pencapsulated_data_ENCAPSULATED_DATA *dst) { //sequence number (starting with 0 on every transmission)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int8_t vencapsulated_data_daTa_GET(Vencapsulated_data_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vencapsulated_data_daTa pencapsulated_data_daTa_GET(pencapsulated_data_ENCAPSULATED_DATA *src) { //image data bytes
	
	return (Vencapsulated_data_daTa) {src + 2, 253};
}


static inline void vencapsulated_data_daTa_SET(int8_t src, size_t index, Vencapsulated_data_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vencapsulated_data_daTa pencapsulated_data_daTa_SET(const int8_t src[], pencapsulated_data_ENCAPSULATED_DATA *dst) {//image data bytes
	
	
	Vencapsulated_data_daTa ret = {dst + 2, 253};
	
	if (src)
		for (size_t i = 0; i < 253; i++)
			vencapsulated_data_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int16_t pgps_input_time_week_GET(pgps_input_GPS_INPUT *src) {//GPS week number
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pgps_input_time_week_SET(int16_t src, pgps_input_GPS_INPUT *dst) { //GPS week number
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int32_t pgps_input_time_week_ms_GET(pgps_input_GPS_INPUT *src) {//GPS time (milliseconds from start of GPS week)
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline void pgps_input_time_week_ms_SET(int32_t src, pgps_input_GPS_INPUT *dst) { //GPS time (milliseconds from start of GPS week)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 2);
}


static inline int64_t pgps_input_time_usec_GET(pgps_input_GPS_INPUT *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src->base.bytes, 6, 8)));
}


static inline void pgps_input_time_usec_SET(int64_t src, pgps_input_GPS_INPUT *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst->base.bytes, 6);
}


static inline int8_t pgps_input_gps_id_GET(pgps_input_GPS_INPUT *src) {//ID of the GPS for multiple GPS inputs
	
	return (int8_t) src->base.bytes[14];
}


static inline void pgps_input_gps_id_SET(int8_t src, pgps_input_GPS_INPUT *dst) { //ID of the GPS for multiple GPS inputs
	
	
	dst->base.bytes[14] = (uint8_t) (src);
}


static inline int8_t pgps_input_fix_type_GET(pgps_input_GPS_INPUT *src) {//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
	
	return (int8_t) src->base.bytes[15];
}


static inline void pgps_input_fix_type_SET(int8_t src, pgps_input_GPS_INPUT *dst) { //0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
	
	
	dst->base.bytes[15] = (uint8_t) (src);
}


static inline int32_t pgps_input_lat_GET(pgps_input_GPS_INPUT *src) {//Latitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 16, 4)));
}


static inline void pgps_input_lat_SET(int32_t src, pgps_input_GPS_INPUT *dst) { //Latitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 16);
}


static inline int32_t pgps_input_lon_GET(pgps_input_GPS_INPUT *src) {//Longitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 20, 4)));
}


static inline void pgps_input_lon_SET(int32_t src, pgps_input_GPS_INPUT *dst) { //Longitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 20);
}


static inline float pgps_input_alt_GET(pgps_input_GPS_INPUT *src) {//Altitude (AMSL, not WGS84), in m (positive for up)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline void pgps_input_alt_SET(float src, pgps_input_GPS_INPUT *dst) { //Altitude (AMSL, not WGS84), in m (positive for up)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 24);
}


static inline float pgps_input_hdop_GET(pgps_input_GPS_INPUT *src) {//GPS HDOP horizontal dilution of position in m
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline void pgps_input_hdop_SET(float src, pgps_input_GPS_INPUT *dst) { //GPS HDOP horizontal dilution of position in m
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 28);
}


static inline float pgps_input_vdop_GET(pgps_input_GPS_INPUT *src) {//GPS VDOP vertical dilution of position in m
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline void pgps_input_vdop_SET(float src, pgps_input_GPS_INPUT *dst) { //GPS VDOP vertical dilution of position in m
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 32);
}


static inline float pgps_input_vn_GET(pgps_input_GPS_INPUT *src) {//GPS velocity in m/s in NORTH direction in earth-fixed NED frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 36, 4)));
}


static inline void pgps_input_vn_SET(float src, pgps_input_GPS_INPUT *dst) { //GPS velocity in m/s in NORTH direction in earth-fixed NED frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 36);
}


static inline float pgps_input_ve_GET(pgps_input_GPS_INPUT *src) {//GPS velocity in m/s in EAST direction in earth-fixed NED frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 40, 4)));
}


static inline void pgps_input_ve_SET(float src, pgps_input_GPS_INPUT *dst) { //GPS velocity in m/s in EAST direction in earth-fixed NED frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 40);
}


static inline float pgps_input_vd_GET(pgps_input_GPS_INPUT *src) {//GPS velocity in m/s in DOWN direction in earth-fixed NED frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 44, 4)));
}


static inline void pgps_input_vd_SET(float src, pgps_input_GPS_INPUT *dst) { //GPS velocity in m/s in DOWN direction in earth-fixed NED frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 44);
}


static inline float pgps_input_speed_accuracy_GET(pgps_input_GPS_INPUT *src) {//GPS speed accuracy in m/s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 48, 4)));
}


static inline void pgps_input_speed_accuracy_SET(float src, pgps_input_GPS_INPUT *dst) { //GPS speed accuracy in m/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 48);
}


static inline float pgps_input_horiz_accuracy_GET(pgps_input_GPS_INPUT *src) {//GPS horizontal accuracy in m
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 52, 4)));
}


static inline void pgps_input_horiz_accuracy_SET(float src, pgps_input_GPS_INPUT *dst) { //GPS horizontal accuracy in m
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 52);
}


static inline float pgps_input_vert_accuracy_GET(pgps_input_GPS_INPUT *src) {//GPS vertical accuracy in m
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 56, 4)));
}


static inline void pgps_input_vert_accuracy_SET(float src, pgps_input_GPS_INPUT *dst) { //GPS vertical accuracy in m
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 56);
}


static inline int8_t pgps_input_satellites_visible_GET(pgps_input_GPS_INPUT *src) {//Number of satellites visible.
	
	return (int8_t) src->base.bytes[60];
}


static inline void pgps_input_satellites_visible_SET(int8_t src, pgps_input_GPS_INPUT *dst) { //Number of satellites visible.
	
	
	dst->base.bytes[60] = (uint8_t) (src);
}

static inline bool pgps_input_ignore_flags_GET(pgps_input_GPS_INPUT *const src, e_GPS_INPUT_IGNORE_FLAGS *ret) {
	if (src->base.field_bit != 488 && !set_field(src, 488, -1)) return false;
	*ret = _en_gps_input_ignore_flags(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pgps_input_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS src, pgps_input_GPS_INPUT *const dst) {
	
	if (dst->base.field_bit != 488) set_field(dst, 488, 0);
	
	
	UMAX id = _id_gps_input_ignore_flags(src);
	set_bits(id, 4, dst->base.bytes, dst->BIT);
}


static inline int8_t pcommand_long_target_system_GET(pcommand_long_COMMAND_LONG *src) {//System which should execute the command
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pcommand_long_target_component_GET(pcommand_long_COMMAND_LONG *src) {//Component which should execute the command, 0 for all components
	
	return (int8_t) src->base.bytes[1];
}


static inline int8_t pcommand_long_confirmation_GET(pcommand_long_COMMAND_LONG *src) {//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
	
	return (int8_t) src->base.bytes[2];
}


static inline float pcommand_long_param1_GET(pcommand_long_COMMAND_LONG *src) {//Parameter 1, as defined by MAV_CMD enum.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 3, 4)));
}


static inline float pcommand_long_param2_GET(pcommand_long_COMMAND_LONG *src) {//Parameter 2, as defined by MAV_CMD enum.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 7, 4)));
}


static inline float pcommand_long_param3_GET(pcommand_long_COMMAND_LONG *src) {//Parameter 3, as defined by MAV_CMD enum.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 11, 4)));
}


static inline float pcommand_long_param4_GET(pcommand_long_COMMAND_LONG *src) {//Parameter 4, as defined by MAV_CMD enum.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 15, 4)));
}


static inline float pcommand_long_param5_GET(pcommand_long_COMMAND_LONG *src) {//Parameter 5, as defined by MAV_CMD enum.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 19, 4)));
}


static inline float pcommand_long_param6_GET(pcommand_long_COMMAND_LONG *src) {//Parameter 6, as defined by MAV_CMD enum.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 23, 4)));
}


static inline float pcommand_long_param7_GET(pcommand_long_COMMAND_LONG *src) {//Parameter 7, as defined by MAV_CMD enum.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 27, 4)));
}

static inline bool pcommand_long_command_GET(pcommand_long_COMMAND_LONG *const src, e_MAV_CMD *ret) {
	if (src->base.field_bit != 250 && !set_field(src, 250, -1)) return false;
	*ret = _en_mav_cmd(get_bits(src->base.bytes, src->BIT, 8));
	return true;
}


static inline int16_t pcompassmot_status_throttle_GET(pcompassmot_status_COMPASSMOT_STATUS *src) {//throttle (percent*10)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pcompassmot_status_throttle_SET(int16_t src, pcompassmot_status_COMPASSMOT_STATUS *dst) { //throttle (percent*10)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t pcompassmot_status_interference_GET(pcompassmot_status_COMPASSMOT_STATUS *src) {//interference (percent)
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pcompassmot_status_interference_SET(int16_t src, pcompassmot_status_COMPASSMOT_STATUS *dst) { //interference (percent)
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline float pcompassmot_status_current_GET(pcompassmot_status_COMPASSMOT_STATUS *src) {//current (Ampere)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pcompassmot_status_current_SET(float src, pcompassmot_status_COMPASSMOT_STATUS *dst) { //current (Ampere)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pcompassmot_status_CompensationX_GET(pcompassmot_status_COMPASSMOT_STATUS *src) {//Motor Compensation X
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pcompassmot_status_CompensationX_SET(float src, pcompassmot_status_COMPASSMOT_STATUS *dst) { //Motor Compensation X
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pcompassmot_status_CompensationY_GET(pcompassmot_status_COMPASSMOT_STATUS *src) {//Motor Compensation Y
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pcompassmot_status_CompensationY_SET(float src, pcompassmot_status_COMPASSMOT_STATUS *dst) { //Motor Compensation Y
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float pcompassmot_status_CompensationZ_GET(pcompassmot_status_COMPASSMOT_STATUS *src) {//Motor Compensation Z
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void pcompassmot_status_CompensationZ_SET(float src, pcompassmot_status_COMPASSMOT_STATUS *dst) { //Motor Compensation Z
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline int16_t plog_request_data_id_GET(plog_request_data_LOG_REQUEST_DATA *src) {//Log id (from LOG_ENTRY reply)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void plog_request_data_id_SET(int16_t src, plog_request_data_LOG_REQUEST_DATA *dst) { //Log id (from LOG_ENTRY reply)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int32_t plog_request_data_ofs_GET(plog_request_data_LOG_REQUEST_DATA *src) {//Offset into the log
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline void plog_request_data_ofs_SET(int32_t src, plog_request_data_LOG_REQUEST_DATA *dst) { //Offset into the log
	
	
	set_bytes((uint32_t) (src), 4, dst, 2);
}


static inline int32_t plog_request_data_count_GET(plog_request_data_LOG_REQUEST_DATA *src) {//Number of bytes
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void plog_request_data_count_SET(int32_t src, plog_request_data_LOG_REQUEST_DATA *dst) { //Number of bytes
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int8_t plog_request_data_target_system_GET(plog_request_data_LOG_REQUEST_DATA *src) {//System ID
	
	return (int8_t) src[10];
}


static inline void plog_request_data_target_system_SET(int8_t src, plog_request_data_LOG_REQUEST_DATA *dst) { //System ID
	
	
	dst[10] = (uint8_t) (src);
}


static inline int8_t plog_request_data_target_component_GET(plog_request_data_LOG_REQUEST_DATA *src) {//Component ID
	
	return (int8_t) src[11];
}


static inline void plog_request_data_target_component_SET(int8_t src, plog_request_data_LOG_REQUEST_DATA *dst) { //Component ID
	
	
	dst[11] = (uint8_t) (src);
}


static inline int16_t pgps_raw_int_eph_GET(pgps_raw_int_GPS_RAW_INT *src) {//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int16_t pgps_raw_int_epv_GET(pgps_raw_int_GPS_RAW_INT *src) {//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline int16_t pgps_raw_int_vel_GET(pgps_raw_int_GPS_RAW_INT *src) {//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}

/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*					 unknown, set to: UINT16_MA */

static inline int16_t pgps_raw_int_cog_GET(pgps_raw_int_GPS_RAW_INT *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 6, 2)));
}


static inline int64_t pgps_raw_int_time_usec_GET(pgps_raw_int_GPS_RAW_INT *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src->base.bytes, 8, 8)));
}


static inline int32_t pgps_raw_int_lat_GET(pgps_raw_int_GPS_RAW_INT *src) {//Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 16, 4)));
}


static inline int32_t pgps_raw_int_lon_GET(pgps_raw_int_GPS_RAW_INT *src) {//Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 20, 4)));
}

/**
*Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
*					 the AMSL altitude in addition to the WGS84 altitude */

static inline int32_t pgps_raw_int_alt_GET(pgps_raw_int_GPS_RAW_INT *src) {
	
	return ((int32_t) (get_bytes(src->base.bytes, 24, 4)));
}


static inline int8_t pgps_raw_int_satellites_visible_GET(pgps_raw_int_GPS_RAW_INT *src) {//Number of satellites visible. If unknown, set to 255
	
	return (int8_t) src->base.bytes[28];
}

static inline bool pgps_raw_int_fix_type_GET(pgps_raw_int_GPS_RAW_INT *const src, e_GPS_FIX_TYPE *ret) {
	if (src->base.field_bit != 234 && !set_field(src, 234, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline bool pgps_raw_int_alt_ellipsoid_GET(pgps_raw_int_GPS_RAW_INT *const src, int32_t *ret) {
	if (src->base.field_bit != 235 && !set_field(src, 235, -1)) return false;
	*ret = ((int32_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline bool pgps_raw_int_h_acc_GET(pgps_raw_int_GPS_RAW_INT *const src, int32_t *ret) {
	if (src->base.field_bit != 236 && !set_field(src, 236, -1)) return false;
	*ret = ((int32_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline bool pgps_raw_int_v_acc_GET(pgps_raw_int_GPS_RAW_INT *const src, int32_t *ret) {
	if (src->base.field_bit != 237 && !set_field(src, 237, -1)) return false;
	*ret = ((int32_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline bool pgps_raw_int_vel_acc_GET(pgps_raw_int_GPS_RAW_INT *const src, int32_t *ret) {
	if (src->base.field_bit != 238 && !set_field(src, 238, -1)) return false;
	*ret = ((int32_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline bool pgps_raw_int_hdg_acc_GET(pgps_raw_int_GPS_RAW_INT *const src, int32_t *ret) {
	if (src->base.field_bit != 239 && !set_field(src, 239, -1)) return false;
	*ret = ((int32_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}


static inline int16_t pcamera_status_img_idx_GET(pcamera_status_CAMERA_STATUS *src) {//Image index
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pcamera_status_img_idx_SET(int16_t src, pcamera_status_CAMERA_STATUS *dst) { //Image index
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int64_t pcamera_status_time_usec_GET(pcamera_status_CAMERA_STATUS *src) {//Image timestamp (microseconds since UNIX epoch, according to camera clock)
	
	return ((int64_t) (get_bytes(src->base.bytes, 2, 8)));
}


static inline void pcamera_status_time_usec_SET(int64_t src, pcamera_status_CAMERA_STATUS *dst) { //Image timestamp (microseconds since UNIX epoch, according to camera clock)
	
	
	set_bytes((src), 8, dst->base.bytes, 2);
}


static inline int8_t pcamera_status_target_system_GET(pcamera_status_CAMERA_STATUS *src) {//System ID
	
	return (int8_t) src->base.bytes[10];
}


static inline void pcamera_status_target_system_SET(int8_t src, pcamera_status_CAMERA_STATUS *dst) { //System ID
	
	
	dst->base.bytes[10] = (uint8_t) (src);
}


static inline int8_t pcamera_status_cam_idx_GET(pcamera_status_CAMERA_STATUS *src) {//Camera ID
	
	return (int8_t) src->base.bytes[11];
}


static inline void pcamera_status_cam_idx_SET(int8_t src, pcamera_status_CAMERA_STATUS *dst) { //Camera ID
	
	
	dst->base.bytes[11] = (uint8_t) (src);
}


static inline float pcamera_status_p1_GET(pcamera_status_CAMERA_STATUS *src) {//Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void pcamera_status_p1_SET(float src, pcamera_status_CAMERA_STATUS *dst) { //Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}


static inline float pcamera_status_p2_GET(pcamera_status_CAMERA_STATUS *src) {//Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline void pcamera_status_p2_SET(float src, pcamera_status_CAMERA_STATUS *dst) { //Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 16);
}


static inline float pcamera_status_p3_GET(pcamera_status_CAMERA_STATUS *src) {//Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline void pcamera_status_p3_SET(float src, pcamera_status_CAMERA_STATUS *dst) { //Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 20);
}


static inline float pcamera_status_p4_GET(pcamera_status_CAMERA_STATUS *src) {//Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline void pcamera_status_p4_SET(float src, pcamera_status_CAMERA_STATUS *dst) { //Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 24);
}

static inline bool pcamera_status_event_id_GET(pcamera_status_CAMERA_STATUS *const src, e_CAMERA_STATUS_TYPES *ret) {
	if (src->base.field_bit != 224 && !set_field(src, 224, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pcamera_status_event_id_SET(e_CAMERA_STATUS_TYPES src, pcamera_status_CAMERA_STATUS *const dst) {
	
	if (dst->base.field_bit != 224) set_field(dst, 224, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}


static inline int32_t prc_channels_scaled_time_boot_ms_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}

/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
*					 8 servos */

static inline int8_t prc_channels_scaled_port_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {
	
	return (int8_t) src[4];
}


static inline int16_t prc_channels_scaled_chan1_scaled_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	
	return ((int16_t) (get_bytes(src, 5, 2)));
}


static inline int16_t prc_channels_scaled_chan2_scaled_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	
	return ((int16_t) (get_bytes(src, 7, 2)));
}


static inline int16_t prc_channels_scaled_chan3_scaled_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	
	return ((int16_t) (get_bytes(src, 9, 2)));
}


static inline int16_t prc_channels_scaled_chan4_scaled_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	
	return ((int16_t) (get_bytes(src, 11, 2)));
}


static inline int16_t prc_channels_scaled_chan5_scaled_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	
	return ((int16_t) (get_bytes(src, 13, 2)));
}


static inline int16_t prc_channels_scaled_chan6_scaled_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	
	return ((int16_t) (get_bytes(src, 15, 2)));
}


static inline int16_t prc_channels_scaled_chan7_scaled_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	
	return ((int16_t) (get_bytes(src, 17, 2)));
}


static inline int16_t prc_channels_scaled_chan8_scaled_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	
	return ((int16_t) (get_bytes(src, 19, 2)));
}


static inline int8_t prc_channels_scaled_rssi_GET(prc_channels_scaled_RC_CHANNELS_SCALED *src) {//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	
	return (int8_t) src[21];
}


static inline int32_t pcamera_settings_time_boot_ms_GET(pcamera_settings_CAMERA_SETTINGS *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void pcamera_settings_time_boot_ms_SET(int32_t src, pcamera_settings_CAMERA_SETTINGS *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}

static inline bool pcamera_settings_mode_id_GET(pcamera_settings_CAMERA_SETTINGS *const src, e_CAMERA_MODE *ret) {
	if (src->base.field_bit != 32 && !set_field(src, 32, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void pcamera_settings_mode_id_SET(e_CAMERA_MODE src, pcamera_settings_CAMERA_SETTINGS *const dst) {
	
	if (dst->base.field_bit != 32) set_field(dst, 32, 0);
	
	
	set_bits(src, 2, dst->base.bytes, dst->BIT);
}


static inline int32_t pdevice_op_read_reply_request_id_GET(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *src) {//request ID - copied from request
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pdevice_op_read_reply_request_id_SET(int32_t src, pdevice_op_read_reply_DEVICE_OP_READ_REPLY *dst) { //request ID - copied from request
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int8_t pdevice_op_read_reply_result_GET(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *src) {//0 for success, anything else is failure code
	
	return (int8_t) src[4];
}


static inline void pdevice_op_read_reply_result_SET(int8_t src, pdevice_op_read_reply_DEVICE_OP_READ_REPLY *dst) { //0 for success, anything else is failure code
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t pdevice_op_read_reply_regstart_GET(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *src) {//starting register
	
	return (int8_t) src[5];
}


static inline void pdevice_op_read_reply_regstart_SET(int8_t src, pdevice_op_read_reply_DEVICE_OP_READ_REPLY *dst) { //starting register
	
	
	dst[5] = (uint8_t) (src);
}


static inline int8_t pdevice_op_read_reply_count_GET(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *src) {//count of bytes read
	
	return (int8_t) src[6];
}


static inline void pdevice_op_read_reply_count_SET(int8_t src, pdevice_op_read_reply_DEVICE_OP_READ_REPLY *dst) { //count of bytes read
	
	
	dst[6] = (uint8_t) (src);
}


static inline int8_t vdevice_op_read_reply_daTa_GET(Vdevice_op_read_reply_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vdevice_op_read_reply_daTa pdevice_op_read_reply_daTa_GET(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *src) { //reply data
	
	return (Vdevice_op_read_reply_daTa) {src + 7, 128};
}


static inline void vdevice_op_read_reply_daTa_SET(int8_t src, size_t index, Vdevice_op_read_reply_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vdevice_op_read_reply_daTa pdevice_op_read_reply_daTa_SET(const int8_t src[], pdevice_op_read_reply_DEVICE_OP_READ_REPLY *dst) {//reply data
	
	
	Vdevice_op_read_reply_daTa ret = {dst + 7, 128};
	
	if (src)
		for (size_t i = 0; i < 128; i++)
			vdevice_op_read_reply_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int64_t praw_pressure_time_usec_GET(praw_pressure_RAW_PRESSURE *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline int16_t praw_pressure_press_abs_GET(praw_pressure_RAW_PRESSURE *src) {//Absolute pressure (raw)
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline int16_t praw_pressure_press_diff1_GET(praw_pressure_RAW_PRESSURE *src) {//Differential pressure 1 (raw, 0 if nonexistant)
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline int16_t praw_pressure_press_diff2_GET(praw_pressure_RAW_PRESSURE *src) {//Differential pressure 2 (raw, 0 if nonexistant)
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline int16_t praw_pressure_temperature_GET(praw_pressure_RAW_PRESSURE *src) {//Raw Temperature measurement (raw)
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline int8_t pdigicam_control_target_system_GET(pdigicam_control_DIGICAM_CONTROL *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pdigicam_control_target_system_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pdigicam_control_target_component_GET(pdigicam_control_DIGICAM_CONTROL *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pdigicam_control_target_component_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t pdigicam_control_session_GET(pdigicam_control_DIGICAM_CONTROL *src) {//0: stop, 1: start or keep it up Session control e.g. show/hide lens
	
	return (int8_t) src[2];
}


static inline void pdigicam_control_session_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) { //0: stop, 1: start or keep it up Session control e.g. show/hide lens
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t pdigicam_control_zoom_pos_GET(pdigicam_control_DIGICAM_CONTROL *src) {//1 to N Zoom's absolute position (0 means ignore)
	
	return (int8_t) src[3];
}


static inline void pdigicam_control_zoom_pos_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) { //1 to N Zoom's absolute position (0 means ignore)
	
	
	dst[3] = (uint8_t) (src);
}


static inline int8_t pdigicam_control_zoom_step_GET(pdigicam_control_DIGICAM_CONTROL *src) {//-100 to 100 Zooming step value to offset zoom from the current position
	
	return (int8_t) src[4];
}


static inline void pdigicam_control_zoom_step_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) { //-100 to 100 Zooming step value to offset zoom from the current position
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t pdigicam_control_focus_lock_GET(pdigicam_control_DIGICAM_CONTROL *src) {//0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
	
	return (int8_t) src[5];
}


static inline void pdigicam_control_focus_lock_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) { //0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
	
	
	dst[5] = (uint8_t) (src);
}


static inline int8_t pdigicam_control_shot_GET(pdigicam_control_DIGICAM_CONTROL *src) {//0: ignore, 1: shot or start filming
	
	return (int8_t) src[6];
}


static inline void pdigicam_control_shot_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) { //0: ignore, 1: shot or start filming
	
	
	dst[6] = (uint8_t) (src);
}

/**
*Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
*					 just onc */

static inline int8_t pdigicam_control_command_id_GET(pdigicam_control_DIGICAM_CONTROL *src) {
	
	return (int8_t) src[7];
}

/**
*Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
*					 just onc */

static inline void pdigicam_control_command_id_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) {
	
	
	dst[7] = (uint8_t) (src);
}


static inline int8_t pdigicam_control_extra_param_GET(pdigicam_control_DIGICAM_CONTROL *src) {//Extra parameters enumeration (0 means ignore)
	
	return (int8_t) src[8];
}


static inline void pdigicam_control_extra_param_SET(int8_t src, pdigicam_control_DIGICAM_CONTROL *dst) { //Extra parameters enumeration (0 means ignore)
	
	
	dst[8] = (uint8_t) (src);
}


static inline float pdigicam_control_extra_value_GET(pdigicam_control_DIGICAM_CONTROL *src) {//Correspondent value to given extra_param
	
	return (intBitsToFloat(get_bytes(src, 9, 4)));
}


static inline void pdigicam_control_extra_value_SET(float src, pdigicam_control_DIGICAM_CONTROL *dst) { //Correspondent value to given extra_param
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 9);
}


static inline int32_t pnamed_value_float_time_boot_ms_GET(pnamed_value_float_NAMED_VALUE_FLOAT *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void pnamed_value_float_time_boot_ms_SET(int32_t src, pnamed_value_float_NAMED_VALUE_FLOAT *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline float pnamed_value_float_value_GET(pnamed_value_float_NAMED_VALUE_FLOAT *src) {//Floating point value
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 4, 4)));
}


static inline void pnamed_value_float_value_SET(float src, pnamed_value_float_NAMED_VALUE_FLOAT *dst) { //Floating point value
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 4);
}

static inline bool pnamed_value_float_name_GET(pnamed_value_float_NAMED_VALUE_FLOAT *const src, Vnamed_value_float_name *ret) {//Name of the debug variable
	
	if (src->base.field_bit != 66 && !set_field(src, 66, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vnamed_value_float_name pnamed_value_float_name_SET(const char src[], size_t len, pnamed_value_float_NAMED_VALUE_FLOAT *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 66, len);
	
	Vnamed_value_float_name ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pgopro_heartbeat_status_GET(pgopro_heartbeat_GOPRO_HEARTBEAT *const src, e_GOPRO_HEARTBEAT_STATUS *ret) {
	if (src->base.field_bit != 2 && !set_field(src, 2, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void pgopro_heartbeat_status_SET(e_GOPRO_HEARTBEAT_STATUS src, pgopro_heartbeat_GOPRO_HEARTBEAT *const dst) {
	
	if (dst->base.field_bit != 2) set_field(dst, 2, 0);
	
	
	set_bits(src, 2, dst->base.bytes, dst->BIT);
}

static inline bool pgopro_heartbeat_capture_mode_GET(pgopro_heartbeat_GOPRO_HEARTBEAT *const src, e_GOPRO_CAPTURE_MODE *ret) {
	if (src->base.field_bit != 3 && !set_field(src, 3, -1)) return false;
	*ret = _en_gopro_capture_mode(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pgopro_heartbeat_capture_mode_SET(e_GOPRO_CAPTURE_MODE src, pgopro_heartbeat_GOPRO_HEARTBEAT *const dst) {
	
	if (dst->base.field_bit != 3) set_field(dst, 3, 0);
	
	
	UMAX id = _id_gopro_capture_mode(src);
	set_bits(id, 4, dst->base.bytes, dst->BIT);
}

static inline bool pgopro_heartbeat_flags_GET(pgopro_heartbeat_GOPRO_HEARTBEAT *const src, e_GOPRO_HEARTBEAT_FLAGS *ret) {
	if (src->base.field_bit != 4 && !set_field(src, 4, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void pgopro_heartbeat_flags_SET(e_GOPRO_HEARTBEAT_FLAGS src, pgopro_heartbeat_GOPRO_HEARTBEAT *const dst) {
	
	if (dst->base.field_bit != 4) set_field(dst, 4, 0);
	
	
	set_bits(-1
	src, 1, dst->base.bytes, dst->BIT);
}


static inline int32_t pattitude_time_boot_ms_GET(pattitude_ATTITUDE *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline float pattitude_roll_GET(pattitude_ATTITUDE *src) {//Roll angle (rad, -pi..+pi)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline float pattitude_pitch_GET(pattitude_ATTITUDE *src) {//Pitch angle (rad, -pi..+pi)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline float pattitude_yaw_GET(pattitude_ATTITUDE *src) {//Yaw angle (rad, -pi..+pi)
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float pattitude_rollspeed_GET(pattitude_ATTITUDE *src) {//Roll angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline float pattitude_pitchspeed_GET(pattitude_ATTITUDE *src) {//Pitch angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline float pattitude_yawspeed_GET(pattitude_ATTITUDE *src) {//Yaw angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline int8_t pmission_write_partial_list_target_system_GET(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pmission_write_partial_list_target_component_GET(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline int16_t pmission_write_partial_list_start_index_GET(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *src) {//Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline int16_t pmission_write_partial_list_end_index_GET(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *src) {//End index, equal or greater than start index.
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}

static inline bool pmission_write_partial_list_mission_type_GET(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 48 && !set_field(src, 48, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline float pahrs2_roll_GET(pahrs2_AHRS2 *src) {//Roll angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void pahrs2_roll_SET(float src, pahrs2_AHRS2 *dst) { //Roll angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float pahrs2_pitch_GET(pahrs2_AHRS2 *src) {//Pitch angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pahrs2_pitch_SET(float src, pahrs2_AHRS2 *dst) { //Pitch angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pahrs2_yaw_GET(pahrs2_AHRS2 *src) {//Yaw angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pahrs2_yaw_SET(float src, pahrs2_AHRS2 *dst) { //Yaw angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pahrs2_altitude_GET(pahrs2_AHRS2 *src) {//Altitude (MSL)
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pahrs2_altitude_SET(float src, pahrs2_AHRS2 *dst) { //Altitude (MSL)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline int32_t pahrs2_lat_GET(pahrs2_AHRS2 *src) {//Latitude in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 16, 4)));
}


static inline void pahrs2_lat_SET(int32_t src, pahrs2_AHRS2 *dst) { //Latitude in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 16);
}


static inline int32_t pahrs2_lng_GET(pahrs2_AHRS2 *src) {//Longitude in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 20, 4)));
}


static inline void pahrs2_lng_SET(int32_t src, pahrs2_AHRS2 *dst) { //Longitude in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 20);
}


static inline int8_t plog_erase_target_system_GET(plog_erase_LOG_ERASE *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void plog_erase_target_system_SET(int8_t src, plog_erase_LOG_ERASE *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t plog_erase_target_component_GET(plog_erase_LOG_ERASE *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void plog_erase_target_component_SET(int8_t src, plog_erase_LOG_ERASE *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int16_t pterrain_request_grid_spacing_GET(pterrain_request_TERRAIN_REQUEST *src) {//Grid spacing in meters
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pterrain_request_grid_spacing_SET(int16_t src, pterrain_request_TERRAIN_REQUEST *dst) { //Grid spacing in meters
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int64_t pterrain_request_mask_GET(pterrain_request_TERRAIN_REQUEST *src) {//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
	
	return ((int64_t) (get_bytes(src, 2, 8)));
}


static inline void pterrain_request_mask_SET(int64_t src, pterrain_request_TERRAIN_REQUEST *dst) { //Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
	
	
	set_bytes((src), 8, dst, 2);
}


static inline int32_t pterrain_request_lat_GET(pterrain_request_TERRAIN_REQUEST *src) {//Latitude of SW corner of first grid (degrees *10^7)
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}


static inline void pterrain_request_lat_SET(int32_t src, pterrain_request_TERRAIN_REQUEST *dst) { //Latitude of SW corner of first grid (degrees *10^7)
	
	
	set_bytes((uint32_t) (src), 4, dst, 10);
}


static inline int32_t pterrain_request_lon_GET(pterrain_request_TERRAIN_REQUEST *src) {//Longitude of SW corner of first grid (in degrees *10^7)
	
	return ((int32_t) (get_bytes(src, 14, 4)));
}


static inline void pterrain_request_lon_SET(int32_t src, pterrain_request_TERRAIN_REQUEST *dst) { //Longitude of SW corner of first grid (in degrees *10^7)
	
	
	set_bytes((uint32_t) (src), 4, dst, 14);
}


static inline int8_t pmount_status_target_system_GET(pmount_status_MOUNT_STATUS *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pmount_status_target_system_SET(int8_t src, pmount_status_MOUNT_STATUS *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pmount_status_target_component_GET(pmount_status_MOUNT_STATUS *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pmount_status_target_component_SET(int8_t src, pmount_status_MOUNT_STATUS *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int32_t pmount_status_pointing_a_GET(pmount_status_MOUNT_STATUS *src) {//pitch(deg*100)
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline void pmount_status_pointing_a_SET(int32_t src, pmount_status_MOUNT_STATUS *dst) { //pitch(deg*100)
	
	
	set_bytes((uint32_t) (src), 4, dst, 2);
}


static inline int32_t pmount_status_pointing_b_GET(pmount_status_MOUNT_STATUS *src) {//roll(deg*100)
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void pmount_status_pointing_b_SET(int32_t src, pmount_status_MOUNT_STATUS *dst) { //roll(deg*100)
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int32_t pmount_status_pointing_c_GET(pmount_status_MOUNT_STATUS *src) {//yaw(deg*100)
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}


static inline void pmount_status_pointing_c_SET(int32_t src, pmount_status_MOUNT_STATUS *dst) { //yaw(deg*100)
	
	
	set_bytes((uint32_t) (src), 4, dst, 10);
}


static inline int32_t pmanual_setpoint_time_boot_ms_GET(pmanual_setpoint_MANUAL_SETPOINT *src) {//Timestamp in milliseconds since system boot
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline float pmanual_setpoint_roll_GET(pmanual_setpoint_MANUAL_SETPOINT *src) {//Desired roll rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline float pmanual_setpoint_pitch_GET(pmanual_setpoint_MANUAL_SETPOINT *src) {//Desired pitch rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline float pmanual_setpoint_yaw_GET(pmanual_setpoint_MANUAL_SETPOINT *src) {//Desired yaw rate in radians per second
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float pmanual_setpoint_thrust_GET(pmanual_setpoint_MANUAL_SETPOINT *src) {//Collective thrust, normalized to 0 .. 1
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline int8_t pmanual_setpoint_mode_switch_GET(pmanual_setpoint_MANUAL_SETPOINT *src) {//Flight mode switch position, 0.. 255
	
	return (int8_t) src[20];
}


static inline int8_t pmanual_setpoint_manual_override_switch_GET(pmanual_setpoint_MANUAL_SETPOINT *src) {//Override mode switch position, 0.. 255
	
	return (int8_t) src[21];
}


static inline float ppid_tuning_desired_GET(ppid_tuning_PID_TUNING *src) {//desired rate (degrees/s)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 0, 4)));
}


static inline void ppid_tuning_desired_SET(float src, ppid_tuning_PID_TUNING *dst) { //desired rate (degrees/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 0);
}


static inline float ppid_tuning_achieved_GET(ppid_tuning_PID_TUNING *src) {//achieved rate (degrees/s)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 4, 4)));
}


static inline void ppid_tuning_achieved_SET(float src, ppid_tuning_PID_TUNING *dst) { //achieved rate (degrees/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 4);
}


static inline float ppid_tuning_FF_GET(ppid_tuning_PID_TUNING *src) {//FF component
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline void ppid_tuning_FF_SET(float src, ppid_tuning_PID_TUNING *dst) { //FF component
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 8);
}


static inline float ppid_tuning_P_GET(ppid_tuning_PID_TUNING *src) {//P component
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void ppid_tuning_P_SET(float src, ppid_tuning_PID_TUNING *dst) { //P component
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}


static inline float ppid_tuning_I_GET(ppid_tuning_PID_TUNING *src) {//I component
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline void ppid_tuning_I_SET(float src, ppid_tuning_PID_TUNING *dst) { //I component
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 16);
}


static inline float ppid_tuning_D_GET(ppid_tuning_PID_TUNING *src) {//D component
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline void ppid_tuning_D_SET(float src, ppid_tuning_PID_TUNING *dst) { //D component
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 20);
}

static inline bool ppid_tuning_axis_GET(ppid_tuning_PID_TUNING *const src, e_PID_TUNING_AXIS *ret) {
	if (src->base.field_bit != 192 && !set_field(src, 192, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void ppid_tuning_axis_SET(e_PID_TUNING_AXIS src, ppid_tuning_PID_TUNING *const dst) {
	
	if (dst->base.field_bit != 192) set_field(dst, 192, 0);
	
	
	set_bits(-1
	src, 3, dst->base.bytes, dst->BIT);
}


static inline float psafety_allowed_area_p1x_GET(psafety_allowed_area_SAFETY_ALLOWED_AREA *src) {//x position 1 / Latitude 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 0, 4)));
}


static inline float psafety_allowed_area_p1y_GET(psafety_allowed_area_SAFETY_ALLOWED_AREA *src) {//y position 1 / Longitude 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 4, 4)));
}


static inline float psafety_allowed_area_p1z_GET(psafety_allowed_area_SAFETY_ALLOWED_AREA *src) {//z position 1 / Altitude 1
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline float psafety_allowed_area_p2x_GET(psafety_allowed_area_SAFETY_ALLOWED_AREA *src) {//x position 2 / Latitude 2
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline float psafety_allowed_area_p2y_GET(psafety_allowed_area_SAFETY_ALLOWED_AREA *src) {//y position 2 / Longitude 2
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline float psafety_allowed_area_p2z_GET(psafety_allowed_area_SAFETY_ALLOWED_AREA *src) {//z position 2 / Altitude 2
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}

static inline bool psafety_allowed_area_frame_GET(psafety_allowed_area_SAFETY_ALLOWED_AREA *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 192 && !set_field(src, 192, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

/**
*Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
*					 average flow. The integration time also indicates the */

static inline int32_t poptical_flow_rad_integration_time_us_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}

/**
*Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
*					 average flow. The integration time also indicates the */

static inline void poptical_flow_rad_integration_time_us_SET(int32_t src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) {
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int32_t poptical_flow_rad_time_delta_distance_us_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {//Time in microseconds since the distance was sampled.
	
	return ((int32_t) (get_bytes(src, 4, 4)));
}


static inline void poptical_flow_rad_time_delta_distance_us_SET(int32_t src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) { //Time in microseconds since the distance was sampled.
	
	
	set_bytes((uint32_t) (src), 4, dst, 4);
}


static inline int64_t poptical_flow_rad_time_usec_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 8, 8)));
}


static inline void poptical_flow_rad_time_usec_SET(int64_t src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) { //Timestamp (microseconds, synced to UNIX time or since system boot)
	
	
	set_bytes((src), 8, dst, 8);
}


static inline int8_t poptical_flow_rad_sensor_id_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {//Sensor ID
	
	return (int8_t) src[16];
}


static inline void poptical_flow_rad_sensor_id_SET(int8_t src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) { //Sensor ID
	
	
	dst[16] = (uint8_t) (src);
}

/**
*Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
*					 motion along the positive Y axis induces a negative flow. */

static inline float poptical_flow_rad_integrated_x_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {
	
	return (intBitsToFloat(get_bytes(src, 17, 4)));
}

/**
*Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
*					 motion along the positive Y axis induces a negative flow. */

static inline void poptical_flow_rad_integrated_x_SET(float src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 17);
}

/**
*Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
*					 motion along the positive X axis induces a positive flow. */

static inline float poptical_flow_rad_integrated_y_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {
	
	return (intBitsToFloat(get_bytes(src, 21, 4)));
}

/**
*Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
*					 motion along the positive X axis induces a positive flow. */

static inline void poptical_flow_rad_integrated_y_SET(float src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 21);
}


static inline float poptical_flow_rad_integrated_xgyro_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {//RH rotation around X axis (rad)
	
	return (intBitsToFloat(get_bytes(src, 25, 4)));
}


static inline void poptical_flow_rad_integrated_xgyro_SET(float src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) { //RH rotation around X axis (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 25);
}


static inline float poptical_flow_rad_integrated_ygyro_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {//RH rotation around Y axis (rad)
	
	return (intBitsToFloat(get_bytes(src, 29, 4)));
}


static inline void poptical_flow_rad_integrated_ygyro_SET(float src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) { //RH rotation around Y axis (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 29);
}


static inline float poptical_flow_rad_integrated_zgyro_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {//RH rotation around Z axis (rad)
	
	return (intBitsToFloat(get_bytes(src, 33, 4)));
}


static inline void poptical_flow_rad_integrated_zgyro_SET(float src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) { //RH rotation around Z axis (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 33);
}


static inline int16_t poptical_flow_rad_temperature_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {//Temperature * 100 in centi-degrees Celsius
	
	return ((int16_t) (get_bytes(src, 37, 2)));
}


static inline void poptical_flow_rad_temperature_SET(int16_t src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) { //Temperature * 100 in centi-degrees Celsius
	
	
	set_bytes((uint16_t) (src), 2, dst, 37);
}


static inline int8_t poptical_flow_rad_quality_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	
	return (int8_t) src[39];
}


static inline void poptical_flow_rad_quality_SET(int8_t src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) { //Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	
	
	dst[39] = (uint8_t) (src);
}

/**
*Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
*					 value: Unknown distance */

static inline float poptical_flow_rad_distance_GET(poptical_flow_rad_OPTICAL_FLOW_RAD *src) {
	
	return (intBitsToFloat(get_bytes(src, 40, 4)));
}

/**
*Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
*					 value: Unknown distance */

static inline void poptical_flow_rad_distance_SET(float src, poptical_flow_rad_OPTICAL_FLOW_RAD *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 40);
}


static inline int16_t plog_data_id_GET(plog_data_LOG_DATA *src) {//Log id (from LOG_ENTRY reply)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void plog_data_id_SET(int16_t src, plog_data_LOG_DATA *dst) { //Log id (from LOG_ENTRY reply)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int32_t plog_data_ofs_GET(plog_data_LOG_DATA *src) {//Offset into the log
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline void plog_data_ofs_SET(int32_t src, plog_data_LOG_DATA *dst) { //Offset into the log
	
	
	set_bytes((uint32_t) (src), 4, dst, 2);
}


static inline int8_t plog_data_count_GET(plog_data_LOG_DATA *src) {//Number of bytes (zero for end of log)
	
	return (int8_t) src[6];
}


static inline void plog_data_count_SET(int8_t src, plog_data_LOG_DATA *dst) { //Number of bytes (zero for end of log)
	
	
	dst[6] = (uint8_t) (src);
}


static inline int8_t vlog_data_daTa_GET(Vlog_data_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vlog_data_daTa plog_data_daTa_GET(plog_data_LOG_DATA *src) { //log data
	
	return (Vlog_data_daTa) {src + 7, 90};
}


static inline void vlog_data_daTa_SET(int8_t src, size_t index, Vlog_data_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vlog_data_daTa plog_data_daTa_SET(const int8_t src[], plog_data_LOG_DATA *dst) {//log data
	
	
	Vlog_data_daTa ret = {dst + 7, 90};
	
	if (src)
		for (size_t i = 0; i < 90; i++)
			vlog_data_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t pmission_clear_all_target_system_GET(pmission_clear_all_MISSION_CLEAR_ALL *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pmission_clear_all_target_component_GET(pmission_clear_all_MISSION_CLEAR_ALL *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}

static inline bool pmission_clear_all_mission_type_GET(pmission_clear_all_MISSION_CLEAR_ALL *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 16 && !set_field(src, 16, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline float pahrs3_roll_GET(pahrs3_AHRS3 *src) {//Roll angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void pahrs3_roll_SET(float src, pahrs3_AHRS3 *dst) { //Roll angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float pahrs3_pitch_GET(pahrs3_AHRS3 *src) {//Pitch angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pahrs3_pitch_SET(float src, pahrs3_AHRS3 *dst) { //Pitch angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pahrs3_yaw_GET(pahrs3_AHRS3 *src) {//Yaw angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pahrs3_yaw_SET(float src, pahrs3_AHRS3 *dst) { //Yaw angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pahrs3_altitude_GET(pahrs3_AHRS3 *src) {//Altitude (MSL)
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pahrs3_altitude_SET(float src, pahrs3_AHRS3 *dst) { //Altitude (MSL)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline int32_t pahrs3_lat_GET(pahrs3_AHRS3 *src) {//Latitude in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 16, 4)));
}


static inline void pahrs3_lat_SET(int32_t src, pahrs3_AHRS3 *dst) { //Latitude in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 16);
}


static inline int32_t pahrs3_lng_GET(pahrs3_AHRS3 *src) {//Longitude in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 20, 4)));
}


static inline void pahrs3_lng_SET(int32_t src, pahrs3_AHRS3 *dst) { //Longitude in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 20);
}


static inline float pahrs3_v1_GET(pahrs3_AHRS3 *src) {//test variable1
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void pahrs3_v1_SET(float src, pahrs3_AHRS3 *dst) { //test variable1
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float pahrs3_v2_GET(pahrs3_AHRS3 *src) {//test variable2
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void pahrs3_v2_SET(float src, pahrs3_AHRS3 *dst) { //test variable2
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float pahrs3_v3_GET(pahrs3_AHRS3 *src) {//test variable3
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void pahrs3_v3_SET(float src, pahrs3_AHRS3 *dst) { //test variable3
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}


static inline float pahrs3_v4_GET(pahrs3_AHRS3 *src) {//test variable4
	
	return (intBitsToFloat(get_bytes(src, 36, 4)));
}


static inline void pahrs3_v4_SET(float src, pahrs3_AHRS3 *dst) { //test variable4
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 36);
}


static inline int64_t pvicon_position_estimate_usec_GET(pvicon_position_estimate_VICON_POSITION_ESTIMATE *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void pvicon_position_estimate_usec_SET(int64_t src, pvicon_position_estimate_VICON_POSITION_ESTIMATE *dst) { //Timestamp (microseconds, synced to UNIX time or since system boot)
	
	
	set_bytes((src), 8, dst, 0);
}


static inline float pvicon_position_estimate_x_GET(pvicon_position_estimate_VICON_POSITION_ESTIMATE *src) {//Global X position
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pvicon_position_estimate_x_SET(float src, pvicon_position_estimate_VICON_POSITION_ESTIMATE *dst) { //Global X position
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pvicon_position_estimate_y_GET(pvicon_position_estimate_VICON_POSITION_ESTIMATE *src) {//Global Y position
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pvicon_position_estimate_y_SET(float src, pvicon_position_estimate_VICON_POSITION_ESTIMATE *dst) { //Global Y position
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float pvicon_position_estimate_z_GET(pvicon_position_estimate_VICON_POSITION_ESTIMATE *src) {//Global Z position
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void pvicon_position_estimate_z_SET(float src, pvicon_position_estimate_VICON_POSITION_ESTIMATE *dst) { //Global Z position
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float pvicon_position_estimate_roll_GET(pvicon_position_estimate_VICON_POSITION_ESTIMATE *src) {//Roll angle in rad
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void pvicon_position_estimate_roll_SET(float src, pvicon_position_estimate_VICON_POSITION_ESTIMATE *dst) { //Roll angle in rad
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float pvicon_position_estimate_pitch_GET(pvicon_position_estimate_VICON_POSITION_ESTIMATE *src) {//Pitch angle in rad
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void pvicon_position_estimate_pitch_SET(float src, pvicon_position_estimate_VICON_POSITION_ESTIMATE *dst) { //Pitch angle in rad
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float pvicon_position_estimate_yaw_GET(pvicon_position_estimate_VICON_POSITION_ESTIMATE *src) {//Yaw angle in rad
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void pvicon_position_estimate_yaw_SET(float src, pvicon_position_estimate_VICON_POSITION_ESTIMATE *dst) { //Yaw angle in rad
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline int16_t pgps2_rtk_wn_GET(pgps2_rtk_GPS2_RTK *src) {//GPS Week Number of last baseline
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pgps2_rtk_wn_SET(int16_t src, pgps2_rtk_GPS2_RTK *dst) { //GPS Week Number of last baseline
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int32_t pgps2_rtk_time_last_baseline_ms_GET(pgps2_rtk_GPS2_RTK *src) {//Time since boot of last baseline message received in ms.
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline void pgps2_rtk_time_last_baseline_ms_SET(int32_t src, pgps2_rtk_GPS2_RTK *dst) { //Time since boot of last baseline message received in ms.
	
	
	set_bytes((uint32_t) (src), 4, dst, 2);
}


static inline int32_t pgps2_rtk_tow_GET(pgps2_rtk_GPS2_RTK *src) {//GPS Time of Week of last baseline
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void pgps2_rtk_tow_SET(int32_t src, pgps2_rtk_GPS2_RTK *dst) { //GPS Time of Week of last baseline
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int32_t pgps2_rtk_accuracy_GET(pgps2_rtk_GPS2_RTK *src) {//Current estimate of baseline accuracy.
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}


static inline void pgps2_rtk_accuracy_SET(int32_t src, pgps2_rtk_GPS2_RTK *dst) { //Current estimate of baseline accuracy.
	
	
	set_bytes((uint32_t) (src), 4, dst, 10);
}


static inline int8_t pgps2_rtk_rtk_receiver_id_GET(pgps2_rtk_GPS2_RTK *src) {//Identification of connected RTK receiver.
	
	return (int8_t) src[14];
}


static inline void pgps2_rtk_rtk_receiver_id_SET(int8_t src, pgps2_rtk_GPS2_RTK *dst) { //Identification of connected RTK receiver.
	
	
	dst[14] = (uint8_t) (src);
}


static inline int8_t pgps2_rtk_rtk_health_GET(pgps2_rtk_GPS2_RTK *src) {//GPS-specific health report for RTK data.
	
	return (int8_t) src[15];
}


static inline void pgps2_rtk_rtk_health_SET(int8_t src, pgps2_rtk_GPS2_RTK *dst) { //GPS-specific health report for RTK data.
	
	
	dst[15] = (uint8_t) (src);
}


static inline int8_t pgps2_rtk_rtk_rate_GET(pgps2_rtk_GPS2_RTK *src) {//Rate of baseline messages being received by GPS, in HZ
	
	return (int8_t) src[16];
}


static inline void pgps2_rtk_rtk_rate_SET(int8_t src, pgps2_rtk_GPS2_RTK *dst) { //Rate of baseline messages being received by GPS, in HZ
	
	
	dst[16] = (uint8_t) (src);
}


static inline int8_t pgps2_rtk_nsats_GET(pgps2_rtk_GPS2_RTK *src) {//Current number of sats used for RTK calculation.
	
	return (int8_t) src[17];
}


static inline void pgps2_rtk_nsats_SET(int8_t src, pgps2_rtk_GPS2_RTK *dst) { //Current number of sats used for RTK calculation.
	
	
	dst[17] = (uint8_t) (src);
}


static inline int8_t pgps2_rtk_baseline_coords_type_GET(pgps2_rtk_GPS2_RTK *src) {//Coordinate system of baseline. 0 == ECEF, 1 == NED
	
	return (int8_t) src[18];
}


static inline void pgps2_rtk_baseline_coords_type_SET(int8_t src, pgps2_rtk_GPS2_RTK *dst) { //Coordinate system of baseline. 0 == ECEF, 1 == NED
	
	
	dst[18] = (uint8_t) (src);
}


static inline int32_t pgps2_rtk_baseline_a_mm_GET(pgps2_rtk_GPS2_RTK *src) {//Current baseline in ECEF x or NED north component in mm.
	
	return ((int32_t) (get_bytes(src, 19, 4)));
}


static inline void pgps2_rtk_baseline_a_mm_SET(int32_t src, pgps2_rtk_GPS2_RTK *dst) { //Current baseline in ECEF x or NED north component in mm.
	
	
	set_bytes((uint32_t) (src), 4, dst, 19);
}


static inline int32_t pgps2_rtk_baseline_b_mm_GET(pgps2_rtk_GPS2_RTK *src) {//Current baseline in ECEF y or NED east component in mm.
	
	return ((int32_t) (get_bytes(src, 23, 4)));
}


static inline void pgps2_rtk_baseline_b_mm_SET(int32_t src, pgps2_rtk_GPS2_RTK *dst) { //Current baseline in ECEF y or NED east component in mm.
	
	
	set_bytes((uint32_t) (src), 4, dst, 23);
}


static inline int32_t pgps2_rtk_baseline_c_mm_GET(pgps2_rtk_GPS2_RTK *src) {//Current baseline in ECEF z or NED down component in mm.
	
	return ((int32_t) (get_bytes(src, 27, 4)));
}


static inline void pgps2_rtk_baseline_c_mm_SET(int32_t src, pgps2_rtk_GPS2_RTK *dst) { //Current baseline in ECEF z or NED down component in mm.
	
	
	set_bytes((uint32_t) (src), 4, dst, 27);
}


static inline int32_t pgps2_rtk_iar_num_hypotheses_GET(pgps2_rtk_GPS2_RTK *src) {//Current number of integer ambiguity hypotheses.
	
	return ((int32_t) (get_bytes(src, 31, 4)));
}


static inline void pgps2_rtk_iar_num_hypotheses_SET(int32_t src, pgps2_rtk_GPS2_RTK *dst) { //Current number of integer ambiguity hypotheses.
	
	
	set_bytes((uint32_t) (src), 4, dst, 31);
}


static inline int8_t pmag_cal_report_compass_id_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//Compass being calibrated
	
	return (int8_t) src->base.bytes[0];
}


static inline void pmag_cal_report_compass_id_SET(int8_t src, pmag_cal_report_MAG_CAL_REPORT *dst) { //Compass being calibrated
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t pmag_cal_report_cal_mask_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//Bitmask of compasses being calibrated
	
	return (int8_t) src->base.bytes[1];
}


static inline void pmag_cal_report_cal_mask_SET(int8_t src, pmag_cal_report_MAG_CAL_REPORT *dst) { //Bitmask of compasses being calibrated
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}


static inline int8_t pmag_cal_report_autosaved_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
	
	return (int8_t) src->base.bytes[2];
}


static inline void pmag_cal_report_autosaved_SET(int8_t src, pmag_cal_report_MAG_CAL_REPORT *dst) { //0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
	
	
	dst->base.bytes[2] = (uint8_t) (src);
}


static inline float pmag_cal_report_fitness_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//RMS milligauss residuals
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 3, 4)));
}


static inline void pmag_cal_report_fitness_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //RMS milligauss residuals
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 3);
}


static inline float pmag_cal_report_ofs_x_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//X offset
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 7, 4)));
}


static inline void pmag_cal_report_ofs_x_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //X offset
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 7);
}


static inline float pmag_cal_report_ofs_y_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//Y offset
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 11, 4)));
}


static inline void pmag_cal_report_ofs_y_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //Y offset
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 11);
}


static inline float pmag_cal_report_ofs_z_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//Z offset
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 15, 4)));
}


static inline void pmag_cal_report_ofs_z_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //Z offset
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 15);
}


static inline float pmag_cal_report_diag_x_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//X diagonal (matrix 11)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 19, 4)));
}


static inline void pmag_cal_report_diag_x_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //X diagonal (matrix 11)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 19);
}


static inline float pmag_cal_report_diag_y_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//Y diagonal (matrix 22)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 23, 4)));
}


static inline void pmag_cal_report_diag_y_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //Y diagonal (matrix 22)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 23);
}


static inline float pmag_cal_report_diag_z_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//Z diagonal (matrix 33)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 27, 4)));
}


static inline void pmag_cal_report_diag_z_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //Z diagonal (matrix 33)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 27);
}


static inline float pmag_cal_report_offdiag_x_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//X off-diagonal (matrix 12 and 21)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 31, 4)));
}


static inline void pmag_cal_report_offdiag_x_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //X off-diagonal (matrix 12 and 21)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 31);
}


static inline float pmag_cal_report_offdiag_y_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//Y off-diagonal (matrix 13 and 31)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 35, 4)));
}


static inline void pmag_cal_report_offdiag_y_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //Y off-diagonal (matrix 13 and 31)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 35);
}


static inline float pmag_cal_report_offdiag_z_GET(pmag_cal_report_MAG_CAL_REPORT *src) {//Z off-diagonal (matrix 32 and 23)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 39, 4)));
}


static inline void pmag_cal_report_offdiag_z_SET(float src, pmag_cal_report_MAG_CAL_REPORT *dst) { //Z off-diagonal (matrix 32 and 23)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 39);
}

static inline bool pmag_cal_report_cal_status_GET(pmag_cal_report_MAG_CAL_REPORT *const src, e_MAG_CAL_STATUS *ret) {
	if (src->base.field_bit != 344 && !set_field(src, 344, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pmag_cal_report_cal_status_SET(e_MAG_CAL_STATUS src, pmag_cal_report_MAG_CAL_REPORT *const dst) {
	
	if (dst->base.field_bit != 344) set_field(dst, 344, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}


static inline int16_t plog_request_list_start_GET(plog_request_list_LOG_REQUEST_LIST *src) {//First log id (0 for first available)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void plog_request_list_start_SET(int16_t src, plog_request_list_LOG_REQUEST_LIST *dst) { //First log id (0 for first available)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t plog_request_list_end_GET(plog_request_list_LOG_REQUEST_LIST *src) {//Last log id (0xffff for last available)
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void plog_request_list_end_SET(int16_t src, plog_request_list_LOG_REQUEST_LIST *dst) { //Last log id (0xffff for last available)
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int8_t plog_request_list_target_system_GET(plog_request_list_LOG_REQUEST_LIST *src) {//System ID
	
	return (int8_t) src[4];
}


static inline void plog_request_list_target_system_SET(int8_t src, plog_request_list_LOG_REQUEST_LIST *dst) { //System ID
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t plog_request_list_target_component_GET(plog_request_list_LOG_REQUEST_LIST *src) {//Component ID
	
	return (int8_t) src[5];
}


static inline void plog_request_list_target_component_SET(int8_t src, plog_request_list_LOG_REQUEST_LIST *dst) { //Component ID
	
	
	dst[5] = (uint8_t) (src);
}


static inline int32_t pscaled_pressure_time_boot_ms_GET(pscaled_pressure_SCALED_PRESSURE *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline float pscaled_pressure_press_abs_GET(pscaled_pressure_SCALED_PRESSURE *src) {//Absolute pressure (hectopascal)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline float pscaled_pressure_press_diff_GET(pscaled_pressure_SCALED_PRESSURE *src) {//Differential pressure 1 (hectopascal)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline int16_t pscaled_pressure_temperature_GET(pscaled_pressure_SCALED_PRESSURE *src) {//Temperature measurement (0.01 degrees celsius)
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}

/**
*A code that identifies the software component that understands this message (analogous to usb device classes
*					 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
*					 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
*					 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
*					 Message_types greater than 32767 are considered local experiments and should not be checked in to any
*					 widely distributed codebase */

static inline int16_t pv2_extension_message_type_GET(pv2_extension_V2_EXTENSION *src) {
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}

/**
*A code that identifies the software component that understands this message (analogous to usb device classes
*					 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
*					 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
*					 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
*					 Message_types greater than 32767 are considered local experiments and should not be checked in to any
*					 widely distributed codebase */

static inline void pv2_extension_message_type_SET(int16_t src, pv2_extension_V2_EXTENSION *dst) {
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int8_t pv2_extension_target_network_GET(pv2_extension_V2_EXTENSION *src) {//Network ID (0 for broadcast)
	
	return (int8_t) src[2];
}


static inline void pv2_extension_target_network_SET(int8_t src, pv2_extension_V2_EXTENSION *dst) { //Network ID (0 for broadcast)
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t pv2_extension_target_system_GET(pv2_extension_V2_EXTENSION *src) {//System ID (0 for broadcast)
	
	return (int8_t) src[3];
}


static inline void pv2_extension_target_system_SET(int8_t src, pv2_extension_V2_EXTENSION *dst) { //System ID (0 for broadcast)
	
	
	dst[3] = (uint8_t) (src);
}


static inline int8_t pv2_extension_target_component_GET(pv2_extension_V2_EXTENSION *src) {//Component ID (0 for broadcast)
	
	return (int8_t) src[4];
}


static inline void pv2_extension_target_component_SET(int8_t src, pv2_extension_V2_EXTENSION *dst) { //Component ID (0 for broadcast)
	
	
	dst[4] = (uint8_t) (src);
}

/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*					 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*					 message_type.  The particular encoding used can be extension specific and might not always be documented
*					 as part of the mavlink specification */

static inline int8_t vv2_extension_payload_GET(Vv2_extension_payload const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*					 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*					 message_type.  The particular encoding used can be extension specific and might not always be documented
*					 as part of the mavlink specification */

static inline Vv2_extension_payload pv2_extension_payload_GET(pv2_extension_V2_EXTENSION *src) {
	
	return (Vv2_extension_payload) {src + 5, 249};
}

/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*					 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*					 message_type.  The particular encoding used can be extension specific and might not always be documented
*					 as part of the mavlink specification */

static inline void vv2_extension_payload_SET(int8_t src, size_t index, Vv2_extension_payload *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*					 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*					 message_type.  The particular encoding used can be extension specific and might not always be documented
*					 as part of the mavlink specification */

static inline Vv2_extension_payload pv2_extension_payload_SET(const int8_t src[], pv2_extension_V2_EXTENSION *dst) {
	
	
	Vv2_extension_payload ret = {dst + 5, 249};
	
	if (src)
		for (size_t i = 0; i < 249; i++)
			vv2_extension_payload_SET(src[i], i, &ret);
	return ret;
}


static inline int32_t pheartbeat_custom_mode_GET(pheartbeat_HEARTBEAT *src) {//A bitfield for use for autopilot-specific flags.
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline int8_t pheartbeat_mavlink_version_GET(pheartbeat_HEARTBEAT *src) {//MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_versio
	
	return (int8_t) src->base.bytes[4];
}

static inline bool pheartbeat_typE_GET(pheartbeat_HEARTBEAT *const src, e_MAV_TYPE *ret) {
	if (src->base.field_bit != 42 && !set_field(src, 42, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline bool pheartbeat_autopilot_GET(pheartbeat_HEARTBEAT *const src, e_MAV_AUTOPILOT *ret) {
	if (src->base.field_bit != 43 && !set_field(src, 43, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline bool pheartbeat_base_mode_GET(pheartbeat_HEARTBEAT *const src, e_MAV_MODE_FLAG *ret) {
	if (src->base.field_bit != 44 && !set_field(src, 44, -1)) return false;
	*ret = _en_mav_mode_flag(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline bool pheartbeat_system_status_GET(pheartbeat_HEARTBEAT *const src, e_MAV_STATE *ret) {
	if (src->base.field_bit != 45 && !set_field(src, 45, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}


static inline int8_t pparam_map_rc_target_system_GET(pparam_map_rc_PARAM_MAP_RC *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pparam_map_rc_target_component_GET(pparam_map_rc_PARAM_MAP_RC *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}

/**
*Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
*					 send -2 to disable any existing map for this rc_channel_index */

static inline int16_t pparam_map_rc_param_index_GET(pparam_map_rc_PARAM_MAP_RC *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}

/**
*Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
*					 on the RC */

static inline int8_t pparam_map_rc_parameter_rc_channel_index_GET(pparam_map_rc_PARAM_MAP_RC *src) {
	
	return (int8_t) src->base.bytes[4];
}


static inline float pparam_map_rc_param_value0_GET(pparam_map_rc_PARAM_MAP_RC *src) {//Initial parameter value
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 5, 4)));
}


static inline float pparam_map_rc_scale_GET(pparam_map_rc_PARAM_MAP_RC *src) {//Scale, maps the RC range [-1, 1] to a parameter value
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 9, 4)));
}

/**
*Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
*					 on implementation */

static inline float pparam_map_rc_param_value_min_GET(pparam_map_rc_PARAM_MAP_RC *src) {
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 13, 4)));
}

/**
*Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
*					 on implementation */

static inline float pparam_map_rc_param_value_max_GET(pparam_map_rc_PARAM_MAP_RC *src) {
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 17, 4)));
}

/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*					 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*					 storage if the ID is stored as strin */

static inline bool pparam_map_rc_param_id_GET(pparam_map_rc_PARAM_MAP_RC *const src, Vparam_map_rc_param_id *ret) {
	
	if (src->base.field_bit != 170 && !set_field(src, 170, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}


static inline int16_t ppower_status_Vcc_GET(ppower_status_POWER_STATUS *src) {//5V rail voltage in millivolts
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void ppower_status_Vcc_SET(int16_t src, ppower_status_POWER_STATUS *dst) { //5V rail voltage in millivolts
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t ppower_status_Vservo_GET(ppower_status_POWER_STATUS *src) {//servo rail voltage in millivolts
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void ppower_status_Vservo_SET(int16_t src, ppower_status_POWER_STATUS *dst) { //servo rail voltage in millivolts
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}

static inline bool ppower_status_flags_GET(ppower_status_POWER_STATUS *const src, e_MAV_POWER_STATUS *ret) {
	if (src->base.field_bit != 32 && !set_field(src, 32, -1)) return false;
	*ret = _en_mav_power_status(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void ppower_status_flags_SET(e_MAV_POWER_STATUS src, ppower_status_POWER_STATUS *const dst) {
	
	if (dst->base.field_bit != 32) set_field(dst, 32, 0);
	
	
	UMAX id = _id_mav_power_status(src);
	set_bits(id, 3, dst->base.bytes, dst->BIT);
}


static inline int8_t premote_log_data_block_target_system_GET(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline void premote_log_data_block_target_system_SET(int8_t src, premote_log_data_block_REMOTE_LOG_DATA_BLOCK *dst) { //System ID
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t premote_log_data_block_target_component_GET(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline void premote_log_data_block_target_component_SET(int8_t src, premote_log_data_block_REMOTE_LOG_DATA_BLOCK *dst) { //Component ID
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}


static inline int8_t vremote_log_data_block_daTa_GET(Vremote_log_data_block_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vremote_log_data_block_daTa premote_log_data_block_daTa_GET(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *src) { //log data block
	
	return (Vremote_log_data_block_daTa) {src->base.bytes + 2, 200};
}


static inline void vremote_log_data_block_daTa_SET(int8_t src, size_t index, Vremote_log_data_block_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vremote_log_data_block_daTa premote_log_data_block_daTa_SET(const int8_t src[], premote_log_data_block_REMOTE_LOG_DATA_BLOCK *dst) {//log data block
	
	
	Vremote_log_data_block_daTa ret = {dst->base.bytes + 2, 200};
	
	if (src)
		for (size_t i = 0; i < 200; i++)
			vremote_log_data_block_daTa_SET(src[i], i, &ret);
	return ret;
}

static inline bool premote_log_data_block_seqno_GET(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *const src, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS *ret) {
	if (src->base.field_bit != 1616 && !set_field(src, 1616, -1)) return false;
	*ret = (uint32_t) (2147483645 + get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void premote_log_data_block_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS src, premote_log_data_block_REMOTE_LOG_DATA_BLOCK *const dst) {
	
	if (dst->base.field_bit != 1616) set_field(dst, 1616, 0);
	
	
	set_bits(-2147483645
	src, 1, dst->base.bytes, dst->BIT);
}


static inline int16_t plogging_data_acked_sequence_GET(plogging_data_acked_LOGGING_DATA_ACKED *src) {//sequence number (can wrap)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void plogging_data_acked_sequence_SET(int16_t src, plogging_data_acked_LOGGING_DATA_ACKED *dst) { //sequence number (can wrap)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int8_t plogging_data_acked_target_system_GET(plogging_data_acked_LOGGING_DATA_ACKED *src) {//system ID of the target
	
	return (int8_t) src[2];
}


static inline void plogging_data_acked_target_system_SET(int8_t src, plogging_data_acked_LOGGING_DATA_ACKED *dst) { //system ID of the target
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t plogging_data_acked_target_component_GET(plogging_data_acked_LOGGING_DATA_ACKED *src) {//component ID of the target
	
	return (int8_t) src[3];
}


static inline void plogging_data_acked_target_component_SET(int8_t src, plogging_data_acked_LOGGING_DATA_ACKED *dst) { //component ID of the target
	
	
	dst[3] = (uint8_t) (src);
}


static inline int8_t plogging_data_acked_length_GET(plogging_data_acked_LOGGING_DATA_ACKED *src) {//data length
	
	return (int8_t) src[4];
}


static inline void plogging_data_acked_length_SET(int8_t src, plogging_data_acked_LOGGING_DATA_ACKED *dst) { //data length
	
	
	dst[4] = (uint8_t) (src);
}

/**
*offset into data where first message starts. This can be used for recovery, when a previous message got
*					 lost (set to 255 if no start exists) */

static inline int8_t plogging_data_acked_first_message_offset_GET(plogging_data_acked_LOGGING_DATA_ACKED *src) {
	
	return (int8_t) src[5];
}

/**
*offset into data where first message starts. This can be used for recovery, when a previous message got
*					 lost (set to 255 if no start exists) */

static inline void plogging_data_acked_first_message_offset_SET(int8_t src, plogging_data_acked_LOGGING_DATA_ACKED *dst) {
	
	
	dst[5] = (uint8_t) (src);
}


static inline int8_t vlogging_data_acked_daTa_GET(Vlogging_data_acked_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vlogging_data_acked_daTa plogging_data_acked_daTa_GET(plogging_data_acked_LOGGING_DATA_ACKED *src) { //logged data
	
	return (Vlogging_data_acked_daTa) {src + 6, 249};
}


static inline void vlogging_data_acked_daTa_SET(int8_t src, size_t index, Vlogging_data_acked_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vlogging_data_acked_daTa plogging_data_acked_daTa_SET(const int8_t src[], plogging_data_acked_LOGGING_DATA_ACKED *dst) {//logged data
	
	
	Vlogging_data_acked_daTa ret = {dst + 6, 249};
	
	if (src)
		for (size_t i = 0; i < 249; i++)
			vlogging_data_acked_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int32_t pterrain_check_lat_GET(pterrain_check_TERRAIN_CHECK *src) {//Latitude (degrees *10^7)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pterrain_check_lat_SET(int32_t src, pterrain_check_TERRAIN_CHECK *dst) { //Latitude (degrees *10^7)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int32_t pterrain_check_lon_GET(pterrain_check_TERRAIN_CHECK *src) {//Longitude (degrees *10^7)
	
	return ((int32_t) (get_bytes(src, 4, 4)));
}


static inline void pterrain_check_lon_SET(int32_t src, pterrain_check_TERRAIN_CHECK *dst) { //Longitude (degrees *10^7)
	
	
	set_bytes((uint32_t) (src), 4, dst, 4);
}


static inline int8_t pmount_configure_target_system_GET(pmount_configure_MOUNT_CONFIGURE *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline void pmount_configure_target_system_SET(int8_t src, pmount_configure_MOUNT_CONFIGURE *dst) { //System ID
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t pmount_configure_target_component_GET(pmount_configure_MOUNT_CONFIGURE *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline void pmount_configure_target_component_SET(int8_t src, pmount_configure_MOUNT_CONFIGURE *dst) { //Component ID
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}


static inline int8_t pmount_configure_stab_roll_GET(pmount_configure_MOUNT_CONFIGURE *src) {//(1 = yes, 0 = no)
	
	return (int8_t) src->base.bytes[2];
}


static inline void pmount_configure_stab_roll_SET(int8_t src, pmount_configure_MOUNT_CONFIGURE *dst) { //(1 = yes, 0 = no)
	
	
	dst->base.bytes[2] = (uint8_t) (src);
}


static inline int8_t pmount_configure_stab_pitch_GET(pmount_configure_MOUNT_CONFIGURE *src) {//(1 = yes, 0 = no)
	
	return (int8_t) src->base.bytes[3];
}


static inline void pmount_configure_stab_pitch_SET(int8_t src, pmount_configure_MOUNT_CONFIGURE *dst) { //(1 = yes, 0 = no)
	
	
	dst->base.bytes[3] = (uint8_t) (src);
}


static inline int8_t pmount_configure_stab_yaw_GET(pmount_configure_MOUNT_CONFIGURE *src) {//(1 = yes, 0 = no)
	
	return (int8_t) src->base.bytes[4];
}


static inline void pmount_configure_stab_yaw_SET(int8_t src, pmount_configure_MOUNT_CONFIGURE *dst) { //(1 = yes, 0 = no)
	
	
	dst->base.bytes[4] = (uint8_t) (src);
}

static inline bool pmount_configure_mount_mode_GET(pmount_configure_MOUNT_CONFIGURE *const src, e_MAV_MOUNT_MODE *ret) {
	if (src->base.field_bit != 40 && !set_field(src, 40, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pmount_configure_mount_mode_SET(e_MAV_MOUNT_MODE src, pmount_configure_MOUNT_CONFIGURE *const dst) {
	
	if (dst->base.field_bit != 40) set_field(dst, 40, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}


static inline int16_t pmission_request_int_seq_GET(pmission_request_int_MISSION_REQUEST_INT *src) {//Sequence
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int8_t pmission_request_int_target_system_GET(pmission_request_int_MISSION_REQUEST_INT *src) {//System ID
	
	return (int8_t) src->base.bytes[2];
}


static inline int8_t pmission_request_int_target_component_GET(pmission_request_int_MISSION_REQUEST_INT *src) {//Component ID
	
	return (int8_t) src->base.bytes[3];
}

static inline bool pmission_request_int_mission_type_GET(pmission_request_int_MISSION_REQUEST_INT *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 32 && !set_field(src, 32, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline int32_t plocal_position_ned_system_global_offset_time_boot_ms_GET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline float plocal_position_ned_system_global_offset_x_GET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *src) {//X Position
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline float plocal_position_ned_system_global_offset_y_GET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *src) {//Y Position
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline float plocal_position_ned_system_global_offset_z_GET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *src) {//Z Position
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float plocal_position_ned_system_global_offset_roll_GET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *src) {//Roll
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline float plocal_position_ned_system_global_offset_pitch_GET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *src) {//Pitch
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline float plocal_position_ned_system_global_offset_yaw_GET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *src) {//Yaw
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}

static inline bool pcommand_ack_command_GET(pcommand_ack_COMMAND_ACK *const src, e_MAV_CMD *ret) {
	if (src->base.field_bit != 2 && !set_field(src, 2, -1)) return false;
	*ret = _en_mav_cmd(get_bits(src->base.bytes, src->BIT, 8));
	return true;
}

static inline bool pcommand_ack_result_GET(pcommand_ack_COMMAND_ACK *const src, e_MAV_RESULT *ret) {
	if (src->base.field_bit != 3 && !set_field(src, 3, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline bool pcommand_ack_progress_GET(pcommand_ack_COMMAND_ACK *const src, int8_t *ret) {
	if (src->base.field_bit != 4 && !set_field(src, 4, -1)) return false;
	*ret = (int8_t) src->base.bytes[src->BYTE];
	return true;
}

static inline bool pcommand_ack_result_param2_GET(pcommand_ack_COMMAND_ACK *const src, int32_t *ret) {
	if (src->base.field_bit != 5 && !set_field(src, 5, -1)) return false;
	*ret = ((int32_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline bool pcommand_ack_target_system_GET(pcommand_ack_COMMAND_ACK *const src, int8_t *ret) {
	if (src->base.field_bit != 6 && !set_field(src, 6, -1)) return false;
	*ret = (int8_t) src->base.bytes[src->BYTE];
	return true;
}

static inline bool pcommand_ack_target_component_GET(pcommand_ack_COMMAND_ACK *const src, int8_t *ret) {
	if (src->base.field_bit != 7 && !set_field(src, 7, -1)) return false;
	*ret = (int8_t) src->base.bytes[src->BYTE];
	return true;
}


static inline int16_t pdata_stream_message_rate_GET(pdata_stream_DATA_STREAM *src) {//The message rate
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int8_t pdata_stream_stream_id_GET(pdata_stream_DATA_STREAM *src) {//The ID of the requested data stream
	
	return (int8_t) src[2];
}


static inline int8_t pdata_stream_on_off_GET(pdata_stream_DATA_STREAM *src) {//1 stream is enabled, 0 stream is stopped.
	
	return (int8_t) src[3];
}


static inline int16_t pmission_request_seq_GET(pmission_request_MISSION_REQUEST *src) {//Sequence
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int8_t pmission_request_target_system_GET(pmission_request_MISSION_REQUEST *src) {//System ID
	
	return (int8_t) src->base.bytes[2];
}


static inline int8_t pmission_request_target_component_GET(pmission_request_MISSION_REQUEST *src) {//Component ID
	
	return (int8_t) src->base.bytes[3];
}

static inline bool pmission_request_mission_type_GET(pmission_request_MISSION_REQUEST *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 32 && !set_field(src, 32, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline int16_t pterrain_report_spacing_GET(pterrain_report_TERRAIN_REPORT *src) {//grid spacing (zero if terrain at this location unavailable)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pterrain_report_spacing_SET(int16_t src, pterrain_report_TERRAIN_REPORT *dst) { //grid spacing (zero if terrain at this location unavailable)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t pterrain_report_pending_GET(pterrain_report_TERRAIN_REPORT *src) {//Number of 4x4 terrain blocks waiting to be received or read from disk
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pterrain_report_pending_SET(int16_t src, pterrain_report_TERRAIN_REPORT *dst) { //Number of 4x4 terrain blocks waiting to be received or read from disk
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t pterrain_report_loaded_GET(pterrain_report_TERRAIN_REPORT *src) {//Number of 4x4 terrain blocks in memory
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void pterrain_report_loaded_SET(int16_t src, pterrain_report_TERRAIN_REPORT *dst) { //Number of 4x4 terrain blocks in memory
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int32_t pterrain_report_lat_GET(pterrain_report_TERRAIN_REPORT *src) {//Latitude (degrees *10^7)
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void pterrain_report_lat_SET(int32_t src, pterrain_report_TERRAIN_REPORT *dst) { //Latitude (degrees *10^7)
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int32_t pterrain_report_lon_GET(pterrain_report_TERRAIN_REPORT *src) {//Longitude (degrees *10^7)
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}


static inline void pterrain_report_lon_SET(int32_t src, pterrain_report_TERRAIN_REPORT *dst) { //Longitude (degrees *10^7)
	
	
	set_bytes((uint32_t) (src), 4, dst, 10);
}


static inline float pterrain_report_terrain_height_GET(pterrain_report_TERRAIN_REPORT *src) {//Terrain height in meters AMSL
	
	return (intBitsToFloat(get_bytes(src, 14, 4)));
}


static inline void pterrain_report_terrain_height_SET(float src, pterrain_report_TERRAIN_REPORT *dst) { //Terrain height in meters AMSL
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 14);
}


static inline float pterrain_report_current_height_GET(pterrain_report_TERRAIN_REPORT *src) {//Current vehicle height above lat/lon terrain height (meters)
	
	return (intBitsToFloat(get_bytes(src, 18, 4)));
}


static inline void pterrain_report_current_height_SET(float src, pterrain_report_TERRAIN_REPORT *dst) { //Current vehicle height above lat/lon terrain height (meters)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 18);
}


static inline int8_t pset_home_position_target_system_GET(pset_home_position_SET_HOME_POSITION *src) {//System ID.
	
	return (int8_t) src->base.bytes[0];
}


static inline void pset_home_position_target_system_SET(int8_t src, pset_home_position_SET_HOME_POSITION *dst) { //System ID.
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int32_t pset_home_position_latitude_GET(pset_home_position_SET_HOME_POSITION *src) {//Latitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 1, 4)));
}


static inline void pset_home_position_latitude_SET(int32_t src, pset_home_position_SET_HOME_POSITION *dst) { //Latitude (WGS84), in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 1);
}


static inline int32_t pset_home_position_longitude_GET(pset_home_position_SET_HOME_POSITION *src) {//Longitude (WGS84, in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 5, 4)));
}


static inline void pset_home_position_longitude_SET(int32_t src, pset_home_position_SET_HOME_POSITION *dst) { //Longitude (WGS84, in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 5);
}


static inline int32_t pset_home_position_altitude_GET(pset_home_position_SET_HOME_POSITION *src) {//Altitude (AMSL), in meters * 1000 (positive for up)
	
	return ((int32_t) (get_bytes(src->base.bytes, 9, 4)));
}


static inline void pset_home_position_altitude_SET(int32_t src, pset_home_position_SET_HOME_POSITION *dst) { //Altitude (AMSL), in meters * 1000 (positive for up)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 9);
}


static inline float pset_home_position_x_GET(pset_home_position_SET_HOME_POSITION *src) {//Local X position of this position in the local coordinate frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 13, 4)));
}


static inline void pset_home_position_x_SET(float src, pset_home_position_SET_HOME_POSITION *dst) { //Local X position of this position in the local coordinate frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 13);
}


static inline float pset_home_position_y_GET(pset_home_position_SET_HOME_POSITION *src) {//Local Y position of this position in the local coordinate frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 17, 4)));
}


static inline void pset_home_position_y_SET(float src, pset_home_position_SET_HOME_POSITION *dst) { //Local Y position of this position in the local coordinate frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 17);
}


static inline float pset_home_position_z_GET(pset_home_position_SET_HOME_POSITION *src) {//Local Z position of this position in the local coordinate frame
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 21, 4)));
}


static inline void pset_home_position_z_SET(float src, pset_home_position_SET_HOME_POSITION *dst) { //Local Z position of this position in the local coordinate frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 21);
}

/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*					 and slope of the groun */

static inline float vset_home_position_q_GET(Vset_home_position_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*					 and slope of the groun */

static inline Vset_home_position_q pset_home_position_q_GET(pset_home_position_SET_HOME_POSITION *src) {
	
	return (Vset_home_position_q) {src->base.bytes + 25, 4};
}

/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*					 and slope of the groun */

static inline void vset_home_position_q_SET(float src, size_t index, Vset_home_position_q *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*					 and slope of the groun */

static inline Vset_home_position_q pset_home_position_q_SET(const float src[], pset_home_position_SET_HOME_POSITION *dst) {
	
	
	Vset_home_position_q ret = {dst->base.bytes + 25, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vset_home_position_q_SET(src[i], i, &ret);
	return ret;
}

/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline float pset_home_position_approach_x_GET(pset_home_position_SET_HOME_POSITION *src) {
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 41, 4)));
}

/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline void pset_home_position_approach_x_SET(float src, pset_home_position_SET_HOME_POSITION *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 41);
}

/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline float pset_home_position_approach_y_GET(pset_home_position_SET_HOME_POSITION *src) {
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 45, 4)));
}

/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline void pset_home_position_approach_y_SET(float src, pset_home_position_SET_HOME_POSITION *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 45);
}

/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline float pset_home_position_approach_z_GET(pset_home_position_SET_HOME_POSITION *src) {
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 49, 4)));
}

/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*					 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*					 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*					 from the threshold / touchdown zone */

static inline void pset_home_position_approach_z_SET(float src, pset_home_position_SET_HOME_POSITION *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 49);
}

static inline bool pset_home_position_time_usec_GET(pset_home_position_SET_HOME_POSITION *const src, int64_t *ret) {
	if (src->base.field_bit != 424 && !set_field(src, 424, -1)) return false;
	*ret = ((int64_t) (get_bytes(src->base.bytes, src->BYTE, 8)));
	return true;
}

static inline void pset_home_position_time_usec_SET(int64_t src, pset_home_position_SET_HOME_POSITION *const dst) {
	
	if (dst->base.field_bit != 424) set_field(dst, 424, 0);
	
	
	set_bytes((src), 8, dst->base.bytes, dst->BYTE);
}


static inline int16_t phil_rc_inputs_raw_chan1_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 1 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan2_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 2 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan3_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 3 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan4_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 4 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan5_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 5 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan6_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 6 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan7_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 7 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan8_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 8 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan9_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 9 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 16, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan10_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 10 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 18, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan11_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 11 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 20, 2)));
}


static inline int16_t phil_rc_inputs_raw_chan12_raw_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//RC channel 12 value, in microseconds
	
	return ((int16_t) (get_bytes(src, 22, 2)));
}


static inline int64_t phil_rc_inputs_raw_time_usec_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src, 24, 8)));
}


static inline int8_t phil_rc_inputs_raw_rssi_GET(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *src) {//Receive signal strength indicator, 0: 0%, 255: 100%
	
	return (int8_t) src[32];
}


static inline int32_t pscaled_imu3_time_boot_ms_GET(pscaled_imu3_SCALED_IMU3 *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pscaled_imu3_time_boot_ms_SET(int32_t src, pscaled_imu3_SCALED_IMU3 *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int16_t pscaled_imu3_xacc_GET(pscaled_imu3_SCALED_IMU3 *src) {//X acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void pscaled_imu3_xacc_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //X acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int16_t pscaled_imu3_yacc_GET(pscaled_imu3_SCALED_IMU3 *src) {//Y acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline void pscaled_imu3_yacc_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //Y acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 6);
}


static inline int16_t pscaled_imu3_zacc_GET(pscaled_imu3_SCALED_IMU3 *src) {//Z acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline void pscaled_imu3_zacc_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //Z acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 8);
}


static inline int16_t pscaled_imu3_xgyro_GET(pscaled_imu3_SCALED_IMU3 *src) {//Angular speed around X axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline void pscaled_imu3_xgyro_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //Angular speed around X axis (millirad /sec)
	
	
	set_bytes((uint16_t) (src), 2, dst, 10);
}


static inline int16_t pscaled_imu3_ygyro_GET(pscaled_imu3_SCALED_IMU3 *src) {//Angular speed around Y axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline void pscaled_imu3_ygyro_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //Angular speed around Y axis (millirad /sec)
	
	
	set_bytes((uint16_t) (src), 2, dst, 12);
}


static inline int16_t pscaled_imu3_zgyro_GET(pscaled_imu3_SCALED_IMU3 *src) {//Angular speed around Z axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline void pscaled_imu3_zgyro_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //Angular speed around Z axis (millirad /sec)
	
	
	set_bytes((uint16_t) (src), 2, dst, 14);
}


static inline int16_t pscaled_imu3_xmag_GET(pscaled_imu3_SCALED_IMU3 *src) {//X Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 16, 2)));
}


static inline void pscaled_imu3_xmag_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //X Magnetic field (milli tesla)
	
	
	set_bytes((uint16_t) (src), 2, dst, 16);
}


static inline int16_t pscaled_imu3_ymag_GET(pscaled_imu3_SCALED_IMU3 *src) {//Y Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 18, 2)));
}


static inline void pscaled_imu3_ymag_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //Y Magnetic field (milli tesla)
	
	
	set_bytes((uint16_t) (src), 2, dst, 18);
}


static inline int16_t pscaled_imu3_zmag_GET(pscaled_imu3_SCALED_IMU3 *src) {//Z Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 20, 2)));
}


static inline void pscaled_imu3_zmag_SET(int16_t src, pscaled_imu3_SCALED_IMU3 *dst) { //Z Magnetic field (milli tesla)
	
	
	set_bytes((uint16_t) (src), 2, dst, 20);
}


static inline int32_t pset_mode_custom_mode_GET(pset_mode_SET_MODE *src) {//The new autopilot-specific mode. This field can be ignored by an autopilot.
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline int8_t pset_mode_target_system_GET(pset_mode_SET_MODE *src) {//The system setting the mode
	
	return (int8_t) src->base.bytes[4];
}

static inline bool pset_mode_base_mode_GET(pset_mode_SET_MODE *const src, e_MAV_MODE *ret) {
	if (src->base.field_bit != 40 && !set_field(src, 40, -1)) return false;
	*ret = _en_mav_mode(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}


static inline int8_t pmount_control_target_system_GET(pmount_control_MOUNT_CONTROL *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pmount_control_target_system_SET(int8_t src, pmount_control_MOUNT_CONTROL *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pmount_control_target_component_GET(pmount_control_MOUNT_CONTROL *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pmount_control_target_component_SET(int8_t src, pmount_control_MOUNT_CONTROL *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int32_t pmount_control_input_a_GET(pmount_control_MOUNT_CONTROL *src) {//pitch(deg*100) or lat, depending on mount mode
	
	return ((int32_t) (get_bytes(src, 2, 4)));
}


static inline void pmount_control_input_a_SET(int32_t src, pmount_control_MOUNT_CONTROL *dst) { //pitch(deg*100) or lat, depending on mount mode
	
	
	set_bytes((uint32_t) (src), 4, dst, 2);
}


static inline int32_t pmount_control_input_b_GET(pmount_control_MOUNT_CONTROL *src) {//roll(deg*100) or lon depending on mount mode
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void pmount_control_input_b_SET(int32_t src, pmount_control_MOUNT_CONTROL *dst) { //roll(deg*100) or lon depending on mount mode
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int32_t pmount_control_input_c_GET(pmount_control_MOUNT_CONTROL *src) {//yaw(deg*100) or alt (in cm) depending on mount mode
	
	return ((int32_t) (get_bytes(src, 10, 4)));
}


static inline void pmount_control_input_c_SET(int32_t src, pmount_control_MOUNT_CONTROL *dst) { //yaw(deg*100) or alt (in cm) depending on mount mode
	
	
	set_bytes((uint32_t) (src), 4, dst, 10);
}


static inline int8_t pmount_control_save_position_GET(pmount_control_MOUNT_CONTROL *src) {//if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
	
	return (int8_t) src[14];
}


static inline void pmount_control_save_position_SET(int8_t src, pmount_control_MOUNT_CONTROL *dst) { //if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
	
	
	dst[14] = (uint8_t) (src);
}

/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*					 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*					 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*					 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*					 bit 11: yaw, bit 12: yaw rat */

static inline int16_t pposition_target_global_int_type_mask_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}

/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*					 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*					 processing latency */

static inline int32_t pposition_target_global_int_time_boot_ms_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline int32_t pposition_target_global_int_lat_int_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//X Position in WGS84 frame in 1e7 * meters
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline int32_t pposition_target_global_int_lon_int_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//Y Position in WGS84 frame in 1e7 * meters
	
	return ((int32_t) (get_bytes(src->base.bytes, 10, 4)));
}


static inline float pposition_target_global_int_alt_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 14, 4)));
}


static inline float pposition_target_global_int_vx_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//X velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 18, 4)));
}


static inline float pposition_target_global_int_vy_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//Y velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 22, 4)));
}


static inline float pposition_target_global_int_vz_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//Z velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 26, 4)));
}


static inline float pposition_target_global_int_afx_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 30, 4)));
}


static inline float pposition_target_global_int_afy_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 34, 4)));
}


static inline float pposition_target_global_int_afz_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 38, 4)));
}


static inline float pposition_target_global_int_yaw_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//yaw setpoint in rad
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 42, 4)));
}


static inline float pposition_target_global_int_yaw_rate_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *src) {//yaw rate setpoint in rad/s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 46, 4)));
}

static inline bool pposition_target_global_int_coordinate_frame_GET(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 400 && !set_field(src, 400, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}


static inline int8_t pled_control_target_system_GET(pled_control_LED_CONTROL *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pled_control_target_system_SET(int8_t src, pled_control_LED_CONTROL *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pled_control_target_component_GET(pled_control_LED_CONTROL *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pled_control_target_component_SET(int8_t src, pled_control_LED_CONTROL *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t pled_control_instance_GET(pled_control_LED_CONTROL *src) {//Instance (LED instance to control or 255 for all LEDs)
	
	return (int8_t) src[2];
}


static inline void pled_control_instance_SET(int8_t src, pled_control_LED_CONTROL *dst) { //Instance (LED instance to control or 255 for all LEDs)
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t pled_control_pattern_GET(pled_control_LED_CONTROL *src) {//Pattern (see LED_PATTERN_ENUM)
	
	return (int8_t) src[3];
}


static inline void pled_control_pattern_SET(int8_t src, pled_control_LED_CONTROL *dst) { //Pattern (see LED_PATTERN_ENUM)
	
	
	dst[3] = (uint8_t) (src);
}


static inline int8_t pled_control_custom_len_GET(pled_control_LED_CONTROL *src) {//Custom Byte Length
	
	return (int8_t) src[4];
}


static inline void pled_control_custom_len_SET(int8_t src, pled_control_LED_CONTROL *dst) { //Custom Byte Length
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t vled_control_custom_bytes_GET(Vled_control_custom_bytes const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vled_control_custom_bytes pled_control_custom_bytes_GET(pled_control_LED_CONTROL *src) { //Custom Bytes
	
	return (Vled_control_custom_bytes) {src + 5, 24};
}


static inline void vled_control_custom_bytes_SET(int8_t src, size_t index, Vled_control_custom_bytes *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vled_control_custom_bytes pled_control_custom_bytes_SET(const int8_t src[], pled_control_LED_CONTROL *dst) {//Custom Bytes
	
	
	Vled_control_custom_bytes ret = {dst + 5, 24};
	
	if (src)
		for (size_t i = 0; i < 24; i++)
			vled_control_custom_bytes_SET(src[i], i, &ret);
	return ret;
}


static inline float psim_state_q1_GET(psim_state_SIM_STATE *src) {//True attitude quaternion component 1, w (1 in null-rotation)
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void psim_state_q1_SET(float src, psim_state_SIM_STATE *dst) { //True attitude quaternion component 1, w (1 in null-rotation)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float psim_state_q2_GET(psim_state_SIM_STATE *src) {//True attitude quaternion component 2, x (0 in null-rotation)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void psim_state_q2_SET(float src, psim_state_SIM_STATE *dst) { //True attitude quaternion component 2, x (0 in null-rotation)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float psim_state_q3_GET(psim_state_SIM_STATE *src) {//True attitude quaternion component 3, y (0 in null-rotation)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void psim_state_q3_SET(float src, psim_state_SIM_STATE *dst) { //True attitude quaternion component 3, y (0 in null-rotation)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float psim_state_q4_GET(psim_state_SIM_STATE *src) {//True attitude quaternion component 4, z (0 in null-rotation)
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void psim_state_q4_SET(float src, psim_state_SIM_STATE *dst) { //True attitude quaternion component 4, z (0 in null-rotation)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float psim_state_roll_GET(psim_state_SIM_STATE *src) {//Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void psim_state_roll_SET(float src, psim_state_SIM_STATE *dst) { //Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float psim_state_pitch_GET(psim_state_SIM_STATE *src) {//Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void psim_state_pitch_SET(float src, psim_state_SIM_STATE *dst) { //Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float psim_state_yaw_GET(psim_state_SIM_STATE *src) {//Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void psim_state_yaw_SET(float src, psim_state_SIM_STATE *dst) { //Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float psim_state_xacc_GET(psim_state_SIM_STATE *src) {//X acceleration m/s/s
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void psim_state_xacc_SET(float src, psim_state_SIM_STATE *dst) { //X acceleration m/s/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float psim_state_yacc_GET(psim_state_SIM_STATE *src) {//Y acceleration m/s/s
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void psim_state_yacc_SET(float src, psim_state_SIM_STATE *dst) { //Y acceleration m/s/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}


static inline float psim_state_zacc_GET(psim_state_SIM_STATE *src) {//Z acceleration m/s/s
	
	return (intBitsToFloat(get_bytes(src, 36, 4)));
}


static inline void psim_state_zacc_SET(float src, psim_state_SIM_STATE *dst) { //Z acceleration m/s/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 36);
}


static inline float psim_state_xgyro_GET(psim_state_SIM_STATE *src) {//Angular speed around X axis rad/s
	
	return (intBitsToFloat(get_bytes(src, 40, 4)));
}


static inline void psim_state_xgyro_SET(float src, psim_state_SIM_STATE *dst) { //Angular speed around X axis rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 40);
}


static inline float psim_state_ygyro_GET(psim_state_SIM_STATE *src) {//Angular speed around Y axis rad/s
	
	return (intBitsToFloat(get_bytes(src, 44, 4)));
}


static inline void psim_state_ygyro_SET(float src, psim_state_SIM_STATE *dst) { //Angular speed around Y axis rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 44);
}


static inline float psim_state_zgyro_GET(psim_state_SIM_STATE *src) {//Angular speed around Z axis rad/s
	
	return (intBitsToFloat(get_bytes(src, 48, 4)));
}


static inline void psim_state_zgyro_SET(float src, psim_state_SIM_STATE *dst) { //Angular speed around Z axis rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 48);
}


static inline float psim_state_lat_GET(psim_state_SIM_STATE *src) {//Latitude in degrees
	
	return (intBitsToFloat(get_bytes(src, 52, 4)));
}


static inline void psim_state_lat_SET(float src, psim_state_SIM_STATE *dst) { //Latitude in degrees
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 52);
}


static inline float psim_state_lon_GET(psim_state_SIM_STATE *src) {//Longitude in degrees
	
	return (intBitsToFloat(get_bytes(src, 56, 4)));
}


static inline void psim_state_lon_SET(float src, psim_state_SIM_STATE *dst) { //Longitude in degrees
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 56);
}


static inline float psim_state_alt_GET(psim_state_SIM_STATE *src) {//Altitude in meters
	
	return (intBitsToFloat(get_bytes(src, 60, 4)));
}


static inline void psim_state_alt_SET(float src, psim_state_SIM_STATE *dst) { //Altitude in meters
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 60);
}


static inline float psim_state_std_dev_horz_GET(psim_state_SIM_STATE *src) {//Horizontal position standard deviation
	
	return (intBitsToFloat(get_bytes(src, 64, 4)));
}


static inline void psim_state_std_dev_horz_SET(float src, psim_state_SIM_STATE *dst) { //Horizontal position standard deviation
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 64);
}


static inline float psim_state_std_dev_vert_GET(psim_state_SIM_STATE *src) {//Vertical position standard deviation
	
	return (intBitsToFloat(get_bytes(src, 68, 4)));
}


static inline void psim_state_std_dev_vert_SET(float src, psim_state_SIM_STATE *dst) { //Vertical position standard deviation
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 68);
}


static inline float psim_state_vn_GET(psim_state_SIM_STATE *src) {//True velocity in m/s in NORTH direction in earth-fixed NED frame
	
	return (intBitsToFloat(get_bytes(src, 72, 4)));
}


static inline void psim_state_vn_SET(float src, psim_state_SIM_STATE *dst) { //True velocity in m/s in NORTH direction in earth-fixed NED frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 72);
}


static inline float psim_state_ve_GET(psim_state_SIM_STATE *src) {//True velocity in m/s in EAST direction in earth-fixed NED frame
	
	return (intBitsToFloat(get_bytes(src, 76, 4)));
}


static inline void psim_state_ve_SET(float src, psim_state_SIM_STATE *dst) { //True velocity in m/s in EAST direction in earth-fixed NED frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 76);
}


static inline float psim_state_vd_GET(psim_state_SIM_STATE *src) {//True velocity in m/s in DOWN direction in earth-fixed NED frame
	
	return (intBitsToFloat(get_bytes(src, 80, 4)));
}


static inline void psim_state_vd_SET(float src, psim_state_SIM_STATE *dst) { //True velocity in m/s in DOWN direction in earth-fixed NED frame
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 80);
}

static inline bool pwifi_config_ap_ssid_GET(pwifi_config_ap_WIFI_CONFIG_AP *const src, Vwifi_config_ap_ssid *ret) {//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
	
	if (src->base.field_bit != 2 && !set_field(src, 2, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vwifi_config_ap_ssid pwifi_config_ap_ssid_SET(const char src[], size_t len, pwifi_config_ap_WIFI_CONFIG_AP *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 2, len);
	
	Vwifi_config_ap_ssid ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pwifi_config_ap_password_GET(pwifi_config_ap_WIFI_CONFIG_AP *const src, Vwifi_config_ap_password *ret) {//Password. Leave it blank for an open AP.
	
	if (src->base.field_bit != 3 && !set_field(src, 3, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vwifi_config_ap_password pwifi_config_ap_password_SET(const char src[], size_t len, pwifi_config_ap_WIFI_CONFIG_AP *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 3, len);
	
	Vwifi_config_ap_password ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int8_t pdata96_typE_GET(pdata96_DATA96 *src) {//data type
	
	return (int8_t) src[0];
}


static inline void pdata96_typE_SET(int8_t src, pdata96_DATA96 *dst) { //data type
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pdata96_len_GET(pdata96_DATA96 *src) {//data length
	
	return (int8_t) src[1];
}


static inline void pdata96_len_SET(int8_t src, pdata96_DATA96 *dst) { //data length
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t vdata96_daTa_GET(Vdata96_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vdata96_daTa pdata96_daTa_GET(pdata96_DATA96 *src) { //raw data
	
	return (Vdata96_daTa) {src + 2, 96};
}


static inline void vdata96_daTa_SET(int8_t src, size_t index, Vdata96_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vdata96_daTa pdata96_daTa_SET(const int8_t src[], pdata96_DATA96 *dst) {//raw data
	
	
	Vdata96_daTa ret = {dst + 2, 96};
	
	if (src)
		for (size_t i = 0; i < 96; i++)
			vdata96_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int32_t pflight_information_time_boot_ms_GET(pflight_information_FLIGHT_INFORMATION *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pflight_information_time_boot_ms_SET(int32_t src, pflight_information_FLIGHT_INFORMATION *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int64_t pflight_information_arming_time_utc_GET(pflight_information_FLIGHT_INFORMATION *src) {//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
	
	return ((int64_t) (get_bytes(src, 4, 8)));
}


static inline void pflight_information_arming_time_utc_SET(int64_t src, pflight_information_FLIGHT_INFORMATION *dst) { //Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
	
	
	set_bytes((src), 8, dst, 4);
}


static inline int64_t pflight_information_takeoff_time_utc_GET(pflight_information_FLIGHT_INFORMATION *src) {//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
	
	return ((int64_t) (get_bytes(src, 12, 8)));
}


static inline void pflight_information_takeoff_time_utc_SET(int64_t src, pflight_information_FLIGHT_INFORMATION *dst) { //Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
	
	
	set_bytes((src), 8, dst, 12);
}


static inline int64_t pflight_information_flight_uuid_GET(pflight_information_FLIGHT_INFORMATION *src) {//Universally unique identifier (UUID) of flight, should correspond to name of logfiles
	
	return ((int64_t) (get_bytes(src, 20, 8)));
}


static inline void pflight_information_flight_uuid_SET(int64_t src, pflight_information_FLIGHT_INFORMATION *dst) { //Universally unique identifier (UUID) of flight, should correspond to name of logfiles
	
	
	set_bytes((src), 8, dst, 20);
}


static inline int16_t prc_channels_raw_chan1_raw_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int16_t prc_channels_raw_chan2_raw_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline int16_t prc_channels_raw_chan3_raw_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline int16_t prc_channels_raw_chan4_raw_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline int16_t prc_channels_raw_chan5_raw_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline int16_t prc_channels_raw_chan6_raw_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline int16_t prc_channels_raw_chan7_raw_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline int16_t prc_channels_raw_chan8_raw_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline int32_t prc_channels_raw_time_boot_ms_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 16, 4)));
}

/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
*					 8 servos */

static inline int8_t prc_channels_raw_port_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {
	
	return (int8_t) src[20];
}


static inline int8_t prc_channels_raw_rssi_GET(prc_channels_raw_RC_CHANNELS_RAW *src) {//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	
	return (int8_t) src[21];
}


static inline int16_t pservo_output_raw_servo1_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Servo output 1 value, in microseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int16_t pservo_output_raw_servo2_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Servo output 2 value, in microseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline int16_t pservo_output_raw_servo3_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Servo output 3 value, in microseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}


static inline int16_t pservo_output_raw_servo4_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Servo output 4 value, in microseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 6, 2)));
}


static inline int16_t pservo_output_raw_servo5_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Servo output 5 value, in microseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 8, 2)));
}


static inline int16_t pservo_output_raw_servo6_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Servo output 6 value, in microseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 10, 2)));
}


static inline int16_t pservo_output_raw_servo7_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Servo output 7 value, in microseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 12, 2)));
}


static inline int16_t pservo_output_raw_servo8_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Servo output 8 value, in microseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 14, 2)));
}


static inline int32_t pservo_output_raw_time_usec_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {//Timestamp (microseconds since system boot)
	
	return ((int32_t) (get_bytes(src->base.bytes, 16, 4)));
}

/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
*					 more than 8 servos */

static inline int8_t pservo_output_raw_port_GET(pservo_output_raw_SERVO_OUTPUT_RAW *src) {
	
	return (int8_t) src->base.bytes[20];
}

static inline bool pservo_output_raw_servo9_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *const src, int16_t *ret) {
	if (src->base.field_bit != 168 && !set_field(src, 168, -1)) return false;
	*ret = ((int16_t) (get_bytes(src->base.bytes, src->BYTE, 2)));
	return true;
}

static inline bool pservo_output_raw_servo10_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *const src, int16_t *ret) {
	if (src->base.field_bit != 169 && !set_field(src, 169, -1)) return false;
	*ret = ((int16_t) (get_bytes(src->base.bytes, src->BYTE, 2)));
	return true;
}

static inline bool pservo_output_raw_servo11_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *const src, int16_t *ret) {
	if (src->base.field_bit != 170 && !set_field(src, 170, -1)) return false;
	*ret = ((int16_t) (get_bytes(src->base.bytes, src->BYTE, 2)));
	return true;
}

static inline bool pservo_output_raw_servo12_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *const src, int16_t *ret) {
	if (src->base.field_bit != 171 && !set_field(src, 171, -1)) return false;
	*ret = ((int16_t) (get_bytes(src->base.bytes, src->BYTE, 2)));
	return true;
}

static inline bool pservo_output_raw_servo13_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *const src, int16_t *ret) {
	if (src->base.field_bit != 172 && !set_field(src, 172, -1)) return false;
	*ret = ((int16_t) (get_bytes(src->base.bytes, src->BYTE, 2)));
	return true;
}

static inline bool pservo_output_raw_servo14_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *const src, int16_t *ret) {
	if (src->base.field_bit != 173 && !set_field(src, 173, -1)) return false;
	*ret = ((int16_t) (get_bytes(src->base.bytes, src->BYTE, 2)));
	return true;
}

static inline bool pservo_output_raw_servo15_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *const src, int16_t *ret) {
	if (src->base.field_bit != 174 && !set_field(src, 174, -1)) return false;
	*ret = ((int16_t) (get_bytes(src->base.bytes, src->BYTE, 2)));
	return true;
}

static inline bool pservo_output_raw_servo16_raw_GET(pservo_output_raw_SERVO_OUTPUT_RAW *const src, int16_t *ret) {
	if (src->base.field_bit != 175 && !set_field(src, 175, -1)) return false;
	*ret = ((int16_t) (get_bytes(src->base.bytes, src->BYTE, 2)));
	return true;
}


static inline int16_t pmeminfo_brkval_GET(pmeminfo_MEMINFO *src) {//heap top
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pmeminfo_brkval_SET(int16_t src, pmeminfo_MEMINFO *dst) { //heap top
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t pmeminfo_freemem_GET(pmeminfo_MEMINFO *src) {//free memory
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pmeminfo_freemem_SET(int16_t src, pmeminfo_MEMINFO *dst) { //free memory
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}

static inline bool pmeminfo_freemem32_GET(pmeminfo_MEMINFO *const src, int32_t *ret) {
	if (src->base.field_bit != 32 && !set_field(src, 32, -1)) return false;
	*ret = ((int32_t) (get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline void pmeminfo_freemem32_SET(int32_t src, pmeminfo_MEMINFO *const dst) {
	
	if (dst->base.field_bit != 32) set_field(dst, 32, 0);
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, dst->BYTE);
}


static inline int16_t pmission_item_reached_seq_GET(pmission_item_reached_MISSION_ITEM_REACHED *src) {//Sequence
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int16_t plogging_ack_sequence_GET(plogging_ack_LOGGING_ACK *src) {//sequence number (must match the one in LOGGING_DATA_ACKED)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void plogging_ack_sequence_SET(int16_t src, plogging_ack_LOGGING_ACK *dst) { //sequence number (must match the one in LOGGING_DATA_ACKED)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int8_t plogging_ack_target_system_GET(plogging_ack_LOGGING_ACK *src) {//system ID of the target
	
	return (int8_t) src[2];
}


static inline void plogging_ack_target_system_SET(int8_t src, plogging_ack_LOGGING_ACK *dst) { //system ID of the target
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t plogging_ack_target_component_GET(plogging_ack_LOGGING_ACK *src) {//component ID of the target
	
	return (int8_t) src[3];
}


static inline void plogging_ack_target_component_SET(int8_t src, plogging_ack_LOGGING_ACK *dst) { //component ID of the target
	
	
	dst[3] = (uint8_t) (src);
}


static inline int64_t pvision_speed_estimate_usec_GET(pvision_speed_estimate_VISION_SPEED_ESTIMATE *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void pvision_speed_estimate_usec_SET(int64_t src, pvision_speed_estimate_VISION_SPEED_ESTIMATE *dst) { //Timestamp (microseconds, synced to UNIX time or since system boot)
	
	
	set_bytes((src), 8, dst, 0);
}


static inline float pvision_speed_estimate_x_GET(pvision_speed_estimate_VISION_SPEED_ESTIMATE *src) {//Global X speed
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pvision_speed_estimate_x_SET(float src, pvision_speed_estimate_VISION_SPEED_ESTIMATE *dst) { //Global X speed
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pvision_speed_estimate_y_GET(pvision_speed_estimate_VISION_SPEED_ESTIMATE *src) {//Global Y speed
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pvision_speed_estimate_y_SET(float src, pvision_speed_estimate_VISION_SPEED_ESTIMATE *dst) { //Global Y speed
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float pvision_speed_estimate_z_GET(pvision_speed_estimate_VISION_SPEED_ESTIMATE *src) {//Global Z speed
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void pvision_speed_estimate_z_SET(float src, pvision_speed_estimate_VISION_SPEED_ESTIMATE *dst) { //Global Z speed
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline int64_t pdebug_vect_time_usec_GET(pdebug_vect_DEBUG_VECT *src) {//Timestamp
	
	return ((int64_t) (get_bytes(src->base.bytes, 0, 8)));
}


static inline void pdebug_vect_time_usec_SET(int64_t src, pdebug_vect_DEBUG_VECT *dst) { //Timestamp
	
	
	set_bytes((src), 8, dst->base.bytes, 0);
}


static inline float pdebug_vect_x_GET(pdebug_vect_DEBUG_VECT *src) {//x
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline void pdebug_vect_x_SET(float src, pdebug_vect_DEBUG_VECT *dst) { //x
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 8);
}


static inline float pdebug_vect_y_GET(pdebug_vect_DEBUG_VECT *src) {//y
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline void pdebug_vect_y_SET(float src, pdebug_vect_DEBUG_VECT *dst) { //y
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 12);
}


static inline float pdebug_vect_z_GET(pdebug_vect_DEBUG_VECT *src) {//z
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline void pdebug_vect_z_SET(float src, pdebug_vect_DEBUG_VECT *dst) { //z
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 16);
}

static inline bool pdebug_vect_name_GET(pdebug_vect_DEBUG_VECT *const src, Vdebug_vect_name *ret) {//Name
	
	if (src->base.field_bit != 162 && !set_field(src, 162, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vdebug_vect_name pdebug_vect_name_SET(const char src[], size_t len, pdebug_vect_DEBUG_VECT *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 162, len);
	
	Vdebug_vect_name ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int8_t plog_request_end_target_system_GET(plog_request_end_LOG_REQUEST_END *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void plog_request_end_target_system_SET(int8_t src, plog_request_end_LOG_REQUEST_END *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t plog_request_end_target_component_GET(plog_request_end_LOG_REQUEST_END *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void plog_request_end_target_component_SET(int8_t src, plog_request_end_LOG_REQUEST_END *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t pmission_ack_target_system_GET(pmission_ack_MISSION_ACK *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pmission_ack_target_component_GET(pmission_ack_MISSION_ACK *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}

static inline bool pmission_ack_typE_GET(pmission_ack_MISSION_ACK *const src, e_MAV_MISSION_RESULT *ret) {
	if (src->base.field_bit != 18 && !set_field(src, 18, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline bool pmission_ack_mission_type_GET(pmission_ack_MISSION_ACK *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 19 && !set_field(src, 19, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline int8_t pchange_operator_control_ack_gcs_system_id_GET(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *src) {//ID of the GCS this message
	
	return (int8_t) src[0];
}


static inline int8_t pchange_operator_control_ack_control_request_GET(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *src) {//0: request control of this MAV, 1: Release control of this MAV
	
	return (int8_t) src[1];
}

/**
*0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
*					 contro */

static inline int8_t pchange_operator_control_ack_ack_GET(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *src) {
	
	return (int8_t) src[2];
}


static inline int16_t pmission_current_seq_GET(pmission_current_MISSION_CURRENT *src) {//Sequence
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int32_t psystem_time_time_boot_ms_GET(psystem_time_SYSTEM_TIME *src) {//Timestamp of the component clock since boot time in milliseconds.
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline int64_t psystem_time_time_unix_usec_GET(psystem_time_SYSTEM_TIME *src) {//Timestamp of the master clock in microseconds since UNIX epoch.
	
	return ((int64_t) (get_bytes(src, 4, 8)));
}


static inline int32_t pcamera_trigger_seq_GET(pcamera_trigger_CAMERA_TRIGGER *src) {//Image frame sequence
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pcamera_trigger_seq_SET(int32_t src, pcamera_trigger_CAMERA_TRIGGER *dst) { //Image frame sequence
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int64_t pcamera_trigger_time_usec_GET(pcamera_trigger_CAMERA_TRIGGER *src) {//Timestamp for the image frame in microseconds
	
	return ((int64_t) (get_bytes(src, 4, 8)));
}


static inline void pcamera_trigger_time_usec_SET(int64_t src, pcamera_trigger_CAMERA_TRIGGER *dst) { //Timestamp for the image frame in microseconds
	
	
	set_bytes((src), 8, dst, 4);
}

static inline bool pgopro_set_response_cmd_id_GET(pgopro_set_response_GOPRO_SET_RESPONSE *const src, e_GOPRO_COMMAND *ret) {
	if (src->base.field_bit != 0 && !set_field(src, 0, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline void pgopro_set_response_cmd_id_SET(e_GOPRO_COMMAND src, pgopro_set_response_GOPRO_SET_RESPONSE *const dst) {
	
	if (dst->base.field_bit != 0) set_field(dst, 0, 0);
	
	
	set_bits(src, 5, dst->base.bytes, dst->BIT);
}

static inline bool pgopro_set_response_status_GET(pgopro_set_response_GOPRO_SET_RESPONSE *const src, e_GOPRO_REQUEST_STATUS *ret) {
	if (src->base.field_bit != 1 && !set_field(src, 1, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void pgopro_set_response_status_SET(e_GOPRO_REQUEST_STATUS src, pgopro_set_response_GOPRO_SET_RESPONSE *const dst) {
	
	if (dst->base.field_bit != 1) set_field(dst, 1, 0);
	
	
	set_bits(src, 1, dst->base.bytes, dst->BIT);
}


static inline int64_t pvision_position_estimate_usec_GET(pvision_position_estimate_VISION_POSITION_ESTIMATE *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline float pvision_position_estimate_x_GET(pvision_position_estimate_VISION_POSITION_ESTIMATE *src) {//Global X position
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline float pvision_position_estimate_y_GET(pvision_position_estimate_VISION_POSITION_ESTIMATE *src) {//Global Y position
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float pvision_position_estimate_z_GET(pvision_position_estimate_VISION_POSITION_ESTIMATE *src) {//Global Z position
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline float pvision_position_estimate_roll_GET(pvision_position_estimate_VISION_POSITION_ESTIMATE *src) {//Roll angle in rad
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline float pvision_position_estimate_pitch_GET(pvision_position_estimate_VISION_POSITION_ESTIMATE *src) {//Pitch angle in rad
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline float pvision_position_estimate_yaw_GET(pvision_position_estimate_VISION_POSITION_ESTIMATE *src) {//Yaw angle in rad
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}

/**
*A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
*					 bit corresponds to Button 1 */

static inline int16_t pmanual_control_buttons_GET(pmanual_control_MANUAL_CONTROL *src) {
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int8_t pmanual_control_target_GET(pmanual_control_MANUAL_CONTROL *src) {//The system to be controlled.
	
	return (int8_t) src[2];
}

/**
*X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*					 Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle */

static inline int16_t pmanual_control_x_GET(pmanual_control_MANUAL_CONTROL *src) {
	
	return ((int16_t) (get_bytes(src, 3, 2)));
}

/**
*Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*					 Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle */

static inline int16_t pmanual_control_y_GET(pmanual_control_MANUAL_CONTROL *src) {
	
	return ((int16_t) (get_bytes(src, 5, 2)));
}

/**
*Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*					 Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
*					 a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
*					 thrust */

static inline int16_t pmanual_control_z_GET(pmanual_control_MANUAL_CONTROL *src) {
	
	return ((int16_t) (get_bytes(src, 7, 2)));
}

/**
*R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*					 Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
*					 being -1000, and the yaw of a vehicle */

static inline int16_t pmanual_control_r_GET(pmanual_control_MANUAL_CONTROL *src) {
	
	return ((int16_t) (get_bytes(src, 9, 2)));
}


static inline int16_t prc_channels_chan1_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline int16_t prc_channels_chan2_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline int16_t prc_channels_chan3_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline int16_t prc_channels_chan4_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline int16_t prc_channels_chan5_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline int16_t prc_channels_chan6_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline int16_t prc_channels_chan7_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline int16_t prc_channels_chan8_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline int16_t prc_channels_chan9_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 16, 2)));
}


static inline int16_t prc_channels_chan10_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 18, 2)));
}


static inline int16_t prc_channels_chan11_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 20, 2)));
}


static inline int16_t prc_channels_chan12_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 22, 2)));
}


static inline int16_t prc_channels_chan13_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 24, 2)));
}


static inline int16_t prc_channels_chan14_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 26, 2)));
}


static inline int16_t prc_channels_chan15_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 28, 2)));
}


static inline int16_t prc_channels_chan16_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 30, 2)));
}


static inline int16_t prc_channels_chan17_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 32, 2)));
}


static inline int16_t prc_channels_chan18_raw_GET(prc_channels_RC_CHANNELS *src) {//RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	
	return ((int16_t) (get_bytes(src, 34, 2)));
}


static inline int32_t prc_channels_time_boot_ms_GET(prc_channels_RC_CHANNELS *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 36, 4)));
}

/**
*Total number of RC channels being received. This can be larger than 18, indicating that more channels
*					 are available but not given in this message. This value should be 0 when no RC channels are available */

static inline int8_t prc_channels_chancount_GET(prc_channels_RC_CHANNELS *src) {
	
	return (int8_t) src[40];
}


static inline int8_t prc_channels_rssi_GET(prc_channels_RC_CHANNELS *src) {//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	
	return (int8_t) src[41];
}


static inline int16_t pprotocol_version_version_GET(pprotocol_version_PROTOCOL_VERSION *src) {//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pprotocol_version_version_SET(int16_t src, pprotocol_version_PROTOCOL_VERSION *dst) { //Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t pprotocol_version_min_version_GET(pprotocol_version_PROTOCOL_VERSION *src) {//Minimum MAVLink version supported
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pprotocol_version_min_version_SET(int16_t src, pprotocol_version_PROTOCOL_VERSION *dst) { //Minimum MAVLink version supported
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t pprotocol_version_max_version_GET(pprotocol_version_PROTOCOL_VERSION *src) {//Maximum MAVLink version supported (set to the same value as version by default)
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void pprotocol_version_max_version_SET(int16_t src, pprotocol_version_PROTOCOL_VERSION *dst) { //Maximum MAVLink version supported (set to the same value as version by default)
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int8_t vprotocol_version_spec_version_hash_GET(Vprotocol_version_spec_version_hash const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vprotocol_version_spec_version_hash pprotocol_version_spec_version_hash_GET(pprotocol_version_PROTOCOL_VERSION *src) { //The first 8 bytes (not characters printed in hex!) of the git hash.
	
	return (Vprotocol_version_spec_version_hash) {src + 6, 8};
}


static inline void vprotocol_version_spec_version_hash_SET(int8_t src, size_t index, Vprotocol_version_spec_version_hash *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vprotocol_version_spec_version_hash pprotocol_version_spec_version_hash_SET(const int8_t src[], pprotocol_version_PROTOCOL_VERSION *dst) {//The first 8 bytes (not characters printed in hex!) of the git hash.
	
	
	Vprotocol_version_spec_version_hash ret = {dst + 6, 8};
	
	if (src)
		for (size_t i = 0; i < 8; i++)
			vprotocol_version_spec_version_hash_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t vprotocol_version_library_version_hash_GET(Vprotocol_version_library_version_hash const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vprotocol_version_library_version_hash pprotocol_version_library_version_hash_GET(pprotocol_version_PROTOCOL_VERSION *src) { //The first 8 bytes (not characters printed in hex!) of the git hash.
	
	return (Vprotocol_version_library_version_hash) {src + 14, 8};
}


static inline void vprotocol_version_library_version_hash_SET(int8_t src, size_t index, Vprotocol_version_library_version_hash *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vprotocol_version_library_version_hash pprotocol_version_library_version_hash_SET(const int8_t src[], pprotocol_version_PROTOCOL_VERSION *dst) {//The first 8 bytes (not characters printed in hex!) of the git hash.
	
	
	Vprotocol_version_library_version_hash ret = {dst + 14, 8};
	
	if (src)
		for (size_t i = 0; i < 8; i++)
			vprotocol_version_library_version_hash_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t prally_fetch_point_target_system_GET(prally_fetch_point_RALLY_FETCH_POINT *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void prally_fetch_point_target_system_SET(int8_t src, prally_fetch_point_RALLY_FETCH_POINT *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t prally_fetch_point_target_component_GET(prally_fetch_point_RALLY_FETCH_POINT *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void prally_fetch_point_target_component_SET(int8_t src, prally_fetch_point_RALLY_FETCH_POINT *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t prally_fetch_point_idx_GET(prally_fetch_point_RALLY_FETCH_POINT *src) {//point index (first point is 0)
	
	return (int8_t) src[2];
}


static inline void prally_fetch_point_idx_SET(int8_t src, prally_fetch_point_RALLY_FETCH_POINT *dst) { //point index (first point is 0)
	
	
	dst[2] = (uint8_t) (src);
}


static inline int16_t pparam_value_param_count_GET(pparam_value_PARAM_VALUE *src) {//Total number of onboard parameters
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int16_t pparam_value_param_index_GET(pparam_value_PARAM_VALUE *src) {//Index of this onboard parameter
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline float pparam_value_param_value_GET(pparam_value_PARAM_VALUE *src) {//Onboard parameter value
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 4, 4)));
}

/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*					 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*					 storage if the ID is stored as strin */

static inline bool pparam_value_param_id_GET(pparam_value_PARAM_VALUE *const src, Vparam_value_param_id *ret) {
	
	if (src->base.field_bit != 66 && !set_field(src, 66, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline bool pparam_value_param_type_GET(pparam_value_PARAM_VALUE *const src, e_MAV_PARAM_TYPE *ret) {
	if (src->base.field_bit != 67 && !set_field(src, 67, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*					 should have the UINT16_MAX value */

static inline int16_t vbattery_status_voltages_GET(Vbattery_status_voltages const *const src, size_t index) { return ((int16_t) (get_bytes(src->bytes, index * 2, 2))); }

/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*					 should have the UINT16_MAX value */

static inline Vbattery_status_voltages pbattery_status_voltages_GET(pbattery_status_BATTERY_STATUS *src) {
	
	return (Vbattery_status_voltages) {src->base.bytes + 0, 10};
}

/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*					 should have the UINT16_MAX value */

static inline void vbattery_status_voltages_SET(int16_t src, size_t index, Vbattery_status_voltages *dst) { set_bytes((uint16_t) (src), 2, dst->bytes, index * 2); }

/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*					 should have the UINT16_MAX value */

static inline Vbattery_status_voltages pbattery_status_voltages_SET(const int16_t src[], pbattery_status_BATTERY_STATUS *dst) {
	
	
	Vbattery_status_voltages ret = {dst->base.bytes + 0, 10};
	
	if (src)
		for (size_t i = 0; i < 10; i++)
			vbattery_status_voltages_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t pbattery_status_id_GET(pbattery_status_BATTERY_STATUS *src) {//Battery ID
	
	return (int8_t) src->base.bytes[20];
}


static inline void pbattery_status_id_SET(int8_t src, pbattery_status_BATTERY_STATUS *dst) { //Battery ID
	
	
	dst->base.bytes[20] = (uint8_t) (src);
}


static inline int16_t pbattery_status_temperature_GET(pbattery_status_BATTERY_STATUS *src) {//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
	
	return ((int16_t) (get_bytes(src->base.bytes, 21, 2)));
}


static inline void pbattery_status_temperature_SET(int16_t src, pbattery_status_BATTERY_STATUS *dst) { //Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 21);
}


static inline int16_t pbattery_status_current_battery_GET(pbattery_status_BATTERY_STATUS *src) {//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
	
	return ((int16_t) (get_bytes(src->base.bytes, 23, 2)));
}


static inline void pbattery_status_current_battery_SET(int16_t src, pbattery_status_BATTERY_STATUS *dst) { //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 23);
}


static inline int32_t pbattery_status_current_consumed_GET(pbattery_status_BATTERY_STATUS *src) {//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
	
	return ((int32_t) (get_bytes(src->base.bytes, 25, 4)));
}


static inline void pbattery_status_current_consumed_SET(int32_t src, pbattery_status_BATTERY_STATUS *dst) { //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 25);
}

/**
*Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
*					 energy consumption estimat */

static inline int32_t pbattery_status_energy_consumed_GET(pbattery_status_BATTERY_STATUS *src) {
	
	return ((int32_t) (get_bytes(src->base.bytes, 29, 4)));
}

/**
*Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
*					 energy consumption estimat */

static inline void pbattery_status_energy_consumed_SET(int32_t src, pbattery_status_BATTERY_STATUS *dst) {
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 29);
}


static inline int8_t pbattery_status_battery_remaining_GET(pbattery_status_BATTERY_STATUS *src) {//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
	
	return (int8_t) src->base.bytes[33];
}


static inline void pbattery_status_battery_remaining_SET(int8_t src, pbattery_status_BATTERY_STATUS *dst) { //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
	
	
	dst->base.bytes[33] = (uint8_t) (src);
}

static inline bool pbattery_status_battery_function_GET(pbattery_status_BATTERY_STATUS *const src, e_MAV_BATTERY_FUNCTION *ret) {
	if (src->base.field_bit != 272 && !set_field(src, 272, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pbattery_status_battery_function_SET(e_MAV_BATTERY_FUNCTION src, pbattery_status_BATTERY_STATUS *const dst) {
	
	if (dst->base.field_bit != 272) set_field(dst, 272, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool pbattery_status_typE_GET(pbattery_status_BATTERY_STATUS *const src, e_MAV_BATTERY_TYPE *ret) {
	if (src->base.field_bit != 273 && !set_field(src, 273, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pbattery_status_typE_SET(e_MAV_BATTERY_TYPE src, pbattery_status_BATTERY_STATUS *const dst) {
	
	if (dst->base.field_bit != 273) set_field(dst, 273, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}


static inline int16_t pserial_control_timeout_GET(pserial_control_SERIAL_CONTROL *src) {//Timeout for reply data in milliseconds
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pserial_control_timeout_SET(int16_t src, pserial_control_SERIAL_CONTROL *dst) { //Timeout for reply data in milliseconds
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int32_t pserial_control_baudrate_GET(pserial_control_SERIAL_CONTROL *src) {//Baudrate of transfer. Zero means no change.
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline void pserial_control_baudrate_SET(int32_t src, pserial_control_SERIAL_CONTROL *dst) { //Baudrate of transfer. Zero means no change.
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 2);
}


static inline int8_t pserial_control_count_GET(pserial_control_SERIAL_CONTROL *src) {//how many bytes in this transfer
	
	return (int8_t) src->base.bytes[6];
}


static inline void pserial_control_count_SET(int8_t src, pserial_control_SERIAL_CONTROL *dst) { //how many bytes in this transfer
	
	
	dst->base.bytes[6] = (uint8_t) (src);
}


static inline int8_t vserial_control_daTa_GET(Vserial_control_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vserial_control_daTa pserial_control_daTa_GET(pserial_control_SERIAL_CONTROL *src) { //serial data
	
	return (Vserial_control_daTa) {src->base.bytes + 7, 70};
}


static inline void vserial_control_daTa_SET(int8_t src, size_t index, Vserial_control_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vserial_control_daTa pserial_control_daTa_SET(const int8_t src[], pserial_control_SERIAL_CONTROL *dst) {//serial data
	
	
	Vserial_control_daTa ret = {dst->base.bytes + 7, 70};
	
	if (src)
		for (size_t i = 0; i < 70; i++)
			vserial_control_daTa_SET(src[i], i, &ret);
	return ret;
}

static inline bool pserial_control_device_GET(pserial_control_SERIAL_CONTROL *const src, e_SERIAL_CONTROL_DEV *ret) {
	if (src->base.field_bit != 616 && !set_field(src, 616, -1)) return false;
	*ret = _en_serial_control_dev(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pserial_control_device_SET(e_SERIAL_CONTROL_DEV src, pserial_control_SERIAL_CONTROL *const dst) {
	
	if (dst->base.field_bit != 616) set_field(dst, 616, 0);
	
	
	UMAX id = _id_serial_control_dev(src);
	set_bits(id, 3, dst->base.bytes, dst->BIT);
}

static inline bool pserial_control_flags_GET(pserial_control_SERIAL_CONTROL *const src, e_SERIAL_CONTROL_FLAG *ret) {
	if (src->base.field_bit != 617 && !set_field(src, 617, -1)) return false;
	*ret = _en_serial_control_flag(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pserial_control_flags_SET(e_SERIAL_CONTROL_FLAG src, pserial_control_SERIAL_CONTROL *const dst) {
	
	if (dst->base.field_bit != 617) set_field(dst, 617, 0);
	
	
	UMAX id = _id_serial_control_flag(src);
	set_bits(id, 3, dst->base.bytes, dst->BIT);
}

/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*					 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*					 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*					 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*					 bit 11: yaw, bit 12: yaw rat */

static inline int16_t pset_position_target_local_ned_type_mask_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int32_t pset_position_target_local_ned_time_boot_ms_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//Timestamp in milliseconds since system boot
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline int8_t pset_position_target_local_ned_target_system_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//System ID
	
	return (int8_t) src->base.bytes[6];
}


static inline int8_t pset_position_target_local_ned_target_component_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//Component ID
	
	return (int8_t) src->base.bytes[7];
}


static inline float pset_position_target_local_ned_x_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//X Position in NED frame in meters
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline float pset_position_target_local_ned_y_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//Y Position in NED frame in meters
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline float pset_position_target_local_ned_z_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//Z Position in NED frame in meters (note, altitude is negative in NED)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline float pset_position_target_local_ned_vx_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//X velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 20, 4)));
}


static inline float pset_position_target_local_ned_vy_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//Y velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 24, 4)));
}


static inline float pset_position_target_local_ned_vz_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//Z velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}


static inline float pset_position_target_local_ned_afx_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 32, 4)));
}


static inline float pset_position_target_local_ned_afy_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 36, 4)));
}


static inline float pset_position_target_local_ned_afz_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 40, 4)));
}


static inline float pset_position_target_local_ned_yaw_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//yaw setpoint in rad
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 44, 4)));
}


static inline float pset_position_target_local_ned_yaw_rate_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *src) {//yaw rate setpoint in rad/s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 48, 4)));
}

static inline bool pset_position_target_local_ned_coordinate_frame_GET(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 416 && !set_field(src, 416, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}


static inline int32_t pmount_orientation_time_boot_ms_GET(pmount_orientation_MOUNT_ORIENTATION *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pmount_orientation_time_boot_ms_SET(int32_t src, pmount_orientation_MOUNT_ORIENTATION *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline float pmount_orientation_roll_GET(pmount_orientation_MOUNT_ORIENTATION *src) {//Roll in degrees
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pmount_orientation_roll_SET(float src, pmount_orientation_MOUNT_ORIENTATION *dst) { //Roll in degrees
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pmount_orientation_pitch_GET(pmount_orientation_MOUNT_ORIENTATION *src) {//Pitch in degrees
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pmount_orientation_pitch_SET(float src, pmount_orientation_MOUNT_ORIENTATION *dst) { //Pitch in degrees
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pmount_orientation_yaw_GET(pmount_orientation_MOUNT_ORIENTATION *src) {//Yaw in degrees
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pmount_orientation_yaw_SET(float src, pmount_orientation_MOUNT_ORIENTATION *dst) { //Yaw in degrees
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline int8_t pset_gps_global_origin_target_system_GET(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int32_t pset_gps_global_origin_latitude_GET(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *src) {//Latitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 1, 4)));
}


static inline int32_t pset_gps_global_origin_longitude_GET(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *src) {//Longitude (WGS84, in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 5, 4)));
}


static inline int32_t pset_gps_global_origin_altitude_GET(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *src) {//Altitude (AMSL), in meters * 1000 (positive for up)
	
	return ((int32_t) (get_bytes(src->base.bytes, 9, 4)));
}

static inline bool pset_gps_global_origin_time_usec_GET(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *const src, int64_t *ret) {
	if (src->base.field_bit != 104 && !set_field(src, 104, -1)) return false;
	*ret = ((int64_t) (get_bytes(src->base.bytes, src->BYTE, 8)));
	return true;
}


static inline int8_t pparam_ext_set_target_system_GET(pparam_ext_set_PARAM_EXT_SET *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline void pparam_ext_set_target_system_SET(int8_t src, pparam_ext_set_PARAM_EXT_SET *dst) { //System ID
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t pparam_ext_set_target_component_GET(pparam_ext_set_PARAM_EXT_SET *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline void pparam_ext_set_target_component_SET(int8_t src, pparam_ext_set_PARAM_EXT_SET *dst) { //Component ID
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}

/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*					 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*					 ID is stored as strin */

static inline bool pparam_ext_set_param_id_GET(pparam_ext_set_PARAM_EXT_SET *const src, Vparam_ext_set_param_id *ret) {
	
	if (src->base.field_bit != 19 && !set_field(src, 19, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vparam_ext_set_param_id pparam_ext_set_param_id_SET(const char src[], size_t len, pparam_ext_set_PARAM_EXT_SET *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 19, len);
	
	Vparam_ext_set_param_id ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pparam_ext_set_param_value_GET(pparam_ext_set_PARAM_EXT_SET *const src, Vparam_ext_set_param_value *ret) {//Parameter value
	
	if (src->base.field_bit != 20 && !set_field(src, 20, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vparam_ext_set_param_value pparam_ext_set_param_value_SET(const char src[], size_t len, pparam_ext_set_PARAM_EXT_SET *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 20, len);
	
	Vparam_ext_set_param_value ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pparam_ext_set_param_type_GET(pparam_ext_set_PARAM_EXT_SET *const src, e_MAV_PARAM_EXT_TYPE *ret) {
	if (src->base.field_bit != 21 && !set_field(src, 21, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pparam_ext_set_param_type_SET(e_MAV_PARAM_EXT_TYPE src, pparam_ext_set_PARAM_EXT_SET *const dst) {
	
	if (dst->base.field_bit != 21) set_field(dst, 21, 0);
	
	
	set_bits(-1
	src, 4, dst->base.bytes, dst->BIT);
}


static inline int16_t pautopilot_version_vendor_id_GET(pautopilot_version_AUTOPILOT_VERSION *src) {//ID of the board vendor
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pautopilot_version_vendor_id_SET(int16_t src, pautopilot_version_AUTOPILOT_VERSION *dst) { //ID of the board vendor
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t pautopilot_version_product_id_GET(pautopilot_version_AUTOPILOT_VERSION *src) {//ID of the product
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pautopilot_version_product_id_SET(int16_t src, pautopilot_version_AUTOPILOT_VERSION *dst) { //ID of the product
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int32_t pautopilot_version_flight_sw_version_GET(pautopilot_version_AUTOPILOT_VERSION *src) {//Firmware version number
	
	return ((int32_t) (get_bytes(src->base.bytes, 4, 4)));
}


static inline void pautopilot_version_flight_sw_version_SET(int32_t src, pautopilot_version_AUTOPILOT_VERSION *dst) { //Firmware version number
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 4);
}


static inline int32_t pautopilot_version_middleware_sw_version_GET(pautopilot_version_AUTOPILOT_VERSION *src) {//Middleware version number
	
	return ((int32_t) (get_bytes(src->base.bytes, 8, 4)));
}


static inline void pautopilot_version_middleware_sw_version_SET(int32_t src, pautopilot_version_AUTOPILOT_VERSION *dst) { //Middleware version number
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 8);
}


static inline int32_t pautopilot_version_os_sw_version_GET(pautopilot_version_AUTOPILOT_VERSION *src) {//Operating system version number
	
	return ((int32_t) (get_bytes(src->base.bytes, 12, 4)));
}


static inline void pautopilot_version_os_sw_version_SET(int32_t src, pautopilot_version_AUTOPILOT_VERSION *dst) { //Operating system version number
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 12);
}


static inline int32_t pautopilot_version_board_version_GET(pautopilot_version_AUTOPILOT_VERSION *src) {//HW / board version (last 8 bytes should be silicon ID, if any)
	
	return ((int32_t) (get_bytes(src->base.bytes, 16, 4)));
}


static inline void pautopilot_version_board_version_SET(int32_t src, pautopilot_version_AUTOPILOT_VERSION *dst) { //HW / board version (last 8 bytes should be silicon ID, if any)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 16);
}


static inline int64_t pautopilot_version_uid_GET(pautopilot_version_AUTOPILOT_VERSION *src) {//UID if provided by hardware (see uid2)
	
	return ((int64_t) (get_bytes(src->base.bytes, 20, 8)));
}


static inline void pautopilot_version_uid_SET(int64_t src, pautopilot_version_AUTOPILOT_VERSION *dst) { //UID if provided by hardware (see uid2)
	
	
	set_bytes((src), 8, dst->base.bytes, 20);
}

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline int8_t vautopilot_version_flight_custom_version_GET(Vautopilot_version_flight_custom_version const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline Vautopilot_version_flight_custom_version pautopilot_version_flight_custom_version_GET(pautopilot_version_AUTOPILOT_VERSION *src) {
	
	return (Vautopilot_version_flight_custom_version) {src->base.bytes + 28, 8};
}

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline void vautopilot_version_flight_custom_version_SET(int8_t src, size_t index, Vautopilot_version_flight_custom_version *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline Vautopilot_version_flight_custom_version pautopilot_version_flight_custom_version_SET(const int8_t src[], pautopilot_version_AUTOPILOT_VERSION *dst) {
	
	
	Vautopilot_version_flight_custom_version ret = {dst->base.bytes + 28, 8};
	
	if (src)
		for (size_t i = 0; i < 8; i++)
			vautopilot_version_flight_custom_version_SET(src[i], i, &ret);
	return ret;
}

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline int8_t vautopilot_version_middleware_custom_version_GET(Vautopilot_version_middleware_custom_version const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline Vautopilot_version_middleware_custom_version pautopilot_version_middleware_custom_version_GET(pautopilot_version_AUTOPILOT_VERSION *src) {
	
	return (Vautopilot_version_middleware_custom_version) {src->base.bytes + 36, 8};
}

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline void vautopilot_version_middleware_custom_version_SET(int8_t src, size_t index, Vautopilot_version_middleware_custom_version *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline Vautopilot_version_middleware_custom_version pautopilot_version_middleware_custom_version_SET(const int8_t src[], pautopilot_version_AUTOPILOT_VERSION *dst) {
	
	
	Vautopilot_version_middleware_custom_version ret = {dst->base.bytes + 36, 8};
	
	if (src)
		for (size_t i = 0; i < 8; i++)
			vautopilot_version_middleware_custom_version_SET(src[i], i, &ret);
	return ret;
}

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline int8_t vautopilot_version_os_custom_version_GET(Vautopilot_version_os_custom_version const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline Vautopilot_version_os_custom_version pautopilot_version_os_custom_version_GET(pautopilot_version_AUTOPILOT_VERSION *src) {
	
	return (Vautopilot_version_os_custom_version) {src->base.bytes + 44, 8};
}

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline void vautopilot_version_os_custom_version_SET(int8_t src, size_t index, Vautopilot_version_os_custom_version *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*					 should allow to identify the commit using the main version number even for very large code bases */

static inline Vautopilot_version_os_custom_version pautopilot_version_os_custom_version_SET(const int8_t src[], pautopilot_version_AUTOPILOT_VERSION *dst) {
	
	
	Vautopilot_version_os_custom_version ret = {dst->base.bytes + 44, 8};
	
	if (src)
		for (size_t i = 0; i < 8; i++)
			vautopilot_version_os_custom_version_SET(src[i], i, &ret);
	return ret;
}

static inline bool pautopilot_version_capabilities_GET(pautopilot_version_AUTOPILOT_VERSION *const src, e_MAV_PROTOCOL_CAPABILITY *ret) {
	if (src->base.field_bit != 419 && !set_field(src, 419, -1)) return false;
	*ret = _en_mav_protocol_capability(get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline void pautopilot_version_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY src, pautopilot_version_AUTOPILOT_VERSION *const dst) {
	
	if (dst->base.field_bit != 419) set_field(dst, 419, 0);
	
	
	UMAX id = _id_mav_protocol_capability(src);
	set_bits(id, 5, dst->base.bytes, dst->BIT);
}

/**
															 * brief Getting pointer to the field
															 * param pautopilot_version pointer to pack
															 * return pointer to current field
															 * if field empty: return NULL
															 */
static inline Qautopilot_version_uid2 *pautopilot_version_uid2(pautopilot_version_AUTOPILOT_VERSION *pautopilot_version) {
	if (pautopilot_version->base.field_bit != 420 && !set_field(pautopilot_version, 420, -1)) return NULL;
	return pautopilot_version;
}

static inline int8_t vautopilot_version_uid2_GET(Vautopilot_version_uid2 const *const src) { return (int8_t) src->bytes[0]; }

static inline bool qautopilot_version_uid2_GET(Qautopilot_version_uid2 *const src, Vautopilot_version_uid2 *ret, size_t d0) {
	
	if (!set_item(src, d0, -1)) return false;
	
	
	ret->bytes = src->base.bytes + src->BYTE;
	
	return true;
}

static inline void vautopilot_version_uid2_SET(int8_t src, Vautopilot_version_uid2 const *const dst) { dst->bytes[0] = (uint8_t) (src); }

static inline Vautopilot_version_uid2 pautopilot_version_uid2_SET(int8_t src, pautopilot_version_AUTOPILOT_VERSION *const dst, size_t d0) {
	
	if (dst->base.field_bit != 420) set_field(dst, 420, 0);
	set_item(dst, d0, 0);
	Vautopilot_version_uid2 ret = {dst->base.bytes + dst->BYTE};
	
	vautopilot_version_uid2_SET(src, &ret);
	return ret;
}


static inline int8_t pmission_request_list_target_system_GET(pmission_request_list_MISSION_REQUEST_LIST *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pmission_request_list_target_component_GET(pmission_request_list_MISSION_REQUEST_LIST *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}

static inline bool pmission_request_list_mission_type_GET(pmission_request_list_MISSION_REQUEST_LIST *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 16 && !set_field(src, 16, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline float psimstate_roll_GET(psimstate_SIMSTATE *src) {//Roll angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void psimstate_roll_SET(float src, psimstate_SIMSTATE *dst) { //Roll angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float psimstate_pitch_GET(psimstate_SIMSTATE *src) {//Pitch angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void psimstate_pitch_SET(float src, psimstate_SIMSTATE *dst) { //Pitch angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float psimstate_yaw_GET(psimstate_SIMSTATE *src) {//Yaw angle (rad)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void psimstate_yaw_SET(float src, psimstate_SIMSTATE *dst) { //Yaw angle (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float psimstate_xacc_GET(psimstate_SIMSTATE *src) {//X acceleration m/s/s
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void psimstate_xacc_SET(float src, psimstate_SIMSTATE *dst) { //X acceleration m/s/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float psimstate_yacc_GET(psimstate_SIMSTATE *src) {//Y acceleration m/s/s
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void psimstate_yacc_SET(float src, psimstate_SIMSTATE *dst) { //Y acceleration m/s/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float psimstate_zacc_GET(psimstate_SIMSTATE *src) {//Z acceleration m/s/s
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void psimstate_zacc_SET(float src, psimstate_SIMSTATE *dst) { //Z acceleration m/s/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float psimstate_xgyro_GET(psimstate_SIMSTATE *src) {//Angular speed around X axis rad/s
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void psimstate_xgyro_SET(float src, psimstate_SIMSTATE *dst) { //Angular speed around X axis rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float psimstate_ygyro_GET(psimstate_SIMSTATE *src) {//Angular speed around Y axis rad/s
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void psimstate_ygyro_SET(float src, psimstate_SIMSTATE *dst) { //Angular speed around Y axis rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float psimstate_zgyro_GET(psimstate_SIMSTATE *src) {//Angular speed around Z axis rad/s
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void psimstate_zgyro_SET(float src, psimstate_SIMSTATE *dst) { //Angular speed around Z axis rad/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}


static inline int32_t psimstate_lat_GET(psimstate_SIMSTATE *src) {//Latitude in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 36, 4)));
}


static inline void psimstate_lat_SET(int32_t src, psimstate_SIMSTATE *dst) { //Latitude in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 36);
}


static inline int32_t psimstate_lng_GET(psimstate_SIMSTATE *src) {//Longitude in degrees * 1E7
	
	return ((int32_t) (get_bytes(src, 40, 4)));
}


static inline void psimstate_lng_SET(int32_t src, psimstate_SIMSTATE *dst) { //Longitude in degrees * 1E7
	
	
	set_bytes((uint32_t) (src), 4, dst, 40);
}


static inline int16_t pset_video_stream_settings_resolution_h_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *src) {//Resolution horizontal in pixels (set to -1 for highest resolution possible)
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pset_video_stream_settings_resolution_h_SET(int16_t src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *dst) { //Resolution horizontal in pixels (set to -1 for highest resolution possible)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t pset_video_stream_settings_resolution_v_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *src) {//Resolution vertical in pixels (set to -1 for highest resolution possible)
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pset_video_stream_settings_resolution_v_SET(int16_t src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *dst) { //Resolution vertical in pixels (set to -1 for highest resolution possible)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int16_t pset_video_stream_settings_rotation_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *src) {//Video image rotation clockwise (0-359 degrees)
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}


static inline void pset_video_stream_settings_rotation_SET(int16_t src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *dst) { //Video image rotation clockwise (0-359 degrees)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 4);
}


static inline int32_t pset_video_stream_settings_bitrate_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *src) {//Bit rate in bits per second (set to -1 for auto)
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline void pset_video_stream_settings_bitrate_SET(int32_t src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *dst) { //Bit rate in bits per second (set to -1 for auto)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 6);
}


static inline int8_t pset_video_stream_settings_target_system_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *src) {//system ID of the target
	
	return (int8_t) src->base.bytes[10];
}


static inline void pset_video_stream_settings_target_system_SET(int8_t src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *dst) { //system ID of the target
	
	
	dst->base.bytes[10] = (uint8_t) (src);
}


static inline int8_t pset_video_stream_settings_target_component_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *src) {//component ID of the target
	
	return (int8_t) src->base.bytes[11];
}


static inline void pset_video_stream_settings_target_component_SET(int8_t src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *dst) { //component ID of the target
	
	
	dst->base.bytes[11] = (uint8_t) (src);
}


static inline int8_t pset_video_stream_settings_camera_id_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *src) {//Camera ID (1 for first, 2 for second, etc.)
	
	return (int8_t) src->base.bytes[12];
}


static inline void pset_video_stream_settings_camera_id_SET(int8_t src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *dst) { //Camera ID (1 for first, 2 for second, etc.)
	
	
	dst->base.bytes[12] = (uint8_t) (src);
}


static inline float pset_video_stream_settings_framerate_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *src) {//Frames per second (set to -1 for highest framerate possible)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 13, 4)));
}


static inline void pset_video_stream_settings_framerate_SET(float src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *dst) { //Frames per second (set to -1 for highest framerate possible)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 13);
}

static inline bool pset_video_stream_settings_uri_GET(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *const src, Vset_video_stream_settings_uri *ret) {//Video stream URI
	
	if (src->base.field_bit != 138 && !set_field(src, 138, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vset_video_stream_settings_uri pset_video_stream_settings_uri_SET(const char src[], size_t len, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 138, len);
	
	Vset_video_stream_settings_uri ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int8_t pplay_tune_target_system_GET(pplay_tune_PLAY_TUNE *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline void pplay_tune_target_system_SET(int8_t src, pplay_tune_PLAY_TUNE *dst) { //System ID
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t pplay_tune_target_component_GET(pplay_tune_PLAY_TUNE *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline void pplay_tune_target_component_SET(int8_t src, pplay_tune_PLAY_TUNE *dst) { //Component ID
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}

static inline bool pplay_tune_tune_GET(pplay_tune_PLAY_TUNE *const src, Vplay_tune_tune *ret) {//tune in board specific format
	
	if (src->base.field_bit != 18 && !set_field(src, 18, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vplay_tune_tune pplay_tune_tune_SET(const char src[], size_t len, pplay_tune_PLAY_TUNE *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 18, len);
	
	Vplay_tune_tune ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int16_t pdigicam_configure_shutter_speed_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//Divisor number e.g. 1000 means 1/1000 (0 means ignore)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pdigicam_configure_shutter_speed_SET(int16_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //Divisor number e.g. 1000 means 1/1000 (0 means ignore)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int8_t pdigicam_configure_target_system_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//System ID
	
	return (int8_t) src[2];
}


static inline void pdigicam_configure_target_system_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //System ID
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t pdigicam_configure_target_component_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//Component ID
	
	return (int8_t) src[3];
}


static inline void pdigicam_configure_target_component_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //Component ID
	
	
	dst[3] = (uint8_t) (src);
}


static inline int8_t pdigicam_configure_mode_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//Mode enumeration from 1 to N P, TV, AV, M, Etc (0 means ignore)
	
	return (int8_t) src[4];
}


static inline void pdigicam_configure_mode_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //Mode enumeration from 1 to N P, TV, AV, M, Etc (0 means ignore)
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t pdigicam_configure_aperture_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//F stop number x 10 e.g. 28 means 2.8 (0 means ignore)
	
	return (int8_t) src[5];
}


static inline void pdigicam_configure_aperture_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //F stop number x 10 e.g. 28 means 2.8 (0 means ignore)
	
	
	dst[5] = (uint8_t) (src);
}


static inline int8_t pdigicam_configure_iso_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//ISO enumeration from 1 to N e.g. 80, 100, 200, Etc (0 means ignore)
	
	return (int8_t) src[6];
}


static inline void pdigicam_configure_iso_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //ISO enumeration from 1 to N e.g. 80, 100, 200, Etc (0 means ignore)
	
	
	dst[6] = (uint8_t) (src);
}


static inline int8_t pdigicam_configure_exposure_type_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//Exposure type enumeration from 1 to N (0 means ignore)
	
	return (int8_t) src[7];
}


static inline void pdigicam_configure_exposure_type_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //Exposure type enumeration from 1 to N (0 means ignore)
	
	
	dst[7] = (uint8_t) (src);
}

/**
*Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
*					 just onc */

static inline int8_t pdigicam_configure_command_id_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {
	
	return (int8_t) src[8];
}

/**
*Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
*					 just onc */

static inline void pdigicam_configure_command_id_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) {
	
	
	dst[8] = (uint8_t) (src);
}


static inline int8_t pdigicam_configure_engine_cut_off_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
	
	return (int8_t) src[9];
}


static inline void pdigicam_configure_engine_cut_off_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
	
	
	dst[9] = (uint8_t) (src);
}


static inline int8_t pdigicam_configure_extra_param_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//Extra parameters enumeration (0 means ignore)
	
	return (int8_t) src[10];
}


static inline void pdigicam_configure_extra_param_SET(int8_t src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //Extra parameters enumeration (0 means ignore)
	
	
	dst[10] = (uint8_t) (src);
}


static inline float pdigicam_configure_extra_value_GET(pdigicam_configure_DIGICAM_CONFIGURE *src) {//Correspondent value to given extra_param
	
	return (intBitsToFloat(get_bytes(src, 11, 4)));
}


static inline void pdigicam_configure_extra_value_SET(float src, pdigicam_configure_DIGICAM_CONFIGURE *dst) { //Correspondent value to given extra_param
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 11);
}


static inline int32_t pscaled_pressure3_time_boot_ms_GET(pscaled_pressure3_SCALED_PRESSURE3 *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pscaled_pressure3_time_boot_ms_SET(int32_t src, pscaled_pressure3_SCALED_PRESSURE3 *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline float pscaled_pressure3_press_abs_GET(pscaled_pressure3_SCALED_PRESSURE3 *src) {//Absolute pressure (hectopascal)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pscaled_pressure3_press_abs_SET(float src, pscaled_pressure3_SCALED_PRESSURE3 *dst) { //Absolute pressure (hectopascal)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pscaled_pressure3_press_diff_GET(pscaled_pressure3_SCALED_PRESSURE3 *src) {//Differential pressure 1 (hectopascal)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pscaled_pressure3_press_diff_SET(float src, pscaled_pressure3_SCALED_PRESSURE3 *dst) { //Differential pressure 1 (hectopascal)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline int16_t pscaled_pressure3_temperature_GET(pscaled_pressure3_SCALED_PRESSURE3 *src) {//Temperature measurement (0.01 degrees celsius)
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline void pscaled_pressure3_temperature_SET(int16_t src, pscaled_pressure3_SCALED_PRESSURE3 *dst) { //Temperature measurement (0.01 degrees celsius)
	
	
	set_bytes((uint16_t) (src), 2, dst, 12);
}


static inline int8_t pmission_request_partial_list_target_system_GET(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pmission_request_partial_list_target_component_GET(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline int16_t pmission_request_partial_list_start_index_GET(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *src) {//Start index, 0 by default
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline int16_t pmission_request_partial_list_end_index_GET(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *src) {//End index, -1 by default (-1: send list to end). Else a valid index of the list
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}

static inline bool pmission_request_partial_list_mission_type_GET(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 48 && !set_field(src, 48, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*					 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*					 ID is stored as strin */

static inline bool pparam_ext_ack_param_id_GET(pparam_ext_ack_PARAM_EXT_ACK *const src, Vparam_ext_ack_param_id *ret) {
	
	if (src->base.field_bit != 3 && !set_field(src, 3, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vparam_ext_ack_param_id pparam_ext_ack_param_id_SET(const char src[], size_t len, pparam_ext_ack_PARAM_EXT_ACK *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 3, len);
	
	Vparam_ext_ack_param_id ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pparam_ext_ack_param_value_GET(pparam_ext_ack_PARAM_EXT_ACK *const src, Vparam_ext_ack_param_value *ret) {//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
	
	if (src->base.field_bit != 4 && !set_field(src, 4, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vparam_ext_ack_param_value pparam_ext_ack_param_value_SET(const char src[], size_t len, pparam_ext_ack_PARAM_EXT_ACK *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 4, len);
	
	Vparam_ext_ack_param_value ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}

static inline bool pparam_ext_ack_param_type_GET(pparam_ext_ack_PARAM_EXT_ACK *const src, e_MAV_PARAM_EXT_TYPE *ret) {
	if (src->base.field_bit != 5 && !set_field(src, 5, -1)) return false;
	*ret = (int8_t) (1 + get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline void pparam_ext_ack_param_type_SET(e_MAV_PARAM_EXT_TYPE src, pparam_ext_ack_PARAM_EXT_ACK *const dst) {
	
	if (dst->base.field_bit != 5) set_field(dst, 5, 0);
	
	
	set_bits(-1
	src, 4, dst->base.bytes, dst->BIT);
}

static inline bool pparam_ext_ack_param_result_GET(pparam_ext_ack_PARAM_EXT_ACK *const src, e_PARAM_ACK *ret) {
	if (src->base.field_bit != 6 && !set_field(src, 6, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 2));
	return true;
}

static inline void pparam_ext_ack_param_result_SET(e_PARAM_ACK src, pparam_ext_ack_PARAM_EXT_ACK *const dst) {
	
	if (dst->base.field_bit != 6) set_field(dst, 6, 0);
	
	
	set_bits(src, 2, dst->base.bytes, dst->BIT);
}


static inline int32_t puavcan_node_info_uptime_sec_GET(puavcan_node_info_UAVCAN_NODE_INFO *src) {//The number of seconds since the start-up of the node.
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void puavcan_node_info_uptime_sec_SET(int32_t src, puavcan_node_info_UAVCAN_NODE_INFO *dst) { //The number of seconds since the start-up of the node.
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline int32_t puavcan_node_info_sw_vcs_commit_GET(puavcan_node_info_UAVCAN_NODE_INFO *src) {//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
	
	return ((int32_t) (get_bytes(src->base.bytes, 4, 4)));
}


static inline void puavcan_node_info_sw_vcs_commit_SET(int32_t src, puavcan_node_info_UAVCAN_NODE_INFO *dst) { //Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 4);
}


static inline int64_t puavcan_node_info_time_usec_GET(puavcan_node_info_UAVCAN_NODE_INFO *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src->base.bytes, 8, 8)));
}


static inline void puavcan_node_info_time_usec_SET(int64_t src, puavcan_node_info_UAVCAN_NODE_INFO *dst) { //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	
	set_bytes((src), 8, dst->base.bytes, 8);
}


static inline int8_t puavcan_node_info_hw_version_major_GET(puavcan_node_info_UAVCAN_NODE_INFO *src) {//Hardware major version number.
	
	return (int8_t) src->base.bytes[16];
}


static inline void puavcan_node_info_hw_version_major_SET(int8_t src, puavcan_node_info_UAVCAN_NODE_INFO *dst) { //Hardware major version number.
	
	
	dst->base.bytes[16] = (uint8_t) (src);
}


static inline int8_t puavcan_node_info_hw_version_minor_GET(puavcan_node_info_UAVCAN_NODE_INFO *src) {//Hardware minor version number.
	
	return (int8_t) src->base.bytes[17];
}


static inline void puavcan_node_info_hw_version_minor_SET(int8_t src, puavcan_node_info_UAVCAN_NODE_INFO *dst) { //Hardware minor version number.
	
	
	dst->base.bytes[17] = (uint8_t) (src);
}


static inline int8_t vuavcan_node_info_hw_unique_id_GET(Vuavcan_node_info_hw_unique_id const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vuavcan_node_info_hw_unique_id puavcan_node_info_hw_unique_id_GET(puavcan_node_info_UAVCAN_NODE_INFO *src) { //Hardware unique 128-bit ID.
	
	return (Vuavcan_node_info_hw_unique_id) {src->base.bytes + 18, 16};
}


static inline void vuavcan_node_info_hw_unique_id_SET(int8_t src, size_t index, Vuavcan_node_info_hw_unique_id *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vuavcan_node_info_hw_unique_id puavcan_node_info_hw_unique_id_SET(const int8_t src[], puavcan_node_info_UAVCAN_NODE_INFO *dst) {//Hardware unique 128-bit ID.
	
	
	Vuavcan_node_info_hw_unique_id ret = {dst->base.bytes + 18, 16};
	
	if (src)
		for (size_t i = 0; i < 16; i++)
			vuavcan_node_info_hw_unique_id_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t puavcan_node_info_sw_version_major_GET(puavcan_node_info_UAVCAN_NODE_INFO *src) {//Software major version number.
	
	return (int8_t) src->base.bytes[34];
}


static inline void puavcan_node_info_sw_version_major_SET(int8_t src, puavcan_node_info_UAVCAN_NODE_INFO *dst) { //Software major version number.
	
	
	dst->base.bytes[34] = (uint8_t) (src);
}


static inline int8_t puavcan_node_info_sw_version_minor_GET(puavcan_node_info_UAVCAN_NODE_INFO *src) {//Software minor version number.
	
	return (int8_t) src->base.bytes[35];
}


static inline void puavcan_node_info_sw_version_minor_SET(int8_t src, puavcan_node_info_UAVCAN_NODE_INFO *dst) { //Software minor version number.
	
	
	dst->base.bytes[35] = (uint8_t) (src);
}

static inline bool puavcan_node_info_name_GET(puavcan_node_info_UAVCAN_NODE_INFO *const src, Vuavcan_node_info_name *ret) {//Node name string. For example, "sapog.px4.io".
	
	if (src->base.field_bit != 290 && !set_field(src, 290, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vuavcan_node_info_name puavcan_node_info_name_SET(const char src[], size_t len, puavcan_node_info_UAVCAN_NODE_INFO *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 290, len);
	
	Vuavcan_node_info_name ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int8_t pdata16_typE_GET(pdata16_DATA16 *src) {//data type
	
	return (int8_t) src[0];
}


static inline void pdata16_typE_SET(int8_t src, pdata16_DATA16 *dst) { //data type
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pdata16_len_GET(pdata16_DATA16 *src) {//data length
	
	return (int8_t) src[1];
}


static inline void pdata16_len_SET(int8_t src, pdata16_DATA16 *dst) { //data length
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t vdata16_daTa_GET(Vdata16_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vdata16_daTa pdata16_daTa_GET(pdata16_DATA16 *src) { //raw data
	
	return (Vdata16_daTa) {src + 2, 16};
}


static inline void vdata16_daTa_SET(int8_t src, size_t index, Vdata16_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vdata16_daTa pdata16_daTa_SET(const int8_t src[], pdata16_DATA16 *dst) {//raw data
	
	
	Vdata16_daTa ret = {dst + 2, 16};
	
	if (src)
		for (size_t i = 0; i < 16; i++)
			vdata16_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t pset_mag_offsets_target_system_GET(pset_mag_offsets_SET_MAG_OFFSETS *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pset_mag_offsets_target_system_SET(int8_t src, pset_mag_offsets_SET_MAG_OFFSETS *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pset_mag_offsets_target_component_GET(pset_mag_offsets_SET_MAG_OFFSETS *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pset_mag_offsets_target_component_SET(int8_t src, pset_mag_offsets_SET_MAG_OFFSETS *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int16_t pset_mag_offsets_mag_ofs_x_GET(pset_mag_offsets_SET_MAG_OFFSETS *src) {//magnetometer X offset
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pset_mag_offsets_mag_ofs_x_SET(int16_t src, pset_mag_offsets_SET_MAG_OFFSETS *dst) { //magnetometer X offset
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t pset_mag_offsets_mag_ofs_y_GET(pset_mag_offsets_SET_MAG_OFFSETS *src) {//magnetometer Y offset
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void pset_mag_offsets_mag_ofs_y_SET(int16_t src, pset_mag_offsets_SET_MAG_OFFSETS *dst) { //magnetometer Y offset
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int16_t pset_mag_offsets_mag_ofs_z_GET(pset_mag_offsets_SET_MAG_OFFSETS *src) {//magnetometer Z offset
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline void pset_mag_offsets_mag_ofs_z_SET(int16_t src, pset_mag_offsets_SET_MAG_OFFSETS *dst) { //magnetometer Z offset
	
	
	set_bytes((uint16_t) (src), 2, dst, 6);
}


static inline int16_t pap_adc_adc1_GET(pap_adc_AP_ADC *src) {//ADC output 1
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pap_adc_adc1_SET(int16_t src, pap_adc_AP_ADC *dst) { //ADC output 1
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t pap_adc_adc2_GET(pap_adc_AP_ADC *src) {//ADC output 2
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pap_adc_adc2_SET(int16_t src, pap_adc_AP_ADC *dst) { //ADC output 2
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t pap_adc_adc3_GET(pap_adc_AP_ADC *src) {//ADC output 3
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void pap_adc_adc3_SET(int16_t src, pap_adc_AP_ADC *dst) { //ADC output 3
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int16_t pap_adc_adc4_GET(pap_adc_AP_ADC *src) {//ADC output 4
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline void pap_adc_adc4_SET(int16_t src, pap_adc_AP_ADC *dst) { //ADC output 4
	
	
	set_bytes((uint16_t) (src), 2, dst, 6);
}


static inline int16_t pap_adc_adc5_GET(pap_adc_AP_ADC *src) {//ADC output 5
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline void pap_adc_adc5_SET(int16_t src, pap_adc_AP_ADC *dst) { //ADC output 5
	
	
	set_bytes((uint16_t) (src), 2, dst, 8);
}


static inline int16_t pap_adc_adc6_GET(pap_adc_AP_ADC *src) {//ADC output 6
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline void pap_adc_adc6_SET(int16_t src, pap_adc_AP_ADC *dst) { //ADC output 6
	
	
	set_bytes((uint16_t) (src), 2, dst, 10);
}


static inline float pwind_direction_GET(pwind_WIND *src) {//wind direction that wind is coming from (degrees)
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void pwind_direction_SET(float src, pwind_WIND *dst) { //wind direction that wind is coming from (degrees)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float pwind_speed_GET(pwind_WIND *src) {//wind speed in ground plane (m/s)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pwind_speed_SET(float src, pwind_WIND *dst) { //wind speed in ground plane (m/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pwind_speed_z_GET(pwind_WIND *src) {//vertical wind speed (m/s)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pwind_speed_z_SET(float src, pwind_WIND *dst) { //vertical wind speed (m/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline int8_t pautopilot_version_request_target_system_GET(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pautopilot_version_request_target_system_SET(int8_t src, pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pautopilot_version_request_target_component_GET(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pautopilot_version_request_target_component_SET(int8_t src, pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int32_t plocal_position_ned_time_boot_ms_GET(plocal_position_ned_LOCAL_POSITION_NED *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline float plocal_position_ned_x_GET(plocal_position_ned_LOCAL_POSITION_NED *src) {//X Position
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline float plocal_position_ned_y_GET(plocal_position_ned_LOCAL_POSITION_NED *src) {//Y Position
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline float plocal_position_ned_z_GET(plocal_position_ned_LOCAL_POSITION_NED *src) {//Z Position
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float plocal_position_ned_vx_GET(plocal_position_ned_LOCAL_POSITION_NED *src) {//X Speed
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline float plocal_position_ned_vy_GET(plocal_position_ned_LOCAL_POSITION_NED *src) {//Y Speed
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline float plocal_position_ned_vz_GET(plocal_position_ned_LOCAL_POSITION_NED *src) {//Z Speed
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline int16_t pdata_transmission_handshake_width_GET(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *src) {//Width of a matrix or image
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pdata_transmission_handshake_width_SET(int16_t src, pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *dst) { //Width of a matrix or image
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t pdata_transmission_handshake_height_GET(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *src) {//Height of a matrix or image
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pdata_transmission_handshake_height_SET(int16_t src, pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *dst) { //Height of a matrix or image
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int16_t pdata_transmission_handshake_packets_GET(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *src) {//number of packets beeing sent (set on ACK only)
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void pdata_transmission_handshake_packets_SET(int16_t src, pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *dst) { //number of packets beeing sent (set on ACK only)
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int32_t pdata_transmission_handshake_size_GET(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *src) {//total data size in bytes (set on ACK only)
	
	return ((int32_t) (get_bytes(src, 6, 4)));
}


static inline void pdata_transmission_handshake_size_SET(int32_t src, pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *dst) { //total data size in bytes (set on ACK only)
	
	
	set_bytes((uint32_t) (src), 4, dst, 6);
}


static inline int8_t pdata_transmission_handshake_typE_GET(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *src) {//type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h
	
	return (int8_t) src[10];
}


static inline void pdata_transmission_handshake_typE_SET(int8_t src, pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *dst) { //type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h
	
	
	dst[10] = (uint8_t) (src);
}

/**
*payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
*					 ACK only */

static inline int8_t pdata_transmission_handshake_payload_GET(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *src) {
	
	return (int8_t) src[11];
}

/**
*payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
*					 ACK only */

static inline void pdata_transmission_handshake_payload_SET(int8_t src, pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *dst) {
	
	
	dst[11] = (uint8_t) (src);
}


static inline int8_t pdata_transmission_handshake_jpg_quality_GET(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *src) {//JPEG quality out of [1,100]
	
	return (int8_t) src[12];
}


static inline void pdata_transmission_handshake_jpg_quality_SET(int8_t src, pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *dst) { //JPEG quality out of [1,100]
	
	
	dst[12] = (uint8_t) (src);
}


static inline int32_t pgps_global_origin_latitude_GET(pgps_global_origin_GPS_GLOBAL_ORIGIN *src) {//Latitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline int32_t pgps_global_origin_longitude_GET(pgps_global_origin_GPS_GLOBAL_ORIGIN *src) {//Longitude (WGS84), in degrees * 1E7
	
	return ((int32_t) (get_bytes(src->base.bytes, 4, 4)));
}


static inline int32_t pgps_global_origin_altitude_GET(pgps_global_origin_GPS_GLOBAL_ORIGIN *src) {//Altitude (AMSL), in meters * 1000 (positive for up)
	
	return ((int32_t) (get_bytes(src->base.bytes, 8, 4)));
}

static inline bool pgps_global_origin_time_usec_GET(pgps_global_origin_GPS_GLOBAL_ORIGIN *const src, int64_t *ret) {
	if (src->base.field_bit != 96 && !set_field(src, 96, -1)) return false;
	*ret = ((int64_t) (get_bytes(src->base.bytes, src->BYTE, 8)));
	return true;
}


static inline int32_t pscaled_imu2_time_boot_ms_GET(pscaled_imu2_SCALED_IMU2 *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pscaled_imu2_time_boot_ms_SET(int32_t src, pscaled_imu2_SCALED_IMU2 *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int16_t pscaled_imu2_xacc_GET(pscaled_imu2_SCALED_IMU2 *src) {//X acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 4, 2)));
}


static inline void pscaled_imu2_xacc_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //X acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 4);
}


static inline int16_t pscaled_imu2_yacc_GET(pscaled_imu2_SCALED_IMU2 *src) {//Y acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 6, 2)));
}


static inline void pscaled_imu2_yacc_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //Y acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 6);
}


static inline int16_t pscaled_imu2_zacc_GET(pscaled_imu2_SCALED_IMU2 *src) {//Z acceleration (mg)
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline void pscaled_imu2_zacc_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //Z acceleration (mg)
	
	
	set_bytes((uint16_t) (src), 2, dst, 8);
}


static inline int16_t pscaled_imu2_xgyro_GET(pscaled_imu2_SCALED_IMU2 *src) {//Angular speed around X axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline void pscaled_imu2_xgyro_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //Angular speed around X axis (millirad /sec)
	
	
	set_bytes((uint16_t) (src), 2, dst, 10);
}


static inline int16_t pscaled_imu2_ygyro_GET(pscaled_imu2_SCALED_IMU2 *src) {//Angular speed around Y axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline void pscaled_imu2_ygyro_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //Angular speed around Y axis (millirad /sec)
	
	
	set_bytes((uint16_t) (src), 2, dst, 12);
}


static inline int16_t pscaled_imu2_zgyro_GET(pscaled_imu2_SCALED_IMU2 *src) {//Angular speed around Z axis (millirad /sec)
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline void pscaled_imu2_zgyro_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //Angular speed around Z axis (millirad /sec)
	
	
	set_bytes((uint16_t) (src), 2, dst, 14);
}


static inline int16_t pscaled_imu2_xmag_GET(pscaled_imu2_SCALED_IMU2 *src) {//X Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 16, 2)));
}


static inline void pscaled_imu2_xmag_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //X Magnetic field (milli tesla)
	
	
	set_bytes((uint16_t) (src), 2, dst, 16);
}


static inline int16_t pscaled_imu2_ymag_GET(pscaled_imu2_SCALED_IMU2 *src) {//Y Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 18, 2)));
}


static inline void pscaled_imu2_ymag_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //Y Magnetic field (milli tesla)
	
	
	set_bytes((uint16_t) (src), 2, dst, 18);
}


static inline int16_t pscaled_imu2_zmag_GET(pscaled_imu2_SCALED_IMU2 *src) {//Z Magnetic field (milli tesla)
	
	return ((int16_t) (get_bytes(src, 20, 2)));
}


static inline void pscaled_imu2_zmag_SET(int16_t src, pscaled_imu2_SCALED_IMU2 *dst) { //Z Magnetic field (milli tesla)
	
	
	set_bytes((uint16_t) (src), 2, dst, 20);
}


static inline int32_t pattitude_quaternion_time_boot_ms_GET(pattitude_quaternion_ATTITUDE_QUATERNION *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline float pattitude_quaternion_q1_GET(pattitude_quaternion_ATTITUDE_QUATERNION *src) {//Quaternion component 1, w (1 in null-rotation)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline float pattitude_quaternion_q2_GET(pattitude_quaternion_ATTITUDE_QUATERNION *src) {//Quaternion component 2, x (0 in null-rotation)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline float pattitude_quaternion_q3_GET(pattitude_quaternion_ATTITUDE_QUATERNION *src) {//Quaternion component 3, y (0 in null-rotation)
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float pattitude_quaternion_q4_GET(pattitude_quaternion_ATTITUDE_QUATERNION *src) {//Quaternion component 4, z (0 in null-rotation)
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline float pattitude_quaternion_rollspeed_GET(pattitude_quaternion_ATTITUDE_QUATERNION *src) {//Roll angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline float pattitude_quaternion_pitchspeed_GET(pattitude_quaternion_ATTITUDE_QUATERNION *src) {//Pitch angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline float pattitude_quaternion_yawspeed_GET(pattitude_quaternion_ATTITUDE_QUATERNION *src) {//Yaw angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline int8_t pdata64_typE_GET(pdata64_DATA64 *src) {//data type
	
	return (int8_t) src[0];
}


static inline void pdata64_typE_SET(int8_t src, pdata64_DATA64 *dst) { //data type
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pdata64_len_GET(pdata64_DATA64 *src) {//data length
	
	return (int8_t) src[1];
}


static inline void pdata64_len_SET(int8_t src, pdata64_DATA64 *dst) { //data length
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t vdata64_daTa_GET(Vdata64_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vdata64_daTa pdata64_daTa_GET(pdata64_DATA64 *src) { //raw data
	
	return (Vdata64_daTa) {src + 2, 64};
}


static inline void vdata64_daTa_SET(int8_t src, size_t index, Vdata64_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vdata64_daTa pdata64_daTa_SET(const int8_t src[], pdata64_DATA64 *dst) {//raw data
	
	
	Vdata64_daTa ret = {dst + 2, 64};
	
	if (src)
		for (size_t i = 0; i < 64; i++)
			vdata64_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int64_t phil_actuator_controls_time_usec_GET(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src->base.bytes, 0, 8)));
}


static inline int64_t phil_actuator_controls_flags_GET(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *src) {//Flags as bitfield, reserved for future use.
	
	return ((int64_t) (get_bytes(src->base.bytes, 8, 8)));
}


static inline float vhil_actuator_controls_controls_GET(Vhil_actuator_controls_controls const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vhil_actuator_controls_controls phil_actuator_controls_controls_GET(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *src) { //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
	
	return (Vhil_actuator_controls_controls) {src->base.bytes + 16, 16};
}

static inline bool phil_actuator_controls_mode_GET(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *const src, e_MAV_MODE *ret) {
	if (src->base.field_bit != 640 && !set_field(src, 640, -1)) return false;
	*ret = _en_mav_mode(get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*					 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*					 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*					 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*					 bit 11: yaw, bit 12: yaw rat */

static inline int16_t pposition_target_local_ned_type_mask_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int32_t pposition_target_local_ned_time_boot_ms_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//Timestamp in milliseconds since system boot
	
	return ((int32_t) (get_bytes(src->base.bytes, 2, 4)));
}


static inline float pposition_target_local_ned_x_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//X Position in NED frame in meters
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 6, 4)));
}


static inline float pposition_target_local_ned_y_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//Y Position in NED frame in meters
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 10, 4)));
}


static inline float pposition_target_local_ned_z_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//Z Position in NED frame in meters (note, altitude is negative in NED)
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 14, 4)));
}


static inline float pposition_target_local_ned_vx_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//X velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 18, 4)));
}


static inline float pposition_target_local_ned_vy_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//Y velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 22, 4)));
}


static inline float pposition_target_local_ned_vz_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//Z velocity in NED frame in meter / s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 26, 4)));
}


static inline float pposition_target_local_ned_afx_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 30, 4)));
}


static inline float pposition_target_local_ned_afy_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 34, 4)));
}


static inline float pposition_target_local_ned_afz_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 38, 4)));
}


static inline float pposition_target_local_ned_yaw_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//yaw setpoint in rad
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 42, 4)));
}


static inline float pposition_target_local_ned_yaw_rate_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *src) {//yaw rate setpoint in rad/s
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 46, 4)));
}

static inline bool pposition_target_local_ned_coordinate_frame_GET(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 400 && !set_field(src, 400, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}


static inline int8_t pgimbal_report_target_system_GET(pgimbal_report_GIMBAL_REPORT *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pgimbal_report_target_system_SET(int8_t src, pgimbal_report_GIMBAL_REPORT *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pgimbal_report_target_component_GET(pgimbal_report_GIMBAL_REPORT *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pgimbal_report_target_component_SET(int8_t src, pgimbal_report_GIMBAL_REPORT *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline float pgimbal_report_delta_time_GET(pgimbal_report_GIMBAL_REPORT *src) {//Time since last update (seconds)
	
	return (intBitsToFloat(get_bytes(src, 2, 4)));
}


static inline void pgimbal_report_delta_time_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Time since last update (seconds)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 2);
}


static inline float pgimbal_report_delta_angle_x_GET(pgimbal_report_GIMBAL_REPORT *src) {//Delta angle X (radians)
	
	return (intBitsToFloat(get_bytes(src, 6, 4)));
}


static inline void pgimbal_report_delta_angle_x_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Delta angle X (radians)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 6);
}


static inline float pgimbal_report_delta_angle_y_GET(pgimbal_report_GIMBAL_REPORT *src) {//Delta angle Y (radians)
	
	return (intBitsToFloat(get_bytes(src, 10, 4)));
}


static inline void pgimbal_report_delta_angle_y_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Delta angle Y (radians)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 10);
}


static inline float pgimbal_report_delta_angle_z_GET(pgimbal_report_GIMBAL_REPORT *src) {//Delta angle X (radians)
	
	return (intBitsToFloat(get_bytes(src, 14, 4)));
}


static inline void pgimbal_report_delta_angle_z_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Delta angle X (radians)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 14);
}


static inline float pgimbal_report_delta_velocity_x_GET(pgimbal_report_GIMBAL_REPORT *src) {//Delta velocity X (m/s)
	
	return (intBitsToFloat(get_bytes(src, 18, 4)));
}


static inline void pgimbal_report_delta_velocity_x_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Delta velocity X (m/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 18);
}


static inline float pgimbal_report_delta_velocity_y_GET(pgimbal_report_GIMBAL_REPORT *src) {//Delta velocity Y (m/s)
	
	return (intBitsToFloat(get_bytes(src, 22, 4)));
}


static inline void pgimbal_report_delta_velocity_y_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Delta velocity Y (m/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 22);
}


static inline float pgimbal_report_delta_velocity_z_GET(pgimbal_report_GIMBAL_REPORT *src) {//Delta velocity Z (m/s)
	
	return (intBitsToFloat(get_bytes(src, 26, 4)));
}


static inline void pgimbal_report_delta_velocity_z_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Delta velocity Z (m/s)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 26);
}


static inline float pgimbal_report_joint_roll_GET(pgimbal_report_GIMBAL_REPORT *src) {//Joint ROLL (radians)
	
	return (intBitsToFloat(get_bytes(src, 30, 4)));
}


static inline void pgimbal_report_joint_roll_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Joint ROLL (radians)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 30);
}


static inline float pgimbal_report_joint_el_GET(pgimbal_report_GIMBAL_REPORT *src) {//Joint EL (radians)
	
	return (intBitsToFloat(get_bytes(src, 34, 4)));
}


static inline void pgimbal_report_joint_el_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Joint EL (radians)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 34);
}


static inline float pgimbal_report_joint_az_GET(pgimbal_report_GIMBAL_REPORT *src) {//Joint AZ (radians)
	
	return (intBitsToFloat(get_bytes(src, 38, 4)));
}


static inline void pgimbal_report_joint_az_SET(float src, pgimbal_report_GIMBAL_REPORT *dst) { //Joint AZ (radians)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 38);
}


static inline int32_t pdevice_op_write_request_id_GET(pdevice_op_write_DEVICE_OP_WRITE *src) {//request ID - copied to reply
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void pdevice_op_write_request_id_SET(int32_t src, pdevice_op_write_DEVICE_OP_WRITE *dst) { //request ID - copied to reply
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline int8_t pdevice_op_write_target_system_GET(pdevice_op_write_DEVICE_OP_WRITE *src) {//System ID
	
	return (int8_t) src->base.bytes[4];
}


static inline void pdevice_op_write_target_system_SET(int8_t src, pdevice_op_write_DEVICE_OP_WRITE *dst) { //System ID
	
	
	dst->base.bytes[4] = (uint8_t) (src);
}


static inline int8_t pdevice_op_write_target_component_GET(pdevice_op_write_DEVICE_OP_WRITE *src) {//Component ID
	
	return (int8_t) src->base.bytes[5];
}


static inline void pdevice_op_write_target_component_SET(int8_t src, pdevice_op_write_DEVICE_OP_WRITE *dst) { //Component ID
	
	
	dst->base.bytes[5] = (uint8_t) (src);
}


static inline int8_t pdevice_op_write_bus_GET(pdevice_op_write_DEVICE_OP_WRITE *src) {//Bus number
	
	return (int8_t) src->base.bytes[6];
}


static inline void pdevice_op_write_bus_SET(int8_t src, pdevice_op_write_DEVICE_OP_WRITE *dst) { //Bus number
	
	
	dst->base.bytes[6] = (uint8_t) (src);
}


static inline int8_t pdevice_op_write_address_GET(pdevice_op_write_DEVICE_OP_WRITE *src) {//Bus address
	
	return (int8_t) src->base.bytes[7];
}


static inline void pdevice_op_write_address_SET(int8_t src, pdevice_op_write_DEVICE_OP_WRITE *dst) { //Bus address
	
	
	dst->base.bytes[7] = (uint8_t) (src);
}


static inline int8_t pdevice_op_write_regstart_GET(pdevice_op_write_DEVICE_OP_WRITE *src) {//First register to write
	
	return (int8_t) src->base.bytes[8];
}


static inline void pdevice_op_write_regstart_SET(int8_t src, pdevice_op_write_DEVICE_OP_WRITE *dst) { //First register to write
	
	
	dst->base.bytes[8] = (uint8_t) (src);
}


static inline int8_t pdevice_op_write_count_GET(pdevice_op_write_DEVICE_OP_WRITE *src) {//count of registers to write
	
	return (int8_t) src->base.bytes[9];
}


static inline void pdevice_op_write_count_SET(int8_t src, pdevice_op_write_DEVICE_OP_WRITE *dst) { //count of registers to write
	
	
	dst->base.bytes[9] = (uint8_t) (src);
}


static inline int8_t vdevice_op_write_daTa_GET(Vdevice_op_write_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vdevice_op_write_daTa pdevice_op_write_daTa_GET(pdevice_op_write_DEVICE_OP_WRITE *src) { //write data
	
	return (Vdevice_op_write_daTa) {src->base.bytes + 10, 128};
}


static inline void vdevice_op_write_daTa_SET(int8_t src, size_t index, Vdevice_op_write_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vdevice_op_write_daTa pdevice_op_write_daTa_SET(const int8_t src[], pdevice_op_write_DEVICE_OP_WRITE *dst) {//write data
	
	
	Vdevice_op_write_daTa ret = {dst->base.bytes + 10, 128};
	
	if (src)
		for (size_t i = 0; i < 128; i++)
			vdevice_op_write_daTa_SET(src[i], i, &ret);
	return ret;
}

static inline bool pdevice_op_write_bustype_GET(pdevice_op_write_DEVICE_OP_WRITE *const src, e_DEVICE_OP_BUSTYPE *ret) {
	if (src->base.field_bit != 1106 && !set_field(src, 1106, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void pdevice_op_write_bustype_SET(e_DEVICE_OP_BUSTYPE src, pdevice_op_write_DEVICE_OP_WRITE *const dst) {
	
	if (dst->base.field_bit != 1106) set_field(dst, 1106, 0);
	
	
	set_bits(src, 1, dst->base.bytes, dst->BIT);
}

static inline bool pdevice_op_write_busname_GET(pdevice_op_write_DEVICE_OP_WRITE *const src, Vdevice_op_write_busname *ret) {//Name of device on bus (for SPI)
	
	if (src->base.field_bit != 1107 && !set_field(src, 1107, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vdevice_op_write_busname pdevice_op_write_busname_SET(const char src[], size_t len, pdevice_op_write_DEVICE_OP_WRITE *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 1107, len);
	
	Vdevice_op_write_busname ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int16_t pdistance_sensor_min_distance_GET(pdistance_sensor_DISTANCE_SENSOR *src) {//Minimum distance the sensor can measure in centimeters
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void pdistance_sensor_min_distance_SET(int16_t src, pdistance_sensor_DISTANCE_SENSOR *dst) { //Minimum distance the sensor can measure in centimeters
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t pdistance_sensor_max_distance_GET(pdistance_sensor_DISTANCE_SENSOR *src) {//Maximum distance the sensor can measure in centimeters
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void pdistance_sensor_max_distance_SET(int16_t src, pdistance_sensor_DISTANCE_SENSOR *dst) { //Maximum distance the sensor can measure in centimeters
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int16_t pdistance_sensor_current_distance_GET(pdistance_sensor_DISTANCE_SENSOR *src) {//Current distance reading
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}


static inline void pdistance_sensor_current_distance_SET(int16_t src, pdistance_sensor_DISTANCE_SENSOR *dst) { //Current distance reading
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 4);
}


static inline int32_t pdistance_sensor_time_boot_ms_GET(pdistance_sensor_DISTANCE_SENSOR *src) {//Time since system boot
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline void pdistance_sensor_time_boot_ms_SET(int32_t src, pdistance_sensor_DISTANCE_SENSOR *dst) { //Time since system boot
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 6);
}


static inline int8_t pdistance_sensor_id_GET(pdistance_sensor_DISTANCE_SENSOR *src) {//Onboard ID of the sensor
	
	return (int8_t) src->base.bytes[10];
}


static inline void pdistance_sensor_id_SET(int8_t src, pdistance_sensor_DISTANCE_SENSOR *dst) { //Onboard ID of the sensor
	
	
	dst->base.bytes[10] = (uint8_t) (src);
}


static inline int8_t pdistance_sensor_covariance_GET(pdistance_sensor_DISTANCE_SENSOR *src) {//Measurement covariance in centimeters, 0 for unknown / invalid readings
	
	return (int8_t) src->base.bytes[11];
}


static inline void pdistance_sensor_covariance_SET(int8_t src, pdistance_sensor_DISTANCE_SENSOR *dst) { //Measurement covariance in centimeters, 0 for unknown / invalid readings
	
	
	dst->base.bytes[11] = (uint8_t) (src);
}

static inline bool pdistance_sensor_typE_GET(pdistance_sensor_DISTANCE_SENSOR *const src, e_MAV_DISTANCE_SENSOR *ret) {
	if (src->base.field_bit != 98 && !set_field(src, 98, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pdistance_sensor_typE_SET(e_MAV_DISTANCE_SENSOR src, pdistance_sensor_DISTANCE_SENSOR *const dst) {
	
	if (dst->base.field_bit != 98) set_field(dst, 98, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool pdistance_sensor_orientation_GET(pdistance_sensor_DISTANCE_SENSOR *const src, e_MAV_SENSOR_ORIENTATION *ret) {
	if (src->base.field_bit != 99 && !set_field(src, 99, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 6));
	return true;
}

static inline void pdistance_sensor_orientation_SET(e_MAV_SENSOR_ORIENTATION src, pdistance_sensor_DISTANCE_SENSOR *const dst) {
	
	if (dst->base.field_bit != 99) set_field(dst, 99, 0);
	
	
	set_bits(src, 6, dst->base.bytes, dst->BIT);
}

/**
*Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
*					 average flow. The integration time also indicates the */

static inline int32_t phil_optical_flow_integration_time_us_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}

/**
*Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
*					 average flow. The integration time also indicates the */

static inline void phil_optical_flow_integration_time_us_SET(int32_t src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) {
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline int32_t phil_optical_flow_time_delta_distance_us_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {//Time in microseconds since the distance was sampled.
	
	return ((int32_t) (get_bytes(src, 4, 4)));
}


static inline void phil_optical_flow_time_delta_distance_us_SET(int32_t src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) { //Time in microseconds since the distance was sampled.
	
	
	set_bytes((uint32_t) (src), 4, dst, 4);
}


static inline int64_t phil_optical_flow_time_usec_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 8, 8)));
}


static inline void phil_optical_flow_time_usec_SET(int64_t src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) { //Timestamp (microseconds, synced to UNIX time or since system boot)
	
	
	set_bytes((src), 8, dst, 8);
}


static inline int8_t phil_optical_flow_sensor_id_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {//Sensor ID
	
	return (int8_t) src[16];
}


static inline void phil_optical_flow_sensor_id_SET(int8_t src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) { //Sensor ID
	
	
	dst[16] = (uint8_t) (src);
}

/**
*Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
*					 motion along the positive Y axis induces a negative flow. */

static inline float phil_optical_flow_integrated_x_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {
	
	return (intBitsToFloat(get_bytes(src, 17, 4)));
}

/**
*Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
*					 motion along the positive Y axis induces a negative flow. */

static inline void phil_optical_flow_integrated_x_SET(float src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 17);
}

/**
*Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
*					 motion along the positive X axis induces a positive flow. */

static inline float phil_optical_flow_integrated_y_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {
	
	return (intBitsToFloat(get_bytes(src, 21, 4)));
}

/**
*Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
*					 motion along the positive X axis induces a positive flow. */

static inline void phil_optical_flow_integrated_y_SET(float src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 21);
}


static inline float phil_optical_flow_integrated_xgyro_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {//RH rotation around X axis (rad)
	
	return (intBitsToFloat(get_bytes(src, 25, 4)));
}


static inline void phil_optical_flow_integrated_xgyro_SET(float src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) { //RH rotation around X axis (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 25);
}


static inline float phil_optical_flow_integrated_ygyro_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {//RH rotation around Y axis (rad)
	
	return (intBitsToFloat(get_bytes(src, 29, 4)));
}


static inline void phil_optical_flow_integrated_ygyro_SET(float src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) { //RH rotation around Y axis (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 29);
}


static inline float phil_optical_flow_integrated_zgyro_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {//RH rotation around Z axis (rad)
	
	return (intBitsToFloat(get_bytes(src, 33, 4)));
}


static inline void phil_optical_flow_integrated_zgyro_SET(float src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) { //RH rotation around Z axis (rad)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 33);
}


static inline int16_t phil_optical_flow_temperature_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {//Temperature * 100 in centi-degrees Celsius
	
	return ((int16_t) (get_bytes(src, 37, 2)));
}


static inline void phil_optical_flow_temperature_SET(int16_t src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) { //Temperature * 100 in centi-degrees Celsius
	
	
	set_bytes((uint16_t) (src), 2, dst, 37);
}


static inline int8_t phil_optical_flow_quality_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	
	return (int8_t) src[39];
}


static inline void phil_optical_flow_quality_SET(int8_t src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) { //Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	
	
	dst[39] = (uint8_t) (src);
}

/**
*Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
*					 value: Unknown distance */

static inline float phil_optical_flow_distance_GET(phil_optical_flow_HIL_OPTICAL_FLOW *src) {
	
	return (intBitsToFloat(get_bytes(src, 40, 4)));
}

/**
*Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
*					 value: Unknown distance */

static inline void phil_optical_flow_distance_SET(float src, phil_optical_flow_HIL_OPTICAL_FLOW *dst) {
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 40);
}


static inline int32_t pscaled_pressure2_time_boot_ms_GET(pscaled_pressure2_SCALED_PRESSURE2 *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src, 0, 4)));
}


static inline void pscaled_pressure2_time_boot_ms_SET(int32_t src, pscaled_pressure2_SCALED_PRESSURE2 *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst, 0);
}


static inline float pscaled_pressure2_press_abs_GET(pscaled_pressure2_SCALED_PRESSURE2 *src) {//Absolute pressure (hectopascal)
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pscaled_pressure2_press_abs_SET(float src, pscaled_pressure2_SCALED_PRESSURE2 *dst) { //Absolute pressure (hectopascal)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pscaled_pressure2_press_diff_GET(pscaled_pressure2_SCALED_PRESSURE2 *src) {//Differential pressure 1 (hectopascal)
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pscaled_pressure2_press_diff_SET(float src, pscaled_pressure2_SCALED_PRESSURE2 *dst) { //Differential pressure 1 (hectopascal)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline int16_t pscaled_pressure2_temperature_GET(pscaled_pressure2_SCALED_PRESSURE2 *src) {//Temperature measurement (0.01 degrees celsius)
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline void pscaled_pressure2_temperature_SET(int16_t src, pscaled_pressure2_SCALED_PRESSURE2 *dst) { //Temperature measurement (0.01 degrees celsius)
	
	
	set_bytes((uint16_t) (src), 2, dst, 12);
}


static inline int64_t pwind_cov_time_usec_GET(pwind_cov_WIND_COV *src) {//Timestamp (micros since boot or Unix epoch)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void pwind_cov_time_usec_SET(int64_t src, pwind_cov_WIND_COV *dst) { //Timestamp (micros since boot or Unix epoch)
	
	
	set_bytes((src), 8, dst, 0);
}


static inline float pwind_cov_wind_x_GET(pwind_cov_WIND_COV *src) {//Wind in X (NED) direction in m/s
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pwind_cov_wind_x_SET(float src, pwind_cov_WIND_COV *dst) { //Wind in X (NED) direction in m/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline float pwind_cov_wind_y_GET(pwind_cov_WIND_COV *src) {//Wind in Y (NED) direction in m/s
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline void pwind_cov_wind_y_SET(float src, pwind_cov_WIND_COV *dst) { //Wind in Y (NED) direction in m/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 12);
}


static inline float pwind_cov_wind_z_GET(pwind_cov_WIND_COV *src) {//Wind in Z (NED) direction in m/s
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline void pwind_cov_wind_z_SET(float src, pwind_cov_WIND_COV *dst) { //Wind in Z (NED) direction in m/s
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 16);
}


static inline float pwind_cov_var_horiz_GET(pwind_cov_WIND_COV *src) {//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline void pwind_cov_var_horiz_SET(float src, pwind_cov_WIND_COV *dst) { //Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 20);
}


static inline float pwind_cov_var_vert_GET(pwind_cov_WIND_COV *src) {//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline void pwind_cov_var_vert_SET(float src, pwind_cov_WIND_COV *dst) { //Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 24);
}


static inline float pwind_cov_wind_alt_GET(pwind_cov_WIND_COV *src) {//AMSL altitude (m) this measurement was taken at
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline void pwind_cov_wind_alt_SET(float src, pwind_cov_WIND_COV *dst) { //AMSL altitude (m) this measurement was taken at
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 28);
}


static inline float pwind_cov_horiz_accuracy_GET(pwind_cov_WIND_COV *src) {//Horizontal speed 1-STD accuracy
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline void pwind_cov_horiz_accuracy_SET(float src, pwind_cov_WIND_COV *dst) { //Horizontal speed 1-STD accuracy
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 32);
}


static inline float pwind_cov_vert_accuracy_GET(pwind_cov_WIND_COV *src) {//Vertical speed 1-STD accuracy
	
	return (intBitsToFloat(get_bytes(src, 36, 4)));
}


static inline void pwind_cov_vert_accuracy_SET(float src, pwind_cov_WIND_COV *dst) { //Vertical speed 1-STD accuracy
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 36);
}


static inline int8_t pchange_operator_control_target_system_GET(pchange_operator_control_CHANGE_OPERATOR_CONTROL *src) {//System the GCS requests control for
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pchange_operator_control_control_request_GET(pchange_operator_control_CHANGE_OPERATOR_CONTROL *src) {//0: request control of this MAV, 1: Release control of this MAV
	
	return (int8_t) src->base.bytes[1];
}

/**
*0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
*					 the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
*					 message indicating an encryption mismatch */

static inline int8_t pchange_operator_control_version_GET(pchange_operator_control_CHANGE_OPERATOR_CONTROL *src) {
	
	return (int8_t) src->base.bytes[2];
}

/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*					 characters may involve A-Z, a-z, 0-9, and "!?,.- */

static inline bool pchange_operator_control_passkey_GET(pchange_operator_control_CHANGE_OPERATOR_CONTROL *const src, Vchange_operator_control_passkey *ret) {
	
	if (src->base.field_bit != 26 && !set_field(src, 26, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}


static inline int8_t pgopro_set_request_target_system_GET(pgopro_set_request_GOPRO_SET_REQUEST *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline void pgopro_set_request_target_system_SET(int8_t src, pgopro_set_request_GOPRO_SET_REQUEST *dst) { //System ID
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t pgopro_set_request_target_component_GET(pgopro_set_request_GOPRO_SET_REQUEST *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline void pgopro_set_request_target_component_SET(int8_t src, pgopro_set_request_GOPRO_SET_REQUEST *dst) { //Component ID
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}


static inline int8_t vgopro_set_request_value_GET(Vgopro_set_request_value const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgopro_set_request_value pgopro_set_request_value_GET(pgopro_set_request_GOPRO_SET_REQUEST *src) { //Value
	
	return (Vgopro_set_request_value) {src->base.bytes + 2, 4};
}


static inline void vgopro_set_request_value_SET(int8_t src, size_t index, Vgopro_set_request_value *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vgopro_set_request_value pgopro_set_request_value_SET(const int8_t src[], pgopro_set_request_GOPRO_SET_REQUEST *dst) {//Value
	
	
	Vgopro_set_request_value ret = {dst->base.bytes + 2, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vgopro_set_request_value_SET(src[i], i, &ret);
	return ret;
}

static inline bool pgopro_set_request_cmd_id_GET(pgopro_set_request_GOPRO_SET_REQUEST *const src, e_GOPRO_COMMAND *ret) {
	if (src->base.field_bit != 48 && !set_field(src, 48, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline void pgopro_set_request_cmd_id_SET(e_GOPRO_COMMAND src, pgopro_set_request_GOPRO_SET_REQUEST *const dst) {
	
	if (dst->base.field_bit != 48) set_field(dst, 48, 0);
	
	
	set_bits(src, 5, dst->base.bytes, dst->BIT);
}


static inline int16_t psys_status_load_GET(psys_status_SYS_STATUS *src) {//Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int16_t psys_status_voltage_battery_GET(psys_status_SYS_STATUS *src) {//Battery voltage, in millivolts (1 = 1 millivolt)
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}

/**
*Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
*					 (packets that were corrupted on reception on the MAV */

static inline int16_t psys_status_drop_rate_comm_GET(psys_status_SYS_STATUS *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}

/**
*Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
*					 on reception on the MAV */

static inline int16_t psys_status_errors_comm_GET(psys_status_SYS_STATUS *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 6, 2)));
}


static inline int16_t psys_status_errors_count1_GET(psys_status_SYS_STATUS *src) {//Autopilot-specific errors
	
	return ((int16_t) (get_bytes(src->base.bytes, 8, 2)));
}


static inline int16_t psys_status_errors_count2_GET(psys_status_SYS_STATUS *src) {//Autopilot-specific errors
	
	return ((int16_t) (get_bytes(src->base.bytes, 10, 2)));
}


static inline int16_t psys_status_errors_count3_GET(psys_status_SYS_STATUS *src) {//Autopilot-specific errors
	
	return ((int16_t) (get_bytes(src->base.bytes, 12, 2)));
}


static inline int16_t psys_status_errors_count4_GET(psys_status_SYS_STATUS *src) {//Autopilot-specific errors
	
	return ((int16_t) (get_bytes(src->base.bytes, 14, 2)));
}


static inline int16_t psys_status_current_battery_GET(psys_status_SYS_STATUS *src) {//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
	
	return ((int16_t) (get_bytes(src->base.bytes, 16, 2)));
}


static inline int8_t psys_status_battery_remaining_GET(psys_status_SYS_STATUS *src) {//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
	
	return (int8_t) src->base.bytes[18];
}

static inline bool psys_status_onboard_control_sensors_present_GET(psys_status_SYS_STATUS *const src, e_MAV_SYS_STATUS_SENSOR *ret) {
	if (src->base.field_bit != 154 && !set_field(src, 154, -1)) return false;
	*ret = _en_mav_sys_status_sensor(get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline bool psys_status_onboard_control_sensors_enabled_GET(psys_status_SYS_STATUS *const src, e_MAV_SYS_STATUS_SENSOR *ret) {
	if (src->base.field_bit != 155 && !set_field(src, 155, -1)) return false;
	*ret = _en_mav_sys_status_sensor(get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline bool psys_status_onboard_control_sensors_health_GET(psys_status_SYS_STATUS *const src, e_MAV_SYS_STATUS_SENSOR *ret) {
	if (src->base.field_bit != 156 && !set_field(src, 156, -1)) return false;
	*ret = _en_mav_sys_status_sensor(get_bits(src->base.bytes, src->BIT, 5));
	return true;
}


static inline int16_t pmission_item_seq_GET(pmission_item_MISSION_ITEM *src) {//Sequence
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int8_t pmission_item_target_system_GET(pmission_item_MISSION_ITEM *src) {//System ID
	
	return (int8_t) src->base.bytes[2];
}


static inline int8_t pmission_item_target_component_GET(pmission_item_MISSION_ITEM *src) {//Component ID
	
	return (int8_t) src->base.bytes[3];
}


static inline int8_t pmission_item_current_GET(pmission_item_MISSION_ITEM *src) {//false:0, true:1
	
	return (int8_t) src->base.bytes[4];
}


static inline int8_t pmission_item_autocontinue_GET(pmission_item_MISSION_ITEM *src) {//autocontinue to next wp
	
	return (int8_t) src->base.bytes[5];
}


static inline float pmission_item_param1_GET(pmission_item_MISSION_ITEM *src) {//PARAM1, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 6, 4)));
}


static inline float pmission_item_param2_GET(pmission_item_MISSION_ITEM *src) {//PARAM2, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 10, 4)));
}


static inline float pmission_item_param3_GET(pmission_item_MISSION_ITEM *src) {//PARAM3, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 14, 4)));
}


static inline float pmission_item_param4_GET(pmission_item_MISSION_ITEM *src) {//PARAM4, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 18, 4)));
}


static inline float pmission_item_x_GET(pmission_item_MISSION_ITEM *src) {//PARAM5 / local: x position, global: latitude
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 22, 4)));
}


static inline float pmission_item_y_GET(pmission_item_MISSION_ITEM *src) {//PARAM6 / y position: global: longitude
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 26, 4)));
}


static inline float pmission_item_z_GET(pmission_item_MISSION_ITEM *src) {//PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 30, 4)));
}

static inline bool pmission_item_frame_GET(pmission_item_MISSION_ITEM *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 274 && !set_field(src, 274, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline bool pmission_item_command_GET(pmission_item_MISSION_ITEM *const src, e_MAV_CMD *ret) {
	if (src->base.field_bit != 275 && !set_field(src, 275, -1)) return false;
	*ret = _en_mav_cmd(get_bits(src->base.bytes, src->BIT, 8));
	return true;
}

static inline bool pmission_item_mission_type_GET(pmission_item_MISSION_ITEM *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 276 && !set_field(src, 276, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline int64_t praw_imu_time_usec_GET(praw_imu_RAW_IMU *src) {//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline int16_t praw_imu_xacc_GET(praw_imu_RAW_IMU *src) {//X acceleration (raw)
	
	return ((int16_t) (get_bytes(src, 8, 2)));
}


static inline int16_t praw_imu_yacc_GET(praw_imu_RAW_IMU *src) {//Y acceleration (raw)
	
	return ((int16_t) (get_bytes(src, 10, 2)));
}


static inline int16_t praw_imu_zacc_GET(praw_imu_RAW_IMU *src) {//Z acceleration (raw)
	
	return ((int16_t) (get_bytes(src, 12, 2)));
}


static inline int16_t praw_imu_xgyro_GET(praw_imu_RAW_IMU *src) {//Angular speed around X axis (raw)
	
	return ((int16_t) (get_bytes(src, 14, 2)));
}


static inline int16_t praw_imu_ygyro_GET(praw_imu_RAW_IMU *src) {//Angular speed around Y axis (raw)
	
	return ((int16_t) (get_bytes(src, 16, 2)));
}


static inline int16_t praw_imu_zgyro_GET(praw_imu_RAW_IMU *src) {//Angular speed around Z axis (raw)
	
	return ((int16_t) (get_bytes(src, 18, 2)));
}


static inline int16_t praw_imu_xmag_GET(praw_imu_RAW_IMU *src) {//X Magnetic field (raw)
	
	return ((int16_t) (get_bytes(src, 20, 2)));
}


static inline int16_t praw_imu_ymag_GET(praw_imu_RAW_IMU *src) {//Y Magnetic field (raw)
	
	return ((int16_t) (get_bytes(src, 22, 2)));
}


static inline int16_t praw_imu_zmag_GET(praw_imu_RAW_IMU *src) {//Z Magnetic field (raw)
	
	return ((int16_t) (get_bytes(src, 24, 2)));
}


static inline int8_t pcommand_int_target_system_GET(pcommand_int_COMMAND_INT *src) {//System ID
	
	return (int8_t) src->base.bytes[0];
}


static inline int8_t pcommand_int_target_component_GET(pcommand_int_COMMAND_INT *src) {//Component ID
	
	return (int8_t) src->base.bytes[1];
}


static inline int8_t pcommand_int_current_GET(pcommand_int_COMMAND_INT *src) {//false:0, true:1
	
	return (int8_t) src->base.bytes[2];
}


static inline int8_t pcommand_int_autocontinue_GET(pcommand_int_COMMAND_INT *src) {//autocontinue to next wp
	
	return (int8_t) src->base.bytes[3];
}


static inline float pcommand_int_param1_GET(pcommand_int_COMMAND_INT *src) {//PARAM1, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 4, 4)));
}


static inline float pcommand_int_param2_GET(pcommand_int_COMMAND_INT *src) {//PARAM2, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 8, 4)));
}


static inline float pcommand_int_param3_GET(pcommand_int_COMMAND_INT *src) {//PARAM3, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 12, 4)));
}


static inline float pcommand_int_param4_GET(pcommand_int_COMMAND_INT *src) {//PARAM4, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 16, 4)));
}


static inline int32_t pcommand_int_x_GET(pcommand_int_COMMAND_INT *src) {//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	
	return ((int32_t) (get_bytes(src->base.bytes, 20, 4)));
}


static inline int32_t pcommand_int_y_GET(pcommand_int_COMMAND_INT *src) {//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	
	return ((int32_t) (get_bytes(src->base.bytes, 24, 4)));
}


static inline float pcommand_int_z_GET(pcommand_int_COMMAND_INT *src) {//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 28, 4)));
}

static inline bool pcommand_int_frame_GET(pcommand_int_COMMAND_INT *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 258 && !set_field(src, 258, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline bool pcommand_int_command_GET(pcommand_int_COMMAND_INT *const src, e_MAV_CMD *ret) {
	if (src->base.field_bit != 259 && !set_field(src, 259, -1)) return false;
	*ret = _en_mav_cmd(get_bits(src->base.bytes, src->BIT, 8));
	return true;
}


static inline int64_t poptical_flow_time_usec_GET(poptical_flow_OPTICAL_FLOW *src) {//Timestamp (UNIX)
	
	return ((int64_t) (get_bytes(src->base.bytes, 0, 8)));
}


static inline int8_t poptical_flow_sensor_id_GET(poptical_flow_OPTICAL_FLOW *src) {//Sensor ID
	
	return (int8_t) src->base.bytes[8];
}


static inline int16_t poptical_flow_flow_x_GET(poptical_flow_OPTICAL_FLOW *src) {//Flow in pixels * 10 in x-sensor direction (dezi-pixels)
	
	return ((int16_t) (get_bytes(src->base.bytes, 9, 2)));
}


static inline int16_t poptical_flow_flow_y_GET(poptical_flow_OPTICAL_FLOW *src) {//Flow in pixels * 10 in y-sensor direction (dezi-pixels)
	
	return ((int16_t) (get_bytes(src->base.bytes, 11, 2)));
}


static inline float poptical_flow_flow_comp_m_x_GET(poptical_flow_OPTICAL_FLOW *src) {//Flow in meters in x-sensor direction, angular-speed compensated
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 13, 4)));
}


static inline float poptical_flow_flow_comp_m_y_GET(poptical_flow_OPTICAL_FLOW *src) {//Flow in meters in y-sensor direction, angular-speed compensated
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 17, 4)));
}


static inline int8_t poptical_flow_quality_GET(poptical_flow_OPTICAL_FLOW *src) {//Optical flow quality / confidence. 0: bad, 255: maximum quality
	
	return (int8_t) src->base.bytes[21];
}


static inline float poptical_flow_ground_distance_GET(poptical_flow_OPTICAL_FLOW *src) {//Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 22, 4)));
}

static inline bool poptical_flow_flow_rate_x_GET(poptical_flow_OPTICAL_FLOW *const src, float *ret) {
	if (src->base.field_bit != 208 && !set_field(src, 208, -1)) return false;
	*ret = (intBitsToFloat(get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

static inline bool poptical_flow_flow_rate_y_GET(poptical_flow_OPTICAL_FLOW *const src, float *ret) {
	if (src->base.field_bit != 209 && !set_field(src, 209, -1)) return false;
	*ret = (intBitsToFloat(get_bytes(src->base.bytes, src->BYTE, 4)));
	return true;
}

/**
*Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
*					 sequence (0,1,2,3,4) */

static inline int16_t pmission_item_int_seq_GET(pmission_item_int_MISSION_ITEM_INT *src) {
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline int8_t pmission_item_int_target_system_GET(pmission_item_int_MISSION_ITEM_INT *src) {//System ID
	
	return (int8_t) src->base.bytes[2];
}


static inline int8_t pmission_item_int_target_component_GET(pmission_item_int_MISSION_ITEM_INT *src) {//Component ID
	
	return (int8_t) src->base.bytes[3];
}


static inline int8_t pmission_item_int_current_GET(pmission_item_int_MISSION_ITEM_INT *src) {//false:0, true:1
	
	return (int8_t) src->base.bytes[4];
}


static inline int8_t pmission_item_int_autocontinue_GET(pmission_item_int_MISSION_ITEM_INT *src) {//autocontinue to next wp
	
	return (int8_t) src->base.bytes[5];
}


static inline float pmission_item_int_param1_GET(pmission_item_int_MISSION_ITEM_INT *src) {//PARAM1, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 6, 4)));
}


static inline float pmission_item_int_param2_GET(pmission_item_int_MISSION_ITEM_INT *src) {//PARAM2, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 10, 4)));
}


static inline float pmission_item_int_param3_GET(pmission_item_int_MISSION_ITEM_INT *src) {//PARAM3, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 14, 4)));
}


static inline float pmission_item_int_param4_GET(pmission_item_int_MISSION_ITEM_INT *src) {//PARAM4, see MAV_CMD enum
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 18, 4)));
}


static inline int32_t pmission_item_int_x_GET(pmission_item_int_MISSION_ITEM_INT *src) {//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	
	return ((int32_t) (get_bytes(src->base.bytes, 22, 4)));
}


static inline int32_t pmission_item_int_y_GET(pmission_item_int_MISSION_ITEM_INT *src) {//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
	
	return ((int32_t) (get_bytes(src->base.bytes, 26, 4)));
}


static inline float pmission_item_int_z_GET(pmission_item_int_MISSION_ITEM_INT *src) {//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 30, 4)));
}

static inline bool pmission_item_int_frame_GET(pmission_item_int_MISSION_ITEM_INT *const src, e_MAV_FRAME *ret) {
	if (src->base.field_bit != 274 && !set_field(src, 274, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 4));
	return true;
}

static inline bool pmission_item_int_command_GET(pmission_item_int_MISSION_ITEM_INT *const src, e_MAV_CMD *ret) {
	if (src->base.field_bit != 275 && !set_field(src, 275, -1)) return false;
	*ret = _en_mav_cmd(get_bits(src->base.bytes, src->BIT, 8));
	return true;
}

static inline bool pmission_item_int_mission_type_GET(pmission_item_int_MISSION_ITEM_INT *const src, e_MAV_MISSION_TYPE *ret) {
	if (src->base.field_bit != 276 && !set_field(src, 276, -1)) return false;
	*ret = _en_mav_mission_type(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}


static inline int64_t pvision_position_delta_time_usec_GET(pvision_position_delta_VISION_POSITION_DELTA *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline void pvision_position_delta_time_usec_SET(int64_t src, pvision_position_delta_VISION_POSITION_DELTA *dst) { //Timestamp (microseconds, synced to UNIX time or since system boot)
	
	
	set_bytes((src), 8, dst, 0);
}


static inline int64_t pvision_position_delta_time_delta_usec_GET(pvision_position_delta_VISION_POSITION_DELTA *src) {//Time in microseconds since the last reported camera frame
	
	return ((int64_t) (get_bytes(src, 8, 8)));
}


static inline void pvision_position_delta_time_delta_usec_SET(int64_t src, pvision_position_delta_VISION_POSITION_DELTA *dst) { //Time in microseconds since the last reported camera frame
	
	
	set_bytes((src), 8, dst, 8);
}


static inline float vvision_position_delta_angle_delta_GET(Vvision_position_delta_angle_delta const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vvision_position_delta_angle_delta pvision_position_delta_angle_delta_GET(pvision_position_delta_VISION_POSITION_DELTA *src) { //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
	
	return (Vvision_position_delta_angle_delta) {src + 16, 3};
}


static inline void vvision_position_delta_angle_delta_SET(float src, size_t index, Vvision_position_delta_angle_delta *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

static inline Vvision_position_delta_angle_delta pvision_position_delta_angle_delta_SET(const float src[], pvision_position_delta_VISION_POSITION_DELTA *dst) {//Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
	
	
	Vvision_position_delta_angle_delta ret = {dst + 16, 3};
	
	if (src)
		for (size_t i = 0; i < 3; i++)
			vvision_position_delta_angle_delta_SET(src[i], i, &ret);
	return ret;
}

/**
*Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
*					 2=down */

static inline float vvision_position_delta_position_delta_GET(Vvision_position_delta_position_delta const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

/**
*Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
*					 2=down */

static inline Vvision_position_delta_position_delta pvision_position_delta_position_delta_GET(pvision_position_delta_VISION_POSITION_DELTA *src) {
	
	return (Vvision_position_delta_position_delta) {src + 28, 3};
}

/**
*Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
*					 2=down */

static inline void vvision_position_delta_position_delta_SET(float src, size_t index, Vvision_position_delta_position_delta *dst) { set_bytes(*(uint32_t *) &(src), 4, dst->bytes, index * 4); }

/**
*Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
*					 2=down */

static inline Vvision_position_delta_position_delta pvision_position_delta_position_delta_SET(const float src[], pvision_position_delta_VISION_POSITION_DELTA *dst) {
	
	
	Vvision_position_delta_position_delta ret = {dst + 28, 3};
	
	if (src)
		for (size_t i = 0; i < 3; i++)
			vvision_position_delta_position_delta_SET(src[i], i, &ret);
	return ret;
}


static inline float pvision_position_delta_confidence_GET(pvision_position_delta_VISION_POSITION_DELTA *src) {//normalised confidence value from 0 to 100
	
	return (intBitsToFloat(get_bytes(src, 40, 4)));
}


static inline void pvision_position_delta_confidence_SET(float src, pvision_position_delta_VISION_POSITION_DELTA *dst) { //normalised confidence value from 0 to 100
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 40);
}


static inline int16_t plogging_data_sequence_GET(plogging_data_LOGGING_DATA *src) {//sequence number (can wrap)
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void plogging_data_sequence_SET(int16_t src, plogging_data_LOGGING_DATA *dst) { //sequence number (can wrap)
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int8_t plogging_data_target_system_GET(plogging_data_LOGGING_DATA *src) {//system ID of the target
	
	return (int8_t) src[2];
}


static inline void plogging_data_target_system_SET(int8_t src, plogging_data_LOGGING_DATA *dst) { //system ID of the target
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t plogging_data_target_component_GET(plogging_data_LOGGING_DATA *src) {//component ID of the target
	
	return (int8_t) src[3];
}


static inline void plogging_data_target_component_SET(int8_t src, plogging_data_LOGGING_DATA *dst) { //component ID of the target
	
	
	dst[3] = (uint8_t) (src);
}


static inline int8_t plogging_data_length_GET(plogging_data_LOGGING_DATA *src) {//data length
	
	return (int8_t) src[4];
}


static inline void plogging_data_length_SET(int8_t src, plogging_data_LOGGING_DATA *dst) { //data length
	
	
	dst[4] = (uint8_t) (src);
}

/**
*offset into data where first message starts. This can be used for recovery, when a previous message got
*					 lost (set to 255 if no start exists) */

static inline int8_t plogging_data_first_message_offset_GET(plogging_data_LOGGING_DATA *src) {
	
	return (int8_t) src[5];
}

/**
*offset into data where first message starts. This can be used for recovery, when a previous message got
*					 lost (set to 255 if no start exists) */

static inline void plogging_data_first_message_offset_SET(int8_t src, plogging_data_LOGGING_DATA *dst) {
	
	
	dst[5] = (uint8_t) (src);
}


static inline int8_t vlogging_data_daTa_GET(Vlogging_data_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vlogging_data_daTa plogging_data_daTa_GET(plogging_data_LOGGING_DATA *src) { //logged data
	
	return (Vlogging_data_daTa) {src + 6, 249};
}


static inline void vlogging_data_daTa_SET(int8_t src, size_t index, Vlogging_data_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vlogging_data_daTa plogging_data_daTa_SET(const int8_t src[], plogging_data_LOGGING_DATA *dst) {//logged data
	
	
	Vlogging_data_daTa ret = {dst + 6, 249};
	
	if (src)
		for (size_t i = 0; i < 249; i++)
			vlogging_data_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int32_t pdevice_op_read_request_id_GET(pdevice_op_read_DEVICE_OP_READ *src) {//request ID - copied to reply
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void pdevice_op_read_request_id_SET(int32_t src, pdevice_op_read_DEVICE_OP_READ *dst) { //request ID - copied to reply
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline int8_t pdevice_op_read_target_system_GET(pdevice_op_read_DEVICE_OP_READ *src) {//System ID
	
	return (int8_t) src->base.bytes[4];
}


static inline void pdevice_op_read_target_system_SET(int8_t src, pdevice_op_read_DEVICE_OP_READ *dst) { //System ID
	
	
	dst->base.bytes[4] = (uint8_t) (src);
}


static inline int8_t pdevice_op_read_target_component_GET(pdevice_op_read_DEVICE_OP_READ *src) {//Component ID
	
	return (int8_t) src->base.bytes[5];
}


static inline void pdevice_op_read_target_component_SET(int8_t src, pdevice_op_read_DEVICE_OP_READ *dst) { //Component ID
	
	
	dst->base.bytes[5] = (uint8_t) (src);
}


static inline int8_t pdevice_op_read_bus_GET(pdevice_op_read_DEVICE_OP_READ *src) {//Bus number
	
	return (int8_t) src->base.bytes[6];
}


static inline void pdevice_op_read_bus_SET(int8_t src, pdevice_op_read_DEVICE_OP_READ *dst) { //Bus number
	
	
	dst->base.bytes[6] = (uint8_t) (src);
}


static inline int8_t pdevice_op_read_address_GET(pdevice_op_read_DEVICE_OP_READ *src) {//Bus address
	
	return (int8_t) src->base.bytes[7];
}


static inline void pdevice_op_read_address_SET(int8_t src, pdevice_op_read_DEVICE_OP_READ *dst) { //Bus address
	
	
	dst->base.bytes[7] = (uint8_t) (src);
}


static inline int8_t pdevice_op_read_regstart_GET(pdevice_op_read_DEVICE_OP_READ *src) {//First register to read
	
	return (int8_t) src->base.bytes[8];
}


static inline void pdevice_op_read_regstart_SET(int8_t src, pdevice_op_read_DEVICE_OP_READ *dst) { //First register to read
	
	
	dst->base.bytes[8] = (uint8_t) (src);
}


static inline int8_t pdevice_op_read_count_GET(pdevice_op_read_DEVICE_OP_READ *src) {//count of registers to read
	
	return (int8_t) src->base.bytes[9];
}


static inline void pdevice_op_read_count_SET(int8_t src, pdevice_op_read_DEVICE_OP_READ *dst) { //count of registers to read
	
	
	dst->base.bytes[9] = (uint8_t) (src);
}

static inline bool pdevice_op_read_bustype_GET(pdevice_op_read_DEVICE_OP_READ *const src, e_DEVICE_OP_BUSTYPE *ret) {
	if (src->base.field_bit != 82 && !set_field(src, 82, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void pdevice_op_read_bustype_SET(e_DEVICE_OP_BUSTYPE src, pdevice_op_read_DEVICE_OP_READ *const dst) {
	
	if (dst->base.field_bit != 82) set_field(dst, 82, 0);
	
	
	set_bits(src, 1, dst->base.bytes, dst->BIT);
}

static inline bool pdevice_op_read_busname_GET(pdevice_op_read_DEVICE_OP_READ *const src, Vdevice_op_read_busname *ret) {//Name of device on bus (for SPI)
	
	if (src->base.field_bit != 83 && !set_field(src, 83, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vdevice_op_read_busname pdevice_op_read_busname_SET(const char src[], size_t len, pdevice_op_read_DEVICE_OP_READ *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 83, len);
	
	Vdevice_op_read_busname ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline int8_t pmag_cal_progress_compass_id_GET(pmag_cal_progress_MAG_CAL_PROGRESS *src) {//Compass being calibrated
	
	return (int8_t) src->base.bytes[0];
}


static inline void pmag_cal_progress_compass_id_SET(int8_t src, pmag_cal_progress_MAG_CAL_PROGRESS *dst) { //Compass being calibrated
	
	
	dst->base.bytes[0] = (uint8_t) (src);
}


static inline int8_t pmag_cal_progress_cal_mask_GET(pmag_cal_progress_MAG_CAL_PROGRESS *src) {//Bitmask of compasses being calibrated
	
	return (int8_t) src->base.bytes[1];
}


static inline void pmag_cal_progress_cal_mask_SET(int8_t src, pmag_cal_progress_MAG_CAL_PROGRESS *dst) { //Bitmask of compasses being calibrated
	
	
	dst->base.bytes[1] = (uint8_t) (src);
}


static inline int8_t pmag_cal_progress_attempt_GET(pmag_cal_progress_MAG_CAL_PROGRESS *src) {//Attempt number
	
	return (int8_t) src->base.bytes[2];
}


static inline void pmag_cal_progress_attempt_SET(int8_t src, pmag_cal_progress_MAG_CAL_PROGRESS *dst) { //Attempt number
	
	
	dst->base.bytes[2] = (uint8_t) (src);
}


static inline int8_t pmag_cal_progress_completion_pct_GET(pmag_cal_progress_MAG_CAL_PROGRESS *src) {//Completion percentage
	
	return (int8_t) src->base.bytes[3];
}


static inline void pmag_cal_progress_completion_pct_SET(int8_t src, pmag_cal_progress_MAG_CAL_PROGRESS *dst) { //Completion percentage
	
	
	dst->base.bytes[3] = (uint8_t) (src);
}


static inline int8_t vmag_cal_progress_completion_mask_GET(Vmag_cal_progress_completion_mask const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vmag_cal_progress_completion_mask pmag_cal_progress_completion_mask_GET(pmag_cal_progress_MAG_CAL_PROGRESS *src) { //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
	
	return (Vmag_cal_progress_completion_mask) {src->base.bytes + 4, 10};
}


static inline void vmag_cal_progress_completion_mask_SET(int8_t src, size_t index, Vmag_cal_progress_completion_mask *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vmag_cal_progress_completion_mask pmag_cal_progress_completion_mask_SET(const int8_t src[], pmag_cal_progress_MAG_CAL_PROGRESS *dst) {//Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
	
	
	Vmag_cal_progress_completion_mask ret = {dst->base.bytes + 4, 10};
	
	if (src)
		for (size_t i = 0; i < 10; i++)
			vmag_cal_progress_completion_mask_SET(src[i], i, &ret);
	return ret;
}


static inline float pmag_cal_progress_direction_x_GET(pmag_cal_progress_MAG_CAL_PROGRESS *src) {//Body frame direction vector for display
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 14, 4)));
}


static inline void pmag_cal_progress_direction_x_SET(float src, pmag_cal_progress_MAG_CAL_PROGRESS *dst) { //Body frame direction vector for display
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 14);
}


static inline float pmag_cal_progress_direction_y_GET(pmag_cal_progress_MAG_CAL_PROGRESS *src) {//Body frame direction vector for display
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 18, 4)));
}


static inline void pmag_cal_progress_direction_y_SET(float src, pmag_cal_progress_MAG_CAL_PROGRESS *dst) { //Body frame direction vector for display
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 18);
}


static inline float pmag_cal_progress_direction_z_GET(pmag_cal_progress_MAG_CAL_PROGRESS *src) {//Body frame direction vector for display
	
	return (intBitsToFloat(get_bytes(src->base.bytes, 22, 4)));
}


static inline void pmag_cal_progress_direction_z_SET(float src, pmag_cal_progress_MAG_CAL_PROGRESS *dst) { //Body frame direction vector for display
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst->base.bytes, 22);
}

static inline bool pmag_cal_progress_cal_status_GET(pmag_cal_progress_MAG_CAL_PROGRESS *const src, e_MAG_CAL_STATUS *ret) {
	if (src->base.field_bit != 208 && !set_field(src, 208, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pmag_cal_progress_cal_status_SET(e_MAG_CAL_STATUS src, pmag_cal_progress_MAG_CAL_PROGRESS *const dst) {
	
	if (dst->base.field_bit != 208) set_field(dst, 208, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}


static inline int16_t phighres_imu_fields_updated_GET(phighres_imu_HIGHRES_IMU *src) {//Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void phighres_imu_fields_updated_SET(int16_t src, phighres_imu_HIGHRES_IMU *dst) { //Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int64_t phighres_imu_time_usec_GET(phighres_imu_HIGHRES_IMU *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 2, 8)));
}


static inline void phighres_imu_time_usec_SET(int64_t src, phighres_imu_HIGHRES_IMU *dst) { //Timestamp (microseconds, synced to UNIX time or since system boot)
	
	
	set_bytes((src), 8, dst, 2);
}


static inline float phighres_imu_xacc_GET(phighres_imu_HIGHRES_IMU *src) {//X acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src, 10, 4)));
}


static inline void phighres_imu_xacc_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //X acceleration (m/s^2)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 10);
}


static inline float phighres_imu_yacc_GET(phighres_imu_HIGHRES_IMU *src) {//Y acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src, 14, 4)));
}


static inline void phighres_imu_yacc_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Y acceleration (m/s^2)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 14);
}


static inline float phighres_imu_zacc_GET(phighres_imu_HIGHRES_IMU *src) {//Z acceleration (m/s^2)
	
	return (intBitsToFloat(get_bytes(src, 18, 4)));
}


static inline void phighres_imu_zacc_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Z acceleration (m/s^2)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 18);
}


static inline float phighres_imu_xgyro_GET(phighres_imu_HIGHRES_IMU *src) {//Angular speed around X axis (rad / sec)
	
	return (intBitsToFloat(get_bytes(src, 22, 4)));
}


static inline void phighres_imu_xgyro_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Angular speed around X axis (rad / sec)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 22);
}


static inline float phighres_imu_ygyro_GET(phighres_imu_HIGHRES_IMU *src) {//Angular speed around Y axis (rad / sec)
	
	return (intBitsToFloat(get_bytes(src, 26, 4)));
}


static inline void phighres_imu_ygyro_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Angular speed around Y axis (rad / sec)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 26);
}


static inline float phighres_imu_zgyro_GET(phighres_imu_HIGHRES_IMU *src) {//Angular speed around Z axis (rad / sec)
	
	return (intBitsToFloat(get_bytes(src, 30, 4)));
}


static inline void phighres_imu_zgyro_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Angular speed around Z axis (rad / sec)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 30);
}


static inline float phighres_imu_xmag_GET(phighres_imu_HIGHRES_IMU *src) {//X Magnetic field (Gauss)
	
	return (intBitsToFloat(get_bytes(src, 34, 4)));
}


static inline void phighres_imu_xmag_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //X Magnetic field (Gauss)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 34);
}


static inline float phighres_imu_ymag_GET(phighres_imu_HIGHRES_IMU *src) {//Y Magnetic field (Gauss)
	
	return (intBitsToFloat(get_bytes(src, 38, 4)));
}


static inline void phighres_imu_ymag_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Y Magnetic field (Gauss)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 38);
}


static inline float phighres_imu_zmag_GET(phighres_imu_HIGHRES_IMU *src) {//Z Magnetic field (Gauss)
	
	return (intBitsToFloat(get_bytes(src, 42, 4)));
}


static inline void phighres_imu_zmag_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Z Magnetic field (Gauss)
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 42);
}


static inline float phighres_imu_abs_pressure_GET(phighres_imu_HIGHRES_IMU *src) {//Absolute pressure in millibar
	
	return (intBitsToFloat(get_bytes(src, 46, 4)));
}


static inline void phighres_imu_abs_pressure_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Absolute pressure in millibar
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 46);
}


static inline float phighres_imu_diff_pressure_GET(phighres_imu_HIGHRES_IMU *src) {//Differential pressure in millibar
	
	return (intBitsToFloat(get_bytes(src, 50, 4)));
}


static inline void phighres_imu_diff_pressure_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Differential pressure in millibar
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 50);
}


static inline float phighres_imu_pressure_alt_GET(phighres_imu_HIGHRES_IMU *src) {//Altitude calculated from pressure
	
	return (intBitsToFloat(get_bytes(src, 54, 4)));
}


static inline void phighres_imu_pressure_alt_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Altitude calculated from pressure
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 54);
}


static inline float phighres_imu_temperature_GET(phighres_imu_HIGHRES_IMU *src) {//Temperature in degrees celsius
	
	return (intBitsToFloat(get_bytes(src, 58, 4)));
}


static inline void phighres_imu_temperature_SET(float src, phighres_imu_HIGHRES_IMU *dst) { //Temperature in degrees celsius
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 58);
}

static inline bool pextended_sys_state_vtol_state_GET(pextended_sys_state_EXTENDED_SYS_STATE *const src, e_MAV_VTOL_STATE *ret) {
	if (src->base.field_bit != 0 && !set_field(src, 0, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pextended_sys_state_vtol_state_SET(e_MAV_VTOL_STATE src, pextended_sys_state_EXTENDED_SYS_STATE *const dst) {
	
	if (dst->base.field_bit != 0) set_field(dst, 0, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool pextended_sys_state_landed_state_GET(pextended_sys_state_EXTENDED_SYS_STATE *const src, e_MAV_LANDED_STATE *ret) {
	if (src->base.field_bit != 1 && !set_field(src, 1, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void pextended_sys_state_landed_state_SET(e_MAV_LANDED_STATE src, pextended_sys_state_EXTENDED_SYS_STATE *const dst) {
	
	if (dst->base.field_bit != 1) set_field(dst, 1, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}


static inline int16_t puavionix_adsb_out_dynamic_accuracyVert_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//Vertical accuracy in cm. If unknown set to UINT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 0, 2)));
}


static inline void puavionix_adsb_out_dynamic_accuracyVert_SET(int16_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //Vertical accuracy in cm. If unknown set to UINT16_MAX
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 0);
}


static inline int16_t puavionix_adsb_out_dynamic_accuracyVel_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 2, 2)));
}


static inline void puavionix_adsb_out_dynamic_accuracyVel_SET(int16_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 2);
}


static inline int16_t puavionix_adsb_out_dynamic_squawk_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//Mode A code (typically 1200 [0x04B0] for VFR)
	
	return ((int16_t) (get_bytes(src->base.bytes, 4, 2)));
}


static inline void puavionix_adsb_out_dynamic_squawk_SET(int16_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //Mode A code (typically 1200 [0x04B0] for VFR)
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 4);
}


static inline int32_t puavionix_adsb_out_dynamic_utcTime_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
	
	return ((int32_t) (get_bytes(src->base.bytes, 6, 4)));
}


static inline void puavionix_adsb_out_dynamic_utcTime_SET(int32_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 6);
}


static inline int32_t puavionix_adsb_out_dynamic_accuracyHor_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
	
	return ((int32_t) (get_bytes(src->base.bytes, 10, 4)));
}


static inline void puavionix_adsb_out_dynamic_accuracyHor_SET(int32_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 10);
}


static inline int32_t puavionix_adsb_out_dynamic_gpsLat_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
	
	return ((int32_t) (get_bytes(src->base.bytes, 14, 4)));
}


static inline void puavionix_adsb_out_dynamic_gpsLat_SET(int32_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 14);
}


static inline int32_t puavionix_adsb_out_dynamic_gpsLon_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
	
	return ((int32_t) (get_bytes(src->base.bytes, 18, 4)));
}


static inline void puavionix_adsb_out_dynamic_gpsLon_SET(int32_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 18);
}


static inline int32_t puavionix_adsb_out_dynamic_gpsAlt_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX
	
	return ((int32_t) (get_bytes(src->base.bytes, 22, 4)));
}


static inline void puavionix_adsb_out_dynamic_gpsAlt_SET(int32_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 22);
}


static inline int8_t puavionix_adsb_out_dynamic_numSats_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//Number of satellites visible. If unknown set to UINT8_MAX
	
	return (int8_t) src->base.bytes[26];
}


static inline void puavionix_adsb_out_dynamic_numSats_SET(int8_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //Number of satellites visible. If unknown set to UINT8_MAX
	
	
	dst->base.bytes[26] = (uint8_t) (src);
}

/**
*Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude
*					 (m * 1E-3). (up +ve). If unknown set to INT32_MA */

static inline int32_t puavionix_adsb_out_dynamic_baroAltMSL_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {
	
	return ((int32_t) (get_bytes(src->base.bytes, 27, 4)));
}

/**
*Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude
*					 (m * 1E-3). (up +ve). If unknown set to INT32_MA */

static inline void puavionix_adsb_out_dynamic_baroAltMSL_SET(int32_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) {
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 27);
}


static inline int16_t puavionix_adsb_out_dynamic_velVert_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//GPS vertical speed in cm/s. If unknown set to INT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 31, 2)));
}


static inline void puavionix_adsb_out_dynamic_velVert_SET(int16_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //GPS vertical speed in cm/s. If unknown set to INT16_MAX
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 31);
}


static inline int16_t puavionix_adsb_out_dynamic_velNS_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 33, 2)));
}


static inline void puavionix_adsb_out_dynamic_velNS_SET(int16_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 33);
}


static inline int16_t puavionix_adsb_out_dynamic_VelEW_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *src) {//East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
	
	return ((int16_t) (get_bytes(src->base.bytes, 35, 2)));
}


static inline void puavionix_adsb_out_dynamic_VelEW_SET(int16_t src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *dst) { //East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
	
	
	set_bytes((uint16_t) (src), 2, dst->base.bytes, 35);
}

static inline bool puavionix_adsb_out_dynamic_gpsFix_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const src, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX *ret) {
	if (src->base.field_bit != 298 && !set_field(src, 298, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void puavionix_adsb_out_dynamic_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const dst) {
	
	if (dst->base.field_bit != 298) set_field(dst, 298, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool puavionix_adsb_out_dynamic_emergencyStatus_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const src, e_UAVIONIX_ADSB_EMERGENCY_STATUS *ret) {
	if (src->base.field_bit != 299 && !set_field(src, 299, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void puavionix_adsb_out_dynamic_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const dst) {
	
	if (dst->base.field_bit != 299) set_field(dst, 299, 0);
	
	
	set_bits(src, 3, dst->base.bytes, dst->BIT);
}

static inline bool puavionix_adsb_out_dynamic_state_GET(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const src, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE *ret) {
	if (src->base.field_bit != 300 && !set_field(src, 300, -1)) return false;
	*ret = _en_uavionix_adsb_out_dynamic_state(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void puavionix_adsb_out_dynamic_state_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const dst) {
	
	if (dst->base.field_bit != 300) set_field(dst, 300, 0);
	
	
	UMAX id = _id_uavionix_adsb_out_dynamic_state(src);
	set_bits(id, 3, dst->base.bytes, dst->BIT);
}


static inline int8_t vgopro_get_response_value_GET(Vgopro_get_response_value const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgopro_get_response_value pgopro_get_response_value_GET(pgopro_get_response_GOPRO_GET_RESPONSE *src) { //Value
	
	return (Vgopro_get_response_value) {src->base.bytes + 0, 4};
}


static inline void vgopro_get_response_value_SET(int8_t src, size_t index, Vgopro_get_response_value *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vgopro_get_response_value pgopro_get_response_value_SET(const int8_t src[], pgopro_get_response_GOPRO_GET_RESPONSE *dst) {//Value
	
	
	Vgopro_get_response_value ret = {dst->base.bytes + 0, 4};
	
	if (src)
		for (size_t i = 0; i < 4; i++)
			vgopro_get_response_value_SET(src[i], i, &ret);
	return ret;
}

static inline bool pgopro_get_response_cmd_id_GET(pgopro_get_response_GOPRO_GET_RESPONSE *const src, e_GOPRO_COMMAND *ret) {
	if (src->base.field_bit != 32 && !set_field(src, 32, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 5));
	return true;
}

static inline void pgopro_get_response_cmd_id_SET(e_GOPRO_COMMAND src, pgopro_get_response_GOPRO_GET_RESPONSE *const dst) {
	
	if (dst->base.field_bit != 32) set_field(dst, 32, 0);
	
	
	set_bits(src, 5, dst->base.bytes, dst->BIT);
}

static inline bool pgopro_get_response_status_GET(pgopro_get_response_GOPRO_GET_RESPONSE *const src, e_GOPRO_REQUEST_STATUS *ret) {
	if (src->base.field_bit != 33 && !set_field(src, 33, -1)) return false;
	*ret = (int8_t) (get_bits(src->base.bytes, src->BIT, 1));
	return true;
}

static inline void pgopro_get_response_status_SET(e_GOPRO_REQUEST_STATUS src, pgopro_get_response_GOPRO_GET_RESPONSE *const dst) {
	
	if (dst->base.field_bit != 33) set_field(dst, 33, 0);
	
	
	set_bits(src, 1, dst->base.bytes, dst->BIT);
}


static inline int8_t pgps_inject_data_target_system_GET(pgps_inject_data_GPS_INJECT_DATA *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pgps_inject_data_target_system_SET(int8_t src, pgps_inject_data_GPS_INJECT_DATA *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pgps_inject_data_target_component_GET(pgps_inject_data_GPS_INJECT_DATA *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pgps_inject_data_target_component_SET(int8_t src, pgps_inject_data_GPS_INJECT_DATA *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t pgps_inject_data_len_GET(pgps_inject_data_GPS_INJECT_DATA *src) {//data length
	
	return (int8_t) src[2];
}


static inline void pgps_inject_data_len_SET(int8_t src, pgps_inject_data_GPS_INJECT_DATA *dst) { //data length
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t vgps_inject_data_daTa_GET(Vgps_inject_data_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgps_inject_data_daTa pgps_inject_data_daTa_GET(pgps_inject_data_GPS_INJECT_DATA *src) { //raw data (110 is enough for 12 satellites of RTCMv2)
	
	return (Vgps_inject_data_daTa) {src + 3, 110};
}


static inline void vgps_inject_data_daTa_SET(int8_t src, size_t index, Vgps_inject_data_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vgps_inject_data_daTa pgps_inject_data_daTa_SET(const int8_t src[], pgps_inject_data_GPS_INJECT_DATA *dst) {//raw data (110 is enough for 12 satellites of RTCMv2)
	
	
	Vgps_inject_data_daTa ret = {dst + 3, 110};
	
	if (src)
		for (size_t i = 0; i < 110; i++)
			vgps_inject_data_daTa_SET(src[i], i, &ret);
	return ret;
}

static inline bool puavionix_adsb_transceiver_health_report_rfHealth_GET(puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *const src, e_UAVIONIX_ADSB_RF_HEALTH *ret) {
	if (src->base.field_bit != 0 && !set_field(src, 0, -1)) return false;
	*ret = _en_uavionix_adsb_rf_health(get_bits(src->base.bytes, src->BIT, 3));
	return true;
}

static inline void puavionix_adsb_transceiver_health_report_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH src, puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *const dst) {
	
	if (dst->base.field_bit != 0) set_field(dst, 0, 0);
	
	
	UMAX id = _id_uavionix_adsb_rf_health(src);
	set_bits(id, 3, dst->base.bytes, dst->BIT);
}


static inline int64_t pattitude_quaternion_cov_time_usec_GET(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *src) {//Timestamp (microseconds since system boot or since UNIX epoch)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline float vattitude_quaternion_cov_q_GET(Vattitude_quaternion_cov_q const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vattitude_quaternion_cov_q pattitude_quaternion_cov_q_GET(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *src) { //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
	
	return (Vattitude_quaternion_cov_q) {src + 8, 4};
}


static inline float pattitude_quaternion_cov_rollspeed_GET(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *src) {//Roll angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline float pattitude_quaternion_cov_pitchspeed_GET(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *src) {//Pitch angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline float pattitude_quaternion_cov_yawspeed_GET(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *src) {//Yaw angular speed (rad/s)
	
	return (intBitsToFloat(get_bytes(src, 32, 4)));
}


static inline float vattitude_quaternion_cov_covariance_GET(Vattitude_quaternion_cov_covariance const *const src, size_t index) { return (intBitsToFloat(get_bytes(src->bytes, index * 4, 4))); }

static inline Vattitude_quaternion_cov_covariance pattitude_quaternion_cov_covariance_GET(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *src) { //Attitude covariance
	
	return (Vattitude_quaternion_cov_covariance) {src + 36, 9};
}


static inline int32_t pnamed_value_int_time_boot_ms_GET(pnamed_value_int_NAMED_VALUE_INT *src) {//Timestamp (milliseconds since system boot)
	
	return ((int32_t) (get_bytes(src->base.bytes, 0, 4)));
}


static inline void pnamed_value_int_time_boot_ms_SET(int32_t src, pnamed_value_int_NAMED_VALUE_INT *dst) { //Timestamp (milliseconds since system boot)
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 0);
}


static inline int32_t pnamed_value_int_value_GET(pnamed_value_int_NAMED_VALUE_INT *src) {//Signed integer value
	
	return ((int32_t) (get_bytes(src->base.bytes, 4, 4)));
}


static inline void pnamed_value_int_value_SET(int32_t src, pnamed_value_int_NAMED_VALUE_INT *dst) { //Signed integer value
	
	
	set_bytes((uint32_t) (src), 4, dst->base.bytes, 4);
}

static inline bool pnamed_value_int_name_GET(pnamed_value_int_NAMED_VALUE_INT *const src, Vnamed_value_int_name *ret) {//Name of the debug variable
	
	if (src->base.field_bit != 66 && !set_field(src, 66, -1)) return false;
	
	ret->bytes = src->base.bytes + src->BYTE;
	ret->len   = src->item_len;
	
	return true;
}

static inline Vnamed_value_int_name pnamed_value_int_name_SET(const char src[], size_t len, pnamed_value_int_NAMED_VALUE_INT *const dst) {
	
	len = 255 < len ? 255 : len;
	
	set_field(dst, 66, len);
	
	Vnamed_value_int_name ret = {dst->base.bytes + dst->BYTE, dst->item_len};
	
	if (src) memcpy(ret.bytes, src, len);
	return ret;
}


static inline float prpm_rpm1_GET(prpm_RPM *src) {//RPM Sensor1
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void prpm_rpm1_SET(float src, prpm_RPM *dst) { //RPM Sensor1
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float prpm_rpm2_GET(prpm_RPM *src) {//RPM Sensor2
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void prpm_rpm2_SET(float src, prpm_RPM *dst) { //RPM Sensor2
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}

/**
*LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
*					 the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
*					 on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
*					 while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
*					 fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
*					 with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
*					 corrupt RTCM data, and to recover from a unreliable transport delivery order */

static inline int8_t pgps_rtcm_data_flags_GET(pgps_rtcm_data_GPS_RTCM_DATA *src) {
	
	return (int8_t) src[0];
}

/**
*LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
*					 the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
*					 on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
*					 while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
*					 fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
*					 with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
*					 corrupt RTCM data, and to recover from a unreliable transport delivery order */

static inline void pgps_rtcm_data_flags_SET(int8_t src, pgps_rtcm_data_GPS_RTCM_DATA *dst) {
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pgps_rtcm_data_len_GET(pgps_rtcm_data_GPS_RTCM_DATA *src) {//data length
	
	return (int8_t) src[1];
}


static inline void pgps_rtcm_data_len_SET(int8_t src, pgps_rtcm_data_GPS_RTCM_DATA *dst) { //data length
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t vgps_rtcm_data_daTa_GET(Vgps_rtcm_data_daTa const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

static inline Vgps_rtcm_data_daTa pgps_rtcm_data_daTa_GET(pgps_rtcm_data_GPS_RTCM_DATA *src) { //RTCM message (may be fragmented)
	
	return (Vgps_rtcm_data_daTa) {src + 2, 180};
}


static inline void vgps_rtcm_data_daTa_SET(int8_t src, size_t index, Vgps_rtcm_data_daTa *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

static inline Vgps_rtcm_data_daTa pgps_rtcm_data_daTa_SET(const int8_t src[], pgps_rtcm_data_GPS_RTCM_DATA *dst) {//RTCM message (may be fragmented)
	
	
	Vgps_rtcm_data_daTa ret = {dst + 2, 180};
	
	if (src)
		for (size_t i = 0; i < 180; i++)
			vgps_rtcm_data_daTa_SET(src[i], i, &ret);
	return ret;
}


static inline int64_t pglobal_vision_position_estimate_usec_GET(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *src) {//Timestamp (microseconds, synced to UNIX time or since system boot)
	
	return ((int64_t) (get_bytes(src, 0, 8)));
}


static inline float pglobal_vision_position_estimate_x_GET(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *src) {//Global X position
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline float pglobal_vision_position_estimate_y_GET(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *src) {//Global Y position
	
	return (intBitsToFloat(get_bytes(src, 12, 4)));
}


static inline float pglobal_vision_position_estimate_z_GET(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *src) {//Global Z position
	
	return (intBitsToFloat(get_bytes(src, 16, 4)));
}


static inline float pglobal_vision_position_estimate_roll_GET(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *src) {//Roll angle in rad
	
	return (intBitsToFloat(get_bytes(src, 20, 4)));
}


static inline float pglobal_vision_position_estimate_pitch_GET(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *src) {//Pitch angle in rad
	
	return (intBitsToFloat(get_bytes(src, 24, 4)));
}


static inline float pglobal_vision_position_estimate_yaw_GET(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *src) {//Yaw angle in rad
	
	return (intBitsToFloat(get_bytes(src, 28, 4)));
}


static inline int8_t pfile_transfer_protocol_target_network_GET(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *src) {//Network ID (0 for broadcast)
	
	return (int8_t) src[0];
}


static inline void pfile_transfer_protocol_target_network_SET(int8_t src, pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *dst) { //Network ID (0 for broadcast)
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pfile_transfer_protocol_target_system_GET(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *src) {//System ID (0 for broadcast)
	
	return (int8_t) src[1];
}


static inline void pfile_transfer_protocol_target_system_SET(int8_t src, pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *dst) { //System ID (0 for broadcast)
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t pfile_transfer_protocol_target_component_GET(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *src) {//Component ID (0 for broadcast)
	
	return (int8_t) src[2];
}


static inline void pfile_transfer_protocol_target_component_SET(int8_t src, pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *dst) { //Component ID (0 for broadcast)
	
	
	dst[2] = (uint8_t) (src);
}

/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*					 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*					 message_type.  The particular encoding used can be extension specific and might not always be documented
*					 as part of the mavlink specification */

static inline int8_t vfile_transfer_protocol_payload_GET(Vfile_transfer_protocol_payload const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*					 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*					 message_type.  The particular encoding used can be extension specific and might not always be documented
*					 as part of the mavlink specification */

static inline Vfile_transfer_protocol_payload pfile_transfer_protocol_payload_GET(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *src) {
	
	return (Vfile_transfer_protocol_payload) {src + 3, 251};
}

/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*					 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*					 message_type.  The particular encoding used can be extension specific and might not always be documented
*					 as part of the mavlink specification */

static inline void vfile_transfer_protocol_payload_SET(int8_t src, size_t index, Vfile_transfer_protocol_payload *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*					 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*					 message_type.  The particular encoding used can be extension specific and might not always be documented
*					 as part of the mavlink specification */

static inline Vfile_transfer_protocol_payload pfile_transfer_protocol_payload_SET(const int8_t src[], pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *dst) {
	
	
	Vfile_transfer_protocol_payload ret = {dst + 3, 251};
	
	if (src)
		for (size_t i = 0; i < 251; i++)
			vfile_transfer_protocol_payload_SET(src[i], i, &ret);
	return ret;
}


static inline float prangefinder_distance_GET(prangefinder_RANGEFINDER *src) {//distance in meters
	
	return (intBitsToFloat(get_bytes(src, 0, 4)));
}


static inline void prangefinder_distance_SET(float src, prangefinder_RANGEFINDER *dst) { //distance in meters
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 0);
}


static inline float prangefinder_voltage_GET(prangefinder_RANGEFINDER *src) {//raw voltage if available, zero otherwise
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void prangefinder_voltage_SET(float src, prangefinder_RANGEFINDER *dst) { //raw voltage if available, zero otherwise
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline int16_t pradio_status_rxerrors_GET(pradio_status_RADIO_STATUS *src) {//Receive errors
	
	return ((int16_t) (get_bytes(src, 0, 2)));
}


static inline void pradio_status_rxerrors_SET(int16_t src, pradio_status_RADIO_STATUS *dst) { //Receive errors
	
	
	set_bytes((uint16_t) (src), 2, dst, 0);
}


static inline int16_t pradio_status_fixeD_GET(pradio_status_RADIO_STATUS *src) {//Count of error corrected packets
	
	return ((int16_t) (get_bytes(src, 2, 2)));
}


static inline void pradio_status_fixeD_SET(int16_t src, pradio_status_RADIO_STATUS *dst) { //Count of error corrected packets
	
	
	set_bytes((uint16_t) (src), 2, dst, 2);
}


static inline int8_t pradio_status_rssi_GET(pradio_status_RADIO_STATUS *src) {//Local signal strength
	
	return (int8_t) src[4];
}


static inline void pradio_status_rssi_SET(int8_t src, pradio_status_RADIO_STATUS *dst) { //Local signal strength
	
	
	dst[4] = (uint8_t) (src);
}


static inline int8_t pradio_status_remrssi_GET(pradio_status_RADIO_STATUS *src) {//Remote signal strength
	
	return (int8_t) src[5];
}


static inline void pradio_status_remrssi_SET(int8_t src, pradio_status_RADIO_STATUS *dst) { //Remote signal strength
	
	
	dst[5] = (uint8_t) (src);
}


static inline int8_t pradio_status_txbuf_GET(pradio_status_RADIO_STATUS *src) {//Remaining free buffer space in percent.
	
	return (int8_t) src[6];
}


static inline void pradio_status_txbuf_SET(int8_t src, pradio_status_RADIO_STATUS *dst) { //Remaining free buffer space in percent.
	
	
	dst[6] = (uint8_t) (src);
}


static inline int8_t pradio_status_noise_GET(pradio_status_RADIO_STATUS *src) {//Background noise level
	
	return (int8_t) src[7];
}


static inline void pradio_status_noise_SET(int8_t src, pradio_status_RADIO_STATUS *dst) { //Background noise level
	
	
	dst[7] = (uint8_t) (src);
}


static inline int8_t pradio_status_remnoise_GET(pradio_status_RADIO_STATUS *src) {//Remote background noise level
	
	return (int8_t) src[8];
}


static inline void pradio_status_remnoise_SET(int8_t src, pradio_status_RADIO_STATUS *dst) { //Remote background noise level
	
	
	dst[8] = (uint8_t) (src);
}


static inline int8_t pfence_point_target_system_GET(pfence_point_FENCE_POINT *src) {//System ID
	
	return (int8_t) src[0];
}


static inline void pfence_point_target_system_SET(int8_t src, pfence_point_FENCE_POINT *dst) { //System ID
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t pfence_point_target_component_GET(pfence_point_FENCE_POINT *src) {//Component ID
	
	return (int8_t) src[1];
}


static inline void pfence_point_target_component_SET(int8_t src, pfence_point_FENCE_POINT *dst) { //Component ID
	
	
	dst[1] = (uint8_t) (src);
}


static inline int8_t pfence_point_idx_GET(pfence_point_FENCE_POINT *src) {//point index (first point is 1, 0 is for return point)
	
	return (int8_t) src[2];
}


static inline void pfence_point_idx_SET(int8_t src, pfence_point_FENCE_POINT *dst) { //point index (first point is 1, 0 is for return point)
	
	
	dst[2] = (uint8_t) (src);
}


static inline int8_t pfence_point_count_GET(pfence_point_FENCE_POINT *src) {//total number of points (for sanity checking)
	
	return (int8_t) src[3];
}


static inline void pfence_point_count_SET(int8_t src, pfence_point_FENCE_POINT *dst) { //total number of points (for sanity checking)
	
	
	dst[3] = (uint8_t) (src);
}


static inline float pfence_point_lat_GET(pfence_point_FENCE_POINT *src) {//Latitude of point
	
	return (intBitsToFloat(get_bytes(src, 4, 4)));
}


static inline void pfence_point_lat_SET(float src, pfence_point_FENCE_POINT *dst) { //Latitude of point
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 4);
}


static inline float pfence_point_lng_GET(pfence_point_FENCE_POINT *src) {//Longitude of point
	
	return (intBitsToFloat(get_bytes(src, 8, 4)));
}


static inline void pfence_point_lng_SET(float src, pfence_point_FENCE_POINT *dst) { //Longitude of point
	
	
	set_bytes(*(uint32_t *) &(src), 4, dst, 8);
}


static inline int8_t presource_request_request_id_GET(presource_request_RESOURCE_REQUEST *src) {//Request ID. This ID should be re-used when sending back URI contents
	
	return (int8_t) src[0];
}


static inline void presource_request_request_id_SET(int8_t src, presource_request_RESOURCE_REQUEST *dst) { //Request ID. This ID should be re-used when sending back URI contents
	
	
	dst[0] = (uint8_t) (src);
}


static inline int8_t presource_request_uri_type_GET(presource_request_RESOURCE_REQUEST *src) {//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
	
	return (int8_t) src[1];
}


static inline void presource_request_uri_type_SET(int8_t src, presource_request_RESOURCE_REQUEST *dst) { //The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
	
	
	dst[1] = (uint8_t) (src);
}

/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*					 on the URI type enum */

static inline int8_t vresource_request_uri_GET(Vresource_request_uri const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*					 on the URI type enum */

static inline Vresource_request_uri presource_request_uri_GET(presource_request_RESOURCE_REQUEST *src) {
	
	return (Vresource_request_uri) {src + 2, 120};
}

/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*					 on the URI type enum */

static inline void vresource_request_uri_SET(int8_t src, size_t index, Vresource_request_uri *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*					 on the URI type enum */

static inline Vresource_request_uri presource_request_uri_SET(const int8_t src[], presource_request_RESOURCE_REQUEST *dst) {
	
	
	Vresource_request_uri ret = {dst + 2, 120};
	
	if (src)
		for (size_t i = 0; i < 120; i++)
			vresource_request_uri_SET(src[i], i, &ret);
	return ret;
}


static inline int8_t presource_request_transfer_type_GET(presource_request_RESOURCE_REQUEST *src) {//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
	
	return (int8_t) src[122];
}


static inline void presource_request_transfer_type_SET(int8_t src, presource_request_RESOURCE_REQUEST *dst) { //The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
	
	
	dst[122] = (uint8_t) (src);
}

/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*					 has a storage associated (e.g. MAVLink FTP) */

static inline int8_t vresource_request_storage_GET(Vresource_request_storage const *const src, size_t index) { return (int8_t) src->bytes[index * 1]; }

/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*					 has a storage associated (e.g. MAVLink FTP) */

static inline Vresource_request_storage presource_request_storage_GET(presource_request_RESOURCE_REQUEST *src) {
	
	return (Vresource_request_storage) {src + 123, 120};
}

/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*					 has a storage associated (e.g. MAVLink FTP) */

static inline void vresource_request_storage_SET(int8_t src, size_t index, Vresource_request_storage *dst) { dst->bytes[index * 1] = (uint8_t) (src); }

/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*					 has a storage associated (e.g. MAVLink FTP) */

static inline Vresource_request_storage presource_request_storage_SET(const int8_t src[], presource_request_RESOURCE_REQUEST *dst) {
	
	
	Vresource_request_storage ret = {dst + 123, 120};
	
	if (src)
		for (size_t i = 0; i < 120; i++)
			vresource_request_storage_SET(src[i], i, &ret);
	return ret;
}


//#######################                 c_CommunicationChannel                       ##################################
typedef struct {
	Receiver    receiver;
	Transmitter transmitter;
} c_CommunicationChannel;


/*
As the component of receiving facilities
c_CommunicationChannel_DISPATCHER(CHANNEL) macro is generating packet dispatcher source code.
The CHANNEL parameter is an arbitrary prefix of related to dispatcher functions

for example if CHANNEL parameter is MY_DISP

The macro will produce code like this

switch (id) {
	 case 1:
	    return pack ? MY_DISP_on_SensorData( channel , pack ), NULL : &m_Pack;
      ...
}

The code expects MY_DISP_on_SensorData - SensorData packet receiving handler function and pointer to the channel in the scope. You have to provide it!

if CHANNEL parameter is BLA_BLA

The macro will issue

switch (id) {
	 case 1:
	    return pack ? BLA_BLA_on_SensorData( channel , pack ), NULL : &m_Pack;
     ...
}

please use more meaningful prefixes
c_CommunicationChannel_DISPATCHER(CHANNEL) macro expecting the following functions in the scope
------------------------------------

static inline void <CHANNEL>_on_REQUEST_DATA_STREAM( <Channel> * channel, REQUEST_DATA_STREAM_request_data_stream * pack ){}
static inline void <CHANNEL>_on_MISSION_CURRENT( <Channel> * channel, MISSION_CURRENT_mission_current * pack ){}
static inline void <CHANNEL>_on_MISSION_REQUEST_PARTIAL_LIST( <Channel> * channel, MISSION_REQUEST_PARTIAL_LIST_mission_request_partial_list * pack ){}
static inline void <CHANNEL>_on_ATTITUDE( <Channel> * channel, ATTITUDE_attitude * pack ){}
static inline void <CHANNEL>_on_RADIO_STATUS( <Channel> * channel, RADIO_STATUS_radio_status * pack ){}
static inline void <CHANNEL>_on_AUTH_KEY( <Channel> * channel, AUTH_KEY_auth_key * pack ){}
static inline void <CHANNEL>_on_POSITION_TARGET_LOCAL_NED( <Channel> * channel, POSITION_TARGET_LOCAL_NED_position_target_local_ned * pack ){}
static inline void <CHANNEL>_on_FOLLOW_TARGET( <Channel> * channel, FOLLOW_TARGET_follow_target * pack ){}
static inline void <CHANNEL>_on_DATA_STREAM( <Channel> * channel, DATA_STREAM_data_stream * pack ){}
static inline void <CHANNEL>_on_RC_CHANNELS_OVERRIDE( <Channel> * channel, RC_CHANNELS_OVERRIDE_rc_channels_override * pack ){}
static inline void <CHANNEL>_on_GLOBAL_POSITION_INT( <Channel> * channel, GLOBAL_POSITION_INT_global_position_int * pack ){}
static inline void <CHANNEL>_on_COMMAND_ACK( <Channel> * channel, COMMAND_ACK_command_ack * pack ){}
static inline void <CHANNEL>_on_MISSION_ITEM_INT( <Channel> * channel, MISSION_ITEM_INT_mission_item_int * pack ){}
static inline void <CHANNEL>_on_LOG_REQUEST_LIST( <Channel> * channel, LOG_REQUEST_LIST_log_request_list * pack ){}
static inline void <CHANNEL>_on_BATTERY_STATUS( <Channel> * channel, BATTERY_STATUS_battery_status * pack ){}
static inline void <CHANNEL>_on_CAMERA_IMAGE_CAPTURED( <Channel> * channel, CAMERA_IMAGE_CAPTURED_camera_image_captured * pack ){}
static inline void <CHANNEL>_on_PLAY_TUNE( <Channel> * channel, PLAY_TUNE_play_tune * pack ){}
static inline void <CHANNEL>_on_TERRAIN_CHECK( <Channel> * channel, TERRAIN_CHECK_terrain_check * pack ){}
static inline void <CHANNEL>_on_GPS2_RTK( <Channel> * channel, GPS2_RTK_gps2_rtk * pack ){}
static inline void <CHANNEL>_on_SET_ACTUATOR_CONTROL_TARGET( <Channel> * channel, SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target * pack ){}
static inline void <CHANNEL>_on_PING33( <Channel> * channel, PING33_ping33 * pack ){}
static inline void <CHANNEL>_on_CONTROL_SYSTEM_STATE( <Channel> * channel, CONTROL_SYSTEM_STATE_control_system_state * pack ){}
static inline void <CHANNEL>_on_GLOBAL_VISION_POSITION_ESTIMATE( <Channel> * channel, GLOBAL_VISION_POSITION_ESTIMATE_global_vision_position_estimate * pack ){}
static inline void <CHANNEL>_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET( <Channel> * channel, LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_local_position_ned_system_global_offset * pack ){}
static inline void <CHANNEL>_on_ATTITUDE_TARGET( <Channel> * channel, ATTITUDE_TARGET_attitude_target * pack ){}
static inline void <CHANNEL>_on_GPS_STATUS( <Channel> * channel, GPS_STATUS_gps_status * pack ){}
static inline void <CHANNEL>_on_LOCAL_POSITION_NED_COV( <Channel> * channel, LOCAL_POSITION_NED_COV_local_position_ned_cov * pack ){}
static inline void <CHANNEL>_on_NAMED_VALUE_FLOAT( <Channel> * channel, NAMED_VALUE_FLOAT_named_value_float * pack ){}
static inline void <CHANNEL>_on_SCALED_PRESSURE2( <Channel> * channel, SCALED_PRESSURE2_scaled_pressure2 * pack ){}
static inline void <CHANNEL>_on_HIL_ACTUATOR_CONTROLS( <Channel> * channel, HIL_ACTUATOR_CONTROLS_hil_actuator_controls * pack ){}
static inline void <CHANNEL>_on_MISSION_ITEM_REACHED( <Channel> * channel, MISSION_ITEM_REACHED_mission_item_reached * pack ){}
static inline void <CHANNEL>_on_MISSION_COUNT( <Channel> * channel, MISSION_COUNT_mission_count * pack ){}
static inline void <CHANNEL>_on_OPTICAL_FLOW( <Channel> * channel, OPTICAL_FLOW_optical_flow * pack ){}
static inline void <CHANNEL>_on_GPS_INPUT( <Channel> * channel, GPS_INPUT_gps_input * pack ){}
static inline void <CHANNEL>_on_TERRAIN_REQUEST( <Channel> * channel, TERRAIN_REQUEST_terrain_request * pack ){}
static inline void <CHANNEL>_on_HIL_OPTICAL_FLOW( <Channel> * channel, HIL_OPTICAL_FLOW_hil_optical_flow * pack ){}
static inline void <CHANNEL>_on_SCALED_IMU3( <Channel> * channel, SCALED_IMU3_scaled_imu3 * pack ){}
static inline void <CHANNEL>_on_TERRAIN_DATA( <Channel> * channel, TERRAIN_DATA_terrain_data * pack ){}
static inline void <CHANNEL>_on_HIL_SENSOR( <Channel> * channel, HIL_SENSOR_hil_sensor * pack ){}
static inline void <CHANNEL>_on_POSITION_TARGET_GLOBAL_INT( <Channel> * channel, POSITION_TARGET_GLOBAL_INT_position_target_global_int * pack ){}
static inline void <CHANNEL>_on_RAW_PRESSURE( <Channel> * channel, RAW_PRESSURE_raw_pressure * pack ){}
static inline void <CHANNEL>_on_GPS_RAW_INT( <Channel> * channel, GPS_RAW_INT_gps_raw_int * pack ){}
static inline void <CHANNEL>_on_GPS_GLOBAL_ORIGIN( <Channel> * channel, GPS_GLOBAL_ORIGIN_gps_global_origin * pack ){}
static inline void <CHANNEL>_on_MISSION_REQUEST_LIST( <Channel> * channel, MISSION_REQUEST_LIST_mission_request_list * pack ){}
static inline void <CHANNEL>_on_CHANGE_OPERATOR_CONTROL_ACK( <Channel> * channel, CHANGE_OPERATOR_CONTROL_ACK_change_operator_control_ack * pack ){}
static inline void <CHANNEL>_on_HIL_STATE_QUATERNION( <Channel> * channel, HIL_STATE_QUATERNION_hil_state_quaternion * pack ){}
static inline void <CHANNEL>_on_MEMORY_VECT( <Channel> * channel, MEMORY_VECT_memory_vect * pack ){}
static inline void <CHANNEL>_on_GPS_INJECT_DATA( <Channel> * channel, GPS_INJECT_DATA_gps_inject_data * pack ){}
static inline void <CHANNEL>_on_GLOBAL_POSITION_INT_COV( <Channel> * channel, GLOBAL_POSITION_INT_COV_global_position_int_cov * pack ){}
static inline void <CHANNEL>_on_LOG_ERASE( <Channel> * channel, LOG_ERASE_log_erase * pack ){}
static inline void <CHANNEL>_on_SIM_STATE( <Channel> * channel, SIM_STATE_sim_state * pack ){}
static inline void <CHANNEL>_on_RESOURCE_REQUEST( <Channel> * channel, RESOURCE_REQUEST_resource_request * pack ){}
static inline void <CHANNEL>_on_LOG_REQUEST_DATA( <Channel> * channel, LOG_REQUEST_DATA_log_request_data * pack ){}
static inline void <CHANNEL>_on_LOCAL_POSITION_NED( <Channel> * channel, LOCAL_POSITION_NED_local_position_ned * pack ){}
static inline void <CHANNEL>_on_SERVO_OUTPUT_RAW( <Channel> * channel, SERVO_OUTPUT_RAW_servo_output_raw * pack ){}
static inline void <CHANNEL>_on_SERIAL_CONTROL( <Channel> * channel, SERIAL_CONTROL_serial_control * pack ){}
static inline void <CHANNEL>_on_TIMESYNC( <Channel> * channel, TIMESYNC_timesync * pack ){}
static inline void <CHANNEL>_on_DISTANCE_SENSOR( <Channel> * channel, DISTANCE_SENSOR_distance_sensor * pack ){}
static inline void <CHANNEL>_on_HIL_STATE( <Channel> * channel, HIL_STATE_hil_state * pack ){}
static inline void <CHANNEL>_on_MISSION_ACK( <Channel> * channel, MISSION_ACK_mission_ack * pack ){}
static inline void <CHANNEL>_on_STATUSTEXT( <Channel> * channel, STATUSTEXT_statustext * pack ){}
static inline void <CHANNEL>_on_HIGHRES_IMU( <Channel> * channel, HIGHRES_IMU_highres_imu * pack ){}
static inline void <CHANNEL>_on_SET_HOME_POSITION( <Channel> * channel, SET_HOME_POSITION_set_home_position * pack ){}
static inline void <CHANNEL>_on_GPS_RTCM_DATA( <Channel> * channel, GPS_RTCM_DATA_gps_rtcm_data * pack ){}
static inline void <CHANNEL>_on_LOG_ENTRY( <Channel> * channel, LOG_ENTRY_log_entry * pack ){}
static inline void <CHANNEL>_on_ESTIMATOR_STATUS( <Channel> * channel, ESTIMATOR_STATUS_estimator_status * pack ){}
static inline void <CHANNEL>_on_MISSION_REQUEST( <Channel> * channel, MISSION_REQUEST_mission_request * pack ){}
static inline void <CHANNEL>_on_DEBUG( <Channel> * channel, DEBUG_debug * pack ){}
static inline void <CHANNEL>_on_HOME_POSITION( <Channel> * channel, HOME_POSITION_home_position * pack ){}
static inline void <CHANNEL>_on_SYSTEM_TIME( <Channel> * channel, SYSTEM_TIME_system_time * pack ){}
static inline void <CHANNEL>_on_ALTITUDE( <Channel> * channel, ALTITUDE_altitude * pack ){}
static inline void <CHANNEL>_on_MISSION_REQUEST_INT( <Channel> * channel, MISSION_REQUEST_INT_mission_request_int * pack ){}
static inline void <CHANNEL>_on_RC_CHANNELS( <Channel> * channel, RC_CHANNELS_rc_channels * pack ){}
static inline void <CHANNEL>_on_STORAGE_INFORMATION( <Channel> * channel, STORAGE_INFORMATION_storage_information * pack ){}
static inline void <CHANNEL>_on_MISSION_SET_CURRENT( <Channel> * channel, MISSION_SET_CURRENT_mission_set_current * pack ){}
static inline void <CHANNEL>_on_SAFETY_ALLOWED_AREA( <Channel> * channel, SAFETY_ALLOWED_AREA_safety_allowed_area * pack ){}
static inline void <CHANNEL>_on_COMMAND_INT( <Channel> * channel, COMMAND_INT_command_int * pack ){}
static inline void <CHANNEL>_on_PING( <Channel> * channel, PING_ping * pack ){}
static inline void <CHANNEL>_on_GPS_RTK( <Channel> * channel, GPS_RTK_gps_rtk * pack ){}
static inline void <CHANNEL>_on_MANUAL_SETPOINT( <Channel> * channel, MANUAL_SETPOINT_manual_setpoint * pack ){}
static inline void <CHANNEL>_on_HIGH_LATENCY( <Channel> * channel, HIGH_LATENCY_high_latency * pack ){}
static inline void <CHANNEL>_on_VIBRATION( <Channel> * channel, VIBRATION_vibration * pack ){}
static inline void <CHANNEL>_on_COLLISION( <Channel> * channel, COLLISION_collision * pack ){}
static inline void <CHANNEL>_on_LOG_REQUEST_END( <Channel> * channel, LOG_REQUEST_END_log_request_end * pack ){}
static inline void <CHANNEL>_on_CAMERA_TRIGGER( <Channel> * channel, CAMERA_TRIGGER_camera_trigger * pack ){}
static inline void <CHANNEL>_on_LANDING_TARGET( <Channel> * channel, LANDING_TARGET_landing_target * pack ){}
static inline void <CHANNEL>_on_ATTITUDE_QUATERNION( <Channel> * channel, ATTITUDE_QUATERNION_attitude_quaternion * pack ){}
static inline void <CHANNEL>_on_HIL_GPS( <Channel> * channel, HIL_GPS_hil_gps * pack ){}
static inline void <CHANNEL>_on_PARAM_VALUE( <Channel> * channel, PARAM_VALUE_param_value * pack ){}
static inline void <CHANNEL>_on_DEBUG_VECT( <Channel> * channel, DEBUG_VECT_debug_vect * pack ){}
static inline void <CHANNEL>_on_HEARTBEAT( <Channel> * channel, HEARTBEAT_heartbeat * pack ){}
static inline void <CHANNEL>_on_SYS_STATUS( <Channel> * channel, SYS_STATUS_sys_status * pack ){}
static inline void <CHANNEL>_on_SET_POSITION_TARGET_GLOBAL_INT( <Channel> * channel, SET_POSITION_TARGET_GLOBAL_INT_set_position_target_global_int * pack ){}
static inline void <CHANNEL>_on_PARAM_REQUEST_LIST( <Channel> * channel, PARAM_REQUEST_LIST_param_request_list * pack ){}
static inline void <CHANNEL>_on_PARAM_SET( <Channel> * channel, PARAM_SET_param_set * pack ){}
static inline void <CHANNEL>_on_AUTOPILOT_VERSION( <Channel> * channel, AUTOPILOT_VERSION_autopilot_version * pack ){}
static inline void <CHANNEL>_on_VISION_SPEED_ESTIMATE( <Channel> * channel, VISION_SPEED_ESTIMATE_vision_speed_estimate * pack ){}
static inline void <CHANNEL>_on_ACTUATOR_CONTROL_TARGET( <Channel> * channel, ACTUATOR_CONTROL_TARGET_actuator_control_target * pack ){}
static inline void <CHANNEL>_on_SCALED_PRESSURE( <Channel> * channel, SCALED_PRESSURE_scaled_pressure * pack ){}
static inline void <CHANNEL>_on_GPS2_RAW( <Channel> * channel, GPS2_RAW_gps2_raw * pack ){}
static inline void <CHANNEL>_on_SET_ATTITUDE_TARGET( <Channel> * channel, SET_ATTITUDE_TARGET_set_attitude_target * pack ){}
static inline void <CHANNEL>_on_FILE_TRANSFER_PROTOCOL( <Channel> * channel, FILE_TRANSFER_PROTOCOL_file_transfer_protocol * pack ){}
static inline void <CHANNEL>_on_MANUAL_CONTROL( <Channel> * channel, MANUAL_CONTROL_manual_control * pack ){}
static inline void <CHANNEL>_on_SAFETY_SET_ALLOWED_AREA( <Channel> * channel, SAFETY_SET_ALLOWED_AREA_safety_set_allowed_area * pack ){}
static inline void <CHANNEL>_on_ADSB_VEHICLE( <Channel> * channel, ADSB_VEHICLE_adsb_vehicle * pack ){}
static inline void <CHANNEL>_on_CAMERA_INFORMATION( <Channel> * channel, CAMERA_INFORMATION_camera_information * pack ){}
static inline void <CHANNEL>_on_ENCAPSULATED_DATA( <Channel> * channel, ENCAPSULATED_DATA_encapsulated_data * pack ){}
static inline void <CHANNEL>_on_RC_CHANNELS_SCALED( <Channel> * channel, RC_CHANNELS_SCALED_rc_channels_scaled * pack ){}
static inline void <CHANNEL>_on_SwitchModeCommand( <Channel> * channel ){}
static inline void <CHANNEL>_on_FLIGHT_INFORMATION( <Channel> * channel, FLIGHT_INFORMATION_flight_information * pack ){}
static inline void <CHANNEL>_on_MISSION_WRITE_PARTIAL_LIST( <Channel> * channel, MISSION_WRITE_PARTIAL_LIST_mission_write_partial_list * pack ){}
static inline void <CHANNEL>_on_WIND_COV( <Channel> * channel, WIND_COV_wind_cov * pack ){}
static inline void <CHANNEL>_on_MISSION_ITEM( <Channel> * channel, MISSION_ITEM_mission_item * pack ){}
static inline void <CHANNEL>_on_MISSION_CLEAR_ALL( <Channel> * channel, MISSION_CLEAR_ALL_mission_clear_all * pack ){}
static inline void <CHANNEL>_on_SET_MODE( <Channel> * channel, SET_MODE_set_mode * pack ){}
static inline void <CHANNEL>_on_VISION_POSITION_ESTIMATE( <Channel> * channel, VISION_POSITION_ESTIMATE_vision_position_estimate * pack ){}
static inline void <CHANNEL>_on_VFR_HUD( <Channel> * channel, VFR_HUD_vfr_hud * pack ){}
static inline void <CHANNEL>_on_TERRAIN_REPORT( <Channel> * channel, TERRAIN_REPORT_terrain_report * pack ){}
static inline void <CHANNEL>_on_NAV_CONTROLLER_OUTPUT( <Channel> * channel, NAV_CONTROLLER_OUTPUT_nav_controller_output * pack ){}
static inline void <CHANNEL>_on_NAMED_VALUE_INT( <Channel> * channel, NAMED_VALUE_INT_named_value_int * pack ){}
static inline void <CHANNEL>_on_PARAM_MAP_RC( <Channel> * channel, PARAM_MAP_RC_param_map_rc * pack ){}
static inline void <CHANNEL>_on_COMMAND_LONG( <Channel> * channel, COMMAND_LONG_command_long * pack ){}
static inline void <CHANNEL>_on_SET_GPS_GLOBAL_ORIGIN( <Channel> * channel, SET_GPS_GLOBAL_ORIGIN_set_gps_global_origin * pack ){}
static inline void <CHANNEL>_on_LOG_DATA( <Channel> * channel, LOG_DATA_log_data * pack ){}
static inline void <CHANNEL>_on_POWER_STATUS( <Channel> * channel, POWER_STATUS_power_status * pack ){}
static inline void <CHANNEL>_on_BUTTON_CHANGE( <Channel> * channel, BUTTON_CHANGE_button_change * pack ){}
static inline void <CHANNEL>_on_DATA_TRANSMISSION_HANDSHAKE( <Channel> * channel, DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake * pack ){}
static inline void <CHANNEL>_on_RAW_IMU( <Channel> * channel, RAW_IMU_raw_imu * pack ){}
static inline void <CHANNEL>_on_V2_EXTENSION( <Channel> * channel, V2_EXTENSION_v2_extension * pack ){}
static inline void <CHANNEL>_on_HIL_CONTROLS( <Channel> * channel, HIL_CONTROLS_hil_controls * pack ){}
static inline void <CHANNEL>_on_EXTENDED_SYS_STATE( <Channel> * channel, EXTENDED_SYS_STATE_extended_sys_state * pack ){}
static inline void <CHANNEL>_on_HIL_RC_INPUTS_RAW( <Channel> * channel, HIL_RC_INPUTS_RAW_hil_rc_inputs_raw * pack ){}
static inline void <CHANNEL>_on_ATTITUDE_QUATERNION_COV( <Channel> * channel, ATTITUDE_QUATERNION_COV_attitude_quaternion_cov * pack ){}
static inline void <CHANNEL>_on_ATT_POS_MOCAP( <Channel> * channel, ATT_POS_MOCAP_att_pos_mocap * pack ){}
static inline void <CHANNEL>_on_OPTICAL_FLOW_RAD( <Channel> * channel, OPTICAL_FLOW_RAD_optical_flow_rad * pack ){}
static inline void <CHANNEL>_on_SETUP_SIGNING( <Channel> * channel, SETUP_SIGNING_setup_signing * pack ){}
static inline void <CHANNEL>_on_CAMERA_SETTINGS( <Channel> * channel, CAMERA_SETTINGS_camera_settings * pack ){}
static inline void <CHANNEL>_on_CAMERA_CAPTURE_STATUS( <Channel> * channel, CAMERA_CAPTURE_STATUS_camera_capture_status * pack ){}
static inline void <CHANNEL>_on_CHANGE_OPERATOR_CONTROL( <Channel> * channel, CHANGE_OPERATOR_CONTROL_change_operator_control * pack ){}
static inline void <CHANNEL>_on_SCALED_PRESSURE3( <Channel> * channel, SCALED_PRESSURE3_scaled_pressure3 * pack ){}
static inline void <CHANNEL>_on_RC_CHANNELS_RAW( <Channel> * channel, RC_CHANNELS_RAW_rc_channels_raw * pack ){}
static inline void <CHANNEL>_on_SCALED_IMU( <Channel> * channel, SCALED_IMU_scaled_imu * pack ){}
static inline void <CHANNEL>_on_SET_POSITION_TARGET_LOCAL_NED( <Channel> * channel, SET_POSITION_TARGET_LOCAL_NED_set_position_target_local_ned * pack ){}
static inline void <CHANNEL>_on_PARAM_REQUEST_READ( <Channel> * channel, PARAM_REQUEST_READ_param_request_read * pack ){}
static inline void <CHANNEL>_on_SCALED_IMU2( <Channel> * channel, SCALED_IMU2_scaled_imu2 * pack ){}
static inline void <CHANNEL>_on_VICON_POSITION_ESTIMATE( <Channel> * channel, VICON_POSITION_ESTIMATE_vicon_position_estimate * pack ){}
static inline void <CHANNEL>_on_MESSAGE_INTERVAL( <Channel> * channel, MESSAGE_INTERVAL_message_interval * pack ){}

-----------------------------------
Please, just copy-paste code above and replace:
<CHANNEL>  with your prefix
<Channel>  with appropriate channel typedef

Particularly this macro helps to build dispatcher functions for channel receiver

static inline Meta* dispatcher (Receiver* receiver, size_t id, Pack* pack)
{
	MyChannel * channel = ...// here is the code to get the pointer to Channel by pointer to Channel Receiver
	c_CommunicationChannel_DISPATCHER( BLA_BLA )
}
*/
#define c_CommunicationChannel_DISPATCHER(CHANNEL)\
        switch (id)\
        {\
            default:    return NULL;\
            case 87:    return pack ? CHANNEL##_on_HIGH_LATENCY( channel , pack ), (Meta const*)NULL : &mhigh_latency_HIGH_LATENCY;\
            case 19:    return pack ? CHANNEL##_on_RAW_PRESSURE( channel , pack ), (Meta const*)NULL : &mraw_pressure_RAW_PRESSURE;\
            case 35:    return pack ? CHANNEL##_on_MISSION_ITEM( channel , pack ), (Meta const*)NULL : &mmission_item_MISSION_ITEM;\
            case 169:    return pack ? CHANNEL##_on_HIL_SENSOR( channel , pack ), (Meta const*)NULL : &mhil_sensor_HIL_SENSOR;\
            case 98:    return pack ? CHANNEL##_on_REQUEST_DATA_STREAM( channel , pack ), (Meta const*)NULL : &mrequest_data_stream_REQUEST_DATA_STREAM;\
            case 188:    return pack ? CHANNEL##_on_MISSION_REQUEST_INT( channel , pack ), (Meta const*)NULL : &mmission_request_int_MISSION_REQUEST_INT;\
            case 42:    return pack ? CHANNEL##_on_VISION_SPEED_ESTIMATE( channel , pack ), (Meta const*)NULL : &mvision_speed_estimate_VISION_SPEED_ESTIMATE;\
            case 161:    return pack ? CHANNEL##_on_LOG_ENTRY( channel , pack ), (Meta const*)NULL : &mlog_entry_LOG_ENTRY;\
            case 31:    return pack ? CHANNEL##_on_DATA_STREAM( channel , pack ), (Meta const*)NULL : &mdata_stream_DATA_STREAM;\
            case 37:    return pack ? CHANNEL##_on_LOG_REQUEST_LIST( channel , pack ), (Meta const*)NULL : &mlog_request_list_LOG_REQUEST_LIST;\
            case 41:    return pack ? CHANNEL##_on_MESSAGE_INTERVAL( channel , pack ), (Meta const*)NULL : &mmessage_interval_MESSAGE_INTERVAL;\
            case 123:    return pack ? CHANNEL##_on_PARAM_SET( channel , pack ), (Meta const*)NULL : &mparam_set_PARAM_SET;\
            case 191:    return pack ? CHANNEL##_on_CAMERA_TRIGGER( channel , pack ), (Meta const*)NULL : &mcamera_trigger_CAMERA_TRIGGER;\
            case 28:    return pack ? CHANNEL##_on_SwitchModeCommand( channel ), (Meta const*)NULL : &mswitchmodecommand_SwitchModeCommand;\
            case 184:    return pack ? CHANNEL##_on_HIL_RC_INPUTS_RAW( channel , pack ), (Meta const*)NULL : &mhil_rc_inputs_raw_HIL_RC_INPUTS_RAW;\
            case 124:    return pack ? CHANNEL##_on_STATUSTEXT( channel , pack ), (Meta const*)NULL : &mstatustext_STATUSTEXT;\
            case 67:    return pack ? CHANNEL##_on_RESOURCE_REQUEST( channel , pack ), (Meta const*)NULL : &mresource_request_RESOURCE_REQUEST;\
            case 89:    return pack ? CHANNEL##_on_GPS_RTK( channel , pack ), (Meta const*)NULL : &mgps_rtk_GPS_RTK;\
            case 116:    return pack ? CHANNEL##_on_HIL_OPTICAL_FLOW( channel , pack ), (Meta const*)NULL : &mhil_optical_flow_HIL_OPTICAL_FLOW;\
            case 79:    return pack ? CHANNEL##_on_SERVO_OUTPUT_RAW( channel , pack ), (Meta const*)NULL : &mservo_output_raw_SERVO_OUTPUT_RAW;\
            case 209:    return pack ? CHANNEL##_on_SET_POSITION_TARGET_LOCAL_NED( channel , pack ), (Meta const*)NULL : &mset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED;\
            case 34:    return pack ? CHANNEL##_on_ALTITUDE( channel , pack ), (Meta const*)NULL : &maltitude_ALTITUDE;\
            case 107:    return pack ? CHANNEL##_on_RADIO_STATUS( channel , pack ), (Meta const*)NULL : &mradio_status_RADIO_STATUS;\
            case 176:    return pack ? CHANNEL##_on_EXTENDED_SYS_STATE( channel , pack ), (Meta const*)NULL : &mextended_sys_state_EXTENDED_SYS_STATE;\
            case 181:    return pack ? CHANNEL##_on_CAMERA_SETTINGS( channel , pack ), (Meta const*)NULL : &mcamera_settings_CAMERA_SETTINGS;\
            case 139:    return pack ? CHANNEL##_on_PING33( channel , pack ), (Meta const*)NULL : &mping33_PING33;\
            case 129:    return pack ? CHANNEL##_on_POSITION_TARGET_LOCAL_NED( channel , pack ), (Meta const*)NULL : &mposition_target_local_ned_POSITION_TARGET_LOCAL_NED;\
            case 13:    return pack ? CHANNEL##_on_OPTICAL_FLOW( channel , pack ), (Meta const*)NULL : &moptical_flow_OPTICAL_FLOW;\
            case 39:    return pack ? CHANNEL##_on_FOLLOW_TARGET( channel , pack ), (Meta const*)NULL : &mfollow_target_FOLLOW_TARGET;\
            case 77:    return pack ? CHANNEL##_on_SET_HOME_POSITION( channel , pack ), (Meta const*)NULL : &mset_home_position_SET_HOME_POSITION;\
            case 55:    return pack ? CHANNEL##_on_RC_CHANNELS_OVERRIDE( channel , pack ), (Meta const*)NULL : &mrc_channels_override_RC_CHANNELS_OVERRIDE;\
            case 15:    return pack ? CHANNEL##_on_VFR_HUD( channel , pack ), (Meta const*)NULL : &mvfr_hud_VFR_HUD;\
            case 210:    return pack ? CHANNEL##_on_HOME_POSITION( channel , pack ), (Meta const*)NULL : &mhome_position_HOME_POSITION;\
            case 154:    return pack ? CHANNEL##_on_CHANGE_OPERATOR_CONTROL( channel , pack ), (Meta const*)NULL : &mchange_operator_control_CHANGE_OPERATOR_CONTROL;\
            case 202:    return pack ? CHANNEL##_on_OPTICAL_FLOW_RAD( channel , pack ), (Meta const*)NULL : &moptical_flow_rad_OPTICAL_FLOW_RAD;\
            case 103:    return pack ? CHANNEL##_on_VIBRATION( channel , pack ), (Meta const*)NULL : &mvibration_VIBRATION;\
            case 99:    return pack ? CHANNEL##_on_SERIAL_CONTROL( channel , pack ), (Meta const*)NULL : &mserial_control_SERIAL_CONTROL;\
            case 133:    return pack ? CHANNEL##_on_POSITION_TARGET_GLOBAL_INT( channel , pack ), (Meta const*)NULL : &mposition_target_global_int_POSITION_TARGET_GLOBAL_INT;\
            case 2:    return pack ? CHANNEL##_on_SCALED_IMU3( channel , pack ), (Meta const*)NULL : &mscaled_imu3_SCALED_IMU3;\
            case 195:    return pack ? CHANNEL##_on_SCALED_IMU( channel , pack ), (Meta const*)NULL : &mscaled_imu_SCALED_IMU;\
            case 168:    return pack ? CHANNEL##_on_HIL_GPS( channel , pack ), (Meta const*)NULL : &mhil_gps_HIL_GPS;\
            case 86:    return pack ? CHANNEL##_on_TERRAIN_REPORT( channel , pack ), (Meta const*)NULL : &mterrain_report_TERRAIN_REPORT;\
            case 178:    return pack ? CHANNEL##_on_SCALED_IMU2( channel , pack ), (Meta const*)NULL : &mscaled_imu2_SCALED_IMU2;\
            case 21:    return pack ? CHANNEL##_on_NAMED_VALUE_INT( channel , pack ), (Meta const*)NULL : &mnamed_value_int_NAMED_VALUE_INT;\
            case 174:    return pack ? CHANNEL##_on_V2_EXTENSION( channel , pack ), (Meta const*)NULL : &mv2_extension_V2_EXTENSION;\
            case 158:    return pack ? CHANNEL##_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET( channel , pack ), (Meta const*)NULL : &mlocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;\
            case 137:    return pack ? CHANNEL##_on_HIGHRES_IMU( channel , pack ), (Meta const*)NULL : &mhighres_imu_HIGHRES_IMU;\
            case 216:    return pack ? CHANNEL##_on_CAMERA_CAPTURE_STATUS( channel , pack ), (Meta const*)NULL : &mcamera_capture_status_CAMERA_CAPTURE_STATUS;\
            case 50:    return pack ? CHANNEL##_on_MISSION_COUNT( channel , pack ), (Meta const*)NULL : &mmission_count_MISSION_COUNT;\
            case 215:    return pack ? CHANNEL##_on_FLIGHT_INFORMATION( channel , pack ), (Meta const*)NULL : &mflight_information_FLIGHT_INFORMATION;\
            case 185:    return pack ? CHANNEL##_on_GPS_GLOBAL_ORIGIN( channel , pack ), (Meta const*)NULL : &mgps_global_origin_GPS_GLOBAL_ORIGIN;\
            case 25:    return pack ? CHANNEL##_on_MISSION_REQUEST_LIST( channel , pack ), (Meta const*)NULL : &mmission_request_list_MISSION_REQUEST_LIST;\
            case 140:    return pack ? CHANNEL##_on_HIL_STATE( channel , pack ), (Meta const*)NULL : &mhil_state_HIL_STATE;\
            case 151:    return pack ? CHANNEL##_on_SCALED_PRESSURE2( channel , pack ), (Meta const*)NULL : &mscaled_pressure2_SCALED_PRESSURE2;\
            case 81:    return pack ? CHANNEL##_on_MISSION_ITEM_REACHED( channel , pack ), (Meta const*)NULL : &mmission_item_reached_MISSION_ITEM_REACHED;\
            case 111:    return pack ? CHANNEL##_on_BATTERY_STATUS( channel , pack ), (Meta const*)NULL : &mbattery_status_BATTERY_STATUS;\
            case 160:    return pack ? CHANNEL##_on_WIND_COV( channel , pack ), (Meta const*)NULL : &mwind_cov_WIND_COV;\
            case 85:    return pack ? CHANNEL##_on_MISSION_ACK( channel , pack ), (Meta const*)NULL : &mmission_ack_MISSION_ACK;\
            case 180:    return pack ? CHANNEL##_on_LOCAL_POSITION_NED_COV( channel , pack ), (Meta const*)NULL : &mlocal_position_ned_cov_LOCAL_POSITION_NED_COV;\
            case 29:    return pack ? CHANNEL##_on_PARAM_MAP_RC( channel , pack ), (Meta const*)NULL : &mparam_map_rc_PARAM_MAP_RC;\
            case 207:    return pack ? CHANNEL##_on_CONTROL_SYSTEM_STATE( channel , pack ), (Meta const*)NULL : &mcontrol_system_state_CONTROL_SYSTEM_STATE;\
            case 94:    return pack ? CHANNEL##_on_SET_ACTUATOR_CONTROL_TARGET( channel , pack ), (Meta const*)NULL : &mset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET;\
            case 132:    return pack ? CHANNEL##_on_SYS_STATUS( channel , pack ), (Meta const*)NULL : &msys_status_SYS_STATUS;\
            case 46:    return pack ? CHANNEL##_on_TIMESYNC( channel , pack ), (Meta const*)NULL : &mtimesync_TIMESYNC;\
            case 59:    return pack ? CHANNEL##_on_MISSION_ITEM_INT( channel , pack ), (Meta const*)NULL : &mmission_item_int_MISSION_ITEM_INT;\
            case 213:    return pack ? CHANNEL##_on_GPS2_RAW( channel , pack ), (Meta const*)NULL : &mgps2_raw_GPS2_RAW;\
            case 24:    return pack ? CHANNEL##_on_SAFETY_SET_ALLOWED_AREA( channel , pack ), (Meta const*)NULL : &msafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA;\
            case 88:    return pack ? CHANNEL##_on_GPS_RTCM_DATA( channel , pack ), (Meta const*)NULL : &mgps_rtcm_data_GPS_RTCM_DATA;\
            case 8:    return pack ? CHANNEL##_on_NAMED_VALUE_FLOAT( channel , pack ), (Meta const*)NULL : &mnamed_value_float_NAMED_VALUE_FLOAT;\
            case 162:    return pack ? CHANNEL##_on_GLOBAL_POSITION_INT( channel , pack ), (Meta const*)NULL : &mglobal_position_int_GLOBAL_POSITION_INT;\
            case 52:    return pack ? CHANNEL##_on_GPS_INJECT_DATA( channel , pack ), (Meta const*)NULL : &mgps_inject_data_GPS_INJECT_DATA;\
            case 74:    return pack ? CHANNEL##_on_SET_ATTITUDE_TARGET( channel , pack ), (Meta const*)NULL : &mset_attitude_target_SET_ATTITUDE_TARGET;\
            case 58:    return pack ? CHANNEL##_on_STORAGE_INFORMATION( channel , pack ), (Meta const*)NULL : &mstorage_information_STORAGE_INFORMATION;\
            case 60:    return pack ? CHANNEL##_on_AUTOPILOT_VERSION( channel , pack ), (Meta const*)NULL : &mautopilot_version_AUTOPILOT_VERSION;\
            case 201:    return pack ? CHANNEL##_on_NAV_CONTROLLER_OUTPUT( channel , pack ), (Meta const*)NULL : &mnav_controller_output_NAV_CONTROLLER_OUTPUT;\
            case 45:    return pack ? CHANNEL##_on_COLLISION( channel , pack ), (Meta const*)NULL : &mcollision_COLLISION;\
            case 3:    return pack ? CHANNEL##_on_ACTUATOR_CONTROL_TARGET( channel , pack ), (Meta const*)NULL : &mactuator_control_target_ACTUATOR_CONTROL_TARGET;\
            case 219:    return pack ? CHANNEL##_on_MANUAL_SETPOINT( channel , pack ), (Meta const*)NULL : &mmanual_setpoint_MANUAL_SETPOINT;\
            case 149:    return pack ? CHANNEL##_on_DATA_TRANSMISSION_HANDSHAKE( channel , pack ), (Meta const*)NULL : &mdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE;\
            case 106:    return pack ? CHANNEL##_on_ATTITUDE_TARGET( channel , pack ), (Meta const*)NULL : &mattitude_target_ATTITUDE_TARGET;\
            case 7:    return pack ? CHANNEL##_on_SET_MODE( channel , pack ), (Meta const*)NULL : &mset_mode_SET_MODE;\
            case 148:    return pack ? CHANNEL##_on_RC_CHANNELS_RAW( channel , pack ), (Meta const*)NULL : &mrc_channels_raw_RC_CHANNELS_RAW;\
            case 163:    return pack ? CHANNEL##_on_RAW_IMU( channel , pack ), (Meta const*)NULL : &mraw_imu_RAW_IMU;\
            case 18:    return pack ? CHANNEL##_on_SCALED_PRESSURE( channel , pack ), (Meta const*)NULL : &mscaled_pressure_SCALED_PRESSURE;\
            case 110:    return pack ? CHANNEL##_on_GLOBAL_VISION_POSITION_ESTIMATE( channel , pack ), (Meta const*)NULL : &mglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE;\
            case 61:    return pack ? CHANNEL##_on_DISTANCE_SENSOR( channel , pack ), (Meta const*)NULL : &mdistance_sensor_DISTANCE_SENSOR;\
            case 78:    return pack ? CHANNEL##_on_ADSB_VEHICLE( channel , pack ), (Meta const*)NULL : &madsb_vehicle_ADSB_VEHICLE;\
            case 189:    return pack ? CHANNEL##_on_MISSION_CURRENT( channel , pack ), (Meta const*)NULL : &mmission_current_MISSION_CURRENT;\
            case 131:    return pack ? CHANNEL##_on_LOCAL_POSITION_NED( channel , pack ), (Meta const*)NULL : &mlocal_position_ned_LOCAL_POSITION_NED;\
            case 126:    return pack ? CHANNEL##_on_DEBUG( channel , pack ), (Meta const*)NULL : &mdebug_DEBUG;\
            case 27:    return pack ? CHANNEL##_on_FILE_TRANSFER_PROTOCOL( channel , pack ), (Meta const*)NULL : &mfile_transfer_protocol_FILE_TRANSFER_PROTOCOL;\
            case 121:    return pack ? CHANNEL##_on_COMMAND_ACK( channel , pack ), (Meta const*)NULL : &mcommand_ack_COMMAND_ACK;\
            case 134:    return pack ? CHANNEL##_on_ATTITUDE( channel , pack ), (Meta const*)NULL : &mattitude_ATTITUDE;\
            case 206:    return pack ? CHANNEL##_on_MISSION_REQUEST_PARTIAL_LIST( channel , pack ), (Meta const*)NULL : &mmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST;\
            case 172:    return pack ? CHANNEL##_on_LANDING_TARGET( channel , pack ), (Meta const*)NULL : &mlanding_target_LANDING_TARGET;\
            case 203:    return pack ? CHANNEL##_on_PLAY_TUNE( channel , pack ), (Meta const*)NULL : &mplay_tune_PLAY_TUNE;\
            case 182:    return pack ? CHANNEL##_on_ATTITUDE_QUATERNION_COV( channel , pack ), (Meta const*)NULL : &mattitude_quaternion_cov_ATTITUDE_QUATERNION_COV;\
            case 147:    return pack ? CHANNEL##_on_MISSION_REQUEST( channel , pack ), (Meta const*)NULL : &mmission_request_MISSION_REQUEST;\
            case 30:    return pack ? CHANNEL##_on_MEMORY_VECT( channel , pack ), (Meta const*)NULL : &mmemory_vect_MEMORY_VECT;\
            case 6:    return pack ? CHANNEL##_on_GLOBAL_POSITION_INT_COV( channel , pack ), (Meta const*)NULL : &mglobal_position_int_cov_GLOBAL_POSITION_INT_COV;\
            case 156:    return pack ? CHANNEL##_on_POWER_STATUS( channel , pack ), (Meta const*)NULL : &mpower_status_POWER_STATUS;\
            case 10:    return pack ? CHANNEL##_on_CAMERA_IMAGE_CAPTURED( channel , pack ), (Meta const*)NULL : &mcamera_image_captured_CAMERA_IMAGE_CAPTURED;\
            case 200:    return pack ? CHANNEL##_on_ATT_POS_MOCAP( channel , pack ), (Meta const*)NULL : &matt_pos_mocap_ATT_POS_MOCAP;\
            case 38:    return pack ? CHANNEL##_on_ESTIMATOR_STATUS( channel , pack ), (Meta const*)NULL : &mestimator_status_ESTIMATOR_STATUS;\
            case 193:    return pack ? CHANNEL##_on_MISSION_CLEAR_ALL( channel , pack ), (Meta const*)NULL : &mmission_clear_all_MISSION_CLEAR_ALL;\
            case 53:    return pack ? CHANNEL##_on_SET_GPS_GLOBAL_ORIGIN( channel , pack ), (Meta const*)NULL : &mset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN;\
            case 72:    return pack ? CHANNEL##_on_LOG_REQUEST_DATA( channel , pack ), (Meta const*)NULL : &mlog_request_data_LOG_REQUEST_DATA;\
            case 173:    return pack ? CHANNEL##_on_TERRAIN_DATA( channel , pack ), (Meta const*)NULL : &mterrain_data_TERRAIN_DATA;\
            case 135:    return pack ? CHANNEL##_on_VICON_POSITION_ESTIMATE( channel , pack ), (Meta const*)NULL : &mvicon_position_estimate_VICON_POSITION_ESTIMATE;\
            case 157:    return pack ? CHANNEL##_on_CAMERA_INFORMATION( channel , pack ), (Meta const*)NULL : &mcamera_information_CAMERA_INFORMATION;\
            case 56:    return pack ? CHANNEL##_on_GPS_RAW_INT( channel , pack ), (Meta const*)NULL : &mgps_raw_int_GPS_RAW_INT;\
            case 101:    return pack ? CHANNEL##_on_COMMAND_LONG( channel , pack ), (Meta const*)NULL : &mcommand_long_COMMAND_LONG;\
            case 62:    return pack ? CHANNEL##_on_CHANGE_OPERATOR_CONTROL_ACK( channel , pack ), (Meta const*)NULL : &mchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK;\
            case 170:    return pack ? CHANNEL##_on_ENCAPSULATED_DATA( channel , pack ), (Meta const*)NULL : &mencapsulated_data_ENCAPSULATED_DATA;\
            case 146:    return pack ? CHANNEL##_on_BUTTON_CHANGE( channel , pack ), (Meta const*)NULL : &mbutton_change_BUTTON_CHANGE;\
            case 187:    return pack ? CHANNEL##_on_DEBUG_VECT( channel , pack ), (Meta const*)NULL : &mdebug_vect_DEBUG_VECT;\
            case 82:    return pack ? CHANNEL##_on_HIL_CONTROLS( channel , pack ), (Meta const*)NULL : &mhil_controls_HIL_CONTROLS;\
            case 205:    return pack ? CHANNEL##_on_SETUP_SIGNING( channel , pack ), (Meta const*)NULL : &msetup_signing_SETUP_SIGNING;\
            case 71:    return pack ? CHANNEL##_on_ATTITUDE_QUATERNION( channel , pack ), (Meta const*)NULL : &mattitude_quaternion_ATTITUDE_QUATERNION;\
            case 23:    return pack ? CHANNEL##_on_RC_CHANNELS( channel , pack ), (Meta const*)NULL : &mrc_channels_RC_CHANNELS;\
            case 95:    return pack ? CHANNEL##_on_AUTH_KEY( channel , pack ), (Meta const*)NULL : &mauth_key_AUTH_KEY;\
            case 102:    return pack ? CHANNEL##_on_LOG_REQUEST_END( channel , pack ), (Meta const*)NULL : &mlog_request_end_LOG_REQUEST_END;\
            case 47:    return pack ? CHANNEL##_on_PARAM_REQUEST_LIST( channel , pack ), (Meta const*)NULL : &mparam_request_list_PARAM_REQUEST_LIST;\
            case 108:    return pack ? CHANNEL##_on_SIM_STATE( channel , pack ), (Meta const*)NULL : &msim_state_SIM_STATE;\
            case 122:    return pack ? CHANNEL##_on_PING( channel , pack ), (Meta const*)NULL : &mping_PING;\
            case 73:    return pack ? CHANNEL##_on_GPS_INPUT( channel , pack ), (Meta const*)NULL : &mgps_input_GPS_INPUT;\
            case 130:    return pack ? CHANNEL##_on_SCALED_PRESSURE3( channel , pack ), (Meta const*)NULL : &mscaled_pressure3_SCALED_PRESSURE3;\
            case 48:    return pack ? CHANNEL##_on_GPS_STATUS( channel , pack ), (Meta const*)NULL : &mgps_status_GPS_STATUS;\
            case 125:    return pack ? CHANNEL##_on_HEARTBEAT( channel , pack ), (Meta const*)NULL : &mheartbeat_HEARTBEAT;\
            case 9:    return pack ? CHANNEL##_on_SYSTEM_TIME( channel , pack ), (Meta const*)NULL : &msystem_time_SYSTEM_TIME;\
            case 4:    return pack ? CHANNEL##_on_LOG_ERASE( channel , pack ), (Meta const*)NULL : &mlog_erase_LOG_ERASE;\
            case 212:    return pack ? CHANNEL##_on_MISSION_SET_CURRENT( channel , pack ), (Meta const*)NULL : &mmission_set_current_MISSION_SET_CURRENT;\
            case 196:    return pack ? CHANNEL##_on_HIL_STATE_QUATERNION( channel , pack ), (Meta const*)NULL : &mhil_state_quaternion_HIL_STATE_QUATERNION;\
            case 92:    return pack ? CHANNEL##_on_TERRAIN_REQUEST( channel , pack ), (Meta const*)NULL : &mterrain_request_TERRAIN_REQUEST;\
            case 152:    return pack ? CHANNEL##_on_TERRAIN_CHECK( channel , pack ), (Meta const*)NULL : &mterrain_check_TERRAIN_CHECK;\
            case 145:    return pack ? CHANNEL##_on_MISSION_WRITE_PARTIAL_LIST( channel , pack ), (Meta const*)NULL : &mmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST;\
            case 199:    return pack ? CHANNEL##_on_HIL_ACTUATOR_CONTROLS( channel , pack ), (Meta const*)NULL : &mhil_actuator_controls_HIL_ACTUATOR_CONTROLS;\
            case 204:    return pack ? CHANNEL##_on_LOG_DATA( channel , pack ), (Meta const*)NULL : &mlog_data_LOG_DATA;\
            case 218:    return pack ? CHANNEL##_on_SET_POSITION_TARGET_GLOBAL_INT( channel , pack ), (Meta const*)NULL : &mset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT;\
            case 167:    return pack ? CHANNEL##_on_COMMAND_INT( channel , pack ), (Meta const*)NULL : &mcommand_int_COMMAND_INT;\
            case 179:    return pack ? CHANNEL##_on_SAFETY_ALLOWED_AREA( channel , pack ), (Meta const*)NULL : &msafety_allowed_area_SAFETY_ALLOWED_AREA;\
            case 120:    return pack ? CHANNEL##_on_RC_CHANNELS_SCALED( channel , pack ), (Meta const*)NULL : &mrc_channels_scaled_RC_CHANNELS_SCALED;\
            case 91:    return pack ? CHANNEL##_on_GPS2_RTK( channel , pack ), (Meta const*)NULL : &mgps2_rtk_GPS2_RTK;\
            case 16:    return pack ? CHANNEL##_on_VISION_POSITION_ESTIMATE( channel , pack ), (Meta const*)NULL : &mvision_position_estimate_VISION_POSITION_ESTIMATE;\
            case 54:    return pack ? CHANNEL##_on_MANUAL_CONTROL( channel , pack ), (Meta const*)NULL : &mmanual_control_MANUAL_CONTROL;\
            case 166:    return pack ? CHANNEL##_on_PARAM_REQUEST_READ( channel , pack ), (Meta const*)NULL : &mparam_request_read_PARAM_REQUEST_READ;\
            case 136:    return pack ? CHANNEL##_on_PARAM_VALUE( channel , pack ), (Meta const*)NULL : &mparam_value_PARAM_VALUE;\
        }

static inline void c_CommunicationChannel_bytes_into_packs(uint8_t *src, size_t src_bytes, c_CommunicationChannel *dst) { receive(src, src_bytes, &dst->receiver); }


static inline size_t c_CommunicationChannel_packs_into_bytes(c_CommunicationChannel *src, uint8_t *dst, size_t dst_bytes) { return transmit(&src->transmitter, dst, dst_bytes); }

extern bool c_CommunicationChannel_send(c_CommunicationChannel *dst, const Pack *pack);


/**
				 * send FOLLOW_TARGET
				 * param FOLLOW_TARGET_follow_target * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_FOLLOW_TARGET(c_CommunicationChannel, FOLLOW_TARGET_follow_target) ( c_CommunicationChannel_send ( c_CommunicationChannel , FOLLOW_TARGET_follow_target)? ( (FOLLOW_TARGET_follow_target) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline FOLLOW_TARGET_follow_target *c_CommunicationChannel_new_FOLLOW_TARGET() { return new_pack(&mfollow_target_FOLLOW_TARGET); }

/**
					* send ADSB_VEHICLE
					* param padsb_vehicle_ADSB_VEHICLE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_ADSB_VEHICLE(c_CommunicationChannel, padsb_vehicle_ADSB_VEHICLE) (c_CommunicationChannel_send ( c_CommunicationChannel , (padsb_vehicle_ADSB_VEHICLE)->base.pack)? ( (padsb_vehicle_ADSB_VEHICLE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline padsb_vehicle_ADSB_VEHICLE *c_CommunicationChannel_new_ADSB_VEHICLE(Cursor dst[]) { return wrap_pack(new_pack(&madsb_vehicle_ADSB_VEHICLE), dst); }

/**
				 * send MESSAGE_INTERVAL
				 * param MESSAGE_INTERVAL_message_interval * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_MESSAGE_INTERVAL(c_CommunicationChannel, MESSAGE_INTERVAL_message_interval) ( c_CommunicationChannel_send ( c_CommunicationChannel , MESSAGE_INTERVAL_message_interval)? ( (MESSAGE_INTERVAL_message_interval) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline MESSAGE_INTERVAL_message_interval *c_CommunicationChannel_new_MESSAGE_INTERVAL() { return new_pack(&mmessage_interval_MESSAGE_INTERVAL); }

/**
					* send EKF_STATUS_REPORT
					* param pekf_status_report_EKF_STATUS_REPORT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_EKF_STATUS_REPORT(c_CommunicationChannel, pekf_status_report_EKF_STATUS_REPORT) (c_CommunicationChannel_send ( c_CommunicationChannel , (pekf_status_report_EKF_STATUS_REPORT)->base.pack)? ( (pekf_status_report_EKF_STATUS_REPORT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pekf_status_report_EKF_STATUS_REPORT *c_CommunicationChannel_new_EKF_STATUS_REPORT(Cursor dst[]) { return wrap_pack(new_pack(&mekf_status_report_EKF_STATUS_REPORT), dst); }

/**
					* send ESTIMATOR_STATUS
					* param pestimator_status_ESTIMATOR_STATUS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_ESTIMATOR_STATUS(c_CommunicationChannel, pestimator_status_ESTIMATOR_STATUS) (c_CommunicationChannel_send ( c_CommunicationChannel , (pestimator_status_ESTIMATOR_STATUS)->base.pack)? ( (pestimator_status_ESTIMATOR_STATUS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pestimator_status_ESTIMATOR_STATUS *c_CommunicationChannel_new_ESTIMATOR_STATUS(Cursor dst[]) { return wrap_pack(new_pack(&mestimator_status_ESTIMATOR_STATUS), dst); }

/**
				 * send HWSTATUS
				 * param HWSTATUS_hwstatus * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_HWSTATUS(c_CommunicationChannel, HWSTATUS_hwstatus) ( c_CommunicationChannel_send ( c_CommunicationChannel , HWSTATUS_hwstatus)? ( (HWSTATUS_hwstatus) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline HWSTATUS_hwstatus *c_CommunicationChannel_new_HWSTATUS() { return new_pack(&mhwstatus_HWSTATUS); }

/**
				 * send TIMESYNC
				 * param TIMESYNC_timesync * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_TIMESYNC(c_CommunicationChannel, TIMESYNC_timesync) ( c_CommunicationChannel_send ( c_CommunicationChannel , TIMESYNC_timesync)? ( (TIMESYNC_timesync) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline TIMESYNC_timesync *c_CommunicationChannel_new_TIMESYNC() { return new_pack(&mtimesync_TIMESYNC); }

/**
				 * send PARAM_EXT_REQUEST_LIST
				 * param PARAM_EXT_REQUEST_LIST_param_ext_request_list * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_PARAM_EXT_REQUEST_LIST(c_CommunicationChannel, PARAM_EXT_REQUEST_LIST_param_ext_request_list) ( c_CommunicationChannel_send ( c_CommunicationChannel , PARAM_EXT_REQUEST_LIST_param_ext_request_list)? ( (PARAM_EXT_REQUEST_LIST_param_ext_request_list) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline PARAM_EXT_REQUEST_LIST_param_ext_request_list *c_CommunicationChannel_new_PARAM_EXT_REQUEST_LIST() { return new_pack(&mparam_ext_request_list_PARAM_EXT_REQUEST_LIST); }

/**
				 * send BUTTON_CHANGE
				 * param BUTTON_CHANGE_button_change * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_BUTTON_CHANGE(c_CommunicationChannel, BUTTON_CHANGE_button_change) ( c_CommunicationChannel_send ( c_CommunicationChannel , BUTTON_CHANGE_button_change)? ( (BUTTON_CHANGE_button_change) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline BUTTON_CHANGE_button_change *c_CommunicationChannel_new_BUTTON_CHANGE() { return new_pack(&mbutton_change_BUTTON_CHANGE); }

/**
					* send UAVCAN_NODE_STATUS
					* param puavcan_node_status_UAVCAN_NODE_STATUS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_UAVCAN_NODE_STATUS(c_CommunicationChannel, puavcan_node_status_UAVCAN_NODE_STATUS) (c_CommunicationChannel_send ( c_CommunicationChannel , (puavcan_node_status_UAVCAN_NODE_STATUS)->base.pack)? ( (puavcan_node_status_UAVCAN_NODE_STATUS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline puavcan_node_status_UAVCAN_NODE_STATUS *c_CommunicationChannel_new_UAVCAN_NODE_STATUS(Cursor dst[]) { return wrap_pack(new_pack(&muavcan_node_status_UAVCAN_NODE_STATUS), dst); }

/**
					* send COLLISION
					* param pcollision_COLLISION the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_COLLISION(c_CommunicationChannel, pcollision_COLLISION) (c_CommunicationChannel_send ( c_CommunicationChannel , (pcollision_COLLISION)->base.pack)? ( (pcollision_COLLISION)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pcollision_COLLISION *c_CommunicationChannel_new_COLLISION(Cursor dst[]) { return wrap_pack(new_pack(&mcollision_COLLISION), dst); }

/**
				 * send GIMBAL_TORQUE_CMD_REPORT
				 * param GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_GIMBAL_TORQUE_CMD_REPORT(c_CommunicationChannel, GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report) ( c_CommunicationChannel_send ( c_CommunicationChannel , GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report)? ( (GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report *c_CommunicationChannel_new_GIMBAL_TORQUE_CMD_REPORT() { return new_pack(&mgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT); }

/**
				 * send ALTITUDE
				 * param ALTITUDE_altitude * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_ALTITUDE(c_CommunicationChannel, ALTITUDE_altitude) ( c_CommunicationChannel_send ( c_CommunicationChannel , ALTITUDE_altitude)? ( (ALTITUDE_altitude) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline ALTITUDE_altitude *c_CommunicationChannel_new_ALTITUDE() { return new_pack(&maltitude_ALTITUDE); }

/**
				 * send HIL_STATE_QUATERNION
				 * param HIL_STATE_QUATERNION_hil_state_quaternion * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_HIL_STATE_QUATERNION(c_CommunicationChannel, HIL_STATE_QUATERNION_hil_state_quaternion) ( c_CommunicationChannel_send ( c_CommunicationChannel , HIL_STATE_QUATERNION_hil_state_quaternion)? ( (HIL_STATE_QUATERNION_hil_state_quaternion) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline HIL_STATE_QUATERNION_hil_state_quaternion *c_CommunicationChannel_new_HIL_STATE_QUATERNION() { return new_pack(&mhil_state_quaternion_HIL_STATE_QUATERNION); }

/**
				 * send SENSOR_OFFSETS
				 * param SENSOR_OFFSETS_sensor_offsets * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SENSOR_OFFSETS(c_CommunicationChannel, SENSOR_OFFSETS_sensor_offsets) ( c_CommunicationChannel_send ( c_CommunicationChannel , SENSOR_OFFSETS_sensor_offsets)? ( (SENSOR_OFFSETS_sensor_offsets) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SENSOR_OFFSETS_sensor_offsets *c_CommunicationChannel_new_SENSOR_OFFSETS() { return new_pack(&msensor_offsets_SENSOR_OFFSETS); }

/**
				 * send STORAGE_INFORMATION
				 * param STORAGE_INFORMATION_storage_information * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_STORAGE_INFORMATION(c_CommunicationChannel, STORAGE_INFORMATION_storage_information) ( c_CommunicationChannel_send ( c_CommunicationChannel , STORAGE_INFORMATION_storage_information)? ( (STORAGE_INFORMATION_storage_information) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline STORAGE_INFORMATION_storage_information *c_CommunicationChannel_new_STORAGE_INFORMATION() { return new_pack(&mstorage_information_STORAGE_INFORMATION); }

/**
					* send CAMERA_INFORMATION
					* param pcamera_information_CAMERA_INFORMATION the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_CAMERA_INFORMATION(c_CommunicationChannel, pcamera_information_CAMERA_INFORMATION) (c_CommunicationChannel_send ( c_CommunicationChannel , (pcamera_information_CAMERA_INFORMATION)->base.pack)? ( (pcamera_information_CAMERA_INFORMATION)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pcamera_information_CAMERA_INFORMATION *c_CommunicationChannel_new_CAMERA_INFORMATION(Cursor dst[]) { return wrap_pack(new_pack(&mcamera_information_CAMERA_INFORMATION), dst); }

/**
				 * send DEVICE_OP_WRITE_REPLY
				 * param DEVICE_OP_WRITE_REPLY_device_op_write_reply * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DEVICE_OP_WRITE_REPLY(c_CommunicationChannel, DEVICE_OP_WRITE_REPLY_device_op_write_reply) ( c_CommunicationChannel_send ( c_CommunicationChannel , DEVICE_OP_WRITE_REPLY_device_op_write_reply)? ( (DEVICE_OP_WRITE_REPLY_device_op_write_reply) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DEVICE_OP_WRITE_REPLY_device_op_write_reply *c_CommunicationChannel_new_DEVICE_OP_WRITE_REPLY() { return new_pack(&mdevice_op_write_reply_DEVICE_OP_WRITE_REPLY); }

/**
				 * send TERRAIN_DATA
				 * param TERRAIN_DATA_terrain_data * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_TERRAIN_DATA(c_CommunicationChannel, TERRAIN_DATA_terrain_data) ( c_CommunicationChannel_send ( c_CommunicationChannel , TERRAIN_DATA_terrain_data)? ( (TERRAIN_DATA_terrain_data) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline TERRAIN_DATA_terrain_data *c_CommunicationChannel_new_TERRAIN_DATA() { return new_pack(&mterrain_data_TERRAIN_DATA); }

/**
				 * send GIMBAL_CONTROL
				 * param GIMBAL_CONTROL_gimbal_control * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_GIMBAL_CONTROL(c_CommunicationChannel, GIMBAL_CONTROL_gimbal_control) ( c_CommunicationChannel_send ( c_CommunicationChannel , GIMBAL_CONTROL_gimbal_control)? ( (GIMBAL_CONTROL_gimbal_control) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline GIMBAL_CONTROL_gimbal_control *c_CommunicationChannel_new_GIMBAL_CONTROL() { return new_pack(&mgimbal_control_GIMBAL_CONTROL); }

/**
					* send VIDEO_STREAM_INFORMATION
					* param pvideo_stream_information_VIDEO_STREAM_INFORMATION the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_VIDEO_STREAM_INFORMATION(c_CommunicationChannel, pvideo_stream_information_VIDEO_STREAM_INFORMATION) (c_CommunicationChannel_send ( c_CommunicationChannel , (pvideo_stream_information_VIDEO_STREAM_INFORMATION)->base.pack)? ( (pvideo_stream_information_VIDEO_STREAM_INFORMATION)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pvideo_stream_information_VIDEO_STREAM_INFORMATION *c_CommunicationChannel_new_VIDEO_STREAM_INFORMATION(Cursor dst[]) { return wrap_pack(new_pack(&mvideo_stream_information_VIDEO_STREAM_INFORMATION), dst); }

/**
				 * send AHRS
				 * param AHRS_ahrs * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_AHRS(c_CommunicationChannel, AHRS_ahrs) ( c_CommunicationChannel_send ( c_CommunicationChannel , AHRS_ahrs)? ( (AHRS_ahrs) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline AHRS_ahrs *c_CommunicationChannel_new_AHRS() { return new_pack(&mahrs_AHRS); }

/**
				 * send DEBUG
				 * param DEBUG_debug * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DEBUG(c_CommunicationChannel, DEBUG_debug) ( c_CommunicationChannel_send ( c_CommunicationChannel , DEBUG_debug)? ( (DEBUG_debug) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DEBUG_debug *c_CommunicationChannel_new_DEBUG() { return new_pack(&mdebug_DEBUG); }

/**
					* send CAMERA_IMAGE_CAPTURED
					* param pcamera_image_captured_CAMERA_IMAGE_CAPTURED the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_CAMERA_IMAGE_CAPTURED(c_CommunicationChannel, pcamera_image_captured_CAMERA_IMAGE_CAPTURED) (c_CommunicationChannel_send ( c_CommunicationChannel , (pcamera_image_captured_CAMERA_IMAGE_CAPTURED)->base.pack)? ( (pcamera_image_captured_CAMERA_IMAGE_CAPTURED)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pcamera_image_captured_CAMERA_IMAGE_CAPTURED *c_CommunicationChannel_new_CAMERA_IMAGE_CAPTURED(Cursor dst[]) { return wrap_pack(new_pack(&mcamera_image_captured_CAMERA_IMAGE_CAPTURED), dst); }

/**
				 * send LOG_ENTRY
				 * param LOG_ENTRY_log_entry * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOG_ENTRY(c_CommunicationChannel, LOG_ENTRY_log_entry) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOG_ENTRY_log_entry)? ( (LOG_ENTRY_log_entry) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOG_ENTRY_log_entry *c_CommunicationChannel_new_LOG_ENTRY() { return new_pack(&mlog_entry_LOG_ENTRY); }

/**
				 * send ACTUATOR_CONTROL_TARGET
				 * param ACTUATOR_CONTROL_TARGET_actuator_control_target * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_ACTUATOR_CONTROL_TARGET(c_CommunicationChannel, ACTUATOR_CONTROL_TARGET_actuator_control_target) ( c_CommunicationChannel_send ( c_CommunicationChannel , ACTUATOR_CONTROL_TARGET_actuator_control_target)? ( (ACTUATOR_CONTROL_TARGET_actuator_control_target) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline ACTUATOR_CONTROL_TARGET_actuator_control_target *c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET() { return new_pack(&mactuator_control_target_ACTUATOR_CONTROL_TARGET); }

/**
					* send HIGH_LATENCY
					* param phigh_latency_HIGH_LATENCY the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_HIGH_LATENCY(c_CommunicationChannel, phigh_latency_HIGH_LATENCY) (c_CommunicationChannel_send ( c_CommunicationChannel , (phigh_latency_HIGH_LATENCY)->base.pack)? ( (phigh_latency_HIGH_LATENCY)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline phigh_latency_HIGH_LATENCY *c_CommunicationChannel_new_HIGH_LATENCY(Cursor dst[]) { return wrap_pack(new_pack(&mhigh_latency_HIGH_LATENCY), dst); }

/**
					* send HOME_POSITION
					* param phome_position_HOME_POSITION the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_HOME_POSITION(c_CommunicationChannel, phome_position_HOME_POSITION) (c_CommunicationChannel_send ( c_CommunicationChannel , (phome_position_HOME_POSITION)->base.pack)? ( (phome_position_HOME_POSITION)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline phome_position_HOME_POSITION *c_CommunicationChannel_new_HOME_POSITION(Cursor dst[]) { return wrap_pack(new_pack(&mhome_position_HOME_POSITION), dst); }

/**
					* send FENCE_STATUS
					* param pfence_status_FENCE_STATUS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_FENCE_STATUS(c_CommunicationChannel, pfence_status_FENCE_STATUS) (c_CommunicationChannel_send ( c_CommunicationChannel , (pfence_status_FENCE_STATUS)->base.pack)? ( (pfence_status_FENCE_STATUS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pfence_status_FENCE_STATUS *c_CommunicationChannel_new_FENCE_STATUS(Cursor dst[]) { return wrap_pack(new_pack(&mfence_status_FENCE_STATUS), dst); }

/**
					* send REMOTE_LOG_BLOCK_STATUS
					* param premote_log_block_status_REMOTE_LOG_BLOCK_STATUS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_REMOTE_LOG_BLOCK_STATUS(c_CommunicationChannel, premote_log_block_status_REMOTE_LOG_BLOCK_STATUS) (c_CommunicationChannel_send ( c_CommunicationChannel , (premote_log_block_status_REMOTE_LOG_BLOCK_STATUS)->base.pack)? ( (premote_log_block_status_REMOTE_LOG_BLOCK_STATUS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *c_CommunicationChannel_new_REMOTE_LOG_BLOCK_STATUS(Cursor dst[]) { return wrap_pack(new_pack(&mremote_log_block_status_REMOTE_LOG_BLOCK_STATUS), dst); }

/**
					* send OBSTACLE_DISTANCE
					* param pobstacle_distance_OBSTACLE_DISTANCE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_OBSTACLE_DISTANCE(c_CommunicationChannel, pobstacle_distance_OBSTACLE_DISTANCE) (c_CommunicationChannel_send ( c_CommunicationChannel , (pobstacle_distance_OBSTACLE_DISTANCE)->base.pack)? ( (pobstacle_distance_OBSTACLE_DISTANCE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pobstacle_distance_OBSTACLE_DISTANCE *c_CommunicationChannel_new_OBSTACLE_DISTANCE(Cursor dst[]) { return wrap_pack(new_pack(&mobstacle_distance_OBSTACLE_DISTANCE), dst); }

/**
					* send GPS2_RAW
					* param pgps2_raw_GPS2_RAW the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_GPS2_RAW(c_CommunicationChannel, pgps2_raw_GPS2_RAW) (c_CommunicationChannel_send ( c_CommunicationChannel , (pgps2_raw_GPS2_RAW)->base.pack)? ( (pgps2_raw_GPS2_RAW)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pgps2_raw_GPS2_RAW *c_CommunicationChannel_new_GPS2_RAW(Cursor dst[]) { return wrap_pack(new_pack(&mgps2_raw_GPS2_RAW), dst); }

/**
				 * send MEMORY_VECT
				 * param MEMORY_VECT_memory_vect * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_MEMORY_VECT(c_CommunicationChannel, MEMORY_VECT_memory_vect) ( c_CommunicationChannel_send ( c_CommunicationChannel , MEMORY_VECT_memory_vect)? ( (MEMORY_VECT_memory_vect) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline MEMORY_VECT_memory_vect *c_CommunicationChannel_new_MEMORY_VECT() { return new_pack(&mmemory_vect_MEMORY_VECT); }

/**
					* send PARAM_EXT_REQUEST_READ
					* param pparam_ext_request_read_PARAM_EXT_REQUEST_READ the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_PARAM_EXT_REQUEST_READ(c_CommunicationChannel, pparam_ext_request_read_PARAM_EXT_REQUEST_READ) (c_CommunicationChannel_send ( c_CommunicationChannel , (pparam_ext_request_read_PARAM_EXT_REQUEST_READ)->base.pack)? ( (pparam_ext_request_read_PARAM_EXT_REQUEST_READ)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pparam_ext_request_read_PARAM_EXT_REQUEST_READ *c_CommunicationChannel_new_PARAM_EXT_REQUEST_READ(Cursor dst[]) { return wrap_pack(new_pack(&mparam_ext_request_read_PARAM_EXT_REQUEST_READ), dst); }

/**
				 * send HIL_SENSOR
				 * param HIL_SENSOR_hil_sensor * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_HIL_SENSOR(c_CommunicationChannel, HIL_SENSOR_hil_sensor) ( c_CommunicationChannel_send ( c_CommunicationChannel , HIL_SENSOR_hil_sensor)? ( (HIL_SENSOR_hil_sensor) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline HIL_SENSOR_hil_sensor *c_CommunicationChannel_new_HIL_SENSOR() { return new_pack(&mhil_sensor_HIL_SENSOR); }

/**
				 * send SETUP_SIGNING
				 * param SETUP_SIGNING_setup_signing * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SETUP_SIGNING(c_CommunicationChannel, SETUP_SIGNING_setup_signing) ( c_CommunicationChannel_send ( c_CommunicationChannel , SETUP_SIGNING_setup_signing)? ( (SETUP_SIGNING_setup_signing) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SETUP_SIGNING_setup_signing *c_CommunicationChannel_new_SETUP_SIGNING() { return new_pack(&msetup_signing_SETUP_SIGNING); }

/**
				 * send GPS_RTK
				 * param GPS_RTK_gps_rtk * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_GPS_RTK(c_CommunicationChannel, GPS_RTK_gps_rtk) ( c_CommunicationChannel_send ( c_CommunicationChannel , GPS_RTK_gps_rtk)? ( (GPS_RTK_gps_rtk) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline GPS_RTK_gps_rtk *c_CommunicationChannel_new_GPS_RTK() { return new_pack(&mgps_rtk_GPS_RTK); }

/**
					* send UAVIONIX_ADSB_OUT_CFG
					* param puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_UAVIONIX_ADSB_OUT_CFG(c_CommunicationChannel, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG) (c_CommunicationChannel_send ( c_CommunicationChannel , (puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG)->base.pack)? ( (puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *c_CommunicationChannel_new_UAVIONIX_ADSB_OUT_CFG(Cursor dst[]) { return wrap_pack(new_pack(&muavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG), dst); }

/**
					* send LANDING_TARGET
					* param planding_target_LANDING_TARGET the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_LANDING_TARGET(c_CommunicationChannel, planding_target_LANDING_TARGET) (c_CommunicationChannel_send ( c_CommunicationChannel , (planding_target_LANDING_TARGET)->base.pack)? ( (planding_target_LANDING_TARGET)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline planding_target_LANDING_TARGET *c_CommunicationChannel_new_LANDING_TARGET(Cursor dst[]) { return wrap_pack(new_pack(&mlanding_target_LANDING_TARGET), dst); }

/**
				 * send SET_ACTUATOR_CONTROL_TARGET
				 * param SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SET_ACTUATOR_CONTROL_TARGET(c_CommunicationChannel, SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target) ( c_CommunicationChannel_send ( c_CommunicationChannel , SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target)? ( (SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target *c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET() { return new_pack(&mset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET); }

/**
				 * send CONTROL_SYSTEM_STATE
				 * param CONTROL_SYSTEM_STATE_control_system_state * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_CONTROL_SYSTEM_STATE(c_CommunicationChannel, CONTROL_SYSTEM_STATE_control_system_state) ( c_CommunicationChannel_send ( c_CommunicationChannel , CONTROL_SYSTEM_STATE_control_system_state)? ( (CONTROL_SYSTEM_STATE_control_system_state) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline CONTROL_SYSTEM_STATE_control_system_state *c_CommunicationChannel_new_CONTROL_SYSTEM_STATE() { return new_pack(&mcontrol_system_state_CONTROL_SYSTEM_STATE); }

/**
				 * send DATA32
				 * param DATA32_data32 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DATA32(c_CommunicationChannel, DATA32_data32) ( c_CommunicationChannel_send ( c_CommunicationChannel , DATA32_data32)? ( (DATA32_data32) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DATA32_data32 *c_CommunicationChannel_new_DATA32() { return new_pack(&mdata32_DATA32); }

/**
					* send PING33
					* param pping33_PING33 the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_PING33(c_CommunicationChannel, pping33_PING33) (c_CommunicationChannel_send ( c_CommunicationChannel , (pping33_PING33)->base.pack)? ( (pping33_PING33)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pping33_PING33 *c_CommunicationChannel_new_PING33(Cursor dst[]) { return wrap_pack(new_pack(&mping33_PING33), dst); }

/**
					* send RALLY_POINT
					* param prally_point_RALLY_POINT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_RALLY_POINT(c_CommunicationChannel, prally_point_RALLY_POINT) (c_CommunicationChannel_send ( c_CommunicationChannel , (prally_point_RALLY_POINT)->base.pack)? ( (prally_point_RALLY_POINT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline prally_point_RALLY_POINT *c_CommunicationChannel_new_RALLY_POINT(Cursor dst[]) { return wrap_pack(new_pack(&mrally_point_RALLY_POINT), dst); }

/**
					* send ADAP_TUNING
					* param padap_tuning_ADAP_TUNING the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_ADAP_TUNING(c_CommunicationChannel, padap_tuning_ADAP_TUNING) (c_CommunicationChannel_send ( c_CommunicationChannel , (padap_tuning_ADAP_TUNING)->base.pack)? ( (padap_tuning_ADAP_TUNING)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline padap_tuning_ADAP_TUNING *c_CommunicationChannel_new_ADAP_TUNING(Cursor dst[]) { return wrap_pack(new_pack(&madap_tuning_ADAP_TUNING), dst); }

/**
				 * send VIBRATION
				 * param VIBRATION_vibration * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_VIBRATION(c_CommunicationChannel, VIBRATION_vibration) ( c_CommunicationChannel_send ( c_CommunicationChannel , VIBRATION_vibration)? ( (VIBRATION_vibration) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline VIBRATION_vibration *c_CommunicationChannel_new_VIBRATION() { return new_pack(&mvibration_VIBRATION); }

/**
					* send PARAM_EXT_VALUE
					* param pparam_ext_value_PARAM_EXT_VALUE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_PARAM_EXT_VALUE(c_CommunicationChannel, pparam_ext_value_PARAM_EXT_VALUE) (c_CommunicationChannel_send ( c_CommunicationChannel , (pparam_ext_value_PARAM_EXT_VALUE)->base.pack)? ( (pparam_ext_value_PARAM_EXT_VALUE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pparam_ext_value_PARAM_EXT_VALUE *c_CommunicationChannel_new_PARAM_EXT_VALUE(Cursor dst[]) { return wrap_pack(new_pack(&mparam_ext_value_PARAM_EXT_VALUE), dst); }

/**
				 * send BATTERY2
				 * param BATTERY2_battery2 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_BATTERY2(c_CommunicationChannel, BATTERY2_battery2) ( c_CommunicationChannel_send ( c_CommunicationChannel , BATTERY2_battery2)? ( (BATTERY2_battery2) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline BATTERY2_battery2 *c_CommunicationChannel_new_BATTERY2() { return new_pack(&mbattery2_BATTERY2); }

/**
					* send LIMITS_STATUS
					* param plimits_status_LIMITS_STATUS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_LIMITS_STATUS(c_CommunicationChannel, plimits_status_LIMITS_STATUS) (c_CommunicationChannel_send ( c_CommunicationChannel , (plimits_status_LIMITS_STATUS)->base.pack)? ( (plimits_status_LIMITS_STATUS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline plimits_status_LIMITS_STATUS *c_CommunicationChannel_new_LIMITS_STATUS(Cursor dst[]) { return wrap_pack(new_pack(&mlimits_status_LIMITS_STATUS), dst); }

/**
					* send CAMERA_FEEDBACK
					* param pcamera_feedback_CAMERA_FEEDBACK the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_CAMERA_FEEDBACK(c_CommunicationChannel, pcamera_feedback_CAMERA_FEEDBACK) (c_CommunicationChannel_send ( c_CommunicationChannel , (pcamera_feedback_CAMERA_FEEDBACK)->base.pack)? ( (pcamera_feedback_CAMERA_FEEDBACK)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pcamera_feedback_CAMERA_FEEDBACK *c_CommunicationChannel_new_CAMERA_FEEDBACK(Cursor dst[]) { return wrap_pack(new_pack(&mcamera_feedback_CAMERA_FEEDBACK), dst); }

/**
				 * send HIL_GPS
				 * param HIL_GPS_hil_gps * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_HIL_GPS(c_CommunicationChannel, HIL_GPS_hil_gps) ( c_CommunicationChannel_send ( c_CommunicationChannel , HIL_GPS_hil_gps)? ( (HIL_GPS_hil_gps) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline HIL_GPS_hil_gps *c_CommunicationChannel_new_HIL_GPS() { return new_pack(&mhil_gps_HIL_GPS); }

/**
				 * send FENCE_FETCH_POINT
				 * param FENCE_FETCH_POINT_fence_fetch_point * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_FENCE_FETCH_POINT(c_CommunicationChannel, FENCE_FETCH_POINT_fence_fetch_point) ( c_CommunicationChannel_send ( c_CommunicationChannel , FENCE_FETCH_POINT_fence_fetch_point)? ( (FENCE_FETCH_POINT_fence_fetch_point) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline FENCE_FETCH_POINT_fence_fetch_point *c_CommunicationChannel_new_FENCE_FETCH_POINT() { return new_pack(&mfence_fetch_point_FENCE_FETCH_POINT); }

/**
				 * send RADIO
				 * param RADIO_radio * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_RADIO(c_CommunicationChannel, RADIO_radio) ( c_CommunicationChannel_send ( c_CommunicationChannel , RADIO_radio)? ( (RADIO_radio) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline RADIO_radio *c_CommunicationChannel_new_RADIO() { return new_pack(&mradio_RADIO); }

/**
				 * send AIRSPEED_AUTOCAL
				 * param AIRSPEED_AUTOCAL_airspeed_autocal * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_AIRSPEED_AUTOCAL(c_CommunicationChannel, AIRSPEED_AUTOCAL_airspeed_autocal) ( c_CommunicationChannel_send ( c_CommunicationChannel , AIRSPEED_AUTOCAL_airspeed_autocal)? ( (AIRSPEED_AUTOCAL_airspeed_autocal) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline AIRSPEED_AUTOCAL_airspeed_autocal *c_CommunicationChannel_new_AIRSPEED_AUTOCAL() { return new_pack(&mairspeed_autocal_AIRSPEED_AUTOCAL); }

/**
				 * send ATT_POS_MOCAP
				 * param ATT_POS_MOCAP_att_pos_mocap * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_ATT_POS_MOCAP(c_CommunicationChannel, ATT_POS_MOCAP_att_pos_mocap) ( c_CommunicationChannel_send ( c_CommunicationChannel , ATT_POS_MOCAP_att_pos_mocap)? ( (ATT_POS_MOCAP_att_pos_mocap) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline ATT_POS_MOCAP_att_pos_mocap *c_CommunicationChannel_new_ATT_POS_MOCAP() { return new_pack(&matt_pos_mocap_ATT_POS_MOCAP); }

/**
					* send STATUSTEXT
					* param pstatustext_STATUSTEXT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_STATUSTEXT(c_CommunicationChannel, pstatustext_STATUSTEXT) (c_CommunicationChannel_send ( c_CommunicationChannel , (pstatustext_STATUSTEXT)->base.pack)? ( (pstatustext_STATUSTEXT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pstatustext_STATUSTEXT *c_CommunicationChannel_new_STATUSTEXT(Cursor dst[]) { return wrap_pack(new_pack(&mstatustext_STATUSTEXT), dst); }

/**
					* send GOPRO_GET_REQUEST
					* param pgopro_get_request_GOPRO_GET_REQUEST the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_GOPRO_GET_REQUEST(c_CommunicationChannel, pgopro_get_request_GOPRO_GET_REQUEST) (c_CommunicationChannel_send ( c_CommunicationChannel , (pgopro_get_request_GOPRO_GET_REQUEST)->base.pack)? ( (pgopro_get_request_GOPRO_GET_REQUEST)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pgopro_get_request_GOPRO_GET_REQUEST *c_CommunicationChannel_new_GOPRO_GET_REQUEST(Cursor dst[]) { return wrap_pack(new_pack(&mgopro_get_request_GOPRO_GET_REQUEST), dst); }

/**
				 * send CAMERA_CAPTURE_STATUS
				 * param CAMERA_CAPTURE_STATUS_camera_capture_status * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_CAMERA_CAPTURE_STATUS(c_CommunicationChannel, CAMERA_CAPTURE_STATUS_camera_capture_status) ( c_CommunicationChannel_send ( c_CommunicationChannel , CAMERA_CAPTURE_STATUS_camera_capture_status)? ( (CAMERA_CAPTURE_STATUS_camera_capture_status) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline CAMERA_CAPTURE_STATUS_camera_capture_status *c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS() { return new_pack(&mcamera_capture_status_CAMERA_CAPTURE_STATUS); }

/**
				 * send ENCAPSULATED_DATA
				 * param ENCAPSULATED_DATA_encapsulated_data * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_ENCAPSULATED_DATA(c_CommunicationChannel, ENCAPSULATED_DATA_encapsulated_data) ( c_CommunicationChannel_send ( c_CommunicationChannel , ENCAPSULATED_DATA_encapsulated_data)? ( (ENCAPSULATED_DATA_encapsulated_data) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline ENCAPSULATED_DATA_encapsulated_data *c_CommunicationChannel_new_ENCAPSULATED_DATA() { return new_pack(&mencapsulated_data_ENCAPSULATED_DATA); }

/**
					* send GPS_INPUT
					* param pgps_input_GPS_INPUT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_GPS_INPUT(c_CommunicationChannel, pgps_input_GPS_INPUT) (c_CommunicationChannel_send ( c_CommunicationChannel , (pgps_input_GPS_INPUT)->base.pack)? ( (pgps_input_GPS_INPUT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pgps_input_GPS_INPUT *c_CommunicationChannel_new_GPS_INPUT(Cursor dst[]) { return wrap_pack(new_pack(&mgps_input_GPS_INPUT), dst); }

/**
				 * send COMPASSMOT_STATUS
				 * param COMPASSMOT_STATUS_compassmot_status * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_COMPASSMOT_STATUS(c_CommunicationChannel, COMPASSMOT_STATUS_compassmot_status) ( c_CommunicationChannel_send ( c_CommunicationChannel , COMPASSMOT_STATUS_compassmot_status)? ( (COMPASSMOT_STATUS_compassmot_status) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline COMPASSMOT_STATUS_compassmot_status *c_CommunicationChannel_new_COMPASSMOT_STATUS() { return new_pack(&mcompassmot_status_COMPASSMOT_STATUS); }

/**
				 * send LOG_REQUEST_DATA
				 * param LOG_REQUEST_DATA_log_request_data * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOG_REQUEST_DATA(c_CommunicationChannel, LOG_REQUEST_DATA_log_request_data) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOG_REQUEST_DATA_log_request_data)? ( (LOG_REQUEST_DATA_log_request_data) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOG_REQUEST_DATA_log_request_data *c_CommunicationChannel_new_LOG_REQUEST_DATA() { return new_pack(&mlog_request_data_LOG_REQUEST_DATA); }

/**
					* send CAMERA_STATUS
					* param pcamera_status_CAMERA_STATUS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_CAMERA_STATUS(c_CommunicationChannel, pcamera_status_CAMERA_STATUS) (c_CommunicationChannel_send ( c_CommunicationChannel , (pcamera_status_CAMERA_STATUS)->base.pack)? ( (pcamera_status_CAMERA_STATUS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pcamera_status_CAMERA_STATUS *c_CommunicationChannel_new_CAMERA_STATUS(Cursor dst[]) { return wrap_pack(new_pack(&mcamera_status_CAMERA_STATUS), dst); }

/**
					* send CAMERA_SETTINGS
					* param pcamera_settings_CAMERA_SETTINGS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_CAMERA_SETTINGS(c_CommunicationChannel, pcamera_settings_CAMERA_SETTINGS) (c_CommunicationChannel_send ( c_CommunicationChannel , (pcamera_settings_CAMERA_SETTINGS)->base.pack)? ( (pcamera_settings_CAMERA_SETTINGS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pcamera_settings_CAMERA_SETTINGS *c_CommunicationChannel_new_CAMERA_SETTINGS(Cursor dst[]) { return wrap_pack(new_pack(&mcamera_settings_CAMERA_SETTINGS), dst); }

/**
				 * send DEVICE_OP_READ_REPLY
				 * param DEVICE_OP_READ_REPLY_device_op_read_reply * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DEVICE_OP_READ_REPLY(c_CommunicationChannel, DEVICE_OP_READ_REPLY_device_op_read_reply) ( c_CommunicationChannel_send ( c_CommunicationChannel , DEVICE_OP_READ_REPLY_device_op_read_reply)? ( (DEVICE_OP_READ_REPLY_device_op_read_reply) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DEVICE_OP_READ_REPLY_device_op_read_reply *c_CommunicationChannel_new_DEVICE_OP_READ_REPLY() { return new_pack(&mdevice_op_read_reply_DEVICE_OP_READ_REPLY); }

/**
				 * send DIGICAM_CONTROL
				 * param DIGICAM_CONTROL_digicam_control * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DIGICAM_CONTROL(c_CommunicationChannel, DIGICAM_CONTROL_digicam_control) ( c_CommunicationChannel_send ( c_CommunicationChannel , DIGICAM_CONTROL_digicam_control)? ( (DIGICAM_CONTROL_digicam_control) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DIGICAM_CONTROL_digicam_control *c_CommunicationChannel_new_DIGICAM_CONTROL() { return new_pack(&mdigicam_control_DIGICAM_CONTROL); }

/**
					* send NAMED_VALUE_FLOAT
					* param pnamed_value_float_NAMED_VALUE_FLOAT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_NAMED_VALUE_FLOAT(c_CommunicationChannel, pnamed_value_float_NAMED_VALUE_FLOAT) (c_CommunicationChannel_send ( c_CommunicationChannel , (pnamed_value_float_NAMED_VALUE_FLOAT)->base.pack)? ( (pnamed_value_float_NAMED_VALUE_FLOAT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pnamed_value_float_NAMED_VALUE_FLOAT *c_CommunicationChannel_new_NAMED_VALUE_FLOAT(Cursor dst[]) { return wrap_pack(new_pack(&mnamed_value_float_NAMED_VALUE_FLOAT), dst); }

/**
					* send GOPRO_HEARTBEAT
					* param pgopro_heartbeat_GOPRO_HEARTBEAT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_GOPRO_HEARTBEAT(c_CommunicationChannel, pgopro_heartbeat_GOPRO_HEARTBEAT) (c_CommunicationChannel_send ( c_CommunicationChannel , (pgopro_heartbeat_GOPRO_HEARTBEAT)->base.pack)? ( (pgopro_heartbeat_GOPRO_HEARTBEAT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pgopro_heartbeat_GOPRO_HEARTBEAT *c_CommunicationChannel_new_GOPRO_HEARTBEAT(Cursor dst[]) { return wrap_pack(new_pack(&mgopro_heartbeat_GOPRO_HEARTBEAT), dst); }

/**
				 * send AHRS2
				 * param AHRS2_ahrs2 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_AHRS2(c_CommunicationChannel, AHRS2_ahrs2) ( c_CommunicationChannel_send ( c_CommunicationChannel , AHRS2_ahrs2)? ( (AHRS2_ahrs2) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline AHRS2_ahrs2 *c_CommunicationChannel_new_AHRS2() { return new_pack(&mahrs2_AHRS2); }

/**
				 * send LOG_ERASE
				 * param LOG_ERASE_log_erase * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOG_ERASE(c_CommunicationChannel, LOG_ERASE_log_erase) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOG_ERASE_log_erase)? ( (LOG_ERASE_log_erase) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOG_ERASE_log_erase *c_CommunicationChannel_new_LOG_ERASE() { return new_pack(&mlog_erase_LOG_ERASE); }

/**
				 * send TERRAIN_REQUEST
				 * param TERRAIN_REQUEST_terrain_request * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_TERRAIN_REQUEST(c_CommunicationChannel, TERRAIN_REQUEST_terrain_request) ( c_CommunicationChannel_send ( c_CommunicationChannel , TERRAIN_REQUEST_terrain_request)? ( (TERRAIN_REQUEST_terrain_request) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline TERRAIN_REQUEST_terrain_request *c_CommunicationChannel_new_TERRAIN_REQUEST() { return new_pack(&mterrain_request_TERRAIN_REQUEST); }

/**
				 * send MOUNT_STATUS
				 * param MOUNT_STATUS_mount_status * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_MOUNT_STATUS(c_CommunicationChannel, MOUNT_STATUS_mount_status) ( c_CommunicationChannel_send ( c_CommunicationChannel , MOUNT_STATUS_mount_status)? ( (MOUNT_STATUS_mount_status) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline MOUNT_STATUS_mount_status *c_CommunicationChannel_new_MOUNT_STATUS() { return new_pack(&mmount_status_MOUNT_STATUS); }

/**
					* send PID_TUNING
					* param ppid_tuning_PID_TUNING the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_PID_TUNING(c_CommunicationChannel, ppid_tuning_PID_TUNING) (c_CommunicationChannel_send ( c_CommunicationChannel , (ppid_tuning_PID_TUNING)->base.pack)? ( (ppid_tuning_PID_TUNING)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline ppid_tuning_PID_TUNING *c_CommunicationChannel_new_PID_TUNING(Cursor dst[]) { return wrap_pack(new_pack(&mpid_tuning_PID_TUNING), dst); }

/**
				 * send OPTICAL_FLOW_RAD
				 * param OPTICAL_FLOW_RAD_optical_flow_rad * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_OPTICAL_FLOW_RAD(c_CommunicationChannel, OPTICAL_FLOW_RAD_optical_flow_rad) ( c_CommunicationChannel_send ( c_CommunicationChannel , OPTICAL_FLOW_RAD_optical_flow_rad)? ( (OPTICAL_FLOW_RAD_optical_flow_rad) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline OPTICAL_FLOW_RAD_optical_flow_rad *c_CommunicationChannel_new_OPTICAL_FLOW_RAD() { return new_pack(&moptical_flow_rad_OPTICAL_FLOW_RAD); }

/**
				 * send LOG_DATA
				 * param LOG_DATA_log_data * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOG_DATA(c_CommunicationChannel, LOG_DATA_log_data) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOG_DATA_log_data)? ( (LOG_DATA_log_data) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOG_DATA_log_data *c_CommunicationChannel_new_LOG_DATA() { return new_pack(&mlog_data_LOG_DATA); }

/**
				 * send AHRS3
				 * param AHRS3_ahrs3 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_AHRS3(c_CommunicationChannel, AHRS3_ahrs3) ( c_CommunicationChannel_send ( c_CommunicationChannel , AHRS3_ahrs3)? ( (AHRS3_ahrs3) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline AHRS3_ahrs3 *c_CommunicationChannel_new_AHRS3() { return new_pack(&mahrs3_AHRS3); }

/**
				 * send VICON_POSITION_ESTIMATE
				 * param VICON_POSITION_ESTIMATE_vicon_position_estimate * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_VICON_POSITION_ESTIMATE(c_CommunicationChannel, VICON_POSITION_ESTIMATE_vicon_position_estimate) ( c_CommunicationChannel_send ( c_CommunicationChannel , VICON_POSITION_ESTIMATE_vicon_position_estimate)? ( (VICON_POSITION_ESTIMATE_vicon_position_estimate) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline VICON_POSITION_ESTIMATE_vicon_position_estimate *c_CommunicationChannel_new_VICON_POSITION_ESTIMATE() { return new_pack(&mvicon_position_estimate_VICON_POSITION_ESTIMATE); }

/**
				 * send GPS2_RTK
				 * param GPS2_RTK_gps2_rtk * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_GPS2_RTK(c_CommunicationChannel, GPS2_RTK_gps2_rtk) ( c_CommunicationChannel_send ( c_CommunicationChannel , GPS2_RTK_gps2_rtk)? ( (GPS2_RTK_gps2_rtk) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline GPS2_RTK_gps2_rtk *c_CommunicationChannel_new_GPS2_RTK() { return new_pack(&mgps2_rtk_GPS2_RTK); }

/**
					* send MAG_CAL_REPORT
					* param pmag_cal_report_MAG_CAL_REPORT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_MAG_CAL_REPORT(c_CommunicationChannel, pmag_cal_report_MAG_CAL_REPORT) (c_CommunicationChannel_send ( c_CommunicationChannel , (pmag_cal_report_MAG_CAL_REPORT)->base.pack)? ( (pmag_cal_report_MAG_CAL_REPORT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pmag_cal_report_MAG_CAL_REPORT *c_CommunicationChannel_new_MAG_CAL_REPORT(Cursor dst[]) { return wrap_pack(new_pack(&mmag_cal_report_MAG_CAL_REPORT), dst); }

/**
				 * send LOG_REQUEST_LIST
				 * param LOG_REQUEST_LIST_log_request_list * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOG_REQUEST_LIST(c_CommunicationChannel, LOG_REQUEST_LIST_log_request_list) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOG_REQUEST_LIST_log_request_list)? ( (LOG_REQUEST_LIST_log_request_list) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOG_REQUEST_LIST_log_request_list *c_CommunicationChannel_new_LOG_REQUEST_LIST() { return new_pack(&mlog_request_list_LOG_REQUEST_LIST); }

/**
					* send MOUNT_CONFIGURE
					* param pmount_configure_MOUNT_CONFIGURE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_MOUNT_CONFIGURE(c_CommunicationChannel, pmount_configure_MOUNT_CONFIGURE) (c_CommunicationChannel_send ( c_CommunicationChannel , (pmount_configure_MOUNT_CONFIGURE)->base.pack)? ( (pmount_configure_MOUNT_CONFIGURE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pmount_configure_MOUNT_CONFIGURE *c_CommunicationChannel_new_MOUNT_CONFIGURE(Cursor dst[]) { return wrap_pack(new_pack(&mmount_configure_MOUNT_CONFIGURE), dst); }

/**
				 * send V2_EXTENSION
				 * param V2_EXTENSION_v2_extension * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_V2_EXTENSION(c_CommunicationChannel, V2_EXTENSION_v2_extension) ( c_CommunicationChannel_send ( c_CommunicationChannel , V2_EXTENSION_v2_extension)? ( (V2_EXTENSION_v2_extension) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline V2_EXTENSION_v2_extension *c_CommunicationChannel_new_V2_EXTENSION() { return new_pack(&mv2_extension_V2_EXTENSION); }

/**
					* send POWER_STATUS
					* param ppower_status_POWER_STATUS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_POWER_STATUS(c_CommunicationChannel, ppower_status_POWER_STATUS) (c_CommunicationChannel_send ( c_CommunicationChannel , (ppower_status_POWER_STATUS)->base.pack)? ( (ppower_status_POWER_STATUS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline ppower_status_POWER_STATUS *c_CommunicationChannel_new_POWER_STATUS(Cursor dst[]) { return wrap_pack(new_pack(&mpower_status_POWER_STATUS), dst); }

/**
					* send REMOTE_LOG_DATA_BLOCK
					* param premote_log_data_block_REMOTE_LOG_DATA_BLOCK the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_REMOTE_LOG_DATA_BLOCK(c_CommunicationChannel, premote_log_data_block_REMOTE_LOG_DATA_BLOCK) (c_CommunicationChannel_send ( c_CommunicationChannel , (premote_log_data_block_REMOTE_LOG_DATA_BLOCK)->base.pack)? ( (premote_log_data_block_REMOTE_LOG_DATA_BLOCK)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline premote_log_data_block_REMOTE_LOG_DATA_BLOCK *c_CommunicationChannel_new_REMOTE_LOG_DATA_BLOCK(Cursor dst[]) { return wrap_pack(new_pack(&mremote_log_data_block_REMOTE_LOG_DATA_BLOCK), dst); }

/**
				 * send LOGGING_DATA_ACKED
				 * param LOGGING_DATA_ACKED_logging_data_acked * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOGGING_DATA_ACKED(c_CommunicationChannel, LOGGING_DATA_ACKED_logging_data_acked) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOGGING_DATA_ACKED_logging_data_acked)? ( (LOGGING_DATA_ACKED_logging_data_acked) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOGGING_DATA_ACKED_logging_data_acked *c_CommunicationChannel_new_LOGGING_DATA_ACKED() { return new_pack(&mlogging_data_acked_LOGGING_DATA_ACKED); }

/**
				 * send TERRAIN_CHECK
				 * param TERRAIN_CHECK_terrain_check * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_TERRAIN_CHECK(c_CommunicationChannel, TERRAIN_CHECK_terrain_check) ( c_CommunicationChannel_send ( c_CommunicationChannel , TERRAIN_CHECK_terrain_check)? ( (TERRAIN_CHECK_terrain_check) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline TERRAIN_CHECK_terrain_check *c_CommunicationChannel_new_TERRAIN_CHECK() { return new_pack(&mterrain_check_TERRAIN_CHECK); }

/**
				 * send TERRAIN_REPORT
				 * param TERRAIN_REPORT_terrain_report * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_TERRAIN_REPORT(c_CommunicationChannel, TERRAIN_REPORT_terrain_report) ( c_CommunicationChannel_send ( c_CommunicationChannel , TERRAIN_REPORT_terrain_report)? ( (TERRAIN_REPORT_terrain_report) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline TERRAIN_REPORT_terrain_report *c_CommunicationChannel_new_TERRAIN_REPORT() { return new_pack(&mterrain_report_TERRAIN_REPORT); }

/**
					* send SET_HOME_POSITION
					* param pset_home_position_SET_HOME_POSITION the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_SET_HOME_POSITION(c_CommunicationChannel, pset_home_position_SET_HOME_POSITION) (c_CommunicationChannel_send ( c_CommunicationChannel , (pset_home_position_SET_HOME_POSITION)->base.pack)? ( (pset_home_position_SET_HOME_POSITION)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pset_home_position_SET_HOME_POSITION *c_CommunicationChannel_new_SET_HOME_POSITION(Cursor dst[]) { return wrap_pack(new_pack(&mset_home_position_SET_HOME_POSITION), dst); }

/**
				 * send SwitchModeCommand
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SwitchModeCommand(c_CommunicationChannel) ( c_CommunicationChannel_send ( c_CommunicationChannel , mswitchmodecommand_SwitchModeCommand.pack_alloc(NULL, 0) ) )

/**
				 * send SCALED_IMU3
				 * param SCALED_IMU3_scaled_imu3 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SCALED_IMU3(c_CommunicationChannel, SCALED_IMU3_scaled_imu3) ( c_CommunicationChannel_send ( c_CommunicationChannel , SCALED_IMU3_scaled_imu3)? ( (SCALED_IMU3_scaled_imu3) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SCALED_IMU3_scaled_imu3 *c_CommunicationChannel_new_SCALED_IMU3() { return new_pack(&mscaled_imu3_SCALED_IMU3); }

/**
				 * send MOUNT_CONTROL
				 * param MOUNT_CONTROL_mount_control * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_MOUNT_CONTROL(c_CommunicationChannel, MOUNT_CONTROL_mount_control) ( c_CommunicationChannel_send ( c_CommunicationChannel , MOUNT_CONTROL_mount_control)? ( (MOUNT_CONTROL_mount_control) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline MOUNT_CONTROL_mount_control *c_CommunicationChannel_new_MOUNT_CONTROL() { return new_pack(&mmount_control_MOUNT_CONTROL); }

/**
				 * send LED_CONTROL
				 * param LED_CONTROL_led_control * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LED_CONTROL(c_CommunicationChannel, LED_CONTROL_led_control) ( c_CommunicationChannel_send ( c_CommunicationChannel , LED_CONTROL_led_control)? ( (LED_CONTROL_led_control) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LED_CONTROL_led_control *c_CommunicationChannel_new_LED_CONTROL() { return new_pack(&mled_control_LED_CONTROL); }

/**
				 * send SIM_STATE
				 * param SIM_STATE_sim_state * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SIM_STATE(c_CommunicationChannel, SIM_STATE_sim_state) ( c_CommunicationChannel_send ( c_CommunicationChannel , SIM_STATE_sim_state)? ( (SIM_STATE_sim_state) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SIM_STATE_sim_state *c_CommunicationChannel_new_SIM_STATE() { return new_pack(&msim_state_SIM_STATE); }

/**
					* send WIFI_CONFIG_AP
					* param pwifi_config_ap_WIFI_CONFIG_AP the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_WIFI_CONFIG_AP(c_CommunicationChannel, pwifi_config_ap_WIFI_CONFIG_AP) (c_CommunicationChannel_send ( c_CommunicationChannel , (pwifi_config_ap_WIFI_CONFIG_AP)->base.pack)? ( (pwifi_config_ap_WIFI_CONFIG_AP)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pwifi_config_ap_WIFI_CONFIG_AP *c_CommunicationChannel_new_WIFI_CONFIG_AP(Cursor dst[]) { return wrap_pack(new_pack(&mwifi_config_ap_WIFI_CONFIG_AP), dst); }

/**
				 * send DATA96
				 * param DATA96_data96 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DATA96(c_CommunicationChannel, DATA96_data96) ( c_CommunicationChannel_send ( c_CommunicationChannel , DATA96_data96)? ( (DATA96_data96) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DATA96_data96 *c_CommunicationChannel_new_DATA96() { return new_pack(&mdata96_DATA96); }

/**
				 * send FLIGHT_INFORMATION
				 * param FLIGHT_INFORMATION_flight_information * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_FLIGHT_INFORMATION(c_CommunicationChannel, FLIGHT_INFORMATION_flight_information) ( c_CommunicationChannel_send ( c_CommunicationChannel , FLIGHT_INFORMATION_flight_information)? ( (FLIGHT_INFORMATION_flight_information) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline FLIGHT_INFORMATION_flight_information *c_CommunicationChannel_new_FLIGHT_INFORMATION() { return new_pack(&mflight_information_FLIGHT_INFORMATION); }

/**
					* send MEMINFO
					* param pmeminfo_MEMINFO the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_MEMINFO(c_CommunicationChannel, pmeminfo_MEMINFO) (c_CommunicationChannel_send ( c_CommunicationChannel , (pmeminfo_MEMINFO)->base.pack)? ( (pmeminfo_MEMINFO)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pmeminfo_MEMINFO *c_CommunicationChannel_new_MEMINFO(Cursor dst[]) { return wrap_pack(new_pack(&mmeminfo_MEMINFO), dst); }

/**
				 * send LOGGING_ACK
				 * param LOGGING_ACK_logging_ack * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOGGING_ACK(c_CommunicationChannel, LOGGING_ACK_logging_ack) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOGGING_ACK_logging_ack)? ( (LOGGING_ACK_logging_ack) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOGGING_ACK_logging_ack *c_CommunicationChannel_new_LOGGING_ACK() { return new_pack(&mlogging_ack_LOGGING_ACK); }

/**
				 * send VISION_SPEED_ESTIMATE
				 * param VISION_SPEED_ESTIMATE_vision_speed_estimate * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_VISION_SPEED_ESTIMATE(c_CommunicationChannel, VISION_SPEED_ESTIMATE_vision_speed_estimate) ( c_CommunicationChannel_send ( c_CommunicationChannel , VISION_SPEED_ESTIMATE_vision_speed_estimate)? ( (VISION_SPEED_ESTIMATE_vision_speed_estimate) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline VISION_SPEED_ESTIMATE_vision_speed_estimate *c_CommunicationChannel_new_VISION_SPEED_ESTIMATE() { return new_pack(&mvision_speed_estimate_VISION_SPEED_ESTIMATE); }

/**
					* send DEBUG_VECT
					* param pdebug_vect_DEBUG_VECT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_DEBUG_VECT(c_CommunicationChannel, pdebug_vect_DEBUG_VECT) (c_CommunicationChannel_send ( c_CommunicationChannel , (pdebug_vect_DEBUG_VECT)->base.pack)? ( (pdebug_vect_DEBUG_VECT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pdebug_vect_DEBUG_VECT *c_CommunicationChannel_new_DEBUG_VECT(Cursor dst[]) { return wrap_pack(new_pack(&mdebug_vect_DEBUG_VECT), dst); }

/**
				 * send CAMERA_TRIGGER
				 * param CAMERA_TRIGGER_camera_trigger * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_CAMERA_TRIGGER(c_CommunicationChannel, CAMERA_TRIGGER_camera_trigger) ( c_CommunicationChannel_send ( c_CommunicationChannel , CAMERA_TRIGGER_camera_trigger)? ( (CAMERA_TRIGGER_camera_trigger) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline CAMERA_TRIGGER_camera_trigger *c_CommunicationChannel_new_CAMERA_TRIGGER() { return new_pack(&mcamera_trigger_CAMERA_TRIGGER); }

/**
				 * send LOG_REQUEST_END
				 * param LOG_REQUEST_END_log_request_end * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOG_REQUEST_END(c_CommunicationChannel, LOG_REQUEST_END_log_request_end) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOG_REQUEST_END_log_request_end)? ( (LOG_REQUEST_END_log_request_end) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOG_REQUEST_END_log_request_end *c_CommunicationChannel_new_LOG_REQUEST_END() { return new_pack(&mlog_request_end_LOG_REQUEST_END); }

/**
					* send GOPRO_SET_RESPONSE
					* param pgopro_set_response_GOPRO_SET_RESPONSE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_GOPRO_SET_RESPONSE(c_CommunicationChannel, pgopro_set_response_GOPRO_SET_RESPONSE) (c_CommunicationChannel_send ( c_CommunicationChannel , (pgopro_set_response_GOPRO_SET_RESPONSE)->base.pack)? ( (pgopro_set_response_GOPRO_SET_RESPONSE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pgopro_set_response_GOPRO_SET_RESPONSE *c_CommunicationChannel_new_GOPRO_SET_RESPONSE(Cursor dst[]) { return wrap_pack(new_pack(&mgopro_set_response_GOPRO_SET_RESPONSE), dst); }

/**
				 * send PROTOCOL_VERSION
				 * param PROTOCOL_VERSION_protocol_version * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_PROTOCOL_VERSION(c_CommunicationChannel, PROTOCOL_VERSION_protocol_version) ( c_CommunicationChannel_send ( c_CommunicationChannel , PROTOCOL_VERSION_protocol_version)? ( (PROTOCOL_VERSION_protocol_version) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline PROTOCOL_VERSION_protocol_version *c_CommunicationChannel_new_PROTOCOL_VERSION() { return new_pack(&mprotocol_version_PROTOCOL_VERSION); }

/**
				 * send RALLY_FETCH_POINT
				 * param RALLY_FETCH_POINT_rally_fetch_point * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_RALLY_FETCH_POINT(c_CommunicationChannel, RALLY_FETCH_POINT_rally_fetch_point) ( c_CommunicationChannel_send ( c_CommunicationChannel , RALLY_FETCH_POINT_rally_fetch_point)? ( (RALLY_FETCH_POINT_rally_fetch_point) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline RALLY_FETCH_POINT_rally_fetch_point *c_CommunicationChannel_new_RALLY_FETCH_POINT() { return new_pack(&mrally_fetch_point_RALLY_FETCH_POINT); }

/**
					* send BATTERY_STATUS
					* param pbattery_status_BATTERY_STATUS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_BATTERY_STATUS(c_CommunicationChannel, pbattery_status_BATTERY_STATUS) (c_CommunicationChannel_send ( c_CommunicationChannel , (pbattery_status_BATTERY_STATUS)->base.pack)? ( (pbattery_status_BATTERY_STATUS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pbattery_status_BATTERY_STATUS *c_CommunicationChannel_new_BATTERY_STATUS(Cursor dst[]) { return wrap_pack(new_pack(&mbattery_status_BATTERY_STATUS), dst); }

/**
				 * send MOUNT_ORIENTATION
				 * param MOUNT_ORIENTATION_mount_orientation * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_MOUNT_ORIENTATION(c_CommunicationChannel, MOUNT_ORIENTATION_mount_orientation) ( c_CommunicationChannel_send ( c_CommunicationChannel , MOUNT_ORIENTATION_mount_orientation)? ( (MOUNT_ORIENTATION_mount_orientation) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline MOUNT_ORIENTATION_mount_orientation *c_CommunicationChannel_new_MOUNT_ORIENTATION() { return new_pack(&mmount_orientation_MOUNT_ORIENTATION); }

/**
					* send SERIAL_CONTROL
					* param pserial_control_SERIAL_CONTROL the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_SERIAL_CONTROL(c_CommunicationChannel, pserial_control_SERIAL_CONTROL) (c_CommunicationChannel_send ( c_CommunicationChannel , (pserial_control_SERIAL_CONTROL)->base.pack)? ( (pserial_control_SERIAL_CONTROL)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pserial_control_SERIAL_CONTROL *c_CommunicationChannel_new_SERIAL_CONTROL(Cursor dst[]) { return wrap_pack(new_pack(&mserial_control_SERIAL_CONTROL), dst); }

/**
					* send PARAM_EXT_SET
					* param pparam_ext_set_PARAM_EXT_SET the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_PARAM_EXT_SET(c_CommunicationChannel, pparam_ext_set_PARAM_EXT_SET) (c_CommunicationChannel_send ( c_CommunicationChannel , (pparam_ext_set_PARAM_EXT_SET)->base.pack)? ( (pparam_ext_set_PARAM_EXT_SET)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pparam_ext_set_PARAM_EXT_SET *c_CommunicationChannel_new_PARAM_EXT_SET(Cursor dst[]) { return wrap_pack(new_pack(&mparam_ext_set_PARAM_EXT_SET), dst); }

/**
					* send AUTOPILOT_VERSION
					* param pautopilot_version_AUTOPILOT_VERSION the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_AUTOPILOT_VERSION(c_CommunicationChannel, pautopilot_version_AUTOPILOT_VERSION) (c_CommunicationChannel_send ( c_CommunicationChannel , (pautopilot_version_AUTOPILOT_VERSION)->base.pack)? ( (pautopilot_version_AUTOPILOT_VERSION)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pautopilot_version_AUTOPILOT_VERSION *c_CommunicationChannel_new_AUTOPILOT_VERSION(Cursor dst[]) { return wrap_pack(new_pack(&mautopilot_version_AUTOPILOT_VERSION), dst); }

/**
				 * send SIMSTATE
				 * param SIMSTATE_simstate * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SIMSTATE(c_CommunicationChannel, SIMSTATE_simstate) ( c_CommunicationChannel_send ( c_CommunicationChannel , SIMSTATE_simstate)? ( (SIMSTATE_simstate) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SIMSTATE_simstate *c_CommunicationChannel_new_SIMSTATE() { return new_pack(&msimstate_SIMSTATE); }

/**
					* send SET_VIDEO_STREAM_SETTINGS
					* param pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_SET_VIDEO_STREAM_SETTINGS(c_CommunicationChannel, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS) (c_CommunicationChannel_send ( c_CommunicationChannel , (pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS)->base.pack)? ( (pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *c_CommunicationChannel_new_SET_VIDEO_STREAM_SETTINGS(Cursor dst[]) { return wrap_pack(new_pack(&mset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS), dst); }

/**
					* send PLAY_TUNE
					* param pplay_tune_PLAY_TUNE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_PLAY_TUNE(c_CommunicationChannel, pplay_tune_PLAY_TUNE) (c_CommunicationChannel_send ( c_CommunicationChannel , (pplay_tune_PLAY_TUNE)->base.pack)? ( (pplay_tune_PLAY_TUNE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pplay_tune_PLAY_TUNE *c_CommunicationChannel_new_PLAY_TUNE(Cursor dst[]) { return wrap_pack(new_pack(&mplay_tune_PLAY_TUNE), dst); }

/**
				 * send DIGICAM_CONFIGURE
				 * param DIGICAM_CONFIGURE_digicam_configure * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DIGICAM_CONFIGURE(c_CommunicationChannel, DIGICAM_CONFIGURE_digicam_configure) ( c_CommunicationChannel_send ( c_CommunicationChannel , DIGICAM_CONFIGURE_digicam_configure)? ( (DIGICAM_CONFIGURE_digicam_configure) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DIGICAM_CONFIGURE_digicam_configure *c_CommunicationChannel_new_DIGICAM_CONFIGURE() { return new_pack(&mdigicam_configure_DIGICAM_CONFIGURE); }

/**
				 * send SCALED_PRESSURE3
				 * param SCALED_PRESSURE3_scaled_pressure3 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SCALED_PRESSURE3(c_CommunicationChannel, SCALED_PRESSURE3_scaled_pressure3) ( c_CommunicationChannel_send ( c_CommunicationChannel , SCALED_PRESSURE3_scaled_pressure3)? ( (SCALED_PRESSURE3_scaled_pressure3) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SCALED_PRESSURE3_scaled_pressure3 *c_CommunicationChannel_new_SCALED_PRESSURE3() { return new_pack(&mscaled_pressure3_SCALED_PRESSURE3); }

/**
					* send PARAM_EXT_ACK
					* param pparam_ext_ack_PARAM_EXT_ACK the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_PARAM_EXT_ACK(c_CommunicationChannel, pparam_ext_ack_PARAM_EXT_ACK) (c_CommunicationChannel_send ( c_CommunicationChannel , (pparam_ext_ack_PARAM_EXT_ACK)->base.pack)? ( (pparam_ext_ack_PARAM_EXT_ACK)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pparam_ext_ack_PARAM_EXT_ACK *c_CommunicationChannel_new_PARAM_EXT_ACK(Cursor dst[]) { return wrap_pack(new_pack(&mparam_ext_ack_PARAM_EXT_ACK), dst); }

/**
					* send UAVCAN_NODE_INFO
					* param puavcan_node_info_UAVCAN_NODE_INFO the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_UAVCAN_NODE_INFO(c_CommunicationChannel, puavcan_node_info_UAVCAN_NODE_INFO) (c_CommunicationChannel_send ( c_CommunicationChannel , (puavcan_node_info_UAVCAN_NODE_INFO)->base.pack)? ( (puavcan_node_info_UAVCAN_NODE_INFO)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline puavcan_node_info_UAVCAN_NODE_INFO *c_CommunicationChannel_new_UAVCAN_NODE_INFO(Cursor dst[]) { return wrap_pack(new_pack(&muavcan_node_info_UAVCAN_NODE_INFO), dst); }

/**
				 * send DATA16
				 * param DATA16_data16 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DATA16(c_CommunicationChannel, DATA16_data16) ( c_CommunicationChannel_send ( c_CommunicationChannel , DATA16_data16)? ( (DATA16_data16) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DATA16_data16 *c_CommunicationChannel_new_DATA16() { return new_pack(&mdata16_DATA16); }

/**
				 * send SET_MAG_OFFSETS
				 * param SET_MAG_OFFSETS_set_mag_offsets * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SET_MAG_OFFSETS(c_CommunicationChannel, SET_MAG_OFFSETS_set_mag_offsets) ( c_CommunicationChannel_send ( c_CommunicationChannel , SET_MAG_OFFSETS_set_mag_offsets)? ( (SET_MAG_OFFSETS_set_mag_offsets) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SET_MAG_OFFSETS_set_mag_offsets *c_CommunicationChannel_new_SET_MAG_OFFSETS() { return new_pack(&mset_mag_offsets_SET_MAG_OFFSETS); }

/**
				 * send SCALED_IMU2
				 * param SCALED_IMU2_scaled_imu2 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SCALED_IMU2(c_CommunicationChannel, SCALED_IMU2_scaled_imu2) ( c_CommunicationChannel_send ( c_CommunicationChannel , SCALED_IMU2_scaled_imu2)? ( (SCALED_IMU2_scaled_imu2) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SCALED_IMU2_scaled_imu2 *c_CommunicationChannel_new_SCALED_IMU2() { return new_pack(&mscaled_imu2_SCALED_IMU2); }

/**
				 * send AP_ADC
				 * param AP_ADC_ap_adc * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_AP_ADC(c_CommunicationChannel, AP_ADC_ap_adc) ( c_CommunicationChannel_send ( c_CommunicationChannel , AP_ADC_ap_adc)? ( (AP_ADC_ap_adc) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline AP_ADC_ap_adc *c_CommunicationChannel_new_AP_ADC() { return new_pack(&map_adc_AP_ADC); }

/**
				 * send WIND
				 * param WIND_wind * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_WIND(c_CommunicationChannel, WIND_wind) ( c_CommunicationChannel_send ( c_CommunicationChannel , WIND_wind)? ( (WIND_wind) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline WIND_wind *c_CommunicationChannel_new_WIND() { return new_pack(&mwind_WIND); }

/**
				 * send AUTOPILOT_VERSION_REQUEST
				 * param AUTOPILOT_VERSION_REQUEST_autopilot_version_request * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_AUTOPILOT_VERSION_REQUEST(c_CommunicationChannel, AUTOPILOT_VERSION_REQUEST_autopilot_version_request) ( c_CommunicationChannel_send ( c_CommunicationChannel , AUTOPILOT_VERSION_REQUEST_autopilot_version_request)? ( (AUTOPILOT_VERSION_REQUEST_autopilot_version_request) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline AUTOPILOT_VERSION_REQUEST_autopilot_version_request *c_CommunicationChannel_new_AUTOPILOT_VERSION_REQUEST() { return new_pack(&mautopilot_version_request_AUTOPILOT_VERSION_REQUEST); }

/**
				 * send DATA_TRANSMISSION_HANDSHAKE
				 * param DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DATA_TRANSMISSION_HANDSHAKE(c_CommunicationChannel, DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake) ( c_CommunicationChannel_send ( c_CommunicationChannel , DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake)? ( (DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake *c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE() { return new_pack(&mdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE); }

/**
				 * send DATA64
				 * param DATA64_data64 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_DATA64(c_CommunicationChannel, DATA64_data64) ( c_CommunicationChannel_send ( c_CommunicationChannel , DATA64_data64)? ( (DATA64_data64) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline DATA64_data64 *c_CommunicationChannel_new_DATA64() { return new_pack(&mdata64_DATA64); }

/**
				 * send GIMBAL_REPORT
				 * param GIMBAL_REPORT_gimbal_report * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_GIMBAL_REPORT(c_CommunicationChannel, GIMBAL_REPORT_gimbal_report) ( c_CommunicationChannel_send ( c_CommunicationChannel , GIMBAL_REPORT_gimbal_report)? ( (GIMBAL_REPORT_gimbal_report) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline GIMBAL_REPORT_gimbal_report *c_CommunicationChannel_new_GIMBAL_REPORT() { return new_pack(&mgimbal_report_GIMBAL_REPORT); }

/**
					* send DEVICE_OP_WRITE
					* param pdevice_op_write_DEVICE_OP_WRITE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_DEVICE_OP_WRITE(c_CommunicationChannel, pdevice_op_write_DEVICE_OP_WRITE) (c_CommunicationChannel_send ( c_CommunicationChannel , (pdevice_op_write_DEVICE_OP_WRITE)->base.pack)? ( (pdevice_op_write_DEVICE_OP_WRITE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pdevice_op_write_DEVICE_OP_WRITE *c_CommunicationChannel_new_DEVICE_OP_WRITE(Cursor dst[]) { return wrap_pack(new_pack(&mdevice_op_write_DEVICE_OP_WRITE), dst); }

/**
					* send DISTANCE_SENSOR
					* param pdistance_sensor_DISTANCE_SENSOR the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_DISTANCE_SENSOR(c_CommunicationChannel, pdistance_sensor_DISTANCE_SENSOR) (c_CommunicationChannel_send ( c_CommunicationChannel , (pdistance_sensor_DISTANCE_SENSOR)->base.pack)? ( (pdistance_sensor_DISTANCE_SENSOR)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pdistance_sensor_DISTANCE_SENSOR *c_CommunicationChannel_new_DISTANCE_SENSOR(Cursor dst[]) { return wrap_pack(new_pack(&mdistance_sensor_DISTANCE_SENSOR), dst); }

/**
				 * send HIL_OPTICAL_FLOW
				 * param HIL_OPTICAL_FLOW_hil_optical_flow * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_HIL_OPTICAL_FLOW(c_CommunicationChannel, HIL_OPTICAL_FLOW_hil_optical_flow) ( c_CommunicationChannel_send ( c_CommunicationChannel , HIL_OPTICAL_FLOW_hil_optical_flow)? ( (HIL_OPTICAL_FLOW_hil_optical_flow) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline HIL_OPTICAL_FLOW_hil_optical_flow *c_CommunicationChannel_new_HIL_OPTICAL_FLOW() { return new_pack(&mhil_optical_flow_HIL_OPTICAL_FLOW); }

/**
				 * send SCALED_PRESSURE2
				 * param SCALED_PRESSURE2_scaled_pressure2 * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_SCALED_PRESSURE2(c_CommunicationChannel, SCALED_PRESSURE2_scaled_pressure2) ( c_CommunicationChannel_send ( c_CommunicationChannel , SCALED_PRESSURE2_scaled_pressure2)? ( (SCALED_PRESSURE2_scaled_pressure2) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline SCALED_PRESSURE2_scaled_pressure2 *c_CommunicationChannel_new_SCALED_PRESSURE2() { return new_pack(&mscaled_pressure2_SCALED_PRESSURE2); }

/**
				 * send WIND_COV
				 * param WIND_COV_wind_cov * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_WIND_COV(c_CommunicationChannel, WIND_COV_wind_cov) ( c_CommunicationChannel_send ( c_CommunicationChannel , WIND_COV_wind_cov)? ( (WIND_COV_wind_cov) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline WIND_COV_wind_cov *c_CommunicationChannel_new_WIND_COV() { return new_pack(&mwind_cov_WIND_COV); }

/**
					* send GOPRO_SET_REQUEST
					* param pgopro_set_request_GOPRO_SET_REQUEST the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_GOPRO_SET_REQUEST(c_CommunicationChannel, pgopro_set_request_GOPRO_SET_REQUEST) (c_CommunicationChannel_send ( c_CommunicationChannel , (pgopro_set_request_GOPRO_SET_REQUEST)->base.pack)? ( (pgopro_set_request_GOPRO_SET_REQUEST)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pgopro_set_request_GOPRO_SET_REQUEST *c_CommunicationChannel_new_GOPRO_SET_REQUEST(Cursor dst[]) { return wrap_pack(new_pack(&mgopro_set_request_GOPRO_SET_REQUEST), dst); }

/**
				 * send VISION_POSITION_DELTA
				 * param VISION_POSITION_DELTA_vision_position_delta * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_VISION_POSITION_DELTA(c_CommunicationChannel, VISION_POSITION_DELTA_vision_position_delta) ( c_CommunicationChannel_send ( c_CommunicationChannel , VISION_POSITION_DELTA_vision_position_delta)? ( (VISION_POSITION_DELTA_vision_position_delta) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline VISION_POSITION_DELTA_vision_position_delta *c_CommunicationChannel_new_VISION_POSITION_DELTA() { return new_pack(&mvision_position_delta_VISION_POSITION_DELTA); }

/**
				 * send LOGGING_DATA
				 * param LOGGING_DATA_logging_data * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_LOGGING_DATA(c_CommunicationChannel, LOGGING_DATA_logging_data) ( c_CommunicationChannel_send ( c_CommunicationChannel , LOGGING_DATA_logging_data)? ( (LOGGING_DATA_logging_data) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline LOGGING_DATA_logging_data *c_CommunicationChannel_new_LOGGING_DATA() { return new_pack(&mlogging_data_LOGGING_DATA); }

/**
					* send DEVICE_OP_READ
					* param pdevice_op_read_DEVICE_OP_READ the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_DEVICE_OP_READ(c_CommunicationChannel, pdevice_op_read_DEVICE_OP_READ) (c_CommunicationChannel_send ( c_CommunicationChannel , (pdevice_op_read_DEVICE_OP_READ)->base.pack)? ( (pdevice_op_read_DEVICE_OP_READ)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pdevice_op_read_DEVICE_OP_READ *c_CommunicationChannel_new_DEVICE_OP_READ(Cursor dst[]) { return wrap_pack(new_pack(&mdevice_op_read_DEVICE_OP_READ), dst); }

/**
					* send MAG_CAL_PROGRESS
					* param pmag_cal_progress_MAG_CAL_PROGRESS the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_MAG_CAL_PROGRESS(c_CommunicationChannel, pmag_cal_progress_MAG_CAL_PROGRESS) (c_CommunicationChannel_send ( c_CommunicationChannel , (pmag_cal_progress_MAG_CAL_PROGRESS)->base.pack)? ( (pmag_cal_progress_MAG_CAL_PROGRESS)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pmag_cal_progress_MAG_CAL_PROGRESS *c_CommunicationChannel_new_MAG_CAL_PROGRESS(Cursor dst[]) { return wrap_pack(new_pack(&mmag_cal_progress_MAG_CAL_PROGRESS), dst); }

/**
				 * send HIGHRES_IMU
				 * param HIGHRES_IMU_highres_imu * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_HIGHRES_IMU(c_CommunicationChannel, HIGHRES_IMU_highres_imu) ( c_CommunicationChannel_send ( c_CommunicationChannel , HIGHRES_IMU_highres_imu)? ( (HIGHRES_IMU_highres_imu) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline HIGHRES_IMU_highres_imu *c_CommunicationChannel_new_HIGHRES_IMU() { return new_pack(&mhighres_imu_HIGHRES_IMU); }

/**
					* send EXTENDED_SYS_STATE
					* param pextended_sys_state_EXTENDED_SYS_STATE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_EXTENDED_SYS_STATE(c_CommunicationChannel, pextended_sys_state_EXTENDED_SYS_STATE) (c_CommunicationChannel_send ( c_CommunicationChannel , (pextended_sys_state_EXTENDED_SYS_STATE)->base.pack)? ( (pextended_sys_state_EXTENDED_SYS_STATE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pextended_sys_state_EXTENDED_SYS_STATE *c_CommunicationChannel_new_EXTENDED_SYS_STATE(Cursor dst[]) { return wrap_pack(new_pack(&mextended_sys_state_EXTENDED_SYS_STATE), dst); }

/**
					* send UAVIONIX_ADSB_OUT_DYNAMIC
					* param puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_UAVIONIX_ADSB_OUT_DYNAMIC(c_CommunicationChannel, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC) (c_CommunicationChannel_send ( c_CommunicationChannel , (puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC)->base.pack)? ( (puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *c_CommunicationChannel_new_UAVIONIX_ADSB_OUT_DYNAMIC(Cursor dst[]) { return wrap_pack(new_pack(&muavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC), dst); }

/**
					* send GOPRO_GET_RESPONSE
					* param pgopro_get_response_GOPRO_GET_RESPONSE the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_GOPRO_GET_RESPONSE(c_CommunicationChannel, pgopro_get_response_GOPRO_GET_RESPONSE) (c_CommunicationChannel_send ( c_CommunicationChannel , (pgopro_get_response_GOPRO_GET_RESPONSE)->base.pack)? ( (pgopro_get_response_GOPRO_GET_RESPONSE)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pgopro_get_response_GOPRO_GET_RESPONSE *c_CommunicationChannel_new_GOPRO_GET_RESPONSE(Cursor dst[]) { return wrap_pack(new_pack(&mgopro_get_response_GOPRO_GET_RESPONSE), dst); }

/**
				 * send GPS_INJECT_DATA
				 * param GPS_INJECT_DATA_gps_inject_data * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_GPS_INJECT_DATA(c_CommunicationChannel, GPS_INJECT_DATA_gps_inject_data) ( c_CommunicationChannel_send ( c_CommunicationChannel , GPS_INJECT_DATA_gps_inject_data)? ( (GPS_INJECT_DATA_gps_inject_data) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline GPS_INJECT_DATA_gps_inject_data *c_CommunicationChannel_new_GPS_INJECT_DATA() { return new_pack(&mgps_inject_data_GPS_INJECT_DATA); }

/**
					* send UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT
					* param puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(c_CommunicationChannel, puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) (c_CommunicationChannel_send ( c_CommunicationChannel , (puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT)->base.pack)? ( (puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *c_CommunicationChannel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(Cursor dst[]) { return wrap_pack(new_pack(&muavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT), dst); }

/**
					* send NAMED_VALUE_INT
					* param pnamed_value_int_NAMED_VALUE_INT the pack data navidgator with pack that will be send
					* param c_CommunicationChannel * pointer to channel that contains transmitter
					* return true if add pack to the sending queue succeed
					*/
#define c_CommunicationChannel_send_NAMED_VALUE_INT(c_CommunicationChannel, pnamed_value_int_NAMED_VALUE_INT) (c_CommunicationChannel_send ( c_CommunicationChannel , (pnamed_value_int_NAMED_VALUE_INT)->base.pack)? ( (pnamed_value_int_NAMED_VALUE_INT)->base.pack = NULL), true : false)

/**
					 * brief Getting new wraped pack  that can be populated with data and send
					 * param dst wrapper
					 * return new wraped pack
					 */
static inline pnamed_value_int_NAMED_VALUE_INT *c_CommunicationChannel_new_NAMED_VALUE_INT(Cursor dst[]) { return wrap_pack(new_pack(&mnamed_value_int_NAMED_VALUE_INT), dst); }

/**
				 * send RPM
				 * param RPM_rpm * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_RPM(c_CommunicationChannel, RPM_rpm) ( c_CommunicationChannel_send ( c_CommunicationChannel , RPM_rpm)? ( (RPM_rpm) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline RPM_rpm *c_CommunicationChannel_new_RPM() { return new_pack(&mrpm_RPM); }

/**
				 * send GPS_RTCM_DATA
				 * param GPS_RTCM_DATA_gps_rtcm_data * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_GPS_RTCM_DATA(c_CommunicationChannel, GPS_RTCM_DATA_gps_rtcm_data) ( c_CommunicationChannel_send ( c_CommunicationChannel , GPS_RTCM_DATA_gps_rtcm_data)? ( (GPS_RTCM_DATA_gps_rtcm_data) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline GPS_RTCM_DATA_gps_rtcm_data *c_CommunicationChannel_new_GPS_RTCM_DATA() { return new_pack(&mgps_rtcm_data_GPS_RTCM_DATA); }

/**
				 * send FILE_TRANSFER_PROTOCOL
				 * param FILE_TRANSFER_PROTOCOL_file_transfer_protocol * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_FILE_TRANSFER_PROTOCOL(c_CommunicationChannel, FILE_TRANSFER_PROTOCOL_file_transfer_protocol) ( c_CommunicationChannel_send ( c_CommunicationChannel , FILE_TRANSFER_PROTOCOL_file_transfer_protocol)? ( (FILE_TRANSFER_PROTOCOL_file_transfer_protocol) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline FILE_TRANSFER_PROTOCOL_file_transfer_protocol *c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL() { return new_pack(&mfile_transfer_protocol_FILE_TRANSFER_PROTOCOL); }

/**
				 * send RANGEFINDER
				 * param RANGEFINDER_rangefinder * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_RANGEFINDER(c_CommunicationChannel, RANGEFINDER_rangefinder) ( c_CommunicationChannel_send ( c_CommunicationChannel , RANGEFINDER_rangefinder)? ( (RANGEFINDER_rangefinder) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline RANGEFINDER_rangefinder *c_CommunicationChannel_new_RANGEFINDER() { return new_pack(&mrangefinder_RANGEFINDER); }

/**
				 * send RADIO_STATUS
				 * param RADIO_STATUS_radio_status * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_RADIO_STATUS(c_CommunicationChannel, RADIO_STATUS_radio_status) ( c_CommunicationChannel_send ( c_CommunicationChannel , RADIO_STATUS_radio_status)? ( (RADIO_STATUS_radio_status) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline RADIO_STATUS_radio_status *c_CommunicationChannel_new_RADIO_STATUS() { return new_pack(&mradio_status_RADIO_STATUS); }

/**
				 * send FENCE_POINT
				 * param FENCE_POINT_fence_point * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_FENCE_POINT(c_CommunicationChannel, FENCE_POINT_fence_point) ( c_CommunicationChannel_send ( c_CommunicationChannel , FENCE_POINT_fence_point)? ( (FENCE_POINT_fence_point) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline FENCE_POINT_fence_point *c_CommunicationChannel_new_FENCE_POINT() { return new_pack(&mfence_point_FENCE_POINT); }

/**
				 * send RESOURCE_REQUEST
				 * param RESOURCE_REQUEST_resource_request * pointer to the pack that will be send
				 * param c_CommunicationChannel * pointer to channel that contains transmitter
				 * return true if add pack to the sending queue succeed
				 */
#define c_CommunicationChannel_send_RESOURCE_REQUEST(c_CommunicationChannel, RESOURCE_REQUEST_resource_request) ( c_CommunicationChannel_send ( c_CommunicationChannel , RESOURCE_REQUEST_resource_request)? ( (RESOURCE_REQUEST_resource_request) = NULL), true : false)

/**
				 * brief Getting new pack that can be populated with data and send !!!!
				 * return new pack
				 */
static inline RESOURCE_REQUEST_resource_request *c_CommunicationChannel_new_RESOURCE_REQUEST() { return new_pack(&mresource_request_RESOURCE_REQUEST); }


#ifdef __cplusplus
}
#endif
									