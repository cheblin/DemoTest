
#ifdef __cplusplus
																														extern "C"
									{
#endif

#pragma once

#include "AdHoc.h"

typedef struct {
	uint8_t *bytes;
	size_t  len;
	size_t  BIT;
} BitsArray;

typedef struct {
	uint8_t *bytes;
	size_t  BIT;
} BitsValue;

typedef struct {
	uint8_t *bytes;
	size_t  len;
} BytesArray;

typedef struct {
	uint8_t *bytes;
} BytesValue;

#define AD_HOC_RECEIVE_REQ_MAX_BYTES 255
#define AD_HOC_RECEIVE_FULL_MAX_BYTES 6850


#define AD_HOC_SEND_REQ_MAX_BYTES 255
#define AD_HOC_SEND_FULL_MAX_BYTES 6850


static inline Cursor *wrap_pack(Pack *pack, Cursors wrapper) {
	wrapper->base.pack  = pack;
	wrapper->base.meta  = pack->meta;
	wrapper->base.bytes = pack->bytes;
	reset_cursor(wrapper);
	return wrapper;
}

static inline Pack *unwrap_pack(Cursors wrapper) {
	Pack *pack = wrapper->base.pack;
	wrapper->base.pack  = NULL;
	wrapper->base.meta  = NULL;
	wrapper->base.bytes = NULL;
	return pack;
}

static inline bool is_equal_data(Cursor *a, Cursor *b) {
	return (a && b && a == b) ||
	       (a->base.bytes != NULL &&
	        b->base.bytes != NULL &&
	        pack_size(&a->base) == pack_size(&b->base) &&
	        memcmp(a->base.bytes, b->base.bytes, a->base.LAST_BYTE) == 0);
}


#define mlog_request_data_LOG_REQUEST_DATA_packMinBytes (12) //mlog_request_data_LOG_REQUEST_DATA.packMinBytes value as constant
extern Meta mlog_request_data_LOG_REQUEST_DATA;

#define mfollow_target_FOLLOW_TARGET_packMinBytes (93) //mfollow_target_FOLLOW_TARGET.packMinBytes value as constant
extern Meta mfollow_target_FOLLOW_TARGET;

#define mmount_status_MOUNT_STATUS_packMinBytes (14) //mmount_status_MOUNT_STATUS.packMinBytes value as constant
extern Meta mmount_status_MOUNT_STATUS;

#define msystem_time_SYSTEM_TIME_packMinBytes (12) //msystem_time_SYSTEM_TIME.packMinBytes value as constant
extern Meta msystem_time_SYSTEM_TIME;

#define mlocal_position_ned_cov_LOCAL_POSITION_NED_COV_packMinBytes (225) //mlocal_position_ned_cov_LOCAL_POSITION_NED_COV.packMinBytes value as constant
extern Meta mlocal_position_ned_cov_LOCAL_POSITION_NED_COV;

#define mmag_cal_progress_MAG_CAL_PROGRESS_packMinBytes (27) //mmag_cal_progress_MAG_CAL_PROGRESS.packMinBytes value as constant
extern Meta mmag_cal_progress_MAG_CAL_PROGRESS;

#define mfile_transfer_protocol_FILE_TRANSFER_PROTOCOL_packMinBytes (254) //mfile_transfer_protocol_FILE_TRANSFER_PROTOCOL.packMinBytes value as constant
extern Meta mfile_transfer_protocol_FILE_TRANSFER_PROTOCOL;

#define mparam_ext_value_PARAM_EXT_VALUE_packMinBytes (5) //mparam_ext_value_PARAM_EXT_VALUE.packMinBytes value as constant
extern Meta mparam_ext_value_PARAM_EXT_VALUE;

#define mdata96_DATA96_packMinBytes (98) //mdata96_DATA96.packMinBytes value as constant
extern Meta mdata96_DATA96;

#define mdata64_DATA64_packMinBytes (66) //mdata64_DATA64.packMinBytes value as constant
extern Meta mdata64_DATA64;

#define mmount_orientation_MOUNT_ORIENTATION_packMinBytes (16) //mmount_orientation_MOUNT_ORIENTATION.packMinBytes value as constant
extern Meta mmount_orientation_MOUNT_ORIENTATION;

#define mmission_request_MISSION_REQUEST_packMinBytes (5) //mmission_request_MISSION_REQUEST.packMinBytes value as constant
extern Meta mmission_request_MISSION_REQUEST;

#define mparam_request_list_PARAM_REQUEST_LIST_packMinBytes (2) //mparam_request_list_PARAM_REQUEST_LIST.packMinBytes value as constant
extern Meta mparam_request_list_PARAM_REQUEST_LIST;

#define mfence_fetch_point_FENCE_FETCH_POINT_packMinBytes (3) //mfence_fetch_point_FENCE_FETCH_POINT.packMinBytes value as constant
extern Meta mfence_fetch_point_FENCE_FETCH_POINT;

#define madap_tuning_ADAP_TUNING_packMinBytes (49) //madap_tuning_ADAP_TUNING.packMinBytes value as constant
extern Meta madap_tuning_ADAP_TUNING;

#define mfence_status_FENCE_STATUS_packMinBytes (8) //mfence_status_FENCE_STATUS.packMinBytes value as constant
extern Meta mfence_status_FENCE_STATUS;

#define mparam_ext_request_list_PARAM_EXT_REQUEST_LIST_packMinBytes (2) //mparam_ext_request_list_PARAM_EXT_REQUEST_LIST.packMinBytes value as constant
extern Meta mparam_ext_request_list_PARAM_EXT_REQUEST_LIST;

#define mposition_target_global_int_POSITION_TARGET_GLOBAL_INT_packMinBytes (51) //mposition_target_global_int_POSITION_TARGET_GLOBAL_INT.packMinBytes value as constant
extern Meta mposition_target_global_int_POSITION_TARGET_GLOBAL_INT;

#define mgps2_rtk_GPS2_RTK_packMinBytes (35) //mgps2_rtk_GPS2_RTK.packMinBytes value as constant
extern Meta mgps2_rtk_GPS2_RTK;

#define mmessage_interval_MESSAGE_INTERVAL_packMinBytes (6) //mmessage_interval_MESSAGE_INTERVAL.packMinBytes value as constant
extern Meta mmessage_interval_MESSAGE_INTERVAL;

#define mresource_request_RESOURCE_REQUEST_packMinBytes (243) //mresource_request_RESOURCE_REQUEST.packMinBytes value as constant
extern Meta mresource_request_RESOURCE_REQUEST;

#define mrc_channels_RC_CHANNELS_packMinBytes (42) //mrc_channels_RC_CHANNELS.packMinBytes value as constant
extern Meta mrc_channels_RC_CHANNELS;

#define mset_home_position_SET_HOME_POSITION_packMinBytes (54) //mset_home_position_SET_HOME_POSITION.packMinBytes value as constant
extern Meta mset_home_position_SET_HOME_POSITION;

#define mglobal_position_int_GLOBAL_POSITION_INT_packMinBytes (28) //mglobal_position_int_GLOBAL_POSITION_INT.packMinBytes value as constant
extern Meta mglobal_position_int_GLOBAL_POSITION_INT;

#define mattitude_quaternion_ATTITUDE_QUATERNION_packMinBytes (32) //mattitude_quaternion_ATTITUDE_QUATERNION.packMinBytes value as constant
extern Meta mattitude_quaternion_ATTITUDE_QUATERNION;

#define mplay_tune_PLAY_TUNE_packMinBytes (3) //mplay_tune_PLAY_TUNE.packMinBytes value as constant
extern Meta mplay_tune_PLAY_TUNE;

#define muavcan_node_info_UAVCAN_NODE_INFO_packMinBytes (37) //muavcan_node_info_UAVCAN_NODE_INFO.packMinBytes value as constant
extern Meta muavcan_node_info_UAVCAN_NODE_INFO;

#define mgopro_set_request_GOPRO_SET_REQUEST_packMinBytes (7) //mgopro_set_request_GOPRO_SET_REQUEST.packMinBytes value as constant
extern Meta mgopro_set_request_GOPRO_SET_REQUEST;

#define mmission_item_int_MISSION_ITEM_INT_packMinBytes (35) //mmission_item_int_MISSION_ITEM_INT.packMinBytes value as constant
extern Meta mmission_item_int_MISSION_ITEM_INT;

#define mautopilot_version_request_AUTOPILOT_VERSION_REQUEST_packMinBytes (2) //mautopilot_version_request_AUTOPILOT_VERSION_REQUEST.packMinBytes value as constant
extern Meta mautopilot_version_request_AUTOPILOT_VERSION_REQUEST;

#define mdata_stream_DATA_STREAM_packMinBytes (4) //mdata_stream_DATA_STREAM.packMinBytes value as constant
extern Meta mdata_stream_DATA_STREAM;

#define mchange_operator_control_CHANGE_OPERATOR_CONTROL_packMinBytes (4) //mchange_operator_control_CHANGE_OPERATOR_CONTROL.packMinBytes value as constant
extern Meta mchange_operator_control_CHANGE_OPERATOR_CONTROL;

#define mterrain_check_TERRAIN_CHECK_packMinBytes (8) //mterrain_check_TERRAIN_CHECK.packMinBytes value as constant
extern Meta mterrain_check_TERRAIN_CHECK;

#define mcamera_trigger_CAMERA_TRIGGER_packMinBytes (12) //mcamera_trigger_CAMERA_TRIGGER.packMinBytes value as constant
extern Meta mcamera_trigger_CAMERA_TRIGGER;

#define map_adc_AP_ADC_packMinBytes (12) //map_adc_AP_ADC.packMinBytes value as constant
extern Meta map_adc_AP_ADC;

#define mmission_item_reached_MISSION_ITEM_REACHED_packMinBytes (2) //mmission_item_reached_MISSION_ITEM_REACHED.packMinBytes value as constant
extern Meta mmission_item_reached_MISSION_ITEM_REACHED;

#define mmeminfo_MEMINFO_packMinBytes (5) //mmeminfo_MEMINFO.packMinBytes value as constant
extern Meta mmeminfo_MEMINFO;

#define mdevice_op_write_reply_DEVICE_OP_WRITE_REPLY_packMinBytes (5) //mdevice_op_write_reply_DEVICE_OP_WRITE_REPLY.packMinBytes value as constant
extern Meta mdevice_op_write_reply_DEVICE_OP_WRITE_REPLY;

#define mping33_PING33_packMinBytes (155) //mping33_PING33.packMinBytes value as constant
extern Meta mping33_PING33;

#define mrally_fetch_point_RALLY_FETCH_POINT_packMinBytes (3) //mrally_fetch_point_RALLY_FETCH_POINT.packMinBytes value as constant
extern Meta mrally_fetch_point_RALLY_FETCH_POINT;

#define mmission_request_list_MISSION_REQUEST_LIST_packMinBytes (3) //mmission_request_list_MISSION_REQUEST_LIST.packMinBytes value as constant
extern Meta mmission_request_list_MISSION_REQUEST_LIST;

#define mgopro_heartbeat_GOPRO_HEARTBEAT_packMinBytes (1) //mgopro_heartbeat_GOPRO_HEARTBEAT.packMinBytes value as constant
extern Meta mgopro_heartbeat_GOPRO_HEARTBEAT;

#define mdebug_vect_DEBUG_VECT_packMinBytes (21) //mdebug_vect_DEBUG_VECT.packMinBytes value as constant
extern Meta mdebug_vect_DEBUG_VECT;

#define mnamed_value_float_NAMED_VALUE_FLOAT_packMinBytes (9) //mnamed_value_float_NAMED_VALUE_FLOAT.packMinBytes value as constant
extern Meta mnamed_value_float_NAMED_VALUE_FLOAT;

#define mv2_extension_V2_EXTENSION_packMinBytes (254) //mv2_extension_V2_EXTENSION.packMinBytes value as constant
extern Meta mv2_extension_V2_EXTENSION;

#define mlocal_position_ned_LOCAL_POSITION_NED_packMinBytes (28) //mlocal_position_ned_LOCAL_POSITION_NED.packMinBytes value as constant
extern Meta mlocal_position_ned_LOCAL_POSITION_NED;

#define mrpm_RPM_packMinBytes (8) //mrpm_RPM.packMinBytes value as constant
extern Meta mrpm_RPM;

#define mpower_status_POWER_STATUS_packMinBytes (5) //mpower_status_POWER_STATUS.packMinBytes value as constant
extern Meta mpower_status_POWER_STATUS;

#define mraw_imu_RAW_IMU_packMinBytes (26) //mraw_imu_RAW_IMU.packMinBytes value as constant
extern Meta mraw_imu_RAW_IMU;

#define mbattery2_BATTERY2_packMinBytes (4) //mbattery2_BATTERY2.packMinBytes value as constant
extern Meta mbattery2_BATTERY2;

#define mmemory_vect_MEMORY_VECT_packMinBytes (36) //mmemory_vect_MEMORY_VECT.packMinBytes value as constant
extern Meta mmemory_vect_MEMORY_VECT;

#define msafety_allowed_area_SAFETY_ALLOWED_AREA_packMinBytes (25) //msafety_allowed_area_SAFETY_ALLOWED_AREA.packMinBytes value as constant
extern Meta msafety_allowed_area_SAFETY_ALLOWED_AREA;

#define mparam_set_PARAM_SET_packMinBytes (7) //mparam_set_PARAM_SET.packMinBytes value as constant
extern Meta mparam_set_PARAM_SET;

#define mremote_log_data_block_REMOTE_LOG_DATA_BLOCK_packMinBytes (203) //mremote_log_data_block_REMOTE_LOG_DATA_BLOCK.packMinBytes value as constant
extern Meta mremote_log_data_block_REMOTE_LOG_DATA_BLOCK;

#define mvision_position_delta_VISION_POSITION_DELTA_packMinBytes (44) //mvision_position_delta_VISION_POSITION_DELTA.packMinBytes value as constant
extern Meta mvision_position_delta_VISION_POSITION_DELTA;

#define mhome_position_HOME_POSITION_packMinBytes (53) //mhome_position_HOME_POSITION.packMinBytes value as constant
extern Meta mhome_position_HOME_POSITION;

#define mdevice_op_read_reply_DEVICE_OP_READ_REPLY_packMinBytes (135) //mdevice_op_read_reply_DEVICE_OP_READ_REPLY.packMinBytes value as constant
extern Meta mdevice_op_read_reply_DEVICE_OP_READ_REPLY;

#define mrc_channels_scaled_RC_CHANNELS_SCALED_packMinBytes (22) //mrc_channels_scaled_RC_CHANNELS_SCALED.packMinBytes value as constant
extern Meta mrc_channels_scaled_RC_CHANNELS_SCALED;

#define mgps_status_GPS_STATUS_packMinBytes (101) //mgps_status_GPS_STATUS.packMinBytes value as constant
extern Meta mgps_status_GPS_STATUS;

#define mprotocol_version_PROTOCOL_VERSION_packMinBytes (22) //mprotocol_version_PROTOCOL_VERSION.packMinBytes value as constant
extern Meta mprotocol_version_PROTOCOL_VERSION;

#define mrc_channels_override_RC_CHANNELS_OVERRIDE_packMinBytes (18) //mrc_channels_override_RC_CHANNELS_OVERRIDE.packMinBytes value as constant
extern Meta mrc_channels_override_RC_CHANNELS_OVERRIDE;

#define mlog_erase_LOG_ERASE_packMinBytes (2) //mlog_erase_LOG_ERASE.packMinBytes value as constant
extern Meta mlog_erase_LOG_ERASE;

#define mwind_cov_WIND_COV_packMinBytes (40) //mwind_cov_WIND_COV.packMinBytes value as constant
extern Meta mwind_cov_WIND_COV;

#define mhil_state_HIL_STATE_packMinBytes (56) //mhil_state_HIL_STATE.packMinBytes value as constant
extern Meta mhil_state_HIL_STATE;

#define mvfr_hud_VFR_HUD_packMinBytes (20) //mvfr_hud_VFR_HUD.packMinBytes value as constant
extern Meta mvfr_hud_VFR_HUD;

#define mgimbal_control_GIMBAL_CONTROL_packMinBytes (14) //mgimbal_control_GIMBAL_CONTROL.packMinBytes value as constant
extern Meta mgimbal_control_GIMBAL_CONTROL;


extern Meta mswitchmodecommand_SwitchModeCommand;

#define msimstate_SIMSTATE_packMinBytes (44) //msimstate_SIMSTATE.packMinBytes value as constant
extern Meta msimstate_SIMSTATE;

#define mcompassmot_status_COMPASSMOT_STATUS_packMinBytes (20) //mcompassmot_status_COMPASSMOT_STATUS.packMinBytes value as constant
extern Meta mcompassmot_status_COMPASSMOT_STATUS;

#define msafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA_packMinBytes (27) //msafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA.packMinBytes value as constant
extern Meta msafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA;

#define mscaled_imu_SCALED_IMU_packMinBytes (22) //mscaled_imu_SCALED_IMU.packMinBytes value as constant
extern Meta mscaled_imu_SCALED_IMU;

#define mglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE_packMinBytes (32) //mglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE.packMinBytes value as constant
extern Meta mglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE;

#define mlogging_ack_LOGGING_ACK_packMinBytes (4) //mlogging_ack_LOGGING_ACK.packMinBytes value as constant
extern Meta mlogging_ack_LOGGING_ACK;

#define mlog_request_end_LOG_REQUEST_END_packMinBytes (2) //mlog_request_end_LOG_REQUEST_END.packMinBytes value as constant
extern Meta mlog_request_end_LOG_REQUEST_END;

#define mlog_request_list_LOG_REQUEST_LIST_packMinBytes (6) //mlog_request_list_LOG_REQUEST_LIST.packMinBytes value as constant
extern Meta mlog_request_list_LOG_REQUEST_LIST;

#define mgps_rtk_GPS_RTK_packMinBytes (35) //mgps_rtk_GPS_RTK.packMinBytes value as constant
extern Meta mgps_rtk_GPS_RTK;

#define mgimbal_report_GIMBAL_REPORT_packMinBytes (42) //mgimbal_report_GIMBAL_REPORT.packMinBytes value as constant
extern Meta mgimbal_report_GIMBAL_REPORT;

#define mactuator_control_target_ACTUATOR_CONTROL_TARGET_packMinBytes (41) //mactuator_control_target_ACTUATOR_CONTROL_TARGET.packMinBytes value as constant
extern Meta mactuator_control_target_ACTUATOR_CONTROL_TARGET;

#define mmission_request_int_MISSION_REQUEST_INT_packMinBytes (5) //mmission_request_int_MISSION_REQUEST_INT.packMinBytes value as constant
extern Meta mmission_request_int_MISSION_REQUEST_INT;

#define msim_state_SIM_STATE_packMinBytes (84) //msim_state_SIM_STATE.packMinBytes value as constant
extern Meta msim_state_SIM_STATE;

#define mobstacle_distance_OBSTACLE_DISTANCE_packMinBytes (158) //mobstacle_distance_OBSTACLE_DISTANCE.packMinBytes value as constant
extern Meta mobstacle_distance_OBSTACLE_DISTANCE;

#define mgps_rtcm_data_GPS_RTCM_DATA_packMinBytes (182) //mgps_rtcm_data_GPS_RTCM_DATA.packMinBytes value as constant
extern Meta mgps_rtcm_data_GPS_RTCM_DATA;

#define mraw_pressure_RAW_PRESSURE_packMinBytes (16) //mraw_pressure_RAW_PRESSURE.packMinBytes value as constant
extern Meta mraw_pressure_RAW_PRESSURE;

#define mscaled_pressure2_SCALED_PRESSURE2_packMinBytes (14) //mscaled_pressure2_SCALED_PRESSURE2.packMinBytes value as constant
extern Meta mscaled_pressure2_SCALED_PRESSURE2;

#define mencapsulated_data_ENCAPSULATED_DATA_packMinBytes (255) //mencapsulated_data_ENCAPSULATED_DATA.packMinBytes value as constant
extern Meta mencapsulated_data_ENCAPSULATED_DATA;

#define mhil_gps_HIL_GPS_packMinBytes (36) //mhil_gps_HIL_GPS.packMinBytes value as constant
extern Meta mhil_gps_HIL_GPS;

#define mdata32_DATA32_packMinBytes (34) //mdata32_DATA32.packMinBytes value as constant
extern Meta mdata32_DATA32;

#define maltitude_ALTITUDE_packMinBytes (32) //maltitude_ALTITUDE.packMinBytes value as constant
extern Meta maltitude_ALTITUDE;

#define mmission_count_MISSION_COUNT_packMinBytes (5) //mmission_count_MISSION_COUNT.packMinBytes value as constant
extern Meta mmission_count_MISSION_COUNT;

#define mscaled_imu3_SCALED_IMU3_packMinBytes (22) //mscaled_imu3_SCALED_IMU3.packMinBytes value as constant
extern Meta mscaled_imu3_SCALED_IMU3;

#define mgps_global_origin_GPS_GLOBAL_ORIGIN_packMinBytes (13) //mgps_global_origin_GPS_GLOBAL_ORIGIN.packMinBytes value as constant
extern Meta mgps_global_origin_GPS_GLOBAL_ORIGIN;

#define mcommand_long_COMMAND_LONG_packMinBytes (32) //mcommand_long_COMMAND_LONG.packMinBytes value as constant
extern Meta mcommand_long_COMMAND_LONG;

#define moptical_flow_rad_OPTICAL_FLOW_RAD_packMinBytes (44) //moptical_flow_rad_OPTICAL_FLOW_RAD.packMinBytes value as constant
extern Meta moptical_flow_rad_OPTICAL_FLOW_RAD;

#define mextended_sys_state_EXTENDED_SYS_STATE_packMinBytes (1) //mextended_sys_state_EXTENDED_SYS_STATE.packMinBytes value as constant
extern Meta mextended_sys_state_EXTENDED_SYS_STATE;

#define mmission_set_current_MISSION_SET_CURRENT_packMinBytes (4) //mmission_set_current_MISSION_SET_CURRENT.packMinBytes value as constant
extern Meta mmission_set_current_MISSION_SET_CURRENT;

#define mekf_status_report_EKF_STATUS_REPORT_packMinBytes (21) //mekf_status_report_EKF_STATUS_REPORT.packMinBytes value as constant
extern Meta mekf_status_report_EKF_STATUS_REPORT;

#define mmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST_packMinBytes (7) //mmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST.packMinBytes value as constant
extern Meta mmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST;

#define matt_pos_mocap_ATT_POS_MOCAP_packMinBytes (36) //matt_pos_mocap_ATT_POS_MOCAP.packMinBytes value as constant
extern Meta matt_pos_mocap_ATT_POS_MOCAP;

#define mheartbeat_HEARTBEAT_packMinBytes (6) //mheartbeat_HEARTBEAT.packMinBytes value as constant
extern Meta mheartbeat_HEARTBEAT;

#define mscaled_pressure_SCALED_PRESSURE_packMinBytes (14) //mscaled_pressure_SCALED_PRESSURE.packMinBytes value as constant
extern Meta mscaled_pressure_SCALED_PRESSURE;

#define mradio_RADIO_packMinBytes (9) //mradio_RADIO.packMinBytes value as constant
extern Meta mradio_RADIO;

#define mflight_information_FLIGHT_INFORMATION_packMinBytes (28) //mflight_information_FLIGHT_INFORMATION.packMinBytes value as constant
extern Meta mflight_information_FLIGHT_INFORMATION;

#define mmanual_setpoint_MANUAL_SETPOINT_packMinBytes (22) //mmanual_setpoint_MANUAL_SETPOINT.packMinBytes value as constant
extern Meta mmanual_setpoint_MANUAL_SETPOINT;

#define mgopro_set_response_GOPRO_SET_RESPONSE_packMinBytes (1) //mgopro_set_response_GOPRO_SET_RESPONSE.packMinBytes value as constant
extern Meta mgopro_set_response_GOPRO_SET_RESPONSE;

#define mparam_map_rc_PARAM_MAP_RC_packMinBytes (22) //mparam_map_rc_PARAM_MAP_RC.packMinBytes value as constant
extern Meta mparam_map_rc_PARAM_MAP_RC;

#define mscaled_imu2_SCALED_IMU2_packMinBytes (22) //mscaled_imu2_SCALED_IMU2.packMinBytes value as constant
extern Meta mscaled_imu2_SCALED_IMU2;

#define mdistance_sensor_DISTANCE_SENSOR_packMinBytes (13) //mdistance_sensor_DISTANCE_SENSOR.packMinBytes value as constant
extern Meta mdistance_sensor_DISTANCE_SENSOR;

#define mgopro_get_response_GOPRO_GET_RESPONSE_packMinBytes (5) //mgopro_get_response_GOPRO_GET_RESPONSE.packMinBytes value as constant
extern Meta mgopro_get_response_GOPRO_GET_RESPONSE;

#define mparam_ext_ack_PARAM_EXT_ACK_packMinBytes (1) //mparam_ext_ack_PARAM_EXT_ACK.packMinBytes value as constant
extern Meta mparam_ext_ack_PARAM_EXT_ACK;

#define mrangefinder_RANGEFINDER_packMinBytes (8) //mrangefinder_RANGEFINDER.packMinBytes value as constant
extern Meta mrangefinder_RANGEFINDER;

#define mfence_point_FENCE_POINT_packMinBytes (12) //mfence_point_FENCE_POINT.packMinBytes value as constant
extern Meta mfence_point_FENCE_POINT;

#define mbattery_status_BATTERY_STATUS_packMinBytes (35) //mbattery_status_BATTERY_STATUS.packMinBytes value as constant
extern Meta mbattery_status_BATTERY_STATUS;

#define mtimesync_TIMESYNC_packMinBytes (16) //mtimesync_TIMESYNC.packMinBytes value as constant
extern Meta mtimesync_TIMESYNC;

#define mdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE_packMinBytes (13) //mdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE.packMinBytes value as constant
extern Meta mdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE;

#define mcamera_status_CAMERA_STATUS_packMinBytes (29) //mcamera_status_CAMERA_STATUS.packMinBytes value as constant
extern Meta mcamera_status_CAMERA_STATUS;

#define mrally_point_RALLY_POINT_packMinBytes (19) //mrally_point_RALLY_POINT.packMinBytes value as constant
extern Meta mrally_point_RALLY_POINT;

#define mmount_configure_MOUNT_CONFIGURE_packMinBytes (6) //mmount_configure_MOUNT_CONFIGURE.packMinBytes value as constant
extern Meta mmount_configure_MOUNT_CONFIGURE;

#define mmag_cal_report_MAG_CAL_REPORT_packMinBytes (44) //mmag_cal_report_MAG_CAL_REPORT.packMinBytes value as constant
extern Meta mmag_cal_report_MAG_CAL_REPORT;

#define mestimator_status_ESTIMATOR_STATUS_packMinBytes (41) //mestimator_status_ESTIMATOR_STATUS.packMinBytes value as constant
extern Meta mestimator_status_ESTIMATOR_STATUS;

#define mhil_sensor_HIL_SENSOR_packMinBytes (64) //mhil_sensor_HIL_SENSOR.packMinBytes value as constant
extern Meta mhil_sensor_HIL_SENSOR;

#define mglobal_position_int_cov_GLOBAL_POSITION_INT_COV_packMinBytes (181) //mglobal_position_int_cov_GLOBAL_POSITION_INT_COV.packMinBytes value as constant
extern Meta mglobal_position_int_cov_GLOBAL_POSITION_INT_COV;

#define mbutton_change_BUTTON_CHANGE_packMinBytes (9) //mbutton_change_BUTTON_CHANGE.packMinBytes value as constant
extern Meta mbutton_change_BUTTON_CHANGE;

#define mgopro_get_request_GOPRO_GET_REQUEST_packMinBytes (3) //mgopro_get_request_GOPRO_GET_REQUEST.packMinBytes value as constant
extern Meta mgopro_get_request_GOPRO_GET_REQUEST;

#define mmission_current_MISSION_CURRENT_packMinBytes (2) //mmission_current_MISSION_CURRENT.packMinBytes value as constant
extern Meta mmission_current_MISSION_CURRENT;

#define mset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED_packMinBytes (53) //mset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED.packMinBytes value as constant
extern Meta mset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED;

#define msys_status_SYS_STATUS_packMinBytes (20) //msys_status_SYS_STATUS.packMinBytes value as constant
extern Meta msys_status_SYS_STATUS;

#define mset_mode_SET_MODE_packMinBytes (6) //mset_mode_SET_MODE.packMinBytes value as constant
extern Meta mset_mode_SET_MODE;

#define mattitude_target_ATTITUDE_TARGET_packMinBytes (37) //mattitude_target_ATTITUDE_TARGET.packMinBytes value as constant
extern Meta mattitude_target_ATTITUDE_TARGET;

#define mhil_controls_HIL_CONTROLS_packMinBytes (42) //mhil_controls_HIL_CONTROLS.packMinBytes value as constant
extern Meta mhil_controls_HIL_CONTROLS;

#define mhil_rc_inputs_raw_HIL_RC_INPUTS_RAW_packMinBytes (33) //mhil_rc_inputs_raw_HIL_RC_INPUTS_RAW.packMinBytes value as constant
extern Meta mhil_rc_inputs_raw_HIL_RC_INPUTS_RAW;

#define mradio_status_RADIO_STATUS_packMinBytes (9) //mradio_status_RADIO_STATUS.packMinBytes value as constant
extern Meta mradio_status_RADIO_STATUS;

#define moptical_flow_OPTICAL_FLOW_packMinBytes (27) //moptical_flow_OPTICAL_FLOW.packMinBytes value as constant
extern Meta moptical_flow_OPTICAL_FLOW;

#define mserial_control_SERIAL_CONTROL_packMinBytes (78) //mserial_control_SERIAL_CONTROL.packMinBytes value as constant
extern Meta mserial_control_SERIAL_CONTROL;

#define mhwstatus_HWSTATUS_packMinBytes (3) //mhwstatus_HWSTATUS.packMinBytes value as constant
extern Meta mhwstatus_HWSTATUS;

#define mterrain_data_TERRAIN_DATA_packMinBytes (43) //mterrain_data_TERRAIN_DATA.packMinBytes value as constant
extern Meta mterrain_data_TERRAIN_DATA;

#define mcamera_settings_CAMERA_SETTINGS_packMinBytes (5) //mcamera_settings_CAMERA_SETTINGS.packMinBytes value as constant
extern Meta mcamera_settings_CAMERA_SETTINGS;

#define mmount_control_MOUNT_CONTROL_packMinBytes (15) //mmount_control_MOUNT_CONTROL.packMinBytes value as constant
extern Meta mmount_control_MOUNT_CONTROL;

#define mcontrol_system_state_CONTROL_SYSTEM_STATE_packMinBytes (100) //mcontrol_system_state_CONTROL_SYSTEM_STATE.packMinBytes value as constant
extern Meta mcontrol_system_state_CONTROL_SYSTEM_STATE;

#define mnamed_value_int_NAMED_VALUE_INT_packMinBytes (9) //mnamed_value_int_NAMED_VALUE_INT.packMinBytes value as constant
extern Meta mnamed_value_int_NAMED_VALUE_INT;

#define mwind_WIND_packMinBytes (12) //mwind_WIND.packMinBytes value as constant
extern Meta mwind_WIND;

#define mset_attitude_target_SET_ATTITUDE_TARGET_packMinBytes (39) //mset_attitude_target_SET_ATTITUDE_TARGET.packMinBytes value as constant
extern Meta mset_attitude_target_SET_ATTITUDE_TARGET;

#define mposition_target_local_ned_POSITION_TARGET_LOCAL_NED_packMinBytes (51) //mposition_target_local_ned_POSITION_TARGET_LOCAL_NED.packMinBytes value as constant
extern Meta mposition_target_local_ned_POSITION_TARGET_LOCAL_NED;

#define mparam_request_read_PARAM_REQUEST_READ_packMinBytes (5) //mparam_request_read_PARAM_REQUEST_READ.packMinBytes value as constant
extern Meta mparam_request_read_PARAM_REQUEST_READ;

#define mhil_optical_flow_HIL_OPTICAL_FLOW_packMinBytes (44) //mhil_optical_flow_HIL_OPTICAL_FLOW.packMinBytes value as constant
extern Meta mhil_optical_flow_HIL_OPTICAL_FLOW;

#define mgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT_packMinBytes (8) //mgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT.packMinBytes value as constant
extern Meta mgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT;

#define mset_mag_offsets_SET_MAG_OFFSETS_packMinBytes (8) //mset_mag_offsets_SET_MAG_OFFSETS.packMinBytes value as constant
extern Meta mset_mag_offsets_SET_MAG_OFFSETS;

#define mparam_ext_set_PARAM_EXT_SET_packMinBytes (3) //mparam_ext_set_PARAM_EXT_SET.packMinBytes value as constant
extern Meta mparam_ext_set_PARAM_EXT_SET;

#define mlogging_data_acked_LOGGING_DATA_ACKED_packMinBytes (255) //mlogging_data_acked_LOGGING_DATA_ACKED.packMinBytes value as constant
extern Meta mlogging_data_acked_LOGGING_DATA_ACKED;

#define mcommand_ack_COMMAND_ACK_packMinBytes (1) //mcommand_ack_COMMAND_ACK.packMinBytes value as constant
extern Meta mcommand_ack_COMMAND_ACK;

#define mvision_position_estimate_VISION_POSITION_ESTIMATE_packMinBytes (32) //mvision_position_estimate_VISION_POSITION_ESTIMATE.packMinBytes value as constant
extern Meta mvision_position_estimate_VISION_POSITION_ESTIMATE;

#define mled_control_LED_CONTROL_packMinBytes (29) //mled_control_LED_CONTROL.packMinBytes value as constant
extern Meta mled_control_LED_CONTROL;

#define mahrs3_AHRS3_packMinBytes (40) //mahrs3_AHRS3.packMinBytes value as constant
extern Meta mahrs3_AHRS3;

#define mattitude_quaternion_cov_ATTITUDE_QUATERNION_COV_packMinBytes (72) //mattitude_quaternion_cov_ATTITUDE_QUATERNION_COV.packMinBytes value as constant
extern Meta mattitude_quaternion_cov_ATTITUDE_QUATERNION_COV;

#define muavcan_node_status_UAVCAN_NODE_STATUS_packMinBytes (16) //muavcan_node_status_UAVCAN_NODE_STATUS.packMinBytes value as constant
extern Meta muavcan_node_status_UAVCAN_NODE_STATUS;

#define mdigicam_control_DIGICAM_CONTROL_packMinBytes (13) //mdigicam_control_DIGICAM_CONTROL.packMinBytes value as constant
extern Meta mdigicam_control_DIGICAM_CONTROL;

#define mterrain_report_TERRAIN_REPORT_packMinBytes (22) //mterrain_report_TERRAIN_REPORT.packMinBytes value as constant
extern Meta mterrain_report_TERRAIN_REPORT;

#define mservo_output_raw_SERVO_OUTPUT_RAW_packMinBytes (22) //mservo_output_raw_SERVO_OUTPUT_RAW.packMinBytes value as constant
extern Meta mservo_output_raw_SERVO_OUTPUT_RAW;

#define mgps_inject_data_GPS_INJECT_DATA_packMinBytes (113) //mgps_inject_data_GPS_INJECT_DATA.packMinBytes value as constant
extern Meta mgps_inject_data_GPS_INJECT_DATA;

#define mvision_speed_estimate_VISION_SPEED_ESTIMATE_packMinBytes (20) //mvision_speed_estimate_VISION_SPEED_ESTIMATE.packMinBytes value as constant
extern Meta mvision_speed_estimate_VISION_SPEED_ESTIMATE;

#define mahrs_AHRS_packMinBytes (28) //mahrs_AHRS.packMinBytes value as constant
extern Meta mahrs_AHRS;

#define mstatustext_STATUSTEXT_packMinBytes (1) //mstatustext_STATUSTEXT.packMinBytes value as constant
extern Meta mstatustext_STATUSTEXT;

#define mlogging_data_LOGGING_DATA_packMinBytes (255) //mlogging_data_LOGGING_DATA.packMinBytes value as constant
extern Meta mlogging_data_LOGGING_DATA;

#define mwifi_config_ap_WIFI_CONFIG_AP_packMinBytes (1) //mwifi_config_ap_WIFI_CONFIG_AP.packMinBytes value as constant
extern Meta mwifi_config_ap_WIFI_CONFIG_AP;

#define mdevice_op_read_DEVICE_OP_READ_packMinBytes (11) //mdevice_op_read_DEVICE_OP_READ.packMinBytes value as constant
extern Meta mdevice_op_read_DEVICE_OP_READ;

#define mset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT_packMinBytes (53) //mset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT.packMinBytes value as constant
extern Meta mset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT;

#define mcommand_int_COMMAND_INT_packMinBytes (33) //mcommand_int_COMMAND_INT.packMinBytes value as constant
extern Meta mcommand_int_COMMAND_INT;

#define mscaled_pressure3_SCALED_PRESSURE3_packMinBytes (14) //mscaled_pressure3_SCALED_PRESSURE3.packMinBytes value as constant
extern Meta mscaled_pressure3_SCALED_PRESSURE3;

#define mcamera_capture_status_CAMERA_CAPTURE_STATUS_packMinBytes (18) //mcamera_capture_status_CAMERA_CAPTURE_STATUS.packMinBytes value as constant
extern Meta mcamera_capture_status_CAMERA_CAPTURE_STATUS;

#define mgps2_raw_GPS2_RAW_packMinBytes (35) //mgps2_raw_GPS2_RAW.packMinBytes value as constant
extern Meta mgps2_raw_GPS2_RAW;

#define muavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_packMinBytes (7) //muavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG.packMinBytes value as constant
extern Meta muavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG;

#define mvicon_position_estimate_VICON_POSITION_ESTIMATE_packMinBytes (32) //mvicon_position_estimate_VICON_POSITION_ESTIMATE.packMinBytes value as constant
extern Meta mvicon_position_estimate_VICON_POSITION_ESTIMATE;

#define mmission_ack_MISSION_ACK_packMinBytes (3) //mmission_ack_MISSION_ACK.packMinBytes value as constant
extern Meta mmission_ack_MISSION_ACK;

#define mrc_channels_raw_RC_CHANNELS_RAW_packMinBytes (22) //mrc_channels_raw_RC_CHANNELS_RAW.packMinBytes value as constant
extern Meta mrc_channels_raw_RC_CHANNELS_RAW;

#define mparam_ext_request_read_PARAM_EXT_REQUEST_READ_packMinBytes (5) //mparam_ext_request_read_PARAM_EXT_REQUEST_READ.packMinBytes value as constant
extern Meta mparam_ext_request_read_PARAM_EXT_REQUEST_READ;

#define mahrs2_AHRS2_packMinBytes (24) //mahrs2_AHRS2.packMinBytes value as constant
extern Meta mahrs2_AHRS2;

#define mairspeed_autocal_AIRSPEED_AUTOCAL_packMinBytes (48) //mairspeed_autocal_AIRSPEED_AUTOCAL.packMinBytes value as constant
extern Meta mairspeed_autocal_AIRSPEED_AUTOCAL;

#define mlocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_packMinBytes (28) //mlocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.packMinBytes value as constant
extern Meta mlocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;

#define mlog_data_LOG_DATA_packMinBytes (97) //mlog_data_LOG_DATA.packMinBytes value as constant
extern Meta mlog_data_LOG_DATA;

#define mstorage_information_STORAGE_INFORMATION_packMinBytes (27) //mstorage_information_STORAGE_INFORMATION.packMinBytes value as constant
extern Meta mstorage_information_STORAGE_INFORMATION;

#define mhigh_latency_HIGH_LATENCY_packMinBytes (38) //mhigh_latency_HIGH_LATENCY.packMinBytes value as constant
extern Meta mhigh_latency_HIGH_LATENCY;

#define mterrain_request_TERRAIN_REQUEST_packMinBytes (18) //mterrain_request_TERRAIN_REQUEST.packMinBytes value as constant
extern Meta mterrain_request_TERRAIN_REQUEST;

#define mchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK_packMinBytes (3) //mchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK.packMinBytes value as constant
extern Meta mchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK;

#define mset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN_packMinBytes (14) //mset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN.packMinBytes value as constant
extern Meta mset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN;

#define msensor_offsets_SENSOR_OFFSETS_packMinBytes (42) //msensor_offsets_SENSOR_OFFSETS.packMinBytes value as constant
extern Meta msensor_offsets_SENSOR_OFFSETS;

#define mdigicam_configure_DIGICAM_CONFIGURE_packMinBytes (15) //mdigicam_configure_DIGICAM_CONFIGURE.packMinBytes value as constant
extern Meta mdigicam_configure_DIGICAM_CONFIGURE;

#define mhil_actuator_controls_HIL_ACTUATOR_CONTROLS_packMinBytes (81) //mhil_actuator_controls_HIL_ACTUATOR_CONTROLS.packMinBytes value as constant
extern Meta mhil_actuator_controls_HIL_ACTUATOR_CONTROLS;

#define mhighres_imu_HIGHRES_IMU_packMinBytes (62) //mhighres_imu_HIGHRES_IMU.packMinBytes value as constant
extern Meta mhighres_imu_HIGHRES_IMU;

#define mpid_tuning_PID_TUNING_packMinBytes (25) //mpid_tuning_PID_TUNING.packMinBytes value as constant
extern Meta mpid_tuning_PID_TUNING;

#define mcollision_COLLISION_packMinBytes (17) //mcollision_COLLISION.packMinBytes value as constant
extern Meta mcollision_COLLISION;

#define mparam_value_PARAM_VALUE_packMinBytes (9) //mparam_value_PARAM_VALUE.packMinBytes value as constant
extern Meta mparam_value_PARAM_VALUE;

#define mmission_item_MISSION_ITEM_packMinBytes (35) //mmission_item_MISSION_ITEM.packMinBytes value as constant
extern Meta mmission_item_MISSION_ITEM;

#define madsb_vehicle_ADSB_VEHICLE_packMinBytes (26) //madsb_vehicle_ADSB_VEHICLE.packMinBytes value as constant
extern Meta madsb_vehicle_ADSB_VEHICLE;

#define mdevice_op_write_DEVICE_OP_WRITE_packMinBytes (139) //mdevice_op_write_DEVICE_OP_WRITE.packMinBytes value as constant
extern Meta mdevice_op_write_DEVICE_OP_WRITE;

#define mcamera_information_CAMERA_INFORMATION_packMinBytes (92) //mcamera_information_CAMERA_INFORMATION.packMinBytes value as constant
extern Meta mcamera_information_CAMERA_INFORMATION;

#define mauth_key_AUTH_KEY_packMinBytes (1) //mauth_key_AUTH_KEY.packMinBytes value as constant
extern Meta mauth_key_AUTH_KEY;

#define mlimits_status_LIMITS_STATUS_packMinBytes (19) //mlimits_status_LIMITS_STATUS.packMinBytes value as constant
extern Meta mlimits_status_LIMITS_STATUS;

#define mlog_entry_LOG_ENTRY_packMinBytes (14) //mlog_entry_LOG_ENTRY.packMinBytes value as constant
extern Meta mlog_entry_LOG_ENTRY;

#define mvideo_stream_information_VIDEO_STREAM_INFORMATION_packMinBytes (17) //mvideo_stream_information_VIDEO_STREAM_INFORMATION.packMinBytes value as constant
extern Meta mvideo_stream_information_VIDEO_STREAM_INFORMATION;

#define mgps_raw_int_GPS_RAW_INT_packMinBytes (30) //mgps_raw_int_GPS_RAW_INT.packMinBytes value as constant
extern Meta mgps_raw_int_GPS_RAW_INT;

#define mping_PING_packMinBytes (14) //mping_PING.packMinBytes value as constant
extern Meta mping_PING;

#define mattitude_ATTITUDE_packMinBytes (28) //mattitude_ATTITUDE.packMinBytes value as constant
extern Meta mattitude_ATTITUDE;

#define msetup_signing_SETUP_SIGNING_packMinBytes (42) //msetup_signing_SETUP_SIGNING.packMinBytes value as constant
extern Meta msetup_signing_SETUP_SIGNING;

#define mhil_state_quaternion_HIL_STATE_QUATERNION_packMinBytes (64) //mhil_state_quaternion_HIL_STATE_QUATERNION.packMinBytes value as constant
extern Meta mhil_state_quaternion_HIL_STATE_QUATERNION;

#define mdebug_DEBUG_packMinBytes (9) //mdebug_DEBUG.packMinBytes value as constant
extern Meta mdebug_DEBUG;

#define mremote_log_block_status_REMOTE_LOG_BLOCK_STATUS_packMinBytes (7) //mremote_log_block_status_REMOTE_LOG_BLOCK_STATUS.packMinBytes value as constant
extern Meta mremote_log_block_status_REMOTE_LOG_BLOCK_STATUS;

#define muavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_packMinBytes (38) //muavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC.packMinBytes value as constant
extern Meta muavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC;

#define mcamera_image_captured_CAMERA_IMAGE_CAPTURED_packMinBytes (51) //mcamera_image_captured_CAMERA_IMAGE_CAPTURED.packMinBytes value as constant
extern Meta mcamera_image_captured_CAMERA_IMAGE_CAPTURED;

#define mmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST_packMinBytes (7) //mmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST.packMinBytes value as constant
extern Meta mmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST;

#define mlanding_target_LANDING_TARGET_packMinBytes (30) //mlanding_target_LANDING_TARGET.packMinBytes value as constant
extern Meta mlanding_target_LANDING_TARGET;

#define mrequest_data_stream_REQUEST_DATA_STREAM_packMinBytes (6) //mrequest_data_stream_REQUEST_DATA_STREAM.packMinBytes value as constant
extern Meta mrequest_data_stream_REQUEST_DATA_STREAM;

#define mmanual_control_MANUAL_CONTROL_packMinBytes (11) //mmanual_control_MANUAL_CONTROL.packMinBytes value as constant
extern Meta mmanual_control_MANUAL_CONTROL;

#define mdata16_DATA16_packMinBytes (18) //mdata16_DATA16.packMinBytes value as constant
extern Meta mdata16_DATA16;

#define mset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS_packMinBytes (18) //mset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS.packMinBytes value as constant
extern Meta mset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS;

#define mgps_input_GPS_INPUT_packMinBytes (62) //mgps_input_GPS_INPUT.packMinBytes value as constant
extern Meta mgps_input_GPS_INPUT;

#define mvibration_VIBRATION_packMinBytes (32) //mvibration_VIBRATION.packMinBytes value as constant
extern Meta mvibration_VIBRATION;

#define mautopilot_version_AUTOPILOT_VERSION_packMinBytes (53) //mautopilot_version_AUTOPILOT_VERSION.packMinBytes value as constant
extern Meta mautopilot_version_AUTOPILOT_VERSION;

#define mmission_clear_all_MISSION_CLEAR_ALL_packMinBytes (3) //mmission_clear_all_MISSION_CLEAR_ALL.packMinBytes value as constant
extern Meta mmission_clear_all_MISSION_CLEAR_ALL;

#define mcamera_feedback_CAMERA_FEEDBACK_packMinBytes (45) //mcamera_feedback_CAMERA_FEEDBACK.packMinBytes value as constant
extern Meta mcamera_feedback_CAMERA_FEEDBACK;

#define mnav_controller_output_NAV_CONTROLLER_OUTPUT_packMinBytes (26) //mnav_controller_output_NAV_CONTROLLER_OUTPUT.packMinBytes value as constant
extern Meta mnav_controller_output_NAV_CONTROLLER_OUTPUT;

#define muavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_packMinBytes (1) //muavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.packMinBytes value as constant
extern Meta muavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;

#define mset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET_packMinBytes (43) //mset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET.packMinBytes value as constant
extern Meta mset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET;


typedef enum {
	e_PID_TUNING_AXIS_PID_TUNING_ROLL    = 1,
	e_PID_TUNING_AXIS_PID_TUNING_PITCH   = 2,
	e_PID_TUNING_AXIS_PID_TUNING_YAW     = 3,
	e_PID_TUNING_AXIS_PID_TUNING_ACCZ    = 4,
	e_PID_TUNING_AXIS_PID_TUNING_STEER   = 5,
	e_PID_TUNING_AXIS_PID_TUNING_LANDING = 6
}           e_PID_TUNING_AXIS;
/**
*Flags in EKF_STATUS message */
typedef enum {
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE           = 1,//True if the attitude estimate is good
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ     = 2,//True if the horizontal velocity estimate is good
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT      = 4,//True if the  vertical velocity estimate is good
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL      = 8,//True if the horizontal position (relative) estimate is good
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS      = 16,//True if the horizontal position (absolute) estimate is good
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS       = 32,//True if the vertical position (absolute) estimate is good
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL       = 64,//True if the vertical position (above ground) estimate is good
	/**
*True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical
*			 flow */
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE     = 128,
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL = 256,//True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimat
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS = 512,//True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimat
	e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH         = 1024//True if the EKF has detected a GPS glitch
}           e_ESTIMATOR_STATUS_FLAGS;

//Get enum by value
static inline e_ESTIMATOR_STATUS_FLAGS _en_estimator_status_flags(UMAX id) {
	switch (id) {
		case 0:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE;
		case 1:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ;
		case 2:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT;
		case 3:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL;
		case 4:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS;
		case 5:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS;
		case 6:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL;
		case 7:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE;
		case 8:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL;
		case 9:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS;
		case 10:return e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_estimator_status_flags(e_ESTIMATOR_STATUS_FLAGS en) {
	switch (en) {
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE:return 0;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ:return 1;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT:return 2;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL:return 3;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS:return 4;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS:return 5;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL:return 6;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE:return 7;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL:return 8;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS:return 9;
		case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH:return 10;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

typedef enum {
	e_MAV_TYPE_GENERIC            = 0,//Generic micro air vehicle.
	e_MAV_TYPE_FIXED_WING         = 1,//Fixed wing aircraft.
	e_MAV_TYPE_QUADROTOR          = 2,//Quadrotor
	e_MAV_TYPE_COAXIAL            = 3,//Coaxial helicopter
	e_MAV_TYPE_HELICOPTER         = 4,//Normal helicopter with tail rotor.
	e_MAV_TYPE_ANTENNA_TRACKER    = 5,//Ground installation
	e_MAV_TYPE_GCS                = 6,//Operator control unit / ground control station
	e_MAV_TYPE_AIRSHIP            = 7,//Airship, controlled
	e_MAV_TYPE_FREE_BALLOON       = 8,//Free balloon, uncontrolled
	e_MAV_TYPE_ROCKET             = 9,//Rocket
	e_MAV_TYPE_GROUND_ROVER       = 10,//Ground rover
	e_MAV_TYPE_SURFACE_BOAT       = 11,//Surface vessel, boat, ship
	e_MAV_TYPE_SUBMARINE          = 12,//Submarine
	e_MAV_TYPE_HEXAROTOR          = 13,//Hexarotor
	e_MAV_TYPE_OCTOROTOR          = 14,//Octorotor
	e_MAV_TYPE_TRICOPTER          = 15,//Tricopter
	e_MAV_TYPE_FLAPPING_WING      = 16,//Flapping wing
	e_MAV_TYPE_KITE               = 17,//Kite
	e_MAV_TYPE_ONBOARD_CONTROLLER = 18,//Onboard companion controller
	e_MAV_TYPE_VTOL_DUOROTOR      = 19,//Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
	e_MAV_TYPE_VTOL_QUADROTOR     = 20,//Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
	e_MAV_TYPE_VTOL_TILTROTOR     = 21,//Tiltrotor VTOL
	e_MAV_TYPE_VTOL_RESERVED2     = 22,//VTOL reserved 2
	e_MAV_TYPE_VTOL_RESERVED3     = 23,//VTOL reserved 3
	e_MAV_TYPE_VTOL_RESERVED4     = 24,//VTOL reserved 4
	e_MAV_TYPE_VTOL_RESERVED5     = 25,//VTOL reserved 5
	e_MAV_TYPE_GIMBAL             = 26,//Onboard gimbal
	e_MAV_TYPE_ADSB               = 27,//Onboard ADSB peripheral
	e_MAV_TYPE_PARAFOIL           = 28//Steerable, nonrigid airfoil
}           e_MAV_TYPE;
typedef enum {
	e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO      = 0,//Video mode
	e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO      = 1,//Photo mode
	e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_BURST      = 2,//Burst mode, hero 3+ only
	e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_TIME_LAPSE = 3,//Time lapse mode, hero 3+ only
	e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_MULTI_SHOT = 4,//Multi shot mode, hero 4 only
	e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PLAYBACK   = 5,//Playback mode, hero 4 only, silver only except when LCD or HDMI is connected to black
	e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_SETUP      = 6,//Playback mode, hero 4 only
	e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_UNKNOWN    = 255//Mode not yet known
}           e_GOPRO_CAPTURE_MODE;

//Get enum by value
static inline e_GOPRO_CAPTURE_MODE _en_gopro_capture_mode(UMAX id) {
	switch (id) {
		case 0:return e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO;
		case 1:return e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO;
		case 2:return e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_BURST;
		case 3:return e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_TIME_LAPSE;
		case 4:return e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_MULTI_SHOT;
		case 5:return e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PLAYBACK;
		case 6:return e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_SETUP;
		case 7:return e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_UNKNOWN;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_gopro_capture_mode(e_GOPRO_CAPTURE_MODE en) {
	switch (en) {
		case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO:return 0;
		case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO:return 1;
		case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_BURST:return 2;
		case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_TIME_LAPSE:return 3;
		case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_MULTI_SHOT:return 4;
		case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PLAYBACK:return 5;
		case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_SETUP:return 6;
		case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_UNKNOWN:return 7;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Micro air vehicle / autopilot classes. This identifies the individual model. */
typedef enum {
	e_MAV_AUTOPILOT_GENERIC                                      = 0,//Generic autopilot, full support for everything
	e_MAV_AUTOPILOT_RESERVED                                     = 1,//Reserved for future use.
	e_MAV_AUTOPILOT_SLUGS                                        = 2,//SLUGS autopilot, http:slugsuav.soe.ucsc.edu
	e_MAV_AUTOPILOT_ARDUPILOTMEGA                                = 3,//ArduPilotMega / ArduCopter, http:diydrones.com
	e_MAV_AUTOPILOT_OPENPILOT                                    = 4,//OpenPilot, http:openpilot.org
	e_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       = 5,//Generic autopilot only supporting simple waypoints
	e_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,//Generic autopilot supporting waypoints and other simple navigation commands
	e_MAV_AUTOPILOT_GENERIC_MISSION_FULL                         = 7,//Generic autopilot supporting the full mission command set
	e_MAV_AUTOPILOT_INVALID                                      = 8,//No valid autopilot, e.g. a GCS or other MAVLink component
	e_MAV_AUTOPILOT_PPZ                                          = 9,//PPZ UAV - http:nongnu.org/paparazzi
	e_MAV_AUTOPILOT_UDB                                          = 10,//UAV Dev Board
	e_MAV_AUTOPILOT_FP                                           = 11,//FlexiPilot
	e_MAV_AUTOPILOT_PX4                                          = 12,//PX4 Autopilot - http:pixhawk.ethz.ch/px4/
	e_MAV_AUTOPILOT_SMACCMPILOT                                  = 13,//SMACCMPilot - http:smaccmpilot.org
	e_MAV_AUTOPILOT_AUTOQUAD                                     = 14,//AutoQuad -- http:autoquad.org
	e_MAV_AUTOPILOT_ARMAZILA                                     = 15,//Armazila -- http:armazila.com
	e_MAV_AUTOPILOT_AEROB                                        = 16,//Aerob -- http:aerob.ru
	e_MAV_AUTOPILOT_ASLUAV                                       = 17,//ASLUAV autopilot -- http:www.asl.ethz.ch
	e_MAV_AUTOPILOT_SMARTAP                                      = 18//SmartAP Autopilot - http:sky-drones.com
}           e_MAV_AUTOPILOT;
/**
*Enumeration of battery functions */
typedef enum {
	e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN    = 0,//Battery function is unknown
	e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL        = 1,//Battery supports all flight systems
	e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION = 2,//Battery for the propulsion system
	e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS   = 3,//Avionics battery
	e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD        = 4//Payload battery
}           e_MAV_BATTERY_FUNCTION;
/**
*Type of landing target */
typedef enum {
	e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON    = 0,//Landing target signaled by light beacon (ex: IR-LOCK)
	e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON    = 1,//Landing target signaled by radio beacon (ex: ILS, NDB)
	e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2,//Landing target represented by a fiducial marker (ex: ARTag)
	e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER    = 3//Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
}           e_LANDING_TARGET_TYPE;
typedef enum {
	e_LIMIT_MODULE_LIMIT_GPSLOCK  = 1,//pre-initialization
	e_LIMIT_MODULE_LIMIT_GEOFENCE = 2,//disabled
	e_LIMIT_MODULE_LIMIT_ALTITUDE = 4//checking limits
}           e_LIMIT_MODULE;

//Get enum by value
static inline e_LIMIT_MODULE _en_limit_module(UMAX id) {
	switch (id) {
		case 0:return e_LIMIT_MODULE_LIMIT_GPSLOCK;
		case 1:return e_LIMIT_MODULE_LIMIT_GEOFENCE;
		case 2:return e_LIMIT_MODULE_LIMIT_ALTITUDE;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_limit_module(e_LIMIT_MODULE en) {
	switch (en) {
		case e_LIMIT_MODULE_LIMIT_GPSLOCK:return 0;
		case e_LIMIT_MODULE_LIMIT_GEOFENCE:return 1;
		case e_LIMIT_MODULE_LIMIT_ALTITUDE:return 2;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Enumeration of landed detector states */
typedef enum {
	e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED = 0,//MAV landed state is unknown
	e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND = 1,//MAV is landed (on ground)
	e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR    = 2,//MAV is in air
	e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF   = 3,//MAV currently taking off
	e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING   = 4//MAV currently landing
}           e_MAV_LANDED_STATE;
/**
*Specifies the datatype of a MAVLink parameter. */
typedef enum {
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8  = 1,//8-bit unsigned integer
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8   = 2,//8-bit signed integer
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16 = 3,//16-bit unsigned integer
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16  = 4,//16-bit signed integer
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32 = 5,//32-bit unsigned integer
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32  = 6,//32-bit signed integer
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64 = 7,//64-bit unsigned integer
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64  = 8,//64-bit signed integer
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32 = 9,//32-bit floating-point
	e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64 = 10//64-bit floating-point
}           e_MAV_PARAM_TYPE;
/**
*Emergency status encoding */
typedef enum {
	e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_NO_EMERGENCY                    = 0,
	e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY               = 1,
	e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY             = 2,
	e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY          = 3,
	e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY               = 4,
	e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY = 5,
	e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY       = 6,
	e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_RESERVED                        = 7
}           e_UAVIONIX_ADSB_EMERGENCY_STATUS;
/**
*Indicates the severity level, generally used for status messages to indicate their relative urgency. Based
*		 on RFC-5424 using expanded definitions at: http:www.kiwisyslog.com/kb/info:-syslog-message-levels/ */
typedef enum {
	e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY = 0,//System is unusable. This is a "panic" condition.
	e_MAV_SEVERITY_MAV_SEVERITY_ALERT     = 1,//Action should be taken immediately. Indicates error in non-critical systems.
	e_MAV_SEVERITY_MAV_SEVERITY_CRITICAL  = 2,//Action must be taken immediately. Indicates failure in a primary system.
	e_MAV_SEVERITY_MAV_SEVERITY_ERROR     = 3,//Indicates an error in secondary/redundant systems.
	/**
*Indicates about a possible future error if this is not resolved within a given timeframe. Example would
*			 be a low battery warning */
	e_MAV_SEVERITY_MAV_SEVERITY_WARNING   = 4,
	/**
*An unusual event has occured, though not an error condition. This should be investigated for the root
*			 cause */
	e_MAV_SEVERITY_MAV_SEVERITY_NOTICE    = 5,
	e_MAV_SEVERITY_MAV_SEVERITY_INFO      = 6,//Normal operational messages. Useful for logging. No action is required for these messages.
	e_MAV_SEVERITY_MAV_SEVERITY_DEBUG     = 7//Useful non-operational messages that can assist in debugging. These should not occur during normal operation
}           e_MAV_SEVERITY;
/**
*These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
*		 simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. */
typedef enum {
	e_MAV_MODE_PREFLIGHT              = 0,//System is not ready to fly, booting, calibrating, etc. No flag is set.
	e_MAV_MODE_MANUAL_DISARMED        = 64,//System is allowed to be active, under manual (RC) control, no stabilization
	e_MAV_MODE_MAV_MODE_TEST_DISARMED = 66,//UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
	e_MAV_MODE_STABILIZE_DISARMED     = 80,//System is allowed to be active, under assisted RC control.
	e_MAV_MODE_GUIDED_DISARMED        = 88,//System is allowed to be active, under autonomous control, manual setpoint
	/**
*System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
*			 and not pre-programmed by waypoints */
	e_MAV_MODE_MAV_MODE_AUTO_DISARMED = 92,
	e_MAV_MODE_MANUAL_ARMED           = 192,//System is allowed to be active, under manual (RC) control, no stabilization
	e_MAV_MODE_MAV_MODE_TEST_ARMED    = 194,//UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
	e_MAV_MODE_STABILIZE_ARMED        = 208,//System is allowed to be active, under assisted RC control.
	e_MAV_MODE_GUIDED_ARMED           = 216,//System is allowed to be active, under autonomous control, manual setpoint
	/**
*System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
*			 and not pre-programmed by waypoints */
	e_MAV_MODE_MAV_MODE_AUTO_ARMED    = 220
}           e_MAV_MODE;

//Get enum by value
static inline e_MAV_MODE _en_mav_mode(UMAX id) {
	switch (id) {
		case 0:return e_MAV_MODE_PREFLIGHT;
		case 1:return e_MAV_MODE_MANUAL_DISARMED;
		case 2:return e_MAV_MODE_MAV_MODE_TEST_DISARMED;
		case 3:return e_MAV_MODE_STABILIZE_DISARMED;
		case 4:return e_MAV_MODE_GUIDED_DISARMED;
		case 5:return e_MAV_MODE_MAV_MODE_AUTO_DISARMED;
		case 6:return e_MAV_MODE_MANUAL_ARMED;
		case 7:return e_MAV_MODE_MAV_MODE_TEST_ARMED;
		case 8:return e_MAV_MODE_STABILIZE_ARMED;
		case 9:return e_MAV_MODE_GUIDED_ARMED;
		case 10:return e_MAV_MODE_MAV_MODE_AUTO_ARMED;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_mav_mode(e_MAV_MODE en) {
	switch (en) {
		case e_MAV_MODE_PREFLIGHT:return 0;
		case e_MAV_MODE_MANUAL_DISARMED:return 1;
		case e_MAV_MODE_MAV_MODE_TEST_DISARMED:return 2;
		case e_MAV_MODE_STABILIZE_DISARMED:return 3;
		case e_MAV_MODE_GUIDED_DISARMED:return 4;
		case e_MAV_MODE_MAV_MODE_AUTO_DISARMED:return 5;
		case e_MAV_MODE_MANUAL_ARMED:return 6;
		case e_MAV_MODE_MAV_MODE_TEST_ARMED:return 7;
		case e_MAV_MODE_STABILIZE_ARMED:return 8;
		case e_MAV_MODE_GUIDED_ARMED:return 9;
		case e_MAV_MODE_MAV_MODE_AUTO_ARMED:return 10;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Enumeration of the ADSB altimeter types */
typedef enum {
	e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0,//Altitude reported from a Baro source using QNH reference
	e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC    = 1//Altitude reported from a GNSS source
}           e_ADSB_ALTITUDE_TYPE;
/**
*Type of mission items being requested/sent in mission protocol. */
typedef enum {
	e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = 0,//Items are mission commands for main mission.
	e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE   = 1,//Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.
	/**
*Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT
*			 rally point items */
	e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY   = 2,
	e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL     = 255//Only used in MISSION_CLEAR_ALL to clear all mission types.
}           e_MAV_MISSION_TYPE;

//Get enum by value
static inline e_MAV_MISSION_TYPE _en_mav_mission_type(UMAX id) {
	switch (id) {
		case 0:return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
		case 1:return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
		case 2:return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
		case 3:return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_mav_mission_type(e_MAV_MISSION_TYPE en) {
	switch (en) {
		case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION:return 0;
		case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE:return 1;
		case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY:return 2;
		case e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL:return 3;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Enumeration of distance sensor types */
typedef enum {
	e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER      = 0,//Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
	e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND = 1,//Ultrasound rangefinder, e.g. MaxBotix units
	e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED   = 2,//Infrared rangefinder, e.g. Sharp units
	e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR      = 3,//Radar type, e.g. uLanding units
	e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN    = 4//Broken or unknown type, e.g. analog units
}           e_MAV_DISTANCE_SENSOR;
/**
*Enumeration of VTOL states */
typedef enum {
	e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED        = 0,//MAV is not configured as VTOL
	e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW = 1,//VTOL is in transition from multicopter to fixed-wing
	e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC = 2,//VTOL is in transition from fixed-wing to multicopter
	e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC               = 3,//VTOL is in multicopter state
	e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW               = 4//VTOL is in fixed-wing state
}           e_MAV_VTOL_STATE;
/**
*Type of GPS fix */
typedef enum {
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS    = 0,//No GPS connected
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX    = 1,//No position information, GPS is connected
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX    = 2,//2D position
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX    = 3,//3D position
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS      = 4,//DGPS/SBAS aided 3D position
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT = 5,//RTK float, 3D position
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED = 6,//RTK Fixed, 3D position
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC    = 7,//Static fixed, typically used for base stations
	e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP       = 8//PPP, 3D position.
}           e_GPS_FIX_TYPE;
/**
*Specifies the datatype of a MAVLink extended parameter. */
typedef enum {
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8  = 1,//8-bit unsigned integer
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8   = 2,//8-bit signed integer
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16 = 3,//16-bit unsigned integer
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16  = 4,//16-bit signed integer
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32 = 5,//32-bit unsigned integer
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32  = 6,//32-bit signed integer
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64 = 7,//64-bit unsigned integer
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64  = 8,//64-bit signed integer
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32 = 9,//32-bit floating-point
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64 = 10,//64-bit floating-point
	e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM = 11//Custom Type
}           e_MAV_PARAM_EXT_TYPE;
/**
*Enumeration of estimator types */
typedef enum {
	e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE   = 1,//This is a naive estimator without any real covariance feedback.
	e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION  = 2,//Computer vision based estimate. Might be up to scale.
	e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO     = 3,//Visual-inertial estimate.
	e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS     = 4,//Plain GPS estimate.
	e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS = 5//Estimator integrating GPS and inertial sensing.
}           e_MAV_ESTIMATOR_TYPE;
typedef enum {
	e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_PHOTO       = 0,//Shooting photos, not video
	e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_VIDEO       = 1,//Shooting video, not stills
	e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_BADEXPOSURE = 2,//Unable to achieve requested exposure (e.g. shutter speed too low)
	e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_CLOSEDLOOP  = 3,//Closed loop feedback from camera, we know for sure it has successfully taken a picture
	/**
*Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken
*			 a pictur */
	e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_OPENLOOP    = 4
}           e_CAMERA_FEEDBACK_FLAGS;
/**
*Generalized UAVCAN node health */
typedef enum {
	e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK       = 0,//The node is functioning properly.
	e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING  = 1,//A critical parameter went out of range or the node has encountered a minor failure.
	e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR    = 2,//The node has encountered a major failure.
	e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL = 3//The node has suffered a fatal malfunction.
}           e_UAVCAN_NODE_HEALTH;
/**
*Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script.
*		 If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows:
*		 Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what
*		 ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data */
typedef enum {
	/**
*Navigate to waypoint.
*					 1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
*					 2	Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
*					 3	0 to pass through the WP, if 	>	0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
*					 4	Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
*					 5	Latitude
*					 6	Longitude
*					 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_WAYPOINT                       = 16,
	/**
*Loiter around this waypoint an unlimited amount of time
*			 1	Empty
*			 2	Empty
*			 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
*			 4	Desired yaw angle.
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM                   = 17,
	/**
*Loiter around this waypoint for X turns
*			 1	Turns
*			 2	Empty
*			 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
*			 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS                   = 18,
	/**
*Loiter around this waypoint for X seconds
*			 1	Seconds (decimal)
*			 2	Empty
*			 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
*			 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME                    = 19,
	/**
*Return to launch location
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH               = 20,
	/**
*Land at location
*			 1	Abort Alt
*			 2	Empty
*			 3	Empty
*			 4	Desired yaw angle. NaN for unchanged.
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude (ground level) */
	e_MAV_CMD_MAV_CMD_NAV_LAND                           = 21,
	/**
*Takeoff from ground / hand
*			 1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
*			 2	Empty
*			 3	Empty
*			 4	Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_TAKEOFF                        = 22,
	/**
*Land at local position (local frame only)
*			 1	Landing target number (if available)
*			 2	Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
*			 3	Landing descend rate [ms^-1]
*			 4	Desired yaw angle [rad]
*			 5	Y-axis position [m]
*			 6	X-axis position [m]
*			 7	Z-axis / ground level position [m] */
	e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL                     = 23,
	/**
*Takeoff from local position (local frame only)
*			 1	Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
*			 2	Empty
*			 3	Takeoff ascend rate [ms^-1]
*			 4	Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
*			 5	Y-axis position [m]
*			 6	X-axis position [m]
*			 7	Z-axis position [m] */
	e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL                  = 24,
	/**
*Vehicle following, i.e. this waypoint represents the position of a moving vehicle
*			 1	Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
*			 2	Ground speed of vehicle to be followed
*			 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
*			 4	Desired yaw angle.
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_FOLLOW                         = 25,
	/**
*Continue on the current course and climb/descend to specified altitude.  When the altitude is reached
*			 continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached
*			 1	Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Desired altitude in meters */
	e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT        = 30,
	/**
*Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.
*			 Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.
*			 Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter
*			 until heading toward the next waypoint.
*			 1	Heading Required (0 = False)
*			 2	Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
*			 3	Empty
*			 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT                  = 31,
	/**
*Being following a target
*			 1	System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode
*			 2	RESERVED
*			 3	RESERVED
*			 4	altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home
*			 5	altitude
*			 6	RESERVED
*			 7	TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout */
	e_MAV_CMD_MAV_CMD_DO_FOLLOW                          = 32,
	/**
*Reposition the MAV after a follow target command has been sent
*			 1	Camera q1 (where 0 is on the ray from the camera to the tracking device)
*			 2	Camera q2
*			 3	Camera q3
*			 4	Camera q4
*			 5	altitude offset from target (m)
*			 6	X offset from target (m)
*			 7	Y offset from target (m) */
	e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION               = 33,
	/**
*Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
*			 vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
*			 1	Region of intereset mode. (see MAV_ROI enum)
*			 2	Waypoint index/ target ID. (see MAV_ROI enum)
*			 3	ROI index (allows a vehicle to manage multiple ROI's)
*			 4	Empty
*			 5	x the location of the fixed ROI (see MAV_FRAME)
*			 6	y
*			 7	z */
	e_MAV_CMD_MAV_CMD_NAV_ROI                            = 80,
	/**
*Control autonomous path planning on the MAV.
*			 1	0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
*			 2	0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
*			 3	Empty
*			 4	Yaw angle at goal, in compass degrees, [0..360]
*			 5	Latitude/X of goal
*			 6	Longitude/Y of goal
*			 7	Altitude/Z of goal */
	e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING                   = 81,
	/**
*Navigate to waypoint using a spline path.
*			 1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Latitude/X of goal
*			 6	Longitude/Y of goal
*			 7	Altitude/Z of goal */
	e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT                = 82,
	/**
*Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon
*			 launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical
*			 speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control
*			 surfaces to prevent them seizing up
*			 1	altitude (m)
*			 2	descent speed (m/s)
*			 3	Wiggle Time (s)
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT                  = 83,
	/**
*Takeoff from ground using VTOL mode
*			 1	Empty
*			 2	Front transition heading, see VTOL_TRANSITION_HEADING enum.
*			 3	Empty
*			 4	Yaw angle in degrees. NaN for unchanged.
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF                   = 84,
	/**
*Land using VTOL mode
*			 1	Empty
*			 2	Empty
*			 3	Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
*			 4	Yaw angle in degrees. NaN for unchanged.
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude (ground level) */
	e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND                      = 85,
	/**
*hand control over to an external controller
*			 1	On / Off (	>	0.5f on)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE                  = 92,
	/**
*Delay the next navigation command a number of seconds or until a specified time
*			 1	Delay in seconds (decimal, -1 to enable time-of-day fields)
*			 2	hour (24h format, UTC, -1 to ignore)
*			 3	minute (24h format, UTC, -1 to ignore)
*			 4	second (24h format, UTC)
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_NAV_DELAY                          = 93,
	/**
*Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground,
*			 the gripper is opened to release the payloa
*			 1	Maximum distance to descend (meters)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Latitude (deg * 1E7)
*			 6	Longitude (deg * 1E7)
*			 7	Altitude (meters) */
	e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE                  = 94,
	/**
*NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeratio
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_NAV_LAST                           = 95,
	/**
*Delay mission state machine.
*			 1	Delay in seconds (decimal)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_CONDITION_DELAY                    = 112,
	/**
*Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
*			 1	Descent / Ascend rate (m/s)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Finish Altitude */
	e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT               = 113,
	/**
*Delay mission state machine until within desired distance of next NAV point.
*			 1	Distance (meters)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE                 = 114,
	/**
*Reach a certain target angle.
*			 1	target angle: [0-360], 0 is north
*			 2	speed during yaw change:[deg per second]
*			 3	direction: negative: counter clockwise, positive: clockwise [-1,1]
*			 4	relative offset or absolute angle: [ 1,0]
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_CONDITION_YAW                      = 115,
	/**
*NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeratio
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_CONDITION_LAST                     = 159,
	/**
*Set system mode.
*			 1	Mode, as defined by ENUM MAV_MODE
*			 2	Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
*			 3	Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_SET_MODE                        = 176,
	/**
*Jump to the desired command in the mission list.  Repeat this action only the specified number of time
*			 1	Sequence number
*			 2	Repeat count
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_JUMP                            = 177,
	/**
*Change speed and/or throttle set points.
*			 1	Speed type (0=Airspeed, 1=Ground Speed)
*			 2	Speed  (m/s, -1 indicates no change)
*			 3	Throttle  ( Percent, -1 indicates no change)
*			 4	absolute or relative [0,1]
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED                    = 178,
	/**
*Changes the home location either to the current location or a specified location.
*			 1	Use current (1=use current location, 0=use specified location)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_DO_SET_HOME                        = 179,
	/**
*Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value
*			 of the parameter
*			 1	Parameter number
*			 2	Parameter value
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER                   = 180,
	/**
*Set a relay to a condition.
*			 1	Relay number
*			 2	Setting (1=on, 0=off, others possible depending on system hardware)
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_SET_RELAY                       = 181,
	/**
*Cycle a relay on and off for a desired number of cyles with a desired period.
*			 1	Relay number
*			 2	Cycle count
*			 3	Cycle time (seconds, decimal)
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY                    = 182,
	/**
*Set a servo to a desired PWM value.
*			 1	Servo number
*			 2	PWM (microseconds, 1000 to 2000 typical)
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_SET_SERVO                       = 183,
	/**
*Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period
*			 1	Servo number
*			 2	PWM (microseconds, 1000 to 2000 typical)
*			 3	Cycle count
*			 4	Cycle time (seconds)
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO                    = 184,
	/**
*Terminate flight immediately
*			 1	Flight termination activated if 	>	0.5
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION               = 185,
	/**
*Change altitude set point.
*			 1	Altitude in meters
*			 2	Mav frame of new altitude (see MAV_FRAME)
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE                 = 186,
	/**
*Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where
*			 a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG
*			 to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will
*			 be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it
*			 will be used to help find the closest landing sequence
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Latitude
*			 6	Longitude
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_LAND_START                      = 189,
	/**
*Mission command to perform a landing from a rally point.
*			 1	Break altitude (meters)
*			 2	Landing speed (m/s)
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_RALLY_LAND                      = 190,
	/**
*Mission command to safely abort an autonmous landing.
*			 1	Altitude (meters)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_GO_AROUND                       = 191,
	/**
*Reposition the vehicle to a specific WGS84 global position.
*			 1	Ground speed, less than 0 (-1) for default
*			 2	Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.
*			 3	Reserved
*			 4	Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
*			 5	Latitude (deg * 1E7)
*			 6	Longitude (deg * 1E7)
*			 7	Altitude (meters) */
	e_MAV_CMD_MAV_CMD_DO_REPOSITION                      = 192,
	/**
*If in a GPS controlled position mode, hold the current position or continue.
*			 1	0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Reserved
*			 6	Reserved
*			 7	Reserved */
	e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE                  = 193,
	/**
*Set moving direction to forward or reverse.
*			 1	Direction (0=Forward, 1=Reverse)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_SET_REVERSE                     = 194,
	/**
*Control onboard camera system.
*			 1	Camera ID (-1 for all)
*			 2	Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
*			 3	Transmission mode: 0: video stream, 	>	0: single images every n seconds (decimal)
*			 4	Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO                   = 200,
	/**
*Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
*			 vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
*			 1	Region of intereset mode. (see MAV_ROI enum)
*			 2	Waypoint index/ target ID. (see MAV_ROI enum)
*			 3	ROI index (allows a vehicle to manage multiple ROI's)
*			 4	Empty
*			 5	MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude
*			 6	MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude
*			 7	MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude */
	e_MAV_CMD_MAV_CMD_DO_SET_ROI                         = 201,
	/**
*Mission command to configure an on-board camera controller system.
*			 1	Modes: P, TV, AV, M, Etc
*			 2	Shutter speed: Divisor number for one second
*			 3	Aperture: F stop number
*			 4	ISO number e.g. 80, 100, 200, Etc
*			 5	Exposure type enumerator
*			 6	Command Identity
*			 7	Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off) */
	e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE               = 202,
	/**
*Mission command to control an on-board camera controller system.
*			 1	Session control e.g. show/hide lens
*			 2	Zoom's absolute position
*			 3	Zooming step value to offset zoom from the current position
*			 4	Focus Locking, Unlocking or Re-locking
*			 5	Shooting Command
*			 6	Command Identity
*			 7	Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count. */
	e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL                 = 203,
	/**
*Mission command to configure a camera or antenna mount
*			 1	Mount operation mode (see MAV_MOUNT_MODE enum)
*			 2	stabilize roll? (1 = yes, 0 = no)
*			 3	stabilize pitch? (1 = yes, 0 = no)
*			 4	stabilize yaw? (1 = yes, 0 = no)
*			 5	roll input (0 = angle, 1 = angular rate)
*			 6	pitch input (0 = angle, 1 = angular rate)
*			 7	yaw input (0 = angle, 1 = angular rate) */
	e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE                 = 204,
	/**
*Mission command to control a camera or antenna mount
*			 1	pitch depending on mount mode (degrees or degrees/second depending on pitch input).
*			 2	roll depending on mount mode (degrees or degrees/second depending on roll input).
*			 3	yaw depending on mount mode (degrees or degrees/second depending on yaw input).
*			 4	alt in meters depending on mount mode.
*			 5	latitude in degrees * 1E7, set if appropriate mount mode.
*			 6	longitude in degrees * 1E7, set if appropriate mount mode.
*			 7	MAV_MOUNT_MODE enum value */
	e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL                   = 205,
	/**
*Mission command to set camera trigger distance for this flight. The camera is trigerred each time this
*			 distance is exceeded. This command can also be used to set the shutter integration time for the camera
*			 1	Camera trigger distance (meters). 0 to stop triggering.
*			 2	Camera shutter integration time (milliseconds). -1 or 0 to ignore
*			 3	Trigger camera once immediately. (0 = no trigger, 1 = trigger)
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST              = 206,
	/**
*Mission command to enable the geofence
*			 1	enable? (0=disable, 1=enable, 2=disable_floor_only)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE                    = 207,
	/**
*Mission command to trigger a parachute
*			 1	action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_PARACHUTE                       = 208,
	/**
*Mission command to perform motor test
*			 1	motor sequence number (a number from 1 to max number of motors on the vehicle)
*			 2	throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
*			 3	throttle
*			 4	timeout (in seconds)
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST                      = 209,
	/**
*Change to/from inverted flight
*			 1	inverted (0=normal, 1=inverted)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT                 = 210,
	/**
*Mission command to operate EPM gripper
*			 1	gripper number (a number from 1 to max number of grippers on the vehicle)
*			 2	gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum)
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_GRIPPER                         = 211,
	/**
*Enable/disable autotune
*			 1	enable (1: enable, 0:disable)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE                 = 212,
	/**
*Sets a desired vehicle turn angle and speed change
*			 1	yaw angle to adjust steering by in centidegress
*			 2	speed - normalized to 0 .. 1
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED                  = 213,
	/**
*Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is
*			 triggered each time this interval expires. This command can also be used to set the shutter integration
*			 time for the camera
*			 1	Camera trigger cycle time (milliseconds). -1 or 0 to ignore.
*			 2	Camera shutter integration time (milliseconds). Should be less than trigger cycle time. -1 or 0 to ignore.
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL          = 214,
	/**
*Mission command to control a camera or antenna mount, using a quaternion as reference.
*			 1	q1 - quaternion param #1, w (1 in null-rotation)
*			 2	q2 - quaternion param #2, x (0 in null-rotation)
*			 3	q3 - quaternion param #3, y (0 in null-rotation)
*			 4	q4 - quaternion param #4, z (0 in null-rotation)
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT              = 220,
	/**
*set id of master controller
*			 1	System ID
*			 2	Component ID
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER                   = 221,
	/**
*set limits for external control
*			 1	timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout
*			 2	absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit
*			 3	absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit
*			 4	horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS                   = 222,
	/**
*Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine
*			 state. It is intended for vehicles with internal combustion engine
*			 1	0: Stop engine, 1:Start Engine
*			 2	0: Warm start, 1:Cold start. Controls use of choke where applicable
*			 3	Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
*			 4	Empty
*			 5	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL                  = 223,
	/**
*NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_LAST                            = 240,
	/**
*Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature
*			 Calibration, only one sensor should be set in a single message and all others should be zero
*			 1	1: gyro calibration, 3: gyro temperature calibration
*			 2	1: magnetometer calibration
*			 3	1: ground pressure calibration
*			 4	1: radio RC calibration, 2: RC trim calibration
*			 5	1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration
*			 6	1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
*			 7	1: ESC calibration, 3: barometer temperature calibration */
	e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION              = 241,
	/**
*Set sensor offsets. This command will be only accepted if in pre-flight mode.
*			 1	Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
*			 2	X axis offset (or generic dimension 1), in the sensor's raw units
*			 3	Y axis offset (or generic dimension 2), in the sensor's raw units
*			 4	Z axis offset (or generic dimension 3), in the sensor's raw units
*			 5	Generic dimension 4, in the sensor's raw units
*			 6	Generic dimension 5, in the sensor's raw units
*			 7	Generic dimension 6, in the sensor's raw units */
	e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS       = 242,
	/**
*Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
*			 1	1: Trigger actuator ID assignment and direction mapping.
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Reserved
*			 6	Reserved
*			 7	Reserved */
	e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN                   = 243,
	/**
*Request storage of different parameter values and logs. This command will be only accepted if in pre-flight
*			 mode
*			 1	Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
*			 2	Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
*			 3	Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, 	>	1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)
*			 4	Reserved
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE                  = 245,
	/**
*Request the reboot or shutdown of system components.
*			 1	0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
*			 2	0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
*			 3	WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded
*			 4	WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded
*			 5	Reserved, send 0
*			 6	Reserved, send 0
*			 7	WIP: ID (e.g. camera ID -1 for all IDs) */
	e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN          = 246,
	/**
*Hold / continue the current action
*			 1	MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan
*			 2	MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position
*			 3	MAV_FRAME coordinate frame of hold point
*			 4	Desired yaw angle in degrees
*			 5	Latitude / X position
*			 6	Longitude / Y position
*			 7	Altitude / Z position */
	e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO                      = 252,
	/**
*start running a mission
*			 1	first_item: the first mission item to run
*			 2	last_item:  the last mission item to run (after this item is run, the mission ends) */
	e_MAV_CMD_MAV_CMD_MISSION_START                      = 300,
	/**
*Arms / Disarms a component
*			 1	1 to arm, 0 to disarm */
	e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM               = 400,
	/**
*Request the home position from the vehicle.
*			 1	Reserved
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Reserved
*			 6	Reserved
*			 7	Reserved */
	e_MAV_CMD_MAV_CMD_GET_HOME_POSITION                  = 410,
	/**
*Starts receiver pairing
*			 1	0:Spektrum
*			 2	0:Spektrum DSM2, 1:Spektrum DSMX */
	e_MAV_CMD_MAV_CMD_START_RX_PAIR                      = 500,
	/**
*Request the interval between messages for a particular MAVLink message ID
*			 1	The MAVLink message ID */
	e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL               = 510,
	/**
*Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREA
*			 1	The MAVLink message ID
*			 2	The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate. */
	e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL               = 511,
	/**
*Request MAVLink protocol version compatibility
*			 1	1: Request supported protocol versions by all nodes on the network
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION           = 519,
	/**
*Request autopilot capabilities
*			 1	1: Request autopilot version
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES     = 520,
	/**
*WIP: Request camera information (CAMERA_INFORMATION).
*			 1	0: No action 1: Request camera capabilities
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION         = 521,
	/**
*WIP: Request camera settings (CAMERA_SETTINGS).
*			 1	0: No Action 1: Request camera settings
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS            = 522,
	/**
*WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a
*			 specific component's storage
*			 1	Storage ID (0 for all, 1 for first, 2 for second, etc.)
*			 2	0: No Action 1: Request storage information
*			 3	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION        = 525,
	/**
*WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the
*			 command's target_component to target a specific component's storage
*			 1	Storage ID (1 for first, 2 for second, etc.)
*			 2	0: No action 1: Format storage
*			 3	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_STORAGE_FORMAT                     = 526,
	/**
*WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
*			 1	0: No Action 1: Request camera capture status
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS      = 527,
	/**
*WIP: Request flight information (FLIGHT_INFORMATION)
*			 1	1: Request flight information
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION         = 528,
	/**
*WIP: Reset all camera settings to Factory Default
*			 1	0: No Action 1: Reset all settings
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS              = 529,
	/**
*Set camera running mode. Use NAN for reserved values.
*			 1	Reserved (Set to 0)
*			 2	Camera mode (see CAMERA_MODE enum)
*			 3	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE                    = 530,
	/**
*Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values
*			 1	Reserved (Set to 0)
*			 2	Duration between two consecutive pictures (in seconds)
*			 3	Number of images to capture total - 0 for unlimited capture
*			 4	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE                = 2000,
	/**
*Stop image capture sequence Use NAN for reserved values.
*			 1	Reserved (Set to 0)
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE                 = 2001,
	/**
*WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values.
*			 1	Sequence number for missing CAMERA_IMAGE_CAPTURE packet
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE       = 2002,
	/**
*Enable or disable on-board camera triggering system.
*			 1	Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
*			 2	1 to reset the trigger sequence, -1 or 0 to ignore
*			 3	1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore */
	e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL                 = 2003,
	/**
*Starts video capture (recording). Use NAN for reserved values.
*			 1	Reserved (Set to 0)
*			 2	Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)
*			 3	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE                = 2500,
	/**
*Stop the current video capture (recording). Use NAN for reserved values.
*			 1	Reserved (Set to 0)
*			 2	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE                 = 2501,
	/**
*WIP: Start video streaming
*			 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
*			 2	Reserved */
	e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING              = 2502,
	/**
*WIP: Stop the current video streaming
*			 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
*			 2	Reserved */
	e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING               = 2503,
	/**
*WIP: Request video stream information (VIDEO_STREAM_INFORMATION)
*			 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
*			 2	0: No Action 1: Request video stream information
*			 3	Reserved (all remaining params) */
	e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION   = 2504,
	/**
*Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
*			 1	Format: 0: ULog
*			 2	Reserved (set to 0)
*			 3	Reserved (set to 0)
*			 4	Reserved (set to 0)
*			 5	Reserved (set to 0)
*			 6	Reserved (set to 0)
*			 7	Reserved (set to 0) */
	e_MAV_CMD_MAV_CMD_LOGGING_START                      = 2510,
	/**
*Request to stop streaming log data over MAVLink
*			 1	Reserved (set to 0)
*			 2	Reserved (set to 0)
*			 3	Reserved (set to 0)
*			 4	Reserved (set to 0)
*			 5	Reserved (set to 0)
*			 6	Reserved (set to 0)
*			 7	Reserved (set to 0) */
	e_MAV_CMD_MAV_CMD_LOGGING_STOP                       = 2511,
	/**
*1	Landing gear ID (default: 0, -1 for all)
*			 2	Landing gear position (Down: 0, Up: 1, NAN for no change)
*			 3	Reserved, set to NAN
*			 4	Reserved, set to NAN
*			 5	Reserved, set to NAN
*			 6	Reserved, set to NAN
*			 7	Reserved, set to NAN */
	e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION             = 2520,
	/**
*Create a panorama at the current position
*			 1	Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)
*			 2	Viewing angle vertical of panorama (in degrees)
*			 3	Speed of the horizontal rotation (in degrees per second)
*			 4	Speed of the vertical rotation (in degrees per second) */
	e_MAV_CMD_MAV_CMD_PANORAMA_CREATE                    = 2800,
	/**
*Request VTOL transition
*			 1	The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used. */
	e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION                 = 3000,
	/**
*Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
*			 <p>
*			 1	Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle */
	e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST          = 3001,
	/**
*This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes. */
	e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD        = 4000,
	/**
*This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
*			 <p>
*			 1	Radius of desired circle in CIRCLE_MODE
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Unscaled target latitude of center of circle in CIRCLE_MODE
*			 6	Unscaled target longitude of center of circle in CIRCLE_MODE */
	e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE          = 4001,
	/**
*WIP: Delay mission state machine until gate has been reached.
*			 1	Geometry: 0: orthogonal to path between previous and next waypoint.
*			 2	Altitude: 0: ignore altitude
*			 3	Empty
*			 4	Empty
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_CONDITION_GATE                     = 4501,
	/**
*Fence return point. There can only be one fence return point.
*			 <p>
*			 1	Reserved
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT             = 5000,
	/**
*Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
*			 <p>
*			 1	Polygon vertex count
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Latitude
*			 6	Longitude
*			 7	Reserved */
	e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
	/**
*Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
*			 <p>
*			 1	Polygon vertex count
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Latitude
*			 6	Longitude
*			 7	Reserved */
	e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
	/**
*Circular fence area. The vehicle must stay inside this area.
*			 <p>
*			 1	radius in meters
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Latitude
*			 6	Longitude
*			 7	Reserved */
	e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION         = 5003,
	/**
*Circular fence area. The vehicle must stay outside this area.
*			 <p>
*			 1	radius in meters
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Latitude
*			 6	Longitude
*			 7	Reserved */
	e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION         = 5004,
	/**
*Rally point. You can have multiple rally points defined.
*			 <p>
*			 1	Reserved
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Latitude
*			 6	Longitude
*			 7	Altitude */
	e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT                    = 5100,
	/**
*Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN
*			 node that is online. Note that some of the response messages can be lost, which the receiver can detect
*			 easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO
*			 received earlier; if not, this command should be sent again in order to request re-transmission of the
*			 node information messages
*			 1	Reserved (set to 0)
*			 2	Reserved (set to 0)
*			 3	Reserved (set to 0)
*			 4	Reserved (set to 0)
*			 5	Reserved (set to 0)
*			 6	Reserved (set to 0)
*			 7	Reserved (set to 0) */
	e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO               = 5200,
	/**
*Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release
*			 position and velocity
*			 1	Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
*			 2	Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.
*			 3	Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
*			 4	Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.
*			 5	Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
*			 6	Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY             = 30001,
	/**
*Control the payload deployment.
*			 1	Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
*			 2	Reserved
*			 3	Reserved
*			 4	Reserved
*			 5	Reserved
*			 6	Reserved
*			 7	Reserved */
	e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY             = 30002,
	/**
*User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1                    = 31000,
	/**
*User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2                    = 31001,
	/**
*User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3                    = 31002,
	/**
*User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4                    = 31003,
	/**
*User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5                    = 31004,
	/**
*User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
*			 ROI item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_SPATIAL_USER_1                     = 31005,
	/**
*User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
*			 ROI item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_SPATIAL_USER_2                     = 31006,
	/**
*User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
*			 ROI item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_SPATIAL_USER_3                     = 31007,
	/**
*User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
*			 ROI item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_SPATIAL_USER_4                     = 31008,
	/**
*User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
*			 ROI item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	Latitude unscaled
*			 6	Longitude unscaled
*			 7	Altitude, in meters AMSL */
	e_MAV_CMD_MAV_CMD_SPATIAL_USER_5                     = 31009,
	/**
*User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
*			 item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	User defined
*			 6	User defined
*			 7	User defined */
	e_MAV_CMD_MAV_CMD_USER_1                             = 31010,
	/**
*User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
*			 item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	User defined
*			 6	User defined
*			 7	User defined */
	e_MAV_CMD_MAV_CMD_USER_2                             = 31011,
	/**
*User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
*			 item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	User defined
*			 6	User defined
*			 7	User defined */
	e_MAV_CMD_MAV_CMD_USER_3                             = 31012,
	/**
*User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
*			 item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	User defined
*			 6	User defined
*			 7	User defined */
	e_MAV_CMD_MAV_CMD_USER_4                             = 31013,
	/**
*User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
*			 item
*			 1	User defined
*			 2	User defined
*			 3	User defined
*			 4	User defined
*			 5	User defined
*			 6	User defined
*			 7	User defined */
	e_MAV_CMD_MAV_CMD_USER_5                             = 31014,
	/**
*A system wide power-off event has been initiated.
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED                = 42000,
	/**
*FLY button has been clicked.
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK                 = 42001,
	/**
*FLY button has been held for 1.5 seconds.
*			 1	Takeoff altitude
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD                  = 42002,
	/**
*PAUSE button has been clicked.
*			 1	1 if Solo is in a shot mode, 0 otherwise
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK               = 42003,
	/**
*Initiate a magnetometer calibration
*			 1	uint8_t bitmask of magnetometers (0 means all)
*			 2	Automatically retry on failure (0=no retry, 1=retry).
*			 3	Save without user input (0=require input, 1=autosave).
*			 4	Delay (seconds)
*			 5	Autoreboot (0=user reboot, 1=autoreboot)
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL                   = 42424,
	/**
*Initiate a magnetometer calibration
*			 1	uint8_t bitmask of magnetometers (0 means all)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL                  = 42425,
	/**
*Cancel a running magnetometer calibration
*			 1	uint8_t bitmask of magnetometers (0 means all)
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL                  = 42426,
	/**
*Command autopilot to get into factory test/diagnostic mode
*			 1	0 means get out of test mode, 1 means get into test mode
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE              = 42427,
	/**
*Reply with the version banner
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_DO_SEND_BANNER                     = 42428,
	/**
*Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle
*			 in. When sent to the vehicle says what position the vehicle is in
*			 1	Position, one of the ACCELCAL_VEHICLE_POS enum values
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS               = 42429,
	/**
*Causes the gimbal to reset and boot as if it was just powered on
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_GIMBAL_RESET                       = 42501,
	/**
*Reports progress and success or failure of gimbal axis calibration procedure
*			 1	Gimbal axis we're reporting calibration progress for
*			 2	Current calibration progress for this axis, 0x64=100%
*			 3	Status of the calibration
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS     = 42502,
	/**
*Starts commutation calibration on the gimbal
*			 1	Empty
*			 2	Empty
*			 3	Empty
*			 4	Empty
*			 5	Empty
*			 6	Empty
*			 7	Empty */
	e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION    = 42503,
	/**
*Erases gimbal application and parameters
*			 1	Magic number
*			 2	Magic number
*			 3	Magic number
*			 4	Magic number
*			 5	Magic number
*			 6	Magic number
*			 7	Magic number */
	e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET                  = 42505
}           e_MAV_CMD;

//Get enum by value
static inline e_MAV_CMD _en_mav_cmd(UMAX id) {
	switch (id) {
		case 0:return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
		case 1:return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
		case 2:return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
		case 3:return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
		case 4:return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
		case 5:return e_MAV_CMD_MAV_CMD_NAV_LAND;
		case 6:return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
		case 7:return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
		case 8:return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
		case 9:return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
		case 10:return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
		case 11:return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
		case 12:return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
		case 13:return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
		case 14:return e_MAV_CMD_MAV_CMD_NAV_ROI;
		case 15:return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
		case 16:return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
		case 17:return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
		case 18:return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
		case 19:return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
		case 20:return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
		case 21:return e_MAV_CMD_MAV_CMD_NAV_DELAY;
		case 22:return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
		case 23:return e_MAV_CMD_MAV_CMD_NAV_LAST;
		case 24:return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
		case 25:return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
		case 26:return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
		case 27:return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
		case 28:return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
		case 29:return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
		case 30:return e_MAV_CMD_MAV_CMD_DO_JUMP;
		case 31:return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
		case 32:return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
		case 33:return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
		case 34:return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
		case 35:return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
		case 36:return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
		case 37:return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
		case 38:return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
		case 39:return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
		case 40:return e_MAV_CMD_MAV_CMD_DO_LAND_START;
		case 41:return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
		case 42:return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
		case 43:return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
		case 44:return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
		case 45:return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
		case 46:return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
		case 47:return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
		case 48:return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
		case 49:return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
		case 50:return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
		case 51:return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
		case 52:return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
		case 53:return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
		case 54:return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
		case 55:return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
		case 56:return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
		case 57:return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
		case 58:return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
		case 59:return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
		case 60:return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
		case 61:return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
		case 62:return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
		case 63:return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
		case 64:return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
		case 65:return e_MAV_CMD_MAV_CMD_DO_LAST;
		case 66:return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
		case 67:return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
		case 68:return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
		case 69:return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
		case 70:return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
		case 71:return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
		case 72:return e_MAV_CMD_MAV_CMD_MISSION_START;
		case 73:return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
		case 74:return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
		case 75:return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
		case 76:return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
		case 77:return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
		case 78:return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
		case 79:return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
		case 80:return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
		case 81:return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
		case 82:return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
		case 83:return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
		case 84:return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
		case 85:return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
		case 86:return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
		case 87:return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
		case 88:return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
		case 89:return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
		case 90:return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
		case 91:return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
		case 92:return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
		case 93:return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
		case 94:return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
		case 95:return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
		case 96:return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
		case 97:return e_MAV_CMD_MAV_CMD_LOGGING_START;
		case 98:return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
		case 99:return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
		case 100:return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
		case 101:return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
		case 102:return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
		case 103:return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
		case 104:return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
		case 105:return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
		case 106:return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
		case 107:return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
		case 108:return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
		case 109:return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
		case 110:return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
		case 111:return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
		case 112:return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
		case 113:return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
		case 114:return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
		case 115:return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
		case 116:return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
		case 117:return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
		case 118:return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
		case 119:return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
		case 120:return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
		case 121:return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
		case 122:return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
		case 123:return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
		case 124:return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
		case 125:return e_MAV_CMD_MAV_CMD_USER_1;
		case 126:return e_MAV_CMD_MAV_CMD_USER_2;
		case 127:return e_MAV_CMD_MAV_CMD_USER_3;
		case 128:return e_MAV_CMD_MAV_CMD_USER_4;
		case 129:return e_MAV_CMD_MAV_CMD_USER_5;
		case 130:return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
		case 131:return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
		case 132:return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
		case 133:return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
		case 134:return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
		case 135:return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
		case 136:return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
		case 137:return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
		case 138:return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
		case 139:return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
		case 140:return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
		case 141:return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
		case 142:return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
		case 143:return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_mav_cmd(e_MAV_CMD en) {
	switch (en) {
		case e_MAV_CMD_MAV_CMD_NAV_WAYPOINT:return 0;
		case e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM:return 1;
		case e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS:return 2;
		case e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME:return 3;
		case e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH:return 4;
		case e_MAV_CMD_MAV_CMD_NAV_LAND:return 5;
		case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF:return 6;
		case e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL:return 7;
		case e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL:return 8;
		case e_MAV_CMD_MAV_CMD_NAV_FOLLOW:return 9;
		case e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:return 10;
		case e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT:return 11;
		case e_MAV_CMD_MAV_CMD_DO_FOLLOW:return 12;
		case e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION:return 13;
		case e_MAV_CMD_MAV_CMD_NAV_ROI:return 14;
		case e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING:return 15;
		case e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT:return 16;
		case e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT:return 17;
		case e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF:return 18;
		case e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND:return 19;
		case e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE:return 20;
		case e_MAV_CMD_MAV_CMD_NAV_DELAY:return 21;
		case e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE:return 22;
		case e_MAV_CMD_MAV_CMD_NAV_LAST:return 23;
		case e_MAV_CMD_MAV_CMD_CONDITION_DELAY:return 24;
		case e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT:return 25;
		case e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE:return 26;
		case e_MAV_CMD_MAV_CMD_CONDITION_YAW:return 27;
		case e_MAV_CMD_MAV_CMD_CONDITION_LAST:return 28;
		case e_MAV_CMD_MAV_CMD_DO_SET_MODE:return 29;
		case e_MAV_CMD_MAV_CMD_DO_JUMP:return 30;
		case e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED:return 31;
		case e_MAV_CMD_MAV_CMD_DO_SET_HOME:return 32;
		case e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER:return 33;
		case e_MAV_CMD_MAV_CMD_DO_SET_RELAY:return 34;
		case e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY:return 35;
		case e_MAV_CMD_MAV_CMD_DO_SET_SERVO:return 36;
		case e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO:return 37;
		case e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION:return 38;
		case e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE:return 39;
		case e_MAV_CMD_MAV_CMD_DO_LAND_START:return 40;
		case e_MAV_CMD_MAV_CMD_DO_RALLY_LAND:return 41;
		case e_MAV_CMD_MAV_CMD_DO_GO_AROUND:return 42;
		case e_MAV_CMD_MAV_CMD_DO_REPOSITION:return 43;
		case e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE:return 44;
		case e_MAV_CMD_MAV_CMD_DO_SET_REVERSE:return 45;
		case e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO:return 46;
		case e_MAV_CMD_MAV_CMD_DO_SET_ROI:return 47;
		case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE:return 48;
		case e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL:return 49;
		case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE:return 50;
		case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL:return 51;
		case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST:return 52;
		case e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE:return 53;
		case e_MAV_CMD_MAV_CMD_DO_PARACHUTE:return 54;
		case e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST:return 55;
		case e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT:return 56;
		case e_MAV_CMD_MAV_CMD_DO_GRIPPER:return 57;
		case e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE:return 58;
		case e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED:return 59;
		case e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:return 60;
		case e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT:return 61;
		case e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER:return 62;
		case e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS:return 63;
		case e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL:return 64;
		case e_MAV_CMD_MAV_CMD_DO_LAST:return 65;
		case e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION:return 66;
		case e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:return 67;
		case e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN:return 68;
		case e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE:return 69;
		case e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:return 70;
		case e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO:return 71;
		case e_MAV_CMD_MAV_CMD_MISSION_START:return 72;
		case e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM:return 73;
		case e_MAV_CMD_MAV_CMD_GET_HOME_POSITION:return 74;
		case e_MAV_CMD_MAV_CMD_START_RX_PAIR:return 75;
		case e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL:return 76;
		case e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL:return 77;
		case e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION:return 78;
		case e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:return 79;
		case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION:return 80;
		case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS:return 81;
		case e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION:return 82;
		case e_MAV_CMD_MAV_CMD_STORAGE_FORMAT:return 83;
		case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:return 84;
		case e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION:return 85;
		case e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS:return 86;
		case e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE:return 87;
		case e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE:return 88;
		case e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE:return 89;
		case e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:return 90;
		case e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL:return 91;
		case e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE:return 92;
		case e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE:return 93;
		case e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING:return 94;
		case e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING:return 95;
		case e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:return 96;
		case e_MAV_CMD_MAV_CMD_LOGGING_START:return 97;
		case e_MAV_CMD_MAV_CMD_LOGGING_STOP:return 98;
		case e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION:return 99;
		case e_MAV_CMD_MAV_CMD_PANORAMA_CREATE:return 100;
		case e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION:return 101;
		case e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST:return 102;
		case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:return 103;
		case e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:return 104;
		case e_MAV_CMD_MAV_CMD_CONDITION_GATE:return 105;
		case e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT:return 106;
		case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:return 107;
		case e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:return 108;
		case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:return 109;
		case e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:return 110;
		case e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT:return 111;
		case e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO:return 112;
		case e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY:return 113;
		case e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY:return 114;
		case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1:return 115;
		case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2:return 116;
		case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3:return 117;
		case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4:return 118;
		case e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5:return 119;
		case e_MAV_CMD_MAV_CMD_SPATIAL_USER_1:return 120;
		case e_MAV_CMD_MAV_CMD_SPATIAL_USER_2:return 121;
		case e_MAV_CMD_MAV_CMD_SPATIAL_USER_3:return 122;
		case e_MAV_CMD_MAV_CMD_SPATIAL_USER_4:return 123;
		case e_MAV_CMD_MAV_CMD_SPATIAL_USER_5:return 124;
		case e_MAV_CMD_MAV_CMD_USER_1:return 125;
		case e_MAV_CMD_MAV_CMD_USER_2:return 126;
		case e_MAV_CMD_MAV_CMD_USER_3:return 127;
		case e_MAV_CMD_MAV_CMD_USER_4:return 128;
		case e_MAV_CMD_MAV_CMD_USER_5:return 129;
		case e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED:return 130;
		case e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK:return 131;
		case e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD:return 132;
		case e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK:return 133;
		case e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL:return 134;
		case e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL:return 135;
		case e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL:return 136;
		case e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE:return 137;
		case e_MAV_CMD_MAV_CMD_DO_SEND_BANNER:return 138;
		case e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS:return 139;
		case e_MAV_CMD_MAV_CMD_GIMBAL_RESET:return 140;
		case e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:return 141;
		case e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:return 142;
		case e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET:return 143;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Camera Modes. */
typedef enum {
	e_CAMERA_MODE_CAMERA_MODE_IMAGE        = 0,//Camera is in image/photo capture mode.
	e_CAMERA_MODE_CAMERA_MODE_VIDEO        = 1,//Camera is in video capture mode.
	e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY = 2//Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
}           e_CAMERA_MODE;
/**
*Source of information about this collision. */
typedef enum {
	e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB                   = 0,//ID field references ADSB_VEHICLE packets
	e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1//ID field references MAVLink SRC ID
}           e_MAV_COLLISION_SRC;
/**
*Possible remote log data block statuses */
typedef enum {
	e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK = 0,//This block has NOT been received
	e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK  = 1//This block has been received
}           e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES;
/**
*Status flags for ADS-B transponder dynamic output */
typedef enum {
	e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = 0,
	e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK           = 1,
	e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX      = 2,
	e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX      = 16
}           e_UAVIONIX_ADSB_RF_HEALTH;

//Get enum by value
static inline e_UAVIONIX_ADSB_RF_HEALTH _en_uavionix_adsb_rf_health(UMAX id) {
	switch (id) {
		case 0:return e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING;
		case 1:return e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK;
		case 2:return e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX;
		case 3:return e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_uavionix_adsb_rf_health(e_UAVIONIX_ADSB_RF_HEALTH en) {
	switch (en) {
		case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING:return 0;
		case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK:return 1;
		case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX:return 2;
		case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX:return 3;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*result from a mavlink command */
typedef enum {
	e_MAV_RESULT_MAV_RESULT_ACCEPTED             = 0,//Command ACCEPTED and EXECUTED
	e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED = 1,//Command TEMPORARY REJECTED/DENIED
	e_MAV_RESULT_MAV_RESULT_DENIED               = 2,//Command PERMANENTLY DENIED
	e_MAV_RESULT_MAV_RESULT_UNSUPPORTED          = 3,//Command UNKNOWN/UNSUPPORTED
	e_MAV_RESULT_MAV_RESULT_FAILED               = 4,//Command executed, but failed
	e_MAV_RESULT_MAV_RESULT_IN_PROGRESS          = 5//WIP: Command being executed
}           e_MAV_RESULT;
/**
*Special ACK block numbers control activation of dataflash log streaming */
typedef enum {
	e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP  = 2147483645UL,//UAV to stop sending DataFlash blocks
	e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_START = 2147483646UL//UAV to start sending DataFlash blocks
}           e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS;
/**
*Status for ADS-B transponder dynamic input */
typedef enum {
	e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = 0,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1 = 1,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D     = 2,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D     = 3,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS   = 4,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK    = 5
}           e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX;
typedef enum {
	e_GOPRO_COMMAND_GOPRO_COMMAND_POWER                 = 0,//(Get/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_CAPTURE_MODE          = 1,//(Get/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER               = 2,//(___/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_BATTERY               = 3,//(Get/___)
	e_GOPRO_COMMAND_GOPRO_COMMAND_MODEL                 = 4,//(Get/___)
	e_GOPRO_COMMAND_GOPRO_COMMAND_VIDEO_SETTINGS        = 5,//(Get/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_LOW_LIGHT             = 6,//(Get/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_RESOLUTION      = 7,//(Get/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_BURST_RATE      = 8,//(Get/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE               = 9,//(Get/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_WHITE_BALANCE = 10,//(Get/Set) Hero 3+ Only
	e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_COLOUR        = 11,//(Get/Set) Hero 3+ Only
	e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_GAIN          = 12,//(Get/Set) Hero 3+ Only
	e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_SHARPNESS     = 13,//(Get/Set) Hero 3+ Only
	e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_EXPOSURE      = 14,//(Get/Set) Hero 3+ Only
	e_GOPRO_COMMAND_GOPRO_COMMAND_TIME                  = 15,//(Get/Set)
	e_GOPRO_COMMAND_GOPRO_COMMAND_CHARGING              = 16//(Get/Set)
}           e_GOPRO_COMMAND;
typedef enum {
	e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS = 0,//The write message with ID indicated succeeded
	e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED  = 1//The write message with ID indicated failed
}           e_GOPRO_REQUEST_STATUS;
/**
*Definitions for aircraft size */
typedef enum {
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA     = 0,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M   = 1,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M = 2,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M     = 3,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_33M     = 4,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M     = 5,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M   = 6,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M     = 7,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M     = 8,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M     = 9,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M   = 10,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M     = 11,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M  = 12,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W80M    = 13,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M    = 14,
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M    = 15
}           e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE;
typedef enum {
	e_MAV_STATE_UNINIT             = 0,//Uninitialized system, state is unknown.
	e_MAV_STATE_ACTIVE             = 1,//System is active and might be already airborne. Motors are engaged.
	e_MAV_STATE_BOOT               = 2,//System is booting up.
	e_MAV_STATE_CALIBRATING        = 3,//System is calibrating and not flight-ready.
	e_MAV_STATE_CRITICAL           = 4,//System is in a non-normal flight mode. It can however still navigate.
	/**
*System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
*			 mayday and going down */
	e_MAV_STATE_EMERGENCY          = 5,
	e_MAV_STATE_FLIGHT_TERMINATION = 6,//System is terminating itself.
	e_MAV_STATE_POWEROFF           = 7,//System just initialized its power-down sequence, will shut down now.
	e_MAV_STATE_STANDBY            = 8//System is grounded and on standby. It can be launched any time.
}           e_MAV_STATE;
/**
*SERIAL_CONTROL flags (bitmask) */
typedef enum {
	e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY     = 1,//Set if this is a reply
	e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND   = 2,//Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
	/**
*Set if access to the serial port should be removed from whatever driver is currently using it, giving
*			 exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without
*			 this flag se */
	e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
	e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING  = 8,//Block on writes to the serial port
	e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI     = 16//Send multiple replies until port is drained
}           e_SERIAL_CONTROL_FLAG;

//Get enum by value
static inline e_SERIAL_CONTROL_FLAG _en_serial_control_flag(UMAX id) {
	switch (id) {
		case 0:return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY;
		case 1:return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND;
		case 2:return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE;
		case 3:return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING;
		case 4:return e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_serial_control_flag(e_SERIAL_CONTROL_FLAG en) {
	switch (en) {
		case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY:return 0;
		case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND:return 1;
		case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE:return 2;
		case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING:return 3;
		case e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI:return 4;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Camera capability flags (Bitmap). */
typedef enum {
	e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO                   = 1,//Camera is able to record video.
	e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE                   = 2,//Camera is able to capture images.
	e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES                       = 4,//Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
	e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8,//Camera can capture images while in video mode
	e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16,//Camera can capture videos while in Photo/Image mode
	e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE           = 32//Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
}           e_CAMERA_CAP_FLAGS;

//Get enum by value
static inline e_CAMERA_CAP_FLAGS _en_camera_cap_flags(UMAX id) {
	switch (id) {
		case 0:return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
		case 1:return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
		case 2:return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES;
		case 3:return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
		case 4:return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
		case 5:return e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_camera_cap_flags(e_CAMERA_CAP_FLAGS en) {
	switch (en) {
		case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO:return 0;
		case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE:return 1;
		case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES:return 2;
		case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE:return 3;
		case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE:return 4;
		case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE:return 5;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

typedef enum {
	e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING = 1//GoPro is currently recording
}           e_GOPRO_HEARTBEAT_FLAGS;
/**
*Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability */
typedef enum {
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT                  = 1,//Autopilot supports MISSION float message type.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT                    = 2,//Autopilot supports the new param float message type.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT                    = 4,//Autopilot supports MISSION_INT scaled integer message type.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT                    = 8,//Autopilot supports COMMAND_INT scaled integer message type.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION                    = 16,//Autopilot supports the new param union message type.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP                            = 32,//Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET            = 64,//Autopilot supports commanding attitude offboard.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED  = 128,//Autopilot supports commanding position and velocity targets in local NED frame.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256,//Autopilot supports commanding position and velocity targets in global scaled integers.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN                        = 512,//Autopilot supports terrain protocol / data handling.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET            = 1024,//Autopilot supports direct actuator control.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION             = 2048,//Autopilot supports the flight termination command.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION            = 4096,//Autopilot supports onboard compass calibration.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2                       = 8192,//Autopilot supports mavlink version 2.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE                  = 16384,//Autopilot supports mission fence protocol.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY                  = 32768,//Autopilot supports mission rally point protocol.
	e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION             = 65536//Autopilot supports the flight information protocol.
}           e_MAV_PROTOCOL_CAPABILITY;

//Get enum by value
static inline e_MAV_PROTOCOL_CAPABILITY _en_mav_protocol_capability(UMAX id) {
	switch (id) {
		case 0:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
		case 1:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
		case 2:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT;
		case 3:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
		case 4:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION;
		case 5:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP;
		case 6:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
		case 7:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
		case 8:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;
		case 9:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN;
		case 10:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
		case 11:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
		case 12:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;
		case 13:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2;
		case 14:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
		case 15:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
		case 16:return e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_mav_protocol_capability(e_MAV_PROTOCOL_CAPABILITY en) {
	switch (en) {
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT:return 0;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT:return 1;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT:return 2;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT:return 3;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION:return 4;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP:return 5;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET:return 6;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED:return 7;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT:return 8;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN:return 9;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET:return 10;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION:return 11;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION:return 12;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2:return 13;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE:return 14;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY:return 15;
		case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION:return 16;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

typedef enum {
	e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED = 0,//No GoPro connected
	e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE = 1,//The detected GoPro is not HeroBus compatible
	e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_CONNECTED    = 2,//A HeroBus compatible GoPro is connected
	e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_ERROR        = 3//An unrecoverable error was encountered with the connected GoPro, it may require a power cycle
}           e_GOPRO_HEARTBEAT_STATUS;
typedef enum {
	e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED      = 0,
	e_MAG_CAL_STATUS_MAG_CAL_WAITING_TO_START = 1,
	e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE = 2,
	e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_TWO = 3,
	e_MAG_CAL_STATUS_MAG_CAL_SUCCESS          = 4,
	e_MAG_CAL_STATUS_MAG_CAL_FAILED           = 5
}           e_MAG_CAL_STATUS;
typedef enum {
	e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_HEARTBEAT  = 0,//Camera heartbeat, announce camera component ID at 1hz
	e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_TRIGGER    = 1,//Camera image triggered
	e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_DISCONNECT = 2,//Camera connection lost
	e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_ERROR      = 3,//Camera unknown error
	e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_LOWBATT    = 4,//Camera battery low. Parameter p1 shows reported voltage
	e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_LOWSTORE   = 5,//Camera storage low. Parameter p1 shows reported shots remaining
	e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_LOWSTOREV  = 6//Camera storage low. Parameter p1 shows reported video minutes remaining
}           e_CAMERA_STATUS_TYPES;
/**
*SERIAL_CONTROL device types */
typedef enum {
	e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1 = 0,//First telemetry port
	e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2 = 1,//Second telemetry port
	e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1   = 2,//First GPS port
	e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2   = 3,//Second GPS port
	e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL  = 10//system shell
}           e_SERIAL_CONTROL_DEV;

//Get enum by value
static inline e_SERIAL_CONTROL_DEV _en_serial_control_dev(UMAX id) {
	switch (id) {
		case 0:return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1;
		case 1:return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2;
		case 2:return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1;
		case 3:return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2;
		case 4:return e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_serial_control_dev(e_SERIAL_CONTROL_DEV en) {
	switch (en) {
		case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1:return 0;
		case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2:return 1;
		case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1:return 2;
		case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2:return 3;
		case e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL:return 4;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*result in a mavlink mission ack */
typedef enum {
	e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED          = 0,//mission accepted OK
	e_MAV_MISSION_RESULT_MAV_MISSION_ERROR             = 1,//generic error / not accepting mission commands at all right now
	e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED_FRAME = 2,//coordinate frame is not supported
	e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED       = 3,//command is not supported
	e_MAV_MISSION_RESULT_MAV_MISSION_NO_SPACE          = 4,//mission item exceeds storage space
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID           = 5,//one of the parameters has an invalid value
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1    = 6,//param1 has an invalid value
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM2    = 7,//param2 has an invalid value
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3    = 8,//param3 has an invalid value
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM4    = 9,//param4 has an invalid value
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM5_X  = 10,//x/param5 has an invalid value
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM6_Y  = 11,//y/param6 has an invalid value
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7    = 12,//param7 has an invalid value
	e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE  = 13,//received waypoint out of sequence
	e_MAV_MISSION_RESULT_MAV_MISSION_DENIED            = 14//not accepting any mission commands from this communication partner
}           e_MAV_MISSION_RESULT;
/**
*Power supply status flags (bitmask) */
typedef enum {
	e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID                = 1,//main brick power supply valid
	e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID                = 2,//main servo power supply valid for FMU
	e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED              = 4,//USB power is connected
	e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT         = 8,//peripheral supply is in over-current state
	e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16,//hi-power peripheral supply is in over-current state
	e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED                    = 32//Power status has changed since boot
}           e_MAV_POWER_STATUS;

//Get enum by value
static inline e_MAV_POWER_STATUS _en_mav_power_status(UMAX id) {
	switch (id) {
		case 0:return e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID;
		case 1:return e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID;
		case 2:return e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED;
		case 3:return e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT;
		case 4:return e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
		case 5:return e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_mav_power_status(e_MAV_POWER_STATUS en) {
	switch (en) {
		case e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID:return 0;
		case e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID:return 1;
		case e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED:return 2;
		case e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT:return 3;
		case e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT:return 4;
		case e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED:return 5;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Generalized UAVCAN node mode */
typedef enum {
	e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL     = 0,//The node is performing its primary functions.
	e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION  = 1,//The node is initializing; this mode is entered immediately after startup.
	e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE     = 2,//The node is under maintenance.
	e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3,//The node is in the process of updating its software.
	e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE         = 7//The node is no longer available online.
}           e_UAVCAN_NODE_MODE;

//Get enum by value
static inline e_UAVCAN_NODE_MODE _en_uavcan_node_mode(UMAX id) {
	switch (id) {
		case 0:return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL;
		case 1:return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION;
		case 2:return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE;
		case 3:return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
		case 4:return e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_uavcan_node_mode(e_UAVCAN_NODE_MODE en) {
	switch (en) {
		case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL:return 0;
		case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION:return 1;
		case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE:return 2;
		case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE:return 3;
		case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE:return 4;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*These flags indicate status such as data validity of each data source. Set = data valid */
typedef enum {
	e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS   = 1,
	e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE = 2,
	e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING  = 4,
	e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY = 8,
	e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN = 16,
	e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK   = 32,
	e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED      = 64
}           e_ADSB_FLAGS;

//Get enum by value
static inline e_ADSB_FLAGS _en_adsb_flags(UMAX id) {
	switch (id) {
		case 0:return e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS;
		case 1:return e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE;
		case 2:return e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING;
		case 3:return e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY;
		case 4:return e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN;
		case 5:return e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK;
		case 6:return e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_adsb_flags(e_ADSB_FLAGS en) {
	switch (en) {
		case e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS:return 0;
		case e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE:return 1;
		case e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING:return 2;
		case e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY:return 3;
		case e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN:return 4;
		case e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK:return 5;
		case e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED:return 6;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Result from a PARAM_EXT_SET message. */
typedef enum {
	e_PARAM_ACK_PARAM_ACK_ACCEPTED          = 0,//Parameter value ACCEPTED and SET
	e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED = 1,//Parameter value UNKNOWN/UNSUPPORTED
	e_PARAM_ACK_PARAM_ACK_FAILED            = 2,//Parameter failed to set
	/**
*Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation
*			 is completed with the actual result. These are for parameters that may take longer to set. Instead of
*			 waiting for an ACK and potentially timing out, you will immediately receive this response to let you
*			 know it was received */
	e_PARAM_ACK_PARAM_ACK_IN_PROGRESS       = 3
}           e_PARAM_ACK;
/**
*Enumeration of possible mount operation modes */
typedef enum {
	e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RETRACT           = 0,//Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
	e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_NEUTRAL           = 1,//Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
	e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_MAVLINK_TARGETING = 2,//Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RC_TARGETING      = 3,//Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
	e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_GPS_POINT         = 4//Load neutral position and start to point to Lat,Lon,Alt
}           e_MAV_MOUNT_MODE;
/**
*Enumeration of sensor orientation, according to its rotations */
typedef enum {
	e_MAV_SENSOR_ORIENTATION_NONE                       = 0,//Roll: 0, Pitch: 0, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_YAW_45                     = 1,//Roll: 0, Pitch: 0, Yaw: 45
	e_MAV_SENSOR_ORIENTATION_YAW_90                     = 2,//Roll: 0, Pitch: 0, Yaw: 90
	e_MAV_SENSOR_ORIENTATION_YAW_135                    = 3,//Roll: 0, Pitch: 0, Yaw: 135
	e_MAV_SENSOR_ORIENTATION_YAW_180                    = 4,//Roll: 0, Pitch: 0, Yaw: 180
	e_MAV_SENSOR_ORIENTATION_YAW_225                    = 5,//Roll: 0, Pitch: 0, Yaw: 225
	e_MAV_SENSOR_ORIENTATION_YAW_270                    = 6,//Roll: 0, Pitch: 0, Yaw: 270
	e_MAV_SENSOR_ORIENTATION_YAW_315                    = 7,//Roll: 0, Pitch: 0, Yaw: 315
	e_MAV_SENSOR_ORIENTATION_ROLL_180                   = 8,//Roll: 180, Pitch: 0, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_45            = 9,//Roll: 180, Pitch: 0, Yaw: 45
	e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_90            = 10,//Roll: 180, Pitch: 0, Yaw: 90
	e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_135           = 11,//Roll: 180, Pitch: 0, Yaw: 135
	e_MAV_SENSOR_ORIENTATION_PITCH_180                  = 12,//Roll: 0, Pitch: 180, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_225           = 13,//Roll: 180, Pitch: 0, Yaw: 225
	e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_270           = 14,//Roll: 180, Pitch: 0, Yaw: 270
	e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_315           = 15,//Roll: 180, Pitch: 0, Yaw: 315
	e_MAV_SENSOR_ORIENTATION_ROLL_90                    = 16,//Roll: 90, Pitch: 0, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_90_YAW_45             = 17,//Roll: 90, Pitch: 0, Yaw: 45
	e_MAV_SENSOR_ORIENTATION_ROLL_90_YAW_90             = 18,//Roll: 90, Pitch: 0, Yaw: 90
	e_MAV_SENSOR_ORIENTATION_ROLL_90_YAW_135            = 19,//Roll: 90, Pitch: 0, Yaw: 135
	e_MAV_SENSOR_ORIENTATION_ROLL_270                   = 20,//Roll: 270, Pitch: 0, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_270_YAW_45            = 21,//Roll: 270, Pitch: 0, Yaw: 45
	e_MAV_SENSOR_ORIENTATION_ROLL_270_YAW_90            = 22,//Roll: 270, Pitch: 0, Yaw: 90
	e_MAV_SENSOR_ORIENTATION_ROLL_270_YAW_135           = 23,//Roll: 270, Pitch: 0, Yaw: 135
	e_MAV_SENSOR_ORIENTATION_PITCH_90                   = 24,//Roll: 0, Pitch: 90, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_PITCH_270                  = 25,//Roll: 0, Pitch: 270, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_PITCH_180_YAW_90           = 26,//Roll: 0, Pitch: 180, Yaw: 90
	e_MAV_SENSOR_ORIENTATION_PITCH_180_YAW_270          = 27,//Roll: 0, Pitch: 180, Yaw: 270
	e_MAV_SENSOR_ORIENTATION_ROLL_90_PITCH_90           = 28,//Roll: 90, Pitch: 90, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_180_PITCH_90          = 29,//Roll: 180, Pitch: 90, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_270_PITCH_90          = 30,//Roll: 270, Pitch: 90, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_90_PITCH_180          = 31,//Roll: 90, Pitch: 180, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_270_PITCH_180         = 32,//Roll: 270, Pitch: 180, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_90_PITCH_270          = 33,//Roll: 90, Pitch: 270, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_180_PITCH_270         = 34,//Roll: 180, Pitch: 270, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_270_PITCH_270         = 35,//Roll: 270, Pitch: 270, Yaw: 0
	e_MAV_SENSOR_ORIENTATION_ROLL_90_PITCH_180_YAW_90   = 36,//Roll: 90, Pitch: 180, Yaw: 90
	e_MAV_SENSOR_ORIENTATION_ROLL_90_YAW_270            = 37,//Roll: 90, Pitch: 0, Yaw: 270
	e_MAV_SENSOR_ORIENTATION_ROLL_315_PITCH_315_YAW_315 = 38//Roll: 315, Pitch: 315, Yaw: 315
}           e_MAV_SENSOR_ORIENTATION;
/**
*State flags for ADS-B transponder dynamic report */
typedef enum {
	e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE        = 1,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED    = 2,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED = 4,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND            = 8,
	e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT                = 16
}           e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE;

//Get enum by value
static inline e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE _en_uavionix_adsb_out_dynamic_state(UMAX id) {
	switch (id) {
		case 0:return e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE;
		case 1:return e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED;
		case 2:return e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED;
		case 3:return e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
		case 4:return e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_uavionix_adsb_out_dynamic_state(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE en) {
	switch (en) {
		case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE:return 0;
		case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED:return 1;
		case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED:return 2;
		case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND:return 3;
		case e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT:return 4;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Possible actions an aircraft can take to avoid a collision. */
typedef enum {
	e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE               = 0,//Ignore any potential collisions
	e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT             = 1,//Report potential collision
	e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND  = 2,//Ascend or Descend to avoid threat
	e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY  = 3,//Move horizontally to avoid threat
	e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4,//Aircraft to move perpendicular to the collision's velocity vector
	e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL                = 5,//Aircraft to fly directly back to its launch point
	e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER              = 6//Aircraft to stop in place
}           e_MAV_COLLISION_ACTION;
/**
*Aircraft-rated danger from this threat. */
typedef enum {
	e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE = 0,//Not a threat
	e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW  = 1,//Craft is mildly concerned about this threat
	e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH = 2//Craft is panicing, and may take actions to avoid threat
}           e_MAV_COLLISION_THREAT_LEVEL;
typedef enum {
	e_LIMITS_STATE_LIMITS_INIT       = 0,//pre-initialization
	e_LIMITS_STATE_LIMITS_DISABLED   = 1,//disabled
	e_LIMITS_STATE_LIMITS_ENABLED    = 2,//checking limits
	e_LIMITS_STATE_LIMITS_TRIGGERED  = 3,//a limit has been breached
	e_LIMITS_STATE_LIMITS_RECOVERING = 4,//taking action eg. RTL
	e_LIMITS_STATE_LIMITS_RECOVERED  = 5//we're no longer in breach of a limit
}           e_LIMITS_STATE;
/**
*Transceiver RF control flags for ADS-B transponder dynamic reports */
typedef enum {
	e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY    = 0,
	e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED = 1,
	e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED = 2
}           e_UAVIONIX_ADSB_OUT_RF_SELECT;
/**
*Enumeration of battery types */
typedef enum {
	e_MAV_BATTERY_TYPE_UNKNOWN = 0,//Not specified.
	e_MAV_BATTERY_TYPE_LIPO    = 1,//Lithium polymer battery
	e_MAV_BATTERY_TYPE_LIFE    = 2,//Lithium-iron-phosphate battery
	e_MAV_BATTERY_TYPE_LION    = 3,//Lithium-ION battery
	e_MAV_BATTERY_TYPE_NIMH    = 4//Nickel metal hydride battery
}           e_MAV_BATTERY_TYPE;
/**
*Flags in EKF_STATUS message */
typedef enum {
	e_EKF_STATUS_FLAGS_EKF_ATTITUDE           = 1,//set if EKF's attitude estimate is good
	e_EKF_STATUS_FLAGS_EKF_VELOCITY_HORIZ     = 2,//set if EKF's horizontal velocity estimate is good
	e_EKF_STATUS_FLAGS_EKF_VELOCITY_VERT      = 4,//set if EKF's vertical velocity estimate is good
	e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL      = 8,//set if EKF's horizontal position (relative) estimate is good
	e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_ABS      = 16,//set if EKF's horizontal position (absolute) estimate is good
	e_EKF_STATUS_FLAGS_EKF_POS_VERT_ABS       = 32,//set if EKF's vertical position (absolute) estimate is good
	e_EKF_STATUS_FLAGS_EKF_POS_VERT_AGL       = 64,//set if EKF's vertical position (above ground) estimate is good
	e_EKF_STATUS_FLAGS_EKF_CONST_POS_MODE     = 128,//EKF is in constant position mode and does not know it's absolute or relative position
	e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL = 256,//set if EKF's predicted horizontal position (relative) estimate is good
	e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_ABS = 512//set if EKF's predicted horizontal position (absolute) estimate is good
}           e_EKF_STATUS_FLAGS;

//Get enum by value
static inline e_EKF_STATUS_FLAGS _en_ekf_status_flags(UMAX id) {
	switch (id) {
		case 0:return e_EKF_STATUS_FLAGS_EKF_ATTITUDE;
		case 1:return e_EKF_STATUS_FLAGS_EKF_VELOCITY_HORIZ;
		case 2:return e_EKF_STATUS_FLAGS_EKF_VELOCITY_VERT;
		case 3:return e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL;
		case 4:return e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_ABS;
		case 5:return e_EKF_STATUS_FLAGS_EKF_POS_VERT_ABS;
		case 6:return e_EKF_STATUS_FLAGS_EKF_POS_VERT_AGL;
		case 7:return e_EKF_STATUS_FLAGS_EKF_CONST_POS_MODE;
		case 8:return e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL;
		case 9:return e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_ABS;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_ekf_status_flags(e_EKF_STATUS_FLAGS en) {
	switch (en) {
		case e_EKF_STATUS_FLAGS_EKF_ATTITUDE:return 0;
		case e_EKF_STATUS_FLAGS_EKF_VELOCITY_HORIZ:return 1;
		case e_EKF_STATUS_FLAGS_EKF_VELOCITY_VERT:return 2;
		case e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL:return 3;
		case e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_ABS:return 4;
		case e_EKF_STATUS_FLAGS_EKF_POS_VERT_ABS:return 5;
		case e_EKF_STATUS_FLAGS_EKF_POS_VERT_AGL:return 6;
		case e_EKF_STATUS_FLAGS_EKF_CONST_POS_MODE:return 7;
		case e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL:return 8;
		case e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_ABS:return 9;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*These encode the sensors whose status is sent as part of the SYS_STATUS message. */
typedef enum {
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO                = 1,//0x01 3D gyro
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL               = 2,//0x02 3D accelerometer
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG                 = 4,//0x04 3D magnetometer
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE      = 8,//0x08 absolute pressure
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE  = 16,//0x10 differential pressure
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS                    = 32,//0x20 GPS
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW           = 64,//0x40 optical flow
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION        = 128,//0x80 computer vision position
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION         = 256,//0x100 laser based position
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH  = 512,//0x200 external ground truth (Vicon or Leica)
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL   = 1024,//0x400 3D angular rate control
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048,//0x800 attitude stabilization
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION           = 4096,//0x1000 yaw position
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL     = 8192,//0x2000 z/altitude control
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL    = 16384,//0x4000 x/y position control
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS          = 32768,//0x8000 motor outputs / control
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER            = 65536,//0x10000 rc receiver
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2               = 131072,//0x20000 2nd 3D gyro
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2              = 262144,//0x40000 2nd 3D accelerometer
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2                = 524288,//0x80000 2nd 3D magnetometer
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE                      = 1048576,//0x100000 geofence
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS                          = 2097152,//0x200000 AHRS subsystem health
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN                       = 4194304,//0x400000 Terrain subsystem health
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR                 = 8388608,//0x800000 Motors are reversed
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING                       = 16777216,//0x1000000 Logging
	e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY                = 33554432//0x2000000 Battery
}           e_MAV_SYS_STATUS_SENSOR;

//Get enum by value
static inline e_MAV_SYS_STATUS_SENSOR _en_mav_sys_status_sensor(UMAX id) {
	switch (id) {
		case 0:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO;
		case 1:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL;
		case 2:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG;
		case 3:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
		case 4:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
		case 5:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS;
		case 6:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
		case 7:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION;
		case 8:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION;
		case 9:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
		case 10:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
		case 11:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
		case 12:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION;
		case 13:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
		case 14:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
		case 15:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
		case 16:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
		case 17:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2;
		case 18:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
		case 19:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2;
		case 20:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE;
		case 21:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS;
		case 22:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN;
		case 23:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR;
		case 24:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING;
		case 25:return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_mav_sys_status_sensor(e_MAV_SYS_STATUS_SENSOR en) {
	switch (en) {
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO:return 0;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL:return 1;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG:return 2;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:return 3;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:return 4;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS:return 5;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:return 6;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION:return 7;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION:return 8;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:return 9;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:return 10;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:return 11;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION:return 12;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:return 13;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:return 14;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:return 15;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER:return 16;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2:return 17;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2:return 18;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2:return 19;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE:return 20;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS:return 21;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN:return 22;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR:return 23;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING:return 24;
		case e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY:return 25;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

typedef enum {
	/**
*Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude,
*					 third value / z: positive altitude over mean sea level (MSL */
	e_MAV_FRAME_MAV_FRAME_GLOBAL                  = 0,
	e_MAV_FRAME_MAV_FRAME_LOCAL_NED               = 1,//Local coordinate frame, Z-up (x: north, y: east, z: down).
	e_MAV_FRAME_MAV_FRAME_MISSION                 = 2,//NOT a coordinate frame, indicates a mission command.
	/**
*Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
*			 position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude
*			 with 0 being at the altitude of the home location */
	e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3,
	e_MAV_FRAME_MAV_FRAME_LOCAL_ENU               = 4,//Local coordinate frame, Z-down (x: east, y: north, z: up)
	/**
*Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second
*			 value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL */
	e_MAV_FRAME_MAV_FRAME_GLOBAL_INT              = 5,
	/**
*Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
*			 position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third
*			 value / z: positive altitude with 0 being at the altitude of the home location */
	e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
	/**
*Offset to the current local frame. Anything expressed in this frame should be added to the current local
*			 frame position */
	e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED        = 7,
	/**
*Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to
*			 command 2 m/s^2 acceleration to the right */
	e_MAV_FRAME_MAV_FRAME_BODY_NED                = 8,
	/**
*Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an
*			 obstacle - e.g. useful to command 2 m/s^2 acceleration to the east */
	e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED         = 9,
	/**
*Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
*			 over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value
*			 / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level
*			 in terrain model */
	e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT      = 10,
	/**
*Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
*			 over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second
*			 value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground
*			 level in terrain model */
	e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT  = 11
}           e_MAV_FRAME;
/**
*ADSB classification for the type of vehicle emitting the transponder signal */
typedef enum {
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO           = 0,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT             = 1,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SMALL             = 2,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LARGE             = 3,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY             = 5,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGHLY_MANUV      = 6,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT         = 7,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED        = 8,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER            = 9,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHTER_AIR       = 10,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_PARACHUTE         = 11,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ULTRA_LIGHT       = 12,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED2       = 13,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV               = 14,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SPACE             = 15,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3       = 16,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SERVICE_SURFACE   = 18,
	e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE    = 19
}           e_ADSB_EMITTER_TYPE;
/**
*GPS lataral offset encoding */
typedef enum {
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA  = 0,
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M  = 1,
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M  = 2,
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M  = 3,
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M = 4,
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M = 5,
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M = 6,
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M = 7
}           e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT;
/**
*GPS longitudinal offset encoding */
typedef enum {
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA           = 0,
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR = 1
}           e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON;
typedef enum {
	e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT                 = 1,//ignore altitude field
	e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP                = 2,//ignore hdop field
	e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP                = 4,//ignore vdop field
	e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ           = 8,//ignore horizontal velocity field (vn and ve)
	e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT            = 16,//ignore vertical velocity field (vd)
	e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY      = 32,//ignore speed accuracy field
	e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64,//ignore horizontal accuracy field
	e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY   = 128//ignore vertical accuracy field
}           e_GPS_INPUT_IGNORE_FLAGS;

//Get enum by value
static inline e_GPS_INPUT_IGNORE_FLAGS _en_gps_input_ignore_flags(UMAX id) {
	switch (id) {
		case 0:return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT;
		case 1:return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP;
		case 2:return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP;
		case 3:return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ;
		case 4:return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT;
		case 5:return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
		case 6:return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
		case 7:return e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_gps_input_ignore_flags(e_GPS_INPUT_IGNORE_FLAGS en) {
	switch (en) {
		case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT:return 0;
		case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP:return 1;
		case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP:return 2;
		case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ:return 3;
		case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT:return 4;
		case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY:return 5;
		case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY:return 6;
		case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY:return 7;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Flags in RALLY_POINT message */
typedef enum {
	e_RALLY_FLAGS_FAVORABLE_WIND   = 1,//Flag set when requiring favorable winds for landing.
	/**
*Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag
*			 not set when plane is to loiter at Rally point until commanded to land */
	e_RALLY_FLAGS_LAND_IMMEDIATELY = 2
}           e_RALLY_FLAGS;
typedef enum {
	e_FENCE_BREACH_FENCE_BREACH_NONE     = 0,//No last fence breach
	e_FENCE_BREACH_FENCE_BREACH_MINALT   = 1,//Breached minimum altitude
	e_FENCE_BREACH_FENCE_BREACH_MAXALT   = 2,//Breached maximum altitude
	e_FENCE_BREACH_FENCE_BREACH_BOUNDARY = 3//Breached fence boundary
}           e_FENCE_BREACH;
/**
*These flags encode the MAV mode. */
typedef enum {
	e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1,//0b00000001 Reserved for future use.
	/**
*0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
*			 not be used for stable implementations */
	e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED         = 2,
	/**
*0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
*			 depends on the actual implementation */
	e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED         = 4,
	e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED       = 8,//0b00001000 guided mode enabled, system flies waypoints / mission items.
	/**
*0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
*			 control inputs to move around */
	e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED    = 16,
	/**
*0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
*			 is full operational */
	e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED          = 32,
	e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,//0b01000000 remote control input is enabled.
	/**
*0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional
*					 note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM
*					 shall be used instead. The flag can still be used to report the armed state */
	e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED         = 128
}           e_MAV_MODE_FLAG;

//Get enum by value
static inline e_MAV_MODE_FLAG _en_mav_mode_flag(UMAX id) {
	switch (id) {
		case 0:return e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
		case 1:return e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED;
		case 2:return e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED;
		case 3:return e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED;
		case 4:return e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED;
		case 5:return e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED;
		case 6:return e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		case 7:return e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED;
		
		default:;//assert(false);//("Unknown enum ID " + id);
	}
	return -1;
}

//Get value by enum
static inline UMAX _id_mav_mode_flag(e_MAV_MODE_FLAG en) {
	switch (en) {
		case e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:return 0;
		case e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED:return 1;
		case e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED:return 2;
		case e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED:return 3;
		case e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED:return 4;
		case e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED:return 5;
		case e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:return 6;
		case e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED:return 7;
		
		default:;// assert(false);//("Unknown enum" + id);
	}
}

/**
*Bus types for device operations */
typedef enum {
	e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C = 0,//I2C Device operation
	e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI = 1//SPI Device operation
}           e_DEVICE_OP_BUSTYPE;


/**
*Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
*				 the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way */

typedef Pack ATTITUDE_TARGET_attitude_target; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pattitude_target_ATTITUDE_TARGET;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pattitude_target_ATTITUDE_TARGET *pattitude_target_ATTITUDE_TARGET_from(ATTITUDE_TARGET_attitude_target *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vattitude_target_q;
//Maximum field array length constant
#define Pattitude_target_q_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_type_mask_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_q_SET( Vattitude_target_q * src, <DST> * dst  ){}
static inline void <DST>_body_roll_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_body_pitch_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_body_yaw_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_thrust_SET( float * src, <DST> * dst  ){}
*/

#define pattitude_target_ATTITUDE_TARGET_PUSH_INTO(DST)\
    static inline void pattitude_target_ATTITUDE_TARGET_push_into_##DST ( pattitude_target_ATTITUDE_TARGET * src, DST * dst) {\
        DST##_time_boot_ms_SET( pattitude_target_time_boot_ms_GET( src  ), dst  );\
        DST##_type_mask_SET( pattitude_target_type_mask_GET( src  ), dst  );\
        Vattitude_target_q item_q = pattitude_target_q_GET( src  );\
       DST##_q_SET( &item_q, dst );\
        DST##_body_roll_rate_SET( pattitude_target_body_roll_rate_GET( src  ), dst  );\
        DST##_body_pitch_rate_SET( pattitude_target_body_pitch_rate_GET( src  ), dst  );\
        DST##_body_yaw_rate_SET( pattitude_target_body_yaw_rate_GET( src  ), dst  );\
        DST##_thrust_SET( pattitude_target_thrust_GET( src  ), dst  );\
    }

/**
*This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
*				 The GCS can then request the individual mission item based on the knowledge of the total number of waypoints */

typedef Pack MISSION_COUNT_mission_count; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_count_MISSION_COUNT;// data navigator over pack fields data
/**
															* Wrap MISSION_COUNT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_count_MISSION_COUNT *MISSION_COUNT_mission_count_wrap(MISSION_COUNT_mission_count *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_COUNT_mission_count *pmission_count_MISSION_COUNT_unwrap(pmission_count_MISSION_COUNT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_count_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_count_MISSION_COUNT_PUSH_INTO(DST)\
    static inline void pmission_count_MISSION_COUNT_push_into_##DST ( pmission_count_MISSION_COUNT * src, DST * dst) {\
        DST##_count_SET( pmission_count_count_GET( src  ), dst  );\
        DST##_target_system_SET( pmission_count_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_count_target_component_GET( src  ), dst  );\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_count_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*The location and information of an ADSB vehicle */

typedef Pack ADSB_VEHICLE_adsb_vehicle; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor padsb_vehicle_ADSB_VEHICLE;// data navigator over pack fields data
/**
															* Wrap ADSB_VEHICLE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline padsb_vehicle_ADSB_VEHICLE *ADSB_VEHICLE_adsb_vehicle_wrap(ADSB_VEHICLE_adsb_vehicle *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline ADSB_VEHICLE_adsb_vehicle *padsb_vehicle_ADSB_VEHICLE_unwrap(padsb_vehicle_ADSB_VEHICLE *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vadsb_vehicle_callsign;
//Maximum field array length constant
#define Padsb_vehicle_callsign_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_heading_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_hor_velocity_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_squawk_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ICAO_address_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_ver_velocity_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_tslc_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_type_SET( e_ADSB_ALTITUDE_TYPE * src, <DST> * dst  ){}
static inline void <DST>_callsign_SET( Vadsb_vehicle_callsign * src, <DST> * dst  ){}
static inline void <DST>_emitter_type_SET( e_ADSB_EMITTER_TYPE * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_ADSB_FLAGS * src, <DST> * dst  ){}
*/

#define padsb_vehicle_ADSB_VEHICLE_PUSH_INTO(DST)\
    static inline void padsb_vehicle_ADSB_VEHICLE_push_into_##DST ( padsb_vehicle_ADSB_VEHICLE * src, DST * dst) {\
        DST##_heading_SET( padsb_vehicle_heading_GET( src  ), dst  );\
        DST##_hor_velocity_SET( padsb_vehicle_hor_velocity_GET( src  ), dst  );\
        DST##_squawk_SET( padsb_vehicle_squawk_GET( src  ), dst  );\
        DST##_ICAO_address_SET( padsb_vehicle_ICAO_address_GET( src  ), dst  );\
        DST##_lat_SET( padsb_vehicle_lat_GET( src  ), dst  );\
        DST##_lon_SET( padsb_vehicle_lon_GET( src  ), dst  );\
        DST##_altitude_SET( padsb_vehicle_altitude_GET( src  ), dst  );\
        DST##_ver_velocity_SET( padsb_vehicle_ver_velocity_GET( src  ), dst  );\
        DST##_tslc_SET( padsb_vehicle_tslc_GET( src  ), dst  );\
        e_ADSB_ALTITUDE_TYPE  item_altitude_type;\
        if( padsb_vehicle_altitude_type_GET( src, &item_altitude_type ) ){\
            DST##_altitude_type_SET( item_altitude_type , dst  );\
        }\
        Vadsb_vehicle_callsign  item_callsign;\
        if( padsb_vehicle_callsign_GET( src, &item_callsign ) ){\
            DST##_callsign_SET( &item_callsign, dst );\
        }\
        e_ADSB_EMITTER_TYPE  item_emitter_type;\
        if( padsb_vehicle_emitter_type_GET( src, &item_emitter_type ) ){\
            DST##_emitter_type_SET( item_emitter_type , dst  );\
        }\
        e_ADSB_FLAGS  item_flags;\
        if( padsb_vehicle_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_heading_GET( <SRC> * src ){}
static inline int16_t <SRC>_hor_velocity_GET( <SRC> * src ){}
static inline int16_t <SRC>_squawk_GET( <SRC> * src ){}
static inline int32_t <SRC>_ICAO_address_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline int32_t <SRC>_altitude_GET( <SRC> * src ){}
static inline int16_t <SRC>_ver_velocity_GET( <SRC> * src ){}
static inline int8_t <SRC>_tslc_GET( <SRC> * src ){}
static inline bool  <SRC>_altitude_type_item_exist( <SRC> * src ){}
static inline e_ADSB_ALTITUDE_TYPE <SRC>_altitude_type_GET( <SRC> * src ){}
static inline size_t  <SRC>_callsign_item_exist( <SRC> * src  ){}
static inline void <SRC>_callsign_GET( <SRC> * src, Vadsb_vehicle_callsign * dst ){}
static inline bool  <SRC>_emitter_type_item_exist( <SRC> * src ){}
static inline e_ADSB_EMITTER_TYPE <SRC>_emitter_type_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_ADSB_FLAGS <SRC>_flags_GET( <SRC> * src ){}
*/

#define padsb_vehicle_ADSB_VEHICLE_PULL_FROM(SRC)\
    static inline void padsb_vehicle_ADSB_VEHICLE_pull_from_##SRC ( SRC * src, padsb_vehicle_ADSB_VEHICLE * dst) {\
        padsb_vehicle_heading_SET( SRC##_heading_GET(src ), dst  );\
        padsb_vehicle_hor_velocity_SET( SRC##_hor_velocity_GET(src ), dst  );\
        padsb_vehicle_squawk_SET( SRC##_squawk_GET(src ), dst  );\
        padsb_vehicle_ICAO_address_SET( SRC##_ICAO_address_GET(src ), dst  );\
        padsb_vehicle_lat_SET( SRC##_lat_GET(src ), dst  );\
        padsb_vehicle_lon_SET( SRC##_lon_GET(src ), dst  );\
        padsb_vehicle_altitude_SET( SRC##_altitude_GET(src ), dst  );\
        padsb_vehicle_ver_velocity_SET( SRC##_ver_velocity_GET(src ), dst  );\
        padsb_vehicle_tslc_SET( SRC##_tslc_GET(src ), dst  );\
        if( SRC##_altitude_type_item_exist(src ) )\
        padsb_vehicle_altitude_type_SET( padsb_vehicle_ADSB_VEHICLE_altitude_type_GET( src ),  dst  );\
        const size_t len_callsign = SRC##_callsign_item_exist(src );\
        if( len_callsign ){\
            Vadsb_vehicle_callsign    item_callsign = padsb_vehicle_callsign_SET( NULL, len_callsign, dst  );\
            padsb_vehicle_ADSB_VEHICLE_callsign_GET(src, &item_callsign );\
        }\
        if( SRC##_emitter_type_item_exist(src ) )\
        padsb_vehicle_emitter_type_SET( padsb_vehicle_ADSB_VEHICLE_emitter_type_GET( src ),  dst  );\
        if( SRC##_flags_item_exist(src ) )\
        padsb_vehicle_flags_SET( padsb_vehicle_ADSB_VEHICLE_flags_GET( src ),  dst  );\
    }

/**
*This interface replaces DATA_STREAM */

typedef Pack MESSAGE_INTERVAL_message_interval; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmessage_interval_MESSAGE_INTERVAL;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmessage_interval_MESSAGE_INTERVAL *pmessage_interval_MESSAGE_INTERVAL_from(MESSAGE_INTERVAL_message_interval *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_message_id_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_interval_us_SET( int32_t * src, <DST> * dst  ){}
*/

#define pmessage_interval_MESSAGE_INTERVAL_PUSH_INTO(DST)\
    static inline void pmessage_interval_MESSAGE_INTERVAL_push_into_##DST ( pmessage_interval_MESSAGE_INTERVAL * src, DST * dst) {\
        DST##_message_id_SET( pmessage_interval_message_id_GET( src  ), dst  );\
        DST##_interval_us_SET( pmessage_interval_interval_us_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_message_id_GET( <SRC> * src ){}
static inline int32_t <SRC>_interval_us_GET( <SRC> * src ){}
*/

#define pmessage_interval_MESSAGE_INTERVAL_PULL_FROM(SRC)\
    static inline void pmessage_interval_MESSAGE_INTERVAL_pull_from_##SRC ( SRC * src, pmessage_interval_MESSAGE_INTERVAL * dst) {\
        pmessage_interval_message_id_SET( SRC##_message_id_GET(src ), dst  );\
        pmessage_interval_interval_us_SET( SRC##_interval_us_GET(src ), dst  );\
    }

/**
*EKF Status message including flags and variances */

typedef Pack EKF_STATUS_REPORT_ekf_status_report; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pekf_status_report_EKF_STATUS_REPORT;// data navigator over pack fields data
/**
															* Wrap EKF_STATUS_REPORT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pekf_status_report_EKF_STATUS_REPORT *EKF_STATUS_REPORT_ekf_status_report_wrap(EKF_STATUS_REPORT_ekf_status_report *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline EKF_STATUS_REPORT_ekf_status_report *pekf_status_report_EKF_STATUS_REPORT_unwrap(pekf_status_report_EKF_STATUS_REPORT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_velocity_variance_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pos_horiz_variance_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pos_vert_variance_SET( float * src, <DST> * dst  ){}
static inline void <DST>_compass_variance_SET( float * src, <DST> * dst  ){}
static inline void <DST>_terrain_alt_variance_SET( float * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_EKF_STATUS_FLAGS * src, <DST> * dst  ){}
*/

#define pekf_status_report_EKF_STATUS_REPORT_PUSH_INTO(DST)\
    static inline void pekf_status_report_EKF_STATUS_REPORT_push_into_##DST ( pekf_status_report_EKF_STATUS_REPORT * src, DST * dst) {\
        DST##_velocity_variance_SET( pekf_status_report_velocity_variance_GET( src  ), dst  );\
        DST##_pos_horiz_variance_SET( pekf_status_report_pos_horiz_variance_GET( src  ), dst  );\
        DST##_pos_vert_variance_SET( pekf_status_report_pos_vert_variance_GET( src  ), dst  );\
        DST##_compass_variance_SET( pekf_status_report_compass_variance_GET( src  ), dst  );\
        DST##_terrain_alt_variance_SET( pekf_status_report_terrain_alt_variance_GET( src  ), dst  );\
        e_EKF_STATUS_FLAGS  item_flags;\
        if( pekf_status_report_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_velocity_variance_GET( <SRC> * src ){}
static inline float <SRC>_pos_horiz_variance_GET( <SRC> * src ){}
static inline float <SRC>_pos_vert_variance_GET( <SRC> * src ){}
static inline float <SRC>_compass_variance_GET( <SRC> * src ){}
static inline float <SRC>_terrain_alt_variance_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_EKF_STATUS_FLAGS <SRC>_flags_GET( <SRC> * src ){}
*/

#define pekf_status_report_EKF_STATUS_REPORT_PULL_FROM(SRC)\
    static inline void pekf_status_report_EKF_STATUS_REPORT_pull_from_##SRC ( SRC * src, pekf_status_report_EKF_STATUS_REPORT * dst) {\
        pekf_status_report_velocity_variance_SET( SRC##_velocity_variance_GET(src ), dst  );\
        pekf_status_report_pos_horiz_variance_SET( SRC##_pos_horiz_variance_GET(src ), dst  );\
        pekf_status_report_pos_vert_variance_SET( SRC##_pos_vert_variance_GET(src ), dst  );\
        pekf_status_report_compass_variance_SET( SRC##_compass_variance_GET(src ), dst  );\
        pekf_status_report_terrain_alt_variance_SET( SRC##_terrain_alt_variance_GET(src ), dst  );\
        if( SRC##_flags_item_exist(src ) )\
        pekf_status_report_flags_SET( pekf_status_report_EKF_STATUS_REPORT_flags_GET( src ),  dst  );\
    }

/**
*Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
*				 is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
*				 enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation
*				 divided by the innovation check threshold. Under normal operation the innovaton test ratios should be
*				 below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation
*				 and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation
*				 test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should
*				 be optional and controllable by the user */

typedef Pack ESTIMATOR_STATUS_estimator_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pestimator_status_ESTIMATOR_STATUS;// data navigator over pack fields data
/**
															* Wrap ESTIMATOR_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pestimator_status_ESTIMATOR_STATUS *ESTIMATOR_STATUS_estimator_status_wrap(ESTIMATOR_STATUS_estimator_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline ESTIMATOR_STATUS_estimator_status *pestimator_status_ESTIMATOR_STATUS_unwrap(pestimator_status_ESTIMATOR_STATUS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_vel_ratio_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pos_horiz_ratio_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pos_vert_ratio_SET( float * src, <DST> * dst  ){}
static inline void <DST>_mag_ratio_SET( float * src, <DST> * dst  ){}
static inline void <DST>_hagl_ratio_SET( float * src, <DST> * dst  ){}
static inline void <DST>_tas_ratio_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pos_horiz_accuracy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pos_vert_accuracy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_ESTIMATOR_STATUS_FLAGS * src, <DST> * dst  ){}
*/

#define pestimator_status_ESTIMATOR_STATUS_PUSH_INTO(DST)\
    static inline void pestimator_status_ESTIMATOR_STATUS_push_into_##DST ( pestimator_status_ESTIMATOR_STATUS * src, DST * dst) {\
        DST##_time_usec_SET( pestimator_status_time_usec_GET( src  ), dst  );\
        DST##_vel_ratio_SET( pestimator_status_vel_ratio_GET( src  ), dst  );\
        DST##_pos_horiz_ratio_SET( pestimator_status_pos_horiz_ratio_GET( src  ), dst  );\
        DST##_pos_vert_ratio_SET( pestimator_status_pos_vert_ratio_GET( src  ), dst  );\
        DST##_mag_ratio_SET( pestimator_status_mag_ratio_GET( src  ), dst  );\
        DST##_hagl_ratio_SET( pestimator_status_hagl_ratio_GET( src  ), dst  );\
        DST##_tas_ratio_SET( pestimator_status_tas_ratio_GET( src  ), dst  );\
        DST##_pos_horiz_accuracy_SET( pestimator_status_pos_horiz_accuracy_GET( src  ), dst  );\
        DST##_pos_vert_accuracy_SET( pestimator_status_pos_vert_accuracy_GET( src  ), dst  );\
        e_ESTIMATOR_STATUS_FLAGS  item_flags;\
        if( pestimator_status_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_vel_ratio_GET( <SRC> * src ){}
static inline float <SRC>_pos_horiz_ratio_GET( <SRC> * src ){}
static inline float <SRC>_pos_vert_ratio_GET( <SRC> * src ){}
static inline float <SRC>_mag_ratio_GET( <SRC> * src ){}
static inline float <SRC>_hagl_ratio_GET( <SRC> * src ){}
static inline float <SRC>_tas_ratio_GET( <SRC> * src ){}
static inline float <SRC>_pos_horiz_accuracy_GET( <SRC> * src ){}
static inline float <SRC>_pos_vert_accuracy_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_ESTIMATOR_STATUS_FLAGS <SRC>_flags_GET( <SRC> * src ){}
*/

#define pestimator_status_ESTIMATOR_STATUS_PULL_FROM(SRC)\
    static inline void pestimator_status_ESTIMATOR_STATUS_pull_from_##SRC ( SRC * src, pestimator_status_ESTIMATOR_STATUS * dst) {\
        pestimator_status_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pestimator_status_vel_ratio_SET( SRC##_vel_ratio_GET(src ), dst  );\
        pestimator_status_pos_horiz_ratio_SET( SRC##_pos_horiz_ratio_GET(src ), dst  );\
        pestimator_status_pos_vert_ratio_SET( SRC##_pos_vert_ratio_GET(src ), dst  );\
        pestimator_status_mag_ratio_SET( SRC##_mag_ratio_GET(src ), dst  );\
        pestimator_status_hagl_ratio_SET( SRC##_hagl_ratio_GET(src ), dst  );\
        pestimator_status_tas_ratio_SET( SRC##_tas_ratio_GET(src ), dst  );\
        pestimator_status_pos_horiz_accuracy_SET( SRC##_pos_horiz_accuracy_GET(src ), dst  );\
        pestimator_status_pos_vert_accuracy_SET( SRC##_pos_vert_accuracy_GET(src ), dst  );\
        if( SRC##_flags_item_exist(src ) )\
        pestimator_status_flags_SET( pestimator_status_ESTIMATOR_STATUS_flags_GET( src ),  dst  );\
    }

/**
*Status of key hardware */

typedef Pack HWSTATUS_hwstatus; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t phwstatus_HWSTATUS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline phwstatus_HWSTATUS *phwstatus_HWSTATUS_from(HWSTATUS_hwstatus *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_Vcc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_I2Cerr_SET( int8_t * src, <DST> * dst  ){}
*/

#define phwstatus_HWSTATUS_PUSH_INTO(DST)\
    static inline void phwstatus_HWSTATUS_push_into_##DST ( phwstatus_HWSTATUS * src, DST * dst) {\
        DST##_Vcc_SET( phwstatus_Vcc_GET( src  ), dst  );\
        DST##_I2Cerr_SET( phwstatus_I2Cerr_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_Vcc_GET( <SRC> * src ){}
static inline int8_t <SRC>_I2Cerr_GET( <SRC> * src ){}
*/

#define phwstatus_HWSTATUS_PULL_FROM(SRC)\
    static inline void phwstatus_HWSTATUS_pull_from_##SRC ( SRC * src, phwstatus_HWSTATUS * dst) {\
        phwstatus_Vcc_SET( SRC##_Vcc_GET(src ), dst  );\
        phwstatus_I2Cerr_SET( SRC##_I2Cerr_GET(src ), dst  );\
    }

/**
*Time synchronization message. */

typedef Pack TIMESYNC_timesync; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t ptimesync_TIMESYNC;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline ptimesync_TIMESYNC *ptimesync_TIMESYNC_from(TIMESYNC_timesync *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_tc1_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_ts1_SET( int64_t * src, <DST> * dst  ){}
*/

#define ptimesync_TIMESYNC_PUSH_INTO(DST)\
    static inline void ptimesync_TIMESYNC_push_into_##DST ( ptimesync_TIMESYNC * src, DST * dst) {\
        DST##_tc1_SET( ptimesync_tc1_GET( src  ), dst  );\
        DST##_ts1_SET( ptimesync_ts1_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_tc1_GET( <SRC> * src ){}
static inline int64_t <SRC>_ts1_GET( <SRC> * src ){}
*/

#define ptimesync_TIMESYNC_PULL_FROM(SRC)\
    static inline void ptimesync_TIMESYNC_pull_from_##SRC ( SRC * src, ptimesync_TIMESYNC * dst) {\
        ptimesync_tc1_SET( SRC##_tc1_GET(src ), dst  );\
        ptimesync_ts1_SET( SRC##_ts1_GET(src ), dst  );\
    }

/**
*Request all parameters of this component. After this request, all parameters are emitted. */

typedef Pack PARAM_EXT_REQUEST_LIST_param_ext_request_list; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pparam_ext_request_list_PARAM_EXT_REQUEST_LIST;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *pparam_ext_request_list_PARAM_EXT_REQUEST_LIST_from(PARAM_EXT_REQUEST_LIST_param_ext_request_list *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define pparam_ext_request_list_PARAM_EXT_REQUEST_LIST_PUSH_INTO(DST)\
    static inline void pparam_ext_request_list_PARAM_EXT_REQUEST_LIST_push_into_##DST ( pparam_ext_request_list_PARAM_EXT_REQUEST_LIST * src, DST * dst) {\
        DST##_target_system_SET( pparam_ext_request_list_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pparam_ext_request_list_target_component_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
*/

#define pparam_ext_request_list_PARAM_EXT_REQUEST_LIST_PULL_FROM(SRC)\
    static inline void pparam_ext_request_list_PARAM_EXT_REQUEST_LIST_pull_from_##SRC ( SRC * src, pparam_ext_request_list_PARAM_EXT_REQUEST_LIST * dst) {\
        pparam_ext_request_list_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pparam_ext_request_list_target_component_SET( SRC##_target_component_GET(src ), dst  );\
    }

/**
*The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
*				 Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
*				 This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
*				 for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset */

typedef Pack GLOBAL_POSITION_INT_COV_global_position_int_cov; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pglobal_position_int_cov_GLOBAL_POSITION_INT_COV;// data navigator over pack fields data
/**
															* Wrap GLOBAL_POSITION_INT_COV in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *GLOBAL_POSITION_INT_COV_global_position_int_cov_wrap(GLOBAL_POSITION_INT_COV_global_position_int_cov *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GLOBAL_POSITION_INT_COV_global_position_int_cov *pglobal_position_int_cov_GLOBAL_POSITION_INT_COV_unwrap(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vglobal_position_int_cov_covariance;
//Maximum field array length constant
#define Pglobal_position_int_cov_covariance_len  ( 36 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_relative_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_covariance_SET( Vglobal_position_int_cov_covariance * src, <DST> * dst  ){}
static inline void <DST>_estimator_type_SET( e_MAV_ESTIMATOR_TYPE * src, <DST> * dst  ){}
*/

#define pglobal_position_int_cov_GLOBAL_POSITION_INT_COV_PUSH_INTO(DST)\
    static inline void pglobal_position_int_cov_GLOBAL_POSITION_INT_COV_push_into_##DST ( pglobal_position_int_cov_GLOBAL_POSITION_INT_COV * src, DST * dst) {\
        DST##_time_usec_SET( pglobal_position_int_cov_time_usec_GET( src  ), dst  );\
        DST##_lat_SET( pglobal_position_int_cov_lat_GET( src  ), dst  );\
        DST##_lon_SET( pglobal_position_int_cov_lon_GET( src  ), dst  );\
        DST##_alt_SET( pglobal_position_int_cov_alt_GET( src  ), dst  );\
        DST##_relative_alt_SET( pglobal_position_int_cov_relative_alt_GET( src  ), dst  );\
        DST##_vx_SET( pglobal_position_int_cov_vx_GET( src  ), dst  );\
        DST##_vy_SET( pglobal_position_int_cov_vy_GET( src  ), dst  );\
        DST##_vz_SET( pglobal_position_int_cov_vz_GET( src  ), dst  );\
        Vglobal_position_int_cov_covariance item_covariance = pglobal_position_int_cov_covariance_GET( src  );\
       DST##_covariance_SET( &item_covariance, dst );\
        e_MAV_ESTIMATOR_TYPE  item_estimator_type;\
        if( pglobal_position_int_cov_estimator_type_GET( src, &item_estimator_type ) ){\
            DST##_estimator_type_SET( item_estimator_type , dst  );\
        }\
    }

/**
*Report button state change */

typedef Pack BUTTON_CHANGE_button_change; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pbutton_change_BUTTON_CHANGE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pbutton_change_BUTTON_CHANGE *pbutton_change_BUTTON_CHANGE_from(BUTTON_CHANGE_button_change *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_last_change_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_state_SET( int8_t * src, <DST> * dst  ){}
*/

#define pbutton_change_BUTTON_CHANGE_PUSH_INTO(DST)\
    static inline void pbutton_change_BUTTON_CHANGE_push_into_##DST ( pbutton_change_BUTTON_CHANGE * src, DST * dst) {\
        DST##_time_boot_ms_SET( pbutton_change_time_boot_ms_GET( src  ), dst  );\
        DST##_last_change_ms_SET( pbutton_change_last_change_ms_GET( src  ), dst  );\
        DST##_state_SET( pbutton_change_state_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int32_t <SRC>_last_change_ms_GET( <SRC> * src ){}
static inline int8_t <SRC>_state_GET( <SRC> * src ){}
*/

#define pbutton_change_BUTTON_CHANGE_PULL_FROM(SRC)\
    static inline void pbutton_change_BUTTON_CHANGE_pull_from_##SRC ( SRC * src, pbutton_change_BUTTON_CHANGE * dst) {\
        pbutton_change_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pbutton_change_last_change_ms_SET( SRC##_last_change_ms_GET(src ), dst  );\
        pbutton_change_state_SET( SRC##_state_GET(src ), dst  );\
    }

/**
*Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
*				 the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
*				 or competition regulations */

typedef Pack SAFETY_SET_ALLOWED_AREA_safety_set_allowed_area; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA;// data navigator over pack fields data
/**
															* Wrap SAFETY_SET_ALLOWED_AREA in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *SAFETY_SET_ALLOWED_AREA_safety_set_allowed_area_wrap(SAFETY_SET_ALLOWED_AREA_safety_set_allowed_area *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SAFETY_SET_ALLOWED_AREA_safety_set_allowed_area *psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA_unwrap(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_p1x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p1y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p1z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p2x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p2y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p2z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
*/

#define psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA_PUSH_INTO(DST)\
    static inline void psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA_push_into_##DST ( psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA * src, DST * dst) {\
        DST##_target_system_SET( psafety_set_allowed_area_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( psafety_set_allowed_area_target_component_GET( src  ), dst  );\
        DST##_p1x_SET( psafety_set_allowed_area_p1x_GET( src  ), dst  );\
        DST##_p1y_SET( psafety_set_allowed_area_p1y_GET( src  ), dst  );\
        DST##_p1z_SET( psafety_set_allowed_area_p1z_GET( src  ), dst  );\
        DST##_p2x_SET( psafety_set_allowed_area_p2x_GET( src  ), dst  );\
        DST##_p2y_SET( psafety_set_allowed_area_p2y_GET( src  ), dst  );\
        DST##_p2z_SET( psafety_set_allowed_area_p2z_GET( src  ), dst  );\
        e_MAV_FRAME  item_frame;\
        if( psafety_set_allowed_area_frame_GET( src, &item_frame ) ){\
            DST##_frame_SET( item_frame , dst  );\
        }\
    }

/**
*General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
*				 for the background information. The UAVCAN specification is available at http:uavcan.org */

typedef Pack UAVCAN_NODE_STATUS_uavcan_node_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor puavcan_node_status_UAVCAN_NODE_STATUS;// data navigator over pack fields data
/**
															* Wrap UAVCAN_NODE_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline puavcan_node_status_UAVCAN_NODE_STATUS *UAVCAN_NODE_STATUS_uavcan_node_status_wrap(UAVCAN_NODE_STATUS_uavcan_node_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline UAVCAN_NODE_STATUS_uavcan_node_status *puavcan_node_status_UAVCAN_NODE_STATUS_unwrap(puavcan_node_status_UAVCAN_NODE_STATUS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_vendor_specific_status_code_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_uptime_sec_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_sub_mode_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_health_SET( e_UAVCAN_NODE_HEALTH * src, <DST> * dst  ){}
static inline void <DST>_mode_SET( e_UAVCAN_NODE_MODE * src, <DST> * dst  ){}
*/

#define puavcan_node_status_UAVCAN_NODE_STATUS_PUSH_INTO(DST)\
    static inline void puavcan_node_status_UAVCAN_NODE_STATUS_push_into_##DST ( puavcan_node_status_UAVCAN_NODE_STATUS * src, DST * dst) {\
        DST##_vendor_specific_status_code_SET( puavcan_node_status_vendor_specific_status_code_GET( src  ), dst  );\
        DST##_uptime_sec_SET( puavcan_node_status_uptime_sec_GET( src  ), dst  );\
        DST##_time_usec_SET( puavcan_node_status_time_usec_GET( src  ), dst  );\
        DST##_sub_mode_SET( puavcan_node_status_sub_mode_GET( src  ), dst  );\
        e_UAVCAN_NODE_HEALTH  item_health;\
        if( puavcan_node_status_health_GET( src, &item_health ) ){\
            DST##_health_SET( item_health , dst  );\
        }\
        e_UAVCAN_NODE_MODE  item_mode;\
        if( puavcan_node_status_mode_GET( src, &item_mode ) ){\
            DST##_mode_SET( item_mode , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_vendor_specific_status_code_GET( <SRC> * src ){}
static inline int32_t <SRC>_uptime_sec_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_sub_mode_GET( <SRC> * src ){}
static inline bool  <SRC>_health_item_exist( <SRC> * src ){}
static inline e_UAVCAN_NODE_HEALTH <SRC>_health_GET( <SRC> * src ){}
static inline bool  <SRC>_mode_item_exist( <SRC> * src ){}
static inline e_UAVCAN_NODE_MODE <SRC>_mode_GET( <SRC> * src ){}
*/

#define puavcan_node_status_UAVCAN_NODE_STATUS_PULL_FROM(SRC)\
    static inline void puavcan_node_status_UAVCAN_NODE_STATUS_pull_from_##SRC ( SRC * src, puavcan_node_status_UAVCAN_NODE_STATUS * dst) {\
        puavcan_node_status_vendor_specific_status_code_SET( SRC##_vendor_specific_status_code_GET(src ), dst  );\
        puavcan_node_status_uptime_sec_SET( SRC##_uptime_sec_GET(src ), dst  );\
        puavcan_node_status_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        puavcan_node_status_sub_mode_SET( SRC##_sub_mode_GET(src ), dst  );\
        if( SRC##_health_item_exist(src ) )\
        puavcan_node_status_health_SET( puavcan_node_status_UAVCAN_NODE_STATUS_health_GET( src ),  dst  );\
        if( SRC##_mode_item_exist(src ) )\
        puavcan_node_status_mode_SET( puavcan_node_status_UAVCAN_NODE_STATUS_mode_GET( src ),  dst  );\
    }

/**
*Information about a potential collision */

typedef Pack COLLISION_collision; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcollision_COLLISION;// data navigator over pack fields data
/**
															* Wrap COLLISION in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcollision_COLLISION *COLLISION_collision_wrap(COLLISION_collision *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline COLLISION_collision *pcollision_COLLISION_unwrap(pcollision_COLLISION *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_id_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_to_minimum_delta_SET( float * src, <DST> * dst  ){}
static inline void <DST>_altitude_minimum_delta_SET( float * src, <DST> * dst  ){}
static inline void <DST>_horizontal_minimum_delta_SET( float * src, <DST> * dst  ){}
static inline void <DST>_sRc_SET( e_MAV_COLLISION_SRC * src, <DST> * dst  ){}
static inline void <DST>_action_SET( e_MAV_COLLISION_ACTION * src, <DST> * dst  ){}
static inline void <DST>_threat_level_SET( e_MAV_COLLISION_THREAT_LEVEL * src, <DST> * dst  ){}
*/

#define pcollision_COLLISION_PUSH_INTO(DST)\
    static inline void pcollision_COLLISION_push_into_##DST ( pcollision_COLLISION * src, DST * dst) {\
        DST##_id_SET( pcollision_id_GET( src  ), dst  );\
        DST##_time_to_minimum_delta_SET( pcollision_time_to_minimum_delta_GET( src  ), dst  );\
        DST##_altitude_minimum_delta_SET( pcollision_altitude_minimum_delta_GET( src  ), dst  );\
        DST##_horizontal_minimum_delta_SET( pcollision_horizontal_minimum_delta_GET( src  ), dst  );\
        e_MAV_COLLISION_SRC  item_sRc;\
        if( pcollision_sRc_GET( src, &item_sRc ) ){\
            DST##_sRc_SET( item_sRc , dst  );\
        }\
        e_MAV_COLLISION_ACTION  item_action;\
        if( pcollision_action_GET( src, &item_action ) ){\
            DST##_action_SET( item_action , dst  );\
        }\
        e_MAV_COLLISION_THREAT_LEVEL  item_threat_level;\
        if( pcollision_threat_level_GET( src, &item_threat_level ) ){\
            DST##_threat_level_SET( item_threat_level , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_id_GET( <SRC> * src ){}
static inline float <SRC>_time_to_minimum_delta_GET( <SRC> * src ){}
static inline float <SRC>_altitude_minimum_delta_GET( <SRC> * src ){}
static inline float <SRC>_horizontal_minimum_delta_GET( <SRC> * src ){}
static inline bool  <SRC>_sRc_item_exist( <SRC> * src ){}
static inline e_MAV_COLLISION_SRC <SRC>_sRc_GET( <SRC> * src ){}
static inline bool  <SRC>_action_item_exist( <SRC> * src ){}
static inline e_MAV_COLLISION_ACTION <SRC>_action_GET( <SRC> * src ){}
static inline bool  <SRC>_threat_level_item_exist( <SRC> * src ){}
static inline e_MAV_COLLISION_THREAT_LEVEL <SRC>_threat_level_GET( <SRC> * src ){}
*/

#define pcollision_COLLISION_PULL_FROM(SRC)\
    static inline void pcollision_COLLISION_pull_from_##SRC ( SRC * src, pcollision_COLLISION * dst) {\
        pcollision_id_SET( SRC##_id_GET(src ), dst  );\
        pcollision_time_to_minimum_delta_SET( SRC##_time_to_minimum_delta_GET(src ), dst  );\
        pcollision_altitude_minimum_delta_SET( SRC##_altitude_minimum_delta_GET(src ), dst  );\
        pcollision_horizontal_minimum_delta_SET( SRC##_horizontal_minimum_delta_GET(src ), dst  );\
        if( SRC##_sRc_item_exist(src ) )\
        pcollision_sRc_SET( pcollision_COLLISION_sRc_GET( src ),  dst  );\
        if( SRC##_action_item_exist(src ) )\
        pcollision_action_SET( pcollision_COLLISION_action_GET( src ),  dst  );\
        if( SRC##_threat_level_item_exist(src ) )\
        pcollision_threat_level_SET( pcollision_COLLISION_threat_level_GET( src ),  dst  );\
    }

/**
*100 Hz gimbal torque command telemetry */

typedef Pack GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT_from(GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_rl_torque_cmd_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_el_torque_cmd_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_az_torque_cmd_SET( int16_t * src, <DST> * dst  ){}
*/

#define pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT_PUSH_INTO(DST)\
    static inline void pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT_push_into_##DST ( pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT * src, DST * dst) {\
        DST##_target_system_SET( pgimbal_torque_cmd_report_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pgimbal_torque_cmd_report_target_component_GET( src  ), dst  );\
        DST##_rl_torque_cmd_SET( pgimbal_torque_cmd_report_rl_torque_cmd_GET( src  ), dst  );\
        DST##_el_torque_cmd_SET( pgimbal_torque_cmd_report_el_torque_cmd_GET( src  ), dst  );\
        DST##_az_torque_cmd_SET( pgimbal_torque_cmd_report_az_torque_cmd_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int16_t <SRC>_rl_torque_cmd_GET( <SRC> * src ){}
static inline int16_t <SRC>_el_torque_cmd_GET( <SRC> * src ){}
static inline int16_t <SRC>_az_torque_cmd_GET( <SRC> * src ){}
*/

#define pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT_PULL_FROM(SRC)\
    static inline void pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT_pull_from_##SRC ( SRC * src, pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT * dst) {\
        pgimbal_torque_cmd_report_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pgimbal_torque_cmd_report_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pgimbal_torque_cmd_report_rl_torque_cmd_SET( SRC##_rl_torque_cmd_GET(src ), dst  );\
        pgimbal_torque_cmd_report_el_torque_cmd_SET( SRC##_el_torque_cmd_GET(src ), dst  );\
        pgimbal_torque_cmd_report_az_torque_cmd_SET( SRC##_az_torque_cmd_GET(src ), dst  );\
    }

/**
*The current system altitude. */

typedef Pack ALTITUDE_altitude; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t paltitude_ALTITUDE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline paltitude_ALTITUDE *paltitude_ALTITUDE_from(ALTITUDE_altitude *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_monotonic_SET( float * src, <DST> * dst  ){}
static inline void <DST>_altitude_amsl_SET( float * src, <DST> * dst  ){}
static inline void <DST>_altitude_local_SET( float * src, <DST> * dst  ){}
static inline void <DST>_altitude_relative_SET( float * src, <DST> * dst  ){}
static inline void <DST>_altitude_terrain_SET( float * src, <DST> * dst  ){}
static inline void <DST>_bottom_clearance_SET( float * src, <DST> * dst  ){}
*/

#define paltitude_ALTITUDE_PUSH_INTO(DST)\
    static inline void paltitude_ALTITUDE_push_into_##DST ( paltitude_ALTITUDE * src, DST * dst) {\
        DST##_time_usec_SET( paltitude_time_usec_GET( src  ), dst  );\
        DST##_altitude_monotonic_SET( paltitude_altitude_monotonic_GET( src  ), dst  );\
        DST##_altitude_amsl_SET( paltitude_altitude_amsl_GET( src  ), dst  );\
        DST##_altitude_local_SET( paltitude_altitude_local_GET( src  ), dst  );\
        DST##_altitude_relative_SET( paltitude_altitude_relative_GET( src  ), dst  );\
        DST##_altitude_terrain_SET( paltitude_altitude_terrain_GET( src  ), dst  );\
        DST##_bottom_clearance_SET( paltitude_bottom_clearance_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_altitude_monotonic_GET( <SRC> * src ){}
static inline float <SRC>_altitude_amsl_GET( <SRC> * src ){}
static inline float <SRC>_altitude_local_GET( <SRC> * src ){}
static inline float <SRC>_altitude_relative_GET( <SRC> * src ){}
static inline float <SRC>_altitude_terrain_GET( <SRC> * src ){}
static inline float <SRC>_bottom_clearance_GET( <SRC> * src ){}
*/

#define paltitude_ALTITUDE_PULL_FROM(SRC)\
    static inline void paltitude_ALTITUDE_pull_from_##SRC ( SRC * src, paltitude_ALTITUDE * dst) {\
        paltitude_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        paltitude_altitude_monotonic_SET( SRC##_altitude_monotonic_GET(src ), dst  );\
        paltitude_altitude_amsl_SET( SRC##_altitude_amsl_GET(src ), dst  );\
        paltitude_altitude_local_SET( SRC##_altitude_local_GET(src ), dst  );\
        paltitude_altitude_relative_SET( SRC##_altitude_relative_GET(src ), dst  );\
        paltitude_altitude_terrain_SET( SRC##_altitude_terrain_GET(src ), dst  );\
        paltitude_bottom_clearance_SET( SRC##_bottom_clearance_GET(src ), dst  );\
    }

/**
*Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
*				 for high throughput applications such as hardware in the loop simulations */

typedef Pack HIL_STATE_QUATERNION_hil_state_quaternion; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t phil_state_quaternion_HIL_STATE_QUATERNION;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline phil_state_quaternion_HIL_STATE_QUATERNION *phil_state_quaternion_HIL_STATE_QUATERNION_from(HIL_STATE_QUATERNION_hil_state_quaternion *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vhil_state_quaternion_attitude_quaternion;
//Maximum field array length constant
#define Phil_state_quaternion_attitude_quaternion_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_ind_airspeed_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_true_airspeed_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_attitude_quaternion_SET( Vhil_state_quaternion_attitude_quaternion * src, <DST> * dst  ){}
static inline void <DST>_rollspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitchspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yawspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( int16_t * src, <DST> * dst  ){}
*/

#define phil_state_quaternion_HIL_STATE_QUATERNION_PUSH_INTO(DST)\
    static inline void phil_state_quaternion_HIL_STATE_QUATERNION_push_into_##DST ( phil_state_quaternion_HIL_STATE_QUATERNION * src, DST * dst) {\
        DST##_ind_airspeed_SET( phil_state_quaternion_ind_airspeed_GET( src  ), dst  );\
        DST##_true_airspeed_SET( phil_state_quaternion_true_airspeed_GET( src  ), dst  );\
        DST##_time_usec_SET( phil_state_quaternion_time_usec_GET( src  ), dst  );\
        Vhil_state_quaternion_attitude_quaternion item_attitude_quaternion = phil_state_quaternion_attitude_quaternion_GET( src  );\
       DST##_attitude_quaternion_SET( &item_attitude_quaternion, dst );\
        DST##_rollspeed_SET( phil_state_quaternion_rollspeed_GET( src  ), dst  );\
        DST##_pitchspeed_SET( phil_state_quaternion_pitchspeed_GET( src  ), dst  );\
        DST##_yawspeed_SET( phil_state_quaternion_yawspeed_GET( src  ), dst  );\
        DST##_lat_SET( phil_state_quaternion_lat_GET( src  ), dst  );\
        DST##_lon_SET( phil_state_quaternion_lon_GET( src  ), dst  );\
        DST##_alt_SET( phil_state_quaternion_alt_GET( src  ), dst  );\
        DST##_vx_SET( phil_state_quaternion_vx_GET( src  ), dst  );\
        DST##_vy_SET( phil_state_quaternion_vy_GET( src  ), dst  );\
        DST##_vz_SET( phil_state_quaternion_vz_GET( src  ), dst  );\
        DST##_xacc_SET( phil_state_quaternion_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( phil_state_quaternion_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( phil_state_quaternion_zacc_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_ind_airspeed_GET( <SRC> * src ){}
static inline int16_t <SRC>_true_airspeed_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_attitude_quaternion_GET( <SRC> * src, Vhil_state_quaternion_attitude_quaternion * dst ){}
static inline float <SRC>_rollspeed_GET( <SRC> * src ){}
static inline float <SRC>_pitchspeed_GET( <SRC> * src ){}
static inline float <SRC>_yawspeed_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline int32_t <SRC>_alt_GET( <SRC> * src ){}
static inline int16_t <SRC>_vx_GET( <SRC> * src ){}
static inline int16_t <SRC>_vy_GET( <SRC> * src ){}
static inline int16_t <SRC>_vz_GET( <SRC> * src ){}
static inline int16_t <SRC>_xacc_GET( <SRC> * src ){}
static inline int16_t <SRC>_yacc_GET( <SRC> * src ){}
static inline int16_t <SRC>_zacc_GET( <SRC> * src ){}
*/

#define phil_state_quaternion_HIL_STATE_QUATERNION_PULL_FROM(SRC)\
    static inline void phil_state_quaternion_HIL_STATE_QUATERNION_pull_from_##SRC ( SRC * src, phil_state_quaternion_HIL_STATE_QUATERNION * dst) {\
        phil_state_quaternion_ind_airspeed_SET( SRC##_ind_airspeed_GET(src ), dst  );\
        phil_state_quaternion_true_airspeed_SET( SRC##_true_airspeed_GET(src ), dst  );\
        phil_state_quaternion_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
       Vhil_state_quaternion_attitude_quaternion item_attitude_quaternion = phil_state_quaternion_attitude_quaternion_SET( NULL, dst  );\
       SRC##_attitude_quaternion_GET( src, &item_attitude_quaternion );\
        phil_state_quaternion_rollspeed_SET( SRC##_rollspeed_GET(src ), dst  );\
        phil_state_quaternion_pitchspeed_SET( SRC##_pitchspeed_GET(src ), dst  );\
        phil_state_quaternion_yawspeed_SET( SRC##_yawspeed_GET(src ), dst  );\
        phil_state_quaternion_lat_SET( SRC##_lat_GET(src ), dst  );\
        phil_state_quaternion_lon_SET( SRC##_lon_GET(src ), dst  );\
        phil_state_quaternion_alt_SET( SRC##_alt_GET(src ), dst  );\
        phil_state_quaternion_vx_SET( SRC##_vx_GET(src ), dst  );\
        phil_state_quaternion_vy_SET( SRC##_vy_GET(src ), dst  );\
        phil_state_quaternion_vz_SET( SRC##_vz_GET(src ), dst  );\
        phil_state_quaternion_xacc_SET( SRC##_xacc_GET(src ), dst  );\
        phil_state_quaternion_yacc_SET( SRC##_yacc_GET(src ), dst  );\
        phil_state_quaternion_zacc_SET( SRC##_zacc_GET(src ), dst  );\
    }

/**
*Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process */

typedef Pack SENSOR_OFFSETS_sensor_offsets; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t psensor_offsets_SENSOR_OFFSETS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline psensor_offsets_SENSOR_OFFSETS *psensor_offsets_SENSOR_OFFSETS_from(SENSOR_OFFSETS_sensor_offsets *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_mag_ofs_x_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_mag_ofs_y_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_mag_ofs_z_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_mag_declination_SET( float * src, <DST> * dst  ){}
static inline void <DST>_raw_press_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_raw_temp_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_gyro_cal_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_gyro_cal_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_gyro_cal_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_accel_cal_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_accel_cal_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_accel_cal_z_SET( float * src, <DST> * dst  ){}
*/

#define psensor_offsets_SENSOR_OFFSETS_PUSH_INTO(DST)\
    static inline void psensor_offsets_SENSOR_OFFSETS_push_into_##DST ( psensor_offsets_SENSOR_OFFSETS * src, DST * dst) {\
        DST##_mag_ofs_x_SET( psensor_offsets_mag_ofs_x_GET( src  ), dst  );\
        DST##_mag_ofs_y_SET( psensor_offsets_mag_ofs_y_GET( src  ), dst  );\
        DST##_mag_ofs_z_SET( psensor_offsets_mag_ofs_z_GET( src  ), dst  );\
        DST##_mag_declination_SET( psensor_offsets_mag_declination_GET( src  ), dst  );\
        DST##_raw_press_SET( psensor_offsets_raw_press_GET( src  ), dst  );\
        DST##_raw_temp_SET( psensor_offsets_raw_temp_GET( src  ), dst  );\
        DST##_gyro_cal_x_SET( psensor_offsets_gyro_cal_x_GET( src  ), dst  );\
        DST##_gyro_cal_y_SET( psensor_offsets_gyro_cal_y_GET( src  ), dst  );\
        DST##_gyro_cal_z_SET( psensor_offsets_gyro_cal_z_GET( src  ), dst  );\
        DST##_accel_cal_x_SET( psensor_offsets_accel_cal_x_GET( src  ), dst  );\
        DST##_accel_cal_y_SET( psensor_offsets_accel_cal_y_GET( src  ), dst  );\
        DST##_accel_cal_z_SET( psensor_offsets_accel_cal_z_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_mag_ofs_x_GET( <SRC> * src ){}
static inline int16_t <SRC>_mag_ofs_y_GET( <SRC> * src ){}
static inline int16_t <SRC>_mag_ofs_z_GET( <SRC> * src ){}
static inline float <SRC>_mag_declination_GET( <SRC> * src ){}
static inline int32_t <SRC>_raw_press_GET( <SRC> * src ){}
static inline int32_t <SRC>_raw_temp_GET( <SRC> * src ){}
static inline float <SRC>_gyro_cal_x_GET( <SRC> * src ){}
static inline float <SRC>_gyro_cal_y_GET( <SRC> * src ){}
static inline float <SRC>_gyro_cal_z_GET( <SRC> * src ){}
static inline float <SRC>_accel_cal_x_GET( <SRC> * src ){}
static inline float <SRC>_accel_cal_y_GET( <SRC> * src ){}
static inline float <SRC>_accel_cal_z_GET( <SRC> * src ){}
*/

#define psensor_offsets_SENSOR_OFFSETS_PULL_FROM(SRC)\
    static inline void psensor_offsets_SENSOR_OFFSETS_pull_from_##SRC ( SRC * src, psensor_offsets_SENSOR_OFFSETS * dst) {\
        psensor_offsets_mag_ofs_x_SET( SRC##_mag_ofs_x_GET(src ), dst  );\
        psensor_offsets_mag_ofs_y_SET( SRC##_mag_ofs_y_GET(src ), dst  );\
        psensor_offsets_mag_ofs_z_SET( SRC##_mag_ofs_z_GET(src ), dst  );\
        psensor_offsets_mag_declination_SET( SRC##_mag_declination_GET(src ), dst  );\
        psensor_offsets_raw_press_SET( SRC##_raw_press_GET(src ), dst  );\
        psensor_offsets_raw_temp_SET( SRC##_raw_temp_GET(src ), dst  );\
        psensor_offsets_gyro_cal_x_SET( SRC##_gyro_cal_x_GET(src ), dst  );\
        psensor_offsets_gyro_cal_y_SET( SRC##_gyro_cal_y_GET(src ), dst  );\
        psensor_offsets_gyro_cal_z_SET( SRC##_gyro_cal_z_GET(src ), dst  );\
        psensor_offsets_accel_cal_x_SET( SRC##_accel_cal_x_GET(src ), dst  );\
        psensor_offsets_accel_cal_y_SET( SRC##_accel_cal_y_GET(src ), dst  );\
        psensor_offsets_accel_cal_z_SET( SRC##_accel_cal_z_GET(src ), dst  );\
    }

/**
*WIP: Information about a storage medium. */

typedef Pack STORAGE_INFORMATION_storage_information; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pstorage_information_STORAGE_INFORMATION;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pstorage_information_STORAGE_INFORMATION *pstorage_information_STORAGE_INFORMATION_from(STORAGE_INFORMATION_storage_information *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_storage_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_storage_count_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_status_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_total_capacity_SET( float * src, <DST> * dst  ){}
static inline void <DST>_used_capacity_SET( float * src, <DST> * dst  ){}
static inline void <DST>_available_capacity_SET( float * src, <DST> * dst  ){}
static inline void <DST>_read_speed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_write_speed_SET( float * src, <DST> * dst  ){}
*/

#define pstorage_information_STORAGE_INFORMATION_PUSH_INTO(DST)\
    static inline void pstorage_information_STORAGE_INFORMATION_push_into_##DST ( pstorage_information_STORAGE_INFORMATION * src, DST * dst) {\
        DST##_time_boot_ms_SET( pstorage_information_time_boot_ms_GET( src  ), dst  );\
        DST##_storage_id_SET( pstorage_information_storage_id_GET( src  ), dst  );\
        DST##_storage_count_SET( pstorage_information_storage_count_GET( src  ), dst  );\
        DST##_status_SET( pstorage_information_status_GET( src  ), dst  );\
        DST##_total_capacity_SET( pstorage_information_total_capacity_GET( src  ), dst  );\
        DST##_used_capacity_SET( pstorage_information_used_capacity_GET( src  ), dst  );\
        DST##_available_capacity_SET( pstorage_information_available_capacity_GET( src  ), dst  );\
        DST##_read_speed_SET( pstorage_information_read_speed_GET( src  ), dst  );\
        DST##_write_speed_SET( pstorage_information_write_speed_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int8_t <SRC>_storage_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_storage_count_GET( <SRC> * src ){}
static inline int8_t <SRC>_status_GET( <SRC> * src ){}
static inline float <SRC>_total_capacity_GET( <SRC> * src ){}
static inline float <SRC>_used_capacity_GET( <SRC> * src ){}
static inline float <SRC>_available_capacity_GET( <SRC> * src ){}
static inline float <SRC>_read_speed_GET( <SRC> * src ){}
static inline float <SRC>_write_speed_GET( <SRC> * src ){}
*/

#define pstorage_information_STORAGE_INFORMATION_PULL_FROM(SRC)\
    static inline void pstorage_information_STORAGE_INFORMATION_pull_from_##SRC ( SRC * src, pstorage_information_STORAGE_INFORMATION * dst) {\
        pstorage_information_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pstorage_information_storage_id_SET( SRC##_storage_id_GET(src ), dst  );\
        pstorage_information_storage_count_SET( SRC##_storage_count_GET(src ), dst  );\
        pstorage_information_status_SET( SRC##_status_GET(src ), dst  );\
        pstorage_information_total_capacity_SET( SRC##_total_capacity_GET(src ), dst  );\
        pstorage_information_used_capacity_SET( SRC##_used_capacity_GET(src ), dst  );\
        pstorage_information_available_capacity_SET( SRC##_available_capacity_GET(src ), dst  );\
        pstorage_information_read_speed_SET( SRC##_read_speed_GET(src ), dst  );\
        pstorage_information_write_speed_SET( SRC##_write_speed_GET(src ), dst  );\
    }

/**
*WIP: Information about a camera */

typedef Pack CAMERA_INFORMATION_camera_information; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcamera_information_CAMERA_INFORMATION;// data navigator over pack fields data
/**
															* Wrap CAMERA_INFORMATION in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcamera_information_CAMERA_INFORMATION *CAMERA_INFORMATION_camera_information_wrap(CAMERA_INFORMATION_camera_information *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline CAMERA_INFORMATION_camera_information *pcamera_information_CAMERA_INFORMATION_unwrap(pcamera_information_CAMERA_INFORMATION *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vcamera_information_vendor_name;
//Maximum field array length constant
#define Pcamera_information_vendor_name_len  ( 32 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vcamera_information_model_name;
//Maximum field array length constant
#define Pcamera_information_model_name_len  ( 32 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vcamera_information_cam_definition_uri;
//Maximum field array length constant
#define Pcamera_information_cam_definition_uri_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_resolution_h_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_resolution_v_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_cam_definition_version_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_firmware_version_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_vendor_name_SET( Vcamera_information_vendor_name * src, <DST> * dst  ){}
static inline void <DST>_model_name_SET( Vcamera_information_model_name * src, <DST> * dst  ){}
static inline void <DST>_focal_length_SET( float * src, <DST> * dst  ){}
static inline void <DST>_sensor_size_h_SET( float * src, <DST> * dst  ){}
static inline void <DST>_sensor_size_v_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lens_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_CAMERA_CAP_FLAGS * src, <DST> * dst  ){}
static inline void <DST>_cam_definition_uri_SET( Vcamera_information_cam_definition_uri * src, <DST> * dst  ){}
*/

#define pcamera_information_CAMERA_INFORMATION_PUSH_INTO(DST)\
    static inline void pcamera_information_CAMERA_INFORMATION_push_into_##DST ( pcamera_information_CAMERA_INFORMATION * src, DST * dst) {\
        DST##_resolution_h_SET( pcamera_information_resolution_h_GET( src  ), dst  );\
        DST##_resolution_v_SET( pcamera_information_resolution_v_GET( src  ), dst  );\
        DST##_cam_definition_version_SET( pcamera_information_cam_definition_version_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( pcamera_information_time_boot_ms_GET( src  ), dst  );\
        DST##_firmware_version_SET( pcamera_information_firmware_version_GET( src  ), dst  );\
        Vcamera_information_vendor_name item_vendor_name = pcamera_information_vendor_name_GET( src  );\
       DST##_vendor_name_SET( &item_vendor_name, dst );\
        Vcamera_information_model_name item_model_name = pcamera_information_model_name_GET( src  );\
       DST##_model_name_SET( &item_model_name, dst );\
        DST##_focal_length_SET( pcamera_information_focal_length_GET( src  ), dst  );\
        DST##_sensor_size_h_SET( pcamera_information_sensor_size_h_GET( src  ), dst  );\
        DST##_sensor_size_v_SET( pcamera_information_sensor_size_v_GET( src  ), dst  );\
        DST##_lens_id_SET( pcamera_information_lens_id_GET( src  ), dst  );\
        e_CAMERA_CAP_FLAGS  item_flags;\
        if( pcamera_information_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
        Vcamera_information_cam_definition_uri  item_cam_definition_uri;\
        if( pcamera_information_cam_definition_uri_GET( src, &item_cam_definition_uri ) ){\
            DST##_cam_definition_uri_SET( &item_cam_definition_uri, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_resolution_h_GET( <SRC> * src ){}
static inline int16_t <SRC>_resolution_v_GET( <SRC> * src ){}
static inline int16_t <SRC>_cam_definition_version_GET( <SRC> * src ){}
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int32_t <SRC>_firmware_version_GET( <SRC> * src ){}
static inline int8_t <SRC>_vendor_name_GET( <SRC> * src, Vcamera_information_vendor_name * dst ){}
static inline int8_t <SRC>_model_name_GET( <SRC> * src, Vcamera_information_model_name * dst ){}
static inline float <SRC>_focal_length_GET( <SRC> * src ){}
static inline float <SRC>_sensor_size_h_GET( <SRC> * src ){}
static inline float <SRC>_sensor_size_v_GET( <SRC> * src ){}
static inline int8_t <SRC>_lens_id_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_CAMERA_CAP_FLAGS <SRC>_flags_GET( <SRC> * src ){}
static inline size_t  <SRC>_cam_definition_uri_item_exist( <SRC> * src  ){}
static inline void <SRC>_cam_definition_uri_GET( <SRC> * src, Vcamera_information_cam_definition_uri * dst ){}
*/

#define pcamera_information_CAMERA_INFORMATION_PULL_FROM(SRC)\
    static inline void pcamera_information_CAMERA_INFORMATION_pull_from_##SRC ( SRC * src, pcamera_information_CAMERA_INFORMATION * dst) {\
        pcamera_information_resolution_h_SET( SRC##_resolution_h_GET(src ), dst  );\
        pcamera_information_resolution_v_SET( SRC##_resolution_v_GET(src ), dst  );\
        pcamera_information_cam_definition_version_SET( SRC##_cam_definition_version_GET(src ), dst  );\
        pcamera_information_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pcamera_information_firmware_version_SET( SRC##_firmware_version_GET(src ), dst  );\
       Vcamera_information_vendor_name item_vendor_name = pcamera_information_vendor_name_SET( NULL, dst  );\
       SRC##_vendor_name_GET( src, &item_vendor_name );\
       Vcamera_information_model_name item_model_name = pcamera_information_model_name_SET( NULL, dst  );\
       SRC##_model_name_GET( src, &item_model_name );\
        pcamera_information_focal_length_SET( SRC##_focal_length_GET(src ), dst  );\
        pcamera_information_sensor_size_h_SET( SRC##_sensor_size_h_GET(src ), dst  );\
        pcamera_information_sensor_size_v_SET( SRC##_sensor_size_v_GET(src ), dst  );\
        pcamera_information_lens_id_SET( SRC##_lens_id_GET(src ), dst  );\
        if( SRC##_flags_item_exist(src ) )\
        pcamera_information_flags_SET( pcamera_information_CAMERA_INFORMATION_flags_GET( src ),  dst  );\
        const size_t len_cam_definition_uri = SRC##_cam_definition_uri_item_exist(src );\
        if( len_cam_definition_uri ){\
            Vcamera_information_cam_definition_uri    item_cam_definition_uri = pcamera_information_cam_definition_uri_SET( NULL, len_cam_definition_uri, dst  );\
            pcamera_information_CAMERA_INFORMATION_cam_definition_uri_GET(src, &item_cam_definition_uri );\
        }\
    }

/**
*The positioning status, as reported by GPS. This message is intended to display status information about
*				 each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate.
*				 This message can contain information for up to 20 satellites */

typedef Pack GPS_STATUS_gps_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pgps_status_GPS_STATUS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pgps_status_GPS_STATUS *pgps_status_GPS_STATUS_from(GPS_STATUS_gps_status *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgps_status_satellite_prn;
//Maximum field array length constant
#define Pgps_status_satellite_prn_len  ( 20 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgps_status_satellite_used;
//Maximum field array length constant
#define Pgps_status_satellite_used_len  ( 20 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgps_status_satellite_elevation;
//Maximum field array length constant
#define Pgps_status_satellite_elevation_len  ( 20 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgps_status_satellite_azimuth;
//Maximum field array length constant
#define Pgps_status_satellite_azimuth_len  ( 20 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgps_status_satellite_snr;
//Maximum field array length constant
#define Pgps_status_satellite_snr_len  ( 20 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_satellites_visible_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_satellite_prn_SET( Vgps_status_satellite_prn * src, <DST> * dst  ){}
static inline void <DST>_satellite_used_SET( Vgps_status_satellite_used * src, <DST> * dst  ){}
static inline void <DST>_satellite_elevation_SET( Vgps_status_satellite_elevation * src, <DST> * dst  ){}
static inline void <DST>_satellite_azimuth_SET( Vgps_status_satellite_azimuth * src, <DST> * dst  ){}
static inline void <DST>_satellite_snr_SET( Vgps_status_satellite_snr * src, <DST> * dst  ){}
*/

#define pgps_status_GPS_STATUS_PUSH_INTO(DST)\
    static inline void pgps_status_GPS_STATUS_push_into_##DST ( pgps_status_GPS_STATUS * src, DST * dst) {\
        DST##_satellites_visible_SET( pgps_status_satellites_visible_GET( src  ), dst  );\
        Vgps_status_satellite_prn item_satellite_prn = pgps_status_satellite_prn_GET( src  );\
       DST##_satellite_prn_SET( &item_satellite_prn, dst );\
        Vgps_status_satellite_used item_satellite_used = pgps_status_satellite_used_GET( src  );\
       DST##_satellite_used_SET( &item_satellite_used, dst );\
        Vgps_status_satellite_elevation item_satellite_elevation = pgps_status_satellite_elevation_GET( src  );\
       DST##_satellite_elevation_SET( &item_satellite_elevation, dst );\
        Vgps_status_satellite_azimuth item_satellite_azimuth = pgps_status_satellite_azimuth_GET( src  );\
       DST##_satellite_azimuth_SET( &item_satellite_azimuth, dst );\
        Vgps_status_satellite_snr item_satellite_snr = pgps_status_satellite_snr_GET( src  );\
       DST##_satellite_snr_SET( &item_satellite_snr, dst );\
    }

/**
*Write registers reply */

typedef Pack DEVICE_OP_WRITE_REPLY_device_op_write_reply; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY_from(DEVICE_OP_WRITE_REPLY_device_op_write_reply *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_request_id_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_result_SET( int8_t * src, <DST> * dst  ){}
*/

#define pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY_PUSH_INTO(DST)\
    static inline void pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY_push_into_##DST ( pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY * src, DST * dst) {\
        DST##_request_id_SET( pdevice_op_write_reply_request_id_GET( src  ), dst  );\
        DST##_result_SET( pdevice_op_write_reply_result_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_request_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_result_GET( <SRC> * src ){}
*/

#define pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY_PULL_FROM(SRC)\
    static inline void pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY_pull_from_##SRC ( SRC * src, pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY * dst) {\
        pdevice_op_write_reply_request_id_SET( SRC##_request_id_GET(src ), dst  );\
        pdevice_op_write_reply_result_SET( SRC##_result_GET(src ), dst  );\
    }

/**
*Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION
*				 MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component
*				 should acknowledge the new parameter value by sending a param_value message to all communication partners.
*				 This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending
*				 GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message */

typedef Pack PARAM_SET_param_set; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pparam_set_PARAM_SET;// data navigator over pack fields data
/**
															* Wrap PARAM_SET in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pparam_set_PARAM_SET *PARAM_SET_param_set_wrap(PARAM_SET_param_set *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PARAM_SET_param_set *pparam_set_PARAM_SET_unwrap(pparam_set_PARAM_SET *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_set_param_id;
//Maximum field array length constant
#define Pparam_set_param_id_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param_value_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param_id_SET( Vparam_set_param_id * src, <DST> * dst  ){}
static inline void <DST>_param_type_SET( e_MAV_PARAM_TYPE * src, <DST> * dst  ){}
*/

#define pparam_set_PARAM_SET_PUSH_INTO(DST)\
    static inline void pparam_set_PARAM_SET_push_into_##DST ( pparam_set_PARAM_SET * src, DST * dst) {\
        DST##_target_system_SET( pparam_set_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pparam_set_target_component_GET( src  ), dst  );\
        DST##_param_value_SET( pparam_set_param_value_GET( src  ), dst  );\
        Vparam_set_param_id  item_param_id;\
        if( pparam_set_param_id_GET( src, &item_param_id ) ){\
            DST##_param_id_SET( &item_param_id, dst );\
        }\
        e_MAV_PARAM_TYPE  item_param_type;\
        if( pparam_set_param_type_GET( src, &item_param_type ) ){\
            DST##_param_type_SET( item_param_type , dst  );\
        }\
    }

/**
*Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES */

typedef Pack TERRAIN_DATA_terrain_data; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pterrain_data_TERRAIN_DATA;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pterrain_data_TERRAIN_DATA *pterrain_data_TERRAIN_DATA_from(TERRAIN_DATA_terrain_data *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vterrain_data_daTa;
//Maximum field array length constant
#define Pterrain_data_daTa_len  ( 16 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_grid_spacing_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_gridbit_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vterrain_data_daTa * src, <DST> * dst  ){}
*/

#define pterrain_data_TERRAIN_DATA_PUSH_INTO(DST)\
    static inline void pterrain_data_TERRAIN_DATA_push_into_##DST ( pterrain_data_TERRAIN_DATA * src, DST * dst) {\
        DST##_grid_spacing_SET( pterrain_data_grid_spacing_GET( src  ), dst  );\
        DST##_lat_SET( pterrain_data_lat_GET( src  ), dst  );\
        DST##_lon_SET( pterrain_data_lon_GET( src  ), dst  );\
        DST##_gridbit_SET( pterrain_data_gridbit_GET( src  ), dst  );\
        Vterrain_data_daTa item_daTa = pterrain_data_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_grid_spacing_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline int8_t <SRC>_gridbit_GET( <SRC> * src ){}
static inline int16_t <SRC>_daTa_GET( <SRC> * src, Vterrain_data_daTa * dst ){}
*/

#define pterrain_data_TERRAIN_DATA_PULL_FROM(SRC)\
    static inline void pterrain_data_TERRAIN_DATA_pull_from_##SRC ( SRC * src, pterrain_data_TERRAIN_DATA * dst) {\
        pterrain_data_grid_spacing_SET( SRC##_grid_spacing_GET(src ), dst  );\
        pterrain_data_lat_SET( SRC##_lat_GET(src ), dst  );\
        pterrain_data_lon_SET( SRC##_lon_GET(src ), dst  );\
        pterrain_data_gridbit_SET( SRC##_gridbit_GET(src ), dst  );\
       Vterrain_data_daTa item_daTa = pterrain_data_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*Control message for rate gimbal */

typedef Pack GIMBAL_CONTROL_gimbal_control; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pgimbal_control_GIMBAL_CONTROL;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pgimbal_control_GIMBAL_CONTROL *pgimbal_control_GIMBAL_CONTROL_from(GIMBAL_CONTROL_gimbal_control *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_demanded_rate_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_demanded_rate_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_demanded_rate_z_SET( float * src, <DST> * dst  ){}
*/

#define pgimbal_control_GIMBAL_CONTROL_PUSH_INTO(DST)\
    static inline void pgimbal_control_GIMBAL_CONTROL_push_into_##DST ( pgimbal_control_GIMBAL_CONTROL * src, DST * dst) {\
        DST##_target_system_SET( pgimbal_control_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pgimbal_control_target_component_GET( src  ), dst  );\
        DST##_demanded_rate_x_SET( pgimbal_control_demanded_rate_x_GET( src  ), dst  );\
        DST##_demanded_rate_y_SET( pgimbal_control_demanded_rate_y_GET( src  ), dst  );\
        DST##_demanded_rate_z_SET( pgimbal_control_demanded_rate_z_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline float <SRC>_demanded_rate_x_GET( <SRC> * src ){}
static inline float <SRC>_demanded_rate_y_GET( <SRC> * src ){}
static inline float <SRC>_demanded_rate_z_GET( <SRC> * src ){}
*/

#define pgimbal_control_GIMBAL_CONTROL_PULL_FROM(SRC)\
    static inline void pgimbal_control_GIMBAL_CONTROL_pull_from_##SRC ( SRC * src, pgimbal_control_GIMBAL_CONTROL * dst) {\
        pgimbal_control_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pgimbal_control_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pgimbal_control_demanded_rate_x_SET( SRC##_demanded_rate_x_GET(src ), dst  );\
        pgimbal_control_demanded_rate_y_SET( SRC##_demanded_rate_y_GET(src ), dst  );\
        pgimbal_control_demanded_rate_z_SET( SRC##_demanded_rate_z_GET(src ), dst  );\
    }

/**
*The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value
*				 of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released
*				 back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
*				 100%. Individual receivers/transmitters might violate this specification */

typedef Pack RC_CHANNELS_OVERRIDE_rc_channels_override; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t prc_channels_override_RC_CHANNELS_OVERRIDE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline prc_channels_override_RC_CHANNELS_OVERRIDE *prc_channels_override_RC_CHANNELS_OVERRIDE_from(RC_CHANNELS_OVERRIDE_rc_channels_override *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_chan1_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan2_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan3_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan4_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan5_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan6_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan7_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan8_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define prc_channels_override_RC_CHANNELS_OVERRIDE_PUSH_INTO(DST)\
    static inline void prc_channels_override_RC_CHANNELS_OVERRIDE_push_into_##DST ( prc_channels_override_RC_CHANNELS_OVERRIDE * src, DST * dst) {\
        DST##_chan1_raw_SET( prc_channels_override_chan1_raw_GET( src  ), dst  );\
        DST##_chan2_raw_SET( prc_channels_override_chan2_raw_GET( src  ), dst  );\
        DST##_chan3_raw_SET( prc_channels_override_chan3_raw_GET( src  ), dst  );\
        DST##_chan4_raw_SET( prc_channels_override_chan4_raw_GET( src  ), dst  );\
        DST##_chan5_raw_SET( prc_channels_override_chan5_raw_GET( src  ), dst  );\
        DST##_chan6_raw_SET( prc_channels_override_chan6_raw_GET( src  ), dst  );\
        DST##_chan7_raw_SET( prc_channels_override_chan7_raw_GET( src  ), dst  );\
        DST##_chan8_raw_SET( prc_channels_override_chan8_raw_GET( src  ), dst  );\
        DST##_target_system_SET( prc_channels_override_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( prc_channels_override_target_component_GET( src  ), dst  );\
    }

/**
*The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
*				 the described unit */

typedef Pack SCALED_IMU_scaled_imu; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pscaled_imu_SCALED_IMU;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pscaled_imu_SCALED_IMU *pscaled_imu_SCALED_IMU_from(SCALED_IMU_scaled_imu *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xgyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ygyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zgyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xmag_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ymag_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zmag_SET( int16_t * src, <DST> * dst  ){}
*/

#define pscaled_imu_SCALED_IMU_PUSH_INTO(DST)\
    static inline void pscaled_imu_SCALED_IMU_push_into_##DST ( pscaled_imu_SCALED_IMU * src, DST * dst) {\
        DST##_time_boot_ms_SET( pscaled_imu_time_boot_ms_GET( src  ), dst  );\
        DST##_xacc_SET( pscaled_imu_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( pscaled_imu_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( pscaled_imu_zacc_GET( src  ), dst  );\
        DST##_xgyro_SET( pscaled_imu_xgyro_GET( src  ), dst  );\
        DST##_ygyro_SET( pscaled_imu_ygyro_GET( src  ), dst  );\
        DST##_zgyro_SET( pscaled_imu_zgyro_GET( src  ), dst  );\
        DST##_xmag_SET( pscaled_imu_xmag_GET( src  ), dst  );\
        DST##_ymag_SET( pscaled_imu_ymag_GET( src  ), dst  );\
        DST##_zmag_SET( pscaled_imu_zmag_GET( src  ), dst  );\
    }

/**
*WIP: Information about video stream */

typedef Pack VIDEO_STREAM_INFORMATION_video_stream_information; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pvideo_stream_information_VIDEO_STREAM_INFORMATION;// data navigator over pack fields data
/**
															* Wrap VIDEO_STREAM_INFORMATION in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pvideo_stream_information_VIDEO_STREAM_INFORMATION *VIDEO_STREAM_INFORMATION_video_stream_information_wrap(VIDEO_STREAM_INFORMATION_video_stream_information *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline VIDEO_STREAM_INFORMATION_video_stream_information *pvideo_stream_information_VIDEO_STREAM_INFORMATION_unwrap(pvideo_stream_information_VIDEO_STREAM_INFORMATION *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vvideo_stream_information_uri;
//Maximum field array length constant
#define Pvideo_stream_information_uri_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_resolution_h_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_resolution_v_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_rotation_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_bitrate_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_camera_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_status_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_framerate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_uri_SET( Vvideo_stream_information_uri * src, <DST> * dst  ){}
*/

#define pvideo_stream_information_VIDEO_STREAM_INFORMATION_PUSH_INTO(DST)\
    static inline void pvideo_stream_information_VIDEO_STREAM_INFORMATION_push_into_##DST ( pvideo_stream_information_VIDEO_STREAM_INFORMATION * src, DST * dst) {\
        DST##_resolution_h_SET( pvideo_stream_information_resolution_h_GET( src  ), dst  );\
        DST##_resolution_v_SET( pvideo_stream_information_resolution_v_GET( src  ), dst  );\
        DST##_rotation_SET( pvideo_stream_information_rotation_GET( src  ), dst  );\
        DST##_bitrate_SET( pvideo_stream_information_bitrate_GET( src  ), dst  );\
        DST##_camera_id_SET( pvideo_stream_information_camera_id_GET( src  ), dst  );\
        DST##_status_SET( pvideo_stream_information_status_GET( src  ), dst  );\
        DST##_framerate_SET( pvideo_stream_information_framerate_GET( src  ), dst  );\
        Vvideo_stream_information_uri  item_uri;\
        if( pvideo_stream_information_uri_GET( src, &item_uri ) ){\
            DST##_uri_SET( &item_uri, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_resolution_h_GET( <SRC> * src ){}
static inline int16_t <SRC>_resolution_v_GET( <SRC> * src ){}
static inline int16_t <SRC>_rotation_GET( <SRC> * src ){}
static inline int32_t <SRC>_bitrate_GET( <SRC> * src ){}
static inline int8_t <SRC>_camera_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_status_GET( <SRC> * src ){}
static inline float <SRC>_framerate_GET( <SRC> * src ){}
static inline size_t  <SRC>_uri_item_exist( <SRC> * src  ){}
static inline void <SRC>_uri_GET( <SRC> * src, Vvideo_stream_information_uri * dst ){}
*/

#define pvideo_stream_information_VIDEO_STREAM_INFORMATION_PULL_FROM(SRC)\
    static inline void pvideo_stream_information_VIDEO_STREAM_INFORMATION_pull_from_##SRC ( SRC * src, pvideo_stream_information_VIDEO_STREAM_INFORMATION * dst) {\
        pvideo_stream_information_resolution_h_SET( SRC##_resolution_h_GET(src ), dst  );\
        pvideo_stream_information_resolution_v_SET( SRC##_resolution_v_GET(src ), dst  );\
        pvideo_stream_information_rotation_SET( SRC##_rotation_GET(src ), dst  );\
        pvideo_stream_information_bitrate_SET( SRC##_bitrate_GET(src ), dst  );\
        pvideo_stream_information_camera_id_SET( SRC##_camera_id_GET(src ), dst  );\
        pvideo_stream_information_status_SET( SRC##_status_GET(src ), dst  );\
        pvideo_stream_information_framerate_SET( SRC##_framerate_GET(src ), dst  );\
        const size_t len_uri = SRC##_uri_item_exist(src );\
        if( len_uri ){\
            Vvideo_stream_information_uri    item_uri = pvideo_stream_information_uri_SET( NULL, len_uri, dst  );\
            pvideo_stream_information_VIDEO_STREAM_INFORMATION_uri_GET(src, &item_uri );\
        }\
    }

/**
*Status of DCM attitude estimator */

typedef Pack AHRS_ahrs; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pahrs_AHRS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pahrs_AHRS *pahrs_AHRS_from(AHRS_ahrs *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_omegaIx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_omegaIy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_omegaIz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_accel_weight_SET( float * src, <DST> * dst  ){}
static inline void <DST>_renorm_val_SET( float * src, <DST> * dst  ){}
static inline void <DST>_error_rp_SET( float * src, <DST> * dst  ){}
static inline void <DST>_error_yaw_SET( float * src, <DST> * dst  ){}
*/

#define pahrs_AHRS_PUSH_INTO(DST)\
    static inline void pahrs_AHRS_push_into_##DST ( pahrs_AHRS * src, DST * dst) {\
        DST##_omegaIx_SET( pahrs_omegaIx_GET( src  ), dst  );\
        DST##_omegaIy_SET( pahrs_omegaIy_GET( src  ), dst  );\
        DST##_omegaIz_SET( pahrs_omegaIz_GET( src  ), dst  );\
        DST##_accel_weight_SET( pahrs_accel_weight_GET( src  ), dst  );\
        DST##_renorm_val_SET( pahrs_renorm_val_GET( src  ), dst  );\
        DST##_error_rp_SET( pahrs_error_rp_GET( src  ), dst  );\
        DST##_error_yaw_SET( pahrs_error_yaw_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_omegaIx_GET( <SRC> * src ){}
static inline float <SRC>_omegaIy_GET( <SRC> * src ){}
static inline float <SRC>_omegaIz_GET( <SRC> * src ){}
static inline float <SRC>_accel_weight_GET( <SRC> * src ){}
static inline float <SRC>_renorm_val_GET( <SRC> * src ){}
static inline float <SRC>_error_rp_GET( <SRC> * src ){}
static inline float <SRC>_error_yaw_GET( <SRC> * src ){}
*/

#define pahrs_AHRS_PULL_FROM(SRC)\
    static inline void pahrs_AHRS_pull_from_##SRC ( SRC * src, pahrs_AHRS * dst) {\
        pahrs_omegaIx_SET( SRC##_omegaIx_GET(src ), dst  );\
        pahrs_omegaIy_SET( SRC##_omegaIy_GET(src ), dst  );\
        pahrs_omegaIz_SET( SRC##_omegaIz_GET(src ), dst  );\
        pahrs_accel_weight_SET( SRC##_accel_weight_GET(src ), dst  );\
        pahrs_renorm_val_SET( SRC##_renorm_val_GET(src ), dst  );\
        pahrs_error_rp_SET( SRC##_error_rp_GET(src ), dst  );\
        pahrs_error_yaw_SET( SRC##_error_yaw_GET(src ), dst  );\
    }

/**
*Send a debug value. The index is used to discriminate between values. These values show up in the plot
*				 of QGroundControl as DEBUG N */

typedef Pack DEBUG_debug; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdebug_DEBUG;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdebug_DEBUG *pdebug_DEBUG_from(DEBUG_debug *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_ind_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_value_SET( float * src, <DST> * dst  ){}
*/

#define pdebug_DEBUG_PUSH_INTO(DST)\
    static inline void pdebug_DEBUG_push_into_##DST ( pdebug_DEBUG * src, DST * dst) {\
        DST##_time_boot_ms_SET( pdebug_time_boot_ms_GET( src  ), dst  );\
        DST##_ind_SET( pdebug_ind_GET( src  ), dst  );\
        DST##_value_SET( pdebug_value_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int8_t <SRC>_ind_GET( <SRC> * src ){}
static inline float <SRC>_value_GET( <SRC> * src ){}
*/

#define pdebug_DEBUG_PULL_FROM(SRC)\
    static inline void pdebug_DEBUG_pull_from_##SRC ( SRC * src, pdebug_DEBUG * dst) {\
        pdebug_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pdebug_ind_SET( SRC##_ind_GET(src ), dst  );\
        pdebug_value_SET( SRC##_value_GET(src ), dst  );\
    }

/**
*Information about a captured image */

typedef Pack CAMERA_IMAGE_CAPTURED_camera_image_captured; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcamera_image_captured_CAMERA_IMAGE_CAPTURED;// data navigator over pack fields data
/**
															* Wrap CAMERA_IMAGE_CAPTURED in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcamera_image_captured_CAMERA_IMAGE_CAPTURED *CAMERA_IMAGE_CAPTURED_camera_image_captured_wrap(CAMERA_IMAGE_CAPTURED_camera_image_captured *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline CAMERA_IMAGE_CAPTURED_camera_image_captured *pcamera_image_captured_CAMERA_IMAGE_CAPTURED_unwrap(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vcamera_image_captured_q;
//Maximum field array length constant
#define Pcamera_image_captured_q_len  ( 4 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vcamera_image_captured_file_url;
//Maximum field array length constant
#define Pcamera_image_captured_file_url_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_utc_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_camera_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_relative_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_q_SET( Vcamera_image_captured_q * src, <DST> * dst  ){}
static inline void <DST>_image_index_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_capture_result_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_file_url_SET( Vcamera_image_captured_file_url * src, <DST> * dst  ){}
*/

#define pcamera_image_captured_CAMERA_IMAGE_CAPTURED_PUSH_INTO(DST)\
    static inline void pcamera_image_captured_CAMERA_IMAGE_CAPTURED_push_into_##DST ( pcamera_image_captured_CAMERA_IMAGE_CAPTURED * src, DST * dst) {\
        DST##_time_boot_ms_SET( pcamera_image_captured_time_boot_ms_GET( src  ), dst  );\
        DST##_time_utc_SET( pcamera_image_captured_time_utc_GET( src  ), dst  );\
        DST##_camera_id_SET( pcamera_image_captured_camera_id_GET( src  ), dst  );\
        DST##_lat_SET( pcamera_image_captured_lat_GET( src  ), dst  );\
        DST##_lon_SET( pcamera_image_captured_lon_GET( src  ), dst  );\
        DST##_alt_SET( pcamera_image_captured_alt_GET( src  ), dst  );\
        DST##_relative_alt_SET( pcamera_image_captured_relative_alt_GET( src  ), dst  );\
        Vcamera_image_captured_q item_q = pcamera_image_captured_q_GET( src  );\
       DST##_q_SET( &item_q, dst );\
        DST##_image_index_SET( pcamera_image_captured_image_index_GET( src  ), dst  );\
        DST##_capture_result_SET( pcamera_image_captured_capture_result_GET( src  ), dst  );\
        Vcamera_image_captured_file_url  item_file_url;\
        if( pcamera_image_captured_file_url_GET( src, &item_file_url ) ){\
            DST##_file_url_SET( &item_file_url, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_utc_GET( <SRC> * src ){}
static inline int8_t <SRC>_camera_id_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline int32_t <SRC>_alt_GET( <SRC> * src ){}
static inline int32_t <SRC>_relative_alt_GET( <SRC> * src ){}
static inline float <SRC>_q_GET( <SRC> * src, Vcamera_image_captured_q * dst ){}
static inline int32_t <SRC>_image_index_GET( <SRC> * src ){}
static inline int8_t <SRC>_capture_result_GET( <SRC> * src ){}
static inline size_t  <SRC>_file_url_item_exist( <SRC> * src  ){}
static inline void <SRC>_file_url_GET( <SRC> * src, Vcamera_image_captured_file_url * dst ){}
*/

#define pcamera_image_captured_CAMERA_IMAGE_CAPTURED_PULL_FROM(SRC)\
    static inline void pcamera_image_captured_CAMERA_IMAGE_CAPTURED_pull_from_##SRC ( SRC * src, pcamera_image_captured_CAMERA_IMAGE_CAPTURED * dst) {\
        pcamera_image_captured_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pcamera_image_captured_time_utc_SET( SRC##_time_utc_GET(src ), dst  );\
        pcamera_image_captured_camera_id_SET( SRC##_camera_id_GET(src ), dst  );\
        pcamera_image_captured_lat_SET( SRC##_lat_GET(src ), dst  );\
        pcamera_image_captured_lon_SET( SRC##_lon_GET(src ), dst  );\
        pcamera_image_captured_alt_SET( SRC##_alt_GET(src ), dst  );\
        pcamera_image_captured_relative_alt_SET( SRC##_relative_alt_GET(src ), dst  );\
       Vcamera_image_captured_q item_q = pcamera_image_captured_q_SET( NULL, dst  );\
       SRC##_q_GET( src, &item_q );\
        pcamera_image_captured_image_index_SET( SRC##_image_index_GET(src ), dst  );\
        pcamera_image_captured_capture_result_SET( SRC##_capture_result_GET(src ), dst  );\
        const size_t len_file_url = SRC##_file_url_item_exist(src );\
        if( len_file_url ){\
            Vcamera_image_captured_file_url    item_file_url = pcamera_image_captured_file_url_SET( NULL, len_file_url, dst  );\
            pcamera_image_captured_CAMERA_IMAGE_CAPTURED_file_url_GET(src, &item_file_url );\
        }\
    }

/**
*Reply to LOG_REQUEST_LIST */

typedef Pack LOG_ENTRY_log_entry; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plog_entry_LOG_ENTRY;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plog_entry_LOG_ENTRY *plog_entry_LOG_ENTRY_from(LOG_ENTRY_log_entry *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_id_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_num_logs_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_last_log_num_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_utc_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_size_SET( int32_t * src, <DST> * dst  ){}
*/

#define plog_entry_LOG_ENTRY_PUSH_INTO(DST)\
    static inline void plog_entry_LOG_ENTRY_push_into_##DST ( plog_entry_LOG_ENTRY * src, DST * dst) {\
        DST##_id_SET( plog_entry_id_GET( src  ), dst  );\
        DST##_num_logs_SET( plog_entry_num_logs_GET( src  ), dst  );\
        DST##_last_log_num_SET( plog_entry_last_log_num_GET( src  ), dst  );\
        DST##_time_utc_SET( plog_entry_time_utc_GET( src  ), dst  );\
        DST##_size_SET( plog_entry_size_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_id_GET( <SRC> * src ){}
static inline int16_t <SRC>_num_logs_GET( <SRC> * src ){}
static inline int16_t <SRC>_last_log_num_GET( <SRC> * src ){}
static inline int32_t <SRC>_time_utc_GET( <SRC> * src ){}
static inline int32_t <SRC>_size_GET( <SRC> * src ){}
*/

#define plog_entry_LOG_ENTRY_PULL_FROM(SRC)\
    static inline void plog_entry_LOG_ENTRY_pull_from_##SRC ( SRC * src, plog_entry_LOG_ENTRY * dst) {\
        plog_entry_id_SET( SRC##_id_GET(src ), dst  );\
        plog_entry_num_logs_SET( SRC##_num_logs_GET(src ), dst  );\
        plog_entry_last_log_num_SET( SRC##_last_log_num_GET(src ), dst  );\
        plog_entry_time_utc_SET( SRC##_time_utc_GET(src ), dst  );\
        plog_entry_size_SET( SRC##_size_GET(src ), dst  );\
    }

/**
*Set the vehicle attitude and body angular rates. */

typedef Pack ACTUATOR_CONTROL_TARGET_actuator_control_target; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pactuator_control_target_ACTUATOR_CONTROL_TARGET;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pactuator_control_target_ACTUATOR_CONTROL_TARGET *pactuator_control_target_ACTUATOR_CONTROL_TARGET_from(ACTUATOR_CONTROL_TARGET_actuator_control_target *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vactuator_control_target_controls;
//Maximum field array length constant
#define Pactuator_control_target_controls_len  ( 8 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_group_mlx_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_controls_SET( Vactuator_control_target_controls * src, <DST> * dst  ){}
*/

#define pactuator_control_target_ACTUATOR_CONTROL_TARGET_PUSH_INTO(DST)\
    static inline void pactuator_control_target_ACTUATOR_CONTROL_TARGET_push_into_##DST ( pactuator_control_target_ACTUATOR_CONTROL_TARGET * src, DST * dst) {\
        DST##_time_usec_SET( pactuator_control_target_time_usec_GET( src  ), dst  );\
        DST##_group_mlx_SET( pactuator_control_target_group_mlx_GET( src  ), dst  );\
        Vactuator_control_target_controls item_controls = pactuator_control_target_controls_GET( src  );\
       DST##_controls_SET( &item_controls, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_group_mlx_GET( <SRC> * src ){}
static inline float <SRC>_controls_GET( <SRC> * src, Vactuator_control_target_controls * dst ){}
*/

#define pactuator_control_target_ACTUATOR_CONTROL_TARGET_PULL_FROM(SRC)\
    static inline void pactuator_control_target_ACTUATOR_CONTROL_TARGET_pull_from_##SRC ( SRC * src, pactuator_control_target_ACTUATOR_CONTROL_TARGET * dst) {\
        pactuator_control_target_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pactuator_control_target_group_mlx_SET( SRC##_group_mlx_GET(src ), dst  );\
       Vactuator_control_target_controls item_controls = pactuator_control_target_controls_SET( NULL, dst  );\
       SRC##_controls_GET( src, &item_controls );\
    }

/**
*Message appropriate for high latency connections like Iridium */

typedef Pack HIGH_LATENCY_high_latency; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor phigh_latency_HIGH_LATENCY;// data navigator over pack fields data
/**
															* Wrap HIGH_LATENCY in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline phigh_latency_HIGH_LATENCY *HIGH_LATENCY_high_latency_wrap(HIGH_LATENCY_high_latency *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline HIGH_LATENCY_high_latency *phigh_latency_HIGH_LATENCY_unwrap(phigh_latency_HIGH_LATENCY *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_heading_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_wp_distance_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_custom_mode_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_throttle_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_heading_sp_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_latitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_longitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_amsl_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_sp_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_airspeed_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_airspeed_sp_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_groundspeed_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_climb_rate_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_gps_nsat_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_battery_remaining_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_temperature_air_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_failsafe_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_wp_num_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_base_mode_SET( e_MAV_MODE_FLAG * src, <DST> * dst  ){}
static inline void <DST>_landed_state_SET( e_MAV_LANDED_STATE * src, <DST> * dst  ){}
static inline void <DST>_gps_fix_type_SET( e_GPS_FIX_TYPE * src, <DST> * dst  ){}
*/

#define phigh_latency_HIGH_LATENCY_PUSH_INTO(DST)\
    static inline void phigh_latency_HIGH_LATENCY_push_into_##DST ( phigh_latency_HIGH_LATENCY * src, DST * dst) {\
        DST##_heading_SET( phigh_latency_heading_GET( src  ), dst  );\
        DST##_wp_distance_SET( phigh_latency_wp_distance_GET( src  ), dst  );\
        DST##_custom_mode_SET( phigh_latency_custom_mode_GET( src  ), dst  );\
        DST##_roll_SET( phigh_latency_roll_GET( src  ), dst  );\
        DST##_pitch_SET( phigh_latency_pitch_GET( src  ), dst  );\
        DST##_throttle_SET( phigh_latency_throttle_GET( src  ), dst  );\
        DST##_heading_sp_SET( phigh_latency_heading_sp_GET( src  ), dst  );\
        DST##_latitude_SET( phigh_latency_latitude_GET( src  ), dst  );\
        DST##_longitude_SET( phigh_latency_longitude_GET( src  ), dst  );\
        DST##_altitude_amsl_SET( phigh_latency_altitude_amsl_GET( src  ), dst  );\
        DST##_altitude_sp_SET( phigh_latency_altitude_sp_GET( src  ), dst  );\
        DST##_airspeed_SET( phigh_latency_airspeed_GET( src  ), dst  );\
        DST##_airspeed_sp_SET( phigh_latency_airspeed_sp_GET( src  ), dst  );\
        DST##_groundspeed_SET( phigh_latency_groundspeed_GET( src  ), dst  );\
        DST##_climb_rate_SET( phigh_latency_climb_rate_GET( src  ), dst  );\
        DST##_gps_nsat_SET( phigh_latency_gps_nsat_GET( src  ), dst  );\
        DST##_battery_remaining_SET( phigh_latency_battery_remaining_GET( src  ), dst  );\
        DST##_temperature_SET( phigh_latency_temperature_GET( src  ), dst  );\
        DST##_temperature_air_SET( phigh_latency_temperature_air_GET( src  ), dst  );\
        DST##_failsafe_SET( phigh_latency_failsafe_GET( src  ), dst  );\
        DST##_wp_num_SET( phigh_latency_wp_num_GET( src  ), dst  );\
        e_MAV_MODE_FLAG  item_base_mode;\
        if( phigh_latency_base_mode_GET( src, &item_base_mode ) ){\
            DST##_base_mode_SET( item_base_mode , dst  );\
        }\
        e_MAV_LANDED_STATE  item_landed_state;\
        if( phigh_latency_landed_state_GET( src, &item_landed_state ) ){\
            DST##_landed_state_SET( item_landed_state , dst  );\
        }\
        e_GPS_FIX_TYPE  item_gps_fix_type;\
        if( phigh_latency_gps_fix_type_GET( src, &item_gps_fix_type ) ){\
            DST##_gps_fix_type_SET( item_gps_fix_type , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_heading_GET( <SRC> * src ){}
static inline int16_t <SRC>_wp_distance_GET( <SRC> * src ){}
static inline int32_t <SRC>_custom_mode_GET( <SRC> * src ){}
static inline int16_t <SRC>_roll_GET( <SRC> * src ){}
static inline int16_t <SRC>_pitch_GET( <SRC> * src ){}
static inline int8_t <SRC>_throttle_GET( <SRC> * src ){}
static inline int16_t <SRC>_heading_sp_GET( <SRC> * src ){}
static inline int32_t <SRC>_latitude_GET( <SRC> * src ){}
static inline int32_t <SRC>_longitude_GET( <SRC> * src ){}
static inline int16_t <SRC>_altitude_amsl_GET( <SRC> * src ){}
static inline int16_t <SRC>_altitude_sp_GET( <SRC> * src ){}
static inline int8_t <SRC>_airspeed_GET( <SRC> * src ){}
static inline int8_t <SRC>_airspeed_sp_GET( <SRC> * src ){}
static inline int8_t <SRC>_groundspeed_GET( <SRC> * src ){}
static inline int8_t <SRC>_climb_rate_GET( <SRC> * src ){}
static inline int8_t <SRC>_gps_nsat_GET( <SRC> * src ){}
static inline int8_t <SRC>_battery_remaining_GET( <SRC> * src ){}
static inline int8_t <SRC>_temperature_GET( <SRC> * src ){}
static inline int8_t <SRC>_temperature_air_GET( <SRC> * src ){}
static inline int8_t <SRC>_failsafe_GET( <SRC> * src ){}
static inline int8_t <SRC>_wp_num_GET( <SRC> * src ){}
static inline bool  <SRC>_base_mode_item_exist( <SRC> * src ){}
static inline e_MAV_MODE_FLAG <SRC>_base_mode_GET( <SRC> * src ){}
static inline bool  <SRC>_landed_state_item_exist( <SRC> * src ){}
static inline e_MAV_LANDED_STATE <SRC>_landed_state_GET( <SRC> * src ){}
static inline bool  <SRC>_gps_fix_type_item_exist( <SRC> * src ){}
static inline e_GPS_FIX_TYPE <SRC>_gps_fix_type_GET( <SRC> * src ){}
*/

#define phigh_latency_HIGH_LATENCY_PULL_FROM(SRC)\
    static inline void phigh_latency_HIGH_LATENCY_pull_from_##SRC ( SRC * src, phigh_latency_HIGH_LATENCY * dst) {\
        phigh_latency_heading_SET( SRC##_heading_GET(src ), dst  );\
        phigh_latency_wp_distance_SET( SRC##_wp_distance_GET(src ), dst  );\
        phigh_latency_custom_mode_SET( SRC##_custom_mode_GET(src ), dst  );\
        phigh_latency_roll_SET( SRC##_roll_GET(src ), dst  );\
        phigh_latency_pitch_SET( SRC##_pitch_GET(src ), dst  );\
        phigh_latency_throttle_SET( SRC##_throttle_GET(src ), dst  );\
        phigh_latency_heading_sp_SET( SRC##_heading_sp_GET(src ), dst  );\
        phigh_latency_latitude_SET( SRC##_latitude_GET(src ), dst  );\
        phigh_latency_longitude_SET( SRC##_longitude_GET(src ), dst  );\
        phigh_latency_altitude_amsl_SET( SRC##_altitude_amsl_GET(src ), dst  );\
        phigh_latency_altitude_sp_SET( SRC##_altitude_sp_GET(src ), dst  );\
        phigh_latency_airspeed_SET( SRC##_airspeed_GET(src ), dst  );\
        phigh_latency_airspeed_sp_SET( SRC##_airspeed_sp_GET(src ), dst  );\
        phigh_latency_groundspeed_SET( SRC##_groundspeed_GET(src ), dst  );\
        phigh_latency_climb_rate_SET( SRC##_climb_rate_GET(src ), dst  );\
        phigh_latency_gps_nsat_SET( SRC##_gps_nsat_GET(src ), dst  );\
        phigh_latency_battery_remaining_SET( SRC##_battery_remaining_GET(src ), dst  );\
        phigh_latency_temperature_SET( SRC##_temperature_GET(src ), dst  );\
        phigh_latency_temperature_air_SET( SRC##_temperature_air_GET(src ), dst  );\
        phigh_latency_failsafe_SET( SRC##_failsafe_GET(src ), dst  );\
        phigh_latency_wp_num_SET( SRC##_wp_num_GET(src ), dst  );\
        if( SRC##_base_mode_item_exist(src ) )\
        phigh_latency_base_mode_SET( phigh_latency_HIGH_LATENCY_base_mode_GET( src ),  dst  );\
        if( SRC##_landed_state_item_exist(src ) )\
        phigh_latency_landed_state_SET( phigh_latency_HIGH_LATENCY_landed_state_GET( src ),  dst  );\
        if( SRC##_gps_fix_type_item_exist(src ) )\
        phigh_latency_gps_fix_type_SET( phigh_latency_HIGH_LATENCY_gps_fix_type_GET( src ),  dst  );\
    }

/**
*value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
*				 of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
*				 different autopilots. See also http:qgroundcontrol.org/parameter_interface for a full documentation
*				 of QGroundControl and IMU code */

typedef Pack PARAM_REQUEST_READ_param_request_read; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pparam_request_read_PARAM_REQUEST_READ;// data navigator over pack fields data
/**
															* Wrap PARAM_REQUEST_READ in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pparam_request_read_PARAM_REQUEST_READ *PARAM_REQUEST_READ_param_request_read_wrap(PARAM_REQUEST_READ_param_request_read *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PARAM_REQUEST_READ_param_request_read *pparam_request_read_PARAM_REQUEST_READ_unwrap(pparam_request_read_PARAM_REQUEST_READ *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_request_read_param_id;
//Maximum field array length constant
#define Pparam_request_read_param_id_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_param_id_SET( Vparam_request_read_param_id * src, <DST> * dst  ){}
*/

#define pparam_request_read_PARAM_REQUEST_READ_PUSH_INTO(DST)\
    static inline void pparam_request_read_PARAM_REQUEST_READ_push_into_##DST ( pparam_request_read_PARAM_REQUEST_READ * src, DST * dst) {\
        DST##_target_system_SET( pparam_request_read_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pparam_request_read_target_component_GET( src  ), dst  );\
        DST##_param_index_SET( pparam_request_read_param_index_GET( src  ), dst  );\
        Vparam_request_read_param_id  item_param_id;\
        if( pparam_request_read_param_id_GET( src, &item_param_id ) ){\
            DST##_param_id_SET( &item_param_id, dst );\
        }\
    }

/**
*Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller
*				 or other system) */

typedef Pack SET_ATTITUDE_TARGET_set_attitude_target; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pset_attitude_target_SET_ATTITUDE_TARGET;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pset_attitude_target_SET_ATTITUDE_TARGET *pset_attitude_target_SET_ATTITUDE_TARGET_from(SET_ATTITUDE_TARGET_set_attitude_target *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vset_attitude_target_q;
//Maximum field array length constant
#define Pset_attitude_target_q_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_type_mask_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_q_SET( Vset_attitude_target_q * src, <DST> * dst  ){}
static inline void <DST>_body_roll_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_body_pitch_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_body_yaw_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_thrust_SET( float * src, <DST> * dst  ){}
*/

#define pset_attitude_target_SET_ATTITUDE_TARGET_PUSH_INTO(DST)\
    static inline void pset_attitude_target_SET_ATTITUDE_TARGET_push_into_##DST ( pset_attitude_target_SET_ATTITUDE_TARGET * src, DST * dst) {\
        DST##_time_boot_ms_SET( pset_attitude_target_time_boot_ms_GET( src  ), dst  );\
        DST##_target_system_SET( pset_attitude_target_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pset_attitude_target_target_component_GET( src  ), dst  );\
        DST##_type_mask_SET( pset_attitude_target_type_mask_GET( src  ), dst  );\
        Vset_attitude_target_q item_q = pset_attitude_target_q_GET( src  );\
       DST##_q_SET( &item_q, dst );\
        DST##_body_roll_rate_SET( pset_attitude_target_body_roll_rate_GET( src  ), dst  );\
        DST##_body_pitch_rate_SET( pset_attitude_target_body_pitch_rate_GET( src  ), dst  );\
        DST##_body_yaw_rate_SET( pset_attitude_target_body_yaw_rate_GET( src  ), dst  );\
        DST##_thrust_SET( pset_attitude_target_thrust_GET( src  ), dst  );\
    }

/**
*current motion information from a designated system */

typedef Pack FOLLOW_TARGET_follow_target; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pfollow_target_FOLLOW_TARGET;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pfollow_target_FOLLOW_TARGET *pfollow_target_FOLLOW_TARGET_from(FOLLOW_TARGET_follow_target *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vfollow_target_vel;
//Maximum field array length constant
#define Pfollow_target_vel_len  ( 3 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vfollow_target_acc;
//Maximum field array length constant
#define Pfollow_target_acc_len  ( 3 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vfollow_target_attitude_q;
//Maximum field array length constant
#define Pfollow_target_attitude_q_len  ( 4 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vfollow_target_rates;
//Maximum field array length constant
#define Pfollow_target_rates_len  ( 3 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vfollow_target_position_cov;
//Maximum field array length constant
#define Pfollow_target_position_cov_len  ( 3 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_timestamp_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_custom_state_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_est_capabilities_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vel_SET( Vfollow_target_vel * src, <DST> * dst  ){}
static inline void <DST>_acc_SET( Vfollow_target_acc * src, <DST> * dst  ){}
static inline void <DST>_attitude_q_SET( Vfollow_target_attitude_q * src, <DST> * dst  ){}
static inline void <DST>_rates_SET( Vfollow_target_rates * src, <DST> * dst  ){}
static inline void <DST>_position_cov_SET( Vfollow_target_position_cov * src, <DST> * dst  ){}
*/

#define pfollow_target_FOLLOW_TARGET_PUSH_INTO(DST)\
    static inline void pfollow_target_FOLLOW_TARGET_push_into_##DST ( pfollow_target_FOLLOW_TARGET * src, DST * dst) {\
        DST##_timestamp_SET( pfollow_target_timestamp_GET( src  ), dst  );\
        DST##_custom_state_SET( pfollow_target_custom_state_GET( src  ), dst  );\
        DST##_est_capabilities_SET( pfollow_target_est_capabilities_GET( src  ), dst  );\
        DST##_lat_SET( pfollow_target_lat_GET( src  ), dst  );\
        DST##_lon_SET( pfollow_target_lon_GET( src  ), dst  );\
        DST##_alt_SET( pfollow_target_alt_GET( src  ), dst  );\
        Vfollow_target_vel item_vel = pfollow_target_vel_GET( src  );\
       DST##_vel_SET( &item_vel, dst );\
        Vfollow_target_acc item_acc = pfollow_target_acc_GET( src  );\
       DST##_acc_SET( &item_acc, dst );\
        Vfollow_target_attitude_q item_attitude_q = pfollow_target_attitude_q_GET( src  );\
       DST##_attitude_q_SET( &item_attitude_q, dst );\
        Vfollow_target_rates item_rates = pfollow_target_rates_GET( src  );\
       DST##_rates_SET( &item_rates, dst );\
        Vfollow_target_position_cov item_position_cov = pfollow_target_position_cov_GET( src  );\
       DST##_position_cov_SET( &item_position_cov, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_timestamp_GET( <SRC> * src ){}
static inline int64_t <SRC>_custom_state_GET( <SRC> * src ){}
static inline int8_t <SRC>_est_capabilities_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline float <SRC>_alt_GET( <SRC> * src ){}
static inline float <SRC>_vel_GET( <SRC> * src, Vfollow_target_vel * dst ){}
static inline float <SRC>_acc_GET( <SRC> * src, Vfollow_target_acc * dst ){}
static inline float <SRC>_attitude_q_GET( <SRC> * src, Vfollow_target_attitude_q * dst ){}
static inline float <SRC>_rates_GET( <SRC> * src, Vfollow_target_rates * dst ){}
static inline float <SRC>_position_cov_GET( <SRC> * src, Vfollow_target_position_cov * dst ){}
*/

#define pfollow_target_FOLLOW_TARGET_PULL_FROM(SRC)\
    static inline void pfollow_target_FOLLOW_TARGET_pull_from_##SRC ( SRC * src, pfollow_target_FOLLOW_TARGET * dst) {\
        pfollow_target_timestamp_SET( SRC##_timestamp_GET(src ), dst  );\
        pfollow_target_custom_state_SET( SRC##_custom_state_GET(src ), dst  );\
        pfollow_target_est_capabilities_SET( SRC##_est_capabilities_GET(src ), dst  );\
        pfollow_target_lat_SET( SRC##_lat_GET(src ), dst  );\
        pfollow_target_lon_SET( SRC##_lon_GET(src ), dst  );\
        pfollow_target_alt_SET( SRC##_alt_GET(src ), dst  );\
       Vfollow_target_vel item_vel = pfollow_target_vel_SET( NULL, dst  );\
       SRC##_vel_GET( src, &item_vel );\
       Vfollow_target_acc item_acc = pfollow_target_acc_SET( NULL, dst  );\
       SRC##_acc_GET( src, &item_acc );\
       Vfollow_target_attitude_q item_attitude_q = pfollow_target_attitude_q_SET( NULL, dst  );\
       SRC##_attitude_q_GET( src, &item_attitude_q );\
       Vfollow_target_rates item_rates = pfollow_target_rates_SET( NULL, dst  );\
       SRC##_rates_GET( src, &item_rates );\
       Vfollow_target_position_cov item_position_cov = pfollow_target_position_cov_SET( NULL, dst  );\
       SRC##_position_cov_GET( src, &item_position_cov );\
    }

/**
*DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
*				 use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
*				 applications such as hardware in the loop simulations */

typedef Pack HIL_STATE_hil_state; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t phil_state_HIL_STATE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline phil_state_HIL_STATE *phil_state_HIL_STATE_from(HIL_STATE_hil_state *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_rollspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitchspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yawspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( int16_t * src, <DST> * dst  ){}
*/

#define phil_state_HIL_STATE_PUSH_INTO(DST)\
    static inline void phil_state_HIL_STATE_push_into_##DST ( phil_state_HIL_STATE * src, DST * dst) {\
        DST##_time_usec_SET( phil_state_time_usec_GET( src  ), dst  );\
        DST##_roll_SET( phil_state_roll_GET( src  ), dst  );\
        DST##_pitch_SET( phil_state_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( phil_state_yaw_GET( src  ), dst  );\
        DST##_rollspeed_SET( phil_state_rollspeed_GET( src  ), dst  );\
        DST##_pitchspeed_SET( phil_state_pitchspeed_GET( src  ), dst  );\
        DST##_yawspeed_SET( phil_state_yawspeed_GET( src  ), dst  );\
        DST##_lat_SET( phil_state_lat_GET( src  ), dst  );\
        DST##_lon_SET( phil_state_lon_GET( src  ), dst  );\
        DST##_alt_SET( phil_state_alt_GET( src  ), dst  );\
        DST##_vx_SET( phil_state_vx_GET( src  ), dst  );\
        DST##_vy_SET( phil_state_vy_GET( src  ), dst  );\
        DST##_vz_SET( phil_state_vz_GET( src  ), dst  );\
        DST##_xacc_SET( phil_state_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( phil_state_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( phil_state_zacc_GET( src  ), dst  );\
    }

/**
*This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
*				 will return to and land on. The position is set automatically by the system during the takeoff in case
*				 it was not explicitely set by the operator before or after. The position the system will return to and
*				 land on. The global and local positions encode the position in the respective coordinate frames, while
*				 the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
*				 and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
*				 the point to which the system should fly in normal flight mode and then perform a landing sequence along
*				 the vector */

typedef Pack HOME_POSITION_home_position; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor phome_position_HOME_POSITION;// data navigator over pack fields data
/**
															* Wrap HOME_POSITION in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline phome_position_HOME_POSITION *HOME_POSITION_home_position_wrap(HOME_POSITION_home_position *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline HOME_POSITION_home_position *phome_position_HOME_POSITION_unwrap(phome_position_HOME_POSITION *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vhome_position_q;
//Maximum field array length constant
#define Phome_position_q_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_latitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_longitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q_SET( Vhome_position_q * src, <DST> * dst  ){}
static inline void <DST>_approach_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_approach_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_approach_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
*/

#define phome_position_HOME_POSITION_PUSH_INTO(DST)\
    static inline void phome_position_HOME_POSITION_push_into_##DST ( phome_position_HOME_POSITION * src, DST * dst) {\
        DST##_latitude_SET( phome_position_latitude_GET( src  ), dst  );\
        DST##_longitude_SET( phome_position_longitude_GET( src  ), dst  );\
        DST##_altitude_SET( phome_position_altitude_GET( src  ), dst  );\
        DST##_x_SET( phome_position_x_GET( src  ), dst  );\
        DST##_y_SET( phome_position_y_GET( src  ), dst  );\
        DST##_z_SET( phome_position_z_GET( src  ), dst  );\
        Vhome_position_q item_q = phome_position_q_GET( src  );\
       DST##_q_SET( &item_q, dst );\
        DST##_approach_x_SET( phome_position_approach_x_GET( src  ), dst  );\
        DST##_approach_y_SET( phome_position_approach_y_GET( src  ), dst  );\
        DST##_approach_z_SET( phome_position_approach_z_GET( src  ), dst  );\
        int64_t  item_time_usec;\
        if( phome_position_time_usec_GET( src, &item_time_usec ) ){\
            DST##_time_usec_SET( item_time_usec , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_latitude_GET( <SRC> * src ){}
static inline int32_t <SRC>_longitude_GET( <SRC> * src ){}
static inline int32_t <SRC>_altitude_GET( <SRC> * src ){}
static inline float <SRC>_x_GET( <SRC> * src ){}
static inline float <SRC>_y_GET( <SRC> * src ){}
static inline float <SRC>_z_GET( <SRC> * src ){}
static inline float <SRC>_q_GET( <SRC> * src, Vhome_position_q * dst ){}
static inline float <SRC>_approach_x_GET( <SRC> * src ){}
static inline float <SRC>_approach_y_GET( <SRC> * src ){}
static inline float <SRC>_approach_z_GET( <SRC> * src ){}
static inline bool  <SRC>_time_usec_item_exist( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
*/

#define phome_position_HOME_POSITION_PULL_FROM(SRC)\
    static inline void phome_position_HOME_POSITION_pull_from_##SRC ( SRC * src, phome_position_HOME_POSITION * dst) {\
        phome_position_latitude_SET( SRC##_latitude_GET(src ), dst  );\
        phome_position_longitude_SET( SRC##_longitude_GET(src ), dst  );\
        phome_position_altitude_SET( SRC##_altitude_GET(src ), dst  );\
        phome_position_x_SET( SRC##_x_GET(src ), dst  );\
        phome_position_y_SET( SRC##_y_GET(src ), dst  );\
        phome_position_z_SET( SRC##_z_GET(src ), dst  );\
       Vhome_position_q item_q = phome_position_q_SET( NULL, dst  );\
       SRC##_q_GET( src, &item_q );\
        phome_position_approach_x_SET( SRC##_approach_x_GET(src ), dst  );\
        phome_position_approach_y_SET( SRC##_approach_y_GET(src ), dst  );\
        phome_position_approach_z_SET( SRC##_approach_z_GET(src ), dst  );\
        if( SRC##_time_usec_item_exist(src ) )\
        phome_position_time_usec_SET( phome_position_HOME_POSITION_time_usec_GET( src ),  dst  );\
    }

/**
*Status of geo-fencing. Sent in extended status stream when fencing enabled */

typedef Pack FENCE_STATUS_fence_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pfence_status_FENCE_STATUS;// data navigator over pack fields data
/**
															* Wrap FENCE_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pfence_status_FENCE_STATUS *FENCE_STATUS_fence_status_wrap(FENCE_STATUS_fence_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline FENCE_STATUS_fence_status *pfence_status_FENCE_STATUS_unwrap(pfence_status_FENCE_STATUS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_breach_count_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_breach_time_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_breach_status_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_breach_type_SET( e_FENCE_BREACH * src, <DST> * dst  ){}
*/

#define pfence_status_FENCE_STATUS_PUSH_INTO(DST)\
    static inline void pfence_status_FENCE_STATUS_push_into_##DST ( pfence_status_FENCE_STATUS * src, DST * dst) {\
        DST##_breach_count_SET( pfence_status_breach_count_GET( src  ), dst  );\
        DST##_breach_time_SET( pfence_status_breach_time_GET( src  ), dst  );\
        DST##_breach_status_SET( pfence_status_breach_status_GET( src  ), dst  );\
        e_FENCE_BREACH  item_breach_type;\
        if( pfence_status_breach_type_GET( src, &item_breach_type ) ){\
            DST##_breach_type_SET( item_breach_type , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_breach_count_GET( <SRC> * src ){}
static inline int32_t <SRC>_breach_time_GET( <SRC> * src ){}
static inline int8_t <SRC>_breach_status_GET( <SRC> * src ){}
static inline bool  <SRC>_breach_type_item_exist( <SRC> * src ){}
static inline e_FENCE_BREACH <SRC>_breach_type_GET( <SRC> * src ){}
*/

#define pfence_status_FENCE_STATUS_PULL_FROM(SRC)\
    static inline void pfence_status_FENCE_STATUS_pull_from_##SRC ( SRC * src, pfence_status_FENCE_STATUS * dst) {\
        pfence_status_breach_count_SET( SRC##_breach_count_GET(src ), dst  );\
        pfence_status_breach_time_SET( SRC##_breach_time_GET(src ), dst  );\
        pfence_status_breach_status_SET( SRC##_breach_status_GET(src ), dst  );\
        if( SRC##_breach_type_item_exist(src ) )\
        pfence_status_breach_type_SET( pfence_status_FENCE_STATUS_breach_type_GET( src ),  dst  );\
    }

/**
*Send Status of each log block that autopilot board might have sent */

typedef Pack REMOTE_LOG_BLOCK_STATUS_remote_log_block_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor premote_log_block_status_REMOTE_LOG_BLOCK_STATUS;// data navigator over pack fields data
/**
															* Wrap REMOTE_LOG_BLOCK_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *REMOTE_LOG_BLOCK_STATUS_remote_log_block_status_wrap(REMOTE_LOG_BLOCK_STATUS_remote_log_block_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline REMOTE_LOG_BLOCK_STATUS_remote_log_block_status *premote_log_block_status_REMOTE_LOG_BLOCK_STATUS_unwrap(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seqno_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_status_SET( e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES * src, <DST> * dst  ){}
*/

#define premote_log_block_status_REMOTE_LOG_BLOCK_STATUS_PUSH_INTO(DST)\
    static inline void premote_log_block_status_REMOTE_LOG_BLOCK_STATUS_push_into_##DST ( premote_log_block_status_REMOTE_LOG_BLOCK_STATUS * src, DST * dst) {\
        DST##_seqno_SET( premote_log_block_status_seqno_GET( src  ), dst  );\
        DST##_target_system_SET( premote_log_block_status_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( premote_log_block_status_target_component_GET( src  ), dst  );\
        e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES  item_status;\
        if( premote_log_block_status_status_GET( src, &item_status ) ){\
            DST##_status_SET( item_status , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_seqno_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline bool  <SRC>_status_item_exist( <SRC> * src ){}
static inline e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES <SRC>_status_GET( <SRC> * src ){}
*/

#define premote_log_block_status_REMOTE_LOG_BLOCK_STATUS_PULL_FROM(SRC)\
    static inline void premote_log_block_status_REMOTE_LOG_BLOCK_STATUS_pull_from_##SRC ( SRC * src, premote_log_block_status_REMOTE_LOG_BLOCK_STATUS * dst) {\
        premote_log_block_status_seqno_SET( SRC##_seqno_GET(src ), dst  );\
        premote_log_block_status_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        premote_log_block_status_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        if( SRC##_status_item_exist(src ) )\
        premote_log_block_status_status_SET( premote_log_block_status_REMOTE_LOG_BLOCK_STATUS_status_GET( src ),  dst  );\
    }

/**
*Obstacle distances in front of the sensor, starting from the left in increment degrees to the right */

typedef Pack OBSTACLE_DISTANCE_obstacle_distance; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pobstacle_distance_OBSTACLE_DISTANCE;// data navigator over pack fields data
/**
															* Wrap OBSTACLE_DISTANCE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pobstacle_distance_OBSTACLE_DISTANCE *OBSTACLE_DISTANCE_obstacle_distance_wrap(OBSTACLE_DISTANCE_obstacle_distance *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline OBSTACLE_DISTANCE_obstacle_distance *pobstacle_distance_OBSTACLE_DISTANCE_unwrap(pobstacle_distance_OBSTACLE_DISTANCE *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vobstacle_distance_distances;
//Maximum field array length constant
#define Pobstacle_distance_distances_len  ( 72 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_distances_SET( Vobstacle_distance_distances * src, <DST> * dst  ){}
static inline void <DST>_min_distance_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_max_distance_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_increment_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_sensor_type_SET( e_MAV_DISTANCE_SENSOR * src, <DST> * dst  ){}
*/

#define pobstacle_distance_OBSTACLE_DISTANCE_PUSH_INTO(DST)\
    static inline void pobstacle_distance_OBSTACLE_DISTANCE_push_into_##DST ( pobstacle_distance_OBSTACLE_DISTANCE * src, DST * dst) {\
        Vobstacle_distance_distances item_distances = pobstacle_distance_distances_GET( src  );\
       DST##_distances_SET( &item_distances, dst );\
        DST##_min_distance_SET( pobstacle_distance_min_distance_GET( src  ), dst  );\
        DST##_max_distance_SET( pobstacle_distance_max_distance_GET( src  ), dst  );\
        DST##_time_usec_SET( pobstacle_distance_time_usec_GET( src  ), dst  );\
        DST##_increment_SET( pobstacle_distance_increment_GET( src  ), dst  );\
        e_MAV_DISTANCE_SENSOR  item_sensor_type;\
        if( pobstacle_distance_sensor_type_GET( src, &item_sensor_type ) ){\
            DST##_sensor_type_SET( item_sensor_type , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_distances_GET( <SRC> * src, Vobstacle_distance_distances * dst ){}
static inline int16_t <SRC>_min_distance_GET( <SRC> * src ){}
static inline int16_t <SRC>_max_distance_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_increment_GET( <SRC> * src ){}
static inline bool  <SRC>_sensor_type_item_exist( <SRC> * src ){}
static inline e_MAV_DISTANCE_SENSOR <SRC>_sensor_type_GET( <SRC> * src ){}
*/

#define pobstacle_distance_OBSTACLE_DISTANCE_PULL_FROM(SRC)\
    static inline void pobstacle_distance_OBSTACLE_DISTANCE_pull_from_##SRC ( SRC * src, pobstacle_distance_OBSTACLE_DISTANCE * dst) {\
       Vobstacle_distance_distances item_distances = pobstacle_distance_distances_SET( NULL, dst  );\
       SRC##_distances_GET( src, &item_distances );\
        pobstacle_distance_min_distance_SET( SRC##_min_distance_GET(src ), dst  );\
        pobstacle_distance_max_distance_SET( SRC##_max_distance_GET(src ), dst  );\
        pobstacle_distance_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pobstacle_distance_increment_SET( SRC##_increment_GET(src ), dst  );\
        if( SRC##_sensor_type_item_exist(src ) )\
        pobstacle_distance_sensor_type_SET( pobstacle_distance_OBSTACLE_DISTANCE_sensor_type_GET( src ),  dst  );\
    }

/**
*Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame). */

typedef Pack GPS2_RAW_gps2_raw; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgps2_raw_GPS2_RAW;// data navigator over pack fields data
/**
															* Wrap GPS2_RAW in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgps2_raw_GPS2_RAW *GPS2_RAW_gps2_raw_wrap(GPS2_RAW_gps2_raw *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GPS2_RAW_gps2_raw *pgps2_raw_GPS2_RAW_unwrap(pgps2_raw_GPS2_RAW *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_eph_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_epv_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vel_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_cog_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_dgps_age_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_satellites_visible_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_dgps_numch_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_fix_type_SET( e_GPS_FIX_TYPE * src, <DST> * dst  ){}
*/

#define pgps2_raw_GPS2_RAW_PUSH_INTO(DST)\
    static inline void pgps2_raw_GPS2_RAW_push_into_##DST ( pgps2_raw_GPS2_RAW * src, DST * dst) {\
        DST##_eph_SET( pgps2_raw_eph_GET( src  ), dst  );\
        DST##_epv_SET( pgps2_raw_epv_GET( src  ), dst  );\
        DST##_vel_SET( pgps2_raw_vel_GET( src  ), dst  );\
        DST##_cog_SET( pgps2_raw_cog_GET( src  ), dst  );\
        DST##_dgps_age_SET( pgps2_raw_dgps_age_GET( src  ), dst  );\
        DST##_time_usec_SET( pgps2_raw_time_usec_GET( src  ), dst  );\
        DST##_lat_SET( pgps2_raw_lat_GET( src  ), dst  );\
        DST##_lon_SET( pgps2_raw_lon_GET( src  ), dst  );\
        DST##_alt_SET( pgps2_raw_alt_GET( src  ), dst  );\
        DST##_satellites_visible_SET( pgps2_raw_satellites_visible_GET( src  ), dst  );\
        DST##_dgps_numch_SET( pgps2_raw_dgps_numch_GET( src  ), dst  );\
        e_GPS_FIX_TYPE  item_fix_type;\
        if( pgps2_raw_fix_type_GET( src, &item_fix_type ) ){\
            DST##_fix_type_SET( item_fix_type , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_eph_GET( <SRC> * src ){}
static inline int16_t <SRC>_epv_GET( <SRC> * src ){}
static inline int16_t <SRC>_vel_GET( <SRC> * src ){}
static inline int16_t <SRC>_cog_GET( <SRC> * src ){}
static inline int32_t <SRC>_dgps_age_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline int32_t <SRC>_alt_GET( <SRC> * src ){}
static inline int8_t <SRC>_satellites_visible_GET( <SRC> * src ){}
static inline int8_t <SRC>_dgps_numch_GET( <SRC> * src ){}
static inline bool  <SRC>_fix_type_item_exist( <SRC> * src ){}
static inline e_GPS_FIX_TYPE <SRC>_fix_type_GET( <SRC> * src ){}
*/

#define pgps2_raw_GPS2_RAW_PULL_FROM(SRC)\
    static inline void pgps2_raw_GPS2_RAW_pull_from_##SRC ( SRC * src, pgps2_raw_GPS2_RAW * dst) {\
        pgps2_raw_eph_SET( SRC##_eph_GET(src ), dst  );\
        pgps2_raw_epv_SET( SRC##_epv_GET(src ), dst  );\
        pgps2_raw_vel_SET( SRC##_vel_GET(src ), dst  );\
        pgps2_raw_cog_SET( SRC##_cog_GET(src ), dst  );\
        pgps2_raw_dgps_age_SET( SRC##_dgps_age_GET(src ), dst  );\
        pgps2_raw_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pgps2_raw_lat_SET( SRC##_lat_GET(src ), dst  );\
        pgps2_raw_lon_SET( SRC##_lon_GET(src ), dst  );\
        pgps2_raw_alt_SET( SRC##_alt_GET(src ), dst  );\
        pgps2_raw_satellites_visible_SET( SRC##_satellites_visible_GET(src ), dst  );\
        pgps2_raw_dgps_numch_SET( SRC##_dgps_numch_GET(src ), dst  );\
        if( SRC##_fix_type_item_exist(src ) )\
        pgps2_raw_fix_type_SET( pgps2_raw_GPS2_RAW_fix_type_GET( src ),  dst  );\
    }

/**
*THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD. */

typedef Pack REQUEST_DATA_STREAM_request_data_stream; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t prequest_data_stream_REQUEST_DATA_STREAM;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline prequest_data_stream_REQUEST_DATA_STREAM *prequest_data_stream_REQUEST_DATA_STREAM_from(REQUEST_DATA_STREAM_request_data_stream *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_req_message_rate_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_req_stream_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_start_stop_SET( int8_t * src, <DST> * dst  ){}
*/

#define prequest_data_stream_REQUEST_DATA_STREAM_PUSH_INTO(DST)\
    static inline void prequest_data_stream_REQUEST_DATA_STREAM_push_into_##DST ( prequest_data_stream_REQUEST_DATA_STREAM * src, DST * dst) {\
        DST##_req_message_rate_SET( prequest_data_stream_req_message_rate_GET( src  ), dst  );\
        DST##_target_system_SET( prequest_data_stream_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( prequest_data_stream_target_component_GET( src  ), dst  );\
        DST##_req_stream_id_SET( prequest_data_stream_req_stream_id_GET( src  ), dst  );\
        DST##_start_stop_SET( prequest_data_stream_start_stop_GET( src  ), dst  );\
    }

/**
*Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
*				 way for testing new messages and getting experimental debug output */

typedef Pack MEMORY_VECT_memory_vect; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmemory_vect_MEMORY_VECT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmemory_vect_MEMORY_VECT *pmemory_vect_MEMORY_VECT_from(MEMORY_VECT_memory_vect *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vmemory_vect_value;
//Maximum field array length constant
#define Pmemory_vect_value_len  ( 32 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_address_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ver_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_typE_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_value_SET( Vmemory_vect_value * src, <DST> * dst  ){}
*/

#define pmemory_vect_MEMORY_VECT_PUSH_INTO(DST)\
    static inline void pmemory_vect_MEMORY_VECT_push_into_##DST ( pmemory_vect_MEMORY_VECT * src, DST * dst) {\
        DST##_address_SET( pmemory_vect_address_GET( src  ), dst  );\
        DST##_ver_SET( pmemory_vect_ver_GET( src  ), dst  );\
        DST##_typE_SET( pmemory_vect_typE_GET( src  ), dst  );\
        Vmemory_vect_value item_value = pmemory_vect_value_GET( src  );\
       DST##_value_SET( &item_value, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_address_GET( <SRC> * src ){}
static inline int8_t <SRC>_ver_GET( <SRC> * src ){}
static inline int8_t <SRC>_typE_GET( <SRC> * src ){}
static inline int8_t <SRC>_value_GET( <SRC> * src, Vmemory_vect_value * dst ){}
*/

#define pmemory_vect_MEMORY_VECT_PULL_FROM(SRC)\
    static inline void pmemory_vect_MEMORY_VECT_pull_from_##SRC ( SRC * src, pmemory_vect_MEMORY_VECT * dst) {\
        pmemory_vect_address_SET( SRC##_address_GET(src ), dst  );\
        pmemory_vect_ver_SET( SRC##_ver_GET(src ), dst  );\
        pmemory_vect_typE_SET( SRC##_typE_GET(src ), dst  );\
       Vmemory_vect_value item_value = pmemory_vect_value_SET( NULL, dst  );\
       SRC##_value_GET( src, &item_value );\
    }

/**
*Request to read the value of a parameter with the either the param_id string id or param_index. */

typedef Pack PARAM_EXT_REQUEST_READ_param_ext_request_read; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pparam_ext_request_read_PARAM_EXT_REQUEST_READ;// data navigator over pack fields data
/**
															* Wrap PARAM_EXT_REQUEST_READ in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pparam_ext_request_read_PARAM_EXT_REQUEST_READ *PARAM_EXT_REQUEST_READ_param_ext_request_read_wrap(PARAM_EXT_REQUEST_READ_param_ext_request_read *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PARAM_EXT_REQUEST_READ_param_ext_request_read *pparam_ext_request_read_PARAM_EXT_REQUEST_READ_unwrap(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_ext_request_read_param_id;
//Maximum field array length constant
#define Pparam_ext_request_read_param_id_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_param_id_SET( Vparam_ext_request_read_param_id * src, <DST> * dst  ){}
*/

#define pparam_ext_request_read_PARAM_EXT_REQUEST_READ_PUSH_INTO(DST)\
    static inline void pparam_ext_request_read_PARAM_EXT_REQUEST_READ_push_into_##DST ( pparam_ext_request_read_PARAM_EXT_REQUEST_READ * src, DST * dst) {\
        DST##_target_system_SET( pparam_ext_request_read_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pparam_ext_request_read_target_component_GET( src  ), dst  );\
        DST##_param_index_SET( pparam_ext_request_read_param_index_GET( src  ), dst  );\
        Vparam_ext_request_read_param_id  item_param_id;\
        if( pparam_ext_request_read_param_id_GET( src, &item_param_id ) ){\
            DST##_param_id_SET( &item_param_id, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int16_t <SRC>_param_index_GET( <SRC> * src ){}
static inline size_t  <SRC>_param_id_item_exist( <SRC> * src  ){}
static inline void <SRC>_param_id_GET( <SRC> * src, Vparam_ext_request_read_param_id * dst ){}
*/

#define pparam_ext_request_read_PARAM_EXT_REQUEST_READ_PULL_FROM(SRC)\
    static inline void pparam_ext_request_read_PARAM_EXT_REQUEST_READ_pull_from_##SRC ( SRC * src, pparam_ext_request_read_PARAM_EXT_REQUEST_READ * dst) {\
        pparam_ext_request_read_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pparam_ext_request_read_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pparam_ext_request_read_param_index_SET( SRC##_param_index_GET(src ), dst  );\
        const size_t len_param_id = SRC##_param_id_item_exist(src );\
        if( len_param_id ){\
            Vparam_ext_request_read_param_id    item_param_id = pparam_ext_request_read_param_id_SET( NULL, len_param_id, dst  );\
            pparam_ext_request_read_PARAM_EXT_REQUEST_READ_param_id_GET(src, &item_param_id );\
        }\
    }

/**
*Sent from autopilot to simulation. Hardware in the loop control outputs */

typedef Pack HIL_CONTROLS_hil_controls; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor phil_controls_HIL_CONTROLS;// data navigator over pack fields data
/**
															* Wrap HIL_CONTROLS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline phil_controls_HIL_CONTROLS *HIL_CONTROLS_hil_controls_wrap(HIL_CONTROLS_hil_controls *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline HIL_CONTROLS_hil_controls *phil_controls_HIL_CONTROLS_unwrap(phil_controls_HIL_CONTROLS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_roll_ailerons_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_elevator_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_rudder_SET( float * src, <DST> * dst  ){}
static inline void <DST>_throttle_SET( float * src, <DST> * dst  ){}
static inline void <DST>_aux1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_aux2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_aux3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_aux4_SET( float * src, <DST> * dst  ){}
static inline void <DST>_nav_mode_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mode_SET( e_MAV_MODE * src, <DST> * dst  ){}
*/

#define phil_controls_HIL_CONTROLS_PUSH_INTO(DST)\
    static inline void phil_controls_HIL_CONTROLS_push_into_##DST ( phil_controls_HIL_CONTROLS * src, DST * dst) {\
        DST##_time_usec_SET( phil_controls_time_usec_GET( src  ), dst  );\
        DST##_roll_ailerons_SET( phil_controls_roll_ailerons_GET( src  ), dst  );\
        DST##_pitch_elevator_SET( phil_controls_pitch_elevator_GET( src  ), dst  );\
        DST##_yaw_rudder_SET( phil_controls_yaw_rudder_GET( src  ), dst  );\
        DST##_throttle_SET( phil_controls_throttle_GET( src  ), dst  );\
        DST##_aux1_SET( phil_controls_aux1_GET( src  ), dst  );\
        DST##_aux2_SET( phil_controls_aux2_GET( src  ), dst  );\
        DST##_aux3_SET( phil_controls_aux3_GET( src  ), dst  );\
        DST##_aux4_SET( phil_controls_aux4_GET( src  ), dst  );\
        DST##_nav_mode_SET( phil_controls_nav_mode_GET( src  ), dst  );\
        e_MAV_MODE  item_mode;\
        if( phil_controls_mode_GET( src, &item_mode ) ){\
            DST##_mode_SET( item_mode , dst  );\
        }\
    }

/**
*The IMU readings in SI units in NED body frame */

typedef Pack HIL_SENSOR_hil_sensor; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t phil_sensor_HIL_SENSOR;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline phil_sensor_HIL_SENSOR *phil_sensor_HIL_SENSOR_from(HIL_SENSOR_hil_sensor *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_fields_updated_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ygyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xmag_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ymag_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zmag_SET( float * src, <DST> * dst  ){}
static inline void <DST>_abs_pressure_SET( float * src, <DST> * dst  ){}
static inline void <DST>_diff_pressure_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pressure_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( float * src, <DST> * dst  ){}
*/

#define phil_sensor_HIL_SENSOR_PUSH_INTO(DST)\
    static inline void phil_sensor_HIL_SENSOR_push_into_##DST ( phil_sensor_HIL_SENSOR * src, DST * dst) {\
        DST##_fields_updated_SET( phil_sensor_fields_updated_GET( src  ), dst  );\
        DST##_time_usec_SET( phil_sensor_time_usec_GET( src  ), dst  );\
        DST##_xacc_SET( phil_sensor_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( phil_sensor_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( phil_sensor_zacc_GET( src  ), dst  );\
        DST##_xgyro_SET( phil_sensor_xgyro_GET( src  ), dst  );\
        DST##_ygyro_SET( phil_sensor_ygyro_GET( src  ), dst  );\
        DST##_zgyro_SET( phil_sensor_zgyro_GET( src  ), dst  );\
        DST##_xmag_SET( phil_sensor_xmag_GET( src  ), dst  );\
        DST##_ymag_SET( phil_sensor_ymag_GET( src  ), dst  );\
        DST##_zmag_SET( phil_sensor_zmag_GET( src  ), dst  );\
        DST##_abs_pressure_SET( phil_sensor_abs_pressure_GET( src  ), dst  );\
        DST##_diff_pressure_SET( phil_sensor_diff_pressure_GET( src  ), dst  );\
        DST##_pressure_alt_SET( phil_sensor_pressure_alt_GET( src  ), dst  );\
        DST##_temperature_SET( phil_sensor_temperature_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_fields_updated_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_xacc_GET( <SRC> * src ){}
static inline float <SRC>_yacc_GET( <SRC> * src ){}
static inline float <SRC>_zacc_GET( <SRC> * src ){}
static inline float <SRC>_xgyro_GET( <SRC> * src ){}
static inline float <SRC>_ygyro_GET( <SRC> * src ){}
static inline float <SRC>_zgyro_GET( <SRC> * src ){}
static inline float <SRC>_xmag_GET( <SRC> * src ){}
static inline float <SRC>_ymag_GET( <SRC> * src ){}
static inline float <SRC>_zmag_GET( <SRC> * src ){}
static inline float <SRC>_abs_pressure_GET( <SRC> * src ){}
static inline float <SRC>_diff_pressure_GET( <SRC> * src ){}
static inline float <SRC>_pressure_alt_GET( <SRC> * src ){}
static inline float <SRC>_temperature_GET( <SRC> * src ){}
*/

#define phil_sensor_HIL_SENSOR_PULL_FROM(SRC)\
    static inline void phil_sensor_HIL_SENSOR_pull_from_##SRC ( SRC * src, phil_sensor_HIL_SENSOR * dst) {\
        phil_sensor_fields_updated_SET( SRC##_fields_updated_GET(src ), dst  );\
        phil_sensor_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        phil_sensor_xacc_SET( SRC##_xacc_GET(src ), dst  );\
        phil_sensor_yacc_SET( SRC##_yacc_GET(src ), dst  );\
        phil_sensor_zacc_SET( SRC##_zacc_GET(src ), dst  );\
        phil_sensor_xgyro_SET( SRC##_xgyro_GET(src ), dst  );\
        phil_sensor_ygyro_SET( SRC##_ygyro_GET(src ), dst  );\
        phil_sensor_zgyro_SET( SRC##_zgyro_GET(src ), dst  );\
        phil_sensor_xmag_SET( SRC##_xmag_GET(src ), dst  );\
        phil_sensor_ymag_SET( SRC##_ymag_GET(src ), dst  );\
        phil_sensor_zmag_SET( SRC##_zmag_GET(src ), dst  );\
        phil_sensor_abs_pressure_SET( SRC##_abs_pressure_GET(src ), dst  );\
        phil_sensor_diff_pressure_SET( SRC##_diff_pressure_GET(src ), dst  );\
        phil_sensor_pressure_alt_SET( SRC##_pressure_alt_GET(src ), dst  );\
        phil_sensor_temperature_SET( SRC##_temperature_GET(src ), dst  );\
    }

/**
*Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
*				 signin */

typedef Pack SETUP_SIGNING_setup_signing; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t psetup_signing_SETUP_SIGNING;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline psetup_signing_SETUP_SIGNING *psetup_signing_SETUP_SIGNING_from(SETUP_SIGNING_setup_signing *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vsetup_signing_secret_key;
//Maximum field array length constant
#define Psetup_signing_secret_key_len  ( 32 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_initial_timestamp_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_secret_key_SET( Vsetup_signing_secret_key * src, <DST> * dst  ){}
*/

#define psetup_signing_SETUP_SIGNING_PUSH_INTO(DST)\
    static inline void psetup_signing_SETUP_SIGNING_push_into_##DST ( psetup_signing_SETUP_SIGNING * src, DST * dst) {\
        DST##_initial_timestamp_SET( psetup_signing_initial_timestamp_GET( src  ), dst  );\
        DST##_target_system_SET( psetup_signing_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( psetup_signing_target_component_GET( src  ), dst  );\
        Vsetup_signing_secret_key item_secret_key = psetup_signing_secret_key_GET( src  );\
       DST##_secret_key_SET( &item_secret_key, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_initial_timestamp_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_secret_key_GET( <SRC> * src, Vsetup_signing_secret_key * dst ){}
*/

#define psetup_signing_SETUP_SIGNING_PULL_FROM(SRC)\
    static inline void psetup_signing_SETUP_SIGNING_pull_from_##SRC ( SRC * src, psetup_signing_SETUP_SIGNING * dst) {\
        psetup_signing_initial_timestamp_SET( SRC##_initial_timestamp_GET(src ), dst  );\
        psetup_signing_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        psetup_signing_target_component_SET( SRC##_target_component_GET(src ), dst  );\
       Vsetup_signing_secret_key item_secret_key = psetup_signing_secret_key_SET( NULL, dst  );\
       SRC##_secret_key_GET( src, &item_secret_key );\
    }

/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting */

typedef Pack GPS_RTK_gps_rtk; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pgps_rtk_GPS_RTK;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pgps_rtk_GPS_RTK *pgps_rtk_GPS_RTK_from(GPS_RTK_gps_rtk *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_wn_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_last_baseline_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_tow_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_accuracy_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_rtk_receiver_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_rtk_health_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_rtk_rate_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_nsats_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_baseline_coords_type_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_baseline_a_mm_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_baseline_b_mm_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_baseline_c_mm_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_iar_num_hypotheses_SET( int32_t * src, <DST> * dst  ){}
*/

#define pgps_rtk_GPS_RTK_PUSH_INTO(DST)\
    static inline void pgps_rtk_GPS_RTK_push_into_##DST ( pgps_rtk_GPS_RTK * src, DST * dst) {\
        DST##_wn_SET( pgps_rtk_wn_GET( src  ), dst  );\
        DST##_time_last_baseline_ms_SET( pgps_rtk_time_last_baseline_ms_GET( src  ), dst  );\
        DST##_tow_SET( pgps_rtk_tow_GET( src  ), dst  );\
        DST##_accuracy_SET( pgps_rtk_accuracy_GET( src  ), dst  );\
        DST##_rtk_receiver_id_SET( pgps_rtk_rtk_receiver_id_GET( src  ), dst  );\
        DST##_rtk_health_SET( pgps_rtk_rtk_health_GET( src  ), dst  );\
        DST##_rtk_rate_SET( pgps_rtk_rtk_rate_GET( src  ), dst  );\
        DST##_nsats_SET( pgps_rtk_nsats_GET( src  ), dst  );\
        DST##_baseline_coords_type_SET( pgps_rtk_baseline_coords_type_GET( src  ), dst  );\
        DST##_baseline_a_mm_SET( pgps_rtk_baseline_a_mm_GET( src  ), dst  );\
        DST##_baseline_b_mm_SET( pgps_rtk_baseline_b_mm_GET( src  ), dst  );\
        DST##_baseline_c_mm_SET( pgps_rtk_baseline_c_mm_GET( src  ), dst  );\
        DST##_iar_num_hypotheses_SET( pgps_rtk_iar_num_hypotheses_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_wn_GET( <SRC> * src ){}
static inline int32_t <SRC>_time_last_baseline_ms_GET( <SRC> * src ){}
static inline int32_t <SRC>_tow_GET( <SRC> * src ){}
static inline int32_t <SRC>_accuracy_GET( <SRC> * src ){}
static inline int8_t <SRC>_rtk_receiver_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_rtk_health_GET( <SRC> * src ){}
static inline int8_t <SRC>_rtk_rate_GET( <SRC> * src ){}
static inline int8_t <SRC>_nsats_GET( <SRC> * src ){}
static inline int8_t <SRC>_baseline_coords_type_GET( <SRC> * src ){}
static inline int32_t <SRC>_baseline_a_mm_GET( <SRC> * src ){}
static inline int32_t <SRC>_baseline_b_mm_GET( <SRC> * src ){}
static inline int32_t <SRC>_baseline_c_mm_GET( <SRC> * src ){}
static inline int32_t <SRC>_iar_num_hypotheses_GET( <SRC> * src ){}
*/

#define pgps_rtk_GPS_RTK_PULL_FROM(SRC)\
    static inline void pgps_rtk_GPS_RTK_pull_from_##SRC ( SRC * src, pgps_rtk_GPS_RTK * dst) {\
        pgps_rtk_wn_SET( SRC##_wn_GET(src ), dst  );\
        pgps_rtk_time_last_baseline_ms_SET( SRC##_time_last_baseline_ms_GET(src ), dst  );\
        pgps_rtk_tow_SET( SRC##_tow_GET(src ), dst  );\
        pgps_rtk_accuracy_SET( SRC##_accuracy_GET(src ), dst  );\
        pgps_rtk_rtk_receiver_id_SET( SRC##_rtk_receiver_id_GET(src ), dst  );\
        pgps_rtk_rtk_health_SET( SRC##_rtk_health_GET(src ), dst  );\
        pgps_rtk_rtk_rate_SET( SRC##_rtk_rate_GET(src ), dst  );\
        pgps_rtk_nsats_SET( SRC##_nsats_GET(src ), dst  );\
        pgps_rtk_baseline_coords_type_SET( SRC##_baseline_coords_type_GET(src ), dst  );\
        pgps_rtk_baseline_a_mm_SET( SRC##_baseline_a_mm_GET(src ), dst  );\
        pgps_rtk_baseline_b_mm_SET( SRC##_baseline_b_mm_GET(src ), dst  );\
        pgps_rtk_baseline_c_mm_SET( SRC##_baseline_c_mm_GET(src ), dst  );\
        pgps_rtk_iar_num_hypotheses_SET( SRC##_iar_num_hypotheses_GET(src ), dst  );\
    }

/**
*Request all parameters of this component. After this request, all parameters are emitted. */

typedef Pack PARAM_REQUEST_LIST_param_request_list; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pparam_request_list_PARAM_REQUEST_LIST;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pparam_request_list_PARAM_REQUEST_LIST *pparam_request_list_PARAM_REQUEST_LIST_from(PARAM_REQUEST_LIST_param_request_list *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define pparam_request_list_PARAM_REQUEST_LIST_PUSH_INTO(DST)\
    static inline void pparam_request_list_PARAM_REQUEST_LIST_push_into_##DST ( pparam_request_list_PARAM_REQUEST_LIST * src, DST * dst) {\
        DST##_target_system_SET( pparam_request_list_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pparam_request_list_target_component_GET( src  ), dst  );\
    }

/**
*Static data to configure the ADS-B transponder (send within 10 sec of a POR and every 10 sec thereafter */

typedef Pack UAVIONIX_ADSB_OUT_CFG_uavionix_adsb_out_cfg; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG;// data navigator over pack fields data
/**
															* Wrap UAVIONIX_ADSB_OUT_CFG in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *UAVIONIX_ADSB_OUT_CFG_uavionix_adsb_out_cfg_wrap(UAVIONIX_ADSB_OUT_CFG_uavionix_adsb_out_cfg *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline UAVIONIX_ADSB_OUT_CFG_uavionix_adsb_out_cfg *puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_unwrap(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vuavionix_adsb_out_cfg_callsign;
//Maximum field array length constant
#define Puavionix_adsb_out_cfg_callsign_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_stallSpeed_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ICAO_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_callsign_SET( Vuavionix_adsb_out_cfg_callsign * src, <DST> * dst  ){}
static inline void <DST>_emitterType_SET( e_ADSB_EMITTER_TYPE * src, <DST> * dst  ){}
static inline void <DST>_aircraftSize_SET( e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE * src, <DST> * dst  ){}
static inline void <DST>_gpsOffsetLat_SET( e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT * src, <DST> * dst  ){}
static inline void <DST>_gpsOffsetLon_SET( e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON * src, <DST> * dst  ){}
static inline void <DST>_rfSelect_SET( e_UAVIONIX_ADSB_OUT_RF_SELECT * src, <DST> * dst  ){}
*/

#define puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_PUSH_INTO(DST)\
    static inline void puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_push_into_##DST ( puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG * src, DST * dst) {\
        DST##_stallSpeed_SET( puavionix_adsb_out_cfg_stallSpeed_GET( src  ), dst  );\
        DST##_ICAO_SET( puavionix_adsb_out_cfg_ICAO_GET( src  ), dst  );\
        Vuavionix_adsb_out_cfg_callsign  item_callsign;\
        if( puavionix_adsb_out_cfg_callsign_GET( src, &item_callsign ) ){\
            DST##_callsign_SET( &item_callsign, dst );\
        }\
        e_ADSB_EMITTER_TYPE  item_emitterType;\
        if( puavionix_adsb_out_cfg_emitterType_GET( src, &item_emitterType ) ){\
            DST##_emitterType_SET( item_emitterType , dst  );\
        }\
        e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE  item_aircraftSize;\
        if( puavionix_adsb_out_cfg_aircraftSize_GET( src, &item_aircraftSize ) ){\
            DST##_aircraftSize_SET( item_aircraftSize , dst  );\
        }\
        e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT  item_gpsOffsetLat;\
        if( puavionix_adsb_out_cfg_gpsOffsetLat_GET( src, &item_gpsOffsetLat ) ){\
            DST##_gpsOffsetLat_SET( item_gpsOffsetLat , dst  );\
        }\
        e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON  item_gpsOffsetLon;\
        if( puavionix_adsb_out_cfg_gpsOffsetLon_GET( src, &item_gpsOffsetLon ) ){\
            DST##_gpsOffsetLon_SET( item_gpsOffsetLon , dst  );\
        }\
        e_UAVIONIX_ADSB_OUT_RF_SELECT  item_rfSelect;\
        if( puavionix_adsb_out_cfg_rfSelect_GET( src, &item_rfSelect ) ){\
            DST##_rfSelect_SET( item_rfSelect , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_stallSpeed_GET( <SRC> * src ){}
static inline int32_t <SRC>_ICAO_GET( <SRC> * src ){}
static inline size_t  <SRC>_callsign_item_exist( <SRC> * src  ){}
static inline void <SRC>_callsign_GET( <SRC> * src, Vuavionix_adsb_out_cfg_callsign * dst ){}
static inline bool  <SRC>_emitterType_item_exist( <SRC> * src ){}
static inline e_ADSB_EMITTER_TYPE <SRC>_emitterType_GET( <SRC> * src ){}
static inline bool  <SRC>_aircraftSize_item_exist( <SRC> * src ){}
static inline e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE <SRC>_aircraftSize_GET( <SRC> * src ){}
static inline bool  <SRC>_gpsOffsetLat_item_exist( <SRC> * src ){}
static inline e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT <SRC>_gpsOffsetLat_GET( <SRC> * src ){}
static inline bool  <SRC>_gpsOffsetLon_item_exist( <SRC> * src ){}
static inline e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON <SRC>_gpsOffsetLon_GET( <SRC> * src ){}
static inline bool  <SRC>_rfSelect_item_exist( <SRC> * src ){}
static inline e_UAVIONIX_ADSB_OUT_RF_SELECT <SRC>_rfSelect_GET( <SRC> * src ){}
*/

#define puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_PULL_FROM(SRC)\
    static inline void puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_pull_from_##SRC ( SRC * src, puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG * dst) {\
        puavionix_adsb_out_cfg_stallSpeed_SET( SRC##_stallSpeed_GET(src ), dst  );\
        puavionix_adsb_out_cfg_ICAO_SET( SRC##_ICAO_GET(src ), dst  );\
        const size_t len_callsign = SRC##_callsign_item_exist(src );\
        if( len_callsign ){\
            Vuavionix_adsb_out_cfg_callsign    item_callsign = puavionix_adsb_out_cfg_callsign_SET( NULL, len_callsign, dst  );\
            puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_callsign_GET(src, &item_callsign );\
        }\
        if( SRC##_emitterType_item_exist(src ) )\
        puavionix_adsb_out_cfg_emitterType_SET( puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_emitterType_GET( src ),  dst  );\
        if( SRC##_aircraftSize_item_exist(src ) )\
        puavionix_adsb_out_cfg_aircraftSize_SET( puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_aircraftSize_GET( src ),  dst  );\
        if( SRC##_gpsOffsetLat_item_exist(src ) )\
        puavionix_adsb_out_cfg_gpsOffsetLat_SET( puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_gpsOffsetLat_GET( src ),  dst  );\
        if( SRC##_gpsOffsetLon_item_exist(src ) )\
        puavionix_adsb_out_cfg_gpsOffsetLon_SET( puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_gpsOffsetLon_GET( src ),  dst  );\
        if( SRC##_rfSelect_item_exist(src ) )\
        puavionix_adsb_out_cfg_rfSelect_SET( puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG_rfSelect_GET( src ),  dst  );\
    }

/**
*The location of a landing area captured from a downward facing camera */

typedef Pack LANDING_TARGET_landing_target; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor planding_target_LANDING_TARGET;// data navigator over pack fields data
/**
															* Wrap LANDING_TARGET in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline planding_target_LANDING_TARGET *LANDING_TARGET_landing_target_wrap(LANDING_TARGET_landing_target *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline LANDING_TARGET_landing_target *planding_target_LANDING_TARGET_unwrap(planding_target_LANDING_TARGET *wrapper) { return unwrap_pack(wrapper); }

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vlanding_target_q;

#define planding_target_q_d0(planding_target_LANDING_TARGET_ptr)\
    for( size_t  d0=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qlanding_target_q * fld_q = planding_target_q( planding_target_LANDING_TARGET_ptr ); d0 <SIZE_MAX && fld_q; d0 = SIZE_MAX )\
                    for( Vlanding_target_q  item_q ; d0 < Planding_target_q_D0  && qlanding_target_q_GET( fld_q, &item_q , d0 ) ; d0++)
//Constant dimension size value
#define Planding_target_q_D0 ( 4)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef planding_target_LANDING_TARGET Qlanding_target_q;

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_target_num_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_angle_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_angle_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_distance_SET( float * src, <DST> * dst  ){}
static inline void <DST>_size_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_size_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q_SET( float * src, <DST> * dst , size_t d0 ){}
static inline void <DST>_typE_SET( e_LANDING_TARGET_TYPE * src, <DST> * dst  ){}
static inline void <DST>_position_valid_SET( int8_t * src, <DST> * dst  ){}
*/

#define planding_target_LANDING_TARGET_PUSH_INTO(DST)\
    static inline void planding_target_LANDING_TARGET_push_into_##DST ( planding_target_LANDING_TARGET * src, DST * dst) {\
        DST##_time_usec_SET( planding_target_time_usec_GET( src  ), dst  );\
        DST##_target_num_SET( planding_target_target_num_GET( src  ), dst  );\
        DST##_angle_x_SET( planding_target_angle_x_GET( src  ), dst  );\
        DST##_angle_y_SET( planding_target_angle_y_GET( src  ), dst  );\
        DST##_distance_SET( planding_target_distance_GET( src  ), dst  );\
        DST##_size_x_SET( planding_target_size_x_GET( src  ), dst  );\
        DST##_size_y_SET( planding_target_size_y_GET( src  ), dst  );\
        e_MAV_FRAME  item_frame;\
        if( planding_target_frame_GET( src, &item_frame ) ){\
            DST##_frame_SET( item_frame , dst  );\
        }\
        float  item_x;\
        if( planding_target_x_GET( src, &item_x ) ){\
            DST##_x_SET( item_x , dst  );\
        }\
        float  item_y;\
        if( planding_target_y_GET( src, &item_y ) ){\
            DST##_y_SET( item_y , dst  );\
        }\
        float  item_z;\
        if( planding_target_z_GET( src, &item_z ) ){\
            DST##_z_SET( item_z , dst  );\
        }\
        planding_target_q_d0 (src) { \
        DST##_q_SET( vlanding_target_q_GET( &item_q ), dst , d0 );\
        }\
        e_LANDING_TARGET_TYPE  item_typE;\
        if( planding_target_typE_GET( src, &item_typE ) ){\
            DST##_typE_SET( item_typE , dst  );\
        }\
        int8_t  item_position_valid;\
        if( planding_target_position_valid_GET( src, &item_position_valid ) ){\
            DST##_position_valid_SET( item_position_valid , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_num_GET( <SRC> * src ){}
static inline float <SRC>_angle_x_GET( <SRC> * src ){}
static inline float <SRC>_angle_y_GET( <SRC> * src ){}
static inline float <SRC>_distance_GET( <SRC> * src ){}
static inline float <SRC>_size_x_GET( <SRC> * src ){}
static inline float <SRC>_size_y_GET( <SRC> * src ){}
static inline bool  <SRC>_frame_item_exist( <SRC> * src ){}
static inline e_MAV_FRAME <SRC>_frame_GET( <SRC> * src ){}
static inline bool  <SRC>_x_item_exist( <SRC> * src ){}
static inline float <SRC>_x_GET( <SRC> * src ){}
static inline bool  <SRC>_y_item_exist( <SRC> * src ){}
static inline float <SRC>_y_GET( <SRC> * src ){}
static inline bool  <SRC>_z_item_exist( <SRC> * src ){}
static inline float <SRC>_z_GET( <SRC> * src ){}
static inline bool <SRC>_q_exist( <SRC> * src){}
static inline bool  <SRC>_q_item_exist( <SRC> * src , size_t d0){}
static inline float <SRC>_q_GET( <SRC> * src , size_t d0){}
static inline bool  <SRC>_typE_item_exist( <SRC> * src ){}
static inline e_LANDING_TARGET_TYPE <SRC>_typE_GET( <SRC> * src ){}
static inline bool  <SRC>_position_valid_item_exist( <SRC> * src ){}
static inline int8_t <SRC>_position_valid_GET( <SRC> * src ){}
*/

#define planding_target_LANDING_TARGET_PULL_FROM(SRC)\
    static inline void planding_target_LANDING_TARGET_pull_from_##SRC ( SRC * src, planding_target_LANDING_TARGET * dst) {\
        planding_target_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        planding_target_target_num_SET( SRC##_target_num_GET(src ), dst  );\
        planding_target_angle_x_SET( SRC##_angle_x_GET(src ), dst  );\
        planding_target_angle_y_SET( SRC##_angle_y_GET(src ), dst  );\
        planding_target_distance_SET( SRC##_distance_GET(src ), dst  );\
        planding_target_size_x_SET( SRC##_size_x_GET(src ), dst  );\
        planding_target_size_y_SET( SRC##_size_y_GET(src ), dst  );\
        if( SRC##_frame_item_exist(src ) )\
        planding_target_frame_SET( planding_target_LANDING_TARGET_frame_GET( src ),  dst  );\
        if( SRC##_x_item_exist(src ) )\
        planding_target_x_SET( planding_target_LANDING_TARGET_x_GET( src ),  dst  );\
        if( SRC##_y_item_exist(src ) )\
        planding_target_y_SET( planding_target_LANDING_TARGET_y_GET( src ),  dst  );\
        if( SRC##_z_item_exist(src ) )\
        planding_target_z_SET( planding_target_LANDING_TARGET_z_GET( src ),  dst  );\
        if( SRC##_q_exist( src ))\
                for( size_t d0=0; d0 < Planding_target_q_D0 ; d0++)\
           {\
        if( SRC##_q_item_exist(src , d0) )\
    planding_target_q_SET( planding_target_LANDING_TARGET_q_GET( src , d0),  dst , d0 );\
        }\
        if( SRC##_typE_item_exist(src ) )\
        planding_target_typE_SET( planding_target_LANDING_TARGET_typE_GET( src ),  dst  );\
        if( SRC##_position_valid_item_exist(src ) )\
        planding_target_position_valid_SET( planding_target_LANDING_TARGET_position_valid_GET( src ),  dst  );\
    }

/**
*Set the vehicle attitude and body angular rates. */

typedef Pack SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET_from(SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vset_actuator_control_target_controls;
//Maximum field array length constant
#define Pset_actuator_control_target_controls_len  ( 8 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_group_mlx_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_controls_SET( Vset_actuator_control_target_controls * src, <DST> * dst  ){}
*/

#define pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET_PUSH_INTO(DST)\
    static inline void pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET_push_into_##DST ( pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET * src, DST * dst) {\
        DST##_time_usec_SET( pset_actuator_control_target_time_usec_GET( src  ), dst  );\
        DST##_group_mlx_SET( pset_actuator_control_target_group_mlx_GET( src  ), dst  );\
        DST##_target_system_SET( pset_actuator_control_target_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pset_actuator_control_target_target_component_GET( src  ), dst  );\
        Vset_actuator_control_target_controls item_controls = pset_actuator_control_target_controls_GET( src  );\
       DST##_controls_SET( &item_controls, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_group_mlx_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline float <SRC>_controls_GET( <SRC> * src, Vset_actuator_control_target_controls * dst ){}
*/

#define pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET_PULL_FROM(SRC)\
    static inline void pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET_pull_from_##SRC ( SRC * src, pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET * dst) {\
        pset_actuator_control_target_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pset_actuator_control_target_group_mlx_SET( SRC##_group_mlx_GET(src ), dst  );\
        pset_actuator_control_target_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pset_actuator_control_target_target_component_SET( SRC##_target_component_GET(src ), dst  );\
       Vset_actuator_control_target_controls item_controls = pset_actuator_control_target_controls_SET( NULL, dst  );\
       SRC##_controls_GET( src, &item_controls );\
    }

/**
*The smoothed, monotonic system state used to feed the control loops of the system. */

typedef Pack CONTROL_SYSTEM_STATE_control_system_state; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pcontrol_system_state_CONTROL_SYSTEM_STATE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pcontrol_system_state_CONTROL_SYSTEM_STATE *pcontrol_system_state_CONTROL_SYSTEM_STATE_from(CONTROL_SYSTEM_STATE_control_system_state *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vcontrol_system_state_vel_variance;
//Maximum field array length constant
#define Pcontrol_system_state_vel_variance_len  ( 3 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vcontrol_system_state_pos_variance;
//Maximum field array length constant
#define Pcontrol_system_state_pos_variance_len  ( 3 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vcontrol_system_state_q;
//Maximum field array length constant
#define Pcontrol_system_state_q_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_x_acc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_acc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_acc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_x_vel_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_vel_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_vel_SET( float * src, <DST> * dst  ){}
static inline void <DST>_x_pos_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_pos_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_pos_SET( float * src, <DST> * dst  ){}
static inline void <DST>_airspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vel_variance_SET( Vcontrol_system_state_vel_variance * src, <DST> * dst  ){}
static inline void <DST>_pos_variance_SET( Vcontrol_system_state_pos_variance * src, <DST> * dst  ){}
static inline void <DST>_q_SET( Vcontrol_system_state_q * src, <DST> * dst  ){}
static inline void <DST>_roll_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_rate_SET( float * src, <DST> * dst  ){}
*/

#define pcontrol_system_state_CONTROL_SYSTEM_STATE_PUSH_INTO(DST)\
    static inline void pcontrol_system_state_CONTROL_SYSTEM_STATE_push_into_##DST ( pcontrol_system_state_CONTROL_SYSTEM_STATE * src, DST * dst) {\
        DST##_time_usec_SET( pcontrol_system_state_time_usec_GET( src  ), dst  );\
        DST##_x_acc_SET( pcontrol_system_state_x_acc_GET( src  ), dst  );\
        DST##_y_acc_SET( pcontrol_system_state_y_acc_GET( src  ), dst  );\
        DST##_z_acc_SET( pcontrol_system_state_z_acc_GET( src  ), dst  );\
        DST##_x_vel_SET( pcontrol_system_state_x_vel_GET( src  ), dst  );\
        DST##_y_vel_SET( pcontrol_system_state_y_vel_GET( src  ), dst  );\
        DST##_z_vel_SET( pcontrol_system_state_z_vel_GET( src  ), dst  );\
        DST##_x_pos_SET( pcontrol_system_state_x_pos_GET( src  ), dst  );\
        DST##_y_pos_SET( pcontrol_system_state_y_pos_GET( src  ), dst  );\
        DST##_z_pos_SET( pcontrol_system_state_z_pos_GET( src  ), dst  );\
        DST##_airspeed_SET( pcontrol_system_state_airspeed_GET( src  ), dst  );\
        Vcontrol_system_state_vel_variance item_vel_variance = pcontrol_system_state_vel_variance_GET( src  );\
       DST##_vel_variance_SET( &item_vel_variance, dst );\
        Vcontrol_system_state_pos_variance item_pos_variance = pcontrol_system_state_pos_variance_GET( src  );\
       DST##_pos_variance_SET( &item_pos_variance, dst );\
        Vcontrol_system_state_q item_q = pcontrol_system_state_q_GET( src  );\
       DST##_q_SET( &item_q, dst );\
        DST##_roll_rate_SET( pcontrol_system_state_roll_rate_GET( src  ), dst  );\
        DST##_pitch_rate_SET( pcontrol_system_state_pitch_rate_GET( src  ), dst  );\
        DST##_yaw_rate_SET( pcontrol_system_state_yaw_rate_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_x_acc_GET( <SRC> * src ){}
static inline float <SRC>_y_acc_GET( <SRC> * src ){}
static inline float <SRC>_z_acc_GET( <SRC> * src ){}
static inline float <SRC>_x_vel_GET( <SRC> * src ){}
static inline float <SRC>_y_vel_GET( <SRC> * src ){}
static inline float <SRC>_z_vel_GET( <SRC> * src ){}
static inline float <SRC>_x_pos_GET( <SRC> * src ){}
static inline float <SRC>_y_pos_GET( <SRC> * src ){}
static inline float <SRC>_z_pos_GET( <SRC> * src ){}
static inline float <SRC>_airspeed_GET( <SRC> * src ){}
static inline float <SRC>_vel_variance_GET( <SRC> * src, Vcontrol_system_state_vel_variance * dst ){}
static inline float <SRC>_pos_variance_GET( <SRC> * src, Vcontrol_system_state_pos_variance * dst ){}
static inline float <SRC>_q_GET( <SRC> * src, Vcontrol_system_state_q * dst ){}
static inline float <SRC>_roll_rate_GET( <SRC> * src ){}
static inline float <SRC>_pitch_rate_GET( <SRC> * src ){}
static inline float <SRC>_yaw_rate_GET( <SRC> * src ){}
*/

#define pcontrol_system_state_CONTROL_SYSTEM_STATE_PULL_FROM(SRC)\
    static inline void pcontrol_system_state_CONTROL_SYSTEM_STATE_pull_from_##SRC ( SRC * src, pcontrol_system_state_CONTROL_SYSTEM_STATE * dst) {\
        pcontrol_system_state_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pcontrol_system_state_x_acc_SET( SRC##_x_acc_GET(src ), dst  );\
        pcontrol_system_state_y_acc_SET( SRC##_y_acc_GET(src ), dst  );\
        pcontrol_system_state_z_acc_SET( SRC##_z_acc_GET(src ), dst  );\
        pcontrol_system_state_x_vel_SET( SRC##_x_vel_GET(src ), dst  );\
        pcontrol_system_state_y_vel_SET( SRC##_y_vel_GET(src ), dst  );\
        pcontrol_system_state_z_vel_SET( SRC##_z_vel_GET(src ), dst  );\
        pcontrol_system_state_x_pos_SET( SRC##_x_pos_GET(src ), dst  );\
        pcontrol_system_state_y_pos_SET( SRC##_y_pos_GET(src ), dst  );\
        pcontrol_system_state_z_pos_SET( SRC##_z_pos_GET(src ), dst  );\
        pcontrol_system_state_airspeed_SET( SRC##_airspeed_GET(src ), dst  );\
       Vcontrol_system_state_vel_variance item_vel_variance = pcontrol_system_state_vel_variance_SET( NULL, dst  );\
       SRC##_vel_variance_GET( src, &item_vel_variance );\
       Vcontrol_system_state_pos_variance item_pos_variance = pcontrol_system_state_pos_variance_SET( NULL, dst  );\
       SRC##_pos_variance_GET( src, &item_pos_variance );\
       Vcontrol_system_state_q item_q = pcontrol_system_state_q_SET( NULL, dst  );\
       SRC##_q_GET( src, &item_q );\
        pcontrol_system_state_roll_rate_SET( SRC##_roll_rate_GET(src ), dst  );\
        pcontrol_system_state_pitch_rate_SET( SRC##_pitch_rate_GET(src ), dst  );\
        pcontrol_system_state_yaw_rate_SET( SRC##_yaw_rate_GET(src ), dst  );\
    }

/**
*Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
*				 Used by an external controller to command the vehicle (manual controller or other system) */

typedef Pack SET_POSITION_TARGET_GLOBAL_INT_set_position_target_global_int; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT;// data navigator over pack fields data
/**
															* Wrap SET_POSITION_TARGET_GLOBAL_INT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *SET_POSITION_TARGET_GLOBAL_INT_set_position_target_global_int_wrap(SET_POSITION_TARGET_GLOBAL_INT_set_position_target_global_int *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SET_POSITION_TARGET_GLOBAL_INT_set_position_target_global_int *pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT_unwrap(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_type_mask_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_lat_int_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_int_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_coordinate_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
*/

#define pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT_PUSH_INTO(DST)\
    static inline void pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT_push_into_##DST ( pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT * src, DST * dst) {\
        DST##_type_mask_SET( pset_position_target_global_int_type_mask_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( pset_position_target_global_int_time_boot_ms_GET( src  ), dst  );\
        DST##_target_system_SET( pset_position_target_global_int_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pset_position_target_global_int_target_component_GET( src  ), dst  );\
        DST##_lat_int_SET( pset_position_target_global_int_lat_int_GET( src  ), dst  );\
        DST##_lon_int_SET( pset_position_target_global_int_lon_int_GET( src  ), dst  );\
        DST##_alt_SET( pset_position_target_global_int_alt_GET( src  ), dst  );\
        DST##_vx_SET( pset_position_target_global_int_vx_GET( src  ), dst  );\
        DST##_vy_SET( pset_position_target_global_int_vy_GET( src  ), dst  );\
        DST##_vz_SET( pset_position_target_global_int_vz_GET( src  ), dst  );\
        DST##_afx_SET( pset_position_target_global_int_afx_GET( src  ), dst  );\
        DST##_afy_SET( pset_position_target_global_int_afy_GET( src  ), dst  );\
        DST##_afz_SET( pset_position_target_global_int_afz_GET( src  ), dst  );\
        DST##_yaw_SET( pset_position_target_global_int_yaw_GET( src  ), dst  );\
        DST##_yaw_rate_SET( pset_position_target_global_int_yaw_rate_GET( src  ), dst  );\
        e_MAV_FRAME  item_coordinate_frame;\
        if( pset_position_target_global_int_coordinate_frame_GET( src, &item_coordinate_frame ) ){\
            DST##_coordinate_frame_SET( item_coordinate_frame , dst  );\
        }\
    }

/**
*Data packet, size 32 */

typedef Pack DATA32_data32; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdata32_DATA32;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdata32_DATA32 *pdata32_DATA32_from(DATA32_data32 *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdata32_daTa;
//Maximum field array length constant
#define Pdata32_daTa_len  ( 32 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_typE_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_len_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vdata32_daTa * src, <DST> * dst  ){}
*/

#define pdata32_DATA32_PUSH_INTO(DST)\
    static inline void pdata32_DATA32_push_into_##DST ( pdata32_DATA32 * src, DST * dst) {\
        DST##_typE_SET( pdata32_typE_GET( src  ), dst  );\
        DST##_len_SET( pdata32_len_GET( src  ), dst  );\
        Vdata32_daTa item_daTa = pdata32_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_typE_GET( <SRC> * src ){}
static inline int8_t <SRC>_len_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vdata32_daTa * dst ){}
*/

#define pdata32_DATA32_PULL_FROM(SRC)\
    static inline void pdata32_DATA32_pull_from_##SRC ( SRC * src, pdata32_DATA32 * dst) {\
        pdata32_typE_SET( SRC##_typE_GET(src ), dst  );\
        pdata32_len_SET( SRC##_len_GET(src ), dst  );\
       Vdata32_daTa item_daTa = pdata32_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }


typedef Pack PING33_ping33; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pping33_PING33;// data navigator over pack fields data
/**
															* Wrap PING33 in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pping33_PING33 *PING33_ping33_wrap(PING33_ping33 *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PING33_ping33 *pping33_PING33_unwrap(pping33_PING33 *wrapper) { return unwrap_pack(wrapper); }

#define pping33_TTTT_d0_d1_d2\
    for( size_t d0=0; d0 < Pping33_TTTT_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_TTTT_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_TTTT_D2 ; d2++)
//Constant dimension size value
#define Pping33_TTTT_D0 ( 3)
//Constant dimension size value
#define Pping33_TTTT_D1 ( 2)
//Constant dimension size value
#define Pping33_TTTT_D2 ( 3)

//value range validation facilities
#define pping33_bit_field_MIN (4)
#define pping33_bit_field_MAX (45)
#define pping33_bit_field_OK(VAL)  (pping33_bit_field_MIN <= (VAL) && (VAL) <= pping33_bit_field_MAX)

#define pping33_field6_d0_d1_d2\
    for( size_t d0=0; d0 < Pping33_field6_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field6_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_field6_D2 ; d2++)
//Constant dimension size value
#define Pping33_field6_D0 ( 3)
//Constant dimension size value
#define Pping33_field6_D1 ( 2)
//Constant dimension size value
#define Pping33_field6_D2 ( 3)

#define pping33_field1_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, D0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field1 * fld_field1 = pping33_field1( pping33_PING33_ptr , &D0 ); d0 <SIZE_MAX && fld_field1; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field1_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_field1_D2  ; d2++)
//Variable dimension maximum size value
#define Pping33_field1_D0_max  (7)
//Constant dimension size value
#define Pping33_field1_D1 ( 2)
//Constant dimension size value
#define Pping33_field1_D2 ( 3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_field1;

#define pping33_field12_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field12 * fld_field12 = pping33_field12( pping33_PING33_ptr ); d0 <SIZE_MAX && fld_field12; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < Pping33_field12_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field12_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_field12_D2  ; d2++)
//Constant dimension size value
#define Pping33_field12_D0 ( 3)
//Constant dimension size value
#define Pping33_field12_D1 ( 2)
//Constant dimension size value
#define Pping33_field12_D2 ( 3)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef pping33_PING33 Qping33_field12;

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vping33_field13;

#define pping33_field13_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field13 * fld_field13 = pping33_field13( pping33_PING33_ptr ); d0 <SIZE_MAX && fld_field13; d0 = SIZE_MAX )\
                    for( Vping33_field13  item_field13 ; d0 < Pping33_field13_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field13_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_field13_D2  && qping33_field13_GET( fld_field13, &item_field13 , d0, d1, d2 ) ; d2++)
//Constant dimension size value
#define Pping33_field13_D0 ( 3)
//Constant dimension size value
#define Pping33_field13_D1 ( 2)
//Constant dimension size value
#define Pping33_field13_D2 ( 3)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef pping33_PING33 Qping33_field13;

//value range validation facilities
#define pping33_bit_field2_MIN (4)
#define pping33_bit_field2_MAX (45)
#define pping33_bit_field2_OK(VAL)  (pping33_bit_field2_MIN <= (VAL) && (VAL) <= pping33_bit_field2_MAX)

//value range validation facilities
#define pping33_Field_Bits_MIN (4)
#define pping33_Field_Bits_MAX (45)
#define pping33_Field_Bits_OK(VAL)  (pping33_Field_Bits_MIN <= (VAL) && (VAL) <= pping33_Field_Bits_MAX)

#define pping33_Field_Bits_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_Field_Bits * fld_Field_Bits = pping33_Field_Bits( pping33_PING33_ptr ); d0 <SIZE_MAX && fld_Field_Bits; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < Pping33_Field_Bits_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_Field_Bits_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_Field_Bits_D2  ; d2++)
//Constant dimension size value
#define Pping33_Field_Bits_D0 ( 3)
//Constant dimension size value
#define Pping33_Field_Bits_D1 ( 3)
//Constant dimension size value
#define Pping33_Field_Bits_D2 ( 3)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef pping33_PING33 Qping33_Field_Bits;

//value range validation facilities
#define pping33_SparseFixAllBits_MIN (4)
#define pping33_SparseFixAllBits_MAX (45)
#define pping33_SparseFixAllBits_OK(VAL)  (pping33_SparseFixAllBits_MIN <= (VAL) && (VAL) <= pping33_SparseFixAllBits_MAX)

/**
																				* This struct hold information about none primitive field parameters and used by functions to access field data
																				*/
typedef BitsValue Vping33_SparseFixAllBits;

#define pping33_SparseFixAllBits_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, D0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_SparseFixAllBits * fld_SparseFixAllBits = pping33_SparseFixAllBits( pping33_PING33_ptr , &D0 ); d0 <SIZE_MAX && fld_SparseFixAllBits; d0 = SIZE_MAX )\
                    for( Vping33_SparseFixAllBits  item_SparseFixAllBits ; d0 < D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_SparseFixAllBits_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_SparseFixAllBits_D2  && qping33_SparseFixAllBits_GET( fld_SparseFixAllBits, &item_SparseFixAllBits , d0, d1, d2 ) ; d2++)
//Variable dimension maximum size value
#define Pping33_SparseFixAllBits_D0_max  (3)
//Constant dimension size value
#define Pping33_SparseFixAllBits_D1 ( 3)
//Constant dimension size value
#define Pping33_SparseFixAllBits_D2 ( 3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_SparseFixAllBits;

//value range validation facilities
#define pping33_FixAllBits_MIN (14)
#define pping33_FixAllBits_MAX (45)
#define pping33_FixAllBits_OK(VAL)  (pping33_FixAllBits_MIN <= (VAL) && (VAL) <= pping33_FixAllBits_MAX)

#define pping33_FixAllBits_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, D0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_FixAllBits * fld_FixAllBits = pping33_FixAllBits( pping33_PING33_ptr , &D0 ); d0 <SIZE_MAX && fld_FixAllBits; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_FixAllBits_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_FixAllBits_D2  ; d2++)
//Variable dimension maximum size value
#define Pping33_FixAllBits_D0_max  (3)
//Constant dimension size value
#define Pping33_FixAllBits_D1 ( 3)
//Constant dimension size value
#define Pping33_FixAllBits_D2 ( 3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_FixAllBits;

//value range validation facilities
#define pping33_VarAllBits_MIN (14)
#define pping33_VarAllBits_MAX (45)
#define pping33_VarAllBits_OK(VAL)  (pping33_VarAllBits_MIN <= (VAL) && (VAL) <= pping33_VarAllBits_MAX)

#define pping33_VarAllBits_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, D0, d1=0, d2=0, D2 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_VarAllBits * fld_VarAllBits = pping33_VarAllBits( pping33_PING33_ptr , &D0 , &D2 ); d0 <SIZE_MAX && fld_VarAllBits; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_VarAllBits_D1  ; d1++)\
                    for( d2 = 0 ; d2 < D2  ; d2++)
//Variable dimension maximum size value
#define Pping33_VarAllBits_D0_max  (3)
//Constant dimension size value
#define Pping33_VarAllBits_D1 ( 3)
//Variable dimension maximum size value
#define Pping33_VarAllBits_D2_max  (3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_VarAllBits;

//value range validation facilities
#define pping33_SparseVarAllBits_MIN (14)
#define pping33_SparseVarAllBits_MAX (45)
#define pping33_SparseVarAllBits_OK(VAL)  (pping33_SparseVarAllBits_MIN <= (VAL) && (VAL) <= pping33_SparseVarAllBits_MAX)

/**
																				* This struct hold information about none primitive field parameters and used by functions to access field data
																				*/
typedef BitsValue Vping33_SparseVarAllBits;

#define pping33_SparseVarAllBits_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, D0, d1=0, d2=0, D2 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_SparseVarAllBits * fld_SparseVarAllBits = pping33_SparseVarAllBits( pping33_PING33_ptr , &D0 , &D2 ); d0 <SIZE_MAX && fld_SparseVarAllBits; d0 = SIZE_MAX )\
                    for( Vping33_SparseVarAllBits  item_SparseVarAllBits ; d0 < D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_SparseVarAllBits_D1  ; d1++)\
                    for( d2 = 0 ; d2 < D2  && qping33_SparseVarAllBits_GET( fld_SparseVarAllBits, &item_SparseVarAllBits , d0, d1, d2 ) ; d2++)
//Variable dimension maximum size value
#define Pping33_SparseVarAllBits_D0_max  (3)
//Constant dimension size value
#define Pping33_SparseVarAllBits_D1 ( 3)
//Variable dimension maximum size value
#define Pping33_SparseVarAllBits_D2_max  (3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_SparseVarAllBits;

//value range validation facilities
#define pping33_VarEachBits_MIN (-14)
#define pping33_VarEachBits_MAX (45)
#define pping33_VarEachBits_OK(VAL)  (pping33_VarEachBits_MIN <= (VAL) && (VAL) <= pping33_VarEachBits_MAX)

#define pping33_VarEachBits_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, D0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_VarEachBits * fld_VarEachBits = pping33_VarEachBits( pping33_PING33_ptr , &D0 ); d0 <SIZE_MAX && fld_VarEachBits; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_VarEachBits_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_VarEachBits_D2  ; d2++)
//Variable dimension maximum size value
#define Pping33_VarEachBits_D0_max  (3)
//Constant dimension size value
#define Pping33_VarEachBits_D1 ( 3)
//Constant dimension size value
#define Pping33_VarEachBits_D2 ( 3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_VarEachBits;

//value range validation facilities
#define pping33_SparsVarEachBits_MIN (-14)
#define pping33_SparsVarEachBits_MAX (450)
#define pping33_SparsVarEachBits_OK(VAL)  (pping33_SparsVarEachBits_MIN <= (VAL) && (VAL) <= pping33_SparsVarEachBits_MAX)

/**
																				* This struct hold information about none primitive field parameters and used by functions to access field data
																				*/
typedef BitsValue Vping33_SparsVarEachBits;

#define pping33_SparsVarEachBits_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, D0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_SparsVarEachBits * fld_SparsVarEachBits = pping33_SparsVarEachBits( pping33_PING33_ptr , &D0 ); d0 <SIZE_MAX && fld_SparsVarEachBits; d0 = SIZE_MAX )\
                    for( Vping33_SparsVarEachBits  item_SparsVarEachBits ; d0 < D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_SparsVarEachBits_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_SparsVarEachBits_D2  && qping33_SparsVarEachBits_GET( fld_SparsVarEachBits, &item_SparsVarEachBits , d0, d1, d2 ) ; d2++)
//Variable dimension maximum size value
#define Pping33_SparsVarEachBits_D0_max  (3)
//Constant dimension size value
#define Pping33_SparsVarEachBits_D1 ( 3)
//Constant dimension size value
#define Pping33_SparsVarEachBits_D2 ( 3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_SparsVarEachBits;

#define pping33_field44_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0, D2 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field44 * fld_field44 = pping33_field44( pping33_PING33_ptr , &D2 ); d0 <SIZE_MAX && fld_field44; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < Pping33_field44_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field44_D1  ; d1++)\
                    for( d2 = 0 ; d2 < D2  ; d2++)
//Constant dimension size value
#define Pping33_field44_D0 ( 3)
//Constant dimension size value
#define Pping33_field44_D1 ( 2)
//Variable dimension maximum size value
#define Pping33_field44_D2_max  (3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_field44;

#define pping33_field634_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0, D2 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field634 * fld_field634 = pping33_field634( pping33_PING33_ptr , &D2 ); d0 <SIZE_MAX && fld_field634; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < Pping33_field634_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field634_D1  ; d1++)\
                    for( d2 = 0 ; d2 < D2  ; d2++)
//Constant dimension size value
#define Pping33_field634_D0 ( 3)
//Constant dimension size value
#define Pping33_field634_D1 ( 2)
//Variable dimension maximum size value
#define Pping33_field634_D2_max  (3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_field634;

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vping33_field33344;

#define pping33_field33344_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0, D2 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field33344 * fld_field33344 = pping33_field33344( pping33_PING33_ptr , &D2 ); d0 <SIZE_MAX && fld_field33344; d0 = SIZE_MAX )\
                    for( Vping33_field33344  item_field33344 ; d0 < Pping33_field33344_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field33344_D1  ; d1++)\
                    for( d2 = 0 ; d2 < D2  && qping33_field33344_GET( fld_field33344, &item_field33344 , d0, d1, d2 ) ; d2++)
//Constant dimension size value
#define Pping33_field33344_D0 ( 3)
//Constant dimension size value
#define Pping33_field33344_D1 ( 2)
//Variable dimension maximum size value
#define Pping33_field33344_D2_max  (3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_field33344;

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vping33_field333634;

#define pping33_field333634_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0, D2 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field333634 * fld_field333634 = pping33_field333634( pping33_PING33_ptr , &D2 ); d0 <SIZE_MAX && fld_field333634; d0 = SIZE_MAX )\
                    for( Vping33_field333634  item_field333634 ; d0 < Pping33_field333634_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field333634_D1  ; d1++)\
                    for( d2 = 0 ; d2 < D2  && qping33_field333634_GET( fld_field333634, &item_field333634 , d0, d1, d2 ) ; d2++)
//Constant dimension size value
#define Pping33_field333634_D0 ( 3)
//Constant dimension size value
#define Pping33_field333634_D1 ( 2)
//Variable dimension maximum size value
#define Pping33_field333634_D2_max  (3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_field333634;

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vping33_field__;

#define pping33_field___d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field__ * fld_field__ = pping33_field__( pping33_PING33_ptr ); d0 <SIZE_MAX && fld_field__; d0 = SIZE_MAX )\
                    for( Vping33_field__  item_field__ ; d0 < Pping33_field___D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field___D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_field___D2  && qping33_field___GET( fld_field__, &item_field__ , d0, d1, d2 ) ; d2++)
//Constant dimension size value
#define Pping33_field___D0 ( 3)
//Constant dimension size value
#define Pping33_field___D1 ( 2)
//Constant dimension size value
#define Pping33_field___D2 ( 3)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef pping33_PING33 Qping33_field__;

#define pping33_field63_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0, D2 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field63 * fld_field63 = pping33_field63( pping33_PING33_ptr , &D2 ); d0 <SIZE_MAX && fld_field63; d0 = SIZE_MAX )\
                    for( d0 = 0 ; d0 < Pping33_field63_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field63_D1  ; d1++)\
                    for( d2 = 0 ; d2 < D2  ; d2++)
//Constant dimension size value
#define Pping33_field63_D0 ( 3)
//Constant dimension size value
#define Pping33_field63_D1 ( 2)
//Variable dimension maximum size value
#define Pping33_field63_D2_max  (3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_field63;

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vping33_uid2;

#define pping33_uid2_d0(pping33_PING33_ptr)\
    for( size_t  d0=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_uid2 * fld_uid2 = pping33_uid2( pping33_PING33_ptr ); d0 <SIZE_MAX && fld_uid2; d0 = SIZE_MAX )\
                    for( Vping33_uid2  item_uid2 ; d0 < Pping33_uid2_D0  && qping33_uid2_GET( fld_uid2, &item_uid2 , d0 ) ; d0++)
//Constant dimension size value
#define Pping33_uid2_D0 ( 18)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef pping33_PING33 Qping33_uid2;

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vping33_field2;

#define pping33_field2_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field2 * fld_field2 = pping33_field2( pping33_PING33_ptr ); d0 <SIZE_MAX && fld_field2; d0 = SIZE_MAX )\
                    for( Vping33_field2  item_field2 ; d0 < Pping33_field2_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field2_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_field2_D2  && qping33_field2_GET( fld_field2, &item_field2 , d0, d1, d2 ) ; d2++)
//Constant dimension size value
#define Pping33_field2_D0 ( 3)
//Constant dimension size value
#define Pping33_field2_D1 ( 2)
//Constant dimension size value
#define Pping33_field2_D2 ( 3)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef pping33_PING33 Qping33_field2;

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vping33_field4;

#define pping33_field4_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_field4 * fld_field4 = pping33_field4( pping33_PING33_ptr ); d0 <SIZE_MAX && fld_field4; d0 = SIZE_MAX )\
                    for( Vping33_field4  item_field4 ; d0 < Pping33_field4_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_field4_D1  ; d1++)\
                    for( d2 = 0 ; d2 < Pping33_field4_D2  && qping33_field4_GET( fld_field4, &item_field4 , d0, d1, d2 ) ; d2++)
//Constant dimension size value
#define Pping33_field4_D0 ( 3)
//Constant dimension size value
#define Pping33_field4_D1 ( 2)
//Constant dimension size value
#define Pping33_field4_D2 ( 3)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef pping33_PING33 Qping33_field4;

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vping33_stringtest1;
//Maximum field array length constant
#define Pping33_stringtest1_len_max  ( 255 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vping33_stringtest2;
//Maximum field array length constant
#define Pping33_stringtest2_len_max  ( 255 )

#define pping33_stringtest2_d0_d1_d2(pping33_PING33_ptr)\
    for( size_t  d0=0, d1=0, d2=0, D2 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qping33_stringtest2 * fld_stringtest2 = pping33_stringtest2( pping33_PING33_ptr , &D2 ); d0 <SIZE_MAX && fld_stringtest2; d0 = SIZE_MAX )\
                    for( Vping33_stringtest2  item_stringtest2 ; d0 < Pping33_stringtest2_D0  ; d0++)\
                for( d1 = 0 ; d1 < Pping33_stringtest2_D1  ; d1++)\
                    for( d2 = 0 ; d2 < D2  && qping33_stringtest2_GET( fld_stringtest2, &item_stringtest2 , d0, d1, d2 ) ; d2++)
//Constant dimension size value
#define Pping33_stringtest2_D0 ( 3)
//Constant dimension size value
#define Pping33_stringtest2_D1 ( 2)
//Variable dimension maximum size value
#define Pping33_stringtest2_D2_max  (3)

/**
												*Optional field current state information. Used in functions to access field data.
												*/
typedef pping33_PING33 Qping33_stringtest2;

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vping33_stringtest3;
//Maximum field array length constant
#define Pping33_stringtest3_len_max  ( 255 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vping33_stringtest4;
//Maximum field array length constant
#define Pping33_stringtest4_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_TTTT_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_bit_field_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_field6_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_testBOOL2_SET( bool * src, <DST> * dst  ){}
static inline void <DST>_testBOOL3_SET( bool * src, <DST> * dst  ){}
static inline void <DST>_testBOOL_SET( bool * src, <DST> * dst  ){}
static inline void <DST>_seq_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_field1_set_params( <DST> * dst , size_t * d0 ){}
static inline void <DST>_field1_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field12_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field13_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_WWWWWWWW_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_bit_field2_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_Field_Bits_SET( int8_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_SparseFixAllBits_set_params( <DST> * dst , size_t * d0 ){}
static inline void <DST>_SparseFixAllBits_SET( int8_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_FixAllBits_set_params( <DST> * dst , size_t * d0 ){}
static inline void <DST>_FixAllBits_SET( int8_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_VarAllBits_set_params( <DST> * dst , size_t * d0, size_t * d2 ){}
static inline void <DST>_VarAllBits_SET( int8_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_SparseVarAllBits_set_params( <DST> * dst , size_t * d0, size_t * d2 ){}
static inline void <DST>_SparseVarAllBits_SET( int8_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_VarEachBits_set_params( <DST> * dst , size_t * d0 ){}
static inline void <DST>_VarEachBits_SET( int8_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_SparsVarEachBits_set_params( <DST> * dst , size_t * d0 ){}
static inline void <DST>_SparsVarEachBits_SET( int16_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_testBOOLX_SET( bool * src, <DST> * dst  ){}
static inline void <DST>_testBOOL2X_SET( bool * src, <DST> * dst  ){}
static inline void <DST>_testBOOL3X_SET( bool * src, <DST> * dst  ){}
static inline void <DST>_MMMMMM_SET( e_MAV_MODE * src, <DST> * dst  ){}
static inline void <DST>_field44_set_params( <DST> * dst , size_t * d2 ){}
static inline void <DST>_field44_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field634_set_params( <DST> * dst , size_t * d2 ){}
static inline void <DST>_field634_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field33344_set_params( <DST> * dst , size_t * d2 ){}
static inline void <DST>_field33344_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field333634_set_params( <DST> * dst , size_t * d2 ){}
static inline void <DST>_field333634_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field___SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field63_set_params( <DST> * dst , size_t * d2 ){}
static inline void <DST>_field63_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_uid2_SET( int8_t * src, <DST> * dst , size_t d0 ){}
static inline void <DST>_field2_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_field4_SET( int32_t * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_stringtest1_SET( Vping33_stringtest1 * src, <DST> * dst  ){}
static inline void <DST>_stringtest2_set_params( <DST> * dst , size_t * d2 ){}
static inline void <DST>_stringtest2_SET( Vping33_stringtest2 * src, <DST> * dst , size_t d0, size_t d1, size_t d2 ){}
static inline void <DST>_stringtest3_SET( Vping33_stringtest3 * src, <DST> * dst  ){}
static inline void <DST>_stringtest4_SET( Vping33_stringtest4 * src, <DST> * dst  ){}
*/

#define pping33_PING33_PUSH_INTO(DST)\
    static inline void pping33_PING33_push_into_##DST ( pping33_PING33 * src, DST * dst) {\
        pping33_TTTT_d0_d1_d2{\
        DST##_TTTT_SET( pping33_TTTT_GET( src , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
        DST##_field_SET( pping33_field_GET( src  ), dst  );\
        DST##_bit_field_SET( pping33_bit_field_GET( src  ), dst  );\
        pping33_field6_d0_d1_d2{\
        DST##_field6_SET( pping33_field6_GET( src , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
        DST##_testBOOL2_SET( pping33_testBOOL2_GET( src  ), dst  );\
        DST##_testBOOL3_SET( pping33_testBOOL3_GET( src  ), dst  );\
        bool  item_testBOOL;\
        if( pping33_testBOOL_GET( src, &item_testBOOL ) ){\
            DST##_testBOOL_SET( item_testBOOL , dst  );\
        }\
        int64_t  item_seq;\
        if( pping33_seq_GET( src, &item_seq ) ){\
            DST##_seq_SET( item_seq , dst  );\
        }\
           size_t pping33_field1_d0 = 0;\
        if( pping33_field1( src , &pping33_field1_d0 ) && ( DST##_field1_set_params( dst , &pping33_field1_d0 ), true) )\
            pping33_field1_d0_d1_d2( src ) \
           {\
        DST##_field1_SET( qping33_field1_GET( fld_field1 , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
        pping33_field12_d0_d1_d2 (src) { \
        DST##_field12_SET( qping33_field12_GET( fld_field12 , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
        pping33_field13_d0_d1_d2 (src) { \
        DST##_field13_SET( vping33_field13_GET( &item_field13 ), dst , d0, d1, d2 );\
        }\
        int32_t  item_WWWWWWWW;\
        if( pping33_WWWWWWWW_GET( src, &item_WWWWWWWW ) ){\
            DST##_WWWWWWWW_SET( item_WWWWWWWW , dst  );\
        }\
        int8_t  item_bit_field2;\
        if( pping33_bit_field2_GET( src, &item_bit_field2 ) ){\
            DST##_bit_field2_SET( item_bit_field2 , dst  );\
        }\
        pping33_Field_Bits_d0_d1_d2 (src) { \
        DST##_Field_Bits_SET( qping33_Field_Bits_GET( fld_Field_Bits , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
           size_t pping33_SparseFixAllBits_d0 = 0;\
        if( pping33_SparseFixAllBits( src , &pping33_SparseFixAllBits_d0 ) && ( DST##_SparseFixAllBits_set_params( dst , &pping33_SparseFixAllBits_d0 ), true) )\
            pping33_SparseFixAllBits_d0_d1_d2( src ) \
           {\
        DST##_SparseFixAllBits_SET( vping33_SparseFixAllBits_GET( &item_SparseFixAllBits ), dst , d0, d1, d2 );\
        }\
           size_t pping33_FixAllBits_d0 = 0;\
        if( pping33_FixAllBits( src , &pping33_FixAllBits_d0 ) && ( DST##_FixAllBits_set_params( dst , &pping33_FixAllBits_d0 ), true) )\
            pping33_FixAllBits_d0_d1_d2( src ) \
           {\
        DST##_FixAllBits_SET( qping33_FixAllBits_GET( fld_FixAllBits , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
           size_t pping33_VarAllBits_d0 = 0;\
        size_t pping33_VarAllBits_d2 = 0;\
        if( pping33_VarAllBits( src , &pping33_VarAllBits_d0, &pping33_VarAllBits_d2 ) && ( DST##_VarAllBits_set_params( dst , &pping33_VarAllBits_d0, &pping33_VarAllBits_d2 ), true) )\
            pping33_VarAllBits_d0_d1_d2( src ) \
           {\
        DST##_VarAllBits_SET( qping33_VarAllBits_GET( fld_VarAllBits , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
           size_t pping33_SparseVarAllBits_d0 = 0;\
        size_t pping33_SparseVarAllBits_d2 = 0;\
        if( pping33_SparseVarAllBits( src , &pping33_SparseVarAllBits_d0, &pping33_SparseVarAllBits_d2 ) && ( DST##_SparseVarAllBits_set_params( dst , &pping33_SparseVarAllBits_d0, &pping33_SparseVarAllBits_d2 ), true) )\
            pping33_SparseVarAllBits_d0_d1_d2( src ) \
           {\
        DST##_SparseVarAllBits_SET( vping33_SparseVarAllBits_GET( &item_SparseVarAllBits ), dst , d0, d1, d2 );\
        }\
           size_t pping33_VarEachBits_d0 = 0;\
        if( pping33_VarEachBits( src , &pping33_VarEachBits_d0 ) && ( DST##_VarEachBits_set_params( dst , &pping33_VarEachBits_d0 ), true) )\
            pping33_VarEachBits_d0_d1_d2( src ) \
           {\
        DST##_VarEachBits_SET( qping33_VarEachBits_GET( fld_VarEachBits , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
           size_t pping33_SparsVarEachBits_d0 = 0;\
        if( pping33_SparsVarEachBits( src , &pping33_SparsVarEachBits_d0 ) && ( DST##_SparsVarEachBits_set_params( dst , &pping33_SparsVarEachBits_d0 ), true) )\
            pping33_SparsVarEachBits_d0_d1_d2( src ) \
           {\
        DST##_SparsVarEachBits_SET( vping33_SparsVarEachBits_GET( &item_SparsVarEachBits ), dst , d0, d1, d2 );\
        }\
        bool  item_testBOOLX;\
        if( pping33_testBOOLX_GET( src, &item_testBOOLX ) ){\
            DST##_testBOOLX_SET( item_testBOOLX , dst  );\
        }\
        bool  item_testBOOL2X;\
        if( pping33_testBOOL2X_GET( src, &item_testBOOL2X ) ){\
            DST##_testBOOL2X_SET( item_testBOOL2X , dst  );\
        }\
        bool  item_testBOOL3X;\
        if( pping33_testBOOL3X_GET( src, &item_testBOOL3X ) ){\
            DST##_testBOOL3X_SET( item_testBOOL3X , dst  );\
        }\
        e_MAV_MODE  item_MMMMMM;\
        if( pping33_MMMMMM_GET( src, &item_MMMMMM ) ){\
            DST##_MMMMMM_SET( item_MMMMMM , dst  );\
        }\
           size_t pping33_field44_d2 = 0;\
        if( pping33_field44( src , &pping33_field44_d2 ) && ( DST##_field44_set_params( dst , &pping33_field44_d2 ), true) )\
            pping33_field44_d0_d1_d2( src ) \
           {\
        DST##_field44_SET( qping33_field44_GET( fld_field44 , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
           size_t pping33_field634_d2 = 0;\
        if( pping33_field634( src , &pping33_field634_d2 ) && ( DST##_field634_set_params( dst , &pping33_field634_d2 ), true) )\
            pping33_field634_d0_d1_d2( src ) \
           {\
        DST##_field634_SET( qping33_field634_GET( fld_field634 , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
           size_t pping33_field33344_d2 = 0;\
        if( pping33_field33344( src , &pping33_field33344_d2 ) && ( DST##_field33344_set_params( dst , &pping33_field33344_d2 ), true) )\
            pping33_field33344_d0_d1_d2( src ) \
           {\
        DST##_field33344_SET( vping33_field33344_GET( &item_field33344 ), dst , d0, d1, d2 );\
        }\
           size_t pping33_field333634_d2 = 0;\
        if( pping33_field333634( src , &pping33_field333634_d2 ) && ( DST##_field333634_set_params( dst , &pping33_field333634_d2 ), true) )\
            pping33_field333634_d0_d1_d2( src ) \
           {\
        DST##_field333634_SET( vping33_field333634_GET( &item_field333634 ), dst , d0, d1, d2 );\
        }\
        pping33_field___d0_d1_d2 (src) { \
        DST##_field___SET( vping33_field___GET( &item_field__ ), dst , d0, d1, d2 );\
        }\
           size_t pping33_field63_d2 = 0;\
        if( pping33_field63( src , &pping33_field63_d2 ) && ( DST##_field63_set_params( dst , &pping33_field63_d2 ), true) )\
            pping33_field63_d0_d1_d2( src ) \
           {\
        DST##_field63_SET( qping33_field63_GET( fld_field63 , d0, d1, d2 ), dst , d0, d1, d2 );\
        }\
        pping33_uid2_d0 (src) { \
        DST##_uid2_SET( vping33_uid2_GET( &item_uid2 ), dst , d0 );\
        }\
        pping33_field2_d0_d1_d2 (src) { \
        DST##_field2_SET( vping33_field2_GET( &item_field2 ), dst , d0, d1, d2 );\
        }\
        pping33_field4_d0_d1_d2 (src) { \
        DST##_field4_SET( vping33_field4_GET( &item_field4 ), dst , d0, d1, d2 );\
        }\
        Vping33_stringtest1  item_stringtest1;\
        if( pping33_stringtest1_GET( src, &item_stringtest1 ) ){\
            DST##_stringtest1_SET( &item_stringtest1, dst );\
        }\
           size_t pping33_stringtest2_d2 = 0;\
        if( pping33_stringtest2( src , &pping33_stringtest2_d2 ) && ( DST##_stringtest2_set_params( dst , &pping33_stringtest2_d2 ), true) )\
            pping33_stringtest2_d0_d1_d2( src ) \
           {\
            DST##_stringtest2_SET( &item_stringtest2, dst , d0, d1, d2);\
        }\
        Vping33_stringtest3  item_stringtest3;\
        if( pping33_stringtest3_GET( src, &item_stringtest3 ) ){\
            DST##_stringtest3_SET( &item_stringtest3, dst );\
        }\
        Vping33_stringtest4  item_stringtest4;\
        if( pping33_stringtest4_GET( src, &item_stringtest4 ) ){\
            DST##_stringtest4_SET( &item_stringtest4, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_TTTT_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int64_t <SRC>_field_GET( <SRC> * src ){}
static inline int8_t <SRC>_bit_field_GET( <SRC> * src ){}
static inline int32_t <SRC>_field6_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_testBOOL2_GET( <SRC> * src ){}
static inline bool <SRC>_testBOOL3_GET( <SRC> * src ){}
static inline bool  <SRC>_testBOOL_item_exist( <SRC> * src ){}
static inline bool <SRC>_testBOOL_GET( <SRC> * src ){}
static inline bool  <SRC>_seq_item_exist( <SRC> * src ){}
static inline int64_t <SRC>_seq_GET( <SRC> * src ){}
static inline bool <SRC>_field1_get_params( <SRC> * src , size_t * d0){}
static inline int32_t <SRC>_field1_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_field12_exist( <SRC> * src){}
static inline int32_t <SRC>_field12_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_field13_exist( <SRC> * src){}
static inline bool  <SRC>_field13_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int32_t <SRC>_field13_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool  <SRC>_WWWWWWWW_item_exist( <SRC> * src ){}
static inline int32_t <SRC>_WWWWWWWW_GET( <SRC> * src ){}
static inline bool  <SRC>_bit_field2_item_exist( <SRC> * src ){}
static inline int8_t <SRC>_bit_field2_GET( <SRC> * src ){}
static inline bool <SRC>_Field_Bits_exist( <SRC> * src){}
static inline int8_t <SRC>_Field_Bits_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_SparseFixAllBits_get_params( <SRC> * src , size_t * d0){}
static inline bool  <SRC>_SparseFixAllBits_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int8_t <SRC>_SparseFixAllBits_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_FixAllBits_get_params( <SRC> * src , size_t * d0){}
static inline int8_t <SRC>_FixAllBits_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_VarAllBits_get_params( <SRC> * src , size_t * d0, size_t * d2){}
static inline int8_t <SRC>_VarAllBits_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_SparseVarAllBits_get_params( <SRC> * src , size_t * d0, size_t * d2){}
static inline bool  <SRC>_SparseVarAllBits_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int8_t <SRC>_SparseVarAllBits_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_VarEachBits_get_params( <SRC> * src , size_t * d0){}
static inline int8_t <SRC>_VarEachBits_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_SparsVarEachBits_get_params( <SRC> * src , size_t * d0){}
static inline bool  <SRC>_SparsVarEachBits_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int16_t <SRC>_SparsVarEachBits_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool  <SRC>_testBOOLX_item_exist( <SRC> * src ){}
static inline bool <SRC>_testBOOLX_GET( <SRC> * src ){}
static inline bool  <SRC>_testBOOL2X_item_exist( <SRC> * src ){}
static inline bool <SRC>_testBOOL2X_GET( <SRC> * src ){}
static inline bool  <SRC>_testBOOL3X_item_exist( <SRC> * src ){}
static inline bool <SRC>_testBOOL3X_GET( <SRC> * src ){}
static inline bool  <SRC>_MMMMMM_item_exist( <SRC> * src ){}
static inline e_MAV_MODE <SRC>_MMMMMM_GET( <SRC> * src ){}
static inline bool <SRC>_field44_get_params( <SRC> * src , size_t * d2){}
static inline int32_t <SRC>_field44_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_field634_get_params( <SRC> * src , size_t * d2){}
static inline int32_t <SRC>_field634_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_field33344_get_params( <SRC> * src , size_t * d2){}
static inline bool  <SRC>_field33344_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int32_t <SRC>_field33344_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_field333634_get_params( <SRC> * src , size_t * d2){}
static inline bool  <SRC>_field333634_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int32_t <SRC>_field333634_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_field___exist( <SRC> * src){}
static inline bool  <SRC>_field___item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int32_t <SRC>_field___GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_field63_get_params( <SRC> * src , size_t * d2){}
static inline int32_t <SRC>_field63_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_uid2_exist( <SRC> * src){}
static inline bool  <SRC>_uid2_item_exist( <SRC> * src , size_t d0){}
static inline int8_t <SRC>_uid2_GET( <SRC> * src , size_t d0){}
static inline bool <SRC>_field2_exist( <SRC> * src){}
static inline bool  <SRC>_field2_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int32_t <SRC>_field2_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline bool <SRC>_field4_exist( <SRC> * src){}
static inline bool  <SRC>_field4_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline int32_t <SRC>_field4_GET( <SRC> * src , size_t d0, size_t d1, size_t d2){}
static inline size_t  <SRC>_stringtest1_item_exist( <SRC> * src  ){}
static inline void <SRC>_stringtest1_GET( <SRC> * src, Vping33_stringtest1 * dst ){}
static inline bool <SRC>_stringtest2_get_params( <SRC> * src , size_t * d2){}
static inline size_t  <SRC>_stringtest2_item_exist( <SRC> * src , size_t d0, size_t d1, size_t d2 ){}
static inline void <SRC>_stringtest2_GET( <SRC> * src, Vping33_stringtest2 * dst , size_t d0, size_t d1, size_t d2){}
static inline size_t  <SRC>_stringtest3_item_exist( <SRC> * src  ){}
static inline void <SRC>_stringtest3_GET( <SRC> * src, Vping33_stringtest3 * dst ){}
static inline size_t  <SRC>_stringtest4_item_exist( <SRC> * src  ){}
static inline void <SRC>_stringtest4_GET( <SRC> * src, Vping33_stringtest4 * dst ){}
*/

#define pping33_PING33_PULL_FROM(SRC)\
    static inline void pping33_PING33_pull_from_##SRC ( SRC * src, pping33_PING33 * dst) {\
pping33_TTTT_d0_d1_d2{\
        pping33_TTTT_SET( SRC##_TTTT_GET(src , d0, d1, d2), dst , d0, d1, d2 );\
        }\
        pping33_field_SET( SRC##_field_GET(src ), dst  );\
        pping33_bit_field_SET( SRC##_bit_field_GET(src ), dst  );\
pping33_field6_d0_d1_d2{\
        pping33_field6_SET( SRC##_field6_GET(src , d0, d1, d2), dst , d0, d1, d2 );\
        }\
        pping33_testBOOL2_SET( SRC##_testBOOL2_GET(src ), dst  );\
        pping33_testBOOL3_SET( SRC##_testBOOL3_GET(src ), dst  );\
        if( SRC##_testBOOL_item_exist(src ) )\
        pping33_testBOOL_SET( pping33_PING33_testBOOL_GET( src ),  dst  );\
        if( SRC##_seq_item_exist(src ) )\
        pping33_seq_SET( pping33_PING33_seq_GET( src ),  dst  );\
            size_t pping33_field1_d0 = 0;\
        if( SRC##_field1_get_params( src , &pping33_field1_d0 ) && ( pping33_field1( dst , &pping33_field1_d0 ), true) )\
                for( size_t d0=0; d0 < pping33_field1_d0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field1_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_field1_D2 ; d2++)\
           {\
        qping33_field1_SET( pping33_PING33_field1_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
        if( SRC##_field12_exist( src ))\
                for( size_t d0=0; d0 < Pping33_field12_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field12_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_field12_D2 ; d2++)\
           {\
        pping33_field12_SET( pping33_PING33_field12_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
        if( SRC##_field13_exist( src ))\
                for( size_t d0=0; d0 < Pping33_field13_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field13_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_field13_D2 ; d2++)\
           {\
        if( SRC##_field13_item_exist(src , d0, d1, d2) )\
    pping33_field13_SET( pping33_PING33_field13_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
        if( SRC##_WWWWWWWW_item_exist(src ) )\
        pping33_WWWWWWWW_SET( pping33_PING33_WWWWWWWW_GET( src ),  dst  );\
        if( SRC##_bit_field2_item_exist(src ) )\
        pping33_bit_field2_SET( pping33_PING33_bit_field2_GET( src ),  dst  );\
        if( SRC##_Field_Bits_exist( src ))\
                for( size_t d0=0; d0 < Pping33_Field_Bits_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_Field_Bits_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_Field_Bits_D2 ; d2++)\
           {\
        pping33_Field_Bits_SET( pping33_PING33_Field_Bits_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_SparseFixAllBits_d0 = 0;\
        if( SRC##_SparseFixAllBits_get_params( src , &pping33_SparseFixAllBits_d0 ) && ( pping33_SparseFixAllBits( dst , &pping33_SparseFixAllBits_d0 ), true) )\
                for( size_t d0=0; d0 < pping33_SparseFixAllBits_d0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_SparseFixAllBits_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_SparseFixAllBits_D2 ; d2++)\
           {\
        if( SRC##_SparseFixAllBits_item_exist(src , d0, d1, d2) )\
    qping33_SparseFixAllBits_SET( pping33_PING33_SparseFixAllBits_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_FixAllBits_d0 = 0;\
        if( SRC##_FixAllBits_get_params( src , &pping33_FixAllBits_d0 ) && ( pping33_FixAllBits( dst , &pping33_FixAllBits_d0 ), true) )\
                for( size_t d0=0; d0 < pping33_FixAllBits_d0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_FixAllBits_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_FixAllBits_D2 ; d2++)\
           {\
        qping33_FixAllBits_SET( pping33_PING33_FixAllBits_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_VarAllBits_d0 = 0;\
        size_t pping33_VarAllBits_d2 = 0;\
        if( SRC##_VarAllBits_get_params( src , &pping33_VarAllBits_d0, &pping33_VarAllBits_d2 ) && ( pping33_VarAllBits( dst , &pping33_VarAllBits_d0, &pping33_VarAllBits_d2 ), true) )\
                for( size_t d0=0; d0 < pping33_VarAllBits_d0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_VarAllBits_D1 ; d1++)\
            for( size_t d2=0; d2 < pping33_VarAllBits_d2 ; d2++)\
           {\
        qping33_VarAllBits_SET( pping33_PING33_VarAllBits_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_SparseVarAllBits_d0 = 0;\
        size_t pping33_SparseVarAllBits_d2 = 0;\
        if( SRC##_SparseVarAllBits_get_params( src , &pping33_SparseVarAllBits_d0, &pping33_SparseVarAllBits_d2 ) && ( pping33_SparseVarAllBits( dst , &pping33_SparseVarAllBits_d0, &pping33_SparseVarAllBits_d2 ), true) )\
                for( size_t d0=0; d0 < pping33_SparseVarAllBits_d0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_SparseVarAllBits_D1 ; d1++)\
            for( size_t d2=0; d2 < pping33_SparseVarAllBits_d2 ; d2++)\
           {\
        if( SRC##_SparseVarAllBits_item_exist(src , d0, d1, d2) )\
    qping33_SparseVarAllBits_SET( pping33_PING33_SparseVarAllBits_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_VarEachBits_d0 = 0;\
        if( SRC##_VarEachBits_get_params( src , &pping33_VarEachBits_d0 ) && ( pping33_VarEachBits( dst , &pping33_VarEachBits_d0 ), true) )\
                for( size_t d0=0; d0 < pping33_VarEachBits_d0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_VarEachBits_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_VarEachBits_D2 ; d2++)\
           {\
        qping33_VarEachBits_SET( pping33_PING33_VarEachBits_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_SparsVarEachBits_d0 = 0;\
        if( SRC##_SparsVarEachBits_get_params( src , &pping33_SparsVarEachBits_d0 ) && ( pping33_SparsVarEachBits( dst , &pping33_SparsVarEachBits_d0 ), true) )\
                for( size_t d0=0; d0 < pping33_SparsVarEachBits_d0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_SparsVarEachBits_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_SparsVarEachBits_D2 ; d2++)\
           {\
        if( SRC##_SparsVarEachBits_item_exist(src , d0, d1, d2) )\
    qping33_SparsVarEachBits_SET( pping33_PING33_SparsVarEachBits_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
        if( SRC##_testBOOLX_item_exist(src ) )\
        pping33_testBOOLX_SET( pping33_PING33_testBOOLX_GET( src ),  dst  );\
        if( SRC##_testBOOL2X_item_exist(src ) )\
        pping33_testBOOL2X_SET( pping33_PING33_testBOOL2X_GET( src ),  dst  );\
        if( SRC##_testBOOL3X_item_exist(src ) )\
        pping33_testBOOL3X_SET( pping33_PING33_testBOOL3X_GET( src ),  dst  );\
        if( SRC##_MMMMMM_item_exist(src ) )\
        pping33_MMMMMM_SET( pping33_PING33_MMMMMM_GET( src ),  dst  );\
            size_t pping33_field44_d2 = 0;\
        if( SRC##_field44_get_params( src , &pping33_field44_d2 ) && ( pping33_field44( dst , &pping33_field44_d2 ), true) )\
                for( size_t d0=0; d0 < Pping33_field44_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field44_D1 ; d1++)\
            for( size_t d2=0; d2 < pping33_field44_d2 ; d2++)\
           {\
        qping33_field44_SET( pping33_PING33_field44_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_field634_d2 = 0;\
        if( SRC##_field634_get_params( src , &pping33_field634_d2 ) && ( pping33_field634( dst , &pping33_field634_d2 ), true) )\
                for( size_t d0=0; d0 < Pping33_field634_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field634_D1 ; d1++)\
            for( size_t d2=0; d2 < pping33_field634_d2 ; d2++)\
           {\
        qping33_field634_SET( pping33_PING33_field634_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_field33344_d2 = 0;\
        if( SRC##_field33344_get_params( src , &pping33_field33344_d2 ) && ( pping33_field33344( dst , &pping33_field33344_d2 ), true) )\
                for( size_t d0=0; d0 < Pping33_field33344_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field33344_D1 ; d1++)\
            for( size_t d2=0; d2 < pping33_field33344_d2 ; d2++)\
           {\
        if( SRC##_field33344_item_exist(src , d0, d1, d2) )\
    qping33_field33344_SET( pping33_PING33_field33344_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_field333634_d2 = 0;\
        if( SRC##_field333634_get_params( src , &pping33_field333634_d2 ) && ( pping33_field333634( dst , &pping33_field333634_d2 ), true) )\
                for( size_t d0=0; d0 < Pping33_field333634_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field333634_D1 ; d1++)\
            for( size_t d2=0; d2 < pping33_field333634_d2 ; d2++)\
           {\
        if( SRC##_field333634_item_exist(src , d0, d1, d2) )\
    qping33_field333634_SET( pping33_PING33_field333634_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
        if( SRC##_field___exist( src ))\
                for( size_t d0=0; d0 < Pping33_field___D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field___D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_field___D2 ; d2++)\
           {\
        if( SRC##_field___item_exist(src , d0, d1, d2) )\
    pping33_field___SET( pping33_PING33_field___GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
            size_t pping33_field63_d2 = 0;\
        if( SRC##_field63_get_params( src , &pping33_field63_d2 ) && ( pping33_field63( dst , &pping33_field63_d2 ), true) )\
                for( size_t d0=0; d0 < Pping33_field63_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field63_D1 ; d1++)\
            for( size_t d2=0; d2 < pping33_field63_d2 ; d2++)\
           {\
        qping33_field63_SET( pping33_PING33_field63_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
        if( SRC##_uid2_exist( src ))\
                for( size_t d0=0; d0 < Pping33_uid2_D0 ; d0++)\
           {\
        if( SRC##_uid2_item_exist(src , d0) )\
    pping33_uid2_SET( pping33_PING33_uid2_GET( src , d0),  dst , d0 );\
        }\
        if( SRC##_field2_exist( src ))\
                for( size_t d0=0; d0 < Pping33_field2_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field2_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_field2_D2 ; d2++)\
           {\
        if( SRC##_field2_item_exist(src , d0, d1, d2) )\
    pping33_field2_SET( pping33_PING33_field2_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
        if( SRC##_field4_exist( src ))\
                for( size_t d0=0; d0 < Pping33_field4_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_field4_D1 ; d1++)\
            for( size_t d2=0; d2 < Pping33_field4_D2 ; d2++)\
           {\
        if( SRC##_field4_item_exist(src , d0, d1, d2) )\
    pping33_field4_SET( pping33_PING33_field4_GET( src , d0, d1, d2),  dst , d0, d1, d2 );\
        }\
        const size_t len_stringtest1 = SRC##_stringtest1_item_exist(src );\
        if( len_stringtest1 ){\
            Vping33_stringtest1    item_stringtest1 = pping33_stringtest1_SET( NULL, len_stringtest1, dst  );\
            pping33_PING33_stringtest1_GET(src, &item_stringtest1 );\
        }\
            size_t pping33_stringtest2_d2 = 0;\
        if( SRC##_stringtest2_get_params( src , &pping33_stringtest2_d2 ) && ( pping33_stringtest2( dst , &pping33_stringtest2_d2 ), true) )\
                for( size_t d0=0; d0 < Pping33_stringtest2_D0 ; d0++)\
        for( size_t d1=0; d1 < Pping33_stringtest2_D1 ; d1++)\
            for( size_t d2=0; d2 < pping33_stringtest2_d2 ; d2++)\
           {\
        const size_t len_stringtest2 = SRC##_stringtest2_item_exist(src , d0, d1, d2);\
        if( len_stringtest2 ){\
            Vping33_stringtest2    item_stringtest2 = qping33_stringtest2_SET( NULL, len_stringtest2, dst , d0, d1, d2 );\
            pping33_PING33_stringtest2_GET(src, &item_stringtest2 , d0, d1, d2);\
        }\
        }\
        const size_t len_stringtest3 = SRC##_stringtest3_item_exist(src );\
        if( len_stringtest3 ){\
            Vping33_stringtest3    item_stringtest3 = pping33_stringtest3_SET( NULL, len_stringtest3, dst  );\
            pping33_PING33_stringtest3_GET(src, &item_stringtest3 );\
        }\
        const size_t len_stringtest4 = SRC##_stringtest4_item_exist(src );\
        if( len_stringtest4 ){\
            Vping33_stringtest4    item_stringtest4 = pping33_stringtest4_SET( NULL, len_stringtest4, dst  );\
            pping33_PING33_stringtest4_GET(src, &item_stringtest4 );\
        }\
    }

/**
*Metrics typically displayed on a HUD for fixed wing aircraft */

typedef Pack VFR_HUD_vfr_hud; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pvfr_hud_VFR_HUD;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pvfr_hud_VFR_HUD *pvfr_hud_VFR_HUD_from(VFR_HUD_vfr_hud *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_throttle_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_airspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_groundspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_heading_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_climb_SET( float * src, <DST> * dst  ){}
*/

#define pvfr_hud_VFR_HUD_PUSH_INTO(DST)\
    static inline void pvfr_hud_VFR_HUD_push_into_##DST ( pvfr_hud_VFR_HUD * src, DST * dst) {\
        DST##_throttle_SET( pvfr_hud_throttle_GET( src  ), dst  );\
        DST##_airspeed_SET( pvfr_hud_airspeed_GET( src  ), dst  );\
        DST##_groundspeed_SET( pvfr_hud_groundspeed_GET( src  ), dst  );\
        DST##_heading_SET( pvfr_hud_heading_GET( src  ), dst  );\
        DST##_alt_SET( pvfr_hud_alt_GET( src  ), dst  );\
        DST##_climb_SET( pvfr_hud_climb_GET( src  ), dst  );\
    }

/**
*GCS */

typedef Pack RALLY_POINT_rally_point; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor prally_point_RALLY_POINT;// data navigator over pack fields data
/**
															* Wrap RALLY_POINT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline prally_point_RALLY_POINT *RALLY_POINT_rally_point_wrap(RALLY_POINT_rally_point *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline RALLY_POINT_rally_point *prally_point_RALLY_POINT_unwrap(prally_point_RALLY_POINT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_land_dir_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_idx_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_count_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lng_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_break_alt_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_RALLY_FLAGS * src, <DST> * dst  ){}
*/

#define prally_point_RALLY_POINT_PUSH_INTO(DST)\
    static inline void prally_point_RALLY_POINT_push_into_##DST ( prally_point_RALLY_POINT * src, DST * dst) {\
        DST##_land_dir_SET( prally_point_land_dir_GET( src  ), dst  );\
        DST##_target_system_SET( prally_point_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( prally_point_target_component_GET( src  ), dst  );\
        DST##_idx_SET( prally_point_idx_GET( src  ), dst  );\
        DST##_count_SET( prally_point_count_GET( src  ), dst  );\
        DST##_lat_SET( prally_point_lat_GET( src  ), dst  );\
        DST##_lng_SET( prally_point_lng_GET( src  ), dst  );\
        DST##_alt_SET( prally_point_alt_GET( src  ), dst  );\
        DST##_break_alt_SET( prally_point_break_alt_GET( src  ), dst  );\
        e_RALLY_FLAGS  item_flags;\
        if( prally_point_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_land_dir_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_idx_GET( <SRC> * src ){}
static inline int8_t <SRC>_count_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lng_GET( <SRC> * src ){}
static inline int16_t <SRC>_alt_GET( <SRC> * src ){}
static inline int16_t <SRC>_break_alt_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_RALLY_FLAGS <SRC>_flags_GET( <SRC> * src ){}
*/

#define prally_point_RALLY_POINT_PULL_FROM(SRC)\
    static inline void prally_point_RALLY_POINT_pull_from_##SRC ( SRC * src, prally_point_RALLY_POINT * dst) {\
        prally_point_land_dir_SET( SRC##_land_dir_GET(src ), dst  );\
        prally_point_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        prally_point_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        prally_point_idx_SET( SRC##_idx_GET(src ), dst  );\
        prally_point_count_SET( SRC##_count_GET(src ), dst  );\
        prally_point_lat_SET( SRC##_lat_GET(src ), dst  );\
        prally_point_lng_SET( SRC##_lng_GET(src ), dst  );\
        prally_point_alt_SET( SRC##_alt_GET(src ), dst  );\
        prally_point_break_alt_SET( SRC##_break_alt_GET(src ), dst  );\
        if( SRC##_flags_item_exist(src ) )\
        prally_point_flags_SET( prally_point_RALLY_POINT_flags_GET( src ),  dst  );\
    }

/**
*Set the mission item with sequence number seq as current item. This means that the MAV will continue to
*				 this mission item on the shortest path (not following the mission items in-between) */

typedef Pack MISSION_SET_CURRENT_mission_set_current; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmission_set_current_MISSION_SET_CURRENT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmission_set_current_MISSION_SET_CURRENT *pmission_set_current_MISSION_SET_CURRENT_from(MISSION_SET_CURRENT_mission_set_current *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define pmission_set_current_MISSION_SET_CURRENT_PUSH_INTO(DST)\
    static inline void pmission_set_current_MISSION_SET_CURRENT_push_into_##DST ( pmission_set_current_MISSION_SET_CURRENT * src, DST * dst) {\
        DST##_seq_SET( pmission_set_current_seq_GET( src  ), dst  );\
        DST##_target_system_SET( pmission_set_current_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_set_current_target_component_GET( src  ), dst  );\
    }

/**
*Adaptive Controller tuning information */

typedef Pack ADAP_TUNING_adap_tuning; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor padap_tuning_ADAP_TUNING;// data navigator over pack fields data
/**
															* Wrap ADAP_TUNING in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline padap_tuning_ADAP_TUNING *ADAP_TUNING_adap_tuning_wrap(ADAP_TUNING_adap_tuning *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline ADAP_TUNING_adap_tuning *padap_tuning_ADAP_TUNING_unwrap(padap_tuning_ADAP_TUNING *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_desired_SET( float * src, <DST> * dst  ){}
static inline void <DST>_achieved_SET( float * src, <DST> * dst  ){}
static inline void <DST>_error_SET( float * src, <DST> * dst  ){}
static inline void <DST>_theta_SET( float * src, <DST> * dst  ){}
static inline void <DST>_omega_SET( float * src, <DST> * dst  ){}
static inline void <DST>_sigma_SET( float * src, <DST> * dst  ){}
static inline void <DST>_theta_dot_SET( float * src, <DST> * dst  ){}
static inline void <DST>_omega_dot_SET( float * src, <DST> * dst  ){}
static inline void <DST>_sigma_dot_SET( float * src, <DST> * dst  ){}
static inline void <DST>_f_SET( float * src, <DST> * dst  ){}
static inline void <DST>_f_dot_SET( float * src, <DST> * dst  ){}
static inline void <DST>_u_SET( float * src, <DST> * dst  ){}
static inline void <DST>_axis_SET( e_PID_TUNING_AXIS * src, <DST> * dst  ){}
*/

#define padap_tuning_ADAP_TUNING_PUSH_INTO(DST)\
    static inline void padap_tuning_ADAP_TUNING_push_into_##DST ( padap_tuning_ADAP_TUNING * src, DST * dst) {\
        DST##_desired_SET( padap_tuning_desired_GET( src  ), dst  );\
        DST##_achieved_SET( padap_tuning_achieved_GET( src  ), dst  );\
        DST##_error_SET( padap_tuning_error_GET( src  ), dst  );\
        DST##_theta_SET( padap_tuning_theta_GET( src  ), dst  );\
        DST##_omega_SET( padap_tuning_omega_GET( src  ), dst  );\
        DST##_sigma_SET( padap_tuning_sigma_GET( src  ), dst  );\
        DST##_theta_dot_SET( padap_tuning_theta_dot_GET( src  ), dst  );\
        DST##_omega_dot_SET( padap_tuning_omega_dot_GET( src  ), dst  );\
        DST##_sigma_dot_SET( padap_tuning_sigma_dot_GET( src  ), dst  );\
        DST##_f_SET( padap_tuning_f_GET( src  ), dst  );\
        DST##_f_dot_SET( padap_tuning_f_dot_GET( src  ), dst  );\
        DST##_u_SET( padap_tuning_u_GET( src  ), dst  );\
        e_PID_TUNING_AXIS  item_axis;\
        if( padap_tuning_axis_GET( src, &item_axis ) ){\
            DST##_axis_SET( item_axis , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_desired_GET( <SRC> * src ){}
static inline float <SRC>_achieved_GET( <SRC> * src ){}
static inline float <SRC>_error_GET( <SRC> * src ){}
static inline float <SRC>_theta_GET( <SRC> * src ){}
static inline float <SRC>_omega_GET( <SRC> * src ){}
static inline float <SRC>_sigma_GET( <SRC> * src ){}
static inline float <SRC>_theta_dot_GET( <SRC> * src ){}
static inline float <SRC>_omega_dot_GET( <SRC> * src ){}
static inline float <SRC>_sigma_dot_GET( <SRC> * src ){}
static inline float <SRC>_f_GET( <SRC> * src ){}
static inline float <SRC>_f_dot_GET( <SRC> * src ){}
static inline float <SRC>_u_GET( <SRC> * src ){}
static inline bool  <SRC>_axis_item_exist( <SRC> * src ){}
static inline e_PID_TUNING_AXIS <SRC>_axis_GET( <SRC> * src ){}
*/

#define padap_tuning_ADAP_TUNING_PULL_FROM(SRC)\
    static inline void padap_tuning_ADAP_TUNING_pull_from_##SRC ( SRC * src, padap_tuning_ADAP_TUNING * dst) {\
        padap_tuning_desired_SET( SRC##_desired_GET(src ), dst  );\
        padap_tuning_achieved_SET( SRC##_achieved_GET(src ), dst  );\
        padap_tuning_error_SET( SRC##_error_GET(src ), dst  );\
        padap_tuning_theta_SET( SRC##_theta_GET(src ), dst  );\
        padap_tuning_omega_SET( SRC##_omega_GET(src ), dst  );\
        padap_tuning_sigma_SET( SRC##_sigma_GET(src ), dst  );\
        padap_tuning_theta_dot_SET( SRC##_theta_dot_GET(src ), dst  );\
        padap_tuning_omega_dot_SET( SRC##_omega_dot_GET(src ), dst  );\
        padap_tuning_sigma_dot_SET( SRC##_sigma_dot_GET(src ), dst  );\
        padap_tuning_f_SET( SRC##_f_GET(src ), dst  );\
        padap_tuning_f_dot_SET( SRC##_f_dot_GET(src ), dst  );\
        padap_tuning_u_SET( SRC##_u_GET(src ), dst  );\
        if( SRC##_axis_item_exist(src ) )\
        padap_tuning_axis_SET( padap_tuning_ADAP_TUNING_axis_GET( src ),  dst  );\
    }

/**
*Vibration levels and accelerometer clipping */

typedef Pack VIBRATION_vibration; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pvibration_VIBRATION;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pvibration_VIBRATION *pvibration_VIBRATION_from(VIBRATION_vibration *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_clipping_0_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_clipping_1_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_clipping_2_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_vibration_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vibration_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vibration_z_SET( float * src, <DST> * dst  ){}
*/

#define pvibration_VIBRATION_PUSH_INTO(DST)\
    static inline void pvibration_VIBRATION_push_into_##DST ( pvibration_VIBRATION * src, DST * dst) {\
        DST##_clipping_0_SET( pvibration_clipping_0_GET( src  ), dst  );\
        DST##_clipping_1_SET( pvibration_clipping_1_GET( src  ), dst  );\
        DST##_clipping_2_SET( pvibration_clipping_2_GET( src  ), dst  );\
        DST##_time_usec_SET( pvibration_time_usec_GET( src  ), dst  );\
        DST##_vibration_x_SET( pvibration_vibration_x_GET( src  ), dst  );\
        DST##_vibration_y_SET( pvibration_vibration_y_GET( src  ), dst  );\
        DST##_vibration_z_SET( pvibration_vibration_z_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_clipping_0_GET( <SRC> * src ){}
static inline int32_t <SRC>_clipping_1_GET( <SRC> * src ){}
static inline int32_t <SRC>_clipping_2_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_vibration_x_GET( <SRC> * src ){}
static inline float <SRC>_vibration_y_GET( <SRC> * src ){}
static inline float <SRC>_vibration_z_GET( <SRC> * src ){}
*/

#define pvibration_VIBRATION_PULL_FROM(SRC)\
    static inline void pvibration_VIBRATION_pull_from_##SRC ( SRC * src, pvibration_VIBRATION * dst) {\
        pvibration_clipping_0_SET( SRC##_clipping_0_GET(src ), dst  );\
        pvibration_clipping_1_SET( SRC##_clipping_1_GET(src ), dst  );\
        pvibration_clipping_2_SET( SRC##_clipping_2_GET(src ), dst  );\
        pvibration_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pvibration_vibration_x_SET( SRC##_vibration_x_GET(src ), dst  );\
        pvibration_vibration_y_SET( SRC##_vibration_y_GET(src ), dst  );\
        pvibration_vibration_z_SET( SRC##_vibration_z_GET(src ), dst  );\
    }

/**
*Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
*				 recipient to keep track of received parameters and allows them to re-request missing parameters after
*				 a loss or timeout */

typedef Pack PARAM_EXT_VALUE_param_ext_value; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pparam_ext_value_PARAM_EXT_VALUE;// data navigator over pack fields data
/**
															* Wrap PARAM_EXT_VALUE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pparam_ext_value_PARAM_EXT_VALUE *PARAM_EXT_VALUE_param_ext_value_wrap(PARAM_EXT_VALUE_param_ext_value *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PARAM_EXT_VALUE_param_ext_value *pparam_ext_value_PARAM_EXT_VALUE_unwrap(pparam_ext_value_PARAM_EXT_VALUE *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_ext_value_param_id;
//Maximum field array length constant
#define Pparam_ext_value_param_id_len_max  ( 255 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_ext_value_param_value;
//Maximum field array length constant
#define Pparam_ext_value_param_value_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_param_count_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_param_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_param_id_SET( Vparam_ext_value_param_id * src, <DST> * dst  ){}
static inline void <DST>_param_value_SET( Vparam_ext_value_param_value * src, <DST> * dst  ){}
static inline void <DST>_param_type_SET( e_MAV_PARAM_EXT_TYPE * src, <DST> * dst  ){}
*/

#define pparam_ext_value_PARAM_EXT_VALUE_PUSH_INTO(DST)\
    static inline void pparam_ext_value_PARAM_EXT_VALUE_push_into_##DST ( pparam_ext_value_PARAM_EXT_VALUE * src, DST * dst) {\
        DST##_param_count_SET( pparam_ext_value_param_count_GET( src  ), dst  );\
        DST##_param_index_SET( pparam_ext_value_param_index_GET( src  ), dst  );\
        Vparam_ext_value_param_id  item_param_id;\
        if( pparam_ext_value_param_id_GET( src, &item_param_id ) ){\
            DST##_param_id_SET( &item_param_id, dst );\
        }\
        Vparam_ext_value_param_value  item_param_value;\
        if( pparam_ext_value_param_value_GET( src, &item_param_value ) ){\
            DST##_param_value_SET( &item_param_value, dst );\
        }\
        e_MAV_PARAM_EXT_TYPE  item_param_type;\
        if( pparam_ext_value_param_type_GET( src, &item_param_type ) ){\
            DST##_param_type_SET( item_param_type , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_param_count_GET( <SRC> * src ){}
static inline int16_t <SRC>_param_index_GET( <SRC> * src ){}
static inline size_t  <SRC>_param_id_item_exist( <SRC> * src  ){}
static inline void <SRC>_param_id_GET( <SRC> * src, Vparam_ext_value_param_id * dst ){}
static inline size_t  <SRC>_param_value_item_exist( <SRC> * src  ){}
static inline void <SRC>_param_value_GET( <SRC> * src, Vparam_ext_value_param_value * dst ){}
static inline bool  <SRC>_param_type_item_exist( <SRC> * src ){}
static inline e_MAV_PARAM_EXT_TYPE <SRC>_param_type_GET( <SRC> * src ){}
*/

#define pparam_ext_value_PARAM_EXT_VALUE_PULL_FROM(SRC)\
    static inline void pparam_ext_value_PARAM_EXT_VALUE_pull_from_##SRC ( SRC * src, pparam_ext_value_PARAM_EXT_VALUE * dst) {\
        pparam_ext_value_param_count_SET( SRC##_param_count_GET(src ), dst  );\
        pparam_ext_value_param_index_SET( SRC##_param_index_GET(src ), dst  );\
        const size_t len_param_id = SRC##_param_id_item_exist(src );\
        if( len_param_id ){\
            Vparam_ext_value_param_id    item_param_id = pparam_ext_value_param_id_SET( NULL, len_param_id, dst  );\
            pparam_ext_value_PARAM_EXT_VALUE_param_id_GET(src, &item_param_id );\
        }\
        const size_t len_param_value = SRC##_param_value_item_exist(src );\
        if( len_param_value ){\
            Vparam_ext_value_param_value    item_param_value = pparam_ext_value_param_value_SET( NULL, len_param_value, dst  );\
            pparam_ext_value_PARAM_EXT_VALUE_param_value_GET(src, &item_param_value );\
        }\
        if( SRC##_param_type_item_exist(src ) )\
        pparam_ext_value_param_type_SET( pparam_ext_value_PARAM_EXT_VALUE_param_type_GET( src ),  dst  );\
    }

/**
*2nd Battery status */

typedef Pack BATTERY2_battery2; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pbattery2_BATTERY2;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pbattery2_BATTERY2 *pbattery2_BATTERY2_from(BATTERY2_battery2 *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_voltage_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_current_battery_SET( int16_t * src, <DST> * dst  ){}
*/

#define pbattery2_BATTERY2_PUSH_INTO(DST)\
    static inline void pbattery2_BATTERY2_push_into_##DST ( pbattery2_BATTERY2 * src, DST * dst) {\
        DST##_voltage_SET( pbattery2_voltage_GET( src  ), dst  );\
        DST##_current_battery_SET( pbattery2_current_battery_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_voltage_GET( <SRC> * src ){}
static inline int16_t <SRC>_current_battery_GET( <SRC> * src ){}
*/

#define pbattery2_BATTERY2_PULL_FROM(SRC)\
    static inline void pbattery2_BATTERY2_pull_from_##SRC ( SRC * src, pbattery2_BATTERY2 * dst) {\
        pbattery2_voltage_SET( SRC##_voltage_GET(src ), dst  );\
        pbattery2_current_battery_SET( SRC##_current_battery_GET(src ), dst  );\
    }

/**
*Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled */

typedef Pack LIMITS_STATUS_limits_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor plimits_status_LIMITS_STATUS;// data navigator over pack fields data
/**
															* Wrap LIMITS_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline plimits_status_LIMITS_STATUS *LIMITS_STATUS_limits_status_wrap(LIMITS_STATUS_limits_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline LIMITS_STATUS_limits_status *plimits_status_LIMITS_STATUS_unwrap(plimits_status_LIMITS_STATUS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_breach_count_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_last_trigger_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_last_action_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_last_recovery_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_last_clear_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_limits_state_SET( e_LIMITS_STATE * src, <DST> * dst  ){}
static inline void <DST>_mods_enabled_SET( e_LIMIT_MODULE * src, <DST> * dst  ){}
static inline void <DST>_mods_required_SET( e_LIMIT_MODULE * src, <DST> * dst  ){}
static inline void <DST>_mods_triggered_SET( e_LIMIT_MODULE * src, <DST> * dst  ){}
*/

#define plimits_status_LIMITS_STATUS_PUSH_INTO(DST)\
    static inline void plimits_status_LIMITS_STATUS_push_into_##DST ( plimits_status_LIMITS_STATUS * src, DST * dst) {\
        DST##_breach_count_SET( plimits_status_breach_count_GET( src  ), dst  );\
        DST##_last_trigger_SET( plimits_status_last_trigger_GET( src  ), dst  );\
        DST##_last_action_SET( plimits_status_last_action_GET( src  ), dst  );\
        DST##_last_recovery_SET( plimits_status_last_recovery_GET( src  ), dst  );\
        DST##_last_clear_SET( plimits_status_last_clear_GET( src  ), dst  );\
        e_LIMITS_STATE  item_limits_state;\
        if( plimits_status_limits_state_GET( src, &item_limits_state ) ){\
            DST##_limits_state_SET( item_limits_state , dst  );\
        }\
        e_LIMIT_MODULE  item_mods_enabled;\
        if( plimits_status_mods_enabled_GET( src, &item_mods_enabled ) ){\
            DST##_mods_enabled_SET( item_mods_enabled , dst  );\
        }\
        e_LIMIT_MODULE  item_mods_required;\
        if( plimits_status_mods_required_GET( src, &item_mods_required ) ){\
            DST##_mods_required_SET( item_mods_required , dst  );\
        }\
        e_LIMIT_MODULE  item_mods_triggered;\
        if( plimits_status_mods_triggered_GET( src, &item_mods_triggered ) ){\
            DST##_mods_triggered_SET( item_mods_triggered , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_breach_count_GET( <SRC> * src ){}
static inline int32_t <SRC>_last_trigger_GET( <SRC> * src ){}
static inline int32_t <SRC>_last_action_GET( <SRC> * src ){}
static inline int32_t <SRC>_last_recovery_GET( <SRC> * src ){}
static inline int32_t <SRC>_last_clear_GET( <SRC> * src ){}
static inline bool  <SRC>_limits_state_item_exist( <SRC> * src ){}
static inline e_LIMITS_STATE <SRC>_limits_state_GET( <SRC> * src ){}
static inline bool  <SRC>_mods_enabled_item_exist( <SRC> * src ){}
static inline e_LIMIT_MODULE <SRC>_mods_enabled_GET( <SRC> * src ){}
static inline bool  <SRC>_mods_required_item_exist( <SRC> * src ){}
static inline e_LIMIT_MODULE <SRC>_mods_required_GET( <SRC> * src ){}
static inline bool  <SRC>_mods_triggered_item_exist( <SRC> * src ){}
static inline e_LIMIT_MODULE <SRC>_mods_triggered_GET( <SRC> * src ){}
*/

#define plimits_status_LIMITS_STATUS_PULL_FROM(SRC)\
    static inline void plimits_status_LIMITS_STATUS_pull_from_##SRC ( SRC * src, plimits_status_LIMITS_STATUS * dst) {\
        plimits_status_breach_count_SET( SRC##_breach_count_GET(src ), dst  );\
        plimits_status_last_trigger_SET( SRC##_last_trigger_GET(src ), dst  );\
        plimits_status_last_action_SET( SRC##_last_action_GET(src ), dst  );\
        plimits_status_last_recovery_SET( SRC##_last_recovery_GET(src ), dst  );\
        plimits_status_last_clear_SET( SRC##_last_clear_GET(src ), dst  );\
        if( SRC##_limits_state_item_exist(src ) )\
        plimits_status_limits_state_SET( plimits_status_LIMITS_STATUS_limits_state_GET( src ),  dst  );\
        if( SRC##_mods_enabled_item_exist(src ) )\
        plimits_status_mods_enabled_SET( plimits_status_LIMITS_STATUS_mods_enabled_GET( src ),  dst  );\
        if( SRC##_mods_required_item_exist(src ) )\
        plimits_status_mods_required_SET( plimits_status_LIMITS_STATUS_mods_required_GET( src ),  dst  );\
        if( SRC##_mods_triggered_item_exist(src ) )\
        plimits_status_mods_triggered_SET( plimits_status_LIMITS_STATUS_mods_triggered_GET( src ),  dst  );\
    }

/**
*Camera Capture Feedback */

typedef Pack CAMERA_FEEDBACK_camera_feedback; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcamera_feedback_CAMERA_FEEDBACK;// data navigator over pack fields data
/**
															* Wrap CAMERA_FEEDBACK in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcamera_feedback_CAMERA_FEEDBACK *CAMERA_FEEDBACK_camera_feedback_wrap(CAMERA_FEEDBACK_camera_feedback *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline CAMERA_FEEDBACK_camera_feedback *pcamera_feedback_CAMERA_FEEDBACK_unwrap(pcamera_feedback_CAMERA_FEEDBACK *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_img_idx_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_cam_idx_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lng_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_msl_SET( float * src, <DST> * dst  ){}
static inline void <DST>_alt_rel_SET( float * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_foc_len_SET( float * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_CAMERA_FEEDBACK_FLAGS * src, <DST> * dst  ){}
*/

#define pcamera_feedback_CAMERA_FEEDBACK_PUSH_INTO(DST)\
    static inline void pcamera_feedback_CAMERA_FEEDBACK_push_into_##DST ( pcamera_feedback_CAMERA_FEEDBACK * src, DST * dst) {\
        DST##_img_idx_SET( pcamera_feedback_img_idx_GET( src  ), dst  );\
        DST##_time_usec_SET( pcamera_feedback_time_usec_GET( src  ), dst  );\
        DST##_target_system_SET( pcamera_feedback_target_system_GET( src  ), dst  );\
        DST##_cam_idx_SET( pcamera_feedback_cam_idx_GET( src  ), dst  );\
        DST##_lat_SET( pcamera_feedback_lat_GET( src  ), dst  );\
        DST##_lng_SET( pcamera_feedback_lng_GET( src  ), dst  );\
        DST##_alt_msl_SET( pcamera_feedback_alt_msl_GET( src  ), dst  );\
        DST##_alt_rel_SET( pcamera_feedback_alt_rel_GET( src  ), dst  );\
        DST##_roll_SET( pcamera_feedback_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pcamera_feedback_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pcamera_feedback_yaw_GET( src  ), dst  );\
        DST##_foc_len_SET( pcamera_feedback_foc_len_GET( src  ), dst  );\
        e_CAMERA_FEEDBACK_FLAGS  item_flags;\
        if( pcamera_feedback_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_img_idx_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_cam_idx_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lng_GET( <SRC> * src ){}
static inline float <SRC>_alt_msl_GET( <SRC> * src ){}
static inline float <SRC>_alt_rel_GET( <SRC> * src ){}
static inline float <SRC>_roll_GET( <SRC> * src ){}
static inline float <SRC>_pitch_GET( <SRC> * src ){}
static inline float <SRC>_yaw_GET( <SRC> * src ){}
static inline float <SRC>_foc_len_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_CAMERA_FEEDBACK_FLAGS <SRC>_flags_GET( <SRC> * src ){}
*/

#define pcamera_feedback_CAMERA_FEEDBACK_PULL_FROM(SRC)\
    static inline void pcamera_feedback_CAMERA_FEEDBACK_pull_from_##SRC ( SRC * src, pcamera_feedback_CAMERA_FEEDBACK * dst) {\
        pcamera_feedback_img_idx_SET( SRC##_img_idx_GET(src ), dst  );\
        pcamera_feedback_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pcamera_feedback_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pcamera_feedback_cam_idx_SET( SRC##_cam_idx_GET(src ), dst  );\
        pcamera_feedback_lat_SET( SRC##_lat_GET(src ), dst  );\
        pcamera_feedback_lng_SET( SRC##_lng_GET(src ), dst  );\
        pcamera_feedback_alt_msl_SET( SRC##_alt_msl_GET(src ), dst  );\
        pcamera_feedback_alt_rel_SET( SRC##_alt_rel_GET(src ), dst  );\
        pcamera_feedback_roll_SET( SRC##_roll_GET(src ), dst  );\
        pcamera_feedback_pitch_SET( SRC##_pitch_GET(src ), dst  );\
        pcamera_feedback_yaw_SET( SRC##_yaw_GET(src ), dst  );\
        pcamera_feedback_foc_len_SET( SRC##_foc_len_GET(src ), dst  );\
        if( SRC##_flags_item_exist(src ) )\
        pcamera_feedback_flags_SET( pcamera_feedback_CAMERA_FEEDBACK_flags_GET( src ),  dst  );\
    }

/**
*The global position, as returned by the Global Positioning System (GPS). This is
*				 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame). */

typedef Pack HIL_GPS_hil_gps; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t phil_gps_HIL_GPS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline phil_gps_HIL_GPS *phil_gps_HIL_GPS_from(HIL_GPS_hil_gps *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_eph_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_epv_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vel_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_cog_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_fix_type_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_vn_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ve_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vd_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_satellites_visible_SET( int8_t * src, <DST> * dst  ){}
*/

#define phil_gps_HIL_GPS_PUSH_INTO(DST)\
    static inline void phil_gps_HIL_GPS_push_into_##DST ( phil_gps_HIL_GPS * src, DST * dst) {\
        DST##_eph_SET( phil_gps_eph_GET( src  ), dst  );\
        DST##_epv_SET( phil_gps_epv_GET( src  ), dst  );\
        DST##_vel_SET( phil_gps_vel_GET( src  ), dst  );\
        DST##_cog_SET( phil_gps_cog_GET( src  ), dst  );\
        DST##_time_usec_SET( phil_gps_time_usec_GET( src  ), dst  );\
        DST##_fix_type_SET( phil_gps_fix_type_GET( src  ), dst  );\
        DST##_lat_SET( phil_gps_lat_GET( src  ), dst  );\
        DST##_lon_SET( phil_gps_lon_GET( src  ), dst  );\
        DST##_alt_SET( phil_gps_alt_GET( src  ), dst  );\
        DST##_vn_SET( phil_gps_vn_GET( src  ), dst  );\
        DST##_ve_SET( phil_gps_ve_GET( src  ), dst  );\
        DST##_vd_SET( phil_gps_vd_GET( src  ), dst  );\
        DST##_satellites_visible_SET( phil_gps_satellites_visible_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_eph_GET( <SRC> * src ){}
static inline int16_t <SRC>_epv_GET( <SRC> * src ){}
static inline int16_t <SRC>_vel_GET( <SRC> * src ){}
static inline int16_t <SRC>_cog_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_fix_type_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline int32_t <SRC>_alt_GET( <SRC> * src ){}
static inline int16_t <SRC>_vn_GET( <SRC> * src ){}
static inline int16_t <SRC>_ve_GET( <SRC> * src ){}
static inline int16_t <SRC>_vd_GET( <SRC> * src ){}
static inline int8_t <SRC>_satellites_visible_GET( <SRC> * src ){}
*/

#define phil_gps_HIL_GPS_PULL_FROM(SRC)\
    static inline void phil_gps_HIL_GPS_pull_from_##SRC ( SRC * src, phil_gps_HIL_GPS * dst) {\
        phil_gps_eph_SET( SRC##_eph_GET(src ), dst  );\
        phil_gps_epv_SET( SRC##_epv_GET(src ), dst  );\
        phil_gps_vel_SET( SRC##_vel_GET(src ), dst  );\
        phil_gps_cog_SET( SRC##_cog_GET(src ), dst  );\
        phil_gps_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        phil_gps_fix_type_SET( SRC##_fix_type_GET(src ), dst  );\
        phil_gps_lat_SET( SRC##_lat_GET(src ), dst  );\
        phil_gps_lon_SET( SRC##_lon_GET(src ), dst  );\
        phil_gps_alt_SET( SRC##_alt_GET(src ), dst  );\
        phil_gps_vn_SET( SRC##_vn_GET(src ), dst  );\
        phil_gps_ve_SET( SRC##_ve_GET(src ), dst  );\
        phil_gps_vd_SET( SRC##_vd_GET(src ), dst  );\
        phil_gps_satellites_visible_SET( SRC##_satellites_visible_GET(src ), dst  );\
    }

/**
*The state of the fixed wing navigation and position controller. */

typedef Pack NAV_CONTROLLER_OUTPUT_nav_controller_output; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pnav_controller_output_NAV_CONTROLLER_OUTPUT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pnav_controller_output_NAV_CONTROLLER_OUTPUT *pnav_controller_output_NAV_CONTROLLER_OUTPUT_from(NAV_CONTROLLER_OUTPUT_nav_controller_output *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_wp_dist_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_nav_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_nav_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_nav_bearing_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_bearing_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_alt_error_SET( float * src, <DST> * dst  ){}
static inline void <DST>_aspd_error_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xtrack_error_SET( float * src, <DST> * dst  ){}
*/

#define pnav_controller_output_NAV_CONTROLLER_OUTPUT_PUSH_INTO(DST)\
    static inline void pnav_controller_output_NAV_CONTROLLER_OUTPUT_push_into_##DST ( pnav_controller_output_NAV_CONTROLLER_OUTPUT * src, DST * dst) {\
        DST##_wp_dist_SET( pnav_controller_output_wp_dist_GET( src  ), dst  );\
        DST##_nav_roll_SET( pnav_controller_output_nav_roll_GET( src  ), dst  );\
        DST##_nav_pitch_SET( pnav_controller_output_nav_pitch_GET( src  ), dst  );\
        DST##_nav_bearing_SET( pnav_controller_output_nav_bearing_GET( src  ), dst  );\
        DST##_target_bearing_SET( pnav_controller_output_target_bearing_GET( src  ), dst  );\
        DST##_alt_error_SET( pnav_controller_output_alt_error_GET( src  ), dst  );\
        DST##_aspd_error_SET( pnav_controller_output_aspd_error_GET( src  ), dst  );\
        DST##_xtrack_error_SET( pnav_controller_output_xtrack_error_GET( src  ), dst  );\
    }

/**
*Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
*				 so transmitting the key requires an encrypted channel for true safety */

typedef Pack AUTH_KEY_auth_key; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pauth_key_AUTH_KEY;// data navigator over pack fields data
/**
															* Wrap AUTH_KEY in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pauth_key_AUTH_KEY *AUTH_KEY_auth_key_wrap(AUTH_KEY_auth_key *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline AUTH_KEY_auth_key *pauth_key_AUTH_KEY_unwrap(pauth_key_AUTH_KEY *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vauth_key_key;
//Maximum field array length constant
#define Pauth_key_key_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_key_SET( Vauth_key_key * src, <DST> * dst  ){}
*/

#define pauth_key_AUTH_KEY_PUSH_INTO(DST)\
    static inline void pauth_key_AUTH_KEY_push_into_##DST ( pauth_key_AUTH_KEY * src, DST * dst) {\
        Vauth_key_key  item_key;\
        if( pauth_key_key_GET( src, &item_key ) ){\
            DST##_key_SET( &item_key, dst );\
        }\
    }

/**
*Request a current fence point from MAV */

typedef Pack FENCE_FETCH_POINT_fence_fetch_point; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pfence_fetch_point_FENCE_FETCH_POINT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pfence_fetch_point_FENCE_FETCH_POINT *pfence_fetch_point_FENCE_FETCH_POINT_from(FENCE_FETCH_POINT_fence_fetch_point *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_idx_SET( int8_t * src, <DST> * dst  ){}
*/

#define pfence_fetch_point_FENCE_FETCH_POINT_PUSH_INTO(DST)\
    static inline void pfence_fetch_point_FENCE_FETCH_POINT_push_into_##DST ( pfence_fetch_point_FENCE_FETCH_POINT * src, DST * dst) {\
        DST##_target_system_SET( pfence_fetch_point_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pfence_fetch_point_target_component_GET( src  ), dst  );\
        DST##_idx_SET( pfence_fetch_point_idx_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_idx_GET( <SRC> * src ){}
*/

#define pfence_fetch_point_FENCE_FETCH_POINT_PULL_FROM(SRC)\
    static inline void pfence_fetch_point_FENCE_FETCH_POINT_pull_from_##SRC ( SRC * src, pfence_fetch_point_FENCE_FETCH_POINT * dst) {\
        pfence_fetch_point_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pfence_fetch_point_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pfence_fetch_point_idx_SET( SRC##_idx_GET(src ), dst  );\
    }

/**
*Status generated by radio */

typedef Pack RADIO_radio; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pradio_RADIO;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pradio_RADIO *pradio_RADIO_from(RADIO_radio *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_rxerrors_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_fixeD_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_rssi_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_remrssi_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_txbuf_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_noise_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_remnoise_SET( int8_t * src, <DST> * dst  ){}
*/

#define pradio_RADIO_PUSH_INTO(DST)\
    static inline void pradio_RADIO_push_into_##DST ( pradio_RADIO * src, DST * dst) {\
        DST##_rxerrors_SET( pradio_rxerrors_GET( src  ), dst  );\
        DST##_fixeD_SET( pradio_fixeD_GET( src  ), dst  );\
        DST##_rssi_SET( pradio_rssi_GET( src  ), dst  );\
        DST##_remrssi_SET( pradio_remrssi_GET( src  ), dst  );\
        DST##_txbuf_SET( pradio_txbuf_GET( src  ), dst  );\
        DST##_noise_SET( pradio_noise_GET( src  ), dst  );\
        DST##_remnoise_SET( pradio_remnoise_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_rxerrors_GET( <SRC> * src ){}
static inline int16_t <SRC>_fixeD_GET( <SRC> * src ){}
static inline int8_t <SRC>_rssi_GET( <SRC> * src ){}
static inline int8_t <SRC>_remrssi_GET( <SRC> * src ){}
static inline int8_t <SRC>_txbuf_GET( <SRC> * src ){}
static inline int8_t <SRC>_noise_GET( <SRC> * src ){}
static inline int8_t <SRC>_remnoise_GET( <SRC> * src ){}
*/

#define pradio_RADIO_PULL_FROM(SRC)\
    static inline void pradio_RADIO_pull_from_##SRC ( SRC * src, pradio_RADIO * dst) {\
        pradio_rxerrors_SET( SRC##_rxerrors_GET(src ), dst  );\
        pradio_fixeD_SET( SRC##_fixeD_GET(src ), dst  );\
        pradio_rssi_SET( SRC##_rssi_GET(src ), dst  );\
        pradio_remrssi_SET( SRC##_remrssi_GET(src ), dst  );\
        pradio_txbuf_SET( SRC##_txbuf_GET(src ), dst  );\
        pradio_noise_SET( SRC##_noise_GET(src ), dst  );\
        pradio_remnoise_SET( SRC##_remnoise_GET(src ), dst  );\
    }

/**
*The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
*				 Z-axis down (aeronautical frame, NED / north-east-down convention */

typedef Pack LOCAL_POSITION_NED_COV_local_position_ned_cov; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor plocal_position_ned_cov_LOCAL_POSITION_NED_COV;// data navigator over pack fields data
/**
															* Wrap LOCAL_POSITION_NED_COV in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline plocal_position_ned_cov_LOCAL_POSITION_NED_COV *LOCAL_POSITION_NED_COV_local_position_ned_cov_wrap(LOCAL_POSITION_NED_COV_local_position_ned_cov *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline LOCAL_POSITION_NED_COV_local_position_ned_cov *plocal_position_ned_cov_LOCAL_POSITION_NED_COV_unwrap(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vlocal_position_ned_cov_covariance;
//Maximum field array length constant
#define Plocal_position_ned_cov_covariance_len  ( 45 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ax_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ay_SET( float * src, <DST> * dst  ){}
static inline void <DST>_az_SET( float * src, <DST> * dst  ){}
static inline void <DST>_covariance_SET( Vlocal_position_ned_cov_covariance * src, <DST> * dst  ){}
static inline void <DST>_estimator_type_SET( e_MAV_ESTIMATOR_TYPE * src, <DST> * dst  ){}
*/

#define plocal_position_ned_cov_LOCAL_POSITION_NED_COV_PUSH_INTO(DST)\
    static inline void plocal_position_ned_cov_LOCAL_POSITION_NED_COV_push_into_##DST ( plocal_position_ned_cov_LOCAL_POSITION_NED_COV * src, DST * dst) {\
        DST##_time_usec_SET( plocal_position_ned_cov_time_usec_GET( src  ), dst  );\
        DST##_x_SET( plocal_position_ned_cov_x_GET( src  ), dst  );\
        DST##_y_SET( plocal_position_ned_cov_y_GET( src  ), dst  );\
        DST##_z_SET( plocal_position_ned_cov_z_GET( src  ), dst  );\
        DST##_vx_SET( plocal_position_ned_cov_vx_GET( src  ), dst  );\
        DST##_vy_SET( plocal_position_ned_cov_vy_GET( src  ), dst  );\
        DST##_vz_SET( plocal_position_ned_cov_vz_GET( src  ), dst  );\
        DST##_ax_SET( plocal_position_ned_cov_ax_GET( src  ), dst  );\
        DST##_ay_SET( plocal_position_ned_cov_ay_GET( src  ), dst  );\
        DST##_az_SET( plocal_position_ned_cov_az_GET( src  ), dst  );\
        Vlocal_position_ned_cov_covariance item_covariance = plocal_position_ned_cov_covariance_GET( src  );\
       DST##_covariance_SET( &item_covariance, dst );\
        e_MAV_ESTIMATOR_TYPE  item_estimator_type;\
        if( plocal_position_ned_cov_estimator_type_GET( src, &item_estimator_type ) ){\
            DST##_estimator_type_SET( item_estimator_type , dst  );\
        }\
    }

/**
*Airspeed auto-calibration */

typedef Pack AIRSPEED_AUTOCAL_airspeed_autocal; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pairspeed_autocal_AIRSPEED_AUTOCAL;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pairspeed_autocal_AIRSPEED_AUTOCAL *pairspeed_autocal_AIRSPEED_AUTOCAL_from(AIRSPEED_AUTOCAL_airspeed_autocal *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_vx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_diff_pressure_SET( float * src, <DST> * dst  ){}
static inline void <DST>_EAS2TAS_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ratio_SET( float * src, <DST> * dst  ){}
static inline void <DST>_state_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_state_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_state_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_Pax_SET( float * src, <DST> * dst  ){}
static inline void <DST>_Pby_SET( float * src, <DST> * dst  ){}
static inline void <DST>_Pcz_SET( float * src, <DST> * dst  ){}
*/

#define pairspeed_autocal_AIRSPEED_AUTOCAL_PUSH_INTO(DST)\
    static inline void pairspeed_autocal_AIRSPEED_AUTOCAL_push_into_##DST ( pairspeed_autocal_AIRSPEED_AUTOCAL * src, DST * dst) {\
        DST##_vx_SET( pairspeed_autocal_vx_GET( src  ), dst  );\
        DST##_vy_SET( pairspeed_autocal_vy_GET( src  ), dst  );\
        DST##_vz_SET( pairspeed_autocal_vz_GET( src  ), dst  );\
        DST##_diff_pressure_SET( pairspeed_autocal_diff_pressure_GET( src  ), dst  );\
        DST##_EAS2TAS_SET( pairspeed_autocal_EAS2TAS_GET( src  ), dst  );\
        DST##_ratio_SET( pairspeed_autocal_ratio_GET( src  ), dst  );\
        DST##_state_x_SET( pairspeed_autocal_state_x_GET( src  ), dst  );\
        DST##_state_y_SET( pairspeed_autocal_state_y_GET( src  ), dst  );\
        DST##_state_z_SET( pairspeed_autocal_state_z_GET( src  ), dst  );\
        DST##_Pax_SET( pairspeed_autocal_Pax_GET( src  ), dst  );\
        DST##_Pby_SET( pairspeed_autocal_Pby_GET( src  ), dst  );\
        DST##_Pcz_SET( pairspeed_autocal_Pcz_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_vx_GET( <SRC> * src ){}
static inline float <SRC>_vy_GET( <SRC> * src ){}
static inline float <SRC>_vz_GET( <SRC> * src ){}
static inline float <SRC>_diff_pressure_GET( <SRC> * src ){}
static inline float <SRC>_EAS2TAS_GET( <SRC> * src ){}
static inline float <SRC>_ratio_GET( <SRC> * src ){}
static inline float <SRC>_state_x_GET( <SRC> * src ){}
static inline float <SRC>_state_y_GET( <SRC> * src ){}
static inline float <SRC>_state_z_GET( <SRC> * src ){}
static inline float <SRC>_Pax_GET( <SRC> * src ){}
static inline float <SRC>_Pby_GET( <SRC> * src ){}
static inline float <SRC>_Pcz_GET( <SRC> * src ){}
*/

#define pairspeed_autocal_AIRSPEED_AUTOCAL_PULL_FROM(SRC)\
    static inline void pairspeed_autocal_AIRSPEED_AUTOCAL_pull_from_##SRC ( SRC * src, pairspeed_autocal_AIRSPEED_AUTOCAL * dst) {\
        pairspeed_autocal_vx_SET( SRC##_vx_GET(src ), dst  );\
        pairspeed_autocal_vy_SET( SRC##_vy_GET(src ), dst  );\
        pairspeed_autocal_vz_SET( SRC##_vz_GET(src ), dst  );\
        pairspeed_autocal_diff_pressure_SET( SRC##_diff_pressure_GET(src ), dst  );\
        pairspeed_autocal_EAS2TAS_SET( SRC##_EAS2TAS_GET(src ), dst  );\
        pairspeed_autocal_ratio_SET( SRC##_ratio_GET(src ), dst  );\
        pairspeed_autocal_state_x_SET( SRC##_state_x_GET(src ), dst  );\
        pairspeed_autocal_state_y_SET( SRC##_state_y_GET(src ), dst  );\
        pairspeed_autocal_state_z_SET( SRC##_state_z_GET(src ), dst  );\
        pairspeed_autocal_Pax_SET( SRC##_Pax_GET(src ), dst  );\
        pairspeed_autocal_Pby_SET( SRC##_Pby_GET(src ), dst  );\
        pairspeed_autocal_Pcz_SET( SRC##_Pcz_GET(src ), dst  );\
    }

/**
*Motion capture attitude and position */

typedef Pack ATT_POS_MOCAP_att_pos_mocap; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t patt_pos_mocap_ATT_POS_MOCAP;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline patt_pos_mocap_ATT_POS_MOCAP *patt_pos_mocap_ATT_POS_MOCAP_from(ATT_POS_MOCAP_att_pos_mocap *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vatt_pos_mocap_q;
//Maximum field array length constant
#define Patt_pos_mocap_q_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_q_SET( Vatt_pos_mocap_q * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
*/

#define patt_pos_mocap_ATT_POS_MOCAP_PUSH_INTO(DST)\
    static inline void patt_pos_mocap_ATT_POS_MOCAP_push_into_##DST ( patt_pos_mocap_ATT_POS_MOCAP * src, DST * dst) {\
        DST##_time_usec_SET( patt_pos_mocap_time_usec_GET( src  ), dst  );\
        Vatt_pos_mocap_q item_q = patt_pos_mocap_q_GET( src  );\
       DST##_q_SET( &item_q, dst );\
        DST##_x_SET( patt_pos_mocap_x_GET( src  ), dst  );\
        DST##_y_SET( patt_pos_mocap_y_GET( src  ), dst  );\
        DST##_z_SET( patt_pos_mocap_z_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_q_GET( <SRC> * src, Vatt_pos_mocap_q * dst ){}
static inline float <SRC>_x_GET( <SRC> * src ){}
static inline float <SRC>_y_GET( <SRC> * src ){}
static inline float <SRC>_z_GET( <SRC> * src ){}
*/

#define patt_pos_mocap_ATT_POS_MOCAP_PULL_FROM(SRC)\
    static inline void patt_pos_mocap_ATT_POS_MOCAP_pull_from_##SRC ( SRC * src, patt_pos_mocap_ATT_POS_MOCAP * dst) {\
        patt_pos_mocap_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
       Vatt_pos_mocap_q item_q = patt_pos_mocap_q_SET( NULL, dst  );\
       SRC##_q_GET( src, &item_q );\
        patt_pos_mocap_x_SET( SRC##_x_GET(src ), dst  );\
        patt_pos_mocap_y_SET( SRC##_y_GET(src ), dst  );\
        patt_pos_mocap_z_SET( SRC##_z_GET(src ), dst  );\
    }

/**
*Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
*				 They consume quite some bandwidth, so use only for important status and error messages. If implemented
*				 wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz) */

typedef Pack STATUSTEXT_statustext; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pstatustext_STATUSTEXT;// data navigator over pack fields data
/**
															* Wrap STATUSTEXT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pstatustext_STATUSTEXT *STATUSTEXT_statustext_wrap(STATUSTEXT_statustext *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline STATUSTEXT_statustext *pstatustext_STATUSTEXT_unwrap(pstatustext_STATUSTEXT *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vstatustext_text;
//Maximum field array length constant
#define Pstatustext_text_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_severity_SET( e_MAV_SEVERITY * src, <DST> * dst  ){}
static inline void <DST>_text_SET( Vstatustext_text * src, <DST> * dst  ){}
*/

#define pstatustext_STATUSTEXT_PUSH_INTO(DST)\
    static inline void pstatustext_STATUSTEXT_push_into_##DST ( pstatustext_STATUSTEXT * src, DST * dst) {\
        e_MAV_SEVERITY  item_severity;\
        if( pstatustext_severity_GET( src, &item_severity ) ){\
            DST##_severity_SET( item_severity , dst  );\
        }\
        Vstatustext_text  item_text;\
        if( pstatustext_text_GET( src, &item_text ) ){\
            DST##_text_SET( &item_text, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline bool  <SRC>_severity_item_exist( <SRC> * src ){}
static inline e_MAV_SEVERITY <SRC>_severity_GET( <SRC> * src ){}
static inline size_t  <SRC>_text_item_exist( <SRC> * src  ){}
static inline void <SRC>_text_GET( <SRC> * src, Vstatustext_text * dst ){}
*/

#define pstatustext_STATUSTEXT_PULL_FROM(SRC)\
    static inline void pstatustext_STATUSTEXT_pull_from_##SRC ( SRC * src, pstatustext_STATUSTEXT * dst) {\
        if( SRC##_severity_item_exist(src ) )\
        pstatustext_severity_SET( pstatustext_STATUSTEXT_severity_GET( src ),  dst  );\
        const size_t len_text = SRC##_text_item_exist(src );\
        if( len_text ){\
            Vstatustext_text    item_text = pstatustext_text_SET( NULL, len_text, dst  );\
            pstatustext_STATUSTEXT_text_GET(src, &item_text );\
        }\
    }

/**
*A ping message either requesting or responding to a ping. This allows to measure the system latencies,
*				 including serial port, radio modem and UDP connections */

typedef Pack PING_ping; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pping_PING;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pping_PING *pping_PING_from(PING_ping *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define pping_PING_PUSH_INTO(DST)\
    static inline void pping_PING_push_into_##DST ( pping_PING * src, DST * dst) {\
        DST##_seq_SET( pping_seq_GET( src  ), dst  );\
        DST##_time_usec_SET( pping_time_usec_GET( src  ), dst  );\
        DST##_target_system_SET( pping_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pping_target_component_GET( src  ), dst  );\
    }

/**
*Request a GOPRO_COMMAND response from the GoPro */

typedef Pack GOPRO_GET_REQUEST_gopro_get_request; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgopro_get_request_GOPRO_GET_REQUEST;// data navigator over pack fields data
/**
															* Wrap GOPRO_GET_REQUEST in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgopro_get_request_GOPRO_GET_REQUEST *GOPRO_GET_REQUEST_gopro_get_request_wrap(GOPRO_GET_REQUEST_gopro_get_request *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GOPRO_GET_REQUEST_gopro_get_request *pgopro_get_request_GOPRO_GET_REQUEST_unwrap(pgopro_get_request_GOPRO_GET_REQUEST *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_cmd_id_SET( e_GOPRO_COMMAND * src, <DST> * dst  ){}
*/

#define pgopro_get_request_GOPRO_GET_REQUEST_PUSH_INTO(DST)\
    static inline void pgopro_get_request_GOPRO_GET_REQUEST_push_into_##DST ( pgopro_get_request_GOPRO_GET_REQUEST * src, DST * dst) {\
        DST##_target_system_SET( pgopro_get_request_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pgopro_get_request_target_component_GET( src  ), dst  );\
        e_GOPRO_COMMAND  item_cmd_id;\
        if( pgopro_get_request_cmd_id_GET( src, &item_cmd_id ) ){\
            DST##_cmd_id_SET( item_cmd_id , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline bool  <SRC>_cmd_id_item_exist( <SRC> * src ){}
static inline e_GOPRO_COMMAND <SRC>_cmd_id_GET( <SRC> * src ){}
*/

#define pgopro_get_request_GOPRO_GET_REQUEST_PULL_FROM(SRC)\
    static inline void pgopro_get_request_GOPRO_GET_REQUEST_pull_from_##SRC ( SRC * src, pgopro_get_request_GOPRO_GET_REQUEST * dst) {\
        pgopro_get_request_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pgopro_get_request_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        if( SRC##_cmd_id_item_exist(src ) )\
        pgopro_get_request_cmd_id_SET( pgopro_get_request_GOPRO_GET_REQUEST_cmd_id_GET( src ),  dst  );\
    }

/**
*WIP: Information about the status of a capture */

typedef Pack CAMERA_CAPTURE_STATUS_camera_capture_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pcamera_capture_status_CAMERA_CAPTURE_STATUS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pcamera_capture_status_CAMERA_CAPTURE_STATUS *pcamera_capture_status_CAMERA_CAPTURE_STATUS_from(CAMERA_CAPTURE_STATUS_camera_capture_status *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_recording_time_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_image_status_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_video_status_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_image_interval_SET( float * src, <DST> * dst  ){}
static inline void <DST>_available_capacity_SET( float * src, <DST> * dst  ){}
*/

#define pcamera_capture_status_CAMERA_CAPTURE_STATUS_PUSH_INTO(DST)\
    static inline void pcamera_capture_status_CAMERA_CAPTURE_STATUS_push_into_##DST ( pcamera_capture_status_CAMERA_CAPTURE_STATUS * src, DST * dst) {\
        DST##_time_boot_ms_SET( pcamera_capture_status_time_boot_ms_GET( src  ), dst  );\
        DST##_recording_time_ms_SET( pcamera_capture_status_recording_time_ms_GET( src  ), dst  );\
        DST##_image_status_SET( pcamera_capture_status_image_status_GET( src  ), dst  );\
        DST##_video_status_SET( pcamera_capture_status_video_status_GET( src  ), dst  );\
        DST##_image_interval_SET( pcamera_capture_status_image_interval_GET( src  ), dst  );\
        DST##_available_capacity_SET( pcamera_capture_status_available_capacity_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int32_t <SRC>_recording_time_ms_GET( <SRC> * src ){}
static inline int8_t <SRC>_image_status_GET( <SRC> * src ){}
static inline int8_t <SRC>_video_status_GET( <SRC> * src ){}
static inline float <SRC>_image_interval_GET( <SRC> * src ){}
static inline float <SRC>_available_capacity_GET( <SRC> * src ){}
*/

#define pcamera_capture_status_CAMERA_CAPTURE_STATUS_PULL_FROM(SRC)\
    static inline void pcamera_capture_status_CAMERA_CAPTURE_STATUS_pull_from_##SRC ( SRC * src, pcamera_capture_status_CAMERA_CAPTURE_STATUS * dst) {\
        pcamera_capture_status_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pcamera_capture_status_recording_time_ms_SET( SRC##_recording_time_ms_GET(src ), dst  );\
        pcamera_capture_status_image_status_SET( SRC##_image_status_GET(src ), dst  );\
        pcamera_capture_status_video_status_SET( SRC##_video_status_GET(src ), dst  );\
        pcamera_capture_status_image_interval_SET( SRC##_image_interval_GET(src ), dst  );\
        pcamera_capture_status_available_capacity_SET( SRC##_available_capacity_GET(src ), dst  );\
    }

/**
*nt. */

typedef Pack GLOBAL_POSITION_INT_global_position_int; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pglobal_position_int_GLOBAL_POSITION_INT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pglobal_position_int_GLOBAL_POSITION_INT *pglobal_position_int_GLOBAL_POSITION_INT_from(GLOBAL_POSITION_INT_global_position_int *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_hdg_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_relative_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( int16_t * src, <DST> * dst  ){}
*/

#define pglobal_position_int_GLOBAL_POSITION_INT_PUSH_INTO(DST)\
    static inline void pglobal_position_int_GLOBAL_POSITION_INT_push_into_##DST ( pglobal_position_int_GLOBAL_POSITION_INT * src, DST * dst) {\
        DST##_hdg_SET( pglobal_position_int_hdg_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( pglobal_position_int_time_boot_ms_GET( src  ), dst  );\
        DST##_lat_SET( pglobal_position_int_lat_GET( src  ), dst  );\
        DST##_lon_SET( pglobal_position_int_lon_GET( src  ), dst  );\
        DST##_alt_SET( pglobal_position_int_alt_GET( src  ), dst  );\
        DST##_relative_alt_SET( pglobal_position_int_relative_alt_GET( src  ), dst  );\
        DST##_vx_SET( pglobal_position_int_vx_GET( src  ), dst  );\
        DST##_vy_SET( pglobal_position_int_vy_GET( src  ), dst  );\
        DST##_vz_SET( pglobal_position_int_vz_GET( src  ), dst  );\
    }


typedef Pack ENCAPSULATED_DATA_encapsulated_data; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pencapsulated_data_ENCAPSULATED_DATA;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pencapsulated_data_ENCAPSULATED_DATA *pencapsulated_data_ENCAPSULATED_DATA_from(ENCAPSULATED_DATA_encapsulated_data *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vencapsulated_data_daTa;
//Maximum field array length constant
#define Pencapsulated_data_daTa_len  ( 253 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seqnr_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vencapsulated_data_daTa * src, <DST> * dst  ){}
*/

#define pencapsulated_data_ENCAPSULATED_DATA_PUSH_INTO(DST)\
    static inline void pencapsulated_data_ENCAPSULATED_DATA_push_into_##DST ( pencapsulated_data_ENCAPSULATED_DATA * src, DST * dst) {\
        DST##_seqnr_SET( pencapsulated_data_seqnr_GET( src  ), dst  );\
        Vencapsulated_data_daTa item_daTa = pencapsulated_data_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_seqnr_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vencapsulated_data_daTa * dst ){}
*/

#define pencapsulated_data_ENCAPSULATED_DATA_PULL_FROM(SRC)\
    static inline void pencapsulated_data_ENCAPSULATED_DATA_pull_from_##SRC ( SRC * src, pencapsulated_data_ENCAPSULATED_DATA * dst) {\
        pencapsulated_data_seqnr_SET( SRC##_seqnr_GET(src ), dst  );\
       Vencapsulated_data_daTa item_daTa = pencapsulated_data_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
*				 estimate of the sytem */

typedef Pack GPS_INPUT_gps_input; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgps_input_GPS_INPUT;// data navigator over pack fields data
/**
															* Wrap GPS_INPUT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgps_input_GPS_INPUT *GPS_INPUT_gps_input_wrap(GPS_INPUT_gps_input *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GPS_INPUT_gps_input *pgps_input_GPS_INPUT_unwrap(pgps_input_GPS_INPUT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_week_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_week_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_gps_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_fix_type_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_hdop_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vdop_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vn_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ve_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vd_SET( float * src, <DST> * dst  ){}
static inline void <DST>_speed_accuracy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_horiz_accuracy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vert_accuracy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_satellites_visible_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_ignore_flags_SET( e_GPS_INPUT_IGNORE_FLAGS * src, <DST> * dst  ){}
*/

#define pgps_input_GPS_INPUT_PUSH_INTO(DST)\
    static inline void pgps_input_GPS_INPUT_push_into_##DST ( pgps_input_GPS_INPUT * src, DST * dst) {\
        DST##_time_week_SET( pgps_input_time_week_GET( src  ), dst  );\
        DST##_time_week_ms_SET( pgps_input_time_week_ms_GET( src  ), dst  );\
        DST##_time_usec_SET( pgps_input_time_usec_GET( src  ), dst  );\
        DST##_gps_id_SET( pgps_input_gps_id_GET( src  ), dst  );\
        DST##_fix_type_SET( pgps_input_fix_type_GET( src  ), dst  );\
        DST##_lat_SET( pgps_input_lat_GET( src  ), dst  );\
        DST##_lon_SET( pgps_input_lon_GET( src  ), dst  );\
        DST##_alt_SET( pgps_input_alt_GET( src  ), dst  );\
        DST##_hdop_SET( pgps_input_hdop_GET( src  ), dst  );\
        DST##_vdop_SET( pgps_input_vdop_GET( src  ), dst  );\
        DST##_vn_SET( pgps_input_vn_GET( src  ), dst  );\
        DST##_ve_SET( pgps_input_ve_GET( src  ), dst  );\
        DST##_vd_SET( pgps_input_vd_GET( src  ), dst  );\
        DST##_speed_accuracy_SET( pgps_input_speed_accuracy_GET( src  ), dst  );\
        DST##_horiz_accuracy_SET( pgps_input_horiz_accuracy_GET( src  ), dst  );\
        DST##_vert_accuracy_SET( pgps_input_vert_accuracy_GET( src  ), dst  );\
        DST##_satellites_visible_SET( pgps_input_satellites_visible_GET( src  ), dst  );\
        e_GPS_INPUT_IGNORE_FLAGS  item_ignore_flags;\
        if( pgps_input_ignore_flags_GET( src, &item_ignore_flags ) ){\
            DST##_ignore_flags_SET( item_ignore_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_time_week_GET( <SRC> * src ){}
static inline int32_t <SRC>_time_week_ms_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_gps_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_fix_type_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline float <SRC>_alt_GET( <SRC> * src ){}
static inline float <SRC>_hdop_GET( <SRC> * src ){}
static inline float <SRC>_vdop_GET( <SRC> * src ){}
static inline float <SRC>_vn_GET( <SRC> * src ){}
static inline float <SRC>_ve_GET( <SRC> * src ){}
static inline float <SRC>_vd_GET( <SRC> * src ){}
static inline float <SRC>_speed_accuracy_GET( <SRC> * src ){}
static inline float <SRC>_horiz_accuracy_GET( <SRC> * src ){}
static inline float <SRC>_vert_accuracy_GET( <SRC> * src ){}
static inline int8_t <SRC>_satellites_visible_GET( <SRC> * src ){}
static inline bool  <SRC>_ignore_flags_item_exist( <SRC> * src ){}
static inline e_GPS_INPUT_IGNORE_FLAGS <SRC>_ignore_flags_GET( <SRC> * src ){}
*/

#define pgps_input_GPS_INPUT_PULL_FROM(SRC)\
    static inline void pgps_input_GPS_INPUT_pull_from_##SRC ( SRC * src, pgps_input_GPS_INPUT * dst) {\
        pgps_input_time_week_SET( SRC##_time_week_GET(src ), dst  );\
        pgps_input_time_week_ms_SET( SRC##_time_week_ms_GET(src ), dst  );\
        pgps_input_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pgps_input_gps_id_SET( SRC##_gps_id_GET(src ), dst  );\
        pgps_input_fix_type_SET( SRC##_fix_type_GET(src ), dst  );\
        pgps_input_lat_SET( SRC##_lat_GET(src ), dst  );\
        pgps_input_lon_SET( SRC##_lon_GET(src ), dst  );\
        pgps_input_alt_SET( SRC##_alt_GET(src ), dst  );\
        pgps_input_hdop_SET( SRC##_hdop_GET(src ), dst  );\
        pgps_input_vdop_SET( SRC##_vdop_GET(src ), dst  );\
        pgps_input_vn_SET( SRC##_vn_GET(src ), dst  );\
        pgps_input_ve_SET( SRC##_ve_GET(src ), dst  );\
        pgps_input_vd_SET( SRC##_vd_GET(src ), dst  );\
        pgps_input_speed_accuracy_SET( SRC##_speed_accuracy_GET(src ), dst  );\
        pgps_input_horiz_accuracy_SET( SRC##_horiz_accuracy_GET(src ), dst  );\
        pgps_input_vert_accuracy_SET( SRC##_vert_accuracy_GET(src ), dst  );\
        pgps_input_satellites_visible_SET( SRC##_satellites_visible_GET(src ), dst  );\
        if( SRC##_ignore_flags_item_exist(src ) )\
        pgps_input_ignore_flags_SET( pgps_input_GPS_INPUT_ignore_flags_GET( src ),  dst  );\
    }

/**
*Send a command with up to seven parameters to the MAV */

typedef Pack COMMAND_LONG_command_long; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcommand_long_COMMAND_LONG;// data navigator over pack fields data
/**
															* Wrap COMMAND_LONG in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcommand_long_COMMAND_LONG *COMMAND_LONG_command_long_wrap(COMMAND_LONG_command_long *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline COMMAND_LONG_command_long *pcommand_long_COMMAND_LONG_unwrap(pcommand_long_COMMAND_LONG *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_confirmation_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param4_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param5_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param6_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param7_SET( float * src, <DST> * dst  ){}
static inline void <DST>_command_SET( e_MAV_CMD * src, <DST> * dst  ){}
*/

#define pcommand_long_COMMAND_LONG_PUSH_INTO(DST)\
    static inline void pcommand_long_COMMAND_LONG_push_into_##DST ( pcommand_long_COMMAND_LONG * src, DST * dst) {\
        DST##_target_system_SET( pcommand_long_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pcommand_long_target_component_GET( src  ), dst  );\
        DST##_confirmation_SET( pcommand_long_confirmation_GET( src  ), dst  );\
        DST##_param1_SET( pcommand_long_param1_GET( src  ), dst  );\
        DST##_param2_SET( pcommand_long_param2_GET( src  ), dst  );\
        DST##_param3_SET( pcommand_long_param3_GET( src  ), dst  );\
        DST##_param4_SET( pcommand_long_param4_GET( src  ), dst  );\
        DST##_param5_SET( pcommand_long_param5_GET( src  ), dst  );\
        DST##_param6_SET( pcommand_long_param6_GET( src  ), dst  );\
        DST##_param7_SET( pcommand_long_param7_GET( src  ), dst  );\
        e_MAV_CMD  item_command;\
        if( pcommand_long_command_GET( src, &item_command ) ){\
            DST##_command_SET( item_command , dst  );\
        }\
    }

/**
*Status of compassmot calibration */

typedef Pack COMPASSMOT_STATUS_compassmot_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pcompassmot_status_COMPASSMOT_STATUS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pcompassmot_status_COMPASSMOT_STATUS *pcompassmot_status_COMPASSMOT_STATUS_from(COMPASSMOT_STATUS_compassmot_status *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_throttle_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_interference_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_current_SET( float * src, <DST> * dst  ){}
static inline void <DST>_CompensationX_SET( float * src, <DST> * dst  ){}
static inline void <DST>_CompensationY_SET( float * src, <DST> * dst  ){}
static inline void <DST>_CompensationZ_SET( float * src, <DST> * dst  ){}
*/

#define pcompassmot_status_COMPASSMOT_STATUS_PUSH_INTO(DST)\
    static inline void pcompassmot_status_COMPASSMOT_STATUS_push_into_##DST ( pcompassmot_status_COMPASSMOT_STATUS * src, DST * dst) {\
        DST##_throttle_SET( pcompassmot_status_throttle_GET( src  ), dst  );\
        DST##_interference_SET( pcompassmot_status_interference_GET( src  ), dst  );\
        DST##_current_SET( pcompassmot_status_current_GET( src  ), dst  );\
        DST##_CompensationX_SET( pcompassmot_status_CompensationX_GET( src  ), dst  );\
        DST##_CompensationY_SET( pcompassmot_status_CompensationY_GET( src  ), dst  );\
        DST##_CompensationZ_SET( pcompassmot_status_CompensationZ_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_throttle_GET( <SRC> * src ){}
static inline int16_t <SRC>_interference_GET( <SRC> * src ){}
static inline float <SRC>_current_GET( <SRC> * src ){}
static inline float <SRC>_CompensationX_GET( <SRC> * src ){}
static inline float <SRC>_CompensationY_GET( <SRC> * src ){}
static inline float <SRC>_CompensationZ_GET( <SRC> * src ){}
*/

#define pcompassmot_status_COMPASSMOT_STATUS_PULL_FROM(SRC)\
    static inline void pcompassmot_status_COMPASSMOT_STATUS_pull_from_##SRC ( SRC * src, pcompassmot_status_COMPASSMOT_STATUS * dst) {\
        pcompassmot_status_throttle_SET( SRC##_throttle_GET(src ), dst  );\
        pcompassmot_status_interference_SET( SRC##_interference_GET(src ), dst  );\
        pcompassmot_status_current_SET( SRC##_current_GET(src ), dst  );\
        pcompassmot_status_CompensationX_SET( SRC##_CompensationX_GET(src ), dst  );\
        pcompassmot_status_CompensationY_SET( SRC##_CompensationY_GET(src ), dst  );\
        pcompassmot_status_CompensationZ_SET( SRC##_CompensationZ_GET(src ), dst  );\
    }

/**
*Request a chunk of a log */

typedef Pack LOG_REQUEST_DATA_log_request_data; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plog_request_data_LOG_REQUEST_DATA;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plog_request_data_LOG_REQUEST_DATA *plog_request_data_LOG_REQUEST_DATA_from(LOG_REQUEST_DATA_log_request_data *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_id_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ofs_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_count_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define plog_request_data_LOG_REQUEST_DATA_PUSH_INTO(DST)\
    static inline void plog_request_data_LOG_REQUEST_DATA_push_into_##DST ( plog_request_data_LOG_REQUEST_DATA * src, DST * dst) {\
        DST##_id_SET( plog_request_data_id_GET( src  ), dst  );\
        DST##_ofs_SET( plog_request_data_ofs_GET( src  ), dst  );\
        DST##_count_SET( plog_request_data_count_GET( src  ), dst  );\
        DST##_target_system_SET( plog_request_data_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( plog_request_data_target_component_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_id_GET( <SRC> * src ){}
static inline int32_t <SRC>_ofs_GET( <SRC> * src ){}
static inline int32_t <SRC>_count_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
*/

#define plog_request_data_LOG_REQUEST_DATA_PULL_FROM(SRC)\
    static inline void plog_request_data_LOG_REQUEST_DATA_pull_from_##SRC ( SRC * src, plog_request_data_LOG_REQUEST_DATA * dst) {\
        plog_request_data_id_SET( SRC##_id_GET(src ), dst  );\
        plog_request_data_ofs_SET( SRC##_ofs_GET(src ), dst  );\
        plog_request_data_count_SET( SRC##_count_GET(src ), dst  );\
        plog_request_data_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        plog_request_data_target_component_SET( SRC##_target_component_GET(src ), dst  );\
    }

/**
*The global position, as returned by the Global Positioning System (GPS). This is
*				 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame). */

typedef Pack GPS_RAW_INT_gps_raw_int; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgps_raw_int_GPS_RAW_INT;// data navigator over pack fields data
/**
															* Wrap GPS_RAW_INT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgps_raw_int_GPS_RAW_INT *GPS_RAW_INT_gps_raw_int_wrap(GPS_RAW_INT_gps_raw_int *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GPS_RAW_INT_gps_raw_int *pgps_raw_int_GPS_RAW_INT_unwrap(pgps_raw_int_GPS_RAW_INT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_eph_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_epv_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_vel_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_cog_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_satellites_visible_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_fix_type_SET( e_GPS_FIX_TYPE * src, <DST> * dst  ){}
static inline void <DST>_alt_ellipsoid_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_h_acc_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_v_acc_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_vel_acc_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_hdg_acc_SET( int32_t * src, <DST> * dst  ){}
*/

#define pgps_raw_int_GPS_RAW_INT_PUSH_INTO(DST)\
    static inline void pgps_raw_int_GPS_RAW_INT_push_into_##DST ( pgps_raw_int_GPS_RAW_INT * src, DST * dst) {\
        DST##_eph_SET( pgps_raw_int_eph_GET( src  ), dst  );\
        DST##_epv_SET( pgps_raw_int_epv_GET( src  ), dst  );\
        DST##_vel_SET( pgps_raw_int_vel_GET( src  ), dst  );\
        DST##_cog_SET( pgps_raw_int_cog_GET( src  ), dst  );\
        DST##_time_usec_SET( pgps_raw_int_time_usec_GET( src  ), dst  );\
        DST##_lat_SET( pgps_raw_int_lat_GET( src  ), dst  );\
        DST##_lon_SET( pgps_raw_int_lon_GET( src  ), dst  );\
        DST##_alt_SET( pgps_raw_int_alt_GET( src  ), dst  );\
        DST##_satellites_visible_SET( pgps_raw_int_satellites_visible_GET( src  ), dst  );\
        e_GPS_FIX_TYPE  item_fix_type;\
        if( pgps_raw_int_fix_type_GET( src, &item_fix_type ) ){\
            DST##_fix_type_SET( item_fix_type , dst  );\
        }\
        int32_t  item_alt_ellipsoid;\
        if( pgps_raw_int_alt_ellipsoid_GET( src, &item_alt_ellipsoid ) ){\
            DST##_alt_ellipsoid_SET( item_alt_ellipsoid , dst  );\
        }\
        int32_t  item_h_acc;\
        if( pgps_raw_int_h_acc_GET( src, &item_h_acc ) ){\
            DST##_h_acc_SET( item_h_acc , dst  );\
        }\
        int32_t  item_v_acc;\
        if( pgps_raw_int_v_acc_GET( src, &item_v_acc ) ){\
            DST##_v_acc_SET( item_v_acc , dst  );\
        }\
        int32_t  item_vel_acc;\
        if( pgps_raw_int_vel_acc_GET( src, &item_vel_acc ) ){\
            DST##_vel_acc_SET( item_vel_acc , dst  );\
        }\
        int32_t  item_hdg_acc;\
        if( pgps_raw_int_hdg_acc_GET( src, &item_hdg_acc ) ){\
            DST##_hdg_acc_SET( item_hdg_acc , dst  );\
        }\
    }

/**
*Camera Event */

typedef Pack CAMERA_STATUS_camera_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcamera_status_CAMERA_STATUS;// data navigator over pack fields data
/**
															* Wrap CAMERA_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcamera_status_CAMERA_STATUS *CAMERA_STATUS_camera_status_wrap(CAMERA_STATUS_camera_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline CAMERA_STATUS_camera_status *pcamera_status_CAMERA_STATUS_unwrap(pcamera_status_CAMERA_STATUS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_img_idx_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_cam_idx_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_p1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p4_SET( float * src, <DST> * dst  ){}
static inline void <DST>_event_id_SET( e_CAMERA_STATUS_TYPES * src, <DST> * dst  ){}
*/

#define pcamera_status_CAMERA_STATUS_PUSH_INTO(DST)\
    static inline void pcamera_status_CAMERA_STATUS_push_into_##DST ( pcamera_status_CAMERA_STATUS * src, DST * dst) {\
        DST##_img_idx_SET( pcamera_status_img_idx_GET( src  ), dst  );\
        DST##_time_usec_SET( pcamera_status_time_usec_GET( src  ), dst  );\
        DST##_target_system_SET( pcamera_status_target_system_GET( src  ), dst  );\
        DST##_cam_idx_SET( pcamera_status_cam_idx_GET( src  ), dst  );\
        DST##_p1_SET( pcamera_status_p1_GET( src  ), dst  );\
        DST##_p2_SET( pcamera_status_p2_GET( src  ), dst  );\
        DST##_p3_SET( pcamera_status_p3_GET( src  ), dst  );\
        DST##_p4_SET( pcamera_status_p4_GET( src  ), dst  );\
        e_CAMERA_STATUS_TYPES  item_event_id;\
        if( pcamera_status_event_id_GET( src, &item_event_id ) ){\
            DST##_event_id_SET( item_event_id , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_img_idx_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_cam_idx_GET( <SRC> * src ){}
static inline float <SRC>_p1_GET( <SRC> * src ){}
static inline float <SRC>_p2_GET( <SRC> * src ){}
static inline float <SRC>_p3_GET( <SRC> * src ){}
static inline float <SRC>_p4_GET( <SRC> * src ){}
static inline bool  <SRC>_event_id_item_exist( <SRC> * src ){}
static inline e_CAMERA_STATUS_TYPES <SRC>_event_id_GET( <SRC> * src ){}
*/

#define pcamera_status_CAMERA_STATUS_PULL_FROM(SRC)\
    static inline void pcamera_status_CAMERA_STATUS_pull_from_##SRC ( SRC * src, pcamera_status_CAMERA_STATUS * dst) {\
        pcamera_status_img_idx_SET( SRC##_img_idx_GET(src ), dst  );\
        pcamera_status_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pcamera_status_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pcamera_status_cam_idx_SET( SRC##_cam_idx_GET(src ), dst  );\
        pcamera_status_p1_SET( SRC##_p1_GET(src ), dst  );\
        pcamera_status_p2_SET( SRC##_p2_GET(src ), dst  );\
        pcamera_status_p3_SET( SRC##_p3_GET(src ), dst  );\
        pcamera_status_p4_SET( SRC##_p4_GET(src ), dst  );\
        if( SRC##_event_id_item_exist(src ) )\
        pcamera_status_event_id_SET( pcamera_status_CAMERA_STATUS_event_id_GET( src ),  dst  );\
    }

/**
*The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
*				 inactive should be set to UINT16_MAX */

typedef Pack RC_CHANNELS_SCALED_rc_channels_scaled; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t prc_channels_scaled_RC_CHANNELS_SCALED;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline prc_channels_scaled_RC_CHANNELS_SCALED *prc_channels_scaled_RC_CHANNELS_SCALED_from(RC_CHANNELS_SCALED_rc_channels_scaled *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_port_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_chan1_scaled_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan2_scaled_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan3_scaled_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan4_scaled_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan5_scaled_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan6_scaled_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan7_scaled_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan8_scaled_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_rssi_SET( int8_t * src, <DST> * dst  ){}
*/

#define prc_channels_scaled_RC_CHANNELS_SCALED_PUSH_INTO(DST)\
    static inline void prc_channels_scaled_RC_CHANNELS_SCALED_push_into_##DST ( prc_channels_scaled_RC_CHANNELS_SCALED * src, DST * dst) {\
        DST##_time_boot_ms_SET( prc_channels_scaled_time_boot_ms_GET( src  ), dst  );\
        DST##_port_SET( prc_channels_scaled_port_GET( src  ), dst  );\
        DST##_chan1_scaled_SET( prc_channels_scaled_chan1_scaled_GET( src  ), dst  );\
        DST##_chan2_scaled_SET( prc_channels_scaled_chan2_scaled_GET( src  ), dst  );\
        DST##_chan3_scaled_SET( prc_channels_scaled_chan3_scaled_GET( src  ), dst  );\
        DST##_chan4_scaled_SET( prc_channels_scaled_chan4_scaled_GET( src  ), dst  );\
        DST##_chan5_scaled_SET( prc_channels_scaled_chan5_scaled_GET( src  ), dst  );\
        DST##_chan6_scaled_SET( prc_channels_scaled_chan6_scaled_GET( src  ), dst  );\
        DST##_chan7_scaled_SET( prc_channels_scaled_chan7_scaled_GET( src  ), dst  );\
        DST##_chan8_scaled_SET( prc_channels_scaled_chan8_scaled_GET( src  ), dst  );\
        DST##_rssi_SET( prc_channels_scaled_rssi_GET( src  ), dst  );\
    }

/**
*WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS. */

typedef Pack CAMERA_SETTINGS_camera_settings; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcamera_settings_CAMERA_SETTINGS;// data navigator over pack fields data
/**
															* Wrap CAMERA_SETTINGS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcamera_settings_CAMERA_SETTINGS *CAMERA_SETTINGS_camera_settings_wrap(CAMERA_SETTINGS_camera_settings *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline CAMERA_SETTINGS_camera_settings *pcamera_settings_CAMERA_SETTINGS_unwrap(pcamera_settings_CAMERA_SETTINGS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_mode_id_SET( e_CAMERA_MODE * src, <DST> * dst  ){}
*/

#define pcamera_settings_CAMERA_SETTINGS_PUSH_INTO(DST)\
    static inline void pcamera_settings_CAMERA_SETTINGS_push_into_##DST ( pcamera_settings_CAMERA_SETTINGS * src, DST * dst) {\
        DST##_time_boot_ms_SET( pcamera_settings_time_boot_ms_GET( src  ), dst  );\
        e_CAMERA_MODE  item_mode_id;\
        if( pcamera_settings_mode_id_GET( src, &item_mode_id ) ){\
            DST##_mode_id_SET( item_mode_id , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline bool  <SRC>_mode_id_item_exist( <SRC> * src ){}
static inline e_CAMERA_MODE <SRC>_mode_id_GET( <SRC> * src ){}
*/

#define pcamera_settings_CAMERA_SETTINGS_PULL_FROM(SRC)\
    static inline void pcamera_settings_CAMERA_SETTINGS_pull_from_##SRC ( SRC * src, pcamera_settings_CAMERA_SETTINGS * dst) {\
        pcamera_settings_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        if( SRC##_mode_id_item_exist(src ) )\
        pcamera_settings_mode_id_SET( pcamera_settings_CAMERA_SETTINGS_mode_id_GET( src ),  dst  );\
    }

/**
*Read registers reply */

typedef Pack DEVICE_OP_READ_REPLY_device_op_read_reply; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdevice_op_read_reply_DEVICE_OP_READ_REPLY;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdevice_op_read_reply_DEVICE_OP_READ_REPLY *pdevice_op_read_reply_DEVICE_OP_READ_REPLY_from(DEVICE_OP_READ_REPLY_device_op_read_reply *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdevice_op_read_reply_daTa;
//Maximum field array length constant
#define Pdevice_op_read_reply_daTa_len  ( 128 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_request_id_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_result_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_regstart_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_count_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vdevice_op_read_reply_daTa * src, <DST> * dst  ){}
*/

#define pdevice_op_read_reply_DEVICE_OP_READ_REPLY_PUSH_INTO(DST)\
    static inline void pdevice_op_read_reply_DEVICE_OP_READ_REPLY_push_into_##DST ( pdevice_op_read_reply_DEVICE_OP_READ_REPLY * src, DST * dst) {\
        DST##_request_id_SET( pdevice_op_read_reply_request_id_GET( src  ), dst  );\
        DST##_result_SET( pdevice_op_read_reply_result_GET( src  ), dst  );\
        DST##_regstart_SET( pdevice_op_read_reply_regstart_GET( src  ), dst  );\
        DST##_count_SET( pdevice_op_read_reply_count_GET( src  ), dst  );\
        Vdevice_op_read_reply_daTa item_daTa = pdevice_op_read_reply_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_request_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_result_GET( <SRC> * src ){}
static inline int8_t <SRC>_regstart_GET( <SRC> * src ){}
static inline int8_t <SRC>_count_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vdevice_op_read_reply_daTa * dst ){}
*/

#define pdevice_op_read_reply_DEVICE_OP_READ_REPLY_PULL_FROM(SRC)\
    static inline void pdevice_op_read_reply_DEVICE_OP_READ_REPLY_pull_from_##SRC ( SRC * src, pdevice_op_read_reply_DEVICE_OP_READ_REPLY * dst) {\
        pdevice_op_read_reply_request_id_SET( SRC##_request_id_GET(src ), dst  );\
        pdevice_op_read_reply_result_SET( SRC##_result_GET(src ), dst  );\
        pdevice_op_read_reply_regstart_SET( SRC##_regstart_GET(src ), dst  );\
        pdevice_op_read_reply_count_SET( SRC##_count_GET(src ), dst  );\
       Vdevice_op_read_reply_daTa item_daTa = pdevice_op_read_reply_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
*				 sensor. The sensor values should be the raw, UNSCALED ADC values */

typedef Pack RAW_PRESSURE_raw_pressure; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t praw_pressure_RAW_PRESSURE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline praw_pressure_RAW_PRESSURE *praw_pressure_RAW_PRESSURE_from(RAW_PRESSURE_raw_pressure *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_press_abs_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_press_diff1_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_press_diff2_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( int16_t * src, <DST> * dst  ){}
*/

#define praw_pressure_RAW_PRESSURE_PUSH_INTO(DST)\
    static inline void praw_pressure_RAW_PRESSURE_push_into_##DST ( praw_pressure_RAW_PRESSURE * src, DST * dst) {\
        DST##_time_usec_SET( praw_pressure_time_usec_GET( src  ), dst  );\
        DST##_press_abs_SET( praw_pressure_press_abs_GET( src  ), dst  );\
        DST##_press_diff1_SET( praw_pressure_press_diff1_GET( src  ), dst  );\
        DST##_press_diff2_SET( praw_pressure_press_diff2_GET( src  ), dst  );\
        DST##_temperature_SET( praw_pressure_temperature_GET( src  ), dst  );\
    }

/**
*Control on-board Camera Control System to take shots. */

typedef Pack DIGICAM_CONTROL_digicam_control; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdigicam_control_DIGICAM_CONTROL;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdigicam_control_DIGICAM_CONTROL *pdigicam_control_DIGICAM_CONTROL_from(DIGICAM_CONTROL_digicam_control *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_session_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_zoom_pos_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_zoom_step_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_focus_lock_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_shot_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_command_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_extra_param_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_extra_value_SET( float * src, <DST> * dst  ){}
*/

#define pdigicam_control_DIGICAM_CONTROL_PUSH_INTO(DST)\
    static inline void pdigicam_control_DIGICAM_CONTROL_push_into_##DST ( pdigicam_control_DIGICAM_CONTROL * src, DST * dst) {\
        DST##_target_system_SET( pdigicam_control_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pdigicam_control_target_component_GET( src  ), dst  );\
        DST##_session_SET( pdigicam_control_session_GET( src  ), dst  );\
        DST##_zoom_pos_SET( pdigicam_control_zoom_pos_GET( src  ), dst  );\
        DST##_zoom_step_SET( pdigicam_control_zoom_step_GET( src  ), dst  );\
        DST##_focus_lock_SET( pdigicam_control_focus_lock_GET( src  ), dst  );\
        DST##_shot_SET( pdigicam_control_shot_GET( src  ), dst  );\
        DST##_command_id_SET( pdigicam_control_command_id_GET( src  ), dst  );\
        DST##_extra_param_SET( pdigicam_control_extra_param_GET( src  ), dst  );\
        DST##_extra_value_SET( pdigicam_control_extra_value_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_session_GET( <SRC> * src ){}
static inline int8_t <SRC>_zoom_pos_GET( <SRC> * src ){}
static inline int8_t <SRC>_zoom_step_GET( <SRC> * src ){}
static inline int8_t <SRC>_focus_lock_GET( <SRC> * src ){}
static inline int8_t <SRC>_shot_GET( <SRC> * src ){}
static inline int8_t <SRC>_command_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_extra_param_GET( <SRC> * src ){}
static inline float <SRC>_extra_value_GET( <SRC> * src ){}
*/

#define pdigicam_control_DIGICAM_CONTROL_PULL_FROM(SRC)\
    static inline void pdigicam_control_DIGICAM_CONTROL_pull_from_##SRC ( SRC * src, pdigicam_control_DIGICAM_CONTROL * dst) {\
        pdigicam_control_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pdigicam_control_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pdigicam_control_session_SET( SRC##_session_GET(src ), dst  );\
        pdigicam_control_zoom_pos_SET( SRC##_zoom_pos_GET(src ), dst  );\
        pdigicam_control_zoom_step_SET( SRC##_zoom_step_GET(src ), dst  );\
        pdigicam_control_focus_lock_SET( SRC##_focus_lock_GET(src ), dst  );\
        pdigicam_control_shot_SET( SRC##_shot_GET(src ), dst  );\
        pdigicam_control_command_id_SET( SRC##_command_id_GET(src ), dst  );\
        pdigicam_control_extra_param_SET( SRC##_extra_param_GET(src ), dst  );\
        pdigicam_control_extra_value_SET( SRC##_extra_value_GET(src ), dst  );\
    }

/**
*Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
*				 efficient way for testing new messages and getting experimental debug output */

typedef Pack NAMED_VALUE_FLOAT_named_value_float; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pnamed_value_float_NAMED_VALUE_FLOAT;// data navigator over pack fields data
/**
															* Wrap NAMED_VALUE_FLOAT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pnamed_value_float_NAMED_VALUE_FLOAT *NAMED_VALUE_FLOAT_named_value_float_wrap(NAMED_VALUE_FLOAT_named_value_float *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline NAMED_VALUE_FLOAT_named_value_float *pnamed_value_float_NAMED_VALUE_FLOAT_unwrap(pnamed_value_float_NAMED_VALUE_FLOAT *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vnamed_value_float_name;
//Maximum field array length constant
#define Pnamed_value_float_name_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_value_SET( float * src, <DST> * dst  ){}
static inline void <DST>_name_SET( Vnamed_value_float_name * src, <DST> * dst  ){}
*/

#define pnamed_value_float_NAMED_VALUE_FLOAT_PUSH_INTO(DST)\
    static inline void pnamed_value_float_NAMED_VALUE_FLOAT_push_into_##DST ( pnamed_value_float_NAMED_VALUE_FLOAT * src, DST * dst) {\
        DST##_time_boot_ms_SET( pnamed_value_float_time_boot_ms_GET( src  ), dst  );\
        DST##_value_SET( pnamed_value_float_value_GET( src  ), dst  );\
        Vnamed_value_float_name  item_name;\
        if( pnamed_value_float_name_GET( src, &item_name ) ){\
            DST##_name_SET( &item_name, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline float <SRC>_value_GET( <SRC> * src ){}
static inline size_t  <SRC>_name_item_exist( <SRC> * src  ){}
static inline void <SRC>_name_GET( <SRC> * src, Vnamed_value_float_name * dst ){}
*/

#define pnamed_value_float_NAMED_VALUE_FLOAT_PULL_FROM(SRC)\
    static inline void pnamed_value_float_NAMED_VALUE_FLOAT_pull_from_##SRC ( SRC * src, pnamed_value_float_NAMED_VALUE_FLOAT * dst) {\
        pnamed_value_float_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pnamed_value_float_value_SET( SRC##_value_GET(src ), dst  );\
        const size_t len_name = SRC##_name_item_exist(src );\
        if( len_name ){\
            Vnamed_value_float_name    item_name = pnamed_value_float_name_SET( NULL, len_name, dst  );\
            pnamed_value_float_NAMED_VALUE_FLOAT_name_GET(src, &item_name );\
        }\
    }

/**
*Heartbeat from a HeroBus attached GoPro */

typedef Pack GOPRO_HEARTBEAT_gopro_heartbeat; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgopro_heartbeat_GOPRO_HEARTBEAT;// data navigator over pack fields data
/**
															* Wrap GOPRO_HEARTBEAT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgopro_heartbeat_GOPRO_HEARTBEAT *GOPRO_HEARTBEAT_gopro_heartbeat_wrap(GOPRO_HEARTBEAT_gopro_heartbeat *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GOPRO_HEARTBEAT_gopro_heartbeat *pgopro_heartbeat_GOPRO_HEARTBEAT_unwrap(pgopro_heartbeat_GOPRO_HEARTBEAT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_status_SET( e_GOPRO_HEARTBEAT_STATUS * src, <DST> * dst  ){}
static inline void <DST>_capture_mode_SET( e_GOPRO_CAPTURE_MODE * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_GOPRO_HEARTBEAT_FLAGS * src, <DST> * dst  ){}
*/

#define pgopro_heartbeat_GOPRO_HEARTBEAT_PUSH_INTO(DST)\
    static inline void pgopro_heartbeat_GOPRO_HEARTBEAT_push_into_##DST ( pgopro_heartbeat_GOPRO_HEARTBEAT * src, DST * dst) {\
        e_GOPRO_HEARTBEAT_STATUS  item_status;\
        if( pgopro_heartbeat_status_GET( src, &item_status ) ){\
            DST##_status_SET( item_status , dst  );\
        }\
        e_GOPRO_CAPTURE_MODE  item_capture_mode;\
        if( pgopro_heartbeat_capture_mode_GET( src, &item_capture_mode ) ){\
            DST##_capture_mode_SET( item_capture_mode , dst  );\
        }\
        e_GOPRO_HEARTBEAT_FLAGS  item_flags;\
        if( pgopro_heartbeat_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline bool  <SRC>_status_item_exist( <SRC> * src ){}
static inline e_GOPRO_HEARTBEAT_STATUS <SRC>_status_GET( <SRC> * src ){}
static inline bool  <SRC>_capture_mode_item_exist( <SRC> * src ){}
static inline e_GOPRO_CAPTURE_MODE <SRC>_capture_mode_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_GOPRO_HEARTBEAT_FLAGS <SRC>_flags_GET( <SRC> * src ){}
*/

#define pgopro_heartbeat_GOPRO_HEARTBEAT_PULL_FROM(SRC)\
    static inline void pgopro_heartbeat_GOPRO_HEARTBEAT_pull_from_##SRC ( SRC * src, pgopro_heartbeat_GOPRO_HEARTBEAT * dst) {\
        if( SRC##_status_item_exist(src ) )\
        pgopro_heartbeat_status_SET( pgopro_heartbeat_GOPRO_HEARTBEAT_status_GET( src ),  dst  );\
        if( SRC##_capture_mode_item_exist(src ) )\
        pgopro_heartbeat_capture_mode_SET( pgopro_heartbeat_GOPRO_HEARTBEAT_capture_mode_GET( src ),  dst  );\
        if( SRC##_flags_item_exist(src ) )\
        pgopro_heartbeat_flags_SET( pgopro_heartbeat_GOPRO_HEARTBEAT_flags_GET( src ),  dst  );\
    }

/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right). */

typedef Pack ATTITUDE_attitude; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pattitude_ATTITUDE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pattitude_ATTITUDE *pattitude_ATTITUDE_from(ATTITUDE_attitude *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_rollspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitchspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yawspeed_SET( float * src, <DST> * dst  ){}
*/

#define pattitude_ATTITUDE_PUSH_INTO(DST)\
    static inline void pattitude_ATTITUDE_push_into_##DST ( pattitude_ATTITUDE * src, DST * dst) {\
        DST##_time_boot_ms_SET( pattitude_time_boot_ms_GET( src  ), dst  );\
        DST##_roll_SET( pattitude_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pattitude_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pattitude_yaw_GET( src  ), dst  );\
        DST##_rollspeed_SET( pattitude_rollspeed_GET( src  ), dst  );\
        DST##_pitchspeed_SET( pattitude_pitchspeed_GET( src  ), dst  );\
        DST##_yawspeed_SET( pattitude_yawspeed_GET( src  ), dst  );\
    }

/**
*This message is sent to the MAV to write a partial list. If start index == end index, only one item will
*				 be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
*				 be REJECTED */

typedef Pack MISSION_WRITE_PARTIAL_LIST_mission_write_partial_list; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST;// data navigator over pack fields data
/**
															* Wrap MISSION_WRITE_PARTIAL_LIST in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *MISSION_WRITE_PARTIAL_LIST_mission_write_partial_list_wrap(MISSION_WRITE_PARTIAL_LIST_mission_write_partial_list *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_WRITE_PARTIAL_LIST_mission_write_partial_list *pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST_unwrap(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_start_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_end_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST_PUSH_INTO(DST)\
    static inline void pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST_push_into_##DST ( pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST * src, DST * dst) {\
        DST##_target_system_SET( pmission_write_partial_list_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_write_partial_list_target_component_GET( src  ), dst  );\
        DST##_start_index_SET( pmission_write_partial_list_start_index_GET( src  ), dst  );\
        DST##_end_index_SET( pmission_write_partial_list_end_index_GET( src  ), dst  );\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_write_partial_list_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*Status of secondary AHRS filter if available */

typedef Pack AHRS2_ahrs2; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pahrs2_AHRS2;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pahrs2_AHRS2 *pahrs2_AHRS2_from(AHRS2_ahrs2 *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_altitude_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lng_SET( int32_t * src, <DST> * dst  ){}
*/

#define pahrs2_AHRS2_PUSH_INTO(DST)\
    static inline void pahrs2_AHRS2_push_into_##DST ( pahrs2_AHRS2 * src, DST * dst) {\
        DST##_roll_SET( pahrs2_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pahrs2_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pahrs2_yaw_GET( src  ), dst  );\
        DST##_altitude_SET( pahrs2_altitude_GET( src  ), dst  );\
        DST##_lat_SET( pahrs2_lat_GET( src  ), dst  );\
        DST##_lng_SET( pahrs2_lng_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_roll_GET( <SRC> * src ){}
static inline float <SRC>_pitch_GET( <SRC> * src ){}
static inline float <SRC>_yaw_GET( <SRC> * src ){}
static inline float <SRC>_altitude_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lng_GET( <SRC> * src ){}
*/

#define pahrs2_AHRS2_PULL_FROM(SRC)\
    static inline void pahrs2_AHRS2_pull_from_##SRC ( SRC * src, pahrs2_AHRS2 * dst) {\
        pahrs2_roll_SET( SRC##_roll_GET(src ), dst  );\
        pahrs2_pitch_SET( SRC##_pitch_GET(src ), dst  );\
        pahrs2_yaw_SET( SRC##_yaw_GET(src ), dst  );\
        pahrs2_altitude_SET( SRC##_altitude_GET(src ), dst  );\
        pahrs2_lat_SET( SRC##_lat_GET(src ), dst  );\
        pahrs2_lng_SET( SRC##_lng_GET(src ), dst  );\
    }

/**
*Erase all logs */

typedef Pack LOG_ERASE_log_erase; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plog_erase_LOG_ERASE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plog_erase_LOG_ERASE *plog_erase_LOG_ERASE_from(LOG_ERASE_log_erase *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define plog_erase_LOG_ERASE_PUSH_INTO(DST)\
    static inline void plog_erase_LOG_ERASE_push_into_##DST ( plog_erase_LOG_ERASE * src, DST * dst) {\
        DST##_target_system_SET( plog_erase_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( plog_erase_target_component_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
*/

#define plog_erase_LOG_ERASE_PULL_FROM(SRC)\
    static inline void plog_erase_LOG_ERASE_pull_from_##SRC ( SRC * src, plog_erase_LOG_ERASE * dst) {\
        plog_erase_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        plog_erase_target_component_SET( SRC##_target_component_GET(src ), dst  );\
    }

/**
*Request for terrain data and terrain status */

typedef Pack TERRAIN_REQUEST_terrain_request; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pterrain_request_TERRAIN_REQUEST;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pterrain_request_TERRAIN_REQUEST *pterrain_request_TERRAIN_REQUEST_from(TERRAIN_REQUEST_terrain_request *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_grid_spacing_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_mask_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
*/

#define pterrain_request_TERRAIN_REQUEST_PUSH_INTO(DST)\
    static inline void pterrain_request_TERRAIN_REQUEST_push_into_##DST ( pterrain_request_TERRAIN_REQUEST * src, DST * dst) {\
        DST##_grid_spacing_SET( pterrain_request_grid_spacing_GET( src  ), dst  );\
        DST##_mask_SET( pterrain_request_mask_GET( src  ), dst  );\
        DST##_lat_SET( pterrain_request_lat_GET( src  ), dst  );\
        DST##_lon_SET( pterrain_request_lon_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_grid_spacing_GET( <SRC> * src ){}
static inline int64_t <SRC>_mask_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
*/

#define pterrain_request_TERRAIN_REQUEST_PULL_FROM(SRC)\
    static inline void pterrain_request_TERRAIN_REQUEST_pull_from_##SRC ( SRC * src, pterrain_request_TERRAIN_REQUEST * dst) {\
        pterrain_request_grid_spacing_SET( SRC##_grid_spacing_GET(src ), dst  );\
        pterrain_request_mask_SET( SRC##_mask_GET(src ), dst  );\
        pterrain_request_lat_SET( SRC##_lat_GET(src ), dst  );\
        pterrain_request_lon_SET( SRC##_lon_GET(src ), dst  );\
    }

/**
*Message with some status from APM to GCS about camera or antenna mount */

typedef Pack MOUNT_STATUS_mount_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmount_status_MOUNT_STATUS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmount_status_MOUNT_STATUS *pmount_status_MOUNT_STATUS_from(MOUNT_STATUS_mount_status *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_pointing_a_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_pointing_b_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_pointing_c_SET( int32_t * src, <DST> * dst  ){}
*/

#define pmount_status_MOUNT_STATUS_PUSH_INTO(DST)\
    static inline void pmount_status_MOUNT_STATUS_push_into_##DST ( pmount_status_MOUNT_STATUS * src, DST * dst) {\
        DST##_target_system_SET( pmount_status_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmount_status_target_component_GET( src  ), dst  );\
        DST##_pointing_a_SET( pmount_status_pointing_a_GET( src  ), dst  );\
        DST##_pointing_b_SET( pmount_status_pointing_b_GET( src  ), dst  );\
        DST##_pointing_c_SET( pmount_status_pointing_c_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int32_t <SRC>_pointing_a_GET( <SRC> * src ){}
static inline int32_t <SRC>_pointing_b_GET( <SRC> * src ){}
static inline int32_t <SRC>_pointing_c_GET( <SRC> * src ){}
*/

#define pmount_status_MOUNT_STATUS_PULL_FROM(SRC)\
    static inline void pmount_status_MOUNT_STATUS_pull_from_##SRC ( SRC * src, pmount_status_MOUNT_STATUS * dst) {\
        pmount_status_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pmount_status_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pmount_status_pointing_a_SET( SRC##_pointing_a_GET(src ), dst  );\
        pmount_status_pointing_b_SET( SRC##_pointing_b_GET(src ), dst  );\
        pmount_status_pointing_c_SET( SRC##_pointing_c_GET(src ), dst  );\
    }

/**
*Setpoint in roll, pitch, yaw and thrust from the operator */

typedef Pack MANUAL_SETPOINT_manual_setpoint; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmanual_setpoint_MANUAL_SETPOINT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmanual_setpoint_MANUAL_SETPOINT *pmanual_setpoint_MANUAL_SETPOINT_from(MANUAL_SETPOINT_manual_setpoint *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_thrust_SET( float * src, <DST> * dst  ){}
static inline void <DST>_mode_switch_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_manual_override_switch_SET( int8_t * src, <DST> * dst  ){}
*/

#define pmanual_setpoint_MANUAL_SETPOINT_PUSH_INTO(DST)\
    static inline void pmanual_setpoint_MANUAL_SETPOINT_push_into_##DST ( pmanual_setpoint_MANUAL_SETPOINT * src, DST * dst) {\
        DST##_time_boot_ms_SET( pmanual_setpoint_time_boot_ms_GET( src  ), dst  );\
        DST##_roll_SET( pmanual_setpoint_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pmanual_setpoint_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pmanual_setpoint_yaw_GET( src  ), dst  );\
        DST##_thrust_SET( pmanual_setpoint_thrust_GET( src  ), dst  );\
        DST##_mode_switch_SET( pmanual_setpoint_mode_switch_GET( src  ), dst  );\
        DST##_manual_override_switch_SET( pmanual_setpoint_manual_override_switch_GET( src  ), dst  );\
    }

/**
*PID tuning information */

typedef Pack PID_TUNING_pid_tuning; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor ppid_tuning_PID_TUNING;// data navigator over pack fields data
/**
															* Wrap PID_TUNING in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline ppid_tuning_PID_TUNING *PID_TUNING_pid_tuning_wrap(PID_TUNING_pid_tuning *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PID_TUNING_pid_tuning *ppid_tuning_PID_TUNING_unwrap(ppid_tuning_PID_TUNING *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_desired_SET( float * src, <DST> * dst  ){}
static inline void <DST>_achieved_SET( float * src, <DST> * dst  ){}
static inline void <DST>_FF_SET( float * src, <DST> * dst  ){}
static inline void <DST>_P_SET( float * src, <DST> * dst  ){}
static inline void <DST>_I_SET( float * src, <DST> * dst  ){}
static inline void <DST>_D_SET( float * src, <DST> * dst  ){}
static inline void <DST>_axis_SET( e_PID_TUNING_AXIS * src, <DST> * dst  ){}
*/

#define ppid_tuning_PID_TUNING_PUSH_INTO(DST)\
    static inline void ppid_tuning_PID_TUNING_push_into_##DST ( ppid_tuning_PID_TUNING * src, DST * dst) {\
        DST##_desired_SET( ppid_tuning_desired_GET( src  ), dst  );\
        DST##_achieved_SET( ppid_tuning_achieved_GET( src  ), dst  );\
        DST##_FF_SET( ppid_tuning_FF_GET( src  ), dst  );\
        DST##_P_SET( ppid_tuning_P_GET( src  ), dst  );\
        DST##_I_SET( ppid_tuning_I_GET( src  ), dst  );\
        DST##_D_SET( ppid_tuning_D_GET( src  ), dst  );\
        e_PID_TUNING_AXIS  item_axis;\
        if( ppid_tuning_axis_GET( src, &item_axis ) ){\
            DST##_axis_SET( item_axis , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_desired_GET( <SRC> * src ){}
static inline float <SRC>_achieved_GET( <SRC> * src ){}
static inline float <SRC>_FF_GET( <SRC> * src ){}
static inline float <SRC>_P_GET( <SRC> * src ){}
static inline float <SRC>_I_GET( <SRC> * src ){}
static inline float <SRC>_D_GET( <SRC> * src ){}
static inline bool  <SRC>_axis_item_exist( <SRC> * src ){}
static inline e_PID_TUNING_AXIS <SRC>_axis_GET( <SRC> * src ){}
*/

#define ppid_tuning_PID_TUNING_PULL_FROM(SRC)\
    static inline void ppid_tuning_PID_TUNING_pull_from_##SRC ( SRC * src, ppid_tuning_PID_TUNING * dst) {\
        ppid_tuning_desired_SET( SRC##_desired_GET(src ), dst  );\
        ppid_tuning_achieved_SET( SRC##_achieved_GET(src ), dst  );\
        ppid_tuning_FF_SET( SRC##_FF_GET(src ), dst  );\
        ppid_tuning_P_SET( SRC##_P_GET(src ), dst  );\
        ppid_tuning_I_SET( SRC##_I_GET(src ), dst  );\
        ppid_tuning_D_SET( SRC##_D_GET(src ), dst  );\
        if( SRC##_axis_item_exist(src ) )\
        ppid_tuning_axis_SET( ppid_tuning_PID_TUNING_axis_GET( src ),  dst  );\
    }

/**
*Read out the safety zone the MAV currently assumes. */

typedef Pack SAFETY_ALLOWED_AREA_safety_allowed_area; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor psafety_allowed_area_SAFETY_ALLOWED_AREA;// data navigator over pack fields data
/**
															* Wrap SAFETY_ALLOWED_AREA in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline psafety_allowed_area_SAFETY_ALLOWED_AREA *SAFETY_ALLOWED_AREA_safety_allowed_area_wrap(SAFETY_ALLOWED_AREA_safety_allowed_area *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SAFETY_ALLOWED_AREA_safety_allowed_area *psafety_allowed_area_SAFETY_ALLOWED_AREA_unwrap(psafety_allowed_area_SAFETY_ALLOWED_AREA *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_p1x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p1y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p1z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p2x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p2y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_p2z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
*/

#define psafety_allowed_area_SAFETY_ALLOWED_AREA_PUSH_INTO(DST)\
    static inline void psafety_allowed_area_SAFETY_ALLOWED_AREA_push_into_##DST ( psafety_allowed_area_SAFETY_ALLOWED_AREA * src, DST * dst) {\
        DST##_p1x_SET( psafety_allowed_area_p1x_GET( src  ), dst  );\
        DST##_p1y_SET( psafety_allowed_area_p1y_GET( src  ), dst  );\
        DST##_p1z_SET( psafety_allowed_area_p1z_GET( src  ), dst  );\
        DST##_p2x_SET( psafety_allowed_area_p2x_GET( src  ), dst  );\
        DST##_p2y_SET( psafety_allowed_area_p2y_GET( src  ), dst  );\
        DST##_p2z_SET( psafety_allowed_area_p2z_GET( src  ), dst  );\
        e_MAV_FRAME  item_frame;\
        if( psafety_allowed_area_frame_GET( src, &item_frame ) ){\
            DST##_frame_SET( item_frame , dst  );\
        }\
    }

/**
*Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor) */

typedef Pack OPTICAL_FLOW_RAD_optical_flow_rad; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t poptical_flow_rad_OPTICAL_FLOW_RAD;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline poptical_flow_rad_OPTICAL_FLOW_RAD *poptical_flow_rad_OPTICAL_FLOW_RAD_from(OPTICAL_FLOW_RAD_optical_flow_rad *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_integration_time_us_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_delta_distance_us_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_sensor_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_integrated_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_integrated_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_integrated_xgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_integrated_ygyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_integrated_zgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_quality_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_distance_SET( float * src, <DST> * dst  ){}
*/

#define poptical_flow_rad_OPTICAL_FLOW_RAD_PUSH_INTO(DST)\
    static inline void poptical_flow_rad_OPTICAL_FLOW_RAD_push_into_##DST ( poptical_flow_rad_OPTICAL_FLOW_RAD * src, DST * dst) {\
        DST##_integration_time_us_SET( poptical_flow_rad_integration_time_us_GET( src  ), dst  );\
        DST##_time_delta_distance_us_SET( poptical_flow_rad_time_delta_distance_us_GET( src  ), dst  );\
        DST##_time_usec_SET( poptical_flow_rad_time_usec_GET( src  ), dst  );\
        DST##_sensor_id_SET( poptical_flow_rad_sensor_id_GET( src  ), dst  );\
        DST##_integrated_x_SET( poptical_flow_rad_integrated_x_GET( src  ), dst  );\
        DST##_integrated_y_SET( poptical_flow_rad_integrated_y_GET( src  ), dst  );\
        DST##_integrated_xgyro_SET( poptical_flow_rad_integrated_xgyro_GET( src  ), dst  );\
        DST##_integrated_ygyro_SET( poptical_flow_rad_integrated_ygyro_GET( src  ), dst  );\
        DST##_integrated_zgyro_SET( poptical_flow_rad_integrated_zgyro_GET( src  ), dst  );\
        DST##_temperature_SET( poptical_flow_rad_temperature_GET( src  ), dst  );\
        DST##_quality_SET( poptical_flow_rad_quality_GET( src  ), dst  );\
        DST##_distance_SET( poptical_flow_rad_distance_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_integration_time_us_GET( <SRC> * src ){}
static inline int32_t <SRC>_time_delta_distance_us_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_sensor_id_GET( <SRC> * src ){}
static inline float <SRC>_integrated_x_GET( <SRC> * src ){}
static inline float <SRC>_integrated_y_GET( <SRC> * src ){}
static inline float <SRC>_integrated_xgyro_GET( <SRC> * src ){}
static inline float <SRC>_integrated_ygyro_GET( <SRC> * src ){}
static inline float <SRC>_integrated_zgyro_GET( <SRC> * src ){}
static inline int16_t <SRC>_temperature_GET( <SRC> * src ){}
static inline int8_t <SRC>_quality_GET( <SRC> * src ){}
static inline float <SRC>_distance_GET( <SRC> * src ){}
*/

#define poptical_flow_rad_OPTICAL_FLOW_RAD_PULL_FROM(SRC)\
    static inline void poptical_flow_rad_OPTICAL_FLOW_RAD_pull_from_##SRC ( SRC * src, poptical_flow_rad_OPTICAL_FLOW_RAD * dst) {\
        poptical_flow_rad_integration_time_us_SET( SRC##_integration_time_us_GET(src ), dst  );\
        poptical_flow_rad_time_delta_distance_us_SET( SRC##_time_delta_distance_us_GET(src ), dst  );\
        poptical_flow_rad_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        poptical_flow_rad_sensor_id_SET( SRC##_sensor_id_GET(src ), dst  );\
        poptical_flow_rad_integrated_x_SET( SRC##_integrated_x_GET(src ), dst  );\
        poptical_flow_rad_integrated_y_SET( SRC##_integrated_y_GET(src ), dst  );\
        poptical_flow_rad_integrated_xgyro_SET( SRC##_integrated_xgyro_GET(src ), dst  );\
        poptical_flow_rad_integrated_ygyro_SET( SRC##_integrated_ygyro_GET(src ), dst  );\
        poptical_flow_rad_integrated_zgyro_SET( SRC##_integrated_zgyro_GET(src ), dst  );\
        poptical_flow_rad_temperature_SET( SRC##_temperature_GET(src ), dst  );\
        poptical_flow_rad_quality_SET( SRC##_quality_GET(src ), dst  );\
        poptical_flow_rad_distance_SET( SRC##_distance_GET(src ), dst  );\
    }

/**
*Reply to LOG_REQUEST_DATA */

typedef Pack LOG_DATA_log_data; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plog_data_LOG_DATA;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plog_data_LOG_DATA *plog_data_LOG_DATA_from(LOG_DATA_log_data *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vlog_data_daTa;
//Maximum field array length constant
#define Plog_data_daTa_len  ( 90 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_id_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ofs_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_count_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vlog_data_daTa * src, <DST> * dst  ){}
*/

#define plog_data_LOG_DATA_PUSH_INTO(DST)\
    static inline void plog_data_LOG_DATA_push_into_##DST ( plog_data_LOG_DATA * src, DST * dst) {\
        DST##_id_SET( plog_data_id_GET( src  ), dst  );\
        DST##_ofs_SET( plog_data_ofs_GET( src  ), dst  );\
        DST##_count_SET( plog_data_count_GET( src  ), dst  );\
        Vlog_data_daTa item_daTa = plog_data_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_id_GET( <SRC> * src ){}
static inline int32_t <SRC>_ofs_GET( <SRC> * src ){}
static inline int8_t <SRC>_count_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vlog_data_daTa * dst ){}
*/

#define plog_data_LOG_DATA_PULL_FROM(SRC)\
    static inline void plog_data_LOG_DATA_pull_from_##SRC ( SRC * src, plog_data_LOG_DATA * dst) {\
        plog_data_id_SET( SRC##_id_GET(src ), dst  );\
        plog_data_ofs_SET( SRC##_ofs_GET(src ), dst  );\
        plog_data_count_SET( SRC##_count_GET(src ), dst  );\
       Vlog_data_daTa item_daTa = plog_data_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*Delete all mission items at once. */

typedef Pack MISSION_CLEAR_ALL_mission_clear_all; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_clear_all_MISSION_CLEAR_ALL;// data navigator over pack fields data
/**
															* Wrap MISSION_CLEAR_ALL in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_clear_all_MISSION_CLEAR_ALL *MISSION_CLEAR_ALL_mission_clear_all_wrap(MISSION_CLEAR_ALL_mission_clear_all *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_CLEAR_ALL_mission_clear_all *pmission_clear_all_MISSION_CLEAR_ALL_unwrap(pmission_clear_all_MISSION_CLEAR_ALL *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_clear_all_MISSION_CLEAR_ALL_PUSH_INTO(DST)\
    static inline void pmission_clear_all_MISSION_CLEAR_ALL_push_into_##DST ( pmission_clear_all_MISSION_CLEAR_ALL * src, DST * dst) {\
        DST##_target_system_SET( pmission_clear_all_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_clear_all_target_component_GET( src  ), dst  );\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_clear_all_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*Status of third AHRS filter if available. This is for ANU research group (Ali and Sean) */

typedef Pack AHRS3_ahrs3; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pahrs3_AHRS3;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pahrs3_AHRS3 *pahrs3_AHRS3_from(AHRS3_ahrs3 *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_altitude_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lng_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_v1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_v2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_v3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_v4_SET( float * src, <DST> * dst  ){}
*/

#define pahrs3_AHRS3_PUSH_INTO(DST)\
    static inline void pahrs3_AHRS3_push_into_##DST ( pahrs3_AHRS3 * src, DST * dst) {\
        DST##_roll_SET( pahrs3_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pahrs3_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pahrs3_yaw_GET( src  ), dst  );\
        DST##_altitude_SET( pahrs3_altitude_GET( src  ), dst  );\
        DST##_lat_SET( pahrs3_lat_GET( src  ), dst  );\
        DST##_lng_SET( pahrs3_lng_GET( src  ), dst  );\
        DST##_v1_SET( pahrs3_v1_GET( src  ), dst  );\
        DST##_v2_SET( pahrs3_v2_GET( src  ), dst  );\
        DST##_v3_SET( pahrs3_v3_GET( src  ), dst  );\
        DST##_v4_SET( pahrs3_v4_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_roll_GET( <SRC> * src ){}
static inline float <SRC>_pitch_GET( <SRC> * src ){}
static inline float <SRC>_yaw_GET( <SRC> * src ){}
static inline float <SRC>_altitude_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lng_GET( <SRC> * src ){}
static inline float <SRC>_v1_GET( <SRC> * src ){}
static inline float <SRC>_v2_GET( <SRC> * src ){}
static inline float <SRC>_v3_GET( <SRC> * src ){}
static inline float <SRC>_v4_GET( <SRC> * src ){}
*/

#define pahrs3_AHRS3_PULL_FROM(SRC)\
    static inline void pahrs3_AHRS3_pull_from_##SRC ( SRC * src, pahrs3_AHRS3 * dst) {\
        pahrs3_roll_SET( SRC##_roll_GET(src ), dst  );\
        pahrs3_pitch_SET( SRC##_pitch_GET(src ), dst  );\
        pahrs3_yaw_SET( SRC##_yaw_GET(src ), dst  );\
        pahrs3_altitude_SET( SRC##_altitude_GET(src ), dst  );\
        pahrs3_lat_SET( SRC##_lat_GET(src ), dst  );\
        pahrs3_lng_SET( SRC##_lng_GET(src ), dst  );\
        pahrs3_v1_SET( SRC##_v1_GET(src ), dst  );\
        pahrs3_v2_SET( SRC##_v2_GET(src ), dst  );\
        pahrs3_v3_SET( SRC##_v3_GET(src ), dst  );\
        pahrs3_v4_SET( SRC##_v4_GET(src ), dst  );\
    }


typedef Pack VICON_POSITION_ESTIMATE_vicon_position_estimate; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pvicon_position_estimate_VICON_POSITION_ESTIMATE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pvicon_position_estimate_VICON_POSITION_ESTIMATE *pvicon_position_estimate_VICON_POSITION_ESTIMATE_from(VICON_POSITION_ESTIMATE_vicon_position_estimate *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
*/

#define pvicon_position_estimate_VICON_POSITION_ESTIMATE_PUSH_INTO(DST)\
    static inline void pvicon_position_estimate_VICON_POSITION_ESTIMATE_push_into_##DST ( pvicon_position_estimate_VICON_POSITION_ESTIMATE * src, DST * dst) {\
        DST##_usec_SET( pvicon_position_estimate_usec_GET( src  ), dst  );\
        DST##_x_SET( pvicon_position_estimate_x_GET( src  ), dst  );\
        DST##_y_SET( pvicon_position_estimate_y_GET( src  ), dst  );\
        DST##_z_SET( pvicon_position_estimate_z_GET( src  ), dst  );\
        DST##_roll_SET( pvicon_position_estimate_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pvicon_position_estimate_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pvicon_position_estimate_yaw_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_usec_GET( <SRC> * src ){}
static inline float <SRC>_x_GET( <SRC> * src ){}
static inline float <SRC>_y_GET( <SRC> * src ){}
static inline float <SRC>_z_GET( <SRC> * src ){}
static inline float <SRC>_roll_GET( <SRC> * src ){}
static inline float <SRC>_pitch_GET( <SRC> * src ){}
static inline float <SRC>_yaw_GET( <SRC> * src ){}
*/

#define pvicon_position_estimate_VICON_POSITION_ESTIMATE_PULL_FROM(SRC)\
    static inline void pvicon_position_estimate_VICON_POSITION_ESTIMATE_pull_from_##SRC ( SRC * src, pvicon_position_estimate_VICON_POSITION_ESTIMATE * dst) {\
        pvicon_position_estimate_usec_SET( SRC##_usec_GET(src ), dst  );\
        pvicon_position_estimate_x_SET( SRC##_x_GET(src ), dst  );\
        pvicon_position_estimate_y_SET( SRC##_y_GET(src ), dst  );\
        pvicon_position_estimate_z_SET( SRC##_z_GET(src ), dst  );\
        pvicon_position_estimate_roll_SET( SRC##_roll_GET(src ), dst  );\
        pvicon_position_estimate_pitch_SET( SRC##_pitch_GET(src ), dst  );\
        pvicon_position_estimate_yaw_SET( SRC##_yaw_GET(src ), dst  );\
    }

/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting */

typedef Pack GPS2_RTK_gps2_rtk; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pgps2_rtk_GPS2_RTK;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pgps2_rtk_GPS2_RTK *pgps2_rtk_GPS2_RTK_from(GPS2_RTK_gps2_rtk *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_wn_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_last_baseline_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_tow_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_accuracy_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_rtk_receiver_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_rtk_health_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_rtk_rate_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_nsats_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_baseline_coords_type_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_baseline_a_mm_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_baseline_b_mm_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_baseline_c_mm_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_iar_num_hypotheses_SET( int32_t * src, <DST> * dst  ){}
*/

#define pgps2_rtk_GPS2_RTK_PUSH_INTO(DST)\
    static inline void pgps2_rtk_GPS2_RTK_push_into_##DST ( pgps2_rtk_GPS2_RTK * src, DST * dst) {\
        DST##_wn_SET( pgps2_rtk_wn_GET( src  ), dst  );\
        DST##_time_last_baseline_ms_SET( pgps2_rtk_time_last_baseline_ms_GET( src  ), dst  );\
        DST##_tow_SET( pgps2_rtk_tow_GET( src  ), dst  );\
        DST##_accuracy_SET( pgps2_rtk_accuracy_GET( src  ), dst  );\
        DST##_rtk_receiver_id_SET( pgps2_rtk_rtk_receiver_id_GET( src  ), dst  );\
        DST##_rtk_health_SET( pgps2_rtk_rtk_health_GET( src  ), dst  );\
        DST##_rtk_rate_SET( pgps2_rtk_rtk_rate_GET( src  ), dst  );\
        DST##_nsats_SET( pgps2_rtk_nsats_GET( src  ), dst  );\
        DST##_baseline_coords_type_SET( pgps2_rtk_baseline_coords_type_GET( src  ), dst  );\
        DST##_baseline_a_mm_SET( pgps2_rtk_baseline_a_mm_GET( src  ), dst  );\
        DST##_baseline_b_mm_SET( pgps2_rtk_baseline_b_mm_GET( src  ), dst  );\
        DST##_baseline_c_mm_SET( pgps2_rtk_baseline_c_mm_GET( src  ), dst  );\
        DST##_iar_num_hypotheses_SET( pgps2_rtk_iar_num_hypotheses_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_wn_GET( <SRC> * src ){}
static inline int32_t <SRC>_time_last_baseline_ms_GET( <SRC> * src ){}
static inline int32_t <SRC>_tow_GET( <SRC> * src ){}
static inline int32_t <SRC>_accuracy_GET( <SRC> * src ){}
static inline int8_t <SRC>_rtk_receiver_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_rtk_health_GET( <SRC> * src ){}
static inline int8_t <SRC>_rtk_rate_GET( <SRC> * src ){}
static inline int8_t <SRC>_nsats_GET( <SRC> * src ){}
static inline int8_t <SRC>_baseline_coords_type_GET( <SRC> * src ){}
static inline int32_t <SRC>_baseline_a_mm_GET( <SRC> * src ){}
static inline int32_t <SRC>_baseline_b_mm_GET( <SRC> * src ){}
static inline int32_t <SRC>_baseline_c_mm_GET( <SRC> * src ){}
static inline int32_t <SRC>_iar_num_hypotheses_GET( <SRC> * src ){}
*/

#define pgps2_rtk_GPS2_RTK_PULL_FROM(SRC)\
    static inline void pgps2_rtk_GPS2_RTK_pull_from_##SRC ( SRC * src, pgps2_rtk_GPS2_RTK * dst) {\
        pgps2_rtk_wn_SET( SRC##_wn_GET(src ), dst  );\
        pgps2_rtk_time_last_baseline_ms_SET( SRC##_time_last_baseline_ms_GET(src ), dst  );\
        pgps2_rtk_tow_SET( SRC##_tow_GET(src ), dst  );\
        pgps2_rtk_accuracy_SET( SRC##_accuracy_GET(src ), dst  );\
        pgps2_rtk_rtk_receiver_id_SET( SRC##_rtk_receiver_id_GET(src ), dst  );\
        pgps2_rtk_rtk_health_SET( SRC##_rtk_health_GET(src ), dst  );\
        pgps2_rtk_rtk_rate_SET( SRC##_rtk_rate_GET(src ), dst  );\
        pgps2_rtk_nsats_SET( SRC##_nsats_GET(src ), dst  );\
        pgps2_rtk_baseline_coords_type_SET( SRC##_baseline_coords_type_GET(src ), dst  );\
        pgps2_rtk_baseline_a_mm_SET( SRC##_baseline_a_mm_GET(src ), dst  );\
        pgps2_rtk_baseline_b_mm_SET( SRC##_baseline_b_mm_GET(src ), dst  );\
        pgps2_rtk_baseline_c_mm_SET( SRC##_baseline_c_mm_GET(src ), dst  );\
        pgps2_rtk_iar_num_hypotheses_SET( SRC##_iar_num_hypotheses_GET(src ), dst  );\
    }

/**
*Reports results of completed compass calibration. Sent until MAG_CAL_ACK received. */

typedef Pack MAG_CAL_REPORT_mag_cal_report; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmag_cal_report_MAG_CAL_REPORT;// data navigator over pack fields data
/**
															* Wrap MAG_CAL_REPORT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmag_cal_report_MAG_CAL_REPORT *MAG_CAL_REPORT_mag_cal_report_wrap(MAG_CAL_REPORT_mag_cal_report *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MAG_CAL_REPORT_mag_cal_report *pmag_cal_report_MAG_CAL_REPORT_unwrap(pmag_cal_report_MAG_CAL_REPORT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_compass_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_cal_mask_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_autosaved_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_fitness_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ofs_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ofs_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ofs_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_diag_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_diag_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_diag_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_offdiag_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_offdiag_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_offdiag_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_cal_status_SET( e_MAG_CAL_STATUS * src, <DST> * dst  ){}
*/

#define pmag_cal_report_MAG_CAL_REPORT_PUSH_INTO(DST)\
    static inline void pmag_cal_report_MAG_CAL_REPORT_push_into_##DST ( pmag_cal_report_MAG_CAL_REPORT * src, DST * dst) {\
        DST##_compass_id_SET( pmag_cal_report_compass_id_GET( src  ), dst  );\
        DST##_cal_mask_SET( pmag_cal_report_cal_mask_GET( src  ), dst  );\
        DST##_autosaved_SET( pmag_cal_report_autosaved_GET( src  ), dst  );\
        DST##_fitness_SET( pmag_cal_report_fitness_GET( src  ), dst  );\
        DST##_ofs_x_SET( pmag_cal_report_ofs_x_GET( src  ), dst  );\
        DST##_ofs_y_SET( pmag_cal_report_ofs_y_GET( src  ), dst  );\
        DST##_ofs_z_SET( pmag_cal_report_ofs_z_GET( src  ), dst  );\
        DST##_diag_x_SET( pmag_cal_report_diag_x_GET( src  ), dst  );\
        DST##_diag_y_SET( pmag_cal_report_diag_y_GET( src  ), dst  );\
        DST##_diag_z_SET( pmag_cal_report_diag_z_GET( src  ), dst  );\
        DST##_offdiag_x_SET( pmag_cal_report_offdiag_x_GET( src  ), dst  );\
        DST##_offdiag_y_SET( pmag_cal_report_offdiag_y_GET( src  ), dst  );\
        DST##_offdiag_z_SET( pmag_cal_report_offdiag_z_GET( src  ), dst  );\
        e_MAG_CAL_STATUS  item_cal_status;\
        if( pmag_cal_report_cal_status_GET( src, &item_cal_status ) ){\
            DST##_cal_status_SET( item_cal_status , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_compass_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_cal_mask_GET( <SRC> * src ){}
static inline int8_t <SRC>_autosaved_GET( <SRC> * src ){}
static inline float <SRC>_fitness_GET( <SRC> * src ){}
static inline float <SRC>_ofs_x_GET( <SRC> * src ){}
static inline float <SRC>_ofs_y_GET( <SRC> * src ){}
static inline float <SRC>_ofs_z_GET( <SRC> * src ){}
static inline float <SRC>_diag_x_GET( <SRC> * src ){}
static inline float <SRC>_diag_y_GET( <SRC> * src ){}
static inline float <SRC>_diag_z_GET( <SRC> * src ){}
static inline float <SRC>_offdiag_x_GET( <SRC> * src ){}
static inline float <SRC>_offdiag_y_GET( <SRC> * src ){}
static inline float <SRC>_offdiag_z_GET( <SRC> * src ){}
static inline bool  <SRC>_cal_status_item_exist( <SRC> * src ){}
static inline e_MAG_CAL_STATUS <SRC>_cal_status_GET( <SRC> * src ){}
*/

#define pmag_cal_report_MAG_CAL_REPORT_PULL_FROM(SRC)\
    static inline void pmag_cal_report_MAG_CAL_REPORT_pull_from_##SRC ( SRC * src, pmag_cal_report_MAG_CAL_REPORT * dst) {\
        pmag_cal_report_compass_id_SET( SRC##_compass_id_GET(src ), dst  );\
        pmag_cal_report_cal_mask_SET( SRC##_cal_mask_GET(src ), dst  );\
        pmag_cal_report_autosaved_SET( SRC##_autosaved_GET(src ), dst  );\
        pmag_cal_report_fitness_SET( SRC##_fitness_GET(src ), dst  );\
        pmag_cal_report_ofs_x_SET( SRC##_ofs_x_GET(src ), dst  );\
        pmag_cal_report_ofs_y_SET( SRC##_ofs_y_GET(src ), dst  );\
        pmag_cal_report_ofs_z_SET( SRC##_ofs_z_GET(src ), dst  );\
        pmag_cal_report_diag_x_SET( SRC##_diag_x_GET(src ), dst  );\
        pmag_cal_report_diag_y_SET( SRC##_diag_y_GET(src ), dst  );\
        pmag_cal_report_diag_z_SET( SRC##_diag_z_GET(src ), dst  );\
        pmag_cal_report_offdiag_x_SET( SRC##_offdiag_x_GET(src ), dst  );\
        pmag_cal_report_offdiag_y_SET( SRC##_offdiag_y_GET(src ), dst  );\
        pmag_cal_report_offdiag_z_SET( SRC##_offdiag_z_GET(src ), dst  );\
        if( SRC##_cal_status_item_exist(src ) )\
        pmag_cal_report_cal_status_SET( pmag_cal_report_MAG_CAL_REPORT_cal_status_GET( src ),  dst  );\
    }

/**
*Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
*				 is called */

typedef Pack LOG_REQUEST_LIST_log_request_list; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plog_request_list_LOG_REQUEST_LIST;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plog_request_list_LOG_REQUEST_LIST *plog_request_list_LOG_REQUEST_LIST_from(LOG_REQUEST_LIST_log_request_list *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_start_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_end_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define plog_request_list_LOG_REQUEST_LIST_PUSH_INTO(DST)\
    static inline void plog_request_list_LOG_REQUEST_LIST_push_into_##DST ( plog_request_list_LOG_REQUEST_LIST * src, DST * dst) {\
        DST##_start_SET( plog_request_list_start_GET( src  ), dst  );\
        DST##_end_SET( plog_request_list_end_GET( src  ), dst  );\
        DST##_target_system_SET( plog_request_list_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( plog_request_list_target_component_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_start_GET( <SRC> * src ){}
static inline int16_t <SRC>_end_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
*/

#define plog_request_list_LOG_REQUEST_LIST_PULL_FROM(SRC)\
    static inline void plog_request_list_LOG_REQUEST_LIST_pull_from_##SRC ( SRC * src, plog_request_list_LOG_REQUEST_LIST * dst) {\
        plog_request_list_start_SET( SRC##_start_GET(src ), dst  );\
        plog_request_list_end_SET( SRC##_end_GET(src ), dst  );\
        plog_request_list_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        plog_request_list_target_component_SET( SRC##_target_component_GET(src ), dst  );\
    }

/**
*The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
*				 are as specified in each field */

typedef Pack SCALED_PRESSURE_scaled_pressure; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pscaled_pressure_SCALED_PRESSURE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pscaled_pressure_SCALED_PRESSURE *pscaled_pressure_SCALED_PRESSURE_from(SCALED_PRESSURE_scaled_pressure *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_press_abs_SET( float * src, <DST> * dst  ){}
static inline void <DST>_press_diff_SET( float * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( int16_t * src, <DST> * dst  ){}
*/

#define pscaled_pressure_SCALED_PRESSURE_PUSH_INTO(DST)\
    static inline void pscaled_pressure_SCALED_PRESSURE_push_into_##DST ( pscaled_pressure_SCALED_PRESSURE * src, DST * dst) {\
        DST##_time_boot_ms_SET( pscaled_pressure_time_boot_ms_GET( src  ), dst  );\
        DST##_press_abs_SET( pscaled_pressure_press_abs_GET( src  ), dst  );\
        DST##_press_diff_SET( pscaled_pressure_press_diff_GET( src  ), dst  );\
        DST##_temperature_SET( pscaled_pressure_temperature_GET( src  ), dst  );\
    }

/**
*Message implementing parts of the V2 payload specs in V1 frames for transitional support. */

typedef Pack V2_EXTENSION_v2_extension; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pv2_extension_V2_EXTENSION;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pv2_extension_V2_EXTENSION *pv2_extension_V2_EXTENSION_from(V2_EXTENSION_v2_extension *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vv2_extension_payload;
//Maximum field array length constant
#define Pv2_extension_payload_len  ( 249 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_message_type_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_network_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_payload_SET( Vv2_extension_payload * src, <DST> * dst  ){}
*/

#define pv2_extension_V2_EXTENSION_PUSH_INTO(DST)\
    static inline void pv2_extension_V2_EXTENSION_push_into_##DST ( pv2_extension_V2_EXTENSION * src, DST * dst) {\
        DST##_message_type_SET( pv2_extension_message_type_GET( src  ), dst  );\
        DST##_target_network_SET( pv2_extension_target_network_GET( src  ), dst  );\
        DST##_target_system_SET( pv2_extension_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pv2_extension_target_component_GET( src  ), dst  );\
        Vv2_extension_payload item_payload = pv2_extension_payload_GET( src  );\
       DST##_payload_SET( &item_payload, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_message_type_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_network_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_payload_GET( <SRC> * src, Vv2_extension_payload * dst ){}
*/

#define pv2_extension_V2_EXTENSION_PULL_FROM(SRC)\
    static inline void pv2_extension_V2_EXTENSION_pull_from_##SRC ( SRC * src, pv2_extension_V2_EXTENSION * dst) {\
        pv2_extension_message_type_SET( SRC##_message_type_GET(src ), dst  );\
        pv2_extension_target_network_SET( SRC##_target_network_GET(src ), dst  );\
        pv2_extension_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pv2_extension_target_component_SET( SRC##_target_component_GET(src ), dst  );\
       Vv2_extension_payload item_payload = pv2_extension_payload_SET( NULL, dst  );\
       SRC##_payload_GET( src, &item_payload );\
    }

/**
*The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot
*				 hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying
*				 out the user interface based on the autopilot) */

typedef Pack HEARTBEAT_heartbeat; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pheartbeat_HEARTBEAT;// data navigator over pack fields data
/**
															* Wrap HEARTBEAT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pheartbeat_HEARTBEAT *HEARTBEAT_heartbeat_wrap(HEARTBEAT_heartbeat *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline HEARTBEAT_heartbeat *pheartbeat_HEARTBEAT_unwrap(pheartbeat_HEARTBEAT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_custom_mode_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_mavlink_version_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_typE_SET( e_MAV_TYPE * src, <DST> * dst  ){}
static inline void <DST>_autopilot_SET( e_MAV_AUTOPILOT * src, <DST> * dst  ){}
static inline void <DST>_base_mode_SET( e_MAV_MODE_FLAG * src, <DST> * dst  ){}
static inline void <DST>_system_status_SET( e_MAV_STATE * src, <DST> * dst  ){}
*/

#define pheartbeat_HEARTBEAT_PUSH_INTO(DST)\
    static inline void pheartbeat_HEARTBEAT_push_into_##DST ( pheartbeat_HEARTBEAT * src, DST * dst) {\
        DST##_custom_mode_SET( pheartbeat_custom_mode_GET( src  ), dst  );\
        DST##_mavlink_version_SET( pheartbeat_mavlink_version_GET( src  ), dst  );\
        e_MAV_TYPE  item_typE;\
        if( pheartbeat_typE_GET( src, &item_typE ) ){\
            DST##_typE_SET( item_typE , dst  );\
        }\
        e_MAV_AUTOPILOT  item_autopilot;\
        if( pheartbeat_autopilot_GET( src, &item_autopilot ) ){\
            DST##_autopilot_SET( item_autopilot , dst  );\
        }\
        e_MAV_MODE_FLAG  item_base_mode;\
        if( pheartbeat_base_mode_GET( src, &item_base_mode ) ){\
            DST##_base_mode_SET( item_base_mode , dst  );\
        }\
        e_MAV_STATE  item_system_status;\
        if( pheartbeat_system_status_GET( src, &item_system_status ) ){\
            DST##_system_status_SET( item_system_status , dst  );\
        }\
    }

/**
*Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value. */

typedef Pack PARAM_MAP_RC_param_map_rc; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pparam_map_rc_PARAM_MAP_RC;// data navigator over pack fields data
/**
															* Wrap PARAM_MAP_RC in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pparam_map_rc_PARAM_MAP_RC *PARAM_MAP_RC_param_map_rc_wrap(PARAM_MAP_RC_param_map_rc *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PARAM_MAP_RC_param_map_rc *pparam_map_rc_PARAM_MAP_RC_unwrap(pparam_map_rc_PARAM_MAP_RC *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_map_rc_param_id;
//Maximum field array length constant
#define Pparam_map_rc_param_id_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_parameter_rc_channel_index_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param_value0_SET( float * src, <DST> * dst  ){}
static inline void <DST>_scale_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param_value_min_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param_value_max_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param_id_SET( Vparam_map_rc_param_id * src, <DST> * dst  ){}
*/

#define pparam_map_rc_PARAM_MAP_RC_PUSH_INTO(DST)\
    static inline void pparam_map_rc_PARAM_MAP_RC_push_into_##DST ( pparam_map_rc_PARAM_MAP_RC * src, DST * dst) {\
        DST##_target_system_SET( pparam_map_rc_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pparam_map_rc_target_component_GET( src  ), dst  );\
        DST##_param_index_SET( pparam_map_rc_param_index_GET( src  ), dst  );\
        DST##_parameter_rc_channel_index_SET( pparam_map_rc_parameter_rc_channel_index_GET( src  ), dst  );\
        DST##_param_value0_SET( pparam_map_rc_param_value0_GET( src  ), dst  );\
        DST##_scale_SET( pparam_map_rc_scale_GET( src  ), dst  );\
        DST##_param_value_min_SET( pparam_map_rc_param_value_min_GET( src  ), dst  );\
        DST##_param_value_max_SET( pparam_map_rc_param_value_max_GET( src  ), dst  );\
        Vparam_map_rc_param_id  item_param_id;\
        if( pparam_map_rc_param_id_GET( src, &item_param_id ) ){\
            DST##_param_id_SET( &item_param_id, dst );\
        }\
    }

/**
*Power supply status */

typedef Pack POWER_STATUS_power_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor ppower_status_POWER_STATUS;// data navigator over pack fields data
/**
															* Wrap POWER_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline ppower_status_POWER_STATUS *POWER_STATUS_power_status_wrap(POWER_STATUS_power_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline POWER_STATUS_power_status *ppower_status_POWER_STATUS_unwrap(ppower_status_POWER_STATUS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_Vcc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_Vservo_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_MAV_POWER_STATUS * src, <DST> * dst  ){}
*/

#define ppower_status_POWER_STATUS_PUSH_INTO(DST)\
    static inline void ppower_status_POWER_STATUS_push_into_##DST ( ppower_status_POWER_STATUS * src, DST * dst) {\
        DST##_Vcc_SET( ppower_status_Vcc_GET( src  ), dst  );\
        DST##_Vservo_SET( ppower_status_Vservo_GET( src  ), dst  );\
        e_MAV_POWER_STATUS  item_flags;\
        if( ppower_status_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_Vcc_GET( <SRC> * src ){}
static inline int16_t <SRC>_Vservo_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_MAV_POWER_STATUS <SRC>_flags_GET( <SRC> * src ){}
*/

#define ppower_status_POWER_STATUS_PULL_FROM(SRC)\
    static inline void ppower_status_POWER_STATUS_pull_from_##SRC ( SRC * src, ppower_status_POWER_STATUS * dst) {\
        ppower_status_Vcc_SET( SRC##_Vcc_GET(src ), dst  );\
        ppower_status_Vservo_SET( SRC##_Vservo_GET(src ), dst  );\
        if( SRC##_flags_item_exist(src ) )\
        ppower_status_flags_SET( ppower_status_POWER_STATUS_flags_GET( src ),  dst  );\
    }

/**
*Send a block of log data to remote location */

typedef Pack REMOTE_LOG_DATA_BLOCK_remote_log_data_block; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor premote_log_data_block_REMOTE_LOG_DATA_BLOCK;// data navigator over pack fields data
/**
															* Wrap REMOTE_LOG_DATA_BLOCK in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline premote_log_data_block_REMOTE_LOG_DATA_BLOCK *REMOTE_LOG_DATA_BLOCK_remote_log_data_block_wrap(REMOTE_LOG_DATA_BLOCK_remote_log_data_block *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline REMOTE_LOG_DATA_BLOCK_remote_log_data_block *premote_log_data_block_REMOTE_LOG_DATA_BLOCK_unwrap(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vremote_log_data_block_daTa;
//Maximum field array length constant
#define Premote_log_data_block_daTa_len  ( 200 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vremote_log_data_block_daTa * src, <DST> * dst  ){}
static inline void <DST>_seqno_SET( e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS * src, <DST> * dst  ){}
*/

#define premote_log_data_block_REMOTE_LOG_DATA_BLOCK_PUSH_INTO(DST)\
    static inline void premote_log_data_block_REMOTE_LOG_DATA_BLOCK_push_into_##DST ( premote_log_data_block_REMOTE_LOG_DATA_BLOCK * src, DST * dst) {\
        DST##_target_system_SET( premote_log_data_block_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( premote_log_data_block_target_component_GET( src  ), dst  );\
        Vremote_log_data_block_daTa item_daTa = premote_log_data_block_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
        e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS  item_seqno;\
        if( premote_log_data_block_seqno_GET( src, &item_seqno ) ){\
            DST##_seqno_SET( item_seqno , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vremote_log_data_block_daTa * dst ){}
static inline bool  <SRC>_seqno_item_exist( <SRC> * src ){}
static inline e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS <SRC>_seqno_GET( <SRC> * src ){}
*/

#define premote_log_data_block_REMOTE_LOG_DATA_BLOCK_PULL_FROM(SRC)\
    static inline void premote_log_data_block_REMOTE_LOG_DATA_BLOCK_pull_from_##SRC ( SRC * src, premote_log_data_block_REMOTE_LOG_DATA_BLOCK * dst) {\
        premote_log_data_block_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        premote_log_data_block_target_component_SET( SRC##_target_component_GET(src ), dst  );\
       Vremote_log_data_block_daTa item_daTa = premote_log_data_block_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
        if( SRC##_seqno_item_exist(src ) )\
        premote_log_data_block_seqno_SET( premote_log_data_block_REMOTE_LOG_DATA_BLOCK_seqno_GET( src ),  dst  );\
    }

/**
*A message containing logged data which requires a LOGGING_ACK to be sent back */

typedef Pack LOGGING_DATA_ACKED_logging_data_acked; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plogging_data_acked_LOGGING_DATA_ACKED;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plogging_data_acked_LOGGING_DATA_ACKED *plogging_data_acked_LOGGING_DATA_ACKED_from(LOGGING_DATA_ACKED_logging_data_acked *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vlogging_data_acked_daTa;
//Maximum field array length constant
#define Plogging_data_acked_daTa_len  ( 249 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_sequence_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_length_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_first_message_offset_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vlogging_data_acked_daTa * src, <DST> * dst  ){}
*/

#define plogging_data_acked_LOGGING_DATA_ACKED_PUSH_INTO(DST)\
    static inline void plogging_data_acked_LOGGING_DATA_ACKED_push_into_##DST ( plogging_data_acked_LOGGING_DATA_ACKED * src, DST * dst) {\
        DST##_sequence_SET( plogging_data_acked_sequence_GET( src  ), dst  );\
        DST##_target_system_SET( plogging_data_acked_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( plogging_data_acked_target_component_GET( src  ), dst  );\
        DST##_length_SET( plogging_data_acked_length_GET( src  ), dst  );\
        DST##_first_message_offset_SET( plogging_data_acked_first_message_offset_GET( src  ), dst  );\
        Vlogging_data_acked_daTa item_daTa = plogging_data_acked_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_sequence_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_length_GET( <SRC> * src ){}
static inline int8_t <SRC>_first_message_offset_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vlogging_data_acked_daTa * dst ){}
*/

#define plogging_data_acked_LOGGING_DATA_ACKED_PULL_FROM(SRC)\
    static inline void plogging_data_acked_LOGGING_DATA_ACKED_pull_from_##SRC ( SRC * src, plogging_data_acked_LOGGING_DATA_ACKED * dst) {\
        plogging_data_acked_sequence_SET( SRC##_sequence_GET(src ), dst  );\
        plogging_data_acked_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        plogging_data_acked_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        plogging_data_acked_length_SET( SRC##_length_GET(src ), dst  );\
        plogging_data_acked_first_message_offset_SET( SRC##_first_message_offset_GET(src ), dst  );\
       Vlogging_data_acked_daTa item_daTa = plogging_data_acked_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
*				 has all terrain data needed for a mission */

typedef Pack TERRAIN_CHECK_terrain_check; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pterrain_check_TERRAIN_CHECK;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pterrain_check_TERRAIN_CHECK *pterrain_check_TERRAIN_CHECK_from(TERRAIN_CHECK_terrain_check *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
*/

#define pterrain_check_TERRAIN_CHECK_PUSH_INTO(DST)\
    static inline void pterrain_check_TERRAIN_CHECK_push_into_##DST ( pterrain_check_TERRAIN_CHECK * src, DST * dst) {\
        DST##_lat_SET( pterrain_check_lat_GET( src  ), dst  );\
        DST##_lon_SET( pterrain_check_lon_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
*/

#define pterrain_check_TERRAIN_CHECK_PULL_FROM(SRC)\
    static inline void pterrain_check_TERRAIN_CHECK_pull_from_##SRC ( SRC * src, pterrain_check_TERRAIN_CHECK * dst) {\
        pterrain_check_lat_SET( SRC##_lat_GET(src ), dst  );\
        pterrain_check_lon_SET( SRC##_lon_GET(src ), dst  );\
    }

/**
*Message to configure a camera mount, directional antenna, etc. */

typedef Pack MOUNT_CONFIGURE_mount_configure; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmount_configure_MOUNT_CONFIGURE;// data navigator over pack fields data
/**
															* Wrap MOUNT_CONFIGURE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmount_configure_MOUNT_CONFIGURE *MOUNT_CONFIGURE_mount_configure_wrap(MOUNT_CONFIGURE_mount_configure *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MOUNT_CONFIGURE_mount_configure *pmount_configure_MOUNT_CONFIGURE_unwrap(pmount_configure_MOUNT_CONFIGURE *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_stab_roll_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_stab_pitch_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_stab_yaw_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mount_mode_SET( e_MAV_MOUNT_MODE * src, <DST> * dst  ){}
*/

#define pmount_configure_MOUNT_CONFIGURE_PUSH_INTO(DST)\
    static inline void pmount_configure_MOUNT_CONFIGURE_push_into_##DST ( pmount_configure_MOUNT_CONFIGURE * src, DST * dst) {\
        DST##_target_system_SET( pmount_configure_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmount_configure_target_component_GET( src  ), dst  );\
        DST##_stab_roll_SET( pmount_configure_stab_roll_GET( src  ), dst  );\
        DST##_stab_pitch_SET( pmount_configure_stab_pitch_GET( src  ), dst  );\
        DST##_stab_yaw_SET( pmount_configure_stab_yaw_GET( src  ), dst  );\
        e_MAV_MOUNT_MODE  item_mount_mode;\
        if( pmount_configure_mount_mode_GET( src, &item_mount_mode ) ){\
            DST##_mount_mode_SET( item_mount_mode , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_stab_roll_GET( <SRC> * src ){}
static inline int8_t <SRC>_stab_pitch_GET( <SRC> * src ){}
static inline int8_t <SRC>_stab_yaw_GET( <SRC> * src ){}
static inline bool  <SRC>_mount_mode_item_exist( <SRC> * src ){}
static inline e_MAV_MOUNT_MODE <SRC>_mount_mode_GET( <SRC> * src ){}
*/

#define pmount_configure_MOUNT_CONFIGURE_PULL_FROM(SRC)\
    static inline void pmount_configure_MOUNT_CONFIGURE_pull_from_##SRC ( SRC * src, pmount_configure_MOUNT_CONFIGURE * dst) {\
        pmount_configure_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pmount_configure_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pmount_configure_stab_roll_SET( SRC##_stab_roll_GET(src ), dst  );\
        pmount_configure_stab_pitch_SET( SRC##_stab_pitch_GET(src ), dst  );\
        pmount_configure_stab_yaw_SET( SRC##_stab_yaw_GET(src ), dst  );\
        if( SRC##_mount_mode_item_exist(src ) )\
        pmount_configure_mount_mode_SET( pmount_configure_MOUNT_CONFIGURE_mount_mode_GET( src ),  dst  );\
    }

/**
*Request the information of the mission item with the sequence number seq. The response of the system to
*				 this message should be a MISSION_ITEM_INT message. http:qgroundcontrol.org/mavlink/waypoint_protoco */

typedef Pack MISSION_REQUEST_INT_mission_request_int; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_request_int_MISSION_REQUEST_INT;// data navigator over pack fields data
/**
															* Wrap MISSION_REQUEST_INT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_request_int_MISSION_REQUEST_INT *MISSION_REQUEST_INT_mission_request_int_wrap(MISSION_REQUEST_INT_mission_request_int *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_REQUEST_INT_mission_request_int *pmission_request_int_MISSION_REQUEST_INT_unwrap(pmission_request_int_MISSION_REQUEST_INT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_request_int_MISSION_REQUEST_INT_PUSH_INTO(DST)\
    static inline void pmission_request_int_MISSION_REQUEST_INT_push_into_##DST ( pmission_request_int_MISSION_REQUEST_INT * src, DST * dst) {\
        DST##_seq_SET( pmission_request_int_seq_GET( src  ), dst  );\
        DST##_target_system_SET( pmission_request_int_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_request_int_target_component_GET( src  ), dst  );\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_request_int_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
*				 frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
*				 convention */

typedef Pack LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_local_position_ned_system_global_offset; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_from(LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_local_position_ned_system_global_offset *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
*/

#define plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PUSH_INTO(DST)\
    static inline void plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_push_into_##DST ( plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET * src, DST * dst) {\
        DST##_time_boot_ms_SET( plocal_position_ned_system_global_offset_time_boot_ms_GET( src  ), dst  );\
        DST##_x_SET( plocal_position_ned_system_global_offset_x_GET( src  ), dst  );\
        DST##_y_SET( plocal_position_ned_system_global_offset_y_GET( src  ), dst  );\
        DST##_z_SET( plocal_position_ned_system_global_offset_z_GET( src  ), dst  );\
        DST##_roll_SET( plocal_position_ned_system_global_offset_roll_GET( src  ), dst  );\
        DST##_pitch_SET( plocal_position_ned_system_global_offset_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( plocal_position_ned_system_global_offset_yaw_GET( src  ), dst  );\
    }

/**
*Report status of a command. Includes feedback whether the command was executed. */

typedef Pack COMMAND_ACK_command_ack; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcommand_ack_COMMAND_ACK;// data navigator over pack fields data
/**
															* Wrap COMMAND_ACK in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcommand_ack_COMMAND_ACK *COMMAND_ACK_command_ack_wrap(COMMAND_ACK_command_ack *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline COMMAND_ACK_command_ack *pcommand_ack_COMMAND_ACK_unwrap(pcommand_ack_COMMAND_ACK *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_command_SET( e_MAV_CMD * src, <DST> * dst  ){}
static inline void <DST>_result_SET( e_MAV_RESULT * src, <DST> * dst  ){}
static inline void <DST>_progress_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_result_param2_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define pcommand_ack_COMMAND_ACK_PUSH_INTO(DST)\
    static inline void pcommand_ack_COMMAND_ACK_push_into_##DST ( pcommand_ack_COMMAND_ACK * src, DST * dst) {\
        e_MAV_CMD  item_command;\
        if( pcommand_ack_command_GET( src, &item_command ) ){\
            DST##_command_SET( item_command , dst  );\
        }\
        e_MAV_RESULT  item_result;\
        if( pcommand_ack_result_GET( src, &item_result ) ){\
            DST##_result_SET( item_result , dst  );\
        }\
        int8_t  item_progress;\
        if( pcommand_ack_progress_GET( src, &item_progress ) ){\
            DST##_progress_SET( item_progress , dst  );\
        }\
        int32_t  item_result_param2;\
        if( pcommand_ack_result_param2_GET( src, &item_result_param2 ) ){\
            DST##_result_param2_SET( item_result_param2 , dst  );\
        }\
        int8_t  item_target_system;\
        if( pcommand_ack_target_system_GET( src, &item_target_system ) ){\
            DST##_target_system_SET( item_target_system , dst  );\
        }\
        int8_t  item_target_component;\
        if( pcommand_ack_target_component_GET( src, &item_target_component ) ){\
            DST##_target_component_SET( item_target_component , dst  );\
        }\
    }

/**
*THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD. */

typedef Pack DATA_STREAM_data_stream; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdata_stream_DATA_STREAM;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdata_stream_DATA_STREAM *pdata_stream_DATA_STREAM_from(DATA_STREAM_data_stream *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_message_rate_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_stream_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_on_off_SET( int8_t * src, <DST> * dst  ){}
*/

#define pdata_stream_DATA_STREAM_PUSH_INTO(DST)\
    static inline void pdata_stream_DATA_STREAM_push_into_##DST ( pdata_stream_DATA_STREAM * src, DST * dst) {\
        DST##_message_rate_SET( pdata_stream_message_rate_GET( src  ), dst  );\
        DST##_stream_id_SET( pdata_stream_stream_id_GET( src  ), dst  );\
        DST##_on_off_SET( pdata_stream_on_off_GET( src  ), dst  );\
    }

/**
*Request the information of the mission item with the sequence number seq. The response of the system to
*				 this message should be a MISSION_ITEM message. http:qgroundcontrol.org/mavlink/waypoint_protoco */

typedef Pack MISSION_REQUEST_mission_request; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_request_MISSION_REQUEST;// data navigator over pack fields data
/**
															* Wrap MISSION_REQUEST in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_request_MISSION_REQUEST *MISSION_REQUEST_mission_request_wrap(MISSION_REQUEST_mission_request *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_REQUEST_mission_request *pmission_request_MISSION_REQUEST_unwrap(pmission_request_MISSION_REQUEST *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_request_MISSION_REQUEST_PUSH_INTO(DST)\
    static inline void pmission_request_MISSION_REQUEST_push_into_##DST ( pmission_request_MISSION_REQUEST * src, DST * dst) {\
        DST##_seq_SET( pmission_request_seq_GET( src  ), dst  );\
        DST##_target_system_SET( pmission_request_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_request_target_component_GET( src  ), dst  );\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_request_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*Response from a TERRAIN_CHECK request */

typedef Pack TERRAIN_REPORT_terrain_report; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pterrain_report_TERRAIN_REPORT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pterrain_report_TERRAIN_REPORT *pterrain_report_TERRAIN_REPORT_from(TERRAIN_REPORT_terrain_report *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_spacing_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_pending_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_loaded_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_terrain_height_SET( float * src, <DST> * dst  ){}
static inline void <DST>_current_height_SET( float * src, <DST> * dst  ){}
*/

#define pterrain_report_TERRAIN_REPORT_PUSH_INTO(DST)\
    static inline void pterrain_report_TERRAIN_REPORT_push_into_##DST ( pterrain_report_TERRAIN_REPORT * src, DST * dst) {\
        DST##_spacing_SET( pterrain_report_spacing_GET( src  ), dst  );\
        DST##_pending_SET( pterrain_report_pending_GET( src  ), dst  );\
        DST##_loaded_SET( pterrain_report_loaded_GET( src  ), dst  );\
        DST##_lat_SET( pterrain_report_lat_GET( src  ), dst  );\
        DST##_lon_SET( pterrain_report_lon_GET( src  ), dst  );\
        DST##_terrain_height_SET( pterrain_report_terrain_height_GET( src  ), dst  );\
        DST##_current_height_SET( pterrain_report_current_height_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_spacing_GET( <SRC> * src ){}
static inline int16_t <SRC>_pending_GET( <SRC> * src ){}
static inline int16_t <SRC>_loaded_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lon_GET( <SRC> * src ){}
static inline float <SRC>_terrain_height_GET( <SRC> * src ){}
static inline float <SRC>_current_height_GET( <SRC> * src ){}
*/

#define pterrain_report_TERRAIN_REPORT_PULL_FROM(SRC)\
    static inline void pterrain_report_TERRAIN_REPORT_pull_from_##SRC ( SRC * src, pterrain_report_TERRAIN_REPORT * dst) {\
        pterrain_report_spacing_SET( SRC##_spacing_GET(src ), dst  );\
        pterrain_report_pending_SET( SRC##_pending_GET(src ), dst  );\
        pterrain_report_loaded_SET( SRC##_loaded_GET(src ), dst  );\
        pterrain_report_lat_SET( SRC##_lat_GET(src ), dst  );\
        pterrain_report_lon_SET( SRC##_lon_GET(src ), dst  );\
        pterrain_report_terrain_height_SET( SRC##_terrain_height_GET(src ), dst  );\
        pterrain_report_current_height_SET( SRC##_current_height_GET(src ), dst  );\
    }

/**
*The position the system will return to and land on. The position is set automatically by the system during
*				 the takeoff in case it was not explicitely set by the operator before or after. The global and local
*				 positions encode the position in the respective coordinate frames, while the q parameter encodes the
*				 orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
*				 can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
*				 the system should fly in normal flight mode and then perform a landing sequence along the vector */

typedef Pack SET_HOME_POSITION_set_home_position; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pset_home_position_SET_HOME_POSITION;// data navigator over pack fields data
/**
															* Wrap SET_HOME_POSITION in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pset_home_position_SET_HOME_POSITION *SET_HOME_POSITION_set_home_position_wrap(SET_HOME_POSITION_set_home_position *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SET_HOME_POSITION_set_home_position *pset_home_position_SET_HOME_POSITION_unwrap(pset_home_position_SET_HOME_POSITION *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vset_home_position_q;
//Maximum field array length constant
#define Pset_home_position_q_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_latitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_longitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q_SET( Vset_home_position_q * src, <DST> * dst  ){}
static inline void <DST>_approach_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_approach_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_approach_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
*/

#define pset_home_position_SET_HOME_POSITION_PUSH_INTO(DST)\
    static inline void pset_home_position_SET_HOME_POSITION_push_into_##DST ( pset_home_position_SET_HOME_POSITION * src, DST * dst) {\
        DST##_target_system_SET( pset_home_position_target_system_GET( src  ), dst  );\
        DST##_latitude_SET( pset_home_position_latitude_GET( src  ), dst  );\
        DST##_longitude_SET( pset_home_position_longitude_GET( src  ), dst  );\
        DST##_altitude_SET( pset_home_position_altitude_GET( src  ), dst  );\
        DST##_x_SET( pset_home_position_x_GET( src  ), dst  );\
        DST##_y_SET( pset_home_position_y_GET( src  ), dst  );\
        DST##_z_SET( pset_home_position_z_GET( src  ), dst  );\
        Vset_home_position_q item_q = pset_home_position_q_GET( src  );\
       DST##_q_SET( &item_q, dst );\
        DST##_approach_x_SET( pset_home_position_approach_x_GET( src  ), dst  );\
        DST##_approach_y_SET( pset_home_position_approach_y_GET( src  ), dst  );\
        DST##_approach_z_SET( pset_home_position_approach_z_GET( src  ), dst  );\
        int64_t  item_time_usec;\
        if( pset_home_position_time_usec_GET( src, &item_time_usec ) ){\
            DST##_time_usec_SET( item_time_usec , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int32_t <SRC>_latitude_GET( <SRC> * src ){}
static inline int32_t <SRC>_longitude_GET( <SRC> * src ){}
static inline int32_t <SRC>_altitude_GET( <SRC> * src ){}
static inline float <SRC>_x_GET( <SRC> * src ){}
static inline float <SRC>_y_GET( <SRC> * src ){}
static inline float <SRC>_z_GET( <SRC> * src ){}
static inline float <SRC>_q_GET( <SRC> * src, Vset_home_position_q * dst ){}
static inline float <SRC>_approach_x_GET( <SRC> * src ){}
static inline float <SRC>_approach_y_GET( <SRC> * src ){}
static inline float <SRC>_approach_z_GET( <SRC> * src ){}
static inline bool  <SRC>_time_usec_item_exist( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
*/

#define pset_home_position_SET_HOME_POSITION_PULL_FROM(SRC)\
    static inline void pset_home_position_SET_HOME_POSITION_pull_from_##SRC ( SRC * src, pset_home_position_SET_HOME_POSITION * dst) {\
        pset_home_position_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pset_home_position_latitude_SET( SRC##_latitude_GET(src ), dst  );\
        pset_home_position_longitude_SET( SRC##_longitude_GET(src ), dst  );\
        pset_home_position_altitude_SET( SRC##_altitude_GET(src ), dst  );\
        pset_home_position_x_SET( SRC##_x_GET(src ), dst  );\
        pset_home_position_y_SET( SRC##_y_GET(src ), dst  );\
        pset_home_position_z_SET( SRC##_z_GET(src ), dst  );\
       Vset_home_position_q item_q = pset_home_position_q_SET( NULL, dst  );\
       SRC##_q_GET( src, &item_q );\
        pset_home_position_approach_x_SET( SRC##_approach_x_GET(src ), dst  );\
        pset_home_position_approach_y_SET( SRC##_approach_y_GET(src ), dst  );\
        pset_home_position_approach_z_SET( SRC##_approach_z_GET(src ), dst  );\
        if( SRC##_time_usec_item_exist(src ) )\
        pset_home_position_time_usec_SET( pset_home_position_SET_HOME_POSITION_time_usec_GET( src ),  dst  );\
    }

/**
*Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
*				 is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
*				 violate this specification */

typedef Pack HIL_RC_INPUTS_RAW_hil_rc_inputs_raw; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t phil_rc_inputs_raw_HIL_RC_INPUTS_RAW;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *phil_rc_inputs_raw_HIL_RC_INPUTS_RAW_from(HIL_RC_INPUTS_RAW_hil_rc_inputs_raw *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_chan1_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan2_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan3_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan4_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan5_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan6_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan7_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan8_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan9_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan10_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan11_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan12_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_rssi_SET( int8_t * src, <DST> * dst  ){}
*/

#define phil_rc_inputs_raw_HIL_RC_INPUTS_RAW_PUSH_INTO(DST)\
    static inline void phil_rc_inputs_raw_HIL_RC_INPUTS_RAW_push_into_##DST ( phil_rc_inputs_raw_HIL_RC_INPUTS_RAW * src, DST * dst) {\
        DST##_chan1_raw_SET( phil_rc_inputs_raw_chan1_raw_GET( src  ), dst  );\
        DST##_chan2_raw_SET( phil_rc_inputs_raw_chan2_raw_GET( src  ), dst  );\
        DST##_chan3_raw_SET( phil_rc_inputs_raw_chan3_raw_GET( src  ), dst  );\
        DST##_chan4_raw_SET( phil_rc_inputs_raw_chan4_raw_GET( src  ), dst  );\
        DST##_chan5_raw_SET( phil_rc_inputs_raw_chan5_raw_GET( src  ), dst  );\
        DST##_chan6_raw_SET( phil_rc_inputs_raw_chan6_raw_GET( src  ), dst  );\
        DST##_chan7_raw_SET( phil_rc_inputs_raw_chan7_raw_GET( src  ), dst  );\
        DST##_chan8_raw_SET( phil_rc_inputs_raw_chan8_raw_GET( src  ), dst  );\
        DST##_chan9_raw_SET( phil_rc_inputs_raw_chan9_raw_GET( src  ), dst  );\
        DST##_chan10_raw_SET( phil_rc_inputs_raw_chan10_raw_GET( src  ), dst  );\
        DST##_chan11_raw_SET( phil_rc_inputs_raw_chan11_raw_GET( src  ), dst  );\
        DST##_chan12_raw_SET( phil_rc_inputs_raw_chan12_raw_GET( src  ), dst  );\
        DST##_time_usec_SET( phil_rc_inputs_raw_time_usec_GET( src  ), dst  );\
        DST##_rssi_SET( phil_rc_inputs_raw_rssi_GET( src  ), dst  );\
    }

/**
*The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
*				 unit */

typedef Pack SCALED_IMU3_scaled_imu3; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pscaled_imu3_SCALED_IMU3;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pscaled_imu3_SCALED_IMU3 *pscaled_imu3_SCALED_IMU3_from(SCALED_IMU3_scaled_imu3 *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xgyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ygyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zgyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xmag_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ymag_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zmag_SET( int16_t * src, <DST> * dst  ){}
*/

#define pscaled_imu3_SCALED_IMU3_PUSH_INTO(DST)\
    static inline void pscaled_imu3_SCALED_IMU3_push_into_##DST ( pscaled_imu3_SCALED_IMU3 * src, DST * dst) {\
        DST##_time_boot_ms_SET( pscaled_imu3_time_boot_ms_GET( src  ), dst  );\
        DST##_xacc_SET( pscaled_imu3_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( pscaled_imu3_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( pscaled_imu3_zacc_GET( src  ), dst  );\
        DST##_xgyro_SET( pscaled_imu3_xgyro_GET( src  ), dst  );\
        DST##_ygyro_SET( pscaled_imu3_ygyro_GET( src  ), dst  );\
        DST##_zgyro_SET( pscaled_imu3_zgyro_GET( src  ), dst  );\
        DST##_xmag_SET( pscaled_imu3_xmag_GET( src  ), dst  );\
        DST##_ymag_SET( pscaled_imu3_ymag_GET( src  ), dst  );\
        DST##_zmag_SET( pscaled_imu3_zmag_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int16_t <SRC>_xacc_GET( <SRC> * src ){}
static inline int16_t <SRC>_yacc_GET( <SRC> * src ){}
static inline int16_t <SRC>_zacc_GET( <SRC> * src ){}
static inline int16_t <SRC>_xgyro_GET( <SRC> * src ){}
static inline int16_t <SRC>_ygyro_GET( <SRC> * src ){}
static inline int16_t <SRC>_zgyro_GET( <SRC> * src ){}
static inline int16_t <SRC>_xmag_GET( <SRC> * src ){}
static inline int16_t <SRC>_ymag_GET( <SRC> * src ){}
static inline int16_t <SRC>_zmag_GET( <SRC> * src ){}
*/

#define pscaled_imu3_SCALED_IMU3_PULL_FROM(SRC)\
    static inline void pscaled_imu3_SCALED_IMU3_pull_from_##SRC ( SRC * src, pscaled_imu3_SCALED_IMU3 * dst) {\
        pscaled_imu3_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pscaled_imu3_xacc_SET( SRC##_xacc_GET(src ), dst  );\
        pscaled_imu3_yacc_SET( SRC##_yacc_GET(src ), dst  );\
        pscaled_imu3_zacc_SET( SRC##_zacc_GET(src ), dst  );\
        pscaled_imu3_xgyro_SET( SRC##_xgyro_GET(src ), dst  );\
        pscaled_imu3_ygyro_SET( SRC##_ygyro_GET(src ), dst  );\
        pscaled_imu3_zgyro_SET( SRC##_zgyro_GET(src ), dst  );\
        pscaled_imu3_xmag_SET( SRC##_xmag_GET(src ), dst  );\
        pscaled_imu3_ymag_SET( SRC##_ymag_GET(src ), dst  );\
        pscaled_imu3_zmag_SET( SRC##_zmag_GET(src ), dst  );\
    }

/**
*THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode,
*				 as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall
*				 aircraft, not only for one component */

typedef Pack SET_MODE_set_mode; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pset_mode_SET_MODE;// data navigator over pack fields data
/**
															* Wrap SET_MODE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pset_mode_SET_MODE *SET_MODE_set_mode_wrap(SET_MODE_set_mode *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SET_MODE_set_mode *pset_mode_SET_MODE_unwrap(pset_mode_SET_MODE *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_custom_mode_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_base_mode_SET( e_MAV_MODE * src, <DST> * dst  ){}
*/

#define pset_mode_SET_MODE_PUSH_INTO(DST)\
    static inline void pset_mode_SET_MODE_push_into_##DST ( pset_mode_SET_MODE * src, DST * dst) {\
        DST##_custom_mode_SET( pset_mode_custom_mode_GET( src  ), dst  );\
        DST##_target_system_SET( pset_mode_target_system_GET( src  ), dst  );\
        e_MAV_MODE  item_base_mode;\
        if( pset_mode_base_mode_GET( src, &item_base_mode ) ){\
            DST##_base_mode_SET( item_base_mode , dst  );\
        }\
    }

/**
*Message to control a camera mount, directional antenna, etc. */

typedef Pack MOUNT_CONTROL_mount_control; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmount_control_MOUNT_CONTROL;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmount_control_MOUNT_CONTROL *pmount_control_MOUNT_CONTROL_from(MOUNT_CONTROL_mount_control *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_input_a_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_input_b_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_input_c_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_save_position_SET( int8_t * src, <DST> * dst  ){}
*/

#define pmount_control_MOUNT_CONTROL_PUSH_INTO(DST)\
    static inline void pmount_control_MOUNT_CONTROL_push_into_##DST ( pmount_control_MOUNT_CONTROL * src, DST * dst) {\
        DST##_target_system_SET( pmount_control_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmount_control_target_component_GET( src  ), dst  );\
        DST##_input_a_SET( pmount_control_input_a_GET( src  ), dst  );\
        DST##_input_b_SET( pmount_control_input_b_GET( src  ), dst  );\
        DST##_input_c_SET( pmount_control_input_c_GET( src  ), dst  );\
        DST##_save_position_SET( pmount_control_save_position_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int32_t <SRC>_input_a_GET( <SRC> * src ){}
static inline int32_t <SRC>_input_b_GET( <SRC> * src ){}
static inline int32_t <SRC>_input_c_GET( <SRC> * src ){}
static inline int8_t <SRC>_save_position_GET( <SRC> * src ){}
*/

#define pmount_control_MOUNT_CONTROL_PULL_FROM(SRC)\
    static inline void pmount_control_MOUNT_CONTROL_pull_from_##SRC ( SRC * src, pmount_control_MOUNT_CONTROL * dst) {\
        pmount_control_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pmount_control_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pmount_control_input_a_SET( SRC##_input_a_GET(src ), dst  );\
        pmount_control_input_b_SET( SRC##_input_b_GET(src ), dst  );\
        pmount_control_input_c_SET( SRC##_input_c_GET(src ), dst  );\
        pmount_control_save_position_SET( SRC##_save_position_GET(src ), dst  );\
    }

/**
*Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
*				 This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
*				 this way */

typedef Pack POSITION_TARGET_GLOBAL_INT_position_target_global_int; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pposition_target_global_int_POSITION_TARGET_GLOBAL_INT;// data navigator over pack fields data
/**
															* Wrap POSITION_TARGET_GLOBAL_INT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *POSITION_TARGET_GLOBAL_INT_position_target_global_int_wrap(POSITION_TARGET_GLOBAL_INT_position_target_global_int *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline POSITION_TARGET_GLOBAL_INT_position_target_global_int *pposition_target_global_int_POSITION_TARGET_GLOBAL_INT_unwrap(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_type_mask_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lat_int_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lon_int_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_coordinate_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
*/

#define pposition_target_global_int_POSITION_TARGET_GLOBAL_INT_PUSH_INTO(DST)\
    static inline void pposition_target_global_int_POSITION_TARGET_GLOBAL_INT_push_into_##DST ( pposition_target_global_int_POSITION_TARGET_GLOBAL_INT * src, DST * dst) {\
        DST##_type_mask_SET( pposition_target_global_int_type_mask_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( pposition_target_global_int_time_boot_ms_GET( src  ), dst  );\
        DST##_lat_int_SET( pposition_target_global_int_lat_int_GET( src  ), dst  );\
        DST##_lon_int_SET( pposition_target_global_int_lon_int_GET( src  ), dst  );\
        DST##_alt_SET( pposition_target_global_int_alt_GET( src  ), dst  );\
        DST##_vx_SET( pposition_target_global_int_vx_GET( src  ), dst  );\
        DST##_vy_SET( pposition_target_global_int_vy_GET( src  ), dst  );\
        DST##_vz_SET( pposition_target_global_int_vz_GET( src  ), dst  );\
        DST##_afx_SET( pposition_target_global_int_afx_GET( src  ), dst  );\
        DST##_afy_SET( pposition_target_global_int_afy_GET( src  ), dst  );\
        DST##_afz_SET( pposition_target_global_int_afz_GET( src  ), dst  );\
        DST##_yaw_SET( pposition_target_global_int_yaw_GET( src  ), dst  );\
        DST##_yaw_rate_SET( pposition_target_global_int_yaw_rate_GET( src  ), dst  );\
        e_MAV_FRAME  item_coordinate_frame;\
        if( pposition_target_global_int_coordinate_frame_GET( src, &item_coordinate_frame ) ){\
            DST##_coordinate_frame_SET( item_coordinate_frame , dst  );\
        }\
    }

/**
*Control vehicle LEDs */

typedef Pack LED_CONTROL_led_control; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pled_control_LED_CONTROL;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pled_control_LED_CONTROL *pled_control_LED_CONTROL_from(LED_CONTROL_led_control *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vled_control_custom_bytes;
//Maximum field array length constant
#define Pled_control_custom_bytes_len  ( 24 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_instance_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_pattern_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_custom_len_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_custom_bytes_SET( Vled_control_custom_bytes * src, <DST> * dst  ){}
*/

#define pled_control_LED_CONTROL_PUSH_INTO(DST)\
    static inline void pled_control_LED_CONTROL_push_into_##DST ( pled_control_LED_CONTROL * src, DST * dst) {\
        DST##_target_system_SET( pled_control_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pled_control_target_component_GET( src  ), dst  );\
        DST##_instance_SET( pled_control_instance_GET( src  ), dst  );\
        DST##_pattern_SET( pled_control_pattern_GET( src  ), dst  );\
        DST##_custom_len_SET( pled_control_custom_len_GET( src  ), dst  );\
        Vled_control_custom_bytes item_custom_bytes = pled_control_custom_bytes_GET( src  );\
       DST##_custom_bytes_SET( &item_custom_bytes, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_instance_GET( <SRC> * src ){}
static inline int8_t <SRC>_pattern_GET( <SRC> * src ){}
static inline int8_t <SRC>_custom_len_GET( <SRC> * src ){}
static inline int8_t <SRC>_custom_bytes_GET( <SRC> * src, Vled_control_custom_bytes * dst ){}
*/

#define pled_control_LED_CONTROL_PULL_FROM(SRC)\
    static inline void pled_control_LED_CONTROL_pull_from_##SRC ( SRC * src, pled_control_LED_CONTROL * dst) {\
        pled_control_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pled_control_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pled_control_instance_SET( SRC##_instance_GET(src ), dst  );\
        pled_control_pattern_SET( SRC##_pattern_GET(src ), dst  );\
        pled_control_custom_len_SET( SRC##_custom_len_GET(src ), dst  );\
       Vled_control_custom_bytes item_custom_bytes = pled_control_custom_bytes_SET( NULL, dst  );\
       SRC##_custom_bytes_GET( src, &item_custom_bytes );\
    }

/**
*Status of simulation environment, if used */

typedef Pack SIM_STATE_sim_state; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t psim_state_SIM_STATE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline psim_state_SIM_STATE *psim_state_SIM_STATE_from(SIM_STATE_sim_state *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_q1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q4_SET( float * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ygyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lon_SET( float * src, <DST> * dst  ){}
static inline void <DST>_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_std_dev_horz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_std_dev_vert_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vn_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ve_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vd_SET( float * src, <DST> * dst  ){}
*/

#define psim_state_SIM_STATE_PUSH_INTO(DST)\
    static inline void psim_state_SIM_STATE_push_into_##DST ( psim_state_SIM_STATE * src, DST * dst) {\
        DST##_q1_SET( psim_state_q1_GET( src  ), dst  );\
        DST##_q2_SET( psim_state_q2_GET( src  ), dst  );\
        DST##_q3_SET( psim_state_q3_GET( src  ), dst  );\
        DST##_q4_SET( psim_state_q4_GET( src  ), dst  );\
        DST##_roll_SET( psim_state_roll_GET( src  ), dst  );\
        DST##_pitch_SET( psim_state_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( psim_state_yaw_GET( src  ), dst  );\
        DST##_xacc_SET( psim_state_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( psim_state_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( psim_state_zacc_GET( src  ), dst  );\
        DST##_xgyro_SET( psim_state_xgyro_GET( src  ), dst  );\
        DST##_ygyro_SET( psim_state_ygyro_GET( src  ), dst  );\
        DST##_zgyro_SET( psim_state_zgyro_GET( src  ), dst  );\
        DST##_lat_SET( psim_state_lat_GET( src  ), dst  );\
        DST##_lon_SET( psim_state_lon_GET( src  ), dst  );\
        DST##_alt_SET( psim_state_alt_GET( src  ), dst  );\
        DST##_std_dev_horz_SET( psim_state_std_dev_horz_GET( src  ), dst  );\
        DST##_std_dev_vert_SET( psim_state_std_dev_vert_GET( src  ), dst  );\
        DST##_vn_SET( psim_state_vn_GET( src  ), dst  );\
        DST##_ve_SET( psim_state_ve_GET( src  ), dst  );\
        DST##_vd_SET( psim_state_vd_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_q1_GET( <SRC> * src ){}
static inline float <SRC>_q2_GET( <SRC> * src ){}
static inline float <SRC>_q3_GET( <SRC> * src ){}
static inline float <SRC>_q4_GET( <SRC> * src ){}
static inline float <SRC>_roll_GET( <SRC> * src ){}
static inline float <SRC>_pitch_GET( <SRC> * src ){}
static inline float <SRC>_yaw_GET( <SRC> * src ){}
static inline float <SRC>_xacc_GET( <SRC> * src ){}
static inline float <SRC>_yacc_GET( <SRC> * src ){}
static inline float <SRC>_zacc_GET( <SRC> * src ){}
static inline float <SRC>_xgyro_GET( <SRC> * src ){}
static inline float <SRC>_ygyro_GET( <SRC> * src ){}
static inline float <SRC>_zgyro_GET( <SRC> * src ){}
static inline float <SRC>_lat_GET( <SRC> * src ){}
static inline float <SRC>_lon_GET( <SRC> * src ){}
static inline float <SRC>_alt_GET( <SRC> * src ){}
static inline float <SRC>_std_dev_horz_GET( <SRC> * src ){}
static inline float <SRC>_std_dev_vert_GET( <SRC> * src ){}
static inline float <SRC>_vn_GET( <SRC> * src ){}
static inline float <SRC>_ve_GET( <SRC> * src ){}
static inline float <SRC>_vd_GET( <SRC> * src ){}
*/

#define psim_state_SIM_STATE_PULL_FROM(SRC)\
    static inline void psim_state_SIM_STATE_pull_from_##SRC ( SRC * src, psim_state_SIM_STATE * dst) {\
        psim_state_q1_SET( SRC##_q1_GET(src ), dst  );\
        psim_state_q2_SET( SRC##_q2_GET(src ), dst  );\
        psim_state_q3_SET( SRC##_q3_GET(src ), dst  );\
        psim_state_q4_SET( SRC##_q4_GET(src ), dst  );\
        psim_state_roll_SET( SRC##_roll_GET(src ), dst  );\
        psim_state_pitch_SET( SRC##_pitch_GET(src ), dst  );\
        psim_state_yaw_SET( SRC##_yaw_GET(src ), dst  );\
        psim_state_xacc_SET( SRC##_xacc_GET(src ), dst  );\
        psim_state_yacc_SET( SRC##_yacc_GET(src ), dst  );\
        psim_state_zacc_SET( SRC##_zacc_GET(src ), dst  );\
        psim_state_xgyro_SET( SRC##_xgyro_GET(src ), dst  );\
        psim_state_ygyro_SET( SRC##_ygyro_GET(src ), dst  );\
        psim_state_zgyro_SET( SRC##_zgyro_GET(src ), dst  );\
        psim_state_lat_SET( SRC##_lat_GET(src ), dst  );\
        psim_state_lon_SET( SRC##_lon_GET(src ), dst  );\
        psim_state_alt_SET( SRC##_alt_GET(src ), dst  );\
        psim_state_std_dev_horz_SET( SRC##_std_dev_horz_GET(src ), dst  );\
        psim_state_std_dev_vert_SET( SRC##_std_dev_vert_GET(src ), dst  );\
        psim_state_vn_SET( SRC##_vn_GET(src ), dst  );\
        psim_state_ve_SET( SRC##_ve_GET(src ), dst  );\
        psim_state_vd_SET( SRC##_vd_GET(src ), dst  );\
    }

/**
*Configure AP SSID and Password. */

typedef Pack WIFI_CONFIG_AP_wifi_config_ap; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pwifi_config_ap_WIFI_CONFIG_AP;// data navigator over pack fields data
/**
															* Wrap WIFI_CONFIG_AP in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pwifi_config_ap_WIFI_CONFIG_AP *WIFI_CONFIG_AP_wifi_config_ap_wrap(WIFI_CONFIG_AP_wifi_config_ap *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline WIFI_CONFIG_AP_wifi_config_ap *pwifi_config_ap_WIFI_CONFIG_AP_unwrap(pwifi_config_ap_WIFI_CONFIG_AP *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vwifi_config_ap_ssid;
//Maximum field array length constant
#define Pwifi_config_ap_ssid_len_max  ( 255 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vwifi_config_ap_password;
//Maximum field array length constant
#define Pwifi_config_ap_password_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_ssid_SET( Vwifi_config_ap_ssid * src, <DST> * dst  ){}
static inline void <DST>_password_SET( Vwifi_config_ap_password * src, <DST> * dst  ){}
*/

#define pwifi_config_ap_WIFI_CONFIG_AP_PUSH_INTO(DST)\
    static inline void pwifi_config_ap_WIFI_CONFIG_AP_push_into_##DST ( pwifi_config_ap_WIFI_CONFIG_AP * src, DST * dst) {\
        Vwifi_config_ap_ssid  item_ssid;\
        if( pwifi_config_ap_ssid_GET( src, &item_ssid ) ){\
            DST##_ssid_SET( &item_ssid, dst );\
        }\
        Vwifi_config_ap_password  item_password;\
        if( pwifi_config_ap_password_GET( src, &item_password ) ){\
            DST##_password_SET( &item_password, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline size_t  <SRC>_ssid_item_exist( <SRC> * src  ){}
static inline void <SRC>_ssid_GET( <SRC> * src, Vwifi_config_ap_ssid * dst ){}
static inline size_t  <SRC>_password_item_exist( <SRC> * src  ){}
static inline void <SRC>_password_GET( <SRC> * src, Vwifi_config_ap_password * dst ){}
*/

#define pwifi_config_ap_WIFI_CONFIG_AP_PULL_FROM(SRC)\
    static inline void pwifi_config_ap_WIFI_CONFIG_AP_pull_from_##SRC ( SRC * src, pwifi_config_ap_WIFI_CONFIG_AP * dst) {\
        const size_t len_ssid = SRC##_ssid_item_exist(src );\
        if( len_ssid ){\
            Vwifi_config_ap_ssid    item_ssid = pwifi_config_ap_ssid_SET( NULL, len_ssid, dst  );\
            pwifi_config_ap_WIFI_CONFIG_AP_ssid_GET(src, &item_ssid );\
        }\
        const size_t len_password = SRC##_password_item_exist(src );\
        if( len_password ){\
            Vwifi_config_ap_password    item_password = pwifi_config_ap_password_SET( NULL, len_password, dst  );\
            pwifi_config_ap_WIFI_CONFIG_AP_password_GET(src, &item_password );\
        }\
    }

/**
*Data packet, size 96 */

typedef Pack DATA96_data96; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdata96_DATA96;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdata96_DATA96 *pdata96_DATA96_from(DATA96_data96 *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdata96_daTa;
//Maximum field array length constant
#define Pdata96_daTa_len  ( 96 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_typE_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_len_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vdata96_daTa * src, <DST> * dst  ){}
*/

#define pdata96_DATA96_PUSH_INTO(DST)\
    static inline void pdata96_DATA96_push_into_##DST ( pdata96_DATA96 * src, DST * dst) {\
        DST##_typE_SET( pdata96_typE_GET( src  ), dst  );\
        DST##_len_SET( pdata96_len_GET( src  ), dst  );\
        Vdata96_daTa item_daTa = pdata96_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_typE_GET( <SRC> * src ){}
static inline int8_t <SRC>_len_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vdata96_daTa * dst ){}
*/

#define pdata96_DATA96_PULL_FROM(SRC)\
    static inline void pdata96_DATA96_pull_from_##SRC ( SRC * src, pdata96_DATA96 * dst) {\
        pdata96_typE_SET( SRC##_typE_GET(src ), dst  );\
        pdata96_len_SET( SRC##_len_GET(src ), dst  );\
       Vdata96_daTa item_daTa = pdata96_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*WIP: Information about flight since last arming */

typedef Pack FLIGHT_INFORMATION_flight_information; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pflight_information_FLIGHT_INFORMATION;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pflight_information_FLIGHT_INFORMATION *pflight_information_FLIGHT_INFORMATION_from(FLIGHT_INFORMATION_flight_information *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_arming_time_utc_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_takeoff_time_utc_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_flight_uuid_SET( int64_t * src, <DST> * dst  ){}
*/

#define pflight_information_FLIGHT_INFORMATION_PUSH_INTO(DST)\
    static inline void pflight_information_FLIGHT_INFORMATION_push_into_##DST ( pflight_information_FLIGHT_INFORMATION * src, DST * dst) {\
        DST##_time_boot_ms_SET( pflight_information_time_boot_ms_GET( src  ), dst  );\
        DST##_arming_time_utc_SET( pflight_information_arming_time_utc_GET( src  ), dst  );\
        DST##_takeoff_time_utc_SET( pflight_information_takeoff_time_utc_GET( src  ), dst  );\
        DST##_flight_uuid_SET( pflight_information_flight_uuid_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int64_t <SRC>_arming_time_utc_GET( <SRC> * src ){}
static inline int64_t <SRC>_takeoff_time_utc_GET( <SRC> * src ){}
static inline int64_t <SRC>_flight_uuid_GET( <SRC> * src ){}
*/

#define pflight_information_FLIGHT_INFORMATION_PULL_FROM(SRC)\
    static inline void pflight_information_FLIGHT_INFORMATION_pull_from_##SRC ( SRC * src, pflight_information_FLIGHT_INFORMATION * dst) {\
        pflight_information_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pflight_information_arming_time_utc_SET( SRC##_arming_time_utc_GET(src ), dst  );\
        pflight_information_takeoff_time_utc_SET( SRC##_takeoff_time_utc_GET(src ), dst  );\
        pflight_information_flight_uuid_SET( SRC##_flight_uuid_GET(src ), dst  );\
    }

/**
*The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
*				 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification */

typedef Pack RC_CHANNELS_RAW_rc_channels_raw; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t prc_channels_raw_RC_CHANNELS_RAW;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline prc_channels_raw_RC_CHANNELS_RAW *prc_channels_raw_RC_CHANNELS_RAW_from(RC_CHANNELS_RAW_rc_channels_raw *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_chan1_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan2_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan3_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan4_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan5_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan6_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan7_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan8_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_port_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_rssi_SET( int8_t * src, <DST> * dst  ){}
*/

#define prc_channels_raw_RC_CHANNELS_RAW_PUSH_INTO(DST)\
    static inline void prc_channels_raw_RC_CHANNELS_RAW_push_into_##DST ( prc_channels_raw_RC_CHANNELS_RAW * src, DST * dst) {\
        DST##_chan1_raw_SET( prc_channels_raw_chan1_raw_GET( src  ), dst  );\
        DST##_chan2_raw_SET( prc_channels_raw_chan2_raw_GET( src  ), dst  );\
        DST##_chan3_raw_SET( prc_channels_raw_chan3_raw_GET( src  ), dst  );\
        DST##_chan4_raw_SET( prc_channels_raw_chan4_raw_GET( src  ), dst  );\
        DST##_chan5_raw_SET( prc_channels_raw_chan5_raw_GET( src  ), dst  );\
        DST##_chan6_raw_SET( prc_channels_raw_chan6_raw_GET( src  ), dst  );\
        DST##_chan7_raw_SET( prc_channels_raw_chan7_raw_GET( src  ), dst  );\
        DST##_chan8_raw_SET( prc_channels_raw_chan8_raw_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( prc_channels_raw_time_boot_ms_GET( src  ), dst  );\
        DST##_port_SET( prc_channels_raw_port_GET( src  ), dst  );\
        DST##_rssi_SET( prc_channels_raw_rssi_GET( src  ), dst  );\
    }

/**
*The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The
*				 standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100% */

typedef Pack SERVO_OUTPUT_RAW_servo_output_raw; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pservo_output_raw_SERVO_OUTPUT_RAW;// data navigator over pack fields data
/**
															* Wrap SERVO_OUTPUT_RAW in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pservo_output_raw_SERVO_OUTPUT_RAW *SERVO_OUTPUT_RAW_servo_output_raw_wrap(SERVO_OUTPUT_RAW_servo_output_raw *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SERVO_OUTPUT_RAW_servo_output_raw *pservo_output_raw_SERVO_OUTPUT_RAW_unwrap(pservo_output_raw_SERVO_OUTPUT_RAW *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_servo1_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo2_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo3_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo4_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo5_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo6_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo7_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo8_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_port_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_servo9_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo10_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo11_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo12_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo13_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo14_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo15_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_servo16_raw_SET( int16_t * src, <DST> * dst  ){}
*/

#define pservo_output_raw_SERVO_OUTPUT_RAW_PUSH_INTO(DST)\
    static inline void pservo_output_raw_SERVO_OUTPUT_RAW_push_into_##DST ( pservo_output_raw_SERVO_OUTPUT_RAW * src, DST * dst) {\
        DST##_servo1_raw_SET( pservo_output_raw_servo1_raw_GET( src  ), dst  );\
        DST##_servo2_raw_SET( pservo_output_raw_servo2_raw_GET( src  ), dst  );\
        DST##_servo3_raw_SET( pservo_output_raw_servo3_raw_GET( src  ), dst  );\
        DST##_servo4_raw_SET( pservo_output_raw_servo4_raw_GET( src  ), dst  );\
        DST##_servo5_raw_SET( pservo_output_raw_servo5_raw_GET( src  ), dst  );\
        DST##_servo6_raw_SET( pservo_output_raw_servo6_raw_GET( src  ), dst  );\
        DST##_servo7_raw_SET( pservo_output_raw_servo7_raw_GET( src  ), dst  );\
        DST##_servo8_raw_SET( pservo_output_raw_servo8_raw_GET( src  ), dst  );\
        DST##_time_usec_SET( pservo_output_raw_time_usec_GET( src  ), dst  );\
        DST##_port_SET( pservo_output_raw_port_GET( src  ), dst  );\
        int16_t  item_servo9_raw;\
        if( pservo_output_raw_servo9_raw_GET( src, &item_servo9_raw ) ){\
            DST##_servo9_raw_SET( item_servo9_raw , dst  );\
        }\
        int16_t  item_servo10_raw;\
        if( pservo_output_raw_servo10_raw_GET( src, &item_servo10_raw ) ){\
            DST##_servo10_raw_SET( item_servo10_raw , dst  );\
        }\
        int16_t  item_servo11_raw;\
        if( pservo_output_raw_servo11_raw_GET( src, &item_servo11_raw ) ){\
            DST##_servo11_raw_SET( item_servo11_raw , dst  );\
        }\
        int16_t  item_servo12_raw;\
        if( pservo_output_raw_servo12_raw_GET( src, &item_servo12_raw ) ){\
            DST##_servo12_raw_SET( item_servo12_raw , dst  );\
        }\
        int16_t  item_servo13_raw;\
        if( pservo_output_raw_servo13_raw_GET( src, &item_servo13_raw ) ){\
            DST##_servo13_raw_SET( item_servo13_raw , dst  );\
        }\
        int16_t  item_servo14_raw;\
        if( pservo_output_raw_servo14_raw_GET( src, &item_servo14_raw ) ){\
            DST##_servo14_raw_SET( item_servo14_raw , dst  );\
        }\
        int16_t  item_servo15_raw;\
        if( pservo_output_raw_servo15_raw_GET( src, &item_servo15_raw ) ){\
            DST##_servo15_raw_SET( item_servo15_raw , dst  );\
        }\
        int16_t  item_servo16_raw;\
        if( pservo_output_raw_servo16_raw_GET( src, &item_servo16_raw ) ){\
            DST##_servo16_raw_SET( item_servo16_raw , dst  );\
        }\
    }

/**
*state of APM memory */

typedef Pack MEMINFO_meminfo; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmeminfo_MEMINFO;// data navigator over pack fields data
/**
															* Wrap MEMINFO in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmeminfo_MEMINFO *MEMINFO_meminfo_wrap(MEMINFO_meminfo *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MEMINFO_meminfo *pmeminfo_MEMINFO_unwrap(pmeminfo_MEMINFO *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_brkval_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_freemem_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_freemem32_SET( int32_t * src, <DST> * dst  ){}
*/

#define pmeminfo_MEMINFO_PUSH_INTO(DST)\
    static inline void pmeminfo_MEMINFO_push_into_##DST ( pmeminfo_MEMINFO * src, DST * dst) {\
        DST##_brkval_SET( pmeminfo_brkval_GET( src  ), dst  );\
        DST##_freemem_SET( pmeminfo_freemem_GET( src  ), dst  );\
        int32_t  item_freemem32;\
        if( pmeminfo_freemem32_GET( src, &item_freemem32 ) ){\
            DST##_freemem32_SET( item_freemem32 , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_brkval_GET( <SRC> * src ){}
static inline int16_t <SRC>_freemem_GET( <SRC> * src ){}
static inline bool  <SRC>_freemem32_item_exist( <SRC> * src ){}
static inline int32_t <SRC>_freemem32_GET( <SRC> * src ){}
*/

#define pmeminfo_MEMINFO_PULL_FROM(SRC)\
    static inline void pmeminfo_MEMINFO_pull_from_##SRC ( SRC * src, pmeminfo_MEMINFO * dst) {\
        pmeminfo_brkval_SET( SRC##_brkval_GET(src ), dst  );\
        pmeminfo_freemem_SET( SRC##_freemem_GET(src ), dst  );\
        if( SRC##_freemem32_item_exist(src ) )\
        pmeminfo_freemem32_SET( pmeminfo_MEMINFO_freemem32_GET( src ),  dst  );\
    }

/**
*A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
*				 or (if the autocontinue on the WP was set) continue to the next waypoint */

typedef Pack MISSION_ITEM_REACHED_mission_item_reached; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmission_item_reached_MISSION_ITEM_REACHED;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmission_item_reached_MISSION_ITEM_REACHED *pmission_item_reached_MISSION_ITEM_REACHED_from(MISSION_ITEM_REACHED_mission_item_reached *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int16_t * src, <DST> * dst  ){}
*/

#define pmission_item_reached_MISSION_ITEM_REACHED_PUSH_INTO(DST)\
    static inline void pmission_item_reached_MISSION_ITEM_REACHED_push_into_##DST ( pmission_item_reached_MISSION_ITEM_REACHED * src, DST * dst) {\
        DST##_seq_SET( pmission_item_reached_seq_GET( src  ), dst  );\
    }

/**
*An ack for a LOGGING_DATA_ACKED message */

typedef Pack LOGGING_ACK_logging_ack; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plogging_ack_LOGGING_ACK;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plogging_ack_LOGGING_ACK *plogging_ack_LOGGING_ACK_from(LOGGING_ACK_logging_ack *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_sequence_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define plogging_ack_LOGGING_ACK_PUSH_INTO(DST)\
    static inline void plogging_ack_LOGGING_ACK_push_into_##DST ( plogging_ack_LOGGING_ACK * src, DST * dst) {\
        DST##_sequence_SET( plogging_ack_sequence_GET( src  ), dst  );\
        DST##_target_system_SET( plogging_ack_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( plogging_ack_target_component_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_sequence_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
*/

#define plogging_ack_LOGGING_ACK_PULL_FROM(SRC)\
    static inline void plogging_ack_LOGGING_ACK_pull_from_##SRC ( SRC * src, plogging_ack_LOGGING_ACK * dst) {\
        plogging_ack_sequence_SET( SRC##_sequence_GET(src ), dst  );\
        plogging_ack_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        plogging_ack_target_component_SET( SRC##_target_component_GET(src ), dst  );\
    }


typedef Pack VISION_SPEED_ESTIMATE_vision_speed_estimate; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pvision_speed_estimate_VISION_SPEED_ESTIMATE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pvision_speed_estimate_VISION_SPEED_ESTIMATE *pvision_speed_estimate_VISION_SPEED_ESTIMATE_from(VISION_SPEED_ESTIMATE_vision_speed_estimate *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
*/

#define pvision_speed_estimate_VISION_SPEED_ESTIMATE_PUSH_INTO(DST)\
    static inline void pvision_speed_estimate_VISION_SPEED_ESTIMATE_push_into_##DST ( pvision_speed_estimate_VISION_SPEED_ESTIMATE * src, DST * dst) {\
        DST##_usec_SET( pvision_speed_estimate_usec_GET( src  ), dst  );\
        DST##_x_SET( pvision_speed_estimate_x_GET( src  ), dst  );\
        DST##_y_SET( pvision_speed_estimate_y_GET( src  ), dst  );\
        DST##_z_SET( pvision_speed_estimate_z_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_usec_GET( <SRC> * src ){}
static inline float <SRC>_x_GET( <SRC> * src ){}
static inline float <SRC>_y_GET( <SRC> * src ){}
static inline float <SRC>_z_GET( <SRC> * src ){}
*/

#define pvision_speed_estimate_VISION_SPEED_ESTIMATE_PULL_FROM(SRC)\
    static inline void pvision_speed_estimate_VISION_SPEED_ESTIMATE_pull_from_##SRC ( SRC * src, pvision_speed_estimate_VISION_SPEED_ESTIMATE * dst) {\
        pvision_speed_estimate_usec_SET( SRC##_usec_GET(src ), dst  );\
        pvision_speed_estimate_x_SET( SRC##_x_GET(src ), dst  );\
        pvision_speed_estimate_y_SET( SRC##_y_GET(src ), dst  );\
        pvision_speed_estimate_z_SET( SRC##_z_GET(src ), dst  );\
    }


typedef Pack DEBUG_VECT_debug_vect; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pdebug_vect_DEBUG_VECT;// data navigator over pack fields data
/**
															* Wrap DEBUG_VECT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pdebug_vect_DEBUG_VECT *DEBUG_VECT_debug_vect_wrap(DEBUG_VECT_debug_vect *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline DEBUG_VECT_debug_vect *pdebug_vect_DEBUG_VECT_unwrap(pdebug_vect_DEBUG_VECT *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdebug_vect_name;
//Maximum field array length constant
#define Pdebug_vect_name_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_name_SET( Vdebug_vect_name * src, <DST> * dst  ){}
*/

#define pdebug_vect_DEBUG_VECT_PUSH_INTO(DST)\
    static inline void pdebug_vect_DEBUG_VECT_push_into_##DST ( pdebug_vect_DEBUG_VECT * src, DST * dst) {\
        DST##_time_usec_SET( pdebug_vect_time_usec_GET( src  ), dst  );\
        DST##_x_SET( pdebug_vect_x_GET( src  ), dst  );\
        DST##_y_SET( pdebug_vect_y_GET( src  ), dst  );\
        DST##_z_SET( pdebug_vect_z_GET( src  ), dst  );\
        Vdebug_vect_name  item_name;\
        if( pdebug_vect_name_GET( src, &item_name ) ){\
            DST##_name_SET( &item_name, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_x_GET( <SRC> * src ){}
static inline float <SRC>_y_GET( <SRC> * src ){}
static inline float <SRC>_z_GET( <SRC> * src ){}
static inline size_t  <SRC>_name_item_exist( <SRC> * src  ){}
static inline void <SRC>_name_GET( <SRC> * src, Vdebug_vect_name * dst ){}
*/

#define pdebug_vect_DEBUG_VECT_PULL_FROM(SRC)\
    static inline void pdebug_vect_DEBUG_VECT_pull_from_##SRC ( SRC * src, pdebug_vect_DEBUG_VECT * dst) {\
        pdebug_vect_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pdebug_vect_x_SET( SRC##_x_GET(src ), dst  );\
        pdebug_vect_y_SET( SRC##_y_GET(src ), dst  );\
        pdebug_vect_z_SET( SRC##_z_GET(src ), dst  );\
        const size_t len_name = SRC##_name_item_exist(src );\
        if( len_name ){\
            Vdebug_vect_name    item_name = pdebug_vect_name_SET( NULL, len_name, dst  );\
            pdebug_vect_DEBUG_VECT_name_GET(src, &item_name );\
        }\
    }

/**
*Stop log transfer and resume normal logging */

typedef Pack LOG_REQUEST_END_log_request_end; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plog_request_end_LOG_REQUEST_END;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plog_request_end_LOG_REQUEST_END *plog_request_end_LOG_REQUEST_END_from(LOG_REQUEST_END_log_request_end *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define plog_request_end_LOG_REQUEST_END_PUSH_INTO(DST)\
    static inline void plog_request_end_LOG_REQUEST_END_push_into_##DST ( plog_request_end_LOG_REQUEST_END * src, DST * dst) {\
        DST##_target_system_SET( plog_request_end_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( plog_request_end_target_component_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
*/

#define plog_request_end_LOG_REQUEST_END_PULL_FROM(SRC)\
    static inline void plog_request_end_LOG_REQUEST_END_pull_from_##SRC ( SRC * src, plog_request_end_LOG_REQUEST_END * dst) {\
        plog_request_end_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        plog_request_end_target_component_SET( SRC##_target_component_GET(src ), dst  );\
    }

/**
*Ack message during waypoint handling. The type field states if this message is a positive ack (type=0)
*				 or if an error happened (type=non-zero) */

typedef Pack MISSION_ACK_mission_ack; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_ack_MISSION_ACK;// data navigator over pack fields data
/**
															* Wrap MISSION_ACK in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_ack_MISSION_ACK *MISSION_ACK_mission_ack_wrap(MISSION_ACK_mission_ack *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_ACK_mission_ack *pmission_ack_MISSION_ACK_unwrap(pmission_ack_MISSION_ACK *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_typE_SET( e_MAV_MISSION_RESULT * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_ack_MISSION_ACK_PUSH_INTO(DST)\
    static inline void pmission_ack_MISSION_ACK_push_into_##DST ( pmission_ack_MISSION_ACK * src, DST * dst) {\
        DST##_target_system_SET( pmission_ack_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_ack_target_component_GET( src  ), dst  );\
        e_MAV_MISSION_RESULT  item_typE;\
        if( pmission_ack_typE_GET( src, &item_typE ) ){\
            DST##_typE_SET( item_typE , dst  );\
        }\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_ack_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*Accept / deny control of this MAV */

typedef Pack CHANGE_OPERATOR_CONTROL_ACK_change_operator_control_ack; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK_from(CHANGE_OPERATOR_CONTROL_ACK_change_operator_control_ack *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_gcs_system_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_control_request_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_ack_SET( int8_t * src, <DST> * dst  ){}
*/

#define pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK_PUSH_INTO(DST)\
    static inline void pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK_push_into_##DST ( pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK * src, DST * dst) {\
        DST##_gcs_system_id_SET( pchange_operator_control_ack_gcs_system_id_GET( src  ), dst  );\
        DST##_control_request_SET( pchange_operator_control_ack_control_request_GET( src  ), dst  );\
        DST##_ack_SET( pchange_operator_control_ack_ack_GET( src  ), dst  );\
    }

/**
*Message that announces the sequence number of the current active mission item. The MAV will fly towards
*				 this mission item */

typedef Pack MISSION_CURRENT_mission_current; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmission_current_MISSION_CURRENT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmission_current_MISSION_CURRENT *pmission_current_MISSION_CURRENT_from(MISSION_CURRENT_mission_current *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int16_t * src, <DST> * dst  ){}
*/

#define pmission_current_MISSION_CURRENT_PUSH_INTO(DST)\
    static inline void pmission_current_MISSION_CURRENT_push_into_##DST ( pmission_current_MISSION_CURRENT * src, DST * dst) {\
        DST##_seq_SET( pmission_current_seq_GET( src  ), dst  );\
    }

/**
*The system time is the time of the master clock, typically the computer clock of the main onboard computer */

typedef Pack SYSTEM_TIME_system_time; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t psystem_time_SYSTEM_TIME;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline psystem_time_SYSTEM_TIME *psystem_time_SYSTEM_TIME_from(SYSTEM_TIME_system_time *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_unix_usec_SET( int64_t * src, <DST> * dst  ){}
*/

#define psystem_time_SYSTEM_TIME_PUSH_INTO(DST)\
    static inline void psystem_time_SYSTEM_TIME_push_into_##DST ( psystem_time_SYSTEM_TIME * src, DST * dst) {\
        DST##_time_boot_ms_SET( psystem_time_time_boot_ms_GET( src  ), dst  );\
        DST##_time_unix_usec_SET( psystem_time_time_unix_usec_GET( src  ), dst  );\
    }

/**
*Camera-IMU triggering and synchronisation message. */

typedef Pack CAMERA_TRIGGER_camera_trigger; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pcamera_trigger_CAMERA_TRIGGER;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pcamera_trigger_CAMERA_TRIGGER *pcamera_trigger_CAMERA_TRIGGER_from(CAMERA_TRIGGER_camera_trigger *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
*/

#define pcamera_trigger_CAMERA_TRIGGER_PUSH_INTO(DST)\
    static inline void pcamera_trigger_CAMERA_TRIGGER_push_into_##DST ( pcamera_trigger_CAMERA_TRIGGER * src, DST * dst) {\
        DST##_seq_SET( pcamera_trigger_seq_GET( src  ), dst  );\
        DST##_time_usec_SET( pcamera_trigger_time_usec_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_seq_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
*/

#define pcamera_trigger_CAMERA_TRIGGER_PULL_FROM(SRC)\
    static inline void pcamera_trigger_CAMERA_TRIGGER_pull_from_##SRC ( SRC * src, pcamera_trigger_CAMERA_TRIGGER * dst) {\
        pcamera_trigger_seq_SET( SRC##_seq_GET(src ), dst  );\
        pcamera_trigger_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
    }

/**
*Response from a GOPRO_COMMAND set request */

typedef Pack GOPRO_SET_RESPONSE_gopro_set_response; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgopro_set_response_GOPRO_SET_RESPONSE;// data navigator over pack fields data
/**
															* Wrap GOPRO_SET_RESPONSE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgopro_set_response_GOPRO_SET_RESPONSE *GOPRO_SET_RESPONSE_gopro_set_response_wrap(GOPRO_SET_RESPONSE_gopro_set_response *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GOPRO_SET_RESPONSE_gopro_set_response *pgopro_set_response_GOPRO_SET_RESPONSE_unwrap(pgopro_set_response_GOPRO_SET_RESPONSE *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_cmd_id_SET( e_GOPRO_COMMAND * src, <DST> * dst  ){}
static inline void <DST>_status_SET( e_GOPRO_REQUEST_STATUS * src, <DST> * dst  ){}
*/

#define pgopro_set_response_GOPRO_SET_RESPONSE_PUSH_INTO(DST)\
    static inline void pgopro_set_response_GOPRO_SET_RESPONSE_push_into_##DST ( pgopro_set_response_GOPRO_SET_RESPONSE * src, DST * dst) {\
        e_GOPRO_COMMAND  item_cmd_id;\
        if( pgopro_set_response_cmd_id_GET( src, &item_cmd_id ) ){\
            DST##_cmd_id_SET( item_cmd_id , dst  );\
        }\
        e_GOPRO_REQUEST_STATUS  item_status;\
        if( pgopro_set_response_status_GET( src, &item_status ) ){\
            DST##_status_SET( item_status , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline bool  <SRC>_cmd_id_item_exist( <SRC> * src ){}
static inline e_GOPRO_COMMAND <SRC>_cmd_id_GET( <SRC> * src ){}
static inline bool  <SRC>_status_item_exist( <SRC> * src ){}
static inline e_GOPRO_REQUEST_STATUS <SRC>_status_GET( <SRC> * src ){}
*/

#define pgopro_set_response_GOPRO_SET_RESPONSE_PULL_FROM(SRC)\
    static inline void pgopro_set_response_GOPRO_SET_RESPONSE_pull_from_##SRC ( SRC * src, pgopro_set_response_GOPRO_SET_RESPONSE * dst) {\
        if( SRC##_cmd_id_item_exist(src ) )\
        pgopro_set_response_cmd_id_SET( pgopro_set_response_GOPRO_SET_RESPONSE_cmd_id_GET( src ),  dst  );\
        if( SRC##_status_item_exist(src ) )\
        pgopro_set_response_status_SET( pgopro_set_response_GOPRO_SET_RESPONSE_status_GET( src ),  dst  );\
    }


typedef Pack VISION_POSITION_ESTIMATE_vision_position_estimate; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pvision_position_estimate_VISION_POSITION_ESTIMATE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pvision_position_estimate_VISION_POSITION_ESTIMATE *pvision_position_estimate_VISION_POSITION_ESTIMATE_from(VISION_POSITION_ESTIMATE_vision_position_estimate *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
*/

#define pvision_position_estimate_VISION_POSITION_ESTIMATE_PUSH_INTO(DST)\
    static inline void pvision_position_estimate_VISION_POSITION_ESTIMATE_push_into_##DST ( pvision_position_estimate_VISION_POSITION_ESTIMATE * src, DST * dst) {\
        DST##_usec_SET( pvision_position_estimate_usec_GET( src  ), dst  );\
        DST##_x_SET( pvision_position_estimate_x_GET( src  ), dst  );\
        DST##_y_SET( pvision_position_estimate_y_GET( src  ), dst  );\
        DST##_z_SET( pvision_position_estimate_z_GET( src  ), dst  );\
        DST##_roll_SET( pvision_position_estimate_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pvision_position_estimate_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pvision_position_estimate_yaw_GET( src  ), dst  );\
    }

/**
*This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
*				 along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as
*				 boolean values of their */

typedef Pack MANUAL_CONTROL_manual_control; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmanual_control_MANUAL_CONTROL;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmanual_control_MANUAL_CONTROL *pmanual_control_MANUAL_CONTROL_from(MANUAL_CONTROL_manual_control *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_buttons_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_y_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_z_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_r_SET( int16_t * src, <DST> * dst  ){}
*/

#define pmanual_control_MANUAL_CONTROL_PUSH_INTO(DST)\
    static inline void pmanual_control_MANUAL_CONTROL_push_into_##DST ( pmanual_control_MANUAL_CONTROL * src, DST * dst) {\
        DST##_buttons_SET( pmanual_control_buttons_GET( src  ), dst  );\
        DST##_target_SET( pmanual_control_target_GET( src  ), dst  );\
        DST##_x_SET( pmanual_control_x_GET( src  ), dst  );\
        DST##_y_SET( pmanual_control_y_GET( src  ), dst  );\
        DST##_z_SET( pmanual_control_z_GET( src  ), dst  );\
        DST##_r_SET( pmanual_control_r_GET( src  ), dst  );\
    }

/**
*The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
*				 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification */

typedef Pack RC_CHANNELS_rc_channels; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t prc_channels_RC_CHANNELS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline prc_channels_RC_CHANNELS *prc_channels_RC_CHANNELS_from(RC_CHANNELS_rc_channels *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_chan1_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan2_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan3_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan4_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan5_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan6_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan7_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan8_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan9_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan10_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan11_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan12_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan13_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan14_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan15_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan16_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan17_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_chan18_raw_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_chancount_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_rssi_SET( int8_t * src, <DST> * dst  ){}
*/

#define prc_channels_RC_CHANNELS_PUSH_INTO(DST)\
    static inline void prc_channels_RC_CHANNELS_push_into_##DST ( prc_channels_RC_CHANNELS * src, DST * dst) {\
        DST##_chan1_raw_SET( prc_channels_chan1_raw_GET( src  ), dst  );\
        DST##_chan2_raw_SET( prc_channels_chan2_raw_GET( src  ), dst  );\
        DST##_chan3_raw_SET( prc_channels_chan3_raw_GET( src  ), dst  );\
        DST##_chan4_raw_SET( prc_channels_chan4_raw_GET( src  ), dst  );\
        DST##_chan5_raw_SET( prc_channels_chan5_raw_GET( src  ), dst  );\
        DST##_chan6_raw_SET( prc_channels_chan6_raw_GET( src  ), dst  );\
        DST##_chan7_raw_SET( prc_channels_chan7_raw_GET( src  ), dst  );\
        DST##_chan8_raw_SET( prc_channels_chan8_raw_GET( src  ), dst  );\
        DST##_chan9_raw_SET( prc_channels_chan9_raw_GET( src  ), dst  );\
        DST##_chan10_raw_SET( prc_channels_chan10_raw_GET( src  ), dst  );\
        DST##_chan11_raw_SET( prc_channels_chan11_raw_GET( src  ), dst  );\
        DST##_chan12_raw_SET( prc_channels_chan12_raw_GET( src  ), dst  );\
        DST##_chan13_raw_SET( prc_channels_chan13_raw_GET( src  ), dst  );\
        DST##_chan14_raw_SET( prc_channels_chan14_raw_GET( src  ), dst  );\
        DST##_chan15_raw_SET( prc_channels_chan15_raw_GET( src  ), dst  );\
        DST##_chan16_raw_SET( prc_channels_chan16_raw_GET( src  ), dst  );\
        DST##_chan17_raw_SET( prc_channels_chan17_raw_GET( src  ), dst  );\
        DST##_chan18_raw_SET( prc_channels_chan18_raw_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( prc_channels_time_boot_ms_GET( src  ), dst  );\
        DST##_chancount_SET( prc_channels_chancount_GET( src  ), dst  );\
        DST##_rssi_SET( prc_channels_rssi_GET( src  ), dst  );\
    }

/**
*WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION
*				 and is used as part of the handshaking to establish which MAVLink version should be used on the network.
*				 Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers
*				 should consider adding this into the default decoding state machine to allow the protocol core to respond
*				 directly */

typedef Pack PROTOCOL_VERSION_protocol_version; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pprotocol_version_PROTOCOL_VERSION;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pprotocol_version_PROTOCOL_VERSION *pprotocol_version_PROTOCOL_VERSION_from(PROTOCOL_VERSION_protocol_version *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vprotocol_version_spec_version_hash;
//Maximum field array length constant
#define Pprotocol_version_spec_version_hash_len  ( 8 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vprotocol_version_library_version_hash;
//Maximum field array length constant
#define Pprotocol_version_library_version_hash_len  ( 8 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_version_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_min_version_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_max_version_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_spec_version_hash_SET( Vprotocol_version_spec_version_hash * src, <DST> * dst  ){}
static inline void <DST>_library_version_hash_SET( Vprotocol_version_library_version_hash * src, <DST> * dst  ){}
*/

#define pprotocol_version_PROTOCOL_VERSION_PUSH_INTO(DST)\
    static inline void pprotocol_version_PROTOCOL_VERSION_push_into_##DST ( pprotocol_version_PROTOCOL_VERSION * src, DST * dst) {\
        DST##_version_SET( pprotocol_version_version_GET( src  ), dst  );\
        DST##_min_version_SET( pprotocol_version_min_version_GET( src  ), dst  );\
        DST##_max_version_SET( pprotocol_version_max_version_GET( src  ), dst  );\
        Vprotocol_version_spec_version_hash item_spec_version_hash = pprotocol_version_spec_version_hash_GET( src  );\
       DST##_spec_version_hash_SET( &item_spec_version_hash, dst );\
        Vprotocol_version_library_version_hash item_library_version_hash = pprotocol_version_library_version_hash_GET( src  );\
       DST##_library_version_hash_SET( &item_library_version_hash, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_version_GET( <SRC> * src ){}
static inline int16_t <SRC>_min_version_GET( <SRC> * src ){}
static inline int16_t <SRC>_max_version_GET( <SRC> * src ){}
static inline int8_t <SRC>_spec_version_hash_GET( <SRC> * src, Vprotocol_version_spec_version_hash * dst ){}
static inline int8_t <SRC>_library_version_hash_GET( <SRC> * src, Vprotocol_version_library_version_hash * dst ){}
*/

#define pprotocol_version_PROTOCOL_VERSION_PULL_FROM(SRC)\
    static inline void pprotocol_version_PROTOCOL_VERSION_pull_from_##SRC ( SRC * src, pprotocol_version_PROTOCOL_VERSION * dst) {\
        pprotocol_version_version_SET( SRC##_version_GET(src ), dst  );\
        pprotocol_version_min_version_SET( SRC##_min_version_GET(src ), dst  );\
        pprotocol_version_max_version_SET( SRC##_max_version_GET(src ), dst  );\
       Vprotocol_version_spec_version_hash item_spec_version_hash = pprotocol_version_spec_version_hash_SET( NULL, dst  );\
       SRC##_spec_version_hash_GET( src, &item_spec_version_hash );\
       Vprotocol_version_library_version_hash item_library_version_hash = pprotocol_version_library_version_hash_SET( NULL, dst  );\
       SRC##_library_version_hash_GET( src, &item_library_version_hash );\
    }

/**
*Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not
*				 respond if the request is invalid */

typedef Pack RALLY_FETCH_POINT_rally_fetch_point; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t prally_fetch_point_RALLY_FETCH_POINT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline prally_fetch_point_RALLY_FETCH_POINT *prally_fetch_point_RALLY_FETCH_POINT_from(RALLY_FETCH_POINT_rally_fetch_point *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_idx_SET( int8_t * src, <DST> * dst  ){}
*/

#define prally_fetch_point_RALLY_FETCH_POINT_PUSH_INTO(DST)\
    static inline void prally_fetch_point_RALLY_FETCH_POINT_push_into_##DST ( prally_fetch_point_RALLY_FETCH_POINT * src, DST * dst) {\
        DST##_target_system_SET( prally_fetch_point_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( prally_fetch_point_target_component_GET( src  ), dst  );\
        DST##_idx_SET( prally_fetch_point_idx_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_idx_GET( <SRC> * src ){}
*/

#define prally_fetch_point_RALLY_FETCH_POINT_PULL_FROM(SRC)\
    static inline void prally_fetch_point_RALLY_FETCH_POINT_pull_from_##SRC ( SRC * src, prally_fetch_point_RALLY_FETCH_POINT * dst) {\
        prally_fetch_point_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        prally_fetch_point_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        prally_fetch_point_idx_SET( SRC##_idx_GET(src ), dst  );\
    }

/**
*Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
*				 the recipient to keep track of received parameters and allows him to re-request missing parameters after
*				 a loss or timeout */

typedef Pack PARAM_VALUE_param_value; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pparam_value_PARAM_VALUE;// data navigator over pack fields data
/**
															* Wrap PARAM_VALUE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pparam_value_PARAM_VALUE *PARAM_VALUE_param_value_wrap(PARAM_VALUE_param_value *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PARAM_VALUE_param_value *pparam_value_PARAM_VALUE_unwrap(pparam_value_PARAM_VALUE *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_value_param_id;
//Maximum field array length constant
#define Pparam_value_param_id_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_param_count_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_param_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_param_value_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param_id_SET( Vparam_value_param_id * src, <DST> * dst  ){}
static inline void <DST>_param_type_SET( e_MAV_PARAM_TYPE * src, <DST> * dst  ){}
*/

#define pparam_value_PARAM_VALUE_PUSH_INTO(DST)\
    static inline void pparam_value_PARAM_VALUE_push_into_##DST ( pparam_value_PARAM_VALUE * src, DST * dst) {\
        DST##_param_count_SET( pparam_value_param_count_GET( src  ), dst  );\
        DST##_param_index_SET( pparam_value_param_index_GET( src  ), dst  );\
        DST##_param_value_SET( pparam_value_param_value_GET( src  ), dst  );\
        Vparam_value_param_id  item_param_id;\
        if( pparam_value_param_id_GET( src, &item_param_id ) ){\
            DST##_param_id_SET( &item_param_id, dst );\
        }\
        e_MAV_PARAM_TYPE  item_param_type;\
        if( pparam_value_param_type_GET( src, &item_param_type ) ){\
            DST##_param_type_SET( item_param_type , dst  );\
        }\
    }

/**
*Battery information */

typedef Pack BATTERY_STATUS_battery_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pbattery_status_BATTERY_STATUS;// data navigator over pack fields data
/**
															* Wrap BATTERY_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pbattery_status_BATTERY_STATUS *BATTERY_STATUS_battery_status_wrap(BATTERY_STATUS_battery_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline BATTERY_STATUS_battery_status *pbattery_status_BATTERY_STATUS_unwrap(pbattery_status_BATTERY_STATUS *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vbattery_status_voltages;
//Maximum field array length constant
#define Pbattery_status_voltages_len  ( 10 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_voltages_SET( Vbattery_status_voltages * src, <DST> * dst  ){}
static inline void <DST>_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_current_battery_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_current_consumed_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_energy_consumed_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_battery_remaining_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_battery_function_SET( e_MAV_BATTERY_FUNCTION * src, <DST> * dst  ){}
static inline void <DST>_typE_SET( e_MAV_BATTERY_TYPE * src, <DST> * dst  ){}
*/

#define pbattery_status_BATTERY_STATUS_PUSH_INTO(DST)\
    static inline void pbattery_status_BATTERY_STATUS_push_into_##DST ( pbattery_status_BATTERY_STATUS * src, DST * dst) {\
        Vbattery_status_voltages item_voltages = pbattery_status_voltages_GET( src  );\
       DST##_voltages_SET( &item_voltages, dst );\
        DST##_id_SET( pbattery_status_id_GET( src  ), dst  );\
        DST##_temperature_SET( pbattery_status_temperature_GET( src  ), dst  );\
        DST##_current_battery_SET( pbattery_status_current_battery_GET( src  ), dst  );\
        DST##_current_consumed_SET( pbattery_status_current_consumed_GET( src  ), dst  );\
        DST##_energy_consumed_SET( pbattery_status_energy_consumed_GET( src  ), dst  );\
        DST##_battery_remaining_SET( pbattery_status_battery_remaining_GET( src  ), dst  );\
        e_MAV_BATTERY_FUNCTION  item_battery_function;\
        if( pbattery_status_battery_function_GET( src, &item_battery_function ) ){\
            DST##_battery_function_SET( item_battery_function , dst  );\
        }\
        e_MAV_BATTERY_TYPE  item_typE;\
        if( pbattery_status_typE_GET( src, &item_typE ) ){\
            DST##_typE_SET( item_typE , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_voltages_GET( <SRC> * src, Vbattery_status_voltages * dst ){}
static inline int8_t <SRC>_id_GET( <SRC> * src ){}
static inline int16_t <SRC>_temperature_GET( <SRC> * src ){}
static inline int16_t <SRC>_current_battery_GET( <SRC> * src ){}
static inline int32_t <SRC>_current_consumed_GET( <SRC> * src ){}
static inline int32_t <SRC>_energy_consumed_GET( <SRC> * src ){}
static inline int8_t <SRC>_battery_remaining_GET( <SRC> * src ){}
static inline bool  <SRC>_battery_function_item_exist( <SRC> * src ){}
static inline e_MAV_BATTERY_FUNCTION <SRC>_battery_function_GET( <SRC> * src ){}
static inline bool  <SRC>_typE_item_exist( <SRC> * src ){}
static inline e_MAV_BATTERY_TYPE <SRC>_typE_GET( <SRC> * src ){}
*/

#define pbattery_status_BATTERY_STATUS_PULL_FROM(SRC)\
    static inline void pbattery_status_BATTERY_STATUS_pull_from_##SRC ( SRC * src, pbattery_status_BATTERY_STATUS * dst) {\
       Vbattery_status_voltages item_voltages = pbattery_status_voltages_SET( NULL, dst  );\
       SRC##_voltages_GET( src, &item_voltages );\
        pbattery_status_id_SET( SRC##_id_GET(src ), dst  );\
        pbattery_status_temperature_SET( SRC##_temperature_GET(src ), dst  );\
        pbattery_status_current_battery_SET( SRC##_current_battery_GET(src ), dst  );\
        pbattery_status_current_consumed_SET( SRC##_current_consumed_GET(src ), dst  );\
        pbattery_status_energy_consumed_SET( SRC##_energy_consumed_GET(src ), dst  );\
        pbattery_status_battery_remaining_SET( SRC##_battery_remaining_GET(src ), dst  );\
        if( SRC##_battery_function_item_exist(src ) )\
        pbattery_status_battery_function_SET( pbattery_status_BATTERY_STATUS_battery_function_GET( src ),  dst  );\
        if( SRC##_typE_item_exist(src ) )\
        pbattery_status_typE_SET( pbattery_status_BATTERY_STATUS_typE_GET( src ),  dst  );\
    }

/**
*Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
*				 telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
*				 or change the devices settings. A message with zero bytes can be used to change just the baudrate */

typedef Pack SERIAL_CONTROL_serial_control; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pserial_control_SERIAL_CONTROL;// data navigator over pack fields data
/**
															* Wrap SERIAL_CONTROL in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pserial_control_SERIAL_CONTROL *SERIAL_CONTROL_serial_control_wrap(SERIAL_CONTROL_serial_control *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SERIAL_CONTROL_serial_control *pserial_control_SERIAL_CONTROL_unwrap(pserial_control_SERIAL_CONTROL *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vserial_control_daTa;
//Maximum field array length constant
#define Pserial_control_daTa_len  ( 70 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_timeout_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_baudrate_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_count_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vserial_control_daTa * src, <DST> * dst  ){}
static inline void <DST>_device_SET( e_SERIAL_CONTROL_DEV * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( e_SERIAL_CONTROL_FLAG * src, <DST> * dst  ){}
*/

#define pserial_control_SERIAL_CONTROL_PUSH_INTO(DST)\
    static inline void pserial_control_SERIAL_CONTROL_push_into_##DST ( pserial_control_SERIAL_CONTROL * src, DST * dst) {\
        DST##_timeout_SET( pserial_control_timeout_GET( src  ), dst  );\
        DST##_baudrate_SET( pserial_control_baudrate_GET( src  ), dst  );\
        DST##_count_SET( pserial_control_count_GET( src  ), dst  );\
        Vserial_control_daTa item_daTa = pserial_control_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
        e_SERIAL_CONTROL_DEV  item_device;\
        if( pserial_control_device_GET( src, &item_device ) ){\
            DST##_device_SET( item_device , dst  );\
        }\
        e_SERIAL_CONTROL_FLAG  item_flags;\
        if( pserial_control_flags_GET( src, &item_flags ) ){\
            DST##_flags_SET( item_flags , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_timeout_GET( <SRC> * src ){}
static inline int32_t <SRC>_baudrate_GET( <SRC> * src ){}
static inline int8_t <SRC>_count_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vserial_control_daTa * dst ){}
static inline bool  <SRC>_device_item_exist( <SRC> * src ){}
static inline e_SERIAL_CONTROL_DEV <SRC>_device_GET( <SRC> * src ){}
static inline bool  <SRC>_flags_item_exist( <SRC> * src ){}
static inline e_SERIAL_CONTROL_FLAG <SRC>_flags_GET( <SRC> * src ){}
*/

#define pserial_control_SERIAL_CONTROL_PULL_FROM(SRC)\
    static inline void pserial_control_SERIAL_CONTROL_pull_from_##SRC ( SRC * src, pserial_control_SERIAL_CONTROL * dst) {\
        pserial_control_timeout_SET( SRC##_timeout_GET(src ), dst  );\
        pserial_control_baudrate_SET( SRC##_baudrate_GET(src ), dst  );\
        pserial_control_count_SET( SRC##_count_GET(src ), dst  );\
       Vserial_control_daTa item_daTa = pserial_control_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
        if( SRC##_device_item_exist(src ) )\
        pserial_control_device_SET( pserial_control_SERIAL_CONTROL_device_GET( src ),  dst  );\
        if( SRC##_flags_item_exist(src ) )\
        pserial_control_flags_SET( pserial_control_SERIAL_CONTROL_flags_GET( src ),  dst  );\
    }

/**
*Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
*				 to command the vehicle (manual controller or other system) */

typedef Pack SET_POSITION_TARGET_LOCAL_NED_set_position_target_local_ned; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED;// data navigator over pack fields data
/**
															* Wrap SET_POSITION_TARGET_LOCAL_NED in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *SET_POSITION_TARGET_LOCAL_NED_set_position_target_local_ned_wrap(SET_POSITION_TARGET_LOCAL_NED_set_position_target_local_ned *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SET_POSITION_TARGET_LOCAL_NED_set_position_target_local_ned *pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED_unwrap(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_type_mask_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_coordinate_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
*/

#define pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED_PUSH_INTO(DST)\
    static inline void pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED_push_into_##DST ( pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED * src, DST * dst) {\
        DST##_type_mask_SET( pset_position_target_local_ned_type_mask_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( pset_position_target_local_ned_time_boot_ms_GET( src  ), dst  );\
        DST##_target_system_SET( pset_position_target_local_ned_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pset_position_target_local_ned_target_component_GET( src  ), dst  );\
        DST##_x_SET( pset_position_target_local_ned_x_GET( src  ), dst  );\
        DST##_y_SET( pset_position_target_local_ned_y_GET( src  ), dst  );\
        DST##_z_SET( pset_position_target_local_ned_z_GET( src  ), dst  );\
        DST##_vx_SET( pset_position_target_local_ned_vx_GET( src  ), dst  );\
        DST##_vy_SET( pset_position_target_local_ned_vy_GET( src  ), dst  );\
        DST##_vz_SET( pset_position_target_local_ned_vz_GET( src  ), dst  );\
        DST##_afx_SET( pset_position_target_local_ned_afx_GET( src  ), dst  );\
        DST##_afy_SET( pset_position_target_local_ned_afy_GET( src  ), dst  );\
        DST##_afz_SET( pset_position_target_local_ned_afz_GET( src  ), dst  );\
        DST##_yaw_SET( pset_position_target_local_ned_yaw_GET( src  ), dst  );\
        DST##_yaw_rate_SET( pset_position_target_local_ned_yaw_rate_GET( src  ), dst  );\
        e_MAV_FRAME  item_coordinate_frame;\
        if( pset_position_target_local_ned_coordinate_frame_GET( src, &item_coordinate_frame ) ){\
            DST##_coordinate_frame_SET( item_coordinate_frame , dst  );\
        }\
    }


typedef Pack MOUNT_ORIENTATION_mount_orientation; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pmount_orientation_MOUNT_ORIENTATION;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pmount_orientation_MOUNT_ORIENTATION *pmount_orientation_MOUNT_ORIENTATION_from(MOUNT_ORIENTATION_mount_orientation *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
*/

#define pmount_orientation_MOUNT_ORIENTATION_PUSH_INTO(DST)\
    static inline void pmount_orientation_MOUNT_ORIENTATION_push_into_##DST ( pmount_orientation_MOUNT_ORIENTATION * src, DST * dst) {\
        DST##_time_boot_ms_SET( pmount_orientation_time_boot_ms_GET( src  ), dst  );\
        DST##_roll_SET( pmount_orientation_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pmount_orientation_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pmount_orientation_yaw_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline float <SRC>_roll_GET( <SRC> * src ){}
static inline float <SRC>_pitch_GET( <SRC> * src ){}
static inline float <SRC>_yaw_GET( <SRC> * src ){}
*/

#define pmount_orientation_MOUNT_ORIENTATION_PULL_FROM(SRC)\
    static inline void pmount_orientation_MOUNT_ORIENTATION_pull_from_##SRC ( SRC * src, pmount_orientation_MOUNT_ORIENTATION * dst) {\
        pmount_orientation_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pmount_orientation_roll_SET( SRC##_roll_GET(src ), dst  );\
        pmount_orientation_pitch_SET( SRC##_pitch_GET(src ), dst  );\
        pmount_orientation_yaw_SET( SRC##_yaw_GET(src ), dst  );\
    }

/**
*As local waypoints exist, the global waypoint reference allows to transform between the local coordinate
*				 frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings
*				 are connected and the MAV should move from in- to outdoor */

typedef Pack SET_GPS_GLOBAL_ORIGIN_set_gps_global_origin; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN;// data navigator over pack fields data
/**
															* Wrap SET_GPS_GLOBAL_ORIGIN in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *SET_GPS_GLOBAL_ORIGIN_set_gps_global_origin_wrap(SET_GPS_GLOBAL_ORIGIN_set_gps_global_origin *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SET_GPS_GLOBAL_ORIGIN_set_gps_global_origin *pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN_unwrap(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_latitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_longitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
*/

#define pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN_PUSH_INTO(DST)\
    static inline void pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN_push_into_##DST ( pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN * src, DST * dst) {\
        DST##_target_system_SET( pset_gps_global_origin_target_system_GET( src  ), dst  );\
        DST##_latitude_SET( pset_gps_global_origin_latitude_GET( src  ), dst  );\
        DST##_longitude_SET( pset_gps_global_origin_longitude_GET( src  ), dst  );\
        DST##_altitude_SET( pset_gps_global_origin_altitude_GET( src  ), dst  );\
        int64_t  item_time_usec;\
        if( pset_gps_global_origin_time_usec_GET( src, &item_time_usec ) ){\
            DST##_time_usec_SET( item_time_usec , dst  );\
        }\
    }

/**
*Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
*				 setting a parameter value and the new value is the same as the current value, you will immediately get
*				 a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
*				 a PARAM_ACK_IN_PROGRESS in response */

typedef Pack PARAM_EXT_SET_param_ext_set; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pparam_ext_set_PARAM_EXT_SET;// data navigator over pack fields data
/**
															* Wrap PARAM_EXT_SET in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pparam_ext_set_PARAM_EXT_SET *PARAM_EXT_SET_param_ext_set_wrap(PARAM_EXT_SET_param_ext_set *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PARAM_EXT_SET_param_ext_set *pparam_ext_set_PARAM_EXT_SET_unwrap(pparam_ext_set_PARAM_EXT_SET *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_ext_set_param_id;
//Maximum field array length constant
#define Pparam_ext_set_param_id_len_max  ( 255 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_ext_set_param_value;
//Maximum field array length constant
#define Pparam_ext_set_param_value_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param_id_SET( Vparam_ext_set_param_id * src, <DST> * dst  ){}
static inline void <DST>_param_value_SET( Vparam_ext_set_param_value * src, <DST> * dst  ){}
static inline void <DST>_param_type_SET( e_MAV_PARAM_EXT_TYPE * src, <DST> * dst  ){}
*/

#define pparam_ext_set_PARAM_EXT_SET_PUSH_INTO(DST)\
    static inline void pparam_ext_set_PARAM_EXT_SET_push_into_##DST ( pparam_ext_set_PARAM_EXT_SET * src, DST * dst) {\
        DST##_target_system_SET( pparam_ext_set_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pparam_ext_set_target_component_GET( src  ), dst  );\
        Vparam_ext_set_param_id  item_param_id;\
        if( pparam_ext_set_param_id_GET( src, &item_param_id ) ){\
            DST##_param_id_SET( &item_param_id, dst );\
        }\
        Vparam_ext_set_param_value  item_param_value;\
        if( pparam_ext_set_param_value_GET( src, &item_param_value ) ){\
            DST##_param_value_SET( &item_param_value, dst );\
        }\
        e_MAV_PARAM_EXT_TYPE  item_param_type;\
        if( pparam_ext_set_param_type_GET( src, &item_param_type ) ){\
            DST##_param_type_SET( item_param_type , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline size_t  <SRC>_param_id_item_exist( <SRC> * src  ){}
static inline void <SRC>_param_id_GET( <SRC> * src, Vparam_ext_set_param_id * dst ){}
static inline size_t  <SRC>_param_value_item_exist( <SRC> * src  ){}
static inline void <SRC>_param_value_GET( <SRC> * src, Vparam_ext_set_param_value * dst ){}
static inline bool  <SRC>_param_type_item_exist( <SRC> * src ){}
static inline e_MAV_PARAM_EXT_TYPE <SRC>_param_type_GET( <SRC> * src ){}
*/

#define pparam_ext_set_PARAM_EXT_SET_PULL_FROM(SRC)\
    static inline void pparam_ext_set_PARAM_EXT_SET_pull_from_##SRC ( SRC * src, pparam_ext_set_PARAM_EXT_SET * dst) {\
        pparam_ext_set_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pparam_ext_set_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        const size_t len_param_id = SRC##_param_id_item_exist(src );\
        if( len_param_id ){\
            Vparam_ext_set_param_id    item_param_id = pparam_ext_set_param_id_SET( NULL, len_param_id, dst  );\
            pparam_ext_set_PARAM_EXT_SET_param_id_GET(src, &item_param_id );\
        }\
        const size_t len_param_value = SRC##_param_value_item_exist(src );\
        if( len_param_value ){\
            Vparam_ext_set_param_value    item_param_value = pparam_ext_set_param_value_SET( NULL, len_param_value, dst  );\
            pparam_ext_set_PARAM_EXT_SET_param_value_GET(src, &item_param_value );\
        }\
        if( SRC##_param_type_item_exist(src ) )\
        pparam_ext_set_param_type_SET( pparam_ext_set_PARAM_EXT_SET_param_type_GET( src ),  dst  );\
    }

/**
*Version and capability of autopilot software */

typedef Pack AUTOPILOT_VERSION_autopilot_version; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pautopilot_version_AUTOPILOT_VERSION;// data navigator over pack fields data
/**
															* Wrap AUTOPILOT_VERSION in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pautopilot_version_AUTOPILOT_VERSION *AUTOPILOT_VERSION_autopilot_version_wrap(AUTOPILOT_VERSION_autopilot_version *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline AUTOPILOT_VERSION_autopilot_version *pautopilot_version_AUTOPILOT_VERSION_unwrap(pautopilot_version_AUTOPILOT_VERSION *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vautopilot_version_flight_custom_version;
//Maximum field array length constant
#define Pautopilot_version_flight_custom_version_len  ( 8 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vautopilot_version_middleware_custom_version;
//Maximum field array length constant
#define Pautopilot_version_middleware_custom_version_len  ( 8 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vautopilot_version_os_custom_version;
//Maximum field array length constant
#define Pautopilot_version_os_custom_version_len  ( 8 )

/**
																					* This value struct hold information about none primitive field parameters and used by functions to access field data
																					*/
typedef BytesValue Vautopilot_version_uid2;

#define pautopilot_version_uid2_d0(pautopilot_version_AUTOPILOT_VERSION_ptr)\
    for( size_t  d0=0 ; d0 <SIZE_MAX; d0 =SIZE_MAX )\
        for( Qautopilot_version_uid2 * fld_uid2 = pautopilot_version_uid2( pautopilot_version_AUTOPILOT_VERSION_ptr ); d0 <SIZE_MAX && fld_uid2; d0 = SIZE_MAX )\
                    for( Vautopilot_version_uid2  item_uid2 ; d0 < Pautopilot_version_uid2_D0  && qautopilot_version_uid2_GET( fld_uid2, &item_uid2 , d0 ) ; d0++)
//Constant dimension size value
#define Pautopilot_version_uid2_D0 ( 18)

/**
													*Optional field current state information. Used in functions to access field data.
													*/
typedef pautopilot_version_AUTOPILOT_VERSION Qautopilot_version_uid2;

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_vendor_id_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_product_id_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_flight_sw_version_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_middleware_sw_version_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_os_sw_version_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_board_version_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_uid_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_flight_custom_version_SET( Vautopilot_version_flight_custom_version * src, <DST> * dst  ){}
static inline void <DST>_middleware_custom_version_SET( Vautopilot_version_middleware_custom_version * src, <DST> * dst  ){}
static inline void <DST>_os_custom_version_SET( Vautopilot_version_os_custom_version * src, <DST> * dst  ){}
static inline void <DST>_capabilities_SET( e_MAV_PROTOCOL_CAPABILITY * src, <DST> * dst  ){}
static inline void <DST>_uid2_SET( int8_t * src, <DST> * dst , size_t d0 ){}
*/

#define pautopilot_version_AUTOPILOT_VERSION_PUSH_INTO(DST)\
    static inline void pautopilot_version_AUTOPILOT_VERSION_push_into_##DST ( pautopilot_version_AUTOPILOT_VERSION * src, DST * dst) {\
        DST##_vendor_id_SET( pautopilot_version_vendor_id_GET( src  ), dst  );\
        DST##_product_id_SET( pautopilot_version_product_id_GET( src  ), dst  );\
        DST##_flight_sw_version_SET( pautopilot_version_flight_sw_version_GET( src  ), dst  );\
        DST##_middleware_sw_version_SET( pautopilot_version_middleware_sw_version_GET( src  ), dst  );\
        DST##_os_sw_version_SET( pautopilot_version_os_sw_version_GET( src  ), dst  );\
        DST##_board_version_SET( pautopilot_version_board_version_GET( src  ), dst  );\
        DST##_uid_SET( pautopilot_version_uid_GET( src  ), dst  );\
        Vautopilot_version_flight_custom_version item_flight_custom_version = pautopilot_version_flight_custom_version_GET( src  );\
       DST##_flight_custom_version_SET( &item_flight_custom_version, dst );\
        Vautopilot_version_middleware_custom_version item_middleware_custom_version = pautopilot_version_middleware_custom_version_GET( src  );\
       DST##_middleware_custom_version_SET( &item_middleware_custom_version, dst );\
        Vautopilot_version_os_custom_version item_os_custom_version = pautopilot_version_os_custom_version_GET( src  );\
       DST##_os_custom_version_SET( &item_os_custom_version, dst );\
        e_MAV_PROTOCOL_CAPABILITY  item_capabilities;\
        if( pautopilot_version_capabilities_GET( src, &item_capabilities ) ){\
            DST##_capabilities_SET( item_capabilities , dst  );\
        }\
        pautopilot_version_uid2_d0 (src) { \
        DST##_uid2_SET( vautopilot_version_uid2_GET( &item_uid2 ), dst , d0 );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_vendor_id_GET( <SRC> * src ){}
static inline int16_t <SRC>_product_id_GET( <SRC> * src ){}
static inline int32_t <SRC>_flight_sw_version_GET( <SRC> * src ){}
static inline int32_t <SRC>_middleware_sw_version_GET( <SRC> * src ){}
static inline int32_t <SRC>_os_sw_version_GET( <SRC> * src ){}
static inline int32_t <SRC>_board_version_GET( <SRC> * src ){}
static inline int64_t <SRC>_uid_GET( <SRC> * src ){}
static inline int8_t <SRC>_flight_custom_version_GET( <SRC> * src, Vautopilot_version_flight_custom_version * dst ){}
static inline int8_t <SRC>_middleware_custom_version_GET( <SRC> * src, Vautopilot_version_middleware_custom_version * dst ){}
static inline int8_t <SRC>_os_custom_version_GET( <SRC> * src, Vautopilot_version_os_custom_version * dst ){}
static inline bool  <SRC>_capabilities_item_exist( <SRC> * src ){}
static inline e_MAV_PROTOCOL_CAPABILITY <SRC>_capabilities_GET( <SRC> * src ){}
static inline bool <SRC>_uid2_exist( <SRC> * src){}
static inline bool  <SRC>_uid2_item_exist( <SRC> * src , size_t d0){}
static inline int8_t <SRC>_uid2_GET( <SRC> * src , size_t d0){}
*/

#define pautopilot_version_AUTOPILOT_VERSION_PULL_FROM(SRC)\
    static inline void pautopilot_version_AUTOPILOT_VERSION_pull_from_##SRC ( SRC * src, pautopilot_version_AUTOPILOT_VERSION * dst) {\
        pautopilot_version_vendor_id_SET( SRC##_vendor_id_GET(src ), dst  );\
        pautopilot_version_product_id_SET( SRC##_product_id_GET(src ), dst  );\
        pautopilot_version_flight_sw_version_SET( SRC##_flight_sw_version_GET(src ), dst  );\
        pautopilot_version_middleware_sw_version_SET( SRC##_middleware_sw_version_GET(src ), dst  );\
        pautopilot_version_os_sw_version_SET( SRC##_os_sw_version_GET(src ), dst  );\
        pautopilot_version_board_version_SET( SRC##_board_version_GET(src ), dst  );\
        pautopilot_version_uid_SET( SRC##_uid_GET(src ), dst  );\
       Vautopilot_version_flight_custom_version item_flight_custom_version = pautopilot_version_flight_custom_version_SET( NULL, dst  );\
       SRC##_flight_custom_version_GET( src, &item_flight_custom_version );\
       Vautopilot_version_middleware_custom_version item_middleware_custom_version = pautopilot_version_middleware_custom_version_SET( NULL, dst  );\
       SRC##_middleware_custom_version_GET( src, &item_middleware_custom_version );\
       Vautopilot_version_os_custom_version item_os_custom_version = pautopilot_version_os_custom_version_SET( NULL, dst  );\
       SRC##_os_custom_version_GET( src, &item_os_custom_version );\
        if( SRC##_capabilities_item_exist(src ) )\
        pautopilot_version_capabilities_SET( pautopilot_version_AUTOPILOT_VERSION_capabilities_GET( src ),  dst  );\
        if( SRC##_uid2_exist( src ))\
                for( size_t d0=0; d0 < Pautopilot_version_uid2_D0 ; d0++)\
           {\
        if( SRC##_uid2_item_exist(src , d0) )\
    pautopilot_version_uid2_SET( pautopilot_version_AUTOPILOT_VERSION_uid2_GET( src , d0),  dst , d0 );\
        }\
    }

/**
*Request the overall list of mission items from the system/component. */

typedef Pack MISSION_REQUEST_LIST_mission_request_list; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_request_list_MISSION_REQUEST_LIST;// data navigator over pack fields data
/**
															* Wrap MISSION_REQUEST_LIST in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_request_list_MISSION_REQUEST_LIST *MISSION_REQUEST_LIST_mission_request_list_wrap(MISSION_REQUEST_LIST_mission_request_list *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_REQUEST_LIST_mission_request_list *pmission_request_list_MISSION_REQUEST_LIST_unwrap(pmission_request_list_MISSION_REQUEST_LIST *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_request_list_MISSION_REQUEST_LIST_PUSH_INTO(DST)\
    static inline void pmission_request_list_MISSION_REQUEST_LIST_push_into_##DST ( pmission_request_list_MISSION_REQUEST_LIST * src, DST * dst) {\
        DST##_target_system_SET( pmission_request_list_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_request_list_target_component_GET( src  ), dst  );\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_request_list_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*Status of simulation environment, if used */

typedef Pack SIMSTATE_simstate; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t psimstate_SIMSTATE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline psimstate_SIMSTATE *psimstate_SIMSTATE_from(SIMSTATE_simstate *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ygyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_lng_SET( int32_t * src, <DST> * dst  ){}
*/

#define psimstate_SIMSTATE_PUSH_INTO(DST)\
    static inline void psimstate_SIMSTATE_push_into_##DST ( psimstate_SIMSTATE * src, DST * dst) {\
        DST##_roll_SET( psimstate_roll_GET( src  ), dst  );\
        DST##_pitch_SET( psimstate_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( psimstate_yaw_GET( src  ), dst  );\
        DST##_xacc_SET( psimstate_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( psimstate_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( psimstate_zacc_GET( src  ), dst  );\
        DST##_xgyro_SET( psimstate_xgyro_GET( src  ), dst  );\
        DST##_ygyro_SET( psimstate_ygyro_GET( src  ), dst  );\
        DST##_zgyro_SET( psimstate_zgyro_GET( src  ), dst  );\
        DST##_lat_SET( psimstate_lat_GET( src  ), dst  );\
        DST##_lng_SET( psimstate_lng_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_roll_GET( <SRC> * src ){}
static inline float <SRC>_pitch_GET( <SRC> * src ){}
static inline float <SRC>_yaw_GET( <SRC> * src ){}
static inline float <SRC>_xacc_GET( <SRC> * src ){}
static inline float <SRC>_yacc_GET( <SRC> * src ){}
static inline float <SRC>_zacc_GET( <SRC> * src ){}
static inline float <SRC>_xgyro_GET( <SRC> * src ){}
static inline float <SRC>_ygyro_GET( <SRC> * src ){}
static inline float <SRC>_zgyro_GET( <SRC> * src ){}
static inline int32_t <SRC>_lat_GET( <SRC> * src ){}
static inline int32_t <SRC>_lng_GET( <SRC> * src ){}
*/

#define psimstate_SIMSTATE_PULL_FROM(SRC)\
    static inline void psimstate_SIMSTATE_pull_from_##SRC ( SRC * src, psimstate_SIMSTATE * dst) {\
        psimstate_roll_SET( SRC##_roll_GET(src ), dst  );\
        psimstate_pitch_SET( SRC##_pitch_GET(src ), dst  );\
        psimstate_yaw_SET( SRC##_yaw_GET(src ), dst  );\
        psimstate_xacc_SET( SRC##_xacc_GET(src ), dst  );\
        psimstate_yacc_SET( SRC##_yacc_GET(src ), dst  );\
        psimstate_zacc_SET( SRC##_zacc_GET(src ), dst  );\
        psimstate_xgyro_SET( SRC##_xgyro_GET(src ), dst  );\
        psimstate_ygyro_SET( SRC##_ygyro_GET(src ), dst  );\
        psimstate_zgyro_SET( SRC##_zgyro_GET(src ), dst  );\
        psimstate_lat_SET( SRC##_lat_GET(src ), dst  );\
        psimstate_lng_SET( SRC##_lng_GET(src ), dst  );\
    }

/**
*WIP: Message that sets video stream settings */

typedef Pack SET_VIDEO_STREAM_SETTINGS_set_video_stream_settings; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS;// data navigator over pack fields data
/**
															* Wrap SET_VIDEO_STREAM_SETTINGS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *SET_VIDEO_STREAM_SETTINGS_set_video_stream_settings_wrap(SET_VIDEO_STREAM_SETTINGS_set_video_stream_settings *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SET_VIDEO_STREAM_SETTINGS_set_video_stream_settings *pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS_unwrap(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vset_video_stream_settings_uri;
//Maximum field array length constant
#define Pset_video_stream_settings_uri_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_resolution_h_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_resolution_v_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_rotation_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_bitrate_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_camera_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_framerate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_uri_SET( Vset_video_stream_settings_uri * src, <DST> * dst  ){}
*/

#define pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS_PUSH_INTO(DST)\
    static inline void pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS_push_into_##DST ( pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS * src, DST * dst) {\
        DST##_resolution_h_SET( pset_video_stream_settings_resolution_h_GET( src  ), dst  );\
        DST##_resolution_v_SET( pset_video_stream_settings_resolution_v_GET( src  ), dst  );\
        DST##_rotation_SET( pset_video_stream_settings_rotation_GET( src  ), dst  );\
        DST##_bitrate_SET( pset_video_stream_settings_bitrate_GET( src  ), dst  );\
        DST##_target_system_SET( pset_video_stream_settings_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pset_video_stream_settings_target_component_GET( src  ), dst  );\
        DST##_camera_id_SET( pset_video_stream_settings_camera_id_GET( src  ), dst  );\
        DST##_framerate_SET( pset_video_stream_settings_framerate_GET( src  ), dst  );\
        Vset_video_stream_settings_uri  item_uri;\
        if( pset_video_stream_settings_uri_GET( src, &item_uri ) ){\
            DST##_uri_SET( &item_uri, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_resolution_h_GET( <SRC> * src ){}
static inline int16_t <SRC>_resolution_v_GET( <SRC> * src ){}
static inline int16_t <SRC>_rotation_GET( <SRC> * src ){}
static inline int32_t <SRC>_bitrate_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_camera_id_GET( <SRC> * src ){}
static inline float <SRC>_framerate_GET( <SRC> * src ){}
static inline size_t  <SRC>_uri_item_exist( <SRC> * src  ){}
static inline void <SRC>_uri_GET( <SRC> * src, Vset_video_stream_settings_uri * dst ){}
*/

#define pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS_PULL_FROM(SRC)\
    static inline void pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS_pull_from_##SRC ( SRC * src, pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS * dst) {\
        pset_video_stream_settings_resolution_h_SET( SRC##_resolution_h_GET(src ), dst  );\
        pset_video_stream_settings_resolution_v_SET( SRC##_resolution_v_GET(src ), dst  );\
        pset_video_stream_settings_rotation_SET( SRC##_rotation_GET(src ), dst  );\
        pset_video_stream_settings_bitrate_SET( SRC##_bitrate_GET(src ), dst  );\
        pset_video_stream_settings_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pset_video_stream_settings_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pset_video_stream_settings_camera_id_SET( SRC##_camera_id_GET(src ), dst  );\
        pset_video_stream_settings_framerate_SET( SRC##_framerate_GET(src ), dst  );\
        const size_t len_uri = SRC##_uri_item_exist(src );\
        if( len_uri ){\
            Vset_video_stream_settings_uri    item_uri = pset_video_stream_settings_uri_SET( NULL, len_uri, dst  );\
            pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS_uri_GET(src, &item_uri );\
        }\
    }

/**
*Control vehicle tone generation (buzzer) */

typedef Pack PLAY_TUNE_play_tune; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pplay_tune_PLAY_TUNE;// data navigator over pack fields data
/**
															* Wrap PLAY_TUNE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pplay_tune_PLAY_TUNE *PLAY_TUNE_play_tune_wrap(PLAY_TUNE_play_tune *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PLAY_TUNE_play_tune *pplay_tune_PLAY_TUNE_unwrap(pplay_tune_PLAY_TUNE *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vplay_tune_tune;
//Maximum field array length constant
#define Pplay_tune_tune_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_tune_SET( Vplay_tune_tune * src, <DST> * dst  ){}
*/

#define pplay_tune_PLAY_TUNE_PUSH_INTO(DST)\
    static inline void pplay_tune_PLAY_TUNE_push_into_##DST ( pplay_tune_PLAY_TUNE * src, DST * dst) {\
        DST##_target_system_SET( pplay_tune_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pplay_tune_target_component_GET( src  ), dst  );\
        Vplay_tune_tune  item_tune;\
        if( pplay_tune_tune_GET( src, &item_tune ) ){\
            DST##_tune_SET( &item_tune, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline size_t  <SRC>_tune_item_exist( <SRC> * src  ){}
static inline void <SRC>_tune_GET( <SRC> * src, Vplay_tune_tune * dst ){}
*/

#define pplay_tune_PLAY_TUNE_PULL_FROM(SRC)\
    static inline void pplay_tune_PLAY_TUNE_pull_from_##SRC ( SRC * src, pplay_tune_PLAY_TUNE * dst) {\
        pplay_tune_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pplay_tune_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        const size_t len_tune = SRC##_tune_item_exist(src );\
        if( len_tune ){\
            Vplay_tune_tune    item_tune = pplay_tune_tune_SET( NULL, len_tune, dst  );\
            pplay_tune_PLAY_TUNE_tune_GET(src, &item_tune );\
        }\
    }

/**
*Configure on-board Camera Control System. */

typedef Pack DIGICAM_CONFIGURE_digicam_configure; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdigicam_configure_DIGICAM_CONFIGURE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdigicam_configure_DIGICAM_CONFIGURE *pdigicam_configure_DIGICAM_CONFIGURE_from(DIGICAM_CONFIGURE_digicam_configure *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_shutter_speed_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mode_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_aperture_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_iso_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_exposure_type_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_command_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_engine_cut_off_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_extra_param_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_extra_value_SET( float * src, <DST> * dst  ){}
*/

#define pdigicam_configure_DIGICAM_CONFIGURE_PUSH_INTO(DST)\
    static inline void pdigicam_configure_DIGICAM_CONFIGURE_push_into_##DST ( pdigicam_configure_DIGICAM_CONFIGURE * src, DST * dst) {\
        DST##_shutter_speed_SET( pdigicam_configure_shutter_speed_GET( src  ), dst  );\
        DST##_target_system_SET( pdigicam_configure_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pdigicam_configure_target_component_GET( src  ), dst  );\
        DST##_mode_SET( pdigicam_configure_mode_GET( src  ), dst  );\
        DST##_aperture_SET( pdigicam_configure_aperture_GET( src  ), dst  );\
        DST##_iso_SET( pdigicam_configure_iso_GET( src  ), dst  );\
        DST##_exposure_type_SET( pdigicam_configure_exposure_type_GET( src  ), dst  );\
        DST##_command_id_SET( pdigicam_configure_command_id_GET( src  ), dst  );\
        DST##_engine_cut_off_SET( pdigicam_configure_engine_cut_off_GET( src  ), dst  );\
        DST##_extra_param_SET( pdigicam_configure_extra_param_GET( src  ), dst  );\
        DST##_extra_value_SET( pdigicam_configure_extra_value_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_shutter_speed_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_mode_GET( <SRC> * src ){}
static inline int8_t <SRC>_aperture_GET( <SRC> * src ){}
static inline int8_t <SRC>_iso_GET( <SRC> * src ){}
static inline int8_t <SRC>_exposure_type_GET( <SRC> * src ){}
static inline int8_t <SRC>_command_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_engine_cut_off_GET( <SRC> * src ){}
static inline int8_t <SRC>_extra_param_GET( <SRC> * src ){}
static inline float <SRC>_extra_value_GET( <SRC> * src ){}
*/

#define pdigicam_configure_DIGICAM_CONFIGURE_PULL_FROM(SRC)\
    static inline void pdigicam_configure_DIGICAM_CONFIGURE_pull_from_##SRC ( SRC * src, pdigicam_configure_DIGICAM_CONFIGURE * dst) {\
        pdigicam_configure_shutter_speed_SET( SRC##_shutter_speed_GET(src ), dst  );\
        pdigicam_configure_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pdigicam_configure_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pdigicam_configure_mode_SET( SRC##_mode_GET(src ), dst  );\
        pdigicam_configure_aperture_SET( SRC##_aperture_GET(src ), dst  );\
        pdigicam_configure_iso_SET( SRC##_iso_GET(src ), dst  );\
        pdigicam_configure_exposure_type_SET( SRC##_exposure_type_GET(src ), dst  );\
        pdigicam_configure_command_id_SET( SRC##_command_id_GET(src ), dst  );\
        pdigicam_configure_engine_cut_off_SET( SRC##_engine_cut_off_GET(src ), dst  );\
        pdigicam_configure_extra_param_SET( SRC##_extra_param_GET(src ), dst  );\
        pdigicam_configure_extra_value_SET( SRC##_extra_value_GET(src ), dst  );\
    }

/**
*Barometer readings for 3rd barometer */

typedef Pack SCALED_PRESSURE3_scaled_pressure3; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pscaled_pressure3_SCALED_PRESSURE3;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pscaled_pressure3_SCALED_PRESSURE3 *pscaled_pressure3_SCALED_PRESSURE3_from(SCALED_PRESSURE3_scaled_pressure3 *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_press_abs_SET( float * src, <DST> * dst  ){}
static inline void <DST>_press_diff_SET( float * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( int16_t * src, <DST> * dst  ){}
*/

#define pscaled_pressure3_SCALED_PRESSURE3_PUSH_INTO(DST)\
    static inline void pscaled_pressure3_SCALED_PRESSURE3_push_into_##DST ( pscaled_pressure3_SCALED_PRESSURE3 * src, DST * dst) {\
        DST##_time_boot_ms_SET( pscaled_pressure3_time_boot_ms_GET( src  ), dst  );\
        DST##_press_abs_SET( pscaled_pressure3_press_abs_GET( src  ), dst  );\
        DST##_press_diff_SET( pscaled_pressure3_press_diff_GET( src  ), dst  );\
        DST##_temperature_SET( pscaled_pressure3_temperature_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline float <SRC>_press_abs_GET( <SRC> * src ){}
static inline float <SRC>_press_diff_GET( <SRC> * src ){}
static inline int16_t <SRC>_temperature_GET( <SRC> * src ){}
*/

#define pscaled_pressure3_SCALED_PRESSURE3_PULL_FROM(SRC)\
    static inline void pscaled_pressure3_SCALED_PRESSURE3_pull_from_##SRC ( SRC * src, pscaled_pressure3_SCALED_PRESSURE3 * dst) {\
        pscaled_pressure3_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pscaled_pressure3_press_abs_SET( SRC##_press_abs_GET(src ), dst  );\
        pscaled_pressure3_press_diff_SET( SRC##_press_diff_GET(src ), dst  );\
        pscaled_pressure3_temperature_SET( SRC##_temperature_GET(src ), dst  );\
    }

/**
*Request a partial list of mission items from the system/component. http:qgroundcontrol.org/mavlink/waypoint_protocol.
*				 If start and end index are the same, just send one waypoint */

typedef Pack MISSION_REQUEST_PARTIAL_LIST_mission_request_partial_list; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST;// data navigator over pack fields data
/**
															* Wrap MISSION_REQUEST_PARTIAL_LIST in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *MISSION_REQUEST_PARTIAL_LIST_mission_request_partial_list_wrap(MISSION_REQUEST_PARTIAL_LIST_mission_request_partial_list *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_REQUEST_PARTIAL_LIST_mission_request_partial_list *pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST_unwrap(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_start_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_end_index_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST_PUSH_INTO(DST)\
    static inline void pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST_push_into_##DST ( pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST * src, DST * dst) {\
        DST##_target_system_SET( pmission_request_partial_list_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_request_partial_list_target_component_GET( src  ), dst  );\
        DST##_start_index_SET( pmission_request_partial_list_start_index_GET( src  ), dst  );\
        DST##_end_index_SET( pmission_request_partial_list_end_index_GET( src  ), dst  );\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_request_partial_list_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*Response from a PARAM_EXT_SET message. */

typedef Pack PARAM_EXT_ACK_param_ext_ack; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pparam_ext_ack_PARAM_EXT_ACK;// data navigator over pack fields data
/**
															* Wrap PARAM_EXT_ACK in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pparam_ext_ack_PARAM_EXT_ACK *PARAM_EXT_ACK_param_ext_ack_wrap(PARAM_EXT_ACK_param_ext_ack *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline PARAM_EXT_ACK_param_ext_ack *pparam_ext_ack_PARAM_EXT_ACK_unwrap(pparam_ext_ack_PARAM_EXT_ACK *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_ext_ack_param_id;
//Maximum field array length constant
#define Pparam_ext_ack_param_id_len_max  ( 255 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vparam_ext_ack_param_value;
//Maximum field array length constant
#define Pparam_ext_ack_param_value_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_param_id_SET( Vparam_ext_ack_param_id * src, <DST> * dst  ){}
static inline void <DST>_param_value_SET( Vparam_ext_ack_param_value * src, <DST> * dst  ){}
static inline void <DST>_param_type_SET( e_MAV_PARAM_EXT_TYPE * src, <DST> * dst  ){}
static inline void <DST>_param_result_SET( e_PARAM_ACK * src, <DST> * dst  ){}
*/

#define pparam_ext_ack_PARAM_EXT_ACK_PUSH_INTO(DST)\
    static inline void pparam_ext_ack_PARAM_EXT_ACK_push_into_##DST ( pparam_ext_ack_PARAM_EXT_ACK * src, DST * dst) {\
        Vparam_ext_ack_param_id  item_param_id;\
        if( pparam_ext_ack_param_id_GET( src, &item_param_id ) ){\
            DST##_param_id_SET( &item_param_id, dst );\
        }\
        Vparam_ext_ack_param_value  item_param_value;\
        if( pparam_ext_ack_param_value_GET( src, &item_param_value ) ){\
            DST##_param_value_SET( &item_param_value, dst );\
        }\
        e_MAV_PARAM_EXT_TYPE  item_param_type;\
        if( pparam_ext_ack_param_type_GET( src, &item_param_type ) ){\
            DST##_param_type_SET( item_param_type , dst  );\
        }\
        e_PARAM_ACK  item_param_result;\
        if( pparam_ext_ack_param_result_GET( src, &item_param_result ) ){\
            DST##_param_result_SET( item_param_result , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline size_t  <SRC>_param_id_item_exist( <SRC> * src  ){}
static inline void <SRC>_param_id_GET( <SRC> * src, Vparam_ext_ack_param_id * dst ){}
static inline size_t  <SRC>_param_value_item_exist( <SRC> * src  ){}
static inline void <SRC>_param_value_GET( <SRC> * src, Vparam_ext_ack_param_value * dst ){}
static inline bool  <SRC>_param_type_item_exist( <SRC> * src ){}
static inline e_MAV_PARAM_EXT_TYPE <SRC>_param_type_GET( <SRC> * src ){}
static inline bool  <SRC>_param_result_item_exist( <SRC> * src ){}
static inline e_PARAM_ACK <SRC>_param_result_GET( <SRC> * src ){}
*/

#define pparam_ext_ack_PARAM_EXT_ACK_PULL_FROM(SRC)\
    static inline void pparam_ext_ack_PARAM_EXT_ACK_pull_from_##SRC ( SRC * src, pparam_ext_ack_PARAM_EXT_ACK * dst) {\
        const size_t len_param_id = SRC##_param_id_item_exist(src );\
        if( len_param_id ){\
            Vparam_ext_ack_param_id    item_param_id = pparam_ext_ack_param_id_SET( NULL, len_param_id, dst  );\
            pparam_ext_ack_PARAM_EXT_ACK_param_id_GET(src, &item_param_id );\
        }\
        const size_t len_param_value = SRC##_param_value_item_exist(src );\
        if( len_param_value ){\
            Vparam_ext_ack_param_value    item_param_value = pparam_ext_ack_param_value_SET( NULL, len_param_value, dst  );\
            pparam_ext_ack_PARAM_EXT_ACK_param_value_GET(src, &item_param_value );\
        }\
        if( SRC##_param_type_item_exist(src ) )\
        pparam_ext_ack_param_type_SET( pparam_ext_ack_PARAM_EXT_ACK_param_type_GET( src ),  dst  );\
        if( SRC##_param_result_item_exist(src ) )\
        pparam_ext_ack_param_result_SET( pparam_ext_ack_PARAM_EXT_ACK_param_result_GET( src ),  dst  );\
    }

/**
*General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
*				 service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
*				 by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
*				 emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
*				 is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
*				 is available at http:uavcan.org */

typedef Pack UAVCAN_NODE_INFO_uavcan_node_info; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor puavcan_node_info_UAVCAN_NODE_INFO;// data navigator over pack fields data
/**
															* Wrap UAVCAN_NODE_INFO in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline puavcan_node_info_UAVCAN_NODE_INFO *UAVCAN_NODE_INFO_uavcan_node_info_wrap(UAVCAN_NODE_INFO_uavcan_node_info *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline UAVCAN_NODE_INFO_uavcan_node_info *puavcan_node_info_UAVCAN_NODE_INFO_unwrap(puavcan_node_info_UAVCAN_NODE_INFO *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vuavcan_node_info_hw_unique_id;
//Maximum field array length constant
#define Puavcan_node_info_hw_unique_id_len  ( 16 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vuavcan_node_info_name;
//Maximum field array length constant
#define Puavcan_node_info_name_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_uptime_sec_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_sw_vcs_commit_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_hw_version_major_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_hw_version_minor_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_hw_unique_id_SET( Vuavcan_node_info_hw_unique_id * src, <DST> * dst  ){}
static inline void <DST>_sw_version_major_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_sw_version_minor_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_name_SET( Vuavcan_node_info_name * src, <DST> * dst  ){}
*/

#define puavcan_node_info_UAVCAN_NODE_INFO_PUSH_INTO(DST)\
    static inline void puavcan_node_info_UAVCAN_NODE_INFO_push_into_##DST ( puavcan_node_info_UAVCAN_NODE_INFO * src, DST * dst) {\
        DST##_uptime_sec_SET( puavcan_node_info_uptime_sec_GET( src  ), dst  );\
        DST##_sw_vcs_commit_SET( puavcan_node_info_sw_vcs_commit_GET( src  ), dst  );\
        DST##_time_usec_SET( puavcan_node_info_time_usec_GET( src  ), dst  );\
        DST##_hw_version_major_SET( puavcan_node_info_hw_version_major_GET( src  ), dst  );\
        DST##_hw_version_minor_SET( puavcan_node_info_hw_version_minor_GET( src  ), dst  );\
        Vuavcan_node_info_hw_unique_id item_hw_unique_id = puavcan_node_info_hw_unique_id_GET( src  );\
       DST##_hw_unique_id_SET( &item_hw_unique_id, dst );\
        DST##_sw_version_major_SET( puavcan_node_info_sw_version_major_GET( src  ), dst  );\
        DST##_sw_version_minor_SET( puavcan_node_info_sw_version_minor_GET( src  ), dst  );\
        Vuavcan_node_info_name  item_name;\
        if( puavcan_node_info_name_GET( src, &item_name ) ){\
            DST##_name_SET( &item_name, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_uptime_sec_GET( <SRC> * src ){}
static inline int32_t <SRC>_sw_vcs_commit_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_hw_version_major_GET( <SRC> * src ){}
static inline int8_t <SRC>_hw_version_minor_GET( <SRC> * src ){}
static inline int8_t <SRC>_hw_unique_id_GET( <SRC> * src, Vuavcan_node_info_hw_unique_id * dst ){}
static inline int8_t <SRC>_sw_version_major_GET( <SRC> * src ){}
static inline int8_t <SRC>_sw_version_minor_GET( <SRC> * src ){}
static inline size_t  <SRC>_name_item_exist( <SRC> * src  ){}
static inline void <SRC>_name_GET( <SRC> * src, Vuavcan_node_info_name * dst ){}
*/

#define puavcan_node_info_UAVCAN_NODE_INFO_PULL_FROM(SRC)\
    static inline void puavcan_node_info_UAVCAN_NODE_INFO_pull_from_##SRC ( SRC * src, puavcan_node_info_UAVCAN_NODE_INFO * dst) {\
        puavcan_node_info_uptime_sec_SET( SRC##_uptime_sec_GET(src ), dst  );\
        puavcan_node_info_sw_vcs_commit_SET( SRC##_sw_vcs_commit_GET(src ), dst  );\
        puavcan_node_info_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        puavcan_node_info_hw_version_major_SET( SRC##_hw_version_major_GET(src ), dst  );\
        puavcan_node_info_hw_version_minor_SET( SRC##_hw_version_minor_GET(src ), dst  );\
       Vuavcan_node_info_hw_unique_id item_hw_unique_id = puavcan_node_info_hw_unique_id_SET( NULL, dst  );\
       SRC##_hw_unique_id_GET( src, &item_hw_unique_id );\
        puavcan_node_info_sw_version_major_SET( SRC##_sw_version_major_GET(src ), dst  );\
        puavcan_node_info_sw_version_minor_SET( SRC##_sw_version_minor_GET(src ), dst  );\
        const size_t len_name = SRC##_name_item_exist(src );\
        if( len_name ){\
            Vuavcan_node_info_name    item_name = puavcan_node_info_name_SET( NULL, len_name, dst  );\
            puavcan_node_info_UAVCAN_NODE_INFO_name_GET(src, &item_name );\
        }\
    }

/**
*Data packet, size 16 */

typedef Pack DATA16_data16; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdata16_DATA16;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdata16_DATA16 *pdata16_DATA16_from(DATA16_data16 *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdata16_daTa;
//Maximum field array length constant
#define Pdata16_daTa_len  ( 16 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_typE_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_len_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vdata16_daTa * src, <DST> * dst  ){}
*/

#define pdata16_DATA16_PUSH_INTO(DST)\
    static inline void pdata16_DATA16_push_into_##DST ( pdata16_DATA16 * src, DST * dst) {\
        DST##_typE_SET( pdata16_typE_GET( src  ), dst  );\
        DST##_len_SET( pdata16_len_GET( src  ), dst  );\
        Vdata16_daTa item_daTa = pdata16_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_typE_GET( <SRC> * src ){}
static inline int8_t <SRC>_len_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vdata16_daTa * dst ){}
*/

#define pdata16_DATA16_PULL_FROM(SRC)\
    static inline void pdata16_DATA16_pull_from_##SRC ( SRC * src, pdata16_DATA16 * dst) {\
        pdata16_typE_SET( SRC##_typE_GET(src ), dst  );\
        pdata16_len_SET( SRC##_len_GET(src ), dst  );\
       Vdata16_daTa item_daTa = pdata16_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets */

typedef Pack SET_MAG_OFFSETS_set_mag_offsets; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pset_mag_offsets_SET_MAG_OFFSETS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pset_mag_offsets_SET_MAG_OFFSETS *pset_mag_offsets_SET_MAG_OFFSETS_from(SET_MAG_OFFSETS_set_mag_offsets *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_mag_ofs_x_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_mag_ofs_y_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_mag_ofs_z_SET( int16_t * src, <DST> * dst  ){}
*/

#define pset_mag_offsets_SET_MAG_OFFSETS_PUSH_INTO(DST)\
    static inline void pset_mag_offsets_SET_MAG_OFFSETS_push_into_##DST ( pset_mag_offsets_SET_MAG_OFFSETS * src, DST * dst) {\
        DST##_target_system_SET( pset_mag_offsets_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pset_mag_offsets_target_component_GET( src  ), dst  );\
        DST##_mag_ofs_x_SET( pset_mag_offsets_mag_ofs_x_GET( src  ), dst  );\
        DST##_mag_ofs_y_SET( pset_mag_offsets_mag_ofs_y_GET( src  ), dst  );\
        DST##_mag_ofs_z_SET( pset_mag_offsets_mag_ofs_z_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int16_t <SRC>_mag_ofs_x_GET( <SRC> * src ){}
static inline int16_t <SRC>_mag_ofs_y_GET( <SRC> * src ){}
static inline int16_t <SRC>_mag_ofs_z_GET( <SRC> * src ){}
*/

#define pset_mag_offsets_SET_MAG_OFFSETS_PULL_FROM(SRC)\
    static inline void pset_mag_offsets_SET_MAG_OFFSETS_pull_from_##SRC ( SRC * src, pset_mag_offsets_SET_MAG_OFFSETS * dst) {\
        pset_mag_offsets_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pset_mag_offsets_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pset_mag_offsets_mag_ofs_x_SET( SRC##_mag_ofs_x_GET(src ), dst  );\
        pset_mag_offsets_mag_ofs_y_SET( SRC##_mag_ofs_y_GET(src ), dst  );\
        pset_mag_offsets_mag_ofs_z_SET( SRC##_mag_ofs_z_GET(src ), dst  );\
    }

/**
*raw ADC output */

typedef Pack AP_ADC_ap_adc; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pap_adc_AP_ADC;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pap_adc_AP_ADC *pap_adc_AP_ADC_from(AP_ADC_ap_adc *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_adc1_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_adc2_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_adc3_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_adc4_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_adc5_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_adc6_SET( int16_t * src, <DST> * dst  ){}
*/

#define pap_adc_AP_ADC_PUSH_INTO(DST)\
    static inline void pap_adc_AP_ADC_push_into_##DST ( pap_adc_AP_ADC * src, DST * dst) {\
        DST##_adc1_SET( pap_adc_adc1_GET( src  ), dst  );\
        DST##_adc2_SET( pap_adc_adc2_GET( src  ), dst  );\
        DST##_adc3_SET( pap_adc_adc3_GET( src  ), dst  );\
        DST##_adc4_SET( pap_adc_adc4_GET( src  ), dst  );\
        DST##_adc5_SET( pap_adc_adc5_GET( src  ), dst  );\
        DST##_adc6_SET( pap_adc_adc6_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_adc1_GET( <SRC> * src ){}
static inline int16_t <SRC>_adc2_GET( <SRC> * src ){}
static inline int16_t <SRC>_adc3_GET( <SRC> * src ){}
static inline int16_t <SRC>_adc4_GET( <SRC> * src ){}
static inline int16_t <SRC>_adc5_GET( <SRC> * src ){}
static inline int16_t <SRC>_adc6_GET( <SRC> * src ){}
*/

#define pap_adc_AP_ADC_PULL_FROM(SRC)\
    static inline void pap_adc_AP_ADC_pull_from_##SRC ( SRC * src, pap_adc_AP_ADC * dst) {\
        pap_adc_adc1_SET( SRC##_adc1_GET(src ), dst  );\
        pap_adc_adc2_SET( SRC##_adc2_GET(src ), dst  );\
        pap_adc_adc3_SET( SRC##_adc3_GET(src ), dst  );\
        pap_adc_adc4_SET( SRC##_adc4_GET(src ), dst  );\
        pap_adc_adc5_SET( SRC##_adc5_GET(src ), dst  );\
        pap_adc_adc6_SET( SRC##_adc6_GET(src ), dst  );\
    }

/**
*Wind estimation */

typedef Pack WIND_wind; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pwind_WIND;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pwind_WIND *pwind_WIND_from(WIND_wind *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_direction_SET( float * src, <DST> * dst  ){}
static inline void <DST>_speed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_speed_z_SET( float * src, <DST> * dst  ){}
*/

#define pwind_WIND_PUSH_INTO(DST)\
    static inline void pwind_WIND_push_into_##DST ( pwind_WIND * src, DST * dst) {\
        DST##_direction_SET( pwind_direction_GET( src  ), dst  );\
        DST##_speed_SET( pwind_speed_GET( src  ), dst  );\
        DST##_speed_z_SET( pwind_speed_z_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_direction_GET( <SRC> * src ){}
static inline float <SRC>_speed_GET( <SRC> * src ){}
static inline float <SRC>_speed_z_GET( <SRC> * src ){}
*/

#define pwind_WIND_PULL_FROM(SRC)\
    static inline void pwind_WIND_pull_from_##SRC ( SRC * src, pwind_WIND * dst) {\
        pwind_direction_SET( SRC##_direction_GET(src ), dst  );\
        pwind_speed_SET( SRC##_speed_GET(src ), dst  );\
        pwind_speed_z_SET( SRC##_speed_z_GET(src ), dst  );\
    }

/**
*Request the autopilot version from the system/component. */

typedef Pack AUTOPILOT_VERSION_REQUEST_autopilot_version_request; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pautopilot_version_request_AUTOPILOT_VERSION_REQUEST;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *pautopilot_version_request_AUTOPILOT_VERSION_REQUEST_from(AUTOPILOT_VERSION_REQUEST_autopilot_version_request *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
*/

#define pautopilot_version_request_AUTOPILOT_VERSION_REQUEST_PUSH_INTO(DST)\
    static inline void pautopilot_version_request_AUTOPILOT_VERSION_REQUEST_push_into_##DST ( pautopilot_version_request_AUTOPILOT_VERSION_REQUEST * src, DST * dst) {\
        DST##_target_system_SET( pautopilot_version_request_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pautopilot_version_request_target_component_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
*/

#define pautopilot_version_request_AUTOPILOT_VERSION_REQUEST_PULL_FROM(SRC)\
    static inline void pautopilot_version_request_AUTOPILOT_VERSION_REQUEST_pull_from_##SRC ( SRC * src, pautopilot_version_request_AUTOPILOT_VERSION_REQUEST * dst) {\
        pautopilot_version_request_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pautopilot_version_request_target_component_SET( SRC##_target_component_GET(src ), dst  );\
    }

/**
*The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
*				 Z-axis down (aeronautical frame, NED / north-east-down convention */

typedef Pack LOCAL_POSITION_NED_local_position_ned; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plocal_position_ned_LOCAL_POSITION_NED;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plocal_position_ned_LOCAL_POSITION_NED *plocal_position_ned_LOCAL_POSITION_NED_from(LOCAL_POSITION_NED_local_position_ned *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( float * src, <DST> * dst  ){}
*/

#define plocal_position_ned_LOCAL_POSITION_NED_PUSH_INTO(DST)\
    static inline void plocal_position_ned_LOCAL_POSITION_NED_push_into_##DST ( plocal_position_ned_LOCAL_POSITION_NED * src, DST * dst) {\
        DST##_time_boot_ms_SET( plocal_position_ned_time_boot_ms_GET( src  ), dst  );\
        DST##_x_SET( plocal_position_ned_x_GET( src  ), dst  );\
        DST##_y_SET( plocal_position_ned_y_GET( src  ), dst  );\
        DST##_z_SET( plocal_position_ned_z_GET( src  ), dst  );\
        DST##_vx_SET( plocal_position_ned_vx_GET( src  ), dst  );\
        DST##_vy_SET( plocal_position_ned_vy_GET( src  ), dst  );\
        DST##_vz_SET( plocal_position_ned_vz_GET( src  ), dst  );\
    }


typedef Pack DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE_from(DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_width_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_height_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_packets_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_size_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_typE_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_payload_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_jpg_quality_SET( int8_t * src, <DST> * dst  ){}
*/

#define pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE_PUSH_INTO(DST)\
    static inline void pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE_push_into_##DST ( pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE * src, DST * dst) {\
        DST##_width_SET( pdata_transmission_handshake_width_GET( src  ), dst  );\
        DST##_height_SET( pdata_transmission_handshake_height_GET( src  ), dst  );\
        DST##_packets_SET( pdata_transmission_handshake_packets_GET( src  ), dst  );\
        DST##_size_SET( pdata_transmission_handshake_size_GET( src  ), dst  );\
        DST##_typE_SET( pdata_transmission_handshake_typE_GET( src  ), dst  );\
        DST##_payload_SET( pdata_transmission_handshake_payload_GET( src  ), dst  );\
        DST##_jpg_quality_SET( pdata_transmission_handshake_jpg_quality_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_width_GET( <SRC> * src ){}
static inline int16_t <SRC>_height_GET( <SRC> * src ){}
static inline int16_t <SRC>_packets_GET( <SRC> * src ){}
static inline int32_t <SRC>_size_GET( <SRC> * src ){}
static inline int8_t <SRC>_typE_GET( <SRC> * src ){}
static inline int8_t <SRC>_payload_GET( <SRC> * src ){}
static inline int8_t <SRC>_jpg_quality_GET( <SRC> * src ){}
*/

#define pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE_PULL_FROM(SRC)\
    static inline void pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE_pull_from_##SRC ( SRC * src, pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE * dst) {\
        pdata_transmission_handshake_width_SET( SRC##_width_GET(src ), dst  );\
        pdata_transmission_handshake_height_SET( SRC##_height_GET(src ), dst  );\
        pdata_transmission_handshake_packets_SET( SRC##_packets_GET(src ), dst  );\
        pdata_transmission_handshake_size_SET( SRC##_size_GET(src ), dst  );\
        pdata_transmission_handshake_typE_SET( SRC##_typE_GET(src ), dst  );\
        pdata_transmission_handshake_payload_SET( SRC##_payload_GET(src ), dst  );\
        pdata_transmission_handshake_jpg_quality_SET( SRC##_jpg_quality_GET(src ), dst  );\
    }

/**
*Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) positio */

typedef Pack GPS_GLOBAL_ORIGIN_gps_global_origin; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgps_global_origin_GPS_GLOBAL_ORIGIN;// data navigator over pack fields data
/**
															* Wrap GPS_GLOBAL_ORIGIN in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgps_global_origin_GPS_GLOBAL_ORIGIN *GPS_GLOBAL_ORIGIN_gps_global_origin_wrap(GPS_GLOBAL_ORIGIN_gps_global_origin *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GPS_GLOBAL_ORIGIN_gps_global_origin *pgps_global_origin_GPS_GLOBAL_ORIGIN_unwrap(pgps_global_origin_GPS_GLOBAL_ORIGIN *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_latitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_longitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_altitude_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
*/

#define pgps_global_origin_GPS_GLOBAL_ORIGIN_PUSH_INTO(DST)\
    static inline void pgps_global_origin_GPS_GLOBAL_ORIGIN_push_into_##DST ( pgps_global_origin_GPS_GLOBAL_ORIGIN * src, DST * dst) {\
        DST##_latitude_SET( pgps_global_origin_latitude_GET( src  ), dst  );\
        DST##_longitude_SET( pgps_global_origin_longitude_GET( src  ), dst  );\
        DST##_altitude_SET( pgps_global_origin_altitude_GET( src  ), dst  );\
        int64_t  item_time_usec;\
        if( pgps_global_origin_time_usec_GET( src, &item_time_usec ) ){\
            DST##_time_usec_SET( item_time_usec , dst  );\
        }\
    }

/**
*The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
*				 the described unit */

typedef Pack SCALED_IMU2_scaled_imu2; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pscaled_imu2_SCALED_IMU2;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pscaled_imu2_SCALED_IMU2 *pscaled_imu2_SCALED_IMU2_from(SCALED_IMU2_scaled_imu2 *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xgyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ygyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zgyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xmag_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ymag_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zmag_SET( int16_t * src, <DST> * dst  ){}
*/

#define pscaled_imu2_SCALED_IMU2_PUSH_INTO(DST)\
    static inline void pscaled_imu2_SCALED_IMU2_push_into_##DST ( pscaled_imu2_SCALED_IMU2 * src, DST * dst) {\
        DST##_time_boot_ms_SET( pscaled_imu2_time_boot_ms_GET( src  ), dst  );\
        DST##_xacc_SET( pscaled_imu2_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( pscaled_imu2_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( pscaled_imu2_zacc_GET( src  ), dst  );\
        DST##_xgyro_SET( pscaled_imu2_xgyro_GET( src  ), dst  );\
        DST##_ygyro_SET( pscaled_imu2_ygyro_GET( src  ), dst  );\
        DST##_zgyro_SET( pscaled_imu2_zgyro_GET( src  ), dst  );\
        DST##_xmag_SET( pscaled_imu2_xmag_GET( src  ), dst  );\
        DST##_ymag_SET( pscaled_imu2_ymag_GET( src  ), dst  );\
        DST##_zmag_SET( pscaled_imu2_zmag_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int16_t <SRC>_xacc_GET( <SRC> * src ){}
static inline int16_t <SRC>_yacc_GET( <SRC> * src ){}
static inline int16_t <SRC>_zacc_GET( <SRC> * src ){}
static inline int16_t <SRC>_xgyro_GET( <SRC> * src ){}
static inline int16_t <SRC>_ygyro_GET( <SRC> * src ){}
static inline int16_t <SRC>_zgyro_GET( <SRC> * src ){}
static inline int16_t <SRC>_xmag_GET( <SRC> * src ){}
static inline int16_t <SRC>_ymag_GET( <SRC> * src ){}
static inline int16_t <SRC>_zmag_GET( <SRC> * src ){}
*/

#define pscaled_imu2_SCALED_IMU2_PULL_FROM(SRC)\
    static inline void pscaled_imu2_SCALED_IMU2_pull_from_##SRC ( SRC * src, pscaled_imu2_SCALED_IMU2 * dst) {\
        pscaled_imu2_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pscaled_imu2_xacc_SET( SRC##_xacc_GET(src ), dst  );\
        pscaled_imu2_yacc_SET( SRC##_yacc_GET(src ), dst  );\
        pscaled_imu2_zacc_SET( SRC##_zacc_GET(src ), dst  );\
        pscaled_imu2_xgyro_SET( SRC##_xgyro_GET(src ), dst  );\
        pscaled_imu2_ygyro_SET( SRC##_ygyro_GET(src ), dst  );\
        pscaled_imu2_zgyro_SET( SRC##_zgyro_GET(src ), dst  );\
        pscaled_imu2_xmag_SET( SRC##_xmag_GET(src ), dst  );\
        pscaled_imu2_ymag_SET( SRC##_ymag_GET(src ), dst  );\
        pscaled_imu2_zmag_SET( SRC##_zmag_GET(src ), dst  );\
    }

/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
*				 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0) */

typedef Pack ATTITUDE_QUATERNION_attitude_quaternion; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pattitude_quaternion_ATTITUDE_QUATERNION;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pattitude_quaternion_ATTITUDE_QUATERNION *pattitude_quaternion_ATTITUDE_QUATERNION_from(ATTITUDE_QUATERNION_attitude_quaternion *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_q1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_q4_SET( float * src, <DST> * dst  ){}
static inline void <DST>_rollspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitchspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yawspeed_SET( float * src, <DST> * dst  ){}
*/

#define pattitude_quaternion_ATTITUDE_QUATERNION_PUSH_INTO(DST)\
    static inline void pattitude_quaternion_ATTITUDE_QUATERNION_push_into_##DST ( pattitude_quaternion_ATTITUDE_QUATERNION * src, DST * dst) {\
        DST##_time_boot_ms_SET( pattitude_quaternion_time_boot_ms_GET( src  ), dst  );\
        DST##_q1_SET( pattitude_quaternion_q1_GET( src  ), dst  );\
        DST##_q2_SET( pattitude_quaternion_q2_GET( src  ), dst  );\
        DST##_q3_SET( pattitude_quaternion_q3_GET( src  ), dst  );\
        DST##_q4_SET( pattitude_quaternion_q4_GET( src  ), dst  );\
        DST##_rollspeed_SET( pattitude_quaternion_rollspeed_GET( src  ), dst  );\
        DST##_pitchspeed_SET( pattitude_quaternion_pitchspeed_GET( src  ), dst  );\
        DST##_yawspeed_SET( pattitude_quaternion_yawspeed_GET( src  ), dst  );\
    }

/**
*Data packet, size 64 */

typedef Pack DATA64_data64; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pdata64_DATA64;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pdata64_DATA64 *pdata64_DATA64_from(DATA64_data64 *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdata64_daTa;
//Maximum field array length constant
#define Pdata64_daTa_len  ( 64 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_typE_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_len_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vdata64_daTa * src, <DST> * dst  ){}
*/

#define pdata64_DATA64_PUSH_INTO(DST)\
    static inline void pdata64_DATA64_push_into_##DST ( pdata64_DATA64 * src, DST * dst) {\
        DST##_typE_SET( pdata64_typE_GET( src  ), dst  );\
        DST##_len_SET( pdata64_len_GET( src  ), dst  );\
        Vdata64_daTa item_daTa = pdata64_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_typE_GET( <SRC> * src ){}
static inline int8_t <SRC>_len_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vdata64_daTa * dst ){}
*/

#define pdata64_DATA64_PULL_FROM(SRC)\
    static inline void pdata64_DATA64_pull_from_##SRC ( SRC * src, pdata64_DATA64 * dst) {\
        pdata64_typE_SET( SRC##_typE_GET(src ), dst  );\
        pdata64_len_SET( SRC##_len_GET(src ), dst  );\
       Vdata64_daTa item_daTa = pdata64_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS */

typedef Pack HIL_ACTUATOR_CONTROLS_hil_actuator_controls; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor phil_actuator_controls_HIL_ACTUATOR_CONTROLS;// data navigator over pack fields data
/**
															* Wrap HIL_ACTUATOR_CONTROLS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline phil_actuator_controls_HIL_ACTUATOR_CONTROLS *HIL_ACTUATOR_CONTROLS_hil_actuator_controls_wrap(HIL_ACTUATOR_CONTROLS_hil_actuator_controls *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline HIL_ACTUATOR_CONTROLS_hil_actuator_controls *phil_actuator_controls_HIL_ACTUATOR_CONTROLS_unwrap(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vhil_actuator_controls_controls;
//Maximum field array length constant
#define Phil_actuator_controls_controls_len  ( 16 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_flags_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_controls_SET( Vhil_actuator_controls_controls * src, <DST> * dst  ){}
static inline void <DST>_mode_SET( e_MAV_MODE * src, <DST> * dst  ){}
*/

#define phil_actuator_controls_HIL_ACTUATOR_CONTROLS_PUSH_INTO(DST)\
    static inline void phil_actuator_controls_HIL_ACTUATOR_CONTROLS_push_into_##DST ( phil_actuator_controls_HIL_ACTUATOR_CONTROLS * src, DST * dst) {\
        DST##_time_usec_SET( phil_actuator_controls_time_usec_GET( src  ), dst  );\
        DST##_flags_SET( phil_actuator_controls_flags_GET( src  ), dst  );\
        Vhil_actuator_controls_controls item_controls = phil_actuator_controls_controls_GET( src  );\
       DST##_controls_SET( &item_controls, dst );\
        e_MAV_MODE  item_mode;\
        if( phil_actuator_controls_mode_GET( src, &item_mode ) ){\
            DST##_mode_SET( item_mode , dst  );\
        }\
    }

/**
*Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
*				 This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
*				 this way */

typedef Pack POSITION_TARGET_LOCAL_NED_position_target_local_ned; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pposition_target_local_ned_POSITION_TARGET_LOCAL_NED;// data navigator over pack fields data
/**
															* Wrap POSITION_TARGET_LOCAL_NED in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *POSITION_TARGET_LOCAL_NED_position_target_local_ned_wrap(POSITION_TARGET_LOCAL_NED_position_target_local_ned *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline POSITION_TARGET_LOCAL_NED_position_target_local_ned *pposition_target_local_ned_POSITION_TARGET_LOCAL_NED_unwrap(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_type_mask_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afx_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_afz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_rate_SET( float * src, <DST> * dst  ){}
static inline void <DST>_coordinate_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
*/

#define pposition_target_local_ned_POSITION_TARGET_LOCAL_NED_PUSH_INTO(DST)\
    static inline void pposition_target_local_ned_POSITION_TARGET_LOCAL_NED_push_into_##DST ( pposition_target_local_ned_POSITION_TARGET_LOCAL_NED * src, DST * dst) {\
        DST##_type_mask_SET( pposition_target_local_ned_type_mask_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( pposition_target_local_ned_time_boot_ms_GET( src  ), dst  );\
        DST##_x_SET( pposition_target_local_ned_x_GET( src  ), dst  );\
        DST##_y_SET( pposition_target_local_ned_y_GET( src  ), dst  );\
        DST##_z_SET( pposition_target_local_ned_z_GET( src  ), dst  );\
        DST##_vx_SET( pposition_target_local_ned_vx_GET( src  ), dst  );\
        DST##_vy_SET( pposition_target_local_ned_vy_GET( src  ), dst  );\
        DST##_vz_SET( pposition_target_local_ned_vz_GET( src  ), dst  );\
        DST##_afx_SET( pposition_target_local_ned_afx_GET( src  ), dst  );\
        DST##_afy_SET( pposition_target_local_ned_afy_GET( src  ), dst  );\
        DST##_afz_SET( pposition_target_local_ned_afz_GET( src  ), dst  );\
        DST##_yaw_SET( pposition_target_local_ned_yaw_GET( src  ), dst  );\
        DST##_yaw_rate_SET( pposition_target_local_ned_yaw_rate_GET( src  ), dst  );\
        e_MAV_FRAME  item_coordinate_frame;\
        if( pposition_target_local_ned_coordinate_frame_GET( src, &item_coordinate_frame ) ){\
            DST##_coordinate_frame_SET( item_coordinate_frame , dst  );\
        }\
    }

/**
*3 axis gimbal mesuraments */

typedef Pack GIMBAL_REPORT_gimbal_report; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pgimbal_report_GIMBAL_REPORT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pgimbal_report_GIMBAL_REPORT *pgimbal_report_GIMBAL_REPORT_from(GIMBAL_REPORT_gimbal_report *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_delta_time_SET( float * src, <DST> * dst  ){}
static inline void <DST>_delta_angle_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_delta_angle_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_delta_angle_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_delta_velocity_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_delta_velocity_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_delta_velocity_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_joint_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_joint_el_SET( float * src, <DST> * dst  ){}
static inline void <DST>_joint_az_SET( float * src, <DST> * dst  ){}
*/

#define pgimbal_report_GIMBAL_REPORT_PUSH_INTO(DST)\
    static inline void pgimbal_report_GIMBAL_REPORT_push_into_##DST ( pgimbal_report_GIMBAL_REPORT * src, DST * dst) {\
        DST##_target_system_SET( pgimbal_report_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pgimbal_report_target_component_GET( src  ), dst  );\
        DST##_delta_time_SET( pgimbal_report_delta_time_GET( src  ), dst  );\
        DST##_delta_angle_x_SET( pgimbal_report_delta_angle_x_GET( src  ), dst  );\
        DST##_delta_angle_y_SET( pgimbal_report_delta_angle_y_GET( src  ), dst  );\
        DST##_delta_angle_z_SET( pgimbal_report_delta_angle_z_GET( src  ), dst  );\
        DST##_delta_velocity_x_SET( pgimbal_report_delta_velocity_x_GET( src  ), dst  );\
        DST##_delta_velocity_y_SET( pgimbal_report_delta_velocity_y_GET( src  ), dst  );\
        DST##_delta_velocity_z_SET( pgimbal_report_delta_velocity_z_GET( src  ), dst  );\
        DST##_joint_roll_SET( pgimbal_report_joint_roll_GET( src  ), dst  );\
        DST##_joint_el_SET( pgimbal_report_joint_el_GET( src  ), dst  );\
        DST##_joint_az_SET( pgimbal_report_joint_az_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline float <SRC>_delta_time_GET( <SRC> * src ){}
static inline float <SRC>_delta_angle_x_GET( <SRC> * src ){}
static inline float <SRC>_delta_angle_y_GET( <SRC> * src ){}
static inline float <SRC>_delta_angle_z_GET( <SRC> * src ){}
static inline float <SRC>_delta_velocity_x_GET( <SRC> * src ){}
static inline float <SRC>_delta_velocity_y_GET( <SRC> * src ){}
static inline float <SRC>_delta_velocity_z_GET( <SRC> * src ){}
static inline float <SRC>_joint_roll_GET( <SRC> * src ){}
static inline float <SRC>_joint_el_GET( <SRC> * src ){}
static inline float <SRC>_joint_az_GET( <SRC> * src ){}
*/

#define pgimbal_report_GIMBAL_REPORT_PULL_FROM(SRC)\
    static inline void pgimbal_report_GIMBAL_REPORT_pull_from_##SRC ( SRC * src, pgimbal_report_GIMBAL_REPORT * dst) {\
        pgimbal_report_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pgimbal_report_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pgimbal_report_delta_time_SET( SRC##_delta_time_GET(src ), dst  );\
        pgimbal_report_delta_angle_x_SET( SRC##_delta_angle_x_GET(src ), dst  );\
        pgimbal_report_delta_angle_y_SET( SRC##_delta_angle_y_GET(src ), dst  );\
        pgimbal_report_delta_angle_z_SET( SRC##_delta_angle_z_GET(src ), dst  );\
        pgimbal_report_delta_velocity_x_SET( SRC##_delta_velocity_x_GET(src ), dst  );\
        pgimbal_report_delta_velocity_y_SET( SRC##_delta_velocity_y_GET(src ), dst  );\
        pgimbal_report_delta_velocity_z_SET( SRC##_delta_velocity_z_GET(src ), dst  );\
        pgimbal_report_joint_roll_SET( SRC##_joint_roll_GET(src ), dst  );\
        pgimbal_report_joint_el_SET( SRC##_joint_el_GET(src ), dst  );\
        pgimbal_report_joint_az_SET( SRC##_joint_az_GET(src ), dst  );\
    }

/**
*Write registers for a device */

typedef Pack DEVICE_OP_WRITE_device_op_write; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pdevice_op_write_DEVICE_OP_WRITE;// data navigator over pack fields data
/**
															* Wrap DEVICE_OP_WRITE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pdevice_op_write_DEVICE_OP_WRITE *DEVICE_OP_WRITE_device_op_write_wrap(DEVICE_OP_WRITE_device_op_write *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline DEVICE_OP_WRITE_device_op_write *pdevice_op_write_DEVICE_OP_WRITE_unwrap(pdevice_op_write_DEVICE_OP_WRITE *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdevice_op_write_daTa;
//Maximum field array length constant
#define Pdevice_op_write_daTa_len  ( 128 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdevice_op_write_busname;
//Maximum field array length constant
#define Pdevice_op_write_busname_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_request_id_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_bus_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_address_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_regstart_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_count_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vdevice_op_write_daTa * src, <DST> * dst  ){}
static inline void <DST>_bustype_SET( e_DEVICE_OP_BUSTYPE * src, <DST> * dst  ){}
static inline void <DST>_busname_SET( Vdevice_op_write_busname * src, <DST> * dst  ){}
*/

#define pdevice_op_write_DEVICE_OP_WRITE_PUSH_INTO(DST)\
    static inline void pdevice_op_write_DEVICE_OP_WRITE_push_into_##DST ( pdevice_op_write_DEVICE_OP_WRITE * src, DST * dst) {\
        DST##_request_id_SET( pdevice_op_write_request_id_GET( src  ), dst  );\
        DST##_target_system_SET( pdevice_op_write_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pdevice_op_write_target_component_GET( src  ), dst  );\
        DST##_bus_SET( pdevice_op_write_bus_GET( src  ), dst  );\
        DST##_address_SET( pdevice_op_write_address_GET( src  ), dst  );\
        DST##_regstart_SET( pdevice_op_write_regstart_GET( src  ), dst  );\
        DST##_count_SET( pdevice_op_write_count_GET( src  ), dst  );\
        Vdevice_op_write_daTa item_daTa = pdevice_op_write_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
        e_DEVICE_OP_BUSTYPE  item_bustype;\
        if( pdevice_op_write_bustype_GET( src, &item_bustype ) ){\
            DST##_bustype_SET( item_bustype , dst  );\
        }\
        Vdevice_op_write_busname  item_busname;\
        if( pdevice_op_write_busname_GET( src, &item_busname ) ){\
            DST##_busname_SET( &item_busname, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_request_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_bus_GET( <SRC> * src ){}
static inline int8_t <SRC>_address_GET( <SRC> * src ){}
static inline int8_t <SRC>_regstart_GET( <SRC> * src ){}
static inline int8_t <SRC>_count_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vdevice_op_write_daTa * dst ){}
static inline bool  <SRC>_bustype_item_exist( <SRC> * src ){}
static inline e_DEVICE_OP_BUSTYPE <SRC>_bustype_GET( <SRC> * src ){}
static inline size_t  <SRC>_busname_item_exist( <SRC> * src  ){}
static inline void <SRC>_busname_GET( <SRC> * src, Vdevice_op_write_busname * dst ){}
*/

#define pdevice_op_write_DEVICE_OP_WRITE_PULL_FROM(SRC)\
    static inline void pdevice_op_write_DEVICE_OP_WRITE_pull_from_##SRC ( SRC * src, pdevice_op_write_DEVICE_OP_WRITE * dst) {\
        pdevice_op_write_request_id_SET( SRC##_request_id_GET(src ), dst  );\
        pdevice_op_write_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pdevice_op_write_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pdevice_op_write_bus_SET( SRC##_bus_GET(src ), dst  );\
        pdevice_op_write_address_SET( SRC##_address_GET(src ), dst  );\
        pdevice_op_write_regstart_SET( SRC##_regstart_GET(src ), dst  );\
        pdevice_op_write_count_SET( SRC##_count_GET(src ), dst  );\
       Vdevice_op_write_daTa item_daTa = pdevice_op_write_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
        if( SRC##_bustype_item_exist(src ) )\
        pdevice_op_write_bustype_SET( pdevice_op_write_DEVICE_OP_WRITE_bustype_GET( src ),  dst  );\
        const size_t len_busname = SRC##_busname_item_exist(src );\
        if( len_busname ){\
            Vdevice_op_write_busname    item_busname = pdevice_op_write_busname_SET( NULL, len_busname, dst  );\
            pdevice_op_write_DEVICE_OP_WRITE_busname_GET(src, &item_busname );\
        }\
    }


typedef Pack DISTANCE_SENSOR_distance_sensor; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pdistance_sensor_DISTANCE_SENSOR;// data navigator over pack fields data
/**
															* Wrap DISTANCE_SENSOR in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pdistance_sensor_DISTANCE_SENSOR *DISTANCE_SENSOR_distance_sensor_wrap(DISTANCE_SENSOR_distance_sensor *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline DISTANCE_SENSOR_distance_sensor *pdistance_sensor_DISTANCE_SENSOR_unwrap(pdistance_sensor_DISTANCE_SENSOR *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_min_distance_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_max_distance_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_current_distance_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_covariance_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_typE_SET( e_MAV_DISTANCE_SENSOR * src, <DST> * dst  ){}
static inline void <DST>_orientation_SET( e_MAV_SENSOR_ORIENTATION * src, <DST> * dst  ){}
*/

#define pdistance_sensor_DISTANCE_SENSOR_PUSH_INTO(DST)\
    static inline void pdistance_sensor_DISTANCE_SENSOR_push_into_##DST ( pdistance_sensor_DISTANCE_SENSOR * src, DST * dst) {\
        DST##_min_distance_SET( pdistance_sensor_min_distance_GET( src  ), dst  );\
        DST##_max_distance_SET( pdistance_sensor_max_distance_GET( src  ), dst  );\
        DST##_current_distance_SET( pdistance_sensor_current_distance_GET( src  ), dst  );\
        DST##_time_boot_ms_SET( pdistance_sensor_time_boot_ms_GET( src  ), dst  );\
        DST##_id_SET( pdistance_sensor_id_GET( src  ), dst  );\
        DST##_covariance_SET( pdistance_sensor_covariance_GET( src  ), dst  );\
        e_MAV_DISTANCE_SENSOR  item_typE;\
        if( pdistance_sensor_typE_GET( src, &item_typE ) ){\
            DST##_typE_SET( item_typE , dst  );\
        }\
        e_MAV_SENSOR_ORIENTATION  item_orientation;\
        if( pdistance_sensor_orientation_GET( src, &item_orientation ) ){\
            DST##_orientation_SET( item_orientation , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_min_distance_GET( <SRC> * src ){}
static inline int16_t <SRC>_max_distance_GET( <SRC> * src ){}
static inline int16_t <SRC>_current_distance_GET( <SRC> * src ){}
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int8_t <SRC>_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_covariance_GET( <SRC> * src ){}
static inline bool  <SRC>_typE_item_exist( <SRC> * src ){}
static inline e_MAV_DISTANCE_SENSOR <SRC>_typE_GET( <SRC> * src ){}
static inline bool  <SRC>_orientation_item_exist( <SRC> * src ){}
static inline e_MAV_SENSOR_ORIENTATION <SRC>_orientation_GET( <SRC> * src ){}
*/

#define pdistance_sensor_DISTANCE_SENSOR_PULL_FROM(SRC)\
    static inline void pdistance_sensor_DISTANCE_SENSOR_pull_from_##SRC ( SRC * src, pdistance_sensor_DISTANCE_SENSOR * dst) {\
        pdistance_sensor_min_distance_SET( SRC##_min_distance_GET(src ), dst  );\
        pdistance_sensor_max_distance_SET( SRC##_max_distance_GET(src ), dst  );\
        pdistance_sensor_current_distance_SET( SRC##_current_distance_GET(src ), dst  );\
        pdistance_sensor_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pdistance_sensor_id_SET( SRC##_id_GET(src ), dst  );\
        pdistance_sensor_covariance_SET( SRC##_covariance_GET(src ), dst  );\
        if( SRC##_typE_item_exist(src ) )\
        pdistance_sensor_typE_SET( pdistance_sensor_DISTANCE_SENSOR_typE_GET( src ),  dst  );\
        if( SRC##_orientation_item_exist(src ) )\
        pdistance_sensor_orientation_SET( pdistance_sensor_DISTANCE_SENSOR_orientation_GET( src ),  dst  );\
    }

/**
*Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor) */

typedef Pack HIL_OPTICAL_FLOW_hil_optical_flow; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t phil_optical_flow_HIL_OPTICAL_FLOW;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline phil_optical_flow_HIL_OPTICAL_FLOW *phil_optical_flow_HIL_OPTICAL_FLOW_from(HIL_OPTICAL_FLOW_hil_optical_flow *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_integration_time_us_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_delta_distance_us_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_sensor_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_integrated_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_integrated_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_integrated_xgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_integrated_ygyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_integrated_zgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_quality_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_distance_SET( float * src, <DST> * dst  ){}
*/

#define phil_optical_flow_HIL_OPTICAL_FLOW_PUSH_INTO(DST)\
    static inline void phil_optical_flow_HIL_OPTICAL_FLOW_push_into_##DST ( phil_optical_flow_HIL_OPTICAL_FLOW * src, DST * dst) {\
        DST##_integration_time_us_SET( phil_optical_flow_integration_time_us_GET( src  ), dst  );\
        DST##_time_delta_distance_us_SET( phil_optical_flow_time_delta_distance_us_GET( src  ), dst  );\
        DST##_time_usec_SET( phil_optical_flow_time_usec_GET( src  ), dst  );\
        DST##_sensor_id_SET( phil_optical_flow_sensor_id_GET( src  ), dst  );\
        DST##_integrated_x_SET( phil_optical_flow_integrated_x_GET( src  ), dst  );\
        DST##_integrated_y_SET( phil_optical_flow_integrated_y_GET( src  ), dst  );\
        DST##_integrated_xgyro_SET( phil_optical_flow_integrated_xgyro_GET( src  ), dst  );\
        DST##_integrated_ygyro_SET( phil_optical_flow_integrated_ygyro_GET( src  ), dst  );\
        DST##_integrated_zgyro_SET( phil_optical_flow_integrated_zgyro_GET( src  ), dst  );\
        DST##_temperature_SET( phil_optical_flow_temperature_GET( src  ), dst  );\
        DST##_quality_SET( phil_optical_flow_quality_GET( src  ), dst  );\
        DST##_distance_SET( phil_optical_flow_distance_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_integration_time_us_GET( <SRC> * src ){}
static inline int32_t <SRC>_time_delta_distance_us_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int8_t <SRC>_sensor_id_GET( <SRC> * src ){}
static inline float <SRC>_integrated_x_GET( <SRC> * src ){}
static inline float <SRC>_integrated_y_GET( <SRC> * src ){}
static inline float <SRC>_integrated_xgyro_GET( <SRC> * src ){}
static inline float <SRC>_integrated_ygyro_GET( <SRC> * src ){}
static inline float <SRC>_integrated_zgyro_GET( <SRC> * src ){}
static inline int16_t <SRC>_temperature_GET( <SRC> * src ){}
static inline int8_t <SRC>_quality_GET( <SRC> * src ){}
static inline float <SRC>_distance_GET( <SRC> * src ){}
*/

#define phil_optical_flow_HIL_OPTICAL_FLOW_PULL_FROM(SRC)\
    static inline void phil_optical_flow_HIL_OPTICAL_FLOW_pull_from_##SRC ( SRC * src, phil_optical_flow_HIL_OPTICAL_FLOW * dst) {\
        phil_optical_flow_integration_time_us_SET( SRC##_integration_time_us_GET(src ), dst  );\
        phil_optical_flow_time_delta_distance_us_SET( SRC##_time_delta_distance_us_GET(src ), dst  );\
        phil_optical_flow_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        phil_optical_flow_sensor_id_SET( SRC##_sensor_id_GET(src ), dst  );\
        phil_optical_flow_integrated_x_SET( SRC##_integrated_x_GET(src ), dst  );\
        phil_optical_flow_integrated_y_SET( SRC##_integrated_y_GET(src ), dst  );\
        phil_optical_flow_integrated_xgyro_SET( SRC##_integrated_xgyro_GET(src ), dst  );\
        phil_optical_flow_integrated_ygyro_SET( SRC##_integrated_ygyro_GET(src ), dst  );\
        phil_optical_flow_integrated_zgyro_SET( SRC##_integrated_zgyro_GET(src ), dst  );\
        phil_optical_flow_temperature_SET( SRC##_temperature_GET(src ), dst  );\
        phil_optical_flow_quality_SET( SRC##_quality_GET(src ), dst  );\
        phil_optical_flow_distance_SET( SRC##_distance_GET(src ), dst  );\
    }

/**
*Barometer readings for 2nd barometer */

typedef Pack SCALED_PRESSURE2_scaled_pressure2; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pscaled_pressure2_SCALED_PRESSURE2;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pscaled_pressure2_SCALED_PRESSURE2 *pscaled_pressure2_SCALED_PRESSURE2_from(SCALED_PRESSURE2_scaled_pressure2 *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_press_abs_SET( float * src, <DST> * dst  ){}
static inline void <DST>_press_diff_SET( float * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( int16_t * src, <DST> * dst  ){}
*/

#define pscaled_pressure2_SCALED_PRESSURE2_PUSH_INTO(DST)\
    static inline void pscaled_pressure2_SCALED_PRESSURE2_push_into_##DST ( pscaled_pressure2_SCALED_PRESSURE2 * src, DST * dst) {\
        DST##_time_boot_ms_SET( pscaled_pressure2_time_boot_ms_GET( src  ), dst  );\
        DST##_press_abs_SET( pscaled_pressure2_press_abs_GET( src  ), dst  );\
        DST##_press_diff_SET( pscaled_pressure2_press_diff_GET( src  ), dst  );\
        DST##_temperature_SET( pscaled_pressure2_temperature_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline float <SRC>_press_abs_GET( <SRC> * src ){}
static inline float <SRC>_press_diff_GET( <SRC> * src ){}
static inline int16_t <SRC>_temperature_GET( <SRC> * src ){}
*/

#define pscaled_pressure2_SCALED_PRESSURE2_PULL_FROM(SRC)\
    static inline void pscaled_pressure2_SCALED_PRESSURE2_pull_from_##SRC ( SRC * src, pscaled_pressure2_SCALED_PRESSURE2 * dst) {\
        pscaled_pressure2_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pscaled_pressure2_press_abs_SET( SRC##_press_abs_GET(src ), dst  );\
        pscaled_pressure2_press_diff_SET( SRC##_press_diff_GET(src ), dst  );\
        pscaled_pressure2_temperature_SET( SRC##_temperature_GET(src ), dst  );\
    }


typedef Pack WIND_COV_wind_cov; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pwind_cov_WIND_COV;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pwind_cov_WIND_COV *pwind_cov_WIND_COV_from(WIND_COV_wind_cov *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_wind_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_wind_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_wind_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_var_horiz_SET( float * src, <DST> * dst  ){}
static inline void <DST>_var_vert_SET( float * src, <DST> * dst  ){}
static inline void <DST>_wind_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_horiz_accuracy_SET( float * src, <DST> * dst  ){}
static inline void <DST>_vert_accuracy_SET( float * src, <DST> * dst  ){}
*/

#define pwind_cov_WIND_COV_PUSH_INTO(DST)\
    static inline void pwind_cov_WIND_COV_push_into_##DST ( pwind_cov_WIND_COV * src, DST * dst) {\
        DST##_time_usec_SET( pwind_cov_time_usec_GET( src  ), dst  );\
        DST##_wind_x_SET( pwind_cov_wind_x_GET( src  ), dst  );\
        DST##_wind_y_SET( pwind_cov_wind_y_GET( src  ), dst  );\
        DST##_wind_z_SET( pwind_cov_wind_z_GET( src  ), dst  );\
        DST##_var_horiz_SET( pwind_cov_var_horiz_GET( src  ), dst  );\
        DST##_var_vert_SET( pwind_cov_var_vert_GET( src  ), dst  );\
        DST##_wind_alt_SET( pwind_cov_wind_alt_GET( src  ), dst  );\
        DST##_horiz_accuracy_SET( pwind_cov_horiz_accuracy_GET( src  ), dst  );\
        DST##_vert_accuracy_SET( pwind_cov_vert_accuracy_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_wind_x_GET( <SRC> * src ){}
static inline float <SRC>_wind_y_GET( <SRC> * src ){}
static inline float <SRC>_wind_z_GET( <SRC> * src ){}
static inline float <SRC>_var_horiz_GET( <SRC> * src ){}
static inline float <SRC>_var_vert_GET( <SRC> * src ){}
static inline float <SRC>_wind_alt_GET( <SRC> * src ){}
static inline float <SRC>_horiz_accuracy_GET( <SRC> * src ){}
static inline float <SRC>_vert_accuracy_GET( <SRC> * src ){}
*/

#define pwind_cov_WIND_COV_PULL_FROM(SRC)\
    static inline void pwind_cov_WIND_COV_pull_from_##SRC ( SRC * src, pwind_cov_WIND_COV * dst) {\
        pwind_cov_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pwind_cov_wind_x_SET( SRC##_wind_x_GET(src ), dst  );\
        pwind_cov_wind_y_SET( SRC##_wind_y_GET(src ), dst  );\
        pwind_cov_wind_z_SET( SRC##_wind_z_GET(src ), dst  );\
        pwind_cov_var_horiz_SET( SRC##_var_horiz_GET(src ), dst  );\
        pwind_cov_var_vert_SET( SRC##_var_vert_GET(src ), dst  );\
        pwind_cov_wind_alt_SET( SRC##_wind_alt_GET(src ), dst  );\
        pwind_cov_horiz_accuracy_SET( SRC##_horiz_accuracy_GET(src ), dst  );\
        pwind_cov_vert_accuracy_SET( SRC##_vert_accuracy_GET(src ), dst  );\
    }

/**
*Request to control this MAV */

typedef Pack CHANGE_OPERATOR_CONTROL_change_operator_control; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pchange_operator_control_CHANGE_OPERATOR_CONTROL;// data navigator over pack fields data
/**
															* Wrap CHANGE_OPERATOR_CONTROL in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pchange_operator_control_CHANGE_OPERATOR_CONTROL *CHANGE_OPERATOR_CONTROL_change_operator_control_wrap(CHANGE_OPERATOR_CONTROL_change_operator_control *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline CHANGE_OPERATOR_CONTROL_change_operator_control *pchange_operator_control_CHANGE_OPERATOR_CONTROL_unwrap(pchange_operator_control_CHANGE_OPERATOR_CONTROL *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vchange_operator_control_passkey;
//Maximum field array length constant
#define Pchange_operator_control_passkey_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_control_request_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_version_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_passkey_SET( Vchange_operator_control_passkey * src, <DST> * dst  ){}
*/

#define pchange_operator_control_CHANGE_OPERATOR_CONTROL_PUSH_INTO(DST)\
    static inline void pchange_operator_control_CHANGE_OPERATOR_CONTROL_push_into_##DST ( pchange_operator_control_CHANGE_OPERATOR_CONTROL * src, DST * dst) {\
        DST##_target_system_SET( pchange_operator_control_target_system_GET( src  ), dst  );\
        DST##_control_request_SET( pchange_operator_control_control_request_GET( src  ), dst  );\
        DST##_version_SET( pchange_operator_control_version_GET( src  ), dst  );\
        Vchange_operator_control_passkey  item_passkey;\
        if( pchange_operator_control_passkey_GET( src, &item_passkey ) ){\
            DST##_passkey_SET( &item_passkey, dst );\
        }\
    }

/**
*Request to set a GOPRO_COMMAND with a desired */

typedef Pack GOPRO_SET_REQUEST_gopro_set_request; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgopro_set_request_GOPRO_SET_REQUEST;// data navigator over pack fields data
/**
															* Wrap GOPRO_SET_REQUEST in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgopro_set_request_GOPRO_SET_REQUEST *GOPRO_SET_REQUEST_gopro_set_request_wrap(GOPRO_SET_REQUEST_gopro_set_request *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GOPRO_SET_REQUEST_gopro_set_request *pgopro_set_request_GOPRO_SET_REQUEST_unwrap(pgopro_set_request_GOPRO_SET_REQUEST *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgopro_set_request_value;
//Maximum field array length constant
#define Pgopro_set_request_value_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_value_SET( Vgopro_set_request_value * src, <DST> * dst  ){}
static inline void <DST>_cmd_id_SET( e_GOPRO_COMMAND * src, <DST> * dst  ){}
*/

#define pgopro_set_request_GOPRO_SET_REQUEST_PUSH_INTO(DST)\
    static inline void pgopro_set_request_GOPRO_SET_REQUEST_push_into_##DST ( pgopro_set_request_GOPRO_SET_REQUEST * src, DST * dst) {\
        DST##_target_system_SET( pgopro_set_request_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pgopro_set_request_target_component_GET( src  ), dst  );\
        Vgopro_set_request_value item_value = pgopro_set_request_value_GET( src  );\
       DST##_value_SET( &item_value, dst );\
        e_GOPRO_COMMAND  item_cmd_id;\
        if( pgopro_set_request_cmd_id_GET( src, &item_cmd_id ) ){\
            DST##_cmd_id_SET( item_cmd_id , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_value_GET( <SRC> * src, Vgopro_set_request_value * dst ){}
static inline bool  <SRC>_cmd_id_item_exist( <SRC> * src ){}
static inline e_GOPRO_COMMAND <SRC>_cmd_id_GET( <SRC> * src ){}
*/

#define pgopro_set_request_GOPRO_SET_REQUEST_PULL_FROM(SRC)\
    static inline void pgopro_set_request_GOPRO_SET_REQUEST_pull_from_##SRC ( SRC * src, pgopro_set_request_GOPRO_SET_REQUEST * dst) {\
        pgopro_set_request_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pgopro_set_request_target_component_SET( SRC##_target_component_GET(src ), dst  );\
       Vgopro_set_request_value item_value = pgopro_set_request_value_SET( NULL, dst  );\
       SRC##_value_GET( src, &item_value );\
        if( SRC##_cmd_id_item_exist(src ) )\
        pgopro_set_request_cmd_id_SET( pgopro_set_request_GOPRO_SET_REQUEST_cmd_id_GET( src ),  dst  );\
    }

/**
*The general system state. If the system is following the MAVLink standard, the system state is mainly
*				 defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and
*				 locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position
*				 setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined
*				 the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents
*				 the internal navigation state machine. The system status shows whether the system is currently active
*				 or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered
*				 to be active, but should start emergency procedures autonomously. After a failure occured it should first
*				 move from active to critical to allow manual intervention and then move to emergency after a certain
*				 timeout */

typedef Pack SYS_STATUS_sys_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor psys_status_SYS_STATUS;// data navigator over pack fields data
/**
															* Wrap SYS_STATUS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline psys_status_SYS_STATUS *SYS_STATUS_sys_status_wrap(SYS_STATUS_sys_status *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline SYS_STATUS_sys_status *psys_status_SYS_STATUS_unwrap(psys_status_SYS_STATUS *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_load_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_voltage_battery_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_drop_rate_comm_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_errors_comm_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_errors_count1_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_errors_count2_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_errors_count3_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_errors_count4_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_current_battery_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_battery_remaining_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_onboard_control_sensors_present_SET( e_MAV_SYS_STATUS_SENSOR * src, <DST> * dst  ){}
static inline void <DST>_onboard_control_sensors_enabled_SET( e_MAV_SYS_STATUS_SENSOR * src, <DST> * dst  ){}
static inline void <DST>_onboard_control_sensors_health_SET( e_MAV_SYS_STATUS_SENSOR * src, <DST> * dst  ){}
*/

#define psys_status_SYS_STATUS_PUSH_INTO(DST)\
    static inline void psys_status_SYS_STATUS_push_into_##DST ( psys_status_SYS_STATUS * src, DST * dst) {\
        DST##_load_SET( psys_status_load_GET( src  ), dst  );\
        DST##_voltage_battery_SET( psys_status_voltage_battery_GET( src  ), dst  );\
        DST##_drop_rate_comm_SET( psys_status_drop_rate_comm_GET( src  ), dst  );\
        DST##_errors_comm_SET( psys_status_errors_comm_GET( src  ), dst  );\
        DST##_errors_count1_SET( psys_status_errors_count1_GET( src  ), dst  );\
        DST##_errors_count2_SET( psys_status_errors_count2_GET( src  ), dst  );\
        DST##_errors_count3_SET( psys_status_errors_count3_GET( src  ), dst  );\
        DST##_errors_count4_SET( psys_status_errors_count4_GET( src  ), dst  );\
        DST##_current_battery_SET( psys_status_current_battery_GET( src  ), dst  );\
        DST##_battery_remaining_SET( psys_status_battery_remaining_GET( src  ), dst  );\
        e_MAV_SYS_STATUS_SENSOR  item_onboard_control_sensors_present;\
        if( psys_status_onboard_control_sensors_present_GET( src, &item_onboard_control_sensors_present ) ){\
            DST##_onboard_control_sensors_present_SET( item_onboard_control_sensors_present , dst  );\
        }\
        e_MAV_SYS_STATUS_SENSOR  item_onboard_control_sensors_enabled;\
        if( psys_status_onboard_control_sensors_enabled_GET( src, &item_onboard_control_sensors_enabled ) ){\
            DST##_onboard_control_sensors_enabled_SET( item_onboard_control_sensors_enabled , dst  );\
        }\
        e_MAV_SYS_STATUS_SENSOR  item_onboard_control_sensors_health;\
        if( psys_status_onboard_control_sensors_health_GET( src, &item_onboard_control_sensors_health ) ){\
            DST##_onboard_control_sensors_health_SET( item_onboard_control_sensors_health , dst  );\
        }\
    }

/**
*Message encoding a mission item. This message is emitted to announce
*				 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http:qgroundcontrol.org/mavlink/waypoint_protocol. */

typedef Pack MISSION_ITEM_mission_item; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_item_MISSION_ITEM;// data navigator over pack fields data
/**
															* Wrap MISSION_ITEM in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_item_MISSION_ITEM *MISSION_ITEM_mission_item_wrap(MISSION_ITEM_mission_item *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_ITEM_mission_item *pmission_item_MISSION_ITEM_unwrap(pmission_item_MISSION_ITEM *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_current_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_autocontinue_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param4_SET( float * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
static inline void <DST>_command_SET( e_MAV_CMD * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_item_MISSION_ITEM_PUSH_INTO(DST)\
    static inline void pmission_item_MISSION_ITEM_push_into_##DST ( pmission_item_MISSION_ITEM * src, DST * dst) {\
        DST##_seq_SET( pmission_item_seq_GET( src  ), dst  );\
        DST##_target_system_SET( pmission_item_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_item_target_component_GET( src  ), dst  );\
        DST##_current_SET( pmission_item_current_GET( src  ), dst  );\
        DST##_autocontinue_SET( pmission_item_autocontinue_GET( src  ), dst  );\
        DST##_param1_SET( pmission_item_param1_GET( src  ), dst  );\
        DST##_param2_SET( pmission_item_param2_GET( src  ), dst  );\
        DST##_param3_SET( pmission_item_param3_GET( src  ), dst  );\
        DST##_param4_SET( pmission_item_param4_GET( src  ), dst  );\
        DST##_x_SET( pmission_item_x_GET( src  ), dst  );\
        DST##_y_SET( pmission_item_y_GET( src  ), dst  );\
        DST##_z_SET( pmission_item_z_GET( src  ), dst  );\
        e_MAV_FRAME  item_frame;\
        if( pmission_item_frame_GET( src, &item_frame ) ){\
            DST##_frame_SET( item_frame , dst  );\
        }\
        e_MAV_CMD  item_command;\
        if( pmission_item_command_GET( src, &item_command ) ){\
            DST##_command_SET( item_command , dst  );\
        }\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_item_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw
*				 values without any scaling to allow data capture and system debugging */

typedef Pack RAW_IMU_raw_imu; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t praw_imu_RAW_IMU;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline praw_imu_RAW_IMU *praw_imu_RAW_IMU_from(RAW_IMU_raw_imu *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xgyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ygyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zgyro_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_xmag_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_ymag_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_zmag_SET( int16_t * src, <DST> * dst  ){}
*/

#define praw_imu_RAW_IMU_PUSH_INTO(DST)\
    static inline void praw_imu_RAW_IMU_push_into_##DST ( praw_imu_RAW_IMU * src, DST * dst) {\
        DST##_time_usec_SET( praw_imu_time_usec_GET( src  ), dst  );\
        DST##_xacc_SET( praw_imu_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( praw_imu_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( praw_imu_zacc_GET( src  ), dst  );\
        DST##_xgyro_SET( praw_imu_xgyro_GET( src  ), dst  );\
        DST##_ygyro_SET( praw_imu_ygyro_GET( src  ), dst  );\
        DST##_zgyro_SET( praw_imu_zgyro_GET( src  ), dst  );\
        DST##_xmag_SET( praw_imu_xmag_GET( src  ), dst  );\
        DST##_ymag_SET( praw_imu_ymag_GET( src  ), dst  );\
        DST##_zmag_SET( praw_imu_zmag_GET( src  ), dst  );\
    }

/**
*Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value */

typedef Pack COMMAND_INT_command_int; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pcommand_int_COMMAND_INT;// data navigator over pack fields data
/**
															* Wrap COMMAND_INT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pcommand_int_COMMAND_INT *COMMAND_INT_command_int_wrap(COMMAND_INT_command_int *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline COMMAND_INT_command_int *pcommand_int_COMMAND_INT_unwrap(pcommand_int_COMMAND_INT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_current_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_autocontinue_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param4_SET( float * src, <DST> * dst  ){}
static inline void <DST>_x_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_y_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
static inline void <DST>_command_SET( e_MAV_CMD * src, <DST> * dst  ){}
*/

#define pcommand_int_COMMAND_INT_PUSH_INTO(DST)\
    static inline void pcommand_int_COMMAND_INT_push_into_##DST ( pcommand_int_COMMAND_INT * src, DST * dst) {\
        DST##_target_system_SET( pcommand_int_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pcommand_int_target_component_GET( src  ), dst  );\
        DST##_current_SET( pcommand_int_current_GET( src  ), dst  );\
        DST##_autocontinue_SET( pcommand_int_autocontinue_GET( src  ), dst  );\
        DST##_param1_SET( pcommand_int_param1_GET( src  ), dst  );\
        DST##_param2_SET( pcommand_int_param2_GET( src  ), dst  );\
        DST##_param3_SET( pcommand_int_param3_GET( src  ), dst  );\
        DST##_param4_SET( pcommand_int_param4_GET( src  ), dst  );\
        DST##_x_SET( pcommand_int_x_GET( src  ), dst  );\
        DST##_y_SET( pcommand_int_y_GET( src  ), dst  );\
        DST##_z_SET( pcommand_int_z_GET( src  ), dst  );\
        e_MAV_FRAME  item_frame;\
        if( pcommand_int_frame_GET( src, &item_frame ) ){\
            DST##_frame_SET( item_frame , dst  );\
        }\
        e_MAV_CMD  item_command;\
        if( pcommand_int_command_GET( src, &item_command ) ){\
            DST##_command_SET( item_command , dst  );\
        }\
    }

/**
*Optical flow from a flow sensor (e.g. optical mouse sensor) */

typedef Pack OPTICAL_FLOW_optical_flow; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor poptical_flow_OPTICAL_FLOW;// data navigator over pack fields data
/**
															* Wrap OPTICAL_FLOW in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline poptical_flow_OPTICAL_FLOW *OPTICAL_FLOW_optical_flow_wrap(OPTICAL_FLOW_optical_flow *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline OPTICAL_FLOW_optical_flow *poptical_flow_OPTICAL_FLOW_unwrap(poptical_flow_OPTICAL_FLOW *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_sensor_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_flow_x_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_flow_y_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_flow_comp_m_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_flow_comp_m_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_quality_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_ground_distance_SET( float * src, <DST> * dst  ){}
static inline void <DST>_flow_rate_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_flow_rate_y_SET( float * src, <DST> * dst  ){}
*/

#define poptical_flow_OPTICAL_FLOW_PUSH_INTO(DST)\
    static inline void poptical_flow_OPTICAL_FLOW_push_into_##DST ( poptical_flow_OPTICAL_FLOW * src, DST * dst) {\
        DST##_time_usec_SET( poptical_flow_time_usec_GET( src  ), dst  );\
        DST##_sensor_id_SET( poptical_flow_sensor_id_GET( src  ), dst  );\
        DST##_flow_x_SET( poptical_flow_flow_x_GET( src  ), dst  );\
        DST##_flow_y_SET( poptical_flow_flow_y_GET( src  ), dst  );\
        DST##_flow_comp_m_x_SET( poptical_flow_flow_comp_m_x_GET( src  ), dst  );\
        DST##_flow_comp_m_y_SET( poptical_flow_flow_comp_m_y_GET( src  ), dst  );\
        DST##_quality_SET( poptical_flow_quality_GET( src  ), dst  );\
        DST##_ground_distance_SET( poptical_flow_ground_distance_GET( src  ), dst  );\
        float  item_flow_rate_x;\
        if( poptical_flow_flow_rate_x_GET( src, &item_flow_rate_x ) ){\
            DST##_flow_rate_x_SET( item_flow_rate_x , dst  );\
        }\
        float  item_flow_rate_y;\
        if( poptical_flow_flow_rate_y_GET( src, &item_flow_rate_y ) ){\
            DST##_flow_rate_y_SET( item_flow_rate_y , dst  );\
        }\
    }

/**
*Message encoding a mission item. This message is emitted to announce
*				 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp:qgroundcontrol.org/mavlink/waypoint_protocol. */

typedef Pack MISSION_ITEM_INT_mission_item_int; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmission_item_int_MISSION_ITEM_INT;// data navigator over pack fields data
/**
															* Wrap MISSION_ITEM_INT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmission_item_int_MISSION_ITEM_INT *MISSION_ITEM_INT_mission_item_int_wrap(MISSION_ITEM_INT_mission_item_int *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MISSION_ITEM_INT_mission_item_int *pmission_item_int_MISSION_ITEM_INT_unwrap(pmission_item_int_MISSION_ITEM_INT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_seq_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_current_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_autocontinue_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_param1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param2_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param3_SET( float * src, <DST> * dst  ){}
static inline void <DST>_param4_SET( float * src, <DST> * dst  ){}
static inline void <DST>_x_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_y_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_frame_SET( e_MAV_FRAME * src, <DST> * dst  ){}
static inline void <DST>_command_SET( e_MAV_CMD * src, <DST> * dst  ){}
static inline void <DST>_mission_type_SET( e_MAV_MISSION_TYPE * src, <DST> * dst  ){}
*/

#define pmission_item_int_MISSION_ITEM_INT_PUSH_INTO(DST)\
    static inline void pmission_item_int_MISSION_ITEM_INT_push_into_##DST ( pmission_item_int_MISSION_ITEM_INT * src, DST * dst) {\
        DST##_seq_SET( pmission_item_int_seq_GET( src  ), dst  );\
        DST##_target_system_SET( pmission_item_int_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pmission_item_int_target_component_GET( src  ), dst  );\
        DST##_current_SET( pmission_item_int_current_GET( src  ), dst  );\
        DST##_autocontinue_SET( pmission_item_int_autocontinue_GET( src  ), dst  );\
        DST##_param1_SET( pmission_item_int_param1_GET( src  ), dst  );\
        DST##_param2_SET( pmission_item_int_param2_GET( src  ), dst  );\
        DST##_param3_SET( pmission_item_int_param3_GET( src  ), dst  );\
        DST##_param4_SET( pmission_item_int_param4_GET( src  ), dst  );\
        DST##_x_SET( pmission_item_int_x_GET( src  ), dst  );\
        DST##_y_SET( pmission_item_int_y_GET( src  ), dst  );\
        DST##_z_SET( pmission_item_int_z_GET( src  ), dst  );\
        e_MAV_FRAME  item_frame;\
        if( pmission_item_int_frame_GET( src, &item_frame ) ){\
            DST##_frame_SET( item_frame , dst  );\
        }\
        e_MAV_CMD  item_command;\
        if( pmission_item_int_command_GET( src, &item_command ) ){\
            DST##_command_SET( item_command , dst  );\
        }\
        e_MAV_MISSION_TYPE  item_mission_type;\
        if( pmission_item_int_mission_type_GET( src, &item_mission_type ) ){\
            DST##_mission_type_SET( item_mission_type , dst  );\
        }\
    }

/**
*camera vision based attitude and position deltas */

typedef Pack VISION_POSITION_DELTA_vision_position_delta; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pvision_position_delta_VISION_POSITION_DELTA;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pvision_position_delta_VISION_POSITION_DELTA *pvision_position_delta_VISION_POSITION_DELTA_from(VISION_POSITION_DELTA_vision_position_delta *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vvision_position_delta_angle_delta;
//Maximum field array length constant
#define Pvision_position_delta_angle_delta_len  ( 3 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vvision_position_delta_position_delta;
//Maximum field array length constant
#define Pvision_position_delta_position_delta_len  ( 3 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_time_delta_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_angle_delta_SET( Vvision_position_delta_angle_delta * src, <DST> * dst  ){}
static inline void <DST>_position_delta_SET( Vvision_position_delta_position_delta * src, <DST> * dst  ){}
static inline void <DST>_confidence_SET( float * src, <DST> * dst  ){}
*/

#define pvision_position_delta_VISION_POSITION_DELTA_PUSH_INTO(DST)\
    static inline void pvision_position_delta_VISION_POSITION_DELTA_push_into_##DST ( pvision_position_delta_VISION_POSITION_DELTA * src, DST * dst) {\
        DST##_time_usec_SET( pvision_position_delta_time_usec_GET( src  ), dst  );\
        DST##_time_delta_usec_SET( pvision_position_delta_time_delta_usec_GET( src  ), dst  );\
        Vvision_position_delta_angle_delta item_angle_delta = pvision_position_delta_angle_delta_GET( src  );\
       DST##_angle_delta_SET( &item_angle_delta, dst );\
        Vvision_position_delta_position_delta item_position_delta = pvision_position_delta_position_delta_GET( src  );\
       DST##_position_delta_SET( &item_position_delta, dst );\
        DST##_confidence_SET( pvision_position_delta_confidence_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_delta_usec_GET( <SRC> * src ){}
static inline float <SRC>_angle_delta_GET( <SRC> * src, Vvision_position_delta_angle_delta * dst ){}
static inline float <SRC>_position_delta_GET( <SRC> * src, Vvision_position_delta_position_delta * dst ){}
static inline float <SRC>_confidence_GET( <SRC> * src ){}
*/

#define pvision_position_delta_VISION_POSITION_DELTA_PULL_FROM(SRC)\
    static inline void pvision_position_delta_VISION_POSITION_DELTA_pull_from_##SRC ( SRC * src, pvision_position_delta_VISION_POSITION_DELTA * dst) {\
        pvision_position_delta_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        pvision_position_delta_time_delta_usec_SET( SRC##_time_delta_usec_GET(src ), dst  );\
       Vvision_position_delta_angle_delta item_angle_delta = pvision_position_delta_angle_delta_SET( NULL, dst  );\
       SRC##_angle_delta_GET( src, &item_angle_delta );\
       Vvision_position_delta_position_delta item_position_delta = pvision_position_delta_position_delta_SET( NULL, dst  );\
       SRC##_position_delta_GET( src, &item_position_delta );\
        pvision_position_delta_confidence_SET( SRC##_confidence_GET(src ), dst  );\
    }

/**
*A message containing logged data (see also MAV_CMD_LOGGING_START) */

typedef Pack LOGGING_DATA_logging_data; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t plogging_data_LOGGING_DATA;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline plogging_data_LOGGING_DATA *plogging_data_LOGGING_DATA_from(LOGGING_DATA_logging_data *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vlogging_data_daTa;
//Maximum field array length constant
#define Plogging_data_daTa_len  ( 249 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_sequence_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_length_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_first_message_offset_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vlogging_data_daTa * src, <DST> * dst  ){}
*/

#define plogging_data_LOGGING_DATA_PUSH_INTO(DST)\
    static inline void plogging_data_LOGGING_DATA_push_into_##DST ( plogging_data_LOGGING_DATA * src, DST * dst) {\
        DST##_sequence_SET( plogging_data_sequence_GET( src  ), dst  );\
        DST##_target_system_SET( plogging_data_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( plogging_data_target_component_GET( src  ), dst  );\
        DST##_length_SET( plogging_data_length_GET( src  ), dst  );\
        DST##_first_message_offset_SET( plogging_data_first_message_offset_GET( src  ), dst  );\
        Vlogging_data_daTa item_daTa = plogging_data_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_sequence_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_length_GET( <SRC> * src ){}
static inline int8_t <SRC>_first_message_offset_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vlogging_data_daTa * dst ){}
*/

#define plogging_data_LOGGING_DATA_PULL_FROM(SRC)\
    static inline void plogging_data_LOGGING_DATA_pull_from_##SRC ( SRC * src, plogging_data_LOGGING_DATA * dst) {\
        plogging_data_sequence_SET( SRC##_sequence_GET(src ), dst  );\
        plogging_data_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        plogging_data_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        plogging_data_length_SET( SRC##_length_GET(src ), dst  );\
        plogging_data_first_message_offset_SET( SRC##_first_message_offset_GET(src ), dst  );\
       Vlogging_data_daTa item_daTa = plogging_data_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*Read registers for a device */

typedef Pack DEVICE_OP_READ_device_op_read; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pdevice_op_read_DEVICE_OP_READ;// data navigator over pack fields data
/**
															* Wrap DEVICE_OP_READ in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pdevice_op_read_DEVICE_OP_READ *DEVICE_OP_READ_device_op_read_wrap(DEVICE_OP_READ_device_op_read *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline DEVICE_OP_READ_device_op_read *pdevice_op_read_DEVICE_OP_READ_unwrap(pdevice_op_read_DEVICE_OP_READ *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vdevice_op_read_busname;
//Maximum field array length constant
#define Pdevice_op_read_busname_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_request_id_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_bus_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_address_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_regstart_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_count_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_bustype_SET( e_DEVICE_OP_BUSTYPE * src, <DST> * dst  ){}
static inline void <DST>_busname_SET( Vdevice_op_read_busname * src, <DST> * dst  ){}
*/

#define pdevice_op_read_DEVICE_OP_READ_PUSH_INTO(DST)\
    static inline void pdevice_op_read_DEVICE_OP_READ_push_into_##DST ( pdevice_op_read_DEVICE_OP_READ * src, DST * dst) {\
        DST##_request_id_SET( pdevice_op_read_request_id_GET( src  ), dst  );\
        DST##_target_system_SET( pdevice_op_read_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pdevice_op_read_target_component_GET( src  ), dst  );\
        DST##_bus_SET( pdevice_op_read_bus_GET( src  ), dst  );\
        DST##_address_SET( pdevice_op_read_address_GET( src  ), dst  );\
        DST##_regstart_SET( pdevice_op_read_regstart_GET( src  ), dst  );\
        DST##_count_SET( pdevice_op_read_count_GET( src  ), dst  );\
        e_DEVICE_OP_BUSTYPE  item_bustype;\
        if( pdevice_op_read_bustype_GET( src, &item_bustype ) ){\
            DST##_bustype_SET( item_bustype , dst  );\
        }\
        Vdevice_op_read_busname  item_busname;\
        if( pdevice_op_read_busname_GET( src, &item_busname ) ){\
            DST##_busname_SET( &item_busname, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_request_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_bus_GET( <SRC> * src ){}
static inline int8_t <SRC>_address_GET( <SRC> * src ){}
static inline int8_t <SRC>_regstart_GET( <SRC> * src ){}
static inline int8_t <SRC>_count_GET( <SRC> * src ){}
static inline bool  <SRC>_bustype_item_exist( <SRC> * src ){}
static inline e_DEVICE_OP_BUSTYPE <SRC>_bustype_GET( <SRC> * src ){}
static inline size_t  <SRC>_busname_item_exist( <SRC> * src  ){}
static inline void <SRC>_busname_GET( <SRC> * src, Vdevice_op_read_busname * dst ){}
*/

#define pdevice_op_read_DEVICE_OP_READ_PULL_FROM(SRC)\
    static inline void pdevice_op_read_DEVICE_OP_READ_pull_from_##SRC ( SRC * src, pdevice_op_read_DEVICE_OP_READ * dst) {\
        pdevice_op_read_request_id_SET( SRC##_request_id_GET(src ), dst  );\
        pdevice_op_read_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pdevice_op_read_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pdevice_op_read_bus_SET( SRC##_bus_GET(src ), dst  );\
        pdevice_op_read_address_SET( SRC##_address_GET(src ), dst  );\
        pdevice_op_read_regstart_SET( SRC##_regstart_GET(src ), dst  );\
        pdevice_op_read_count_SET( SRC##_count_GET(src ), dst  );\
        if( SRC##_bustype_item_exist(src ) )\
        pdevice_op_read_bustype_SET( pdevice_op_read_DEVICE_OP_READ_bustype_GET( src ),  dst  );\
        const size_t len_busname = SRC##_busname_item_exist(src );\
        if( len_busname ){\
            Vdevice_op_read_busname    item_busname = pdevice_op_read_busname_SET( NULL, len_busname, dst  );\
            pdevice_op_read_DEVICE_OP_READ_busname_GET(src, &item_busname );\
        }\
    }

/**
*Reports progress of compass calibration. */

typedef Pack MAG_CAL_PROGRESS_mag_cal_progress; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pmag_cal_progress_MAG_CAL_PROGRESS;// data navigator over pack fields data
/**
															* Wrap MAG_CAL_PROGRESS in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pmag_cal_progress_MAG_CAL_PROGRESS *MAG_CAL_PROGRESS_mag_cal_progress_wrap(MAG_CAL_PROGRESS_mag_cal_progress *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline MAG_CAL_PROGRESS_mag_cal_progress *pmag_cal_progress_MAG_CAL_PROGRESS_unwrap(pmag_cal_progress_MAG_CAL_PROGRESS *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vmag_cal_progress_completion_mask;
//Maximum field array length constant
#define Pmag_cal_progress_completion_mask_len  ( 10 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_compass_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_cal_mask_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_attempt_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_completion_pct_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_completion_mask_SET( Vmag_cal_progress_completion_mask * src, <DST> * dst  ){}
static inline void <DST>_direction_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_direction_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_direction_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_cal_status_SET( e_MAG_CAL_STATUS * src, <DST> * dst  ){}
*/

#define pmag_cal_progress_MAG_CAL_PROGRESS_PUSH_INTO(DST)\
    static inline void pmag_cal_progress_MAG_CAL_PROGRESS_push_into_##DST ( pmag_cal_progress_MAG_CAL_PROGRESS * src, DST * dst) {\
        DST##_compass_id_SET( pmag_cal_progress_compass_id_GET( src  ), dst  );\
        DST##_cal_mask_SET( pmag_cal_progress_cal_mask_GET( src  ), dst  );\
        DST##_attempt_SET( pmag_cal_progress_attempt_GET( src  ), dst  );\
        DST##_completion_pct_SET( pmag_cal_progress_completion_pct_GET( src  ), dst  );\
        Vmag_cal_progress_completion_mask item_completion_mask = pmag_cal_progress_completion_mask_GET( src  );\
       DST##_completion_mask_SET( &item_completion_mask, dst );\
        DST##_direction_x_SET( pmag_cal_progress_direction_x_GET( src  ), dst  );\
        DST##_direction_y_SET( pmag_cal_progress_direction_y_GET( src  ), dst  );\
        DST##_direction_z_SET( pmag_cal_progress_direction_z_GET( src  ), dst  );\
        e_MAG_CAL_STATUS  item_cal_status;\
        if( pmag_cal_progress_cal_status_GET( src, &item_cal_status ) ){\
            DST##_cal_status_SET( item_cal_status , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_compass_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_cal_mask_GET( <SRC> * src ){}
static inline int8_t <SRC>_attempt_GET( <SRC> * src ){}
static inline int8_t <SRC>_completion_pct_GET( <SRC> * src ){}
static inline int8_t <SRC>_completion_mask_GET( <SRC> * src, Vmag_cal_progress_completion_mask * dst ){}
static inline float <SRC>_direction_x_GET( <SRC> * src ){}
static inline float <SRC>_direction_y_GET( <SRC> * src ){}
static inline float <SRC>_direction_z_GET( <SRC> * src ){}
static inline bool  <SRC>_cal_status_item_exist( <SRC> * src ){}
static inline e_MAG_CAL_STATUS <SRC>_cal_status_GET( <SRC> * src ){}
*/

#define pmag_cal_progress_MAG_CAL_PROGRESS_PULL_FROM(SRC)\
    static inline void pmag_cal_progress_MAG_CAL_PROGRESS_pull_from_##SRC ( SRC * src, pmag_cal_progress_MAG_CAL_PROGRESS * dst) {\
        pmag_cal_progress_compass_id_SET( SRC##_compass_id_GET(src ), dst  );\
        pmag_cal_progress_cal_mask_SET( SRC##_cal_mask_GET(src ), dst  );\
        pmag_cal_progress_attempt_SET( SRC##_attempt_GET(src ), dst  );\
        pmag_cal_progress_completion_pct_SET( SRC##_completion_pct_GET(src ), dst  );\
       Vmag_cal_progress_completion_mask item_completion_mask = pmag_cal_progress_completion_mask_SET( NULL, dst  );\
       SRC##_completion_mask_GET( src, &item_completion_mask );\
        pmag_cal_progress_direction_x_SET( SRC##_direction_x_GET(src ), dst  );\
        pmag_cal_progress_direction_y_SET( SRC##_direction_y_GET(src ), dst  );\
        pmag_cal_progress_direction_z_SET( SRC##_direction_z_GET(src ), dst  );\
        if( SRC##_cal_status_item_exist(src ) )\
        pmag_cal_progress_cal_status_SET( pmag_cal_progress_MAG_CAL_PROGRESS_cal_status_GET( src ),  dst  );\
    }

/**
*The IMU readings in SI units in NED body frame */

typedef Pack HIGHRES_IMU_highres_imu; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t phighres_imu_HIGHRES_IMU;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline phighres_imu_HIGHRES_IMU *phighres_imu_HIGHRES_IMU_from(HIGHRES_IMU_highres_imu *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_fields_updated_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_xacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zacc_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ygyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zgyro_SET( float * src, <DST> * dst  ){}
static inline void <DST>_xmag_SET( float * src, <DST> * dst  ){}
static inline void <DST>_ymag_SET( float * src, <DST> * dst  ){}
static inline void <DST>_zmag_SET( float * src, <DST> * dst  ){}
static inline void <DST>_abs_pressure_SET( float * src, <DST> * dst  ){}
static inline void <DST>_diff_pressure_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pressure_alt_SET( float * src, <DST> * dst  ){}
static inline void <DST>_temperature_SET( float * src, <DST> * dst  ){}
*/

#define phighres_imu_HIGHRES_IMU_PUSH_INTO(DST)\
    static inline void phighres_imu_HIGHRES_IMU_push_into_##DST ( phighres_imu_HIGHRES_IMU * src, DST * dst) {\
        DST##_fields_updated_SET( phighres_imu_fields_updated_GET( src  ), dst  );\
        DST##_time_usec_SET( phighres_imu_time_usec_GET( src  ), dst  );\
        DST##_xacc_SET( phighres_imu_xacc_GET( src  ), dst  );\
        DST##_yacc_SET( phighres_imu_yacc_GET( src  ), dst  );\
        DST##_zacc_SET( phighres_imu_zacc_GET( src  ), dst  );\
        DST##_xgyro_SET( phighres_imu_xgyro_GET( src  ), dst  );\
        DST##_ygyro_SET( phighres_imu_ygyro_GET( src  ), dst  );\
        DST##_zgyro_SET( phighres_imu_zgyro_GET( src  ), dst  );\
        DST##_xmag_SET( phighres_imu_xmag_GET( src  ), dst  );\
        DST##_ymag_SET( phighres_imu_ymag_GET( src  ), dst  );\
        DST##_zmag_SET( phighres_imu_zmag_GET( src  ), dst  );\
        DST##_abs_pressure_SET( phighres_imu_abs_pressure_GET( src  ), dst  );\
        DST##_diff_pressure_SET( phighres_imu_diff_pressure_GET( src  ), dst  );\
        DST##_pressure_alt_SET( phighres_imu_pressure_alt_GET( src  ), dst  );\
        DST##_temperature_SET( phighres_imu_temperature_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_fields_updated_GET( <SRC> * src ){}
static inline int64_t <SRC>_time_usec_GET( <SRC> * src ){}
static inline float <SRC>_xacc_GET( <SRC> * src ){}
static inline float <SRC>_yacc_GET( <SRC> * src ){}
static inline float <SRC>_zacc_GET( <SRC> * src ){}
static inline float <SRC>_xgyro_GET( <SRC> * src ){}
static inline float <SRC>_ygyro_GET( <SRC> * src ){}
static inline float <SRC>_zgyro_GET( <SRC> * src ){}
static inline float <SRC>_xmag_GET( <SRC> * src ){}
static inline float <SRC>_ymag_GET( <SRC> * src ){}
static inline float <SRC>_zmag_GET( <SRC> * src ){}
static inline float <SRC>_abs_pressure_GET( <SRC> * src ){}
static inline float <SRC>_diff_pressure_GET( <SRC> * src ){}
static inline float <SRC>_pressure_alt_GET( <SRC> * src ){}
static inline float <SRC>_temperature_GET( <SRC> * src ){}
*/

#define phighres_imu_HIGHRES_IMU_PULL_FROM(SRC)\
    static inline void phighres_imu_HIGHRES_IMU_pull_from_##SRC ( SRC * src, phighres_imu_HIGHRES_IMU * dst) {\
        phighres_imu_fields_updated_SET( SRC##_fields_updated_GET(src ), dst  );\
        phighres_imu_time_usec_SET( SRC##_time_usec_GET(src ), dst  );\
        phighres_imu_xacc_SET( SRC##_xacc_GET(src ), dst  );\
        phighres_imu_yacc_SET( SRC##_yacc_GET(src ), dst  );\
        phighres_imu_zacc_SET( SRC##_zacc_GET(src ), dst  );\
        phighres_imu_xgyro_SET( SRC##_xgyro_GET(src ), dst  );\
        phighres_imu_ygyro_SET( SRC##_ygyro_GET(src ), dst  );\
        phighres_imu_zgyro_SET( SRC##_zgyro_GET(src ), dst  );\
        phighres_imu_xmag_SET( SRC##_xmag_GET(src ), dst  );\
        phighres_imu_ymag_SET( SRC##_ymag_GET(src ), dst  );\
        phighres_imu_zmag_SET( SRC##_zmag_GET(src ), dst  );\
        phighres_imu_abs_pressure_SET( SRC##_abs_pressure_GET(src ), dst  );\
        phighres_imu_diff_pressure_SET( SRC##_diff_pressure_GET(src ), dst  );\
        phighres_imu_pressure_alt_SET( SRC##_pressure_alt_GET(src ), dst  );\
        phighres_imu_temperature_SET( SRC##_temperature_GET(src ), dst  );\
    }

/**
*Provides state for additional features */

typedef Pack EXTENDED_SYS_STATE_extended_sys_state; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pextended_sys_state_EXTENDED_SYS_STATE;// data navigator over pack fields data
/**
															* Wrap EXTENDED_SYS_STATE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pextended_sys_state_EXTENDED_SYS_STATE *EXTENDED_SYS_STATE_extended_sys_state_wrap(EXTENDED_SYS_STATE_extended_sys_state *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline EXTENDED_SYS_STATE_extended_sys_state *pextended_sys_state_EXTENDED_SYS_STATE_unwrap(pextended_sys_state_EXTENDED_SYS_STATE *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_vtol_state_SET( e_MAV_VTOL_STATE * src, <DST> * dst  ){}
static inline void <DST>_landed_state_SET( e_MAV_LANDED_STATE * src, <DST> * dst  ){}
*/

#define pextended_sys_state_EXTENDED_SYS_STATE_PUSH_INTO(DST)\
    static inline void pextended_sys_state_EXTENDED_SYS_STATE_push_into_##DST ( pextended_sys_state_EXTENDED_SYS_STATE * src, DST * dst) {\
        e_MAV_VTOL_STATE  item_vtol_state;\
        if( pextended_sys_state_vtol_state_GET( src, &item_vtol_state ) ){\
            DST##_vtol_state_SET( item_vtol_state , dst  );\
        }\
        e_MAV_LANDED_STATE  item_landed_state;\
        if( pextended_sys_state_landed_state_GET( src, &item_landed_state ) ){\
            DST##_landed_state_SET( item_landed_state , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline bool  <SRC>_vtol_state_item_exist( <SRC> * src ){}
static inline e_MAV_VTOL_STATE <SRC>_vtol_state_GET( <SRC> * src ){}
static inline bool  <SRC>_landed_state_item_exist( <SRC> * src ){}
static inline e_MAV_LANDED_STATE <SRC>_landed_state_GET( <SRC> * src ){}
*/

#define pextended_sys_state_EXTENDED_SYS_STATE_PULL_FROM(SRC)\
    static inline void pextended_sys_state_EXTENDED_SYS_STATE_pull_from_##SRC ( SRC * src, pextended_sys_state_EXTENDED_SYS_STATE * dst) {\
        if( SRC##_vtol_state_item_exist(src ) )\
        pextended_sys_state_vtol_state_SET( pextended_sys_state_EXTENDED_SYS_STATE_vtol_state_GET( src ),  dst  );\
        if( SRC##_landed_state_item_exist(src ) )\
        pextended_sys_state_landed_state_SET( pextended_sys_state_EXTENDED_SYS_STATE_landed_state_GET( src ),  dst  );\
    }

/**
*Dynamic data used to generate ADS-B out transponder data (send at 5Hz) */

typedef Pack UAVIONIX_ADSB_OUT_DYNAMIC_uavionix_adsb_out_dynamic; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC;// data navigator over pack fields data
/**
															* Wrap UAVIONIX_ADSB_OUT_DYNAMIC in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *UAVIONIX_ADSB_OUT_DYNAMIC_uavionix_adsb_out_dynamic_wrap(UAVIONIX_ADSB_OUT_DYNAMIC_uavionix_adsb_out_dynamic *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline UAVIONIX_ADSB_OUT_DYNAMIC_uavionix_adsb_out_dynamic *puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_unwrap(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_accuracyVert_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_accuracyVel_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_squawk_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_utcTime_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_accuracyHor_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_gpsLat_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_gpsLon_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_gpsAlt_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_numSats_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_baroAltMSL_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_velVert_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_velNS_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_VelEW_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_gpsFix_SET( e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX * src, <DST> * dst  ){}
static inline void <DST>_emergencyStatus_SET( e_UAVIONIX_ADSB_EMERGENCY_STATUS * src, <DST> * dst  ){}
static inline void <DST>_state_SET( e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE * src, <DST> * dst  ){}
*/

#define puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_PUSH_INTO(DST)\
    static inline void puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_push_into_##DST ( puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC * src, DST * dst) {\
        DST##_accuracyVert_SET( puavionix_adsb_out_dynamic_accuracyVert_GET( src  ), dst  );\
        DST##_accuracyVel_SET( puavionix_adsb_out_dynamic_accuracyVel_GET( src  ), dst  );\
        DST##_squawk_SET( puavionix_adsb_out_dynamic_squawk_GET( src  ), dst  );\
        DST##_utcTime_SET( puavionix_adsb_out_dynamic_utcTime_GET( src  ), dst  );\
        DST##_accuracyHor_SET( puavionix_adsb_out_dynamic_accuracyHor_GET( src  ), dst  );\
        DST##_gpsLat_SET( puavionix_adsb_out_dynamic_gpsLat_GET( src  ), dst  );\
        DST##_gpsLon_SET( puavionix_adsb_out_dynamic_gpsLon_GET( src  ), dst  );\
        DST##_gpsAlt_SET( puavionix_adsb_out_dynamic_gpsAlt_GET( src  ), dst  );\
        DST##_numSats_SET( puavionix_adsb_out_dynamic_numSats_GET( src  ), dst  );\
        DST##_baroAltMSL_SET( puavionix_adsb_out_dynamic_baroAltMSL_GET( src  ), dst  );\
        DST##_velVert_SET( puavionix_adsb_out_dynamic_velVert_GET( src  ), dst  );\
        DST##_velNS_SET( puavionix_adsb_out_dynamic_velNS_GET( src  ), dst  );\
        DST##_VelEW_SET( puavionix_adsb_out_dynamic_VelEW_GET( src  ), dst  );\
        e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX  item_gpsFix;\
        if( puavionix_adsb_out_dynamic_gpsFix_GET( src, &item_gpsFix ) ){\
            DST##_gpsFix_SET( item_gpsFix , dst  );\
        }\
        e_UAVIONIX_ADSB_EMERGENCY_STATUS  item_emergencyStatus;\
        if( puavionix_adsb_out_dynamic_emergencyStatus_GET( src, &item_emergencyStatus ) ){\
            DST##_emergencyStatus_SET( item_emergencyStatus , dst  );\
        }\
        e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE  item_state;\
        if( puavionix_adsb_out_dynamic_state_GET( src, &item_state ) ){\
            DST##_state_SET( item_state , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_accuracyVert_GET( <SRC> * src ){}
static inline int16_t <SRC>_accuracyVel_GET( <SRC> * src ){}
static inline int16_t <SRC>_squawk_GET( <SRC> * src ){}
static inline int32_t <SRC>_utcTime_GET( <SRC> * src ){}
static inline int32_t <SRC>_accuracyHor_GET( <SRC> * src ){}
static inline int32_t <SRC>_gpsLat_GET( <SRC> * src ){}
static inline int32_t <SRC>_gpsLon_GET( <SRC> * src ){}
static inline int32_t <SRC>_gpsAlt_GET( <SRC> * src ){}
static inline int8_t <SRC>_numSats_GET( <SRC> * src ){}
static inline int32_t <SRC>_baroAltMSL_GET( <SRC> * src ){}
static inline int16_t <SRC>_velVert_GET( <SRC> * src ){}
static inline int16_t <SRC>_velNS_GET( <SRC> * src ){}
static inline int16_t <SRC>_VelEW_GET( <SRC> * src ){}
static inline bool  <SRC>_gpsFix_item_exist( <SRC> * src ){}
static inline e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX <SRC>_gpsFix_GET( <SRC> * src ){}
static inline bool  <SRC>_emergencyStatus_item_exist( <SRC> * src ){}
static inline e_UAVIONIX_ADSB_EMERGENCY_STATUS <SRC>_emergencyStatus_GET( <SRC> * src ){}
static inline bool  <SRC>_state_item_exist( <SRC> * src ){}
static inline e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE <SRC>_state_GET( <SRC> * src ){}
*/

#define puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_PULL_FROM(SRC)\
    static inline void puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_pull_from_##SRC ( SRC * src, puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC * dst) {\
        puavionix_adsb_out_dynamic_accuracyVert_SET( SRC##_accuracyVert_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_accuracyVel_SET( SRC##_accuracyVel_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_squawk_SET( SRC##_squawk_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_utcTime_SET( SRC##_utcTime_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_accuracyHor_SET( SRC##_accuracyHor_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_gpsLat_SET( SRC##_gpsLat_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_gpsLon_SET( SRC##_gpsLon_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_gpsAlt_SET( SRC##_gpsAlt_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_numSats_SET( SRC##_numSats_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_baroAltMSL_SET( SRC##_baroAltMSL_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_velVert_SET( SRC##_velVert_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_velNS_SET( SRC##_velNS_GET(src ), dst  );\
        puavionix_adsb_out_dynamic_VelEW_SET( SRC##_VelEW_GET(src ), dst  );\
        if( SRC##_gpsFix_item_exist(src ) )\
        puavionix_adsb_out_dynamic_gpsFix_SET( puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_gpsFix_GET( src ),  dst  );\
        if( SRC##_emergencyStatus_item_exist(src ) )\
        puavionix_adsb_out_dynamic_emergencyStatus_SET( puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_emergencyStatus_GET( src ),  dst  );\
        if( SRC##_state_item_exist(src ) )\
        puavionix_adsb_out_dynamic_state_SET( puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC_state_GET( src ),  dst  );\
    }

/**
*Response from a GOPRO_COMMAND get request */

typedef Pack GOPRO_GET_RESPONSE_gopro_get_response; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pgopro_get_response_GOPRO_GET_RESPONSE;// data navigator over pack fields data
/**
															* Wrap GOPRO_GET_RESPONSE in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pgopro_get_response_GOPRO_GET_RESPONSE *GOPRO_GET_RESPONSE_gopro_get_response_wrap(GOPRO_GET_RESPONSE_gopro_get_response *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline GOPRO_GET_RESPONSE_gopro_get_response *pgopro_get_response_GOPRO_GET_RESPONSE_unwrap(pgopro_get_response_GOPRO_GET_RESPONSE *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgopro_get_response_value;
//Maximum field array length constant
#define Pgopro_get_response_value_len  ( 4 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_value_SET( Vgopro_get_response_value * src, <DST> * dst  ){}
static inline void <DST>_cmd_id_SET( e_GOPRO_COMMAND * src, <DST> * dst  ){}
static inline void <DST>_status_SET( e_GOPRO_REQUEST_STATUS * src, <DST> * dst  ){}
*/

#define pgopro_get_response_GOPRO_GET_RESPONSE_PUSH_INTO(DST)\
    static inline void pgopro_get_response_GOPRO_GET_RESPONSE_push_into_##DST ( pgopro_get_response_GOPRO_GET_RESPONSE * src, DST * dst) {\
        Vgopro_get_response_value item_value = pgopro_get_response_value_GET( src  );\
       DST##_value_SET( &item_value, dst );\
        e_GOPRO_COMMAND  item_cmd_id;\
        if( pgopro_get_response_cmd_id_GET( src, &item_cmd_id ) ){\
            DST##_cmd_id_SET( item_cmd_id , dst  );\
        }\
        e_GOPRO_REQUEST_STATUS  item_status;\
        if( pgopro_get_response_status_GET( src, &item_status ) ){\
            DST##_status_SET( item_status , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_value_GET( <SRC> * src, Vgopro_get_response_value * dst ){}
static inline bool  <SRC>_cmd_id_item_exist( <SRC> * src ){}
static inline e_GOPRO_COMMAND <SRC>_cmd_id_GET( <SRC> * src ){}
static inline bool  <SRC>_status_item_exist( <SRC> * src ){}
static inline e_GOPRO_REQUEST_STATUS <SRC>_status_GET( <SRC> * src ){}
*/

#define pgopro_get_response_GOPRO_GET_RESPONSE_PULL_FROM(SRC)\
    static inline void pgopro_get_response_GOPRO_GET_RESPONSE_pull_from_##SRC ( SRC * src, pgopro_get_response_GOPRO_GET_RESPONSE * dst) {\
       Vgopro_get_response_value item_value = pgopro_get_response_value_SET( NULL, dst  );\
       SRC##_value_GET( src, &item_value );\
        if( SRC##_cmd_id_item_exist(src ) )\
        pgopro_get_response_cmd_id_SET( pgopro_get_response_GOPRO_GET_RESPONSE_cmd_id_GET( src ),  dst  );\
        if( SRC##_status_item_exist(src ) )\
        pgopro_get_response_status_SET( pgopro_get_response_GOPRO_GET_RESPONSE_status_GET( src ),  dst  );\
    }

/**
*data for injecting into the onboard GPS (used for DGPS) */

typedef Pack GPS_INJECT_DATA_gps_inject_data; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pgps_inject_data_GPS_INJECT_DATA;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pgps_inject_data_GPS_INJECT_DATA *pgps_inject_data_GPS_INJECT_DATA_from(GPS_INJECT_DATA_gps_inject_data *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgps_inject_data_daTa;
//Maximum field array length constant
#define Pgps_inject_data_daTa_len  ( 110 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_len_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vgps_inject_data_daTa * src, <DST> * dst  ){}
*/

#define pgps_inject_data_GPS_INJECT_DATA_PUSH_INTO(DST)\
    static inline void pgps_inject_data_GPS_INJECT_DATA_push_into_##DST ( pgps_inject_data_GPS_INJECT_DATA * src, DST * dst) {\
        DST##_target_system_SET( pgps_inject_data_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pgps_inject_data_target_component_GET( src  ), dst  );\
        DST##_len_SET( pgps_inject_data_len_GET( src  ), dst  );\
        Vgps_inject_data_daTa item_daTa = pgps_inject_data_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_len_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vgps_inject_data_daTa * dst ){}
*/

#define pgps_inject_data_GPS_INJECT_DATA_PULL_FROM(SRC)\
    static inline void pgps_inject_data_GPS_INJECT_DATA_pull_from_##SRC ( SRC * src, pgps_inject_data_GPS_INJECT_DATA * dst) {\
        pgps_inject_data_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pgps_inject_data_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pgps_inject_data_len_SET( SRC##_len_GET(src ), dst  );\
       Vgps_inject_data_daTa item_daTa = pgps_inject_data_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }

/**
*Transceiver heartbeat with health report (updated every 10s) */

typedef Pack UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_uavionix_adsb_transceiver_health_report; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;// data navigator over pack fields data
/**
															* Wrap UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_uavionix_adsb_transceiver_health_report_wrap(UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_uavionix_adsb_transceiver_health_report *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_uavionix_adsb_transceiver_health_report *puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_unwrap(puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *wrapper) { return unwrap_pack(wrapper); }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_rfHealth_SET( e_UAVIONIX_ADSB_RF_HEALTH * src, <DST> * dst  ){}
*/

#define puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PUSH_INTO(DST)\
    static inline void puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_push_into_##DST ( puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT * src, DST * dst) {\
        e_UAVIONIX_ADSB_RF_HEALTH  item_rfHealth;\
        if( puavionix_adsb_transceiver_health_report_rfHealth_GET( src, &item_rfHealth ) ){\
            DST##_rfHealth_SET( item_rfHealth , dst  );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline bool  <SRC>_rfHealth_item_exist( <SRC> * src ){}
static inline e_UAVIONIX_ADSB_RF_HEALTH <SRC>_rfHealth_GET( <SRC> * src ){}
*/

#define puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PULL_FROM(SRC)\
    static inline void puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_pull_from_##SRC ( SRC * src, puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT * dst) {\
        if( SRC##_rfHealth_item_exist(src ) )\
        puavionix_adsb_transceiver_health_report_rfHealth_SET( puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_rfHealth_GET( src ),  dst  );\
    }

/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
*				 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0) */

typedef Pack ATTITUDE_QUATERNION_COV_attitude_quaternion_cov; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV_from(ATTITUDE_QUATERNION_COV_attitude_quaternion_cov *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vattitude_quaternion_cov_q;
//Maximum field array length constant
#define Pattitude_quaternion_cov_q_len  ( 4 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vattitude_quaternion_cov_covariance;
//Maximum field array length constant
#define Pattitude_quaternion_cov_covariance_len  ( 9 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_q_SET( Vattitude_quaternion_cov_q * src, <DST> * dst  ){}
static inline void <DST>_rollspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitchspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yawspeed_SET( float * src, <DST> * dst  ){}
static inline void <DST>_covariance_SET( Vattitude_quaternion_cov_covariance * src, <DST> * dst  ){}
*/

#define pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV_PUSH_INTO(DST)\
    static inline void pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV_push_into_##DST ( pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV * src, DST * dst) {\
        DST##_time_usec_SET( pattitude_quaternion_cov_time_usec_GET( src  ), dst  );\
        Vattitude_quaternion_cov_q item_q = pattitude_quaternion_cov_q_GET( src  );\
       DST##_q_SET( &item_q, dst );\
        DST##_rollspeed_SET( pattitude_quaternion_cov_rollspeed_GET( src  ), dst  );\
        DST##_pitchspeed_SET( pattitude_quaternion_cov_pitchspeed_GET( src  ), dst  );\
        DST##_yawspeed_SET( pattitude_quaternion_cov_yawspeed_GET( src  ), dst  );\
        Vattitude_quaternion_cov_covariance item_covariance = pattitude_quaternion_cov_covariance_GET( src  );\
       DST##_covariance_SET( &item_covariance, dst );\
    }

/**
*Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
*				 efficient way for testing new messages and getting experimental debug output */

typedef Pack NAMED_VALUE_INT_named_value_int; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

/**
															* Packet wrapper. Raw packs with varSIZE fields, to expose their data, needs a special navigator that wrap the pack
															*/
typedef Cursor pnamed_value_int_NAMED_VALUE_INT;// data navigator over pack fields data
/**
															* Wrap NAMED_VALUE_INT in the data navigator. A read-only pack can be wrapped in several navigators
															*/
static inline pnamed_value_int_NAMED_VALUE_INT *NAMED_VALUE_INT_named_value_int_wrap(NAMED_VALUE_INT_named_value_int *pack, Cursors wrapper) { return wrap_pack(pack, wrapper); }

/**
															*Unwrap/disjoint the pack with the data navigator.
															*/
static inline NAMED_VALUE_INT_named_value_int *pnamed_value_int_NAMED_VALUE_INT_unwrap(pnamed_value_int_NAMED_VALUE_INT *wrapper) { return unwrap_pack(wrapper); }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vnamed_value_int_name;
//Maximum field array length constant
#define Pnamed_value_int_name_len_max  ( 255 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_time_boot_ms_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_value_SET( int32_t * src, <DST> * dst  ){}
static inline void <DST>_name_SET( Vnamed_value_int_name * src, <DST> * dst  ){}
*/

#define pnamed_value_int_NAMED_VALUE_INT_PUSH_INTO(DST)\
    static inline void pnamed_value_int_NAMED_VALUE_INT_push_into_##DST ( pnamed_value_int_NAMED_VALUE_INT * src, DST * dst) {\
        DST##_time_boot_ms_SET( pnamed_value_int_time_boot_ms_GET( src  ), dst  );\
        DST##_value_SET( pnamed_value_int_value_GET( src  ), dst  );\
        Vnamed_value_int_name  item_name;\
        if( pnamed_value_int_name_GET( src, &item_name ) ){\
            DST##_name_SET( &item_name, dst );\
        }\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int32_t <SRC>_time_boot_ms_GET( <SRC> * src ){}
static inline int32_t <SRC>_value_GET( <SRC> * src ){}
static inline size_t  <SRC>_name_item_exist( <SRC> * src  ){}
static inline void <SRC>_name_GET( <SRC> * src, Vnamed_value_int_name * dst ){}
*/

#define pnamed_value_int_NAMED_VALUE_INT_PULL_FROM(SRC)\
    static inline void pnamed_value_int_NAMED_VALUE_INT_pull_from_##SRC ( SRC * src, pnamed_value_int_NAMED_VALUE_INT * dst) {\
        pnamed_value_int_time_boot_ms_SET( SRC##_time_boot_ms_GET(src ), dst  );\
        pnamed_value_int_value_SET( SRC##_value_GET(src ), dst  );\
        const size_t len_name = SRC##_name_item_exist(src );\
        if( len_name ){\
            Vnamed_value_int_name    item_name = pnamed_value_int_name_SET( NULL, len_name, dst  );\
            pnamed_value_int_NAMED_VALUE_INT_name_GET(src, &item_name );\
        }\
    }

/**
*RPM sensor output */

typedef Pack RPM_rpm; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t prpm_RPM;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline prpm_RPM *prpm_RPM_from(RPM_rpm *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_rpm1_SET( float * src, <DST> * dst  ){}
static inline void <DST>_rpm2_SET( float * src, <DST> * dst  ){}
*/

#define prpm_RPM_PUSH_INTO(DST)\
    static inline void prpm_RPM_push_into_##DST ( prpm_RPM * src, DST * dst) {\
        DST##_rpm1_SET( prpm_rpm1_GET( src  ), dst  );\
        DST##_rpm2_SET( prpm_rpm2_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_rpm1_GET( <SRC> * src ){}
static inline float <SRC>_rpm2_GET( <SRC> * src ){}
*/

#define prpm_RPM_PULL_FROM(SRC)\
    static inline void prpm_RPM_pull_from_##SRC ( SRC * src, prpm_RPM * dst) {\
        prpm_rpm1_SET( SRC##_rpm1_GET(src ), dst  );\
        prpm_rpm2_SET( SRC##_rpm2_GET(src ), dst  );\
    }

/**
*RTCM message for injecting into the onboard GPS (used for DGPS) */

typedef Pack GPS_RTCM_DATA_gps_rtcm_data; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pgps_rtcm_data_GPS_RTCM_DATA;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pgps_rtcm_data_GPS_RTCM_DATA *pgps_rtcm_data_GPS_RTCM_DATA_from(GPS_RTCM_DATA_gps_rtcm_data *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vgps_rtcm_data_daTa;
//Maximum field array length constant
#define Pgps_rtcm_data_daTa_len  ( 180 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_flags_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_len_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_daTa_SET( Vgps_rtcm_data_daTa * src, <DST> * dst  ){}
*/

#define pgps_rtcm_data_GPS_RTCM_DATA_PUSH_INTO(DST)\
    static inline void pgps_rtcm_data_GPS_RTCM_DATA_push_into_##DST ( pgps_rtcm_data_GPS_RTCM_DATA * src, DST * dst) {\
        DST##_flags_SET( pgps_rtcm_data_flags_GET( src  ), dst  );\
        DST##_len_SET( pgps_rtcm_data_len_GET( src  ), dst  );\
        Vgps_rtcm_data_daTa item_daTa = pgps_rtcm_data_daTa_GET( src  );\
       DST##_daTa_SET( &item_daTa, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_flags_GET( <SRC> * src ){}
static inline int8_t <SRC>_len_GET( <SRC> * src ){}
static inline int8_t <SRC>_daTa_GET( <SRC> * src, Vgps_rtcm_data_daTa * dst ){}
*/

#define pgps_rtcm_data_GPS_RTCM_DATA_PULL_FROM(SRC)\
    static inline void pgps_rtcm_data_GPS_RTCM_DATA_pull_from_##SRC ( SRC * src, pgps_rtcm_data_GPS_RTCM_DATA * dst) {\
        pgps_rtcm_data_flags_SET( SRC##_flags_GET(src ), dst  );\
        pgps_rtcm_data_len_SET( SRC##_len_GET(src ), dst  );\
       Vgps_rtcm_data_daTa item_daTa = pgps_rtcm_data_daTa_SET( NULL, dst  );\
       SRC##_daTa_GET( src, &item_daTa );\
    }


typedef Pack GLOBAL_VISION_POSITION_ESTIMATE_global_vision_position_estimate; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE_from(GLOBAL_VISION_POSITION_ESTIMATE_global_vision_position_estimate *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_usec_SET( int64_t * src, <DST> * dst  ){}
static inline void <DST>_x_SET( float * src, <DST> * dst  ){}
static inline void <DST>_y_SET( float * src, <DST> * dst  ){}
static inline void <DST>_z_SET( float * src, <DST> * dst  ){}
static inline void <DST>_roll_SET( float * src, <DST> * dst  ){}
static inline void <DST>_pitch_SET( float * src, <DST> * dst  ){}
static inline void <DST>_yaw_SET( float * src, <DST> * dst  ){}
*/

#define pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE_PUSH_INTO(DST)\
    static inline void pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE_push_into_##DST ( pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE * src, DST * dst) {\
        DST##_usec_SET( pglobal_vision_position_estimate_usec_GET( src  ), dst  );\
        DST##_x_SET( pglobal_vision_position_estimate_x_GET( src  ), dst  );\
        DST##_y_SET( pglobal_vision_position_estimate_y_GET( src  ), dst  );\
        DST##_z_SET( pglobal_vision_position_estimate_z_GET( src  ), dst  );\
        DST##_roll_SET( pglobal_vision_position_estimate_roll_GET( src  ), dst  );\
        DST##_pitch_SET( pglobal_vision_position_estimate_pitch_GET( src  ), dst  );\
        DST##_yaw_SET( pglobal_vision_position_estimate_yaw_GET( src  ), dst  );\
    }

/**
*File transfer message */

typedef Pack FILE_TRANSFER_PROTOCOL_file_transfer_protocol; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL_from(FILE_TRANSFER_PROTOCOL_file_transfer_protocol *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vfile_transfer_protocol_payload;
//Maximum field array length constant
#define Pfile_transfer_protocol_payload_len  ( 251 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_network_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_payload_SET( Vfile_transfer_protocol_payload * src, <DST> * dst  ){}
*/

#define pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL_PUSH_INTO(DST)\
    static inline void pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL_push_into_##DST ( pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL * src, DST * dst) {\
        DST##_target_network_SET( pfile_transfer_protocol_target_network_GET( src  ), dst  );\
        DST##_target_system_SET( pfile_transfer_protocol_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pfile_transfer_protocol_target_component_GET( src  ), dst  );\
        Vfile_transfer_protocol_payload item_payload = pfile_transfer_protocol_payload_GET( src  );\
       DST##_payload_SET( &item_payload, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_network_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_payload_GET( <SRC> * src, Vfile_transfer_protocol_payload * dst ){}
*/

#define pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL_PULL_FROM(SRC)\
    static inline void pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL_pull_from_##SRC ( SRC * src, pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL * dst) {\
        pfile_transfer_protocol_target_network_SET( SRC##_target_network_GET(src ), dst  );\
        pfile_transfer_protocol_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pfile_transfer_protocol_target_component_SET( SRC##_target_component_GET(src ), dst  );\
       Vfile_transfer_protocol_payload item_payload = pfile_transfer_protocol_payload_SET( NULL, dst  );\
       SRC##_payload_GET( src, &item_payload );\
    }

/**
*Rangefinder reporting */

typedef Pack RANGEFINDER_rangefinder; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t prangefinder_RANGEFINDER;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline prangefinder_RANGEFINDER *prangefinder_RANGEFINDER_from(RANGEFINDER_rangefinder *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_distance_SET( float * src, <DST> * dst  ){}
static inline void <DST>_voltage_SET( float * src, <DST> * dst  ){}
*/

#define prangefinder_RANGEFINDER_PUSH_INTO(DST)\
    static inline void prangefinder_RANGEFINDER_push_into_##DST ( prangefinder_RANGEFINDER * src, DST * dst) {\
        DST##_distance_SET( prangefinder_distance_GET( src  ), dst  );\
        DST##_voltage_SET( prangefinder_voltage_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline float <SRC>_distance_GET( <SRC> * src ){}
static inline float <SRC>_voltage_GET( <SRC> * src ){}
*/

#define prangefinder_RANGEFINDER_PULL_FROM(SRC)\
    static inline void prangefinder_RANGEFINDER_pull_from_##SRC ( SRC * src, prangefinder_RANGEFINDER * dst) {\
        prangefinder_distance_SET( SRC##_distance_GET(src ), dst  );\
        prangefinder_voltage_SET( SRC##_voltage_GET(src ), dst  );\
    }

/**
*Status generated by radio and injected into MAVLink stream. */

typedef Pack RADIO_STATUS_radio_status; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pradio_status_RADIO_STATUS;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pradio_status_RADIO_STATUS *pradio_status_RADIO_STATUS_from(RADIO_STATUS_radio_status *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_rxerrors_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_fixeD_SET( int16_t * src, <DST> * dst  ){}
static inline void <DST>_rssi_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_remrssi_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_txbuf_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_noise_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_remnoise_SET( int8_t * src, <DST> * dst  ){}
*/

#define pradio_status_RADIO_STATUS_PUSH_INTO(DST)\
    static inline void pradio_status_RADIO_STATUS_push_into_##DST ( pradio_status_RADIO_STATUS * src, DST * dst) {\
        DST##_rxerrors_SET( pradio_status_rxerrors_GET( src  ), dst  );\
        DST##_fixeD_SET( pradio_status_fixeD_GET( src  ), dst  );\
        DST##_rssi_SET( pradio_status_rssi_GET( src  ), dst  );\
        DST##_remrssi_SET( pradio_status_remrssi_GET( src  ), dst  );\
        DST##_txbuf_SET( pradio_status_txbuf_GET( src  ), dst  );\
        DST##_noise_SET( pradio_status_noise_GET( src  ), dst  );\
        DST##_remnoise_SET( pradio_status_remnoise_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int16_t <SRC>_rxerrors_GET( <SRC> * src ){}
static inline int16_t <SRC>_fixeD_GET( <SRC> * src ){}
static inline int8_t <SRC>_rssi_GET( <SRC> * src ){}
static inline int8_t <SRC>_remrssi_GET( <SRC> * src ){}
static inline int8_t <SRC>_txbuf_GET( <SRC> * src ){}
static inline int8_t <SRC>_noise_GET( <SRC> * src ){}
static inline int8_t <SRC>_remnoise_GET( <SRC> * src ){}
*/

#define pradio_status_RADIO_STATUS_PULL_FROM(SRC)\
    static inline void pradio_status_RADIO_STATUS_pull_from_##SRC ( SRC * src, pradio_status_RADIO_STATUS * dst) {\
        pradio_status_rxerrors_SET( SRC##_rxerrors_GET(src ), dst  );\
        pradio_status_fixeD_SET( SRC##_fixeD_GET(src ), dst  );\
        pradio_status_rssi_SET( SRC##_rssi_GET(src ), dst  );\
        pradio_status_remrssi_SET( SRC##_remrssi_GET(src ), dst  );\
        pradio_status_txbuf_SET( SRC##_txbuf_GET(src ), dst  );\
        pradio_status_noise_SET( SRC##_noise_GET(src ), dst  );\
        pradio_status_remnoise_SET( SRC##_remnoise_GET(src ), dst  );\
    }

/**
*GCS */

typedef Pack FENCE_POINT_fence_point; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t pfence_point_FENCE_POINT;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline pfence_point_FENCE_POINT *pfence_point_FENCE_POINT_from(FENCE_POINT_fence_point *pack) { return pack->bytes; }

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_target_system_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_target_component_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_idx_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_count_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_lat_SET( float * src, <DST> * dst  ){}
static inline void <DST>_lng_SET( float * src, <DST> * dst  ){}
*/

#define pfence_point_FENCE_POINT_PUSH_INTO(DST)\
    static inline void pfence_point_FENCE_POINT_push_into_##DST ( pfence_point_FENCE_POINT * src, DST * dst) {\
        DST##_target_system_SET( pfence_point_target_system_GET( src  ), dst  );\
        DST##_target_component_SET( pfence_point_target_component_GET( src  ), dst  );\
        DST##_idx_SET( pfence_point_idx_GET( src  ), dst  );\
        DST##_count_SET( pfence_point_count_GET( src  ), dst  );\
        DST##_lat_SET( pfence_point_lat_GET( src  ), dst  );\
        DST##_lng_SET( pfence_point_lng_GET( src  ), dst  );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_target_system_GET( <SRC> * src ){}
static inline int8_t <SRC>_target_component_GET( <SRC> * src ){}
static inline int8_t <SRC>_idx_GET( <SRC> * src ){}
static inline int8_t <SRC>_count_GET( <SRC> * src ){}
static inline float <SRC>_lat_GET( <SRC> * src ){}
static inline float <SRC>_lng_GET( <SRC> * src ){}
*/

#define pfence_point_FENCE_POINT_PULL_FROM(SRC)\
    static inline void pfence_point_FENCE_POINT_pull_from_##SRC ( SRC * src, pfence_point_FENCE_POINT * dst) {\
        pfence_point_target_system_SET( SRC##_target_system_GET(src ), dst  );\
        pfence_point_target_component_SET( SRC##_target_component_GET(src ), dst  );\
        pfence_point_idx_SET( SRC##_idx_GET(src ), dst  );\
        pfence_point_count_SET( SRC##_count_GET(src ), dst  );\
        pfence_point_lat_SET( SRC##_lat_GET(src ), dst  );\
        pfence_point_lng_SET( SRC##_lng_GET(src ), dst  );\
    }

/**
*The autopilot is requesting a resource (file, binary, other type of data) */

typedef Pack RESOURCE_REQUEST_resource_request; //Raw pack. It contains a pointer to its Meta information and pack bytes. It can be acquired via new_channels_functions.

typedef uint8_t presource_request_RESOURCE_REQUEST;//Packet bytes.
/**
															* Acquire packet bytes
															*/
static inline presource_request_RESOURCE_REQUEST *presource_request_RESOURCE_REQUEST_from(RESOURCE_REQUEST_resource_request *pack) { return pack->bytes; }

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vresource_request_uri;
//Maximum field array length constant
#define Presource_request_uri_len  ( 120 )

/**
													* This struct hold information about none primitive field parameters and used by functions to access field data
													*/
typedef BytesArray Vresource_request_storage;
//Maximum field array length constant
#define Presource_request_storage_len  ( 120 )

/*MThe macro to simplifies writing code to export data from the pack.
* Macro is expecting and calls the following functions..
static inline void <DST>_request_id_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_uri_type_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_uri_SET( Vresource_request_uri * src, <DST> * dst  ){}
static inline void <DST>_transfer_type_SET( int8_t * src, <DST> * dst  ){}
static inline void <DST>_storage_SET( Vresource_request_storage * src, <DST> * dst  ){}
*/

#define presource_request_RESOURCE_REQUEST_PUSH_INTO(DST)\
    static inline void presource_request_RESOURCE_REQUEST_push_into_##DST ( presource_request_RESOURCE_REQUEST * src, DST * dst) {\
        DST##_request_id_SET( presource_request_request_id_GET( src  ), dst  );\
        DST##_uri_type_SET( presource_request_uri_type_GET( src  ), dst  );\
        Vresource_request_uri item_uri = presource_request_uri_GET( src  );\
       DST##_uri_SET( &item_uri, dst );\
        DST##_transfer_type_SET( presource_request_transfer_type_GET( src  ), dst  );\
        Vresource_request_storage item_storage = presource_request_storage_GET( src  );\
       DST##_storage_SET( &item_storage, dst );\
    }

/*The macro to simplify writing code to import data into the pack from external sources.
* Macro is expecting and calls the following functions..
static inline int8_t <SRC>_request_id_GET( <SRC> * src ){}
static inline int8_t <SRC>_uri_type_GET( <SRC> * src ){}
static inline int8_t <SRC>_uri_GET( <SRC> * src, Vresource_request_uri * dst ){}
static inline int8_t <SRC>_transfer_type_GET( <SRC> * src ){}
static inline int8_t <SRC>_storage_GET( <SRC> * src, Vresource_request_storage * dst ){}
*/

#define presource_request_RESOURCE_REQUEST_PULL_FROM(SRC)\
    static inline void presource_request_RESOURCE_REQUEST_pull_from_##SRC ( SRC * src, presource_request_RESOURCE_REQUEST * dst) {\
        presource_request_request_id_SET( SRC##_request_id_GET(src ), dst  );\
        presource_request_uri_type_SET( SRC##_uri_type_GET(src ), dst  );\
       Vresource_request_uri item_uri = presource_request_uri_SET( NULL, dst  );\
       SRC##_uri_GET( src, &item_uri );\
        presource_request_transfer_type_SET( SRC##_transfer_type_GET(src ), dst  );\
       Vresource_request_storage item_storage = presource_request_storage_SET( NULL, dst  );\
       SRC##_storage_GET( src, &item_storage );\
    }


#ifdef __cplusplus
}
#endif
									