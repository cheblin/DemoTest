
#include "MicroAirVehicle.h"
#include <assert.h>
#include <stddef.h>

static float   some_float   = 0;
static bool    some_bool    = true;
static int32_t some_int32_t = 0;
static size_t  some_size_t  = 0;
static char *some_string = "NULL";
static int8_t  some_int8_t  = 0;
static int64_t some_int64_t = 0;
static int16_t some_int16_t = 0;


void write_RESOURCE_REQUEST(presource_request_RESOURCE_REQUEST *const presource_request);

void read_RESOURCE_REQUEST(presource_request_RESOURCE_REQUEST *const presource_request);

void read_FENCE_POINT(pfence_point_FENCE_POINT *const pfence_point);

void write_RADIO_STATUS(pradio_status_RADIO_STATUS *const pradio_status);

void read_RADIO_STATUS(pradio_status_RADIO_STATUS *const pradio_status);

void read_RANGEFINDER(prangefinder_RANGEFINDER *const prangefinder);

void write_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *const pfile_transfer_protocol);

void read_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *const pfile_transfer_protocol);

void write_GLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *const pglobal_vision_position_estimate);

void read_GLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *const pglobal_vision_position_estimate);

void write_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA *const pgps_rtcm_data);

void read_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA *const pgps_rtcm_data);

void read_RPM(prpm_RPM *const prpm);

void write_NAMED_VALUE_INT(pnamed_value_int_NAMED_VALUE_INT *const pnamed_value_int);

void read_NAMED_VALUE_INT(pnamed_value_int_NAMED_VALUE_INT *const pnamed_value_int);

void write_ATTITUDE_QUATERNION_COV(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *const pattitude_quaternion_cov);

void read_ATTITUDE_QUATERNION_COV(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *const pattitude_quaternion_cov);

void read_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *const puavionix_adsb_transceiver_health_report);

void write_GPS_INJECT_DATA(pgps_inject_data_GPS_INJECT_DATA *const pgps_inject_data);

void read_GPS_INJECT_DATA(pgps_inject_data_GPS_INJECT_DATA *const pgps_inject_data);

void read_GOPRO_GET_RESPONSE(pgopro_get_response_GOPRO_GET_RESPONSE *const pgopro_get_response);

void read_UAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const puavionix_adsb_out_dynamic);

void write_EXTENDED_SYS_STATE(pextended_sys_state_EXTENDED_SYS_STATE *const pextended_sys_state);

void read_EXTENDED_SYS_STATE(pextended_sys_state_EXTENDED_SYS_STATE *const pextended_sys_state);

void write_HIGHRES_IMU(phighres_imu_HIGHRES_IMU *const phighres_imu);

void read_HIGHRES_IMU(phighres_imu_HIGHRES_IMU *const phighres_imu);

void read_MAG_CAL_PROGRESS(pmag_cal_progress_MAG_CAL_PROGRESS *const pmag_cal_progress);

void read_DEVICE_OP_READ(pdevice_op_read_DEVICE_OP_READ *const pdevice_op_read);

void read_LOGGING_DATA(plogging_data_LOGGING_DATA *const plogging_data);

void read_VISION_POSITION_DELTA(pvision_position_delta_VISION_POSITION_DELTA *const pvision_position_delta);

void write_MISSION_ITEM_INT(pmission_item_int_MISSION_ITEM_INT *const pmission_item_int);

void read_MISSION_ITEM_INT(pmission_item_int_MISSION_ITEM_INT *const pmission_item_int);

void write_OPTICAL_FLOW(poptical_flow_OPTICAL_FLOW *const poptical_flow);

void read_OPTICAL_FLOW(poptical_flow_OPTICAL_FLOW *const poptical_flow);

void write_COMMAND_INT(pcommand_int_COMMAND_INT *const pcommand_int);

void read_COMMAND_INT(pcommand_int_COMMAND_INT *const pcommand_int);

void write_RAW_IMU(praw_imu_RAW_IMU *const praw_imu);

void read_RAW_IMU(praw_imu_RAW_IMU *const praw_imu);

void write_MISSION_ITEM(pmission_item_MISSION_ITEM *const pmission_item);

void read_MISSION_ITEM(pmission_item_MISSION_ITEM *const pmission_item);

void write_SYS_STATUS(psys_status_SYS_STATUS *const psys_status);

void read_SYS_STATUS(psys_status_SYS_STATUS *const psys_status);

void read_GOPRO_SET_REQUEST(pgopro_set_request_GOPRO_SET_REQUEST *const pgopro_set_request);

void write_CHANGE_OPERATOR_CONTROL(pchange_operator_control_CHANGE_OPERATOR_CONTROL *const pchange_operator_control);

void read_CHANGE_OPERATOR_CONTROL(pchange_operator_control_CHANGE_OPERATOR_CONTROL *const pchange_operator_control);

void write_WIND_COV(pwind_cov_WIND_COV *const pwind_cov);

void read_WIND_COV(pwind_cov_WIND_COV *const pwind_cov);

void write_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2 *const pscaled_pressure2);

void read_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2 *const pscaled_pressure2);

void write_HIL_OPTICAL_FLOW(phil_optical_flow_HIL_OPTICAL_FLOW *const phil_optical_flow);

void read_HIL_OPTICAL_FLOW(phil_optical_flow_HIL_OPTICAL_FLOW *const phil_optical_flow);

void write_DISTANCE_SENSOR(pdistance_sensor_DISTANCE_SENSOR *const pdistance_sensor);

void read_DISTANCE_SENSOR(pdistance_sensor_DISTANCE_SENSOR *const pdistance_sensor);

void read_DEVICE_OP_WRITE(pdevice_op_write_DEVICE_OP_WRITE *const pdevice_op_write);

void read_GIMBAL_REPORT(pgimbal_report_GIMBAL_REPORT *const pgimbal_report);

void write_POSITION_TARGET_LOCAL_NED(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *const pposition_target_local_ned);

void read_POSITION_TARGET_LOCAL_NED(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *const pposition_target_local_ned);

void write_HIL_ACTUATOR_CONTROLS(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *const phil_actuator_controls);

void read_HIL_ACTUATOR_CONTROLS(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *const phil_actuator_controls);

void read_DATA64(pdata64_DATA64 *const pdata64);

void write_ATTITUDE_QUATERNION(pattitude_quaternion_ATTITUDE_QUATERNION *const pattitude_quaternion);

void read_ATTITUDE_QUATERNION(pattitude_quaternion_ATTITUDE_QUATERNION *const pattitude_quaternion);

void write_SCALED_IMU2(pscaled_imu2_SCALED_IMU2 *const pscaled_imu2);

void read_SCALED_IMU2(pscaled_imu2_SCALED_IMU2 *const pscaled_imu2);

void write_GPS_GLOBAL_ORIGIN(pgps_global_origin_GPS_GLOBAL_ORIGIN *const pgps_global_origin);

void read_GPS_GLOBAL_ORIGIN(pgps_global_origin_GPS_GLOBAL_ORIGIN *const pgps_global_origin);

void write_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *const pdata_transmission_handshake);

void read_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *const pdata_transmission_handshake);

void write_LOCAL_POSITION_NED(plocal_position_ned_LOCAL_POSITION_NED *const plocal_position_ned);

void read_LOCAL_POSITION_NED(plocal_position_ned_LOCAL_POSITION_NED *const plocal_position_ned);

void read_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *const pautopilot_version_request);

void read_WIND(pwind_WIND *const pwind);

void read_AP_ADC(pap_adc_AP_ADC *const pap_adc);

void read_SET_MAG_OFFSETS(pset_mag_offsets_SET_MAG_OFFSETS *const pset_mag_offsets);

void read_DATA16(pdata16_DATA16 *const pdata16);

void read_UAVCAN_NODE_INFO(puavcan_node_info_UAVCAN_NODE_INFO *const puavcan_node_info);

void read_PARAM_EXT_ACK(pparam_ext_ack_PARAM_EXT_ACK *const pparam_ext_ack);

void write_MISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *const pmission_request_partial_list);

void read_MISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *const pmission_request_partial_list);

void write_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3 *const pscaled_pressure3);

void read_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3 *const pscaled_pressure3);

void read_DIGICAM_CONFIGURE(pdigicam_configure_DIGICAM_CONFIGURE *const pdigicam_configure);

void write_PLAY_TUNE(pplay_tune_PLAY_TUNE *const pplay_tune);

void read_PLAY_TUNE(pplay_tune_PLAY_TUNE *const pplay_tune);

void read_SET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *const pset_video_stream_settings);

void read_SIMSTATE(psimstate_SIMSTATE *const psimstate);

void write_MISSION_REQUEST_LIST(pmission_request_list_MISSION_REQUEST_LIST *const pmission_request_list);

void read_MISSION_REQUEST_LIST(pmission_request_list_MISSION_REQUEST_LIST *const pmission_request_list);

void write_AUTOPILOT_VERSION(pautopilot_version_AUTOPILOT_VERSION *const pautopilot_version);

void read_AUTOPILOT_VERSION(pautopilot_version_AUTOPILOT_VERSION *const pautopilot_version);

void read_PARAM_EXT_SET(pparam_ext_set_PARAM_EXT_SET *const pparam_ext_set);

void write_SET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *const pset_gps_global_origin);

void read_SET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *const pset_gps_global_origin);

void read_MOUNT_ORIENTATION(pmount_orientation_MOUNT_ORIENTATION *const pmount_orientation);

void write_SET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *const pset_position_target_local_ned);

void read_SET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *const pset_position_target_local_ned);

void write_SERIAL_CONTROL(pserial_control_SERIAL_CONTROL *const pserial_control);

void read_SERIAL_CONTROL(pserial_control_SERIAL_CONTROL *const pserial_control);

void write_BATTERY_STATUS(pbattery_status_BATTERY_STATUS *const pbattery_status);

void read_BATTERY_STATUS(pbattery_status_BATTERY_STATUS *const pbattery_status);

void write_PARAM_VALUE(pparam_value_PARAM_VALUE *const pparam_value);

void read_PARAM_VALUE(pparam_value_PARAM_VALUE *const pparam_value);

void read_RALLY_FETCH_POINT(prally_fetch_point_RALLY_FETCH_POINT *const prally_fetch_point);

void read_PROTOCOL_VERSION(pprotocol_version_PROTOCOL_VERSION *const pprotocol_version);

void write_RC_CHANNELS(prc_channels_RC_CHANNELS *const prc_channels);

void read_RC_CHANNELS(prc_channels_RC_CHANNELS *const prc_channels);

void write_MANUAL_CONTROL(pmanual_control_MANUAL_CONTROL *const pmanual_control);

void read_MANUAL_CONTROL(pmanual_control_MANUAL_CONTROL *const pmanual_control);

void write_VISION_POSITION_ESTIMATE(pvision_position_estimate_VISION_POSITION_ESTIMATE *const pvision_position_estimate);

void read_VISION_POSITION_ESTIMATE(pvision_position_estimate_VISION_POSITION_ESTIMATE *const pvision_position_estimate);

void read_GOPRO_SET_RESPONSE(pgopro_set_response_GOPRO_SET_RESPONSE *const pgopro_set_response);

void write_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER *const pcamera_trigger);

void read_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER *const pcamera_trigger);

void write_SYSTEM_TIME(psystem_time_SYSTEM_TIME *const psystem_time);

void read_SYSTEM_TIME(psystem_time_SYSTEM_TIME *const psystem_time);

void write_MISSION_CURRENT(pmission_current_MISSION_CURRENT *const pmission_current);

void read_MISSION_CURRENT(pmission_current_MISSION_CURRENT *const pmission_current);

void write_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *const pchange_operator_control_ack);

void read_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *const pchange_operator_control_ack);

void write_MISSION_ACK(pmission_ack_MISSION_ACK *const pmission_ack);

void read_MISSION_ACK(pmission_ack_MISSION_ACK *const pmission_ack);

void write_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END *const plog_request_end);

void read_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END *const plog_request_end);

void write_DEBUG_VECT(pdebug_vect_DEBUG_VECT *const pdebug_vect);

void read_DEBUG_VECT(pdebug_vect_DEBUG_VECT *const pdebug_vect);

void write_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE *const pvision_speed_estimate);

void read_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE *const pvision_speed_estimate);

void read_LOGGING_ACK(plogging_ack_LOGGING_ACK *const plogging_ack);

void write_MISSION_ITEM_REACHED(pmission_item_reached_MISSION_ITEM_REACHED *const pmission_item_reached);

void read_MISSION_ITEM_REACHED(pmission_item_reached_MISSION_ITEM_REACHED *const pmission_item_reached);

void read_MEMINFO(pmeminfo_MEMINFO *const pmeminfo);

void write_SERVO_OUTPUT_RAW(pservo_output_raw_SERVO_OUTPUT_RAW *const pservo_output_raw);

void read_SERVO_OUTPUT_RAW(pservo_output_raw_SERVO_OUTPUT_RAW *const pservo_output_raw);

void write_RC_CHANNELS_RAW(prc_channels_raw_RC_CHANNELS_RAW *const prc_channels_raw);

void read_RC_CHANNELS_RAW(prc_channels_raw_RC_CHANNELS_RAW *const prc_channels_raw);

void write_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION *const pflight_information);

void read_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION *const pflight_information);

void read_DATA96(pdata96_DATA96 *const pdata96);

void read_WIFI_CONFIG_AP(pwifi_config_ap_WIFI_CONFIG_AP *const pwifi_config_ap);

void write_SIM_STATE(psim_state_SIM_STATE *const psim_state);

void read_SIM_STATE(psim_state_SIM_STATE *const psim_state);

void read_LED_CONTROL(pled_control_LED_CONTROL *const pled_control);

void write_POSITION_TARGET_GLOBAL_INT(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *const pposition_target_global_int);

void read_POSITION_TARGET_GLOBAL_INT(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *const pposition_target_global_int);

void read_MOUNT_CONTROL(pmount_control_MOUNT_CONTROL *const pmount_control);

void write_SET_MODE(pset_mode_SET_MODE *const pset_mode);

void read_SET_MODE(pset_mode_SET_MODE *const pset_mode);

void write_SCALED_IMU3(pscaled_imu3_SCALED_IMU3 *const pscaled_imu3);

void read_SCALED_IMU3(pscaled_imu3_SCALED_IMU3 *const pscaled_imu3);

void write_HIL_RC_INPUTS_RAW(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *const phil_rc_inputs_raw);

void read_HIL_RC_INPUTS_RAW(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *const phil_rc_inputs_raw);

void write_SET_HOME_POSITION(pset_home_position_SET_HOME_POSITION *const pset_home_position);

void read_SET_HOME_POSITION(pset_home_position_SET_HOME_POSITION *const pset_home_position);

void write_TERRAIN_REPORT(pterrain_report_TERRAIN_REPORT *const pterrain_report);

void read_TERRAIN_REPORT(pterrain_report_TERRAIN_REPORT *const pterrain_report);

void write_MISSION_REQUEST(pmission_request_MISSION_REQUEST *const pmission_request);

void read_MISSION_REQUEST(pmission_request_MISSION_REQUEST *const pmission_request);

void write_DATA_STREAM(pdata_stream_DATA_STREAM *const pdata_stream);

void read_DATA_STREAM(pdata_stream_DATA_STREAM *const pdata_stream);

void write_COMMAND_ACK(pcommand_ack_COMMAND_ACK *const pcommand_ack);

void read_COMMAND_ACK(pcommand_ack_COMMAND_ACK *const pcommand_ack);

void write_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *const plocal_position_ned_system_global_offset);

void read_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *const plocal_position_ned_system_global_offset);

void write_MISSION_REQUEST_INT(pmission_request_int_MISSION_REQUEST_INT *const pmission_request_int);

void read_MISSION_REQUEST_INT(pmission_request_int_MISSION_REQUEST_INT *const pmission_request_int);

void read_MOUNT_CONFIGURE(pmount_configure_MOUNT_CONFIGURE *const pmount_configure);

void write_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK *const pterrain_check);

void read_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK *const pterrain_check);

void read_LOGGING_DATA_ACKED(plogging_data_acked_LOGGING_DATA_ACKED *const plogging_data_acked);

void read_REMOTE_LOG_DATA_BLOCK(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *const premote_log_data_block);

void write_POWER_STATUS(ppower_status_POWER_STATUS *const ppower_status);

void read_POWER_STATUS(ppower_status_POWER_STATUS *const ppower_status);

void write_PARAM_MAP_RC(pparam_map_rc_PARAM_MAP_RC *const pparam_map_rc);

void read_PARAM_MAP_RC(pparam_map_rc_PARAM_MAP_RC *const pparam_map_rc);

void write_HEARTBEAT(pheartbeat_HEARTBEAT *const pheartbeat);

void read_HEARTBEAT(pheartbeat_HEARTBEAT *const pheartbeat);

void write_V2_EXTENSION(pv2_extension_V2_EXTENSION *const pv2_extension);

void read_V2_EXTENSION(pv2_extension_V2_EXTENSION *const pv2_extension);

void write_SCALED_PRESSURE(pscaled_pressure_SCALED_PRESSURE *const pscaled_pressure);

void read_SCALED_PRESSURE(pscaled_pressure_SCALED_PRESSURE *const pscaled_pressure);

void write_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST *const plog_request_list);

void read_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST *const plog_request_list);

void read_MAG_CAL_REPORT(pmag_cal_report_MAG_CAL_REPORT *const pmag_cal_report);

void write_GPS2_RTK(pgps2_rtk_GPS2_RTK *const pgps2_rtk);

void read_GPS2_RTK(pgps2_rtk_GPS2_RTK *const pgps2_rtk);

void write_VICON_POSITION_ESTIMATE(pvicon_position_estimate_VICON_POSITION_ESTIMATE *const pvicon_position_estimate);

void read_VICON_POSITION_ESTIMATE(pvicon_position_estimate_VICON_POSITION_ESTIMATE *const pvicon_position_estimate);

void read_AHRS3(pahrs3_AHRS3 *const pahrs3);

void write_MISSION_CLEAR_ALL(pmission_clear_all_MISSION_CLEAR_ALL *const pmission_clear_all);

void read_MISSION_CLEAR_ALL(pmission_clear_all_MISSION_CLEAR_ALL *const pmission_clear_all);

void write_LOG_DATA(plog_data_LOG_DATA *const plog_data);

void read_LOG_DATA(plog_data_LOG_DATA *const plog_data);

void write_OPTICAL_FLOW_RAD(poptical_flow_rad_OPTICAL_FLOW_RAD *const poptical_flow_rad);

void read_OPTICAL_FLOW_RAD(poptical_flow_rad_OPTICAL_FLOW_RAD *const poptical_flow_rad);

void write_SAFETY_ALLOWED_AREA(psafety_allowed_area_SAFETY_ALLOWED_AREA *const psafety_allowed_area);

void read_SAFETY_ALLOWED_AREA(psafety_allowed_area_SAFETY_ALLOWED_AREA *const psafety_allowed_area);

void read_PID_TUNING(ppid_tuning_PID_TUNING *const ppid_tuning);

void write_MANUAL_SETPOINT(pmanual_setpoint_MANUAL_SETPOINT *const pmanual_setpoint);

void read_MANUAL_SETPOINT(pmanual_setpoint_MANUAL_SETPOINT *const pmanual_setpoint);

void read_MOUNT_STATUS(pmount_status_MOUNT_STATUS *const pmount_status);

void write_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST *const pterrain_request);

void read_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST *const pterrain_request);

void write_LOG_ERASE(plog_erase_LOG_ERASE *const plog_erase);

void read_LOG_ERASE(plog_erase_LOG_ERASE *const plog_erase);

void read_AHRS2(pahrs2_AHRS2 *const pahrs2);

void write_MISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *const pmission_write_partial_list);

void read_MISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *const pmission_write_partial_list);

void write_ATTITUDE(pattitude_ATTITUDE *const pattitude);

void read_ATTITUDE(pattitude_ATTITUDE *const pattitude);

void read_GOPRO_HEARTBEAT(pgopro_heartbeat_GOPRO_HEARTBEAT *const pgopro_heartbeat);

void write_NAMED_VALUE_FLOAT(pnamed_value_float_NAMED_VALUE_FLOAT *const pnamed_value_float);

void read_NAMED_VALUE_FLOAT(pnamed_value_float_NAMED_VALUE_FLOAT *const pnamed_value_float);

void read_DIGICAM_CONTROL(pdigicam_control_DIGICAM_CONTROL *const pdigicam_control);

void write_RAW_PRESSURE(praw_pressure_RAW_PRESSURE *const praw_pressure);

void read_RAW_PRESSURE(praw_pressure_RAW_PRESSURE *const praw_pressure);

void read_DEVICE_OP_READ_REPLY(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *const pdevice_op_read_reply);

void write_CAMERA_SETTINGS(pcamera_settings_CAMERA_SETTINGS *const pcamera_settings);

void read_CAMERA_SETTINGS(pcamera_settings_CAMERA_SETTINGS *const pcamera_settings);

void write_RC_CHANNELS_SCALED(prc_channels_scaled_RC_CHANNELS_SCALED *const prc_channels_scaled);

void read_RC_CHANNELS_SCALED(prc_channels_scaled_RC_CHANNELS_SCALED *const prc_channels_scaled);

void read_CAMERA_STATUS(pcamera_status_CAMERA_STATUS *const pcamera_status);

void write_GPS_RAW_INT(pgps_raw_int_GPS_RAW_INT *const pgps_raw_int);

void read_GPS_RAW_INT(pgps_raw_int_GPS_RAW_INT *const pgps_raw_int);

void write_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA *const plog_request_data);

void read_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA *const plog_request_data);

void read_COMPASSMOT_STATUS(pcompassmot_status_COMPASSMOT_STATUS *const pcompassmot_status);

void write_COMMAND_LONG(pcommand_long_COMMAND_LONG *const pcommand_long);

void read_COMMAND_LONG(pcommand_long_COMMAND_LONG *const pcommand_long);

void write_GPS_INPUT(pgps_input_GPS_INPUT *const pgps_input);

void read_GPS_INPUT(pgps_input_GPS_INPUT *const pgps_input);

void write_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA *const pencapsulated_data);

void read_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA *const pencapsulated_data);

void write_GLOBAL_POSITION_INT(pglobal_position_int_GLOBAL_POSITION_INT *const pglobal_position_int);

void read_GLOBAL_POSITION_INT(pglobal_position_int_GLOBAL_POSITION_INT *const pglobal_position_int);

void write_CAMERA_CAPTURE_STATUS(pcamera_capture_status_CAMERA_CAPTURE_STATUS *const pcamera_capture_status);

void read_CAMERA_CAPTURE_STATUS(pcamera_capture_status_CAMERA_CAPTURE_STATUS *const pcamera_capture_status);

void read_GOPRO_GET_REQUEST(pgopro_get_request_GOPRO_GET_REQUEST *const pgopro_get_request);

void write_PING(pping_PING *const pping);

void read_PING(pping_PING *const pping);

void write_STATUSTEXT(pstatustext_STATUSTEXT *const pstatustext);

void read_STATUSTEXT(pstatustext_STATUSTEXT *const pstatustext);

void write_ATT_POS_MOCAP(patt_pos_mocap_ATT_POS_MOCAP *const patt_pos_mocap);

void read_ATT_POS_MOCAP(patt_pos_mocap_ATT_POS_MOCAP *const patt_pos_mocap);

void read_AIRSPEED_AUTOCAL(pairspeed_autocal_AIRSPEED_AUTOCAL *const pairspeed_autocal);

void write_LOCAL_POSITION_NED_COV(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *const plocal_position_ned_cov);

void read_LOCAL_POSITION_NED_COV(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *const plocal_position_ned_cov);

void read_RADIO(pradio_RADIO *const pradio);

void read_FENCE_FETCH_POINT(pfence_fetch_point_FENCE_FETCH_POINT *const pfence_fetch_point);

void write_AUTH_KEY(pauth_key_AUTH_KEY *const pauth_key);

void read_AUTH_KEY(pauth_key_AUTH_KEY *const pauth_key);

void write_NAV_CONTROLLER_OUTPUT(pnav_controller_output_NAV_CONTROLLER_OUTPUT *const pnav_controller_output);

void read_NAV_CONTROLLER_OUTPUT(pnav_controller_output_NAV_CONTROLLER_OUTPUT *const pnav_controller_output);

void write_HIL_GPS(phil_gps_HIL_GPS *const phil_gps);

void read_HIL_GPS(phil_gps_HIL_GPS *const phil_gps);

void read_CAMERA_FEEDBACK(pcamera_feedback_CAMERA_FEEDBACK *const pcamera_feedback);

void read_LIMITS_STATUS(plimits_status_LIMITS_STATUS *const plimits_status);

void read_BATTERY2(pbattery2_BATTERY2 *const pbattery2);

void read_PARAM_EXT_VALUE(pparam_ext_value_PARAM_EXT_VALUE *const pparam_ext_value);

void write_VIBRATION(pvibration_VIBRATION *const pvibration);

void read_VIBRATION(pvibration_VIBRATION *const pvibration);

void read_ADAP_TUNING(padap_tuning_ADAP_TUNING *const padap_tuning);

void write_MISSION_SET_CURRENT(pmission_set_current_MISSION_SET_CURRENT *const pmission_set_current);

void read_MISSION_SET_CURRENT(pmission_set_current_MISSION_SET_CURRENT *const pmission_set_current);

void read_RALLY_POINT(prally_point_RALLY_POINT *const prally_point);

void write_VFR_HUD(pvfr_hud_VFR_HUD *const pvfr_hud);

void read_VFR_HUD(pvfr_hud_VFR_HUD *const pvfr_hud);

void write_PING33(pping33_PING33 *const pping33);

void read_PING33(pping33_PING33 *const pping33);

void read_DATA32(pdata32_DATA32 *const pdata32);

void write_SET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *const pset_position_target_global_int);

void read_SET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *const pset_position_target_global_int);

void write_CONTROL_SYSTEM_STATE(pcontrol_system_state_CONTROL_SYSTEM_STATE *const pcontrol_system_state);

void read_CONTROL_SYSTEM_STATE(pcontrol_system_state_CONTROL_SYSTEM_STATE *const pcontrol_system_state);

void write_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *const pset_actuator_control_target);

void read_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *const pset_actuator_control_target);

void write_LANDING_TARGET(planding_target_LANDING_TARGET *const planding_target);

void read_LANDING_TARGET(planding_target_LANDING_TARGET *const planding_target);

void read_UAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const puavionix_adsb_out_cfg);

void write_PARAM_REQUEST_LIST(pparam_request_list_PARAM_REQUEST_LIST *const pparam_request_list);

void read_PARAM_REQUEST_LIST(pparam_request_list_PARAM_REQUEST_LIST *const pparam_request_list);

void write_GPS_RTK(pgps_rtk_GPS_RTK *const pgps_rtk);

void read_GPS_RTK(pgps_rtk_GPS_RTK *const pgps_rtk);

void write_SETUP_SIGNING(psetup_signing_SETUP_SIGNING *const psetup_signing);

void read_SETUP_SIGNING(psetup_signing_SETUP_SIGNING *const psetup_signing);

void write_HIL_SENSOR(phil_sensor_HIL_SENSOR *const phil_sensor);

void read_HIL_SENSOR(phil_sensor_HIL_SENSOR *const phil_sensor);

void write_HIL_CONTROLS(phil_controls_HIL_CONTROLS *const phil_controls);

void read_HIL_CONTROLS(phil_controls_HIL_CONTROLS *const phil_controls);

void read_PARAM_EXT_REQUEST_READ(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *const pparam_ext_request_read);

void write_MEMORY_VECT(pmemory_vect_MEMORY_VECT *const pmemory_vect);

void read_MEMORY_VECT(pmemory_vect_MEMORY_VECT *const pmemory_vect);

void write_REQUEST_DATA_STREAM(prequest_data_stream_REQUEST_DATA_STREAM *const prequest_data_stream);

void read_REQUEST_DATA_STREAM(prequest_data_stream_REQUEST_DATA_STREAM *const prequest_data_stream);

void write_GPS2_RAW(pgps2_raw_GPS2_RAW *const pgps2_raw);

void read_GPS2_RAW(pgps2_raw_GPS2_RAW *const pgps2_raw);

void read_OBSTACLE_DISTANCE(pobstacle_distance_OBSTACLE_DISTANCE *const pobstacle_distance);

void read_REMOTE_LOG_BLOCK_STATUS(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *const premote_log_block_status);

void read_FENCE_STATUS(pfence_status_FENCE_STATUS *const pfence_status);

void write_HOME_POSITION(phome_position_HOME_POSITION *const phome_position);

void read_HOME_POSITION(phome_position_HOME_POSITION *const phome_position);

void write_HIL_STATE(phil_state_HIL_STATE *const phil_state);

void read_HIL_STATE(phil_state_HIL_STATE *const phil_state);

void write_FOLLOW_TARGET(pfollow_target_FOLLOW_TARGET *const pfollow_target);

void read_FOLLOW_TARGET(pfollow_target_FOLLOW_TARGET *const pfollow_target);

void write_SET_ATTITUDE_TARGET(pset_attitude_target_SET_ATTITUDE_TARGET *const pset_attitude_target);

void read_SET_ATTITUDE_TARGET(pset_attitude_target_SET_ATTITUDE_TARGET *const pset_attitude_target);

void write_PARAM_REQUEST_READ(pparam_request_read_PARAM_REQUEST_READ *const pparam_request_read);

void read_PARAM_REQUEST_READ(pparam_request_read_PARAM_REQUEST_READ *const pparam_request_read);

void write_HIGH_LATENCY(phigh_latency_HIGH_LATENCY *const phigh_latency);

void read_HIGH_LATENCY(phigh_latency_HIGH_LATENCY *const phigh_latency);

void write_ACTUATOR_CONTROL_TARGET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *const pactuator_control_target);

void read_ACTUATOR_CONTROL_TARGET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *const pactuator_control_target);

void write_LOG_ENTRY(plog_entry_LOG_ENTRY *const plog_entry);

void read_LOG_ENTRY(plog_entry_LOG_ENTRY *const plog_entry);

void write_CAMERA_IMAGE_CAPTURED(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *const pcamera_image_captured);

void read_CAMERA_IMAGE_CAPTURED(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *const pcamera_image_captured);

void write_DEBUG(pdebug_DEBUG *const pdebug);

void read_DEBUG(pdebug_DEBUG *const pdebug);

void read_AHRS(pahrs_AHRS *const pahrs);

void read_VIDEO_STREAM_INFORMATION(pvideo_stream_information_VIDEO_STREAM_INFORMATION *const pvideo_stream_information);

void write_SCALED_IMU(pscaled_imu_SCALED_IMU *const pscaled_imu);

void read_SCALED_IMU(pscaled_imu_SCALED_IMU *const pscaled_imu);

void write_RC_CHANNELS_OVERRIDE(prc_channels_override_RC_CHANNELS_OVERRIDE *const prc_channels_override);

void read_RC_CHANNELS_OVERRIDE(prc_channels_override_RC_CHANNELS_OVERRIDE *const prc_channels_override);

void read_GIMBAL_CONTROL(pgimbal_control_GIMBAL_CONTROL *const pgimbal_control);

void write_TERRAIN_DATA(pterrain_data_TERRAIN_DATA *const pterrain_data);

void read_TERRAIN_DATA(pterrain_data_TERRAIN_DATA *const pterrain_data);

void write_PARAM_SET(pparam_set_PARAM_SET *const pparam_set);

void read_PARAM_SET(pparam_set_PARAM_SET *const pparam_set);

void read_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *const pdevice_op_write_reply);

void write_GPS_STATUS(pgps_status_GPS_STATUS *const pgps_status);

void read_GPS_STATUS(pgps_status_GPS_STATUS *const pgps_status);

void write_CAMERA_INFORMATION(pcamera_information_CAMERA_INFORMATION *const pcamera_information);

void read_CAMERA_INFORMATION(pcamera_information_CAMERA_INFORMATION *const pcamera_information);

void write_STORAGE_INFORMATION(pstorage_information_STORAGE_INFORMATION *const pstorage_information);

void read_STORAGE_INFORMATION(pstorage_information_STORAGE_INFORMATION *const pstorage_information);

void read_SENSOR_OFFSETS(psensor_offsets_SENSOR_OFFSETS *const psensor_offsets);

void write_HIL_STATE_QUATERNION(phil_state_quaternion_HIL_STATE_QUATERNION *const phil_state_quaternion);

void read_HIL_STATE_QUATERNION(phil_state_quaternion_HIL_STATE_QUATERNION *const phil_state_quaternion);

void write_ALTITUDE(paltitude_ALTITUDE *const paltitude);

void read_ALTITUDE(paltitude_ALTITUDE *const paltitude);

void read_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *const pgimbal_torque_cmd_report);

void write_COLLISION(pcollision_COLLISION *const pcollision);

void read_COLLISION(pcollision_COLLISION *const pcollision);

void read_UAVCAN_NODE_STATUS(puavcan_node_status_UAVCAN_NODE_STATUS *const puavcan_node_status);

void write_SAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *const psafety_set_allowed_area);

void read_SAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *const psafety_set_allowed_area);

void write_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE *const pbutton_change);

void read_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE *const pbutton_change);

void write_GLOBAL_POSITION_INT_COV(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *const pglobal_position_int_cov);

void read_GLOBAL_POSITION_INT_COV(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *const pglobal_position_int_cov);

void read_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *const pparam_ext_request_list);

void write_TIMESYNC(ptimesync_TIMESYNC *const ptimesync);

void read_TIMESYNC(ptimesync_TIMESYNC *const ptimesync);

void read_HWSTATUS(phwstatus_HWSTATUS *const phwstatus);

void write_ESTIMATOR_STATUS(pestimator_status_ESTIMATOR_STATUS *const pestimator_status);

void read_ESTIMATOR_STATUS(pestimator_status_ESTIMATOR_STATUS *const pestimator_status);

void read_EKF_STATUS_REPORT(pekf_status_report_EKF_STATUS_REPORT *const pekf_status_report);

void write_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL *const pmessage_interval);

void read_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL *const pmessage_interval);

void write_ADSB_VEHICLE(padsb_vehicle_ADSB_VEHICLE *const padsb_vehicle);

void read_ADSB_VEHICLE(padsb_vehicle_ADSB_VEHICLE *const padsb_vehicle);

void write_MISSION_COUNT(pmission_count_MISSION_COUNT *const pmission_count);

void read_MISSION_COUNT(pmission_count_MISSION_COUNT *const pmission_count);

void write_ATTITUDE_TARGET(pattitude_target_ATTITUDE_TARGET *const pattitude_target);

void read_ATTITUDE_TARGET(pattitude_target_ATTITUDE_TARGET *const pattitude_target);


void read_ATTITUDE_TARGET(pattitude_target_ATTITUDE_TARGET *const pattitude_target) {
	int32_t            time_boot_ms = pattitude_target_time_boot_ms_GET(pattitude_target);
	int8_t             type_mask    = pattitude_target_type_mask_GET(pattitude_target);
	Vattitude_target_q item_q       = pattitude_target_q_GET(pattitude_target);
	for (size_t        index        = 0; index < item_q.len; index++)
		some_float                     = vattitude_target_q_GET(&item_q, index);
	float              body_roll_rate  = pattitude_target_body_roll_rate_GET(pattitude_target);
	float              body_pitch_rate = pattitude_target_body_pitch_rate_GET(pattitude_target);
	float              body_yaw_rate   = pattitude_target_body_yaw_rate_GET(pattitude_target);
	float              thrust          = pattitude_target_thrust_GET(pattitude_target);
	
}


void write_ATTITUDE_TARGET(pattitude_target_ATTITUDE_TARGET *const pattitude_target) {
	pattitude_target_time_boot_ms_SET(1154011954, -1960390265, 274147270, 71482463, 1504795223, -1927044430, -1505827449, -1683690605, -270695088, 158272404, pattitude_target);
	pattitude_target_type_mask_SET(-35, 58, 25, 127, -3, 18, -45, -84, 65, 33, pattitude_target);
	pattitude_target_q_SET(&-1.5730887E38F, 1.8079428E38F, 1.5456385E38F, 2.5958673E38F, 1.1436452E38F, -3.2895558E37F, -2.2592028E38F, 1.069779E38F, -2.5680682E38F, -3.268332E38F, pattitude_target);
	pattitude_target_body_roll_rate_SET(-3.2253548E38F, 1.1920448E38F, -3.2440846E38F, -1.7274021E38F, 2.1394721E38F, -3.0135156E38F, -3.3230895E38F, 2.4068318E38F, -1.200642E38F, -1.2193555E38F, pattitude_target);
	pattitude_target_body_pitch_rate_SET(2.9045336E37F, -2.173998E38F, -2.5022683E37F, 2.9997584E38F, 7.494058E37F, 3.2806925E38F, 1.32575836E36F, -2.2411639E38F, 2.9058416E38F, -9.756994E37F, pattitude_target);
	pattitude_target_body_yaw_rate_SET(-2.260872E38F, 1.7031947E38F, -3.21064E38F, -3.3245946E38F, -1.5184947E38F, 2.383768E38F, 2.0105158E38F, 1.5644985E38F, 2.6184351E38F, 1.1369778E38F, pattitude_target);
	pattitude_target_thrust_SET(-2.4401298E37F, 2.556494E38F, 3.332568E38F, -1.0983155E38F, 1.6044257E38F, 4.008435E37F, -1.2589787E38F, 1.7312432E38F, 7.2752694E37F, -1.2319902E37F, pattitude_target);
	
}


void read_MISSION_COUNT(pmission_count_MISSION_COUNT *const pmission_count) {
	int16_t            count            = pmission_count_count_GET(pmission_count);
	int8_t             target_system    = pmission_count_target_system_GET(pmission_count);
	int8_t             target_component = pmission_count_target_component_GET(pmission_count);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_count_mission_type_GET(pmission_count, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_COUNT(pmission_count_MISSION_COUNT *const pmission_count) {
	pmission_count_count_SET(-30987, 17539, -22181, -29793, -25816, 18401, -11986, 18201, -25396, 16916, pmission_count);
	pmission_count_target_system_SET(99, 9, 26, -65, 39, -73, 80, 0, 56, 96, pmission_count);
	pmission_count_target_component_SET(29, 28, -107, 2, 78, 12, -103, -67, -97, -27, pmission_count);
	pmission_count_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, pmission_count);
	
}


void read_ADSB_VEHICLE(padsb_vehicle_ADSB_VEHICLE *const padsb_vehicle) {
	int16_t              heading      = padsb_vehicle_heading_GET(padsb_vehicle);
	int16_t              hor_velocity = padsb_vehicle_hor_velocity_GET(padsb_vehicle);
	int16_t              squawk       = padsb_vehicle_squawk_GET(padsb_vehicle);
	int32_t              ICAO_address = padsb_vehicle_ICAO_address_GET(padsb_vehicle);
	int32_t              lat          = padsb_vehicle_lat_GET(padsb_vehicle);
	int32_t              lon          = padsb_vehicle_lon_GET(padsb_vehicle);
	int32_t              altitude     = padsb_vehicle_altitude_GET(padsb_vehicle);
	int16_t              ver_velocity = padsb_vehicle_ver_velocity_GET(padsb_vehicle);
	int8_t               tslc         = padsb_vehicle_tslc_GET(padsb_vehicle);
	e_ADSB_ALTITUDE_TYPE item_altitude_type;
	if (padsb_vehicle_altitude_type_GET(padsb_vehicle, &item_altitude_type)) {
		e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH = item_altitude_type;
	}
	Vadsb_vehicle_callsign item_callsign;
	if (padsb_vehicle_callsign_GET(padsb_vehicle, &item_callsign)) {
		memcpy(some_string, item_callsign.bytes, item_callsign.len);
	}
	e_ADSB_EMITTER_TYPE item_emitter_type;
	if (padsb_vehicle_emitter_type_GET(padsb_vehicle, &item_emitter_type)) {
		e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO = item_emitter_type;
	}
	e_ADSB_FLAGS item_flags;
	if (padsb_vehicle_flags_GET(padsb_vehicle, &item_flags)) {
		e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS = item_flags;
	}
	
}


void write_ADSB_VEHICLE(padsb_vehicle_ADSB_VEHICLE *const padsb_vehicle) {
	padsb_vehicle_heading_SET(-23802, -30829, -27350, 11726, -8317, -6318, -15551, -28057, 15106, -1854, padsb_vehicle);
	padsb_vehicle_hor_velocity_SET(-23898, 17349, -13239, -30389, 11653, 24000, -7036, 28212, 29450, -2268, padsb_vehicle);
	padsb_vehicle_squawk_SET(19021, 8220, -2159, 29172, 28233, 22733, 27028, 6169, 28751, 31252, padsb_vehicle);
	padsb_vehicle_ICAO_address_SET(1045287946, -655410115, -2129564478, -998591893, 1127181962, -982582034, -194677453, -332332802, -794807218, 2089404663, padsb_vehicle);
	padsb_vehicle_lat_SET(302610010, 883231588, -521790105, 884126682, 699610206, 1043852912, -1620923989, -1083224494, 1365525742, 131006253, padsb_vehicle);
	padsb_vehicle_lon_SET(1404122843, 625197488, -1095502917, -1377778078, -2126787082, 66138355, -1124491781, -541101910, -590617461, 1075991721, padsb_vehicle);
	padsb_vehicle_altitude_SET(351183102, 374074832, 1685191047, 755523422, -1031711982, -1865836961, -1713798859, -755624367, -1093687556, 1277640193, padsb_vehicle);
	padsb_vehicle_ver_velocity_SET(19276, -9296, 25762, -29011, -19959, -17866, 25691, -20799, 6381, 31795, padsb_vehicle);
	padsb_vehicle_tslc_SET(31, -101, 97, 45, 84, 66, 125, -102, -75, 76, padsb_vehicle);
	padsb_vehicle_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, padsb_vehicle);
	padsb_vehicle_callsign_SET(some_string, strlen(some_string), padsb_vehicle);
	padsb_vehicle_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED2, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, padsb_vehicle);
	padsb_vehicle_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED, e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY, e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED, e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS, e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING, e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING, e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE, e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, padsb_vehicle);
	
}


void read_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL *const pmessage_interval) {
	int16_t message_id  = pmessage_interval_message_id_GET(pmessage_interval);
	int32_t interval_us = pmessage_interval_interval_us_GET(pmessage_interval);
	
}


void write_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL *const pmessage_interval) {
	pmessage_interval_message_id_SET(-6329, -18881, 30876, 28873, -568, -30618, 9940, -29534, -7686, 5511, pmessage_interval);
	pmessage_interval_interval_us_SET(1988642749, 2131463053, 1060675642, -91364353, 972035027, 817043181, 188134052, -174183394, 1548233022, 1253436214, pmessage_interval);
	
}


void read_EKF_STATUS_REPORT(pekf_status_report_EKF_STATUS_REPORT *const pekf_status_report) {
	float              velocity_variance    = pekf_status_report_velocity_variance_GET(pekf_status_report);
	float              pos_horiz_variance   = pekf_status_report_pos_horiz_variance_GET(pekf_status_report);
	float              pos_vert_variance    = pekf_status_report_pos_vert_variance_GET(pekf_status_report);
	float              compass_variance     = pekf_status_report_compass_variance_GET(pekf_status_report);
	float              terrain_alt_variance = pekf_status_report_terrain_alt_variance_GET(pekf_status_report);
	e_EKF_STATUS_FLAGS item_flags;
	if (pekf_status_report_flags_GET(pekf_status_report, &item_flags)) {
		e_EKF_STATUS_FLAGS_EKF_ATTITUDE = item_flags;
	}
	
}


void read_ESTIMATOR_STATUS(pestimator_status_ESTIMATOR_STATUS *const pestimator_status) {
	int64_t                  time_usec          = pestimator_status_time_usec_GET(pestimator_status);
	float                    vel_ratio          = pestimator_status_vel_ratio_GET(pestimator_status);
	float                    pos_horiz_ratio    = pestimator_status_pos_horiz_ratio_GET(pestimator_status);
	float                    pos_vert_ratio     = pestimator_status_pos_vert_ratio_GET(pestimator_status);
	float                    mag_ratio          = pestimator_status_mag_ratio_GET(pestimator_status);
	float                    hagl_ratio         = pestimator_status_hagl_ratio_GET(pestimator_status);
	float                    tas_ratio          = pestimator_status_tas_ratio_GET(pestimator_status);
	float                    pos_horiz_accuracy = pestimator_status_pos_horiz_accuracy_GET(pestimator_status);
	float                    pos_vert_accuracy  = pestimator_status_pos_vert_accuracy_GET(pestimator_status);
	e_ESTIMATOR_STATUS_FLAGS item_flags;
	if (pestimator_status_flags_GET(pestimator_status, &item_flags)) {
		e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE = item_flags;
	}
	
}


void write_ESTIMATOR_STATUS(pestimator_status_ESTIMATOR_STATUS *const pestimator_status) {
	pestimator_status_time_usec_SET(3769516319382738357L, 4560456279370833946L, 4762630362300266695L, -5934642217390292903L, -8434154325037789822L, -4624046737976523945L, -6652171039923671203L, 6767153681864564203L, 4137845466694277393L, -7721038213680522371L, pestimator_status);
	pestimator_status_vel_ratio_SET(-9.634645E37F, -2.9394836E38F, -1.8353228E38F, -3.2374054E38F, 9.435774E37F, 9.2335E37F, -2.395913E38F, 2.6111363E38F, -2.3241268E38F, 1.875656E38F, pestimator_status);
	pestimator_status_pos_horiz_ratio_SET(-1.0229791E38F, 2.6562614E38F, -3.2972736E38F, 2.2564093E38F, -9.533722E37F, 1.3175293E38F, 5.17049E37F, 2.929563E37F, -2.9374517E38F, -3.0126524E38F, pestimator_status);
	pestimator_status_pos_vert_ratio_SET(3.0895971E38F, 2.9303827E38F, 1.7983123E38F, -1.5999594E38F, 3.3000462E38F, 1.1144955E38F, 2.8262797E38F, -6.211105E37F, 3.1822577E38F, -2.0081044E38F, pestimator_status);
	pestimator_status_mag_ratio_SET(3.2657438E38F, 3.1277918E38F, 1.72825E38F, 9.364074E37F, 2.2581228E38F, -2.769473E38F, -3.8950819E37F, -2.6588657E38F, -6.229915E36F, -2.789633E38F, pestimator_status);
	pestimator_status_hagl_ratio_SET(-8.520145E37F, -6.993642E37F, -2.2169801E38F, 5.2304704E37F, 1.3629479E38F, -2.4724107E38F, -2.7030878E38F, 3.265254E38F, -1.2077657E38F, 1.7078608E38F, pestimator_status);
	pestimator_status_tas_ratio_SET(-9.890002E36F, -3.0821959E38F, 2.2908493E38F, 1.5644447E38F, 1.1078305E37F, 2.1036516E38F, 1.7343372E37F, 1.9748783E38F, -8.251979E36F, 1.0009724E38F, pestimator_status);
	pestimator_status_pos_horiz_accuracy_SET(3.0087924E38F, -1.5979999E38F, -2.3251178E38F, -2.9206777E38F, 1.2029917E38F, 2.0169733E38F, 2.789363E38F, -1.2752266E38F, 7.927653E37F, 1.1865543E38F, pestimator_status);
	pestimator_status_pos_vert_accuracy_SET(-3.0891607E38F, 1.4632126E38F, 2.9824334E38F, -1.9091848E38F, 2.0608498E38F, 9.877422E37F, 4.7170886E36F, -2.3581893E38F, 2.9727177E38F, -2.8501777E38F, pestimator_status);
	pestimator_status_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL, pestimator_status);
	
}


void read_HWSTATUS(phwstatus_HWSTATUS *const phwstatus) {
	int16_t Vcc    = phwstatus_Vcc_GET(phwstatus);
	int8_t  I2Cerr = phwstatus_I2Cerr_GET(phwstatus);
	
}


void read_TIMESYNC(ptimesync_TIMESYNC *const ptimesync) {
	int64_t tc1 = ptimesync_tc1_GET(ptimesync);
	int64_t ts1 = ptimesync_ts1_GET(ptimesync);
	
}


void write_TIMESYNC(ptimesync_TIMESYNC *const ptimesync) {
	ptimesync_tc1_SET(6263897464180046044L, 3592891863384999044L, 4832418686659159790L, -3835555356397188782L, -6872911824791601247L, -4404165579461710034L, 7846015860863553299L, -3682209519959501531L, 6432695793163881989L, -5492878571094549819L, ptimesync);
	ptimesync_ts1_SET(-991658279868465731L, 2003077157501856252L, 1222592924025320433L, 8457510651236751000L, 2449411229740490181L, 4589397511251236532L, -8769412694214503148L, 1773122075489496445L, -2971202854258468483L, 8340602925086858559L, ptimesync);
	
}


void read_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *const pparam_ext_request_list) {
	int8_t target_system    = pparam_ext_request_list_target_system_GET(pparam_ext_request_list);
	int8_t target_component = pparam_ext_request_list_target_component_GET(pparam_ext_request_list);
	
}


void read_GLOBAL_POSITION_INT_COV(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *const pglobal_position_int_cov) {
	int64_t                             time_usec       = pglobal_position_int_cov_time_usec_GET(pglobal_position_int_cov);
	int32_t                             lat             = pglobal_position_int_cov_lat_GET(pglobal_position_int_cov);
	int32_t                             lon             = pglobal_position_int_cov_lon_GET(pglobal_position_int_cov);
	int32_t                             alt             = pglobal_position_int_cov_alt_GET(pglobal_position_int_cov);
	int32_t                             relative_alt    = pglobal_position_int_cov_relative_alt_GET(pglobal_position_int_cov);
	float                               vx              = pglobal_position_int_cov_vx_GET(pglobal_position_int_cov);
	float                               vy              = pglobal_position_int_cov_vy_GET(pglobal_position_int_cov);
	float                               vz              = pglobal_position_int_cov_vz_GET(pglobal_position_int_cov);
	Vglobal_position_int_cov_covariance item_covariance = pglobal_position_int_cov_covariance_GET(pglobal_position_int_cov);
	for (size_t                         index           = 0; index < item_covariance.len; index++)
		some_float = vglobal_position_int_cov_covariance_GET(&item_covariance, index);
	e_MAV_ESTIMATOR_TYPE                item_estimator_type;
	if (pglobal_position_int_cov_estimator_type_GET(pglobal_position_int_cov, &item_estimator_type)) {
		e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE = item_estimator_type;
	}
	
}


void write_GLOBAL_POSITION_INT_COV(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *const pglobal_position_int_cov) {
	pglobal_position_int_cov_time_usec_SET(7412021288796450859L, -208071308336419164L, 5951164719578265040L, -6239974951928776338L, -7944154647527337234L, -1323872430715208109L, 2999333294469262107L, -6440930354971190745L, 4837855227543513100L, -6347691539865961943L, pglobal_position_int_cov);
	pglobal_position_int_cov_lat_SET(-644216562, -549923233, 166730797, 778924607, 1319989420, 552555572, 1219733851, 870866195, 1960374511, 1519757701, pglobal_position_int_cov);
	pglobal_position_int_cov_lon_SET(-15845527, 1682764156, -72672781, -1422252154, -872127079, -1040265424, -1160894464, 1645950139, 1794832015, -1643370904, pglobal_position_int_cov);
	pglobal_position_int_cov_alt_SET(-1340720419, 840660977, 1554068891, -264944787, -825623169, -408274384, -2045366152, 254748870, 387621429, 300611000, pglobal_position_int_cov);
	pglobal_position_int_cov_relative_alt_SET(-1068901572, 1224243017, 856704356, -527694492, 2133895172, 1166592667, -523347020, -297532092, -129050228, 865053105, pglobal_position_int_cov);
	pglobal_position_int_cov_vx_SET(3.9283313E37F, -4.2528162E37F, -1.76495E38F, -2.687432E38F, 1.8168041E38F, -2.0353483E38F, 3.1495707E37F, 1.8633557E38F, -1.7916716E38F, 2.3063037E38F, pglobal_position_int_cov);
	pglobal_position_int_cov_vy_SET(2.9428093E38F, -2.7802953E38F, -2.4177772E38F, 2.1847542E38F, 3.3602555E37F, -1.0535067E38F, 3.1422424E38F, 3.280719E38F, 3.3624661E38F, 1.0623133E38F, pglobal_position_int_cov);
	pglobal_position_int_cov_vz_SET(2.6251715E37F, -1.615408E38F, -7.6281204E37F, -2.2740056E38F, 3.3770741E38F, -1.0987942E38F, 1.7301415E38F, -3.0611757E37F, -1.826244E38F, -2.254473E38F, pglobal_position_int_cov);
	pglobal_position_int_cov_covariance_SET(&-2.647967E38F, -7.954574E37F, 2.8175925E37F, 2.1952293E38F, 1.4148365E38F, 5.902138E36F, 1.8027626E38F, -2.9805054E38F, -2.3065748E38F, -6.0308817E37F, pglobal_position_int_cov);
	pglobal_position_int_cov_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, pglobal_position_int_cov);
	
}


void read_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE *const pbutton_change) {
	int32_t time_boot_ms   = pbutton_change_time_boot_ms_GET(pbutton_change);
	int32_t last_change_ms = pbutton_change_last_change_ms_GET(pbutton_change);
	int8_t  state          = pbutton_change_state_GET(pbutton_change);
	
}


void write_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE *const pbutton_change) {
	pbutton_change_time_boot_ms_SET(-760893013, 2019128836, 670472777, -2138948593, 41964410, 146022037, -1812690848, 30851510, 507452516, 848682736, pbutton_change);
	pbutton_change_last_change_ms_SET(887811514, 2140783778, -144140684, 758577067, -1525670647, 2040543647, 1862978915, -1737460071, 1114639769, -794547757, pbutton_change);
	pbutton_change_state_SET(-43, 119, -56, -11, -91, 1, 17, 66, -120, -2, pbutton_change);
	
}


void read_SAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *const psafety_set_allowed_area) {
	int8_t      target_system    = psafety_set_allowed_area_target_system_GET(psafety_set_allowed_area);
	int8_t      target_component = psafety_set_allowed_area_target_component_GET(psafety_set_allowed_area);
	float       p1x              = psafety_set_allowed_area_p1x_GET(psafety_set_allowed_area);
	float       p1y              = psafety_set_allowed_area_p1y_GET(psafety_set_allowed_area);
	float       p1z              = psafety_set_allowed_area_p1z_GET(psafety_set_allowed_area);
	float       p2x              = psafety_set_allowed_area_p2x_GET(psafety_set_allowed_area);
	float       p2y              = psafety_set_allowed_area_p2y_GET(psafety_set_allowed_area);
	float       p2z              = psafety_set_allowed_area_p2z_GET(psafety_set_allowed_area);
	e_MAV_FRAME item_frame;
	if (psafety_set_allowed_area_frame_GET(psafety_set_allowed_area, &item_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_frame;
	}
	
}


void write_SAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *const psafety_set_allowed_area) {
	psafety_set_allowed_area_target_system_SET(-56, -91, -54, 102, -27, 6, 28, 113, -57, -31, psafety_set_allowed_area);
	psafety_set_allowed_area_target_component_SET(-30, -31, 98, -99, 60, 21, 112, 92, 61, 99, psafety_set_allowed_area);
	psafety_set_allowed_area_p1x_SET(3.0913167E38F, 2.3606153E38F, 2.2877187E38F, 2.5876108E38F, -3.318074E38F, -1.3226613E38F, 1.7575736E38F, 1.9411075E38F, -3.283149E38F, -2.1167702E38F, psafety_set_allowed_area);
	psafety_set_allowed_area_p1y_SET(2.8585176E38F, -2.615008E38F, -8.565739E37F, -1.3689971E38F, -2.2925674E38F, 1.838832E38F, 1.3983505E38F, 2.0055076E38F, -8.535183E37F, -3.3072213E38F, psafety_set_allowed_area);
	psafety_set_allowed_area_p1z_SET(2.8177672E38F, 2.5370183E38F, -2.3002524E38F, -2.096881E38F, -1.8275471E38F, -1.9290216E37F, 7.634931E37F, 1.5549277E38F, -1.9617954E38F, -2.8385463E38F, psafety_set_allowed_area);
	psafety_set_allowed_area_p2x_SET(-3.0363236E38F, 5.79744E37F, -1.7373764E38F, 3.1791237E38F, 1.258928E38F, 2.6616456E38F, -2.6888532E38F, -2.049154E38F, 2.421555E38F, 3.2152643E38F, psafety_set_allowed_area);
	psafety_set_allowed_area_p2y_SET(-2.3824933E38F, 2.644621E38F, -6.495885E37F, -1.1705558E37F, 2.1978546E38F, -5.367038E37F, -2.323756E38F, -2.3097417E38F, -2.1356846E38F, 2.5738422E38F, psafety_set_allowed_area);
	psafety_set_allowed_area_p2z_SET(2.9245872E38F, 1.8174718E38F, 2.2843037E38F, -2.9418558E38F, -2.7951067E38F, -3.1934865E38F, 1.2782417E38F, 2.5759252E38F, 1.5070537E38F, -3.2186172E38F, psafety_set_allowed_area);
	psafety_set_allowed_area_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, psafety_set_allowed_area);
	
}


void read_UAVCAN_NODE_STATUS(puavcan_node_status_UAVCAN_NODE_STATUS *const puavcan_node_status) {
	int16_t              vendor_specific_status_code = puavcan_node_status_vendor_specific_status_code_GET(puavcan_node_status);
	int32_t              uptime_sec                  = puavcan_node_status_uptime_sec_GET(puavcan_node_status);
	int64_t              time_usec                   = puavcan_node_status_time_usec_GET(puavcan_node_status);
	int8_t               sub_mode                    = puavcan_node_status_sub_mode_GET(puavcan_node_status);
	e_UAVCAN_NODE_HEALTH item_health;
	if (puavcan_node_status_health_GET(puavcan_node_status, &item_health)) {
		e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK = item_health;
	}
	e_UAVCAN_NODE_MODE item_mode;
	if (puavcan_node_status_mode_GET(puavcan_node_status, &item_mode)) {
		e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL = item_mode;
	}
	
}


void read_COLLISION(pcollision_COLLISION *const pcollision) {
	int32_t             id                       = pcollision_id_GET(pcollision);
	float               time_to_minimum_delta    = pcollision_time_to_minimum_delta_GET(pcollision);
	float               altitude_minimum_delta   = pcollision_altitude_minimum_delta_GET(pcollision);
	float               horizontal_minimum_delta = pcollision_horizontal_minimum_delta_GET(pcollision);
	e_MAV_COLLISION_SRC item_sRc;
	if (pcollision_sRc_GET(pcollision, &item_sRc)) {
		e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB = item_sRc;
	}
	e_MAV_COLLISION_ACTION item_action;
	if (pcollision_action_GET(pcollision, &item_action)) {
		e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE = item_action;
	}
	e_MAV_COLLISION_THREAT_LEVEL item_threat_level;
	if (pcollision_threat_level_GET(pcollision, &item_threat_level)) {
		e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE = item_threat_level;
	}
	
}


void write_COLLISION(pcollision_COLLISION *const pcollision) {
	pcollision_id_SET(-1140735341, -1881699422, 1420916382, 504008205, 2141795140, 1262103946, -485429206, 1832033832, 879576069, 86183654, pcollision);
	pcollision_time_to_minimum_delta_SET(3.033277E38F, 3.934257E37F, 1.3942782E38F, -1.10506E38F, 4.830084E37F, 1.6512403E37F, 3.088522E38F, 1.1867759E38F, -3.7302759E37F, -2.5420808E38F, pcollision);
	pcollision_altitude_minimum_delta_SET(5.2385636E37F, 2.3741934E38F, 3.2337637E38F, 1.9508751E38F, 1.9090898E37F, 3.366108E38F, 1.2095803E38F, -2.4897838E38F, 8.99235E37F, 2.014125E38F, pcollision);
	pcollision_horizontal_minimum_delta_SET(2.9302216E38F, 1.4687578E38F, -3.3277447E38F, -2.8671997E38F, 2.895677E38F, 1.3830784E38F, 8.3498386E37F, 2.350225E38F, -1.1001324E38F, -1.1810028E38F, pcollision);
	pcollision_sRc_SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, pcollision);
	pcollision_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, pcollision);
	pcollision_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, pcollision);
	
}


void read_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *const pgimbal_torque_cmd_report) {
	int8_t  target_system    = pgimbal_torque_cmd_report_target_system_GET(pgimbal_torque_cmd_report);
	int8_t  target_component = pgimbal_torque_cmd_report_target_component_GET(pgimbal_torque_cmd_report);
	int16_t rl_torque_cmd    = pgimbal_torque_cmd_report_rl_torque_cmd_GET(pgimbal_torque_cmd_report);
	int16_t el_torque_cmd    = pgimbal_torque_cmd_report_el_torque_cmd_GET(pgimbal_torque_cmd_report);
	int16_t az_torque_cmd    = pgimbal_torque_cmd_report_az_torque_cmd_GET(pgimbal_torque_cmd_report);
	
}


void read_ALTITUDE(paltitude_ALTITUDE *const paltitude) {
	int64_t time_usec          = paltitude_time_usec_GET(paltitude);
	float   altitude_monotonic = paltitude_altitude_monotonic_GET(paltitude);
	float   altitude_amsl      = paltitude_altitude_amsl_GET(paltitude);
	float   altitude_local     = paltitude_altitude_local_GET(paltitude);
	float   altitude_relative  = paltitude_altitude_relative_GET(paltitude);
	float   altitude_terrain   = paltitude_altitude_terrain_GET(paltitude);
	float   bottom_clearance   = paltitude_bottom_clearance_GET(paltitude);
	
}


void write_ALTITUDE(paltitude_ALTITUDE *const paltitude) {
	paltitude_time_usec_SET(-4505569425943244904L, 7450074949232136857L, 4273545445580184442L, -709555727734233779L, -5768230367666784708L, 6020985165515918115L, -3794727606891150036L, -8601481853459666340L, -1913578641132833248L, 4956356596120080736L, paltitude);
	paltitude_altitude_monotonic_SET(3.5388292E37F, -1.4012362E38F, -2.6544616E38F, 1.9332726E38F, 3.2191772E38F, -1.4880856E38F, -1.201566E38F, -1.0206655E38F, -2.9933563E38F, -1.2660835E38F, paltitude);
	paltitude_altitude_amsl_SET(2.1717554E38F, 2.49688E38F, 7.736372E37F, -1.9634565E38F, -2.4254726E38F, 1.7194406E37F, 9.181183E37F, 1.3639882E38F, -3.0257816E38F, 1.5555389E38F, paltitude);
	paltitude_altitude_local_SET(3.0522628E37F, -2.1958879E38F, 1.2786047E38F, 2.1973287E38F, 1.0865489E38F, -2.647361E38F, 3.098851E38F, 2.7623096E38F, 2.7642257E38F, -2.2609137E37F, paltitude);
	paltitude_altitude_relative_SET(3.3472128E38F, -3.7635066E36F, -3.0523996E38F, 1.9047493E38F, -2.285138E38F, 2.5391979E38F, -2.9409417E38F, 1.416822E38F, -2.5457803E38F, 2.4267477E38F, paltitude);
	paltitude_altitude_terrain_SET(-1.2470782E38F, 1.4636225E38F, 2.1924747E38F, 2.2445617E37F, 2.260882E38F, -1.1557572E38F, -1.7678884E38F, 1.553153E38F, -3.9629265E37F, 9.707614E37F, paltitude);
	paltitude_bottom_clearance_SET(6.95916E37F, -2.3589485E38F, -1.7167713E38F, 1.9471342E38F, 2.0326702E38F, -2.8186389E37F, -3.2006776E38F, -1.8381203E38F, -3.4010064E38F, 3.503499E37F, paltitude);
	
}


void read_HIL_STATE_QUATERNION(phil_state_quaternion_HIL_STATE_QUATERNION *const phil_state_quaternion) {
	int16_t                                   ind_airspeed             = phil_state_quaternion_ind_airspeed_GET(phil_state_quaternion);
	int16_t                                   true_airspeed            = phil_state_quaternion_true_airspeed_GET(phil_state_quaternion);
	int64_t                                   time_usec                = phil_state_quaternion_time_usec_GET(phil_state_quaternion);
	Vhil_state_quaternion_attitude_quaternion item_attitude_quaternion = phil_state_quaternion_attitude_quaternion_GET(phil_state_quaternion);
	for (size_t                               index                    = 0; index < item_attitude_quaternion.len; index++)
		some_float                                       = vhil_state_quaternion_attitude_quaternion_GET(&item_attitude_quaternion, index);
	float                                     rollspeed  = phil_state_quaternion_rollspeed_GET(phil_state_quaternion);
	float                                     pitchspeed = phil_state_quaternion_pitchspeed_GET(phil_state_quaternion);
	float                                     yawspeed   = phil_state_quaternion_yawspeed_GET(phil_state_quaternion);
	int32_t                                   lat        = phil_state_quaternion_lat_GET(phil_state_quaternion);
	int32_t                                   lon        = phil_state_quaternion_lon_GET(phil_state_quaternion);
	int32_t                                   alt        = phil_state_quaternion_alt_GET(phil_state_quaternion);
	int16_t                                   vx         = phil_state_quaternion_vx_GET(phil_state_quaternion);
	int16_t                                   vy         = phil_state_quaternion_vy_GET(phil_state_quaternion);
	int16_t                                   vz         = phil_state_quaternion_vz_GET(phil_state_quaternion);
	int16_t                                   xacc       = phil_state_quaternion_xacc_GET(phil_state_quaternion);
	int16_t                                   yacc       = phil_state_quaternion_yacc_GET(phil_state_quaternion);
	int16_t                                   zacc       = phil_state_quaternion_zacc_GET(phil_state_quaternion);
	
}


void write_HIL_STATE_QUATERNION(phil_state_quaternion_HIL_STATE_QUATERNION *const phil_state_quaternion) {
	phil_state_quaternion_ind_airspeed_SET(-18490, -23341, -24857, 26935, -16778, -25221, -15526, 25157, 24611, 22513, phil_state_quaternion);
	phil_state_quaternion_true_airspeed_SET(-3914, 5660, -5524, 937, -29250, -1122, 31759, 12551, -28143, -14447, phil_state_quaternion);
	phil_state_quaternion_time_usec_SET(5914256485638391356L, -8858397664709118531L, 2271364889770662457L, -3600435683301667992L, 4925206199223205285L, 8286178640766418642L, 2896925027231419345L, 8789202916000933296L, -2692522290103316693L, -2013094767874962469L, phil_state_quaternion);
	phil_state_quaternion_attitude_quaternion_SET(&3.973131E37F, 3.2066439E38F, 3.1095222E38F, 3.162771E38F, 1.1586479E38F, 1.3136205E38F, -7.1434363E37F, 5.21355E37F, -1.6601521E38F, -2.8694125E38F, phil_state_quaternion);
	phil_state_quaternion_rollspeed_SET(-3.1154238E38F, 1.4808275E38F, 4.6651013E37F, 3.3164249E38F, 3.0006856E38F, 1.4484679E38F, -6.9838644E37F, -3.3306534E38F, 3.2421584E38F, 2.4436298E38F, phil_state_quaternion);
	phil_state_quaternion_pitchspeed_SET(1.2648994E38F, 1.0748168E38F, -2.7433332E38F, -2.6923152E37F, 3.4581582E37F, -2.9823283E38F, -3.3952634E38F, -2.3857675E38F, 4.3056427E37F, -1.5110897E38F, phil_state_quaternion);
	phil_state_quaternion_yawspeed_SET(-3.6152984E37F, 1.1836068E38F, 2.553711E36F, -1.5323292E37F, -9.890288E37F, 1.3863566E38F, 1.3216663E37F, -2.3016028E38F, 1.8792738E38F, -4.847438E37F, phil_state_quaternion);
	phil_state_quaternion_lat_SET(1599745063, 1169562745, 1481636584, -641362399, -262118504, 2127266111, -1002705339, 1609355711, 1215468431, -448674447, phil_state_quaternion);
	phil_state_quaternion_lon_SET(-1817851817, 2058406259, -100529823, -46716864, -1550815545, -1271888157, -1311935521, -68959974, -1385079106, 588377751, phil_state_quaternion);
	phil_state_quaternion_alt_SET(1691907116, -734286265, 1465209961, -813342847, 2069683715, -164092894, -754536474, 867734691, -236942969, -704497485, phil_state_quaternion);
	phil_state_quaternion_vx_SET(-21664, -4669, -11359, 18448, -30074, 12720, 2488, -1513, -18071, 11906, phil_state_quaternion);
	phil_state_quaternion_vy_SET(-723, 23056, 11271, -30911, 5805, -1450, -30834, 5667, 20365, 16597, phil_state_quaternion);
	phil_state_quaternion_vz_SET(7971, 217, -13427, 31693, 25447, -382, 5079, 23372, -27784, 32086, phil_state_quaternion);
	phil_state_quaternion_xacc_SET(-19065, 11273, 27450, -30866, 26352, 2274, -9925, 21317, -26933, 3368, phil_state_quaternion);
	phil_state_quaternion_yacc_SET(-20874, -18383, 14982, 4460, -32026, -24029, -8579, -27061, 14490, 3934, phil_state_quaternion);
	phil_state_quaternion_zacc_SET(-9022, -6729, 19627, -6561, -24296, 23291, -15825, -19018, -14084, 27845, phil_state_quaternion);
	
}


void read_SENSOR_OFFSETS(psensor_offsets_SENSOR_OFFSETS *const psensor_offsets) {
	int16_t mag_ofs_x       = psensor_offsets_mag_ofs_x_GET(psensor_offsets);
	int16_t mag_ofs_y       = psensor_offsets_mag_ofs_y_GET(psensor_offsets);
	int16_t mag_ofs_z       = psensor_offsets_mag_ofs_z_GET(psensor_offsets);
	float   mag_declination = psensor_offsets_mag_declination_GET(psensor_offsets);
	int32_t raw_press       = psensor_offsets_raw_press_GET(psensor_offsets);
	int32_t raw_temp        = psensor_offsets_raw_temp_GET(psensor_offsets);
	float   gyro_cal_x      = psensor_offsets_gyro_cal_x_GET(psensor_offsets);
	float   gyro_cal_y      = psensor_offsets_gyro_cal_y_GET(psensor_offsets);
	float   gyro_cal_z      = psensor_offsets_gyro_cal_z_GET(psensor_offsets);
	float   accel_cal_x     = psensor_offsets_accel_cal_x_GET(psensor_offsets);
	float   accel_cal_y     = psensor_offsets_accel_cal_y_GET(psensor_offsets);
	float   accel_cal_z     = psensor_offsets_accel_cal_z_GET(psensor_offsets);
	
}


void read_STORAGE_INFORMATION(pstorage_information_STORAGE_INFORMATION *const pstorage_information) {
	int32_t time_boot_ms       = pstorage_information_time_boot_ms_GET(pstorage_information);
	int8_t  storage_id         = pstorage_information_storage_id_GET(pstorage_information);
	int8_t  storage_count      = pstorage_information_storage_count_GET(pstorage_information);
	int8_t  status             = pstorage_information_status_GET(pstorage_information);
	float   total_capacity     = pstorage_information_total_capacity_GET(pstorage_information);
	float   used_capacity      = pstorage_information_used_capacity_GET(pstorage_information);
	float   available_capacity = pstorage_information_available_capacity_GET(pstorage_information);
	float   read_speed         = pstorage_information_read_speed_GET(pstorage_information);
	float   write_speed        = pstorage_information_write_speed_GET(pstorage_information);
	
}


void write_STORAGE_INFORMATION(pstorage_information_STORAGE_INFORMATION *const pstorage_information) {
	pstorage_information_time_boot_ms_SET(1338644624, -974597837, -22349632, 1382857453, 186146360, -1553929942, 1377964386, 243109827, 83730704, 1963235444, pstorage_information);
	pstorage_information_storage_id_SET(-27, -55, 72, 121, -13, 114, -76, 20, -30, 110, pstorage_information);
	pstorage_information_storage_count_SET(112, -128, -63, -86, -2, -59, 78, 28, 84, -96, pstorage_information);
	pstorage_information_status_SET(-5, -34, 74, 81, -42, 66, 121, 113, 6, -57, pstorage_information);
	pstorage_information_total_capacity_SET(3.217278E38F, -2.5554278E38F, -2.0459942E38F, -3.369066E38F, 2.0313748E38F, 1.748517E37F, 2.6773073E38F, -7.140866E37F, -4.7458577E35F, -2.0052187E37F, pstorage_information);
	pstorage_information_used_capacity_SET(-5.583723E37F, -3.0769014E38F, -3.0083537E38F, 1.3102659E38F, -8.1731887E36F, 2.2951567E38F, -5.598401E37F, 5.9170654E37F, -1.9688764E38F, 7.663382E37F, pstorage_information);
	pstorage_information_available_capacity_SET(-6.008167E37F, -1.9865854E38F, -7.0770017E37F, -7.613506E36F, -9.213099E37F, -7.8211066E37F, 3.2124384E38F, -2.99476E38F, -1.9277631E38F, 2.9800865E38F, pstorage_information);
	pstorage_information_read_speed_SET(1.0856388E38F, -3.5472504E37F, -2.839296E38F, -2.8660906E37F, 1.272058E38F, -1.6309558E38F, 3.4610534E36F, 2.566131E38F, -6.652702E37F, 5.463175E37F, pstorage_information);
	pstorage_information_write_speed_SET(-1.2852206E37F, -3.225594E37F, 1.0099426E38F, -2.2965217E38F, 1.7561916E38F, -1.6113961E38F, 1.9569523E38F, -1.6418353E38F, -3.3972403E38F, -2.3800815E38F, pstorage_information);
	
}


void read_CAMERA_INFORMATION(pcamera_information_CAMERA_INFORMATION *const pcamera_information) {
	int16_t                         resolution_h           = pcamera_information_resolution_h_GET(pcamera_information);
	int16_t                         resolution_v           = pcamera_information_resolution_v_GET(pcamera_information);
	int16_t                         cam_definition_version = pcamera_information_cam_definition_version_GET(pcamera_information);
	int32_t                         time_boot_ms           = pcamera_information_time_boot_ms_GET(pcamera_information);
	int32_t                         firmware_version       = pcamera_information_firmware_version_GET(pcamera_information);
	Vcamera_information_vendor_name item_vendor_name       = pcamera_information_vendor_name_GET(pcamera_information);
	for (size_t                     index                  = 0; index < item_vendor_name.len; index++)
		some_int8_t                                 = vcamera_information_vendor_name_GET(&item_vendor_name, index);
	Vcamera_information_model_name  item_model_name = pcamera_information_model_name_GET(pcamera_information);
	for (size_t                     index           = 0; index < item_model_name.len; index++)
		some_int8_t                               = vcamera_information_model_name_GET(&item_model_name, index);
	float                           focal_length  = pcamera_information_focal_length_GET(pcamera_information);
	float                           sensor_size_h = pcamera_information_sensor_size_h_GET(pcamera_information);
	float                           sensor_size_v = pcamera_information_sensor_size_v_GET(pcamera_information);
	int8_t                          lens_id       = pcamera_information_lens_id_GET(pcamera_information);
	e_CAMERA_CAP_FLAGS              item_flags;
	if (pcamera_information_flags_GET(pcamera_information, &item_flags)) {
		e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO = item_flags;
	}
	Vcamera_information_cam_definition_uri item_cam_definition_uri;
	if (pcamera_information_cam_definition_uri_GET(pcamera_information, &item_cam_definition_uri)) {
		memcpy(some_string, item_cam_definition_uri.bytes, item_cam_definition_uri.len);
	}
	
}


void write_CAMERA_INFORMATION(pcamera_information_CAMERA_INFORMATION *const pcamera_information) {
	pcamera_information_resolution_h_SET(750, 2089, -14606, 25079, 5457, -19053, -6534, -16927, -11262, -21366, pcamera_information);
	pcamera_information_resolution_v_SET(-31346, 28408, -10003, -11555, 17407, -15395, 14052, 16414, -27061, 873, pcamera_information);
	pcamera_information_cam_definition_version_SET(6655, 21712, -19709, 23661, -3537, 4795, -3943, -9506, 12654, 21951, pcamera_information);
	pcamera_information_time_boot_ms_SET(-1624778251, 311037328, -1927224411, -1247911424, 1485992180, -1674604438, -564106770, 1369237934, 372191752, 1819492016, pcamera_information);
	pcamera_information_firmware_version_SET(986711281, 1280171973, 1040085832, 1828852249, 1099426078, -1617657411, 161560663, 1661488287, -1793130572, -1418539079, pcamera_information);
	pcamera_information_vendor_name_SET(&-47, -118, 64, 101, -62, -74, 58, -6, 104, 119, pcamera_information);
	pcamera_information_model_name_SET(&63, 76, -100, 57, 52, -80, 47, -54, -49, 69, pcamera_information);
	pcamera_information_focal_length_SET(-5.1583624E37F, 7.910045E37F, -3.3761156E38F, -2.4658745E38F, 2.0974827E38F, 2.0721698E38F, 2.699564E38F, 5.128727E37F, -1.9953119E38F, 1.5999358E38F, pcamera_information);
	pcamera_information_sensor_size_h_SET(3.0963903E38F, 1.5741151E37F, 2.864637E38F, 2.6536906E38F, -5.761482E37F, 8.407042E37F, -9.813509E37F, 1.2684071E37F, 2.7908947E38F, -7.3217435E37F, pcamera_information);
	pcamera_information_sensor_size_v_SET(8.474448E37F, 2.9369641E38F, 2.4428177E38F, -1.6896084E38F, -6.904011E37F, -2.0814233E38F, -3.2058354E38F, 3.0050442E38F, -4.4611474E37F, 2.9361717E38F, pcamera_information);
	pcamera_information_lens_id_SET(-93, 104, -94, -76, -87, 69, 43, 96, -64, 44, pcamera_information);
	pcamera_information_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, pcamera_information);
	pcamera_information_cam_definition_uri_SET(some_string, strlen(some_string), pcamera_information);
	
}


void read_GPS_STATUS(pgps_status_GPS_STATUS *const pgps_status) {
	int8_t                          satellites_visible = pgps_status_satellites_visible_GET(pgps_status);
	Vgps_status_satellite_prn       item_satellite_prn = pgps_status_satellite_prn_GET(pgps_status);
	for (size_t                     index              = 0; index < item_satellite_prn.len; index++)
		some_int8_t                                     = vgps_status_satellite_prn_GET(&item_satellite_prn, index);
	Vgps_status_satellite_used      item_satellite_used = pgps_status_satellite_used_GET(pgps_status);
	for (size_t                     index               = 0; index < item_satellite_used.len; index++)
		some_int8_t                                          = vgps_status_satellite_used_GET(&item_satellite_used, index);
	Vgps_status_satellite_elevation item_satellite_elevation = pgps_status_satellite_elevation_GET(pgps_status);
	for (size_t                     index                    = 0; index < item_satellite_elevation.len; index++)
		some_int8_t                                        = vgps_status_satellite_elevation_GET(&item_satellite_elevation, index);
	Vgps_status_satellite_azimuth   item_satellite_azimuth = pgps_status_satellite_azimuth_GET(pgps_status);
	for (size_t                     index                  = 0; index < item_satellite_azimuth.len; index++)
		some_int8_t                                    = vgps_status_satellite_azimuth_GET(&item_satellite_azimuth, index);
	Vgps_status_satellite_snr       item_satellite_snr = pgps_status_satellite_snr_GET(pgps_status);
	for (size_t                     index              = 0; index < item_satellite_snr.len; index++)
		some_int8_t = vgps_status_satellite_snr_GET(&item_satellite_snr, index);
	
}


void write_GPS_STATUS(pgps_status_GPS_STATUS *const pgps_status) {
	pgps_status_satellites_visible_SET(-4, 75, 74, -99, -68, 16, 91, 11, 22, 118, pgps_status);
	pgps_status_satellite_prn_SET(&8, -95, 53, -113, -75, 44, -28, -88, 7, -83, pgps_status);
	pgps_status_satellite_used_SET(&-124, 112, 33, -79, 47, 71, -107, -3, 67, -14, pgps_status);
	pgps_status_satellite_elevation_SET(&21, 66, 65, 45, -69, 109, 115, -9, 116, 65, pgps_status);
	pgps_status_satellite_azimuth_SET(&45, -45, -124, 15, -18, 17, 3, -92, 121, 51, pgps_status);
	pgps_status_satellite_snr_SET(&-7, -26, 25, -124, -31, -51, 51, -111, -10, 93, pgps_status);
	
}


void read_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *const pdevice_op_write_reply) {
	int32_t request_id = pdevice_op_write_reply_request_id_GET(pdevice_op_write_reply);
	int8_t  result     = pdevice_op_write_reply_result_GET(pdevice_op_write_reply);
	
}


void read_PARAM_SET(pparam_set_PARAM_SET *const pparam_set) {
	int8_t              target_system    = pparam_set_target_system_GET(pparam_set);
	int8_t              target_component = pparam_set_target_component_GET(pparam_set);
	float               param_value      = pparam_set_param_value_GET(pparam_set);
	Vparam_set_param_id item_param_id;
	if (pparam_set_param_id_GET(pparam_set, &item_param_id)) {
		memcpy(some_string, item_param_id.bytes, item_param_id.len);
	}
	e_MAV_PARAM_TYPE item_param_type;
	if (pparam_set_param_type_GET(pparam_set, &item_param_type)) {
		e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8 = item_param_type;
	}
	
}


void write_PARAM_SET(pparam_set_PARAM_SET *const pparam_set) {
	pparam_set_target_system_SET(121, 33, 63, -104, 91, -69, 20, -80, -6, 108, pparam_set);
	pparam_set_target_component_SET(112, 83, -93, -103, 93, 7, -17, -110, 36, -22, pparam_set);
	pparam_set_param_value_SET(1.3810095E38F, 1.8699102E38F, 1.7129957E38F, -8.1462605E37F, 2.4031767E38F, 2.7297825E38F, 1.2388406E38F, 9.222623E37F, 1.15111E38F, -3.148254E38F, pparam_set);
	pparam_set_param_id_SET(some_string, strlen(some_string), pparam_set);
	pparam_set_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, pparam_set);
	
}


void read_TERRAIN_DATA(pterrain_data_TERRAIN_DATA *const pterrain_data) {
	int16_t            grid_spacing = pterrain_data_grid_spacing_GET(pterrain_data);
	int32_t            lat          = pterrain_data_lat_GET(pterrain_data);
	int32_t            lon          = pterrain_data_lon_GET(pterrain_data);
	int8_t             gridbit      = pterrain_data_gridbit_GET(pterrain_data);
	Vterrain_data_daTa item_daTa    = pterrain_data_daTa_GET(pterrain_data);
	for (size_t        index        = 0; index < item_daTa.len; index++)
		some_int16_t = vterrain_data_daTa_GET(&item_daTa, index);
	
}


void write_TERRAIN_DATA(pterrain_data_TERRAIN_DATA *const pterrain_data) {
	pterrain_data_grid_spacing_SET(28766, 12098, 1483, -19568, -22844, 10945, 26641, -7525, 8697, -508, pterrain_data);
	pterrain_data_lat_SET(1794905097, -996277540, -1255972275, -968745379, -238078831, 1730339511, 1688245203, -77694974, 351573427, 985430598, pterrain_data);
	pterrain_data_lon_SET(2049516592, 930358356, 914990046, 1526339483, 1724958901, -1619663591, -29829063, 1081140357, -355740486, -2092029911, pterrain_data);
	pterrain_data_gridbit_SET(74, -48, 111, 40, 92, -48, 2, 44, 15, 51, pterrain_data);
	pterrain_data_daTa_SET(&-9383, -15557, -21312, -24801, -4876, -27431, 9359, -30973, -1079, 6718, pterrain_data);
	
}


void read_GIMBAL_CONTROL(pgimbal_control_GIMBAL_CONTROL *const pgimbal_control) {
	int8_t target_system    = pgimbal_control_target_system_GET(pgimbal_control);
	int8_t target_component = pgimbal_control_target_component_GET(pgimbal_control);
	float  demanded_rate_x  = pgimbal_control_demanded_rate_x_GET(pgimbal_control);
	float  demanded_rate_y  = pgimbal_control_demanded_rate_y_GET(pgimbal_control);
	float  demanded_rate_z  = pgimbal_control_demanded_rate_z_GET(pgimbal_control);
	
}


void read_RC_CHANNELS_OVERRIDE(prc_channels_override_RC_CHANNELS_OVERRIDE *const prc_channels_override) {
	int16_t chan1_raw        = prc_channels_override_chan1_raw_GET(prc_channels_override);
	int16_t chan2_raw        = prc_channels_override_chan2_raw_GET(prc_channels_override);
	int16_t chan3_raw        = prc_channels_override_chan3_raw_GET(prc_channels_override);
	int16_t chan4_raw        = prc_channels_override_chan4_raw_GET(prc_channels_override);
	int16_t chan5_raw        = prc_channels_override_chan5_raw_GET(prc_channels_override);
	int16_t chan6_raw        = prc_channels_override_chan6_raw_GET(prc_channels_override);
	int16_t chan7_raw        = prc_channels_override_chan7_raw_GET(prc_channels_override);
	int16_t chan8_raw        = prc_channels_override_chan8_raw_GET(prc_channels_override);
	int8_t  target_system    = prc_channels_override_target_system_GET(prc_channels_override);
	int8_t  target_component = prc_channels_override_target_component_GET(prc_channels_override);
	
}


void write_RC_CHANNELS_OVERRIDE(prc_channels_override_RC_CHANNELS_OVERRIDE *const prc_channels_override) {
	prc_channels_override_chan1_raw_SET(15541, 13624, 22490, -26660, -10322, -29237, -21865, -3635, 5652, -26872, prc_channels_override);
	prc_channels_override_chan2_raw_SET(7335, -12225, 28418, 11004, -32078, -30489, -7367, -243, -21553, -9135, prc_channels_override);
	prc_channels_override_chan3_raw_SET(23916, 22253, 31666, -19370, -31599, 12369, -14542, 24517, 20838, 22296, prc_channels_override);
	prc_channels_override_chan4_raw_SET(-23551, 2179, 30900, -6423, -26949, 11737, 10605, 16552, 5256, 18441, prc_channels_override);
	prc_channels_override_chan5_raw_SET(4699, 23059, -601, 24115, -26027, 17732, -29507, -20380, -8871, 19554, prc_channels_override);
	prc_channels_override_chan6_raw_SET(-14309, -11649, -10834, -29545, 10432, 6691, 19153, 816, -30694, 2227, prc_channels_override);
	prc_channels_override_chan7_raw_SET(18108, 2778, 11041, 10465, -19919, 4405, -27373, 339, 21873, 28867, prc_channels_override);
	prc_channels_override_chan8_raw_SET(24318, -1391, 29805, -3327, 3363, 26133, -11135, 11417, 21958, -5676, prc_channels_override);
	prc_channels_override_target_system_SET(-65, -67, 2, -21, -11, 118, 124, -64, -50, 8, prc_channels_override);
	prc_channels_override_target_component_SET(-15, 50, 72, 12, -125, -77, 99, -34, -86, -20, prc_channels_override);
	
}


void read_SCALED_IMU(pscaled_imu_SCALED_IMU *const pscaled_imu) {
	int32_t time_boot_ms = pscaled_imu_time_boot_ms_GET(pscaled_imu);
	int16_t xacc         = pscaled_imu_xacc_GET(pscaled_imu);
	int16_t yacc         = pscaled_imu_yacc_GET(pscaled_imu);
	int16_t zacc         = pscaled_imu_zacc_GET(pscaled_imu);
	int16_t xgyro        = pscaled_imu_xgyro_GET(pscaled_imu);
	int16_t ygyro        = pscaled_imu_ygyro_GET(pscaled_imu);
	int16_t zgyro        = pscaled_imu_zgyro_GET(pscaled_imu);
	int16_t xmag         = pscaled_imu_xmag_GET(pscaled_imu);
	int16_t ymag         = pscaled_imu_ymag_GET(pscaled_imu);
	int16_t zmag         = pscaled_imu_zmag_GET(pscaled_imu);
	
}


void write_SCALED_IMU(pscaled_imu_SCALED_IMU *const pscaled_imu) {
	pscaled_imu_time_boot_ms_SET(-1341080810, 1929490511, -446568085, -215013028, 459413437, 1457922286, 1793587004, 225308521, 1526593166, -862422876, pscaled_imu);
	pscaled_imu_xacc_SET(-24513, 23087, 12756, -18651, -7856, -20319, -24684, -6013, -4175, -5601, pscaled_imu);
	pscaled_imu_yacc_SET(-31179, 6301, 26393, 8627, -6914, 16173, -27878, -30259, -6740, 25848, pscaled_imu);
	pscaled_imu_zacc_SET(6398, 29936, 28799, -2801, 3505, 2987, 7688, 25745, -14205, -21670, pscaled_imu);
	pscaled_imu_xgyro_SET(-27265, 18056, -9625, 8409, 3093, -31701, 5595, 1547, -13796, 15806, pscaled_imu);
	pscaled_imu_ygyro_SET(-8950, -27675, 7313, 14288, 16024, 19172, -12777, -29161, -29058, 30464, pscaled_imu);
	pscaled_imu_zgyro_SET(-2996, 11367, 32440, -26763, -29933, -28410, -15793, -7311, 28098, 24016, pscaled_imu);
	pscaled_imu_xmag_SET(-4963, -7423, -1201, -3926, 12993, 16549, 24756, -1666, -16135, -15205, pscaled_imu);
	pscaled_imu_ymag_SET(8660, 27219, 19151, -13457, 27861, -3199, -4389, 18817, -4191, -26654, pscaled_imu);
	pscaled_imu_zmag_SET(24651, 25154, 30126, 24546, 1957, -28010, 30418, 9529, -3079, 11, pscaled_imu);
	
}


void read_VIDEO_STREAM_INFORMATION(pvideo_stream_information_VIDEO_STREAM_INFORMATION *const pvideo_stream_information) {
	int16_t                       resolution_h = pvideo_stream_information_resolution_h_GET(pvideo_stream_information);
	int16_t                       resolution_v = pvideo_stream_information_resolution_v_GET(pvideo_stream_information);
	int16_t                       rotation     = pvideo_stream_information_rotation_GET(pvideo_stream_information);
	int32_t                       bitrate      = pvideo_stream_information_bitrate_GET(pvideo_stream_information);
	int8_t                        camera_id    = pvideo_stream_information_camera_id_GET(pvideo_stream_information);
	int8_t                        status       = pvideo_stream_information_status_GET(pvideo_stream_information);
	float                         framerate    = pvideo_stream_information_framerate_GET(pvideo_stream_information);
	Vvideo_stream_information_uri item_uri;
	if (pvideo_stream_information_uri_GET(pvideo_stream_information, &item_uri)) {
		memcpy(some_string, item_uri.bytes, item_uri.len);
	}
	
}


void read_AHRS(pahrs_AHRS *const pahrs) {
	float omegaIx      = pahrs_omegaIx_GET(pahrs);
	float omegaIy      = pahrs_omegaIy_GET(pahrs);
	float omegaIz      = pahrs_omegaIz_GET(pahrs);
	float accel_weight = pahrs_accel_weight_GET(pahrs);
	float renorm_val   = pahrs_renorm_val_GET(pahrs);
	float error_rp     = pahrs_error_rp_GET(pahrs);
	float error_yaw    = pahrs_error_yaw_GET(pahrs);
	
}


void read_DEBUG(pdebug_DEBUG *const pdebug) {
	int32_t time_boot_ms = pdebug_time_boot_ms_GET(pdebug);
	int8_t  ind          = pdebug_ind_GET(pdebug);
	float   value        = pdebug_value_GET(pdebug);
	
}


void write_DEBUG(pdebug_DEBUG *const pdebug) {
	pdebug_time_boot_ms_SET(-1274914791, -498580691, 866354318, -995367342, -77391374, 2063461788, 411521780, 436556478, -643312575, 1542965177, pdebug);
	pdebug_ind_SET(79, 99, 31, -106, -121, 97, 48, 39, -93, 91, pdebug);
	pdebug_value_SET(-1.453966E38F, -3.1606833E38F, -1.2103337E38F, -1.7995976E38F, 2.5495395E38F, -1.0268046E38F, 8.946058E37F, -1.7032675E38F, -1.3874622E38F, 1.0861992E38F, pdebug);
	
}


void read_CAMERA_IMAGE_CAPTURED(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *const pcamera_image_captured) {
	int32_t                         time_boot_ms = pcamera_image_captured_time_boot_ms_GET(pcamera_image_captured);
	int64_t                         time_utc     = pcamera_image_captured_time_utc_GET(pcamera_image_captured);
	int8_t                          camera_id    = pcamera_image_captured_camera_id_GET(pcamera_image_captured);
	int32_t                         lat          = pcamera_image_captured_lat_GET(pcamera_image_captured);
	int32_t                         lon          = pcamera_image_captured_lon_GET(pcamera_image_captured);
	int32_t                         alt          = pcamera_image_captured_alt_GET(pcamera_image_captured);
	int32_t                         relative_alt = pcamera_image_captured_relative_alt_GET(pcamera_image_captured);
	Vcamera_image_captured_q        item_q       = pcamera_image_captured_q_GET(pcamera_image_captured);
	for (size_t                     index        = 0; index < item_q.len; index++)
		some_float                                 = vcamera_image_captured_q_GET(&item_q, index);
	int32_t                         image_index    = pcamera_image_captured_image_index_GET(pcamera_image_captured);
	int8_t                          capture_result = pcamera_image_captured_capture_result_GET(pcamera_image_captured);
	Vcamera_image_captured_file_url item_file_url;
	if (pcamera_image_captured_file_url_GET(pcamera_image_captured, &item_file_url)) {
		memcpy(some_string, item_file_url.bytes, item_file_url.len);
	}
	
}


void write_CAMERA_IMAGE_CAPTURED(pcamera_image_captured_CAMERA_IMAGE_CAPTURED *const pcamera_image_captured) {
	pcamera_image_captured_time_boot_ms_SET(-95406324, 1053019430, -1069169947, -1732172587, -713345106, -728876728, -1841207032, -120453698, 1385740462, -1310948831, pcamera_image_captured);
	pcamera_image_captured_time_utc_SET(5602200610385356008L, 5053136634630803021L, -8873880085220842102L, -7563481635406289286L, -1833526672870827194L, 4370187743035923557L, -2741965309554024802L, 5838833415108950643L, 6670465560885601877L, -1579291434313927507L, pcamera_image_captured);
	pcamera_image_captured_camera_id_SET(1, -7, -95, 16, 117, 66, 32, 17, -76, -111, pcamera_image_captured);
	pcamera_image_captured_lat_SET(-130762256, -678983938, 2076772126, -1806036490, -549186842, 1098482767, -919054587, 456932043, -1636064214, 1361817919, pcamera_image_captured);
	pcamera_image_captured_lon_SET(583723074, -830132326, 1298324494, 1615906921, -1842098480, -863402777, 1339128756, 624667200, -2011951241, 500147234, pcamera_image_captured);
	pcamera_image_captured_alt_SET(562596156, -1265908566, -2073963554, 835026224, -1393305206, 808860470, -147392323, -830307226, 1145863658, -19828284, pcamera_image_captured);
	pcamera_image_captured_relative_alt_SET(944118315, 378730497, 553773111, 269075807, 1366616523, 1535759549, -867677016, 459186310, -705779298, 492542203, pcamera_image_captured);
	pcamera_image_captured_q_SET(&-3.302764E38F, -1.829794E38F, -1.9945235E38F, -2.762137E38F, -9.161748E37F, 2.2011345E38F, -2.3065627E36F, -2.6037724E38F, 1.6153952E37F, 4.1762572E37F, pcamera_image_captured);
	pcamera_image_captured_image_index_SET(1997883443, 1540463322, 32598502, 2112484712, -272199037, 990945170, 682462764, 1378130831, 1855059114, -1008462382, pcamera_image_captured);
	pcamera_image_captured_capture_result_SET(18, 59, -17, 45, -33, 95, -88, 16, 7, -87, pcamera_image_captured);
	pcamera_image_captured_file_url_SET(some_string, strlen(some_string), pcamera_image_captured);
	
}


void read_LOG_ENTRY(plog_entry_LOG_ENTRY *const plog_entry) {
	int16_t id           = plog_entry_id_GET(plog_entry);
	int16_t num_logs     = plog_entry_num_logs_GET(plog_entry);
	int16_t last_log_num = plog_entry_last_log_num_GET(plog_entry);
	int32_t time_utc     = plog_entry_time_utc_GET(plog_entry);
	int32_t size         = plog_entry_size_GET(plog_entry);
	
}


void write_LOG_ENTRY(plog_entry_LOG_ENTRY *const plog_entry) {
	plog_entry_id_SET(18882, -27869, -23026, -30162, 1647, -2223, -10810, 31927, -724, 30091, plog_entry);
	plog_entry_num_logs_SET(-7143, -22783, -16608, -5038, -31374, 17108, 10029, -22436, 26479, -30712, plog_entry);
	plog_entry_last_log_num_SET(-29347, 28980, -18918, 19409, 17263, 21964, -21178, -20149, -12844, 15781, plog_entry);
	plog_entry_time_utc_SET(-1925287281, 2020292591, 1077455355, 1494113888, -1308198660, -552364940, 1963070236, -904302221, -1000578433, -1134382322, plog_entry);
	plog_entry_size_SET(-2055361033, -293698027, 1307774457, 1857512328, 443883516, -589259804, -101829827, -1981971273, 1331967439, -1085351167, plog_entry);
	
}


void read_ACTUATOR_CONTROL_TARGET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *const pactuator_control_target) {
	int64_t                           time_usec     = pactuator_control_target_time_usec_GET(pactuator_control_target);
	int8_t                            group_mlx     = pactuator_control_target_group_mlx_GET(pactuator_control_target);
	Vactuator_control_target_controls item_controls = pactuator_control_target_controls_GET(pactuator_control_target);
	for (size_t                       index         = 0; index < item_controls.len; index++)
		some_float = vactuator_control_target_controls_GET(&item_controls, index);
	
}


void write_ACTUATOR_CONTROL_TARGET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *const pactuator_control_target) {
	pactuator_control_target_time_usec_SET(4363833590066771183L, 3709421626956622900L, -118572861178817008L, 8618417741003967577L, 4849078420317830812L, -823628512493487274L, -6380893585475657074L, -6532677321244451102L, 6468686706573050991L, -7761297505284201641L, pactuator_control_target);
	pactuator_control_target_group_mlx_SET(107, 80, -32, -4, -11, -61, 49, 101, -112, 93, pactuator_control_target);
	pactuator_control_target_controls_SET(&1.739966E38F, -2.9716353E38F, -1.6122399E38F, -1.5854261E37F, -1.855985E38F, 2.1624464E38F, -2.9833256E38F, 8.275827E37F, -2.1859006E38F, -5.278293E37F, pactuator_control_target);
	
}


void read_HIGH_LATENCY(phigh_latency_HIGH_LATENCY *const phigh_latency) {
	int16_t         heading           = phigh_latency_heading_GET(phigh_latency);
	int16_t         wp_distance       = phigh_latency_wp_distance_GET(phigh_latency);
	int32_t         custom_mode       = phigh_latency_custom_mode_GET(phigh_latency);
	int16_t         roll              = phigh_latency_roll_GET(phigh_latency);
	int16_t         pitch             = phigh_latency_pitch_GET(phigh_latency);
	int8_t          throttle          = phigh_latency_throttle_GET(phigh_latency);
	int16_t         heading_sp        = phigh_latency_heading_sp_GET(phigh_latency);
	int32_t         latitude          = phigh_latency_latitude_GET(phigh_latency);
	int32_t         longitude         = phigh_latency_longitude_GET(phigh_latency);
	int16_t         altitude_amsl     = phigh_latency_altitude_amsl_GET(phigh_latency);
	int16_t         altitude_sp       = phigh_latency_altitude_sp_GET(phigh_latency);
	int8_t          airspeed          = phigh_latency_airspeed_GET(phigh_latency);
	int8_t          airspeed_sp       = phigh_latency_airspeed_sp_GET(phigh_latency);
	int8_t          groundspeed       = phigh_latency_groundspeed_GET(phigh_latency);
	int8_t          climb_rate        = phigh_latency_climb_rate_GET(phigh_latency);
	int8_t          gps_nsat          = phigh_latency_gps_nsat_GET(phigh_latency);
	int8_t          battery_remaining = phigh_latency_battery_remaining_GET(phigh_latency);
	int8_t          temperature       = phigh_latency_temperature_GET(phigh_latency);
	int8_t          temperature_air   = phigh_latency_temperature_air_GET(phigh_latency);
	int8_t          failsafe          = phigh_latency_failsafe_GET(phigh_latency);
	int8_t          wp_num            = phigh_latency_wp_num_GET(phigh_latency);
	e_MAV_MODE_FLAG item_base_mode;
	if (phigh_latency_base_mode_GET(phigh_latency, &item_base_mode)) {
		e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = item_base_mode;
	}
	e_MAV_LANDED_STATE item_landed_state;
	if (phigh_latency_landed_state_GET(phigh_latency, &item_landed_state)) {
		e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED = item_landed_state;
	}
	e_GPS_FIX_TYPE item_gps_fix_type;
	if (phigh_latency_gps_fix_type_GET(phigh_latency, &item_gps_fix_type)) {
		e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS = item_gps_fix_type;
	}
	
}


void write_HIGH_LATENCY(phigh_latency_HIGH_LATENCY *const phigh_latency) {
	phigh_latency_heading_SET(-313, -20100, 14171, 24871, -27209, 9075, -32511, -27071, 642, 12469, phigh_latency);
	phigh_latency_wp_distance_SET(25485, -2525, 14941, -15824, -29384, -21762, 18134, -4571, 31352, -11614, phigh_latency);
	phigh_latency_custom_mode_SET(-745859913, 1110947371, 868064111, -317725718, 872994800, 567582686, -2083891873, -1225906126, 1951427225, -1517577461, phigh_latency);
	phigh_latency_roll_SET(-19750, -24506, 12862, -14394, -7445, -12031, 11723, 3321, 22462, -22487, phigh_latency);
	phigh_latency_pitch_SET(15157, 24874, -2523, -31311, 31120, 24004, 16579, -3938, 13975, -17631, phigh_latency);
	phigh_latency_throttle_SET(114, -49, 38, -121, 59, -84, 73, -95, -70, -58, phigh_latency);
	phigh_latency_heading_sp_SET(-29674, 943, 15727, -7961, 27156, 24655, -27723, 19285, 3240, 31291, phigh_latency);
	phigh_latency_latitude_SET(-500412760, 712826410, 610372089, -2108852178, 930404970, 715989139, 1524532318, 264637386, -1598655130, -1476932216, phigh_latency);
	phigh_latency_longitude_SET(2110682475, -426157021, -2045813874, -1267046054, 1368693574, 1794607401, -2055736877, 1955828674, 797669604, 1694081821, phigh_latency);
	phigh_latency_altitude_amsl_SET(13687, -8436, 31708, -28399, -5688, -8504, -11675, -25330, -13908, -6499, phigh_latency);
	phigh_latency_altitude_sp_SET(693, -951, -11157, 23348, -20284, -21299, -16519, -23571, -30228, 12373, phigh_latency);
	phigh_latency_airspeed_SET(86, 84, 45, 5, 91, -9, -77, 43, -14, 67, phigh_latency);
	phigh_latency_airspeed_sp_SET(122, 58, 71, -98, 122, 108, 87, -57, 104, -116, phigh_latency);
	phigh_latency_groundspeed_SET(-43, 15, -94, 13, -42, -124, 112, 59, -39, 74, phigh_latency);
	phigh_latency_climb_rate_SET(-51, -84, 86, -3, 105, 2, 2, -98, -65, 99, phigh_latency);
	phigh_latency_gps_nsat_SET(60, 34, 65, -79, -83, 29, -98, 29, -78, -76, phigh_latency);
	phigh_latency_battery_remaining_SET(96, -13, -8, -24, 111, -43, 80, 75, 68, 111, phigh_latency);
	phigh_latency_temperature_SET(55, 85, 113, 68, -14, 47, 30, 114, 19, 102, phigh_latency);
	phigh_latency_temperature_air_SET(-78, -57, 116, 22, 1, 44, -99, 70, 29, -54, phigh_latency);
	phigh_latency_failsafe_SET(56, 1, 42, 81, 72, -98, -8, 53, 66, -122, phigh_latency);
	phigh_latency_wp_num_SET(-106, -81, -43, 94, -42, -80, -61, 26, -42, 108, phigh_latency);
	phigh_latency_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, phigh_latency);
	phigh_latency_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, phigh_latency);
	phigh_latency_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, phigh_latency);
	
}


void read_PARAM_REQUEST_READ(pparam_request_read_PARAM_REQUEST_READ *const pparam_request_read) {
	int8_t                       target_system    = pparam_request_read_target_system_GET(pparam_request_read);
	int8_t                       target_component = pparam_request_read_target_component_GET(pparam_request_read);
	int16_t                      param_index      = pparam_request_read_param_index_GET(pparam_request_read);
	Vparam_request_read_param_id item_param_id;
	if (pparam_request_read_param_id_GET(pparam_request_read, &item_param_id)) {
		memcpy(some_string, item_param_id.bytes, item_param_id.len);
	}
	
}


void write_PARAM_REQUEST_READ(pparam_request_read_PARAM_REQUEST_READ *const pparam_request_read) {
	pparam_request_read_target_system_SET(-57, -103, -38, 84, -112, -86, 91, -110, 92, 101, pparam_request_read);
	pparam_request_read_target_component_SET(-61, -90, 62, -10, 102, -62, 60, -7, 127, -91, pparam_request_read);
	pparam_request_read_param_index_SET(26288, 13302, -22357, 2925, -27504, 27656, -6144, 21812, -8605, -25254, pparam_request_read);
	pparam_request_read_param_id_SET(some_string, strlen(some_string), pparam_request_read);
	
}


void read_SET_ATTITUDE_TARGET(pset_attitude_target_SET_ATTITUDE_TARGET *const pset_attitude_target) {
	int32_t                time_boot_ms     = pset_attitude_target_time_boot_ms_GET(pset_attitude_target);
	int8_t                 target_system    = pset_attitude_target_target_system_GET(pset_attitude_target);
	int8_t                 target_component = pset_attitude_target_target_component_GET(pset_attitude_target);
	int8_t                 type_mask        = pset_attitude_target_type_mask_GET(pset_attitude_target);
	Vset_attitude_target_q item_q           = pset_attitude_target_q_GET(pset_attitude_target);
	for (size_t            index            = 0; index < item_q.len; index++)
		some_float                         = vset_attitude_target_q_GET(&item_q, index);
	float                  body_roll_rate  = pset_attitude_target_body_roll_rate_GET(pset_attitude_target);
	float                  body_pitch_rate = pset_attitude_target_body_pitch_rate_GET(pset_attitude_target);
	float                  body_yaw_rate   = pset_attitude_target_body_yaw_rate_GET(pset_attitude_target);
	float                  thrust          = pset_attitude_target_thrust_GET(pset_attitude_target);
	
}


void write_SET_ATTITUDE_TARGET(pset_attitude_target_SET_ATTITUDE_TARGET *const pset_attitude_target) {
	pset_attitude_target_time_boot_ms_SET(-444432681, 1551966361, 826499908, -1021674262, 1338638508, -1125539683, 1618060700, -1215046496, 170167249, -479667020, pset_attitude_target);
	pset_attitude_target_target_system_SET(103, -31, -43, -74, 81, -99, -94, 66, 114, -126, pset_attitude_target);
	pset_attitude_target_target_component_SET(-111, -11, 118, -93, -111, 52, 122, -39, -104, 73, pset_attitude_target);
	pset_attitude_target_type_mask_SET(-44, 111, 20, -14, 4, 61, -10, -86, -67, 85, pset_attitude_target);
	pset_attitude_target_q_SET(&-2.815382E38F, 1.9503798E38F, 2.7624425E38F, 2.9886737E38F, 2.3826183E38F, -2.9936626E37F, 2.4525504E38F, 1.5039054E38F, -3.242061E38F, 6.0162667E37F, pset_attitude_target);
	pset_attitude_target_body_roll_rate_SET(2.201195E38F, -1.3450299E38F, -5.232468E37F, -1.2502416E38F, -8.92979E37F, 2.1756626E38F, 2.1434554E38F, -2.2072495E38F, 2.0786582E38F, 1.3249274E38F, pset_attitude_target);
	pset_attitude_target_body_pitch_rate_SET(7.4400635E37F, 1.563853E38F, -1.9192202E38F, 2.9811205E37F, 1.8865448E38F, 8.1127636E37F, 3.8654234E37F, 2.465789E38F, 1.722815E37F, 6.6195964E37F, pset_attitude_target);
	pset_attitude_target_body_yaw_rate_SET(1.1235285E38F, -2.7968056E38F, 9.2876684E36F, -1.5898251E38F, -2.936239E38F, 1.5904378E38F, -3.1723936E38F, -1.8909293E38F, -2.194447E38F, 3.1164267E38F, pset_attitude_target);
	pset_attitude_target_thrust_SET(2.579549E38F, 1.7420339E38F, 2.2750903E38F, 2.10868E38F, 3.3575578E38F, 1.19025E37F, 1.6575526E38F, -1.1099568E37F, 1.3239315E38F, -1.6659668E38F, pset_attitude_target);
	
}


void read_FOLLOW_TARGET(pfollow_target_FOLLOW_TARGET *const pfollow_target) {
	int64_t                     timestamp        = pfollow_target_timestamp_GET(pfollow_target);
	int64_t                     custom_state     = pfollow_target_custom_state_GET(pfollow_target);
	int8_t                      est_capabilities = pfollow_target_est_capabilities_GET(pfollow_target);
	int32_t                     lat              = pfollow_target_lat_GET(pfollow_target);
	int32_t                     lon              = pfollow_target_lon_GET(pfollow_target);
	float                       alt              = pfollow_target_alt_GET(pfollow_target);
	Vfollow_target_vel          item_vel         = pfollow_target_vel_GET(pfollow_target);
	for (size_t                 index            = 0; index < item_vel.len; index++)
		some_float                       = vfollow_target_vel_GET(&item_vel, index);
	Vfollow_target_acc          item_acc = pfollow_target_acc_GET(pfollow_target);
	for (size_t                 index    = 0; index < item_acc.len; index++)
		some_float                              = vfollow_target_acc_GET(&item_acc, index);
	Vfollow_target_attitude_q   item_attitude_q = pfollow_target_attitude_q_GET(pfollow_target);
	for (size_t                 index           = 0; index < item_attitude_q.len; index++)
		some_float                         = vfollow_target_attitude_q_GET(&item_attitude_q, index);
	Vfollow_target_rates        item_rates = pfollow_target_rates_GET(pfollow_target);
	for (size_t                 index      = 0; index < item_rates.len; index++)
		some_float                                = vfollow_target_rates_GET(&item_rates, index);
	Vfollow_target_position_cov item_position_cov = pfollow_target_position_cov_GET(pfollow_target);
	for (size_t                 index             = 0; index < item_position_cov.len; index++)
		some_float = vfollow_target_position_cov_GET(&item_position_cov, index);
	
}


void write_FOLLOW_TARGET(pfollow_target_FOLLOW_TARGET *const pfollow_target) {
	pfollow_target_timestamp_SET(-6813969735828513451L, -843802146753315404L, -5194968082557660480L, -1184730456078284509L, -7329321654516991711L, 9078553312187952046L, 4420406568420584664L, 8508558696613781237L, 992932918285902612L, -8624502737958306252L, pfollow_target);
	pfollow_target_custom_state_SET(-2746019942742852774L, 3715899888098186036L, 2033462150108293642L, 5081272439117982405L, 1649763676945594068L, -1818341029477416433L, 2949940370866239934L, 4008882913087164875L, -1016575785206865171L, -6132875635378238424L, pfollow_target);
	pfollow_target_est_capabilities_SET(-102, -116, -89, -46, 35, 99, 90, 114, 43, -112, pfollow_target);
	pfollow_target_lat_SET(-261458814, 231178607, -234992043, -370941172, 460656969, 616991675, -1169220968, 86528577, 1212600201, 906297974, pfollow_target);
	pfollow_target_lon_SET(869886526, -1879806231, 37160950, 988041083, 956136247, 892937243, -2114104562, 363805351, 425638987, 1170578327, pfollow_target);
	pfollow_target_alt_SET(-7.9732637E37F, -2.0754493E38F, -8.449786E36F, -1.0446733E38F, 4.273283E37F, -2.2182587E38F, 1.529236E38F, 2.7098918E38F, 1.0945721E38F, -1.1528478E38F, pfollow_target);
	pfollow_target_vel_SET(&7.8270833E37F, 9.637203E37F, -2.5647904E38F, -1.4039765E38F, 2.687537E38F, 8.362902E37F, -2.4889545E38F, 1.7910351E38F, 2.1735549E38F, -2.30445E38F, pfollow_target);
	pfollow_target_acc_SET(&-3.1777094E38F, -3.0959147E38F, 3.3088908E37F, 2.0156638E38F, -3.243192E38F, 1.2707529E38F, -2.0486465E38F, 2.937727E37F, 6.2631204E37F, -2.3022458E38F, pfollow_target);
	pfollow_target_attitude_q_SET(&2.8718486E38F, 2.4252046E38F, -7.302716E36F, -3.0962252E38F, -7.414799E37F, -1.3573779E38F, 2.1819757E38F, -2.8422443E37F, 1.7149802E38F, 4.2117329E37F, pfollow_target);
	pfollow_target_rates_SET(&7.2479713E37F, -1.6555816E38F, -2.0290484E38F, 2.909109E38F, 7.946554E37F, -1.1957084E38F, -2.4523354E38F, -2.8116253E38F, 1.7514869E38F, -6.7120416E37F, pfollow_target);
	pfollow_target_position_cov_SET(&-3.2403496E38F, 2.1149147E37F, -1.6666942E38F, -1.7092538E38F, -2.3185011E38F, -8.2997537E37F, 6.390084E37F, 2.2663157E38F, -1.4083585E38F, 1.2234562E38F, pfollow_target);
	
}


void read_HIL_STATE(phil_state_HIL_STATE *const phil_state) {
	int64_t time_usec  = phil_state_time_usec_GET(phil_state);
	float   roll       = phil_state_roll_GET(phil_state);
	float   pitch      = phil_state_pitch_GET(phil_state);
	float   yaw        = phil_state_yaw_GET(phil_state);
	float   rollspeed  = phil_state_rollspeed_GET(phil_state);
	float   pitchspeed = phil_state_pitchspeed_GET(phil_state);
	float   yawspeed   = phil_state_yawspeed_GET(phil_state);
	int32_t lat        = phil_state_lat_GET(phil_state);
	int32_t lon        = phil_state_lon_GET(phil_state);
	int32_t alt        = phil_state_alt_GET(phil_state);
	int16_t vx         = phil_state_vx_GET(phil_state);
	int16_t vy         = phil_state_vy_GET(phil_state);
	int16_t vz         = phil_state_vz_GET(phil_state);
	int16_t xacc       = phil_state_xacc_GET(phil_state);
	int16_t yacc       = phil_state_yacc_GET(phil_state);
	int16_t zacc       = phil_state_zacc_GET(phil_state);
	
}


void write_HIL_STATE(phil_state_HIL_STATE *const phil_state) {
	phil_state_time_usec_SET(-5322040918428179450L, 2375240212523999886L, -6907895179964982956L, -2852813777355790262L, 8758910712952154300L, 2090963623212581971L, 720581169062888520L, 5914518753919682315L, 6683864960201411636L, 381566225343826677L, phil_state);
	phil_state_roll_SET(-1.725287E38F, -2.9734808E38F, -3.044474E38F, 2.517925E37F, -6.84065E37F, 2.378209E38F, -2.4213548E38F, 5.665664E37F, -1.0910871E38F, 2.1167745E38F, phil_state);
	phil_state_pitch_SET(-9.434788E37F, -2.986451E38F, 5.16194E37F, 2.709746E37F, -2.801725E37F, 2.3968902E38F, 1.9791514E38F, 1.1673654E38F, -2.664653E38F, 2.2308338E38F, phil_state);
	phil_state_yaw_SET(-3.3114875E38F, -4.9357984E37F, -1.4985881E38F, 2.2800945E38F, 1.1086238E38F, 1.739511E38F, 2.5249101E38F, -5.029167E37F, -7.044852E37F, -3.3669518E38F, phil_state);
	phil_state_rollspeed_SET(-2.0317326E38F, -1.668289E37F, 3.697873E37F, 3.2268697E38F, 3.2603779E38F, 2.4095052E38F, -1.933507E38F, 1.3746693E38F, 3.0418493E37F, -7.4359274E37F, phil_state);
	phil_state_pitchspeed_SET(2.2552768E38F, -6.0541147E37F, -2.2965338E38F, -2.941023E38F, 2.3761267E38F, -2.4274643E38F, 1.46719E38F, -2.0307801E38F, -3.3721347E37F, 2.224384E38F, phil_state);
	phil_state_yawspeed_SET(-1.5283157E38F, 8.236867E37F, -3.2370742E38F, 2.3270368E38F, -2.5080303E38F, -5.5349314E36F, -2.587955E38F, 2.8168622E38F, -1.848775E38F, 2.7839104E38F, phil_state);
	phil_state_lat_SET(-996797797, -1176056764, 1332597813, -1570057111, -515347151, -19292940, -1306012802, -39664086, -98764689, -1463848235, phil_state);
	phil_state_lon_SET(-122233866, 1613412064, 13462045, 832965966, 376800595, 1352486342, 1186409797, 2079367627, -307705159, 1768822738, phil_state);
	phil_state_alt_SET(-134833994, 851563178, 1952045069, 1288601707, -1303232657, -826944596, -2102505757, -1170621044, 1319719346, -1853465154, phil_state);
	phil_state_vx_SET(-14157, -26396, -27561, -11226, 18697, -30446, -10527, -10472, 25669, 2415, phil_state);
	phil_state_vy_SET(21058, -30291, -32607, -1340, 14460, 22967, -14730, 30952, 31504, -11258, phil_state);
	phil_state_vz_SET(3531, 7600, 30521, -13160, -27834, -20395, 2521, -25703, -24559, 7486, phil_state);
	phil_state_xacc_SET(-18526, -29639, 24448, 5428, -29669, 15335, 19640, 24397, -25441, -10119, phil_state);
	phil_state_yacc_SET(9292, -8710, 20899, -21377, 28162, 27853, -18150, 30393, -31847, -27259, phil_state);
	phil_state_zacc_SET(-24481, 2421, 5783, 32453, -2207, 17861, 21411, -16222, 7790, -16780, phil_state);
	
}


void read_HOME_POSITION(phome_position_HOME_POSITION *const phome_position) {
	int32_t          latitude  = phome_position_latitude_GET(phome_position);
	int32_t          longitude = phome_position_longitude_GET(phome_position);
	int32_t          altitude  = phome_position_altitude_GET(phome_position);
	float            x         = phome_position_x_GET(phome_position);
	float            y         = phome_position_y_GET(phome_position);
	float            z         = phome_position_z_GET(phome_position);
	Vhome_position_q item_q    = phome_position_q_GET(phome_position);
	for (size_t      index     = 0; index < item_q.len; index++)
		some_float              = vhome_position_q_GET(&item_q, index);
	float            approach_x = phome_position_approach_x_GET(phome_position);
	float            approach_y = phome_position_approach_y_GET(phome_position);
	float            approach_z = phome_position_approach_z_GET(phome_position);
	int64_t          item_time_usec;
	if (phome_position_time_usec_GET(phome_position, &item_time_usec)) {
		some_int64_t = item_time_usec;
	}
	
}


void write_HOME_POSITION(phome_position_HOME_POSITION *const phome_position) {
	phome_position_latitude_SET(1842634829, -336172509, 2013234274, -1044907335, -375184795, 476465534, -745180896, -1179357610, 1215473839, -714290127, phome_position);
	phome_position_longitude_SET(55435862, -867812690, 961769552, -618155164, 1099810547, 1893215697, -495729378, 1116380999, 295963106, 2076009344, phome_position);
	phome_position_altitude_SET(1447022252, 2009297296, 1952591576, -407500672, 1941279879, -1227517693, -1074762362, -405844088, 481045805, 152381670, phome_position);
	phome_position_x_SET(9.585767E37F, -2.0670633E38F, -2.615488E38F, 2.3850702E37F, -1.7286844E38F, 7.9100586E37F, 2.5494593E38F, -4.3633127E37F, 2.5309658E38F, -2.2944547E38F, phome_position);
	phome_position_y_SET(2.9307646E37F, 1.4500815E38F, 1.1698085E38F, 1.4613827E38F, 2.869503E38F, -1.9001572E37F, 3.2550588E38F, 4.9242506E37F, -2.8455606E38F, -1.135102E38F, phome_position);
	phome_position_z_SET(-7.1844296E37F, -2.753828E38F, -2.3135648E38F, -1.6282858E38F, 4.324524E37F, 1.7489436E36F, -8.535296E37F, 1.6104753E36F, -2.524014E37F, -2.7842877E38F, phome_position);
	phome_position_q_SET(&2.0259413E38F, -8.966049E37F, -3.566726E37F, 1.656447E38F, 5.2656345E37F, 8.516192E37F, -4.336119E37F, -4.7021445E37F, -3.182969E38F, 3.1709185E38F, phome_position);
	phome_position_approach_x_SET(2.1849362E38F, -1.0099026E38F, -1.8809651E38F, 2.5264162E36F, -7.0130417E37F, -8.979789E37F, 1.792387E38F, 9.545161E37F, 1.4522153E38F, 1.4005728E38F, phome_position);
	phome_position_approach_y_SET(-2.8051193E38F, 1.6935158E38F, 1.4579968E38F, 3.3026065E38F, 9.8126615E36F, -3.174366E38F, 2.908758E37F, -3.3780657E37F, 3.3441294E38F, -2.1345662E38F, phome_position);
	phome_position_approach_z_SET(2.8638726E38F, -2.952628E37F, -3.3228351E38F, -2.5703458E38F, -4.279816E37F, -1.5144883E38F, 2.0217171E38F, -6.489186E37F, -2.0441574E38F, 8.488581E37F, phome_position);
	phome_position_time_usec_SET(-6581039568798942375L, -4051521795241356311L, -9145791805277081229L, 1233973270540642962L, -3183812339304323271L, -7845395717998199497L, -3118417114069421517L, 5437931638045676687L, 1685678448018991698L, -3118922125096275306L, phome_position);
	
}


void read_FENCE_STATUS(pfence_status_FENCE_STATUS *const pfence_status) {
	int16_t        breach_count  = pfence_status_breach_count_GET(pfence_status);
	int32_t        breach_time   = pfence_status_breach_time_GET(pfence_status);
	int8_t         breach_status = pfence_status_breach_status_GET(pfence_status);
	e_FENCE_BREACH item_breach_type;
	if (pfence_status_breach_type_GET(pfence_status, &item_breach_type)) {
		e_FENCE_BREACH_FENCE_BREACH_NONE = item_breach_type;
	}
	
}


void read_REMOTE_LOG_BLOCK_STATUS(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *const premote_log_block_status) {
	int32_t                              seqno            = premote_log_block_status_seqno_GET(premote_log_block_status);
	int8_t                               target_system    = premote_log_block_status_target_system_GET(premote_log_block_status);
	int8_t                               target_component = premote_log_block_status_target_component_GET(premote_log_block_status);
	e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES item_status;
	if (premote_log_block_status_status_GET(premote_log_block_status, &item_status)) {
		e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK = item_status;
	}
	
}


void read_OBSTACLE_DISTANCE(pobstacle_distance_OBSTACLE_DISTANCE *const pobstacle_distance) {
	Vobstacle_distance_distances item_distances = pobstacle_distance_distances_GET(pobstacle_distance);
	for (size_t                  index          = 0; index < item_distances.len; index++)
		some_int16_t                          = vobstacle_distance_distances_GET(&item_distances, index);
	int16_t                      min_distance = pobstacle_distance_min_distance_GET(pobstacle_distance);
	int16_t                      max_distance = pobstacle_distance_max_distance_GET(pobstacle_distance);
	int64_t                      time_usec    = pobstacle_distance_time_usec_GET(pobstacle_distance);
	int8_t                       increment    = pobstacle_distance_increment_GET(pobstacle_distance);
	e_MAV_DISTANCE_SENSOR        item_sensor_type;
	if (pobstacle_distance_sensor_type_GET(pobstacle_distance, &item_sensor_type)) {
		e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER = item_sensor_type;
	}
	
}


void read_GPS2_RAW(pgps2_raw_GPS2_RAW *const pgps2_raw) {
	int16_t        eph                = pgps2_raw_eph_GET(pgps2_raw);
	int16_t        epv                = pgps2_raw_epv_GET(pgps2_raw);
	int16_t        vel                = pgps2_raw_vel_GET(pgps2_raw);
	int16_t        cog                = pgps2_raw_cog_GET(pgps2_raw);
	int32_t        dgps_age           = pgps2_raw_dgps_age_GET(pgps2_raw);
	int64_t        time_usec          = pgps2_raw_time_usec_GET(pgps2_raw);
	int32_t        lat                = pgps2_raw_lat_GET(pgps2_raw);
	int32_t        lon                = pgps2_raw_lon_GET(pgps2_raw);
	int32_t        alt                = pgps2_raw_alt_GET(pgps2_raw);
	int8_t         satellites_visible = pgps2_raw_satellites_visible_GET(pgps2_raw);
	int8_t         dgps_numch         = pgps2_raw_dgps_numch_GET(pgps2_raw);
	e_GPS_FIX_TYPE item_fix_type;
	if (pgps2_raw_fix_type_GET(pgps2_raw, &item_fix_type)) {
		e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS = item_fix_type;
	}
	
}


void write_GPS2_RAW(pgps2_raw_GPS2_RAW *const pgps2_raw) {
	pgps2_raw_eph_SET(-20149, -17147, -21811, 27384, -17274, -25127, -27849, -12074, -19170, 17971, pgps2_raw);
	pgps2_raw_epv_SET(21922, -26231, -27133, -27004, 17703, -15579, -30960, -31559, 15256, -25446, pgps2_raw);
	pgps2_raw_vel_SET(-21460, 9806, -13819, -5036, 17251, 8229, -24484, 9600, -31831, 18044, pgps2_raw);
	pgps2_raw_cog_SET(8727, -14813, -24124, 20955, 12612, -28813, -19368, 6969, -22864, -12926, pgps2_raw);
	pgps2_raw_dgps_age_SET(1985619728, 26180776, 974316798, -1177913478, -1825293393, -1265717342, -271291423, 1018262424, -1381313173, -627484170, pgps2_raw);
	pgps2_raw_time_usec_SET(-7016967065549174597L, 3033255265563738922L, 4447955998116569579L, -1869396481040429729L, 919675498332234682L, -4144562784332285124L, 6556491808549007337L, -7603674216124749684L, 5086034166509081878L, -1903957184149145059L, pgps2_raw);
	pgps2_raw_lat_SET(-993124971, -398406303, 517878333, 217926602, -443856623, -273355169, -1082286253, -108246558, -417817845, -1292777378, pgps2_raw);
	pgps2_raw_lon_SET(85995159, 689568234, -919072131, 41472142, -720570951, -520980666, -1696580616, -609526782, -1019803394, -1390689359, pgps2_raw);
	pgps2_raw_alt_SET(877972507, -2058699308, 1709543516, 664477459, -760828283, 671026036, -902158343, -379729618, -9082554, -810824416, pgps2_raw);
	pgps2_raw_satellites_visible_SET(126, -77, 127, 2, -107, 54, -106, 54, -51, 72, pgps2_raw);
	pgps2_raw_dgps_numch_SET(-34, 50, -119, -92, 28, 82, 53, 46, 121, -11, pgps2_raw);
	pgps2_raw_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, pgps2_raw);
	
}


void read_REQUEST_DATA_STREAM(prequest_data_stream_REQUEST_DATA_STREAM *const prequest_data_stream) {
	int16_t req_message_rate = prequest_data_stream_req_message_rate_GET(prequest_data_stream);
	int8_t  target_system    = prequest_data_stream_target_system_GET(prequest_data_stream);
	int8_t  target_component = prequest_data_stream_target_component_GET(prequest_data_stream);
	int8_t  req_stream_id    = prequest_data_stream_req_stream_id_GET(prequest_data_stream);
	int8_t  start_stop       = prequest_data_stream_start_stop_GET(prequest_data_stream);
	
}


void write_REQUEST_DATA_STREAM(prequest_data_stream_REQUEST_DATA_STREAM *const prequest_data_stream) {
	prequest_data_stream_req_message_rate_SET(-1928, -3331, -32275, -31969, -11755, 23644, 31553, 18732, -703, -25996, prequest_data_stream);
	prequest_data_stream_target_system_SET(71, -31, 90, -61, 8, 97, -56, -84, 109, -20, prequest_data_stream);
	prequest_data_stream_target_component_SET(86, -102, 51, 68, -105, -51, -36, -121, 82, -16, prequest_data_stream);
	prequest_data_stream_req_stream_id_SET(63, -41, -99, 5, 68, 91, -86, 78, -65, -97, prequest_data_stream);
	prequest_data_stream_start_stop_SET(-13, 71, -96, 124, 100, 57, -91, 7, -53, 26, prequest_data_stream);
	
}


void read_MEMORY_VECT(pmemory_vect_MEMORY_VECT *const pmemory_vect) {
	int16_t            address    = pmemory_vect_address_GET(pmemory_vect);
	int8_t             ver        = pmemory_vect_ver_GET(pmemory_vect);
	int8_t             typE       = pmemory_vect_typE_GET(pmemory_vect);
	Vmemory_vect_value item_value = pmemory_vect_value_GET(pmemory_vect);
	for (size_t        index      = 0; index < item_value.len; index++)
		some_int8_t = vmemory_vect_value_GET(&item_value, index);
	
}


void write_MEMORY_VECT(pmemory_vect_MEMORY_VECT *const pmemory_vect) {
	pmemory_vect_address_SET(-11929, 13147, 15951, 21531, 1229, 9234, -1231, -12387, 32355, -719, pmemory_vect);
	pmemory_vect_ver_SET(19, -99, 56, -107, -76, 127, -9, 78, 125, -45, pmemory_vect);
	pmemory_vect_typE_SET(63, -16, -108, -59, -105, 11, 118, 92, 104, -6, pmemory_vect);
	pmemory_vect_value_SET(&-65, -43, -24, 84, -72, 39, 43, -51, -32, -84, pmemory_vect);
	
}


void read_PARAM_EXT_REQUEST_READ(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *const pparam_ext_request_read) {
	int8_t                           target_system    = pparam_ext_request_read_target_system_GET(pparam_ext_request_read);
	int8_t                           target_component = pparam_ext_request_read_target_component_GET(pparam_ext_request_read);
	int16_t                          param_index      = pparam_ext_request_read_param_index_GET(pparam_ext_request_read);
	Vparam_ext_request_read_param_id item_param_id;
	if (pparam_ext_request_read_param_id_GET(pparam_ext_request_read, &item_param_id)) {
		memcpy(some_string, item_param_id.bytes, item_param_id.len);
	}
	
}


void read_HIL_CONTROLS(phil_controls_HIL_CONTROLS *const phil_controls) {
	int64_t    time_usec      = phil_controls_time_usec_GET(phil_controls);
	float      roll_ailerons  = phil_controls_roll_ailerons_GET(phil_controls);
	float      pitch_elevator = phil_controls_pitch_elevator_GET(phil_controls);
	float      yaw_rudder     = phil_controls_yaw_rudder_GET(phil_controls);
	float      throttle       = phil_controls_throttle_GET(phil_controls);
	float      aux1           = phil_controls_aux1_GET(phil_controls);
	float      aux2           = phil_controls_aux2_GET(phil_controls);
	float      aux3           = phil_controls_aux3_GET(phil_controls);
	float      aux4           = phil_controls_aux4_GET(phil_controls);
	int8_t     nav_mode       = phil_controls_nav_mode_GET(phil_controls);
	e_MAV_MODE item_mode;
	if (phil_controls_mode_GET(phil_controls, &item_mode)) {
		e_MAV_MODE_PREFLIGHT = item_mode;
	}
	
}


void write_HIL_CONTROLS(phil_controls_HIL_CONTROLS *const phil_controls) {
	phil_controls_time_usec_SET(-9024514455502075314L, 7847037825013428637L, -44520338121141961L, 1925250679835929399L, 941058062044502403L, -9052915424845145192L, -6107465649923145266L, -1419638333972610870L, 6544712297259327533L, -3020112731039349311L, phil_controls);
	phil_controls_roll_ailerons_SET(-3.163423E38F, 1.6357967E38F, -2.5452012E38F, -3.3833982E38F, -3.120327E38F, 2.077581E38F, -4.7673606E37F, 2.036801E38F, 5.0586206E37F, -2.3611637E38F, phil_controls);
	phil_controls_pitch_elevator_SET(3.2560247E38F, 2.9987689E38F, -2.3436055E38F, -1.3651659E38F, -7.069195E37F, 1.0836036E38F, -6.4208065E37F, -7.5248105E37F, -2.9444826E38F, 1.5063983E38F, phil_controls);
	phil_controls_yaw_rudder_SET(8.307261E37F, 1.7706085E38F, -1.563514E38F, -1.7516812E38F, -1.843575E38F, -1.383007E38F, -1.934382E38F, -1.4683911E38F, 1.9710872E38F, 2.585138E38F, phil_controls);
	phil_controls_throttle_SET(-1.0675397E38F, -1.1991932E38F, 2.679956E38F, 2.6770914E36F, 3.0026426E38F, 2.4967273E38F, -3.0470128E38F, 1.3767303E38F, 2.305674E38F, 5.2282094E37F, phil_controls);
	phil_controls_aux1_SET(-1.5057452E38F, -1.8172197E38F, -1.6324206E38F, -6.682472E37F, 1.8621913E38F, -1.0040446E38F, -2.6422185E38F, 2.9230567E38F, 9.35528E37F, -3.3185535E38F, phil_controls);
	phil_controls_aux2_SET(3.0333694E38F, 1.3393661E38F, 1.7415446E38F, -1.8923744E38F, -8.304948E37F, -1.3248854E38F, 3.0683087E38F, -2.4384494E38F, 3.3160886E38F, -1.0689583E38F, phil_controls);
	phil_controls_aux3_SET(-2.0383373E38F, 2.7777283E38F, -2.4977752E37F, 1.6193489E38F, -1.0755801E38F, 2.1720607E38F, -1.883885E38F, 1.1244844E38F, 9.534767E37F, 3.1547392E37F, phil_controls);
	phil_controls_aux4_SET(1.2260522E38F, 4.3969105E37F, 8.548776E37F, 3.1181566E38F, 2.7180378E38F, -2.9311522E38F, 1.3569034E38F, 3.5477507E37F, 1.531905E38F, 3.1606821E38F, phil_controls);
	phil_controls_nav_mode_SET(-35, -37, -74, -93, -46, -38, -116, -72, 105, -25, phil_controls);
	phil_controls_mode_SET(e_MAV_MODE_GUIDED_ARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, e_MAV_MODE_MANUAL_DISARMED, e_MAV_MODE_MAV_MODE_TEST_DISARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, e_MAV_MODE_GUIDED_ARMED, e_MAV_MODE_MAV_MODE_TEST_ARMED, e_MAV_MODE_STABILIZE_DISARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, e_MAV_MODE_MANUAL_DISARMED, phil_controls);
	
}


void read_HIL_SENSOR(phil_sensor_HIL_SENSOR *const phil_sensor) {
	int32_t fields_updated = phil_sensor_fields_updated_GET(phil_sensor);
	int64_t time_usec      = phil_sensor_time_usec_GET(phil_sensor);
	float   xacc           = phil_sensor_xacc_GET(phil_sensor);
	float   yacc           = phil_sensor_yacc_GET(phil_sensor);
	float   zacc           = phil_sensor_zacc_GET(phil_sensor);
	float   xgyro          = phil_sensor_xgyro_GET(phil_sensor);
	float   ygyro          = phil_sensor_ygyro_GET(phil_sensor);
	float   zgyro          = phil_sensor_zgyro_GET(phil_sensor);
	float   xmag           = phil_sensor_xmag_GET(phil_sensor);
	float   ymag           = phil_sensor_ymag_GET(phil_sensor);
	float   zmag           = phil_sensor_zmag_GET(phil_sensor);
	float   abs_pressure   = phil_sensor_abs_pressure_GET(phil_sensor);
	float   diff_pressure  = phil_sensor_diff_pressure_GET(phil_sensor);
	float   pressure_alt   = phil_sensor_pressure_alt_GET(phil_sensor);
	float   temperature    = phil_sensor_temperature_GET(phil_sensor);
	
}


void write_HIL_SENSOR(phil_sensor_HIL_SENSOR *const phil_sensor) {
	phil_sensor_fields_updated_SET(-1936948842, -1647328130, -229299964, 140938244, 1303317016, -767263903, 1039887645, 606708258, 2067326685, -1016329210, phil_sensor);
	phil_sensor_time_usec_SET(778506450068610556L, 551150941270145380L, -1643648608197863705L, 5137398336061405847L, 6852853942534689338L, 1008456875980997787L, -7977902210348332422L, -3788885298640613246L, -8654495426734347095L, 5575292022337918234L, phil_sensor);
	phil_sensor_xacc_SET(1.0069553E38F, 5.3297254E37F, -5.355096E37F, -1.9721752E37F, 2.4835291E38F, -1.8653465E37F, -2.4160684E38F, 2.2404745E38F, -3.241878E37F, -2.898346E38F, phil_sensor);
	phil_sensor_yacc_SET(3.239484E36F, 2.9245711E38F, -1.0115769E38F, 5.580579E37F, 2.9631724E38F, 1.6812707E38F, -4.6424144E37F, -2.8838575E38F, -3.0340294E38F, -1.956363E38F, phil_sensor);
	phil_sensor_zacc_SET(-3.1478176E38F, -2.8834463E38F, -2.3027579E38F, -2.0722692E38F, 1.1674078E37F, 1.2796957E38F, 2.0650116E37F, 1.0191878E38F, 3.1608235E38F, -2.016485E38F, phil_sensor);
	phil_sensor_xgyro_SET(-1.0351576E38F, 1.6600494E38F, 2.7627143E38F, -5.8399583E37F, -3.5684836E37F, 2.9575309E38F, 2.159582E38F, 1.6917384E38F, 1.7910785E38F, -2.559062E38F, phil_sensor);
	phil_sensor_ygyro_SET(3.429909E37F, -1.8140993E38F, -8.960959E37F, 3.2294235E38F, -6.695611E37F, 1.4933896E38F, -1.2869683E38F, 9.469554E37F, -1.7440815E37F, 8.3150735E37F, phil_sensor);
	phil_sensor_zgyro_SET(-1.3630594E37F, 1.016124E38F, -5.417799E37F, 1.9718492E38F, 2.3560386E38F, 1.2767518E38F, 1.4936522E38F, 3.2736487E38F, -1.3890307E38F, 1.9196923E38F, phil_sensor);
	phil_sensor_xmag_SET(-8.889213E37F, -1.2730214E38F, -1.917962E38F, -2.1313425E38F, -7.82472E37F, 3.0553834E38F, -1.6624139E38F, 1.967629E38F, 3.2295837E38F, 3.0890737E38F, phil_sensor);
	phil_sensor_ymag_SET(1.6261461E38F, 5.991379E37F, 1.6204345E38F, 2.8868998E38F, 1.4892793E38F, -8.360042E37F, 3.392733E38F, -9.005568E37F, 2.6653828E38F, -8.59187E37F, phil_sensor);
	phil_sensor_zmag_SET(-1.4385856E38F, -5.8215936E37F, 7.367798E37F, -3.0068129E38F, -3.3905867E38F, 2.1370372E38F, 1.9659557E38F, -1.6956912E37F, 3.3379254E38F, 1.2451311E38F, phil_sensor);
	phil_sensor_abs_pressure_SET(-5.846095E36F, 2.4754795E38F, 1.1845013E38F, 1.4129744E38F, -1.7976182E38F, -1.8033914E38F, 3.051131E38F, -2.90695E38F, 1.044237E38F, -1.1018864E37F, phil_sensor);
	phil_sensor_diff_pressure_SET(-1.9213458E38F, -3.1898271E38F, -1.179166E38F, -2.0973368E38F, -3.5251746E37F, -3.3196433E38F, -2.5861105E38F, 2.9469587E38F, -2.3244911E38F, -5.4011134E37F, phil_sensor);
	phil_sensor_pressure_alt_SET(-3.1051164E38F, 2.5682366E38F, 5.353848E37F, 2.8912942E38F, -6.1009113E37F, 6.4508323E35F, -2.5766808E38F, 1.8531042E38F, 1.8026586E38F, -3.2315884E38F, phil_sensor);
	phil_sensor_temperature_SET(1.8878116E38F, -2.176097E38F, 4.4416276E37F, 2.0777448E38F, -1.6037761E38F, 2.84861E38F, -1.2777324E38F, 1.8779282E38F, -1.2507352E38F, 5.501226E36F, phil_sensor);
	
}


void read_SETUP_SIGNING(psetup_signing_SETUP_SIGNING *const psetup_signing) {
	int64_t                   initial_timestamp = psetup_signing_initial_timestamp_GET(psetup_signing);
	int8_t                    target_system     = psetup_signing_target_system_GET(psetup_signing);
	int8_t                    target_component  = psetup_signing_target_component_GET(psetup_signing);
	Vsetup_signing_secret_key item_secret_key   = psetup_signing_secret_key_GET(psetup_signing);
	for (size_t               index             = 0; index < item_secret_key.len; index++)
		some_int8_t = vsetup_signing_secret_key_GET(&item_secret_key, index);
	
}


void write_SETUP_SIGNING(psetup_signing_SETUP_SIGNING *const psetup_signing) {
	psetup_signing_initial_timestamp_SET(-3020052902315459872L, 4245004322084120768L, 4075641027521922415L, -8827317314381325458L, 2226197321133434876L, 6130616801219781965L, -1348090477491223271L, 3975213701745827506L, 4390418579327366601L, 6936166315219562120L, psetup_signing);
	psetup_signing_target_system_SET(39, -96, -124, -6, 48, 48, -114, -72, -22, -89, psetup_signing);
	psetup_signing_target_component_SET(40, -83, 66, 82, -101, -98, -58, 38, -47, -28, psetup_signing);
	psetup_signing_secret_key_SET(&-15, -95, -73, 117, 76, 76, -47, -45, -81, 59, psetup_signing);
	
}


void read_GPS_RTK(pgps_rtk_GPS_RTK *const pgps_rtk) {
	int16_t wn                    = pgps_rtk_wn_GET(pgps_rtk);
	int32_t time_last_baseline_ms = pgps_rtk_time_last_baseline_ms_GET(pgps_rtk);
	int32_t tow                   = pgps_rtk_tow_GET(pgps_rtk);
	int32_t accuracy              = pgps_rtk_accuracy_GET(pgps_rtk);
	int8_t  rtk_receiver_id       = pgps_rtk_rtk_receiver_id_GET(pgps_rtk);
	int8_t  rtk_health            = pgps_rtk_rtk_health_GET(pgps_rtk);
	int8_t  rtk_rate              = pgps_rtk_rtk_rate_GET(pgps_rtk);
	int8_t  nsats                 = pgps_rtk_nsats_GET(pgps_rtk);
	int8_t  baseline_coords_type  = pgps_rtk_baseline_coords_type_GET(pgps_rtk);
	int32_t baseline_a_mm         = pgps_rtk_baseline_a_mm_GET(pgps_rtk);
	int32_t baseline_b_mm         = pgps_rtk_baseline_b_mm_GET(pgps_rtk);
	int32_t baseline_c_mm         = pgps_rtk_baseline_c_mm_GET(pgps_rtk);
	int32_t iar_num_hypotheses    = pgps_rtk_iar_num_hypotheses_GET(pgps_rtk);
	
}


void write_GPS_RTK(pgps_rtk_GPS_RTK *const pgps_rtk) {
	pgps_rtk_wn_SET(18128, -1811, 5649, 16591, -21853, 13589, 21694, -12012, 1410, 14568, pgps_rtk);
	pgps_rtk_time_last_baseline_ms_SET(1408927709, -1159117222, -2111681130, 2144804856, 13661658, 1474455354, 945934552, 934864063, 1826927745, -322609058, pgps_rtk);
	pgps_rtk_tow_SET(1555791315, 1468696485, 2122387483, -671695640, -1883719579, 1525753055, 294932133, 1158311138, 1366460683, -1934715876, pgps_rtk);
	pgps_rtk_accuracy_SET(1143661130, -1574230825, -422099376, -106338669, 1706644299, -810034672, -1558076746, -1906725584, -697721699, 691891118, pgps_rtk);
	pgps_rtk_rtk_receiver_id_SET(28, -12, 103, -38, 116, 49, -63, 2, 54, -10, pgps_rtk);
	pgps_rtk_rtk_health_SET(99, 70, 99, 57, -85, 58, -26, 82, -82, 15, pgps_rtk);
	pgps_rtk_rtk_rate_SET(99, 117, 113, 66, -124, -100, 121, -19, 31, 81, pgps_rtk);
	pgps_rtk_nsats_SET(5, 70, -20, -85, -32, -22, -61, -125, 105, 90, pgps_rtk);
	pgps_rtk_baseline_coords_type_SET(-23, 87, -35, -115, 3, -124, 110, 20, 9, -20, pgps_rtk);
	pgps_rtk_baseline_a_mm_SET(-1869022947, 2092336899, -1641826068, -1691456961, -1673258702, -832750257, 1938984390, 1377934733, 683345649, 1183539171, pgps_rtk);
	pgps_rtk_baseline_b_mm_SET(1623259041, -1400109973, -786854109, 902380684, 822068213, -642823731, 417219391, -182763789, 61437671, 1939229770, pgps_rtk);
	pgps_rtk_baseline_c_mm_SET(1652085685, -2071172840, 1615586068, -2118857528, -1766111737, -77524804, -1703963619, 40471318, 1271433320, -628123301, pgps_rtk);
	pgps_rtk_iar_num_hypotheses_SET(2088115457, -368897511, -2111776430, -1579805184, -1834703552, -1146942894, 1358663310, -502251609, 1721439454, -273679763, pgps_rtk);
	
}


void read_PARAM_REQUEST_LIST(pparam_request_list_PARAM_REQUEST_LIST *const pparam_request_list) {
	int8_t target_system    = pparam_request_list_target_system_GET(pparam_request_list);
	int8_t target_component = pparam_request_list_target_component_GET(pparam_request_list);
	
}


void write_PARAM_REQUEST_LIST(pparam_request_list_PARAM_REQUEST_LIST *const pparam_request_list) {
	pparam_request_list_target_system_SET(-24, -81, -13, 103, 119, 127, -76, 28, 63, 37, pparam_request_list);
	pparam_request_list_target_component_SET(36, -72, -99, -67, 85, 26, 2, -88, 43, -85, pparam_request_list);
	
}


void read_UAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const puavionix_adsb_out_cfg) {
	int16_t                         stallSpeed = puavionix_adsb_out_cfg_stallSpeed_GET(puavionix_adsb_out_cfg);
	int32_t                         ICAO       = puavionix_adsb_out_cfg_ICAO_GET(puavionix_adsb_out_cfg);
	Vuavionix_adsb_out_cfg_callsign item_callsign;
	if (puavionix_adsb_out_cfg_callsign_GET(puavionix_adsb_out_cfg, &item_callsign)) {
		memcpy(some_string, item_callsign.bytes, item_callsign.len);
	}
	e_ADSB_EMITTER_TYPE item_emitterType;
	if (puavionix_adsb_out_cfg_emitterType_GET(puavionix_adsb_out_cfg, &item_emitterType)) {
		e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO = item_emitterType;
	}
	e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE item_aircraftSize;
	if (puavionix_adsb_out_cfg_aircraftSize_GET(puavionix_adsb_out_cfg, &item_aircraftSize)) {
		e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA = item_aircraftSize;
	}
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT item_gpsOffsetLat;
	if (puavionix_adsb_out_cfg_gpsOffsetLat_GET(puavionix_adsb_out_cfg, &item_gpsOffsetLat)) {
		e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA = item_gpsOffsetLat;
	}
	e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON item_gpsOffsetLon;
	if (puavionix_adsb_out_cfg_gpsOffsetLon_GET(puavionix_adsb_out_cfg, &item_gpsOffsetLon)) {
		e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA = item_gpsOffsetLon;
	}
	e_UAVIONIX_ADSB_OUT_RF_SELECT item_rfSelect;
	if (puavionix_adsb_out_cfg_rfSelect_GET(puavionix_adsb_out_cfg, &item_rfSelect)) {
		e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY = item_rfSelect;
	}
	
}


void read_LANDING_TARGET(planding_target_LANDING_TARGET *const planding_target) {
	int64_t     time_usec  = planding_target_time_usec_GET(planding_target);
	int8_t      target_num = planding_target_target_num_GET(planding_target);
	float       angle_x    = planding_target_angle_x_GET(planding_target);
	float       angle_y    = planding_target_angle_y_GET(planding_target);
	float       distance   = planding_target_distance_GET(planding_target);
	float       size_x     = planding_target_size_x_GET(planding_target);
	float       size_y     = planding_target_size_y_GET(planding_target);
	e_MAV_FRAME item_frame;
	if (planding_target_frame_GET(planding_target, &item_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_frame;
	}
	float item_x;
	if (planding_target_x_GET(planding_target, &item_x)) {
		some_float = item_x;
	}
	float item_y;
	if (planding_target_y_GET(planding_target, &item_y)) {
		some_float = item_y;
	}
	float item_z;
	if (planding_target_z_GET(planding_target, &item_z)) {
		some_float = item_z;
	}
	planding_target_q_d0 (planding_target) {
				some_float = vlanding_target_q_GET(&item_q);
			}
	e_LANDING_TARGET_TYPE item_typE;
	if (planding_target_typE_GET(planding_target, &item_typE)) {
		e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON = item_typE;
	}
	int8_t item_position_valid;
	if (planding_target_position_valid_GET(planding_target, &item_position_valid)) {
		some_int8_t = item_position_valid;
	}
	
}


void write_LANDING_TARGET(planding_target_LANDING_TARGET *const planding_target) {
	planding_target_time_usec_SET(-2167299582140316696L, 3066156590651639479L, 457939932171376964L, -8238837168879354783L, 3505183553097799241L, 2471830627992451534L, -723947244409826817L, 8750797881470356098L, -613044189892724744L, 676878019680983096L, planding_target);
	planding_target_target_num_SET(-28, 117, 38, 31, -32, -124, -53, -39, 21, 101, planding_target);
	planding_target_angle_x_SET(3.3798676E38F, 2.9680753E37F, -2.6945068E38F, 3.046887E38F, -1.4909848E38F, -1.9322857E38F, -9.935562E37F, 7.4144113E37F, -2.2701858E38F, -3.3520815E38F, planding_target);
	planding_target_angle_y_SET(-1.1868182E38F, 2.4023727E38F, 3.3146538E38F, -3.0153096E38F, -1.8498876E38F, -1.464743E38F, 3.0649853E38F, 1.1347339E38F, -3.1869452E38F, -6.7208183E37F, planding_target);
	planding_target_distance_SET(1.2962194E38F, 1.3194755E38F, -2.7469214E38F, -2.7024066E38F, -3.3234876E38F, -1.6041977E38F, -1.1388059E38F, -1.7035788E38F, -2.0212082E38F, 1.6864074E38F, planding_target);
	planding_target_size_x_SET(-1.2189503E37F, 1.57505E38F, -1.9719305E38F, 3.132048E38F, 1.4470607E38F, 9.557047E37F, 2.2450222E38F, -2.011798E38F, 2.4989515E38F, -3.196533E38F, planding_target);
	planding_target_size_y_SET(-1.4540758E38F, -1.4619835E38F, 2.3729093E38F, -1.7512015E38F, 1.5440907E38F, 1.2031656E38F, 9.985899E37F, 2.8538403E38F, 2.8834761E38F, -4.0253933E37F, planding_target);
	planding_target_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_LOCAL_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_LOCAL_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, planding_target);
	planding_target_x_SET(3.3870553E38F, 9.26825E37F, -2.5742544E38F, 2.3601296E37F, 1.5732919E38F, -3.2387183E38F, -4.9529786E37F, -2.979911E38F, -1.9584656E38F, -3.2733892E38F, planding_target);
	planding_target_y_SET(-1.796175E38F, 7.4027317E37F, -1.7939925E38F, -3.308673E37F, -1.994318E38F, -2.1854588E37F, -5.816569E37F, -1.7276333E38F, 5.0116044E37F, -3.1416867E37F, planding_target);
	planding_target_z_SET(-1.4832836E38F, -5.752275E37F, -9.959114E37F, 1.6191172E38F, 2.9889185E38F, -1.2913946E38F, 2.3398394E38F, 8.2143176E37F, -1.366922E38F, 8.881742E37F, planding_target);
	for (size_t d0 = 0; d0 < Planding_target_q_D0; d0++) {
		planding_target_q_SET(-1.8591935E37F, -9.789886E37F, -2.0800712E38F, 1.2120041E38F, -1.3741056E38F, 3.303246E38F, -1.6832597E38F, 1.9677144E38F, -2.9052231E38F, 1.1303897E38F, planding_target, d0);
	}
	planding_target_typE_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, planding_target);
	planding_target_position_valid_SET(67, -111, 17, -76, 117, -89, 86, -127, 45, 117, planding_target);
	
}


void read_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *const pset_actuator_control_target) {
	int64_t                               time_usec        = pset_actuator_control_target_time_usec_GET(pset_actuator_control_target);
	int8_t                                group_mlx        = pset_actuator_control_target_group_mlx_GET(pset_actuator_control_target);
	int8_t                                target_system    = pset_actuator_control_target_target_system_GET(pset_actuator_control_target);
	int8_t                                target_component = pset_actuator_control_target_target_component_GET(pset_actuator_control_target);
	Vset_actuator_control_target_controls item_controls    = pset_actuator_control_target_controls_GET(pset_actuator_control_target);
	for (size_t                           index            = 0; index < item_controls.len; index++)
		some_float = vset_actuator_control_target_controls_GET(&item_controls, index);
	
}


void write_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *const pset_actuator_control_target) {
	pset_actuator_control_target_time_usec_SET(-3022902370662435777L, -5694286988124552281L, -1959282174734870245L, 4066774914953675275L, -6934781917017588969L, 3654708723701450132L, 6499014970175011368L, 3052730566823567239L, 8026908310734087507L, -4592133711439075331L, pset_actuator_control_target);
	pset_actuator_control_target_group_mlx_SET(-21, 65, -98, -13, 82, -19, -36, -41, 122, 56, pset_actuator_control_target);
	pset_actuator_control_target_target_system_SET(78, -105, 62, 22, -119, 121, 127, -62, -31, -82, pset_actuator_control_target);
	pset_actuator_control_target_target_component_SET(40, 120, 74, -27, -111, 45, 73, 22, 5, 67, pset_actuator_control_target);
	pset_actuator_control_target_controls_SET(&-3.7021206E37F, 1.593693E37F, -1.5812644E38F, 4.9888957E37F, 2.5909687E38F, -5.963739E37F, 7.290714E37F, 1.1979893E38F, -1.8938578E38F, -1.9745865E38F, pset_actuator_control_target);
	
}


void read_CONTROL_SYSTEM_STATE(pcontrol_system_state_CONTROL_SYSTEM_STATE *const pcontrol_system_state) {
	int64_t                            time_usec         = pcontrol_system_state_time_usec_GET(pcontrol_system_state);
	float                              x_acc             = pcontrol_system_state_x_acc_GET(pcontrol_system_state);
	float                              y_acc             = pcontrol_system_state_y_acc_GET(pcontrol_system_state);
	float                              z_acc             = pcontrol_system_state_z_acc_GET(pcontrol_system_state);
	float                              x_vel             = pcontrol_system_state_x_vel_GET(pcontrol_system_state);
	float                              y_vel             = pcontrol_system_state_y_vel_GET(pcontrol_system_state);
	float                              z_vel             = pcontrol_system_state_z_vel_GET(pcontrol_system_state);
	float                              x_pos             = pcontrol_system_state_x_pos_GET(pcontrol_system_state);
	float                              y_pos             = pcontrol_system_state_y_pos_GET(pcontrol_system_state);
	float                              z_pos             = pcontrol_system_state_z_pos_GET(pcontrol_system_state);
	float                              airspeed          = pcontrol_system_state_airspeed_GET(pcontrol_system_state);
	Vcontrol_system_state_vel_variance item_vel_variance = pcontrol_system_state_vel_variance_GET(pcontrol_system_state);
	for (size_t                        index             = 0; index < item_vel_variance.len; index++)
		some_float                                       = vcontrol_system_state_vel_variance_GET(&item_vel_variance, index);
	Vcontrol_system_state_pos_variance item_pos_variance = pcontrol_system_state_pos_variance_GET(pcontrol_system_state);
	for (size_t                        index             = 0; index < item_pos_variance.len; index++)
		some_float                            = vcontrol_system_state_pos_variance_GET(&item_pos_variance, index);
	Vcontrol_system_state_q            item_q = pcontrol_system_state_q_GET(pcontrol_system_state);
	for (size_t                        index  = 0; index < item_q.len; index++)
		some_float                                = vcontrol_system_state_q_GET(&item_q, index);
	float                              roll_rate  = pcontrol_system_state_roll_rate_GET(pcontrol_system_state);
	float                              pitch_rate = pcontrol_system_state_pitch_rate_GET(pcontrol_system_state);
	float                              yaw_rate   = pcontrol_system_state_yaw_rate_GET(pcontrol_system_state);
	
}


void write_CONTROL_SYSTEM_STATE(pcontrol_system_state_CONTROL_SYSTEM_STATE *const pcontrol_system_state) {
	pcontrol_system_state_time_usec_SET(-6169855763730040752L, -7152533246725427977L, 5611822594888843271L, 8269709051787824656L, -6495972024667251029L, -1193352022017618279L, 5306985109765833256L, -2829285986929481408L, -3343668230401789300L, -2222109144695436466L, pcontrol_system_state);
	pcontrol_system_state_x_acc_SET(2.1949763E37F, -2.8768917E38F, -1.1610611E38F, 1.0518445E38F, 2.2195027E38F, -2.176308E38F, -2.4897842E38F, -1.2293657E38F, -2.195228E38F, 4.878751E37F, pcontrol_system_state);
	pcontrol_system_state_y_acc_SET(3.1129673E38F, -2.6285617E38F, 2.6545528E38F, 2.6616158E38F, 2.3587862E38F, -8.593875E37F, 1.3264147E38F, -2.1754582E38F, -3.0531233E38F, -1.0530883E38F, pcontrol_system_state);
	pcontrol_system_state_z_acc_SET(1.969787E38F, 2.9856628E38F, 1.2509245E38F, 3.2272762E38F, 2.1557903E38F, 1.6563106E38F, -3.0622984E38F, 2.5041762E38F, 4.3137734E37F, 2.679235E38F, pcontrol_system_state);
	pcontrol_system_state_x_vel_SET(1.9441223E38F, 1.0129485E37F, -1.8395131E38F, 8.0622735E37F, -3.0214366E37F, 1.847504E38F, 3.0121534E38F, 1.72496E38F, -1.336456E38F, -6.061208E37F, pcontrol_system_state);
	pcontrol_system_state_y_vel_SET(-2.9276886E38F, -3.228085E37F, 1.9287103E38F, -2.2604743E38F, -7.905353E37F, 1.5347558E38F, 1.7712288E38F, 1.3299558E38F, -2.7188996E37F, -1.7124182E36F, pcontrol_system_state);
	pcontrol_system_state_z_vel_SET(-1.3266399E38F, -1.3654336E37F, 3.1643376E38F, 1.1521033E38F, 2.8520877E38F, 1.3709014E38F, -3.2273372E38F, -2.0660492E38F, 6.774789E37F, 2.4715595E38F, pcontrol_system_state);
	pcontrol_system_state_x_pos_SET(1.8074564E38F, 3.1925774E38F, 9.684555E37F, 1.1708642E38F, -1.3191027E38F, -3.3009565E38F, -2.88428E38F, -9.739331E37F, 2.502898E38F, 1.9876845E38F, pcontrol_system_state);
	pcontrol_system_state_y_pos_SET(5.925599E37F, -4.0701155E37F, -1.6861566E38F, -4.4614907E37F, 8.409292E37F, -6.8868536E37F, -3.21678E38F, 2.8358226E38F, 5.1862405E37F, -1.6649015E38F, pcontrol_system_state);
	pcontrol_system_state_z_pos_SET(3.3384747E38F, 3.2594167E38F, 1.2256133E38F, 1.7137042E38F, 4.091697E37F, 2.2541941E38F, 1.3873826E38F, 1.6327873E38F, -3.3722594E37F, -5.4070257E37F, pcontrol_system_state);
	pcontrol_system_state_airspeed_SET(-6.716432E37F, 2.6118515E38F, -5.230037E37F, 2.3276298E38F, -2.6294055E38F, -2.7574824E38F, 5.6991365E37F, 1.2187929E38F, -7.737494E37F, -9.893218E37F, pcontrol_system_state);
	pcontrol_system_state_vel_variance_SET(&-3.391953E38F, 1.9788872E38F, -4.0666726E37F, 2.7921775E38F, 2.3298378E38F, -2.9659307E37F, 3.0850902E38F, -5.9705846E37F, -1.589564E38F, 2.6546697E38F, pcontrol_system_state);
	pcontrol_system_state_pos_variance_SET(&-3.0321357E38F, 2.763058E38F, -2.8468793E37F, -1.7902642E38F, -3.3684557E38F, 5.695674E37F, 3.1328144E37F, -7.9455924E37F, -2.4711465E38F, 1.7544589E38F, pcontrol_system_state);
	pcontrol_system_state_q_SET(&-4.869131E37F, -1.4122106E37F, -5.0874627E37F, -3.1906755E38F, -1.4099412E38F, -3.001343E38F, -2.2371471E38F, -2.8731903E38F, 1.6481843E38F, -1.8462684E38F, pcontrol_system_state);
	pcontrol_system_state_roll_rate_SET(1.9892397E38F, 2.0447121E38F, 1.2383867E38F, -5.2427037E37F, -3.6310124E36F, -2.667933E38F, -1.9072014E38F, 3.260573E38F, 4.1962434E37F, 2.4152762E38F, pcontrol_system_state);
	pcontrol_system_state_pitch_rate_SET(-2.5246517E38F, -2.2406887E38F, 1.5344346E38F, 3.3275098E38F, 2.024865E38F, 1.0245803E37F, -2.2493977E38F, 9.769607E37F, 9.658146E37F, 3.0345866E38F, pcontrol_system_state);
	pcontrol_system_state_yaw_rate_SET(-4.519706E37F, -2.8693945E38F, -2.721976E36F, -2.8078118E38F, -2.6296862E38F, 2.6701884E38F, 1.6558226E37F, 2.6139452E38F, 1.8976192E38F, -9.026761E37F, pcontrol_system_state);
	
}


void read_SET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *const pset_position_target_global_int) {
	int16_t     type_mask        = pset_position_target_global_int_type_mask_GET(pset_position_target_global_int);
	int32_t     time_boot_ms     = pset_position_target_global_int_time_boot_ms_GET(pset_position_target_global_int);
	int8_t      target_system    = pset_position_target_global_int_target_system_GET(pset_position_target_global_int);
	int8_t      target_component = pset_position_target_global_int_target_component_GET(pset_position_target_global_int);
	int32_t     lat_int          = pset_position_target_global_int_lat_int_GET(pset_position_target_global_int);
	int32_t     lon_int          = pset_position_target_global_int_lon_int_GET(pset_position_target_global_int);
	float       alt              = pset_position_target_global_int_alt_GET(pset_position_target_global_int);
	float       vx               = pset_position_target_global_int_vx_GET(pset_position_target_global_int);
	float       vy               = pset_position_target_global_int_vy_GET(pset_position_target_global_int);
	float       vz               = pset_position_target_global_int_vz_GET(pset_position_target_global_int);
	float       afx              = pset_position_target_global_int_afx_GET(pset_position_target_global_int);
	float       afy              = pset_position_target_global_int_afy_GET(pset_position_target_global_int);
	float       afz              = pset_position_target_global_int_afz_GET(pset_position_target_global_int);
	float       yaw              = pset_position_target_global_int_yaw_GET(pset_position_target_global_int);
	float       yaw_rate         = pset_position_target_global_int_yaw_rate_GET(pset_position_target_global_int);
	e_MAV_FRAME item_coordinate_frame;
	if (pset_position_target_global_int_coordinate_frame_GET(pset_position_target_global_int, &item_coordinate_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_coordinate_frame;
	}
	
}


void write_SET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *const pset_position_target_global_int) {
	pset_position_target_global_int_type_mask_SET(-21001, -8836, -32020, 21230, 9466, 18486, 21414, -8265, 1267, 2299, pset_position_target_global_int);
	pset_position_target_global_int_time_boot_ms_SET(364917776, -1143046586, 1915952835, 2655751, 94586757, -840907835, -924808969, 687096640, -1627632598, -1920472055, pset_position_target_global_int);
	pset_position_target_global_int_target_system_SET(13, 67, 40, -101, 123, -27, -69, -52, 101, -22, pset_position_target_global_int);
	pset_position_target_global_int_target_component_SET(24, -56, -123, -62, -10, 46, -46, -54, -58, -15, pset_position_target_global_int);
	pset_position_target_global_int_lat_int_SET(868953227, 1626143114, -424370708, -1174240669, 2142227850, -385239043, -285056356, 1731172064, -17637048, -179526357, pset_position_target_global_int);
	pset_position_target_global_int_lon_int_SET(1972294005, -1582310638, -572628467, 251707737, -897660447, 1454180054, 1666204193, 2124234179, 501130696, -1222310193, pset_position_target_global_int);
	pset_position_target_global_int_alt_SET(-6.399168E36F, -3.0294413E38F, -9.535219E37F, -2.7601066E37F, 1.5541025E38F, -2.2832594E38F, 1.4277512E38F, 2.8132795E38F, -9.559956E37F, 1.3720675E38F, pset_position_target_global_int);
	pset_position_target_global_int_vx_SET(1.5371989E38F, -3.2317438E38F, -2.80342E38F, 1.7766204E38F, -2.9804565E38F, 6.7384696E37F, -9.963644E37F, 1.6305891E38F, 1.5368656E38F, -3.8899398E37F, pset_position_target_global_int);
	pset_position_target_global_int_vy_SET(1.8207935E38F, 6.650443E37F, 1.8190768E38F, -1.0668902E38F, -2.8170304E38F, -2.0850321E38F, -2.3206423E36F, 3.725001E37F, 5.510793E37F, 2.6432059E38F, pset_position_target_global_int);
	pset_position_target_global_int_vz_SET(1.5975741E38F, -1.2882312E38F, 1.2825539E38F, 1.0158487E37F, 1.9764529E38F, 2.7470751E38F, 1.4024472E38F, -2.8182392E38F, -3.2056926E37F, 2.8395357E38F, pset_position_target_global_int);
	pset_position_target_global_int_afx_SET(-1.7262947E38F, -3.165154E38F, 1.8973494E38F, -2.4527792E38F, -1.4124397E38F, -9.6928E37F, -1.1628273E38F, -1.371987E38F, 2.1887864E38F, -3.116871E38F, pset_position_target_global_int);
	pset_position_target_global_int_afy_SET(-3.126143E38F, -1.70498E38F, 1.4883057E38F, -2.7735394E38F, 8.589785E37F, -2.495463E38F, -2.3676971E38F, -3.161212E38F, 8.633102E37F, -9.8475E36F, pset_position_target_global_int);
	pset_position_target_global_int_afz_SET(1.3631041E38F, -8.2710485E37F, 2.694338E38F, 1.02395967E37F, -1.2892313E38F, 2.4934665E38F, 2.2903146E38F, 1.6848529E38F, -3.1142652E38F, 1.8767523E38F, pset_position_target_global_int);
	pset_position_target_global_int_yaw_SET(-1.3213234E38F, 2.8467262E37F, -3.37723E38F, -8.3581843E37F, 7.258249E37F, 2.181929E38F, -1.1048713E38F, 2.352389E38F, -6.951264E37F, -2.5742706E35F, pset_position_target_global_int);
	pset_position_target_global_int_yaw_rate_SET(-1.5154072E38F, -1.5627345E38F, -2.2045971E38F, 1.7736042E38F, -2.5316244E38F, 2.5747955E38F, 9.223464E37F, 1.953346E38F, 2.558347E38F, 7.54647E37F, pset_position_target_global_int);
	pset_position_target_global_int_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_MISSION, pset_position_target_global_int);
	
}


void read_DATA32(pdata32_DATA32 *const pdata32) {
	int8_t       typE      = pdata32_typE_GET(pdata32);
	int8_t       len       = pdata32_len_GET(pdata32);
	Vdata32_daTa item_daTa = pdata32_daTa_GET(pdata32);
	for (size_t  index     = 0; index < item_daTa.len; index++)
		some_int8_t = vdata32_daTa_GET(&item_daTa, index);
	
}


void read_PING33(pping33_PING33 *const pping33) {
	pping33_TTTT_d0_d1_d2 {
				int32_t TTTT = pping33_TTTT_GET(pping33, d0, d1, d2);
			}
	int64_t field     = pping33_field_GET(pping33);
	int8_t  bit_field = pping33_bit_field_GET(pping33);
	pping33_field6_d0_d1_d2 {
				int32_t field6 = pping33_field6_GET(pping33, d0, d1, d2);
			}
	bool testBOOL2    = pping33_testBOOL2_GET(pping33);
	bool testBOOL3    = pping33_testBOOL3_GET(pping33);
	bool item_testBOOL;
	if (pping33_testBOOL_GET(pping33, &item_testBOOL)) {
		some_bool = item_testBOOL;
	}
	int64_t item_seq;
	if (pping33_seq_GET(pping33, &item_seq)) {
		some_int64_t = item_seq;
	}
	
	pping33_field1_d0_d1_d2 (pping33) {
						some_int32_t = qping33_field1_GET(fld_field1, d0, d1, d2);
					}
	pping33_field12_d0_d1_d2 (pping33) {
						some_int32_t = qping33_field12_GET(fld_field12, d0, d1, d2);
					}
	pping33_field13_d0_d1_d2 (pping33) {
						some_int32_t = vping33_field13_GET(&item_field13);
					}
	int32_t item_WWWWWWWW;
	if (pping33_WWWWWWWW_GET(pping33, &item_WWWWWWWW)) {
		some_int32_t = item_WWWWWWWW;
	}
	int8_t item_bit_field2;
	if (pping33_bit_field2_GET(pping33, &item_bit_field2)) {
		some_int8_t = item_bit_field2;
	}
	pping33_Field_Bits_d0_d1_d2 (pping33) {
						some_int8_t = qping33_Field_Bits_GET(fld_Field_Bits, d0, d1, d2);
					}
	
	pping33_SparseFixAllBits_d0_d1_d2 (pping33) {
						some_int8_t = vping33_SparseFixAllBits_GET(&item_SparseFixAllBits);
					}
	
	pping33_FixAllBits_d0_d1_d2 (pping33) {
						some_int8_t = qping33_FixAllBits_GET(fld_FixAllBits, d0, d1, d2);
					}
	
	pping33_VarAllBits_d0_d1_d2 (pping33) {
						some_int8_t = qping33_VarAllBits_GET(fld_VarAllBits, d0, d1, d2);
					}
	
	pping33_SparseVarAllBits_d0_d1_d2 (pping33) {
						some_int8_t = vping33_SparseVarAllBits_GET(&item_SparseVarAllBits);
					}
	
	pping33_VarEachBits_d0_d1_d2 (pping33) {
						some_int8_t = qping33_VarEachBits_GET(fld_VarEachBits, d0, d1, d2);
					}
	
	pping33_SparsVarEachBits_d0_d1_d2 (pping33) {
						some_int16_t = vping33_SparsVarEachBits_GET(&item_SparsVarEachBits);
					}
	bool item_testBOOLX;
	if (pping33_testBOOLX_GET(pping33, &item_testBOOLX)) {
		some_bool = item_testBOOLX;
	}
	bool item_testBOOL2X;
	if (pping33_testBOOL2X_GET(pping33, &item_testBOOL2X)) {
		some_bool = item_testBOOL2X;
	}
	bool item_testBOOL3X;
	if (pping33_testBOOL3X_GET(pping33, &item_testBOOL3X)) {
		some_bool = item_testBOOL3X;
	}
	e_MAV_MODE item_MMMMMM;
	if (pping33_MMMMMM_GET(pping33, &item_MMMMMM)) {
		e_MAV_MODE_PREFLIGHT = item_MMMMMM;
	}
	
	pping33_field44_d0_d1_d2 (pping33) {
						some_int32_t = qping33_field44_GET(fld_field44, d0, d1, d2);
					}
	
	pping33_field634_d0_d1_d2 (pping33) {
						some_int32_t = qping33_field634_GET(fld_field634, d0, d1, d2);
					}
	
	pping33_field33344_d0_d1_d2 (pping33) {
						some_int32_t = vping33_field33344_GET(&item_field33344);
					}
	
	pping33_field333634_d0_d1_d2 (pping33) {
						some_int32_t = vping33_field333634_GET(&item_field333634);
					}
	pping33_field___d0_d1_d2 (pping33) {
						some_int32_t = vping33_field___GET(&item_field__);
					}
	
	pping33_field63_d0_d1_d2 (pping33) {
						some_int32_t = qping33_field63_GET(fld_field63, d0, d1, d2);
					}
	pping33_uid2_d0 (pping33) {
				some_int8_t = vping33_uid2_GET(&item_uid2);
			}
	pping33_field2_d0_d1_d2 (pping33) {
						some_int32_t = vping33_field2_GET(&item_field2);
					}
	pping33_field4_d0_d1_d2 (pping33) {
						some_int32_t = vping33_field4_GET(&item_field4);
					}
	Vping33_stringtest1 item_stringtest1;
	if (pping33_stringtest1_GET(pping33, &item_stringtest1)) {
		memcpy(some_string, item_stringtest1.bytes, item_stringtest1.len);
	}
	
	pping33_stringtest2_d0_d1_d2 (pping33) {
						memcpy(some_string, item_stringtest2.bytes, item_stringtest2.len);
					}
	Vping33_stringtest3 item_stringtest3;
	if (pping33_stringtest3_GET(pping33, &item_stringtest3)) {
		memcpy(some_string, item_stringtest3.bytes, item_stringtest3.len);
	}
	Vping33_stringtest4 item_stringtest4;
	if (pping33_stringtest4_GET(pping33, &item_stringtest4)) {
		memcpy(some_string, item_stringtest4.bytes, item_stringtest4.len);
	}
	
}


void write_PING33(pping33_PING33 *const pping33) {
	pping33_TTTT_d0_d1_d2 {
				pping33_TTTT_SET(-1811386598, -969641038, 1040722343, -1302635932, 891968769, -77041463, 1911438681, 1259011225, -935236032, -1769552495, pping33, d0, d1, d2);
			}
	pping33_field_SET(5292691933697610253L, -4886885828986924612L, 5165394714545338033L, 6154131262350633305L, -3914114441315202781L, -5292529790384940987L, 4291271569171960149L, 773946600277601187L, 2552240861952675060L, -4042985964337702804L, pping33);
	pping33_bit_field_SET(6, 18, 28, 11, 5, 36, 32, 16, 35, 15, pping33);
	pping33_field6_d0_d1_d2 {
				pping33_field6_SET(-2095097337, 1394637800, -313125569, 800231148, 2090110784, -1204438422, 1009283391, -395696869, 264307794, -1301708551, pping33, d0, d1, d2);
			}
	pping33_testBOOL2_SET(false, true, true, true, false, false, false, true, true, false, pping33);
	pping33_testBOOL3_SET(true, true, false, true, false, true, false, true, false, true, pping33);
	pping33_testBOOL_SET(false, false, true, false, false, false, false, true, false, true, pping33);
	pping33_seq_SET(2672327586L, 1948613403L, 3487063819L, 144924972L, 1364796200L, 1418713815L, 3611348447L, 3474442979L, 3708662533L, 3870824741L, pping33);
	size_t pping33_field1_d0 = 0;
	
	if (!pping33_field1(pping33, &pping33_field1_d0)) pping33_field1(pping33, &some_size_t);
	pping33_field1_d0_d1_d2 (pping33) {
						qping33_field1_SET(66108169, 1692547755, -1411164716, 2022667028, 1543183916, -1588846813, 717320171, 370715315, -512219066, -1688943780, fld_field1, d0, d1, d2);
					}
	for (size_t d0           = 0; d0 < Pping33_field12_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field12_D1; d1++)\

					for (size_t d2 = 0; d2 < Pping33_field12_D2; d2++) {
						pping33_field12_SET(-245763127, -1367716436, -55779108, -1565658834, -260284237, 924051311, 999607759, 599731776, -2127676454, 2085796596, pping33, d0, d1, d2);
					}
	for (size_t                 d0 = 0; d0 < Pping33_field13_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field13_D1; d1++)\

					for (size_t d2 = 0; d2 < Pping33_field13_D2; d2++) {
						pping33_field13_SET(1341468785, 343267433, 1677323032, 1291087415, 2074963673, -1294151204, 1233039491, -1088911379, 787031959, -636991767, pping33, d0, d1, d2);
					}
	pping33_WWWWWWWW_SET(-1562261466, 61304102, -942142431, 777838038, 728310964, -1005308286, 706589810, -1111728978, -1559786480, -1578895456, pping33);
	pping33_bit_field2_SET(36, 34, 30, 40, 13, 44, 43, 18, 44, 30, pping33);
	for (size_t d0 = 0; d0 < Pping33_Field_Bits_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_Field_Bits_D1; d1++)\

					for (size_t d2                          = 0; d2 < Pping33_Field_Bits_D2; d2++) {
						pping33_Field_Bits_SET(24, 7, 32, 6, 19, 11, 45, 37, 42, 32, pping33, d0, d1, d2);
					}
	size_t                      pping33_SparseFixAllBits_d0 = 0;
	
	if (!pping33_SparseFixAllBits(pping33, &pping33_SparseFixAllBits_d0)) pping33_SparseFixAllBits(pping33, &some_size_t);
	pping33_SparseFixAllBits_d0_d1_d2 (pping33) {
						qping33_SparseFixAllBits_SET(7, 33, 8, 16, 34, 38, 4, 5, 22, 15, fld_SparseFixAllBits, d0, d1, d2);
					}
	size_t pping33_FixAllBits_d0                            = 0;
	
	if (!pping33_FixAllBits(pping33, &pping33_FixAllBits_d0)) pping33_FixAllBits(pping33, &some_size_t);
	pping33_FixAllBits_d0_d1_d2 (pping33) {
						qping33_FixAllBits_SET(14, 25, 14, 41, 44, 37, 26, 41, 37, 25, fld_FixAllBits, d0, d1, d2);
					}
	size_t pping33_VarAllBits_d0                            = 0;
	size_t pping33_VarAllBits_d2                            = 0;
	
	if (!pping33_VarAllBits(pping33, &pping33_VarAllBits_d0, &pping33_VarAllBits_d2)) pping33_VarAllBits(pping33, &some_size_t, &some_size_t);
	pping33_VarAllBits_d0_d1_d2 (pping33) {
						qping33_VarAllBits_SET(28, 23, 42, 17, 35, 18, 19, 32, 14, 31, fld_VarAllBits, d0, d1, d2);
					}
	size_t pping33_SparseVarAllBits_d0                      = 0;
	size_t pping33_SparseVarAllBits_d2                      = 0;
	
	if (!pping33_SparseVarAllBits(pping33, &pping33_SparseVarAllBits_d0, &pping33_SparseVarAllBits_d2)) pping33_SparseVarAllBits(pping33, &some_size_t, &some_size_t);
	pping33_SparseVarAllBits_d0_d1_d2 (pping33) {
						qping33_SparseVarAllBits_SET(17, 29, 24, 20, 36, 39, 42, 27, 20, 31, fld_SparseVarAllBits, d0, d1, d2);
					}
	size_t pping33_VarEachBits_d0                           = 0;
	
	if (!pping33_VarEachBits(pping33, &pping33_VarEachBits_d0)) pping33_VarEachBits(pping33, &some_size_t);
	pping33_VarEachBits_d0_d1_d2 (pping33) {
						qping33_VarEachBits_SET(25, -3, 5, 17, 13, 21, 25, 11, -8, 40, fld_VarEachBits, d0, d1, d2);
					}
	size_t pping33_SparsVarEachBits_d0                      = 0;
	
	if (!pping33_SparsVarEachBits(pping33, &pping33_SparsVarEachBits_d0)) pping33_SparsVarEachBits(pping33, &some_size_t);
	pping33_SparsVarEachBits_d0_d1_d2 (pping33) {
						qping33_SparsVarEachBits_SET(284, 336, 166, 133, -10, 5, 321, 95, 418, 278, fld_SparsVarEachBits, d0, d1, d2);
					}
	pping33_testBOOLX_SET(true, true, true, true, true, true, false, false, false, true, pping33);
	pping33_testBOOL2X_SET(true, false, false, true, false, true, false, false, false, false, pping33);
	pping33_testBOOL3X_SET(false, true, true, true, true, false, false, false, false, true, pping33);
	pping33_MMMMMM_SET(e_MAV_MODE_MANUAL_ARMED, e_MAV_MODE_MAV_MODE_TEST_ARMED, e_MAV_MODE_GUIDED_ARMED, e_MAV_MODE_MANUAL_DISARMED, e_MAV_MODE_STABILIZE_ARMED, e_MAV_MODE_GUIDED_ARMED, e_MAV_MODE_MAV_MODE_TEST_DISARMED, e_MAV_MODE_MANUAL_ARMED, e_MAV_MODE_MAV_MODE_TEST_ARMED, e_MAV_MODE_MANUAL_ARMED, pping33);
	size_t pping33_field44_d2     = 0;
	
	if (!pping33_field44(pping33, &pping33_field44_d2)) pping33_field44(pping33, &some_size_t);
	pping33_field44_d0_d1_d2 (pping33) {
						qping33_field44_SET(2042357983, 1232639339, 520162465, -870462025, 593517864, -1439709327, -1523162810, 1807013725, 1421029364, -296871756, fld_field44, d0, d1, d2);
					}
	size_t pping33_field634_d2    = 0;
	
	if (!pping33_field634(pping33, &pping33_field634_d2)) pping33_field634(pping33, &some_size_t);
	pping33_field634_d0_d1_d2 (pping33) {
						qping33_field634_SET(-1712476596, -1439265315, -603061671, 1160883522, -1389775292, -303372857, -2088827431, 1627728486, -1438626795, 1128962812, fld_field634, d0, d1, d2);
					}
	size_t pping33_field33344_d2  = 0;
	
	if (!pping33_field33344(pping33, &pping33_field33344_d2)) pping33_field33344(pping33, &some_size_t);
	pping33_field33344_d0_d1_d2 (pping33) {
						qping33_field33344_SET(1319073725, -111003688, 1743432925, 2136269691, 1042388148, -692987111, 790788655, 1880455953, -707230480, 932258361, fld_field33344, d0, d1, d2);
					}
	size_t pping33_field333634_d2 = 0;
	
	if (!pping33_field333634(pping33, &pping33_field333634_d2)) pping33_field333634(pping33, &some_size_t);
	pping33_field333634_d0_d1_d2 (pping33) {
						qping33_field333634_SET(-1161138503, 1053782970, -629082170, 99097400, -1854837353, -206249989, 2130747967, 585467943, 1205213095, -116659484, fld_field333634, d0, d1, d2);
					}
	for (size_t d0                = 0; d0 < Pping33_field___D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field___D1; d1++)\

					for (size_t d2                 = 0; d2 < Pping33_field___D2; d2++) {
						pping33_field___SET(-597924063, -1151566945, -718035089, -424096309, -176853486, 1088530854, 241822884, 1924204060, 1047356053, 917185104, pping33, d0, d1, d2);
					}
	size_t                      pping33_field63_d2 = 0;
	
	if (!pping33_field63(pping33, &pping33_field63_d2)) pping33_field63(pping33, &some_size_t);
	pping33_field63_d0_d1_d2 (pping33) {
						qping33_field63_SET(-1207284964, 43762359, -572525012, 1766272952, 385628804, -1686418894, -1172696199, 501947553, -254953636, 737477171, fld_field63, d0, d1, d2);
					}
	for (size_t d0                                 = 0; d0 < Pping33_uid2_D0; d0++) {
		pping33_uid2_SET(29, 17, 109, -59, 46, -89, 61, 99, -1, -45, pping33, d0);
	}
	for (size_t d0                                 = 0; d0 < Pping33_field2_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field2_D1; d1++)\

					for (size_t d2 = 0; d2 < Pping33_field2_D2; d2++) {
						pping33_field2_SET(-737554776, 1254864339, 1581184932, 1329778646, 304714262, 229847261, 449664777, 632509300, -894519317, -483230245, pping33, d0, d1, d2);
					}
	for (size_t                 d0 = 0; d0 < Pping33_field4_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field4_D1; d1++)\

					for (size_t d2 = 0; d2 < Pping33_field4_D2; d2++) {
						pping33_field4_SET(1374822844, 670652617, -477384912, -1765628341, 597920141, -1085375277, 2042976040, -571845555, 491870866, 1002634854, pping33, d0, d1, d2);
					}
	pping33_stringtest1_SET(some_string, strlen(some_string), pping33);
	size_t pping33_stringtest2_d2 = 0;
	
	if (!pping33_stringtest2(pping33, &pping33_stringtest2_d2)) pping33_stringtest2(pping33, &some_size_t);
	pping33_stringtest2_d0_d1_d2 (pping33) {
						qping33_stringtest2_SET(some_string, strlen(some_string), fld_stringtest2, d0, d1, d2);
					}
	pping33_stringtest3_SET(some_string, strlen(some_string), pping33);
	pping33_stringtest4_SET(some_string, strlen(some_string), pping33);
	
}


void read_VFR_HUD(pvfr_hud_VFR_HUD *const pvfr_hud) {
	int16_t throttle    = pvfr_hud_throttle_GET(pvfr_hud);
	float   airspeed    = pvfr_hud_airspeed_GET(pvfr_hud);
	float   groundspeed = pvfr_hud_groundspeed_GET(pvfr_hud);
	int16_t heading     = pvfr_hud_heading_GET(pvfr_hud);
	float   alt         = pvfr_hud_alt_GET(pvfr_hud);
	float   climb       = pvfr_hud_climb_GET(pvfr_hud);
	
}


void write_VFR_HUD(pvfr_hud_VFR_HUD *const pvfr_hud) {
	pvfr_hud_throttle_SET(-20558, -19362, -5050, 28954, -818, -14151, -6402, 13769, -20715, -15892, pvfr_hud);
	pvfr_hud_airspeed_SET(2.7149427E38F, -2.7852146E38F, 1.8790194E38F, -3.14868E38F, -9.871714E37F, 1.6170214E38F, -2.6412705E38F, 1.678194E38F, 5.636771E37F, -2.0231205E38F, pvfr_hud);
	pvfr_hud_groundspeed_SET(1.253465E38F, 1.636349E38F, 2.10295E38F, -5.030686E37F, -3.5314461E37F, -1.4509854E38F, -1.922858E38F, -2.0454743E38F, -1.573287E38F, -1.5915435E38F, pvfr_hud);
	pvfr_hud_heading_SET(-16634, 10425, -4862, -25589, 12572, -27563, -880, 27775, 8321, 22815, pvfr_hud);
	pvfr_hud_alt_SET(2.0016501E38F, 2.641045E38F, -1.188387E38F, 8.0797494E37F, 2.3223372E37F, 1.0586014E38F, -1.0648087E38F, 3.9867368E37F, 1.2059212E38F, -1.2519118E38F, pvfr_hud);
	pvfr_hud_climb_SET(6.7489814E37F, -3.526953E36F, 2.4684826E38F, -2.7182354E38F, 2.126416E38F, 1.9165415E38F, 1.4764486E38F, -4.343298E37F, -2.1373236E38F, -1.9554334E38F, pvfr_hud);
	
}


void read_RALLY_POINT(prally_point_RALLY_POINT *const prally_point) {
	int16_t       land_dir         = prally_point_land_dir_GET(prally_point);
	int8_t        target_system    = prally_point_target_system_GET(prally_point);
	int8_t        target_component = prally_point_target_component_GET(prally_point);
	int8_t        idx              = prally_point_idx_GET(prally_point);
	int8_t        count            = prally_point_count_GET(prally_point);
	int32_t       lat              = prally_point_lat_GET(prally_point);
	int32_t       lng              = prally_point_lng_GET(prally_point);
	int16_t       alt              = prally_point_alt_GET(prally_point);
	int16_t       break_alt        = prally_point_break_alt_GET(prally_point);
	e_RALLY_FLAGS item_flags;
	if (prally_point_flags_GET(prally_point, &item_flags)) {
		e_RALLY_FLAGS_FAVORABLE_WIND = item_flags;
	}
	
}


void read_MISSION_SET_CURRENT(pmission_set_current_MISSION_SET_CURRENT *const pmission_set_current) {
	int16_t seq              = pmission_set_current_seq_GET(pmission_set_current);
	int8_t  target_system    = pmission_set_current_target_system_GET(pmission_set_current);
	int8_t  target_component = pmission_set_current_target_component_GET(pmission_set_current);
	
}


void write_MISSION_SET_CURRENT(pmission_set_current_MISSION_SET_CURRENT *const pmission_set_current) {
	pmission_set_current_seq_SET(17619, -944, -25432, -6137, -32272, -7037, 21624, 19741, -20937, -1778, pmission_set_current);
	pmission_set_current_target_system_SET(21, -91, 109, 75, 38, -81, -65, 45, 24, -23, pmission_set_current);
	pmission_set_current_target_component_SET(-4, 51, 22, 109, 78, 68, -25, 91, -104, -46, pmission_set_current);
	
}


void read_ADAP_TUNING(padap_tuning_ADAP_TUNING *const padap_tuning) {
	float             desired   = padap_tuning_desired_GET(padap_tuning);
	float             achieved  = padap_tuning_achieved_GET(padap_tuning);
	float             error     = padap_tuning_error_GET(padap_tuning);
	float             theta     = padap_tuning_theta_GET(padap_tuning);
	float             omega     = padap_tuning_omega_GET(padap_tuning);
	float             sigma     = padap_tuning_sigma_GET(padap_tuning);
	float             theta_dot = padap_tuning_theta_dot_GET(padap_tuning);
	float             omega_dot = padap_tuning_omega_dot_GET(padap_tuning);
	float             sigma_dot = padap_tuning_sigma_dot_GET(padap_tuning);
	float             f         = padap_tuning_f_GET(padap_tuning);
	float             f_dot     = padap_tuning_f_dot_GET(padap_tuning);
	float             u         = padap_tuning_u_GET(padap_tuning);
	e_PID_TUNING_AXIS item_axis;
	if (padap_tuning_axis_GET(padap_tuning, &item_axis)) {
		e_PID_TUNING_AXIS_PID_TUNING_ROLL = item_axis;
	}
	
}


void read_VIBRATION(pvibration_VIBRATION *const pvibration) {
	int32_t clipping_0  = pvibration_clipping_0_GET(pvibration);
	int32_t clipping_1  = pvibration_clipping_1_GET(pvibration);
	int32_t clipping_2  = pvibration_clipping_2_GET(pvibration);
	int64_t time_usec   = pvibration_time_usec_GET(pvibration);
	float   vibration_x = pvibration_vibration_x_GET(pvibration);
	float   vibration_y = pvibration_vibration_y_GET(pvibration);
	float   vibration_z = pvibration_vibration_z_GET(pvibration);
	
}


void write_VIBRATION(pvibration_VIBRATION *const pvibration) {
	pvibration_clipping_0_SET(-328834520, 1135185929, -1828407303, -2111233152, 266942432, 1832726146, 226854320, 1534777511, -1527268023, -1906790986, pvibration);
	pvibration_clipping_1_SET(1262424689, -1903637732, -1638839584, -153331107, -495407772, 119254461, -1552728380, -1672253186, -88352777, -1031215172, pvibration);
	pvibration_clipping_2_SET(101447332, 929942073, -1350812833, 1167146097, 454622419, -421868153, 351638538, -799699762, -307630770, 1192117833, pvibration);
	pvibration_time_usec_SET(160089403718831109L, -4640327453767316982L, -4107108539601047465L, 6071045131279050538L, -2830047242259952170L, 7390409383356794236L, -5826005881700872936L, -5900271149001279205L, -4098126338323964643L, 8086380922287458111L, pvibration);
	pvibration_vibration_x_SET(5.269762E37F, 2.3240212E38F, -1.7206682E38F, -2.1236024E38F, -1.6057772E38F, -2.234799E38F, 2.6520827E38F, -8.548673E37F, 2.3224842E38F, -1.5576141E38F, pvibration);
	pvibration_vibration_y_SET(-1.7942757E38F, -2.5215747E38F, -3.2646593E38F, 1.0047055E38F, -1.7234278E38F, -1.0630235E38F, -2.1216784E36F, 7.008232E37F, 1.4340988E38F, 9.795951E37F, pvibration);
	pvibration_vibration_z_SET(-2.9510452E38F, -6.179721E37F, 1.2069125E38F, -3.2800033E38F, 7.678339E37F, 7.786206E37F, 6.1953837E37F, 3.3309915E38F, 5.139079E37F, -2.3995561E38F, pvibration);
	
}


void read_PARAM_EXT_VALUE(pparam_ext_value_PARAM_EXT_VALUE *const pparam_ext_value) {
	int16_t                   param_count = pparam_ext_value_param_count_GET(pparam_ext_value);
	int16_t                   param_index = pparam_ext_value_param_index_GET(pparam_ext_value);
	Vparam_ext_value_param_id item_param_id;
	if (pparam_ext_value_param_id_GET(pparam_ext_value, &item_param_id)) {
		memcpy(some_string, item_param_id.bytes, item_param_id.len);
	}
	Vparam_ext_value_param_value item_param_value;
	if (pparam_ext_value_param_value_GET(pparam_ext_value, &item_param_value)) {
		memcpy(some_string, item_param_value.bytes, item_param_value.len);
	}
	e_MAV_PARAM_EXT_TYPE item_param_type;
	if (pparam_ext_value_param_type_GET(pparam_ext_value, &item_param_type)) {
		e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8 = item_param_type;
	}
	
}


void read_BATTERY2(pbattery2_BATTERY2 *const pbattery2) {
	int16_t voltage         = pbattery2_voltage_GET(pbattery2);
	int16_t current_battery = pbattery2_current_battery_GET(pbattery2);
	
}


void read_LIMITS_STATUS(plimits_status_LIMITS_STATUS *const plimits_status) {
	int16_t        breach_count  = plimits_status_breach_count_GET(plimits_status);
	int32_t        last_trigger  = plimits_status_last_trigger_GET(plimits_status);
	int32_t        last_action   = plimits_status_last_action_GET(plimits_status);
	int32_t        last_recovery = plimits_status_last_recovery_GET(plimits_status);
	int32_t        last_clear    = plimits_status_last_clear_GET(plimits_status);
	e_LIMITS_STATE item_limits_state;
	if (plimits_status_limits_state_GET(plimits_status, &item_limits_state)) {
		e_LIMITS_STATE_LIMITS_INIT = item_limits_state;
	}
	e_LIMIT_MODULE item_mods_enabled;
	if (plimits_status_mods_enabled_GET(plimits_status, &item_mods_enabled)) {
		e_LIMIT_MODULE_LIMIT_GPSLOCK = item_mods_enabled;
	}
	e_LIMIT_MODULE item_mods_required;
	if (plimits_status_mods_required_GET(plimits_status, &item_mods_required)) {
		e_LIMIT_MODULE_LIMIT_GPSLOCK = item_mods_required;
	}
	e_LIMIT_MODULE item_mods_triggered;
	if (plimits_status_mods_triggered_GET(plimits_status, &item_mods_triggered)) {
		e_LIMIT_MODULE_LIMIT_GPSLOCK = item_mods_triggered;
	}
	
}


void read_CAMERA_FEEDBACK(pcamera_feedback_CAMERA_FEEDBACK *const pcamera_feedback) {
	int16_t                 img_idx       = pcamera_feedback_img_idx_GET(pcamera_feedback);
	int64_t                 time_usec     = pcamera_feedback_time_usec_GET(pcamera_feedback);
	int8_t                  target_system = pcamera_feedback_target_system_GET(pcamera_feedback);
	int8_t                  cam_idx       = pcamera_feedback_cam_idx_GET(pcamera_feedback);
	int32_t                 lat           = pcamera_feedback_lat_GET(pcamera_feedback);
	int32_t                 lng           = pcamera_feedback_lng_GET(pcamera_feedback);
	float                   alt_msl       = pcamera_feedback_alt_msl_GET(pcamera_feedback);
	float                   alt_rel       = pcamera_feedback_alt_rel_GET(pcamera_feedback);
	float                   roll          = pcamera_feedback_roll_GET(pcamera_feedback);
	float                   pitch         = pcamera_feedback_pitch_GET(pcamera_feedback);
	float                   yaw           = pcamera_feedback_yaw_GET(pcamera_feedback);
	float                   foc_len       = pcamera_feedback_foc_len_GET(pcamera_feedback);
	e_CAMERA_FEEDBACK_FLAGS item_flags;
	if (pcamera_feedback_flags_GET(pcamera_feedback, &item_flags)) {
		e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_PHOTO = item_flags;
	}
	
}


void read_HIL_GPS(phil_gps_HIL_GPS *const phil_gps) {
	int16_t eph                = phil_gps_eph_GET(phil_gps);
	int16_t epv                = phil_gps_epv_GET(phil_gps);
	int16_t vel                = phil_gps_vel_GET(phil_gps);
	int16_t cog                = phil_gps_cog_GET(phil_gps);
	int64_t time_usec          = phil_gps_time_usec_GET(phil_gps);
	int8_t  fix_type           = phil_gps_fix_type_GET(phil_gps);
	int32_t lat                = phil_gps_lat_GET(phil_gps);
	int32_t lon                = phil_gps_lon_GET(phil_gps);
	int32_t alt                = phil_gps_alt_GET(phil_gps);
	int16_t vn                 = phil_gps_vn_GET(phil_gps);
	int16_t ve                 = phil_gps_ve_GET(phil_gps);
	int16_t vd                 = phil_gps_vd_GET(phil_gps);
	int8_t  satellites_visible = phil_gps_satellites_visible_GET(phil_gps);
	
}


void write_HIL_GPS(phil_gps_HIL_GPS *const phil_gps) {
	phil_gps_eph_SET(-27260, -21232, -20872, 23392, -21473, 19362, 16534, 11101, -5826, -30246, phil_gps);
	phil_gps_epv_SET(-5766, -25990, 7739, -22680, -21370, -585, -5898, -29853, -17019, 23665, phil_gps);
	phil_gps_vel_SET(23551, 19635, -18489, 2344, 14115, 9760, 24146, -22419, -30267, -32150, phil_gps);
	phil_gps_cog_SET(-11041, 16417, -17666, -15595, 16788, 28103, -26584, 10564, -14682, 13397, phil_gps);
	phil_gps_time_usec_SET(-6132051155712578983L, -1883012987874505390L, -8761719890457648521L, 2034210697327663127L, 1576773663199623882L, -7112082321854495799L, 7583964562201481581L, -9027401901378279246L, 1559481706866537007L, -6827313247215488380L, phil_gps);
	phil_gps_fix_type_SET(6, -20, -22, -22, -45, 92, 27, -15, 26, 21, phil_gps);
	phil_gps_lat_SET(352260145, -1872155543, -1437113976, 1977741759, 883968398, 1989412462, 711479163, 632422519, 1060895940, -452888546, phil_gps);
	phil_gps_lon_SET(129681001, -1079822916, 266173822, -1503775714, -227600314, 1561241372, -1725361620, -495872441, -737294182, 699164278, phil_gps);
	phil_gps_alt_SET(-1745937955, -1243539962, -1308351759, -1394674370, 1352807209, 604441593, -2133424134, -1525967678, -1507201627, -42207013, phil_gps);
	phil_gps_vn_SET(8480, 6282, 11838, 730, -21418, 27839, 8376, -5567, -498, -13932, phil_gps);
	phil_gps_ve_SET(30973, 1114, 9641, 17996, 7717, -21614, -18812, 12582, -17864, -3974, phil_gps);
	phil_gps_vd_SET(13367, -6098, -17394, 22721, 30615, -4706, -19991, -2029, -3121, 4157, phil_gps);
	phil_gps_satellites_visible_SET(101, -116, 10, -109, 43, 125, -25, -11, 105, -57, phil_gps);
	
}


void read_NAV_CONTROLLER_OUTPUT(pnav_controller_output_NAV_CONTROLLER_OUTPUT *const pnav_controller_output) {
	int16_t wp_dist        = pnav_controller_output_wp_dist_GET(pnav_controller_output);
	float   nav_roll       = pnav_controller_output_nav_roll_GET(pnav_controller_output);
	float   nav_pitch      = pnav_controller_output_nav_pitch_GET(pnav_controller_output);
	int16_t nav_bearing    = pnav_controller_output_nav_bearing_GET(pnav_controller_output);
	int16_t target_bearing = pnav_controller_output_target_bearing_GET(pnav_controller_output);
	float   alt_error      = pnav_controller_output_alt_error_GET(pnav_controller_output);
	float   aspd_error     = pnav_controller_output_aspd_error_GET(pnav_controller_output);
	float   xtrack_error   = pnav_controller_output_xtrack_error_GET(pnav_controller_output);
	
}


void write_NAV_CONTROLLER_OUTPUT(pnav_controller_output_NAV_CONTROLLER_OUTPUT *const pnav_controller_output) {
	pnav_controller_output_wp_dist_SET(-617, 7020, 15750, -25560, 19757, -20506, -13096, 19703, 21254, -2567, pnav_controller_output);
	pnav_controller_output_nav_roll_SET(2.6318722E38F, 2.5847907E38F, -1.6714626E38F, 2.2854404E37F, -1.7339126E38F, -1.8912227E38F, 1.763269E38F, 2.8135657E38F, 4.925114E37F, -2.9364903E38F, pnav_controller_output);
	pnav_controller_output_nav_pitch_SET(-5.6335823E37F, -1.3465186E38F, 3.36072E38F, -2.4201472E38F, -1.1776508E38F, 8.472945E37F, 5.2918794E37F, -1.4144769E38F, -2.6096405E38F, -1.9189066E37F, pnav_controller_output);
	pnav_controller_output_nav_bearing_SET(11179, 25552, -29924, -8271, 16828, 27712, 3190, 5570, -24619, -24428, pnav_controller_output);
	pnav_controller_output_target_bearing_SET(-9012, 31813, -25650, 6494, 24440, -23204, 12541, -3596, -32101, -8448, pnav_controller_output);
	pnav_controller_output_alt_error_SET(-1.9353439E38F, -1.6159118E38F, 2.959366E38F, -8.4717455E37F, 3.348573E38F, -2.5094425E38F, -3.3751771E38F, -1.5408079E38F, 2.2733683E38F, 3.1560377E38F, pnav_controller_output);
	pnav_controller_output_aspd_error_SET(-2.7008835E38F, -7.1439236E37F, 9.303168E37F, 1.7225035E38F, -8.986929E36F, -1.3446995E38F, -8.834416E37F, 2.266062E37F, -1.6683745E38F, -2.4252726E38F, pnav_controller_output);
	pnav_controller_output_xtrack_error_SET(2.889425E38F, -3.0175686E38F, 2.0074998E38F, 3.3709823E38F, -1.8280053E38F, -1.1259107E38F, 8.946361E37F, 1.6014819E38F, -1.3755754E38F, 1.3718993E38F, pnav_controller_output);
	
}


void read_AUTH_KEY(pauth_key_AUTH_KEY *const pauth_key) {
	Vauth_key_key item_key;
	if (pauth_key_key_GET(pauth_key, &item_key)) {
		memcpy(some_string, item_key.bytes, item_key.len);
	}
	
}


void write_AUTH_KEY(pauth_key_AUTH_KEY *const pauth_key) {
	pauth_key_key_SET(some_string, strlen(some_string), pauth_key);
	
}


void read_FENCE_FETCH_POINT(pfence_fetch_point_FENCE_FETCH_POINT *const pfence_fetch_point) {
	int8_t target_system    = pfence_fetch_point_target_system_GET(pfence_fetch_point);
	int8_t target_component = pfence_fetch_point_target_component_GET(pfence_fetch_point);
	int8_t idx              = pfence_fetch_point_idx_GET(pfence_fetch_point);
	
}


void read_RADIO(pradio_RADIO *const pradio) {
	int16_t rxerrors = pradio_rxerrors_GET(pradio);
	int16_t fixeD    = pradio_fixeD_GET(pradio);
	int8_t  rssi     = pradio_rssi_GET(pradio);
	int8_t  remrssi  = pradio_remrssi_GET(pradio);
	int8_t  txbuf    = pradio_txbuf_GET(pradio);
	int8_t  noise    = pradio_noise_GET(pradio);
	int8_t  remnoise = pradio_remnoise_GET(pradio);
	
}


void read_LOCAL_POSITION_NED_COV(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *const plocal_position_ned_cov) {
	int64_t                            time_usec       = plocal_position_ned_cov_time_usec_GET(plocal_position_ned_cov);
	float                              x               = plocal_position_ned_cov_x_GET(plocal_position_ned_cov);
	float                              y               = plocal_position_ned_cov_y_GET(plocal_position_ned_cov);
	float                              z               = plocal_position_ned_cov_z_GET(plocal_position_ned_cov);
	float                              vx              = plocal_position_ned_cov_vx_GET(plocal_position_ned_cov);
	float                              vy              = plocal_position_ned_cov_vy_GET(plocal_position_ned_cov);
	float                              vz              = plocal_position_ned_cov_vz_GET(plocal_position_ned_cov);
	float                              ax              = plocal_position_ned_cov_ax_GET(plocal_position_ned_cov);
	float                              ay              = plocal_position_ned_cov_ay_GET(plocal_position_ned_cov);
	float                              az              = plocal_position_ned_cov_az_GET(plocal_position_ned_cov);
	Vlocal_position_ned_cov_covariance item_covariance = plocal_position_ned_cov_covariance_GET(plocal_position_ned_cov);
	for (size_t                        index           = 0; index < item_covariance.len; index++)
		some_float = vlocal_position_ned_cov_covariance_GET(&item_covariance, index);
	e_MAV_ESTIMATOR_TYPE               item_estimator_type;
	if (plocal_position_ned_cov_estimator_type_GET(plocal_position_ned_cov, &item_estimator_type)) {
		e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE = item_estimator_type;
	}
	
}


void write_LOCAL_POSITION_NED_COV(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *const plocal_position_ned_cov) {
	plocal_position_ned_cov_time_usec_SET(-1077344246675018395L, -8902355837407511605L, -2436222226453755317L, -6476406809629807118L, 4513129920327114111L, 6489001147602217885L, -5729626624954797860L, 6326172661365731474L, -7298162790141878565L, 3261555471556083840L, plocal_position_ned_cov);
	plocal_position_ned_cov_x_SET(2.981215E38F, -2.0393977E38F, -3.1682913E38F, 1.4189271E38F, -2.401393E38F, -3.2241594E38F, 1.1095641E38F, 1.060861E38F, 1.1247419E38F, 6.024994E37F, plocal_position_ned_cov);
	plocal_position_ned_cov_y_SET(-2.2323706E38F, 2.8501227E38F, -2.0941655E38F, 2.2914032E38F, 1.6518735E38F, 2.3537943E38F, -1.9310136E38F, -5.27352E37F, 6.655799E37F, -7.038951E36F, plocal_position_ned_cov);
	plocal_position_ned_cov_z_SET(4.486577E37F, 1.8870095E38F, 1.7636178E38F, 2.9926024E38F, -2.4599387E38F, 1.9455508E38F, -2.477636E38F, 3.3791575E38F, -2.1002502E38F, -2.0211054E38F, plocal_position_ned_cov);
	plocal_position_ned_cov_vx_SET(1.5732485E38F, -2.7673046E38F, -1.2583826E38F, 2.5853547E38F, 2.625307E38F, -3.345586E38F, 6.5694623E37F, -1.397826E38F, -1.0499153E38F, 2.8267882E38F, plocal_position_ned_cov);
	plocal_position_ned_cov_vy_SET(-3.1345008E38F, 2.3131496E38F, 2.1038696E38F, 1.1129065E38F, -6.92214E37F, 3.055975E38F, -1.6264324E38F, 1.3477179E38F, -1.2896657E38F, -9.559226E37F, plocal_position_ned_cov);
	plocal_position_ned_cov_vz_SET(8.497451E37F, 5.211587E37F, 7.903676E37F, -2.4415892E38F, -2.8617001E38F, -2.4582666E37F, -2.6909864E37F, -2.845701E38F, -3.121446E38F, 4.8271907E37F, plocal_position_ned_cov);
	plocal_position_ned_cov_ax_SET(-4.9585887E37F, 2.9868252E38F, -1.944504E38F, 1.8349364E38F, 3.434751E37F, 1.9966136E37F, -2.5197738E38F, 2.9714667E38F, -3.1088056E38F, 2.7622143E38F, plocal_position_ned_cov);
	plocal_position_ned_cov_ay_SET(-2.3498214E38F, 1.562483E37F, 1.2774445E38F, -2.8847261E38F, 2.181422E38F, 2.5116067E38F, 2.833898E38F, -2.7115914E38F, -2.2640832E38F, -1.7732976E38F, plocal_position_ned_cov);
	plocal_position_ned_cov_az_SET(1.4842848E38F, 5.834636E37F, 2.0306355E38F, 3.0836211E38F, 1.0586704E38F, 3.3418823E38F, -3.250837E38F, 1.5985397E38F, -1.649386E38F, 4.2097601E37F, plocal_position_ned_cov);
	plocal_position_ned_cov_covariance_SET(&-7.651648E37F, 7.5968647E37F, -3.0189411E38F, -1.0410678E38F, 1.9803317E38F, -1.6813928E38F, -1.1100532E38F, 2.4744716E38F, -3.7237987E37F, -5.2502047E37F, plocal_position_ned_cov);
	plocal_position_ned_cov_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, plocal_position_ned_cov);
	
}


void read_AIRSPEED_AUTOCAL(pairspeed_autocal_AIRSPEED_AUTOCAL *const pairspeed_autocal) {
	float vx            = pairspeed_autocal_vx_GET(pairspeed_autocal);
	float vy            = pairspeed_autocal_vy_GET(pairspeed_autocal);
	float vz            = pairspeed_autocal_vz_GET(pairspeed_autocal);
	float diff_pressure = pairspeed_autocal_diff_pressure_GET(pairspeed_autocal);
	float EAS2TAS       = pairspeed_autocal_EAS2TAS_GET(pairspeed_autocal);
	float ratio         = pairspeed_autocal_ratio_GET(pairspeed_autocal);
	float state_x       = pairspeed_autocal_state_x_GET(pairspeed_autocal);
	float state_y       = pairspeed_autocal_state_y_GET(pairspeed_autocal);
	float state_z       = pairspeed_autocal_state_z_GET(pairspeed_autocal);
	float Pax           = pairspeed_autocal_Pax_GET(pairspeed_autocal);
	float Pby           = pairspeed_autocal_Pby_GET(pairspeed_autocal);
	float Pcz           = pairspeed_autocal_Pcz_GET(pairspeed_autocal);
	
}


void read_ATT_POS_MOCAP(patt_pos_mocap_ATT_POS_MOCAP *const patt_pos_mocap) {
	int64_t          time_usec = patt_pos_mocap_time_usec_GET(patt_pos_mocap);
	Vatt_pos_mocap_q item_q    = patt_pos_mocap_q_GET(patt_pos_mocap);
	for (size_t      index     = 0; index < item_q.len; index++)
		some_float     = vatt_pos_mocap_q_GET(&item_q, index);
	float            x = patt_pos_mocap_x_GET(patt_pos_mocap);
	float            y = patt_pos_mocap_y_GET(patt_pos_mocap);
	float            z = patt_pos_mocap_z_GET(patt_pos_mocap);
	
}


void write_ATT_POS_MOCAP(patt_pos_mocap_ATT_POS_MOCAP *const patt_pos_mocap) {
	patt_pos_mocap_time_usec_SET(5433745084351366724L, -5919436879206963081L, 570585163779148712L, 1338859560988998676L, -7693652922536349297L, -4490504715587715046L, 8139325237151061884L, 650189819485380662L, 162008247064121756L, 1916530081512457859L, patt_pos_mocap);
	patt_pos_mocap_q_SET(&6.348115E37F, -3.1062973E38F, -3.2236016E38F, 1.4790872E38F, 1.389902E38F, -6.6369495E37F, -1.5105021E38F, -2.8675895E38F, -2.3659656E38F, -1.9749956E37F, patt_pos_mocap);
	patt_pos_mocap_x_SET(-1.8742687E38F, 2.8261193E38F, -3.1992473E38F, -1.3623635E38F, 8.584902E37F, -2.2933304E38F, -3.1273409E38F, 3.1123126E38F, -2.3563576E38F, 2.4578392E38F, patt_pos_mocap);
	patt_pos_mocap_y_SET(5.589917E36F, -2.6588681E38F, 4.8355835E37F, -2.6914847E38F, -1.6552304E38F, 9.552403E37F, -2.8346164E38F, -1.4970102E38F, -2.1323905E38F, -1.8121948E38F, patt_pos_mocap);
	patt_pos_mocap_z_SET(-3.2817282E38F, -1.9268614E38F, 1.206822E38F, -3.1164232E37F, -3.1565153E38F, -1.4949892E38F, 1.2136342E38F, 2.4832663E38F, 2.5232766E38F, 1.7644983E37F, patt_pos_mocap);
	
}


void read_STATUSTEXT(pstatustext_STATUSTEXT *const pstatustext) {
	e_MAV_SEVERITY item_severity;
	if (pstatustext_severity_GET(pstatustext, &item_severity)) {
		e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY = item_severity;
	}
	Vstatustext_text item_text;
	if (pstatustext_text_GET(pstatustext, &item_text)) {
		memcpy(some_string, item_text.bytes, item_text.len);
	}
	
}


void write_STATUSTEXT(pstatustext_STATUSTEXT *const pstatustext) {
	pstatustext_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY, e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, e_MAV_SEVERITY_MAV_SEVERITY_ERROR, e_MAV_SEVERITY_MAV_SEVERITY_WARNING, e_MAV_SEVERITY_MAV_SEVERITY_INFO, e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, e_MAV_SEVERITY_MAV_SEVERITY_WARNING, e_MAV_SEVERITY_MAV_SEVERITY_ERROR, e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, e_MAV_SEVERITY_MAV_SEVERITY_NOTICE, pstatustext);
	pstatustext_text_SET(some_string, strlen(some_string), pstatustext);
	
}


void read_PING(pping_PING *const pping) {
	int32_t seq              = pping_seq_GET(pping);
	int64_t time_usec        = pping_time_usec_GET(pping);
	int8_t  target_system    = pping_target_system_GET(pping);
	int8_t  target_component = pping_target_component_GET(pping);
	
}


void write_PING(pping_PING *const pping) {
	pping_seq_SET(-1741181789, -1651785321, -1842424251, -1833026624, 331711493, -1642943330, -269330527, 1282695819, 823682821, -824023543, pping);
	pping_time_usec_SET(-4058340994445270780L, 9180956485339623433L, 3795637507857420934L, -8050715572620720761L, -8760048919536780398L, -6234237805002529662L, -1197671649600560155L, -987271574826548877L, 7360339194145009153L, -7398220426055626763L, pping);
	pping_target_system_SET(-22, -28, 116, -8, 125, 22, 106, 107, -84, 107, pping);
	pping_target_component_SET(119, 120, 20, 117, 123, -33, 26, -45, -66, -6, pping);
	
}


void read_GOPRO_GET_REQUEST(pgopro_get_request_GOPRO_GET_REQUEST *const pgopro_get_request) {
	int8_t          target_system    = pgopro_get_request_target_system_GET(pgopro_get_request);
	int8_t          target_component = pgopro_get_request_target_component_GET(pgopro_get_request);
	e_GOPRO_COMMAND item_cmd_id;
	if (pgopro_get_request_cmd_id_GET(pgopro_get_request, &item_cmd_id)) {
		e_GOPRO_COMMAND_GOPRO_COMMAND_POWER = item_cmd_id;
	}
	
}


void read_CAMERA_CAPTURE_STATUS(pcamera_capture_status_CAMERA_CAPTURE_STATUS *const pcamera_capture_status) {
	int32_t time_boot_ms       = pcamera_capture_status_time_boot_ms_GET(pcamera_capture_status);
	int32_t recording_time_ms  = pcamera_capture_status_recording_time_ms_GET(pcamera_capture_status);
	int8_t  image_status       = pcamera_capture_status_image_status_GET(pcamera_capture_status);
	int8_t  video_status       = pcamera_capture_status_video_status_GET(pcamera_capture_status);
	float   image_interval     = pcamera_capture_status_image_interval_GET(pcamera_capture_status);
	float   available_capacity = pcamera_capture_status_available_capacity_GET(pcamera_capture_status);
	
}


void write_CAMERA_CAPTURE_STATUS(pcamera_capture_status_CAMERA_CAPTURE_STATUS *const pcamera_capture_status) {
	pcamera_capture_status_time_boot_ms_SET(2128290309, -1872442450, -1267897745, 1179183836, 2130831642, 1660066199, -406798366, -1515224462, -1441579563, -859628165, pcamera_capture_status);
	pcamera_capture_status_recording_time_ms_SET(-1598957040, 1604157560, -810933713, 599044739, -614542383, 1319249223, 2089849002, -1311688114, 874206827, -1473596252, pcamera_capture_status);
	pcamera_capture_status_image_status_SET(-26, 59, 116, 127, 39, -14, 117, -4, 57, -58, pcamera_capture_status);
	pcamera_capture_status_video_status_SET(36, 100, -19, 97, -98, -37, -124, 116, -87, -95, pcamera_capture_status);
	pcamera_capture_status_image_interval_SET(1.0701976E38F, 1.6771407E38F, 2.389358E38F, 4.4075324E37F, -9.798145E37F, -2.4395545E37F, -2.2136845E38F, -3.1119935E37F, -1.6036496E38F, 3.1843572E38F, pcamera_capture_status);
	pcamera_capture_status_available_capacity_SET(2.2193368E38F, -3.5323312E37F, -2.8113415E38F, 2.093897E38F, -3.1678698E38F, -4.537614E37F, -1.1181513E38F, 1.7275494E38F, -2.2691526E38F, 1.1565115E38F, pcamera_capture_status);
	
}


void read_GLOBAL_POSITION_INT(pglobal_position_int_GLOBAL_POSITION_INT *const pglobal_position_int) {
	int16_t hdg          = pglobal_position_int_hdg_GET(pglobal_position_int);
	int32_t time_boot_ms = pglobal_position_int_time_boot_ms_GET(pglobal_position_int);
	int32_t lat          = pglobal_position_int_lat_GET(pglobal_position_int);
	int32_t lon          = pglobal_position_int_lon_GET(pglobal_position_int);
	int32_t alt          = pglobal_position_int_alt_GET(pglobal_position_int);
	int32_t relative_alt = pglobal_position_int_relative_alt_GET(pglobal_position_int);
	int16_t vx           = pglobal_position_int_vx_GET(pglobal_position_int);
	int16_t vy           = pglobal_position_int_vy_GET(pglobal_position_int);
	int16_t vz           = pglobal_position_int_vz_GET(pglobal_position_int);
	
}


void write_GLOBAL_POSITION_INT(pglobal_position_int_GLOBAL_POSITION_INT *const pglobal_position_int) {
	pglobal_position_int_hdg_SET(-4037, 10146, -13189, 31443, 12220, -14893, 5142, 28989, -5743, -340, pglobal_position_int);
	pglobal_position_int_time_boot_ms_SET(1210805727, -1051500169, 453820230, -1574278287, 637538263, -470450353, -705332274, -771680843, 94363190, 2001998026, pglobal_position_int);
	pglobal_position_int_lat_SET(2026115903, -1026706478, -1941588809, -1727526040, 1896275440, -1373234061, 179322031, -1664287726, -137899904, 682248658, pglobal_position_int);
	pglobal_position_int_lon_SET(1277096430, -1696047369, 281893766, -1073991029, -359030791, 1657696789, 2104698572, 684074966, 2079224726, -505524829, pglobal_position_int);
	pglobal_position_int_alt_SET(2066352703, 1745562891, 658919922, -244991878, -1636120070, 274336972, -426544804, -1198430112, -1501820789, 161888587, pglobal_position_int);
	pglobal_position_int_relative_alt_SET(1064837677, 263523906, 1384886259, 1710837225, 1920855118, 624559289, 135468696, -1695015945, 700619018, 809176346, pglobal_position_int);
	pglobal_position_int_vx_SET(-16939, 4713, -20800, 21712, -3843, 5881, -9806, 32700, -12633, 7845, pglobal_position_int);
	pglobal_position_int_vy_SET(-16340, -28257, 25362, 12510, -7878, 26422, -22295, 21217, -7842, -19778, pglobal_position_int);
	pglobal_position_int_vz_SET(30499, 7681, 7405, 7926, 26457, 16447, 4495, -5141, 32311, -8004, pglobal_position_int);
	
}


void read_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA *const pencapsulated_data) {
	int16_t                 seqnr     = pencapsulated_data_seqnr_GET(pencapsulated_data);
	Vencapsulated_data_daTa item_daTa = pencapsulated_data_daTa_GET(pencapsulated_data);
	for (size_t             index     = 0; index < item_daTa.len; index++)
		some_int8_t = vencapsulated_data_daTa_GET(&item_daTa, index);
	
}


void write_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA *const pencapsulated_data) {
	pencapsulated_data_seqnr_SET(4185, -23630, 3858, -3479, -25114, -6578, 16512, -23150, -4751, -24890, pencapsulated_data);
	pencapsulated_data_daTa_SET(&75, 15, -85, 48, 5, -105, 76, 104, 92, -6, pencapsulated_data);
	
}


void read_GPS_INPUT(pgps_input_GPS_INPUT *const pgps_input) {
	int16_t                  time_week          = pgps_input_time_week_GET(pgps_input);
	int32_t                  time_week_ms       = pgps_input_time_week_ms_GET(pgps_input);
	int64_t                  time_usec          = pgps_input_time_usec_GET(pgps_input);
	int8_t                   gps_id             = pgps_input_gps_id_GET(pgps_input);
	int8_t                   fix_type           = pgps_input_fix_type_GET(pgps_input);
	int32_t                  lat                = pgps_input_lat_GET(pgps_input);
	int32_t                  lon                = pgps_input_lon_GET(pgps_input);
	float                    alt                = pgps_input_alt_GET(pgps_input);
	float                    hdop               = pgps_input_hdop_GET(pgps_input);
	float                    vdop               = pgps_input_vdop_GET(pgps_input);
	float                    vn                 = pgps_input_vn_GET(pgps_input);
	float                    ve                 = pgps_input_ve_GET(pgps_input);
	float                    vd                 = pgps_input_vd_GET(pgps_input);
	float                    speed_accuracy     = pgps_input_speed_accuracy_GET(pgps_input);
	float                    horiz_accuracy     = pgps_input_horiz_accuracy_GET(pgps_input);
	float                    vert_accuracy      = pgps_input_vert_accuracy_GET(pgps_input);
	int8_t                   satellites_visible = pgps_input_satellites_visible_GET(pgps_input);
	e_GPS_INPUT_IGNORE_FLAGS item_ignore_flags;
	if (pgps_input_ignore_flags_GET(pgps_input, &item_ignore_flags)) {
		e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT = item_ignore_flags;
	}
	
}


void write_GPS_INPUT(pgps_input_GPS_INPUT *const pgps_input) {
	pgps_input_time_week_SET(24162, 3714, 2835, 7096, -24201, -9312, -4337, -12780, -27677, 2187, pgps_input);
	pgps_input_time_week_ms_SET(1859163485, -398092624, 1726609617, 918709275, 1681532966, 853535283, 193989795, -117816133, -1748827101, -1658591620, pgps_input);
	pgps_input_time_usec_SET(3958057898185863147L, 8329322644842119122L, 6854309747755947095L, -4367717656388249250L, -5715983877744484509L, 155523155723458915L, -980390175974443442L, -3327923209966457411L, 5088953879750954232L, 7119989748243923891L, pgps_input);
	pgps_input_gps_id_SET(47, -4, -115, 99, 99, 1, 54, 49, 109, -83, pgps_input);
	pgps_input_fix_type_SET(38, 82, 117, 54, -119, 1, -11, 1, 63, -58, pgps_input);
	pgps_input_lat_SET(-1799314795, 1581687375, 244765232, -805496587, -1343459727, 1082222266, -1840889296, 1583757833, -501651686, 790523097, pgps_input);
	pgps_input_lon_SET(407609388, -1482502470, -1289442886, 2143669292, -60018277, -1825187026, -1092321042, -2055648683, 17961608, 805243056, pgps_input);
	pgps_input_alt_SET(-2.4408553E38F, 1.2677126E38F, 3.1029278E38F, -2.7058241E38F, -7.266151E37F, -1.9179195E38F, 1.6950932E38F, 1.2812875E38F, -3.251877E38F, 3.396914E38F, pgps_input);
	pgps_input_hdop_SET(3.1855013E38F, 2.4298335E38F, 8.1277076E37F, 2.8149796E38F, 2.8765712E38F, -2.8055217E38F, 1.3111303E38F, 2.35222E38F, -1.0606832E38F, -1.6265231E38F, pgps_input);
	pgps_input_vdop_SET(-2.5306225E38F, 1.6920905E38F, 2.5037428E38F, -2.2223787E38F, 2.9681319E38F, -8.1014044E37F, 1.1419434E38F, -6.651814E37F, 2.7514768E38F, -2.8852825E38F, pgps_input);
	pgps_input_vn_SET(-3.8330727E37F, -2.6571301E38F, -2.7436772E38F, 7.1151396E36F, 2.8899217E38F, 3.1939791E38F, 2.1919153E38F, 1.1217046E38F, 3.3754318E37F, -3.258548E38F, pgps_input);
	pgps_input_ve_SET(-6.819974E37F, 7.5706315E37F, -1.9034031E38F, 2.1808083E38F, 1.1674749E38F, 3.1091753E38F, 8.045506E37F, 3.3469049E38F, 2.1502175E38F, 1.4825878E38F, pgps_input);
	pgps_input_vd_SET(-1.742671E38F, -1.4242571E38F, 2.0902485E38F, -2.7262627E38F, 2.8056193E37F, -7.4361145E37F, -3.2497296E38F, 2.7439348E38F, 3.0842268E38F, 9.826691E37F, pgps_input);
	pgps_input_speed_accuracy_SET(-3.072741E38F, 2.3605338E38F, -3.385472E38F, -3.0826206E38F, -2.1734756E38F, -2.5333385E38F, 3.191434E38F, 1.1793349E38F, 7.7514456E37F, -1.6857795E38F, pgps_input);
	pgps_input_horiz_accuracy_SET(-1.0713395E38F, -1.5064573E38F, 3.085292E38F, 8.761505E37F, 1.7787468E38F, -1.8368255E38F, 1.4487972E38F, 2.1710549E38F, 2.7838015E38F, 8.801492E37F, pgps_input);
	pgps_input_vert_accuracy_SET(1.6585167E38F, -5.592614E36F, 4.8710103E37F, -2.8777861E38F, 3.0789477E38F, -2.8977446E38F, 2.776605E38F, 1.6827488E38F, 3.0428906E38F, -1.8829875E38F, pgps_input);
	pgps_input_satellites_visible_SET(-24, 111, -114, -100, 66, 95, -63, -19, -94, 114, pgps_input);
	pgps_input_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, pgps_input);
	
}


void read_COMMAND_LONG(pcommand_long_COMMAND_LONG *const pcommand_long) {
	int8_t    target_system    = pcommand_long_target_system_GET(pcommand_long);
	int8_t    target_component = pcommand_long_target_component_GET(pcommand_long);
	int8_t    confirmation     = pcommand_long_confirmation_GET(pcommand_long);
	float     param1           = pcommand_long_param1_GET(pcommand_long);
	float     param2           = pcommand_long_param2_GET(pcommand_long);
	float     param3           = pcommand_long_param3_GET(pcommand_long);
	float     param4           = pcommand_long_param4_GET(pcommand_long);
	float     param5           = pcommand_long_param5_GET(pcommand_long);
	float     param6           = pcommand_long_param6_GET(pcommand_long);
	float     param7           = pcommand_long_param7_GET(pcommand_long);
	e_MAV_CMD item_command;
	if (pcommand_long_command_GET(pcommand_long, &item_command)) {
		e_MAV_CMD_MAV_CMD_NAV_WAYPOINT = item_command;
	}
	
}


void write_COMMAND_LONG(pcommand_long_COMMAND_LONG *const pcommand_long) {
	pcommand_long_target_system_SET(16, -88, -96, -89, 37, -88, 15, 75, 123, -79, pcommand_long);
	pcommand_long_target_component_SET(85, -55, 47, 9, -6, -38, -74, 102, -97, 41, pcommand_long);
	pcommand_long_confirmation_SET(46, -27, 122, -75, -36, -3, -8, 42, 113, -91, pcommand_long);
	pcommand_long_param1_SET(5.44202E36F, 2.3585047E38F, -1.5238917E37F, 2.50237E38F, 1.3727804E38F, -1.8155085E38F, 3.1781005E38F, -2.573266E38F, -1.8177881E37F, -4.6529734E37F, pcommand_long);
	pcommand_long_param2_SET(3.0766825E38F, 3.0825013E38F, -2.8922688E37F, 1.4260167E38F, 8.828449E37F, -2.4241071E38F, -2.5228066E38F, 1.957369E38F, -1.8927326E38F, 1.1729598E38F, pcommand_long);
	pcommand_long_param3_SET(-6.5393267E37F, 1.2750969E38F, -1.1297291E38F, 2.405045E38F, -1.6112701E38F, 6.2358654E37F, -1.3065034E38F, 2.9764069E38F, -2.7053866E38F, -1.4992189E38F, pcommand_long);
	pcommand_long_param4_SET(-1.9210772E38F, 2.5822763E38F, -1.1235872E38F, -8.893622E37F, -3.0060056E38F, -2.8985263E38F, -4.834021E37F, -1.4622055E38F, -2.5693643E38F, 3.3621984E38F, pcommand_long);
	pcommand_long_param5_SET(-2.9362986E38F, 2.0366588E38F, 1.5013552E38F, 3.025635E38F, -5.766444E37F, 2.1138477E38F, 5.807832E37F, -9.529448E37F, -2.3889366E38F, -2.9859496E38F, pcommand_long);
	pcommand_long_param6_SET(3.263309E38F, 1.776998E38F, -2.30939E38F, -2.2690617E38F, 4.1758978E35F, -6.7973864E37F, 2.4612165E38F, -2.9272093E38F, -8.36265E37F, 6.68381E37F, pcommand_long);
	pcommand_long_param7_SET(1.4143454E37F, 2.8547187E38F, -1.707255E38F, -2.674513E38F, -4.8195448E36F, -2.8062127E38F, 2.3655597E37F, -2.4581824E37F, -1.6719136E38F, 2.4332158E38F, pcommand_long);
	pcommand_long_command_SET(e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL, e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3, e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE, e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION, e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL, e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION, e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL, e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT, e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL, e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, pcommand_long);
	
}


void read_COMPASSMOT_STATUS(pcompassmot_status_COMPASSMOT_STATUS *const pcompassmot_status) {
	int16_t throttle      = pcompassmot_status_throttle_GET(pcompassmot_status);
	int16_t interference  = pcompassmot_status_interference_GET(pcompassmot_status);
	float   current       = pcompassmot_status_current_GET(pcompassmot_status);
	float   CompensationX = pcompassmot_status_CompensationX_GET(pcompassmot_status);
	float   CompensationY = pcompassmot_status_CompensationY_GET(pcompassmot_status);
	float   CompensationZ = pcompassmot_status_CompensationZ_GET(pcompassmot_status);
	
}


void read_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA *const plog_request_data) {
	int16_t id               = plog_request_data_id_GET(plog_request_data);
	int32_t ofs              = plog_request_data_ofs_GET(plog_request_data);
	int32_t count            = plog_request_data_count_GET(plog_request_data);
	int8_t  target_system    = plog_request_data_target_system_GET(plog_request_data);
	int8_t  target_component = plog_request_data_target_component_GET(plog_request_data);
	
}


void write_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA *const plog_request_data) {
	plog_request_data_id_SET(-10177, 27985, 1922, -28184, -17841, -28608, -28533, 23570, 30948, 2378, plog_request_data);
	plog_request_data_ofs_SET(-1197801525, -2127507529, 171723848, -92941980, -1509707656, -660044916, 376557557, 1928297748, 1500764653, 450795339, plog_request_data);
	plog_request_data_count_SET(-1407760628, -1800735803, -885815653, 1970240578, -962728232, -1351353610, 2006326408, 1002498262, -1939726683, -1769093379, plog_request_data);
	plog_request_data_target_system_SET(71, -119, 111, 126, 92, -116, 60, 81, -112, 78, plog_request_data);
	plog_request_data_target_component_SET(-37, -101, -106, 29, 92, -46, 127, -79, -8, 108, plog_request_data);
	
}


void read_GPS_RAW_INT(pgps_raw_int_GPS_RAW_INT *const pgps_raw_int) {
	int16_t        eph                = pgps_raw_int_eph_GET(pgps_raw_int);
	int16_t        epv                = pgps_raw_int_epv_GET(pgps_raw_int);
	int16_t        vel                = pgps_raw_int_vel_GET(pgps_raw_int);
	int16_t        cog                = pgps_raw_int_cog_GET(pgps_raw_int);
	int64_t        time_usec          = pgps_raw_int_time_usec_GET(pgps_raw_int);
	int32_t        lat                = pgps_raw_int_lat_GET(pgps_raw_int);
	int32_t        lon                = pgps_raw_int_lon_GET(pgps_raw_int);
	int32_t        alt                = pgps_raw_int_alt_GET(pgps_raw_int);
	int8_t         satellites_visible = pgps_raw_int_satellites_visible_GET(pgps_raw_int);
	e_GPS_FIX_TYPE item_fix_type;
	if (pgps_raw_int_fix_type_GET(pgps_raw_int, &item_fix_type)) {
		e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS = item_fix_type;
	}
	int32_t item_alt_ellipsoid;
	if (pgps_raw_int_alt_ellipsoid_GET(pgps_raw_int, &item_alt_ellipsoid)) {
		some_int32_t = item_alt_ellipsoid;
	}
	int32_t item_h_acc;
	if (pgps_raw_int_h_acc_GET(pgps_raw_int, &item_h_acc)) {
		some_int32_t = item_h_acc;
	}
	int32_t item_v_acc;
	if (pgps_raw_int_v_acc_GET(pgps_raw_int, &item_v_acc)) {
		some_int32_t = item_v_acc;
	}
	int32_t item_vel_acc;
	if (pgps_raw_int_vel_acc_GET(pgps_raw_int, &item_vel_acc)) {
		some_int32_t = item_vel_acc;
	}
	int32_t item_hdg_acc;
	if (pgps_raw_int_hdg_acc_GET(pgps_raw_int, &item_hdg_acc)) {
		some_int32_t = item_hdg_acc;
	}
	
}


void write_GPS_RAW_INT(pgps_raw_int_GPS_RAW_INT *const pgps_raw_int) {
	pgps_raw_int_eph_SET(864, 17880, -20658, -28422, 29565, -32695, 20075, -4247, -6501, -553, pgps_raw_int);
	pgps_raw_int_epv_SET(15838, 10645, 24606, 1586, -18770, -20489, 20298, 30507, 30167, -21380, pgps_raw_int);
	pgps_raw_int_vel_SET(28446, 24728, 14169, 10493, -3720, 32293, -6719, 13038, 7835, -16860, pgps_raw_int);
	pgps_raw_int_cog_SET(19529, -13399, -24744, 23695, 19044, 5998, -30699, 22618, -28861, 3434, pgps_raw_int);
	pgps_raw_int_time_usec_SET(-8872711240324985492L, -393807712850530497L, 6138761984581087421L, -8555370726408887529L, -9161404949311489249L, 5983454955517117642L, -1271854698885730686L, -39433887942269347L, 9009878011065900567L, 7594107611243848379L, pgps_raw_int);
	pgps_raw_int_lat_SET(-1947724652, -429303880, -1443889704, -579570862, 411174244, 269794577, -706060941, 1929969105, 69537098, 560326209, pgps_raw_int);
	pgps_raw_int_lon_SET(1675502513, -888189338, 1107983119, 1164147717, 1719604412, -1019368027, 1558471485, 756674435, 2110672261, -1265440543, pgps_raw_int);
	pgps_raw_int_alt_SET(62048774, 51898188, -1304481253, -717719692, -2005580184, 1162563234, -800540610, 1710859259, -1098422098, 1852019849, pgps_raw_int);
	pgps_raw_int_satellites_visible_SET(116, 126, -22, 56, 95, -13, 48, -89, 26, 87, pgps_raw_int);
	pgps_raw_int_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, pgps_raw_int);
	pgps_raw_int_alt_ellipsoid_SET(-1925910445, 98503687, -527610588, -80387901, -750166901, -1837892636, 653247983, 264494104, 1845906077, -1123176330, pgps_raw_int);
	pgps_raw_int_h_acc_SET(467453518, 554278685, -1893677095, -1957985965, -1199465583, -1371808271, -183837406, -1309848469, -703483184, -1779061195, pgps_raw_int);
	pgps_raw_int_v_acc_SET(-925526780, 1599192426, 744287006, -1708283770, 1899492378, 2119375523, -1593784425, 1945066650, -1482014409, -1092463055, pgps_raw_int);
	pgps_raw_int_vel_acc_SET(-1770475640, 898617509, -142970815, 1872911334, 516144882, 777763878, 402914006, 1509389776, 92789810, 410660175, pgps_raw_int);
	pgps_raw_int_hdg_acc_SET(1903221020, 61555207, 2099231839, -465389582, -886682382, 1477397386, -692153710, 1371448060, 1228253144, 1886555316, pgps_raw_int);
	
}


void read_CAMERA_STATUS(pcamera_status_CAMERA_STATUS *const pcamera_status) {
	int16_t               img_idx       = pcamera_status_img_idx_GET(pcamera_status);
	int64_t               time_usec     = pcamera_status_time_usec_GET(pcamera_status);
	int8_t                target_system = pcamera_status_target_system_GET(pcamera_status);
	int8_t                cam_idx       = pcamera_status_cam_idx_GET(pcamera_status);
	float                 p1            = pcamera_status_p1_GET(pcamera_status);
	float                 p2            = pcamera_status_p2_GET(pcamera_status);
	float                 p3            = pcamera_status_p3_GET(pcamera_status);
	float                 p4            = pcamera_status_p4_GET(pcamera_status);
	e_CAMERA_STATUS_TYPES item_event_id;
	if (pcamera_status_event_id_GET(pcamera_status, &item_event_id)) {
		e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_HEARTBEAT = item_event_id;
	}
	
}


void read_RC_CHANNELS_SCALED(prc_channels_scaled_RC_CHANNELS_SCALED *const prc_channels_scaled) {
	int32_t time_boot_ms = prc_channels_scaled_time_boot_ms_GET(prc_channels_scaled);
	int8_t  port         = prc_channels_scaled_port_GET(prc_channels_scaled);
	int16_t chan1_scaled = prc_channels_scaled_chan1_scaled_GET(prc_channels_scaled);
	int16_t chan2_scaled = prc_channels_scaled_chan2_scaled_GET(prc_channels_scaled);
	int16_t chan3_scaled = prc_channels_scaled_chan3_scaled_GET(prc_channels_scaled);
	int16_t chan4_scaled = prc_channels_scaled_chan4_scaled_GET(prc_channels_scaled);
	int16_t chan5_scaled = prc_channels_scaled_chan5_scaled_GET(prc_channels_scaled);
	int16_t chan6_scaled = prc_channels_scaled_chan6_scaled_GET(prc_channels_scaled);
	int16_t chan7_scaled = prc_channels_scaled_chan7_scaled_GET(prc_channels_scaled);
	int16_t chan8_scaled = prc_channels_scaled_chan8_scaled_GET(prc_channels_scaled);
	int8_t  rssi         = prc_channels_scaled_rssi_GET(prc_channels_scaled);
	
}


void write_RC_CHANNELS_SCALED(prc_channels_scaled_RC_CHANNELS_SCALED *const prc_channels_scaled) {
	prc_channels_scaled_time_boot_ms_SET(493439284, 209273148, -2049190846, 505414491, -1504202807, 1218035222, -839707771, 1918331081, -16424197, -496505797, prc_channels_scaled);
	prc_channels_scaled_port_SET(22, -114, 88, -125, -70, -55, 112, 109, 76, 26, prc_channels_scaled);
	prc_channels_scaled_chan1_scaled_SET(-12895, -29330, 18889, -21620, -4155, 7649, -31048, -5535, 9985, 32371, prc_channels_scaled);
	prc_channels_scaled_chan2_scaled_SET(-20268, 26203, -32113, -8890, -28794, 31833, -5331, 29887, 18056, -5705, prc_channels_scaled);
	prc_channels_scaled_chan3_scaled_SET(-9607, 23439, 30324, 3885, -10726, 1530, 7484, -3739, -26251, 13437, prc_channels_scaled);
	prc_channels_scaled_chan4_scaled_SET(30952, -17519, -14135, 4154, 3911, 28878, -26972, -6659, 15747, 31976, prc_channels_scaled);
	prc_channels_scaled_chan5_scaled_SET(-16321, -2129, -19344, 27402, 9057, -11109, 6374, 24726, -1324, 9579, prc_channels_scaled);
	prc_channels_scaled_chan6_scaled_SET(-5837, 24517, -14492, 16635, 1105, 21292, 11721, -6622, -3656, 10464, prc_channels_scaled);
	prc_channels_scaled_chan7_scaled_SET(-7450, -26743, -5609, 26027, -14392, -11897, -23376, 28666, 29842, -28553, prc_channels_scaled);
	prc_channels_scaled_chan8_scaled_SET(-17474, 466, -26033, -3193, -7312, 9381, -28789, -2123, -18467, 4764, prc_channels_scaled);
	prc_channels_scaled_rssi_SET(-36, 5, -84, 34, 33, 17, -35, -64, 101, 104, prc_channels_scaled);
	
}


void read_CAMERA_SETTINGS(pcamera_settings_CAMERA_SETTINGS *const pcamera_settings) {
	int32_t       time_boot_ms = pcamera_settings_time_boot_ms_GET(pcamera_settings);
	e_CAMERA_MODE item_mode_id;
	if (pcamera_settings_mode_id_GET(pcamera_settings, &item_mode_id)) {
		e_CAMERA_MODE_CAMERA_MODE_IMAGE = item_mode_id;
	}
	
}


void write_CAMERA_SETTINGS(pcamera_settings_CAMERA_SETTINGS *const pcamera_settings) {
	pcamera_settings_time_boot_ms_SET(-381237385, -962440683, 685994882, -1076449828, -823312774, -1921131903, 1964609534, 1296500835, 1512072947, 88774653, pcamera_settings);
	pcamera_settings_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, e_CAMERA_MODE_CAMERA_MODE_IMAGE, e_CAMERA_MODE_CAMERA_MODE_IMAGE, e_CAMERA_MODE_CAMERA_MODE_IMAGE, e_CAMERA_MODE_CAMERA_MODE_IMAGE, e_CAMERA_MODE_CAMERA_MODE_VIDEO, e_CAMERA_MODE_CAMERA_MODE_VIDEO, e_CAMERA_MODE_CAMERA_MODE_IMAGE, e_CAMERA_MODE_CAMERA_MODE_VIDEO, e_CAMERA_MODE_CAMERA_MODE_IMAGE, pcamera_settings);
	
}


void read_DEVICE_OP_READ_REPLY(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *const pdevice_op_read_reply) {
	int32_t                    request_id = pdevice_op_read_reply_request_id_GET(pdevice_op_read_reply);
	int8_t                     result     = pdevice_op_read_reply_result_GET(pdevice_op_read_reply);
	int8_t                     regstart   = pdevice_op_read_reply_regstart_GET(pdevice_op_read_reply);
	int8_t                     count      = pdevice_op_read_reply_count_GET(pdevice_op_read_reply);
	Vdevice_op_read_reply_daTa item_daTa  = pdevice_op_read_reply_daTa_GET(pdevice_op_read_reply);
	for (size_t                index      = 0; index < item_daTa.len; index++)
		some_int8_t = vdevice_op_read_reply_daTa_GET(&item_daTa, index);
	
}


void read_RAW_PRESSURE(praw_pressure_RAW_PRESSURE *const praw_pressure) {
	int64_t time_usec   = praw_pressure_time_usec_GET(praw_pressure);
	int16_t press_abs   = praw_pressure_press_abs_GET(praw_pressure);
	int16_t press_diff1 = praw_pressure_press_diff1_GET(praw_pressure);
	int16_t press_diff2 = praw_pressure_press_diff2_GET(praw_pressure);
	int16_t temperature = praw_pressure_temperature_GET(praw_pressure);
	
}


void write_RAW_PRESSURE(praw_pressure_RAW_PRESSURE *const praw_pressure) {
	praw_pressure_time_usec_SET(6000989655783211933L, 4697895639441596031L, -6651050482369869034L, 5395791979581396949L, 4411013685752259001L, -833362159823872505L, -4696740166770578982L, -907014824268511649L, -663667974728888818L, -8113415741243962196L, praw_pressure);
	praw_pressure_press_abs_SET(-20415, 22298, 10314, -29274, 28127, -20992, -15748, -7901, 28001, 17176, praw_pressure);
	praw_pressure_press_diff1_SET(23717, -976, 24013, -30827, -27098, -8366, 8115, 23508, -32440, -3784, praw_pressure);
	praw_pressure_press_diff2_SET(7511, 5426, -16700, 8959, 28799, 21252, -19885, 14485, -10283, -13236, praw_pressure);
	praw_pressure_temperature_SET(-28504, 25912, 13143, -19396, -15556, 30466, -15241, -840, 26165, -28523, praw_pressure);
	
}


void read_DIGICAM_CONTROL(pdigicam_control_DIGICAM_CONTROL *const pdigicam_control) {
	int8_t target_system    = pdigicam_control_target_system_GET(pdigicam_control);
	int8_t target_component = pdigicam_control_target_component_GET(pdigicam_control);
	int8_t session          = pdigicam_control_session_GET(pdigicam_control);
	int8_t zoom_pos         = pdigicam_control_zoom_pos_GET(pdigicam_control);
	int8_t zoom_step        = pdigicam_control_zoom_step_GET(pdigicam_control);
	int8_t focus_lock       = pdigicam_control_focus_lock_GET(pdigicam_control);
	int8_t shot             = pdigicam_control_shot_GET(pdigicam_control);
	int8_t command_id       = pdigicam_control_command_id_GET(pdigicam_control);
	int8_t extra_param      = pdigicam_control_extra_param_GET(pdigicam_control);
	float  extra_value      = pdigicam_control_extra_value_GET(pdigicam_control);
	
}


void read_NAMED_VALUE_FLOAT(pnamed_value_float_NAMED_VALUE_FLOAT *const pnamed_value_float) {
	int32_t                 time_boot_ms = pnamed_value_float_time_boot_ms_GET(pnamed_value_float);
	float                   value        = pnamed_value_float_value_GET(pnamed_value_float);
	Vnamed_value_float_name item_name;
	if (pnamed_value_float_name_GET(pnamed_value_float, &item_name)) {
		memcpy(some_string, item_name.bytes, item_name.len);
	}
	
}


void write_NAMED_VALUE_FLOAT(pnamed_value_float_NAMED_VALUE_FLOAT *const pnamed_value_float) {
	pnamed_value_float_time_boot_ms_SET(1516887855, -1309628024, -1211480131, -690043670, -765690330, -1631745625, -1545899403, 456787357, -1186747871, -1544785445, pnamed_value_float);
	pnamed_value_float_value_SET(-1.5456194E38F, 5.4445005E37F, -3.357842E38F, -3.2894168E38F, -2.276749E38F, -5.5758125E36F, -4.982263E37F, 5.675507E36F, -3.7869193E37F, 2.1815222E38F, pnamed_value_float);
	pnamed_value_float_name_SET(some_string, strlen(some_string), pnamed_value_float);
	
}


void read_GOPRO_HEARTBEAT(pgopro_heartbeat_GOPRO_HEARTBEAT *const pgopro_heartbeat) {
	e_GOPRO_HEARTBEAT_STATUS item_status;
	if (pgopro_heartbeat_status_GET(pgopro_heartbeat, &item_status)) {
		e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED = item_status;
	}
	e_GOPRO_CAPTURE_MODE item_capture_mode;
	if (pgopro_heartbeat_capture_mode_GET(pgopro_heartbeat, &item_capture_mode)) {
		e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO = item_capture_mode;
	}
	e_GOPRO_HEARTBEAT_FLAGS item_flags;
	if (pgopro_heartbeat_flags_GET(pgopro_heartbeat, &item_flags)) {
		e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING = item_flags;
	}
	
}


void read_ATTITUDE(pattitude_ATTITUDE *const pattitude) {
	int32_t time_boot_ms = pattitude_time_boot_ms_GET(pattitude);
	float   roll         = pattitude_roll_GET(pattitude);
	float   pitch        = pattitude_pitch_GET(pattitude);
	float   yaw          = pattitude_yaw_GET(pattitude);
	float   rollspeed    = pattitude_rollspeed_GET(pattitude);
	float   pitchspeed   = pattitude_pitchspeed_GET(pattitude);
	float   yawspeed     = pattitude_yawspeed_GET(pattitude);
	
}


void write_ATTITUDE(pattitude_ATTITUDE *const pattitude) {
	pattitude_time_boot_ms_SET(-893823429, 1061481642, 662021749, 365423943, -1482210031, 1073730264, 1100547983, 213704779, 1565935325, -1844715340, pattitude);
	pattitude_roll_SET(-1.1918295E38F, 1.9852644E38F, -2.2299432E38F, -2.7925657E38F, -3.2566346E38F, 3.0009768E38F, 3.1957733E38F, 7.0404244E37F, 3.0816047E38F, 2.2437188E38F, pattitude);
	pattitude_pitch_SET(2.5162562E38F, 1.4296935E38F, 1.3795131E38F, -1.3285641E37F, -9.673398E36F, 2.4157972E38F, 3.050988E38F, -1.7079693E38F, 2.1592787E38F, 1.2815828E38F, pattitude);
	pattitude_yaw_SET(1.0075689E38F, 1.0826646E38F, -2.8639789E38F, -6.306599E37F, -7.752603E37F, 1.7949807E38F, -1.5933219E38F, -2.4691532E38F, -1.1956043E38F, -2.8112744E38F, pattitude);
	pattitude_rollspeed_SET(2.5766538E38F, 1.7895953E38F, -1.7135659E38F, -3.3838562E38F, -3.341656E38F, 1.0019235E38F, 1.5035215E38F, -1.7802145E38F, -2.2138457E38F, -2.4738137E38F, pattitude);
	pattitude_pitchspeed_SET(-1.932039E38F, 1.6679532E38F, 2.6231463E38F, 2.574022E38F, -1.6309436E38F, -6.8094316E37F, 3.0515804E38F, 2.8731449E38F, -2.8203709E38F, 7.8109263E37F, pattitude);
	pattitude_yawspeed_SET(-3.0027178E38F, -2.1916906E38F, 3.30749E38F, 3.1218598E38F, -2.8112312E38F, -2.566132E38F, -2.628535E38F, -1.6089197E38F, -6.0295223E37F, 3.3377068E37F, pattitude);
	
}


void read_MISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *const pmission_write_partial_list) {
	int8_t             target_system    = pmission_write_partial_list_target_system_GET(pmission_write_partial_list);
	int8_t             target_component = pmission_write_partial_list_target_component_GET(pmission_write_partial_list);
	int16_t            start_index      = pmission_write_partial_list_start_index_GET(pmission_write_partial_list);
	int16_t            end_index        = pmission_write_partial_list_end_index_GET(pmission_write_partial_list);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_write_partial_list_mission_type_GET(pmission_write_partial_list, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *const pmission_write_partial_list) {
	pmission_write_partial_list_target_system_SET(-33, 61, 14, 23, 7, -49, 123, 48, -58, -19, pmission_write_partial_list);
	pmission_write_partial_list_target_component_SET(-38, -58, -85, 11, -14, 22, -54, 11, 21, -92, pmission_write_partial_list);
	pmission_write_partial_list_start_index_SET(6038, -12605, -32552, -30698, -25285, 25844, -31473, 32063, -10294, -17968, pmission_write_partial_list);
	pmission_write_partial_list_end_index_SET(4281, -304, 18188, 15595, -7287, 26141, 11396, 18562, 11988, 27617, pmission_write_partial_list);
	pmission_write_partial_list_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, pmission_write_partial_list);
	
}


void read_AHRS2(pahrs2_AHRS2 *const pahrs2) {
	float   roll     = pahrs2_roll_GET(pahrs2);
	float   pitch    = pahrs2_pitch_GET(pahrs2);
	float   yaw      = pahrs2_yaw_GET(pahrs2);
	float   altitude = pahrs2_altitude_GET(pahrs2);
	int32_t lat      = pahrs2_lat_GET(pahrs2);
	int32_t lng      = pahrs2_lng_GET(pahrs2);
	
}


void read_LOG_ERASE(plog_erase_LOG_ERASE *const plog_erase) {
	int8_t target_system    = plog_erase_target_system_GET(plog_erase);
	int8_t target_component = plog_erase_target_component_GET(plog_erase);
	
}


void write_LOG_ERASE(plog_erase_LOG_ERASE *const plog_erase) {
	plog_erase_target_system_SET(-45, 34, -81, -93, -22, 9, -22, 0, 11, 73, plog_erase);
	plog_erase_target_component_SET(58, 54, -63, -34, -69, 124, 61, -4, 64, -60, plog_erase);
	
}


void read_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST *const pterrain_request) {
	int16_t grid_spacing = pterrain_request_grid_spacing_GET(pterrain_request);
	int64_t mask         = pterrain_request_mask_GET(pterrain_request);
	int32_t lat          = pterrain_request_lat_GET(pterrain_request);
	int32_t lon          = pterrain_request_lon_GET(pterrain_request);
	
}


void write_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST *const pterrain_request) {
	pterrain_request_grid_spacing_SET(27667, -18103, 29073, -25795, 9976, 3244, 2726, 7096, 26411, 14958, pterrain_request);
	pterrain_request_mask_SET(547470640137406178L, -8328699539233302226L, 1086060300980379765L, -5769382998682341872L, -338274270281795833L, 5262249874586400837L, -4449324372038011365L, -7078747298995121666L, 6322322094800120090L, -2724737637927911708L, pterrain_request);
	pterrain_request_lat_SET(633113564, -2081219501, -1706927540, 1930848261, 229234478, -856781336, -1412609190, 167628849, 316882957, -494196455, pterrain_request);
	pterrain_request_lon_SET(1245741940, 208383129, -1368800939, -1599452002, 193841621, -1229306783, -1507319740, 566878816, -1608646756, -1796083462, pterrain_request);
	
}


void read_MOUNT_STATUS(pmount_status_MOUNT_STATUS *const pmount_status) {
	int8_t  target_system    = pmount_status_target_system_GET(pmount_status);
	int8_t  target_component = pmount_status_target_component_GET(pmount_status);
	int32_t pointing_a       = pmount_status_pointing_a_GET(pmount_status);
	int32_t pointing_b       = pmount_status_pointing_b_GET(pmount_status);
	int32_t pointing_c       = pmount_status_pointing_c_GET(pmount_status);
	
}


void read_MANUAL_SETPOINT(pmanual_setpoint_MANUAL_SETPOINT *const pmanual_setpoint) {
	int32_t time_boot_ms           = pmanual_setpoint_time_boot_ms_GET(pmanual_setpoint);
	float   roll                   = pmanual_setpoint_roll_GET(pmanual_setpoint);
	float   pitch                  = pmanual_setpoint_pitch_GET(pmanual_setpoint);
	float   yaw                    = pmanual_setpoint_yaw_GET(pmanual_setpoint);
	float   thrust                 = pmanual_setpoint_thrust_GET(pmanual_setpoint);
	int8_t  mode_switch            = pmanual_setpoint_mode_switch_GET(pmanual_setpoint);
	int8_t  manual_override_switch = pmanual_setpoint_manual_override_switch_GET(pmanual_setpoint);
	
}


void write_MANUAL_SETPOINT(pmanual_setpoint_MANUAL_SETPOINT *const pmanual_setpoint) {
	pmanual_setpoint_time_boot_ms_SET(1063948199, 1433642578, 559111290, 366923884, -355315165, 97201, -1452725536, -2113643533, 446065458, -1797545968, pmanual_setpoint);
	pmanual_setpoint_roll_SET(1.1411288E38F, 1.7430431E38F, -2.6247943E38F, -2.14787E38F, -3.2788868E38F, -1.2727021E38F, -2.716609E38F, 2.7624356E38F, 1.5422142E36F, -1.3671673E37F, pmanual_setpoint);
	pmanual_setpoint_pitch_SET(-1.7115135E37F, 2.7363798E38F, -9.913594E37F, -2.3492326E38F, -2.989204E38F, 7.7104483E37F, 4.7327796E37F, 3.079532E38F, -2.7885165E38F, -6.1482246E37F, pmanual_setpoint);
	pmanual_setpoint_yaw_SET(1.1606026E38F, -2.23276E38F, 3.2123982E38F, 1.8702653E38F, -3.3060664E38F, -7.7013993E37F, -2.636813E37F, -3.6199091E37F, -2.2920747E38F, -1.9769727E38F, pmanual_setpoint);
	pmanual_setpoint_thrust_SET(5.6659423E37F, -6.039238E37F, -2.283298E38F, -2.4460347E38F, -1.2339682E38F, 3.829719E37F, -2.068599E38F, -3.13284E38F, 1.8300849E38F, 2.7742318E38F, pmanual_setpoint);
	pmanual_setpoint_mode_switch_SET(-23, -104, 35, 11, -3, -95, 118, 36, -86, -13, pmanual_setpoint);
	pmanual_setpoint_manual_override_switch_SET(-69, -48, 12, 106, 109, 59, -17, 100, -6, 98, pmanual_setpoint);
	
}


void read_PID_TUNING(ppid_tuning_PID_TUNING *const ppid_tuning) {
	float             desired  = ppid_tuning_desired_GET(ppid_tuning);
	float             achieved = ppid_tuning_achieved_GET(ppid_tuning);
	float             FF       = ppid_tuning_FF_GET(ppid_tuning);
	float             P        = ppid_tuning_P_GET(ppid_tuning);
	float             I        = ppid_tuning_I_GET(ppid_tuning);
	float             D        = ppid_tuning_D_GET(ppid_tuning);
	e_PID_TUNING_AXIS item_axis;
	if (ppid_tuning_axis_GET(ppid_tuning, &item_axis)) {
		e_PID_TUNING_AXIS_PID_TUNING_ROLL = item_axis;
	}
	
}


void read_SAFETY_ALLOWED_AREA(psafety_allowed_area_SAFETY_ALLOWED_AREA *const psafety_allowed_area) {
	float       p1x = psafety_allowed_area_p1x_GET(psafety_allowed_area);
	float       p1y = psafety_allowed_area_p1y_GET(psafety_allowed_area);
	float       p1z = psafety_allowed_area_p1z_GET(psafety_allowed_area);
	float       p2x = psafety_allowed_area_p2x_GET(psafety_allowed_area);
	float       p2y = psafety_allowed_area_p2y_GET(psafety_allowed_area);
	float       p2z = psafety_allowed_area_p2z_GET(psafety_allowed_area);
	e_MAV_FRAME item_frame;
	if (psafety_allowed_area_frame_GET(psafety_allowed_area, &item_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_frame;
	}
	
}


void write_SAFETY_ALLOWED_AREA(psafety_allowed_area_SAFETY_ALLOWED_AREA *const psafety_allowed_area) {
	psafety_allowed_area_p1x_SET(-6.713317E37F, -9.308023E37F, 1.9096134E38F, -2.7934082E38F, -1.3680263E38F, -7.5361423E37F, 9.33191E37F, -2.0494816E37F, -2.9347314E38F, 2.750364E38F, psafety_allowed_area);
	psafety_allowed_area_p1y_SET(3.2039025E38F, -2.575867E38F, 2.795506E38F, -1.6113627E38F, 2.621619E38F, -1.6670859E38F, 9.542585E37F, 2.9047331E38F, 1.467841E38F, 1.2558333E38F, psafety_allowed_area);
	psafety_allowed_area_p1z_SET(-2.0299836E38F, -1.837132E38F, 3.1050233E38F, -3.3978567E38F, -1.3814435E37F, 4.3590524E37F, 1.0255022E38F, 3.1620907E38F, -3.6847928E37F, -2.3555865E38F, psafety_allowed_area);
	psafety_allowed_area_p2x_SET(4.833785E37F, 2.668539E38F, 1.6180534E38F, 3.6246382E37F, 2.7003612E37F, 5.5216695E37F, -1.3149518E38F, -1.3791652E38F, -1.4764543E38F, -1.2008417E38F, psafety_allowed_area);
	psafety_allowed_area_p2y_SET(3.1808512E38F, -1.7653398E38F, 1.5823536E38F, -2.2861168E38F, 1.1744223E38F, 2.2201397E38F, 1.5882869E38F, -2.6848525E37F, 1.0647053E38F, -9.926126E37F, psafety_allowed_area);
	psafety_allowed_area_p2z_SET(1.9734209E38F, 8.961165E37F, 1.5957445E38F, 1.5542784E38F, -1.3369779E38F, 2.037151E38F, -9.362604E37F, 1.2228796E38F, 1.1701908E38F, -2.84968E38F, psafety_allowed_area);
	psafety_allowed_area_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, e_MAV_FRAME_MAV_FRAME_LOCAL_NED, e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, e_MAV_FRAME_MAV_FRAME_MISSION, psafety_allowed_area);
	
}


void read_OPTICAL_FLOW_RAD(poptical_flow_rad_OPTICAL_FLOW_RAD *const poptical_flow_rad) {
	int32_t integration_time_us    = poptical_flow_rad_integration_time_us_GET(poptical_flow_rad);
	int32_t time_delta_distance_us = poptical_flow_rad_time_delta_distance_us_GET(poptical_flow_rad);
	int64_t time_usec              = poptical_flow_rad_time_usec_GET(poptical_flow_rad);
	int8_t  sensor_id              = poptical_flow_rad_sensor_id_GET(poptical_flow_rad);
	float   integrated_x           = poptical_flow_rad_integrated_x_GET(poptical_flow_rad);
	float   integrated_y           = poptical_flow_rad_integrated_y_GET(poptical_flow_rad);
	float   integrated_xgyro       = poptical_flow_rad_integrated_xgyro_GET(poptical_flow_rad);
	float   integrated_ygyro       = poptical_flow_rad_integrated_ygyro_GET(poptical_flow_rad);
	float   integrated_zgyro       = poptical_flow_rad_integrated_zgyro_GET(poptical_flow_rad);
	int16_t temperature            = poptical_flow_rad_temperature_GET(poptical_flow_rad);
	int8_t  quality                = poptical_flow_rad_quality_GET(poptical_flow_rad);
	float   distance               = poptical_flow_rad_distance_GET(poptical_flow_rad);
	
}


void write_OPTICAL_FLOW_RAD(poptical_flow_rad_OPTICAL_FLOW_RAD *const poptical_flow_rad) {
	poptical_flow_rad_integration_time_us_SET(291211270, 1655384942, -19992841, 1685043966, 646902483, -286592861, -949028469, -1158285202, -1813290357, -661873521, poptical_flow_rad);
	poptical_flow_rad_time_delta_distance_us_SET(1821908661, -1441481423, -728866398, -808005770, 1683133066, -55905696, 1553072208, -574451403, 1758011691, -1241452224, poptical_flow_rad);
	poptical_flow_rad_time_usec_SET(2332347751583848096L, -8100584987283196954L, -1379160823597977792L, 7983702426358590079L, -4625617936004661576L, -866348606771212001L, -9005671081198746152L, 6059112354079412921L, 1081173993597565633L, -5027749100762009418L, poptical_flow_rad);
	poptical_flow_rad_sensor_id_SET(51, 53, 22, -49, -64, 79, -121, 54, 30, -4, poptical_flow_rad);
	poptical_flow_rad_integrated_x_SET(7.753724E37F, -2.0631665E38F, -2.472046E38F, -4.690785E37F, -4.0745148E37F, 2.9723583E38F, 3.2945499E38F, 1.0237396E38F, 4.4064397E37F, 8.774088E37F, poptical_flow_rad);
	poptical_flow_rad_integrated_y_SET(2.4958878E38F, -2.0670698E38F, -1.748708E38F, 9.627253E37F, 2.6509464E38F, -2.8945836E38F, 5.608818E37F, 3.657325E37F, -2.1135966E38F, -2.7513158E38F, poptical_flow_rad);
	poptical_flow_rad_integrated_xgyro_SET(-1.4053569E38F, -9.705161E37F, 2.057933E38F, -1.5655041E38F, -1.4984369E37F, 2.1788441E38F, -2.4010609E38F, 3.1703278E38F, -2.337818E38F, -1.1803409E38F, poptical_flow_rad);
	poptical_flow_rad_integrated_ygyro_SET(2.1880887E38F, 2.871964E38F, 3.3632484E38F, -3.845656E36F, -1.8066404E38F, 5.2927617E37F, -2.0213141E38F, 8.0996687E37F, 3.1009986E37F, -2.7658686E38F, poptical_flow_rad);
	poptical_flow_rad_integrated_zgyro_SET(-5.979719E36F, -8.107617E37F, -2.717537E35F, 1.1916182E38F, 1.2295461E38F, 3.7858623E35F, -1.5987857E38F, 3.2024681E38F, 8.458589E37F, 3.2569613E38F, poptical_flow_rad);
	poptical_flow_rad_temperature_SET(-21111, 22621, -21373, -31337, -7390, 17775, 10955, 17245, 20796, 20136, poptical_flow_rad);
	poptical_flow_rad_quality_SET(112, -8, -75, -32, -23, -5, -104, 90, -105, -106, poptical_flow_rad);
	poptical_flow_rad_distance_SET(1.0543735E38F, -2.1616753E38F, -6.853668E36F, 3.0770949E38F, 3.277924E38F, -7.921835E37F, -2.342539E38F, -8.347397E37F, -1.0718145E38F, -2.5803237E38F, poptical_flow_rad);
	
}


void read_LOG_DATA(plog_data_LOG_DATA *const plog_data) {
	int16_t        id        = plog_data_id_GET(plog_data);
	int32_t        ofs       = plog_data_ofs_GET(plog_data);
	int8_t         count     = plog_data_count_GET(plog_data);
	Vlog_data_daTa item_daTa = plog_data_daTa_GET(plog_data);
	for (size_t    index     = 0; index < item_daTa.len; index++)
		some_int8_t = vlog_data_daTa_GET(&item_daTa, index);
	
}


void write_LOG_DATA(plog_data_LOG_DATA *const plog_data) {
	plog_data_id_SET(3258, 28636, -22105, 7913, 14174, -30312, 10577, 30970, 24179, 26657, plog_data);
	plog_data_ofs_SET(1117719201, 1408739960, -1442449962, -1101250728, 71204129, 132360294, 1959609277, -1065654106, 1837074693, 1090019868, plog_data);
	plog_data_count_SET(98, -108, 6, 103, -30, -2, -122, -12, -19, 14, plog_data);
	plog_data_daTa_SET(&1, 117, 106, 7, 0, 106, -73, -27, -20, -20, plog_data);
	
}


void read_MISSION_CLEAR_ALL(pmission_clear_all_MISSION_CLEAR_ALL *const pmission_clear_all) {
	int8_t             target_system    = pmission_clear_all_target_system_GET(pmission_clear_all);
	int8_t             target_component = pmission_clear_all_target_component_GET(pmission_clear_all);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_clear_all_mission_type_GET(pmission_clear_all, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_CLEAR_ALL(pmission_clear_all_MISSION_CLEAR_ALL *const pmission_clear_all) {
	pmission_clear_all_target_system_SET(110, -89, -110, -74, 51, -105, 35, -100, 115, -92, pmission_clear_all);
	pmission_clear_all_target_component_SET(66, -46, 10, -117, 114, -39, 2, -111, -60, 124, pmission_clear_all);
	pmission_clear_all_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, pmission_clear_all);
	
}


void read_AHRS3(pahrs3_AHRS3 *const pahrs3) {
	float   roll     = pahrs3_roll_GET(pahrs3);
	float   pitch    = pahrs3_pitch_GET(pahrs3);
	float   yaw      = pahrs3_yaw_GET(pahrs3);
	float   altitude = pahrs3_altitude_GET(pahrs3);
	int32_t lat      = pahrs3_lat_GET(pahrs3);
	int32_t lng      = pahrs3_lng_GET(pahrs3);
	float   v1       = pahrs3_v1_GET(pahrs3);
	float   v2       = pahrs3_v2_GET(pahrs3);
	float   v3       = pahrs3_v3_GET(pahrs3);
	float   v4       = pahrs3_v4_GET(pahrs3);
	
}


void read_VICON_POSITION_ESTIMATE(pvicon_position_estimate_VICON_POSITION_ESTIMATE *const pvicon_position_estimate) {
	int64_t usec  = pvicon_position_estimate_usec_GET(pvicon_position_estimate);
	float   x     = pvicon_position_estimate_x_GET(pvicon_position_estimate);
	float   y     = pvicon_position_estimate_y_GET(pvicon_position_estimate);
	float   z     = pvicon_position_estimate_z_GET(pvicon_position_estimate);
	float   roll  = pvicon_position_estimate_roll_GET(pvicon_position_estimate);
	float   pitch = pvicon_position_estimate_pitch_GET(pvicon_position_estimate);
	float   yaw   = pvicon_position_estimate_yaw_GET(pvicon_position_estimate);
	
}


void write_VICON_POSITION_ESTIMATE(pvicon_position_estimate_VICON_POSITION_ESTIMATE *const pvicon_position_estimate) {
	pvicon_position_estimate_usec_SET(1846176042275463264L, -6800203630329879693L, -4679665462298647105L, -6572165796433641632L, -9180257262619110873L, 2214010289752357815L, 380457984617719218L, -7816524799056929268L, 7034034194722733749L, 1321386144939916007L, pvicon_position_estimate);
	pvicon_position_estimate_x_SET(-8.684636E36F, 1.1603051E38F, -2.6117509E38F, 3.0117944E38F, 2.960842E38F, -1.6919585E38F, 2.5010142E38F, -1.961146E38F, -1.5924879E38F, 1.703102E37F, pvicon_position_estimate);
	pvicon_position_estimate_y_SET(1.1026474E38F, 2.034576E38F, -2.971274E38F, -1.3130976E38F, 2.2333136E38F, 5.9292405E37F, -1.1228434E38F, 1.925929E38F, 1.0680316E38F, -1.7652014E38F, pvicon_position_estimate);
	pvicon_position_estimate_z_SET(2.3685427E38F, -5.154846E37F, -3.0009362E38F, -3.162032E38F, -1.6176971E38F, -4.8785803E36F, 3.303029E38F, -7.752097E37F, 2.1866506E38F, -2.1374473E38F, pvicon_position_estimate);
	pvicon_position_estimate_roll_SET(-2.256144E37F, 2.4809044E38F, 3.1611348E38F, 1.5201966E38F, -1.795691E38F, 4.175796E36F, -2.3865196E38F, -4.574457E37F, 2.246729E38F, 7.968822E37F, pvicon_position_estimate);
	pvicon_position_estimate_pitch_SET(1.2854843E38F, 2.803331E38F, 4.871054E37F, 2.7295152E38F, -2.7211948E38F, 2.405761E38F, 2.9933E38F, -2.6245262E38F, 1.321974E38F, -1.929099E38F, pvicon_position_estimate);
	pvicon_position_estimate_yaw_SET(-1.4418998E38F, -2.657044E38F, -2.2846266E38F, -3.2452581E38F, 1.2345383E38F, 8.802558E37F, -7.114613E36F, 2.1533706E38F, 1.401325E38F, 9.398874E37F, pvicon_position_estimate);
	
}


void read_GPS2_RTK(pgps2_rtk_GPS2_RTK *const pgps2_rtk) {
	int16_t wn                    = pgps2_rtk_wn_GET(pgps2_rtk);
	int32_t time_last_baseline_ms = pgps2_rtk_time_last_baseline_ms_GET(pgps2_rtk);
	int32_t tow                   = pgps2_rtk_tow_GET(pgps2_rtk);
	int32_t accuracy              = pgps2_rtk_accuracy_GET(pgps2_rtk);
	int8_t  rtk_receiver_id       = pgps2_rtk_rtk_receiver_id_GET(pgps2_rtk);
	int8_t  rtk_health            = pgps2_rtk_rtk_health_GET(pgps2_rtk);
	int8_t  rtk_rate              = pgps2_rtk_rtk_rate_GET(pgps2_rtk);
	int8_t  nsats                 = pgps2_rtk_nsats_GET(pgps2_rtk);
	int8_t  baseline_coords_type  = pgps2_rtk_baseline_coords_type_GET(pgps2_rtk);
	int32_t baseline_a_mm         = pgps2_rtk_baseline_a_mm_GET(pgps2_rtk);
	int32_t baseline_b_mm         = pgps2_rtk_baseline_b_mm_GET(pgps2_rtk);
	int32_t baseline_c_mm         = pgps2_rtk_baseline_c_mm_GET(pgps2_rtk);
	int32_t iar_num_hypotheses    = pgps2_rtk_iar_num_hypotheses_GET(pgps2_rtk);
	
}


void write_GPS2_RTK(pgps2_rtk_GPS2_RTK *const pgps2_rtk) {
	pgps2_rtk_wn_SET(17034, -16075, -11329, -25837, -25320, 28699, 28405, -23170, 4190, 19873, pgps2_rtk);
	pgps2_rtk_time_last_baseline_ms_SET(-26740520, 100278673, 1153466097, -1442338460, 1539663979, -2027795832, 1345590891, -1837970840, -1623996530, 1302063873, pgps2_rtk);
	pgps2_rtk_tow_SET(-222653050, 912906840, -1088857737, 1606494808, -9531074, -1811685690, 1604199428, 1791847998, 2065420868, 38417745, pgps2_rtk);
	pgps2_rtk_accuracy_SET(-2102978467, 875324650, 563924189, -1465978704, -824273395, -1462852789, -1475714026, 1464511300, -348283250, 1004448904, pgps2_rtk);
	pgps2_rtk_rtk_receiver_id_SET(-127, 31, 115, 120, 99, 59, -126, -11, 20, 41, pgps2_rtk);
	pgps2_rtk_rtk_health_SET(-11, 24, 1, 84, 104, 39, 13, 38, -56, 43, pgps2_rtk);
	pgps2_rtk_rtk_rate_SET(121, 114, 88, -80, 93, 19, 74, 15, 3, 47, pgps2_rtk);
	pgps2_rtk_nsats_SET(17, 95, 90, 81, -18, 8, 0, -22, -117, 122, pgps2_rtk);
	pgps2_rtk_baseline_coords_type_SET(106, -113, -27, 19, -121, 63, 29, -102, -54, 120, pgps2_rtk);
	pgps2_rtk_baseline_a_mm_SET(820070332, -1094711215, 609156180, 1186802409, 433060398, -590416460, -1105889701, 922400048, -1073585779, -1291241401, pgps2_rtk);
	pgps2_rtk_baseline_b_mm_SET(-1164678879, -1807529219, -1634047834, 296762426, 795330392, -1829413013, -1905751358, -365146838, 22056820, -900661604, pgps2_rtk);
	pgps2_rtk_baseline_c_mm_SET(-965927157, -1537971126, -322114387, -943310296, 1825350045, 750019022, 1216665700, -632205207, 1138348222, -601027160, pgps2_rtk);
	pgps2_rtk_iar_num_hypotheses_SET(-984709726, -899995284, -1851792627, -229141255, 986252054, -1862959120, -806963522, 51553076, 1704082561, 164725492, pgps2_rtk);
	
}


void read_MAG_CAL_REPORT(pmag_cal_report_MAG_CAL_REPORT *const pmag_cal_report) {
	int8_t           compass_id = pmag_cal_report_compass_id_GET(pmag_cal_report);
	int8_t           cal_mask   = pmag_cal_report_cal_mask_GET(pmag_cal_report);
	int8_t           autosaved  = pmag_cal_report_autosaved_GET(pmag_cal_report);
	float            fitness    = pmag_cal_report_fitness_GET(pmag_cal_report);
	float            ofs_x      = pmag_cal_report_ofs_x_GET(pmag_cal_report);
	float            ofs_y      = pmag_cal_report_ofs_y_GET(pmag_cal_report);
	float            ofs_z      = pmag_cal_report_ofs_z_GET(pmag_cal_report);
	float            diag_x     = pmag_cal_report_diag_x_GET(pmag_cal_report);
	float            diag_y     = pmag_cal_report_diag_y_GET(pmag_cal_report);
	float            diag_z     = pmag_cal_report_diag_z_GET(pmag_cal_report);
	float            offdiag_x  = pmag_cal_report_offdiag_x_GET(pmag_cal_report);
	float            offdiag_y  = pmag_cal_report_offdiag_y_GET(pmag_cal_report);
	float            offdiag_z  = pmag_cal_report_offdiag_z_GET(pmag_cal_report);
	e_MAG_CAL_STATUS item_cal_status;
	if (pmag_cal_report_cal_status_GET(pmag_cal_report, &item_cal_status)) {
		e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED = item_cal_status;
	}
	
}


void read_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST *const plog_request_list) {
	int16_t start            = plog_request_list_start_GET(plog_request_list);
	int16_t end              = plog_request_list_end_GET(plog_request_list);
	int8_t  target_system    = plog_request_list_target_system_GET(plog_request_list);
	int8_t  target_component = plog_request_list_target_component_GET(plog_request_list);
	
}


void write_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST *const plog_request_list) {
	plog_request_list_start_SET(26904, -25042, -9649, 12507, -6190, 15480, -13560, -18240, -185, -95, plog_request_list);
	plog_request_list_end_SET(-29228, -8366, 9774, 27875, 13023, -6944, 1056, -17040, -26714, -24585, plog_request_list);
	plog_request_list_target_system_SET(-125, 88, -122, -1, 31, -67, -9, -29, 57, 59, plog_request_list);
	plog_request_list_target_component_SET(38, -83, -61, -19, 109, 6, 87, -67, -64, -112, plog_request_list);
	
}


void read_SCALED_PRESSURE(pscaled_pressure_SCALED_PRESSURE *const pscaled_pressure) {
	int32_t time_boot_ms = pscaled_pressure_time_boot_ms_GET(pscaled_pressure);
	float   press_abs    = pscaled_pressure_press_abs_GET(pscaled_pressure);
	float   press_diff   = pscaled_pressure_press_diff_GET(pscaled_pressure);
	int16_t temperature  = pscaled_pressure_temperature_GET(pscaled_pressure);
	
}


void write_SCALED_PRESSURE(pscaled_pressure_SCALED_PRESSURE *const pscaled_pressure) {
	pscaled_pressure_time_boot_ms_SET(1512659490, 460757348, 1120070738, 1408507632, 1662786817, 961608330, 344759435, -681648839, 1287146752, 1145322869, pscaled_pressure);
	pscaled_pressure_press_abs_SET(-1.689164E38F, 1.9282619E38F, -2.2557753E38F, -4.027923E37F, 1.087858E37F, -1.5616054E38F, -1.0263625E38F, 1.0464523E38F, -2.7910421E38F, 8.667564E37F, pscaled_pressure);
	pscaled_pressure_press_diff_SET(7.8978735E37F, 3.154679E38F, 1.8829254E38F, -9.006194E37F, -3.0838309E38F, 2.6399272E38F, 1.8220374E38F, 1.0718151E38F, 1.6421471E38F, -2.5681206E38F, pscaled_pressure);
	pscaled_pressure_temperature_SET(8219, 26038, 1900, 7480, -13598, 25413, 9439, -8125, 1508, -1297, pscaled_pressure);
	
}


void read_V2_EXTENSION(pv2_extension_V2_EXTENSION *const pv2_extension) {
	int16_t               message_type     = pv2_extension_message_type_GET(pv2_extension);
	int8_t                target_network   = pv2_extension_target_network_GET(pv2_extension);
	int8_t                target_system    = pv2_extension_target_system_GET(pv2_extension);
	int8_t                target_component = pv2_extension_target_component_GET(pv2_extension);
	Vv2_extension_payload item_payload     = pv2_extension_payload_GET(pv2_extension);
	for (size_t           index            = 0; index < item_payload.len; index++)
		some_int8_t = vv2_extension_payload_GET(&item_payload, index);
	
}


void write_V2_EXTENSION(pv2_extension_V2_EXTENSION *const pv2_extension) {
	pv2_extension_message_type_SET(-3156, 24815, -21948, 15307, -6232, 28243, 21889, -11296, 29600, 16077, pv2_extension);
	pv2_extension_target_network_SET(-37, -51, -92, 58, 116, 32, -111, 88, 83, 102, pv2_extension);
	pv2_extension_target_system_SET(-87, -75, 25, 21, 100, 32, -98, -38, -88, -5, pv2_extension);
	pv2_extension_target_component_SET(27, -86, -71, 77, 107, -40, 122, 108, -78, 97, pv2_extension);
	pv2_extension_payload_SET(&21, 106, 52, 95, -35, 103, 117, -8, -74, 127, pv2_extension);
	
}


void read_HEARTBEAT(pheartbeat_HEARTBEAT *const pheartbeat) {
	int32_t    custom_mode     = pheartbeat_custom_mode_GET(pheartbeat);
	int8_t     mavlink_version = pheartbeat_mavlink_version_GET(pheartbeat);
	e_MAV_TYPE item_typE;
	if (pheartbeat_typE_GET(pheartbeat, &item_typE)) {
		e_MAV_TYPE_GENERIC = item_typE;
	}
	e_MAV_AUTOPILOT item_autopilot;
	if (pheartbeat_autopilot_GET(pheartbeat, &item_autopilot)) {
		e_MAV_AUTOPILOT_GENERIC = item_autopilot;
	}
	e_MAV_MODE_FLAG item_base_mode;
	if (pheartbeat_base_mode_GET(pheartbeat, &item_base_mode)) {
		e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = item_base_mode;
	}
	e_MAV_STATE item_system_status;
	if (pheartbeat_system_status_GET(pheartbeat, &item_system_status)) {
		e_MAV_STATE_UNINIT = item_system_status;
	}
	
}


void write_HEARTBEAT(pheartbeat_HEARTBEAT *const pheartbeat) {
	pheartbeat_custom_mode_SET(514818541, 1419468019, 1502581840, 1388194302, 656123531, -273051556, -8937847, -715267145, -2118204484, -1286807247, pheartbeat);
	pheartbeat_mavlink_version_SET(62, 96, 102, -7, 40, -124, 81, 108, -127, -103, pheartbeat);
	pheartbeat_typE_SET(e_MAV_TYPE_ROCKET, e_MAV_TYPE_VTOL_QUADROTOR, e_MAV_TYPE_OCTOROTOR, e_MAV_TYPE_VTOL_RESERVED2, e_MAV_TYPE_VTOL_RESERVED5, e_MAV_TYPE_VTOL_RESERVED3, e_MAV_TYPE_ADSB, e_MAV_TYPE_VTOL_DUOROTOR, e_MAV_TYPE_ROCKET, e_MAV_TYPE_GROUND_ROVER, pheartbeat);
	pheartbeat_autopilot_SET(e_MAV_AUTOPILOT_AEROB, e_MAV_AUTOPILOT_OPENPILOT, e_MAV_AUTOPILOT_OPENPILOT, e_MAV_AUTOPILOT_AEROB, e_MAV_AUTOPILOT_RESERVED, e_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, e_MAV_AUTOPILOT_ASLUAV, e_MAV_AUTOPILOT_FP, e_MAV_AUTOPILOT_INVALID, e_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, pheartbeat);
	pheartbeat_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, pheartbeat);
	pheartbeat_system_status_SET(e_MAV_STATE_CRITICAL, e_MAV_STATE_CRITICAL, e_MAV_STATE_POWEROFF, e_MAV_STATE_UNINIT, e_MAV_STATE_EMERGENCY, e_MAV_STATE_CRITICAL, e_MAV_STATE_CALIBRATING, e_MAV_STATE_ACTIVE, e_MAV_STATE_ACTIVE, e_MAV_STATE_ACTIVE, pheartbeat);
	
}


void read_PARAM_MAP_RC(pparam_map_rc_PARAM_MAP_RC *const pparam_map_rc) {
	int8_t                 target_system              = pparam_map_rc_target_system_GET(pparam_map_rc);
	int8_t                 target_component           = pparam_map_rc_target_component_GET(pparam_map_rc);
	int16_t                param_index                = pparam_map_rc_param_index_GET(pparam_map_rc);
	int8_t                 parameter_rc_channel_index = pparam_map_rc_parameter_rc_channel_index_GET(pparam_map_rc);
	float                  param_value0               = pparam_map_rc_param_value0_GET(pparam_map_rc);
	float                  scale                      = pparam_map_rc_scale_GET(pparam_map_rc);
	float                  param_value_min            = pparam_map_rc_param_value_min_GET(pparam_map_rc);
	float                  param_value_max            = pparam_map_rc_param_value_max_GET(pparam_map_rc);
	Vparam_map_rc_param_id item_param_id;
	if (pparam_map_rc_param_id_GET(pparam_map_rc, &item_param_id)) {
		memcpy(some_string, item_param_id.bytes, item_param_id.len);
	}
	
}


void write_PARAM_MAP_RC(pparam_map_rc_PARAM_MAP_RC *const pparam_map_rc) {
	pparam_map_rc_target_system_SET(29, -88, -43, -107, -101, -119, 46, 12, -26, -102, pparam_map_rc);
	pparam_map_rc_target_component_SET(36, -85, -117, -72, 56, -1, 12, -126, 74, -101, pparam_map_rc);
	pparam_map_rc_param_index_SET(6179, -21328, -21393, -1057, 16484, -19595, 19055, -20766, -21969, 26655, pparam_map_rc);
	pparam_map_rc_parameter_rc_channel_index_SET(-12, 90, 47, -107, 73, 122, -41, -109, -64, -5, pparam_map_rc);
	pparam_map_rc_param_value0_SET(-9.115511E37F, -4.1008079E37F, -2.166767E38F, 2.8256514E38F, 3.2798476E38F, -3.1812625E38F, -1.5320552E38F, 1.923377E37F, -2.6095665E38F, -2.3152616E38F, pparam_map_rc);
	pparam_map_rc_scale_SET(3.2114967E38F, 2.6494936E38F, -3.1731997E37F, -7.639134E37F, -1.7548574E38F, -2.551548E38F, -2.0835695E38F, -3.055599E38F, 2.0026137E38F, -2.5678762E38F, pparam_map_rc);
	pparam_map_rc_param_value_min_SET(2.4984333E38F, 3.0269612E38F, -2.0035481E38F, -5.1573026E37F, 2.0546117E38F, -1.0354169E38F, 3.323525E38F, -1.23106906E36F, -2.8463606E37F, -1.6528871E38F, pparam_map_rc);
	pparam_map_rc_param_value_max_SET(2.3886586E38F, 2.1209328E38F, -1.8049807E38F, -2.0834831E38F, 3.2751449E38F, -2.2687936E38F, 7.1030236E37F, -1.2595926E38F, -1.802691E37F, 1.4460373E38F, pparam_map_rc);
	pparam_map_rc_param_id_SET(some_string, strlen(some_string), pparam_map_rc);
	
}


void read_POWER_STATUS(ppower_status_POWER_STATUS *const ppower_status) {
	int16_t            Vcc    = ppower_status_Vcc_GET(ppower_status);
	int16_t            Vservo = ppower_status_Vservo_GET(ppower_status);
	e_MAV_POWER_STATUS item_flags;
	if (ppower_status_flags_GET(ppower_status, &item_flags)) {
		e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID = item_flags;
	}
	
}


void write_POWER_STATUS(ppower_status_POWER_STATUS *const ppower_status) {
	ppower_status_Vcc_SET(31240, 32591, 17128, 6939, 30175, -28118, 6389, 11956, 23547, -22257, ppower_status);
	ppower_status_Vservo_SET(31022, -12853, 7943, 22816, 17495, 15216, -217, 23082, -22695, -23116, ppower_status);
	ppower_status_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID, e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT, e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED, e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT, e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED, e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID, ppower_status);
	
}


void read_REMOTE_LOG_DATA_BLOCK(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *const premote_log_data_block) {
	int8_t                               target_system    = premote_log_data_block_target_system_GET(premote_log_data_block);
	int8_t                               target_component = premote_log_data_block_target_component_GET(premote_log_data_block);
	Vremote_log_data_block_daTa          item_daTa        = premote_log_data_block_daTa_GET(premote_log_data_block);
	for (size_t                          index            = 0; index < item_daTa.len; index++)
		some_int8_t = vremote_log_data_block_daTa_GET(&item_daTa, index);
	e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS item_seqno;
	if (premote_log_data_block_seqno_GET(premote_log_data_block, &item_seqno)) {
		e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP = item_seqno;
	}
	
}


void read_LOGGING_DATA_ACKED(plogging_data_acked_LOGGING_DATA_ACKED *const plogging_data_acked) {
	int16_t                  sequence             = plogging_data_acked_sequence_GET(plogging_data_acked);
	int8_t                   target_system        = plogging_data_acked_target_system_GET(plogging_data_acked);
	int8_t                   target_component     = plogging_data_acked_target_component_GET(plogging_data_acked);
	int8_t                   length               = plogging_data_acked_length_GET(plogging_data_acked);
	int8_t                   first_message_offset = plogging_data_acked_first_message_offset_GET(plogging_data_acked);
	Vlogging_data_acked_daTa item_daTa            = plogging_data_acked_daTa_GET(plogging_data_acked);
	for (size_t              index                = 0; index < item_daTa.len; index++)
		some_int8_t = vlogging_data_acked_daTa_GET(&item_daTa, index);
	
}


void read_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK *const pterrain_check) {
	int32_t lat = pterrain_check_lat_GET(pterrain_check);
	int32_t lon = pterrain_check_lon_GET(pterrain_check);
	
}


void write_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK *const pterrain_check) {
	pterrain_check_lat_SET(126493390, -1954947089, -374396803, -1582536930, -259599031, 376767129, -1814103969, -549565124, -59836965, -731025765, pterrain_check);
	pterrain_check_lon_SET(-581531278, 985288990, 559413443, -697405145, 1646643766, -1491957186, -874330106, 14253298, -1333211631, -1352635853, pterrain_check);
	
}


void read_MOUNT_CONFIGURE(pmount_configure_MOUNT_CONFIGURE *const pmount_configure) {
	int8_t           target_system    = pmount_configure_target_system_GET(pmount_configure);
	int8_t           target_component = pmount_configure_target_component_GET(pmount_configure);
	int8_t           stab_roll        = pmount_configure_stab_roll_GET(pmount_configure);
	int8_t           stab_pitch       = pmount_configure_stab_pitch_GET(pmount_configure);
	int8_t           stab_yaw         = pmount_configure_stab_yaw_GET(pmount_configure);
	e_MAV_MOUNT_MODE item_mount_mode;
	if (pmount_configure_mount_mode_GET(pmount_configure, &item_mount_mode)) {
		e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RETRACT = item_mount_mode;
	}
	
}


void read_MISSION_REQUEST_INT(pmission_request_int_MISSION_REQUEST_INT *const pmission_request_int) {
	int16_t            seq              = pmission_request_int_seq_GET(pmission_request_int);
	int8_t             target_system    = pmission_request_int_target_system_GET(pmission_request_int);
	int8_t             target_component = pmission_request_int_target_component_GET(pmission_request_int);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_request_int_mission_type_GET(pmission_request_int, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_REQUEST_INT(pmission_request_int_MISSION_REQUEST_INT *const pmission_request_int) {
	pmission_request_int_seq_SET(-31032, -22756, 32328, 8342, 3946, 12520, 8884, 3969, -5255, 25702, pmission_request_int);
	pmission_request_int_target_system_SET(33, -48, 49, -89, -119, -114, 95, -66, -58, -77, pmission_request_int);
	pmission_request_int_target_component_SET(59, -85, -127, -114, 115, 38, -109, -15, -12, 24, pmission_request_int);
	pmission_request_int_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, pmission_request_int);
	
}


void read_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *const plocal_position_ned_system_global_offset) {
	int32_t time_boot_ms = plocal_position_ned_system_global_offset_time_boot_ms_GET(plocal_position_ned_system_global_offset);
	float   x            = plocal_position_ned_system_global_offset_x_GET(plocal_position_ned_system_global_offset);
	float   y            = plocal_position_ned_system_global_offset_y_GET(plocal_position_ned_system_global_offset);
	float   z            = plocal_position_ned_system_global_offset_z_GET(plocal_position_ned_system_global_offset);
	float   roll         = plocal_position_ned_system_global_offset_roll_GET(plocal_position_ned_system_global_offset);
	float   pitch        = plocal_position_ned_system_global_offset_pitch_GET(plocal_position_ned_system_global_offset);
	float   yaw          = plocal_position_ned_system_global_offset_yaw_GET(plocal_position_ned_system_global_offset);
	
}


void write_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *const plocal_position_ned_system_global_offset) {
	plocal_position_ned_system_global_offset_time_boot_ms_SET(-1572214893, -56921091, 1643772712, -364094248, -481374493, -1749622656, -694775301, 1168938216, 856480044, -1324115587, plocal_position_ned_system_global_offset);
	plocal_position_ned_system_global_offset_x_SET(-2.710754E38F, 3.499423E37F, -3.1910378E38F, -2.1785054E38F, -1.3477423E38F, 2.6352846E38F, -2.7073198E38F, -8.1922716E37F, -5.930368E37F, 9.476983E36F, plocal_position_ned_system_global_offset);
	plocal_position_ned_system_global_offset_y_SET(2.5479008E38F, 2.2554597E38F, 8.676431E37F, 1.473496E37F, -1.7654479E38F, 2.728461E38F, -2.442503E37F, 1.6522134E38F, -5.9481675E37F, -1.816555E38F, plocal_position_ned_system_global_offset);
	plocal_position_ned_system_global_offset_z_SET(-2.1235622E38F, -3.9159682E37F, -2.712694E38F, 2.3414861E38F, 2.5911038E38F, -1.4046246E38F, 2.9335747E38F, 1.4305134E37F, 2.5449613E38F, -3.3795395E38F, plocal_position_ned_system_global_offset);
	plocal_position_ned_system_global_offset_roll_SET(2.4774391E38F, 1.3675342E38F, 3.3869768E38F, -2.5336754E38F, 2.1721274E38F, -1.0713403E38F, 3.2565613E38F, 3.3882504E38F, -2.2097308E37F, -1.8799256E38F, plocal_position_ned_system_global_offset);
	plocal_position_ned_system_global_offset_pitch_SET(3.2046854E38F, 3.0543944E38F, 1.8948421E38F, -2.9485076E38F, -2.6654096E37F, 1.4499589E37F, 2.8508428E37F, -1.8641252E38F, 2.1808347E38F, 1.624937E38F, plocal_position_ned_system_global_offset);
	plocal_position_ned_system_global_offset_yaw_SET(8.0586506E37F, 2.3343013E37F, 7.377193E37F, 5.222802E36F, -3.8111477E37F, 3.3239324E38F, -7.970109E37F, 2.9259264E38F, 1.4195226E38F, -2.3862433E38F, plocal_position_ned_system_global_offset);
	
}


void read_COMMAND_ACK(pcommand_ack_COMMAND_ACK *const pcommand_ack) {
	e_MAV_CMD item_command;
	if (pcommand_ack_command_GET(pcommand_ack, &item_command)) {
		e_MAV_CMD_MAV_CMD_NAV_WAYPOINT = item_command;
	}
	e_MAV_RESULT item_result;
	if (pcommand_ack_result_GET(pcommand_ack, &item_result)) {
		e_MAV_RESULT_MAV_RESULT_ACCEPTED = item_result;
	}
	int8_t item_progress;
	if (pcommand_ack_progress_GET(pcommand_ack, &item_progress)) {
		some_int8_t = item_progress;
	}
	int32_t item_result_param2;
	if (pcommand_ack_result_param2_GET(pcommand_ack, &item_result_param2)) {
		some_int32_t = item_result_param2;
	}
	int8_t item_target_system;
	if (pcommand_ack_target_system_GET(pcommand_ack, &item_target_system)) {
		some_int8_t = item_target_system;
	}
	int8_t item_target_component;
	if (pcommand_ack_target_component_GET(pcommand_ack, &item_target_component)) {
		some_int8_t = item_target_component;
	}
	
}


void write_COMMAND_ACK(pcommand_ack_COMMAND_ACK *const pcommand_ack) {
	pcommand_ack_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM, e_MAV_CMD_MAV_CMD_USER_1, e_MAV_CMD_MAV_CMD_DO_SET_HOME, e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM, e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH, e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING, e_MAV_CMD_MAV_CMD_USER_2, e_MAV_CMD_MAV_CMD_NAV_ROI, e_MAV_CMD_MAV_CMD_DO_SET_MODE, e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL, pcommand_ack);
	pcommand_ack_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, e_MAV_RESULT_MAV_RESULT_DENIED, e_MAV_RESULT_MAV_RESULT_DENIED, e_MAV_RESULT_MAV_RESULT_DENIED, e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, e_MAV_RESULT_MAV_RESULT_FAILED, e_MAV_RESULT_MAV_RESULT_ACCEPTED, e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, pcommand_ack);
	pcommand_ack_progress_SET(-78, -85, -16, 48, -80, 24, -104, 125, 23, 1, pcommand_ack);
	pcommand_ack_result_param2_SET(-667356583, -1893193091, -496418164, -349624018, -10001202, -1251657731, 343552405, -1105775651, 318486339, -1093678064, pcommand_ack);
	pcommand_ack_target_system_SET(113, 28, -81, 105, 122, 72, -10, -8, 101, 124, pcommand_ack);
	pcommand_ack_target_component_SET(113, 27, 55, 72, 7, 10, -74, -90, 22, 106, pcommand_ack);
	
}


void read_DATA_STREAM(pdata_stream_DATA_STREAM *const pdata_stream) {
	int16_t message_rate = pdata_stream_message_rate_GET(pdata_stream);
	int8_t  stream_id    = pdata_stream_stream_id_GET(pdata_stream);
	int8_t  on_off       = pdata_stream_on_off_GET(pdata_stream);
	
}


void write_DATA_STREAM(pdata_stream_DATA_STREAM *const pdata_stream) {
	pdata_stream_message_rate_SET(-10301, -12779, -14277, -29104, -10574, -10445, -2383, 2317, 29228, 6882, pdata_stream);
	pdata_stream_stream_id_SET(-108, 96, -85, 103, 78, 114, 54, -77, -99, -57, pdata_stream);
	pdata_stream_on_off_SET(-19, -78, -10, -115, -41, -43, -45, -112, 68, -47, pdata_stream);
	
}


void read_MISSION_REQUEST(pmission_request_MISSION_REQUEST *const pmission_request) {
	int16_t            seq              = pmission_request_seq_GET(pmission_request);
	int8_t             target_system    = pmission_request_target_system_GET(pmission_request);
	int8_t             target_component = pmission_request_target_component_GET(pmission_request);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_request_mission_type_GET(pmission_request, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_REQUEST(pmission_request_MISSION_REQUEST *const pmission_request) {
	pmission_request_seq_SET(-16705, 25678, -10110, -27934, 20748, -1288, 30291, 19936, 3948, 24147, pmission_request);
	pmission_request_target_system_SET(-23, 61, -49, -86, 28, 121, -31, -90, -11, -7, pmission_request);
	pmission_request_target_component_SET(-31, -13, -77, 0, -82, 127, -30, 86, 54, 96, pmission_request);
	pmission_request_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, pmission_request);
	
}


void read_TERRAIN_REPORT(pterrain_report_TERRAIN_REPORT *const pterrain_report) {
	int16_t spacing        = pterrain_report_spacing_GET(pterrain_report);
	int16_t pending        = pterrain_report_pending_GET(pterrain_report);
	int16_t loaded         = pterrain_report_loaded_GET(pterrain_report);
	int32_t lat            = pterrain_report_lat_GET(pterrain_report);
	int32_t lon            = pterrain_report_lon_GET(pterrain_report);
	float   terrain_height = pterrain_report_terrain_height_GET(pterrain_report);
	float   current_height = pterrain_report_current_height_GET(pterrain_report);
	
}


void write_TERRAIN_REPORT(pterrain_report_TERRAIN_REPORT *const pterrain_report) {
	pterrain_report_spacing_SET(32347, 19286, -27390, -27531, 11696, 18178, 20098, -30857, 22755, -15644, pterrain_report);
	pterrain_report_pending_SET(-848, -10293, -20665, 15015, 20336, -21242, 23347, -10267, 8218, 278, pterrain_report);
	pterrain_report_loaded_SET(31034, -19323, 9395, -4112, 11649, -18010, -9831, 18837, -11927, -19646, pterrain_report);
	pterrain_report_lat_SET(-183845280, -5386830, 1333086277, 1479625413, 747574733, 818317496, -2073481534, 147027722, 699975218, -59314909, pterrain_report);
	pterrain_report_lon_SET(-831914737, -729382602, 68137652, 1668651913, 1712561078, -1697093688, -1406960747, -408328102, 712951208, 490005338, pterrain_report);
	pterrain_report_terrain_height_SET(2.9960885E38F, 1.210934E38F, 1.1197376E38F, -3.314849E38F, -9.515224E37F, -5.1819513E37F, -1.1771812E38F, -1.263677E38F, 1.5486103E38F, -2.534912E38F, pterrain_report);
	pterrain_report_current_height_SET(2.3330888E38F, 1.1521343E37F, 1.241951E38F, -1.7841937E38F, 2.3307156E38F, -1.5293527E38F, 3.5707477E36F, 2.4246158E38F, 2.0042795E38F, -1.2243076E38F, pterrain_report);
	
}


void read_SET_HOME_POSITION(pset_home_position_SET_HOME_POSITION *const pset_home_position) {
	int8_t               target_system = pset_home_position_target_system_GET(pset_home_position);
	int32_t              latitude      = pset_home_position_latitude_GET(pset_home_position);
	int32_t              longitude     = pset_home_position_longitude_GET(pset_home_position);
	int32_t              altitude      = pset_home_position_altitude_GET(pset_home_position);
	float                x             = pset_home_position_x_GET(pset_home_position);
	float                y             = pset_home_position_y_GET(pset_home_position);
	float                z             = pset_home_position_z_GET(pset_home_position);
	Vset_home_position_q item_q        = pset_home_position_q_GET(pset_home_position);
	for (size_t          index         = 0; index < item_q.len; index++)
		some_float                  = vset_home_position_q_GET(&item_q, index);
	float                approach_x = pset_home_position_approach_x_GET(pset_home_position);
	float                approach_y = pset_home_position_approach_y_GET(pset_home_position);
	float                approach_z = pset_home_position_approach_z_GET(pset_home_position);
	int64_t              item_time_usec;
	if (pset_home_position_time_usec_GET(pset_home_position, &item_time_usec)) {
		some_int64_t = item_time_usec;
	}
	
}


void write_SET_HOME_POSITION(pset_home_position_SET_HOME_POSITION *const pset_home_position) {
	pset_home_position_target_system_SET(59, -99, 108, -48, 29, -109, 35, -113, -86, 80, pset_home_position);
	pset_home_position_latitude_SET(-176700178, 566639249, 833935248, -1630111962, 757513545, 1267481120, 811636081, -505959251, 771673570, 503565518, pset_home_position);
	pset_home_position_longitude_SET(818851160, -1917807117, 27842762, 896412790, 1375812410, 735325085, -1765168946, 593843658, -975102952, 325509769, pset_home_position);
	pset_home_position_altitude_SET(-1853570021, -1265484163, 1547022348, -1235593151, -238582124, -391209489, -1671810208, -893731399, -383526640, 1032188941, pset_home_position);
	pset_home_position_x_SET(9.058776E37F, 5.854687E37F, -2.6838232E38F, 2.593271E38F, 5.7627893E37F, 1.8864848E38F, 2.629593E38F, 2.0141662E38F, 2.0144396E38F, -2.0067874E38F, pset_home_position);
	pset_home_position_y_SET(-2.7563172E38F, -8.66036E37F, 7.5074797E37F, -6.4940057E37F, 3.3721743E38F, -7.45255E36F, -2.4261717E38F, 2.7576672E38F, -1.0580484E38F, 2.2154592E38F, pset_home_position);
	pset_home_position_z_SET(-9.053572E37F, -1.0366707E38F, 2.7956E38F, 6.7378717E37F, 1.0777242E37F, 7.0134184E37F, -1.8728396E38F, -1.2949609E38F, 3.2493683E37F, 6.4042657E37F, pset_home_position);
	pset_home_position_q_SET(&-2.6561572E38F, 6.1711706E37F, -1.1792015E38F, 3.3408276E38F, 1.2278861E38F, 4.7358174E37F, 3.136132E38F, 7.25685E37F, 5.425772E37F, 2.6438707E37F, pset_home_position);
	pset_home_position_approach_x_SET(-2.1538386E38F, -4.540356E37F, -3.3474895E37F, 3.2089498E38F, 8.720761E37F, 3.0317497E38F, -2.1501192E38F, 3.3837314E38F, -1.1294081E38F, 1.3636388E38F, pset_home_position);
	pset_home_position_approach_y_SET(1.4758085E38F, 3.0426488E38F, 1.9286405E38F, -3.3986516E38F, -2.4973171E38F, -3.1254727E38F, 2.752141E38F, 2.3347589E38F, 1.3664918E38F, 9.215806E37F, pset_home_position);
	pset_home_position_approach_z_SET(1.4264456E38F, 6.5504937E37F, 2.60274E37F, -2.3569507E38F, -1.4073323E38F, -3.1589275E38F, -2.4514381E38F, -2.6414407E38F, -3.371061E38F, 3.4265103E36F, pset_home_position);
	pset_home_position_time_usec_SET(424978792071342418L, -7882224065341235419L, -5254854555432801382L, -5334826218897000671L, -8921969193934192842L, 4940425207457034536L, -3478490007674913881L, -4625063876032294901L, -2529970131259789163L, -1747745838039391973L, pset_home_position);
	
}


void read_HIL_RC_INPUTS_RAW(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *const phil_rc_inputs_raw) {
	int16_t chan1_raw  = phil_rc_inputs_raw_chan1_raw_GET(phil_rc_inputs_raw);
	int16_t chan2_raw  = phil_rc_inputs_raw_chan2_raw_GET(phil_rc_inputs_raw);
	int16_t chan3_raw  = phil_rc_inputs_raw_chan3_raw_GET(phil_rc_inputs_raw);
	int16_t chan4_raw  = phil_rc_inputs_raw_chan4_raw_GET(phil_rc_inputs_raw);
	int16_t chan5_raw  = phil_rc_inputs_raw_chan5_raw_GET(phil_rc_inputs_raw);
	int16_t chan6_raw  = phil_rc_inputs_raw_chan6_raw_GET(phil_rc_inputs_raw);
	int16_t chan7_raw  = phil_rc_inputs_raw_chan7_raw_GET(phil_rc_inputs_raw);
	int16_t chan8_raw  = phil_rc_inputs_raw_chan8_raw_GET(phil_rc_inputs_raw);
	int16_t chan9_raw  = phil_rc_inputs_raw_chan9_raw_GET(phil_rc_inputs_raw);
	int16_t chan10_raw = phil_rc_inputs_raw_chan10_raw_GET(phil_rc_inputs_raw);
	int16_t chan11_raw = phil_rc_inputs_raw_chan11_raw_GET(phil_rc_inputs_raw);
	int16_t chan12_raw = phil_rc_inputs_raw_chan12_raw_GET(phil_rc_inputs_raw);
	int64_t time_usec  = phil_rc_inputs_raw_time_usec_GET(phil_rc_inputs_raw);
	int8_t  rssi       = phil_rc_inputs_raw_rssi_GET(phil_rc_inputs_raw);
	
}


void write_HIL_RC_INPUTS_RAW(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *const phil_rc_inputs_raw) {
	phil_rc_inputs_raw_chan1_raw_SET(-24147, -13030, 28662, 16452, -7945, 12261, 5946, -21476, 8993, 1623, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan2_raw_SET(-29934, 7933, 12570, 22860, 26352, 19278, 2699, -23889, 2376, 28137, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan3_raw_SET(22514, 16541, 16172, -405, 12718, 22179, -2842, -17164, -9406, 7202, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan4_raw_SET(23123, 24410, 30593, -192, 23003, 32082, 20365, -15839, -28669, 18344, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan5_raw_SET(-24061, 13165, -18097, 26697, -3221, 1548, 23641, -29000, 28128, 15210, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan6_raw_SET(-11398, -13248, -22412, 10761, 4569, 12865, 18806, -6077, 10272, -9504, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan7_raw_SET(-2077, -16855, -7455, -31244, -8176, -28888, -26432, 21421, -15610, -16029, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan8_raw_SET(25217, -8127, -31400, 5235, -22364, -20718, 2149, -2529, -24553, -10623, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan9_raw_SET(12962, 20400, 22839, 24687, -8065, -17510, 15848, -8799, 5735, -20922, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan10_raw_SET(-19302, -24550, 11171, 14562, 11316, 16445, -1140, 30295, 21840, -439, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan11_raw_SET(-30794, -13971, -14431, 24579, 26419, -17144, 6497, -16212, -30184, -24898, phil_rc_inputs_raw);
	phil_rc_inputs_raw_chan12_raw_SET(-7391, 2187, -5262, -23221, 32108, 4132, 25672, 11040, -23921, 18530, phil_rc_inputs_raw);
	phil_rc_inputs_raw_time_usec_SET(-5803629192392654655L, 7181636687788377697L, 5074634850863140539L, -4279160911312392923L, -3846015830888436831L, 3386927559754484620L, -7650627963216605050L, -9033402608930438912L, 3995051699211228247L, -6870721209196931790L, phil_rc_inputs_raw);
	phil_rc_inputs_raw_rssi_SET(-62, -11, 107, 100, 125, -117, -85, -9, 115, -105, phil_rc_inputs_raw);
	
}


void read_SCALED_IMU3(pscaled_imu3_SCALED_IMU3 *const pscaled_imu3) {
	int32_t time_boot_ms = pscaled_imu3_time_boot_ms_GET(pscaled_imu3);
	int16_t xacc         = pscaled_imu3_xacc_GET(pscaled_imu3);
	int16_t yacc         = pscaled_imu3_yacc_GET(pscaled_imu3);
	int16_t zacc         = pscaled_imu3_zacc_GET(pscaled_imu3);
	int16_t xgyro        = pscaled_imu3_xgyro_GET(pscaled_imu3);
	int16_t ygyro        = pscaled_imu3_ygyro_GET(pscaled_imu3);
	int16_t zgyro        = pscaled_imu3_zgyro_GET(pscaled_imu3);
	int16_t xmag         = pscaled_imu3_xmag_GET(pscaled_imu3);
	int16_t ymag         = pscaled_imu3_ymag_GET(pscaled_imu3);
	int16_t zmag         = pscaled_imu3_zmag_GET(pscaled_imu3);
	
}


void write_SCALED_IMU3(pscaled_imu3_SCALED_IMU3 *const pscaled_imu3) {
	pscaled_imu3_time_boot_ms_SET(1135168087, 602688399, -1642207510, 1279929160, 1481808773, 1345926848, -1173553273, -2027088129, -2004362989, 1653263777, pscaled_imu3);
	pscaled_imu3_xacc_SET(-24964, -17105, -5582, -23710, -16528, -9750, -7390, 12959, -24821, 4493, pscaled_imu3);
	pscaled_imu3_yacc_SET(-28217, -27482, 27381, -26581, 32671, 6091, 29999, 11180, 6665, 4706, pscaled_imu3);
	pscaled_imu3_zacc_SET(18193, 19865, -31946, 14644, 27461, 1537, 15604, 24577, 31583, 28239, pscaled_imu3);
	pscaled_imu3_xgyro_SET(29317, 15209, 31210, 3107, -3697, -19774, 26123, 1040, -28565, -30717, pscaled_imu3);
	pscaled_imu3_ygyro_SET(28371, -12105, -4369, -28431, -28185, 12520, 11329, -22804, -28532, 15359, pscaled_imu3);
	pscaled_imu3_zgyro_SET(-11270, 21268, -25955, -440, -2708, -21691, 15238, -26071, -18118, 14488, pscaled_imu3);
	pscaled_imu3_xmag_SET(-11975, -2360, 19169, -31146, -14322, 20760, 10663, 31898, -27034, 1041, pscaled_imu3);
	pscaled_imu3_ymag_SET(19194, 30019, 17243, 2790, 18072, 10978, 26370, 13032, 6358, -19389, pscaled_imu3);
	pscaled_imu3_zmag_SET(19231, -26027, -17652, 28286, 31880, 7514, -26658, -5086, 28633, -113, pscaled_imu3);
	
}


void read_SET_MODE(pset_mode_SET_MODE *const pset_mode) {
	int32_t    custom_mode   = pset_mode_custom_mode_GET(pset_mode);
	int8_t     target_system = pset_mode_target_system_GET(pset_mode);
	e_MAV_MODE item_base_mode;
	if (pset_mode_base_mode_GET(pset_mode, &item_base_mode)) {
		e_MAV_MODE_PREFLIGHT = item_base_mode;
	}
	
}


void write_SET_MODE(pset_mode_SET_MODE *const pset_mode) {
	pset_mode_custom_mode_SET(1146374990, -879927478, 292883975, 1106460010, -1554576211, 1815066980, -519376530, 1596672865, -561271092, -230095405, pset_mode);
	pset_mode_target_system_SET(-2, -92, -29, 113, 75, 85, -99, -22, -20, 72, pset_mode);
	pset_mode_base_mode_SET(e_MAV_MODE_MANUAL_ARMED, e_MAV_MODE_MANUAL_ARMED, e_MAV_MODE_GUIDED_ARMED, e_MAV_MODE_MAV_MODE_TEST_DISARMED, e_MAV_MODE_MAV_MODE_AUTO_ARMED, e_MAV_MODE_MANUAL_DISARMED, e_MAV_MODE_MANUAL_DISARMED, e_MAV_MODE_STABILIZE_DISARMED, e_MAV_MODE_GUIDED_DISARMED, e_MAV_MODE_GUIDED_DISARMED, pset_mode);
	
}


void read_MOUNT_CONTROL(pmount_control_MOUNT_CONTROL *const pmount_control) {
	int8_t  target_system    = pmount_control_target_system_GET(pmount_control);
	int8_t  target_component = pmount_control_target_component_GET(pmount_control);
	int32_t input_a          = pmount_control_input_a_GET(pmount_control);
	int32_t input_b          = pmount_control_input_b_GET(pmount_control);
	int32_t input_c          = pmount_control_input_c_GET(pmount_control);
	int8_t  save_position    = pmount_control_save_position_GET(pmount_control);
	
}


void read_POSITION_TARGET_GLOBAL_INT(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *const pposition_target_global_int) {
	int16_t     type_mask    = pposition_target_global_int_type_mask_GET(pposition_target_global_int);
	int32_t     time_boot_ms = pposition_target_global_int_time_boot_ms_GET(pposition_target_global_int);
	int32_t     lat_int      = pposition_target_global_int_lat_int_GET(pposition_target_global_int);
	int32_t     lon_int      = pposition_target_global_int_lon_int_GET(pposition_target_global_int);
	float       alt          = pposition_target_global_int_alt_GET(pposition_target_global_int);
	float       vx           = pposition_target_global_int_vx_GET(pposition_target_global_int);
	float       vy           = pposition_target_global_int_vy_GET(pposition_target_global_int);
	float       vz           = pposition_target_global_int_vz_GET(pposition_target_global_int);
	float       afx          = pposition_target_global_int_afx_GET(pposition_target_global_int);
	float       afy          = pposition_target_global_int_afy_GET(pposition_target_global_int);
	float       afz          = pposition_target_global_int_afz_GET(pposition_target_global_int);
	float       yaw          = pposition_target_global_int_yaw_GET(pposition_target_global_int);
	float       yaw_rate     = pposition_target_global_int_yaw_rate_GET(pposition_target_global_int);
	e_MAV_FRAME item_coordinate_frame;
	if (pposition_target_global_int_coordinate_frame_GET(pposition_target_global_int, &item_coordinate_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_coordinate_frame;
	}
	
}


void write_POSITION_TARGET_GLOBAL_INT(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *const pposition_target_global_int) {
	pposition_target_global_int_type_mask_SET(3832, -30200, 171, 13254, -32254, 14049, 20497, -30459, -20163, -25651, pposition_target_global_int);
	pposition_target_global_int_time_boot_ms_SET(-886734905, -1615827328, -1255596990, 1566212895, -551405865, 389368833, -1730476846, -1051091413, 1888063547, 1588426190, pposition_target_global_int);
	pposition_target_global_int_lat_int_SET(-1386387657, 1628794320, 2006442788, 562294706, 1712089628, -806219400, 298343256, -1946213031, 1448604653, 826639709, pposition_target_global_int);
	pposition_target_global_int_lon_int_SET(-472626695, 772106431, 1649205194, -712139023, 1127748684, -1964029356, -1770483848, 1629603787, 1582995686, 1192291246, pposition_target_global_int);
	pposition_target_global_int_alt_SET(-1.5760449E38F, 7.382797E37F, -2.645774E38F, 2.610174E37F, -2.958743E38F, 3.3300074E38F, 2.3336817E38F, -3.3721733E38F, -3.2843724E38F, -1.8635042E38F, pposition_target_global_int);
	pposition_target_global_int_vx_SET(-3.0363475E38F, 1.2154017E38F, 2.4073863E38F, 3.1277828E38F, 1.4733619E38F, 1.8179065E37F, -4.5611235E37F, -2.7016723E38F, 2.4740546E38F, 1.927759E38F, pposition_target_global_int);
	pposition_target_global_int_vy_SET(-3.2136456E38F, 8.1994404E37F, -1.3128805E38F, 1.2415357E37F, -2.1905923E38F, 3.172205E38F, -8.0948887E37F, 3.6196843E37F, -8.724276E37F, 8.530957E37F, pposition_target_global_int);
	pposition_target_global_int_vz_SET(-1.7138713E38F, -2.7021289E38F, -2.9668602E38F, 4.214654E37F, 2.9835343E38F, -5.627011E37F, 1.171162E38F, 2.1466219E38F, 1.4694712E38F, 2.7870412E38F, pposition_target_global_int);
	pposition_target_global_int_afx_SET(1.5529799E38F, -2.5686633E38F, 1.4398106E38F, 1.7388093E37F, -3.328799E38F, 2.0941923E38F, -5.689293E37F, 3.0310242E38F, -1.7925611E37F, -1.0822906E38F, pposition_target_global_int);
	pposition_target_global_int_afy_SET(-1.0757217E38F, -3.1037792E38F, 3.0820277E38F, 1.4626167E38F, 2.203708E38F, 3.958587E37F, -3.5132616E36F, -9.599494E37F, -2.0819088E38F, -1.8989132E38F, pposition_target_global_int);
	pposition_target_global_int_afz_SET(3.3202055E38F, 3.3572414E38F, 3.2472225E38F, 2.6547703E38F, 1.3560812E38F, -1.8620652E38F, -5.625898E37F, 1.7209207E38F, 1.4196376E38F, -1.2212215E38F, pposition_target_global_int);
	pposition_target_global_int_yaw_SET(1.4851997E38F, 4.927577E37F, 1.8194376E37F, -2.6796856E38F, 2.492242E38F, 6.27275E37F, 3.2273847E38F, -2.896421E38F, 8.617084E37F, -2.2925276E37F, pposition_target_global_int);
	pposition_target_global_int_yaw_rate_SET(3.3917775E38F, -2.0018282E38F, 1.6149796E38F, -1.9617796E38F, 4.279681E37F, -1.7445884E38F, 2.0777091E37F, 2.0817078E38F, 2.574637E38F, -6.0944214E37F, pposition_target_global_int);
	pposition_target_global_int_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, pposition_target_global_int);
	
}


void read_LED_CONTROL(pled_control_LED_CONTROL *const pled_control) {
	int8_t                    target_system     = pled_control_target_system_GET(pled_control);
	int8_t                    target_component  = pled_control_target_component_GET(pled_control);
	int8_t                    instance          = pled_control_instance_GET(pled_control);
	int8_t                    pattern           = pled_control_pattern_GET(pled_control);
	int8_t                    custom_len        = pled_control_custom_len_GET(pled_control);
	Vled_control_custom_bytes item_custom_bytes = pled_control_custom_bytes_GET(pled_control);
	for (size_t               index             = 0; index < item_custom_bytes.len; index++)
		some_int8_t = vled_control_custom_bytes_GET(&item_custom_bytes, index);
	
}


void read_SIM_STATE(psim_state_SIM_STATE *const psim_state) {
	float q1           = psim_state_q1_GET(psim_state);
	float q2           = psim_state_q2_GET(psim_state);
	float q3           = psim_state_q3_GET(psim_state);
	float q4           = psim_state_q4_GET(psim_state);
	float roll         = psim_state_roll_GET(psim_state);
	float pitch        = psim_state_pitch_GET(psim_state);
	float yaw          = psim_state_yaw_GET(psim_state);
	float xacc         = psim_state_xacc_GET(psim_state);
	float yacc         = psim_state_yacc_GET(psim_state);
	float zacc         = psim_state_zacc_GET(psim_state);
	float xgyro        = psim_state_xgyro_GET(psim_state);
	float ygyro        = psim_state_ygyro_GET(psim_state);
	float zgyro        = psim_state_zgyro_GET(psim_state);
	float lat          = psim_state_lat_GET(psim_state);
	float lon          = psim_state_lon_GET(psim_state);
	float alt          = psim_state_alt_GET(psim_state);
	float std_dev_horz = psim_state_std_dev_horz_GET(psim_state);
	float std_dev_vert = psim_state_std_dev_vert_GET(psim_state);
	float vn           = psim_state_vn_GET(psim_state);
	float ve           = psim_state_ve_GET(psim_state);
	float vd           = psim_state_vd_GET(psim_state);
	
}


void write_SIM_STATE(psim_state_SIM_STATE *const psim_state) {
	psim_state_q1_SET(1.6036483E38F, -9.112768E37F, 2.1339975E38F, -1.9686448E38F, 1.9690731E38F, 1.0879252E38F, -3.139668E38F, -3.0053223E38F, -2.5469472E38F, 2.3824434E38F, psim_state);
	psim_state_q2_SET(-1.2389107E38F, 1.7794212E38F, -1.1489926E38F, 1.8460516E38F, 3.2427688E36F, -3.3776743E38F, -2.6188256E38F, -1.8407425E38F, -2.023963E38F, 1.5279033E38F, psim_state);
	psim_state_q3_SET(1.0672629E37F, -1.6128985E38F, -2.487344E38F, 2.2195511E38F, 2.2249371E38F, -7.012344E37F, -2.7467482E38F, -1.9171842E38F, -1.8633275E38F, 2.313998E38F, psim_state);
	psim_state_q4_SET(-1.1080686E37F, 3.048912E38F, -2.1331016E38F, -9.363935E37F, -3.9267975E37F, 2.11181E38F, 9.598742E37F, -2.5144638E37F, -3.2841482E38F, -1.3566921E38F, psim_state);
	psim_state_roll_SET(1.863862E37F, -2.404441E38F, 3.2326731E38F, -2.0384195E38F, -1.3898842E38F, -1.6808934E38F, 5.340804E37F, 1.0861371E37F, -7.077678E36F, 1.285537E38F, psim_state);
	psim_state_pitch_SET(2.7639184E38F, 2.3369764E38F, -2.9897957E38F, -1.7320214E38F, -3.054588E38F, -5.196124E37F, 3.0143015E38F, -2.022943E38F, -2.514614E38F, 2.26374E38F, psim_state);
	psim_state_yaw_SET(-1.4492898E38F, 3.261418E37F, 2.4376023E38F, 2.2258513E38F, 2.0824082E38F, 3.850315E37F, -1.3632004E38F, 2.7779969E38F, 2.0761663E38F, -2.6234668E38F, psim_state);
	psim_state_xacc_SET(-2.5833778E38F, 7.0035683E37F, -1.1618174E38F, -2.5131682E38F, -2.871365E38F, -5.5755954E37F, -1.5658408E38F, -2.1818386E38F, -3.9689022E37F, 2.5123557E38F, psim_state);
	psim_state_yacc_SET(2.8204699E38F, 1.3047037E38F, -1.4611051E38F, -1.5253093E38F, 4.887109E36F, -1.1010683E38F, 3.1478687E38F, 2.4536696E38F, -2.9153159E38F, 2.5888756E38F, psim_state);
	psim_state_zacc_SET(2.9772005E38F, 1.7032553E38F, -3.1561405E38F, -1.0992781E38F, 1.450005E38F, 9.065917E37F, 1.3703774E38F, 3.000247E37F, 1.6582487E38F, 2.836022E37F, psim_state);
	psim_state_xgyro_SET(-1.4873933E37F, 1.1319064E38F, 2.8088444E38F, -1.0125699E38F, 3.3007593E38F, 1.2810595E38F, -3.3976677E38F, 2.4574161E38F, 2.0685585E38F, -2.245798E37F, psim_state);
	psim_state_ygyro_SET(-3.4750714E37F, 2.9408453E38F, 3.1696336E38F, 2.9792091E38F, -1.5839061E38F, 1.1093189E38F, 2.8531091E38F, -2.7693375E38F, -3.1694072E38F, -2.5335007E38F, psim_state);
	psim_state_zgyro_SET(-3.2763848E38F, 1.2137089E38F, 9.634081E37F, 2.0329933E37F, -2.9390487E38F, 4.827234E37F, -5.457699E37F, -3.1382045E38F, 1.9883428E38F, -1.1359532E37F, psim_state);
	psim_state_lat_SET(2.7048388E38F, 1.867434E38F, -1.390374E38F, 2.1666246E38F, -3.6933302E37F, -2.2076867E38F, -9.258865E37F, -4.9745175E37F, -9.808192E37F, 4.977852E37F, psim_state);
	psim_state_lon_SET(3.337507E38F, -1.4262754E38F, -1.6565645E38F, 1.630621E38F, 2.6291443E38F, 8.05851E36F, -3.355683E38F, 2.3008542E38F, 1.8970992E38F, -1.2242664E38F, psim_state);
	psim_state_alt_SET(-4.2425193E37F, -2.8569106E38F, 2.4719164E38F, -1.0984813E38F, -3.3411187E38F, 1.3389795E38F, -3.3473937E38F, 3.2522223E38F, 8.08137E37F, 2.0613043E38F, psim_state);
	psim_state_std_dev_horz_SET(2.4459004E38F, -9.051491E37F, 1.95178E38F, -2.3792658E38F, -2.3730677E38F, -1.980939E38F, -7.969341E37F, 1.2734743E38F, -2.9091604E38F, 4.4371346E37F, psim_state);
	psim_state_std_dev_vert_SET(-1.2398616E38F, -1.817479E38F, 1.2984666E38F, 1.8467947E38F, -1.630548E38F, 1.992922E38F, -3.089342E38F, -1.08013E38F, 2.940248E38F, -7.5349456E37F, psim_state);
	psim_state_vn_SET(1.6374034E37F, 2.997327E38F, -2.0203747E37F, 1.2969763E38F, 4.239591E37F, -1.5638517E38F, -1.099017E38F, 1.6313697E38F, -1.2337655E38F, 4.6784167E37F, psim_state);
	psim_state_ve_SET(3.0448069E38F, -4.4394214E37F, -3.0496872E38F, 3.2421243E38F, -3.4943638E37F, 6.371554E36F, 2.7251955E38F, 2.8984882E38F, -1.4780593E38F, -2.684446E38F, psim_state);
	psim_state_vd_SET(1.3082746E38F, -7.0117147E37F, 3.1213945E38F, 2.6658084E38F, 3.0075402E38F, -2.3319709E38F, 1.8624562E38F, -2.104911E38F, 2.8007328E38F, -8.713004E37F, psim_state);
	
}


void read_WIFI_CONFIG_AP(pwifi_config_ap_WIFI_CONFIG_AP *const pwifi_config_ap) {
	Vwifi_config_ap_ssid item_ssid;
	if (pwifi_config_ap_ssid_GET(pwifi_config_ap, &item_ssid)) {
		memcpy(some_string, item_ssid.bytes, item_ssid.len);
	}
	Vwifi_config_ap_password item_password;
	if (pwifi_config_ap_password_GET(pwifi_config_ap, &item_password)) {
		memcpy(some_string, item_password.bytes, item_password.len);
	}
	
}


void read_DATA96(pdata96_DATA96 *const pdata96) {
	int8_t       typE      = pdata96_typE_GET(pdata96);
	int8_t       len       = pdata96_len_GET(pdata96);
	Vdata96_daTa item_daTa = pdata96_daTa_GET(pdata96);
	for (size_t  index     = 0; index < item_daTa.len; index++)
		some_int8_t = vdata96_daTa_GET(&item_daTa, index);
	
}


void read_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION *const pflight_information) {
	int32_t time_boot_ms     = pflight_information_time_boot_ms_GET(pflight_information);
	int64_t arming_time_utc  = pflight_information_arming_time_utc_GET(pflight_information);
	int64_t takeoff_time_utc = pflight_information_takeoff_time_utc_GET(pflight_information);
	int64_t flight_uuid      = pflight_information_flight_uuid_GET(pflight_information);
	
}


void write_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION *const pflight_information) {
	pflight_information_time_boot_ms_SET(1994568552, -2083841989, -160954285, -2066336648, -284692317, -1473609182, 40212399, 631699634, 701060487, 398160591, pflight_information);
	pflight_information_arming_time_utc_SET(-7885952538243262837L, -2052882694185107667L, -5421506561258920955L, 7598291926216038174L, -2132095365893826695L, -604182751047779238L, -8128403306398175802L, -8038257236186309315L, -8963430943107306357L, 8353995398529330791L, pflight_information);
	pflight_information_takeoff_time_utc_SET(2423929395224332300L, -2466301014135949897L, 3736379381291663356L, -5493107898223391054L, 719778711780033343L, -1964286697679406748L, -1972420116847201104L, -3099427306543083481L, 8244889557672100643L, 8950112648433627211L, pflight_information);
	pflight_information_flight_uuid_SET(2670295366351335260L, 8262578635067378239L, -946202022050135946L, -6863868500986837171L, 4365962839232421030L, 5691421749650657146L, 271143329338012393L, -5085892590103402990L, -6468485241428870386L, -5422002567147704704L, pflight_information);
	
}


void read_RC_CHANNELS_RAW(prc_channels_raw_RC_CHANNELS_RAW *const prc_channels_raw) {
	int16_t chan1_raw    = prc_channels_raw_chan1_raw_GET(prc_channels_raw);
	int16_t chan2_raw    = prc_channels_raw_chan2_raw_GET(prc_channels_raw);
	int16_t chan3_raw    = prc_channels_raw_chan3_raw_GET(prc_channels_raw);
	int16_t chan4_raw    = prc_channels_raw_chan4_raw_GET(prc_channels_raw);
	int16_t chan5_raw    = prc_channels_raw_chan5_raw_GET(prc_channels_raw);
	int16_t chan6_raw    = prc_channels_raw_chan6_raw_GET(prc_channels_raw);
	int16_t chan7_raw    = prc_channels_raw_chan7_raw_GET(prc_channels_raw);
	int16_t chan8_raw    = prc_channels_raw_chan8_raw_GET(prc_channels_raw);
	int32_t time_boot_ms = prc_channels_raw_time_boot_ms_GET(prc_channels_raw);
	int8_t  port         = prc_channels_raw_port_GET(prc_channels_raw);
	int8_t  rssi         = prc_channels_raw_rssi_GET(prc_channels_raw);
	
}


void write_RC_CHANNELS_RAW(prc_channels_raw_RC_CHANNELS_RAW *const prc_channels_raw) {
	prc_channels_raw_chan1_raw_SET(4454, 12374, -30390, 14108, -23626, -31027, -19717, 25559, 31010, -31900, prc_channels_raw);
	prc_channels_raw_chan2_raw_SET(1617, 9592, -3458, 17380, 21317, 10343, -15550, -581, -19828, 18267, prc_channels_raw);
	prc_channels_raw_chan3_raw_SET(8315, 16385, -7789, -13421, -4374, -15908, -27562, -32021, 31413, 28026, prc_channels_raw);
	prc_channels_raw_chan4_raw_SET(-2157, -17602, 15368, -8057, 7992, 913, 19461, 14240, 15307, 31225, prc_channels_raw);
	prc_channels_raw_chan5_raw_SET(-27622, -25165, 19533, 1067, -753, -8132, -23933, 28169, -18048, 21505, prc_channels_raw);
	prc_channels_raw_chan6_raw_SET(28083, 11749, -28240, 30069, -25481, 8223, -13911, -12863, -24136, -21214, prc_channels_raw);
	prc_channels_raw_chan7_raw_SET(27738, -29605, -27118, -9255, -3081, -6381, -25981, -19347, 11368, 3327, prc_channels_raw);
	prc_channels_raw_chan8_raw_SET(27919, 11448, -12625, 8085, -211, -7196, -5568, 1656, -1246, 31951, prc_channels_raw);
	prc_channels_raw_time_boot_ms_SET(1547056219, 526310152, 699508581, 26903724, 352885314, 901542444, -1291924131, 975949779, 232498589, -793173043, prc_channels_raw);
	prc_channels_raw_port_SET(-60, 83, 9, -10, 86, -85, 85, 1, -6, -4, prc_channels_raw);
	prc_channels_raw_rssi_SET(-77, 78, 64, 46, 12, 78, 0, 115, -70, -19, prc_channels_raw);
	
}


void read_SERVO_OUTPUT_RAW(pservo_output_raw_SERVO_OUTPUT_RAW *const pservo_output_raw) {
	int16_t servo1_raw = pservo_output_raw_servo1_raw_GET(pservo_output_raw);
	int16_t servo2_raw = pservo_output_raw_servo2_raw_GET(pservo_output_raw);
	int16_t servo3_raw = pservo_output_raw_servo3_raw_GET(pservo_output_raw);
	int16_t servo4_raw = pservo_output_raw_servo4_raw_GET(pservo_output_raw);
	int16_t servo5_raw = pservo_output_raw_servo5_raw_GET(pservo_output_raw);
	int16_t servo6_raw = pservo_output_raw_servo6_raw_GET(pservo_output_raw);
	int16_t servo7_raw = pservo_output_raw_servo7_raw_GET(pservo_output_raw);
	int16_t servo8_raw = pservo_output_raw_servo8_raw_GET(pservo_output_raw);
	int32_t time_usec  = pservo_output_raw_time_usec_GET(pservo_output_raw);
	int8_t  port       = pservo_output_raw_port_GET(pservo_output_raw);
	int16_t item_servo9_raw;
	if (pservo_output_raw_servo9_raw_GET(pservo_output_raw, &item_servo9_raw)) {
		some_int16_t = item_servo9_raw;
	}
	int16_t item_servo10_raw;
	if (pservo_output_raw_servo10_raw_GET(pservo_output_raw, &item_servo10_raw)) {
		some_int16_t = item_servo10_raw;
	}
	int16_t item_servo11_raw;
	if (pservo_output_raw_servo11_raw_GET(pservo_output_raw, &item_servo11_raw)) {
		some_int16_t = item_servo11_raw;
	}
	int16_t item_servo12_raw;
	if (pservo_output_raw_servo12_raw_GET(pservo_output_raw, &item_servo12_raw)) {
		some_int16_t = item_servo12_raw;
	}
	int16_t item_servo13_raw;
	if (pservo_output_raw_servo13_raw_GET(pservo_output_raw, &item_servo13_raw)) {
		some_int16_t = item_servo13_raw;
	}
	int16_t item_servo14_raw;
	if (pservo_output_raw_servo14_raw_GET(pservo_output_raw, &item_servo14_raw)) {
		some_int16_t = item_servo14_raw;
	}
	int16_t item_servo15_raw;
	if (pservo_output_raw_servo15_raw_GET(pservo_output_raw, &item_servo15_raw)) {
		some_int16_t = item_servo15_raw;
	}
	int16_t item_servo16_raw;
	if (pservo_output_raw_servo16_raw_GET(pservo_output_raw, &item_servo16_raw)) {
		some_int16_t = item_servo16_raw;
	}
	
}


void write_SERVO_OUTPUT_RAW(pservo_output_raw_SERVO_OUTPUT_RAW *const pservo_output_raw) {
	pservo_output_raw_servo1_raw_SET(-10922, -9062, -2676, 5795, -26499, -21240, -19720, 17845, 20168, 21330, pservo_output_raw);
	pservo_output_raw_servo2_raw_SET(23331, -28250, -19301, 19803, 27873, 15104, 23635, 8325, 27845, -31322, pservo_output_raw);
	pservo_output_raw_servo3_raw_SET(5309, 19852, 20595, -21896, -24424, 12093, 5837, 23156, 26591, 7244, pservo_output_raw);
	pservo_output_raw_servo4_raw_SET(9198, 476, 21721, -13619, -17354, 10109, -19523, 28513, -22698, 24062, pservo_output_raw);
	pservo_output_raw_servo5_raw_SET(-28395, -634, -14369, -27822, 8576, -22298, 18489, -13691, 8898, -1802, pservo_output_raw);
	pservo_output_raw_servo6_raw_SET(-32132, -7677, 5180, -19420, 22306, 18722, -2090, -7600, 16397, 5684, pservo_output_raw);
	pservo_output_raw_servo7_raw_SET(31149, 18779, -14920, 19092, -22236, -22829, -11807, -2718, 15103, 26195, pservo_output_raw);
	pservo_output_raw_servo8_raw_SET(1855, 31628, 25112, 20246, -26692, 24514, -21373, 32186, -32417, 17524, pservo_output_raw);
	pservo_output_raw_time_usec_SET(1721982793, -1985277980, 1338477023, 1238297900, -1694496451, 1243784671, 1144390232, -1712530237, 1169533421, -841596443, pservo_output_raw);
	pservo_output_raw_port_SET(7, 0, 29, -74, -9, 13, -69, -81, 95, -106, pservo_output_raw);
	pservo_output_raw_servo9_raw_SET(-23333, 26201, 11031, -9762, 9341, -7916, -12287, -32628, -27845, -15159, pservo_output_raw);
	pservo_output_raw_servo10_raw_SET(32376, 22682, 6339, -13679, -21731, 1549, 434, 28244, -13934, 11170, pservo_output_raw);
	pservo_output_raw_servo11_raw_SET(29679, 3939, -25731, 21576, 24812, -20266, 901, 20671, -3972, 7655, pservo_output_raw);
	pservo_output_raw_servo12_raw_SET(25093, -2899, -328, 10927, 23798, 11889, 22485, 26912, -24397, -14344, pservo_output_raw);
	pservo_output_raw_servo13_raw_SET(-1400, 14463, 210, -10303, -7293, 3700, 16830, -23937, -32461, -2061, pservo_output_raw);
	pservo_output_raw_servo14_raw_SET(-15406, -6557, 14931, 24835, -4664, -18277, 4402, -24785, 20183, -23996, pservo_output_raw);
	pservo_output_raw_servo15_raw_SET(-28972, 25794, -27970, -6317, -3988, 9755, -29267, 14974, 14340, 29162, pservo_output_raw);
	pservo_output_raw_servo16_raw_SET(-31478, -27959, -17545, 20261, 1882, 22315, 19607, -13815, -28748, -19457, pservo_output_raw);
	
}


void read_MEMINFO(pmeminfo_MEMINFO *const pmeminfo) {
	int16_t brkval  = pmeminfo_brkval_GET(pmeminfo);
	int16_t freemem = pmeminfo_freemem_GET(pmeminfo);
	int32_t item_freemem32;
	if (pmeminfo_freemem32_GET(pmeminfo, &item_freemem32)) {
		some_int32_t = item_freemem32;
	}
	
}


void read_MISSION_ITEM_REACHED(pmission_item_reached_MISSION_ITEM_REACHED *const pmission_item_reached) {
	int16_t seq = pmission_item_reached_seq_GET(pmission_item_reached);
	
}


void write_MISSION_ITEM_REACHED(pmission_item_reached_MISSION_ITEM_REACHED *const pmission_item_reached) {
	pmission_item_reached_seq_SET(25528, -23906, -11921, 24012, -17958, -11012, 28984, 32094, -29512, 28406, pmission_item_reached);
	
}


void read_LOGGING_ACK(plogging_ack_LOGGING_ACK *const plogging_ack) {
	int16_t sequence         = plogging_ack_sequence_GET(plogging_ack);
	int8_t  target_system    = plogging_ack_target_system_GET(plogging_ack);
	int8_t  target_component = plogging_ack_target_component_GET(plogging_ack);
	
}


void read_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE *const pvision_speed_estimate) {
	int64_t usec = pvision_speed_estimate_usec_GET(pvision_speed_estimate);
	float   x    = pvision_speed_estimate_x_GET(pvision_speed_estimate);
	float   y    = pvision_speed_estimate_y_GET(pvision_speed_estimate);
	float   z    = pvision_speed_estimate_z_GET(pvision_speed_estimate);
	
}


void write_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE *const pvision_speed_estimate) {
	pvision_speed_estimate_usec_SET(2646962826633692299L, -3680657520949700824L, -5103005150880616149L, 3512435560991808473L, 3870633600219504789L, -7098733225620096660L, 3106249915152968630L, 1927599473422498484L, 9043468450091139327L, 6200583854043772564L, pvision_speed_estimate);
	pvision_speed_estimate_x_SET(-3.15166E38F, -3.836043E37F, -3.0131304E37F, 2.0860683E38F, -2.4464178E38F, 1.4497171E38F, -3.0596826E38F, -2.8216503E38F, 5.3992996E37F, 1.0360076E38F, pvision_speed_estimate);
	pvision_speed_estimate_y_SET(4.1552666E37F, 2.8336193E38F, -3.2909386E37F, -1.7020672E38F, -2.891935E38F, 3.2680686E38F, -3.4549168E37F, 3.2973821E38F, -3.1061525E38F, 1.857155E38F, pvision_speed_estimate);
	pvision_speed_estimate_z_SET(-1.0834365E38F, -7.285947E37F, -8.4856506E37F, -1.5536395E38F, -2.7144238E37F, 8.30618E37F, 1.8527622E38F, 5.687839E37F, -2.2083747E38F, -3.0035515E38F, pvision_speed_estimate);
	
}


void read_DEBUG_VECT(pdebug_vect_DEBUG_VECT *const pdebug_vect) {
	int64_t          time_usec = pdebug_vect_time_usec_GET(pdebug_vect);
	float            x         = pdebug_vect_x_GET(pdebug_vect);
	float            y         = pdebug_vect_y_GET(pdebug_vect);
	float            z         = pdebug_vect_z_GET(pdebug_vect);
	Vdebug_vect_name item_name;
	if (pdebug_vect_name_GET(pdebug_vect, &item_name)) {
		memcpy(some_string, item_name.bytes, item_name.len);
	}
	
}


void write_DEBUG_VECT(pdebug_vect_DEBUG_VECT *const pdebug_vect) {
	pdebug_vect_time_usec_SET(4731735606178943480L, -3275345563382540258L, -5656046627103330110L, 2424474948347737914L, 3010018347444046664L, -685271323872819279L, -7235243586420418334L, 7712747420946714890L, 1121554434852075361L, 5243059519691129853L, pdebug_vect);
	pdebug_vect_x_SET(8.1347735E37F, 2.190341E38F, -1.079059E38F, 2.0908923E38F, -2.6060919E38F, -1.9638626E38F, -1.132888E38F, -2.973965E38F, -2.581578E38F, 5.8974194E37F, pdebug_vect);
	pdebug_vect_y_SET(-2.4634781E37F, 2.6234952E38F, 2.6462216E38F, -2.921169E38F, -1.3039822E38F, -6.171909E37F, -1.2589586E38F, -2.7408855E38F, 2.8511669E38F, 2.8261215E38F, pdebug_vect);
	pdebug_vect_z_SET(-3.2944002E38F, 2.0288685E38F, -2.650993E38F, 4.605919E36F, 1.5435604E38F, -2.497049E38F, 7.8290324E37F, -1.4287805E38F, 2.5956944E37F, 1.9137098E38F, pdebug_vect);
	pdebug_vect_name_SET(some_string, strlen(some_string), pdebug_vect);
	
}


void read_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END *const plog_request_end) {
	int8_t target_system    = plog_request_end_target_system_GET(plog_request_end);
	int8_t target_component = plog_request_end_target_component_GET(plog_request_end);
	
}


void write_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END *const plog_request_end) {
	plog_request_end_target_system_SET(-60, -18, -85, 34, 115, 104, -65, 59, 2, -59, plog_request_end);
	plog_request_end_target_component_SET(126, -40, -100, 85, -4, 52, -81, -9, 99, -92, plog_request_end);
	
}


void read_MISSION_ACK(pmission_ack_MISSION_ACK *const pmission_ack) {
	int8_t               target_system    = pmission_ack_target_system_GET(pmission_ack);
	int8_t               target_component = pmission_ack_target_component_GET(pmission_ack);
	e_MAV_MISSION_RESULT item_typE;
	if (pmission_ack_typE_GET(pmission_ack, &item_typE)) {
		e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED = item_typE;
	}
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_ack_mission_type_GET(pmission_ack, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_ACK(pmission_ack_MISSION_ACK *const pmission_ack) {
	pmission_ack_target_system_SET(-50, -47, 118, 10, -8, -55, 32, 76, -71, 107, pmission_ack);
	pmission_ack_target_component_SET(59, 9, -80, 2, 57, -24, -11, 78, -84, 40, pmission_ack);
	pmission_ack_typE_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7, e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1, e_MAV_MISSION_RESULT_MAV_MISSION_NO_SPACE, e_MAV_MISSION_RESULT_MAV_MISSION_DENIED, e_MAV_MISSION_RESULT_MAV_MISSION_ERROR, e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM2, e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7, e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1, e_MAV_MISSION_RESULT_MAV_MISSION_ERROR, e_MAV_MISSION_RESULT_MAV_MISSION_DENIED, pmission_ack);
	pmission_ack_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, pmission_ack);
	
}


void read_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *const pchange_operator_control_ack) {
	int8_t gcs_system_id   = pchange_operator_control_ack_gcs_system_id_GET(pchange_operator_control_ack);
	int8_t control_request = pchange_operator_control_ack_control_request_GET(pchange_operator_control_ack);
	int8_t ack             = pchange_operator_control_ack_ack_GET(pchange_operator_control_ack);
	
}


void write_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *const pchange_operator_control_ack) {
	pchange_operator_control_ack_gcs_system_id_SET(10, -43, -47, 89, 96, 66, 76, -85, 11, 109, pchange_operator_control_ack);
	pchange_operator_control_ack_control_request_SET(90, 36, 10, -101, -86, -88, 103, 78, 108, -54, pchange_operator_control_ack);
	pchange_operator_control_ack_ack_SET(-34, 86, 52, 108, 57, 40, 40, 31, 114, -37, pchange_operator_control_ack);
	
}


void read_MISSION_CURRENT(pmission_current_MISSION_CURRENT *const pmission_current) {
	int16_t seq = pmission_current_seq_GET(pmission_current);
	
}


void write_MISSION_CURRENT(pmission_current_MISSION_CURRENT *const pmission_current) {
	pmission_current_seq_SET(-3759, 13484, 2919, -13968, -3893, -2273, -7952, 902, 9687, -19365, pmission_current);
	
}


void read_SYSTEM_TIME(psystem_time_SYSTEM_TIME *const psystem_time) {
	int32_t time_boot_ms   = psystem_time_time_boot_ms_GET(psystem_time);
	int64_t time_unix_usec = psystem_time_time_unix_usec_GET(psystem_time);
	
}


void write_SYSTEM_TIME(psystem_time_SYSTEM_TIME *const psystem_time) {
	psystem_time_time_boot_ms_SET(-1839210948, 584807058, -894245613, -1722238518, 2089173804, 1518637170, -437043978, 1130453314, -2136012698, 762213160, psystem_time);
	psystem_time_time_unix_usec_SET(7351429349148853546L, -4912458168509742721L, 2597174804786404431L, 5388003652043397104L, -5570504769975777387L, 938692076978193805L, -972309492046011155L, -3746003704999642478L, -4855474047813835401L, 5961363341390730198L, psystem_time);
	
}


void read_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER *const pcamera_trigger) {
	int32_t seq       = pcamera_trigger_seq_GET(pcamera_trigger);
	int64_t time_usec = pcamera_trigger_time_usec_GET(pcamera_trigger);
	
}


void write_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER *const pcamera_trigger) {
	pcamera_trigger_seq_SET(-1589527387, -828849644, 517882061, 1750315542, -1840910296, -184949677, 1743394053, 1258663759, -140581134, -1971002657, pcamera_trigger);
	pcamera_trigger_time_usec_SET(1068261630545042339L, 6834733220850199078L, -8445440708093420316L, -6345233266893448185L, -4292565219141907655L, 6471766884790827986L, -6231239336383752464L, -8921888303287161853L, -9103373837342386696L, 3460664835260925981L, pcamera_trigger);
	
}


void read_GOPRO_SET_RESPONSE(pgopro_set_response_GOPRO_SET_RESPONSE *const pgopro_set_response) {
	e_GOPRO_COMMAND item_cmd_id;
	if (pgopro_set_response_cmd_id_GET(pgopro_set_response, &item_cmd_id)) {
		e_GOPRO_COMMAND_GOPRO_COMMAND_POWER = item_cmd_id;
	}
	e_GOPRO_REQUEST_STATUS item_status;
	if (pgopro_set_response_status_GET(pgopro_set_response, &item_status)) {
		e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS = item_status;
	}
	
}


void read_VISION_POSITION_ESTIMATE(pvision_position_estimate_VISION_POSITION_ESTIMATE *const pvision_position_estimate) {
	int64_t usec  = pvision_position_estimate_usec_GET(pvision_position_estimate);
	float   x     = pvision_position_estimate_x_GET(pvision_position_estimate);
	float   y     = pvision_position_estimate_y_GET(pvision_position_estimate);
	float   z     = pvision_position_estimate_z_GET(pvision_position_estimate);
	float   roll  = pvision_position_estimate_roll_GET(pvision_position_estimate);
	float   pitch = pvision_position_estimate_pitch_GET(pvision_position_estimate);
	float   yaw   = pvision_position_estimate_yaw_GET(pvision_position_estimate);
	
}


void write_VISION_POSITION_ESTIMATE(pvision_position_estimate_VISION_POSITION_ESTIMATE *const pvision_position_estimate) {
	pvision_position_estimate_usec_SET(4544960355803589477L, -493386736799146606L, 2194393503632293657L, -6350294761929837089L, 3194301845312673586L, -4139913670361753610L, -7511418036228488159L, -6060071824820354740L, -5322275335605567907L, 4603484175833195597L, pvision_position_estimate);
	pvision_position_estimate_x_SET(2.0311067E38F, 2.1648219E38F, 2.8886826E38F, 6.7776496E37F, -7.3944103E37F, 1.2782644E38F, -3.3109833E38F, 1.468321E38F, -1.3769993E38F, -3.0897284E38F, pvision_position_estimate);
	pvision_position_estimate_y_SET(2.3887939E38F, -1.87224E38F, 1.4313609E38F, -8.800496E37F, 1.271353E38F, -3.166383E38F, -1.4265585E38F, -5.5224565E37F, -1.6398989E38F, 1.4140404E38F, pvision_position_estimate);
	pvision_position_estimate_z_SET(3.0683886E38F, 1.3296246E37F, -6.414232E37F, 1.4368201E38F, -1.3892982E38F, -3.0701037E38F, 2.6179922E38F, 1.9587843E38F, 3.1345004E38F, 3.2272516E38F, pvision_position_estimate);
	pvision_position_estimate_roll_SET(3.2926973E38F, 3.2202775E38F, -2.2678657E38F, 2.7759457E38F, -8.018598E37F, 2.1130117E38F, -2.3070154E38F, -6.8081107E37F, 1.260251E37F, -1.0294027E38F, pvision_position_estimate);
	pvision_position_estimate_pitch_SET(-2.1664394E36F, -5.0386373E37F, -5.576813E37F, 4.835499E37F, 3.0099041E38F, -4.53499E37F, 1.9163354E38F, 3.2774056E38F, -4.256422E37F, 2.5120332E38F, pvision_position_estimate);
	pvision_position_estimate_yaw_SET(-8.0945216E37F, 1.2684094E38F, 6.466529E37F, 5.5067253E35F, -7.449619E37F, 3.2136117E38F, 1.4486621E38F, -2.1227118E38F, 2.9495245E37F, 6.9304857E37F, pvision_position_estimate);
	
}


void read_MANUAL_CONTROL(pmanual_control_MANUAL_CONTROL *const pmanual_control) {
	int16_t buttons = pmanual_control_buttons_GET(pmanual_control);
	int8_t  target  = pmanual_control_target_GET(pmanual_control);
	int16_t x       = pmanual_control_x_GET(pmanual_control);
	int16_t y       = pmanual_control_y_GET(pmanual_control);
	int16_t z       = pmanual_control_z_GET(pmanual_control);
	int16_t r       = pmanual_control_r_GET(pmanual_control);
	
}


void write_MANUAL_CONTROL(pmanual_control_MANUAL_CONTROL *const pmanual_control) {
	pmanual_control_buttons_SET(-4496, -7991, 29729, -3018, 25633, -31482, -10016, 31608, 31790, -29710, pmanual_control);
	pmanual_control_target_SET(-79, -30, 13, -88, 91, 9, -116, 119, -72, -14, pmanual_control);
	pmanual_control_x_SET(-18735, -10104, 8912, -1342, -10620, 798, -20220, -13829, -22207, 25999, pmanual_control);
	pmanual_control_y_SET(2510, 29417, 1129, -6267, 24247, -15325, 26094, -14490, 15931, -30183, pmanual_control);
	pmanual_control_z_SET(-7309, 18680, 1383, 7046, 12782, -23177, -18230, -32670, 25233, 10756, pmanual_control);
	pmanual_control_r_SET(-13639, 5490, -24044, 4530, -29878, -31708, -22444, 9900, 11596, -4670, pmanual_control);
	
}


void read_RC_CHANNELS(prc_channels_RC_CHANNELS *const prc_channels) {
	int16_t chan1_raw    = prc_channels_chan1_raw_GET(prc_channels);
	int16_t chan2_raw    = prc_channels_chan2_raw_GET(prc_channels);
	int16_t chan3_raw    = prc_channels_chan3_raw_GET(prc_channels);
	int16_t chan4_raw    = prc_channels_chan4_raw_GET(prc_channels);
	int16_t chan5_raw    = prc_channels_chan5_raw_GET(prc_channels);
	int16_t chan6_raw    = prc_channels_chan6_raw_GET(prc_channels);
	int16_t chan7_raw    = prc_channels_chan7_raw_GET(prc_channels);
	int16_t chan8_raw    = prc_channels_chan8_raw_GET(prc_channels);
	int16_t chan9_raw    = prc_channels_chan9_raw_GET(prc_channels);
	int16_t chan10_raw   = prc_channels_chan10_raw_GET(prc_channels);
	int16_t chan11_raw   = prc_channels_chan11_raw_GET(prc_channels);
	int16_t chan12_raw   = prc_channels_chan12_raw_GET(prc_channels);
	int16_t chan13_raw   = prc_channels_chan13_raw_GET(prc_channels);
	int16_t chan14_raw   = prc_channels_chan14_raw_GET(prc_channels);
	int16_t chan15_raw   = prc_channels_chan15_raw_GET(prc_channels);
	int16_t chan16_raw   = prc_channels_chan16_raw_GET(prc_channels);
	int16_t chan17_raw   = prc_channels_chan17_raw_GET(prc_channels);
	int16_t chan18_raw   = prc_channels_chan18_raw_GET(prc_channels);
	int32_t time_boot_ms = prc_channels_time_boot_ms_GET(prc_channels);
	int8_t  chancount    = prc_channels_chancount_GET(prc_channels);
	int8_t  rssi         = prc_channels_rssi_GET(prc_channels);
	
}


void write_RC_CHANNELS(prc_channels_RC_CHANNELS *const prc_channels) {
	prc_channels_chan1_raw_SET(19952, -2677, 15346, 11197, -20979, -4807, -27688, 11218, 30036, -13735, prc_channels);
	prc_channels_chan2_raw_SET(-32466, -3295, -26411, -30157, -11303, 10100, 1923, 11198, 5308, 11061, prc_channels);
	prc_channels_chan3_raw_SET(18694, -32066, 13627, -28077, -10095, -21598, -9256, -29064, -1011, -25422, prc_channels);
	prc_channels_chan4_raw_SET(-15217, -15865, 20290, -15117, -4605, -29921, 24603, 31450, -21529, 20548, prc_channels);
	prc_channels_chan5_raw_SET(-2841, 23108, -27216, -20637, 1396, 21585, -30022, -29912, -9309, -6877, prc_channels);
	prc_channels_chan6_raw_SET(-10519, 12517, -12860, -31700, 16113, -28867, -31274, -20913, -23634, -2821, prc_channels);
	prc_channels_chan7_raw_SET(-15859, 25860, -20989, -10623, -17740, -29520, -15636, 16245, 24046, -18286, prc_channels);
	prc_channels_chan8_raw_SET(-17445, -12244, 17736, 26276, -10474, -3543, -5902, 1659, -2615, 5578, prc_channels);
	prc_channels_chan9_raw_SET(32274, 8430, -31150, 8983, -18751, -13131, 31917, 6644, 28342, -26538, prc_channels);
	prc_channels_chan10_raw_SET(-10141, 776, 21005, 11132, -23487, 16215, -32733, 16656, 28358, -32529, prc_channels);
	prc_channels_chan11_raw_SET(29882, 19457, -507, -19933, 7915, -12364, 17488, 24690, 8438, 7284, prc_channels);
	prc_channels_chan12_raw_SET(19201, -5654, 16541, 28991, 5102, 29412, 2624, 26421, 15660, 31010, prc_channels);
	prc_channels_chan13_raw_SET(-5210, 12970, 28012, -32696, -13635, 21571, 31721, 32675, 4436, -22389, prc_channels);
	prc_channels_chan14_raw_SET(26822, -713, 1931, -3749, -17619, 5781, 8420, -17752, 1794, 28487, prc_channels);
	prc_channels_chan15_raw_SET(13847, -13461, -18055, -22534, 2264, 31536, 30050, -3387, -5402, 28655, prc_channels);
	prc_channels_chan16_raw_SET(28456, 4489, -4838, -23125, 24297, 12113, 30623, -2427, -14842, 10626, prc_channels);
	prc_channels_chan17_raw_SET(13077, 13317, -3971, -32680, -6028, -8156, 20338, -903, -29689, 17087, prc_channels);
	prc_channels_chan18_raw_SET(9571, -6197, 9268, 10726, -22666, 20888, -26607, 4381, -22284, -2214, prc_channels);
	prc_channels_time_boot_ms_SET(1411855081, -1412081090, 562316062, -1224248432, 1781857064, -494898051, 519111673, -976281346, -1528674016, 1758141506, prc_channels);
	prc_channels_chancount_SET(-14, 124, -14, -35, -50, -90, 48, -108, -114, -33, prc_channels);
	prc_channels_rssi_SET(-33, 107, 116, -107, 79, -24, -70, 52, 124, 31, prc_channels);
	
}


void read_PROTOCOL_VERSION(pprotocol_version_PROTOCOL_VERSION *const pprotocol_version) {
	int16_t                                version                = pprotocol_version_version_GET(pprotocol_version);
	int16_t                                min_version            = pprotocol_version_min_version_GET(pprotocol_version);
	int16_t                                max_version            = pprotocol_version_max_version_GET(pprotocol_version);
	Vprotocol_version_spec_version_hash    item_spec_version_hash = pprotocol_version_spec_version_hash_GET(pprotocol_version);
	for (size_t                            index                  = 0; index < item_spec_version_hash.len; index++)
		some_int8_t                                                  = vprotocol_version_spec_version_hash_GET(&item_spec_version_hash, index);
	Vprotocol_version_library_version_hash item_library_version_hash = pprotocol_version_library_version_hash_GET(pprotocol_version);
	for (size_t                            index                     = 0; index < item_library_version_hash.len; index++)
		some_int8_t = vprotocol_version_library_version_hash_GET(&item_library_version_hash, index);
	
}


void read_RALLY_FETCH_POINT(prally_fetch_point_RALLY_FETCH_POINT *const prally_fetch_point) {
	int8_t target_system    = prally_fetch_point_target_system_GET(prally_fetch_point);
	int8_t target_component = prally_fetch_point_target_component_GET(prally_fetch_point);
	int8_t idx              = prally_fetch_point_idx_GET(prally_fetch_point);
	
}


void read_PARAM_VALUE(pparam_value_PARAM_VALUE *const pparam_value) {
	int16_t               param_count = pparam_value_param_count_GET(pparam_value);
	int16_t               param_index = pparam_value_param_index_GET(pparam_value);
	float                 param_value = pparam_value_param_value_GET(pparam_value);
	Vparam_value_param_id item_param_id;
	if (pparam_value_param_id_GET(pparam_value, &item_param_id)) {
		memcpy(some_string, item_param_id.bytes, item_param_id.len);
	}
	e_MAV_PARAM_TYPE item_param_type;
	if (pparam_value_param_type_GET(pparam_value, &item_param_type)) {
		e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8 = item_param_type;
	}
	
}


void write_PARAM_VALUE(pparam_value_PARAM_VALUE *const pparam_value) {
	pparam_value_param_count_SET(-2834, -28306, -17664, -14835, 7569, -330, 31666, -27508, 13858, 6577, pparam_value);
	pparam_value_param_index_SET(9605, 21867, 11500, -22897, -25462, -26944, -12535, 32238, -14918, 8005, pparam_value);
	pparam_value_param_value_SET(-7.5090267E37F, 7.7084616E37F, -1.5737748E38F, -2.3640236E38F, 2.272229E38F, 1.2031253E38F, 2.7679575E38F, 1.873861E38F, 1.4065757E38F, -2.4454808E38F, pparam_value);
	pparam_value_param_id_SET(some_string, strlen(some_string), pparam_value);
	pparam_value_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, pparam_value);
	
}


void read_BATTERY_STATUS(pbattery_status_BATTERY_STATUS *const pbattery_status) {
	Vbattery_status_voltages item_voltages = pbattery_status_voltages_GET(pbattery_status);
	for (size_t              index         = 0; index < item_voltages.len; index++)
		some_int16_t                           = vbattery_status_voltages_GET(&item_voltages, index);
	int8_t                   id                = pbattery_status_id_GET(pbattery_status);
	int16_t                  temperature       = pbattery_status_temperature_GET(pbattery_status);
	int16_t                  current_battery   = pbattery_status_current_battery_GET(pbattery_status);
	int32_t                  current_consumed  = pbattery_status_current_consumed_GET(pbattery_status);
	int32_t                  energy_consumed   = pbattery_status_energy_consumed_GET(pbattery_status);
	int8_t                   battery_remaining = pbattery_status_battery_remaining_GET(pbattery_status);
	e_MAV_BATTERY_FUNCTION   item_battery_function;
	if (pbattery_status_battery_function_GET(pbattery_status, &item_battery_function)) {
		e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN = item_battery_function;
	}
	e_MAV_BATTERY_TYPE item_typE;
	if (pbattery_status_typE_GET(pbattery_status, &item_typE)) {
		e_MAV_BATTERY_TYPE_UNKNOWN = item_typE;
	}
	
}


void write_BATTERY_STATUS(pbattery_status_BATTERY_STATUS *const pbattery_status) {
	pbattery_status_voltages_SET(&11707, -11124, 3869, -1591, -5778, -20897, -21588, -19191, -8461, 27313, pbattery_status);
	pbattery_status_id_SET(72, 66, -25, -127, -20, 12, 69, -128, 56, -18, pbattery_status);
	pbattery_status_temperature_SET(-6429, -32549, -14055, -14065, 21676, 5532, -826, -461, 9588, 21509, pbattery_status);
	pbattery_status_current_battery_SET(31442, 800, 2166, 32583, 1383, 19330, -19619, -10780, -15610, -24531, pbattery_status);
	pbattery_status_current_consumed_SET(-294731728, -1592954167, -718478511, -1684225910, 1875646378, 1835664939, 1058299874, -583295209, 1877945931, 440506365, pbattery_status);
	pbattery_status_energy_consumed_SET(-1443413866, 223396365, -691492518, -1877840963, -977947491, -1385747763, -1463751792, -1465966117, 521750454, -1001075370, pbattery_status);
	pbattery_status_battery_remaining_SET(-97, 114, -123, 46, 69, -46, -111, -22, -81, -73, pbattery_status);
	pbattery_status_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, pbattery_status);
	pbattery_status_typE_SET(e_MAV_BATTERY_TYPE_LIPO, e_MAV_BATTERY_TYPE_LIPO, e_MAV_BATTERY_TYPE_LION, e_MAV_BATTERY_TYPE_LIFE, e_MAV_BATTERY_TYPE_LION, e_MAV_BATTERY_TYPE_NIMH, e_MAV_BATTERY_TYPE_LION, e_MAV_BATTERY_TYPE_LIPO, e_MAV_BATTERY_TYPE_LIPO, e_MAV_BATTERY_TYPE_NIMH, pbattery_status);
	
}


void read_SERIAL_CONTROL(pserial_control_SERIAL_CONTROL *const pserial_control) {
	int16_t              timeout   = pserial_control_timeout_GET(pserial_control);
	int32_t              baudrate  = pserial_control_baudrate_GET(pserial_control);
	int8_t               count     = pserial_control_count_GET(pserial_control);
	Vserial_control_daTa item_daTa = pserial_control_daTa_GET(pserial_control);
	for (size_t          index     = 0; index < item_daTa.len; index++)
		some_int8_t = vserial_control_daTa_GET(&item_daTa, index);
	e_SERIAL_CONTROL_DEV item_device;
	if (pserial_control_device_GET(pserial_control, &item_device)) {
		e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1 = item_device;
	}
	e_SERIAL_CONTROL_FLAG item_flags;
	if (pserial_control_flags_GET(pserial_control, &item_flags)) {
		e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY = item_flags;
	}
	
}


void write_SERIAL_CONTROL(pserial_control_SERIAL_CONTROL *const pserial_control) {
	pserial_control_timeout_SET(19038, -7836, -1809, 24778, -23456, 2534, -23751, 21620, -16871, 11209, pserial_control);
	pserial_control_baudrate_SET(-752951885, -1896218643, 1861039032, -883507433, -449155620, 1271499335, 1211941484, 386197741, 1521380442, -876055226, pserial_control);
	pserial_control_count_SET(-108, 54, -90, -102, 44, -87, 2, -69, 27, -95, pserial_control);
	pserial_control_daTa_SET(&-79, 67, 39, -84, -51, 111, 48, 33, 87, -50, pserial_control);
	pserial_control_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, pserial_control);
	pserial_control_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND, pserial_control);
	
}


void read_SET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *const pset_position_target_local_ned) {
	int16_t     type_mask        = pset_position_target_local_ned_type_mask_GET(pset_position_target_local_ned);
	int32_t     time_boot_ms     = pset_position_target_local_ned_time_boot_ms_GET(pset_position_target_local_ned);
	int8_t      target_system    = pset_position_target_local_ned_target_system_GET(pset_position_target_local_ned);
	int8_t      target_component = pset_position_target_local_ned_target_component_GET(pset_position_target_local_ned);
	float       x                = pset_position_target_local_ned_x_GET(pset_position_target_local_ned);
	float       y                = pset_position_target_local_ned_y_GET(pset_position_target_local_ned);
	float       z                = pset_position_target_local_ned_z_GET(pset_position_target_local_ned);
	float       vx               = pset_position_target_local_ned_vx_GET(pset_position_target_local_ned);
	float       vy               = pset_position_target_local_ned_vy_GET(pset_position_target_local_ned);
	float       vz               = pset_position_target_local_ned_vz_GET(pset_position_target_local_ned);
	float       afx              = pset_position_target_local_ned_afx_GET(pset_position_target_local_ned);
	float       afy              = pset_position_target_local_ned_afy_GET(pset_position_target_local_ned);
	float       afz              = pset_position_target_local_ned_afz_GET(pset_position_target_local_ned);
	float       yaw              = pset_position_target_local_ned_yaw_GET(pset_position_target_local_ned);
	float       yaw_rate         = pset_position_target_local_ned_yaw_rate_GET(pset_position_target_local_ned);
	e_MAV_FRAME item_coordinate_frame;
	if (pset_position_target_local_ned_coordinate_frame_GET(pset_position_target_local_ned, &item_coordinate_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_coordinate_frame;
	}
	
}


void write_SET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *const pset_position_target_local_ned) {
	pset_position_target_local_ned_type_mask_SET(-26187, -16061, -7927, -14695, -6643, -29675, -21851, 30335, -17753, 14109, pset_position_target_local_ned);
	pset_position_target_local_ned_time_boot_ms_SET(106603286, -864052861, 231004994, 869968545, -1638192717, 1612092433, -887165922, 808326116, -1076629577, 1218504090, pset_position_target_local_ned);
	pset_position_target_local_ned_target_system_SET(-112, -47, 103, 17, 11, 34, -76, -11, -19, 96, pset_position_target_local_ned);
	pset_position_target_local_ned_target_component_SET(52, -127, 59, -30, -18, -47, -1, 35, -95, 114, pset_position_target_local_ned);
	pset_position_target_local_ned_x_SET(-1.6948275E38F, -1.2732497E38F, 1.1458058E38F, -3.1067872E37F, -2.3145193E38F, -3.2850581E38F, 1.7256876E38F, -2.487342E38F, 7.933511E37F, -1.855886E38F, pset_position_target_local_ned);
	pset_position_target_local_ned_y_SET(8.909285E37F, 2.9849659E38F, -4.313128E36F, 1.7544398E38F, 2.1895323E37F, -2.5172273E38F, -3.10552E37F, 2.739549E38F, -2.2118994E38F, -2.5641856E38F, pset_position_target_local_ned);
	pset_position_target_local_ned_z_SET(-1.4410036E38F, 6.042612E37F, 9.443974E37F, 3.1940347E38F, -2.9417952E38F, -6.0784166E37F, -1.7970767E38F, -2.4704596E38F, 2.2384584E38F, 1.5277287E38F, pset_position_target_local_ned);
	pset_position_target_local_ned_vx_SET(7.9511645E37F, 3.3786442E38F, -2.0895015E38F, -2.1996506E38F, 5.5845845E37F, 2.9241503E38F, -2.0566114E38F, -5.065405E37F, 3.388583E38F, -2.179463E38F, pset_position_target_local_ned);
	pset_position_target_local_ned_vy_SET(-2.345289E38F, -2.3691223E37F, 4.851553E37F, -2.7911011E38F, -2.1504062E38F, -2.3807485E37F, 1.5881361E38F, -3.2289495E38F, -3.3516722E38F, 9.558181E37F, pset_position_target_local_ned);
	pset_position_target_local_ned_vz_SET(1.1188325E38F, -2.5300963E38F, -7.1371665E37F, -5.99475E37F, 3.1621068E38F, -2.1535319E38F, 1.2409274E38F, 3.3891971E38F, -3.6974176E37F, 1.0898361E38F, pset_position_target_local_ned);
	pset_position_target_local_ned_afx_SET(-2.9421244E38F, -1.3085809E38F, -1.2317183E38F, -1.889378E38F, 5.512089E37F, 1.3481865E38F, 2.4157258E38F, 1.1436087E38F, 2.9925505E38F, 1.1149469E38F, pset_position_target_local_ned);
	pset_position_target_local_ned_afy_SET(2.6705547E38F, -3.3158038E38F, -1.8409033E38F, 3.7790292E37F, -2.8246894E38F, 1.1612262E38F, -1.5914496E38F, -3.3993078E37F, -2.7108974E38F, -6.2325923E37F, pset_position_target_local_ned);
	pset_position_target_local_ned_afz_SET(-5.3087407E37F, 2.3416752E38F, 1.815603E38F, -6.7366583E37F, -1.3153529E38F, -2.0295778E38F, 1.9584435E38F, -2.0522162E38F, -5.537619E37F, -4.5929664E37F, pset_position_target_local_ned);
	pset_position_target_local_ned_yaw_SET(2.1509217E37F, 1.647854E38F, 2.7851253E38F, 1.0077382E38F, 1.1966798E38F, -1.8729241E38F, 2.0109135E38F, 3.344626E38F, -1.8516288E38F, 1.8176252E38F, pset_position_target_local_ned);
	pset_position_target_local_ned_yaw_rate_SET(-1.3972702E38F, 1.4650353E38F, -2.39905E38F, 2.3070574E38F, 7.906168E37F, -4.2900034E37F, 2.8263954E38F, -2.9503186E37F, -1.6193702E38F, 3.2499673E38F, pset_position_target_local_ned);
	pset_position_target_local_ned_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, pset_position_target_local_ned);
	
}


void read_MOUNT_ORIENTATION(pmount_orientation_MOUNT_ORIENTATION *const pmount_orientation) {
	int32_t time_boot_ms = pmount_orientation_time_boot_ms_GET(pmount_orientation);
	float   roll         = pmount_orientation_roll_GET(pmount_orientation);
	float   pitch        = pmount_orientation_pitch_GET(pmount_orientation);
	float   yaw          = pmount_orientation_yaw_GET(pmount_orientation);
	
}


void read_SET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *const pset_gps_global_origin) {
	int8_t  target_system = pset_gps_global_origin_target_system_GET(pset_gps_global_origin);
	int32_t latitude      = pset_gps_global_origin_latitude_GET(pset_gps_global_origin);
	int32_t longitude     = pset_gps_global_origin_longitude_GET(pset_gps_global_origin);
	int32_t altitude      = pset_gps_global_origin_altitude_GET(pset_gps_global_origin);
	int64_t item_time_usec;
	if (pset_gps_global_origin_time_usec_GET(pset_gps_global_origin, &item_time_usec)) {
		some_int64_t = item_time_usec;
	}
	
}


void write_SET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *const pset_gps_global_origin) {
	pset_gps_global_origin_target_system_SET(-122, 70, 113, -105, 84, 34, 39, 63, -61, -5, pset_gps_global_origin);
	pset_gps_global_origin_latitude_SET(1847163615, -1761379268, -1063497325, 872247045, -1424809909, 1981882570, -1143796620, -414970339, 1791642633, -879012377, pset_gps_global_origin);
	pset_gps_global_origin_longitude_SET(-115808199, -1997511660, -1925436349, -654221479, 1506737906, 112778333, -338800546, 1370079211, -922294102, 661995738, pset_gps_global_origin);
	pset_gps_global_origin_altitude_SET(-728046885, -1647892595, -732841752, 1252010797, -494104091, 724118394, -382134055, 970001315, 1106733849, 1700929485, pset_gps_global_origin);
	pset_gps_global_origin_time_usec_SET(-8049153204908383674L, -3088607228885651658L, 1866953272322449896L, -2604013524001615264L, -653912177628814214L, 57097185717705857L, 4775761205970435477L, 972407301532543395L, -2100856210506714438L, -8671116253161445104L, pset_gps_global_origin);
	
}


void read_PARAM_EXT_SET(pparam_ext_set_PARAM_EXT_SET *const pparam_ext_set) {
	int8_t                  target_system    = pparam_ext_set_target_system_GET(pparam_ext_set);
	int8_t                  target_component = pparam_ext_set_target_component_GET(pparam_ext_set);
	Vparam_ext_set_param_id item_param_id;
	if (pparam_ext_set_param_id_GET(pparam_ext_set, &item_param_id)) {
		memcpy(some_string, item_param_id.bytes, item_param_id.len);
	}
	Vparam_ext_set_param_value item_param_value;
	if (pparam_ext_set_param_value_GET(pparam_ext_set, &item_param_value)) {
		memcpy(some_string, item_param_value.bytes, item_param_value.len);
	}
	e_MAV_PARAM_EXT_TYPE item_param_type;
	if (pparam_ext_set_param_type_GET(pparam_ext_set, &item_param_type)) {
		e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8 = item_param_type;
	}
	
}


void read_AUTOPILOT_VERSION(pautopilot_version_AUTOPILOT_VERSION *const pautopilot_version) {
	int16_t                                      vendor_id                  = pautopilot_version_vendor_id_GET(pautopilot_version);
	int16_t                                      product_id                 = pautopilot_version_product_id_GET(pautopilot_version);
	int32_t                                      flight_sw_version          = pautopilot_version_flight_sw_version_GET(pautopilot_version);
	int32_t                                      middleware_sw_version      = pautopilot_version_middleware_sw_version_GET(pautopilot_version);
	int32_t                                      os_sw_version              = pautopilot_version_os_sw_version_GET(pautopilot_version);
	int32_t                                      board_version              = pautopilot_version_board_version_GET(pautopilot_version);
	int64_t                                      uid                        = pautopilot_version_uid_GET(pautopilot_version);
	Vautopilot_version_flight_custom_version     item_flight_custom_version = pautopilot_version_flight_custom_version_GET(pautopilot_version);
	for (size_t                                  index                      = 0; index < item_flight_custom_version.len; index++)
		some_int8_t                                                             = vautopilot_version_flight_custom_version_GET(&item_flight_custom_version, index);
	Vautopilot_version_middleware_custom_version item_middleware_custom_version = pautopilot_version_middleware_custom_version_GET(pautopilot_version);
	for (size_t                                  index                          = 0; index < item_middleware_custom_version.len; index++)
		some_int8_t                                                     = vautopilot_version_middleware_custom_version_GET(&item_middleware_custom_version, index);
	Vautopilot_version_os_custom_version         item_os_custom_version = pautopilot_version_os_custom_version_GET(pautopilot_version);
	for (size_t                                  index                  = 0; index < item_os_custom_version.len; index++)
		some_int8_t = vautopilot_version_os_custom_version_GET(&item_os_custom_version, index);
	e_MAV_PROTOCOL_CAPABILITY                    item_capabilities;
	if (pautopilot_version_capabilities_GET(pautopilot_version, &item_capabilities)) {
		e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = item_capabilities;
	}
	pautopilot_version_uid2_d0 (pautopilot_version) {
				some_int8_t = vautopilot_version_uid2_GET(&item_uid2);
			}
	
}


void write_AUTOPILOT_VERSION(pautopilot_version_AUTOPILOT_VERSION *const pautopilot_version) {
	pautopilot_version_vendor_id_SET(9346, -28746, -25357, 9343, 25352, -32479, -24873, 29581, 2707, -22830, pautopilot_version);
	pautopilot_version_product_id_SET(18795, 27758, 26028, -7346, 16002, -26181, -17078, 221, -15873, 15197, pautopilot_version);
	pautopilot_version_flight_sw_version_SET(2086075335, -1106786771, -697137163, -45684307, -238346959, 1386786375, 2090955879, 1258822640, -1044446528, -439761577, pautopilot_version);
	pautopilot_version_middleware_sw_version_SET(-2113228020, -1765874939, -126748852, 373383337, 272736501, -2087282676, 102371789, -794853406, 641872903, -1010627711, pautopilot_version);
	pautopilot_version_os_sw_version_SET(-350979243, 1042111273, -1856012733, -524681703, 1630716854, -2064837599, -1396080709, -1139366305, 792127316, 895975100, pautopilot_version);
	pautopilot_version_board_version_SET(782481780, 1133974362, 246300791, -1345860822, 961666953, -644908858, 1545103170, -801147108, 394326978, -2131028205, pautopilot_version);
	pautopilot_version_uid_SET(6116683559106216873L, -6666073382061361511L, -1927488881751704241L, -6018309427609180212L, 1894535826787976073L, 6438634317237463421L, 8043314481724527190L, -6783626554388308397L, 4112705223134952109L, 8530268415461151338L, pautopilot_version);
	pautopilot_version_flight_custom_version_SET(&-126, 81, -5, -110, -128, -29, 120, 12, -126, 62, pautopilot_version);
	pautopilot_version_middleware_custom_version_SET(&94, -121, -101, -7, -71, -57, -98, 96, 58, -5, pautopilot_version);
	pautopilot_version_os_custom_version_SET(&-13, -108, 66, 104, 42, 44, -10, -115, 3, -108, pautopilot_version);
	pautopilot_version_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET, pautopilot_version);
	for (size_t d0 = 0; d0 < Pautopilot_version_uid2_D0; d0++) {
		pautopilot_version_uid2_SET(63, -85, -41, 99, -31, 107, -32, -125, 98, 61, pautopilot_version, d0);
	}
	
}


void read_MISSION_REQUEST_LIST(pmission_request_list_MISSION_REQUEST_LIST *const pmission_request_list) {
	int8_t             target_system    = pmission_request_list_target_system_GET(pmission_request_list);
	int8_t             target_component = pmission_request_list_target_component_GET(pmission_request_list);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_request_list_mission_type_GET(pmission_request_list, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_REQUEST_LIST(pmission_request_list_MISSION_REQUEST_LIST *const pmission_request_list) {
	pmission_request_list_target_system_SET(45, -15, 37, 43, -3, 25, 17, -16, 101, 81, pmission_request_list);
	pmission_request_list_target_component_SET(37, -49, -98, 66, 71, 127, 38, 122, 68, 5, pmission_request_list);
	pmission_request_list_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, pmission_request_list);
	
}


void read_SIMSTATE(psimstate_SIMSTATE *const psimstate) {
	float   roll  = psimstate_roll_GET(psimstate);
	float   pitch = psimstate_pitch_GET(psimstate);
	float   yaw   = psimstate_yaw_GET(psimstate);
	float   xacc  = psimstate_xacc_GET(psimstate);
	float   yacc  = psimstate_yacc_GET(psimstate);
	float   zacc  = psimstate_zacc_GET(psimstate);
	float   xgyro = psimstate_xgyro_GET(psimstate);
	float   ygyro = psimstate_ygyro_GET(psimstate);
	float   zgyro = psimstate_zgyro_GET(psimstate);
	int32_t lat   = psimstate_lat_GET(psimstate);
	int32_t lng   = psimstate_lng_GET(psimstate);
	
}


void read_SET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *const pset_video_stream_settings) {
	int16_t                        resolution_h     = pset_video_stream_settings_resolution_h_GET(pset_video_stream_settings);
	int16_t                        resolution_v     = pset_video_stream_settings_resolution_v_GET(pset_video_stream_settings);
	int16_t                        rotation         = pset_video_stream_settings_rotation_GET(pset_video_stream_settings);
	int32_t                        bitrate          = pset_video_stream_settings_bitrate_GET(pset_video_stream_settings);
	int8_t                         target_system    = pset_video_stream_settings_target_system_GET(pset_video_stream_settings);
	int8_t                         target_component = pset_video_stream_settings_target_component_GET(pset_video_stream_settings);
	int8_t                         camera_id        = pset_video_stream_settings_camera_id_GET(pset_video_stream_settings);
	float                          framerate        = pset_video_stream_settings_framerate_GET(pset_video_stream_settings);
	Vset_video_stream_settings_uri item_uri;
	if (pset_video_stream_settings_uri_GET(pset_video_stream_settings, &item_uri)) {
		memcpy(some_string, item_uri.bytes, item_uri.len);
	}
	
}


void read_PLAY_TUNE(pplay_tune_PLAY_TUNE *const pplay_tune) {
	int8_t          target_system    = pplay_tune_target_system_GET(pplay_tune);
	int8_t          target_component = pplay_tune_target_component_GET(pplay_tune);
	Vplay_tune_tune item_tune;
	if (pplay_tune_tune_GET(pplay_tune, &item_tune)) {
		memcpy(some_string, item_tune.bytes, item_tune.len);
	}
	
}


void write_PLAY_TUNE(pplay_tune_PLAY_TUNE *const pplay_tune) {
	pplay_tune_target_system_SET(-108, 44, -89, -44, -71, 26, 118, -110, -123, -127, pplay_tune);
	pplay_tune_target_component_SET(-92, -127, 4, -49, -110, 27, 117, -14, 112, -98, pplay_tune);
	pplay_tune_tune_SET(some_string, strlen(some_string), pplay_tune);
	
}


void read_DIGICAM_CONFIGURE(pdigicam_configure_DIGICAM_CONFIGURE *const pdigicam_configure) {
	int16_t shutter_speed    = pdigicam_configure_shutter_speed_GET(pdigicam_configure);
	int8_t  target_system    = pdigicam_configure_target_system_GET(pdigicam_configure);
	int8_t  target_component = pdigicam_configure_target_component_GET(pdigicam_configure);
	int8_t  mode             = pdigicam_configure_mode_GET(pdigicam_configure);
	int8_t  aperture         = pdigicam_configure_aperture_GET(pdigicam_configure);
	int8_t  iso              = pdigicam_configure_iso_GET(pdigicam_configure);
	int8_t  exposure_type    = pdigicam_configure_exposure_type_GET(pdigicam_configure);
	int8_t  command_id       = pdigicam_configure_command_id_GET(pdigicam_configure);
	int8_t  engine_cut_off   = pdigicam_configure_engine_cut_off_GET(pdigicam_configure);
	int8_t  extra_param      = pdigicam_configure_extra_param_GET(pdigicam_configure);
	float   extra_value      = pdigicam_configure_extra_value_GET(pdigicam_configure);
	
}


void read_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3 *const pscaled_pressure3) {
	int32_t time_boot_ms = pscaled_pressure3_time_boot_ms_GET(pscaled_pressure3);
	float   press_abs    = pscaled_pressure3_press_abs_GET(pscaled_pressure3);
	float   press_diff   = pscaled_pressure3_press_diff_GET(pscaled_pressure3);
	int16_t temperature  = pscaled_pressure3_temperature_GET(pscaled_pressure3);
	
}


void write_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3 *const pscaled_pressure3) {
	pscaled_pressure3_time_boot_ms_SET(-465477089, -249637748, 1301402898, -1503662603, -247239147, -762482896, 1044990476, 847565631, 338814577, 350897158, pscaled_pressure3);
	pscaled_pressure3_press_abs_SET(-1.4206378E38F, 8.217732E37F, -2.1782192E38F, -1.8355487E38F, -2.4500859E38F, -2.1761476E38F, -1.978968E38F, 2.3458862E38F, 2.8930916E38F, -2.0926165E38F, pscaled_pressure3);
	pscaled_pressure3_press_diff_SET(2.1409043E38F, 3.2279895E38F, 2.7418406E38F, 1.9643355E38F, -9.538415E37F, 2.2234654E38F, -8.398739E37F, 2.8557284E38F, -6.760005E37F, -2.326896E37F, pscaled_pressure3);
	pscaled_pressure3_temperature_SET(19480, 21492, -30727, 23506, -26784, 17210, -32174, 17373, -30840, 7189, pscaled_pressure3);
	
}


void read_MISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *const pmission_request_partial_list) {
	int8_t             target_system    = pmission_request_partial_list_target_system_GET(pmission_request_partial_list);
	int8_t             target_component = pmission_request_partial_list_target_component_GET(pmission_request_partial_list);
	int16_t            start_index      = pmission_request_partial_list_start_index_GET(pmission_request_partial_list);
	int16_t            end_index        = pmission_request_partial_list_end_index_GET(pmission_request_partial_list);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_request_partial_list_mission_type_GET(pmission_request_partial_list, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *const pmission_request_partial_list) {
	pmission_request_partial_list_target_system_SET(-100, 13, -50, 55, 59, 8, -64, -53, -16, -91, pmission_request_partial_list);
	pmission_request_partial_list_target_component_SET(62, -34, -46, 75, 115, 39, 120, 25, -61, 65, pmission_request_partial_list);
	pmission_request_partial_list_start_index_SET(-24755, -28242, 28564, 32293, 26696, -28563, 1246, 25596, 9971, -20281, pmission_request_partial_list);
	pmission_request_partial_list_end_index_SET(-16961, -20955, 9075, -15963, -5937, -14840, 17641, -13187, -31533, -25133, pmission_request_partial_list);
	pmission_request_partial_list_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, pmission_request_partial_list);
	
}


void read_PARAM_EXT_ACK(pparam_ext_ack_PARAM_EXT_ACK *const pparam_ext_ack) {
	Vparam_ext_ack_param_id item_param_id;
	if (pparam_ext_ack_param_id_GET(pparam_ext_ack, &item_param_id)) {
		memcpy(some_string, item_param_id.bytes, item_param_id.len);
	}
	Vparam_ext_ack_param_value item_param_value;
	if (pparam_ext_ack_param_value_GET(pparam_ext_ack, &item_param_value)) {
		memcpy(some_string, item_param_value.bytes, item_param_value.len);
	}
	e_MAV_PARAM_EXT_TYPE item_param_type;
	if (pparam_ext_ack_param_type_GET(pparam_ext_ack, &item_param_type)) {
		e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8 = item_param_type;
	}
	e_PARAM_ACK item_param_result;
	if (pparam_ext_ack_param_result_GET(pparam_ext_ack, &item_param_result)) {
		e_PARAM_ACK_PARAM_ACK_ACCEPTED = item_param_result;
	}
	
}


void read_UAVCAN_NODE_INFO(puavcan_node_info_UAVCAN_NODE_INFO *const puavcan_node_info) {
	int32_t                        uptime_sec        = puavcan_node_info_uptime_sec_GET(puavcan_node_info);
	int32_t                        sw_vcs_commit     = puavcan_node_info_sw_vcs_commit_GET(puavcan_node_info);
	int64_t                        time_usec         = puavcan_node_info_time_usec_GET(puavcan_node_info);
	int8_t                         hw_version_major  = puavcan_node_info_hw_version_major_GET(puavcan_node_info);
	int8_t                         hw_version_minor  = puavcan_node_info_hw_version_minor_GET(puavcan_node_info);
	Vuavcan_node_info_hw_unique_id item_hw_unique_id = puavcan_node_info_hw_unique_id_GET(puavcan_node_info);
	for (size_t                    index             = 0; index < item_hw_unique_id.len; index++)
		some_int8_t                                 = vuavcan_node_info_hw_unique_id_GET(&item_hw_unique_id, index);
	int8_t                         sw_version_major = puavcan_node_info_sw_version_major_GET(puavcan_node_info);
	int8_t                         sw_version_minor = puavcan_node_info_sw_version_minor_GET(puavcan_node_info);
	Vuavcan_node_info_name         item_name;
	if (puavcan_node_info_name_GET(puavcan_node_info, &item_name)) {
		memcpy(some_string, item_name.bytes, item_name.len);
	}
	
}


void read_DATA16(pdata16_DATA16 *const pdata16) {
	int8_t       typE      = pdata16_typE_GET(pdata16);
	int8_t       len       = pdata16_len_GET(pdata16);
	Vdata16_daTa item_daTa = pdata16_daTa_GET(pdata16);
	for (size_t  index     = 0; index < item_daTa.len; index++)
		some_int8_t = vdata16_daTa_GET(&item_daTa, index);
	
}


void read_SET_MAG_OFFSETS(pset_mag_offsets_SET_MAG_OFFSETS *const pset_mag_offsets) {
	int8_t  target_system    = pset_mag_offsets_target_system_GET(pset_mag_offsets);
	int8_t  target_component = pset_mag_offsets_target_component_GET(pset_mag_offsets);
	int16_t mag_ofs_x        = pset_mag_offsets_mag_ofs_x_GET(pset_mag_offsets);
	int16_t mag_ofs_y        = pset_mag_offsets_mag_ofs_y_GET(pset_mag_offsets);
	int16_t mag_ofs_z        = pset_mag_offsets_mag_ofs_z_GET(pset_mag_offsets);
	
}


void read_AP_ADC(pap_adc_AP_ADC *const pap_adc) {
	int16_t adc1 = pap_adc_adc1_GET(pap_adc);
	int16_t adc2 = pap_adc_adc2_GET(pap_adc);
	int16_t adc3 = pap_adc_adc3_GET(pap_adc);
	int16_t adc4 = pap_adc_adc4_GET(pap_adc);
	int16_t adc5 = pap_adc_adc5_GET(pap_adc);
	int16_t adc6 = pap_adc_adc6_GET(pap_adc);
	
}


void read_WIND(pwind_WIND *const pwind) {
	float direction = pwind_direction_GET(pwind);
	float speed     = pwind_speed_GET(pwind);
	float speed_z   = pwind_speed_z_GET(pwind);
	
}


void read_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *const pautopilot_version_request) {
	int8_t target_system    = pautopilot_version_request_target_system_GET(pautopilot_version_request);
	int8_t target_component = pautopilot_version_request_target_component_GET(pautopilot_version_request);
	
}


void read_LOCAL_POSITION_NED(plocal_position_ned_LOCAL_POSITION_NED *const plocal_position_ned) {
	int32_t time_boot_ms = plocal_position_ned_time_boot_ms_GET(plocal_position_ned);
	float   x            = plocal_position_ned_x_GET(plocal_position_ned);
	float   y            = plocal_position_ned_y_GET(plocal_position_ned);
	float   z            = plocal_position_ned_z_GET(plocal_position_ned);
	float   vx           = plocal_position_ned_vx_GET(plocal_position_ned);
	float   vy           = plocal_position_ned_vy_GET(plocal_position_ned);
	float   vz           = plocal_position_ned_vz_GET(plocal_position_ned);
	
}


void write_LOCAL_POSITION_NED(plocal_position_ned_LOCAL_POSITION_NED *const plocal_position_ned) {
	plocal_position_ned_time_boot_ms_SET(-1448818202, -2145853144, 852102246, 134857304, -388240895, -1866323221, -2074386904, -783098357, -45006439, 401293036, plocal_position_ned);
	plocal_position_ned_x_SET(-2.8679597E38F, -6.273963E37F, -1.8083137E38F, -1.5853137E38F, 2.9153228E38F, 2.0360022E38F, 3.9308048E37F, 6.5049257E37F, -2.9803405E38F, 2.843629E37F, plocal_position_ned);
	plocal_position_ned_y_SET(2.134692E38F, 6.3919596E36F, -2.5075585E38F, -6.979008E37F, -1.7875346E38F, -3.2520434E38F, 3.1042997E38F, -1.4460397E38F, 2.9730293E38F, 2.725898E38F, plocal_position_ned);
	plocal_position_ned_z_SET(-2.0604141E38F, 2.5222446E38F, 9.242817E37F, -3.0154398E38F, 1.6382224E38F, 5.1195245E36F, 2.3949317E38F, -1.8681243E38F, -8.641092E37F, 3.900105E36F, plocal_position_ned);
	plocal_position_ned_vx_SET(3.2579312E38F, 6.9218423E37F, 1.2448182E38F, 6.896762E37F, 3.0952933E38F, -2.2281937E38F, -1.8014303E38F, 8.97347E37F, 3.8411555E37F, 6.814655E37F, plocal_position_ned);
	plocal_position_ned_vy_SET(8.474045E37F, 5.0808927E37F, 1.4749542E38F, 2.2208512E38F, -3.0371913E38F, 2.1000447E37F, -2.3362767E36F, 1.2286773E38F, -1.5251397E38F, 3.1914315E38F, plocal_position_ned);
	plocal_position_ned_vz_SET(2.4055084E38F, -1.2228896E38F, -1.2774901E38F, -2.4085574E38F, 2.1563481E38F, 1.2433823E38F, -1.4416349E38F, 2.4506726E38F, 1.1105385E38F, 1.0456571E38F, plocal_position_ned);
	
}


void read_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *const pdata_transmission_handshake) {
	int16_t width       = pdata_transmission_handshake_width_GET(pdata_transmission_handshake);
	int16_t height      = pdata_transmission_handshake_height_GET(pdata_transmission_handshake);
	int16_t packets     = pdata_transmission_handshake_packets_GET(pdata_transmission_handshake);
	int32_t size        = pdata_transmission_handshake_size_GET(pdata_transmission_handshake);
	int8_t  typE        = pdata_transmission_handshake_typE_GET(pdata_transmission_handshake);
	int8_t  payload     = pdata_transmission_handshake_payload_GET(pdata_transmission_handshake);
	int8_t  jpg_quality = pdata_transmission_handshake_jpg_quality_GET(pdata_transmission_handshake);
	
}


void write_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *const pdata_transmission_handshake) {
	pdata_transmission_handshake_width_SET(-2878, -20490, -26865, -3191, -8079, 5047, 16094, -24185, -30497, -11746, pdata_transmission_handshake);
	pdata_transmission_handshake_height_SET(29853, 181, 9836, 16074, -18938, 8223, 3684, -16394, 11014, 935, pdata_transmission_handshake);
	pdata_transmission_handshake_packets_SET(-6901, -24393, -20432, 30280, 16715, 11897, 18601, 12850, -31747, -25304, pdata_transmission_handshake);
	pdata_transmission_handshake_size_SET(743790249, 5026442, 258738536, 1824466101, -726964825, -123315609, -1053624602, -2013234512, -986515794, 1449729851, pdata_transmission_handshake);
	pdata_transmission_handshake_typE_SET(-16, -38, 63, 19, 73, -2, 92, -26, -36, 63, pdata_transmission_handshake);
	pdata_transmission_handshake_payload_SET(-66, -117, -25, 54, -98, -74, 86, 95, -41, 114, pdata_transmission_handshake);
	pdata_transmission_handshake_jpg_quality_SET(89, -44, 115, -76, -12, 52, -76, -92, -120, 50, pdata_transmission_handshake);
	
}


void read_GPS_GLOBAL_ORIGIN(pgps_global_origin_GPS_GLOBAL_ORIGIN *const pgps_global_origin) {
	int32_t latitude  = pgps_global_origin_latitude_GET(pgps_global_origin);
	int32_t longitude = pgps_global_origin_longitude_GET(pgps_global_origin);
	int32_t altitude  = pgps_global_origin_altitude_GET(pgps_global_origin);
	int64_t item_time_usec;
	if (pgps_global_origin_time_usec_GET(pgps_global_origin, &item_time_usec)) {
		some_int64_t = item_time_usec;
	}
	
}


void write_GPS_GLOBAL_ORIGIN(pgps_global_origin_GPS_GLOBAL_ORIGIN *const pgps_global_origin) {
	pgps_global_origin_latitude_SET(-1232087848, -1774490572, 1033415952, 1242827311, 1931054518, -144594691, 874407968, -1276457621, 1148728606, 785122091, pgps_global_origin);
	pgps_global_origin_longitude_SET(273689346, 278703419, 363520406, 1462370977, 1875263228, 387062598, 1735033154, 723430405, 1482703936, 1794037497, pgps_global_origin);
	pgps_global_origin_altitude_SET(-750345158, 1585245276, -233269838, -117722192, -861782293, 861803537, -1176116466, -478938838, 2084707073, 1239354715, pgps_global_origin);
	pgps_global_origin_time_usec_SET(-7996801343196099392L, 7654250407577123322L, -5823323505187978653L, 3146261060630430320L, -2857114186007891052L, -2057984407636559990L, 3362256511647637662L, 4709752445774174733L, -3087811357141805078L, 7083284359304774411L, pgps_global_origin);
	
}


void read_SCALED_IMU2(pscaled_imu2_SCALED_IMU2 *const pscaled_imu2) {
	int32_t time_boot_ms = pscaled_imu2_time_boot_ms_GET(pscaled_imu2);
	int16_t xacc         = pscaled_imu2_xacc_GET(pscaled_imu2);
	int16_t yacc         = pscaled_imu2_yacc_GET(pscaled_imu2);
	int16_t zacc         = pscaled_imu2_zacc_GET(pscaled_imu2);
	int16_t xgyro        = pscaled_imu2_xgyro_GET(pscaled_imu2);
	int16_t ygyro        = pscaled_imu2_ygyro_GET(pscaled_imu2);
	int16_t zgyro        = pscaled_imu2_zgyro_GET(pscaled_imu2);
	int16_t xmag         = pscaled_imu2_xmag_GET(pscaled_imu2);
	int16_t ymag         = pscaled_imu2_ymag_GET(pscaled_imu2);
	int16_t zmag         = pscaled_imu2_zmag_GET(pscaled_imu2);
	
}


void write_SCALED_IMU2(pscaled_imu2_SCALED_IMU2 *const pscaled_imu2) {
	pscaled_imu2_time_boot_ms_SET(-1766952499, 1568498503, 1137715583, -1720036374, -733280105, -882759315, 1675820324, -378356612, 2001856086, 961603438, pscaled_imu2);
	pscaled_imu2_xacc_SET(-25376, 18805, -32715, -30220, 18091, 21251, 11348, 26039, 12199, -2256, pscaled_imu2);
	pscaled_imu2_yacc_SET(19983, 8142, 14886, -31223, 11620, -25012, -5202, -12907, 5464, -13383, pscaled_imu2);
	pscaled_imu2_zacc_SET(16444, 2106, 3278, -10826, 27310, -29597, 24685, 21864, -30896, -13992, pscaled_imu2);
	pscaled_imu2_xgyro_SET(22122, -21378, -28825, -25774, 26347, -17165, -8213, -7344, 4774, -12000, pscaled_imu2);
	pscaled_imu2_ygyro_SET(24182, -28051, 7963, -2126, -22778, -22908, -22517, -29073, -14635, 31083, pscaled_imu2);
	pscaled_imu2_zgyro_SET(-22072, -18522, -22881, -17241, -16274, 5148, 24649, -26834, 1496, 17978, pscaled_imu2);
	pscaled_imu2_xmag_SET(-27592, -19601, 17550, -29758, -9328, -5952, -6876, 17607, 8551, -854, pscaled_imu2);
	pscaled_imu2_ymag_SET(5125, 9110, -9519, -1988, 12764, -641, 32681, 10523, -14332, 31269, pscaled_imu2);
	pscaled_imu2_zmag_SET(-7049, 29862, -7179, 20096, -13394, 29422, 22652, -8961, -6900, 18866, pscaled_imu2);
	
}


void read_ATTITUDE_QUATERNION(pattitude_quaternion_ATTITUDE_QUATERNION *const pattitude_quaternion) {
	int32_t time_boot_ms = pattitude_quaternion_time_boot_ms_GET(pattitude_quaternion);
	float   q1           = pattitude_quaternion_q1_GET(pattitude_quaternion);
	float   q2           = pattitude_quaternion_q2_GET(pattitude_quaternion);
	float   q3           = pattitude_quaternion_q3_GET(pattitude_quaternion);
	float   q4           = pattitude_quaternion_q4_GET(pattitude_quaternion);
	float   rollspeed    = pattitude_quaternion_rollspeed_GET(pattitude_quaternion);
	float   pitchspeed   = pattitude_quaternion_pitchspeed_GET(pattitude_quaternion);
	float   yawspeed     = pattitude_quaternion_yawspeed_GET(pattitude_quaternion);
	
}


void write_ATTITUDE_QUATERNION(pattitude_quaternion_ATTITUDE_QUATERNION *const pattitude_quaternion) {
	pattitude_quaternion_time_boot_ms_SET(2127774053, -1413252222, -1344599284, 2072414444, 463806135, -1449370664, -587689811, 248597328, 1258481100, 501079043, pattitude_quaternion);
	pattitude_quaternion_q1_SET(-2.321344E38F, -8.677411E37F, -3.1626155E36F, -2.111422E38F, 2.0868898E38F, 7.134142E37F, 2.5184875E38F, -2.6339423E38F, 2.1195453E38F, -2.88568E38F, pattitude_quaternion);
	pattitude_quaternion_q2_SET(-3.3682643E38F, -2.5777239E38F, -1.3977455E38F, -6.0415335E37F, -2.5066854E38F, -3.2459086E38F, 2.5087964E38F, 2.3715599E38F, -3.344493E38F, 7.4413727E37F, pattitude_quaternion);
	pattitude_quaternion_q3_SET(-7.877313E37F, 1.6847499E38F, -2.0157573E38F, 1.9358586E38F, -2.8344706E38F, -3.0632231E38F, 1.3570083E38F, -1.6986458E38F, 1.8323515E37F, 2.023808E38F, pattitude_quaternion);
	pattitude_quaternion_q4_SET(-3.22189E38F, 2.9603838E38F, 1.0085548E38F, 1.4803796E38F, 5.887136E37F, -4.2878043E37F, -2.4392916E38F, -3.3692504E38F, 2.2581686E38F, -2.2077918E38F, pattitude_quaternion);
	pattitude_quaternion_rollspeed_SET(2.540631E38F, 2.4071318E38F, -3.3888371E38F, -1.2930729E38F, -2.4713641E38F, 3.3283225E38F, 8.719888E37F, -1.6029685E38F, -9.793132E37F, 2.6974605E38F, pattitude_quaternion);
	pattitude_quaternion_pitchspeed_SET(-1.6666092E38F, 2.1235281E38F, -2.5629356E38F, 8.08086E37F, 1.0846457E38F, -1.6283253E38F, 6.1698233E37F, -2.061457E37F, -9.774382E37F, -5.2724367E37F, pattitude_quaternion);
	pattitude_quaternion_yawspeed_SET(1.2215897E38F, -2.9283532E38F, -2.5901037E38F, -1.3184104E38F, -3.186415E37F, 2.9667618E37F, 7.752065E36F, -3.3705146E37F, 2.6612497E38F, 4.1055834E36F, pattitude_quaternion);
	
}


void read_DATA64(pdata64_DATA64 *const pdata64) {
	int8_t       typE      = pdata64_typE_GET(pdata64);
	int8_t       len       = pdata64_len_GET(pdata64);
	Vdata64_daTa item_daTa = pdata64_daTa_GET(pdata64);
	for (size_t  index     = 0; index < item_daTa.len; index++)
		some_int8_t = vdata64_daTa_GET(&item_daTa, index);
	
}


void read_HIL_ACTUATOR_CONTROLS(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *const phil_actuator_controls) {
	int64_t                         time_usec     = phil_actuator_controls_time_usec_GET(phil_actuator_controls);
	int64_t                         flags         = phil_actuator_controls_flags_GET(phil_actuator_controls);
	Vhil_actuator_controls_controls item_controls = phil_actuator_controls_controls_GET(phil_actuator_controls);
	for (size_t                     index         = 0; index < item_controls.len; index++)
		some_float = vhil_actuator_controls_controls_GET(&item_controls, index);
	e_MAV_MODE                      item_mode;
	if (phil_actuator_controls_mode_GET(phil_actuator_controls, &item_mode)) {
		e_MAV_MODE_PREFLIGHT = item_mode;
	}
	
}


void write_HIL_ACTUATOR_CONTROLS(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *const phil_actuator_controls) {
	phil_actuator_controls_time_usec_SET(-7137636167818705466L, -4768366933890271305L, -5223203110533546983L, 1169247942754238425L, 5958468625232990875L, -8619801760108074577L, -4686594355473134747L, -4092125617572398701L, 5769661444629536655L, 7808943174065276751L, phil_actuator_controls);
	phil_actuator_controls_flags_SET(-2749238012012327269L, -2534935274310406761L, -1185073395092944988L, -8190310518084865472L, -7044867048357598037L, 4020763505192109775L, -3143131097769354995L, 8982447556721087812L, -7424258760023428890L, -8202964543950196253L, phil_actuator_controls);
	phil_actuator_controls_controls_SET(&3.2468836E38F, 1.4986711E38F, -2.9924412E38F, 2.1053985E38F, 2.8739615E38F, -1.295086E38F, -1.0465671E38F, 6.212966E37F, -1.4348478E38F, -6.3321475E37F, phil_actuator_controls);
	phil_actuator_controls_mode_SET(e_MAV_MODE_STABILIZE_ARMED, e_MAV_MODE_MANUAL_DISARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, e_MAV_MODE_MANUAL_DISARMED, e_MAV_MODE_GUIDED_ARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, e_MAV_MODE_MAV_MODE_TEST_ARMED, e_MAV_MODE_MAV_MODE_TEST_DISARMED, e_MAV_MODE_PREFLIGHT, phil_actuator_controls);
	
}


void read_POSITION_TARGET_LOCAL_NED(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *const pposition_target_local_ned) {
	int16_t     type_mask    = pposition_target_local_ned_type_mask_GET(pposition_target_local_ned);
	int32_t     time_boot_ms = pposition_target_local_ned_time_boot_ms_GET(pposition_target_local_ned);
	float       x            = pposition_target_local_ned_x_GET(pposition_target_local_ned);
	float       y            = pposition_target_local_ned_y_GET(pposition_target_local_ned);
	float       z            = pposition_target_local_ned_z_GET(pposition_target_local_ned);
	float       vx           = pposition_target_local_ned_vx_GET(pposition_target_local_ned);
	float       vy           = pposition_target_local_ned_vy_GET(pposition_target_local_ned);
	float       vz           = pposition_target_local_ned_vz_GET(pposition_target_local_ned);
	float       afx          = pposition_target_local_ned_afx_GET(pposition_target_local_ned);
	float       afy          = pposition_target_local_ned_afy_GET(pposition_target_local_ned);
	float       afz          = pposition_target_local_ned_afz_GET(pposition_target_local_ned);
	float       yaw          = pposition_target_local_ned_yaw_GET(pposition_target_local_ned);
	float       yaw_rate     = pposition_target_local_ned_yaw_rate_GET(pposition_target_local_ned);
	e_MAV_FRAME item_coordinate_frame;
	if (pposition_target_local_ned_coordinate_frame_GET(pposition_target_local_ned, &item_coordinate_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_coordinate_frame;
	}
	
}


void write_POSITION_TARGET_LOCAL_NED(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *const pposition_target_local_ned) {
	pposition_target_local_ned_type_mask_SET(-18084, -24947, 6804, -7538, 12066, 20677, 23795, 1533, -26608, 2002, pposition_target_local_ned);
	pposition_target_local_ned_time_boot_ms_SET(-1003696798, 1462809348, -816941265, 531996087, 1620373012, -1559104420, 2030506430, -2108425268, 1116301389, -1529189618, pposition_target_local_ned);
	pposition_target_local_ned_x_SET(3.0661576E38F, 9.974601E37F, 5.242528E37F, -2.0488319E38F, -1.4965307E38F, -1.3058428E38F, -1.4898962E36F, 9.800716E36F, 1.2750346E38F, 1.5822098E38F, pposition_target_local_ned);
	pposition_target_local_ned_y_SET(-2.5850882E38F, -1.5210036E38F, 2.4626421E38F, -1.761308E38F, 4.309477E37F, -2.0973734E38F, 1.9192778E38F, 4.479374E37F, -2.134683E38F, -1.2865166E38F, pposition_target_local_ned);
	pposition_target_local_ned_z_SET(2.3795126E38F, 2.665894E38F, 2.8799592E38F, -8.1466195E37F, 2.8730715E38F, 1.3195928E38F, -2.0051354E38F, 2.2479061E38F, 2.2249984E38F, 7.3161714E37F, pposition_target_local_ned);
	pposition_target_local_ned_vx_SET(5.04253E37F, 3.293823E38F, -1.9691745E38F, -3.0049063E38F, 8.371606E37F, -5.20288E37F, -7.7693164E36F, -5.542388E37F, -1.7393827E38F, 2.4698418E38F, pposition_target_local_ned);
	pposition_target_local_ned_vy_SET(-7.2170883E37F, -2.3621318E38F, 1.4036336E38F, -1.2083197E38F, 1.9673722E38F, -9.684471E37F, 2.7884123E38F, 7.738001E37F, 4.800934E37F, 2.8453426E38F, pposition_target_local_ned);
	pposition_target_local_ned_vz_SET(4.1594588E37F, -4.748264E37F, -1.4820969E38F, -4.4930065E37F, -1.9603813E38F, -1.1932536E38F, -3.241092E38F, -2.7006667E38F, -1.1335332E38F, -3.4011893E38F, pposition_target_local_ned);
	pposition_target_local_ned_afx_SET(-1.8693761E38F, 1.1946392E38F, 5.330358E37F, -2.5651074E38F, 4.00408E37F, 2.0302438E38F, -1.4446278E38F, 1.0396119E38F, -2.1340965E38F, 6.959263E36F, pposition_target_local_ned);
	pposition_target_local_ned_afy_SET(1.032951E38F, 2.3827314E38F, -1.3695882E38F, -3.248675E37F, 4.9756457E37F, -3.1311783E38F, 2.955347E38F, 1.9445318E38F, 2.7703453E38F, 6.409707E37F, pposition_target_local_ned);
	pposition_target_local_ned_afz_SET(-2.79734E37F, -3.451741E37F, -1.7820841E37F, -1.8464773E38F, -1.7341278E38F, 6.837377E37F, 3.3894012E38F, 8.2337207E37F, -1.6382116E38F, -7.9162934E37F, pposition_target_local_ned);
	pposition_target_local_ned_yaw_SET(3.1531182E38F, 3.5316117E37F, -2.8546133E38F, 9.886966E36F, -1.2618507E38F, -9.171172E37F, -2.0878017E38F, -3.2769074E38F, -1.8833665E38F, -2.8682252E38F, pposition_target_local_ned);
	pposition_target_local_ned_yaw_rate_SET(-2.9272502E38F, -5.5834974E37F, 1.92639E38F, 8.611421E37F, 8.629856E37F, -2.462306E38F, 3.1287913E38F, 1.2019369E38F, -6.053788E37F, 1.1323267E38F, pposition_target_local_ned);
	pposition_target_local_ned_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, pposition_target_local_ned);
	
}


void read_GIMBAL_REPORT(pgimbal_report_GIMBAL_REPORT *const pgimbal_report) {
	int8_t target_system    = pgimbal_report_target_system_GET(pgimbal_report);
	int8_t target_component = pgimbal_report_target_component_GET(pgimbal_report);
	float  delta_time       = pgimbal_report_delta_time_GET(pgimbal_report);
	float  delta_angle_x    = pgimbal_report_delta_angle_x_GET(pgimbal_report);
	float  delta_angle_y    = pgimbal_report_delta_angle_y_GET(pgimbal_report);
	float  delta_angle_z    = pgimbal_report_delta_angle_z_GET(pgimbal_report);
	float  delta_velocity_x = pgimbal_report_delta_velocity_x_GET(pgimbal_report);
	float  delta_velocity_y = pgimbal_report_delta_velocity_y_GET(pgimbal_report);
	float  delta_velocity_z = pgimbal_report_delta_velocity_z_GET(pgimbal_report);
	float  joint_roll       = pgimbal_report_joint_roll_GET(pgimbal_report);
	float  joint_el         = pgimbal_report_joint_el_GET(pgimbal_report);
	float  joint_az         = pgimbal_report_joint_az_GET(pgimbal_report);
	
}


void read_DEVICE_OP_WRITE(pdevice_op_write_DEVICE_OP_WRITE *const pdevice_op_write) {
	int32_t               request_id       = pdevice_op_write_request_id_GET(pdevice_op_write);
	int8_t                target_system    = pdevice_op_write_target_system_GET(pdevice_op_write);
	int8_t                target_component = pdevice_op_write_target_component_GET(pdevice_op_write);
	int8_t                bus              = pdevice_op_write_bus_GET(pdevice_op_write);
	int8_t                address          = pdevice_op_write_address_GET(pdevice_op_write);
	int8_t                regstart         = pdevice_op_write_regstart_GET(pdevice_op_write);
	int8_t                count            = pdevice_op_write_count_GET(pdevice_op_write);
	Vdevice_op_write_daTa item_daTa        = pdevice_op_write_daTa_GET(pdevice_op_write);
	for (size_t           index            = 0; index < item_daTa.len; index++)
		some_int8_t = vdevice_op_write_daTa_GET(&item_daTa, index);
	e_DEVICE_OP_BUSTYPE   item_bustype;
	if (pdevice_op_write_bustype_GET(pdevice_op_write, &item_bustype)) {
		e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C = item_bustype;
	}
	Vdevice_op_write_busname item_busname;
	if (pdevice_op_write_busname_GET(pdevice_op_write, &item_busname)) {
		memcpy(some_string, item_busname.bytes, item_busname.len);
	}
	
}


void read_DISTANCE_SENSOR(pdistance_sensor_DISTANCE_SENSOR *const pdistance_sensor) {
	int16_t               min_distance     = pdistance_sensor_min_distance_GET(pdistance_sensor);
	int16_t               max_distance     = pdistance_sensor_max_distance_GET(pdistance_sensor);
	int16_t               current_distance = pdistance_sensor_current_distance_GET(pdistance_sensor);
	int32_t               time_boot_ms     = pdistance_sensor_time_boot_ms_GET(pdistance_sensor);
	int8_t                id               = pdistance_sensor_id_GET(pdistance_sensor);
	int8_t                covariance       = pdistance_sensor_covariance_GET(pdistance_sensor);
	e_MAV_DISTANCE_SENSOR item_typE;
	if (pdistance_sensor_typE_GET(pdistance_sensor, &item_typE)) {
		e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER = item_typE;
	}
	e_MAV_SENSOR_ORIENTATION item_orientation;
	if (pdistance_sensor_orientation_GET(pdistance_sensor, &item_orientation)) {
		e_MAV_SENSOR_ORIENTATION_NONE = item_orientation;
	}
	
}


void write_DISTANCE_SENSOR(pdistance_sensor_DISTANCE_SENSOR *const pdistance_sensor) {
	pdistance_sensor_min_distance_SET(15268, 30610, -32539, 26221, -4496, 14226, 28285, 23379, 8800, 21190, pdistance_sensor);
	pdistance_sensor_max_distance_SET(-14503, -8797, -25853, -27112, 32178, -24339, -6813, 6833, 3522, -17816, pdistance_sensor);
	pdistance_sensor_current_distance_SET(-19245, -19996, 19604, -19706, -15496, 15063, 32414, -16732, 15546, 13327, pdistance_sensor);
	pdistance_sensor_time_boot_ms_SET(703362110, 716491744, -573465635, 1233166905, 233483543, -673441275, -1272436146, -1214200236, 151558457, -1590341175, pdistance_sensor);
	pdistance_sensor_id_SET(-45, -74, 21, -17, 11, -31, -37, 103, -48, -55, pdistance_sensor);
	pdistance_sensor_covariance_SET(81, -106, 120, -24, -60, -5, 34, 60, -43, 53, pdistance_sensor);
	pdistance_sensor_typE_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, pdistance_sensor);
	pdistance_sensor_orientation_SET(e_MAV_SENSOR_ORIENTATION_ROLL_315_PITCH_315_YAW_315, e_MAV_SENSOR_ORIENTATION_YAW_90, e_MAV_SENSOR_ORIENTATION_ROLL_270_YAW_45, e_MAV_SENSOR_ORIENTATION_PITCH_180, e_MAV_SENSOR_ORIENTATION_ROLL_90_YAW_135, e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_315, e_MAV_SENSOR_ORIENTATION_ROLL_90_PITCH_180, e_MAV_SENSOR_ORIENTATION_YAW_270, e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_270, e_MAV_SENSOR_ORIENTATION_ROLL_315_PITCH_315_YAW_315, pdistance_sensor);
	
}


void read_HIL_OPTICAL_FLOW(phil_optical_flow_HIL_OPTICAL_FLOW *const phil_optical_flow) {
	int32_t integration_time_us    = phil_optical_flow_integration_time_us_GET(phil_optical_flow);
	int32_t time_delta_distance_us = phil_optical_flow_time_delta_distance_us_GET(phil_optical_flow);
	int64_t time_usec              = phil_optical_flow_time_usec_GET(phil_optical_flow);
	int8_t  sensor_id              = phil_optical_flow_sensor_id_GET(phil_optical_flow);
	float   integrated_x           = phil_optical_flow_integrated_x_GET(phil_optical_flow);
	float   integrated_y           = phil_optical_flow_integrated_y_GET(phil_optical_flow);
	float   integrated_xgyro       = phil_optical_flow_integrated_xgyro_GET(phil_optical_flow);
	float   integrated_ygyro       = phil_optical_flow_integrated_ygyro_GET(phil_optical_flow);
	float   integrated_zgyro       = phil_optical_flow_integrated_zgyro_GET(phil_optical_flow);
	int16_t temperature            = phil_optical_flow_temperature_GET(phil_optical_flow);
	int8_t  quality                = phil_optical_flow_quality_GET(phil_optical_flow);
	float   distance               = phil_optical_flow_distance_GET(phil_optical_flow);
	
}


void write_HIL_OPTICAL_FLOW(phil_optical_flow_HIL_OPTICAL_FLOW *const phil_optical_flow) {
	phil_optical_flow_integration_time_us_SET(-393490455, 648843687, -215354386, 943234853, -155886948, -756956550, 1194403777, -218179777, -300676371, -1236504239, phil_optical_flow);
	phil_optical_flow_time_delta_distance_us_SET(-76557265, -1318048133, 1384330708, -759434073, 199902935, -480529465, -1334528503, -2037684194, 67458815, -494911547, phil_optical_flow);
	phil_optical_flow_time_usec_SET(-6943027038152327551L, 6947148894666556879L, 4361779903139008100L, -8510330753216127442L, 6470498180638370996L, -571525739963037211L, -3775780871229558941L, -3270172873415852311L, -7502034602978490046L, 1911952974887582898L, phil_optical_flow);
	phil_optical_flow_sensor_id_SET(-82, -62, 95, 116, 69, 96, 98, 107, 4, -85, phil_optical_flow);
	phil_optical_flow_integrated_x_SET(-3.0183661E38F, 2.2027618E38F, -7.2525967E37F, -3.3692837E38F, -3.114184E38F, 3.3337059E38F, 5.205111E36F, 1.5057752E38F, -6.282092E37F, -8.623678E37F, phil_optical_flow);
	phil_optical_flow_integrated_y_SET(-8.018262E36F, -4.1134567E37F, 9.558126E37F, -7.6667736E37F, -1.0951501E38F, 3.1566362E38F, -2.7948106E38F, -2.3954373E37F, 3.3201286E38F, 1.6628656E38F, phil_optical_flow);
	phil_optical_flow_integrated_xgyro_SET(3.2842663E38F, -2.6080662E38F, -1.9565376E38F, 1.2291722E38F, 2.8274555E38F, 1.0271591E38F, 2.8066142E37F, 1.6778901E38F, 6.4303174E37F, 1.6604194E38F, phil_optical_flow);
	phil_optical_flow_integrated_ygyro_SET(-1.2337847E38F, -2.643924E38F, -9.812466E37F, 3.1699358E38F, -3.216069E38F, -1.7389438E38F, -3.1215732E38F, -1.0264478E38F, -2.265716E38F, 3.3532926E38F, phil_optical_flow);
	phil_optical_flow_integrated_zgyro_SET(1.7359439E38F, 8.4504145E37F, 3.0581136E38F, -8.2215554E37F, 3.3469059E38F, -2.649134E38F, 2.3802432E38F, 2.719135E38F, 1.791093E38F, 6.890748E36F, phil_optical_flow);
	phil_optical_flow_temperature_SET(8657, 30989, 32697, 16083, -4304, -5316, 7643, 22848, -7976, -15598, phil_optical_flow);
	phil_optical_flow_quality_SET(123, -33, -22, -88, -51, -5, 112, -46, -83, 60, phil_optical_flow);
	phil_optical_flow_distance_SET(-9.083281E37F, 2.1971646E38F, 1.3560171E38F, 2.1746889E38F, -1.3399253E38F, 2.4900466E37F, -2.3236117E38F, 1.489561E37F, -3.1111488E38F, -2.5124443E38F, phil_optical_flow);
	
}


void read_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2 *const pscaled_pressure2) {
	int32_t time_boot_ms = pscaled_pressure2_time_boot_ms_GET(pscaled_pressure2);
	float   press_abs    = pscaled_pressure2_press_abs_GET(pscaled_pressure2);
	float   press_diff   = pscaled_pressure2_press_diff_GET(pscaled_pressure2);
	int16_t temperature  = pscaled_pressure2_temperature_GET(pscaled_pressure2);
	
}


void write_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2 *const pscaled_pressure2) {
	pscaled_pressure2_time_boot_ms_SET(1402740523, 1280216106, -737915396, 1574922766, -1364970319, -1110695664, -1238792927, -2033390900, -33227788, 129135567, pscaled_pressure2);
	pscaled_pressure2_press_abs_SET(9.50804E37F, -1.6136997E38F, 1.5319696E38F, 3.3444079E38F, -2.6183062E38F, 1.1596061E38F, 2.3174152E38F, -3.2810438E38F, 1.6405781E37F, -2.22727E38F, pscaled_pressure2);
	pscaled_pressure2_press_diff_SET(1.980586E38F, -1.9431818E38F, -2.3035838E38F, -2.9595123E38F, -1.0837997E38F, -1.556062E38F, -2.7889865E38F, 1.9385256E38F, 1.3607773E38F, 1.7557166E38F, pscaled_pressure2);
	pscaled_pressure2_temperature_SET(13717, 5398, -23347, -12820, -27153, -848, 1713, -8726, 32665, 7503, pscaled_pressure2);
	
}


void read_WIND_COV(pwind_cov_WIND_COV *const pwind_cov) {
	int64_t time_usec      = pwind_cov_time_usec_GET(pwind_cov);
	float   wind_x         = pwind_cov_wind_x_GET(pwind_cov);
	float   wind_y         = pwind_cov_wind_y_GET(pwind_cov);
	float   wind_z         = pwind_cov_wind_z_GET(pwind_cov);
	float   var_horiz      = pwind_cov_var_horiz_GET(pwind_cov);
	float   var_vert       = pwind_cov_var_vert_GET(pwind_cov);
	float   wind_alt       = pwind_cov_wind_alt_GET(pwind_cov);
	float   horiz_accuracy = pwind_cov_horiz_accuracy_GET(pwind_cov);
	float   vert_accuracy  = pwind_cov_vert_accuracy_GET(pwind_cov);
	
}


void write_WIND_COV(pwind_cov_WIND_COV *const pwind_cov) {
	pwind_cov_time_usec_SET(5635355666423183415L, -8362049011433379951L, 5020070609817459115L, 6614828686174392667L, 4967189069327409288L, -369991950524907783L, -7189288430000085475L, 8892171685899459102L, -3440102617094890378L, -2981691085635932541L, pwind_cov);
	pwind_cov_wind_x_SET(2.9856407E38F, -2.3344918E38F, -1.4694133E38F, -3.0390874E37F, -1.8854529E37F, 8.4598296E37F, -2.6472682E38F, -1.3088233E38F, 1.3949359E37F, -7.545366E37F, pwind_cov);
	pwind_cov_wind_y_SET(-3.2707963E38F, -1.3516306E38F, -3.2425823E38F, 1.7121508E38F, 7.5450665E37F, -8.0563374E37F, -1.9898865E38F, -1.3549184E38F, -3.1486204E38F, 2.8377306E38F, pwind_cov);
	pwind_cov_wind_z_SET(-8.768528E37F, 2.9014453E37F, -1.7480845E38F, 1.6545203E38F, -2.9905366E38F, -1.1925744E38F, 2.788786E38F, 3.292915E38F, 3.3304562E38F, -4.6513787E37F, pwind_cov);
	pwind_cov_var_horiz_SET(2.7035831E38F, -2.094568E38F, 1.4012251E38F, -1.5441986E38F, -1.4051059E38F, 3.2662413E38F, 3.2007926E38F, 2.289339E38F, -8.767173E37F, -3.1104422E38F, pwind_cov);
	pwind_cov_var_vert_SET(-1.2680234E38F, 6.499466E37F, 1.3346669E38F, -9.725856E37F, 2.2347583E38F, 2.5768203E37F, -3.3244745E37F, -1.1841855E38F, -1.1350052E38F, -2.96777E38F, pwind_cov);
	pwind_cov_wind_alt_SET(8.977901E37F, 2.6181662E38F, -1.0068694E38F, 4.4307015E37F, 1.551117E38F, 2.4704423E38F, -2.0692414E38F, 2.4962154E38F, -2.297759E37F, 2.6958797E38F, pwind_cov);
	pwind_cov_horiz_accuracy_SET(7.230338E37F, -3.3371926E38F, 2.5260512E37F, 1.624928E38F, -2.7882886E38F, 2.24573E38F, 1.4984124E37F, 2.294351E38F, -1.6829819E38F, 2.1292524E38F, pwind_cov);
	pwind_cov_vert_accuracy_SET(-1.8980865E38F, -1.1815602E38F, -2.3800065E38F, 1.1156365E38F, 6.998395E37F, 1.3794187E38F, 4.4615815E37F, -8.2709633E37F, -5.668352E37F, 2.7231956E38F, pwind_cov);
	
}


void read_CHANGE_OPERATOR_CONTROL(pchange_operator_control_CHANGE_OPERATOR_CONTROL *const pchange_operator_control) {
	int8_t                           target_system   = pchange_operator_control_target_system_GET(pchange_operator_control);
	int8_t                           control_request = pchange_operator_control_control_request_GET(pchange_operator_control);
	int8_t                           version         = pchange_operator_control_version_GET(pchange_operator_control);
	Vchange_operator_control_passkey item_passkey;
	if (pchange_operator_control_passkey_GET(pchange_operator_control, &item_passkey)) {
		memcpy(some_string, item_passkey.bytes, item_passkey.len);
	}
	
}


void write_CHANGE_OPERATOR_CONTROL(pchange_operator_control_CHANGE_OPERATOR_CONTROL *const pchange_operator_control) {
	pchange_operator_control_target_system_SET(-116, -87, 29, 101, -23, 54, 54, -86, 56, 24, pchange_operator_control);
	pchange_operator_control_control_request_SET(-90, 38, 17, 62, -95, -19, -49, 52, 125, 96, pchange_operator_control);
	pchange_operator_control_version_SET(-110, 29, -121, 38, -97, -49, 20, 53, -21, -70, pchange_operator_control);
	pchange_operator_control_passkey_SET(some_string, strlen(some_string), pchange_operator_control);
	
}


void read_GOPRO_SET_REQUEST(pgopro_set_request_GOPRO_SET_REQUEST *const pgopro_set_request) {
	int8_t                   target_system    = pgopro_set_request_target_system_GET(pgopro_set_request);
	int8_t                   target_component = pgopro_set_request_target_component_GET(pgopro_set_request);
	Vgopro_set_request_value item_value       = pgopro_set_request_value_GET(pgopro_set_request);
	for (size_t              index            = 0; index < item_value.len; index++)
		some_int8_t = vgopro_set_request_value_GET(&item_value, index);
	e_GOPRO_COMMAND          item_cmd_id;
	if (pgopro_set_request_cmd_id_GET(pgopro_set_request, &item_cmd_id)) {
		e_GOPRO_COMMAND_GOPRO_COMMAND_POWER = item_cmd_id;
	}
	
}


void read_SYS_STATUS(psys_status_SYS_STATUS *const psys_status) {
	int16_t                 load              = psys_status_load_GET(psys_status);
	int16_t                 voltage_battery   = psys_status_voltage_battery_GET(psys_status);
	int16_t                 drop_rate_comm    = psys_status_drop_rate_comm_GET(psys_status);
	int16_t                 errors_comm       = psys_status_errors_comm_GET(psys_status);
	int16_t                 errors_count1     = psys_status_errors_count1_GET(psys_status);
	int16_t                 errors_count2     = psys_status_errors_count2_GET(psys_status);
	int16_t                 errors_count3     = psys_status_errors_count3_GET(psys_status);
	int16_t                 errors_count4     = psys_status_errors_count4_GET(psys_status);
	int16_t                 current_battery   = psys_status_current_battery_GET(psys_status);
	int8_t                  battery_remaining = psys_status_battery_remaining_GET(psys_status);
	e_MAV_SYS_STATUS_SENSOR item_onboard_control_sensors_present;
	if (psys_status_onboard_control_sensors_present_GET(psys_status, &item_onboard_control_sensors_present)) {
		e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO = item_onboard_control_sensors_present;
	}
	e_MAV_SYS_STATUS_SENSOR item_onboard_control_sensors_enabled;
	if (psys_status_onboard_control_sensors_enabled_GET(psys_status, &item_onboard_control_sensors_enabled)) {
		e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO = item_onboard_control_sensors_enabled;
	}
	e_MAV_SYS_STATUS_SENSOR item_onboard_control_sensors_health;
	if (psys_status_onboard_control_sensors_health_GET(psys_status, &item_onboard_control_sensors_health)) {
		e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO = item_onboard_control_sensors_health;
	}
	
}


void write_SYS_STATUS(psys_status_SYS_STATUS *const psys_status) {
	psys_status_load_SET(-12936, 229, 27592, -19582, -17750, 11999, 18680, 13808, -12395, 25850, psys_status);
	psys_status_voltage_battery_SET(-8872, -10224, -8110, 19372, -15362, 10891, 31389, -1517, -29641, -29571, psys_status);
	psys_status_drop_rate_comm_SET(-29223, 26303, 32154, -2030, -2746, -11723, -11558, 14623, 18157, 8204, psys_status);
	psys_status_errors_comm_SET(-15990, 4946, 5006, 3906, -28073, -2897, -2173, -11267, -18436, -31368, psys_status);
	psys_status_errors_count1_SET(4403, -6666, 23387, -25762, -28444, -7825, 27329, -17129, 17910, 6156, psys_status);
	psys_status_errors_count2_SET(12770, 4533, -32023, 16095, 5319, -24585, -28871, -20069, 13311, -21079, psys_status);
	psys_status_errors_count3_SET(19240, -30113, -16887, -18293, -18699, -14134, 25989, -25671, 22825, -2114, psys_status);
	psys_status_errors_count4_SET(-25711, -30632, 13799, 12371, -27126, 13692, -8555, 1742, 31219, -7310, psys_status);
	psys_status_current_battery_SET(24182, -7345, 5014, 21425, 16007, -14686, -16974, -16876, -20730, -15620, psys_status);
	psys_status_battery_remaining_SET(-87, -56, 31, -66, -107, 85, 62, -12, -52, -120, psys_status);
	psys_status_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH, psys_status);
	psys_status_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS, psys_status);
	psys_status_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY, e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER, psys_status);
	
}


void read_MISSION_ITEM(pmission_item_MISSION_ITEM *const pmission_item) {
	int16_t     seq              = pmission_item_seq_GET(pmission_item);
	int8_t      target_system    = pmission_item_target_system_GET(pmission_item);
	int8_t      target_component = pmission_item_target_component_GET(pmission_item);
	int8_t      current          = pmission_item_current_GET(pmission_item);
	int8_t      autocontinue     = pmission_item_autocontinue_GET(pmission_item);
	float       param1           = pmission_item_param1_GET(pmission_item);
	float       param2           = pmission_item_param2_GET(pmission_item);
	float       param3           = pmission_item_param3_GET(pmission_item);
	float       param4           = pmission_item_param4_GET(pmission_item);
	float       x                = pmission_item_x_GET(pmission_item);
	float       y                = pmission_item_y_GET(pmission_item);
	float       z                = pmission_item_z_GET(pmission_item);
	e_MAV_FRAME item_frame;
	if (pmission_item_frame_GET(pmission_item, &item_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_frame;
	}
	e_MAV_CMD item_command;
	if (pmission_item_command_GET(pmission_item, &item_command)) {
		e_MAV_CMD_MAV_CMD_NAV_WAYPOINT = item_command;
	}
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_item_mission_type_GET(pmission_item, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_ITEM(pmission_item_MISSION_ITEM *const pmission_item) {
	pmission_item_seq_SET(-29171, -8302, 19127, -1569, -19629, -25444, 13857, 14459, 25500, 14284, pmission_item);
	pmission_item_target_system_SET(-105, 24, 30, 51, -52, -106, -71, 46, 124, -49, pmission_item);
	pmission_item_target_component_SET(-92, 90, -31, -67, -92, 33, 67, -11, -44, -66, pmission_item);
	pmission_item_current_SET(22, -63, -20, 83, -58, 34, 49, -116, -91, -103, pmission_item);
	pmission_item_autocontinue_SET(-96, 33, -70, -112, 127, 4, 71, 85, 115, -57, pmission_item);
	pmission_item_param1_SET(-9.3931E36F, -2.8645764E38F, 2.2595067E38F, -2.343323E38F, 2.708616E38F, 2.9745533E38F, -1.0268724E38F, 2.8028775E38F, -1.6858412E38F, -1.024556E38F, pmission_item);
	pmission_item_param2_SET(-1.0648574E38F, 1.1593796E38F, 3.3650546E38F, 1.3856925E38F, -1.8651406E38F, -2.4171403E38F, 3.0437966E38F, 1.7008966E38F, 1.02519556E37F, -4.530413E37F, pmission_item);
	pmission_item_param3_SET(2.1219998E37F, 1.938931E38F, 3.2811832E38F, 1.5073875E38F, -1.3449647E37F, -2.9568163E38F, -1.0678043E38F, -3.3595205E37F, 4.2455954E37F, -4.753984E37F, pmission_item);
	pmission_item_param4_SET(-1.1174093E38F, -2.9420457E38F, -2.1754915E38F, -1.3501311E38F, 2.4285163E38F, -2.9768213E38F, 2.804685E38F, 2.2096495E38F, -1.3359321E38F, -1.3483662E38F, pmission_item);
	pmission_item_x_SET(6.255658E37F, 2.7813558E38F, 2.7825707E38F, 1.9036258E38F, 3.3838691E38F, -1.5180054E38F, -1.8605346E38F, 1.871037E38F, -1.2800464E38F, -2.548945E38F, pmission_item);
	pmission_item_y_SET(3.5920556E37F, -5.607859E37F, -1.6593234E36F, -1.6466651E38F, -5.3145947E37F, -3.2373778E38F, 2.9307368E38F, 2.1879341E38F, 2.224866E38F, -1.3047009E38F, pmission_item);
	pmission_item_z_SET(2.2756032E38F, 3.0250912E38F, 1.4268384E38F, 9.321484E37F, -2.5031303E38F, -3.0359114E38F, -4.8163272E36F, 3.2760596E38F, 1.4755502E38F, 1.0272802E38F, pmission_item);
	pmission_item_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, e_MAV_FRAME_MAV_FRAME_LOCAL_NED, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, pmission_item);
	pmission_item_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_RELAY, e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST, e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL, e_MAV_CMD_MAV_CMD_NAV_LAND, e_MAV_CMD_MAV_CMD_NAV_WAYPOINT, e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION, e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED, e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME, e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION, e_MAV_CMD_MAV_CMD_DO_LAST, pmission_item);
	pmission_item_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, pmission_item);
	
}


void read_RAW_IMU(praw_imu_RAW_IMU *const praw_imu) {
	int64_t time_usec = praw_imu_time_usec_GET(praw_imu);
	int16_t xacc      = praw_imu_xacc_GET(praw_imu);
	int16_t yacc      = praw_imu_yacc_GET(praw_imu);
	int16_t zacc      = praw_imu_zacc_GET(praw_imu);
	int16_t xgyro     = praw_imu_xgyro_GET(praw_imu);
	int16_t ygyro     = praw_imu_ygyro_GET(praw_imu);
	int16_t zgyro     = praw_imu_zgyro_GET(praw_imu);
	int16_t xmag      = praw_imu_xmag_GET(praw_imu);
	int16_t ymag      = praw_imu_ymag_GET(praw_imu);
	int16_t zmag      = praw_imu_zmag_GET(praw_imu);
	
}


void write_RAW_IMU(praw_imu_RAW_IMU *const praw_imu) {
	praw_imu_time_usec_SET(3201304577353554943L, 7139508918556904926L, 895252084737692687L, 5775513152465134434L, -8967444170169552167L, -7522466877983697270L, 6278927591296245820L, 314304336981594133L, 1012006634960835469L, -8726656344684440470L, praw_imu);
	praw_imu_xacc_SET(-11062, -24446, -22947, 2021, 17563, 11476, 4700, 25439, 1822, -31097, praw_imu);
	praw_imu_yacc_SET(5078, -11464, 23403, 28541, 32613, 12515, -9346, -4997, 10505, -32497, praw_imu);
	praw_imu_zacc_SET(17263, 26844, -16462, -1417, 21382, -2584, 7045, 31351, -31674, 21622, praw_imu);
	praw_imu_xgyro_SET(-24747, 24134, -24453, -11771, 28755, -6795, 32720, 5612, 7921, -14355, praw_imu);
	praw_imu_ygyro_SET(25579, 4911, -7379, -18286, -16222, -19815, 15074, -29478, -28928, 32323, praw_imu);
	praw_imu_zgyro_SET(22381, -3573, 15506, 7031, -5047, 12430, -29678, -1136, -32586, -30284, praw_imu);
	praw_imu_xmag_SET(26048, 30551, 11493, -2692, -28243, 9772, 26735, 18915, -1034, -18513, praw_imu);
	praw_imu_ymag_SET(14708, 20628, -7795, 31308, 16822, -6378, 17913, 29500, -560, 30030, praw_imu);
	praw_imu_zmag_SET(27735, 31425, -3766, 1476, 23746, 5392, 10729, 22058, 7814, -25433, praw_imu);
	
}


void read_COMMAND_INT(pcommand_int_COMMAND_INT *const pcommand_int) {
	int8_t      target_system    = pcommand_int_target_system_GET(pcommand_int);
	int8_t      target_component = pcommand_int_target_component_GET(pcommand_int);
	int8_t      current          = pcommand_int_current_GET(pcommand_int);
	int8_t      autocontinue     = pcommand_int_autocontinue_GET(pcommand_int);
	float       param1           = pcommand_int_param1_GET(pcommand_int);
	float       param2           = pcommand_int_param2_GET(pcommand_int);
	float       param3           = pcommand_int_param3_GET(pcommand_int);
	float       param4           = pcommand_int_param4_GET(pcommand_int);
	int32_t     x                = pcommand_int_x_GET(pcommand_int);
	int32_t     y                = pcommand_int_y_GET(pcommand_int);
	float       z                = pcommand_int_z_GET(pcommand_int);
	e_MAV_FRAME item_frame;
	if (pcommand_int_frame_GET(pcommand_int, &item_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_frame;
	}
	e_MAV_CMD item_command;
	if (pcommand_int_command_GET(pcommand_int, &item_command)) {
		e_MAV_CMD_MAV_CMD_NAV_WAYPOINT = item_command;
	}
	
}


void write_COMMAND_INT(pcommand_int_COMMAND_INT *const pcommand_int) {
	pcommand_int_target_system_SET(-21, -85, -33, 103, -99, -69, -47, 89, -74, -85, pcommand_int);
	pcommand_int_target_component_SET(-127, -33, -50, 123, -3, -80, -80, -90, 85, -92, pcommand_int);
	pcommand_int_current_SET(43, -28, 47, -46, -76, 5, 19, 29, 103, -61, pcommand_int);
	pcommand_int_autocontinue_SET(51, -18, 103, 38, -99, -69, 5, 46, 120, 125, pcommand_int);
	pcommand_int_param1_SET(2.1558984E38F, -1.6166784E38F, 5.576631E37F, -1.4310593E38F, -1.2564167E37F, -2.7247113E38F, 3.1178374E37F, 5.116181E37F, 2.4744503E38F, 1.9257258E38F, pcommand_int);
	pcommand_int_param2_SET(1.8362647E38F, 2.0821404E38F, -1.8698514E38F, 2.9377592E38F, -3.0646321E38F, 2.420667E38F, 1.399401E38F, -3.1612168E38F, 1.2392179E38F, 2.7533643E38F, pcommand_int);
	pcommand_int_param3_SET(3.0770062E38F, -3.2183464E38F, -1.0533014E38F, 3.3330363E37F, 5.228063E37F, 2.343126E38F, 1.0338675E38F, -2.3116146E37F, -2.8213235E38F, -2.6241242E38F, pcommand_int);
	pcommand_int_param4_SET(-1.610352E37F, -2.0286778E38F, 2.3641144E38F, 2.1501332E38F, -2.83506E38F, 1.7824766E38F, 3.1894683E38F, 2.7518583E38F, 3.411643E37F, 1.6682259E38F, pcommand_int);
	pcommand_int_x_SET(-1804995281, 2039578433, -790556171, 1751146246, 1581803644, -348219339, -1343310676, 1403588915, 565582944, -1412326513, pcommand_int);
	pcommand_int_y_SET(2006535098, -1277246473, 126013234, 1751862260, 281672578, -1187035626, 2118154644, 1456997389, 1793558130, 2134805832, pcommand_int);
	pcommand_int_z_SET(-1.9719086E37F, 4.8259925E37F, -1.210257E38F, -1.9981719E38F, 2.44216E38F, 1.6254126E38F, 1.6610706E38F, 1.9207945E38F, -3.1183905E38F, 1.5673543E38F, pcommand_int);
	pcommand_int_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_LOCAL_NED, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL, pcommand_int);
	pcommand_int_command_SET(e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD, e_MAV_CMD_MAV_CMD_DO_JUMP, e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT, e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL, e_MAV_CMD_MAV_CMD_DO_JUMP, e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING, e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION, e_MAV_CMD_MAV_CMD_DO_SET_ROI, e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER, e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT, pcommand_int);
	
}


void read_OPTICAL_FLOW(poptical_flow_OPTICAL_FLOW *const poptical_flow) {
	int64_t time_usec       = poptical_flow_time_usec_GET(poptical_flow);
	int8_t  sensor_id       = poptical_flow_sensor_id_GET(poptical_flow);
	int16_t flow_x          = poptical_flow_flow_x_GET(poptical_flow);
	int16_t flow_y          = poptical_flow_flow_y_GET(poptical_flow);
	float   flow_comp_m_x   = poptical_flow_flow_comp_m_x_GET(poptical_flow);
	float   flow_comp_m_y   = poptical_flow_flow_comp_m_y_GET(poptical_flow);
	int8_t  quality         = poptical_flow_quality_GET(poptical_flow);
	float   ground_distance = poptical_flow_ground_distance_GET(poptical_flow);
	float   item_flow_rate_x;
	if (poptical_flow_flow_rate_x_GET(poptical_flow, &item_flow_rate_x)) {
		some_float = item_flow_rate_x;
	}
	float item_flow_rate_y;
	if (poptical_flow_flow_rate_y_GET(poptical_flow, &item_flow_rate_y)) {
		some_float = item_flow_rate_y;
	}
	
}


void write_OPTICAL_FLOW(poptical_flow_OPTICAL_FLOW *const poptical_flow) {
	poptical_flow_time_usec_SET(-3327822636951762346L, 5708490604614761688L, 407861330215985891L, -2729102141741948848L, -7312727125365034532L, -5381146198285234602L, -3576482463160944843L, -4033338236517499983L, 3690266267572979267L, 3327540500810383767L, poptical_flow);
	poptical_flow_sensor_id_SET(56, 99, -67, -11, -39, 62, 62, 68, 101, -128, poptical_flow);
	poptical_flow_flow_x_SET(21731, -19332, 12325, 31473, 12906, 21837, 2853, -6574, -12015, 11498, poptical_flow);
	poptical_flow_flow_y_SET(5490, -16454, 26470, 18333, -6181, -11820, 31276, -29777, -19188, 5677, poptical_flow);
	poptical_flow_flow_comp_m_x_SET(1.3286841E38F, 3.8667537E37F, -3.2179736E38F, -3.1914875E38F, -2.3656514E38F, 1.3386472E38F, 2.430079E38F, -1.9294163E38F, -4.0589434E37F, 1.666721E38F, poptical_flow);
	poptical_flow_flow_comp_m_y_SET(-2.5991099E38F, -2.6941713E38F, -1.6737529E38F, 1.6121433E38F, 2.2716317E38F, 6.181472E37F, 9.689933E37F, -1.0497299E38F, 3.21302E38F, 2.474189E37F, poptical_flow);
	poptical_flow_quality_SET(-67, -34, -112, -39, 82, -121, -76, -72, -39, -116, poptical_flow);
	poptical_flow_ground_distance_SET(-7.8081476E37F, -2.8756244E38F, 1.7913252E38F, -2.3981282E38F, -1.9211016E38F, -2.4990813E38F, 2.9850687E38F, 2.4098738E38F, -1.8668116E38F, -5.494539E37F, poptical_flow);
	poptical_flow_flow_rate_x_SET(-2.488118E38F, -3.9649284E37F, 1.827787E38F, -1.3293194E38F, 5.7878584E37F, -2.869588E38F, -2.844044E38F, 8.727393E37F, -2.4409353E38F, -3.3504748E38F, poptical_flow);
	poptical_flow_flow_rate_y_SET(1.371514E38F, -2.976738E38F, -1.4089539E38F, -2.3449135E38F, -2.7930385E38F, 2.737013E37F, -3.37054E38F, -2.5067255E38F, -1.4468695E38F, -2.8167434E38F, poptical_flow);
	
}


void read_MISSION_ITEM_INT(pmission_item_int_MISSION_ITEM_INT *const pmission_item_int) {
	int16_t     seq              = pmission_item_int_seq_GET(pmission_item_int);
	int8_t      target_system    = pmission_item_int_target_system_GET(pmission_item_int);
	int8_t      target_component = pmission_item_int_target_component_GET(pmission_item_int);
	int8_t      current          = pmission_item_int_current_GET(pmission_item_int);
	int8_t      autocontinue     = pmission_item_int_autocontinue_GET(pmission_item_int);
	float       param1           = pmission_item_int_param1_GET(pmission_item_int);
	float       param2           = pmission_item_int_param2_GET(pmission_item_int);
	float       param3           = pmission_item_int_param3_GET(pmission_item_int);
	float       param4           = pmission_item_int_param4_GET(pmission_item_int);
	int32_t     x                = pmission_item_int_x_GET(pmission_item_int);
	int32_t     y                = pmission_item_int_y_GET(pmission_item_int);
	float       z                = pmission_item_int_z_GET(pmission_item_int);
	e_MAV_FRAME item_frame;
	if (pmission_item_int_frame_GET(pmission_item_int, &item_frame)) {
		e_MAV_FRAME_MAV_FRAME_GLOBAL = item_frame;
	}
	e_MAV_CMD item_command;
	if (pmission_item_int_command_GET(pmission_item_int, &item_command)) {
		e_MAV_CMD_MAV_CMD_NAV_WAYPOINT = item_command;
	}
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_item_int_mission_type_GET(pmission_item_int, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
}


void write_MISSION_ITEM_INT(pmission_item_int_MISSION_ITEM_INT *const pmission_item_int) {
	pmission_item_int_seq_SET(-18037, 22597, 16694, 29425, 9768, -19688, -20112, -1291, 1992, 3306, pmission_item_int);
	pmission_item_int_target_system_SET(29, -3, 11, -109, -14, -108, -59, 72, -66, -62, pmission_item_int);
	pmission_item_int_target_component_SET(8, -95, -20, -122, 110, 22, 24, 34, 34, -83, pmission_item_int);
	pmission_item_int_current_SET(88, -4, -37, 65, 79, -98, -1, -74, -117, -1, pmission_item_int);
	pmission_item_int_autocontinue_SET(122, -114, 91, -91, -92, 100, 59, -117, 48, 19, pmission_item_int);
	pmission_item_int_param1_SET(-2.0485579E38F, -1.5748384E38F, -1.9856132E38F, 1.2472683E38F, -3.3579578E38F, -8.7464634E36F, 2.4518358E37F, 2.1758113E38F, -7.606201E37F, 1.4438629E38F, pmission_item_int);
	pmission_item_int_param2_SET(-6.4350036E37F, 1.0667225E38F, -4.5531317E37F, 1.4776258E38F, -1.1346609E38F, 2.1361744E38F, 4.211045E37F, -2.9007452E38F, -2.2852438E38F, 3.1881307E38F, pmission_item_int);
	pmission_item_int_param3_SET(-8.3998803E37F, -2.5008578E38F, 2.2628892E38F, -2.8545301E38F, -8.0578875E37F, -1.9021479E38F, -7.510463E37F, -1.2564946E38F, -2.256912E38F, 1.8607162E38F, pmission_item_int);
	pmission_item_int_param4_SET(-1.5223018E38F, -1.6868868E38F, 8.903404E37F, -4.425602E37F, -1.4433227E38F, -7.8754406E37F, -2.7868907E38F, -8.362727E37F, 2.5239159E38F, 2.1434493E38F, pmission_item_int);
	pmission_item_int_x_SET(-419591282, -736495530, -1273970549, 1722670885, 875970290, 823023208, 1710296541, -207915548, 620525898, -1998468772, pmission_item_int);
	pmission_item_int_y_SET(65727553, 753750798, 637611627, -2010978662, -1479724619, 1042040939, -1355099785, 1457339779, -541809409, 205846082, pmission_item_int);
	pmission_item_int_z_SET(-1.6781173E38F, 2.7561624E38F, 3.1171979E38F, 1.2796889E38F, 1.5692286E37F, 2.897785E38F, 2.9206927E38F, 1.2320842E38F, -3.3118824E38F, 6.956304E37F, pmission_item_int);
	pmission_item_int_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_LOCAL_NED, e_MAV_FRAME_MAV_FRAME_BODY_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, pmission_item_int);
	pmission_item_int_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING, e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED, e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE, e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL, e_MAV_CMD_MAV_CMD_DO_LAST, e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE, e_MAV_CMD_MAV_CMD_DO_SET_ROI, e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE, e_MAV_CMD_MAV_CMD_DO_SET_SERVO, e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE, pmission_item_int);
	pmission_item_int_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, pmission_item_int);
	
}


void read_VISION_POSITION_DELTA(pvision_position_delta_VISION_POSITION_DELTA *const pvision_position_delta) {
	int64_t                               time_usec        = pvision_position_delta_time_usec_GET(pvision_position_delta);
	int64_t                               time_delta_usec  = pvision_position_delta_time_delta_usec_GET(pvision_position_delta);
	Vvision_position_delta_angle_delta    item_angle_delta = pvision_position_delta_angle_delta_GET(pvision_position_delta);
	for (size_t                           index            = 0; index < item_angle_delta.len; index++)
		some_float                                            = vvision_position_delta_angle_delta_GET(&item_angle_delta, index);
	Vvision_position_delta_position_delta item_position_delta = pvision_position_delta_position_delta_GET(pvision_position_delta);
	for (size_t                           index               = 0; index < item_position_delta.len; index++)
		some_float                                   = vvision_position_delta_position_delta_GET(&item_position_delta, index);
	float                                 confidence = pvision_position_delta_confidence_GET(pvision_position_delta);
	
}


void read_LOGGING_DATA(plogging_data_LOGGING_DATA *const plogging_data) {
	int16_t            sequence             = plogging_data_sequence_GET(plogging_data);
	int8_t             target_system        = plogging_data_target_system_GET(plogging_data);
	int8_t             target_component     = plogging_data_target_component_GET(plogging_data);
	int8_t             length               = plogging_data_length_GET(plogging_data);
	int8_t             first_message_offset = plogging_data_first_message_offset_GET(plogging_data);
	Vlogging_data_daTa item_daTa            = plogging_data_daTa_GET(plogging_data);
	for (size_t        index                = 0; index < item_daTa.len; index++)
		some_int8_t = vlogging_data_daTa_GET(&item_daTa, index);
	
}


void read_DEVICE_OP_READ(pdevice_op_read_DEVICE_OP_READ *const pdevice_op_read) {
	int32_t             request_id       = pdevice_op_read_request_id_GET(pdevice_op_read);
	int8_t              target_system    = pdevice_op_read_target_system_GET(pdevice_op_read);
	int8_t              target_component = pdevice_op_read_target_component_GET(pdevice_op_read);
	int8_t              bus              = pdevice_op_read_bus_GET(pdevice_op_read);
	int8_t              address          = pdevice_op_read_address_GET(pdevice_op_read);
	int8_t              regstart         = pdevice_op_read_regstart_GET(pdevice_op_read);
	int8_t              count            = pdevice_op_read_count_GET(pdevice_op_read);
	e_DEVICE_OP_BUSTYPE item_bustype;
	if (pdevice_op_read_bustype_GET(pdevice_op_read, &item_bustype)) {
		e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C = item_bustype;
	}
	Vdevice_op_read_busname item_busname;
	if (pdevice_op_read_busname_GET(pdevice_op_read, &item_busname)) {
		memcpy(some_string, item_busname.bytes, item_busname.len);
	}
	
}


void read_MAG_CAL_PROGRESS(pmag_cal_progress_MAG_CAL_PROGRESS *const pmag_cal_progress) {
	int8_t                            compass_id           = pmag_cal_progress_compass_id_GET(pmag_cal_progress);
	int8_t                            cal_mask             = pmag_cal_progress_cal_mask_GET(pmag_cal_progress);
	int8_t                            attempt              = pmag_cal_progress_attempt_GET(pmag_cal_progress);
	int8_t                            completion_pct       = pmag_cal_progress_completion_pct_GET(pmag_cal_progress);
	Vmag_cal_progress_completion_mask item_completion_mask = pmag_cal_progress_completion_mask_GET(pmag_cal_progress);
	for (size_t                       index                = 0; index < item_completion_mask.len; index++)
		some_int8_t                               = vmag_cal_progress_completion_mask_GET(&item_completion_mask, index);
	float                             direction_x = pmag_cal_progress_direction_x_GET(pmag_cal_progress);
	float                             direction_y = pmag_cal_progress_direction_y_GET(pmag_cal_progress);
	float                             direction_z = pmag_cal_progress_direction_z_GET(pmag_cal_progress);
	e_MAG_CAL_STATUS                  item_cal_status;
	if (pmag_cal_progress_cal_status_GET(pmag_cal_progress, &item_cal_status)) {
		e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED = item_cal_status;
	}
	
}


void read_HIGHRES_IMU(phighres_imu_HIGHRES_IMU *const phighres_imu) {
	int16_t fields_updated = phighres_imu_fields_updated_GET(phighres_imu);
	int64_t time_usec      = phighres_imu_time_usec_GET(phighres_imu);
	float   xacc           = phighres_imu_xacc_GET(phighres_imu);
	float   yacc           = phighres_imu_yacc_GET(phighres_imu);
	float   zacc           = phighres_imu_zacc_GET(phighres_imu);
	float   xgyro          = phighres_imu_xgyro_GET(phighres_imu);
	float   ygyro          = phighres_imu_ygyro_GET(phighres_imu);
	float   zgyro          = phighres_imu_zgyro_GET(phighres_imu);
	float   xmag           = phighres_imu_xmag_GET(phighres_imu);
	float   ymag           = phighres_imu_ymag_GET(phighres_imu);
	float   zmag           = phighres_imu_zmag_GET(phighres_imu);
	float   abs_pressure   = phighres_imu_abs_pressure_GET(phighres_imu);
	float   diff_pressure  = phighres_imu_diff_pressure_GET(phighres_imu);
	float   pressure_alt   = phighres_imu_pressure_alt_GET(phighres_imu);
	float   temperature    = phighres_imu_temperature_GET(phighres_imu);
	
}


void write_HIGHRES_IMU(phighres_imu_HIGHRES_IMU *const phighres_imu) {
	phighres_imu_fields_updated_SET(18860, -15230, -10655, -23493, 23059, 28949, 20350, 7234, 27883, -19925, phighres_imu);
	phighres_imu_time_usec_SET(-5467232341431853543L, -1363886393329672750L, -7392941908703623010L, -5275288821299324299L, -4126293573013948118L, 2449065872127534271L, -1164905650205915204L, -1717623622579803134L, -1236836698222597081L, -4085455377367138729L, phighres_imu);
	phighres_imu_xacc_SET(-2.5527646E38F, -2.4996056E38F, -1.006616E38F, -6.7265293E37F, -2.4473295E37F, -7.8120096E36F, -1.3982675E38F, 1.7477752E38F, 6.498635E37F, -3.355335E38F, phighres_imu);
	phighres_imu_yacc_SET(-6.5399514E37F, 1.5566227E38F, -1.6840168E38F, -7.203477E37F, -2.8093906E38F, -1.7914294E38F, -6.0116616E37F, 1.3386666E38F, -1.9656434E38F, 2.5501268E38F, phighres_imu);
	phighres_imu_zacc_SET(2.0998513E38F, 3.3983427E38F, 1.6056783E38F, 5.302856E37F, -1.1495868E38F, -3.3543365E38F, -4.570792E37F, 1.135988E38F, 8.0427223E37F, 4.0641603E37F, phighres_imu);
	phighres_imu_xgyro_SET(-2.587675E38F, -2.3933629E38F, 1.0642541E38F, 5.6496733E37F, -3.0604413E36F, 6.3558865E37F, 4.609454E37F, 1.9703324E37F, -5.542611E37F, 1.3457539E38F, phighres_imu);
	phighres_imu_ygyro_SET(1.8896896E38F, -1.4414327E38F, -3.171162E38F, 1.2863703E38F, -1.1594729E38F, -3.1499846E38F, -2.28171E37F, -6.2408173E37F, -2.2877398E38F, -3.0509728E37F, phighres_imu);
	phighres_imu_zgyro_SET(2.8112028E38F, -2.0107095E38F, -2.0786868E38F, 2.4961186E38F, 1.6745448E38F, -2.2935085E38F, 1.1421933E38F, -2.3338084E38F, -1.866343E38F, 2.692364E38F, phighres_imu);
	phighres_imu_xmag_SET(-8.753675E37F, -3.0204611E38F, 1.6036744E38F, 1.7645526E38F, 1.1631893E38F, 7.2540905E37F, 2.4967431E38F, 5.9054786E37F, -1.054761E38F, 1.4673987E38F, phighres_imu);
	phighres_imu_ymag_SET(2.5346274E38F, 2.433294E38F, -1.997337E38F, -2.0043179E38F, 2.6422996E38F, -3.2169932E38F, -1.3874657E38F, 1.1354429E38F, 2.632057E38F, 9.60921E37F, phighres_imu);
	phighres_imu_zmag_SET(-1.371378E38F, -5.9177774E37F, 1.4625868E38F, -1.1704213E38F, 2.1571653E38F, -8.3181615E37F, 1.1912003E38F, -9.122106E37F, 7.939606E37F, -1.0423887E38F, phighres_imu);
	phighres_imu_abs_pressure_SET(2.0751266E38F, -1.3895976E38F, 1.8175787E38F, 4.857085E37F, -8.983035E37F, -1.7044877E38F, 2.807255E37F, -3.1042842E38F, -2.9572412E38F, 1.6711113E38F, phighres_imu);
	phighres_imu_diff_pressure_SET(1.8295243E38F, 9.612825E37F, -1.173488E38F, 8.57963E37F, 9.930803E36F, -1.4454148E38F, 2.9197908E38F, 4.2096679E37F, 2.9604083E38F, 8.2748596E37F, phighres_imu);
	phighres_imu_pressure_alt_SET(4.395614E37F, 2.149013E38F, -1.7837343E38F, -1.2460195E38F, -2.0996283E38F, 2.6524167E38F, 4.8039876E37F, -2.6940151E38F, 3.2274303E38F, 1.9681271E38F, phighres_imu);
	phighres_imu_temperature_SET(-4.1469328E37F, 3.9502046E37F, 4.56326E37F, 1.0318516E38F, 1.7029726E38F, -6.772741E37F, 1.7997353E38F, 5.676705E37F, -1.1432775E38F, 2.457273E37F, phighres_imu);
	
}


void read_EXTENDED_SYS_STATE(pextended_sys_state_EXTENDED_SYS_STATE *const pextended_sys_state) {
	e_MAV_VTOL_STATE item_vtol_state;
	if (pextended_sys_state_vtol_state_GET(pextended_sys_state, &item_vtol_state)) {
		e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED = item_vtol_state;
	}
	e_MAV_LANDED_STATE item_landed_state;
	if (pextended_sys_state_landed_state_GET(pextended_sys_state, &item_landed_state)) {
		e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED = item_landed_state;
	}
	
}


void write_EXTENDED_SYS_STATE(pextended_sys_state_EXTENDED_SYS_STATE *const pextended_sys_state) {
	pextended_sys_state_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, pextended_sys_state);
	pextended_sys_state_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, pextended_sys_state);
	
}


void read_UAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const puavionix_adsb_out_dynamic) {
	int16_t                             accuracyVert = puavionix_adsb_out_dynamic_accuracyVert_GET(puavionix_adsb_out_dynamic);
	int16_t                             accuracyVel  = puavionix_adsb_out_dynamic_accuracyVel_GET(puavionix_adsb_out_dynamic);
	int16_t                             squawk       = puavionix_adsb_out_dynamic_squawk_GET(puavionix_adsb_out_dynamic);
	int32_t                             utcTime      = puavionix_adsb_out_dynamic_utcTime_GET(puavionix_adsb_out_dynamic);
	int32_t                             accuracyHor  = puavionix_adsb_out_dynamic_accuracyHor_GET(puavionix_adsb_out_dynamic);
	int32_t                             gpsLat       = puavionix_adsb_out_dynamic_gpsLat_GET(puavionix_adsb_out_dynamic);
	int32_t                             gpsLon       = puavionix_adsb_out_dynamic_gpsLon_GET(puavionix_adsb_out_dynamic);
	int32_t                             gpsAlt       = puavionix_adsb_out_dynamic_gpsAlt_GET(puavionix_adsb_out_dynamic);
	int8_t                              numSats      = puavionix_adsb_out_dynamic_numSats_GET(puavionix_adsb_out_dynamic);
	int32_t                             baroAltMSL   = puavionix_adsb_out_dynamic_baroAltMSL_GET(puavionix_adsb_out_dynamic);
	int16_t                             velVert      = puavionix_adsb_out_dynamic_velVert_GET(puavionix_adsb_out_dynamic);
	int16_t                             velNS        = puavionix_adsb_out_dynamic_velNS_GET(puavionix_adsb_out_dynamic);
	int16_t                             VelEW        = puavionix_adsb_out_dynamic_VelEW_GET(puavionix_adsb_out_dynamic);
	e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX item_gpsFix;
	if (puavionix_adsb_out_dynamic_gpsFix_GET(puavionix_adsb_out_dynamic, &item_gpsFix)) {
		e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = item_gpsFix;
	}
	e_UAVIONIX_ADSB_EMERGENCY_STATUS item_emergencyStatus;
	if (puavionix_adsb_out_dynamic_emergencyStatus_GET(puavionix_adsb_out_dynamic, &item_emergencyStatus)) {
		e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_NO_EMERGENCY = item_emergencyStatus;
	}
	e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE item_state;
	if (puavionix_adsb_out_dynamic_state_GET(puavionix_adsb_out_dynamic, &item_state)) {
		e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE = item_state;
	}
	
}


void read_GOPRO_GET_RESPONSE(pgopro_get_response_GOPRO_GET_RESPONSE *const pgopro_get_response) {
	Vgopro_get_response_value item_value = pgopro_get_response_value_GET(pgopro_get_response);
	for (size_t               index      = 0; index < item_value.len; index++)
		some_int8_t = vgopro_get_response_value_GET(&item_value, index);
	e_GOPRO_COMMAND           item_cmd_id;
	if (pgopro_get_response_cmd_id_GET(pgopro_get_response, &item_cmd_id)) {
		e_GOPRO_COMMAND_GOPRO_COMMAND_POWER = item_cmd_id;
	}
	e_GOPRO_REQUEST_STATUS item_status;
	if (pgopro_get_response_status_GET(pgopro_get_response, &item_status)) {
		e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS = item_status;
	}
	
}


void read_GPS_INJECT_DATA(pgps_inject_data_GPS_INJECT_DATA *const pgps_inject_data) {
	int8_t                target_system    = pgps_inject_data_target_system_GET(pgps_inject_data);
	int8_t                target_component = pgps_inject_data_target_component_GET(pgps_inject_data);
	int8_t                len              = pgps_inject_data_len_GET(pgps_inject_data);
	Vgps_inject_data_daTa item_daTa        = pgps_inject_data_daTa_GET(pgps_inject_data);
	for (size_t           index            = 0; index < item_daTa.len; index++)
		some_int8_t = vgps_inject_data_daTa_GET(&item_daTa, index);
	
}


void write_GPS_INJECT_DATA(pgps_inject_data_GPS_INJECT_DATA *const pgps_inject_data) {
	pgps_inject_data_target_system_SET(-7, -35, -68, -90, 103, 86, 37, -22, -61, -20, pgps_inject_data);
	pgps_inject_data_target_component_SET(84, 115, -64, 20, 115, 39, -119, 99, 89, -69, pgps_inject_data);
	pgps_inject_data_len_SET(-60, -41, -126, -7, -60, 122, -10, -82, 9, -34, pgps_inject_data);
	pgps_inject_data_daTa_SET(&-43, -67, 23, -119, -42, -118, 105, -50, -109, 87, pgps_inject_data);
	
}


void read_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *const puavionix_adsb_transceiver_health_report) {
	e_UAVIONIX_ADSB_RF_HEALTH item_rfHealth;
	if (puavionix_adsb_transceiver_health_report_rfHealth_GET(puavionix_adsb_transceiver_health_report, &item_rfHealth)) {
		e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = item_rfHealth;
	}
	
}


void read_ATTITUDE_QUATERNION_COV(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *const pattitude_quaternion_cov) {
	int64_t                             time_usec = pattitude_quaternion_cov_time_usec_GET(pattitude_quaternion_cov);
	Vattitude_quaternion_cov_q          item_q    = pattitude_quaternion_cov_q_GET(pattitude_quaternion_cov);
	for (size_t                         index     = 0; index < item_q.len; index++)
		some_float                                      = vattitude_quaternion_cov_q_GET(&item_q, index);
	float                               rollspeed       = pattitude_quaternion_cov_rollspeed_GET(pattitude_quaternion_cov);
	float                               pitchspeed      = pattitude_quaternion_cov_pitchspeed_GET(pattitude_quaternion_cov);
	float                               yawspeed        = pattitude_quaternion_cov_yawspeed_GET(pattitude_quaternion_cov);
	Vattitude_quaternion_cov_covariance item_covariance = pattitude_quaternion_cov_covariance_GET(pattitude_quaternion_cov);
	for (size_t                         index           = 0; index < item_covariance.len; index++)
		some_float = vattitude_quaternion_cov_covariance_GET(&item_covariance, index);
	
}


void write_ATTITUDE_QUATERNION_COV(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *const pattitude_quaternion_cov) {
	pattitude_quaternion_cov_time_usec_SET(-5754574358553069734L, -8366999313523185153L, 342220200877248481L, 57147087129842802L, 4988469462967054404L, 7503886124409790421L, 5908036383634186683L, 4557176285952857775L, -8211894918767680060L, 339797592976306667L, pattitude_quaternion_cov);
	pattitude_quaternion_cov_q_SET(&-3.819467E37F, -2.8270235E38F, 2.6123774E38F, 4.2101032E37F, 2.8718744E38F, -1.1577252E38F, -1.9500109E38F, -1.9318093E38F, -2.6456495E38F, -1.1608547E38F, pattitude_quaternion_cov);
	pattitude_quaternion_cov_rollspeed_SET(-8.3482094E37F, -2.5956426E38F, 2.625999E38F, -2.3765743E38F, -1.1494938E38F, 4.9363683E37F, 2.2750925E38F, -1.3926219E38F, -3.289183E38F, 1.0713137E38F, pattitude_quaternion_cov);
	pattitude_quaternion_cov_pitchspeed_SET(4.578647E37F, 2.2101022E38F, 2.0818784E38F, 9.871129E37F, -7.079939E36F, -8.94869E37F, -1.0131606E38F, -3.3890956E37F, 2.6957754E38F, -2.5438476E38F, pattitude_quaternion_cov);
	pattitude_quaternion_cov_yawspeed_SET(5.9180907E37F, 2.1982002E37F, -8.125103E37F, -4.391114E36F, 1.5812544E38F, 1.2382922E38F, -1.818859E38F, 3.013322E37F, 3.5720234E37F, -2.877607E38F, pattitude_quaternion_cov);
	pattitude_quaternion_cov_covariance_SET(&-1.261102E38F, -1.3262152E38F, -1.1067031E38F, 2.4438217E38F, 1.9535719E38F, 1.4757607E38F, 1.4824781E38F, -1.2769394E38F, 1.0580666E38F, 9.286929E37F, pattitude_quaternion_cov);
	
}


void read_NAMED_VALUE_INT(pnamed_value_int_NAMED_VALUE_INT *const pnamed_value_int) {
	int32_t               time_boot_ms = pnamed_value_int_time_boot_ms_GET(pnamed_value_int);
	int32_t               value        = pnamed_value_int_value_GET(pnamed_value_int);
	Vnamed_value_int_name item_name;
	if (pnamed_value_int_name_GET(pnamed_value_int, &item_name)) {
		memcpy(some_string, item_name.bytes, item_name.len);
	}
	
}


void write_NAMED_VALUE_INT(pnamed_value_int_NAMED_VALUE_INT *const pnamed_value_int) {
	pnamed_value_int_time_boot_ms_SET(-115125695, -1365320022, 564978168, 773804662, -1171523921, -1564568772, 65384786, 235135705, 787432997, 878315547, pnamed_value_int);
	pnamed_value_int_value_SET(-699228834, -897282779, 907545441, -1754029180, -1060135481, 1308125061, 1438724214, -1140247741, 298178966, 811666451, pnamed_value_int);
	pnamed_value_int_name_SET(some_string, strlen(some_string), pnamed_value_int);
	
}


void read_RPM(prpm_RPM *const prpm) {
	float rpm1 = prpm_rpm1_GET(prpm);
	float rpm2 = prpm_rpm2_GET(prpm);
	
}


void read_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA *const pgps_rtcm_data) {
	int8_t              flags     = pgps_rtcm_data_flags_GET(pgps_rtcm_data);
	int8_t              len       = pgps_rtcm_data_len_GET(pgps_rtcm_data);
	Vgps_rtcm_data_daTa item_daTa = pgps_rtcm_data_daTa_GET(pgps_rtcm_data);
	for (size_t         index     = 0; index < item_daTa.len; index++)
		some_int8_t = vgps_rtcm_data_daTa_GET(&item_daTa, index);
	
}


void write_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA *const pgps_rtcm_data) {
	pgps_rtcm_data_flags_SET(81, 32, -3, 12, 125, -105, 109, -126, -95, -128, pgps_rtcm_data);
	pgps_rtcm_data_len_SET(54, -68, 28, -15, 11, 17, 0, 88, 1, 100, pgps_rtcm_data);
	pgps_rtcm_data_daTa_SET(&82, -22, -70, -74, 3, -111, 116, 40, 45, 115, pgps_rtcm_data);
	
}


void read_GLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *const pglobal_vision_position_estimate) {
	int64_t usec  = pglobal_vision_position_estimate_usec_GET(pglobal_vision_position_estimate);
	float   x     = pglobal_vision_position_estimate_x_GET(pglobal_vision_position_estimate);
	float   y     = pglobal_vision_position_estimate_y_GET(pglobal_vision_position_estimate);
	float   z     = pglobal_vision_position_estimate_z_GET(pglobal_vision_position_estimate);
	float   roll  = pglobal_vision_position_estimate_roll_GET(pglobal_vision_position_estimate);
	float   pitch = pglobal_vision_position_estimate_pitch_GET(pglobal_vision_position_estimate);
	float   yaw   = pglobal_vision_position_estimate_yaw_GET(pglobal_vision_position_estimate);
	
}


void write_GLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *const pglobal_vision_position_estimate) {
	pglobal_vision_position_estimate_usec_SET(5565805409876055445L, -5939367752808336679L, -1856797526830536809L, 2087917550806908982L, -5700669560757617700L, -1366405532991423086L, 3234621055925966626L, -4469890762067474263L, 3476684378026851951L, -6461152772945944166L, pglobal_vision_position_estimate);
	pglobal_vision_position_estimate_x_SET(2.681833E38F, 3.376172E38F, 1.8595806E38F, 5.509832E37F, -1.2379322E38F, 2.5201462E38F, 2.483155E38F, -1.703714E38F, 2.1056873E38F, 8.2177134E37F, pglobal_vision_position_estimate);
	pglobal_vision_position_estimate_y_SET(-9.667989E37F, 1.0990192E38F, -1.4728771E38F, 1.5018281E38F, -3.3807781E38F, 1.4065703E38F, 1.8453504E38F, 1.9649323E38F, -7.0884486E37F, 1.3691271E38F, pglobal_vision_position_estimate);
	pglobal_vision_position_estimate_z_SET(-1.6618037E38F, -3.2017814E38F, 2.1704991E38F, -2.4930452E38F, -1.8138344E38F, -1.3705138E38F, 7.511323E37F, 1.1344323E38F, 3.2033415E38F, -2.9122526E38F, pglobal_vision_position_estimate);
	pglobal_vision_position_estimate_roll_SET(1.8766853E38F, 2.5694458E38F, -3.7294625E37F, 3.4522563E37F, -2.524658E38F, 6.521267E36F, 9.880276E37F, -1.5360531E38F, -1.7185691E38F, -2.5586988E38F, pglobal_vision_position_estimate);
	pglobal_vision_position_estimate_pitch_SET(-1.1792195E38F, -2.3427266E38F, 1.9339263E38F, 3.2332982E38F, 2.5591937E38F, 1.8896358E37F, -8.582952E37F, -3.3821153E38F, 2.4602321E38F, -3.1077373E38F, pglobal_vision_position_estimate);
	pglobal_vision_position_estimate_yaw_SET(-1.714309E38F, -1.2444855E38F, -6.1974614E36F, 1.5866162E38F, 1.5094844E38F, 2.9110278E38F, -3.2979504E38F, 1.3683515E38F, -2.2445291E38F, 1.746557E38F, pglobal_vision_position_estimate);
	
}


void read_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *const pfile_transfer_protocol) {
	int8_t                          target_network   = pfile_transfer_protocol_target_network_GET(pfile_transfer_protocol);
	int8_t                          target_system    = pfile_transfer_protocol_target_system_GET(pfile_transfer_protocol);
	int8_t                          target_component = pfile_transfer_protocol_target_component_GET(pfile_transfer_protocol);
	Vfile_transfer_protocol_payload item_payload     = pfile_transfer_protocol_payload_GET(pfile_transfer_protocol);
	for (size_t                     index            = 0; index < item_payload.len; index++)
		some_int8_t = vfile_transfer_protocol_payload_GET(&item_payload, index);
	
}


void write_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *const pfile_transfer_protocol) {
	pfile_transfer_protocol_target_network_SET(6, -42, 4, 88, -113, -73, -115, 22, 119, -66, pfile_transfer_protocol);
	pfile_transfer_protocol_target_system_SET(-54, -74, 39, -28, -21, -11, -7, 85, 48, -119, pfile_transfer_protocol);
	pfile_transfer_protocol_target_component_SET(2, -77, 90, -76, 19, -57, -87, -116, 81, -80, pfile_transfer_protocol);
	pfile_transfer_protocol_payload_SET(&19, 78, -92, -53, -17, 28, 24, -87, 122, -125, pfile_transfer_protocol);
	
}


void read_RANGEFINDER(prangefinder_RANGEFINDER *const prangefinder) {
	float distance = prangefinder_distance_GET(prangefinder);
	float voltage  = prangefinder_voltage_GET(prangefinder);
	
}


void read_RADIO_STATUS(pradio_status_RADIO_STATUS *const pradio_status) {
	int16_t rxerrors = pradio_status_rxerrors_GET(pradio_status);
	int16_t fixeD    = pradio_status_fixeD_GET(pradio_status);
	int8_t  rssi     = pradio_status_rssi_GET(pradio_status);
	int8_t  remrssi  = pradio_status_remrssi_GET(pradio_status);
	int8_t  txbuf    = pradio_status_txbuf_GET(pradio_status);
	int8_t  noise    = pradio_status_noise_GET(pradio_status);
	int8_t  remnoise = pradio_status_remnoise_GET(pradio_status);
	
}


void write_RADIO_STATUS(pradio_status_RADIO_STATUS *const pradio_status) {
	pradio_status_rxerrors_SET(12389, 12767, -15652, -5563, -13983, 1827, 4674, 3114, 15476, -17996, pradio_status);
	pradio_status_fixeD_SET(13185, 3240, -26430, 5986, -6165, -24304, -5410, 25142, 29608, -19440, pradio_status);
	pradio_status_rssi_SET(22, 57, -67, -48, -116, -81, 44, 78, 10, 0, pradio_status);
	pradio_status_remrssi_SET(-48, 127, -110, -118, -23, -96, 6, -120, 22, -63, pradio_status);
	pradio_status_txbuf_SET(-102, 117, -9, -17, 109, -128, -31, 5, 105, -110, pradio_status);
	pradio_status_noise_SET(103, 73, -23, 106, -72, 112, -62, -74, -86, 17, pradio_status);
	pradio_status_remnoise_SET(41, -8, -48, -121, 15, -112, -126, -22, 62, 53, pradio_status);
	
}


void read_FENCE_POINT(pfence_point_FENCE_POINT *const pfence_point) {
	int8_t target_system    = pfence_point_target_system_GET(pfence_point);
	int8_t target_component = pfence_point_target_component_GET(pfence_point);
	int8_t idx              = pfence_point_idx_GET(pfence_point);
	int8_t count            = pfence_point_count_GET(pfence_point);
	float  lat              = pfence_point_lat_GET(pfence_point);
	float  lng              = pfence_point_lng_GET(pfence_point);
	
}


void read_RESOURCE_REQUEST(presource_request_RESOURCE_REQUEST *const presource_request) {
	int8_t                    request_id = presource_request_request_id_GET(presource_request);
	int8_t                    uri_type   = presource_request_uri_type_GET(presource_request);
	Vresource_request_uri     item_uri   = presource_request_uri_GET(presource_request);
	for (size_t               index      = 0; index < item_uri.len; index++)
		some_int8_t                         = vresource_request_uri_GET(&item_uri, index);
	int8_t                    transfer_type = presource_request_transfer_type_GET(presource_request);
	Vresource_request_storage item_storage  = presource_request_storage_GET(presource_request);
	for (size_t               index         = 0; index < item_storage.len; index++)
		some_int8_t = vresource_request_storage_GET(&item_storage, index);
	
}


void write_RESOURCE_REQUEST(presource_request_RESOURCE_REQUEST *const presource_request) {
	presource_request_request_id_SET(74, -57, 59, -6, 102, 115, -96, 111, 17, 52, presource_request);
	presource_request_uri_type_SET(79, -37, 15, 36, 69, 24, -36, 28, 5, -8, presource_request);
	presource_request_uri_SET(&55, 13, -52, -57, -20, -104, -113, 46, -46, -10, presource_request);
	presource_request_transfer_type_SET(-101, 82, -64, 57, 83, 33, -76, -123, -64, 100, presource_request);
	presource_request_storage_SET(&36, 63, -93, 55, -1, 44, 90, -123, -33, 70, presource_request);
	
}


typedef struct {
	c_CommunicationChannel channel;
	RBUF_INIT(Pack*, 5) sending_out_packs;
}              c_CommunicationChannel_DEMO;

bool c_CommunicationChannel_send(c_CommunicationChannel *dst, const Pack *pack) {
	c_CommunicationChannel_DEMO *ch = (c_CommunicationChannel_DEMO *) dst;
	if (RBUF_ISFULL(ch->sending_out_packs)) return false;
	RBUF_PUT(ch->sending_out_packs, pack)
	return true;
}

const Pack *CommunicationChannel_DEMO_pull(Transmitter *transmitter) {
	c_CommunicationChannel_DEMO *ch = (c_CommunicationChannel_DEMO *) ((uint8_t *) transmitter - offsetof(c_CommunicationChannel_DEMO, channel) - offsetof(c_CommunicationChannel, transmitter));
	if (RBUF_ISEMPTY(ch->sending_out_packs)) return NULL;
	return RBUF_GET(ch->sending_out_packs);
}

static inline void CommunicationChannel_DEMO_on_FOLLOW_TARGET(c_CommunicationChannel_DEMO *channel, FOLLOW_TARGET_follow_target *pfollow_target) {
	
	read_FOLLOW_TARGET(pfollow_target->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ADSB_VEHICLE(c_CommunicationChannel_DEMO *channel, ADSB_VEHICLE_adsb_vehicle *padsb_vehicle) {
	
	CURSORS(curs);
	read_ADSB_VEHICLE(wrap_pack(padsb_vehicle, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MESSAGE_INTERVAL(c_CommunicationChannel_DEMO *channel, MESSAGE_INTERVAL_message_interval *pmessage_interval) {
	
	read_MESSAGE_INTERVAL(pmessage_interval->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_EKF_STATUS_REPORT(c_CommunicationChannel_DEMO *channel, EKF_STATUS_REPORT_ekf_status_report *pekf_status_report) {
	
	CURSORS(curs);
	read_EKF_STATUS_REPORT(wrap_pack(pekf_status_report, curs));
	
}

static inline void CommunicationChannel_DEMO_on_ESTIMATOR_STATUS(c_CommunicationChannel_DEMO *channel, ESTIMATOR_STATUS_estimator_status *pestimator_status) {
	
	CURSORS(curs);
	read_ESTIMATOR_STATUS(wrap_pack(pestimator_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_HWSTATUS(c_CommunicationChannel_DEMO *channel, HWSTATUS_hwstatus *phwstatus) {
	
	read_HWSTATUS(phwstatus->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_TIMESYNC(c_CommunicationChannel_DEMO *channel, TIMESYNC_timesync *ptimesync) {
	
	read_TIMESYNC(ptimesync->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PARAM_EXT_REQUEST_LIST(c_CommunicationChannel_DEMO *channel, PARAM_EXT_REQUEST_LIST_param_ext_request_list *pparam_ext_request_list) {
	
	read_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_BUTTON_CHANGE(c_CommunicationChannel_DEMO *channel, BUTTON_CHANGE_button_change *pbutton_change) {
	
	read_BUTTON_CHANGE(pbutton_change->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_UAVCAN_NODE_STATUS(c_CommunicationChannel_DEMO *channel, UAVCAN_NODE_STATUS_uavcan_node_status *puavcan_node_status) {
	
	CURSORS(curs);
	read_UAVCAN_NODE_STATUS(wrap_pack(puavcan_node_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_COLLISION(c_CommunicationChannel_DEMO *channel, COLLISION_collision *pcollision) {
	
	CURSORS(curs);
	read_COLLISION(wrap_pack(pcollision, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GIMBAL_TORQUE_CMD_REPORT(c_CommunicationChannel_DEMO *channel, GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report *pgimbal_torque_cmd_report) {
	
	read_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ALTITUDE(c_CommunicationChannel_DEMO *channel, ALTITUDE_altitude *paltitude) {
	
	read_ALTITUDE(paltitude->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HIL_STATE_QUATERNION(c_CommunicationChannel_DEMO *channel, HIL_STATE_QUATERNION_hil_state_quaternion *phil_state_quaternion) {
	
	read_HIL_STATE_QUATERNION(phil_state_quaternion->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SENSOR_OFFSETS(c_CommunicationChannel_DEMO *channel, SENSOR_OFFSETS_sensor_offsets *psensor_offsets) {
	
	read_SENSOR_OFFSETS(psensor_offsets->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_STORAGE_INFORMATION(c_CommunicationChannel_DEMO *channel, STORAGE_INFORMATION_storage_information *pstorage_information) {
	
	read_STORAGE_INFORMATION(pstorage_information->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_INFORMATION(c_CommunicationChannel_DEMO *channel, CAMERA_INFORMATION_camera_information *pcamera_information) {
	
	CURSORS(curs);
	read_CAMERA_INFORMATION(wrap_pack(pcamera_information, curs));
	
}

static inline void CommunicationChannel_DEMO_on_DEVICE_OP_WRITE_REPLY(c_CommunicationChannel_DEMO *channel, DEVICE_OP_WRITE_REPLY_device_op_write_reply *pdevice_op_write_reply) {
	
	read_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_TERRAIN_DATA(c_CommunicationChannel_DEMO *channel, TERRAIN_DATA_terrain_data *pterrain_data) {
	
	read_TERRAIN_DATA(pterrain_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GIMBAL_CONTROL(c_CommunicationChannel_DEMO *channel, GIMBAL_CONTROL_gimbal_control *pgimbal_control) {
	
	read_GIMBAL_CONTROL(pgimbal_control->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_VIDEO_STREAM_INFORMATION(c_CommunicationChannel_DEMO *channel, VIDEO_STREAM_INFORMATION_video_stream_information *pvideo_stream_information) {
	
	CURSORS(curs);
	read_VIDEO_STREAM_INFORMATION(wrap_pack(pvideo_stream_information, curs));
	
}

static inline void CommunicationChannel_DEMO_on_AHRS(c_CommunicationChannel_DEMO *channel, AHRS_ahrs *pahrs) {
	
	read_AHRS(pahrs->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DEBUG(c_CommunicationChannel_DEMO *channel, DEBUG_debug *pdebug) {
	
	read_DEBUG(pdebug->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_IMAGE_CAPTURED(c_CommunicationChannel_DEMO *channel, CAMERA_IMAGE_CAPTURED_camera_image_captured *pcamera_image_captured) {
	
	CURSORS(curs);
	read_CAMERA_IMAGE_CAPTURED(wrap_pack(pcamera_image_captured, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOG_ENTRY(c_CommunicationChannel_DEMO *channel, LOG_ENTRY_log_entry *plog_entry) {
	
	read_LOG_ENTRY(plog_entry->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ACTUATOR_CONTROL_TARGET(c_CommunicationChannel_DEMO *channel, ACTUATOR_CONTROL_TARGET_actuator_control_target *pactuator_control_target) {
	
	read_ACTUATOR_CONTROL_TARGET(pactuator_control_target->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HIGH_LATENCY(c_CommunicationChannel_DEMO *channel, HIGH_LATENCY_high_latency *phigh_latency) {
	
	CURSORS(curs);
	read_HIGH_LATENCY(wrap_pack(phigh_latency, curs));
	
}

static inline void CommunicationChannel_DEMO_on_HOME_POSITION(c_CommunicationChannel_DEMO *channel, HOME_POSITION_home_position *phome_position) {
	
	CURSORS(curs);
	read_HOME_POSITION(wrap_pack(phome_position, curs));
	
}

static inline void CommunicationChannel_DEMO_on_FENCE_STATUS(c_CommunicationChannel_DEMO *channel, FENCE_STATUS_fence_status *pfence_status) {
	
	CURSORS(curs);
	read_FENCE_STATUS(wrap_pack(pfence_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_REMOTE_LOG_BLOCK_STATUS(c_CommunicationChannel_DEMO *channel, REMOTE_LOG_BLOCK_STATUS_remote_log_block_status *premote_log_block_status) {
	
	CURSORS(curs);
	read_REMOTE_LOG_BLOCK_STATUS(wrap_pack(premote_log_block_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_OBSTACLE_DISTANCE(c_CommunicationChannel_DEMO *channel, OBSTACLE_DISTANCE_obstacle_distance *pobstacle_distance) {
	
	CURSORS(curs);
	read_OBSTACLE_DISTANCE(wrap_pack(pobstacle_distance, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GPS2_RAW(c_CommunicationChannel_DEMO *channel, GPS2_RAW_gps2_raw *pgps2_raw) {
	
	CURSORS(curs);
	read_GPS2_RAW(wrap_pack(pgps2_raw, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MEMORY_VECT(c_CommunicationChannel_DEMO *channel, MEMORY_VECT_memory_vect *pmemory_vect) {
	
	read_MEMORY_VECT(pmemory_vect->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PARAM_EXT_REQUEST_READ(c_CommunicationChannel_DEMO *channel, PARAM_EXT_REQUEST_READ_param_ext_request_read *pparam_ext_request_read) {
	
	CURSORS(curs);
	read_PARAM_EXT_REQUEST_READ(wrap_pack(pparam_ext_request_read, curs));
	
}

static inline void CommunicationChannel_DEMO_on_HIL_SENSOR(c_CommunicationChannel_DEMO *channel, HIL_SENSOR_hil_sensor *phil_sensor) {
	
	read_HIL_SENSOR(phil_sensor->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SETUP_SIGNING(c_CommunicationChannel_DEMO *channel, SETUP_SIGNING_setup_signing *psetup_signing) {
	
	read_SETUP_SIGNING(psetup_signing->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS_RTK(c_CommunicationChannel_DEMO *channel, GPS_RTK_gps_rtk *pgps_rtk) {
	
	read_GPS_RTK(pgps_rtk->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_UAVIONIX_ADSB_OUT_CFG(c_CommunicationChannel_DEMO *channel, UAVIONIX_ADSB_OUT_CFG_uavionix_adsb_out_cfg *puavionix_adsb_out_cfg) {
	
	CURSORS(curs);
	read_UAVIONIX_ADSB_OUT_CFG(wrap_pack(puavionix_adsb_out_cfg, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LANDING_TARGET(c_CommunicationChannel_DEMO *channel, LANDING_TARGET_landing_target *planding_target) {
	
	CURSORS(curs);
	read_LANDING_TARGET(wrap_pack(planding_target, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SET_ACTUATOR_CONTROL_TARGET(c_CommunicationChannel_DEMO *channel, SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target *pset_actuator_control_target) {
	
	read_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_CONTROL_SYSTEM_STATE(c_CommunicationChannel_DEMO *channel, CONTROL_SYSTEM_STATE_control_system_state *pcontrol_system_state) {
	
	read_CONTROL_SYSTEM_STATE(pcontrol_system_state->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DATA32(c_CommunicationChannel_DEMO *channel, DATA32_data32 *pdata32) {
	
	read_DATA32(pdata32->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PING33(c_CommunicationChannel_DEMO *channel, PING33_ping33 *pping33) {
	
	CURSORS(curs);
	read_PING33(wrap_pack(pping33, curs));
	
}

static inline void CommunicationChannel_DEMO_on_RALLY_POINT(c_CommunicationChannel_DEMO *channel, RALLY_POINT_rally_point *prally_point) {
	
	CURSORS(curs);
	read_RALLY_POINT(wrap_pack(prally_point, curs));
	
}

static inline void CommunicationChannel_DEMO_on_ADAP_TUNING(c_CommunicationChannel_DEMO *channel, ADAP_TUNING_adap_tuning *padap_tuning) {
	
	CURSORS(curs);
	read_ADAP_TUNING(wrap_pack(padap_tuning, curs));
	
}

static inline void CommunicationChannel_DEMO_on_VIBRATION(c_CommunicationChannel_DEMO *channel, VIBRATION_vibration *pvibration) {
	
	read_VIBRATION(pvibration->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PARAM_EXT_VALUE(c_CommunicationChannel_DEMO *channel, PARAM_EXT_VALUE_param_ext_value *pparam_ext_value) {
	
	CURSORS(curs);
	read_PARAM_EXT_VALUE(wrap_pack(pparam_ext_value, curs));
	
}

static inline void CommunicationChannel_DEMO_on_BATTERY2(c_CommunicationChannel_DEMO *channel, BATTERY2_battery2 *pbattery2) {
	
	read_BATTERY2(pbattery2->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LIMITS_STATUS(c_CommunicationChannel_DEMO *channel, LIMITS_STATUS_limits_status *plimits_status) {
	
	CURSORS(curs);
	read_LIMITS_STATUS(wrap_pack(plimits_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_FEEDBACK(c_CommunicationChannel_DEMO *channel, CAMERA_FEEDBACK_camera_feedback *pcamera_feedback) {
	
	CURSORS(curs);
	read_CAMERA_FEEDBACK(wrap_pack(pcamera_feedback, curs));
	
}

static inline void CommunicationChannel_DEMO_on_HIL_GPS(c_CommunicationChannel_DEMO *channel, HIL_GPS_hil_gps *phil_gps) {
	
	read_HIL_GPS(phil_gps->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_FENCE_FETCH_POINT(c_CommunicationChannel_DEMO *channel, FENCE_FETCH_POINT_fence_fetch_point *pfence_fetch_point) {
	
	read_FENCE_FETCH_POINT(pfence_fetch_point->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_RADIO(c_CommunicationChannel_DEMO *channel, RADIO_radio *pradio) {
	
	read_RADIO(pradio->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_AIRSPEED_AUTOCAL(c_CommunicationChannel_DEMO *channel, AIRSPEED_AUTOCAL_airspeed_autocal *pairspeed_autocal) {
	
	read_AIRSPEED_AUTOCAL(pairspeed_autocal->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ATT_POS_MOCAP(c_CommunicationChannel_DEMO *channel, ATT_POS_MOCAP_att_pos_mocap *patt_pos_mocap) {
	
	read_ATT_POS_MOCAP(patt_pos_mocap->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_STATUSTEXT(c_CommunicationChannel_DEMO *channel, STATUSTEXT_statustext *pstatustext) {
	
	CURSORS(curs);
	read_STATUSTEXT(wrap_pack(pstatustext, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GOPRO_GET_REQUEST(c_CommunicationChannel_DEMO *channel, GOPRO_GET_REQUEST_gopro_get_request *pgopro_get_request) {
	
	CURSORS(curs);
	read_GOPRO_GET_REQUEST(wrap_pack(pgopro_get_request, curs));
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_CAPTURE_STATUS(c_CommunicationChannel_DEMO *channel, CAMERA_CAPTURE_STATUS_camera_capture_status *pcamera_capture_status) {
	
	read_CAMERA_CAPTURE_STATUS(pcamera_capture_status->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ENCAPSULATED_DATA(c_CommunicationChannel_DEMO *channel, ENCAPSULATED_DATA_encapsulated_data *pencapsulated_data) {
	
	read_ENCAPSULATED_DATA(pencapsulated_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS_INPUT(c_CommunicationChannel_DEMO *channel, GPS_INPUT_gps_input *pgps_input) {
	
	CURSORS(curs);
	read_GPS_INPUT(wrap_pack(pgps_input, curs));
	
}

static inline void CommunicationChannel_DEMO_on_COMPASSMOT_STATUS(c_CommunicationChannel_DEMO *channel, COMPASSMOT_STATUS_compassmot_status *pcompassmot_status) {
	
	read_COMPASSMOT_STATUS(pcompassmot_status->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LOG_REQUEST_DATA(c_CommunicationChannel_DEMO *channel, LOG_REQUEST_DATA_log_request_data *plog_request_data) {
	
	read_LOG_REQUEST_DATA(plog_request_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_STATUS(c_CommunicationChannel_DEMO *channel, CAMERA_STATUS_camera_status *pcamera_status) {
	
	CURSORS(curs);
	read_CAMERA_STATUS(wrap_pack(pcamera_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_SETTINGS(c_CommunicationChannel_DEMO *channel, CAMERA_SETTINGS_camera_settings *pcamera_settings) {
	
	CURSORS(curs);
	read_CAMERA_SETTINGS(wrap_pack(pcamera_settings, curs));
	
}

static inline void CommunicationChannel_DEMO_on_DEVICE_OP_READ_REPLY(c_CommunicationChannel_DEMO *channel, DEVICE_OP_READ_REPLY_device_op_read_reply *pdevice_op_read_reply) {
	
	read_DEVICE_OP_READ_REPLY(pdevice_op_read_reply->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DIGICAM_CONTROL(c_CommunicationChannel_DEMO *channel, DIGICAM_CONTROL_digicam_control *pdigicam_control) {
	
	read_DIGICAM_CONTROL(pdigicam_control->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_NAMED_VALUE_FLOAT(c_CommunicationChannel_DEMO *channel, NAMED_VALUE_FLOAT_named_value_float *pnamed_value_float) {
	
	CURSORS(curs);
	read_NAMED_VALUE_FLOAT(wrap_pack(pnamed_value_float, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GOPRO_HEARTBEAT(c_CommunicationChannel_DEMO *channel, GOPRO_HEARTBEAT_gopro_heartbeat *pgopro_heartbeat) {
	
	CURSORS(curs);
	read_GOPRO_HEARTBEAT(wrap_pack(pgopro_heartbeat, curs));
	
}

static inline void CommunicationChannel_DEMO_on_AHRS2(c_CommunicationChannel_DEMO *channel, AHRS2_ahrs2 *pahrs2) {
	
	read_AHRS2(pahrs2->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LOG_ERASE(c_CommunicationChannel_DEMO *channel, LOG_ERASE_log_erase *plog_erase) {
	
	read_LOG_ERASE(plog_erase->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_TERRAIN_REQUEST(c_CommunicationChannel_DEMO *channel, TERRAIN_REQUEST_terrain_request *pterrain_request) {
	
	read_TERRAIN_REQUEST(pterrain_request->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MOUNT_STATUS(c_CommunicationChannel_DEMO *channel, MOUNT_STATUS_mount_status *pmount_status) {
	
	read_MOUNT_STATUS(pmount_status->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PID_TUNING(c_CommunicationChannel_DEMO *channel, PID_TUNING_pid_tuning *ppid_tuning) {
	
	CURSORS(curs);
	read_PID_TUNING(wrap_pack(ppid_tuning, curs));
	
}

static inline void CommunicationChannel_DEMO_on_OPTICAL_FLOW_RAD(c_CommunicationChannel_DEMO *channel, OPTICAL_FLOW_RAD_optical_flow_rad *poptical_flow_rad) {
	
	read_OPTICAL_FLOW_RAD(poptical_flow_rad->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LOG_DATA(c_CommunicationChannel_DEMO *channel, LOG_DATA_log_data *plog_data) {
	
	read_LOG_DATA(plog_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_AHRS3(c_CommunicationChannel_DEMO *channel, AHRS3_ahrs3 *pahrs3) {
	
	read_AHRS3(pahrs3->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_VICON_POSITION_ESTIMATE(c_CommunicationChannel_DEMO *channel, VICON_POSITION_ESTIMATE_vicon_position_estimate *pvicon_position_estimate) {
	
	read_VICON_POSITION_ESTIMATE(pvicon_position_estimate->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS2_RTK(c_CommunicationChannel_DEMO *channel, GPS2_RTK_gps2_rtk *pgps2_rtk) {
	
	read_GPS2_RTK(pgps2_rtk->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MAG_CAL_REPORT(c_CommunicationChannel_DEMO *channel, MAG_CAL_REPORT_mag_cal_report *pmag_cal_report) {
	
	CURSORS(curs);
	read_MAG_CAL_REPORT(wrap_pack(pmag_cal_report, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOG_REQUEST_LIST(c_CommunicationChannel_DEMO *channel, LOG_REQUEST_LIST_log_request_list *plog_request_list) {
	
	read_LOG_REQUEST_LIST(plog_request_list->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MOUNT_CONFIGURE(c_CommunicationChannel_DEMO *channel, MOUNT_CONFIGURE_mount_configure *pmount_configure) {
	
	CURSORS(curs);
	read_MOUNT_CONFIGURE(wrap_pack(pmount_configure, curs));
	
}

static inline void CommunicationChannel_DEMO_on_V2_EXTENSION(c_CommunicationChannel_DEMO *channel, V2_EXTENSION_v2_extension *pv2_extension) {
	
	read_V2_EXTENSION(pv2_extension->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_POWER_STATUS(c_CommunicationChannel_DEMO *channel, POWER_STATUS_power_status *ppower_status) {
	
	CURSORS(curs);
	read_POWER_STATUS(wrap_pack(ppower_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_REMOTE_LOG_DATA_BLOCK(c_CommunicationChannel_DEMO *channel, REMOTE_LOG_DATA_BLOCK_remote_log_data_block *premote_log_data_block) {
	
	CURSORS(curs);
	read_REMOTE_LOG_DATA_BLOCK(wrap_pack(premote_log_data_block, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOGGING_DATA_ACKED(c_CommunicationChannel_DEMO *channel, LOGGING_DATA_ACKED_logging_data_acked *plogging_data_acked) {
	
	read_LOGGING_DATA_ACKED(plogging_data_acked->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_TERRAIN_CHECK(c_CommunicationChannel_DEMO *channel, TERRAIN_CHECK_terrain_check *pterrain_check) {
	
	read_TERRAIN_CHECK(pterrain_check->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_TERRAIN_REPORT(c_CommunicationChannel_DEMO *channel, TERRAIN_REPORT_terrain_report *pterrain_report) {
	
	read_TERRAIN_REPORT(pterrain_report->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SET_HOME_POSITION(c_CommunicationChannel_DEMO *channel, SET_HOME_POSITION_set_home_position *pset_home_position) {
	
	CURSORS(curs);
	read_SET_HOME_POSITION(wrap_pack(pset_home_position, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SwitchModeCommand(c_CommunicationChannel_DEMO *channel) {
}

static inline void CommunicationChannel_DEMO_on_SCALED_IMU3(c_CommunicationChannel_DEMO *channel, SCALED_IMU3_scaled_imu3 *pscaled_imu3) {
	
	read_SCALED_IMU3(pscaled_imu3->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MOUNT_CONTROL(c_CommunicationChannel_DEMO *channel, MOUNT_CONTROL_mount_control *pmount_control) {
	
	read_MOUNT_CONTROL(pmount_control->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LED_CONTROL(c_CommunicationChannel_DEMO *channel, LED_CONTROL_led_control *pled_control) {
	
	read_LED_CONTROL(pled_control->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SIM_STATE(c_CommunicationChannel_DEMO *channel, SIM_STATE_sim_state *psim_state) {
	
	read_SIM_STATE(psim_state->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_WIFI_CONFIG_AP(c_CommunicationChannel_DEMO *channel, WIFI_CONFIG_AP_wifi_config_ap *pwifi_config_ap) {
	
	CURSORS(curs);
	read_WIFI_CONFIG_AP(wrap_pack(pwifi_config_ap, curs));
	
}

static inline void CommunicationChannel_DEMO_on_DATA96(c_CommunicationChannel_DEMO *channel, DATA96_data96 *pdata96) {
	
	read_DATA96(pdata96->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_FLIGHT_INFORMATION(c_CommunicationChannel_DEMO *channel, FLIGHT_INFORMATION_flight_information *pflight_information) {
	
	read_FLIGHT_INFORMATION(pflight_information->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MEMINFO(c_CommunicationChannel_DEMO *channel, MEMINFO_meminfo *pmeminfo) {
	
	CURSORS(curs);
	read_MEMINFO(wrap_pack(pmeminfo, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOGGING_ACK(c_CommunicationChannel_DEMO *channel, LOGGING_ACK_logging_ack *plogging_ack) {
	
	read_LOGGING_ACK(plogging_ack->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_VISION_SPEED_ESTIMATE(c_CommunicationChannel_DEMO *channel, VISION_SPEED_ESTIMATE_vision_speed_estimate *pvision_speed_estimate) {
	
	read_VISION_SPEED_ESTIMATE(pvision_speed_estimate->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DEBUG_VECT(c_CommunicationChannel_DEMO *channel, DEBUG_VECT_debug_vect *pdebug_vect) {
	
	CURSORS(curs);
	read_DEBUG_VECT(wrap_pack(pdebug_vect, curs));
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_TRIGGER(c_CommunicationChannel_DEMO *channel, CAMERA_TRIGGER_camera_trigger *pcamera_trigger) {
	
	read_CAMERA_TRIGGER(pcamera_trigger->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LOG_REQUEST_END(c_CommunicationChannel_DEMO *channel, LOG_REQUEST_END_log_request_end *plog_request_end) {
	
	read_LOG_REQUEST_END(plog_request_end->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GOPRO_SET_RESPONSE(c_CommunicationChannel_DEMO *channel, GOPRO_SET_RESPONSE_gopro_set_response *pgopro_set_response) {
	
	CURSORS(curs);
	read_GOPRO_SET_RESPONSE(wrap_pack(pgopro_set_response, curs));
	
}

static inline void CommunicationChannel_DEMO_on_PROTOCOL_VERSION(c_CommunicationChannel_DEMO *channel, PROTOCOL_VERSION_protocol_version *pprotocol_version) {
	
	read_PROTOCOL_VERSION(pprotocol_version->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_RALLY_FETCH_POINT(c_CommunicationChannel_DEMO *channel, RALLY_FETCH_POINT_rally_fetch_point *prally_fetch_point) {
	
	read_RALLY_FETCH_POINT(prally_fetch_point->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_BATTERY_STATUS(c_CommunicationChannel_DEMO *channel, BATTERY_STATUS_battery_status *pbattery_status) {
	
	CURSORS(curs);
	read_BATTERY_STATUS(wrap_pack(pbattery_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MOUNT_ORIENTATION(c_CommunicationChannel_DEMO *channel, MOUNT_ORIENTATION_mount_orientation *pmount_orientation) {
	
	read_MOUNT_ORIENTATION(pmount_orientation->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SERIAL_CONTROL(c_CommunicationChannel_DEMO *channel, SERIAL_CONTROL_serial_control *pserial_control) {
	
	CURSORS(curs);
	read_SERIAL_CONTROL(wrap_pack(pserial_control, curs));
	
}

static inline void CommunicationChannel_DEMO_on_PARAM_EXT_SET(c_CommunicationChannel_DEMO *channel, PARAM_EXT_SET_param_ext_set *pparam_ext_set) {
	
	CURSORS(curs);
	read_PARAM_EXT_SET(wrap_pack(pparam_ext_set, curs));
	
}

static inline void CommunicationChannel_DEMO_on_AUTOPILOT_VERSION(c_CommunicationChannel_DEMO *channel, AUTOPILOT_VERSION_autopilot_version *pautopilot_version) {
	
	CURSORS(curs);
	read_AUTOPILOT_VERSION(wrap_pack(pautopilot_version, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SIMSTATE(c_CommunicationChannel_DEMO *channel, SIMSTATE_simstate *psimstate) {
	
	read_SIMSTATE(psimstate->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SET_VIDEO_STREAM_SETTINGS(c_CommunicationChannel_DEMO *channel, SET_VIDEO_STREAM_SETTINGS_set_video_stream_settings *pset_video_stream_settings) {
	
	CURSORS(curs);
	read_SET_VIDEO_STREAM_SETTINGS(wrap_pack(pset_video_stream_settings, curs));
	
}

static inline void CommunicationChannel_DEMO_on_PLAY_TUNE(c_CommunicationChannel_DEMO *channel, PLAY_TUNE_play_tune *pplay_tune) {
	
	CURSORS(curs);
	read_PLAY_TUNE(wrap_pack(pplay_tune, curs));
	
}

static inline void CommunicationChannel_DEMO_on_DIGICAM_CONFIGURE(c_CommunicationChannel_DEMO *channel, DIGICAM_CONFIGURE_digicam_configure *pdigicam_configure) {
	
	read_DIGICAM_CONFIGURE(pdigicam_configure->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SCALED_PRESSURE3(c_CommunicationChannel_DEMO *channel, SCALED_PRESSURE3_scaled_pressure3 *pscaled_pressure3) {
	
	read_SCALED_PRESSURE3(pscaled_pressure3->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PARAM_EXT_ACK(c_CommunicationChannel_DEMO *channel, PARAM_EXT_ACK_param_ext_ack *pparam_ext_ack) {
	
	CURSORS(curs);
	read_PARAM_EXT_ACK(wrap_pack(pparam_ext_ack, curs));
	
}

static inline void CommunicationChannel_DEMO_on_UAVCAN_NODE_INFO(c_CommunicationChannel_DEMO *channel, UAVCAN_NODE_INFO_uavcan_node_info *puavcan_node_info) {
	
	CURSORS(curs);
	read_UAVCAN_NODE_INFO(wrap_pack(puavcan_node_info, curs));
	
}

static inline void CommunicationChannel_DEMO_on_DATA16(c_CommunicationChannel_DEMO *channel, DATA16_data16 *pdata16) {
	
	read_DATA16(pdata16->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SET_MAG_OFFSETS(c_CommunicationChannel_DEMO *channel, SET_MAG_OFFSETS_set_mag_offsets *pset_mag_offsets) {
	
	read_SET_MAG_OFFSETS(pset_mag_offsets->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SCALED_IMU2(c_CommunicationChannel_DEMO *channel, SCALED_IMU2_scaled_imu2 *pscaled_imu2) {
	
	read_SCALED_IMU2(pscaled_imu2->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_AP_ADC(c_CommunicationChannel_DEMO *channel, AP_ADC_ap_adc *pap_adc) {
	
	read_AP_ADC(pap_adc->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_WIND(c_CommunicationChannel_DEMO *channel, WIND_wind *pwind) {
	
	read_WIND(pwind->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_AUTOPILOT_VERSION_REQUEST(c_CommunicationChannel_DEMO *channel, AUTOPILOT_VERSION_REQUEST_autopilot_version_request *pautopilot_version_request) {
	
	read_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DATA_TRANSMISSION_HANDSHAKE(c_CommunicationChannel_DEMO *channel, DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake *pdata_transmission_handshake) {
	
	read_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DATA64(c_CommunicationChannel_DEMO *channel, DATA64_data64 *pdata64) {
	
	read_DATA64(pdata64->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GIMBAL_REPORT(c_CommunicationChannel_DEMO *channel, GIMBAL_REPORT_gimbal_report *pgimbal_report) {
	
	read_GIMBAL_REPORT(pgimbal_report->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DEVICE_OP_WRITE(c_CommunicationChannel_DEMO *channel, DEVICE_OP_WRITE_device_op_write *pdevice_op_write) {
	
	CURSORS(curs);
	read_DEVICE_OP_WRITE(wrap_pack(pdevice_op_write, curs));
	
}

static inline void CommunicationChannel_DEMO_on_DISTANCE_SENSOR(c_CommunicationChannel_DEMO *channel, DISTANCE_SENSOR_distance_sensor *pdistance_sensor) {
	
	CURSORS(curs);
	read_DISTANCE_SENSOR(wrap_pack(pdistance_sensor, curs));
	
}

static inline void CommunicationChannel_DEMO_on_HIL_OPTICAL_FLOW(c_CommunicationChannel_DEMO *channel, HIL_OPTICAL_FLOW_hil_optical_flow *phil_optical_flow) {
	
	read_HIL_OPTICAL_FLOW(phil_optical_flow->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SCALED_PRESSURE2(c_CommunicationChannel_DEMO *channel, SCALED_PRESSURE2_scaled_pressure2 *pscaled_pressure2) {
	
	read_SCALED_PRESSURE2(pscaled_pressure2->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_WIND_COV(c_CommunicationChannel_DEMO *channel, WIND_COV_wind_cov *pwind_cov) {
	
	read_WIND_COV(pwind_cov->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GOPRO_SET_REQUEST(c_CommunicationChannel_DEMO *channel, GOPRO_SET_REQUEST_gopro_set_request *pgopro_set_request) {
	
	CURSORS(curs);
	read_GOPRO_SET_REQUEST(wrap_pack(pgopro_set_request, curs));
	
}

static inline void CommunicationChannel_DEMO_on_VISION_POSITION_DELTA(c_CommunicationChannel_DEMO *channel, VISION_POSITION_DELTA_vision_position_delta *pvision_position_delta) {
	
	read_VISION_POSITION_DELTA(pvision_position_delta->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LOGGING_DATA(c_CommunicationChannel_DEMO *channel, LOGGING_DATA_logging_data *plogging_data) {
	
	read_LOGGING_DATA(plogging_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DEVICE_OP_READ(c_CommunicationChannel_DEMO *channel, DEVICE_OP_READ_device_op_read *pdevice_op_read) {
	
	CURSORS(curs);
	read_DEVICE_OP_READ(wrap_pack(pdevice_op_read, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MAG_CAL_PROGRESS(c_CommunicationChannel_DEMO *channel, MAG_CAL_PROGRESS_mag_cal_progress *pmag_cal_progress) {
	
	CURSORS(curs);
	read_MAG_CAL_PROGRESS(wrap_pack(pmag_cal_progress, curs));
	
}

static inline void CommunicationChannel_DEMO_on_HIGHRES_IMU(c_CommunicationChannel_DEMO *channel, HIGHRES_IMU_highres_imu *phighres_imu) {
	
	read_HIGHRES_IMU(phighres_imu->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_EXTENDED_SYS_STATE(c_CommunicationChannel_DEMO *channel, EXTENDED_SYS_STATE_extended_sys_state *pextended_sys_state) {
	
	CURSORS(curs);
	read_EXTENDED_SYS_STATE(wrap_pack(pextended_sys_state, curs));
	
}

static inline void CommunicationChannel_DEMO_on_UAVIONIX_ADSB_OUT_DYNAMIC(c_CommunicationChannel_DEMO *channel, UAVIONIX_ADSB_OUT_DYNAMIC_uavionix_adsb_out_dynamic *puavionix_adsb_out_dynamic) {
	
	CURSORS(curs);
	read_UAVIONIX_ADSB_OUT_DYNAMIC(wrap_pack(puavionix_adsb_out_dynamic, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GOPRO_GET_RESPONSE(c_CommunicationChannel_DEMO *channel, GOPRO_GET_RESPONSE_gopro_get_response *pgopro_get_response) {
	
	CURSORS(curs);
	read_GOPRO_GET_RESPONSE(wrap_pack(pgopro_get_response, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GPS_INJECT_DATA(c_CommunicationChannel_DEMO *channel, GPS_INJECT_DATA_gps_inject_data *pgps_inject_data) {
	
	read_GPS_INJECT_DATA(pgps_inject_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(c_CommunicationChannel_DEMO *channel, UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_uavionix_adsb_transceiver_health_report *puavionix_adsb_transceiver_health_report) {
	
	CURSORS(curs);
	read_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(wrap_pack(puavionix_adsb_transceiver_health_report, curs));
	
}

static inline void CommunicationChannel_DEMO_on_NAMED_VALUE_INT(c_CommunicationChannel_DEMO *channel, NAMED_VALUE_INT_named_value_int *pnamed_value_int) {
	
	CURSORS(curs);
	read_NAMED_VALUE_INT(wrap_pack(pnamed_value_int, curs));
	
}

static inline void CommunicationChannel_DEMO_on_RPM(c_CommunicationChannel_DEMO *channel, RPM_rpm *prpm) {
	
	read_RPM(prpm->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS_RTCM_DATA(c_CommunicationChannel_DEMO *channel, GPS_RTCM_DATA_gps_rtcm_data *pgps_rtcm_data) {
	
	read_GPS_RTCM_DATA(pgps_rtcm_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_FILE_TRANSFER_PROTOCOL(c_CommunicationChannel_DEMO *channel, FILE_TRANSFER_PROTOCOL_file_transfer_protocol *pfile_transfer_protocol) {
	
	read_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_RANGEFINDER(c_CommunicationChannel_DEMO *channel, RANGEFINDER_rangefinder *prangefinder) {
	
	read_RANGEFINDER(prangefinder->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_RADIO_STATUS(c_CommunicationChannel_DEMO *channel, RADIO_STATUS_radio_status *pradio_status) {
	
	read_RADIO_STATUS(pradio_status->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_FENCE_POINT(c_CommunicationChannel_DEMO *channel, FENCE_POINT_fence_point *pfence_point) {
	
	read_FENCE_POINT(pfence_point->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_RESOURCE_REQUEST(c_CommunicationChannel_DEMO *channel, RESOURCE_REQUEST_resource_request *presource_request) {
	
	read_RESOURCE_REQUEST(presource_request->bytes);
	
}

static inline Meta const *CommunicationChannel_DEMO_dispatcher(Receiver *receiver, size_t id, Pack *pack) {
	c_CommunicationChannel_DEMO *channel = (c_CommunicationChannel_DEMO *) ((uint8_t *) receiver - offsetof(c_CommunicationChannel_DEMO, channel) - offsetof(c_CommunicationChannel, receiver));
	c_CommunicationChannel_DISPATCHER(CommunicationChannel_DEMO)
}


int main() {
	CURSORS(cur);
	c_CommunicationChannel_DEMO CommunicationChannel_instance = {.channel.receiver.dispatch = CommunicationChannel_DEMO_dispatcher, .channel.transmitter.pull = CommunicationChannel_DEMO_pull};
	
	{
		
		RESOURCE_REQUEST_resource_request *presource_request = c_CommunicationChannel_new_RESOURCE_REQUEST();
		write_RESOURCE_REQUEST(presource_request_RESOURCE_REQUEST_from(presource_request));
		if (!c_CommunicationChannel_send_RESOURCE_REQUEST(&CommunicationChannel_instance.channel, presource_request)) {
			free_pack(presource_request);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		ATTITUDE_TARGET_attitude_target *pattitude_target = c_CommunicationChannel_new_ATTITUDE_TARGET();
		write_ATTITUDE_TARGET(pattitude_target_ATTITUDE_TARGET_from(pattitude_target));
		if (!c_CommunicationChannel_send_ATTITUDE_TARGET(&CommunicationChannel_instance.channel, pattitude_target)) {
			free_pack(pattitude_target);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmission_count_MISSION_COUNT *pmission_count = c_CommunicationChannel_new_MISSION_COUNT(cur);
		write_MISSION_COUNT(pmission_count);
		if (!c_CommunicationChannel_send_MISSION_COUNT(&CommunicationChannel_instance.channel, pmission_count)) {
			free_pack(unwrap_pack(pmission_count));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		padsb_vehicle_ADSB_VEHICLE *padsb_vehicle = c_CommunicationChannel_new_ADSB_VEHICLE(cur);
		write_ADSB_VEHICLE(padsb_vehicle);
		if (!c_CommunicationChannel_send_ADSB_VEHICLE(&CommunicationChannel_instance.channel, padsb_vehicle)) {
			free_pack(unwrap_pack(padsb_vehicle));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		MESSAGE_INTERVAL_message_interval *pmessage_interval = c_CommunicationChannel_new_MESSAGE_INTERVAL();
		write_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL_from(pmessage_interval));
		if (!c_CommunicationChannel_send_MESSAGE_INTERVAL(&CommunicationChannel_instance.channel, pmessage_interval)) {
			free_pack(pmessage_interval);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pestimator_status_ESTIMATOR_STATUS *pestimator_status = c_CommunicationChannel_new_ESTIMATOR_STATUS(cur);
		write_ESTIMATOR_STATUS(pestimator_status);
		if (!c_CommunicationChannel_send_ESTIMATOR_STATUS(&CommunicationChannel_instance.channel, pestimator_status)) {
			free_pack(unwrap_pack(pestimator_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		TIMESYNC_timesync *ptimesync = c_CommunicationChannel_new_TIMESYNC();
		write_TIMESYNC(ptimesync_TIMESYNC_from(ptimesync));
		if (!c_CommunicationChannel_send_TIMESYNC(&CommunicationChannel_instance.channel, ptimesync)) {
			free_pack(ptimesync);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *pglobal_position_int_cov = c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV(cur);
		write_GLOBAL_POSITION_INT_COV(pglobal_position_int_cov);
		if (!c_CommunicationChannel_send_GLOBAL_POSITION_INT_COV(&CommunicationChannel_instance.channel, pglobal_position_int_cov)) {
			free_pack(unwrap_pack(pglobal_position_int_cov));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		BUTTON_CHANGE_button_change *pbutton_change = c_CommunicationChannel_new_BUTTON_CHANGE();
		write_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE_from(pbutton_change));
		if (!c_CommunicationChannel_send_BUTTON_CHANGE(&CommunicationChannel_instance.channel, pbutton_change)) {
			free_pack(pbutton_change);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *psafety_set_allowed_area = c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA(cur);
		write_SAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area);
		if (!c_CommunicationChannel_send_SAFETY_SET_ALLOWED_AREA(&CommunicationChannel_instance.channel, psafety_set_allowed_area)) {
			free_pack(unwrap_pack(psafety_set_allowed_area));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		STORAGE_INFORMATION_storage_information *pstorage_information = c_CommunicationChannel_new_STORAGE_INFORMATION();
		write_STORAGE_INFORMATION(pstorage_information_STORAGE_INFORMATION_from(pstorage_information));
		if (!c_CommunicationChannel_send_STORAGE_INFORMATION(&CommunicationChannel_instance.channel, pstorage_information)) {
			free_pack(pstorage_information);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pcollision_COLLISION *pcollision = c_CommunicationChannel_new_COLLISION(cur);
		write_COLLISION(pcollision);
		if (!c_CommunicationChannel_send_COLLISION(&CommunicationChannel_instance.channel, pcollision)) {
			free_pack(unwrap_pack(pcollision));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		ALTITUDE_altitude *paltitude = c_CommunicationChannel_new_ALTITUDE();
		write_ALTITUDE(paltitude_ALTITUDE_from(paltitude));
		if (!c_CommunicationChannel_send_ALTITUDE(&CommunicationChannel_instance.channel, paltitude)) {
			free_pack(paltitude);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		HIL_STATE_QUATERNION_hil_state_quaternion *phil_state_quaternion = c_CommunicationChannel_new_HIL_STATE_QUATERNION();
		write_HIL_STATE_QUATERNION(phil_state_quaternion_HIL_STATE_QUATERNION_from(phil_state_quaternion));
		if (!c_CommunicationChannel_send_HIL_STATE_QUATERNION(&CommunicationChannel_instance.channel, phil_state_quaternion)) {
			free_pack(phil_state_quaternion);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pcamera_information_CAMERA_INFORMATION *pcamera_information = c_CommunicationChannel_new_CAMERA_INFORMATION(cur);
		write_CAMERA_INFORMATION(pcamera_information);
		if (!c_CommunicationChannel_send_CAMERA_INFORMATION(&CommunicationChannel_instance.channel, pcamera_information)) {
			free_pack(unwrap_pack(pcamera_information));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		GPS_STATUS_gps_status *pgps_status = c_CommunicationChannel_new_GPS_STATUS();
		write_GPS_STATUS(pgps_status_GPS_STATUS_from(pgps_status));
		if (!c_CommunicationChannel_send_GPS_STATUS(&CommunicationChannel_instance.channel, pgps_status)) {
			free_pack(pgps_status);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pparam_set_PARAM_SET *pparam_set = c_CommunicationChannel_new_PARAM_SET(cur);
		write_PARAM_SET(pparam_set);
		if (!c_CommunicationChannel_send_PARAM_SET(&CommunicationChannel_instance.channel, pparam_set)) {
			free_pack(unwrap_pack(pparam_set));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		TERRAIN_DATA_terrain_data *pterrain_data = c_CommunicationChannel_new_TERRAIN_DATA();
		write_TERRAIN_DATA(pterrain_data_TERRAIN_DATA_from(pterrain_data));
		if (!c_CommunicationChannel_send_TERRAIN_DATA(&CommunicationChannel_instance.channel, pterrain_data)) {
			free_pack(pterrain_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		RC_CHANNELS_OVERRIDE_rc_channels_override *prc_channels_override = c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE();
		write_RC_CHANNELS_OVERRIDE(prc_channels_override_RC_CHANNELS_OVERRIDE_from(prc_channels_override));
		if (!c_CommunicationChannel_send_RC_CHANNELS_OVERRIDE(&CommunicationChannel_instance.channel, prc_channels_override)) {
			free_pack(prc_channels_override);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SCALED_IMU_scaled_imu *pscaled_imu = c_CommunicationChannel_new_SCALED_IMU();
		write_SCALED_IMU(pscaled_imu_SCALED_IMU_from(pscaled_imu));
		if (!c_CommunicationChannel_send_SCALED_IMU(&CommunicationChannel_instance.channel, pscaled_imu)) {
			free_pack(pscaled_imu);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		DEBUG_debug *pdebug = c_CommunicationChannel_new_DEBUG();
		write_DEBUG(pdebug_DEBUG_from(pdebug));
		if (!c_CommunicationChannel_send_DEBUG(&CommunicationChannel_instance.channel, pdebug)) {
			free_pack(pdebug);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pcamera_image_captured_CAMERA_IMAGE_CAPTURED *pcamera_image_captured = c_CommunicationChannel_new_CAMERA_IMAGE_CAPTURED(cur);
		write_CAMERA_IMAGE_CAPTURED(pcamera_image_captured);
		if (!c_CommunicationChannel_send_CAMERA_IMAGE_CAPTURED(&CommunicationChannel_instance.channel, pcamera_image_captured)) {
			free_pack(unwrap_pack(pcamera_image_captured));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		LOG_ENTRY_log_entry *plog_entry = c_CommunicationChannel_new_LOG_ENTRY();
		write_LOG_ENTRY(plog_entry_LOG_ENTRY_from(plog_entry));
		if (!c_CommunicationChannel_send_LOG_ENTRY(&CommunicationChannel_instance.channel, plog_entry)) {
			free_pack(plog_entry);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		ACTUATOR_CONTROL_TARGET_actuator_control_target *pactuator_control_target = c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET();
		write_ACTUATOR_CONTROL_TARGET(pactuator_control_target_ACTUATOR_CONTROL_TARGET_from(pactuator_control_target));
		if (!c_CommunicationChannel_send_ACTUATOR_CONTROL_TARGET(&CommunicationChannel_instance.channel, pactuator_control_target)) {
			free_pack(pactuator_control_target);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		phigh_latency_HIGH_LATENCY *phigh_latency = c_CommunicationChannel_new_HIGH_LATENCY(cur);
		write_HIGH_LATENCY(phigh_latency);
		if (!c_CommunicationChannel_send_HIGH_LATENCY(&CommunicationChannel_instance.channel, phigh_latency)) {
			free_pack(unwrap_pack(phigh_latency));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pparam_request_read_PARAM_REQUEST_READ *pparam_request_read = c_CommunicationChannel_new_PARAM_REQUEST_READ(cur);
		write_PARAM_REQUEST_READ(pparam_request_read);
		if (!c_CommunicationChannel_send_PARAM_REQUEST_READ(&CommunicationChannel_instance.channel, pparam_request_read)) {
			free_pack(unwrap_pack(pparam_request_read));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		SET_ATTITUDE_TARGET_set_attitude_target *pset_attitude_target = c_CommunicationChannel_new_SET_ATTITUDE_TARGET();
		write_SET_ATTITUDE_TARGET(pset_attitude_target_SET_ATTITUDE_TARGET_from(pset_attitude_target));
		if (!c_CommunicationChannel_send_SET_ATTITUDE_TARGET(&CommunicationChannel_instance.channel, pset_attitude_target)) {
			free_pack(pset_attitude_target);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		FOLLOW_TARGET_follow_target *pfollow_target = c_CommunicationChannel_new_FOLLOW_TARGET();
		write_FOLLOW_TARGET(pfollow_target_FOLLOW_TARGET_from(pfollow_target));
		if (!c_CommunicationChannel_send_FOLLOW_TARGET(&CommunicationChannel_instance.channel, pfollow_target)) {
			free_pack(pfollow_target);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		HIL_STATE_hil_state *phil_state = c_CommunicationChannel_new_HIL_STATE();
		write_HIL_STATE(phil_state_HIL_STATE_from(phil_state));
		if (!c_CommunicationChannel_send_HIL_STATE(&CommunicationChannel_instance.channel, phil_state)) {
			free_pack(phil_state);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		phome_position_HOME_POSITION *phome_position = c_CommunicationChannel_new_HOME_POSITION(cur);
		write_HOME_POSITION(phome_position);
		if (!c_CommunicationChannel_send_HOME_POSITION(&CommunicationChannel_instance.channel, phome_position)) {
			free_pack(unwrap_pack(phome_position));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pgps2_raw_GPS2_RAW *pgps2_raw = c_CommunicationChannel_new_GPS2_RAW(cur);
		write_GPS2_RAW(pgps2_raw);
		if (!c_CommunicationChannel_send_GPS2_RAW(&CommunicationChannel_instance.channel, pgps2_raw)) {
			free_pack(unwrap_pack(pgps2_raw));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		MEMORY_VECT_memory_vect *pmemory_vect = c_CommunicationChannel_new_MEMORY_VECT();
		write_MEMORY_VECT(pmemory_vect_MEMORY_VECT_from(pmemory_vect));
		if (!c_CommunicationChannel_send_MEMORY_VECT(&CommunicationChannel_instance.channel, pmemory_vect)) {
			free_pack(pmemory_vect);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		REQUEST_DATA_STREAM_request_data_stream *prequest_data_stream = c_CommunicationChannel_new_REQUEST_DATA_STREAM();
		write_REQUEST_DATA_STREAM(prequest_data_stream_REQUEST_DATA_STREAM_from(prequest_data_stream));
		if (!c_CommunicationChannel_send_REQUEST_DATA_STREAM(&CommunicationChannel_instance.channel, prequest_data_stream)) {
			free_pack(prequest_data_stream);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		phil_controls_HIL_CONTROLS *phil_controls = c_CommunicationChannel_new_HIL_CONTROLS(cur);
		write_HIL_CONTROLS(phil_controls);
		if (!c_CommunicationChannel_send_HIL_CONTROLS(&CommunicationChannel_instance.channel, phil_controls)) {
			free_pack(unwrap_pack(phil_controls));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		HIL_SENSOR_hil_sensor *phil_sensor = c_CommunicationChannel_new_HIL_SENSOR();
		write_HIL_SENSOR(phil_sensor_HIL_SENSOR_from(phil_sensor));
		if (!c_CommunicationChannel_send_HIL_SENSOR(&CommunicationChannel_instance.channel, phil_sensor)) {
			free_pack(phil_sensor);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SETUP_SIGNING_setup_signing *psetup_signing = c_CommunicationChannel_new_SETUP_SIGNING();
		write_SETUP_SIGNING(psetup_signing_SETUP_SIGNING_from(psetup_signing));
		if (!c_CommunicationChannel_send_SETUP_SIGNING(&CommunicationChannel_instance.channel, psetup_signing)) {
			free_pack(psetup_signing);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		GPS_RTK_gps_rtk *pgps_rtk = c_CommunicationChannel_new_GPS_RTK();
		write_GPS_RTK(pgps_rtk_GPS_RTK_from(pgps_rtk));
		if (!c_CommunicationChannel_send_GPS_RTK(&CommunicationChannel_instance.channel, pgps_rtk)) {
			free_pack(pgps_rtk);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		PARAM_REQUEST_LIST_param_request_list *pparam_request_list = c_CommunicationChannel_new_PARAM_REQUEST_LIST();
		write_PARAM_REQUEST_LIST(pparam_request_list_PARAM_REQUEST_LIST_from(pparam_request_list));
		if (!c_CommunicationChannel_send_PARAM_REQUEST_LIST(&CommunicationChannel_instance.channel, pparam_request_list)) {
			free_pack(pparam_request_list);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		planding_target_LANDING_TARGET *planding_target = c_CommunicationChannel_new_LANDING_TARGET(cur);
		write_LANDING_TARGET(planding_target);
		if (!c_CommunicationChannel_send_LANDING_TARGET(&CommunicationChannel_instance.channel, planding_target)) {
			free_pack(unwrap_pack(planding_target));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		SET_ACTUATOR_CONTROL_TARGET_set_actuator_control_target *pset_actuator_control_target = c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET();
		write_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET_from(pset_actuator_control_target));
		if (!c_CommunicationChannel_send_SET_ACTUATOR_CONTROL_TARGET(&CommunicationChannel_instance.channel, pset_actuator_control_target)) {
			free_pack(pset_actuator_control_target);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		CONTROL_SYSTEM_STATE_control_system_state *pcontrol_system_state = c_CommunicationChannel_new_CONTROL_SYSTEM_STATE();
		write_CONTROL_SYSTEM_STATE(pcontrol_system_state_CONTROL_SYSTEM_STATE_from(pcontrol_system_state));
		if (!c_CommunicationChannel_send_CONTROL_SYSTEM_STATE(&CommunicationChannel_instance.channel, pcontrol_system_state)) {
			free_pack(pcontrol_system_state);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *pset_position_target_global_int = c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT(cur);
		write_SET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int);
		if (!c_CommunicationChannel_send_SET_POSITION_TARGET_GLOBAL_INT(&CommunicationChannel_instance.channel, pset_position_target_global_int)) {
			free_pack(unwrap_pack(pset_position_target_global_int));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		VIBRATION_vibration *pvibration = c_CommunicationChannel_new_VIBRATION();
		write_VIBRATION(pvibration_VIBRATION_from(pvibration));
		if (!c_CommunicationChannel_send_VIBRATION(&CommunicationChannel_instance.channel, pvibration)) {
			free_pack(pvibration);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pping33_PING33 *pping33 = c_CommunicationChannel_new_PING33(cur);
		write_PING33(pping33);
		if (!c_CommunicationChannel_send_PING33(&CommunicationChannel_instance.channel, pping33)) {
			free_pack(unwrap_pack(pping33));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		VFR_HUD_vfr_hud *pvfr_hud = c_CommunicationChannel_new_VFR_HUD();
		write_VFR_HUD(pvfr_hud_VFR_HUD_from(pvfr_hud));
		if (!c_CommunicationChannel_send_VFR_HUD(&CommunicationChannel_instance.channel, pvfr_hud)) {
			free_pack(pvfr_hud);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		MISSION_SET_CURRENT_mission_set_current *pmission_set_current = c_CommunicationChannel_new_MISSION_SET_CURRENT();
		write_MISSION_SET_CURRENT(pmission_set_current_MISSION_SET_CURRENT_from(pmission_set_current));
		if (!c_CommunicationChannel_send_MISSION_SET_CURRENT(&CommunicationChannel_instance.channel, pmission_set_current)) {
			free_pack(pmission_set_current);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		HIL_GPS_hil_gps *phil_gps = c_CommunicationChannel_new_HIL_GPS();
		write_HIL_GPS(phil_gps_HIL_GPS_from(phil_gps));
		if (!c_CommunicationChannel_send_HIL_GPS(&CommunicationChannel_instance.channel, phil_gps)) {
			free_pack(phil_gps);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		NAV_CONTROLLER_OUTPUT_nav_controller_output *pnav_controller_output = c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT();
		write_NAV_CONTROLLER_OUTPUT(pnav_controller_output_NAV_CONTROLLER_OUTPUT_from(pnav_controller_output));
		if (!c_CommunicationChannel_send_NAV_CONTROLLER_OUTPUT(&CommunicationChannel_instance.channel, pnav_controller_output)) {
			free_pack(pnav_controller_output);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pauth_key_AUTH_KEY *pauth_key = c_CommunicationChannel_new_AUTH_KEY(cur);
		write_AUTH_KEY(pauth_key);
		if (!c_CommunicationChannel_send_AUTH_KEY(&CommunicationChannel_instance.channel, pauth_key)) {
			free_pack(unwrap_pack(pauth_key));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		plocal_position_ned_cov_LOCAL_POSITION_NED_COV *plocal_position_ned_cov = c_CommunicationChannel_new_LOCAL_POSITION_NED_COV(cur);
		write_LOCAL_POSITION_NED_COV(plocal_position_ned_cov);
		if (!c_CommunicationChannel_send_LOCAL_POSITION_NED_COV(&CommunicationChannel_instance.channel, plocal_position_ned_cov)) {
			free_pack(unwrap_pack(plocal_position_ned_cov));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		ATT_POS_MOCAP_att_pos_mocap *patt_pos_mocap = c_CommunicationChannel_new_ATT_POS_MOCAP();
		write_ATT_POS_MOCAP(patt_pos_mocap_ATT_POS_MOCAP_from(patt_pos_mocap));
		if (!c_CommunicationChannel_send_ATT_POS_MOCAP(&CommunicationChannel_instance.channel, patt_pos_mocap)) {
			free_pack(patt_pos_mocap);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pstatustext_STATUSTEXT *pstatustext = c_CommunicationChannel_new_STATUSTEXT(cur);
		write_STATUSTEXT(pstatustext);
		if (!c_CommunicationChannel_send_STATUSTEXT(&CommunicationChannel_instance.channel, pstatustext)) {
			free_pack(unwrap_pack(pstatustext));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		PING_ping *pping = c_CommunicationChannel_new_PING();
		write_PING(pping_PING_from(pping));
		if (!c_CommunicationChannel_send_PING(&CommunicationChannel_instance.channel, pping)) {
			free_pack(pping);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		CAMERA_CAPTURE_STATUS_camera_capture_status *pcamera_capture_status = c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS();
		write_CAMERA_CAPTURE_STATUS(pcamera_capture_status_CAMERA_CAPTURE_STATUS_from(pcamera_capture_status));
		if (!c_CommunicationChannel_send_CAMERA_CAPTURE_STATUS(&CommunicationChannel_instance.channel, pcamera_capture_status)) {
			free_pack(pcamera_capture_status);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		GLOBAL_POSITION_INT_global_position_int *pglobal_position_int = c_CommunicationChannel_new_GLOBAL_POSITION_INT();
		write_GLOBAL_POSITION_INT(pglobal_position_int_GLOBAL_POSITION_INT_from(pglobal_position_int));
		if (!c_CommunicationChannel_send_GLOBAL_POSITION_INT(&CommunicationChannel_instance.channel, pglobal_position_int)) {
			free_pack(pglobal_position_int);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		ENCAPSULATED_DATA_encapsulated_data *pencapsulated_data = c_CommunicationChannel_new_ENCAPSULATED_DATA();
		write_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA_from(pencapsulated_data));
		if (!c_CommunicationChannel_send_ENCAPSULATED_DATA(&CommunicationChannel_instance.channel, pencapsulated_data)) {
			free_pack(pencapsulated_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pgps_input_GPS_INPUT *pgps_input = c_CommunicationChannel_new_GPS_INPUT(cur);
		write_GPS_INPUT(pgps_input);
		if (!c_CommunicationChannel_send_GPS_INPUT(&CommunicationChannel_instance.channel, pgps_input)) {
			free_pack(unwrap_pack(pgps_input));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pcommand_long_COMMAND_LONG *pcommand_long = c_CommunicationChannel_new_COMMAND_LONG(cur);
		write_COMMAND_LONG(pcommand_long);
		if (!c_CommunicationChannel_send_COMMAND_LONG(&CommunicationChannel_instance.channel, pcommand_long)) {
			free_pack(unwrap_pack(pcommand_long));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		LOG_REQUEST_DATA_log_request_data *plog_request_data = c_CommunicationChannel_new_LOG_REQUEST_DATA();
		write_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA_from(plog_request_data));
		if (!c_CommunicationChannel_send_LOG_REQUEST_DATA(&CommunicationChannel_instance.channel, plog_request_data)) {
			free_pack(plog_request_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pgps_raw_int_GPS_RAW_INT *pgps_raw_int = c_CommunicationChannel_new_GPS_RAW_INT(cur);
		write_GPS_RAW_INT(pgps_raw_int);
		if (!c_CommunicationChannel_send_GPS_RAW_INT(&CommunicationChannel_instance.channel, pgps_raw_int)) {
			free_pack(unwrap_pack(pgps_raw_int));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		RC_CHANNELS_SCALED_rc_channels_scaled *prc_channels_scaled = c_CommunicationChannel_new_RC_CHANNELS_SCALED();
		write_RC_CHANNELS_SCALED(prc_channels_scaled_RC_CHANNELS_SCALED_from(prc_channels_scaled));
		if (!c_CommunicationChannel_send_RC_CHANNELS_SCALED(&CommunicationChannel_instance.channel, prc_channels_scaled)) {
			free_pack(prc_channels_scaled);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pcamera_settings_CAMERA_SETTINGS *pcamera_settings = c_CommunicationChannel_new_CAMERA_SETTINGS(cur);
		write_CAMERA_SETTINGS(pcamera_settings);
		if (!c_CommunicationChannel_send_CAMERA_SETTINGS(&CommunicationChannel_instance.channel, pcamera_settings)) {
			free_pack(unwrap_pack(pcamera_settings));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		RAW_PRESSURE_raw_pressure *praw_pressure = c_CommunicationChannel_new_RAW_PRESSURE();
		write_RAW_PRESSURE(praw_pressure_RAW_PRESSURE_from(praw_pressure));
		if (!c_CommunicationChannel_send_RAW_PRESSURE(&CommunicationChannel_instance.channel, praw_pressure)) {
			free_pack(praw_pressure);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pnamed_value_float_NAMED_VALUE_FLOAT *pnamed_value_float = c_CommunicationChannel_new_NAMED_VALUE_FLOAT(cur);
		write_NAMED_VALUE_FLOAT(pnamed_value_float);
		if (!c_CommunicationChannel_send_NAMED_VALUE_FLOAT(&CommunicationChannel_instance.channel, pnamed_value_float)) {
			free_pack(unwrap_pack(pnamed_value_float));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		ATTITUDE_attitude *pattitude = c_CommunicationChannel_new_ATTITUDE();
		write_ATTITUDE(pattitude_ATTITUDE_from(pattitude));
		if (!c_CommunicationChannel_send_ATTITUDE(&CommunicationChannel_instance.channel, pattitude)) {
			free_pack(pattitude);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		TERRAIN_REQUEST_terrain_request *pterrain_request = c_CommunicationChannel_new_TERRAIN_REQUEST();
		write_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST_from(pterrain_request));
		if (!c_CommunicationChannel_send_TERRAIN_REQUEST(&CommunicationChannel_instance.channel, pterrain_request)) {
			free_pack(pterrain_request);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *pmission_write_partial_list = c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST(cur);
		write_MISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list);
		if (!c_CommunicationChannel_send_MISSION_WRITE_PARTIAL_LIST(&CommunicationChannel_instance.channel, pmission_write_partial_list)) {
			free_pack(unwrap_pack(pmission_write_partial_list));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		LOG_ERASE_log_erase *plog_erase = c_CommunicationChannel_new_LOG_ERASE();
		write_LOG_ERASE(plog_erase_LOG_ERASE_from(plog_erase));
		if (!c_CommunicationChannel_send_LOG_ERASE(&CommunicationChannel_instance.channel, plog_erase)) {
			free_pack(plog_erase);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		MANUAL_SETPOINT_manual_setpoint *pmanual_setpoint = c_CommunicationChannel_new_MANUAL_SETPOINT();
		write_MANUAL_SETPOINT(pmanual_setpoint_MANUAL_SETPOINT_from(pmanual_setpoint));
		if (!c_CommunicationChannel_send_MANUAL_SETPOINT(&CommunicationChannel_instance.channel, pmanual_setpoint)) {
			free_pack(pmanual_setpoint);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		psafety_allowed_area_SAFETY_ALLOWED_AREA *psafety_allowed_area = c_CommunicationChannel_new_SAFETY_ALLOWED_AREA(cur);
		write_SAFETY_ALLOWED_AREA(psafety_allowed_area);
		if (!c_CommunicationChannel_send_SAFETY_ALLOWED_AREA(&CommunicationChannel_instance.channel, psafety_allowed_area)) {
			free_pack(unwrap_pack(psafety_allowed_area));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		OPTICAL_FLOW_RAD_optical_flow_rad *poptical_flow_rad = c_CommunicationChannel_new_OPTICAL_FLOW_RAD();
		write_OPTICAL_FLOW_RAD(poptical_flow_rad_OPTICAL_FLOW_RAD_from(poptical_flow_rad));
		if (!c_CommunicationChannel_send_OPTICAL_FLOW_RAD(&CommunicationChannel_instance.channel, poptical_flow_rad)) {
			free_pack(poptical_flow_rad);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		LOG_DATA_log_data *plog_data = c_CommunicationChannel_new_LOG_DATA();
		write_LOG_DATA(plog_data_LOG_DATA_from(plog_data));
		if (!c_CommunicationChannel_send_LOG_DATA(&CommunicationChannel_instance.channel, plog_data)) {
			free_pack(plog_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmission_clear_all_MISSION_CLEAR_ALL *pmission_clear_all = c_CommunicationChannel_new_MISSION_CLEAR_ALL(cur);
		write_MISSION_CLEAR_ALL(pmission_clear_all);
		if (!c_CommunicationChannel_send_MISSION_CLEAR_ALL(&CommunicationChannel_instance.channel, pmission_clear_all)) {
			free_pack(unwrap_pack(pmission_clear_all));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		VICON_POSITION_ESTIMATE_vicon_position_estimate *pvicon_position_estimate = c_CommunicationChannel_new_VICON_POSITION_ESTIMATE();
		write_VICON_POSITION_ESTIMATE(pvicon_position_estimate_VICON_POSITION_ESTIMATE_from(pvicon_position_estimate));
		if (!c_CommunicationChannel_send_VICON_POSITION_ESTIMATE(&CommunicationChannel_instance.channel, pvicon_position_estimate)) {
			free_pack(pvicon_position_estimate);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		GPS2_RTK_gps2_rtk *pgps2_rtk = c_CommunicationChannel_new_GPS2_RTK();
		write_GPS2_RTK(pgps2_rtk_GPS2_RTK_from(pgps2_rtk));
		if (!c_CommunicationChannel_send_GPS2_RTK(&CommunicationChannel_instance.channel, pgps2_rtk)) {
			free_pack(pgps2_rtk);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		LOG_REQUEST_LIST_log_request_list *plog_request_list = c_CommunicationChannel_new_LOG_REQUEST_LIST();
		write_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST_from(plog_request_list));
		if (!c_CommunicationChannel_send_LOG_REQUEST_LIST(&CommunicationChannel_instance.channel, plog_request_list)) {
			free_pack(plog_request_list);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SCALED_PRESSURE_scaled_pressure *pscaled_pressure = c_CommunicationChannel_new_SCALED_PRESSURE();
		write_SCALED_PRESSURE(pscaled_pressure_SCALED_PRESSURE_from(pscaled_pressure));
		if (!c_CommunicationChannel_send_SCALED_PRESSURE(&CommunicationChannel_instance.channel, pscaled_pressure)) {
			free_pack(pscaled_pressure);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmission_request_int_MISSION_REQUEST_INT *pmission_request_int = c_CommunicationChannel_new_MISSION_REQUEST_INT(cur);
		write_MISSION_REQUEST_INT(pmission_request_int);
		if (!c_CommunicationChannel_send_MISSION_REQUEST_INT(&CommunicationChannel_instance.channel, pmission_request_int)) {
			free_pack(unwrap_pack(pmission_request_int));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		V2_EXTENSION_v2_extension *pv2_extension = c_CommunicationChannel_new_V2_EXTENSION();
		write_V2_EXTENSION(pv2_extension_V2_EXTENSION_from(pv2_extension));
		if (!c_CommunicationChannel_send_V2_EXTENSION(&CommunicationChannel_instance.channel, pv2_extension)) {
			free_pack(pv2_extension);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pheartbeat_HEARTBEAT *pheartbeat = c_CommunicationChannel_new_HEARTBEAT(cur);
		write_HEARTBEAT(pheartbeat);
		if (!c_CommunicationChannel_send_HEARTBEAT(&CommunicationChannel_instance.channel, pheartbeat)) {
			free_pack(unwrap_pack(pheartbeat));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pparam_map_rc_PARAM_MAP_RC *pparam_map_rc = c_CommunicationChannel_new_PARAM_MAP_RC(cur);
		write_PARAM_MAP_RC(pparam_map_rc);
		if (!c_CommunicationChannel_send_PARAM_MAP_RC(&CommunicationChannel_instance.channel, pparam_map_rc)) {
			free_pack(unwrap_pack(pparam_map_rc));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		ppower_status_POWER_STATUS *ppower_status = c_CommunicationChannel_new_POWER_STATUS(cur);
		write_POWER_STATUS(ppower_status);
		if (!c_CommunicationChannel_send_POWER_STATUS(&CommunicationChannel_instance.channel, ppower_status)) {
			free_pack(unwrap_pack(ppower_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		TERRAIN_CHECK_terrain_check *pterrain_check = c_CommunicationChannel_new_TERRAIN_CHECK();
		write_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK_from(pterrain_check));
		if (!c_CommunicationChannel_send_TERRAIN_CHECK(&CommunicationChannel_instance.channel, pterrain_check)) {
			free_pack(pterrain_check);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_local_position_ned_system_global_offset *plocal_position_ned_system_global_offset = c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		write_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_from(plocal_position_ned_system_global_offset));
		if (!c_CommunicationChannel_send_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(&CommunicationChannel_instance.channel, plocal_position_ned_system_global_offset)) {
			free_pack(plocal_position_ned_system_global_offset);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pcommand_ack_COMMAND_ACK *pcommand_ack = c_CommunicationChannel_new_COMMAND_ACK(cur);
		write_COMMAND_ACK(pcommand_ack);
		if (!c_CommunicationChannel_send_COMMAND_ACK(&CommunicationChannel_instance.channel, pcommand_ack)) {
			free_pack(unwrap_pack(pcommand_ack));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		DATA_STREAM_data_stream *pdata_stream = c_CommunicationChannel_new_DATA_STREAM();
		write_DATA_STREAM(pdata_stream_DATA_STREAM_from(pdata_stream));
		if (!c_CommunicationChannel_send_DATA_STREAM(&CommunicationChannel_instance.channel, pdata_stream)) {
			free_pack(pdata_stream);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmission_request_MISSION_REQUEST *pmission_request = c_CommunicationChannel_new_MISSION_REQUEST(cur);
		write_MISSION_REQUEST(pmission_request);
		if (!c_CommunicationChannel_send_MISSION_REQUEST(&CommunicationChannel_instance.channel, pmission_request)) {
			free_pack(unwrap_pack(pmission_request));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		TERRAIN_REPORT_terrain_report *pterrain_report = c_CommunicationChannel_new_TERRAIN_REPORT();
		write_TERRAIN_REPORT(pterrain_report_TERRAIN_REPORT_from(pterrain_report));
		if (!c_CommunicationChannel_send_TERRAIN_REPORT(&CommunicationChannel_instance.channel, pterrain_report)) {
			free_pack(pterrain_report);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pset_home_position_SET_HOME_POSITION *pset_home_position = c_CommunicationChannel_new_SET_HOME_POSITION(cur);
		write_SET_HOME_POSITION(pset_home_position);
		if (!c_CommunicationChannel_send_SET_HOME_POSITION(&CommunicationChannel_instance.channel, pset_home_position)) {
			free_pack(unwrap_pack(pset_home_position));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		if (!c_CommunicationChannel_send_SwitchModeCommand(&CommunicationChannel_instance.channel)) {
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		HIL_RC_INPUTS_RAW_hil_rc_inputs_raw *phil_rc_inputs_raw = c_CommunicationChannel_new_HIL_RC_INPUTS_RAW();
		write_HIL_RC_INPUTS_RAW(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW_from(phil_rc_inputs_raw));
		if (!c_CommunicationChannel_send_HIL_RC_INPUTS_RAW(&CommunicationChannel_instance.channel, phil_rc_inputs_raw)) {
			free_pack(phil_rc_inputs_raw);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SCALED_IMU3_scaled_imu3 *pscaled_imu3 = c_CommunicationChannel_new_SCALED_IMU3();
		write_SCALED_IMU3(pscaled_imu3_SCALED_IMU3_from(pscaled_imu3));
		if (!c_CommunicationChannel_send_SCALED_IMU3(&CommunicationChannel_instance.channel, pscaled_imu3)) {
			free_pack(pscaled_imu3);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pset_mode_SET_MODE *pset_mode = c_CommunicationChannel_new_SET_MODE(cur);
		write_SET_MODE(pset_mode);
		if (!c_CommunicationChannel_send_SET_MODE(&CommunicationChannel_instance.channel, pset_mode)) {
			free_pack(unwrap_pack(pset_mode));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *pposition_target_global_int = c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT(cur);
		write_POSITION_TARGET_GLOBAL_INT(pposition_target_global_int);
		if (!c_CommunicationChannel_send_POSITION_TARGET_GLOBAL_INT(&CommunicationChannel_instance.channel, pposition_target_global_int)) {
			free_pack(unwrap_pack(pposition_target_global_int));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		FLIGHT_INFORMATION_flight_information *pflight_information = c_CommunicationChannel_new_FLIGHT_INFORMATION();
		write_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION_from(pflight_information));
		if (!c_CommunicationChannel_send_FLIGHT_INFORMATION(&CommunicationChannel_instance.channel, pflight_information)) {
			free_pack(pflight_information);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SIM_STATE_sim_state *psim_state = c_CommunicationChannel_new_SIM_STATE();
		write_SIM_STATE(psim_state_SIM_STATE_from(psim_state));
		if (!c_CommunicationChannel_send_SIM_STATE(&CommunicationChannel_instance.channel, psim_state)) {
			free_pack(psim_state);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		MISSION_ITEM_REACHED_mission_item_reached *pmission_item_reached = c_CommunicationChannel_new_MISSION_ITEM_REACHED();
		write_MISSION_ITEM_REACHED(pmission_item_reached_MISSION_ITEM_REACHED_from(pmission_item_reached));
		if (!c_CommunicationChannel_send_MISSION_ITEM_REACHED(&CommunicationChannel_instance.channel, pmission_item_reached)) {
			free_pack(pmission_item_reached);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		RC_CHANNELS_RAW_rc_channels_raw *prc_channels_raw = c_CommunicationChannel_new_RC_CHANNELS_RAW();
		write_RC_CHANNELS_RAW(prc_channels_raw_RC_CHANNELS_RAW_from(prc_channels_raw));
		if (!c_CommunicationChannel_send_RC_CHANNELS_RAW(&CommunicationChannel_instance.channel, prc_channels_raw)) {
			free_pack(prc_channels_raw);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pservo_output_raw_SERVO_OUTPUT_RAW *pservo_output_raw = c_CommunicationChannel_new_SERVO_OUTPUT_RAW(cur);
		write_SERVO_OUTPUT_RAW(pservo_output_raw);
		if (!c_CommunicationChannel_send_SERVO_OUTPUT_RAW(&CommunicationChannel_instance.channel, pservo_output_raw)) {
			free_pack(unwrap_pack(pservo_output_raw));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		VISION_SPEED_ESTIMATE_vision_speed_estimate *pvision_speed_estimate = c_CommunicationChannel_new_VISION_SPEED_ESTIMATE();
		write_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE_from(pvision_speed_estimate));
		if (!c_CommunicationChannel_send_VISION_SPEED_ESTIMATE(&CommunicationChannel_instance.channel, pvision_speed_estimate)) {
			free_pack(pvision_speed_estimate);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pdebug_vect_DEBUG_VECT *pdebug_vect = c_CommunicationChannel_new_DEBUG_VECT(cur);
		write_DEBUG_VECT(pdebug_vect);
		if (!c_CommunicationChannel_send_DEBUG_VECT(&CommunicationChannel_instance.channel, pdebug_vect)) {
			free_pack(unwrap_pack(pdebug_vect));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		LOG_REQUEST_END_log_request_end *plog_request_end = c_CommunicationChannel_new_LOG_REQUEST_END();
		write_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END_from(plog_request_end));
		if (!c_CommunicationChannel_send_LOG_REQUEST_END(&CommunicationChannel_instance.channel, plog_request_end)) {
			free_pack(plog_request_end);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmission_ack_MISSION_ACK *pmission_ack = c_CommunicationChannel_new_MISSION_ACK(cur);
		write_MISSION_ACK(pmission_ack);
		if (!c_CommunicationChannel_send_MISSION_ACK(&CommunicationChannel_instance.channel, pmission_ack)) {
			free_pack(unwrap_pack(pmission_ack));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		CHANGE_OPERATOR_CONTROL_ACK_change_operator_control_ack *pchange_operator_control_ack = c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK();
		write_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK_from(pchange_operator_control_ack));
		if (!c_CommunicationChannel_send_CHANGE_OPERATOR_CONTROL_ACK(&CommunicationChannel_instance.channel, pchange_operator_control_ack)) {
			free_pack(pchange_operator_control_ack);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		MISSION_CURRENT_mission_current *pmission_current = c_CommunicationChannel_new_MISSION_CURRENT();
		write_MISSION_CURRENT(pmission_current_MISSION_CURRENT_from(pmission_current));
		if (!c_CommunicationChannel_send_MISSION_CURRENT(&CommunicationChannel_instance.channel, pmission_current)) {
			free_pack(pmission_current);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SYSTEM_TIME_system_time *psystem_time = c_CommunicationChannel_new_SYSTEM_TIME();
		write_SYSTEM_TIME(psystem_time_SYSTEM_TIME_from(psystem_time));
		if (!c_CommunicationChannel_send_SYSTEM_TIME(&CommunicationChannel_instance.channel, psystem_time)) {
			free_pack(psystem_time);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		CAMERA_TRIGGER_camera_trigger *pcamera_trigger = c_CommunicationChannel_new_CAMERA_TRIGGER();
		write_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER_from(pcamera_trigger));
		if (!c_CommunicationChannel_send_CAMERA_TRIGGER(&CommunicationChannel_instance.channel, pcamera_trigger)) {
			free_pack(pcamera_trigger);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		VISION_POSITION_ESTIMATE_vision_position_estimate *pvision_position_estimate = c_CommunicationChannel_new_VISION_POSITION_ESTIMATE();
		write_VISION_POSITION_ESTIMATE(pvision_position_estimate_VISION_POSITION_ESTIMATE_from(pvision_position_estimate));
		if (!c_CommunicationChannel_send_VISION_POSITION_ESTIMATE(&CommunicationChannel_instance.channel, pvision_position_estimate)) {
			free_pack(pvision_position_estimate);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		MANUAL_CONTROL_manual_control *pmanual_control = c_CommunicationChannel_new_MANUAL_CONTROL();
		write_MANUAL_CONTROL(pmanual_control_MANUAL_CONTROL_from(pmanual_control));
		if (!c_CommunicationChannel_send_MANUAL_CONTROL(&CommunicationChannel_instance.channel, pmanual_control)) {
			free_pack(pmanual_control);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		RC_CHANNELS_rc_channels *prc_channels = c_CommunicationChannel_new_RC_CHANNELS();
		write_RC_CHANNELS(prc_channels_RC_CHANNELS_from(prc_channels));
		if (!c_CommunicationChannel_send_RC_CHANNELS(&CommunicationChannel_instance.channel, prc_channels)) {
			free_pack(prc_channels);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pparam_value_PARAM_VALUE *pparam_value = c_CommunicationChannel_new_PARAM_VALUE(cur);
		write_PARAM_VALUE(pparam_value);
		if (!c_CommunicationChannel_send_PARAM_VALUE(&CommunicationChannel_instance.channel, pparam_value)) {
			free_pack(unwrap_pack(pparam_value));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pbattery_status_BATTERY_STATUS *pbattery_status = c_CommunicationChannel_new_BATTERY_STATUS(cur);
		write_BATTERY_STATUS(pbattery_status);
		if (!c_CommunicationChannel_send_BATTERY_STATUS(&CommunicationChannel_instance.channel, pbattery_status)) {
			free_pack(unwrap_pack(pbattery_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *pset_position_target_local_ned = c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED(cur);
		write_SET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned);
		if (!c_CommunicationChannel_send_SET_POSITION_TARGET_LOCAL_NED(&CommunicationChannel_instance.channel, pset_position_target_local_ned)) {
			free_pack(unwrap_pack(pset_position_target_local_ned));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pserial_control_SERIAL_CONTROL *pserial_control = c_CommunicationChannel_new_SERIAL_CONTROL(cur);
		write_SERIAL_CONTROL(pserial_control);
		if (!c_CommunicationChannel_send_SERIAL_CONTROL(&CommunicationChannel_instance.channel, pserial_control)) {
			free_pack(unwrap_pack(pserial_control));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *pset_gps_global_origin = c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN(cur);
		write_SET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin);
		if (!c_CommunicationChannel_send_SET_GPS_GLOBAL_ORIGIN(&CommunicationChannel_instance.channel, pset_gps_global_origin)) {
			free_pack(unwrap_pack(pset_gps_global_origin));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pautopilot_version_AUTOPILOT_VERSION *pautopilot_version = c_CommunicationChannel_new_AUTOPILOT_VERSION(cur);
		write_AUTOPILOT_VERSION(pautopilot_version);
		if (!c_CommunicationChannel_send_AUTOPILOT_VERSION(&CommunicationChannel_instance.channel, pautopilot_version)) {
			free_pack(unwrap_pack(pautopilot_version));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pmission_request_list_MISSION_REQUEST_LIST *pmission_request_list = c_CommunicationChannel_new_MISSION_REQUEST_LIST(cur);
		write_MISSION_REQUEST_LIST(pmission_request_list);
		if (!c_CommunicationChannel_send_MISSION_REQUEST_LIST(&CommunicationChannel_instance.channel, pmission_request_list)) {
			free_pack(unwrap_pack(pmission_request_list));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pplay_tune_PLAY_TUNE *pplay_tune = c_CommunicationChannel_new_PLAY_TUNE(cur);
		write_PLAY_TUNE(pplay_tune);
		if (!c_CommunicationChannel_send_PLAY_TUNE(&CommunicationChannel_instance.channel, pplay_tune)) {
			free_pack(unwrap_pack(pplay_tune));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		SCALED_PRESSURE3_scaled_pressure3 *pscaled_pressure3 = c_CommunicationChannel_new_SCALED_PRESSURE3();
		write_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3_from(pscaled_pressure3));
		if (!c_CommunicationChannel_send_SCALED_PRESSURE3(&CommunicationChannel_instance.channel, pscaled_pressure3)) {
			free_pack(pscaled_pressure3);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *pmission_request_partial_list = c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST(cur);
		write_MISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list);
		if (!c_CommunicationChannel_send_MISSION_REQUEST_PARTIAL_LIST(&CommunicationChannel_instance.channel, pmission_request_partial_list)) {
			free_pack(unwrap_pack(pmission_request_partial_list));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		LOCAL_POSITION_NED_local_position_ned *plocal_position_ned = c_CommunicationChannel_new_LOCAL_POSITION_NED();
		write_LOCAL_POSITION_NED(plocal_position_ned_LOCAL_POSITION_NED_from(plocal_position_ned));
		if (!c_CommunicationChannel_send_LOCAL_POSITION_NED(&CommunicationChannel_instance.channel, plocal_position_ned)) {
			free_pack(plocal_position_ned);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake *pdata_transmission_handshake = c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE();
		write_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE_from(pdata_transmission_handshake));
		if (!c_CommunicationChannel_send_DATA_TRANSMISSION_HANDSHAKE(&CommunicationChannel_instance.channel, pdata_transmission_handshake)) {
			free_pack(pdata_transmission_handshake);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pgps_global_origin_GPS_GLOBAL_ORIGIN *pgps_global_origin = c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN(cur);
		write_GPS_GLOBAL_ORIGIN(pgps_global_origin);
		if (!c_CommunicationChannel_send_GPS_GLOBAL_ORIGIN(&CommunicationChannel_instance.channel, pgps_global_origin)) {
			free_pack(unwrap_pack(pgps_global_origin));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		SCALED_IMU2_scaled_imu2 *pscaled_imu2 = c_CommunicationChannel_new_SCALED_IMU2();
		write_SCALED_IMU2(pscaled_imu2_SCALED_IMU2_from(pscaled_imu2));
		if (!c_CommunicationChannel_send_SCALED_IMU2(&CommunicationChannel_instance.channel, pscaled_imu2)) {
			free_pack(pscaled_imu2);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		ATTITUDE_QUATERNION_attitude_quaternion *pattitude_quaternion = c_CommunicationChannel_new_ATTITUDE_QUATERNION();
		write_ATTITUDE_QUATERNION(pattitude_quaternion_ATTITUDE_QUATERNION_from(pattitude_quaternion));
		if (!c_CommunicationChannel_send_ATTITUDE_QUATERNION(&CommunicationChannel_instance.channel, pattitude_quaternion)) {
			free_pack(pattitude_quaternion);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		phil_actuator_controls_HIL_ACTUATOR_CONTROLS *phil_actuator_controls = c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS(cur);
		write_HIL_ACTUATOR_CONTROLS(phil_actuator_controls);
		if (!c_CommunicationChannel_send_HIL_ACTUATOR_CONTROLS(&CommunicationChannel_instance.channel, phil_actuator_controls)) {
			free_pack(unwrap_pack(phil_actuator_controls));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *pposition_target_local_ned = c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED(cur);
		write_POSITION_TARGET_LOCAL_NED(pposition_target_local_ned);
		if (!c_CommunicationChannel_send_POSITION_TARGET_LOCAL_NED(&CommunicationChannel_instance.channel, pposition_target_local_ned)) {
			free_pack(unwrap_pack(pposition_target_local_ned));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pdistance_sensor_DISTANCE_SENSOR *pdistance_sensor = c_CommunicationChannel_new_DISTANCE_SENSOR(cur);
		write_DISTANCE_SENSOR(pdistance_sensor);
		if (!c_CommunicationChannel_send_DISTANCE_SENSOR(&CommunicationChannel_instance.channel, pdistance_sensor)) {
			free_pack(unwrap_pack(pdistance_sensor));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		HIL_OPTICAL_FLOW_hil_optical_flow *phil_optical_flow = c_CommunicationChannel_new_HIL_OPTICAL_FLOW();
		write_HIL_OPTICAL_FLOW(phil_optical_flow_HIL_OPTICAL_FLOW_from(phil_optical_flow));
		if (!c_CommunicationChannel_send_HIL_OPTICAL_FLOW(&CommunicationChannel_instance.channel, phil_optical_flow)) {
			free_pack(phil_optical_flow);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SCALED_PRESSURE2_scaled_pressure2 *pscaled_pressure2 = c_CommunicationChannel_new_SCALED_PRESSURE2();
		write_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2_from(pscaled_pressure2));
		if (!c_CommunicationChannel_send_SCALED_PRESSURE2(&CommunicationChannel_instance.channel, pscaled_pressure2)) {
			free_pack(pscaled_pressure2);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		WIND_COV_wind_cov *pwind_cov = c_CommunicationChannel_new_WIND_COV();
		write_WIND_COV(pwind_cov_WIND_COV_from(pwind_cov));
		if (!c_CommunicationChannel_send_WIND_COV(&CommunicationChannel_instance.channel, pwind_cov)) {
			free_pack(pwind_cov);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pchange_operator_control_CHANGE_OPERATOR_CONTROL *pchange_operator_control = c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL(cur);
		write_CHANGE_OPERATOR_CONTROL(pchange_operator_control);
		if (!c_CommunicationChannel_send_CHANGE_OPERATOR_CONTROL(&CommunicationChannel_instance.channel, pchange_operator_control)) {
			free_pack(unwrap_pack(pchange_operator_control));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		psys_status_SYS_STATUS *psys_status = c_CommunicationChannel_new_SYS_STATUS(cur);
		write_SYS_STATUS(psys_status);
		if (!c_CommunicationChannel_send_SYS_STATUS(&CommunicationChannel_instance.channel, psys_status)) {
			free_pack(unwrap_pack(psys_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pmission_item_MISSION_ITEM *pmission_item = c_CommunicationChannel_new_MISSION_ITEM(cur);
		write_MISSION_ITEM(pmission_item);
		if (!c_CommunicationChannel_send_MISSION_ITEM(&CommunicationChannel_instance.channel, pmission_item)) {
			free_pack(unwrap_pack(pmission_item));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		RAW_IMU_raw_imu *praw_imu = c_CommunicationChannel_new_RAW_IMU();
		write_RAW_IMU(praw_imu_RAW_IMU_from(praw_imu));
		if (!c_CommunicationChannel_send_RAW_IMU(&CommunicationChannel_instance.channel, praw_imu)) {
			free_pack(praw_imu);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pcommand_int_COMMAND_INT *pcommand_int = c_CommunicationChannel_new_COMMAND_INT(cur);
		write_COMMAND_INT(pcommand_int);
		if (!c_CommunicationChannel_send_COMMAND_INT(&CommunicationChannel_instance.channel, pcommand_int)) {
			free_pack(unwrap_pack(pcommand_int));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		poptical_flow_OPTICAL_FLOW *poptical_flow = c_CommunicationChannel_new_OPTICAL_FLOW(cur);
		write_OPTICAL_FLOW(poptical_flow);
		if (!c_CommunicationChannel_send_OPTICAL_FLOW(&CommunicationChannel_instance.channel, poptical_flow)) {
			free_pack(unwrap_pack(poptical_flow));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pmission_item_int_MISSION_ITEM_INT *pmission_item_int = c_CommunicationChannel_new_MISSION_ITEM_INT(cur);
		write_MISSION_ITEM_INT(pmission_item_int);
		if (!c_CommunicationChannel_send_MISSION_ITEM_INT(&CommunicationChannel_instance.channel, pmission_item_int)) {
			free_pack(unwrap_pack(pmission_item_int));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		HIGHRES_IMU_highres_imu *phighres_imu = c_CommunicationChannel_new_HIGHRES_IMU();
		write_HIGHRES_IMU(phighres_imu_HIGHRES_IMU_from(phighres_imu));
		if (!c_CommunicationChannel_send_HIGHRES_IMU(&CommunicationChannel_instance.channel, phighres_imu)) {
			free_pack(phighres_imu);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pextended_sys_state_EXTENDED_SYS_STATE *pextended_sys_state = c_CommunicationChannel_new_EXTENDED_SYS_STATE(cur);
		write_EXTENDED_SYS_STATE(pextended_sys_state);
		if (!c_CommunicationChannel_send_EXTENDED_SYS_STATE(&CommunicationChannel_instance.channel, pextended_sys_state)) {
			free_pack(unwrap_pack(pextended_sys_state));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		GPS_INJECT_DATA_gps_inject_data *pgps_inject_data = c_CommunicationChannel_new_GPS_INJECT_DATA();
		write_GPS_INJECT_DATA(pgps_inject_data_GPS_INJECT_DATA_from(pgps_inject_data));
		if (!c_CommunicationChannel_send_GPS_INJECT_DATA(&CommunicationChannel_instance.channel, pgps_inject_data)) {
			free_pack(pgps_inject_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		ATTITUDE_QUATERNION_COV_attitude_quaternion_cov *pattitude_quaternion_cov = c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV();
		write_ATTITUDE_QUATERNION_COV(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV_from(pattitude_quaternion_cov));
		if (!c_CommunicationChannel_send_ATTITUDE_QUATERNION_COV(&CommunicationChannel_instance.channel, pattitude_quaternion_cov)) {
			free_pack(pattitude_quaternion_cov);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pnamed_value_int_NAMED_VALUE_INT *pnamed_value_int = c_CommunicationChannel_new_NAMED_VALUE_INT(cur);
		write_NAMED_VALUE_INT(pnamed_value_int);
		if (!c_CommunicationChannel_send_NAMED_VALUE_INT(&CommunicationChannel_instance.channel, pnamed_value_int)) {
			free_pack(unwrap_pack(pnamed_value_int));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		RADIO_STATUS_radio_status *pradio_status = c_CommunicationChannel_new_RADIO_STATUS();
		write_RADIO_STATUS(pradio_status_RADIO_STATUS_from(pradio_status));
		if (!c_CommunicationChannel_send_RADIO_STATUS(&CommunicationChannel_instance.channel, pradio_status)) {
			free_pack(pradio_status);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		GPS_RTCM_DATA_gps_rtcm_data *pgps_rtcm_data = c_CommunicationChannel_new_GPS_RTCM_DATA();
		write_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA_from(pgps_rtcm_data));
		if (!c_CommunicationChannel_send_GPS_RTCM_DATA(&CommunicationChannel_instance.channel, pgps_rtcm_data)) {
			free_pack(pgps_rtcm_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		GLOBAL_VISION_POSITION_ESTIMATE_global_vision_position_estimate *pglobal_vision_position_estimate = c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE();
		write_GLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE_from(pglobal_vision_position_estimate));
		if (!c_CommunicationChannel_send_GLOBAL_VISION_POSITION_ESTIMATE(&CommunicationChannel_instance.channel, pglobal_vision_position_estimate)) {
			free_pack(pglobal_vision_position_estimate);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		FILE_TRANSFER_PROTOCOL_file_transfer_protocol *pfile_transfer_protocol = c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL();
		write_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL_from(pfile_transfer_protocol));
		if (!c_CommunicationChannel_send_FILE_TRANSFER_PROTOCOL(&CommunicationChannel_instance.channel, pfile_transfer_protocol)) {
			free_pack(pfile_transfer_protocol);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
	}
	
	
	
	
	
	// c_CommunicationChannel_packs_into_bytes(uint8_t* dst, size_t bytes) for sending out packs
	
	//c_CommunicationChannel_bytes_into_packs(uint8_t* src, size_t bytes) for receiving packs
	
	
	
	return 0;
}
					  