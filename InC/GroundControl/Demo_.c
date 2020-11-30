
#include "GroundControl.h"
#include <assert.h>
#include <stddef.h>

static float   some_float   = 0;
static bool    some_bool    = true;
static int32_t some_int32_t = 0;
static size_t  some_size_t  = 0;
static char    *some_string = "NULL";
static int8_t  some_int8_t  = 0;
static int64_t some_int64_t = 0;
static int16_t some_int16_t = 0;


void write_RESOURCE_REQUEST(presource_request_RESOURCE_REQUEST *const presource_request);

void read_RESOURCE_REQUEST(presource_request_RESOURCE_REQUEST *const presource_request);

void write_FENCE_POINT(pfence_point_FENCE_POINT *const pfence_point);

void read_FENCE_POINT(pfence_point_FENCE_POINT *const pfence_point);

void write_RADIO_STATUS(pradio_status_RADIO_STATUS *const pradio_status);

void read_RADIO_STATUS(pradio_status_RADIO_STATUS *const pradio_status);

void write_RANGEFINDER(prangefinder_RANGEFINDER *const prangefinder);

void read_RANGEFINDER(prangefinder_RANGEFINDER *const prangefinder);

void write_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *const pfile_transfer_protocol);

void read_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *const pfile_transfer_protocol);

void read_GLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate_GLOBAL_VISION_POSITION_ESTIMATE *const pglobal_vision_position_estimate);

void write_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA *const pgps_rtcm_data);

void read_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA *const pgps_rtcm_data);

void write_RPM(prpm_RPM *const prpm);

void read_RPM(prpm_RPM *const prpm);

void write_NAMED_VALUE_INT(pnamed_value_int_NAMED_VALUE_INT *const pnamed_value_int);

void read_NAMED_VALUE_INT(pnamed_value_int_NAMED_VALUE_INT *const pnamed_value_int);

void read_ATTITUDE_QUATERNION_COV(pattitude_quaternion_cov_ATTITUDE_QUATERNION_COV *const pattitude_quaternion_cov);

void write_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *const puavionix_adsb_transceiver_health_report);

void read_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *const puavionix_adsb_transceiver_health_report);

void write_GPS_INJECT_DATA(pgps_inject_data_GPS_INJECT_DATA *const pgps_inject_data);

void read_GPS_INJECT_DATA(pgps_inject_data_GPS_INJECT_DATA *const pgps_inject_data);

void write_GOPRO_GET_RESPONSE(pgopro_get_response_GOPRO_GET_RESPONSE *const pgopro_get_response);

void read_GOPRO_GET_RESPONSE(pgopro_get_response_GOPRO_GET_RESPONSE *const pgopro_get_response);

void write_UAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const puavionix_adsb_out_dynamic);

void read_UAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const puavionix_adsb_out_dynamic);

void write_EXTENDED_SYS_STATE(pextended_sys_state_EXTENDED_SYS_STATE *const pextended_sys_state);

void read_EXTENDED_SYS_STATE(pextended_sys_state_EXTENDED_SYS_STATE *const pextended_sys_state);

void write_HIGHRES_IMU(phighres_imu_HIGHRES_IMU *const phighres_imu);

void read_HIGHRES_IMU(phighres_imu_HIGHRES_IMU *const phighres_imu);

void write_MAG_CAL_PROGRESS(pmag_cal_progress_MAG_CAL_PROGRESS *const pmag_cal_progress);

void read_MAG_CAL_PROGRESS(pmag_cal_progress_MAG_CAL_PROGRESS *const pmag_cal_progress);

void write_DEVICE_OP_READ(pdevice_op_read_DEVICE_OP_READ *const pdevice_op_read);

void read_DEVICE_OP_READ(pdevice_op_read_DEVICE_OP_READ *const pdevice_op_read);

void write_LOGGING_DATA(plogging_data_LOGGING_DATA *const plogging_data);

void read_LOGGING_DATA(plogging_data_LOGGING_DATA *const plogging_data);

void write_VISION_POSITION_DELTA(pvision_position_delta_VISION_POSITION_DELTA *const pvision_position_delta);

void read_VISION_POSITION_DELTA(pvision_position_delta_VISION_POSITION_DELTA *const pvision_position_delta);

void read_MISSION_ITEM_INT(pmission_item_int_MISSION_ITEM_INT *const pmission_item_int);

void read_OPTICAL_FLOW(poptical_flow_OPTICAL_FLOW *const poptical_flow);

void read_COMMAND_INT(pcommand_int_COMMAND_INT *const pcommand_int);

void read_RAW_IMU(praw_imu_RAW_IMU *const praw_imu);

void read_MISSION_ITEM(pmission_item_MISSION_ITEM *const pmission_item);

void read_SYS_STATUS(psys_status_SYS_STATUS *const psys_status);

void write_GOPRO_SET_REQUEST(pgopro_set_request_GOPRO_SET_REQUEST *const pgopro_set_request);

void read_GOPRO_SET_REQUEST(pgopro_set_request_GOPRO_SET_REQUEST *const pgopro_set_request);

void read_CHANGE_OPERATOR_CONTROL(pchange_operator_control_CHANGE_OPERATOR_CONTROL *const pchange_operator_control);

void write_WIND_COV(pwind_cov_WIND_COV *const pwind_cov);

void read_WIND_COV(pwind_cov_WIND_COV *const pwind_cov);

void write_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2 *const pscaled_pressure2);

void read_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2 *const pscaled_pressure2);

void write_HIL_OPTICAL_FLOW(phil_optical_flow_HIL_OPTICAL_FLOW *const phil_optical_flow);

void read_HIL_OPTICAL_FLOW(phil_optical_flow_HIL_OPTICAL_FLOW *const phil_optical_flow);

void write_DISTANCE_SENSOR(pdistance_sensor_DISTANCE_SENSOR *const pdistance_sensor);

void read_DISTANCE_SENSOR(pdistance_sensor_DISTANCE_SENSOR *const pdistance_sensor);

void write_DEVICE_OP_WRITE(pdevice_op_write_DEVICE_OP_WRITE *const pdevice_op_write);

void read_DEVICE_OP_WRITE(pdevice_op_write_DEVICE_OP_WRITE *const pdevice_op_write);

void write_GIMBAL_REPORT(pgimbal_report_GIMBAL_REPORT *const pgimbal_report);

void read_GIMBAL_REPORT(pgimbal_report_GIMBAL_REPORT *const pgimbal_report);

void read_POSITION_TARGET_LOCAL_NED(pposition_target_local_ned_POSITION_TARGET_LOCAL_NED *const pposition_target_local_ned);

void read_HIL_ACTUATOR_CONTROLS(phil_actuator_controls_HIL_ACTUATOR_CONTROLS *const phil_actuator_controls);

void write_DATA64(pdata64_DATA64 *const pdata64);

void read_DATA64(pdata64_DATA64 *const pdata64);

void read_ATTITUDE_QUATERNION(pattitude_quaternion_ATTITUDE_QUATERNION *const pattitude_quaternion);

void write_SCALED_IMU2(pscaled_imu2_SCALED_IMU2 *const pscaled_imu2);

void read_SCALED_IMU2(pscaled_imu2_SCALED_IMU2 *const pscaled_imu2);

void read_GPS_GLOBAL_ORIGIN(pgps_global_origin_GPS_GLOBAL_ORIGIN *const pgps_global_origin);

void write_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *const pdata_transmission_handshake);

void read_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE *const pdata_transmission_handshake);

void read_LOCAL_POSITION_NED(plocal_position_ned_LOCAL_POSITION_NED *const plocal_position_ned);

void write_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *const pautopilot_version_request);

void read_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *const pautopilot_version_request);

void write_WIND(pwind_WIND *const pwind);

void read_WIND(pwind_WIND *const pwind);

void write_AP_ADC(pap_adc_AP_ADC *const pap_adc);

void read_AP_ADC(pap_adc_AP_ADC *const pap_adc);

void write_SET_MAG_OFFSETS(pset_mag_offsets_SET_MAG_OFFSETS *const pset_mag_offsets);

void read_SET_MAG_OFFSETS(pset_mag_offsets_SET_MAG_OFFSETS *const pset_mag_offsets);

void write_DATA16(pdata16_DATA16 *const pdata16);

void read_DATA16(pdata16_DATA16 *const pdata16);

void write_UAVCAN_NODE_INFO(puavcan_node_info_UAVCAN_NODE_INFO *const puavcan_node_info);

void read_UAVCAN_NODE_INFO(puavcan_node_info_UAVCAN_NODE_INFO *const puavcan_node_info);

void write_PARAM_EXT_ACK(pparam_ext_ack_PARAM_EXT_ACK *const pparam_ext_ack);

void read_PARAM_EXT_ACK(pparam_ext_ack_PARAM_EXT_ACK *const pparam_ext_ack);

void read_MISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list_MISSION_REQUEST_PARTIAL_LIST *const pmission_request_partial_list);

void write_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3 *const pscaled_pressure3);

void read_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3 *const pscaled_pressure3);

void write_DIGICAM_CONFIGURE(pdigicam_configure_DIGICAM_CONFIGURE *const pdigicam_configure);

void read_DIGICAM_CONFIGURE(pdigicam_configure_DIGICAM_CONFIGURE *const pdigicam_configure);

void write_PLAY_TUNE(pplay_tune_PLAY_TUNE *const pplay_tune);

void read_PLAY_TUNE(pplay_tune_PLAY_TUNE *const pplay_tune);

void write_SET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *const pset_video_stream_settings);

void read_SET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *const pset_video_stream_settings);

void write_SIMSTATE(psimstate_SIMSTATE *const psimstate);

void read_SIMSTATE(psimstate_SIMSTATE *const psimstate);

void read_MISSION_REQUEST_LIST(pmission_request_list_MISSION_REQUEST_LIST *const pmission_request_list);

void write_AUTOPILOT_VERSION(pautopilot_version_AUTOPILOT_VERSION *const pautopilot_version);

void read_AUTOPILOT_VERSION(pautopilot_version_AUTOPILOT_VERSION *const pautopilot_version);

void write_PARAM_EXT_SET(pparam_ext_set_PARAM_EXT_SET *const pparam_ext_set);

void read_PARAM_EXT_SET(pparam_ext_set_PARAM_EXT_SET *const pparam_ext_set);

void read_SET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin_SET_GPS_GLOBAL_ORIGIN *const pset_gps_global_origin);

void write_MOUNT_ORIENTATION(pmount_orientation_MOUNT_ORIENTATION *const pmount_orientation);

void read_MOUNT_ORIENTATION(pmount_orientation_MOUNT_ORIENTATION *const pmount_orientation);

void read_SET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned_SET_POSITION_TARGET_LOCAL_NED *const pset_position_target_local_ned);

void write_SERIAL_CONTROL(pserial_control_SERIAL_CONTROL *const pserial_control);

void read_SERIAL_CONTROL(pserial_control_SERIAL_CONTROL *const pserial_control);

void write_BATTERY_STATUS(pbattery_status_BATTERY_STATUS *const pbattery_status);

void read_BATTERY_STATUS(pbattery_status_BATTERY_STATUS *const pbattery_status);

void read_PARAM_VALUE(pparam_value_PARAM_VALUE *const pparam_value);

void write_RALLY_FETCH_POINT(prally_fetch_point_RALLY_FETCH_POINT *const prally_fetch_point);

void read_RALLY_FETCH_POINT(prally_fetch_point_RALLY_FETCH_POINT *const prally_fetch_point);

void write_PROTOCOL_VERSION(pprotocol_version_PROTOCOL_VERSION *const pprotocol_version);

void read_PROTOCOL_VERSION(pprotocol_version_PROTOCOL_VERSION *const pprotocol_version);

void read_RC_CHANNELS(prc_channels_RC_CHANNELS *const prc_channels);

void read_MANUAL_CONTROL(pmanual_control_MANUAL_CONTROL *const pmanual_control);

void read_VISION_POSITION_ESTIMATE(pvision_position_estimate_VISION_POSITION_ESTIMATE *const pvision_position_estimate);

void write_GOPRO_SET_RESPONSE(pgopro_set_response_GOPRO_SET_RESPONSE *const pgopro_set_response);

void read_GOPRO_SET_RESPONSE(pgopro_set_response_GOPRO_SET_RESPONSE *const pgopro_set_response);

void write_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER *const pcamera_trigger);

void read_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER *const pcamera_trigger);

void read_SYSTEM_TIME(psystem_time_SYSTEM_TIME *const psystem_time);

void read_MISSION_CURRENT(pmission_current_MISSION_CURRENT *const pmission_current);

void read_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *const pchange_operator_control_ack);

void read_MISSION_ACK(pmission_ack_MISSION_ACK *const pmission_ack);

void write_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END *const plog_request_end);

void read_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END *const plog_request_end);

void write_DEBUG_VECT(pdebug_vect_DEBUG_VECT *const pdebug_vect);

void read_DEBUG_VECT(pdebug_vect_DEBUG_VECT *const pdebug_vect);

void write_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE *const pvision_speed_estimate);

void read_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE *const pvision_speed_estimate);

void write_LOGGING_ACK(plogging_ack_LOGGING_ACK *const plogging_ack);

void read_LOGGING_ACK(plogging_ack_LOGGING_ACK *const plogging_ack);

void read_MISSION_ITEM_REACHED(pmission_item_reached_MISSION_ITEM_REACHED *const pmission_item_reached);

void write_MEMINFO(pmeminfo_MEMINFO *const pmeminfo);

void read_MEMINFO(pmeminfo_MEMINFO *const pmeminfo);

void read_SERVO_OUTPUT_RAW(pservo_output_raw_SERVO_OUTPUT_RAW *const pservo_output_raw);

void read_RC_CHANNELS_RAW(prc_channels_raw_RC_CHANNELS_RAW *const prc_channels_raw);

void write_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION *const pflight_information);

void read_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION *const pflight_information);

void write_DATA96(pdata96_DATA96 *const pdata96);

void read_DATA96(pdata96_DATA96 *const pdata96);

void write_WIFI_CONFIG_AP(pwifi_config_ap_WIFI_CONFIG_AP *const pwifi_config_ap);

void read_WIFI_CONFIG_AP(pwifi_config_ap_WIFI_CONFIG_AP *const pwifi_config_ap);

void write_SIM_STATE(psim_state_SIM_STATE *const psim_state);

void read_SIM_STATE(psim_state_SIM_STATE *const psim_state);

void write_LED_CONTROL(pled_control_LED_CONTROL *const pled_control);

void read_LED_CONTROL(pled_control_LED_CONTROL *const pled_control);

void read_POSITION_TARGET_GLOBAL_INT(pposition_target_global_int_POSITION_TARGET_GLOBAL_INT *const pposition_target_global_int);

void write_MOUNT_CONTROL(pmount_control_MOUNT_CONTROL *const pmount_control);

void read_MOUNT_CONTROL(pmount_control_MOUNT_CONTROL *const pmount_control);

void read_SET_MODE(pset_mode_SET_MODE *const pset_mode);

void write_SCALED_IMU3(pscaled_imu3_SCALED_IMU3 *const pscaled_imu3);

void read_SCALED_IMU3(pscaled_imu3_SCALED_IMU3 *const pscaled_imu3);

void read_HIL_RC_INPUTS_RAW(phil_rc_inputs_raw_HIL_RC_INPUTS_RAW *const phil_rc_inputs_raw);

void write_SET_HOME_POSITION(pset_home_position_SET_HOME_POSITION *const pset_home_position);

void read_SET_HOME_POSITION(pset_home_position_SET_HOME_POSITION *const pset_home_position);

void write_TERRAIN_REPORT(pterrain_report_TERRAIN_REPORT *const pterrain_report);

void read_TERRAIN_REPORT(pterrain_report_TERRAIN_REPORT *const pterrain_report);

void read_MISSION_REQUEST(pmission_request_MISSION_REQUEST *const pmission_request);

void read_DATA_STREAM(pdata_stream_DATA_STREAM *const pdata_stream);

void read_COMMAND_ACK(pcommand_ack_COMMAND_ACK *const pcommand_ack);

void read_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *const plocal_position_ned_system_global_offset);

void read_MISSION_REQUEST_INT(pmission_request_int_MISSION_REQUEST_INT *const pmission_request_int);

void write_MOUNT_CONFIGURE(pmount_configure_MOUNT_CONFIGURE *const pmount_configure);

void read_MOUNT_CONFIGURE(pmount_configure_MOUNT_CONFIGURE *const pmount_configure);

void write_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK *const pterrain_check);

void read_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK *const pterrain_check);

void write_LOGGING_DATA_ACKED(plogging_data_acked_LOGGING_DATA_ACKED *const plogging_data_acked);

void read_LOGGING_DATA_ACKED(plogging_data_acked_LOGGING_DATA_ACKED *const plogging_data_acked);

void write_REMOTE_LOG_DATA_BLOCK(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *const premote_log_data_block);

void read_REMOTE_LOG_DATA_BLOCK(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *const premote_log_data_block);

void write_POWER_STATUS(ppower_status_POWER_STATUS *const ppower_status);

void read_POWER_STATUS(ppower_status_POWER_STATUS *const ppower_status);

void read_PARAM_MAP_RC(pparam_map_rc_PARAM_MAP_RC *const pparam_map_rc);

void read_HEARTBEAT(pheartbeat_HEARTBEAT *const pheartbeat);

void write_V2_EXTENSION(pv2_extension_V2_EXTENSION *const pv2_extension);

void read_V2_EXTENSION(pv2_extension_V2_EXTENSION *const pv2_extension);

void read_SCALED_PRESSURE(pscaled_pressure_SCALED_PRESSURE *const pscaled_pressure);

void write_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST *const plog_request_list);

void read_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST *const plog_request_list);

void write_MAG_CAL_REPORT(pmag_cal_report_MAG_CAL_REPORT *const pmag_cal_report);

void read_MAG_CAL_REPORT(pmag_cal_report_MAG_CAL_REPORT *const pmag_cal_report);

void write_GPS2_RTK(pgps2_rtk_GPS2_RTK *const pgps2_rtk);

void read_GPS2_RTK(pgps2_rtk_GPS2_RTK *const pgps2_rtk);

void write_VICON_POSITION_ESTIMATE(pvicon_position_estimate_VICON_POSITION_ESTIMATE *const pvicon_position_estimate);

void read_VICON_POSITION_ESTIMATE(pvicon_position_estimate_VICON_POSITION_ESTIMATE *const pvicon_position_estimate);

void write_AHRS3(pahrs3_AHRS3 *const pahrs3);

void read_AHRS3(pahrs3_AHRS3 *const pahrs3);

void read_MISSION_CLEAR_ALL(pmission_clear_all_MISSION_CLEAR_ALL *const pmission_clear_all);

void write_LOG_DATA(plog_data_LOG_DATA *const plog_data);

void read_LOG_DATA(plog_data_LOG_DATA *const plog_data);

void write_OPTICAL_FLOW_RAD(poptical_flow_rad_OPTICAL_FLOW_RAD *const poptical_flow_rad);

void read_OPTICAL_FLOW_RAD(poptical_flow_rad_OPTICAL_FLOW_RAD *const poptical_flow_rad);

void read_SAFETY_ALLOWED_AREA(psafety_allowed_area_SAFETY_ALLOWED_AREA *const psafety_allowed_area);

void write_PID_TUNING(ppid_tuning_PID_TUNING *const ppid_tuning);

void read_PID_TUNING(ppid_tuning_PID_TUNING *const ppid_tuning);

void read_MANUAL_SETPOINT(pmanual_setpoint_MANUAL_SETPOINT *const pmanual_setpoint);

void write_MOUNT_STATUS(pmount_status_MOUNT_STATUS *const pmount_status);

void read_MOUNT_STATUS(pmount_status_MOUNT_STATUS *const pmount_status);

void write_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST *const pterrain_request);

void read_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST *const pterrain_request);

void write_LOG_ERASE(plog_erase_LOG_ERASE *const plog_erase);

void read_LOG_ERASE(plog_erase_LOG_ERASE *const plog_erase);

void write_AHRS2(pahrs2_AHRS2 *const pahrs2);

void read_AHRS2(pahrs2_AHRS2 *const pahrs2);

void read_MISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list_MISSION_WRITE_PARTIAL_LIST *const pmission_write_partial_list);

void read_ATTITUDE(pattitude_ATTITUDE *const pattitude);

void write_GOPRO_HEARTBEAT(pgopro_heartbeat_GOPRO_HEARTBEAT *const pgopro_heartbeat);

void read_GOPRO_HEARTBEAT(pgopro_heartbeat_GOPRO_HEARTBEAT *const pgopro_heartbeat);

void write_NAMED_VALUE_FLOAT(pnamed_value_float_NAMED_VALUE_FLOAT *const pnamed_value_float);

void read_NAMED_VALUE_FLOAT(pnamed_value_float_NAMED_VALUE_FLOAT *const pnamed_value_float);

void write_DIGICAM_CONTROL(pdigicam_control_DIGICAM_CONTROL *const pdigicam_control);

void read_DIGICAM_CONTROL(pdigicam_control_DIGICAM_CONTROL *const pdigicam_control);

void read_RAW_PRESSURE(praw_pressure_RAW_PRESSURE *const praw_pressure);

void write_DEVICE_OP_READ_REPLY(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *const pdevice_op_read_reply);

void read_DEVICE_OP_READ_REPLY(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *const pdevice_op_read_reply);

void write_CAMERA_SETTINGS(pcamera_settings_CAMERA_SETTINGS *const pcamera_settings);

void read_CAMERA_SETTINGS(pcamera_settings_CAMERA_SETTINGS *const pcamera_settings);

void read_RC_CHANNELS_SCALED(prc_channels_scaled_RC_CHANNELS_SCALED *const prc_channels_scaled);

void write_CAMERA_STATUS(pcamera_status_CAMERA_STATUS *const pcamera_status);

void read_CAMERA_STATUS(pcamera_status_CAMERA_STATUS *const pcamera_status);

void read_GPS_RAW_INT(pgps_raw_int_GPS_RAW_INT *const pgps_raw_int);

void write_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA *const plog_request_data);

void read_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA *const plog_request_data);

void write_COMPASSMOT_STATUS(pcompassmot_status_COMPASSMOT_STATUS *const pcompassmot_status);

void read_COMPASSMOT_STATUS(pcompassmot_status_COMPASSMOT_STATUS *const pcompassmot_status);

void read_COMMAND_LONG(pcommand_long_COMMAND_LONG *const pcommand_long);

void write_GPS_INPUT(pgps_input_GPS_INPUT *const pgps_input);

void read_GPS_INPUT(pgps_input_GPS_INPUT *const pgps_input);

void write_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA *const pencapsulated_data);

void read_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA *const pencapsulated_data);

void read_GLOBAL_POSITION_INT(pglobal_position_int_GLOBAL_POSITION_INT *const pglobal_position_int);

void write_CAMERA_CAPTURE_STATUS(pcamera_capture_status_CAMERA_CAPTURE_STATUS *const pcamera_capture_status);

void read_CAMERA_CAPTURE_STATUS(pcamera_capture_status_CAMERA_CAPTURE_STATUS *const pcamera_capture_status);

void write_GOPRO_GET_REQUEST(pgopro_get_request_GOPRO_GET_REQUEST *const pgopro_get_request);

void read_GOPRO_GET_REQUEST(pgopro_get_request_GOPRO_GET_REQUEST *const pgopro_get_request);

void read_PING(pping_PING *const pping);

void write_STATUSTEXT(pstatustext_STATUSTEXT *const pstatustext);

void read_STATUSTEXT(pstatustext_STATUSTEXT *const pstatustext);

void write_ATT_POS_MOCAP(patt_pos_mocap_ATT_POS_MOCAP *const patt_pos_mocap);

void read_ATT_POS_MOCAP(patt_pos_mocap_ATT_POS_MOCAP *const patt_pos_mocap);

void write_AIRSPEED_AUTOCAL(pairspeed_autocal_AIRSPEED_AUTOCAL *const pairspeed_autocal);

void read_AIRSPEED_AUTOCAL(pairspeed_autocal_AIRSPEED_AUTOCAL *const pairspeed_autocal);

void read_LOCAL_POSITION_NED_COV(plocal_position_ned_cov_LOCAL_POSITION_NED_COV *const plocal_position_ned_cov);

void write_RADIO(pradio_RADIO *const pradio);

void read_RADIO(pradio_RADIO *const pradio);

void write_FENCE_FETCH_POINT(pfence_fetch_point_FENCE_FETCH_POINT *const pfence_fetch_point);

void read_FENCE_FETCH_POINT(pfence_fetch_point_FENCE_FETCH_POINT *const pfence_fetch_point);

void read_AUTH_KEY(pauth_key_AUTH_KEY *const pauth_key);

void read_NAV_CONTROLLER_OUTPUT(pnav_controller_output_NAV_CONTROLLER_OUTPUT *const pnav_controller_output);

void write_HIL_GPS(phil_gps_HIL_GPS *const phil_gps);

void read_HIL_GPS(phil_gps_HIL_GPS *const phil_gps);

void write_CAMERA_FEEDBACK(pcamera_feedback_CAMERA_FEEDBACK *const pcamera_feedback);

void read_CAMERA_FEEDBACK(pcamera_feedback_CAMERA_FEEDBACK *const pcamera_feedback);

void write_LIMITS_STATUS(plimits_status_LIMITS_STATUS *const plimits_status);

void read_LIMITS_STATUS(plimits_status_LIMITS_STATUS *const plimits_status);

void write_BATTERY2(pbattery2_BATTERY2 *const pbattery2);

void read_BATTERY2(pbattery2_BATTERY2 *const pbattery2);

void write_PARAM_EXT_VALUE(pparam_ext_value_PARAM_EXT_VALUE *const pparam_ext_value);

void read_PARAM_EXT_VALUE(pparam_ext_value_PARAM_EXT_VALUE *const pparam_ext_value);

void write_VIBRATION(pvibration_VIBRATION *const pvibration);

void read_VIBRATION(pvibration_VIBRATION *const pvibration);

void write_ADAP_TUNING(padap_tuning_ADAP_TUNING *const padap_tuning);

void read_ADAP_TUNING(padap_tuning_ADAP_TUNING *const padap_tuning);

void read_MISSION_SET_CURRENT(pmission_set_current_MISSION_SET_CURRENT *const pmission_set_current);

void write_RALLY_POINT(prally_point_RALLY_POINT *const prally_point);

void read_RALLY_POINT(prally_point_RALLY_POINT *const prally_point);

void read_VFR_HUD(pvfr_hud_VFR_HUD *const pvfr_hud);

void write_PING33(pping33_PING33 *const pping33);

void read_PING33(pping33_PING33 *const pping33);

void write_DATA32(pdata32_DATA32 *const pdata32);

void read_DATA32(pdata32_DATA32 *const pdata32);

void read_SET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int_SET_POSITION_TARGET_GLOBAL_INT *const pset_position_target_global_int);

void write_CONTROL_SYSTEM_STATE(pcontrol_system_state_CONTROL_SYSTEM_STATE *const pcontrol_system_state);

void read_CONTROL_SYSTEM_STATE(pcontrol_system_state_CONTROL_SYSTEM_STATE *const pcontrol_system_state);

void write_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *const pset_actuator_control_target);

void read_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target_SET_ACTUATOR_CONTROL_TARGET *const pset_actuator_control_target);

void write_LANDING_TARGET(planding_target_LANDING_TARGET *const planding_target);

void read_LANDING_TARGET(planding_target_LANDING_TARGET *const planding_target);

void write_UAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const puavionix_adsb_out_cfg);

void read_UAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const puavionix_adsb_out_cfg);

void read_PARAM_REQUEST_LIST(pparam_request_list_PARAM_REQUEST_LIST *const pparam_request_list);

void write_GPS_RTK(pgps_rtk_GPS_RTK *const pgps_rtk);

void read_GPS_RTK(pgps_rtk_GPS_RTK *const pgps_rtk);

void write_SETUP_SIGNING(psetup_signing_SETUP_SIGNING *const psetup_signing);

void read_SETUP_SIGNING(psetup_signing_SETUP_SIGNING *const psetup_signing);

void write_HIL_SENSOR(phil_sensor_HIL_SENSOR *const phil_sensor);

void read_HIL_SENSOR(phil_sensor_HIL_SENSOR *const phil_sensor);

void read_HIL_CONTROLS(phil_controls_HIL_CONTROLS *const phil_controls);

void write_PARAM_EXT_REQUEST_READ(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *const pparam_ext_request_read);

void read_PARAM_EXT_REQUEST_READ(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *const pparam_ext_request_read);

void write_MEMORY_VECT(pmemory_vect_MEMORY_VECT *const pmemory_vect);

void read_MEMORY_VECT(pmemory_vect_MEMORY_VECT *const pmemory_vect);

void read_REQUEST_DATA_STREAM(prequest_data_stream_REQUEST_DATA_STREAM *const prequest_data_stream);

void write_GPS2_RAW(pgps2_raw_GPS2_RAW *const pgps2_raw);

void read_GPS2_RAW(pgps2_raw_GPS2_RAW *const pgps2_raw);

void write_OBSTACLE_DISTANCE(pobstacle_distance_OBSTACLE_DISTANCE *const pobstacle_distance);

void read_OBSTACLE_DISTANCE(pobstacle_distance_OBSTACLE_DISTANCE *const pobstacle_distance);

void write_REMOTE_LOG_BLOCK_STATUS(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *const premote_log_block_status);

void read_REMOTE_LOG_BLOCK_STATUS(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *const premote_log_block_status);

void write_FENCE_STATUS(pfence_status_FENCE_STATUS *const pfence_status);

void read_FENCE_STATUS(pfence_status_FENCE_STATUS *const pfence_status);

void write_HOME_POSITION(phome_position_HOME_POSITION *const phome_position);

void read_HOME_POSITION(phome_position_HOME_POSITION *const phome_position);

void read_HIL_STATE(phil_state_HIL_STATE *const phil_state);

void write_FOLLOW_TARGET(pfollow_target_FOLLOW_TARGET *const pfollow_target);

void read_FOLLOW_TARGET(pfollow_target_FOLLOW_TARGET *const pfollow_target);

void read_SET_ATTITUDE_TARGET(pset_attitude_target_SET_ATTITUDE_TARGET *const pset_attitude_target);

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

void write_AHRS(pahrs_AHRS *const pahrs);

void read_AHRS(pahrs_AHRS *const pahrs);

void write_VIDEO_STREAM_INFORMATION(pvideo_stream_information_VIDEO_STREAM_INFORMATION *const pvideo_stream_information);

void read_VIDEO_STREAM_INFORMATION(pvideo_stream_information_VIDEO_STREAM_INFORMATION *const pvideo_stream_information);

void read_SCALED_IMU(pscaled_imu_SCALED_IMU *const pscaled_imu);

void read_RC_CHANNELS_OVERRIDE(prc_channels_override_RC_CHANNELS_OVERRIDE *const prc_channels_override);

void write_GIMBAL_CONTROL(pgimbal_control_GIMBAL_CONTROL *const pgimbal_control);

void read_GIMBAL_CONTROL(pgimbal_control_GIMBAL_CONTROL *const pgimbal_control);

void write_TERRAIN_DATA(pterrain_data_TERRAIN_DATA *const pterrain_data);

void read_TERRAIN_DATA(pterrain_data_TERRAIN_DATA *const pterrain_data);

void read_PARAM_SET(pparam_set_PARAM_SET *const pparam_set);

void write_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *const pdevice_op_write_reply);

void read_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *const pdevice_op_write_reply);

void read_GPS_STATUS(pgps_status_GPS_STATUS *const pgps_status);

void write_CAMERA_INFORMATION(pcamera_information_CAMERA_INFORMATION *const pcamera_information);

void read_CAMERA_INFORMATION(pcamera_information_CAMERA_INFORMATION *const pcamera_information);

void write_STORAGE_INFORMATION(pstorage_information_STORAGE_INFORMATION *const pstorage_information);

void read_STORAGE_INFORMATION(pstorage_information_STORAGE_INFORMATION *const pstorage_information);

void write_SENSOR_OFFSETS(psensor_offsets_SENSOR_OFFSETS *const psensor_offsets);

void read_SENSOR_OFFSETS(psensor_offsets_SENSOR_OFFSETS *const psensor_offsets);

void write_HIL_STATE_QUATERNION(phil_state_quaternion_HIL_STATE_QUATERNION *const phil_state_quaternion);

void read_HIL_STATE_QUATERNION(phil_state_quaternion_HIL_STATE_QUATERNION *const phil_state_quaternion);

void write_ALTITUDE(paltitude_ALTITUDE *const paltitude);

void read_ALTITUDE(paltitude_ALTITUDE *const paltitude);

void write_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *const pgimbal_torque_cmd_report);

void read_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *const pgimbal_torque_cmd_report);

void write_COLLISION(pcollision_COLLISION *const pcollision);

void read_COLLISION(pcollision_COLLISION *const pcollision);

void write_UAVCAN_NODE_STATUS(puavcan_node_status_UAVCAN_NODE_STATUS *const puavcan_node_status);

void read_UAVCAN_NODE_STATUS(puavcan_node_status_UAVCAN_NODE_STATUS *const puavcan_node_status);

void read_SAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area_SAFETY_SET_ALLOWED_AREA *const psafety_set_allowed_area);

void write_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE *const pbutton_change);

void read_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE *const pbutton_change);

void read_GLOBAL_POSITION_INT_COV(pglobal_position_int_cov_GLOBAL_POSITION_INT_COV *const pglobal_position_int_cov);

void write_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *const pparam_ext_request_list);

void read_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *const pparam_ext_request_list);

void write_TIMESYNC(ptimesync_TIMESYNC *const ptimesync);

void read_TIMESYNC(ptimesync_TIMESYNC *const ptimesync);

void write_HWSTATUS(phwstatus_HWSTATUS *const phwstatus);

void read_HWSTATUS(phwstatus_HWSTATUS *const phwstatus);

void write_ESTIMATOR_STATUS(pestimator_status_ESTIMATOR_STATUS *const pestimator_status);

void read_ESTIMATOR_STATUS(pestimator_status_ESTIMATOR_STATUS *const pestimator_status);

void write_EKF_STATUS_REPORT(pekf_status_report_EKF_STATUS_REPORT *const pekf_status_report);

void read_EKF_STATUS_REPORT(pekf_status_report_EKF_STATUS_REPORT *const pekf_status_report);

void write_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL *const pmessage_interval);

void read_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL *const pmessage_interval);

void write_ADSB_VEHICLE(padsb_vehicle_ADSB_VEHICLE *const padsb_vehicle);

void read_ADSB_VEHICLE(padsb_vehicle_ADSB_VEHICLE *const padsb_vehicle);

void read_MISSION_COUNT(pmission_count_MISSION_COUNT *const pmission_count);

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


void read_MISSION_COUNT(pmission_count_MISSION_COUNT *const pmission_count) {
	int16_t            count            = pmission_count_count_GET(pmission_count);
	int8_t             target_system    = pmission_count_target_system_GET(pmission_count);
	int8_t             target_component = pmission_count_target_component_GET(pmission_count);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_count_mission_type_GET(pmission_count, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
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
	padsb_vehicle_heading_SET(867, -30078, -6264, 9333, -29575, 28419, -18012, 244, 28398, 8117, padsb_vehicle);
	padsb_vehicle_hor_velocity_SET(25030, -21810, -32741, 8231, -18242, -29519, 28055, -27014, 165, 9492, padsb_vehicle);
	padsb_vehicle_squawk_SET(-12475, 17095, -31721, 1229, 8268, -10269, 15456, -9499, -16268, 16416, padsb_vehicle);
	padsb_vehicle_ICAO_address_SET(-534879762, 94115482, -18576398, 1327750109, -513438637, -2146080618, -1841508545, 1414283598, 395861449, -138131922, padsb_vehicle);
	padsb_vehicle_lat_SET(-2083787912, -1267460113, -613734468, -1996210194, -181098227, 639605360, 669606932, -214819256, 2064993927, 92264938, padsb_vehicle);
	padsb_vehicle_lon_SET(-791568767, -1712597789, -1309287253, -968181607, 440454410, -52163711, -579149888, -1733720687, -1729924431, -1043307744, padsb_vehicle);
	padsb_vehicle_altitude_SET(1132800224, -844803036, -2029991817, 924082807, -882314734, 155262961, 434650154, -464021346, 2130962814, -1159982442, padsb_vehicle);
	padsb_vehicle_ver_velocity_SET(-18501, -20767, -285, -30182, 1663, -15387, 29808, -22195, -28118, 30787, padsb_vehicle);
	padsb_vehicle_tslc_SET(45, -124, 102, 99, -88, -27, 20, 27, -103, 59, padsb_vehicle);
	padsb_vehicle_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, padsb_vehicle);
	padsb_vehicle_callsign_SET(some_string, strlen(some_string), padsb_vehicle);
	padsb_vehicle_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGHLY_MANUV, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SPACE, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_EMERGENCY_SURFACE, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LARGE, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SPACE, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY, padsb_vehicle);
	padsb_vehicle_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE, e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY, e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED, e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS, e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS, e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS, e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE, e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED, e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE, padsb_vehicle);
	
}


void read_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL *const pmessage_interval) {
	int16_t message_id  = pmessage_interval_message_id_GET(pmessage_interval);
	int32_t interval_us = pmessage_interval_interval_us_GET(pmessage_interval);
	
}


void write_MESSAGE_INTERVAL(pmessage_interval_MESSAGE_INTERVAL *const pmessage_interval) {
	pmessage_interval_message_id_SET(1247, 22925, 6251, -11818, -14726, 7542, 5360, 25262, 18280, -20740, pmessage_interval);
	pmessage_interval_interval_us_SET(163361855, 183893324, 168777855, -713067280, -185396681, -1426678971, 923308305, 1071516840, -2017452050, 1852564613, pmessage_interval);
	
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


void write_EKF_STATUS_REPORT(pekf_status_report_EKF_STATUS_REPORT *const pekf_status_report) {
	pekf_status_report_velocity_variance_SET(1.6215275E38F, 5.574961E37F, 2.4825345E38F, -1.5301195E38F, -8.1104894E37F, -2.724266E38F, 2.0709383E38F, -2.353831E38F, -7.386086E37F, -6.627588E37F, pekf_status_report);
	pekf_status_report_pos_horiz_variance_SET(1.9832059E38F, 1.661356E38F, 2.044112E38F, 2.354923E38F, 1.7966939E37F, 5.2955693E37F, 3.0724859E38F, -2.9571419E38F, 2.060347E38F, 2.5600581E38F, pekf_status_report);
	pekf_status_report_pos_vert_variance_SET(-2.4922045E38F, -2.0296573E38F, 2.5587639E38F, -7.4454256E37F, 8.685849E37F, -3.3540165E38F, -2.4205263E38F, -2.4221154E38F, 3.918195E37F, -1.6270333E38F, pekf_status_report);
	pekf_status_report_compass_variance_SET(3.1101706E37F, 6.992878E37F, 1.1888607E38F, -3.0946647E38F, -1.6118226E38F, 2.8078475E38F, -2.6626154E37F, 9.274557E37F, 2.0579807E37F, -1.1803613E37F, pekf_status_report);
	pekf_status_report_terrain_alt_variance_SET(-4.490308E37F, -2.881231E38F, -2.6295091E38F, 2.752152E38F, -3.9291545E37F, 1.6533414E38F, -1.6642154E38F, -1.5897521E38F, 6.3222274E37F, -2.3165195E38F, pekf_status_report);
	pekf_status_report_flags_SET(e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL, e_EKF_STATUS_FLAGS_EKF_POS_VERT_AGL, e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL, e_EKF_STATUS_FLAGS_EKF_ATTITUDE, e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL, e_EKF_STATUS_FLAGS_EKF_VELOCITY_HORIZ, e_EKF_STATUS_FLAGS_EKF_POS_VERT_AGL, e_EKF_STATUS_FLAGS_EKF_CONST_POS_MODE, e_EKF_STATUS_FLAGS_EKF_ATTITUDE, e_EKF_STATUS_FLAGS_EKF_CONST_POS_MODE, pekf_status_report);
	
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
	pestimator_status_time_usec_SET(-862404962332184207L, 1164587375774805697L, 430089670283840587L, -3053806957592534984L, 8497374216349227627L, 5935759128087703466L, -7399625489200664239L, -928638766235278636L, -4134275975393401498L, 6886537573670591831L, pestimator_status);
	pestimator_status_vel_ratio_SET(7.3945984E37F, 3.1553144E38F, 1.6165462E38F, -2.6358596E38F, 2.4734817E37F, 1.5620475E38F, 2.223522E37F, -2.3771735E38F, 8.107349E37F, 2.1670107E37F, pestimator_status);
	pestimator_status_pos_horiz_ratio_SET(1.8549363E38F, -2.817757E38F, 2.3107143E38F, 3.362756E36F, 1.3032925E38F, -5.1236546E37F, 2.820475E38F, 5.1115957E37F, 2.9192638E38F, 2.235009E38F, pestimator_status);
	pestimator_status_pos_vert_ratio_SET(-3.1162708E37F, -3.277556E38F, -2.0871058E37F, -3.1672993E38F, 5.444965E37F, 7.9422803E37F, -2.248734E37F, 1.924852E38F, 2.9589557E38F, 2.849048E38F, pestimator_status);
	pestimator_status_mag_ratio_SET(1.5324381E38F, 3.1747543E38F, -8.916433E37F, 2.0902427E38F, -1.866341E38F, 3.2901239E38F, -3.0255713E38F, 3.1371002E38F, -1.5572152E38F, 2.900976E38F, pestimator_status);
	pestimator_status_hagl_ratio_SET(1.5222575E38F, 1.4061771E38F, 2.946359E38F, 2.1139733E38F, -2.3318567E38F, 1.979387E38F, 3.0298932E38F, -2.868436E38F, 1.4284715E38F, 1.2040404E37F, pestimator_status);
	pestimator_status_tas_ratio_SET(3.6816944E37F, 2.2618325E38F, 1.5732316E37F, -3.1562754E38F, 2.8420096E38F, 2.1185257E38F, -1.0921699E38F, 2.6813865E38F, 1.2192965E38F, -2.3803408E37F, pestimator_status);
	pestimator_status_pos_horiz_accuracy_SET(-1.2222309E38F, -3.0032505E38F, -3.8300113E37F, -1.3428626E38F, -2.8180944E38F, 1.1098108E38F, 2.6184406E38F, -2.3048308E38F, 1.8680322E38F, -2.5082862E38F, pestimator_status);
	pestimator_status_pos_vert_accuracy_SET(-2.4644951E38F, 3.014634E38F, 2.928718E38F, -6.2843875E37F, 2.6175239E38F, 2.3712748E37F, 2.6038393E38F, -6.8763874E37F, -2.6981477E38F, 2.5341033E38F, pestimator_status);
	pestimator_status_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL, e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL, pestimator_status);
	
}


void read_HWSTATUS(phwstatus_HWSTATUS *const phwstatus) {
	int16_t Vcc    = phwstatus_Vcc_GET(phwstatus);
	int8_t  I2Cerr = phwstatus_I2Cerr_GET(phwstatus);
	
}


void write_HWSTATUS(phwstatus_HWSTATUS *const phwstatus) {
	phwstatus_Vcc_SET(-6812, 8518, -31064, -16399, -4753, -19548, 12156, -2402, 21144, 21199, phwstatus);
	phwstatus_I2Cerr_SET(115, -75, -96, 115, 53, -53, 41, 47, -8, -101, phwstatus);
	
}


void read_TIMESYNC(ptimesync_TIMESYNC *const ptimesync) {
	int64_t tc1 = ptimesync_tc1_GET(ptimesync);
	int64_t ts1 = ptimesync_ts1_GET(ptimesync);
	
}


void write_TIMESYNC(ptimesync_TIMESYNC *const ptimesync) {
	ptimesync_tc1_SET(-3918568770553391841L, 6767050373255555597L, -139179985684821010L, 1983505022513614848L, 3172389102684366314L, -2106110383485450698L, 4220475047159341288L, 6230113379868009106L, -6810237790463851675L, -4884152454114196037L, ptimesync);
	ptimesync_ts1_SET(-5631875199953397478L, -2586635947594189304L, 9077759321370839543L, 7188063376915704482L, 1077746728837297312L, 3422076482887409780L, 8969170889886228604L, -5615411679043562086L, 7809863889688255428L, -559227111496135792L, ptimesync);
	
}


void read_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *const pparam_ext_request_list) {
	int8_t target_system    = pparam_ext_request_list_target_system_GET(pparam_ext_request_list);
	int8_t target_component = pparam_ext_request_list_target_component_GET(pparam_ext_request_list);
	
}


void write_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST *const pparam_ext_request_list) {
	pparam_ext_request_list_target_system_SET(-16, 58, -89, -66, 127, 97, -13, 7, -75, -46, pparam_ext_request_list);
	pparam_ext_request_list_target_component_SET(-76, -72, -35, 111, -116, -20, 95, -48, -60, -84, pparam_ext_request_list);
	
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


void read_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE *const pbutton_change) {
	int32_t time_boot_ms   = pbutton_change_time_boot_ms_GET(pbutton_change);
	int32_t last_change_ms = pbutton_change_last_change_ms_GET(pbutton_change);
	int8_t  state          = pbutton_change_state_GET(pbutton_change);
	
}


void write_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE *const pbutton_change) {
	pbutton_change_time_boot_ms_SET(320175018, -1844499776, 915877809, -1782504776, 1093572215, -1414397026, -158720524, -358640795, -268491490, -1086836391, pbutton_change);
	pbutton_change_last_change_ms_SET(1759510344, 309919620, -1814299225, -153423326, 78005947, 1193415127, 1140620853, -381162149, -559920743, 1301403402, pbutton_change);
	pbutton_change_state_SET(94, 19, 30, -123, -3, 102, -74, 77, 20, -103, pbutton_change);
	
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


void write_UAVCAN_NODE_STATUS(puavcan_node_status_UAVCAN_NODE_STATUS *const puavcan_node_status) {
	puavcan_node_status_vendor_specific_status_code_SET(-17939, -29302, -1211, 5812, 3668, 30790, -18634, 5336, -16696, -27364, puavcan_node_status);
	puavcan_node_status_uptime_sec_SET(-1494235854, -1093659253, -1274230238, 1908821200, -75213807, -666193134, 1249343086, 2006972376, 1569418116, -1630910860, puavcan_node_status);
	puavcan_node_status_time_usec_SET(-5923154386397183730L, -3989415478373401331L, -1305686837365565628L, 3220998694218303690L, -7161608295600810874L, 4327761202261635776L, -5783085815207642762L, -4312015613929932510L, -6792550162011956115L, 3257917092798278506L, puavcan_node_status);
	puavcan_node_status_sub_mode_SET(100, -44, -96, 8, 123, 110, -80, 56, -25, 24, puavcan_node_status);
	puavcan_node_status_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, puavcan_node_status);
	puavcan_node_status_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, puavcan_node_status);
	
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
	pcollision_id_SET(-1969530052, 198456936, -971841662, -1271445915, -405137424, 146378244, 1454213809, 1566165248, 2135729820, 653620373, pcollision);
	pcollision_time_to_minimum_delta_SET(-2.6351787E38F, 2.3597924E38F, 3.4016826E38F, -3.395238E38F, 1.7111281E38F, -8.332446E37F, -1.1159062E37F, 1.2902175E38F, -1.8140358E38F, 3.2137581E38F, pcollision);
	pcollision_altitude_minimum_delta_SET(-1.983559E37F, 1.2483656E38F, 6.0771976E37F, 7.6998223E37F, -2.8889121E35F, 1.7893326E38F, 1.9897451E38F, 3.9478493E37F, 1.5294856E38F, -1.3405663E38F, pcollision);
	pcollision_horizontal_minimum_delta_SET(2.5311266E37F, -2.991988E38F, -1.5313869E38F, 1.7687795E38F, 1.797654E38F, 7.110926E37F, -2.3922656E38F, -3.169029E38F, -1.3230836E38F, 1.8737524E37F, pcollision);
	pcollision_sRc_SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, pcollision);
	pcollision_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, pcollision);
	pcollision_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, pcollision);
	
}


void read_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *const pgimbal_torque_cmd_report) {
	int8_t  target_system    = pgimbal_torque_cmd_report_target_system_GET(pgimbal_torque_cmd_report);
	int8_t  target_component = pgimbal_torque_cmd_report_target_component_GET(pgimbal_torque_cmd_report);
	int16_t rl_torque_cmd    = pgimbal_torque_cmd_report_rl_torque_cmd_GET(pgimbal_torque_cmd_report);
	int16_t el_torque_cmd    = pgimbal_torque_cmd_report_el_torque_cmd_GET(pgimbal_torque_cmd_report);
	int16_t az_torque_cmd    = pgimbal_torque_cmd_report_az_torque_cmd_GET(pgimbal_torque_cmd_report);
	
}


void write_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT *const pgimbal_torque_cmd_report) {
	pgimbal_torque_cmd_report_target_system_SET(-20, 104, 38, 82, -91, 9, -30, 66, 89, 101, pgimbal_torque_cmd_report);
	pgimbal_torque_cmd_report_target_component_SET(39, 1, 120, 41, 72, 60, 26, -97, 21, -127, pgimbal_torque_cmd_report);
	pgimbal_torque_cmd_report_rl_torque_cmd_SET(15057, -10859, -32673, 14627, 29342, -17008, 28913, -11817, -27965, -8158, pgimbal_torque_cmd_report);
	pgimbal_torque_cmd_report_el_torque_cmd_SET(-16818, -18759, -6159, -5053, -10986, 32643, 25630, 6775, 13007, -1039, pgimbal_torque_cmd_report);
	pgimbal_torque_cmd_report_az_torque_cmd_SET(26152, -14494, -14992, -9786, -1988, -226, -2002, 26547, 2308, 27706, pgimbal_torque_cmd_report);
	
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
	paltitude_time_usec_SET(196228254289653852L, -521459470998637195L, 2846249266034130752L, 1862981550937412092L, -7277761440574027791L, -1937182964900221724L, 4908237259432778999L, -4429969945326430768L, 3794006433636722228L, -2664149893235582331L, paltitude);
	paltitude_altitude_monotonic_SET(-1.142788E38F, 2.7386528E38F, 2.2514754E38F, -3.2228686E38F, 1.2411336E38F, -2.7697598E38F, -3.2758688E38F, -2.1098667E38F, -2.1232916E38F, 1.5590372E38F, paltitude);
	paltitude_altitude_amsl_SET(2.2871342E38F, 1.7284785E38F, 1.7931845E38F, -2.4765216E38F, -2.159324E37F, 1.159829E37F, -3.3393208E37F, -1.8847113E38F, 8.6533696E36F, 2.5353762E37F, paltitude);
	paltitude_altitude_local_SET(-1.0753684E38F, 2.7557665E38F, 8.513801E37F, -4.0991178E36F, -3.4882629E37F, 1.6863299E38F, -2.8655418E38F, 3.3930905E38F, 5.4303556E37F, 1.7442108E38F, paltitude);
	paltitude_altitude_relative_SET(2.6661572E38F, -1.930271E38F, -2.4476104E38F, -2.8581168E38F, 1.9267218E38F, -9.929031E37F, -1.637269E38F, 2.7840686E38F, 2.42387E37F, -1.759116E38F, paltitude);
	paltitude_altitude_terrain_SET(1.4744253E38F, 1.588906E38F, 1.0174248E38F, 1.79987E38F, -4.0381976E37F, -1.9941734E38F, -3.1350433E38F, 2.258603E38F, -3.3225625E38F, 6.3158437E35F, paltitude);
	paltitude_bottom_clearance_SET(1.8443866E38F, 4.746533E37F, -1.1299166E38F, 2.4085906E36F, -8.021387E37F, -1.1992848E38F, -1.5482651E37F, -5.9633403E37F, -1.153406E38F, 3.1397253E38F, paltitude);
	
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
	phil_state_quaternion_ind_airspeed_SET(-21316, -1866, -935, 17886, -14419, -22883, 21937, 18406, 2213, -32399, phil_state_quaternion);
	phil_state_quaternion_true_airspeed_SET(10849, -32764, -22037, 8022, -19416, -20215, -29331, 32085, 16347, 1921, phil_state_quaternion);
	phil_state_quaternion_time_usec_SET(-800788345293560451L, -828934714668559302L, -8637843038639571687L, 172417737657019485L, 8675571636603586980L, -6395171416530257099L, -8293459145722885121L, 3780387183069559452L, 4617443032042788457L, -2683351855009630929L, phil_state_quaternion);
	phil_state_quaternion_attitude_quaternion_SET(&-3.8616065E37F, -4.5549693E37F, 1.1303494E38F, 3.8421509E37F, -8.309969E37F, -3.3154556E38F, -2.6287185E38F, 2.9588965E38F, -2.2582585E38F, 2.5958326E38F, phil_state_quaternion);
	phil_state_quaternion_rollspeed_SET(-5.854461E37F, -1.3292969E38F, -2.1959516E37F, 3.213006E38F, 3.155567E38F, -1.8518444E38F, 1.6397184E38F, 6.098881E37F, 2.1940673E38F, -2.2539815E38F, phil_state_quaternion);
	phil_state_quaternion_pitchspeed_SET(-2.6772113E38F, -3.2822052E38F, -2.1113615E38F, 3.2131063E38F, 2.6671809E38F, -1.7001313E38F, -1.220191E38F, -3.2420748E38F, 1.9681107E37F, -2.4118154E38F, phil_state_quaternion);
	phil_state_quaternion_yawspeed_SET(-3.3674684E37F, 2.4329795E38F, 3.242138E38F, 3.2988124E38F, 1.0025221E38F, -2.0929181E38F, -2.7546932E38F, 5.2275363E36F, 2.3797933E38F, 2.9755212E38F, phil_state_quaternion);
	phil_state_quaternion_lat_SET(1666694297, -217294257, -874454915, -761755732, 180648515, 1834292778, -706192232, 1095575548, -1996195787, -924926985, phil_state_quaternion);
	phil_state_quaternion_lon_SET(976331945, 567875802, 1182886284, 2078724076, -1099440201, -481792925, 1386620744, 1391719571, -173600140, -594239211, phil_state_quaternion);
	phil_state_quaternion_alt_SET(1260737438, -1171858800, -839694793, -784458257, -423645860, 2143110188, 1954114995, 2127342781, -1639465704, -652244604, phil_state_quaternion);
	phil_state_quaternion_vx_SET(11208, -3653, 4720, 31674, 12641, -18082, -7055, -28755, -28920, 23557, phil_state_quaternion);
	phil_state_quaternion_vy_SET(-31825, -28324, -29410, 8254, -2089, -112, -15007, -17283, 782, -24726, phil_state_quaternion);
	phil_state_quaternion_vz_SET(-11849, 21654, -10671, 20688, 19963, -24365, -15971, -1819, -25183, 2180, phil_state_quaternion);
	phil_state_quaternion_xacc_SET(22551, -30015, -27361, 3008, -28123, 21904, -16179, -3278, 14404, -31751, phil_state_quaternion);
	phil_state_quaternion_yacc_SET(17600, -20290, 7449, 9246, -27918, -7752, 22376, -22082, 14902, -26642, phil_state_quaternion);
	phil_state_quaternion_zacc_SET(24126, 5058, -25330, -15389, 2542, -9692, 18898, -31129, 23999, 7750, phil_state_quaternion);
	
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


void write_SENSOR_OFFSETS(psensor_offsets_SENSOR_OFFSETS *const psensor_offsets) {
	psensor_offsets_mag_ofs_x_SET(-8725, -23488, 30413, 25776, 17125, -25741, -5318, 30634, -13843, 14807, psensor_offsets);
	psensor_offsets_mag_ofs_y_SET(-7911, 27586, -25011, -6742, 30389, -27648, -29798, 29403, 7421, 413, psensor_offsets);
	psensor_offsets_mag_ofs_z_SET(-17771, -31999, -28324, -11219, -14506, -13194, 712, 29702, 11988, 14729, psensor_offsets);
	psensor_offsets_mag_declination_SET(-1.058024E36F, -2.8950477E38F, 2.625725E38F, -4.117934E37F, 4.471488E37F, 2.7168963E38F, 3.0063466E38F, -1.868275E38F, -1.4427383E38F, -2.4272367E38F, psensor_offsets);
	psensor_offsets_raw_press_SET(-294789851, -1833670986, -1514428161, 1716306582, -479921845, -1417900893, -371500935, -1908808354, -1652167182, -624453418, psensor_offsets);
	psensor_offsets_raw_temp_SET(-2135768533, 268599130, -295842201, 548660406, 1808436228, -505246062, 1381581569, -548052770, -1912569010, -1320647541, psensor_offsets);
	psensor_offsets_gyro_cal_x_SET(-9.055151E37F, -2.0760594E38F, -8.0111223E37F, -3.2261095E38F, -1.7847695E38F, 3.7767426E37F, -2.2409702E38F, -1.48792E38F, 3.129898E38F, -1.7290681E38F, psensor_offsets);
	psensor_offsets_gyro_cal_y_SET(7.1523514E37F, -3.2279593E38F, -1.7117597E38F, -2.0594221E38F, -2.753003E38F, 2.9894465E38F, 5.183165E36F, 2.4212335E38F, 6.240696E37F, -2.0761557E38F, psensor_offsets);
	psensor_offsets_gyro_cal_z_SET(3.38788E38F, -1.3000829E38F, -3.3477732E38F, -1.3681271E37F, 3.3866827E38F, 2.0647548E38F, -6.949797E37F, 3.160955E38F, 7.2289205E37F, 2.4282594E38F, psensor_offsets);
	psensor_offsets_accel_cal_x_SET(-2.2194526E38F, 2.6080252E38F, -1.2393967E38F, 1.2449733E38F, 1.7250045E38F, -3.2618086E38F, 2.9513612E38F, -2.462923E38F, 2.6169292E38F, 1.3107569E38F, psensor_offsets);
	psensor_offsets_accel_cal_y_SET(2.9117472E38F, -2.8913694E38F, 1.8409248E38F, -1.9145899E38F, -5.9499574E37F, 8.1441374E37F, -2.8596719E38F, -1.8346029E37F, 1.6410157E38F, 2.0012473E38F, psensor_offsets);
	psensor_offsets_accel_cal_z_SET(2.5532958E38F, 1.9643932E38F, -2.603463E38F, 2.2110114E38F, 6.181184E37F, 2.402221E38F, 1.493143E38F, -1.8209357E38F, 2.202053E38F, -9.7011786E36F, psensor_offsets);
	
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
	pstorage_information_time_boot_ms_SET(389460518, 1153842507, -542717113, -801198919, -150114803, -605793358, 610525944, -1857848159, 719135264, 647169980, pstorage_information);
	pstorage_information_storage_id_SET(-93, -95, -50, -22, -88, 67, 119, 104, 107, 99, pstorage_information);
	pstorage_information_storage_count_SET(117, -35, 5, 99, 14, -9, -99, -10, 2, 79, pstorage_information);
	pstorage_information_status_SET(-6, -24, 3, 77, 87, -18, -9, -66, -84, 59, pstorage_information);
	pstorage_information_total_capacity_SET(2.1967949E38F, -1.0097183E38F, -3.1404069E37F, 1.1142326E38F, -2.8697151E38F, -1.2490868E38F, 2.88709E38F, 2.76156E38F, -4.2605676E37F, -1.0292777E38F, pstorage_information);
	pstorage_information_used_capacity_SET(7.4275984E37F, -2.9023686E38F, 2.482834E38F, 5.915759E37F, 7.309981E37F, -2.403902E38F, -3.793699E37F, -2.8375069E38F, 6.9356653E37F, -2.9554002E38F, pstorage_information);
	pstorage_information_available_capacity_SET(-1.5816676E38F, 1.2615872E38F, -2.4569178E38F, 7.781017E37F, 2.0896861E38F, 3.2096246E38F, 2.25285E38F, 2.440811E38F, -3.3406822E38F, 8.1542193E37F, pstorage_information);
	pstorage_information_read_speed_SET(2.2762818E38F, -1.0941038E38F, -3.1045318E37F, -3.1595374E38F, -4.331714E37F, 3.1693772E38F, 1.0024594E38F, -2.1956931E38F, 1.3259905E38F, -1.4774706E38F, pstorage_information);
	pstorage_information_write_speed_SET(2.0678172E38F, 1.3374905E37F, 1.5861641E38F, -9.012295E37F, 2.1760166E38F, 1.3907551E38F, 3.1850263E38F, -1.6082877E38F, -1.4091559E38F, -2.9994852E38F, pstorage_information);
	
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
	pcamera_information_resolution_h_SET(-21840, -18137, -14332, 27246, -3536, 5902, 25884, 1060, 23585, -10919, pcamera_information);
	pcamera_information_resolution_v_SET(29452, 21096, 18421, 3188, 10029, -27411, -22746, -13742, -13555, -21539, pcamera_information);
	pcamera_information_cam_definition_version_SET(-15001, 6842, 1096, 23034, -25756, -24748, 25687, -29855, -20458, 16010, pcamera_information);
	pcamera_information_time_boot_ms_SET(348579373, -434138576, -986696625, 790933877, 194586252, -1040813902, 1250451754, -347731704, 2125069396, -558144785, pcamera_information);
	pcamera_information_firmware_version_SET(1810615931, 2006704832, -1396891271, 602015888, -51876064, 1795935213, 493287540, 2116289020, -1855845570, -46687017, pcamera_information);
	pcamera_information_vendor_name_SET(&8, -20, -47, -125, 81, 31, 83, 44, -23, -76, pcamera_information);
	pcamera_information_model_name_SET(&28, 94, 56, -86, 100, 114, 17, 6, 7, -15, pcamera_information);
	pcamera_information_focal_length_SET(8.511773E37F, -1.9376652E38F, 2.8584769E38F, 1.1070834E38F, 2.3063501E38F, -5.0041806E37F, -2.1393563E38F, 2.4901424E38F, -3.2360666E37F, -1.1663865E38F, pcamera_information);
	pcamera_information_sensor_size_h_SET(-2.156243E38F, -5.1183923E37F, 5.8779843E37F, 3.0356717E38F, -2.585124E38F, 5.735876E37F, -1.5996402E38F, 2.1698647E38F, 1.1946446E38F, 2.6886725E38F, pcamera_information);
	pcamera_information_sensor_size_v_SET(-2.5316197E38F, -7.43827E36F, 9.169902E37F, -2.9863686E38F, -2.4024352E38F, 3.3053428E38F, -2.1303223E38F, 2.1798428E38F, 1.383635E38F, 2.9376614E38F, pcamera_information);
	pcamera_information_lens_id_SET(0, 44, 125, -100, 120, 38, -73, 67, -88, -5, pcamera_information);
	pcamera_information_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE, e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE, pcamera_information);
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


void read_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *const pdevice_op_write_reply) {
	int32_t request_id = pdevice_op_write_reply_request_id_GET(pdevice_op_write_reply);
	int8_t  result     = pdevice_op_write_reply_result_GET(pdevice_op_write_reply);
	
}


void write_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY *const pdevice_op_write_reply) {
	pdevice_op_write_reply_request_id_SET(-942495108, -1108053958, -1320453735, -1360878417, -572651977, -1219468241, 197196837, 2050703230, 185009172, 1124905544, pdevice_op_write_reply);
	pdevice_op_write_reply_result_SET(67, -116, -54, -74, -103, 106, -71, 14, 121, -110, pdevice_op_write_reply);
	
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
	pterrain_data_grid_spacing_SET(-19051, -10593, -20752, 21404, 28964, 2418, -15986, 4166, -25573, -4278, pterrain_data);
	pterrain_data_lat_SET(-1846884825, -266681823, -660495916, 1397266795, -763821219, -1749198407, 1907119945, 1858396112, 1876648418, 1452150144, pterrain_data);
	pterrain_data_lon_SET(687800753, 1433195940, -1811449412, -1978140476, 908314002, -364499644, 1484816833, 823731713, 1110620349, 640387342, pterrain_data);
	pterrain_data_gridbit_SET(-16, 22, -42, -57, 79, -43, 71, 119, 113, 78, pterrain_data);
	pterrain_data_daTa_SET(&-15063, -27511, -26406, 16668, -26661, -20686, 13439, 10597, 286, -18292, pterrain_data);
	
}


void read_GIMBAL_CONTROL(pgimbal_control_GIMBAL_CONTROL *const pgimbal_control) {
	int8_t target_system    = pgimbal_control_target_system_GET(pgimbal_control);
	int8_t target_component = pgimbal_control_target_component_GET(pgimbal_control);
	float  demanded_rate_x  = pgimbal_control_demanded_rate_x_GET(pgimbal_control);
	float  demanded_rate_y  = pgimbal_control_demanded_rate_y_GET(pgimbal_control);
	float  demanded_rate_z  = pgimbal_control_demanded_rate_z_GET(pgimbal_control);
	
}


void write_GIMBAL_CONTROL(pgimbal_control_GIMBAL_CONTROL *const pgimbal_control) {
	pgimbal_control_target_system_SET(-107, 4, -8, -80, 107, 23, 19, -75, 113, 69, pgimbal_control);
	pgimbal_control_target_component_SET(67, -55, -24, -6, -32, -122, -45, 65, -23, -117, pgimbal_control);
	pgimbal_control_demanded_rate_x_SET(8.4821463E37F, 1.5116558E38F, -5.714988E37F, 8.549948E37F, 8.918213E37F, -2.8218374E37F, -2.051493E38F, 3.0018227E37F, -1.8856161E37F, -1.8890947E38F, pgimbal_control);
	pgimbal_control_demanded_rate_y_SET(-2.934591E38F, 5.374779E35F, 2.7172297E38F, 1.9377236E38F, 1.8655973E38F, -1.0407343E38F, 7.9601095E37F, -5.9766623E37F, 1.9921222E38F, -7.4910196E36F, pgimbal_control);
	pgimbal_control_demanded_rate_z_SET(9.219464E36F, -1.5737956E38F, -3.6662626E37F, 2.8232325E38F, 1.8668419E38F, 1.868774E38F, -1.483793E38F, -2.8008334E38F, -7.2162975E36F, -2.8537758E38F, pgimbal_control);
	
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


void write_VIDEO_STREAM_INFORMATION(pvideo_stream_information_VIDEO_STREAM_INFORMATION *const pvideo_stream_information) {
	pvideo_stream_information_resolution_h_SET(24428, 11330, 31352, 15819, -558, 12974, -14236, -16829, -16406, 25596, pvideo_stream_information);
	pvideo_stream_information_resolution_v_SET(-5313, 14899, -1995, -12327, -3313, 457, -30383, -1514, -30881, -17657, pvideo_stream_information);
	pvideo_stream_information_rotation_SET(30182, -29545, -18353, 23538, 32469, -12088, -15992, -9482, -288, -2063, pvideo_stream_information);
	pvideo_stream_information_bitrate_SET(1241124885, 339176866, -1058957755, -727171581, -2052937784, 222715438, 1797912817, 841478431, -194995930, -1693909882, pvideo_stream_information);
	pvideo_stream_information_camera_id_SET(-30, -18, 79, 48, -33, -34, 71, -105, -66, -44, pvideo_stream_information);
	pvideo_stream_information_status_SET(102, 11, 46, 110, -120, -74, -19, 95, 109, 100, pvideo_stream_information);
	pvideo_stream_information_framerate_SET(1.371096E37F, 2.1413355E38F, 2.6871444E38F, 1.2793094E38F, -1.7722542E38F, 2.752502E38F, -1.8101442E38F, -2.9001487E38F, 6.8793405E37F, -1.7000228E38F, pvideo_stream_information);
	pvideo_stream_information_uri_SET(some_string, strlen(some_string), pvideo_stream_information);
	
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


void write_AHRS(pahrs_AHRS *const pahrs) {
	pahrs_omegaIx_SET(-9.224992E37F, 2.4708403E38F, -2.3339291E38F, 1.7746727E38F, 1.8796918E38F, 2.5329417E38F, -1.9896553E38F, 2.8660213E38F, -1.6915038E37F, 5.9469673E37F, pahrs);
	pahrs_omegaIy_SET(-5.441729E37F, -3.3670911E38F, -9.02829E37F, 2.937356E37F, -1.0593233E38F, 2.1360984E38F, 2.9290037E38F, -1.8549174E38F, -2.4432324E38F, -9.086752E37F, pahrs);
	pahrs_omegaIz_SET(3.1637038E38F, 1.581562E38F, 2.173881E37F, -3.8867676E37F, 2.636968E38F, 3.2484822E38F, 2.6468279E38F, -2.0057928E38F, 2.488817E38F, 1.437792E38F, pahrs);
	pahrs_accel_weight_SET(1.7917619E38F, -9.654268E37F, 1.1677408E38F, 1.7301816E38F, -1.4814262E38F, -2.019562E38F, -6.4953357E37F, 1.7469846E38F, 9.583737E36F, -1.5140426E38F, pahrs);
	pahrs_renorm_val_SET(-2.1308432E38F, 9.457668E37F, 2.1541521E38F, 7.737515E37F, -2.649371E38F, 2.1875901E38F, 2.5787447E38F, 9.928851E37F, -2.8055994E38F, -7.239192E37F, pahrs);
	pahrs_error_rp_SET(-4.718643E37F, -1.126257E38F, -1.3444473E38F, 1.0761017E38F, -3.2956745E38F, 3.3763924E38F, 8.1317945E37F, 6.4211254E37F, -2.6897057E38F, -1.1803086E38F, pahrs);
	pahrs_error_yaw_SET(1.3606402E38F, -6.13392E37F, 2.1647787E38F, 1.048061E38F, -8.148538E37F, 1.0308335E38F, -1.046669E38F, 3.0270592E38F, 1.805941E38F, 6.840773E37F, pahrs);
	
}


void read_DEBUG(pdebug_DEBUG *const pdebug) {
	int32_t time_boot_ms = pdebug_time_boot_ms_GET(pdebug);
	int8_t  ind          = pdebug_ind_GET(pdebug);
	float   value        = pdebug_value_GET(pdebug);
	
}


void write_DEBUG(pdebug_DEBUG *const pdebug) {
	pdebug_time_boot_ms_SET(1858272756, 114832370, -78633300, 241295966, 988456181, 1038713879, -806181194, 1216337975, 2058314366, -1725328389, pdebug);
	pdebug_ind_SET(-58, -38, 44, 119, 77, 104, -78, -66, 72, 39, pdebug);
	pdebug_value_SET(2.8609477E38F, 1.1440474E38F, 2.825318E38F, 2.8726212E38F, 7.7781885E37F, 3.390987E38F, 2.8226204E38F, 3.0281942E38F, -2.7780109E38F, 6.6922527E36F, pdebug);
	
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
	pcamera_image_captured_time_boot_ms_SET(-1864146084, -1688626605, 1456092308, 1430773610, -239246648, 1708159322, 1423552083, 605354116, -728802885, -1631365114, pcamera_image_captured);
	pcamera_image_captured_time_utc_SET(4086893558862410338L, 2138241241112464754L, 166099317031168465L, -685947904279492484L, -773720401438084281L, -2038495972143141848L, 8043908120318403668L, 1883083816507408017L, -2044653832028210651L, -2356580251282590365L, pcamera_image_captured);
	pcamera_image_captured_camera_id_SET(-75, 45, -76, -115, 94, -119, -37, 125, 57, -81, pcamera_image_captured);
	pcamera_image_captured_lat_SET(-2101581372, 473373350, 179382749, -1463555429, -802696224, -500339758, -2049997522, -1385239, 1054395849, 1492383169, pcamera_image_captured);
	pcamera_image_captured_lon_SET(575238610, 329703068, 410075339, -1360120567, -66688548, -1209776908, 1589117310, 953966807, -2105236504, 2086815755, pcamera_image_captured);
	pcamera_image_captured_alt_SET(-404819742, -1427254129, 814958566, -977382021, -348620674, -118809386, -1506226025, -34406206, 1844031411, -592004252, pcamera_image_captured);
	pcamera_image_captured_relative_alt_SET(-1232011557, -718900002, -41838794, -78413968, 328810817, 146626438, 1818957221, -2118544787, 210521887, 416082621, pcamera_image_captured);
	pcamera_image_captured_q_SET(&-2.6959878E38F, -8.2771185E37F, 1.3066517E38F, 2.6092513E38F, 1.0280264E38F, 3.1126475E38F, 2.9452128E38F, -4.726169E37F, 6.017857E37F, 2.8196834E37F, pcamera_image_captured);
	pcamera_image_captured_image_index_SET(-1506935166, -848238702, -431408704, -1115407141, -404972578, 1688406698, 1633622313, 744130779, 54552927, 985104728, pcamera_image_captured);
	pcamera_image_captured_capture_result_SET(-34, -12, 27, 67, 55, 123, 47, 52, 4, 60, pcamera_image_captured);
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
	plog_entry_id_SET(-21033, 7291, 4251, 31576, 31048, -17159, -19163, -27084, 23017, -19746, plog_entry);
	plog_entry_num_logs_SET(-3877, 3615, 1279, 149, -31060, 20419, -16761, 5731, -24694, 15302, plog_entry);
	plog_entry_last_log_num_SET(-32628, 10895, -29198, 27853, 4701, -15132, 19000, 16045, 15455, 27855, plog_entry);
	plog_entry_time_utc_SET(1847556260, 2006414114, -1441511108, -441031243, 862610581, 1500152724, -820463704, 916026333, -1025838521, -714590833, plog_entry);
	plog_entry_size_SET(1192494834, -1723354248, 1800109546, 1931376250, -1078357951, 97028156, 764396265, -1033344942, -1825947220, -768158581, plog_entry);
	
}


void read_ACTUATOR_CONTROL_TARGET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *const pactuator_control_target) {
	int64_t                           time_usec     = pactuator_control_target_time_usec_GET(pactuator_control_target);
	int8_t                            group_mlx     = pactuator_control_target_group_mlx_GET(pactuator_control_target);
	Vactuator_control_target_controls item_controls = pactuator_control_target_controls_GET(pactuator_control_target);
	for (size_t                       index         = 0; index < item_controls.len; index++)
		some_float = vactuator_control_target_controls_GET(&item_controls, index);
	
}


void write_ACTUATOR_CONTROL_TARGET(pactuator_control_target_ACTUATOR_CONTROL_TARGET *const pactuator_control_target) {
	pactuator_control_target_time_usec_SET(-6031016372560827674L, -2073671072132076000L, -5355392295177036156L, 4520219995843973755L, -1434660239470221070L, -7798025508352439838L, 7174194470137474179L, -4031396986658382998L, -3132686428183682172L, 1718610698887659516L, pactuator_control_target);
	pactuator_control_target_group_mlx_SET(-75, -98, -49, -104, -96, -46, -109, 34, 120, 60, pactuator_control_target);
	pactuator_control_target_controls_SET(&2.9017E38F, -3.327488E38F, 3.4887458E37F, -1.6931246E38F, -2.6469232E38F, -1.4173941E38F, 6.46497E37F, -9.298763E37F, 3.306727E37F, -1.87417E38F, pactuator_control_target);
	
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
	phigh_latency_heading_SET(-21543, -16057, -17363, 16707, -12876, -10076, 15882, -32305, 9572, -18349, phigh_latency);
	phigh_latency_wp_distance_SET(-20454, 10693, -5527, -28060, -26414, 9260, -17339, -30665, -20016, 29518, phigh_latency);
	phigh_latency_custom_mode_SET(871041667, 125773871, 298940890, 171330223, -1233897485, -2057812030, 87122559, -1797647545, 723406053, 716416459, phigh_latency);
	phigh_latency_roll_SET(-8514, 23259, -1110, -11455, -14011, -14430, -16003, -16764, 134, -1655, phigh_latency);
	phigh_latency_pitch_SET(25949, -24693, -20096, -21838, 26007, 21810, 17130, 8656, -20917, -21659, phigh_latency);
	phigh_latency_throttle_SET(-44, 80, -32, 30, 120, 104, 22, 127, 64, -72, phigh_latency);
	phigh_latency_heading_sp_SET(25119, -31320, -28587, 9658, 553, 13589, 27684, -26862, 16665, -19007, phigh_latency);
	phigh_latency_latitude_SET(1712379713, -1237695359, -573229331, -312806812, 1664494025, -314212974, 89784369, -1196971201, 1981548734, -478597836, phigh_latency);
	phigh_latency_longitude_SET(1612971045, 1781725209, 1905009936, 519721060, -458827550, 674750568, -1880248068, 1531986384, -212684955, -882409760, phigh_latency);
	phigh_latency_altitude_amsl_SET(-3320, -16004, -17324, -30315, -13171, 28072, 1901, 27425, -3786, -4220, phigh_latency);
	phigh_latency_altitude_sp_SET(16360, -25831, 29191, 18569, -5031, 25219, -14707, -10999, -617, -14217, phigh_latency);
	phigh_latency_airspeed_SET(-27, -1, -62, -107, -126, 82, 34, -25, -43, 32, phigh_latency);
	phigh_latency_airspeed_sp_SET(-4, 106, -85, -54, -78, -74, 92, -50, 81, 75, phigh_latency);
	phigh_latency_groundspeed_SET(-86, -78, 93, 29, 25, -15, 74, -101, -99, 8, phigh_latency);
	phigh_latency_climb_rate_SET(-95, -99, -91, -66, 23, -25, -97, -99, 19, -127, phigh_latency);
	phigh_latency_gps_nsat_SET(33, -98, 63, 41, 107, 98, 18, -19, -82, -48, phigh_latency);
	phigh_latency_battery_remaining_SET(115, 23, 86, -1, 76, 24, -67, -95, -8, 80, phigh_latency);
	phigh_latency_temperature_SET(98, -54, 64, -81, -47, 43, 98, 28, -52, 22, phigh_latency);
	phigh_latency_temperature_air_SET(86, -92, 103, 100, -10, 127, 110, -16, -105, -66, phigh_latency);
	phigh_latency_failsafe_SET(85, 27, 90, 112, -98, -71, -42, 32, 54, -26, phigh_latency);
	phigh_latency_wp_num_SET(114, -51, -75, -72, 56, -25, 75, 123, 88, 108, phigh_latency);
	phigh_latency_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, phigh_latency);
	phigh_latency_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, phigh_latency);
	phigh_latency_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, phigh_latency);
	
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
	pfollow_target_timestamp_SET(1120741668197113255L, 5025445293596053098L, 3624930154053166359L, 2809584456188944819L, 8467522685833673286L, -5479663942834573408L, -5566826857419998396L, -3237415289597921449L, -7591051743727858449L, 8425699377597908607L, pfollow_target);
	pfollow_target_custom_state_SET(6977001252467624998L, 8174221194250239729L, -372605608877663031L, 800730327455400202L, -140160320016271978L, 4244387729443498480L, 2281338080813454720L, -6664286816631055762L, -9084949165008585999L, -6447454226903153047L, pfollow_target);
	pfollow_target_est_capabilities_SET(6, 49, 19, 60, 52, -7, 0, 80, -17, 0, pfollow_target);
	pfollow_target_lat_SET(-2024154413, 195690734, -1227287621, 580277262, -1537930783, 1731830560, 1955080589, 389492939, -852205780, -544127791, pfollow_target);
	pfollow_target_lon_SET(-518632610, -1769001724, -750998822, -1822970252, -1575364607, 492425997, 355672395, 1991519239, 1083656651, 115822512, pfollow_target);
	pfollow_target_alt_SET(2.11333E38F, 1.0446705E37F, 1.2141717E38F, -3.1436713E38F, -2.7461044E38F, 1.9656227E38F, 1.1057626E38F, -9.884752E37F, -6.2474416E37F, 5.6035993E37F, pfollow_target);
	pfollow_target_vel_SET(&1.2912066E38F, -3.0064433E38F, 2.2765893E38F, 3.1504752E38F, -1.882136E38F, -5.9061084E37F, 6.506463E37F, -1.6762181E38F, 9.091857E37F, 1.1703596E38F, pfollow_target);
	pfollow_target_acc_SET(&2.8678772E37F, -4.6373924E37F, 2.4542752E38F, -3.0086006E38F, -3.2330794E38F, -1.3070538E38F, -3.7782116E37F, 1.9893107E38F, 2.0663366E38F, 3.0932502E38F, pfollow_target);
	pfollow_target_attitude_q_SET(&-3.3446779E38F, -7.389099E37F, -1.1467791E38F, -1.6351323E38F, -1.1235354E38F, -2.0013834E38F, 1.6742401E38F, -2.9287589E38F, -2.7899266E38F, -3.273028E38F, pfollow_target);
	pfollow_target_rates_SET(&1.0209724E38F, 5.2813894E37F, 2.3286239E38F, -2.76288E38F, -1.2705273E36F, -5.800673E36F, 9.87247E37F, -1.8780434E38F, 8.3485724E37F, 1.118375E38F, pfollow_target);
	pfollow_target_position_cov_SET(&1.0903965E38F, 1.9419225E38F, -2.5881653E38F, -7.740292E37F, 1.7629012E38F, -3.1730323E38F, -9.721948E37F, 1.3180023E38F, 1.6261413E37F, 8.6800625E36F, pfollow_target);
	
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
	phome_position_latitude_SET(-1407331670, -1868426637, -1471170502, 1077906866, -2147315924, -1547585243, 545133901, -172552893, 349254907, 215415262, phome_position);
	phome_position_longitude_SET(1788008781, -1370121339, -1681910217, 2081402166, 1140419884, -1453164465, -1554963418, -813999596, -1341617368, 1204153102, phome_position);
	phome_position_altitude_SET(603608160, -959015562, -1760633124, -1806699372, 267729042, -1282216464, 1068831684, -1829953428, 625773929, 1872609062, phome_position);
	phome_position_x_SET(-1.9950673E38F, -1.3726996E38F, 9.174586E37F, 4.263095E37F, 1.3248801E38F, -7.1337727E37F, 1.3485377E38F, -2.4307131E38F, 1.7577091E38F, 2.347012E38F, phome_position);
	phome_position_y_SET(3.0523572E38F, -2.3615742E38F, 5.0864886E37F, 4.44509E37F, -1.3787179E38F, -1.712727E38F, -3.0611397E38F, -2.4575388E38F, 1.6421913E38F, -1.316226E38F, phome_position);
	phome_position_z_SET(2.3127379E38F, 8.889388E37F, 2.3268786E38F, -2.3254541E38F, 1.1705813E38F, 2.9784282E38F, -1.930405E38F, 3.1894606E38F, 1.1366747E38F, 2.1629172E38F, phome_position);
	phome_position_q_SET(&-2.1931398E38F, 2.9988025E38F, 2.6355152E38F, 3.085648E38F, 1.1805589E38F, -4.0266746E37F, 2.3638335E37F, -1.0619114E38F, -1.4032835E37F, 2.4673634E38F, phome_position);
	phome_position_approach_x_SET(-8.2950436E37F, 1.9847518E38F, 2.2681762E38F, 2.6415056E38F, 3.115224E38F, 1.8699357E38F, 2.8776922E38F, 2.3158932E38F, 1.7661922E38F, -1.2705373E38F, phome_position);
	phome_position_approach_y_SET(2.0735358E38F, 2.7800235E38F, 1.4171894E38F, 2.7071175E38F, 3.0640348E38F, -2.0164435E38F, 3.2793265E38F, -1.5623771E38F, 1.5538218E38F, -2.636851E38F, phome_position);
	phome_position_approach_z_SET(1.4823578E38F, 2.4205472E38F, -2.4942965E38F, 2.9886378E38F, -3.376201E38F, -2.2831691E38F, 2.6773925E38F, 3.0154794E37F, -1.68067E38F, 2.3034464E37F, phome_position);
	phome_position_time_usec_SET(-1407672325694774073L, 1188874384256584311L, -24057161283499219L, -5196846070542895217L, -7668779397687593108L, 6398587179376311463L, 2164680679114045243L, 4171922924130836449L, 4145244556448209513L, -1300702386012295893L, phome_position);
	
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


void write_FENCE_STATUS(pfence_status_FENCE_STATUS *const pfence_status) {
	pfence_status_breach_count_SET(13444, 30356, 23835, -16445, -30733, -23375, 21681, 23966, 21623, -10278, pfence_status);
	pfence_status_breach_time_SET(-1615260501, 1516765338, 978615775, 221390394, -956186528, 1408528090, -1950596383, 1709784748, -1407120746, 972571325, pfence_status);
	pfence_status_breach_status_SET(66, 19, -58, -112, 34, 81, -41, -48, 37, 116, pfence_status);
	pfence_status_breach_type_SET(e_FENCE_BREACH_FENCE_BREACH_BOUNDARY, e_FENCE_BREACH_FENCE_BREACH_NONE, e_FENCE_BREACH_FENCE_BREACH_BOUNDARY, e_FENCE_BREACH_FENCE_BREACH_BOUNDARY, e_FENCE_BREACH_FENCE_BREACH_MINALT, e_FENCE_BREACH_FENCE_BREACH_NONE, e_FENCE_BREACH_FENCE_BREACH_MAXALT, e_FENCE_BREACH_FENCE_BREACH_NONE, e_FENCE_BREACH_FENCE_BREACH_NONE, e_FENCE_BREACH_FENCE_BREACH_MAXALT, pfence_status);
	
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


void write_REMOTE_LOG_BLOCK_STATUS(premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *const premote_log_block_status) {
	premote_log_block_status_seqno_SET(-1330935507, 1431570678, 1817892281, -191151469, -998984434, -160813709, 383399410, -1017960426, -1388243722, 1942838013, premote_log_block_status);
	premote_log_block_status_target_system_SET(-108, 66, 103, -97, -76, -81, -102, -5, 82, 113, premote_log_block_status);
	premote_log_block_status_target_component_SET(115, 83, -77, 23, 21, -79, 99, -73, -31, -66, premote_log_block_status);
	premote_log_block_status_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK, e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK, premote_log_block_status);
	
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


void write_OBSTACLE_DISTANCE(pobstacle_distance_OBSTACLE_DISTANCE *const pobstacle_distance) {
	pobstacle_distance_distances_SET(&1246, -259, -20465, -25826, 14936, 3495, -14143, -13200, 30742, -15133, pobstacle_distance);
	pobstacle_distance_min_distance_SET(-26723, 3498, 26540, 5959, -31105, -24106, 32, -7584, -2861, 21655, pobstacle_distance);
	pobstacle_distance_max_distance_SET(15623, 1746, 22387, -14016, 16537, 19, -6854, -20383, -22393, 22689, pobstacle_distance);
	pobstacle_distance_time_usec_SET(5737766608599659370L, -4080914471899775829L, 3291219816492790268L, -1387733695931819998L, 8906822588987718551L, -7591935365867605700L, -5162781515319643672L, 1172763479290576016L, -7137694720244011983L, 8925959858689805372L, pobstacle_distance);
	pobstacle_distance_increment_SET(46, 89, -20, 65, -118, -5, -18, -28, 98, -6, pobstacle_distance);
	pobstacle_distance_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, pobstacle_distance);
	
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
	pgps2_raw_eph_SET(22737, -3693, -22386, -24368, 9449, 26671, 6994, -6925, 4283, 23190, pgps2_raw);
	pgps2_raw_epv_SET(19278, -6237, 11480, -3110, 26629, -23750, -29396, -6205, 2937, 870, pgps2_raw);
	pgps2_raw_vel_SET(-17719, -16228, 16205, 28332, -884, -13464, 6346, -2028, 5417, -21297, pgps2_raw);
	pgps2_raw_cog_SET(28460, -32559, 26269, 25566, 523, -13265, -6661, 24131, 9927, 18485, pgps2_raw);
	pgps2_raw_dgps_age_SET(1885146598, 598551966, 1093984745, -738810847, -1632832290, 1596040869, 511586432, -1729712032, 1581583886, -1858650477, pgps2_raw);
	pgps2_raw_time_usec_SET(-627676148858838732L, -1053201834844839168L, -1194267323110685595L, 1664299870843455179L, -4964760436355842065L, 698765463925384377L, -2443954257471288588L, 730800082858150095L, 2496359302176804314L, -1421980643254902467L, pgps2_raw);
	pgps2_raw_lat_SET(1886335916, 334571466, -1504034774, -105696866, -1023573990, -459756569, 2091286308, 1826597760, -1809100242, -1451974951, pgps2_raw);
	pgps2_raw_lon_SET(-895766687, -61715423, 1719577915, 142240280, -209422218, 149632006, 2030846762, -490261209, 1347953067, -29810320, pgps2_raw);
	pgps2_raw_alt_SET(129193126, -1481390879, 1378860607, -1412886392, 376854618, -585393029, -1604891664, 1160090955, -1721710975, 116798673, pgps2_raw);
	pgps2_raw_satellites_visible_SET(-38, 14, -127, -41, -114, -66, -38, 13, -23, 44, pgps2_raw);
	pgps2_raw_dgps_numch_SET(-96, 62, -117, -87, -38, -102, 56, 67, -44, -121, pgps2_raw);
	pgps2_raw_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, pgps2_raw);
	
}


void read_REQUEST_DATA_STREAM(prequest_data_stream_REQUEST_DATA_STREAM *const prequest_data_stream) {
	int16_t req_message_rate = prequest_data_stream_req_message_rate_GET(prequest_data_stream);
	int8_t  target_system    = prequest_data_stream_target_system_GET(prequest_data_stream);
	int8_t  target_component = prequest_data_stream_target_component_GET(prequest_data_stream);
	int8_t  req_stream_id    = prequest_data_stream_req_stream_id_GET(prequest_data_stream);
	int8_t  start_stop       = prequest_data_stream_start_stop_GET(prequest_data_stream);
	
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
	pmemory_vect_address_SET(-10591, -21618, 28517, -18682, 6064, -19848, 10370, -17829, -2196, 11164, pmemory_vect);
	pmemory_vect_ver_SET(70, -20, -95, 117, 4, -65, 28, -116, -125, -126, pmemory_vect);
	pmemory_vect_typE_SET(-31, 82, -17, 32, 41, -19, -115, 24, -26, -117, pmemory_vect);
	pmemory_vect_value_SET(&-12, 121, -123, 25, 90, -110, -119, -11, 25, 64, pmemory_vect);
	
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


void write_PARAM_EXT_REQUEST_READ(pparam_ext_request_read_PARAM_EXT_REQUEST_READ *const pparam_ext_request_read) {
	pparam_ext_request_read_target_system_SET(35, 1, 110, -9, -63, -75, 42, 57, 87, 37, pparam_ext_request_read);
	pparam_ext_request_read_target_component_SET(-65, 63, 22, 107, -67, -47, 32, -93, -114, -33, pparam_ext_request_read);
	pparam_ext_request_read_param_index_SET(26434, 11275, 22996, 19949, -19894, 26968, -20831, 851, -24795, 4301, pparam_ext_request_read);
	pparam_ext_request_read_param_id_SET(some_string, strlen(some_string), pparam_ext_request_read);
	
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
	phil_sensor_fields_updated_SET(-1200275170, -1215311368, 54177999, -892760334, 1678198060, -1150784687, -1035472408, -1444160972, 1830283801, -1254631787, phil_sensor);
	phil_sensor_time_usec_SET(-5650643594793495602L, 5196282813219957067L, -1514591173554419850L, 2058697229496180184L, 4050396061497025289L, -8433603852800715278L, -3879143132645664287L, 902027444061804487L, 109800151809737656L, -8032556505051753519L, phil_sensor);
	phil_sensor_xacc_SET(-2.9217809E38F, 1.0433578E38F, 2.0371975E38F, -2.9254964E38F, 9.782631E37F, -2.3883912E38F, -5.504146E37F, 2.311348E37F, -2.5741907E38F, 3.393173E37F, phil_sensor);
	phil_sensor_yacc_SET(5.2259915E37F, 1.0922213E38F, -2.4273574E38F, 1.3670864E38F, -7.216435E36F, -3.1889083E38F, 2.4438977E38F, 2.6747367E38F, 9.460443E37F, -6.0220117E37F, phil_sensor);
	phil_sensor_zacc_SET(3.1518674E38F, -2.8677682E38F, -8.686825E37F, 1.9472176E38F, 3.1300172E37F, -2.7914305E38F, -2.5831746E38F, 5.366241E37F, 1.7092512E38F, -1.4369604E38F, phil_sensor);
	phil_sensor_xgyro_SET(-1.2118234E38F, -1.908663E38F, 2.3761113E38F, -2.000835E38F, -1.8022105E38F, -2.1498871E38F, 2.9686576E38F, -1.7811967E38F, 1.7358974E38F, -1.0568799E38F, phil_sensor);
	phil_sensor_ygyro_SET(9.920955E37F, 6.5906346E37F, 1.4547757E38F, 7.589744E37F, -7.8267435E37F, 1.4753792E38F, -1.2238399E38F, -6.733391E36F, -2.232825E37F, -6.029906E37F, phil_sensor);
	phil_sensor_zgyro_SET(-1.254349E37F, 3.3448876E38F, 1.1921136E38F, 3.2525513E38F, 1.1097404E38F, -3.362773E38F, -3.1758365E38F, -3.2334996E38F, 5.519E37F, 2.7946331E38F, phil_sensor);
	phil_sensor_xmag_SET(-1.0231574E38F, 2.2315492E38F, -1.8018217E38F, 1.4260067E37F, 2.5555183E38F, -2.8946879E38F, -2.6328933E38F, 2.0110383E37F, 2.0243177E38F, 1.5908484E38F, phil_sensor);
	phil_sensor_ymag_SET(-1.9364827E38F, 2.5250841E38F, -8.702565E37F, -3.1320125E38F, 2.5486235E38F, 1.6200813E38F, -1.9960923E38F, -4.9683537E37F, 5.832331E37F, 4.787382E37F, phil_sensor);
	phil_sensor_zmag_SET(2.6694602E38F, -3.308356E38F, 5.007666E37F, -3.0631732E37F, -1.7155475E38F, -3.219687E38F, 2.1216007E38F, -1.987611E38F, 1.6714646E38F, -1.4490986E38F, phil_sensor);
	phil_sensor_abs_pressure_SET(7.503974E37F, 2.3280918E38F, 4.2923283E36F, -1.2382018E38F, 2.5369915E38F, -3.001423E38F, 8.92647E37F, 2.0605275E38F, -2.9582765E38F, 3.3843586E38F, phil_sensor);
	phil_sensor_diff_pressure_SET(-9.197226E37F, 1.0707201E38F, -7.451348E37F, -1.3428891E38F, -2.790184E38F, -3.227416E38F, 3.3572304E38F, -1.9229578E38F, 1.1895284E38F, 4.712919E37F, phil_sensor);
	phil_sensor_pressure_alt_SET(2.3963722E38F, -2.0922786E38F, 3.1266628E38F, 2.876441E38F, 3.3782862E38F, 8.3190635E37F, 2.9350513E38F, 2.757056E38F, -9.3733666E36F, 2.8449158E38F, phil_sensor);
	phil_sensor_temperature_SET(3.0322233E38F, -1.9515511E38F, -8.315425E37F, -6.911281E37F, -8.711378E37F, -1.5876171E38F, 6.560391E36F, -8.025823E37F, 2.9702135E38F, -3.49928E37F, phil_sensor);
	
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
	psetup_signing_initial_timestamp_SET(9161192757305826964L, -1927422780851390802L, 8821161587056135715L, -6956316200746071342L, -1107194395928713021L, 7796566154085553855L, 8033756058618791090L, 4641623577253091636L, -8777414543555414613L, -2299398181104503384L, psetup_signing);
	psetup_signing_target_system_SET(-65, -58, 105, 56, -52, 0, 17, -111, 70, 121, psetup_signing);
	psetup_signing_target_component_SET(-16, 80, -115, 59, -57, -79, 117, -106, 77, 45, psetup_signing);
	psetup_signing_secret_key_SET(&-29, 21, -54, -103, -18, -30, -123, -18, 5, -34, psetup_signing);
	
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
	pgps_rtk_wn_SET(5180, -26260, -31601, -3982, -17256, -30901, -26077, -22459, 18742, -19744, pgps_rtk);
	pgps_rtk_time_last_baseline_ms_SET(-1611439768, 538547559, -1960553200, 81689295, 2109957993, -79500947, -2106148454, -340577009, -182740214, -541338810, pgps_rtk);
	pgps_rtk_tow_SET(-1346895706, 43362565, 698947557, -293683368, -692047384, -174149383, -440275859, -267833692, 832002362, -619825661, pgps_rtk);
	pgps_rtk_accuracy_SET(2138354238, 621646324, -1898107492, 2017877637, 627205887, 378252642, 1223026258, -1136416073, 1174487112, -140412773, pgps_rtk);
	pgps_rtk_rtk_receiver_id_SET(111, 24, 97, -95, 9, 61, -73, 87, -17, -102, pgps_rtk);
	pgps_rtk_rtk_health_SET(13, 48, 50, 16, -97, -61, 93, -48, 107, -76, pgps_rtk);
	pgps_rtk_rtk_rate_SET(95, 78, 97, 9, -94, -55, -68, -96, -49, 56, pgps_rtk);
	pgps_rtk_nsats_SET(-3, -114, -92, 33, 66, -69, 0, 119, -37, -57, pgps_rtk);
	pgps_rtk_baseline_coords_type_SET(55, -93, -125, 54, -23, 85, 85, -83, -106, -101, pgps_rtk);
	pgps_rtk_baseline_a_mm_SET(1257958054, -147094703, -1336605452, -313820596, 669810130, -1825006377, 1743289394, -1139170636, 1085523761, 2066245080, pgps_rtk);
	pgps_rtk_baseline_b_mm_SET(-1038907129, 1740069945, 1717414000, 515833725, -751564019, -261430987, 1274423051, -1991588665, 704841533, 2044080405, pgps_rtk);
	pgps_rtk_baseline_c_mm_SET(-386341155, -2107575657, -939753400, 50350194, 1348111584, 1270705949, 262277150, 1632680704, -1388036866, 615868660, pgps_rtk);
	pgps_rtk_iar_num_hypotheses_SET(1707638834, 319443986, -1397762643, -396331579, 1853723780, 1407956892, -1139264192, -382120495, 1962926290, -1218382998, pgps_rtk);
	
}


void read_PARAM_REQUEST_LIST(pparam_request_list_PARAM_REQUEST_LIST *const pparam_request_list) {
	int8_t target_system    = pparam_request_list_target_system_GET(pparam_request_list);
	int8_t target_component = pparam_request_list_target_component_GET(pparam_request_list);
	
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


void write_UAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *const puavionix_adsb_out_cfg) {
	puavionix_adsb_out_cfg_stallSpeed_SET(-26505, -15338, 1831, 1354, -22249, -20184, -8437, 27018, 10614, 18646, puavionix_adsb_out_cfg);
	puavionix_adsb_out_cfg_ICAO_SET(-13351129, 1190700082, -1619239483, -1960282062, -1053050312, -1636140314, -1287696388, 1699481944, -929187062, -524442303, puavionix_adsb_out_cfg);
	puavionix_adsb_out_cfg_callsign_SET(some_string, strlen(some_string), puavionix_adsb_out_cfg);
	puavionix_adsb_out_cfg_emitterType_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHTER_AIR, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SPACE, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SMALL, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SMALL, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_PARACHUTE, puavionix_adsb_out_cfg);
	puavionix_adsb_out_cfg_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA, e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M, puavionix_adsb_out_cfg);
	puavionix_adsb_out_cfg_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M, puavionix_adsb_out_cfg);
	puavionix_adsb_out_cfg_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR, e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR, puavionix_adsb_out_cfg);
	puavionix_adsb_out_cfg_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY, e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, puavionix_adsb_out_cfg);
	
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
	planding_target_time_usec_SET(3897195475546492558L, 8278414363643778329L, -2114722087616072954L, -4053338840587692446L, -2690914214556579260L, -9086977777130855444L, -413392357966588645L, 7810951215840365261L, 6627502523241921167L, 5068012238875066723L, planding_target);
	planding_target_target_num_SET(109, 15, -60, -25, -116, -50, -112, 64, 42, -45, planding_target);
	planding_target_angle_x_SET(2.155407E38F, -2.3120442E38F, 2.2153882E38F, -2.379829E38F, 1.6098623E38F, -2.5197017E37F, 7.446821E37F, -8.732078E37F, -3.2244352E38F, 1.7937674E38F, planding_target);
	planding_target_angle_y_SET(8.4629926E36F, 5.199243E36F, -2.2862277E38F, 2.5013452E38F, -6.0798505E37F, -1.4278999E38F, -2.240934E38F, -2.229533E37F, 3.0416702E38F, 2.2999137E38F, planding_target);
	planding_target_distance_SET(-2.8798073E38F, 3.0231725E38F, -1.8785935E38F, 2.0545625E38F, -2.7471921E38F, 2.0618972E38F, 2.2704131E38F, 8.4607306E37F, 1.7389345E38F, 3.3211217E38F, planding_target);
	planding_target_size_x_SET(7.1679547E37F, -4.7260914E36F, 2.9250115E38F, 1.7843527E38F, 1.5088532E38F, -3.2190336E38F, -2.0721205E38F, -1.4803329E38F, -1.3240169E38F, 3.2914375E38F, planding_target);
	planding_target_size_y_SET(-1.0515295E38F, 9.60222E37F, 6.094058E37F, -8.006333E37F, 2.7262098E38F, -6.68097E37F, 6.944723E37F, -3.2041323E38F, -1.4096236E38F, 1.6800969E38F, planding_target);
	planding_target_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL, e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_MISSION, e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, e_MAV_FRAME_MAV_FRAME_GLOBAL, planding_target);
	planding_target_x_SET(1.4044199E38F, -2.0961098E38F, 1.0979361E38F, -3.0198717E38F, 2.9791288E38F, 1.4519214E38F, -1.4928889E38F, -5.651067E37F, 1.8466834E38F, -1.1446127E38F, planding_target);
	planding_target_y_SET(8.2337613E37F, -4.0422847E37F, -2.482267E38F, 2.2825935E38F, 1.380368E37F, 2.9324089E38F, 1.8800305E38F, 3.031794E38F, 9.886607E37F, -1.8643094E38F, planding_target);
	planding_target_z_SET(-1.9221849E37F, -2.1261492E38F, 7.4997384E37F, -2.8538638E38F, -7.7777357E37F, 3.1109215E38F, -2.5766775E38F, -3.3846015E38F, -1.3468937E38F, -1.2555026E38F, planding_target);
	for (size_t d0 = 0; d0 < Planding_target_q_D0; d0++) {
		planding_target_q_SET(-1.9188355E37F, 5.8644493E37F, 1.2511361E38F, 2.635881E38F, 2.809431E38F, 2.4462168E38F, 1.1791134E38F, -9.700839E37F, 2.1943876E38F, -1.8212502E38F, planding_target, d0);
	}
	planding_target_typE_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, planding_target);
	planding_target_position_valid_SET(-124, -70, -84, 109, -28, -46, -111, 11, 51, 29, planding_target);
	
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
	pset_actuator_control_target_time_usec_SET(3384634405407987064L, 4160574201355336957L, -3489166697043205227L, 4692364848981388289L, 4269629120988993065L, -2442889874956662996L, -1681923745358153784L, -2305786246483437436L, -2250675639663750882L, 4432069422058801654L, pset_actuator_control_target);
	pset_actuator_control_target_group_mlx_SET(113, -103, 28, 20, -7, -26, -123, 98, -98, 17, pset_actuator_control_target);
	pset_actuator_control_target_target_system_SET(13, 127, 122, -78, -93, 46, 42, 60, 31, -96, pset_actuator_control_target);
	pset_actuator_control_target_target_component_SET(88, -125, -18, 118, -60, -110, -31, 71, -7, -65, pset_actuator_control_target);
	pset_actuator_control_target_controls_SET(&-3.98331E37F, 4.9494936E37F, -3.7161345E37F, 1.485673E38F, 1.3593668E37F, 7.26016E37F, -1.9927737E38F, -3.224957E37F, 2.385647E38F, -2.966764E38F, pset_actuator_control_target);
	
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
	pcontrol_system_state_time_usec_SET(2164268840961337542L, 2937878306714433723L, 1788218673723511121L, 5709933052178023749L, 4399133851693986967L, 7595444734592286071L, -8904110095483220414L, -8771063325433413928L, 1715553820211799230L, -727778830310458265L, pcontrol_system_state);
	pcontrol_system_state_x_acc_SET(1.6399866E38F, -2.1975202E38F, 9.261071E37F, 3.3263856E38F, 1.1127629E38F, -1.3480359E38F, -1.5523877E38F, 7.7970176E37F, -1.2831328E38F, -1.7202363E38F, pcontrol_system_state);
	pcontrol_system_state_y_acc_SET(-1.9014224E38F, -2.0242534E38F, -2.9438989E38F, 1.3837131E38F, -1.3736433E37F, -1.7195033E38F, -2.1072036E38F, -1.7997736E38F, -2.549012E37F, 3.0279508E38F, pcontrol_system_state);
	pcontrol_system_state_z_acc_SET(-1.8838326E38F, -3.0239184E38F, -1.2765306E38F, 1.5585437E38F, -8.800815E37F, -7.3728653E37F, 3.3024746E38F, 1.8803822E38F, 1.4773923E38F, -2.99851E37F, pcontrol_system_state);
	pcontrol_system_state_x_vel_SET(-2.5839678E38F, -1.3103447E38F, 2.7580917E38F, 3.9268588E37F, -2.167608E38F, 1.82128E38F, 2.5600885E38F, 6.510375E37F, 6.2441654E37F, -1.496168E38F, pcontrol_system_state);
	pcontrol_system_state_y_vel_SET(-2.0671272E38F, 2.7268523E37F, -4.872723E37F, -1.1748549E38F, -1.6474297E38F, 2.518077E37F, -4.595453E37F, 3.9502992E37F, 2.9196271E38F, -2.6732114E38F, pcontrol_system_state);
	pcontrol_system_state_z_vel_SET(1.7633825E38F, 1.2907101E38F, -2.3227635E38F, 1.4011096E38F, -1.9150179E38F, -2.4014254E37F, -1.2624962E38F, -1.4737068E38F, 2.7057438E38F, -2.9908113E38F, pcontrol_system_state);
	pcontrol_system_state_x_pos_SET(-1.3575806E38F, -1.648119E38F, 1.7913893E38F, -2.4162698E38F, 5.4348613E37F, 1.091602E38F, 2.824888E38F, 2.0364653E38F, -2.4748343E38F, 1.282269E38F, pcontrol_system_state);
	pcontrol_system_state_y_pos_SET(-2.7098163E38F, 9.245933E37F, -1.6139791E38F, 2.3306768E38F, 2.1612327E38F, -1.0115172E38F, 3.3908986E38F, -3.2138218E38F, 2.474735E38F, 1.0262775E38F, pcontrol_system_state);
	pcontrol_system_state_z_pos_SET(2.298664E38F, -2.9189649E38F, -3.1284102E38F, 2.0099872E38F, -1.4072649E38F, -1.151831E37F, -2.5858573E38F, -7.721804E37F, -1.0775879E38F, 3.2305224E37F, pcontrol_system_state);
	pcontrol_system_state_airspeed_SET(2.1956035E37F, 1.7397133E38F, 1.2320593E38F, 1.3203475E38F, 2.0224412E38F, -9.284566E36F, 2.8501976E38F, 5.71012E37F, 1.3280284E38F, -3.1550552E38F, pcontrol_system_state);
	pcontrol_system_state_vel_variance_SET(&-5.8087163E37F, -1.2455087E38F, -3.397169E38F, -1.5833629E38F, -9.810444E37F, -5.079882E37F, -7.9152656E37F, 2.772558E38F, -2.0023971E38F, 2.5729632E38F, pcontrol_system_state);
	pcontrol_system_state_pos_variance_SET(&-3.1177542E38F, -3.1677177E38F, 2.8663745E37F, -1.3600813E38F, -1.3280034E37F, -3.0569556E38F, 2.2137792E38F, 9.059234E37F, 1.1921708E37F, 3.786051E37F, pcontrol_system_state);
	pcontrol_system_state_q_SET(&-1.0622578E38F, -3.0710055E37F, -7.822357E37F, 1.4180182E38F, -1.0584398E38F, 2.5878825E38F, -1.5461663E38F, -2.1618548E38F, 3.3956733E38F, -5.9959436E36F, pcontrol_system_state);
	pcontrol_system_state_roll_rate_SET(1.2174732E38F, -2.2180648E38F, -1.0167543E38F, 1.0079805E38F, 2.6553398E38F, -1.4245962E38F, 2.967686E38F, -3.0088135E38F, 1.666354E38F, 2.0993655E38F, pcontrol_system_state);
	pcontrol_system_state_pitch_rate_SET(2.9690107E38F, 2.337073E38F, 3.150748E38F, -2.2717518E38F, -1.1606458E37F, -3.1630523E38F, 2.025557E38F, -1.4003547E38F, 1.2917167E38F, 1.5111459E38F, pcontrol_system_state);
	pcontrol_system_state_yaw_rate_SET(1.2162667E38F, 1.4061161E38F, 2.891391E38F, -8.1468365E37F, 2.4522519E36F, -3.3687028E38F, 1.2125472E38F, -1.2903253E38F, 1.3740851E38F, 6.281569E37F, pcontrol_system_state);
	
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


void read_DATA32(pdata32_DATA32 *const pdata32) {
	int8_t       typE      = pdata32_typE_GET(pdata32);
	int8_t       len       = pdata32_len_GET(pdata32);
	Vdata32_daTa item_daTa = pdata32_daTa_GET(pdata32);
	for (size_t  index     = 0; index < item_daTa.len; index++)
		some_int8_t = vdata32_daTa_GET(&item_daTa, index);
	
}


void write_DATA32(pdata32_DATA32 *const pdata32) {
	pdata32_typE_SET(35, 7, 36, 33, 30, -109, 39, -43, -37, -55, pdata32);
	pdata32_len_SET(14, 4, 14, -63, 21, 6, -69, 5, 105, 39, pdata32);
	pdata32_daTa_SET(&-93, 82, 12, -12, 85, 16, -26, -80, 9, -31, pdata32);
	
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
				pping33_TTTT_SET(-811213159, 202847299, -1413540759, -2022240138, -2063970453, 1990912295, 91012966, 1974372266, -1846133229, -1830721956, pping33, d0, d1, d2);
			}
	pping33_field_SET(-8474095490765391393L, -9029733515979948486L, -3932210496692868464L, -3544380880404102948L, -3960534464012734617L, 3546837766156621823L, 3435766396312387231L, 3477159626816756739L, -650614739155979233L, -249463345096368270L, pping33);
	pping33_bit_field_SET(28, 43, 24, 39, 5, 7, 30, 16, 17, 18, pping33);
	pping33_field6_d0_d1_d2 {
				pping33_field6_SET(379460481, 1586505201, -1038030301, 690007455, 125164219, 25553, 664619644, -404323257, -1149998100, 53393066, pping33, d0, d1, d2);
			}
	pping33_testBOOL2_SET(false, true, false, false, false, false, false, true, true, false, pping33);
	pping33_testBOOL3_SET(false, false, true, true, true, true, false, true, true, true, pping33);
	pping33_testBOOL_SET(false, true, false, false, true, true, false, false, true, true, pping33);
	pping33_seq_SET(702463103L, 2683003879L, 2792861292L, 989144479L, 364460021L, 2785442803L, 389730079L, 1519665740L, 2601583788L, 1248285143L, pping33);
	size_t pping33_field1_d0 = 0;
	
	if (!pping33_field1(pping33, &pping33_field1_d0)) pping33_field1(pping33, &some_size_t);
	pping33_field1_d0_d1_d2 (pping33) {
						qping33_field1_SET(-442284004, 2135381987, 773056797, -135277145, -840350121, 21040295, 312277011, 1488384806, 380332120, -484872050, fld_field1, d0, d1, d2);
					}
	for (size_t d0           = 0; d0 < Pping33_field12_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field12_D1; d1++)\

					for (size_t d2 = 0; d2 < Pping33_field12_D2; d2++) {
						pping33_field12_SET(1225731598, 763822225, 206354076, -698482374, 1458646406, -325737448, 303970905, 883700407, 91481493, 2091727542, pping33, d0, d1, d2);
					}
	for (size_t                 d0 = 0; d0 < Pping33_field13_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field13_D1; d1++)\

					for (size_t d2 = 0; d2 < Pping33_field13_D2; d2++) {
						pping33_field13_SET(-1624703558, 708096607, -1261209300, 459779402, -273322354, 2145189477, 2094752362, -1405021214, -2014178229, -832438257, pping33, d0, d1, d2);
					}
	pping33_WWWWWWWW_SET(1984022140, 32740442, -966263901, 221305869, 1347068659, -1474425308, 1978112446, -1535053502, 1211851192, -1807815090, pping33);
	pping33_bit_field2_SET(30, 38, 27, 45, 10, 36, 7, 40, 32, 43, pping33);
	for (size_t d0 = 0; d0 < Pping33_Field_Bits_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_Field_Bits_D1; d1++)\

					for (size_t d2                          = 0; d2 < Pping33_Field_Bits_D2; d2++) {
						pping33_Field_Bits_SET(37, 28, 15, 9, 5, 42, 22, 29, 7, 16, pping33, d0, d1, d2);
					}
	size_t                      pping33_SparseFixAllBits_d0 = 0;
	
	if (!pping33_SparseFixAllBits(pping33, &pping33_SparseFixAllBits_d0)) pping33_SparseFixAllBits(pping33, &some_size_t);
	pping33_SparseFixAllBits_d0_d1_d2 (pping33) {
						qping33_SparseFixAllBits_SET(19, 22, 24, 7, 39, 11, 23, 25, 39, 6, fld_SparseFixAllBits, d0, d1, d2);
					}
	size_t pping33_FixAllBits_d0                            = 0;
	
	if (!pping33_FixAllBits(pping33, &pping33_FixAllBits_d0)) pping33_FixAllBits(pping33, &some_size_t);
	pping33_FixAllBits_d0_d1_d2 (pping33) {
						qping33_FixAllBits_SET(14, 28, 31, 40, 29, 36, 20, 32, 25, 15, fld_FixAllBits, d0, d1, d2);
					}
	size_t pping33_VarAllBits_d0                            = 0;
	size_t pping33_VarAllBits_d2                            = 0;
	
	if (!pping33_VarAllBits(pping33, &pping33_VarAllBits_d0, &pping33_VarAllBits_d2)) pping33_VarAllBits(pping33, &some_size_t, &some_size_t);
	pping33_VarAllBits_d0_d1_d2 (pping33) {
						qping33_VarAllBits_SET(18, 32, 15, 42, 33, 45, 35, 44, 43, 15, fld_VarAllBits, d0, d1, d2);
					}
	size_t pping33_SparseVarAllBits_d0                      = 0;
	size_t pping33_SparseVarAllBits_d2                      = 0;
	
	if (!pping33_SparseVarAllBits(pping33, &pping33_SparseVarAllBits_d0, &pping33_SparseVarAllBits_d2)) pping33_SparseVarAllBits(pping33, &some_size_t, &some_size_t);
	pping33_SparseVarAllBits_d0_d1_d2 (pping33) {
						qping33_SparseVarAllBits_SET(38, 18, 42, 32, 26, 34, 26, 24, 18, 25, fld_SparseVarAllBits, d0, d1, d2);
					}
	size_t pping33_VarEachBits_d0                           = 0;
	
	if (!pping33_VarEachBits(pping33, &pping33_VarEachBits_d0)) pping33_VarEachBits(pping33, &some_size_t);
	pping33_VarEachBits_d0_d1_d2 (pping33) {
						qping33_VarEachBits_SET(-3, -12, 34, 12, -13, -12, 21, 45, 25, 19, fld_VarEachBits, d0, d1, d2);
					}
	size_t pping33_SparsVarEachBits_d0                      = 0;
	
	if (!pping33_SparsVarEachBits(pping33, &pping33_SparsVarEachBits_d0)) pping33_SparsVarEachBits(pping33, &some_size_t);
	pping33_SparsVarEachBits_d0_d1_d2 (pping33) {
						qping33_SparsVarEachBits_SET(199, 40, 97, 429, 228, 293, 367, 426, 378, 327, fld_SparsVarEachBits, d0, d1, d2);
					}
	pping33_testBOOLX_SET(false, true, false, false, false, true, false, true, false, true, pping33);
	pping33_testBOOL2X_SET(false, true, false, false, false, true, true, false, false, true, pping33);
	pping33_testBOOL3X_SET(false, false, true, true, true, false, false, false, true, true, pping33);
	pping33_MMMMMM_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, e_MAV_MODE_GUIDED_DISARMED, e_MAV_MODE_MAV_MODE_AUTO_ARMED, e_MAV_MODE_STABILIZE_ARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, e_MAV_MODE_MANUAL_ARMED, e_MAV_MODE_STABILIZE_ARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, e_MAV_MODE_STABILIZE_DISARMED, e_MAV_MODE_MAV_MODE_AUTO_DISARMED, pping33);
	size_t pping33_field44_d2     = 0;
	
	if (!pping33_field44(pping33, &pping33_field44_d2)) pping33_field44(pping33, &some_size_t);
	pping33_field44_d0_d1_d2 (pping33) {
						qping33_field44_SET(1912891226, -2047631311, 1280240464, 1871855388, 1471102500, 921554700, 238391648, -53863285, 2098131528, 1491614626, fld_field44, d0, d1, d2);
					}
	size_t pping33_field634_d2    = 0;
	
	if (!pping33_field634(pping33, &pping33_field634_d2)) pping33_field634(pping33, &some_size_t);
	pping33_field634_d0_d1_d2 (pping33) {
						qping33_field634_SET(110007668, 170529956, 1644936550, 1895473540, 1292194693, 1748169106, -1087943895, 1747471117, -767805915, -1708518854, fld_field634, d0, d1, d2);
					}
	size_t pping33_field33344_d2  = 0;
	
	if (!pping33_field33344(pping33, &pping33_field33344_d2)) pping33_field33344(pping33, &some_size_t);
	pping33_field33344_d0_d1_d2 (pping33) {
						qping33_field33344_SET(1057043166, -376261520, 716906558, 157729302, 769359071, -357204174, -34706727, 2012716744, -1474980138, -2008049798, fld_field33344, d0, d1, d2);
					}
	size_t pping33_field333634_d2 = 0;
	
	if (!pping33_field333634(pping33, &pping33_field333634_d2)) pping33_field333634(pping33, &some_size_t);
	pping33_field333634_d0_d1_d2 (pping33) {
						qping33_field333634_SET(-1060737043, 1353188649, 506688198, 640821405, 1330452851, 180331688, 1734439010, 1550289018, -1614731434, 130511237, fld_field333634, d0, d1, d2);
					}
	for (size_t d0                = 0; d0 < Pping33_field___D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field___D1; d1++)\

					for (size_t d2                 = 0; d2 < Pping33_field___D2; d2++) {
						pping33_field___SET(1735107656, -275198315, 1424743874, 399695761, -1939819800, 1091489367, -381222535, 113312058, 248191625, 1747138724, pping33, d0, d1, d2);
					}
	size_t                      pping33_field63_d2 = 0;
	
	if (!pping33_field63(pping33, &pping33_field63_d2)) pping33_field63(pping33, &some_size_t);
	pping33_field63_d0_d1_d2 (pping33) {
						qping33_field63_SET(1493595333, 1727456218, -346238115, 1649226477, -197935369, -254717855, 1303960626, -1900028645, 1756776689, 602324804, fld_field63, d0, d1, d2);
					}
	for (size_t d0                                 = 0; d0 < Pping33_uid2_D0; d0++) {
		pping33_uid2_SET(54, -96, -112, 114, 84, -29, -23, 116, -100, 14, pping33, d0);
	}
	for (size_t d0                                 = 0; d0 < Pping33_field2_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field2_D1; d1++)\

					for (size_t d2 = 0; d2 < Pping33_field2_D2; d2++) {
						pping33_field2_SET(1591873830, 118626491, 1070752665, 550644230, 996805424, 1715387733, -703400665, -2047235856, -1925455937, 1080075851, pping33, d0, d1, d2);
					}
	for (size_t                 d0 = 0; d0 < Pping33_field4_D0; d0++)\

			for (size_t d1 = 0; d1 < Pping33_field4_D1; d1++)\

					for (size_t d2 = 0; d2 < Pping33_field4_D2; d2++) {
						pping33_field4_SET(1537792020, -779774520, -2033221803, 848628591, -184956100, 162921396, 1966646817, 1938599305, 1582149897, -877845507, pping33, d0, d1, d2);
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


void write_RALLY_POINT(prally_point_RALLY_POINT *const prally_point) {
	prally_point_land_dir_SET(-1625, 1742, -28094, -16982, 32197, 31261, 13582, -12545, 22443, 32374, prally_point);
	prally_point_target_system_SET(-57, 117, 7, 117, -45, 97, 25, -89, -29, 79, prally_point);
	prally_point_target_component_SET(82, 2, -106, -28, 45, -59, -65, -98, -96, -38, prally_point);
	prally_point_idx_SET(-127, 21, -55, 3, -20, -90, 125, 82, 115, -118, prally_point);
	prally_point_count_SET(-67, 53, 81, -121, -105, -73, 19, -113, -41, 103, prally_point);
	prally_point_lat_SET(-1179835628, -947194279, -1908123515, 288564210, -1129973273, -1101736758, -1986986709, -308528348, 1828230011, 1440473764, prally_point);
	prally_point_lng_SET(-1220629656, 1394842425, -381675470, 1170153209, -1724732689, -556242728, -1797391465, -1594338863, 818252650, -1189986296, prally_point);
	prally_point_alt_SET(2025, -23691, 7909, 11309, -19972, 28754, -19318, -5619, 7317, 2728, prally_point);
	prally_point_break_alt_SET(-20245, 10483, 32595, 8411, -25527, -5492, -17934, 7177, 13242, -16008, prally_point);
	prally_point_flags_SET(e_RALLY_FLAGS_LAND_IMMEDIATELY, e_RALLY_FLAGS_FAVORABLE_WIND, e_RALLY_FLAGS_FAVORABLE_WIND, e_RALLY_FLAGS_FAVORABLE_WIND, e_RALLY_FLAGS_FAVORABLE_WIND, e_RALLY_FLAGS_LAND_IMMEDIATELY, e_RALLY_FLAGS_FAVORABLE_WIND, e_RALLY_FLAGS_LAND_IMMEDIATELY, e_RALLY_FLAGS_FAVORABLE_WIND, e_RALLY_FLAGS_LAND_IMMEDIATELY, prally_point);
	
}


void read_MISSION_SET_CURRENT(pmission_set_current_MISSION_SET_CURRENT *const pmission_set_current) {
	int16_t seq              = pmission_set_current_seq_GET(pmission_set_current);
	int8_t  target_system    = pmission_set_current_target_system_GET(pmission_set_current);
	int8_t  target_component = pmission_set_current_target_component_GET(pmission_set_current);
	
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


void write_ADAP_TUNING(padap_tuning_ADAP_TUNING *const padap_tuning) {
	padap_tuning_desired_SET(-2.4241599E38F, -1.9926725E38F, -3.1454904E38F, -1.3339573E38F, -2.0589617E38F, -2.645564E38F, -2.3633544E38F, -1.7516585E38F, -2.8137961E38F, 2.1247737E38F, padap_tuning);
	padap_tuning_achieved_SET(2.3398633E38F, 2.4149273E38F, -4.8557524E37F, 3.0171491E37F, -3.8390015E37F, -1.5366619E37F, 5.3569484E37F, 1.6252992E38F, -1.2469599E38F, -2.3719006E38F, padap_tuning);
	padap_tuning_error_SET(3.0181167E38F, 3.2643045E38F, -9.056367E36F, -1.9964757E38F, 7.9218026E37F, -2.6625135E38F, 2.1642086E38F, 1.6981185E38F, 1.7837996E38F, -2.9775184E38F, padap_tuning);
	padap_tuning_theta_SET(2.5757891E38F, -1.9381234E38F, 1.9852447E38F, 2.0793188E38F, 2.9745551E38F, -1.5703406E37F, -3.1867426E38F, 8.036136E37F, 2.6352643E38F, 2.7516236E38F, padap_tuning);
	padap_tuning_omega_SET(-2.210976E38F, 2.3976382E38F, -1.2531583E38F, -3.0970177E38F, -2.1764486E38F, 3.337023E38F, -2.8610521E38F, 7.3633346E37F, 2.4491726E38F, -3.0112148E38F, padap_tuning);
	padap_tuning_sigma_SET(2.6361646E38F, -8.183983E37F, -1.2154188E38F, -1.4179747E38F, -1.0666494E38F, -8.1060744E37F, 1.7030793E38F, 2.7883782E38F, -3.0530052E38F, -2.0165135E38F, padap_tuning);
	padap_tuning_theta_dot_SET(-5.870236E37F, -2.637998E38F, -1.5289324E38F, 1.0254515E38F, 3.2969714E38F, -1.3544611E38F, -2.7849044E38F, -3.2565745E38F, 4.8579687E37F, -2.828662E37F, padap_tuning);
	padap_tuning_omega_dot_SET(1.7327607E38F, 1.6029076E38F, 1.3487935E38F, 2.0381931E38F, -2.8097425E38F, 1.9208286E38F, -1.7352472E38F, 2.833752E38F, 1.3955886E38F, 2.3433438E38F, padap_tuning);
	padap_tuning_sigma_dot_SET(-7.676482E37F, -9.565875E37F, -3.0079037E38F, 6.2130466E37F, 2.4528504E37F, -1.9361974E38F, -2.0995789E38F, 3.2011198E38F, 1.0707721E38F, 1.0881744E38F, padap_tuning);
	padap_tuning_f_SET(-1.3125666E38F, -6.9526543E37F, -3.0521668E38F, 2.1485457E38F, 3.159398E38F, 2.7358622E38F, 7.9786593E37F, 1.7471059E38F, 2.176994E38F, 3.8652492E37F, padap_tuning);
	padap_tuning_f_dot_SET(-2.7268132E38F, 2.7382312E38F, 2.268555E38F, -1.3333164E38F, -1.6975568E38F, 1.2206888E38F, -1.4933514E37F, -1.9291393E38F, 2.7771854E38F, -1.110501E37F, padap_tuning);
	padap_tuning_u_SET(6.807309E37F, -7.0727997E37F, 1.0005789E38F, -2.058182E38F, -3.2240474E38F, -1.4646622E38F, 2.2288003E38F, -1.949088E38F, 2.2670864E38F, -1.9167528E38F, padap_tuning);
	padap_tuning_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_STEER, e_PID_TUNING_AXIS_PID_TUNING_LANDING, e_PID_TUNING_AXIS_PID_TUNING_ACCZ, e_PID_TUNING_AXIS_PID_TUNING_ROLL, e_PID_TUNING_AXIS_PID_TUNING_LANDING, e_PID_TUNING_AXIS_PID_TUNING_YAW, e_PID_TUNING_AXIS_PID_TUNING_ACCZ, e_PID_TUNING_AXIS_PID_TUNING_STEER, e_PID_TUNING_AXIS_PID_TUNING_LANDING, e_PID_TUNING_AXIS_PID_TUNING_ROLL, padap_tuning);
	
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
	pvibration_clipping_0_SET(-1844762863, -1071390480, -1089734149, 1161689389, -1812772406, 1182027133, 1289703605, -549637770, -156522046, 1352124796, pvibration);
	pvibration_clipping_1_SET(-1368839584, 950113028, -381437691, -628234071, -751245257, -1480685477, 414661540, 1817375337, 1034412582, 1460206412, pvibration);
	pvibration_clipping_2_SET(-913634150, -767726182, 473674029, 891302056, 342689932, -226607549, -1184722067, -813254084, -761453058, -364089339, pvibration);
	pvibration_time_usec_SET(-946366889336347614L, 8187090830747217664L, -2607439769915673923L, -464615073941886755L, 6932913157003325121L, 6173032345635859964L, 6284380364327299650L, 6666019843512522331L, 2627350552612800103L, -6733514109500232036L, pvibration);
	pvibration_vibration_x_SET(-2.5932377E38F, 9.515597E37F, -2.2153099E38F, -5.0153384E37F, 3.0981437E38F, -1.4530532E38F, -1.5565443E38F, 1.6523211E38F, -2.8744841E38F, -6.5575794E37F, pvibration);
	pvibration_vibration_y_SET(-1.2803804E38F, 2.7996378E38F, 2.216932E38F, 1.4267799E38F, 3.6568688E37F, 2.6956132E38F, 2.0886326E38F, -3.0401742E38F, -1.0082189E37F, 7.0652207E37F, pvibration);
	pvibration_vibration_z_SET(9.320213E37F, -1.3604991E38F, 6.3920084E36F, 1.8610857E38F, 3.1598272E38F, -2.2197077E38F, -2.4279484E38F, 2.2071817E38F, 3.27137E38F, -1.8570347E38F, pvibration);
	
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


void write_PARAM_EXT_VALUE(pparam_ext_value_PARAM_EXT_VALUE *const pparam_ext_value) {
	pparam_ext_value_param_count_SET(-31103, -24908, 28161, -19340, 32501, 28105, 12461, 16971, -11648, 29820, pparam_ext_value);
	pparam_ext_value_param_index_SET(23221, -24408, 19249, -28997, 31109, 20094, 17838, 20000, 2830, 8921, pparam_ext_value);
	pparam_ext_value_param_id_SET(some_string, strlen(some_string), pparam_ext_value);
	pparam_ext_value_param_value_SET(some_string, strlen(some_string), pparam_ext_value);
	pparam_ext_value_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, pparam_ext_value);
	
}


void read_BATTERY2(pbattery2_BATTERY2 *const pbattery2) {
	int16_t voltage         = pbattery2_voltage_GET(pbattery2);
	int16_t current_battery = pbattery2_current_battery_GET(pbattery2);
	
}


void write_BATTERY2(pbattery2_BATTERY2 *const pbattery2) {
	pbattery2_voltage_SET(17426, -7573, 7160, 23157, -11912, 8965, -9371, 22450, 16955, 2897, pbattery2);
	pbattery2_current_battery_SET(-14231, -14182, 17200, -1561, -22261, 25946, -18749, 30344, 30346, 29894, pbattery2);
	
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


void write_LIMITS_STATUS(plimits_status_LIMITS_STATUS *const plimits_status) {
	plimits_status_breach_count_SET(25127, -8914, -19476, 15563, 30162, -14339, 19114, -29249, 4799, -6388, plimits_status);
	plimits_status_last_trigger_SET(-422028910, -269442665, -1130723371, 1364286179, -832127934, -1712433186, 29755023, -1090455927, 1265617968, -623019427, plimits_status);
	plimits_status_last_action_SET(1665386313, -249301007, 999079414, -340624177, -1740322156, -597969477, 1652524482, 284965032, 27748001, 382026238, plimits_status);
	plimits_status_last_recovery_SET(769011813, -1843073538, 1073504488, -113777133, 325852906, -386983881, 2021954021, 916846519, -1044885007, 1844228471, plimits_status);
	plimits_status_last_clear_SET(2041361814, 6450065, -658799732, 175895504, 1015566498, 1908156877, 418756220, 918742846, 63998631, -436354430, plimits_status);
	plimits_status_limits_state_SET(e_LIMITS_STATE_LIMITS_ENABLED, e_LIMITS_STATE_LIMITS_RECOVERING, e_LIMITS_STATE_LIMITS_TRIGGERED, e_LIMITS_STATE_LIMITS_RECOVERED, e_LIMITS_STATE_LIMITS_ENABLED, e_LIMITS_STATE_LIMITS_RECOVERED, e_LIMITS_STATE_LIMITS_RECOVERING, e_LIMITS_STATE_LIMITS_DISABLED, e_LIMITS_STATE_LIMITS_RECOVERED, e_LIMITS_STATE_LIMITS_RECOVERED, plimits_status);
	plimits_status_mods_enabled_SET(e_LIMIT_MODULE_LIMIT_GPSLOCK, e_LIMIT_MODULE_LIMIT_ALTITUDE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GPSLOCK, e_LIMIT_MODULE_LIMIT_GPSLOCK, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, plimits_status);
	plimits_status_mods_required_SET(e_LIMIT_MODULE_LIMIT_ALTITUDE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_ALTITUDE, e_LIMIT_MODULE_LIMIT_ALTITUDE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_ALTITUDE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GPSLOCK, e_LIMIT_MODULE_LIMIT_GPSLOCK, plimits_status);
	plimits_status_mods_triggered_SET(e_LIMIT_MODULE_LIMIT_ALTITUDE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_ALTITUDE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GEOFENCE, e_LIMIT_MODULE_LIMIT_GPSLOCK, e_LIMIT_MODULE_LIMIT_GEOFENCE, plimits_status);
	
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


void write_CAMERA_FEEDBACK(pcamera_feedback_CAMERA_FEEDBACK *const pcamera_feedback) {
	pcamera_feedback_img_idx_SET(-24948, 11533, 23007, -18259, 10767, -16521, -947, -145, 1469, -28171, pcamera_feedback);
	pcamera_feedback_time_usec_SET(8195700062552158938L, 8850333660976185529L, 5190978613475656242L, 7094204019190621662L, 6922161642950280389L, -781176810021860157L, 1239590533468483593L, -1402651914043347032L, 1272190955787067713L, -725890338656563369L, pcamera_feedback);
	pcamera_feedback_target_system_SET(98, 115, 30, -126, 66, 66, 24, -126, -114, 101, pcamera_feedback);
	pcamera_feedback_cam_idx_SET(-103, -47, 121, -119, -128, 103, -94, -77, 49, -35, pcamera_feedback);
	pcamera_feedback_lat_SET(-35998248, -1837667633, -700260380, 768081492, -582945239, -1260432333, -2051773252, 211116741, 1504189852, 1086647484, pcamera_feedback);
	pcamera_feedback_lng_SET(-143289101, -1994360688, -560964871, -642279492, -1681177496, 687993613, -1445318030, 1138476467, 1622556240, -1479433041, pcamera_feedback);
	pcamera_feedback_alt_msl_SET(-7.6026513E37F, -1.4429046E38F, 1.7013815E38F, 2.0506818E38F, 3.0832096E38F, -2.3540521E38F, 3.1559007E38F, 2.4494506E38F, -2.157521E38F, 8.1417355E37F, pcamera_feedback);
	pcamera_feedback_alt_rel_SET(2.5966097E38F, 1.9568019E38F, 2.1962178E37F, -2.6517352E38F, 2.2198698E38F, 2.5241793E38F, 1.2512487E38F, -2.471505E38F, 3.762306E37F, 7.287777E37F, pcamera_feedback);
	pcamera_feedback_roll_SET(-6.073805E36F, 3.0137338E37F, 2.619078E38F, -2.2906507E38F, -3.0809197E38F, -1.0857545E38F, -2.4846505E38F, 2.1676316E37F, -1.7393783E38F, -2.3655076E38F, pcamera_feedback);
	pcamera_feedback_pitch_SET(-9.988048E36F, 1.163682E38F, -3.1565139E38F, 2.7684019E37F, -2.1055796E38F, 2.7340289E38F, -1.192868E38F, -2.2762358E38F, -1.5546353E38F, -2.3388983E38F, pcamera_feedback);
	pcamera_feedback_yaw_SET(-3.0193127E38F, -1.7910745E38F, -1.0462141E38F, 2.7503594E38F, -8.0293653E37F, -6.294788E37F, 6.3251024E37F, -1.8124418E38F, -3.335939E38F, 2.734071E38F, pcamera_feedback);
	pcamera_feedback_foc_len_SET(3.9520966E36F, -7.369643E37F, 3.1815178E38F, -4.720938E37F, -1.8595708E38F, 7.633417E37F, 3.0348401E38F, 1.913217E38F, -2.1663765E38F, 2.8104357E38F, pcamera_feedback);
	pcamera_feedback_flags_SET(e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_PHOTO, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_PHOTO, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_BADEXPOSURE, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_VIDEO, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_OPENLOOP, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_OPENLOOP, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_CLOSEDLOOP, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_VIDEO, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_OPENLOOP, e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_VIDEO, pcamera_feedback);
	
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
	phil_gps_eph_SET(4934, -29777, -9969, 3718, 29934, -30682, 12947, -2074, -5187, -27778, phil_gps);
	phil_gps_epv_SET(-26389, -26075, -31059, 21276, -1959, 31774, -7488, -13593, 26450, 15180, phil_gps);
	phil_gps_vel_SET(-27230, -9390, -2750, 2592, -16683, -27346, -6437, -238, -13046, 19563, phil_gps);
	phil_gps_cog_SET(17118, 29085, -6328, -10964, 29484, -21321, 13641, -4644, 22480, -25337, phil_gps);
	phil_gps_time_usec_SET(-7258314369815243188L, -7157859521591535313L, 9077758797098107564L, 267808561322965681L, -15571771223542852L, -8652315268064560970L, 9024265742080489245L, 6708546132574746038L, -1120596787922883807L, 7127710049602402836L, phil_gps);
	phil_gps_fix_type_SET(59, 17, 81, 66, -73, -36, 56, -124, -39, -48, phil_gps);
	phil_gps_lat_SET(403785568, -452297174, -1274705742, -146943573, 905307038, 1637922525, -1785716296, -588854267, 1728332539, 830711752, phil_gps);
	phil_gps_lon_SET(2001786752, 319695271, -1783117300, 12247862, 2130583836, -368013908, 1182594568, 1670999445, 1543628757, -1696494180, phil_gps);
	phil_gps_alt_SET(634365175, 2088422059, -748300437, -1788905529, -1694451036, 391465342, -929925543, 1039159482, 1587371240, -1254921681, phil_gps);
	phil_gps_vn_SET(27873, -1500, -22475, -25184, 6995, -24357, 1400, 32269, 10231, 14400, phil_gps);
	phil_gps_ve_SET(17399, -16318, 30859, 17783, -18804, -3341, 12715, -13419, -28144, -19430, phil_gps);
	phil_gps_vd_SET(-20422, -16260, 31099, 27224, 7537, 6922, -22390, -19985, 24245, -5859, phil_gps);
	phil_gps_satellites_visible_SET(-92, -63, -119, 67, 66, 114, 20, 27, 28, 0, phil_gps);
	
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


void read_AUTH_KEY(pauth_key_AUTH_KEY *const pauth_key) {
	Vauth_key_key item_key;
	if (pauth_key_key_GET(pauth_key, &item_key)) {
		memcpy(some_string, item_key.bytes, item_key.len);
	}
	
}


void read_FENCE_FETCH_POINT(pfence_fetch_point_FENCE_FETCH_POINT *const pfence_fetch_point) {
	int8_t target_system    = pfence_fetch_point_target_system_GET(pfence_fetch_point);
	int8_t target_component = pfence_fetch_point_target_component_GET(pfence_fetch_point);
	int8_t idx              = pfence_fetch_point_idx_GET(pfence_fetch_point);
	
}


void write_FENCE_FETCH_POINT(pfence_fetch_point_FENCE_FETCH_POINT *const pfence_fetch_point) {
	pfence_fetch_point_target_system_SET(51, -109, 1, -21, 20, 97, -126, -113, -51, -57, pfence_fetch_point);
	pfence_fetch_point_target_component_SET(20, -44, 95, -108, 122, -66, 24, -38, -53, -127, pfence_fetch_point);
	pfence_fetch_point_idx_SET(-45, -57, -77, 10, 60, 125, -69, 39, -15, 125, pfence_fetch_point);
	
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


void write_RADIO(pradio_RADIO *const pradio) {
	pradio_rxerrors_SET(-28792, 681, 24560, -15761, 1493, 20413, -24321, -13108, -6993, -14031, pradio);
	pradio_fixeD_SET(18805, 20763, -11613, 6071, -18687, -7851, 28306, 10188, -24398, 13923, pradio);
	pradio_rssi_SET(-121, -28, -71, 95, 25, -58, -102, -89, 31, -14, pradio);
	pradio_remrssi_SET(-39, 70, -86, -30, 34, 110, 110, 77, 5, -23, pradio);
	pradio_txbuf_SET(64, 16, 53, 122, -113, -16, 125, -23, -7, -40, pradio);
	pradio_noise_SET(-40, -74, 65, 126, 74, -13, -109, 38, 46, -96, pradio);
	pradio_remnoise_SET(78, -88, 25, -8, 4, 30, -54, -11, -40, -43, pradio);
	
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


void write_AIRSPEED_AUTOCAL(pairspeed_autocal_AIRSPEED_AUTOCAL *const pairspeed_autocal) {
	pairspeed_autocal_vx_SET(-2.0753434E38F, 1.9054717E38F, 7.806375E37F, 3.2031253E38F, 1.2876375E38F, -2.3197302E38F, -1.6670108E38F, 8.821619E37F, -8.635743E37F, 2.9047053E38F, pairspeed_autocal);
	pairspeed_autocal_vy_SET(-3.0442627E38F, 1.4358384E37F, -1.8280905E38F, 5.634795E37F, 6.4444253E37F, 1.9496209E38F, 3.1692399E38F, -3.0017206E38F, 1.6156271E38F, 2.2818512E38F, pairspeed_autocal);
	pairspeed_autocal_vz_SET(3.0283501E38F, 2.0669593E38F, -4.8487453E37F, 2.5507787E38F, -1.9925622E38F, 1.4690417E38F, 1.3065497E38F, 2.2903272E38F, -2.990727E38F, 1.0942925E37F, pairspeed_autocal);
	pairspeed_autocal_diff_pressure_SET(-2.8874586E38F, -1.2361713E38F, 2.7636899E38F, 6.4052367E37F, 2.2092945E38F, -2.8756388E38F, -3.0776021E38F, 2.3739043E38F, 5.070011E36F, 1.2794699E37F, pairspeed_autocal);
	pairspeed_autocal_EAS2TAS_SET(-5.6730133E37F, -2.3573675E38F, 6.3474703E37F, -2.2604894E38F, 8.891667E37F, 2.1512952E38F, -1.4311241E38F, -2.880014E38F, -1.3323369E38F, 1.3905057E38F, pairspeed_autocal);
	pairspeed_autocal_ratio_SET(3.1639802E38F, -2.2918068E37F, -2.84286E38F, -1.7904177E38F, -2.1364805E38F, 2.4717724E38F, -1.5652544E38F, -2.3847205E38F, 2.6138116E38F, -2.9840267E37F, pairspeed_autocal);
	pairspeed_autocal_state_x_SET(1.3376501E38F, 1.7269502E38F, -1.032978E38F, 2.0428737E38F, -3.2139673E38F, 1.4600113E36F, -2.8957715E38F, 2.7025585E38F, 2.186197E38F, -2.079578E38F, pairspeed_autocal);
	pairspeed_autocal_state_y_SET(-5.001034E37F, 2.6449035E38F, -2.7993638E38F, 2.7042514E38F, 4.1814477E37F, 2.1867892E38F, -2.3223833E37F, -1.1967628E38F, 9.555489E37F, -1.7022453E37F, pairspeed_autocal);
	pairspeed_autocal_state_z_SET(1.8317659E38F, 2.4048458E38F, -1.3956209E38F, -3.224878E38F, 8.750075E37F, -1.0339295E38F, 1.174588E38F, -4.8166793E37F, 2.316936E38F, 3.9683855E37F, pairspeed_autocal);
	pairspeed_autocal_Pax_SET(-1.5917253E38F, -6.1831326E37F, 3.4014708E38F, 8.816754E37F, 1.4268757E38F, 2.0714774E38F, 2.9185759E38F, 3.1766937E38F, 2.0616607E38F, 4.816354E37F, pairspeed_autocal);
	pairspeed_autocal_Pby_SET(1.6601771E38F, 2.9314258E38F, 3.1244265E38F, -2.6221396E37F, -2.5342605E37F, 2.539932E38F, -9.87391E37F, -4.1929658E37F, 1.2038815E38F, 3.0306623E38F, pairspeed_autocal);
	pairspeed_autocal_Pcz_SET(-2.2826755E38F, 7.7817477E36F, -2.563957E38F, -9.200959E37F, 2.020831E38F, 5.8366026E37F, -5.017888E37F, -1.1472991E38F, -5.01602E37F, -1.8178156E38F, pairspeed_autocal);
	
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
	patt_pos_mocap_time_usec_SET(6206521554054582877L, -153967487250471260L, 5958503557455176641L, -5976758702261947766L, -1916583663859003864L, -4587328224322919330L, 6290360660477468077L, 3889285594932453670L, 8331469240163565971L, -1845536061176849548L, patt_pos_mocap);
	patt_pos_mocap_q_SET(&-3.106748E38F, -1.2332437E38F, 2.5232431E38F, 2.8682768E37F, 2.5473615E38F, 2.3711293E38F, -2.0831984E38F, 3.1116616E38F, -1.5897764E38F, -3.2162928E38F, patt_pos_mocap);
	patt_pos_mocap_x_SET(3.0449933E38F, 1.602928E38F, -1.6928922E38F, -2.9295332E37F, 1.8685588E38F, 1.7679337E38F, 2.910114E38F, -3.3257734E38F, 3.3814612E38F, 1.5153114E38F, patt_pos_mocap);
	patt_pos_mocap_y_SET(2.835055E38F, 2.2135895E38F, 2.1024357E37F, -1.9476202E38F, 2.5238226E38F, -6.7287974E37F, 1.5181453E38F, -6.425851E37F, 7.9104485E37F, 4.0611015E37F, patt_pos_mocap);
	patt_pos_mocap_z_SET(-2.271289E38F, -7.575938E36F, 1.2343998E38F, -2.4674565E38F, 1.0012103E38F, 8.651014E37F, 5.182551E37F, -1.760192E38F, 1.9493144E37F, 1.4988498E38F, patt_pos_mocap);
	
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
	pstatustext_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_NOTICE, e_MAV_SEVERITY_MAV_SEVERITY_ALERT, e_MAV_SEVERITY_MAV_SEVERITY_INFO, e_MAV_SEVERITY_MAV_SEVERITY_ERROR, e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, e_MAV_SEVERITY_MAV_SEVERITY_INFO, e_MAV_SEVERITY_MAV_SEVERITY_INFO, e_MAV_SEVERITY_MAV_SEVERITY_WARNING, e_MAV_SEVERITY_MAV_SEVERITY_NOTICE, e_MAV_SEVERITY_MAV_SEVERITY_ALERT, pstatustext);
	pstatustext_text_SET(some_string, strlen(some_string), pstatustext);
	
}


void read_PING(pping_PING *const pping) {
	int32_t seq              = pping_seq_GET(pping);
	int64_t time_usec        = pping_time_usec_GET(pping);
	int8_t  target_system    = pping_target_system_GET(pping);
	int8_t  target_component = pping_target_component_GET(pping);
	
}


void read_GOPRO_GET_REQUEST(pgopro_get_request_GOPRO_GET_REQUEST *const pgopro_get_request) {
	int8_t          target_system    = pgopro_get_request_target_system_GET(pgopro_get_request);
	int8_t          target_component = pgopro_get_request_target_component_GET(pgopro_get_request);
	e_GOPRO_COMMAND item_cmd_id;
	if (pgopro_get_request_cmd_id_GET(pgopro_get_request, &item_cmd_id)) {
		e_GOPRO_COMMAND_GOPRO_COMMAND_POWER = item_cmd_id;
	}
	
}


void write_GOPRO_GET_REQUEST(pgopro_get_request_GOPRO_GET_REQUEST *const pgopro_get_request) {
	pgopro_get_request_target_system_SET(-32, 70, -48, -11, 104, 12, -9, 84, -13, 99, pgopro_get_request);
	pgopro_get_request_target_component_SET(-60, 127, 39, -32, -128, 102, -60, -17, 118, 37, pgopro_get_request);
	pgopro_get_request_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_VIDEO_SETTINGS, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_COLOUR, e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_BURST_RATE, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_GAIN, e_GOPRO_COMMAND_GOPRO_COMMAND_POWER, e_GOPRO_COMMAND_GOPRO_COMMAND_VIDEO_SETTINGS, e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_COLOUR, e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_COLOUR, pgopro_get_request);
	
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
	pcamera_capture_status_time_boot_ms_SET(-746981698, 417202272, -1430666638, 262318015, -1360829443, -1999899769, -390448034, 249122503, 568889970, -1317473964, pcamera_capture_status);
	pcamera_capture_status_recording_time_ms_SET(-2111046093, -1795551522, 1617134341, 146847834, -541915154, 1641259203, -265392038, -1272090451, 1005438829, -2093961236, pcamera_capture_status);
	pcamera_capture_status_image_status_SET(-69, 101, 18, -5, -40, -17, -95, 49, 125, -59, pcamera_capture_status);
	pcamera_capture_status_video_status_SET(-51, 113, -60, -83, -85, 108, 76, 4, 75, 121, pcamera_capture_status);
	pcamera_capture_status_image_interval_SET(1.1490792E38F, 3.0348689E37F, -2.2432195E38F, -3.2813205E38F, 3.4835528E37F, 9.076589E37F, -1.4145582E38F, 7.855228E37F, -1.8681527E38F, -2.700842E38F, pcamera_capture_status);
	pcamera_capture_status_available_capacity_SET(3.6220814E37F, 2.8914166E36F, -1.2968377E38F, 2.416177E38F, -2.5284267E38F, 7.846451E37F, -3.3258033E37F, 5.2728495E37F, 8.846688E37F, -1.5265562E38F, pcamera_capture_status);
	
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


void read_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA *const pencapsulated_data) {
	int16_t                 seqnr     = pencapsulated_data_seqnr_GET(pencapsulated_data);
	Vencapsulated_data_daTa item_daTa = pencapsulated_data_daTa_GET(pencapsulated_data);
	for (size_t             index     = 0; index < item_daTa.len; index++)
		some_int8_t = vencapsulated_data_daTa_GET(&item_daTa, index);
	
}


void write_ENCAPSULATED_DATA(pencapsulated_data_ENCAPSULATED_DATA *const pencapsulated_data) {
	pencapsulated_data_seqnr_SET(-26041, -9136, -13928, 27128, -3906, -3627, -24662, 23496, -25057, -15061, pencapsulated_data);
	pencapsulated_data_daTa_SET(&-16, -59, -39, 68, -96, -67, -110, -47, 25, -70, pencapsulated_data);
	
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
	pgps_input_time_week_SET(-14830, -1015, 14363, 23479, -6733, -15415, 31958, -7976, -30662, 26124, pgps_input);
	pgps_input_time_week_ms_SET(-1702621096, -1793105870, -1791831980, 2061855959, -336201345, 682144145, -1888274704, -762920332, -1183940350, 861558178, pgps_input);
	pgps_input_time_usec_SET(-2913911187661244865L, 7382507170332795852L, 7671282620374475368L, 1172551937829490591L, 1672042787627711125L, -6489047226869934881L, -896444160534724379L, -1037650274437420585L, 2076261686432241314L, 5239698868013174988L, pgps_input);
	pgps_input_gps_id_SET(-123, 64, 9, -85, 91, -115, 94, 120, 113, 98, pgps_input);
	pgps_input_fix_type_SET(-34, -117, 2, -6, 8, 29, 27, -115, -29, -39, pgps_input);
	pgps_input_lat_SET(-899640650, 196437806, 323245863, 1700116547, 1557119342, 413480980, 283644441, 1128529187, 1159076867, -184815594, pgps_input);
	pgps_input_lon_SET(-581016260, 271895685, 1545055555, -1493903960, 1387025624, -1587559111, 1746810229, -201282860, 1875653362, 92772529, pgps_input);
	pgps_input_alt_SET(1.0624084E38F, -3.3189567E38F, 1.6125435E38F, -1.6879467E38F, 1.351723E38F, 6.8371526E37F, 3.05107E38F, -2.5998725E38F, 2.3898307E38F, 1.0566518E38F, pgps_input);
	pgps_input_hdop_SET(-2.1360813E38F, -6.3987777E37F, -3.0826356E38F, -2.0901171E38F, 1.832674E38F, 2.503788E38F, -2.8242848E38F, -2.8026964E37F, 1.749029E38F, -3.100379E37F, pgps_input);
	pgps_input_vdop_SET(1.7133537E38F, -2.5893506E38F, -1.5856815E38F, -4.371291E37F, 1.1785231E38F, 2.9465762E37F, 2.002615E38F, 1.6505638E38F, 2.613643E38F, 1.994752E37F, pgps_input);
	pgps_input_vn_SET(1.0861365E38F, 6.6176426E36F, -1.2323895E38F, -9.594769E37F, -1.3949163E38F, -2.389802E38F, -2.5328482E38F, 1.9458617E38F, -1.535558E38F, 7.9352676E37F, pgps_input);
	pgps_input_ve_SET(5.338765E37F, 3.9999873E37F, -1.3955235E37F, 2.4879929E38F, 2.2618276E38F, -1.6665938E37F, -2.4495634E38F, -2.613949E38F, -1.244792E38F, -3.6034355E37F, pgps_input);
	pgps_input_vd_SET(4.6967327E37F, 2.1778335E38F, 2.1962704E38F, 7.886526E36F, -1.3621489E37F, -8.856784E37F, -3.031134E38F, 3.1408068E38F, 8.859015E37F, 1.7029415E38F, pgps_input);
	pgps_input_speed_accuracy_SET(-3.7068363E37F, -1.2344081E38F, -2.3043503E38F, -1.2150404E37F, 3.2140478E38F, -2.4924422E38F, 6.034867E37F, -2.263082E38F, -5.51694E36F, -1.6778601E38F, pgps_input);
	pgps_input_horiz_accuracy_SET(1.6889231E38F, -2.6909147E38F, -2.5934833E38F, 3.1040839E38F, -3.3042404E38F, 2.3630161E38F, -1.5248956E38F, 2.458362E38F, 3.1744078E38F, 2.4288064E38F, pgps_input);
	pgps_input_vert_accuracy_SET(5.1862025E37F, 2.7709067E38F, -3.2801396E38F, -1.8197605E38F, 1.7602531E38F, 2.3264011E38F, 2.6195158E38F, -8.892542E37F, 2.7900726E38F, -1.3533315E38F, pgps_input);
	pgps_input_satellites_visible_SET(-52, 77, -95, 26, -66, -123, -53, 92, -118, 100, pgps_input);
	pgps_input_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY, e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, pgps_input);
	
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


void read_COMPASSMOT_STATUS(pcompassmot_status_COMPASSMOT_STATUS *const pcompassmot_status) {
	int16_t throttle      = pcompassmot_status_throttle_GET(pcompassmot_status);
	int16_t interference  = pcompassmot_status_interference_GET(pcompassmot_status);
	float   current       = pcompassmot_status_current_GET(pcompassmot_status);
	float   CompensationX = pcompassmot_status_CompensationX_GET(pcompassmot_status);
	float   CompensationY = pcompassmot_status_CompensationY_GET(pcompassmot_status);
	float   CompensationZ = pcompassmot_status_CompensationZ_GET(pcompassmot_status);
	
}


void write_COMPASSMOT_STATUS(pcompassmot_status_COMPASSMOT_STATUS *const pcompassmot_status) {
	pcompassmot_status_throttle_SET(-8628, -16579, -22283, 9088, -19244, 9546, 8537, 25468, -18543, 15166, pcompassmot_status);
	pcompassmot_status_interference_SET(-17141, 21513, 14545, -25682, -29046, 31342, 6441, -31741, 8978, 12707, pcompassmot_status);
	pcompassmot_status_current_SET(-1.6506039E38F, -2.8425412E38F, -1.6402308E38F, -8.0743395E37F, -2.0286462E38F, 1.7908895E38F, 2.4507903E38F, -2.0764847E38F, 7.3979237E37F, 8.261419E36F, pcompassmot_status);
	pcompassmot_status_CompensationX_SET(-2.5243795E38F, 1.1701668E38F, -1.8442375E38F, 1.1249144E38F, 4.159594E37F, 3.38317E38F, 2.7856137E38F, -7.297131E36F, -2.5721199E38F, -2.756938E38F, pcompassmot_status);
	pcompassmot_status_CompensationY_SET(3.1216563E38F, 7.947654E37F, 8.18219E36F, -1.9827512E38F, -2.471289E38F, -1.5589545E38F, 2.6721103E38F, 3.1134073E38F, 1.0183454E38F, 4.3332136E37F, pcompassmot_status);
	pcompassmot_status_CompensationZ_SET(1.297203E38F, -2.2006745E38F, 2.7546031E38F, 3.0977742E38F, 1.2592349E38F, 1.1781046E38F, 2.9012871E37F, 1.7941671E38F, -2.6015573E38F, -2.4760183E37F, pcompassmot_status);
	
}


void read_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA *const plog_request_data) {
	int16_t id               = plog_request_data_id_GET(plog_request_data);
	int32_t ofs              = plog_request_data_ofs_GET(plog_request_data);
	int32_t count            = plog_request_data_count_GET(plog_request_data);
	int8_t  target_system    = plog_request_data_target_system_GET(plog_request_data);
	int8_t  target_component = plog_request_data_target_component_GET(plog_request_data);
	
}


void write_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA *const plog_request_data) {
	plog_request_data_id_SET(-12193, 10595, -32235, 5681, 30736, 24780, -7539, 15521, -3485, -12876, plog_request_data);
	plog_request_data_ofs_SET(-1428129568, -1804394939, 651267417, 1004199355, -350567725, -687007707, -1016565611, -1679215560, 1339152955, 37513123, plog_request_data);
	plog_request_data_count_SET(327762209, 1727233812, 1509241629, 1029170157, 49920204, 1397244922, 1129105908, -949696973, 1774778974, -120660789, plog_request_data);
	plog_request_data_target_system_SET(84, -52, -110, -43, 12, 81, -24, 18, 16, 50, plog_request_data);
	plog_request_data_target_component_SET(73, 63, -50, -64, 26, 90, -10, -6, 81, -4, plog_request_data);
	
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


void write_CAMERA_STATUS(pcamera_status_CAMERA_STATUS *const pcamera_status) {
	pcamera_status_img_idx_SET(-16508, -13967, 25684, -11050, 24226, -24358, -22174, -2115, -8715, 28340, pcamera_status);
	pcamera_status_time_usec_SET(-7509768600922858265L, 3196455897412467977L, -7496213379071053443L, -7232238282594580843L, 278922431524226964L, -7433040682214025885L, 8277402013148370518L, -1204752954522999045L, 3853424103812201201L, -6087016704243351380L, pcamera_status);
	pcamera_status_target_system_SET(-27, 61, 28, 62, 3, -15, -36, 39, -8, 92, pcamera_status);
	pcamera_status_cam_idx_SET(-89, -6, -118, 121, -43, 61, 103, 20, 36, -114, pcamera_status);
	pcamera_status_p1_SET(3.1527864E38F, -1.721558E38F, -2.9522217E38F, 1.0174237E38F, -1.6320421E38F, -1.1127322E38F, 1.6942826E37F, 4.585876E37F, -1.4396362E38F, 2.55451E38F, pcamera_status);
	pcamera_status_p2_SET(2.2019127E38F, -1.260956E38F, 2.3864465E36F, -2.3983554E38F, 1.1032589E38F, 2.0123069E38F, 1.0526672E38F, 2.545549E38F, 2.3334953E38F, -2.3291424E37F, pcamera_status);
	pcamera_status_p3_SET(-2.0855666E38F, 2.5582051E38F, 5.863098E37F, 4.9891807E37F, -1.3049542E38F, 2.8942134E38F, -2.1614078E38F, 2.9050303E38F, -3.119631E38F, -2.3797674E38F, pcamera_status);
	pcamera_status_p4_SET(1.8706497E38F, 2.4486838E38F, 1.4586929E38F, 3.0481997E38F, -3.2082614E38F, -3.3581993E38F, -1.7291659E38F, -2.6469958E38F, -1.5653421E37F, 5.397819E37F, pcamera_status);
	pcamera_status_event_id_SET(e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_LOWSTORE, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_ERROR, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_TRIGGER, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_DISCONNECT, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_ERROR, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_LOWSTOREV, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_HEARTBEAT, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_LOWSTOREV, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_ERROR, e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_TRIGGER, pcamera_status);
	
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


void read_CAMERA_SETTINGS(pcamera_settings_CAMERA_SETTINGS *const pcamera_settings) {
	int32_t       time_boot_ms = pcamera_settings_time_boot_ms_GET(pcamera_settings);
	e_CAMERA_MODE item_mode_id;
	if (pcamera_settings_mode_id_GET(pcamera_settings, &item_mode_id)) {
		e_CAMERA_MODE_CAMERA_MODE_IMAGE = item_mode_id;
	}
	
}


void write_CAMERA_SETTINGS(pcamera_settings_CAMERA_SETTINGS *const pcamera_settings) {
	pcamera_settings_time_boot_ms_SET(-1844690904, -321572975, 849887506, 579059255, -927615587, 1076340147, 106484345, -194426361, -2041700881, -1328683100, pcamera_settings);
	pcamera_settings_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, e_CAMERA_MODE_CAMERA_MODE_IMAGE, e_CAMERA_MODE_CAMERA_MODE_VIDEO, e_CAMERA_MODE_CAMERA_MODE_IMAGE, e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, e_CAMERA_MODE_CAMERA_MODE_VIDEO, e_CAMERA_MODE_CAMERA_MODE_IMAGE, e_CAMERA_MODE_CAMERA_MODE_VIDEO, e_CAMERA_MODE_CAMERA_MODE_IMAGE, pcamera_settings);
	
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


void write_DEVICE_OP_READ_REPLY(pdevice_op_read_reply_DEVICE_OP_READ_REPLY *const pdevice_op_read_reply) {
	pdevice_op_read_reply_request_id_SET(-295390087, -231652689, 140138116, -1804710016, -861635079, -1255270380, 1479452109, 1825585747, 1988104713, 1255386250, pdevice_op_read_reply);
	pdevice_op_read_reply_result_SET(-90, -45, -17, 18, 42, -126, 123, -76, -115, 104, pdevice_op_read_reply);
	pdevice_op_read_reply_regstart_SET(-1, 57, -109, -112, 1, 46, -66, -66, 100, -17, pdevice_op_read_reply);
	pdevice_op_read_reply_count_SET(-21, -21, -122, 103, 124, -86, 98, 112, 38, -25, pdevice_op_read_reply);
	pdevice_op_read_reply_daTa_SET(&-95, 78, 64, 95, -63, 28, 100, 40, -31, -36, pdevice_op_read_reply);
	
}


void read_RAW_PRESSURE(praw_pressure_RAW_PRESSURE *const praw_pressure) {
	int64_t time_usec   = praw_pressure_time_usec_GET(praw_pressure);
	int16_t press_abs   = praw_pressure_press_abs_GET(praw_pressure);
	int16_t press_diff1 = praw_pressure_press_diff1_GET(praw_pressure);
	int16_t press_diff2 = praw_pressure_press_diff2_GET(praw_pressure);
	int16_t temperature = praw_pressure_temperature_GET(praw_pressure);
	
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


void write_DIGICAM_CONTROL(pdigicam_control_DIGICAM_CONTROL *const pdigicam_control) {
	pdigicam_control_target_system_SET(95, -37, -80, 121, -90, 50, 50, -47, 20, -28, pdigicam_control);
	pdigicam_control_target_component_SET(-46, 36, -114, -20, 111, -59, -28, 120, 36, 18, pdigicam_control);
	pdigicam_control_session_SET(-15, -81, -53, 23, 92, -89, -83, -109, 73, -101, pdigicam_control);
	pdigicam_control_zoom_pos_SET(-12, -34, 91, -12, -116, 19, 115, 119, 57, -62, pdigicam_control);
	pdigicam_control_zoom_step_SET(94, -5, 8, -28, 45, 122, -123, -51, 7, -7, pdigicam_control);
	pdigicam_control_focus_lock_SET(-44, 50, 63, -91, -104, 110, -39, 126, -12, -71, pdigicam_control);
	pdigicam_control_shot_SET(57, 68, 68, -110, -15, -32, -4, 107, -100, 93, pdigicam_control);
	pdigicam_control_command_id_SET(-93, -46, -102, 73, 52, -67, 99, 69, 31, 101, pdigicam_control);
	pdigicam_control_extra_param_SET(-104, -125, -86, -105, -79, -67, 6, 100, -32, -31, pdigicam_control);
	pdigicam_control_extra_value_SET(2.162738E38F, -2.8498061E38F, 4.961578E37F, -2.5628236E38F, 7.141218E37F, -9.757571E37F, 2.2357688E38F, -1.6579737E38F, 4.8488584E37F, 5.017171E37F, pdigicam_control);
	
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
	pnamed_value_float_time_boot_ms_SET(-922543722, -815697527, 835853660, 444431429, 1436177762, -431059770, 956878996, -656059714, 563518971, -2009921876, pnamed_value_float);
	pnamed_value_float_value_SET(2.3895498E38F, -1.7645729E38F, -4.217717E37F, 2.037659E38F, 1.6348573E38F, -1.079598E38F, 3.2122414E38F, -3.6602057E37F, 2.7280403E38F, 2.1483236E38F, pnamed_value_float);
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


void write_GOPRO_HEARTBEAT(pgopro_heartbeat_GOPRO_HEARTBEAT *const pgopro_heartbeat) {
	pgopro_heartbeat_status_SET(e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_CONNECTED, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_ERROR, e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED, pgopro_heartbeat);
	pgopro_heartbeat_capture_mode_SET(e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_UNKNOWN, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_TIME_LAPSE, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PLAYBACK, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_UNKNOWN, e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PLAYBACK, pgopro_heartbeat);
	pgopro_heartbeat_flags_SET(e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, pgopro_heartbeat);
	
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


void read_AHRS2(pahrs2_AHRS2 *const pahrs2) {
	float   roll     = pahrs2_roll_GET(pahrs2);
	float   pitch    = pahrs2_pitch_GET(pahrs2);
	float   yaw      = pahrs2_yaw_GET(pahrs2);
	float   altitude = pahrs2_altitude_GET(pahrs2);
	int32_t lat      = pahrs2_lat_GET(pahrs2);
	int32_t lng      = pahrs2_lng_GET(pahrs2);
	
}


void write_AHRS2(pahrs2_AHRS2 *const pahrs2) {
	pahrs2_roll_SET(-5.4567475E37F, 1.6676354E38F, -2.954115E37F, 2.2819492E38F, 1.785852E37F, 2.4826848E38F, -2.8414877E38F, -4.667514E37F, -8.988983E37F, 3.2386025E38F, pahrs2);
	pahrs2_pitch_SET(-3.3190987E38F, 1.7322184E38F, 1.5943915E38F, -8.530554E37F, 1.5634687E38F, -2.2024622E38F, -1.1119865E38F, 7.106236E37F, -1.749342E38F, 1.745083E38F, pahrs2);
	pahrs2_yaw_SET(2.1245625E38F, -1.7966842E38F, 1.3172382E38F, 3.3285302E38F, 2.6257273E38F, 3.2421733E37F, -3.3307806E38F, -2.8090795E37F, -3.3612881E38F, -2.9442102E38F, pahrs2);
	pahrs2_altitude_SET(-1.6317274E38F, 2.3025944E38F, -1.5494337E38F, 2.713704E38F, -1.77135E38F, 1.112248E38F, -3.0798395E37F, -1.2603461E38F, -2.1816476E38F, 3.2704801E38F, pahrs2);
	pahrs2_lat_SET(-2053408737, -135417595, -1596467631, -1200627449, 27885176, -1933571148, -1007327215, 1056744866, -1547628470, -852741650, pahrs2);
	pahrs2_lng_SET(-944534573, 1704999311, -1299899689, 18041052, -1749460477, 482571819, -1632082867, 1501770640, 1517668685, -556317113, pahrs2);
	
}


void read_LOG_ERASE(plog_erase_LOG_ERASE *const plog_erase) {
	int8_t target_system    = plog_erase_target_system_GET(plog_erase);
	int8_t target_component = plog_erase_target_component_GET(plog_erase);
	
}


void write_LOG_ERASE(plog_erase_LOG_ERASE *const plog_erase) {
	plog_erase_target_system_SET(116, -31, -119, -18, 4, 91, 59, 70, 101, -2, plog_erase);
	plog_erase_target_component_SET(-17, 96, 24, 102, -13, 30, -6, 123, -120, -70, plog_erase);
	
}


void read_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST *const pterrain_request) {
	int16_t grid_spacing = pterrain_request_grid_spacing_GET(pterrain_request);
	int64_t mask         = pterrain_request_mask_GET(pterrain_request);
	int32_t lat          = pterrain_request_lat_GET(pterrain_request);
	int32_t lon          = pterrain_request_lon_GET(pterrain_request);
	
}


void write_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST *const pterrain_request) {
	pterrain_request_grid_spacing_SET(18764, 19393, -16202, 2268, 24138, 20577, -17186, 12217, -18497, 11384, pterrain_request);
	pterrain_request_mask_SET(7819902497133485516L, -81556576031287481L, 4713695107268350061L, -5533291868573578737L, -4313620204906481860L, -6552869619390875067L, 6264092264859008366L, -3444359189421612522L, 922766268309681240L, 2577300878908116575L, pterrain_request);
	pterrain_request_lat_SET(-298609780, -1868830800, -346594017, -1089690629, -2143135546, -902023940, -110459084, 1965716901, 858270562, -1605935894, pterrain_request);
	pterrain_request_lon_SET(860686542, 548819666, 1383404870, 290144455, -732973489, 1087473882, 1396968828, 1277862041, -786181303, 1618725228, pterrain_request);
	
}


void read_MOUNT_STATUS(pmount_status_MOUNT_STATUS *const pmount_status) {
	int8_t  target_system    = pmount_status_target_system_GET(pmount_status);
	int8_t  target_component = pmount_status_target_component_GET(pmount_status);
	int32_t pointing_a       = pmount_status_pointing_a_GET(pmount_status);
	int32_t pointing_b       = pmount_status_pointing_b_GET(pmount_status);
	int32_t pointing_c       = pmount_status_pointing_c_GET(pmount_status);
	
}


void write_MOUNT_STATUS(pmount_status_MOUNT_STATUS *const pmount_status) {
	pmount_status_target_system_SET(-115, 99, 93, -27, 40, 43, -76, 125, 86, 1, pmount_status);
	pmount_status_target_component_SET(22, -9, -58, -114, 61, -94, 68, -32, -104, -94, pmount_status);
	pmount_status_pointing_a_SET(2114766363, -422122710, 472075770, -637522399, 793434856, -1114174722, 2092397818, -467357948, -158457857, -1807093470, pmount_status);
	pmount_status_pointing_b_SET(317320569, -1081076008, 891139957, -564128508, 1444350021, 648608659, 28233766, -633941253, 1803652284, -105877273, pmount_status);
	pmount_status_pointing_c_SET(1902099196, -654883931, 1846092048, 22651411, -1171764314, -1264104868, -121690862, 589092404, -1325428354, 1049665118, pmount_status);
	
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


void write_PID_TUNING(ppid_tuning_PID_TUNING *const ppid_tuning) {
	ppid_tuning_desired_SET(-3.1818787E38F, -3.8724018E37F, -6.6066674E37F, 1.5574739E38F, 1.1322973E38F, -2.5352075E38F, -1.5376682E38F, 2.9082225E38F, -1.8654349E38F, -2.2514432E38F, ppid_tuning);
	ppid_tuning_achieved_SET(-2.5560838E38F, -8.1447636E37F, -1.4825824E38F, 6.407567E37F, 1.0736025E38F, 1.0325207E38F, 7.3190916E37F, 1.402446E38F, -2.5699505E38F, -1.1226539E38F, ppid_tuning);
	ppid_tuning_FF_SET(1.6308949E37F, -2.6305222E38F, -1.3760623E38F, 2.8028556E38F, -1.4522037E38F, 1.3344879E37F, 1.8113563E38F, 5.091681E37F, 1.833244E38F, 2.93461E38F, ppid_tuning);
	ppid_tuning_P_SET(2.8929328E37F, -1.9623288E38F, 4.3543337E37F, 8.701083E37F, 3.5330254E37F, 1.04433664E37F, 2.341855E38F, 2.1375715E38F, 2.1156727E38F, 2.4849838E37F, ppid_tuning);
	ppid_tuning_I_SET(-2.9493279E38F, 2.5590059E38F, -3.3642313E38F, -1.6376406E38F, -8.604409E37F, -2.5572823E38F, -2.0409505E38F, 5.037949E37F, -2.4850749E38F, -1.4245414E38F, ppid_tuning);
	ppid_tuning_D_SET(2.5288792E38F, 3.9182811E37F, -3.6884378E37F, 4.573778E37F, -2.4897998E38F, -2.9695752E38F, -2.5958833E38F, -1.1416801E38F, 1.607781E37F, 8.0330704E37F, ppid_tuning);
	ppid_tuning_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_ACCZ, e_PID_TUNING_AXIS_PID_TUNING_PITCH, e_PID_TUNING_AXIS_PID_TUNING_STEER, e_PID_TUNING_AXIS_PID_TUNING_LANDING, e_PID_TUNING_AXIS_PID_TUNING_PITCH, e_PID_TUNING_AXIS_PID_TUNING_ACCZ, e_PID_TUNING_AXIS_PID_TUNING_YAW, e_PID_TUNING_AXIS_PID_TUNING_ROLL, e_PID_TUNING_AXIS_PID_TUNING_LANDING, e_PID_TUNING_AXIS_PID_TUNING_LANDING, ppid_tuning);
	
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
	poptical_flow_rad_integration_time_us_SET(-1481950147, -77307125, -498590627, 1745603784, 548192955, 1126537460, 948755537, -1792990324, -386533007, 1700727101, poptical_flow_rad);
	poptical_flow_rad_time_delta_distance_us_SET(-1247512894, 775997566, 1933071370, 729322918, -285630663, 1142699460, 2096236288, 137004741, -1179749214, 1765468049, poptical_flow_rad);
	poptical_flow_rad_time_usec_SET(-8441826901778788549L, 1601131271094246137L, 8837743595206015564L, 8637945365499507278L, -1954654296763074395L, -7021987652784241342L, 4563740482789961666L, 8735852476947453169L, -5253617674584048928L, -1329586347506716316L, poptical_flow_rad);
	poptical_flow_rad_sensor_id_SET(-37, -92, -62, 43, 74, 127, 125, -40, -83, 78, poptical_flow_rad);
	poptical_flow_rad_integrated_x_SET(-1.6915608E38F, 6.2427097E37F, 2.1199775E38F, 2.659654E38F, -2.6938277E38F, 2.7067109E38F, -1.9885856E38F, 2.2633032E37F, -2.7452178E36F, -2.6635035E38F, poptical_flow_rad);
	poptical_flow_rad_integrated_y_SET(-1.0918922E38F, -3.2035885E38F, -1.2800505E38F, -3.1955116E37F, 1.8774443E38F, -3.1743867E38F, -1.762861E38F, 2.676136E38F, 1.4836698E38F, 3.2318004E38F, poptical_flow_rad);
	poptical_flow_rad_integrated_xgyro_SET(-2.5169545E38F, -2.5376851E37F, 7.657242E37F, 1.8865E38F, -1.9439515E38F, -1.8490546E37F, -1.1816611E38F, -9.38191E37F, 6.578508E36F, 1.6812621E38F, poptical_flow_rad);
	poptical_flow_rad_integrated_ygyro_SET(-9.748058E37F, 1.6981487E38F, -5.6642467E37F, -2.3815372E38F, -2.7216349E38F, -9.111691E37F, 1.5901201E38F, 3.0927182E38F, -3.0441463E38F, 5.045261E37F, poptical_flow_rad);
	poptical_flow_rad_integrated_zgyro_SET(-2.089577E38F, -1.020831E38F, -2.5737295E38F, -1.7796543E38F, -1.8682584E38F, 9.766289E37F, 1.9279733E37F, -7.361267E37F, 2.878466E38F, -1.7965333E38F, poptical_flow_rad);
	poptical_flow_rad_temperature_SET(-5807, 20866, 20531, -5179, 5039, -4582, -1756, 9114, -25386, 16672, poptical_flow_rad);
	poptical_flow_rad_quality_SET(-115, 62, -125, 73, -66, -10, -61, -53, 39, 15, poptical_flow_rad);
	poptical_flow_rad_distance_SET(3.7978234E37F, -3.376733E38F, 1.621846E38F, 3.1570319E38F, 1.002165E38F, 2.4028543E37F, 2.0270392E38F, 2.137696E38F, -7.131495E37F, -9.93701E37F, poptical_flow_rad);
	
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
	plog_data_id_SET(-21863, 6958, 281, -20955, -22919, -20612, 19524, 18708, -30183, 18700, plog_data);
	plog_data_ofs_SET(1313354382, -906977171, -839402246, -1375568114, 1703454967, -1639491919, 305391086, -1220704239, -1183279509, -933486149, plog_data);
	plog_data_count_SET(-12, 87, -120, 80, -32, -77, 122, 11, 3, 92, plog_data);
	plog_data_daTa_SET(&-3, -81, -31, 47, -108, -39, -121, -61, -111, 53, plog_data);
	
}


void read_MISSION_CLEAR_ALL(pmission_clear_all_MISSION_CLEAR_ALL *const pmission_clear_all) {
	int8_t             target_system    = pmission_clear_all_target_system_GET(pmission_clear_all);
	int8_t             target_component = pmission_clear_all_target_component_GET(pmission_clear_all);
	e_MAV_MISSION_TYPE item_mission_type;
	if (pmission_clear_all_mission_type_GET(pmission_clear_all, &item_mission_type)) {
		e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION = item_mission_type;
	}
	
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


void write_AHRS3(pahrs3_AHRS3 *const pahrs3) {
	pahrs3_roll_SET(7.838708E37F, -2.1639287E38F, -9.470865E37F, 2.4763177E37F, -3.7624717E37F, -2.4052311E38F, 4.296512E37F, -1.371687E38F, 2.770705E38F, 5.173316E37F, pahrs3);
	pahrs3_pitch_SET(-2.2223805E38F, 2.8866032E37F, -6.7956497E37F, 1.4468657E38F, -9.201215E37F, 2.2968202E38F, 2.4181405E38F, -4.775178E37F, 3.2683412E38F, 7.693614E37F, pahrs3);
	pahrs3_yaw_SET(-2.4566059E38F, 3.2418842E38F, 5.3465527E37F, -2.4085522E38F, 1.2605789E38F, 1.9200582E38F, -3.2637188E38F, 5.298669E37F, -5.035177E37F, 2.161356E38F, pahrs3);
	pahrs3_altitude_SET(7.368009E37F, 2.8721841E38F, -5.2137374E37F, -2.9320675E38F, -2.3380432E38F, 2.7622265E38F, 2.1769072E37F, -1.8735718E38F, 2.3085988E38F, 4.377767E37F, pahrs3);
	pahrs3_lat_SET(1185361058, -917179748, -2064823763, -1224472335, 1177246047, -557653661, -989284287, 146130011, 1561564584, 417047710, pahrs3);
	pahrs3_lng_SET(-1988821009, 647460675, 1456257405, -909731065, 483599774, -811642002, 1719702699, 1518681076, 472468588, 1641201361, pahrs3);
	pahrs3_v1_SET(-2.1308255E38F, -1.7150428E38F, -1.0445137E38F, 1.4376859E37F, 3.0958176E38F, -5.756554E37F, -2.480472E38F, 5.9613607E37F, 1.2427188E38F, 5.1809793E37F, pahrs3);
	pahrs3_v2_SET(1.3681406E38F, 1.1437417E38F, 1.0057119E36F, -2.9556164E38F, -2.5068874E38F, -2.5860322E38F, 1.5595431E38F, -3.6894258E36F, -3.2427985E38F, -2.224918E37F, pahrs3);
	pahrs3_v3_SET(-2.7273225E38F, -3.1128027E38F, -1.0840052E38F, 3.2332966E38F, -3.3905451E38F, -3.3064978E38F, 1.4249066E38F, -1.9962227E38F, 2.335043E37F, 3.060085E38F, pahrs3);
	pahrs3_v4_SET(-1.2145523E38F, 3.0581545E38F, 5.768325E37F, -7.5991617E37F, 1.724651E38F, -1.3256432E38F, 1.2984969E38F, -3.3895667E38F, -2.851905E38F, -1.1319377E38F, pahrs3);
	
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
	pvicon_position_estimate_usec_SET(-4502790053882913731L, -6184082880561845497L, -5539576005715601188L, -3344558593911456224L, -100545702722008148L, -5710474232780187705L, -1700862091364108116L, -6497244946253028690L, -5790665984061176287L, -6600899989770728006L, pvicon_position_estimate);
	pvicon_position_estimate_x_SET(1.984069E38F, -4.0351633E37F, 1.8643867E38F, 2.2267869E38F, 2.9705984E38F, 1.6168502E38F, -2.5911515E38F, 2.17905E38F, 1.0281026E38F, -9.1447E37F, pvicon_position_estimate);
	pvicon_position_estimate_y_SET(2.0595154E38F, -1.7292642E37F, 1.3946977E38F, -1.2587418E38F, -6.576461E37F, 2.0945835E38F, -7.2199496E37F, -2.1798305E38F, -5.039367E37F, -1.082329E38F, pvicon_position_estimate);
	pvicon_position_estimate_z_SET(1.4963101E38F, 3.3469951E38F, -6.669876E37F, 9.814743E37F, -9.136025E37F, -3.0450627E38F, 2.9952008E38F, -2.125238E38F, 9.294195E37F, 2.2253825E38F, pvicon_position_estimate);
	pvicon_position_estimate_roll_SET(-2.283623E38F, 2.3997908E38F, -1.1482716E38F, -1.6238311E38F, 1.1450852E38F, 1.5600855E38F, -1.1974782E38F, -1.9884452E37F, 3.7636427E37F, -1.1994135E38F, pvicon_position_estimate);
	pvicon_position_estimate_pitch_SET(3.1211373E38F, -2.7070572E37F, -1.904697E37F, -1.2012663E38F, -1.2183182E38F, -1.542618E38F, -7.5670557E37F, 1.229209E38F, -2.0669964E38F, 4.32956E37F, pvicon_position_estimate);
	pvicon_position_estimate_yaw_SET(6.1656867E37F, 3.4110406E37F, 1.5568888E37F, -2.485078E38F, -9.410169E37F, 5.0325186E37F, 9.758304E37F, 3.3595236E38F, -5.4378753E37F, 3.2614656E38F, pvicon_position_estimate);
	
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
	pgps2_rtk_wn_SET(-29331, 14542, 9869, 32386, 22359, 5171, 10773, 20225, 15921, 4807, pgps2_rtk);
	pgps2_rtk_time_last_baseline_ms_SET(-1797185739, 1665288268, 1481442675, 1461165072, 168735348, 1391041356, -2055773522, -489955890, 1732113950, -317556285, pgps2_rtk);
	pgps2_rtk_tow_SET(-1736923480, 1966720081, 1681600580, -1891335705, -970415934, 1290615756, 1970888181, 632192011, 446803943, -776759517, pgps2_rtk);
	pgps2_rtk_accuracy_SET(-1428140386, -1401233523, -11659496, -1565057101, 1013272872, -2025211108, 1126070494, -1154844386, 1191669979, 1487184340, pgps2_rtk);
	pgps2_rtk_rtk_receiver_id_SET(86, -92, -68, -11, 26, -66, 13, 4, 62, -128, pgps2_rtk);
	pgps2_rtk_rtk_health_SET(65, 16, -15, -47, 112, 55, -9, 0, -119, 47, pgps2_rtk);
	pgps2_rtk_rtk_rate_SET(-86, 80, 85, -119, 18, 59, -9, -61, 125, 68, pgps2_rtk);
	pgps2_rtk_nsats_SET(-70, -85, 19, -56, -45, 66, 102, -84, 114, 40, pgps2_rtk);
	pgps2_rtk_baseline_coords_type_SET(21, 33, 108, -123, -34, 17, -23, -55, -116, 113, pgps2_rtk);
	pgps2_rtk_baseline_a_mm_SET(-1808563244, 882427052, -433066890, 908088650, 412933731, 677886293, -239900866, -1914676961, -992355427, -1299128519, pgps2_rtk);
	pgps2_rtk_baseline_b_mm_SET(-130177990, -64708001, -1931319478, -2017267722, -1552004386, 1559978117, 1625234888, 385279392, 912170356, 942471547, pgps2_rtk);
	pgps2_rtk_baseline_c_mm_SET(-1749830390, -141143082, -994226864, 1751863937, -1339378352, -1644262133, 228357071, -1732375106, -608979076, 1731230724, pgps2_rtk);
	pgps2_rtk_iar_num_hypotheses_SET(-1216611031, -42057163, 1465557714, -1194008333, -502418652, 493362614, 330864058, 1055513439, 2056822750, -161843959, pgps2_rtk);
	
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


void write_MAG_CAL_REPORT(pmag_cal_report_MAG_CAL_REPORT *const pmag_cal_report) {
	pmag_cal_report_compass_id_SET(-77, 6, -92, 122, -105, -26, 13, -2, -27, 108, pmag_cal_report);
	pmag_cal_report_cal_mask_SET(-43, -27, -92, -66, -126, -80, -107, 70, -83, 115, pmag_cal_report);
	pmag_cal_report_autosaved_SET(97, -42, -115, 16, -44, 120, -47, 55, 36, -33, pmag_cal_report);
	pmag_cal_report_fitness_SET(-2.4200533E38F, 3.0947665E38F, -2.211231E38F, 1.7007669E38F, 2.1490132E38F, -4.5873987E36F, 2.5028295E38F, 1.3054982E37F, 2.1078415E38F, -6.417212E37F, pmag_cal_report);
	pmag_cal_report_ofs_x_SET(2.2923455E38F, -6.1710276E37F, 2.7143768E38F, 1.9342265E38F, -2.6814723E38F, 2.5475944E38F, -8.353776E37F, 1.4375669E38F, -1.9023729E37F, -2.3059364E38F, pmag_cal_report);
	pmag_cal_report_ofs_y_SET(-2.2142785E38F, -4.4283266E36F, 2.2020815E38F, -3.2657154E38F, 2.7425175E38F, 2.7083493E38F, -2.3905278E38F, 2.8879943E38F, -2.211078E38F, 2.5433722E38F, pmag_cal_report);
	pmag_cal_report_ofs_z_SET(1.1411851E38F, 1.8323188E38F, 2.0404595E37F, -2.5662152E38F, 3.1502672E37F, 1.8074588E38F, -2.2889234E37F, -1.2026088E38F, 2.8343962E38F, 1.4339418E38F, pmag_cal_report);
	pmag_cal_report_diag_x_SET(2.3666538E38F, 2.3974084E38F, -8.605197E36F, -9.576944E37F, -2.7324367E38F, -8.627016E37F, -2.8587111E38F, -1.8661318E38F, 2.0171309E37F, 5.288451E37F, pmag_cal_report);
	pmag_cal_report_diag_y_SET(1.7531803E38F, 1.7839322E38F, -9.06131E37F, 9.604482E37F, 2.2116964E38F, 1.118179E38F, -3.1020238E38F, -2.5722012E38F, -1.3251918E38F, 1.7497811E38F, pmag_cal_report);
	pmag_cal_report_diag_z_SET(-1.9719413E38F, -6.368263E37F, 2.0262963E38F, 2.933035E38F, -2.2996115E38F, 2.5830997E38F, 1.2173845E38F, -2.2135461E38F, -1.0651214E38F, -1.1485183E38F, pmag_cal_report);
	pmag_cal_report_offdiag_x_SET(5.7654803E37F, 1.9755742E38F, -1.9809795E38F, 2.626883E38F, 5.159272E37F, 1.1751802E37F, -8.084655E37F, 3.0954863E38F, -2.4176235E38F, 2.0171114E38F, pmag_cal_report);
	pmag_cal_report_offdiag_y_SET(-1.7190099E38F, -2.3728083E38F, -2.2791922E37F, 2.997279E38F, 2.9315325E38F, -2.151226E38F, -4.5725846E37F, 1.6594628E38F, 1.3702789E38F, -9.359213E36F, pmag_cal_report);
	pmag_cal_report_offdiag_z_SET(-8.383439E37F, 2.6211138E38F, 1.9378544E37F, -3.3001206E38F, -3.2446126E38F, 4.1161157E36F, -2.335635E38F, 7.9598727E37F, -2.2883146E38F, 3.738132E37F, pmag_cal_report);
	pmag_cal_report_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED, e_MAG_CAL_STATUS_MAG_CAL_SUCCESS, e_MAG_CAL_STATUS_MAG_CAL_WAITING_TO_START, e_MAG_CAL_STATUS_MAG_CAL_FAILED, e_MAG_CAL_STATUS_MAG_CAL_SUCCESS, e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED, e_MAG_CAL_STATUS_MAG_CAL_SUCCESS, e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE, e_MAG_CAL_STATUS_MAG_CAL_WAITING_TO_START, e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED, pmag_cal_report);
	
}


void read_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST *const plog_request_list) {
	int16_t start            = plog_request_list_start_GET(plog_request_list);
	int16_t end              = plog_request_list_end_GET(plog_request_list);
	int8_t  target_system    = plog_request_list_target_system_GET(plog_request_list);
	int8_t  target_component = plog_request_list_target_component_GET(plog_request_list);
	
}


void write_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST *const plog_request_list) {
	plog_request_list_start_SET(30690, -20020, 5269, 2552, 21169, 10526, 32603, -25943, -8584, -17164, plog_request_list);
	plog_request_list_end_SET(31533, 17710, 30604, -23810, 6998, -24618, 25970, 1537, -32711, -25678, plog_request_list);
	plog_request_list_target_system_SET(20, 81, 119, 11, -37, -76, 32, -24, -22, 110, plog_request_list);
	plog_request_list_target_component_SET(69, -48, -27, 24, -45, 19, -121, -109, 38, 85, plog_request_list);
	
}


void read_SCALED_PRESSURE(pscaled_pressure_SCALED_PRESSURE *const pscaled_pressure) {
	int32_t time_boot_ms = pscaled_pressure_time_boot_ms_GET(pscaled_pressure);
	float   press_abs    = pscaled_pressure_press_abs_GET(pscaled_pressure);
	float   press_diff   = pscaled_pressure_press_diff_GET(pscaled_pressure);
	int16_t temperature  = pscaled_pressure_temperature_GET(pscaled_pressure);
	
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
	pv2_extension_message_type_SET(-17651, 26299, 31759, 27198, -6485, 1617, -30509, -24387, -3368, -12893, pv2_extension);
	pv2_extension_target_network_SET(-9, -5, 0, 94, -126, -24, 3, -22, 32, -115, pv2_extension);
	pv2_extension_target_system_SET(115, -36, 111, 36, -78, 32, 82, 24, 77, -18, pv2_extension);
	pv2_extension_target_component_SET(-30, -118, 46, 97, 46, 125, 74, -124, 126, 46, pv2_extension);
	pv2_extension_payload_SET(&-90, -118, -10, -118, -124, 115, 88, -41, 100, 26, pv2_extension);
	
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


void read_POWER_STATUS(ppower_status_POWER_STATUS *const ppower_status) {
	int16_t            Vcc    = ppower_status_Vcc_GET(ppower_status);
	int16_t            Vservo = ppower_status_Vservo_GET(ppower_status);
	e_MAV_POWER_STATUS item_flags;
	if (ppower_status_flags_GET(ppower_status, &item_flags)) {
		e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID = item_flags;
	}
	
}


void write_POWER_STATUS(ppower_status_POWER_STATUS *const ppower_status) {
	ppower_status_Vcc_SET(16633, 21324, 27584, 11212, 12482, -12096, 28098, -28117, -28685, 8926, ppower_status);
	ppower_status_Vservo_SET(-21147, 13860, -26951, 4629, 3614, 32161, 19080, 10377, 8310, -28136, ppower_status);
	ppower_status_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED, e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID, e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT, e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED, e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED, e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED, e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, ppower_status);
	
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


void write_REMOTE_LOG_DATA_BLOCK(premote_log_data_block_REMOTE_LOG_DATA_BLOCK *const premote_log_data_block) {
	premote_log_data_block_target_system_SET(-77, 122, -102, -65, -63, 95, -18, -87, -6, 82, premote_log_data_block);
	premote_log_data_block_target_component_SET(-104, 113, -111, 25, -101, -118, 27, -47, 47, 110, premote_log_data_block);
	premote_log_data_block_daTa_SET(&-51, -86, 74, 88, -91, -66, 52, 57, 79, 21, premote_log_data_block);
	premote_log_data_block_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_START, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_START, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_START, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_START, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_START, e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_START, premote_log_data_block);
	
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


void write_LOGGING_DATA_ACKED(plogging_data_acked_LOGGING_DATA_ACKED *const plogging_data_acked) {
	plogging_data_acked_sequence_SET(-1746, -27501, 28147, -20316, 7407, -4346, -3109, 29616, 31021, -24089, plogging_data_acked);
	plogging_data_acked_target_system_SET(19, 111, 111, -62, -77, 97, -111, 105, 113, 125, plogging_data_acked);
	plogging_data_acked_target_component_SET(-51, -17, 36, 54, 6, 99, 42, -17, 13, -64, plogging_data_acked);
	plogging_data_acked_length_SET(109, 17, 61, -1, 65, -112, -105, -85, 117, -94, plogging_data_acked);
	plogging_data_acked_first_message_offset_SET(24, -105, 46, -88, 44, 108, -57, 59, -59, 98, plogging_data_acked);
	plogging_data_acked_daTa_SET(&-71, 54, 93, -61, -50, -63, 21, -69, -120, 44, plogging_data_acked);
	
}


void read_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK *const pterrain_check) {
	int32_t lat = pterrain_check_lat_GET(pterrain_check);
	int32_t lon = pterrain_check_lon_GET(pterrain_check);
	
}


void write_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK *const pterrain_check) {
	pterrain_check_lat_SET(-1585907860, -1777982992, -807934230, 1069898912, -1228097705, 658079352, 496002442, -1377463327, 546536973, -1774850541, pterrain_check);
	pterrain_check_lon_SET(313776785, -1787629113, -601988187, 379018329, -605529769, -67754885, -1328252395, -268940168, 1614079629, -1120647840, pterrain_check);
	
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


void write_MOUNT_CONFIGURE(pmount_configure_MOUNT_CONFIGURE *const pmount_configure) {
	pmount_configure_target_system_SET(-101, 52, 87, 52, -126, -38, -112, 60, 71, 67, pmount_configure);
	pmount_configure_target_component_SET(61, 59, -17, -107, 12, -9, 35, -118, -14, 1, pmount_configure);
	pmount_configure_stab_roll_SET(103, -47, 68, 40, -128, -116, -47, 89, 105, -127, pmount_configure);
	pmount_configure_stab_pitch_SET(92, 117, -21, 4, 50, -106, 45, 38, 18, -104, pmount_configure);
	pmount_configure_stab_yaw_SET(-43, -32, -86, -114, 46, 16, -119, 89, -5, 59, pmount_configure);
	pmount_configure_mount_mode_SET(e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RETRACT, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RC_TARGETING, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_MAVLINK_TARGETING, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_GPS_POINT, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RETRACT, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_GPS_POINT, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RETRACT, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_MAVLINK_TARGETING, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RETRACT, e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RC_TARGETING, pmount_configure);
	
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


void read_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET *const plocal_position_ned_system_global_offset) {
	int32_t time_boot_ms = plocal_position_ned_system_global_offset_time_boot_ms_GET(plocal_position_ned_system_global_offset);
	float   x            = plocal_position_ned_system_global_offset_x_GET(plocal_position_ned_system_global_offset);
	float   y            = plocal_position_ned_system_global_offset_y_GET(plocal_position_ned_system_global_offset);
	float   z            = plocal_position_ned_system_global_offset_z_GET(plocal_position_ned_system_global_offset);
	float   roll         = plocal_position_ned_system_global_offset_roll_GET(plocal_position_ned_system_global_offset);
	float   pitch        = plocal_position_ned_system_global_offset_pitch_GET(plocal_position_ned_system_global_offset);
	float   yaw          = plocal_position_ned_system_global_offset_yaw_GET(plocal_position_ned_system_global_offset);
	
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


void read_DATA_STREAM(pdata_stream_DATA_STREAM *const pdata_stream) {
	int16_t message_rate = pdata_stream_message_rate_GET(pdata_stream);
	int8_t  stream_id    = pdata_stream_stream_id_GET(pdata_stream);
	int8_t  on_off       = pdata_stream_on_off_GET(pdata_stream);
	
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
	pterrain_report_spacing_SET(29193, 21898, -4011, -9456, 20290, -6098, 18848, -22503, -892, 25859, pterrain_report);
	pterrain_report_pending_SET(30206, 25544, -18615, -11230, 4764, 25848, -3736, -10966, 16423, 9383, pterrain_report);
	pterrain_report_loaded_SET(17570, -8920, 12873, 14832, -3791, -3253, 19956, -1547, 2973, 15702, pterrain_report);
	pterrain_report_lat_SET(989242671, -902997200, 1538255136, -1988532852, -441397401, -1016849187, -2112225239, -1365246464, 71063644, -1386083070, pterrain_report);
	pterrain_report_lon_SET(1014005026, 1514035933, 348365034, 678752807, 633234569, 1116197652, -929480616, -1241566447, -1685493503, 601388274, pterrain_report);
	pterrain_report_terrain_height_SET(-2.7047046E38F, -2.9897588E38F, 2.5076908E37F, -1.6001494E38F, 1.2327514E38F, 2.902962E38F, -1.4655057E38F, 8.838362E37F, -5.0140495E37F, -2.9539344E38F, pterrain_report);
	pterrain_report_current_height_SET(1.0442971E38F, 7.292977E37F, -1.3343163E37F, -2.375274E37F, -3.3753185E38F, 1.0710344E38F, 5.357517E37F, -1.9796509E37F, 2.9455797E38F, 1.7348415E38F, pterrain_report);
	
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
	pset_home_position_target_system_SET(112, 2, 116, 56, 118, -21, 15, 57, -75, 125, pset_home_position);
	pset_home_position_latitude_SET(-783921469, 1744513693, -1780107656, -899367094, -963748977, -1255464812, -885788778, -1016882078, 1546875541, 722591666, pset_home_position);
	pset_home_position_longitude_SET(-1796448912, -1051122021, -2086982248, -1885562330, -1735699844, 1644786203, -560822976, -1183938699, 754516015, -856765456, pset_home_position);
	pset_home_position_altitude_SET(1437827626, 639083959, 1249605907, -830083698, -1711705481, 1160321743, -120628448, -1402892587, -1199846833, -1679801662, pset_home_position);
	pset_home_position_x_SET(-6.582795E37F, 2.717947E38F, 1.4878834E38F, -2.3210125E38F, -1.200007E38F, -3.3276992E37F, 1.0944826E38F, -2.5764351E38F, -9.684807E37F, 2.1561364E38F, pset_home_position);
	pset_home_position_y_SET(8.668904E37F, -1.2550914E38F, -4.692605E37F, 3.3625955E38F, -6.5636093E37F, 2.0994886E38F, -7.556863E36F, 2.7088987E38F, 1.793072E38F, -4.681111E35F, pset_home_position);
	pset_home_position_z_SET(-2.2558917E38F, -6.126809E36F, 3.088682E37F, 9.866752E37F, -1.9284468E38F, 6.4469185E37F, 1.2499631E38F, 2.2905185E38F, -1.4135242E38F, -2.5007578E38F, pset_home_position);
	pset_home_position_q_SET(&3.2624234E38F, 3.822436E37F, 2.8120946E38F, -1.1738428E38F, 4.1263203E37F, 2.3717507E38F, -7.8607855E37F, -1.0181196E38F, 2.345793E37F, 1.501668E38F, pset_home_position);
	pset_home_position_approach_x_SET(1.1404861E37F, -9.143494E37F, 2.4075969E38F, -3.3721897E37F, 2.9144552E37F, 7.422118E37F, 2.5270487E38F, 2.991127E38F, 3.1444339E38F, 3.3536056E38F, pset_home_position);
	pset_home_position_approach_y_SET(-7.6872533E37F, -2.9190789E38F, 1.8527028E38F, 1.9525316E38F, 1.3050799E37F, -2.4211658E38F, -1.1822623E38F, -9.571641E37F, -2.5195018E38F, -2.8304153E38F, pset_home_position);
	pset_home_position_approach_z_SET(-1.1480674E38F, -2.4965127E38F, 2.8115578E38F, -2.3166094E38F, -1.756416E38F, 3.1126773E38F, -2.075569E38F, 8.340847E37F, 2.1936107E38F, 2.6615963E38F, pset_home_position);
	pset_home_position_time_usec_SET(-3598625937298599980L, -3545207523218708595L, -1051230503405614581L, -4530050263661590017L, -1535458570014125760L, 1243242627342985535L, 3730339866872176395L, 6669608140144157638L, -1773252282346730324L, -6127046042050776871L, pset_home_position);
	
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
	pscaled_imu3_time_boot_ms_SET(1966951447, 1032196574, 187302577, -1235041686, 585114238, -249305711, 1628069446, -660031133, 1879619984, -2136105049, pscaled_imu3);
	pscaled_imu3_xacc_SET(15300, -18356, 5732, 13307, -17886, -24950, -14676, 12651, -25316, -3264, pscaled_imu3);
	pscaled_imu3_yacc_SET(-10198, -6906, 12070, -16324, -14383, 19897, -22409, -22368, -13780, 26374, pscaled_imu3);
	pscaled_imu3_zacc_SET(6701, 26887, -16583, -18561, 30149, -8413, -712, -10827, -2586, -13609, pscaled_imu3);
	pscaled_imu3_xgyro_SET(26075, 17616, 10733, -20142, 2392, -17832, 32213, -19579, 30279, 29507, pscaled_imu3);
	pscaled_imu3_ygyro_SET(14839, -12489, 14672, 18947, 12550, 25276, -1973, -20584, -29553, -483, pscaled_imu3);
	pscaled_imu3_zgyro_SET(-16338, 28312, 27409, 11648, -28242, 25306, -7564, -21470, -11767, 9123, pscaled_imu3);
	pscaled_imu3_xmag_SET(-7160, 12691, 3971, -7001, -16290, 5136, 614, -14787, 5290, 26576, pscaled_imu3);
	pscaled_imu3_ymag_SET(-14446, 1540, -11582, -29242, 14041, 28073, 2847, 28121, 11136, 25139, pscaled_imu3);
	pscaled_imu3_zmag_SET(4801, 26550, -14842, -662, -32676, 26800, -16524, -32299, 30330, 16660, pscaled_imu3);
	
}


void read_SET_MODE(pset_mode_SET_MODE *const pset_mode) {
	int32_t    custom_mode   = pset_mode_custom_mode_GET(pset_mode);
	int8_t     target_system = pset_mode_target_system_GET(pset_mode);
	e_MAV_MODE item_base_mode;
	if (pset_mode_base_mode_GET(pset_mode, &item_base_mode)) {
		e_MAV_MODE_PREFLIGHT = item_base_mode;
	}
	
}


void read_MOUNT_CONTROL(pmount_control_MOUNT_CONTROL *const pmount_control) {
	int8_t  target_system    = pmount_control_target_system_GET(pmount_control);
	int8_t  target_component = pmount_control_target_component_GET(pmount_control);
	int32_t input_a          = pmount_control_input_a_GET(pmount_control);
	int32_t input_b          = pmount_control_input_b_GET(pmount_control);
	int32_t input_c          = pmount_control_input_c_GET(pmount_control);
	int8_t  save_position    = pmount_control_save_position_GET(pmount_control);
	
}


void write_MOUNT_CONTROL(pmount_control_MOUNT_CONTROL *const pmount_control) {
	pmount_control_target_system_SET(-66, -11, 114, -84, -53, 102, 75, 64, 75, 79, pmount_control);
	pmount_control_target_component_SET(-48, 92, -95, 41, -44, 73, 71, 112, -86, 84, pmount_control);
	pmount_control_input_a_SET(-240294805, -781308708, -1429599027, -1032223644, -472094116, 1320742438, 980486741, -771088223, 1107597725, -276669951, pmount_control);
	pmount_control_input_b_SET(-306604338, -1136562070, 1940947578, -509653462, 1696784592, 294675837, 363214631, 373132973, 261682387, -1019308306, pmount_control);
	pmount_control_input_c_SET(-499439865, -1645883456, -288321850, -1683827756, -923150768, -1205431329, 543759765, 530235547, 2064579582, 275975008, pmount_control);
	pmount_control_save_position_SET(-88, -120, -57, 4, 44, -93, -18, 109, 64, 106, pmount_control);
	
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


void write_LED_CONTROL(pled_control_LED_CONTROL *const pled_control) {
	pled_control_target_system_SET(-14, 17, 17, 100, 11, 13, 46, -15, -45, 95, pled_control);
	pled_control_target_component_SET(88, -68, -117, -107, 84, 64, 94, 46, 92, 118, pled_control);
	pled_control_instance_SET(-117, -65, 32, -23, -109, 103, -43, 8, 52, -63, pled_control);
	pled_control_pattern_SET(-121, -60, 66, 24, 83, -3, 103, 47, 37, 42, pled_control);
	pled_control_custom_len_SET(66, -11, 93, -110, -77, -54, 68, 117, 114, 60, pled_control);
	pled_control_custom_bytes_SET(&-1, 30, 74, -80, 76, 94, -18, 20, 31, -30, pled_control);
	
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
	psim_state_q1_SET(-1.0185062E38F, -1.3576717E38F, -1.0276335E38F, -6.141799E37F, -4.152883E37F, 3.0836843E37F, -4.018641E37F, 2.7623528E38F, 3.1569416E38F, -1.0328004E38F, psim_state);
	psim_state_q2_SET(1.8278487E38F, 9.524025E37F, -1.3473263E38F, 2.4590416E38F, -1.7100132E38F, 4.1855503E37F, 3.0423339E38F, -3.2246565E38F, 1.6854125E38F, -7.00462E37F, psim_state);
	psim_state_q3_SET(-2.346384E38F, -3.1449777E38F, -2.219545E38F, 2.3332132E38F, 8.431369E37F, 6.78879E37F, -2.9384709E38F, 2.2659706E37F, -6.921742E37F, -4.3793023E37F, psim_state);
	psim_state_q4_SET(2.7832819E38F, -2.7375363E38F, -4.648874E37F, 8.90939E37F, -1.3687416E37F, -3.1914712E38F, 2.7469826E38F, -9.423324E37F, 1.5976444E37F, 1.2712529E38F, psim_state);
	psim_state_roll_SET(6.7887334E37F, 3.2032178E38F, -1.8454874E37F, -2.2514951E38F, 2.9581231E38F, 3.9533185E37F, 1.9062435E38F, 9.690195E37F, -1.7942235E38F, 3.3560798E38F, psim_state);
	psim_state_pitch_SET(-1.6911708E38F, 2.3645797E38F, -3.0593289E38F, 1.4877939E38F, 2.6197436E38F, -1.7350976E37F, 7.500669E37F, -2.6284206E38F, -2.675512E37F, -1.9125241E37F, psim_state);
	psim_state_yaw_SET(2.8957762E38F, -1.2486545E38F, -2.6022125E38F, -6.13993E37F, 7.3245465E37F, 2.9071642E38F, 5.583193E37F, 1.9305189E38F, 2.8392838E38F, 9.832223E37F, psim_state);
	psim_state_xacc_SET(-7.0758066E37F, -2.4395684E38F, 2.5039645E38F, -2.2064414E38F, 1.6798405E38F, 1.3982125E38F, -1.2617687E38F, 1.603052E38F, 3.0927164E38F, 1.9736904E38F, psim_state);
	psim_state_yacc_SET(2.8233216E38F, 2.3510284E38F, -7.298524E37F, -3.0822823E38F, 1.2512881E38F, -2.0467825E38F, 1.8599373E38F, -2.7347065E38F, -2.3183891E38F, 2.9426937E38F, psim_state);
	psim_state_zacc_SET(1.0515606E38F, -1.1200837E38F, -2.8817292E38F, -2.0073215E38F, -2.875815E38F, 2.8144342E38F, 3.3523257E38F, -3.0187247E38F, -8.343463E37F, -2.3072841E38F, psim_state);
	psim_state_xgyro_SET(8.551445E37F, 1.1021612E37F, 2.327256E38F, 1.5862127E38F, -1.3083044E38F, 3.3526823E38F, -2.586442E38F, 2.327622E38F, 1.7870746E38F, -2.835265E38F, psim_state);
	psim_state_ygyro_SET(2.2772063E37F, -6.221919E37F, -5.3880586E37F, -1.2407134E38F, 2.2094016E38F, -1.3699654E38F, -2.7252894E38F, -7.834249E36F, 2.6775566E36F, -2.7021129E38F, psim_state);
	psim_state_zgyro_SET(-1.943203E38F, 1.9295238E38F, -2.7345801E38F, -3.1162586E38F, -3.0921976E38F, -1.4733196E38F, -1.1571698E37F, -6.077427E37F, 7.0326725E37F, 1.159775E38F, psim_state);
	psim_state_lat_SET(-1.6501476E38F, 1.5057235E38F, 1.7212217E38F, -3.2385858E38F, -2.9948E38F, -2.272783E38F, 2.355034E38F, 1.9042767E38F, -6.4166445E37F, -8.667047E37F, psim_state);
	psim_state_lon_SET(6.667657E37F, 1.6729578E38F, 3.2727802E38F, 3.2597905E38F, 1.95923E38F, -3.2844018E37F, 1.9475044E38F, 2.7878239E38F, 1.7695985E38F, -3.0454194E38F, psim_state);
	psim_state_alt_SET(-1.9986982E38F, -9.357164E37F, -1.3143922E38F, -1.3488205E38F, 1.3916224E37F, -1.2209658E38F, 8.4975595E36F, 9.450307E37F, 5.810582E37F, -2.1049036E38F, psim_state);
	psim_state_std_dev_horz_SET(4.069791E37F, 2.728951E38F, 1.9716066E38F, -2.3407769E38F, 8.538365E37F, -1.767064E38F, -5.339518E37F, 2.6261453E38F, -8.975727E37F, -7.556817E37F, psim_state);
	psim_state_std_dev_vert_SET(1.0560988E38F, 4.5405145E37F, 3.0568386E38F, -2.4059292E38F, 3.3925955E38F, -1.7007507E38F, 1.941065E38F, 2.32782E38F, -2.8817842E38F, -1.4851789E38F, psim_state);
	psim_state_vn_SET(1.5958607E38F, 3.0041928E38F, 8.65625E37F, 1.6420867E38F, 1.4904391E38F, 2.9976718E38F, 3.0066881E38F, -6.9947454E37F, -1.444522E38F, -1.6890923E38F, psim_state);
	psim_state_ve_SET(9.269951E37F, -1.0999551E37F, -1.4359599E38F, -1.0497404E38F, -2.3377976E38F, 2.2608534E38F, 3.1704467E38F, -1.5184952E37F, -3.3414446E38F, -2.8626555E37F, psim_state);
	psim_state_vd_SET(-1.7049367E38F, 1.8810795E38F, 4.422636E37F, 3.3895405E38F, -9.197885E37F, -2.1481274E38F, 5.1173184E37F, 3.4633485E37F, 2.9113547E38F, 3.3386745E38F, psim_state);
	
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


void write_WIFI_CONFIG_AP(pwifi_config_ap_WIFI_CONFIG_AP *const pwifi_config_ap) {
	pwifi_config_ap_ssid_SET(some_string, strlen(some_string), pwifi_config_ap);
	pwifi_config_ap_password_SET(some_string, strlen(some_string), pwifi_config_ap);
	
}


void read_DATA96(pdata96_DATA96 *const pdata96) {
	int8_t       typE      = pdata96_typE_GET(pdata96);
	int8_t       len       = pdata96_len_GET(pdata96);
	Vdata96_daTa item_daTa = pdata96_daTa_GET(pdata96);
	for (size_t  index     = 0; index < item_daTa.len; index++)
		some_int8_t = vdata96_daTa_GET(&item_daTa, index);
	
}


void write_DATA96(pdata96_DATA96 *const pdata96) {
	pdata96_typE_SET(-78, -56, 46, -21, 30, 117, 29, 124, 119, -127, pdata96);
	pdata96_len_SET(77, 123, 87, -80, 91, -5, 60, 116, -71, -31, pdata96);
	pdata96_daTa_SET(&-18, -20, -97, 56, -95, -25, -36, -127, -95, -20, pdata96);
	
}


void read_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION *const pflight_information) {
	int32_t time_boot_ms     = pflight_information_time_boot_ms_GET(pflight_information);
	int64_t arming_time_utc  = pflight_information_arming_time_utc_GET(pflight_information);
	int64_t takeoff_time_utc = pflight_information_takeoff_time_utc_GET(pflight_information);
	int64_t flight_uuid      = pflight_information_flight_uuid_GET(pflight_information);
	
}


void write_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION *const pflight_information) {
	pflight_information_time_boot_ms_SET(266661279, -742552665, -878787782, 504994385, -532783168, -1317337971, -659151591, 1418710595, 1494427223, -1534806582, pflight_information);
	pflight_information_arming_time_utc_SET(-4998771794599563467L, 4499753115088935808L, 3496723917262816125L, 2188287378438847300L, 4625473451954484292L, -858714266007533105L, -1666608913600271632L, -757511904678138073L, 6015750998187490295L, 3729059930555710883L, pflight_information);
	pflight_information_takeoff_time_utc_SET(1374165998894140976L, 6596355323054221811L, 7449703289108675921L, -1117422125096792542L, -4584540991199631047L, -3235909995333007780L, 7119492851705094141L, 6167153617193488791L, -7335418437297871811L, -3261687896663485432L, pflight_information);
	pflight_information_flight_uuid_SET(1861563228796097046L, 8380008720377346625L, 2202023628203179885L, 6185322263834357335L, -8958478658270982880L, -525921481898070804L, 4658438231092272400L, 1812249492866340163L, -362915252956138464L, 8335721027080685338L, pflight_information);
	
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


void read_MEMINFO(pmeminfo_MEMINFO *const pmeminfo) {
	int16_t brkval  = pmeminfo_brkval_GET(pmeminfo);
	int16_t freemem = pmeminfo_freemem_GET(pmeminfo);
	int32_t item_freemem32;
	if (pmeminfo_freemem32_GET(pmeminfo, &item_freemem32)) {
		some_int32_t = item_freemem32;
	}
	
}


void write_MEMINFO(pmeminfo_MEMINFO *const pmeminfo) {
	pmeminfo_brkval_SET(-32204, -17495, -27111, -31098, -22579, 17713, 21036, 22614, -3937, 6462, pmeminfo);
	pmeminfo_freemem_SET(-5586, -23642, -3759, -808, 11453, -4126, 10407, -12040, 13897, -30569, pmeminfo);
	pmeminfo_freemem32_SET(382129566, -685923100, 145630072, -173865277, 753745071, 283607302, 2104281346, 2027967165, 49062395, 1744295642, pmeminfo);
	
}


void read_MISSION_ITEM_REACHED(pmission_item_reached_MISSION_ITEM_REACHED *const pmission_item_reached) {
	int16_t seq = pmission_item_reached_seq_GET(pmission_item_reached);
	
}


void read_LOGGING_ACK(plogging_ack_LOGGING_ACK *const plogging_ack) {
	int16_t sequence         = plogging_ack_sequence_GET(plogging_ack);
	int8_t  target_system    = plogging_ack_target_system_GET(plogging_ack);
	int8_t  target_component = plogging_ack_target_component_GET(plogging_ack);
	
}


void write_LOGGING_ACK(plogging_ack_LOGGING_ACK *const plogging_ack) {
	plogging_ack_sequence_SET(21022, 20509, 23809, -71, 4088, 23258, 6537, -10391, -27396, -29970, plogging_ack);
	plogging_ack_target_system_SET(97, 91, 1, -30, 4, 82, 43, -17, -119, -59, plogging_ack);
	plogging_ack_target_component_SET(-46, -92, 75, 8, -71, -105, -111, -35, 78, 82, plogging_ack);
	
}


void read_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE *const pvision_speed_estimate) {
	int64_t usec = pvision_speed_estimate_usec_GET(pvision_speed_estimate);
	float   x    = pvision_speed_estimate_x_GET(pvision_speed_estimate);
	float   y    = pvision_speed_estimate_y_GET(pvision_speed_estimate);
	float   z    = pvision_speed_estimate_z_GET(pvision_speed_estimate);
	
}


void write_VISION_SPEED_ESTIMATE(pvision_speed_estimate_VISION_SPEED_ESTIMATE *const pvision_speed_estimate) {
	pvision_speed_estimate_usec_SET(-6957237508062374029L, 1129889029094021023L, -6492076728963760715L, -5308004817364719736L, -2053511732616341113L, -1165827843832334912L, 7275894213113433341L, 8240848611590799423L, -2322328358259258132L, 6348994659686104138L, pvision_speed_estimate);
	pvision_speed_estimate_x_SET(2.9134477E38F, -5.9037546E37F, 3.1480679E38F, -7.2859897E37F, 2.2825438E38F, -2.0661226E38F, -2.3495673E38F, 2.2486016E38F, 1.4500182E38F, -3.30857E37F, pvision_speed_estimate);
	pvision_speed_estimate_y_SET(7.6186866E36F, -8.351215E36F, 2.6320728E38F, -1.2704658E38F, 1.103288E38F, -1.73821E38F, -3.1505799E38F, -4.2758676E37F, -6.7812893E37F, -1.5014545E38F, pvision_speed_estimate);
	pvision_speed_estimate_z_SET(-2.1657788E38F, -3.0701055E38F, 2.3455236E38F, 5.181203E37F, -2.7051911E38F, -1.6117531E38F, 9.312448E37F, 2.2123815E38F, 3.3128395E38F, -9.610451E37F, pvision_speed_estimate);
	
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
	pdebug_vect_time_usec_SET(6194002345795363944L, 3539221305192473146L, 8759824657058794544L, 3004894458316620815L, 7469146448313029992L, 884086220428541122L, -554480415203852547L, -6506920596565455251L, -4148029672080290466L, 7581376483206317801L, pdebug_vect);
	pdebug_vect_x_SET(1.55763E38F, -3.298574E38F, -1.0423415E38F, -2.7932186E38F, -3.0358928E38F, -3.051563E38F, -1.5962016E38F, 1.2267709E38F, 2.9688627E38F, -1.6381231E38F, pdebug_vect);
	pdebug_vect_y_SET(-2.6172687E38F, -1.2891241E38F, 1.2855059E38F, 4.7582797E37F, -4.1441762E37F, 5.3239687E37F, -2.667907E38F, 3.0605093E38F, -1.0282614E37F, -3.3965854E38F, pdebug_vect);
	pdebug_vect_z_SET(2.8185704E38F, -7.46124E37F, 8.280685E37F, 6.2129853E37F, -3.1270703E38F, -3.990454E37F, -2.6376592E37F, -1.7257375E38F, 4.899517E37F, -7.355326E37F, pdebug_vect);
	pdebug_vect_name_SET(some_string, strlen(some_string), pdebug_vect);
	
}


void read_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END *const plog_request_end) {
	int8_t target_system    = plog_request_end_target_system_GET(plog_request_end);
	int8_t target_component = plog_request_end_target_component_GET(plog_request_end);
	
}


void write_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END *const plog_request_end) {
	plog_request_end_target_system_SET(-85, 126, 17, -57, -41, 7, 105, 49, 111, 51, plog_request_end);
	plog_request_end_target_component_SET(29, -70, -6, -48, -86, 13, 52, -25, 57, 43, plog_request_end);
	
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


void read_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack_CHANGE_OPERATOR_CONTROL_ACK *const pchange_operator_control_ack) {
	int8_t gcs_system_id   = pchange_operator_control_ack_gcs_system_id_GET(pchange_operator_control_ack);
	int8_t control_request = pchange_operator_control_ack_control_request_GET(pchange_operator_control_ack);
	int8_t ack             = pchange_operator_control_ack_ack_GET(pchange_operator_control_ack);
	
}


void read_MISSION_CURRENT(pmission_current_MISSION_CURRENT *const pmission_current) {
	int16_t seq = pmission_current_seq_GET(pmission_current);
	
}


void read_SYSTEM_TIME(psystem_time_SYSTEM_TIME *const psystem_time) {
	int32_t time_boot_ms   = psystem_time_time_boot_ms_GET(psystem_time);
	int64_t time_unix_usec = psystem_time_time_unix_usec_GET(psystem_time);
	
}


void read_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER *const pcamera_trigger) {
	int32_t seq       = pcamera_trigger_seq_GET(pcamera_trigger);
	int64_t time_usec = pcamera_trigger_time_usec_GET(pcamera_trigger);
	
}


void write_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER *const pcamera_trigger) {
	pcamera_trigger_seq_SET(-1603053573, 1121483740, -636055792, 93682386, 894963310, 490757121, -1545526684, -288998311, 1040108949, -403466924, pcamera_trigger);
	pcamera_trigger_time_usec_SET(8054496038720670071L, 6806010353791376430L, -4813974355448498447L, -7586391528120602220L, 4280266805886919333L, 2169146920066568359L, 8019071752031354153L, 4843475842521677948L, -6647758439655685016L, -2509541972181522004L, pcamera_trigger);
	
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


void write_GOPRO_SET_RESPONSE(pgopro_set_response_GOPRO_SET_RESPONSE *const pgopro_set_response) {
	pgopro_set_response_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_BATTERY, e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_BURST_RATE, e_GOPRO_COMMAND_GOPRO_COMMAND_VIDEO_SETTINGS, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_WHITE_BALANCE, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_GAIN, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_COLOUR, e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_RESOLUTION, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE, e_GOPRO_COMMAND_GOPRO_COMMAND_CHARGING, e_GOPRO_COMMAND_GOPRO_COMMAND_POWER, pgopro_set_response);
	pgopro_set_response_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, pgopro_set_response);
	
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


void read_MANUAL_CONTROL(pmanual_control_MANUAL_CONTROL *const pmanual_control) {
	int16_t buttons = pmanual_control_buttons_GET(pmanual_control);
	int8_t  target  = pmanual_control_target_GET(pmanual_control);
	int16_t x       = pmanual_control_x_GET(pmanual_control);
	int16_t y       = pmanual_control_y_GET(pmanual_control);
	int16_t z       = pmanual_control_z_GET(pmanual_control);
	int16_t r       = pmanual_control_r_GET(pmanual_control);
	
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


void write_PROTOCOL_VERSION(pprotocol_version_PROTOCOL_VERSION *const pprotocol_version) {
	pprotocol_version_version_SET(-27014, -11541, 4503, -12428, 20123, -25836, 14964, -1053, -26903, 21200, pprotocol_version);
	pprotocol_version_min_version_SET(-2259, -24390, 3180, -2717, 11423, 28584, -19929, 28076, 9284, 25788, pprotocol_version);
	pprotocol_version_max_version_SET(-1795, 11783, 28590, 6238, 2337, 18925, -13724, -6876, 23181, -5443, pprotocol_version);
	pprotocol_version_spec_version_hash_SET(&7, -66, 85, -3, -97, 19, 20, 85, -110, 60, pprotocol_version);
	pprotocol_version_library_version_hash_SET(&39, 91, -79, -24, 121, -66, -43, 115, -121, 38, pprotocol_version);
	
}


void read_RALLY_FETCH_POINT(prally_fetch_point_RALLY_FETCH_POINT *const prally_fetch_point) {
	int8_t target_system    = prally_fetch_point_target_system_GET(prally_fetch_point);
	int8_t target_component = prally_fetch_point_target_component_GET(prally_fetch_point);
	int8_t idx              = prally_fetch_point_idx_GET(prally_fetch_point);
	
}


void write_RALLY_FETCH_POINT(prally_fetch_point_RALLY_FETCH_POINT *const prally_fetch_point) {
	prally_fetch_point_target_system_SET(-86, 109, 71, -71, 106, 73, 97, 61, -116, -6, prally_fetch_point);
	prally_fetch_point_target_component_SET(17, 29, 102, 41, 55, -21, -104, 27, 100, 37, prally_fetch_point);
	prally_fetch_point_idx_SET(-107, -17, -101, -113, -120, 8, -127, -6, -100, -112, prally_fetch_point);
	
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
	pbattery_status_voltages_SET(&-6720, -4864, 21465, -5123, 11743, 22476, -8641, -27192, 1997, -22102, pbattery_status);
	pbattery_status_id_SET(52, -20, -87, -113, 89, 53, -110, 57, 53, -27, pbattery_status);
	pbattery_status_temperature_SET(26345, 6649, -22828, -29009, 5106, -3558, -26692, -28984, -16413, -21756, pbattery_status);
	pbattery_status_current_battery_SET(-14628, -14315, 32664, -13705, 2657, -17712, 5121, -13569, -28113, -13002, pbattery_status);
	pbattery_status_current_consumed_SET(-480807175, -1376143915, -1597717651, -1668424261, -1206675305, -1242382029, 560258742, -528559627, -1713334148, 2008507123, pbattery_status);
	pbattery_status_energy_consumed_SET(-753812831, 887945488, -511880309, -1569035233, 1126540448, -2363373, 1574146495, -1247351297, -49644809, -203306021, pbattery_status);
	pbattery_status_battery_remaining_SET(111, 1, 84, 117, 39, -24, 13, 115, 17, 106, pbattery_status);
	pbattery_status_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, pbattery_status);
	pbattery_status_typE_SET(e_MAV_BATTERY_TYPE_LIPO, e_MAV_BATTERY_TYPE_NIMH, e_MAV_BATTERY_TYPE_UNKNOWN, e_MAV_BATTERY_TYPE_LION, e_MAV_BATTERY_TYPE_NIMH, e_MAV_BATTERY_TYPE_NIMH, e_MAV_BATTERY_TYPE_NIMH, e_MAV_BATTERY_TYPE_NIMH, e_MAV_BATTERY_TYPE_UNKNOWN, e_MAV_BATTERY_TYPE_LIPO, pbattery_status);
	
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
	pserial_control_timeout_SET(-8414, 12488, 3600, -26449, 8935, -9804, -6090, 14756, 25034, 7161, pserial_control);
	pserial_control_baudrate_SET(-1276419700, 1306355120, 58186108, 195436515, 1583946175, 1357219472, 551309796, -1187456476, -552341689, -506274991, pserial_control);
	pserial_control_count_SET(40, 38, 93, 48, 57, -106, -70, -100, -61, 21, pserial_control);
	pserial_control_daTa_SET(&15, -128, 110, -68, -59, 68, 92, 34, 23, 89, pserial_control);
	pserial_control_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, pserial_control);
	pserial_control_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, pserial_control);
	
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


void read_MOUNT_ORIENTATION(pmount_orientation_MOUNT_ORIENTATION *const pmount_orientation) {
	int32_t time_boot_ms = pmount_orientation_time_boot_ms_GET(pmount_orientation);
	float   roll         = pmount_orientation_roll_GET(pmount_orientation);
	float   pitch        = pmount_orientation_pitch_GET(pmount_orientation);
	float   yaw          = pmount_orientation_yaw_GET(pmount_orientation);
	
}


void write_MOUNT_ORIENTATION(pmount_orientation_MOUNT_ORIENTATION *const pmount_orientation) {
	pmount_orientation_time_boot_ms_SET(-2022574013, -704794349, -488702234, -278018236, -2045421946, 1729223044, -1158038199, -1828627248, -1286985439, 1881261589, pmount_orientation);
	pmount_orientation_roll_SET(2.7117056E38F, 1.4120673E38F, -1.9022375E38F, 1.4040731E38F, 2.609187E38F, 2.8369759E38F, 3.1409654E38F, 3.1652777E38F, 1.4529212E38F, -1.5676822E38F, pmount_orientation);
	pmount_orientation_pitch_SET(2.2681928E38F, -3.1085302E38F, -3.02816E37F, 2.4323067E38F, 2.4183498E38F, 2.4396447E38F, -2.896556E38F, 3.0052765E38F, -1.910632E38F, -2.9029546E38F, pmount_orientation);
	pmount_orientation_yaw_SET(-2.1034528E38F, 4.200945E37F, 2.7543776E38F, 1.8135886E38F, 1.1137957E36F, -1.6054309E38F, 2.0546752E37F, -8.930868E37F, 1.4148032E38F, -2.2764524E38F, pmount_orientation);
	
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


void write_PARAM_EXT_SET(pparam_ext_set_PARAM_EXT_SET *const pparam_ext_set) {
	pparam_ext_set_target_system_SET(79, 97, 126, -123, -75, -101, 109, 43, 111, -81, pparam_ext_set);
	pparam_ext_set_target_component_SET(-126, 126, 3, -116, 3, 15, -19, 62, 9, 8, pparam_ext_set);
	pparam_ext_set_param_id_SET(some_string, strlen(some_string), pparam_ext_set);
	pparam_ext_set_param_value_SET(some_string, strlen(some_string), pparam_ext_set);
	pparam_ext_set_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, pparam_ext_set);
	
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
	pautopilot_version_vendor_id_SET(-18467, 6322, 31567, -21741, -24505, -13917, -30206, -21701, 11222, 195, pautopilot_version);
	pautopilot_version_product_id_SET(23462, -7522, -10419, -24636, -16588, 3111, 15846, 14439, 29229, -5447, pautopilot_version);
	pautopilot_version_flight_sw_version_SET(-1822946857, 89449675, -1866088387, 80648880, 1013268763, 702390572, -1701090334, 231321358, -124687911, 1896151903, pautopilot_version);
	pautopilot_version_middleware_sw_version_SET(-279540099, 355117937, -1432155427, -683253074, 454048993, -2102435842, 1459297046, -401403564, -1244876296, -279004146, pautopilot_version);
	pautopilot_version_os_sw_version_SET(-804681651, 1458704656, 1567652579, -2045212120, 782334733, 869608419, -1204962827, 1752713697, 685511416, -1305453754, pautopilot_version);
	pautopilot_version_board_version_SET(245892727, -605036589, -1510386917, 241264099, -1926052940, 1659260793, -268617702, -1524874656, -1681608555, 1629287110, pautopilot_version);
	pautopilot_version_uid_SET(-8617216227315253588L, 6581939596115468231L, 2814848976754365559L, 6589684876388254676L, 2122740258989251751L, -3591983070526942251L, -1797353077390497955L, -8467642652373167571L, 3116818477177846664L, -4073979823567794752L, pautopilot_version);
	pautopilot_version_flight_custom_version_SET(&-56, 82, -43, -95, 93, 40, 73, 8, -122, -70, pautopilot_version);
	pautopilot_version_middleware_custom_version_SET(&-123, 80, -59, 36, -88, 6, 77, -20, 47, -9, pautopilot_version);
	pautopilot_version_os_custom_version_SET(&-51, 64, 79, 1, -118, -83, 2, -21, 41, 85, pautopilot_version);
	pautopilot_version_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION, e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT, pautopilot_version);
	for (size_t d0 = 0; d0 < Pautopilot_version_uid2_D0; d0++) {
		pautopilot_version_uid2_SET(-115, -107, 118, 103, 42, 104, 3, 82, -114, 22, pautopilot_version, d0);
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


void write_SIMSTATE(psimstate_SIMSTATE *const psimstate) {
	psimstate_roll_SET(-1.1811806E38F, -2.911383E38F, -1.8692674E38F, 1.6875472E38F, -1.2315456E38F, -2.0520998E38F, 2.2386555E38F, 3.0751338E38F, 3.5667074E36F, 1.6478798E38F, psimstate);
	psimstate_pitch_SET(2.1099567E36F, -1.3285095E38F, 9.455631E37F, 6.74515E37F, -7.7266336E37F, 2.8795132E38F, 1.6490265E38F, -7.835138E37F, 3.1939375E37F, 2.0175678E37F, psimstate);
	psimstate_yaw_SET(-4.306846E36F, -1.4710719E38F, 2.7168872E38F, -2.3133287E38F, 1.8696607E38F, -9.106572E37F, -4.4663174E37F, -2.5440624E38F, -6.054359E37F, -1.3980137E38F, psimstate);
	psimstate_xacc_SET(-9.339892E37F, -1.6412745E38F, 3.0831415E37F, 1.302314E37F, -2.3974045E38F, 3.3670136E37F, 5.2167945E37F, 2.586669E38F, -1.8200994E38F, 1.3722188E38F, psimstate);
	psimstate_yacc_SET(-2.9339457E38F, 5.156573E37F, -3.3491954E38F, 3.2282238E38F, -1.013665E38F, -2.8553013E37F, 2.9998714E38F, 2.82091E37F, 6.5960536E37F, -2.2236394E37F, psimstate);
	psimstate_zacc_SET(3.0014577E37F, -5.5785586E37F, -1.4853105E38F, -3.2549152E38F, -2.340061E38F, -4.298826E37F, 2.6075747E38F, -2.1015288E38F, 1.3084596E37F, 1.6933835E38F, psimstate);
	psimstate_xgyro_SET(-1.0601418E38F, 2.298995E38F, 1.451137E38F, 2.6766588E38F, -2.892931E38F, -7.318202E36F, -1.8955585E38F, -9.068612E37F, 1.4432661E38F, 3.102779E38F, psimstate);
	psimstate_ygyro_SET(-1.8536218E38F, -3.035677E38F, 1.804305E38F, 2.5648652E38F, -2.413844E38F, 1.1573618E37F, 1.1550563E38F, 2.14103E38F, 3.172361E38F, -2.1023614E38F, psimstate);
	psimstate_zgyro_SET(1.3071653E38F, 3.1697727E38F, 3.313388E38F, -2.0515927E38F, -1.8173656E38F, -2.6444727E38F, 1.8249356E38F, 2.4930071E38F, 1.8123864E38F, 7.443615E37F, psimstate);
	psimstate_lat_SET(-626022471, 1118221842, -334570945, -1459286432, 918496462, -1694469907, -1676670122, 112692808, 1128077902, -776820800, psimstate);
	psimstate_lng_SET(-141486086, -1585856531, 386097908, -1067018360, 282301544, -1111428434, 2017065160, 1030064904, 1894772611, 185312585, psimstate);
	
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


void write_SET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *const pset_video_stream_settings) {
	pset_video_stream_settings_resolution_h_SET(-31414, -20324, 27289, 29675, -29129, -6466, -31582, -12749, -812, -22607, pset_video_stream_settings);
	pset_video_stream_settings_resolution_v_SET(25158, 11119, -22115, 10265, -7397, -15011, 24456, 13469, -28912, -518, pset_video_stream_settings);
	pset_video_stream_settings_rotation_SET(-7516, -8688, 172, -16436, -20605, -3962, 6678, 13138, 30900, -11289, pset_video_stream_settings);
	pset_video_stream_settings_bitrate_SET(1114855636, -29898605, -1706296962, -1594108064, 986710780, -141168071, 620323050, 558966008, -1689804444, -696779661, pset_video_stream_settings);
	pset_video_stream_settings_target_system_SET(39, -34, -2, 111, 36, 41, 17, -8, 80, 20, pset_video_stream_settings);
	pset_video_stream_settings_target_component_SET(96, -64, 76, 36, 62, -51, -79, -74, -7, -78, pset_video_stream_settings);
	pset_video_stream_settings_camera_id_SET(-41, 121, -4, 84, -97, -127, 61, 66, 119, 110, pset_video_stream_settings);
	pset_video_stream_settings_framerate_SET(2.3096551E38F, -2.3135719E38F, -1.6698561E38F, -1.6592803E38F, 1.7254678E38F, -3.1414714E38F, 2.2859302E38F, -1.772219E38F, 1.3106542E38F, 9.052524E37F, pset_video_stream_settings);
	pset_video_stream_settings_uri_SET(some_string, strlen(some_string), pset_video_stream_settings);
	
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
	pplay_tune_target_system_SET(113, -56, 24, 98, -43, 18, -117, 92, -53, -19, pplay_tune);
	pplay_tune_target_component_SET(-93, 50, 0, 118, 7, 59, 57, 108, 31, -52, pplay_tune);
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


void write_DIGICAM_CONFIGURE(pdigicam_configure_DIGICAM_CONFIGURE *const pdigicam_configure) {
	pdigicam_configure_shutter_speed_SET(-26744, -25198, -16055, -10161, -10588, -23, 19963, -9755, 18711, 23779, pdigicam_configure);
	pdigicam_configure_target_system_SET(9, -65, 117, 56, 106, 29, 59, -85, 28, -39, pdigicam_configure);
	pdigicam_configure_target_component_SET(30, 61, -6, -125, 102, 60, -111, 84, -117, 75, pdigicam_configure);
	pdigicam_configure_mode_SET(-128, -71, -113, -48, -101, -60, 62, 77, 14, 3, pdigicam_configure);
	pdigicam_configure_aperture_SET(91, -91, -9, -41, 104, -49, -99, 104, 103, -66, pdigicam_configure);
	pdigicam_configure_iso_SET(30, -11, -1, -44, 27, 124, 19, -92, -57, -1, pdigicam_configure);
	pdigicam_configure_exposure_type_SET(-123, -46, -62, -33, -121, -70, -31, -17, -37, -4, pdigicam_configure);
	pdigicam_configure_command_id_SET(-56, -102, 65, -41, -114, 73, -36, 103, 77, 123, pdigicam_configure);
	pdigicam_configure_engine_cut_off_SET(-37, -58, 109, -111, -11, 106, 82, -23, -94, -90, pdigicam_configure);
	pdigicam_configure_extra_param_SET(-4, -92, 85, -29, -124, 54, 21, -41, -66, -70, pdigicam_configure);
	pdigicam_configure_extra_value_SET(1.1256814E38F, 9.46764E37F, 1.6449036E38F, 2.069913E38F, -1.461287E38F, 8.595293E37F, -2.890823E37F, 2.8311646E38F, -1.4316551E38F, 4.9708566E37F, pdigicam_configure);
	
}


void read_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3 *const pscaled_pressure3) {
	int32_t time_boot_ms = pscaled_pressure3_time_boot_ms_GET(pscaled_pressure3);
	float   press_abs    = pscaled_pressure3_press_abs_GET(pscaled_pressure3);
	float   press_diff   = pscaled_pressure3_press_diff_GET(pscaled_pressure3);
	int16_t temperature  = pscaled_pressure3_temperature_GET(pscaled_pressure3);
	
}


void write_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3 *const pscaled_pressure3) {
	pscaled_pressure3_time_boot_ms_SET(542772664, -893909709, -1431609819, 1606125075, 885044451, 1772913543, 1274777123, -526013288, -977180134, 736024010, pscaled_pressure3);
	pscaled_pressure3_press_abs_SET(-1.1938784E38F, -5.745818E37F, -2.227291E38F, 1.9702081E38F, -5.1258846E37F, 2.9889946E38F, 3.285449E38F, 2.4336342E38F, 5.3026824E37F, -6.101769E36F, pscaled_pressure3);
	pscaled_pressure3_press_diff_SET(-1.1042863E38F, 2.8100275E38F, 1.7095696E38F, 3.2383352E38F, -4.309858E37F, 6.0339844E37F, -1.9310241E38F, -1.4438429E37F, -4.671986E37F, 1.8531133E38F, pscaled_pressure3);
	pscaled_pressure3_temperature_SET(29782, 14920, -20489, -20242, 18801, -7559, -31812, 11836, 15290, -27165, pscaled_pressure3);
	
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


void write_PARAM_EXT_ACK(pparam_ext_ack_PARAM_EXT_ACK *const pparam_ext_ack) {
	pparam_ext_ack_param_id_SET(some_string, strlen(some_string), pparam_ext_ack);
	pparam_ext_ack_param_value_SET(some_string, strlen(some_string), pparam_ext_ack);
	pparam_ext_ack_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, pparam_ext_ack);
	pparam_ext_ack_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, e_PARAM_ACK_PARAM_ACK_FAILED, e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, e_PARAM_ACK_PARAM_ACK_FAILED, e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, e_PARAM_ACK_PARAM_ACK_ACCEPTED, e_PARAM_ACK_PARAM_ACK_ACCEPTED, e_PARAM_ACK_PARAM_ACK_ACCEPTED, e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, e_PARAM_ACK_PARAM_ACK_FAILED, pparam_ext_ack);
	
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


void write_UAVCAN_NODE_INFO(puavcan_node_info_UAVCAN_NODE_INFO *const puavcan_node_info) {
	puavcan_node_info_uptime_sec_SET(2142336722, 1858308858, 937011223, -315688226, -303609647, 1583755058, -1515599113, 197398425, -487125501, 465703875, puavcan_node_info);
	puavcan_node_info_sw_vcs_commit_SET(13010314, -425984025, 973704496, -913293963, -1183512689, -2103668505, 125559723, 36982091, 806503322, 2020659806, puavcan_node_info);
	puavcan_node_info_time_usec_SET(-1860808037130956617L, 2976108376383784797L, -1859992223714777865L, 8140208670550121591L, -7592430725017988116L, 8765539791784810761L, 3410862828232109255L, 2664748804309073648L, -799406243715901942L, -293783048765502402L, puavcan_node_info);
	puavcan_node_info_hw_version_major_SET(71, 107, -83, -29, -123, -32, 82, -83, -40, 46, puavcan_node_info);
	puavcan_node_info_hw_version_minor_SET(-115, 120, 62, 22, -75, -5, 20, 120, 19, -13, puavcan_node_info);
	puavcan_node_info_hw_unique_id_SET(&46, -91, 45, -70, 73, -71, -56, -103, 43, -16, puavcan_node_info);
	puavcan_node_info_sw_version_major_SET(46, -20, 21, -69, 0, -83, 42, -69, 9, 55, puavcan_node_info);
	puavcan_node_info_sw_version_minor_SET(4, -46, -24, 98, -123, -32, -98, 14, 39, 9, puavcan_node_info);
	puavcan_node_info_name_SET(some_string, strlen(some_string), puavcan_node_info);
	
}


void read_DATA16(pdata16_DATA16 *const pdata16) {
	int8_t       typE      = pdata16_typE_GET(pdata16);
	int8_t       len       = pdata16_len_GET(pdata16);
	Vdata16_daTa item_daTa = pdata16_daTa_GET(pdata16);
	for (size_t  index     = 0; index < item_daTa.len; index++)
		some_int8_t = vdata16_daTa_GET(&item_daTa, index);
	
}


void write_DATA16(pdata16_DATA16 *const pdata16) {
	pdata16_typE_SET(97, 53, 9, 64, 46, -52, 46, -47, 125, -62, pdata16);
	pdata16_len_SET(-3, 88, -127, 37, -67, -87, 115, -31, 4, 8, pdata16);
	pdata16_daTa_SET(&92, -49, -96, 34, -111, 83, 5, -101, -120, 31, pdata16);
	
}


void read_SET_MAG_OFFSETS(pset_mag_offsets_SET_MAG_OFFSETS *const pset_mag_offsets) {
	int8_t  target_system    = pset_mag_offsets_target_system_GET(pset_mag_offsets);
	int8_t  target_component = pset_mag_offsets_target_component_GET(pset_mag_offsets);
	int16_t mag_ofs_x        = pset_mag_offsets_mag_ofs_x_GET(pset_mag_offsets);
	int16_t mag_ofs_y        = pset_mag_offsets_mag_ofs_y_GET(pset_mag_offsets);
	int16_t mag_ofs_z        = pset_mag_offsets_mag_ofs_z_GET(pset_mag_offsets);
	
}


void write_SET_MAG_OFFSETS(pset_mag_offsets_SET_MAG_OFFSETS *const pset_mag_offsets) {
	pset_mag_offsets_target_system_SET(126, -93, -116, -37, 13, 1, 96, -66, -94, 57, pset_mag_offsets);
	pset_mag_offsets_target_component_SET(-6, 114, -115, -32, -103, -36, 22, 60, 106, -106, pset_mag_offsets);
	pset_mag_offsets_mag_ofs_x_SET(20080, 24176, 31578, 12712, -27917, -17299, 7675, -10240, 24527, 19823, pset_mag_offsets);
	pset_mag_offsets_mag_ofs_y_SET(2328, -7594, -14530, -19533, 192, -20663, 8011, -23868, -16291, -10938, pset_mag_offsets);
	pset_mag_offsets_mag_ofs_z_SET(-18928, -9961, -18660, -13585, 20519, 15365, 10729, 1145, 32351, -13003, pset_mag_offsets);
	
}


void read_AP_ADC(pap_adc_AP_ADC *const pap_adc) {
	int16_t adc1 = pap_adc_adc1_GET(pap_adc);
	int16_t adc2 = pap_adc_adc2_GET(pap_adc);
	int16_t adc3 = pap_adc_adc3_GET(pap_adc);
	int16_t adc4 = pap_adc_adc4_GET(pap_adc);
	int16_t adc5 = pap_adc_adc5_GET(pap_adc);
	int16_t adc6 = pap_adc_adc6_GET(pap_adc);
	
}


void write_AP_ADC(pap_adc_AP_ADC *const pap_adc) {
	pap_adc_adc1_SET(-10613, 22877, 15062, 10857, 2164, -29885, -27333, 32542, -32710, -12944, pap_adc);
	pap_adc_adc2_SET(-14978, 23323, -25343, 17282, -4217, -25636, 5428, 9275, 12613, -12412, pap_adc);
	pap_adc_adc3_SET(4728, -30224, -3570, -7082, -11913, -29300, -22952, 5992, -32557, 7775, pap_adc);
	pap_adc_adc4_SET(30720, -10921, 30569, -15275, 29061, -15895, 8652, -2209, -7508, 25626, pap_adc);
	pap_adc_adc5_SET(-28242, -22473, -2007, -3795, -23769, 360, -12705, -20543, 10052, -3972, pap_adc);
	pap_adc_adc6_SET(-18396, -7801, -26481, -31101, 19198, 3851, -3926, 19403, 10796, 13134, pap_adc);
	
}


void read_WIND(pwind_WIND *const pwind) {
	float direction = pwind_direction_GET(pwind);
	float speed     = pwind_speed_GET(pwind);
	float speed_z   = pwind_speed_z_GET(pwind);
	
}


void write_WIND(pwind_WIND *const pwind) {
	pwind_direction_SET(-8.076102E37F, 2.5836713E38F, -2.2220688E38F, -2.7441482E38F, -2.5840504E38F, 2.1186279E38F, -1.3557575E38F, 2.7117318E38F, 7.2105705E37F, -2.7502077E38F, pwind);
	pwind_speed_SET(-1.9914775E38F, 2.90853E38F, 4.7629664E37F, 1.9862536E37F, -7.9508394E37F, 3.092657E38F, 2.8273273E38F, 3.3223816E38F, -2.288838E37F, 6.619437E37F, pwind);
	pwind_speed_z_SET(-3.5988E37F, 2.3608526E38F, 1.9846904E38F, 1.1690664E38F, 3.329079E38F, 2.7319169E37F, 3.22029E38F, 8.672975E37F, 4.4883867E37F, 2.1086094E38F, pwind);
	
}


void read_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *const pautopilot_version_request) {
	int8_t target_system    = pautopilot_version_request_target_system_GET(pautopilot_version_request);
	int8_t target_component = pautopilot_version_request_target_component_GET(pautopilot_version_request);
	
}


void write_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST *const pautopilot_version_request) {
	pautopilot_version_request_target_system_SET(-118, -75, 30, -4, -72, 41, 72, 43, 47, -43, pautopilot_version_request);
	pautopilot_version_request_target_component_SET(52, -58, -88, -48, -63, -78, -96, -59, -53, -53, pautopilot_version_request);
	
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
	pdata_transmission_handshake_width_SET(7473, -31576, -3590, 1860, 3288, -12338, 24871, -6559, 19910, -21140, pdata_transmission_handshake);
	pdata_transmission_handshake_height_SET(29339, -10008, -13243, -26592, 10152, -18448, -17490, 18804, -20688, 9709, pdata_transmission_handshake);
	pdata_transmission_handshake_packets_SET(-10989, -31656, -24354, -28682, -6108, 15184, -21220, -12291, -16017, -17753, pdata_transmission_handshake);
	pdata_transmission_handshake_size_SET(1046513845, 1648087998, 1011071321, 286192873, -420227734, 566472792, 1771027661, 1060939925, 488174262, -1185783770, pdata_transmission_handshake);
	pdata_transmission_handshake_typE_SET(105, 119, 23, 32, 68, 15, 29, -20, -38, -124, pdata_transmission_handshake);
	pdata_transmission_handshake_payload_SET(10, 93, -93, -39, -63, 122, -88, -85, 35, -50, pdata_transmission_handshake);
	pdata_transmission_handshake_jpg_quality_SET(-19, 7, -87, -28, 34, -47, 8, -67, 94, -92, pdata_transmission_handshake);
	
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
	pscaled_imu2_time_boot_ms_SET(534210837, 299478022, -543081612, 658367088, 1456199175, 220608892, 1617134527, -1215906656, 424866557, 1190427111, pscaled_imu2);
	pscaled_imu2_xacc_SET(20367, 15360, -25734, -10866, -14361, 16823, -18097, -9455, -7519, 9471, pscaled_imu2);
	pscaled_imu2_yacc_SET(1602, -25196, -27452, -27181, -12771, -8403, -8258, 19188, -12987, -7428, pscaled_imu2);
	pscaled_imu2_zacc_SET(2494, 399, 30115, 9433, -5124, 13687, 9679, -24972, 19255, -18859, pscaled_imu2);
	pscaled_imu2_xgyro_SET(31669, 28244, -18431, 8832, 26467, 14258, 26598, 2059, -6186, -32223, pscaled_imu2);
	pscaled_imu2_ygyro_SET(27224, -11614, 14964, -18839, 8375, -28154, 15150, -18099, -18359, 30448, pscaled_imu2);
	pscaled_imu2_zgyro_SET(-11605, -27157, 1160, 31369, -19171, 29687, -27510, -27585, -7909, 28919, pscaled_imu2);
	pscaled_imu2_xmag_SET(20834, 31448, -19157, 15350, 11896, 23188, -2620, 13971, 8346, -23593, pscaled_imu2);
	pscaled_imu2_ymag_SET(-21551, 9955, -15833, 8096, -16626, -9840, -24009, -5107, 30929, 22120, pscaled_imu2);
	pscaled_imu2_zmag_SET(27268, 15797, 6635, 14440, 26511, 31378, -9337, -4866, -32123, 3136, pscaled_imu2);
	
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


void read_DATA64(pdata64_DATA64 *const pdata64) {
	int8_t       typE      = pdata64_typE_GET(pdata64);
	int8_t       len       = pdata64_len_GET(pdata64);
	Vdata64_daTa item_daTa = pdata64_daTa_GET(pdata64);
	for (size_t  index     = 0; index < item_daTa.len; index++)
		some_int8_t = vdata64_daTa_GET(&item_daTa, index);
	
}


void write_DATA64(pdata64_DATA64 *const pdata64) {
	pdata64_typE_SET(-82, -2, -109, 62, -71, 8, 71, 97, 98, -117, pdata64);
	pdata64_len_SET(-56, -30, 82, -56, -31, 88, -77, -115, 3, 75, pdata64);
	pdata64_daTa_SET(&114, -93, 120, 114, -53, 25, -117, 105, 0, -48, pdata64);
	
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


void write_GIMBAL_REPORT(pgimbal_report_GIMBAL_REPORT *const pgimbal_report) {
	pgimbal_report_target_system_SET(-49, 66, -56, 126, 127, 57, 49, -3, 70, 97, pgimbal_report);
	pgimbal_report_target_component_SET(23, -68, -86, 24, 106, -106, -94, -121, 116, 7, pgimbal_report);
	pgimbal_report_delta_time_SET(9.605873E37F, 3.071908E38F, 1.2451124E37F, 3.422329E37F, 1.727306E38F, 3.1763464E38F, 1.479183E37F, -3.2002584E38F, -2.7070247E38F, -1.6002162E38F, pgimbal_report);
	pgimbal_report_delta_angle_x_SET(3.5743688E37F, -1.4209506E38F, 1.5937925E38F, 3.1655913E37F, -1.421094E38F, 7.6352193E37F, 1.8276646E38F, -2.4639588E38F, 2.7312678E38F, 2.054697E38F, pgimbal_report);
	pgimbal_report_delta_angle_y_SET(1.2567917E38F, -5.3295763E37F, 1.9618157E38F, 1.685785E38F, -7.001025E37F, -5.0548516E37F, 3.2674524E38F, 8.0362253E37F, 2.3355882E38F, -1.3171899E38F, pgimbal_report);
	pgimbal_report_delta_angle_z_SET(-2.7178577E38F, -1.2578705E36F, -2.0804915E38F, -7.1570544E37F, 2.949424E38F, 6.1474914E37F, 1.1712964E38F, -6.092097E37F, -3.161984E38F, -1.3501891E37F, pgimbal_report);
	pgimbal_report_delta_velocity_x_SET(-3.2704931E38F, 1.1593339E38F, -2.9939816E38F, -8.514807E37F, -3.631202E37F, 3.048265E38F, 4.991187E37F, -8.325924E37F, -8.192672E37F, -2.1524312E37F, pgimbal_report);
	pgimbal_report_delta_velocity_y_SET(-2.9228125E38F, -1.6515987E38F, 2.7269797E38F, -2.0342882E38F, 9.672645E37F, 2.4770895E38F, -1.7594244E38F, -1.8029089E38F, -1.3396787E38F, -8.883423E37F, pgimbal_report);
	pgimbal_report_delta_velocity_z_SET(2.1690094E38F, 1.9930985E36F, -2.329281E38F, -9.712377E37F, 2.9116001E38F, 1.3575162E38F, -2.9741755E37F, -2.3410168E38F, -1.9091662E38F, -3.1456928E38F, pgimbal_report);
	pgimbal_report_joint_roll_SET(3.649749E37F, 3.3843363E38F, -2.3808803E38F, 2.6391506E38F, -3.19609E38F, 1.0055276E38F, 2.7089296E38F, 2.6208455E38F, -1.8808183E38F, 2.9672224E38F, pgimbal_report);
	pgimbal_report_joint_el_SET(-2.0449368E38F, -1.1721287E38F, 2.4639538E38F, 7.0441955E37F, -2.0650988E38F, -5.5987173E37F, -2.9472307E38F, -2.1521117E38F, 6.04917E37F, -1.0008211E37F, pgimbal_report);
	pgimbal_report_joint_az_SET(-2.287151E38F, 1.6153848E38F, -4.0911593E37F, 1.2503991E38F, 2.2118704E38F, 1.5979477E38F, 2.3632542E38F, -2.3108307E38F, 2.218344E37F, 4.3934316E37F, pgimbal_report);
	
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


void write_DEVICE_OP_WRITE(pdevice_op_write_DEVICE_OP_WRITE *const pdevice_op_write) {
	pdevice_op_write_request_id_SET(725601079, -999523947, 1269802998, -1570677732, -1439350051, 1531812841, -329271423, -571963193, -871286003, 1657698271, pdevice_op_write);
	pdevice_op_write_target_system_SET(-67, -63, -35, -113, 0, 66, 27, -89, -48, 87, pdevice_op_write);
	pdevice_op_write_target_component_SET(-7, -46, -35, 65, 10, -80, -55, -4, -67, -79, pdevice_op_write);
	pdevice_op_write_bus_SET(-59, -21, -52, 53, 18, -101, -16, -123, -80, -72, pdevice_op_write);
	pdevice_op_write_address_SET(-44, -69, -119, 118, -100, -71, 37, 24, 62, -99, pdevice_op_write);
	pdevice_op_write_regstart_SET(74, -50, -56, -127, -51, -44, 28, -31, 106, -14, pdevice_op_write);
	pdevice_op_write_count_SET(-46, -14, -5, -1, 105, -113, 28, -55, -67, 29, pdevice_op_write);
	pdevice_op_write_daTa_SET(&-74, -12, -39, -65, -78, -45, -49, -22, 19, 119, pdevice_op_write);
	pdevice_op_write_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, pdevice_op_write);
	pdevice_op_write_busname_SET(some_string, strlen(some_string), pdevice_op_write);
	
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
	pdistance_sensor_min_distance_SET(-28489, -8013, 12042, 5559, 21749, 18678, -13500, 11545, -32215, -31951, pdistance_sensor);
	pdistance_sensor_max_distance_SET(3536, -31089, 29218, -5539, 3695, -16666, 1349, 5898, -26964, -24278, pdistance_sensor);
	pdistance_sensor_current_distance_SET(31182, -4864, -5204, -851, -1386, 10957, -27392, 18458, 4647, 9449, pdistance_sensor);
	pdistance_sensor_time_boot_ms_SET(-1038996204, -1521678676, 1255516987, -1162347648, -1040834576, 1090426766, 2070794240, -337655917, 1932917676, -194065040, pdistance_sensor);
	pdistance_sensor_id_SET(73, 11, -32, -23, -106, 117, 11, -60, -115, 113, pdistance_sensor);
	pdistance_sensor_covariance_SET(107, 85, -97, 76, -76, -19, 122, -51, -40, -113, pdistance_sensor);
	pdistance_sensor_typE_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, pdistance_sensor);
	pdistance_sensor_orientation_SET(e_MAV_SENSOR_ORIENTATION_ROLL_270_YAW_135, e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_135, e_MAV_SENSOR_ORIENTATION_ROLL_90_YAW_270, e_MAV_SENSOR_ORIENTATION_YAW_225, e_MAV_SENSOR_ORIENTATION_ROLL_180_YAW_270, e_MAV_SENSOR_ORIENTATION_ROLL_90, e_MAV_SENSOR_ORIENTATION_PITCH_180_YAW_270, e_MAV_SENSOR_ORIENTATION_YAW_315, e_MAV_SENSOR_ORIENTATION_YAW_225, e_MAV_SENSOR_ORIENTATION_YAW_225, pdistance_sensor);
	
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
	phil_optical_flow_integration_time_us_SET(-2126932171, -1669280198, -1369577466, 134911284, 278776955, -1126095254, -2130007425, 999943353, -1134547608, -2018623841, phil_optical_flow);
	phil_optical_flow_time_delta_distance_us_SET(423307899, 1137623054, -1148229984, -1473319692, -1425029117, -1028348153, -612570942, -1481691392, 680913721, 1430751899, phil_optical_flow);
	phil_optical_flow_time_usec_SET(5783346382778813633L, 5422886858430320597L, -77780725874489047L, 121290102246673336L, 1306237202713864788L, 3210837632949403231L, 3866532822704633894L, 5653619731778843987L, -1431315777440532255L, -1101265164355311818L, phil_optical_flow);
	phil_optical_flow_sensor_id_SET(-118, 96, -10, 114, -112, -3, 36, -20, -15, -77, phil_optical_flow);
	phil_optical_flow_integrated_x_SET(-2.572881E38F, 3.7289758E37F, -3.2992049E38F, 1.7097946E38F, 1.622717E38F, -1.2904558E38F, 2.3241307E38F, 4.649865E37F, 2.1111326E37F, -2.6745473E37F, phil_optical_flow);
	phil_optical_flow_integrated_y_SET(2.3944754E38F, -6.703301E37F, -1.8250418E38F, 1.4535789E38F, 1.02493493E37F, -7.0864817E37F, -2.9776488E38F, -2.0611376E38F, -8.357709E37F, 3.4455666E36F, phil_optical_flow);
	phil_optical_flow_integrated_xgyro_SET(1.2113098E38F, -2.874553E38F, 1.3179828E38F, 1.4053613E38F, -2.1978958E38F, 1.1840907E38F, 3.2714959E38F, -1.6729019E38F, -1.7562814E38F, 2.1868612E38F, phil_optical_flow);
	phil_optical_flow_integrated_ygyro_SET(-1.6317662E38F, -1.9186659E38F, -2.7643738E37F, 1.3575823E38F, 3.1476572E38F, -3.0141523E38F, -2.356596E38F, 1.3474942E38F, -8.4945495E37F, -1.4603463E38F, phil_optical_flow);
	phil_optical_flow_integrated_zgyro_SET(1.8667129E38F, -3.3864647E38F, 2.8235775E38F, 4.604464E37F, -7.141983E37F, -1.9505145E38F, 5.485604E37F, -8.560193E37F, -2.0316316E38F, 2.5543928E38F, phil_optical_flow);
	phil_optical_flow_temperature_SET(2500, 6283, 21961, -15834, -20442, 17828, -10182, -1545, 5580, -25511, phil_optical_flow);
	phil_optical_flow_quality_SET(-116, 31, -10, 50, 78, 126, 86, 10, 39, -90, phil_optical_flow);
	phil_optical_flow_distance_SET(1.2201871E38F, 3.1223573E37F, 1.3171723E38F, 2.659585E38F, 1.1240062E38F, 3.095658E38F, 6.5038726E37F, -2.635844E38F, -1.597072E37F, 2.5236682E38F, phil_optical_flow);
	
}


void read_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2 *const pscaled_pressure2) {
	int32_t time_boot_ms = pscaled_pressure2_time_boot_ms_GET(pscaled_pressure2);
	float   press_abs    = pscaled_pressure2_press_abs_GET(pscaled_pressure2);
	float   press_diff   = pscaled_pressure2_press_diff_GET(pscaled_pressure2);
	int16_t temperature  = pscaled_pressure2_temperature_GET(pscaled_pressure2);
	
}


void write_SCALED_PRESSURE2(pscaled_pressure2_SCALED_PRESSURE2 *const pscaled_pressure2) {
	pscaled_pressure2_time_boot_ms_SET(1879819239, 1053148188, 509692107, 1718324344, -1531548332, -1804087313, -1125389560, -1494751032, -256624856, -1037817256, pscaled_pressure2);
	pscaled_pressure2_press_abs_SET(1.2369447E37F, -1.4959544E38F, -7.4649196E37F, -1.7580393E38F, 1.841569E38F, 1.3625461E38F, -2.5944613E38F, 1.7962421E37F, -1.2035086E38F, 1.2677162E38F, pscaled_pressure2);
	pscaled_pressure2_press_diff_SET(-2.358022E38F, -3.1244571E38F, -2.902197E38F, -2.6441877E38F, 7.2702003E37F, 1.1054425E38F, 9.242977E37F, -7.211601E37F, -2.7896931E38F, -2.9136093E38F, pscaled_pressure2);
	pscaled_pressure2_temperature_SET(29445, 17775, -3121, -13873, 18031, 5132, -15105, -10994, -22794, -29061, pscaled_pressure2);
	
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
	pwind_cov_time_usec_SET(8320270369497673095L, -7548584081070562671L, 4351809358661928853L, -3947324585664693650L, -8534055580864923712L, -1249037077133197948L, 2491903265024774931L, 3129568255144729935L, -830447390784724661L, 4078140008913815987L, pwind_cov);
	pwind_cov_wind_x_SET(1.1313771E37F, -1.8943742E38F, -3.2646041E38F, 2.8671837E38F, 7.7001E37F, 3.8359951E37F, 2.1735368E38F, 2.6999956E37F, -1.5380941E38F, 2.0466556E37F, pwind_cov);
	pwind_cov_wind_y_SET(-3.2529E38F, 2.7948444E38F, -2.73902E38F, 3.3693603E38F, -1.9271246E38F, -7.909238E37F, -9.913775E37F, -1.7100432E38F, 2.6252835E38F, 2.0856811E37F, pwind_cov);
	pwind_cov_wind_z_SET(1.4190911E38F, -1.202253E38F, -5.8474217E37F, -1.1266399E38F, -6.4583127E37F, -6.3681336E37F, 3.7995586E37F, -1.4981165E38F, -4.233206E36F, 2.0960373E38F, pwind_cov);
	pwind_cov_var_horiz_SET(1.5213719E38F, -6.9486967E37F, -2.9288705E37F, 5.2614964E37F, -1.0396242E38F, -2.4257322E38F, 3.1533237E38F, 9.474283E37F, 1.4590893E38F, -1.2004555E38F, pwind_cov);
	pwind_cov_var_vert_SET(-5.168439E37F, 1.7524E37F, -1.0867872E38F, 1.0421954E38F, 3.3383785E38F, -8.055894E37F, -1.1195666E38F, 3.0873016E38F, -3.2581959E38F, -2.0773548E38F, pwind_cov);
	pwind_cov_wind_alt_SET(2.0596122E38F, -7.8985823E37F, 1.0112773E38F, -8.441381E37F, -2.0621671E38F, 1.457684E38F, -3.2961388E38F, 2.9860477E38F, -6.155382E37F, 1.4059611E36F, pwind_cov);
	pwind_cov_horiz_accuracy_SET(6.9507534E37F, 5.4933304E37F, 1.820678E38F, -8.229673E37F, 1.8984242E38F, 1.6719589E38F, 1.1843272E38F, 1.2596416E38F, -9.891932E37F, -1.3313871E38F, pwind_cov);
	pwind_cov_vert_accuracy_SET(-1.5713223E38F, -1.2994401E38F, 1.7162196E38F, 3.2956433E38F, -1.6352982E38F, -1.9742658E38F, -1.3794556E38F, 1.203676E38F, 1.4532206E38F, 1.6836997E38F, pwind_cov);
	
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


void write_GOPRO_SET_REQUEST(pgopro_set_request_GOPRO_SET_REQUEST *const pgopro_set_request) {
	pgopro_set_request_target_system_SET(-107, 114, -63, -67, -72, 99, 50, -7, 91, -89, pgopro_set_request);
	pgopro_set_request_target_component_SET(70, -40, 75, 7, 105, 55, 18, 15, 8, -87, pgopro_set_request);
	pgopro_set_request_value_SET(&-8, 30, 7, 47, -21, -57, 37, -44, 19, 4, pgopro_set_request);
	pgopro_set_request_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_LOW_LIGHT, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_SHARPNESS, e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER, e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER, e_GOPRO_COMMAND_GOPRO_COMMAND_CAPTURE_MODE, e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_BURST_RATE, e_GOPRO_COMMAND_GOPRO_COMMAND_BATTERY, e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER, e_GOPRO_COMMAND_GOPRO_COMMAND_VIDEO_SETTINGS, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_GAIN, pgopro_set_request);
	
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


void write_VISION_POSITION_DELTA(pvision_position_delta_VISION_POSITION_DELTA *const pvision_position_delta) {
	pvision_position_delta_time_usec_SET(6721169476505701619L, -2020086360022423001L, -3393172802860105335L, -445987410510109847L, 3330953419595354758L, -3207811308372918627L, 5988649308075469732L, -8705594425306915010L, 2515040694092092049L, 2214522333231762884L, pvision_position_delta);
	pvision_position_delta_time_delta_usec_SET(9181195224738966296L, -7056985869375556472L, 7184987150043804224L, -258551771004516883L, -1354861548645038324L, -2095791643477089104L, -6703024839104027527L, 1956800736210426415L, -6979446033212134633L, 2956805363141517102L, pvision_position_delta);
	pvision_position_delta_angle_delta_SET(&-1.9609473E37F, -2.58798E37F, 2.9773444E38F, 3.1451736E38F, -1.2908994E38F, 1.8451778E38F, -6.770231E37F, -3.0541668E38F, 7.362831E37F, -6.0247275E37F, pvision_position_delta);
	pvision_position_delta_position_delta_SET(&-2.3670744E38F, 1.6151631E37F, -2.0978601E38F, 6.0680163E37F, -1.6341709E38F, -2.531003E38F, -2.4245907E38F, -1.1000738E38F, 2.4402753E38F, 7.659703E37F, pvision_position_delta);
	pvision_position_delta_confidence_SET(-1.9183803E37F, -2.426043E38F, 1.3618771E38F, -2.3117947E38F, 1.8462E37F, 1.7458376E38F, 8.724753E37F, -2.5536207E38F, -2.7807656E38F, -3.0853601E38F, pvision_position_delta);
	
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


void write_LOGGING_DATA(plogging_data_LOGGING_DATA *const plogging_data) {
	plogging_data_sequence_SET(12827, 21074, 23375, -8005, 21106, 1220, 15878, 29431, -3742, 18985, plogging_data);
	plogging_data_target_system_SET(0, -103, -45, 32, -127, 60, -91, -123, 8, 34, plogging_data);
	plogging_data_target_component_SET(-54, 2, 60, -127, 2, -29, 9, 55, -76, 79, plogging_data);
	plogging_data_length_SET(99, -97, -91, 121, 26, -128, -108, -64, 60, -3, plogging_data);
	plogging_data_first_message_offset_SET(-18, -71, -83, -38, -37, 74, -88, 5, -15, -85, plogging_data);
	plogging_data_daTa_SET(&-116, -56, 86, -114, -68, -3, 13, -93, -41, 121, plogging_data);
	
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


void write_DEVICE_OP_READ(pdevice_op_read_DEVICE_OP_READ *const pdevice_op_read) {
	pdevice_op_read_request_id_SET(-744614549, 227936936, -235481002, -1529996874, 1548200237, 2004375719, 577017485, -1529614282, -1911224202, -2081610909, pdevice_op_read);
	pdevice_op_read_target_system_SET(4, -89, 35, 72, -85, 127, 75, -118, 112, 21, pdevice_op_read);
	pdevice_op_read_target_component_SET(-124, -110, -119, 125, -1, 116, -76, -90, -109, -72, pdevice_op_read);
	pdevice_op_read_bus_SET(28, -96, -90, 48, -47, -34, 34, -70, 94, 84, pdevice_op_read);
	pdevice_op_read_address_SET(10, -88, -17, -20, -7, 53, 53, -85, 45, 92, pdevice_op_read);
	pdevice_op_read_regstart_SET(-28, 71, 119, -100, -104, 102, 60, 88, -120, 77, pdevice_op_read);
	pdevice_op_read_count_SET(56, -87, 46, -75, 77, 23, -58, 37, 98, 34, pdevice_op_read);
	pdevice_op_read_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, pdevice_op_read);
	pdevice_op_read_busname_SET(some_string, strlen(some_string), pdevice_op_read);
	
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


void write_MAG_CAL_PROGRESS(pmag_cal_progress_MAG_CAL_PROGRESS *const pmag_cal_progress) {
	pmag_cal_progress_compass_id_SET(-118, -18, 53, -35, 104, -65, 98, 89, 99, 113, pmag_cal_progress);
	pmag_cal_progress_cal_mask_SET(-84, 21, -87, -45, -40, -120, -97, 60, 3, -60, pmag_cal_progress);
	pmag_cal_progress_attempt_SET(-94, -50, 57, 69, -76, 105, 79, 38, 16, -19, pmag_cal_progress);
	pmag_cal_progress_completion_pct_SET(-112, -104, -115, 44, 16, -106, -1, -94, -73, 78, pmag_cal_progress);
	pmag_cal_progress_completion_mask_SET(&-41, -65, -68, 17, 55, 71, -51, 115, -121, -5, pmag_cal_progress);
	pmag_cal_progress_direction_x_SET(2.9311785E38F, 2.0510925E37F, 1.7345965E38F, 1.4176445E37F, 4.4267206E37F, 7.9874664E37F, 8.3580823E37F, -2.751333E38F, -2.5141813E38F, 6.855074E37F, pmag_cal_progress);
	pmag_cal_progress_direction_y_SET(1.021648E38F, -2.531528E38F, -5.3344933E37F, 1.6895306E37F, 3.001569E38F, 2.2568365E38F, -3.354498E38F, -1.0233085E38F, -4.0295198E37F, -7.5855766E37F, pmag_cal_progress);
	pmag_cal_progress_direction_z_SET(-4.3341197E37F, -3.9442287E37F, 1.3550326E38F, -3.1157834E38F, 6.381774E37F, 1.9250102E37F, -4.4663925E37F, -1.273434E38F, -2.2411795E38F, 4.069208E37F, pmag_cal_progress);
	pmag_cal_progress_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED, e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE, e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE, e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE, e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE, e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED, e_MAG_CAL_STATUS_MAG_CAL_SUCCESS, e_MAG_CAL_STATUS_MAG_CAL_FAILED, e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_TWO, e_MAG_CAL_STATUS_MAG_CAL_SUCCESS, pmag_cal_progress);
	
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
	phighres_imu_fields_updated_SET(17452, -20617, 24832, -17614, 9634, 2682, -17020, 16641, 804, 8979, phighres_imu);
	phighres_imu_time_usec_SET(-1825129714950564706L, -4237135932214563974L, 2237816160039572375L, 8870090508421572201L, 8836497900773058344L, 8827451248465753506L, 6975237032463290643L, -3316041588898299707L, -7651638724020897455L, -5827059871763434410L, phighres_imu);
	phighres_imu_xacc_SET(-2.3842127E38F, 3.083666E37F, 1.519496E38F, 2.44021E38F, 2.7044036E38F, 3.3771768E38F, 2.2559916E37F, 2.389836E38F, -1.191849E38F, -3.3356887E38F, phighres_imu);
	phighres_imu_yacc_SET(3.084758E38F, -2.6735493E38F, 3.004932E38F, -1.4191923E38F, 2.1516414E38F, -9.24847E37F, 2.031324E38F, -1.508301E38F, -2.8224644E38F, 2.484275E38F, phighres_imu);
	phighres_imu_zacc_SET(-1.6518063E38F, 1.4643644E38F, -2.6125447E37F, -7.6750626E37F, -1.0376018E38F, 2.2003532E38F, -3.2262353E38F, 2.947581E38F, 5.1477907E37F, 1.960504E38F, phighres_imu);
	phighres_imu_xgyro_SET(2.0974784E38F, -2.2188741E38F, -1.2624625E38F, -1.2872142E38F, -3.328367E38F, -2.6416254E37F, -2.873035E38F, -7.1891036E37F, 1.7781211E38F, -5.0487664E37F, phighres_imu);
	phighres_imu_ygyro_SET(1.1365869E38F, -2.4915796E38F, -2.5542669E38F, -1.2716109E38F, 2.6981584E38F, 1.6241271E38F, -1.1694396E38F, 1.5946526E38F, -1.999038E38F, 2.0441168E38F, phighres_imu);
	phighres_imu_zgyro_SET(-3.018764E38F, -2.0167761E38F, 2.9792251E38F, -9.94548E37F, 7.60496E37F, 1.2647158E38F, -9.889178E37F, 3.3747774E38F, 3.203251E38F, 1.798671E38F, phighres_imu);
	phighres_imu_xmag_SET(-5.6077496E37F, 2.6247664E37F, -1.931216E38F, -1.0734025E38F, -2.5170079E38F, -1.0424237E38F, 1.7233047E38F, -2.0343995E38F, 2.870889E38F, -2.4957095E38F, phighres_imu);
	phighres_imu_ymag_SET(1.4404555E38F, -5.524742E37F, 1.3985731E38F, 2.3079866E37F, -3.317564E38F, 3.3617068E38F, 5.199283E37F, -3.081081E38F, -9.020032E37F, -2.0704594E38F, phighres_imu);
	phighres_imu_zmag_SET(2.1863603E37F, -3.0413264E38F, 4.86343E37F, -1.2477854E36F, -3.0872207E38F, 2.1626415E38F, -9.160125E36F, 2.3637307E38F, 1.440477E38F, 1.3353406E38F, phighres_imu);
	phighres_imu_abs_pressure_SET(-1.3482068E38F, -1.8546556E38F, 3.2926044E38F, -2.9373497E38F, -1.7810556E38F, -3.1615017E38F, 2.6259717E37F, 8.669136E37F, -1.7136056E38F, 2.949823E38F, phighres_imu);
	phighres_imu_diff_pressure_SET(-2.6994388E38F, -1.5734574E38F, -1.8774037E38F, 1.7542583E38F, -1.8315336E38F, 7.0167427E37F, -1.1690474E38F, -4.2804425E36F, -6.2198357E37F, -1.0491172E36F, phighres_imu);
	phighres_imu_pressure_alt_SET(-9.761654E35F, -1.3596806E38F, 3.2020623E37F, 3.0919576E38F, 2.8253287E38F, 4.36419E37F, 3.2791918E38F, -3.190258E38F, -6.4225046E37F, 1.4626183E36F, phighres_imu);
	phighres_imu_temperature_SET(-1.0845643E38F, 2.106171E38F, 1.1054286E37F, -2.5327604E38F, -2.7894493E38F, -9.777218E37F, -1.1119879E38F, -4.957775E37F, -1.2873248E38F, -1.709351E38F, phighres_imu);
	
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
	pextended_sys_state_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, pextended_sys_state);
	pextended_sys_state_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, pextended_sys_state);
	
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


void write_UAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *const puavionix_adsb_out_dynamic) {
	puavionix_adsb_out_dynamic_accuracyVert_SET(-18060, -26375, -19402, 7235, 32119, 5276, 1884, -27057, 31604, 26774, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_accuracyVel_SET(28401, -23552, 17365, -17896, 20562, -22847, -19145, -259, 8913, 28104, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_squawk_SET(25315, 8847, 22049, 22861, -16999, -10325, 21105, -2371, -24306, -3038, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_utcTime_SET(278130572, 396512761, -462419375, -1636948781, 182537729, 941629784, -488007384, 357936216, 1106292489, -932351971, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_accuracyHor_SET(1365913269, -992312658, -353598793, 1591853866, -1968994645, 872908451, 730236158, -168908773, -64228888, 716006709, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_gpsLat_SET(-1416753815, -876235019, 1188350728, 1581117446, 754687154, 740566476, -1644466350, 72940561, -40697666, 1702354980, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_gpsLon_SET(1054973240, 1300485344, 951183293, 1343014396, 424154569, 1169613655, -719285438, -497570856, -35669889, 550114002, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_gpsAlt_SET(790115455, 1659951298, -1446578321, 1179456257, -1622624805, 448302371, 467209243, -1464768867, 1198726359, -818152758, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_numSats_SET(-18, 57, 42, 23, 62, 85, -110, -37, 81, 115, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_baroAltMSL_SET(-67516784, -211368735, 1070214819, 1245378707, 2027749245, -626942388, 1366776493, -1406643467, 747653307, 1944564937, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_velVert_SET(7646, -9939, 18669, 12339, 20558, -30565, -2340, -10263, -19271, -567, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_velNS_SET(-31295, -5385, -11051, 20679, 24612, -328, 28923, 19312, 15455, 18133, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_VelEW_SET(-15091, -4727, -23186, 8068, 15093, -14265, -16101, -3433, 1211, -37, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D, e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY, e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY, puavionix_adsb_out_dynamic);
	puavionix_adsb_out_dynamic_state_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT, e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED, puavionix_adsb_out_dynamic);
	
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


void write_GOPRO_GET_RESPONSE(pgopro_get_response_GOPRO_GET_RESPONSE *const pgopro_get_response) {
	pgopro_get_response_value_SET(&-74, -75, 23, -16, 90, 39, 90, 29, -26, 25, pgopro_get_response);
	pgopro_get_response_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_LOW_LIGHT, e_GOPRO_COMMAND_GOPRO_COMMAND_MODEL, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE, e_GOPRO_COMMAND_GOPRO_COMMAND_POWER, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_WHITE_BALANCE, e_GOPRO_COMMAND_GOPRO_COMMAND_TIME, e_GOPRO_COMMAND_GOPRO_COMMAND_CHARGING, e_GOPRO_COMMAND_GOPRO_COMMAND_SHUTTER, e_GOPRO_COMMAND_GOPRO_COMMAND_LOW_LIGHT, e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE, pgopro_get_response);
	pgopro_get_response_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, pgopro_get_response);
	
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
	pgps_inject_data_target_system_SET(79, -7, -99, -115, 68, 65, -78, 108, 105, 121, pgps_inject_data);
	pgps_inject_data_target_component_SET(4, 91, -87, -48, 81, -127, -27, 92, -99, 31, pgps_inject_data);
	pgps_inject_data_len_SET(-63, 29, 78, 38, 27, -20, -29, -55, -127, 100, pgps_inject_data);
	pgps_inject_data_daTa_SET(&41, 67, -51, 33, 92, 30, -55, 55, 74, -73, pgps_inject_data);
	
}


void read_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *const puavionix_adsb_transceiver_health_report) {
	e_UAVIONIX_ADSB_RF_HEALTH item_rfHealth;
	if (puavionix_adsb_transceiver_health_report_rfHealth_GET(puavionix_adsb_transceiver_health_report, &item_rfHealth)) {
		e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = item_rfHealth;
	}
	
}


void write_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *const puavionix_adsb_transceiver_health_report) {
	puavionix_adsb_transceiver_health_report_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING, e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING, puavionix_adsb_transceiver_health_report);
	
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


void read_NAMED_VALUE_INT(pnamed_value_int_NAMED_VALUE_INT *const pnamed_value_int) {
	int32_t               time_boot_ms = pnamed_value_int_time_boot_ms_GET(pnamed_value_int);
	int32_t               value        = pnamed_value_int_value_GET(pnamed_value_int);
	Vnamed_value_int_name item_name;
	if (pnamed_value_int_name_GET(pnamed_value_int, &item_name)) {
		memcpy(some_string, item_name.bytes, item_name.len);
	}
	
}


void write_NAMED_VALUE_INT(pnamed_value_int_NAMED_VALUE_INT *const pnamed_value_int) {
	pnamed_value_int_time_boot_ms_SET(-420835189, -223069256, -362619778, 635489592, -230162027, 987350182, 888661080, -1372516486, -523184507, 1561829763, pnamed_value_int);
	pnamed_value_int_value_SET(1176313919, -138082981, -473893672, -858056343, 489861101, 1317284114, 1721863962, -1344248041, 10357068, 1859644510, pnamed_value_int);
	pnamed_value_int_name_SET(some_string, strlen(some_string), pnamed_value_int);
	
}


void read_RPM(prpm_RPM *const prpm) {
	float rpm1 = prpm_rpm1_GET(prpm);
	float rpm2 = prpm_rpm2_GET(prpm);
	
}


void write_RPM(prpm_RPM *const prpm) {
	prpm_rpm1_SET(1.3077401E38F, 5.969411E37F, 3.0149139E38F, 2.0404108E37F, 1.4567704E38F, 6.639586E37F, -2.7390181E38F, 3.0835341E38F, 1.8609902E38F, -2.4226965E38F, prpm);
	prpm_rpm2_SET(9.592888E37F, 2.3694235E38F, 2.0470967E38F, -3.1893647E38F, 5.176246E37F, 2.4843792E38F, 2.918331E38F, 1.1931313E38F, 2.7382581E38F, 2.493416E36F, prpm);
	
}


void read_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA *const pgps_rtcm_data) {
	int8_t              flags     = pgps_rtcm_data_flags_GET(pgps_rtcm_data);
	int8_t              len       = pgps_rtcm_data_len_GET(pgps_rtcm_data);
	Vgps_rtcm_data_daTa item_daTa = pgps_rtcm_data_daTa_GET(pgps_rtcm_data);
	for (size_t         index     = 0; index < item_daTa.len; index++)
		some_int8_t = vgps_rtcm_data_daTa_GET(&item_daTa, index);
	
}


void write_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA *const pgps_rtcm_data) {
	pgps_rtcm_data_flags_SET(-36, 90, -52, -88, 118, -64, -116, -54, 113, 73, pgps_rtcm_data);
	pgps_rtcm_data_len_SET(124, -125, -78, 30, 88, -41, -37, -3, -110, -17, pgps_rtcm_data);
	pgps_rtcm_data_daTa_SET(&-59, -116, -7, -126, 50, -17, -42, -78, -14, 3, pgps_rtcm_data);
	
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


void read_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *const pfile_transfer_protocol) {
	int8_t                          target_network   = pfile_transfer_protocol_target_network_GET(pfile_transfer_protocol);
	int8_t                          target_system    = pfile_transfer_protocol_target_system_GET(pfile_transfer_protocol);
	int8_t                          target_component = pfile_transfer_protocol_target_component_GET(pfile_transfer_protocol);
	Vfile_transfer_protocol_payload item_payload     = pfile_transfer_protocol_payload_GET(pfile_transfer_protocol);
	for (size_t                     index            = 0; index < item_payload.len; index++)
		some_int8_t = vfile_transfer_protocol_payload_GET(&item_payload, index);
	
}


void write_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL *const pfile_transfer_protocol) {
	pfile_transfer_protocol_target_network_SET(-127, 52, 119, -22, -9, 81, -12, 71, -94, 61, pfile_transfer_protocol);
	pfile_transfer_protocol_target_system_SET(127, -22, -60, -18, 109, 57, 62, -105, -18, 112, pfile_transfer_protocol);
	pfile_transfer_protocol_target_component_SET(-49, 83, 85, 65, 41, 21, -114, 94, 126, -29, pfile_transfer_protocol);
	pfile_transfer_protocol_payload_SET(&104, 109, -2, 69, 106, 48, -51, -35, 8, -28, pfile_transfer_protocol);
	
}


void read_RANGEFINDER(prangefinder_RANGEFINDER *const prangefinder) {
	float distance = prangefinder_distance_GET(prangefinder);
	float voltage  = prangefinder_voltage_GET(prangefinder);
	
}


void write_RANGEFINDER(prangefinder_RANGEFINDER *const prangefinder) {
	prangefinder_distance_SET(2.1121769E38F, -2.741744E38F, 6.2132586E37F, -6.889367E37F, -2.9415134E38F, -7.537145E37F, 6.9204864E37F, -2.695948E38F, -1.530019E38F, -1.86112E38F, prangefinder);
	prangefinder_voltage_SET(-2.5918617E38F, -1.8468316E38F, -7.9177735E37F, -1.8273076E38F, 2.446241E38F, 1.3150987E38F, -9.846935E37F, 4.624357E37F, 3.0701556E38F, -1.4517542E38F, prangefinder);
	
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
	pradio_status_rxerrors_SET(-13434, -9178, -8613, -28096, -15568, 7255, -2749, -32351, -27701, 21308, pradio_status);
	pradio_status_fixeD_SET(7676, -22860, -30548, 4610, -22531, 20260, 24764, 24466, -15877, 21225, pradio_status);
	pradio_status_rssi_SET(112, -116, 3, 14, 103, -123, -2, -54, 64, -27, pradio_status);
	pradio_status_remrssi_SET(-65, 71, -85, 54, -49, -26, -87, 115, -38, 77, pradio_status);
	pradio_status_txbuf_SET(57, 58, -32, -53, -3, -2, 65, -27, 12, -94, pradio_status);
	pradio_status_noise_SET(-88, -83, -1, 79, -24, 74, -115, -19, 43, 6, pradio_status);
	pradio_status_remnoise_SET(60, 64, 18, 7, -16, -33, -1, -46, 2, -48, pradio_status);
	
}


void read_FENCE_POINT(pfence_point_FENCE_POINT *const pfence_point) {
	int8_t target_system    = pfence_point_target_system_GET(pfence_point);
	int8_t target_component = pfence_point_target_component_GET(pfence_point);
	int8_t idx              = pfence_point_idx_GET(pfence_point);
	int8_t count            = pfence_point_count_GET(pfence_point);
	float  lat              = pfence_point_lat_GET(pfence_point);
	float  lng              = pfence_point_lng_GET(pfence_point);
	
}


void write_FENCE_POINT(pfence_point_FENCE_POINT *const pfence_point) {
	pfence_point_target_system_SET(115, -4, 83, -124, -110, -4, -80, -36, 69, 93, pfence_point);
	pfence_point_target_component_SET(-7, -46, -80, 90, 0, -7, -28, -4, 31, 102, pfence_point);
	pfence_point_idx_SET(-105, 110, 18, -44, -63, -69, 105, -5, -83, 76, pfence_point);
	pfence_point_count_SET(-71, 26, -22, 113, -52, 67, 73, 53, -11, 27, pfence_point);
	pfence_point_lat_SET(8.510455E37F, -4.0488126E37F, 2.4779677E38F, -2.4430548E38F, 3.02785E38F, 3.2622244E38F, -9.12258E37F, 2.9336542E38F, 2.0160425E38F, 5.5783934E36F, pfence_point);
	pfence_point_lng_SET(-2.016589E38F, -2.4414833E38F, -2.129927E38F, 5.598796E37F, -2.4572965E38F, 1.2182742E38F, -2.642749E37F, -7.963887E37F, 2.296604E37F, -1.6363331E38F, pfence_point);
	
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
	presource_request_request_id_SET(-117, -36, 114, 31, -66, 78, 127, -90, -15, -27, presource_request);
	presource_request_uri_type_SET(-70, -92, 16, -11, 45, 15, 106, 51, -55, -90, presource_request);
	presource_request_uri_SET(&-14, 103, 54, -28, 72, -24, -72, -29, 33, 8, presource_request);
	presource_request_transfer_type_SET(2, 9, -96, 13, -15, 95, 33, 97, 57, 1, presource_request);
	presource_request_storage_SET(&-7, 121, -88, -110, -57, 123, -124, 102, -82, 62, presource_request);
	
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

static inline void CommunicationChannel_DEMO_on_RESOURCE_REQUEST(c_CommunicationChannel_DEMO *channel, RESOURCE_REQUEST_resource_request *presource_request) {
	
	read_RESOURCE_REQUEST(presource_request->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ATTITUDE_TARGET(c_CommunicationChannel_DEMO *channel, ATTITUDE_TARGET_attitude_target *pattitude_target) {
	
	read_ATTITUDE_TARGET(pattitude_target->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_COUNT(c_CommunicationChannel_DEMO *channel, MISSION_COUNT_mission_count *pmission_count) {
	
	CURSORS(curs);
	read_MISSION_COUNT(wrap_pack(pmission_count, curs));
	
}

static inline void CommunicationChannel_DEMO_on_ADSB_VEHICLE(c_CommunicationChannel_DEMO *channel, ADSB_VEHICLE_adsb_vehicle *padsb_vehicle) {
	
	CURSORS(curs);
	read_ADSB_VEHICLE(wrap_pack(padsb_vehicle, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MESSAGE_INTERVAL(c_CommunicationChannel_DEMO *channel, MESSAGE_INTERVAL_message_interval *pmessage_interval) {
	
	read_MESSAGE_INTERVAL(pmessage_interval->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ESTIMATOR_STATUS(c_CommunicationChannel_DEMO *channel, ESTIMATOR_STATUS_estimator_status *pestimator_status) {
	
	CURSORS(curs);
	read_ESTIMATOR_STATUS(wrap_pack(pestimator_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_TIMESYNC(c_CommunicationChannel_DEMO *channel, TIMESYNC_timesync *ptimesync) {
	
	read_TIMESYNC(ptimesync->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GLOBAL_POSITION_INT_COV(c_CommunicationChannel_DEMO *channel, GLOBAL_POSITION_INT_COV_global_position_int_cov *pglobal_position_int_cov) {
	
	CURSORS(curs);
	read_GLOBAL_POSITION_INT_COV(wrap_pack(pglobal_position_int_cov, curs));
	
}

static inline void CommunicationChannel_DEMO_on_BUTTON_CHANGE(c_CommunicationChannel_DEMO *channel, BUTTON_CHANGE_button_change *pbutton_change) {
	
	read_BUTTON_CHANGE(pbutton_change->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SAFETY_SET_ALLOWED_AREA(c_CommunicationChannel_DEMO *channel, SAFETY_SET_ALLOWED_AREA_safety_set_allowed_area *psafety_set_allowed_area) {
	
	CURSORS(curs);
	read_SAFETY_SET_ALLOWED_AREA(wrap_pack(psafety_set_allowed_area, curs));
	
}

static inline void CommunicationChannel_DEMO_on_STORAGE_INFORMATION(c_CommunicationChannel_DEMO *channel, STORAGE_INFORMATION_storage_information *pstorage_information) {
	
	read_STORAGE_INFORMATION(pstorage_information->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_COLLISION(c_CommunicationChannel_DEMO *channel, COLLISION_collision *pcollision) {
	
	CURSORS(curs);
	read_COLLISION(wrap_pack(pcollision, curs));
	
}

static inline void CommunicationChannel_DEMO_on_ALTITUDE(c_CommunicationChannel_DEMO *channel, ALTITUDE_altitude *paltitude) {
	
	read_ALTITUDE(paltitude->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HIL_STATE_QUATERNION(c_CommunicationChannel_DEMO *channel, HIL_STATE_QUATERNION_hil_state_quaternion *phil_state_quaternion) {
	
	read_HIL_STATE_QUATERNION(phil_state_quaternion->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_INFORMATION(c_CommunicationChannel_DEMO *channel, CAMERA_INFORMATION_camera_information *pcamera_information) {
	
	CURSORS(curs);
	read_CAMERA_INFORMATION(wrap_pack(pcamera_information, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GPS_STATUS(c_CommunicationChannel_DEMO *channel, GPS_STATUS_gps_status *pgps_status) {
	
	read_GPS_STATUS(pgps_status->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PARAM_SET(c_CommunicationChannel_DEMO *channel, PARAM_SET_param_set *pparam_set) {
	
	CURSORS(curs);
	read_PARAM_SET(wrap_pack(pparam_set, curs));
	
}

static inline void CommunicationChannel_DEMO_on_TERRAIN_DATA(c_CommunicationChannel_DEMO *channel, TERRAIN_DATA_terrain_data *pterrain_data) {
	
	read_TERRAIN_DATA(pterrain_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_RC_CHANNELS_OVERRIDE(c_CommunicationChannel_DEMO *channel, RC_CHANNELS_OVERRIDE_rc_channels_override *prc_channels_override) {
	
	read_RC_CHANNELS_OVERRIDE(prc_channels_override->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SCALED_IMU(c_CommunicationChannel_DEMO *channel, SCALED_IMU_scaled_imu *pscaled_imu) {
	
	read_SCALED_IMU(pscaled_imu->bytes);
	
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

static inline void CommunicationChannel_DEMO_on_PARAM_REQUEST_READ(c_CommunicationChannel_DEMO *channel, PARAM_REQUEST_READ_param_request_read *pparam_request_read) {
	
	CURSORS(curs);
	read_PARAM_REQUEST_READ(wrap_pack(pparam_request_read, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SET_ATTITUDE_TARGET(c_CommunicationChannel_DEMO *channel, SET_ATTITUDE_TARGET_set_attitude_target *pset_attitude_target) {
	
	read_SET_ATTITUDE_TARGET(pset_attitude_target->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_FOLLOW_TARGET(c_CommunicationChannel_DEMO *channel, FOLLOW_TARGET_follow_target *pfollow_target) {
	
	read_FOLLOW_TARGET(pfollow_target->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HIL_STATE(c_CommunicationChannel_DEMO *channel, HIL_STATE_hil_state *phil_state) {
	
	read_HIL_STATE(phil_state->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HOME_POSITION(c_CommunicationChannel_DEMO *channel, HOME_POSITION_home_position *phome_position) {
	
	CURSORS(curs);
	read_HOME_POSITION(wrap_pack(phome_position, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GPS2_RAW(c_CommunicationChannel_DEMO *channel, GPS2_RAW_gps2_raw *pgps2_raw) {
	
	CURSORS(curs);
	read_GPS2_RAW(wrap_pack(pgps2_raw, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MEMORY_VECT(c_CommunicationChannel_DEMO *channel, MEMORY_VECT_memory_vect *pmemory_vect) {
	
	read_MEMORY_VECT(pmemory_vect->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_REQUEST_DATA_STREAM(c_CommunicationChannel_DEMO *channel, REQUEST_DATA_STREAM_request_data_stream *prequest_data_stream) {
	
	read_REQUEST_DATA_STREAM(prequest_data_stream->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HIL_CONTROLS(c_CommunicationChannel_DEMO *channel, HIL_CONTROLS_hil_controls *phil_controls) {
	
	CURSORS(curs);
	read_HIL_CONTROLS(wrap_pack(phil_controls, curs));
	
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

static inline void CommunicationChannel_DEMO_on_PARAM_REQUEST_LIST(c_CommunicationChannel_DEMO *channel, PARAM_REQUEST_LIST_param_request_list *pparam_request_list) {
	
	read_PARAM_REQUEST_LIST(pparam_request_list->bytes);
	
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

static inline void CommunicationChannel_DEMO_on_SET_POSITION_TARGET_GLOBAL_INT(c_CommunicationChannel_DEMO *channel, SET_POSITION_TARGET_GLOBAL_INT_set_position_target_global_int *pset_position_target_global_int) {
	
	CURSORS(curs);
	read_SET_POSITION_TARGET_GLOBAL_INT(wrap_pack(pset_position_target_global_int, curs));
	
}

static inline void CommunicationChannel_DEMO_on_VIBRATION(c_CommunicationChannel_DEMO *channel, VIBRATION_vibration *pvibration) {
	
	read_VIBRATION(pvibration->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PING33(c_CommunicationChannel_DEMO *channel, PING33_ping33 *pping33) {
	
	CURSORS(curs);
	read_PING33(wrap_pack(pping33, curs));
	
}

static inline void CommunicationChannel_DEMO_on_VFR_HUD(c_CommunicationChannel_DEMO *channel, VFR_HUD_vfr_hud *pvfr_hud) {
	
	read_VFR_HUD(pvfr_hud->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_SET_CURRENT(c_CommunicationChannel_DEMO *channel, MISSION_SET_CURRENT_mission_set_current *pmission_set_current) {
	
	read_MISSION_SET_CURRENT(pmission_set_current->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HIL_GPS(c_CommunicationChannel_DEMO *channel, HIL_GPS_hil_gps *phil_gps) {
	
	read_HIL_GPS(phil_gps->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_NAV_CONTROLLER_OUTPUT(c_CommunicationChannel_DEMO *channel, NAV_CONTROLLER_OUTPUT_nav_controller_output *pnav_controller_output) {
	
	read_NAV_CONTROLLER_OUTPUT(pnav_controller_output->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_AUTH_KEY(c_CommunicationChannel_DEMO *channel, AUTH_KEY_auth_key *pauth_key) {
	
	CURSORS(curs);
	read_AUTH_KEY(wrap_pack(pauth_key, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOCAL_POSITION_NED_COV(c_CommunicationChannel_DEMO *channel, LOCAL_POSITION_NED_COV_local_position_ned_cov *plocal_position_ned_cov) {
	
	CURSORS(curs);
	read_LOCAL_POSITION_NED_COV(wrap_pack(plocal_position_ned_cov, curs));
	
}

static inline void CommunicationChannel_DEMO_on_ATT_POS_MOCAP(c_CommunicationChannel_DEMO *channel, ATT_POS_MOCAP_att_pos_mocap *patt_pos_mocap) {
	
	read_ATT_POS_MOCAP(patt_pos_mocap->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_STATUSTEXT(c_CommunicationChannel_DEMO *channel, STATUSTEXT_statustext *pstatustext) {
	
	CURSORS(curs);
	read_STATUSTEXT(wrap_pack(pstatustext, curs));
	
}

static inline void CommunicationChannel_DEMO_on_PING(c_CommunicationChannel_DEMO *channel, PING_ping *pping) {
	
	read_PING(pping->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_CAPTURE_STATUS(c_CommunicationChannel_DEMO *channel, CAMERA_CAPTURE_STATUS_camera_capture_status *pcamera_capture_status) {
	
	read_CAMERA_CAPTURE_STATUS(pcamera_capture_status->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GLOBAL_POSITION_INT(c_CommunicationChannel_DEMO *channel, GLOBAL_POSITION_INT_global_position_int *pglobal_position_int) {
	
	read_GLOBAL_POSITION_INT(pglobal_position_int->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ENCAPSULATED_DATA(c_CommunicationChannel_DEMO *channel, ENCAPSULATED_DATA_encapsulated_data *pencapsulated_data) {
	
	read_ENCAPSULATED_DATA(pencapsulated_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS_INPUT(c_CommunicationChannel_DEMO *channel, GPS_INPUT_gps_input *pgps_input) {
	
	CURSORS(curs);
	read_GPS_INPUT(wrap_pack(pgps_input, curs));
	
}

static inline void CommunicationChannel_DEMO_on_COMMAND_LONG(c_CommunicationChannel_DEMO *channel, COMMAND_LONG_command_long *pcommand_long) {
	
	CURSORS(curs);
	read_COMMAND_LONG(wrap_pack(pcommand_long, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOG_REQUEST_DATA(c_CommunicationChannel_DEMO *channel, LOG_REQUEST_DATA_log_request_data *plog_request_data) {
	
	read_LOG_REQUEST_DATA(plog_request_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS_RAW_INT(c_CommunicationChannel_DEMO *channel, GPS_RAW_INT_gps_raw_int *pgps_raw_int) {
	
	CURSORS(curs);
	read_GPS_RAW_INT(wrap_pack(pgps_raw_int, curs));
	
}

static inline void CommunicationChannel_DEMO_on_RC_CHANNELS_SCALED(c_CommunicationChannel_DEMO *channel, RC_CHANNELS_SCALED_rc_channels_scaled *prc_channels_scaled) {
	
	read_RC_CHANNELS_SCALED(prc_channels_scaled->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_SETTINGS(c_CommunicationChannel_DEMO *channel, CAMERA_SETTINGS_camera_settings *pcamera_settings) {
	
	CURSORS(curs);
	read_CAMERA_SETTINGS(wrap_pack(pcamera_settings, curs));
	
}

static inline void CommunicationChannel_DEMO_on_RAW_PRESSURE(c_CommunicationChannel_DEMO *channel, RAW_PRESSURE_raw_pressure *praw_pressure) {
	
	read_RAW_PRESSURE(praw_pressure->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_NAMED_VALUE_FLOAT(c_CommunicationChannel_DEMO *channel, NAMED_VALUE_FLOAT_named_value_float *pnamed_value_float) {
	
	CURSORS(curs);
	read_NAMED_VALUE_FLOAT(wrap_pack(pnamed_value_float, curs));
	
}

static inline void CommunicationChannel_DEMO_on_ATTITUDE(c_CommunicationChannel_DEMO *channel, ATTITUDE_attitude *pattitude) {
	
	read_ATTITUDE(pattitude->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_TERRAIN_REQUEST(c_CommunicationChannel_DEMO *channel, TERRAIN_REQUEST_terrain_request *pterrain_request) {
	
	read_TERRAIN_REQUEST(pterrain_request->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_WRITE_PARTIAL_LIST(c_CommunicationChannel_DEMO *channel, MISSION_WRITE_PARTIAL_LIST_mission_write_partial_list *pmission_write_partial_list) {
	
	CURSORS(curs);
	read_MISSION_WRITE_PARTIAL_LIST(wrap_pack(pmission_write_partial_list, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOG_ERASE(c_CommunicationChannel_DEMO *channel, LOG_ERASE_log_erase *plog_erase) {
	
	read_LOG_ERASE(plog_erase->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MANUAL_SETPOINT(c_CommunicationChannel_DEMO *channel, MANUAL_SETPOINT_manual_setpoint *pmanual_setpoint) {
	
	read_MANUAL_SETPOINT(pmanual_setpoint->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SAFETY_ALLOWED_AREA(c_CommunicationChannel_DEMO *channel, SAFETY_ALLOWED_AREA_safety_allowed_area *psafety_allowed_area) {
	
	CURSORS(curs);
	read_SAFETY_ALLOWED_AREA(wrap_pack(psafety_allowed_area, curs));
	
}

static inline void CommunicationChannel_DEMO_on_OPTICAL_FLOW_RAD(c_CommunicationChannel_DEMO *channel, OPTICAL_FLOW_RAD_optical_flow_rad *poptical_flow_rad) {
	
	read_OPTICAL_FLOW_RAD(poptical_flow_rad->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LOG_DATA(c_CommunicationChannel_DEMO *channel, LOG_DATA_log_data *plog_data) {
	
	read_LOG_DATA(plog_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_CLEAR_ALL(c_CommunicationChannel_DEMO *channel, MISSION_CLEAR_ALL_mission_clear_all *pmission_clear_all) {
	
	CURSORS(curs);
	read_MISSION_CLEAR_ALL(wrap_pack(pmission_clear_all, curs));
	
}

static inline void CommunicationChannel_DEMO_on_VICON_POSITION_ESTIMATE(c_CommunicationChannel_DEMO *channel, VICON_POSITION_ESTIMATE_vicon_position_estimate *pvicon_position_estimate) {
	
	read_VICON_POSITION_ESTIMATE(pvicon_position_estimate->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS2_RTK(c_CommunicationChannel_DEMO *channel, GPS2_RTK_gps2_rtk *pgps2_rtk) {
	
	read_GPS2_RTK(pgps2_rtk->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LOG_REQUEST_LIST(c_CommunicationChannel_DEMO *channel, LOG_REQUEST_LIST_log_request_list *plog_request_list) {
	
	read_LOG_REQUEST_LIST(plog_request_list->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SCALED_PRESSURE(c_CommunicationChannel_DEMO *channel, SCALED_PRESSURE_scaled_pressure *pscaled_pressure) {
	
	read_SCALED_PRESSURE(pscaled_pressure->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_REQUEST_INT(c_CommunicationChannel_DEMO *channel, MISSION_REQUEST_INT_mission_request_int *pmission_request_int) {
	
	CURSORS(curs);
	read_MISSION_REQUEST_INT(wrap_pack(pmission_request_int, curs));
	
}

static inline void CommunicationChannel_DEMO_on_V2_EXTENSION(c_CommunicationChannel_DEMO *channel, V2_EXTENSION_v2_extension *pv2_extension) {
	
	read_V2_EXTENSION(pv2_extension->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HEARTBEAT(c_CommunicationChannel_DEMO *channel, HEARTBEAT_heartbeat *pheartbeat) {
	
	CURSORS(curs);
	read_HEARTBEAT(wrap_pack(pheartbeat, curs));
	
}

static inline void CommunicationChannel_DEMO_on_PARAM_MAP_RC(c_CommunicationChannel_DEMO *channel, PARAM_MAP_RC_param_map_rc *pparam_map_rc) {
	
	CURSORS(curs);
	read_PARAM_MAP_RC(wrap_pack(pparam_map_rc, curs));
	
}

static inline void CommunicationChannel_DEMO_on_POWER_STATUS(c_CommunicationChannel_DEMO *channel, POWER_STATUS_power_status *ppower_status) {
	
	CURSORS(curs);
	read_POWER_STATUS(wrap_pack(ppower_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_TERRAIN_CHECK(c_CommunicationChannel_DEMO *channel, TERRAIN_CHECK_terrain_check *pterrain_check) {
	
	read_TERRAIN_CHECK(pterrain_check->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(c_CommunicationChannel_DEMO *channel, LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_local_position_ned_system_global_offset *plocal_position_ned_system_global_offset) {
	
	read_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_COMMAND_ACK(c_CommunicationChannel_DEMO *channel, COMMAND_ACK_command_ack *pcommand_ack) {
	
	CURSORS(curs);
	read_COMMAND_ACK(wrap_pack(pcommand_ack, curs));
	
}

static inline void CommunicationChannel_DEMO_on_DATA_STREAM(c_CommunicationChannel_DEMO *channel, DATA_STREAM_data_stream *pdata_stream) {
	
	read_DATA_STREAM(pdata_stream->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_REQUEST(c_CommunicationChannel_DEMO *channel, MISSION_REQUEST_mission_request *pmission_request) {
	
	CURSORS(curs);
	read_MISSION_REQUEST(wrap_pack(pmission_request, curs));
	
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

static inline void CommunicationChannel_DEMO_on_HIL_RC_INPUTS_RAW(c_CommunicationChannel_DEMO *channel, HIL_RC_INPUTS_RAW_hil_rc_inputs_raw *phil_rc_inputs_raw) {
	
	read_HIL_RC_INPUTS_RAW(phil_rc_inputs_raw->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SCALED_IMU3(c_CommunicationChannel_DEMO *channel, SCALED_IMU3_scaled_imu3 *pscaled_imu3) {
	
	read_SCALED_IMU3(pscaled_imu3->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SET_MODE(c_CommunicationChannel_DEMO *channel, SET_MODE_set_mode *pset_mode) {
	
	CURSORS(curs);
	read_SET_MODE(wrap_pack(pset_mode, curs));
	
}

static inline void CommunicationChannel_DEMO_on_POSITION_TARGET_GLOBAL_INT(c_CommunicationChannel_DEMO *channel, POSITION_TARGET_GLOBAL_INT_position_target_global_int *pposition_target_global_int) {
	
	CURSORS(curs);
	read_POSITION_TARGET_GLOBAL_INT(wrap_pack(pposition_target_global_int, curs));
	
}

static inline void CommunicationChannel_DEMO_on_FLIGHT_INFORMATION(c_CommunicationChannel_DEMO *channel, FLIGHT_INFORMATION_flight_information *pflight_information) {
	
	read_FLIGHT_INFORMATION(pflight_information->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SIM_STATE(c_CommunicationChannel_DEMO *channel, SIM_STATE_sim_state *psim_state) {
	
	read_SIM_STATE(psim_state->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_ITEM_REACHED(c_CommunicationChannel_DEMO *channel, MISSION_ITEM_REACHED_mission_item_reached *pmission_item_reached) {
	
	read_MISSION_ITEM_REACHED(pmission_item_reached->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_RC_CHANNELS_RAW(c_CommunicationChannel_DEMO *channel, RC_CHANNELS_RAW_rc_channels_raw *prc_channels_raw) {
	
	read_RC_CHANNELS_RAW(prc_channels_raw->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SERVO_OUTPUT_RAW(c_CommunicationChannel_DEMO *channel, SERVO_OUTPUT_RAW_servo_output_raw *pservo_output_raw) {
	
	CURSORS(curs);
	read_SERVO_OUTPUT_RAW(wrap_pack(pservo_output_raw, curs));
	
}

static inline void CommunicationChannel_DEMO_on_VISION_SPEED_ESTIMATE(c_CommunicationChannel_DEMO *channel, VISION_SPEED_ESTIMATE_vision_speed_estimate *pvision_speed_estimate) {
	
	read_VISION_SPEED_ESTIMATE(pvision_speed_estimate->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DEBUG_VECT(c_CommunicationChannel_DEMO *channel, DEBUG_VECT_debug_vect *pdebug_vect) {
	
	CURSORS(curs);
	read_DEBUG_VECT(wrap_pack(pdebug_vect, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOG_REQUEST_END(c_CommunicationChannel_DEMO *channel, LOG_REQUEST_END_log_request_end *plog_request_end) {
	
	read_LOG_REQUEST_END(plog_request_end->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_ACK(c_CommunicationChannel_DEMO *channel, MISSION_ACK_mission_ack *pmission_ack) {
	
	CURSORS(curs);
	read_MISSION_ACK(wrap_pack(pmission_ack, curs));
	
}

static inline void CommunicationChannel_DEMO_on_CHANGE_OPERATOR_CONTROL_ACK(c_CommunicationChannel_DEMO *channel, CHANGE_OPERATOR_CONTROL_ACK_change_operator_control_ack *pchange_operator_control_ack) {
	
	read_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_CURRENT(c_CommunicationChannel_DEMO *channel, MISSION_CURRENT_mission_current *pmission_current) {
	
	read_MISSION_CURRENT(pmission_current->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_SYSTEM_TIME(c_CommunicationChannel_DEMO *channel, SYSTEM_TIME_system_time *psystem_time) {
	
	read_SYSTEM_TIME(psystem_time->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_CAMERA_TRIGGER(c_CommunicationChannel_DEMO *channel, CAMERA_TRIGGER_camera_trigger *pcamera_trigger) {
	
	read_CAMERA_TRIGGER(pcamera_trigger->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_VISION_POSITION_ESTIMATE(c_CommunicationChannel_DEMO *channel, VISION_POSITION_ESTIMATE_vision_position_estimate *pvision_position_estimate) {
	
	read_VISION_POSITION_ESTIMATE(pvision_position_estimate->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MANUAL_CONTROL(c_CommunicationChannel_DEMO *channel, MANUAL_CONTROL_manual_control *pmanual_control) {
	
	read_MANUAL_CONTROL(pmanual_control->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_RC_CHANNELS(c_CommunicationChannel_DEMO *channel, RC_CHANNELS_rc_channels *prc_channels) {
	
	read_RC_CHANNELS(prc_channels->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_PARAM_VALUE(c_CommunicationChannel_DEMO *channel, PARAM_VALUE_param_value *pparam_value) {
	
	CURSORS(curs);
	read_PARAM_VALUE(wrap_pack(pparam_value, curs));
	
}

static inline void CommunicationChannel_DEMO_on_BATTERY_STATUS(c_CommunicationChannel_DEMO *channel, BATTERY_STATUS_battery_status *pbattery_status) {
	
	CURSORS(curs);
	read_BATTERY_STATUS(wrap_pack(pbattery_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SET_POSITION_TARGET_LOCAL_NED(c_CommunicationChannel_DEMO *channel, SET_POSITION_TARGET_LOCAL_NED_set_position_target_local_ned *pset_position_target_local_ned) {
	
	CURSORS(curs);
	read_SET_POSITION_TARGET_LOCAL_NED(wrap_pack(pset_position_target_local_ned, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SERIAL_CONTROL(c_CommunicationChannel_DEMO *channel, SERIAL_CONTROL_serial_control *pserial_control) {
	
	CURSORS(curs);
	read_SERIAL_CONTROL(wrap_pack(pserial_control, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SET_GPS_GLOBAL_ORIGIN(c_CommunicationChannel_DEMO *channel, SET_GPS_GLOBAL_ORIGIN_set_gps_global_origin *pset_gps_global_origin) {
	
	CURSORS(curs);
	read_SET_GPS_GLOBAL_ORIGIN(wrap_pack(pset_gps_global_origin, curs));
	
}

static inline void CommunicationChannel_DEMO_on_AUTOPILOT_VERSION(c_CommunicationChannel_DEMO *channel, AUTOPILOT_VERSION_autopilot_version *pautopilot_version) {
	
	CURSORS(curs);
	read_AUTOPILOT_VERSION(wrap_pack(pautopilot_version, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_REQUEST_LIST(c_CommunicationChannel_DEMO *channel, MISSION_REQUEST_LIST_mission_request_list *pmission_request_list) {
	
	CURSORS(curs);
	read_MISSION_REQUEST_LIST(wrap_pack(pmission_request_list, curs));
	
}

static inline void CommunicationChannel_DEMO_on_PLAY_TUNE(c_CommunicationChannel_DEMO *channel, PLAY_TUNE_play_tune *pplay_tune) {
	
	CURSORS(curs);
	read_PLAY_TUNE(wrap_pack(pplay_tune, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SCALED_PRESSURE3(c_CommunicationChannel_DEMO *channel, SCALED_PRESSURE3_scaled_pressure3 *pscaled_pressure3) {
	
	read_SCALED_PRESSURE3(pscaled_pressure3->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_REQUEST_PARTIAL_LIST(c_CommunicationChannel_DEMO *channel, MISSION_REQUEST_PARTIAL_LIST_mission_request_partial_list *pmission_request_partial_list) {
	
	CURSORS(curs);
	read_MISSION_REQUEST_PARTIAL_LIST(wrap_pack(pmission_request_partial_list, curs));
	
}

static inline void CommunicationChannel_DEMO_on_LOCAL_POSITION_NED(c_CommunicationChannel_DEMO *channel, LOCAL_POSITION_NED_local_position_ned *plocal_position_ned) {
	
	read_LOCAL_POSITION_NED(plocal_position_ned->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_DATA_TRANSMISSION_HANDSHAKE(c_CommunicationChannel_DEMO *channel, DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake *pdata_transmission_handshake) {
	
	read_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS_GLOBAL_ORIGIN(c_CommunicationChannel_DEMO *channel, GPS_GLOBAL_ORIGIN_gps_global_origin *pgps_global_origin) {
	
	CURSORS(curs);
	read_GPS_GLOBAL_ORIGIN(wrap_pack(pgps_global_origin, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SCALED_IMU2(c_CommunicationChannel_DEMO *channel, SCALED_IMU2_scaled_imu2 *pscaled_imu2) {
	
	read_SCALED_IMU2(pscaled_imu2->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ATTITUDE_QUATERNION(c_CommunicationChannel_DEMO *channel, ATTITUDE_QUATERNION_attitude_quaternion *pattitude_quaternion) {
	
	read_ATTITUDE_QUATERNION(pattitude_quaternion->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_HIL_ACTUATOR_CONTROLS(c_CommunicationChannel_DEMO *channel, HIL_ACTUATOR_CONTROLS_hil_actuator_controls *phil_actuator_controls) {
	
	CURSORS(curs);
	read_HIL_ACTUATOR_CONTROLS(wrap_pack(phil_actuator_controls, curs));
	
}

static inline void CommunicationChannel_DEMO_on_POSITION_TARGET_LOCAL_NED(c_CommunicationChannel_DEMO *channel, POSITION_TARGET_LOCAL_NED_position_target_local_ned *pposition_target_local_ned) {
	
	CURSORS(curs);
	read_POSITION_TARGET_LOCAL_NED(wrap_pack(pposition_target_local_ned, curs));
	
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

static inline void CommunicationChannel_DEMO_on_CHANGE_OPERATOR_CONTROL(c_CommunicationChannel_DEMO *channel, CHANGE_OPERATOR_CONTROL_change_operator_control *pchange_operator_control) {
	
	CURSORS(curs);
	read_CHANGE_OPERATOR_CONTROL(wrap_pack(pchange_operator_control, curs));
	
}

static inline void CommunicationChannel_DEMO_on_SYS_STATUS(c_CommunicationChannel_DEMO *channel, SYS_STATUS_sys_status *psys_status) {
	
	CURSORS(curs);
	read_SYS_STATUS(wrap_pack(psys_status, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_ITEM(c_CommunicationChannel_DEMO *channel, MISSION_ITEM_mission_item *pmission_item) {
	
	CURSORS(curs);
	read_MISSION_ITEM(wrap_pack(pmission_item, curs));
	
}

static inline void CommunicationChannel_DEMO_on_RAW_IMU(c_CommunicationChannel_DEMO *channel, RAW_IMU_raw_imu *praw_imu) {
	
	read_RAW_IMU(praw_imu->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_COMMAND_INT(c_CommunicationChannel_DEMO *channel, COMMAND_INT_command_int *pcommand_int) {
	
	CURSORS(curs);
	read_COMMAND_INT(wrap_pack(pcommand_int, curs));
	
}

static inline void CommunicationChannel_DEMO_on_OPTICAL_FLOW(c_CommunicationChannel_DEMO *channel, OPTICAL_FLOW_optical_flow *poptical_flow) {
	
	CURSORS(curs);
	read_OPTICAL_FLOW(wrap_pack(poptical_flow, curs));
	
}

static inline void CommunicationChannel_DEMO_on_MISSION_ITEM_INT(c_CommunicationChannel_DEMO *channel, MISSION_ITEM_INT_mission_item_int *pmission_item_int) {
	
	CURSORS(curs);
	read_MISSION_ITEM_INT(wrap_pack(pmission_item_int, curs));
	
}

static inline void CommunicationChannel_DEMO_on_HIGHRES_IMU(c_CommunicationChannel_DEMO *channel, HIGHRES_IMU_highres_imu *phighres_imu) {
	
	read_HIGHRES_IMU(phighres_imu->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_EXTENDED_SYS_STATE(c_CommunicationChannel_DEMO *channel, EXTENDED_SYS_STATE_extended_sys_state *pextended_sys_state) {
	
	CURSORS(curs);
	read_EXTENDED_SYS_STATE(wrap_pack(pextended_sys_state, curs));
	
}

static inline void CommunicationChannel_DEMO_on_GPS_INJECT_DATA(c_CommunicationChannel_DEMO *channel, GPS_INJECT_DATA_gps_inject_data *pgps_inject_data) {
	
	read_GPS_INJECT_DATA(pgps_inject_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_ATTITUDE_QUATERNION_COV(c_CommunicationChannel_DEMO *channel, ATTITUDE_QUATERNION_COV_attitude_quaternion_cov *pattitude_quaternion_cov) {
	
	read_ATTITUDE_QUATERNION_COV(pattitude_quaternion_cov->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_NAMED_VALUE_INT(c_CommunicationChannel_DEMO *channel, NAMED_VALUE_INT_named_value_int *pnamed_value_int) {
	
	CURSORS(curs);
	read_NAMED_VALUE_INT(wrap_pack(pnamed_value_int, curs));
	
}

static inline void CommunicationChannel_DEMO_on_RADIO_STATUS(c_CommunicationChannel_DEMO *channel, RADIO_STATUS_radio_status *pradio_status) {
	
	read_RADIO_STATUS(pradio_status->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GPS_RTCM_DATA(c_CommunicationChannel_DEMO *channel, GPS_RTCM_DATA_gps_rtcm_data *pgps_rtcm_data) {
	
	read_GPS_RTCM_DATA(pgps_rtcm_data->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_GLOBAL_VISION_POSITION_ESTIMATE(c_CommunicationChannel_DEMO *channel, GLOBAL_VISION_POSITION_ESTIMATE_global_vision_position_estimate *pglobal_vision_position_estimate) {
	
	read_GLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate->bytes);
	
}

static inline void CommunicationChannel_DEMO_on_FILE_TRANSFER_PROTOCOL(c_CommunicationChannel_DEMO *channel, FILE_TRANSFER_PROTOCOL_file_transfer_protocol *pfile_transfer_protocol) {
	
	read_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol->bytes);
	
}

static inline Meta const *CommunicationChannel_DEMO_dispatcher(Receiver *receiver, size_t id, Pack *pack) {
	c_CommunicationChannel_DEMO *channel = (c_CommunicationChannel_DEMO *) ((uint8_t *) receiver - offsetof(c_CommunicationChannel_DEMO, channel) - offsetof(c_CommunicationChannel, receiver));
	c_CommunicationChannel_DISPATCHER(CommunicationChannel_DEMO)
}


int main() {
	CURSORS(cur);
	c_CommunicationChannel_DEMO CommunicationChannel_instance = {.channel.receiver.dispatch = CommunicationChannel_DEMO_dispatcher, .channel.transmitter.pull = CommunicationChannel_DEMO_pull};
	
	{
		
		FOLLOW_TARGET_follow_target *pfollow_target = c_CommunicationChannel_new_FOLLOW_TARGET();
		write_FOLLOW_TARGET(pfollow_target_FOLLOW_TARGET_from(pfollow_target));
		if (!c_CommunicationChannel_send_FOLLOW_TARGET(&CommunicationChannel_instance.channel, pfollow_target)) {
			free_pack(pfollow_target);
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
		
		pekf_status_report_EKF_STATUS_REPORT *pekf_status_report = c_CommunicationChannel_new_EKF_STATUS_REPORT(cur);
		write_EKF_STATUS_REPORT(pekf_status_report);
		if (!c_CommunicationChannel_send_EKF_STATUS_REPORT(&CommunicationChannel_instance.channel, pekf_status_report)) {
			free_pack(unwrap_pack(pekf_status_report));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pestimator_status_ESTIMATOR_STATUS *pestimator_status = c_CommunicationChannel_new_ESTIMATOR_STATUS(cur);
		write_ESTIMATOR_STATUS(pestimator_status);
		if (!c_CommunicationChannel_send_ESTIMATOR_STATUS(&CommunicationChannel_instance.channel, pestimator_status)) {
			free_pack(unwrap_pack(pestimator_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		HWSTATUS_hwstatus *phwstatus = c_CommunicationChannel_new_HWSTATUS();
		write_HWSTATUS(phwstatus_HWSTATUS_from(phwstatus));
		if (!c_CommunicationChannel_send_HWSTATUS(&CommunicationChannel_instance.channel, phwstatus)) {
			free_pack(phwstatus);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		TIMESYNC_timesync *ptimesync = c_CommunicationChannel_new_TIMESYNC();
		write_TIMESYNC(ptimesync_TIMESYNC_from(ptimesync));
		if (!c_CommunicationChannel_send_TIMESYNC(&CommunicationChannel_instance.channel, ptimesync)) {
			free_pack(ptimesync);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		PARAM_EXT_REQUEST_LIST_param_ext_request_list *pparam_ext_request_list = c_CommunicationChannel_new_PARAM_EXT_REQUEST_LIST();
		write_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list_PARAM_EXT_REQUEST_LIST_from(pparam_ext_request_list));
		if (!c_CommunicationChannel_send_PARAM_EXT_REQUEST_LIST(&CommunicationChannel_instance.channel, pparam_ext_request_list)) {
			free_pack(pparam_ext_request_list);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		BUTTON_CHANGE_button_change *pbutton_change = c_CommunicationChannel_new_BUTTON_CHANGE();
		write_BUTTON_CHANGE(pbutton_change_BUTTON_CHANGE_from(pbutton_change));
		if (!c_CommunicationChannel_send_BUTTON_CHANGE(&CommunicationChannel_instance.channel, pbutton_change)) {
			free_pack(pbutton_change);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		puavcan_node_status_UAVCAN_NODE_STATUS *puavcan_node_status = c_CommunicationChannel_new_UAVCAN_NODE_STATUS(cur);
		write_UAVCAN_NODE_STATUS(puavcan_node_status);
		if (!c_CommunicationChannel_send_UAVCAN_NODE_STATUS(&CommunicationChannel_instance.channel, puavcan_node_status)) {
			free_pack(unwrap_pack(puavcan_node_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pcollision_COLLISION *pcollision = c_CommunicationChannel_new_COLLISION(cur);
		write_COLLISION(pcollision);
		if (!c_CommunicationChannel_send_COLLISION(&CommunicationChannel_instance.channel, pcollision)) {
			free_pack(unwrap_pack(pcollision));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		GIMBAL_TORQUE_CMD_REPORT_gimbal_torque_cmd_report *pgimbal_torque_cmd_report = c_CommunicationChannel_new_GIMBAL_TORQUE_CMD_REPORT();
		write_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report_GIMBAL_TORQUE_CMD_REPORT_from(pgimbal_torque_cmd_report));
		if (!c_CommunicationChannel_send_GIMBAL_TORQUE_CMD_REPORT(&CommunicationChannel_instance.channel, pgimbal_torque_cmd_report)) {
			free_pack(pgimbal_torque_cmd_report);
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
		
		SENSOR_OFFSETS_sensor_offsets *psensor_offsets = c_CommunicationChannel_new_SENSOR_OFFSETS();
		write_SENSOR_OFFSETS(psensor_offsets_SENSOR_OFFSETS_from(psensor_offsets));
		if (!c_CommunicationChannel_send_SENSOR_OFFSETS(&CommunicationChannel_instance.channel, psensor_offsets)) {
			free_pack(psensor_offsets);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		STORAGE_INFORMATION_storage_information *pstorage_information = c_CommunicationChannel_new_STORAGE_INFORMATION();
		write_STORAGE_INFORMATION(pstorage_information_STORAGE_INFORMATION_from(pstorage_information));
		if (!c_CommunicationChannel_send_STORAGE_INFORMATION(&CommunicationChannel_instance.channel, pstorage_information)) {
			free_pack(pstorage_information);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pcamera_information_CAMERA_INFORMATION *pcamera_information = c_CommunicationChannel_new_CAMERA_INFORMATION(cur);
		write_CAMERA_INFORMATION(pcamera_information);
		if (!c_CommunicationChannel_send_CAMERA_INFORMATION(&CommunicationChannel_instance.channel, pcamera_information)) {
			free_pack(unwrap_pack(pcamera_information));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		DEVICE_OP_WRITE_REPLY_device_op_write_reply *pdevice_op_write_reply = c_CommunicationChannel_new_DEVICE_OP_WRITE_REPLY();
		write_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply_DEVICE_OP_WRITE_REPLY_from(pdevice_op_write_reply));
		if (!c_CommunicationChannel_send_DEVICE_OP_WRITE_REPLY(&CommunicationChannel_instance.channel, pdevice_op_write_reply)) {
			free_pack(pdevice_op_write_reply);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		TERRAIN_DATA_terrain_data *pterrain_data = c_CommunicationChannel_new_TERRAIN_DATA();
		write_TERRAIN_DATA(pterrain_data_TERRAIN_DATA_from(pterrain_data));
		if (!c_CommunicationChannel_send_TERRAIN_DATA(&CommunicationChannel_instance.channel, pterrain_data)) {
			free_pack(pterrain_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		GIMBAL_CONTROL_gimbal_control *pgimbal_control = c_CommunicationChannel_new_GIMBAL_CONTROL();
		write_GIMBAL_CONTROL(pgimbal_control_GIMBAL_CONTROL_from(pgimbal_control));
		if (!c_CommunicationChannel_send_GIMBAL_CONTROL(&CommunicationChannel_instance.channel, pgimbal_control)) {
			free_pack(pgimbal_control);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pvideo_stream_information_VIDEO_STREAM_INFORMATION *pvideo_stream_information = c_CommunicationChannel_new_VIDEO_STREAM_INFORMATION(cur);
		write_VIDEO_STREAM_INFORMATION(pvideo_stream_information);
		if (!c_CommunicationChannel_send_VIDEO_STREAM_INFORMATION(&CommunicationChannel_instance.channel, pvideo_stream_information)) {
			free_pack(unwrap_pack(pvideo_stream_information));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		AHRS_ahrs *pahrs = c_CommunicationChannel_new_AHRS();
		write_AHRS(pahrs_AHRS_from(pahrs));
		if (!c_CommunicationChannel_send_AHRS(&CommunicationChannel_instance.channel, pahrs)) {
			free_pack(pahrs);
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
		
		
		phome_position_HOME_POSITION *phome_position = c_CommunicationChannel_new_HOME_POSITION(cur);
		write_HOME_POSITION(phome_position);
		if (!c_CommunicationChannel_send_HOME_POSITION(&CommunicationChannel_instance.channel, phome_position)) {
			free_pack(unwrap_pack(phome_position));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pfence_status_FENCE_STATUS *pfence_status = c_CommunicationChannel_new_FENCE_STATUS(cur);
		write_FENCE_STATUS(pfence_status);
		if (!c_CommunicationChannel_send_FENCE_STATUS(&CommunicationChannel_instance.channel, pfence_status)) {
			free_pack(unwrap_pack(pfence_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		premote_log_block_status_REMOTE_LOG_BLOCK_STATUS *premote_log_block_status = c_CommunicationChannel_new_REMOTE_LOG_BLOCK_STATUS(cur);
		write_REMOTE_LOG_BLOCK_STATUS(premote_log_block_status);
		if (!c_CommunicationChannel_send_REMOTE_LOG_BLOCK_STATUS(&CommunicationChannel_instance.channel, premote_log_block_status)) {
			free_pack(unwrap_pack(premote_log_block_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pobstacle_distance_OBSTACLE_DISTANCE *pobstacle_distance = c_CommunicationChannel_new_OBSTACLE_DISTANCE(cur);
		write_OBSTACLE_DISTANCE(pobstacle_distance);
		if (!c_CommunicationChannel_send_OBSTACLE_DISTANCE(&CommunicationChannel_instance.channel, pobstacle_distance)) {
			free_pack(unwrap_pack(pobstacle_distance));
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
		
		pparam_ext_request_read_PARAM_EXT_REQUEST_READ *pparam_ext_request_read = c_CommunicationChannel_new_PARAM_EXT_REQUEST_READ(cur);
		write_PARAM_EXT_REQUEST_READ(pparam_ext_request_read);
		if (!c_CommunicationChannel_send_PARAM_EXT_REQUEST_READ(&CommunicationChannel_instance.channel, pparam_ext_request_read)) {
			free_pack(unwrap_pack(pparam_ext_request_read));
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
		
		puavionix_adsb_out_cfg_UAVIONIX_ADSB_OUT_CFG *puavionix_adsb_out_cfg = c_CommunicationChannel_new_UAVIONIX_ADSB_OUT_CFG(cur);
		write_UAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg);
		if (!c_CommunicationChannel_send_UAVIONIX_ADSB_OUT_CFG(&CommunicationChannel_instance.channel, puavionix_adsb_out_cfg)) {
			free_pack(unwrap_pack(puavionix_adsb_out_cfg));
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
		
		DATA32_data32 *pdata32 = c_CommunicationChannel_new_DATA32();
		write_DATA32(pdata32_DATA32_from(pdata32));
		if (!c_CommunicationChannel_send_DATA32(&CommunicationChannel_instance.channel, pdata32)) {
			free_pack(pdata32);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pping33_PING33 *pping33 = c_CommunicationChannel_new_PING33(cur);
		write_PING33(pping33);
		if (!c_CommunicationChannel_send_PING33(&CommunicationChannel_instance.channel, pping33)) {
			free_pack(unwrap_pack(pping33));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		prally_point_RALLY_POINT *prally_point = c_CommunicationChannel_new_RALLY_POINT(cur);
		write_RALLY_POINT(prally_point);
		if (!c_CommunicationChannel_send_RALLY_POINT(&CommunicationChannel_instance.channel, prally_point)) {
			free_pack(unwrap_pack(prally_point));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		padap_tuning_ADAP_TUNING *padap_tuning = c_CommunicationChannel_new_ADAP_TUNING(cur);
		write_ADAP_TUNING(padap_tuning);
		if (!c_CommunicationChannel_send_ADAP_TUNING(&CommunicationChannel_instance.channel, padap_tuning)) {
			free_pack(unwrap_pack(padap_tuning));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		VIBRATION_vibration *pvibration = c_CommunicationChannel_new_VIBRATION();
		write_VIBRATION(pvibration_VIBRATION_from(pvibration));
		if (!c_CommunicationChannel_send_VIBRATION(&CommunicationChannel_instance.channel, pvibration)) {
			free_pack(pvibration);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pparam_ext_value_PARAM_EXT_VALUE *pparam_ext_value = c_CommunicationChannel_new_PARAM_EXT_VALUE(cur);
		write_PARAM_EXT_VALUE(pparam_ext_value);
		if (!c_CommunicationChannel_send_PARAM_EXT_VALUE(&CommunicationChannel_instance.channel, pparam_ext_value)) {
			free_pack(unwrap_pack(pparam_ext_value));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		BATTERY2_battery2 *pbattery2 = c_CommunicationChannel_new_BATTERY2();
		write_BATTERY2(pbattery2_BATTERY2_from(pbattery2));
		if (!c_CommunicationChannel_send_BATTERY2(&CommunicationChannel_instance.channel, pbattery2)) {
			free_pack(pbattery2);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		plimits_status_LIMITS_STATUS *plimits_status = c_CommunicationChannel_new_LIMITS_STATUS(cur);
		write_LIMITS_STATUS(plimits_status);
		if (!c_CommunicationChannel_send_LIMITS_STATUS(&CommunicationChannel_instance.channel, plimits_status)) {
			free_pack(unwrap_pack(plimits_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pcamera_feedback_CAMERA_FEEDBACK *pcamera_feedback = c_CommunicationChannel_new_CAMERA_FEEDBACK(cur);
		write_CAMERA_FEEDBACK(pcamera_feedback);
		if (!c_CommunicationChannel_send_CAMERA_FEEDBACK(&CommunicationChannel_instance.channel, pcamera_feedback)) {
			free_pack(unwrap_pack(pcamera_feedback));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		HIL_GPS_hil_gps *phil_gps = c_CommunicationChannel_new_HIL_GPS();
		write_HIL_GPS(phil_gps_HIL_GPS_from(phil_gps));
		if (!c_CommunicationChannel_send_HIL_GPS(&CommunicationChannel_instance.channel, phil_gps)) {
			free_pack(phil_gps);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		FENCE_FETCH_POINT_fence_fetch_point *pfence_fetch_point = c_CommunicationChannel_new_FENCE_FETCH_POINT();
		write_FENCE_FETCH_POINT(pfence_fetch_point_FENCE_FETCH_POINT_from(pfence_fetch_point));
		if (!c_CommunicationChannel_send_FENCE_FETCH_POINT(&CommunicationChannel_instance.channel, pfence_fetch_point)) {
			free_pack(pfence_fetch_point);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		RADIO_radio *pradio = c_CommunicationChannel_new_RADIO();
		write_RADIO(pradio_RADIO_from(pradio));
		if (!c_CommunicationChannel_send_RADIO(&CommunicationChannel_instance.channel, pradio)) {
			free_pack(pradio);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		AIRSPEED_AUTOCAL_airspeed_autocal *pairspeed_autocal = c_CommunicationChannel_new_AIRSPEED_AUTOCAL();
		write_AIRSPEED_AUTOCAL(pairspeed_autocal_AIRSPEED_AUTOCAL_from(pairspeed_autocal));
		if (!c_CommunicationChannel_send_AIRSPEED_AUTOCAL(&CommunicationChannel_instance.channel, pairspeed_autocal)) {
			free_pack(pairspeed_autocal);
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
		
		
		pgopro_get_request_GOPRO_GET_REQUEST *pgopro_get_request = c_CommunicationChannel_new_GOPRO_GET_REQUEST(cur);
		write_GOPRO_GET_REQUEST(pgopro_get_request);
		if (!c_CommunicationChannel_send_GOPRO_GET_REQUEST(&CommunicationChannel_instance.channel, pgopro_get_request)) {
			free_pack(unwrap_pack(pgopro_get_request));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		CAMERA_CAPTURE_STATUS_camera_capture_status *pcamera_capture_status = c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS();
		write_CAMERA_CAPTURE_STATUS(pcamera_capture_status_CAMERA_CAPTURE_STATUS_from(pcamera_capture_status));
		if (!c_CommunicationChannel_send_CAMERA_CAPTURE_STATUS(&CommunicationChannel_instance.channel, pcamera_capture_status)) {
			free_pack(pcamera_capture_status);
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
		
		
		COMPASSMOT_STATUS_compassmot_status *pcompassmot_status = c_CommunicationChannel_new_COMPASSMOT_STATUS();
		write_COMPASSMOT_STATUS(pcompassmot_status_COMPASSMOT_STATUS_from(pcompassmot_status));
		if (!c_CommunicationChannel_send_COMPASSMOT_STATUS(&CommunicationChannel_instance.channel, pcompassmot_status)) {
			free_pack(pcompassmot_status);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		LOG_REQUEST_DATA_log_request_data *plog_request_data = c_CommunicationChannel_new_LOG_REQUEST_DATA();
		write_LOG_REQUEST_DATA(plog_request_data_LOG_REQUEST_DATA_from(plog_request_data));
		if (!c_CommunicationChannel_send_LOG_REQUEST_DATA(&CommunicationChannel_instance.channel, plog_request_data)) {
			free_pack(plog_request_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pcamera_status_CAMERA_STATUS *pcamera_status = c_CommunicationChannel_new_CAMERA_STATUS(cur);
		write_CAMERA_STATUS(pcamera_status);
		if (!c_CommunicationChannel_send_CAMERA_STATUS(&CommunicationChannel_instance.channel, pcamera_status)) {
			free_pack(unwrap_pack(pcamera_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pcamera_settings_CAMERA_SETTINGS *pcamera_settings = c_CommunicationChannel_new_CAMERA_SETTINGS(cur);
		write_CAMERA_SETTINGS(pcamera_settings);
		if (!c_CommunicationChannel_send_CAMERA_SETTINGS(&CommunicationChannel_instance.channel, pcamera_settings)) {
			free_pack(unwrap_pack(pcamera_settings));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		DEVICE_OP_READ_REPLY_device_op_read_reply *pdevice_op_read_reply = c_CommunicationChannel_new_DEVICE_OP_READ_REPLY();
		write_DEVICE_OP_READ_REPLY(pdevice_op_read_reply_DEVICE_OP_READ_REPLY_from(pdevice_op_read_reply));
		if (!c_CommunicationChannel_send_DEVICE_OP_READ_REPLY(&CommunicationChannel_instance.channel, pdevice_op_read_reply)) {
			free_pack(pdevice_op_read_reply);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		DIGICAM_CONTROL_digicam_control *pdigicam_control = c_CommunicationChannel_new_DIGICAM_CONTROL();
		write_DIGICAM_CONTROL(pdigicam_control_DIGICAM_CONTROL_from(pdigicam_control));
		if (!c_CommunicationChannel_send_DIGICAM_CONTROL(&CommunicationChannel_instance.channel, pdigicam_control)) {
			free_pack(pdigicam_control);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pnamed_value_float_NAMED_VALUE_FLOAT *pnamed_value_float = c_CommunicationChannel_new_NAMED_VALUE_FLOAT(cur);
		write_NAMED_VALUE_FLOAT(pnamed_value_float);
		if (!c_CommunicationChannel_send_NAMED_VALUE_FLOAT(&CommunicationChannel_instance.channel, pnamed_value_float)) {
			free_pack(unwrap_pack(pnamed_value_float));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pgopro_heartbeat_GOPRO_HEARTBEAT *pgopro_heartbeat = c_CommunicationChannel_new_GOPRO_HEARTBEAT(cur);
		write_GOPRO_HEARTBEAT(pgopro_heartbeat);
		if (!c_CommunicationChannel_send_GOPRO_HEARTBEAT(&CommunicationChannel_instance.channel, pgopro_heartbeat)) {
			free_pack(unwrap_pack(pgopro_heartbeat));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		AHRS2_ahrs2 *pahrs2 = c_CommunicationChannel_new_AHRS2();
		write_AHRS2(pahrs2_AHRS2_from(pahrs2));
		if (!c_CommunicationChannel_send_AHRS2(&CommunicationChannel_instance.channel, pahrs2)) {
			free_pack(pahrs2);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		LOG_ERASE_log_erase *plog_erase = c_CommunicationChannel_new_LOG_ERASE();
		write_LOG_ERASE(plog_erase_LOG_ERASE_from(plog_erase));
		if (!c_CommunicationChannel_send_LOG_ERASE(&CommunicationChannel_instance.channel, plog_erase)) {
			free_pack(plog_erase);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		TERRAIN_REQUEST_terrain_request *pterrain_request = c_CommunicationChannel_new_TERRAIN_REQUEST();
		write_TERRAIN_REQUEST(pterrain_request_TERRAIN_REQUEST_from(pterrain_request));
		if (!c_CommunicationChannel_send_TERRAIN_REQUEST(&CommunicationChannel_instance.channel, pterrain_request)) {
			free_pack(pterrain_request);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		MOUNT_STATUS_mount_status *pmount_status = c_CommunicationChannel_new_MOUNT_STATUS();
		write_MOUNT_STATUS(pmount_status_MOUNT_STATUS_from(pmount_status));
		if (!c_CommunicationChannel_send_MOUNT_STATUS(&CommunicationChannel_instance.channel, pmount_status)) {
			free_pack(pmount_status);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		ppid_tuning_PID_TUNING *ppid_tuning = c_CommunicationChannel_new_PID_TUNING(cur);
		write_PID_TUNING(ppid_tuning);
		if (!c_CommunicationChannel_send_PID_TUNING(&CommunicationChannel_instance.channel, ppid_tuning)) {
			free_pack(unwrap_pack(ppid_tuning));
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
		
		AHRS3_ahrs3 *pahrs3 = c_CommunicationChannel_new_AHRS3();
		write_AHRS3(pahrs3_AHRS3_from(pahrs3));
		if (!c_CommunicationChannel_send_AHRS3(&CommunicationChannel_instance.channel, pahrs3)) {
			free_pack(pahrs3);
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
		
		pmag_cal_report_MAG_CAL_REPORT *pmag_cal_report = c_CommunicationChannel_new_MAG_CAL_REPORT(cur);
		write_MAG_CAL_REPORT(pmag_cal_report);
		if (!c_CommunicationChannel_send_MAG_CAL_REPORT(&CommunicationChannel_instance.channel, pmag_cal_report)) {
			free_pack(unwrap_pack(pmag_cal_report));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		LOG_REQUEST_LIST_log_request_list *plog_request_list = c_CommunicationChannel_new_LOG_REQUEST_LIST();
		write_LOG_REQUEST_LIST(plog_request_list_LOG_REQUEST_LIST_from(plog_request_list));
		if (!c_CommunicationChannel_send_LOG_REQUEST_LIST(&CommunicationChannel_instance.channel, plog_request_list)) {
			free_pack(plog_request_list);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmount_configure_MOUNT_CONFIGURE *pmount_configure = c_CommunicationChannel_new_MOUNT_CONFIGURE(cur);
		write_MOUNT_CONFIGURE(pmount_configure);
		if (!c_CommunicationChannel_send_MOUNT_CONFIGURE(&CommunicationChannel_instance.channel, pmount_configure)) {
			free_pack(unwrap_pack(pmount_configure));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		V2_EXTENSION_v2_extension *pv2_extension = c_CommunicationChannel_new_V2_EXTENSION();
		write_V2_EXTENSION(pv2_extension_V2_EXTENSION_from(pv2_extension));
		if (!c_CommunicationChannel_send_V2_EXTENSION(&CommunicationChannel_instance.channel, pv2_extension)) {
			free_pack(pv2_extension);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		ppower_status_POWER_STATUS *ppower_status = c_CommunicationChannel_new_POWER_STATUS(cur);
		write_POWER_STATUS(ppower_status);
		if (!c_CommunicationChannel_send_POWER_STATUS(&CommunicationChannel_instance.channel, ppower_status)) {
			free_pack(unwrap_pack(ppower_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		premote_log_data_block_REMOTE_LOG_DATA_BLOCK *premote_log_data_block = c_CommunicationChannel_new_REMOTE_LOG_DATA_BLOCK(cur);
		write_REMOTE_LOG_DATA_BLOCK(premote_log_data_block);
		if (!c_CommunicationChannel_send_REMOTE_LOG_DATA_BLOCK(&CommunicationChannel_instance.channel, premote_log_data_block)) {
			free_pack(unwrap_pack(premote_log_data_block));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		LOGGING_DATA_ACKED_logging_data_acked *plogging_data_acked = c_CommunicationChannel_new_LOGGING_DATA_ACKED();
		write_LOGGING_DATA_ACKED(plogging_data_acked_LOGGING_DATA_ACKED_from(plogging_data_acked));
		if (!c_CommunicationChannel_send_LOGGING_DATA_ACKED(&CommunicationChannel_instance.channel, plogging_data_acked)) {
			free_pack(plogging_data_acked);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		TERRAIN_CHECK_terrain_check *pterrain_check = c_CommunicationChannel_new_TERRAIN_CHECK();
		write_TERRAIN_CHECK(pterrain_check_TERRAIN_CHECK_from(pterrain_check));
		if (!c_CommunicationChannel_send_TERRAIN_CHECK(&CommunicationChannel_instance.channel, pterrain_check)) {
			free_pack(pterrain_check);
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
		
		SCALED_IMU3_scaled_imu3 *pscaled_imu3 = c_CommunicationChannel_new_SCALED_IMU3();
		write_SCALED_IMU3(pscaled_imu3_SCALED_IMU3_from(pscaled_imu3));
		if (!c_CommunicationChannel_send_SCALED_IMU3(&CommunicationChannel_instance.channel, pscaled_imu3)) {
			free_pack(pscaled_imu3);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		MOUNT_CONTROL_mount_control *pmount_control = c_CommunicationChannel_new_MOUNT_CONTROL();
		write_MOUNT_CONTROL(pmount_control_MOUNT_CONTROL_from(pmount_control));
		if (!c_CommunicationChannel_send_MOUNT_CONTROL(&CommunicationChannel_instance.channel, pmount_control)) {
			free_pack(pmount_control);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		LED_CONTROL_led_control *pled_control = c_CommunicationChannel_new_LED_CONTROL();
		write_LED_CONTROL(pled_control_LED_CONTROL_from(pled_control));
		if (!c_CommunicationChannel_send_LED_CONTROL(&CommunicationChannel_instance.channel, pled_control)) {
			free_pack(pled_control);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SIM_STATE_sim_state *psim_state = c_CommunicationChannel_new_SIM_STATE();
		write_SIM_STATE(psim_state_SIM_STATE_from(psim_state));
		if (!c_CommunicationChannel_send_SIM_STATE(&CommunicationChannel_instance.channel, psim_state)) {
			free_pack(psim_state);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pwifi_config_ap_WIFI_CONFIG_AP *pwifi_config_ap = c_CommunicationChannel_new_WIFI_CONFIG_AP(cur);
		write_WIFI_CONFIG_AP(pwifi_config_ap);
		if (!c_CommunicationChannel_send_WIFI_CONFIG_AP(&CommunicationChannel_instance.channel, pwifi_config_ap)) {
			free_pack(unwrap_pack(pwifi_config_ap));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		DATA96_data96 *pdata96 = c_CommunicationChannel_new_DATA96();
		write_DATA96(pdata96_DATA96_from(pdata96));
		if (!c_CommunicationChannel_send_DATA96(&CommunicationChannel_instance.channel, pdata96)) {
			free_pack(pdata96);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		FLIGHT_INFORMATION_flight_information *pflight_information = c_CommunicationChannel_new_FLIGHT_INFORMATION();
		write_FLIGHT_INFORMATION(pflight_information_FLIGHT_INFORMATION_from(pflight_information));
		if (!c_CommunicationChannel_send_FLIGHT_INFORMATION(&CommunicationChannel_instance.channel, pflight_information)) {
			free_pack(pflight_information);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pmeminfo_MEMINFO *pmeminfo = c_CommunicationChannel_new_MEMINFO(cur);
		write_MEMINFO(pmeminfo);
		if (!c_CommunicationChannel_send_MEMINFO(&CommunicationChannel_instance.channel, pmeminfo)) {
			free_pack(unwrap_pack(pmeminfo));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		LOGGING_ACK_logging_ack *plogging_ack = c_CommunicationChannel_new_LOGGING_ACK();
		write_LOGGING_ACK(plogging_ack_LOGGING_ACK_from(plogging_ack));
		if (!c_CommunicationChannel_send_LOGGING_ACK(&CommunicationChannel_instance.channel, plogging_ack)) {
			free_pack(plogging_ack);
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
		
		
		CAMERA_TRIGGER_camera_trigger *pcamera_trigger = c_CommunicationChannel_new_CAMERA_TRIGGER();
		write_CAMERA_TRIGGER(pcamera_trigger_CAMERA_TRIGGER_from(pcamera_trigger));
		if (!c_CommunicationChannel_send_CAMERA_TRIGGER(&CommunicationChannel_instance.channel, pcamera_trigger)) {
			free_pack(pcamera_trigger);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		LOG_REQUEST_END_log_request_end *plog_request_end = c_CommunicationChannel_new_LOG_REQUEST_END();
		write_LOG_REQUEST_END(plog_request_end_LOG_REQUEST_END_from(plog_request_end));
		if (!c_CommunicationChannel_send_LOG_REQUEST_END(&CommunicationChannel_instance.channel, plog_request_end)) {
			free_pack(plog_request_end);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pgopro_set_response_GOPRO_SET_RESPONSE *pgopro_set_response = c_CommunicationChannel_new_GOPRO_SET_RESPONSE(cur);
		write_GOPRO_SET_RESPONSE(pgopro_set_response);
		if (!c_CommunicationChannel_send_GOPRO_SET_RESPONSE(&CommunicationChannel_instance.channel, pgopro_set_response)) {
			free_pack(unwrap_pack(pgopro_set_response));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		PROTOCOL_VERSION_protocol_version *pprotocol_version = c_CommunicationChannel_new_PROTOCOL_VERSION();
		write_PROTOCOL_VERSION(pprotocol_version_PROTOCOL_VERSION_from(pprotocol_version));
		if (!c_CommunicationChannel_send_PROTOCOL_VERSION(&CommunicationChannel_instance.channel, pprotocol_version)) {
			free_pack(pprotocol_version);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		RALLY_FETCH_POINT_rally_fetch_point *prally_fetch_point = c_CommunicationChannel_new_RALLY_FETCH_POINT();
		write_RALLY_FETCH_POINT(prally_fetch_point_RALLY_FETCH_POINT_from(prally_fetch_point));
		if (!c_CommunicationChannel_send_RALLY_FETCH_POINT(&CommunicationChannel_instance.channel, prally_fetch_point)) {
			free_pack(prally_fetch_point);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pbattery_status_BATTERY_STATUS *pbattery_status = c_CommunicationChannel_new_BATTERY_STATUS(cur);
		write_BATTERY_STATUS(pbattery_status);
		if (!c_CommunicationChannel_send_BATTERY_STATUS(&CommunicationChannel_instance.channel, pbattery_status)) {
			free_pack(unwrap_pack(pbattery_status));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		MOUNT_ORIENTATION_mount_orientation *pmount_orientation = c_CommunicationChannel_new_MOUNT_ORIENTATION();
		write_MOUNT_ORIENTATION(pmount_orientation_MOUNT_ORIENTATION_from(pmount_orientation));
		if (!c_CommunicationChannel_send_MOUNT_ORIENTATION(&CommunicationChannel_instance.channel, pmount_orientation)) {
			free_pack(pmount_orientation);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pserial_control_SERIAL_CONTROL *pserial_control = c_CommunicationChannel_new_SERIAL_CONTROL(cur);
		write_SERIAL_CONTROL(pserial_control);
		if (!c_CommunicationChannel_send_SERIAL_CONTROL(&CommunicationChannel_instance.channel, pserial_control)) {
			free_pack(unwrap_pack(pserial_control));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pparam_ext_set_PARAM_EXT_SET *pparam_ext_set = c_CommunicationChannel_new_PARAM_EXT_SET(cur);
		write_PARAM_EXT_SET(pparam_ext_set);
		if (!c_CommunicationChannel_send_PARAM_EXT_SET(&CommunicationChannel_instance.channel, pparam_ext_set)) {
			free_pack(unwrap_pack(pparam_ext_set));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pautopilot_version_AUTOPILOT_VERSION *pautopilot_version = c_CommunicationChannel_new_AUTOPILOT_VERSION(cur);
		write_AUTOPILOT_VERSION(pautopilot_version);
		if (!c_CommunicationChannel_send_AUTOPILOT_VERSION(&CommunicationChannel_instance.channel, pautopilot_version)) {
			free_pack(unwrap_pack(pautopilot_version));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		SIMSTATE_simstate *psimstate = c_CommunicationChannel_new_SIMSTATE();
		write_SIMSTATE(psimstate_SIMSTATE_from(psimstate));
		if (!c_CommunicationChannel_send_SIMSTATE(&CommunicationChannel_instance.channel, psimstate)) {
			free_pack(psimstate);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pset_video_stream_settings_SET_VIDEO_STREAM_SETTINGS *pset_video_stream_settings = c_CommunicationChannel_new_SET_VIDEO_STREAM_SETTINGS(cur);
		write_SET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings);
		if (!c_CommunicationChannel_send_SET_VIDEO_STREAM_SETTINGS(&CommunicationChannel_instance.channel, pset_video_stream_settings)) {
			free_pack(unwrap_pack(pset_video_stream_settings));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pplay_tune_PLAY_TUNE *pplay_tune = c_CommunicationChannel_new_PLAY_TUNE(cur);
		write_PLAY_TUNE(pplay_tune);
		if (!c_CommunicationChannel_send_PLAY_TUNE(&CommunicationChannel_instance.channel, pplay_tune)) {
			free_pack(unwrap_pack(pplay_tune));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		DIGICAM_CONFIGURE_digicam_configure *pdigicam_configure = c_CommunicationChannel_new_DIGICAM_CONFIGURE();
		write_DIGICAM_CONFIGURE(pdigicam_configure_DIGICAM_CONFIGURE_from(pdigicam_configure));
		if (!c_CommunicationChannel_send_DIGICAM_CONFIGURE(&CommunicationChannel_instance.channel, pdigicam_configure)) {
			free_pack(pdigicam_configure);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SCALED_PRESSURE3_scaled_pressure3 *pscaled_pressure3 = c_CommunicationChannel_new_SCALED_PRESSURE3();
		write_SCALED_PRESSURE3(pscaled_pressure3_SCALED_PRESSURE3_from(pscaled_pressure3));
		if (!c_CommunicationChannel_send_SCALED_PRESSURE3(&CommunicationChannel_instance.channel, pscaled_pressure3)) {
			free_pack(pscaled_pressure3);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pparam_ext_ack_PARAM_EXT_ACK *pparam_ext_ack = c_CommunicationChannel_new_PARAM_EXT_ACK(cur);
		write_PARAM_EXT_ACK(pparam_ext_ack);
		if (!c_CommunicationChannel_send_PARAM_EXT_ACK(&CommunicationChannel_instance.channel, pparam_ext_ack)) {
			free_pack(unwrap_pack(pparam_ext_ack));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		puavcan_node_info_UAVCAN_NODE_INFO *puavcan_node_info = c_CommunicationChannel_new_UAVCAN_NODE_INFO(cur);
		write_UAVCAN_NODE_INFO(puavcan_node_info);
		if (!c_CommunicationChannel_send_UAVCAN_NODE_INFO(&CommunicationChannel_instance.channel, puavcan_node_info)) {
			free_pack(unwrap_pack(puavcan_node_info));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		DATA16_data16 *pdata16 = c_CommunicationChannel_new_DATA16();
		write_DATA16(pdata16_DATA16_from(pdata16));
		if (!c_CommunicationChannel_send_DATA16(&CommunicationChannel_instance.channel, pdata16)) {
			free_pack(pdata16);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SET_MAG_OFFSETS_set_mag_offsets *pset_mag_offsets = c_CommunicationChannel_new_SET_MAG_OFFSETS();
		write_SET_MAG_OFFSETS(pset_mag_offsets_SET_MAG_OFFSETS_from(pset_mag_offsets));
		if (!c_CommunicationChannel_send_SET_MAG_OFFSETS(&CommunicationChannel_instance.channel, pset_mag_offsets)) {
			free_pack(pset_mag_offsets);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		SCALED_IMU2_scaled_imu2 *pscaled_imu2 = c_CommunicationChannel_new_SCALED_IMU2();
		write_SCALED_IMU2(pscaled_imu2_SCALED_IMU2_from(pscaled_imu2));
		if (!c_CommunicationChannel_send_SCALED_IMU2(&CommunicationChannel_instance.channel, pscaled_imu2)) {
			free_pack(pscaled_imu2);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		AP_ADC_ap_adc *pap_adc = c_CommunicationChannel_new_AP_ADC();
		write_AP_ADC(pap_adc_AP_ADC_from(pap_adc));
		if (!c_CommunicationChannel_send_AP_ADC(&CommunicationChannel_instance.channel, pap_adc)) {
			free_pack(pap_adc);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		WIND_wind *pwind = c_CommunicationChannel_new_WIND();
		write_WIND(pwind_WIND_from(pwind));
		if (!c_CommunicationChannel_send_WIND(&CommunicationChannel_instance.channel, pwind)) {
			free_pack(pwind);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		AUTOPILOT_VERSION_REQUEST_autopilot_version_request *pautopilot_version_request = c_CommunicationChannel_new_AUTOPILOT_VERSION_REQUEST();
		write_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request_AUTOPILOT_VERSION_REQUEST_from(pautopilot_version_request));
		if (!c_CommunicationChannel_send_AUTOPILOT_VERSION_REQUEST(&CommunicationChannel_instance.channel, pautopilot_version_request)) {
			free_pack(pautopilot_version_request);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		DATA_TRANSMISSION_HANDSHAKE_data_transmission_handshake *pdata_transmission_handshake = c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE();
		write_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake_DATA_TRANSMISSION_HANDSHAKE_from(pdata_transmission_handshake));
		if (!c_CommunicationChannel_send_DATA_TRANSMISSION_HANDSHAKE(&CommunicationChannel_instance.channel, pdata_transmission_handshake)) {
			free_pack(pdata_transmission_handshake);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		DATA64_data64 *pdata64 = c_CommunicationChannel_new_DATA64();
		write_DATA64(pdata64_DATA64_from(pdata64));
		if (!c_CommunicationChannel_send_DATA64(&CommunicationChannel_instance.channel, pdata64)) {
			free_pack(pdata64);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		GIMBAL_REPORT_gimbal_report *pgimbal_report = c_CommunicationChannel_new_GIMBAL_REPORT();
		write_GIMBAL_REPORT(pgimbal_report_GIMBAL_REPORT_from(pgimbal_report));
		if (!c_CommunicationChannel_send_GIMBAL_REPORT(&CommunicationChannel_instance.channel, pgimbal_report)) {
			free_pack(pgimbal_report);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pdevice_op_write_DEVICE_OP_WRITE *pdevice_op_write = c_CommunicationChannel_new_DEVICE_OP_WRITE(cur);
		write_DEVICE_OP_WRITE(pdevice_op_write);
		if (!c_CommunicationChannel_send_DEVICE_OP_WRITE(&CommunicationChannel_instance.channel, pdevice_op_write)) {
			free_pack(unwrap_pack(pdevice_op_write));
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
		
		pgopro_set_request_GOPRO_SET_REQUEST *pgopro_set_request = c_CommunicationChannel_new_GOPRO_SET_REQUEST(cur);
		write_GOPRO_SET_REQUEST(pgopro_set_request);
		if (!c_CommunicationChannel_send_GOPRO_SET_REQUEST(&CommunicationChannel_instance.channel, pgopro_set_request)) {
			free_pack(unwrap_pack(pgopro_set_request));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		VISION_POSITION_DELTA_vision_position_delta *pvision_position_delta = c_CommunicationChannel_new_VISION_POSITION_DELTA();
		write_VISION_POSITION_DELTA(pvision_position_delta_VISION_POSITION_DELTA_from(pvision_position_delta));
		if (!c_CommunicationChannel_send_VISION_POSITION_DELTA(&CommunicationChannel_instance.channel, pvision_position_delta)) {
			free_pack(pvision_position_delta);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		LOGGING_DATA_logging_data *plogging_data = c_CommunicationChannel_new_LOGGING_DATA();
		write_LOGGING_DATA(plogging_data_LOGGING_DATA_from(plogging_data));
		if (!c_CommunicationChannel_send_LOGGING_DATA(&CommunicationChannel_instance.channel, plogging_data)) {
			free_pack(plogging_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		pdevice_op_read_DEVICE_OP_READ *pdevice_op_read = c_CommunicationChannel_new_DEVICE_OP_READ(cur);
		write_DEVICE_OP_READ(pdevice_op_read);
		if (!c_CommunicationChannel_send_DEVICE_OP_READ(&CommunicationChannel_instance.channel, pdevice_op_read)) {
			free_pack(unwrap_pack(pdevice_op_read));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pmag_cal_progress_MAG_CAL_PROGRESS *pmag_cal_progress = c_CommunicationChannel_new_MAG_CAL_PROGRESS(cur);
		write_MAG_CAL_PROGRESS(pmag_cal_progress);
		if (!c_CommunicationChannel_send_MAG_CAL_PROGRESS(&CommunicationChannel_instance.channel, pmag_cal_progress)) {
			free_pack(unwrap_pack(pmag_cal_progress));
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
		
		
		puavionix_adsb_out_dynamic_UAVIONIX_ADSB_OUT_DYNAMIC *puavionix_adsb_out_dynamic = c_CommunicationChannel_new_UAVIONIX_ADSB_OUT_DYNAMIC(cur);
		write_UAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic);
		if (!c_CommunicationChannel_send_UAVIONIX_ADSB_OUT_DYNAMIC(&CommunicationChannel_instance.channel, puavionix_adsb_out_dynamic)) {
			free_pack(unwrap_pack(puavionix_adsb_out_dynamic));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pgopro_get_response_GOPRO_GET_RESPONSE *pgopro_get_response = c_CommunicationChannel_new_GOPRO_GET_RESPONSE(cur);
		write_GOPRO_GET_RESPONSE(pgopro_get_response);
		if (!c_CommunicationChannel_send_GOPRO_GET_RESPONSE(&CommunicationChannel_instance.channel, pgopro_get_response)) {
			free_pack(unwrap_pack(pgopro_get_response));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		GPS_INJECT_DATA_gps_inject_data *pgps_inject_data = c_CommunicationChannel_new_GPS_INJECT_DATA();
		write_GPS_INJECT_DATA(pgps_inject_data_GPS_INJECT_DATA_from(pgps_inject_data));
		if (!c_CommunicationChannel_send_GPS_INJECT_DATA(&CommunicationChannel_instance.channel, pgps_inject_data)) {
			free_pack(pgps_inject_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		puavionix_adsb_transceiver_health_report_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT *puavionix_adsb_transceiver_health_report = c_CommunicationChannel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur);
		write_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report);
		if (!c_CommunicationChannel_send_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(&CommunicationChannel_instance.channel, puavionix_adsb_transceiver_health_report)) {
			free_pack(unwrap_pack(puavionix_adsb_transceiver_health_report));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		pnamed_value_int_NAMED_VALUE_INT *pnamed_value_int = c_CommunicationChannel_new_NAMED_VALUE_INT(cur);
		write_NAMED_VALUE_INT(pnamed_value_int);
		if (!c_CommunicationChannel_send_NAMED_VALUE_INT(&CommunicationChannel_instance.channel, pnamed_value_int)) {
			free_pack(unwrap_pack(pnamed_value_int));
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		
		RPM_rpm *prpm = c_CommunicationChannel_new_RPM();
		write_RPM(prpm_RPM_from(prpm));
		if (!c_CommunicationChannel_send_RPM(&CommunicationChannel_instance.channel, prpm)) {
			free_pack(prpm);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		GPS_RTCM_DATA_gps_rtcm_data *pgps_rtcm_data = c_CommunicationChannel_new_GPS_RTCM_DATA();
		write_GPS_RTCM_DATA(pgps_rtcm_data_GPS_RTCM_DATA_from(pgps_rtcm_data));
		if (!c_CommunicationChannel_send_GPS_RTCM_DATA(&CommunicationChannel_instance.channel, pgps_rtcm_data)) {
			free_pack(pgps_rtcm_data);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		FILE_TRANSFER_PROTOCOL_file_transfer_protocol *pfile_transfer_protocol = c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL();
		write_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol_FILE_TRANSFER_PROTOCOL_from(pfile_transfer_protocol));
		if (!c_CommunicationChannel_send_FILE_TRANSFER_PROTOCOL(&CommunicationChannel_instance.channel, pfile_transfer_protocol)) {
			free_pack(pfile_transfer_protocol);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		RANGEFINDER_rangefinder *prangefinder = c_CommunicationChannel_new_RANGEFINDER();
		write_RANGEFINDER(prangefinder_RANGEFINDER_from(prangefinder));
		if (!c_CommunicationChannel_send_RANGEFINDER(&CommunicationChannel_instance.channel, prangefinder)) {
			free_pack(prangefinder);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		RADIO_STATUS_radio_status *pradio_status = c_CommunicationChannel_new_RADIO_STATUS();
		write_RADIO_STATUS(pradio_status_RADIO_STATUS_from(pradio_status));
		if (!c_CommunicationChannel_send_RADIO_STATUS(&CommunicationChannel_instance.channel, pradio_status)) {
			free_pack(pradio_status);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		FENCE_POINT_fence_point *pfence_point = c_CommunicationChannel_new_FENCE_POINT();
		write_FENCE_POINT(pfence_point_FENCE_POINT_from(pfence_point));
		if (!c_CommunicationChannel_send_FENCE_POINT(&CommunicationChannel_instance.channel, pfence_point)) {
			free_pack(pfence_point);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		RESOURCE_REQUEST_resource_request *presource_request = c_CommunicationChannel_new_RESOURCE_REQUEST();
		write_RESOURCE_REQUEST(presource_request_RESOURCE_REQUEST_from(presource_request));
		if (!c_CommunicationChannel_send_RESOURCE_REQUEST(&CommunicationChannel_instance.channel, presource_request)) {
			free_pack(presource_request);
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
	}
	
	
	
	
	
	// c_CommunicationChannel_packs_into_bytes(uint8_t* dst, size_t bytes) for sending out packs
	
	//c_CommunicationChannel_bytes_into_packs(uint8_t* src, size_t bytes) for receiving packs
	
	
	
	return 0;
}
					  