#include "GroundControl.hpp"
#include <cassert>

static float       some_float   = 0;
static int32_t     some_int32_t = 0;
static size_t      some_size_t  = 0;
static std::string some_string  = "nullptr";
static int8_t      some_int8_t  = 0;
static bool        some_bool    = true;
static int64_t     some_int64_t = 0;
static int16_t     some_int16_t = 0;

using namespace com::company::demo;

void fill(const RESOURCE_REQUEST &presource_request);


void on_RESOURCE_REQUEST(const RESOURCE_REQUEST &presource_request);


void fill(const FENCE_POINT &pfence_point);


void on_FENCE_POINT(const FENCE_POINT &pfence_point);


void fill(const RADIO_STATUS &pradio_status);


void on_RADIO_STATUS(const RADIO_STATUS &pradio_status);


void fill(const RANGEFINDER &prangefinder);


void on_RANGEFINDER(const RANGEFINDER &prangefinder);


void fill(const FILE_TRANSFER_PROTOCOL &pfile_transfer_protocol);


void on_FILE_TRANSFER_PROTOCOL(const FILE_TRANSFER_PROTOCOL &pfile_transfer_protocol);


void on_GLOBAL_VISION_POSITION_ESTIMATE(const GLOBAL_VISION_POSITION_ESTIMATE &pglobal_vision_position_estimate);


void fill(const GPS_RTCM_DATA &pgps_rtcm_data);


void on_GPS_RTCM_DATA(const GPS_RTCM_DATA &pgps_rtcm_data);


void fill(const RPM &prpm);


void on_RPM(const RPM &prpm);


void fill(const NAMED_VALUE_INT &pnamed_value_int);


void on_NAMED_VALUE_INT(const NAMED_VALUE_INT &pnamed_value_int);


void on_ATTITUDE_QUATERNION_COV(const ATTITUDE_QUATERNION_COV &pattitude_quaternion_cov);


void fill(const UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT &puavionix_adsb_transceiver_health_report);


void on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(const UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT &puavionix_adsb_transceiver_health_report);


void fill(const GPS_INJECT_DATA &pgps_inject_data);


void on_GPS_INJECT_DATA(const GPS_INJECT_DATA &pgps_inject_data);


void fill(const GOPRO_GET_RESPONSE &pgopro_get_response);


void on_GOPRO_GET_RESPONSE(const GOPRO_GET_RESPONSE &pgopro_get_response);


void fill(const UAVIONIX_ADSB_OUT_DYNAMIC &puavionix_adsb_out_dynamic);


void on_UAVIONIX_ADSB_OUT_DYNAMIC(const UAVIONIX_ADSB_OUT_DYNAMIC &puavionix_adsb_out_dynamic);


void fill(const EXTENDED_SYS_STATE &pextended_sys_state);


void on_EXTENDED_SYS_STATE(const EXTENDED_SYS_STATE &pextended_sys_state);


void fill(const HIGHRES_IMU &phighres_imu);


void on_HIGHRES_IMU(const HIGHRES_IMU &phighres_imu);


void fill(const MAG_CAL_PROGRESS &pmag_cal_progress);


void on_MAG_CAL_PROGRESS(const MAG_CAL_PROGRESS &pmag_cal_progress);


void fill(const DEVICE_OP_READ &pdevice_op_read);


void on_DEVICE_OP_READ(const DEVICE_OP_READ &pdevice_op_read);


void fill(const LOGGING_DATA &plogging_data);


void on_LOGGING_DATA(const LOGGING_DATA &plogging_data);


void fill(const VISION_POSITION_DELTA &pvision_position_delta);


void on_VISION_POSITION_DELTA(const VISION_POSITION_DELTA &pvision_position_delta);


void on_MISSION_ITEM_INT(const MISSION_ITEM_INT &pmission_item_int);


void on_OPTICAL_FLOW(const OPTICAL_FLOW &poptical_flow);


void on_COMMAND_INT(const COMMAND_INT &pcommand_int);


void on_RAW_IMU(const RAW_IMU &praw_imu);


void on_MISSION_ITEM(const MISSION_ITEM &pmission_item);


void on_SYS_STATUS(const SYS_STATUS &psys_status);


void fill(const GOPRO_SET_REQUEST &pgopro_set_request);


void on_GOPRO_SET_REQUEST(const GOPRO_SET_REQUEST &pgopro_set_request);


void on_CHANGE_OPERATOR_CONTROL(const CHANGE_OPERATOR_CONTROL &pchange_operator_control);


void fill(const WIND_COV &pwind_cov);


void on_WIND_COV(const WIND_COV &pwind_cov);


void fill(const SCALED_PRESSURE2 &pscaled_pressure2);


void on_SCALED_PRESSURE2(const SCALED_PRESSURE2 &pscaled_pressure2);


void fill(const HIL_OPTICAL_FLOW &phil_optical_flow);


void on_HIL_OPTICAL_FLOW(const HIL_OPTICAL_FLOW &phil_optical_flow);


void fill(const DISTANCE_SENSOR &pdistance_sensor);


void on_DISTANCE_SENSOR(const DISTANCE_SENSOR &pdistance_sensor);


void fill(const DEVICE_OP_WRITE &pdevice_op_write);


void on_DEVICE_OP_WRITE(const DEVICE_OP_WRITE &pdevice_op_write);


void fill(const GIMBAL_REPORT &pgimbal_report);


void on_GIMBAL_REPORT(const GIMBAL_REPORT &pgimbal_report);


void on_POSITION_TARGET_LOCAL_NED(const POSITION_TARGET_LOCAL_NED &pposition_target_local_ned);


void on_HIL_ACTUATOR_CONTROLS(const HIL_ACTUATOR_CONTROLS &phil_actuator_controls);


void fill(const DATA64 &pdata64);


void on_DATA64(const DATA64 &pdata64);


void on_ATTITUDE_QUATERNION(const ATTITUDE_QUATERNION &pattitude_quaternion);


void fill(const SCALED_IMU2 &pscaled_imu2);


void on_SCALED_IMU2(const SCALED_IMU2 &pscaled_imu2);


void on_GPS_GLOBAL_ORIGIN(const GPS_GLOBAL_ORIGIN &pgps_global_origin);


void fill(const DATA_TRANSMISSION_HANDSHAKE &pdata_transmission_handshake);


void on_DATA_TRANSMISSION_HANDSHAKE(const DATA_TRANSMISSION_HANDSHAKE &pdata_transmission_handshake);


void on_LOCAL_POSITION_NED(const LOCAL_POSITION_NED &plocal_position_ned);


void fill(const AUTOPILOT_VERSION_REQUEST &pautopilot_version_request);


void on_AUTOPILOT_VERSION_REQUEST(const AUTOPILOT_VERSION_REQUEST &pautopilot_version_request);


void fill(const WIND &pwind);


void on_WIND(const WIND &pwind);


void fill(const AP_ADC &pap_adc);


void on_AP_ADC(const AP_ADC &pap_adc);


void fill(const SET_MAG_OFFSETS &pset_mag_offsets);


void on_SET_MAG_OFFSETS(const SET_MAG_OFFSETS &pset_mag_offsets);


void fill(const DATA16 &pdata16);


void on_DATA16(const DATA16 &pdata16);


void fill(const UAVCAN_NODE_INFO &puavcan_node_info);


void on_UAVCAN_NODE_INFO(const UAVCAN_NODE_INFO &puavcan_node_info);


void fill(const PARAM_EXT_ACK &pparam_ext_ack);


void on_PARAM_EXT_ACK(const PARAM_EXT_ACK &pparam_ext_ack);


void on_MISSION_REQUEST_PARTIAL_LIST(const MISSION_REQUEST_PARTIAL_LIST &pmission_request_partial_list);


void fill(const SCALED_PRESSURE3 &pscaled_pressure3);


void on_SCALED_PRESSURE3(const SCALED_PRESSURE3 &pscaled_pressure3);


void fill(const DIGICAM_CONFIGURE &pdigicam_configure);


void on_DIGICAM_CONFIGURE(const DIGICAM_CONFIGURE &pdigicam_configure);


void fill(const PLAY_TUNE &pplay_tune);


void on_PLAY_TUNE(const PLAY_TUNE &pplay_tune);


void fill(const SET_VIDEO_STREAM_SETTINGS &pset_video_stream_settings);


void on_SET_VIDEO_STREAM_SETTINGS(const SET_VIDEO_STREAM_SETTINGS &pset_video_stream_settings);


void fill(const SIMSTATE &psimstate);


void on_SIMSTATE(const SIMSTATE &psimstate);


void on_MISSION_REQUEST_LIST(const MISSION_REQUEST_LIST &pmission_request_list);


void fill(const AUTOPILOT_VERSION &pautopilot_version);


void on_AUTOPILOT_VERSION(const AUTOPILOT_VERSION &pautopilot_version);


void fill(const PARAM_EXT_SET &pparam_ext_set);


void on_PARAM_EXT_SET(const PARAM_EXT_SET &pparam_ext_set);


void on_SET_GPS_GLOBAL_ORIGIN(const SET_GPS_GLOBAL_ORIGIN &pset_gps_global_origin);


void fill(const MOUNT_ORIENTATION &pmount_orientation);


void on_MOUNT_ORIENTATION(const MOUNT_ORIENTATION &pmount_orientation);


void on_SET_POSITION_TARGET_LOCAL_NED(const SET_POSITION_TARGET_LOCAL_NED &pset_position_target_local_ned);


void fill(const SERIAL_CONTROL &pserial_control);


void on_SERIAL_CONTROL(const SERIAL_CONTROL &pserial_control);


void fill(const BATTERY_STATUS &pbattery_status);


void on_BATTERY_STATUS(const BATTERY_STATUS &pbattery_status);


void on_PARAM_VALUE(const PARAM_VALUE &pparam_value);


void fill(const RALLY_FETCH_POINT &prally_fetch_point);


void on_RALLY_FETCH_POINT(const RALLY_FETCH_POINT &prally_fetch_point);


void fill(const PROTOCOL_VERSION &pprotocol_version);


void on_PROTOCOL_VERSION(const PROTOCOL_VERSION &pprotocol_version);


void on_RC_CHANNELS(const RC_CHANNELS &prc_channels);


void on_MANUAL_CONTROL(const MANUAL_CONTROL &pmanual_control);


void on_VISION_POSITION_ESTIMATE(const VISION_POSITION_ESTIMATE &pvision_position_estimate);


void fill(const GOPRO_SET_RESPONSE &pgopro_set_response);


void on_GOPRO_SET_RESPONSE(const GOPRO_SET_RESPONSE &pgopro_set_response);


void fill(const CAMERA_TRIGGER &pcamera_trigger);


void on_CAMERA_TRIGGER(const CAMERA_TRIGGER &pcamera_trigger);


void on_SYSTEM_TIME(const SYSTEM_TIME &psystem_time);


void on_MISSION_CURRENT(const MISSION_CURRENT &pmission_current);


void on_CHANGE_OPERATOR_CONTROL_ACK(const CHANGE_OPERATOR_CONTROL_ACK &pchange_operator_control_ack);


void on_MISSION_ACK(const MISSION_ACK &pmission_ack);


void fill(const LOG_REQUEST_END &plog_request_end);


void on_LOG_REQUEST_END(const LOG_REQUEST_END &plog_request_end);


void fill(const DEBUG_VECT &pdebug_vect);


void on_DEBUG_VECT(const DEBUG_VECT &pdebug_vect);


void fill(const VISION_SPEED_ESTIMATE &pvision_speed_estimate);


void on_VISION_SPEED_ESTIMATE(const VISION_SPEED_ESTIMATE &pvision_speed_estimate);


void fill(const LOGGING_ACK &plogging_ack);


void on_LOGGING_ACK(const LOGGING_ACK &plogging_ack);


void on_MISSION_ITEM_REACHED(const MISSION_ITEM_REACHED &pmission_item_reached);


void fill(const MEMINFO &pmeminfo);


void on_MEMINFO(const MEMINFO &pmeminfo);


void on_SERVO_OUTPUT_RAW(const SERVO_OUTPUT_RAW &pservo_output_raw);


void on_RC_CHANNELS_RAW(const RC_CHANNELS_RAW &prc_channels_raw);


void fill(const FLIGHT_INFORMATION &pflight_information);


void on_FLIGHT_INFORMATION(const FLIGHT_INFORMATION &pflight_information);


void fill(const DATA96 &pdata96);


void on_DATA96(const DATA96 &pdata96);


void fill(const WIFI_CONFIG_AP &pwifi_config_ap);


void on_WIFI_CONFIG_AP(const WIFI_CONFIG_AP &pwifi_config_ap);


void fill(const SIM_STATE &psim_state);


void on_SIM_STATE(const SIM_STATE &psim_state);


void fill(const LED_CONTROL &pled_control);


void on_LED_CONTROL(const LED_CONTROL &pled_control);


void on_POSITION_TARGET_GLOBAL_INT(const POSITION_TARGET_GLOBAL_INT &pposition_target_global_int);


void fill(const MOUNT_CONTROL &pmount_control);


void on_MOUNT_CONTROL(const MOUNT_CONTROL &pmount_control);


void on_SET_MODE(const SET_MODE &pset_mode);


void fill(const SCALED_IMU3 &pscaled_imu3);


void on_SCALED_IMU3(const SCALED_IMU3 &pscaled_imu3);


void on_HIL_RC_INPUTS_RAW(const HIL_RC_INPUTS_RAW &phil_rc_inputs_raw);


void fill(const SET_HOME_POSITION &pset_home_position);


void on_SET_HOME_POSITION(const SET_HOME_POSITION &pset_home_position);


void fill(const TERRAIN_REPORT &pterrain_report);


void on_TERRAIN_REPORT(const TERRAIN_REPORT &pterrain_report);


void on_MISSION_REQUEST(const MISSION_REQUEST &pmission_request);


void on_DATA_STREAM(const DATA_STREAM &pdata_stream);


void on_COMMAND_ACK(const COMMAND_ACK &pcommand_ack);


void on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(const LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET &plocal_position_ned_system_global_offset);


void on_MISSION_REQUEST_INT(const MISSION_REQUEST_INT &pmission_request_int);


void fill(const MOUNT_CONFIGURE &pmount_configure);


void on_MOUNT_CONFIGURE(const MOUNT_CONFIGURE &pmount_configure);


void fill(const TERRAIN_CHECK &pterrain_check);


void on_TERRAIN_CHECK(const TERRAIN_CHECK &pterrain_check);


void fill(const LOGGING_DATA_ACKED &plogging_data_acked);


void on_LOGGING_DATA_ACKED(const LOGGING_DATA_ACKED &plogging_data_acked);


void fill(const REMOTE_LOG_DATA_BLOCK &premote_log_data_block);


void on_REMOTE_LOG_DATA_BLOCK(const REMOTE_LOG_DATA_BLOCK &premote_log_data_block);


void fill(const POWER_STATUS &ppower_status);


void on_POWER_STATUS(const POWER_STATUS &ppower_status);


void on_PARAM_MAP_RC(const PARAM_MAP_RC &pparam_map_rc);


void on_HEARTBEAT(const HEARTBEAT &pheartbeat);


void fill(const V2_EXTENSION &pv2_extension);


void on_V2_EXTENSION(const V2_EXTENSION &pv2_extension);


void on_SCALED_PRESSURE(const SCALED_PRESSURE &pscaled_pressure);


void fill(const LOG_REQUEST_LIST &plog_request_list);


void on_LOG_REQUEST_LIST(const LOG_REQUEST_LIST &plog_request_list);


void fill(const MAG_CAL_REPORT &pmag_cal_report);


void on_MAG_CAL_REPORT(const MAG_CAL_REPORT &pmag_cal_report);


void fill(const GPS2_RTK &pgps2_rtk);


void on_GPS2_RTK(const GPS2_RTK &pgps2_rtk);


void fill(const VICON_POSITION_ESTIMATE &pvicon_position_estimate);


void on_VICON_POSITION_ESTIMATE(const VICON_POSITION_ESTIMATE &pvicon_position_estimate);


void fill(const AHRS3 &pahrs3);


void on_AHRS3(const AHRS3 &pahrs3);


void on_MISSION_CLEAR_ALL(const MISSION_CLEAR_ALL &pmission_clear_all);


void fill(const LOG_DATA &plog_data);


void on_LOG_DATA(const LOG_DATA &plog_data);


void fill(const OPTICAL_FLOW_RAD &poptical_flow_rad);


void on_OPTICAL_FLOW_RAD(const OPTICAL_FLOW_RAD &poptical_flow_rad);


void on_SAFETY_ALLOWED_AREA(const SAFETY_ALLOWED_AREA &psafety_allowed_area);


void fill(const PID_TUNING &ppid_tuning);


void on_PID_TUNING(const PID_TUNING &ppid_tuning);


void on_MANUAL_SETPOINT(const MANUAL_SETPOINT &pmanual_setpoint);


void fill(const MOUNT_STATUS &pmount_status);


void on_MOUNT_STATUS(const MOUNT_STATUS &pmount_status);


void fill(const TERRAIN_REQUEST &pterrain_request);


void on_TERRAIN_REQUEST(const TERRAIN_REQUEST &pterrain_request);


void fill(const LOG_ERASE &plog_erase);


void on_LOG_ERASE(const LOG_ERASE &plog_erase);


void fill(const AHRS2 &pahrs2);


void on_AHRS2(const AHRS2 &pahrs2);


void on_MISSION_WRITE_PARTIAL_LIST(const MISSION_WRITE_PARTIAL_LIST &pmission_write_partial_list);


void on_ATTITUDE(const ATTITUDE &pattitude);


void fill(const GOPRO_HEARTBEAT &pgopro_heartbeat);


void on_GOPRO_HEARTBEAT(const GOPRO_HEARTBEAT &pgopro_heartbeat);


void fill(const NAMED_VALUE_FLOAT &pnamed_value_float);


void on_NAMED_VALUE_FLOAT(const NAMED_VALUE_FLOAT &pnamed_value_float);


void fill(const DIGICAM_CONTROL &pdigicam_control);


void on_DIGICAM_CONTROL(const DIGICAM_CONTROL &pdigicam_control);


void on_RAW_PRESSURE(const RAW_PRESSURE &praw_pressure);


void fill(const DEVICE_OP_READ_REPLY &pdevice_op_read_reply);


void on_DEVICE_OP_READ_REPLY(const DEVICE_OP_READ_REPLY &pdevice_op_read_reply);


void fill(const CAMERA_SETTINGS &pcamera_settings);


void on_CAMERA_SETTINGS(const CAMERA_SETTINGS &pcamera_settings);


void on_RC_CHANNELS_SCALED(const RC_CHANNELS_SCALED &prc_channels_scaled);


void fill(const CAMERA_STATUS &pcamera_status);


void on_CAMERA_STATUS(const CAMERA_STATUS &pcamera_status);


void on_GPS_RAW_INT(const GPS_RAW_INT &pgps_raw_int);


void fill(const LOG_REQUEST_DATA &plog_request_data);


void on_LOG_REQUEST_DATA(const LOG_REQUEST_DATA &plog_request_data);


void fill(const COMPASSMOT_STATUS &pcompassmot_status);


void on_COMPASSMOT_STATUS(const COMPASSMOT_STATUS &pcompassmot_status);


void on_COMMAND_LONG(const COMMAND_LONG &pcommand_long);


void fill(const GPS_INPUT &pgps_input);


void on_GPS_INPUT(const GPS_INPUT &pgps_input);


void fill(const ENCAPSULATED_DATA &pencapsulated_data);


void on_ENCAPSULATED_DATA(const ENCAPSULATED_DATA &pencapsulated_data);


void on_GLOBAL_POSITION_INT(const GLOBAL_POSITION_INT &pglobal_position_int);


void fill(const CAMERA_CAPTURE_STATUS &pcamera_capture_status);


void on_CAMERA_CAPTURE_STATUS(const CAMERA_CAPTURE_STATUS &pcamera_capture_status);


void fill(const GOPRO_GET_REQUEST &pgopro_get_request);


void on_GOPRO_GET_REQUEST(const GOPRO_GET_REQUEST &pgopro_get_request);


void on_PING(const PING &pping);


void fill(const STATUSTEXT &pstatustext);


void on_STATUSTEXT(const STATUSTEXT &pstatustext);


void fill(const ATT_POS_MOCAP &patt_pos_mocap);


void on_ATT_POS_MOCAP(const ATT_POS_MOCAP &patt_pos_mocap);


void fill(const AIRSPEED_AUTOCAL &pairspeed_autocal);


void on_AIRSPEED_AUTOCAL(const AIRSPEED_AUTOCAL &pairspeed_autocal);


void on_LOCAL_POSITION_NED_COV(const LOCAL_POSITION_NED_COV &plocal_position_ned_cov);


void fill(const RADIO &pradio);


void on_RADIO(const RADIO &pradio);


void fill(const FENCE_FETCH_POINT &pfence_fetch_point);


void on_FENCE_FETCH_POINT(const FENCE_FETCH_POINT &pfence_fetch_point);


void on_AUTH_KEY(const AUTH_KEY &pauth_key);


void on_NAV_CONTROLLER_OUTPUT(const NAV_CONTROLLER_OUTPUT &pnav_controller_output);


void fill(const HIL_GPS &phil_gps);


void on_HIL_GPS(const HIL_GPS &phil_gps);


void fill(const CAMERA_FEEDBACK &pcamera_feedback);


void on_CAMERA_FEEDBACK(const CAMERA_FEEDBACK &pcamera_feedback);


void fill(const LIMITS_STATUS &plimits_status);


void on_LIMITS_STATUS(const LIMITS_STATUS &plimits_status);


void fill(const BATTERY2 &pbattery2);


void on_BATTERY2(const BATTERY2 &pbattery2);


void fill(const PARAM_EXT_VALUE &pparam_ext_value);


void on_PARAM_EXT_VALUE(const PARAM_EXT_VALUE &pparam_ext_value);


void fill(const VIBRATION &pvibration);


void on_VIBRATION(const VIBRATION &pvibration);


void fill(const ADAP_TUNING &padap_tuning);


void on_ADAP_TUNING(const ADAP_TUNING &padap_tuning);


void on_MISSION_SET_CURRENT(const MISSION_SET_CURRENT &pmission_set_current);


void fill(const RALLY_POINT &prally_point);


void on_RALLY_POINT(const RALLY_POINT &prally_point);


void on_VFR_HUD(const VFR_HUD &pvfr_hud);


void fill(const PING33 &pping33);


void on_PING33(const PING33 &pping33);


void fill(const DATA32 &pdata32);


void on_DATA32(const DATA32 &pdata32);


void on_SET_POSITION_TARGET_GLOBAL_INT(const SET_POSITION_TARGET_GLOBAL_INT &pset_position_target_global_int);


void fill(const CONTROL_SYSTEM_STATE &pcontrol_system_state);


void on_CONTROL_SYSTEM_STATE(const CONTROL_SYSTEM_STATE &pcontrol_system_state);


void fill(const SET_ACTUATOR_CONTROL_TARGET &pset_actuator_control_target);


void on_SET_ACTUATOR_CONTROL_TARGET(const SET_ACTUATOR_CONTROL_TARGET &pset_actuator_control_target);


void fill(const LANDING_TARGET &planding_target);


void on_LANDING_TARGET(const LANDING_TARGET &planding_target);


void fill(const UAVIONIX_ADSB_OUT_CFG &puavionix_adsb_out_cfg);


void on_UAVIONIX_ADSB_OUT_CFG(const UAVIONIX_ADSB_OUT_CFG &puavionix_adsb_out_cfg);


void on_PARAM_REQUEST_LIST(const PARAM_REQUEST_LIST &pparam_request_list);


void fill(const GPS_RTK &pgps_rtk);


void on_GPS_RTK(const GPS_RTK &pgps_rtk);


void fill(const SETUP_SIGNING &psetup_signing);


void on_SETUP_SIGNING(const SETUP_SIGNING &psetup_signing);


void fill(const HIL_SENSOR &phil_sensor);


void on_HIL_SENSOR(const HIL_SENSOR &phil_sensor);


void on_HIL_CONTROLS(const HIL_CONTROLS &phil_controls);


void fill(const PARAM_EXT_REQUEST_READ &pparam_ext_request_read);


void on_PARAM_EXT_REQUEST_READ(const PARAM_EXT_REQUEST_READ &pparam_ext_request_read);


void fill(const MEMORY_VECT &pmemory_vect);


void on_MEMORY_VECT(const MEMORY_VECT &pmemory_vect);


void on_REQUEST_DATA_STREAM(const REQUEST_DATA_STREAM &prequest_data_stream);


void fill(const GPS2_RAW &pgps2_raw);


void on_GPS2_RAW(const GPS2_RAW &pgps2_raw);


void fill(const OBSTACLE_DISTANCE &pobstacle_distance);


void on_OBSTACLE_DISTANCE(const OBSTACLE_DISTANCE &pobstacle_distance);


void fill(const REMOTE_LOG_BLOCK_STATUS &premote_log_block_status);


void on_REMOTE_LOG_BLOCK_STATUS(const REMOTE_LOG_BLOCK_STATUS &premote_log_block_status);


void fill(const FENCE_STATUS &pfence_status);


void on_FENCE_STATUS(const FENCE_STATUS &pfence_status);


void fill(const HOME_POSITION &phome_position);


void on_HOME_POSITION(const HOME_POSITION &phome_position);


void on_HIL_STATE(const HIL_STATE &phil_state);


void fill(const FOLLOW_TARGET &pfollow_target);


void on_FOLLOW_TARGET(const FOLLOW_TARGET &pfollow_target);


void on_SET_ATTITUDE_TARGET(const SET_ATTITUDE_TARGET &pset_attitude_target);


void on_PARAM_REQUEST_READ(const PARAM_REQUEST_READ &pparam_request_read);


void fill(const HIGH_LATENCY &phigh_latency);


void on_HIGH_LATENCY(const HIGH_LATENCY &phigh_latency);


void fill(const ACTUATOR_CONTROL_TARGET &pactuator_control_target);


void on_ACTUATOR_CONTROL_TARGET(const ACTUATOR_CONTROL_TARGET &pactuator_control_target);


void fill(const LOG_ENTRY &plog_entry);


void on_LOG_ENTRY(const LOG_ENTRY &plog_entry);


void fill(const CAMERA_IMAGE_CAPTURED &pcamera_image_captured);


void on_CAMERA_IMAGE_CAPTURED(const CAMERA_IMAGE_CAPTURED &pcamera_image_captured);


void fill(const DEBUG &pdebug);


void on_DEBUG(const DEBUG &pdebug);


void fill(const AHRS &pahrs);


void on_AHRS(const AHRS &pahrs);


void fill(const VIDEO_STREAM_INFORMATION &pvideo_stream_information);


void on_VIDEO_STREAM_INFORMATION(const VIDEO_STREAM_INFORMATION &pvideo_stream_information);


void on_SCALED_IMU(const SCALED_IMU &pscaled_imu);


void on_RC_CHANNELS_OVERRIDE(const RC_CHANNELS_OVERRIDE &prc_channels_override);


void fill(const GIMBAL_CONTROL &pgimbal_control);


void on_GIMBAL_CONTROL(const GIMBAL_CONTROL &pgimbal_control);


void fill(const TERRAIN_DATA &pterrain_data);


void on_TERRAIN_DATA(const TERRAIN_DATA &pterrain_data);


void on_PARAM_SET(const PARAM_SET &pparam_set);


void fill(const DEVICE_OP_WRITE_REPLY &pdevice_op_write_reply);


void on_DEVICE_OP_WRITE_REPLY(const DEVICE_OP_WRITE_REPLY &pdevice_op_write_reply);


void on_GPS_STATUS(const GPS_STATUS &pgps_status);


void fill(const CAMERA_INFORMATION &pcamera_information);


void on_CAMERA_INFORMATION(const CAMERA_INFORMATION &pcamera_information);


void fill(const STORAGE_INFORMATION &pstorage_information);


void on_STORAGE_INFORMATION(const STORAGE_INFORMATION &pstorage_information);


void fill(const SENSOR_OFFSETS &psensor_offsets);


void on_SENSOR_OFFSETS(const SENSOR_OFFSETS &psensor_offsets);


void fill(const HIL_STATE_QUATERNION &phil_state_quaternion);


void on_HIL_STATE_QUATERNION(const HIL_STATE_QUATERNION &phil_state_quaternion);


void fill(const ALTITUDE &paltitude);


void on_ALTITUDE(const ALTITUDE &paltitude);


void fill(const GIMBAL_TORQUE_CMD_REPORT &pgimbal_torque_cmd_report);


void on_GIMBAL_TORQUE_CMD_REPORT(const GIMBAL_TORQUE_CMD_REPORT &pgimbal_torque_cmd_report);


void fill(const COLLISION &pcollision);


void on_COLLISION(const COLLISION &pcollision);


void fill(const UAVCAN_NODE_STATUS &puavcan_node_status);


void on_UAVCAN_NODE_STATUS(const UAVCAN_NODE_STATUS &puavcan_node_status);


void on_SAFETY_SET_ALLOWED_AREA(const SAFETY_SET_ALLOWED_AREA &psafety_set_allowed_area);


void fill(const BUTTON_CHANGE &pbutton_change);


void on_BUTTON_CHANGE(const BUTTON_CHANGE &pbutton_change);


void on_GLOBAL_POSITION_INT_COV(const GLOBAL_POSITION_INT_COV &pglobal_position_int_cov);


void fill(const PARAM_EXT_REQUEST_LIST &pparam_ext_request_list);


void on_PARAM_EXT_REQUEST_LIST(const PARAM_EXT_REQUEST_LIST &pparam_ext_request_list);


void fill(const TIMESYNC &ptimesync);


void on_TIMESYNC(const TIMESYNC &ptimesync);


void fill(const HWSTATUS &phwstatus);


void on_HWSTATUS(const HWSTATUS &phwstatus);


void fill(const ESTIMATOR_STATUS &pestimator_status);


void on_ESTIMATOR_STATUS(const ESTIMATOR_STATUS &pestimator_status);


void fill(const EKF_STATUS_REPORT &pekf_status_report);


void on_EKF_STATUS_REPORT(const EKF_STATUS_REPORT &pekf_status_report);


void fill(const MESSAGE_INTERVAL &pmessage_interval);


void on_MESSAGE_INTERVAL(const MESSAGE_INTERVAL &pmessage_interval);


void fill(const ADSB_VEHICLE &padsb_vehicle);


void on_ADSB_VEHICLE(const ADSB_VEHICLE &padsb_vehicle);


void on_MISSION_COUNT(const MISSION_COUNT &pmission_count);


void on_ATTITUDE_TARGET(const ATTITUDE_TARGET &pattitude_target);


void on_ATTITUDE_TARGET(const ATTITUDE_TARGET &pattitude_target) {
	auto some_time_boot_ms = pattitude_target.time_boot_ms();
	auto some_type_mask    = pattitude_target.type_mask();
	
	const auto  src_q = pattitude_target.q();
	for (size_t index = 0; index < ATTITUDE_TARGET::q_::len; index++)
		some_float                   = src_q.get(index);
	auto        some_body_roll_rate  = pattitude_target.body_roll_rate();
	auto        some_body_pitch_rate = pattitude_target.body_pitch_rate();
	auto        some_body_yaw_rate   = pattitude_target.body_yaw_rate();
	auto        some_thrust          = pattitude_target.thrust();
	
}


void on_MISSION_COUNT(const MISSION_COUNT &pmission_count) {
	auto some_target_system    = pmission_count.target_system();
	auto some_target_component = pmission_count.target_component();
	auto some_count            = pmission_count.count();
	
	const auto src_mission_type = pmission_count.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_ADSB_VEHICLE(const ADSB_VEHICLE &padsb_vehicle) {
	auto some_ICAO_address = padsb_vehicle.ICAO_address();
	auto some_lat          = padsb_vehicle.lat();
	auto some_lon          = padsb_vehicle.lon();
	
	const auto src_altitude_type = padsb_vehicle.altitude_type();
	
	if (src_altitude_type.IS_EXISTS)
		auto some_altitude_type = src_altitude_type.CASE.EXISTS.value;
	auto     some_altitude      = padsb_vehicle.altitude();
	auto     some_heading       = padsb_vehicle.heading();
	auto     some_hor_velocity  = padsb_vehicle.hor_velocity();
	auto     some_ver_velocity  = padsb_vehicle.ver_velocity();
	
	const auto src_callsign = padsb_vehicle.callsign();
	if (src_callsign.IS_EXISTS) {
		auto        array = src_callsign.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_emitter_type = padsb_vehicle.emitter_type();
	
	if (src_emitter_type.IS_EXISTS)
		auto some_emitter_type = src_emitter_type.CASE.EXISTS.value;
	auto     some_tslc         = padsb_vehicle.tslc();
	
	const auto src_flags = padsb_vehicle.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags  = src_flags.CASE.EXISTS.value;
	auto     some_squawk = padsb_vehicle.squawk();
	
}


void fill(const ADSB_VEHICLE &padsb_vehicle) {
	padsb_vehicle.ICAO_address(some_int32_t);
	padsb_vehicle.lat(some_int32_t);
	padsb_vehicle.lon(some_int32_t);
	
	const auto src_altitude_type = padsb_vehicle.altitude_type();
	
	if (!src_altitude_type.IS_EXISTS)
		padsb_vehicle.altitude_type(com::company::demo::ADSB_ALTITUDE_TYPE::ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
	
	padsb_vehicle.altitude(some_int32_t);
	padsb_vehicle.heading(some_int16_t);
	padsb_vehicle.hor_velocity(some_int16_t);
	padsb_vehicle.ver_velocity(some_int16_t);
	
	const auto src_callsign = padsb_vehicle.callsign();
	
	if (!src_callsign.IS_EXISTS)
		padsb_vehicle.callsign(some_string);
	
	
	const auto src_emitter_type = padsb_vehicle.emitter_type();
	
	if (!src_emitter_type.IS_EXISTS)
		padsb_vehicle.emitter_type(com::company::demo::ADSB_EMITTER_TYPE::ADSB_EMITTER_TYPE_NO_INFO);
	
	padsb_vehicle.tslc(some_int8_t);
	
	const auto src_flags = padsb_vehicle.flags();
	
	if (!src_flags.IS_EXISTS)
		padsb_vehicle.flags(com::company::demo::ADSB_FLAGS::ADSB_FLAGS_VALID_COORDS);
	
	padsb_vehicle.squawk(some_int16_t);
	
}


void on_MESSAGE_INTERVAL(const MESSAGE_INTERVAL &pmessage_interval) {
	auto some_message_id  = pmessage_interval.message_id();
	auto some_interval_us = pmessage_interval.interval_us();
	
}


void fill(const MESSAGE_INTERVAL &pmessage_interval) {
	pmessage_interval.message_id(some_int16_t);
	pmessage_interval.interval_us(some_int32_t);
	
}


void on_EKF_STATUS_REPORT(const EKF_STATUS_REPORT &pekf_status_report) {
	
	const auto src_flags = pekf_status_report.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags                = src_flags.CASE.EXISTS.value;
	auto     some_velocity_variance    = pekf_status_report.velocity_variance();
	auto     some_pos_horiz_variance   = pekf_status_report.pos_horiz_variance();
	auto     some_pos_vert_variance    = pekf_status_report.pos_vert_variance();
	auto     some_compass_variance     = pekf_status_report.compass_variance();
	auto     some_terrain_alt_variance = pekf_status_report.terrain_alt_variance();
	
}


void fill(const EKF_STATUS_REPORT &pekf_status_report) {
	
	const auto src_flags = pekf_status_report.flags();
	
	if (!src_flags.IS_EXISTS)
		pekf_status_report.flags(com::company::demo::EKF_STATUS_FLAGS::EKF_ATTITUDE);
	
	pekf_status_report.velocity_variance(some_float);
	pekf_status_report.pos_horiz_variance(some_float);
	pekf_status_report.pos_vert_variance(some_float);
	pekf_status_report.compass_variance(some_float);
	pekf_status_report.terrain_alt_variance(some_float);
	
}


void on_ESTIMATOR_STATUS(const ESTIMATOR_STATUS &pestimator_status) {
	auto some_time_usec = pestimator_status.time_usec();
	
	const auto src_flags = pestimator_status.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags              = src_flags.CASE.EXISTS.value;
	auto     some_vel_ratio          = pestimator_status.vel_ratio();
	auto     some_pos_horiz_ratio    = pestimator_status.pos_horiz_ratio();
	auto     some_pos_vert_ratio     = pestimator_status.pos_vert_ratio();
	auto     some_mag_ratio          = pestimator_status.mag_ratio();
	auto     some_hagl_ratio         = pestimator_status.hagl_ratio();
	auto     some_tas_ratio          = pestimator_status.tas_ratio();
	auto     some_pos_horiz_accuracy = pestimator_status.pos_horiz_accuracy();
	auto     some_pos_vert_accuracy  = pestimator_status.pos_vert_accuracy();
	
}


void fill(const ESTIMATOR_STATUS &pestimator_status) {
	pestimator_status.time_usec(some_int64_t);
	
	const auto src_flags = pestimator_status.flags();
	
	if (!src_flags.IS_EXISTS)
		pestimator_status.flags(com::company::demo::ESTIMATOR_STATUS_FLAGS::ESTIMATOR_ATTITUDE);
	
	pestimator_status.vel_ratio(some_float);
	pestimator_status.pos_horiz_ratio(some_float);
	pestimator_status.pos_vert_ratio(some_float);
	pestimator_status.mag_ratio(some_float);
	pestimator_status.hagl_ratio(some_float);
	pestimator_status.tas_ratio(some_float);
	pestimator_status.pos_horiz_accuracy(some_float);
	pestimator_status.pos_vert_accuracy(some_float);
	
}


void on_HWSTATUS(const HWSTATUS &phwstatus) {
	auto some_Vcc    = phwstatus.Vcc();
	auto some_I2Cerr = phwstatus.I2Cerr();
	
}


void fill(const HWSTATUS &phwstatus) {
	phwstatus.Vcc(some_int16_t);
	phwstatus.I2Cerr(some_int8_t);
	
}


void on_TIMESYNC(const TIMESYNC &ptimesync) {
	auto some_tc1 = ptimesync.tc1();
	auto some_ts1 = ptimesync.ts1();
	
}


void fill(const TIMESYNC &ptimesync) {
	ptimesync.tc1(some_int64_t);
	ptimesync.ts1(some_int64_t);
	
}


void on_PARAM_EXT_REQUEST_LIST(const PARAM_EXT_REQUEST_LIST &pparam_ext_request_list) {
	auto some_target_system    = pparam_ext_request_list.target_system();
	auto some_target_component = pparam_ext_request_list.target_component();
	
}


void fill(const PARAM_EXT_REQUEST_LIST &pparam_ext_request_list) {
	pparam_ext_request_list.target_system(some_int8_t);
	pparam_ext_request_list.target_component(some_int8_t);
	
}


void on_GLOBAL_POSITION_INT_COV(const GLOBAL_POSITION_INT_COV &pglobal_position_int_cov) {
	auto some_time_usec = pglobal_position_int_cov.time_usec();
	
	const auto src_estimator_type = pglobal_position_int_cov.estimator_type();
	
	if (src_estimator_type.IS_EXISTS)
		auto some_estimator_type = src_estimator_type.CASE.EXISTS.value;
	auto     some_lat            = pglobal_position_int_cov.lat();
	auto     some_lon            = pglobal_position_int_cov.lon();
	auto     some_alt            = pglobal_position_int_cov.alt();
	auto     some_relative_alt   = pglobal_position_int_cov.relative_alt();
	auto     some_vx             = pglobal_position_int_cov.vx();
	auto     some_vy             = pglobal_position_int_cov.vy();
	auto     some_vz             = pglobal_position_int_cov.vz();
	
	const auto  src_covariance = pglobal_position_int_cov.covariance();
	for (size_t index          = 0; index < GLOBAL_POSITION_INT_COV::covariance_::len; index++)
		some_float = src_covariance.get(index);
	
}


void on_BUTTON_CHANGE(const BUTTON_CHANGE &pbutton_change) {
	auto some_time_boot_ms   = pbutton_change.time_boot_ms();
	auto some_last_change_ms = pbutton_change.last_change_ms();
	auto some_state          = pbutton_change.state();
	
}


void fill(const BUTTON_CHANGE &pbutton_change) {
	pbutton_change.time_boot_ms(some_int32_t);
	pbutton_change.last_change_ms(some_int32_t);
	pbutton_change.state(some_int8_t);
	
}


void on_SAFETY_SET_ALLOWED_AREA(const SAFETY_SET_ALLOWED_AREA &psafety_set_allowed_area) {
	auto some_target_system    = psafety_set_allowed_area.target_system();
	auto some_target_component = psafety_set_allowed_area.target_component();
	
	const auto src_frame = psafety_set_allowed_area.frame();
	
	if (src_frame.IS_EXISTS)
		auto some_frame = src_frame.CASE.EXISTS.value;
	auto     some_p1x   = psafety_set_allowed_area.p1x();
	auto     some_p1y   = psafety_set_allowed_area.p1y();
	auto     some_p1z   = psafety_set_allowed_area.p1z();
	auto     some_p2x   = psafety_set_allowed_area.p2x();
	auto     some_p2y   = psafety_set_allowed_area.p2y();
	auto     some_p2z   = psafety_set_allowed_area.p2z();
	
}


void on_UAVCAN_NODE_STATUS(const UAVCAN_NODE_STATUS &puavcan_node_status) {
	auto some_time_usec  = puavcan_node_status.time_usec();
	auto some_uptime_sec = puavcan_node_status.uptime_sec();
	
	const auto src_health = puavcan_node_status.health();
	
	if (src_health.IS_EXISTS)
		auto some_health = src_health.CASE.EXISTS.value;
	
	const auto src_mode = puavcan_node_status.mode();
	
	if (src_mode.IS_EXISTS)
		auto some_mode                        = src_mode.CASE.EXISTS.value;
	auto     some_sub_mode                    = puavcan_node_status.sub_mode();
	auto     some_vendor_specific_status_code = puavcan_node_status.vendor_specific_status_code();
	
}


void fill(const UAVCAN_NODE_STATUS &puavcan_node_status) {
	puavcan_node_status.time_usec(some_int64_t);
	puavcan_node_status.uptime_sec(some_int32_t);
	
	const auto src_health = puavcan_node_status.health();
	
	if (!src_health.IS_EXISTS)
		puavcan_node_status.health(com::company::demo::UAVCAN_NODE_HEALTH::UAVCAN_NODE_HEALTH_OK);
	
	
	const auto src_mode = puavcan_node_status.mode();
	
	if (!src_mode.IS_EXISTS)
		puavcan_node_status.mode(com::company::demo::UAVCAN_NODE_MODE::UAVCAN_NODE_MODE_OPERATIONAL);
	
	puavcan_node_status.sub_mode(some_int8_t);
	puavcan_node_status.vendor_specific_status_code(some_int16_t);
	
}


void on_COLLISION(const COLLISION &pcollision) {
	
	const auto src_sRc = pcollision.sRc();
	
	if (src_sRc.IS_EXISTS)
		auto some_sRc = src_sRc.CASE.EXISTS.value;
	auto     some_id  = pcollision.id();
	
	const auto src_action = pcollision.action();
	
	if (src_action.IS_EXISTS)
		auto some_action = src_action.CASE.EXISTS.value;
	
	const auto src_threat_level = pcollision.threat_level();
	
	if (src_threat_level.IS_EXISTS)
		auto some_threat_level             = src_threat_level.CASE.EXISTS.value;
	auto     some_time_to_minimum_delta    = pcollision.time_to_minimum_delta();
	auto     some_altitude_minimum_delta   = pcollision.altitude_minimum_delta();
	auto     some_horizontal_minimum_delta = pcollision.horizontal_minimum_delta();
	
}


void fill(const COLLISION &pcollision) {
	
	const auto src_sRc = pcollision.sRc();
	
	if (!src_sRc.IS_EXISTS)
		pcollision.sRc(com::company::demo::MAV_COLLISION_SRC::MAV_COLLISION_SRC_ADSB);
	
	pcollision.id(some_int32_t);
	
	const auto src_action = pcollision.action();
	
	if (!src_action.IS_EXISTS)
		pcollision.action(com::company::demo::MAV_COLLISION_ACTION::MAV_COLLISION_ACTION_NONE);
	
	
	const auto src_threat_level = pcollision.threat_level();
	
	if (!src_threat_level.IS_EXISTS)
		pcollision.threat_level(com::company::demo::MAV_COLLISION_THREAT_LEVEL::MAV_COLLISION_THREAT_LEVEL_NONE);
	
	pcollision.time_to_minimum_delta(some_float);
	pcollision.altitude_minimum_delta(some_float);
	pcollision.horizontal_minimum_delta(some_float);
	
}


void on_GIMBAL_TORQUE_CMD_REPORT(const GIMBAL_TORQUE_CMD_REPORT &pgimbal_torque_cmd_report) {
	auto some_target_system    = pgimbal_torque_cmd_report.target_system();
	auto some_target_component = pgimbal_torque_cmd_report.target_component();
	auto some_rl_torque_cmd    = pgimbal_torque_cmd_report.rl_torque_cmd();
	auto some_el_torque_cmd    = pgimbal_torque_cmd_report.el_torque_cmd();
	auto some_az_torque_cmd    = pgimbal_torque_cmd_report.az_torque_cmd();
	
}


void fill(const GIMBAL_TORQUE_CMD_REPORT &pgimbal_torque_cmd_report) {
	pgimbal_torque_cmd_report.target_system(some_int8_t);
	pgimbal_torque_cmd_report.target_component(some_int8_t);
	pgimbal_torque_cmd_report.rl_torque_cmd(some_int16_t);
	pgimbal_torque_cmd_report.el_torque_cmd(some_int16_t);
	pgimbal_torque_cmd_report.az_torque_cmd(some_int16_t);
	
}


void on_ALTITUDE(const ALTITUDE &paltitude) {
	auto some_time_usec          = paltitude.time_usec();
	auto some_altitude_monotonic = paltitude.altitude_monotonic();
	auto some_altitude_amsl      = paltitude.altitude_amsl();
	auto some_altitude_local     = paltitude.altitude_local();
	auto some_altitude_relative  = paltitude.altitude_relative();
	auto some_altitude_terrain   = paltitude.altitude_terrain();
	auto some_bottom_clearance   = paltitude.bottom_clearance();
	
}


void fill(const ALTITUDE &paltitude) {
	paltitude.time_usec(some_int64_t);
	paltitude.altitude_monotonic(some_float);
	paltitude.altitude_amsl(some_float);
	paltitude.altitude_local(some_float);
	paltitude.altitude_relative(some_float);
	paltitude.altitude_terrain(some_float);
	paltitude.bottom_clearance(some_float);
	
}


void on_HIL_STATE_QUATERNION(const HIL_STATE_QUATERNION &phil_state_quaternion) {
	auto some_time_usec = phil_state_quaternion.time_usec();
	
	const auto  src_attitude_quaternion = phil_state_quaternion.attitude_quaternion();
	for (size_t index                   = 0; index < HIL_STATE_QUATERNION::attitude_quaternion_::len; index++)
		some_float                 = src_attitude_quaternion.get(index);
	auto        some_rollspeed     = phil_state_quaternion.rollspeed();
	auto        some_pitchspeed    = phil_state_quaternion.pitchspeed();
	auto        some_yawspeed      = phil_state_quaternion.yawspeed();
	auto        some_lat           = phil_state_quaternion.lat();
	auto        some_lon           = phil_state_quaternion.lon();
	auto        some_alt           = phil_state_quaternion.alt();
	auto        some_vx            = phil_state_quaternion.vx();
	auto        some_vy            = phil_state_quaternion.vy();
	auto        some_vz            = phil_state_quaternion.vz();
	auto        some_ind_airspeed  = phil_state_quaternion.ind_airspeed();
	auto        some_true_airspeed = phil_state_quaternion.true_airspeed();
	auto        some_xacc          = phil_state_quaternion.xacc();
	auto        some_yacc          = phil_state_quaternion.yacc();
	auto        some_zacc          = phil_state_quaternion.zacc();
	
}


void fill(const HIL_STATE_QUATERNION &phil_state_quaternion) {
	phil_state_quaternion.time_usec(some_int64_t);
	phil_state_quaternion.attitude_quaternion(&some_float);
	phil_state_quaternion.rollspeed(some_float);
	phil_state_quaternion.pitchspeed(some_float);
	phil_state_quaternion.yawspeed(some_float);
	phil_state_quaternion.lat(some_int32_t);
	phil_state_quaternion.lon(some_int32_t);
	phil_state_quaternion.alt(some_int32_t);
	phil_state_quaternion.vx(some_int16_t);
	phil_state_quaternion.vy(some_int16_t);
	phil_state_quaternion.vz(some_int16_t);
	phil_state_quaternion.ind_airspeed(some_int16_t);
	phil_state_quaternion.true_airspeed(some_int16_t);
	phil_state_quaternion.xacc(some_int16_t);
	phil_state_quaternion.yacc(some_int16_t);
	phil_state_quaternion.zacc(some_int16_t);
	
}


void on_SENSOR_OFFSETS(const SENSOR_OFFSETS &psensor_offsets) {
	auto some_mag_ofs_x       = psensor_offsets.mag_ofs_x();
	auto some_mag_ofs_y       = psensor_offsets.mag_ofs_y();
	auto some_mag_ofs_z       = psensor_offsets.mag_ofs_z();
	auto some_mag_declination = psensor_offsets.mag_declination();
	auto some_raw_press       = psensor_offsets.raw_press();
	auto some_raw_temp        = psensor_offsets.raw_temp();
	auto some_gyro_cal_x      = psensor_offsets.gyro_cal_x();
	auto some_gyro_cal_y      = psensor_offsets.gyro_cal_y();
	auto some_gyro_cal_z      = psensor_offsets.gyro_cal_z();
	auto some_accel_cal_x     = psensor_offsets.accel_cal_x();
	auto some_accel_cal_y     = psensor_offsets.accel_cal_y();
	auto some_accel_cal_z     = psensor_offsets.accel_cal_z();
	
}


void fill(const SENSOR_OFFSETS &psensor_offsets) {
	psensor_offsets.mag_ofs_x(some_int16_t);
	psensor_offsets.mag_ofs_y(some_int16_t);
	psensor_offsets.mag_ofs_z(some_int16_t);
	psensor_offsets.mag_declination(some_float);
	psensor_offsets.raw_press(some_int32_t);
	psensor_offsets.raw_temp(some_int32_t);
	psensor_offsets.gyro_cal_x(some_float);
	psensor_offsets.gyro_cal_y(some_float);
	psensor_offsets.gyro_cal_z(some_float);
	psensor_offsets.accel_cal_x(some_float);
	psensor_offsets.accel_cal_y(some_float);
	psensor_offsets.accel_cal_z(some_float);
	
}


void on_STORAGE_INFORMATION(const STORAGE_INFORMATION &pstorage_information) {
	auto some_time_boot_ms       = pstorage_information.time_boot_ms();
	auto some_storage_id         = pstorage_information.storage_id();
	auto some_storage_count      = pstorage_information.storage_count();
	auto some_status             = pstorage_information.status();
	auto some_total_capacity     = pstorage_information.total_capacity();
	auto some_used_capacity      = pstorage_information.used_capacity();
	auto some_available_capacity = pstorage_information.available_capacity();
	auto some_read_speed         = pstorage_information.read_speed();
	auto some_write_speed        = pstorage_information.write_speed();
	
}


void fill(const STORAGE_INFORMATION &pstorage_information) {
	pstorage_information.time_boot_ms(some_int32_t);
	pstorage_information.storage_id(some_int8_t);
	pstorage_information.storage_count(some_int8_t);
	pstorage_information.status(some_int8_t);
	pstorage_information.total_capacity(some_float);
	pstorage_information.used_capacity(some_float);
	pstorage_information.available_capacity(some_float);
	pstorage_information.read_speed(some_float);
	pstorage_information.write_speed(some_float);
	
}


void on_CAMERA_INFORMATION(const CAMERA_INFORMATION &pcamera_information) {
	auto some_time_boot_ms = pcamera_information.time_boot_ms();
	
	const auto  src_vendor_name = pcamera_information.vendor_name();
	for (size_t index           = 0; index < CAMERA_INFORMATION::vendor_name_::len; index++)
		some_int8_t = src_vendor_name.get(index);
	
	const auto  src_model_name = pcamera_information.model_name();
	for (size_t index          = 0; index < CAMERA_INFORMATION::model_name_::len; index++)
		some_int8_t                   = src_model_name.get(index);
	auto        some_firmware_version = pcamera_information.firmware_version();
	auto        some_focal_length     = pcamera_information.focal_length();
	auto        some_sensor_size_h    = pcamera_information.sensor_size_h();
	auto        some_sensor_size_v    = pcamera_information.sensor_size_v();
	auto        some_resolution_h     = pcamera_information.resolution_h();
	auto        some_resolution_v     = pcamera_information.resolution_v();
	auto        some_lens_id          = pcamera_information.lens_id();
	
	const auto src_flags = pcamera_information.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags                  = src_flags.CASE.EXISTS.value;
	auto     some_cam_definition_version = pcamera_information.cam_definition_version();
	
	const auto src_cam_definition_uri = pcamera_information.cam_definition_uri();
	if (src_cam_definition_uri.IS_EXISTS) {
		auto        array = src_cam_definition_uri.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void fill(const CAMERA_INFORMATION &pcamera_information) {
	pcamera_information.time_boot_ms(some_int32_t);
	pcamera_information.vendor_name(&some_int8_t);
	pcamera_information.model_name(&some_int8_t);
	pcamera_information.firmware_version(some_int32_t);
	pcamera_information.focal_length(some_float);
	pcamera_information.sensor_size_h(some_float);
	pcamera_information.sensor_size_v(some_float);
	pcamera_information.resolution_h(some_int16_t);
	pcamera_information.resolution_v(some_int16_t);
	pcamera_information.lens_id(some_int8_t);
	
	const auto src_flags = pcamera_information.flags();
	
	if (!src_flags.IS_EXISTS)
		pcamera_information.flags(com::company::demo::CAMERA_CAP_FLAGS::CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
	
	pcamera_information.cam_definition_version(some_int16_t);
	
	const auto src_cam_definition_uri = pcamera_information.cam_definition_uri();
	
	if (!src_cam_definition_uri.IS_EXISTS)
		pcamera_information.cam_definition_uri(some_string);
	
	
}


void on_GPS_STATUS(const GPS_STATUS &pgps_status) {
	auto some_satellites_visible = pgps_status.satellites_visible();
	
	const auto  src_satellite_prn = pgps_status.satellite_prn();
	for (size_t index             = 0; index < GPS_STATUS::satellite_prn_::len; index++)
		some_int8_t = src_satellite_prn.get(index);
	
	const auto  src_satellite_used = pgps_status.satellite_used();
	for (size_t index              = 0; index < GPS_STATUS::satellite_used_::len; index++)
		some_int8_t = src_satellite_used.get(index);
	
	const auto  src_satellite_elevation = pgps_status.satellite_elevation();
	for (size_t index                   = 0; index < GPS_STATUS::satellite_elevation_::len; index++)
		some_int8_t = src_satellite_elevation.get(index);
	
	const auto  src_satellite_azimuth = pgps_status.satellite_azimuth();
	for (size_t index                 = 0; index < GPS_STATUS::satellite_azimuth_::len; index++)
		some_int8_t = src_satellite_azimuth.get(index);
	
	const auto  src_satellite_snr = pgps_status.satellite_snr();
	for (size_t index             = 0; index < GPS_STATUS::satellite_snr_::len; index++)
		some_int8_t = src_satellite_snr.get(index);
	
}


void on_DEVICE_OP_WRITE_REPLY(const DEVICE_OP_WRITE_REPLY &pdevice_op_write_reply) {
	auto some_request_id = pdevice_op_write_reply.request_id();
	auto some_result     = pdevice_op_write_reply.result();
	
}


void fill(const DEVICE_OP_WRITE_REPLY &pdevice_op_write_reply) {
	pdevice_op_write_reply.request_id(some_int32_t);
	pdevice_op_write_reply.result(some_int8_t);
	
}


void on_PARAM_SET(const PARAM_SET &pparam_set) {
	auto some_target_system    = pparam_set.target_system();
	auto some_target_component = pparam_set.target_component();
	
	const auto src_param_id     = pparam_set.param_id();
	if (src_param_id.IS_EXISTS) {
		auto        array = src_param_id.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_param_value = pparam_set.param_value();
	
	const auto src_param_type = pparam_set.param_type();
	
	if (src_param_type.IS_EXISTS)
		auto some_param_type = src_param_type.CASE.EXISTS.value;
	
}


void on_TERRAIN_DATA(const TERRAIN_DATA &pterrain_data) {
	auto some_lat          = pterrain_data.lat();
	auto some_lon          = pterrain_data.lon();
	auto some_grid_spacing = pterrain_data.grid_spacing();
	auto some_gridbit      = pterrain_data.gridbit();
	
	const auto  src_daTa = pterrain_data.daTa();
	for (size_t index    = 0; index < TERRAIN_DATA::daTa_::len; index++)
		some_int16_t = src_daTa.get(index);
	
}


void fill(const TERRAIN_DATA &pterrain_data) {
	pterrain_data.lat(some_int32_t);
	pterrain_data.lon(some_int32_t);
	pterrain_data.grid_spacing(some_int16_t);
	pterrain_data.gridbit(some_int8_t);
	pterrain_data.daTa(&some_int16_t);
}


void on_GIMBAL_CONTROL(const GIMBAL_CONTROL &pgimbal_control) {
	auto some_target_system    = pgimbal_control.target_system();
	auto some_target_component = pgimbal_control.target_component();
	auto some_demanded_rate_x  = pgimbal_control.demanded_rate_x();
	auto some_demanded_rate_y  = pgimbal_control.demanded_rate_y();
	auto some_demanded_rate_z  = pgimbal_control.demanded_rate_z();
	
}


void fill(const GIMBAL_CONTROL &pgimbal_control) {
	pgimbal_control.target_system(some_int8_t);
	pgimbal_control.target_component(some_int8_t);
	pgimbal_control.demanded_rate_x(some_float);
	pgimbal_control.demanded_rate_y(some_float);
	pgimbal_control.demanded_rate_z(some_float);
	
}


void on_RC_CHANNELS_OVERRIDE(const RC_CHANNELS_OVERRIDE &prc_channels_override) {
	auto some_target_system    = prc_channels_override.target_system();
	auto some_target_component = prc_channels_override.target_component();
	auto some_chan1_raw        = prc_channels_override.chan1_raw();
	auto some_chan2_raw        = prc_channels_override.chan2_raw();
	auto some_chan3_raw        = prc_channels_override.chan3_raw();
	auto some_chan4_raw        = prc_channels_override.chan4_raw();
	auto some_chan5_raw        = prc_channels_override.chan5_raw();
	auto some_chan6_raw        = prc_channels_override.chan6_raw();
	auto some_chan7_raw        = prc_channels_override.chan7_raw();
	auto some_chan8_raw        = prc_channels_override.chan8_raw();
	
}


void on_SCALED_IMU(const SCALED_IMU &pscaled_imu) {
	auto some_time_boot_ms = pscaled_imu.time_boot_ms();
	auto some_xacc         = pscaled_imu.xacc();
	auto some_yacc         = pscaled_imu.yacc();
	auto some_zacc         = pscaled_imu.zacc();
	auto some_xgyro        = pscaled_imu.xgyro();
	auto some_ygyro        = pscaled_imu.ygyro();
	auto some_zgyro        = pscaled_imu.zgyro();
	auto some_xmag         = pscaled_imu.xmag();
	auto some_ymag         = pscaled_imu.ymag();
	auto some_zmag         = pscaled_imu.zmag();
	
}


void on_VIDEO_STREAM_INFORMATION(const VIDEO_STREAM_INFORMATION &pvideo_stream_information) {
	auto some_camera_id    = pvideo_stream_information.camera_id();
	auto some_status       = pvideo_stream_information.status();
	auto some_framerate    = pvideo_stream_information.framerate();
	auto some_resolution_h = pvideo_stream_information.resolution_h();
	auto some_resolution_v = pvideo_stream_information.resolution_v();
	auto some_bitrate      = pvideo_stream_information.bitrate();
	auto some_rotation     = pvideo_stream_information.rotation();
	
	const auto src_uri = pvideo_stream_information.uri();
	if (src_uri.IS_EXISTS) {
		auto        array = src_uri.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void fill(const VIDEO_STREAM_INFORMATION &pvideo_stream_information) {
	pvideo_stream_information.camera_id(some_int8_t);
	pvideo_stream_information.status(some_int8_t);
	pvideo_stream_information.framerate(some_float);
	pvideo_stream_information.resolution_h(some_int16_t);
	pvideo_stream_information.resolution_v(some_int16_t);
	pvideo_stream_information.bitrate(some_int32_t);
	pvideo_stream_information.rotation(some_int16_t);
	
	const auto src_uri = pvideo_stream_information.uri();
	
	if (!src_uri.IS_EXISTS)
		pvideo_stream_information.uri(some_string);
	
	
}


void on_AHRS(const AHRS &pahrs) {
	auto some_omegaIx      = pahrs.omegaIx();
	auto some_omegaIy      = pahrs.omegaIy();
	auto some_omegaIz      = pahrs.omegaIz();
	auto some_accel_weight = pahrs.accel_weight();
	auto some_renorm_val   = pahrs.renorm_val();
	auto some_error_rp     = pahrs.error_rp();
	auto some_error_yaw    = pahrs.error_yaw();
	
}


void fill(const AHRS &pahrs) {
	pahrs.omegaIx(some_float);
	pahrs.omegaIy(some_float);
	pahrs.omegaIz(some_float);
	pahrs.accel_weight(some_float);
	pahrs.renorm_val(some_float);
	pahrs.error_rp(some_float);
	pahrs.error_yaw(some_float);
	
}


void on_DEBUG(const DEBUG &pdebug) {
	auto some_time_boot_ms = pdebug.time_boot_ms();
	auto some_ind          = pdebug.ind();
	auto some_value        = pdebug.value();
	
}


void fill(const DEBUG &pdebug) {
	pdebug.time_boot_ms(some_int32_t);
	pdebug.ind(some_int8_t);
	pdebug.value(some_float);
	
}


void on_CAMERA_IMAGE_CAPTURED(const CAMERA_IMAGE_CAPTURED &pcamera_image_captured) {
	auto some_time_boot_ms = pcamera_image_captured.time_boot_ms();
	auto some_time_utc     = pcamera_image_captured.time_utc();
	auto some_camera_id    = pcamera_image_captured.camera_id();
	auto some_lat          = pcamera_image_captured.lat();
	auto some_lon          = pcamera_image_captured.lon();
	auto some_alt          = pcamera_image_captured.alt();
	auto some_relative_alt = pcamera_image_captured.relative_alt();
	
	const auto  src_q = pcamera_image_captured.q();
	for (size_t index = 0; index < CAMERA_IMAGE_CAPTURED::q_::len; index++)
		some_float                  = src_q.get(index);
	auto        some_image_index    = pcamera_image_captured.image_index();
	auto        some_capture_result = pcamera_image_captured.capture_result();
	
	const auto src_file_url = pcamera_image_captured.file_url();
	if (src_file_url.IS_EXISTS) {
		auto        array = src_file_url.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void fill(const CAMERA_IMAGE_CAPTURED &pcamera_image_captured) {
	pcamera_image_captured.time_boot_ms(some_int32_t);
	pcamera_image_captured.time_utc(some_int64_t);
	pcamera_image_captured.camera_id(some_int8_t);
	pcamera_image_captured.lat(some_int32_t);
	pcamera_image_captured.lon(some_int32_t);
	pcamera_image_captured.alt(some_int32_t);
	pcamera_image_captured.relative_alt(some_int32_t);
	pcamera_image_captured.q(&some_float);
	pcamera_image_captured.image_index(some_int32_t);
	pcamera_image_captured.capture_result(some_int8_t);
	
	const auto src_file_url = pcamera_image_captured.file_url();
	
	if (!src_file_url.IS_EXISTS)
		pcamera_image_captured.file_url(some_string);
	
	
}


void on_LOG_ENTRY(const LOG_ENTRY &plog_entry) {
	auto some_id           = plog_entry.id();
	auto some_num_logs     = plog_entry.num_logs();
	auto some_last_log_num = plog_entry.last_log_num();
	auto some_time_utc     = plog_entry.time_utc();
	auto some_size         = plog_entry.size();
	
}


void fill(const LOG_ENTRY &plog_entry) {
	plog_entry.id(some_int16_t);
	plog_entry.num_logs(some_int16_t);
	plog_entry.last_log_num(some_int16_t);
	plog_entry.time_utc(some_int32_t);
	plog_entry.size(some_int32_t);
	
}


void on_ACTUATOR_CONTROL_TARGET(const ACTUATOR_CONTROL_TARGET &pactuator_control_target) {
	auto some_time_usec = pactuator_control_target.time_usec();
	auto some_group_mlx = pactuator_control_target.group_mlx();
	
	const auto  src_controls = pactuator_control_target.controls();
	for (size_t index        = 0; index < ACTUATOR_CONTROL_TARGET::controls_::len; index++)
		some_float = src_controls.get(index);
	
}


void fill(const ACTUATOR_CONTROL_TARGET &pactuator_control_target) {
	pactuator_control_target.time_usec(some_int64_t);
	pactuator_control_target.group_mlx(some_int8_t);
	pactuator_control_target.controls(&some_float);
}


void on_HIGH_LATENCY(const HIGH_LATENCY &phigh_latency) {
	
	const auto src_base_mode = phigh_latency.base_mode();
	
	if (src_base_mode.IS_EXISTS)
		auto some_base_mode   = src_base_mode.CASE.EXISTS.value;
	auto     some_custom_mode = phigh_latency.custom_mode();
	
	const auto src_landed_state = phigh_latency.landed_state();
	
	if (src_landed_state.IS_EXISTS)
		auto some_landed_state  = src_landed_state.CASE.EXISTS.value;
	auto     some_roll          = phigh_latency.roll();
	auto     some_pitch         = phigh_latency.pitch();
	auto     some_heading       = phigh_latency.heading();
	auto     some_throttle      = phigh_latency.throttle();
	auto     some_heading_sp    = phigh_latency.heading_sp();
	auto     some_latitude      = phigh_latency.latitude();
	auto     some_longitude     = phigh_latency.longitude();
	auto     some_altitude_amsl = phigh_latency.altitude_amsl();
	auto     some_altitude_sp   = phigh_latency.altitude_sp();
	auto     some_airspeed      = phigh_latency.airspeed();
	auto     some_airspeed_sp   = phigh_latency.airspeed_sp();
	auto     some_groundspeed   = phigh_latency.groundspeed();
	auto     some_climb_rate    = phigh_latency.climb_rate();
	auto     some_gps_nsat      = phigh_latency.gps_nsat();
	
	const auto src_gps_fix_type = phigh_latency.gps_fix_type();
	
	if (src_gps_fix_type.IS_EXISTS)
		auto some_gps_fix_type      = src_gps_fix_type.CASE.EXISTS.value;
	auto     some_battery_remaining = phigh_latency.battery_remaining();
	auto     some_temperature       = phigh_latency.temperature();
	auto     some_temperature_air   = phigh_latency.temperature_air();
	auto     some_failsafe          = phigh_latency.failsafe();
	auto     some_wp_num            = phigh_latency.wp_num();
	auto     some_wp_distance       = phigh_latency.wp_distance();
	
}


void fill(const HIGH_LATENCY &phigh_latency) {
	
	const auto src_base_mode = phigh_latency.base_mode();
	
	if (!src_base_mode.IS_EXISTS)
		phigh_latency.base_mode(com::company::demo::MAV_MODE_FLAG::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
	
	phigh_latency.custom_mode(some_int32_t);
	
	const auto src_landed_state = phigh_latency.landed_state();
	
	if (!src_landed_state.IS_EXISTS)
		phigh_latency.landed_state(com::company::demo::MAV_LANDED_STATE::MAV_LANDED_STATE_UNDEFINED);
	
	phigh_latency.roll(some_int16_t);
	phigh_latency.pitch(some_int16_t);
	phigh_latency.heading(some_int16_t);
	phigh_latency.throttle(some_int8_t);
	phigh_latency.heading_sp(some_int16_t);
	phigh_latency.latitude(some_int32_t);
	phigh_latency.longitude(some_int32_t);
	phigh_latency.altitude_amsl(some_int16_t);
	phigh_latency.altitude_sp(some_int16_t);
	phigh_latency.airspeed(some_int8_t);
	phigh_latency.airspeed_sp(some_int8_t);
	phigh_latency.groundspeed(some_int8_t);
	phigh_latency.climb_rate(some_int8_t);
	phigh_latency.gps_nsat(some_int8_t);
	
	const auto src_gps_fix_type = phigh_latency.gps_fix_type();
	
	if (!src_gps_fix_type.IS_EXISTS)
		phigh_latency.gps_fix_type(com::company::demo::GPS_FIX_TYPE::GPS_FIX_TYPE_NO_GPS);
	
	phigh_latency.battery_remaining(some_int8_t);
	phigh_latency.temperature(some_int8_t);
	phigh_latency.temperature_air(some_int8_t);
	phigh_latency.failsafe(some_int8_t);
	phigh_latency.wp_num(some_int8_t);
	phigh_latency.wp_distance(some_int16_t);
	
}


void on_PARAM_REQUEST_READ(const PARAM_REQUEST_READ &pparam_request_read) {
	auto some_target_system    = pparam_request_read.target_system();
	auto some_target_component = pparam_request_read.target_component();
	
	const auto src_param_id     = pparam_request_read.param_id();
	if (src_param_id.IS_EXISTS) {
		auto        array = src_param_id.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_param_index = pparam_request_read.param_index();
	
}


void on_SET_ATTITUDE_TARGET(const SET_ATTITUDE_TARGET &pset_attitude_target) {
	auto some_time_boot_ms     = pset_attitude_target.time_boot_ms();
	auto some_target_system    = pset_attitude_target.target_system();
	auto some_target_component = pset_attitude_target.target_component();
	auto some_type_mask        = pset_attitude_target.type_mask();
	
	const auto  src_q = pset_attitude_target.q();
	for (size_t index = 0; index < SET_ATTITUDE_TARGET::q_::len; index++)
		some_float                   = src_q.get(index);
	auto        some_body_roll_rate  = pset_attitude_target.body_roll_rate();
	auto        some_body_pitch_rate = pset_attitude_target.body_pitch_rate();
	auto        some_body_yaw_rate   = pset_attitude_target.body_yaw_rate();
	auto        some_thrust          = pset_attitude_target.thrust();
	
}


void on_FOLLOW_TARGET(const FOLLOW_TARGET &pfollow_target) {
	auto some_timestamp        = pfollow_target.timestamp();
	auto some_est_capabilities = pfollow_target.est_capabilities();
	auto some_lat              = pfollow_target.lat();
	auto some_lon              = pfollow_target.lon();
	auto some_alt              = pfollow_target.alt();
	
	const auto  src_vel = pfollow_target.vel();
	for (size_t index   = 0; index < FOLLOW_TARGET::vel_::len; index++)
		some_float = src_vel.get(index);
	
	const auto  src_acc = pfollow_target.acc();
	for (size_t index   = 0; index < FOLLOW_TARGET::acc_::len; index++)
		some_float = src_acc.get(index);
	
	const auto  src_attitude_q = pfollow_target.attitude_q();
	for (size_t index          = 0; index < FOLLOW_TARGET::attitude_q_::len; index++)
		some_float = src_attitude_q.get(index);
	
	const auto  src_rates = pfollow_target.rates();
	for (size_t index     = 0; index < FOLLOW_TARGET::rates_::len; index++)
		some_float = src_rates.get(index);
	
	const auto  src_position_cov = pfollow_target.position_cov();
	for (size_t index            = 0; index < FOLLOW_TARGET::position_cov_::len; index++)
		some_float                = src_position_cov.get(index);
	auto        some_custom_state = pfollow_target.custom_state();
	
}


void fill(const FOLLOW_TARGET &pfollow_target) {
	pfollow_target.timestamp(some_int64_t);
	pfollow_target.est_capabilities(some_int8_t);
	pfollow_target.lat(some_int32_t);
	pfollow_target.lon(some_int32_t);
	pfollow_target.alt(some_float);
	pfollow_target.vel(&some_float);
	pfollow_target.acc(&some_float);
	pfollow_target.attitude_q(&some_float);
	pfollow_target.rates(&some_float);
	pfollow_target.position_cov(&some_float);
	pfollow_target.custom_state(some_int64_t);
	
}


void on_HIL_STATE(const HIL_STATE &phil_state) {
	auto some_time_usec  = phil_state.time_usec();
	auto some_roll       = phil_state.roll();
	auto some_pitch      = phil_state.pitch();
	auto some_yaw        = phil_state.yaw();
	auto some_rollspeed  = phil_state.rollspeed();
	auto some_pitchspeed = phil_state.pitchspeed();
	auto some_yawspeed   = phil_state.yawspeed();
	auto some_lat        = phil_state.lat();
	auto some_lon        = phil_state.lon();
	auto some_alt        = phil_state.alt();
	auto some_vx         = phil_state.vx();
	auto some_vy         = phil_state.vy();
	auto some_vz         = phil_state.vz();
	auto some_xacc       = phil_state.xacc();
	auto some_yacc       = phil_state.yacc();
	auto some_zacc       = phil_state.zacc();
	
}


void on_HOME_POSITION(const HOME_POSITION &phome_position) {
	auto some_latitude  = phome_position.latitude();
	auto some_longitude = phome_position.longitude();
	auto some_altitude  = phome_position.altitude();
	auto some_x         = phome_position.x();
	auto some_y         = phome_position.y();
	auto some_z         = phome_position.z();
	
	const auto  src_q = phome_position.q();
	for (size_t index = 0; index < HOME_POSITION::q_::len; index++)
		some_float              = src_q.get(index);
	auto        some_approach_x = phome_position.approach_x();
	auto        some_approach_y = phome_position.approach_y();
	auto        some_approach_z = phome_position.approach_z();
	
	const auto src_time_usec = phome_position.time_usec();
	
	if (src_time_usec.IS_EXISTS)
		auto some_time_usec = src_time_usec.CASE.EXISTS.value;
	
}


void fill(const HOME_POSITION &phome_position) {
	phome_position.latitude(some_int32_t);
	phome_position.longitude(some_int32_t);
	phome_position.altitude(some_int32_t);
	phome_position.x(some_float);
	phome_position.y(some_float);
	phome_position.z(some_float);
	phome_position.q(&some_float);
	phome_position.approach_x(some_float);
	phome_position.approach_y(some_float);
	phome_position.approach_z(some_float);
	
	const auto src_time_usec = phome_position.time_usec();
	
	if (!src_time_usec.IS_EXISTS)
		phome_position.time_usec(some_int64_t);
	
	
}


void on_FENCE_STATUS(const FENCE_STATUS &pfence_status) {
	auto some_breach_status = pfence_status.breach_status();
	auto some_breach_count  = pfence_status.breach_count();
	
	const auto src_breach_type = pfence_status.breach_type();
	
	if (src_breach_type.IS_EXISTS)
		auto some_breach_type = src_breach_type.CASE.EXISTS.value;
	auto     some_breach_time = pfence_status.breach_time();
	
}


void fill(const FENCE_STATUS &pfence_status) {
	pfence_status.breach_status(some_int8_t);
	pfence_status.breach_count(some_int16_t);
	
	const auto src_breach_type = pfence_status.breach_type();
	
	if (!src_breach_type.IS_EXISTS)
		pfence_status.breach_type(com::company::demo::FENCE_BREACH::FENCE_BREACH_NONE);
	
	pfence_status.breach_time(some_int32_t);
	
}


void on_REMOTE_LOG_BLOCK_STATUS(const REMOTE_LOG_BLOCK_STATUS &premote_log_block_status) {
	auto some_target_system    = premote_log_block_status.target_system();
	auto some_target_component = premote_log_block_status.target_component();
	auto some_seqno            = premote_log_block_status.seqno();
	
	const auto src_status = premote_log_block_status.status();
	
	if (src_status.IS_EXISTS)
		auto some_status = src_status.CASE.EXISTS.value;
	
}


void fill(const REMOTE_LOG_BLOCK_STATUS &premote_log_block_status) {
	premote_log_block_status.target_system(some_int8_t);
	premote_log_block_status.target_component(some_int8_t);
	premote_log_block_status.seqno(some_int32_t);
	
	const auto src_status = premote_log_block_status.status();
	
	if (!src_status.IS_EXISTS)
		premote_log_block_status.status(com::company::demo::MAV_REMOTE_LOG_DATA_BLOCK_STATUSES::MAV_REMOTE_LOG_DATA_BLOCK_NACK);
	
	
}


void on_OBSTACLE_DISTANCE(const OBSTACLE_DISTANCE &pobstacle_distance) {
	auto some_time_usec = pobstacle_distance.time_usec();
	
	const auto src_sensor_type = pobstacle_distance.sensor_type();
	
	if (src_sensor_type.IS_EXISTS)
		auto some_sensor_type = src_sensor_type.CASE.EXISTS.value;
	
	const auto  src_distances = pobstacle_distance.distances();
	for (size_t index         = 0; index < OBSTACLE_DISTANCE::distances_::len; index++)
		some_int16_t              = src_distances.get(index);
	auto        some_increment    = pobstacle_distance.increment();
	auto        some_min_distance = pobstacle_distance.min_distance();
	auto        some_max_distance = pobstacle_distance.max_distance();
	
}


void fill(const OBSTACLE_DISTANCE &pobstacle_distance) {
	pobstacle_distance.time_usec(some_int64_t);
	
	const auto src_sensor_type = pobstacle_distance.sensor_type();
	
	if (!src_sensor_type.IS_EXISTS)
		pobstacle_distance.sensor_type(com::company::demo::MAV_DISTANCE_SENSOR::MAV_DISTANCE_SENSOR_LASER);
	
	pobstacle_distance.distances(&some_int16_t);
	pobstacle_distance.increment(some_int8_t);
	pobstacle_distance.min_distance(some_int16_t);
	pobstacle_distance.max_distance(some_int16_t);
	
}


void on_GPS2_RAW(const GPS2_RAW &pgps2_raw) {
	auto some_time_usec = pgps2_raw.time_usec();
	
	const auto src_fix_type = pgps2_raw.fix_type();
	
	if (src_fix_type.IS_EXISTS)
		auto some_fix_type           = src_fix_type.CASE.EXISTS.value;
	auto     some_lat                = pgps2_raw.lat();
	auto     some_lon                = pgps2_raw.lon();
	auto     some_alt                = pgps2_raw.alt();
	auto     some_eph                = pgps2_raw.eph();
	auto     some_epv                = pgps2_raw.epv();
	auto     some_vel                = pgps2_raw.vel();
	auto     some_cog                = pgps2_raw.cog();
	auto     some_satellites_visible = pgps2_raw.satellites_visible();
	auto     some_dgps_numch         = pgps2_raw.dgps_numch();
	auto     some_dgps_age           = pgps2_raw.dgps_age();
	
}


void fill(const GPS2_RAW &pgps2_raw) {
	pgps2_raw.time_usec(some_int64_t);
	
	const auto src_fix_type = pgps2_raw.fix_type();
	
	if (!src_fix_type.IS_EXISTS)
		pgps2_raw.fix_type(com::company::demo::GPS_FIX_TYPE::GPS_FIX_TYPE_NO_GPS);
	
	pgps2_raw.lat(some_int32_t);
	pgps2_raw.lon(some_int32_t);
	pgps2_raw.alt(some_int32_t);
	pgps2_raw.eph(some_int16_t);
	pgps2_raw.epv(some_int16_t);
	pgps2_raw.vel(some_int16_t);
	pgps2_raw.cog(some_int16_t);
	pgps2_raw.satellites_visible(some_int8_t);
	pgps2_raw.dgps_numch(some_int8_t);
	pgps2_raw.dgps_age(some_int32_t);
	
}


void on_REQUEST_DATA_STREAM(const REQUEST_DATA_STREAM &prequest_data_stream) {
	auto some_target_system    = prequest_data_stream.target_system();
	auto some_target_component = prequest_data_stream.target_component();
	auto some_req_stream_id    = prequest_data_stream.req_stream_id();
	auto some_req_message_rate = prequest_data_stream.req_message_rate();
	auto some_start_stop       = prequest_data_stream.start_stop();
	
}


void on_MEMORY_VECT(const MEMORY_VECT &pmemory_vect) {
	auto some_address = pmemory_vect.address();
	auto some_ver     = pmemory_vect.ver();
	auto some_typE    = pmemory_vect.typE();
	
	const auto  src_value = pmemory_vect.value();
	for (size_t index     = 0; index < MEMORY_VECT::value_::len; index++)
		some_int8_t = src_value.get(index);
	
}


void fill(const MEMORY_VECT &pmemory_vect) {
	pmemory_vect.address(some_int16_t);
	pmemory_vect.ver(some_int8_t);
	pmemory_vect.typE(some_int8_t);
	pmemory_vect.value(&some_int8_t);
}


void on_PARAM_EXT_REQUEST_READ(const PARAM_EXT_REQUEST_READ &pparam_ext_request_read) {
	auto some_target_system    = pparam_ext_request_read.target_system();
	auto some_target_component = pparam_ext_request_read.target_component();
	
	const auto src_param_id     = pparam_ext_request_read.param_id();
	if (src_param_id.IS_EXISTS) {
		auto        array = src_param_id.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_param_index = pparam_ext_request_read.param_index();
	
}


void fill(const PARAM_EXT_REQUEST_READ &pparam_ext_request_read) {
	pparam_ext_request_read.target_system(some_int8_t);
	pparam_ext_request_read.target_component(some_int8_t);
	
	const auto src_param_id = pparam_ext_request_read.param_id();
	
	if (!src_param_id.IS_EXISTS)
		pparam_ext_request_read.param_id(some_string);
	
	pparam_ext_request_read.param_index(some_int16_t);
	
}


void on_HIL_CONTROLS(const HIL_CONTROLS &phil_controls) {
	auto some_time_usec      = phil_controls.time_usec();
	auto some_roll_ailerons  = phil_controls.roll_ailerons();
	auto some_pitch_elevator = phil_controls.pitch_elevator();
	auto some_yaw_rudder     = phil_controls.yaw_rudder();
	auto some_throttle       = phil_controls.throttle();
	auto some_aux1           = phil_controls.aux1();
	auto some_aux2           = phil_controls.aux2();
	auto some_aux3           = phil_controls.aux3();
	auto some_aux4           = phil_controls.aux4();
	
	const auto src_mode = phil_controls.mode();
	
	if (src_mode.IS_EXISTS)
		auto some_mode     = src_mode.CASE.EXISTS.value;
	auto     some_nav_mode = phil_controls.nav_mode();
	
}


void on_HIL_SENSOR(const HIL_SENSOR &phil_sensor) {
	auto some_time_usec      = phil_sensor.time_usec();
	auto some_xacc           = phil_sensor.xacc();
	auto some_yacc           = phil_sensor.yacc();
	auto some_zacc           = phil_sensor.zacc();
	auto some_xgyro          = phil_sensor.xgyro();
	auto some_ygyro          = phil_sensor.ygyro();
	auto some_zgyro          = phil_sensor.zgyro();
	auto some_xmag           = phil_sensor.xmag();
	auto some_ymag           = phil_sensor.ymag();
	auto some_zmag           = phil_sensor.zmag();
	auto some_abs_pressure   = phil_sensor.abs_pressure();
	auto some_diff_pressure  = phil_sensor.diff_pressure();
	auto some_pressure_alt   = phil_sensor.pressure_alt();
	auto some_temperature    = phil_sensor.temperature();
	auto some_fields_updated = phil_sensor.fields_updated();
	
}


void fill(const HIL_SENSOR &phil_sensor) {
	phil_sensor.time_usec(some_int64_t);
	phil_sensor.xacc(some_float);
	phil_sensor.yacc(some_float);
	phil_sensor.zacc(some_float);
	phil_sensor.xgyro(some_float);
	phil_sensor.ygyro(some_float);
	phil_sensor.zgyro(some_float);
	phil_sensor.xmag(some_float);
	phil_sensor.ymag(some_float);
	phil_sensor.zmag(some_float);
	phil_sensor.abs_pressure(some_float);
	phil_sensor.diff_pressure(some_float);
	phil_sensor.pressure_alt(some_float);
	phil_sensor.temperature(some_float);
	phil_sensor.fields_updated(some_int32_t);
	
}


void on_SETUP_SIGNING(const SETUP_SIGNING &psetup_signing) {
	auto some_target_system    = psetup_signing.target_system();
	auto some_target_component = psetup_signing.target_component();
	
	const auto  src_secret_key = psetup_signing.secret_key();
	for (size_t index          = 0; index < SETUP_SIGNING::secret_key_::len; index++)
		some_int8_t                    = src_secret_key.get(index);
	auto        some_initial_timestamp = psetup_signing.initial_timestamp();
	
}


void fill(const SETUP_SIGNING &psetup_signing) {
	psetup_signing.target_system(some_int8_t);
	psetup_signing.target_component(some_int8_t);
	psetup_signing.secret_key(&some_int8_t);
	psetup_signing.initial_timestamp(some_int64_t);
	
}


void on_GPS_RTK(const GPS_RTK &pgps_rtk) {
	auto some_time_last_baseline_ms = pgps_rtk.time_last_baseline_ms();
	auto some_rtk_receiver_id       = pgps_rtk.rtk_receiver_id();
	auto some_wn                    = pgps_rtk.wn();
	auto some_tow                   = pgps_rtk.tow();
	auto some_rtk_health            = pgps_rtk.rtk_health();
	auto some_rtk_rate              = pgps_rtk.rtk_rate();
	auto some_nsats                 = pgps_rtk.nsats();
	auto some_baseline_coords_type  = pgps_rtk.baseline_coords_type();
	auto some_baseline_a_mm         = pgps_rtk.baseline_a_mm();
	auto some_baseline_b_mm         = pgps_rtk.baseline_b_mm();
	auto some_baseline_c_mm         = pgps_rtk.baseline_c_mm();
	auto some_accuracy              = pgps_rtk.accuracy();
	auto some_iar_num_hypotheses    = pgps_rtk.iar_num_hypotheses();
	
}


void fill(const GPS_RTK &pgps_rtk) {
	pgps_rtk.time_last_baseline_ms(some_int32_t);
	pgps_rtk.rtk_receiver_id(some_int8_t);
	pgps_rtk.wn(some_int16_t);
	pgps_rtk.tow(some_int32_t);
	pgps_rtk.rtk_health(some_int8_t);
	pgps_rtk.rtk_rate(some_int8_t);
	pgps_rtk.nsats(some_int8_t);
	pgps_rtk.baseline_coords_type(some_int8_t);
	pgps_rtk.baseline_a_mm(some_int32_t);
	pgps_rtk.baseline_b_mm(some_int32_t);
	pgps_rtk.baseline_c_mm(some_int32_t);
	pgps_rtk.accuracy(some_int32_t);
	pgps_rtk.iar_num_hypotheses(some_int32_t);
	
}


void on_PARAM_REQUEST_LIST(const PARAM_REQUEST_LIST &pparam_request_list) {
	auto some_target_system    = pparam_request_list.target_system();
	auto some_target_component = pparam_request_list.target_component();
	
}


void on_UAVIONIX_ADSB_OUT_CFG(const UAVIONIX_ADSB_OUT_CFG &puavionix_adsb_out_cfg) {
	auto some_ICAO = puavionix_adsb_out_cfg.ICAO();
	
	const auto src_callsign = puavionix_adsb_out_cfg.callsign();
	if (src_callsign.IS_EXISTS) {
		auto        array = src_callsign.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_emitterType = puavionix_adsb_out_cfg.emitterType();
	
	if (src_emitterType.IS_EXISTS)
		auto some_emitterType = src_emitterType.CASE.EXISTS.value;
	
	const auto src_aircraftSize = puavionix_adsb_out_cfg.aircraftSize();
	
	if (src_aircraftSize.IS_EXISTS)
		auto some_aircraftSize = src_aircraftSize.CASE.EXISTS.value;
	
	const auto src_gpsOffsetLat = puavionix_adsb_out_cfg.gpsOffsetLat();
	
	if (src_gpsOffsetLat.IS_EXISTS)
		auto some_gpsOffsetLat = src_gpsOffsetLat.CASE.EXISTS.value;
	
	const auto src_gpsOffsetLon = puavionix_adsb_out_cfg.gpsOffsetLon();
	
	if (src_gpsOffsetLon.IS_EXISTS)
		auto some_gpsOffsetLon = src_gpsOffsetLon.CASE.EXISTS.value;
	auto     some_stallSpeed   = puavionix_adsb_out_cfg.stallSpeed();
	
	const auto src_rfSelect = puavionix_adsb_out_cfg.rfSelect();
	
	if (src_rfSelect.IS_EXISTS)
		auto some_rfSelect = src_rfSelect.CASE.EXISTS.value;
	
}


void fill(const UAVIONIX_ADSB_OUT_CFG &puavionix_adsb_out_cfg) {
	puavionix_adsb_out_cfg.ICAO(some_int32_t);
	
	const auto src_callsign = puavionix_adsb_out_cfg.callsign();
	
	if (!src_callsign.IS_EXISTS)
		puavionix_adsb_out_cfg.callsign(some_string);
	
	
	const auto src_emitterType = puavionix_adsb_out_cfg.emitterType();
	
	if (!src_emitterType.IS_EXISTS)
		puavionix_adsb_out_cfg.emitterType(com::company::demo::ADSB_EMITTER_TYPE::ADSB_EMITTER_TYPE_NO_INFO);
	
	
	const auto src_aircraftSize = puavionix_adsb_out_cfg.aircraftSize();
	
	if (!src_aircraftSize.IS_EXISTS)
		puavionix_adsb_out_cfg.aircraftSize(com::company::demo::UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE::UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA);
	
	
	const auto src_gpsOffsetLat = puavionix_adsb_out_cfg.gpsOffsetLat();
	
	if (!src_gpsOffsetLat.IS_EXISTS)
		puavionix_adsb_out_cfg.gpsOffsetLat(com::company::demo::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT::NO_DATA);
	
	
	const auto src_gpsOffsetLon = puavionix_adsb_out_cfg.gpsOffsetLon();
	
	if (!src_gpsOffsetLon.IS_EXISTS)
		puavionix_adsb_out_cfg.gpsOffsetLon(com::company::demo::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
	
	puavionix_adsb_out_cfg.stallSpeed(some_int16_t);
	
	const auto src_rfSelect = puavionix_adsb_out_cfg.rfSelect();
	
	if (!src_rfSelect.IS_EXISTS)
		puavionix_adsb_out_cfg.rfSelect(com::company::demo::UAVIONIX_ADSB_OUT_RF_SELECT::UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY);
	
	
}


void on_LANDING_TARGET(const LANDING_TARGET &planding_target) {
	auto some_time_usec  = planding_target.time_usec();
	auto some_target_num = planding_target.target_num();
	
	const auto src_frame = planding_target.frame();
	
	if (src_frame.IS_EXISTS)
		auto some_frame    = src_frame.CASE.EXISTS.value;
	auto     some_angle_x  = planding_target.angle_x();
	auto     some_angle_y  = planding_target.angle_y();
	auto     some_distance = planding_target.distance();
	auto     some_size_x   = planding_target.size_x();
	auto     some_size_y   = planding_target.size_y();
	
	const auto src_x = planding_target.x();
	
	if (src_x.IS_EXISTS)
		auto some_x = src_x.CASE.EXISTS.value;
	
	const auto src_y = planding_target.y();
	
	if (src_y.IS_EXISTS)
		auto some_y = src_y.CASE.EXISTS.value;
	
	const auto src_z = planding_target.z();
	
	if (src_z.IS_EXISTS)
		auto some_z = src_z.CASE.EXISTS.value;
	LANDING_TARGET_q_d0(planding_target.) {
					
					auto some_q = src_q.CASE.EXISTS.value;
					
				}
	
	const auto src_typE = planding_target.typE();
	
	if (src_typE.IS_EXISTS)
		auto some_typE = src_typE.CASE.EXISTS.value;
	
	const auto src_position_valid = planding_target.position_valid();
	
	if (src_position_valid.IS_EXISTS)
		auto some_position_valid = src_position_valid.CASE.EXISTS.value;
	
}


void fill(const LANDING_TARGET &planding_target) {
	planding_target.time_usec(some_int64_t);
	planding_target.target_num(some_int8_t);
	
	const auto src_frame = planding_target.frame();
	
	if (!src_frame.IS_EXISTS)
		planding_target.frame(com::company::demo::MAV_FRAME::MAV_FRAME_GLOBAL);
	
	planding_target.angle_x(some_float);
	planding_target.angle_y(some_float);
	planding_target.distance(some_float);
	planding_target.size_x(some_float);
	planding_target.size_y(some_float);
	
	const auto src_x = planding_target.x();
	
	if (!src_x.IS_EXISTS)
		planding_target.x(some_float);
	
	
	const auto src_y = planding_target.y();
	
	if (!src_y.IS_EXISTS)
		planding_target.y(some_float);
	
	
	const auto src_z = planding_target.z();
	
	if (!src_z.IS_EXISTS)
		planding_target.z(some_float);
	
	
	for (size_t d0 = 0; d0 < com::company::demo::LANDING_TARGET::q_::d0; d0++) {
		
		planding_target.q(some_float, d0);
		
	}
	
	const auto src_typE = planding_target.typE();
	
	if (!src_typE.IS_EXISTS)
		planding_target.typE(com::company::demo::LANDING_TARGET_TYPE::LANDING_TARGET_TYPE_LIGHT_BEACON);
	
	
	const auto src_position_valid = planding_target.position_valid();
	
	if (!src_position_valid.IS_EXISTS)
		planding_target.position_valid(some_int8_t);
	
	
}


void on_SET_ACTUATOR_CONTROL_TARGET(const SET_ACTUATOR_CONTROL_TARGET &pset_actuator_control_target) {
	auto some_time_usec        = pset_actuator_control_target.time_usec();
	auto some_group_mlx        = pset_actuator_control_target.group_mlx();
	auto some_target_system    = pset_actuator_control_target.target_system();
	auto some_target_component = pset_actuator_control_target.target_component();
	
	const auto  src_controls = pset_actuator_control_target.controls();
	for (size_t index        = 0; index < SET_ACTUATOR_CONTROL_TARGET::controls_::len; index++)
		some_float = src_controls.get(index);
	
}


void fill(const SET_ACTUATOR_CONTROL_TARGET &pset_actuator_control_target) {
	pset_actuator_control_target.time_usec(some_int64_t);
	pset_actuator_control_target.group_mlx(some_int8_t);
	pset_actuator_control_target.target_system(some_int8_t);
	pset_actuator_control_target.target_component(some_int8_t);
	pset_actuator_control_target.controls(&some_float);
}


void on_CONTROL_SYSTEM_STATE(const CONTROL_SYSTEM_STATE &pcontrol_system_state) {
	auto some_time_usec = pcontrol_system_state.time_usec();
	auto some_x_acc     = pcontrol_system_state.x_acc();
	auto some_y_acc     = pcontrol_system_state.y_acc();
	auto some_z_acc     = pcontrol_system_state.z_acc();
	auto some_x_vel     = pcontrol_system_state.x_vel();
	auto some_y_vel     = pcontrol_system_state.y_vel();
	auto some_z_vel     = pcontrol_system_state.z_vel();
	auto some_x_pos     = pcontrol_system_state.x_pos();
	auto some_y_pos     = pcontrol_system_state.y_pos();
	auto some_z_pos     = pcontrol_system_state.z_pos();
	auto some_airspeed  = pcontrol_system_state.airspeed();
	
	const auto  src_vel_variance = pcontrol_system_state.vel_variance();
	for (size_t index            = 0; index < CONTROL_SYSTEM_STATE::vel_variance_::len; index++)
		some_float = src_vel_variance.get(index);
	
	const auto  src_pos_variance = pcontrol_system_state.pos_variance();
	for (size_t index            = 0; index < CONTROL_SYSTEM_STATE::pos_variance_::len; index++)
		some_float = src_pos_variance.get(index);
	
	const auto  src_q = pcontrol_system_state.q();
	for (size_t index = 0; index < CONTROL_SYSTEM_STATE::q_::len; index++)
		some_float              = src_q.get(index);
	auto        some_roll_rate  = pcontrol_system_state.roll_rate();
	auto        some_pitch_rate = pcontrol_system_state.pitch_rate();
	auto        some_yaw_rate   = pcontrol_system_state.yaw_rate();
	
}


void fill(const CONTROL_SYSTEM_STATE &pcontrol_system_state) {
	pcontrol_system_state.time_usec(some_int64_t);
	pcontrol_system_state.x_acc(some_float);
	pcontrol_system_state.y_acc(some_float);
	pcontrol_system_state.z_acc(some_float);
	pcontrol_system_state.x_vel(some_float);
	pcontrol_system_state.y_vel(some_float);
	pcontrol_system_state.z_vel(some_float);
	pcontrol_system_state.x_pos(some_float);
	pcontrol_system_state.y_pos(some_float);
	pcontrol_system_state.z_pos(some_float);
	pcontrol_system_state.airspeed(some_float);
	pcontrol_system_state.vel_variance(&some_float);
	pcontrol_system_state.pos_variance(&some_float);
	pcontrol_system_state.q(&some_float);
	pcontrol_system_state.roll_rate(some_float);
	pcontrol_system_state.pitch_rate(some_float);
	pcontrol_system_state.yaw_rate(some_float);
	
}


void on_SET_POSITION_TARGET_GLOBAL_INT(const SET_POSITION_TARGET_GLOBAL_INT &pset_position_target_global_int) {
	auto some_time_boot_ms     = pset_position_target_global_int.time_boot_ms();
	auto some_target_system    = pset_position_target_global_int.target_system();
	auto some_target_component = pset_position_target_global_int.target_component();
	
	const auto src_coordinate_frame = pset_position_target_global_int.coordinate_frame();
	
	if (src_coordinate_frame.IS_EXISTS)
		auto some_coordinate_frame = src_coordinate_frame.CASE.EXISTS.value;
	auto     some_type_mask        = pset_position_target_global_int.type_mask();
	auto     some_lat_int          = pset_position_target_global_int.lat_int();
	auto     some_lon_int          = pset_position_target_global_int.lon_int();
	auto     some_alt              = pset_position_target_global_int.alt();
	auto     some_vx               = pset_position_target_global_int.vx();
	auto     some_vy               = pset_position_target_global_int.vy();
	auto     some_vz               = pset_position_target_global_int.vz();
	auto     some_afx              = pset_position_target_global_int.afx();
	auto     some_afy              = pset_position_target_global_int.afy();
	auto     some_afz              = pset_position_target_global_int.afz();
	auto     some_yaw              = pset_position_target_global_int.yaw();
	auto     some_yaw_rate         = pset_position_target_global_int.yaw_rate();
	
}


void on_DATA32(const DATA32 &pdata32) {
	auto some_typE = pdata32.typE();
	auto some_len  = pdata32.len();
	
	const auto  src_daTa = pdata32.daTa();
	for (size_t index    = 0; index < DATA32::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const DATA32 &pdata32) {
	pdata32.typE(some_int8_t);
	pdata32.len(some_int8_t);
	pdata32.daTa(&some_int8_t);
}


void on_PING33(const PING33 &pping33) {
	
	const auto src_testBOOL = pping33.testBOOL();
	
	if (src_testBOOL.IS_EXISTS)
		auto some_testBOOL = src_testBOOL.CASE.EXISTS.value;
	
	const auto src_seq = pping33.seq();
	
	if (src_seq.IS_EXISTS)
		auto some_seq   = src_seq.CASE.EXISTS.value;
	auto     some_field = pping33.field();
	
	const auto is_field1_exists = pping33.field1();
	if (is_field1_exists.IS_EXISTS) {
		const auto case_field1_exists = is_field1_exists.CASE.EXISTS;
		some_size_t = case_field1_exists.d0();
		some_size_t = PING33::field1_::d0_max;
		some_size_t = PING33::field1_::d1;
		some_size_t = PING33::field1_::d2;
		
		PING33_field1_d0_d1_d2(pping33.) {
							auto some_field1 = case_field1_exists.get(d0, d1, d2);
							
						}
	}
	PING33_field12_d0_d1_d2(pping33.) {
						auto some_field12 = case_field12_exists.get(d0, d1, d2);
						
					}
	PING33_field13_d0_d1_d2(pping33.) {
							
							auto some_field13 = src_field13.CASE.EXISTS.value;
							
						}
	
	PING33_TTTT_d0_d1_d2 {
				auto some_TTTT = pping33.TTTT(d0, d1, d2);
				
			}
	
	const auto src_WWWWWWWW = pping33.WWWWWWWW();
	
	if (src_WWWWWWWW.IS_EXISTS)
		auto some_WWWWWWWW  = src_WWWWWWWW.CASE.EXISTS.value;
	auto     some_testBOOL2 = pping33.testBOOL2();
	auto     some_testBOOL3 = pping33.testBOOL3();
	auto     some_bit_field = pping33.bit_field();
	
	const auto src_bit_field2 = pping33.bit_field2();
	
	if (src_bit_field2.IS_EXISTS)
		auto some_bit_field2 = src_bit_field2.CASE.EXISTS.value;
	PING33_Field_Bits_d0_d1_d2(pping33.) {
						auto some_Field_Bits = case_Field_Bits_exists.get(d0, d1, d2);
						
					}
	
	const auto is_SparseFixAllBits_exists = pping33.SparseFixAllBits();
	if (is_SparseFixAllBits_exists.IS_EXISTS) {
		const auto case_SparseFixAllBits_exists = is_SparseFixAllBits_exists.CASE.EXISTS;
		some_size_t = case_SparseFixAllBits_exists.d0();
		some_size_t = PING33::SparseFixAllBits_::d0_max;
		some_size_t = PING33::SparseFixAllBits_::d1;
		some_size_t = PING33::SparseFixAllBits_::d2;
		
		PING33_SparseFixAllBits_d0_d1_d2(pping33.) {
								
								auto some_SparseFixAllBits = src_SparseFixAllBits.CASE.EXISTS.value;
								
							}
	}
	
	const auto is_FixAllBits_exists = pping33.FixAllBits();
	if (is_FixAllBits_exists.IS_EXISTS) {
		const auto case_FixAllBits_exists = is_FixAllBits_exists.CASE.EXISTS;
		some_size_t = case_FixAllBits_exists.d0();
		some_size_t = PING33::FixAllBits_::d0_max;
		some_size_t = PING33::FixAllBits_::d1;
		some_size_t = PING33::FixAllBits_::d2;
		
		PING33_FixAllBits_d0_d1_d2(pping33.) {
							auto some_FixAllBits = case_FixAllBits_exists.get(d0, d1, d2);
							
						}
	}
	
	const auto is_VarAllBits_exists = pping33.VarAllBits();
	if (is_VarAllBits_exists.IS_EXISTS) {
		const auto case_VarAllBits_exists = is_VarAllBits_exists.CASE.EXISTS;
		some_size_t = case_VarAllBits_exists.d0();
		some_size_t = PING33::VarAllBits_::d0_max;
		some_size_t = PING33::VarAllBits_::d1;
		some_size_t = case_VarAllBits_exists.d2();
		some_size_t = PING33::VarAllBits_::d2_max;
		
		PING33_VarAllBits_d0_d1_d2(pping33.) {
							auto some_VarAllBits = case_VarAllBits_exists.get(d0, d1, d2);
							
						}
	}
	
	const auto is_SparseVarAllBits_exists = pping33.SparseVarAllBits();
	if (is_SparseVarAllBits_exists.IS_EXISTS) {
		const auto case_SparseVarAllBits_exists = is_SparseVarAllBits_exists.CASE.EXISTS;
		some_size_t = case_SparseVarAllBits_exists.d0();
		some_size_t = PING33::SparseVarAllBits_::d0_max;
		some_size_t = PING33::SparseVarAllBits_::d1;
		some_size_t = case_SparseVarAllBits_exists.d2();
		some_size_t = PING33::SparseVarAllBits_::d2_max;
		
		PING33_SparseVarAllBits_d0_d1_d2(pping33.) {
								
								auto some_SparseVarAllBits = src_SparseVarAllBits.CASE.EXISTS.value;
								
							}
	}
	
	const auto is_VarEachBits_exists = pping33.VarEachBits();
	if (is_VarEachBits_exists.IS_EXISTS) {
		const auto case_VarEachBits_exists = is_VarEachBits_exists.CASE.EXISTS;
		some_size_t = case_VarEachBits_exists.d0();
		some_size_t = PING33::VarEachBits_::d0_max;
		some_size_t = PING33::VarEachBits_::d1;
		some_size_t = PING33::VarEachBits_::d2;
		
		PING33_VarEachBits_d0_d1_d2(pping33.) {
							auto some_VarEachBits = case_VarEachBits_exists.get(d0, d1, d2);
							
						}
	}
	
	const auto is_SparsVarEachBits_exists = pping33.SparsVarEachBits();
	if (is_SparsVarEachBits_exists.IS_EXISTS) {
		const auto case_SparsVarEachBits_exists = is_SparsVarEachBits_exists.CASE.EXISTS;
		some_size_t = case_SparsVarEachBits_exists.d0();
		some_size_t = PING33::SparsVarEachBits_::d0_max;
		some_size_t = PING33::SparsVarEachBits_::d1;
		some_size_t = PING33::SparsVarEachBits_::d2;
		
		PING33_SparsVarEachBits_d0_d1_d2(pping33.) {
								
								auto some_SparsVarEachBits = src_SparsVarEachBits.CASE.EXISTS.value;
								
							}
	}
	
	const auto src_testBOOLX = pping33.testBOOLX();
	
	if (src_testBOOLX.IS_EXISTS)
		auto some_testBOOLX = src_testBOOLX.CASE.EXISTS.value;
	
	const auto src_testBOOL2X = pping33.testBOOL2X();
	
	if (src_testBOOL2X.IS_EXISTS)
		auto some_testBOOL2X = src_testBOOL2X.CASE.EXISTS.value;
	
	const auto src_testBOOL3X = pping33.testBOOL3X();
	
	if (src_testBOOL3X.IS_EXISTS)
		auto some_testBOOL3X = src_testBOOL3X.CASE.EXISTS.value;
	
	const auto src_MMMMMM = pping33.MMMMMM();
	
	if (src_MMMMMM.IS_EXISTS)
		auto some_MMMMMM = src_MMMMMM.CASE.EXISTS.value;
	
	const auto is_field44_exists = pping33.field44();
	if (is_field44_exists.IS_EXISTS) {
		const auto case_field44_exists = is_field44_exists.CASE.EXISTS;
		some_size_t = PING33::field44_::d0;
		some_size_t = PING33::field44_::d1;
		some_size_t = case_field44_exists.d2();
		some_size_t = PING33::field44_::d2_max;
		
		PING33_field44_d0_d1_d2(pping33.) {
							auto some_field44 = case_field44_exists.get(d0, d1, d2);
							
						}
	}
	
	const auto is_field634_exists = pping33.field634();
	if (is_field634_exists.IS_EXISTS) {
		const auto case_field634_exists = is_field634_exists.CASE.EXISTS;
		some_size_t = PING33::field634_::d0;
		some_size_t = PING33::field634_::d1;
		some_size_t = case_field634_exists.d2();
		some_size_t = PING33::field634_::d2_max;
		
		PING33_field634_d0_d1_d2(pping33.) {
							auto some_field634 = case_field634_exists.get(d0, d1, d2);
							
						}
	}
	
	const auto is_field33344_exists = pping33.field33344();
	if (is_field33344_exists.IS_EXISTS) {
		const auto case_field33344_exists = is_field33344_exists.CASE.EXISTS;
		some_size_t = PING33::field33344_::d0;
		some_size_t = PING33::field33344_::d1;
		some_size_t = case_field33344_exists.d2();
		some_size_t = PING33::field33344_::d2_max;
		
		PING33_field33344_d0_d1_d2(pping33.) {
								
								auto some_field33344 = src_field33344.CASE.EXISTS.value;
								
							}
	}
	
	const auto is_field333634_exists = pping33.field333634();
	if (is_field333634_exists.IS_EXISTS) {
		const auto case_field333634_exists = is_field333634_exists.CASE.EXISTS;
		some_size_t = PING33::field333634_::d0;
		some_size_t = PING33::field333634_::d1;
		some_size_t = case_field333634_exists.d2();
		some_size_t = PING33::field333634_::d2_max;
		
		PING33_field333634_d0_d1_d2(pping33.) {
								
								auto some_field333634 = src_field333634.CASE.EXISTS.value;
								
							}
	}
	PING33_field___d0_d1_d2(pping33.) {
							
							auto some_field__ = src_field__.CASE.EXISTS.value;
							
						}
	
	PING33_field6_d0_d1_d2 {
				auto some_field6 = pping33.field6(d0, d1, d2);
				
			}
	
	const auto is_field63_exists = pping33.field63();
	if (is_field63_exists.IS_EXISTS) {
		const auto case_field63_exists = is_field63_exists.CASE.EXISTS;
		some_size_t = PING33::field63_::d0;
		some_size_t = PING33::field63_::d1;
		some_size_t = case_field63_exists.d2();
		some_size_t = PING33::field63_::d2_max;
		
		PING33_field63_d0_d1_d2(pping33.) {
							auto some_field63 = case_field63_exists.get(d0, d1, d2);
							
						}
	}
	PING33_uid2_d0(pping33.) {
					
					auto some_uid2 = src_uid2.CASE.EXISTS.value;
					
				}
	PING33_field2_d0_d1_d2(pping33.) {
							
							auto some_field2 = src_field2.CASE.EXISTS.value;
							
						}
	PING33_field4_d0_d1_d2(pping33.) {
							
							auto some_field4 = src_field4.CASE.EXISTS.value;
							
						}
	
	const auto src_stringtest1 = pping33.stringtest1();
	if (src_stringtest1.IS_EXISTS) {
		auto        array = src_stringtest1.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto is_stringtest2_exists = pping33.stringtest2();
	if (is_stringtest2_exists.IS_EXISTS) {
		const auto case_stringtest2_exists = is_stringtest2_exists.CASE.EXISTS;
		some_size_t = PING33::stringtest2_::d0;
		some_size_t = PING33::stringtest2_::d1;
		some_size_t = case_stringtest2_exists.d2();
		some_size_t = PING33::stringtest2_::d2_max;
		
		PING33_stringtest2_d0_d1_d2(pping33.) {
								
								auto        array = src_stringtest2.CASE.EXISTS;
								for (size_t index = 0; index < array.length; index++)
									some_string = array.get(index);
								
							}
	}
	
	const auto src_stringtest3 = pping33.stringtest3();
	if (src_stringtest3.IS_EXISTS) {
		auto        array = src_stringtest3.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_stringtest4 = pping33.stringtest4();
	if (src_stringtest4.IS_EXISTS) {
		auto        array = src_stringtest4.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void fill(const PING33 &pping33) {
	
	const auto src_testBOOL = pping33.testBOOL();
	
	if (!src_testBOOL.IS_EXISTS)
		pping33.testBOOL(some_bool);
	
	
	const auto src_seq = pping33.seq();
	
	if (!src_seq.IS_EXISTS)
		pping33.seq(some_int64_t);
	
	pping33.field(some_int64_t);
	
	if (!pping33.field1().IS_EXISTS) {
		const auto  dst_field1         = pping33.field1().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_field1_exists = pping33.field1().CASE.EXISTS;
		for (size_t d0                 = 0, max_d0 = case_field1_exists.d0(); d0 < max_d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::field1_::d1; d1++)
				for (size_t d2 = 0; d2 < com::company::demo::PING33::field1_::d2; d2++) {
					case_field1_exists.set(some_int32_t, d0, d1, d2);
					
				}
	}
	
	
	for (size_t d0 = 0; d0 < com::company::demo::PING33::field12_::d0; d0++)
		for (size_t d1 = 0; d1 < com::company::demo::PING33::field12_::d1; d1++)
			for (size_t d2 = 0; d2 < com::company::demo::PING33::field12_::d2; d2++) {
				pping33.field12(some_int32_t, d0, d1, d2);
				
			}
	
	
	for (size_t d0 = 0; d0 < com::company::demo::PING33::field13_::d0; d0++)
		for (size_t d1 = 0; d1 < com::company::demo::PING33::field13_::d1; d1++)
			for (size_t d2 = 0; d2 < com::company::demo::PING33::field13_::d2; d2++) {
				
				pping33.field13(some_int32_t, d0, d1, d2);
				
			}
	for (size_t d0 = 0; d0 < com::company::demo::PING33::TTTT_::d0; d0++)
		for (size_t d1 = 0; d1 < com::company::demo::PING33::TTTT_::d1; d1++)
			for (size_t d2 = 0; d2 < com::company::demo::PING33::TTTT_::d2; d2++) {
				pping33.TTTT(some_int32_t, d0, d1, d2);
				
			}
	
	const auto src_WWWWWWWW = pping33.WWWWWWWW();
	
	if (!src_WWWWWWWW.IS_EXISTS)
		pping33.WWWWWWWW(some_int32_t);
	
	pping33.testBOOL2(some_bool);
	pping33.testBOOL3(some_bool);
	pping33.bit_field(some_int8_t);
	
	const auto src_bit_field2 = pping33.bit_field2();
	
	if (!src_bit_field2.IS_EXISTS)
		pping33.bit_field2(some_int8_t);
	
	
	for (size_t d0 = 0; d0 < com::company::demo::PING33::Field_Bits_::d0; d0++)
		for (size_t d1 = 0; d1 < com::company::demo::PING33::Field_Bits_::d1; d1++)
			for (size_t d2 = 0; d2 < com::company::demo::PING33::Field_Bits_::d2; d2++) {
				pping33.Field_Bits(some_int8_t, d0, d1, d2);
				
			}
	
	if (!pping33.SparseFixAllBits().IS_EXISTS) {
		const auto  dst_SparseFixAllBits         = pping33.SparseFixAllBits().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_SparseFixAllBits_exists = pping33.SparseFixAllBits().CASE.EXISTS;
		for (size_t d0                           = 0, max_d0 = case_SparseFixAllBits_exists.d0(); d0 < max_d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::SparseFixAllBits_::d1; d1++)
				for (size_t d2 = 0; d2 < com::company::demo::PING33::SparseFixAllBits_::d2; d2++) {
					
					case_SparseFixAllBits_exists.set(some_int8_t, d0, d1, d2);
					
				}
	}
	
	if (!pping33.FixAllBits().IS_EXISTS) {
		const auto  dst_FixAllBits         = pping33.FixAllBits().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_FixAllBits_exists = pping33.FixAllBits().CASE.EXISTS;
		for (size_t d0                     = 0, max_d0 = case_FixAllBits_exists.d0(); d0 < max_d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::FixAllBits_::d1; d1++)
				for (size_t d2 = 0; d2 < com::company::demo::PING33::FixAllBits_::d2; d2++) {
					case_FixAllBits_exists.set(some_int8_t, d0, d1, d2);
					
				}
	}
	
	if (!pping33.VarAllBits().IS_EXISTS) {
		const auto  dst_VarAllBits         = pping33.VarAllBits().CASE.EMPTY.init(some_size_t, some_size_t).CASE.EXISTS;
		const auto  case_VarAllBits_exists = pping33.VarAllBits().CASE.EXISTS;
		for (size_t d0                     = 0, max_d0 = case_VarAllBits_exists.d0(); d0 < max_d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::VarAllBits_::d1; d1++)
				for (size_t d2 = 0, max_d2 = case_VarAllBits_exists.d2(); d2 < max_d2; d2++) {
					case_VarAllBits_exists.set(some_int8_t, d0, d1, d2);
					
				}
	}
	
	if (!pping33.SparseVarAllBits().IS_EXISTS) {
		const auto  dst_SparseVarAllBits         = pping33.SparseVarAllBits().CASE.EMPTY.init(some_size_t, some_size_t).CASE.EXISTS;
		const auto  case_SparseVarAllBits_exists = pping33.SparseVarAllBits().CASE.EXISTS;
		for (size_t d0                           = 0, max_d0 = case_SparseVarAllBits_exists.d0(); d0 < max_d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::SparseVarAllBits_::d1; d1++)
				for (size_t d2 = 0, max_d2 = case_SparseVarAllBits_exists.d2(); d2 < max_d2; d2++) {
					
					case_SparseVarAllBits_exists.set(some_int8_t, d0, d1, d2);
					
				}
	}
	
	if (!pping33.VarEachBits().IS_EXISTS) {
		const auto  dst_VarEachBits         = pping33.VarEachBits().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_VarEachBits_exists = pping33.VarEachBits().CASE.EXISTS;
		for (size_t d0                      = 0, max_d0 = case_VarEachBits_exists.d0(); d0 < max_d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::VarEachBits_::d1; d1++)
				for (size_t d2 = 0; d2 < com::company::demo::PING33::VarEachBits_::d2; d2++) {
					case_VarEachBits_exists.set(some_int8_t, d0, d1, d2);
					
				}
	}
	
	if (!pping33.SparsVarEachBits().IS_EXISTS) {
		const auto  dst_SparsVarEachBits         = pping33.SparsVarEachBits().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_SparsVarEachBits_exists = pping33.SparsVarEachBits().CASE.EXISTS;
		for (size_t d0                           = 0, max_d0 = case_SparsVarEachBits_exists.d0(); d0 < max_d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::SparsVarEachBits_::d1; d1++)
				for (size_t d2 = 0; d2 < com::company::demo::PING33::SparsVarEachBits_::d2; d2++) {
					
					case_SparsVarEachBits_exists.set(some_int16_t, d0, d1, d2);
					
				}
	}
	
	const auto src_testBOOLX = pping33.testBOOLX();
	
	if (!src_testBOOLX.IS_EXISTS)
		pping33.testBOOLX(some_bool);
	
	
	const auto src_testBOOL2X = pping33.testBOOL2X();
	
	if (!src_testBOOL2X.IS_EXISTS)
		pping33.testBOOL2X(some_bool);
	
	
	const auto src_testBOOL3X = pping33.testBOOL3X();
	
	if (!src_testBOOL3X.IS_EXISTS)
		pping33.testBOOL3X(some_bool);
	
	
	const auto src_MMMMMM = pping33.MMMMMM();
	
	if (!src_MMMMMM.IS_EXISTS)
		pping33.MMMMMM(com::company::demo::MAV_MODE::PREFLIGHT);
	
	
	if (!pping33.field44().IS_EXISTS) {
		const auto  dst_field44         = pping33.field44().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_field44_exists = pping33.field44().CASE.EXISTS;
		for (size_t d0                  = 0; d0 < com::company::demo::PING33::field44_::d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::field44_::d1; d1++)
				for (size_t d2 = 0, max_d2 = case_field44_exists.d2(); d2 < max_d2; d2++) {
					case_field44_exists.set(some_int32_t, d0, d1, d2);
					
				}
	}
	
	if (!pping33.field634().IS_EXISTS) {
		const auto  dst_field634         = pping33.field634().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_field634_exists = pping33.field634().CASE.EXISTS;
		for (size_t d0                   = 0; d0 < com::company::demo::PING33::field634_::d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::field634_::d1; d1++)
				for (size_t d2 = 0, max_d2 = case_field634_exists.d2(); d2 < max_d2; d2++) {
					case_field634_exists.set(some_int32_t, d0, d1, d2);
					
				}
	}
	
	if (!pping33.field33344().IS_EXISTS) {
		const auto  dst_field33344         = pping33.field33344().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_field33344_exists = pping33.field33344().CASE.EXISTS;
		for (size_t d0                     = 0; d0 < com::company::demo::PING33::field33344_::d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::field33344_::d1; d1++)
				for (size_t d2 = 0, max_d2 = case_field33344_exists.d2(); d2 < max_d2; d2++) {
					
					case_field33344_exists.set(some_int32_t, d0, d1, d2);
					
				}
	}
	
	if (!pping33.field333634().IS_EXISTS) {
		const auto  dst_field333634         = pping33.field333634().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_field333634_exists = pping33.field333634().CASE.EXISTS;
		for (size_t d0                      = 0; d0 < com::company::demo::PING33::field333634_::d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::field333634_::d1; d1++)
				for (size_t d2 = 0, max_d2 = case_field333634_exists.d2(); d2 < max_d2; d2++) {
					
					case_field333634_exists.set(some_int32_t, d0, d1, d2);
					
				}
	}
	
	
	for (size_t d0 = 0; d0 < com::company::demo::PING33::field___::d0; d0++)
		for (size_t d1 = 0; d1 < com::company::demo::PING33::field___::d1; d1++)
			for (size_t d2 = 0; d2 < com::company::demo::PING33::field___::d2; d2++) {
				
				pping33.field__(some_int32_t, d0, d1, d2);
				
			}
	for (size_t d0 = 0; d0 < com::company::demo::PING33::field6_::d0; d0++)
		for (size_t d1 = 0; d1 < com::company::demo::PING33::field6_::d1; d1++)
			for (size_t d2 = 0; d2 < com::company::demo::PING33::field6_::d2; d2++) {
				pping33.field6(some_int32_t, d0, d1, d2);
				
			}
	
	if (!pping33.field63().IS_EXISTS) {
		const auto  dst_field63         = pping33.field63().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_field63_exists = pping33.field63().CASE.EXISTS;
		for (size_t d0                  = 0; d0 < com::company::demo::PING33::field63_::d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::field63_::d1; d1++)
				for (size_t d2 = 0, max_d2 = case_field63_exists.d2(); d2 < max_d2; d2++) {
					case_field63_exists.set(some_int32_t, d0, d1, d2);
					
				}
	}
	
	
	for (size_t d0 = 0; d0 < com::company::demo::PING33::uid2_::d0; d0++) {
		
		pping33.uid2(some_int8_t, d0);
		
	}
	
	
	for (size_t d0 = 0; d0 < com::company::demo::PING33::field2_::d0; d0++)
		for (size_t d1 = 0; d1 < com::company::demo::PING33::field2_::d1; d1++)
			for (size_t d2 = 0; d2 < com::company::demo::PING33::field2_::d2; d2++) {
				
				pping33.field2(some_int32_t, d0, d1, d2);
				
			}
	
	
	for (size_t d0 = 0; d0 < com::company::demo::PING33::field4_::d0; d0++)
		for (size_t d1 = 0; d1 < com::company::demo::PING33::field4_::d1; d1++)
			for (size_t d2 = 0; d2 < com::company::demo::PING33::field4_::d2; d2++) {
				
				pping33.field4(some_int32_t, d0, d1, d2);
				
			}
	
	const auto src_stringtest1 = pping33.stringtest1();
	
	if (!src_stringtest1.IS_EXISTS)
		pping33.stringtest1(some_string);
	
	
	if (!pping33.stringtest2().IS_EXISTS) {
		const auto  dst_stringtest2         = pping33.stringtest2().CASE.EMPTY.init(some_size_t).CASE.EXISTS;
		const auto  case_stringtest2_exists = pping33.stringtest2().CASE.EXISTS;
		for (size_t d0                      = 0; d0 < com::company::demo::PING33::stringtest2_::d0; d0++)
			for (size_t d1 = 0; d1 < com::company::demo::PING33::stringtest2_::d1; d1++)
				for (size_t d2 = 0, max_d2 = case_stringtest2_exists.d2(); d2 < max_d2; d2++) {
					
					case_stringtest2_exists.set(some_string, d0, d1, d2);
					
				}
	}
	
	const auto src_stringtest3 = pping33.stringtest3();
	
	if (!src_stringtest3.IS_EXISTS)
		pping33.stringtest3(some_string);
	
	
	const auto src_stringtest4 = pping33.stringtest4();
	
	if (!src_stringtest4.IS_EXISTS)
		pping33.stringtest4(some_string);
	
	
}


void on_VFR_HUD(const VFR_HUD &pvfr_hud) {
	auto some_airspeed    = pvfr_hud.airspeed();
	auto some_groundspeed = pvfr_hud.groundspeed();
	auto some_heading     = pvfr_hud.heading();
	auto some_throttle    = pvfr_hud.throttle();
	auto some_alt         = pvfr_hud.alt();
	auto some_climb       = pvfr_hud.climb();
	
}


void on_RALLY_POINT(const RALLY_POINT &prally_point) {
	auto some_target_system    = prally_point.target_system();
	auto some_target_component = prally_point.target_component();
	auto some_idx              = prally_point.idx();
	auto some_count            = prally_point.count();
	auto some_lat              = prally_point.lat();
	auto some_lng              = prally_point.lng();
	auto some_alt              = prally_point.alt();
	auto some_break_alt        = prally_point.break_alt();
	auto some_land_dir         = prally_point.land_dir();
	
	const auto src_flags = prally_point.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags = src_flags.CASE.EXISTS.value;
	
}


void fill(const RALLY_POINT &prally_point) {
	prally_point.target_system(some_int8_t);
	prally_point.target_component(some_int8_t);
	prally_point.idx(some_int8_t);
	prally_point.count(some_int8_t);
	prally_point.lat(some_int32_t);
	prally_point.lng(some_int32_t);
	prally_point.alt(some_int16_t);
	prally_point.break_alt(some_int16_t);
	prally_point.land_dir(some_int16_t);
	
	const auto src_flags = prally_point.flags();
	
	if (!src_flags.IS_EXISTS)
		prally_point.flags(com::company::demo::RALLY_FLAGS::FAVORABLE_WIND);
	
	
}


void on_MISSION_SET_CURRENT(const MISSION_SET_CURRENT &pmission_set_current) {
	auto some_target_system    = pmission_set_current.target_system();
	auto some_target_component = pmission_set_current.target_component();
	auto some_seq              = pmission_set_current.seq();
	
}


void on_ADAP_TUNING(const ADAP_TUNING &padap_tuning) {
	
	const auto src_axis = padap_tuning.axis();
	
	if (src_axis.IS_EXISTS)
		auto some_axis      = src_axis.CASE.EXISTS.value;
	auto     some_desired   = padap_tuning.desired();
	auto     some_achieved  = padap_tuning.achieved();
	auto     some_error     = padap_tuning.error();
	auto     some_theta     = padap_tuning.theta();
	auto     some_omega     = padap_tuning.omega();
	auto     some_sigma     = padap_tuning.sigma();
	auto     some_theta_dot = padap_tuning.theta_dot();
	auto     some_omega_dot = padap_tuning.omega_dot();
	auto     some_sigma_dot = padap_tuning.sigma_dot();
	auto     some_f         = padap_tuning.f();
	auto     some_f_dot     = padap_tuning.f_dot();
	auto     some_u         = padap_tuning.u();
	
}


void fill(const ADAP_TUNING &padap_tuning) {
	
	const auto src_axis = padap_tuning.axis();
	
	if (!src_axis.IS_EXISTS)
		padap_tuning.axis(com::company::demo::PID_TUNING_AXIS::PID_TUNING_ROLL);
	
	padap_tuning.desired(some_float);
	padap_tuning.achieved(some_float);
	padap_tuning.error(some_float);
	padap_tuning.theta(some_float);
	padap_tuning.omega(some_float);
	padap_tuning.sigma(some_float);
	padap_tuning.theta_dot(some_float);
	padap_tuning.omega_dot(some_float);
	padap_tuning.sigma_dot(some_float);
	padap_tuning.f(some_float);
	padap_tuning.f_dot(some_float);
	padap_tuning.u(some_float);
	
}


void on_VIBRATION(const VIBRATION &pvibration) {
	auto some_time_usec   = pvibration.time_usec();
	auto some_vibration_x = pvibration.vibration_x();
	auto some_vibration_y = pvibration.vibration_y();
	auto some_vibration_z = pvibration.vibration_z();
	auto some_clipping_0  = pvibration.clipping_0();
	auto some_clipping_1  = pvibration.clipping_1();
	auto some_clipping_2  = pvibration.clipping_2();
	
}


void fill(const VIBRATION &pvibration) {
	pvibration.time_usec(some_int64_t);
	pvibration.vibration_x(some_float);
	pvibration.vibration_y(some_float);
	pvibration.vibration_z(some_float);
	pvibration.clipping_0(some_int32_t);
	pvibration.clipping_1(some_int32_t);
	pvibration.clipping_2(some_int32_t);
	
}


void on_PARAM_EXT_VALUE(const PARAM_EXT_VALUE &pparam_ext_value) {
	
	const auto src_param_id = pparam_ext_value.param_id();
	if (src_param_id.IS_EXISTS) {
		auto        array = src_param_id.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_param_value = pparam_ext_value.param_value();
	if (src_param_value.IS_EXISTS) {
		auto        array = src_param_value.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_param_type = pparam_ext_value.param_type();
	
	if (src_param_type.IS_EXISTS)
		auto some_param_type  = src_param_type.CASE.EXISTS.value;
	auto     some_param_count = pparam_ext_value.param_count();
	auto     some_param_index = pparam_ext_value.param_index();
	
}


void fill(const PARAM_EXT_VALUE &pparam_ext_value) {
	
	const auto src_param_id = pparam_ext_value.param_id();
	
	if (!src_param_id.IS_EXISTS)
		pparam_ext_value.param_id(some_string);
	
	
	const auto src_param_value = pparam_ext_value.param_value();
	
	if (!src_param_value.IS_EXISTS)
		pparam_ext_value.param_value(some_string);
	
	
	const auto src_param_type = pparam_ext_value.param_type();
	
	if (!src_param_type.IS_EXISTS)
		pparam_ext_value.param_type(com::company::demo::MAV_PARAM_EXT_TYPE::MAV_PARAM_EXT_TYPE_UINT8);
	
	pparam_ext_value.param_count(some_int16_t);
	pparam_ext_value.param_index(some_int16_t);
	
}


void on_BATTERY2(const BATTERY2 &pbattery2) {
	auto some_voltage         = pbattery2.voltage();
	auto some_current_battery = pbattery2.current_battery();
	
}


void fill(const BATTERY2 &pbattery2) {
	pbattery2.voltage(some_int16_t);
	pbattery2.current_battery(some_int16_t);
	
}


void on_LIMITS_STATUS(const LIMITS_STATUS &plimits_status) {
	
	const auto src_limits_state = plimits_status.limits_state();
	
	if (src_limits_state.IS_EXISTS)
		auto some_limits_state  = src_limits_state.CASE.EXISTS.value;
	auto     some_last_trigger  = plimits_status.last_trigger();
	auto     some_last_action   = plimits_status.last_action();
	auto     some_last_recovery = plimits_status.last_recovery();
	auto     some_last_clear    = plimits_status.last_clear();
	auto     some_breach_count  = plimits_status.breach_count();
	
	const auto src_mods_enabled = plimits_status.mods_enabled();
	
	if (src_mods_enabled.IS_EXISTS)
		auto some_mods_enabled = src_mods_enabled.CASE.EXISTS.value;
	
	const auto src_mods_required = plimits_status.mods_required();
	
	if (src_mods_required.IS_EXISTS)
		auto some_mods_required = src_mods_required.CASE.EXISTS.value;
	
	const auto src_mods_triggered = plimits_status.mods_triggered();
	
	if (src_mods_triggered.IS_EXISTS)
		auto some_mods_triggered = src_mods_triggered.CASE.EXISTS.value;
	
}


void fill(const LIMITS_STATUS &plimits_status) {
	
	const auto src_limits_state = plimits_status.limits_state();
	
	if (!src_limits_state.IS_EXISTS)
		plimits_status.limits_state(com::company::demo::LIMITS_STATE::LIMITS_INIT);
	
	plimits_status.last_trigger(some_int32_t);
	plimits_status.last_action(some_int32_t);
	plimits_status.last_recovery(some_int32_t);
	plimits_status.last_clear(some_int32_t);
	plimits_status.breach_count(some_int16_t);
	
	const auto src_mods_enabled = plimits_status.mods_enabled();
	
	if (!src_mods_enabled.IS_EXISTS)
		plimits_status.mods_enabled(com::company::demo::LIMIT_MODULE::LIMIT_GPSLOCK);
	
	
	const auto src_mods_required = plimits_status.mods_required();
	
	if (!src_mods_required.IS_EXISTS)
		plimits_status.mods_required(com::company::demo::LIMIT_MODULE::LIMIT_GPSLOCK);
	
	
	const auto src_mods_triggered = plimits_status.mods_triggered();
	
	if (!src_mods_triggered.IS_EXISTS)
		plimits_status.mods_triggered(com::company::demo::LIMIT_MODULE::LIMIT_GPSLOCK);
	
	
}


void on_CAMERA_FEEDBACK(const CAMERA_FEEDBACK &pcamera_feedback) {
	auto some_time_usec     = pcamera_feedback.time_usec();
	auto some_target_system = pcamera_feedback.target_system();
	auto some_cam_idx       = pcamera_feedback.cam_idx();
	auto some_img_idx       = pcamera_feedback.img_idx();
	auto some_lat           = pcamera_feedback.lat();
	auto some_lng           = pcamera_feedback.lng();
	auto some_alt_msl       = pcamera_feedback.alt_msl();
	auto some_alt_rel       = pcamera_feedback.alt_rel();
	auto some_roll          = pcamera_feedback.roll();
	auto some_pitch         = pcamera_feedback.pitch();
	auto some_yaw           = pcamera_feedback.yaw();
	auto some_foc_len       = pcamera_feedback.foc_len();
	
	const auto src_flags = pcamera_feedback.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags = src_flags.CASE.EXISTS.value;
	
}


void fill(const CAMERA_FEEDBACK &pcamera_feedback) {
	pcamera_feedback.time_usec(some_int64_t);
	pcamera_feedback.target_system(some_int8_t);
	pcamera_feedback.cam_idx(some_int8_t);
	pcamera_feedback.img_idx(some_int16_t);
	pcamera_feedback.lat(some_int32_t);
	pcamera_feedback.lng(some_int32_t);
	pcamera_feedback.alt_msl(some_float);
	pcamera_feedback.alt_rel(some_float);
	pcamera_feedback.roll(some_float);
	pcamera_feedback.pitch(some_float);
	pcamera_feedback.yaw(some_float);
	pcamera_feedback.foc_len(some_float);
	
	const auto src_flags = pcamera_feedback.flags();
	
	if (!src_flags.IS_EXISTS)
		pcamera_feedback.flags(com::company::demo::CAMERA_FEEDBACK_FLAGS::CAMERA_FEEDBACK_PHOTO);
	
	
}


void on_HIL_GPS(const HIL_GPS &phil_gps) {
	auto some_time_usec          = phil_gps.time_usec();
	auto some_fix_type           = phil_gps.fix_type();
	auto some_lat                = phil_gps.lat();
	auto some_lon                = phil_gps.lon();
	auto some_alt                = phil_gps.alt();
	auto some_eph                = phil_gps.eph();
	auto some_epv                = phil_gps.epv();
	auto some_vel                = phil_gps.vel();
	auto some_vn                 = phil_gps.vn();
	auto some_ve                 = phil_gps.ve();
	auto some_vd                 = phil_gps.vd();
	auto some_cog                = phil_gps.cog();
	auto some_satellites_visible = phil_gps.satellites_visible();
	
}


void fill(const HIL_GPS &phil_gps) {
	phil_gps.time_usec(some_int64_t);
	phil_gps.fix_type(some_int8_t);
	phil_gps.lat(some_int32_t);
	phil_gps.lon(some_int32_t);
	phil_gps.alt(some_int32_t);
	phil_gps.eph(some_int16_t);
	phil_gps.epv(some_int16_t);
	phil_gps.vel(some_int16_t);
	phil_gps.vn(some_int16_t);
	phil_gps.ve(some_int16_t);
	phil_gps.vd(some_int16_t);
	phil_gps.cog(some_int16_t);
	phil_gps.satellites_visible(some_int8_t);
	
}


void on_NAV_CONTROLLER_OUTPUT(const NAV_CONTROLLER_OUTPUT &pnav_controller_output) {
	auto some_nav_roll       = pnav_controller_output.nav_roll();
	auto some_nav_pitch      = pnav_controller_output.nav_pitch();
	auto some_nav_bearing    = pnav_controller_output.nav_bearing();
	auto some_target_bearing = pnav_controller_output.target_bearing();
	auto some_wp_dist        = pnav_controller_output.wp_dist();
	auto some_alt_error      = pnav_controller_output.alt_error();
	auto some_aspd_error     = pnav_controller_output.aspd_error();
	auto some_xtrack_error   = pnav_controller_output.xtrack_error();
	
}


void on_AUTH_KEY(const AUTH_KEY &pauth_key) {
	
	const auto src_key = pauth_key.key();
	if (src_key.IS_EXISTS) {
		auto        array = src_key.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void on_FENCE_FETCH_POINT(const FENCE_FETCH_POINT &pfence_fetch_point) {
	auto some_target_system    = pfence_fetch_point.target_system();
	auto some_target_component = pfence_fetch_point.target_component();
	auto some_idx              = pfence_fetch_point.idx();
	
}


void fill(const FENCE_FETCH_POINT &pfence_fetch_point) {
	pfence_fetch_point.target_system(some_int8_t);
	pfence_fetch_point.target_component(some_int8_t);
	pfence_fetch_point.idx(some_int8_t);
	
}


void on_RADIO(const RADIO &pradio) {
	auto some_rssi     = pradio.rssi();
	auto some_remrssi  = pradio.remrssi();
	auto some_txbuf    = pradio.txbuf();
	auto some_noise    = pradio.noise();
	auto some_remnoise = pradio.remnoise();
	auto some_rxerrors = pradio.rxerrors();
	auto some_fixeD    = pradio.fixeD();
	
}


void fill(const RADIO &pradio) {
	pradio.rssi(some_int8_t);
	pradio.remrssi(some_int8_t);
	pradio.txbuf(some_int8_t);
	pradio.noise(some_int8_t);
	pradio.remnoise(some_int8_t);
	pradio.rxerrors(some_int16_t);
	pradio.fixeD(some_int16_t);
	
}


void on_LOCAL_POSITION_NED_COV(const LOCAL_POSITION_NED_COV &plocal_position_ned_cov) {
	auto some_time_usec = plocal_position_ned_cov.time_usec();
	
	const auto src_estimator_type = plocal_position_ned_cov.estimator_type();
	
	if (src_estimator_type.IS_EXISTS)
		auto some_estimator_type = src_estimator_type.CASE.EXISTS.value;
	auto     some_x              = plocal_position_ned_cov.x();
	auto     some_y              = plocal_position_ned_cov.y();
	auto     some_z              = plocal_position_ned_cov.z();
	auto     some_vx             = plocal_position_ned_cov.vx();
	auto     some_vy             = plocal_position_ned_cov.vy();
	auto     some_vz             = plocal_position_ned_cov.vz();
	auto     some_ax             = plocal_position_ned_cov.ax();
	auto     some_ay             = plocal_position_ned_cov.ay();
	auto     some_az             = plocal_position_ned_cov.az();
	
	const auto  src_covariance = plocal_position_ned_cov.covariance();
	for (size_t index          = 0; index < LOCAL_POSITION_NED_COV::covariance_::len; index++)
		some_float = src_covariance.get(index);
	
}


void on_AIRSPEED_AUTOCAL(const AIRSPEED_AUTOCAL &pairspeed_autocal) {
	auto some_vx            = pairspeed_autocal.vx();
	auto some_vy            = pairspeed_autocal.vy();
	auto some_vz            = pairspeed_autocal.vz();
	auto some_diff_pressure = pairspeed_autocal.diff_pressure();
	auto some_EAS2TAS       = pairspeed_autocal.EAS2TAS();
	auto some_ratio         = pairspeed_autocal.ratio();
	auto some_state_x       = pairspeed_autocal.state_x();
	auto some_state_y       = pairspeed_autocal.state_y();
	auto some_state_z       = pairspeed_autocal.state_z();
	auto some_Pax           = pairspeed_autocal.Pax();
	auto some_Pby           = pairspeed_autocal.Pby();
	auto some_Pcz           = pairspeed_autocal.Pcz();
	
}


void fill(const AIRSPEED_AUTOCAL &pairspeed_autocal) {
	pairspeed_autocal.vx(some_float);
	pairspeed_autocal.vy(some_float);
	pairspeed_autocal.vz(some_float);
	pairspeed_autocal.diff_pressure(some_float);
	pairspeed_autocal.EAS2TAS(some_float);
	pairspeed_autocal.ratio(some_float);
	pairspeed_autocal.state_x(some_float);
	pairspeed_autocal.state_y(some_float);
	pairspeed_autocal.state_z(some_float);
	pairspeed_autocal.Pax(some_float);
	pairspeed_autocal.Pby(some_float);
	pairspeed_autocal.Pcz(some_float);
	
}


void on_ATT_POS_MOCAP(const ATT_POS_MOCAP &patt_pos_mocap) {
	auto some_time_usec = patt_pos_mocap.time_usec();
	
	const auto  src_q = patt_pos_mocap.q();
	for (size_t index = 0; index < ATT_POS_MOCAP::q_::len; index++)
		some_float     = src_q.get(index);
	auto        some_x = patt_pos_mocap.x();
	auto        some_y = patt_pos_mocap.y();
	auto        some_z = patt_pos_mocap.z();
	
}


void fill(const ATT_POS_MOCAP &patt_pos_mocap) {
	patt_pos_mocap.time_usec(some_int64_t);
	patt_pos_mocap.q(&some_float);
	patt_pos_mocap.x(some_float);
	patt_pos_mocap.y(some_float);
	patt_pos_mocap.z(some_float);
	
}


void on_STATUSTEXT(const STATUSTEXT &pstatustext) {
	
	const auto src_severity = pstatustext.severity();
	
	if (src_severity.IS_EXISTS)
		auto some_severity = src_severity.CASE.EXISTS.value;
	
	const auto src_text = pstatustext.text();
	if (src_text.IS_EXISTS) {
		auto        array = src_text.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void fill(const STATUSTEXT &pstatustext) {
	
	const auto src_severity = pstatustext.severity();
	
	if (!src_severity.IS_EXISTS)
		pstatustext.severity(com::company::demo::MAV_SEVERITY::MAV_SEVERITY_EMERGENCY);
	
	
	const auto src_text = pstatustext.text();
	
	if (!src_text.IS_EXISTS)
		pstatustext.text(some_string);
	
	
}


void on_PING(const PING &pping) {
	auto some_time_usec        = pping.time_usec();
	auto some_seq              = pping.seq();
	auto some_target_system    = pping.target_system();
	auto some_target_component = pping.target_component();
	
}


void on_GOPRO_GET_REQUEST(const GOPRO_GET_REQUEST &pgopro_get_request) {
	auto some_target_system    = pgopro_get_request.target_system();
	auto some_target_component = pgopro_get_request.target_component();
	
	const auto src_cmd_id = pgopro_get_request.cmd_id();
	
	if (src_cmd_id.IS_EXISTS)
		auto some_cmd_id = src_cmd_id.CASE.EXISTS.value;
	
}


void fill(const GOPRO_GET_REQUEST &pgopro_get_request) {
	pgopro_get_request.target_system(some_int8_t);
	pgopro_get_request.target_component(some_int8_t);
	
	const auto src_cmd_id = pgopro_get_request.cmd_id();
	
	if (!src_cmd_id.IS_EXISTS)
		pgopro_get_request.cmd_id(com::company::demo::GOPRO_COMMAND::GOPRO_COMMAND_POWER);
	
	
}


void on_CAMERA_CAPTURE_STATUS(const CAMERA_CAPTURE_STATUS &pcamera_capture_status) {
	auto some_time_boot_ms       = pcamera_capture_status.time_boot_ms();
	auto some_image_status       = pcamera_capture_status.image_status();
	auto some_video_status       = pcamera_capture_status.video_status();
	auto some_image_interval     = pcamera_capture_status.image_interval();
	auto some_recording_time_ms  = pcamera_capture_status.recording_time_ms();
	auto some_available_capacity = pcamera_capture_status.available_capacity();
	
}


void fill(const CAMERA_CAPTURE_STATUS &pcamera_capture_status) {
	pcamera_capture_status.time_boot_ms(some_int32_t);
	pcamera_capture_status.image_status(some_int8_t);
	pcamera_capture_status.video_status(some_int8_t);
	pcamera_capture_status.image_interval(some_float);
	pcamera_capture_status.recording_time_ms(some_int32_t);
	pcamera_capture_status.available_capacity(some_float);
	
}


void on_GLOBAL_POSITION_INT(const GLOBAL_POSITION_INT &pglobal_position_int) {
	auto some_time_boot_ms = pglobal_position_int.time_boot_ms();
	auto some_lat          = pglobal_position_int.lat();
	auto some_lon          = pglobal_position_int.lon();
	auto some_alt          = pglobal_position_int.alt();
	auto some_relative_alt = pglobal_position_int.relative_alt();
	auto some_vx           = pglobal_position_int.vx();
	auto some_vy           = pglobal_position_int.vy();
	auto some_vz           = pglobal_position_int.vz();
	auto some_hdg          = pglobal_position_int.hdg();
	
}


void on_ENCAPSULATED_DATA(const ENCAPSULATED_DATA &pencapsulated_data) {
	auto some_seqnr = pencapsulated_data.seqnr();
	
	const auto  src_daTa = pencapsulated_data.daTa();
	for (size_t index    = 0; index < ENCAPSULATED_DATA::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const ENCAPSULATED_DATA &pencapsulated_data) {
	pencapsulated_data.seqnr(some_int16_t);
	pencapsulated_data.daTa(&some_int8_t);
}


void on_GPS_INPUT(const GPS_INPUT &pgps_input) {
	auto some_time_usec = pgps_input.time_usec();
	auto some_gps_id    = pgps_input.gps_id();
	
	const auto src_ignore_flags = pgps_input.ignore_flags();
	
	if (src_ignore_flags.IS_EXISTS)
		auto some_ignore_flags       = src_ignore_flags.CASE.EXISTS.value;
	auto     some_time_week_ms       = pgps_input.time_week_ms();
	auto     some_time_week          = pgps_input.time_week();
	auto     some_fix_type           = pgps_input.fix_type();
	auto     some_lat                = pgps_input.lat();
	auto     some_lon                = pgps_input.lon();
	auto     some_alt                = pgps_input.alt();
	auto     some_hdop               = pgps_input.hdop();
	auto     some_vdop               = pgps_input.vdop();
	auto     some_vn                 = pgps_input.vn();
	auto     some_ve                 = pgps_input.ve();
	auto     some_vd                 = pgps_input.vd();
	auto     some_speed_accuracy     = pgps_input.speed_accuracy();
	auto     some_horiz_accuracy     = pgps_input.horiz_accuracy();
	auto     some_vert_accuracy      = pgps_input.vert_accuracy();
	auto     some_satellites_visible = pgps_input.satellites_visible();
	
}


void fill(const GPS_INPUT &pgps_input) {
	pgps_input.time_usec(some_int64_t);
	pgps_input.gps_id(some_int8_t);
	
	const auto src_ignore_flags = pgps_input.ignore_flags();
	
	if (!src_ignore_flags.IS_EXISTS)
		pgps_input.ignore_flags(com::company::demo::GPS_INPUT_IGNORE_FLAGS::GPS_INPUT_IGNORE_FLAG_ALT);
	
	pgps_input.time_week_ms(some_int32_t);
	pgps_input.time_week(some_int16_t);
	pgps_input.fix_type(some_int8_t);
	pgps_input.lat(some_int32_t);
	pgps_input.lon(some_int32_t);
	pgps_input.alt(some_float);
	pgps_input.hdop(some_float);
	pgps_input.vdop(some_float);
	pgps_input.vn(some_float);
	pgps_input.ve(some_float);
	pgps_input.vd(some_float);
	pgps_input.speed_accuracy(some_float);
	pgps_input.horiz_accuracy(some_float);
	pgps_input.vert_accuracy(some_float);
	pgps_input.satellites_visible(some_int8_t);
	
}


void on_COMMAND_LONG(const COMMAND_LONG &pcommand_long) {
	auto some_target_system    = pcommand_long.target_system();
	auto some_target_component = pcommand_long.target_component();
	
	const auto src_command = pcommand_long.command();
	
	if (src_command.IS_EXISTS)
		auto some_command      = src_command.CASE.EXISTS.value;
	auto     some_confirmation = pcommand_long.confirmation();
	auto     some_param1       = pcommand_long.param1();
	auto     some_param2       = pcommand_long.param2();
	auto     some_param3       = pcommand_long.param3();
	auto     some_param4       = pcommand_long.param4();
	auto     some_param5       = pcommand_long.param5();
	auto     some_param6       = pcommand_long.param6();
	auto     some_param7       = pcommand_long.param7();
	
}


void on_COMPASSMOT_STATUS(const COMPASSMOT_STATUS &pcompassmot_status) {
	auto some_throttle      = pcompassmot_status.throttle();
	auto some_current       = pcompassmot_status.current();
	auto some_interference  = pcompassmot_status.interference();
	auto some_CompensationX = pcompassmot_status.CompensationX();
	auto some_CompensationY = pcompassmot_status.CompensationY();
	auto some_CompensationZ = pcompassmot_status.CompensationZ();
	
}


void fill(const COMPASSMOT_STATUS &pcompassmot_status) {
	pcompassmot_status.throttle(some_int16_t);
	pcompassmot_status.current(some_float);
	pcompassmot_status.interference(some_int16_t);
	pcompassmot_status.CompensationX(some_float);
	pcompassmot_status.CompensationY(some_float);
	pcompassmot_status.CompensationZ(some_float);
	
}


void on_LOG_REQUEST_DATA(const LOG_REQUEST_DATA &plog_request_data) {
	auto some_target_system    = plog_request_data.target_system();
	auto some_target_component = plog_request_data.target_component();
	auto some_id               = plog_request_data.id();
	auto some_ofs              = plog_request_data.ofs();
	auto some_count            = plog_request_data.count();
	
}


void fill(const LOG_REQUEST_DATA &plog_request_data) {
	plog_request_data.target_system(some_int8_t);
	plog_request_data.target_component(some_int8_t);
	plog_request_data.id(some_int16_t);
	plog_request_data.ofs(some_int32_t);
	plog_request_data.count(some_int32_t);
	
}


void on_GPS_RAW_INT(const GPS_RAW_INT &pgps_raw_int) {
	auto some_time_usec = pgps_raw_int.time_usec();
	
	const auto src_fix_type = pgps_raw_int.fix_type();
	
	if (src_fix_type.IS_EXISTS)
		auto some_fix_type           = src_fix_type.CASE.EXISTS.value;
	auto     some_lat                = pgps_raw_int.lat();
	auto     some_lon                = pgps_raw_int.lon();
	auto     some_alt                = pgps_raw_int.alt();
	auto     some_eph                = pgps_raw_int.eph();
	auto     some_epv                = pgps_raw_int.epv();
	auto     some_vel                = pgps_raw_int.vel();
	auto     some_cog                = pgps_raw_int.cog();
	auto     some_satellites_visible = pgps_raw_int.satellites_visible();
	
	const auto src_alt_ellipsoid = pgps_raw_int.alt_ellipsoid();
	
	if (src_alt_ellipsoid.IS_EXISTS)
		auto some_alt_ellipsoid = src_alt_ellipsoid.CASE.EXISTS.value;
	
	const auto src_h_acc = pgps_raw_int.h_acc();
	
	if (src_h_acc.IS_EXISTS)
		auto some_h_acc = src_h_acc.CASE.EXISTS.value;
	
	const auto src_v_acc = pgps_raw_int.v_acc();
	
	if (src_v_acc.IS_EXISTS)
		auto some_v_acc = src_v_acc.CASE.EXISTS.value;
	
	const auto src_vel_acc = pgps_raw_int.vel_acc();
	
	if (src_vel_acc.IS_EXISTS)
		auto some_vel_acc = src_vel_acc.CASE.EXISTS.value;
	
	const auto src_hdg_acc = pgps_raw_int.hdg_acc();
	
	if (src_hdg_acc.IS_EXISTS)
		auto some_hdg_acc = src_hdg_acc.CASE.EXISTS.value;
	
}


void on_CAMERA_STATUS(const CAMERA_STATUS &pcamera_status) {
	auto some_time_usec     = pcamera_status.time_usec();
	auto some_target_system = pcamera_status.target_system();
	auto some_cam_idx       = pcamera_status.cam_idx();
	auto some_img_idx       = pcamera_status.img_idx();
	
	const auto src_event_id = pcamera_status.event_id();
	
	if (src_event_id.IS_EXISTS)
		auto some_event_id = src_event_id.CASE.EXISTS.value;
	auto     some_p1       = pcamera_status.p1();
	auto     some_p2       = pcamera_status.p2();
	auto     some_p3       = pcamera_status.p3();
	auto     some_p4       = pcamera_status.p4();
	
}


void fill(const CAMERA_STATUS &pcamera_status) {
	pcamera_status.time_usec(some_int64_t);
	pcamera_status.target_system(some_int8_t);
	pcamera_status.cam_idx(some_int8_t);
	pcamera_status.img_idx(some_int16_t);
	
	const auto src_event_id = pcamera_status.event_id();
	
	if (!src_event_id.IS_EXISTS)
		pcamera_status.event_id(com::company::demo::CAMERA_STATUS_TYPES::CAMERA_STATUS_TYPE_HEARTBEAT);
	
	pcamera_status.p1(some_float);
	pcamera_status.p2(some_float);
	pcamera_status.p3(some_float);
	pcamera_status.p4(some_float);
	
}


void on_RC_CHANNELS_SCALED(const RC_CHANNELS_SCALED &prc_channels_scaled) {
	auto some_time_boot_ms = prc_channels_scaled.time_boot_ms();
	auto some_port         = prc_channels_scaled.port();
	auto some_chan1_scaled = prc_channels_scaled.chan1_scaled();
	auto some_chan2_scaled = prc_channels_scaled.chan2_scaled();
	auto some_chan3_scaled = prc_channels_scaled.chan3_scaled();
	auto some_chan4_scaled = prc_channels_scaled.chan4_scaled();
	auto some_chan5_scaled = prc_channels_scaled.chan5_scaled();
	auto some_chan6_scaled = prc_channels_scaled.chan6_scaled();
	auto some_chan7_scaled = prc_channels_scaled.chan7_scaled();
	auto some_chan8_scaled = prc_channels_scaled.chan8_scaled();
	auto some_rssi         = prc_channels_scaled.rssi();
	
}


void on_CAMERA_SETTINGS(const CAMERA_SETTINGS &pcamera_settings) {
	auto some_time_boot_ms = pcamera_settings.time_boot_ms();
	
	const auto src_mode_id = pcamera_settings.mode_id();
	
	if (src_mode_id.IS_EXISTS)
		auto some_mode_id = src_mode_id.CASE.EXISTS.value;
	
}


void fill(const CAMERA_SETTINGS &pcamera_settings) {
	pcamera_settings.time_boot_ms(some_int32_t);
	
	const auto src_mode_id = pcamera_settings.mode_id();
	
	if (!src_mode_id.IS_EXISTS)
		pcamera_settings.mode_id(com::company::demo::CAMERA_MODE::CAMERA_MODE_IMAGE);
	
	
}


void on_DEVICE_OP_READ_REPLY(const DEVICE_OP_READ_REPLY &pdevice_op_read_reply) {
	auto some_request_id = pdevice_op_read_reply.request_id();
	auto some_result     = pdevice_op_read_reply.result();
	auto some_regstart   = pdevice_op_read_reply.regstart();
	auto some_count      = pdevice_op_read_reply.count();
	
	const auto  src_daTa = pdevice_op_read_reply.daTa();
	for (size_t index    = 0; index < DEVICE_OP_READ_REPLY::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const DEVICE_OP_READ_REPLY &pdevice_op_read_reply) {
	pdevice_op_read_reply.request_id(some_int32_t);
	pdevice_op_read_reply.result(some_int8_t);
	pdevice_op_read_reply.regstart(some_int8_t);
	pdevice_op_read_reply.count(some_int8_t);
	pdevice_op_read_reply.daTa(&some_int8_t);
}


void on_RAW_PRESSURE(const RAW_PRESSURE &praw_pressure) {
	auto some_time_usec   = praw_pressure.time_usec();
	auto some_press_abs   = praw_pressure.press_abs();
	auto some_press_diff1 = praw_pressure.press_diff1();
	auto some_press_diff2 = praw_pressure.press_diff2();
	auto some_temperature = praw_pressure.temperature();
	
}


void on_DIGICAM_CONTROL(const DIGICAM_CONTROL &pdigicam_control) {
	auto some_target_system    = pdigicam_control.target_system();
	auto some_target_component = pdigicam_control.target_component();
	auto some_session          = pdigicam_control.session();
	auto some_zoom_pos         = pdigicam_control.zoom_pos();
	auto some_zoom_step        = pdigicam_control.zoom_step();
	auto some_focus_lock       = pdigicam_control.focus_lock();
	auto some_shot             = pdigicam_control.shot();
	auto some_command_id       = pdigicam_control.command_id();
	auto some_extra_param      = pdigicam_control.extra_param();
	auto some_extra_value      = pdigicam_control.extra_value();
	
}


void fill(const DIGICAM_CONTROL &pdigicam_control) {
	pdigicam_control.target_system(some_int8_t);
	pdigicam_control.target_component(some_int8_t);
	pdigicam_control.session(some_int8_t);
	pdigicam_control.zoom_pos(some_int8_t);
	pdigicam_control.zoom_step(some_int8_t);
	pdigicam_control.focus_lock(some_int8_t);
	pdigicam_control.shot(some_int8_t);
	pdigicam_control.command_id(some_int8_t);
	pdigicam_control.extra_param(some_int8_t);
	pdigicam_control.extra_value(some_float);
	
}


void on_NAMED_VALUE_FLOAT(const NAMED_VALUE_FLOAT &pnamed_value_float) {
	auto some_time_boot_ms = pnamed_value_float.time_boot_ms();
	
	const auto src_name   = pnamed_value_float.name();
	if (src_name.IS_EXISTS) {
		auto        array = src_name.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_value = pnamed_value_float.value();
	
}


void fill(const NAMED_VALUE_FLOAT &pnamed_value_float) {
	pnamed_value_float.time_boot_ms(some_int32_t);
	
	const auto src_name = pnamed_value_float.name();
	
	if (!src_name.IS_EXISTS)
		pnamed_value_float.name(some_string);
	
	pnamed_value_float.value(some_float);
	
}


void on_GOPRO_HEARTBEAT(const GOPRO_HEARTBEAT &pgopro_heartbeat) {
	
	const auto src_status = pgopro_heartbeat.status();
	
	if (src_status.IS_EXISTS)
		auto some_status = src_status.CASE.EXISTS.value;
	
	const auto src_capture_mode = pgopro_heartbeat.capture_mode();
	
	if (src_capture_mode.IS_EXISTS)
		auto some_capture_mode = src_capture_mode.CASE.EXISTS.value;
	
	const auto src_flags = pgopro_heartbeat.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags = src_flags.CASE.EXISTS.value;
	
}


void fill(const GOPRO_HEARTBEAT &pgopro_heartbeat) {
	
	const auto src_status = pgopro_heartbeat.status();
	
	if (!src_status.IS_EXISTS)
		pgopro_heartbeat.status(com::company::demo::GOPRO_HEARTBEAT_STATUS::GOPRO_HEARTBEAT_STATUS_DISCONNECTED);
	
	
	const auto src_capture_mode = pgopro_heartbeat.capture_mode();
	
	if (!src_capture_mode.IS_EXISTS)
		pgopro_heartbeat.capture_mode(com::company::demo::GOPRO_CAPTURE_MODE::GOPRO_CAPTURE_MODE_VIDEO);
	
	
	const auto src_flags = pgopro_heartbeat.flags();
	
	if (!src_flags.IS_EXISTS)
		pgopro_heartbeat.flags(com::company::demo::GOPRO_HEARTBEAT_FLAGS::GOPRO_FLAG_RECORDING);
	
	
}


void on_ATTITUDE(const ATTITUDE &pattitude) {
	auto some_time_boot_ms = pattitude.time_boot_ms();
	auto some_roll         = pattitude.roll();
	auto some_pitch        = pattitude.pitch();
	auto some_yaw          = pattitude.yaw();
	auto some_rollspeed    = pattitude.rollspeed();
	auto some_pitchspeed   = pattitude.pitchspeed();
	auto some_yawspeed     = pattitude.yawspeed();
	
}


void on_MISSION_WRITE_PARTIAL_LIST(const MISSION_WRITE_PARTIAL_LIST &pmission_write_partial_list) {
	auto some_target_system    = pmission_write_partial_list.target_system();
	auto some_target_component = pmission_write_partial_list.target_component();
	auto some_start_index      = pmission_write_partial_list.start_index();
	auto some_end_index        = pmission_write_partial_list.end_index();
	
	const auto src_mission_type = pmission_write_partial_list.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_AHRS2(const AHRS2 &pahrs2) {
	auto some_roll     = pahrs2.roll();
	auto some_pitch    = pahrs2.pitch();
	auto some_yaw      = pahrs2.yaw();
	auto some_altitude = pahrs2.altitude();
	auto some_lat      = pahrs2.lat();
	auto some_lng      = pahrs2.lng();
	
}


void fill(const AHRS2 &pahrs2) {
	pahrs2.roll(some_float);
	pahrs2.pitch(some_float);
	pahrs2.yaw(some_float);
	pahrs2.altitude(some_float);
	pahrs2.lat(some_int32_t);
	pahrs2.lng(some_int32_t);
	
}


void on_LOG_ERASE(const LOG_ERASE &plog_erase) {
	auto some_target_system    = plog_erase.target_system();
	auto some_target_component = plog_erase.target_component();
	
}


void fill(const LOG_ERASE &plog_erase) {
	plog_erase.target_system(some_int8_t);
	plog_erase.target_component(some_int8_t);
	
}


void on_TERRAIN_REQUEST(const TERRAIN_REQUEST &pterrain_request) {
	auto some_lat          = pterrain_request.lat();
	auto some_lon          = pterrain_request.lon();
	auto some_grid_spacing = pterrain_request.grid_spacing();
	auto some_mask         = pterrain_request.mask();
	
}


void fill(const TERRAIN_REQUEST &pterrain_request) {
	pterrain_request.lat(some_int32_t);
	pterrain_request.lon(some_int32_t);
	pterrain_request.grid_spacing(some_int16_t);
	pterrain_request.mask(some_int64_t);
	
}


void on_MOUNT_STATUS(const MOUNT_STATUS &pmount_status) {
	auto some_target_system    = pmount_status.target_system();
	auto some_target_component = pmount_status.target_component();
	auto some_pointing_a       = pmount_status.pointing_a();
	auto some_pointing_b       = pmount_status.pointing_b();
	auto some_pointing_c       = pmount_status.pointing_c();
	
}


void fill(const MOUNT_STATUS &pmount_status) {
	pmount_status.target_system(some_int8_t);
	pmount_status.target_component(some_int8_t);
	pmount_status.pointing_a(some_int32_t);
	pmount_status.pointing_b(some_int32_t);
	pmount_status.pointing_c(some_int32_t);
	
}


void on_MANUAL_SETPOINT(const MANUAL_SETPOINT &pmanual_setpoint) {
	auto some_time_boot_ms           = pmanual_setpoint.time_boot_ms();
	auto some_roll                   = pmanual_setpoint.roll();
	auto some_pitch                  = pmanual_setpoint.pitch();
	auto some_yaw                    = pmanual_setpoint.yaw();
	auto some_thrust                 = pmanual_setpoint.thrust();
	auto some_mode_switch            = pmanual_setpoint.mode_switch();
	auto some_manual_override_switch = pmanual_setpoint.manual_override_switch();
	
}


void on_PID_TUNING(const PID_TUNING &ppid_tuning) {
	
	const auto src_axis = ppid_tuning.axis();
	
	if (src_axis.IS_EXISTS)
		auto some_axis     = src_axis.CASE.EXISTS.value;
	auto     some_desired  = ppid_tuning.desired();
	auto     some_achieved = ppid_tuning.achieved();
	auto     some_FF       = ppid_tuning.FF();
	auto     some_P        = ppid_tuning.P();
	auto     some_I        = ppid_tuning.I();
	auto     some_D        = ppid_tuning.D();
	
}


void fill(const PID_TUNING &ppid_tuning) {
	
	const auto src_axis = ppid_tuning.axis();
	
	if (!src_axis.IS_EXISTS)
		ppid_tuning.axis(com::company::demo::PID_TUNING_AXIS::PID_TUNING_ROLL);
	
	ppid_tuning.desired(some_float);
	ppid_tuning.achieved(some_float);
	ppid_tuning.FF(some_float);
	ppid_tuning.P(some_float);
	ppid_tuning.I(some_float);
	ppid_tuning.D(some_float);
	
}


void on_SAFETY_ALLOWED_AREA(const SAFETY_ALLOWED_AREA &psafety_allowed_area) {
	
	const auto src_frame = psafety_allowed_area.frame();
	
	if (src_frame.IS_EXISTS)
		auto some_frame = src_frame.CASE.EXISTS.value;
	auto     some_p1x   = psafety_allowed_area.p1x();
	auto     some_p1y   = psafety_allowed_area.p1y();
	auto     some_p1z   = psafety_allowed_area.p1z();
	auto     some_p2x   = psafety_allowed_area.p2x();
	auto     some_p2y   = psafety_allowed_area.p2y();
	auto     some_p2z   = psafety_allowed_area.p2z();
	
}


void on_OPTICAL_FLOW_RAD(const OPTICAL_FLOW_RAD &poptical_flow_rad) {
	auto some_time_usec              = poptical_flow_rad.time_usec();
	auto some_sensor_id              = poptical_flow_rad.sensor_id();
	auto some_integration_time_us    = poptical_flow_rad.integration_time_us();
	auto some_integrated_x           = poptical_flow_rad.integrated_x();
	auto some_integrated_y           = poptical_flow_rad.integrated_y();
	auto some_integrated_xgyro       = poptical_flow_rad.integrated_xgyro();
	auto some_integrated_ygyro       = poptical_flow_rad.integrated_ygyro();
	auto some_integrated_zgyro       = poptical_flow_rad.integrated_zgyro();
	auto some_temperature            = poptical_flow_rad.temperature();
	auto some_quality                = poptical_flow_rad.quality();
	auto some_time_delta_distance_us = poptical_flow_rad.time_delta_distance_us();
	auto some_distance               = poptical_flow_rad.distance();
	
}


void fill(const OPTICAL_FLOW_RAD &poptical_flow_rad) {
	poptical_flow_rad.time_usec(some_int64_t);
	poptical_flow_rad.sensor_id(some_int8_t);
	poptical_flow_rad.integration_time_us(some_int32_t);
	poptical_flow_rad.integrated_x(some_float);
	poptical_flow_rad.integrated_y(some_float);
	poptical_flow_rad.integrated_xgyro(some_float);
	poptical_flow_rad.integrated_ygyro(some_float);
	poptical_flow_rad.integrated_zgyro(some_float);
	poptical_flow_rad.temperature(some_int16_t);
	poptical_flow_rad.quality(some_int8_t);
	poptical_flow_rad.time_delta_distance_us(some_int32_t);
	poptical_flow_rad.distance(some_float);
	
}


void on_LOG_DATA(const LOG_DATA &plog_data) {
	auto some_id    = plog_data.id();
	auto some_ofs   = plog_data.ofs();
	auto some_count = plog_data.count();
	
	const auto  src_daTa = plog_data.daTa();
	for (size_t index    = 0; index < LOG_DATA::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const LOG_DATA &plog_data) {
	plog_data.id(some_int16_t);
	plog_data.ofs(some_int32_t);
	plog_data.count(some_int8_t);
	plog_data.daTa(&some_int8_t);
}


void on_MISSION_CLEAR_ALL(const MISSION_CLEAR_ALL &pmission_clear_all) {
	auto some_target_system    = pmission_clear_all.target_system();
	auto some_target_component = pmission_clear_all.target_component();
	
	const auto src_mission_type = pmission_clear_all.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_AHRS3(const AHRS3 &pahrs3) {
	auto some_roll     = pahrs3.roll();
	auto some_pitch    = pahrs3.pitch();
	auto some_yaw      = pahrs3.yaw();
	auto some_altitude = pahrs3.altitude();
	auto some_lat      = pahrs3.lat();
	auto some_lng      = pahrs3.lng();
	auto some_v1       = pahrs3.v1();
	auto some_v2       = pahrs3.v2();
	auto some_v3       = pahrs3.v3();
	auto some_v4       = pahrs3.v4();
	
}


void fill(const AHRS3 &pahrs3) {
	pahrs3.roll(some_float);
	pahrs3.pitch(some_float);
	pahrs3.yaw(some_float);
	pahrs3.altitude(some_float);
	pahrs3.lat(some_int32_t);
	pahrs3.lng(some_int32_t);
	pahrs3.v1(some_float);
	pahrs3.v2(some_float);
	pahrs3.v3(some_float);
	pahrs3.v4(some_float);
	
}


void on_VICON_POSITION_ESTIMATE(const VICON_POSITION_ESTIMATE &pvicon_position_estimate) {
	auto some_usec  = pvicon_position_estimate.usec();
	auto some_x     = pvicon_position_estimate.x();
	auto some_y     = pvicon_position_estimate.y();
	auto some_z     = pvicon_position_estimate.z();
	auto some_roll  = pvicon_position_estimate.roll();
	auto some_pitch = pvicon_position_estimate.pitch();
	auto some_yaw   = pvicon_position_estimate.yaw();
	
}


void fill(const VICON_POSITION_ESTIMATE &pvicon_position_estimate) {
	pvicon_position_estimate.usec(some_int64_t);
	pvicon_position_estimate.x(some_float);
	pvicon_position_estimate.y(some_float);
	pvicon_position_estimate.z(some_float);
	pvicon_position_estimate.roll(some_float);
	pvicon_position_estimate.pitch(some_float);
	pvicon_position_estimate.yaw(some_float);
	
}


void on_GPS2_RTK(const GPS2_RTK &pgps2_rtk) {
	auto some_time_last_baseline_ms = pgps2_rtk.time_last_baseline_ms();
	auto some_rtk_receiver_id       = pgps2_rtk.rtk_receiver_id();
	auto some_wn                    = pgps2_rtk.wn();
	auto some_tow                   = pgps2_rtk.tow();
	auto some_rtk_health            = pgps2_rtk.rtk_health();
	auto some_rtk_rate              = pgps2_rtk.rtk_rate();
	auto some_nsats                 = pgps2_rtk.nsats();
	auto some_baseline_coords_type  = pgps2_rtk.baseline_coords_type();
	auto some_baseline_a_mm         = pgps2_rtk.baseline_a_mm();
	auto some_baseline_b_mm         = pgps2_rtk.baseline_b_mm();
	auto some_baseline_c_mm         = pgps2_rtk.baseline_c_mm();
	auto some_accuracy              = pgps2_rtk.accuracy();
	auto some_iar_num_hypotheses    = pgps2_rtk.iar_num_hypotheses();
	
}


void fill(const GPS2_RTK &pgps2_rtk) {
	pgps2_rtk.time_last_baseline_ms(some_int32_t);
	pgps2_rtk.rtk_receiver_id(some_int8_t);
	pgps2_rtk.wn(some_int16_t);
	pgps2_rtk.tow(some_int32_t);
	pgps2_rtk.rtk_health(some_int8_t);
	pgps2_rtk.rtk_rate(some_int8_t);
	pgps2_rtk.nsats(some_int8_t);
	pgps2_rtk.baseline_coords_type(some_int8_t);
	pgps2_rtk.baseline_a_mm(some_int32_t);
	pgps2_rtk.baseline_b_mm(some_int32_t);
	pgps2_rtk.baseline_c_mm(some_int32_t);
	pgps2_rtk.accuracy(some_int32_t);
	pgps2_rtk.iar_num_hypotheses(some_int32_t);
	
}


void on_MAG_CAL_REPORT(const MAG_CAL_REPORT &pmag_cal_report) {
	auto some_compass_id = pmag_cal_report.compass_id();
	auto some_cal_mask   = pmag_cal_report.cal_mask();
	
	const auto src_cal_status = pmag_cal_report.cal_status();
	
	if (src_cal_status.IS_EXISTS)
		auto some_cal_status = src_cal_status.CASE.EXISTS.value;
	auto     some_autosaved  = pmag_cal_report.autosaved();
	auto     some_fitness    = pmag_cal_report.fitness();
	auto     some_ofs_x      = pmag_cal_report.ofs_x();
	auto     some_ofs_y      = pmag_cal_report.ofs_y();
	auto     some_ofs_z      = pmag_cal_report.ofs_z();
	auto     some_diag_x     = pmag_cal_report.diag_x();
	auto     some_diag_y     = pmag_cal_report.diag_y();
	auto     some_diag_z     = pmag_cal_report.diag_z();
	auto     some_offdiag_x  = pmag_cal_report.offdiag_x();
	auto     some_offdiag_y  = pmag_cal_report.offdiag_y();
	auto     some_offdiag_z  = pmag_cal_report.offdiag_z();
	
}


void fill(const MAG_CAL_REPORT &pmag_cal_report) {
	pmag_cal_report.compass_id(some_int8_t);
	pmag_cal_report.cal_mask(some_int8_t);
	
	const auto src_cal_status = pmag_cal_report.cal_status();
	
	if (!src_cal_status.IS_EXISTS)
		pmag_cal_report.cal_status(com::company::demo::MAG_CAL_STATUS::MAG_CAL_NOT_STARTED);
	
	pmag_cal_report.autosaved(some_int8_t);
	pmag_cal_report.fitness(some_float);
	pmag_cal_report.ofs_x(some_float);
	pmag_cal_report.ofs_y(some_float);
	pmag_cal_report.ofs_z(some_float);
	pmag_cal_report.diag_x(some_float);
	pmag_cal_report.diag_y(some_float);
	pmag_cal_report.diag_z(some_float);
	pmag_cal_report.offdiag_x(some_float);
	pmag_cal_report.offdiag_y(some_float);
	pmag_cal_report.offdiag_z(some_float);
	
}


void on_LOG_REQUEST_LIST(const LOG_REQUEST_LIST &plog_request_list) {
	auto some_target_system    = plog_request_list.target_system();
	auto some_target_component = plog_request_list.target_component();
	auto some_start            = plog_request_list.start();
	auto some_end              = plog_request_list.end();
	
}


void fill(const LOG_REQUEST_LIST &plog_request_list) {
	plog_request_list.target_system(some_int8_t);
	plog_request_list.target_component(some_int8_t);
	plog_request_list.start(some_int16_t);
	plog_request_list.end(some_int16_t);
	
}


void on_SCALED_PRESSURE(const SCALED_PRESSURE &pscaled_pressure) {
	auto some_time_boot_ms = pscaled_pressure.time_boot_ms();
	auto some_press_abs    = pscaled_pressure.press_abs();
	auto some_press_diff   = pscaled_pressure.press_diff();
	auto some_temperature  = pscaled_pressure.temperature();
	
}


void on_V2_EXTENSION(const V2_EXTENSION &pv2_extension) {
	auto some_target_network   = pv2_extension.target_network();
	auto some_target_system    = pv2_extension.target_system();
	auto some_target_component = pv2_extension.target_component();
	auto some_message_type     = pv2_extension.message_type();
	
	const auto  src_payload = pv2_extension.payload();
	for (size_t index       = 0; index < V2_EXTENSION::payload_::len; index++)
		some_int8_t = src_payload.get(index);
	
}


void fill(const V2_EXTENSION &pv2_extension) {
	pv2_extension.target_network(some_int8_t);
	pv2_extension.target_system(some_int8_t);
	pv2_extension.target_component(some_int8_t);
	pv2_extension.message_type(some_int16_t);
	pv2_extension.payload(&some_int8_t);
}


void on_HEARTBEAT(const HEARTBEAT &pheartbeat) {
	
	const auto src_typE = pheartbeat.typE();
	
	if (src_typE.IS_EXISTS)
		auto some_typE = src_typE.CASE.EXISTS.value;
	
	const auto src_autopilot = pheartbeat.autopilot();
	
	if (src_autopilot.IS_EXISTS)
		auto some_autopilot = src_autopilot.CASE.EXISTS.value;
	
	const auto src_base_mode = pheartbeat.base_mode();
	
	if (src_base_mode.IS_EXISTS)
		auto some_base_mode   = src_base_mode.CASE.EXISTS.value;
	auto     some_custom_mode = pheartbeat.custom_mode();
	
	const auto src_system_status = pheartbeat.system_status();
	
	if (src_system_status.IS_EXISTS)
		auto some_system_status   = src_system_status.CASE.EXISTS.value;
	auto     some_mavlink_version = pheartbeat.mavlink_version();
	
}


void on_PARAM_MAP_RC(const PARAM_MAP_RC &pparam_map_rc) {
	auto some_target_system    = pparam_map_rc.target_system();
	auto some_target_component = pparam_map_rc.target_component();
	
	const auto src_param_id                    = pparam_map_rc.param_id();
	if (src_param_id.IS_EXISTS) {
		auto        array = src_param_id.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_param_index                = pparam_map_rc.param_index();
	auto       some_parameter_rc_channel_index = pparam_map_rc.parameter_rc_channel_index();
	auto       some_param_value0               = pparam_map_rc.param_value0();
	auto       some_scale                      = pparam_map_rc.scale();
	auto       some_param_value_min            = pparam_map_rc.param_value_min();
	auto       some_param_value_max            = pparam_map_rc.param_value_max();
	
}


void on_POWER_STATUS(const POWER_STATUS &ppower_status) {
	auto some_Vcc    = ppower_status.Vcc();
	auto some_Vservo = ppower_status.Vservo();
	
	const auto src_flags = ppower_status.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags = src_flags.CASE.EXISTS.value;
	
}


void fill(const POWER_STATUS &ppower_status) {
	ppower_status.Vcc(some_int16_t);
	ppower_status.Vservo(some_int16_t);
	
	const auto src_flags = ppower_status.flags();
	
	if (!src_flags.IS_EXISTS)
		ppower_status.flags(com::company::demo::MAV_POWER_STATUS::MAV_POWER_STATUS_BRICK_VALID);
	
	
}


void on_REMOTE_LOG_DATA_BLOCK(const REMOTE_LOG_DATA_BLOCK &premote_log_data_block) {
	auto some_target_system    = premote_log_data_block.target_system();
	auto some_target_component = premote_log_data_block.target_component();
	
	const auto src_seqno = premote_log_data_block.seqno();
	
	if (src_seqno.IS_EXISTS)
		auto some_seqno = src_seqno.CASE.EXISTS.value;
	
	const auto  src_daTa = premote_log_data_block.daTa();
	for (size_t index    = 0; index < REMOTE_LOG_DATA_BLOCK::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const REMOTE_LOG_DATA_BLOCK &premote_log_data_block) {
	premote_log_data_block.target_system(some_int8_t);
	premote_log_data_block.target_component(some_int8_t);
	
	const auto src_seqno = premote_log_data_block.seqno();
	
	if (!src_seqno.IS_EXISTS)
		premote_log_data_block.seqno(com::company::demo::MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS::MAV_REMOTE_LOG_DATA_BLOCK_STOP);
	
	premote_log_data_block.daTa(&some_int8_t);
}


void on_LOGGING_DATA_ACKED(const LOGGING_DATA_ACKED &plogging_data_acked) {
	auto some_target_system        = plogging_data_acked.target_system();
	auto some_target_component     = plogging_data_acked.target_component();
	auto some_sequence             = plogging_data_acked.sequence();
	auto some_length               = plogging_data_acked.length();
	auto some_first_message_offset = plogging_data_acked.first_message_offset();
	
	const auto  src_daTa = plogging_data_acked.daTa();
	for (size_t index    = 0; index < LOGGING_DATA_ACKED::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const LOGGING_DATA_ACKED &plogging_data_acked) {
	plogging_data_acked.target_system(some_int8_t);
	plogging_data_acked.target_component(some_int8_t);
	plogging_data_acked.sequence(some_int16_t);
	plogging_data_acked.length(some_int8_t);
	plogging_data_acked.first_message_offset(some_int8_t);
	plogging_data_acked.daTa(&some_int8_t);
}


void on_TERRAIN_CHECK(const TERRAIN_CHECK &pterrain_check) {
	auto some_lat = pterrain_check.lat();
	auto some_lon = pterrain_check.lon();
	
}


void fill(const TERRAIN_CHECK &pterrain_check) {
	pterrain_check.lat(some_int32_t);
	pterrain_check.lon(some_int32_t);
	
}


void on_MOUNT_CONFIGURE(const MOUNT_CONFIGURE &pmount_configure) {
	auto some_target_system    = pmount_configure.target_system();
	auto some_target_component = pmount_configure.target_component();
	
	const auto src_mount_mode = pmount_configure.mount_mode();
	
	if (src_mount_mode.IS_EXISTS)
		auto some_mount_mode = src_mount_mode.CASE.EXISTS.value;
	auto     some_stab_roll  = pmount_configure.stab_roll();
	auto     some_stab_pitch = pmount_configure.stab_pitch();
	auto     some_stab_yaw   = pmount_configure.stab_yaw();
	
}


void fill(const MOUNT_CONFIGURE &pmount_configure) {
	pmount_configure.target_system(some_int8_t);
	pmount_configure.target_component(some_int8_t);
	
	const auto src_mount_mode = pmount_configure.mount_mode();
	
	if (!src_mount_mode.IS_EXISTS)
		pmount_configure.mount_mode(com::company::demo::MAV_MOUNT_MODE::MAV_MOUNT_MODE_RETRACT);
	
	pmount_configure.stab_roll(some_int8_t);
	pmount_configure.stab_pitch(some_int8_t);
	pmount_configure.stab_yaw(some_int8_t);
	
}


void on_MISSION_REQUEST_INT(const MISSION_REQUEST_INT &pmission_request_int) {
	auto some_target_system    = pmission_request_int.target_system();
	auto some_target_component = pmission_request_int.target_component();
	auto some_seq              = pmission_request_int.seq();
	
	const auto src_mission_type = pmission_request_int.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(const LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET &plocal_position_ned_system_global_offset) {
	auto some_time_boot_ms = plocal_position_ned_system_global_offset.time_boot_ms();
	auto some_x            = plocal_position_ned_system_global_offset.x();
	auto some_y            = plocal_position_ned_system_global_offset.y();
	auto some_z            = plocal_position_ned_system_global_offset.z();
	auto some_roll         = plocal_position_ned_system_global_offset.roll();
	auto some_pitch        = plocal_position_ned_system_global_offset.pitch();
	auto some_yaw          = plocal_position_ned_system_global_offset.yaw();
	
}


void on_COMMAND_ACK(const COMMAND_ACK &pcommand_ack) {
	
	const auto src_command = pcommand_ack.command();
	
	if (src_command.IS_EXISTS)
		auto some_command = src_command.CASE.EXISTS.value;
	
	const auto src_result = pcommand_ack.result();
	
	if (src_result.IS_EXISTS)
		auto some_result = src_result.CASE.EXISTS.value;
	
	const auto src_progress = pcommand_ack.progress();
	
	if (src_progress.IS_EXISTS)
		auto some_progress = src_progress.CASE.EXISTS.value;
	
	const auto src_result_param2 = pcommand_ack.result_param2();
	
	if (src_result_param2.IS_EXISTS)
		auto some_result_param2 = src_result_param2.CASE.EXISTS.value;
	
	const auto src_target_system = pcommand_ack.target_system();
	
	if (src_target_system.IS_EXISTS)
		auto some_target_system = src_target_system.CASE.EXISTS.value;
	
	const auto src_target_component = pcommand_ack.target_component();
	
	if (src_target_component.IS_EXISTS)
		auto some_target_component = src_target_component.CASE.EXISTS.value;
	
}


void on_DATA_STREAM(const DATA_STREAM &pdata_stream) {
	auto some_stream_id    = pdata_stream.stream_id();
	auto some_message_rate = pdata_stream.message_rate();
	auto some_on_off       = pdata_stream.on_off();
	
}


void on_MISSION_REQUEST(const MISSION_REQUEST &pmission_request) {
	auto some_target_system    = pmission_request.target_system();
	auto some_target_component = pmission_request.target_component();
	auto some_seq              = pmission_request.seq();
	
	const auto src_mission_type = pmission_request.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_TERRAIN_REPORT(const TERRAIN_REPORT &pterrain_report) {
	auto some_lat            = pterrain_report.lat();
	auto some_lon            = pterrain_report.lon();
	auto some_spacing        = pterrain_report.spacing();
	auto some_terrain_height = pterrain_report.terrain_height();
	auto some_current_height = pterrain_report.current_height();
	auto some_pending        = pterrain_report.pending();
	auto some_loaded         = pterrain_report.loaded();
	
}


void fill(const TERRAIN_REPORT &pterrain_report) {
	pterrain_report.lat(some_int32_t);
	pterrain_report.lon(some_int32_t);
	pterrain_report.spacing(some_int16_t);
	pterrain_report.terrain_height(some_float);
	pterrain_report.current_height(some_float);
	pterrain_report.pending(some_int16_t);
	pterrain_report.loaded(some_int16_t);
	
}


void on_SET_HOME_POSITION(const SET_HOME_POSITION &pset_home_position) {
	auto some_target_system = pset_home_position.target_system();
	auto some_latitude      = pset_home_position.latitude();
	auto some_longitude     = pset_home_position.longitude();
	auto some_altitude      = pset_home_position.altitude();
	auto some_x             = pset_home_position.x();
	auto some_y             = pset_home_position.y();
	auto some_z             = pset_home_position.z();
	
	const auto  src_q = pset_home_position.q();
	for (size_t index = 0; index < SET_HOME_POSITION::q_::len; index++)
		some_float              = src_q.get(index);
	auto        some_approach_x = pset_home_position.approach_x();
	auto        some_approach_y = pset_home_position.approach_y();
	auto        some_approach_z = pset_home_position.approach_z();
	
	const auto src_time_usec = pset_home_position.time_usec();
	
	if (src_time_usec.IS_EXISTS)
		auto some_time_usec = src_time_usec.CASE.EXISTS.value;
	
}


void fill(const SET_HOME_POSITION &pset_home_position) {
	pset_home_position.target_system(some_int8_t);
	pset_home_position.latitude(some_int32_t);
	pset_home_position.longitude(some_int32_t);
	pset_home_position.altitude(some_int32_t);
	pset_home_position.x(some_float);
	pset_home_position.y(some_float);
	pset_home_position.z(some_float);
	pset_home_position.q(&some_float);
	pset_home_position.approach_x(some_float);
	pset_home_position.approach_y(some_float);
	pset_home_position.approach_z(some_float);
	
	const auto src_time_usec = pset_home_position.time_usec();
	
	if (!src_time_usec.IS_EXISTS)
		pset_home_position.time_usec(some_int64_t);
	
	
}

void on_SwitchModeCommand() {}


void on_HIL_RC_INPUTS_RAW(const HIL_RC_INPUTS_RAW &phil_rc_inputs_raw) {
	auto some_time_usec  = phil_rc_inputs_raw.time_usec();
	auto some_chan1_raw  = phil_rc_inputs_raw.chan1_raw();
	auto some_chan2_raw  = phil_rc_inputs_raw.chan2_raw();
	auto some_chan3_raw  = phil_rc_inputs_raw.chan3_raw();
	auto some_chan4_raw  = phil_rc_inputs_raw.chan4_raw();
	auto some_chan5_raw  = phil_rc_inputs_raw.chan5_raw();
	auto some_chan6_raw  = phil_rc_inputs_raw.chan6_raw();
	auto some_chan7_raw  = phil_rc_inputs_raw.chan7_raw();
	auto some_chan8_raw  = phil_rc_inputs_raw.chan8_raw();
	auto some_chan9_raw  = phil_rc_inputs_raw.chan9_raw();
	auto some_chan10_raw = phil_rc_inputs_raw.chan10_raw();
	auto some_chan11_raw = phil_rc_inputs_raw.chan11_raw();
	auto some_chan12_raw = phil_rc_inputs_raw.chan12_raw();
	auto some_rssi       = phil_rc_inputs_raw.rssi();
	
}


void on_SCALED_IMU3(const SCALED_IMU3 &pscaled_imu3) {
	auto some_time_boot_ms = pscaled_imu3.time_boot_ms();
	auto some_xacc         = pscaled_imu3.xacc();
	auto some_yacc         = pscaled_imu3.yacc();
	auto some_zacc         = pscaled_imu3.zacc();
	auto some_xgyro        = pscaled_imu3.xgyro();
	auto some_ygyro        = pscaled_imu3.ygyro();
	auto some_zgyro        = pscaled_imu3.zgyro();
	auto some_xmag         = pscaled_imu3.xmag();
	auto some_ymag         = pscaled_imu3.ymag();
	auto some_zmag         = pscaled_imu3.zmag();
	
}


void fill(const SCALED_IMU3 &pscaled_imu3) {
	pscaled_imu3.time_boot_ms(some_int32_t);
	pscaled_imu3.xacc(some_int16_t);
	pscaled_imu3.yacc(some_int16_t);
	pscaled_imu3.zacc(some_int16_t);
	pscaled_imu3.xgyro(some_int16_t);
	pscaled_imu3.ygyro(some_int16_t);
	pscaled_imu3.zgyro(some_int16_t);
	pscaled_imu3.xmag(some_int16_t);
	pscaled_imu3.ymag(some_int16_t);
	pscaled_imu3.zmag(some_int16_t);
	
}


void on_SET_MODE(const SET_MODE &pset_mode) {
	auto some_target_system = pset_mode.target_system();
	
	const auto src_base_mode = pset_mode.base_mode();
	
	if (src_base_mode.IS_EXISTS)
		auto some_base_mode   = src_base_mode.CASE.EXISTS.value;
	auto     some_custom_mode = pset_mode.custom_mode();
	
}


void on_MOUNT_CONTROL(const MOUNT_CONTROL &pmount_control) {
	auto some_target_system    = pmount_control.target_system();
	auto some_target_component = pmount_control.target_component();
	auto some_input_a          = pmount_control.input_a();
	auto some_input_b          = pmount_control.input_b();
	auto some_input_c          = pmount_control.input_c();
	auto some_save_position    = pmount_control.save_position();
	
}


void fill(const MOUNT_CONTROL &pmount_control) {
	pmount_control.target_system(some_int8_t);
	pmount_control.target_component(some_int8_t);
	pmount_control.input_a(some_int32_t);
	pmount_control.input_b(some_int32_t);
	pmount_control.input_c(some_int32_t);
	pmount_control.save_position(some_int8_t);
	
}


void on_POSITION_TARGET_GLOBAL_INT(const POSITION_TARGET_GLOBAL_INT &pposition_target_global_int) {
	auto some_time_boot_ms = pposition_target_global_int.time_boot_ms();
	
	const auto src_coordinate_frame = pposition_target_global_int.coordinate_frame();
	
	if (src_coordinate_frame.IS_EXISTS)
		auto some_coordinate_frame = src_coordinate_frame.CASE.EXISTS.value;
	auto     some_type_mask        = pposition_target_global_int.type_mask();
	auto     some_lat_int          = pposition_target_global_int.lat_int();
	auto     some_lon_int          = pposition_target_global_int.lon_int();
	auto     some_alt              = pposition_target_global_int.alt();
	auto     some_vx               = pposition_target_global_int.vx();
	auto     some_vy               = pposition_target_global_int.vy();
	auto     some_vz               = pposition_target_global_int.vz();
	auto     some_afx              = pposition_target_global_int.afx();
	auto     some_afy              = pposition_target_global_int.afy();
	auto     some_afz              = pposition_target_global_int.afz();
	auto     some_yaw              = pposition_target_global_int.yaw();
	auto     some_yaw_rate         = pposition_target_global_int.yaw_rate();
	
}


void on_LED_CONTROL(const LED_CONTROL &pled_control) {
	auto some_target_system    = pled_control.target_system();
	auto some_target_component = pled_control.target_component();
	auto some_instance         = pled_control.instance();
	auto some_pattern          = pled_control.pattern();
	auto some_custom_len       = pled_control.custom_len();
	
	const auto  src_custom_bytes = pled_control.custom_bytes();
	for (size_t index            = 0; index < LED_CONTROL::custom_bytes_::len; index++)
		some_int8_t = src_custom_bytes.get(index);
	
}


void fill(const LED_CONTROL &pled_control) {
	pled_control.target_system(some_int8_t);
	pled_control.target_component(some_int8_t);
	pled_control.instance(some_int8_t);
	pled_control.pattern(some_int8_t);
	pled_control.custom_len(some_int8_t);
	pled_control.custom_bytes(&some_int8_t);
}


void on_SIM_STATE(const SIM_STATE &psim_state) {
	auto some_q1           = psim_state.q1();
	auto some_q2           = psim_state.q2();
	auto some_q3           = psim_state.q3();
	auto some_q4           = psim_state.q4();
	auto some_roll         = psim_state.roll();
	auto some_pitch        = psim_state.pitch();
	auto some_yaw          = psim_state.yaw();
	auto some_xacc         = psim_state.xacc();
	auto some_yacc         = psim_state.yacc();
	auto some_zacc         = psim_state.zacc();
	auto some_xgyro        = psim_state.xgyro();
	auto some_ygyro        = psim_state.ygyro();
	auto some_zgyro        = psim_state.zgyro();
	auto some_lat          = psim_state.lat();
	auto some_lon          = psim_state.lon();
	auto some_alt          = psim_state.alt();
	auto some_std_dev_horz = psim_state.std_dev_horz();
	auto some_std_dev_vert = psim_state.std_dev_vert();
	auto some_vn           = psim_state.vn();
	auto some_ve           = psim_state.ve();
	auto some_vd           = psim_state.vd();
	
}


void fill(const SIM_STATE &psim_state) {
	psim_state.q1(some_float);
	psim_state.q2(some_float);
	psim_state.q3(some_float);
	psim_state.q4(some_float);
	psim_state.roll(some_float);
	psim_state.pitch(some_float);
	psim_state.yaw(some_float);
	psim_state.xacc(some_float);
	psim_state.yacc(some_float);
	psim_state.zacc(some_float);
	psim_state.xgyro(some_float);
	psim_state.ygyro(some_float);
	psim_state.zgyro(some_float);
	psim_state.lat(some_float);
	psim_state.lon(some_float);
	psim_state.alt(some_float);
	psim_state.std_dev_horz(some_float);
	psim_state.std_dev_vert(some_float);
	psim_state.vn(some_float);
	psim_state.ve(some_float);
	psim_state.vd(some_float);
	
}


void on_WIFI_CONFIG_AP(const WIFI_CONFIG_AP &pwifi_config_ap) {
	
	const auto src_ssid = pwifi_config_ap.ssid();
	if (src_ssid.IS_EXISTS) {
		auto        array = src_ssid.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_password = pwifi_config_ap.password();
	if (src_password.IS_EXISTS) {
		auto        array = src_password.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void fill(const WIFI_CONFIG_AP &pwifi_config_ap) {
	
	const auto src_ssid = pwifi_config_ap.ssid();
	
	if (!src_ssid.IS_EXISTS)
		pwifi_config_ap.ssid(some_string);
	
	
	const auto src_password = pwifi_config_ap.password();
	
	if (!src_password.IS_EXISTS)
		pwifi_config_ap.password(some_string);
	
	
}


void on_DATA96(const DATA96 &pdata96) {
	auto some_typE = pdata96.typE();
	auto some_len  = pdata96.len();
	
	const auto  src_daTa = pdata96.daTa();
	for (size_t index    = 0; index < DATA96::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const DATA96 &pdata96) {
	pdata96.typE(some_int8_t);
	pdata96.len(some_int8_t);
	pdata96.daTa(&some_int8_t);
}


void on_FLIGHT_INFORMATION(const FLIGHT_INFORMATION &pflight_information) {
	auto some_time_boot_ms     = pflight_information.time_boot_ms();
	auto some_arming_time_utc  = pflight_information.arming_time_utc();
	auto some_takeoff_time_utc = pflight_information.takeoff_time_utc();
	auto some_flight_uuid      = pflight_information.flight_uuid();
	
}


void fill(const FLIGHT_INFORMATION &pflight_information) {
	pflight_information.time_boot_ms(some_int32_t);
	pflight_information.arming_time_utc(some_int64_t);
	pflight_information.takeoff_time_utc(some_int64_t);
	pflight_information.flight_uuid(some_int64_t);
	
}


void on_RC_CHANNELS_RAW(const RC_CHANNELS_RAW &prc_channels_raw) {
	auto some_time_boot_ms = prc_channels_raw.time_boot_ms();
	auto some_port         = prc_channels_raw.port();
	auto some_chan1_raw    = prc_channels_raw.chan1_raw();
	auto some_chan2_raw    = prc_channels_raw.chan2_raw();
	auto some_chan3_raw    = prc_channels_raw.chan3_raw();
	auto some_chan4_raw    = prc_channels_raw.chan4_raw();
	auto some_chan5_raw    = prc_channels_raw.chan5_raw();
	auto some_chan6_raw    = prc_channels_raw.chan6_raw();
	auto some_chan7_raw    = prc_channels_raw.chan7_raw();
	auto some_chan8_raw    = prc_channels_raw.chan8_raw();
	auto some_rssi         = prc_channels_raw.rssi();
	
}


void on_SERVO_OUTPUT_RAW(const SERVO_OUTPUT_RAW &pservo_output_raw) {
	auto some_time_usec  = pservo_output_raw.time_usec();
	auto some_port       = pservo_output_raw.port();
	auto some_servo1_raw = pservo_output_raw.servo1_raw();
	auto some_servo2_raw = pservo_output_raw.servo2_raw();
	auto some_servo3_raw = pservo_output_raw.servo3_raw();
	auto some_servo4_raw = pservo_output_raw.servo4_raw();
	auto some_servo5_raw = pservo_output_raw.servo5_raw();
	auto some_servo6_raw = pservo_output_raw.servo6_raw();
	auto some_servo7_raw = pservo_output_raw.servo7_raw();
	auto some_servo8_raw = pservo_output_raw.servo8_raw();
	
	const auto src_servo9_raw = pservo_output_raw.servo9_raw();
	
	if (src_servo9_raw.IS_EXISTS)
		auto some_servo9_raw = src_servo9_raw.CASE.EXISTS.value;
	
	const auto src_servo10_raw = pservo_output_raw.servo10_raw();
	
	if (src_servo10_raw.IS_EXISTS)
		auto some_servo10_raw = src_servo10_raw.CASE.EXISTS.value;
	
	const auto src_servo11_raw = pservo_output_raw.servo11_raw();
	
	if (src_servo11_raw.IS_EXISTS)
		auto some_servo11_raw = src_servo11_raw.CASE.EXISTS.value;
	
	const auto src_servo12_raw = pservo_output_raw.servo12_raw();
	
	if (src_servo12_raw.IS_EXISTS)
		auto some_servo12_raw = src_servo12_raw.CASE.EXISTS.value;
	
	const auto src_servo13_raw = pservo_output_raw.servo13_raw();
	
	if (src_servo13_raw.IS_EXISTS)
		auto some_servo13_raw = src_servo13_raw.CASE.EXISTS.value;
	
	const auto src_servo14_raw = pservo_output_raw.servo14_raw();
	
	if (src_servo14_raw.IS_EXISTS)
		auto some_servo14_raw = src_servo14_raw.CASE.EXISTS.value;
	
	const auto src_servo15_raw = pservo_output_raw.servo15_raw();
	
	if (src_servo15_raw.IS_EXISTS)
		auto some_servo15_raw = src_servo15_raw.CASE.EXISTS.value;
	
	const auto src_servo16_raw = pservo_output_raw.servo16_raw();
	
	if (src_servo16_raw.IS_EXISTS)
		auto some_servo16_raw = src_servo16_raw.CASE.EXISTS.value;
	
}


void on_MEMINFO(const MEMINFO &pmeminfo) {
	auto some_brkval  = pmeminfo.brkval();
	auto some_freemem = pmeminfo.freemem();
	
	const auto src_freemem32 = pmeminfo.freemem32();
	
	if (src_freemem32.IS_EXISTS)
		auto some_freemem32 = src_freemem32.CASE.EXISTS.value;
	
}


void fill(const MEMINFO &pmeminfo) {
	pmeminfo.brkval(some_int16_t);
	pmeminfo.freemem(some_int16_t);
	
	const auto src_freemem32 = pmeminfo.freemem32();
	
	if (!src_freemem32.IS_EXISTS)
		pmeminfo.freemem32(some_int32_t);
	
	
}


void on_MISSION_ITEM_REACHED(const MISSION_ITEM_REACHED &pmission_item_reached) {
	auto some_seq = pmission_item_reached.seq();
	
}


void on_LOGGING_ACK(const LOGGING_ACK &plogging_ack) {
	auto some_target_system    = plogging_ack.target_system();
	auto some_target_component = plogging_ack.target_component();
	auto some_sequence         = plogging_ack.sequence();
	
}


void fill(const LOGGING_ACK &plogging_ack) {
	plogging_ack.target_system(some_int8_t);
	plogging_ack.target_component(some_int8_t);
	plogging_ack.sequence(some_int16_t);
	
}


void on_VISION_SPEED_ESTIMATE(const VISION_SPEED_ESTIMATE &pvision_speed_estimate) {
	auto some_usec = pvision_speed_estimate.usec();
	auto some_x    = pvision_speed_estimate.x();
	auto some_y    = pvision_speed_estimate.y();
	auto some_z    = pvision_speed_estimate.z();
	
}


void fill(const VISION_SPEED_ESTIMATE &pvision_speed_estimate) {
	pvision_speed_estimate.usec(some_int64_t);
	pvision_speed_estimate.x(some_float);
	pvision_speed_estimate.y(some_float);
	pvision_speed_estimate.z(some_float);
	
}


void on_DEBUG_VECT(const DEBUG_VECT &pdebug_vect) {
	
	const auto src_name       = pdebug_vect.name();
	if (src_name.IS_EXISTS) {
		auto        array = src_name.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_time_usec = pdebug_vect.time_usec();
	auto       some_x         = pdebug_vect.x();
	auto       some_y         = pdebug_vect.y();
	auto       some_z         = pdebug_vect.z();
	
}


void fill(const DEBUG_VECT &pdebug_vect) {
	
	const auto src_name = pdebug_vect.name();
	
	if (!src_name.IS_EXISTS)
		pdebug_vect.name(some_string);
	
	pdebug_vect.time_usec(some_int64_t);
	pdebug_vect.x(some_float);
	pdebug_vect.y(some_float);
	pdebug_vect.z(some_float);
	
}


void on_LOG_REQUEST_END(const LOG_REQUEST_END &plog_request_end) {
	auto some_target_system    = plog_request_end.target_system();
	auto some_target_component = plog_request_end.target_component();
	
}


void fill(const LOG_REQUEST_END &plog_request_end) {
	plog_request_end.target_system(some_int8_t);
	plog_request_end.target_component(some_int8_t);
	
}


void on_MISSION_ACK(const MISSION_ACK &pmission_ack) {
	auto some_target_system    = pmission_ack.target_system();
	auto some_target_component = pmission_ack.target_component();
	
	const auto src_typE = pmission_ack.typE();
	
	if (src_typE.IS_EXISTS)
		auto some_typE = src_typE.CASE.EXISTS.value;
	
	const auto src_mission_type = pmission_ack.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_CHANGE_OPERATOR_CONTROL_ACK(const CHANGE_OPERATOR_CONTROL_ACK &pchange_operator_control_ack) {
	auto some_gcs_system_id   = pchange_operator_control_ack.gcs_system_id();
	auto some_control_request = pchange_operator_control_ack.control_request();
	auto some_ack             = pchange_operator_control_ack.ack();
	
}


void on_MISSION_CURRENT(const MISSION_CURRENT &pmission_current) {
	auto some_seq = pmission_current.seq();
	
}


void on_SYSTEM_TIME(const SYSTEM_TIME &psystem_time) {
	auto some_time_unix_usec = psystem_time.time_unix_usec();
	auto some_time_boot_ms   = psystem_time.time_boot_ms();
	
}


void on_CAMERA_TRIGGER(const CAMERA_TRIGGER &pcamera_trigger) {
	auto some_time_usec = pcamera_trigger.time_usec();
	auto some_seq       = pcamera_trigger.seq();
	
}


void fill(const CAMERA_TRIGGER &pcamera_trigger) {
	pcamera_trigger.time_usec(some_int64_t);
	pcamera_trigger.seq(some_int32_t);
	
}


void on_GOPRO_SET_RESPONSE(const GOPRO_SET_RESPONSE &pgopro_set_response) {
	
	const auto src_cmd_id = pgopro_set_response.cmd_id();
	
	if (src_cmd_id.IS_EXISTS)
		auto some_cmd_id = src_cmd_id.CASE.EXISTS.value;
	
	const auto src_status = pgopro_set_response.status();
	
	if (src_status.IS_EXISTS)
		auto some_status = src_status.CASE.EXISTS.value;
	
}


void fill(const GOPRO_SET_RESPONSE &pgopro_set_response) {
	
	const auto src_cmd_id = pgopro_set_response.cmd_id();
	
	if (!src_cmd_id.IS_EXISTS)
		pgopro_set_response.cmd_id(com::company::demo::GOPRO_COMMAND::GOPRO_COMMAND_POWER);
	
	
	const auto src_status = pgopro_set_response.status();
	
	if (!src_status.IS_EXISTS)
		pgopro_set_response.status(com::company::demo::GOPRO_REQUEST_STATUS::GOPRO_REQUEST_SUCCESS);
	
	
}


void on_VISION_POSITION_ESTIMATE(const VISION_POSITION_ESTIMATE &pvision_position_estimate) {
	auto some_usec  = pvision_position_estimate.usec();
	auto some_x     = pvision_position_estimate.x();
	auto some_y     = pvision_position_estimate.y();
	auto some_z     = pvision_position_estimate.z();
	auto some_roll  = pvision_position_estimate.roll();
	auto some_pitch = pvision_position_estimate.pitch();
	auto some_yaw   = pvision_position_estimate.yaw();
	
}


void on_MANUAL_CONTROL(const MANUAL_CONTROL &pmanual_control) {
	auto some_target  = pmanual_control.target();
	auto some_x       = pmanual_control.x();
	auto some_y       = pmanual_control.y();
	auto some_z       = pmanual_control.z();
	auto some_r       = pmanual_control.r();
	auto some_buttons = pmanual_control.buttons();
	
}


void on_RC_CHANNELS(const RC_CHANNELS &prc_channels) {
	auto some_time_boot_ms = prc_channels.time_boot_ms();
	auto some_chancount    = prc_channels.chancount();
	auto some_chan1_raw    = prc_channels.chan1_raw();
	auto some_chan2_raw    = prc_channels.chan2_raw();
	auto some_chan3_raw    = prc_channels.chan3_raw();
	auto some_chan4_raw    = prc_channels.chan4_raw();
	auto some_chan5_raw    = prc_channels.chan5_raw();
	auto some_chan6_raw    = prc_channels.chan6_raw();
	auto some_chan7_raw    = prc_channels.chan7_raw();
	auto some_chan8_raw    = prc_channels.chan8_raw();
	auto some_chan9_raw    = prc_channels.chan9_raw();
	auto some_chan10_raw   = prc_channels.chan10_raw();
	auto some_chan11_raw   = prc_channels.chan11_raw();
	auto some_chan12_raw   = prc_channels.chan12_raw();
	auto some_chan13_raw   = prc_channels.chan13_raw();
	auto some_chan14_raw   = prc_channels.chan14_raw();
	auto some_chan15_raw   = prc_channels.chan15_raw();
	auto some_chan16_raw   = prc_channels.chan16_raw();
	auto some_chan17_raw   = prc_channels.chan17_raw();
	auto some_chan18_raw   = prc_channels.chan18_raw();
	auto some_rssi         = prc_channels.rssi();
	
}


void on_PROTOCOL_VERSION(const PROTOCOL_VERSION &pprotocol_version) {
	auto some_version     = pprotocol_version.version();
	auto some_min_version = pprotocol_version.min_version();
	auto some_max_version = pprotocol_version.max_version();
	
	const auto  src_spec_version_hash = pprotocol_version.spec_version_hash();
	for (size_t index                 = 0; index < PROTOCOL_VERSION::spec_version_hash_::len; index++)
		some_int8_t = src_spec_version_hash.get(index);
	
	const auto  src_library_version_hash = pprotocol_version.library_version_hash();
	for (size_t index                    = 0; index < PROTOCOL_VERSION::library_version_hash_::len; index++)
		some_int8_t = src_library_version_hash.get(index);
	
}


void fill(const PROTOCOL_VERSION &pprotocol_version) {
	pprotocol_version.version(some_int16_t);
	pprotocol_version.min_version(some_int16_t);
	pprotocol_version.max_version(some_int16_t);
	pprotocol_version.spec_version_hash(&some_int8_t);
	pprotocol_version.library_version_hash(&some_int8_t);
}


void on_RALLY_FETCH_POINT(const RALLY_FETCH_POINT &prally_fetch_point) {
	auto some_target_system    = prally_fetch_point.target_system();
	auto some_target_component = prally_fetch_point.target_component();
	auto some_idx              = prally_fetch_point.idx();
	
}


void fill(const RALLY_FETCH_POINT &prally_fetch_point) {
	prally_fetch_point.target_system(some_int8_t);
	prally_fetch_point.target_component(some_int8_t);
	prally_fetch_point.idx(some_int8_t);
	
}


void on_PARAM_VALUE(const PARAM_VALUE &pparam_value) {
	
	const auto src_param_id     = pparam_value.param_id();
	if (src_param_id.IS_EXISTS) {
		auto        array = src_param_id.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_param_value = pparam_value.param_value();
	
	const auto src_param_type = pparam_value.param_type();
	
	if (src_param_type.IS_EXISTS)
		auto some_param_type  = src_param_type.CASE.EXISTS.value;
	auto     some_param_count = pparam_value.param_count();
	auto     some_param_index = pparam_value.param_index();
	
}


void on_BATTERY_STATUS(const BATTERY_STATUS &pbattery_status) {
	auto some_id = pbattery_status.id();
	
	const auto src_battery_function = pbattery_status.battery_function();
	
	if (src_battery_function.IS_EXISTS)
		auto some_battery_function = src_battery_function.CASE.EXISTS.value;
	
	const auto src_typE = pbattery_status.typE();
	
	if (src_typE.IS_EXISTS)
		auto some_typE        = src_typE.CASE.EXISTS.value;
	auto     some_temperature = pbattery_status.temperature();
	
	const auto  src_voltages = pbattery_status.voltages();
	for (size_t index        = 0; index < BATTERY_STATUS::voltages_::len; index++)
		some_int16_t                   = src_voltages.get(index);
	auto        some_current_battery   = pbattery_status.current_battery();
	auto        some_current_consumed  = pbattery_status.current_consumed();
	auto        some_energy_consumed   = pbattery_status.energy_consumed();
	auto        some_battery_remaining = pbattery_status.battery_remaining();
	
}


void fill(const BATTERY_STATUS &pbattery_status) {
	pbattery_status.id(some_int8_t);
	
	const auto src_battery_function = pbattery_status.battery_function();
	
	if (!src_battery_function.IS_EXISTS)
		pbattery_status.battery_function(com::company::demo::MAV_BATTERY_FUNCTION::MAV_BATTERY_FUNCTION_UNKNOWN);
	
	
	const auto src_typE = pbattery_status.typE();
	
	if (!src_typE.IS_EXISTS)
		pbattery_status.typE(com::company::demo::MAV_BATTERY_TYPE::UNKNOWN);
	
	pbattery_status.temperature(some_int16_t);
	pbattery_status.voltages(&some_int16_t);
	pbattery_status.current_battery(some_int16_t);
	pbattery_status.current_consumed(some_int32_t);
	pbattery_status.energy_consumed(some_int32_t);
	pbattery_status.battery_remaining(some_int8_t);
	
}


void on_SERIAL_CONTROL(const SERIAL_CONTROL &pserial_control) {
	
	const auto src_device = pserial_control.device();
	
	if (src_device.IS_EXISTS)
		auto some_device = src_device.CASE.EXISTS.value;
	
	const auto src_flags = pserial_control.flags();
	
	if (src_flags.IS_EXISTS)
		auto some_flags    = src_flags.CASE.EXISTS.value;
	auto     some_timeout  = pserial_control.timeout();
	auto     some_baudrate = pserial_control.baudrate();
	auto     some_count    = pserial_control.count();
	
	const auto  src_daTa = pserial_control.daTa();
	for (size_t index    = 0; index < SERIAL_CONTROL::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const SERIAL_CONTROL &pserial_control) {
	
	const auto src_device = pserial_control.device();
	
	if (!src_device.IS_EXISTS)
		pserial_control.device(com::company::demo::SERIAL_CONTROL_DEV::SERIAL_CONTROL_DEV_TELEM1);
	
	
	const auto src_flags = pserial_control.flags();
	
	if (!src_flags.IS_EXISTS)
		pserial_control.flags(com::company::demo::SERIAL_CONTROL_FLAG::SERIAL_CONTROL_FLAG_REPLY);
	
	pserial_control.timeout(some_int16_t);
	pserial_control.baudrate(some_int32_t);
	pserial_control.count(some_int8_t);
	pserial_control.daTa(&some_int8_t);
}


void on_SET_POSITION_TARGET_LOCAL_NED(const SET_POSITION_TARGET_LOCAL_NED &pset_position_target_local_ned) {
	auto some_time_boot_ms     = pset_position_target_local_ned.time_boot_ms();
	auto some_target_system    = pset_position_target_local_ned.target_system();
	auto some_target_component = pset_position_target_local_ned.target_component();
	
	const auto src_coordinate_frame = pset_position_target_local_ned.coordinate_frame();
	
	if (src_coordinate_frame.IS_EXISTS)
		auto some_coordinate_frame = src_coordinate_frame.CASE.EXISTS.value;
	auto     some_type_mask        = pset_position_target_local_ned.type_mask();
	auto     some_x                = pset_position_target_local_ned.x();
	auto     some_y                = pset_position_target_local_ned.y();
	auto     some_z                = pset_position_target_local_ned.z();
	auto     some_vx               = pset_position_target_local_ned.vx();
	auto     some_vy               = pset_position_target_local_ned.vy();
	auto     some_vz               = pset_position_target_local_ned.vz();
	auto     some_afx              = pset_position_target_local_ned.afx();
	auto     some_afy              = pset_position_target_local_ned.afy();
	auto     some_afz              = pset_position_target_local_ned.afz();
	auto     some_yaw              = pset_position_target_local_ned.yaw();
	auto     some_yaw_rate         = pset_position_target_local_ned.yaw_rate();
	
}


void on_MOUNT_ORIENTATION(const MOUNT_ORIENTATION &pmount_orientation) {
	auto some_time_boot_ms = pmount_orientation.time_boot_ms();
	auto some_roll         = pmount_orientation.roll();
	auto some_pitch        = pmount_orientation.pitch();
	auto some_yaw          = pmount_orientation.yaw();
	
}


void fill(const MOUNT_ORIENTATION &pmount_orientation) {
	pmount_orientation.time_boot_ms(some_int32_t);
	pmount_orientation.roll(some_float);
	pmount_orientation.pitch(some_float);
	pmount_orientation.yaw(some_float);
	
}


void on_SET_GPS_GLOBAL_ORIGIN(const SET_GPS_GLOBAL_ORIGIN &pset_gps_global_origin) {
	auto some_target_system = pset_gps_global_origin.target_system();
	auto some_latitude      = pset_gps_global_origin.latitude();
	auto some_longitude     = pset_gps_global_origin.longitude();
	auto some_altitude      = pset_gps_global_origin.altitude();
	
	const auto src_time_usec = pset_gps_global_origin.time_usec();
	
	if (src_time_usec.IS_EXISTS)
		auto some_time_usec = src_time_usec.CASE.EXISTS.value;
	
}


void on_PARAM_EXT_SET(const PARAM_EXT_SET &pparam_ext_set) {
	auto some_target_system    = pparam_ext_set.target_system();
	auto some_target_component = pparam_ext_set.target_component();
	
	const auto src_param_id = pparam_ext_set.param_id();
	if (src_param_id.IS_EXISTS) {
		auto        array = src_param_id.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_param_value = pparam_ext_set.param_value();
	if (src_param_value.IS_EXISTS) {
		auto        array = src_param_value.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_param_type = pparam_ext_set.param_type();
	
	if (src_param_type.IS_EXISTS)
		auto some_param_type = src_param_type.CASE.EXISTS.value;
	
}


void fill(const PARAM_EXT_SET &pparam_ext_set) {
	pparam_ext_set.target_system(some_int8_t);
	pparam_ext_set.target_component(some_int8_t);
	
	const auto src_param_id = pparam_ext_set.param_id();
	
	if (!src_param_id.IS_EXISTS)
		pparam_ext_set.param_id(some_string);
	
	
	const auto src_param_value = pparam_ext_set.param_value();
	
	if (!src_param_value.IS_EXISTS)
		pparam_ext_set.param_value(some_string);
	
	
	const auto src_param_type = pparam_ext_set.param_type();
	
	if (!src_param_type.IS_EXISTS)
		pparam_ext_set.param_type(com::company::demo::MAV_PARAM_EXT_TYPE::MAV_PARAM_EXT_TYPE_UINT8);
	
	
}


void on_AUTOPILOT_VERSION(const AUTOPILOT_VERSION &pautopilot_version) {
	
	const auto src_capabilities = pautopilot_version.capabilities();
	
	if (src_capabilities.IS_EXISTS)
		auto some_capabilities          = src_capabilities.CASE.EXISTS.value;
	auto     some_flight_sw_version     = pautopilot_version.flight_sw_version();
	auto     some_middleware_sw_version = pautopilot_version.middleware_sw_version();
	auto     some_os_sw_version         = pautopilot_version.os_sw_version();
	auto     some_board_version         = pautopilot_version.board_version();
	
	const auto  src_flight_custom_version = pautopilot_version.flight_custom_version();
	for (size_t index                     = 0; index < AUTOPILOT_VERSION::flight_custom_version_::len; index++)
		some_int8_t = src_flight_custom_version.get(index);
	
	const auto  src_middleware_custom_version = pautopilot_version.middleware_custom_version();
	for (size_t index                         = 0; index < AUTOPILOT_VERSION::middleware_custom_version_::len; index++)
		some_int8_t = src_middleware_custom_version.get(index);
	
	const auto  src_os_custom_version = pautopilot_version.os_custom_version();
	for (size_t index                 = 0; index < AUTOPILOT_VERSION::os_custom_version_::len; index++)
		some_int8_t             = src_os_custom_version.get(index);
	auto        some_vendor_id  = pautopilot_version.vendor_id();
	auto        some_product_id = pautopilot_version.product_id();
	auto        some_uid        = pautopilot_version.uid();
	AUTOPILOT_VERSION_uid2_d0(pautopilot_version.) {
					
					auto some_uid2 = src_uid2.CASE.EXISTS.value;
					
				}
	
}


void fill(const AUTOPILOT_VERSION &pautopilot_version) {
	
	const auto src_capabilities = pautopilot_version.capabilities();
	
	if (!src_capabilities.IS_EXISTS)
		pautopilot_version.capabilities(com::company::demo::MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
	
	pautopilot_version.flight_sw_version(some_int32_t);
	pautopilot_version.middleware_sw_version(some_int32_t);
	pautopilot_version.os_sw_version(some_int32_t);
	pautopilot_version.board_version(some_int32_t);
	pautopilot_version.flight_custom_version(&some_int8_t);
	pautopilot_version.middleware_custom_version(&some_int8_t);
	pautopilot_version.os_custom_version(&some_int8_t);
	pautopilot_version.vendor_id(some_int16_t);
	pautopilot_version.product_id(some_int16_t);
	pautopilot_version.uid(some_int64_t);
	
	
	for (size_t d0 = 0; d0 < com::company::demo::AUTOPILOT_VERSION::uid2_::d0; d0++) {
		
		pautopilot_version.uid2(some_int8_t, d0);
		
	}
	
}


void on_MISSION_REQUEST_LIST(const MISSION_REQUEST_LIST &pmission_request_list) {
	auto some_target_system    = pmission_request_list.target_system();
	auto some_target_component = pmission_request_list.target_component();
	
	const auto src_mission_type = pmission_request_list.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_SIMSTATE(const SIMSTATE &psimstate) {
	auto some_roll  = psimstate.roll();
	auto some_pitch = psimstate.pitch();
	auto some_yaw   = psimstate.yaw();
	auto some_xacc  = psimstate.xacc();
	auto some_yacc  = psimstate.yacc();
	auto some_zacc  = psimstate.zacc();
	auto some_xgyro = psimstate.xgyro();
	auto some_ygyro = psimstate.ygyro();
	auto some_zgyro = psimstate.zgyro();
	auto some_lat   = psimstate.lat();
	auto some_lng   = psimstate.lng();
	
}


void fill(const SIMSTATE &psimstate) {
	psimstate.roll(some_float);
	psimstate.pitch(some_float);
	psimstate.yaw(some_float);
	psimstate.xacc(some_float);
	psimstate.yacc(some_float);
	psimstate.zacc(some_float);
	psimstate.xgyro(some_float);
	psimstate.ygyro(some_float);
	psimstate.zgyro(some_float);
	psimstate.lat(some_int32_t);
	psimstate.lng(some_int32_t);
	
}


void on_SET_VIDEO_STREAM_SETTINGS(const SET_VIDEO_STREAM_SETTINGS &pset_video_stream_settings) {
	auto some_target_system    = pset_video_stream_settings.target_system();
	auto some_target_component = pset_video_stream_settings.target_component();
	auto some_camera_id        = pset_video_stream_settings.camera_id();
	auto some_framerate        = pset_video_stream_settings.framerate();
	auto some_resolution_h     = pset_video_stream_settings.resolution_h();
	auto some_resolution_v     = pset_video_stream_settings.resolution_v();
	auto some_bitrate          = pset_video_stream_settings.bitrate();
	auto some_rotation         = pset_video_stream_settings.rotation();
	
	const auto src_uri = pset_video_stream_settings.uri();
	if (src_uri.IS_EXISTS) {
		auto        array = src_uri.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void fill(const SET_VIDEO_STREAM_SETTINGS &pset_video_stream_settings) {
	pset_video_stream_settings.target_system(some_int8_t);
	pset_video_stream_settings.target_component(some_int8_t);
	pset_video_stream_settings.camera_id(some_int8_t);
	pset_video_stream_settings.framerate(some_float);
	pset_video_stream_settings.resolution_h(some_int16_t);
	pset_video_stream_settings.resolution_v(some_int16_t);
	pset_video_stream_settings.bitrate(some_int32_t);
	pset_video_stream_settings.rotation(some_int16_t);
	
	const auto src_uri = pset_video_stream_settings.uri();
	
	if (!src_uri.IS_EXISTS)
		pset_video_stream_settings.uri(some_string);
	
	
}


void on_PLAY_TUNE(const PLAY_TUNE &pplay_tune) {
	auto some_target_system    = pplay_tune.target_system();
	auto some_target_component = pplay_tune.target_component();
	
	const auto src_tune = pplay_tune.tune();
	if (src_tune.IS_EXISTS) {
		auto        array = src_tune.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void fill(const PLAY_TUNE &pplay_tune) {
	pplay_tune.target_system(some_int8_t);
	pplay_tune.target_component(some_int8_t);
	
	const auto src_tune = pplay_tune.tune();
	
	if (!src_tune.IS_EXISTS)
		pplay_tune.tune(some_string);
	
	
}


void on_DIGICAM_CONFIGURE(const DIGICAM_CONFIGURE &pdigicam_configure) {
	auto some_target_system    = pdigicam_configure.target_system();
	auto some_target_component = pdigicam_configure.target_component();
	auto some_mode             = pdigicam_configure.mode();
	auto some_shutter_speed    = pdigicam_configure.shutter_speed();
	auto some_aperture         = pdigicam_configure.aperture();
	auto some_iso              = pdigicam_configure.iso();
	auto some_exposure_type    = pdigicam_configure.exposure_type();
	auto some_command_id       = pdigicam_configure.command_id();
	auto some_engine_cut_off   = pdigicam_configure.engine_cut_off();
	auto some_extra_param      = pdigicam_configure.extra_param();
	auto some_extra_value      = pdigicam_configure.extra_value();
	
}


void fill(const DIGICAM_CONFIGURE &pdigicam_configure) {
	pdigicam_configure.target_system(some_int8_t);
	pdigicam_configure.target_component(some_int8_t);
	pdigicam_configure.mode(some_int8_t);
	pdigicam_configure.shutter_speed(some_int16_t);
	pdigicam_configure.aperture(some_int8_t);
	pdigicam_configure.iso(some_int8_t);
	pdigicam_configure.exposure_type(some_int8_t);
	pdigicam_configure.command_id(some_int8_t);
	pdigicam_configure.engine_cut_off(some_int8_t);
	pdigicam_configure.extra_param(some_int8_t);
	pdigicam_configure.extra_value(some_float);
	
}


void on_SCALED_PRESSURE3(const SCALED_PRESSURE3 &pscaled_pressure3) {
	auto some_time_boot_ms = pscaled_pressure3.time_boot_ms();
	auto some_press_abs    = pscaled_pressure3.press_abs();
	auto some_press_diff   = pscaled_pressure3.press_diff();
	auto some_temperature  = pscaled_pressure3.temperature();
	
}


void fill(const SCALED_PRESSURE3 &pscaled_pressure3) {
	pscaled_pressure3.time_boot_ms(some_int32_t);
	pscaled_pressure3.press_abs(some_float);
	pscaled_pressure3.press_diff(some_float);
	pscaled_pressure3.temperature(some_int16_t);
	
}


void on_MISSION_REQUEST_PARTIAL_LIST(const MISSION_REQUEST_PARTIAL_LIST &pmission_request_partial_list) {
	auto some_target_system    = pmission_request_partial_list.target_system();
	auto some_target_component = pmission_request_partial_list.target_component();
	auto some_start_index      = pmission_request_partial_list.start_index();
	auto some_end_index        = pmission_request_partial_list.end_index();
	
	const auto src_mission_type = pmission_request_partial_list.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_PARAM_EXT_ACK(const PARAM_EXT_ACK &pparam_ext_ack) {
	
	const auto src_param_id = pparam_ext_ack.param_id();
	if (src_param_id.IS_EXISTS) {
		auto        array = src_param_id.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_param_value = pparam_ext_ack.param_value();
	if (src_param_value.IS_EXISTS) {
		auto        array = src_param_value.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
	const auto src_param_type = pparam_ext_ack.param_type();
	
	if (src_param_type.IS_EXISTS)
		auto some_param_type = src_param_type.CASE.EXISTS.value;
	
	const auto src_param_result = pparam_ext_ack.param_result();
	
	if (src_param_result.IS_EXISTS)
		auto some_param_result = src_param_result.CASE.EXISTS.value;
	
}


void fill(const PARAM_EXT_ACK &pparam_ext_ack) {
	
	const auto src_param_id = pparam_ext_ack.param_id();
	
	if (!src_param_id.IS_EXISTS)
		pparam_ext_ack.param_id(some_string);
	
	
	const auto src_param_value = pparam_ext_ack.param_value();
	
	if (!src_param_value.IS_EXISTS)
		pparam_ext_ack.param_value(some_string);
	
	
	const auto src_param_type = pparam_ext_ack.param_type();
	
	if (!src_param_type.IS_EXISTS)
		pparam_ext_ack.param_type(com::company::demo::MAV_PARAM_EXT_TYPE::MAV_PARAM_EXT_TYPE_UINT8);
	
	
	const auto src_param_result = pparam_ext_ack.param_result();
	
	if (!src_param_result.IS_EXISTS)
		pparam_ext_ack.param_result(com::company::demo::PARAM_ACK::PARAM_ACK_ACCEPTED);
	
	
}


void on_UAVCAN_NODE_INFO(const UAVCAN_NODE_INFO &puavcan_node_info) {
	auto some_time_usec  = puavcan_node_info.time_usec();
	auto some_uptime_sec = puavcan_node_info.uptime_sec();
	
	const auto src_name              = puavcan_node_info.name();
	if (src_name.IS_EXISTS) {
		auto        array = src_name.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_hw_version_major = puavcan_node_info.hw_version_major();
	auto       some_hw_version_minor = puavcan_node_info.hw_version_minor();
	
	const auto  src_hw_unique_id = puavcan_node_info.hw_unique_id();
	for (size_t index            = 0; index < UAVCAN_NODE_INFO::hw_unique_id_::len; index++)
		some_int8_t                   = src_hw_unique_id.get(index);
	auto        some_sw_version_major = puavcan_node_info.sw_version_major();
	auto        some_sw_version_minor = puavcan_node_info.sw_version_minor();
	auto        some_sw_vcs_commit    = puavcan_node_info.sw_vcs_commit();
	
}


void fill(const UAVCAN_NODE_INFO &puavcan_node_info) {
	puavcan_node_info.time_usec(some_int64_t);
	puavcan_node_info.uptime_sec(some_int32_t);
	
	const auto src_name = puavcan_node_info.name();
	
	if (!src_name.IS_EXISTS)
		puavcan_node_info.name(some_string);
	
	puavcan_node_info.hw_version_major(some_int8_t);
	puavcan_node_info.hw_version_minor(some_int8_t);
	puavcan_node_info.hw_unique_id(&some_int8_t);
	puavcan_node_info.sw_version_major(some_int8_t);
	puavcan_node_info.sw_version_minor(some_int8_t);
	puavcan_node_info.sw_vcs_commit(some_int32_t);
	
}


void on_DATA16(const DATA16 &pdata16) {
	auto some_typE = pdata16.typE();
	auto some_len  = pdata16.len();
	
	const auto  src_daTa = pdata16.daTa();
	for (size_t index    = 0; index < DATA16::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const DATA16 &pdata16) {
	pdata16.typE(some_int8_t);
	pdata16.len(some_int8_t);
	pdata16.daTa(&some_int8_t);
}


void on_SET_MAG_OFFSETS(const SET_MAG_OFFSETS &pset_mag_offsets) {
	auto some_target_system    = pset_mag_offsets.target_system();
	auto some_target_component = pset_mag_offsets.target_component();
	auto some_mag_ofs_x        = pset_mag_offsets.mag_ofs_x();
	auto some_mag_ofs_y        = pset_mag_offsets.mag_ofs_y();
	auto some_mag_ofs_z        = pset_mag_offsets.mag_ofs_z();
	
}


void fill(const SET_MAG_OFFSETS &pset_mag_offsets) {
	pset_mag_offsets.target_system(some_int8_t);
	pset_mag_offsets.target_component(some_int8_t);
	pset_mag_offsets.mag_ofs_x(some_int16_t);
	pset_mag_offsets.mag_ofs_y(some_int16_t);
	pset_mag_offsets.mag_ofs_z(some_int16_t);
	
}


void on_AP_ADC(const AP_ADC &pap_adc) {
	auto some_adc1 = pap_adc.adc1();
	auto some_adc2 = pap_adc.adc2();
	auto some_adc3 = pap_adc.adc3();
	auto some_adc4 = pap_adc.adc4();
	auto some_adc5 = pap_adc.adc5();
	auto some_adc6 = pap_adc.adc6();
	
}


void fill(const AP_ADC &pap_adc) {
	pap_adc.adc1(some_int16_t);
	pap_adc.adc2(some_int16_t);
	pap_adc.adc3(some_int16_t);
	pap_adc.adc4(some_int16_t);
	pap_adc.adc5(some_int16_t);
	pap_adc.adc6(some_int16_t);
	
}


void on_WIND(const WIND &pwind) {
	auto some_direction = pwind.direction();
	auto some_speed     = pwind.speed();
	auto some_speed_z   = pwind.speed_z();
	
}


void fill(const WIND &pwind) {
	pwind.direction(some_float);
	pwind.speed(some_float);
	pwind.speed_z(some_float);
	
}


void on_AUTOPILOT_VERSION_REQUEST(const AUTOPILOT_VERSION_REQUEST &pautopilot_version_request) {
	auto some_target_system    = pautopilot_version_request.target_system();
	auto some_target_component = pautopilot_version_request.target_component();
	
}


void fill(const AUTOPILOT_VERSION_REQUEST &pautopilot_version_request) {
	pautopilot_version_request.target_system(some_int8_t);
	pautopilot_version_request.target_component(some_int8_t);
	
}


void on_LOCAL_POSITION_NED(const LOCAL_POSITION_NED &plocal_position_ned) {
	auto some_time_boot_ms = plocal_position_ned.time_boot_ms();
	auto some_x            = plocal_position_ned.x();
	auto some_y            = plocal_position_ned.y();
	auto some_z            = plocal_position_ned.z();
	auto some_vx           = plocal_position_ned.vx();
	auto some_vy           = plocal_position_ned.vy();
	auto some_vz           = plocal_position_ned.vz();
	
}


void on_DATA_TRANSMISSION_HANDSHAKE(const DATA_TRANSMISSION_HANDSHAKE &pdata_transmission_handshake) {
	auto some_typE        = pdata_transmission_handshake.typE();
	auto some_size        = pdata_transmission_handshake.size();
	auto some_width       = pdata_transmission_handshake.width();
	auto some_height      = pdata_transmission_handshake.height();
	auto some_packets     = pdata_transmission_handshake.packets();
	auto some_payload     = pdata_transmission_handshake.payload();
	auto some_jpg_quality = pdata_transmission_handshake.jpg_quality();
	
}


void fill(const DATA_TRANSMISSION_HANDSHAKE &pdata_transmission_handshake) {
	pdata_transmission_handshake.typE(some_int8_t);
	pdata_transmission_handshake.size(some_int32_t);
	pdata_transmission_handshake.width(some_int16_t);
	pdata_transmission_handshake.height(some_int16_t);
	pdata_transmission_handshake.packets(some_int16_t);
	pdata_transmission_handshake.payload(some_int8_t);
	pdata_transmission_handshake.jpg_quality(some_int8_t);
	
}


void on_GPS_GLOBAL_ORIGIN(const GPS_GLOBAL_ORIGIN &pgps_global_origin) {
	auto some_latitude  = pgps_global_origin.latitude();
	auto some_longitude = pgps_global_origin.longitude();
	auto some_altitude  = pgps_global_origin.altitude();
	
	const auto src_time_usec = pgps_global_origin.time_usec();
	
	if (src_time_usec.IS_EXISTS)
		auto some_time_usec = src_time_usec.CASE.EXISTS.value;
	
}


void on_SCALED_IMU2(const SCALED_IMU2 &pscaled_imu2) {
	auto some_time_boot_ms = pscaled_imu2.time_boot_ms();
	auto some_xacc         = pscaled_imu2.xacc();
	auto some_yacc         = pscaled_imu2.yacc();
	auto some_zacc         = pscaled_imu2.zacc();
	auto some_xgyro        = pscaled_imu2.xgyro();
	auto some_ygyro        = pscaled_imu2.ygyro();
	auto some_zgyro        = pscaled_imu2.zgyro();
	auto some_xmag         = pscaled_imu2.xmag();
	auto some_ymag         = pscaled_imu2.ymag();
	auto some_zmag         = pscaled_imu2.zmag();
	
}


void fill(const SCALED_IMU2 &pscaled_imu2) {
	pscaled_imu2.time_boot_ms(some_int32_t);
	pscaled_imu2.xacc(some_int16_t);
	pscaled_imu2.yacc(some_int16_t);
	pscaled_imu2.zacc(some_int16_t);
	pscaled_imu2.xgyro(some_int16_t);
	pscaled_imu2.ygyro(some_int16_t);
	pscaled_imu2.zgyro(some_int16_t);
	pscaled_imu2.xmag(some_int16_t);
	pscaled_imu2.ymag(some_int16_t);
	pscaled_imu2.zmag(some_int16_t);
	
}


void on_ATTITUDE_QUATERNION(const ATTITUDE_QUATERNION &pattitude_quaternion) {
	auto some_time_boot_ms = pattitude_quaternion.time_boot_ms();
	auto some_q1           = pattitude_quaternion.q1();
	auto some_q2           = pattitude_quaternion.q2();
	auto some_q3           = pattitude_quaternion.q3();
	auto some_q4           = pattitude_quaternion.q4();
	auto some_rollspeed    = pattitude_quaternion.rollspeed();
	auto some_pitchspeed   = pattitude_quaternion.pitchspeed();
	auto some_yawspeed     = pattitude_quaternion.yawspeed();
	
}


void on_DATA64(const DATA64 &pdata64) {
	auto some_typE = pdata64.typE();
	auto some_len  = pdata64.len();
	
	const auto  src_daTa = pdata64.daTa();
	for (size_t index    = 0; index < DATA64::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const DATA64 &pdata64) {
	pdata64.typE(some_int8_t);
	pdata64.len(some_int8_t);
	pdata64.daTa(&some_int8_t);
}


void on_HIL_ACTUATOR_CONTROLS(const HIL_ACTUATOR_CONTROLS &phil_actuator_controls) {
	auto some_time_usec = phil_actuator_controls.time_usec();
	
	const auto  src_controls = phil_actuator_controls.controls();
	for (size_t index        = 0; index < HIL_ACTUATOR_CONTROLS::controls_::len; index++)
		some_float = src_controls.get(index);
	
	const auto src_mode = phil_actuator_controls.mode();
	
	if (src_mode.IS_EXISTS)
		auto some_mode  = src_mode.CASE.EXISTS.value;
	auto     some_flags = phil_actuator_controls.flags();
	
}


void on_POSITION_TARGET_LOCAL_NED(const POSITION_TARGET_LOCAL_NED &pposition_target_local_ned) {
	auto some_time_boot_ms = pposition_target_local_ned.time_boot_ms();
	
	const auto src_coordinate_frame = pposition_target_local_ned.coordinate_frame();
	
	if (src_coordinate_frame.IS_EXISTS)
		auto some_coordinate_frame = src_coordinate_frame.CASE.EXISTS.value;
	auto     some_type_mask        = pposition_target_local_ned.type_mask();
	auto     some_x                = pposition_target_local_ned.x();
	auto     some_y                = pposition_target_local_ned.y();
	auto     some_z                = pposition_target_local_ned.z();
	auto     some_vx               = pposition_target_local_ned.vx();
	auto     some_vy               = pposition_target_local_ned.vy();
	auto     some_vz               = pposition_target_local_ned.vz();
	auto     some_afx              = pposition_target_local_ned.afx();
	auto     some_afy              = pposition_target_local_ned.afy();
	auto     some_afz              = pposition_target_local_ned.afz();
	auto     some_yaw              = pposition_target_local_ned.yaw();
	auto     some_yaw_rate         = pposition_target_local_ned.yaw_rate();
	
}


void on_GIMBAL_REPORT(const GIMBAL_REPORT &pgimbal_report) {
	auto some_target_system    = pgimbal_report.target_system();
	auto some_target_component = pgimbal_report.target_component();
	auto some_delta_time       = pgimbal_report.delta_time();
	auto some_delta_angle_x    = pgimbal_report.delta_angle_x();
	auto some_delta_angle_y    = pgimbal_report.delta_angle_y();
	auto some_delta_angle_z    = pgimbal_report.delta_angle_z();
	auto some_delta_velocity_x = pgimbal_report.delta_velocity_x();
	auto some_delta_velocity_y = pgimbal_report.delta_velocity_y();
	auto some_delta_velocity_z = pgimbal_report.delta_velocity_z();
	auto some_joint_roll       = pgimbal_report.joint_roll();
	auto some_joint_el         = pgimbal_report.joint_el();
	auto some_joint_az         = pgimbal_report.joint_az();
	
}


void fill(const GIMBAL_REPORT &pgimbal_report) {
	pgimbal_report.target_system(some_int8_t);
	pgimbal_report.target_component(some_int8_t);
	pgimbal_report.delta_time(some_float);
	pgimbal_report.delta_angle_x(some_float);
	pgimbal_report.delta_angle_y(some_float);
	pgimbal_report.delta_angle_z(some_float);
	pgimbal_report.delta_velocity_x(some_float);
	pgimbal_report.delta_velocity_y(some_float);
	pgimbal_report.delta_velocity_z(some_float);
	pgimbal_report.joint_roll(some_float);
	pgimbal_report.joint_el(some_float);
	pgimbal_report.joint_az(some_float);
	
}


void on_DEVICE_OP_WRITE(const DEVICE_OP_WRITE &pdevice_op_write) {
	auto some_target_system    = pdevice_op_write.target_system();
	auto some_target_component = pdevice_op_write.target_component();
	auto some_request_id       = pdevice_op_write.request_id();
	
	const auto src_bustype = pdevice_op_write.bustype();
	
	if (src_bustype.IS_EXISTS)
		auto some_bustype = src_bustype.CASE.EXISTS.value;
	auto     some_bus     = pdevice_op_write.bus();
	auto     some_address = pdevice_op_write.address();
	
	const auto src_busname   = pdevice_op_write.busname();
	if (src_busname.IS_EXISTS) {
		auto        array = src_busname.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_regstart = pdevice_op_write.regstart();
	auto       some_count    = pdevice_op_write.count();
	
	const auto  src_daTa = pdevice_op_write.daTa();
	for (size_t index    = 0; index < DEVICE_OP_WRITE::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const DEVICE_OP_WRITE &pdevice_op_write) {
	pdevice_op_write.target_system(some_int8_t);
	pdevice_op_write.target_component(some_int8_t);
	pdevice_op_write.request_id(some_int32_t);
	
	const auto src_bustype = pdevice_op_write.bustype();
	
	if (!src_bustype.IS_EXISTS)
		pdevice_op_write.bustype(com::company::demo::DEVICE_OP_BUSTYPE::DEVICE_OP_BUSTYPE_I2C);
	
	pdevice_op_write.bus(some_int8_t);
	pdevice_op_write.address(some_int8_t);
	
	const auto src_busname = pdevice_op_write.busname();
	
	if (!src_busname.IS_EXISTS)
		pdevice_op_write.busname(some_string);
	
	pdevice_op_write.regstart(some_int8_t);
	pdevice_op_write.count(some_int8_t);
	pdevice_op_write.daTa(&some_int8_t);
}


void on_DISTANCE_SENSOR(const DISTANCE_SENSOR &pdistance_sensor) {
	auto some_time_boot_ms     = pdistance_sensor.time_boot_ms();
	auto some_min_distance     = pdistance_sensor.min_distance();
	auto some_max_distance     = pdistance_sensor.max_distance();
	auto some_current_distance = pdistance_sensor.current_distance();
	
	const auto src_typE = pdistance_sensor.typE();
	
	if (src_typE.IS_EXISTS)
		auto some_typE = src_typE.CASE.EXISTS.value;
	auto     some_id   = pdistance_sensor.id();
	
	const auto src_orientation = pdistance_sensor.orientation();
	
	if (src_orientation.IS_EXISTS)
		auto some_orientation = src_orientation.CASE.EXISTS.value;
	auto     some_covariance  = pdistance_sensor.covariance();
	
}


void fill(const DISTANCE_SENSOR &pdistance_sensor) {
	pdistance_sensor.time_boot_ms(some_int32_t);
	pdistance_sensor.min_distance(some_int16_t);
	pdistance_sensor.max_distance(some_int16_t);
	pdistance_sensor.current_distance(some_int16_t);
	
	const auto src_typE = pdistance_sensor.typE();
	
	if (!src_typE.IS_EXISTS)
		pdistance_sensor.typE(com::company::demo::MAV_DISTANCE_SENSOR::MAV_DISTANCE_SENSOR_LASER);
	
	pdistance_sensor.id(some_int8_t);
	
	const auto src_orientation = pdistance_sensor.orientation();
	
	if (!src_orientation.IS_EXISTS)
		pdistance_sensor.orientation(com::company::demo::MAV_SENSOR_ORIENTATION::NONE);
	
	pdistance_sensor.covariance(some_int8_t);
	
}


void on_HIL_OPTICAL_FLOW(const HIL_OPTICAL_FLOW &phil_optical_flow) {
	auto some_time_usec              = phil_optical_flow.time_usec();
	auto some_sensor_id              = phil_optical_flow.sensor_id();
	auto some_integration_time_us    = phil_optical_flow.integration_time_us();
	auto some_integrated_x           = phil_optical_flow.integrated_x();
	auto some_integrated_y           = phil_optical_flow.integrated_y();
	auto some_integrated_xgyro       = phil_optical_flow.integrated_xgyro();
	auto some_integrated_ygyro       = phil_optical_flow.integrated_ygyro();
	auto some_integrated_zgyro       = phil_optical_flow.integrated_zgyro();
	auto some_temperature            = phil_optical_flow.temperature();
	auto some_quality                = phil_optical_flow.quality();
	auto some_time_delta_distance_us = phil_optical_flow.time_delta_distance_us();
	auto some_distance               = phil_optical_flow.distance();
	
}


void fill(const HIL_OPTICAL_FLOW &phil_optical_flow) {
	phil_optical_flow.time_usec(some_int64_t);
	phil_optical_flow.sensor_id(some_int8_t);
	phil_optical_flow.integration_time_us(some_int32_t);
	phil_optical_flow.integrated_x(some_float);
	phil_optical_flow.integrated_y(some_float);
	phil_optical_flow.integrated_xgyro(some_float);
	phil_optical_flow.integrated_ygyro(some_float);
	phil_optical_flow.integrated_zgyro(some_float);
	phil_optical_flow.temperature(some_int16_t);
	phil_optical_flow.quality(some_int8_t);
	phil_optical_flow.time_delta_distance_us(some_int32_t);
	phil_optical_flow.distance(some_float);
	
}


void on_SCALED_PRESSURE2(const SCALED_PRESSURE2 &pscaled_pressure2) {
	auto some_time_boot_ms = pscaled_pressure2.time_boot_ms();
	auto some_press_abs    = pscaled_pressure2.press_abs();
	auto some_press_diff   = pscaled_pressure2.press_diff();
	auto some_temperature  = pscaled_pressure2.temperature();
	
}


void fill(const SCALED_PRESSURE2 &pscaled_pressure2) {
	pscaled_pressure2.time_boot_ms(some_int32_t);
	pscaled_pressure2.press_abs(some_float);
	pscaled_pressure2.press_diff(some_float);
	pscaled_pressure2.temperature(some_int16_t);
	
}


void on_WIND_COV(const WIND_COV &pwind_cov) {
	auto some_time_usec      = pwind_cov.time_usec();
	auto some_wind_x         = pwind_cov.wind_x();
	auto some_wind_y         = pwind_cov.wind_y();
	auto some_wind_z         = pwind_cov.wind_z();
	auto some_var_horiz      = pwind_cov.var_horiz();
	auto some_var_vert       = pwind_cov.var_vert();
	auto some_wind_alt       = pwind_cov.wind_alt();
	auto some_horiz_accuracy = pwind_cov.horiz_accuracy();
	auto some_vert_accuracy  = pwind_cov.vert_accuracy();
	
}


void fill(const WIND_COV &pwind_cov) {
	pwind_cov.time_usec(some_int64_t);
	pwind_cov.wind_x(some_float);
	pwind_cov.wind_y(some_float);
	pwind_cov.wind_z(some_float);
	pwind_cov.var_horiz(some_float);
	pwind_cov.var_vert(some_float);
	pwind_cov.wind_alt(some_float);
	pwind_cov.horiz_accuracy(some_float);
	pwind_cov.vert_accuracy(some_float);
	
}


void on_CHANGE_OPERATOR_CONTROL(const CHANGE_OPERATOR_CONTROL &pchange_operator_control) {
	auto some_target_system   = pchange_operator_control.target_system();
	auto some_control_request = pchange_operator_control.control_request();
	auto some_version         = pchange_operator_control.version();
	
	const auto src_passkey = pchange_operator_control.passkey();
	if (src_passkey.IS_EXISTS) {
		auto        array = src_passkey.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	
}


void on_GOPRO_SET_REQUEST(const GOPRO_SET_REQUEST &pgopro_set_request) {
	auto some_target_system    = pgopro_set_request.target_system();
	auto some_target_component = pgopro_set_request.target_component();
	
	const auto src_cmd_id = pgopro_set_request.cmd_id();
	
	if (src_cmd_id.IS_EXISTS)
		auto some_cmd_id = src_cmd_id.CASE.EXISTS.value;
	
	const auto  src_value = pgopro_set_request.value();
	for (size_t index     = 0; index < GOPRO_SET_REQUEST::value_::len; index++)
		some_int8_t = src_value.get(index);
	
}


void fill(const GOPRO_SET_REQUEST &pgopro_set_request) {
	pgopro_set_request.target_system(some_int8_t);
	pgopro_set_request.target_component(some_int8_t);
	
	const auto src_cmd_id = pgopro_set_request.cmd_id();
	
	if (!src_cmd_id.IS_EXISTS)
		pgopro_set_request.cmd_id(com::company::demo::GOPRO_COMMAND::GOPRO_COMMAND_POWER);
	
	pgopro_set_request.value(&some_int8_t);
}


void on_SYS_STATUS(const SYS_STATUS &psys_status) {
	
	const auto src_onboard_control_sensors_present = psys_status.onboard_control_sensors_present();
	
	if (src_onboard_control_sensors_present.IS_EXISTS)
		auto some_onboard_control_sensors_present = src_onboard_control_sensors_present.CASE.EXISTS.value;
	
	const auto src_onboard_control_sensors_enabled = psys_status.onboard_control_sensors_enabled();
	
	if (src_onboard_control_sensors_enabled.IS_EXISTS)
		auto some_onboard_control_sensors_enabled = src_onboard_control_sensors_enabled.CASE.EXISTS.value;
	
	const auto src_onboard_control_sensors_health = psys_status.onboard_control_sensors_health();
	
	if (src_onboard_control_sensors_health.IS_EXISTS)
		auto some_onboard_control_sensors_health = src_onboard_control_sensors_health.CASE.EXISTS.value;
	auto     some_load                           = psys_status.load();
	auto     some_voltage_battery                = psys_status.voltage_battery();
	auto     some_current_battery                = psys_status.current_battery();
	auto     some_battery_remaining              = psys_status.battery_remaining();
	auto     some_drop_rate_comm                 = psys_status.drop_rate_comm();
	auto     some_errors_comm                    = psys_status.errors_comm();
	auto     some_errors_count1                  = psys_status.errors_count1();
	auto     some_errors_count2                  = psys_status.errors_count2();
	auto     some_errors_count3                  = psys_status.errors_count3();
	auto     some_errors_count4                  = psys_status.errors_count4();
	
}


void on_MISSION_ITEM(const MISSION_ITEM &pmission_item) {
	auto some_target_system    = pmission_item.target_system();
	auto some_target_component = pmission_item.target_component();
	auto some_seq              = pmission_item.seq();
	
	const auto src_frame = pmission_item.frame();
	
	if (src_frame.IS_EXISTS)
		auto some_frame = src_frame.CASE.EXISTS.value;
	
	const auto src_command = pmission_item.command();
	
	if (src_command.IS_EXISTS)
		auto some_command      = src_command.CASE.EXISTS.value;
	auto     some_current      = pmission_item.current();
	auto     some_autocontinue = pmission_item.autocontinue();
	auto     some_param1       = pmission_item.param1();
	auto     some_param2       = pmission_item.param2();
	auto     some_param3       = pmission_item.param3();
	auto     some_param4       = pmission_item.param4();
	auto     some_x            = pmission_item.x();
	auto     some_y            = pmission_item.y();
	auto     some_z            = pmission_item.z();
	
	const auto src_mission_type = pmission_item.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_RAW_IMU(const RAW_IMU &praw_imu) {
	auto some_time_usec = praw_imu.time_usec();
	auto some_xacc      = praw_imu.xacc();
	auto some_yacc      = praw_imu.yacc();
	auto some_zacc      = praw_imu.zacc();
	auto some_xgyro     = praw_imu.xgyro();
	auto some_ygyro     = praw_imu.ygyro();
	auto some_zgyro     = praw_imu.zgyro();
	auto some_xmag      = praw_imu.xmag();
	auto some_ymag      = praw_imu.ymag();
	auto some_zmag      = praw_imu.zmag();
	
}


void on_COMMAND_INT(const COMMAND_INT &pcommand_int) {
	auto some_target_system    = pcommand_int.target_system();
	auto some_target_component = pcommand_int.target_component();
	
	const auto src_frame = pcommand_int.frame();
	
	if (src_frame.IS_EXISTS)
		auto some_frame = src_frame.CASE.EXISTS.value;
	
	const auto src_command = pcommand_int.command();
	
	if (src_command.IS_EXISTS)
		auto some_command      = src_command.CASE.EXISTS.value;
	auto     some_current      = pcommand_int.current();
	auto     some_autocontinue = pcommand_int.autocontinue();
	auto     some_param1       = pcommand_int.param1();
	auto     some_param2       = pcommand_int.param2();
	auto     some_param3       = pcommand_int.param3();
	auto     some_param4       = pcommand_int.param4();
	auto     some_x            = pcommand_int.x();
	auto     some_y            = pcommand_int.y();
	auto     some_z            = pcommand_int.z();
	
}


void on_OPTICAL_FLOW(const OPTICAL_FLOW &poptical_flow) {
	auto some_time_usec       = poptical_flow.time_usec();
	auto some_sensor_id       = poptical_flow.sensor_id();
	auto some_flow_x          = poptical_flow.flow_x();
	auto some_flow_y          = poptical_flow.flow_y();
	auto some_flow_comp_m_x   = poptical_flow.flow_comp_m_x();
	auto some_flow_comp_m_y   = poptical_flow.flow_comp_m_y();
	auto some_quality         = poptical_flow.quality();
	auto some_ground_distance = poptical_flow.ground_distance();
	
	const auto src_flow_rate_x = poptical_flow.flow_rate_x();
	
	if (src_flow_rate_x.IS_EXISTS)
		auto some_flow_rate_x = src_flow_rate_x.CASE.EXISTS.value;
	
	const auto src_flow_rate_y = poptical_flow.flow_rate_y();
	
	if (src_flow_rate_y.IS_EXISTS)
		auto some_flow_rate_y = src_flow_rate_y.CASE.EXISTS.value;
	
}


void on_MISSION_ITEM_INT(const MISSION_ITEM_INT &pmission_item_int) {
	auto some_target_system    = pmission_item_int.target_system();
	auto some_target_component = pmission_item_int.target_component();
	auto some_seq              = pmission_item_int.seq();
	
	const auto src_frame = pmission_item_int.frame();
	
	if (src_frame.IS_EXISTS)
		auto some_frame = src_frame.CASE.EXISTS.value;
	
	const auto src_command = pmission_item_int.command();
	
	if (src_command.IS_EXISTS)
		auto some_command      = src_command.CASE.EXISTS.value;
	auto     some_current      = pmission_item_int.current();
	auto     some_autocontinue = pmission_item_int.autocontinue();
	auto     some_param1       = pmission_item_int.param1();
	auto     some_param2       = pmission_item_int.param2();
	auto     some_param3       = pmission_item_int.param3();
	auto     some_param4       = pmission_item_int.param4();
	auto     some_x            = pmission_item_int.x();
	auto     some_y            = pmission_item_int.y();
	auto     some_z            = pmission_item_int.z();
	
	const auto src_mission_type = pmission_item_int.mission_type();
	
	if (src_mission_type.IS_EXISTS)
		auto some_mission_type = src_mission_type.CASE.EXISTS.value;
	
}


void on_VISION_POSITION_DELTA(const VISION_POSITION_DELTA &pvision_position_delta) {
	auto some_time_usec       = pvision_position_delta.time_usec();
	auto some_time_delta_usec = pvision_position_delta.time_delta_usec();
	
	const auto  src_angle_delta = pvision_position_delta.angle_delta();
	for (size_t index           = 0; index < VISION_POSITION_DELTA::angle_delta_::len; index++)
		some_float = src_angle_delta.get(index);
	
	const auto  src_position_delta = pvision_position_delta.position_delta();
	for (size_t index              = 0; index < VISION_POSITION_DELTA::position_delta_::len; index++)
		some_float              = src_position_delta.get(index);
	auto        some_confidence = pvision_position_delta.confidence();
	
}


void fill(const VISION_POSITION_DELTA &pvision_position_delta) {
	pvision_position_delta.time_usec(some_int64_t);
	pvision_position_delta.time_delta_usec(some_int64_t);
	pvision_position_delta.angle_delta(&some_float);
	pvision_position_delta.position_delta(&some_float);
	pvision_position_delta.confidence(some_float);
	
}


void on_LOGGING_DATA(const LOGGING_DATA &plogging_data) {
	auto some_target_system        = plogging_data.target_system();
	auto some_target_component     = plogging_data.target_component();
	auto some_sequence             = plogging_data.sequence();
	auto some_length               = plogging_data.length();
	auto some_first_message_offset = plogging_data.first_message_offset();
	
	const auto  src_daTa = plogging_data.daTa();
	for (size_t index    = 0; index < LOGGING_DATA::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const LOGGING_DATA &plogging_data) {
	plogging_data.target_system(some_int8_t);
	plogging_data.target_component(some_int8_t);
	plogging_data.sequence(some_int16_t);
	plogging_data.length(some_int8_t);
	plogging_data.first_message_offset(some_int8_t);
	plogging_data.daTa(&some_int8_t);
}


void on_DEVICE_OP_READ(const DEVICE_OP_READ &pdevice_op_read) {
	auto some_target_system    = pdevice_op_read.target_system();
	auto some_target_component = pdevice_op_read.target_component();
	auto some_request_id       = pdevice_op_read.request_id();
	
	const auto src_bustype = pdevice_op_read.bustype();
	
	if (src_bustype.IS_EXISTS)
		auto some_bustype = src_bustype.CASE.EXISTS.value;
	auto     some_bus     = pdevice_op_read.bus();
	auto     some_address = pdevice_op_read.address();
	
	const auto src_busname   = pdevice_op_read.busname();
	if (src_busname.IS_EXISTS) {
		auto        array = src_busname.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_regstart = pdevice_op_read.regstart();
	auto       some_count    = pdevice_op_read.count();
	
}


void fill(const DEVICE_OP_READ &pdevice_op_read) {
	pdevice_op_read.target_system(some_int8_t);
	pdevice_op_read.target_component(some_int8_t);
	pdevice_op_read.request_id(some_int32_t);
	
	const auto src_bustype = pdevice_op_read.bustype();
	
	if (!src_bustype.IS_EXISTS)
		pdevice_op_read.bustype(com::company::demo::DEVICE_OP_BUSTYPE::DEVICE_OP_BUSTYPE_I2C);
	
	pdevice_op_read.bus(some_int8_t);
	pdevice_op_read.address(some_int8_t);
	
	const auto src_busname = pdevice_op_read.busname();
	
	if (!src_busname.IS_EXISTS)
		pdevice_op_read.busname(some_string);
	
	pdevice_op_read.regstart(some_int8_t);
	pdevice_op_read.count(some_int8_t);
	
}


void on_MAG_CAL_PROGRESS(const MAG_CAL_PROGRESS &pmag_cal_progress) {
	auto some_compass_id = pmag_cal_progress.compass_id();
	auto some_cal_mask   = pmag_cal_progress.cal_mask();
	
	const auto src_cal_status = pmag_cal_progress.cal_status();
	
	if (src_cal_status.IS_EXISTS)
		auto some_cal_status     = src_cal_status.CASE.EXISTS.value;
	auto     some_attempt        = pmag_cal_progress.attempt();
	auto     some_completion_pct = pmag_cal_progress.completion_pct();
	
	const auto  src_completion_mask = pmag_cal_progress.completion_mask();
	for (size_t index               = 0; index < MAG_CAL_PROGRESS::completion_mask_::len; index++)
		some_int8_t              = src_completion_mask.get(index);
	auto        some_direction_x = pmag_cal_progress.direction_x();
	auto        some_direction_y = pmag_cal_progress.direction_y();
	auto        some_direction_z = pmag_cal_progress.direction_z();
	
}


void fill(const MAG_CAL_PROGRESS &pmag_cal_progress) {
	pmag_cal_progress.compass_id(some_int8_t);
	pmag_cal_progress.cal_mask(some_int8_t);
	
	const auto src_cal_status = pmag_cal_progress.cal_status();
	
	if (!src_cal_status.IS_EXISTS)
		pmag_cal_progress.cal_status(com::company::demo::MAG_CAL_STATUS::MAG_CAL_NOT_STARTED);
	
	pmag_cal_progress.attempt(some_int8_t);
	pmag_cal_progress.completion_pct(some_int8_t);
	pmag_cal_progress.completion_mask(&some_int8_t);
	pmag_cal_progress.direction_x(some_float);
	pmag_cal_progress.direction_y(some_float);
	pmag_cal_progress.direction_z(some_float);
	
}


void on_HIGHRES_IMU(const HIGHRES_IMU &phighres_imu) {
	auto some_time_usec      = phighres_imu.time_usec();
	auto some_xacc           = phighres_imu.xacc();
	auto some_yacc           = phighres_imu.yacc();
	auto some_zacc           = phighres_imu.zacc();
	auto some_xgyro          = phighres_imu.xgyro();
	auto some_ygyro          = phighres_imu.ygyro();
	auto some_zgyro          = phighres_imu.zgyro();
	auto some_xmag           = phighres_imu.xmag();
	auto some_ymag           = phighres_imu.ymag();
	auto some_zmag           = phighres_imu.zmag();
	auto some_abs_pressure   = phighres_imu.abs_pressure();
	auto some_diff_pressure  = phighres_imu.diff_pressure();
	auto some_pressure_alt   = phighres_imu.pressure_alt();
	auto some_temperature    = phighres_imu.temperature();
	auto some_fields_updated = phighres_imu.fields_updated();
	
}


void fill(const HIGHRES_IMU &phighres_imu) {
	phighres_imu.time_usec(some_int64_t);
	phighres_imu.xacc(some_float);
	phighres_imu.yacc(some_float);
	phighres_imu.zacc(some_float);
	phighres_imu.xgyro(some_float);
	phighres_imu.ygyro(some_float);
	phighres_imu.zgyro(some_float);
	phighres_imu.xmag(some_float);
	phighres_imu.ymag(some_float);
	phighres_imu.zmag(some_float);
	phighres_imu.abs_pressure(some_float);
	phighres_imu.diff_pressure(some_float);
	phighres_imu.pressure_alt(some_float);
	phighres_imu.temperature(some_float);
	phighres_imu.fields_updated(some_int16_t);
	
}


void on_EXTENDED_SYS_STATE(const EXTENDED_SYS_STATE &pextended_sys_state) {
	
	const auto src_vtol_state = pextended_sys_state.vtol_state();
	
	if (src_vtol_state.IS_EXISTS)
		auto some_vtol_state = src_vtol_state.CASE.EXISTS.value;
	
	const auto src_landed_state = pextended_sys_state.landed_state();
	
	if (src_landed_state.IS_EXISTS)
		auto some_landed_state = src_landed_state.CASE.EXISTS.value;
	
}


void fill(const EXTENDED_SYS_STATE &pextended_sys_state) {
	
	const auto src_vtol_state = pextended_sys_state.vtol_state();
	
	if (!src_vtol_state.IS_EXISTS)
		pextended_sys_state.vtol_state(com::company::demo::MAV_VTOL_STATE::MAV_VTOL_STATE_UNDEFINED);
	
	
	const auto src_landed_state = pextended_sys_state.landed_state();
	
	if (!src_landed_state.IS_EXISTS)
		pextended_sys_state.landed_state(com::company::demo::MAV_LANDED_STATE::MAV_LANDED_STATE_UNDEFINED);
	
	
}


void on_UAVIONIX_ADSB_OUT_DYNAMIC(const UAVIONIX_ADSB_OUT_DYNAMIC &puavionix_adsb_out_dynamic) {
	auto some_utcTime = puavionix_adsb_out_dynamic.utcTime();
	auto some_gpsLat  = puavionix_adsb_out_dynamic.gpsLat();
	auto some_gpsLon  = puavionix_adsb_out_dynamic.gpsLon();
	auto some_gpsAlt  = puavionix_adsb_out_dynamic.gpsAlt();
	
	const auto src_gpsFix = puavionix_adsb_out_dynamic.gpsFix();
	
	if (src_gpsFix.IS_EXISTS)
		auto some_gpsFix       = src_gpsFix.CASE.EXISTS.value;
	auto     some_numSats      = puavionix_adsb_out_dynamic.numSats();
	auto     some_baroAltMSL   = puavionix_adsb_out_dynamic.baroAltMSL();
	auto     some_accuracyHor  = puavionix_adsb_out_dynamic.accuracyHor();
	auto     some_accuracyVert = puavionix_adsb_out_dynamic.accuracyVert();
	auto     some_accuracyVel  = puavionix_adsb_out_dynamic.accuracyVel();
	auto     some_velVert      = puavionix_adsb_out_dynamic.velVert();
	auto     some_velNS        = puavionix_adsb_out_dynamic.velNS();
	auto     some_VelEW        = puavionix_adsb_out_dynamic.VelEW();
	
	const auto src_emergencyStatus = puavionix_adsb_out_dynamic.emergencyStatus();
	
	if (src_emergencyStatus.IS_EXISTS)
		auto some_emergencyStatus = src_emergencyStatus.CASE.EXISTS.value;
	
	const auto src_state = puavionix_adsb_out_dynamic.state();
	
	if (src_state.IS_EXISTS)
		auto some_state  = src_state.CASE.EXISTS.value;
	auto     some_squawk = puavionix_adsb_out_dynamic.squawk();
	
}


void fill(const UAVIONIX_ADSB_OUT_DYNAMIC &puavionix_adsb_out_dynamic) {
	puavionix_adsb_out_dynamic.utcTime(some_int32_t);
	puavionix_adsb_out_dynamic.gpsLat(some_int32_t);
	puavionix_adsb_out_dynamic.gpsLon(some_int32_t);
	puavionix_adsb_out_dynamic.gpsAlt(some_int32_t);
	
	const auto src_gpsFix = puavionix_adsb_out_dynamic.gpsFix();
	
	if (!src_gpsFix.IS_EXISTS)
		puavionix_adsb_out_dynamic.gpsFix(com::company::demo::UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX::UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0);
	
	puavionix_adsb_out_dynamic.numSats(some_int8_t);
	puavionix_adsb_out_dynamic.baroAltMSL(some_int32_t);
	puavionix_adsb_out_dynamic.accuracyHor(some_int32_t);
	puavionix_adsb_out_dynamic.accuracyVert(some_int16_t);
	puavionix_adsb_out_dynamic.accuracyVel(some_int16_t);
	puavionix_adsb_out_dynamic.velVert(some_int16_t);
	puavionix_adsb_out_dynamic.velNS(some_int16_t);
	puavionix_adsb_out_dynamic.VelEW(some_int16_t);
	
	const auto src_emergencyStatus = puavionix_adsb_out_dynamic.emergencyStatus();
	
	if (!src_emergencyStatus.IS_EXISTS)
		puavionix_adsb_out_dynamic.emergencyStatus(com::company::demo::UAVIONIX_ADSB_EMERGENCY_STATUS::UAVIONIX_ADSB_OUT_NO_EMERGENCY);
	
	
	const auto src_state = puavionix_adsb_out_dynamic.state();
	
	if (!src_state.IS_EXISTS)
		puavionix_adsb_out_dynamic.state(com::company::demo::UAVIONIX_ADSB_OUT_DYNAMIC_STATE::UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE);
	
	puavionix_adsb_out_dynamic.squawk(some_int16_t);
	
}


void on_GOPRO_GET_RESPONSE(const GOPRO_GET_RESPONSE &pgopro_get_response) {
	
	const auto src_cmd_id = pgopro_get_response.cmd_id();
	
	if (src_cmd_id.IS_EXISTS)
		auto some_cmd_id = src_cmd_id.CASE.EXISTS.value;
	
	const auto src_status = pgopro_get_response.status();
	
	if (src_status.IS_EXISTS)
		auto some_status = src_status.CASE.EXISTS.value;
	
	const auto  src_value = pgopro_get_response.value();
	for (size_t index     = 0; index < GOPRO_GET_RESPONSE::value_::len; index++)
		some_int8_t = src_value.get(index);
	
}


void fill(const GOPRO_GET_RESPONSE &pgopro_get_response) {
	
	const auto src_cmd_id = pgopro_get_response.cmd_id();
	
	if (!src_cmd_id.IS_EXISTS)
		pgopro_get_response.cmd_id(com::company::demo::GOPRO_COMMAND::GOPRO_COMMAND_POWER);
	
	
	const auto src_status = pgopro_get_response.status();
	
	if (!src_status.IS_EXISTS)
		pgopro_get_response.status(com::company::demo::GOPRO_REQUEST_STATUS::GOPRO_REQUEST_SUCCESS);
	
	pgopro_get_response.value(&some_int8_t);
}


void on_GPS_INJECT_DATA(const GPS_INJECT_DATA &pgps_inject_data) {
	auto some_target_system    = pgps_inject_data.target_system();
	auto some_target_component = pgps_inject_data.target_component();
	auto some_len              = pgps_inject_data.len();
	
	const auto  src_daTa = pgps_inject_data.daTa();
	for (size_t index    = 0; index < GPS_INJECT_DATA::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const GPS_INJECT_DATA &pgps_inject_data) {
	pgps_inject_data.target_system(some_int8_t);
	pgps_inject_data.target_component(some_int8_t);
	pgps_inject_data.len(some_int8_t);
	pgps_inject_data.daTa(&some_int8_t);
}


void on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(const UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT &puavionix_adsb_transceiver_health_report) {
	
	const auto src_rfHealth = puavionix_adsb_transceiver_health_report.rfHealth();
	
	if (src_rfHealth.IS_EXISTS)
		auto some_rfHealth = src_rfHealth.CASE.EXISTS.value;
	
}


void fill(const UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT &puavionix_adsb_transceiver_health_report) {
	
	const auto src_rfHealth = puavionix_adsb_transceiver_health_report.rfHealth();
	
	if (!src_rfHealth.IS_EXISTS)
		puavionix_adsb_transceiver_health_report.rfHealth(com::company::demo::UAVIONIX_ADSB_RF_HEALTH::UAVIONIX_ADSB_RF_HEALTH_INITIALIZING);
	
	
}


void on_ATTITUDE_QUATERNION_COV(const ATTITUDE_QUATERNION_COV &pattitude_quaternion_cov) {
	auto some_time_usec = pattitude_quaternion_cov.time_usec();
	
	const auto  src_q = pattitude_quaternion_cov.q();
	for (size_t index = 0; index < ATTITUDE_QUATERNION_COV::q_::len; index++)
		some_float              = src_q.get(index);
	auto        some_rollspeed  = pattitude_quaternion_cov.rollspeed();
	auto        some_pitchspeed = pattitude_quaternion_cov.pitchspeed();
	auto        some_yawspeed   = pattitude_quaternion_cov.yawspeed();
	
	const auto  src_covariance = pattitude_quaternion_cov.covariance();
	for (size_t index          = 0; index < ATTITUDE_QUATERNION_COV::covariance_::len; index++)
		some_float = src_covariance.get(index);
	
}


void on_NAMED_VALUE_INT(const NAMED_VALUE_INT &pnamed_value_int) {
	auto some_time_boot_ms = pnamed_value_int.time_boot_ms();
	
	const auto src_name   = pnamed_value_int.name();
	if (src_name.IS_EXISTS) {
		auto        array = src_name.CASE.EXISTS;
		for (size_t index = 0; index < array.length; index++)
			some_string = array.get(index);
	}
	auto       some_value = pnamed_value_int.value();
	
}


void fill(const NAMED_VALUE_INT &pnamed_value_int) {
	pnamed_value_int.time_boot_ms(some_int32_t);
	
	const auto src_name = pnamed_value_int.name();
	
	if (!src_name.IS_EXISTS)
		pnamed_value_int.name(some_string);
	
	pnamed_value_int.value(some_int32_t);
	
}


void on_RPM(const RPM &prpm) {
	auto some_rpm1 = prpm.rpm1();
	auto some_rpm2 = prpm.rpm2();
	
}


void fill(const RPM &prpm) {
	prpm.rpm1(some_float);
	prpm.rpm2(some_float);
	
}


void on_GPS_RTCM_DATA(const GPS_RTCM_DATA &pgps_rtcm_data) {
	auto some_flags = pgps_rtcm_data.flags();
	auto some_len   = pgps_rtcm_data.len();
	
	const auto  src_daTa = pgps_rtcm_data.daTa();
	for (size_t index    = 0; index < GPS_RTCM_DATA::daTa_::len; index++)
		some_int8_t = src_daTa.get(index);
	
}


void fill(const GPS_RTCM_DATA &pgps_rtcm_data) {
	pgps_rtcm_data.flags(some_int8_t);
	pgps_rtcm_data.len(some_int8_t);
	pgps_rtcm_data.daTa(&some_int8_t);
}


void on_GLOBAL_VISION_POSITION_ESTIMATE(const GLOBAL_VISION_POSITION_ESTIMATE &pglobal_vision_position_estimate) {
	auto some_usec  = pglobal_vision_position_estimate.usec();
	auto some_x     = pglobal_vision_position_estimate.x();
	auto some_y     = pglobal_vision_position_estimate.y();
	auto some_z     = pglobal_vision_position_estimate.z();
	auto some_roll  = pglobal_vision_position_estimate.roll();
	auto some_pitch = pglobal_vision_position_estimate.pitch();
	auto some_yaw   = pglobal_vision_position_estimate.yaw();
	
}


void on_FILE_TRANSFER_PROTOCOL(const FILE_TRANSFER_PROTOCOL &pfile_transfer_protocol) {
	auto some_target_network   = pfile_transfer_protocol.target_network();
	auto some_target_system    = pfile_transfer_protocol.target_system();
	auto some_target_component = pfile_transfer_protocol.target_component();
	
	const auto  src_payload = pfile_transfer_protocol.payload();
	for (size_t index       = 0; index < FILE_TRANSFER_PROTOCOL::payload_::len; index++)
		some_int8_t = src_payload.get(index);
	
}


void fill(const FILE_TRANSFER_PROTOCOL &pfile_transfer_protocol) {
	pfile_transfer_protocol.target_network(some_int8_t);
	pfile_transfer_protocol.target_system(some_int8_t);
	pfile_transfer_protocol.target_component(some_int8_t);
	pfile_transfer_protocol.payload(&some_int8_t);
}


void on_RANGEFINDER(const RANGEFINDER &prangefinder) {
	auto some_distance = prangefinder.distance();
	auto some_voltage  = prangefinder.voltage();
	
}


void fill(const RANGEFINDER &prangefinder) {
	prangefinder.distance(some_float);
	prangefinder.voltage(some_float);
	
}


void on_RADIO_STATUS(const RADIO_STATUS &pradio_status) {
	auto some_rssi     = pradio_status.rssi();
	auto some_remrssi  = pradio_status.remrssi();
	auto some_txbuf    = pradio_status.txbuf();
	auto some_noise    = pradio_status.noise();
	auto some_remnoise = pradio_status.remnoise();
	auto some_rxerrors = pradio_status.rxerrors();
	auto some_fixeD    = pradio_status.fixeD();
	
}


void fill(const RADIO_STATUS &pradio_status) {
	pradio_status.rssi(some_int8_t);
	pradio_status.remrssi(some_int8_t);
	pradio_status.txbuf(some_int8_t);
	pradio_status.noise(some_int8_t);
	pradio_status.remnoise(some_int8_t);
	pradio_status.rxerrors(some_int16_t);
	pradio_status.fixeD(some_int16_t);
	
}


void on_FENCE_POINT(const FENCE_POINT &pfence_point) {
	auto some_target_system    = pfence_point.target_system();
	auto some_target_component = pfence_point.target_component();
	auto some_idx              = pfence_point.idx();
	auto some_count            = pfence_point.count();
	auto some_lat              = pfence_point.lat();
	auto some_lng              = pfence_point.lng();
	
}


void fill(const FENCE_POINT &pfence_point) {
	pfence_point.target_system(some_int8_t);
	pfence_point.target_component(some_int8_t);
	pfence_point.idx(some_int8_t);
	pfence_point.count(some_int8_t);
	pfence_point.lat(some_float);
	pfence_point.lng(some_float);
	
}


void on_RESOURCE_REQUEST(const RESOURCE_REQUEST &presource_request) {
	auto some_request_id = presource_request.request_id();
	auto some_uri_type   = presource_request.uri_type();
	
	const auto  src_uri = presource_request.uri();
	for (size_t index   = 0; index < RESOURCE_REQUEST::uri_::len; index++)
		some_int8_t                = src_uri.get(index);
	auto        some_transfer_type = presource_request.transfer_type();
	
	const auto  src_storage = presource_request.storage();
	for (size_t index       = 0; index < RESOURCE_REQUEST::storage_::len; index++)
		some_int8_t = src_storage.get(index);
	
}


void fill(const RESOURCE_REQUEST &presource_request) {
	presource_request.request_id(some_int8_t);
	presource_request.uri_type(some_int8_t);
	presource_request.uri(&some_int8_t);
	presource_request.transfer_type(some_int8_t);
	presource_request.storage(&some_int8_t);
}

struct CommunicationChannel_demo {
	CommunicationChannel<CommunicationChannel_demo> channel;
	
	void on_RESOURCE_REQUEST(Pack *pack) {
		
		RESOURCE_REQUEST presource_request = {pack, *pack->bytes};
		
		::on_RESOURCE_REQUEST(presource_request);
		//printf( "RESOURCE_REQUEST pass\n");
	}
	
	void on_ATTITUDE_TARGET(Pack *pack) {
		
		ATTITUDE_TARGET pattitude_target = {pack, *pack->bytes};
		
		::on_ATTITUDE_TARGET(pattitude_target);
		//printf( "ATTITUDE_TARGET pass\n");
	}
	
	void on_MISSION_COUNT(Pack *pack) {
		
		CURSORS(curs);
		MISSION_COUNT pmission_count = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_COUNT(pmission_count);
		//printf( "MISSION_COUNT pass\n");
	}
	
	void on_ADSB_VEHICLE(Pack *pack) {
		
		CURSORS(curs);
		ADSB_VEHICLE padsb_vehicle = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_ADSB_VEHICLE(padsb_vehicle);
		//printf( "ADSB_VEHICLE pass\n");
	}
	
	void on_MESSAGE_INTERVAL(Pack *pack) {
		
		MESSAGE_INTERVAL pmessage_interval = {pack, *pack->bytes};
		
		::on_MESSAGE_INTERVAL(pmessage_interval);
		//printf( "MESSAGE_INTERVAL pass\n");
	}
	
	void on_ESTIMATOR_STATUS(Pack *pack) {
		
		CURSORS(curs);
		ESTIMATOR_STATUS pestimator_status = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_ESTIMATOR_STATUS(pestimator_status);
		//printf( "ESTIMATOR_STATUS pass\n");
	}
	
	void on_TIMESYNC(Pack *pack) {
		
		TIMESYNC ptimesync = {pack, *pack->bytes};
		
		::on_TIMESYNC(ptimesync);
		//printf( "TIMESYNC pass\n");
	}
	
	void on_GLOBAL_POSITION_INT_COV(Pack *pack) {
		
		CURSORS(curs);
		GLOBAL_POSITION_INT_COV pglobal_position_int_cov = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_GLOBAL_POSITION_INT_COV(pglobal_position_int_cov);
		//printf( "GLOBAL_POSITION_INT_COV pass\n");
	}
	
	void on_BUTTON_CHANGE(Pack *pack) {
		
		BUTTON_CHANGE pbutton_change = {pack, *pack->bytes};
		
		::on_BUTTON_CHANGE(pbutton_change);
		//printf( "BUTTON_CHANGE pass\n");
	}
	
	void on_SAFETY_SET_ALLOWED_AREA(Pack *pack) {
		
		CURSORS(curs);
		SAFETY_SET_ALLOWED_AREA psafety_set_allowed_area = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area);
		//printf( "SAFETY_SET_ALLOWED_AREA pass\n");
	}
	
	void on_STORAGE_INFORMATION(Pack *pack) {
		
		STORAGE_INFORMATION pstorage_information = {pack, *pack->bytes};
		
		::on_STORAGE_INFORMATION(pstorage_information);
		//printf( "STORAGE_INFORMATION pass\n");
	}
	
	void on_COLLISION(Pack *pack) {
		
		CURSORS(curs);
		COLLISION pcollision = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_COLLISION(pcollision);
		//printf( "COLLISION pass\n");
	}
	
	void on_ALTITUDE(Pack *pack) {
		
		ALTITUDE paltitude = {pack, *pack->bytes};
		
		::on_ALTITUDE(paltitude);
		//printf( "ALTITUDE pass\n");
	}
	
	void on_HIL_STATE_QUATERNION(Pack *pack) {
		
		HIL_STATE_QUATERNION phil_state_quaternion = {pack, *pack->bytes};
		
		::on_HIL_STATE_QUATERNION(phil_state_quaternion);
		//printf( "HIL_STATE_QUATERNION pass\n");
	}
	
	void on_CAMERA_INFORMATION(Pack *pack) {
		
		CURSORS(curs);
		CAMERA_INFORMATION pcamera_information = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_CAMERA_INFORMATION(pcamera_information);
		//printf( "CAMERA_INFORMATION pass\n");
	}
	
	void on_GPS_STATUS(Pack *pack) {
		
		GPS_STATUS pgps_status = {pack, *pack->bytes};
		
		::on_GPS_STATUS(pgps_status);
		//printf( "GPS_STATUS pass\n");
	}
	
	void on_PARAM_SET(Pack *pack) {
		
		CURSORS(curs);
		PARAM_SET pparam_set = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_PARAM_SET(pparam_set);
		//printf( "PARAM_SET pass\n");
	}
	
	void on_TERRAIN_DATA(Pack *pack) {
		
		TERRAIN_DATA pterrain_data = {pack, *pack->bytes};
		
		::on_TERRAIN_DATA(pterrain_data);
		//printf( "TERRAIN_DATA pass\n");
	}
	
	void on_RC_CHANNELS_OVERRIDE(Pack *pack) {
		
		RC_CHANNELS_OVERRIDE prc_channels_override = {pack, *pack->bytes};
		
		::on_RC_CHANNELS_OVERRIDE(prc_channels_override);
		//printf( "RC_CHANNELS_OVERRIDE pass\n");
	}
	
	void on_SCALED_IMU(Pack *pack) {
		
		SCALED_IMU pscaled_imu = {pack, *pack->bytes};
		
		::on_SCALED_IMU(pscaled_imu);
		//printf( "SCALED_IMU pass\n");
	}
	
	void on_DEBUG(Pack *pack) {
		
		DEBUG pdebug = {pack, *pack->bytes};
		
		::on_DEBUG(pdebug);
		//printf( "DEBUG pass\n");
	}
	
	void on_CAMERA_IMAGE_CAPTURED(Pack *pack) {
		
		CURSORS(curs);
		CAMERA_IMAGE_CAPTURED pcamera_image_captured = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_CAMERA_IMAGE_CAPTURED(pcamera_image_captured);
		//printf( "CAMERA_IMAGE_CAPTURED pass\n");
	}
	
	void on_LOG_ENTRY(Pack *pack) {
		
		LOG_ENTRY plog_entry = {pack, *pack->bytes};
		
		::on_LOG_ENTRY(plog_entry);
		//printf( "LOG_ENTRY pass\n");
	}
	
	void on_ACTUATOR_CONTROL_TARGET(Pack *pack) {
		
		ACTUATOR_CONTROL_TARGET pactuator_control_target = {pack, *pack->bytes};
		
		::on_ACTUATOR_CONTROL_TARGET(pactuator_control_target);
		//printf( "ACTUATOR_CONTROL_TARGET pass\n");
	}
	
	void on_HIGH_LATENCY(Pack *pack) {
		
		CURSORS(curs);
		HIGH_LATENCY phigh_latency = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_HIGH_LATENCY(phigh_latency);
		//printf( "HIGH_LATENCY pass\n");
	}
	
	void on_PARAM_REQUEST_READ(Pack *pack) {
		
		CURSORS(curs);
		PARAM_REQUEST_READ pparam_request_read = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_PARAM_REQUEST_READ(pparam_request_read);
		//printf( "PARAM_REQUEST_READ pass\n");
	}
	
	void on_SET_ATTITUDE_TARGET(Pack *pack) {
		
		SET_ATTITUDE_TARGET pset_attitude_target = {pack, *pack->bytes};
		
		::on_SET_ATTITUDE_TARGET(pset_attitude_target);
		//printf( "SET_ATTITUDE_TARGET pass\n");
	}
	
	void on_FOLLOW_TARGET(Pack *pack) {
		
		FOLLOW_TARGET pfollow_target = {pack, *pack->bytes};
		
		::on_FOLLOW_TARGET(pfollow_target);
		//printf( "FOLLOW_TARGET pass\n");
	}
	
	void on_HIL_STATE(Pack *pack) {
		
		HIL_STATE phil_state = {pack, *pack->bytes};
		
		::on_HIL_STATE(phil_state);
		//printf( "HIL_STATE pass\n");
	}
	
	void on_HOME_POSITION(Pack *pack) {
		
		CURSORS(curs);
		HOME_POSITION phome_position = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_HOME_POSITION(phome_position);
		//printf( "HOME_POSITION pass\n");
	}
	
	void on_GPS2_RAW(Pack *pack) {
		
		CURSORS(curs);
		GPS2_RAW pgps2_raw = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_GPS2_RAW(pgps2_raw);
		//printf( "GPS2_RAW pass\n");
	}
	
	void on_MEMORY_VECT(Pack *pack) {
		
		MEMORY_VECT pmemory_vect = {pack, *pack->bytes};
		
		::on_MEMORY_VECT(pmemory_vect);
		//printf( "MEMORY_VECT pass\n");
	}
	
	void on_REQUEST_DATA_STREAM(Pack *pack) {
		
		REQUEST_DATA_STREAM prequest_data_stream = {pack, *pack->bytes};
		
		::on_REQUEST_DATA_STREAM(prequest_data_stream);
		//printf( "REQUEST_DATA_STREAM pass\n");
	}
	
	void on_HIL_CONTROLS(Pack *pack) {
		
		CURSORS(curs);
		HIL_CONTROLS phil_controls = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_HIL_CONTROLS(phil_controls);
		//printf( "HIL_CONTROLS pass\n");
	}
	
	void on_HIL_SENSOR(Pack *pack) {
		
		HIL_SENSOR phil_sensor = {pack, *pack->bytes};
		
		::on_HIL_SENSOR(phil_sensor);
		//printf( "HIL_SENSOR pass\n");
	}
	
	void on_SETUP_SIGNING(Pack *pack) {
		
		SETUP_SIGNING psetup_signing = {pack, *pack->bytes};
		
		::on_SETUP_SIGNING(psetup_signing);
		//printf( "SETUP_SIGNING pass\n");
	}
	
	void on_GPS_RTK(Pack *pack) {
		
		GPS_RTK pgps_rtk = {pack, *pack->bytes};
		
		::on_GPS_RTK(pgps_rtk);
		//printf( "GPS_RTK pass\n");
	}
	
	void on_PARAM_REQUEST_LIST(Pack *pack) {
		
		PARAM_REQUEST_LIST pparam_request_list = {pack, *pack->bytes};
		
		::on_PARAM_REQUEST_LIST(pparam_request_list);
		//printf( "PARAM_REQUEST_LIST pass\n");
	}
	
	void on_LANDING_TARGET(Pack *pack) {
		
		CURSORS(curs);
		LANDING_TARGET planding_target = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_LANDING_TARGET(planding_target);
		//printf( "LANDING_TARGET pass\n");
	}
	
	void on_SET_ACTUATOR_CONTROL_TARGET(Pack *pack) {
		
		SET_ACTUATOR_CONTROL_TARGET pset_actuator_control_target = {pack, *pack->bytes};
		
		::on_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target);
		//printf( "SET_ACTUATOR_CONTROL_TARGET pass\n");
	}
	
	void on_CONTROL_SYSTEM_STATE(Pack *pack) {
		
		CONTROL_SYSTEM_STATE pcontrol_system_state = {pack, *pack->bytes};
		
		::on_CONTROL_SYSTEM_STATE(pcontrol_system_state);
		//printf( "CONTROL_SYSTEM_STATE pass\n");
	}
	
	void on_SET_POSITION_TARGET_GLOBAL_INT(Pack *pack) {
		
		CURSORS(curs);
		SET_POSITION_TARGET_GLOBAL_INT pset_position_target_global_int = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int);
		//printf( "SET_POSITION_TARGET_GLOBAL_INT pass\n");
	}
	
	void on_VIBRATION(Pack *pack) {
		
		VIBRATION pvibration = {pack, *pack->bytes};
		
		::on_VIBRATION(pvibration);
		//printf( "VIBRATION pass\n");
	}
	
	void on_PING33(Pack *pack) {
		
		CURSORS(curs);
		PING33 pping33 = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_PING33(pping33);
		//printf( "PING33 pass\n");
	}
	
	void on_VFR_HUD(Pack *pack) {
		
		VFR_HUD pvfr_hud = {pack, *pack->bytes};
		
		::on_VFR_HUD(pvfr_hud);
		//printf( "VFR_HUD pass\n");
	}
	
	void on_MISSION_SET_CURRENT(Pack *pack) {
		
		MISSION_SET_CURRENT pmission_set_current = {pack, *pack->bytes};
		
		::on_MISSION_SET_CURRENT(pmission_set_current);
		//printf( "MISSION_SET_CURRENT pass\n");
	}
	
	void on_HIL_GPS(Pack *pack) {
		
		HIL_GPS phil_gps = {pack, *pack->bytes};
		
		::on_HIL_GPS(phil_gps);
		//printf( "HIL_GPS pass\n");
	}
	
	void on_NAV_CONTROLLER_OUTPUT(Pack *pack) {
		
		NAV_CONTROLLER_OUTPUT pnav_controller_output = {pack, *pack->bytes};
		
		::on_NAV_CONTROLLER_OUTPUT(pnav_controller_output);
		//printf( "NAV_CONTROLLER_OUTPUT pass\n");
	}
	
	void on_AUTH_KEY(Pack *pack) {
		
		CURSORS(curs);
		AUTH_KEY pauth_key = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_AUTH_KEY(pauth_key);
		//printf( "AUTH_KEY pass\n");
	}
	
	void on_LOCAL_POSITION_NED_COV(Pack *pack) {
		
		CURSORS(curs);
		LOCAL_POSITION_NED_COV plocal_position_ned_cov = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_LOCAL_POSITION_NED_COV(plocal_position_ned_cov);
		//printf( "LOCAL_POSITION_NED_COV pass\n");
	}
	
	void on_ATT_POS_MOCAP(Pack *pack) {
		
		ATT_POS_MOCAP patt_pos_mocap = {pack, *pack->bytes};
		
		::on_ATT_POS_MOCAP(patt_pos_mocap);
		//printf( "ATT_POS_MOCAP pass\n");
	}
	
	void on_STATUSTEXT(Pack *pack) {
		
		CURSORS(curs);
		STATUSTEXT pstatustext = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_STATUSTEXT(pstatustext);
		//printf( "STATUSTEXT pass\n");
	}
	
	void on_PING(Pack *pack) {
		
		PING pping = {pack, *pack->bytes};
		
		::on_PING(pping);
		//printf( "PING pass\n");
	}
	
	void on_CAMERA_CAPTURE_STATUS(Pack *pack) {
		
		CAMERA_CAPTURE_STATUS pcamera_capture_status = {pack, *pack->bytes};
		
		::on_CAMERA_CAPTURE_STATUS(pcamera_capture_status);
		//printf( "CAMERA_CAPTURE_STATUS pass\n");
	}
	
	void on_GLOBAL_POSITION_INT(Pack *pack) {
		
		GLOBAL_POSITION_INT pglobal_position_int = {pack, *pack->bytes};
		
		::on_GLOBAL_POSITION_INT(pglobal_position_int);
		//printf( "GLOBAL_POSITION_INT pass\n");
	}
	
	void on_ENCAPSULATED_DATA(Pack *pack) {
		
		ENCAPSULATED_DATA pencapsulated_data = {pack, *pack->bytes};
		
		::on_ENCAPSULATED_DATA(pencapsulated_data);
		//printf( "ENCAPSULATED_DATA pass\n");
	}
	
	void on_GPS_INPUT(Pack *pack) {
		
		CURSORS(curs);
		GPS_INPUT pgps_input = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_GPS_INPUT(pgps_input);
		//printf( "GPS_INPUT pass\n");
	}
	
	void on_COMMAND_LONG(Pack *pack) {
		
		CURSORS(curs);
		COMMAND_LONG pcommand_long = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_COMMAND_LONG(pcommand_long);
		//printf( "COMMAND_LONG pass\n");
	}
	
	void on_LOG_REQUEST_DATA(Pack *pack) {
		
		LOG_REQUEST_DATA plog_request_data = {pack, *pack->bytes};
		
		::on_LOG_REQUEST_DATA(plog_request_data);
		//printf( "LOG_REQUEST_DATA pass\n");
	}
	
	void on_GPS_RAW_INT(Pack *pack) {
		
		CURSORS(curs);
		GPS_RAW_INT pgps_raw_int = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_GPS_RAW_INT(pgps_raw_int);
		//printf( "GPS_RAW_INT pass\n");
	}
	
	void on_RC_CHANNELS_SCALED(Pack *pack) {
		
		RC_CHANNELS_SCALED prc_channels_scaled = {pack, *pack->bytes};
		
		::on_RC_CHANNELS_SCALED(prc_channels_scaled);
		//printf( "RC_CHANNELS_SCALED pass\n");
	}
	
	void on_CAMERA_SETTINGS(Pack *pack) {
		
		CURSORS(curs);
		CAMERA_SETTINGS pcamera_settings = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_CAMERA_SETTINGS(pcamera_settings);
		//printf( "CAMERA_SETTINGS pass\n");
	}
	
	void on_RAW_PRESSURE(Pack *pack) {
		
		RAW_PRESSURE praw_pressure = {pack, *pack->bytes};
		
		::on_RAW_PRESSURE(praw_pressure);
		//printf( "RAW_PRESSURE pass\n");
	}
	
	void on_NAMED_VALUE_FLOAT(Pack *pack) {
		
		CURSORS(curs);
		NAMED_VALUE_FLOAT pnamed_value_float = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_NAMED_VALUE_FLOAT(pnamed_value_float);
		//printf( "NAMED_VALUE_FLOAT pass\n");
	}
	
	void on_ATTITUDE(Pack *pack) {
		
		ATTITUDE pattitude = {pack, *pack->bytes};
		
		::on_ATTITUDE(pattitude);
		//printf( "ATTITUDE pass\n");
	}
	
	void on_TERRAIN_REQUEST(Pack *pack) {
		
		TERRAIN_REQUEST pterrain_request = {pack, *pack->bytes};
		
		::on_TERRAIN_REQUEST(pterrain_request);
		//printf( "TERRAIN_REQUEST pass\n");
	}
	
	void on_MISSION_WRITE_PARTIAL_LIST(Pack *pack) {
		
		CURSORS(curs);
		MISSION_WRITE_PARTIAL_LIST pmission_write_partial_list = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list);
		//printf( "MISSION_WRITE_PARTIAL_LIST pass\n");
	}
	
	void on_LOG_ERASE(Pack *pack) {
		
		LOG_ERASE plog_erase = {pack, *pack->bytes};
		
		::on_LOG_ERASE(plog_erase);
		//printf( "LOG_ERASE pass\n");
	}
	
	void on_MANUAL_SETPOINT(Pack *pack) {
		
		MANUAL_SETPOINT pmanual_setpoint = {pack, *pack->bytes};
		
		::on_MANUAL_SETPOINT(pmanual_setpoint);
		//printf( "MANUAL_SETPOINT pass\n");
	}
	
	void on_SAFETY_ALLOWED_AREA(Pack *pack) {
		
		CURSORS(curs);
		SAFETY_ALLOWED_AREA psafety_allowed_area = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SAFETY_ALLOWED_AREA(psafety_allowed_area);
		//printf( "SAFETY_ALLOWED_AREA pass\n");
	}
	
	void on_OPTICAL_FLOW_RAD(Pack *pack) {
		
		OPTICAL_FLOW_RAD poptical_flow_rad = {pack, *pack->bytes};
		
		::on_OPTICAL_FLOW_RAD(poptical_flow_rad);
		//printf( "OPTICAL_FLOW_RAD pass\n");
	}
	
	void on_LOG_DATA(Pack *pack) {
		
		LOG_DATA plog_data = {pack, *pack->bytes};
		
		::on_LOG_DATA(plog_data);
		//printf( "LOG_DATA pass\n");
	}
	
	void on_MISSION_CLEAR_ALL(Pack *pack) {
		
		CURSORS(curs);
		MISSION_CLEAR_ALL pmission_clear_all = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_CLEAR_ALL(pmission_clear_all);
		//printf( "MISSION_CLEAR_ALL pass\n");
	}
	
	void on_VICON_POSITION_ESTIMATE(Pack *pack) {
		
		VICON_POSITION_ESTIMATE pvicon_position_estimate = {pack, *pack->bytes};
		
		::on_VICON_POSITION_ESTIMATE(pvicon_position_estimate);
		//printf( "VICON_POSITION_ESTIMATE pass\n");
	}
	
	void on_GPS2_RTK(Pack *pack) {
		
		GPS2_RTK pgps2_rtk = {pack, *pack->bytes};
		
		::on_GPS2_RTK(pgps2_rtk);
		//printf( "GPS2_RTK pass\n");
	}
	
	void on_LOG_REQUEST_LIST(Pack *pack) {
		
		LOG_REQUEST_LIST plog_request_list = {pack, *pack->bytes};
		
		::on_LOG_REQUEST_LIST(plog_request_list);
		//printf( "LOG_REQUEST_LIST pass\n");
	}
	
	void on_SCALED_PRESSURE(Pack *pack) {
		
		SCALED_PRESSURE pscaled_pressure = {pack, *pack->bytes};
		
		::on_SCALED_PRESSURE(pscaled_pressure);
		//printf( "SCALED_PRESSURE pass\n");
	}
	
	void on_MISSION_REQUEST_INT(Pack *pack) {
		
		CURSORS(curs);
		MISSION_REQUEST_INT pmission_request_int = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_REQUEST_INT(pmission_request_int);
		//printf( "MISSION_REQUEST_INT pass\n");
	}
	
	void on_V2_EXTENSION(Pack *pack) {
		
		V2_EXTENSION pv2_extension = {pack, *pack->bytes};
		
		::on_V2_EXTENSION(pv2_extension);
		//printf( "V2_EXTENSION pass\n");
	}
	
	void on_HEARTBEAT(Pack *pack) {
		
		CURSORS(curs);
		HEARTBEAT pheartbeat = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_HEARTBEAT(pheartbeat);
		//printf( "HEARTBEAT pass\n");
	}
	
	void on_PARAM_MAP_RC(Pack *pack) {
		
		CURSORS(curs);
		PARAM_MAP_RC pparam_map_rc = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_PARAM_MAP_RC(pparam_map_rc);
		//printf( "PARAM_MAP_RC pass\n");
	}
	
	void on_POWER_STATUS(Pack *pack) {
		
		CURSORS(curs);
		POWER_STATUS ppower_status = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_POWER_STATUS(ppower_status);
		//printf( "POWER_STATUS pass\n");
	}
	
	void on_TERRAIN_CHECK(Pack *pack) {
		
		TERRAIN_CHECK pterrain_check = {pack, *pack->bytes};
		
		::on_TERRAIN_CHECK(pterrain_check);
		//printf( "TERRAIN_CHECK pass\n");
	}
	
	void on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(Pack *pack) {
		
		LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET plocal_position_ned_system_global_offset = {pack, *pack->bytes};
		
		::on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset);
		//printf( "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET pass\n");
	}
	
	void on_COMMAND_ACK(Pack *pack) {
		
		CURSORS(curs);
		COMMAND_ACK pcommand_ack = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_COMMAND_ACK(pcommand_ack);
		//printf( "COMMAND_ACK pass\n");
	}
	
	void on_DATA_STREAM(Pack *pack) {
		
		DATA_STREAM pdata_stream = {pack, *pack->bytes};
		
		::on_DATA_STREAM(pdata_stream);
		//printf( "DATA_STREAM pass\n");
	}
	
	void on_MISSION_REQUEST(Pack *pack) {
		
		CURSORS(curs);
		MISSION_REQUEST pmission_request = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_REQUEST(pmission_request);
		//printf( "MISSION_REQUEST pass\n");
	}
	
	void on_TERRAIN_REPORT(Pack *pack) {
		
		TERRAIN_REPORT pterrain_report = {pack, *pack->bytes};
		
		::on_TERRAIN_REPORT(pterrain_report);
		//printf( "TERRAIN_REPORT pass\n");
	}
	
	void on_SET_HOME_POSITION(Pack *pack) {
		
		CURSORS(curs);
		SET_HOME_POSITION pset_home_position = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SET_HOME_POSITION(pset_home_position);
		//printf( "SET_HOME_POSITION pass\n");
	}
	
	void on_SwitchModeCommand() {
		::on_SwitchModeCommand();
		printf("SwitchModeCommand pass\n");
	}
	
	void on_HIL_RC_INPUTS_RAW(Pack *pack) {
		
		HIL_RC_INPUTS_RAW phil_rc_inputs_raw = {pack, *pack->bytes};
		
		::on_HIL_RC_INPUTS_RAW(phil_rc_inputs_raw);
		//printf( "HIL_RC_INPUTS_RAW pass\n");
	}
	
	void on_SCALED_IMU3(Pack *pack) {
		
		SCALED_IMU3 pscaled_imu3 = {pack, *pack->bytes};
		
		::on_SCALED_IMU3(pscaled_imu3);
		//printf( "SCALED_IMU3 pass\n");
	}
	
	void on_SET_MODE(Pack *pack) {
		
		CURSORS(curs);
		SET_MODE pset_mode = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SET_MODE(pset_mode);
		//printf( "SET_MODE pass\n");
	}
	
	void on_POSITION_TARGET_GLOBAL_INT(Pack *pack) {
		
		CURSORS(curs);
		POSITION_TARGET_GLOBAL_INT pposition_target_global_int = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_POSITION_TARGET_GLOBAL_INT(pposition_target_global_int);
		//printf( "POSITION_TARGET_GLOBAL_INT pass\n");
	}
	
	void on_FLIGHT_INFORMATION(Pack *pack) {
		
		FLIGHT_INFORMATION pflight_information = {pack, *pack->bytes};
		
		::on_FLIGHT_INFORMATION(pflight_information);
		//printf( "FLIGHT_INFORMATION pass\n");
	}
	
	void on_SIM_STATE(Pack *pack) {
		
		SIM_STATE psim_state = {pack, *pack->bytes};
		
		::on_SIM_STATE(psim_state);
		//printf( "SIM_STATE pass\n");
	}
	
	void on_MISSION_ITEM_REACHED(Pack *pack) {
		
		MISSION_ITEM_REACHED pmission_item_reached = {pack, *pack->bytes};
		
		::on_MISSION_ITEM_REACHED(pmission_item_reached);
		//printf( "MISSION_ITEM_REACHED pass\n");
	}
	
	void on_RC_CHANNELS_RAW(Pack *pack) {
		
		RC_CHANNELS_RAW prc_channels_raw = {pack, *pack->bytes};
		
		::on_RC_CHANNELS_RAW(prc_channels_raw);
		//printf( "RC_CHANNELS_RAW pass\n");
	}
	
	void on_SERVO_OUTPUT_RAW(Pack *pack) {
		
		CURSORS(curs);
		SERVO_OUTPUT_RAW pservo_output_raw = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SERVO_OUTPUT_RAW(pservo_output_raw);
		//printf( "SERVO_OUTPUT_RAW pass\n");
	}
	
	void on_VISION_SPEED_ESTIMATE(Pack *pack) {
		
		VISION_SPEED_ESTIMATE pvision_speed_estimate = {pack, *pack->bytes};
		
		::on_VISION_SPEED_ESTIMATE(pvision_speed_estimate);
		//printf( "VISION_SPEED_ESTIMATE pass\n");
	}
	
	void on_DEBUG_VECT(Pack *pack) {
		
		CURSORS(curs);
		DEBUG_VECT pdebug_vect = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_DEBUG_VECT(pdebug_vect);
		//printf( "DEBUG_VECT pass\n");
	}
	
	void on_LOG_REQUEST_END(Pack *pack) {
		
		LOG_REQUEST_END plog_request_end = {pack, *pack->bytes};
		
		::on_LOG_REQUEST_END(plog_request_end);
		//printf( "LOG_REQUEST_END pass\n");
	}
	
	void on_MISSION_ACK(Pack *pack) {
		
		CURSORS(curs);
		MISSION_ACK pmission_ack = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_ACK(pmission_ack);
		//printf( "MISSION_ACK pass\n");
	}
	
	void on_CHANGE_OPERATOR_CONTROL_ACK(Pack *pack) {
		
		CHANGE_OPERATOR_CONTROL_ACK pchange_operator_control_ack = {pack, *pack->bytes};
		
		::on_CHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack);
		//printf( "CHANGE_OPERATOR_CONTROL_ACK pass\n");
	}
	
	void on_MISSION_CURRENT(Pack *pack) {
		
		MISSION_CURRENT pmission_current = {pack, *pack->bytes};
		
		::on_MISSION_CURRENT(pmission_current);
		//printf( "MISSION_CURRENT pass\n");
	}
	
	void on_SYSTEM_TIME(Pack *pack) {
		
		SYSTEM_TIME psystem_time = {pack, *pack->bytes};
		
		::on_SYSTEM_TIME(psystem_time);
		//printf( "SYSTEM_TIME pass\n");
	}
	
	void on_CAMERA_TRIGGER(Pack *pack) {
		
		CAMERA_TRIGGER pcamera_trigger = {pack, *pack->bytes};
		
		::on_CAMERA_TRIGGER(pcamera_trigger);
		//printf( "CAMERA_TRIGGER pass\n");
	}
	
	void on_VISION_POSITION_ESTIMATE(Pack *pack) {
		
		VISION_POSITION_ESTIMATE pvision_position_estimate = {pack, *pack->bytes};
		
		::on_VISION_POSITION_ESTIMATE(pvision_position_estimate);
		//printf( "VISION_POSITION_ESTIMATE pass\n");
	}
	
	void on_MANUAL_CONTROL(Pack *pack) {
		
		MANUAL_CONTROL pmanual_control = {pack, *pack->bytes};
		
		::on_MANUAL_CONTROL(pmanual_control);
		//printf( "MANUAL_CONTROL pass\n");
	}
	
	void on_RC_CHANNELS(Pack *pack) {
		
		RC_CHANNELS prc_channels = {pack, *pack->bytes};
		
		::on_RC_CHANNELS(prc_channels);
		//printf( "RC_CHANNELS pass\n");
	}
	
	void on_PARAM_VALUE(Pack *pack) {
		
		CURSORS(curs);
		PARAM_VALUE pparam_value = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_PARAM_VALUE(pparam_value);
		//printf( "PARAM_VALUE pass\n");
	}
	
	void on_BATTERY_STATUS(Pack *pack) {
		
		CURSORS(curs);
		BATTERY_STATUS pbattery_status = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_BATTERY_STATUS(pbattery_status);
		//printf( "BATTERY_STATUS pass\n");
	}
	
	void on_SET_POSITION_TARGET_LOCAL_NED(Pack *pack) {
		
		CURSORS(curs);
		SET_POSITION_TARGET_LOCAL_NED pset_position_target_local_ned = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned);
		//printf( "SET_POSITION_TARGET_LOCAL_NED pass\n");
	}
	
	void on_SERIAL_CONTROL(Pack *pack) {
		
		CURSORS(curs);
		SERIAL_CONTROL pserial_control = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SERIAL_CONTROL(pserial_control);
		//printf( "SERIAL_CONTROL pass\n");
	}
	
	void on_SET_GPS_GLOBAL_ORIGIN(Pack *pack) {
		
		CURSORS(curs);
		SET_GPS_GLOBAL_ORIGIN pset_gps_global_origin = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin);
		//printf( "SET_GPS_GLOBAL_ORIGIN pass\n");
	}
	
	void on_AUTOPILOT_VERSION(Pack *pack) {
		
		CURSORS(curs);
		AUTOPILOT_VERSION pautopilot_version = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_AUTOPILOT_VERSION(pautopilot_version);
		//printf( "AUTOPILOT_VERSION pass\n");
	}
	
	void on_MISSION_REQUEST_LIST(Pack *pack) {
		
		CURSORS(curs);
		MISSION_REQUEST_LIST pmission_request_list = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_REQUEST_LIST(pmission_request_list);
		//printf( "MISSION_REQUEST_LIST pass\n");
	}
	
	void on_PLAY_TUNE(Pack *pack) {
		
		CURSORS(curs);
		PLAY_TUNE pplay_tune = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_PLAY_TUNE(pplay_tune);
		//printf( "PLAY_TUNE pass\n");
	}
	
	void on_SCALED_PRESSURE3(Pack *pack) {
		
		SCALED_PRESSURE3 pscaled_pressure3 = {pack, *pack->bytes};
		
		::on_SCALED_PRESSURE3(pscaled_pressure3);
		//printf( "SCALED_PRESSURE3 pass\n");
	}
	
	void on_MISSION_REQUEST_PARTIAL_LIST(Pack *pack) {
		
		CURSORS(curs);
		MISSION_REQUEST_PARTIAL_LIST pmission_request_partial_list = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list);
		//printf( "MISSION_REQUEST_PARTIAL_LIST pass\n");
	}
	
	void on_LOCAL_POSITION_NED(Pack *pack) {
		
		LOCAL_POSITION_NED plocal_position_ned = {pack, *pack->bytes};
		
		::on_LOCAL_POSITION_NED(plocal_position_ned);
		//printf( "LOCAL_POSITION_NED pass\n");
	}
	
	void on_DATA_TRANSMISSION_HANDSHAKE(Pack *pack) {
		
		DATA_TRANSMISSION_HANDSHAKE pdata_transmission_handshake = {pack, *pack->bytes};
		
		::on_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake);
		//printf( "DATA_TRANSMISSION_HANDSHAKE pass\n");
	}
	
	void on_GPS_GLOBAL_ORIGIN(Pack *pack) {
		
		CURSORS(curs);
		GPS_GLOBAL_ORIGIN pgps_global_origin = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_GPS_GLOBAL_ORIGIN(pgps_global_origin);
		//printf( "GPS_GLOBAL_ORIGIN pass\n");
	}
	
	void on_SCALED_IMU2(Pack *pack) {
		
		SCALED_IMU2 pscaled_imu2 = {pack, *pack->bytes};
		
		::on_SCALED_IMU2(pscaled_imu2);
		//printf( "SCALED_IMU2 pass\n");
	}
	
	void on_ATTITUDE_QUATERNION(Pack *pack) {
		
		ATTITUDE_QUATERNION pattitude_quaternion = {pack, *pack->bytes};
		
		::on_ATTITUDE_QUATERNION(pattitude_quaternion);
		//printf( "ATTITUDE_QUATERNION pass\n");
	}
	
	void on_HIL_ACTUATOR_CONTROLS(Pack *pack) {
		
		CURSORS(curs);
		HIL_ACTUATOR_CONTROLS phil_actuator_controls = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_HIL_ACTUATOR_CONTROLS(phil_actuator_controls);
		//printf( "HIL_ACTUATOR_CONTROLS pass\n");
	}
	
	void on_POSITION_TARGET_LOCAL_NED(Pack *pack) {
		
		CURSORS(curs);
		POSITION_TARGET_LOCAL_NED pposition_target_local_ned = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_POSITION_TARGET_LOCAL_NED(pposition_target_local_ned);
		//printf( "POSITION_TARGET_LOCAL_NED pass\n");
	}
	
	void on_DISTANCE_SENSOR(Pack *pack) {
		
		CURSORS(curs);
		DISTANCE_SENSOR pdistance_sensor = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_DISTANCE_SENSOR(pdistance_sensor);
		//printf( "DISTANCE_SENSOR pass\n");
	}
	
	void on_HIL_OPTICAL_FLOW(Pack *pack) {
		
		HIL_OPTICAL_FLOW phil_optical_flow = {pack, *pack->bytes};
		
		::on_HIL_OPTICAL_FLOW(phil_optical_flow);
		//printf( "HIL_OPTICAL_FLOW pass\n");
	}
	
	void on_SCALED_PRESSURE2(Pack *pack) {
		
		SCALED_PRESSURE2 pscaled_pressure2 = {pack, *pack->bytes};
		
		::on_SCALED_PRESSURE2(pscaled_pressure2);
		//printf( "SCALED_PRESSURE2 pass\n");
	}
	
	void on_WIND_COV(Pack *pack) {
		
		WIND_COV pwind_cov = {pack, *pack->bytes};
		
		::on_WIND_COV(pwind_cov);
		//printf( "WIND_COV pass\n");
	}
	
	void on_CHANGE_OPERATOR_CONTROL(Pack *pack) {
		
		CURSORS(curs);
		CHANGE_OPERATOR_CONTROL pchange_operator_control = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_CHANGE_OPERATOR_CONTROL(pchange_operator_control);
		//printf( "CHANGE_OPERATOR_CONTROL pass\n");
	}
	
	void on_SYS_STATUS(Pack *pack) {
		
		CURSORS(curs);
		SYS_STATUS psys_status = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_SYS_STATUS(psys_status);
		//printf( "SYS_STATUS pass\n");
	}
	
	void on_MISSION_ITEM(Pack *pack) {
		
		CURSORS(curs);
		MISSION_ITEM pmission_item = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_ITEM(pmission_item);
		//printf( "MISSION_ITEM pass\n");
	}
	
	void on_RAW_IMU(Pack *pack) {
		
		RAW_IMU praw_imu = {pack, *pack->bytes};
		
		::on_RAW_IMU(praw_imu);
		//printf( "RAW_IMU pass\n");
	}
	
	void on_COMMAND_INT(Pack *pack) {
		
		CURSORS(curs);
		COMMAND_INT pcommand_int = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_COMMAND_INT(pcommand_int);
		//printf( "COMMAND_INT pass\n");
	}
	
	void on_OPTICAL_FLOW(Pack *pack) {
		
		CURSORS(curs);
		OPTICAL_FLOW poptical_flow = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_OPTICAL_FLOW(poptical_flow);
		//printf( "OPTICAL_FLOW pass\n");
	}
	
	void on_MISSION_ITEM_INT(Pack *pack) {
		
		CURSORS(curs);
		MISSION_ITEM_INT pmission_item_int = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_MISSION_ITEM_INT(pmission_item_int);
		//printf( "MISSION_ITEM_INT pass\n");
	}
	
	void on_HIGHRES_IMU(Pack *pack) {
		
		HIGHRES_IMU phighres_imu = {pack, *pack->bytes};
		
		::on_HIGHRES_IMU(phighres_imu);
		//printf( "HIGHRES_IMU pass\n");
	}
	
	void on_EXTENDED_SYS_STATE(Pack *pack) {
		
		CURSORS(curs);
		EXTENDED_SYS_STATE pextended_sys_state = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_EXTENDED_SYS_STATE(pextended_sys_state);
		//printf( "EXTENDED_SYS_STATE pass\n");
	}
	
	void on_GPS_INJECT_DATA(Pack *pack) {
		
		GPS_INJECT_DATA pgps_inject_data = {pack, *pack->bytes};
		
		::on_GPS_INJECT_DATA(pgps_inject_data);
		//printf( "GPS_INJECT_DATA pass\n");
	}
	
	void on_ATTITUDE_QUATERNION_COV(Pack *pack) {
		
		ATTITUDE_QUATERNION_COV pattitude_quaternion_cov = {pack, *pack->bytes};
		
		::on_ATTITUDE_QUATERNION_COV(pattitude_quaternion_cov);
		//printf( "ATTITUDE_QUATERNION_COV pass\n");
	}
	
	void on_NAMED_VALUE_INT(Pack *pack) {
		
		CURSORS(curs);
		NAMED_VALUE_INT pnamed_value_int = {*org::unirail::utils::wrap_pack(pack, curs)};
		
		::on_NAMED_VALUE_INT(pnamed_value_int);
		//printf( "NAMED_VALUE_INT pass\n");
	}
	
	void on_RADIO_STATUS(Pack *pack) {
		
		RADIO_STATUS pradio_status = {pack, *pack->bytes};
		
		::on_RADIO_STATUS(pradio_status);
		//printf( "RADIO_STATUS pass\n");
	}
	
	void on_GPS_RTCM_DATA(Pack *pack) {
		
		GPS_RTCM_DATA pgps_rtcm_data = {pack, *pack->bytes};
		
		::on_GPS_RTCM_DATA(pgps_rtcm_data);
		//printf( "GPS_RTCM_DATA pass\n");
	}
	
	void on_GLOBAL_VISION_POSITION_ESTIMATE(Pack *pack) {
		
		GLOBAL_VISION_POSITION_ESTIMATE pglobal_vision_position_estimate = {pack, *pack->bytes};
		
		::on_GLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate);
		//printf( "GLOBAL_VISION_POSITION_ESTIMATE pass\n");
	}
	
	void on_FILE_TRANSFER_PROTOCOL(Pack *pack) {
		
		FILE_TRANSFER_PROTOCOL pfile_transfer_protocol = {pack, *pack->bytes};
		
		::on_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol);
		//printf( "FILE_TRANSFER_PROTOCOL pass\n");
	}
	
	static CommunicationChannel_demo *from(Receiver *receiver) {
		return reinterpret_cast<CommunicationChannel_demo *>(reinterpret_cast<uint8_t *>(receiver) - offsetof(CommunicationChannel_demo, channel) - offsetof(CommunicationChannel<CommunicationChannel_demo>, receiver));
	}
	
	RBUF_INIT(Pack*, 5) sending_out_packs;
	
	static const Pack *pull_sending_pack(Transmitter *transmitter) {
		auto channel = reinterpret_cast<CommunicationChannel_demo *>(reinterpret_cast<uint8_t *>(transmitter) - offsetof(CommunicationChannel_demo, channel) - offsetof(CommunicationChannel<CommunicationChannel_demo>, transmitter));
		if (RBUF_ISEMPTY(channel->sending_out_packs)) return nullptr;
		return RBUF_GET(channel->sending_out_packs);
	}
	
	static bool push_sending_pack(Transmitter *transmitter, Pack *pack) {
		auto channel = reinterpret_cast<CommunicationChannel_demo *>(reinterpret_cast<uint8_t *>(transmitter) - offsetof(CommunicationChannel_demo, channel) - offsetof(CommunicationChannel<CommunicationChannel_demo>, transmitter));
		
		if (RBUF_ISFULL(channel->sending_out_packs)) return false;
		RBUF_PUT(channel->sending_out_packs, pack)
		return true;
	}
	
};


int main() {
	CURSORS(cur);
	uint8_t buffer[512];
	int     bytes_out;
	
	
	CommunicationChannel_demo _CommunicationChannel{};
	
	
	{
		
		auto pfollow_target = _CommunicationChannel.channel.NEW.FOLLOW_TARGET();
		fill(pfollow_target);
		
		if (_CommunicationChannel.channel.send(pfollow_target)) {
			free_pack(pfollow_target.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto padsb_vehicle = _CommunicationChannel.channel.NEW.ADSB_VEHICLE(cur);
		fill(padsb_vehicle);
		if (!_CommunicationChannel.channel.send(padsb_vehicle)) {
			free_pack(padsb_vehicle.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmessage_interval = _CommunicationChannel.channel.NEW.MESSAGE_INTERVAL();
		fill(pmessage_interval);
		
		if (_CommunicationChannel.channel.send(pmessage_interval)) {
			free_pack(pmessage_interval.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pekf_status_report = _CommunicationChannel.channel.NEW.EKF_STATUS_REPORT(cur);
		fill(pekf_status_report);
		if (!_CommunicationChannel.channel.send(pekf_status_report)) {
			free_pack(pekf_status_report.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pestimator_status = _CommunicationChannel.channel.NEW.ESTIMATOR_STATUS(cur);
		fill(pestimator_status);
		if (!_CommunicationChannel.channel.send(pestimator_status)) {
			free_pack(pestimator_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto phwstatus = _CommunicationChannel.channel.NEW.HWSTATUS();
		fill(phwstatus);
		
		if (_CommunicationChannel.channel.send(phwstatus)) {
			free_pack(phwstatus.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto ptimesync = _CommunicationChannel.channel.NEW.TIMESYNC();
		fill(ptimesync);
		
		if (_CommunicationChannel.channel.send(ptimesync)) {
			free_pack(ptimesync.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pparam_ext_request_list = _CommunicationChannel.channel.NEW.PARAM_EXT_REQUEST_LIST();
		fill(pparam_ext_request_list);
		
		if (_CommunicationChannel.channel.send(pparam_ext_request_list)) {
			free_pack(pparam_ext_request_list.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pbutton_change = _CommunicationChannel.channel.NEW.BUTTON_CHANGE();
		fill(pbutton_change);
		
		if (_CommunicationChannel.channel.send(pbutton_change)) {
			free_pack(pbutton_change.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto puavcan_node_status = _CommunicationChannel.channel.NEW.UAVCAN_NODE_STATUS(cur);
		fill(puavcan_node_status);
		if (!_CommunicationChannel.channel.send(puavcan_node_status)) {
			free_pack(puavcan_node_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcollision = _CommunicationChannel.channel.NEW.COLLISION(cur);
		fill(pcollision);
		if (!_CommunicationChannel.channel.send(pcollision)) {
			free_pack(pcollision.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgimbal_torque_cmd_report = _CommunicationChannel.channel.NEW.GIMBAL_TORQUE_CMD_REPORT();
		fill(pgimbal_torque_cmd_report);
		
		if (_CommunicationChannel.channel.send(pgimbal_torque_cmd_report)) {
			free_pack(pgimbal_torque_cmd_report.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto paltitude = _CommunicationChannel.channel.NEW.ALTITUDE();
		fill(paltitude);
		
		if (_CommunicationChannel.channel.send(paltitude)) {
			free_pack(paltitude.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto phil_state_quaternion = _CommunicationChannel.channel.NEW.HIL_STATE_QUATERNION();
		fill(phil_state_quaternion);
		
		if (_CommunicationChannel.channel.send(phil_state_quaternion)) {
			free_pack(phil_state_quaternion.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto psensor_offsets = _CommunicationChannel.channel.NEW.SENSOR_OFFSETS();
		fill(psensor_offsets);
		
		if (_CommunicationChannel.channel.send(psensor_offsets)) {
			free_pack(psensor_offsets.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pstorage_information = _CommunicationChannel.channel.NEW.STORAGE_INFORMATION();
		fill(pstorage_information);
		
		if (_CommunicationChannel.channel.send(pstorage_information)) {
			free_pack(pstorage_information.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcamera_information = _CommunicationChannel.channel.NEW.CAMERA_INFORMATION(cur);
		fill(pcamera_information);
		if (!_CommunicationChannel.channel.send(pcamera_information)) {
			free_pack(pcamera_information.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdevice_op_write_reply = _CommunicationChannel.channel.NEW.DEVICE_OP_WRITE_REPLY();
		fill(pdevice_op_write_reply);
		
		if (_CommunicationChannel.channel.send(pdevice_op_write_reply)) {
			free_pack(pdevice_op_write_reply.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pterrain_data = _CommunicationChannel.channel.NEW.TERRAIN_DATA();
		fill(pterrain_data);
		
		if (_CommunicationChannel.channel.send(pterrain_data)) {
			free_pack(pterrain_data.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgimbal_control = _CommunicationChannel.channel.NEW.GIMBAL_CONTROL();
		fill(pgimbal_control);
		
		if (_CommunicationChannel.channel.send(pgimbal_control)) {
			free_pack(pgimbal_control.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pvideo_stream_information = _CommunicationChannel.channel.NEW.VIDEO_STREAM_INFORMATION(cur);
		fill(pvideo_stream_information);
		if (!_CommunicationChannel.channel.send(pvideo_stream_information)) {
			free_pack(pvideo_stream_information.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pahrs = _CommunicationChannel.channel.NEW.AHRS();
		fill(pahrs);
		
		if (_CommunicationChannel.channel.send(pahrs)) {
			free_pack(pahrs.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdebug = _CommunicationChannel.channel.NEW.DEBUG();
		fill(pdebug);
		
		if (_CommunicationChannel.channel.send(pdebug)) {
			free_pack(pdebug.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcamera_image_captured = _CommunicationChannel.channel.NEW.CAMERA_IMAGE_CAPTURED(cur);
		fill(pcamera_image_captured);
		if (!_CommunicationChannel.channel.send(pcamera_image_captured)) {
			free_pack(pcamera_image_captured.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plog_entry = _CommunicationChannel.channel.NEW.LOG_ENTRY();
		fill(plog_entry);
		
		if (_CommunicationChannel.channel.send(plog_entry)) {
			free_pack(plog_entry.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pactuator_control_target = _CommunicationChannel.channel.NEW.ACTUATOR_CONTROL_TARGET();
		fill(pactuator_control_target);
		
		if (_CommunicationChannel.channel.send(pactuator_control_target)) {
			free_pack(pactuator_control_target.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto phigh_latency = _CommunicationChannel.channel.NEW.HIGH_LATENCY(cur);
		fill(phigh_latency);
		if (!_CommunicationChannel.channel.send(phigh_latency)) {
			free_pack(phigh_latency.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto phome_position = _CommunicationChannel.channel.NEW.HOME_POSITION(cur);
		fill(phome_position);
		if (!_CommunicationChannel.channel.send(phome_position)) {
			free_pack(phome_position.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pfence_status = _CommunicationChannel.channel.NEW.FENCE_STATUS(cur);
		fill(pfence_status);
		if (!_CommunicationChannel.channel.send(pfence_status)) {
			free_pack(pfence_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto premote_log_block_status = _CommunicationChannel.channel.NEW.REMOTE_LOG_BLOCK_STATUS(cur);
		fill(premote_log_block_status);
		if (!_CommunicationChannel.channel.send(premote_log_block_status)) {
			free_pack(premote_log_block_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pobstacle_distance = _CommunicationChannel.channel.NEW.OBSTACLE_DISTANCE(cur);
		fill(pobstacle_distance);
		if (!_CommunicationChannel.channel.send(pobstacle_distance)) {
			free_pack(pobstacle_distance.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgps2_raw = _CommunicationChannel.channel.NEW.GPS2_RAW(cur);
		fill(pgps2_raw);
		if (!_CommunicationChannel.channel.send(pgps2_raw)) {
			free_pack(pgps2_raw.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmemory_vect = _CommunicationChannel.channel.NEW.MEMORY_VECT();
		fill(pmemory_vect);
		
		if (_CommunicationChannel.channel.send(pmemory_vect)) {
			free_pack(pmemory_vect.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pparam_ext_request_read = _CommunicationChannel.channel.NEW.PARAM_EXT_REQUEST_READ(cur);
		fill(pparam_ext_request_read);
		if (!_CommunicationChannel.channel.send(pparam_ext_request_read)) {
			free_pack(pparam_ext_request_read.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto phil_sensor = _CommunicationChannel.channel.NEW.HIL_SENSOR();
		fill(phil_sensor);
		
		if (_CommunicationChannel.channel.send(phil_sensor)) {
			free_pack(phil_sensor.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto psetup_signing = _CommunicationChannel.channel.NEW.SETUP_SIGNING();
		fill(psetup_signing);
		
		if (_CommunicationChannel.channel.send(psetup_signing)) {
			free_pack(psetup_signing.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgps_rtk = _CommunicationChannel.channel.NEW.GPS_RTK();
		fill(pgps_rtk);
		
		if (_CommunicationChannel.channel.send(pgps_rtk)) {
			free_pack(pgps_rtk.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto puavionix_adsb_out_cfg = _CommunicationChannel.channel.NEW.UAVIONIX_ADSB_OUT_CFG(cur);
		fill(puavionix_adsb_out_cfg);
		if (!_CommunicationChannel.channel.send(puavionix_adsb_out_cfg)) {
			free_pack(puavionix_adsb_out_cfg.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto planding_target = _CommunicationChannel.channel.NEW.LANDING_TARGET(cur);
		fill(planding_target);
		if (!_CommunicationChannel.channel.send(planding_target)) {
			free_pack(planding_target.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pset_actuator_control_target = _CommunicationChannel.channel.NEW.SET_ACTUATOR_CONTROL_TARGET();
		fill(pset_actuator_control_target);
		
		if (_CommunicationChannel.channel.send(pset_actuator_control_target)) {
			free_pack(pset_actuator_control_target.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcontrol_system_state = _CommunicationChannel.channel.NEW.CONTROL_SYSTEM_STATE();
		fill(pcontrol_system_state);
		
		if (_CommunicationChannel.channel.send(pcontrol_system_state)) {
			free_pack(pcontrol_system_state.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdata32 = _CommunicationChannel.channel.NEW.DATA32();
		fill(pdata32);
		
		if (_CommunicationChannel.channel.send(pdata32)) {
			free_pack(pdata32.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pping33 = _CommunicationChannel.channel.NEW.PING33(cur);
		fill(pping33);
		if (!_CommunicationChannel.channel.send(pping33)) {
			free_pack(pping33.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto prally_point = _CommunicationChannel.channel.NEW.RALLY_POINT(cur);
		fill(prally_point);
		if (!_CommunicationChannel.channel.send(prally_point)) {
			free_pack(prally_point.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto padap_tuning = _CommunicationChannel.channel.NEW.ADAP_TUNING(cur);
		fill(padap_tuning);
		if (!_CommunicationChannel.channel.send(padap_tuning)) {
			free_pack(padap_tuning.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pvibration = _CommunicationChannel.channel.NEW.VIBRATION();
		fill(pvibration);
		
		if (_CommunicationChannel.channel.send(pvibration)) {
			free_pack(pvibration.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pparam_ext_value = _CommunicationChannel.channel.NEW.PARAM_EXT_VALUE(cur);
		fill(pparam_ext_value);
		if (!_CommunicationChannel.channel.send(pparam_ext_value)) {
			free_pack(pparam_ext_value.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pbattery2 = _CommunicationChannel.channel.NEW.BATTERY2();
		fill(pbattery2);
		
		if (_CommunicationChannel.channel.send(pbattery2)) {
			free_pack(pbattery2.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plimits_status = _CommunicationChannel.channel.NEW.LIMITS_STATUS(cur);
		fill(plimits_status);
		if (!_CommunicationChannel.channel.send(plimits_status)) {
			free_pack(plimits_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcamera_feedback = _CommunicationChannel.channel.NEW.CAMERA_FEEDBACK(cur);
		fill(pcamera_feedback);
		if (!_CommunicationChannel.channel.send(pcamera_feedback)) {
			free_pack(pcamera_feedback.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto phil_gps = _CommunicationChannel.channel.NEW.HIL_GPS();
		fill(phil_gps);
		
		if (_CommunicationChannel.channel.send(phil_gps)) {
			free_pack(phil_gps.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pfence_fetch_point = _CommunicationChannel.channel.NEW.FENCE_FETCH_POINT();
		fill(pfence_fetch_point);
		
		if (_CommunicationChannel.channel.send(pfence_fetch_point)) {
			free_pack(pfence_fetch_point.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pradio = _CommunicationChannel.channel.NEW.RADIO();
		fill(pradio);
		
		if (_CommunicationChannel.channel.send(pradio)) {
			free_pack(pradio.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pairspeed_autocal = _CommunicationChannel.channel.NEW.AIRSPEED_AUTOCAL();
		fill(pairspeed_autocal);
		
		if (_CommunicationChannel.channel.send(pairspeed_autocal)) {
			free_pack(pairspeed_autocal.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto patt_pos_mocap = _CommunicationChannel.channel.NEW.ATT_POS_MOCAP();
		fill(patt_pos_mocap);
		
		if (_CommunicationChannel.channel.send(patt_pos_mocap)) {
			free_pack(patt_pos_mocap.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pstatustext = _CommunicationChannel.channel.NEW.STATUSTEXT(cur);
		fill(pstatustext);
		if (!_CommunicationChannel.channel.send(pstatustext)) {
			free_pack(pstatustext.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgopro_get_request = _CommunicationChannel.channel.NEW.GOPRO_GET_REQUEST(cur);
		fill(pgopro_get_request);
		if (!_CommunicationChannel.channel.send(pgopro_get_request)) {
			free_pack(pgopro_get_request.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcamera_capture_status = _CommunicationChannel.channel.NEW.CAMERA_CAPTURE_STATUS();
		fill(pcamera_capture_status);
		
		if (_CommunicationChannel.channel.send(pcamera_capture_status)) {
			free_pack(pcamera_capture_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pencapsulated_data = _CommunicationChannel.channel.NEW.ENCAPSULATED_DATA();
		fill(pencapsulated_data);
		
		if (_CommunicationChannel.channel.send(pencapsulated_data)) {
			free_pack(pencapsulated_data.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgps_input = _CommunicationChannel.channel.NEW.GPS_INPUT(cur);
		fill(pgps_input);
		if (!_CommunicationChannel.channel.send(pgps_input)) {
			free_pack(pgps_input.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcompassmot_status = _CommunicationChannel.channel.NEW.COMPASSMOT_STATUS();
		fill(pcompassmot_status);
		
		if (_CommunicationChannel.channel.send(pcompassmot_status)) {
			free_pack(pcompassmot_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plog_request_data = _CommunicationChannel.channel.NEW.LOG_REQUEST_DATA();
		fill(plog_request_data);
		
		if (_CommunicationChannel.channel.send(plog_request_data)) {
			free_pack(plog_request_data.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcamera_status = _CommunicationChannel.channel.NEW.CAMERA_STATUS(cur);
		fill(pcamera_status);
		if (!_CommunicationChannel.channel.send(pcamera_status)) {
			free_pack(pcamera_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcamera_settings = _CommunicationChannel.channel.NEW.CAMERA_SETTINGS(cur);
		fill(pcamera_settings);
		if (!_CommunicationChannel.channel.send(pcamera_settings)) {
			free_pack(pcamera_settings.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdevice_op_read_reply = _CommunicationChannel.channel.NEW.DEVICE_OP_READ_REPLY();
		fill(pdevice_op_read_reply);
		
		if (_CommunicationChannel.channel.send(pdevice_op_read_reply)) {
			free_pack(pdevice_op_read_reply.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdigicam_control = _CommunicationChannel.channel.NEW.DIGICAM_CONTROL();
		fill(pdigicam_control);
		
		if (_CommunicationChannel.channel.send(pdigicam_control)) {
			free_pack(pdigicam_control.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pnamed_value_float = _CommunicationChannel.channel.NEW.NAMED_VALUE_FLOAT(cur);
		fill(pnamed_value_float);
		if (!_CommunicationChannel.channel.send(pnamed_value_float)) {
			free_pack(pnamed_value_float.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgopro_heartbeat = _CommunicationChannel.channel.NEW.GOPRO_HEARTBEAT(cur);
		fill(pgopro_heartbeat);
		if (!_CommunicationChannel.channel.send(pgopro_heartbeat)) {
			free_pack(pgopro_heartbeat.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pahrs2 = _CommunicationChannel.channel.NEW.AHRS2();
		fill(pahrs2);
		
		if (_CommunicationChannel.channel.send(pahrs2)) {
			free_pack(pahrs2.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plog_erase = _CommunicationChannel.channel.NEW.LOG_ERASE();
		fill(plog_erase);
		
		if (_CommunicationChannel.channel.send(plog_erase)) {
			free_pack(plog_erase.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pterrain_request = _CommunicationChannel.channel.NEW.TERRAIN_REQUEST();
		fill(pterrain_request);
		
		if (_CommunicationChannel.channel.send(pterrain_request)) {
			free_pack(pterrain_request.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmount_status = _CommunicationChannel.channel.NEW.MOUNT_STATUS();
		fill(pmount_status);
		
		if (_CommunicationChannel.channel.send(pmount_status)) {
			free_pack(pmount_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto ppid_tuning = _CommunicationChannel.channel.NEW.PID_TUNING(cur);
		fill(ppid_tuning);
		if (!_CommunicationChannel.channel.send(ppid_tuning)) {
			free_pack(ppid_tuning.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto poptical_flow_rad = _CommunicationChannel.channel.NEW.OPTICAL_FLOW_RAD();
		fill(poptical_flow_rad);
		
		if (_CommunicationChannel.channel.send(poptical_flow_rad)) {
			free_pack(poptical_flow_rad.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plog_data = _CommunicationChannel.channel.NEW.LOG_DATA();
		fill(plog_data);
		
		if (_CommunicationChannel.channel.send(plog_data)) {
			free_pack(plog_data.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pahrs3 = _CommunicationChannel.channel.NEW.AHRS3();
		fill(pahrs3);
		
		if (_CommunicationChannel.channel.send(pahrs3)) {
			free_pack(pahrs3.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pvicon_position_estimate = _CommunicationChannel.channel.NEW.VICON_POSITION_ESTIMATE();
		fill(pvicon_position_estimate);
		
		if (_CommunicationChannel.channel.send(pvicon_position_estimate)) {
			free_pack(pvicon_position_estimate.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgps2_rtk = _CommunicationChannel.channel.NEW.GPS2_RTK();
		fill(pgps2_rtk);
		
		if (_CommunicationChannel.channel.send(pgps2_rtk)) {
			free_pack(pgps2_rtk.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmag_cal_report = _CommunicationChannel.channel.NEW.MAG_CAL_REPORT(cur);
		fill(pmag_cal_report);
		if (!_CommunicationChannel.channel.send(pmag_cal_report)) {
			free_pack(pmag_cal_report.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plog_request_list = _CommunicationChannel.channel.NEW.LOG_REQUEST_LIST();
		fill(plog_request_list);
		
		if (_CommunicationChannel.channel.send(plog_request_list)) {
			free_pack(plog_request_list.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmount_configure = _CommunicationChannel.channel.NEW.MOUNT_CONFIGURE(cur);
		fill(pmount_configure);
		if (!_CommunicationChannel.channel.send(pmount_configure)) {
			free_pack(pmount_configure.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pv2_extension = _CommunicationChannel.channel.NEW.V2_EXTENSION();
		fill(pv2_extension);
		
		if (_CommunicationChannel.channel.send(pv2_extension)) {
			free_pack(pv2_extension.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto ppower_status = _CommunicationChannel.channel.NEW.POWER_STATUS(cur);
		fill(ppower_status);
		if (!_CommunicationChannel.channel.send(ppower_status)) {
			free_pack(ppower_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto premote_log_data_block = _CommunicationChannel.channel.NEW.REMOTE_LOG_DATA_BLOCK(cur);
		fill(premote_log_data_block);
		if (!_CommunicationChannel.channel.send(premote_log_data_block)) {
			free_pack(premote_log_data_block.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plogging_data_acked = _CommunicationChannel.channel.NEW.LOGGING_DATA_ACKED();
		fill(plogging_data_acked);
		
		if (_CommunicationChannel.channel.send(plogging_data_acked)) {
			free_pack(plogging_data_acked.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pterrain_check = _CommunicationChannel.channel.NEW.TERRAIN_CHECK();
		fill(pterrain_check);
		
		if (_CommunicationChannel.channel.send(pterrain_check)) {
			free_pack(pterrain_check.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pterrain_report = _CommunicationChannel.channel.NEW.TERRAIN_REPORT();
		fill(pterrain_report);
		
		if (_CommunicationChannel.channel.send(pterrain_report)) {
			free_pack(pterrain_report.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pset_home_position = _CommunicationChannel.channel.NEW.SET_HOME_POSITION(cur);
		fill(pset_home_position);
		if (!_CommunicationChannel.channel.send(pset_home_position)) {
			free_pack(pset_home_position.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		if (!_CommunicationChannel.channel.send_SwitchModeCommand())
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		
		auto pscaled_imu3 = _CommunicationChannel.channel.NEW.SCALED_IMU3();
		fill(pscaled_imu3);
		
		if (_CommunicationChannel.channel.send(pscaled_imu3)) {
			free_pack(pscaled_imu3.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmount_control = _CommunicationChannel.channel.NEW.MOUNT_CONTROL();
		fill(pmount_control);
		
		if (_CommunicationChannel.channel.send(pmount_control)) {
			free_pack(pmount_control.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pled_control = _CommunicationChannel.channel.NEW.LED_CONTROL();
		fill(pled_control);
		
		if (_CommunicationChannel.channel.send(pled_control)) {
			free_pack(pled_control.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto psim_state = _CommunicationChannel.channel.NEW.SIM_STATE();
		fill(psim_state);
		
		if (_CommunicationChannel.channel.send(psim_state)) {
			free_pack(psim_state.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pwifi_config_ap = _CommunicationChannel.channel.NEW.WIFI_CONFIG_AP(cur);
		fill(pwifi_config_ap);
		if (!_CommunicationChannel.channel.send(pwifi_config_ap)) {
			free_pack(pwifi_config_ap.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdata96 = _CommunicationChannel.channel.NEW.DATA96();
		fill(pdata96);
		
		if (_CommunicationChannel.channel.send(pdata96)) {
			free_pack(pdata96.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pflight_information = _CommunicationChannel.channel.NEW.FLIGHT_INFORMATION();
		fill(pflight_information);
		
		if (_CommunicationChannel.channel.send(pflight_information)) {
			free_pack(pflight_information.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmeminfo = _CommunicationChannel.channel.NEW.MEMINFO(cur);
		fill(pmeminfo);
		if (!_CommunicationChannel.channel.send(pmeminfo)) {
			free_pack(pmeminfo.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plogging_ack = _CommunicationChannel.channel.NEW.LOGGING_ACK();
		fill(plogging_ack);
		
		if (_CommunicationChannel.channel.send(plogging_ack)) {
			free_pack(plogging_ack.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pvision_speed_estimate = _CommunicationChannel.channel.NEW.VISION_SPEED_ESTIMATE();
		fill(pvision_speed_estimate);
		
		if (_CommunicationChannel.channel.send(pvision_speed_estimate)) {
			free_pack(pvision_speed_estimate.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdebug_vect = _CommunicationChannel.channel.NEW.DEBUG_VECT(cur);
		fill(pdebug_vect);
		if (!_CommunicationChannel.channel.send(pdebug_vect)) {
			free_pack(pdebug_vect.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pcamera_trigger = _CommunicationChannel.channel.NEW.CAMERA_TRIGGER();
		fill(pcamera_trigger);
		
		if (_CommunicationChannel.channel.send(pcamera_trigger)) {
			free_pack(pcamera_trigger.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plog_request_end = _CommunicationChannel.channel.NEW.LOG_REQUEST_END();
		fill(plog_request_end);
		
		if (_CommunicationChannel.channel.send(plog_request_end)) {
			free_pack(plog_request_end.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgopro_set_response = _CommunicationChannel.channel.NEW.GOPRO_SET_RESPONSE(cur);
		fill(pgopro_set_response);
		if (!_CommunicationChannel.channel.send(pgopro_set_response)) {
			free_pack(pgopro_set_response.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pprotocol_version = _CommunicationChannel.channel.NEW.PROTOCOL_VERSION();
		fill(pprotocol_version);
		
		if (_CommunicationChannel.channel.send(pprotocol_version)) {
			free_pack(pprotocol_version.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto prally_fetch_point = _CommunicationChannel.channel.NEW.RALLY_FETCH_POINT();
		fill(prally_fetch_point);
		
		if (_CommunicationChannel.channel.send(prally_fetch_point)) {
			free_pack(prally_fetch_point.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pbattery_status = _CommunicationChannel.channel.NEW.BATTERY_STATUS(cur);
		fill(pbattery_status);
		if (!_CommunicationChannel.channel.send(pbattery_status)) {
			free_pack(pbattery_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmount_orientation = _CommunicationChannel.channel.NEW.MOUNT_ORIENTATION();
		fill(pmount_orientation);
		
		if (_CommunicationChannel.channel.send(pmount_orientation)) {
			free_pack(pmount_orientation.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pserial_control = _CommunicationChannel.channel.NEW.SERIAL_CONTROL(cur);
		fill(pserial_control);
		if (!_CommunicationChannel.channel.send(pserial_control)) {
			free_pack(pserial_control.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pparam_ext_set = _CommunicationChannel.channel.NEW.PARAM_EXT_SET(cur);
		fill(pparam_ext_set);
		if (!_CommunicationChannel.channel.send(pparam_ext_set)) {
			free_pack(pparam_ext_set.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pautopilot_version = _CommunicationChannel.channel.NEW.AUTOPILOT_VERSION(cur);
		fill(pautopilot_version);
		if (!_CommunicationChannel.channel.send(pautopilot_version)) {
			free_pack(pautopilot_version.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto psimstate = _CommunicationChannel.channel.NEW.SIMSTATE();
		fill(psimstate);
		
		if (_CommunicationChannel.channel.send(psimstate)) {
			free_pack(psimstate.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pset_video_stream_settings = _CommunicationChannel.channel.NEW.SET_VIDEO_STREAM_SETTINGS(cur);
		fill(pset_video_stream_settings);
		if (!_CommunicationChannel.channel.send(pset_video_stream_settings)) {
			free_pack(pset_video_stream_settings.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pplay_tune = _CommunicationChannel.channel.NEW.PLAY_TUNE(cur);
		fill(pplay_tune);
		if (!_CommunicationChannel.channel.send(pplay_tune)) {
			free_pack(pplay_tune.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdigicam_configure = _CommunicationChannel.channel.NEW.DIGICAM_CONFIGURE();
		fill(pdigicam_configure);
		
		if (_CommunicationChannel.channel.send(pdigicam_configure)) {
			free_pack(pdigicam_configure.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pscaled_pressure3 = _CommunicationChannel.channel.NEW.SCALED_PRESSURE3();
		fill(pscaled_pressure3);
		
		if (_CommunicationChannel.channel.send(pscaled_pressure3)) {
			free_pack(pscaled_pressure3.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pparam_ext_ack = _CommunicationChannel.channel.NEW.PARAM_EXT_ACK(cur);
		fill(pparam_ext_ack);
		if (!_CommunicationChannel.channel.send(pparam_ext_ack)) {
			free_pack(pparam_ext_ack.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto puavcan_node_info = _CommunicationChannel.channel.NEW.UAVCAN_NODE_INFO(cur);
		fill(puavcan_node_info);
		if (!_CommunicationChannel.channel.send(puavcan_node_info)) {
			free_pack(puavcan_node_info.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdata16 = _CommunicationChannel.channel.NEW.DATA16();
		fill(pdata16);
		
		if (_CommunicationChannel.channel.send(pdata16)) {
			free_pack(pdata16.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pset_mag_offsets = _CommunicationChannel.channel.NEW.SET_MAG_OFFSETS();
		fill(pset_mag_offsets);
		
		if (_CommunicationChannel.channel.send(pset_mag_offsets)) {
			free_pack(pset_mag_offsets.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pscaled_imu2 = _CommunicationChannel.channel.NEW.SCALED_IMU2();
		fill(pscaled_imu2);
		
		if (_CommunicationChannel.channel.send(pscaled_imu2)) {
			free_pack(pscaled_imu2.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pap_adc = _CommunicationChannel.channel.NEW.AP_ADC();
		fill(pap_adc);
		
		if (_CommunicationChannel.channel.send(pap_adc)) {
			free_pack(pap_adc.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pwind = _CommunicationChannel.channel.NEW.WIND();
		fill(pwind);
		
		if (_CommunicationChannel.channel.send(pwind)) {
			free_pack(pwind.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pautopilot_version_request = _CommunicationChannel.channel.NEW.AUTOPILOT_VERSION_REQUEST();
		fill(pautopilot_version_request);
		
		if (_CommunicationChannel.channel.send(pautopilot_version_request)) {
			free_pack(pautopilot_version_request.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdata_transmission_handshake = _CommunicationChannel.channel.NEW.DATA_TRANSMISSION_HANDSHAKE();
		fill(pdata_transmission_handshake);
		
		if (_CommunicationChannel.channel.send(pdata_transmission_handshake)) {
			free_pack(pdata_transmission_handshake.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdata64 = _CommunicationChannel.channel.NEW.DATA64();
		fill(pdata64);
		
		if (_CommunicationChannel.channel.send(pdata64)) {
			free_pack(pdata64.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgimbal_report = _CommunicationChannel.channel.NEW.GIMBAL_REPORT();
		fill(pgimbal_report);
		
		if (_CommunicationChannel.channel.send(pgimbal_report)) {
			free_pack(pgimbal_report.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdevice_op_write = _CommunicationChannel.channel.NEW.DEVICE_OP_WRITE(cur);
		fill(pdevice_op_write);
		if (!_CommunicationChannel.channel.send(pdevice_op_write)) {
			free_pack(pdevice_op_write.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdistance_sensor = _CommunicationChannel.channel.NEW.DISTANCE_SENSOR(cur);
		fill(pdistance_sensor);
		if (!_CommunicationChannel.channel.send(pdistance_sensor)) {
			free_pack(pdistance_sensor.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto phil_optical_flow = _CommunicationChannel.channel.NEW.HIL_OPTICAL_FLOW();
		fill(phil_optical_flow);
		
		if (_CommunicationChannel.channel.send(phil_optical_flow)) {
			free_pack(phil_optical_flow.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pscaled_pressure2 = _CommunicationChannel.channel.NEW.SCALED_PRESSURE2();
		fill(pscaled_pressure2);
		
		if (_CommunicationChannel.channel.send(pscaled_pressure2)) {
			free_pack(pscaled_pressure2.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pwind_cov = _CommunicationChannel.channel.NEW.WIND_COV();
		fill(pwind_cov);
		
		if (_CommunicationChannel.channel.send(pwind_cov)) {
			free_pack(pwind_cov.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgopro_set_request = _CommunicationChannel.channel.NEW.GOPRO_SET_REQUEST(cur);
		fill(pgopro_set_request);
		if (!_CommunicationChannel.channel.send(pgopro_set_request)) {
			free_pack(pgopro_set_request.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pvision_position_delta = _CommunicationChannel.channel.NEW.VISION_POSITION_DELTA();
		fill(pvision_position_delta);
		
		if (_CommunicationChannel.channel.send(pvision_position_delta)) {
			free_pack(pvision_position_delta.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto plogging_data = _CommunicationChannel.channel.NEW.LOGGING_DATA();
		fill(plogging_data);
		
		if (_CommunicationChannel.channel.send(plogging_data)) {
			free_pack(plogging_data.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pdevice_op_read = _CommunicationChannel.channel.NEW.DEVICE_OP_READ(cur);
		fill(pdevice_op_read);
		if (!_CommunicationChannel.channel.send(pdevice_op_read)) {
			free_pack(pdevice_op_read.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pmag_cal_progress = _CommunicationChannel.channel.NEW.MAG_CAL_PROGRESS(cur);
		fill(pmag_cal_progress);
		if (!_CommunicationChannel.channel.send(pmag_cal_progress)) {
			free_pack(pmag_cal_progress.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto phighres_imu = _CommunicationChannel.channel.NEW.HIGHRES_IMU();
		fill(phighres_imu);
		
		if (_CommunicationChannel.channel.send(phighres_imu)) {
			free_pack(phighres_imu.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pextended_sys_state = _CommunicationChannel.channel.NEW.EXTENDED_SYS_STATE(cur);
		fill(pextended_sys_state);
		if (!_CommunicationChannel.channel.send(pextended_sys_state)) {
			free_pack(pextended_sys_state.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto puavionix_adsb_out_dynamic = _CommunicationChannel.channel.NEW.UAVIONIX_ADSB_OUT_DYNAMIC(cur);
		fill(puavionix_adsb_out_dynamic);
		if (!_CommunicationChannel.channel.send(puavionix_adsb_out_dynamic)) {
			free_pack(puavionix_adsb_out_dynamic.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgopro_get_response = _CommunicationChannel.channel.NEW.GOPRO_GET_RESPONSE(cur);
		fill(pgopro_get_response);
		if (!_CommunicationChannel.channel.send(pgopro_get_response)) {
			free_pack(pgopro_get_response.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgps_inject_data = _CommunicationChannel.channel.NEW.GPS_INJECT_DATA();
		fill(pgps_inject_data);
		
		if (_CommunicationChannel.channel.send(pgps_inject_data)) {
			free_pack(pgps_inject_data.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto puavionix_adsb_transceiver_health_report = _CommunicationChannel.channel.NEW.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur);
		fill(puavionix_adsb_transceiver_health_report);
		if (!_CommunicationChannel.channel.send(puavionix_adsb_transceiver_health_report)) {
			free_pack(puavionix_adsb_transceiver_health_report.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pnamed_value_int = _CommunicationChannel.channel.NEW.NAMED_VALUE_INT(cur);
		fill(pnamed_value_int);
		if (!_CommunicationChannel.channel.send(pnamed_value_int)) {
			free_pack(pnamed_value_int.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto prpm = _CommunicationChannel.channel.NEW.RPM();
		fill(prpm);
		
		if (_CommunicationChannel.channel.send(prpm)) {
			free_pack(prpm.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pgps_rtcm_data = _CommunicationChannel.channel.NEW.GPS_RTCM_DATA();
		fill(pgps_rtcm_data);
		
		if (_CommunicationChannel.channel.send(pgps_rtcm_data)) {
			free_pack(pgps_rtcm_data.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pfile_transfer_protocol = _CommunicationChannel.channel.NEW.FILE_TRANSFER_PROTOCOL();
		fill(pfile_transfer_protocol);
		
		if (_CommunicationChannel.channel.send(pfile_transfer_protocol)) {
			free_pack(pfile_transfer_protocol.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto prangefinder = _CommunicationChannel.channel.NEW.RANGEFINDER();
		fill(prangefinder);
		
		if (_CommunicationChannel.channel.send(prangefinder)) {
			free_pack(prangefinder.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pradio_status = _CommunicationChannel.channel.NEW.RADIO_STATUS();
		fill(pradio_status);
		
		if (_CommunicationChannel.channel.send(pradio_status)) {
			free_pack(pradio_status.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto pfence_point = _CommunicationChannel.channel.NEW.FENCE_POINT();
		fill(pfence_point);
		
		if (_CommunicationChannel.channel.send(pfence_point)) {
			free_pack(pfence_point.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
		auto presource_request = _CommunicationChannel.channel.NEW.RESOURCE_REQUEST();
		fill(presource_request);
		
		if (_CommunicationChannel.channel.send(presource_request)) {
			free_pack(presource_request.unwrap_());
			assert("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
		}
		
	}
	
	bytes_out = _CommunicationChannel.channel.packs_into_bytes(buffer, 512); // sending packs
	_CommunicationChannel.channel.bytes_into_packs(buffer, 512); // receiving bytes
	
	
	
	
	return 0;
}
					  