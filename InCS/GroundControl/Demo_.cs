
using System;

using com.company.demo;
using System.Collections.Generic;
using Pack = org.unirail.AdHoc.Pack;
using Lib = org.unirail.AdHoc;

using HEARTBEAT = com.company.demo.GroundControl.HEARTBEAT;
using SYS_STATUS = com.company.demo.GroundControl.SYS_STATUS;
using SYSTEM_TIME = com.company.demo.GroundControl.SYSTEM_TIME;
using PING = com.company.demo.GroundControl.PING;
using CHANGE_OPERATOR_CONTROL = com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL;
using CHANGE_OPERATOR_CONTROL_ACK = com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL_ACK;
using AUTH_KEY = com.company.demo.GroundControl.AUTH_KEY;
using SET_MODE = com.company.demo.GroundControl.SET_MODE;
using PARAM_REQUEST_READ = com.company.demo.GroundControl.PARAM_REQUEST_READ;
using PARAM_REQUEST_LIST = com.company.demo.GroundControl.PARAM_REQUEST_LIST;
using PARAM_VALUE = com.company.demo.GroundControl.PARAM_VALUE;
using PARAM_SET = com.company.demo.GroundControl.PARAM_SET;
using GPS_RAW_INT = com.company.demo.GroundControl.GPS_RAW_INT;
using GPS_STATUS = com.company.demo.GroundControl.GPS_STATUS;
using SCALED_IMU = com.company.demo.GroundControl.SCALED_IMU;
using RAW_IMU = com.company.demo.GroundControl.RAW_IMU;
using RAW_PRESSURE = com.company.demo.GroundControl.RAW_PRESSURE;
using SCALED_PRESSURE = com.company.demo.GroundControl.SCALED_PRESSURE;
using ATTITUDE = com.company.demo.GroundControl.ATTITUDE;
using ATTITUDE_QUATERNION = com.company.demo.GroundControl.ATTITUDE_QUATERNION;
using LOCAL_POSITION_NED = com.company.demo.GroundControl.LOCAL_POSITION_NED;
using GLOBAL_POSITION_INT = com.company.demo.GroundControl.GLOBAL_POSITION_INT;
using RC_CHANNELS_SCALED = com.company.demo.GroundControl.RC_CHANNELS_SCALED;
using RC_CHANNELS_RAW = com.company.demo.GroundControl.RC_CHANNELS_RAW;
using SERVO_OUTPUT_RAW = com.company.demo.GroundControl.SERVO_OUTPUT_RAW;
using MISSION_REQUEST_PARTIAL_LIST = com.company.demo.GroundControl.MISSION_REQUEST_PARTIAL_LIST;
using MISSION_WRITE_PARTIAL_LIST = com.company.demo.GroundControl.MISSION_WRITE_PARTIAL_LIST;
using MISSION_ITEM = com.company.demo.GroundControl.MISSION_ITEM;
using MISSION_REQUEST = com.company.demo.GroundControl.MISSION_REQUEST;
using MISSION_SET_CURRENT = com.company.demo.GroundControl.MISSION_SET_CURRENT;
using MISSION_CURRENT = com.company.demo.GroundControl.MISSION_CURRENT;
using MISSION_REQUEST_LIST = com.company.demo.GroundControl.MISSION_REQUEST_LIST;
using MISSION_COUNT = com.company.demo.GroundControl.MISSION_COUNT;
using MISSION_CLEAR_ALL = com.company.demo.GroundControl.MISSION_CLEAR_ALL;
using MISSION_ITEM_REACHED = com.company.demo.GroundControl.MISSION_ITEM_REACHED;
using MISSION_ACK = com.company.demo.GroundControl.MISSION_ACK;
using SET_GPS_GLOBAL_ORIGIN = com.company.demo.GroundControl.SET_GPS_GLOBAL_ORIGIN;
using GPS_GLOBAL_ORIGIN = com.company.demo.GroundControl.GPS_GLOBAL_ORIGIN;
using PARAM_MAP_RC = com.company.demo.GroundControl.PARAM_MAP_RC;
using MISSION_REQUEST_INT = com.company.demo.GroundControl.MISSION_REQUEST_INT;
using SAFETY_SET_ALLOWED_AREA = com.company.demo.GroundControl.SAFETY_SET_ALLOWED_AREA;
using SAFETY_ALLOWED_AREA = com.company.demo.GroundControl.SAFETY_ALLOWED_AREA;
using ATTITUDE_QUATERNION_COV = com.company.demo.GroundControl.ATTITUDE_QUATERNION_COV;
using NAV_CONTROLLER_OUTPUT = com.company.demo.GroundControl.NAV_CONTROLLER_OUTPUT;
using GLOBAL_POSITION_INT_COV = com.company.demo.GroundControl.GLOBAL_POSITION_INT_COV;
using LOCAL_POSITION_NED_COV = com.company.demo.GroundControl.LOCAL_POSITION_NED_COV;
using RC_CHANNELS = com.company.demo.GroundControl.RC_CHANNELS;
using REQUEST_DATA_STREAM = com.company.demo.GroundControl.REQUEST_DATA_STREAM;
using DATA_STREAM = com.company.demo.GroundControl.DATA_STREAM;
using MANUAL_CONTROL = com.company.demo.GroundControl.MANUAL_CONTROL;
using RC_CHANNELS_OVERRIDE = com.company.demo.GroundControl.RC_CHANNELS_OVERRIDE;
using MISSION_ITEM_INT = com.company.demo.GroundControl.MISSION_ITEM_INT;
using VFR_HUD = com.company.demo.GroundControl.VFR_HUD;
using COMMAND_INT = com.company.demo.GroundControl.COMMAND_INT;
using COMMAND_LONG = com.company.demo.GroundControl.COMMAND_LONG;
using COMMAND_ACK = com.company.demo.GroundControl.COMMAND_ACK;
using MANUAL_SETPOINT = com.company.demo.GroundControl.MANUAL_SETPOINT;
using SET_ATTITUDE_TARGET = com.company.demo.GroundControl.SET_ATTITUDE_TARGET;
using ATTITUDE_TARGET = com.company.demo.GroundControl.ATTITUDE_TARGET;
using SET_POSITION_TARGET_LOCAL_NED = com.company.demo.GroundControl.SET_POSITION_TARGET_LOCAL_NED;
using POSITION_TARGET_LOCAL_NED = com.company.demo.GroundControl.POSITION_TARGET_LOCAL_NED;
using SET_POSITION_TARGET_GLOBAL_INT = com.company.demo.GroundControl.SET_POSITION_TARGET_GLOBAL_INT;
using POSITION_TARGET_GLOBAL_INT = com.company.demo.GroundControl.POSITION_TARGET_GLOBAL_INT;
using LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = com.company.demo.GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;
using HIL_STATE = com.company.demo.GroundControl.HIL_STATE;
using HIL_CONTROLS = com.company.demo.GroundControl.HIL_CONTROLS;
using HIL_RC_INPUTS_RAW = com.company.demo.GroundControl.HIL_RC_INPUTS_RAW;
using HIL_ACTUATOR_CONTROLS = com.company.demo.GroundControl.HIL_ACTUATOR_CONTROLS;
using OPTICAL_FLOW = com.company.demo.GroundControl.OPTICAL_FLOW;
using GLOBAL_VISION_POSITION_ESTIMATE = com.company.demo.GroundControl.GLOBAL_VISION_POSITION_ESTIMATE;
using VISION_POSITION_ESTIMATE = com.company.demo.GroundControl.VISION_POSITION_ESTIMATE;
using SwitchModeCommand = com.company.demo.GroundControl.SwitchModeCommand;
using PING33 = com.company.demo.GroundControl.PING33;
using VISION_SPEED_ESTIMATE = com.company.demo.GroundControl.VISION_SPEED_ESTIMATE;
using VICON_POSITION_ESTIMATE = com.company.demo.GroundControl.VICON_POSITION_ESTIMATE;
using HIGHRES_IMU = com.company.demo.GroundControl.HIGHRES_IMU;
using OPTICAL_FLOW_RAD = com.company.demo.GroundControl.OPTICAL_FLOW_RAD;
using HIL_SENSOR = com.company.demo.GroundControl.HIL_SENSOR;
using SIM_STATE = com.company.demo.GroundControl.SIM_STATE;
using RADIO_STATUS = com.company.demo.GroundControl.RADIO_STATUS;
using FILE_TRANSFER_PROTOCOL = com.company.demo.GroundControl.FILE_TRANSFER_PROTOCOL;
using TIMESYNC = com.company.demo.GroundControl.TIMESYNC;
using CAMERA_TRIGGER = com.company.demo.GroundControl.CAMERA_TRIGGER;
using HIL_GPS = com.company.demo.GroundControl.HIL_GPS;
using HIL_OPTICAL_FLOW = com.company.demo.GroundControl.HIL_OPTICAL_FLOW;
using HIL_STATE_QUATERNION = com.company.demo.GroundControl.HIL_STATE_QUATERNION;
using SCALED_IMU2 = com.company.demo.GroundControl.SCALED_IMU2;
using LOG_REQUEST_LIST = com.company.demo.GroundControl.LOG_REQUEST_LIST;
using LOG_ENTRY = com.company.demo.GroundControl.LOG_ENTRY;
using LOG_REQUEST_DATA = com.company.demo.GroundControl.LOG_REQUEST_DATA;
using LOG_DATA = com.company.demo.GroundControl.LOG_DATA;
using LOG_ERASE = com.company.demo.GroundControl.LOG_ERASE;
using LOG_REQUEST_END = com.company.demo.GroundControl.LOG_REQUEST_END;
using GPS_INJECT_DATA = com.company.demo.GroundControl.GPS_INJECT_DATA;
using GPS2_RAW = com.company.demo.GroundControl.GPS2_RAW;
using POWER_STATUS = com.company.demo.GroundControl.POWER_STATUS;
using SERIAL_CONTROL = com.company.demo.GroundControl.SERIAL_CONTROL;
using GPS_RTK = com.company.demo.GroundControl.GPS_RTK;
using GPS2_RTK = com.company.demo.GroundControl.GPS2_RTK;
using SCALED_IMU3 = com.company.demo.GroundControl.SCALED_IMU3;
using DATA_TRANSMISSION_HANDSHAKE = com.company.demo.GroundControl.DATA_TRANSMISSION_HANDSHAKE;
using ENCAPSULATED_DATA = com.company.demo.GroundControl.ENCAPSULATED_DATA;
using DISTANCE_SENSOR = com.company.demo.GroundControl.DISTANCE_SENSOR;
using TERRAIN_REQUEST = com.company.demo.GroundControl.TERRAIN_REQUEST;
using TERRAIN_DATA = com.company.demo.GroundControl.TERRAIN_DATA;
using TERRAIN_CHECK = com.company.demo.GroundControl.TERRAIN_CHECK;
using TERRAIN_REPORT = com.company.demo.GroundControl.TERRAIN_REPORT;
using SCALED_PRESSURE2 = com.company.demo.GroundControl.SCALED_PRESSURE2;
using ATT_POS_MOCAP = com.company.demo.GroundControl.ATT_POS_MOCAP;
using SET_ACTUATOR_CONTROL_TARGET = com.company.demo.GroundControl.SET_ACTUATOR_CONTROL_TARGET;
using ACTUATOR_CONTROL_TARGET = com.company.demo.GroundControl.ACTUATOR_CONTROL_TARGET;
using ALTITUDE = com.company.demo.GroundControl.ALTITUDE;
using RESOURCE_REQUEST = com.company.demo.GroundControl.RESOURCE_REQUEST;
using SCALED_PRESSURE3 = com.company.demo.GroundControl.SCALED_PRESSURE3;
using FOLLOW_TARGET = com.company.demo.GroundControl.FOLLOW_TARGET;
using CONTROL_SYSTEM_STATE = com.company.demo.GroundControl.CONTROL_SYSTEM_STATE;
using BATTERY_STATUS = com.company.demo.GroundControl.BATTERY_STATUS;
using AUTOPILOT_VERSION = com.company.demo.GroundControl.AUTOPILOT_VERSION;
using LANDING_TARGET = com.company.demo.GroundControl.LANDING_TARGET;
using ESTIMATOR_STATUS = com.company.demo.GroundControl.ESTIMATOR_STATUS;
using WIND_COV = com.company.demo.GroundControl.WIND_COV;
using GPS_INPUT = com.company.demo.GroundControl.GPS_INPUT;
using GPS_RTCM_DATA = com.company.demo.GroundControl.GPS_RTCM_DATA;
using HIGH_LATENCY = com.company.demo.GroundControl.HIGH_LATENCY;
using VIBRATION = com.company.demo.GroundControl.VIBRATION;
using HOME_POSITION = com.company.demo.GroundControl.HOME_POSITION;
using SET_HOME_POSITION = com.company.demo.GroundControl.SET_HOME_POSITION;
using MESSAGE_INTERVAL = com.company.demo.GroundControl.MESSAGE_INTERVAL;
using EXTENDED_SYS_STATE = com.company.demo.GroundControl.EXTENDED_SYS_STATE;
using ADSB_VEHICLE = com.company.demo.GroundControl.ADSB_VEHICLE;
using COLLISION = com.company.demo.GroundControl.COLLISION;
using V2_EXTENSION = com.company.demo.GroundControl.V2_EXTENSION;
using MEMORY_VECT = com.company.demo.GroundControl.MEMORY_VECT;
using DEBUG_VECT = com.company.demo.GroundControl.DEBUG_VECT;
using NAMED_VALUE_FLOAT = com.company.demo.GroundControl.NAMED_VALUE_FLOAT;
using NAMED_VALUE_INT = com.company.demo.GroundControl.NAMED_VALUE_INT;
using STATUSTEXT = com.company.demo.GroundControl.STATUSTEXT;
using DEBUG = com.company.demo.GroundControl.DEBUG;
using SETUP_SIGNING = com.company.demo.GroundControl.SETUP_SIGNING;
using BUTTON_CHANGE = com.company.demo.GroundControl.BUTTON_CHANGE;
using PLAY_TUNE = com.company.demo.GroundControl.PLAY_TUNE;
using CAMERA_INFORMATION = com.company.demo.GroundControl.CAMERA_INFORMATION;
using CAMERA_SETTINGS = com.company.demo.GroundControl.CAMERA_SETTINGS;
using STORAGE_INFORMATION = com.company.demo.GroundControl.STORAGE_INFORMATION;
using CAMERA_CAPTURE_STATUS = com.company.demo.GroundControl.CAMERA_CAPTURE_STATUS;
using CAMERA_IMAGE_CAPTURED = com.company.demo.GroundControl.CAMERA_IMAGE_CAPTURED;
using FLIGHT_INFORMATION = com.company.demo.GroundControl.FLIGHT_INFORMATION;
using MOUNT_ORIENTATION = com.company.demo.GroundControl.MOUNT_ORIENTATION;
using LOGGING_DATA = com.company.demo.GroundControl.LOGGING_DATA;
using LOGGING_DATA_ACKED = com.company.demo.GroundControl.LOGGING_DATA_ACKED;
using LOGGING_ACK = com.company.demo.GroundControl.LOGGING_ACK;
using VIDEO_STREAM_INFORMATION = com.company.demo.GroundControl.VIDEO_STREAM_INFORMATION;
using SET_VIDEO_STREAM_SETTINGS = com.company.demo.GroundControl.SET_VIDEO_STREAM_SETTINGS;
using WIFI_CONFIG_AP = com.company.demo.GroundControl.WIFI_CONFIG_AP;
using PROTOCOL_VERSION = com.company.demo.GroundControl.PROTOCOL_VERSION;
using UAVCAN_NODE_STATUS = com.company.demo.GroundControl.UAVCAN_NODE_STATUS;
using UAVCAN_NODE_INFO = com.company.demo.GroundControl.UAVCAN_NODE_INFO;
using PARAM_EXT_REQUEST_READ = com.company.demo.GroundControl.PARAM_EXT_REQUEST_READ;
using PARAM_EXT_REQUEST_LIST = com.company.demo.GroundControl.PARAM_EXT_REQUEST_LIST;
using PARAM_EXT_VALUE = com.company.demo.GroundControl.PARAM_EXT_VALUE;
using PARAM_EXT_SET = com.company.demo.GroundControl.PARAM_EXT_SET;
using PARAM_EXT_ACK = com.company.demo.GroundControl.PARAM_EXT_ACK;
using OBSTACLE_DISTANCE = com.company.demo.GroundControl.OBSTACLE_DISTANCE;
using UAVIONIX_ADSB_OUT_CFG = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG;
using UAVIONIX_ADSB_OUT_DYNAMIC = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC;
using UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT = com.company.demo.GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;
using SENSOR_OFFSETS = com.company.demo.GroundControl.SENSOR_OFFSETS;
using SET_MAG_OFFSETS = com.company.demo.GroundControl.SET_MAG_OFFSETS;
using MEMINFO = com.company.demo.GroundControl.MEMINFO;
using AP_ADC = com.company.demo.GroundControl.AP_ADC;
using DIGICAM_CONFIGURE = com.company.demo.GroundControl.DIGICAM_CONFIGURE;
using DIGICAM_CONTROL = com.company.demo.GroundControl.DIGICAM_CONTROL;
using MOUNT_CONFIGURE = com.company.demo.GroundControl.MOUNT_CONFIGURE;
using MOUNT_CONTROL = com.company.demo.GroundControl.MOUNT_CONTROL;
using MOUNT_STATUS = com.company.demo.GroundControl.MOUNT_STATUS;
using FENCE_POINT = com.company.demo.GroundControl.FENCE_POINT;
using FENCE_FETCH_POINT = com.company.demo.GroundControl.FENCE_FETCH_POINT;
using FENCE_STATUS = com.company.demo.GroundControl.FENCE_STATUS;
using AHRS = com.company.demo.GroundControl.AHRS;
using SIMSTATE = com.company.demo.GroundControl.SIMSTATE;
using HWSTATUS = com.company.demo.GroundControl.HWSTATUS;
using RADIO = com.company.demo.GroundControl.RADIO;
using LIMITS_STATUS = com.company.demo.GroundControl.LIMITS_STATUS;
using WIND = com.company.demo.GroundControl.WIND;
using DATA16 = com.company.demo.GroundControl.DATA16;
using DATA32 = com.company.demo.GroundControl.DATA32;
using DATA64 = com.company.demo.GroundControl.DATA64;
using DATA96 = com.company.demo.GroundControl.DATA96;
using RANGEFINDER = com.company.demo.GroundControl.RANGEFINDER;
using AIRSPEED_AUTOCAL = com.company.demo.GroundControl.AIRSPEED_AUTOCAL;
using RALLY_POINT = com.company.demo.GroundControl.RALLY_POINT;
using RALLY_FETCH_POINT = com.company.demo.GroundControl.RALLY_FETCH_POINT;
using COMPASSMOT_STATUS = com.company.demo.GroundControl.COMPASSMOT_STATUS;
using AHRS2 = com.company.demo.GroundControl.AHRS2;
using CAMERA_STATUS = com.company.demo.GroundControl.CAMERA_STATUS;
using CAMERA_FEEDBACK = com.company.demo.GroundControl.CAMERA_FEEDBACK;
using BATTERY2 = com.company.demo.GroundControl.BATTERY2;
using AHRS3 = com.company.demo.GroundControl.AHRS3;
using AUTOPILOT_VERSION_REQUEST = com.company.demo.GroundControl.AUTOPILOT_VERSION_REQUEST;
using REMOTE_LOG_DATA_BLOCK = com.company.demo.GroundControl.REMOTE_LOG_DATA_BLOCK;
using REMOTE_LOG_BLOCK_STATUS = com.company.demo.GroundControl.REMOTE_LOG_BLOCK_STATUS;
using LED_CONTROL = com.company.demo.GroundControl.LED_CONTROL;
using MAG_CAL_PROGRESS = com.company.demo.GroundControl.MAG_CAL_PROGRESS;
using MAG_CAL_REPORT = com.company.demo.GroundControl.MAG_CAL_REPORT;
using EKF_STATUS_REPORT = com.company.demo.GroundControl.EKF_STATUS_REPORT;
using PID_TUNING = com.company.demo.GroundControl.PID_TUNING;
using GIMBAL_REPORT = com.company.demo.GroundControl.GIMBAL_REPORT;
using GIMBAL_CONTROL = com.company.demo.GroundControl.GIMBAL_CONTROL;
using GIMBAL_TORQUE_CMD_REPORT = com.company.demo.GroundControl.GIMBAL_TORQUE_CMD_REPORT;
using GOPRO_HEARTBEAT = com.company.demo.GroundControl.GOPRO_HEARTBEAT;
using GOPRO_GET_REQUEST = com.company.demo.GroundControl.GOPRO_GET_REQUEST;
using GOPRO_GET_RESPONSE = com.company.demo.GroundControl.GOPRO_GET_RESPONSE;
using GOPRO_SET_REQUEST = com.company.demo.GroundControl.GOPRO_SET_REQUEST;
using GOPRO_SET_RESPONSE = com.company.demo.GroundControl.GOPRO_SET_RESPONSE;
using RPM = com.company.demo.GroundControl.RPM;
using DEVICE_OP_READ = com.company.demo.GroundControl.DEVICE_OP_READ;
using DEVICE_OP_READ_REPLY = com.company.demo.GroundControl.DEVICE_OP_READ_REPLY;
using DEVICE_OP_WRITE = com.company.demo.GroundControl.DEVICE_OP_WRITE;
using DEVICE_OP_WRITE_REPLY = com.company.demo.GroundControl.DEVICE_OP_WRITE_REPLY;
using ADAP_TUNING = com.company.demo.GroundControl.ADAP_TUNING;
using VISION_POSITION_DELTA = com.company.demo.GroundControl.VISION_POSITION_DELTA;
using MAV_SYS_STATUS_SENSOR = com.company.demo.GroundControl.MAV_SYS_STATUS_SENSOR;
using MAV_PROTOCOL_CAPABILITY = com.company.demo.GroundControl.MAV_PROTOCOL_CAPABILITY;
using MAV_CMD = com.company.demo.GroundControl.MAV_CMD;
using ADSB_ALTITUDE_TYPE = com.company.demo.GroundControl.ADSB_ALTITUDE_TYPE;
using MAV_SENSOR_ORIENTATION = com.company.demo.GroundControl.MAV_SENSOR_ORIENTATION;
using MAV_MISSION_RESULT = com.company.demo.GroundControl.MAV_MISSION_RESULT;
using MAV_PARAM_TYPE = com.company.demo.GroundControl.MAV_PARAM_TYPE;
using GOPRO_REQUEST_STATUS = com.company.demo.GroundControl.GOPRO_REQUEST_STATUS;
using UAVIONIX_ADSB_OUT_DYNAMIC_STATE = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC_STATE;
using GPS_INPUT_IGNORE_FLAGS = com.company.demo.GroundControl.GPS_INPUT_IGNORE_FLAGS;
using ADSB_FLAGS = com.company.demo.GroundControl.ADSB_FLAGS;
using PID_TUNING_AXIS = com.company.demo.GroundControl.PID_TUNING_AXIS;
using MAV_MISSION_TYPE = com.company.demo.GroundControl.MAV_MISSION_TYPE;
using MAV_REMOTE_LOG_DATA_BLOCK_STATUSES = com.company.demo.GroundControl.MAV_REMOTE_LOG_DATA_BLOCK_STATUSES;
using ADSB_EMITTER_TYPE = com.company.demo.GroundControl.ADSB_EMITTER_TYPE;
using SERIAL_CONTROL_FLAG = com.company.demo.GroundControl.SERIAL_CONTROL_FLAG;
using GOPRO_COMMAND = com.company.demo.GroundControl.GOPRO_COMMAND;
using LIMIT_MODULE = com.company.demo.GroundControl.LIMIT_MODULE;
using MAV_COLLISION_THREAT_LEVEL = com.company.demo.GroundControl.MAV_COLLISION_THREAT_LEVEL;
using SERIAL_CONTROL_DEV = com.company.demo.GroundControl.SERIAL_CONTROL_DEV;
using MAV_RESULT = com.company.demo.GroundControl.MAV_RESULT;
using FENCE_BREACH = com.company.demo.GroundControl.FENCE_BREACH;
using UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE;
using MAV_SEVERITY = com.company.demo.GroundControl.MAV_SEVERITY;
using MAV_COLLISION_SRC = com.company.demo.GroundControl.MAV_COLLISION_SRC;
using MAV_VTOL_STATE = com.company.demo.GroundControl.MAV_VTOL_STATE;
using CAMERA_CAP_FLAGS = com.company.demo.GroundControl.CAMERA_CAP_FLAGS;
using UAVIONIX_ADSB_OUT_RF_SELECT = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_RF_SELECT;
using RALLY_FLAGS = com.company.demo.GroundControl.RALLY_FLAGS;
using MAV_MODE = com.company.demo.GroundControl.MAV_MODE;
using UAVIONIX_ADSB_RF_HEALTH = com.company.demo.GroundControl.UAVIONIX_ADSB_RF_HEALTH;
using GOPRO_HEARTBEAT_STATUS = com.company.demo.GroundControl.GOPRO_HEARTBEAT_STATUS;
using MAV_DISTANCE_SENSOR = com.company.demo.GroundControl.MAV_DISTANCE_SENSOR;
using DEVICE_OP_BUSTYPE = com.company.demo.GroundControl.DEVICE_OP_BUSTYPE;
using LANDING_TARGET_TYPE = com.company.demo.GroundControl.LANDING_TARGET_TYPE;
using MAV_ESTIMATOR_TYPE = com.company.demo.GroundControl.MAV_ESTIMATOR_TYPE;
using MAV_AUTOPILOT = com.company.demo.GroundControl.MAV_AUTOPILOT;
using MAV_LANDED_STATE = com.company.demo.GroundControl.MAV_LANDED_STATE;
using CAMERA_MODE = com.company.demo.GroundControl.CAMERA_MODE;
using UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON;
using CAMERA_STATUS_TYPES = com.company.demo.GroundControl.CAMERA_STATUS_TYPES;
using MAV_POWER_STATUS = com.company.demo.GroundControl.MAV_POWER_STATUS;
using UAVCAN_NODE_MODE = com.company.demo.GroundControl.UAVCAN_NODE_MODE;
using UAVCAN_NODE_HEALTH = com.company.demo.GroundControl.UAVCAN_NODE_HEALTH;
using MAV_MODE_FLAG = com.company.demo.GroundControl.MAV_MODE_FLAG;
using UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT;
using MAV_FRAME = com.company.demo.GroundControl.MAV_FRAME;
using MAV_PARAM_EXT_TYPE = com.company.demo.GroundControl.MAV_PARAM_EXT_TYPE;
using MAV_TYPE = com.company.demo.GroundControl.MAV_TYPE;
using MAV_COLLISION_ACTION = com.company.demo.GroundControl.MAV_COLLISION_ACTION;
using GOPRO_HEARTBEAT_FLAGS = com.company.demo.GroundControl.GOPRO_HEARTBEAT_FLAGS;
using UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX = com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX;
using ESTIMATOR_STATUS_FLAGS = com.company.demo.GroundControl.ESTIMATOR_STATUS_FLAGS;
using LIMITS_STATE = com.company.demo.GroundControl.LIMITS_STATE;
using GOPRO_CAPTURE_MODE = com.company.demo.GroundControl.GOPRO_CAPTURE_MODE;
using EKF_STATUS_FLAGS = com.company.demo.GroundControl.EKF_STATUS_FLAGS;
using CAMERA_FEEDBACK_FLAGS = com.company.demo.GroundControl.CAMERA_FEEDBACK_FLAGS;
using UAVIONIX_ADSB_EMERGENCY_STATUS = com.company.demo.GroundControl.UAVIONIX_ADSB_EMERGENCY_STATUS;
using MAV_MOUNT_MODE = com.company.demo.GroundControl.MAV_MOUNT_MODE;
using MAV_STATE = com.company.demo.GroundControl.MAV_STATE;
using MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS = com.company.demo.GroundControl.MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS;
using PARAM_ACK = com.company.demo.GroundControl.PARAM_ACK;
using GPS_FIX_TYPE = com.company.demo.GroundControl.GPS_FIX_TYPE;
using MAV_BATTERY_FUNCTION = com.company.demo.GroundControl.MAV_BATTERY_FUNCTION;
using MAG_CAL_STATUS = com.company.demo.GroundControl.MAG_CAL_STATUS;
using MAV_BATTERY_TYPE = com.company.demo.GroundControl.MAV_BATTERY_TYPE;

namespace org.unirail
{
    public struct Demo
    {

        static   CAMERA_FEEDBACK_FLAGS    some_CAMERA_FEEDBACK_FLAGS       = CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO ;
        static   MAV_SENSOR_ORIENTATION    some_MAV_SENSOR_ORIENTATION       = MAV_SENSOR_ORIENTATION.NONE ;
        static   MAV_ESTIMATOR_TYPE    some_MAV_ESTIMATOR_TYPE       = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE ;
        static String      some_string    = "null" ;
        static   MAV_SEVERITY    some_MAV_SEVERITY       = MAV_SEVERITY.MAV_SEVERITY_EMERGENCY ;
        static   LIMIT_MODULE    some_LIMIT_MODULE       = LIMIT_MODULE.LIMIT_GPSLOCK ;
        static   bool    some_bool       = true ;
        static   GOPRO_HEARTBEAT_STATUS    some_GOPRO_HEARTBEAT_STATUS       = GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_DISCONNECTED ;
        static   GOPRO_CAPTURE_MODE    some_GOPRO_CAPTURE_MODE       = GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO ;
        static   ADSB_ALTITUDE_TYPE    some_ADSB_ALTITUDE_TYPE       = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH ;
        static   PARAM_ACK    some_PARAM_ACK       = PARAM_ACK.PARAM_ACK_ACCEPTED ;
        static   UAVCAN_NODE_HEALTH    some_UAVCAN_NODE_HEALTH       = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK ;
        static   MAV_BATTERY_FUNCTION    some_MAV_BATTERY_FUNCTION       = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN ;
        static   MAV_MODE    some_MAV_MODE       = MAV_MODE.PREFLIGHT ;
        static   CAMERA_MODE    some_CAMERA_MODE       = CAMERA_MODE.CAMERA_MODE_IMAGE ;
        static   MAV_VTOL_STATE    some_MAV_VTOL_STATE       = MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED ;
        static   MAV_BATTERY_TYPE    some_MAV_BATTERY_TYPE       = MAV_BATTERY_TYPE.UNKNOWN ;
        static   MAV_LANDED_STATE    some_MAV_LANDED_STATE       = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED ;
        static   MAV_MISSION_TYPE    some_MAV_MISSION_TYPE       = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION ;
        static   UAVIONIX_ADSB_OUT_DYNAMIC_STATE    some_UAVIONIX_ADSB_OUT_DYNAMIC_STATE       = UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE ;
        static   UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT    some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT       = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.NO_DATA ;
        static   MAV_TYPE    some_MAV_TYPE       = MAV_TYPE.GENERIC ;
        static   MAV_RESULT    some_MAV_RESULT       = MAV_RESULT.MAV_RESULT_ACCEPTED ;
        static   MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS    some_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS       = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP ;
        static   UAVIONIX_ADSB_EMERGENCY_STATUS    some_UAVIONIX_ADSB_EMERGENCY_STATUS       = UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_EMERGENCY ;
        static   LIMITS_STATE    some_LIMITS_STATE       = LIMITS_STATE.LIMITS_INIT ;
        static   ESTIMATOR_STATUS_FLAGS    some_ESTIMATOR_STATUS_FLAGS       = ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE ;
        static   MAV_FRAME    some_MAV_FRAME       = MAV_FRAME.MAV_FRAME_GLOBAL ;
        static   DEVICE_OP_BUSTYPE    some_DEVICE_OP_BUSTYPE       = DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C ;
        static   FENCE_BREACH    some_FENCE_BREACH       = FENCE_BREACH.FENCE_BREACH_NONE ;
        static   MAV_MISSION_RESULT    some_MAV_MISSION_RESULT       = MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED ;
        static   CAMERA_CAP_FLAGS    some_CAMERA_CAP_FLAGS       = CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO ;
        static   MAV_MOUNT_MODE    some_MAV_MOUNT_MODE       = MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT ;
        static   ADSB_EMITTER_TYPE    some_ADSB_EMITTER_TYPE       = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO ;
        static   float    some_float ;
        static   short    some_short ;
        static   UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON    some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON       = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA ;
        static   MAV_CMD    some_MAV_CMD       = MAV_CMD.MAV_CMD_NAV_WAYPOINT ;
        static   GOPRO_COMMAND    some_GOPRO_COMMAND       = GOPRO_COMMAND.GOPRO_COMMAND_POWER ;
        static   UAVCAN_NODE_MODE    some_UAVCAN_NODE_MODE       = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL ;
        static   MAV_SYS_STATUS_SENSOR    some_MAV_SYS_STATUS_SENSOR       = MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO ;
        static   MAG_CAL_STATUS    some_MAG_CAL_STATUS       = MAG_CAL_STATUS.MAG_CAL_NOT_STARTED ;
        static   MAV_POWER_STATUS    some_MAV_POWER_STATUS       = MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID ;
        static   long    some_long ;
        static   SERIAL_CONTROL_DEV    some_SERIAL_CONTROL_DEV       = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1 ;
        static   UAVIONIX_ADSB_RF_HEALTH    some_UAVIONIX_ADSB_RF_HEALTH       = UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING ;
        static   MAV_PARAM_TYPE    some_MAV_PARAM_TYPE       = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8 ;
        static   EKF_STATUS_FLAGS    some_EKF_STATUS_FLAGS       = EKF_STATUS_FLAGS.EKF_ATTITUDE ;
        static   sbyte    some_sbyte ;
        static   MAV_DISTANCE_SENSOR    some_MAV_DISTANCE_SENSOR       = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER ;
        static   MAV_AUTOPILOT    some_MAV_AUTOPILOT       = MAV_AUTOPILOT.GENERIC ;
        static   UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE    some_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE       = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA ;
        static   MAV_COLLISION_THREAT_LEVEL    some_MAV_COLLISION_THREAT_LEVEL       = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE ;
        static   MAV_COLLISION_ACTION    some_MAV_COLLISION_ACTION       = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE ;
        static   MAV_REMOTE_LOG_DATA_BLOCK_STATUSES    some_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES       = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK ;
        static   MAV_PROTOCOL_CAPABILITY    some_MAV_PROTOCOL_CAPABILITY       = MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT ;
        static   UAVIONIX_ADSB_OUT_RF_SELECT    some_UAVIONIX_ADSB_OUT_RF_SELECT       = UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY ;
        static   RALLY_FLAGS    some_RALLY_FLAGS       = RALLY_FLAGS.FAVORABLE_WIND ;
        static   UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX    some_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX       = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 ;
        static   MAV_MODE_FLAG    some_MAV_MODE_FLAG       = MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ;
        static   MAV_STATE    some_MAV_STATE       = MAV_STATE.UNINIT ;
        static   GOPRO_REQUEST_STATUS    some_GOPRO_REQUEST_STATUS       = GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS ;
        static   int    some_int ;
        static   ADSB_FLAGS    some_ADSB_FLAGS       = ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS ;
        static   CAMERA_STATUS_TYPES    some_CAMERA_STATUS_TYPES       = CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_HEARTBEAT ;
        static   GPS_FIX_TYPE    some_GPS_FIX_TYPE       = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS ;
        static   LANDING_TARGET_TYPE    some_LANDING_TARGET_TYPE       = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON ;
        static   MAV_PARAM_EXT_TYPE    some_MAV_PARAM_EXT_TYPE       = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8 ;
        static   GOPRO_HEARTBEAT_FLAGS    some_GOPRO_HEARTBEAT_FLAGS       = GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING ;
        static   PID_TUNING_AXIS    some_PID_TUNING_AXIS       = PID_TUNING_AXIS.PID_TUNING_ROLL ;
        static   SERIAL_CONTROL_FLAG    some_SERIAL_CONTROL_FLAG       = SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY ;
        static   GPS_INPUT_IGNORE_FLAGS    some_GPS_INPUT_IGNORE_FLAGS       = GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT ;
        static   MAV_COLLISION_SRC    some_MAV_COLLISION_SRC       = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB ;
        public static void onHEARTBEAT(HEARTBEAT heartbeat)
        {
            {
                var item = heartbeat.typE();
                if(item.HasValue)
                    some_MAV_TYPE = item.Value ;
            }
            {
                var item = heartbeat.autopilot();
                if(item.HasValue)
                    some_MAV_AUTOPILOT = item.Value ;
            }
            {
                var item = heartbeat.base_mode();
                if(item.HasValue)
                    some_MAV_MODE_FLAG = item.Value ;
            }
            var some_custom_mode = heartbeat.custom_mode() ;
            {
                var item = heartbeat.system_status();
                if(item.HasValue)
                    some_MAV_STATE = item.Value ;
            }
            var some_mavlink_version = heartbeat.mavlink_version() ;
        }
        public static void onSYS_STATUS(SYS_STATUS sys_status)
        {
            {
                var item = sys_status.onboard_control_sensors_present();
                if(item.HasValue)
                    some_MAV_SYS_STATUS_SENSOR = item.Value ;
            }
            {
                var item = sys_status.onboard_control_sensors_enabled();
                if(item.HasValue)
                    some_MAV_SYS_STATUS_SENSOR = item.Value ;
            }
            {
                var item = sys_status.onboard_control_sensors_health();
                if(item.HasValue)
                    some_MAV_SYS_STATUS_SENSOR = item.Value ;
            }
            var some_load = sys_status.load() ;
            var some_voltage_battery = sys_status.voltage_battery() ;
            var some_current_battery = sys_status.current_battery() ;
            var some_battery_remaining = sys_status.battery_remaining() ;
            var some_drop_rate_comm = sys_status.drop_rate_comm() ;
            var some_errors_comm = sys_status.errors_comm() ;
            var some_errors_count1 = sys_status.errors_count1() ;
            var some_errors_count2 = sys_status.errors_count2() ;
            var some_errors_count3 = sys_status.errors_count3() ;
            var some_errors_count4 = sys_status.errors_count4() ;
        }
        public static void onSYSTEM_TIME(SYSTEM_TIME system_time)
        {
            var some_time_unix_usec = system_time.time_unix_usec() ;
            var some_time_boot_ms = system_time.time_boot_ms() ;
        }
        public static void onPING(PING ping)
        {
            var some_time_usec = ping.time_usec() ;
            var some_seq = ping.seq() ;
            var some_target_system = ping.target_system() ;
            var some_target_component = ping.target_component() ;
        }
        public static void onCHANGE_OPERATOR_CONTROL(CHANGE_OPERATOR_CONTROL change_operator_control)
        {
            var some_target_system = change_operator_control.target_system() ;
            var some_control_request = change_operator_control.control_request() ;
            var some_version = change_operator_control.version() ;
            {
                var item = change_operator_control.passkey();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void onCHANGE_OPERATOR_CONTROL_ACK(CHANGE_OPERATOR_CONTROL_ACK change_operator_control_ack)
        {
            var some_gcs_system_id = change_operator_control_ack.gcs_system_id() ;
            var some_control_request = change_operator_control_ack.control_request() ;
            var some_ack = change_operator_control_ack.ack() ;
        }
        public static void onAUTH_KEY(AUTH_KEY auth_key)
        {
            {
                var item = auth_key.key();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void onSET_MODE(SET_MODE set_mode)
        {
            var some_target_system = set_mode.target_system() ;
            {
                var item = set_mode.base_mode();
                if(item.HasValue)
                    some_MAV_MODE = item.Value ;
            }
            var some_custom_mode = set_mode.custom_mode() ;
        }
        public static void onPARAM_REQUEST_READ(PARAM_REQUEST_READ param_request_read)
        {
            var some_target_system = param_request_read.target_system() ;
            var some_target_component = param_request_read.target_component() ;
            {
                var item = param_request_read.param_id();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_param_index = param_request_read.param_index() ;
        }
        public static void onPARAM_REQUEST_LIST(PARAM_REQUEST_LIST param_request_list)
        {
            var some_target_system = param_request_list.target_system() ;
            var some_target_component = param_request_list.target_component() ;
        }
        public static void onPARAM_VALUE(PARAM_VALUE param_value)
        {
            {
                var item = param_value.param_id();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_param_value = param_value.param_value() ;
            {
                var item = param_value.param_type();
                if(item.HasValue)
                    some_MAV_PARAM_TYPE = item.Value ;
            }
            var some_param_count = param_value.param_count() ;
            var some_param_index = param_value.param_index() ;
        }
        public static void onPARAM_SET(PARAM_SET param_set)
        {
            var some_target_system = param_set.target_system() ;
            var some_target_component = param_set.target_component() ;
            {
                var item = param_set.param_id();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_param_value = param_set.param_value() ;
            {
                var item = param_set.param_type();
                if(item.HasValue)
                    some_MAV_PARAM_TYPE = item.Value ;
            }
        }
        public static void onGPS_RAW_INT(GPS_RAW_INT gps_raw_int)
        {
            var some_time_usec = gps_raw_int.time_usec() ;
            {
                var item = gps_raw_int.fix_type();
                if(item.HasValue)
                    some_GPS_FIX_TYPE = item.Value ;
            }
            var some_lat = gps_raw_int.lat() ;
            var some_lon = gps_raw_int.lon() ;
            var some_alt = gps_raw_int.alt() ;
            var some_eph = gps_raw_int.eph() ;
            var some_epv = gps_raw_int.epv() ;
            var some_vel = gps_raw_int.vel() ;
            var some_cog = gps_raw_int.cog() ;
            var some_satellites_visible = gps_raw_int.satellites_visible() ;
            {
                var item = gps_raw_int.alt_ellipsoid();
                if(item.HasValue)
                    some_int = item.Value ;
            }
            {
                var item = gps_raw_int.h_acc();
                if(item.HasValue)
                    some_int = item.Value ;
            }
            {
                var item = gps_raw_int.v_acc();
                if(item.HasValue)
                    some_int = item.Value ;
            }
            {
                var item = gps_raw_int.vel_acc();
                if(item.HasValue)
                    some_int = item.Value ;
            }
            {
                var item = gps_raw_int.hdg_acc();
                if(item.HasValue)
                    some_int = item.Value ;
            }
        }
        public static void onGPS_STATUS(GPS_STATUS gps_status)
        {
            var some_satellites_visible = gps_status.satellites_visible() ;
            {
                var item = gps_status.satellite_prn();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            {
                var item = gps_status.satellite_used();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            {
                var item = gps_status.satellite_elevation();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            {
                var item = gps_status.satellite_azimuth();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            {
                var item = gps_status.satellite_snr();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void onSCALED_IMU(SCALED_IMU scaled_imu)
        {
            var some_time_boot_ms = scaled_imu.time_boot_ms() ;
            var some_xacc = scaled_imu.xacc() ;
            var some_yacc = scaled_imu.yacc() ;
            var some_zacc = scaled_imu.zacc() ;
            var some_xgyro = scaled_imu.xgyro() ;
            var some_ygyro = scaled_imu.ygyro() ;
            var some_zgyro = scaled_imu.zgyro() ;
            var some_xmag = scaled_imu.xmag() ;
            var some_ymag = scaled_imu.ymag() ;
            var some_zmag = scaled_imu.zmag() ;
        }
        public static void onRAW_IMU(RAW_IMU raw_imu)
        {
            var some_time_usec = raw_imu.time_usec() ;
            var some_xacc = raw_imu.xacc() ;
            var some_yacc = raw_imu.yacc() ;
            var some_zacc = raw_imu.zacc() ;
            var some_xgyro = raw_imu.xgyro() ;
            var some_ygyro = raw_imu.ygyro() ;
            var some_zgyro = raw_imu.zgyro() ;
            var some_xmag = raw_imu.xmag() ;
            var some_ymag = raw_imu.ymag() ;
            var some_zmag = raw_imu.zmag() ;
        }
        public static void onRAW_PRESSURE(RAW_PRESSURE raw_pressure)
        {
            var some_time_usec = raw_pressure.time_usec() ;
            var some_press_abs = raw_pressure.press_abs() ;
            var some_press_diff1 = raw_pressure.press_diff1() ;
            var some_press_diff2 = raw_pressure.press_diff2() ;
            var some_temperature = raw_pressure.temperature() ;
        }
        public static void onSCALED_PRESSURE(SCALED_PRESSURE scaled_pressure)
        {
            var some_time_boot_ms = scaled_pressure.time_boot_ms() ;
            var some_press_abs = scaled_pressure.press_abs() ;
            var some_press_diff = scaled_pressure.press_diff() ;
            var some_temperature = scaled_pressure.temperature() ;
        }
        public static void onATTITUDE(ATTITUDE attitude)
        {
            var some_time_boot_ms = attitude.time_boot_ms() ;
            var some_roll = attitude.roll() ;
            var some_pitch = attitude.pitch() ;
            var some_yaw = attitude.yaw() ;
            var some_rollspeed = attitude.rollspeed() ;
            var some_pitchspeed = attitude.pitchspeed() ;
            var some_yawspeed = attitude.yawspeed() ;
        }
        public static void onATTITUDE_QUATERNION(ATTITUDE_QUATERNION attitude_quaternion)
        {
            var some_time_boot_ms = attitude_quaternion.time_boot_ms() ;
            var some_q1 = attitude_quaternion.q1() ;
            var some_q2 = attitude_quaternion.q2() ;
            var some_q3 = attitude_quaternion.q3() ;
            var some_q4 = attitude_quaternion.q4() ;
            var some_rollspeed = attitude_quaternion.rollspeed() ;
            var some_pitchspeed = attitude_quaternion.pitchspeed() ;
            var some_yawspeed = attitude_quaternion.yawspeed() ;
        }
        public static void onLOCAL_POSITION_NED(LOCAL_POSITION_NED local_position_ned)
        {
            var some_time_boot_ms = local_position_ned.time_boot_ms() ;
            var some_x = local_position_ned.x() ;
            var some_y = local_position_ned.y() ;
            var some_z = local_position_ned.z() ;
            var some_vx = local_position_ned.vx() ;
            var some_vy = local_position_ned.vy() ;
            var some_vz = local_position_ned.vz() ;
        }
        public static void onGLOBAL_POSITION_INT(GLOBAL_POSITION_INT global_position_int)
        {
            var some_time_boot_ms = global_position_int.time_boot_ms() ;
            var some_lat = global_position_int.lat() ;
            var some_lon = global_position_int.lon() ;
            var some_alt = global_position_int.alt() ;
            var some_relative_alt = global_position_int.relative_alt() ;
            var some_vx = global_position_int.vx() ;
            var some_vy = global_position_int.vy() ;
            var some_vz = global_position_int.vz() ;
            var some_hdg = global_position_int.hdg() ;
        }
        public static void onRC_CHANNELS_SCALED(RC_CHANNELS_SCALED rc_channels_scaled)
        {
            var some_time_boot_ms = rc_channels_scaled.time_boot_ms() ;
            var some_port = rc_channels_scaled.port() ;
            var some_chan1_scaled = rc_channels_scaled.chan1_scaled() ;
            var some_chan2_scaled = rc_channels_scaled.chan2_scaled() ;
            var some_chan3_scaled = rc_channels_scaled.chan3_scaled() ;
            var some_chan4_scaled = rc_channels_scaled.chan4_scaled() ;
            var some_chan5_scaled = rc_channels_scaled.chan5_scaled() ;
            var some_chan6_scaled = rc_channels_scaled.chan6_scaled() ;
            var some_chan7_scaled = rc_channels_scaled.chan7_scaled() ;
            var some_chan8_scaled = rc_channels_scaled.chan8_scaled() ;
            var some_rssi = rc_channels_scaled.rssi() ;
        }
        public static void onRC_CHANNELS_RAW(RC_CHANNELS_RAW rc_channels_raw)
        {
            var some_time_boot_ms = rc_channels_raw.time_boot_ms() ;
            var some_port = rc_channels_raw.port() ;
            var some_chan1_raw = rc_channels_raw.chan1_raw() ;
            var some_chan2_raw = rc_channels_raw.chan2_raw() ;
            var some_chan3_raw = rc_channels_raw.chan3_raw() ;
            var some_chan4_raw = rc_channels_raw.chan4_raw() ;
            var some_chan5_raw = rc_channels_raw.chan5_raw() ;
            var some_chan6_raw = rc_channels_raw.chan6_raw() ;
            var some_chan7_raw = rc_channels_raw.chan7_raw() ;
            var some_chan8_raw = rc_channels_raw.chan8_raw() ;
            var some_rssi = rc_channels_raw.rssi() ;
        }
        public static void onSERVO_OUTPUT_RAW(SERVO_OUTPUT_RAW servo_output_raw)
        {
            var some_time_usec = servo_output_raw.time_usec() ;
            var some_port = servo_output_raw.port() ;
            var some_servo1_raw = servo_output_raw.servo1_raw() ;
            var some_servo2_raw = servo_output_raw.servo2_raw() ;
            var some_servo3_raw = servo_output_raw.servo3_raw() ;
            var some_servo4_raw = servo_output_raw.servo4_raw() ;
            var some_servo5_raw = servo_output_raw.servo5_raw() ;
            var some_servo6_raw = servo_output_raw.servo6_raw() ;
            var some_servo7_raw = servo_output_raw.servo7_raw() ;
            var some_servo8_raw = servo_output_raw.servo8_raw() ;
            {
                var item = servo_output_raw.servo9_raw();
                if(item.HasValue)
                    some_short = item.Value ;
            }
            {
                var item = servo_output_raw.servo10_raw();
                if(item.HasValue)
                    some_short = item.Value ;
            }
            {
                var item = servo_output_raw.servo11_raw();
                if(item.HasValue)
                    some_short = item.Value ;
            }
            {
                var item = servo_output_raw.servo12_raw();
                if(item.HasValue)
                    some_short = item.Value ;
            }
            {
                var item = servo_output_raw.servo13_raw();
                if(item.HasValue)
                    some_short = item.Value ;
            }
            {
                var item = servo_output_raw.servo14_raw();
                if(item.HasValue)
                    some_short = item.Value ;
            }
            {
                var item = servo_output_raw.servo15_raw();
                if(item.HasValue)
                    some_short = item.Value ;
            }
            {
                var item = servo_output_raw.servo16_raw();
                if(item.HasValue)
                    some_short = item.Value ;
            }
        }
        public static void onMISSION_REQUEST_PARTIAL_LIST(MISSION_REQUEST_PARTIAL_LIST mission_request_partial_list)
        {
            var some_target_system = mission_request_partial_list.target_system() ;
            var some_target_component = mission_request_partial_list.target_component() ;
            var some_start_index = mission_request_partial_list.start_index() ;
            var some_end_index = mission_request_partial_list.end_index() ;
            {
                var item = mission_request_partial_list.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onMISSION_WRITE_PARTIAL_LIST(MISSION_WRITE_PARTIAL_LIST mission_write_partial_list)
        {
            var some_target_system = mission_write_partial_list.target_system() ;
            var some_target_component = mission_write_partial_list.target_component() ;
            var some_start_index = mission_write_partial_list.start_index() ;
            var some_end_index = mission_write_partial_list.end_index() ;
            {
                var item = mission_write_partial_list.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onMISSION_ITEM(MISSION_ITEM mission_item)
        {
            var some_target_system = mission_item.target_system() ;
            var some_target_component = mission_item.target_component() ;
            var some_seq = mission_item.seq() ;
            {
                var item = mission_item.frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            {
                var item = mission_item.command();
                if(item.HasValue)
                    some_MAV_CMD = item.Value ;
            }
            var some_current = mission_item.current() ;
            var some_autocontinue = mission_item.autocontinue() ;
            var some_param1 = mission_item.param1() ;
            var some_param2 = mission_item.param2() ;
            var some_param3 = mission_item.param3() ;
            var some_param4 = mission_item.param4() ;
            var some_x = mission_item.x() ;
            var some_y = mission_item.y() ;
            var some_z = mission_item.z() ;
            {
                var item = mission_item.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onMISSION_REQUEST(MISSION_REQUEST mission_request)
        {
            var some_target_system = mission_request.target_system() ;
            var some_target_component = mission_request.target_component() ;
            var some_seq = mission_request.seq() ;
            {
                var item = mission_request.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onMISSION_SET_CURRENT(MISSION_SET_CURRENT mission_set_current)
        {
            var some_target_system = mission_set_current.target_system() ;
            var some_target_component = mission_set_current.target_component() ;
            var some_seq = mission_set_current.seq() ;
        }
        public static void onMISSION_CURRENT(MISSION_CURRENT mission_current)
        {
            var some_seq = mission_current.seq() ;
        }
        public static void onMISSION_REQUEST_LIST(MISSION_REQUEST_LIST mission_request_list)
        {
            var some_target_system = mission_request_list.target_system() ;
            var some_target_component = mission_request_list.target_component() ;
            {
                var item = mission_request_list.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onMISSION_COUNT(MISSION_COUNT mission_count)
        {
            var some_target_system = mission_count.target_system() ;
            var some_target_component = mission_count.target_component() ;
            var some_count = mission_count.count() ;
            {
                var item = mission_count.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onMISSION_CLEAR_ALL(MISSION_CLEAR_ALL mission_clear_all)
        {
            var some_target_system = mission_clear_all.target_system() ;
            var some_target_component = mission_clear_all.target_component() ;
            {
                var item = mission_clear_all.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onMISSION_ITEM_REACHED(MISSION_ITEM_REACHED mission_item_reached)
        {
            var some_seq = mission_item_reached.seq() ;
        }
        public static void onMISSION_ACK(MISSION_ACK mission_ack)
        {
            var some_target_system = mission_ack.target_system() ;
            var some_target_component = mission_ack.target_component() ;
            {
                var item = mission_ack.typE();
                if(item.HasValue)
                    some_MAV_MISSION_RESULT = item.Value ;
            }
            {
                var item = mission_ack.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onSET_GPS_GLOBAL_ORIGIN(SET_GPS_GLOBAL_ORIGIN set_gps_global_origin)
        {
            var some_target_system = set_gps_global_origin.target_system() ;
            var some_latitude = set_gps_global_origin.latitude() ;
            var some_longitude = set_gps_global_origin.longitude() ;
            var some_altitude = set_gps_global_origin.altitude() ;
            {
                var item = set_gps_global_origin.time_usec();
                if(item.HasValue)
                    some_long = item.Value ;
            }
        }
        public static void onGPS_GLOBAL_ORIGIN(GPS_GLOBAL_ORIGIN gps_global_origin)
        {
            var some_latitude = gps_global_origin.latitude() ;
            var some_longitude = gps_global_origin.longitude() ;
            var some_altitude = gps_global_origin.altitude() ;
            {
                var item = gps_global_origin.time_usec();
                if(item.HasValue)
                    some_long = item.Value ;
            }
        }
        public static void onPARAM_MAP_RC(PARAM_MAP_RC param_map_rc)
        {
            var some_target_system = param_map_rc.target_system() ;
            var some_target_component = param_map_rc.target_component() ;
            {
                var item = param_map_rc.param_id();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_param_index = param_map_rc.param_index() ;
            var some_parameter_rc_channel_index = param_map_rc.parameter_rc_channel_index() ;
            var some_param_value0 = param_map_rc.param_value0() ;
            var some_scale = param_map_rc.scale() ;
            var some_param_value_min = param_map_rc.param_value_min() ;
            var some_param_value_max = param_map_rc.param_value_max() ;
        }
        public static void onMISSION_REQUEST_INT(MISSION_REQUEST_INT mission_request_int)
        {
            var some_target_system = mission_request_int.target_system() ;
            var some_target_component = mission_request_int.target_component() ;
            var some_seq = mission_request_int.seq() ;
            {
                var item = mission_request_int.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onSAFETY_SET_ALLOWED_AREA(SAFETY_SET_ALLOWED_AREA safety_set_allowed_area)
        {
            var some_target_system = safety_set_allowed_area.target_system() ;
            var some_target_component = safety_set_allowed_area.target_component() ;
            {
                var item = safety_set_allowed_area.frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            var some_p1x = safety_set_allowed_area.p1x() ;
            var some_p1y = safety_set_allowed_area.p1y() ;
            var some_p1z = safety_set_allowed_area.p1z() ;
            var some_p2x = safety_set_allowed_area.p2x() ;
            var some_p2y = safety_set_allowed_area.p2y() ;
            var some_p2z = safety_set_allowed_area.p2z() ;
        }
        public static void onSAFETY_ALLOWED_AREA(SAFETY_ALLOWED_AREA safety_allowed_area)
        {
            {
                var item = safety_allowed_area.frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            var some_p1x = safety_allowed_area.p1x() ;
            var some_p1y = safety_allowed_area.p1y() ;
            var some_p1z = safety_allowed_area.p1z() ;
            var some_p2x = safety_allowed_area.p2x() ;
            var some_p2y = safety_allowed_area.p2y() ;
            var some_p2z = safety_allowed_area.p2z() ;
        }
        public static void onATTITUDE_QUATERNION_COV(ATTITUDE_QUATERNION_COV attitude_quaternion_cov)
        {
            var some_time_usec = attitude_quaternion_cov.time_usec() ;
            {
                var item = attitude_quaternion_cov.q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_rollspeed = attitude_quaternion_cov.rollspeed() ;
            var some_pitchspeed = attitude_quaternion_cov.pitchspeed() ;
            var some_yawspeed = attitude_quaternion_cov.yawspeed() ;
            {
                var item = attitude_quaternion_cov.covariance();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
        }
        public static void onNAV_CONTROLLER_OUTPUT(NAV_CONTROLLER_OUTPUT nav_controller_output)
        {
            var some_nav_roll = nav_controller_output.nav_roll() ;
            var some_nav_pitch = nav_controller_output.nav_pitch() ;
            var some_nav_bearing = nav_controller_output.nav_bearing() ;
            var some_target_bearing = nav_controller_output.target_bearing() ;
            var some_wp_dist = nav_controller_output.wp_dist() ;
            var some_alt_error = nav_controller_output.alt_error() ;
            var some_aspd_error = nav_controller_output.aspd_error() ;
            var some_xtrack_error = nav_controller_output.xtrack_error() ;
        }
        public static void onGLOBAL_POSITION_INT_COV(GLOBAL_POSITION_INT_COV global_position_int_cov)
        {
            var some_time_usec = global_position_int_cov.time_usec() ;
            {
                var item = global_position_int_cov.estimator_type();
                if(item.HasValue)
                    some_MAV_ESTIMATOR_TYPE = item.Value ;
            }
            var some_lat = global_position_int_cov.lat() ;
            var some_lon = global_position_int_cov.lon() ;
            var some_alt = global_position_int_cov.alt() ;
            var some_relative_alt = global_position_int_cov.relative_alt() ;
            var some_vx = global_position_int_cov.vx() ;
            var some_vy = global_position_int_cov.vy() ;
            var some_vz = global_position_int_cov.vz() ;
            {
                var item = global_position_int_cov.covariance();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
        }
        public static void onLOCAL_POSITION_NED_COV(LOCAL_POSITION_NED_COV local_position_ned_cov)
        {
            var some_time_usec = local_position_ned_cov.time_usec() ;
            {
                var item = local_position_ned_cov.estimator_type();
                if(item.HasValue)
                    some_MAV_ESTIMATOR_TYPE = item.Value ;
            }
            var some_x = local_position_ned_cov.x() ;
            var some_y = local_position_ned_cov.y() ;
            var some_z = local_position_ned_cov.z() ;
            var some_vx = local_position_ned_cov.vx() ;
            var some_vy = local_position_ned_cov.vy() ;
            var some_vz = local_position_ned_cov.vz() ;
            var some_ax = local_position_ned_cov.ax() ;
            var some_ay = local_position_ned_cov.ay() ;
            var some_az = local_position_ned_cov.az() ;
            {
                var item = local_position_ned_cov.covariance();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
        }
        public static void onRC_CHANNELS(RC_CHANNELS rc_channels)
        {
            var some_time_boot_ms = rc_channels.time_boot_ms() ;
            var some_chancount = rc_channels.chancount() ;
            var some_chan1_raw = rc_channels.chan1_raw() ;
            var some_chan2_raw = rc_channels.chan2_raw() ;
            var some_chan3_raw = rc_channels.chan3_raw() ;
            var some_chan4_raw = rc_channels.chan4_raw() ;
            var some_chan5_raw = rc_channels.chan5_raw() ;
            var some_chan6_raw = rc_channels.chan6_raw() ;
            var some_chan7_raw = rc_channels.chan7_raw() ;
            var some_chan8_raw = rc_channels.chan8_raw() ;
            var some_chan9_raw = rc_channels.chan9_raw() ;
            var some_chan10_raw = rc_channels.chan10_raw() ;
            var some_chan11_raw = rc_channels.chan11_raw() ;
            var some_chan12_raw = rc_channels.chan12_raw() ;
            var some_chan13_raw = rc_channels.chan13_raw() ;
            var some_chan14_raw = rc_channels.chan14_raw() ;
            var some_chan15_raw = rc_channels.chan15_raw() ;
            var some_chan16_raw = rc_channels.chan16_raw() ;
            var some_chan17_raw = rc_channels.chan17_raw() ;
            var some_chan18_raw = rc_channels.chan18_raw() ;
            var some_rssi = rc_channels.rssi() ;
        }
        public static void onREQUEST_DATA_STREAM(REQUEST_DATA_STREAM request_data_stream)
        {
            var some_target_system = request_data_stream.target_system() ;
            var some_target_component = request_data_stream.target_component() ;
            var some_req_stream_id = request_data_stream.req_stream_id() ;
            var some_req_message_rate = request_data_stream.req_message_rate() ;
            var some_start_stop = request_data_stream.start_stop() ;
        }
        public static void onDATA_STREAM(DATA_STREAM data_stream)
        {
            var some_stream_id = data_stream.stream_id() ;
            var some_message_rate = data_stream.message_rate() ;
            var some_on_off = data_stream.on_off() ;
        }
        public static void onMANUAL_CONTROL(MANUAL_CONTROL manual_control)
        {
            var some_target = manual_control.target() ;
            var some_x = manual_control.x() ;
            var some_y = manual_control.y() ;
            var some_z = manual_control.z() ;
            var some_r = manual_control.r() ;
            var some_buttons = manual_control.buttons() ;
        }
        public static void onRC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE rc_channels_override)
        {
            var some_target_system = rc_channels_override.target_system() ;
            var some_target_component = rc_channels_override.target_component() ;
            var some_chan1_raw = rc_channels_override.chan1_raw() ;
            var some_chan2_raw = rc_channels_override.chan2_raw() ;
            var some_chan3_raw = rc_channels_override.chan3_raw() ;
            var some_chan4_raw = rc_channels_override.chan4_raw() ;
            var some_chan5_raw = rc_channels_override.chan5_raw() ;
            var some_chan6_raw = rc_channels_override.chan6_raw() ;
            var some_chan7_raw = rc_channels_override.chan7_raw() ;
            var some_chan8_raw = rc_channels_override.chan8_raw() ;
        }
        public static void onMISSION_ITEM_INT(MISSION_ITEM_INT mission_item_int)
        {
            var some_target_system = mission_item_int.target_system() ;
            var some_target_component = mission_item_int.target_component() ;
            var some_seq = mission_item_int.seq() ;
            {
                var item = mission_item_int.frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            {
                var item = mission_item_int.command();
                if(item.HasValue)
                    some_MAV_CMD = item.Value ;
            }
            var some_current = mission_item_int.current() ;
            var some_autocontinue = mission_item_int.autocontinue() ;
            var some_param1 = mission_item_int.param1() ;
            var some_param2 = mission_item_int.param2() ;
            var some_param3 = mission_item_int.param3() ;
            var some_param4 = mission_item_int.param4() ;
            var some_x = mission_item_int.x() ;
            var some_y = mission_item_int.y() ;
            var some_z = mission_item_int.z() ;
            {
                var item = mission_item_int.mission_type();
                if(item.HasValue)
                    some_MAV_MISSION_TYPE = item.Value ;
            }
        }
        public static void onVFR_HUD(VFR_HUD vfr_hud)
        {
            var some_airspeed = vfr_hud.airspeed() ;
            var some_groundspeed = vfr_hud.groundspeed() ;
            var some_heading = vfr_hud.heading() ;
            var some_throttle = vfr_hud.throttle() ;
            var some_alt = vfr_hud.alt() ;
            var some_climb = vfr_hud.climb() ;
        }
        public static void onCOMMAND_INT(COMMAND_INT command_int)
        {
            var some_target_system = command_int.target_system() ;
            var some_target_component = command_int.target_component() ;
            {
                var item = command_int.frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            {
                var item = command_int.command();
                if(item.HasValue)
                    some_MAV_CMD = item.Value ;
            }
            var some_current = command_int.current() ;
            var some_autocontinue = command_int.autocontinue() ;
            var some_param1 = command_int.param1() ;
            var some_param2 = command_int.param2() ;
            var some_param3 = command_int.param3() ;
            var some_param4 = command_int.param4() ;
            var some_x = command_int.x() ;
            var some_y = command_int.y() ;
            var some_z = command_int.z() ;
        }
        public static void onCOMMAND_LONG(COMMAND_LONG command_long)
        {
            var some_target_system = command_long.target_system() ;
            var some_target_component = command_long.target_component() ;
            {
                var item = command_long.command();
                if(item.HasValue)
                    some_MAV_CMD = item.Value ;
            }
            var some_confirmation = command_long.confirmation() ;
            var some_param1 = command_long.param1() ;
            var some_param2 = command_long.param2() ;
            var some_param3 = command_long.param3() ;
            var some_param4 = command_long.param4() ;
            var some_param5 = command_long.param5() ;
            var some_param6 = command_long.param6() ;
            var some_param7 = command_long.param7() ;
        }
        public static void onCOMMAND_ACK(COMMAND_ACK command_ack)
        {
            {
                var item = command_ack.command();
                if(item.HasValue)
                    some_MAV_CMD = item.Value ;
            }
            {
                var item = command_ack.result();
                if(item.HasValue)
                    some_MAV_RESULT = item.Value ;
            }
            {
                var item = command_ack.progress();
                if(item.HasValue)
                    some_sbyte = item.Value ;
            }
            {
                var item = command_ack.result_param2();
                if(item.HasValue)
                    some_int = item.Value ;
            }
            {
                var item = command_ack.target_system();
                if(item.HasValue)
                    some_sbyte = item.Value ;
            }
            {
                var item = command_ack.target_component();
                if(item.HasValue)
                    some_sbyte = item.Value ;
            }
        }
        public static void onMANUAL_SETPOINT(MANUAL_SETPOINT manual_setpoint)
        {
            var some_time_boot_ms = manual_setpoint.time_boot_ms() ;
            var some_roll = manual_setpoint.roll() ;
            var some_pitch = manual_setpoint.pitch() ;
            var some_yaw = manual_setpoint.yaw() ;
            var some_thrust = manual_setpoint.thrust() ;
            var some_mode_switch = manual_setpoint.mode_switch() ;
            var some_manual_override_switch = manual_setpoint.manual_override_switch() ;
        }
        public static void onSET_ATTITUDE_TARGET(SET_ATTITUDE_TARGET set_attitude_target)
        {
            var some_time_boot_ms = set_attitude_target.time_boot_ms() ;
            var some_target_system = set_attitude_target.target_system() ;
            var some_target_component = set_attitude_target.target_component() ;
            var some_type_mask = set_attitude_target.type_mask() ;
            {
                var item = set_attitude_target.q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_body_roll_rate = set_attitude_target.body_roll_rate() ;
            var some_body_pitch_rate = set_attitude_target.body_pitch_rate() ;
            var some_body_yaw_rate = set_attitude_target.body_yaw_rate() ;
            var some_thrust = set_attitude_target.thrust() ;
        }
        public static void onATTITUDE_TARGET(ATTITUDE_TARGET attitude_target)
        {
            var some_time_boot_ms = attitude_target.time_boot_ms() ;
            var some_type_mask = attitude_target.type_mask() ;
            {
                var item = attitude_target.q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_body_roll_rate = attitude_target.body_roll_rate() ;
            var some_body_pitch_rate = attitude_target.body_pitch_rate() ;
            var some_body_yaw_rate = attitude_target.body_yaw_rate() ;
            var some_thrust = attitude_target.thrust() ;
        }
        public static void onSET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED set_position_target_local_ned)
        {
            var some_time_boot_ms = set_position_target_local_ned.time_boot_ms() ;
            var some_target_system = set_position_target_local_ned.target_system() ;
            var some_target_component = set_position_target_local_ned.target_component() ;
            {
                var item = set_position_target_local_ned.coordinate_frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            var some_type_mask = set_position_target_local_ned.type_mask() ;
            var some_x = set_position_target_local_ned.x() ;
            var some_y = set_position_target_local_ned.y() ;
            var some_z = set_position_target_local_ned.z() ;
            var some_vx = set_position_target_local_ned.vx() ;
            var some_vy = set_position_target_local_ned.vy() ;
            var some_vz = set_position_target_local_ned.vz() ;
            var some_afx = set_position_target_local_ned.afx() ;
            var some_afy = set_position_target_local_ned.afy() ;
            var some_afz = set_position_target_local_ned.afz() ;
            var some_yaw = set_position_target_local_ned.yaw() ;
            var some_yaw_rate = set_position_target_local_ned.yaw_rate() ;
        }
        public static void onPOSITION_TARGET_LOCAL_NED(POSITION_TARGET_LOCAL_NED position_target_local_ned)
        {
            var some_time_boot_ms = position_target_local_ned.time_boot_ms() ;
            {
                var item = position_target_local_ned.coordinate_frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            var some_type_mask = position_target_local_ned.type_mask() ;
            var some_x = position_target_local_ned.x() ;
            var some_y = position_target_local_ned.y() ;
            var some_z = position_target_local_ned.z() ;
            var some_vx = position_target_local_ned.vx() ;
            var some_vy = position_target_local_ned.vy() ;
            var some_vz = position_target_local_ned.vz() ;
            var some_afx = position_target_local_ned.afx() ;
            var some_afy = position_target_local_ned.afy() ;
            var some_afz = position_target_local_ned.afz() ;
            var some_yaw = position_target_local_ned.yaw() ;
            var some_yaw_rate = position_target_local_ned.yaw_rate() ;
        }
        public static void onSET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT set_position_target_global_int)
        {
            var some_time_boot_ms = set_position_target_global_int.time_boot_ms() ;
            var some_target_system = set_position_target_global_int.target_system() ;
            var some_target_component = set_position_target_global_int.target_component() ;
            {
                var item = set_position_target_global_int.coordinate_frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            var some_type_mask = set_position_target_global_int.type_mask() ;
            var some_lat_int = set_position_target_global_int.lat_int() ;
            var some_lon_int = set_position_target_global_int.lon_int() ;
            var some_alt = set_position_target_global_int.alt() ;
            var some_vx = set_position_target_global_int.vx() ;
            var some_vy = set_position_target_global_int.vy() ;
            var some_vz = set_position_target_global_int.vz() ;
            var some_afx = set_position_target_global_int.afx() ;
            var some_afy = set_position_target_global_int.afy() ;
            var some_afz = set_position_target_global_int.afz() ;
            var some_yaw = set_position_target_global_int.yaw() ;
            var some_yaw_rate = set_position_target_global_int.yaw_rate() ;
        }
        public static void onPOSITION_TARGET_GLOBAL_INT(POSITION_TARGET_GLOBAL_INT position_target_global_int)
        {
            var some_time_boot_ms = position_target_global_int.time_boot_ms() ;
            {
                var item = position_target_global_int.coordinate_frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            var some_type_mask = position_target_global_int.type_mask() ;
            var some_lat_int = position_target_global_int.lat_int() ;
            var some_lon_int = position_target_global_int.lon_int() ;
            var some_alt = position_target_global_int.alt() ;
            var some_vx = position_target_global_int.vx() ;
            var some_vy = position_target_global_int.vy() ;
            var some_vz = position_target_global_int.vz() ;
            var some_afx = position_target_global_int.afx() ;
            var some_afy = position_target_global_int.afy() ;
            var some_afz = position_target_global_int.afz() ;
            var some_yaw = position_target_global_int.yaw() ;
            var some_yaw_rate = position_target_global_int.yaw_rate() ;
        }
        public static void onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET local_position_ned_system_global_offset)
        {
            var some_time_boot_ms = local_position_ned_system_global_offset.time_boot_ms() ;
            var some_x = local_position_ned_system_global_offset.x() ;
            var some_y = local_position_ned_system_global_offset.y() ;
            var some_z = local_position_ned_system_global_offset.z() ;
            var some_roll = local_position_ned_system_global_offset.roll() ;
            var some_pitch = local_position_ned_system_global_offset.pitch() ;
            var some_yaw = local_position_ned_system_global_offset.yaw() ;
        }
        public static void onHIL_STATE(HIL_STATE hil_state)
        {
            var some_time_usec = hil_state.time_usec() ;
            var some_roll = hil_state.roll() ;
            var some_pitch = hil_state.pitch() ;
            var some_yaw = hil_state.yaw() ;
            var some_rollspeed = hil_state.rollspeed() ;
            var some_pitchspeed = hil_state.pitchspeed() ;
            var some_yawspeed = hil_state.yawspeed() ;
            var some_lat = hil_state.lat() ;
            var some_lon = hil_state.lon() ;
            var some_alt = hil_state.alt() ;
            var some_vx = hil_state.vx() ;
            var some_vy = hil_state.vy() ;
            var some_vz = hil_state.vz() ;
            var some_xacc = hil_state.xacc() ;
            var some_yacc = hil_state.yacc() ;
            var some_zacc = hil_state.zacc() ;
        }
        public static void onHIL_CONTROLS(HIL_CONTROLS hil_controls)
        {
            var some_time_usec = hil_controls.time_usec() ;
            var some_roll_ailerons = hil_controls.roll_ailerons() ;
            var some_pitch_elevator = hil_controls.pitch_elevator() ;
            var some_yaw_rudder = hil_controls.yaw_rudder() ;
            var some_throttle = hil_controls.throttle() ;
            var some_aux1 = hil_controls.aux1() ;
            var some_aux2 = hil_controls.aux2() ;
            var some_aux3 = hil_controls.aux3() ;
            var some_aux4 = hil_controls.aux4() ;
            {
                var item = hil_controls.mode();
                if(item.HasValue)
                    some_MAV_MODE = item.Value ;
            }
            var some_nav_mode = hil_controls.nav_mode() ;
        }
        public static void onHIL_RC_INPUTS_RAW(HIL_RC_INPUTS_RAW hil_rc_inputs_raw)
        {
            var some_time_usec = hil_rc_inputs_raw.time_usec() ;
            var some_chan1_raw = hil_rc_inputs_raw.chan1_raw() ;
            var some_chan2_raw = hil_rc_inputs_raw.chan2_raw() ;
            var some_chan3_raw = hil_rc_inputs_raw.chan3_raw() ;
            var some_chan4_raw = hil_rc_inputs_raw.chan4_raw() ;
            var some_chan5_raw = hil_rc_inputs_raw.chan5_raw() ;
            var some_chan6_raw = hil_rc_inputs_raw.chan6_raw() ;
            var some_chan7_raw = hil_rc_inputs_raw.chan7_raw() ;
            var some_chan8_raw = hil_rc_inputs_raw.chan8_raw() ;
            var some_chan9_raw = hil_rc_inputs_raw.chan9_raw() ;
            var some_chan10_raw = hil_rc_inputs_raw.chan10_raw() ;
            var some_chan11_raw = hil_rc_inputs_raw.chan11_raw() ;
            var some_chan12_raw = hil_rc_inputs_raw.chan12_raw() ;
            var some_rssi = hil_rc_inputs_raw.rssi() ;
        }
        public static void onHIL_ACTUATOR_CONTROLS(HIL_ACTUATOR_CONTROLS hil_actuator_controls)
        {
            var some_time_usec = hil_actuator_controls.time_usec() ;
            {
                var item = hil_actuator_controls.controls();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            {
                var item = hil_actuator_controls.mode();
                if(item.HasValue)
                    some_MAV_MODE = item.Value ;
            }
            var some_flags = hil_actuator_controls.flags() ;
        }
        public static void onOPTICAL_FLOW(OPTICAL_FLOW optical_flow)
        {
            var some_time_usec = optical_flow.time_usec() ;
            var some_sensor_id = optical_flow.sensor_id() ;
            var some_flow_x = optical_flow.flow_x() ;
            var some_flow_y = optical_flow.flow_y() ;
            var some_flow_comp_m_x = optical_flow.flow_comp_m_x() ;
            var some_flow_comp_m_y = optical_flow.flow_comp_m_y() ;
            var some_quality = optical_flow.quality() ;
            var some_ground_distance = optical_flow.ground_distance() ;
            {
                var item = optical_flow.flow_rate_x();
                if(item.HasValue)
                    some_float = item.Value ;
            }
            {
                var item = optical_flow.flow_rate_y();
                if(item.HasValue)
                    some_float = item.Value ;
            }
        }
        public static void onGLOBAL_VISION_POSITION_ESTIMATE(GLOBAL_VISION_POSITION_ESTIMATE global_vision_position_estimate)
        {
            var some_usec = global_vision_position_estimate.usec() ;
            var some_x = global_vision_position_estimate.x() ;
            var some_y = global_vision_position_estimate.y() ;
            var some_z = global_vision_position_estimate.z() ;
            var some_roll = global_vision_position_estimate.roll() ;
            var some_pitch = global_vision_position_estimate.pitch() ;
            var some_yaw = global_vision_position_estimate.yaw() ;
        }
        public static void onVISION_POSITION_ESTIMATE(VISION_POSITION_ESTIMATE vision_position_estimate)
        {
            var some_usec = vision_position_estimate.usec() ;
            var some_x = vision_position_estimate.x() ;
            var some_y = vision_position_estimate.y() ;
            var some_z = vision_position_estimate.z() ;
            var some_roll = vision_position_estimate.roll() ;
            var some_pitch = vision_position_estimate.pitch() ;
            var some_yaw = vision_position_estimate.yaw() ;
        }
        public static void onSwitchModeCommand() {}
        public static void onPING33(PING33 ping33)
        {
            {
                var item = ping33.testBOOL();
                if(item.HasValue)
                    some_bool = item.Value ;
            }
            {
                var item = ping33.seq();
                if(item.HasValue)
                    some_long = item.Value ;
            }
            var some_field = ping33.field() ;
            {
                var _fld = ping33.field1().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = fld.d0();
                    some_int = PING33.field1_.d0_max;
                    some_int = PING33.field1_.d1;
                    some_int = PING33.field1_.d2;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_field1 = fld.get(d0, d1, d2);
                    });
                }
            }
            {
                var _fld = ping33.field12();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_field12 = fld.get(d0, d1, d2);
                    }
                                 );
                }
            }
            {
                var _fld = ping33.field13();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_int = item ;
                    }
                                 );
                }
            }
            PING33.TTTT_.enumerate(ping33, (item, d0, d1, d2) =>
            {
                var some_TTTT = ping33.TTTT(d0, d1, d2) ;
            }
                                  );
            {
                var item = ping33.WWWWWWWW();
                if(item.HasValue)
                    some_int = item.Value ;
            }
            var some_testBOOL2 = ping33.testBOOL2() ;
            var some_testBOOL3 = ping33.testBOOL3() ;
            var some_bit_field = ping33.bit_field() ;
            {
                var item = ping33.bit_field2();
                if(item.HasValue)
                    some_sbyte = item.Value ;
            }
            {
                var _fld = ping33.Field_Bits();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_Field_Bits = fld.get(d0, d1, d2);
                    }
                                 );
                }
            }
            {
                var _fld = ping33.SparseFixAllBits().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = fld.d0();
                    some_int = PING33.SparseFixAllBits_.d0_max;
                    some_int = PING33.SparseFixAllBits_.d1;
                    some_int = PING33.SparseFixAllBits_.d2;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_sbyte = item ;
                    });
                }
            }
            {
                var _fld = ping33.FixAllBits().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = fld.d0();
                    some_int = PING33.FixAllBits_.d0_max;
                    some_int = PING33.FixAllBits_.d1;
                    some_int = PING33.FixAllBits_.d2;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_FixAllBits = fld.get(d0, d1, d2);
                    });
                }
            }
            {
                var _fld = ping33.VarAllBits().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = fld.d0();
                    some_int = PING33.VarAllBits_.d0_max;
                    some_int = PING33.VarAllBits_.d1;
                    some_int = fld.d2();
                    some_int = PING33.VarAllBits_.d2_max;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_VarAllBits = fld.get(d0, d1, d2);
                    });
                }
            }
            {
                var _fld = ping33.SparseVarAllBits().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = fld.d0();
                    some_int = PING33.SparseVarAllBits_.d0_max;
                    some_int = PING33.SparseVarAllBits_.d1;
                    some_int = fld.d2();
                    some_int = PING33.SparseVarAllBits_.d2_max;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_sbyte = item ;
                    });
                }
            }
            {
                var _fld = ping33.VarEachBits().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = fld.d0();
                    some_int = PING33.VarEachBits_.d0_max;
                    some_int = PING33.VarEachBits_.d1;
                    some_int = PING33.VarEachBits_.d2;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_VarEachBits = fld.get(d0, d1, d2);
                    });
                }
            }
            {
                var _fld = ping33.SparsVarEachBits().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = fld.d0();
                    some_int = PING33.SparsVarEachBits_.d0_max;
                    some_int = PING33.SparsVarEachBits_.d1;
                    some_int = PING33.SparsVarEachBits_.d2;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_short = item ;
                    });
                }
            }
            {
                var item = ping33.testBOOLX();
                if(item.HasValue)
                    some_bool = item.Value ;
            }
            {
                var item = ping33.testBOOL2X();
                if(item.HasValue)
                    some_bool = item.Value ;
            }
            {
                var item = ping33.testBOOL3X();
                if(item.HasValue)
                    some_bool = item.Value ;
            }
            {
                var item = ping33.MMMMMM();
                if(item.HasValue)
                    some_MAV_MODE = item.Value ;
            }
            {
                var _fld = ping33.field44().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = PING33.field44_.d0;
                    some_int = PING33.field44_.d1;
                    some_int = fld.d2();
                    some_int = PING33.field44_.d2_max;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_field44 = fld.get(d0, d1, d2);
                    });
                }
            }
            {
                var _fld = ping33.field634().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = PING33.field634_.d0;
                    some_int = PING33.field634_.d1;
                    some_int = fld.d2();
                    some_int = PING33.field634_.d2_max;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_field634 = fld.get(d0, d1, d2);
                    });
                }
            }
            {
                var _fld = ping33.field33344().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = PING33.field33344_.d0;
                    some_int = PING33.field33344_.d1;
                    some_int = fld.d2();
                    some_int = PING33.field33344_.d2_max;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_int = item ;
                    });
                }
            }
            {
                var _fld = ping33.field333634().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = PING33.field333634_.d0;
                    some_int = PING33.field333634_.d1;
                    some_int = fld.d2();
                    some_int = PING33.field333634_.d2_max;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_int = item ;
                    });
                }
            }
            {
                var _fld = ping33.field__();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_int = item ;
                    }
                                 );
                }
            }
            PING33.field6_.enumerate(ping33, (item, d0, d1, d2) =>
            {
                var some_field6 = ping33.field6(d0, d1, d2) ;
            }
                                    );
            {
                var _fld = ping33.field63().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = PING33.field63_.d0;
                    some_int = PING33.field63_.d1;
                    some_int = fld.d2();
                    some_int = PING33.field63_.d2_max;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        var some_field63 = fld.get(d0, d1, d2);
                    });
                }
            }
            {
                var _fld = ping33.uid2();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0) =>
                    {
                        some_sbyte = item ;
                    }
                                 );
                }
            }
            {
                var _fld = ping33.field2();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_int = item ;
                    }
                                 );
                }
            }
            {
                var _fld = ping33.field4();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0, d1, d2) =>
                    {
                        some_int = item ;
                    }
                                 );
                }
            }
            {
                var item = ping33.stringtest1();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var _fld = ping33.stringtest2().Field() ;
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    some_int = PING33.stringtest2_.d0;
                    some_int = PING33.stringtest2_.d1;
                    some_int = fld.d2();
                    some_int = PING33.stringtest2_.d2_max;
                    fld.enumerate((item, d0, d1, d2) => {some_string = item.get();});
                }
            }
            {
                var item = ping33.stringtest3();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = ping33.stringtest4();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void fill(PING33 ping33)
        {
            ping33.testBOOL(some_bool);
            ping33.seq(some_long);
            ping33.field(some_long);
            {
                var init = ping33.field1().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.field1().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0 ; d0 < fld.d0(); d0 ++)
                            for(int d1 = 0; d1 < PING33.field1_.d1; d1++)
                                for(int d2 = 0; d2 < PING33.field1_.d2; d2++)
                                {
                                    fld.set(some_int, d0, d1, d2) ;
                                }
                    }
                }
            }
            for(int d0 = 0; d0 < PING33.field12_.d0; d0++)
                for(int d1 = 0; d1 < PING33.field12_.d1; d1++)
                    for(int d2 = 0; d2 < PING33.field12_.d2; d2++)
                    {
                        ping33.field12(some_int, d0, d1, d2) ;
                    }
            for(int d0 = 0; d0 < PING33.field13_.d0; d0++)
                for(int d1 = 0; d1 < PING33.field13_.d1; d1++)
                    for(int d2 = 0; d2 < PING33.field13_.d2; d2++)
                    {
                        ping33.field13(some_int, d0, d1, d2);
                    }
            PING33.TTTT_.enumerate(ping33, (item, d0, d1, d2) =>
            {
                ping33.TTTT(some_int, d0, d1, d2);
            }
                                  );
            ping33.WWWWWWWW(some_int);
            ping33.testBOOL2(some_bool);
            ping33.testBOOL3(some_bool);
            ping33.bit_field(some_sbyte);
            ping33.bit_field2(some_sbyte);
            for(int d0 = 0; d0 < PING33.Field_Bits_.d0; d0++)
                for(int d1 = 0; d1 < PING33.Field_Bits_.d1; d1++)
                    for(int d2 = 0; d2 < PING33.Field_Bits_.d2; d2++)
                    {
                        ping33.Field_Bits(some_sbyte, d0, d1, d2) ;
                    }
            {
                var init = ping33.SparseFixAllBits().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.SparseFixAllBits().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0 ; d0 < fld.d0(); d0 ++)
                            for(int d1 = 0; d1 < PING33.SparseFixAllBits_.d1; d1++)
                                for(int d2 = 0; d2 < PING33.SparseFixAllBits_.d2; d2++)
                                {
                                    fld.set(some_sbyte, d0, d1, d2);
                                }
                    }
                }
            }
            {
                var init = ping33.FixAllBits().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.FixAllBits().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0 ; d0 < fld.d0(); d0 ++)
                            for(int d1 = 0; d1 < PING33.FixAllBits_.d1; d1++)
                                for(int d2 = 0; d2 < PING33.FixAllBits_.d2; d2++)
                                {
                                    fld.set(some_sbyte, d0, d1, d2) ;
                                }
                    }
                }
            }
            {
                var init = ping33.VarAllBits().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int, some_int);
                    var _fld = ping33.VarAllBits().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0 ; d0 < fld.d0(); d0 ++)
                            for(int d1 = 0; d1 < PING33.VarAllBits_.d1; d1++)
                                for(int d2 = 0 ; d2 < fld.d2(); d2 ++)
                                {
                                    fld.set(some_sbyte, d0, d1, d2) ;
                                }
                    }
                }
            }
            {
                var init = ping33.SparseVarAllBits().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int, some_int);
                    var _fld = ping33.SparseVarAllBits().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0 ; d0 < fld.d0(); d0 ++)
                            for(int d1 = 0; d1 < PING33.SparseVarAllBits_.d1; d1++)
                                for(int d2 = 0 ; d2 < fld.d2(); d2 ++)
                                {
                                    fld.set(some_sbyte, d0, d1, d2);
                                }
                    }
                }
            }
            {
                var init = ping33.VarEachBits().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.VarEachBits().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0 ; d0 < fld.d0(); d0 ++)
                            for(int d1 = 0; d1 < PING33.VarEachBits_.d1; d1++)
                                for(int d2 = 0; d2 < PING33.VarEachBits_.d2; d2++)
                                {
                                    fld.set(some_sbyte, d0, d1, d2) ;
                                }
                    }
                }
            }
            {
                var init = ping33.SparsVarEachBits().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.SparsVarEachBits().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0 ; d0 < fld.d0(); d0 ++)
                            for(int d1 = 0; d1 < PING33.SparsVarEachBits_.d1; d1++)
                                for(int d2 = 0; d2 < PING33.SparsVarEachBits_.d2; d2++)
                                {
                                    fld.set(some_short, d0, d1, d2);
                                }
                    }
                }
            }
            ping33.testBOOLX(some_bool);
            ping33.testBOOL2X(some_bool);
            ping33.testBOOL3X(some_bool);
            ping33.MMMMMM(some_MAV_MODE);
            {
                var init = ping33.field44().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.field44().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0; d0 < PING33.field44_.d0; d0++)
                            for(int d1 = 0; d1 < PING33.field44_.d1; d1++)
                                for(int d2 = 0 ; d2 < fld.d2(); d2 ++)
                                {
                                    fld.set(some_int, d0, d1, d2) ;
                                }
                    }
                }
            }
            {
                var init = ping33.field634().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.field634().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0; d0 < PING33.field634_.d0; d0++)
                            for(int d1 = 0; d1 < PING33.field634_.d1; d1++)
                                for(int d2 = 0 ; d2 < fld.d2(); d2 ++)
                                {
                                    fld.set(some_int, d0, d1, d2) ;
                                }
                    }
                }
            }
            {
                var init = ping33.field33344().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.field33344().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0; d0 < PING33.field33344_.d0; d0++)
                            for(int d1 = 0; d1 < PING33.field33344_.d1; d1++)
                                for(int d2 = 0 ; d2 < fld.d2(); d2 ++)
                                {
                                    fld.set(some_int, d0, d1, d2);
                                }
                    }
                }
            }
            {
                var init = ping33.field333634().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.field333634().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0; d0 < PING33.field333634_.d0; d0++)
                            for(int d1 = 0; d1 < PING33.field333634_.d1; d1++)
                                for(int d2 = 0 ; d2 < fld.d2(); d2 ++)
                                {
                                    fld.set(some_int, d0, d1, d2);
                                }
                    }
                }
            }
            for(int d0 = 0; d0 < PING33.field___.d0; d0++)
                for(int d1 = 0; d1 < PING33.field___.d1; d1++)
                    for(int d2 = 0; d2 < PING33.field___.d2; d2++)
                    {
                        ping33.field__(some_int, d0, d1, d2);
                    }
            PING33.field6_.enumerate(ping33, (item, d0, d1, d2) =>
            {
                ping33.field6(some_int, d0, d1, d2);
            }
                                    );
            {
                var init = ping33.field63().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.field63().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0; d0 < PING33.field63_.d0; d0++)
                            for(int d1 = 0; d1 < PING33.field63_.d1; d1++)
                                for(int d2 = 0 ; d2 < fld.d2(); d2 ++)
                                {
                                    fld.set(some_int, d0, d1, d2) ;
                                }
                    }
                }
            }
            for(int d0 = 0; d0 < PING33.uid2_.d0; d0++)
            {
                ping33.uid2(some_sbyte, d0);
            }
            for(int d0 = 0; d0 < PING33.field2_.d0; d0++)
                for(int d1 = 0; d1 < PING33.field2_.d1; d1++)
                    for(int d2 = 0; d2 < PING33.field2_.d2; d2++)
                    {
                        ping33.field2(some_int, d0, d1, d2);
                    }
            for(int d0 = 0; d0 < PING33.field4_.d0; d0++)
                for(int d1 = 0; d1 < PING33.field4_.d1; d1++)
                    for(int d2 = 0; d2 < PING33.field4_.d2; d2++)
                    {
                        ping33.field4(some_int, d0, d1, d2);
                    }
            ping33.stringtest1(some_string, null);
            {
                var init = ping33.stringtest2().Initializer();
                if(init.HasValue)
                {
                    init.Value.init(some_int);
                    var _fld = ping33.stringtest2().Field();
                    if(_fld.HasValue)
                    {
                        var fld = _fld.Value;
                        for(int d0 = 0; d0 < PING33.stringtest2_.d0; d0++)
                            for(int d1 = 0; d1 < PING33.stringtest2_.d1; d1++)
                                for(int d2 = 0 ; d2 < fld.d2(); d2 ++)
                                {
                                    fld.set(some_string, d0, d1, d2, null);
                                }
                    }
                }
            }
            ping33.stringtest3(some_string, null);
            ping33.stringtest4(some_string, null);
        }
        public static void onVISION_SPEED_ESTIMATE(VISION_SPEED_ESTIMATE vision_speed_estimate)
        {
            var some_usec = vision_speed_estimate.usec() ;
            var some_x = vision_speed_estimate.x() ;
            var some_y = vision_speed_estimate.y() ;
            var some_z = vision_speed_estimate.z() ;
        }
        public static void fill(VISION_SPEED_ESTIMATE vision_speed_estimate)
        {
            vision_speed_estimate.usec(some_long);
            vision_speed_estimate.x(some_float);
            vision_speed_estimate.y(some_float);
            vision_speed_estimate.z(some_float);
        }
        public static void onVICON_POSITION_ESTIMATE(VICON_POSITION_ESTIMATE vicon_position_estimate)
        {
            var some_usec = vicon_position_estimate.usec() ;
            var some_x = vicon_position_estimate.x() ;
            var some_y = vicon_position_estimate.y() ;
            var some_z = vicon_position_estimate.z() ;
            var some_roll = vicon_position_estimate.roll() ;
            var some_pitch = vicon_position_estimate.pitch() ;
            var some_yaw = vicon_position_estimate.yaw() ;
        }
        public static void fill(VICON_POSITION_ESTIMATE vicon_position_estimate)
        {
            vicon_position_estimate.usec(some_long);
            vicon_position_estimate.x(some_float);
            vicon_position_estimate.y(some_float);
            vicon_position_estimate.z(some_float);
            vicon_position_estimate.roll(some_float);
            vicon_position_estimate.pitch(some_float);
            vicon_position_estimate.yaw(some_float);
        }
        public static void onHIGHRES_IMU(HIGHRES_IMU highres_imu)
        {
            var some_time_usec = highres_imu.time_usec() ;
            var some_xacc = highres_imu.xacc() ;
            var some_yacc = highres_imu.yacc() ;
            var some_zacc = highres_imu.zacc() ;
            var some_xgyro = highres_imu.xgyro() ;
            var some_ygyro = highres_imu.ygyro() ;
            var some_zgyro = highres_imu.zgyro() ;
            var some_xmag = highres_imu.xmag() ;
            var some_ymag = highres_imu.ymag() ;
            var some_zmag = highres_imu.zmag() ;
            var some_abs_pressure = highres_imu.abs_pressure() ;
            var some_diff_pressure = highres_imu.diff_pressure() ;
            var some_pressure_alt = highres_imu.pressure_alt() ;
            var some_temperature = highres_imu.temperature() ;
            var some_fields_updated = highres_imu.fields_updated() ;
        }
        public static void fill(HIGHRES_IMU highres_imu)
        {
            highres_imu.time_usec(some_long);
            highres_imu.xacc(some_float);
            highres_imu.yacc(some_float);
            highres_imu.zacc(some_float);
            highres_imu.xgyro(some_float);
            highres_imu.ygyro(some_float);
            highres_imu.zgyro(some_float);
            highres_imu.xmag(some_float);
            highres_imu.ymag(some_float);
            highres_imu.zmag(some_float);
            highres_imu.abs_pressure(some_float);
            highres_imu.diff_pressure(some_float);
            highres_imu.pressure_alt(some_float);
            highres_imu.temperature(some_float);
            highres_imu.fields_updated(some_short);
        }
        public static void onOPTICAL_FLOW_RAD(OPTICAL_FLOW_RAD optical_flow_rad)
        {
            var some_time_usec = optical_flow_rad.time_usec() ;
            var some_sensor_id = optical_flow_rad.sensor_id() ;
            var some_integration_time_us = optical_flow_rad.integration_time_us() ;
            var some_integrated_x = optical_flow_rad.integrated_x() ;
            var some_integrated_y = optical_flow_rad.integrated_y() ;
            var some_integrated_xgyro = optical_flow_rad.integrated_xgyro() ;
            var some_integrated_ygyro = optical_flow_rad.integrated_ygyro() ;
            var some_integrated_zgyro = optical_flow_rad.integrated_zgyro() ;
            var some_temperature = optical_flow_rad.temperature() ;
            var some_quality = optical_flow_rad.quality() ;
            var some_time_delta_distance_us = optical_flow_rad.time_delta_distance_us() ;
            var some_distance = optical_flow_rad.distance() ;
        }
        public static void fill(OPTICAL_FLOW_RAD optical_flow_rad)
        {
            optical_flow_rad.time_usec(some_long);
            optical_flow_rad.sensor_id(some_sbyte);
            optical_flow_rad.integration_time_us(some_int);
            optical_flow_rad.integrated_x(some_float);
            optical_flow_rad.integrated_y(some_float);
            optical_flow_rad.integrated_xgyro(some_float);
            optical_flow_rad.integrated_ygyro(some_float);
            optical_flow_rad.integrated_zgyro(some_float);
            optical_flow_rad.temperature(some_short);
            optical_flow_rad.quality(some_sbyte);
            optical_flow_rad.time_delta_distance_us(some_int);
            optical_flow_rad.distance(some_float);
        }
        public static void onHIL_SENSOR(HIL_SENSOR hil_sensor)
        {
            var some_time_usec = hil_sensor.time_usec() ;
            var some_xacc = hil_sensor.xacc() ;
            var some_yacc = hil_sensor.yacc() ;
            var some_zacc = hil_sensor.zacc() ;
            var some_xgyro = hil_sensor.xgyro() ;
            var some_ygyro = hil_sensor.ygyro() ;
            var some_zgyro = hil_sensor.zgyro() ;
            var some_xmag = hil_sensor.xmag() ;
            var some_ymag = hil_sensor.ymag() ;
            var some_zmag = hil_sensor.zmag() ;
            var some_abs_pressure = hil_sensor.abs_pressure() ;
            var some_diff_pressure = hil_sensor.diff_pressure() ;
            var some_pressure_alt = hil_sensor.pressure_alt() ;
            var some_temperature = hil_sensor.temperature() ;
            var some_fields_updated = hil_sensor.fields_updated() ;
        }
        public static void fill(HIL_SENSOR hil_sensor)
        {
            hil_sensor.time_usec(some_long);
            hil_sensor.xacc(some_float);
            hil_sensor.yacc(some_float);
            hil_sensor.zacc(some_float);
            hil_sensor.xgyro(some_float);
            hil_sensor.ygyro(some_float);
            hil_sensor.zgyro(some_float);
            hil_sensor.xmag(some_float);
            hil_sensor.ymag(some_float);
            hil_sensor.zmag(some_float);
            hil_sensor.abs_pressure(some_float);
            hil_sensor.diff_pressure(some_float);
            hil_sensor.pressure_alt(some_float);
            hil_sensor.temperature(some_float);
            hil_sensor.fields_updated(some_int);
        }
        public static void onSIM_STATE(SIM_STATE sim_state)
        {
            var some_q1 = sim_state.q1() ;
            var some_q2 = sim_state.q2() ;
            var some_q3 = sim_state.q3() ;
            var some_q4 = sim_state.q4() ;
            var some_roll = sim_state.roll() ;
            var some_pitch = sim_state.pitch() ;
            var some_yaw = sim_state.yaw() ;
            var some_xacc = sim_state.xacc() ;
            var some_yacc = sim_state.yacc() ;
            var some_zacc = sim_state.zacc() ;
            var some_xgyro = sim_state.xgyro() ;
            var some_ygyro = sim_state.ygyro() ;
            var some_zgyro = sim_state.zgyro() ;
            var some_lat = sim_state.lat() ;
            var some_lon = sim_state.lon() ;
            var some_alt = sim_state.alt() ;
            var some_std_dev_horz = sim_state.std_dev_horz() ;
            var some_std_dev_vert = sim_state.std_dev_vert() ;
            var some_vn = sim_state.vn() ;
            var some_ve = sim_state.ve() ;
            var some_vd = sim_state.vd() ;
        }
        public static void fill(SIM_STATE sim_state)
        {
            sim_state.q1(some_float);
            sim_state.q2(some_float);
            sim_state.q3(some_float);
            sim_state.q4(some_float);
            sim_state.roll(some_float);
            sim_state.pitch(some_float);
            sim_state.yaw(some_float);
            sim_state.xacc(some_float);
            sim_state.yacc(some_float);
            sim_state.zacc(some_float);
            sim_state.xgyro(some_float);
            sim_state.ygyro(some_float);
            sim_state.zgyro(some_float);
            sim_state.lat(some_float);
            sim_state.lon(some_float);
            sim_state.alt(some_float);
            sim_state.std_dev_horz(some_float);
            sim_state.std_dev_vert(some_float);
            sim_state.vn(some_float);
            sim_state.ve(some_float);
            sim_state.vd(some_float);
        }
        public static void onRADIO_STATUS(RADIO_STATUS radio_status)
        {
            var some_rssi = radio_status.rssi() ;
            var some_remrssi = radio_status.remrssi() ;
            var some_txbuf = radio_status.txbuf() ;
            var some_noise = radio_status.noise() ;
            var some_remnoise = radio_status.remnoise() ;
            var some_rxerrors = radio_status.rxerrors() ;
            var some_fixeD = radio_status.fixeD() ;
        }
        public static void fill(RADIO_STATUS radio_status)
        {
            radio_status.rssi(some_sbyte);
            radio_status.remrssi(some_sbyte);
            radio_status.txbuf(some_sbyte);
            radio_status.noise(some_sbyte);
            radio_status.remnoise(some_sbyte);
            radio_status.rxerrors(some_short);
            radio_status.fixeD(some_short);
        }
        public static void onFILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL file_transfer_protocol)
        {
            var some_target_network = file_transfer_protocol.target_network() ;
            var some_target_system = file_transfer_protocol.target_system() ;
            var some_target_component = file_transfer_protocol.target_component() ;
            {
                var item = file_transfer_protocol.payload();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(FILE_TRANSFER_PROTOCOL file_transfer_protocol)
        {
            file_transfer_protocol.target_network(some_sbyte);
            file_transfer_protocol.target_system(some_sbyte);
            file_transfer_protocol.target_component(some_sbyte);
            {
                var item = file_transfer_protocol.payload();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onTIMESYNC(TIMESYNC timesync)
        {
            var some_tc1 = timesync.tc1() ;
            var some_ts1 = timesync.ts1() ;
        }
        public static void fill(TIMESYNC timesync)
        {
            timesync.tc1(some_long);
            timesync.ts1(some_long);
        }
        public static void onCAMERA_TRIGGER(CAMERA_TRIGGER camera_trigger)
        {
            var some_time_usec = camera_trigger.time_usec() ;
            var some_seq = camera_trigger.seq() ;
        }
        public static void fill(CAMERA_TRIGGER camera_trigger)
        {
            camera_trigger.time_usec(some_long);
            camera_trigger.seq(some_int);
        }
        public static void onHIL_GPS(HIL_GPS hil_gps)
        {
            var some_time_usec = hil_gps.time_usec() ;
            var some_fix_type = hil_gps.fix_type() ;
            var some_lat = hil_gps.lat() ;
            var some_lon = hil_gps.lon() ;
            var some_alt = hil_gps.alt() ;
            var some_eph = hil_gps.eph() ;
            var some_epv = hil_gps.epv() ;
            var some_vel = hil_gps.vel() ;
            var some_vn = hil_gps.vn() ;
            var some_ve = hil_gps.ve() ;
            var some_vd = hil_gps.vd() ;
            var some_cog = hil_gps.cog() ;
            var some_satellites_visible = hil_gps.satellites_visible() ;
        }
        public static void fill(HIL_GPS hil_gps)
        {
            hil_gps.time_usec(some_long);
            hil_gps.fix_type(some_sbyte);
            hil_gps.lat(some_int);
            hil_gps.lon(some_int);
            hil_gps.alt(some_int);
            hil_gps.eph(some_short);
            hil_gps.epv(some_short);
            hil_gps.vel(some_short);
            hil_gps.vn(some_short);
            hil_gps.ve(some_short);
            hil_gps.vd(some_short);
            hil_gps.cog(some_short);
            hil_gps.satellites_visible(some_sbyte);
        }
        public static void onHIL_OPTICAL_FLOW(HIL_OPTICAL_FLOW hil_optical_flow)
        {
            var some_time_usec = hil_optical_flow.time_usec() ;
            var some_sensor_id = hil_optical_flow.sensor_id() ;
            var some_integration_time_us = hil_optical_flow.integration_time_us() ;
            var some_integrated_x = hil_optical_flow.integrated_x() ;
            var some_integrated_y = hil_optical_flow.integrated_y() ;
            var some_integrated_xgyro = hil_optical_flow.integrated_xgyro() ;
            var some_integrated_ygyro = hil_optical_flow.integrated_ygyro() ;
            var some_integrated_zgyro = hil_optical_flow.integrated_zgyro() ;
            var some_temperature = hil_optical_flow.temperature() ;
            var some_quality = hil_optical_flow.quality() ;
            var some_time_delta_distance_us = hil_optical_flow.time_delta_distance_us() ;
            var some_distance = hil_optical_flow.distance() ;
        }
        public static void fill(HIL_OPTICAL_FLOW hil_optical_flow)
        {
            hil_optical_flow.time_usec(some_long);
            hil_optical_flow.sensor_id(some_sbyte);
            hil_optical_flow.integration_time_us(some_int);
            hil_optical_flow.integrated_x(some_float);
            hil_optical_flow.integrated_y(some_float);
            hil_optical_flow.integrated_xgyro(some_float);
            hil_optical_flow.integrated_ygyro(some_float);
            hil_optical_flow.integrated_zgyro(some_float);
            hil_optical_flow.temperature(some_short);
            hil_optical_flow.quality(some_sbyte);
            hil_optical_flow.time_delta_distance_us(some_int);
            hil_optical_flow.distance(some_float);
        }
        public static void onHIL_STATE_QUATERNION(HIL_STATE_QUATERNION hil_state_quaternion)
        {
            var some_time_usec = hil_state_quaternion.time_usec() ;
            {
                var item = hil_state_quaternion.attitude_quaternion();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_rollspeed = hil_state_quaternion.rollspeed() ;
            var some_pitchspeed = hil_state_quaternion.pitchspeed() ;
            var some_yawspeed = hil_state_quaternion.yawspeed() ;
            var some_lat = hil_state_quaternion.lat() ;
            var some_lon = hil_state_quaternion.lon() ;
            var some_alt = hil_state_quaternion.alt() ;
            var some_vx = hil_state_quaternion.vx() ;
            var some_vy = hil_state_quaternion.vy() ;
            var some_vz = hil_state_quaternion.vz() ;
            var some_ind_airspeed = hil_state_quaternion.ind_airspeed() ;
            var some_true_airspeed = hil_state_quaternion.true_airspeed() ;
            var some_xacc = hil_state_quaternion.xacc() ;
            var some_yacc = hil_state_quaternion.yacc() ;
            var some_zacc = hil_state_quaternion.zacc() ;
        }
        public static void fill(HIL_STATE_QUATERNION hil_state_quaternion)
        {
            hil_state_quaternion.time_usec(some_long);
            {
                var item = hil_state_quaternion.attitude_quaternion();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            hil_state_quaternion.rollspeed(some_float);
            hil_state_quaternion.pitchspeed(some_float);
            hil_state_quaternion.yawspeed(some_float);
            hil_state_quaternion.lat(some_int);
            hil_state_quaternion.lon(some_int);
            hil_state_quaternion.alt(some_int);
            hil_state_quaternion.vx(some_short);
            hil_state_quaternion.vy(some_short);
            hil_state_quaternion.vz(some_short);
            hil_state_quaternion.ind_airspeed(some_short);
            hil_state_quaternion.true_airspeed(some_short);
            hil_state_quaternion.xacc(some_short);
            hil_state_quaternion.yacc(some_short);
            hil_state_quaternion.zacc(some_short);
        }
        public static void onSCALED_IMU2(SCALED_IMU2 scaled_imu2)
        {
            var some_time_boot_ms = scaled_imu2.time_boot_ms() ;
            var some_xacc = scaled_imu2.xacc() ;
            var some_yacc = scaled_imu2.yacc() ;
            var some_zacc = scaled_imu2.zacc() ;
            var some_xgyro = scaled_imu2.xgyro() ;
            var some_ygyro = scaled_imu2.ygyro() ;
            var some_zgyro = scaled_imu2.zgyro() ;
            var some_xmag = scaled_imu2.xmag() ;
            var some_ymag = scaled_imu2.ymag() ;
            var some_zmag = scaled_imu2.zmag() ;
        }
        public static void fill(SCALED_IMU2 scaled_imu2)
        {
            scaled_imu2.time_boot_ms(some_int);
            scaled_imu2.xacc(some_short);
            scaled_imu2.yacc(some_short);
            scaled_imu2.zacc(some_short);
            scaled_imu2.xgyro(some_short);
            scaled_imu2.ygyro(some_short);
            scaled_imu2.zgyro(some_short);
            scaled_imu2.xmag(some_short);
            scaled_imu2.ymag(some_short);
            scaled_imu2.zmag(some_short);
        }
        public static void onLOG_REQUEST_LIST(LOG_REQUEST_LIST log_request_list)
        {
            var some_target_system = log_request_list.target_system() ;
            var some_target_component = log_request_list.target_component() ;
            var some_start = log_request_list.start() ;
            var some_end = log_request_list.end() ;
        }
        public static void fill(LOG_REQUEST_LIST log_request_list)
        {
            log_request_list.target_system(some_sbyte);
            log_request_list.target_component(some_sbyte);
            log_request_list.start(some_short);
            log_request_list.end(some_short);
        }
        public static void onLOG_ENTRY(LOG_ENTRY log_entry)
        {
            var some_id = log_entry.id() ;
            var some_num_logs = log_entry.num_logs() ;
            var some_last_log_num = log_entry.last_log_num() ;
            var some_time_utc = log_entry.time_utc() ;
            var some_size = log_entry.size() ;
        }
        public static void fill(LOG_ENTRY log_entry)
        {
            log_entry.id(some_short);
            log_entry.num_logs(some_short);
            log_entry.last_log_num(some_short);
            log_entry.time_utc(some_int);
            log_entry.size(some_int);
        }
        public static void onLOG_REQUEST_DATA(LOG_REQUEST_DATA log_request_data)
        {
            var some_target_system = log_request_data.target_system() ;
            var some_target_component = log_request_data.target_component() ;
            var some_id = log_request_data.id() ;
            var some_ofs = log_request_data.ofs() ;
            var some_count = log_request_data.count() ;
        }
        public static void fill(LOG_REQUEST_DATA log_request_data)
        {
            log_request_data.target_system(some_sbyte);
            log_request_data.target_component(some_sbyte);
            log_request_data.id(some_short);
            log_request_data.ofs(some_int);
            log_request_data.count(some_int);
        }
        public static void onLOG_DATA(LOG_DATA log_data)
        {
            var some_id = log_data.id() ;
            var some_ofs = log_data.ofs() ;
            var some_count = log_data.count() ;
            {
                var item = log_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(LOG_DATA log_data)
        {
            log_data.id(some_short);
            log_data.ofs(some_int);
            log_data.count(some_sbyte);
            {
                var item = log_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onLOG_ERASE(LOG_ERASE log_erase)
        {
            var some_target_system = log_erase.target_system() ;
            var some_target_component = log_erase.target_component() ;
        }
        public static void fill(LOG_ERASE log_erase)
        {
            log_erase.target_system(some_sbyte);
            log_erase.target_component(some_sbyte);
        }
        public static void onLOG_REQUEST_END(LOG_REQUEST_END log_request_end)
        {
            var some_target_system = log_request_end.target_system() ;
            var some_target_component = log_request_end.target_component() ;
        }
        public static void fill(LOG_REQUEST_END log_request_end)
        {
            log_request_end.target_system(some_sbyte);
            log_request_end.target_component(some_sbyte);
        }
        public static void onGPS_INJECT_DATA(GPS_INJECT_DATA gps_inject_data)
        {
            var some_target_system = gps_inject_data.target_system() ;
            var some_target_component = gps_inject_data.target_component() ;
            var some_len = gps_inject_data.len() ;
            {
                var item = gps_inject_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(GPS_INJECT_DATA gps_inject_data)
        {
            gps_inject_data.target_system(some_sbyte);
            gps_inject_data.target_component(some_sbyte);
            gps_inject_data.len(some_sbyte);
            {
                var item = gps_inject_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onGPS2_RAW(GPS2_RAW gps2_raw)
        {
            var some_time_usec = gps2_raw.time_usec() ;
            {
                var item = gps2_raw.fix_type();
                if(item.HasValue)
                    some_GPS_FIX_TYPE = item.Value ;
            }
            var some_lat = gps2_raw.lat() ;
            var some_lon = gps2_raw.lon() ;
            var some_alt = gps2_raw.alt() ;
            var some_eph = gps2_raw.eph() ;
            var some_epv = gps2_raw.epv() ;
            var some_vel = gps2_raw.vel() ;
            var some_cog = gps2_raw.cog() ;
            var some_satellites_visible = gps2_raw.satellites_visible() ;
            var some_dgps_numch = gps2_raw.dgps_numch() ;
            var some_dgps_age = gps2_raw.dgps_age() ;
        }
        public static void fill(GPS2_RAW gps2_raw)
        {
            gps2_raw.time_usec(some_long);
            gps2_raw.fix_type(some_GPS_FIX_TYPE);
            gps2_raw.lat(some_int);
            gps2_raw.lon(some_int);
            gps2_raw.alt(some_int);
            gps2_raw.eph(some_short);
            gps2_raw.epv(some_short);
            gps2_raw.vel(some_short);
            gps2_raw.cog(some_short);
            gps2_raw.satellites_visible(some_sbyte);
            gps2_raw.dgps_numch(some_sbyte);
            gps2_raw.dgps_age(some_int);
        }
        public static void onPOWER_STATUS(POWER_STATUS power_status)
        {
            var some_Vcc = power_status.Vcc() ;
            var some_Vservo = power_status.Vservo() ;
            {
                var item = power_status.flags();
                if(item.HasValue)
                    some_MAV_POWER_STATUS = item.Value ;
            }
        }
        public static void fill(POWER_STATUS power_status)
        {
            power_status.Vcc(some_short);
            power_status.Vservo(some_short);
            power_status.flags(some_MAV_POWER_STATUS);
        }
        public static void onSERIAL_CONTROL(SERIAL_CONTROL serial_control)
        {
            {
                var item = serial_control.device();
                if(item.HasValue)
                    some_SERIAL_CONTROL_DEV = item.Value ;
            }
            {
                var item = serial_control.flags();
                if(item.HasValue)
                    some_SERIAL_CONTROL_FLAG = item.Value ;
            }
            var some_timeout = serial_control.timeout() ;
            var some_baudrate = serial_control.baudrate() ;
            var some_count = serial_control.count() ;
            {
                var item = serial_control.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(SERIAL_CONTROL serial_control)
        {
            serial_control.device(some_SERIAL_CONTROL_DEV);
            serial_control.flags(some_SERIAL_CONTROL_FLAG);
            serial_control.timeout(some_short);
            serial_control.baudrate(some_int);
            serial_control.count(some_sbyte);
            {
                var item = serial_control.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onGPS_RTK(GPS_RTK gps_rtk)
        {
            var some_time_last_baseline_ms = gps_rtk.time_last_baseline_ms() ;
            var some_rtk_receiver_id = gps_rtk.rtk_receiver_id() ;
            var some_wn = gps_rtk.wn() ;
            var some_tow = gps_rtk.tow() ;
            var some_rtk_health = gps_rtk.rtk_health() ;
            var some_rtk_rate = gps_rtk.rtk_rate() ;
            var some_nsats = gps_rtk.nsats() ;
            var some_baseline_coords_type = gps_rtk.baseline_coords_type() ;
            var some_baseline_a_mm = gps_rtk.baseline_a_mm() ;
            var some_baseline_b_mm = gps_rtk.baseline_b_mm() ;
            var some_baseline_c_mm = gps_rtk.baseline_c_mm() ;
            var some_accuracy = gps_rtk.accuracy() ;
            var some_iar_num_hypotheses = gps_rtk.iar_num_hypotheses() ;
        }
        public static void fill(GPS_RTK gps_rtk)
        {
            gps_rtk.time_last_baseline_ms(some_int);
            gps_rtk.rtk_receiver_id(some_sbyte);
            gps_rtk.wn(some_short);
            gps_rtk.tow(some_int);
            gps_rtk.rtk_health(some_sbyte);
            gps_rtk.rtk_rate(some_sbyte);
            gps_rtk.nsats(some_sbyte);
            gps_rtk.baseline_coords_type(some_sbyte);
            gps_rtk.baseline_a_mm(some_int);
            gps_rtk.baseline_b_mm(some_int);
            gps_rtk.baseline_c_mm(some_int);
            gps_rtk.accuracy(some_int);
            gps_rtk.iar_num_hypotheses(some_int);
        }
        public static void onGPS2_RTK(GPS2_RTK gps2_rtk)
        {
            var some_time_last_baseline_ms = gps2_rtk.time_last_baseline_ms() ;
            var some_rtk_receiver_id = gps2_rtk.rtk_receiver_id() ;
            var some_wn = gps2_rtk.wn() ;
            var some_tow = gps2_rtk.tow() ;
            var some_rtk_health = gps2_rtk.rtk_health() ;
            var some_rtk_rate = gps2_rtk.rtk_rate() ;
            var some_nsats = gps2_rtk.nsats() ;
            var some_baseline_coords_type = gps2_rtk.baseline_coords_type() ;
            var some_baseline_a_mm = gps2_rtk.baseline_a_mm() ;
            var some_baseline_b_mm = gps2_rtk.baseline_b_mm() ;
            var some_baseline_c_mm = gps2_rtk.baseline_c_mm() ;
            var some_accuracy = gps2_rtk.accuracy() ;
            var some_iar_num_hypotheses = gps2_rtk.iar_num_hypotheses() ;
        }
        public static void fill(GPS2_RTK gps2_rtk)
        {
            gps2_rtk.time_last_baseline_ms(some_int);
            gps2_rtk.rtk_receiver_id(some_sbyte);
            gps2_rtk.wn(some_short);
            gps2_rtk.tow(some_int);
            gps2_rtk.rtk_health(some_sbyte);
            gps2_rtk.rtk_rate(some_sbyte);
            gps2_rtk.nsats(some_sbyte);
            gps2_rtk.baseline_coords_type(some_sbyte);
            gps2_rtk.baseline_a_mm(some_int);
            gps2_rtk.baseline_b_mm(some_int);
            gps2_rtk.baseline_c_mm(some_int);
            gps2_rtk.accuracy(some_int);
            gps2_rtk.iar_num_hypotheses(some_int);
        }
        public static void onSCALED_IMU3(SCALED_IMU3 scaled_imu3)
        {
            var some_time_boot_ms = scaled_imu3.time_boot_ms() ;
            var some_xacc = scaled_imu3.xacc() ;
            var some_yacc = scaled_imu3.yacc() ;
            var some_zacc = scaled_imu3.zacc() ;
            var some_xgyro = scaled_imu3.xgyro() ;
            var some_ygyro = scaled_imu3.ygyro() ;
            var some_zgyro = scaled_imu3.zgyro() ;
            var some_xmag = scaled_imu3.xmag() ;
            var some_ymag = scaled_imu3.ymag() ;
            var some_zmag = scaled_imu3.zmag() ;
        }
        public static void fill(SCALED_IMU3 scaled_imu3)
        {
            scaled_imu3.time_boot_ms(some_int);
            scaled_imu3.xacc(some_short);
            scaled_imu3.yacc(some_short);
            scaled_imu3.zacc(some_short);
            scaled_imu3.xgyro(some_short);
            scaled_imu3.ygyro(some_short);
            scaled_imu3.zgyro(some_short);
            scaled_imu3.xmag(some_short);
            scaled_imu3.ymag(some_short);
            scaled_imu3.zmag(some_short);
        }
        public static void onDATA_TRANSMISSION_HANDSHAKE(DATA_TRANSMISSION_HANDSHAKE data_transmission_handshake)
        {
            var some_typE = data_transmission_handshake.typE() ;
            var some_size = data_transmission_handshake.size() ;
            var some_width = data_transmission_handshake.width() ;
            var some_height = data_transmission_handshake.height() ;
            var some_packets = data_transmission_handshake.packets() ;
            var some_payload = data_transmission_handshake.payload() ;
            var some_jpg_quality = data_transmission_handshake.jpg_quality() ;
        }
        public static void fill(DATA_TRANSMISSION_HANDSHAKE data_transmission_handshake)
        {
            data_transmission_handshake.typE(some_sbyte);
            data_transmission_handshake.size(some_int);
            data_transmission_handshake.width(some_short);
            data_transmission_handshake.height(some_short);
            data_transmission_handshake.packets(some_short);
            data_transmission_handshake.payload(some_sbyte);
            data_transmission_handshake.jpg_quality(some_sbyte);
        }
        public static void onENCAPSULATED_DATA(ENCAPSULATED_DATA encapsulated_data)
        {
            var some_seqnr = encapsulated_data.seqnr() ;
            {
                var item = encapsulated_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(ENCAPSULATED_DATA encapsulated_data)
        {
            encapsulated_data.seqnr(some_short);
            {
                var item = encapsulated_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onDISTANCE_SENSOR(DISTANCE_SENSOR distance_sensor)
        {
            var some_time_boot_ms = distance_sensor.time_boot_ms() ;
            var some_min_distance = distance_sensor.min_distance() ;
            var some_max_distance = distance_sensor.max_distance() ;
            var some_current_distance = distance_sensor.current_distance() ;
            {
                var item = distance_sensor.typE();
                if(item.HasValue)
                    some_MAV_DISTANCE_SENSOR = item.Value ;
            }
            var some_id = distance_sensor.id() ;
            {
                var item = distance_sensor.orientation();
                if(item.HasValue)
                    some_MAV_SENSOR_ORIENTATION = item.Value ;
            }
            var some_covariance = distance_sensor.covariance() ;
        }
        public static void fill(DISTANCE_SENSOR distance_sensor)
        {
            distance_sensor.time_boot_ms(some_int);
            distance_sensor.min_distance(some_short);
            distance_sensor.max_distance(some_short);
            distance_sensor.current_distance(some_short);
            distance_sensor.typE(some_MAV_DISTANCE_SENSOR);
            distance_sensor.id(some_sbyte);
            distance_sensor.orientation(some_MAV_SENSOR_ORIENTATION);
            distance_sensor.covariance(some_sbyte);
        }
        public static void onTERRAIN_REQUEST(TERRAIN_REQUEST terrain_request)
        {
            var some_lat = terrain_request.lat() ;
            var some_lon = terrain_request.lon() ;
            var some_grid_spacing = terrain_request.grid_spacing() ;
            var some_mask = terrain_request.mask() ;
        }
        public static void fill(TERRAIN_REQUEST terrain_request)
        {
            terrain_request.lat(some_int);
            terrain_request.lon(some_int);
            terrain_request.grid_spacing(some_short);
            terrain_request.mask(some_long);
        }
        public static void onTERRAIN_DATA(TERRAIN_DATA terrain_data)
        {
            var some_lat = terrain_data.lat() ;
            var some_lon = terrain_data.lon() ;
            var some_grid_spacing = terrain_data.grid_spacing() ;
            var some_gridbit = terrain_data.gridbit() ;
            {
                var item = terrain_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_short = item.get(i);
            }
        }
        public static void fill(TERRAIN_DATA terrain_data)
        {
            terrain_data.lat(some_int);
            terrain_data.lon(some_int);
            terrain_data.grid_spacing(some_short);
            terrain_data.gridbit(some_sbyte);
            {
                var item = terrain_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_short, i);
            }
        }
        public static void onTERRAIN_CHECK(TERRAIN_CHECK terrain_check)
        {
            var some_lat = terrain_check.lat() ;
            var some_lon = terrain_check.lon() ;
        }
        public static void fill(TERRAIN_CHECK terrain_check)
        {
            terrain_check.lat(some_int);
            terrain_check.lon(some_int);
        }
        public static void onTERRAIN_REPORT(TERRAIN_REPORT terrain_report)
        {
            var some_lat = terrain_report.lat() ;
            var some_lon = terrain_report.lon() ;
            var some_spacing = terrain_report.spacing() ;
            var some_terrain_height = terrain_report.terrain_height() ;
            var some_current_height = terrain_report.current_height() ;
            var some_pending = terrain_report.pending() ;
            var some_loaded = terrain_report.loaded() ;
        }
        public static void fill(TERRAIN_REPORT terrain_report)
        {
            terrain_report.lat(some_int);
            terrain_report.lon(some_int);
            terrain_report.spacing(some_short);
            terrain_report.terrain_height(some_float);
            terrain_report.current_height(some_float);
            terrain_report.pending(some_short);
            terrain_report.loaded(some_short);
        }
        public static void onSCALED_PRESSURE2(SCALED_PRESSURE2 scaled_pressure2)
        {
            var some_time_boot_ms = scaled_pressure2.time_boot_ms() ;
            var some_press_abs = scaled_pressure2.press_abs() ;
            var some_press_diff = scaled_pressure2.press_diff() ;
            var some_temperature = scaled_pressure2.temperature() ;
        }
        public static void fill(SCALED_PRESSURE2 scaled_pressure2)
        {
            scaled_pressure2.time_boot_ms(some_int);
            scaled_pressure2.press_abs(some_float);
            scaled_pressure2.press_diff(some_float);
            scaled_pressure2.temperature(some_short);
        }
        public static void onATT_POS_MOCAP(ATT_POS_MOCAP att_pos_mocap)
        {
            var some_time_usec = att_pos_mocap.time_usec() ;
            {
                var item = att_pos_mocap.q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_x = att_pos_mocap.x() ;
            var some_y = att_pos_mocap.y() ;
            var some_z = att_pos_mocap.z() ;
        }
        public static void fill(ATT_POS_MOCAP att_pos_mocap)
        {
            att_pos_mocap.time_usec(some_long);
            {
                var item = att_pos_mocap.q();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            att_pos_mocap.x(some_float);
            att_pos_mocap.y(some_float);
            att_pos_mocap.z(some_float);
        }
        public static void onSET_ACTUATOR_CONTROL_TARGET(SET_ACTUATOR_CONTROL_TARGET set_actuator_control_target)
        {
            var some_time_usec = set_actuator_control_target.time_usec() ;
            var some_group_mlx = set_actuator_control_target.group_mlx() ;
            var some_target_system = set_actuator_control_target.target_system() ;
            var some_target_component = set_actuator_control_target.target_component() ;
            {
                var item = set_actuator_control_target.controls();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
        }
        public static void fill(SET_ACTUATOR_CONTROL_TARGET set_actuator_control_target)
        {
            set_actuator_control_target.time_usec(some_long);
            set_actuator_control_target.group_mlx(some_sbyte);
            set_actuator_control_target.target_system(some_sbyte);
            set_actuator_control_target.target_component(some_sbyte);
            {
                var item = set_actuator_control_target.controls();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
        }
        public static void onACTUATOR_CONTROL_TARGET(ACTUATOR_CONTROL_TARGET actuator_control_target)
        {
            var some_time_usec = actuator_control_target.time_usec() ;
            var some_group_mlx = actuator_control_target.group_mlx() ;
            {
                var item = actuator_control_target.controls();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
        }
        public static void fill(ACTUATOR_CONTROL_TARGET actuator_control_target)
        {
            actuator_control_target.time_usec(some_long);
            actuator_control_target.group_mlx(some_sbyte);
            {
                var item = actuator_control_target.controls();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
        }
        public static void onALTITUDE(ALTITUDE altitude)
        {
            var some_time_usec = altitude.time_usec() ;
            var some_altitude_monotonic = altitude.altitude_monotonic() ;
            var some_altitude_amsl = altitude.altitude_amsl() ;
            var some_altitude_local = altitude.altitude_local() ;
            var some_altitude_relative = altitude.altitude_relative() ;
            var some_altitude_terrain = altitude.altitude_terrain() ;
            var some_bottom_clearance = altitude.bottom_clearance() ;
        }
        public static void fill(ALTITUDE altitude)
        {
            altitude.time_usec(some_long);
            altitude.altitude_monotonic(some_float);
            altitude.altitude_amsl(some_float);
            altitude.altitude_local(some_float);
            altitude.altitude_relative(some_float);
            altitude.altitude_terrain(some_float);
            altitude.bottom_clearance(some_float);
        }
        public static void onRESOURCE_REQUEST(RESOURCE_REQUEST resource_request)
        {
            var some_request_id = resource_request.request_id() ;
            var some_uri_type = resource_request.uri_type() ;
            {
                var item = resource_request.uri();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            var some_transfer_type = resource_request.transfer_type() ;
            {
                var item = resource_request.storage();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(RESOURCE_REQUEST resource_request)
        {
            resource_request.request_id(some_sbyte);
            resource_request.uri_type(some_sbyte);
            {
                var item = resource_request.uri();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            resource_request.transfer_type(some_sbyte);
            {
                var item = resource_request.storage();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onSCALED_PRESSURE3(SCALED_PRESSURE3 scaled_pressure3)
        {
            var some_time_boot_ms = scaled_pressure3.time_boot_ms() ;
            var some_press_abs = scaled_pressure3.press_abs() ;
            var some_press_diff = scaled_pressure3.press_diff() ;
            var some_temperature = scaled_pressure3.temperature() ;
        }
        public static void fill(SCALED_PRESSURE3 scaled_pressure3)
        {
            scaled_pressure3.time_boot_ms(some_int);
            scaled_pressure3.press_abs(some_float);
            scaled_pressure3.press_diff(some_float);
            scaled_pressure3.temperature(some_short);
        }
        public static void onFOLLOW_TARGET(FOLLOW_TARGET follow_target)
        {
            var some_timestamp = follow_target.timestamp() ;
            var some_est_capabilities = follow_target.est_capabilities() ;
            var some_lat = follow_target.lat() ;
            var some_lon = follow_target.lon() ;
            var some_alt = follow_target.alt() ;
            {
                var item = follow_target.vel();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            {
                var item = follow_target.acc();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            {
                var item = follow_target.attitude_q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            {
                var item = follow_target.rates();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            {
                var item = follow_target.position_cov();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_custom_state = follow_target.custom_state() ;
        }
        public static void fill(FOLLOW_TARGET follow_target)
        {
            follow_target.timestamp(some_long);
            follow_target.est_capabilities(some_sbyte);
            follow_target.lat(some_int);
            follow_target.lon(some_int);
            follow_target.alt(some_float);
            {
                var item = follow_target.vel();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            {
                var item = follow_target.acc();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            {
                var item = follow_target.attitude_q();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            {
                var item = follow_target.rates();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            {
                var item = follow_target.position_cov();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            follow_target.custom_state(some_long);
        }
        public static void onCONTROL_SYSTEM_STATE(CONTROL_SYSTEM_STATE control_system_state)
        {
            var some_time_usec = control_system_state.time_usec() ;
            var some_x_acc = control_system_state.x_acc() ;
            var some_y_acc = control_system_state.y_acc() ;
            var some_z_acc = control_system_state.z_acc() ;
            var some_x_vel = control_system_state.x_vel() ;
            var some_y_vel = control_system_state.y_vel() ;
            var some_z_vel = control_system_state.z_vel() ;
            var some_x_pos = control_system_state.x_pos() ;
            var some_y_pos = control_system_state.y_pos() ;
            var some_z_pos = control_system_state.z_pos() ;
            var some_airspeed = control_system_state.airspeed() ;
            {
                var item = control_system_state.vel_variance();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            {
                var item = control_system_state.pos_variance();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            {
                var item = control_system_state.q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_roll_rate = control_system_state.roll_rate() ;
            var some_pitch_rate = control_system_state.pitch_rate() ;
            var some_yaw_rate = control_system_state.yaw_rate() ;
        }
        public static void fill(CONTROL_SYSTEM_STATE control_system_state)
        {
            control_system_state.time_usec(some_long);
            control_system_state.x_acc(some_float);
            control_system_state.y_acc(some_float);
            control_system_state.z_acc(some_float);
            control_system_state.x_vel(some_float);
            control_system_state.y_vel(some_float);
            control_system_state.z_vel(some_float);
            control_system_state.x_pos(some_float);
            control_system_state.y_pos(some_float);
            control_system_state.z_pos(some_float);
            control_system_state.airspeed(some_float);
            {
                var item = control_system_state.vel_variance();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            {
                var item = control_system_state.pos_variance();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            {
                var item = control_system_state.q();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            control_system_state.roll_rate(some_float);
            control_system_state.pitch_rate(some_float);
            control_system_state.yaw_rate(some_float);
        }
        public static void onBATTERY_STATUS(BATTERY_STATUS battery_status)
        {
            var some_id = battery_status.id() ;
            {
                var item = battery_status.battery_function();
                if(item.HasValue)
                    some_MAV_BATTERY_FUNCTION = item.Value ;
            }
            {
                var item = battery_status.typE();
                if(item.HasValue)
                    some_MAV_BATTERY_TYPE = item.Value ;
            }
            var some_temperature = battery_status.temperature() ;
            {
                var item = battery_status.voltages();
                for(int i = 0; i < item.len(); i++)
                    some_short = item.get(i);
            }
            var some_current_battery = battery_status.current_battery() ;
            var some_current_consumed = battery_status.current_consumed() ;
            var some_energy_consumed = battery_status.energy_consumed() ;
            var some_battery_remaining = battery_status.battery_remaining() ;
        }
        public static void fill(BATTERY_STATUS battery_status)
        {
            battery_status.id(some_sbyte);
            battery_status.battery_function(some_MAV_BATTERY_FUNCTION);
            battery_status.typE(some_MAV_BATTERY_TYPE);
            battery_status.temperature(some_short);
            {
                var item = battery_status.voltages();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_short, i);
            }
            battery_status.current_battery(some_short);
            battery_status.current_consumed(some_int);
            battery_status.energy_consumed(some_int);
            battery_status.battery_remaining(some_sbyte);
        }
        public static void onAUTOPILOT_VERSION(AUTOPILOT_VERSION autopilot_version)
        {
            {
                var item = autopilot_version.capabilities();
                if(item.HasValue)
                    some_MAV_PROTOCOL_CAPABILITY = item.Value ;
            }
            var some_flight_sw_version = autopilot_version.flight_sw_version() ;
            var some_middleware_sw_version = autopilot_version.middleware_sw_version() ;
            var some_os_sw_version = autopilot_version.os_sw_version() ;
            var some_board_version = autopilot_version.board_version() ;
            {
                var item = autopilot_version.flight_custom_version();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            {
                var item = autopilot_version.middleware_custom_version();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            {
                var item = autopilot_version.os_custom_version();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            var some_vendor_id = autopilot_version.vendor_id() ;
            var some_product_id = autopilot_version.product_id() ;
            var some_uid = autopilot_version.uid() ;
            {
                var _fld = autopilot_version.uid2();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0) =>
                    {
                        some_sbyte = item ;
                    }
                                 );
                }
            }
        }
        public static void fill(AUTOPILOT_VERSION autopilot_version)
        {
            autopilot_version.capabilities(some_MAV_PROTOCOL_CAPABILITY);
            autopilot_version.flight_sw_version(some_int);
            autopilot_version.middleware_sw_version(some_int);
            autopilot_version.os_sw_version(some_int);
            autopilot_version.board_version(some_int);
            {
                var item = autopilot_version.flight_custom_version();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            {
                var item = autopilot_version.middleware_custom_version();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            {
                var item = autopilot_version.os_custom_version();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            autopilot_version.vendor_id(some_short);
            autopilot_version.product_id(some_short);
            autopilot_version.uid(some_long);
            for(int d0 = 0; d0 < AUTOPILOT_VERSION.uid2_.d0; d0++)
            {
                autopilot_version.uid2(some_sbyte, d0);
            }
        }
        public static void onLANDING_TARGET(LANDING_TARGET landing_target)
        {
            var some_time_usec = landing_target.time_usec() ;
            var some_target_num = landing_target.target_num() ;
            {
                var item = landing_target.frame();
                if(item.HasValue)
                    some_MAV_FRAME = item.Value ;
            }
            var some_angle_x = landing_target.angle_x() ;
            var some_angle_y = landing_target.angle_y() ;
            var some_distance = landing_target.distance() ;
            var some_size_x = landing_target.size_x() ;
            var some_size_y = landing_target.size_y() ;
            {
                var item = landing_target.x();
                if(item.HasValue)
                    some_float = item.Value ;
            }
            {
                var item = landing_target.y();
                if(item.HasValue)
                    some_float = item.Value ;
            }
            {
                var item = landing_target.z();
                if(item.HasValue)
                    some_float = item.Value ;
            }
            {
                var _fld = landing_target.q();
                if(_fld.HasValue)
                {
                    var fld = _fld.Value;
                    fld.enumerate((item, d0) =>
                    {
                        some_float = item ;
                    }
                                 );
                }
            }
            {
                var item = landing_target.typE();
                if(item.HasValue)
                    some_LANDING_TARGET_TYPE = item.Value ;
            }
            {
                var item = landing_target.position_valid();
                if(item.HasValue)
                    some_sbyte = item.Value ;
            }
        }
        public static void fill(LANDING_TARGET landing_target)
        {
            landing_target.time_usec(some_long);
            landing_target.target_num(some_sbyte);
            landing_target.frame(some_MAV_FRAME);
            landing_target.angle_x(some_float);
            landing_target.angle_y(some_float);
            landing_target.distance(some_float);
            landing_target.size_x(some_float);
            landing_target.size_y(some_float);
            landing_target.x(some_float);
            landing_target.y(some_float);
            landing_target.z(some_float);
            for(int d0 = 0; d0 < LANDING_TARGET.q_.d0; d0++)
            {
                landing_target.q(some_float, d0);
            }
            landing_target.typE(some_LANDING_TARGET_TYPE);
            landing_target.position_valid(some_sbyte);
        }
        public static void onESTIMATOR_STATUS(ESTIMATOR_STATUS estimator_status)
        {
            var some_time_usec = estimator_status.time_usec() ;
            {
                var item = estimator_status.flags();
                if(item.HasValue)
                    some_ESTIMATOR_STATUS_FLAGS = item.Value ;
            }
            var some_vel_ratio = estimator_status.vel_ratio() ;
            var some_pos_horiz_ratio = estimator_status.pos_horiz_ratio() ;
            var some_pos_vert_ratio = estimator_status.pos_vert_ratio() ;
            var some_mag_ratio = estimator_status.mag_ratio() ;
            var some_hagl_ratio = estimator_status.hagl_ratio() ;
            var some_tas_ratio = estimator_status.tas_ratio() ;
            var some_pos_horiz_accuracy = estimator_status.pos_horiz_accuracy() ;
            var some_pos_vert_accuracy = estimator_status.pos_vert_accuracy() ;
        }
        public static void fill(ESTIMATOR_STATUS estimator_status)
        {
            estimator_status.time_usec(some_long);
            estimator_status.flags(some_ESTIMATOR_STATUS_FLAGS);
            estimator_status.vel_ratio(some_float);
            estimator_status.pos_horiz_ratio(some_float);
            estimator_status.pos_vert_ratio(some_float);
            estimator_status.mag_ratio(some_float);
            estimator_status.hagl_ratio(some_float);
            estimator_status.tas_ratio(some_float);
            estimator_status.pos_horiz_accuracy(some_float);
            estimator_status.pos_vert_accuracy(some_float);
        }
        public static void onWIND_COV(WIND_COV wind_cov)
        {
            var some_time_usec = wind_cov.time_usec() ;
            var some_wind_x = wind_cov.wind_x() ;
            var some_wind_y = wind_cov.wind_y() ;
            var some_wind_z = wind_cov.wind_z() ;
            var some_var_horiz = wind_cov.var_horiz() ;
            var some_var_vert = wind_cov.var_vert() ;
            var some_wind_alt = wind_cov.wind_alt() ;
            var some_horiz_accuracy = wind_cov.horiz_accuracy() ;
            var some_vert_accuracy = wind_cov.vert_accuracy() ;
        }
        public static void fill(WIND_COV wind_cov)
        {
            wind_cov.time_usec(some_long);
            wind_cov.wind_x(some_float);
            wind_cov.wind_y(some_float);
            wind_cov.wind_z(some_float);
            wind_cov.var_horiz(some_float);
            wind_cov.var_vert(some_float);
            wind_cov.wind_alt(some_float);
            wind_cov.horiz_accuracy(some_float);
            wind_cov.vert_accuracy(some_float);
        }
        public static void onGPS_INPUT(GPS_INPUT gps_input)
        {
            var some_time_usec = gps_input.time_usec() ;
            var some_gps_id = gps_input.gps_id() ;
            {
                var item = gps_input.ignore_flags();
                if(item.HasValue)
                    some_GPS_INPUT_IGNORE_FLAGS = item.Value ;
            }
            var some_time_week_ms = gps_input.time_week_ms() ;
            var some_time_week = gps_input.time_week() ;
            var some_fix_type = gps_input.fix_type() ;
            var some_lat = gps_input.lat() ;
            var some_lon = gps_input.lon() ;
            var some_alt = gps_input.alt() ;
            var some_hdop = gps_input.hdop() ;
            var some_vdop = gps_input.vdop() ;
            var some_vn = gps_input.vn() ;
            var some_ve = gps_input.ve() ;
            var some_vd = gps_input.vd() ;
            var some_speed_accuracy = gps_input.speed_accuracy() ;
            var some_horiz_accuracy = gps_input.horiz_accuracy() ;
            var some_vert_accuracy = gps_input.vert_accuracy() ;
            var some_satellites_visible = gps_input.satellites_visible() ;
        }
        public static void fill(GPS_INPUT gps_input)
        {
            gps_input.time_usec(some_long);
            gps_input.gps_id(some_sbyte);
            gps_input.ignore_flags(some_GPS_INPUT_IGNORE_FLAGS);
            gps_input.time_week_ms(some_int);
            gps_input.time_week(some_short);
            gps_input.fix_type(some_sbyte);
            gps_input.lat(some_int);
            gps_input.lon(some_int);
            gps_input.alt(some_float);
            gps_input.hdop(some_float);
            gps_input.vdop(some_float);
            gps_input.vn(some_float);
            gps_input.ve(some_float);
            gps_input.vd(some_float);
            gps_input.speed_accuracy(some_float);
            gps_input.horiz_accuracy(some_float);
            gps_input.vert_accuracy(some_float);
            gps_input.satellites_visible(some_sbyte);
        }
        public static void onGPS_RTCM_DATA(GPS_RTCM_DATA gps_rtcm_data)
        {
            var some_flags = gps_rtcm_data.flags() ;
            var some_len = gps_rtcm_data.len() ;
            {
                var item = gps_rtcm_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(GPS_RTCM_DATA gps_rtcm_data)
        {
            gps_rtcm_data.flags(some_sbyte);
            gps_rtcm_data.len(some_sbyte);
            {
                var item = gps_rtcm_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onHIGH_LATENCY(HIGH_LATENCY high_latency)
        {
            {
                var item = high_latency.base_mode();
                if(item.HasValue)
                    some_MAV_MODE_FLAG = item.Value ;
            }
            var some_custom_mode = high_latency.custom_mode() ;
            {
                var item = high_latency.landed_state();
                if(item.HasValue)
                    some_MAV_LANDED_STATE = item.Value ;
            }
            var some_roll = high_latency.roll() ;
            var some_pitch = high_latency.pitch() ;
            var some_heading = high_latency.heading() ;
            var some_throttle = high_latency.throttle() ;
            var some_heading_sp = high_latency.heading_sp() ;
            var some_latitude = high_latency.latitude() ;
            var some_longitude = high_latency.longitude() ;
            var some_altitude_amsl = high_latency.altitude_amsl() ;
            var some_altitude_sp = high_latency.altitude_sp() ;
            var some_airspeed = high_latency.airspeed() ;
            var some_airspeed_sp = high_latency.airspeed_sp() ;
            var some_groundspeed = high_latency.groundspeed() ;
            var some_climb_rate = high_latency.climb_rate() ;
            var some_gps_nsat = high_latency.gps_nsat() ;
            {
                var item = high_latency.gps_fix_type();
                if(item.HasValue)
                    some_GPS_FIX_TYPE = item.Value ;
            }
            var some_battery_remaining = high_latency.battery_remaining() ;
            var some_temperature = high_latency.temperature() ;
            var some_temperature_air = high_latency.temperature_air() ;
            var some_failsafe = high_latency.failsafe() ;
            var some_wp_num = high_latency.wp_num() ;
            var some_wp_distance = high_latency.wp_distance() ;
        }
        public static void fill(HIGH_LATENCY high_latency)
        {
            high_latency.base_mode(some_MAV_MODE_FLAG);
            high_latency.custom_mode(some_int);
            high_latency.landed_state(some_MAV_LANDED_STATE);
            high_latency.roll(some_short);
            high_latency.pitch(some_short);
            high_latency.heading(some_short);
            high_latency.throttle(some_sbyte);
            high_latency.heading_sp(some_short);
            high_latency.latitude(some_int);
            high_latency.longitude(some_int);
            high_latency.altitude_amsl(some_short);
            high_latency.altitude_sp(some_short);
            high_latency.airspeed(some_sbyte);
            high_latency.airspeed_sp(some_sbyte);
            high_latency.groundspeed(some_sbyte);
            high_latency.climb_rate(some_sbyte);
            high_latency.gps_nsat(some_sbyte);
            high_latency.gps_fix_type(some_GPS_FIX_TYPE);
            high_latency.battery_remaining(some_sbyte);
            high_latency.temperature(some_sbyte);
            high_latency.temperature_air(some_sbyte);
            high_latency.failsafe(some_sbyte);
            high_latency.wp_num(some_sbyte);
            high_latency.wp_distance(some_short);
        }
        public static void onVIBRATION(VIBRATION vibration)
        {
            var some_time_usec = vibration.time_usec() ;
            var some_vibration_x = vibration.vibration_x() ;
            var some_vibration_y = vibration.vibration_y() ;
            var some_vibration_z = vibration.vibration_z() ;
            var some_clipping_0 = vibration.clipping_0() ;
            var some_clipping_1 = vibration.clipping_1() ;
            var some_clipping_2 = vibration.clipping_2() ;
        }
        public static void fill(VIBRATION vibration)
        {
            vibration.time_usec(some_long);
            vibration.vibration_x(some_float);
            vibration.vibration_y(some_float);
            vibration.vibration_z(some_float);
            vibration.clipping_0(some_int);
            vibration.clipping_1(some_int);
            vibration.clipping_2(some_int);
        }
        public static void onHOME_POSITION(HOME_POSITION home_position)
        {
            var some_latitude = home_position.latitude() ;
            var some_longitude = home_position.longitude() ;
            var some_altitude = home_position.altitude() ;
            var some_x = home_position.x() ;
            var some_y = home_position.y() ;
            var some_z = home_position.z() ;
            {
                var item = home_position.q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_approach_x = home_position.approach_x() ;
            var some_approach_y = home_position.approach_y() ;
            var some_approach_z = home_position.approach_z() ;
            {
                var item = home_position.time_usec();
                if(item.HasValue)
                    some_long = item.Value ;
            }
        }
        public static void fill(HOME_POSITION home_position)
        {
            home_position.latitude(some_int);
            home_position.longitude(some_int);
            home_position.altitude(some_int);
            home_position.x(some_float);
            home_position.y(some_float);
            home_position.z(some_float);
            {
                var item = home_position.q();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            home_position.approach_x(some_float);
            home_position.approach_y(some_float);
            home_position.approach_z(some_float);
            home_position.time_usec(some_long);
        }
        public static void onSET_HOME_POSITION(SET_HOME_POSITION set_home_position)
        {
            var some_target_system = set_home_position.target_system() ;
            var some_latitude = set_home_position.latitude() ;
            var some_longitude = set_home_position.longitude() ;
            var some_altitude = set_home_position.altitude() ;
            var some_x = set_home_position.x() ;
            var some_y = set_home_position.y() ;
            var some_z = set_home_position.z() ;
            {
                var item = set_home_position.q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_approach_x = set_home_position.approach_x() ;
            var some_approach_y = set_home_position.approach_y() ;
            var some_approach_z = set_home_position.approach_z() ;
            {
                var item = set_home_position.time_usec();
                if(item.HasValue)
                    some_long = item.Value ;
            }
        }
        public static void fill(SET_HOME_POSITION set_home_position)
        {
            set_home_position.target_system(some_sbyte);
            set_home_position.latitude(some_int);
            set_home_position.longitude(some_int);
            set_home_position.altitude(some_int);
            set_home_position.x(some_float);
            set_home_position.y(some_float);
            set_home_position.z(some_float);
            {
                var item = set_home_position.q();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            set_home_position.approach_x(some_float);
            set_home_position.approach_y(some_float);
            set_home_position.approach_z(some_float);
            set_home_position.time_usec(some_long);
        }
        public static void onMESSAGE_INTERVAL(MESSAGE_INTERVAL message_interval)
        {
            var some_message_id = message_interval.message_id() ;
            var some_interval_us = message_interval.interval_us() ;
        }
        public static void fill(MESSAGE_INTERVAL message_interval)
        {
            message_interval.message_id(some_short);
            message_interval.interval_us(some_int);
        }
        public static void onEXTENDED_SYS_STATE(EXTENDED_SYS_STATE extended_sys_state)
        {
            {
                var item = extended_sys_state.vtol_state();
                if(item.HasValue)
                    some_MAV_VTOL_STATE = item.Value ;
            }
            {
                var item = extended_sys_state.landed_state();
                if(item.HasValue)
                    some_MAV_LANDED_STATE = item.Value ;
            }
        }
        public static void fill(EXTENDED_SYS_STATE extended_sys_state)
        {
            extended_sys_state.vtol_state(some_MAV_VTOL_STATE);
            extended_sys_state.landed_state(some_MAV_LANDED_STATE);
        }
        public static void onADSB_VEHICLE(ADSB_VEHICLE adsb_vehicle)
        {
            var some_ICAO_address = adsb_vehicle.ICAO_address() ;
            var some_lat = adsb_vehicle.lat() ;
            var some_lon = adsb_vehicle.lon() ;
            {
                var item = adsb_vehicle.altitude_type();
                if(item.HasValue)
                    some_ADSB_ALTITUDE_TYPE = item.Value ;
            }
            var some_altitude = adsb_vehicle.altitude() ;
            var some_heading = adsb_vehicle.heading() ;
            var some_hor_velocity = adsb_vehicle.hor_velocity() ;
            var some_ver_velocity = adsb_vehicle.ver_velocity() ;
            {
                var item = adsb_vehicle.callsign();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = adsb_vehicle.emitter_type();
                if(item.HasValue)
                    some_ADSB_EMITTER_TYPE = item.Value ;
            }
            var some_tslc = adsb_vehicle.tslc() ;
            {
                var item = adsb_vehicle.flags();
                if(item.HasValue)
                    some_ADSB_FLAGS = item.Value ;
            }
            var some_squawk = adsb_vehicle.squawk() ;
        }
        public static void fill(ADSB_VEHICLE adsb_vehicle)
        {
            adsb_vehicle.ICAO_address(some_int);
            adsb_vehicle.lat(some_int);
            adsb_vehicle.lon(some_int);
            adsb_vehicle.altitude_type(some_ADSB_ALTITUDE_TYPE);
            adsb_vehicle.altitude(some_int);
            adsb_vehicle.heading(some_short);
            adsb_vehicle.hor_velocity(some_short);
            adsb_vehicle.ver_velocity(some_short);
            adsb_vehicle.callsign(some_string, null);
            adsb_vehicle.emitter_type(some_ADSB_EMITTER_TYPE);
            adsb_vehicle.tslc(some_sbyte);
            adsb_vehicle.flags(some_ADSB_FLAGS);
            adsb_vehicle.squawk(some_short);
        }
        public static void onCOLLISION(COLLISION collision)
        {
            {
                var item = collision.sRc();
                if(item.HasValue)
                    some_MAV_COLLISION_SRC = item.Value ;
            }
            var some_id = collision.id() ;
            {
                var item = collision.action();
                if(item.HasValue)
                    some_MAV_COLLISION_ACTION = item.Value ;
            }
            {
                var item = collision.threat_level();
                if(item.HasValue)
                    some_MAV_COLLISION_THREAT_LEVEL = item.Value ;
            }
            var some_time_to_minimum_delta = collision.time_to_minimum_delta() ;
            var some_altitude_minimum_delta = collision.altitude_minimum_delta() ;
            var some_horizontal_minimum_delta = collision.horizontal_minimum_delta() ;
        }
        public static void fill(COLLISION collision)
        {
            collision.sRc(some_MAV_COLLISION_SRC);
            collision.id(some_int);
            collision.action(some_MAV_COLLISION_ACTION);
            collision.threat_level(some_MAV_COLLISION_THREAT_LEVEL);
            collision.time_to_minimum_delta(some_float);
            collision.altitude_minimum_delta(some_float);
            collision.horizontal_minimum_delta(some_float);
        }
        public static void onV2_EXTENSION(V2_EXTENSION v2_extension)
        {
            var some_target_network = v2_extension.target_network() ;
            var some_target_system = v2_extension.target_system() ;
            var some_target_component = v2_extension.target_component() ;
            var some_message_type = v2_extension.message_type() ;
            {
                var item = v2_extension.payload();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(V2_EXTENSION v2_extension)
        {
            v2_extension.target_network(some_sbyte);
            v2_extension.target_system(some_sbyte);
            v2_extension.target_component(some_sbyte);
            v2_extension.message_type(some_short);
            {
                var item = v2_extension.payload();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onMEMORY_VECT(MEMORY_VECT memory_vect)
        {
            var some_address = memory_vect.address() ;
            var some_ver = memory_vect.ver() ;
            var some_typE = memory_vect.typE() ;
            {
                var item = memory_vect.value();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(MEMORY_VECT memory_vect)
        {
            memory_vect.address(some_short);
            memory_vect.ver(some_sbyte);
            memory_vect.typE(some_sbyte);
            {
                var item = memory_vect.value();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onDEBUG_VECT(DEBUG_VECT debug_vect)
        {
            {
                var item = debug_vect.name();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_time_usec = debug_vect.time_usec() ;
            var some_x = debug_vect.x() ;
            var some_y = debug_vect.y() ;
            var some_z = debug_vect.z() ;
        }
        public static void fill(DEBUG_VECT debug_vect)
        {
            debug_vect.name(some_string, null);
            debug_vect.time_usec(some_long);
            debug_vect.x(some_float);
            debug_vect.y(some_float);
            debug_vect.z(some_float);
        }
        public static void onNAMED_VALUE_FLOAT(NAMED_VALUE_FLOAT named_value_float)
        {
            var some_time_boot_ms = named_value_float.time_boot_ms() ;
            {
                var item = named_value_float.name();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_value = named_value_float.value() ;
        }
        public static void fill(NAMED_VALUE_FLOAT named_value_float)
        {
            named_value_float.time_boot_ms(some_int);
            named_value_float.name(some_string, null);
            named_value_float.value(some_float);
        }
        public static void onNAMED_VALUE_INT(NAMED_VALUE_INT named_value_int)
        {
            var some_time_boot_ms = named_value_int.time_boot_ms() ;
            {
                var item = named_value_int.name();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_value = named_value_int.value() ;
        }
        public static void fill(NAMED_VALUE_INT named_value_int)
        {
            named_value_int.time_boot_ms(some_int);
            named_value_int.name(some_string, null);
            named_value_int.value(some_int);
        }
        public static void onSTATUSTEXT(STATUSTEXT statustext)
        {
            {
                var item = statustext.severity();
                if(item.HasValue)
                    some_MAV_SEVERITY = item.Value ;
            }
            {
                var item = statustext.text();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void fill(STATUSTEXT statustext)
        {
            statustext.severity(some_MAV_SEVERITY);
            statustext.text(some_string, null);
        }
        public static void onDEBUG(DEBUG debug)
        {
            var some_time_boot_ms = debug.time_boot_ms() ;
            var some_ind = debug.ind() ;
            var some_value = debug.value() ;
        }
        public static void fill(DEBUG debug)
        {
            debug.time_boot_ms(some_int);
            debug.ind(some_sbyte);
            debug.value(some_float);
        }
        public static void onSETUP_SIGNING(SETUP_SIGNING setup_signing)
        {
            var some_target_system = setup_signing.target_system() ;
            var some_target_component = setup_signing.target_component() ;
            {
                var item = setup_signing.secret_key();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            var some_initial_timestamp = setup_signing.initial_timestamp() ;
        }
        public static void fill(SETUP_SIGNING setup_signing)
        {
            setup_signing.target_system(some_sbyte);
            setup_signing.target_component(some_sbyte);
            {
                var item = setup_signing.secret_key();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            setup_signing.initial_timestamp(some_long);
        }
        public static void onBUTTON_CHANGE(BUTTON_CHANGE button_change)
        {
            var some_time_boot_ms = button_change.time_boot_ms() ;
            var some_last_change_ms = button_change.last_change_ms() ;
            var some_state = button_change.state() ;
        }
        public static void fill(BUTTON_CHANGE button_change)
        {
            button_change.time_boot_ms(some_int);
            button_change.last_change_ms(some_int);
            button_change.state(some_sbyte);
        }
        public static void onPLAY_TUNE(PLAY_TUNE play_tune)
        {
            var some_target_system = play_tune.target_system() ;
            var some_target_component = play_tune.target_component() ;
            {
                var item = play_tune.tune();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void fill(PLAY_TUNE play_tune)
        {
            play_tune.target_system(some_sbyte);
            play_tune.target_component(some_sbyte);
            play_tune.tune(some_string, null);
        }
        public static void onCAMERA_INFORMATION(CAMERA_INFORMATION camera_information)
        {
            var some_time_boot_ms = camera_information.time_boot_ms() ;
            {
                var item = camera_information.vendor_name();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            {
                var item = camera_information.model_name();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            var some_firmware_version = camera_information.firmware_version() ;
            var some_focal_length = camera_information.focal_length() ;
            var some_sensor_size_h = camera_information.sensor_size_h() ;
            var some_sensor_size_v = camera_information.sensor_size_v() ;
            var some_resolution_h = camera_information.resolution_h() ;
            var some_resolution_v = camera_information.resolution_v() ;
            var some_lens_id = camera_information.lens_id() ;
            {
                var item = camera_information.flags();
                if(item.HasValue)
                    some_CAMERA_CAP_FLAGS = item.Value ;
            }
            var some_cam_definition_version = camera_information.cam_definition_version() ;
            {
                var item = camera_information.cam_definition_uri();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void fill(CAMERA_INFORMATION camera_information)
        {
            camera_information.time_boot_ms(some_int);
            {
                var item = camera_information.vendor_name();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            {
                var item = camera_information.model_name();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            camera_information.firmware_version(some_int);
            camera_information.focal_length(some_float);
            camera_information.sensor_size_h(some_float);
            camera_information.sensor_size_v(some_float);
            camera_information.resolution_h(some_short);
            camera_information.resolution_v(some_short);
            camera_information.lens_id(some_sbyte);
            camera_information.flags(some_CAMERA_CAP_FLAGS);
            camera_information.cam_definition_version(some_short);
            camera_information.cam_definition_uri(some_string, null);
        }
        public static void onCAMERA_SETTINGS(CAMERA_SETTINGS camera_settings)
        {
            var some_time_boot_ms = camera_settings.time_boot_ms() ;
            {
                var item = camera_settings.mode_id();
                if(item.HasValue)
                    some_CAMERA_MODE = item.Value ;
            }
        }
        public static void fill(CAMERA_SETTINGS camera_settings)
        {
            camera_settings.time_boot_ms(some_int);
            camera_settings.mode_id(some_CAMERA_MODE);
        }
        public static void onSTORAGE_INFORMATION(STORAGE_INFORMATION storage_information)
        {
            var some_time_boot_ms = storage_information.time_boot_ms() ;
            var some_storage_id = storage_information.storage_id() ;
            var some_storage_count = storage_information.storage_count() ;
            var some_status = storage_information.status() ;
            var some_total_capacity = storage_information.total_capacity() ;
            var some_used_capacity = storage_information.used_capacity() ;
            var some_available_capacity = storage_information.available_capacity() ;
            var some_read_speed = storage_information.read_speed() ;
            var some_write_speed = storage_information.write_speed() ;
        }
        public static void fill(STORAGE_INFORMATION storage_information)
        {
            storage_information.time_boot_ms(some_int);
            storage_information.storage_id(some_sbyte);
            storage_information.storage_count(some_sbyte);
            storage_information.status(some_sbyte);
            storage_information.total_capacity(some_float);
            storage_information.used_capacity(some_float);
            storage_information.available_capacity(some_float);
            storage_information.read_speed(some_float);
            storage_information.write_speed(some_float);
        }
        public static void onCAMERA_CAPTURE_STATUS(CAMERA_CAPTURE_STATUS camera_capture_status)
        {
            var some_time_boot_ms = camera_capture_status.time_boot_ms() ;
            var some_image_status = camera_capture_status.image_status() ;
            var some_video_status = camera_capture_status.video_status() ;
            var some_image_interval = camera_capture_status.image_interval() ;
            var some_recording_time_ms = camera_capture_status.recording_time_ms() ;
            var some_available_capacity = camera_capture_status.available_capacity() ;
        }
        public static void fill(CAMERA_CAPTURE_STATUS camera_capture_status)
        {
            camera_capture_status.time_boot_ms(some_int);
            camera_capture_status.image_status(some_sbyte);
            camera_capture_status.video_status(some_sbyte);
            camera_capture_status.image_interval(some_float);
            camera_capture_status.recording_time_ms(some_int);
            camera_capture_status.available_capacity(some_float);
        }
        public static void onCAMERA_IMAGE_CAPTURED(CAMERA_IMAGE_CAPTURED camera_image_captured)
        {
            var some_time_boot_ms = camera_image_captured.time_boot_ms() ;
            var some_time_utc = camera_image_captured.time_utc() ;
            var some_camera_id = camera_image_captured.camera_id() ;
            var some_lat = camera_image_captured.lat() ;
            var some_lon = camera_image_captured.lon() ;
            var some_alt = camera_image_captured.alt() ;
            var some_relative_alt = camera_image_captured.relative_alt() ;
            {
                var item = camera_image_captured.q();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_image_index = camera_image_captured.image_index() ;
            var some_capture_result = camera_image_captured.capture_result() ;
            {
                var item = camera_image_captured.file_url();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void fill(CAMERA_IMAGE_CAPTURED camera_image_captured)
        {
            camera_image_captured.time_boot_ms(some_int);
            camera_image_captured.time_utc(some_long);
            camera_image_captured.camera_id(some_sbyte);
            camera_image_captured.lat(some_int);
            camera_image_captured.lon(some_int);
            camera_image_captured.alt(some_int);
            camera_image_captured.relative_alt(some_int);
            {
                var item = camera_image_captured.q();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            camera_image_captured.image_index(some_int);
            camera_image_captured.capture_result(some_sbyte);
            camera_image_captured.file_url(some_string, null);
        }
        public static void onFLIGHT_INFORMATION(FLIGHT_INFORMATION flight_information)
        {
            var some_time_boot_ms = flight_information.time_boot_ms() ;
            var some_arming_time_utc = flight_information.arming_time_utc() ;
            var some_takeoff_time_utc = flight_information.takeoff_time_utc() ;
            var some_flight_uuid = flight_information.flight_uuid() ;
        }
        public static void fill(FLIGHT_INFORMATION flight_information)
        {
            flight_information.time_boot_ms(some_int);
            flight_information.arming_time_utc(some_long);
            flight_information.takeoff_time_utc(some_long);
            flight_information.flight_uuid(some_long);
        }
        public static void onMOUNT_ORIENTATION(MOUNT_ORIENTATION mount_orientation)
        {
            var some_time_boot_ms = mount_orientation.time_boot_ms() ;
            var some_roll = mount_orientation.roll() ;
            var some_pitch = mount_orientation.pitch() ;
            var some_yaw = mount_orientation.yaw() ;
        }
        public static void fill(MOUNT_ORIENTATION mount_orientation)
        {
            mount_orientation.time_boot_ms(some_int);
            mount_orientation.roll(some_float);
            mount_orientation.pitch(some_float);
            mount_orientation.yaw(some_float);
        }
        public static void onLOGGING_DATA(LOGGING_DATA logging_data)
        {
            var some_target_system = logging_data.target_system() ;
            var some_target_component = logging_data.target_component() ;
            var some_sequence = logging_data.sequence() ;
            var some_length = logging_data.length() ;
            var some_first_message_offset = logging_data.first_message_offset() ;
            {
                var item = logging_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(LOGGING_DATA logging_data)
        {
            logging_data.target_system(some_sbyte);
            logging_data.target_component(some_sbyte);
            logging_data.sequence(some_short);
            logging_data.length(some_sbyte);
            logging_data.first_message_offset(some_sbyte);
            {
                var item = logging_data.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onLOGGING_DATA_ACKED(LOGGING_DATA_ACKED logging_data_acked)
        {
            var some_target_system = logging_data_acked.target_system() ;
            var some_target_component = logging_data_acked.target_component() ;
            var some_sequence = logging_data_acked.sequence() ;
            var some_length = logging_data_acked.length() ;
            var some_first_message_offset = logging_data_acked.first_message_offset() ;
            {
                var item = logging_data_acked.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(LOGGING_DATA_ACKED logging_data_acked)
        {
            logging_data_acked.target_system(some_sbyte);
            logging_data_acked.target_component(some_sbyte);
            logging_data_acked.sequence(some_short);
            logging_data_acked.length(some_sbyte);
            logging_data_acked.first_message_offset(some_sbyte);
            {
                var item = logging_data_acked.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onLOGGING_ACK(LOGGING_ACK logging_ack)
        {
            var some_target_system = logging_ack.target_system() ;
            var some_target_component = logging_ack.target_component() ;
            var some_sequence = logging_ack.sequence() ;
        }
        public static void fill(LOGGING_ACK logging_ack)
        {
            logging_ack.target_system(some_sbyte);
            logging_ack.target_component(some_sbyte);
            logging_ack.sequence(some_short);
        }
        public static void onVIDEO_STREAM_INFORMATION(VIDEO_STREAM_INFORMATION video_stream_information)
        {
            var some_camera_id = video_stream_information.camera_id() ;
            var some_status = video_stream_information.status() ;
            var some_framerate = video_stream_information.framerate() ;
            var some_resolution_h = video_stream_information.resolution_h() ;
            var some_resolution_v = video_stream_information.resolution_v() ;
            var some_bitrate = video_stream_information.bitrate() ;
            var some_rotation = video_stream_information.rotation() ;
            {
                var item = video_stream_information.uri();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void fill(VIDEO_STREAM_INFORMATION video_stream_information)
        {
            video_stream_information.camera_id(some_sbyte);
            video_stream_information.status(some_sbyte);
            video_stream_information.framerate(some_float);
            video_stream_information.resolution_h(some_short);
            video_stream_information.resolution_v(some_short);
            video_stream_information.bitrate(some_int);
            video_stream_information.rotation(some_short);
            video_stream_information.uri(some_string, null);
        }
        public static void onSET_VIDEO_STREAM_SETTINGS(SET_VIDEO_STREAM_SETTINGS set_video_stream_settings)
        {
            var some_target_system = set_video_stream_settings.target_system() ;
            var some_target_component = set_video_stream_settings.target_component() ;
            var some_camera_id = set_video_stream_settings.camera_id() ;
            var some_framerate = set_video_stream_settings.framerate() ;
            var some_resolution_h = set_video_stream_settings.resolution_h() ;
            var some_resolution_v = set_video_stream_settings.resolution_v() ;
            var some_bitrate = set_video_stream_settings.bitrate() ;
            var some_rotation = set_video_stream_settings.rotation() ;
            {
                var item = set_video_stream_settings.uri();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void fill(SET_VIDEO_STREAM_SETTINGS set_video_stream_settings)
        {
            set_video_stream_settings.target_system(some_sbyte);
            set_video_stream_settings.target_component(some_sbyte);
            set_video_stream_settings.camera_id(some_sbyte);
            set_video_stream_settings.framerate(some_float);
            set_video_stream_settings.resolution_h(some_short);
            set_video_stream_settings.resolution_v(some_short);
            set_video_stream_settings.bitrate(some_int);
            set_video_stream_settings.rotation(some_short);
            set_video_stream_settings.uri(some_string, null);
        }
        public static void onWIFI_CONFIG_AP(WIFI_CONFIG_AP wifi_config_ap)
        {
            {
                var item = wifi_config_ap.ssid();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = wifi_config_ap.password();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
        }
        public static void fill(WIFI_CONFIG_AP wifi_config_ap)
        {
            wifi_config_ap.ssid(some_string, null);
            wifi_config_ap.password(some_string, null);
        }
        public static void onPROTOCOL_VERSION(PROTOCOL_VERSION protocol_version)
        {
            var some_version = protocol_version.version() ;
            var some_min_version = protocol_version.min_version() ;
            var some_max_version = protocol_version.max_version() ;
            {
                var item = protocol_version.spec_version_hash();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            {
                var item = protocol_version.library_version_hash();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(PROTOCOL_VERSION protocol_version)
        {
            protocol_version.version(some_short);
            protocol_version.min_version(some_short);
            protocol_version.max_version(some_short);
            {
                var item = protocol_version.spec_version_hash();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            {
                var item = protocol_version.library_version_hash();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onUAVCAN_NODE_STATUS(UAVCAN_NODE_STATUS uavcan_node_status)
        {
            var some_time_usec = uavcan_node_status.time_usec() ;
            var some_uptime_sec = uavcan_node_status.uptime_sec() ;
            {
                var item = uavcan_node_status.health();
                if(item.HasValue)
                    some_UAVCAN_NODE_HEALTH = item.Value ;
            }
            {
                var item = uavcan_node_status.mode();
                if(item.HasValue)
                    some_UAVCAN_NODE_MODE = item.Value ;
            }
            var some_sub_mode = uavcan_node_status.sub_mode() ;
            var some_vendor_specific_status_code = uavcan_node_status.vendor_specific_status_code() ;
        }
        public static void fill(UAVCAN_NODE_STATUS uavcan_node_status)
        {
            uavcan_node_status.time_usec(some_long);
            uavcan_node_status.uptime_sec(some_int);
            uavcan_node_status.health(some_UAVCAN_NODE_HEALTH);
            uavcan_node_status.mode(some_UAVCAN_NODE_MODE);
            uavcan_node_status.sub_mode(some_sbyte);
            uavcan_node_status.vendor_specific_status_code(some_short);
        }
        public static void onUAVCAN_NODE_INFO(UAVCAN_NODE_INFO uavcan_node_info)
        {
            var some_time_usec = uavcan_node_info.time_usec() ;
            var some_uptime_sec = uavcan_node_info.uptime_sec() ;
            {
                var item = uavcan_node_info.name();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_hw_version_major = uavcan_node_info.hw_version_major() ;
            var some_hw_version_minor = uavcan_node_info.hw_version_minor() ;
            {
                var item = uavcan_node_info.hw_unique_id();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            var some_sw_version_major = uavcan_node_info.sw_version_major() ;
            var some_sw_version_minor = uavcan_node_info.sw_version_minor() ;
            var some_sw_vcs_commit = uavcan_node_info.sw_vcs_commit() ;
        }
        public static void fill(UAVCAN_NODE_INFO uavcan_node_info)
        {
            uavcan_node_info.time_usec(some_long);
            uavcan_node_info.uptime_sec(some_int);
            uavcan_node_info.name(some_string, null);
            uavcan_node_info.hw_version_major(some_sbyte);
            uavcan_node_info.hw_version_minor(some_sbyte);
            {
                var item = uavcan_node_info.hw_unique_id();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            uavcan_node_info.sw_version_major(some_sbyte);
            uavcan_node_info.sw_version_minor(some_sbyte);
            uavcan_node_info.sw_vcs_commit(some_int);
        }
        public static void onPARAM_EXT_REQUEST_READ(PARAM_EXT_REQUEST_READ param_ext_request_read)
        {
            var some_target_system = param_ext_request_read.target_system() ;
            var some_target_component = param_ext_request_read.target_component() ;
            {
                var item = param_ext_request_read.param_id();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_param_index = param_ext_request_read.param_index() ;
        }
        public static void fill(PARAM_EXT_REQUEST_READ param_ext_request_read)
        {
            param_ext_request_read.target_system(some_sbyte);
            param_ext_request_read.target_component(some_sbyte);
            param_ext_request_read.param_id(some_string, null);
            param_ext_request_read.param_index(some_short);
        }
        public static void onPARAM_EXT_REQUEST_LIST(PARAM_EXT_REQUEST_LIST param_ext_request_list)
        {
            var some_target_system = param_ext_request_list.target_system() ;
            var some_target_component = param_ext_request_list.target_component() ;
        }
        public static void fill(PARAM_EXT_REQUEST_LIST param_ext_request_list)
        {
            param_ext_request_list.target_system(some_sbyte);
            param_ext_request_list.target_component(some_sbyte);
        }
        public static void onPARAM_EXT_VALUE(PARAM_EXT_VALUE param_ext_value)
        {
            {
                var item = param_ext_value.param_id();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = param_ext_value.param_value();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = param_ext_value.param_type();
                if(item.HasValue)
                    some_MAV_PARAM_EXT_TYPE = item.Value ;
            }
            var some_param_count = param_ext_value.param_count() ;
            var some_param_index = param_ext_value.param_index() ;
        }
        public static void fill(PARAM_EXT_VALUE param_ext_value)
        {
            param_ext_value.param_id(some_string, null);
            param_ext_value.param_value(some_string, null);
            param_ext_value.param_type(some_MAV_PARAM_EXT_TYPE);
            param_ext_value.param_count(some_short);
            param_ext_value.param_index(some_short);
        }
        public static void onPARAM_EXT_SET(PARAM_EXT_SET param_ext_set)
        {
            var some_target_system = param_ext_set.target_system() ;
            var some_target_component = param_ext_set.target_component() ;
            {
                var item = param_ext_set.param_id();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = param_ext_set.param_value();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = param_ext_set.param_type();
                if(item.HasValue)
                    some_MAV_PARAM_EXT_TYPE = item.Value ;
            }
        }
        public static void fill(PARAM_EXT_SET param_ext_set)
        {
            param_ext_set.target_system(some_sbyte);
            param_ext_set.target_component(some_sbyte);
            param_ext_set.param_id(some_string, null);
            param_ext_set.param_value(some_string, null);
            param_ext_set.param_type(some_MAV_PARAM_EXT_TYPE);
        }
        public static void onPARAM_EXT_ACK(PARAM_EXT_ACK param_ext_ack)
        {
            {
                var item = param_ext_ack.param_id();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = param_ext_ack.param_value();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = param_ext_ack.param_type();
                if(item.HasValue)
                    some_MAV_PARAM_EXT_TYPE = item.Value ;
            }
            {
                var item = param_ext_ack.param_result();
                if(item.HasValue)
                    some_PARAM_ACK = item.Value ;
            }
        }
        public static void fill(PARAM_EXT_ACK param_ext_ack)
        {
            param_ext_ack.param_id(some_string, null);
            param_ext_ack.param_value(some_string, null);
            param_ext_ack.param_type(some_MAV_PARAM_EXT_TYPE);
            param_ext_ack.param_result(some_PARAM_ACK);
        }
        public static void onOBSTACLE_DISTANCE(OBSTACLE_DISTANCE obstacle_distance)
        {
            var some_time_usec = obstacle_distance.time_usec() ;
            {
                var item = obstacle_distance.sensor_type();
                if(item.HasValue)
                    some_MAV_DISTANCE_SENSOR = item.Value ;
            }
            {
                var item = obstacle_distance.distances();
                for(int i = 0; i < item.len(); i++)
                    some_short = item.get(i);
            }
            var some_increment = obstacle_distance.increment() ;
            var some_min_distance = obstacle_distance.min_distance() ;
            var some_max_distance = obstacle_distance.max_distance() ;
        }
        public static void fill(OBSTACLE_DISTANCE obstacle_distance)
        {
            obstacle_distance.time_usec(some_long);
            obstacle_distance.sensor_type(some_MAV_DISTANCE_SENSOR);
            {
                var item = obstacle_distance.distances();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_short, i);
            }
            obstacle_distance.increment(some_sbyte);
            obstacle_distance.min_distance(some_short);
            obstacle_distance.max_distance(some_short);
        }
        public static void onUAVIONIX_ADSB_OUT_CFG(UAVIONIX_ADSB_OUT_CFG uavionix_adsb_out_cfg)
        {
            var some_ICAO = uavionix_adsb_out_cfg.ICAO() ;
            {
                var item = uavionix_adsb_out_cfg.callsign();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            {
                var item = uavionix_adsb_out_cfg.emitterType();
                if(item.HasValue)
                    some_ADSB_EMITTER_TYPE = item.Value ;
            }
            {
                var item = uavionix_adsb_out_cfg.aircraftSize();
                if(item.HasValue)
                    some_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE = item.Value ;
            }
            {
                var item = uavionix_adsb_out_cfg.gpsOffsetLat();
                if(item.HasValue)
                    some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT = item.Value ;
            }
            {
                var item = uavionix_adsb_out_cfg.gpsOffsetLon();
                if(item.HasValue)
                    some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON = item.Value ;
            }
            var some_stallSpeed = uavionix_adsb_out_cfg.stallSpeed() ;
            {
                var item = uavionix_adsb_out_cfg.rfSelect();
                if(item.HasValue)
                    some_UAVIONIX_ADSB_OUT_RF_SELECT = item.Value ;
            }
        }
        public static void fill(UAVIONIX_ADSB_OUT_CFG uavionix_adsb_out_cfg)
        {
            uavionix_adsb_out_cfg.ICAO(some_int);
            uavionix_adsb_out_cfg.callsign(some_string, null);
            uavionix_adsb_out_cfg.emitterType(some_ADSB_EMITTER_TYPE);
            uavionix_adsb_out_cfg.aircraftSize(some_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE);
            uavionix_adsb_out_cfg.gpsOffsetLat(some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT);
            uavionix_adsb_out_cfg.gpsOffsetLon(some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON);
            uavionix_adsb_out_cfg.stallSpeed(some_short);
            uavionix_adsb_out_cfg.rfSelect(some_UAVIONIX_ADSB_OUT_RF_SELECT);
        }
        public static void onUAVIONIX_ADSB_OUT_DYNAMIC(UAVIONIX_ADSB_OUT_DYNAMIC uavionix_adsb_out_dynamic)
        {
            var some_utcTime = uavionix_adsb_out_dynamic.utcTime() ;
            var some_gpsLat = uavionix_adsb_out_dynamic.gpsLat() ;
            var some_gpsLon = uavionix_adsb_out_dynamic.gpsLon() ;
            var some_gpsAlt = uavionix_adsb_out_dynamic.gpsAlt() ;
            {
                var item = uavionix_adsb_out_dynamic.gpsFix();
                if(item.HasValue)
                    some_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX = item.Value ;
            }
            var some_numSats = uavionix_adsb_out_dynamic.numSats() ;
            var some_baroAltMSL = uavionix_adsb_out_dynamic.baroAltMSL() ;
            var some_accuracyHor = uavionix_adsb_out_dynamic.accuracyHor() ;
            var some_accuracyVert = uavionix_adsb_out_dynamic.accuracyVert() ;
            var some_accuracyVel = uavionix_adsb_out_dynamic.accuracyVel() ;
            var some_velVert = uavionix_adsb_out_dynamic.velVert() ;
            var some_velNS = uavionix_adsb_out_dynamic.velNS() ;
            var some_VelEW = uavionix_adsb_out_dynamic.VelEW() ;
            {
                var item = uavionix_adsb_out_dynamic.emergencyStatus();
                if(item.HasValue)
                    some_UAVIONIX_ADSB_EMERGENCY_STATUS = item.Value ;
            }
            {
                var item = uavionix_adsb_out_dynamic.state();
                if(item.HasValue)
                    some_UAVIONIX_ADSB_OUT_DYNAMIC_STATE = item.Value ;
            }
            var some_squawk = uavionix_adsb_out_dynamic.squawk() ;
        }
        public static void fill(UAVIONIX_ADSB_OUT_DYNAMIC uavionix_adsb_out_dynamic)
        {
            uavionix_adsb_out_dynamic.utcTime(some_int);
            uavionix_adsb_out_dynamic.gpsLat(some_int);
            uavionix_adsb_out_dynamic.gpsLon(some_int);
            uavionix_adsb_out_dynamic.gpsAlt(some_int);
            uavionix_adsb_out_dynamic.gpsFix(some_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX);
            uavionix_adsb_out_dynamic.numSats(some_sbyte);
            uavionix_adsb_out_dynamic.baroAltMSL(some_int);
            uavionix_adsb_out_dynamic.accuracyHor(some_int);
            uavionix_adsb_out_dynamic.accuracyVert(some_short);
            uavionix_adsb_out_dynamic.accuracyVel(some_short);
            uavionix_adsb_out_dynamic.velVert(some_short);
            uavionix_adsb_out_dynamic.velNS(some_short);
            uavionix_adsb_out_dynamic.VelEW(some_short);
            uavionix_adsb_out_dynamic.emergencyStatus(some_UAVIONIX_ADSB_EMERGENCY_STATUS);
            uavionix_adsb_out_dynamic.state(some_UAVIONIX_ADSB_OUT_DYNAMIC_STATE);
            uavionix_adsb_out_dynamic.squawk(some_short);
        }
        public static void onUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT uavionix_adsb_transceiver_health_report)
        {
            {
                var item = uavionix_adsb_transceiver_health_report.rfHealth();
                if(item.HasValue)
                    some_UAVIONIX_ADSB_RF_HEALTH = item.Value ;
            }
        }
        public static void fill(UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT uavionix_adsb_transceiver_health_report)
        {
            uavionix_adsb_transceiver_health_report.rfHealth(some_UAVIONIX_ADSB_RF_HEALTH);
        }
        public static void onSENSOR_OFFSETS(SENSOR_OFFSETS sensor_offsets)
        {
            var some_mag_ofs_x = sensor_offsets.mag_ofs_x() ;
            var some_mag_ofs_y = sensor_offsets.mag_ofs_y() ;
            var some_mag_ofs_z = sensor_offsets.mag_ofs_z() ;
            var some_mag_declination = sensor_offsets.mag_declination() ;
            var some_raw_press = sensor_offsets.raw_press() ;
            var some_raw_temp = sensor_offsets.raw_temp() ;
            var some_gyro_cal_x = sensor_offsets.gyro_cal_x() ;
            var some_gyro_cal_y = sensor_offsets.gyro_cal_y() ;
            var some_gyro_cal_z = sensor_offsets.gyro_cal_z() ;
            var some_accel_cal_x = sensor_offsets.accel_cal_x() ;
            var some_accel_cal_y = sensor_offsets.accel_cal_y() ;
            var some_accel_cal_z = sensor_offsets.accel_cal_z() ;
        }
        public static void fill(SENSOR_OFFSETS sensor_offsets)
        {
            sensor_offsets.mag_ofs_x(some_short);
            sensor_offsets.mag_ofs_y(some_short);
            sensor_offsets.mag_ofs_z(some_short);
            sensor_offsets.mag_declination(some_float);
            sensor_offsets.raw_press(some_int);
            sensor_offsets.raw_temp(some_int);
            sensor_offsets.gyro_cal_x(some_float);
            sensor_offsets.gyro_cal_y(some_float);
            sensor_offsets.gyro_cal_z(some_float);
            sensor_offsets.accel_cal_x(some_float);
            sensor_offsets.accel_cal_y(some_float);
            sensor_offsets.accel_cal_z(some_float);
        }
        public static void onSET_MAG_OFFSETS(SET_MAG_OFFSETS set_mag_offsets)
        {
            var some_target_system = set_mag_offsets.target_system() ;
            var some_target_component = set_mag_offsets.target_component() ;
            var some_mag_ofs_x = set_mag_offsets.mag_ofs_x() ;
            var some_mag_ofs_y = set_mag_offsets.mag_ofs_y() ;
            var some_mag_ofs_z = set_mag_offsets.mag_ofs_z() ;
        }
        public static void fill(SET_MAG_OFFSETS set_mag_offsets)
        {
            set_mag_offsets.target_system(some_sbyte);
            set_mag_offsets.target_component(some_sbyte);
            set_mag_offsets.mag_ofs_x(some_short);
            set_mag_offsets.mag_ofs_y(some_short);
            set_mag_offsets.mag_ofs_z(some_short);
        }
        public static void onMEMINFO(MEMINFO meminfo)
        {
            var some_brkval = meminfo.brkval() ;
            var some_freemem = meminfo.freemem() ;
            {
                var item = meminfo.freemem32();
                if(item.HasValue)
                    some_int = item.Value ;
            }
        }
        public static void fill(MEMINFO meminfo)
        {
            meminfo.brkval(some_short);
            meminfo.freemem(some_short);
            meminfo.freemem32(some_int);
        }
        public static void onAP_ADC(AP_ADC ap_adc)
        {
            var some_adc1 = ap_adc.adc1() ;
            var some_adc2 = ap_adc.adc2() ;
            var some_adc3 = ap_adc.adc3() ;
            var some_adc4 = ap_adc.adc4() ;
            var some_adc5 = ap_adc.adc5() ;
            var some_adc6 = ap_adc.adc6() ;
        }
        public static void fill(AP_ADC ap_adc)
        {
            ap_adc.adc1(some_short);
            ap_adc.adc2(some_short);
            ap_adc.adc3(some_short);
            ap_adc.adc4(some_short);
            ap_adc.adc5(some_short);
            ap_adc.adc6(some_short);
        }
        public static void onDIGICAM_CONFIGURE(DIGICAM_CONFIGURE digicam_configure)
        {
            var some_target_system = digicam_configure.target_system() ;
            var some_target_component = digicam_configure.target_component() ;
            var some_mode = digicam_configure.mode() ;
            var some_shutter_speed = digicam_configure.shutter_speed() ;
            var some_aperture = digicam_configure.aperture() ;
            var some_iso = digicam_configure.iso() ;
            var some_exposure_type = digicam_configure.exposure_type() ;
            var some_command_id = digicam_configure.command_id() ;
            var some_engine_cut_off = digicam_configure.engine_cut_off() ;
            var some_extra_param = digicam_configure.extra_param() ;
            var some_extra_value = digicam_configure.extra_value() ;
        }
        public static void fill(DIGICAM_CONFIGURE digicam_configure)
        {
            digicam_configure.target_system(some_sbyte);
            digicam_configure.target_component(some_sbyte);
            digicam_configure.mode(some_sbyte);
            digicam_configure.shutter_speed(some_short);
            digicam_configure.aperture(some_sbyte);
            digicam_configure.iso(some_sbyte);
            digicam_configure.exposure_type(some_sbyte);
            digicam_configure.command_id(some_sbyte);
            digicam_configure.engine_cut_off(some_sbyte);
            digicam_configure.extra_param(some_sbyte);
            digicam_configure.extra_value(some_float);
        }
        public static void onDIGICAM_CONTROL(DIGICAM_CONTROL digicam_control)
        {
            var some_target_system = digicam_control.target_system() ;
            var some_target_component = digicam_control.target_component() ;
            var some_session = digicam_control.session() ;
            var some_zoom_pos = digicam_control.zoom_pos() ;
            var some_zoom_step = digicam_control.zoom_step() ;
            var some_focus_lock = digicam_control.focus_lock() ;
            var some_shot = digicam_control.shot() ;
            var some_command_id = digicam_control.command_id() ;
            var some_extra_param = digicam_control.extra_param() ;
            var some_extra_value = digicam_control.extra_value() ;
        }
        public static void fill(DIGICAM_CONTROL digicam_control)
        {
            digicam_control.target_system(some_sbyte);
            digicam_control.target_component(some_sbyte);
            digicam_control.session(some_sbyte);
            digicam_control.zoom_pos(some_sbyte);
            digicam_control.zoom_step(some_sbyte);
            digicam_control.focus_lock(some_sbyte);
            digicam_control.shot(some_sbyte);
            digicam_control.command_id(some_sbyte);
            digicam_control.extra_param(some_sbyte);
            digicam_control.extra_value(some_float);
        }
        public static void onMOUNT_CONFIGURE(MOUNT_CONFIGURE mount_configure)
        {
            var some_target_system = mount_configure.target_system() ;
            var some_target_component = mount_configure.target_component() ;
            {
                var item = mount_configure.mount_mode();
                if(item.HasValue)
                    some_MAV_MOUNT_MODE = item.Value ;
            }
            var some_stab_roll = mount_configure.stab_roll() ;
            var some_stab_pitch = mount_configure.stab_pitch() ;
            var some_stab_yaw = mount_configure.stab_yaw() ;
        }
        public static void fill(MOUNT_CONFIGURE mount_configure)
        {
            mount_configure.target_system(some_sbyte);
            mount_configure.target_component(some_sbyte);
            mount_configure.mount_mode(some_MAV_MOUNT_MODE);
            mount_configure.stab_roll(some_sbyte);
            mount_configure.stab_pitch(some_sbyte);
            mount_configure.stab_yaw(some_sbyte);
        }
        public static void onMOUNT_CONTROL(MOUNT_CONTROL mount_control)
        {
            var some_target_system = mount_control.target_system() ;
            var some_target_component = mount_control.target_component() ;
            var some_input_a = mount_control.input_a() ;
            var some_input_b = mount_control.input_b() ;
            var some_input_c = mount_control.input_c() ;
            var some_save_position = mount_control.save_position() ;
        }
        public static void fill(MOUNT_CONTROL mount_control)
        {
            mount_control.target_system(some_sbyte);
            mount_control.target_component(some_sbyte);
            mount_control.input_a(some_int);
            mount_control.input_b(some_int);
            mount_control.input_c(some_int);
            mount_control.save_position(some_sbyte);
        }
        public static void onMOUNT_STATUS(MOUNT_STATUS mount_status)
        {
            var some_target_system = mount_status.target_system() ;
            var some_target_component = mount_status.target_component() ;
            var some_pointing_a = mount_status.pointing_a() ;
            var some_pointing_b = mount_status.pointing_b() ;
            var some_pointing_c = mount_status.pointing_c() ;
        }
        public static void fill(MOUNT_STATUS mount_status)
        {
            mount_status.target_system(some_sbyte);
            mount_status.target_component(some_sbyte);
            mount_status.pointing_a(some_int);
            mount_status.pointing_b(some_int);
            mount_status.pointing_c(some_int);
        }
        public static void onFENCE_POINT(FENCE_POINT fence_point)
        {
            var some_target_system = fence_point.target_system() ;
            var some_target_component = fence_point.target_component() ;
            var some_idx = fence_point.idx() ;
            var some_count = fence_point.count() ;
            var some_lat = fence_point.lat() ;
            var some_lng = fence_point.lng() ;
        }
        public static void fill(FENCE_POINT fence_point)
        {
            fence_point.target_system(some_sbyte);
            fence_point.target_component(some_sbyte);
            fence_point.idx(some_sbyte);
            fence_point.count(some_sbyte);
            fence_point.lat(some_float);
            fence_point.lng(some_float);
        }
        public static void onFENCE_FETCH_POINT(FENCE_FETCH_POINT fence_fetch_point)
        {
            var some_target_system = fence_fetch_point.target_system() ;
            var some_target_component = fence_fetch_point.target_component() ;
            var some_idx = fence_fetch_point.idx() ;
        }
        public static void fill(FENCE_FETCH_POINT fence_fetch_point)
        {
            fence_fetch_point.target_system(some_sbyte);
            fence_fetch_point.target_component(some_sbyte);
            fence_fetch_point.idx(some_sbyte);
        }
        public static void onFENCE_STATUS(FENCE_STATUS fence_status)
        {
            var some_breach_status = fence_status.breach_status() ;
            var some_breach_count = fence_status.breach_count() ;
            {
                var item = fence_status.breach_type();
                if(item.HasValue)
                    some_FENCE_BREACH = item.Value ;
            }
            var some_breach_time = fence_status.breach_time() ;
        }
        public static void fill(FENCE_STATUS fence_status)
        {
            fence_status.breach_status(some_sbyte);
            fence_status.breach_count(some_short);
            fence_status.breach_type(some_FENCE_BREACH);
            fence_status.breach_time(some_int);
        }
        public static void onAHRS(AHRS ahrs)
        {
            var some_omegaIx = ahrs.omegaIx() ;
            var some_omegaIy = ahrs.omegaIy() ;
            var some_omegaIz = ahrs.omegaIz() ;
            var some_accel_weight = ahrs.accel_weight() ;
            var some_renorm_val = ahrs.renorm_val() ;
            var some_error_rp = ahrs.error_rp() ;
            var some_error_yaw = ahrs.error_yaw() ;
        }
        public static void fill(AHRS ahrs)
        {
            ahrs.omegaIx(some_float);
            ahrs.omegaIy(some_float);
            ahrs.omegaIz(some_float);
            ahrs.accel_weight(some_float);
            ahrs.renorm_val(some_float);
            ahrs.error_rp(some_float);
            ahrs.error_yaw(some_float);
        }
        public static void onSIMSTATE(SIMSTATE simstate)
        {
            var some_roll = simstate.roll() ;
            var some_pitch = simstate.pitch() ;
            var some_yaw = simstate.yaw() ;
            var some_xacc = simstate.xacc() ;
            var some_yacc = simstate.yacc() ;
            var some_zacc = simstate.zacc() ;
            var some_xgyro = simstate.xgyro() ;
            var some_ygyro = simstate.ygyro() ;
            var some_zgyro = simstate.zgyro() ;
            var some_lat = simstate.lat() ;
            var some_lng = simstate.lng() ;
        }
        public static void fill(SIMSTATE simstate)
        {
            simstate.roll(some_float);
            simstate.pitch(some_float);
            simstate.yaw(some_float);
            simstate.xacc(some_float);
            simstate.yacc(some_float);
            simstate.zacc(some_float);
            simstate.xgyro(some_float);
            simstate.ygyro(some_float);
            simstate.zgyro(some_float);
            simstate.lat(some_int);
            simstate.lng(some_int);
        }
        public static void onHWSTATUS(HWSTATUS hwstatus)
        {
            var some_Vcc = hwstatus.Vcc() ;
            var some_I2Cerr = hwstatus.I2Cerr() ;
        }
        public static void fill(HWSTATUS hwstatus)
        {
            hwstatus.Vcc(some_short);
            hwstatus.I2Cerr(some_sbyte);
        }
        public static void onRADIO(RADIO radio)
        {
            var some_rssi = radio.rssi() ;
            var some_remrssi = radio.remrssi() ;
            var some_txbuf = radio.txbuf() ;
            var some_noise = radio.noise() ;
            var some_remnoise = radio.remnoise() ;
            var some_rxerrors = radio.rxerrors() ;
            var some_fixeD = radio.fixeD() ;
        }
        public static void fill(RADIO radio)
        {
            radio.rssi(some_sbyte);
            radio.remrssi(some_sbyte);
            radio.txbuf(some_sbyte);
            radio.noise(some_sbyte);
            radio.remnoise(some_sbyte);
            radio.rxerrors(some_short);
            radio.fixeD(some_short);
        }
        public static void onLIMITS_STATUS(LIMITS_STATUS limits_status)
        {
            {
                var item = limits_status.limits_state();
                if(item.HasValue)
                    some_LIMITS_STATE = item.Value ;
            }
            var some_last_trigger = limits_status.last_trigger() ;
            var some_last_action = limits_status.last_action() ;
            var some_last_recovery = limits_status.last_recovery() ;
            var some_last_clear = limits_status.last_clear() ;
            var some_breach_count = limits_status.breach_count() ;
            {
                var item = limits_status.mods_enabled();
                if(item.HasValue)
                    some_LIMIT_MODULE = item.Value ;
            }
            {
                var item = limits_status.mods_required();
                if(item.HasValue)
                    some_LIMIT_MODULE = item.Value ;
            }
            {
                var item = limits_status.mods_triggered();
                if(item.HasValue)
                    some_LIMIT_MODULE = item.Value ;
            }
        }
        public static void fill(LIMITS_STATUS limits_status)
        {
            limits_status.limits_state(some_LIMITS_STATE);
            limits_status.last_trigger(some_int);
            limits_status.last_action(some_int);
            limits_status.last_recovery(some_int);
            limits_status.last_clear(some_int);
            limits_status.breach_count(some_short);
            limits_status.mods_enabled(some_LIMIT_MODULE);
            limits_status.mods_required(some_LIMIT_MODULE);
            limits_status.mods_triggered(some_LIMIT_MODULE);
        }
        public static void onWIND(WIND wind)
        {
            var some_direction = wind.direction() ;
            var some_speed = wind.speed() ;
            var some_speed_z = wind.speed_z() ;
        }
        public static void fill(WIND wind)
        {
            wind.direction(some_float);
            wind.speed(some_float);
            wind.speed_z(some_float);
        }
        public static void onDATA16(DATA16 data16)
        {
            var some_typE = data16.typE() ;
            var some_len = data16.len() ;
            {
                var item = data16.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(DATA16 data16)
        {
            data16.typE(some_sbyte);
            data16.len(some_sbyte);
            {
                var item = data16.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onDATA32(DATA32 data32)
        {
            var some_typE = data32.typE() ;
            var some_len = data32.len() ;
            {
                var item = data32.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(DATA32 data32)
        {
            data32.typE(some_sbyte);
            data32.len(some_sbyte);
            {
                var item = data32.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onDATA64(DATA64 data64)
        {
            var some_typE = data64.typE() ;
            var some_len = data64.len() ;
            {
                var item = data64.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(DATA64 data64)
        {
            data64.typE(some_sbyte);
            data64.len(some_sbyte);
            {
                var item = data64.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onDATA96(DATA96 data96)
        {
            var some_typE = data96.typE() ;
            var some_len = data96.len() ;
            {
                var item = data96.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(DATA96 data96)
        {
            data96.typE(some_sbyte);
            data96.len(some_sbyte);
            {
                var item = data96.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onRANGEFINDER(RANGEFINDER rangefinder)
        {
            var some_distance = rangefinder.distance() ;
            var some_voltage = rangefinder.voltage() ;
        }
        public static void fill(RANGEFINDER rangefinder)
        {
            rangefinder.distance(some_float);
            rangefinder.voltage(some_float);
        }
        public static void onAIRSPEED_AUTOCAL(AIRSPEED_AUTOCAL airspeed_autocal)
        {
            var some_vx = airspeed_autocal.vx() ;
            var some_vy = airspeed_autocal.vy() ;
            var some_vz = airspeed_autocal.vz() ;
            var some_diff_pressure = airspeed_autocal.diff_pressure() ;
            var some_EAS2TAS = airspeed_autocal.EAS2TAS() ;
            var some_ratio = airspeed_autocal.ratio() ;
            var some_state_x = airspeed_autocal.state_x() ;
            var some_state_y = airspeed_autocal.state_y() ;
            var some_state_z = airspeed_autocal.state_z() ;
            var some_Pax = airspeed_autocal.Pax() ;
            var some_Pby = airspeed_autocal.Pby() ;
            var some_Pcz = airspeed_autocal.Pcz() ;
        }
        public static void fill(AIRSPEED_AUTOCAL airspeed_autocal)
        {
            airspeed_autocal.vx(some_float);
            airspeed_autocal.vy(some_float);
            airspeed_autocal.vz(some_float);
            airspeed_autocal.diff_pressure(some_float);
            airspeed_autocal.EAS2TAS(some_float);
            airspeed_autocal.ratio(some_float);
            airspeed_autocal.state_x(some_float);
            airspeed_autocal.state_y(some_float);
            airspeed_autocal.state_z(some_float);
            airspeed_autocal.Pax(some_float);
            airspeed_autocal.Pby(some_float);
            airspeed_autocal.Pcz(some_float);
        }
        public static void onRALLY_POINT(RALLY_POINT rally_point)
        {
            var some_target_system = rally_point.target_system() ;
            var some_target_component = rally_point.target_component() ;
            var some_idx = rally_point.idx() ;
            var some_count = rally_point.count() ;
            var some_lat = rally_point.lat() ;
            var some_lng = rally_point.lng() ;
            var some_alt = rally_point.alt() ;
            var some_break_alt = rally_point.break_alt() ;
            var some_land_dir = rally_point.land_dir() ;
            {
                var item = rally_point.flags();
                if(item.HasValue)
                    some_RALLY_FLAGS = item.Value ;
            }
        }
        public static void fill(RALLY_POINT rally_point)
        {
            rally_point.target_system(some_sbyte);
            rally_point.target_component(some_sbyte);
            rally_point.idx(some_sbyte);
            rally_point.count(some_sbyte);
            rally_point.lat(some_int);
            rally_point.lng(some_int);
            rally_point.alt(some_short);
            rally_point.break_alt(some_short);
            rally_point.land_dir(some_short);
            rally_point.flags(some_RALLY_FLAGS);
        }
        public static void onRALLY_FETCH_POINT(RALLY_FETCH_POINT rally_fetch_point)
        {
            var some_target_system = rally_fetch_point.target_system() ;
            var some_target_component = rally_fetch_point.target_component() ;
            var some_idx = rally_fetch_point.idx() ;
        }
        public static void fill(RALLY_FETCH_POINT rally_fetch_point)
        {
            rally_fetch_point.target_system(some_sbyte);
            rally_fetch_point.target_component(some_sbyte);
            rally_fetch_point.idx(some_sbyte);
        }
        public static void onCOMPASSMOT_STATUS(COMPASSMOT_STATUS compassmot_status)
        {
            var some_throttle = compassmot_status.throttle() ;
            var some_current = compassmot_status.current() ;
            var some_interference = compassmot_status.interference() ;
            var some_CompensationX = compassmot_status.CompensationX() ;
            var some_CompensationY = compassmot_status.CompensationY() ;
            var some_CompensationZ = compassmot_status.CompensationZ() ;
        }
        public static void fill(COMPASSMOT_STATUS compassmot_status)
        {
            compassmot_status.throttle(some_short);
            compassmot_status.current(some_float);
            compassmot_status.interference(some_short);
            compassmot_status.CompensationX(some_float);
            compassmot_status.CompensationY(some_float);
            compassmot_status.CompensationZ(some_float);
        }
        public static void onAHRS2(AHRS2 ahrs2)
        {
            var some_roll = ahrs2.roll() ;
            var some_pitch = ahrs2.pitch() ;
            var some_yaw = ahrs2.yaw() ;
            var some_altitude = ahrs2.altitude() ;
            var some_lat = ahrs2.lat() ;
            var some_lng = ahrs2.lng() ;
        }
        public static void fill(AHRS2 ahrs2)
        {
            ahrs2.roll(some_float);
            ahrs2.pitch(some_float);
            ahrs2.yaw(some_float);
            ahrs2.altitude(some_float);
            ahrs2.lat(some_int);
            ahrs2.lng(some_int);
        }
        public static void onCAMERA_STATUS(CAMERA_STATUS camera_status)
        {
            var some_time_usec = camera_status.time_usec() ;
            var some_target_system = camera_status.target_system() ;
            var some_cam_idx = camera_status.cam_idx() ;
            var some_img_idx = camera_status.img_idx() ;
            {
                var item = camera_status.event_id();
                if(item.HasValue)
                    some_CAMERA_STATUS_TYPES = item.Value ;
            }
            var some_p1 = camera_status.p1() ;
            var some_p2 = camera_status.p2() ;
            var some_p3 = camera_status.p3() ;
            var some_p4 = camera_status.p4() ;
        }
        public static void fill(CAMERA_STATUS camera_status)
        {
            camera_status.time_usec(some_long);
            camera_status.target_system(some_sbyte);
            camera_status.cam_idx(some_sbyte);
            camera_status.img_idx(some_short);
            camera_status.event_id(some_CAMERA_STATUS_TYPES);
            camera_status.p1(some_float);
            camera_status.p2(some_float);
            camera_status.p3(some_float);
            camera_status.p4(some_float);
        }
        public static void onCAMERA_FEEDBACK(CAMERA_FEEDBACK camera_feedback)
        {
            var some_time_usec = camera_feedback.time_usec() ;
            var some_target_system = camera_feedback.target_system() ;
            var some_cam_idx = camera_feedback.cam_idx() ;
            var some_img_idx = camera_feedback.img_idx() ;
            var some_lat = camera_feedback.lat() ;
            var some_lng = camera_feedback.lng() ;
            var some_alt_msl = camera_feedback.alt_msl() ;
            var some_alt_rel = camera_feedback.alt_rel() ;
            var some_roll = camera_feedback.roll() ;
            var some_pitch = camera_feedback.pitch() ;
            var some_yaw = camera_feedback.yaw() ;
            var some_foc_len = camera_feedback.foc_len() ;
            {
                var item = camera_feedback.flags();
                if(item.HasValue)
                    some_CAMERA_FEEDBACK_FLAGS = item.Value ;
            }
        }
        public static void fill(CAMERA_FEEDBACK camera_feedback)
        {
            camera_feedback.time_usec(some_long);
            camera_feedback.target_system(some_sbyte);
            camera_feedback.cam_idx(some_sbyte);
            camera_feedback.img_idx(some_short);
            camera_feedback.lat(some_int);
            camera_feedback.lng(some_int);
            camera_feedback.alt_msl(some_float);
            camera_feedback.alt_rel(some_float);
            camera_feedback.roll(some_float);
            camera_feedback.pitch(some_float);
            camera_feedback.yaw(some_float);
            camera_feedback.foc_len(some_float);
            camera_feedback.flags(some_CAMERA_FEEDBACK_FLAGS);
        }
        public static void onBATTERY2(BATTERY2 battery2)
        {
            var some_voltage = battery2.voltage() ;
            var some_current_battery = battery2.current_battery() ;
        }
        public static void fill(BATTERY2 battery2)
        {
            battery2.voltage(some_short);
            battery2.current_battery(some_short);
        }
        public static void onAHRS3(AHRS3 ahrs3)
        {
            var some_roll = ahrs3.roll() ;
            var some_pitch = ahrs3.pitch() ;
            var some_yaw = ahrs3.yaw() ;
            var some_altitude = ahrs3.altitude() ;
            var some_lat = ahrs3.lat() ;
            var some_lng = ahrs3.lng() ;
            var some_v1 = ahrs3.v1() ;
            var some_v2 = ahrs3.v2() ;
            var some_v3 = ahrs3.v3() ;
            var some_v4 = ahrs3.v4() ;
        }
        public static void fill(AHRS3 ahrs3)
        {
            ahrs3.roll(some_float);
            ahrs3.pitch(some_float);
            ahrs3.yaw(some_float);
            ahrs3.altitude(some_float);
            ahrs3.lat(some_int);
            ahrs3.lng(some_int);
            ahrs3.v1(some_float);
            ahrs3.v2(some_float);
            ahrs3.v3(some_float);
            ahrs3.v4(some_float);
        }
        public static void onAUTOPILOT_VERSION_REQUEST(AUTOPILOT_VERSION_REQUEST autopilot_version_request)
        {
            var some_target_system = autopilot_version_request.target_system() ;
            var some_target_component = autopilot_version_request.target_component() ;
        }
        public static void fill(AUTOPILOT_VERSION_REQUEST autopilot_version_request)
        {
            autopilot_version_request.target_system(some_sbyte);
            autopilot_version_request.target_component(some_sbyte);
        }
        public static void onREMOTE_LOG_DATA_BLOCK(REMOTE_LOG_DATA_BLOCK remote_log_data_block)
        {
            var some_target_system = remote_log_data_block.target_system() ;
            var some_target_component = remote_log_data_block.target_component() ;
            {
                var item = remote_log_data_block.seqno();
                if(item.HasValue)
                    some_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS = item.Value ;
            }
            {
                var item = remote_log_data_block.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(REMOTE_LOG_DATA_BLOCK remote_log_data_block)
        {
            remote_log_data_block.target_system(some_sbyte);
            remote_log_data_block.target_component(some_sbyte);
            remote_log_data_block.seqno(some_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS);
            {
                var item = remote_log_data_block.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onREMOTE_LOG_BLOCK_STATUS(REMOTE_LOG_BLOCK_STATUS remote_log_block_status)
        {
            var some_target_system = remote_log_block_status.target_system() ;
            var some_target_component = remote_log_block_status.target_component() ;
            var some_seqno = remote_log_block_status.seqno() ;
            {
                var item = remote_log_block_status.status();
                if(item.HasValue)
                    some_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES = item.Value ;
            }
        }
        public static void fill(REMOTE_LOG_BLOCK_STATUS remote_log_block_status)
        {
            remote_log_block_status.target_system(some_sbyte);
            remote_log_block_status.target_component(some_sbyte);
            remote_log_block_status.seqno(some_int);
            remote_log_block_status.status(some_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES);
        }
        public static void onLED_CONTROL(LED_CONTROL led_control)
        {
            var some_target_system = led_control.target_system() ;
            var some_target_component = led_control.target_component() ;
            var some_instance = led_control.instance() ;
            var some_pattern = led_control.pattern() ;
            var some_custom_len = led_control.custom_len() ;
            {
                var item = led_control.custom_bytes();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(LED_CONTROL led_control)
        {
            led_control.target_system(some_sbyte);
            led_control.target_component(some_sbyte);
            led_control.instance(some_sbyte);
            led_control.pattern(some_sbyte);
            led_control.custom_len(some_sbyte);
            {
                var item = led_control.custom_bytes();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onMAG_CAL_PROGRESS(MAG_CAL_PROGRESS mag_cal_progress)
        {
            var some_compass_id = mag_cal_progress.compass_id() ;
            var some_cal_mask = mag_cal_progress.cal_mask() ;
            {
                var item = mag_cal_progress.cal_status();
                if(item.HasValue)
                    some_MAG_CAL_STATUS = item.Value ;
            }
            var some_attempt = mag_cal_progress.attempt() ;
            var some_completion_pct = mag_cal_progress.completion_pct() ;
            {
                var item = mag_cal_progress.completion_mask();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
            var some_direction_x = mag_cal_progress.direction_x() ;
            var some_direction_y = mag_cal_progress.direction_y() ;
            var some_direction_z = mag_cal_progress.direction_z() ;
        }
        public static void fill(MAG_CAL_PROGRESS mag_cal_progress)
        {
            mag_cal_progress.compass_id(some_sbyte);
            mag_cal_progress.cal_mask(some_sbyte);
            mag_cal_progress.cal_status(some_MAG_CAL_STATUS);
            mag_cal_progress.attempt(some_sbyte);
            mag_cal_progress.completion_pct(some_sbyte);
            {
                var item = mag_cal_progress.completion_mask();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
            mag_cal_progress.direction_x(some_float);
            mag_cal_progress.direction_y(some_float);
            mag_cal_progress.direction_z(some_float);
        }
        public static void onMAG_CAL_REPORT(MAG_CAL_REPORT mag_cal_report)
        {
            var some_compass_id = mag_cal_report.compass_id() ;
            var some_cal_mask = mag_cal_report.cal_mask() ;
            {
                var item = mag_cal_report.cal_status();
                if(item.HasValue)
                    some_MAG_CAL_STATUS = item.Value ;
            }
            var some_autosaved = mag_cal_report.autosaved() ;
            var some_fitness = mag_cal_report.fitness() ;
            var some_ofs_x = mag_cal_report.ofs_x() ;
            var some_ofs_y = mag_cal_report.ofs_y() ;
            var some_ofs_z = mag_cal_report.ofs_z() ;
            var some_diag_x = mag_cal_report.diag_x() ;
            var some_diag_y = mag_cal_report.diag_y() ;
            var some_diag_z = mag_cal_report.diag_z() ;
            var some_offdiag_x = mag_cal_report.offdiag_x() ;
            var some_offdiag_y = mag_cal_report.offdiag_y() ;
            var some_offdiag_z = mag_cal_report.offdiag_z() ;
        }
        public static void fill(MAG_CAL_REPORT mag_cal_report)
        {
            mag_cal_report.compass_id(some_sbyte);
            mag_cal_report.cal_mask(some_sbyte);
            mag_cal_report.cal_status(some_MAG_CAL_STATUS);
            mag_cal_report.autosaved(some_sbyte);
            mag_cal_report.fitness(some_float);
            mag_cal_report.ofs_x(some_float);
            mag_cal_report.ofs_y(some_float);
            mag_cal_report.ofs_z(some_float);
            mag_cal_report.diag_x(some_float);
            mag_cal_report.diag_y(some_float);
            mag_cal_report.diag_z(some_float);
            mag_cal_report.offdiag_x(some_float);
            mag_cal_report.offdiag_y(some_float);
            mag_cal_report.offdiag_z(some_float);
        }
        public static void onEKF_STATUS_REPORT(EKF_STATUS_REPORT ekf_status_report)
        {
            {
                var item = ekf_status_report.flags();
                if(item.HasValue)
                    some_EKF_STATUS_FLAGS = item.Value ;
            }
            var some_velocity_variance = ekf_status_report.velocity_variance() ;
            var some_pos_horiz_variance = ekf_status_report.pos_horiz_variance() ;
            var some_pos_vert_variance = ekf_status_report.pos_vert_variance() ;
            var some_compass_variance = ekf_status_report.compass_variance() ;
            var some_terrain_alt_variance = ekf_status_report.terrain_alt_variance() ;
        }
        public static void fill(EKF_STATUS_REPORT ekf_status_report)
        {
            ekf_status_report.flags(some_EKF_STATUS_FLAGS);
            ekf_status_report.velocity_variance(some_float);
            ekf_status_report.pos_horiz_variance(some_float);
            ekf_status_report.pos_vert_variance(some_float);
            ekf_status_report.compass_variance(some_float);
            ekf_status_report.terrain_alt_variance(some_float);
        }
        public static void onPID_TUNING(PID_TUNING pid_tuning)
        {
            {
                var item = pid_tuning.axis();
                if(item.HasValue)
                    some_PID_TUNING_AXIS = item.Value ;
            }
            var some_desired = pid_tuning.desired() ;
            var some_achieved = pid_tuning.achieved() ;
            var some_FF = pid_tuning.FF() ;
            var some_P = pid_tuning.P() ;
            var some_I = pid_tuning.I() ;
            var some_D = pid_tuning.D() ;
        }
        public static void fill(PID_TUNING pid_tuning)
        {
            pid_tuning.axis(some_PID_TUNING_AXIS);
            pid_tuning.desired(some_float);
            pid_tuning.achieved(some_float);
            pid_tuning.FF(some_float);
            pid_tuning.P(some_float);
            pid_tuning.I(some_float);
            pid_tuning.D(some_float);
        }
        public static void onGIMBAL_REPORT(GIMBAL_REPORT gimbal_report)
        {
            var some_target_system = gimbal_report.target_system() ;
            var some_target_component = gimbal_report.target_component() ;
            var some_delta_time = gimbal_report.delta_time() ;
            var some_delta_angle_x = gimbal_report.delta_angle_x() ;
            var some_delta_angle_y = gimbal_report.delta_angle_y() ;
            var some_delta_angle_z = gimbal_report.delta_angle_z() ;
            var some_delta_velocity_x = gimbal_report.delta_velocity_x() ;
            var some_delta_velocity_y = gimbal_report.delta_velocity_y() ;
            var some_delta_velocity_z = gimbal_report.delta_velocity_z() ;
            var some_joint_roll = gimbal_report.joint_roll() ;
            var some_joint_el = gimbal_report.joint_el() ;
            var some_joint_az = gimbal_report.joint_az() ;
        }
        public static void fill(GIMBAL_REPORT gimbal_report)
        {
            gimbal_report.target_system(some_sbyte);
            gimbal_report.target_component(some_sbyte);
            gimbal_report.delta_time(some_float);
            gimbal_report.delta_angle_x(some_float);
            gimbal_report.delta_angle_y(some_float);
            gimbal_report.delta_angle_z(some_float);
            gimbal_report.delta_velocity_x(some_float);
            gimbal_report.delta_velocity_y(some_float);
            gimbal_report.delta_velocity_z(some_float);
            gimbal_report.joint_roll(some_float);
            gimbal_report.joint_el(some_float);
            gimbal_report.joint_az(some_float);
        }
        public static void onGIMBAL_CONTROL(GIMBAL_CONTROL gimbal_control)
        {
            var some_target_system = gimbal_control.target_system() ;
            var some_target_component = gimbal_control.target_component() ;
            var some_demanded_rate_x = gimbal_control.demanded_rate_x() ;
            var some_demanded_rate_y = gimbal_control.demanded_rate_y() ;
            var some_demanded_rate_z = gimbal_control.demanded_rate_z() ;
        }
        public static void fill(GIMBAL_CONTROL gimbal_control)
        {
            gimbal_control.target_system(some_sbyte);
            gimbal_control.target_component(some_sbyte);
            gimbal_control.demanded_rate_x(some_float);
            gimbal_control.demanded_rate_y(some_float);
            gimbal_control.demanded_rate_z(some_float);
        }
        public static void onGIMBAL_TORQUE_CMD_REPORT(GIMBAL_TORQUE_CMD_REPORT gimbal_torque_cmd_report)
        {
            var some_target_system = gimbal_torque_cmd_report.target_system() ;
            var some_target_component = gimbal_torque_cmd_report.target_component() ;
            var some_rl_torque_cmd = gimbal_torque_cmd_report.rl_torque_cmd() ;
            var some_el_torque_cmd = gimbal_torque_cmd_report.el_torque_cmd() ;
            var some_az_torque_cmd = gimbal_torque_cmd_report.az_torque_cmd() ;
        }
        public static void fill(GIMBAL_TORQUE_CMD_REPORT gimbal_torque_cmd_report)
        {
            gimbal_torque_cmd_report.target_system(some_sbyte);
            gimbal_torque_cmd_report.target_component(some_sbyte);
            gimbal_torque_cmd_report.rl_torque_cmd(some_short);
            gimbal_torque_cmd_report.el_torque_cmd(some_short);
            gimbal_torque_cmd_report.az_torque_cmd(some_short);
        }
        public static void onGOPRO_HEARTBEAT(GOPRO_HEARTBEAT gopro_heartbeat)
        {
            {
                var item = gopro_heartbeat.status();
                if(item.HasValue)
                    some_GOPRO_HEARTBEAT_STATUS = item.Value ;
            }
            {
                var item = gopro_heartbeat.capture_mode();
                if(item.HasValue)
                    some_GOPRO_CAPTURE_MODE = item.Value ;
            }
            {
                var item = gopro_heartbeat.flags();
                if(item.HasValue)
                    some_GOPRO_HEARTBEAT_FLAGS = item.Value ;
            }
        }
        public static void fill(GOPRO_HEARTBEAT gopro_heartbeat)
        {
            gopro_heartbeat.status(some_GOPRO_HEARTBEAT_STATUS);
            gopro_heartbeat.capture_mode(some_GOPRO_CAPTURE_MODE);
            gopro_heartbeat.flags(some_GOPRO_HEARTBEAT_FLAGS);
        }
        public static void onGOPRO_GET_REQUEST(GOPRO_GET_REQUEST gopro_get_request)
        {
            var some_target_system = gopro_get_request.target_system() ;
            var some_target_component = gopro_get_request.target_component() ;
            {
                var item = gopro_get_request.cmd_id();
                if(item.HasValue)
                    some_GOPRO_COMMAND = item.Value ;
            }
        }
        public static void fill(GOPRO_GET_REQUEST gopro_get_request)
        {
            gopro_get_request.target_system(some_sbyte);
            gopro_get_request.target_component(some_sbyte);
            gopro_get_request.cmd_id(some_GOPRO_COMMAND);
        }
        public static void onGOPRO_GET_RESPONSE(GOPRO_GET_RESPONSE gopro_get_response)
        {
            {
                var item = gopro_get_response.cmd_id();
                if(item.HasValue)
                    some_GOPRO_COMMAND = item.Value ;
            }
            {
                var item = gopro_get_response.status();
                if(item.HasValue)
                    some_GOPRO_REQUEST_STATUS = item.Value ;
            }
            {
                var item = gopro_get_response.value();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(GOPRO_GET_RESPONSE gopro_get_response)
        {
            gopro_get_response.cmd_id(some_GOPRO_COMMAND);
            gopro_get_response.status(some_GOPRO_REQUEST_STATUS);
            {
                var item = gopro_get_response.value();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onGOPRO_SET_REQUEST(GOPRO_SET_REQUEST gopro_set_request)
        {
            var some_target_system = gopro_set_request.target_system() ;
            var some_target_component = gopro_set_request.target_component() ;
            {
                var item = gopro_set_request.cmd_id();
                if(item.HasValue)
                    some_GOPRO_COMMAND = item.Value ;
            }
            {
                var item = gopro_set_request.value();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(GOPRO_SET_REQUEST gopro_set_request)
        {
            gopro_set_request.target_system(some_sbyte);
            gopro_set_request.target_component(some_sbyte);
            gopro_set_request.cmd_id(some_GOPRO_COMMAND);
            {
                var item = gopro_set_request.value();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onGOPRO_SET_RESPONSE(GOPRO_SET_RESPONSE gopro_set_response)
        {
            {
                var item = gopro_set_response.cmd_id();
                if(item.HasValue)
                    some_GOPRO_COMMAND = item.Value ;
            }
            {
                var item = gopro_set_response.status();
                if(item.HasValue)
                    some_GOPRO_REQUEST_STATUS = item.Value ;
            }
        }
        public static void fill(GOPRO_SET_RESPONSE gopro_set_response)
        {
            gopro_set_response.cmd_id(some_GOPRO_COMMAND);
            gopro_set_response.status(some_GOPRO_REQUEST_STATUS);
        }
        public static void onRPM(RPM rpm)
        {
            var some_rpm1 = rpm.rpm1() ;
            var some_rpm2 = rpm.rpm2() ;
        }
        public static void fill(RPM rpm)
        {
            rpm.rpm1(some_float);
            rpm.rpm2(some_float);
        }
        public static void onDEVICE_OP_READ(DEVICE_OP_READ device_op_read)
        {
            var some_target_system = device_op_read.target_system() ;
            var some_target_component = device_op_read.target_component() ;
            var some_request_id = device_op_read.request_id() ;
            {
                var item = device_op_read.bustype();
                if(item.HasValue)
                    some_DEVICE_OP_BUSTYPE = item.Value ;
            }
            var some_bus = device_op_read.bus() ;
            var some_address = device_op_read.address() ;
            {
                var item = device_op_read.busname();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_regstart = device_op_read.regstart() ;
            var some_count = device_op_read.count() ;
        }
        public static void fill(DEVICE_OP_READ device_op_read)
        {
            device_op_read.target_system(some_sbyte);
            device_op_read.target_component(some_sbyte);
            device_op_read.request_id(some_int);
            device_op_read.bustype(some_DEVICE_OP_BUSTYPE);
            device_op_read.bus(some_sbyte);
            device_op_read.address(some_sbyte);
            device_op_read.busname(some_string, null);
            device_op_read.regstart(some_sbyte);
            device_op_read.count(some_sbyte);
        }
        public static void onDEVICE_OP_READ_REPLY(DEVICE_OP_READ_REPLY device_op_read_reply)
        {
            var some_request_id = device_op_read_reply.request_id() ;
            var some_result = device_op_read_reply.result() ;
            var some_regstart = device_op_read_reply.regstart() ;
            var some_count = device_op_read_reply.count() ;
            {
                var item = device_op_read_reply.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(DEVICE_OP_READ_REPLY device_op_read_reply)
        {
            device_op_read_reply.request_id(some_int);
            device_op_read_reply.result(some_sbyte);
            device_op_read_reply.regstart(some_sbyte);
            device_op_read_reply.count(some_sbyte);
            {
                var item = device_op_read_reply.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onDEVICE_OP_WRITE(DEVICE_OP_WRITE device_op_write)
        {
            var some_target_system = device_op_write.target_system() ;
            var some_target_component = device_op_write.target_component() ;
            var some_request_id = device_op_write.request_id() ;
            {
                var item = device_op_write.bustype();
                if(item.HasValue)
                    some_DEVICE_OP_BUSTYPE = item.Value ;
            }
            var some_bus = device_op_write.bus() ;
            var some_address = device_op_write.address() ;
            {
                var item = device_op_write.busname();
                if(item.HasValue)
                    some_string = item.Value.get();
            }
            var some_regstart = device_op_write.regstart() ;
            var some_count = device_op_write.count() ;
            {
                var item = device_op_write.daTa();
                for(int i = 0; i < item.len(); i++)
                    some_sbyte = item.get(i);
            }
        }
        public static void fill(DEVICE_OP_WRITE device_op_write)
        {
            device_op_write.target_system(some_sbyte);
            device_op_write.target_component(some_sbyte);
            device_op_write.request_id(some_int);
            device_op_write.bustype(some_DEVICE_OP_BUSTYPE);
            device_op_write.bus(some_sbyte);
            device_op_write.address(some_sbyte);
            device_op_write.busname(some_string, null);
            device_op_write.regstart(some_sbyte);
            device_op_write.count(some_sbyte);
            {
                var item = device_op_write.daTa();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_sbyte, i);
            }
        }
        public static void onDEVICE_OP_WRITE_REPLY(DEVICE_OP_WRITE_REPLY device_op_write_reply)
        {
            var some_request_id = device_op_write_reply.request_id() ;
            var some_result = device_op_write_reply.result() ;
        }
        public static void fill(DEVICE_OP_WRITE_REPLY device_op_write_reply)
        {
            device_op_write_reply.request_id(some_int);
            device_op_write_reply.result(some_sbyte);
        }
        public static void onADAP_TUNING(ADAP_TUNING adap_tuning)
        {
            {
                var item = adap_tuning.axis();
                if(item.HasValue)
                    some_PID_TUNING_AXIS = item.Value ;
            }
            var some_desired = adap_tuning.desired() ;
            var some_achieved = adap_tuning.achieved() ;
            var some_error = adap_tuning.error() ;
            var some_theta = adap_tuning.theta() ;
            var some_omega = adap_tuning.omega() ;
            var some_sigma = adap_tuning.sigma() ;
            var some_theta_dot = adap_tuning.theta_dot() ;
            var some_omega_dot = adap_tuning.omega_dot() ;
            var some_sigma_dot = adap_tuning.sigma_dot() ;
            var some_f = adap_tuning.f() ;
            var some_f_dot = adap_tuning.f_dot() ;
            var some_u = adap_tuning.u() ;
        }
        public static void fill(ADAP_TUNING adap_tuning)
        {
            adap_tuning.axis(some_PID_TUNING_AXIS);
            adap_tuning.desired(some_float);
            adap_tuning.achieved(some_float);
            adap_tuning.error(some_float);
            adap_tuning.theta(some_float);
            adap_tuning.omega(some_float);
            adap_tuning.sigma(some_float);
            adap_tuning.theta_dot(some_float);
            adap_tuning.omega_dot(some_float);
            adap_tuning.sigma_dot(some_float);
            adap_tuning.f(some_float);
            adap_tuning.f_dot(some_float);
            adap_tuning.u(some_float);
        }
        public static void onVISION_POSITION_DELTA(VISION_POSITION_DELTA vision_position_delta)
        {
            var some_time_usec = vision_position_delta.time_usec() ;
            var some_time_delta_usec = vision_position_delta.time_delta_usec() ;
            {
                var item = vision_position_delta.angle_delta();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            {
                var item = vision_position_delta.position_delta();
                for(int i = 0; i < item.len(); i++)
                    some_float = item.get(i);
            }
            var some_confidence = vision_position_delta.confidence() ;
        }
        public static void fill(VISION_POSITION_DELTA vision_position_delta)
        {
            vision_position_delta.time_usec(some_long);
            vision_position_delta.time_delta_usec(some_long);
            {
                var item = vision_position_delta.angle_delta();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            {
                var item = vision_position_delta.position_delta();
                for(int i = 0; i < item.len(); i++)
                    item.set(some_float, i);
            }
            vision_position_delta.confidence(some_float);
        }


        public class CommunicationChannel_demo: CommunicationChannel
        {
            protected override void onRESOURCE_REQUEST(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_RESOURCE_REQUEST(this,  new com.company.demo.GroundControl.RESOURCE_REQUEST(data));
            }
            protected override void onATTITUDE_TARGET(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_ATTITUDE_TARGET(this,  new com.company.demo.GroundControl.ATTITUDE_TARGET(data));
            }
            protected override void onMISSION_COUNT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_COUNT(this, new com.company.demo.GroundControl.MISSION_COUNT(cur));
            }
            protected override void onADSB_VEHICLE(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_ADSB_VEHICLE(this, new com.company.demo.GroundControl.ADSB_VEHICLE(cur));
            }
            protected override void onMESSAGE_INTERVAL(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_MESSAGE_INTERVAL(this,  new com.company.demo.GroundControl.MESSAGE_INTERVAL(data));
            }
            protected override void onESTIMATOR_STATUS(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_ESTIMATOR_STATUS(this, new com.company.demo.GroundControl.ESTIMATOR_STATUS(cur));
            }
            protected override void onTIMESYNC(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_TIMESYNC(this,  new com.company.demo.GroundControl.TIMESYNC(data));
            }
            protected override void onGLOBAL_POSITION_INT_COV(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_GLOBAL_POSITION_INT_COV(this, new com.company.demo.GroundControl.GLOBAL_POSITION_INT_COV(cur));
            }
            protected override void onBUTTON_CHANGE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_BUTTON_CHANGE(this,  new com.company.demo.GroundControl.BUTTON_CHANGE(data));
            }
            protected override void onSAFETY_SET_ALLOWED_AREA(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SAFETY_SET_ALLOWED_AREA(this, new com.company.demo.GroundControl.SAFETY_SET_ALLOWED_AREA(cur));
            }
            protected override void onSTORAGE_INFORMATION(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_STORAGE_INFORMATION(this,  new com.company.demo.GroundControl.STORAGE_INFORMATION(data));
            }
            protected override void onCOLLISION(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_COLLISION(this, new com.company.demo.GroundControl.COLLISION(cur));
            }
            protected override void onALTITUDE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_ALTITUDE(this,  new com.company.demo.GroundControl.ALTITUDE(data));
            }
            protected override void onHIL_STATE_QUATERNION(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_HIL_STATE_QUATERNION(this,  new com.company.demo.GroundControl.HIL_STATE_QUATERNION(data));
            }
            protected override void onCAMERA_INFORMATION(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_CAMERA_INFORMATION(this, new com.company.demo.GroundControl.CAMERA_INFORMATION(cur));
            }
            protected override void onGPS_STATUS(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_GPS_STATUS(this,  new com.company.demo.GroundControl.GPS_STATUS(data));
            }
            protected override void onPARAM_SET(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_PARAM_SET(this, new com.company.demo.GroundControl.PARAM_SET(cur));
            }
            protected override void onTERRAIN_DATA(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_TERRAIN_DATA(this,  new com.company.demo.GroundControl.TERRAIN_DATA(data));
            }
            protected override void onRC_CHANNELS_OVERRIDE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_RC_CHANNELS_OVERRIDE(this,  new com.company.demo.GroundControl.RC_CHANNELS_OVERRIDE(data));
            }
            protected override void onSCALED_IMU(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SCALED_IMU(this,  new com.company.demo.GroundControl.SCALED_IMU(data));
            }
            protected override void onDEBUG(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_DEBUG(this,  new com.company.demo.GroundControl.DEBUG(data));
            }
            protected override void onCAMERA_IMAGE_CAPTURED(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_CAMERA_IMAGE_CAPTURED(this, new com.company.demo.GroundControl.CAMERA_IMAGE_CAPTURED(cur));
            }
            protected override void onLOG_ENTRY(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_LOG_ENTRY(this,  new com.company.demo.GroundControl.LOG_ENTRY(data));
            }
            protected override void onACTUATOR_CONTROL_TARGET(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_ACTUATOR_CONTROL_TARGET(this,  new com.company.demo.GroundControl.ACTUATOR_CONTROL_TARGET(data));
            }
            protected override void onHIGH_LATENCY(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_HIGH_LATENCY(this, new com.company.demo.GroundControl.HIGH_LATENCY(cur));
            }
            protected override void onPARAM_REQUEST_READ(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_PARAM_REQUEST_READ(this, new com.company.demo.GroundControl.PARAM_REQUEST_READ(cur));
            }
            protected override void onSET_ATTITUDE_TARGET(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SET_ATTITUDE_TARGET(this,  new com.company.demo.GroundControl.SET_ATTITUDE_TARGET(data));
            }
            protected override void onFOLLOW_TARGET(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_FOLLOW_TARGET(this,  new com.company.demo.GroundControl.FOLLOW_TARGET(data));
            }
            protected override void onHIL_STATE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_HIL_STATE(this,  new com.company.demo.GroundControl.HIL_STATE(data));
            }
            protected override void onHOME_POSITION(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_HOME_POSITION(this, new com.company.demo.GroundControl.HOME_POSITION(cur));
            }
            protected override void onGPS2_RAW(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_GPS2_RAW(this, new com.company.demo.GroundControl.GPS2_RAW(cur));
            }
            protected override void onMEMORY_VECT(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_MEMORY_VECT(this,  new com.company.demo.GroundControl.MEMORY_VECT(data));
            }
            protected override void onREQUEST_DATA_STREAM(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_REQUEST_DATA_STREAM(this,  new com.company.demo.GroundControl.REQUEST_DATA_STREAM(data));
            }
            protected override void onHIL_CONTROLS(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_HIL_CONTROLS(this, new com.company.demo.GroundControl.HIL_CONTROLS(cur));
            }
            protected override void onHIL_SENSOR(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_HIL_SENSOR(this,  new com.company.demo.GroundControl.HIL_SENSOR(data));
            }
            protected override void onSETUP_SIGNING(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SETUP_SIGNING(this,  new com.company.demo.GroundControl.SETUP_SIGNING(data));
            }
            protected override void onGPS_RTK(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_GPS_RTK(this,  new com.company.demo.GroundControl.GPS_RTK(data));
            }
            protected override void onPARAM_REQUEST_LIST(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_PARAM_REQUEST_LIST(this,  new com.company.demo.GroundControl.PARAM_REQUEST_LIST(data));
            }
            protected override void onLANDING_TARGET(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_LANDING_TARGET(this, new com.company.demo.GroundControl.LANDING_TARGET(cur));
            }
            protected override void onSET_ACTUATOR_CONTROL_TARGET(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SET_ACTUATOR_CONTROL_TARGET(this,  new com.company.demo.GroundControl.SET_ACTUATOR_CONTROL_TARGET(data));
            }
            protected override void onCONTROL_SYSTEM_STATE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_CONTROL_SYSTEM_STATE(this,  new com.company.demo.GroundControl.CONTROL_SYSTEM_STATE(data));
            }
            protected override void onSET_POSITION_TARGET_GLOBAL_INT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SET_POSITION_TARGET_GLOBAL_INT(this, new com.company.demo.GroundControl.SET_POSITION_TARGET_GLOBAL_INT(cur));
            }
            protected override void onVIBRATION(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_VIBRATION(this,  new com.company.demo.GroundControl.VIBRATION(data));
            }
            protected override void onPING33(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_PING33(this, new com.company.demo.GroundControl.PING33(cur));
            }
            protected override void onVFR_HUD(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_VFR_HUD(this,  new com.company.demo.GroundControl.VFR_HUD(data));
            }
            protected override void onMISSION_SET_CURRENT(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_MISSION_SET_CURRENT(this,  new com.company.demo.GroundControl.MISSION_SET_CURRENT(data));
            }
            protected override void onHIL_GPS(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_HIL_GPS(this,  new com.company.demo.GroundControl.HIL_GPS(data));
            }
            protected override void onNAV_CONTROLLER_OUTPUT(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_NAV_CONTROLLER_OUTPUT(this,  new com.company.demo.GroundControl.NAV_CONTROLLER_OUTPUT(data));
            }
            protected override void onAUTH_KEY(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_AUTH_KEY(this, new com.company.demo.GroundControl.AUTH_KEY(cur));
            }
            protected override void onLOCAL_POSITION_NED_COV(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_LOCAL_POSITION_NED_COV(this, new com.company.demo.GroundControl.LOCAL_POSITION_NED_COV(cur));
            }
            protected override void onATT_POS_MOCAP(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_ATT_POS_MOCAP(this,  new com.company.demo.GroundControl.ATT_POS_MOCAP(data));
            }
            protected override void onSTATUSTEXT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_STATUSTEXT(this, new com.company.demo.GroundControl.STATUSTEXT(cur));
            }
            protected override void onPING(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_PING(this,  new com.company.demo.GroundControl.PING(data));
            }
            protected override void onCAMERA_CAPTURE_STATUS(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_CAMERA_CAPTURE_STATUS(this,  new com.company.demo.GroundControl.CAMERA_CAPTURE_STATUS(data));
            }
            protected override void onGLOBAL_POSITION_INT(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_GLOBAL_POSITION_INT(this,  new com.company.demo.GroundControl.GLOBAL_POSITION_INT(data));
            }
            protected override void onENCAPSULATED_DATA(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_ENCAPSULATED_DATA(this,  new com.company.demo.GroundControl.ENCAPSULATED_DATA(data));
            }
            protected override void onGPS_INPUT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_GPS_INPUT(this, new com.company.demo.GroundControl.GPS_INPUT(cur));
            }
            protected override void onCOMMAND_LONG(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_COMMAND_LONG(this, new com.company.demo.GroundControl.COMMAND_LONG(cur));
            }
            protected override void onLOG_REQUEST_DATA(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_LOG_REQUEST_DATA(this,  new com.company.demo.GroundControl.LOG_REQUEST_DATA(data));
            }
            protected override void onGPS_RAW_INT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_GPS_RAW_INT(this, new com.company.demo.GroundControl.GPS_RAW_INT(cur));
            }
            protected override void onRC_CHANNELS_SCALED(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_RC_CHANNELS_SCALED(this,  new com.company.demo.GroundControl.RC_CHANNELS_SCALED(data));
            }
            protected override void onCAMERA_SETTINGS(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_CAMERA_SETTINGS(this, new com.company.demo.GroundControl.CAMERA_SETTINGS(cur));
            }
            protected override void onRAW_PRESSURE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_RAW_PRESSURE(this,  new com.company.demo.GroundControl.RAW_PRESSURE(data));
            }
            protected override void onNAMED_VALUE_FLOAT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_NAMED_VALUE_FLOAT(this, new com.company.demo.GroundControl.NAMED_VALUE_FLOAT(cur));
            }
            protected override void onATTITUDE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_ATTITUDE(this,  new com.company.demo.GroundControl.ATTITUDE(data));
            }
            protected override void onTERRAIN_REQUEST(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_TERRAIN_REQUEST(this,  new com.company.demo.GroundControl.TERRAIN_REQUEST(data));
            }
            protected override void onMISSION_WRITE_PARTIAL_LIST(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_WRITE_PARTIAL_LIST(this, new com.company.demo.GroundControl.MISSION_WRITE_PARTIAL_LIST(cur));
            }
            protected override void onLOG_ERASE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_LOG_ERASE(this,  new com.company.demo.GroundControl.LOG_ERASE(data));
            }
            protected override void onMANUAL_SETPOINT(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_MANUAL_SETPOINT(this,  new com.company.demo.GroundControl.MANUAL_SETPOINT(data));
            }
            protected override void onSAFETY_ALLOWED_AREA(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SAFETY_ALLOWED_AREA(this, new com.company.demo.GroundControl.SAFETY_ALLOWED_AREA(cur));
            }
            protected override void onOPTICAL_FLOW_RAD(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_OPTICAL_FLOW_RAD(this,  new com.company.demo.GroundControl.OPTICAL_FLOW_RAD(data));
            }
            protected override void onLOG_DATA(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_LOG_DATA(this,  new com.company.demo.GroundControl.LOG_DATA(data));
            }
            protected override void onMISSION_CLEAR_ALL(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_CLEAR_ALL(this, new com.company.demo.GroundControl.MISSION_CLEAR_ALL(cur));
            }
            protected override void onVICON_POSITION_ESTIMATE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_VICON_POSITION_ESTIMATE(this,  new com.company.demo.GroundControl.VICON_POSITION_ESTIMATE(data));
            }
            protected override void onGPS2_RTK(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_GPS2_RTK(this,  new com.company.demo.GroundControl.GPS2_RTK(data));
            }
            protected override void onLOG_REQUEST_LIST(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_LOG_REQUEST_LIST(this,  new com.company.demo.GroundControl.LOG_REQUEST_LIST(data));
            }
            protected override void onSCALED_PRESSURE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SCALED_PRESSURE(this,  new com.company.demo.GroundControl.SCALED_PRESSURE(data));
            }
            protected override void onMISSION_REQUEST_INT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_REQUEST_INT(this, new com.company.demo.GroundControl.MISSION_REQUEST_INT(cur));
            }
            protected override void onV2_EXTENSION(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_V2_EXTENSION(this,  new com.company.demo.GroundControl.V2_EXTENSION(data));
            }
            protected override void onHEARTBEAT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_HEARTBEAT(this, new com.company.demo.GroundControl.HEARTBEAT(cur));
            }
            protected override void onPARAM_MAP_RC(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_PARAM_MAP_RC(this, new com.company.demo.GroundControl.PARAM_MAP_RC(cur));
            }
            protected override void onPOWER_STATUS(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_POWER_STATUS(this, new com.company.demo.GroundControl.POWER_STATUS(cur));
            }
            protected override void onTERRAIN_CHECK(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_TERRAIN_CHECK(this,  new com.company.demo.GroundControl.TERRAIN_CHECK(data));
            }
            protected override void onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(this,  new com.company.demo.GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(data));
            }
            protected override void onCOMMAND_ACK(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_COMMAND_ACK(this, new com.company.demo.GroundControl.COMMAND_ACK(cur));
            }
            protected override void onDATA_STREAM(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_DATA_STREAM(this,  new com.company.demo.GroundControl.DATA_STREAM(data));
            }
            protected override void onMISSION_REQUEST(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_REQUEST(this, new com.company.demo.GroundControl.MISSION_REQUEST(cur));
            }
            protected override void onTERRAIN_REPORT(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_TERRAIN_REPORT(this,  new com.company.demo.GroundControl.TERRAIN_REPORT(data));
            }
            protected override void onSET_HOME_POSITION(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SET_HOME_POSITION(this, new com.company.demo.GroundControl.SET_HOME_POSITION(cur));
            }
            protected override void onSwitchModeCommand()
            {
                on_SwitchModeCommand(this, null);
            }
            protected override void onHIL_RC_INPUTS_RAW(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_HIL_RC_INPUTS_RAW(this,  new com.company.demo.GroundControl.HIL_RC_INPUTS_RAW(data));
            }
            protected override void onSCALED_IMU3(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SCALED_IMU3(this,  new com.company.demo.GroundControl.SCALED_IMU3(data));
            }
            protected override void onSET_MODE(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SET_MODE(this, new com.company.demo.GroundControl.SET_MODE(cur));
            }
            protected override void onPOSITION_TARGET_GLOBAL_INT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_POSITION_TARGET_GLOBAL_INT(this, new com.company.demo.GroundControl.POSITION_TARGET_GLOBAL_INT(cur));
            }
            protected override void onFLIGHT_INFORMATION(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_FLIGHT_INFORMATION(this,  new com.company.demo.GroundControl.FLIGHT_INFORMATION(data));
            }
            protected override void onSIM_STATE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SIM_STATE(this,  new com.company.demo.GroundControl.SIM_STATE(data));
            }
            protected override void onMISSION_ITEM_REACHED(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_MISSION_ITEM_REACHED(this,  new com.company.demo.GroundControl.MISSION_ITEM_REACHED(data));
            }
            protected override void onRC_CHANNELS_RAW(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_RC_CHANNELS_RAW(this,  new com.company.demo.GroundControl.RC_CHANNELS_RAW(data));
            }
            protected override void onSERVO_OUTPUT_RAW(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SERVO_OUTPUT_RAW(this, new com.company.demo.GroundControl.SERVO_OUTPUT_RAW(cur));
            }
            protected override void onVISION_SPEED_ESTIMATE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_VISION_SPEED_ESTIMATE(this,  new com.company.demo.GroundControl.VISION_SPEED_ESTIMATE(data));
            }
            protected override void onDEBUG_VECT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_DEBUG_VECT(this, new com.company.demo.GroundControl.DEBUG_VECT(cur));
            }
            protected override void onLOG_REQUEST_END(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_LOG_REQUEST_END(this,  new com.company.demo.GroundControl.LOG_REQUEST_END(data));
            }
            protected override void onMISSION_ACK(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_ACK(this, new com.company.demo.GroundControl.MISSION_ACK(cur));
            }
            protected override void onCHANGE_OPERATOR_CONTROL_ACK(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_CHANGE_OPERATOR_CONTROL_ACK(this,  new com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL_ACK(data));
            }
            protected override void onMISSION_CURRENT(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_MISSION_CURRENT(this,  new com.company.demo.GroundControl.MISSION_CURRENT(data));
            }
            protected override void onSYSTEM_TIME(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SYSTEM_TIME(this,  new com.company.demo.GroundControl.SYSTEM_TIME(data));
            }
            protected override void onCAMERA_TRIGGER(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_CAMERA_TRIGGER(this,  new com.company.demo.GroundControl.CAMERA_TRIGGER(data));
            }
            protected override void onVISION_POSITION_ESTIMATE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_VISION_POSITION_ESTIMATE(this,  new com.company.demo.GroundControl.VISION_POSITION_ESTIMATE(data));
            }
            protected override void onMANUAL_CONTROL(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_MANUAL_CONTROL(this,  new com.company.demo.GroundControl.MANUAL_CONTROL(data));
            }
            protected override void onRC_CHANNELS(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_RC_CHANNELS(this,  new com.company.demo.GroundControl.RC_CHANNELS(data));
            }
            protected override void onPARAM_VALUE(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_PARAM_VALUE(this, new com.company.demo.GroundControl.PARAM_VALUE(cur));
            }
            protected override void onBATTERY_STATUS(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_BATTERY_STATUS(this, new com.company.demo.GroundControl.BATTERY_STATUS(cur));
            }
            protected override void onSET_POSITION_TARGET_LOCAL_NED(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SET_POSITION_TARGET_LOCAL_NED(this, new com.company.demo.GroundControl.SET_POSITION_TARGET_LOCAL_NED(cur));
            }
            protected override void onSERIAL_CONTROL(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SERIAL_CONTROL(this, new com.company.demo.GroundControl.SERIAL_CONTROL(cur));
            }
            protected override void onSET_GPS_GLOBAL_ORIGIN(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SET_GPS_GLOBAL_ORIGIN(this, new com.company.demo.GroundControl.SET_GPS_GLOBAL_ORIGIN(cur));
            }
            protected override void onAUTOPILOT_VERSION(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_AUTOPILOT_VERSION(this, new com.company.demo.GroundControl.AUTOPILOT_VERSION(cur));
            }
            protected override void onMISSION_REQUEST_LIST(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_REQUEST_LIST(this, new com.company.demo.GroundControl.MISSION_REQUEST_LIST(cur));
            }
            protected override void onPLAY_TUNE(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_PLAY_TUNE(this, new com.company.demo.GroundControl.PLAY_TUNE(cur));
            }
            protected override void onSCALED_PRESSURE3(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SCALED_PRESSURE3(this,  new com.company.demo.GroundControl.SCALED_PRESSURE3(data));
            }
            protected override void onMISSION_REQUEST_PARTIAL_LIST(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_REQUEST_PARTIAL_LIST(this, new com.company.demo.GroundControl.MISSION_REQUEST_PARTIAL_LIST(cur));
            }
            protected override void onLOCAL_POSITION_NED(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_LOCAL_POSITION_NED(this,  new com.company.demo.GroundControl.LOCAL_POSITION_NED(data));
            }
            protected override void onDATA_TRANSMISSION_HANDSHAKE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_DATA_TRANSMISSION_HANDSHAKE(this,  new com.company.demo.GroundControl.DATA_TRANSMISSION_HANDSHAKE(data));
            }
            protected override void onGPS_GLOBAL_ORIGIN(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_GPS_GLOBAL_ORIGIN(this, new com.company.demo.GroundControl.GPS_GLOBAL_ORIGIN(cur));
            }
            protected override void onSCALED_IMU2(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SCALED_IMU2(this,  new com.company.demo.GroundControl.SCALED_IMU2(data));
            }
            protected override void onATTITUDE_QUATERNION(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_ATTITUDE_QUATERNION(this,  new com.company.demo.GroundControl.ATTITUDE_QUATERNION(data));
            }
            protected override void onHIL_ACTUATOR_CONTROLS(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_HIL_ACTUATOR_CONTROLS(this, new com.company.demo.GroundControl.HIL_ACTUATOR_CONTROLS(cur));
            }
            protected override void onPOSITION_TARGET_LOCAL_NED(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_POSITION_TARGET_LOCAL_NED(this, new com.company.demo.GroundControl.POSITION_TARGET_LOCAL_NED(cur));
            }
            protected override void onDISTANCE_SENSOR(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_DISTANCE_SENSOR(this, new com.company.demo.GroundControl.DISTANCE_SENSOR(cur));
            }
            protected override void onHIL_OPTICAL_FLOW(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_HIL_OPTICAL_FLOW(this,  new com.company.demo.GroundControl.HIL_OPTICAL_FLOW(data));
            }
            protected override void onSCALED_PRESSURE2(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_SCALED_PRESSURE2(this,  new com.company.demo.GroundControl.SCALED_PRESSURE2(data));
            }
            protected override void onWIND_COV(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_WIND_COV(this,  new com.company.demo.GroundControl.WIND_COV(data));
            }
            protected override void onCHANGE_OPERATOR_CONTROL(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_CHANGE_OPERATOR_CONTROL(this, new com.company.demo.GroundControl.CHANGE_OPERATOR_CONTROL(cur));
            }
            protected override void onSYS_STATUS(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_SYS_STATUS(this, new com.company.demo.GroundControl.SYS_STATUS(cur));
            }
            protected override void onMISSION_ITEM(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_ITEM(this, new com.company.demo.GroundControl.MISSION_ITEM(cur));
            }
            protected override void onRAW_IMU(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_RAW_IMU(this,  new com.company.demo.GroundControl.RAW_IMU(data));
            }
            protected override void onCOMMAND_INT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_COMMAND_INT(this, new com.company.demo.GroundControl.COMMAND_INT(cur));
            }
            protected override void onOPTICAL_FLOW(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_OPTICAL_FLOW(this, new com.company.demo.GroundControl.OPTICAL_FLOW(cur));
            }
            protected override void onMISSION_ITEM_INT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_MISSION_ITEM_INT(this, new com.company.demo.GroundControl.MISSION_ITEM_INT(cur));
            }
            protected override void onHIGHRES_IMU(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_HIGHRES_IMU(this,  new com.company.demo.GroundControl.HIGHRES_IMU(data));
            }
            protected override void onEXTENDED_SYS_STATE(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_EXTENDED_SYS_STATE(this, new com.company.demo.GroundControl.EXTENDED_SYS_STATE(cur));
            }
            protected override void onGPS_INJECT_DATA(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_GPS_INJECT_DATA(this,  new com.company.demo.GroundControl.GPS_INJECT_DATA(data));
            }
            protected override void onATTITUDE_QUATERNION_COV(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_ATTITUDE_QUATERNION_COV(this,  new com.company.demo.GroundControl.ATTITUDE_QUATERNION_COV(data));
            }
            protected override void onNAMED_VALUE_INT(Pack pack)
            {
                var cur = new Pack.Cursor(null);
                cur.wrap(pack);
                on_NAMED_VALUE_INT(this, new com.company.demo.GroundControl.NAMED_VALUE_INT(cur));
            }
            protected override void onRADIO_STATUS(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_RADIO_STATUS(this,  new com.company.demo.GroundControl.RADIO_STATUS(data));
            }
            protected override void onGPS_RTCM_DATA(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_GPS_RTCM_DATA(this,  new com.company.demo.GroundControl.GPS_RTCM_DATA(data));
            }
            protected override void onGLOBAL_VISION_POSITION_ESTIMATE(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_GLOBAL_VISION_POSITION_ESTIMATE(this,  new com.company.demo.GroundControl.GLOBAL_VISION_POSITION_ESTIMATE(data));
            }
            protected override void onFILE_TRANSFER_PROTOCOL(Pack pack)
            {
                var data = new Pack.Cursor(null);
                data.wrap(pack);
                on_FILE_TRANSFER_PROTOCOL(this,  new com.company.demo.GroundControl.FILE_TRANSFER_PROTOCOL(data));
            }

            readonly Queue<Pack> sendingPacks = new Queue<Pack>();
            protected internal override Pack pullSendingPack() { return sendingPacks.Dequeue(); }
            protected internal override bool pushSendingPack(Pack pack) {  sendingPacks.Enqueue(pack); return true;}
            public  event Pack.Handler< CommunicationChannel_demo, RESOURCE_REQUEST > on_RESOURCE_REQUEST;
            public  event Pack.Handler< CommunicationChannel_demo, ATTITUDE_TARGET > on_ATTITUDE_TARGET;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_COUNT > on_MISSION_COUNT;
            public  event Pack.Handler< CommunicationChannel_demo, ADSB_VEHICLE > on_ADSB_VEHICLE;
            public  event Pack.Handler< CommunicationChannel_demo, MESSAGE_INTERVAL > on_MESSAGE_INTERVAL;
            public  event Pack.Handler< CommunicationChannel_demo, ESTIMATOR_STATUS > on_ESTIMATOR_STATUS;
            public  event Pack.Handler< CommunicationChannel_demo, TIMESYNC > on_TIMESYNC;
            public  event Pack.Handler< CommunicationChannel_demo, GLOBAL_POSITION_INT_COV > on_GLOBAL_POSITION_INT_COV;
            public  event Pack.Handler< CommunicationChannel_demo, BUTTON_CHANGE > on_BUTTON_CHANGE;
            public  event Pack.Handler< CommunicationChannel_demo, SAFETY_SET_ALLOWED_AREA > on_SAFETY_SET_ALLOWED_AREA;
            public  event Pack.Handler< CommunicationChannel_demo, STORAGE_INFORMATION > on_STORAGE_INFORMATION;
            public  event Pack.Handler< CommunicationChannel_demo, COLLISION > on_COLLISION;
            public  event Pack.Handler< CommunicationChannel_demo, ALTITUDE > on_ALTITUDE;
            public  event Pack.Handler< CommunicationChannel_demo, HIL_STATE_QUATERNION > on_HIL_STATE_QUATERNION;
            public  event Pack.Handler< CommunicationChannel_demo, CAMERA_INFORMATION > on_CAMERA_INFORMATION;
            public  event Pack.Handler< CommunicationChannel_demo, GPS_STATUS > on_GPS_STATUS;
            public  event Pack.Handler< CommunicationChannel_demo, PARAM_SET > on_PARAM_SET;
            public  event Pack.Handler< CommunicationChannel_demo, TERRAIN_DATA > on_TERRAIN_DATA;
            public  event Pack.Handler< CommunicationChannel_demo, RC_CHANNELS_OVERRIDE > on_RC_CHANNELS_OVERRIDE;
            public  event Pack.Handler< CommunicationChannel_demo, SCALED_IMU > on_SCALED_IMU;
            public  event Pack.Handler< CommunicationChannel_demo, DEBUG > on_DEBUG;
            public  event Pack.Handler< CommunicationChannel_demo, CAMERA_IMAGE_CAPTURED > on_CAMERA_IMAGE_CAPTURED;
            public  event Pack.Handler< CommunicationChannel_demo, LOG_ENTRY > on_LOG_ENTRY;
            public  event Pack.Handler< CommunicationChannel_demo, ACTUATOR_CONTROL_TARGET > on_ACTUATOR_CONTROL_TARGET;
            public  event Pack.Handler< CommunicationChannel_demo, HIGH_LATENCY > on_HIGH_LATENCY;
            public  event Pack.Handler< CommunicationChannel_demo, PARAM_REQUEST_READ > on_PARAM_REQUEST_READ;
            public  event Pack.Handler< CommunicationChannel_demo, SET_ATTITUDE_TARGET > on_SET_ATTITUDE_TARGET;
            public  event Pack.Handler< CommunicationChannel_demo, FOLLOW_TARGET > on_FOLLOW_TARGET;
            public  event Pack.Handler< CommunicationChannel_demo, HIL_STATE > on_HIL_STATE;
            public  event Pack.Handler< CommunicationChannel_demo, HOME_POSITION > on_HOME_POSITION;
            public  event Pack.Handler< CommunicationChannel_demo, GPS2_RAW > on_GPS2_RAW;
            public  event Pack.Handler< CommunicationChannel_demo, MEMORY_VECT > on_MEMORY_VECT;
            public  event Pack.Handler< CommunicationChannel_demo, REQUEST_DATA_STREAM > on_REQUEST_DATA_STREAM;
            public  event Pack.Handler< CommunicationChannel_demo, HIL_CONTROLS > on_HIL_CONTROLS;
            public  event Pack.Handler< CommunicationChannel_demo, HIL_SENSOR > on_HIL_SENSOR;
            public  event Pack.Handler< CommunicationChannel_demo, SETUP_SIGNING > on_SETUP_SIGNING;
            public  event Pack.Handler< CommunicationChannel_demo, GPS_RTK > on_GPS_RTK;
            public  event Pack.Handler< CommunicationChannel_demo, PARAM_REQUEST_LIST > on_PARAM_REQUEST_LIST;
            public  event Pack.Handler< CommunicationChannel_demo, LANDING_TARGET > on_LANDING_TARGET;
            public  event Pack.Handler< CommunicationChannel_demo, SET_ACTUATOR_CONTROL_TARGET > on_SET_ACTUATOR_CONTROL_TARGET;
            public  event Pack.Handler< CommunicationChannel_demo, CONTROL_SYSTEM_STATE > on_CONTROL_SYSTEM_STATE;
            public  event Pack.Handler< CommunicationChannel_demo, SET_POSITION_TARGET_GLOBAL_INT > on_SET_POSITION_TARGET_GLOBAL_INT;
            public  event Pack.Handler< CommunicationChannel_demo, VIBRATION > on_VIBRATION;
            public  event Pack.Handler< CommunicationChannel_demo, PING33 > on_PING33;
            public  event Pack.Handler< CommunicationChannel_demo, VFR_HUD > on_VFR_HUD;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_SET_CURRENT > on_MISSION_SET_CURRENT;
            public  event Pack.Handler< CommunicationChannel_demo, HIL_GPS > on_HIL_GPS;
            public  event Pack.Handler< CommunicationChannel_demo, NAV_CONTROLLER_OUTPUT > on_NAV_CONTROLLER_OUTPUT;
            public  event Pack.Handler< CommunicationChannel_demo, AUTH_KEY > on_AUTH_KEY;
            public  event Pack.Handler< CommunicationChannel_demo, LOCAL_POSITION_NED_COV > on_LOCAL_POSITION_NED_COV;
            public  event Pack.Handler< CommunicationChannel_demo, ATT_POS_MOCAP > on_ATT_POS_MOCAP;
            public  event Pack.Handler< CommunicationChannel_demo, STATUSTEXT > on_STATUSTEXT;
            public  event Pack.Handler< CommunicationChannel_demo, PING > on_PING;
            public  event Pack.Handler< CommunicationChannel_demo, CAMERA_CAPTURE_STATUS > on_CAMERA_CAPTURE_STATUS;
            public  event Pack.Handler< CommunicationChannel_demo, GLOBAL_POSITION_INT > on_GLOBAL_POSITION_INT;
            public  event Pack.Handler< CommunicationChannel_demo, ENCAPSULATED_DATA > on_ENCAPSULATED_DATA;
            public  event Pack.Handler< CommunicationChannel_demo, GPS_INPUT > on_GPS_INPUT;
            public  event Pack.Handler< CommunicationChannel_demo, COMMAND_LONG > on_COMMAND_LONG;
            public  event Pack.Handler< CommunicationChannel_demo, LOG_REQUEST_DATA > on_LOG_REQUEST_DATA;
            public  event Pack.Handler< CommunicationChannel_demo, GPS_RAW_INT > on_GPS_RAW_INT;
            public  event Pack.Handler< CommunicationChannel_demo, RC_CHANNELS_SCALED > on_RC_CHANNELS_SCALED;
            public  event Pack.Handler< CommunicationChannel_demo, CAMERA_SETTINGS > on_CAMERA_SETTINGS;
            public  event Pack.Handler< CommunicationChannel_demo, RAW_PRESSURE > on_RAW_PRESSURE;
            public  event Pack.Handler< CommunicationChannel_demo, NAMED_VALUE_FLOAT > on_NAMED_VALUE_FLOAT;
            public  event Pack.Handler< CommunicationChannel_demo, ATTITUDE > on_ATTITUDE;
            public  event Pack.Handler< CommunicationChannel_demo, TERRAIN_REQUEST > on_TERRAIN_REQUEST;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_WRITE_PARTIAL_LIST > on_MISSION_WRITE_PARTIAL_LIST;
            public  event Pack.Handler< CommunicationChannel_demo, LOG_ERASE > on_LOG_ERASE;
            public  event Pack.Handler< CommunicationChannel_demo, MANUAL_SETPOINT > on_MANUAL_SETPOINT;
            public  event Pack.Handler< CommunicationChannel_demo, SAFETY_ALLOWED_AREA > on_SAFETY_ALLOWED_AREA;
            public  event Pack.Handler< CommunicationChannel_demo, OPTICAL_FLOW_RAD > on_OPTICAL_FLOW_RAD;
            public  event Pack.Handler< CommunicationChannel_demo, LOG_DATA > on_LOG_DATA;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_CLEAR_ALL > on_MISSION_CLEAR_ALL;
            public  event Pack.Handler< CommunicationChannel_demo, VICON_POSITION_ESTIMATE > on_VICON_POSITION_ESTIMATE;
            public  event Pack.Handler< CommunicationChannel_demo, GPS2_RTK > on_GPS2_RTK;
            public  event Pack.Handler< CommunicationChannel_demo, LOG_REQUEST_LIST > on_LOG_REQUEST_LIST;
            public  event Pack.Handler< CommunicationChannel_demo, SCALED_PRESSURE > on_SCALED_PRESSURE;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_REQUEST_INT > on_MISSION_REQUEST_INT;
            public  event Pack.Handler< CommunicationChannel_demo, V2_EXTENSION > on_V2_EXTENSION;
            public  event Pack.Handler< CommunicationChannel_demo, HEARTBEAT > on_HEARTBEAT;
            public  event Pack.Handler< CommunicationChannel_demo, PARAM_MAP_RC > on_PARAM_MAP_RC;
            public  event Pack.Handler< CommunicationChannel_demo, POWER_STATUS > on_POWER_STATUS;
            public  event Pack.Handler< CommunicationChannel_demo, TERRAIN_CHECK > on_TERRAIN_CHECK;
            public  event Pack.Handler< CommunicationChannel_demo, LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET > on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;
            public  event Pack.Handler< CommunicationChannel_demo, COMMAND_ACK > on_COMMAND_ACK;
            public  event Pack.Handler< CommunicationChannel_demo, DATA_STREAM > on_DATA_STREAM;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_REQUEST > on_MISSION_REQUEST;
            public  event Pack.Handler< CommunicationChannel_demo, TERRAIN_REPORT > on_TERRAIN_REPORT;
            public  event Pack.Handler< CommunicationChannel_demo, SET_HOME_POSITION > on_SET_HOME_POSITION;
            public  event Pack.Handler< CommunicationChannel_demo, object > on_SwitchModeCommand;
            public  event Pack.Handler< CommunicationChannel_demo, HIL_RC_INPUTS_RAW > on_HIL_RC_INPUTS_RAW;
            public  event Pack.Handler< CommunicationChannel_demo, SCALED_IMU3 > on_SCALED_IMU3;
            public  event Pack.Handler< CommunicationChannel_demo, SET_MODE > on_SET_MODE;
            public  event Pack.Handler< CommunicationChannel_demo, POSITION_TARGET_GLOBAL_INT > on_POSITION_TARGET_GLOBAL_INT;
            public  event Pack.Handler< CommunicationChannel_demo, FLIGHT_INFORMATION > on_FLIGHT_INFORMATION;
            public  event Pack.Handler< CommunicationChannel_demo, SIM_STATE > on_SIM_STATE;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_ITEM_REACHED > on_MISSION_ITEM_REACHED;
            public  event Pack.Handler< CommunicationChannel_demo, RC_CHANNELS_RAW > on_RC_CHANNELS_RAW;
            public  event Pack.Handler< CommunicationChannel_demo, SERVO_OUTPUT_RAW > on_SERVO_OUTPUT_RAW;
            public  event Pack.Handler< CommunicationChannel_demo, VISION_SPEED_ESTIMATE > on_VISION_SPEED_ESTIMATE;
            public  event Pack.Handler< CommunicationChannel_demo, DEBUG_VECT > on_DEBUG_VECT;
            public  event Pack.Handler< CommunicationChannel_demo, LOG_REQUEST_END > on_LOG_REQUEST_END;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_ACK > on_MISSION_ACK;
            public  event Pack.Handler< CommunicationChannel_demo, CHANGE_OPERATOR_CONTROL_ACK > on_CHANGE_OPERATOR_CONTROL_ACK;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_CURRENT > on_MISSION_CURRENT;
            public  event Pack.Handler< CommunicationChannel_demo, SYSTEM_TIME > on_SYSTEM_TIME;
            public  event Pack.Handler< CommunicationChannel_demo, CAMERA_TRIGGER > on_CAMERA_TRIGGER;
            public  event Pack.Handler< CommunicationChannel_demo, VISION_POSITION_ESTIMATE > on_VISION_POSITION_ESTIMATE;
            public  event Pack.Handler< CommunicationChannel_demo, MANUAL_CONTROL > on_MANUAL_CONTROL;
            public  event Pack.Handler< CommunicationChannel_demo, RC_CHANNELS > on_RC_CHANNELS;
            public  event Pack.Handler< CommunicationChannel_demo, PARAM_VALUE > on_PARAM_VALUE;
            public  event Pack.Handler< CommunicationChannel_demo, BATTERY_STATUS > on_BATTERY_STATUS;
            public  event Pack.Handler< CommunicationChannel_demo, SET_POSITION_TARGET_LOCAL_NED > on_SET_POSITION_TARGET_LOCAL_NED;
            public  event Pack.Handler< CommunicationChannel_demo, SERIAL_CONTROL > on_SERIAL_CONTROL;
            public  event Pack.Handler< CommunicationChannel_demo, SET_GPS_GLOBAL_ORIGIN > on_SET_GPS_GLOBAL_ORIGIN;
            public  event Pack.Handler< CommunicationChannel_demo, AUTOPILOT_VERSION > on_AUTOPILOT_VERSION;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_REQUEST_LIST > on_MISSION_REQUEST_LIST;
            public  event Pack.Handler< CommunicationChannel_demo, PLAY_TUNE > on_PLAY_TUNE;
            public  event Pack.Handler< CommunicationChannel_demo, SCALED_PRESSURE3 > on_SCALED_PRESSURE3;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_REQUEST_PARTIAL_LIST > on_MISSION_REQUEST_PARTIAL_LIST;
            public  event Pack.Handler< CommunicationChannel_demo, LOCAL_POSITION_NED > on_LOCAL_POSITION_NED;
            public  event Pack.Handler< CommunicationChannel_demo, DATA_TRANSMISSION_HANDSHAKE > on_DATA_TRANSMISSION_HANDSHAKE;
            public  event Pack.Handler< CommunicationChannel_demo, GPS_GLOBAL_ORIGIN > on_GPS_GLOBAL_ORIGIN;
            public  event Pack.Handler< CommunicationChannel_demo, SCALED_IMU2 > on_SCALED_IMU2;
            public  event Pack.Handler< CommunicationChannel_demo, ATTITUDE_QUATERNION > on_ATTITUDE_QUATERNION;
            public  event Pack.Handler< CommunicationChannel_demo, HIL_ACTUATOR_CONTROLS > on_HIL_ACTUATOR_CONTROLS;
            public  event Pack.Handler< CommunicationChannel_demo, POSITION_TARGET_LOCAL_NED > on_POSITION_TARGET_LOCAL_NED;
            public  event Pack.Handler< CommunicationChannel_demo, DISTANCE_SENSOR > on_DISTANCE_SENSOR;
            public  event Pack.Handler< CommunicationChannel_demo, HIL_OPTICAL_FLOW > on_HIL_OPTICAL_FLOW;
            public  event Pack.Handler< CommunicationChannel_demo, SCALED_PRESSURE2 > on_SCALED_PRESSURE2;
            public  event Pack.Handler< CommunicationChannel_demo, WIND_COV > on_WIND_COV;
            public  event Pack.Handler< CommunicationChannel_demo, CHANGE_OPERATOR_CONTROL > on_CHANGE_OPERATOR_CONTROL;
            public  event Pack.Handler< CommunicationChannel_demo, SYS_STATUS > on_SYS_STATUS;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_ITEM > on_MISSION_ITEM;
            public  event Pack.Handler< CommunicationChannel_demo, RAW_IMU > on_RAW_IMU;
            public  event Pack.Handler< CommunicationChannel_demo, COMMAND_INT > on_COMMAND_INT;
            public  event Pack.Handler< CommunicationChannel_demo, OPTICAL_FLOW > on_OPTICAL_FLOW;
            public  event Pack.Handler< CommunicationChannel_demo, MISSION_ITEM_INT > on_MISSION_ITEM_INT;
            public  event Pack.Handler< CommunicationChannel_demo, HIGHRES_IMU > on_HIGHRES_IMU;
            public  event Pack.Handler< CommunicationChannel_demo, EXTENDED_SYS_STATE > on_EXTENDED_SYS_STATE;
            public  event Pack.Handler< CommunicationChannel_demo, GPS_INJECT_DATA > on_GPS_INJECT_DATA;
            public  event Pack.Handler< CommunicationChannel_demo, ATTITUDE_QUATERNION_COV > on_ATTITUDE_QUATERNION_COV;
            public  event Pack.Handler< CommunicationChannel_demo, NAMED_VALUE_INT > on_NAMED_VALUE_INT;
            public  event Pack.Handler< CommunicationChannel_demo, RADIO_STATUS > on_RADIO_STATUS;
            public  event Pack.Handler< CommunicationChannel_demo, GPS_RTCM_DATA > on_GPS_RTCM_DATA;
            public  event Pack.Handler< CommunicationChannel_demo, GLOBAL_VISION_POSITION_ESTIMATE > on_GLOBAL_VISION_POSITION_ESTIMATE;
            public  event Pack.Handler< CommunicationChannel_demo, FILE_TRANSFER_PROTOCOL > on_FILE_TRANSFER_PROTOCOL;

        }

        public static void Main2(string[] args)
        {
            var cur = new Pack.Cursor(null);
            var buff = new byte[512];
            int bytes_out = 0;
            var CommunicationChannel_instance = new  CommunicationChannel_demo();
            CommunicationChannel_instance.on_RESOURCE_REQUEST += (ch, pack) =>  onRESOURCE_REQUEST(pack);
            CommunicationChannel_instance.on_ATTITUDE_TARGET += (ch, pack) =>  onATTITUDE_TARGET(pack);
            CommunicationChannel_instance.on_MISSION_COUNT += (ch, pack) =>  onMISSION_COUNT(pack);
            CommunicationChannel_instance.on_ADSB_VEHICLE += (ch, pack) =>  onADSB_VEHICLE(pack);
            CommunicationChannel_instance.on_MESSAGE_INTERVAL += (ch, pack) =>  onMESSAGE_INTERVAL(pack);
            CommunicationChannel_instance.on_ESTIMATOR_STATUS += (ch, pack) =>  onESTIMATOR_STATUS(pack);
            CommunicationChannel_instance.on_TIMESYNC += (ch, pack) =>  onTIMESYNC(pack);
            CommunicationChannel_instance.on_GLOBAL_POSITION_INT_COV += (ch, pack) =>  onGLOBAL_POSITION_INT_COV(pack);
            CommunicationChannel_instance.on_BUTTON_CHANGE += (ch, pack) =>  onBUTTON_CHANGE(pack);
            CommunicationChannel_instance.on_SAFETY_SET_ALLOWED_AREA += (ch, pack) =>  onSAFETY_SET_ALLOWED_AREA(pack);
            CommunicationChannel_instance.on_STORAGE_INFORMATION += (ch, pack) =>  onSTORAGE_INFORMATION(pack);
            CommunicationChannel_instance.on_COLLISION += (ch, pack) =>  onCOLLISION(pack);
            CommunicationChannel_instance.on_ALTITUDE += (ch, pack) =>  onALTITUDE(pack);
            CommunicationChannel_instance.on_HIL_STATE_QUATERNION += (ch, pack) =>  onHIL_STATE_QUATERNION(pack);
            CommunicationChannel_instance.on_CAMERA_INFORMATION += (ch, pack) =>  onCAMERA_INFORMATION(pack);
            CommunicationChannel_instance.on_GPS_STATUS += (ch, pack) =>  onGPS_STATUS(pack);
            CommunicationChannel_instance.on_PARAM_SET += (ch, pack) =>  onPARAM_SET(pack);
            CommunicationChannel_instance.on_TERRAIN_DATA += (ch, pack) =>  onTERRAIN_DATA(pack);
            CommunicationChannel_instance.on_RC_CHANNELS_OVERRIDE += (ch, pack) =>  onRC_CHANNELS_OVERRIDE(pack);
            CommunicationChannel_instance.on_SCALED_IMU += (ch, pack) =>  onSCALED_IMU(pack);
            CommunicationChannel_instance.on_DEBUG += (ch, pack) =>  onDEBUG(pack);
            CommunicationChannel_instance.on_CAMERA_IMAGE_CAPTURED += (ch, pack) =>  onCAMERA_IMAGE_CAPTURED(pack);
            CommunicationChannel_instance.on_LOG_ENTRY += (ch, pack) =>  onLOG_ENTRY(pack);
            CommunicationChannel_instance.on_ACTUATOR_CONTROL_TARGET += (ch, pack) =>  onACTUATOR_CONTROL_TARGET(pack);
            CommunicationChannel_instance.on_HIGH_LATENCY += (ch, pack) =>  onHIGH_LATENCY(pack);
            CommunicationChannel_instance.on_PARAM_REQUEST_READ += (ch, pack) =>  onPARAM_REQUEST_READ(pack);
            CommunicationChannel_instance.on_SET_ATTITUDE_TARGET += (ch, pack) =>  onSET_ATTITUDE_TARGET(pack);
            CommunicationChannel_instance.on_FOLLOW_TARGET += (ch, pack) =>  onFOLLOW_TARGET(pack);
            CommunicationChannel_instance.on_HIL_STATE += (ch, pack) =>  onHIL_STATE(pack);
            CommunicationChannel_instance.on_HOME_POSITION += (ch, pack) =>  onHOME_POSITION(pack);
            CommunicationChannel_instance.on_GPS2_RAW += (ch, pack) =>  onGPS2_RAW(pack);
            CommunicationChannel_instance.on_MEMORY_VECT += (ch, pack) =>  onMEMORY_VECT(pack);
            CommunicationChannel_instance.on_REQUEST_DATA_STREAM += (ch, pack) =>  onREQUEST_DATA_STREAM(pack);
            CommunicationChannel_instance.on_HIL_CONTROLS += (ch, pack) =>  onHIL_CONTROLS(pack);
            CommunicationChannel_instance.on_HIL_SENSOR += (ch, pack) =>  onHIL_SENSOR(pack);
            CommunicationChannel_instance.on_SETUP_SIGNING += (ch, pack) =>  onSETUP_SIGNING(pack);
            CommunicationChannel_instance.on_GPS_RTK += (ch, pack) =>  onGPS_RTK(pack);
            CommunicationChannel_instance.on_PARAM_REQUEST_LIST += (ch, pack) =>  onPARAM_REQUEST_LIST(pack);
            CommunicationChannel_instance.on_LANDING_TARGET += (ch, pack) =>  onLANDING_TARGET(pack);
            CommunicationChannel_instance.on_SET_ACTUATOR_CONTROL_TARGET += (ch, pack) =>  onSET_ACTUATOR_CONTROL_TARGET(pack);
            CommunicationChannel_instance.on_CONTROL_SYSTEM_STATE += (ch, pack) =>  onCONTROL_SYSTEM_STATE(pack);
            CommunicationChannel_instance.on_SET_POSITION_TARGET_GLOBAL_INT += (ch, pack) =>  onSET_POSITION_TARGET_GLOBAL_INT(pack);
            CommunicationChannel_instance.on_VIBRATION += (ch, pack) =>  onVIBRATION(pack);
            CommunicationChannel_instance.on_PING33 += (ch, pack) =>  onPING33(pack);
            CommunicationChannel_instance.on_VFR_HUD += (ch, pack) =>  onVFR_HUD(pack);
            CommunicationChannel_instance.on_MISSION_SET_CURRENT += (ch, pack) =>  onMISSION_SET_CURRENT(pack);
            CommunicationChannel_instance.on_HIL_GPS += (ch, pack) =>  onHIL_GPS(pack);
            CommunicationChannel_instance.on_NAV_CONTROLLER_OUTPUT += (ch, pack) =>  onNAV_CONTROLLER_OUTPUT(pack);
            CommunicationChannel_instance.on_AUTH_KEY += (ch, pack) =>  onAUTH_KEY(pack);
            CommunicationChannel_instance.on_LOCAL_POSITION_NED_COV += (ch, pack) =>  onLOCAL_POSITION_NED_COV(pack);
            CommunicationChannel_instance.on_ATT_POS_MOCAP += (ch, pack) =>  onATT_POS_MOCAP(pack);
            CommunicationChannel_instance.on_STATUSTEXT += (ch, pack) =>  onSTATUSTEXT(pack);
            CommunicationChannel_instance.on_PING += (ch, pack) =>  onPING(pack);
            CommunicationChannel_instance.on_CAMERA_CAPTURE_STATUS += (ch, pack) =>  onCAMERA_CAPTURE_STATUS(pack);
            CommunicationChannel_instance.on_GLOBAL_POSITION_INT += (ch, pack) =>  onGLOBAL_POSITION_INT(pack);
            CommunicationChannel_instance.on_ENCAPSULATED_DATA += (ch, pack) =>  onENCAPSULATED_DATA(pack);
            CommunicationChannel_instance.on_GPS_INPUT += (ch, pack) =>  onGPS_INPUT(pack);
            CommunicationChannel_instance.on_COMMAND_LONG += (ch, pack) =>  onCOMMAND_LONG(pack);
            CommunicationChannel_instance.on_LOG_REQUEST_DATA += (ch, pack) =>  onLOG_REQUEST_DATA(pack);
            CommunicationChannel_instance.on_GPS_RAW_INT += (ch, pack) =>  onGPS_RAW_INT(pack);
            CommunicationChannel_instance.on_RC_CHANNELS_SCALED += (ch, pack) =>  onRC_CHANNELS_SCALED(pack);
            CommunicationChannel_instance.on_CAMERA_SETTINGS += (ch, pack) =>  onCAMERA_SETTINGS(pack);
            CommunicationChannel_instance.on_RAW_PRESSURE += (ch, pack) =>  onRAW_PRESSURE(pack);
            CommunicationChannel_instance.on_NAMED_VALUE_FLOAT += (ch, pack) =>  onNAMED_VALUE_FLOAT(pack);
            CommunicationChannel_instance.on_ATTITUDE += (ch, pack) =>  onATTITUDE(pack);
            CommunicationChannel_instance.on_TERRAIN_REQUEST += (ch, pack) =>  onTERRAIN_REQUEST(pack);
            CommunicationChannel_instance.on_MISSION_WRITE_PARTIAL_LIST += (ch, pack) =>  onMISSION_WRITE_PARTIAL_LIST(pack);
            CommunicationChannel_instance.on_LOG_ERASE += (ch, pack) =>  onLOG_ERASE(pack);
            CommunicationChannel_instance.on_MANUAL_SETPOINT += (ch, pack) =>  onMANUAL_SETPOINT(pack);
            CommunicationChannel_instance.on_SAFETY_ALLOWED_AREA += (ch, pack) =>  onSAFETY_ALLOWED_AREA(pack);
            CommunicationChannel_instance.on_OPTICAL_FLOW_RAD += (ch, pack) =>  onOPTICAL_FLOW_RAD(pack);
            CommunicationChannel_instance.on_LOG_DATA += (ch, pack) =>  onLOG_DATA(pack);
            CommunicationChannel_instance.on_MISSION_CLEAR_ALL += (ch, pack) =>  onMISSION_CLEAR_ALL(pack);
            CommunicationChannel_instance.on_VICON_POSITION_ESTIMATE += (ch, pack) =>  onVICON_POSITION_ESTIMATE(pack);
            CommunicationChannel_instance.on_GPS2_RTK += (ch, pack) =>  onGPS2_RTK(pack);
            CommunicationChannel_instance.on_LOG_REQUEST_LIST += (ch, pack) =>  onLOG_REQUEST_LIST(pack);
            CommunicationChannel_instance.on_SCALED_PRESSURE += (ch, pack) =>  onSCALED_PRESSURE(pack);
            CommunicationChannel_instance.on_MISSION_REQUEST_INT += (ch, pack) =>  onMISSION_REQUEST_INT(pack);
            CommunicationChannel_instance.on_V2_EXTENSION += (ch, pack) =>  onV2_EXTENSION(pack);
            CommunicationChannel_instance.on_HEARTBEAT += (ch, pack) =>  onHEARTBEAT(pack);
            CommunicationChannel_instance.on_PARAM_MAP_RC += (ch, pack) =>  onPARAM_MAP_RC(pack);
            CommunicationChannel_instance.on_POWER_STATUS += (ch, pack) =>  onPOWER_STATUS(pack);
            CommunicationChannel_instance.on_TERRAIN_CHECK += (ch, pack) =>  onTERRAIN_CHECK(pack);
            CommunicationChannel_instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET += (ch, pack) =>  onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(pack);
            CommunicationChannel_instance.on_COMMAND_ACK += (ch, pack) =>  onCOMMAND_ACK(pack);
            CommunicationChannel_instance.on_DATA_STREAM += (ch, pack) =>  onDATA_STREAM(pack);
            CommunicationChannel_instance.on_MISSION_REQUEST += (ch, pack) =>  onMISSION_REQUEST(pack);
            CommunicationChannel_instance.on_TERRAIN_REPORT += (ch, pack) =>  onTERRAIN_REPORT(pack);
            CommunicationChannel_instance.on_SET_HOME_POSITION += (ch, pack) =>  onSET_HOME_POSITION(pack);
            CommunicationChannel_instance.on_SwitchModeCommand += (ch, pack) => onSwitchModeCommand();
            CommunicationChannel_instance.on_HIL_RC_INPUTS_RAW += (ch, pack) =>  onHIL_RC_INPUTS_RAW(pack);
            CommunicationChannel_instance.on_SCALED_IMU3 += (ch, pack) =>  onSCALED_IMU3(pack);
            CommunicationChannel_instance.on_SET_MODE += (ch, pack) =>  onSET_MODE(pack);
            CommunicationChannel_instance.on_POSITION_TARGET_GLOBAL_INT += (ch, pack) =>  onPOSITION_TARGET_GLOBAL_INT(pack);
            CommunicationChannel_instance.on_FLIGHT_INFORMATION += (ch, pack) =>  onFLIGHT_INFORMATION(pack);
            CommunicationChannel_instance.on_SIM_STATE += (ch, pack) =>  onSIM_STATE(pack);
            CommunicationChannel_instance.on_MISSION_ITEM_REACHED += (ch, pack) =>  onMISSION_ITEM_REACHED(pack);
            CommunicationChannel_instance.on_RC_CHANNELS_RAW += (ch, pack) =>  onRC_CHANNELS_RAW(pack);
            CommunicationChannel_instance.on_SERVO_OUTPUT_RAW += (ch, pack) =>  onSERVO_OUTPUT_RAW(pack);
            CommunicationChannel_instance.on_VISION_SPEED_ESTIMATE += (ch, pack) =>  onVISION_SPEED_ESTIMATE(pack);
            CommunicationChannel_instance.on_DEBUG_VECT += (ch, pack) =>  onDEBUG_VECT(pack);
            CommunicationChannel_instance.on_LOG_REQUEST_END += (ch, pack) =>  onLOG_REQUEST_END(pack);
            CommunicationChannel_instance.on_MISSION_ACK += (ch, pack) =>  onMISSION_ACK(pack);
            CommunicationChannel_instance.on_CHANGE_OPERATOR_CONTROL_ACK += (ch, pack) =>  onCHANGE_OPERATOR_CONTROL_ACK(pack);
            CommunicationChannel_instance.on_MISSION_CURRENT += (ch, pack) =>  onMISSION_CURRENT(pack);
            CommunicationChannel_instance.on_SYSTEM_TIME += (ch, pack) =>  onSYSTEM_TIME(pack);
            CommunicationChannel_instance.on_CAMERA_TRIGGER += (ch, pack) =>  onCAMERA_TRIGGER(pack);
            CommunicationChannel_instance.on_VISION_POSITION_ESTIMATE += (ch, pack) =>  onVISION_POSITION_ESTIMATE(pack);
            CommunicationChannel_instance.on_MANUAL_CONTROL += (ch, pack) =>  onMANUAL_CONTROL(pack);
            CommunicationChannel_instance.on_RC_CHANNELS += (ch, pack) =>  onRC_CHANNELS(pack);
            CommunicationChannel_instance.on_PARAM_VALUE += (ch, pack) =>  onPARAM_VALUE(pack);
            CommunicationChannel_instance.on_BATTERY_STATUS += (ch, pack) =>  onBATTERY_STATUS(pack);
            CommunicationChannel_instance.on_SET_POSITION_TARGET_LOCAL_NED += (ch, pack) =>  onSET_POSITION_TARGET_LOCAL_NED(pack);
            CommunicationChannel_instance.on_SERIAL_CONTROL += (ch, pack) =>  onSERIAL_CONTROL(pack);
            CommunicationChannel_instance.on_SET_GPS_GLOBAL_ORIGIN += (ch, pack) =>  onSET_GPS_GLOBAL_ORIGIN(pack);
            CommunicationChannel_instance.on_AUTOPILOT_VERSION += (ch, pack) =>  onAUTOPILOT_VERSION(pack);
            CommunicationChannel_instance.on_MISSION_REQUEST_LIST += (ch, pack) =>  onMISSION_REQUEST_LIST(pack);
            CommunicationChannel_instance.on_PLAY_TUNE += (ch, pack) =>  onPLAY_TUNE(pack);
            CommunicationChannel_instance.on_SCALED_PRESSURE3 += (ch, pack) =>  onSCALED_PRESSURE3(pack);
            CommunicationChannel_instance.on_MISSION_REQUEST_PARTIAL_LIST += (ch, pack) =>  onMISSION_REQUEST_PARTIAL_LIST(pack);
            CommunicationChannel_instance.on_LOCAL_POSITION_NED += (ch, pack) =>  onLOCAL_POSITION_NED(pack);
            CommunicationChannel_instance.on_DATA_TRANSMISSION_HANDSHAKE += (ch, pack) =>  onDATA_TRANSMISSION_HANDSHAKE(pack);
            CommunicationChannel_instance.on_GPS_GLOBAL_ORIGIN += (ch, pack) =>  onGPS_GLOBAL_ORIGIN(pack);
            CommunicationChannel_instance.on_SCALED_IMU2 += (ch, pack) =>  onSCALED_IMU2(pack);
            CommunicationChannel_instance.on_ATTITUDE_QUATERNION += (ch, pack) =>  onATTITUDE_QUATERNION(pack);
            CommunicationChannel_instance.on_HIL_ACTUATOR_CONTROLS += (ch, pack) =>  onHIL_ACTUATOR_CONTROLS(pack);
            CommunicationChannel_instance.on_POSITION_TARGET_LOCAL_NED += (ch, pack) =>  onPOSITION_TARGET_LOCAL_NED(pack);
            CommunicationChannel_instance.on_DISTANCE_SENSOR += (ch, pack) =>  onDISTANCE_SENSOR(pack);
            CommunicationChannel_instance.on_HIL_OPTICAL_FLOW += (ch, pack) =>  onHIL_OPTICAL_FLOW(pack);
            CommunicationChannel_instance.on_SCALED_PRESSURE2 += (ch, pack) =>  onSCALED_PRESSURE2(pack);
            CommunicationChannel_instance.on_WIND_COV += (ch, pack) =>  onWIND_COV(pack);
            CommunicationChannel_instance.on_CHANGE_OPERATOR_CONTROL += (ch, pack) =>  onCHANGE_OPERATOR_CONTROL(pack);
            CommunicationChannel_instance.on_SYS_STATUS += (ch, pack) =>  onSYS_STATUS(pack);
            CommunicationChannel_instance.on_MISSION_ITEM += (ch, pack) =>  onMISSION_ITEM(pack);
            CommunicationChannel_instance.on_RAW_IMU += (ch, pack) =>  onRAW_IMU(pack);
            CommunicationChannel_instance.on_COMMAND_INT += (ch, pack) =>  onCOMMAND_INT(pack);
            CommunicationChannel_instance.on_OPTICAL_FLOW += (ch, pack) =>  onOPTICAL_FLOW(pack);
            CommunicationChannel_instance.on_MISSION_ITEM_INT += (ch, pack) =>  onMISSION_ITEM_INT(pack);
            CommunicationChannel_instance.on_HIGHRES_IMU += (ch, pack) =>  onHIGHRES_IMU(pack);
            CommunicationChannel_instance.on_EXTENDED_SYS_STATE += (ch, pack) =>  onEXTENDED_SYS_STATE(pack);
            CommunicationChannel_instance.on_GPS_INJECT_DATA += (ch, pack) =>  onGPS_INJECT_DATA(pack);
            CommunicationChannel_instance.on_ATTITUDE_QUATERNION_COV += (ch, pack) =>  onATTITUDE_QUATERNION_COV(pack);
            CommunicationChannel_instance.on_NAMED_VALUE_INT += (ch, pack) =>  onNAMED_VALUE_INT(pack);
            CommunicationChannel_instance.on_RADIO_STATUS += (ch, pack) =>  onRADIO_STATUS(pack);
            CommunicationChannel_instance.on_GPS_RTCM_DATA += (ch, pack) =>  onGPS_RTCM_DATA(pack);
            CommunicationChannel_instance.on_GLOBAL_VISION_POSITION_ESTIMATE += (ch, pack) =>  onGLOBAL_VISION_POSITION_ESTIMATE(pack);
            CommunicationChannel_instance.on_FILE_TRANSFER_PROTOCOL += (ch, pack) =>  onFILE_TRANSFER_PROTOCOL(pack);
            {
                var pfollow_target = CommunicationChannel.NEW.FOLLOW_TARGET(cur);
                fill(pfollow_target);
                if(!CommunicationChannel_instance.send(pfollow_target))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var padsb_vehicle = CommunicationChannel.NEW.ADSB_VEHICLE(cur);
                fill(padsb_vehicle);
                if(!CommunicationChannel_instance.send(padsb_vehicle))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmessage_interval = CommunicationChannel.NEW.MESSAGE_INTERVAL(cur);
                fill(pmessage_interval);
                if(!CommunicationChannel_instance.send(pmessage_interval))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pekf_status_report = CommunicationChannel.NEW.EKF_STATUS_REPORT(cur);
                fill(pekf_status_report);
                if(!CommunicationChannel_instance.send(pekf_status_report))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pestimator_status = CommunicationChannel.NEW.ESTIMATOR_STATUS(cur);
                fill(pestimator_status);
                if(!CommunicationChannel_instance.send(pestimator_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var phwstatus = CommunicationChannel.NEW.HWSTATUS(cur);
                fill(phwstatus);
                if(!CommunicationChannel_instance.send(phwstatus))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var ptimesync = CommunicationChannel.NEW.TIMESYNC(cur);
                fill(ptimesync);
                if(!CommunicationChannel_instance.send(ptimesync))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pparam_ext_request_list = CommunicationChannel.NEW.PARAM_EXT_REQUEST_LIST(cur);
                fill(pparam_ext_request_list);
                if(!CommunicationChannel_instance.send(pparam_ext_request_list))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pbutton_change = CommunicationChannel.NEW.BUTTON_CHANGE(cur);
                fill(pbutton_change);
                if(!CommunicationChannel_instance.send(pbutton_change))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var puavcan_node_status = CommunicationChannel.NEW.UAVCAN_NODE_STATUS(cur);
                fill(puavcan_node_status);
                if(!CommunicationChannel_instance.send(puavcan_node_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcollision = CommunicationChannel.NEW.COLLISION(cur);
                fill(pcollision);
                if(!CommunicationChannel_instance.send(pcollision))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgimbal_torque_cmd_report = CommunicationChannel.NEW.GIMBAL_TORQUE_CMD_REPORT(cur);
                fill(pgimbal_torque_cmd_report);
                if(!CommunicationChannel_instance.send(pgimbal_torque_cmd_report))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var paltitude = CommunicationChannel.NEW.ALTITUDE(cur);
                fill(paltitude);
                if(!CommunicationChannel_instance.send(paltitude))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var phil_state_quaternion = CommunicationChannel.NEW.HIL_STATE_QUATERNION(cur);
                fill(phil_state_quaternion);
                if(!CommunicationChannel_instance.send(phil_state_quaternion))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var psensor_offsets = CommunicationChannel.NEW.SENSOR_OFFSETS(cur);
                fill(psensor_offsets);
                if(!CommunicationChannel_instance.send(psensor_offsets))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pstorage_information = CommunicationChannel.NEW.STORAGE_INFORMATION(cur);
                fill(pstorage_information);
                if(!CommunicationChannel_instance.send(pstorage_information))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcamera_information = CommunicationChannel.NEW.CAMERA_INFORMATION(cur);
                fill(pcamera_information);
                if(!CommunicationChannel_instance.send(pcamera_information))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdevice_op_write_reply = CommunicationChannel.NEW.DEVICE_OP_WRITE_REPLY(cur);
                fill(pdevice_op_write_reply);
                if(!CommunicationChannel_instance.send(pdevice_op_write_reply))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pterrain_data = CommunicationChannel.NEW.TERRAIN_DATA(cur);
                fill(pterrain_data);
                if(!CommunicationChannel_instance.send(pterrain_data))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgimbal_control = CommunicationChannel.NEW.GIMBAL_CONTROL(cur);
                fill(pgimbal_control);
                if(!CommunicationChannel_instance.send(pgimbal_control))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pvideo_stream_information = CommunicationChannel.NEW.VIDEO_STREAM_INFORMATION(cur);
                fill(pvideo_stream_information);
                if(!CommunicationChannel_instance.send(pvideo_stream_information))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pahrs = CommunicationChannel.NEW.AHRS(cur);
                fill(pahrs);
                if(!CommunicationChannel_instance.send(pahrs))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdebug = CommunicationChannel.NEW.DEBUG(cur);
                fill(pdebug);
                if(!CommunicationChannel_instance.send(pdebug))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcamera_image_captured = CommunicationChannel.NEW.CAMERA_IMAGE_CAPTURED(cur);
                fill(pcamera_image_captured);
                if(!CommunicationChannel_instance.send(pcamera_image_captured))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plog_entry = CommunicationChannel.NEW.LOG_ENTRY(cur);
                fill(plog_entry);
                if(!CommunicationChannel_instance.send(plog_entry))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pactuator_control_target = CommunicationChannel.NEW.ACTUATOR_CONTROL_TARGET(cur);
                fill(pactuator_control_target);
                if(!CommunicationChannel_instance.send(pactuator_control_target))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var phigh_latency = CommunicationChannel.NEW.HIGH_LATENCY(cur);
                fill(phigh_latency);
                if(!CommunicationChannel_instance.send(phigh_latency))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var phome_position = CommunicationChannel.NEW.HOME_POSITION(cur);
                fill(phome_position);
                if(!CommunicationChannel_instance.send(phome_position))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pfence_status = CommunicationChannel.NEW.FENCE_STATUS(cur);
                fill(pfence_status);
                if(!CommunicationChannel_instance.send(pfence_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var premote_log_block_status = CommunicationChannel.NEW.REMOTE_LOG_BLOCK_STATUS(cur);
                fill(premote_log_block_status);
                if(!CommunicationChannel_instance.send(premote_log_block_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pobstacle_distance = CommunicationChannel.NEW.OBSTACLE_DISTANCE(cur);
                fill(pobstacle_distance);
                if(!CommunicationChannel_instance.send(pobstacle_distance))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgps2_raw = CommunicationChannel.NEW.GPS2_RAW(cur);
                fill(pgps2_raw);
                if(!CommunicationChannel_instance.send(pgps2_raw))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmemory_vect = CommunicationChannel.NEW.MEMORY_VECT(cur);
                fill(pmemory_vect);
                if(!CommunicationChannel_instance.send(pmemory_vect))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pparam_ext_request_read = CommunicationChannel.NEW.PARAM_EXT_REQUEST_READ(cur);
                fill(pparam_ext_request_read);
                if(!CommunicationChannel_instance.send(pparam_ext_request_read))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var phil_sensor = CommunicationChannel.NEW.HIL_SENSOR(cur);
                fill(phil_sensor);
                if(!CommunicationChannel_instance.send(phil_sensor))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var psetup_signing = CommunicationChannel.NEW.SETUP_SIGNING(cur);
                fill(psetup_signing);
                if(!CommunicationChannel_instance.send(psetup_signing))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgps_rtk = CommunicationChannel.NEW.GPS_RTK(cur);
                fill(pgps_rtk);
                if(!CommunicationChannel_instance.send(pgps_rtk))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var puavionix_adsb_out_cfg = CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_CFG(cur);
                fill(puavionix_adsb_out_cfg);
                if(!CommunicationChannel_instance.send(puavionix_adsb_out_cfg))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var planding_target = CommunicationChannel.NEW.LANDING_TARGET(cur);
                fill(planding_target);
                if(!CommunicationChannel_instance.send(planding_target))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pset_actuator_control_target = CommunicationChannel.NEW.SET_ACTUATOR_CONTROL_TARGET(cur);
                fill(pset_actuator_control_target);
                if(!CommunicationChannel_instance.send(pset_actuator_control_target))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcontrol_system_state = CommunicationChannel.NEW.CONTROL_SYSTEM_STATE(cur);
                fill(pcontrol_system_state);
                if(!CommunicationChannel_instance.send(pcontrol_system_state))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdata32 = CommunicationChannel.NEW.DATA32(cur);
                fill(pdata32);
                if(!CommunicationChannel_instance.send(pdata32))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pping33 = CommunicationChannel.NEW.PING33(cur);
                fill(pping33);
                if(!CommunicationChannel_instance.send(pping33))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var prally_point = CommunicationChannel.NEW.RALLY_POINT(cur);
                fill(prally_point);
                if(!CommunicationChannel_instance.send(prally_point))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var padap_tuning = CommunicationChannel.NEW.ADAP_TUNING(cur);
                fill(padap_tuning);
                if(!CommunicationChannel_instance.send(padap_tuning))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pvibration = CommunicationChannel.NEW.VIBRATION(cur);
                fill(pvibration);
                if(!CommunicationChannel_instance.send(pvibration))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pparam_ext_value = CommunicationChannel.NEW.PARAM_EXT_VALUE(cur);
                fill(pparam_ext_value);
                if(!CommunicationChannel_instance.send(pparam_ext_value))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pbattery2 = CommunicationChannel.NEW.BATTERY2(cur);
                fill(pbattery2);
                if(!CommunicationChannel_instance.send(pbattery2))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plimits_status = CommunicationChannel.NEW.LIMITS_STATUS(cur);
                fill(plimits_status);
                if(!CommunicationChannel_instance.send(plimits_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcamera_feedback = CommunicationChannel.NEW.CAMERA_FEEDBACK(cur);
                fill(pcamera_feedback);
                if(!CommunicationChannel_instance.send(pcamera_feedback))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var phil_gps = CommunicationChannel.NEW.HIL_GPS(cur);
                fill(phil_gps);
                if(!CommunicationChannel_instance.send(phil_gps))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pfence_fetch_point = CommunicationChannel.NEW.FENCE_FETCH_POINT(cur);
                fill(pfence_fetch_point);
                if(!CommunicationChannel_instance.send(pfence_fetch_point))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pradio = CommunicationChannel.NEW.RADIO(cur);
                fill(pradio);
                if(!CommunicationChannel_instance.send(pradio))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pairspeed_autocal = CommunicationChannel.NEW.AIRSPEED_AUTOCAL(cur);
                fill(pairspeed_autocal);
                if(!CommunicationChannel_instance.send(pairspeed_autocal))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var patt_pos_mocap = CommunicationChannel.NEW.ATT_POS_MOCAP(cur);
                fill(patt_pos_mocap);
                if(!CommunicationChannel_instance.send(patt_pos_mocap))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pstatustext = CommunicationChannel.NEW.STATUSTEXT(cur);
                fill(pstatustext);
                if(!CommunicationChannel_instance.send(pstatustext))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgopro_get_request = CommunicationChannel.NEW.GOPRO_GET_REQUEST(cur);
                fill(pgopro_get_request);
                if(!CommunicationChannel_instance.send(pgopro_get_request))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcamera_capture_status = CommunicationChannel.NEW.CAMERA_CAPTURE_STATUS(cur);
                fill(pcamera_capture_status);
                if(!CommunicationChannel_instance.send(pcamera_capture_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pencapsulated_data = CommunicationChannel.NEW.ENCAPSULATED_DATA(cur);
                fill(pencapsulated_data);
                if(!CommunicationChannel_instance.send(pencapsulated_data))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgps_input = CommunicationChannel.NEW.GPS_INPUT(cur);
                fill(pgps_input);
                if(!CommunicationChannel_instance.send(pgps_input))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcompassmot_status = CommunicationChannel.NEW.COMPASSMOT_STATUS(cur);
                fill(pcompassmot_status);
                if(!CommunicationChannel_instance.send(pcompassmot_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plog_request_data = CommunicationChannel.NEW.LOG_REQUEST_DATA(cur);
                fill(plog_request_data);
                if(!CommunicationChannel_instance.send(plog_request_data))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcamera_status = CommunicationChannel.NEW.CAMERA_STATUS(cur);
                fill(pcamera_status);
                if(!CommunicationChannel_instance.send(pcamera_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcamera_settings = CommunicationChannel.NEW.CAMERA_SETTINGS(cur);
                fill(pcamera_settings);
                if(!CommunicationChannel_instance.send(pcamera_settings))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdevice_op_read_reply = CommunicationChannel.NEW.DEVICE_OP_READ_REPLY(cur);
                fill(pdevice_op_read_reply);
                if(!CommunicationChannel_instance.send(pdevice_op_read_reply))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdigicam_control = CommunicationChannel.NEW.DIGICAM_CONTROL(cur);
                fill(pdigicam_control);
                if(!CommunicationChannel_instance.send(pdigicam_control))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pnamed_value_float = CommunicationChannel.NEW.NAMED_VALUE_FLOAT(cur);
                fill(pnamed_value_float);
                if(!CommunicationChannel_instance.send(pnamed_value_float))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgopro_heartbeat = CommunicationChannel.NEW.GOPRO_HEARTBEAT(cur);
                fill(pgopro_heartbeat);
                if(!CommunicationChannel_instance.send(pgopro_heartbeat))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pahrs2 = CommunicationChannel.NEW.AHRS2(cur);
                fill(pahrs2);
                if(!CommunicationChannel_instance.send(pahrs2))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plog_erase = CommunicationChannel.NEW.LOG_ERASE(cur);
                fill(plog_erase);
                if(!CommunicationChannel_instance.send(plog_erase))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pterrain_request = CommunicationChannel.NEW.TERRAIN_REQUEST(cur);
                fill(pterrain_request);
                if(!CommunicationChannel_instance.send(pterrain_request))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmount_status = CommunicationChannel.NEW.MOUNT_STATUS(cur);
                fill(pmount_status);
                if(!CommunicationChannel_instance.send(pmount_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var ppid_tuning = CommunicationChannel.NEW.PID_TUNING(cur);
                fill(ppid_tuning);
                if(!CommunicationChannel_instance.send(ppid_tuning))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var poptical_flow_rad = CommunicationChannel.NEW.OPTICAL_FLOW_RAD(cur);
                fill(poptical_flow_rad);
                if(!CommunicationChannel_instance.send(poptical_flow_rad))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plog_data = CommunicationChannel.NEW.LOG_DATA(cur);
                fill(plog_data);
                if(!CommunicationChannel_instance.send(plog_data))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pahrs3 = CommunicationChannel.NEW.AHRS3(cur);
                fill(pahrs3);
                if(!CommunicationChannel_instance.send(pahrs3))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pvicon_position_estimate = CommunicationChannel.NEW.VICON_POSITION_ESTIMATE(cur);
                fill(pvicon_position_estimate);
                if(!CommunicationChannel_instance.send(pvicon_position_estimate))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgps2_rtk = CommunicationChannel.NEW.GPS2_RTK(cur);
                fill(pgps2_rtk);
                if(!CommunicationChannel_instance.send(pgps2_rtk))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmag_cal_report = CommunicationChannel.NEW.MAG_CAL_REPORT(cur);
                fill(pmag_cal_report);
                if(!CommunicationChannel_instance.send(pmag_cal_report))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plog_request_list = CommunicationChannel.NEW.LOG_REQUEST_LIST(cur);
                fill(plog_request_list);
                if(!CommunicationChannel_instance.send(plog_request_list))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmount_configure = CommunicationChannel.NEW.MOUNT_CONFIGURE(cur);
                fill(pmount_configure);
                if(!CommunicationChannel_instance.send(pmount_configure))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pv2_extension = CommunicationChannel.NEW.V2_EXTENSION(cur);
                fill(pv2_extension);
                if(!CommunicationChannel_instance.send(pv2_extension))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var ppower_status = CommunicationChannel.NEW.POWER_STATUS(cur);
                fill(ppower_status);
                if(!CommunicationChannel_instance.send(ppower_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var premote_log_data_block = CommunicationChannel.NEW.REMOTE_LOG_DATA_BLOCK(cur);
                fill(premote_log_data_block);
                if(!CommunicationChannel_instance.send(premote_log_data_block))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plogging_data_acked = CommunicationChannel.NEW.LOGGING_DATA_ACKED(cur);
                fill(plogging_data_acked);
                if(!CommunicationChannel_instance.send(plogging_data_acked))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pterrain_check = CommunicationChannel.NEW.TERRAIN_CHECK(cur);
                fill(pterrain_check);
                if(!CommunicationChannel_instance.send(pterrain_check))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pterrain_report = CommunicationChannel.NEW.TERRAIN_REPORT(cur);
                fill(pterrain_report);
                if(!CommunicationChannel_instance.send(pterrain_report))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pset_home_position = CommunicationChannel.NEW.SET_HOME_POSITION(cur);
                fill(pset_home_position);
                if(!CommunicationChannel_instance.send(pset_home_position))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            if(! CommunicationChannel_instance.sendSwitchModeCommand())
                throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            {
                var pscaled_imu3 = CommunicationChannel.NEW.SCALED_IMU3(cur);
                fill(pscaled_imu3);
                if(!CommunicationChannel_instance.send(pscaled_imu3))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmount_control = CommunicationChannel.NEW.MOUNT_CONTROL(cur);
                fill(pmount_control);
                if(!CommunicationChannel_instance.send(pmount_control))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pled_control = CommunicationChannel.NEW.LED_CONTROL(cur);
                fill(pled_control);
                if(!CommunicationChannel_instance.send(pled_control))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var psim_state = CommunicationChannel.NEW.SIM_STATE(cur);
                fill(psim_state);
                if(!CommunicationChannel_instance.send(psim_state))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pwifi_config_ap = CommunicationChannel.NEW.WIFI_CONFIG_AP(cur);
                fill(pwifi_config_ap);
                if(!CommunicationChannel_instance.send(pwifi_config_ap))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdata96 = CommunicationChannel.NEW.DATA96(cur);
                fill(pdata96);
                if(!CommunicationChannel_instance.send(pdata96))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pflight_information = CommunicationChannel.NEW.FLIGHT_INFORMATION(cur);
                fill(pflight_information);
                if(!CommunicationChannel_instance.send(pflight_information))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmeminfo = CommunicationChannel.NEW.MEMINFO(cur);
                fill(pmeminfo);
                if(!CommunicationChannel_instance.send(pmeminfo))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plogging_ack = CommunicationChannel.NEW.LOGGING_ACK(cur);
                fill(plogging_ack);
                if(!CommunicationChannel_instance.send(plogging_ack))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pvision_speed_estimate = CommunicationChannel.NEW.VISION_SPEED_ESTIMATE(cur);
                fill(pvision_speed_estimate);
                if(!CommunicationChannel_instance.send(pvision_speed_estimate))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdebug_vect = CommunicationChannel.NEW.DEBUG_VECT(cur);
                fill(pdebug_vect);
                if(!CommunicationChannel_instance.send(pdebug_vect))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pcamera_trigger = CommunicationChannel.NEW.CAMERA_TRIGGER(cur);
                fill(pcamera_trigger);
                if(!CommunicationChannel_instance.send(pcamera_trigger))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plog_request_end = CommunicationChannel.NEW.LOG_REQUEST_END(cur);
                fill(plog_request_end);
                if(!CommunicationChannel_instance.send(plog_request_end))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgopro_set_response = CommunicationChannel.NEW.GOPRO_SET_RESPONSE(cur);
                fill(pgopro_set_response);
                if(!CommunicationChannel_instance.send(pgopro_set_response))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pprotocol_version = CommunicationChannel.NEW.PROTOCOL_VERSION(cur);
                fill(pprotocol_version);
                if(!CommunicationChannel_instance.send(pprotocol_version))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var prally_fetch_point = CommunicationChannel.NEW.RALLY_FETCH_POINT(cur);
                fill(prally_fetch_point);
                if(!CommunicationChannel_instance.send(prally_fetch_point))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pbattery_status = CommunicationChannel.NEW.BATTERY_STATUS(cur);
                fill(pbattery_status);
                if(!CommunicationChannel_instance.send(pbattery_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmount_orientation = CommunicationChannel.NEW.MOUNT_ORIENTATION(cur);
                fill(pmount_orientation);
                if(!CommunicationChannel_instance.send(pmount_orientation))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pserial_control = CommunicationChannel.NEW.SERIAL_CONTROL(cur);
                fill(pserial_control);
                if(!CommunicationChannel_instance.send(pserial_control))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pparam_ext_set = CommunicationChannel.NEW.PARAM_EXT_SET(cur);
                fill(pparam_ext_set);
                if(!CommunicationChannel_instance.send(pparam_ext_set))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pautopilot_version = CommunicationChannel.NEW.AUTOPILOT_VERSION(cur);
                fill(pautopilot_version);
                if(!CommunicationChannel_instance.send(pautopilot_version))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var psimstate = CommunicationChannel.NEW.SIMSTATE(cur);
                fill(psimstate);
                if(!CommunicationChannel_instance.send(psimstate))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pset_video_stream_settings = CommunicationChannel.NEW.SET_VIDEO_STREAM_SETTINGS(cur);
                fill(pset_video_stream_settings);
                if(!CommunicationChannel_instance.send(pset_video_stream_settings))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pplay_tune = CommunicationChannel.NEW.PLAY_TUNE(cur);
                fill(pplay_tune);
                if(!CommunicationChannel_instance.send(pplay_tune))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdigicam_configure = CommunicationChannel.NEW.DIGICAM_CONFIGURE(cur);
                fill(pdigicam_configure);
                if(!CommunicationChannel_instance.send(pdigicam_configure))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pscaled_pressure3 = CommunicationChannel.NEW.SCALED_PRESSURE3(cur);
                fill(pscaled_pressure3);
                if(!CommunicationChannel_instance.send(pscaled_pressure3))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pparam_ext_ack = CommunicationChannel.NEW.PARAM_EXT_ACK(cur);
                fill(pparam_ext_ack);
                if(!CommunicationChannel_instance.send(pparam_ext_ack))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var puavcan_node_info = CommunicationChannel.NEW.UAVCAN_NODE_INFO(cur);
                fill(puavcan_node_info);
                if(!CommunicationChannel_instance.send(puavcan_node_info))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdata16 = CommunicationChannel.NEW.DATA16(cur);
                fill(pdata16);
                if(!CommunicationChannel_instance.send(pdata16))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pset_mag_offsets = CommunicationChannel.NEW.SET_MAG_OFFSETS(cur);
                fill(pset_mag_offsets);
                if(!CommunicationChannel_instance.send(pset_mag_offsets))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pscaled_imu2 = CommunicationChannel.NEW.SCALED_IMU2(cur);
                fill(pscaled_imu2);
                if(!CommunicationChannel_instance.send(pscaled_imu2))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pap_adc = CommunicationChannel.NEW.AP_ADC(cur);
                fill(pap_adc);
                if(!CommunicationChannel_instance.send(pap_adc))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pwind = CommunicationChannel.NEW.WIND(cur);
                fill(pwind);
                if(!CommunicationChannel_instance.send(pwind))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pautopilot_version_request = CommunicationChannel.NEW.AUTOPILOT_VERSION_REQUEST(cur);
                fill(pautopilot_version_request);
                if(!CommunicationChannel_instance.send(pautopilot_version_request))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdata_transmission_handshake = CommunicationChannel.NEW.DATA_TRANSMISSION_HANDSHAKE(cur);
                fill(pdata_transmission_handshake);
                if(!CommunicationChannel_instance.send(pdata_transmission_handshake))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdata64 = CommunicationChannel.NEW.DATA64(cur);
                fill(pdata64);
                if(!CommunicationChannel_instance.send(pdata64))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgimbal_report = CommunicationChannel.NEW.GIMBAL_REPORT(cur);
                fill(pgimbal_report);
                if(!CommunicationChannel_instance.send(pgimbal_report))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdevice_op_write = CommunicationChannel.NEW.DEVICE_OP_WRITE(cur);
                fill(pdevice_op_write);
                if(!CommunicationChannel_instance.send(pdevice_op_write))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdistance_sensor = CommunicationChannel.NEW.DISTANCE_SENSOR(cur);
                fill(pdistance_sensor);
                if(!CommunicationChannel_instance.send(pdistance_sensor))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var phil_optical_flow = CommunicationChannel.NEW.HIL_OPTICAL_FLOW(cur);
                fill(phil_optical_flow);
                if(!CommunicationChannel_instance.send(phil_optical_flow))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pscaled_pressure2 = CommunicationChannel.NEW.SCALED_PRESSURE2(cur);
                fill(pscaled_pressure2);
                if(!CommunicationChannel_instance.send(pscaled_pressure2))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pwind_cov = CommunicationChannel.NEW.WIND_COV(cur);
                fill(pwind_cov);
                if(!CommunicationChannel_instance.send(pwind_cov))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgopro_set_request = CommunicationChannel.NEW.GOPRO_SET_REQUEST(cur);
                fill(pgopro_set_request);
                if(!CommunicationChannel_instance.send(pgopro_set_request))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pvision_position_delta = CommunicationChannel.NEW.VISION_POSITION_DELTA(cur);
                fill(pvision_position_delta);
                if(!CommunicationChannel_instance.send(pvision_position_delta))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var plogging_data = CommunicationChannel.NEW.LOGGING_DATA(cur);
                fill(plogging_data);
                if(!CommunicationChannel_instance.send(plogging_data))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pdevice_op_read = CommunicationChannel.NEW.DEVICE_OP_READ(cur);
                fill(pdevice_op_read);
                if(!CommunicationChannel_instance.send(pdevice_op_read))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pmag_cal_progress = CommunicationChannel.NEW.MAG_CAL_PROGRESS(cur);
                fill(pmag_cal_progress);
                if(!CommunicationChannel_instance.send(pmag_cal_progress))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var phighres_imu = CommunicationChannel.NEW.HIGHRES_IMU(cur);
                fill(phighres_imu);
                if(!CommunicationChannel_instance.send(phighres_imu))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pextended_sys_state = CommunicationChannel.NEW.EXTENDED_SYS_STATE(cur);
                fill(pextended_sys_state);
                if(!CommunicationChannel_instance.send(pextended_sys_state))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var puavionix_adsb_out_dynamic = CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_DYNAMIC(cur);
                fill(puavionix_adsb_out_dynamic);
                if(!CommunicationChannel_instance.send(puavionix_adsb_out_dynamic))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgopro_get_response = CommunicationChannel.NEW.GOPRO_GET_RESPONSE(cur);
                fill(pgopro_get_response);
                if(!CommunicationChannel_instance.send(pgopro_get_response))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgps_inject_data = CommunicationChannel.NEW.GPS_INJECT_DATA(cur);
                fill(pgps_inject_data);
                if(!CommunicationChannel_instance.send(pgps_inject_data))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var puavionix_adsb_transceiver_health_report = CommunicationChannel.NEW.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur);
                fill(puavionix_adsb_transceiver_health_report);
                if(!CommunicationChannel_instance.send(puavionix_adsb_transceiver_health_report))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pnamed_value_int = CommunicationChannel.NEW.NAMED_VALUE_INT(cur);
                fill(pnamed_value_int);
                if(!CommunicationChannel_instance.send(pnamed_value_int))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var prpm = CommunicationChannel.NEW.RPM(cur);
                fill(prpm);
                if(!CommunicationChannel_instance.send(prpm))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pgps_rtcm_data = CommunicationChannel.NEW.GPS_RTCM_DATA(cur);
                fill(pgps_rtcm_data);
                if(!CommunicationChannel_instance.send(pgps_rtcm_data))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pfile_transfer_protocol = CommunicationChannel.NEW.FILE_TRANSFER_PROTOCOL(cur);
                fill(pfile_transfer_protocol);
                if(!CommunicationChannel_instance.send(pfile_transfer_protocol))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var prangefinder = CommunicationChannel.NEW.RANGEFINDER(cur);
                fill(prangefinder);
                if(!CommunicationChannel_instance.send(prangefinder))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pradio_status = CommunicationChannel.NEW.RADIO_STATUS(cur);
                fill(pradio_status);
                if(!CommunicationChannel_instance.send(pradio_status))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var pfence_point = CommunicationChannel.NEW.FENCE_POINT(cur);
                fill(pfence_point);
                if(!CommunicationChannel_instance.send(pfence_point))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            {
                var presource_request = CommunicationChannel.NEW.RESOURCE_REQUEST(cur);
                fill(presource_request);
                if(!CommunicationChannel_instance.send(presource_request))    //put pack into the channel send-buffer
                    throw new SystemException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
            }
            bytes_out = CommunicationChannel_instance.Read(buff, 0, buff.Length);// sending packs
            CommunicationChannel_instance.Write(buff, 0, buff.Length);// receiving packs
        }
    }
}
