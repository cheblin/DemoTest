import {org as _org} from "./AdHoc";
import _Lib = _org.unirail.AdHoc;
import _Pack = _Lib.Pack;
import _Cursor = _Pack.Cursor;

import {org as _ad_hoc} from "./GroundControl";
import _Config = _ad_hoc.unirail.AdHoc.Config_;

import {com as _gen} from "./GroundControl";
import _Host_root = _gen.company.demo;
import _Host = _gen.company.demo.GroundControl;


export namespace org.unirail {
	export namespace Demo_ {
		let some_string            = "null";
		let some_boolean           = true;
		let some_number            = 0;
		let some_numbers: number[] = [];
		
		type Handler<Channel, Pack> = (src: Channel, pack: Pack) => void;
		
		export function onATTITUDE_TARGET(pattitude_target: _Host.ATTITUDE_TARGET) {
			const some_time_boot_ms = pattitude_target.time_boot_ms();
			const some_type_mask    = pattitude_target.type_mask();
			{
				const item = pattitude_target.q();
				for (let value of item)
					some_number = value;
				
			}
			const some_body_roll_rate  = pattitude_target.body_roll_rate();
			const some_body_pitch_rate = pattitude_target.body_pitch_rate();
			const some_body_yaw_rate   = pattitude_target.body_yaw_rate();
			const some_thrust          = pattitude_target.thrust();
			
		}
		
		export function onMISSION_COUNT(pmission_count: _Host.MISSION_COUNT) {
			const some_target_system    = pmission_count.target_system();
			const some_target_component = pmission_count.target_component();
			const some_count            = pmission_count.count();
			{
				
				const item = pmission_count.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onADSB_VEHICLE(padsb_vehicle: _Host.ADSB_VEHICLE) {
			const some_ICAO_address = padsb_vehicle.ICAO_address();
			const some_lat          = padsb_vehicle.lat();
			const some_lon          = padsb_vehicle.lon();
			{
				
				const item = padsb_vehicle.altitude_type();
				if (item !== null)
					_Host.ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH = item;
				
			}
			const some_altitude     = padsb_vehicle.altitude();
			const some_heading      = padsb_vehicle.heading();
			const some_hor_velocity = padsb_vehicle.hor_velocity();
			const some_ver_velocity = padsb_vehicle.ver_velocity();
			{
				
				const item = padsb_vehicle.callsign();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = padsb_vehicle.emitter_type();
				if (item !== null)
					_Host.ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO = item;
				
			}
			const some_tslc = padsb_vehicle.tslc();
			{
				
				const item = padsb_vehicle.flags();
				if (item !== null)
					_Host.ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS = item;
				
			}
			const some_squawk = padsb_vehicle.squawk();
			
		}
		
		export function fill_ADSB_VEHICLE(padsb_vehicle: _Host.ADSB_VEHICLE) {
			padsb_vehicle.ICAO_address_(some_number);
			padsb_vehicle.lat_(some_number);
			padsb_vehicle.lon_(some_number);
			
			padsb_vehicle.altitude_type_(_Host.ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
			
			padsb_vehicle.altitude_(some_number);
			padsb_vehicle.heading_(some_number);
			padsb_vehicle.hor_velocity_(some_number);
			padsb_vehicle.ver_velocity_(some_number);
			padsb_vehicle.callsign_(some_string);
			
			padsb_vehicle.emitter_type_(_Host.ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
			
			padsb_vehicle.tslc_(some_number);
			
			padsb_vehicle.flags_(_Host.ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
			
			padsb_vehicle.squawk_(some_number);
			
		}
		
		export function onMESSAGE_INTERVAL(pmessage_interval: _Host.MESSAGE_INTERVAL) {
			const some_message_id  = pmessage_interval.message_id();
			const some_interval_us = pmessage_interval.interval_us();
			
		}
		
		export function fill_MESSAGE_INTERVAL(pmessage_interval: _Host.MESSAGE_INTERVAL) {
			pmessage_interval.message_id_(some_number);
			pmessage_interval.interval_us_(some_number);
			
		}
		
		export function onEKF_STATUS_REPORT(pekf_status_report: _Host.EKF_STATUS_REPORT) {
			{
				
				const item = pekf_status_report.flags();
				if (item !== null)
					_Host.EKF_STATUS_FLAGS.EKF_ATTITUDE = item;
				
			}
			const some_velocity_variance    = pekf_status_report.velocity_variance();
			const some_pos_horiz_variance   = pekf_status_report.pos_horiz_variance();
			const some_pos_vert_variance    = pekf_status_report.pos_vert_variance();
			const some_compass_variance     = pekf_status_report.compass_variance();
			const some_terrain_alt_variance = pekf_status_report.terrain_alt_variance();
			
		}
		
		export function fill_EKF_STATUS_REPORT(pekf_status_report: _Host.EKF_STATUS_REPORT) {
			
			pekf_status_report.flags_(_Host.EKF_STATUS_FLAGS.EKF_ATTITUDE);
			
			pekf_status_report.velocity_variance_(some_number);
			pekf_status_report.pos_horiz_variance_(some_number);
			pekf_status_report.pos_vert_variance_(some_number);
			pekf_status_report.compass_variance_(some_number);
			pekf_status_report.terrain_alt_variance_(some_number);
			
		}
		
		export function onESTIMATOR_STATUS(pestimator_status: _Host.ESTIMATOR_STATUS) {
			const some_time_usec = pestimator_status.time_usec();
			{
				
				const item = pestimator_status.flags();
				if (item !== null)
					_Host.ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE = item;
				
			}
			const some_vel_ratio          = pestimator_status.vel_ratio();
			const some_pos_horiz_ratio    = pestimator_status.pos_horiz_ratio();
			const some_pos_vert_ratio     = pestimator_status.pos_vert_ratio();
			const some_mag_ratio          = pestimator_status.mag_ratio();
			const some_hagl_ratio         = pestimator_status.hagl_ratio();
			const some_tas_ratio          = pestimator_status.tas_ratio();
			const some_pos_horiz_accuracy = pestimator_status.pos_horiz_accuracy();
			const some_pos_vert_accuracy  = pestimator_status.pos_vert_accuracy();
			
		}
		
		export function fill_ESTIMATOR_STATUS(pestimator_status: _Host.ESTIMATOR_STATUS) {
			pestimator_status.time_usec_(some_number);
			
			pestimator_status.flags_(_Host.ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
			
			pestimator_status.vel_ratio_(some_number);
			pestimator_status.pos_horiz_ratio_(some_number);
			pestimator_status.pos_vert_ratio_(some_number);
			pestimator_status.mag_ratio_(some_number);
			pestimator_status.hagl_ratio_(some_number);
			pestimator_status.tas_ratio_(some_number);
			pestimator_status.pos_horiz_accuracy_(some_number);
			pestimator_status.pos_vert_accuracy_(some_number);
			
		}
		
		export function onHWSTATUS(phwstatus: _Host.HWSTATUS) {
			const some_Vcc    = phwstatus.Vcc();
			const some_I2Cerr = phwstatus.I2Cerr();
			
		}
		
		export function fill_HWSTATUS(phwstatus: _Host.HWSTATUS) {
			phwstatus.Vcc_(some_number);
			phwstatus.I2Cerr_(some_number);
			
		}
		
		export function onTIMESYNC(ptimesync: _Host.TIMESYNC) {
			const some_tc1 = ptimesync.tc1();
			const some_ts1 = ptimesync.ts1();
			
		}
		
		export function fill_TIMESYNC(ptimesync: _Host.TIMESYNC) {
			ptimesync.tc1_(some_number);
			ptimesync.ts1_(some_number);
			
		}
		
		export function onPARAM_EXT_REQUEST_LIST(pparam_ext_request_list: _Host.PARAM_EXT_REQUEST_LIST) {
			const some_target_system    = pparam_ext_request_list.target_system();
			const some_target_component = pparam_ext_request_list.target_component();
			
		}
		
		export function fill_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list: _Host.PARAM_EXT_REQUEST_LIST) {
			pparam_ext_request_list.target_system_(some_number);
			pparam_ext_request_list.target_component_(some_number);
			
		}
		
		export function onGLOBAL_POSITION_INT_COV(pglobal_position_int_cov: _Host.GLOBAL_POSITION_INT_COV) {
			const some_time_usec = pglobal_position_int_cov.time_usec();
			{
				
				const item = pglobal_position_int_cov.estimator_type();
				if (item !== null)
					_Host.MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE = item;
				
			}
			const some_lat          = pglobal_position_int_cov.lat();
			const some_lon          = pglobal_position_int_cov.lon();
			const some_alt          = pglobal_position_int_cov.alt();
			const some_relative_alt = pglobal_position_int_cov.relative_alt();
			const some_vx           = pglobal_position_int_cov.vx();
			const some_vy           = pglobal_position_int_cov.vy();
			const some_vz           = pglobal_position_int_cov.vz();
			{
				const item = pglobal_position_int_cov.covariance();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function onBUTTON_CHANGE(pbutton_change: _Host.BUTTON_CHANGE) {
			const some_time_boot_ms   = pbutton_change.time_boot_ms();
			const some_last_change_ms = pbutton_change.last_change_ms();
			const some_state          = pbutton_change.state();
			
		}
		
		export function fill_BUTTON_CHANGE(pbutton_change: _Host.BUTTON_CHANGE) {
			pbutton_change.time_boot_ms_(some_number);
			pbutton_change.last_change_ms_(some_number);
			pbutton_change.state_(some_number);
			
		}
		
		export function onSAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area: _Host.SAFETY_SET_ALLOWED_AREA) {
			const some_target_system    = psafety_set_allowed_area.target_system();
			const some_target_component = psafety_set_allowed_area.target_component();
			{
				
				const item = psafety_set_allowed_area.frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			const some_p1x = psafety_set_allowed_area.p1x();
			const some_p1y = psafety_set_allowed_area.p1y();
			const some_p1z = psafety_set_allowed_area.p1z();
			const some_p2x = psafety_set_allowed_area.p2x();
			const some_p2y = psafety_set_allowed_area.p2y();
			const some_p2z = psafety_set_allowed_area.p2z();
			
		}
		
		export function onUAVCAN_NODE_STATUS(puavcan_node_status: _Host.UAVCAN_NODE_STATUS) {
			const some_time_usec  = puavcan_node_status.time_usec();
			const some_uptime_sec = puavcan_node_status.uptime_sec();
			{
				
				const item = puavcan_node_status.health();
				if (item !== null)
					_Host.UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK = item;
				
			}
			{
				
				const item = puavcan_node_status.mode();
				if (item !== null)
					_Host.UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL = item;
				
			}
			const some_sub_mode                    = puavcan_node_status.sub_mode();
			const some_vendor_specific_status_code = puavcan_node_status.vendor_specific_status_code();
			
		}
		
		export function fill_UAVCAN_NODE_STATUS(puavcan_node_status: _Host.UAVCAN_NODE_STATUS) {
			puavcan_node_status.time_usec_(some_number);
			puavcan_node_status.uptime_sec_(some_number);
			
			puavcan_node_status.health_(_Host.UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
			
			
			puavcan_node_status.mode_(_Host.UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
			
			puavcan_node_status.sub_mode_(some_number);
			puavcan_node_status.vendor_specific_status_code_(some_number);
			
		}
		
		export function onCOLLISION(pcollision: _Host.COLLISION) {
			{
				
				const item = pcollision.sRc();
				if (item !== null)
					_Host.MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB = item;
				
			}
			const some_id = pcollision.id();
			{
				
				const item = pcollision.action();
				if (item !== null)
					_Host.MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE = item;
				
			}
			{
				
				const item = pcollision.threat_level();
				if (item !== null)
					_Host.MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE = item;
				
			}
			const some_time_to_minimum_delta    = pcollision.time_to_minimum_delta();
			const some_altitude_minimum_delta   = pcollision.altitude_minimum_delta();
			const some_horizontal_minimum_delta = pcollision.horizontal_minimum_delta();
			
		}
		
		export function fill_COLLISION(pcollision: _Host.COLLISION) {
			
			pcollision.sRc_(_Host.MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
			
			pcollision.id_(some_number);
			
			pcollision.action_(_Host.MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
			
			
			pcollision.threat_level_(_Host.MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
			
			pcollision.time_to_minimum_delta_(some_number);
			pcollision.altitude_minimum_delta_(some_number);
			pcollision.horizontal_minimum_delta_(some_number);
			
		}
		
		export function onGIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report: _Host.GIMBAL_TORQUE_CMD_REPORT) {
			const some_target_system    = pgimbal_torque_cmd_report.target_system();
			const some_target_component = pgimbal_torque_cmd_report.target_component();
			const some_rl_torque_cmd    = pgimbal_torque_cmd_report.rl_torque_cmd();
			const some_el_torque_cmd    = pgimbal_torque_cmd_report.el_torque_cmd();
			const some_az_torque_cmd    = pgimbal_torque_cmd_report.az_torque_cmd();
			
		}
		
		export function fill_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report: _Host.GIMBAL_TORQUE_CMD_REPORT) {
			pgimbal_torque_cmd_report.target_system_(some_number);
			pgimbal_torque_cmd_report.target_component_(some_number);
			pgimbal_torque_cmd_report.rl_torque_cmd_(some_number);
			pgimbal_torque_cmd_report.el_torque_cmd_(some_number);
			pgimbal_torque_cmd_report.az_torque_cmd_(some_number);
			
		}
		
		export function onALTITUDE(paltitude: _Host.ALTITUDE) {
			const some_time_usec          = paltitude.time_usec();
			const some_altitude_monotonic = paltitude.altitude_monotonic();
			const some_altitude_amsl      = paltitude.altitude_amsl();
			const some_altitude_local     = paltitude.altitude_local();
			const some_altitude_relative  = paltitude.altitude_relative();
			const some_altitude_terrain   = paltitude.altitude_terrain();
			const some_bottom_clearance   = paltitude.bottom_clearance();
			
		}
		
		export function fill_ALTITUDE(paltitude: _Host.ALTITUDE) {
			paltitude.time_usec_(some_number);
			paltitude.altitude_monotonic_(some_number);
			paltitude.altitude_amsl_(some_number);
			paltitude.altitude_local_(some_number);
			paltitude.altitude_relative_(some_number);
			paltitude.altitude_terrain_(some_number);
			paltitude.bottom_clearance_(some_number);
			
		}
		
		export function onHIL_STATE_QUATERNION(phil_state_quaternion: _Host.HIL_STATE_QUATERNION) {
			const some_time_usec = phil_state_quaternion.time_usec();
			{
				const item = phil_state_quaternion.attitude_quaternion();
				for (let value of item)
					some_number = value;
				
			}
			const some_rollspeed     = phil_state_quaternion.rollspeed();
			const some_pitchspeed    = phil_state_quaternion.pitchspeed();
			const some_yawspeed      = phil_state_quaternion.yawspeed();
			const some_lat           = phil_state_quaternion.lat();
			const some_lon           = phil_state_quaternion.lon();
			const some_alt           = phil_state_quaternion.alt();
			const some_vx            = phil_state_quaternion.vx();
			const some_vy            = phil_state_quaternion.vy();
			const some_vz            = phil_state_quaternion.vz();
			const some_ind_airspeed  = phil_state_quaternion.ind_airspeed();
			const some_true_airspeed = phil_state_quaternion.true_airspeed();
			const some_xacc          = phil_state_quaternion.xacc();
			const some_yacc          = phil_state_quaternion.yacc();
			const some_zacc          = phil_state_quaternion.zacc();
			
		}
		
		export function fill_HIL_STATE_QUATERNION(phil_state_quaternion: _Host.HIL_STATE_QUATERNION) {
			phil_state_quaternion.time_usec_(some_number);
			{
				const item = phil_state_quaternion.attitude_quaternion();
				
				for (let i = 0; i < _Host.HIL_STATE_QUATERNION.attitude_quaternion.item_len; i++)
					item.set(some_number, i);
				
			}
			phil_state_quaternion.rollspeed_(some_number);
			phil_state_quaternion.pitchspeed_(some_number);
			phil_state_quaternion.yawspeed_(some_number);
			phil_state_quaternion.lat_(some_number);
			phil_state_quaternion.lon_(some_number);
			phil_state_quaternion.alt_(some_number);
			phil_state_quaternion.vx_(some_number);
			phil_state_quaternion.vy_(some_number);
			phil_state_quaternion.vz_(some_number);
			phil_state_quaternion.ind_airspeed_(some_number);
			phil_state_quaternion.true_airspeed_(some_number);
			phil_state_quaternion.xacc_(some_number);
			phil_state_quaternion.yacc_(some_number);
			phil_state_quaternion.zacc_(some_number);
			
		}
		
		export function onSENSOR_OFFSETS(psensor_offsets: _Host.SENSOR_OFFSETS) {
			const some_mag_ofs_x       = psensor_offsets.mag_ofs_x();
			const some_mag_ofs_y       = psensor_offsets.mag_ofs_y();
			const some_mag_ofs_z       = psensor_offsets.mag_ofs_z();
			const some_mag_declination = psensor_offsets.mag_declination();
			const some_raw_press       = psensor_offsets.raw_press();
			const some_raw_temp        = psensor_offsets.raw_temp();
			const some_gyro_cal_x      = psensor_offsets.gyro_cal_x();
			const some_gyro_cal_y      = psensor_offsets.gyro_cal_y();
			const some_gyro_cal_z      = psensor_offsets.gyro_cal_z();
			const some_accel_cal_x     = psensor_offsets.accel_cal_x();
			const some_accel_cal_y     = psensor_offsets.accel_cal_y();
			const some_accel_cal_z     = psensor_offsets.accel_cal_z();
			
		}
		
		export function fill_SENSOR_OFFSETS(psensor_offsets: _Host.SENSOR_OFFSETS) {
			psensor_offsets.mag_ofs_x_(some_number);
			psensor_offsets.mag_ofs_y_(some_number);
			psensor_offsets.mag_ofs_z_(some_number);
			psensor_offsets.mag_declination_(some_number);
			psensor_offsets.raw_press_(some_number);
			psensor_offsets.raw_temp_(some_number);
			psensor_offsets.gyro_cal_x_(some_number);
			psensor_offsets.gyro_cal_y_(some_number);
			psensor_offsets.gyro_cal_z_(some_number);
			psensor_offsets.accel_cal_x_(some_number);
			psensor_offsets.accel_cal_y_(some_number);
			psensor_offsets.accel_cal_z_(some_number);
			
		}
		
		export function onSTORAGE_INFORMATION(pstorage_information: _Host.STORAGE_INFORMATION) {
			const some_time_boot_ms       = pstorage_information.time_boot_ms();
			const some_storage_id         = pstorage_information.storage_id();
			const some_storage_count      = pstorage_information.storage_count();
			const some_status             = pstorage_information.status();
			const some_total_capacity     = pstorage_information.total_capacity();
			const some_used_capacity      = pstorage_information.used_capacity();
			const some_available_capacity = pstorage_information.available_capacity();
			const some_read_speed         = pstorage_information.read_speed();
			const some_write_speed        = pstorage_information.write_speed();
			
		}
		
		export function fill_STORAGE_INFORMATION(pstorage_information: _Host.STORAGE_INFORMATION) {
			pstorage_information.time_boot_ms_(some_number);
			pstorage_information.storage_id_(some_number);
			pstorage_information.storage_count_(some_number);
			pstorage_information.status_(some_number);
			pstorage_information.total_capacity_(some_number);
			pstorage_information.used_capacity_(some_number);
			pstorage_information.available_capacity_(some_number);
			pstorage_information.read_speed_(some_number);
			pstorage_information.write_speed_(some_number);
			
		}
		
		export function onCAMERA_INFORMATION(pcamera_information: _Host.CAMERA_INFORMATION) {
			const some_time_boot_ms = pcamera_information.time_boot_ms();
			{
				const item = pcamera_information.vendor_name();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pcamera_information.model_name();
				for (let value of item)
					some_number = value;
				
			}
			const some_firmware_version = pcamera_information.firmware_version();
			const some_focal_length     = pcamera_information.focal_length();
			const some_sensor_size_h    = pcamera_information.sensor_size_h();
			const some_sensor_size_v    = pcamera_information.sensor_size_v();
			const some_resolution_h     = pcamera_information.resolution_h();
			const some_resolution_v     = pcamera_information.resolution_v();
			const some_lens_id          = pcamera_information.lens_id();
			{
				
				const item = pcamera_information.flags();
				if (item !== null)
					_Host.CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO = item;
				
			}
			const some_cam_definition_version = pcamera_information.cam_definition_version();
			{
				
				const item = pcamera_information.cam_definition_uri();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function fill_CAMERA_INFORMATION(pcamera_information: _Host.CAMERA_INFORMATION) {
			pcamera_information.time_boot_ms_(some_number);
			{
				const item = pcamera_information.vendor_name();
				
				for (let i = 0; i < _Host.CAMERA_INFORMATION.vendor_name.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pcamera_information.model_name();
				
				for (let i = 0; i < _Host.CAMERA_INFORMATION.model_name.item_len; i++)
					item.set(some_number, i);
				
			}
			pcamera_information.firmware_version_(some_number);
			pcamera_information.focal_length_(some_number);
			pcamera_information.sensor_size_h_(some_number);
			pcamera_information.sensor_size_v_(some_number);
			pcamera_information.resolution_h_(some_number);
			pcamera_information.resolution_v_(some_number);
			pcamera_information.lens_id_(some_number);
			
			pcamera_information.flags_(_Host.CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
			
			pcamera_information.cam_definition_version_(some_number);
			pcamera_information.cam_definition_uri_(some_string);
			
		}
		
		export function onGPS_STATUS(pgps_status: _Host.GPS_STATUS) {
			const some_satellites_visible = pgps_status.satellites_visible();
			{
				const item = pgps_status.satellite_prn();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pgps_status.satellite_used();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pgps_status.satellite_elevation();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pgps_status.satellite_azimuth();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pgps_status.satellite_snr();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function onDEVICE_OP_WRITE_REPLY(pdevice_op_write_reply: _Host.DEVICE_OP_WRITE_REPLY) {
			const some_request_id = pdevice_op_write_reply.request_id();
			const some_result     = pdevice_op_write_reply.result();
			
		}
		
		export function fill_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply: _Host.DEVICE_OP_WRITE_REPLY) {
			pdevice_op_write_reply.request_id_(some_number);
			pdevice_op_write_reply.result_(some_number);
			
		}
		
		export function onPARAM_SET(pparam_set: _Host.PARAM_SET) {
			const some_target_system    = pparam_set.target_system();
			const some_target_component = pparam_set.target_component();
			{
				
				const item = pparam_set.param_id();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_param_value = pparam_set.param_value();
			{
				
				const item = pparam_set.param_type();
				if (item !== null)
					_Host.MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8 = item;
				
			}
			
		}
		
		export function onTERRAIN_DATA(pterrain_data: _Host.TERRAIN_DATA) {
			const some_lat          = pterrain_data.lat();
			const some_lon          = pterrain_data.lon();
			const some_grid_spacing = pterrain_data.grid_spacing();
			const some_gridbit      = pterrain_data.gridbit();
			{
				const item = pterrain_data.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_TERRAIN_DATA(pterrain_data: _Host.TERRAIN_DATA) {
			pterrain_data.lat_(some_number);
			pterrain_data.lon_(some_number);
			pterrain_data.grid_spacing_(some_number);
			pterrain_data.gridbit_(some_number);
			{
				const item = pterrain_data.daTa();
				
				for (let i = 0; i < _Host.TERRAIN_DATA.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onGIMBAL_CONTROL(pgimbal_control: _Host.GIMBAL_CONTROL) {
			const some_target_system    = pgimbal_control.target_system();
			const some_target_component = pgimbal_control.target_component();
			const some_demanded_rate_x  = pgimbal_control.demanded_rate_x();
			const some_demanded_rate_y  = pgimbal_control.demanded_rate_y();
			const some_demanded_rate_z  = pgimbal_control.demanded_rate_z();
			
		}
		
		export function fill_GIMBAL_CONTROL(pgimbal_control: _Host.GIMBAL_CONTROL) {
			pgimbal_control.target_system_(some_number);
			pgimbal_control.target_component_(some_number);
			pgimbal_control.demanded_rate_x_(some_number);
			pgimbal_control.demanded_rate_y_(some_number);
			pgimbal_control.demanded_rate_z_(some_number);
			
		}
		
		export function onRC_CHANNELS_OVERRIDE(prc_channels_override: _Host.RC_CHANNELS_OVERRIDE) {
			const some_target_system    = prc_channels_override.target_system();
			const some_target_component = prc_channels_override.target_component();
			const some_chan1_raw        = prc_channels_override.chan1_raw();
			const some_chan2_raw        = prc_channels_override.chan2_raw();
			const some_chan3_raw        = prc_channels_override.chan3_raw();
			const some_chan4_raw        = prc_channels_override.chan4_raw();
			const some_chan5_raw        = prc_channels_override.chan5_raw();
			const some_chan6_raw        = prc_channels_override.chan6_raw();
			const some_chan7_raw        = prc_channels_override.chan7_raw();
			const some_chan8_raw        = prc_channels_override.chan8_raw();
			
		}
		
		export function onSCALED_IMU(pscaled_imu: _Host.SCALED_IMU) {
			const some_time_boot_ms = pscaled_imu.time_boot_ms();
			const some_xacc         = pscaled_imu.xacc();
			const some_yacc         = pscaled_imu.yacc();
			const some_zacc         = pscaled_imu.zacc();
			const some_xgyro        = pscaled_imu.xgyro();
			const some_ygyro        = pscaled_imu.ygyro();
			const some_zgyro        = pscaled_imu.zgyro();
			const some_xmag         = pscaled_imu.xmag();
			const some_ymag         = pscaled_imu.ymag();
			const some_zmag         = pscaled_imu.zmag();
			
		}
		
		export function onVIDEO_STREAM_INFORMATION(pvideo_stream_information: _Host.VIDEO_STREAM_INFORMATION) {
			const some_camera_id    = pvideo_stream_information.camera_id();
			const some_status       = pvideo_stream_information.status();
			const some_framerate    = pvideo_stream_information.framerate();
			const some_resolution_h = pvideo_stream_information.resolution_h();
			const some_resolution_v = pvideo_stream_information.resolution_v();
			const some_bitrate      = pvideo_stream_information.bitrate();
			const some_rotation     = pvideo_stream_information.rotation();
			{
				
				const item = pvideo_stream_information.uri();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function fill_VIDEO_STREAM_INFORMATION(pvideo_stream_information: _Host.VIDEO_STREAM_INFORMATION) {
			pvideo_stream_information.camera_id_(some_number);
			pvideo_stream_information.status_(some_number);
			pvideo_stream_information.framerate_(some_number);
			pvideo_stream_information.resolution_h_(some_number);
			pvideo_stream_information.resolution_v_(some_number);
			pvideo_stream_information.bitrate_(some_number);
			pvideo_stream_information.rotation_(some_number);
			pvideo_stream_information.uri_(some_string);
			
		}
		
		export function onAHRS(pahrs: _Host.AHRS) {
			const some_omegaIx      = pahrs.omegaIx();
			const some_omegaIy      = pahrs.omegaIy();
			const some_omegaIz      = pahrs.omegaIz();
			const some_accel_weight = pahrs.accel_weight();
			const some_renorm_val   = pahrs.renorm_val();
			const some_error_rp     = pahrs.error_rp();
			const some_error_yaw    = pahrs.error_yaw();
			
		}
		
		export function fill_AHRS(pahrs: _Host.AHRS) {
			pahrs.omegaIx_(some_number);
			pahrs.omegaIy_(some_number);
			pahrs.omegaIz_(some_number);
			pahrs.accel_weight_(some_number);
			pahrs.renorm_val_(some_number);
			pahrs.error_rp_(some_number);
			pahrs.error_yaw_(some_number);
			
		}
		
		export function onDEBUG(pdebug: _Host.DEBUG) {
			const some_time_boot_ms = pdebug.time_boot_ms();
			const some_ind          = pdebug.ind();
			const some_value        = pdebug.value();
			
		}
		
		export function fill_DEBUG(pdebug: _Host.DEBUG) {
			pdebug.time_boot_ms_(some_number);
			pdebug.ind_(some_number);
			pdebug.value_(some_number);
			
		}
		
		export function onCAMERA_IMAGE_CAPTURED(pcamera_image_captured: _Host.CAMERA_IMAGE_CAPTURED) {
			const some_time_boot_ms = pcamera_image_captured.time_boot_ms();
			const some_time_utc     = pcamera_image_captured.time_utc();
			const some_camera_id    = pcamera_image_captured.camera_id();
			const some_lat          = pcamera_image_captured.lat();
			const some_lon          = pcamera_image_captured.lon();
			const some_alt          = pcamera_image_captured.alt();
			const some_relative_alt = pcamera_image_captured.relative_alt();
			{
				const item = pcamera_image_captured.q();
				for (let value of item)
					some_number = value;
				
			}
			const some_image_index    = pcamera_image_captured.image_index();
			const some_capture_result = pcamera_image_captured.capture_result();
			{
				
				const item = pcamera_image_captured.file_url();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function fill_CAMERA_IMAGE_CAPTURED(pcamera_image_captured: _Host.CAMERA_IMAGE_CAPTURED) {
			pcamera_image_captured.time_boot_ms_(some_number);
			pcamera_image_captured.time_utc_(some_number);
			pcamera_image_captured.camera_id_(some_number);
			pcamera_image_captured.lat_(some_number);
			pcamera_image_captured.lon_(some_number);
			pcamera_image_captured.alt_(some_number);
			pcamera_image_captured.relative_alt_(some_number);
			{
				const item = pcamera_image_captured.q();
				
				for (let i = 0; i < _Host.CAMERA_IMAGE_CAPTURED.q.item_len; i++)
					item.set(some_number, i);
				
			}
			pcamera_image_captured.image_index_(some_number);
			pcamera_image_captured.capture_result_(some_number);
			pcamera_image_captured.file_url_(some_string);
			
		}
		
		export function onLOG_ENTRY(plog_entry: _Host.LOG_ENTRY) {
			const some_id           = plog_entry.id();
			const some_num_logs     = plog_entry.num_logs();
			const some_last_log_num = plog_entry.last_log_num();
			const some_time_utc     = plog_entry.time_utc();
			const some_size         = plog_entry.size();
			
		}
		
		export function fill_LOG_ENTRY(plog_entry: _Host.LOG_ENTRY) {
			plog_entry.id_(some_number);
			plog_entry.num_logs_(some_number);
			plog_entry.last_log_num_(some_number);
			plog_entry.time_utc_(some_number);
			plog_entry.size_(some_number);
			
		}
		
		export function onACTUATOR_CONTROL_TARGET(pactuator_control_target: _Host.ACTUATOR_CONTROL_TARGET) {
			const some_time_usec = pactuator_control_target.time_usec();
			const some_group_mlx = pactuator_control_target.group_mlx();
			{
				const item = pactuator_control_target.controls();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_ACTUATOR_CONTROL_TARGET(pactuator_control_target: _Host.ACTUATOR_CONTROL_TARGET) {
			pactuator_control_target.time_usec_(some_number);
			pactuator_control_target.group_mlx_(some_number);
			{
				const item = pactuator_control_target.controls();
				
				for (let i = 0; i < _Host.ACTUATOR_CONTROL_TARGET.controls.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onHIGH_LATENCY(phigh_latency: _Host.HIGH_LATENCY) {
			{
				
				const item = phigh_latency.base_mode();
				if (item !== null)
					_Host.MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = item;
				
			}
			const some_custom_mode = phigh_latency.custom_mode();
			{
				
				const item = phigh_latency.landed_state();
				if (item !== null)
					_Host.MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED = item;
				
			}
			const some_roll          = phigh_latency.roll();
			const some_pitch         = phigh_latency.pitch();
			const some_heading       = phigh_latency.heading();
			const some_throttle      = phigh_latency.throttle();
			const some_heading_sp    = phigh_latency.heading_sp();
			const some_latitude      = phigh_latency.latitude();
			const some_longitude     = phigh_latency.longitude();
			const some_altitude_amsl = phigh_latency.altitude_amsl();
			const some_altitude_sp   = phigh_latency.altitude_sp();
			const some_airspeed      = phigh_latency.airspeed();
			const some_airspeed_sp   = phigh_latency.airspeed_sp();
			const some_groundspeed   = phigh_latency.groundspeed();
			const some_climb_rate    = phigh_latency.climb_rate();
			const some_gps_nsat      = phigh_latency.gps_nsat();
			{
				
				const item = phigh_latency.gps_fix_type();
				if (item !== null)
					_Host.GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS = item;
				
			}
			const some_battery_remaining = phigh_latency.battery_remaining();
			const some_temperature       = phigh_latency.temperature();
			const some_temperature_air   = phigh_latency.temperature_air();
			const some_failsafe          = phigh_latency.failsafe();
			const some_wp_num            = phigh_latency.wp_num();
			const some_wp_distance       = phigh_latency.wp_distance();
			
		}
		
		export function fill_HIGH_LATENCY(phigh_latency: _Host.HIGH_LATENCY) {
			
			phigh_latency.base_mode_(_Host.MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
			
			phigh_latency.custom_mode_(some_number);
			
			phigh_latency.landed_state_(_Host.MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
			
			phigh_latency.roll_(some_number);
			phigh_latency.pitch_(some_number);
			phigh_latency.heading_(some_number);
			phigh_latency.throttle_(some_number);
			phigh_latency.heading_sp_(some_number);
			phigh_latency.latitude_(some_number);
			phigh_latency.longitude_(some_number);
			phigh_latency.altitude_amsl_(some_number);
			phigh_latency.altitude_sp_(some_number);
			phigh_latency.airspeed_(some_number);
			phigh_latency.airspeed_sp_(some_number);
			phigh_latency.groundspeed_(some_number);
			phigh_latency.climb_rate_(some_number);
			phigh_latency.gps_nsat_(some_number);
			
			phigh_latency.gps_fix_type_(_Host.GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
			
			phigh_latency.battery_remaining_(some_number);
			phigh_latency.temperature_(some_number);
			phigh_latency.temperature_air_(some_number);
			phigh_latency.failsafe_(some_number);
			phigh_latency.wp_num_(some_number);
			phigh_latency.wp_distance_(some_number);
			
		}
		
		export function onPARAM_REQUEST_READ(pparam_request_read: _Host.PARAM_REQUEST_READ) {
			const some_target_system    = pparam_request_read.target_system();
			const some_target_component = pparam_request_read.target_component();
			{
				
				const item = pparam_request_read.param_id();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_param_index = pparam_request_read.param_index();
			
		}
		
		export function onSET_ATTITUDE_TARGET(pset_attitude_target: _Host.SET_ATTITUDE_TARGET) {
			const some_time_boot_ms     = pset_attitude_target.time_boot_ms();
			const some_target_system    = pset_attitude_target.target_system();
			const some_target_component = pset_attitude_target.target_component();
			const some_type_mask        = pset_attitude_target.type_mask();
			{
				const item = pset_attitude_target.q();
				for (let value of item)
					some_number = value;
				
			}
			const some_body_roll_rate  = pset_attitude_target.body_roll_rate();
			const some_body_pitch_rate = pset_attitude_target.body_pitch_rate();
			const some_body_yaw_rate   = pset_attitude_target.body_yaw_rate();
			const some_thrust          = pset_attitude_target.thrust();
			
		}
		
		export function onFOLLOW_TARGET(pfollow_target: _Host.FOLLOW_TARGET) {
			const some_timestamp        = pfollow_target.timestamp();
			const some_est_capabilities = pfollow_target.est_capabilities();
			const some_lat              = pfollow_target.lat();
			const some_lon              = pfollow_target.lon();
			const some_alt              = pfollow_target.alt();
			{
				const item = pfollow_target.vel();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pfollow_target.acc();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pfollow_target.attitude_q();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pfollow_target.rates();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pfollow_target.position_cov();
				for (let value of item)
					some_number = value;
				
			}
			const some_custom_state = pfollow_target.custom_state();
			
		}
		
		export function fill_FOLLOW_TARGET(pfollow_target: _Host.FOLLOW_TARGET) {
			pfollow_target.timestamp_(some_number);
			pfollow_target.est_capabilities_(some_number);
			pfollow_target.lat_(some_number);
			pfollow_target.lon_(some_number);
			pfollow_target.alt_(some_number);
			{
				const item = pfollow_target.vel();
				
				for (let i = 0; i < _Host.FOLLOW_TARGET.vel.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pfollow_target.acc();
				
				for (let i = 0; i < _Host.FOLLOW_TARGET.acc.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pfollow_target.attitude_q();
				
				for (let i = 0; i < _Host.FOLLOW_TARGET.attitude_q.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pfollow_target.rates();
				
				for (let i = 0; i < _Host.FOLLOW_TARGET.rates.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pfollow_target.position_cov();
				
				for (let i = 0; i < _Host.FOLLOW_TARGET.position_cov.item_len; i++)
					item.set(some_number, i);
				
			}
			pfollow_target.custom_state_(some_number);
			
		}
		
		export function onHIL_STATE(phil_state: _Host.HIL_STATE) {
			const some_time_usec  = phil_state.time_usec();
			const some_roll       = phil_state.roll();
			const some_pitch      = phil_state.pitch();
			const some_yaw        = phil_state.yaw();
			const some_rollspeed  = phil_state.rollspeed();
			const some_pitchspeed = phil_state.pitchspeed();
			const some_yawspeed   = phil_state.yawspeed();
			const some_lat        = phil_state.lat();
			const some_lon        = phil_state.lon();
			const some_alt        = phil_state.alt();
			const some_vx         = phil_state.vx();
			const some_vy         = phil_state.vy();
			const some_vz         = phil_state.vz();
			const some_xacc       = phil_state.xacc();
			const some_yacc       = phil_state.yacc();
			const some_zacc       = phil_state.zacc();
			
		}
		
		export function onHOME_POSITION(phome_position: _Host.HOME_POSITION) {
			const some_latitude  = phome_position.latitude();
			const some_longitude = phome_position.longitude();
			const some_altitude  = phome_position.altitude();
			const some_x         = phome_position.x();
			const some_y         = phome_position.y();
			const some_z         = phome_position.z();
			{
				const item = phome_position.q();
				for (let value of item)
					some_number = value;
				
			}
			const some_approach_x = phome_position.approach_x();
			const some_approach_y = phome_position.approach_y();
			const some_approach_z = phome_position.approach_z();
			{
				
				const item = phome_position.time_usec();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function fill_HOME_POSITION(phome_position: _Host.HOME_POSITION) {
			phome_position.latitude_(some_number);
			phome_position.longitude_(some_number);
			phome_position.altitude_(some_number);
			phome_position.x_(some_number);
			phome_position.y_(some_number);
			phome_position.z_(some_number);
			{
				const item = phome_position.q();
				
				for (let i = 0; i < _Host.HOME_POSITION.q.item_len; i++)
					item.set(some_number, i);
				
			}
			phome_position.approach_x_(some_number);
			phome_position.approach_y_(some_number);
			phome_position.approach_z_(some_number);
			
			phome_position.time_usec_(some_number);
			
			
		}
		
		export function onFENCE_STATUS(pfence_status: _Host.FENCE_STATUS) {
			const some_breach_status = pfence_status.breach_status();
			const some_breach_count  = pfence_status.breach_count();
			{
				
				const item = pfence_status.breach_type();
				if (item !== null)
					_Host.FENCE_BREACH.FENCE_BREACH_NONE = item;
				
			}
			const some_breach_time = pfence_status.breach_time();
			
		}
		
		export function fill_FENCE_STATUS(pfence_status: _Host.FENCE_STATUS) {
			pfence_status.breach_status_(some_number);
			pfence_status.breach_count_(some_number);
			
			pfence_status.breach_type_(_Host.FENCE_BREACH.FENCE_BREACH_NONE);
			
			pfence_status.breach_time_(some_number);
			
		}
		
		export function onREMOTE_LOG_BLOCK_STATUS(premote_log_block_status: _Host.REMOTE_LOG_BLOCK_STATUS) {
			const some_target_system    = premote_log_block_status.target_system();
			const some_target_component = premote_log_block_status.target_component();
			const some_seqno            = premote_log_block_status.seqno();
			{
				
				const item = premote_log_block_status.status();
				if (item !== null)
					_Host.MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK = item;
				
			}
			
		}
		
		export function fill_REMOTE_LOG_BLOCK_STATUS(premote_log_block_status: _Host.REMOTE_LOG_BLOCK_STATUS) {
			premote_log_block_status.target_system_(some_number);
			premote_log_block_status.target_component_(some_number);
			premote_log_block_status.seqno_(some_number);
			
			premote_log_block_status.status_(_Host.MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
			
			
		}
		
		export function onOBSTACLE_DISTANCE(pobstacle_distance: _Host.OBSTACLE_DISTANCE) {
			const some_time_usec = pobstacle_distance.time_usec();
			{
				
				const item = pobstacle_distance.sensor_type();
				if (item !== null)
					_Host.MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER = item;
				
			}
			{
				const item = pobstacle_distance.distances();
				for (let value of item)
					some_number = value;
				
			}
			const some_increment    = pobstacle_distance.increment();
			const some_min_distance = pobstacle_distance.min_distance();
			const some_max_distance = pobstacle_distance.max_distance();
			
		}
		
		export function fill_OBSTACLE_DISTANCE(pobstacle_distance: _Host.OBSTACLE_DISTANCE) {
			pobstacle_distance.time_usec_(some_number);
			
			pobstacle_distance.sensor_type_(_Host.MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
			
			{
				const item = pobstacle_distance.distances();
				
				for (let i = 0; i < _Host.OBSTACLE_DISTANCE.distances.item_len; i++)
					item.set(some_number, i);
				
			}
			pobstacle_distance.increment_(some_number);
			pobstacle_distance.min_distance_(some_number);
			pobstacle_distance.max_distance_(some_number);
			
		}
		
		export function onGPS2_RAW(pgps2_raw: _Host.GPS2_RAW) {
			const some_time_usec = pgps2_raw.time_usec();
			{
				
				const item = pgps2_raw.fix_type();
				if (item !== null)
					_Host.GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS = item;
				
			}
			const some_lat                = pgps2_raw.lat();
			const some_lon                = pgps2_raw.lon();
			const some_alt                = pgps2_raw.alt();
			const some_eph                = pgps2_raw.eph();
			const some_epv                = pgps2_raw.epv();
			const some_vel                = pgps2_raw.vel();
			const some_cog                = pgps2_raw.cog();
			const some_satellites_visible = pgps2_raw.satellites_visible();
			const some_dgps_numch         = pgps2_raw.dgps_numch();
			const some_dgps_age           = pgps2_raw.dgps_age();
			
		}
		
		export function fill_GPS2_RAW(pgps2_raw: _Host.GPS2_RAW) {
			pgps2_raw.time_usec_(some_number);
			
			pgps2_raw.fix_type_(_Host.GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
			
			pgps2_raw.lat_(some_number);
			pgps2_raw.lon_(some_number);
			pgps2_raw.alt_(some_number);
			pgps2_raw.eph_(some_number);
			pgps2_raw.epv_(some_number);
			pgps2_raw.vel_(some_number);
			pgps2_raw.cog_(some_number);
			pgps2_raw.satellites_visible_(some_number);
			pgps2_raw.dgps_numch_(some_number);
			pgps2_raw.dgps_age_(some_number);
			
		}
		
		export function onREQUEST_DATA_STREAM(prequest_data_stream: _Host.REQUEST_DATA_STREAM) {
			const some_target_system    = prequest_data_stream.target_system();
			const some_target_component = prequest_data_stream.target_component();
			const some_req_stream_id    = prequest_data_stream.req_stream_id();
			const some_req_message_rate = prequest_data_stream.req_message_rate();
			const some_start_stop       = prequest_data_stream.start_stop();
			
		}
		
		export function onMEMORY_VECT(pmemory_vect: _Host.MEMORY_VECT) {
			const some_address = pmemory_vect.address();
			const some_ver     = pmemory_vect.ver();
			const some_typE    = pmemory_vect.typE();
			{
				const item = pmemory_vect.value();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_MEMORY_VECT(pmemory_vect: _Host.MEMORY_VECT) {
			pmemory_vect.address_(some_number);
			pmemory_vect.ver_(some_number);
			pmemory_vect.typE_(some_number);
			{
				const item = pmemory_vect.value();
				
				for (let i = 0; i < _Host.MEMORY_VECT.value.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onPARAM_EXT_REQUEST_READ(pparam_ext_request_read: _Host.PARAM_EXT_REQUEST_READ) {
			const some_target_system    = pparam_ext_request_read.target_system();
			const some_target_component = pparam_ext_request_read.target_component();
			{
				
				const item = pparam_ext_request_read.param_id();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_param_index = pparam_ext_request_read.param_index();
			
		}
		
		export function fill_PARAM_EXT_REQUEST_READ(pparam_ext_request_read: _Host.PARAM_EXT_REQUEST_READ) {
			pparam_ext_request_read.target_system_(some_number);
			pparam_ext_request_read.target_component_(some_number);
			pparam_ext_request_read.param_id_(some_string);
			pparam_ext_request_read.param_index_(some_number);
			
		}
		
		export function onHIL_CONTROLS(phil_controls: _Host.HIL_CONTROLS) {
			const some_time_usec      = phil_controls.time_usec();
			const some_roll_ailerons  = phil_controls.roll_ailerons();
			const some_pitch_elevator = phil_controls.pitch_elevator();
			const some_yaw_rudder     = phil_controls.yaw_rudder();
			const some_throttle       = phil_controls.throttle();
			const some_aux1           = phil_controls.aux1();
			const some_aux2           = phil_controls.aux2();
			const some_aux3           = phil_controls.aux3();
			const some_aux4           = phil_controls.aux4();
			{
				
				const item = phil_controls.mode();
				if (item !== null)
					_Host.MAV_MODE.PREFLIGHT = item;
				
			}
			const some_nav_mode = phil_controls.nav_mode();
			
		}
		
		export function onHIL_SENSOR(phil_sensor: _Host.HIL_SENSOR) {
			const some_time_usec      = phil_sensor.time_usec();
			const some_xacc           = phil_sensor.xacc();
			const some_yacc           = phil_sensor.yacc();
			const some_zacc           = phil_sensor.zacc();
			const some_xgyro          = phil_sensor.xgyro();
			const some_ygyro          = phil_sensor.ygyro();
			const some_zgyro          = phil_sensor.zgyro();
			const some_xmag           = phil_sensor.xmag();
			const some_ymag           = phil_sensor.ymag();
			const some_zmag           = phil_sensor.zmag();
			const some_abs_pressure   = phil_sensor.abs_pressure();
			const some_diff_pressure  = phil_sensor.diff_pressure();
			const some_pressure_alt   = phil_sensor.pressure_alt();
			const some_temperature    = phil_sensor.temperature();
			const some_fields_updated = phil_sensor.fields_updated();
			
		}
		
		export function fill_HIL_SENSOR(phil_sensor: _Host.HIL_SENSOR) {
			phil_sensor.time_usec_(some_number);
			phil_sensor.xacc_(some_number);
			phil_sensor.yacc_(some_number);
			phil_sensor.zacc_(some_number);
			phil_sensor.xgyro_(some_number);
			phil_sensor.ygyro_(some_number);
			phil_sensor.zgyro_(some_number);
			phil_sensor.xmag_(some_number);
			phil_sensor.ymag_(some_number);
			phil_sensor.zmag_(some_number);
			phil_sensor.abs_pressure_(some_number);
			phil_sensor.diff_pressure_(some_number);
			phil_sensor.pressure_alt_(some_number);
			phil_sensor.temperature_(some_number);
			phil_sensor.fields_updated_(some_number);
			
		}
		
		export function onSETUP_SIGNING(psetup_signing: _Host.SETUP_SIGNING) {
			const some_target_system    = psetup_signing.target_system();
			const some_target_component = psetup_signing.target_component();
			{
				const item = psetup_signing.secret_key();
				for (let value of item)
					some_number = value;
				
			}
			const some_initial_timestamp = psetup_signing.initial_timestamp();
			
		}
		
		export function fill_SETUP_SIGNING(psetup_signing: _Host.SETUP_SIGNING) {
			psetup_signing.target_system_(some_number);
			psetup_signing.target_component_(some_number);
			{
				const item = psetup_signing.secret_key();
				
				for (let i = 0; i < _Host.SETUP_SIGNING.secret_key.item_len; i++)
					item.set(some_number, i);
				
			}
			psetup_signing.initial_timestamp_(some_number);
			
		}
		
		export function onGPS_RTK(pgps_rtk: _Host.GPS_RTK) {
			const some_time_last_baseline_ms = pgps_rtk.time_last_baseline_ms();
			const some_rtk_receiver_id       = pgps_rtk.rtk_receiver_id();
			const some_wn                    = pgps_rtk.wn();
			const some_tow                   = pgps_rtk.tow();
			const some_rtk_health            = pgps_rtk.rtk_health();
			const some_rtk_rate              = pgps_rtk.rtk_rate();
			const some_nsats                 = pgps_rtk.nsats();
			const some_baseline_coords_type  = pgps_rtk.baseline_coords_type();
			const some_baseline_a_mm         = pgps_rtk.baseline_a_mm();
			const some_baseline_b_mm         = pgps_rtk.baseline_b_mm();
			const some_baseline_c_mm         = pgps_rtk.baseline_c_mm();
			const some_accuracy              = pgps_rtk.accuracy();
			const some_iar_num_hypotheses    = pgps_rtk.iar_num_hypotheses();
			
		}
		
		export function fill_GPS_RTK(pgps_rtk: _Host.GPS_RTK) {
			pgps_rtk.time_last_baseline_ms_(some_number);
			pgps_rtk.rtk_receiver_id_(some_number);
			pgps_rtk.wn_(some_number);
			pgps_rtk.tow_(some_number);
			pgps_rtk.rtk_health_(some_number);
			pgps_rtk.rtk_rate_(some_number);
			pgps_rtk.nsats_(some_number);
			pgps_rtk.baseline_coords_type_(some_number);
			pgps_rtk.baseline_a_mm_(some_number);
			pgps_rtk.baseline_b_mm_(some_number);
			pgps_rtk.baseline_c_mm_(some_number);
			pgps_rtk.accuracy_(some_number);
			pgps_rtk.iar_num_hypotheses_(some_number);
			
		}
		
		export function onPARAM_REQUEST_LIST(pparam_request_list: _Host.PARAM_REQUEST_LIST) {
			const some_target_system    = pparam_request_list.target_system();
			const some_target_component = pparam_request_list.target_component();
			
		}
		
		export function onUAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg: _Host.UAVIONIX_ADSB_OUT_CFG) {
			const some_ICAO = puavionix_adsb_out_cfg.ICAO();
			{
				
				const item = puavionix_adsb_out_cfg.callsign();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = puavionix_adsb_out_cfg.emitterType();
				if (item !== null)
					_Host.ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO = item;
				
			}
			{
				
				const item = puavionix_adsb_out_cfg.aircraftSize();
				if (item !== null)
					_Host.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA = item;
				
			}
			{
				
				const item = puavionix_adsb_out_cfg.gpsOffsetLat();
				if (item !== null)
					_Host.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.NO_DATA = item;
				
			}
			{
				
				const item = puavionix_adsb_out_cfg.gpsOffsetLon();
				if (item !== null)
					_Host.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA = item;
				
			}
			const some_stallSpeed = puavionix_adsb_out_cfg.stallSpeed();
			{
				
				const item = puavionix_adsb_out_cfg.rfSelect();
				if (item !== null)
					_Host.UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY = item;
				
			}
			
		}
		
		export function fill_UAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg: _Host.UAVIONIX_ADSB_OUT_CFG) {
			puavionix_adsb_out_cfg.ICAO_(some_number);
			puavionix_adsb_out_cfg.callsign_(some_string);
			
			puavionix_adsb_out_cfg.emitterType_(_Host.ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
			
			
			puavionix_adsb_out_cfg.aircraftSize_(_Host.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA);
			
			
			puavionix_adsb_out_cfg.gpsOffsetLat_(_Host.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.NO_DATA);
			
			
			puavionix_adsb_out_cfg.gpsOffsetLon_(_Host.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
			
			puavionix_adsb_out_cfg.stallSpeed_(some_number);
			
			puavionix_adsb_out_cfg.rfSelect_(_Host.UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY);
			
			
		}
		
		export function onLANDING_TARGET(planding_target: _Host.LANDING_TARGET) {
			const some_time_usec  = planding_target.time_usec();
			const some_target_num = planding_target.target_num();
			{
				
				const item = planding_target.frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			const some_angle_x  = planding_target.angle_x();
			const some_angle_y  = planding_target.angle_y();
			const some_distance = planding_target.distance();
			const some_size_x   = planding_target.size_x();
			const some_size_y   = planding_target.size_y();
			{
				
				const item = planding_target.x();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = planding_target.y();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = planding_target.z();
				if (item !== null)
					some_number = item;
				
			}
			{
				const fld = planding_target.q();
				if (fld)
					fld.enumerate((item: number, d0: number) => {
							some_number = item;
						}
					);
			}
			{
				
				const item = planding_target.typE();
				if (item !== null)
					_Host.LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON = item;
				
			}
			{
				
				const item = planding_target.position_valid();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function fill_LANDING_TARGET(planding_target: _Host.LANDING_TARGET) {
			planding_target.time_usec_(some_number);
			planding_target.target_num_(some_number);
			
			planding_target.frame_(_Host.MAV_FRAME.MAV_FRAME_GLOBAL);
			
			planding_target.angle_x_(some_number);
			planding_target.angle_y_(some_number);
			planding_target.distance_(some_number);
			planding_target.size_x_(some_number);
			planding_target.size_y_(some_number);
			
			planding_target.x_(some_number);
			
			
			planding_target.y_(some_number);
			
			
			planding_target.z_(some_number);
			
			
			for (let d0 = 0; d0 < _Host.LANDING_TARGET.q.d0; d0++) {
				
				planding_target.q_(some_number, d0);
				
			}
			
			planding_target.typE_(_Host.LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
			
			
			planding_target.position_valid_(some_number);
			
			
		}
		
		export function onSET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target: _Host.SET_ACTUATOR_CONTROL_TARGET) {
			const some_time_usec        = pset_actuator_control_target.time_usec();
			const some_group_mlx        = pset_actuator_control_target.group_mlx();
			const some_target_system    = pset_actuator_control_target.target_system();
			const some_target_component = pset_actuator_control_target.target_component();
			{
				const item = pset_actuator_control_target.controls();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target: _Host.SET_ACTUATOR_CONTROL_TARGET) {
			pset_actuator_control_target.time_usec_(some_number);
			pset_actuator_control_target.group_mlx_(some_number);
			pset_actuator_control_target.target_system_(some_number);
			pset_actuator_control_target.target_component_(some_number);
			{
				const item = pset_actuator_control_target.controls();
				
				for (let i = 0; i < _Host.SET_ACTUATOR_CONTROL_TARGET.controls.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onCONTROL_SYSTEM_STATE(pcontrol_system_state: _Host.CONTROL_SYSTEM_STATE) {
			const some_time_usec = pcontrol_system_state.time_usec();
			const some_x_acc     = pcontrol_system_state.x_acc();
			const some_y_acc     = pcontrol_system_state.y_acc();
			const some_z_acc     = pcontrol_system_state.z_acc();
			const some_x_vel     = pcontrol_system_state.x_vel();
			const some_y_vel     = pcontrol_system_state.y_vel();
			const some_z_vel     = pcontrol_system_state.z_vel();
			const some_x_pos     = pcontrol_system_state.x_pos();
			const some_y_pos     = pcontrol_system_state.y_pos();
			const some_z_pos     = pcontrol_system_state.z_pos();
			const some_airspeed  = pcontrol_system_state.airspeed();
			{
				const item = pcontrol_system_state.vel_variance();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pcontrol_system_state.pos_variance();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pcontrol_system_state.q();
				for (let value of item)
					some_number = value;
				
			}
			const some_roll_rate  = pcontrol_system_state.roll_rate();
			const some_pitch_rate = pcontrol_system_state.pitch_rate();
			const some_yaw_rate   = pcontrol_system_state.yaw_rate();
			
		}
		
		export function fill_CONTROL_SYSTEM_STATE(pcontrol_system_state: _Host.CONTROL_SYSTEM_STATE) {
			pcontrol_system_state.time_usec_(some_number);
			pcontrol_system_state.x_acc_(some_number);
			pcontrol_system_state.y_acc_(some_number);
			pcontrol_system_state.z_acc_(some_number);
			pcontrol_system_state.x_vel_(some_number);
			pcontrol_system_state.y_vel_(some_number);
			pcontrol_system_state.z_vel_(some_number);
			pcontrol_system_state.x_pos_(some_number);
			pcontrol_system_state.y_pos_(some_number);
			pcontrol_system_state.z_pos_(some_number);
			pcontrol_system_state.airspeed_(some_number);
			{
				const item = pcontrol_system_state.vel_variance();
				
				for (let i = 0; i < _Host.CONTROL_SYSTEM_STATE.vel_variance.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pcontrol_system_state.pos_variance();
				
				for (let i = 0; i < _Host.CONTROL_SYSTEM_STATE.pos_variance.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pcontrol_system_state.q();
				
				for (let i = 0; i < _Host.CONTROL_SYSTEM_STATE.q.item_len; i++)
					item.set(some_number, i);
				
			}
			pcontrol_system_state.roll_rate_(some_number);
			pcontrol_system_state.pitch_rate_(some_number);
			pcontrol_system_state.yaw_rate_(some_number);
			
		}
		
		export function onSET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int: _Host.SET_POSITION_TARGET_GLOBAL_INT) {
			const some_time_boot_ms     = pset_position_target_global_int.time_boot_ms();
			const some_target_system    = pset_position_target_global_int.target_system();
			const some_target_component = pset_position_target_global_int.target_component();
			{
				
				const item = pset_position_target_global_int.coordinate_frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			const some_type_mask = pset_position_target_global_int.type_mask();
			const some_lat_int   = pset_position_target_global_int.lat_int();
			const some_lon_int   = pset_position_target_global_int.lon_int();
			const some_alt       = pset_position_target_global_int.alt();
			const some_vx        = pset_position_target_global_int.vx();
			const some_vy        = pset_position_target_global_int.vy();
			const some_vz        = pset_position_target_global_int.vz();
			const some_afx       = pset_position_target_global_int.afx();
			const some_afy       = pset_position_target_global_int.afy();
			const some_afz       = pset_position_target_global_int.afz();
			const some_yaw       = pset_position_target_global_int.yaw();
			const some_yaw_rate  = pset_position_target_global_int.yaw_rate();
			
		}
		
		export function onDATA32(pdata32: _Host.DATA32) {
			const some_typE = pdata32.typE();
			const some_len  = pdata32.len();
			{
				const item = pdata32.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_DATA32(pdata32: _Host.DATA32) {
			pdata32.typE_(some_number);
			pdata32.len_(some_number);
			{
				const item = pdata32.daTa();
				
				for (let i = 0; i < _Host.DATA32.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onPING33(pping33: _Host.PING33) {
			{
				
				const item = pping33.testBOOL();
				if (item !== null)
					some_boolean = item;
				
			}
			{
				
				const item = pping33.seq();
				if (item !== null)
					some_number = item;
				
			}
			const some_field = pping33.field();
			{
				const fld = pping33.field1();
				if (fld instanceof _Host.PING33.field1.Field) {
					some_number = fld.d0();
					some_number = _Host.PING33.field1.d0_max;
					some_number = _Host.PING33.field1.d1;
					some_number = _Host.PING33.field1.d2;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						const some_field1 = item;
					});
				}
			}
			{
				const fld = pping33.field12();
				if (fld)
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
							const some_field12 = item;
						}
					);
			}
			{
				const fld = pping33.field13();
				if (fld)
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
							some_number = item;
						}
					);
			}
			
			_Host.PING33.TTTT.enumerate(pping33, (item: number, d0: number, d1: number, d2: number) => {
					const some_TTTT = pping33.TTTT(d0, d1, d2);
				}
			);
			{
				
				const item = pping33.WWWWWWWW();
				if (item !== null)
					some_number = item;
				
			}
			const some_testBOOL2 = pping33.testBOOL2();
			const some_testBOOL3 = pping33.testBOOL3();
			const some_bit_field = pping33.bit_field();
			{
				
				const item = pping33.bit_field2();
				if (item !== null)
					some_number = item;
				
			}
			{
				const fld = pping33.Field_Bits();
				if (fld)
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
							const some_Field_Bits = item;
						}
					);
			}
			{
				const fld = pping33.SparseFixAllBits();
				if (fld instanceof _Host.PING33.SparseFixAllBits.Field) {
					some_number = fld.d0();
					some_number = _Host.PING33.SparseFixAllBits.d0_max;
					some_number = _Host.PING33.SparseFixAllBits.d1;
					some_number = _Host.PING33.SparseFixAllBits.d2;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						some_number = item;
					});
				}
			}
			{
				const fld = pping33.FixAllBits();
				if (fld instanceof _Host.PING33.FixAllBits.Field) {
					some_number = fld.d0();
					some_number = _Host.PING33.FixAllBits.d0_max;
					some_number = _Host.PING33.FixAllBits.d1;
					some_number = _Host.PING33.FixAllBits.d2;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						const some_FixAllBits = item;
					});
				}
			}
			{
				const fld = pping33.VarAllBits();
				if (fld instanceof _Host.PING33.VarAllBits.Field) {
					some_number = fld.d0();
					some_number = _Host.PING33.VarAllBits.d0_max;
					some_number = _Host.PING33.VarAllBits.d1;
					some_number = fld.d2();
					some_number = _Host.PING33.VarAllBits.d2_max;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						const some_VarAllBits = item;
					});
				}
			}
			{
				const fld = pping33.SparseVarAllBits();
				if (fld instanceof _Host.PING33.SparseVarAllBits.Field) {
					some_number = fld.d0();
					some_number = _Host.PING33.SparseVarAllBits.d0_max;
					some_number = _Host.PING33.SparseVarAllBits.d1;
					some_number = fld.d2();
					some_number = _Host.PING33.SparseVarAllBits.d2_max;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						some_number = item;
					});
				}
			}
			{
				const fld = pping33.VarEachBits();
				if (fld instanceof _Host.PING33.VarEachBits.Field) {
					some_number = fld.d0();
					some_number = _Host.PING33.VarEachBits.d0_max;
					some_number = _Host.PING33.VarEachBits.d1;
					some_number = _Host.PING33.VarEachBits.d2;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						const some_VarEachBits = item;
					});
				}
			}
			{
				const fld = pping33.SparsVarEachBits();
				if (fld instanceof _Host.PING33.SparsVarEachBits.Field) {
					some_number = fld.d0();
					some_number = _Host.PING33.SparsVarEachBits.d0_max;
					some_number = _Host.PING33.SparsVarEachBits.d1;
					some_number = _Host.PING33.SparsVarEachBits.d2;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						some_number = item;
					});
				}
			}
			{
				
				const item = pping33.testBOOLX();
				if (item !== null)
					some_boolean = item;
				
			}
			{
				
				const item = pping33.testBOOL2X();
				if (item !== null)
					some_boolean = item;
				
			}
			{
				
				const item = pping33.testBOOL3X();
				if (item !== null)
					some_boolean = item;
				
			}
			{
				
				const item = pping33.MMMMMM();
				if (item !== null)
					_Host.MAV_MODE.PREFLIGHT = item;
				
			}
			{
				const fld = pping33.field44();
				if (fld instanceof _Host.PING33.field44.Field) {
					some_number = _Host.PING33.field44.d0;
					some_number = _Host.PING33.field44.d1;
					some_number = fld.d2();
					some_number = _Host.PING33.field44.d2_max;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						const some_field44 = item;
					});
				}
			}
			{
				const fld = pping33.field634();
				if (fld instanceof _Host.PING33.field634.Field) {
					some_number = _Host.PING33.field634.d0;
					some_number = _Host.PING33.field634.d1;
					some_number = fld.d2();
					some_number = _Host.PING33.field634.d2_max;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						const some_field634 = item;
					});
				}
			}
			{
				const fld = pping33.field33344();
				if (fld instanceof _Host.PING33.field33344.Field) {
					some_number = _Host.PING33.field33344.d0;
					some_number = _Host.PING33.field33344.d1;
					some_number = fld.d2();
					some_number = _Host.PING33.field33344.d2_max;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						some_number = item;
					});
				}
			}
			{
				const fld = pping33.field333634();
				if (fld instanceof _Host.PING33.field333634.Field) {
					some_number = _Host.PING33.field333634.d0;
					some_number = _Host.PING33.field333634.d1;
					some_number = fld.d2();
					some_number = _Host.PING33.field333634.d2_max;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						some_number = item;
					});
				}
			}
			{
				const fld = pping33.field__();
				if (fld)
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
							some_number = item;
						}
					);
			}
			
			_Host.PING33.field6.enumerate(pping33, (item: number, d0: number, d1: number, d2: number) => {
					const some_field6 = pping33.field6(d0, d1, d2);
				}
			);
			{
				const fld = pping33.field63();
				if (fld instanceof _Host.PING33.field63.Field) {
					some_number = _Host.PING33.field63.d0;
					some_number = _Host.PING33.field63.d1;
					some_number = fld.d2();
					some_number = _Host.PING33.field63.d2_max;
					
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
						const some_field63 = item;
					});
				}
			}
			{
				const fld = pping33.uid2();
				if (fld)
					fld.enumerate((item: number, d0: number) => {
							some_number = item;
						}
					);
			}
			{
				const fld = pping33.field2();
				if (fld)
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
							some_number = item;
						}
					);
			}
			{
				const fld = pping33.field4();
				if (fld)
					fld.enumerate((item: number, d0: number, d1: number, d2: number) => {
							some_number = item;
						}
					);
			}
			{
				
				const item = pping33.stringtest1();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				const fld = pping33.stringtest2();
				if (fld instanceof _Host.PING33.stringtest2.Field) {
					some_number = _Host.PING33.stringtest2.d0;
					some_number = _Host.PING33.stringtest2.d1;
					some_number = fld.d2();
					some_number = _Host.PING33.stringtest2.d2_max;
					
					fld.enumerate((item: _Cursor.UTF8, d0: number, d1: number, d2: number) => {some_string = item.toString();});
				}
			}
			{
				
				const item = pping33.stringtest3();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = pping33.stringtest4();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function fill_PING33(pping33: _Host.PING33) {
			
			pping33.testBOOL_(some_boolean);
			
			
			pping33.seq_(some_number);
			
			pping33.field_(some_number);
			
			{
				
				let init = pping33.field1();
				if (init instanceof _Host.PING33.field1.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < fld.d0(); d0++)
							for (let d1 = 0; d1 < _Host.PING33.field1.d1; d1++)
								for (let d2 = 0; d2 < _Host.PING33.field1.d2; d2++) {
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			for (let d0 = 0; d0 < _Host.PING33.field12.d0; d0++)
				for (let d1 = 0; d1 < _Host.PING33.field12.d1; d1++)
					for (let d2 = 0; d2 < _Host.PING33.field12.d2; d2++) {
						pping33.field12_(some_number, d0, d1, d2);
						
					}
			
			for (let d0 = 0; d0 < _Host.PING33.field13.d0; d0++)
				for (let d1 = 0; d1 < _Host.PING33.field13.d1; d1++)
					for (let d2 = 0; d2 < _Host.PING33.field13.d2; d2++) {
						
						pping33.field13_(some_number, d0, d1, d2);
						
					}
			
			_Host.PING33.TTTT.enumerate(pping33, (item: number, d0: number, d1: number, d2: number) => {
					pping33.TTTT_(some_number, d0, d1, d2);
				}
			);
			
			pping33.WWWWWWWW_(some_number);
			
			pping33.testBOOL2_(some_boolean);
			pping33.testBOOL3_(some_boolean);
			pping33.bit_field_(some_number);
			
			pping33.bit_field2_(some_number);
			
			
			for (let d0 = 0; d0 < _Host.PING33.Field_Bits.d0; d0++)
				for (let d1 = 0; d1 < _Host.PING33.Field_Bits.d1; d1++)
					for (let d2 = 0; d2 < _Host.PING33.Field_Bits.d2; d2++) {
						pping33.Field_Bits_(some_number, d0, d1, d2);
						
					}
			
			{
				
				let init = pping33.SparseFixAllBits();
				if (init instanceof _Host.PING33.SparseFixAllBits.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < fld.d0(); d0++)
							for (let d1 = 0; d1 < _Host.PING33.SparseFixAllBits.d1; d1++)
								for (let d2 = 0; d2 < _Host.PING33.SparseFixAllBits.d2; d2++) {
									
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			{
				
				let init = pping33.FixAllBits();
				if (init instanceof _Host.PING33.FixAllBits.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < fld.d0(); d0++)
							for (let d1 = 0; d1 < _Host.PING33.FixAllBits.d1; d1++)
								for (let d2 = 0; d2 < _Host.PING33.FixAllBits.d2; d2++) {
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			{
				
				let init = pping33.VarAllBits();
				if (init instanceof _Host.PING33.VarAllBits.Initializer) {
					const fld = init.init(some_number, some_number);
					if (fld)
						for (let d0 = 0; d0 < fld.d0(); d0++)
							for (let d1 = 0; d1 < _Host.PING33.VarAllBits.d1; d1++)
								for (let d2 = 0; d2 < fld.d2(); d2++) {
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			{
				
				let init = pping33.SparseVarAllBits();
				if (init instanceof _Host.PING33.SparseVarAllBits.Initializer) {
					const fld = init.init(some_number, some_number);
					if (fld)
						for (let d0 = 0; d0 < fld.d0(); d0++)
							for (let d1 = 0; d1 < _Host.PING33.SparseVarAllBits.d1; d1++)
								for (let d2 = 0; d2 < fld.d2(); d2++) {
									
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			{
				
				let init = pping33.VarEachBits();
				if (init instanceof _Host.PING33.VarEachBits.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < fld.d0(); d0++)
							for (let d1 = 0; d1 < _Host.PING33.VarEachBits.d1; d1++)
								for (let d2 = 0; d2 < _Host.PING33.VarEachBits.d2; d2++) {
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			{
				
				let init = pping33.SparsVarEachBits();
				if (init instanceof _Host.PING33.SparsVarEachBits.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < fld.d0(); d0++)
							for (let d1 = 0; d1 < _Host.PING33.SparsVarEachBits.d1; d1++)
								for (let d2 = 0; d2 < _Host.PING33.SparsVarEachBits.d2; d2++) {
									
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			pping33.testBOOLX_(some_boolean);
			
			
			pping33.testBOOL2X_(some_boolean);
			
			
			pping33.testBOOL3X_(some_boolean);
			
			
			pping33.MMMMMM_(_Host.MAV_MODE.PREFLIGHT);
			
			
			{
				
				let init = pping33.field44();
				if (init instanceof _Host.PING33.field44.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < _Host.PING33.field44.d0; d0++)
							for (let d1 = 0; d1 < _Host.PING33.field44.d1; d1++)
								for (let d2 = 0; d2 < fld.d2(); d2++) {
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			{
				
				let init = pping33.field634();
				if (init instanceof _Host.PING33.field634.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < _Host.PING33.field634.d0; d0++)
							for (let d1 = 0; d1 < _Host.PING33.field634.d1; d1++)
								for (let d2 = 0; d2 < fld.d2(); d2++) {
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			{
				
				let init = pping33.field33344();
				if (init instanceof _Host.PING33.field33344.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < _Host.PING33.field33344.d0; d0++)
							for (let d1 = 0; d1 < _Host.PING33.field33344.d1; d1++)
								for (let d2 = 0; d2 < fld.d2(); d2++) {
									
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			{
				
				let init = pping33.field333634();
				if (init instanceof _Host.PING33.field333634.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < _Host.PING33.field333634.d0; d0++)
							for (let d1 = 0; d1 < _Host.PING33.field333634.d1; d1++)
								for (let d2 = 0; d2 < fld.d2(); d2++) {
									
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			for (let d0 = 0; d0 < _Host.PING33.field__.d0; d0++)
				for (let d1 = 0; d1 < _Host.PING33.field__.d1; d1++)
					for (let d2 = 0; d2 < _Host.PING33.field__.d2; d2++) {
						
						pping33.field___(some_number, d0, d1, d2);
						
					}
			
			_Host.PING33.field6.enumerate(pping33, (item: number, d0: number, d1: number, d2: number) => {
					pping33.field6_(some_number, d0, d1, d2);
				}
			);
			
			{
				
				let init = pping33.field63();
				if (init instanceof _Host.PING33.field63.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < _Host.PING33.field63.d0; d0++)
							for (let d1 = 0; d1 < _Host.PING33.field63.d1; d1++)
								for (let d2 = 0; d2 < fld.d2(); d2++) {
									fld.set(some_number, d0, d1, d2);
									
								}
				}
			}
			
			for (let d0 = 0; d0 < _Host.PING33.uid2.d0; d0++) {
				
				pping33.uid2_(some_number, d0);
				
			}
			
			for (let d0 = 0; d0 < _Host.PING33.field2.d0; d0++)
				for (let d1 = 0; d1 < _Host.PING33.field2.d1; d1++)
					for (let d2 = 0; d2 < _Host.PING33.field2.d2; d2++) {
						
						pping33.field2_(some_number, d0, d1, d2);
						
					}
			
			for (let d0 = 0; d0 < _Host.PING33.field4.d0; d0++)
				for (let d1 = 0; d1 < _Host.PING33.field4.d1; d1++)
					for (let d2 = 0; d2 < _Host.PING33.field4.d2; d2++) {
						
						pping33.field4_(some_number, d0, d1, d2);
						
					}
			pping33.stringtest1_(some_string);
			
			{
				
				let init = pping33.stringtest2();
				if (init instanceof _Host.PING33.stringtest2.Initializer) {
					const fld = init.init(some_number);
					if (fld)
						for (let d0 = 0; d0 < _Host.PING33.stringtest2.d0; d0++)
							for (let d1 = 0; d1 < _Host.PING33.stringtest2.d1; d1++)
								for (let d2 = 0; d2 < fld.d2(); d2++) {
									fld.set(some_string, d0, d1, d2);
								}
				}
			}
			pping33.stringtest3_(some_string);
			pping33.stringtest4_(some_string);
			
		}
		
		export function onVFR_HUD(pvfr_hud: _Host.VFR_HUD) {
			const some_airspeed    = pvfr_hud.airspeed();
			const some_groundspeed = pvfr_hud.groundspeed();
			const some_heading     = pvfr_hud.heading();
			const some_throttle    = pvfr_hud.throttle();
			const some_alt         = pvfr_hud.alt();
			const some_climb       = pvfr_hud.climb();
			
		}
		
		export function onRALLY_POINT(prally_point: _Host.RALLY_POINT) {
			const some_target_system    = prally_point.target_system();
			const some_target_component = prally_point.target_component();
			const some_idx              = prally_point.idx();
			const some_count            = prally_point.count();
			const some_lat              = prally_point.lat();
			const some_lng              = prally_point.lng();
			const some_alt              = prally_point.alt();
			const some_break_alt        = prally_point.break_alt();
			const some_land_dir         = prally_point.land_dir();
			{
				
				const item = prally_point.flags();
				if (item !== null)
					_Host.RALLY_FLAGS.FAVORABLE_WIND = item;
				
			}
			
		}
		
		export function fill_RALLY_POINT(prally_point: _Host.RALLY_POINT) {
			prally_point.target_system_(some_number);
			prally_point.target_component_(some_number);
			prally_point.idx_(some_number);
			prally_point.count_(some_number);
			prally_point.lat_(some_number);
			prally_point.lng_(some_number);
			prally_point.alt_(some_number);
			prally_point.break_alt_(some_number);
			prally_point.land_dir_(some_number);
			
			prally_point.flags_(_Host.RALLY_FLAGS.FAVORABLE_WIND);
			
			
		}
		
		export function onMISSION_SET_CURRENT(pmission_set_current: _Host.MISSION_SET_CURRENT) {
			const some_target_system    = pmission_set_current.target_system();
			const some_target_component = pmission_set_current.target_component();
			const some_seq              = pmission_set_current.seq();
			
		}
		
		export function onADAP_TUNING(padap_tuning: _Host.ADAP_TUNING) {
			{
				
				const item = padap_tuning.axis();
				if (item !== null)
					_Host.PID_TUNING_AXIS.PID_TUNING_ROLL = item;
				
			}
			const some_desired   = padap_tuning.desired();
			const some_achieved  = padap_tuning.achieved();
			const some_error     = padap_tuning.error();
			const some_theta     = padap_tuning.theta();
			const some_omega     = padap_tuning.omega();
			const some_sigma     = padap_tuning.sigma();
			const some_theta_dot = padap_tuning.theta_dot();
			const some_omega_dot = padap_tuning.omega_dot();
			const some_sigma_dot = padap_tuning.sigma_dot();
			const some_f         = padap_tuning.f();
			const some_f_dot     = padap_tuning.f_dot();
			const some_u         = padap_tuning.u();
			
		}
		
		export function fill_ADAP_TUNING(padap_tuning: _Host.ADAP_TUNING) {
			
			padap_tuning.axis_(_Host.PID_TUNING_AXIS.PID_TUNING_ROLL);
			
			padap_tuning.desired_(some_number);
			padap_tuning.achieved_(some_number);
			padap_tuning.error_(some_number);
			padap_tuning.theta_(some_number);
			padap_tuning.omega_(some_number);
			padap_tuning.sigma_(some_number);
			padap_tuning.theta_dot_(some_number);
			padap_tuning.omega_dot_(some_number);
			padap_tuning.sigma_dot_(some_number);
			padap_tuning.f_(some_number);
			padap_tuning.f_dot_(some_number);
			padap_tuning.u_(some_number);
			
		}
		
		export function onVIBRATION(pvibration: _Host.VIBRATION) {
			const some_time_usec   = pvibration.time_usec();
			const some_vibration_x = pvibration.vibration_x();
			const some_vibration_y = pvibration.vibration_y();
			const some_vibration_z = pvibration.vibration_z();
			const some_clipping_0  = pvibration.clipping_0();
			const some_clipping_1  = pvibration.clipping_1();
			const some_clipping_2  = pvibration.clipping_2();
			
		}
		
		export function fill_VIBRATION(pvibration: _Host.VIBRATION) {
			pvibration.time_usec_(some_number);
			pvibration.vibration_x_(some_number);
			pvibration.vibration_y_(some_number);
			pvibration.vibration_z_(some_number);
			pvibration.clipping_0_(some_number);
			pvibration.clipping_1_(some_number);
			pvibration.clipping_2_(some_number);
			
		}
		
		export function onPARAM_EXT_VALUE(pparam_ext_value: _Host.PARAM_EXT_VALUE) {
			{
				
				const item = pparam_ext_value.param_id();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = pparam_ext_value.param_value();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = pparam_ext_value.param_type();
				if (item !== null)
					_Host.MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8 = item;
				
			}
			const some_param_count = pparam_ext_value.param_count();
			const some_param_index = pparam_ext_value.param_index();
			
		}
		
		export function fill_PARAM_EXT_VALUE(pparam_ext_value: _Host.PARAM_EXT_VALUE) {
			pparam_ext_value.param_id_(some_string);
			pparam_ext_value.param_value_(some_string);
			
			pparam_ext_value.param_type_(_Host.MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
			
			pparam_ext_value.param_count_(some_number);
			pparam_ext_value.param_index_(some_number);
			
		}
		
		export function onBATTERY2(pbattery2: _Host.BATTERY2) {
			const some_voltage         = pbattery2.voltage();
			const some_current_battery = pbattery2.current_battery();
			
		}
		
		export function fill_BATTERY2(pbattery2: _Host.BATTERY2) {
			pbattery2.voltage_(some_number);
			pbattery2.current_battery_(some_number);
			
		}
		
		export function onLIMITS_STATUS(plimits_status: _Host.LIMITS_STATUS) {
			{
				
				const item = plimits_status.limits_state();
				if (item !== null)
					_Host.LIMITS_STATE.LIMITS_INIT = item;
				
			}
			const some_last_trigger  = plimits_status.last_trigger();
			const some_last_action   = plimits_status.last_action();
			const some_last_recovery = plimits_status.last_recovery();
			const some_last_clear    = plimits_status.last_clear();
			const some_breach_count  = plimits_status.breach_count();
			{
				
				const item = plimits_status.mods_enabled();
				if (item !== null)
					_Host.LIMIT_MODULE.LIMIT_GPSLOCK = item;
				
			}
			{
				
				const item = plimits_status.mods_required();
				if (item !== null)
					_Host.LIMIT_MODULE.LIMIT_GPSLOCK = item;
				
			}
			{
				
				const item = plimits_status.mods_triggered();
				if (item !== null)
					_Host.LIMIT_MODULE.LIMIT_GPSLOCK = item;
				
			}
			
		}
		
		export function fill_LIMITS_STATUS(plimits_status: _Host.LIMITS_STATUS) {
			
			plimits_status.limits_state_(_Host.LIMITS_STATE.LIMITS_INIT);
			
			plimits_status.last_trigger_(some_number);
			plimits_status.last_action_(some_number);
			plimits_status.last_recovery_(some_number);
			plimits_status.last_clear_(some_number);
			plimits_status.breach_count_(some_number);
			
			plimits_status.mods_enabled_(_Host.LIMIT_MODULE.LIMIT_GPSLOCK);
			
			
			plimits_status.mods_required_(_Host.LIMIT_MODULE.LIMIT_GPSLOCK);
			
			
			plimits_status.mods_triggered_(_Host.LIMIT_MODULE.LIMIT_GPSLOCK);
			
			
		}
		
		export function onCAMERA_FEEDBACK(pcamera_feedback: _Host.CAMERA_FEEDBACK) {
			const some_time_usec     = pcamera_feedback.time_usec();
			const some_target_system = pcamera_feedback.target_system();
			const some_cam_idx       = pcamera_feedback.cam_idx();
			const some_img_idx       = pcamera_feedback.img_idx();
			const some_lat           = pcamera_feedback.lat();
			const some_lng           = pcamera_feedback.lng();
			const some_alt_msl       = pcamera_feedback.alt_msl();
			const some_alt_rel       = pcamera_feedback.alt_rel();
			const some_roll          = pcamera_feedback.roll();
			const some_pitch         = pcamera_feedback.pitch();
			const some_yaw           = pcamera_feedback.yaw();
			const some_foc_len       = pcamera_feedback.foc_len();
			{
				
				const item = pcamera_feedback.flags();
				if (item !== null)
					_Host.CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO = item;
				
			}
			
		}
		
		export function fill_CAMERA_FEEDBACK(pcamera_feedback: _Host.CAMERA_FEEDBACK) {
			pcamera_feedback.time_usec_(some_number);
			pcamera_feedback.target_system_(some_number);
			pcamera_feedback.cam_idx_(some_number);
			pcamera_feedback.img_idx_(some_number);
			pcamera_feedback.lat_(some_number);
			pcamera_feedback.lng_(some_number);
			pcamera_feedback.alt_msl_(some_number);
			pcamera_feedback.alt_rel_(some_number);
			pcamera_feedback.roll_(some_number);
			pcamera_feedback.pitch_(some_number);
			pcamera_feedback.yaw_(some_number);
			pcamera_feedback.foc_len_(some_number);
			
			pcamera_feedback.flags_(_Host.CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO);
			
			
		}
		
		export function onHIL_GPS(phil_gps: _Host.HIL_GPS) {
			const some_time_usec          = phil_gps.time_usec();
			const some_fix_type           = phil_gps.fix_type();
			const some_lat                = phil_gps.lat();
			const some_lon                = phil_gps.lon();
			const some_alt                = phil_gps.alt();
			const some_eph                = phil_gps.eph();
			const some_epv                = phil_gps.epv();
			const some_vel                = phil_gps.vel();
			const some_vn                 = phil_gps.vn();
			const some_ve                 = phil_gps.ve();
			const some_vd                 = phil_gps.vd();
			const some_cog                = phil_gps.cog();
			const some_satellites_visible = phil_gps.satellites_visible();
			
		}
		
		export function fill_HIL_GPS(phil_gps: _Host.HIL_GPS) {
			phil_gps.time_usec_(some_number);
			phil_gps.fix_type_(some_number);
			phil_gps.lat_(some_number);
			phil_gps.lon_(some_number);
			phil_gps.alt_(some_number);
			phil_gps.eph_(some_number);
			phil_gps.epv_(some_number);
			phil_gps.vel_(some_number);
			phil_gps.vn_(some_number);
			phil_gps.ve_(some_number);
			phil_gps.vd_(some_number);
			phil_gps.cog_(some_number);
			phil_gps.satellites_visible_(some_number);
			
		}
		
		export function onNAV_CONTROLLER_OUTPUT(pnav_controller_output: _Host.NAV_CONTROLLER_OUTPUT) {
			const some_nav_roll       = pnav_controller_output.nav_roll();
			const some_nav_pitch      = pnav_controller_output.nav_pitch();
			const some_nav_bearing    = pnav_controller_output.nav_bearing();
			const some_target_bearing = pnav_controller_output.target_bearing();
			const some_wp_dist        = pnav_controller_output.wp_dist();
			const some_alt_error      = pnav_controller_output.alt_error();
			const some_aspd_error     = pnav_controller_output.aspd_error();
			const some_xtrack_error   = pnav_controller_output.xtrack_error();
			
		}
		
		export function onAUTH_KEY(pauth_key: _Host.AUTH_KEY) {
			{
				
				const item = pauth_key.key();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function onFENCE_FETCH_POINT(pfence_fetch_point: _Host.FENCE_FETCH_POINT) {
			const some_target_system    = pfence_fetch_point.target_system();
			const some_target_component = pfence_fetch_point.target_component();
			const some_idx              = pfence_fetch_point.idx();
			
		}
		
		export function fill_FENCE_FETCH_POINT(pfence_fetch_point: _Host.FENCE_FETCH_POINT) {
			pfence_fetch_point.target_system_(some_number);
			pfence_fetch_point.target_component_(some_number);
			pfence_fetch_point.idx_(some_number);
			
		}
		
		export function onRADIO(pradio: _Host.RADIO) {
			const some_rssi     = pradio.rssi();
			const some_remrssi  = pradio.remrssi();
			const some_txbuf    = pradio.txbuf();
			const some_noise    = pradio.noise();
			const some_remnoise = pradio.remnoise();
			const some_rxerrors = pradio.rxerrors();
			const some_fixeD    = pradio.fixeD();
			
		}
		
		export function fill_RADIO(pradio: _Host.RADIO) {
			pradio.rssi_(some_number);
			pradio.remrssi_(some_number);
			pradio.txbuf_(some_number);
			pradio.noise_(some_number);
			pradio.remnoise_(some_number);
			pradio.rxerrors_(some_number);
			pradio.fixeD_(some_number);
			
		}
		
		export function onLOCAL_POSITION_NED_COV(plocal_position_ned_cov: _Host.LOCAL_POSITION_NED_COV) {
			const some_time_usec = plocal_position_ned_cov.time_usec();
			{
				
				const item = plocal_position_ned_cov.estimator_type();
				if (item !== null)
					_Host.MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE = item;
				
			}
			const some_x  = plocal_position_ned_cov.x();
			const some_y  = plocal_position_ned_cov.y();
			const some_z  = plocal_position_ned_cov.z();
			const some_vx = plocal_position_ned_cov.vx();
			const some_vy = plocal_position_ned_cov.vy();
			const some_vz = plocal_position_ned_cov.vz();
			const some_ax = plocal_position_ned_cov.ax();
			const some_ay = plocal_position_ned_cov.ay();
			const some_az = plocal_position_ned_cov.az();
			{
				const item = plocal_position_ned_cov.covariance();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function onAIRSPEED_AUTOCAL(pairspeed_autocal: _Host.AIRSPEED_AUTOCAL) {
			const some_vx            = pairspeed_autocal.vx();
			const some_vy            = pairspeed_autocal.vy();
			const some_vz            = pairspeed_autocal.vz();
			const some_diff_pressure = pairspeed_autocal.diff_pressure();
			const some_EAS2TAS       = pairspeed_autocal.EAS2TAS();
			const some_ratio         = pairspeed_autocal.ratio();
			const some_state_x       = pairspeed_autocal.state_x();
			const some_state_y       = pairspeed_autocal.state_y();
			const some_state_z       = pairspeed_autocal.state_z();
			const some_Pax           = pairspeed_autocal.Pax();
			const some_Pby           = pairspeed_autocal.Pby();
			const some_Pcz           = pairspeed_autocal.Pcz();
			
		}
		
		export function fill_AIRSPEED_AUTOCAL(pairspeed_autocal: _Host.AIRSPEED_AUTOCAL) {
			pairspeed_autocal.vx_(some_number);
			pairspeed_autocal.vy_(some_number);
			pairspeed_autocal.vz_(some_number);
			pairspeed_autocal.diff_pressure_(some_number);
			pairspeed_autocal.EAS2TAS_(some_number);
			pairspeed_autocal.ratio_(some_number);
			pairspeed_autocal.state_x_(some_number);
			pairspeed_autocal.state_y_(some_number);
			pairspeed_autocal.state_z_(some_number);
			pairspeed_autocal.Pax_(some_number);
			pairspeed_autocal.Pby_(some_number);
			pairspeed_autocal.Pcz_(some_number);
			
		}
		
		export function onATT_POS_MOCAP(patt_pos_mocap: _Host.ATT_POS_MOCAP) {
			const some_time_usec = patt_pos_mocap.time_usec();
			{
				const item = patt_pos_mocap.q();
				for (let value of item)
					some_number = value;
				
			}
			const some_x = patt_pos_mocap.x();
			const some_y = patt_pos_mocap.y();
			const some_z = patt_pos_mocap.z();
			
		}
		
		export function fill_ATT_POS_MOCAP(patt_pos_mocap: _Host.ATT_POS_MOCAP) {
			patt_pos_mocap.time_usec_(some_number);
			{
				const item = patt_pos_mocap.q();
				
				for (let i = 0; i < _Host.ATT_POS_MOCAP.q.item_len; i++)
					item.set(some_number, i);
				
			}
			patt_pos_mocap.x_(some_number);
			patt_pos_mocap.y_(some_number);
			patt_pos_mocap.z_(some_number);
			
		}
		
		export function onSTATUSTEXT(pstatustext: _Host.STATUSTEXT) {
			{
				
				const item = pstatustext.severity();
				if (item !== null)
					_Host.MAV_SEVERITY.MAV_SEVERITY_EMERGENCY = item;
				
			}
			{
				
				const item = pstatustext.text();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function fill_STATUSTEXT(pstatustext: _Host.STATUSTEXT) {
			
			pstatustext.severity_(_Host.MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
			
			pstatustext.text_(some_string);
			
		}
		
		export function onPING(pping: _Host.PING) {
			const some_time_usec        = pping.time_usec();
			const some_seq              = pping.seq();
			const some_target_system    = pping.target_system();
			const some_target_component = pping.target_component();
			
		}
		
		export function onGOPRO_GET_REQUEST(pgopro_get_request: _Host.GOPRO_GET_REQUEST) {
			const some_target_system    = pgopro_get_request.target_system();
			const some_target_component = pgopro_get_request.target_component();
			{
				
				const item = pgopro_get_request.cmd_id();
				if (item !== null)
					_Host.GOPRO_COMMAND.GOPRO_COMMAND_POWER = item;
				
			}
			
		}
		
		export function fill_GOPRO_GET_REQUEST(pgopro_get_request: _Host.GOPRO_GET_REQUEST) {
			pgopro_get_request.target_system_(some_number);
			pgopro_get_request.target_component_(some_number);
			
			pgopro_get_request.cmd_id_(_Host.GOPRO_COMMAND.GOPRO_COMMAND_POWER);
			
			
		}
		
		export function onCAMERA_CAPTURE_STATUS(pcamera_capture_status: _Host.CAMERA_CAPTURE_STATUS) {
			const some_time_boot_ms       = pcamera_capture_status.time_boot_ms();
			const some_image_status       = pcamera_capture_status.image_status();
			const some_video_status       = pcamera_capture_status.video_status();
			const some_image_interval     = pcamera_capture_status.image_interval();
			const some_recording_time_ms  = pcamera_capture_status.recording_time_ms();
			const some_available_capacity = pcamera_capture_status.available_capacity();
			
		}
		
		export function fill_CAMERA_CAPTURE_STATUS(pcamera_capture_status: _Host.CAMERA_CAPTURE_STATUS) {
			pcamera_capture_status.time_boot_ms_(some_number);
			pcamera_capture_status.image_status_(some_number);
			pcamera_capture_status.video_status_(some_number);
			pcamera_capture_status.image_interval_(some_number);
			pcamera_capture_status.recording_time_ms_(some_number);
			pcamera_capture_status.available_capacity_(some_number);
			
		}
		
		export function onGLOBAL_POSITION_INT(pglobal_position_int: _Host.GLOBAL_POSITION_INT) {
			const some_time_boot_ms = pglobal_position_int.time_boot_ms();
			const some_lat          = pglobal_position_int.lat();
			const some_lon          = pglobal_position_int.lon();
			const some_alt          = pglobal_position_int.alt();
			const some_relative_alt = pglobal_position_int.relative_alt();
			const some_vx           = pglobal_position_int.vx();
			const some_vy           = pglobal_position_int.vy();
			const some_vz           = pglobal_position_int.vz();
			const some_hdg          = pglobal_position_int.hdg();
			
		}
		
		export function onENCAPSULATED_DATA(pencapsulated_data: _Host.ENCAPSULATED_DATA) {
			const some_seqnr = pencapsulated_data.seqnr();
			{
				const item = pencapsulated_data.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_ENCAPSULATED_DATA(pencapsulated_data: _Host.ENCAPSULATED_DATA) {
			pencapsulated_data.seqnr_(some_number);
			{
				const item = pencapsulated_data.daTa();
				
				for (let i = 0; i < _Host.ENCAPSULATED_DATA.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onGPS_INPUT(pgps_input: _Host.GPS_INPUT) {
			const some_time_usec = pgps_input.time_usec();
			const some_gps_id    = pgps_input.gps_id();
			{
				
				const item = pgps_input.ignore_flags();
				if (item !== null)
					_Host.GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT = item;
				
			}
			const some_time_week_ms       = pgps_input.time_week_ms();
			const some_time_week          = pgps_input.time_week();
			const some_fix_type           = pgps_input.fix_type();
			const some_lat                = pgps_input.lat();
			const some_lon                = pgps_input.lon();
			const some_alt                = pgps_input.alt();
			const some_hdop               = pgps_input.hdop();
			const some_vdop               = pgps_input.vdop();
			const some_vn                 = pgps_input.vn();
			const some_ve                 = pgps_input.ve();
			const some_vd                 = pgps_input.vd();
			const some_speed_accuracy     = pgps_input.speed_accuracy();
			const some_horiz_accuracy     = pgps_input.horiz_accuracy();
			const some_vert_accuracy      = pgps_input.vert_accuracy();
			const some_satellites_visible = pgps_input.satellites_visible();
			
		}
		
		export function fill_GPS_INPUT(pgps_input: _Host.GPS_INPUT) {
			pgps_input.time_usec_(some_number);
			pgps_input.gps_id_(some_number);
			
			pgps_input.ignore_flags_(_Host.GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
			
			pgps_input.time_week_ms_(some_number);
			pgps_input.time_week_(some_number);
			pgps_input.fix_type_(some_number);
			pgps_input.lat_(some_number);
			pgps_input.lon_(some_number);
			pgps_input.alt_(some_number);
			pgps_input.hdop_(some_number);
			pgps_input.vdop_(some_number);
			pgps_input.vn_(some_number);
			pgps_input.ve_(some_number);
			pgps_input.vd_(some_number);
			pgps_input.speed_accuracy_(some_number);
			pgps_input.horiz_accuracy_(some_number);
			pgps_input.vert_accuracy_(some_number);
			pgps_input.satellites_visible_(some_number);
			
		}
		
		export function onCOMMAND_LONG(pcommand_long: _Host.COMMAND_LONG) {
			const some_target_system    = pcommand_long.target_system();
			const some_target_component = pcommand_long.target_component();
			{
				
				const item = pcommand_long.command();
				if (item !== null)
					_Host.MAV_CMD.MAV_CMD_NAV_WAYPOINT = item;
				
			}
			const some_confirmation = pcommand_long.confirmation();
			const some_param1       = pcommand_long.param1();
			const some_param2       = pcommand_long.param2();
			const some_param3       = pcommand_long.param3();
			const some_param4       = pcommand_long.param4();
			const some_param5       = pcommand_long.param5();
			const some_param6       = pcommand_long.param6();
			const some_param7       = pcommand_long.param7();
			
		}
		
		export function onCOMPASSMOT_STATUS(pcompassmot_status: _Host.COMPASSMOT_STATUS) {
			const some_throttle      = pcompassmot_status.throttle();
			const some_current       = pcompassmot_status.current();
			const some_interference  = pcompassmot_status.interference();
			const some_CompensationX = pcompassmot_status.CompensationX();
			const some_CompensationY = pcompassmot_status.CompensationY();
			const some_CompensationZ = pcompassmot_status.CompensationZ();
			
		}
		
		export function fill_COMPASSMOT_STATUS(pcompassmot_status: _Host.COMPASSMOT_STATUS) {
			pcompassmot_status.throttle_(some_number);
			pcompassmot_status.current_(some_number);
			pcompassmot_status.interference_(some_number);
			pcompassmot_status.CompensationX_(some_number);
			pcompassmot_status.CompensationY_(some_number);
			pcompassmot_status.CompensationZ_(some_number);
			
		}
		
		export function onLOG_REQUEST_DATA(plog_request_data: _Host.LOG_REQUEST_DATA) {
			const some_target_system    = plog_request_data.target_system();
			const some_target_component = plog_request_data.target_component();
			const some_id               = plog_request_data.id();
			const some_ofs              = plog_request_data.ofs();
			const some_count            = plog_request_data.count();
			
		}
		
		export function fill_LOG_REQUEST_DATA(plog_request_data: _Host.LOG_REQUEST_DATA) {
			plog_request_data.target_system_(some_number);
			plog_request_data.target_component_(some_number);
			plog_request_data.id_(some_number);
			plog_request_data.ofs_(some_number);
			plog_request_data.count_(some_number);
			
		}
		
		export function onGPS_RAW_INT(pgps_raw_int: _Host.GPS_RAW_INT) {
			const some_time_usec = pgps_raw_int.time_usec();
			{
				
				const item = pgps_raw_int.fix_type();
				if (item !== null)
					_Host.GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS = item;
				
			}
			const some_lat                = pgps_raw_int.lat();
			const some_lon                = pgps_raw_int.lon();
			const some_alt                = pgps_raw_int.alt();
			const some_eph                = pgps_raw_int.eph();
			const some_epv                = pgps_raw_int.epv();
			const some_vel                = pgps_raw_int.vel();
			const some_cog                = pgps_raw_int.cog();
			const some_satellites_visible = pgps_raw_int.satellites_visible();
			{
				
				const item = pgps_raw_int.alt_ellipsoid();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pgps_raw_int.h_acc();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pgps_raw_int.v_acc();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pgps_raw_int.vel_acc();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pgps_raw_int.hdg_acc();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function onCAMERA_STATUS(pcamera_status: _Host.CAMERA_STATUS) {
			const some_time_usec     = pcamera_status.time_usec();
			const some_target_system = pcamera_status.target_system();
			const some_cam_idx       = pcamera_status.cam_idx();
			const some_img_idx       = pcamera_status.img_idx();
			{
				
				const item = pcamera_status.event_id();
				if (item !== null)
					_Host.CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_HEARTBEAT = item;
				
			}
			const some_p1 = pcamera_status.p1();
			const some_p2 = pcamera_status.p2();
			const some_p3 = pcamera_status.p3();
			const some_p4 = pcamera_status.p4();
			
		}
		
		export function fill_CAMERA_STATUS(pcamera_status: _Host.CAMERA_STATUS) {
			pcamera_status.time_usec_(some_number);
			pcamera_status.target_system_(some_number);
			pcamera_status.cam_idx_(some_number);
			pcamera_status.img_idx_(some_number);
			
			pcamera_status.event_id_(_Host.CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_HEARTBEAT);
			
			pcamera_status.p1_(some_number);
			pcamera_status.p2_(some_number);
			pcamera_status.p3_(some_number);
			pcamera_status.p4_(some_number);
			
		}
		
		export function onRC_CHANNELS_SCALED(prc_channels_scaled: _Host.RC_CHANNELS_SCALED) {
			const some_time_boot_ms = prc_channels_scaled.time_boot_ms();
			const some_port         = prc_channels_scaled.port();
			const some_chan1_scaled = prc_channels_scaled.chan1_scaled();
			const some_chan2_scaled = prc_channels_scaled.chan2_scaled();
			const some_chan3_scaled = prc_channels_scaled.chan3_scaled();
			const some_chan4_scaled = prc_channels_scaled.chan4_scaled();
			const some_chan5_scaled = prc_channels_scaled.chan5_scaled();
			const some_chan6_scaled = prc_channels_scaled.chan6_scaled();
			const some_chan7_scaled = prc_channels_scaled.chan7_scaled();
			const some_chan8_scaled = prc_channels_scaled.chan8_scaled();
			const some_rssi         = prc_channels_scaled.rssi();
			
		}
		
		export function onCAMERA_SETTINGS(pcamera_settings: _Host.CAMERA_SETTINGS) {
			const some_time_boot_ms = pcamera_settings.time_boot_ms();
			{
				
				const item = pcamera_settings.mode_id();
				if (item !== null)
					_Host.CAMERA_MODE.CAMERA_MODE_IMAGE = item;
				
			}
			
		}
		
		export function fill_CAMERA_SETTINGS(pcamera_settings: _Host.CAMERA_SETTINGS) {
			pcamera_settings.time_boot_ms_(some_number);
			
			pcamera_settings.mode_id_(_Host.CAMERA_MODE.CAMERA_MODE_IMAGE);
			
			
		}
		
		export function onDEVICE_OP_READ_REPLY(pdevice_op_read_reply: _Host.DEVICE_OP_READ_REPLY) {
			const some_request_id = pdevice_op_read_reply.request_id();
			const some_result     = pdevice_op_read_reply.result();
			const some_regstart   = pdevice_op_read_reply.regstart();
			const some_count      = pdevice_op_read_reply.count();
			{
				const item = pdevice_op_read_reply.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_DEVICE_OP_READ_REPLY(pdevice_op_read_reply: _Host.DEVICE_OP_READ_REPLY) {
			pdevice_op_read_reply.request_id_(some_number);
			pdevice_op_read_reply.result_(some_number);
			pdevice_op_read_reply.regstart_(some_number);
			pdevice_op_read_reply.count_(some_number);
			{
				const item = pdevice_op_read_reply.daTa();
				
				for (let i = 0; i < _Host.DEVICE_OP_READ_REPLY.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onRAW_PRESSURE(praw_pressure: _Host.RAW_PRESSURE) {
			const some_time_usec   = praw_pressure.time_usec();
			const some_press_abs   = praw_pressure.press_abs();
			const some_press_diff1 = praw_pressure.press_diff1();
			const some_press_diff2 = praw_pressure.press_diff2();
			const some_temperature = praw_pressure.temperature();
			
		}
		
		export function onDIGICAM_CONTROL(pdigicam_control: _Host.DIGICAM_CONTROL) {
			const some_target_system    = pdigicam_control.target_system();
			const some_target_component = pdigicam_control.target_component();
			const some_session          = pdigicam_control.session();
			const some_zoom_pos         = pdigicam_control.zoom_pos();
			const some_zoom_step        = pdigicam_control.zoom_step();
			const some_focus_lock       = pdigicam_control.focus_lock();
			const some_shot             = pdigicam_control.shot();
			const some_command_id       = pdigicam_control.command_id();
			const some_extra_param      = pdigicam_control.extra_param();
			const some_extra_value      = pdigicam_control.extra_value();
			
		}
		
		export function fill_DIGICAM_CONTROL(pdigicam_control: _Host.DIGICAM_CONTROL) {
			pdigicam_control.target_system_(some_number);
			pdigicam_control.target_component_(some_number);
			pdigicam_control.session_(some_number);
			pdigicam_control.zoom_pos_(some_number);
			pdigicam_control.zoom_step_(some_number);
			pdigicam_control.focus_lock_(some_number);
			pdigicam_control.shot_(some_number);
			pdigicam_control.command_id_(some_number);
			pdigicam_control.extra_param_(some_number);
			pdigicam_control.extra_value_(some_number);
			
		}
		
		export function onNAMED_VALUE_FLOAT(pnamed_value_float: _Host.NAMED_VALUE_FLOAT) {
			const some_time_boot_ms = pnamed_value_float.time_boot_ms();
			{
				
				const item = pnamed_value_float.name();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_value = pnamed_value_float.value();
			
		}
		
		export function fill_NAMED_VALUE_FLOAT(pnamed_value_float: _Host.NAMED_VALUE_FLOAT) {
			pnamed_value_float.time_boot_ms_(some_number);
			pnamed_value_float.name_(some_string);
			pnamed_value_float.value_(some_number);
			
		}
		
		export function onGOPRO_HEARTBEAT(pgopro_heartbeat: _Host.GOPRO_HEARTBEAT) {
			{
				
				const item = pgopro_heartbeat.status();
				if (item !== null)
					_Host.GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_DISCONNECTED = item;
				
			}
			{
				
				const item = pgopro_heartbeat.capture_mode();
				if (item !== null)
					_Host.GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO = item;
				
			}
			{
				
				const item = pgopro_heartbeat.flags();
				if (item !== null)
					_Host.GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING = item;
				
			}
			
		}
		
		export function fill_GOPRO_HEARTBEAT(pgopro_heartbeat: _Host.GOPRO_HEARTBEAT) {
			
			pgopro_heartbeat.status_(_Host.GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_DISCONNECTED);
			
			
			pgopro_heartbeat.capture_mode_(_Host.GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO);
			
			
			pgopro_heartbeat.flags_(_Host.GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
			
			
		}
		
		export function onATTITUDE(pattitude: _Host.ATTITUDE) {
			const some_time_boot_ms = pattitude.time_boot_ms();
			const some_roll         = pattitude.roll();
			const some_pitch        = pattitude.pitch();
			const some_yaw          = pattitude.yaw();
			const some_rollspeed    = pattitude.rollspeed();
			const some_pitchspeed   = pattitude.pitchspeed();
			const some_yawspeed     = pattitude.yawspeed();
			
		}
		
		export function onMISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list: _Host.MISSION_WRITE_PARTIAL_LIST) {
			const some_target_system    = pmission_write_partial_list.target_system();
			const some_target_component = pmission_write_partial_list.target_component();
			const some_start_index      = pmission_write_partial_list.start_index();
			const some_end_index        = pmission_write_partial_list.end_index();
			{
				
				const item = pmission_write_partial_list.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onAHRS2(pahrs2: _Host.AHRS2) {
			const some_roll     = pahrs2.roll();
			const some_pitch    = pahrs2.pitch();
			const some_yaw      = pahrs2.yaw();
			const some_altitude = pahrs2.altitude();
			const some_lat      = pahrs2.lat();
			const some_lng      = pahrs2.lng();
			
		}
		
		export function fill_AHRS2(pahrs2: _Host.AHRS2) {
			pahrs2.roll_(some_number);
			pahrs2.pitch_(some_number);
			pahrs2.yaw_(some_number);
			pahrs2.altitude_(some_number);
			pahrs2.lat_(some_number);
			pahrs2.lng_(some_number);
			
		}
		
		export function onLOG_ERASE(plog_erase: _Host.LOG_ERASE) {
			const some_target_system    = plog_erase.target_system();
			const some_target_component = plog_erase.target_component();
			
		}
		
		export function fill_LOG_ERASE(plog_erase: _Host.LOG_ERASE) {
			plog_erase.target_system_(some_number);
			plog_erase.target_component_(some_number);
			
		}
		
		export function onTERRAIN_REQUEST(pterrain_request: _Host.TERRAIN_REQUEST) {
			const some_lat          = pterrain_request.lat();
			const some_lon          = pterrain_request.lon();
			const some_grid_spacing = pterrain_request.grid_spacing();
			const some_mask         = pterrain_request.mask();
			
		}
		
		export function fill_TERRAIN_REQUEST(pterrain_request: _Host.TERRAIN_REQUEST) {
			pterrain_request.lat_(some_number);
			pterrain_request.lon_(some_number);
			pterrain_request.grid_spacing_(some_number);
			pterrain_request.mask_(some_number);
			
		}
		
		export function onMOUNT_STATUS(pmount_status: _Host.MOUNT_STATUS) {
			const some_target_system    = pmount_status.target_system();
			const some_target_component = pmount_status.target_component();
			const some_pointing_a       = pmount_status.pointing_a();
			const some_pointing_b       = pmount_status.pointing_b();
			const some_pointing_c       = pmount_status.pointing_c();
			
		}
		
		export function fill_MOUNT_STATUS(pmount_status: _Host.MOUNT_STATUS) {
			pmount_status.target_system_(some_number);
			pmount_status.target_component_(some_number);
			pmount_status.pointing_a_(some_number);
			pmount_status.pointing_b_(some_number);
			pmount_status.pointing_c_(some_number);
			
		}
		
		export function onMANUAL_SETPOINT(pmanual_setpoint: _Host.MANUAL_SETPOINT) {
			const some_time_boot_ms           = pmanual_setpoint.time_boot_ms();
			const some_roll                   = pmanual_setpoint.roll();
			const some_pitch                  = pmanual_setpoint.pitch();
			const some_yaw                    = pmanual_setpoint.yaw();
			const some_thrust                 = pmanual_setpoint.thrust();
			const some_mode_switch            = pmanual_setpoint.mode_switch();
			const some_manual_override_switch = pmanual_setpoint.manual_override_switch();
			
		}
		
		export function onPID_TUNING(ppid_tuning: _Host.PID_TUNING) {
			{
				
				const item = ppid_tuning.axis();
				if (item !== null)
					_Host.PID_TUNING_AXIS.PID_TUNING_ROLL = item;
				
			}
			const some_desired  = ppid_tuning.desired();
			const some_achieved = ppid_tuning.achieved();
			const some_FF       = ppid_tuning.FF();
			const some_P        = ppid_tuning.P();
			const some_I        = ppid_tuning.I();
			const some_D        = ppid_tuning.D();
			
		}
		
		export function fill_PID_TUNING(ppid_tuning: _Host.PID_TUNING) {
			
			ppid_tuning.axis_(_Host.PID_TUNING_AXIS.PID_TUNING_ROLL);
			
			ppid_tuning.desired_(some_number);
			ppid_tuning.achieved_(some_number);
			ppid_tuning.FF_(some_number);
			ppid_tuning.P_(some_number);
			ppid_tuning.I_(some_number);
			ppid_tuning.D_(some_number);
			
		}
		
		export function onSAFETY_ALLOWED_AREA(psafety_allowed_area: _Host.SAFETY_ALLOWED_AREA) {
			{
				
				const item = psafety_allowed_area.frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			const some_p1x = psafety_allowed_area.p1x();
			const some_p1y = psafety_allowed_area.p1y();
			const some_p1z = psafety_allowed_area.p1z();
			const some_p2x = psafety_allowed_area.p2x();
			const some_p2y = psafety_allowed_area.p2y();
			const some_p2z = psafety_allowed_area.p2z();
			
		}
		
		export function onOPTICAL_FLOW_RAD(poptical_flow_rad: _Host.OPTICAL_FLOW_RAD) {
			const some_time_usec              = poptical_flow_rad.time_usec();
			const some_sensor_id              = poptical_flow_rad.sensor_id();
			const some_integration_time_us    = poptical_flow_rad.integration_time_us();
			const some_integrated_x           = poptical_flow_rad.integrated_x();
			const some_integrated_y           = poptical_flow_rad.integrated_y();
			const some_integrated_xgyro       = poptical_flow_rad.integrated_xgyro();
			const some_integrated_ygyro       = poptical_flow_rad.integrated_ygyro();
			const some_integrated_zgyro       = poptical_flow_rad.integrated_zgyro();
			const some_temperature            = poptical_flow_rad.temperature();
			const some_quality                = poptical_flow_rad.quality();
			const some_time_delta_distance_us = poptical_flow_rad.time_delta_distance_us();
			const some_distance               = poptical_flow_rad.distance();
			
		}
		
		export function fill_OPTICAL_FLOW_RAD(poptical_flow_rad: _Host.OPTICAL_FLOW_RAD) {
			poptical_flow_rad.time_usec_(some_number);
			poptical_flow_rad.sensor_id_(some_number);
			poptical_flow_rad.integration_time_us_(some_number);
			poptical_flow_rad.integrated_x_(some_number);
			poptical_flow_rad.integrated_y_(some_number);
			poptical_flow_rad.integrated_xgyro_(some_number);
			poptical_flow_rad.integrated_ygyro_(some_number);
			poptical_flow_rad.integrated_zgyro_(some_number);
			poptical_flow_rad.temperature_(some_number);
			poptical_flow_rad.quality_(some_number);
			poptical_flow_rad.time_delta_distance_us_(some_number);
			poptical_flow_rad.distance_(some_number);
			
		}
		
		export function onLOG_DATA(plog_data: _Host.LOG_DATA) {
			const some_id    = plog_data.id();
			const some_ofs   = plog_data.ofs();
			const some_count = plog_data.count();
			{
				const item = plog_data.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_LOG_DATA(plog_data: _Host.LOG_DATA) {
			plog_data.id_(some_number);
			plog_data.ofs_(some_number);
			plog_data.count_(some_number);
			{
				const item = plog_data.daTa();
				
				for (let i = 0; i < _Host.LOG_DATA.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onMISSION_CLEAR_ALL(pmission_clear_all: _Host.MISSION_CLEAR_ALL) {
			const some_target_system    = pmission_clear_all.target_system();
			const some_target_component = pmission_clear_all.target_component();
			{
				
				const item = pmission_clear_all.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onAHRS3(pahrs3: _Host.AHRS3) {
			const some_roll     = pahrs3.roll();
			const some_pitch    = pahrs3.pitch();
			const some_yaw      = pahrs3.yaw();
			const some_altitude = pahrs3.altitude();
			const some_lat      = pahrs3.lat();
			const some_lng      = pahrs3.lng();
			const some_v1       = pahrs3.v1();
			const some_v2       = pahrs3.v2();
			const some_v3       = pahrs3.v3();
			const some_v4       = pahrs3.v4();
			
		}
		
		export function fill_AHRS3(pahrs3: _Host.AHRS3) {
			pahrs3.roll_(some_number);
			pahrs3.pitch_(some_number);
			pahrs3.yaw_(some_number);
			pahrs3.altitude_(some_number);
			pahrs3.lat_(some_number);
			pahrs3.lng_(some_number);
			pahrs3.v1_(some_number);
			pahrs3.v2_(some_number);
			pahrs3.v3_(some_number);
			pahrs3.v4_(some_number);
			
		}
		
		export function onVICON_POSITION_ESTIMATE(pvicon_position_estimate: _Host.VICON_POSITION_ESTIMATE) {
			const some_usec  = pvicon_position_estimate.usec();
			const some_x     = pvicon_position_estimate.x();
			const some_y     = pvicon_position_estimate.y();
			const some_z     = pvicon_position_estimate.z();
			const some_roll  = pvicon_position_estimate.roll();
			const some_pitch = pvicon_position_estimate.pitch();
			const some_yaw   = pvicon_position_estimate.yaw();
			
		}
		
		export function fill_VICON_POSITION_ESTIMATE(pvicon_position_estimate: _Host.VICON_POSITION_ESTIMATE) {
			pvicon_position_estimate.usec_(some_number);
			pvicon_position_estimate.x_(some_number);
			pvicon_position_estimate.y_(some_number);
			pvicon_position_estimate.z_(some_number);
			pvicon_position_estimate.roll_(some_number);
			pvicon_position_estimate.pitch_(some_number);
			pvicon_position_estimate.yaw_(some_number);
			
		}
		
		export function onGPS2_RTK(pgps2_rtk: _Host.GPS2_RTK) {
			const some_time_last_baseline_ms = pgps2_rtk.time_last_baseline_ms();
			const some_rtk_receiver_id       = pgps2_rtk.rtk_receiver_id();
			const some_wn                    = pgps2_rtk.wn();
			const some_tow                   = pgps2_rtk.tow();
			const some_rtk_health            = pgps2_rtk.rtk_health();
			const some_rtk_rate              = pgps2_rtk.rtk_rate();
			const some_nsats                 = pgps2_rtk.nsats();
			const some_baseline_coords_type  = pgps2_rtk.baseline_coords_type();
			const some_baseline_a_mm         = pgps2_rtk.baseline_a_mm();
			const some_baseline_b_mm         = pgps2_rtk.baseline_b_mm();
			const some_baseline_c_mm         = pgps2_rtk.baseline_c_mm();
			const some_accuracy              = pgps2_rtk.accuracy();
			const some_iar_num_hypotheses    = pgps2_rtk.iar_num_hypotheses();
			
		}
		
		export function fill_GPS2_RTK(pgps2_rtk: _Host.GPS2_RTK) {
			pgps2_rtk.time_last_baseline_ms_(some_number);
			pgps2_rtk.rtk_receiver_id_(some_number);
			pgps2_rtk.wn_(some_number);
			pgps2_rtk.tow_(some_number);
			pgps2_rtk.rtk_health_(some_number);
			pgps2_rtk.rtk_rate_(some_number);
			pgps2_rtk.nsats_(some_number);
			pgps2_rtk.baseline_coords_type_(some_number);
			pgps2_rtk.baseline_a_mm_(some_number);
			pgps2_rtk.baseline_b_mm_(some_number);
			pgps2_rtk.baseline_c_mm_(some_number);
			pgps2_rtk.accuracy_(some_number);
			pgps2_rtk.iar_num_hypotheses_(some_number);
			
		}
		
		export function onMAG_CAL_REPORT(pmag_cal_report: _Host.MAG_CAL_REPORT) {
			const some_compass_id = pmag_cal_report.compass_id();
			const some_cal_mask   = pmag_cal_report.cal_mask();
			{
				
				const item = pmag_cal_report.cal_status();
				if (item !== null)
					_Host.MAG_CAL_STATUS.MAG_CAL_NOT_STARTED = item;
				
			}
			const some_autosaved = pmag_cal_report.autosaved();
			const some_fitness   = pmag_cal_report.fitness();
			const some_ofs_x     = pmag_cal_report.ofs_x();
			const some_ofs_y     = pmag_cal_report.ofs_y();
			const some_ofs_z     = pmag_cal_report.ofs_z();
			const some_diag_x    = pmag_cal_report.diag_x();
			const some_diag_y    = pmag_cal_report.diag_y();
			const some_diag_z    = pmag_cal_report.diag_z();
			const some_offdiag_x = pmag_cal_report.offdiag_x();
			const some_offdiag_y = pmag_cal_report.offdiag_y();
			const some_offdiag_z = pmag_cal_report.offdiag_z();
			
		}
		
		export function fill_MAG_CAL_REPORT(pmag_cal_report: _Host.MAG_CAL_REPORT) {
			pmag_cal_report.compass_id_(some_number);
			pmag_cal_report.cal_mask_(some_number);
			
			pmag_cal_report.cal_status_(_Host.MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
			
			pmag_cal_report.autosaved_(some_number);
			pmag_cal_report.fitness_(some_number);
			pmag_cal_report.ofs_x_(some_number);
			pmag_cal_report.ofs_y_(some_number);
			pmag_cal_report.ofs_z_(some_number);
			pmag_cal_report.diag_x_(some_number);
			pmag_cal_report.diag_y_(some_number);
			pmag_cal_report.diag_z_(some_number);
			pmag_cal_report.offdiag_x_(some_number);
			pmag_cal_report.offdiag_y_(some_number);
			pmag_cal_report.offdiag_z_(some_number);
			
		}
		
		export function onLOG_REQUEST_LIST(plog_request_list: _Host.LOG_REQUEST_LIST) {
			const some_target_system    = plog_request_list.target_system();
			const some_target_component = plog_request_list.target_component();
			const some_start            = plog_request_list.start();
			const some_end              = plog_request_list.end();
			
		}
		
		export function fill_LOG_REQUEST_LIST(plog_request_list: _Host.LOG_REQUEST_LIST) {
			plog_request_list.target_system_(some_number);
			plog_request_list.target_component_(some_number);
			plog_request_list.start_(some_number);
			plog_request_list.end_(some_number);
			
		}
		
		export function onSCALED_PRESSURE(pscaled_pressure: _Host.SCALED_PRESSURE) {
			const some_time_boot_ms = pscaled_pressure.time_boot_ms();
			const some_press_abs    = pscaled_pressure.press_abs();
			const some_press_diff   = pscaled_pressure.press_diff();
			const some_temperature  = pscaled_pressure.temperature();
			
		}
		
		export function onV2_EXTENSION(pv2_extension: _Host.V2_EXTENSION) {
			const some_target_network   = pv2_extension.target_network();
			const some_target_system    = pv2_extension.target_system();
			const some_target_component = pv2_extension.target_component();
			const some_message_type     = pv2_extension.message_type();
			{
				const item = pv2_extension.payload();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_V2_EXTENSION(pv2_extension: _Host.V2_EXTENSION) {
			pv2_extension.target_network_(some_number);
			pv2_extension.target_system_(some_number);
			pv2_extension.target_component_(some_number);
			pv2_extension.message_type_(some_number);
			{
				const item = pv2_extension.payload();
				
				for (let i = 0; i < _Host.V2_EXTENSION.payload.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onHEARTBEAT(pheartbeat: _Host.HEARTBEAT) {
			{
				
				const item = pheartbeat.typE();
				if (item !== null)
					_Host.MAV_TYPE.GENERIC = item;
				
			}
			{
				
				const item = pheartbeat.autopilot();
				if (item !== null)
					_Host.MAV_AUTOPILOT.GENERIC = item;
				
			}
			{
				
				const item = pheartbeat.base_mode();
				if (item !== null)
					_Host.MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = item;
				
			}
			const some_custom_mode = pheartbeat.custom_mode();
			{
				
				const item = pheartbeat.system_status();
				if (item !== null)
					_Host.MAV_STATE.UNINIT = item;
				
			}
			const some_mavlink_version = pheartbeat.mavlink_version();
			
		}
		
		export function onPARAM_MAP_RC(pparam_map_rc: _Host.PARAM_MAP_RC) {
			const some_target_system    = pparam_map_rc.target_system();
			const some_target_component = pparam_map_rc.target_component();
			{
				
				const item = pparam_map_rc.param_id();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_param_index                = pparam_map_rc.param_index();
			const some_parameter_rc_channel_index = pparam_map_rc.parameter_rc_channel_index();
			const some_param_value0               = pparam_map_rc.param_value0();
			const some_scale                      = pparam_map_rc.scale();
			const some_param_value_min            = pparam_map_rc.param_value_min();
			const some_param_value_max            = pparam_map_rc.param_value_max();
			
		}
		
		export function onPOWER_STATUS(ppower_status: _Host.POWER_STATUS) {
			const some_Vcc    = ppower_status.Vcc();
			const some_Vservo = ppower_status.Vservo();
			{
				
				const item = ppower_status.flags();
				if (item !== null)
					_Host.MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID = item;
				
			}
			
		}
		
		export function fill_POWER_STATUS(ppower_status: _Host.POWER_STATUS) {
			ppower_status.Vcc_(some_number);
			ppower_status.Vservo_(some_number);
			
			ppower_status.flags_(_Host.MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
			
			
		}
		
		export function onREMOTE_LOG_DATA_BLOCK(premote_log_data_block: _Host.REMOTE_LOG_DATA_BLOCK) {
			const some_target_system    = premote_log_data_block.target_system();
			const some_target_component = premote_log_data_block.target_component();
			{
				
				const item = premote_log_data_block.seqno();
				if (item !== null)
					_Host.MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP = item;
				
			}
			{
				const item = premote_log_data_block.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_REMOTE_LOG_DATA_BLOCK(premote_log_data_block: _Host.REMOTE_LOG_DATA_BLOCK) {
			premote_log_data_block.target_system_(some_number);
			premote_log_data_block.target_component_(some_number);
			
			premote_log_data_block.seqno_(_Host.MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
			
			{
				const item = premote_log_data_block.daTa();
				
				for (let i = 0; i < _Host.REMOTE_LOG_DATA_BLOCK.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onLOGGING_DATA_ACKED(plogging_data_acked: _Host.LOGGING_DATA_ACKED) {
			const some_target_system        = plogging_data_acked.target_system();
			const some_target_component     = plogging_data_acked.target_component();
			const some_sequence             = plogging_data_acked.sequence();
			const some_length               = plogging_data_acked.length();
			const some_first_message_offset = plogging_data_acked.first_message_offset();
			{
				const item = plogging_data_acked.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_LOGGING_DATA_ACKED(plogging_data_acked: _Host.LOGGING_DATA_ACKED) {
			plogging_data_acked.target_system_(some_number);
			plogging_data_acked.target_component_(some_number);
			plogging_data_acked.sequence_(some_number);
			plogging_data_acked.length_(some_number);
			plogging_data_acked.first_message_offset_(some_number);
			{
				const item = plogging_data_acked.daTa();
				
				for (let i = 0; i < _Host.LOGGING_DATA_ACKED.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onTERRAIN_CHECK(pterrain_check: _Host.TERRAIN_CHECK) {
			const some_lat = pterrain_check.lat();
			const some_lon = pterrain_check.lon();
			
		}
		
		export function fill_TERRAIN_CHECK(pterrain_check: _Host.TERRAIN_CHECK) {
			pterrain_check.lat_(some_number);
			pterrain_check.lon_(some_number);
			
		}
		
		export function onMOUNT_CONFIGURE(pmount_configure: _Host.MOUNT_CONFIGURE) {
			const some_target_system    = pmount_configure.target_system();
			const some_target_component = pmount_configure.target_component();
			{
				
				const item = pmount_configure.mount_mode();
				if (item !== null)
					_Host.MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT = item;
				
			}
			const some_stab_roll  = pmount_configure.stab_roll();
			const some_stab_pitch = pmount_configure.stab_pitch();
			const some_stab_yaw   = pmount_configure.stab_yaw();
			
		}
		
		export function fill_MOUNT_CONFIGURE(pmount_configure: _Host.MOUNT_CONFIGURE) {
			pmount_configure.target_system_(some_number);
			pmount_configure.target_component_(some_number);
			
			pmount_configure.mount_mode_(_Host.MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT);
			
			pmount_configure.stab_roll_(some_number);
			pmount_configure.stab_pitch_(some_number);
			pmount_configure.stab_yaw_(some_number);
			
		}
		
		export function onMISSION_REQUEST_INT(pmission_request_int: _Host.MISSION_REQUEST_INT) {
			const some_target_system    = pmission_request_int.target_system();
			const some_target_component = pmission_request_int.target_component();
			const some_seq              = pmission_request_int.seq();
			{
				
				const item = pmission_request_int.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset: _Host.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) {
			const some_time_boot_ms = plocal_position_ned_system_global_offset.time_boot_ms();
			const some_x            = plocal_position_ned_system_global_offset.x();
			const some_y            = plocal_position_ned_system_global_offset.y();
			const some_z            = plocal_position_ned_system_global_offset.z();
			const some_roll         = plocal_position_ned_system_global_offset.roll();
			const some_pitch        = plocal_position_ned_system_global_offset.pitch();
			const some_yaw          = plocal_position_ned_system_global_offset.yaw();
			
		}
		
		export function onCOMMAND_ACK(pcommand_ack: _Host.COMMAND_ACK) {
			{
				
				const item = pcommand_ack.command();
				if (item !== null)
					_Host.MAV_CMD.MAV_CMD_NAV_WAYPOINT = item;
				
			}
			{
				
				const item = pcommand_ack.result();
				if (item !== null)
					_Host.MAV_RESULT.MAV_RESULT_ACCEPTED = item;
				
			}
			{
				
				const item = pcommand_ack.progress();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pcommand_ack.result_param2();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pcommand_ack.target_system();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pcommand_ack.target_component();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function onDATA_STREAM(pdata_stream: _Host.DATA_STREAM) {
			const some_stream_id    = pdata_stream.stream_id();
			const some_message_rate = pdata_stream.message_rate();
			const some_on_off       = pdata_stream.on_off();
			
		}
		
		export function onMISSION_REQUEST(pmission_request: _Host.MISSION_REQUEST) {
			const some_target_system    = pmission_request.target_system();
			const some_target_component = pmission_request.target_component();
			const some_seq              = pmission_request.seq();
			{
				
				const item = pmission_request.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onTERRAIN_REPORT(pterrain_report: _Host.TERRAIN_REPORT) {
			const some_lat            = pterrain_report.lat();
			const some_lon            = pterrain_report.lon();
			const some_spacing        = pterrain_report.spacing();
			const some_terrain_height = pterrain_report.terrain_height();
			const some_current_height = pterrain_report.current_height();
			const some_pending        = pterrain_report.pending();
			const some_loaded         = pterrain_report.loaded();
			
		}
		
		export function fill_TERRAIN_REPORT(pterrain_report: _Host.TERRAIN_REPORT) {
			pterrain_report.lat_(some_number);
			pterrain_report.lon_(some_number);
			pterrain_report.spacing_(some_number);
			pterrain_report.terrain_height_(some_number);
			pterrain_report.current_height_(some_number);
			pterrain_report.pending_(some_number);
			pterrain_report.loaded_(some_number);
			
		}
		
		export function onSET_HOME_POSITION(pset_home_position: _Host.SET_HOME_POSITION) {
			const some_target_system = pset_home_position.target_system();
			const some_latitude      = pset_home_position.latitude();
			const some_longitude     = pset_home_position.longitude();
			const some_altitude      = pset_home_position.altitude();
			const some_x             = pset_home_position.x();
			const some_y             = pset_home_position.y();
			const some_z             = pset_home_position.z();
			{
				const item = pset_home_position.q();
				for (let value of item)
					some_number = value;
				
			}
			const some_approach_x = pset_home_position.approach_x();
			const some_approach_y = pset_home_position.approach_y();
			const some_approach_z = pset_home_position.approach_z();
			{
				
				const item = pset_home_position.time_usec();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function fill_SET_HOME_POSITION(pset_home_position: _Host.SET_HOME_POSITION) {
			pset_home_position.target_system_(some_number);
			pset_home_position.latitude_(some_number);
			pset_home_position.longitude_(some_number);
			pset_home_position.altitude_(some_number);
			pset_home_position.x_(some_number);
			pset_home_position.y_(some_number);
			pset_home_position.z_(some_number);
			{
				const item = pset_home_position.q();
				
				for (let i = 0; i < _Host.SET_HOME_POSITION.q.item_len; i++)
					item.set(some_number, i);
				
			}
			pset_home_position.approach_x_(some_number);
			pset_home_position.approach_y_(some_number);
			pset_home_position.approach_z_(some_number);
			
			pset_home_position.time_usec_(some_number);
			
			
		}
		
		export function onSwitchModeCommand() {}
		
		export function onHIL_RC_INPUTS_RAW(phil_rc_inputs_raw: _Host.HIL_RC_INPUTS_RAW) {
			const some_time_usec  = phil_rc_inputs_raw.time_usec();
			const some_chan1_raw  = phil_rc_inputs_raw.chan1_raw();
			const some_chan2_raw  = phil_rc_inputs_raw.chan2_raw();
			const some_chan3_raw  = phil_rc_inputs_raw.chan3_raw();
			const some_chan4_raw  = phil_rc_inputs_raw.chan4_raw();
			const some_chan5_raw  = phil_rc_inputs_raw.chan5_raw();
			const some_chan6_raw  = phil_rc_inputs_raw.chan6_raw();
			const some_chan7_raw  = phil_rc_inputs_raw.chan7_raw();
			const some_chan8_raw  = phil_rc_inputs_raw.chan8_raw();
			const some_chan9_raw  = phil_rc_inputs_raw.chan9_raw();
			const some_chan10_raw = phil_rc_inputs_raw.chan10_raw();
			const some_chan11_raw = phil_rc_inputs_raw.chan11_raw();
			const some_chan12_raw = phil_rc_inputs_raw.chan12_raw();
			const some_rssi       = phil_rc_inputs_raw.rssi();
			
		}
		
		export function onSCALED_IMU3(pscaled_imu3: _Host.SCALED_IMU3) {
			const some_time_boot_ms = pscaled_imu3.time_boot_ms();
			const some_xacc         = pscaled_imu3.xacc();
			const some_yacc         = pscaled_imu3.yacc();
			const some_zacc         = pscaled_imu3.zacc();
			const some_xgyro        = pscaled_imu3.xgyro();
			const some_ygyro        = pscaled_imu3.ygyro();
			const some_zgyro        = pscaled_imu3.zgyro();
			const some_xmag         = pscaled_imu3.xmag();
			const some_ymag         = pscaled_imu3.ymag();
			const some_zmag         = pscaled_imu3.zmag();
			
		}
		
		export function fill_SCALED_IMU3(pscaled_imu3: _Host.SCALED_IMU3) {
			pscaled_imu3.time_boot_ms_(some_number);
			pscaled_imu3.xacc_(some_number);
			pscaled_imu3.yacc_(some_number);
			pscaled_imu3.zacc_(some_number);
			pscaled_imu3.xgyro_(some_number);
			pscaled_imu3.ygyro_(some_number);
			pscaled_imu3.zgyro_(some_number);
			pscaled_imu3.xmag_(some_number);
			pscaled_imu3.ymag_(some_number);
			pscaled_imu3.zmag_(some_number);
			
		}
		
		export function onSET_MODE(pset_mode: _Host.SET_MODE) {
			const some_target_system = pset_mode.target_system();
			{
				
				const item = pset_mode.base_mode();
				if (item !== null)
					_Host.MAV_MODE.PREFLIGHT = item;
				
			}
			const some_custom_mode = pset_mode.custom_mode();
			
		}
		
		export function onMOUNT_CONTROL(pmount_control: _Host.MOUNT_CONTROL) {
			const some_target_system    = pmount_control.target_system();
			const some_target_component = pmount_control.target_component();
			const some_input_a          = pmount_control.input_a();
			const some_input_b          = pmount_control.input_b();
			const some_input_c          = pmount_control.input_c();
			const some_save_position    = pmount_control.save_position();
			
		}
		
		export function fill_MOUNT_CONTROL(pmount_control: _Host.MOUNT_CONTROL) {
			pmount_control.target_system_(some_number);
			pmount_control.target_component_(some_number);
			pmount_control.input_a_(some_number);
			pmount_control.input_b_(some_number);
			pmount_control.input_c_(some_number);
			pmount_control.save_position_(some_number);
			
		}
		
		export function onPOSITION_TARGET_GLOBAL_INT(pposition_target_global_int: _Host.POSITION_TARGET_GLOBAL_INT) {
			const some_time_boot_ms = pposition_target_global_int.time_boot_ms();
			{
				
				const item = pposition_target_global_int.coordinate_frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			const some_type_mask = pposition_target_global_int.type_mask();
			const some_lat_int   = pposition_target_global_int.lat_int();
			const some_lon_int   = pposition_target_global_int.lon_int();
			const some_alt       = pposition_target_global_int.alt();
			const some_vx        = pposition_target_global_int.vx();
			const some_vy        = pposition_target_global_int.vy();
			const some_vz        = pposition_target_global_int.vz();
			const some_afx       = pposition_target_global_int.afx();
			const some_afy       = pposition_target_global_int.afy();
			const some_afz       = pposition_target_global_int.afz();
			const some_yaw       = pposition_target_global_int.yaw();
			const some_yaw_rate  = pposition_target_global_int.yaw_rate();
			
		}
		
		export function onLED_CONTROL(pled_control: _Host.LED_CONTROL) {
			const some_target_system    = pled_control.target_system();
			const some_target_component = pled_control.target_component();
			const some_instance         = pled_control.instance();
			const some_pattern          = pled_control.pattern();
			const some_custom_len       = pled_control.custom_len();
			{
				const item = pled_control.custom_bytes();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_LED_CONTROL(pled_control: _Host.LED_CONTROL) {
			pled_control.target_system_(some_number);
			pled_control.target_component_(some_number);
			pled_control.instance_(some_number);
			pled_control.pattern_(some_number);
			pled_control.custom_len_(some_number);
			{
				const item = pled_control.custom_bytes();
				
				for (let i = 0; i < _Host.LED_CONTROL.custom_bytes.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onSIM_STATE(psim_state: _Host.SIM_STATE) {
			const some_q1           = psim_state.q1();
			const some_q2           = psim_state.q2();
			const some_q3           = psim_state.q3();
			const some_q4           = psim_state.q4();
			const some_roll         = psim_state.roll();
			const some_pitch        = psim_state.pitch();
			const some_yaw          = psim_state.yaw();
			const some_xacc         = psim_state.xacc();
			const some_yacc         = psim_state.yacc();
			const some_zacc         = psim_state.zacc();
			const some_xgyro        = psim_state.xgyro();
			const some_ygyro        = psim_state.ygyro();
			const some_zgyro        = psim_state.zgyro();
			const some_lat          = psim_state.lat();
			const some_lon          = psim_state.lon();
			const some_alt          = psim_state.alt();
			const some_std_dev_horz = psim_state.std_dev_horz();
			const some_std_dev_vert = psim_state.std_dev_vert();
			const some_vn           = psim_state.vn();
			const some_ve           = psim_state.ve();
			const some_vd           = psim_state.vd();
			
		}
		
		export function fill_SIM_STATE(psim_state: _Host.SIM_STATE) {
			psim_state.q1_(some_number);
			psim_state.q2_(some_number);
			psim_state.q3_(some_number);
			psim_state.q4_(some_number);
			psim_state.roll_(some_number);
			psim_state.pitch_(some_number);
			psim_state.yaw_(some_number);
			psim_state.xacc_(some_number);
			psim_state.yacc_(some_number);
			psim_state.zacc_(some_number);
			psim_state.xgyro_(some_number);
			psim_state.ygyro_(some_number);
			psim_state.zgyro_(some_number);
			psim_state.lat_(some_number);
			psim_state.lon_(some_number);
			psim_state.alt_(some_number);
			psim_state.std_dev_horz_(some_number);
			psim_state.std_dev_vert_(some_number);
			psim_state.vn_(some_number);
			psim_state.ve_(some_number);
			psim_state.vd_(some_number);
			
		}
		
		export function onWIFI_CONFIG_AP(pwifi_config_ap: _Host.WIFI_CONFIG_AP) {
			{
				
				const item = pwifi_config_ap.ssid();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = pwifi_config_ap.password();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function fill_WIFI_CONFIG_AP(pwifi_config_ap: _Host.WIFI_CONFIG_AP) {
			pwifi_config_ap.ssid_(some_string);
			pwifi_config_ap.password_(some_string);
			
		}
		
		export function onDATA96(pdata96: _Host.DATA96) {
			const some_typE = pdata96.typE();
			const some_len  = pdata96.len();
			{
				const item = pdata96.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_DATA96(pdata96: _Host.DATA96) {
			pdata96.typE_(some_number);
			pdata96.len_(some_number);
			{
				const item = pdata96.daTa();
				
				for (let i = 0; i < _Host.DATA96.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onFLIGHT_INFORMATION(pflight_information: _Host.FLIGHT_INFORMATION) {
			const some_time_boot_ms     = pflight_information.time_boot_ms();
			const some_arming_time_utc  = pflight_information.arming_time_utc();
			const some_takeoff_time_utc = pflight_information.takeoff_time_utc();
			const some_flight_uuid      = pflight_information.flight_uuid();
			
		}
		
		export function fill_FLIGHT_INFORMATION(pflight_information: _Host.FLIGHT_INFORMATION) {
			pflight_information.time_boot_ms_(some_number);
			pflight_information.arming_time_utc_(some_number);
			pflight_information.takeoff_time_utc_(some_number);
			pflight_information.flight_uuid_(some_number);
			
		}
		
		export function onRC_CHANNELS_RAW(prc_channels_raw: _Host.RC_CHANNELS_RAW) {
			const some_time_boot_ms = prc_channels_raw.time_boot_ms();
			const some_port         = prc_channels_raw.port();
			const some_chan1_raw    = prc_channels_raw.chan1_raw();
			const some_chan2_raw    = prc_channels_raw.chan2_raw();
			const some_chan3_raw    = prc_channels_raw.chan3_raw();
			const some_chan4_raw    = prc_channels_raw.chan4_raw();
			const some_chan5_raw    = prc_channels_raw.chan5_raw();
			const some_chan6_raw    = prc_channels_raw.chan6_raw();
			const some_chan7_raw    = prc_channels_raw.chan7_raw();
			const some_chan8_raw    = prc_channels_raw.chan8_raw();
			const some_rssi         = prc_channels_raw.rssi();
			
		}
		
		export function onSERVO_OUTPUT_RAW(pservo_output_raw: _Host.SERVO_OUTPUT_RAW) {
			const some_time_usec  = pservo_output_raw.time_usec();
			const some_port       = pservo_output_raw.port();
			const some_servo1_raw = pservo_output_raw.servo1_raw();
			const some_servo2_raw = pservo_output_raw.servo2_raw();
			const some_servo3_raw = pservo_output_raw.servo3_raw();
			const some_servo4_raw = pservo_output_raw.servo4_raw();
			const some_servo5_raw = pservo_output_raw.servo5_raw();
			const some_servo6_raw = pservo_output_raw.servo6_raw();
			const some_servo7_raw = pservo_output_raw.servo7_raw();
			const some_servo8_raw = pservo_output_raw.servo8_raw();
			{
				
				const item = pservo_output_raw.servo9_raw();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pservo_output_raw.servo10_raw();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pservo_output_raw.servo11_raw();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pservo_output_raw.servo12_raw();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pservo_output_raw.servo13_raw();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pservo_output_raw.servo14_raw();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pservo_output_raw.servo15_raw();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = pservo_output_raw.servo16_raw();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function onMEMINFO(pmeminfo: _Host.MEMINFO) {
			const some_brkval  = pmeminfo.brkval();
			const some_freemem = pmeminfo.freemem();
			{
				
				const item = pmeminfo.freemem32();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function fill_MEMINFO(pmeminfo: _Host.MEMINFO) {
			pmeminfo.brkval_(some_number);
			pmeminfo.freemem_(some_number);
			
			pmeminfo.freemem32_(some_number);
			
			
		}
		
		export function onMISSION_ITEM_REACHED(pmission_item_reached: _Host.MISSION_ITEM_REACHED) {
			const some_seq = pmission_item_reached.seq();
			
		}
		
		export function onLOGGING_ACK(plogging_ack: _Host.LOGGING_ACK) {
			const some_target_system    = plogging_ack.target_system();
			const some_target_component = plogging_ack.target_component();
			const some_sequence         = plogging_ack.sequence();
			
		}
		
		export function fill_LOGGING_ACK(plogging_ack: _Host.LOGGING_ACK) {
			plogging_ack.target_system_(some_number);
			plogging_ack.target_component_(some_number);
			plogging_ack.sequence_(some_number);
			
		}
		
		export function onVISION_SPEED_ESTIMATE(pvision_speed_estimate: _Host.VISION_SPEED_ESTIMATE) {
			const some_usec = pvision_speed_estimate.usec();
			const some_x    = pvision_speed_estimate.x();
			const some_y    = pvision_speed_estimate.y();
			const some_z    = pvision_speed_estimate.z();
			
		}
		
		export function fill_VISION_SPEED_ESTIMATE(pvision_speed_estimate: _Host.VISION_SPEED_ESTIMATE) {
			pvision_speed_estimate.usec_(some_number);
			pvision_speed_estimate.x_(some_number);
			pvision_speed_estimate.y_(some_number);
			pvision_speed_estimate.z_(some_number);
			
		}
		
		export function onDEBUG_VECT(pdebug_vect: _Host.DEBUG_VECT) {
			{
				
				const item = pdebug_vect.name();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_time_usec = pdebug_vect.time_usec();
			const some_x         = pdebug_vect.x();
			const some_y         = pdebug_vect.y();
			const some_z         = pdebug_vect.z();
			
		}
		
		export function fill_DEBUG_VECT(pdebug_vect: _Host.DEBUG_VECT) {
			pdebug_vect.name_(some_string);
			pdebug_vect.time_usec_(some_number);
			pdebug_vect.x_(some_number);
			pdebug_vect.y_(some_number);
			pdebug_vect.z_(some_number);
			
		}
		
		export function onLOG_REQUEST_END(plog_request_end: _Host.LOG_REQUEST_END) {
			const some_target_system    = plog_request_end.target_system();
			const some_target_component = plog_request_end.target_component();
			
		}
		
		export function fill_LOG_REQUEST_END(plog_request_end: _Host.LOG_REQUEST_END) {
			plog_request_end.target_system_(some_number);
			plog_request_end.target_component_(some_number);
			
		}
		
		export function onMISSION_ACK(pmission_ack: _Host.MISSION_ACK) {
			const some_target_system    = pmission_ack.target_system();
			const some_target_component = pmission_ack.target_component();
			{
				
				const item = pmission_ack.typE();
				if (item !== null)
					_Host.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED = item;
				
			}
			{
				
				const item = pmission_ack.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onCHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack: _Host.CHANGE_OPERATOR_CONTROL_ACK) {
			const some_gcs_system_id   = pchange_operator_control_ack.gcs_system_id();
			const some_control_request = pchange_operator_control_ack.control_request();
			const some_ack             = pchange_operator_control_ack.ack();
			
		}
		
		export function onMISSION_CURRENT(pmission_current: _Host.MISSION_CURRENT) {
			const some_seq = pmission_current.seq();
			
		}
		
		export function onSYSTEM_TIME(psystem_time: _Host.SYSTEM_TIME) {
			const some_time_unix_usec = psystem_time.time_unix_usec();
			const some_time_boot_ms   = psystem_time.time_boot_ms();
			
		}
		
		export function onCAMERA_TRIGGER(pcamera_trigger: _Host.CAMERA_TRIGGER) {
			const some_time_usec = pcamera_trigger.time_usec();
			const some_seq       = pcamera_trigger.seq();
			
		}
		
		export function fill_CAMERA_TRIGGER(pcamera_trigger: _Host.CAMERA_TRIGGER) {
			pcamera_trigger.time_usec_(some_number);
			pcamera_trigger.seq_(some_number);
			
		}
		
		export function onGOPRO_SET_RESPONSE(pgopro_set_response: _Host.GOPRO_SET_RESPONSE) {
			{
				
				const item = pgopro_set_response.cmd_id();
				if (item !== null)
					_Host.GOPRO_COMMAND.GOPRO_COMMAND_POWER = item;
				
			}
			{
				
				const item = pgopro_set_response.status();
				if (item !== null)
					_Host.GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS = item;
				
			}
			
		}
		
		export function fill_GOPRO_SET_RESPONSE(pgopro_set_response: _Host.GOPRO_SET_RESPONSE) {
			
			pgopro_set_response.cmd_id_(_Host.GOPRO_COMMAND.GOPRO_COMMAND_POWER);
			
			
			pgopro_set_response.status_(_Host.GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
			
			
		}
		
		export function onVISION_POSITION_ESTIMATE(pvision_position_estimate: _Host.VISION_POSITION_ESTIMATE) {
			const some_usec  = pvision_position_estimate.usec();
			const some_x     = pvision_position_estimate.x();
			const some_y     = pvision_position_estimate.y();
			const some_z     = pvision_position_estimate.z();
			const some_roll  = pvision_position_estimate.roll();
			const some_pitch = pvision_position_estimate.pitch();
			const some_yaw   = pvision_position_estimate.yaw();
			
		}
		
		export function onMANUAL_CONTROL(pmanual_control: _Host.MANUAL_CONTROL) {
			const some_target  = pmanual_control.target();
			const some_x       = pmanual_control.x();
			const some_y       = pmanual_control.y();
			const some_z       = pmanual_control.z();
			const some_r       = pmanual_control.r();
			const some_buttons = pmanual_control.buttons();
			
		}
		
		export function onRC_CHANNELS(prc_channels: _Host.RC_CHANNELS) {
			const some_time_boot_ms = prc_channels.time_boot_ms();
			const some_chancount    = prc_channels.chancount();
			const some_chan1_raw    = prc_channels.chan1_raw();
			const some_chan2_raw    = prc_channels.chan2_raw();
			const some_chan3_raw    = prc_channels.chan3_raw();
			const some_chan4_raw    = prc_channels.chan4_raw();
			const some_chan5_raw    = prc_channels.chan5_raw();
			const some_chan6_raw    = prc_channels.chan6_raw();
			const some_chan7_raw    = prc_channels.chan7_raw();
			const some_chan8_raw    = prc_channels.chan8_raw();
			const some_chan9_raw    = prc_channels.chan9_raw();
			const some_chan10_raw   = prc_channels.chan10_raw();
			const some_chan11_raw   = prc_channels.chan11_raw();
			const some_chan12_raw   = prc_channels.chan12_raw();
			const some_chan13_raw   = prc_channels.chan13_raw();
			const some_chan14_raw   = prc_channels.chan14_raw();
			const some_chan15_raw   = prc_channels.chan15_raw();
			const some_chan16_raw   = prc_channels.chan16_raw();
			const some_chan17_raw   = prc_channels.chan17_raw();
			const some_chan18_raw   = prc_channels.chan18_raw();
			const some_rssi         = prc_channels.rssi();
			
		}
		
		export function onPROTOCOL_VERSION(pprotocol_version: _Host.PROTOCOL_VERSION) {
			const some_version     = pprotocol_version.version();
			const some_min_version = pprotocol_version.min_version();
			const some_max_version = pprotocol_version.max_version();
			{
				const item = pprotocol_version.spec_version_hash();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pprotocol_version.library_version_hash();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_PROTOCOL_VERSION(pprotocol_version: _Host.PROTOCOL_VERSION) {
			pprotocol_version.version_(some_number);
			pprotocol_version.min_version_(some_number);
			pprotocol_version.max_version_(some_number);
			{
				const item = pprotocol_version.spec_version_hash();
				
				for (let i = 0; i < _Host.PROTOCOL_VERSION.spec_version_hash.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pprotocol_version.library_version_hash();
				
				for (let i = 0; i < _Host.PROTOCOL_VERSION.library_version_hash.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onRALLY_FETCH_POINT(prally_fetch_point: _Host.RALLY_FETCH_POINT) {
			const some_target_system    = prally_fetch_point.target_system();
			const some_target_component = prally_fetch_point.target_component();
			const some_idx              = prally_fetch_point.idx();
			
		}
		
		export function fill_RALLY_FETCH_POINT(prally_fetch_point: _Host.RALLY_FETCH_POINT) {
			prally_fetch_point.target_system_(some_number);
			prally_fetch_point.target_component_(some_number);
			prally_fetch_point.idx_(some_number);
			
		}
		
		export function onPARAM_VALUE(pparam_value: _Host.PARAM_VALUE) {
			{
				
				const item = pparam_value.param_id();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_param_value = pparam_value.param_value();
			{
				
				const item = pparam_value.param_type();
				if (item !== null)
					_Host.MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8 = item;
				
			}
			const some_param_count = pparam_value.param_count();
			const some_param_index = pparam_value.param_index();
			
		}
		
		export function onBATTERY_STATUS(pbattery_status: _Host.BATTERY_STATUS) {
			const some_id = pbattery_status.id();
			{
				
				const item = pbattery_status.battery_function();
				if (item !== null)
					_Host.MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN = item;
				
			}
			{
				
				const item = pbattery_status.typE();
				if (item !== null)
					_Host.MAV_BATTERY_TYPE.UNKNOWN = item;
				
			}
			const some_temperature = pbattery_status.temperature();
			{
				const item = pbattery_status.voltages();
				for (let value of item)
					some_number = value;
				
			}
			const some_current_battery   = pbattery_status.current_battery();
			const some_current_consumed  = pbattery_status.current_consumed();
			const some_energy_consumed   = pbattery_status.energy_consumed();
			const some_battery_remaining = pbattery_status.battery_remaining();
			
		}
		
		export function fill_BATTERY_STATUS(pbattery_status: _Host.BATTERY_STATUS) {
			pbattery_status.id_(some_number);
			
			pbattery_status.battery_function_(_Host.MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
			
			
			pbattery_status.typE_(_Host.MAV_BATTERY_TYPE.UNKNOWN);
			
			pbattery_status.temperature_(some_number);
			{
				const item = pbattery_status.voltages();
				
				for (let i = 0; i < _Host.BATTERY_STATUS.voltages.item_len; i++)
					item.set(some_number, i);
				
			}
			pbattery_status.current_battery_(some_number);
			pbattery_status.current_consumed_(some_number);
			pbattery_status.energy_consumed_(some_number);
			pbattery_status.battery_remaining_(some_number);
			
		}
		
		export function onSERIAL_CONTROL(pserial_control: _Host.SERIAL_CONTROL) {
			{
				
				const item = pserial_control.device();
				if (item !== null)
					_Host.SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1 = item;
				
			}
			{
				
				const item = pserial_control.flags();
				if (item !== null)
					_Host.SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY = item;
				
			}
			const some_timeout  = pserial_control.timeout();
			const some_baudrate = pserial_control.baudrate();
			const some_count    = pserial_control.count();
			{
				const item = pserial_control.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_SERIAL_CONTROL(pserial_control: _Host.SERIAL_CONTROL) {
			
			pserial_control.device_(_Host.SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
			
			
			pserial_control.flags_(_Host.SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
			
			pserial_control.timeout_(some_number);
			pserial_control.baudrate_(some_number);
			pserial_control.count_(some_number);
			{
				const item = pserial_control.daTa();
				
				for (let i = 0; i < _Host.SERIAL_CONTROL.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onSET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned: _Host.SET_POSITION_TARGET_LOCAL_NED) {
			const some_time_boot_ms     = pset_position_target_local_ned.time_boot_ms();
			const some_target_system    = pset_position_target_local_ned.target_system();
			const some_target_component = pset_position_target_local_ned.target_component();
			{
				
				const item = pset_position_target_local_ned.coordinate_frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			const some_type_mask = pset_position_target_local_ned.type_mask();
			const some_x         = pset_position_target_local_ned.x();
			const some_y         = pset_position_target_local_ned.y();
			const some_z         = pset_position_target_local_ned.z();
			const some_vx        = pset_position_target_local_ned.vx();
			const some_vy        = pset_position_target_local_ned.vy();
			const some_vz        = pset_position_target_local_ned.vz();
			const some_afx       = pset_position_target_local_ned.afx();
			const some_afy       = pset_position_target_local_ned.afy();
			const some_afz       = pset_position_target_local_ned.afz();
			const some_yaw       = pset_position_target_local_ned.yaw();
			const some_yaw_rate  = pset_position_target_local_ned.yaw_rate();
			
		}
		
		export function onMOUNT_ORIENTATION(pmount_orientation: _Host.MOUNT_ORIENTATION) {
			const some_time_boot_ms = pmount_orientation.time_boot_ms();
			const some_roll         = pmount_orientation.roll();
			const some_pitch        = pmount_orientation.pitch();
			const some_yaw          = pmount_orientation.yaw();
			
		}
		
		export function fill_MOUNT_ORIENTATION(pmount_orientation: _Host.MOUNT_ORIENTATION) {
			pmount_orientation.time_boot_ms_(some_number);
			pmount_orientation.roll_(some_number);
			pmount_orientation.pitch_(some_number);
			pmount_orientation.yaw_(some_number);
			
		}
		
		export function onSET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin: _Host.SET_GPS_GLOBAL_ORIGIN) {
			const some_target_system = pset_gps_global_origin.target_system();
			const some_latitude      = pset_gps_global_origin.latitude();
			const some_longitude     = pset_gps_global_origin.longitude();
			const some_altitude      = pset_gps_global_origin.altitude();
			{
				
				const item = pset_gps_global_origin.time_usec();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function onPARAM_EXT_SET(pparam_ext_set: _Host.PARAM_EXT_SET) {
			const some_target_system    = pparam_ext_set.target_system();
			const some_target_component = pparam_ext_set.target_component();
			{
				
				const item = pparam_ext_set.param_id();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = pparam_ext_set.param_value();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = pparam_ext_set.param_type();
				if (item !== null)
					_Host.MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8 = item;
				
			}
			
		}
		
		export function fill_PARAM_EXT_SET(pparam_ext_set: _Host.PARAM_EXT_SET) {
			pparam_ext_set.target_system_(some_number);
			pparam_ext_set.target_component_(some_number);
			pparam_ext_set.param_id_(some_string);
			pparam_ext_set.param_value_(some_string);
			
			pparam_ext_set.param_type_(_Host.MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
			
			
		}
		
		export function onAUTOPILOT_VERSION(pautopilot_version: _Host.AUTOPILOT_VERSION) {
			{
				
				const item = pautopilot_version.capabilities();
				if (item !== null)
					_Host.MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = item;
				
			}
			const some_flight_sw_version     = pautopilot_version.flight_sw_version();
			const some_middleware_sw_version = pautopilot_version.middleware_sw_version();
			const some_os_sw_version         = pautopilot_version.os_sw_version();
			const some_board_version         = pautopilot_version.board_version();
			{
				const item = pautopilot_version.flight_custom_version();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pautopilot_version.middleware_custom_version();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pautopilot_version.os_custom_version();
				for (let value of item)
					some_number = value;
				
			}
			const some_vendor_id  = pautopilot_version.vendor_id();
			const some_product_id = pautopilot_version.product_id();
			const some_uid        = pautopilot_version.uid();
			{
				const fld = pautopilot_version.uid2();
				if (fld)
					fld.enumerate((item: number, d0: number) => {
							some_number = item;
						}
					);
			}
			
		}
		
		export function fill_AUTOPILOT_VERSION(pautopilot_version: _Host.AUTOPILOT_VERSION) {
			
			pautopilot_version.capabilities_(_Host.MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
			
			pautopilot_version.flight_sw_version_(some_number);
			pautopilot_version.middleware_sw_version_(some_number);
			pautopilot_version.os_sw_version_(some_number);
			pautopilot_version.board_version_(some_number);
			{
				const item = pautopilot_version.flight_custom_version();
				
				for (let i = 0; i < _Host.AUTOPILOT_VERSION.flight_custom_version.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pautopilot_version.middleware_custom_version();
				
				for (let i = 0; i < _Host.AUTOPILOT_VERSION.middleware_custom_version.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pautopilot_version.os_custom_version();
				
				for (let i = 0; i < _Host.AUTOPILOT_VERSION.os_custom_version.item_len; i++)
					item.set(some_number, i);
				
			}
			pautopilot_version.vendor_id_(some_number);
			pautopilot_version.product_id_(some_number);
			pautopilot_version.uid_(some_number);
			
			for (let d0 = 0; d0 < _Host.AUTOPILOT_VERSION.uid2.d0; d0++) {
				
				pautopilot_version.uid2_(some_number, d0);
				
			}
			
		}
		
		export function onMISSION_REQUEST_LIST(pmission_request_list: _Host.MISSION_REQUEST_LIST) {
			const some_target_system    = pmission_request_list.target_system();
			const some_target_component = pmission_request_list.target_component();
			{
				
				const item = pmission_request_list.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onSIMSTATE(psimstate: _Host.SIMSTATE) {
			const some_roll  = psimstate.roll();
			const some_pitch = psimstate.pitch();
			const some_yaw   = psimstate.yaw();
			const some_xacc  = psimstate.xacc();
			const some_yacc  = psimstate.yacc();
			const some_zacc  = psimstate.zacc();
			const some_xgyro = psimstate.xgyro();
			const some_ygyro = psimstate.ygyro();
			const some_zgyro = psimstate.zgyro();
			const some_lat   = psimstate.lat();
			const some_lng   = psimstate.lng();
			
		}
		
		export function fill_SIMSTATE(psimstate: _Host.SIMSTATE) {
			psimstate.roll_(some_number);
			psimstate.pitch_(some_number);
			psimstate.yaw_(some_number);
			psimstate.xacc_(some_number);
			psimstate.yacc_(some_number);
			psimstate.zacc_(some_number);
			psimstate.xgyro_(some_number);
			psimstate.ygyro_(some_number);
			psimstate.zgyro_(some_number);
			psimstate.lat_(some_number);
			psimstate.lng_(some_number);
			
		}
		
		export function onSET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings: _Host.SET_VIDEO_STREAM_SETTINGS) {
			const some_target_system    = pset_video_stream_settings.target_system();
			const some_target_component = pset_video_stream_settings.target_component();
			const some_camera_id        = pset_video_stream_settings.camera_id();
			const some_framerate        = pset_video_stream_settings.framerate();
			const some_resolution_h     = pset_video_stream_settings.resolution_h();
			const some_resolution_v     = pset_video_stream_settings.resolution_v();
			const some_bitrate          = pset_video_stream_settings.bitrate();
			const some_rotation         = pset_video_stream_settings.rotation();
			{
				
				const item = pset_video_stream_settings.uri();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function fill_SET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings: _Host.SET_VIDEO_STREAM_SETTINGS) {
			pset_video_stream_settings.target_system_(some_number);
			pset_video_stream_settings.target_component_(some_number);
			pset_video_stream_settings.camera_id_(some_number);
			pset_video_stream_settings.framerate_(some_number);
			pset_video_stream_settings.resolution_h_(some_number);
			pset_video_stream_settings.resolution_v_(some_number);
			pset_video_stream_settings.bitrate_(some_number);
			pset_video_stream_settings.rotation_(some_number);
			pset_video_stream_settings.uri_(some_string);
			
		}
		
		export function onPLAY_TUNE(pplay_tune: _Host.PLAY_TUNE) {
			const some_target_system    = pplay_tune.target_system();
			const some_target_component = pplay_tune.target_component();
			{
				
				const item = pplay_tune.tune();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function fill_PLAY_TUNE(pplay_tune: _Host.PLAY_TUNE) {
			pplay_tune.target_system_(some_number);
			pplay_tune.target_component_(some_number);
			pplay_tune.tune_(some_string);
			
		}
		
		export function onDIGICAM_CONFIGURE(pdigicam_configure: _Host.DIGICAM_CONFIGURE) {
			const some_target_system    = pdigicam_configure.target_system();
			const some_target_component = pdigicam_configure.target_component();
			const some_mode             = pdigicam_configure.mode();
			const some_shutter_speed    = pdigicam_configure.shutter_speed();
			const some_aperture         = pdigicam_configure.aperture();
			const some_iso              = pdigicam_configure.iso();
			const some_exposure_type    = pdigicam_configure.exposure_type();
			const some_command_id       = pdigicam_configure.command_id();
			const some_engine_cut_off   = pdigicam_configure.engine_cut_off();
			const some_extra_param      = pdigicam_configure.extra_param();
			const some_extra_value      = pdigicam_configure.extra_value();
			
		}
		
		export function fill_DIGICAM_CONFIGURE(pdigicam_configure: _Host.DIGICAM_CONFIGURE) {
			pdigicam_configure.target_system_(some_number);
			pdigicam_configure.target_component_(some_number);
			pdigicam_configure.mode_(some_number);
			pdigicam_configure.shutter_speed_(some_number);
			pdigicam_configure.aperture_(some_number);
			pdigicam_configure.iso_(some_number);
			pdigicam_configure.exposure_type_(some_number);
			pdigicam_configure.command_id_(some_number);
			pdigicam_configure.engine_cut_off_(some_number);
			pdigicam_configure.extra_param_(some_number);
			pdigicam_configure.extra_value_(some_number);
			
		}
		
		export function onSCALED_PRESSURE3(pscaled_pressure3: _Host.SCALED_PRESSURE3) {
			const some_time_boot_ms = pscaled_pressure3.time_boot_ms();
			const some_press_abs    = pscaled_pressure3.press_abs();
			const some_press_diff   = pscaled_pressure3.press_diff();
			const some_temperature  = pscaled_pressure3.temperature();
			
		}
		
		export function fill_SCALED_PRESSURE3(pscaled_pressure3: _Host.SCALED_PRESSURE3) {
			pscaled_pressure3.time_boot_ms_(some_number);
			pscaled_pressure3.press_abs_(some_number);
			pscaled_pressure3.press_diff_(some_number);
			pscaled_pressure3.temperature_(some_number);
			
		}
		
		export function onMISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list: _Host.MISSION_REQUEST_PARTIAL_LIST) {
			const some_target_system    = pmission_request_partial_list.target_system();
			const some_target_component = pmission_request_partial_list.target_component();
			const some_start_index      = pmission_request_partial_list.start_index();
			const some_end_index        = pmission_request_partial_list.end_index();
			{
				
				const item = pmission_request_partial_list.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onPARAM_EXT_ACK(pparam_ext_ack: _Host.PARAM_EXT_ACK) {
			{
				
				const item = pparam_ext_ack.param_id();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = pparam_ext_ack.param_value();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			{
				
				const item = pparam_ext_ack.param_type();
				if (item !== null)
					_Host.MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8 = item;
				
			}
			{
				
				const item = pparam_ext_ack.param_result();
				if (item !== null)
					_Host.PARAM_ACK.PARAM_ACK_ACCEPTED = item;
				
			}
			
		}
		
		export function fill_PARAM_EXT_ACK(pparam_ext_ack: _Host.PARAM_EXT_ACK) {
			pparam_ext_ack.param_id_(some_string);
			pparam_ext_ack.param_value_(some_string);
			
			pparam_ext_ack.param_type_(_Host.MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
			
			
			pparam_ext_ack.param_result_(_Host.PARAM_ACK.PARAM_ACK_ACCEPTED);
			
			
		}
		
		export function onUAVCAN_NODE_INFO(puavcan_node_info: _Host.UAVCAN_NODE_INFO) {
			const some_time_usec  = puavcan_node_info.time_usec();
			const some_uptime_sec = puavcan_node_info.uptime_sec();
			{
				
				const item = puavcan_node_info.name();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_hw_version_major = puavcan_node_info.hw_version_major();
			const some_hw_version_minor = puavcan_node_info.hw_version_minor();
			{
				const item = puavcan_node_info.hw_unique_id();
				for (let value of item)
					some_number = value;
				
			}
			const some_sw_version_major = puavcan_node_info.sw_version_major();
			const some_sw_version_minor = puavcan_node_info.sw_version_minor();
			const some_sw_vcs_commit    = puavcan_node_info.sw_vcs_commit();
			
		}
		
		export function fill_UAVCAN_NODE_INFO(puavcan_node_info: _Host.UAVCAN_NODE_INFO) {
			puavcan_node_info.time_usec_(some_number);
			puavcan_node_info.uptime_sec_(some_number);
			puavcan_node_info.name_(some_string);
			puavcan_node_info.hw_version_major_(some_number);
			puavcan_node_info.hw_version_minor_(some_number);
			{
				const item = puavcan_node_info.hw_unique_id();
				
				for (let i = 0; i < _Host.UAVCAN_NODE_INFO.hw_unique_id.item_len; i++)
					item.set(some_number, i);
				
			}
			puavcan_node_info.sw_version_major_(some_number);
			puavcan_node_info.sw_version_minor_(some_number);
			puavcan_node_info.sw_vcs_commit_(some_number);
			
		}
		
		export function onDATA16(pdata16: _Host.DATA16) {
			const some_typE = pdata16.typE();
			const some_len  = pdata16.len();
			{
				const item = pdata16.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_DATA16(pdata16: _Host.DATA16) {
			pdata16.typE_(some_number);
			pdata16.len_(some_number);
			{
				const item = pdata16.daTa();
				
				for (let i = 0; i < _Host.DATA16.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onSET_MAG_OFFSETS(pset_mag_offsets: _Host.SET_MAG_OFFSETS) {
			const some_target_system    = pset_mag_offsets.target_system();
			const some_target_component = pset_mag_offsets.target_component();
			const some_mag_ofs_x        = pset_mag_offsets.mag_ofs_x();
			const some_mag_ofs_y        = pset_mag_offsets.mag_ofs_y();
			const some_mag_ofs_z        = pset_mag_offsets.mag_ofs_z();
			
		}
		
		export function fill_SET_MAG_OFFSETS(pset_mag_offsets: _Host.SET_MAG_OFFSETS) {
			pset_mag_offsets.target_system_(some_number);
			pset_mag_offsets.target_component_(some_number);
			pset_mag_offsets.mag_ofs_x_(some_number);
			pset_mag_offsets.mag_ofs_y_(some_number);
			pset_mag_offsets.mag_ofs_z_(some_number);
			
		}
		
		export function onAP_ADC(pap_adc: _Host.AP_ADC) {
			const some_adc1 = pap_adc.adc1();
			const some_adc2 = pap_adc.adc2();
			const some_adc3 = pap_adc.adc3();
			const some_adc4 = pap_adc.adc4();
			const some_adc5 = pap_adc.adc5();
			const some_adc6 = pap_adc.adc6();
			
		}
		
		export function fill_AP_ADC(pap_adc: _Host.AP_ADC) {
			pap_adc.adc1_(some_number);
			pap_adc.adc2_(some_number);
			pap_adc.adc3_(some_number);
			pap_adc.adc4_(some_number);
			pap_adc.adc5_(some_number);
			pap_adc.adc6_(some_number);
			
		}
		
		export function onWIND(pwind: _Host.WIND) {
			const some_direction = pwind.direction();
			const some_speed     = pwind.speed();
			const some_speed_z   = pwind.speed_z();
			
		}
		
		export function fill_WIND(pwind: _Host.WIND) {
			pwind.direction_(some_number);
			pwind.speed_(some_number);
			pwind.speed_z_(some_number);
			
		}
		
		export function onAUTOPILOT_VERSION_REQUEST(pautopilot_version_request: _Host.AUTOPILOT_VERSION_REQUEST) {
			const some_target_system    = pautopilot_version_request.target_system();
			const some_target_component = pautopilot_version_request.target_component();
			
		}
		
		export function fill_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request: _Host.AUTOPILOT_VERSION_REQUEST) {
			pautopilot_version_request.target_system_(some_number);
			pautopilot_version_request.target_component_(some_number);
			
		}
		
		export function onLOCAL_POSITION_NED(plocal_position_ned: _Host.LOCAL_POSITION_NED) {
			const some_time_boot_ms = plocal_position_ned.time_boot_ms();
			const some_x            = plocal_position_ned.x();
			const some_y            = plocal_position_ned.y();
			const some_z            = plocal_position_ned.z();
			const some_vx           = plocal_position_ned.vx();
			const some_vy           = plocal_position_ned.vy();
			const some_vz           = plocal_position_ned.vz();
			
		}
		
		export function onDATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake: _Host.DATA_TRANSMISSION_HANDSHAKE) {
			const some_typE        = pdata_transmission_handshake.typE();
			const some_size        = pdata_transmission_handshake.size();
			const some_width       = pdata_transmission_handshake.width();
			const some_height      = pdata_transmission_handshake.height();
			const some_packets     = pdata_transmission_handshake.packets();
			const some_payload     = pdata_transmission_handshake.payload();
			const some_jpg_quality = pdata_transmission_handshake.jpg_quality();
			
		}
		
		export function fill_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake: _Host.DATA_TRANSMISSION_HANDSHAKE) {
			pdata_transmission_handshake.typE_(some_number);
			pdata_transmission_handshake.size_(some_number);
			pdata_transmission_handshake.width_(some_number);
			pdata_transmission_handshake.height_(some_number);
			pdata_transmission_handshake.packets_(some_number);
			pdata_transmission_handshake.payload_(some_number);
			pdata_transmission_handshake.jpg_quality_(some_number);
			
		}
		
		export function onGPS_GLOBAL_ORIGIN(pgps_global_origin: _Host.GPS_GLOBAL_ORIGIN) {
			const some_latitude  = pgps_global_origin.latitude();
			const some_longitude = pgps_global_origin.longitude();
			const some_altitude  = pgps_global_origin.altitude();
			{
				
				const item = pgps_global_origin.time_usec();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function onSCALED_IMU2(pscaled_imu2: _Host.SCALED_IMU2) {
			const some_time_boot_ms = pscaled_imu2.time_boot_ms();
			const some_xacc         = pscaled_imu2.xacc();
			const some_yacc         = pscaled_imu2.yacc();
			const some_zacc         = pscaled_imu2.zacc();
			const some_xgyro        = pscaled_imu2.xgyro();
			const some_ygyro        = pscaled_imu2.ygyro();
			const some_zgyro        = pscaled_imu2.zgyro();
			const some_xmag         = pscaled_imu2.xmag();
			const some_ymag         = pscaled_imu2.ymag();
			const some_zmag         = pscaled_imu2.zmag();
			
		}
		
		export function fill_SCALED_IMU2(pscaled_imu2: _Host.SCALED_IMU2) {
			pscaled_imu2.time_boot_ms_(some_number);
			pscaled_imu2.xacc_(some_number);
			pscaled_imu2.yacc_(some_number);
			pscaled_imu2.zacc_(some_number);
			pscaled_imu2.xgyro_(some_number);
			pscaled_imu2.ygyro_(some_number);
			pscaled_imu2.zgyro_(some_number);
			pscaled_imu2.xmag_(some_number);
			pscaled_imu2.ymag_(some_number);
			pscaled_imu2.zmag_(some_number);
			
		}
		
		export function onATTITUDE_QUATERNION(pattitude_quaternion: _Host.ATTITUDE_QUATERNION) {
			const some_time_boot_ms = pattitude_quaternion.time_boot_ms();
			const some_q1           = pattitude_quaternion.q1();
			const some_q2           = pattitude_quaternion.q2();
			const some_q3           = pattitude_quaternion.q3();
			const some_q4           = pattitude_quaternion.q4();
			const some_rollspeed    = pattitude_quaternion.rollspeed();
			const some_pitchspeed   = pattitude_quaternion.pitchspeed();
			const some_yawspeed     = pattitude_quaternion.yawspeed();
			
		}
		
		export function onDATA64(pdata64: _Host.DATA64) {
			const some_typE = pdata64.typE();
			const some_len  = pdata64.len();
			{
				const item = pdata64.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_DATA64(pdata64: _Host.DATA64) {
			pdata64.typE_(some_number);
			pdata64.len_(some_number);
			{
				const item = pdata64.daTa();
				
				for (let i = 0; i < _Host.DATA64.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onHIL_ACTUATOR_CONTROLS(phil_actuator_controls: _Host.HIL_ACTUATOR_CONTROLS) {
			const some_time_usec = phil_actuator_controls.time_usec();
			{
				const item = phil_actuator_controls.controls();
				for (let value of item)
					some_number = value;
				
			}
			{
				
				const item = phil_actuator_controls.mode();
				if (item !== null)
					_Host.MAV_MODE.PREFLIGHT = item;
				
			}
			const some_flags = phil_actuator_controls.flags();
			
		}
		
		export function onPOSITION_TARGET_LOCAL_NED(pposition_target_local_ned: _Host.POSITION_TARGET_LOCAL_NED) {
			const some_time_boot_ms = pposition_target_local_ned.time_boot_ms();
			{
				
				const item = pposition_target_local_ned.coordinate_frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			const some_type_mask = pposition_target_local_ned.type_mask();
			const some_x         = pposition_target_local_ned.x();
			const some_y         = pposition_target_local_ned.y();
			const some_z         = pposition_target_local_ned.z();
			const some_vx        = pposition_target_local_ned.vx();
			const some_vy        = pposition_target_local_ned.vy();
			const some_vz        = pposition_target_local_ned.vz();
			const some_afx       = pposition_target_local_ned.afx();
			const some_afy       = pposition_target_local_ned.afy();
			const some_afz       = pposition_target_local_ned.afz();
			const some_yaw       = pposition_target_local_ned.yaw();
			const some_yaw_rate  = pposition_target_local_ned.yaw_rate();
			
		}
		
		export function onGIMBAL_REPORT(pgimbal_report: _Host.GIMBAL_REPORT) {
			const some_target_system    = pgimbal_report.target_system();
			const some_target_component = pgimbal_report.target_component();
			const some_delta_time       = pgimbal_report.delta_time();
			const some_delta_angle_x    = pgimbal_report.delta_angle_x();
			const some_delta_angle_y    = pgimbal_report.delta_angle_y();
			const some_delta_angle_z    = pgimbal_report.delta_angle_z();
			const some_delta_velocity_x = pgimbal_report.delta_velocity_x();
			const some_delta_velocity_y = pgimbal_report.delta_velocity_y();
			const some_delta_velocity_z = pgimbal_report.delta_velocity_z();
			const some_joint_roll       = pgimbal_report.joint_roll();
			const some_joint_el         = pgimbal_report.joint_el();
			const some_joint_az         = pgimbal_report.joint_az();
			
		}
		
		export function fill_GIMBAL_REPORT(pgimbal_report: _Host.GIMBAL_REPORT) {
			pgimbal_report.target_system_(some_number);
			pgimbal_report.target_component_(some_number);
			pgimbal_report.delta_time_(some_number);
			pgimbal_report.delta_angle_x_(some_number);
			pgimbal_report.delta_angle_y_(some_number);
			pgimbal_report.delta_angle_z_(some_number);
			pgimbal_report.delta_velocity_x_(some_number);
			pgimbal_report.delta_velocity_y_(some_number);
			pgimbal_report.delta_velocity_z_(some_number);
			pgimbal_report.joint_roll_(some_number);
			pgimbal_report.joint_el_(some_number);
			pgimbal_report.joint_az_(some_number);
			
		}
		
		export function onDEVICE_OP_WRITE(pdevice_op_write: _Host.DEVICE_OP_WRITE) {
			const some_target_system    = pdevice_op_write.target_system();
			const some_target_component = pdevice_op_write.target_component();
			const some_request_id       = pdevice_op_write.request_id();
			{
				
				const item = pdevice_op_write.bustype();
				if (item !== null)
					_Host.DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C = item;
				
			}
			const some_bus     = pdevice_op_write.bus();
			const some_address = pdevice_op_write.address();
			{
				
				const item = pdevice_op_write.busname();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_regstart = pdevice_op_write.regstart();
			const some_count    = pdevice_op_write.count();
			{
				const item = pdevice_op_write.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_DEVICE_OP_WRITE(pdevice_op_write: _Host.DEVICE_OP_WRITE) {
			pdevice_op_write.target_system_(some_number);
			pdevice_op_write.target_component_(some_number);
			pdevice_op_write.request_id_(some_number);
			
			pdevice_op_write.bustype_(_Host.DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
			
			pdevice_op_write.bus_(some_number);
			pdevice_op_write.address_(some_number);
			pdevice_op_write.busname_(some_string);
			pdevice_op_write.regstart_(some_number);
			pdevice_op_write.count_(some_number);
			{
				const item = pdevice_op_write.daTa();
				
				for (let i = 0; i < _Host.DEVICE_OP_WRITE.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onDISTANCE_SENSOR(pdistance_sensor: _Host.DISTANCE_SENSOR) {
			const some_time_boot_ms     = pdistance_sensor.time_boot_ms();
			const some_min_distance     = pdistance_sensor.min_distance();
			const some_max_distance     = pdistance_sensor.max_distance();
			const some_current_distance = pdistance_sensor.current_distance();
			{
				
				const item = pdistance_sensor.typE();
				if (item !== null)
					_Host.MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER = item;
				
			}
			const some_id = pdistance_sensor.id();
			{
				
				const item = pdistance_sensor.orientation();
				if (item !== null)
					_Host.MAV_SENSOR_ORIENTATION.NONE = item;
				
			}
			const some_covariance = pdistance_sensor.covariance();
			
		}
		
		export function fill_DISTANCE_SENSOR(pdistance_sensor: _Host.DISTANCE_SENSOR) {
			pdistance_sensor.time_boot_ms_(some_number);
			pdistance_sensor.min_distance_(some_number);
			pdistance_sensor.max_distance_(some_number);
			pdistance_sensor.current_distance_(some_number);
			
			pdistance_sensor.typE_(_Host.MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
			
			pdistance_sensor.id_(some_number);
			
			pdistance_sensor.orientation_(_Host.MAV_SENSOR_ORIENTATION.NONE);
			
			pdistance_sensor.covariance_(some_number);
			
		}
		
		export function onHIL_OPTICAL_FLOW(phil_optical_flow: _Host.HIL_OPTICAL_FLOW) {
			const some_time_usec              = phil_optical_flow.time_usec();
			const some_sensor_id              = phil_optical_flow.sensor_id();
			const some_integration_time_us    = phil_optical_flow.integration_time_us();
			const some_integrated_x           = phil_optical_flow.integrated_x();
			const some_integrated_y           = phil_optical_flow.integrated_y();
			const some_integrated_xgyro       = phil_optical_flow.integrated_xgyro();
			const some_integrated_ygyro       = phil_optical_flow.integrated_ygyro();
			const some_integrated_zgyro       = phil_optical_flow.integrated_zgyro();
			const some_temperature            = phil_optical_flow.temperature();
			const some_quality                = phil_optical_flow.quality();
			const some_time_delta_distance_us = phil_optical_flow.time_delta_distance_us();
			const some_distance               = phil_optical_flow.distance();
			
		}
		
		export function fill_HIL_OPTICAL_FLOW(phil_optical_flow: _Host.HIL_OPTICAL_FLOW) {
			phil_optical_flow.time_usec_(some_number);
			phil_optical_flow.sensor_id_(some_number);
			phil_optical_flow.integration_time_us_(some_number);
			phil_optical_flow.integrated_x_(some_number);
			phil_optical_flow.integrated_y_(some_number);
			phil_optical_flow.integrated_xgyro_(some_number);
			phil_optical_flow.integrated_ygyro_(some_number);
			phil_optical_flow.integrated_zgyro_(some_number);
			phil_optical_flow.temperature_(some_number);
			phil_optical_flow.quality_(some_number);
			phil_optical_flow.time_delta_distance_us_(some_number);
			phil_optical_flow.distance_(some_number);
			
		}
		
		export function onSCALED_PRESSURE2(pscaled_pressure2: _Host.SCALED_PRESSURE2) {
			const some_time_boot_ms = pscaled_pressure2.time_boot_ms();
			const some_press_abs    = pscaled_pressure2.press_abs();
			const some_press_diff   = pscaled_pressure2.press_diff();
			const some_temperature  = pscaled_pressure2.temperature();
			
		}
		
		export function fill_SCALED_PRESSURE2(pscaled_pressure2: _Host.SCALED_PRESSURE2) {
			pscaled_pressure2.time_boot_ms_(some_number);
			pscaled_pressure2.press_abs_(some_number);
			pscaled_pressure2.press_diff_(some_number);
			pscaled_pressure2.temperature_(some_number);
			
		}
		
		export function onWIND_COV(pwind_cov: _Host.WIND_COV) {
			const some_time_usec      = pwind_cov.time_usec();
			const some_wind_x         = pwind_cov.wind_x();
			const some_wind_y         = pwind_cov.wind_y();
			const some_wind_z         = pwind_cov.wind_z();
			const some_var_horiz      = pwind_cov.var_horiz();
			const some_var_vert       = pwind_cov.var_vert();
			const some_wind_alt       = pwind_cov.wind_alt();
			const some_horiz_accuracy = pwind_cov.horiz_accuracy();
			const some_vert_accuracy  = pwind_cov.vert_accuracy();
			
		}
		
		export function fill_WIND_COV(pwind_cov: _Host.WIND_COV) {
			pwind_cov.time_usec_(some_number);
			pwind_cov.wind_x_(some_number);
			pwind_cov.wind_y_(some_number);
			pwind_cov.wind_z_(some_number);
			pwind_cov.var_horiz_(some_number);
			pwind_cov.var_vert_(some_number);
			pwind_cov.wind_alt_(some_number);
			pwind_cov.horiz_accuracy_(some_number);
			pwind_cov.vert_accuracy_(some_number);
			
		}
		
		export function onCHANGE_OPERATOR_CONTROL(pchange_operator_control: _Host.CHANGE_OPERATOR_CONTROL) {
			const some_target_system   = pchange_operator_control.target_system();
			const some_control_request = pchange_operator_control.control_request();
			const some_version         = pchange_operator_control.version();
			{
				
				const item = pchange_operator_control.passkey();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			
		}
		
		export function onGOPRO_SET_REQUEST(pgopro_set_request: _Host.GOPRO_SET_REQUEST) {
			const some_target_system    = pgopro_set_request.target_system();
			const some_target_component = pgopro_set_request.target_component();
			{
				
				const item = pgopro_set_request.cmd_id();
				if (item !== null)
					_Host.GOPRO_COMMAND.GOPRO_COMMAND_POWER = item;
				
			}
			{
				const item = pgopro_set_request.value();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_GOPRO_SET_REQUEST(pgopro_set_request: _Host.GOPRO_SET_REQUEST) {
			pgopro_set_request.target_system_(some_number);
			pgopro_set_request.target_component_(some_number);
			
			pgopro_set_request.cmd_id_(_Host.GOPRO_COMMAND.GOPRO_COMMAND_POWER);
			
			{
				const item = pgopro_set_request.value();
				
				for (let i = 0; i < _Host.GOPRO_SET_REQUEST.value.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onSYS_STATUS(psys_status: _Host.SYS_STATUS) {
			{
				
				const item = psys_status.onboard_control_sensors_present();
				if (item !== null)
					_Host.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO = item;
				
			}
			{
				
				const item = psys_status.onboard_control_sensors_enabled();
				if (item !== null)
					_Host.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO = item;
				
			}
			{
				
				const item = psys_status.onboard_control_sensors_health();
				if (item !== null)
					_Host.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO = item;
				
			}
			const some_load              = psys_status.load();
			const some_voltage_battery   = psys_status.voltage_battery();
			const some_current_battery   = psys_status.current_battery();
			const some_battery_remaining = psys_status.battery_remaining();
			const some_drop_rate_comm    = psys_status.drop_rate_comm();
			const some_errors_comm       = psys_status.errors_comm();
			const some_errors_count1     = psys_status.errors_count1();
			const some_errors_count2     = psys_status.errors_count2();
			const some_errors_count3     = psys_status.errors_count3();
			const some_errors_count4     = psys_status.errors_count4();
			
		}
		
		export function onMISSION_ITEM(pmission_item: _Host.MISSION_ITEM) {
			const some_target_system    = pmission_item.target_system();
			const some_target_component = pmission_item.target_component();
			const some_seq              = pmission_item.seq();
			{
				
				const item = pmission_item.frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			{
				
				const item = pmission_item.command();
				if (item !== null)
					_Host.MAV_CMD.MAV_CMD_NAV_WAYPOINT = item;
				
			}
			const some_current      = pmission_item.current();
			const some_autocontinue = pmission_item.autocontinue();
			const some_param1       = pmission_item.param1();
			const some_param2       = pmission_item.param2();
			const some_param3       = pmission_item.param3();
			const some_param4       = pmission_item.param4();
			const some_x            = pmission_item.x();
			const some_y            = pmission_item.y();
			const some_z            = pmission_item.z();
			{
				
				const item = pmission_item.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onRAW_IMU(praw_imu: _Host.RAW_IMU) {
			const some_time_usec = praw_imu.time_usec();
			const some_xacc      = praw_imu.xacc();
			const some_yacc      = praw_imu.yacc();
			const some_zacc      = praw_imu.zacc();
			const some_xgyro     = praw_imu.xgyro();
			const some_ygyro     = praw_imu.ygyro();
			const some_zgyro     = praw_imu.zgyro();
			const some_xmag      = praw_imu.xmag();
			const some_ymag      = praw_imu.ymag();
			const some_zmag      = praw_imu.zmag();
			
		}
		
		export function onCOMMAND_INT(pcommand_int: _Host.COMMAND_INT) {
			const some_target_system    = pcommand_int.target_system();
			const some_target_component = pcommand_int.target_component();
			{
				
				const item = pcommand_int.frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			{
				
				const item = pcommand_int.command();
				if (item !== null)
					_Host.MAV_CMD.MAV_CMD_NAV_WAYPOINT = item;
				
			}
			const some_current      = pcommand_int.current();
			const some_autocontinue = pcommand_int.autocontinue();
			const some_param1       = pcommand_int.param1();
			const some_param2       = pcommand_int.param2();
			const some_param3       = pcommand_int.param3();
			const some_param4       = pcommand_int.param4();
			const some_x            = pcommand_int.x();
			const some_y            = pcommand_int.y();
			const some_z            = pcommand_int.z();
			
		}
		
		export function onOPTICAL_FLOW(poptical_flow: _Host.OPTICAL_FLOW) {
			const some_time_usec       = poptical_flow.time_usec();
			const some_sensor_id       = poptical_flow.sensor_id();
			const some_flow_x          = poptical_flow.flow_x();
			const some_flow_y          = poptical_flow.flow_y();
			const some_flow_comp_m_x   = poptical_flow.flow_comp_m_x();
			const some_flow_comp_m_y   = poptical_flow.flow_comp_m_y();
			const some_quality         = poptical_flow.quality();
			const some_ground_distance = poptical_flow.ground_distance();
			{
				
				const item = poptical_flow.flow_rate_x();
				if (item !== null)
					some_number = item;
				
			}
			{
				
				const item = poptical_flow.flow_rate_y();
				if (item !== null)
					some_number = item;
				
			}
			
		}
		
		export function onMISSION_ITEM_INT(pmission_item_int: _Host.MISSION_ITEM_INT) {
			const some_target_system    = pmission_item_int.target_system();
			const some_target_component = pmission_item_int.target_component();
			const some_seq              = pmission_item_int.seq();
			{
				
				const item = pmission_item_int.frame();
				if (item !== null)
					_Host.MAV_FRAME.MAV_FRAME_GLOBAL = item;
				
			}
			{
				
				const item = pmission_item_int.command();
				if (item !== null)
					_Host.MAV_CMD.MAV_CMD_NAV_WAYPOINT = item;
				
			}
			const some_current      = pmission_item_int.current();
			const some_autocontinue = pmission_item_int.autocontinue();
			const some_param1       = pmission_item_int.param1();
			const some_param2       = pmission_item_int.param2();
			const some_param3       = pmission_item_int.param3();
			const some_param4       = pmission_item_int.param4();
			const some_x            = pmission_item_int.x();
			const some_y            = pmission_item_int.y();
			const some_z            = pmission_item_int.z();
			{
				
				const item = pmission_item_int.mission_type();
				if (item !== null)
					_Host.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION = item;
				
			}
			
		}
		
		export function onVISION_POSITION_DELTA(pvision_position_delta: _Host.VISION_POSITION_DELTA) {
			const some_time_usec       = pvision_position_delta.time_usec();
			const some_time_delta_usec = pvision_position_delta.time_delta_usec();
			{
				const item = pvision_position_delta.angle_delta();
				for (let value of item)
					some_number = value;
				
			}
			{
				const item = pvision_position_delta.position_delta();
				for (let value of item)
					some_number = value;
				
			}
			const some_confidence = pvision_position_delta.confidence();
			
		}
		
		export function fill_VISION_POSITION_DELTA(pvision_position_delta: _Host.VISION_POSITION_DELTA) {
			pvision_position_delta.time_usec_(some_number);
			pvision_position_delta.time_delta_usec_(some_number);
			{
				const item = pvision_position_delta.angle_delta();
				
				for (let i = 0; i < _Host.VISION_POSITION_DELTA.angle_delta.item_len; i++)
					item.set(some_number, i);
				
			}
			{
				const item = pvision_position_delta.position_delta();
				
				for (let i = 0; i < _Host.VISION_POSITION_DELTA.position_delta.item_len; i++)
					item.set(some_number, i);
				
			}
			pvision_position_delta.confidence_(some_number);
			
		}
		
		export function onLOGGING_DATA(plogging_data: _Host.LOGGING_DATA) {
			const some_target_system        = plogging_data.target_system();
			const some_target_component     = plogging_data.target_component();
			const some_sequence             = plogging_data.sequence();
			const some_length               = plogging_data.length();
			const some_first_message_offset = plogging_data.first_message_offset();
			{
				const item = plogging_data.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_LOGGING_DATA(plogging_data: _Host.LOGGING_DATA) {
			plogging_data.target_system_(some_number);
			plogging_data.target_component_(some_number);
			plogging_data.sequence_(some_number);
			plogging_data.length_(some_number);
			plogging_data.first_message_offset_(some_number);
			{
				const item = plogging_data.daTa();
				
				for (let i = 0; i < _Host.LOGGING_DATA.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onDEVICE_OP_READ(pdevice_op_read: _Host.DEVICE_OP_READ) {
			const some_target_system    = pdevice_op_read.target_system();
			const some_target_component = pdevice_op_read.target_component();
			const some_request_id       = pdevice_op_read.request_id();
			{
				
				const item = pdevice_op_read.bustype();
				if (item !== null)
					_Host.DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C = item;
				
			}
			const some_bus     = pdevice_op_read.bus();
			const some_address = pdevice_op_read.address();
			{
				
				const item = pdevice_op_read.busname();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_regstart = pdevice_op_read.regstart();
			const some_count    = pdevice_op_read.count();
			
		}
		
		export function fill_DEVICE_OP_READ(pdevice_op_read: _Host.DEVICE_OP_READ) {
			pdevice_op_read.target_system_(some_number);
			pdevice_op_read.target_component_(some_number);
			pdevice_op_read.request_id_(some_number);
			
			pdevice_op_read.bustype_(_Host.DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
			
			pdevice_op_read.bus_(some_number);
			pdevice_op_read.address_(some_number);
			pdevice_op_read.busname_(some_string);
			pdevice_op_read.regstart_(some_number);
			pdevice_op_read.count_(some_number);
			
		}
		
		export function onMAG_CAL_PROGRESS(pmag_cal_progress: _Host.MAG_CAL_PROGRESS) {
			const some_compass_id = pmag_cal_progress.compass_id();
			const some_cal_mask   = pmag_cal_progress.cal_mask();
			{
				
				const item = pmag_cal_progress.cal_status();
				if (item !== null)
					_Host.MAG_CAL_STATUS.MAG_CAL_NOT_STARTED = item;
				
			}
			const some_attempt        = pmag_cal_progress.attempt();
			const some_completion_pct = pmag_cal_progress.completion_pct();
			{
				const item = pmag_cal_progress.completion_mask();
				for (let value of item)
					some_number = value;
				
			}
			const some_direction_x = pmag_cal_progress.direction_x();
			const some_direction_y = pmag_cal_progress.direction_y();
			const some_direction_z = pmag_cal_progress.direction_z();
			
		}
		
		export function fill_MAG_CAL_PROGRESS(pmag_cal_progress: _Host.MAG_CAL_PROGRESS) {
			pmag_cal_progress.compass_id_(some_number);
			pmag_cal_progress.cal_mask_(some_number);
			
			pmag_cal_progress.cal_status_(_Host.MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
			
			pmag_cal_progress.attempt_(some_number);
			pmag_cal_progress.completion_pct_(some_number);
			{
				const item = pmag_cal_progress.completion_mask();
				
				for (let i = 0; i < _Host.MAG_CAL_PROGRESS.completion_mask.item_len; i++)
					item.set(some_number, i);
				
			}
			pmag_cal_progress.direction_x_(some_number);
			pmag_cal_progress.direction_y_(some_number);
			pmag_cal_progress.direction_z_(some_number);
			
		}
		
		export function onHIGHRES_IMU(phighres_imu: _Host.HIGHRES_IMU) {
			const some_time_usec      = phighres_imu.time_usec();
			const some_xacc           = phighres_imu.xacc();
			const some_yacc           = phighres_imu.yacc();
			const some_zacc           = phighres_imu.zacc();
			const some_xgyro          = phighres_imu.xgyro();
			const some_ygyro          = phighres_imu.ygyro();
			const some_zgyro          = phighres_imu.zgyro();
			const some_xmag           = phighres_imu.xmag();
			const some_ymag           = phighres_imu.ymag();
			const some_zmag           = phighres_imu.zmag();
			const some_abs_pressure   = phighres_imu.abs_pressure();
			const some_diff_pressure  = phighres_imu.diff_pressure();
			const some_pressure_alt   = phighres_imu.pressure_alt();
			const some_temperature    = phighres_imu.temperature();
			const some_fields_updated = phighres_imu.fields_updated();
			
		}
		
		export function fill_HIGHRES_IMU(phighres_imu: _Host.HIGHRES_IMU) {
			phighres_imu.time_usec_(some_number);
			phighres_imu.xacc_(some_number);
			phighres_imu.yacc_(some_number);
			phighres_imu.zacc_(some_number);
			phighres_imu.xgyro_(some_number);
			phighres_imu.ygyro_(some_number);
			phighres_imu.zgyro_(some_number);
			phighres_imu.xmag_(some_number);
			phighres_imu.ymag_(some_number);
			phighres_imu.zmag_(some_number);
			phighres_imu.abs_pressure_(some_number);
			phighres_imu.diff_pressure_(some_number);
			phighres_imu.pressure_alt_(some_number);
			phighres_imu.temperature_(some_number);
			phighres_imu.fields_updated_(some_number);
			
		}
		
		export function onEXTENDED_SYS_STATE(pextended_sys_state: _Host.EXTENDED_SYS_STATE) {
			{
				
				const item = pextended_sys_state.vtol_state();
				if (item !== null)
					_Host.MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED = item;
				
			}
			{
				
				const item = pextended_sys_state.landed_state();
				if (item !== null)
					_Host.MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED = item;
				
			}
			
		}
		
		export function fill_EXTENDED_SYS_STATE(pextended_sys_state: _Host.EXTENDED_SYS_STATE) {
			
			pextended_sys_state.vtol_state_(_Host.MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
			
			
			pextended_sys_state.landed_state_(_Host.MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
			
			
		}
		
		export function onUAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic: _Host.UAVIONIX_ADSB_OUT_DYNAMIC) {
			const some_utcTime = puavionix_adsb_out_dynamic.utcTime();
			const some_gpsLat  = puavionix_adsb_out_dynamic.gpsLat();
			const some_gpsLon  = puavionix_adsb_out_dynamic.gpsLon();
			const some_gpsAlt  = puavionix_adsb_out_dynamic.gpsAlt();
			{
				
				const item = puavionix_adsb_out_dynamic.gpsFix();
				if (item !== null)
					_Host.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = item;
				
			}
			const some_numSats      = puavionix_adsb_out_dynamic.numSats();
			const some_baroAltMSL   = puavionix_adsb_out_dynamic.baroAltMSL();
			const some_accuracyHor  = puavionix_adsb_out_dynamic.accuracyHor();
			const some_accuracyVert = puavionix_adsb_out_dynamic.accuracyVert();
			const some_accuracyVel  = puavionix_adsb_out_dynamic.accuracyVel();
			const some_velVert      = puavionix_adsb_out_dynamic.velVert();
			const some_velNS        = puavionix_adsb_out_dynamic.velNS();
			const some_VelEW        = puavionix_adsb_out_dynamic.VelEW();
			{
				
				const item = puavionix_adsb_out_dynamic.emergencyStatus();
				if (item !== null)
					_Host.UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_EMERGENCY = item;
				
			}
			{
				
				const item = puavionix_adsb_out_dynamic.state();
				if (item !== null)
					_Host.UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE = item;
				
			}
			const some_squawk = puavionix_adsb_out_dynamic.squawk();
			
		}
		
		export function fill_UAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic: _Host.UAVIONIX_ADSB_OUT_DYNAMIC) {
			puavionix_adsb_out_dynamic.utcTime_(some_number);
			puavionix_adsb_out_dynamic.gpsLat_(some_number);
			puavionix_adsb_out_dynamic.gpsLon_(some_number);
			puavionix_adsb_out_dynamic.gpsAlt_(some_number);
			
			puavionix_adsb_out_dynamic.gpsFix_(_Host.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0);
			
			puavionix_adsb_out_dynamic.numSats_(some_number);
			puavionix_adsb_out_dynamic.baroAltMSL_(some_number);
			puavionix_adsb_out_dynamic.accuracyHor_(some_number);
			puavionix_adsb_out_dynamic.accuracyVert_(some_number);
			puavionix_adsb_out_dynamic.accuracyVel_(some_number);
			puavionix_adsb_out_dynamic.velVert_(some_number);
			puavionix_adsb_out_dynamic.velNS_(some_number);
			puavionix_adsb_out_dynamic.VelEW_(some_number);
			
			puavionix_adsb_out_dynamic.emergencyStatus_(_Host.UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_EMERGENCY);
			
			
			puavionix_adsb_out_dynamic.state_(_Host.UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE);
			
			puavionix_adsb_out_dynamic.squawk_(some_number);
			
		}
		
		export function onGOPRO_GET_RESPONSE(pgopro_get_response: _Host.GOPRO_GET_RESPONSE) {
			{
				
				const item = pgopro_get_response.cmd_id();
				if (item !== null)
					_Host.GOPRO_COMMAND.GOPRO_COMMAND_POWER = item;
				
			}
			{
				
				const item = pgopro_get_response.status();
				if (item !== null)
					_Host.GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS = item;
				
			}
			{
				const item = pgopro_get_response.value();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_GOPRO_GET_RESPONSE(pgopro_get_response: _Host.GOPRO_GET_RESPONSE) {
			
			pgopro_get_response.cmd_id_(_Host.GOPRO_COMMAND.GOPRO_COMMAND_POWER);
			
			
			pgopro_get_response.status_(_Host.GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
			
			{
				const item = pgopro_get_response.value();
				
				for (let i = 0; i < _Host.GOPRO_GET_RESPONSE.value.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onGPS_INJECT_DATA(pgps_inject_data: _Host.GPS_INJECT_DATA) {
			const some_target_system    = pgps_inject_data.target_system();
			const some_target_component = pgps_inject_data.target_component();
			const some_len              = pgps_inject_data.len();
			{
				const item = pgps_inject_data.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_GPS_INJECT_DATA(pgps_inject_data: _Host.GPS_INJECT_DATA) {
			pgps_inject_data.target_system_(some_number);
			pgps_inject_data.target_component_(some_number);
			pgps_inject_data.len_(some_number);
			{
				const item = pgps_inject_data.daTa();
				
				for (let i = 0; i < _Host.GPS_INJECT_DATA.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report: _Host.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) {
			{
				
				const item = puavionix_adsb_transceiver_health_report.rfHealth();
				if (item !== null)
					_Host.UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = item;
				
			}
			
		}
		
		export function fill_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report: _Host.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) {
			
			puavionix_adsb_transceiver_health_report.rfHealth_(_Host.UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING);
			
			
		}
		
		export function onATTITUDE_QUATERNION_COV(pattitude_quaternion_cov: _Host.ATTITUDE_QUATERNION_COV) {
			const some_time_usec = pattitude_quaternion_cov.time_usec();
			{
				const item = pattitude_quaternion_cov.q();
				for (let value of item)
					some_number = value;
				
			}
			const some_rollspeed  = pattitude_quaternion_cov.rollspeed();
			const some_pitchspeed = pattitude_quaternion_cov.pitchspeed();
			const some_yawspeed   = pattitude_quaternion_cov.yawspeed();
			{
				const item = pattitude_quaternion_cov.covariance();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function onNAMED_VALUE_INT(pnamed_value_int: _Host.NAMED_VALUE_INT) {
			const some_time_boot_ms = pnamed_value_int.time_boot_ms();
			{
				
				const item = pnamed_value_int.name();
				
				if (item !== null)
					some_string = item.toString();
				
			}
			const some_value = pnamed_value_int.value();
			
		}
		
		export function fill_NAMED_VALUE_INT(pnamed_value_int: _Host.NAMED_VALUE_INT) {
			pnamed_value_int.time_boot_ms_(some_number);
			pnamed_value_int.name_(some_string);
			pnamed_value_int.value_(some_number);
			
		}
		
		export function onRPM(prpm: _Host.RPM) {
			const some_rpm1 = prpm.rpm1();
			const some_rpm2 = prpm.rpm2();
			
		}
		
		export function fill_RPM(prpm: _Host.RPM) {
			prpm.rpm1_(some_number);
			prpm.rpm2_(some_number);
			
		}
		
		export function onGPS_RTCM_DATA(pgps_rtcm_data: _Host.GPS_RTCM_DATA) {
			const some_flags = pgps_rtcm_data.flags();
			const some_len   = pgps_rtcm_data.len();
			{
				const item = pgps_rtcm_data.daTa();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_GPS_RTCM_DATA(pgps_rtcm_data: _Host.GPS_RTCM_DATA) {
			pgps_rtcm_data.flags_(some_number);
			pgps_rtcm_data.len_(some_number);
			{
				const item = pgps_rtcm_data.daTa();
				
				for (let i = 0; i < _Host.GPS_RTCM_DATA.daTa.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onGLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate: _Host.GLOBAL_VISION_POSITION_ESTIMATE) {
			const some_usec  = pglobal_vision_position_estimate.usec();
			const some_x     = pglobal_vision_position_estimate.x();
			const some_y     = pglobal_vision_position_estimate.y();
			const some_z     = pglobal_vision_position_estimate.z();
			const some_roll  = pglobal_vision_position_estimate.roll();
			const some_pitch = pglobal_vision_position_estimate.pitch();
			const some_yaw   = pglobal_vision_position_estimate.yaw();
			
		}
		
		export function onFILE_TRANSFER_PROTOCOL(pfile_transfer_protocol: _Host.FILE_TRANSFER_PROTOCOL) {
			const some_target_network   = pfile_transfer_protocol.target_network();
			const some_target_system    = pfile_transfer_protocol.target_system();
			const some_target_component = pfile_transfer_protocol.target_component();
			{
				const item = pfile_transfer_protocol.payload();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol: _Host.FILE_TRANSFER_PROTOCOL) {
			pfile_transfer_protocol.target_network_(some_number);
			pfile_transfer_protocol.target_system_(some_number);
			pfile_transfer_protocol.target_component_(some_number);
			{
				const item = pfile_transfer_protocol.payload();
				
				for (let i = 0; i < _Host.FILE_TRANSFER_PROTOCOL.payload.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		export function onRANGEFINDER(prangefinder: _Host.RANGEFINDER) {
			const some_distance = prangefinder.distance();
			const some_voltage  = prangefinder.voltage();
			
		}
		
		export function fill_RANGEFINDER(prangefinder: _Host.RANGEFINDER) {
			prangefinder.distance_(some_number);
			prangefinder.voltage_(some_number);
			
		}
		
		export function onRADIO_STATUS(pradio_status: _Host.RADIO_STATUS) {
			const some_rssi     = pradio_status.rssi();
			const some_remrssi  = pradio_status.remrssi();
			const some_txbuf    = pradio_status.txbuf();
			const some_noise    = pradio_status.noise();
			const some_remnoise = pradio_status.remnoise();
			const some_rxerrors = pradio_status.rxerrors();
			const some_fixeD    = pradio_status.fixeD();
			
		}
		
		export function fill_RADIO_STATUS(pradio_status: _Host.RADIO_STATUS) {
			pradio_status.rssi_(some_number);
			pradio_status.remrssi_(some_number);
			pradio_status.txbuf_(some_number);
			pradio_status.noise_(some_number);
			pradio_status.remnoise_(some_number);
			pradio_status.rxerrors_(some_number);
			pradio_status.fixeD_(some_number);
			
		}
		
		export function onFENCE_POINT(pfence_point: _Host.FENCE_POINT) {
			const some_target_system    = pfence_point.target_system();
			const some_target_component = pfence_point.target_component();
			const some_idx              = pfence_point.idx();
			const some_count            = pfence_point.count();
			const some_lat              = pfence_point.lat();
			const some_lng              = pfence_point.lng();
			
		}
		
		export function fill_FENCE_POINT(pfence_point: _Host.FENCE_POINT) {
			pfence_point.target_system_(some_number);
			pfence_point.target_component_(some_number);
			pfence_point.idx_(some_number);
			pfence_point.count_(some_number);
			pfence_point.lat_(some_number);
			pfence_point.lng_(some_number);
			
		}
		
		export function onRESOURCE_REQUEST(presource_request: _Host.RESOURCE_REQUEST) {
			const some_request_id = presource_request.request_id();
			const some_uri_type   = presource_request.uri_type();
			{
				const item = presource_request.uri();
				for (let value of item)
					some_number = value;
				
			}
			const some_transfer_type = presource_request.transfer_type();
			{
				const item = presource_request.storage();
				for (let value of item)
					some_number = value;
				
			}
			
		}
		
		export function fill_RESOURCE_REQUEST(presource_request: _Host.RESOURCE_REQUEST) {
			presource_request.request_id_(some_number);
			presource_request.uri_type_(some_number);
			{
				const item = presource_request.uri();
				
				for (let i = 0; i < _Host.RESOURCE_REQUEST.uri.item_len; i++)
					item.set(some_number, i);
				
			}
			presource_request.transfer_type_(some_number);
			{
				const item = presource_request.storage();
				
				for (let i = 0; i < _Host.RESOURCE_REQUEST.storage.item_len; i++)
					item.set(some_number, i);
				
			}
			
		}
		
		
		class CommunicationChannel_demo extends _Host_root.CommunicationChannel {
			onRESOURCE_REQUEST(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_RESOURCE_REQUEST.forEach(h => h(this, cur.as_pack(_Host.RESOURCE_REQUEST.impl_)));
			}
			
			onATTITUDE_TARGET(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ATTITUDE_TARGET.forEach(h => h(this, cur.as_pack(_Host.ATTITUDE_TARGET.impl_)));
			}
			
			onMISSION_COUNT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_COUNT.forEach(h => h(this, cur.as_pack(_Host.MISSION_COUNT.impl_)));
			}
			
			onADSB_VEHICLE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ADSB_VEHICLE.forEach(h => h(this, cur.as_pack(_Host.ADSB_VEHICLE.impl_)));
			}
			
			onMESSAGE_INTERVAL(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MESSAGE_INTERVAL.forEach(h => h(this, cur.as_pack(_Host.MESSAGE_INTERVAL.impl_)));
			}
			
			onESTIMATOR_STATUS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ESTIMATOR_STATUS.forEach(h => h(this, cur.as_pack(_Host.ESTIMATOR_STATUS.impl_)));
			}
			
			onTIMESYNC(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_TIMESYNC.forEach(h => h(this, cur.as_pack(_Host.TIMESYNC.impl_)));
			}
			
			onGLOBAL_POSITION_INT_COV(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GLOBAL_POSITION_INT_COV.forEach(h => h(this, cur.as_pack(_Host.GLOBAL_POSITION_INT_COV.impl_)));
			}
			
			onBUTTON_CHANGE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_BUTTON_CHANGE.forEach(h => h(this, cur.as_pack(_Host.BUTTON_CHANGE.impl_)));
			}
			
			onSAFETY_SET_ALLOWED_AREA(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SAFETY_SET_ALLOWED_AREA.forEach(h => h(this, cur.as_pack(_Host.SAFETY_SET_ALLOWED_AREA.impl_)));
			}
			
			onSTORAGE_INFORMATION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_STORAGE_INFORMATION.forEach(h => h(this, cur.as_pack(_Host.STORAGE_INFORMATION.impl_)));
			}
			
			onCOLLISION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_COLLISION.forEach(h => h(this, cur.as_pack(_Host.COLLISION.impl_)));
			}
			
			onALTITUDE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ALTITUDE.forEach(h => h(this, cur.as_pack(_Host.ALTITUDE.impl_)));
			}
			
			onHIL_STATE_QUATERNION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIL_STATE_QUATERNION.forEach(h => h(this, cur.as_pack(_Host.HIL_STATE_QUATERNION.impl_)));
			}
			
			onCAMERA_INFORMATION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_CAMERA_INFORMATION.forEach(h => h(this, cur.as_pack(_Host.CAMERA_INFORMATION.impl_)));
			}
			
			onGPS_STATUS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS_STATUS.forEach(h => h(this, cur.as_pack(_Host.GPS_STATUS.impl_)));
			}
			
			onPARAM_SET(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_PARAM_SET.forEach(h => h(this, cur.as_pack(_Host.PARAM_SET.impl_)));
			}
			
			onTERRAIN_DATA(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_TERRAIN_DATA.forEach(h => h(this, cur.as_pack(_Host.TERRAIN_DATA.impl_)));
			}
			
			onRC_CHANNELS_OVERRIDE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_RC_CHANNELS_OVERRIDE.forEach(h => h(this, cur.as_pack(_Host.RC_CHANNELS_OVERRIDE.impl_)));
			}
			
			onSCALED_IMU(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SCALED_IMU.forEach(h => h(this, cur.as_pack(_Host.SCALED_IMU.impl_)));
			}
			
			onDEBUG(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_DEBUG.forEach(h => h(this, cur.as_pack(_Host.DEBUG.impl_)));
			}
			
			onCAMERA_IMAGE_CAPTURED(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_CAMERA_IMAGE_CAPTURED.forEach(h => h(this, cur.as_pack(_Host.CAMERA_IMAGE_CAPTURED.impl_)));
			}
			
			onLOG_ENTRY(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOG_ENTRY.forEach(h => h(this, cur.as_pack(_Host.LOG_ENTRY.impl_)));
			}
			
			onACTUATOR_CONTROL_TARGET(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ACTUATOR_CONTROL_TARGET.forEach(h => h(this, cur.as_pack(_Host.ACTUATOR_CONTROL_TARGET.impl_)));
			}
			
			onHIGH_LATENCY(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIGH_LATENCY.forEach(h => h(this, cur.as_pack(_Host.HIGH_LATENCY.impl_)));
			}
			
			onPARAM_REQUEST_READ(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_PARAM_REQUEST_READ.forEach(h => h(this, cur.as_pack(_Host.PARAM_REQUEST_READ.impl_)));
			}
			
			onSET_ATTITUDE_TARGET(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SET_ATTITUDE_TARGET.forEach(h => h(this, cur.as_pack(_Host.SET_ATTITUDE_TARGET.impl_)));
			}
			
			onFOLLOW_TARGET(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_FOLLOW_TARGET.forEach(h => h(this, cur.as_pack(_Host.FOLLOW_TARGET.impl_)));
			}
			
			onHIL_STATE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIL_STATE.forEach(h => h(this, cur.as_pack(_Host.HIL_STATE.impl_)));
			}
			
			onHOME_POSITION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HOME_POSITION.forEach(h => h(this, cur.as_pack(_Host.HOME_POSITION.impl_)));
			}
			
			onGPS2_RAW(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS2_RAW.forEach(h => h(this, cur.as_pack(_Host.GPS2_RAW.impl_)));
			}
			
			onMEMORY_VECT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MEMORY_VECT.forEach(h => h(this, cur.as_pack(_Host.MEMORY_VECT.impl_)));
			}
			
			onREQUEST_DATA_STREAM(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_REQUEST_DATA_STREAM.forEach(h => h(this, cur.as_pack(_Host.REQUEST_DATA_STREAM.impl_)));
			}
			
			onHIL_CONTROLS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIL_CONTROLS.forEach(h => h(this, cur.as_pack(_Host.HIL_CONTROLS.impl_)));
			}
			
			onHIL_SENSOR(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIL_SENSOR.forEach(h => h(this, cur.as_pack(_Host.HIL_SENSOR.impl_)));
			}
			
			onSETUP_SIGNING(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SETUP_SIGNING.forEach(h => h(this, cur.as_pack(_Host.SETUP_SIGNING.impl_)));
			}
			
			onGPS_RTK(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS_RTK.forEach(h => h(this, cur.as_pack(_Host.GPS_RTK.impl_)));
			}
			
			onPARAM_REQUEST_LIST(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_PARAM_REQUEST_LIST.forEach(h => h(this, cur.as_pack(_Host.PARAM_REQUEST_LIST.impl_)));
			}
			
			onLANDING_TARGET(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LANDING_TARGET.forEach(h => h(this, cur.as_pack(_Host.LANDING_TARGET.impl_)));
			}
			
			onSET_ACTUATOR_CONTROL_TARGET(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SET_ACTUATOR_CONTROL_TARGET.forEach(h => h(this, cur.as_pack(_Host.SET_ACTUATOR_CONTROL_TARGET.impl_)));
			}
			
			onCONTROL_SYSTEM_STATE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_CONTROL_SYSTEM_STATE.forEach(h => h(this, cur.as_pack(_Host.CONTROL_SYSTEM_STATE.impl_)));
			}
			
			onSET_POSITION_TARGET_GLOBAL_INT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SET_POSITION_TARGET_GLOBAL_INT.forEach(h => h(this, cur.as_pack(_Host.SET_POSITION_TARGET_GLOBAL_INT.impl_)));
			}
			
			onVIBRATION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_VIBRATION.forEach(h => h(this, cur.as_pack(_Host.VIBRATION.impl_)));
			}
			
			onPING33(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_PING33.forEach(h => h(this, cur.as_pack(_Host.PING33.impl_)));
			}
			
			onVFR_HUD(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_VFR_HUD.forEach(h => h(this, cur.as_pack(_Host.VFR_HUD.impl_)));
			}
			
			onMISSION_SET_CURRENT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_SET_CURRENT.forEach(h => h(this, cur.as_pack(_Host.MISSION_SET_CURRENT.impl_)));
			}
			
			onHIL_GPS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIL_GPS.forEach(h => h(this, cur.as_pack(_Host.HIL_GPS.impl_)));
			}
			
			onNAV_CONTROLLER_OUTPUT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_NAV_CONTROLLER_OUTPUT.forEach(h => h(this, cur.as_pack(_Host.NAV_CONTROLLER_OUTPUT.impl_)));
			}
			
			onAUTH_KEY(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_AUTH_KEY.forEach(h => h(this, cur.as_pack(_Host.AUTH_KEY.impl_)));
			}
			
			onLOCAL_POSITION_NED_COV(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOCAL_POSITION_NED_COV.forEach(h => h(this, cur.as_pack(_Host.LOCAL_POSITION_NED_COV.impl_)));
			}
			
			onATT_POS_MOCAP(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ATT_POS_MOCAP.forEach(h => h(this, cur.as_pack(_Host.ATT_POS_MOCAP.impl_)));
			}
			
			onSTATUSTEXT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_STATUSTEXT.forEach(h => h(this, cur.as_pack(_Host.STATUSTEXT.impl_)));
			}
			
			onPING(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_PING.forEach(h => h(this, cur.as_pack(_Host.PING.impl_)));
			}
			
			onCAMERA_CAPTURE_STATUS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_CAMERA_CAPTURE_STATUS.forEach(h => h(this, cur.as_pack(_Host.CAMERA_CAPTURE_STATUS.impl_)));
			}
			
			onGLOBAL_POSITION_INT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GLOBAL_POSITION_INT.forEach(h => h(this, cur.as_pack(_Host.GLOBAL_POSITION_INT.impl_)));
			}
			
			onENCAPSULATED_DATA(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ENCAPSULATED_DATA.forEach(h => h(this, cur.as_pack(_Host.ENCAPSULATED_DATA.impl_)));
			}
			
			onGPS_INPUT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS_INPUT.forEach(h => h(this, cur.as_pack(_Host.GPS_INPUT.impl_)));
			}
			
			onCOMMAND_LONG(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_COMMAND_LONG.forEach(h => h(this, cur.as_pack(_Host.COMMAND_LONG.impl_)));
			}
			
			onLOG_REQUEST_DATA(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOG_REQUEST_DATA.forEach(h => h(this, cur.as_pack(_Host.LOG_REQUEST_DATA.impl_)));
			}
			
			onGPS_RAW_INT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS_RAW_INT.forEach(h => h(this, cur.as_pack(_Host.GPS_RAW_INT.impl_)));
			}
			
			onRC_CHANNELS_SCALED(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_RC_CHANNELS_SCALED.forEach(h => h(this, cur.as_pack(_Host.RC_CHANNELS_SCALED.impl_)));
			}
			
			onCAMERA_SETTINGS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_CAMERA_SETTINGS.forEach(h => h(this, cur.as_pack(_Host.CAMERA_SETTINGS.impl_)));
			}
			
			onRAW_PRESSURE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_RAW_PRESSURE.forEach(h => h(this, cur.as_pack(_Host.RAW_PRESSURE.impl_)));
			}
			
			onNAMED_VALUE_FLOAT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_NAMED_VALUE_FLOAT.forEach(h => h(this, cur.as_pack(_Host.NAMED_VALUE_FLOAT.impl_)));
			}
			
			onATTITUDE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ATTITUDE.forEach(h => h(this, cur.as_pack(_Host.ATTITUDE.impl_)));
			}
			
			onTERRAIN_REQUEST(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_TERRAIN_REQUEST.forEach(h => h(this, cur.as_pack(_Host.TERRAIN_REQUEST.impl_)));
			}
			
			onMISSION_WRITE_PARTIAL_LIST(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_WRITE_PARTIAL_LIST.forEach(h => h(this, cur.as_pack(_Host.MISSION_WRITE_PARTIAL_LIST.impl_)));
			}
			
			onLOG_ERASE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOG_ERASE.forEach(h => h(this, cur.as_pack(_Host.LOG_ERASE.impl_)));
			}
			
			onMANUAL_SETPOINT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MANUAL_SETPOINT.forEach(h => h(this, cur.as_pack(_Host.MANUAL_SETPOINT.impl_)));
			}
			
			onSAFETY_ALLOWED_AREA(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SAFETY_ALLOWED_AREA.forEach(h => h(this, cur.as_pack(_Host.SAFETY_ALLOWED_AREA.impl_)));
			}
			
			onOPTICAL_FLOW_RAD(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_OPTICAL_FLOW_RAD.forEach(h => h(this, cur.as_pack(_Host.OPTICAL_FLOW_RAD.impl_)));
			}
			
			onLOG_DATA(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOG_DATA.forEach(h => h(this, cur.as_pack(_Host.LOG_DATA.impl_)));
			}
			
			onMISSION_CLEAR_ALL(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_CLEAR_ALL.forEach(h => h(this, cur.as_pack(_Host.MISSION_CLEAR_ALL.impl_)));
			}
			
			onVICON_POSITION_ESTIMATE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_VICON_POSITION_ESTIMATE.forEach(h => h(this, cur.as_pack(_Host.VICON_POSITION_ESTIMATE.impl_)));
			}
			
			onGPS2_RTK(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS2_RTK.forEach(h => h(this, cur.as_pack(_Host.GPS2_RTK.impl_)));
			}
			
			onLOG_REQUEST_LIST(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOG_REQUEST_LIST.forEach(h => h(this, cur.as_pack(_Host.LOG_REQUEST_LIST.impl_)));
			}
			
			onSCALED_PRESSURE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SCALED_PRESSURE.forEach(h => h(this, cur.as_pack(_Host.SCALED_PRESSURE.impl_)));
			}
			
			onMISSION_REQUEST_INT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_REQUEST_INT.forEach(h => h(this, cur.as_pack(_Host.MISSION_REQUEST_INT.impl_)));
			}
			
			onV2_EXTENSION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_V2_EXTENSION.forEach(h => h(this, cur.as_pack(_Host.V2_EXTENSION.impl_)));
			}
			
			onHEARTBEAT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HEARTBEAT.forEach(h => h(this, cur.as_pack(_Host.HEARTBEAT.impl_)));
			}
			
			onPARAM_MAP_RC(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_PARAM_MAP_RC.forEach(h => h(this, cur.as_pack(_Host.PARAM_MAP_RC.impl_)));
			}
			
			onPOWER_STATUS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_POWER_STATUS.forEach(h => h(this, cur.as_pack(_Host.POWER_STATUS.impl_)));
			}
			
			onTERRAIN_CHECK(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_TERRAIN_CHECK.forEach(h => h(this, cur.as_pack(_Host.TERRAIN_CHECK.impl_)));
			}
			
			onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.forEach(h => h(this, cur.as_pack(_Host.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.impl_)));
			}
			
			onCOMMAND_ACK(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_COMMAND_ACK.forEach(h => h(this, cur.as_pack(_Host.COMMAND_ACK.impl_)));
			}
			
			onDATA_STREAM(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_DATA_STREAM.forEach(h => h(this, cur.as_pack(_Host.DATA_STREAM.impl_)));
			}
			
			onMISSION_REQUEST(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_REQUEST.forEach(h => h(this, cur.as_pack(_Host.MISSION_REQUEST.impl_)));
			}
			
			onTERRAIN_REPORT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_TERRAIN_REPORT.forEach(h => h(this, cur.as_pack(_Host.TERRAIN_REPORT.impl_)));
			}
			
			onSET_HOME_POSITION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SET_HOME_POSITION.forEach(h => h(this, cur.as_pack(_Host.SET_HOME_POSITION.impl_)));
			}
			
			onSwitchModeCommand() {
				this.on_SwitchModeCommand.forEach(h => h(this, null!));
			}
			
			onHIL_RC_INPUTS_RAW(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIL_RC_INPUTS_RAW.forEach(h => h(this, cur.as_pack(_Host.HIL_RC_INPUTS_RAW.impl_)));
			}
			
			onSCALED_IMU3(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SCALED_IMU3.forEach(h => h(this, cur.as_pack(_Host.SCALED_IMU3.impl_)));
			}
			
			onSET_MODE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SET_MODE.forEach(h => h(this, cur.as_pack(_Host.SET_MODE.impl_)));
			}
			
			onPOSITION_TARGET_GLOBAL_INT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_POSITION_TARGET_GLOBAL_INT.forEach(h => h(this, cur.as_pack(_Host.POSITION_TARGET_GLOBAL_INT.impl_)));
			}
			
			onFLIGHT_INFORMATION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_FLIGHT_INFORMATION.forEach(h => h(this, cur.as_pack(_Host.FLIGHT_INFORMATION.impl_)));
			}
			
			onSIM_STATE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SIM_STATE.forEach(h => h(this, cur.as_pack(_Host.SIM_STATE.impl_)));
			}
			
			onMISSION_ITEM_REACHED(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_ITEM_REACHED.forEach(h => h(this, cur.as_pack(_Host.MISSION_ITEM_REACHED.impl_)));
			}
			
			onRC_CHANNELS_RAW(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_RC_CHANNELS_RAW.forEach(h => h(this, cur.as_pack(_Host.RC_CHANNELS_RAW.impl_)));
			}
			
			onSERVO_OUTPUT_RAW(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SERVO_OUTPUT_RAW.forEach(h => h(this, cur.as_pack(_Host.SERVO_OUTPUT_RAW.impl_)));
			}
			
			onVISION_SPEED_ESTIMATE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_VISION_SPEED_ESTIMATE.forEach(h => h(this, cur.as_pack(_Host.VISION_SPEED_ESTIMATE.impl_)));
			}
			
			onDEBUG_VECT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_DEBUG_VECT.forEach(h => h(this, cur.as_pack(_Host.DEBUG_VECT.impl_)));
			}
			
			onLOG_REQUEST_END(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOG_REQUEST_END.forEach(h => h(this, cur.as_pack(_Host.LOG_REQUEST_END.impl_)));
			}
			
			onMISSION_ACK(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_ACK.forEach(h => h(this, cur.as_pack(_Host.MISSION_ACK.impl_)));
			}
			
			onCHANGE_OPERATOR_CONTROL_ACK(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_CHANGE_OPERATOR_CONTROL_ACK.forEach(h => h(this, cur.as_pack(_Host.CHANGE_OPERATOR_CONTROL_ACK.impl_)));
			}
			
			onMISSION_CURRENT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_CURRENT.forEach(h => h(this, cur.as_pack(_Host.MISSION_CURRENT.impl_)));
			}
			
			onSYSTEM_TIME(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SYSTEM_TIME.forEach(h => h(this, cur.as_pack(_Host.SYSTEM_TIME.impl_)));
			}
			
			onCAMERA_TRIGGER(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_CAMERA_TRIGGER.forEach(h => h(this, cur.as_pack(_Host.CAMERA_TRIGGER.impl_)));
			}
			
			onVISION_POSITION_ESTIMATE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_VISION_POSITION_ESTIMATE.forEach(h => h(this, cur.as_pack(_Host.VISION_POSITION_ESTIMATE.impl_)));
			}
			
			onMANUAL_CONTROL(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MANUAL_CONTROL.forEach(h => h(this, cur.as_pack(_Host.MANUAL_CONTROL.impl_)));
			}
			
			onRC_CHANNELS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_RC_CHANNELS.forEach(h => h(this, cur.as_pack(_Host.RC_CHANNELS.impl_)));
			}
			
			onPARAM_VALUE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_PARAM_VALUE.forEach(h => h(this, cur.as_pack(_Host.PARAM_VALUE.impl_)));
			}
			
			onBATTERY_STATUS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_BATTERY_STATUS.forEach(h => h(this, cur.as_pack(_Host.BATTERY_STATUS.impl_)));
			}
			
			onSET_POSITION_TARGET_LOCAL_NED(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SET_POSITION_TARGET_LOCAL_NED.forEach(h => h(this, cur.as_pack(_Host.SET_POSITION_TARGET_LOCAL_NED.impl_)));
			}
			
			onSERIAL_CONTROL(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SERIAL_CONTROL.forEach(h => h(this, cur.as_pack(_Host.SERIAL_CONTROL.impl_)));
			}
			
			onSET_GPS_GLOBAL_ORIGIN(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SET_GPS_GLOBAL_ORIGIN.forEach(h => h(this, cur.as_pack(_Host.SET_GPS_GLOBAL_ORIGIN.impl_)));
			}
			
			onAUTOPILOT_VERSION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_AUTOPILOT_VERSION.forEach(h => h(this, cur.as_pack(_Host.AUTOPILOT_VERSION.impl_)));
			}
			
			onMISSION_REQUEST_LIST(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_REQUEST_LIST.forEach(h => h(this, cur.as_pack(_Host.MISSION_REQUEST_LIST.impl_)));
			}
			
			onPLAY_TUNE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_PLAY_TUNE.forEach(h => h(this, cur.as_pack(_Host.PLAY_TUNE.impl_)));
			}
			
			onSCALED_PRESSURE3(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SCALED_PRESSURE3.forEach(h => h(this, cur.as_pack(_Host.SCALED_PRESSURE3.impl_)));
			}
			
			onMISSION_REQUEST_PARTIAL_LIST(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_REQUEST_PARTIAL_LIST.forEach(h => h(this, cur.as_pack(_Host.MISSION_REQUEST_PARTIAL_LIST.impl_)));
			}
			
			onLOCAL_POSITION_NED(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_LOCAL_POSITION_NED.forEach(h => h(this, cur.as_pack(_Host.LOCAL_POSITION_NED.impl_)));
			}
			
			onDATA_TRANSMISSION_HANDSHAKE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_DATA_TRANSMISSION_HANDSHAKE.forEach(h => h(this, cur.as_pack(_Host.DATA_TRANSMISSION_HANDSHAKE.impl_)));
			}
			
			onGPS_GLOBAL_ORIGIN(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS_GLOBAL_ORIGIN.forEach(h => h(this, cur.as_pack(_Host.GPS_GLOBAL_ORIGIN.impl_)));
			}
			
			onSCALED_IMU2(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SCALED_IMU2.forEach(h => h(this, cur.as_pack(_Host.SCALED_IMU2.impl_)));
			}
			
			onATTITUDE_QUATERNION(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ATTITUDE_QUATERNION.forEach(h => h(this, cur.as_pack(_Host.ATTITUDE_QUATERNION.impl_)));
			}
			
			onHIL_ACTUATOR_CONTROLS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIL_ACTUATOR_CONTROLS.forEach(h => h(this, cur.as_pack(_Host.HIL_ACTUATOR_CONTROLS.impl_)));
			}
			
			onPOSITION_TARGET_LOCAL_NED(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_POSITION_TARGET_LOCAL_NED.forEach(h => h(this, cur.as_pack(_Host.POSITION_TARGET_LOCAL_NED.impl_)));
			}
			
			onDISTANCE_SENSOR(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_DISTANCE_SENSOR.forEach(h => h(this, cur.as_pack(_Host.DISTANCE_SENSOR.impl_)));
			}
			
			onHIL_OPTICAL_FLOW(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIL_OPTICAL_FLOW.forEach(h => h(this, cur.as_pack(_Host.HIL_OPTICAL_FLOW.impl_)));
			}
			
			onSCALED_PRESSURE2(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SCALED_PRESSURE2.forEach(h => h(this, cur.as_pack(_Host.SCALED_PRESSURE2.impl_)));
			}
			
			onWIND_COV(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_WIND_COV.forEach(h => h(this, cur.as_pack(_Host.WIND_COV.impl_)));
			}
			
			onCHANGE_OPERATOR_CONTROL(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_CHANGE_OPERATOR_CONTROL.forEach(h => h(this, cur.as_pack(_Host.CHANGE_OPERATOR_CONTROL.impl_)));
			}
			
			onSYS_STATUS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_SYS_STATUS.forEach(h => h(this, cur.as_pack(_Host.SYS_STATUS.impl_)));
			}
			
			onMISSION_ITEM(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_ITEM.forEach(h => h(this, cur.as_pack(_Host.MISSION_ITEM.impl_)));
			}
			
			onRAW_IMU(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_RAW_IMU.forEach(h => h(this, cur.as_pack(_Host.RAW_IMU.impl_)));
			}
			
			onCOMMAND_INT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_COMMAND_INT.forEach(h => h(this, cur.as_pack(_Host.COMMAND_INT.impl_)));
			}
			
			onOPTICAL_FLOW(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_OPTICAL_FLOW.forEach(h => h(this, cur.as_pack(_Host.OPTICAL_FLOW.impl_)));
			}
			
			onMISSION_ITEM_INT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_MISSION_ITEM_INT.forEach(h => h(this, cur.as_pack(_Host.MISSION_ITEM_INT.impl_)));
			}
			
			onHIGHRES_IMU(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_HIGHRES_IMU.forEach(h => h(this, cur.as_pack(_Host.HIGHRES_IMU.impl_)));
			}
			
			onEXTENDED_SYS_STATE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_EXTENDED_SYS_STATE.forEach(h => h(this, cur.as_pack(_Host.EXTENDED_SYS_STATE.impl_)));
			}
			
			onGPS_INJECT_DATA(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS_INJECT_DATA.forEach(h => h(this, cur.as_pack(_Host.GPS_INJECT_DATA.impl_)));
			}
			
			onATTITUDE_QUATERNION_COV(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_ATTITUDE_QUATERNION_COV.forEach(h => h(this, cur.as_pack(_Host.ATTITUDE_QUATERNION_COV.impl_)));
			}
			
			onNAMED_VALUE_INT(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_NAMED_VALUE_INT.forEach(h => h(this, cur.as_pack(_Host.NAMED_VALUE_INT.impl_)));
			}
			
			onRADIO_STATUS(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_RADIO_STATUS.forEach(h => h(this, cur.as_pack(_Host.RADIO_STATUS.impl_)));
			}
			
			onGPS_RTCM_DATA(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GPS_RTCM_DATA.forEach(h => h(this, cur.as_pack(_Host.GPS_RTCM_DATA.impl_)));
			}
			
			onGLOBAL_VISION_POSITION_ESTIMATE(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_GLOBAL_VISION_POSITION_ESTIMATE.forEach(h => h(this, cur.as_pack(_Host.GLOBAL_VISION_POSITION_ESTIMATE.impl_)));
			}
			
			onFILE_TRANSFER_PROTOCOL(pack: _Pack) {
				let cur = _Config.Cursor();
				cur.wrap(pack);
				this.on_FILE_TRANSFER_PROTOCOL.forEach(h => h(this, cur.as_pack(_Host.FILE_TRANSFER_PROTOCOL.impl_)));
			}
			
			readonly sendingPacks = new Array<_Pack>(5);
			
			public pullSendingPack(): _Pack | null { return this.sendingPacks.shift()!; }
			
			public pushSendingPack(pack: _Pack): boolean {
				this.sendingPacks.push(pack);
				return true;
			}
			
			public on_RESOURCE_REQUEST: Handler<CommunicationChannel_demo, _Host.RESOURCE_REQUEST>[]                                               = [];
			public on_ATTITUDE_TARGET: Handler<CommunicationChannel_demo, _Host.ATTITUDE_TARGET>[]                                                 = [];
			public on_MISSION_COUNT: Handler<CommunicationChannel_demo, _Host.MISSION_COUNT>[]                                                     = [];
			public on_ADSB_VEHICLE: Handler<CommunicationChannel_demo, _Host.ADSB_VEHICLE>[]                                                       = [];
			public on_MESSAGE_INTERVAL: Handler<CommunicationChannel_demo, _Host.MESSAGE_INTERVAL>[]                                               = [];
			public on_ESTIMATOR_STATUS: Handler<CommunicationChannel_demo, _Host.ESTIMATOR_STATUS>[]                                               = [];
			public on_TIMESYNC: Handler<CommunicationChannel_demo, _Host.TIMESYNC>[]                                                               = [];
			public on_GLOBAL_POSITION_INT_COV: Handler<CommunicationChannel_demo, _Host.GLOBAL_POSITION_INT_COV>[]                                 = [];
			public on_BUTTON_CHANGE: Handler<CommunicationChannel_demo, _Host.BUTTON_CHANGE>[]                                                     = [];
			public on_SAFETY_SET_ALLOWED_AREA: Handler<CommunicationChannel_demo, _Host.SAFETY_SET_ALLOWED_AREA>[]                                 = [];
			public on_STORAGE_INFORMATION: Handler<CommunicationChannel_demo, _Host.STORAGE_INFORMATION>[]                                         = [];
			public on_COLLISION: Handler<CommunicationChannel_demo, _Host.COLLISION>[]                                                             = [];
			public on_ALTITUDE: Handler<CommunicationChannel_demo, _Host.ALTITUDE>[]                                                               = [];
			public on_HIL_STATE_QUATERNION: Handler<CommunicationChannel_demo, _Host.HIL_STATE_QUATERNION>[]                                       = [];
			public on_CAMERA_INFORMATION: Handler<CommunicationChannel_demo, _Host.CAMERA_INFORMATION>[]                                           = [];
			public on_GPS_STATUS: Handler<CommunicationChannel_demo, _Host.GPS_STATUS>[]                                                           = [];
			public on_PARAM_SET: Handler<CommunicationChannel_demo, _Host.PARAM_SET>[]                                                             = [];
			public on_TERRAIN_DATA: Handler<CommunicationChannel_demo, _Host.TERRAIN_DATA>[]                                                       = [];
			public on_RC_CHANNELS_OVERRIDE: Handler<CommunicationChannel_demo, _Host.RC_CHANNELS_OVERRIDE>[]                                       = [];
			public on_SCALED_IMU: Handler<CommunicationChannel_demo, _Host.SCALED_IMU>[]                                                           = [];
			public on_DEBUG: Handler<CommunicationChannel_demo, _Host.DEBUG>[]                                                                     = [];
			public on_CAMERA_IMAGE_CAPTURED: Handler<CommunicationChannel_demo, _Host.CAMERA_IMAGE_CAPTURED>[]                                     = [];
			public on_LOG_ENTRY: Handler<CommunicationChannel_demo, _Host.LOG_ENTRY>[]                                                             = [];
			public on_ACTUATOR_CONTROL_TARGET: Handler<CommunicationChannel_demo, _Host.ACTUATOR_CONTROL_TARGET>[]                                 = [];
			public on_HIGH_LATENCY: Handler<CommunicationChannel_demo, _Host.HIGH_LATENCY>[]                                                       = [];
			public on_PARAM_REQUEST_READ: Handler<CommunicationChannel_demo, _Host.PARAM_REQUEST_READ>[]                                           = [];
			public on_SET_ATTITUDE_TARGET: Handler<CommunicationChannel_demo, _Host.SET_ATTITUDE_TARGET>[]                                         = [];
			public on_FOLLOW_TARGET: Handler<CommunicationChannel_demo, _Host.FOLLOW_TARGET>[]                                                     = [];
			public on_HIL_STATE: Handler<CommunicationChannel_demo, _Host.HIL_STATE>[]                                                             = [];
			public on_HOME_POSITION: Handler<CommunicationChannel_demo, _Host.HOME_POSITION>[]                                                     = [];
			public on_GPS2_RAW: Handler<CommunicationChannel_demo, _Host.GPS2_RAW>[]                                                               = [];
			public on_MEMORY_VECT: Handler<CommunicationChannel_demo, _Host.MEMORY_VECT>[]                                                         = [];
			public on_REQUEST_DATA_STREAM: Handler<CommunicationChannel_demo, _Host.REQUEST_DATA_STREAM>[]                                         = [];
			public on_HIL_CONTROLS: Handler<CommunicationChannel_demo, _Host.HIL_CONTROLS>[]                                                       = [];
			public on_HIL_SENSOR: Handler<CommunicationChannel_demo, _Host.HIL_SENSOR>[]                                                           = [];
			public on_SETUP_SIGNING: Handler<CommunicationChannel_demo, _Host.SETUP_SIGNING>[]                                                     = [];
			public on_GPS_RTK: Handler<CommunicationChannel_demo, _Host.GPS_RTK>[]                                                                 = [];
			public on_PARAM_REQUEST_LIST: Handler<CommunicationChannel_demo, _Host.PARAM_REQUEST_LIST>[]                                           = [];
			public on_LANDING_TARGET: Handler<CommunicationChannel_demo, _Host.LANDING_TARGET>[]                                                   = [];
			public on_SET_ACTUATOR_CONTROL_TARGET: Handler<CommunicationChannel_demo, _Host.SET_ACTUATOR_CONTROL_TARGET>[]                         = [];
			public on_CONTROL_SYSTEM_STATE: Handler<CommunicationChannel_demo, _Host.CONTROL_SYSTEM_STATE>[]                                       = [];
			public on_SET_POSITION_TARGET_GLOBAL_INT: Handler<CommunicationChannel_demo, _Host.SET_POSITION_TARGET_GLOBAL_INT>[]                   = [];
			public on_VIBRATION: Handler<CommunicationChannel_demo, _Host.VIBRATION>[]                                                             = [];
			public on_PING33: Handler<CommunicationChannel_demo, _Host.PING33>[]                                                                   = [];
			public on_VFR_HUD: Handler<CommunicationChannel_demo, _Host.VFR_HUD>[]                                                                 = [];
			public on_MISSION_SET_CURRENT: Handler<CommunicationChannel_demo, _Host.MISSION_SET_CURRENT>[]                                         = [];
			public on_HIL_GPS: Handler<CommunicationChannel_demo, _Host.HIL_GPS>[]                                                                 = [];
			public on_NAV_CONTROLLER_OUTPUT: Handler<CommunicationChannel_demo, _Host.NAV_CONTROLLER_OUTPUT>[]                                     = [];
			public on_AUTH_KEY: Handler<CommunicationChannel_demo, _Host.AUTH_KEY>[]                                                               = [];
			public on_LOCAL_POSITION_NED_COV: Handler<CommunicationChannel_demo, _Host.LOCAL_POSITION_NED_COV>[]                                   = [];
			public on_ATT_POS_MOCAP: Handler<CommunicationChannel_demo, _Host.ATT_POS_MOCAP>[]                                                     = [];
			public on_STATUSTEXT: Handler<CommunicationChannel_demo, _Host.STATUSTEXT>[]                                                           = [];
			public on_PING: Handler<CommunicationChannel_demo, _Host.PING>[]                                                                       = [];
			public on_CAMERA_CAPTURE_STATUS: Handler<CommunicationChannel_demo, _Host.CAMERA_CAPTURE_STATUS>[]                                     = [];
			public on_GLOBAL_POSITION_INT: Handler<CommunicationChannel_demo, _Host.GLOBAL_POSITION_INT>[]                                         = [];
			public on_ENCAPSULATED_DATA: Handler<CommunicationChannel_demo, _Host.ENCAPSULATED_DATA>[]                                             = [];
			public on_GPS_INPUT: Handler<CommunicationChannel_demo, _Host.GPS_INPUT>[]                                                             = [];
			public on_COMMAND_LONG: Handler<CommunicationChannel_demo, _Host.COMMAND_LONG>[]                                                       = [];
			public on_LOG_REQUEST_DATA: Handler<CommunicationChannel_demo, _Host.LOG_REQUEST_DATA>[]                                               = [];
			public on_GPS_RAW_INT: Handler<CommunicationChannel_demo, _Host.GPS_RAW_INT>[]                                                         = [];
			public on_RC_CHANNELS_SCALED: Handler<CommunicationChannel_demo, _Host.RC_CHANNELS_SCALED>[]                                           = [];
			public on_CAMERA_SETTINGS: Handler<CommunicationChannel_demo, _Host.CAMERA_SETTINGS>[]                                                 = [];
			public on_RAW_PRESSURE: Handler<CommunicationChannel_demo, _Host.RAW_PRESSURE>[]                                                       = [];
			public on_NAMED_VALUE_FLOAT: Handler<CommunicationChannel_demo, _Host.NAMED_VALUE_FLOAT>[]                                             = [];
			public on_ATTITUDE: Handler<CommunicationChannel_demo, _Host.ATTITUDE>[]                                                               = [];
			public on_TERRAIN_REQUEST: Handler<CommunicationChannel_demo, _Host.TERRAIN_REQUEST>[]                                                 = [];
			public on_MISSION_WRITE_PARTIAL_LIST: Handler<CommunicationChannel_demo, _Host.MISSION_WRITE_PARTIAL_LIST>[]                           = [];
			public on_LOG_ERASE: Handler<CommunicationChannel_demo, _Host.LOG_ERASE>[]                                                             = [];
			public on_MANUAL_SETPOINT: Handler<CommunicationChannel_demo, _Host.MANUAL_SETPOINT>[]                                                 = [];
			public on_SAFETY_ALLOWED_AREA: Handler<CommunicationChannel_demo, _Host.SAFETY_ALLOWED_AREA>[]                                         = [];
			public on_OPTICAL_FLOW_RAD: Handler<CommunicationChannel_demo, _Host.OPTICAL_FLOW_RAD>[]                                               = [];
			public on_LOG_DATA: Handler<CommunicationChannel_demo, _Host.LOG_DATA>[]                                                               = [];
			public on_MISSION_CLEAR_ALL: Handler<CommunicationChannel_demo, _Host.MISSION_CLEAR_ALL>[]                                             = [];
			public on_VICON_POSITION_ESTIMATE: Handler<CommunicationChannel_demo, _Host.VICON_POSITION_ESTIMATE>[]                                 = [];
			public on_GPS2_RTK: Handler<CommunicationChannel_demo, _Host.GPS2_RTK>[]                                                               = [];
			public on_LOG_REQUEST_LIST: Handler<CommunicationChannel_demo, _Host.LOG_REQUEST_LIST>[]                                               = [];
			public on_SCALED_PRESSURE: Handler<CommunicationChannel_demo, _Host.SCALED_PRESSURE>[]                                                 = [];
			public on_MISSION_REQUEST_INT: Handler<CommunicationChannel_demo, _Host.MISSION_REQUEST_INT>[]                                         = [];
			public on_V2_EXTENSION: Handler<CommunicationChannel_demo, _Host.V2_EXTENSION>[]                                                       = [];
			public on_HEARTBEAT: Handler<CommunicationChannel_demo, _Host.HEARTBEAT>[]                                                             = [];
			public on_PARAM_MAP_RC: Handler<CommunicationChannel_demo, _Host.PARAM_MAP_RC>[]                                                       = [];
			public on_POWER_STATUS: Handler<CommunicationChannel_demo, _Host.POWER_STATUS>[]                                                       = [];
			public on_TERRAIN_CHECK: Handler<CommunicationChannel_demo, _Host.TERRAIN_CHECK>[]                                                     = [];
			public on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: Handler<CommunicationChannel_demo, _Host.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET>[] = [];
			public on_COMMAND_ACK: Handler<CommunicationChannel_demo, _Host.COMMAND_ACK>[]                                                         = [];
			public on_DATA_STREAM: Handler<CommunicationChannel_demo, _Host.DATA_STREAM>[]                                                         = [];
			public on_MISSION_REQUEST: Handler<CommunicationChannel_demo, _Host.MISSION_REQUEST>[]                                                 = [];
			public on_TERRAIN_REPORT: Handler<CommunicationChannel_demo, _Host.TERRAIN_REPORT>[]                                                   = [];
			public on_SET_HOME_POSITION: Handler<CommunicationChannel_demo, _Host.SET_HOME_POSITION>[]                                             = [];
			public on_SwitchModeCommand: Handler<CommunicationChannel_demo, object>[]                                                              = [];
			public on_HIL_RC_INPUTS_RAW: Handler<CommunicationChannel_demo, _Host.HIL_RC_INPUTS_RAW>[]                                             = [];
			public on_SCALED_IMU3: Handler<CommunicationChannel_demo, _Host.SCALED_IMU3>[]                                                         = [];
			public on_SET_MODE: Handler<CommunicationChannel_demo, _Host.SET_MODE>[]                                                               = [];
			public on_POSITION_TARGET_GLOBAL_INT: Handler<CommunicationChannel_demo, _Host.POSITION_TARGET_GLOBAL_INT>[]                           = [];
			public on_FLIGHT_INFORMATION: Handler<CommunicationChannel_demo, _Host.FLIGHT_INFORMATION>[]                                           = [];
			public on_SIM_STATE: Handler<CommunicationChannel_demo, _Host.SIM_STATE>[]                                                             = [];
			public on_MISSION_ITEM_REACHED: Handler<CommunicationChannel_demo, _Host.MISSION_ITEM_REACHED>[]                                       = [];
			public on_RC_CHANNELS_RAW: Handler<CommunicationChannel_demo, _Host.RC_CHANNELS_RAW>[]                                                 = [];
			public on_SERVO_OUTPUT_RAW: Handler<CommunicationChannel_demo, _Host.SERVO_OUTPUT_RAW>[]                                               = [];
			public on_VISION_SPEED_ESTIMATE: Handler<CommunicationChannel_demo, _Host.VISION_SPEED_ESTIMATE>[]                                     = [];
			public on_DEBUG_VECT: Handler<CommunicationChannel_demo, _Host.DEBUG_VECT>[]                                                           = [];
			public on_LOG_REQUEST_END: Handler<CommunicationChannel_demo, _Host.LOG_REQUEST_END>[]                                                 = [];
			public on_MISSION_ACK: Handler<CommunicationChannel_demo, _Host.MISSION_ACK>[]                                                         = [];
			public on_CHANGE_OPERATOR_CONTROL_ACK: Handler<CommunicationChannel_demo, _Host.CHANGE_OPERATOR_CONTROL_ACK>[]                         = [];
			public on_MISSION_CURRENT: Handler<CommunicationChannel_demo, _Host.MISSION_CURRENT>[]                                                 = [];
			public on_SYSTEM_TIME: Handler<CommunicationChannel_demo, _Host.SYSTEM_TIME>[]                                                         = [];
			public on_CAMERA_TRIGGER: Handler<CommunicationChannel_demo, _Host.CAMERA_TRIGGER>[]                                                   = [];
			public on_VISION_POSITION_ESTIMATE: Handler<CommunicationChannel_demo, _Host.VISION_POSITION_ESTIMATE>[]                               = [];
			public on_MANUAL_CONTROL: Handler<CommunicationChannel_demo, _Host.MANUAL_CONTROL>[]                                                   = [];
			public on_RC_CHANNELS: Handler<CommunicationChannel_demo, _Host.RC_CHANNELS>[]                                                         = [];
			public on_PARAM_VALUE: Handler<CommunicationChannel_demo, _Host.PARAM_VALUE>[]                                                         = [];
			public on_BATTERY_STATUS: Handler<CommunicationChannel_demo, _Host.BATTERY_STATUS>[]                                                   = [];
			public on_SET_POSITION_TARGET_LOCAL_NED: Handler<CommunicationChannel_demo, _Host.SET_POSITION_TARGET_LOCAL_NED>[]                     = [];
			public on_SERIAL_CONTROL: Handler<CommunicationChannel_demo, _Host.SERIAL_CONTROL>[]                                                   = [];
			public on_SET_GPS_GLOBAL_ORIGIN: Handler<CommunicationChannel_demo, _Host.SET_GPS_GLOBAL_ORIGIN>[]                                     = [];
			public on_AUTOPILOT_VERSION: Handler<CommunicationChannel_demo, _Host.AUTOPILOT_VERSION>[]                                             = [];
			public on_MISSION_REQUEST_LIST: Handler<CommunicationChannel_demo, _Host.MISSION_REQUEST_LIST>[]                                       = [];
			public on_PLAY_TUNE: Handler<CommunicationChannel_demo, _Host.PLAY_TUNE>[]                                                             = [];
			public on_SCALED_PRESSURE3: Handler<CommunicationChannel_demo, _Host.SCALED_PRESSURE3>[]                                               = [];
			public on_MISSION_REQUEST_PARTIAL_LIST: Handler<CommunicationChannel_demo, _Host.MISSION_REQUEST_PARTIAL_LIST>[]                       = [];
			public on_LOCAL_POSITION_NED: Handler<CommunicationChannel_demo, _Host.LOCAL_POSITION_NED>[]                                           = [];
			public on_DATA_TRANSMISSION_HANDSHAKE: Handler<CommunicationChannel_demo, _Host.DATA_TRANSMISSION_HANDSHAKE>[]                         = [];
			public on_GPS_GLOBAL_ORIGIN: Handler<CommunicationChannel_demo, _Host.GPS_GLOBAL_ORIGIN>[]                                             = [];
			public on_SCALED_IMU2: Handler<CommunicationChannel_demo, _Host.SCALED_IMU2>[]                                                         = [];
			public on_ATTITUDE_QUATERNION: Handler<CommunicationChannel_demo, _Host.ATTITUDE_QUATERNION>[]                                         = [];
			public on_HIL_ACTUATOR_CONTROLS: Handler<CommunicationChannel_demo, _Host.HIL_ACTUATOR_CONTROLS>[]                                     = [];
			public on_POSITION_TARGET_LOCAL_NED: Handler<CommunicationChannel_demo, _Host.POSITION_TARGET_LOCAL_NED>[]                             = [];
			public on_DISTANCE_SENSOR: Handler<CommunicationChannel_demo, _Host.DISTANCE_SENSOR>[]                                                 = [];
			public on_HIL_OPTICAL_FLOW: Handler<CommunicationChannel_demo, _Host.HIL_OPTICAL_FLOW>[]                                               = [];
			public on_SCALED_PRESSURE2: Handler<CommunicationChannel_demo, _Host.SCALED_PRESSURE2>[]                                               = [];
			public on_WIND_COV: Handler<CommunicationChannel_demo, _Host.WIND_COV>[]                                                               = [];
			public on_CHANGE_OPERATOR_CONTROL: Handler<CommunicationChannel_demo, _Host.CHANGE_OPERATOR_CONTROL>[]                                 = [];
			public on_SYS_STATUS: Handler<CommunicationChannel_demo, _Host.SYS_STATUS>[]                                                           = [];
			public on_MISSION_ITEM: Handler<CommunicationChannel_demo, _Host.MISSION_ITEM>[]                                                       = [];
			public on_RAW_IMU: Handler<CommunicationChannel_demo, _Host.RAW_IMU>[]                                                                 = [];
			public on_COMMAND_INT: Handler<CommunicationChannel_demo, _Host.COMMAND_INT>[]                                                         = [];
			public on_OPTICAL_FLOW: Handler<CommunicationChannel_demo, _Host.OPTICAL_FLOW>[]                                                       = [];
			public on_MISSION_ITEM_INT: Handler<CommunicationChannel_demo, _Host.MISSION_ITEM_INT>[]                                               = [];
			public on_HIGHRES_IMU: Handler<CommunicationChannel_demo, _Host.HIGHRES_IMU>[]                                                         = [];
			public on_EXTENDED_SYS_STATE: Handler<CommunicationChannel_demo, _Host.EXTENDED_SYS_STATE>[]                                           = [];
			public on_GPS_INJECT_DATA: Handler<CommunicationChannel_demo, _Host.GPS_INJECT_DATA>[]                                                 = [];
			public on_ATTITUDE_QUATERNION_COV: Handler<CommunicationChannel_demo, _Host.ATTITUDE_QUATERNION_COV>[]                                 = [];
			public on_NAMED_VALUE_INT: Handler<CommunicationChannel_demo, _Host.NAMED_VALUE_INT>[]                                                 = [];
			public on_RADIO_STATUS: Handler<CommunicationChannel_demo, _Host.RADIO_STATUS>[]                                                       = [];
			public on_GPS_RTCM_DATA: Handler<CommunicationChannel_demo, _Host.GPS_RTCM_DATA>[]                                                     = [];
			public on_GLOBAL_VISION_POSITION_ESTIMATE: Handler<CommunicationChannel_demo, _Host.GLOBAL_VISION_POSITION_ESTIMATE>[]                 = [];
			public on_FILE_TRANSFER_PROTOCOL: Handler<CommunicationChannel_demo, _Host.FILE_TRANSFER_PROTOCOL>[]                                   = [];
			
		}
		
		export function Main() {
			let cur       = _Config.Cursor();
			let buff      = new Uint8Array(new ArrayBuffer(512));
			let bytes_out = 0;
			
			let CommunicationChannel_instance = new CommunicationChannel_demo();
			CommunicationChannel_instance.on_RESOURCE_REQUEST.push((ch, pack) => onRESOURCE_REQUEST(pack));
			CommunicationChannel_instance.on_ATTITUDE_TARGET.push((ch, pack) => onATTITUDE_TARGET(pack));
			CommunicationChannel_instance.on_MISSION_COUNT.push((ch, pack) => onMISSION_COUNT(pack));
			CommunicationChannel_instance.on_ADSB_VEHICLE.push((ch, pack) => onADSB_VEHICLE(pack));
			CommunicationChannel_instance.on_MESSAGE_INTERVAL.push((ch, pack) => onMESSAGE_INTERVAL(pack));
			CommunicationChannel_instance.on_ESTIMATOR_STATUS.push((ch, pack) => onESTIMATOR_STATUS(pack));
			CommunicationChannel_instance.on_TIMESYNC.push((ch, pack) => onTIMESYNC(pack));
			CommunicationChannel_instance.on_GLOBAL_POSITION_INT_COV.push((ch, pack) => onGLOBAL_POSITION_INT_COV(pack));
			CommunicationChannel_instance.on_BUTTON_CHANGE.push((ch, pack) => onBUTTON_CHANGE(pack));
			CommunicationChannel_instance.on_SAFETY_SET_ALLOWED_AREA.push((ch, pack) => onSAFETY_SET_ALLOWED_AREA(pack));
			CommunicationChannel_instance.on_STORAGE_INFORMATION.push((ch, pack) => onSTORAGE_INFORMATION(pack));
			CommunicationChannel_instance.on_COLLISION.push((ch, pack) => onCOLLISION(pack));
			CommunicationChannel_instance.on_ALTITUDE.push((ch, pack) => onALTITUDE(pack));
			CommunicationChannel_instance.on_HIL_STATE_QUATERNION.push((ch, pack) => onHIL_STATE_QUATERNION(pack));
			CommunicationChannel_instance.on_CAMERA_INFORMATION.push((ch, pack) => onCAMERA_INFORMATION(pack));
			CommunicationChannel_instance.on_GPS_STATUS.push((ch, pack) => onGPS_STATUS(pack));
			CommunicationChannel_instance.on_PARAM_SET.push((ch, pack) => onPARAM_SET(pack));
			CommunicationChannel_instance.on_TERRAIN_DATA.push((ch, pack) => onTERRAIN_DATA(pack));
			CommunicationChannel_instance.on_RC_CHANNELS_OVERRIDE.push((ch, pack) => onRC_CHANNELS_OVERRIDE(pack));
			CommunicationChannel_instance.on_SCALED_IMU.push((ch, pack) => onSCALED_IMU(pack));
			CommunicationChannel_instance.on_DEBUG.push((ch, pack) => onDEBUG(pack));
			CommunicationChannel_instance.on_CAMERA_IMAGE_CAPTURED.push((ch, pack) => onCAMERA_IMAGE_CAPTURED(pack));
			CommunicationChannel_instance.on_LOG_ENTRY.push((ch, pack) => onLOG_ENTRY(pack));
			CommunicationChannel_instance.on_ACTUATOR_CONTROL_TARGET.push((ch, pack) => onACTUATOR_CONTROL_TARGET(pack));
			CommunicationChannel_instance.on_HIGH_LATENCY.push((ch, pack) => onHIGH_LATENCY(pack));
			CommunicationChannel_instance.on_PARAM_REQUEST_READ.push((ch, pack) => onPARAM_REQUEST_READ(pack));
			CommunicationChannel_instance.on_SET_ATTITUDE_TARGET.push((ch, pack) => onSET_ATTITUDE_TARGET(pack));
			CommunicationChannel_instance.on_FOLLOW_TARGET.push((ch, pack) => onFOLLOW_TARGET(pack));
			CommunicationChannel_instance.on_HIL_STATE.push((ch, pack) => onHIL_STATE(pack));
			CommunicationChannel_instance.on_HOME_POSITION.push((ch, pack) => onHOME_POSITION(pack));
			CommunicationChannel_instance.on_GPS2_RAW.push((ch, pack) => onGPS2_RAW(pack));
			CommunicationChannel_instance.on_MEMORY_VECT.push((ch, pack) => onMEMORY_VECT(pack));
			CommunicationChannel_instance.on_REQUEST_DATA_STREAM.push((ch, pack) => onREQUEST_DATA_STREAM(pack));
			CommunicationChannel_instance.on_HIL_CONTROLS.push((ch, pack) => onHIL_CONTROLS(pack));
			CommunicationChannel_instance.on_HIL_SENSOR.push((ch, pack) => onHIL_SENSOR(pack));
			CommunicationChannel_instance.on_SETUP_SIGNING.push((ch, pack) => onSETUP_SIGNING(pack));
			CommunicationChannel_instance.on_GPS_RTK.push((ch, pack) => onGPS_RTK(pack));
			CommunicationChannel_instance.on_PARAM_REQUEST_LIST.push((ch, pack) => onPARAM_REQUEST_LIST(pack));
			CommunicationChannel_instance.on_LANDING_TARGET.push((ch, pack) => onLANDING_TARGET(pack));
			CommunicationChannel_instance.on_SET_ACTUATOR_CONTROL_TARGET.push((ch, pack) => onSET_ACTUATOR_CONTROL_TARGET(pack));
			CommunicationChannel_instance.on_CONTROL_SYSTEM_STATE.push((ch, pack) => onCONTROL_SYSTEM_STATE(pack));
			CommunicationChannel_instance.on_SET_POSITION_TARGET_GLOBAL_INT.push((ch, pack) => onSET_POSITION_TARGET_GLOBAL_INT(pack));
			CommunicationChannel_instance.on_VIBRATION.push((ch, pack) => onVIBRATION(pack));
			CommunicationChannel_instance.on_PING33.push((ch, pack) => onPING33(pack));
			CommunicationChannel_instance.on_VFR_HUD.push((ch, pack) => onVFR_HUD(pack));
			CommunicationChannel_instance.on_MISSION_SET_CURRENT.push((ch, pack) => onMISSION_SET_CURRENT(pack));
			CommunicationChannel_instance.on_HIL_GPS.push((ch, pack) => onHIL_GPS(pack));
			CommunicationChannel_instance.on_NAV_CONTROLLER_OUTPUT.push((ch, pack) => onNAV_CONTROLLER_OUTPUT(pack));
			CommunicationChannel_instance.on_AUTH_KEY.push((ch, pack) => onAUTH_KEY(pack));
			CommunicationChannel_instance.on_LOCAL_POSITION_NED_COV.push((ch, pack) => onLOCAL_POSITION_NED_COV(pack));
			CommunicationChannel_instance.on_ATT_POS_MOCAP.push((ch, pack) => onATT_POS_MOCAP(pack));
			CommunicationChannel_instance.on_STATUSTEXT.push((ch, pack) => onSTATUSTEXT(pack));
			CommunicationChannel_instance.on_PING.push((ch, pack) => onPING(pack));
			CommunicationChannel_instance.on_CAMERA_CAPTURE_STATUS.push((ch, pack) => onCAMERA_CAPTURE_STATUS(pack));
			CommunicationChannel_instance.on_GLOBAL_POSITION_INT.push((ch, pack) => onGLOBAL_POSITION_INT(pack));
			CommunicationChannel_instance.on_ENCAPSULATED_DATA.push((ch, pack) => onENCAPSULATED_DATA(pack));
			CommunicationChannel_instance.on_GPS_INPUT.push((ch, pack) => onGPS_INPUT(pack));
			CommunicationChannel_instance.on_COMMAND_LONG.push((ch, pack) => onCOMMAND_LONG(pack));
			CommunicationChannel_instance.on_LOG_REQUEST_DATA.push((ch, pack) => onLOG_REQUEST_DATA(pack));
			CommunicationChannel_instance.on_GPS_RAW_INT.push((ch, pack) => onGPS_RAW_INT(pack));
			CommunicationChannel_instance.on_RC_CHANNELS_SCALED.push((ch, pack) => onRC_CHANNELS_SCALED(pack));
			CommunicationChannel_instance.on_CAMERA_SETTINGS.push((ch, pack) => onCAMERA_SETTINGS(pack));
			CommunicationChannel_instance.on_RAW_PRESSURE.push((ch, pack) => onRAW_PRESSURE(pack));
			CommunicationChannel_instance.on_NAMED_VALUE_FLOAT.push((ch, pack) => onNAMED_VALUE_FLOAT(pack));
			CommunicationChannel_instance.on_ATTITUDE.push((ch, pack) => onATTITUDE(pack));
			CommunicationChannel_instance.on_TERRAIN_REQUEST.push((ch, pack) => onTERRAIN_REQUEST(pack));
			CommunicationChannel_instance.on_MISSION_WRITE_PARTIAL_LIST.push((ch, pack) => onMISSION_WRITE_PARTIAL_LIST(pack));
			CommunicationChannel_instance.on_LOG_ERASE.push((ch, pack) => onLOG_ERASE(pack));
			CommunicationChannel_instance.on_MANUAL_SETPOINT.push((ch, pack) => onMANUAL_SETPOINT(pack));
			CommunicationChannel_instance.on_SAFETY_ALLOWED_AREA.push((ch, pack) => onSAFETY_ALLOWED_AREA(pack));
			CommunicationChannel_instance.on_OPTICAL_FLOW_RAD.push((ch, pack) => onOPTICAL_FLOW_RAD(pack));
			CommunicationChannel_instance.on_LOG_DATA.push((ch, pack) => onLOG_DATA(pack));
			CommunicationChannel_instance.on_MISSION_CLEAR_ALL.push((ch, pack) => onMISSION_CLEAR_ALL(pack));
			CommunicationChannel_instance.on_VICON_POSITION_ESTIMATE.push((ch, pack) => onVICON_POSITION_ESTIMATE(pack));
			CommunicationChannel_instance.on_GPS2_RTK.push((ch, pack) => onGPS2_RTK(pack));
			CommunicationChannel_instance.on_LOG_REQUEST_LIST.push((ch, pack) => onLOG_REQUEST_LIST(pack));
			CommunicationChannel_instance.on_SCALED_PRESSURE.push((ch, pack) => onSCALED_PRESSURE(pack));
			CommunicationChannel_instance.on_MISSION_REQUEST_INT.push((ch, pack) => onMISSION_REQUEST_INT(pack));
			CommunicationChannel_instance.on_V2_EXTENSION.push((ch, pack) => onV2_EXTENSION(pack));
			CommunicationChannel_instance.on_HEARTBEAT.push((ch, pack) => onHEARTBEAT(pack));
			CommunicationChannel_instance.on_PARAM_MAP_RC.push((ch, pack) => onPARAM_MAP_RC(pack));
			CommunicationChannel_instance.on_POWER_STATUS.push((ch, pack) => onPOWER_STATUS(pack));
			CommunicationChannel_instance.on_TERRAIN_CHECK.push((ch, pack) => onTERRAIN_CHECK(pack));
			CommunicationChannel_instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.push((ch, pack) => onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(pack));
			CommunicationChannel_instance.on_COMMAND_ACK.push((ch, pack) => onCOMMAND_ACK(pack));
			CommunicationChannel_instance.on_DATA_STREAM.push((ch, pack) => onDATA_STREAM(pack));
			CommunicationChannel_instance.on_MISSION_REQUEST.push((ch, pack) => onMISSION_REQUEST(pack));
			CommunicationChannel_instance.on_TERRAIN_REPORT.push((ch, pack) => onTERRAIN_REPORT(pack));
			CommunicationChannel_instance.on_SET_HOME_POSITION.push((ch, pack) => onSET_HOME_POSITION(pack));
			CommunicationChannel_instance.on_SwitchModeCommand.push((ch, pack) => onSwitchModeCommand());
			CommunicationChannel_instance.on_HIL_RC_INPUTS_RAW.push((ch, pack) => onHIL_RC_INPUTS_RAW(pack));
			CommunicationChannel_instance.on_SCALED_IMU3.push((ch, pack) => onSCALED_IMU3(pack));
			CommunicationChannel_instance.on_SET_MODE.push((ch, pack) => onSET_MODE(pack));
			CommunicationChannel_instance.on_POSITION_TARGET_GLOBAL_INT.push((ch, pack) => onPOSITION_TARGET_GLOBAL_INT(pack));
			CommunicationChannel_instance.on_FLIGHT_INFORMATION.push((ch, pack) => onFLIGHT_INFORMATION(pack));
			CommunicationChannel_instance.on_SIM_STATE.push((ch, pack) => onSIM_STATE(pack));
			CommunicationChannel_instance.on_MISSION_ITEM_REACHED.push((ch, pack) => onMISSION_ITEM_REACHED(pack));
			CommunicationChannel_instance.on_RC_CHANNELS_RAW.push((ch, pack) => onRC_CHANNELS_RAW(pack));
			CommunicationChannel_instance.on_SERVO_OUTPUT_RAW.push((ch, pack) => onSERVO_OUTPUT_RAW(pack));
			CommunicationChannel_instance.on_VISION_SPEED_ESTIMATE.push((ch, pack) => onVISION_SPEED_ESTIMATE(pack));
			CommunicationChannel_instance.on_DEBUG_VECT.push((ch, pack) => onDEBUG_VECT(pack));
			CommunicationChannel_instance.on_LOG_REQUEST_END.push((ch, pack) => onLOG_REQUEST_END(pack));
			CommunicationChannel_instance.on_MISSION_ACK.push((ch, pack) => onMISSION_ACK(pack));
			CommunicationChannel_instance.on_CHANGE_OPERATOR_CONTROL_ACK.push((ch, pack) => onCHANGE_OPERATOR_CONTROL_ACK(pack));
			CommunicationChannel_instance.on_MISSION_CURRENT.push((ch, pack) => onMISSION_CURRENT(pack));
			CommunicationChannel_instance.on_SYSTEM_TIME.push((ch, pack) => onSYSTEM_TIME(pack));
			CommunicationChannel_instance.on_CAMERA_TRIGGER.push((ch, pack) => onCAMERA_TRIGGER(pack));
			CommunicationChannel_instance.on_VISION_POSITION_ESTIMATE.push((ch, pack) => onVISION_POSITION_ESTIMATE(pack));
			CommunicationChannel_instance.on_MANUAL_CONTROL.push((ch, pack) => onMANUAL_CONTROL(pack));
			CommunicationChannel_instance.on_RC_CHANNELS.push((ch, pack) => onRC_CHANNELS(pack));
			CommunicationChannel_instance.on_PARAM_VALUE.push((ch, pack) => onPARAM_VALUE(pack));
			CommunicationChannel_instance.on_BATTERY_STATUS.push((ch, pack) => onBATTERY_STATUS(pack));
			CommunicationChannel_instance.on_SET_POSITION_TARGET_LOCAL_NED.push((ch, pack) => onSET_POSITION_TARGET_LOCAL_NED(pack));
			CommunicationChannel_instance.on_SERIAL_CONTROL.push((ch, pack) => onSERIAL_CONTROL(pack));
			CommunicationChannel_instance.on_SET_GPS_GLOBAL_ORIGIN.push((ch, pack) => onSET_GPS_GLOBAL_ORIGIN(pack));
			CommunicationChannel_instance.on_AUTOPILOT_VERSION.push((ch, pack) => onAUTOPILOT_VERSION(pack));
			CommunicationChannel_instance.on_MISSION_REQUEST_LIST.push((ch, pack) => onMISSION_REQUEST_LIST(pack));
			CommunicationChannel_instance.on_PLAY_TUNE.push((ch, pack) => onPLAY_TUNE(pack));
			CommunicationChannel_instance.on_SCALED_PRESSURE3.push((ch, pack) => onSCALED_PRESSURE3(pack));
			CommunicationChannel_instance.on_MISSION_REQUEST_PARTIAL_LIST.push((ch, pack) => onMISSION_REQUEST_PARTIAL_LIST(pack));
			CommunicationChannel_instance.on_LOCAL_POSITION_NED.push((ch, pack) => onLOCAL_POSITION_NED(pack));
			CommunicationChannel_instance.on_DATA_TRANSMISSION_HANDSHAKE.push((ch, pack) => onDATA_TRANSMISSION_HANDSHAKE(pack));
			CommunicationChannel_instance.on_GPS_GLOBAL_ORIGIN.push((ch, pack) => onGPS_GLOBAL_ORIGIN(pack));
			CommunicationChannel_instance.on_SCALED_IMU2.push((ch, pack) => onSCALED_IMU2(pack));
			CommunicationChannel_instance.on_ATTITUDE_QUATERNION.push((ch, pack) => onATTITUDE_QUATERNION(pack));
			CommunicationChannel_instance.on_HIL_ACTUATOR_CONTROLS.push((ch, pack) => onHIL_ACTUATOR_CONTROLS(pack));
			CommunicationChannel_instance.on_POSITION_TARGET_LOCAL_NED.push((ch, pack) => onPOSITION_TARGET_LOCAL_NED(pack));
			CommunicationChannel_instance.on_DISTANCE_SENSOR.push((ch, pack) => onDISTANCE_SENSOR(pack));
			CommunicationChannel_instance.on_HIL_OPTICAL_FLOW.push((ch, pack) => onHIL_OPTICAL_FLOW(pack));
			CommunicationChannel_instance.on_SCALED_PRESSURE2.push((ch, pack) => onSCALED_PRESSURE2(pack));
			CommunicationChannel_instance.on_WIND_COV.push((ch, pack) => onWIND_COV(pack));
			CommunicationChannel_instance.on_CHANGE_OPERATOR_CONTROL.push((ch, pack) => onCHANGE_OPERATOR_CONTROL(pack));
			CommunicationChannel_instance.on_SYS_STATUS.push((ch, pack) => onSYS_STATUS(pack));
			CommunicationChannel_instance.on_MISSION_ITEM.push((ch, pack) => onMISSION_ITEM(pack));
			CommunicationChannel_instance.on_RAW_IMU.push((ch, pack) => onRAW_IMU(pack));
			CommunicationChannel_instance.on_COMMAND_INT.push((ch, pack) => onCOMMAND_INT(pack));
			CommunicationChannel_instance.on_OPTICAL_FLOW.push((ch, pack) => onOPTICAL_FLOW(pack));
			CommunicationChannel_instance.on_MISSION_ITEM_INT.push((ch, pack) => onMISSION_ITEM_INT(pack));
			CommunicationChannel_instance.on_HIGHRES_IMU.push((ch, pack) => onHIGHRES_IMU(pack));
			CommunicationChannel_instance.on_EXTENDED_SYS_STATE.push((ch, pack) => onEXTENDED_SYS_STATE(pack));
			CommunicationChannel_instance.on_GPS_INJECT_DATA.push((ch, pack) => onGPS_INJECT_DATA(pack));
			CommunicationChannel_instance.on_ATTITUDE_QUATERNION_COV.push((ch, pack) => onATTITUDE_QUATERNION_COV(pack));
			CommunicationChannel_instance.on_NAMED_VALUE_INT.push((ch, pack) => onNAMED_VALUE_INT(pack));
			CommunicationChannel_instance.on_RADIO_STATUS.push((ch, pack) => onRADIO_STATUS(pack));
			CommunicationChannel_instance.on_GPS_RTCM_DATA.push((ch, pack) => onGPS_RTCM_DATA(pack));
			CommunicationChannel_instance.on_GLOBAL_VISION_POSITION_ESTIMATE.push((ch, pack) => onGLOBAL_VISION_POSITION_ESTIMATE(pack));
			CommunicationChannel_instance.on_FILE_TRANSFER_PROTOCOL.push((ch, pack) => onFILE_TRANSFER_PROTOCOL(pack));
			{
				let pfollow_target = _Host_root.CommunicationChannel.NEW.FOLLOW_TARGET(cur);
				fill_FOLLOW_TARGET(pfollow_target);
				if (!CommunicationChannel_instance.send(pfollow_target))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let padsb_vehicle = _Host_root.CommunicationChannel.NEW.ADSB_VEHICLE(cur);
				fill_ADSB_VEHICLE(padsb_vehicle);
				if (!CommunicationChannel_instance.send(padsb_vehicle))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmessage_interval = _Host_root.CommunicationChannel.NEW.MESSAGE_INTERVAL(cur);
				fill_MESSAGE_INTERVAL(pmessage_interval);
				if (!CommunicationChannel_instance.send(pmessage_interval))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pekf_status_report = _Host_root.CommunicationChannel.NEW.EKF_STATUS_REPORT(cur);
				fill_EKF_STATUS_REPORT(pekf_status_report);
				if (!CommunicationChannel_instance.send(pekf_status_report))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pestimator_status = _Host_root.CommunicationChannel.NEW.ESTIMATOR_STATUS(cur);
				fill_ESTIMATOR_STATUS(pestimator_status);
				if (!CommunicationChannel_instance.send(pestimator_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let phwstatus = _Host_root.CommunicationChannel.NEW.HWSTATUS(cur);
				fill_HWSTATUS(phwstatus);
				if (!CommunicationChannel_instance.send(phwstatus))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let ptimesync = _Host_root.CommunicationChannel.NEW.TIMESYNC(cur);
				fill_TIMESYNC(ptimesync);
				if (!CommunicationChannel_instance.send(ptimesync))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pparam_ext_request_list = _Host_root.CommunicationChannel.NEW.PARAM_EXT_REQUEST_LIST(cur);
				fill_PARAM_EXT_REQUEST_LIST(pparam_ext_request_list);
				if (!CommunicationChannel_instance.send(pparam_ext_request_list))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pbutton_change = _Host_root.CommunicationChannel.NEW.BUTTON_CHANGE(cur);
				fill_BUTTON_CHANGE(pbutton_change);
				if (!CommunicationChannel_instance.send(pbutton_change))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let puavcan_node_status = _Host_root.CommunicationChannel.NEW.UAVCAN_NODE_STATUS(cur);
				fill_UAVCAN_NODE_STATUS(puavcan_node_status);
				if (!CommunicationChannel_instance.send(puavcan_node_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcollision = _Host_root.CommunicationChannel.NEW.COLLISION(cur);
				fill_COLLISION(pcollision);
				if (!CommunicationChannel_instance.send(pcollision))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgimbal_torque_cmd_report = _Host_root.CommunicationChannel.NEW.GIMBAL_TORQUE_CMD_REPORT(cur);
				fill_GIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report);
				if (!CommunicationChannel_instance.send(pgimbal_torque_cmd_report))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let paltitude = _Host_root.CommunicationChannel.NEW.ALTITUDE(cur);
				fill_ALTITUDE(paltitude);
				if (!CommunicationChannel_instance.send(paltitude))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let phil_state_quaternion = _Host_root.CommunicationChannel.NEW.HIL_STATE_QUATERNION(cur);
				fill_HIL_STATE_QUATERNION(phil_state_quaternion);
				if (!CommunicationChannel_instance.send(phil_state_quaternion))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let psensor_offsets = _Host_root.CommunicationChannel.NEW.SENSOR_OFFSETS(cur);
				fill_SENSOR_OFFSETS(psensor_offsets);
				if (!CommunicationChannel_instance.send(psensor_offsets))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pstorage_information = _Host_root.CommunicationChannel.NEW.STORAGE_INFORMATION(cur);
				fill_STORAGE_INFORMATION(pstorage_information);
				if (!CommunicationChannel_instance.send(pstorage_information))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcamera_information = _Host_root.CommunicationChannel.NEW.CAMERA_INFORMATION(cur);
				fill_CAMERA_INFORMATION(pcamera_information);
				if (!CommunicationChannel_instance.send(pcamera_information))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdevice_op_write_reply = _Host_root.CommunicationChannel.NEW.DEVICE_OP_WRITE_REPLY(cur);
				fill_DEVICE_OP_WRITE_REPLY(pdevice_op_write_reply);
				if (!CommunicationChannel_instance.send(pdevice_op_write_reply))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pterrain_data = _Host_root.CommunicationChannel.NEW.TERRAIN_DATA(cur);
				fill_TERRAIN_DATA(pterrain_data);
				if (!CommunicationChannel_instance.send(pterrain_data))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgimbal_control = _Host_root.CommunicationChannel.NEW.GIMBAL_CONTROL(cur);
				fill_GIMBAL_CONTROL(pgimbal_control);
				if (!CommunicationChannel_instance.send(pgimbal_control))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pvideo_stream_information = _Host_root.CommunicationChannel.NEW.VIDEO_STREAM_INFORMATION(cur);
				fill_VIDEO_STREAM_INFORMATION(pvideo_stream_information);
				if (!CommunicationChannel_instance.send(pvideo_stream_information))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pahrs = _Host_root.CommunicationChannel.NEW.AHRS(cur);
				fill_AHRS(pahrs);
				if (!CommunicationChannel_instance.send(pahrs))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdebug = _Host_root.CommunicationChannel.NEW.DEBUG(cur);
				fill_DEBUG(pdebug);
				if (!CommunicationChannel_instance.send(pdebug))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcamera_image_captured = _Host_root.CommunicationChannel.NEW.CAMERA_IMAGE_CAPTURED(cur);
				fill_CAMERA_IMAGE_CAPTURED(pcamera_image_captured);
				if (!CommunicationChannel_instance.send(pcamera_image_captured))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plog_entry = _Host_root.CommunicationChannel.NEW.LOG_ENTRY(cur);
				fill_LOG_ENTRY(plog_entry);
				if (!CommunicationChannel_instance.send(plog_entry))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pactuator_control_target = _Host_root.CommunicationChannel.NEW.ACTUATOR_CONTROL_TARGET(cur);
				fill_ACTUATOR_CONTROL_TARGET(pactuator_control_target);
				if (!CommunicationChannel_instance.send(pactuator_control_target))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let phigh_latency = _Host_root.CommunicationChannel.NEW.HIGH_LATENCY(cur);
				fill_HIGH_LATENCY(phigh_latency);
				if (!CommunicationChannel_instance.send(phigh_latency))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let phome_position = _Host_root.CommunicationChannel.NEW.HOME_POSITION(cur);
				fill_HOME_POSITION(phome_position);
				if (!CommunicationChannel_instance.send(phome_position))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pfence_status = _Host_root.CommunicationChannel.NEW.FENCE_STATUS(cur);
				fill_FENCE_STATUS(pfence_status);
				if (!CommunicationChannel_instance.send(pfence_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let premote_log_block_status = _Host_root.CommunicationChannel.NEW.REMOTE_LOG_BLOCK_STATUS(cur);
				fill_REMOTE_LOG_BLOCK_STATUS(premote_log_block_status);
				if (!CommunicationChannel_instance.send(premote_log_block_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pobstacle_distance = _Host_root.CommunicationChannel.NEW.OBSTACLE_DISTANCE(cur);
				fill_OBSTACLE_DISTANCE(pobstacle_distance);
				if (!CommunicationChannel_instance.send(pobstacle_distance))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgps2_raw = _Host_root.CommunicationChannel.NEW.GPS2_RAW(cur);
				fill_GPS2_RAW(pgps2_raw);
				if (!CommunicationChannel_instance.send(pgps2_raw))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmemory_vect = _Host_root.CommunicationChannel.NEW.MEMORY_VECT(cur);
				fill_MEMORY_VECT(pmemory_vect);
				if (!CommunicationChannel_instance.send(pmemory_vect))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pparam_ext_request_read = _Host_root.CommunicationChannel.NEW.PARAM_EXT_REQUEST_READ(cur);
				fill_PARAM_EXT_REQUEST_READ(pparam_ext_request_read);
				if (!CommunicationChannel_instance.send(pparam_ext_request_read))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let phil_sensor = _Host_root.CommunicationChannel.NEW.HIL_SENSOR(cur);
				fill_HIL_SENSOR(phil_sensor);
				if (!CommunicationChannel_instance.send(phil_sensor))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let psetup_signing = _Host_root.CommunicationChannel.NEW.SETUP_SIGNING(cur);
				fill_SETUP_SIGNING(psetup_signing);
				if (!CommunicationChannel_instance.send(psetup_signing))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgps_rtk = _Host_root.CommunicationChannel.NEW.GPS_RTK(cur);
				fill_GPS_RTK(pgps_rtk);
				if (!CommunicationChannel_instance.send(pgps_rtk))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let puavionix_adsb_out_cfg = _Host_root.CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_CFG(cur);
				fill_UAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg);
				if (!CommunicationChannel_instance.send(puavionix_adsb_out_cfg))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let planding_target = _Host_root.CommunicationChannel.NEW.LANDING_TARGET(cur);
				fill_LANDING_TARGET(planding_target);
				if (!CommunicationChannel_instance.send(planding_target))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pset_actuator_control_target = _Host_root.CommunicationChannel.NEW.SET_ACTUATOR_CONTROL_TARGET(cur);
				fill_SET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target);
				if (!CommunicationChannel_instance.send(pset_actuator_control_target))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcontrol_system_state = _Host_root.CommunicationChannel.NEW.CONTROL_SYSTEM_STATE(cur);
				fill_CONTROL_SYSTEM_STATE(pcontrol_system_state);
				if (!CommunicationChannel_instance.send(pcontrol_system_state))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdata32 = _Host_root.CommunicationChannel.NEW.DATA32(cur);
				fill_DATA32(pdata32);
				if (!CommunicationChannel_instance.send(pdata32))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pping33 = _Host_root.CommunicationChannel.NEW.PING33(cur);
				fill_PING33(pping33);
				if (!CommunicationChannel_instance.send(pping33))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let prally_point = _Host_root.CommunicationChannel.NEW.RALLY_POINT(cur);
				fill_RALLY_POINT(prally_point);
				if (!CommunicationChannel_instance.send(prally_point))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let padap_tuning = _Host_root.CommunicationChannel.NEW.ADAP_TUNING(cur);
				fill_ADAP_TUNING(padap_tuning);
				if (!CommunicationChannel_instance.send(padap_tuning))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pvibration = _Host_root.CommunicationChannel.NEW.VIBRATION(cur);
				fill_VIBRATION(pvibration);
				if (!CommunicationChannel_instance.send(pvibration))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pparam_ext_value = _Host_root.CommunicationChannel.NEW.PARAM_EXT_VALUE(cur);
				fill_PARAM_EXT_VALUE(pparam_ext_value);
				if (!CommunicationChannel_instance.send(pparam_ext_value))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pbattery2 = _Host_root.CommunicationChannel.NEW.BATTERY2(cur);
				fill_BATTERY2(pbattery2);
				if (!CommunicationChannel_instance.send(pbattery2))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plimits_status = _Host_root.CommunicationChannel.NEW.LIMITS_STATUS(cur);
				fill_LIMITS_STATUS(plimits_status);
				if (!CommunicationChannel_instance.send(plimits_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcamera_feedback = _Host_root.CommunicationChannel.NEW.CAMERA_FEEDBACK(cur);
				fill_CAMERA_FEEDBACK(pcamera_feedback);
				if (!CommunicationChannel_instance.send(pcamera_feedback))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let phil_gps = _Host_root.CommunicationChannel.NEW.HIL_GPS(cur);
				fill_HIL_GPS(phil_gps);
				if (!CommunicationChannel_instance.send(phil_gps))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pfence_fetch_point = _Host_root.CommunicationChannel.NEW.FENCE_FETCH_POINT(cur);
				fill_FENCE_FETCH_POINT(pfence_fetch_point);
				if (!CommunicationChannel_instance.send(pfence_fetch_point))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pradio = _Host_root.CommunicationChannel.NEW.RADIO(cur);
				fill_RADIO(pradio);
				if (!CommunicationChannel_instance.send(pradio))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pairspeed_autocal = _Host_root.CommunicationChannel.NEW.AIRSPEED_AUTOCAL(cur);
				fill_AIRSPEED_AUTOCAL(pairspeed_autocal);
				if (!CommunicationChannel_instance.send(pairspeed_autocal))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let patt_pos_mocap = _Host_root.CommunicationChannel.NEW.ATT_POS_MOCAP(cur);
				fill_ATT_POS_MOCAP(patt_pos_mocap);
				if (!CommunicationChannel_instance.send(patt_pos_mocap))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pstatustext = _Host_root.CommunicationChannel.NEW.STATUSTEXT(cur);
				fill_STATUSTEXT(pstatustext);
				if (!CommunicationChannel_instance.send(pstatustext))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgopro_get_request = _Host_root.CommunicationChannel.NEW.GOPRO_GET_REQUEST(cur);
				fill_GOPRO_GET_REQUEST(pgopro_get_request);
				if (!CommunicationChannel_instance.send(pgopro_get_request))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcamera_capture_status = _Host_root.CommunicationChannel.NEW.CAMERA_CAPTURE_STATUS(cur);
				fill_CAMERA_CAPTURE_STATUS(pcamera_capture_status);
				if (!CommunicationChannel_instance.send(pcamera_capture_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pencapsulated_data = _Host_root.CommunicationChannel.NEW.ENCAPSULATED_DATA(cur);
				fill_ENCAPSULATED_DATA(pencapsulated_data);
				if (!CommunicationChannel_instance.send(pencapsulated_data))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgps_input = _Host_root.CommunicationChannel.NEW.GPS_INPUT(cur);
				fill_GPS_INPUT(pgps_input);
				if (!CommunicationChannel_instance.send(pgps_input))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcompassmot_status = _Host_root.CommunicationChannel.NEW.COMPASSMOT_STATUS(cur);
				fill_COMPASSMOT_STATUS(pcompassmot_status);
				if (!CommunicationChannel_instance.send(pcompassmot_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plog_request_data = _Host_root.CommunicationChannel.NEW.LOG_REQUEST_DATA(cur);
				fill_LOG_REQUEST_DATA(plog_request_data);
				if (!CommunicationChannel_instance.send(plog_request_data))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcamera_status = _Host_root.CommunicationChannel.NEW.CAMERA_STATUS(cur);
				fill_CAMERA_STATUS(pcamera_status);
				if (!CommunicationChannel_instance.send(pcamera_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcamera_settings = _Host_root.CommunicationChannel.NEW.CAMERA_SETTINGS(cur);
				fill_CAMERA_SETTINGS(pcamera_settings);
				if (!CommunicationChannel_instance.send(pcamera_settings))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdevice_op_read_reply = _Host_root.CommunicationChannel.NEW.DEVICE_OP_READ_REPLY(cur);
				fill_DEVICE_OP_READ_REPLY(pdevice_op_read_reply);
				if (!CommunicationChannel_instance.send(pdevice_op_read_reply))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdigicam_control = _Host_root.CommunicationChannel.NEW.DIGICAM_CONTROL(cur);
				fill_DIGICAM_CONTROL(pdigicam_control);
				if (!CommunicationChannel_instance.send(pdigicam_control))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pnamed_value_float = _Host_root.CommunicationChannel.NEW.NAMED_VALUE_FLOAT(cur);
				fill_NAMED_VALUE_FLOAT(pnamed_value_float);
				if (!CommunicationChannel_instance.send(pnamed_value_float))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgopro_heartbeat = _Host_root.CommunicationChannel.NEW.GOPRO_HEARTBEAT(cur);
				fill_GOPRO_HEARTBEAT(pgopro_heartbeat);
				if (!CommunicationChannel_instance.send(pgopro_heartbeat))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pahrs2 = _Host_root.CommunicationChannel.NEW.AHRS2(cur);
				fill_AHRS2(pahrs2);
				if (!CommunicationChannel_instance.send(pahrs2))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plog_erase = _Host_root.CommunicationChannel.NEW.LOG_ERASE(cur);
				fill_LOG_ERASE(plog_erase);
				if (!CommunicationChannel_instance.send(plog_erase))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pterrain_request = _Host_root.CommunicationChannel.NEW.TERRAIN_REQUEST(cur);
				fill_TERRAIN_REQUEST(pterrain_request);
				if (!CommunicationChannel_instance.send(pterrain_request))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmount_status = _Host_root.CommunicationChannel.NEW.MOUNT_STATUS(cur);
				fill_MOUNT_STATUS(pmount_status);
				if (!CommunicationChannel_instance.send(pmount_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let ppid_tuning = _Host_root.CommunicationChannel.NEW.PID_TUNING(cur);
				fill_PID_TUNING(ppid_tuning);
				if (!CommunicationChannel_instance.send(ppid_tuning))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let poptical_flow_rad = _Host_root.CommunicationChannel.NEW.OPTICAL_FLOW_RAD(cur);
				fill_OPTICAL_FLOW_RAD(poptical_flow_rad);
				if (!CommunicationChannel_instance.send(poptical_flow_rad))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plog_data = _Host_root.CommunicationChannel.NEW.LOG_DATA(cur);
				fill_LOG_DATA(plog_data);
				if (!CommunicationChannel_instance.send(plog_data))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pahrs3 = _Host_root.CommunicationChannel.NEW.AHRS3(cur);
				fill_AHRS3(pahrs3);
				if (!CommunicationChannel_instance.send(pahrs3))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pvicon_position_estimate = _Host_root.CommunicationChannel.NEW.VICON_POSITION_ESTIMATE(cur);
				fill_VICON_POSITION_ESTIMATE(pvicon_position_estimate);
				if (!CommunicationChannel_instance.send(pvicon_position_estimate))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgps2_rtk = _Host_root.CommunicationChannel.NEW.GPS2_RTK(cur);
				fill_GPS2_RTK(pgps2_rtk);
				if (!CommunicationChannel_instance.send(pgps2_rtk))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmag_cal_report = _Host_root.CommunicationChannel.NEW.MAG_CAL_REPORT(cur);
				fill_MAG_CAL_REPORT(pmag_cal_report);
				if (!CommunicationChannel_instance.send(pmag_cal_report))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plog_request_list = _Host_root.CommunicationChannel.NEW.LOG_REQUEST_LIST(cur);
				fill_LOG_REQUEST_LIST(plog_request_list);
				if (!CommunicationChannel_instance.send(plog_request_list))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmount_configure = _Host_root.CommunicationChannel.NEW.MOUNT_CONFIGURE(cur);
				fill_MOUNT_CONFIGURE(pmount_configure);
				if (!CommunicationChannel_instance.send(pmount_configure))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pv2_extension = _Host_root.CommunicationChannel.NEW.V2_EXTENSION(cur);
				fill_V2_EXTENSION(pv2_extension);
				if (!CommunicationChannel_instance.send(pv2_extension))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let ppower_status = _Host_root.CommunicationChannel.NEW.POWER_STATUS(cur);
				fill_POWER_STATUS(ppower_status);
				if (!CommunicationChannel_instance.send(ppower_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let premote_log_data_block = _Host_root.CommunicationChannel.NEW.REMOTE_LOG_DATA_BLOCK(cur);
				fill_REMOTE_LOG_DATA_BLOCK(premote_log_data_block);
				if (!CommunicationChannel_instance.send(premote_log_data_block))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plogging_data_acked = _Host_root.CommunicationChannel.NEW.LOGGING_DATA_ACKED(cur);
				fill_LOGGING_DATA_ACKED(plogging_data_acked);
				if (!CommunicationChannel_instance.send(plogging_data_acked))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pterrain_check = _Host_root.CommunicationChannel.NEW.TERRAIN_CHECK(cur);
				fill_TERRAIN_CHECK(pterrain_check);
				if (!CommunicationChannel_instance.send(pterrain_check))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pterrain_report = _Host_root.CommunicationChannel.NEW.TERRAIN_REPORT(cur);
				fill_TERRAIN_REPORT(pterrain_report);
				if (!CommunicationChannel_instance.send(pterrain_report))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pset_home_position = _Host_root.CommunicationChannel.NEW.SET_HOME_POSITION(cur);
				fill_SET_HOME_POSITION(pset_home_position);
				if (!CommunicationChannel_instance.send(pset_home_position))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			if (!CommunicationChannel_instance.sendSwitchModeCommand())
				throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			{
				let pscaled_imu3 = _Host_root.CommunicationChannel.NEW.SCALED_IMU3(cur);
				fill_SCALED_IMU3(pscaled_imu3);
				if (!CommunicationChannel_instance.send(pscaled_imu3))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmount_control = _Host_root.CommunicationChannel.NEW.MOUNT_CONTROL(cur);
				fill_MOUNT_CONTROL(pmount_control);
				if (!CommunicationChannel_instance.send(pmount_control))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pled_control = _Host_root.CommunicationChannel.NEW.LED_CONTROL(cur);
				fill_LED_CONTROL(pled_control);
				if (!CommunicationChannel_instance.send(pled_control))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let psim_state = _Host_root.CommunicationChannel.NEW.SIM_STATE(cur);
				fill_SIM_STATE(psim_state);
				if (!CommunicationChannel_instance.send(psim_state))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pwifi_config_ap = _Host_root.CommunicationChannel.NEW.WIFI_CONFIG_AP(cur);
				fill_WIFI_CONFIG_AP(pwifi_config_ap);
				if (!CommunicationChannel_instance.send(pwifi_config_ap))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdata96 = _Host_root.CommunicationChannel.NEW.DATA96(cur);
				fill_DATA96(pdata96);
				if (!CommunicationChannel_instance.send(pdata96))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pflight_information = _Host_root.CommunicationChannel.NEW.FLIGHT_INFORMATION(cur);
				fill_FLIGHT_INFORMATION(pflight_information);
				if (!CommunicationChannel_instance.send(pflight_information))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmeminfo = _Host_root.CommunicationChannel.NEW.MEMINFO(cur);
				fill_MEMINFO(pmeminfo);
				if (!CommunicationChannel_instance.send(pmeminfo))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plogging_ack = _Host_root.CommunicationChannel.NEW.LOGGING_ACK(cur);
				fill_LOGGING_ACK(plogging_ack);
				if (!CommunicationChannel_instance.send(plogging_ack))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pvision_speed_estimate = _Host_root.CommunicationChannel.NEW.VISION_SPEED_ESTIMATE(cur);
				fill_VISION_SPEED_ESTIMATE(pvision_speed_estimate);
				if (!CommunicationChannel_instance.send(pvision_speed_estimate))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdebug_vect = _Host_root.CommunicationChannel.NEW.DEBUG_VECT(cur);
				fill_DEBUG_VECT(pdebug_vect);
				if (!CommunicationChannel_instance.send(pdebug_vect))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pcamera_trigger = _Host_root.CommunicationChannel.NEW.CAMERA_TRIGGER(cur);
				fill_CAMERA_TRIGGER(pcamera_trigger);
				if (!CommunicationChannel_instance.send(pcamera_trigger))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plog_request_end = _Host_root.CommunicationChannel.NEW.LOG_REQUEST_END(cur);
				fill_LOG_REQUEST_END(plog_request_end);
				if (!CommunicationChannel_instance.send(plog_request_end))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgopro_set_response = _Host_root.CommunicationChannel.NEW.GOPRO_SET_RESPONSE(cur);
				fill_GOPRO_SET_RESPONSE(pgopro_set_response);
				if (!CommunicationChannel_instance.send(pgopro_set_response))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pprotocol_version = _Host_root.CommunicationChannel.NEW.PROTOCOL_VERSION(cur);
				fill_PROTOCOL_VERSION(pprotocol_version);
				if (!CommunicationChannel_instance.send(pprotocol_version))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let prally_fetch_point = _Host_root.CommunicationChannel.NEW.RALLY_FETCH_POINT(cur);
				fill_RALLY_FETCH_POINT(prally_fetch_point);
				if (!CommunicationChannel_instance.send(prally_fetch_point))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pbattery_status = _Host_root.CommunicationChannel.NEW.BATTERY_STATUS(cur);
				fill_BATTERY_STATUS(pbattery_status);
				if (!CommunicationChannel_instance.send(pbattery_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmount_orientation = _Host_root.CommunicationChannel.NEW.MOUNT_ORIENTATION(cur);
				fill_MOUNT_ORIENTATION(pmount_orientation);
				if (!CommunicationChannel_instance.send(pmount_orientation))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pserial_control = _Host_root.CommunicationChannel.NEW.SERIAL_CONTROL(cur);
				fill_SERIAL_CONTROL(pserial_control);
				if (!CommunicationChannel_instance.send(pserial_control))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pparam_ext_set = _Host_root.CommunicationChannel.NEW.PARAM_EXT_SET(cur);
				fill_PARAM_EXT_SET(pparam_ext_set);
				if (!CommunicationChannel_instance.send(pparam_ext_set))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pautopilot_version = _Host_root.CommunicationChannel.NEW.AUTOPILOT_VERSION(cur);
				fill_AUTOPILOT_VERSION(pautopilot_version);
				if (!CommunicationChannel_instance.send(pautopilot_version))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let psimstate = _Host_root.CommunicationChannel.NEW.SIMSTATE(cur);
				fill_SIMSTATE(psimstate);
				if (!CommunicationChannel_instance.send(psimstate))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pset_video_stream_settings = _Host_root.CommunicationChannel.NEW.SET_VIDEO_STREAM_SETTINGS(cur);
				fill_SET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings);
				if (!CommunicationChannel_instance.send(pset_video_stream_settings))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pplay_tune = _Host_root.CommunicationChannel.NEW.PLAY_TUNE(cur);
				fill_PLAY_TUNE(pplay_tune);
				if (!CommunicationChannel_instance.send(pplay_tune))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdigicam_configure = _Host_root.CommunicationChannel.NEW.DIGICAM_CONFIGURE(cur);
				fill_DIGICAM_CONFIGURE(pdigicam_configure);
				if (!CommunicationChannel_instance.send(pdigicam_configure))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pscaled_pressure3 = _Host_root.CommunicationChannel.NEW.SCALED_PRESSURE3(cur);
				fill_SCALED_PRESSURE3(pscaled_pressure3);
				if (!CommunicationChannel_instance.send(pscaled_pressure3))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pparam_ext_ack = _Host_root.CommunicationChannel.NEW.PARAM_EXT_ACK(cur);
				fill_PARAM_EXT_ACK(pparam_ext_ack);
				if (!CommunicationChannel_instance.send(pparam_ext_ack))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let puavcan_node_info = _Host_root.CommunicationChannel.NEW.UAVCAN_NODE_INFO(cur);
				fill_UAVCAN_NODE_INFO(puavcan_node_info);
				if (!CommunicationChannel_instance.send(puavcan_node_info))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdata16 = _Host_root.CommunicationChannel.NEW.DATA16(cur);
				fill_DATA16(pdata16);
				if (!CommunicationChannel_instance.send(pdata16))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pset_mag_offsets = _Host_root.CommunicationChannel.NEW.SET_MAG_OFFSETS(cur);
				fill_SET_MAG_OFFSETS(pset_mag_offsets);
				if (!CommunicationChannel_instance.send(pset_mag_offsets))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pscaled_imu2 = _Host_root.CommunicationChannel.NEW.SCALED_IMU2(cur);
				fill_SCALED_IMU2(pscaled_imu2);
				if (!CommunicationChannel_instance.send(pscaled_imu2))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pap_adc = _Host_root.CommunicationChannel.NEW.AP_ADC(cur);
				fill_AP_ADC(pap_adc);
				if (!CommunicationChannel_instance.send(pap_adc))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pwind = _Host_root.CommunicationChannel.NEW.WIND(cur);
				fill_WIND(pwind);
				if (!CommunicationChannel_instance.send(pwind))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pautopilot_version_request = _Host_root.CommunicationChannel.NEW.AUTOPILOT_VERSION_REQUEST(cur);
				fill_AUTOPILOT_VERSION_REQUEST(pautopilot_version_request);
				if (!CommunicationChannel_instance.send(pautopilot_version_request))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdata_transmission_handshake = _Host_root.CommunicationChannel.NEW.DATA_TRANSMISSION_HANDSHAKE(cur);
				fill_DATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake);
				if (!CommunicationChannel_instance.send(pdata_transmission_handshake))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdata64 = _Host_root.CommunicationChannel.NEW.DATA64(cur);
				fill_DATA64(pdata64);
				if (!CommunicationChannel_instance.send(pdata64))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgimbal_report = _Host_root.CommunicationChannel.NEW.GIMBAL_REPORT(cur);
				fill_GIMBAL_REPORT(pgimbal_report);
				if (!CommunicationChannel_instance.send(pgimbal_report))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdevice_op_write = _Host_root.CommunicationChannel.NEW.DEVICE_OP_WRITE(cur);
				fill_DEVICE_OP_WRITE(pdevice_op_write);
				if (!CommunicationChannel_instance.send(pdevice_op_write))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdistance_sensor = _Host_root.CommunicationChannel.NEW.DISTANCE_SENSOR(cur);
				fill_DISTANCE_SENSOR(pdistance_sensor);
				if (!CommunicationChannel_instance.send(pdistance_sensor))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let phil_optical_flow = _Host_root.CommunicationChannel.NEW.HIL_OPTICAL_FLOW(cur);
				fill_HIL_OPTICAL_FLOW(phil_optical_flow);
				if (!CommunicationChannel_instance.send(phil_optical_flow))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pscaled_pressure2 = _Host_root.CommunicationChannel.NEW.SCALED_PRESSURE2(cur);
				fill_SCALED_PRESSURE2(pscaled_pressure2);
				if (!CommunicationChannel_instance.send(pscaled_pressure2))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pwind_cov = _Host_root.CommunicationChannel.NEW.WIND_COV(cur);
				fill_WIND_COV(pwind_cov);
				if (!CommunicationChannel_instance.send(pwind_cov))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgopro_set_request = _Host_root.CommunicationChannel.NEW.GOPRO_SET_REQUEST(cur);
				fill_GOPRO_SET_REQUEST(pgopro_set_request);
				if (!CommunicationChannel_instance.send(pgopro_set_request))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pvision_position_delta = _Host_root.CommunicationChannel.NEW.VISION_POSITION_DELTA(cur);
				fill_VISION_POSITION_DELTA(pvision_position_delta);
				if (!CommunicationChannel_instance.send(pvision_position_delta))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let plogging_data = _Host_root.CommunicationChannel.NEW.LOGGING_DATA(cur);
				fill_LOGGING_DATA(plogging_data);
				if (!CommunicationChannel_instance.send(plogging_data))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pdevice_op_read = _Host_root.CommunicationChannel.NEW.DEVICE_OP_READ(cur);
				fill_DEVICE_OP_READ(pdevice_op_read);
				if (!CommunicationChannel_instance.send(pdevice_op_read))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pmag_cal_progress = _Host_root.CommunicationChannel.NEW.MAG_CAL_PROGRESS(cur);
				fill_MAG_CAL_PROGRESS(pmag_cal_progress);
				if (!CommunicationChannel_instance.send(pmag_cal_progress))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let phighres_imu = _Host_root.CommunicationChannel.NEW.HIGHRES_IMU(cur);
				fill_HIGHRES_IMU(phighres_imu);
				if (!CommunicationChannel_instance.send(phighres_imu))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pextended_sys_state = _Host_root.CommunicationChannel.NEW.EXTENDED_SYS_STATE(cur);
				fill_EXTENDED_SYS_STATE(pextended_sys_state);
				if (!CommunicationChannel_instance.send(pextended_sys_state))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let puavionix_adsb_out_dynamic = _Host_root.CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_DYNAMIC(cur);
				fill_UAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic);
				if (!CommunicationChannel_instance.send(puavionix_adsb_out_dynamic))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgopro_get_response = _Host_root.CommunicationChannel.NEW.GOPRO_GET_RESPONSE(cur);
				fill_GOPRO_GET_RESPONSE(pgopro_get_response);
				if (!CommunicationChannel_instance.send(pgopro_get_response))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgps_inject_data = _Host_root.CommunicationChannel.NEW.GPS_INJECT_DATA(cur);
				fill_GPS_INJECT_DATA(pgps_inject_data);
				if (!CommunicationChannel_instance.send(pgps_inject_data))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let puavionix_adsb_transceiver_health_report = _Host_root.CommunicationChannel.NEW.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur);
				fill_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report);
				if (!CommunicationChannel_instance.send(puavionix_adsb_transceiver_health_report))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pnamed_value_int = _Host_root.CommunicationChannel.NEW.NAMED_VALUE_INT(cur);
				fill_NAMED_VALUE_INT(pnamed_value_int);
				if (!CommunicationChannel_instance.send(pnamed_value_int))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let prpm = _Host_root.CommunicationChannel.NEW.RPM(cur);
				fill_RPM(prpm);
				if (!CommunicationChannel_instance.send(prpm))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pgps_rtcm_data = _Host_root.CommunicationChannel.NEW.GPS_RTCM_DATA(cur);
				fill_GPS_RTCM_DATA(pgps_rtcm_data);
				if (!CommunicationChannel_instance.send(pgps_rtcm_data))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pfile_transfer_protocol = _Host_root.CommunicationChannel.NEW.FILE_TRANSFER_PROTOCOL(cur);
				fill_FILE_TRANSFER_PROTOCOL(pfile_transfer_protocol);
				if (!CommunicationChannel_instance.send(pfile_transfer_protocol))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let prangefinder = _Host_root.CommunicationChannel.NEW.RANGEFINDER(cur);
				fill_RANGEFINDER(prangefinder);
				if (!CommunicationChannel_instance.send(prangefinder))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pradio_status = _Host_root.CommunicationChannel.NEW.RADIO_STATUS(cur);
				fill_RADIO_STATUS(pradio_status);
				if (!CommunicationChannel_instance.send(pradio_status))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let pfence_point = _Host_root.CommunicationChannel.NEW.FENCE_POINT(cur);
				fill_FENCE_POINT(pfence_point);
				if (!CommunicationChannel_instance.send(pfence_point))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			{
				let presource_request = _Host_root.CommunicationChannel.NEW.RESOURCE_REQUEST(cur);
				fill_RESOURCE_REQUEST(presource_request);
				if (!CommunicationChannel_instance.send(presource_request))   //put pack into the channel send-buffer
					throw new Error("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW");
			}
			
			bytes_out = CommunicationChannel_instance.transmitter.packs_into_bytes(buff, 0, buff.byteLength);// sending packs
			CommunicationChannel_instance.receiver.bytes_into_packs(buff, 0, buff.byteLength);// receiving packs
			
		}
	}
}
					  