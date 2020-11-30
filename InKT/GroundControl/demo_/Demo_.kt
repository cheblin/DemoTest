package demo_

import  com.company.demo.GroundControl.*
import org.unirail.AdHoc
import java.util.concurrent.ConcurrentLinkedQueue
import com.company.demo.GroundControl.util_.*
import kotlin.random.Random

var some_MAV_RESULT = MAV_RESULT.MAV_RESULT_ACCEPTED
var some_MAV_PARAM_TYPE = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8
var some_LANDING_TARGET_TYPE = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON
var some_GPS_FIX_TYPE = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS
var some_CAMERA_MODE = CAMERA_MODE.CAMERA_MODE_IMAGE
var some_MAV_BATTERY_TYPE = MAV_BATTERY_TYPE.UNKNOWN
var some_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0
var some_MAV_FRAME = MAV_FRAME.MAV_FRAME_GLOBAL
var some_ADSB_ALTITUDE_TYPE = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH
var some_EKF_STATUS_FLAGS = EKF_STATUS_FLAGS.EKF_ATTITUDE
var some_MAV_SENSOR_ORIENTATION = MAV_SENSOR_ORIENTATION.NONE
var some_PARAM_ACK = PARAM_ACK.PARAM_ACK_ACCEPTED
var some_UAVIONIX_ADSB_RF_HEALTH = UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING
var some_MAG_CAL_STATUS = MAG_CAL_STATUS.MAG_CAL_NOT_STARTED
var some_RALLY_FLAGS = RALLY_FLAGS.FAVORABLE_WIND
var some_CAMERA_FEEDBACK_FLAGS = CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO
var some_PID_TUNING_AXIS = PID_TUNING_AXIS.PID_TUNING_ROLL
var some_UAVIONIX_ADSB_EMERGENCY_STATUS = UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_EMERGENCY
var some_FENCE_BREACH = FENCE_BREACH.FENCE_BREACH_NONE
var some_ESTIMATOR_STATUS_FLAGS = ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE
var some_MAV_MISSION_RESULT = MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED
var some_DEVICE_OP_BUSTYPE = DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C
var some_MAV_COLLISION_THREAT_LEVEL = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE
var some_ADSB_FLAGS = ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS
var some_MAV_CMD = MAV_CMD.MAV_CMD_NAV_WAYPOINT
var some_UAVIONIX_ADSB_OUT_RF_SELECT = UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY
var some_MAV_DISTANCE_SENSOR = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER
var some_ADSB_EMITTER_TYPE = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO
var some_SERIAL_CONTROL_FLAG = SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY
var some_LIMIT_MODULE = LIMIT_MODULE.LIMIT_GPSLOCK
var some_MAV_SEVERITY = MAV_SEVERITY.MAV_SEVERITY_EMERGENCY
var some_MAV_PROTOCOL_CAPABILITY = MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
var some_MAV_POWER_STATUS = MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID
var some_MAV_ESTIMATOR_TYPE = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE
var some_MAV_MODE = MAV_MODE.PREFLIGHT
var some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.NO_DATA
var some_MAV_COLLISION_SRC = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB
var some_Int = 0.toInt()
var some_CAMERA_CAP_FLAGS = CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO
var some_UAVIONIX_ADSB_OUT_DYNAMIC_STATE = UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE
var some_MAV_AUTOPILOT = MAV_AUTOPILOT.GENERIC
var some_MAV_MISSION_TYPE = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION
var some_GOPRO_HEARTBEAT_FLAGS = GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING
var some_MAV_MODE_FLAG = MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
var some_MAV_COLLISION_ACTION = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE
var some_MAV_STATE = MAV_STATE.UNINIT
var some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA
var some_GPS_INPUT_IGNORE_FLAGS = GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT
var some_CAMERA_STATUS_TYPES = CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_HEARTBEAT
var some_MAV_TYPE = MAV_TYPE.GENERIC
var some_MAV_BATTERY_FUNCTION = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN
var some_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP
var some_UAVCAN_NODE_MODE = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL
var some_MAV_PARAM_EXT_TYPE = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8
var some_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK
var some_GOPRO_REQUEST_STATUS = GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS
var some_MAV_SYS_STATUS_SENSOR = MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO
var some_GOPRO_HEARTBEAT_STATUS = GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_DISCONNECTED
var some_SERIAL_CONTROL_DEV = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1
var some_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA
var some_GOPRO_CAPTURE_MODE = GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO
var some_MAV_VTOL_STATE = MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED
var some_MAV_MOUNT_MODE = MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT
var some_GOPRO_COMMAND = GOPRO_COMMAND.GOPRO_COMMAND_POWER
var some_UAVCAN_NODE_HEALTH = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK
var some_MAV_LANDED_STATE = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED
var some_LIMITS_STATE = LIMITS_STATE.LIMITS_INIT


fun onATTITUDE_TARGET(pattitude_target: ATTITUDE_TARGET) {
    val some_time_boot_ms = pattitude_target.time_boot_ms()
    val some_type_mask = pattitude_target.type_mask()
    pattitude_target.q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_body_roll_rate = pattitude_target.body_roll_rate()
    val some_body_pitch_rate = pattitude_target.body_pitch_rate()
    val some_body_yaw_rate = pattitude_target.body_yaw_rate()
    val some_thrust = pattitude_target.thrust()

}

fun test_.ATTITUDE_TARGET.copyFrom(SRC: ATTITUDE_TARGET) {
    ATTITUDE_TARGET.push(
            SRC, { src -> time_boot_ms(src) }, { src -> type_mask(src) }, { src ->
        val item = q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> body_roll_rate(src) }, { src -> body_pitch_rate(src) }, { src -> body_yaw_rate(src) }, { src -> thrust(src) }

    )
}

fun onMISSION_COUNT(pmission_count: MISSION_COUNT) {
    val some_target_system = pmission_count.target_system()
    val some_target_component = pmission_count.target_component()
    val some_count = pmission_count.count()

    pmission_count.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_COUNT.copyFrom(SRC: MISSION_COUNT) {
    MISSION_COUNT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> count(src) }, { src -> mission_type(src) }

    )
}

fun onADSB_VEHICLE(padsb_vehicle: ADSB_VEHICLE) {
    val some_ICAO_address = padsb_vehicle.ICAO_address()
    val some_lat = padsb_vehicle.lat()
    val some_lon = padsb_vehicle.lon()

    padsb_vehicle.altitude_type()?.let { item ->
        some_ADSB_ALTITUDE_TYPE = item.get()
    }
    val some_altitude = padsb_vehicle.altitude()
    val some_heading = padsb_vehicle.heading()
    val some_hor_velocity = padsb_vehicle.hor_velocity()
    val some_ver_velocity = padsb_vehicle.ver_velocity()

    padsb_vehicle.callsign()?.let { item -> char = item.get() }

    padsb_vehicle.emitter_type()?.let { item ->
        some_ADSB_EMITTER_TYPE = item.get()
    }
    val some_tslc = padsb_vehicle.tslc()

    padsb_vehicle.flags()?.let { item ->
        some_ADSB_FLAGS = item.get()
    }
    val some_squawk = padsb_vehicle.squawk()

}

fun ADSB_VEHICLE.copyFrom(SRC: ADSB_VEHICLE) {
    ADSB_VEHICLE.push(
            SRC, { src -> ICAO_address(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> altitude_type(src) }, { src -> altitude(src) }, { src -> heading(src) }, { src -> hor_velocity(src) }, { src -> ver_velocity(src) }, { src ->
        val item = callsign(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> emitter_type(src) }, { src -> tslc(src) }, { src -> flags(src) }, { src -> squawk(src) }

    )
}

fun fill(padsb_vehicle: ADSB_VEHICLE) {
    padsb_vehicle.ICAO_address(Int)
    padsb_vehicle.lat(Int)
    padsb_vehicle.lon(Int)

    padsb_vehicle.altitude_type(some_ADSB_ALTITUDE_TYPE)
    padsb_vehicle.altitude(Int)
    padsb_vehicle.heading(Short)
    padsb_vehicle.hor_velocity(Short)
    padsb_vehicle.ver_velocity(Short)

    padsb_vehicle.callsign(some_String, null)

    padsb_vehicle.emitter_type(some_ADSB_EMITTER_TYPE)
    padsb_vehicle.tslc(Byte)

    padsb_vehicle.flags(some_ADSB_FLAGS)
    padsb_vehicle.squawk(Short)

}

fun ADSB_VEHICLE.copyInto(DST: ADSB_VEHICLE) {
    ADSB_VEHICLE.pull(
            DST, { -> ICAO_address() }, { -> lat() }, { -> lon() }, { altitude_type() != null }, { altitude_type()!!.get() }, { -> altitude() }, { -> heading() }, { -> hor_velocity() }, { -> ver_velocity() }, { callsign()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = callsign()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { emitter_type() != null }, { emitter_type()!!.get() }, { -> tslc() }, { flags() != null }, { flags()!!.get() }, { -> squawk() }

    )
}

fun onMESSAGE_INTERVAL(pmessage_interval: MESSAGE_INTERVAL) {
    val some_message_id = pmessage_interval.message_id()
    val some_interval_us = pmessage_interval.interval_us()

}

fun MESSAGE_INTERVAL.copyFrom(SRC: MESSAGE_INTERVAL) {
    MESSAGE_INTERVAL.push(
            SRC, { src -> message_id(src) }, { src -> interval_us(src) }

    )
}

fun fill(pmessage_interval: MESSAGE_INTERVAL) {
    pmessage_interval.message_id(Short)
    pmessage_interval.interval_us(Int)

}

fun MESSAGE_INTERVAL.copyInto(DST: MESSAGE_INTERVAL) {
    MESSAGE_INTERVAL.pull(
            DST, { -> message_id() }, { -> interval_us() }

    )
}

fun onEKF_STATUS_REPORT(pekf_status_report: EKF_STATUS_REPORT) {

    pekf_status_report.flags()?.let { item ->
        some_EKF_STATUS_FLAGS = item.get()
    }
    val some_velocity_variance = pekf_status_report.velocity_variance()
    val some_pos_horiz_variance = pekf_status_report.pos_horiz_variance()
    val some_pos_vert_variance = pekf_status_report.pos_vert_variance()
    val some_compass_variance = pekf_status_report.compass_variance()
    val some_terrain_alt_variance = pekf_status_report.terrain_alt_variance()

}

fun EKF_STATUS_REPORT.copyFrom(SRC: EKF_STATUS_REPORT) {
    EKF_STATUS_REPORT.push(
            SRC, { src -> flags(src) }, { src -> velocity_variance(src) }, { src -> pos_horiz_variance(src) }, { src -> pos_vert_variance(src) }, { src -> compass_variance(src) }, { src -> terrain_alt_variance(src) }

    )
}

fun fill(pekf_status_report: EKF_STATUS_REPORT) {

    pekf_status_report.flags(some_EKF_STATUS_FLAGS)
    pekf_status_report.velocity_variance(Float)
    pekf_status_report.pos_horiz_variance(Float)
    pekf_status_report.pos_vert_variance(Float)
    pekf_status_report.compass_variance(Float)
    pekf_status_report.terrain_alt_variance(Float)

}

fun EKF_STATUS_REPORT.copyInto(DST: EKF_STATUS_REPORT) {
    EKF_STATUS_REPORT.pull(
            DST, { flags() != null }, { flags()!!.get() }, { -> velocity_variance() }, { -> pos_horiz_variance() }, { -> pos_vert_variance() }, { -> compass_variance() }, { -> terrain_alt_variance() }

    )
}

fun onESTIMATOR_STATUS(pestimator_status: ESTIMATOR_STATUS) {
    val some_time_usec = pestimator_status.time_usec()

    pestimator_status.flags()?.let { item ->
        some_ESTIMATOR_STATUS_FLAGS = item.get()
    }
    val some_vel_ratio = pestimator_status.vel_ratio()
    val some_pos_horiz_ratio = pestimator_status.pos_horiz_ratio()
    val some_pos_vert_ratio = pestimator_status.pos_vert_ratio()
    val some_mag_ratio = pestimator_status.mag_ratio()
    val some_hagl_ratio = pestimator_status.hagl_ratio()
    val some_tas_ratio = pestimator_status.tas_ratio()
    val some_pos_horiz_accuracy = pestimator_status.pos_horiz_accuracy()
    val some_pos_vert_accuracy = pestimator_status.pos_vert_accuracy()

}

fun ESTIMATOR_STATUS.copyFrom(SRC: ESTIMATOR_STATUS) {
    ESTIMATOR_STATUS.push(
            SRC, { src -> time_usec(src) }, { src -> flags(src) }, { src -> vel_ratio(src) }, { src -> pos_horiz_ratio(src) }, { src -> pos_vert_ratio(src) }, { src -> mag_ratio(src) }, { src -> hagl_ratio(src) }, { src -> tas_ratio(src) }, { src -> pos_horiz_accuracy(src) }, { src -> pos_vert_accuracy(src) }

    )
}

fun fill(pestimator_status: ESTIMATOR_STATUS) {
    pestimator_status.time_usec(Long)

    pestimator_status.flags(some_ESTIMATOR_STATUS_FLAGS)
    pestimator_status.vel_ratio(Float)
    pestimator_status.pos_horiz_ratio(Float)
    pestimator_status.pos_vert_ratio(Float)
    pestimator_status.mag_ratio(Float)
    pestimator_status.hagl_ratio(Float)
    pestimator_status.tas_ratio(Float)
    pestimator_status.pos_horiz_accuracy(Float)
    pestimator_status.pos_vert_accuracy(Float)

}

fun ESTIMATOR_STATUS.copyInto(DST: ESTIMATOR_STATUS) {
    ESTIMATOR_STATUS.pull(
            DST, { -> time_usec() }, { flags() != null }, { flags()!!.get() }, { -> vel_ratio() }, { -> pos_horiz_ratio() }, { -> pos_vert_ratio() }, { -> mag_ratio() }, { -> hagl_ratio() }, { -> tas_ratio() }, { -> pos_horiz_accuracy() }, { -> pos_vert_accuracy() }

    )
}

fun onHWSTATUS(phwstatus: HWSTATUS) {
    val some_Vcc = phwstatus.Vcc()
    val some_I2Cerr = phwstatus.I2Cerr()

}

fun HWSTATUS.copyFrom(SRC: HWSTATUS) {
    HWSTATUS.push(
            SRC, { src -> Vcc(src) }, { src -> I2Cerr(src) }

    )
}

fun fill(phwstatus: HWSTATUS) {
    phwstatus.Vcc(Short)
    phwstatus.I2Cerr(Byte)

}

fun HWSTATUS.copyInto(DST: HWSTATUS) {
    HWSTATUS.pull(
            DST, { -> Vcc() }, { -> I2Cerr() }

    )
}

fun onTIMESYNC(ptimesync: TIMESYNC) {
    val some_tc1 = ptimesync.tc1()
    val some_ts1 = ptimesync.ts1()

}

fun TIMESYNC.copyFrom(SRC: TIMESYNC) {
    TIMESYNC.push(
            SRC, { src -> tc1(src) }, { src -> ts1(src) }

    )
}

fun fill(ptimesync: TIMESYNC) {
    ptimesync.tc1(Long)
    ptimesync.ts1(Long)

}

fun TIMESYNC.copyInto(DST: TIMESYNC) {
    TIMESYNC.pull(
            DST, { -> tc1() }, { -> ts1() }

    )
}

fun onPARAM_EXT_REQUEST_LIST(pparam_ext_request_list: PARAM_EXT_REQUEST_LIST) {
    val some_target_system = pparam_ext_request_list.target_system()
    val some_target_component = pparam_ext_request_list.target_component()

}

fun PARAM_EXT_REQUEST_LIST.copyFrom(SRC: PARAM_EXT_REQUEST_LIST) {
    PARAM_EXT_REQUEST_LIST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }

    )
}

fun fill(pparam_ext_request_list: PARAM_EXT_REQUEST_LIST) {
    pparam_ext_request_list.target_system(Byte)
    pparam_ext_request_list.target_component(Byte)

}

fun PARAM_EXT_REQUEST_LIST.copyInto(DST: PARAM_EXT_REQUEST_LIST) {
    PARAM_EXT_REQUEST_LIST.pull(
            DST, { -> target_system() }, { -> target_component() }

    )
}

fun onGLOBAL_POSITION_INT_COV(pglobal_position_int_cov: GLOBAL_POSITION_INT_COV) {
    val some_time_usec = pglobal_position_int_cov.time_usec()

    pglobal_position_int_cov.estimator_type()?.let { item ->
        some_MAV_ESTIMATOR_TYPE = item.get()
    }
    val some_lat = pglobal_position_int_cov.lat()
    val some_lon = pglobal_position_int_cov.lon()
    val some_alt = pglobal_position_int_cov.alt()
    val some_relative_alt = pglobal_position_int_cov.relative_alt()
    val some_vx = pglobal_position_int_cov.vx()
    val some_vy = pglobal_position_int_cov.vy()
    val some_vz = pglobal_position_int_cov.vz()
    pglobal_position_int_cov.covariance().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }

}

fun test_.GLOBAL_POSITION_INT_COV.copyFrom(SRC: GLOBAL_POSITION_INT_COV) {
    GLOBAL_POSITION_INT_COV.push(
            SRC, { src -> time_usec(src) }, { src -> estimator_type(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> relative_alt(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src ->
        val item = covariance()
        for (i in 0 until 36)
            item.set(src.get(i), i)
    }

    )
}

fun onBUTTON_CHANGE(pbutton_change: BUTTON_CHANGE) {
    val some_time_boot_ms = pbutton_change.time_boot_ms()
    val some_last_change_ms = pbutton_change.last_change_ms()
    val some_state = pbutton_change.state()

}

fun BUTTON_CHANGE.copyFrom(SRC: BUTTON_CHANGE) {
    BUTTON_CHANGE.push(
            SRC, { src -> time_boot_ms(src) }, { src -> last_change_ms(src) }, { src -> state(src) }

    )
}

fun fill(pbutton_change: BUTTON_CHANGE) {
    pbutton_change.time_boot_ms(Int)
    pbutton_change.last_change_ms(Int)
    pbutton_change.state(Byte)

}

fun BUTTON_CHANGE.copyInto(DST: BUTTON_CHANGE) {
    BUTTON_CHANGE.pull(
            DST, { -> time_boot_ms() }, { -> last_change_ms() }, { -> state() }

    )
}

fun onSAFETY_SET_ALLOWED_AREA(psafety_set_allowed_area: SAFETY_SET_ALLOWED_AREA) {
    val some_target_system = psafety_set_allowed_area.target_system()
    val some_target_component = psafety_set_allowed_area.target_component()

    psafety_set_allowed_area.frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }
    val some_p1x = psafety_set_allowed_area.p1x()
    val some_p1y = psafety_set_allowed_area.p1y()
    val some_p1z = psafety_set_allowed_area.p1z()
    val some_p2x = psafety_set_allowed_area.p2x()
    val some_p2y = psafety_set_allowed_area.p2y()
    val some_p2z = psafety_set_allowed_area.p2z()

}

fun test_.SAFETY_SET_ALLOWED_AREA.copyFrom(SRC: SAFETY_SET_ALLOWED_AREA) {
    SAFETY_SET_ALLOWED_AREA.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> frame(src) }, { src -> p1x(src) }, { src -> p1y(src) }, { src -> p1z(src) }, { src -> p2x(src) }, { src -> p2y(src) }, { src -> p2z(src) }

    )
}

fun onUAVCAN_NODE_STATUS(puavcan_node_status: UAVCAN_NODE_STATUS) {
    val some_time_usec = puavcan_node_status.time_usec()
    val some_uptime_sec = puavcan_node_status.uptime_sec()

    puavcan_node_status.health()?.let { item ->
        some_UAVCAN_NODE_HEALTH = item.get()
    }

    puavcan_node_status.mode()?.let { item ->
        some_UAVCAN_NODE_MODE = item.get()
    }
    val some_sub_mode = puavcan_node_status.sub_mode()
    val some_vendor_specific_status_code = puavcan_node_status.vendor_specific_status_code()

}

fun UAVCAN_NODE_STATUS.copyFrom(SRC: UAVCAN_NODE_STATUS) {
    UAVCAN_NODE_STATUS.push(
            SRC, { src -> time_usec(src) }, { src -> uptime_sec(src) }, { src -> health(src) }, { src -> mode(src) }, { src -> sub_mode(src) }, { src -> vendor_specific_status_code(src) }

    )
}

fun fill(puavcan_node_status: UAVCAN_NODE_STATUS) {
    puavcan_node_status.time_usec(Long)
    puavcan_node_status.uptime_sec(Int)

    puavcan_node_status.health(some_UAVCAN_NODE_HEALTH)

    puavcan_node_status.mode(some_UAVCAN_NODE_MODE)
    puavcan_node_status.sub_mode(Byte)
    puavcan_node_status.vendor_specific_status_code(Short)

}

fun UAVCAN_NODE_STATUS.copyInto(DST: UAVCAN_NODE_STATUS) {
    UAVCAN_NODE_STATUS.pull(
            DST, { -> time_usec() }, { -> uptime_sec() }, { health() != null }, { health()!!.get() }, { mode() != null }, { mode()!!.get() }, { -> sub_mode() }, { -> vendor_specific_status_code() }

    )
}

fun onCOLLISION(pcollision: COLLISION) {

    pcollision.sRc()?.let { item ->
        some_MAV_COLLISION_SRC = item.get()
    }
    val some_id = pcollision.id()

    pcollision.action()?.let { item ->
        some_MAV_COLLISION_ACTION = item.get()
    }

    pcollision.threat_level()?.let { item ->
        some_MAV_COLLISION_THREAT_LEVEL = item.get()
    }
    val some_time_to_minimum_delta = pcollision.time_to_minimum_delta()
    val some_altitude_minimum_delta = pcollision.altitude_minimum_delta()
    val some_horizontal_minimum_delta = pcollision.horizontal_minimum_delta()

}

fun COLLISION.copyFrom(SRC: COLLISION) {
    COLLISION.push(
            SRC, { src -> sRc(src) }, { src -> id(src) }, { src -> action(src) }, { src -> threat_level(src) }, { src -> time_to_minimum_delta(src) }, { src -> altitude_minimum_delta(src) }, { src -> horizontal_minimum_delta(src) }

    )
}

fun fill(pcollision: COLLISION) {

    pcollision.sRc(some_MAV_COLLISION_SRC)
    pcollision.id(Int)

    pcollision.action(some_MAV_COLLISION_ACTION)

    pcollision.threat_level(some_MAV_COLLISION_THREAT_LEVEL)
    pcollision.time_to_minimum_delta(Float)
    pcollision.altitude_minimum_delta(Float)
    pcollision.horizontal_minimum_delta(Float)

}

fun COLLISION.copyInto(DST: COLLISION) {
    COLLISION.pull(
            DST, { sRc() != null }, { sRc()!!.get() }, { -> id() }, { action() != null }, { action()!!.get() }, { threat_level() != null }, { threat_level()!!.get() }, { -> time_to_minimum_delta() }, { -> altitude_minimum_delta() }, { -> horizontal_minimum_delta() }

    )
}

fun onGIMBAL_TORQUE_CMD_REPORT(pgimbal_torque_cmd_report: GIMBAL_TORQUE_CMD_REPORT) {
    val some_target_system = pgimbal_torque_cmd_report.target_system()
    val some_target_component = pgimbal_torque_cmd_report.target_component()
    val some_rl_torque_cmd = pgimbal_torque_cmd_report.rl_torque_cmd()
    val some_el_torque_cmd = pgimbal_torque_cmd_report.el_torque_cmd()
    val some_az_torque_cmd = pgimbal_torque_cmd_report.az_torque_cmd()

}

fun GIMBAL_TORQUE_CMD_REPORT.copyFrom(SRC: GIMBAL_TORQUE_CMD_REPORT) {
    GIMBAL_TORQUE_CMD_REPORT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> rl_torque_cmd(src) }, { src -> el_torque_cmd(src) }, { src -> az_torque_cmd(src) }

    )
}

fun fill(pgimbal_torque_cmd_report: GIMBAL_TORQUE_CMD_REPORT) {
    pgimbal_torque_cmd_report.target_system(Byte)
    pgimbal_torque_cmd_report.target_component(Byte)
    pgimbal_torque_cmd_report.rl_torque_cmd(Short)
    pgimbal_torque_cmd_report.el_torque_cmd(Short)
    pgimbal_torque_cmd_report.az_torque_cmd(Short)

}

fun GIMBAL_TORQUE_CMD_REPORT.copyInto(DST: GIMBAL_TORQUE_CMD_REPORT) {
    GIMBAL_TORQUE_CMD_REPORT.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> rl_torque_cmd() }, { -> el_torque_cmd() }, { -> az_torque_cmd() }

    )
}

fun onALTITUDE(paltitude: ALTITUDE) {
    val some_time_usec = paltitude.time_usec()
    val some_altitude_monotonic = paltitude.altitude_monotonic()
    val some_altitude_amsl = paltitude.altitude_amsl()
    val some_altitude_local = paltitude.altitude_local()
    val some_altitude_relative = paltitude.altitude_relative()
    val some_altitude_terrain = paltitude.altitude_terrain()
    val some_bottom_clearance = paltitude.bottom_clearance()

}

fun ALTITUDE.copyFrom(SRC: ALTITUDE) {
    ALTITUDE.push(
            SRC, { src -> time_usec(src) }, { src -> altitude_monotonic(src) }, { src -> altitude_amsl(src) }, { src -> altitude_local(src) }, { src -> altitude_relative(src) }, { src -> altitude_terrain(src) }, { src -> bottom_clearance(src) }

    )
}

fun fill(paltitude: ALTITUDE) {
    paltitude.time_usec(Long)
    paltitude.altitude_monotonic(Float)
    paltitude.altitude_amsl(Float)
    paltitude.altitude_local(Float)
    paltitude.altitude_relative(Float)
    paltitude.altitude_terrain(Float)
    paltitude.bottom_clearance(Float)

}

fun ALTITUDE.copyInto(DST: ALTITUDE) {
    ALTITUDE.pull(
            DST, { -> time_usec() }, { -> altitude_monotonic() }, { -> altitude_amsl() }, { -> altitude_local() }, { -> altitude_relative() }, { -> altitude_terrain() }, { -> bottom_clearance() }

    )
}

fun onHIL_STATE_QUATERNION(phil_state_quaternion: HIL_STATE_QUATERNION) {
    val some_time_usec = phil_state_quaternion.time_usec()
    phil_state_quaternion.attitude_quaternion().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_rollspeed = phil_state_quaternion.rollspeed()
    val some_pitchspeed = phil_state_quaternion.pitchspeed()
    val some_yawspeed = phil_state_quaternion.yawspeed()
    val some_lat = phil_state_quaternion.lat()
    val some_lon = phil_state_quaternion.lon()
    val some_alt = phil_state_quaternion.alt()
    val some_vx = phil_state_quaternion.vx()
    val some_vy = phil_state_quaternion.vy()
    val some_vz = phil_state_quaternion.vz()
    val some_ind_airspeed = phil_state_quaternion.ind_airspeed()
    val some_true_airspeed = phil_state_quaternion.true_airspeed()
    val some_xacc = phil_state_quaternion.xacc()
    val some_yacc = phil_state_quaternion.yacc()
    val some_zacc = phil_state_quaternion.zacc()

}

fun HIL_STATE_QUATERNION.copyFrom(SRC: HIL_STATE_QUATERNION) {
    HIL_STATE_QUATERNION.push(
            SRC, { src -> time_usec(src) }, { src ->
        val item = attitude_quaternion()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> rollspeed(src) }, { src -> pitchspeed(src) }, { src -> yawspeed(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> ind_airspeed(src) }, { src -> true_airspeed(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }

    )
}

fun fill(phil_state_quaternion: HIL_STATE_QUATERNION) {
    phil_state_quaternion.time_usec(Long)
    phil_state_quaternion.attitude_quaternion().let { item ->
        phil_state_quaternion.attitude_quaternion().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    phil_state_quaternion.rollspeed(Float)
    phil_state_quaternion.pitchspeed(Float)
    phil_state_quaternion.yawspeed(Float)
    phil_state_quaternion.lat(Int)
    phil_state_quaternion.lon(Int)
    phil_state_quaternion.alt(Int)
    phil_state_quaternion.vx(Short)
    phil_state_quaternion.vy(Short)
    phil_state_quaternion.vz(Short)
    phil_state_quaternion.ind_airspeed(Short)
    phil_state_quaternion.true_airspeed(Short)
    phil_state_quaternion.xacc(Short)
    phil_state_quaternion.yacc(Short)
    phil_state_quaternion.zacc(Short)

}

fun HIL_STATE_QUATERNION.copyInto(DST: HIL_STATE_QUATERNION) {
    HIL_STATE_QUATERNION.pull(
            DST, { -> time_usec() }, { dst ->
        var src = this.attitude_quaternion()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }, { -> rollspeed() }, { -> pitchspeed() }, { -> yawspeed() }, { -> lat() }, { -> lon() }, { -> alt() }, { -> vx() }, { -> vy() }, { -> vz() }, { -> ind_airspeed() }, { -> true_airspeed() }, { -> xacc() }, { -> yacc() }, { -> zacc() }

    )
}

fun onSENSOR_OFFSETS(psensor_offsets: SENSOR_OFFSETS) {
    val some_mag_ofs_x = psensor_offsets.mag_ofs_x()
    val some_mag_ofs_y = psensor_offsets.mag_ofs_y()
    val some_mag_ofs_z = psensor_offsets.mag_ofs_z()
    val some_mag_declination = psensor_offsets.mag_declination()
    val some_raw_press = psensor_offsets.raw_press()
    val some_raw_temp = psensor_offsets.raw_temp()
    val some_gyro_cal_x = psensor_offsets.gyro_cal_x()
    val some_gyro_cal_y = psensor_offsets.gyro_cal_y()
    val some_gyro_cal_z = psensor_offsets.gyro_cal_z()
    val some_accel_cal_x = psensor_offsets.accel_cal_x()
    val some_accel_cal_y = psensor_offsets.accel_cal_y()
    val some_accel_cal_z = psensor_offsets.accel_cal_z()

}

fun SENSOR_OFFSETS.copyFrom(SRC: SENSOR_OFFSETS) {
    SENSOR_OFFSETS.push(
            SRC, { src -> mag_ofs_x(src) }, { src -> mag_ofs_y(src) }, { src -> mag_ofs_z(src) }, { src -> mag_declination(src) }, { src -> raw_press(src) }, { src -> raw_temp(src) }, { src -> gyro_cal_x(src) }, { src -> gyro_cal_y(src) }, { src -> gyro_cal_z(src) }, { src -> accel_cal_x(src) }, { src -> accel_cal_y(src) }, { src -> accel_cal_z(src) }

    )
}

fun fill(psensor_offsets: SENSOR_OFFSETS) {
    psensor_offsets.mag_ofs_x(Short)
    psensor_offsets.mag_ofs_y(Short)
    psensor_offsets.mag_ofs_z(Short)
    psensor_offsets.mag_declination(Float)
    psensor_offsets.raw_press(Int)
    psensor_offsets.raw_temp(Int)
    psensor_offsets.gyro_cal_x(Float)
    psensor_offsets.gyro_cal_y(Float)
    psensor_offsets.gyro_cal_z(Float)
    psensor_offsets.accel_cal_x(Float)
    psensor_offsets.accel_cal_y(Float)
    psensor_offsets.accel_cal_z(Float)

}

fun SENSOR_OFFSETS.copyInto(DST: SENSOR_OFFSETS) {
    SENSOR_OFFSETS.pull(
            DST, { -> mag_ofs_x() }, { -> mag_ofs_y() }, { -> mag_ofs_z() }, { -> mag_declination() }, { -> raw_press() }, { -> raw_temp() }, { -> gyro_cal_x() }, { -> gyro_cal_y() }, { -> gyro_cal_z() }, { -> accel_cal_x() }, { -> accel_cal_y() }, { -> accel_cal_z() }

    )
}

fun onSTORAGE_INFORMATION(pstorage_information: STORAGE_INFORMATION) {
    val some_time_boot_ms = pstorage_information.time_boot_ms()
    val some_storage_id = pstorage_information.storage_id()
    val some_storage_count = pstorage_information.storage_count()
    val some_status = pstorage_information.status()
    val some_total_capacity = pstorage_information.total_capacity()
    val some_used_capacity = pstorage_information.used_capacity()
    val some_available_capacity = pstorage_information.available_capacity()
    val some_read_speed = pstorage_information.read_speed()
    val some_write_speed = pstorage_information.write_speed()

}

fun STORAGE_INFORMATION.copyFrom(SRC: STORAGE_INFORMATION) {
    STORAGE_INFORMATION.push(
            SRC, { src -> time_boot_ms(src) }, { src -> storage_id(src) }, { src -> storage_count(src) }, { src -> status(src) }, { src -> total_capacity(src) }, { src -> used_capacity(src) }, { src -> available_capacity(src) }, { src -> read_speed(src) }, { src -> write_speed(src) }

    )
}

fun fill(pstorage_information: STORAGE_INFORMATION) {
    pstorage_information.time_boot_ms(Int)
    pstorage_information.storage_id(Byte)
    pstorage_information.storage_count(Byte)
    pstorage_information.status(Byte)
    pstorage_information.total_capacity(Float)
    pstorage_information.used_capacity(Float)
    pstorage_information.available_capacity(Float)
    pstorage_information.read_speed(Float)
    pstorage_information.write_speed(Float)

}

fun STORAGE_INFORMATION.copyInto(DST: STORAGE_INFORMATION) {
    STORAGE_INFORMATION.pull(
            DST, { -> time_boot_ms() }, { -> storage_id() }, { -> storage_count() }, { -> status() }, { -> total_capacity() }, { -> used_capacity() }, { -> available_capacity() }, { -> read_speed() }, { -> write_speed() }

    )
}

fun onCAMERA_INFORMATION(pcamera_information: CAMERA_INFORMATION) {
    val some_time_boot_ms = pcamera_information.time_boot_ms()
    pcamera_information.vendor_name().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    pcamera_information.model_name().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    val some_firmware_version = pcamera_information.firmware_version()
    val some_focal_length = pcamera_information.focal_length()
    val some_sensor_size_h = pcamera_information.sensor_size_h()
    val some_sensor_size_v = pcamera_information.sensor_size_v()
    val some_resolution_h = pcamera_information.resolution_h()
    val some_resolution_v = pcamera_information.resolution_v()
    val some_lens_id = pcamera_information.lens_id()

    pcamera_information.flags()?.let { item ->
        some_CAMERA_CAP_FLAGS = item.get()
    }
    val some_cam_definition_version = pcamera_information.cam_definition_version()

    pcamera_information.cam_definition_uri()?.let { item -> char = item.get() }

}

fun CAMERA_INFORMATION.copyFrom(SRC: CAMERA_INFORMATION) {
    CAMERA_INFORMATION.push(
            SRC, { src -> time_boot_ms(src) }, { src ->
        val item = vendor_name()
        for (i in 0 until 32)
            item.set(src.get(i), i)
    }, { src ->
        val item = model_name()
        for (i in 0 until 32)
            item.set(src.get(i), i)
    }, { src -> firmware_version(src) }, { src -> focal_length(src) }, { src -> sensor_size_h(src) }, { src -> sensor_size_v(src) }, { src -> resolution_h(src) }, { src -> resolution_v(src) }, { src -> lens_id(src) }, { src -> flags(src) }, { src -> cam_definition_version(src) }, { src ->
        val item = cam_definition_uri(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun fill(pcamera_information: CAMERA_INFORMATION) {
    pcamera_information.time_boot_ms(Int)
    pcamera_information.vendor_name().let { item ->
        pcamera_information.vendor_name().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    pcamera_information.model_name().let { item ->
        pcamera_information.model_name().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    pcamera_information.firmware_version(Int)
    pcamera_information.focal_length(Float)
    pcamera_information.sensor_size_h(Float)
    pcamera_information.sensor_size_v(Float)
    pcamera_information.resolution_h(Short)
    pcamera_information.resolution_v(Short)
    pcamera_information.lens_id(Byte)

    pcamera_information.flags(some_CAMERA_CAP_FLAGS)
    pcamera_information.cam_definition_version(Short)

    pcamera_information.cam_definition_uri(some_String, null)

}

fun CAMERA_INFORMATION.copyInto(DST: CAMERA_INFORMATION) {
    CAMERA_INFORMATION.pull(
            DST, { -> time_boot_ms() }, { dst ->
        var src = this.vendor_name()
        for (i in 0 until 32)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.model_name()
        for (i in 0 until 32)
            dst.set(src.get(i), i)
    }, { -> firmware_version() }, { -> focal_length() }, { -> sensor_size_h() }, { -> sensor_size_v() }, { -> resolution_h() }, { -> resolution_v() }, { -> lens_id() }, { flags() != null }, { flags()!!.get() }, { -> cam_definition_version() }, { cam_definition_uri()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = cam_definition_uri()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }

    )
}

fun onGPS_STATUS(pgps_status: GPS_STATUS) {
    val some_satellites_visible = pgps_status.satellites_visible()
    pgps_status.satellite_prn().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    pgps_status.satellite_used().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    pgps_status.satellite_elevation().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    pgps_status.satellite_azimuth().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    pgps_status.satellite_snr().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun test_.GPS_STATUS.copyFrom(SRC: GPS_STATUS) {
    GPS_STATUS.push(
            SRC, { src -> satellites_visible(src) }, { src ->
        val item = satellite_prn()
        for (i in 0 until 20)
            item.set(src.get(i), i)
    }, { src ->
        val item = satellite_used()
        for (i in 0 until 20)
            item.set(src.get(i), i)
    }, { src ->
        val item = satellite_elevation()
        for (i in 0 until 20)
            item.set(src.get(i), i)
    }, { src ->
        val item = satellite_azimuth()
        for (i in 0 until 20)
            item.set(src.get(i), i)
    }, { src ->
        val item = satellite_snr()
        for (i in 0 until 20)
            item.set(src.get(i), i)
    }

    )
}

fun onDEVICE_OP_WRITE_REPLY(pdevice_op_write_reply: DEVICE_OP_WRITE_REPLY) {
    val some_request_id = pdevice_op_write_reply.request_id()
    val some_result = pdevice_op_write_reply.result()

}

fun DEVICE_OP_WRITE_REPLY.copyFrom(SRC: DEVICE_OP_WRITE_REPLY) {
    DEVICE_OP_WRITE_REPLY.push(
            SRC, { src -> request_id(src) }, { src -> result(src) }

    )
}

fun fill(pdevice_op_write_reply: DEVICE_OP_WRITE_REPLY) {
    pdevice_op_write_reply.request_id(Int)
    pdevice_op_write_reply.result(Byte)

}

fun DEVICE_OP_WRITE_REPLY.copyInto(DST: DEVICE_OP_WRITE_REPLY) {
    DEVICE_OP_WRITE_REPLY.pull(
            DST, { -> request_id() }, { -> result() }

    )
}

fun onPARAM_SET(pparam_set: PARAM_SET) {
    val some_target_system = pparam_set.target_system()
    val some_target_component = pparam_set.target_component()

    pparam_set.param_id()?.let { item -> char = item.get() }
    val some_param_value = pparam_set.param_value()

    pparam_set.param_type()?.let { item ->
        some_MAV_PARAM_TYPE = item.get()
    }

}

fun test_.PARAM_SET.copyFrom(SRC: PARAM_SET) {
    PARAM_SET.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = param_id(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> param_value(src) }, { src -> param_type(src) }

    )
}

fun onTERRAIN_DATA(pterrain_data: TERRAIN_DATA) {
    val some_lat = pterrain_data.lat()
    val some_lon = pterrain_data.lon()
    val some_grid_spacing = pterrain_data.grid_spacing()
    val some_gridbit = pterrain_data.gridbit()
    pterrain_data.daTa().let { item ->
        for (index in 0 until item.len())
            Short = item.get(index)
    }

}

fun TERRAIN_DATA.copyFrom(SRC: TERRAIN_DATA) {
    TERRAIN_DATA.push(
            SRC, { src -> lat(src) }, { src -> lon(src) }, { src -> grid_spacing(src) }, { src -> gridbit(src) }, { src ->
        val item = daTa()
        for (i in 0 until 16)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pterrain_data: TERRAIN_DATA) {
    pterrain_data.lat(Int)
    pterrain_data.lon(Int)
    pterrain_data.grid_spacing(Short)
    pterrain_data.gridbit(Byte)
    pterrain_data.daTa().let { item ->
        pterrain_data.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Short, index)
        }
    }

}

fun TERRAIN_DATA.copyInto(DST: TERRAIN_DATA) {
    TERRAIN_DATA.pull(
            DST, { -> lat() }, { -> lon() }, { -> grid_spacing() }, { -> gridbit() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 16)
            dst.set(src.get(i), i)
    }

    )
}

fun onGIMBAL_CONTROL(pgimbal_control: GIMBAL_CONTROL) {
    val some_target_system = pgimbal_control.target_system()
    val some_target_component = pgimbal_control.target_component()
    val some_demanded_rate_x = pgimbal_control.demanded_rate_x()
    val some_demanded_rate_y = pgimbal_control.demanded_rate_y()
    val some_demanded_rate_z = pgimbal_control.demanded_rate_z()

}

fun GIMBAL_CONTROL.copyFrom(SRC: GIMBAL_CONTROL) {
    GIMBAL_CONTROL.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> demanded_rate_x(src) }, { src -> demanded_rate_y(src) }, { src -> demanded_rate_z(src) }

    )
}

fun fill(pgimbal_control: GIMBAL_CONTROL) {
    pgimbal_control.target_system(Byte)
    pgimbal_control.target_component(Byte)
    pgimbal_control.demanded_rate_x(Float)
    pgimbal_control.demanded_rate_y(Float)
    pgimbal_control.demanded_rate_z(Float)

}

fun GIMBAL_CONTROL.copyInto(DST: GIMBAL_CONTROL) {
    GIMBAL_CONTROL.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> demanded_rate_x() }, { -> demanded_rate_y() }, { -> demanded_rate_z() }

    )
}

fun onRC_CHANNELS_OVERRIDE(prc_channels_override: RC_CHANNELS_OVERRIDE) {
    val some_target_system = prc_channels_override.target_system()
    val some_target_component = prc_channels_override.target_component()
    val some_chan1_raw = prc_channels_override.chan1_raw()
    val some_chan2_raw = prc_channels_override.chan2_raw()
    val some_chan3_raw = prc_channels_override.chan3_raw()
    val some_chan4_raw = prc_channels_override.chan4_raw()
    val some_chan5_raw = prc_channels_override.chan5_raw()
    val some_chan6_raw = prc_channels_override.chan6_raw()
    val some_chan7_raw = prc_channels_override.chan7_raw()
    val some_chan8_raw = prc_channels_override.chan8_raw()

}

fun test_.RC_CHANNELS_OVERRIDE.copyFrom(SRC: RC_CHANNELS_OVERRIDE) {
    RC_CHANNELS_OVERRIDE.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> chan1_raw(src) }, { src -> chan2_raw(src) }, { src -> chan3_raw(src) }, { src -> chan4_raw(src) }, { src -> chan5_raw(src) }, { src -> chan6_raw(src) }, { src -> chan7_raw(src) }, { src -> chan8_raw(src) }

    )
}

fun onSCALED_IMU(pscaled_imu: SCALED_IMU) {
    val some_time_boot_ms = pscaled_imu.time_boot_ms()
    val some_xacc = pscaled_imu.xacc()
    val some_yacc = pscaled_imu.yacc()
    val some_zacc = pscaled_imu.zacc()
    val some_xgyro = pscaled_imu.xgyro()
    val some_ygyro = pscaled_imu.ygyro()
    val some_zgyro = pscaled_imu.zgyro()
    val some_xmag = pscaled_imu.xmag()
    val some_ymag = pscaled_imu.ymag()
    val some_zmag = pscaled_imu.zmag()

}

fun test_.SCALED_IMU.copyFrom(SRC: SCALED_IMU) {
    SCALED_IMU.push(
            SRC, { src -> time_boot_ms(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }, { src -> xgyro(src) }, { src -> ygyro(src) }, { src -> zgyro(src) }, { src -> xmag(src) }, { src -> ymag(src) }, { src -> zmag(src) }

    )
}

fun onVIDEO_STREAM_INFORMATION(pvideo_stream_information: VIDEO_STREAM_INFORMATION) {
    val some_camera_id = pvideo_stream_information.camera_id()
    val some_status = pvideo_stream_information.status()
    val some_framerate = pvideo_stream_information.framerate()
    val some_resolution_h = pvideo_stream_information.resolution_h()
    val some_resolution_v = pvideo_stream_information.resolution_v()
    val some_bitrate = pvideo_stream_information.bitrate()
    val some_rotation = pvideo_stream_information.rotation()

    pvideo_stream_information.uri()?.let { item -> char = item.get() }

}

fun VIDEO_STREAM_INFORMATION.copyFrom(SRC: VIDEO_STREAM_INFORMATION) {
    VIDEO_STREAM_INFORMATION.push(
            SRC, { src -> camera_id(src) }, { src -> status(src) }, { src -> framerate(src) }, { src -> resolution_h(src) }, { src -> resolution_v(src) }, { src -> bitrate(src) }, { src -> rotation(src) }, { src ->
        val item = uri(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun fill(pvideo_stream_information: VIDEO_STREAM_INFORMATION) {
    pvideo_stream_information.camera_id(Byte)
    pvideo_stream_information.status(Byte)
    pvideo_stream_information.framerate(Float)
    pvideo_stream_information.resolution_h(Short)
    pvideo_stream_information.resolution_v(Short)
    pvideo_stream_information.bitrate(Int)
    pvideo_stream_information.rotation(Short)

    pvideo_stream_information.uri(some_String, null)

}

fun VIDEO_STREAM_INFORMATION.copyInto(DST: VIDEO_STREAM_INFORMATION) {
    VIDEO_STREAM_INFORMATION.pull(
            DST, { -> camera_id() }, { -> status() }, { -> framerate() }, { -> resolution_h() }, { -> resolution_v() }, { -> bitrate() }, { -> rotation() }, { uri()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = uri()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }

    )
}

fun onAHRS(pahrs: AHRS) {
    val some_omegaIx = pahrs.omegaIx()
    val some_omegaIy = pahrs.omegaIy()
    val some_omegaIz = pahrs.omegaIz()
    val some_accel_weight = pahrs.accel_weight()
    val some_renorm_val = pahrs.renorm_val()
    val some_error_rp = pahrs.error_rp()
    val some_error_yaw = pahrs.error_yaw()

}

fun AHRS.copyFrom(SRC: AHRS) {
    AHRS.push(
            SRC, { src -> omegaIx(src) }, { src -> omegaIy(src) }, { src -> omegaIz(src) }, { src -> accel_weight(src) }, { src -> renorm_val(src) }, { src -> error_rp(src) }, { src -> error_yaw(src) }

    )
}

fun fill(pahrs: AHRS) {
    pahrs.omegaIx(Float)
    pahrs.omegaIy(Float)
    pahrs.omegaIz(Float)
    pahrs.accel_weight(Float)
    pahrs.renorm_val(Float)
    pahrs.error_rp(Float)
    pahrs.error_yaw(Float)

}

fun AHRS.copyInto(DST: AHRS) {
    AHRS.pull(
            DST, { -> omegaIx() }, { -> omegaIy() }, { -> omegaIz() }, { -> accel_weight() }, { -> renorm_val() }, { -> error_rp() }, { -> error_yaw() }

    )
}

fun onDEBUG(pdebug: DEBUG) {
    val some_time_boot_ms = pdebug.time_boot_ms()
    val some_ind = pdebug.ind()
    val some_value = pdebug.value()

}

fun DEBUG.copyFrom(SRC: DEBUG) {
    DEBUG.push(
            SRC, { src -> time_boot_ms(src) }, { src -> ind(src) }, { src -> value(src) }

    )
}

fun fill(pdebug: DEBUG) {
    pdebug.time_boot_ms(Int)
    pdebug.ind(Byte)
    pdebug.value(Float)

}

fun DEBUG.copyInto(DST: DEBUG) {
    DEBUG.pull(
            DST, { -> time_boot_ms() }, { -> ind() }, { -> value() }

    )
}

fun onCAMERA_IMAGE_CAPTURED(pcamera_image_captured: CAMERA_IMAGE_CAPTURED) {
    val some_time_boot_ms = pcamera_image_captured.time_boot_ms()
    val some_time_utc = pcamera_image_captured.time_utc()
    val some_camera_id = pcamera_image_captured.camera_id()
    val some_lat = pcamera_image_captured.lat()
    val some_lon = pcamera_image_captured.lon()
    val some_alt = pcamera_image_captured.alt()
    val some_relative_alt = pcamera_image_captured.relative_alt()
    pcamera_image_captured.q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_image_index = pcamera_image_captured.image_index()
    val some_capture_result = pcamera_image_captured.capture_result()

    pcamera_image_captured.file_url()?.let { item -> char = item.get() }

}

fun CAMERA_IMAGE_CAPTURED.copyFrom(SRC: CAMERA_IMAGE_CAPTURED) {
    CAMERA_IMAGE_CAPTURED.push(
            SRC, { src -> time_boot_ms(src) }, { src -> time_utc(src) }, { src -> camera_id(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> relative_alt(src) }, { src ->
        val item = q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> image_index(src) }, { src -> capture_result(src) }, { src ->
        val item = file_url(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun fill(pcamera_image_captured: CAMERA_IMAGE_CAPTURED) {
    pcamera_image_captured.time_boot_ms(Int)
    pcamera_image_captured.time_utc(Long)
    pcamera_image_captured.camera_id(Byte)
    pcamera_image_captured.lat(Int)
    pcamera_image_captured.lon(Int)
    pcamera_image_captured.alt(Int)
    pcamera_image_captured.relative_alt(Int)
    pcamera_image_captured.q().let { item ->
        pcamera_image_captured.q().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pcamera_image_captured.image_index(Int)
    pcamera_image_captured.capture_result(Byte)

    pcamera_image_captured.file_url(some_String, null)

}

fun CAMERA_IMAGE_CAPTURED.copyInto(DST: CAMERA_IMAGE_CAPTURED) {
    CAMERA_IMAGE_CAPTURED.pull(
            DST, { -> time_boot_ms() }, { -> time_utc() }, { -> camera_id() }, { -> lat() }, { -> lon() }, { -> alt() }, { -> relative_alt() }, { dst ->
        var src = this.q()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }, { -> image_index() }, { -> capture_result() }, { file_url()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = file_url()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }

    )
}

fun onLOG_ENTRY(plog_entry: LOG_ENTRY) {
    val some_id = plog_entry.id()
    val some_num_logs = plog_entry.num_logs()
    val some_last_log_num = plog_entry.last_log_num()
    val some_time_utc = plog_entry.time_utc()
    val some_size = plog_entry.size()

}

fun LOG_ENTRY.copyFrom(SRC: LOG_ENTRY) {
    LOG_ENTRY.push(
            SRC, { src -> id(src) }, { src -> num_logs(src) }, { src -> last_log_num(src) }, { src -> time_utc(src) }, { src -> size(src) }

    )
}

fun fill(plog_entry: LOG_ENTRY) {
    plog_entry.id(Short)
    plog_entry.num_logs(Short)
    plog_entry.last_log_num(Short)
    plog_entry.time_utc(Int)
    plog_entry.size(Int)

}

fun LOG_ENTRY.copyInto(DST: LOG_ENTRY) {
    LOG_ENTRY.pull(
            DST, { -> id() }, { -> num_logs() }, { -> last_log_num() }, { -> time_utc() }, { -> size() }

    )
}

fun onACTUATOR_CONTROL_TARGET(pactuator_control_target: ACTUATOR_CONTROL_TARGET) {
    val some_time_usec = pactuator_control_target.time_usec()
    val some_group_mlx = pactuator_control_target.group_mlx()
    pactuator_control_target.controls().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }

}

fun ACTUATOR_CONTROL_TARGET.copyFrom(SRC: ACTUATOR_CONTROL_TARGET) {
    ACTUATOR_CONTROL_TARGET.push(
            SRC, { src -> time_usec(src) }, { src -> group_mlx(src) }, { src ->
        val item = controls()
        for (i in 0 until 8)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pactuator_control_target: ACTUATOR_CONTROL_TARGET) {
    pactuator_control_target.time_usec(Long)
    pactuator_control_target.group_mlx(Byte)
    pactuator_control_target.controls().let { item ->
        pactuator_control_target.controls().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }

}

fun ACTUATOR_CONTROL_TARGET.copyInto(DST: ACTUATOR_CONTROL_TARGET) {
    ACTUATOR_CONTROL_TARGET.pull(
            DST, { -> time_usec() }, { -> group_mlx() }, { dst ->
        var src = this.controls()
        for (i in 0 until 8)
            dst.set(src.get(i), i)
    }

    )
}

fun onHIGH_LATENCY(phigh_latency: HIGH_LATENCY) {

    phigh_latency.base_mode()?.let { item ->
        some_MAV_MODE_FLAG = item.get()
    }
    val some_custom_mode = phigh_latency.custom_mode()

    phigh_latency.landed_state()?.let { item ->
        some_MAV_LANDED_STATE = item.get()
    }
    val some_roll = phigh_latency.roll()
    val some_pitch = phigh_latency.pitch()
    val some_heading = phigh_latency.heading()
    val some_throttle = phigh_latency.throttle()
    val some_heading_sp = phigh_latency.heading_sp()
    val some_latitude = phigh_latency.latitude()
    val some_longitude = phigh_latency.longitude()
    val some_altitude_amsl = phigh_latency.altitude_amsl()
    val some_altitude_sp = phigh_latency.altitude_sp()
    val some_airspeed = phigh_latency.airspeed()
    val some_airspeed_sp = phigh_latency.airspeed_sp()
    val some_groundspeed = phigh_latency.groundspeed()
    val some_climb_rate = phigh_latency.climb_rate()
    val some_gps_nsat = phigh_latency.gps_nsat()

    phigh_latency.gps_fix_type()?.let { item ->
        some_GPS_FIX_TYPE = item.get()
    }
    val some_battery_remaining = phigh_latency.battery_remaining()
    val some_temperature = phigh_latency.temperature()
    val some_temperature_air = phigh_latency.temperature_air()
    val some_failsafe = phigh_latency.failsafe()
    val some_wp_num = phigh_latency.wp_num()
    val some_wp_distance = phigh_latency.wp_distance()

}

fun HIGH_LATENCY.copyFrom(SRC: HIGH_LATENCY) {
    HIGH_LATENCY.push(
            SRC, { src -> base_mode(src) }, { src -> custom_mode(src) }, { src -> landed_state(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> heading(src) }, { src -> throttle(src) }, { src -> heading_sp(src) }, { src -> latitude(src) }, { src -> longitude(src) }, { src -> altitude_amsl(src) }, { src -> altitude_sp(src) }, { src -> airspeed(src) }, { src -> airspeed_sp(src) }, { src -> groundspeed(src) }, { src -> climb_rate(src) }, { src -> gps_nsat(src) }, { src -> gps_fix_type(src) }, { src -> battery_remaining(src) }, { src -> temperature(src) }, { src -> temperature_air(src) }, { src -> failsafe(src) }, { src -> wp_num(src) }, { src -> wp_distance(src) }

    )
}

fun fill(phigh_latency: HIGH_LATENCY) {

    phigh_latency.base_mode(some_MAV_MODE_FLAG)
    phigh_latency.custom_mode(Int)

    phigh_latency.landed_state(some_MAV_LANDED_STATE)
    phigh_latency.roll(Short)
    phigh_latency.pitch(Short)
    phigh_latency.heading(Short)
    phigh_latency.throttle(Byte)
    phigh_latency.heading_sp(Short)
    phigh_latency.latitude(Int)
    phigh_latency.longitude(Int)
    phigh_latency.altitude_amsl(Short)
    phigh_latency.altitude_sp(Short)
    phigh_latency.airspeed(Byte)
    phigh_latency.airspeed_sp(Byte)
    phigh_latency.groundspeed(Byte)
    phigh_latency.climb_rate(Byte)
    phigh_latency.gps_nsat(Byte)

    phigh_latency.gps_fix_type(some_GPS_FIX_TYPE)
    phigh_latency.battery_remaining(Byte)
    phigh_latency.temperature(Byte)
    phigh_latency.temperature_air(Byte)
    phigh_latency.failsafe(Byte)
    phigh_latency.wp_num(Byte)
    phigh_latency.wp_distance(Short)

}

fun HIGH_LATENCY.copyInto(DST: HIGH_LATENCY) {
    HIGH_LATENCY.pull(
            DST, { base_mode() != null }, { base_mode()!!.get() }, { -> custom_mode() }, { landed_state() != null }, { landed_state()!!.get() }, { -> roll() }, { -> pitch() }, { -> heading() }, { -> throttle() }, { -> heading_sp() }, { -> latitude() }, { -> longitude() }, { -> altitude_amsl() }, { -> altitude_sp() }, { -> airspeed() }, { -> airspeed_sp() }, { -> groundspeed() }, { -> climb_rate() }, { -> gps_nsat() }, { gps_fix_type() != null }, { gps_fix_type()!!.get() }, { -> battery_remaining() }, { -> temperature() }, { -> temperature_air() }, { -> failsafe() }, { -> wp_num() }, { -> wp_distance() }

    )
}

fun onPARAM_REQUEST_READ(pparam_request_read: PARAM_REQUEST_READ) {
    val some_target_system = pparam_request_read.target_system()
    val some_target_component = pparam_request_read.target_component()

    pparam_request_read.param_id()?.let { item -> char = item.get() }
    val some_param_index = pparam_request_read.param_index()

}

fun test_.PARAM_REQUEST_READ.copyFrom(SRC: PARAM_REQUEST_READ) {
    PARAM_REQUEST_READ.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = param_id(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> param_index(src) }

    )
}

fun onSET_ATTITUDE_TARGET(pset_attitude_target: SET_ATTITUDE_TARGET) {
    val some_time_boot_ms = pset_attitude_target.time_boot_ms()
    val some_target_system = pset_attitude_target.target_system()
    val some_target_component = pset_attitude_target.target_component()
    val some_type_mask = pset_attitude_target.type_mask()
    pset_attitude_target.q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_body_roll_rate = pset_attitude_target.body_roll_rate()
    val some_body_pitch_rate = pset_attitude_target.body_pitch_rate()
    val some_body_yaw_rate = pset_attitude_target.body_yaw_rate()
    val some_thrust = pset_attitude_target.thrust()

}

fun test_.SET_ATTITUDE_TARGET.copyFrom(SRC: SET_ATTITUDE_TARGET) {
    SET_ATTITUDE_TARGET.push(
            SRC, { src -> time_boot_ms(src) }, { src -> target_system(src) }, { src -> target_component(src) }, { src -> type_mask(src) }, { src ->
        val item = q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> body_roll_rate(src) }, { src -> body_pitch_rate(src) }, { src -> body_yaw_rate(src) }, { src -> thrust(src) }

    )
}

fun onFOLLOW_TARGET(pfollow_target: FOLLOW_TARGET) {
    val some_timestamp = pfollow_target.timestamp()
    val some_est_capabilities = pfollow_target.est_capabilities()
    val some_lat = pfollow_target.lat()
    val some_lon = pfollow_target.lon()
    val some_alt = pfollow_target.alt()
    pfollow_target.vel().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    pfollow_target.acc().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    pfollow_target.attitude_q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    pfollow_target.rates().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    pfollow_target.position_cov().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_custom_state = pfollow_target.custom_state()

}

fun FOLLOW_TARGET.copyFrom(SRC: FOLLOW_TARGET) {
    FOLLOW_TARGET.push(
            SRC, { src -> timestamp(src) }, { src -> est_capabilities(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src ->
        val item = vel()
        for (i in 0 until 3)
            item.set(src.get(i), i)
    }, { src ->
        val item = acc()
        for (i in 0 until 3)
            item.set(src.get(i), i)
    }, { src ->
        val item = attitude_q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src ->
        val item = rates()
        for (i in 0 until 3)
            item.set(src.get(i), i)
    }, { src ->
        val item = position_cov()
        for (i in 0 until 3)
            item.set(src.get(i), i)
    }, { src -> custom_state(src) }

    )
}

fun fill(pfollow_target: FOLLOW_TARGET) {
    pfollow_target.timestamp(Long)
    pfollow_target.est_capabilities(Byte)
    pfollow_target.lat(Int)
    pfollow_target.lon(Int)
    pfollow_target.alt(Float)
    pfollow_target.vel().let { item ->
        pfollow_target.vel().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pfollow_target.acc().let { item ->
        pfollow_target.acc().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pfollow_target.attitude_q().let { item ->
        pfollow_target.attitude_q().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pfollow_target.rates().let { item ->
        pfollow_target.rates().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pfollow_target.position_cov().let { item ->
        pfollow_target.position_cov().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pfollow_target.custom_state(Long)

}

fun FOLLOW_TARGET.copyInto(DST: FOLLOW_TARGET) {
    FOLLOW_TARGET.pull(
            DST, { -> timestamp() }, { -> est_capabilities() }, { -> lat() }, { -> lon() }, { -> alt() }, { dst ->
        var src = this.vel()
        for (i in 0 until 3)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.acc()
        for (i in 0 until 3)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.attitude_q()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.rates()
        for (i in 0 until 3)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.position_cov()
        for (i in 0 until 3)
            dst.set(src.get(i), i)
    }, { -> custom_state() }

    )
}

fun onHIL_STATE(phil_state: HIL_STATE) {
    val some_time_usec = phil_state.time_usec()
    val some_roll = phil_state.roll()
    val some_pitch = phil_state.pitch()
    val some_yaw = phil_state.yaw()
    val some_rollspeed = phil_state.rollspeed()
    val some_pitchspeed = phil_state.pitchspeed()
    val some_yawspeed = phil_state.yawspeed()
    val some_lat = phil_state.lat()
    val some_lon = phil_state.lon()
    val some_alt = phil_state.alt()
    val some_vx = phil_state.vx()
    val some_vy = phil_state.vy()
    val some_vz = phil_state.vz()
    val some_xacc = phil_state.xacc()
    val some_yacc = phil_state.yacc()
    val some_zacc = phil_state.zacc()

}

fun test_.HIL_STATE.copyFrom(SRC: HIL_STATE) {
    HIL_STATE.push(
            SRC, { src -> time_usec(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }, { src -> rollspeed(src) }, { src -> pitchspeed(src) }, { src -> yawspeed(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }

    )
}

fun onHOME_POSITION(phome_position: HOME_POSITION) {
    val some_latitude = phome_position.latitude()
    val some_longitude = phome_position.longitude()
    val some_altitude = phome_position.altitude()
    val some_x = phome_position.x()
    val some_y = phome_position.y()
    val some_z = phome_position.z()
    phome_position.q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_approach_x = phome_position.approach_x()
    val some_approach_y = phome_position.approach_y()
    val some_approach_z = phome_position.approach_z()

    phome_position.time_usec().let { item -> println("Receive Long pack.") }

}

fun HOME_POSITION.copyFrom(SRC: HOME_POSITION) {
    HOME_POSITION.push(
            SRC, { src -> latitude(src) }, { src -> longitude(src) }, { src -> altitude(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src ->
        val item = q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> approach_x(src) }, { src -> approach_y(src) }, { src -> approach_z(src) }, { src -> time_usec(src) }

    )
}

fun fill(phome_position: HOME_POSITION) {
    phome_position.latitude(Int)
    phome_position.longitude(Int)
    phome_position.altitude(Int)
    phome_position.x(Float)
    phome_position.y(Float)
    phome_position.z(Float)
    phome_position.q().let { item ->
        phome_position.q().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    phome_position.approach_x(Float)
    phome_position.approach_y(Float)
    phome_position.approach_z(Float)

    phome_position.time_usec(Long)

}

fun HOME_POSITION.copyInto(DST: HOME_POSITION) {
    HOME_POSITION.pull(
            DST, { -> latitude() }, { -> longitude() }, { -> altitude() }, { -> x() }, { -> y() }, { -> z() }, { dst ->
        var src = this.q()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }, { -> approach_x() }, { -> approach_y() }, { -> approach_z() }, { time_usec() != null }, { time_usec().get() }

    )
}

fun onFENCE_STATUS(pfence_status: FENCE_STATUS) {
    val some_breach_status = pfence_status.breach_status()
    val some_breach_count = pfence_status.breach_count()

    pfence_status.breach_type()?.let { item ->
        some_FENCE_BREACH = item.get()
    }
    val some_breach_time = pfence_status.breach_time()

}

fun FENCE_STATUS.copyFrom(SRC: FENCE_STATUS) {
    FENCE_STATUS.push(
            SRC, { src -> breach_status(src) }, { src -> breach_count(src) }, { src -> breach_type(src) }, { src -> breach_time(src) }

    )
}

fun fill(pfence_status: FENCE_STATUS) {
    pfence_status.breach_status(Byte)
    pfence_status.breach_count(Short)

    pfence_status.breach_type(some_FENCE_BREACH)
    pfence_status.breach_time(Int)

}

fun FENCE_STATUS.copyInto(DST: FENCE_STATUS) {
    FENCE_STATUS.pull(
            DST, { -> breach_status() }, { -> breach_count() }, { breach_type() != null }, { breach_type()!!.get() }, { -> breach_time() }

    )
}

fun onREMOTE_LOG_BLOCK_STATUS(premote_log_block_status: REMOTE_LOG_BLOCK_STATUS) {
    val some_target_system = premote_log_block_status.target_system()
    val some_target_component = premote_log_block_status.target_component()
    val some_seqno = premote_log_block_status.seqno()

    premote_log_block_status.status()?.let { item ->
        some_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES = item.get()
    }

}

fun REMOTE_LOG_BLOCK_STATUS.copyFrom(SRC: REMOTE_LOG_BLOCK_STATUS) {
    REMOTE_LOG_BLOCK_STATUS.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> seqno(src) }, { src -> status(src) }

    )
}

fun fill(premote_log_block_status: REMOTE_LOG_BLOCK_STATUS) {
    premote_log_block_status.target_system(Byte)
    premote_log_block_status.target_component(Byte)
    premote_log_block_status.seqno(Int)

    premote_log_block_status.status(some_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)

}

fun REMOTE_LOG_BLOCK_STATUS.copyInto(DST: REMOTE_LOG_BLOCK_STATUS) {
    REMOTE_LOG_BLOCK_STATUS.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> seqno() }, { status() != null }, { status()!!.get() }

    )
}

fun onOBSTACLE_DISTANCE(pobstacle_distance: OBSTACLE_DISTANCE) {
    val some_time_usec = pobstacle_distance.time_usec()

    pobstacle_distance.sensor_type()?.let { item ->
        some_MAV_DISTANCE_SENSOR = item.get()
    }
    pobstacle_distance.distances().let { item ->
        for (index in 0 until item.len())
            Short = item.get(index)
    }
    val some_increment = pobstacle_distance.increment()
    val some_min_distance = pobstacle_distance.min_distance()
    val some_max_distance = pobstacle_distance.max_distance()

}

fun OBSTACLE_DISTANCE.copyFrom(SRC: OBSTACLE_DISTANCE) {
    OBSTACLE_DISTANCE.push(
            SRC, { src -> time_usec(src) }, { src -> sensor_type(src) }, { src ->
        val item = distances()
        for (i in 0 until 72)
            item.set(src.get(i), i)
    }, { src -> increment(src) }, { src -> min_distance(src) }, { src -> max_distance(src) }

    )
}

fun fill(pobstacle_distance: OBSTACLE_DISTANCE) {
    pobstacle_distance.time_usec(Long)

    pobstacle_distance.sensor_type(some_MAV_DISTANCE_SENSOR)
    pobstacle_distance.distances().let { item ->
        pobstacle_distance.distances().let { item ->
            for (index in 0 until item.len())
                item.set(Short, index)
        }
    }
    pobstacle_distance.increment(Byte)
    pobstacle_distance.min_distance(Short)
    pobstacle_distance.max_distance(Short)

}

fun OBSTACLE_DISTANCE.copyInto(DST: OBSTACLE_DISTANCE) {
    OBSTACLE_DISTANCE.pull(
            DST, { -> time_usec() }, { sensor_type() != null }, { sensor_type()!!.get() }, { dst ->
        var src = this.distances()
        for (i in 0 until 72)
            dst.set(src.get(i), i)
    }, { -> increment() }, { -> min_distance() }, { -> max_distance() }

    )
}

fun onGPS2_RAW(pgps2_raw: GPS2_RAW) {
    val some_time_usec = pgps2_raw.time_usec()

    pgps2_raw.fix_type()?.let { item ->
        some_GPS_FIX_TYPE = item.get()
    }
    val some_lat = pgps2_raw.lat()
    val some_lon = pgps2_raw.lon()
    val some_alt = pgps2_raw.alt()
    val some_eph = pgps2_raw.eph()
    val some_epv = pgps2_raw.epv()
    val some_vel = pgps2_raw.vel()
    val some_cog = pgps2_raw.cog()
    val some_satellites_visible = pgps2_raw.satellites_visible()
    val some_dgps_numch = pgps2_raw.dgps_numch()
    val some_dgps_age = pgps2_raw.dgps_age()

}

fun GPS2_RAW.copyFrom(SRC: GPS2_RAW) {
    GPS2_RAW.push(
            SRC, { src -> time_usec(src) }, { src -> fix_type(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> eph(src) }, { src -> epv(src) }, { src -> vel(src) }, { src -> cog(src) }, { src -> satellites_visible(src) }, { src -> dgps_numch(src) }, { src -> dgps_age(src) }

    )
}

fun fill(pgps2_raw: GPS2_RAW) {
    pgps2_raw.time_usec(Long)

    pgps2_raw.fix_type(some_GPS_FIX_TYPE)
    pgps2_raw.lat(Int)
    pgps2_raw.lon(Int)
    pgps2_raw.alt(Int)
    pgps2_raw.eph(Short)
    pgps2_raw.epv(Short)
    pgps2_raw.vel(Short)
    pgps2_raw.cog(Short)
    pgps2_raw.satellites_visible(Byte)
    pgps2_raw.dgps_numch(Byte)
    pgps2_raw.dgps_age(Int)

}

fun GPS2_RAW.copyInto(DST: GPS2_RAW) {
    GPS2_RAW.pull(
            DST, { -> time_usec() }, { fix_type() != null }, { fix_type()!!.get() }, { -> lat() }, { -> lon() }, { -> alt() }, { -> eph() }, { -> epv() }, { -> vel() }, { -> cog() }, { -> satellites_visible() }, { -> dgps_numch() }, { -> dgps_age() }

    )
}

fun onREQUEST_DATA_STREAM(prequest_data_stream: REQUEST_DATA_STREAM) {
    val some_target_system = prequest_data_stream.target_system()
    val some_target_component = prequest_data_stream.target_component()
    val some_req_stream_id = prequest_data_stream.req_stream_id()
    val some_req_message_rate = prequest_data_stream.req_message_rate()
    val some_start_stop = prequest_data_stream.start_stop()

}

fun test_.REQUEST_DATA_STREAM.copyFrom(SRC: REQUEST_DATA_STREAM) {
    REQUEST_DATA_STREAM.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> req_stream_id(src) }, { src -> req_message_rate(src) }, { src -> start_stop(src) }

    )
}

fun onMEMORY_VECT(pmemory_vect: MEMORY_VECT) {
    val some_address = pmemory_vect.address()
    val some_ver = pmemory_vect.ver()
    val some_typE = pmemory_vect.typE()
    pmemory_vect.value().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun MEMORY_VECT.copyFrom(SRC: MEMORY_VECT) {
    MEMORY_VECT.push(
            SRC, { src -> address(src) }, { src -> ver(src) }, { src -> typE(src) }, { src ->
        val item = value()
        for (i in 0 until 32)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pmemory_vect: MEMORY_VECT) {
    pmemory_vect.address(Short)
    pmemory_vect.ver(Byte)
    pmemory_vect.typE(Byte)
    pmemory_vect.value().let { item ->
        pmemory_vect.value().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun MEMORY_VECT.copyInto(DST: MEMORY_VECT) {
    MEMORY_VECT.pull(
            DST, { -> address() }, { -> ver() }, { -> typE() }, { dst ->
        var src = this.value()
        for (i in 0 until 32)
            dst.set(src.get(i), i)
    }

    )
}

fun onPARAM_EXT_REQUEST_READ(pparam_ext_request_read: PARAM_EXT_REQUEST_READ) {
    val some_target_system = pparam_ext_request_read.target_system()
    val some_target_component = pparam_ext_request_read.target_component()

    pparam_ext_request_read.param_id()?.let { item -> char = item.get() }
    val some_param_index = pparam_ext_request_read.param_index()

}

fun PARAM_EXT_REQUEST_READ.copyFrom(SRC: PARAM_EXT_REQUEST_READ) {
    PARAM_EXT_REQUEST_READ.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = param_id(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> param_index(src) }

    )
}

fun fill(pparam_ext_request_read: PARAM_EXT_REQUEST_READ) {
    pparam_ext_request_read.target_system(Byte)
    pparam_ext_request_read.target_component(Byte)

    pparam_ext_request_read.param_id(some_String, null)
    pparam_ext_request_read.param_index(Short)

}

fun PARAM_EXT_REQUEST_READ.copyInto(DST: PARAM_EXT_REQUEST_READ) {
    PARAM_EXT_REQUEST_READ.pull(
            DST, { -> target_system() }, { -> target_component() }, { param_id()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = param_id()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { -> param_index() }

    )
}

fun onHIL_CONTROLS(phil_controls: HIL_CONTROLS) {
    val some_time_usec = phil_controls.time_usec()
    val some_roll_ailerons = phil_controls.roll_ailerons()
    val some_pitch_elevator = phil_controls.pitch_elevator()
    val some_yaw_rudder = phil_controls.yaw_rudder()
    val some_throttle = phil_controls.throttle()
    val some_aux1 = phil_controls.aux1()
    val some_aux2 = phil_controls.aux2()
    val some_aux3 = phil_controls.aux3()
    val some_aux4 = phil_controls.aux4()

    phil_controls.mode()?.let { item ->
        some_MAV_MODE = item.get()
    }
    val some_nav_mode = phil_controls.nav_mode()

}

fun test_.HIL_CONTROLS.copyFrom(SRC: HIL_CONTROLS) {
    HIL_CONTROLS.push(
            SRC, { src -> time_usec(src) }, { src -> roll_ailerons(src) }, { src -> pitch_elevator(src) }, { src -> yaw_rudder(src) }, { src -> throttle(src) }, { src -> aux1(src) }, { src -> aux2(src) }, { src -> aux3(src) }, { src -> aux4(src) }, { src -> mode(src) }, { src -> nav_mode(src) }

    )
}

fun onHIL_SENSOR(phil_sensor: HIL_SENSOR) {
    val some_time_usec = phil_sensor.time_usec()
    val some_xacc = phil_sensor.xacc()
    val some_yacc = phil_sensor.yacc()
    val some_zacc = phil_sensor.zacc()
    val some_xgyro = phil_sensor.xgyro()
    val some_ygyro = phil_sensor.ygyro()
    val some_zgyro = phil_sensor.zgyro()
    val some_xmag = phil_sensor.xmag()
    val some_ymag = phil_sensor.ymag()
    val some_zmag = phil_sensor.zmag()
    val some_abs_pressure = phil_sensor.abs_pressure()
    val some_diff_pressure = phil_sensor.diff_pressure()
    val some_pressure_alt = phil_sensor.pressure_alt()
    val some_temperature = phil_sensor.temperature()
    val some_fields_updated = phil_sensor.fields_updated()

}

fun HIL_SENSOR.copyFrom(SRC: HIL_SENSOR) {
    HIL_SENSOR.push(
            SRC, { src -> time_usec(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }, { src -> xgyro(src) }, { src -> ygyro(src) }, { src -> zgyro(src) }, { src -> xmag(src) }, { src -> ymag(src) }, { src -> zmag(src) }, { src -> abs_pressure(src) }, { src -> diff_pressure(src) }, { src -> pressure_alt(src) }, { src -> temperature(src) }, { src -> fields_updated(src) }

    )
}

fun fill(phil_sensor: HIL_SENSOR) {
    phil_sensor.time_usec(Long)
    phil_sensor.xacc(Float)
    phil_sensor.yacc(Float)
    phil_sensor.zacc(Float)
    phil_sensor.xgyro(Float)
    phil_sensor.ygyro(Float)
    phil_sensor.zgyro(Float)
    phil_sensor.xmag(Float)
    phil_sensor.ymag(Float)
    phil_sensor.zmag(Float)
    phil_sensor.abs_pressure(Float)
    phil_sensor.diff_pressure(Float)
    phil_sensor.pressure_alt(Float)
    phil_sensor.temperature(Float)
    phil_sensor.fields_updated(Int)

}

fun HIL_SENSOR.copyInto(DST: HIL_SENSOR) {
    HIL_SENSOR.pull(
            DST, { -> time_usec() }, { -> xacc() }, { -> yacc() }, { -> zacc() }, { -> xgyro() }, { -> ygyro() }, { -> zgyro() }, { -> xmag() }, { -> ymag() }, { -> zmag() }, { -> abs_pressure() }, { -> diff_pressure() }, { -> pressure_alt() }, { -> temperature() }, { -> fields_updated() }

    )
}

fun onSETUP_SIGNING(psetup_signing: SETUP_SIGNING) {
    val some_target_system = psetup_signing.target_system()
    val some_target_component = psetup_signing.target_component()
    psetup_signing.secret_key().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    val some_initial_timestamp = psetup_signing.initial_timestamp()

}

fun SETUP_SIGNING.copyFrom(SRC: SETUP_SIGNING) {
    SETUP_SIGNING.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = secret_key()
        for (i in 0 until 32)
            item.set(src.get(i), i)
    }, { src -> initial_timestamp(src) }

    )
}

fun fill(psetup_signing: SETUP_SIGNING) {
    psetup_signing.target_system(Byte)
    psetup_signing.target_component(Byte)
    psetup_signing.secret_key().let { item ->
        psetup_signing.secret_key().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    psetup_signing.initial_timestamp(Long)

}

fun SETUP_SIGNING.copyInto(DST: SETUP_SIGNING) {
    SETUP_SIGNING.pull(
            DST, { -> target_system() }, { -> target_component() }, { dst ->
        var src = this.secret_key()
        for (i in 0 until 32)
            dst.set(src.get(i), i)
    }, { -> initial_timestamp() }

    )
}

fun onGPS_RTK(pgps_rtk: GPS_RTK) {
    val some_time_last_baseline_ms = pgps_rtk.time_last_baseline_ms()
    val some_rtk_receiver_id = pgps_rtk.rtk_receiver_id()
    val some_wn = pgps_rtk.wn()
    val some_tow = pgps_rtk.tow()
    val some_rtk_health = pgps_rtk.rtk_health()
    val some_rtk_rate = pgps_rtk.rtk_rate()
    val some_nsats = pgps_rtk.nsats()
    val some_baseline_coords_type = pgps_rtk.baseline_coords_type()
    val some_baseline_a_mm = pgps_rtk.baseline_a_mm()
    val some_baseline_b_mm = pgps_rtk.baseline_b_mm()
    val some_baseline_c_mm = pgps_rtk.baseline_c_mm()
    val some_accuracy = pgps_rtk.accuracy()
    val some_iar_num_hypotheses = pgps_rtk.iar_num_hypotheses()

}

fun GPS_RTK.copyFrom(SRC: GPS_RTK) {
    GPS_RTK.push(
            SRC, { src -> time_last_baseline_ms(src) }, { src -> rtk_receiver_id(src) }, { src -> wn(src) }, { src -> tow(src) }, { src -> rtk_health(src) }, { src -> rtk_rate(src) }, { src -> nsats(src) }, { src -> baseline_coords_type(src) }, { src -> baseline_a_mm(src) }, { src -> baseline_b_mm(src) }, { src -> baseline_c_mm(src) }, { src -> accuracy(src) }, { src -> iar_num_hypotheses(src) }

    )
}

fun fill(pgps_rtk: GPS_RTK) {
    pgps_rtk.time_last_baseline_ms(Int)
    pgps_rtk.rtk_receiver_id(Byte)
    pgps_rtk.wn(Short)
    pgps_rtk.tow(Int)
    pgps_rtk.rtk_health(Byte)
    pgps_rtk.rtk_rate(Byte)
    pgps_rtk.nsats(Byte)
    pgps_rtk.baseline_coords_type(Byte)
    pgps_rtk.baseline_a_mm(Int)
    pgps_rtk.baseline_b_mm(Int)
    pgps_rtk.baseline_c_mm(Int)
    pgps_rtk.accuracy(Int)
    pgps_rtk.iar_num_hypotheses(Int)

}

fun GPS_RTK.copyInto(DST: GPS_RTK) {
    GPS_RTK.pull(
            DST, { -> time_last_baseline_ms() }, { -> rtk_receiver_id() }, { -> wn() }, { -> tow() }, { -> rtk_health() }, { -> rtk_rate() }, { -> nsats() }, { -> baseline_coords_type() }, { -> baseline_a_mm() }, { -> baseline_b_mm() }, { -> baseline_c_mm() }, { -> accuracy() }, { -> iar_num_hypotheses() }

    )
}

fun onPARAM_REQUEST_LIST(pparam_request_list: PARAM_REQUEST_LIST) {
    val some_target_system = pparam_request_list.target_system()
    val some_target_component = pparam_request_list.target_component()

}

fun test_.PARAM_REQUEST_LIST.copyFrom(SRC: PARAM_REQUEST_LIST) {
    PARAM_REQUEST_LIST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }

    )
}

fun onUAVIONIX_ADSB_OUT_CFG(puavionix_adsb_out_cfg: UAVIONIX_ADSB_OUT_CFG) {
    val some_ICAO = puavionix_adsb_out_cfg.ICAO()

    puavionix_adsb_out_cfg.callsign()?.let { item -> char = item.get() }

    puavionix_adsb_out_cfg.emitterType()?.let { item ->
        some_ADSB_EMITTER_TYPE = item.get()
    }

    puavionix_adsb_out_cfg.aircraftSize()?.let { item ->
        some_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE = item.get()
    }

    puavionix_adsb_out_cfg.gpsOffsetLat()?.let { item ->
        some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT = item.get()
    }

    puavionix_adsb_out_cfg.gpsOffsetLon()?.let { item ->
        some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON = item.get()
    }
    val some_stallSpeed = puavionix_adsb_out_cfg.stallSpeed()

    puavionix_adsb_out_cfg.rfSelect()?.let { item ->
        some_UAVIONIX_ADSB_OUT_RF_SELECT = item.get()
    }

}

fun UAVIONIX_ADSB_OUT_CFG.copyFrom(SRC: UAVIONIX_ADSB_OUT_CFG) {
    UAVIONIX_ADSB_OUT_CFG.push(
            SRC, { src -> ICAO(src) }, { src ->
        val item = callsign(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> emitterType(src) }, { src -> aircraftSize(src) }, { src -> gpsOffsetLat(src) }, { src -> gpsOffsetLon(src) }, { src -> stallSpeed(src) }, { src -> rfSelect(src) }

    )
}

fun fill(puavionix_adsb_out_cfg: UAVIONIX_ADSB_OUT_CFG) {
    puavionix_adsb_out_cfg.ICAO(Int)

    puavionix_adsb_out_cfg.callsign(some_String, null)

    puavionix_adsb_out_cfg.emitterType(some_ADSB_EMITTER_TYPE)

    puavionix_adsb_out_cfg.aircraftSize(some_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)

    puavionix_adsb_out_cfg.gpsOffsetLat(some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)

    puavionix_adsb_out_cfg.gpsOffsetLon(some_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)
    puavionix_adsb_out_cfg.stallSpeed(Short)

    puavionix_adsb_out_cfg.rfSelect(some_UAVIONIX_ADSB_OUT_RF_SELECT)

}

fun UAVIONIX_ADSB_OUT_CFG.copyInto(DST: UAVIONIX_ADSB_OUT_CFG) {
    UAVIONIX_ADSB_OUT_CFG.pull(
            DST, { -> ICAO() }, { callsign()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = callsign()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { emitterType() != null }, { emitterType()!!.get() }, { aircraftSize() != null }, { aircraftSize()!!.get() }, { gpsOffsetLat() != null }, { gpsOffsetLat()!!.get() }, { gpsOffsetLon() != null }, { gpsOffsetLon()!!.get() }, { -> stallSpeed() }, { rfSelect() != null }, { rfSelect()!!.get() }

    )
}

fun onLANDING_TARGET(planding_target: LANDING_TARGET) {
    val some_time_usec = planding_target.time_usec()
    val some_target_num = planding_target.target_num()

    planding_target.frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }
    val some_angle_x = planding_target.angle_x()
    val some_angle_y = planding_target.angle_y()
    val some_distance = planding_target.distance()
    val some_size_x = planding_target.size_x()
    val some_size_y = planding_target.size_y()

    planding_target.x().let { item -> println("Receive Float pack.") }

    planding_target.y().let { item -> println("Receive Float pack.") }

    planding_target.z().let { item -> println("Receive Float pack.") }
    planding_target.q()?.let { fld ->
        fld.enumerate { d0 ->
            println("Receive Float pack.")
        }
    }

    planding_target.typE()?.let { item ->
        some_LANDING_TARGET_TYPE = item.get()
    }

    planding_target.position_valid().let { item -> println("Receive Byte pack.") }

}

fun LANDING_TARGET.copyFrom(SRC: LANDING_TARGET) {
    LANDING_TARGET.push(
            SRC, { src -> time_usec(src) }, { src -> target_num(src) }, { src -> frame(src) }, { src -> angle_x(src) }, { src -> angle_y(src) }, { src -> distance(src) }, { src -> size_x(src) }, { src -> size_y(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src, d0 -> q(src, d0) }, { src -> typE(src) }, { src -> position_valid(src) }

    )
}

fun fill(planding_target: LANDING_TARGET) {
    planding_target.time_usec(Long)
    planding_target.target_num(Byte)

    planding_target.frame(some_MAV_FRAME)
    planding_target.angle_x(Float)
    planding_target.angle_y(Float)
    planding_target.distance(Float)
    planding_target.size_x(Float)
    planding_target.size_y(Float)

    planding_target.x(Float)

    planding_target.y(Float)

    planding_target.z(Float)

    for (d0 in 0 until LANDING_TARGET.q.d0) {
        planding_target.q(Float, d0)
    }

    planding_target.typE(some_LANDING_TARGET_TYPE)

    planding_target.position_valid(Byte)

}

fun LANDING_TARGET.copyInto(DST: LANDING_TARGET) {
    LANDING_TARGET.pull(
            DST, { -> time_usec() }, { -> target_num() }, { frame() != null }, { frame()!!.get() }, { -> angle_x() }, { -> angle_y() }, { -> distance() }, { -> size_x() }, { -> size_y() }, { x() != null }, { x().get() }, { y() != null }, { y().get() }, { z() != null }, { z().get() }, { q() != null }, { d0 -> q()!!.get(d0) != null }, { d0 -> q()!!.get(d0).get() }, { typE() != null }, { typE()!!.get() }, { position_valid() != null }, { position_valid().get() }

    )
}

fun onSET_ACTUATOR_CONTROL_TARGET(pset_actuator_control_target: SET_ACTUATOR_CONTROL_TARGET) {
    val some_time_usec = pset_actuator_control_target.time_usec()
    val some_group_mlx = pset_actuator_control_target.group_mlx()
    val some_target_system = pset_actuator_control_target.target_system()
    val some_target_component = pset_actuator_control_target.target_component()
    pset_actuator_control_target.controls().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }

}

fun SET_ACTUATOR_CONTROL_TARGET.copyFrom(SRC: SET_ACTUATOR_CONTROL_TARGET) {
    SET_ACTUATOR_CONTROL_TARGET.push(
            SRC, { src -> time_usec(src) }, { src -> group_mlx(src) }, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = controls()
        for (i in 0 until 8)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pset_actuator_control_target: SET_ACTUATOR_CONTROL_TARGET) {
    pset_actuator_control_target.time_usec(Long)
    pset_actuator_control_target.group_mlx(Byte)
    pset_actuator_control_target.target_system(Byte)
    pset_actuator_control_target.target_component(Byte)
    pset_actuator_control_target.controls().let { item ->
        pset_actuator_control_target.controls().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }

}

fun SET_ACTUATOR_CONTROL_TARGET.copyInto(DST: SET_ACTUATOR_CONTROL_TARGET) {
    SET_ACTUATOR_CONTROL_TARGET.pull(
            DST, { -> time_usec() }, { -> group_mlx() }, { -> target_system() }, { -> target_component() }, { dst ->
        var src = this.controls()
        for (i in 0 until 8)
            dst.set(src.get(i), i)
    }

    )
}

fun onCONTROL_SYSTEM_STATE(pcontrol_system_state: CONTROL_SYSTEM_STATE) {
    val some_time_usec = pcontrol_system_state.time_usec()
    val some_x_acc = pcontrol_system_state.x_acc()
    val some_y_acc = pcontrol_system_state.y_acc()
    val some_z_acc = pcontrol_system_state.z_acc()
    val some_x_vel = pcontrol_system_state.x_vel()
    val some_y_vel = pcontrol_system_state.y_vel()
    val some_z_vel = pcontrol_system_state.z_vel()
    val some_x_pos = pcontrol_system_state.x_pos()
    val some_y_pos = pcontrol_system_state.y_pos()
    val some_z_pos = pcontrol_system_state.z_pos()
    val some_airspeed = pcontrol_system_state.airspeed()
    pcontrol_system_state.vel_variance().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    pcontrol_system_state.pos_variance().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    pcontrol_system_state.q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_roll_rate = pcontrol_system_state.roll_rate()
    val some_pitch_rate = pcontrol_system_state.pitch_rate()
    val some_yaw_rate = pcontrol_system_state.yaw_rate()

}

fun CONTROL_SYSTEM_STATE.copyFrom(SRC: CONTROL_SYSTEM_STATE) {
    CONTROL_SYSTEM_STATE.push(
            SRC, { src -> time_usec(src) }, { src -> x_acc(src) }, { src -> y_acc(src) }, { src -> z_acc(src) }, { src -> x_vel(src) }, { src -> y_vel(src) }, { src -> z_vel(src) }, { src -> x_pos(src) }, { src -> y_pos(src) }, { src -> z_pos(src) }, { src -> airspeed(src) }, { src ->
        val item = vel_variance()
        for (i in 0 until 3)
            item.set(src.get(i), i)
    }, { src ->
        val item = pos_variance()
        for (i in 0 until 3)
            item.set(src.get(i), i)
    }, { src ->
        val item = q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> roll_rate(src) }, { src -> pitch_rate(src) }, { src -> yaw_rate(src) }

    )
}

fun fill(pcontrol_system_state: CONTROL_SYSTEM_STATE) {
    pcontrol_system_state.time_usec(Long)
    pcontrol_system_state.x_acc(Float)
    pcontrol_system_state.y_acc(Float)
    pcontrol_system_state.z_acc(Float)
    pcontrol_system_state.x_vel(Float)
    pcontrol_system_state.y_vel(Float)
    pcontrol_system_state.z_vel(Float)
    pcontrol_system_state.x_pos(Float)
    pcontrol_system_state.y_pos(Float)
    pcontrol_system_state.z_pos(Float)
    pcontrol_system_state.airspeed(Float)
    pcontrol_system_state.vel_variance().let { item ->
        pcontrol_system_state.vel_variance().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pcontrol_system_state.pos_variance().let { item ->
        pcontrol_system_state.pos_variance().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pcontrol_system_state.q().let { item ->
        pcontrol_system_state.q().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pcontrol_system_state.roll_rate(Float)
    pcontrol_system_state.pitch_rate(Float)
    pcontrol_system_state.yaw_rate(Float)

}

fun CONTROL_SYSTEM_STATE.copyInto(DST: CONTROL_SYSTEM_STATE) {
    CONTROL_SYSTEM_STATE.pull(
            DST, { -> time_usec() }, { -> x_acc() }, { -> y_acc() }, { -> z_acc() }, { -> x_vel() }, { -> y_vel() }, { -> z_vel() }, { -> x_pos() }, { -> y_pos() }, { -> z_pos() }, { -> airspeed() }, { dst ->
        var src = this.vel_variance()
        for (i in 0 until 3)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.pos_variance()
        for (i in 0 until 3)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.q()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }, { -> roll_rate() }, { -> pitch_rate() }, { -> yaw_rate() }

    )
}

fun onSET_POSITION_TARGET_GLOBAL_INT(pset_position_target_global_int: SET_POSITION_TARGET_GLOBAL_INT) {
    val some_time_boot_ms = pset_position_target_global_int.time_boot_ms()
    val some_target_system = pset_position_target_global_int.target_system()
    val some_target_component = pset_position_target_global_int.target_component()

    pset_position_target_global_int.coordinate_frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }
    val some_type_mask = pset_position_target_global_int.type_mask()
    val some_lat_int = pset_position_target_global_int.lat_int()
    val some_lon_int = pset_position_target_global_int.lon_int()
    val some_alt = pset_position_target_global_int.alt()
    val some_vx = pset_position_target_global_int.vx()
    val some_vy = pset_position_target_global_int.vy()
    val some_vz = pset_position_target_global_int.vz()
    val some_afx = pset_position_target_global_int.afx()
    val some_afy = pset_position_target_global_int.afy()
    val some_afz = pset_position_target_global_int.afz()
    val some_yaw = pset_position_target_global_int.yaw()
    val some_yaw_rate = pset_position_target_global_int.yaw_rate()

}

fun test_.SET_POSITION_TARGET_GLOBAL_INT.copyFrom(SRC: SET_POSITION_TARGET_GLOBAL_INT) {
    SET_POSITION_TARGET_GLOBAL_INT.push(
            SRC, { src -> time_boot_ms(src) }, { src -> target_system(src) }, { src -> target_component(src) }, { src -> coordinate_frame(src) }, { src -> type_mask(src) }, { src -> lat_int(src) }, { src -> lon_int(src) }, { src -> alt(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> afx(src) }, { src -> afy(src) }, { src -> afz(src) }, { src -> yaw(src) }, { src -> yaw_rate(src) }

    )
}

fun onDATA32(pdata32: DATA32) {
    val some_typE = pdata32.typE()
    val some_len = pdata32.len()
    pdata32.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun DATA32.copyFrom(SRC: DATA32) {
    DATA32.push(
            SRC, { src -> typE(src) }, { src -> len(src) }, { src ->
        val item = daTa()
        for (i in 0 until 32)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pdata32: DATA32) {
    pdata32.typE(Byte)
    pdata32.len(Byte)
    pdata32.daTa().let { item ->
        pdata32.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun DATA32.copyInto(DST: DATA32) {
    DATA32.pull(
            DST, { -> typE() }, { -> len() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 32)
            dst.set(src.get(i), i)
    }

    )
}

fun onPING33(pping33: PING33) {

    pping33.testBOOL().let { item -> println("Receive Boolean pack.") }

    pping33.seq().let { item -> println("Receive Long pack.") }
    val some_field = pping33.field()

    pping33.field1().Field()?.let { fld ->
        some_Int = fld.d0()
        some_Int = PING33.field1.d0_max
        some_Int = PING33.field1.d1
        some_Int = PING33.field1.d2

        fld.enumerate { d0, d1, d2 ->
            val some_field1 = fld.get(d0, d1, d2)

        }
    }
    pping33.field12()?.let { fld ->
        fld.enumerate { d0, d1, d2 ->
            val some_field12 = fld.get(d0, d1, d2)

        }
    }
    pping33.field13()?.let { fld ->
        fld.enumerate { d0, d1, d2 ->
            println("Receive Int pack.")
        }
    }

    PING33.TTTT.enumerate(pping33) { item, d0, d1, d2 ->
        val some_TTTT = pping33.TTTT(d0, d1, d2)

    }


    pping33.WWWWWWWW().let { item -> println("Receive Int pack.") }
    val some_testBOOL2 = pping33.testBOOL2()
    val some_testBOOL3 = pping33.testBOOL3()
    val some_bit_field = pping33.bit_field()

    pping33.bit_field2().let { item -> println("Receive Byte pack.") }
    pping33.Field_Bits()?.let { fld ->
        fld.enumerate { d0, d1, d2 ->
            val some_Field_Bits = fld.get(d0, d1, d2)

        }
    }

    pping33.SparseFixAllBits().Field()?.let { fld ->
        some_Int = fld.d0()
        some_Int = PING33.SparseFixAllBits.d0_max
        some_Int = PING33.SparseFixAllBits.d1
        some_Int = PING33.SparseFixAllBits.d2

        fld.enumerate { d0, d1, d2 ->
            println("Receive Byte pack.")
        }
    }

    pping33.FixAllBits().Field()?.let { fld ->
        some_Int = fld.d0()
        some_Int = PING33.FixAllBits.d0_max
        some_Int = PING33.FixAllBits.d1
        some_Int = PING33.FixAllBits.d2

        fld.enumerate { d0, d1, d2 ->
            val some_FixAllBits = fld.get(d0, d1, d2)

        }
    }

    pping33.VarAllBits().Field()?.let { fld ->
        some_Int = fld.d0()
        some_Int = PING33.VarAllBits.d0_max
        some_Int = PING33.VarAllBits.d1
        some_Int = fld.d2()
        some_Int = PING33.VarAllBits.d2_max

        fld.enumerate { d0, d1, d2 ->
            val some_VarAllBits = fld.get(d0, d1, d2)

        }
    }

    pping33.SparseVarAllBits().Field()?.let { fld ->
        some_Int = fld.d0()
        some_Int = PING33.SparseVarAllBits.d0_max
        some_Int = PING33.SparseVarAllBits.d1
        some_Int = fld.d2()
        some_Int = PING33.SparseVarAllBits.d2_max

        fld.enumerate { d0, d1, d2 ->
            println("Receive Byte pack.")
        }
    }

    pping33.VarEachBits().Field()?.let { fld ->
        some_Int = fld.d0()
        some_Int = PING33.VarEachBits.d0_max
        some_Int = PING33.VarEachBits.d1
        some_Int = PING33.VarEachBits.d2

        fld.enumerate { d0, d1, d2 ->
            val some_VarEachBits = fld.get(d0, d1, d2)

        }
    }

    pping33.SparsVarEachBits().Field()?.let { fld ->
        some_Int = fld.d0()
        some_Int = PING33.SparsVarEachBits.d0_max
        some_Int = PING33.SparsVarEachBits.d1
        some_Int = PING33.SparsVarEachBits.d2

        fld.enumerate { d0, d1, d2 ->
            println("Receive Short pack.")
        }
    }

    pping33.testBOOLX().let { item -> println("Receive Boolean pack.") }

    pping33.testBOOL2X().let { item -> println("Receive Boolean pack.") }

    pping33.testBOOL3X().let { item -> println("Receive Boolean pack.") }

    pping33.MMMMMM()?.let { item ->
        some_MAV_MODE = item.get()
    }

    pping33.field44().Field()?.let { fld ->
        some_Int = PING33.field44.d0
        some_Int = PING33.field44.d1
        some_Int = fld.d2()
        some_Int = PING33.field44.d2_max

        fld.enumerate { d0, d1, d2 ->
            val some_field44 = fld.get(d0, d1, d2)

        }
    }

    pping33.field634().Field()?.let { fld ->
        some_Int = PING33.field634.d0
        some_Int = PING33.field634.d1
        some_Int = fld.d2()
        some_Int = PING33.field634.d2_max

        fld.enumerate { d0, d1, d2 ->
            val some_field634 = fld.get(d0, d1, d2)

        }
    }

    pping33.field33344().Field()?.let { fld ->
        some_Int = PING33.field33344.d0
        some_Int = PING33.field33344.d1
        some_Int = fld.d2()
        some_Int = PING33.field33344.d2_max

        fld.enumerate { d0, d1, d2 ->
            println("Receive Int pack.")
        }
    }

    pping33.field333634().Field()?.let { fld ->
        some_Int = PING33.field333634.d0
        some_Int = PING33.field333634.d1
        some_Int = fld.d2()
        some_Int = PING33.field333634.d2_max

        fld.enumerate { d0, d1, d2 ->
            println("Receive Int pack.")
        }
    }
    pping33.field__()?.let { fld ->
        fld.enumerate { d0, d1, d2 ->
            println("Receive Int pack.")
        }
    }

    PING33.field6.enumerate(pping33) { item, d0, d1, d2 ->
        val some_field6 = pping33.field6(d0, d1, d2)

    }


    pping33.field63().Field()?.let { fld ->
        some_Int = PING33.field63.d0
        some_Int = PING33.field63.d1
        some_Int = fld.d2()
        some_Int = PING33.field63.d2_max

        fld.enumerate { d0, d1, d2 ->
            val some_field63 = fld.get(d0, d1, d2)

        }
    }
    pping33.uid2()?.let { fld ->
        fld.enumerate { d0 ->
            println("Receive Byte pack.")
        }
    }
    pping33.field2()?.let { fld ->
        fld.enumerate { d0, d1, d2 ->
            println("Receive Int pack.")
        }
    }
    pping33.field4()?.let { fld ->
        fld.enumerate { d0, d1, d2 ->
            println("Receive Int pack.")
        }
    }

    pping33.stringtest1()?.let { item -> char = item.get() }

    pping33.stringtest2().Field()?.let { fld ->
        some_Int = PING33.stringtest2.d0
        some_Int = PING33.stringtest2.d1
        some_Int = fld.d2()
        some_Int = PING33.stringtest2.d2_max

        fld.enumerate { d0, d1, d2 ->
            char = item.get()
        }
    }

    pping33.stringtest3()?.let { item -> char = item.get() }

    pping33.stringtest4()?.let { item -> char = item.get() }

}

fun PING33.copyFrom(SRC: PING33) {
    PING33.push(
            SRC, { src -> testBOOL(src) }, { src -> seq(src) }, { src -> field(src) }, { d0 -> field1().Initializer()!!.init(d0) }, { src, d0, d1, d2 -> field1().Field()!!.set(src, d0, d1, d2) }, { src, d0, d1, d2 -> field12(src, d0, d1, d2) }, { src, d0, d1, d2 -> field13(src, d0, d1, d2) }, { src, d0, d1, d2 -> TTTT(src, d0, d1, d2) }, { src -> WWWWWWWW(src) }, { src -> testBOOL2(src) }, { src -> testBOOL3(src) }, { src -> bit_field(src) }, { src -> bit_field2(src) }, { src, d0, d1, d2 -> Field_Bits(src, d0, d1, d2) }, { d0 -> SparseFixAllBits().Initializer()!!.init(d0) }, { src, d0, d1, d2 -> SparseFixAllBits().Field()!!.set(src, d0, d1, d2) }, { d0 -> FixAllBits().Initializer()!!.init(d0) }, { src, d0, d1, d2 -> FixAllBits().Field()!!.set(src, d0, d1, d2) }, { d0, d2 -> VarAllBits().Initializer()!!.init(d0, d2) }, { src, d0, d1, d2 -> VarAllBits().Field()!!.set(src, d0, d1, d2) }, { d0, d2 -> SparseVarAllBits().Initializer()!!.init(d0, d2) }, { src, d0, d1, d2 -> SparseVarAllBits().Field()!!.set(src, d0, d1, d2) }, { d0 -> VarEachBits().Initializer()!!.init(d0) }, { src, d0, d1, d2 -> VarEachBits().Field()!!.set(src, d0, d1, d2) }, { d0 -> SparsVarEachBits().Initializer()!!.init(d0) }, { src, d0, d1, d2 -> SparsVarEachBits().Field()!!.set(src, d0, d1, d2) }, { src -> testBOOLX(src) }, { src -> testBOOL2X(src) }, { src -> testBOOL3X(src) }, { src -> MMMMMM(src) }, { d2 -> field44().Initializer()!!.init(d2) }, { src, d0, d1, d2 -> field44().Field()!!.set(src, d0, d1, d2) }, { d2 -> field634().Initializer()!!.init(d2) }, { src, d0, d1, d2 -> field634().Field()!!.set(src, d0, d1, d2) }, { d2 -> field33344().Initializer()!!.init(d2) }, { src, d0, d1, d2 -> field33344().Field()!!.set(src, d0, d1, d2) }, { d2 -> field333634().Initializer()!!.init(d2) }, { src, d0, d1, d2 -> field333634().Field()!!.set(src, d0, d1, d2) }, { src, d0, d1, d2 -> field__(src, d0, d1, d2) }, { src, d0, d1, d2 -> field6(src, d0, d1, d2) }, { d2 -> field63().Initializer()!!.init(d2) }, { src, d0, d1, d2 -> field63().Field()!!.set(src, d0, d1, d2) }, { src, d0 -> uid2(src, d0) }, { src, d0, d1, d2 -> field2(src, d0, d1, d2) }, { src, d0, d1, d2 -> field4(src, d0, d1, d2) }, { src ->
        val item = stringtest1(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { d2 -> stringtest2().Initializer()!!.init(d2) }, { src, d0, d1, d2 ->
        val item = stringtest2().Field()!!.set(src.len(), d0, d1, d2)
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src ->
        val item = stringtest3(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src ->
        val item = stringtest4(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun fill(pping33: PING33) {

    pping33.testBOOL(Boolean)

    pping33.seq(Long)
    pping33.field(Long)

    pping33.field1().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until fld.d0())
                for (d1 in 0 until PING33.field1.d1)
                    for (d2 in 0 until PING33.field1.d2) {
                        fld.set(Int, d0, d1, d2)

                    }
        }
    }

    for (d0 in 0 until PING33.field12.d0)
        for (d1 in 0 until PING33.field12.d1)
            for (d2 in 0 until PING33.field12.d2) {
                pping33.field12(Int, d0, d1, d2)

            }

    for (d0 in 0 until PING33.field13.d0)
        for (d1 in 0 until PING33.field13.d1)
            for (d2 in 0 until PING33.field13.d2) {
                pping33.field13(Int, d0, d1, d2)
            }
    PING33.TTTT.enumerate(pping33) { item, d0, d1, d2 ->
        pping33.TTTT(Int, d0, d1, d2)

    }

    pping33.WWWWWWWW(Int)
    pping33.testBOOL2(Boolean)
    pping33.testBOOL3(Boolean)
    pping33.bit_field(Byte)

    pping33.bit_field2(Byte)

    for (d0 in 0 until PING33.Field_Bits.d0)
        for (d1 in 0 until PING33.Field_Bits.d1)
            for (d2 in 0 until PING33.Field_Bits.d2) {
                pping33.Field_Bits(Byte, d0, d1, d2)

            }

    pping33.SparseFixAllBits().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until fld.d0())
                for (d1 in 0 until PING33.SparseFixAllBits.d1)
                    for (d2 in 0 until PING33.SparseFixAllBits.d2) {
                        fld.set(Byte, d0, d1, d2)
                    }
        }
    }

    pping33.FixAllBits().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until fld.d0())
                for (d1 in 0 until PING33.FixAllBits.d1)
                    for (d2 in 0 until PING33.FixAllBits.d2) {
                        fld.set(Byte, d0, d1, d2)

                    }
        }
    }

    pping33.VarAllBits().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int, some_Int).let { fld ->
            for (d0 in 0 until fld.d0())
                for (d1 in 0 until PING33.VarAllBits.d1)
                    for (d2 in 0 until fld.d2()) {
                        fld.set(Byte, d0, d1, d2)

                    }
        }
    }

    pping33.SparseVarAllBits().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int, some_Int).let { fld ->
            for (d0 in 0 until fld.d0())
                for (d1 in 0 until PING33.SparseVarAllBits.d1)
                    for (d2 in 0 until fld.d2()) {
                        fld.set(Byte, d0, d1, d2)
                    }
        }
    }

    pping33.VarEachBits().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until fld.d0())
                for (d1 in 0 until PING33.VarEachBits.d1)
                    for (d2 in 0 until PING33.VarEachBits.d2) {
                        fld.set(Byte, d0, d1, d2)

                    }
        }
    }

    pping33.SparsVarEachBits().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until fld.d0())
                for (d1 in 0 until PING33.SparsVarEachBits.d1)
                    for (d2 in 0 until PING33.SparsVarEachBits.d2) {
                        fld.set(Short, d0, d1, d2)
                    }
        }
    }

    pping33.testBOOLX(Boolean)

    pping33.testBOOL2X(Boolean)

    pping33.testBOOL3X(Boolean)

    pping33.MMMMMM(some_MAV_MODE)

    pping33.field44().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until PING33.field44.d0)
                for (d1 in 0 until PING33.field44.d1)
                    for (d2 in 0 until fld.d2()) {
                        fld.set(Int, d0, d1, d2)

                    }
        }
    }

    pping33.field634().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until PING33.field634.d0)
                for (d1 in 0 until PING33.field634.d1)
                    for (d2 in 0 until fld.d2()) {
                        fld.set(Int, d0, d1, d2)

                    }
        }
    }

    pping33.field33344().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until PING33.field33344.d0)
                for (d1 in 0 until PING33.field33344.d1)
                    for (d2 in 0 until fld.d2()) {
                        fld.set(Int, d0, d1, d2)
                    }
        }
    }

    pping33.field333634().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until PING33.field333634.d0)
                for (d1 in 0 until PING33.field333634.d1)
                    for (d2 in 0 until fld.d2()) {
                        fld.set(Int, d0, d1, d2)
                    }
        }
    }

    for (d0 in 0 until PING33.field__.d0)
        for (d1 in 0 until PING33.field__.d1)
            for (d2 in 0 until PING33.field__.d2) {
                pping33.field__(Int, d0, d1, d2)
            }
    PING33.field6.enumerate(pping33) { item, d0, d1, d2 ->
        pping33.field6(Int, d0, d1, d2)

    }

    pping33.field63().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until PING33.field63.d0)
                for (d1 in 0 until PING33.field63.d1)
                    for (d2 in 0 until fld.d2()) {
                        fld.set(Int, d0, d1, d2)

                    }
        }
    }

    for (d0 in 0 until PING33.uid2.d0) {
        pping33.uid2(Byte, d0)
    }

    for (d0 in 0 until PING33.field2.d0)
        for (d1 in 0 until PING33.field2.d1)
            for (d2 in 0 until PING33.field2.d2) {
                pping33.field2(Int, d0, d1, d2)
            }

    for (d0 in 0 until PING33.field4.d0)
        for (d1 in 0 until PING33.field4.d1)
            for (d2 in 0 until PING33.field4.d2) {
                pping33.field4(Int, d0, d1, d2)
            }

    pping33.stringtest1(some_String, null)

    pping33.stringtest2().Initializer()?.let { field_initializer ->
        field_initializer.init(some_Int).let { fld ->
            for (d0 in 0 until PING33.stringtest2.d0)
                for (d1 in 0 until PING33.stringtest2.d1)
                    for (d2 in 0 until fld.d2()) {
                        fld.set(some_String, d0, d1, d2, null)
                    }
        }
    }

    pping33.stringtest3(some_String, null)

    pping33.stringtest4(some_String, null)

}

fun PING33.copyInto(DST: PING33) {
    PING33.pull(
            DST, { testBOOL() != null }, { testBOOL().get() }, { seq() != null }, { seq().get() }, { -> field() }, { init_field -> field1().Field()?.let { fld -> init_field.init(fld.d0()) } }, { d0, d1, d2 -> field1().Field()!!.get(d0, d1, d2) }, { field12() != null }, { d0, d1, d2 -> field12()!!.get(d0, d1, d2) }, { field13() != null }, { d0, d1, d2 -> field13()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> field13()!!.get(d0, d1, d2).get() }, { d0, d1, d2 -> TTTT(d0, d1, d2) }, { WWWWWWWW() != null }, { WWWWWWWW().get() }, { -> testBOOL2() }, { -> testBOOL3() }, { -> bit_field() }, { bit_field2() != null }, { bit_field2().get() }, { Field_Bits() != null }, { d0, d1, d2 -> Field_Bits()!!.get(d0, d1, d2) }, { init_field -> SparseFixAllBits().Field()?.let { fld -> init_field.init(fld.d0()) } }, { d0, d1, d2 -> SparseFixAllBits().Field()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> SparseFixAllBits().Field()!!.get(d0, d1, d2).get() }, { init_field -> FixAllBits().Field()?.let { fld -> init_field.init(fld.d0()) } }, { d0, d1, d2 -> FixAllBits().Field()!!.get(d0, d1, d2) }, { init_field -> VarAllBits().Field()?.let { fld -> init_field.init(fld.d0(), fld.d2()) } }, { d0, d1, d2 -> VarAllBits().Field()!!.get(d0, d1, d2) }, { init_field -> SparseVarAllBits().Field()?.let { fld -> init_field.init(fld.d0(), fld.d2()) } }, { d0, d1, d2 -> SparseVarAllBits().Field()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> SparseVarAllBits().Field()!!.get(d0, d1, d2).get() }, { init_field -> VarEachBits().Field()?.let { fld -> init_field.init(fld.d0()) } }, { d0, d1, d2 -> VarEachBits().Field()!!.get(d0, d1, d2) }, { init_field -> SparsVarEachBits().Field()?.let { fld -> init_field.init(fld.d0()) } }, { d0, d1, d2 -> SparsVarEachBits().Field()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> SparsVarEachBits().Field()!!.get(d0, d1, d2).get() }, { testBOOLX() != null }, { testBOOLX().get() }, { testBOOL2X() != null }, { testBOOL2X().get() }, { testBOOL3X() != null }, { testBOOL3X().get() }, { MMMMMM() != null }, { MMMMMM()!!.get() }, { init_field -> field44().Field()?.let { fld -> init_field.init(fld.d2()) } }, { d0, d1, d2 -> field44().Field()!!.get(d0, d1, d2) }, { init_field -> field634().Field()?.let { fld -> init_field.init(fld.d2()) } }, { d0, d1, d2 -> field634().Field()!!.get(d0, d1, d2) }, { init_field -> field33344().Field()?.let { fld -> init_field.init(fld.d2()) } }, { d0, d1, d2 -> field33344().Field()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> field33344().Field()!!.get(d0, d1, d2).get() }, { init_field -> field333634().Field()?.let { fld -> init_field.init(fld.d2()) } }, { d0, d1, d2 -> field333634().Field()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> field333634().Field()!!.get(d0, d1, d2).get() }, { field__() != null }, { d0, d1, d2 -> field__()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> field__()!!.get(d0, d1, d2).get() }, { d0, d1, d2 -> field6(d0, d1, d2) }, { init_field -> field63().Field()?.let { fld -> init_field.init(fld.d2()) } }, { d0, d1, d2 -> field63().Field()!!.get(d0, d1, d2) }, { uid2() != null }, { d0 -> uid2()!!.get(d0) != null }, { d0 -> uid2()!!.get(d0).get() }, { field2() != null }, { d0, d1, d2 -> field2()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> field2()!!.get(d0, d1, d2).get() }, { field4() != null }, { d0, d1, d2 -> field4()!!.get(d0, d1, d2) != null }, { d0, d1, d2 -> field4()!!.get(d0, d1, d2).get() }, {
        stringtest1()?.let { item -> item.len() } ?: 0
    }, { dst ->
        val item = stringtest1()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { init_field -> stringtest2().Field()?.let { fld -> init_field.init(fld.d2()) } }, { d0, d1, d2 -> stringtest2().Field()!!.get(d0, d1, d2)?.let { item -> item.len() } ?: 0 }, { dst, d0: Int, d1: Int, d2: Int ->
        val item = stringtest2().Field()!!.get(d0, d1, d2)!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { stringtest3()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = stringtest3()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { stringtest4()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = stringtest4()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }

    )
}

fun onVFR_HUD(pvfr_hud: VFR_HUD) {
    val some_airspeed = pvfr_hud.airspeed()
    val some_groundspeed = pvfr_hud.groundspeed()
    val some_heading = pvfr_hud.heading()
    val some_throttle = pvfr_hud.throttle()
    val some_alt = pvfr_hud.alt()
    val some_climb = pvfr_hud.climb()

}

fun test_.VFR_HUD.copyFrom(SRC: VFR_HUD) {
    VFR_HUD.push(
            SRC, { src -> airspeed(src) }, { src -> groundspeed(src) }, { src -> heading(src) }, { src -> throttle(src) }, { src -> alt(src) }, { src -> climb(src) }

    )
}

fun onRALLY_POINT(prally_point: RALLY_POINT) {
    val some_target_system = prally_point.target_system()
    val some_target_component = prally_point.target_component()
    val some_idx = prally_point.idx()
    val some_count = prally_point.count()
    val some_lat = prally_point.lat()
    val some_lng = prally_point.lng()
    val some_alt = prally_point.alt()
    val some_break_alt = prally_point.break_alt()
    val some_land_dir = prally_point.land_dir()

    prally_point.flags()?.let { item ->
        some_RALLY_FLAGS = item.get()
    }

}

fun RALLY_POINT.copyFrom(SRC: RALLY_POINT) {
    RALLY_POINT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> idx(src) }, { src -> count(src) }, { src -> lat(src) }, { src -> lng(src) }, { src -> alt(src) }, { src -> break_alt(src) }, { src -> land_dir(src) }, { src -> flags(src) }

    )
}

fun fill(prally_point: RALLY_POINT) {
    prally_point.target_system(Byte)
    prally_point.target_component(Byte)
    prally_point.idx(Byte)
    prally_point.count(Byte)
    prally_point.lat(Int)
    prally_point.lng(Int)
    prally_point.alt(Short)
    prally_point.break_alt(Short)
    prally_point.land_dir(Short)

    prally_point.flags(some_RALLY_FLAGS)

}

fun RALLY_POINT.copyInto(DST: RALLY_POINT) {
    RALLY_POINT.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> idx() }, { -> count() }, { -> lat() }, { -> lng() }, { -> alt() }, { -> break_alt() }, { -> land_dir() }, { flags() != null }, { flags()!!.get() }

    )
}

fun onMISSION_SET_CURRENT(pmission_set_current: MISSION_SET_CURRENT) {
    val some_target_system = pmission_set_current.target_system()
    val some_target_component = pmission_set_current.target_component()
    val some_seq = pmission_set_current.seq()

}

fun test_.MISSION_SET_CURRENT.copyFrom(SRC: MISSION_SET_CURRENT) {
    MISSION_SET_CURRENT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> seq(src) }

    )
}

fun onADAP_TUNING(padap_tuning: ADAP_TUNING) {

    padap_tuning.axis()?.let { item ->
        some_PID_TUNING_AXIS = item.get()
    }
    val some_desired = padap_tuning.desired()
    val some_achieved = padap_tuning.achieved()
    val some_error = padap_tuning.error()
    val some_theta = padap_tuning.theta()
    val some_omega = padap_tuning.omega()
    val some_sigma = padap_tuning.sigma()
    val some_theta_dot = padap_tuning.theta_dot()
    val some_omega_dot = padap_tuning.omega_dot()
    val some_sigma_dot = padap_tuning.sigma_dot()
    val some_f = padap_tuning.f()
    val some_f_dot = padap_tuning.f_dot()
    val some_u = padap_tuning.u()

}

fun ADAP_TUNING.copyFrom(SRC: ADAP_TUNING) {
    ADAP_TUNING.push(
            SRC, { src -> axis(src) }, { src -> desired(src) }, { src -> achieved(src) }, { src -> error(src) }, { src -> theta(src) }, { src -> omega(src) }, { src -> sigma(src) }, { src -> theta_dot(src) }, { src -> omega_dot(src) }, { src -> sigma_dot(src) }, { src -> f(src) }, { src -> f_dot(src) }, { src -> u(src) }

    )
}

fun fill(padap_tuning: ADAP_TUNING) {

    padap_tuning.axis(some_PID_TUNING_AXIS)
    padap_tuning.desired(Float)
    padap_tuning.achieved(Float)
    padap_tuning.error(Float)
    padap_tuning.theta(Float)
    padap_tuning.omega(Float)
    padap_tuning.sigma(Float)
    padap_tuning.theta_dot(Float)
    padap_tuning.omega_dot(Float)
    padap_tuning.sigma_dot(Float)
    padap_tuning.f(Float)
    padap_tuning.f_dot(Float)
    padap_tuning.u(Float)

}

fun ADAP_TUNING.copyInto(DST: ADAP_TUNING) {
    ADAP_TUNING.pull(
            DST, { axis() != null }, { axis()!!.get() }, { -> desired() }, { -> achieved() }, { -> error() }, { -> theta() }, { -> omega() }, { -> sigma() }, { -> theta_dot() }, { -> omega_dot() }, { -> sigma_dot() }, { -> f() }, { -> f_dot() }, { -> u() }

    )
}

fun onVIBRATION(pvibration: VIBRATION) {
    val some_time_usec = pvibration.time_usec()
    val some_vibration_x = pvibration.vibration_x()
    val some_vibration_y = pvibration.vibration_y()
    val some_vibration_z = pvibration.vibration_z()
    val some_clipping_0 = pvibration.clipping_0()
    val some_clipping_1 = pvibration.clipping_1()
    val some_clipping_2 = pvibration.clipping_2()

}

fun VIBRATION.copyFrom(SRC: VIBRATION) {
    VIBRATION.push(
            SRC, { src -> time_usec(src) }, { src -> vibration_x(src) }, { src -> vibration_y(src) }, { src -> vibration_z(src) }, { src -> clipping_0(src) }, { src -> clipping_1(src) }, { src -> clipping_2(src) }

    )
}

fun fill(pvibration: VIBRATION) {
    pvibration.time_usec(Long)
    pvibration.vibration_x(Float)
    pvibration.vibration_y(Float)
    pvibration.vibration_z(Float)
    pvibration.clipping_0(Int)
    pvibration.clipping_1(Int)
    pvibration.clipping_2(Int)

}

fun VIBRATION.copyInto(DST: VIBRATION) {
    VIBRATION.pull(
            DST, { -> time_usec() }, { -> vibration_x() }, { -> vibration_y() }, { -> vibration_z() }, { -> clipping_0() }, { -> clipping_1() }, { -> clipping_2() }

    )
}

fun onPARAM_EXT_VALUE(pparam_ext_value: PARAM_EXT_VALUE) {

    pparam_ext_value.param_id()?.let { item -> char = item.get() }

    pparam_ext_value.param_value()?.let { item -> char = item.get() }

    pparam_ext_value.param_type()?.let { item ->
        some_MAV_PARAM_EXT_TYPE = item.get()
    }
    val some_param_count = pparam_ext_value.param_count()
    val some_param_index = pparam_ext_value.param_index()

}

fun PARAM_EXT_VALUE.copyFrom(SRC: PARAM_EXT_VALUE) {
    PARAM_EXT_VALUE.push(
            SRC, { src ->
        val item = param_id(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src ->
        val item = param_value(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> param_type(src) }, { src -> param_count(src) }, { src -> param_index(src) }

    )
}

fun fill(pparam_ext_value: PARAM_EXT_VALUE) {

    pparam_ext_value.param_id(some_String, null)

    pparam_ext_value.param_value(some_String, null)

    pparam_ext_value.param_type(some_MAV_PARAM_EXT_TYPE)
    pparam_ext_value.param_count(Short)
    pparam_ext_value.param_index(Short)

}

fun PARAM_EXT_VALUE.copyInto(DST: PARAM_EXT_VALUE) {
    PARAM_EXT_VALUE.pull(
            DST, { param_id()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = param_id()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { param_value()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = param_value()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { param_type() != null }, { param_type()!!.get() }, { -> param_count() }, { -> param_index() }

    )
}

fun onBATTERY2(pbattery2: BATTERY2) {
    val some_voltage = pbattery2.voltage()
    val some_current_battery = pbattery2.current_battery()

}

fun BATTERY2.copyFrom(SRC: BATTERY2) {
    BATTERY2.push(
            SRC, { src -> voltage(src) }, { src -> current_battery(src) }

    )
}

fun fill(pbattery2: BATTERY2) {
    pbattery2.voltage(Short)
    pbattery2.current_battery(Short)

}

fun BATTERY2.copyInto(DST: BATTERY2) {
    BATTERY2.pull(
            DST, { -> voltage() }, { -> current_battery() }

    )
}

fun onLIMITS_STATUS(plimits_status: LIMITS_STATUS) {

    plimits_status.limits_state()?.let { item ->
        some_LIMITS_STATE = item.get()
    }
    val some_last_trigger = plimits_status.last_trigger()
    val some_last_action = plimits_status.last_action()
    val some_last_recovery = plimits_status.last_recovery()
    val some_last_clear = plimits_status.last_clear()
    val some_breach_count = plimits_status.breach_count()

    plimits_status.mods_enabled()?.let { item ->
        some_LIMIT_MODULE = item.get()
    }

    plimits_status.mods_required()?.let { item ->
        some_LIMIT_MODULE = item.get()
    }

    plimits_status.mods_triggered()?.let { item ->
        some_LIMIT_MODULE = item.get()
    }

}

fun LIMITS_STATUS.copyFrom(SRC: LIMITS_STATUS) {
    LIMITS_STATUS.push(
            SRC, { src -> limits_state(src) }, { src -> last_trigger(src) }, { src -> last_action(src) }, { src -> last_recovery(src) }, { src -> last_clear(src) }, { src -> breach_count(src) }, { src -> mods_enabled(src) }, { src -> mods_required(src) }, { src -> mods_triggered(src) }

    )
}

fun fill(plimits_status: LIMITS_STATUS) {

    plimits_status.limits_state(some_LIMITS_STATE)
    plimits_status.last_trigger(Int)
    plimits_status.last_action(Int)
    plimits_status.last_recovery(Int)
    plimits_status.last_clear(Int)
    plimits_status.breach_count(Short)

    plimits_status.mods_enabled(some_LIMIT_MODULE)

    plimits_status.mods_required(some_LIMIT_MODULE)

    plimits_status.mods_triggered(some_LIMIT_MODULE)

}

fun LIMITS_STATUS.copyInto(DST: LIMITS_STATUS) {
    LIMITS_STATUS.pull(
            DST, { limits_state() != null }, { limits_state()!!.get() }, { -> last_trigger() }, { -> last_action() }, { -> last_recovery() }, { -> last_clear() }, { -> breach_count() }, { mods_enabled() != null }, { mods_enabled()!!.get() }, { mods_required() != null }, { mods_required()!!.get() }, { mods_triggered() != null }, { mods_triggered()!!.get() }

    )
}

fun onCAMERA_FEEDBACK(pcamera_feedback: CAMERA_FEEDBACK) {
    val some_time_usec = pcamera_feedback.time_usec()
    val some_target_system = pcamera_feedback.target_system()
    val some_cam_idx = pcamera_feedback.cam_idx()
    val some_img_idx = pcamera_feedback.img_idx()
    val some_lat = pcamera_feedback.lat()
    val some_lng = pcamera_feedback.lng()
    val some_alt_msl = pcamera_feedback.alt_msl()
    val some_alt_rel = pcamera_feedback.alt_rel()
    val some_roll = pcamera_feedback.roll()
    val some_pitch = pcamera_feedback.pitch()
    val some_yaw = pcamera_feedback.yaw()
    val some_foc_len = pcamera_feedback.foc_len()

    pcamera_feedback.flags()?.let { item ->
        some_CAMERA_FEEDBACK_FLAGS = item.get()
    }

}

fun CAMERA_FEEDBACK.copyFrom(SRC: CAMERA_FEEDBACK) {
    CAMERA_FEEDBACK.push(
            SRC, { src -> time_usec(src) }, { src -> target_system(src) }, { src -> cam_idx(src) }, { src -> img_idx(src) }, { src -> lat(src) }, { src -> lng(src) }, { src -> alt_msl(src) }, { src -> alt_rel(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }, { src -> foc_len(src) }, { src -> flags(src) }

    )
}

fun fill(pcamera_feedback: CAMERA_FEEDBACK) {
    pcamera_feedback.time_usec(Long)
    pcamera_feedback.target_system(Byte)
    pcamera_feedback.cam_idx(Byte)
    pcamera_feedback.img_idx(Short)
    pcamera_feedback.lat(Int)
    pcamera_feedback.lng(Int)
    pcamera_feedback.alt_msl(Float)
    pcamera_feedback.alt_rel(Float)
    pcamera_feedback.roll(Float)
    pcamera_feedback.pitch(Float)
    pcamera_feedback.yaw(Float)
    pcamera_feedback.foc_len(Float)

    pcamera_feedback.flags(some_CAMERA_FEEDBACK_FLAGS)

}

fun CAMERA_FEEDBACK.copyInto(DST: CAMERA_FEEDBACK) {
    CAMERA_FEEDBACK.pull(
            DST, { -> time_usec() }, { -> target_system() }, { -> cam_idx() }, { -> img_idx() }, { -> lat() }, { -> lng() }, { -> alt_msl() }, { -> alt_rel() }, { -> roll() }, { -> pitch() }, { -> yaw() }, { -> foc_len() }, { flags() != null }, { flags()!!.get() }

    )
}

fun onHIL_GPS(phil_gps: HIL_GPS) {
    val some_time_usec = phil_gps.time_usec()
    val some_fix_type = phil_gps.fix_type()
    val some_lat = phil_gps.lat()
    val some_lon = phil_gps.lon()
    val some_alt = phil_gps.alt()
    val some_eph = phil_gps.eph()
    val some_epv = phil_gps.epv()
    val some_vel = phil_gps.vel()
    val some_vn = phil_gps.vn()
    val some_ve = phil_gps.ve()
    val some_vd = phil_gps.vd()
    val some_cog = phil_gps.cog()
    val some_satellites_visible = phil_gps.satellites_visible()

}

fun HIL_GPS.copyFrom(SRC: HIL_GPS) {
    HIL_GPS.push(
            SRC, { src -> time_usec(src) }, { src -> fix_type(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> eph(src) }, { src -> epv(src) }, { src -> vel(src) }, { src -> vn(src) }, { src -> ve(src) }, { src -> vd(src) }, { src -> cog(src) }, { src -> satellites_visible(src) }

    )
}

fun fill(phil_gps: HIL_GPS) {
    phil_gps.time_usec(Long)
    phil_gps.fix_type(Byte)
    phil_gps.lat(Int)
    phil_gps.lon(Int)
    phil_gps.alt(Int)
    phil_gps.eph(Short)
    phil_gps.epv(Short)
    phil_gps.vel(Short)
    phil_gps.vn(Short)
    phil_gps.ve(Short)
    phil_gps.vd(Short)
    phil_gps.cog(Short)
    phil_gps.satellites_visible(Byte)

}

fun HIL_GPS.copyInto(DST: HIL_GPS) {
    HIL_GPS.pull(
            DST, { -> time_usec() }, { -> fix_type() }, { -> lat() }, { -> lon() }, { -> alt() }, { -> eph() }, { -> epv() }, { -> vel() }, { -> vn() }, { -> ve() }, { -> vd() }, { -> cog() }, { -> satellites_visible() }

    )
}

fun onNAV_CONTROLLER_OUTPUT(pnav_controller_output: NAV_CONTROLLER_OUTPUT) {
    val some_nav_roll = pnav_controller_output.nav_roll()
    val some_nav_pitch = pnav_controller_output.nav_pitch()
    val some_nav_bearing = pnav_controller_output.nav_bearing()
    val some_target_bearing = pnav_controller_output.target_bearing()
    val some_wp_dist = pnav_controller_output.wp_dist()
    val some_alt_error = pnav_controller_output.alt_error()
    val some_aspd_error = pnav_controller_output.aspd_error()
    val some_xtrack_error = pnav_controller_output.xtrack_error()

}

fun test_.NAV_CONTROLLER_OUTPUT.copyFrom(SRC: NAV_CONTROLLER_OUTPUT) {
    NAV_CONTROLLER_OUTPUT.push(
            SRC, { src -> nav_roll(src) }, { src -> nav_pitch(src) }, { src -> nav_bearing(src) }, { src -> target_bearing(src) }, { src -> wp_dist(src) }, { src -> alt_error(src) }, { src -> aspd_error(src) }, { src -> xtrack_error(src) }

    )
}

fun onAUTH_KEY(pauth_key: AUTH_KEY) {

    pauth_key.key()?.let { item -> char = item.get() }

}

fun test_.AUTH_KEY.copyFrom(SRC: AUTH_KEY) {
    AUTH_KEY.push(
            SRC, { src ->
        val item = key(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun onFENCE_FETCH_POINT(pfence_fetch_point: FENCE_FETCH_POINT) {
    val some_target_system = pfence_fetch_point.target_system()
    val some_target_component = pfence_fetch_point.target_component()
    val some_idx = pfence_fetch_point.idx()

}

fun FENCE_FETCH_POINT.copyFrom(SRC: FENCE_FETCH_POINT) {
    FENCE_FETCH_POINT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> idx(src) }

    )
}

fun fill(pfence_fetch_point: FENCE_FETCH_POINT) {
    pfence_fetch_point.target_system(Byte)
    pfence_fetch_point.target_component(Byte)
    pfence_fetch_point.idx(Byte)

}

fun FENCE_FETCH_POINT.copyInto(DST: FENCE_FETCH_POINT) {
    FENCE_FETCH_POINT.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> idx() }

    )
}

fun onRADIO(pradio: RADIO) {
    val some_rssi = pradio.rssi()
    val some_remrssi = pradio.remrssi()
    val some_txbuf = pradio.txbuf()
    val some_noise = pradio.noise()
    val some_remnoise = pradio.remnoise()
    val some_rxerrors = pradio.rxerrors()
    val some_fixeD = pradio.fixeD()

}

fun RADIO.copyFrom(SRC: RADIO) {
    RADIO.push(
            SRC, { src -> rssi(src) }, { src -> remrssi(src) }, { src -> txbuf(src) }, { src -> noise(src) }, { src -> remnoise(src) }, { src -> rxerrors(src) }, { src -> fixeD(src) }

    )
}

fun fill(pradio: RADIO) {
    pradio.rssi(Byte)
    pradio.remrssi(Byte)
    pradio.txbuf(Byte)
    pradio.noise(Byte)
    pradio.remnoise(Byte)
    pradio.rxerrors(Short)
    pradio.fixeD(Short)

}

fun RADIO.copyInto(DST: RADIO) {
    RADIO.pull(
            DST, { -> rssi() }, { -> remrssi() }, { -> txbuf() }, { -> noise() }, { -> remnoise() }, { -> rxerrors() }, { -> fixeD() }

    )
}

fun onLOCAL_POSITION_NED_COV(plocal_position_ned_cov: LOCAL_POSITION_NED_COV) {
    val some_time_usec = plocal_position_ned_cov.time_usec()

    plocal_position_ned_cov.estimator_type()?.let { item ->
        some_MAV_ESTIMATOR_TYPE = item.get()
    }
    val some_x = plocal_position_ned_cov.x()
    val some_y = plocal_position_ned_cov.y()
    val some_z = plocal_position_ned_cov.z()
    val some_vx = plocal_position_ned_cov.vx()
    val some_vy = plocal_position_ned_cov.vy()
    val some_vz = plocal_position_ned_cov.vz()
    val some_ax = plocal_position_ned_cov.ax()
    val some_ay = plocal_position_ned_cov.ay()
    val some_az = plocal_position_ned_cov.az()
    plocal_position_ned_cov.covariance().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }

}

fun test_.LOCAL_POSITION_NED_COV.copyFrom(SRC: LOCAL_POSITION_NED_COV) {
    LOCAL_POSITION_NED_COV.push(
            SRC, { src -> time_usec(src) }, { src -> estimator_type(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> ax(src) }, { src -> ay(src) }, { src -> az(src) }, { src ->
        val item = covariance()
        for (i in 0 until 45)
            item.set(src.get(i), i)
    }

    )
}

fun onAIRSPEED_AUTOCAL(pairspeed_autocal: AIRSPEED_AUTOCAL) {
    val some_vx = pairspeed_autocal.vx()
    val some_vy = pairspeed_autocal.vy()
    val some_vz = pairspeed_autocal.vz()
    val some_diff_pressure = pairspeed_autocal.diff_pressure()
    val some_EAS2TAS = pairspeed_autocal.EAS2TAS()
    val some_ratio = pairspeed_autocal.ratio()
    val some_state_x = pairspeed_autocal.state_x()
    val some_state_y = pairspeed_autocal.state_y()
    val some_state_z = pairspeed_autocal.state_z()
    val some_Pax = pairspeed_autocal.Pax()
    val some_Pby = pairspeed_autocal.Pby()
    val some_Pcz = pairspeed_autocal.Pcz()

}

fun AIRSPEED_AUTOCAL.copyFrom(SRC: AIRSPEED_AUTOCAL) {
    AIRSPEED_AUTOCAL.push(
            SRC, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> diff_pressure(src) }, { src -> EAS2TAS(src) }, { src -> ratio(src) }, { src -> state_x(src) }, { src -> state_y(src) }, { src -> state_z(src) }, { src -> Pax(src) }, { src -> Pby(src) }, { src -> Pcz(src) }

    )
}

fun fill(pairspeed_autocal: AIRSPEED_AUTOCAL) {
    pairspeed_autocal.vx(Float)
    pairspeed_autocal.vy(Float)
    pairspeed_autocal.vz(Float)
    pairspeed_autocal.diff_pressure(Float)
    pairspeed_autocal.EAS2TAS(Float)
    pairspeed_autocal.ratio(Float)
    pairspeed_autocal.state_x(Float)
    pairspeed_autocal.state_y(Float)
    pairspeed_autocal.state_z(Float)
    pairspeed_autocal.Pax(Float)
    pairspeed_autocal.Pby(Float)
    pairspeed_autocal.Pcz(Float)

}

fun AIRSPEED_AUTOCAL.copyInto(DST: AIRSPEED_AUTOCAL) {
    AIRSPEED_AUTOCAL.pull(
            DST, { -> vx() }, { -> vy() }, { -> vz() }, { -> diff_pressure() }, { -> EAS2TAS() }, { -> ratio() }, { -> state_x() }, { -> state_y() }, { -> state_z() }, { -> Pax() }, { -> Pby() }, { -> Pcz() }

    )
}

fun onATT_POS_MOCAP(patt_pos_mocap: ATT_POS_MOCAP) {
    val some_time_usec = patt_pos_mocap.time_usec()
    patt_pos_mocap.q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_x = patt_pos_mocap.x()
    val some_y = patt_pos_mocap.y()
    val some_z = patt_pos_mocap.z()

}

fun ATT_POS_MOCAP.copyFrom(SRC: ATT_POS_MOCAP) {
    ATT_POS_MOCAP.push(
            SRC, { src -> time_usec(src) }, { src ->
        val item = q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }

    )
}

fun fill(patt_pos_mocap: ATT_POS_MOCAP) {
    patt_pos_mocap.time_usec(Long)
    patt_pos_mocap.q().let { item ->
        patt_pos_mocap.q().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    patt_pos_mocap.x(Float)
    patt_pos_mocap.y(Float)
    patt_pos_mocap.z(Float)

}

fun ATT_POS_MOCAP.copyInto(DST: ATT_POS_MOCAP) {
    ATT_POS_MOCAP.pull(
            DST, { -> time_usec() }, { dst ->
        var src = this.q()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }, { -> x() }, { -> y() }, { -> z() }

    )
}

fun onSTATUSTEXT(pstatustext: STATUSTEXT) {

    pstatustext.severity()?.let { item ->
        some_MAV_SEVERITY = item.get()
    }

    pstatustext.text()?.let { item -> char = item.get() }

}

fun STATUSTEXT.copyFrom(SRC: STATUSTEXT) {
    STATUSTEXT.push(
            SRC, { src -> severity(src) }, { src ->
        val item = text(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun fill(pstatustext: STATUSTEXT) {

    pstatustext.severity(some_MAV_SEVERITY)

    pstatustext.text(some_String, null)

}

fun STATUSTEXT.copyInto(DST: STATUSTEXT) {
    STATUSTEXT.pull(
            DST, { severity() != null }, { severity()!!.get() }, { text()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = text()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }

    )
}

fun onPING(pping: PING) {
    val some_time_usec = pping.time_usec()
    val some_seq = pping.seq()
    val some_target_system = pping.target_system()
    val some_target_component = pping.target_component()

}

fun test_.PING.copyFrom(SRC: PING) {
    PING.push(
            SRC, { src -> time_usec(src) }, { src -> seq(src) }, { src -> target_system(src) }, { src -> target_component(src) }

    )
}

fun onGOPRO_GET_REQUEST(pgopro_get_request: GOPRO_GET_REQUEST) {
    val some_target_system = pgopro_get_request.target_system()
    val some_target_component = pgopro_get_request.target_component()

    pgopro_get_request.cmd_id()?.let { item ->
        some_GOPRO_COMMAND = item.get()
    }

}

fun GOPRO_GET_REQUEST.copyFrom(SRC: GOPRO_GET_REQUEST) {
    GOPRO_GET_REQUEST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> cmd_id(src) }

    )
}

fun fill(pgopro_get_request: GOPRO_GET_REQUEST) {
    pgopro_get_request.target_system(Byte)
    pgopro_get_request.target_component(Byte)

    pgopro_get_request.cmd_id(some_GOPRO_COMMAND)

}

fun GOPRO_GET_REQUEST.copyInto(DST: GOPRO_GET_REQUEST) {
    GOPRO_GET_REQUEST.pull(
            DST, { -> target_system() }, { -> target_component() }, { cmd_id() != null }, { cmd_id()!!.get() }

    )
}

fun onCAMERA_CAPTURE_STATUS(pcamera_capture_status: CAMERA_CAPTURE_STATUS) {
    val some_time_boot_ms = pcamera_capture_status.time_boot_ms()
    val some_image_status = pcamera_capture_status.image_status()
    val some_video_status = pcamera_capture_status.video_status()
    val some_image_interval = pcamera_capture_status.image_interval()
    val some_recording_time_ms = pcamera_capture_status.recording_time_ms()
    val some_available_capacity = pcamera_capture_status.available_capacity()

}

fun CAMERA_CAPTURE_STATUS.copyFrom(SRC: CAMERA_CAPTURE_STATUS) {
    CAMERA_CAPTURE_STATUS.push(
            SRC, { src -> time_boot_ms(src) }, { src -> image_status(src) }, { src -> video_status(src) }, { src -> image_interval(src) }, { src -> recording_time_ms(src) }, { src -> available_capacity(src) }

    )
}

fun fill(pcamera_capture_status: CAMERA_CAPTURE_STATUS) {
    pcamera_capture_status.time_boot_ms(Int)
    pcamera_capture_status.image_status(Byte)
    pcamera_capture_status.video_status(Byte)
    pcamera_capture_status.image_interval(Float)
    pcamera_capture_status.recording_time_ms(Int)
    pcamera_capture_status.available_capacity(Float)

}

fun CAMERA_CAPTURE_STATUS.copyInto(DST: CAMERA_CAPTURE_STATUS) {
    CAMERA_CAPTURE_STATUS.pull(
            DST, { -> time_boot_ms() }, { -> image_status() }, { -> video_status() }, { -> image_interval() }, { -> recording_time_ms() }, { -> available_capacity() }

    )
}

fun onGLOBAL_POSITION_INT(pglobal_position_int: GLOBAL_POSITION_INT) {
    val some_time_boot_ms = pglobal_position_int.time_boot_ms()
    val some_lat = pglobal_position_int.lat()
    val some_lon = pglobal_position_int.lon()
    val some_alt = pglobal_position_int.alt()
    val some_relative_alt = pglobal_position_int.relative_alt()
    val some_vx = pglobal_position_int.vx()
    val some_vy = pglobal_position_int.vy()
    val some_vz = pglobal_position_int.vz()
    val some_hdg = pglobal_position_int.hdg()

}

fun test_.GLOBAL_POSITION_INT.copyFrom(SRC: GLOBAL_POSITION_INT) {
    GLOBAL_POSITION_INT.push(
            SRC, { src -> time_boot_ms(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> relative_alt(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> hdg(src) }

    )
}

fun onENCAPSULATED_DATA(pencapsulated_data: ENCAPSULATED_DATA) {
    val some_seqnr = pencapsulated_data.seqnr()
    pencapsulated_data.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun ENCAPSULATED_DATA.copyFrom(SRC: ENCAPSULATED_DATA) {
    ENCAPSULATED_DATA.push(
            SRC, { src -> seqnr(src) }, { src ->
        val item = daTa()
        for (i in 0 until 253)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pencapsulated_data: ENCAPSULATED_DATA) {
    pencapsulated_data.seqnr(Short)
    pencapsulated_data.daTa().let { item ->
        pencapsulated_data.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun ENCAPSULATED_DATA.copyInto(DST: ENCAPSULATED_DATA) {
    ENCAPSULATED_DATA.pull(
            DST, { -> seqnr() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 253)
            dst.set(src.get(i), i)
    }

    )
}

fun onGPS_INPUT(pgps_input: GPS_INPUT) {
    val some_time_usec = pgps_input.time_usec()
    val some_gps_id = pgps_input.gps_id()

    pgps_input.ignore_flags()?.let { item ->
        some_GPS_INPUT_IGNORE_FLAGS = item.get()
    }
    val some_time_week_ms = pgps_input.time_week_ms()
    val some_time_week = pgps_input.time_week()
    val some_fix_type = pgps_input.fix_type()
    val some_lat = pgps_input.lat()
    val some_lon = pgps_input.lon()
    val some_alt = pgps_input.alt()
    val some_hdop = pgps_input.hdop()
    val some_vdop = pgps_input.vdop()
    val some_vn = pgps_input.vn()
    val some_ve = pgps_input.ve()
    val some_vd = pgps_input.vd()
    val some_speed_accuracy = pgps_input.speed_accuracy()
    val some_horiz_accuracy = pgps_input.horiz_accuracy()
    val some_vert_accuracy = pgps_input.vert_accuracy()
    val some_satellites_visible = pgps_input.satellites_visible()

}

fun GPS_INPUT.copyFrom(SRC: GPS_INPUT) {
    GPS_INPUT.push(
            SRC, { src -> time_usec(src) }, { src -> gps_id(src) }, { src -> ignore_flags(src) }, { src -> time_week_ms(src) }, { src -> time_week(src) }, { src -> fix_type(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> hdop(src) }, { src -> vdop(src) }, { src -> vn(src) }, { src -> ve(src) }, { src -> vd(src) }, { src -> speed_accuracy(src) }, { src -> horiz_accuracy(src) }, { src -> vert_accuracy(src) }, { src -> satellites_visible(src) }

    )
}

fun fill(pgps_input: GPS_INPUT) {
    pgps_input.time_usec(Long)
    pgps_input.gps_id(Byte)

    pgps_input.ignore_flags(some_GPS_INPUT_IGNORE_FLAGS)
    pgps_input.time_week_ms(Int)
    pgps_input.time_week(Short)
    pgps_input.fix_type(Byte)
    pgps_input.lat(Int)
    pgps_input.lon(Int)
    pgps_input.alt(Float)
    pgps_input.hdop(Float)
    pgps_input.vdop(Float)
    pgps_input.vn(Float)
    pgps_input.ve(Float)
    pgps_input.vd(Float)
    pgps_input.speed_accuracy(Float)
    pgps_input.horiz_accuracy(Float)
    pgps_input.vert_accuracy(Float)
    pgps_input.satellites_visible(Byte)

}

fun GPS_INPUT.copyInto(DST: GPS_INPUT) {
    GPS_INPUT.pull(
            DST, { -> time_usec() }, { -> gps_id() }, { ignore_flags() != null }, { ignore_flags()!!.get() }, { -> time_week_ms() }, { -> time_week() }, { -> fix_type() }, { -> lat() }, { -> lon() }, { -> alt() }, { -> hdop() }, { -> vdop() }, { -> vn() }, { -> ve() }, { -> vd() }, { -> speed_accuracy() }, { -> horiz_accuracy() }, { -> vert_accuracy() }, { -> satellites_visible() }

    )
}

fun onCOMMAND_LONG(pcommand_long: COMMAND_LONG) {
    val some_target_system = pcommand_long.target_system()
    val some_target_component = pcommand_long.target_component()

    pcommand_long.command()?.let { item ->
        some_MAV_CMD = item.get()
    }
    val some_confirmation = pcommand_long.confirmation()
    val some_param1 = pcommand_long.param1()
    val some_param2 = pcommand_long.param2()
    val some_param3 = pcommand_long.param3()
    val some_param4 = pcommand_long.param4()
    val some_param5 = pcommand_long.param5()
    val some_param6 = pcommand_long.param6()
    val some_param7 = pcommand_long.param7()

}

fun test_.COMMAND_LONG.copyFrom(SRC: COMMAND_LONG) {
    COMMAND_LONG.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> command(src) }, { src -> confirmation(src) }, { src -> param1(src) }, { src -> param2(src) }, { src -> param3(src) }, { src -> param4(src) }, { src -> param5(src) }, { src -> param6(src) }, { src -> param7(src) }

    )
}

fun onCOMPASSMOT_STATUS(pcompassmot_status: COMPASSMOT_STATUS) {
    val some_throttle = pcompassmot_status.throttle()
    val some_current = pcompassmot_status.current()
    val some_interference = pcompassmot_status.interference()
    val some_CompensationX = pcompassmot_status.CompensationX()
    val some_CompensationY = pcompassmot_status.CompensationY()
    val some_CompensationZ = pcompassmot_status.CompensationZ()

}

fun COMPASSMOT_STATUS.copyFrom(SRC: COMPASSMOT_STATUS) {
    COMPASSMOT_STATUS.push(
            SRC, { src -> throttle(src) }, { src -> current(src) }, { src -> interference(src) }, { src -> CompensationX(src) }, { src -> CompensationY(src) }, { src -> CompensationZ(src) }

    )
}

fun fill(pcompassmot_status: COMPASSMOT_STATUS) {
    pcompassmot_status.throttle(Short)
    pcompassmot_status.current(Float)
    pcompassmot_status.interference(Short)
    pcompassmot_status.CompensationX(Float)
    pcompassmot_status.CompensationY(Float)
    pcompassmot_status.CompensationZ(Float)

}

fun COMPASSMOT_STATUS.copyInto(DST: COMPASSMOT_STATUS) {
    COMPASSMOT_STATUS.pull(
            DST, { -> throttle() }, { -> current() }, { -> interference() }, { -> CompensationX() }, { -> CompensationY() }, { -> CompensationZ() }

    )
}

fun onLOG_REQUEST_DATA(plog_request_data: LOG_REQUEST_DATA) {
    val some_target_system = plog_request_data.target_system()
    val some_target_component = plog_request_data.target_component()
    val some_id = plog_request_data.id()
    val some_ofs = plog_request_data.ofs()
    val some_count = plog_request_data.count()

}

fun LOG_REQUEST_DATA.copyFrom(SRC: LOG_REQUEST_DATA) {
    LOG_REQUEST_DATA.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> id(src) }, { src -> ofs(src) }, { src -> count(src) }

    )
}

fun fill(plog_request_data: LOG_REQUEST_DATA) {
    plog_request_data.target_system(Byte)
    plog_request_data.target_component(Byte)
    plog_request_data.id(Short)
    plog_request_data.ofs(Int)
    plog_request_data.count(Int)

}

fun LOG_REQUEST_DATA.copyInto(DST: LOG_REQUEST_DATA) {
    LOG_REQUEST_DATA.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> id() }, { -> ofs() }, { -> count() }

    )
}

fun onGPS_RAW_INT(pgps_raw_int: GPS_RAW_INT) {
    val some_time_usec = pgps_raw_int.time_usec()

    pgps_raw_int.fix_type()?.let { item ->
        some_GPS_FIX_TYPE = item.get()
    }
    val some_lat = pgps_raw_int.lat()
    val some_lon = pgps_raw_int.lon()
    val some_alt = pgps_raw_int.alt()
    val some_eph = pgps_raw_int.eph()
    val some_epv = pgps_raw_int.epv()
    val some_vel = pgps_raw_int.vel()
    val some_cog = pgps_raw_int.cog()
    val some_satellites_visible = pgps_raw_int.satellites_visible()

    pgps_raw_int.alt_ellipsoid().let { item -> println("Receive Int pack.") }

    pgps_raw_int.h_acc().let { item -> println("Receive Int pack.") }

    pgps_raw_int.v_acc().let { item -> println("Receive Int pack.") }

    pgps_raw_int.vel_acc().let { item -> println("Receive Int pack.") }

    pgps_raw_int.hdg_acc().let { item -> println("Receive Int pack.") }

}

fun test_.GPS_RAW_INT.copyFrom(SRC: GPS_RAW_INT) {
    GPS_RAW_INT.push(
            SRC, { src -> time_usec(src) }, { src -> fix_type(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> eph(src) }, { src -> epv(src) }, { src -> vel(src) }, { src -> cog(src) }, { src -> satellites_visible(src) }, { src -> alt_ellipsoid(src) }, { src -> h_acc(src) }, { src -> v_acc(src) }, { src -> vel_acc(src) }, { src -> hdg_acc(src) }

    )
}

fun onCAMERA_STATUS(pcamera_status: CAMERA_STATUS) {
    val some_time_usec = pcamera_status.time_usec()
    val some_target_system = pcamera_status.target_system()
    val some_cam_idx = pcamera_status.cam_idx()
    val some_img_idx = pcamera_status.img_idx()

    pcamera_status.event_id()?.let { item ->
        some_CAMERA_STATUS_TYPES = item.get()
    }
    val some_p1 = pcamera_status.p1()
    val some_p2 = pcamera_status.p2()
    val some_p3 = pcamera_status.p3()
    val some_p4 = pcamera_status.p4()

}

fun CAMERA_STATUS.copyFrom(SRC: CAMERA_STATUS) {
    CAMERA_STATUS.push(
            SRC, { src -> time_usec(src) }, { src -> target_system(src) }, { src -> cam_idx(src) }, { src -> img_idx(src) }, { src -> event_id(src) }, { src -> p1(src) }, { src -> p2(src) }, { src -> p3(src) }, { src -> p4(src) }

    )
}

fun fill(pcamera_status: CAMERA_STATUS) {
    pcamera_status.time_usec(Long)
    pcamera_status.target_system(Byte)
    pcamera_status.cam_idx(Byte)
    pcamera_status.img_idx(Short)

    pcamera_status.event_id(some_CAMERA_STATUS_TYPES)
    pcamera_status.p1(Float)
    pcamera_status.p2(Float)
    pcamera_status.p3(Float)
    pcamera_status.p4(Float)

}

fun CAMERA_STATUS.copyInto(DST: CAMERA_STATUS) {
    CAMERA_STATUS.pull(
            DST, { -> time_usec() }, { -> target_system() }, { -> cam_idx() }, { -> img_idx() }, { event_id() != null }, { event_id()!!.get() }, { -> p1() }, { -> p2() }, { -> p3() }, { -> p4() }

    )
}

fun onRC_CHANNELS_SCALED(prc_channels_scaled: RC_CHANNELS_SCALED) {
    val some_time_boot_ms = prc_channels_scaled.time_boot_ms()
    val some_port = prc_channels_scaled.port()
    val some_chan1_scaled = prc_channels_scaled.chan1_scaled()
    val some_chan2_scaled = prc_channels_scaled.chan2_scaled()
    val some_chan3_scaled = prc_channels_scaled.chan3_scaled()
    val some_chan4_scaled = prc_channels_scaled.chan4_scaled()
    val some_chan5_scaled = prc_channels_scaled.chan5_scaled()
    val some_chan6_scaled = prc_channels_scaled.chan6_scaled()
    val some_chan7_scaled = prc_channels_scaled.chan7_scaled()
    val some_chan8_scaled = prc_channels_scaled.chan8_scaled()
    val some_rssi = prc_channels_scaled.rssi()

}

fun test_.RC_CHANNELS_SCALED.copyFrom(SRC: RC_CHANNELS_SCALED) {
    RC_CHANNELS_SCALED.push(
            SRC, { src -> time_boot_ms(src) }, { src -> port(src) }, { src -> chan1_scaled(src) }, { src -> chan2_scaled(src) }, { src -> chan3_scaled(src) }, { src -> chan4_scaled(src) }, { src -> chan5_scaled(src) }, { src -> chan6_scaled(src) }, { src -> chan7_scaled(src) }, { src -> chan8_scaled(src) }, { src -> rssi(src) }

    )
}

fun onCAMERA_SETTINGS(pcamera_settings: CAMERA_SETTINGS) {
    val some_time_boot_ms = pcamera_settings.time_boot_ms()

    pcamera_settings.mode_id()?.let { item ->
        some_CAMERA_MODE = item.get()
    }

}

fun CAMERA_SETTINGS.copyFrom(SRC: CAMERA_SETTINGS) {
    CAMERA_SETTINGS.push(
            SRC, { src -> time_boot_ms(src) }, { src -> mode_id(src) }

    )
}

fun fill(pcamera_settings: CAMERA_SETTINGS) {
    pcamera_settings.time_boot_ms(Int)

    pcamera_settings.mode_id(some_CAMERA_MODE)

}

fun CAMERA_SETTINGS.copyInto(DST: CAMERA_SETTINGS) {
    CAMERA_SETTINGS.pull(
            DST, { -> time_boot_ms() }, { mode_id() != null }, { mode_id()!!.get() }

    )
}

fun onDEVICE_OP_READ_REPLY(pdevice_op_read_reply: DEVICE_OP_READ_REPLY) {
    val some_request_id = pdevice_op_read_reply.request_id()
    val some_result = pdevice_op_read_reply.result()
    val some_regstart = pdevice_op_read_reply.regstart()
    val some_count = pdevice_op_read_reply.count()
    pdevice_op_read_reply.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun DEVICE_OP_READ_REPLY.copyFrom(SRC: DEVICE_OP_READ_REPLY) {
    DEVICE_OP_READ_REPLY.push(
            SRC, { src -> request_id(src) }, { src -> result(src) }, { src -> regstart(src) }, { src -> count(src) }, { src ->
        val item = daTa()
        for (i in 0 until 128)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pdevice_op_read_reply: DEVICE_OP_READ_REPLY) {
    pdevice_op_read_reply.request_id(Int)
    pdevice_op_read_reply.result(Byte)
    pdevice_op_read_reply.regstart(Byte)
    pdevice_op_read_reply.count(Byte)
    pdevice_op_read_reply.daTa().let { item ->
        pdevice_op_read_reply.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun DEVICE_OP_READ_REPLY.copyInto(DST: DEVICE_OP_READ_REPLY) {
    DEVICE_OP_READ_REPLY.pull(
            DST, { -> request_id() }, { -> result() }, { -> regstart() }, { -> count() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 128)
            dst.set(src.get(i), i)
    }

    )
}

fun onRAW_PRESSURE(praw_pressure: RAW_PRESSURE) {
    val some_time_usec = praw_pressure.time_usec()
    val some_press_abs = praw_pressure.press_abs()
    val some_press_diff1 = praw_pressure.press_diff1()
    val some_press_diff2 = praw_pressure.press_diff2()
    val some_temperature = praw_pressure.temperature()

}

fun test_.RAW_PRESSURE.copyFrom(SRC: RAW_PRESSURE) {
    RAW_PRESSURE.push(
            SRC, { src -> time_usec(src) }, { src -> press_abs(src) }, { src -> press_diff1(src) }, { src -> press_diff2(src) }, { src -> temperature(src) }

    )
}

fun onDIGICAM_CONTROL(pdigicam_control: DIGICAM_CONTROL) {
    val some_target_system = pdigicam_control.target_system()
    val some_target_component = pdigicam_control.target_component()
    val some_session = pdigicam_control.session()
    val some_zoom_pos = pdigicam_control.zoom_pos()
    val some_zoom_step = pdigicam_control.zoom_step()
    val some_focus_lock = pdigicam_control.focus_lock()
    val some_shot = pdigicam_control.shot()
    val some_command_id = pdigicam_control.command_id()
    val some_extra_param = pdigicam_control.extra_param()
    val some_extra_value = pdigicam_control.extra_value()

}

fun DIGICAM_CONTROL.copyFrom(SRC: DIGICAM_CONTROL) {
    DIGICAM_CONTROL.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> session(src) }, { src -> zoom_pos(src) }, { src -> zoom_step(src) }, { src -> focus_lock(src) }, { src -> shot(src) }, { src -> command_id(src) }, { src -> extra_param(src) }, { src -> extra_value(src) }

    )
}

fun fill(pdigicam_control: DIGICAM_CONTROL) {
    pdigicam_control.target_system(Byte)
    pdigicam_control.target_component(Byte)
    pdigicam_control.session(Byte)
    pdigicam_control.zoom_pos(Byte)
    pdigicam_control.zoom_step(Byte)
    pdigicam_control.focus_lock(Byte)
    pdigicam_control.shot(Byte)
    pdigicam_control.command_id(Byte)
    pdigicam_control.extra_param(Byte)
    pdigicam_control.extra_value(Float)

}

fun DIGICAM_CONTROL.copyInto(DST: DIGICAM_CONTROL) {
    DIGICAM_CONTROL.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> session() }, { -> zoom_pos() }, { -> zoom_step() }, { -> focus_lock() }, { -> shot() }, { -> command_id() }, { -> extra_param() }, { -> extra_value() }

    )
}

fun onNAMED_VALUE_FLOAT(pnamed_value_float: NAMED_VALUE_FLOAT) {
    val some_time_boot_ms = pnamed_value_float.time_boot_ms()

    pnamed_value_float.name()?.let { item -> char = item.get() }
    val some_value = pnamed_value_float.value()

}

fun NAMED_VALUE_FLOAT.copyFrom(SRC: NAMED_VALUE_FLOAT) {
    NAMED_VALUE_FLOAT.push(
            SRC, { src -> time_boot_ms(src) }, { src ->
        val item = name(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> value(src) }

    )
}

fun fill(pnamed_value_float: NAMED_VALUE_FLOAT) {
    pnamed_value_float.time_boot_ms(Int)

    pnamed_value_float.name(some_String, null)
    pnamed_value_float.value(Float)

}

fun NAMED_VALUE_FLOAT.copyInto(DST: NAMED_VALUE_FLOAT) {
    NAMED_VALUE_FLOAT.pull(
            DST, { -> time_boot_ms() }, { name()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = name()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { -> value() }

    )
}

fun onGOPRO_HEARTBEAT(pgopro_heartbeat: GOPRO_HEARTBEAT) {

    pgopro_heartbeat.status()?.let { item ->
        some_GOPRO_HEARTBEAT_STATUS = item.get()
    }

    pgopro_heartbeat.capture_mode()?.let { item ->
        some_GOPRO_CAPTURE_MODE = item.get()
    }

    pgopro_heartbeat.flags()?.let { item ->
        some_GOPRO_HEARTBEAT_FLAGS = item.get()
    }

}

fun GOPRO_HEARTBEAT.copyFrom(SRC: GOPRO_HEARTBEAT) {
    GOPRO_HEARTBEAT.push(
            SRC, { src -> status(src) }, { src -> capture_mode(src) }, { src -> flags(src) }

    )
}

fun fill(pgopro_heartbeat: GOPRO_HEARTBEAT) {

    pgopro_heartbeat.status(some_GOPRO_HEARTBEAT_STATUS)

    pgopro_heartbeat.capture_mode(some_GOPRO_CAPTURE_MODE)

    pgopro_heartbeat.flags(some_GOPRO_HEARTBEAT_FLAGS)

}

fun GOPRO_HEARTBEAT.copyInto(DST: GOPRO_HEARTBEAT) {
    GOPRO_HEARTBEAT.pull(
            DST, { status() != null }, { status()!!.get() }, { capture_mode() != null }, { capture_mode()!!.get() }, { flags() != null }, { flags()!!.get() }

    )
}

fun onATTITUDE(pattitude: ATTITUDE) {
    val some_time_boot_ms = pattitude.time_boot_ms()
    val some_roll = pattitude.roll()
    val some_pitch = pattitude.pitch()
    val some_yaw = pattitude.yaw()
    val some_rollspeed = pattitude.rollspeed()
    val some_pitchspeed = pattitude.pitchspeed()
    val some_yawspeed = pattitude.yawspeed()

}

fun test_.ATTITUDE.copyFrom(SRC: ATTITUDE) {
    ATTITUDE.push(
            SRC, { src -> time_boot_ms(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }, { src -> rollspeed(src) }, { src -> pitchspeed(src) }, { src -> yawspeed(src) }

    )
}

fun onMISSION_WRITE_PARTIAL_LIST(pmission_write_partial_list: MISSION_WRITE_PARTIAL_LIST) {
    val some_target_system = pmission_write_partial_list.target_system()
    val some_target_component = pmission_write_partial_list.target_component()
    val some_start_index = pmission_write_partial_list.start_index()
    val some_end_index = pmission_write_partial_list.end_index()

    pmission_write_partial_list.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_WRITE_PARTIAL_LIST.copyFrom(SRC: MISSION_WRITE_PARTIAL_LIST) {
    MISSION_WRITE_PARTIAL_LIST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> start_index(src) }, { src -> end_index(src) }, { src -> mission_type(src) }

    )
}

fun onAHRS2(pahrs2: AHRS2) {
    val some_roll = pahrs2.roll()
    val some_pitch = pahrs2.pitch()
    val some_yaw = pahrs2.yaw()
    val some_altitude = pahrs2.altitude()
    val some_lat = pahrs2.lat()
    val some_lng = pahrs2.lng()

}

fun AHRS2.copyFrom(SRC: AHRS2) {
    AHRS2.push(
            SRC, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }, { src -> altitude(src) }, { src -> lat(src) }, { src -> lng(src) }

    )
}

fun fill(pahrs2: AHRS2) {
    pahrs2.roll(Float)
    pahrs2.pitch(Float)
    pahrs2.yaw(Float)
    pahrs2.altitude(Float)
    pahrs2.lat(Int)
    pahrs2.lng(Int)

}

fun AHRS2.copyInto(DST: AHRS2) {
    AHRS2.pull(
            DST, { -> roll() }, { -> pitch() }, { -> yaw() }, { -> altitude() }, { -> lat() }, { -> lng() }

    )
}

fun onLOG_ERASE(plog_erase: LOG_ERASE) {
    val some_target_system = plog_erase.target_system()
    val some_target_component = plog_erase.target_component()

}

fun LOG_ERASE.copyFrom(SRC: LOG_ERASE) {
    LOG_ERASE.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }

    )
}

fun fill(plog_erase: LOG_ERASE) {
    plog_erase.target_system(Byte)
    plog_erase.target_component(Byte)

}

fun LOG_ERASE.copyInto(DST: LOG_ERASE) {
    LOG_ERASE.pull(
            DST, { -> target_system() }, { -> target_component() }

    )
}

fun onTERRAIN_REQUEST(pterrain_request: TERRAIN_REQUEST) {
    val some_lat = pterrain_request.lat()
    val some_lon = pterrain_request.lon()
    val some_grid_spacing = pterrain_request.grid_spacing()
    val some_mask = pterrain_request.mask()

}

fun TERRAIN_REQUEST.copyFrom(SRC: TERRAIN_REQUEST) {
    TERRAIN_REQUEST.push(
            SRC, { src -> lat(src) }, { src -> lon(src) }, { src -> grid_spacing(src) }, { src -> mask(src) }

    )
}

fun fill(pterrain_request: TERRAIN_REQUEST) {
    pterrain_request.lat(Int)
    pterrain_request.lon(Int)
    pterrain_request.grid_spacing(Short)
    pterrain_request.mask(Long)

}

fun TERRAIN_REQUEST.copyInto(DST: TERRAIN_REQUEST) {
    TERRAIN_REQUEST.pull(
            DST, { -> lat() }, { -> lon() }, { -> grid_spacing() }, { -> mask() }

    )
}

fun onMOUNT_STATUS(pmount_status: MOUNT_STATUS) {
    val some_target_system = pmount_status.target_system()
    val some_target_component = pmount_status.target_component()
    val some_pointing_a = pmount_status.pointing_a()
    val some_pointing_b = pmount_status.pointing_b()
    val some_pointing_c = pmount_status.pointing_c()

}

fun MOUNT_STATUS.copyFrom(SRC: MOUNT_STATUS) {
    MOUNT_STATUS.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> pointing_a(src) }, { src -> pointing_b(src) }, { src -> pointing_c(src) }

    )
}

fun fill(pmount_status: MOUNT_STATUS) {
    pmount_status.target_system(Byte)
    pmount_status.target_component(Byte)
    pmount_status.pointing_a(Int)
    pmount_status.pointing_b(Int)
    pmount_status.pointing_c(Int)

}

fun MOUNT_STATUS.copyInto(DST: MOUNT_STATUS) {
    MOUNT_STATUS.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> pointing_a() }, { -> pointing_b() }, { -> pointing_c() }

    )
}

fun onMANUAL_SETPOINT(pmanual_setpoint: MANUAL_SETPOINT) {
    val some_time_boot_ms = pmanual_setpoint.time_boot_ms()
    val some_roll = pmanual_setpoint.roll()
    val some_pitch = pmanual_setpoint.pitch()
    val some_yaw = pmanual_setpoint.yaw()
    val some_thrust = pmanual_setpoint.thrust()
    val some_mode_switch = pmanual_setpoint.mode_switch()
    val some_manual_override_switch = pmanual_setpoint.manual_override_switch()

}

fun test_.MANUAL_SETPOINT.copyFrom(SRC: MANUAL_SETPOINT) {
    MANUAL_SETPOINT.push(
            SRC, { src -> time_boot_ms(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }, { src -> thrust(src) }, { src -> mode_switch(src) }, { src -> manual_override_switch(src) }

    )
}

fun onPID_TUNING(ppid_tuning: PID_TUNING) {

    ppid_tuning.axis()?.let { item ->
        some_PID_TUNING_AXIS = item.get()
    }
    val some_desired = ppid_tuning.desired()
    val some_achieved = ppid_tuning.achieved()
    val some_FF = ppid_tuning.FF()
    val some_P = ppid_tuning.P()
    val some_I = ppid_tuning.I()
    val some_D = ppid_tuning.D()

}

fun PID_TUNING.copyFrom(SRC: PID_TUNING) {
    PID_TUNING.push(
            SRC, { src -> axis(src) }, { src -> desired(src) }, { src -> achieved(src) }, { src -> FF(src) }, { src -> P(src) }, { src -> I(src) }, { src -> D(src) }

    )
}

fun fill(ppid_tuning: PID_TUNING) {

    ppid_tuning.axis(some_PID_TUNING_AXIS)
    ppid_tuning.desired(Float)
    ppid_tuning.achieved(Float)
    ppid_tuning.FF(Float)
    ppid_tuning.P(Float)
    ppid_tuning.I(Float)
    ppid_tuning.D(Float)

}

fun PID_TUNING.copyInto(DST: PID_TUNING) {
    PID_TUNING.pull(
            DST, { axis() != null }, { axis()!!.get() }, { -> desired() }, { -> achieved() }, { -> FF() }, { -> P() }, { -> I() }, { -> D() }

    )
}

fun onSAFETY_ALLOWED_AREA(psafety_allowed_area: SAFETY_ALLOWED_AREA) {

    psafety_allowed_area.frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }
    val some_p1x = psafety_allowed_area.p1x()
    val some_p1y = psafety_allowed_area.p1y()
    val some_p1z = psafety_allowed_area.p1z()
    val some_p2x = psafety_allowed_area.p2x()
    val some_p2y = psafety_allowed_area.p2y()
    val some_p2z = psafety_allowed_area.p2z()

}

fun test_.SAFETY_ALLOWED_AREA.copyFrom(SRC: SAFETY_ALLOWED_AREA) {
    SAFETY_ALLOWED_AREA.push(
            SRC, { src -> frame(src) }, { src -> p1x(src) }, { src -> p1y(src) }, { src -> p1z(src) }, { src -> p2x(src) }, { src -> p2y(src) }, { src -> p2z(src) }

    )
}

fun onOPTICAL_FLOW_RAD(poptical_flow_rad: OPTICAL_FLOW_RAD) {
    val some_time_usec = poptical_flow_rad.time_usec()
    val some_sensor_id = poptical_flow_rad.sensor_id()
    val some_integration_time_us = poptical_flow_rad.integration_time_us()
    val some_integrated_x = poptical_flow_rad.integrated_x()
    val some_integrated_y = poptical_flow_rad.integrated_y()
    val some_integrated_xgyro = poptical_flow_rad.integrated_xgyro()
    val some_integrated_ygyro = poptical_flow_rad.integrated_ygyro()
    val some_integrated_zgyro = poptical_flow_rad.integrated_zgyro()
    val some_temperature = poptical_flow_rad.temperature()
    val some_quality = poptical_flow_rad.quality()
    val some_time_delta_distance_us = poptical_flow_rad.time_delta_distance_us()
    val some_distance = poptical_flow_rad.distance()

}

fun OPTICAL_FLOW_RAD.copyFrom(SRC: OPTICAL_FLOW_RAD) {
    OPTICAL_FLOW_RAD.push(
            SRC, { src -> time_usec(src) }, { src -> sensor_id(src) }, { src -> integration_time_us(src) }, { src -> integrated_x(src) }, { src -> integrated_y(src) }, { src -> integrated_xgyro(src) }, { src -> integrated_ygyro(src) }, { src -> integrated_zgyro(src) }, { src -> temperature(src) }, { src -> quality(src) }, { src -> time_delta_distance_us(src) }, { src -> distance(src) }

    )
}

fun fill(poptical_flow_rad: OPTICAL_FLOW_RAD) {
    poptical_flow_rad.time_usec(Long)
    poptical_flow_rad.sensor_id(Byte)
    poptical_flow_rad.integration_time_us(Int)
    poptical_flow_rad.integrated_x(Float)
    poptical_flow_rad.integrated_y(Float)
    poptical_flow_rad.integrated_xgyro(Float)
    poptical_flow_rad.integrated_ygyro(Float)
    poptical_flow_rad.integrated_zgyro(Float)
    poptical_flow_rad.temperature(Short)
    poptical_flow_rad.quality(Byte)
    poptical_flow_rad.time_delta_distance_us(Int)
    poptical_flow_rad.distance(Float)

}

fun OPTICAL_FLOW_RAD.copyInto(DST: OPTICAL_FLOW_RAD) {
    OPTICAL_FLOW_RAD.pull(
            DST, { -> time_usec() }, { -> sensor_id() }, { -> integration_time_us() }, { -> integrated_x() }, { -> integrated_y() }, { -> integrated_xgyro() }, { -> integrated_ygyro() }, { -> integrated_zgyro() }, { -> temperature() }, { -> quality() }, { -> time_delta_distance_us() }, { -> distance() }

    )
}

fun onLOG_DATA(plog_data: LOG_DATA) {
    val some_id = plog_data.id()
    val some_ofs = plog_data.ofs()
    val some_count = plog_data.count()
    plog_data.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun LOG_DATA.copyFrom(SRC: LOG_DATA) {
    LOG_DATA.push(
            SRC, { src -> id(src) }, { src -> ofs(src) }, { src -> count(src) }, { src ->
        val item = daTa()
        for (i in 0 until 90)
            item.set(src.get(i), i)
    }

    )
}

fun fill(plog_data: LOG_DATA) {
    plog_data.id(Short)
    plog_data.ofs(Int)
    plog_data.count(Byte)
    plog_data.daTa().let { item ->
        plog_data.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun LOG_DATA.copyInto(DST: LOG_DATA) {
    LOG_DATA.pull(
            DST, { -> id() }, { -> ofs() }, { -> count() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 90)
            dst.set(src.get(i), i)
    }

    )
}

fun onMISSION_CLEAR_ALL(pmission_clear_all: MISSION_CLEAR_ALL) {
    val some_target_system = pmission_clear_all.target_system()
    val some_target_component = pmission_clear_all.target_component()

    pmission_clear_all.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_CLEAR_ALL.copyFrom(SRC: MISSION_CLEAR_ALL) {
    MISSION_CLEAR_ALL.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> mission_type(src) }

    )
}

fun onAHRS3(pahrs3: AHRS3) {
    val some_roll = pahrs3.roll()
    val some_pitch = pahrs3.pitch()
    val some_yaw = pahrs3.yaw()
    val some_altitude = pahrs3.altitude()
    val some_lat = pahrs3.lat()
    val some_lng = pahrs3.lng()
    val some_v1 = pahrs3.v1()
    val some_v2 = pahrs3.v2()
    val some_v3 = pahrs3.v3()
    val some_v4 = pahrs3.v4()

}

fun AHRS3.copyFrom(SRC: AHRS3) {
    AHRS3.push(
            SRC, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }, { src -> altitude(src) }, { src -> lat(src) }, { src -> lng(src) }, { src -> v1(src) }, { src -> v2(src) }, { src -> v3(src) }, { src -> v4(src) }

    )
}

fun fill(pahrs3: AHRS3) {
    pahrs3.roll(Float)
    pahrs3.pitch(Float)
    pahrs3.yaw(Float)
    pahrs3.altitude(Float)
    pahrs3.lat(Int)
    pahrs3.lng(Int)
    pahrs3.v1(Float)
    pahrs3.v2(Float)
    pahrs3.v3(Float)
    pahrs3.v4(Float)

}

fun AHRS3.copyInto(DST: AHRS3) {
    AHRS3.pull(
            DST, { -> roll() }, { -> pitch() }, { -> yaw() }, { -> altitude() }, { -> lat() }, { -> lng() }, { -> v1() }, { -> v2() }, { -> v3() }, { -> v4() }

    )
}

fun onVICON_POSITION_ESTIMATE(pvicon_position_estimate: VICON_POSITION_ESTIMATE) {
    val some_usec = pvicon_position_estimate.usec()
    val some_x = pvicon_position_estimate.x()
    val some_y = pvicon_position_estimate.y()
    val some_z = pvicon_position_estimate.z()
    val some_roll = pvicon_position_estimate.roll()
    val some_pitch = pvicon_position_estimate.pitch()
    val some_yaw = pvicon_position_estimate.yaw()

}

fun VICON_POSITION_ESTIMATE.copyFrom(SRC: VICON_POSITION_ESTIMATE) {
    VICON_POSITION_ESTIMATE.push(
            SRC, { src -> usec(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }

    )
}

fun fill(pvicon_position_estimate: VICON_POSITION_ESTIMATE) {
    pvicon_position_estimate.usec(Long)
    pvicon_position_estimate.x(Float)
    pvicon_position_estimate.y(Float)
    pvicon_position_estimate.z(Float)
    pvicon_position_estimate.roll(Float)
    pvicon_position_estimate.pitch(Float)
    pvicon_position_estimate.yaw(Float)

}

fun VICON_POSITION_ESTIMATE.copyInto(DST: VICON_POSITION_ESTIMATE) {
    VICON_POSITION_ESTIMATE.pull(
            DST, { -> usec() }, { -> x() }, { -> y() }, { -> z() }, { -> roll() }, { -> pitch() }, { -> yaw() }

    )
}

fun onGPS2_RTK(pgps2_rtk: GPS2_RTK) {
    val some_time_last_baseline_ms = pgps2_rtk.time_last_baseline_ms()
    val some_rtk_receiver_id = pgps2_rtk.rtk_receiver_id()
    val some_wn = pgps2_rtk.wn()
    val some_tow = pgps2_rtk.tow()
    val some_rtk_health = pgps2_rtk.rtk_health()
    val some_rtk_rate = pgps2_rtk.rtk_rate()
    val some_nsats = pgps2_rtk.nsats()
    val some_baseline_coords_type = pgps2_rtk.baseline_coords_type()
    val some_baseline_a_mm = pgps2_rtk.baseline_a_mm()
    val some_baseline_b_mm = pgps2_rtk.baseline_b_mm()
    val some_baseline_c_mm = pgps2_rtk.baseline_c_mm()
    val some_accuracy = pgps2_rtk.accuracy()
    val some_iar_num_hypotheses = pgps2_rtk.iar_num_hypotheses()

}

fun GPS2_RTK.copyFrom(SRC: GPS2_RTK) {
    GPS2_RTK.push(
            SRC, { src -> time_last_baseline_ms(src) }, { src -> rtk_receiver_id(src) }, { src -> wn(src) }, { src -> tow(src) }, { src -> rtk_health(src) }, { src -> rtk_rate(src) }, { src -> nsats(src) }, { src -> baseline_coords_type(src) }, { src -> baseline_a_mm(src) }, { src -> baseline_b_mm(src) }, { src -> baseline_c_mm(src) }, { src -> accuracy(src) }, { src -> iar_num_hypotheses(src) }

    )
}

fun fill(pgps2_rtk: GPS2_RTK) {
    pgps2_rtk.time_last_baseline_ms(Int)
    pgps2_rtk.rtk_receiver_id(Byte)
    pgps2_rtk.wn(Short)
    pgps2_rtk.tow(Int)
    pgps2_rtk.rtk_health(Byte)
    pgps2_rtk.rtk_rate(Byte)
    pgps2_rtk.nsats(Byte)
    pgps2_rtk.baseline_coords_type(Byte)
    pgps2_rtk.baseline_a_mm(Int)
    pgps2_rtk.baseline_b_mm(Int)
    pgps2_rtk.baseline_c_mm(Int)
    pgps2_rtk.accuracy(Int)
    pgps2_rtk.iar_num_hypotheses(Int)

}

fun GPS2_RTK.copyInto(DST: GPS2_RTK) {
    GPS2_RTK.pull(
            DST, { -> time_last_baseline_ms() }, { -> rtk_receiver_id() }, { -> wn() }, { -> tow() }, { -> rtk_health() }, { -> rtk_rate() }, { -> nsats() }, { -> baseline_coords_type() }, { -> baseline_a_mm() }, { -> baseline_b_mm() }, { -> baseline_c_mm() }, { -> accuracy() }, { -> iar_num_hypotheses() }

    )
}

fun onMAG_CAL_REPORT(pmag_cal_report: MAG_CAL_REPORT) {
    val some_compass_id = pmag_cal_report.compass_id()
    val some_cal_mask = pmag_cal_report.cal_mask()

    pmag_cal_report.cal_status()?.let { item ->
        some_MAG_CAL_STATUS = item.get()
    }
    val some_autosaved = pmag_cal_report.autosaved()
    val some_fitness = pmag_cal_report.fitness()
    val some_ofs_x = pmag_cal_report.ofs_x()
    val some_ofs_y = pmag_cal_report.ofs_y()
    val some_ofs_z = pmag_cal_report.ofs_z()
    val some_diag_x = pmag_cal_report.diag_x()
    val some_diag_y = pmag_cal_report.diag_y()
    val some_diag_z = pmag_cal_report.diag_z()
    val some_offdiag_x = pmag_cal_report.offdiag_x()
    val some_offdiag_y = pmag_cal_report.offdiag_y()
    val some_offdiag_z = pmag_cal_report.offdiag_z()

}

fun MAG_CAL_REPORT.copyFrom(SRC: MAG_CAL_REPORT) {
    MAG_CAL_REPORT.push(
            SRC, { src -> compass_id(src) }, { src -> cal_mask(src) }, { src -> cal_status(src) }, { src -> autosaved(src) }, { src -> fitness(src) }, { src -> ofs_x(src) }, { src -> ofs_y(src) }, { src -> ofs_z(src) }, { src -> diag_x(src) }, { src -> diag_y(src) }, { src -> diag_z(src) }, { src -> offdiag_x(src) }, { src -> offdiag_y(src) }, { src -> offdiag_z(src) }

    )
}

fun fill(pmag_cal_report: MAG_CAL_REPORT) {
    pmag_cal_report.compass_id(Byte)
    pmag_cal_report.cal_mask(Byte)

    pmag_cal_report.cal_status(some_MAG_CAL_STATUS)
    pmag_cal_report.autosaved(Byte)
    pmag_cal_report.fitness(Float)
    pmag_cal_report.ofs_x(Float)
    pmag_cal_report.ofs_y(Float)
    pmag_cal_report.ofs_z(Float)
    pmag_cal_report.diag_x(Float)
    pmag_cal_report.diag_y(Float)
    pmag_cal_report.diag_z(Float)
    pmag_cal_report.offdiag_x(Float)
    pmag_cal_report.offdiag_y(Float)
    pmag_cal_report.offdiag_z(Float)

}

fun MAG_CAL_REPORT.copyInto(DST: MAG_CAL_REPORT) {
    MAG_CAL_REPORT.pull(
            DST, { -> compass_id() }, { -> cal_mask() }, { cal_status() != null }, { cal_status()!!.get() }, { -> autosaved() }, { -> fitness() }, { -> ofs_x() }, { -> ofs_y() }, { -> ofs_z() }, { -> diag_x() }, { -> diag_y() }, { -> diag_z() }, { -> offdiag_x() }, { -> offdiag_y() }, { -> offdiag_z() }

    )
}

fun onLOG_REQUEST_LIST(plog_request_list: LOG_REQUEST_LIST) {
    val some_target_system = plog_request_list.target_system()
    val some_target_component = plog_request_list.target_component()
    val some_start = plog_request_list.start()
    val some_end = plog_request_list.end()

}

fun LOG_REQUEST_LIST.copyFrom(SRC: LOG_REQUEST_LIST) {
    LOG_REQUEST_LIST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> start(src) }, { src -> end(src) }

    )
}

fun fill(plog_request_list: LOG_REQUEST_LIST) {
    plog_request_list.target_system(Byte)
    plog_request_list.target_component(Byte)
    plog_request_list.start(Short)
    plog_request_list.end(Short)

}

fun LOG_REQUEST_LIST.copyInto(DST: LOG_REQUEST_LIST) {
    LOG_REQUEST_LIST.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> start() }, { -> end() }

    )
}

fun onSCALED_PRESSURE(pscaled_pressure: SCALED_PRESSURE) {
    val some_time_boot_ms = pscaled_pressure.time_boot_ms()
    val some_press_abs = pscaled_pressure.press_abs()
    val some_press_diff = pscaled_pressure.press_diff()
    val some_temperature = pscaled_pressure.temperature()

}

fun test_.SCALED_PRESSURE.copyFrom(SRC: SCALED_PRESSURE) {
    SCALED_PRESSURE.push(
            SRC, { src -> time_boot_ms(src) }, { src -> press_abs(src) }, { src -> press_diff(src) }, { src -> temperature(src) }

    )
}

fun onV2_EXTENSION(pv2_extension: V2_EXTENSION) {
    val some_target_network = pv2_extension.target_network()
    val some_target_system = pv2_extension.target_system()
    val some_target_component = pv2_extension.target_component()
    val some_message_type = pv2_extension.message_type()
    pv2_extension.payload().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun V2_EXTENSION.copyFrom(SRC: V2_EXTENSION) {
    V2_EXTENSION.push(
            SRC, { src -> target_network(src) }, { src -> target_system(src) }, { src -> target_component(src) }, { src -> message_type(src) }, { src ->
        val item = payload()
        for (i in 0 until 249)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pv2_extension: V2_EXTENSION) {
    pv2_extension.target_network(Byte)
    pv2_extension.target_system(Byte)
    pv2_extension.target_component(Byte)
    pv2_extension.message_type(Short)
    pv2_extension.payload().let { item ->
        pv2_extension.payload().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun V2_EXTENSION.copyInto(DST: V2_EXTENSION) {
    V2_EXTENSION.pull(
            DST, { -> target_network() }, { -> target_system() }, { -> target_component() }, { -> message_type() }, { dst ->
        var src = this.payload()
        for (i in 0 until 249)
            dst.set(src.get(i), i)
    }

    )
}

fun onHEARTBEAT(pheartbeat: HEARTBEAT) {

    pheartbeat.typE()?.let { item ->
        some_MAV_TYPE = item.get()
    }

    pheartbeat.autopilot()?.let { item ->
        some_MAV_AUTOPILOT = item.get()
    }

    pheartbeat.base_mode()?.let { item ->
        some_MAV_MODE_FLAG = item.get()
    }
    val some_custom_mode = pheartbeat.custom_mode()

    pheartbeat.system_status()?.let { item ->
        some_MAV_STATE = item.get()
    }
    val some_mavlink_version = pheartbeat.mavlink_version()

}

fun test_.HEARTBEAT.copyFrom(SRC: HEARTBEAT) {
    HEARTBEAT.push(
            SRC, { src -> typE(src) }, { src -> autopilot(src) }, { src -> base_mode(src) }, { src -> custom_mode(src) }, { src -> system_status(src) }, { src -> mavlink_version(src) }

    )
}

fun onPARAM_MAP_RC(pparam_map_rc: PARAM_MAP_RC) {
    val some_target_system = pparam_map_rc.target_system()
    val some_target_component = pparam_map_rc.target_component()

    pparam_map_rc.param_id()?.let { item -> char = item.get() }
    val some_param_index = pparam_map_rc.param_index()
    val some_parameter_rc_channel_index = pparam_map_rc.parameter_rc_channel_index()
    val some_param_value0 = pparam_map_rc.param_value0()
    val some_scale = pparam_map_rc.scale()
    val some_param_value_min = pparam_map_rc.param_value_min()
    val some_param_value_max = pparam_map_rc.param_value_max()

}

fun test_.PARAM_MAP_RC.copyFrom(SRC: PARAM_MAP_RC) {
    PARAM_MAP_RC.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = param_id(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> param_index(src) }, { src -> parameter_rc_channel_index(src) }, { src -> param_value0(src) }, { src -> scale(src) }, { src -> param_value_min(src) }, { src -> param_value_max(src) }

    )
}

fun onPOWER_STATUS(ppower_status: POWER_STATUS) {
    val some_Vcc = ppower_status.Vcc()
    val some_Vservo = ppower_status.Vservo()

    ppower_status.flags()?.let { item ->
        some_MAV_POWER_STATUS = item.get()
    }

}

fun POWER_STATUS.copyFrom(SRC: POWER_STATUS) {
    POWER_STATUS.push(
            SRC, { src -> Vcc(src) }, { src -> Vservo(src) }, { src -> flags(src) }

    )
}

fun fill(ppower_status: POWER_STATUS) {
    ppower_status.Vcc(Short)
    ppower_status.Vservo(Short)

    ppower_status.flags(some_MAV_POWER_STATUS)

}

fun POWER_STATUS.copyInto(DST: POWER_STATUS) {
    POWER_STATUS.pull(
            DST, { -> Vcc() }, { -> Vservo() }, { flags() != null }, { flags()!!.get() }

    )
}

fun onREMOTE_LOG_DATA_BLOCK(premote_log_data_block: REMOTE_LOG_DATA_BLOCK) {
    val some_target_system = premote_log_data_block.target_system()
    val some_target_component = premote_log_data_block.target_component()

    premote_log_data_block.seqno()?.let { item ->
        some_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS = item.get()
    }
    premote_log_data_block.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun REMOTE_LOG_DATA_BLOCK.copyFrom(SRC: REMOTE_LOG_DATA_BLOCK) {
    REMOTE_LOG_DATA_BLOCK.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> seqno(src) }, { src ->
        val item = daTa()
        for (i in 0 until 200)
            item.set(src.get(i), i)
    }

    )
}

fun fill(premote_log_data_block: REMOTE_LOG_DATA_BLOCK) {
    premote_log_data_block.target_system(Byte)
    premote_log_data_block.target_component(Byte)

    premote_log_data_block.seqno(some_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS)
    premote_log_data_block.daTa().let { item ->
        premote_log_data_block.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun REMOTE_LOG_DATA_BLOCK.copyInto(DST: REMOTE_LOG_DATA_BLOCK) {
    REMOTE_LOG_DATA_BLOCK.pull(
            DST, { -> target_system() }, { -> target_component() }, { seqno() != null }, { seqno()!!.get() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 200)
            dst.set(src.get(i), i)
    }

    )
}

fun onLOGGING_DATA_ACKED(plogging_data_acked: LOGGING_DATA_ACKED) {
    val some_target_system = plogging_data_acked.target_system()
    val some_target_component = plogging_data_acked.target_component()
    val some_sequence = plogging_data_acked.sequence()
    val some_length = plogging_data_acked.length()
    val some_first_message_offset = plogging_data_acked.first_message_offset()
    plogging_data_acked.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun LOGGING_DATA_ACKED.copyFrom(SRC: LOGGING_DATA_ACKED) {
    LOGGING_DATA_ACKED.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> sequence(src) }, { src -> length(src) }, { src -> first_message_offset(src) }, { src ->
        val item = daTa()
        for (i in 0 until 249)
            item.set(src.get(i), i)
    }

    )
}

fun fill(plogging_data_acked: LOGGING_DATA_ACKED) {
    plogging_data_acked.target_system(Byte)
    plogging_data_acked.target_component(Byte)
    plogging_data_acked.sequence(Short)
    plogging_data_acked.length(Byte)
    plogging_data_acked.first_message_offset(Byte)
    plogging_data_acked.daTa().let { item ->
        plogging_data_acked.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun LOGGING_DATA_ACKED.copyInto(DST: LOGGING_DATA_ACKED) {
    LOGGING_DATA_ACKED.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> sequence() }, { -> length() }, { -> first_message_offset() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 249)
            dst.set(src.get(i), i)
    }

    )
}

fun onTERRAIN_CHECK(pterrain_check: TERRAIN_CHECK) {
    val some_lat = pterrain_check.lat()
    val some_lon = pterrain_check.lon()

}

fun TERRAIN_CHECK.copyFrom(SRC: TERRAIN_CHECK) {
    TERRAIN_CHECK.push(
            SRC, { src -> lat(src) }, { src -> lon(src) }

    )
}

fun fill(pterrain_check: TERRAIN_CHECK) {
    pterrain_check.lat(Int)
    pterrain_check.lon(Int)

}

fun TERRAIN_CHECK.copyInto(DST: TERRAIN_CHECK) {
    TERRAIN_CHECK.pull(
            DST, { -> lat() }, { -> lon() }

    )
}

fun onMOUNT_CONFIGURE(pmount_configure: MOUNT_CONFIGURE) {
    val some_target_system = pmount_configure.target_system()
    val some_target_component = pmount_configure.target_component()

    pmount_configure.mount_mode()?.let { item ->
        some_MAV_MOUNT_MODE = item.get()
    }
    val some_stab_roll = pmount_configure.stab_roll()
    val some_stab_pitch = pmount_configure.stab_pitch()
    val some_stab_yaw = pmount_configure.stab_yaw()

}

fun MOUNT_CONFIGURE.copyFrom(SRC: MOUNT_CONFIGURE) {
    MOUNT_CONFIGURE.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> mount_mode(src) }, { src -> stab_roll(src) }, { src -> stab_pitch(src) }, { src -> stab_yaw(src) }

    )
}

fun fill(pmount_configure: MOUNT_CONFIGURE) {
    pmount_configure.target_system(Byte)
    pmount_configure.target_component(Byte)

    pmount_configure.mount_mode(some_MAV_MOUNT_MODE)
    pmount_configure.stab_roll(Byte)
    pmount_configure.stab_pitch(Byte)
    pmount_configure.stab_yaw(Byte)

}

fun MOUNT_CONFIGURE.copyInto(DST: MOUNT_CONFIGURE) {
    MOUNT_CONFIGURE.pull(
            DST, { -> target_system() }, { -> target_component() }, { mount_mode() != null }, { mount_mode()!!.get() }, { -> stab_roll() }, { -> stab_pitch() }, { -> stab_yaw() }

    )
}

fun onMISSION_REQUEST_INT(pmission_request_int: MISSION_REQUEST_INT) {
    val some_target_system = pmission_request_int.target_system()
    val some_target_component = pmission_request_int.target_component()
    val some_seq = pmission_request_int.seq()

    pmission_request_int.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_REQUEST_INT.copyFrom(SRC: MISSION_REQUEST_INT) {
    MISSION_REQUEST_INT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> seq(src) }, { src -> mission_type(src) }

    )
}

fun onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(plocal_position_ned_system_global_offset: LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) {
    val some_time_boot_ms = plocal_position_ned_system_global_offset.time_boot_ms()
    val some_x = plocal_position_ned_system_global_offset.x()
    val some_y = plocal_position_ned_system_global_offset.y()
    val some_z = plocal_position_ned_system_global_offset.z()
    val some_roll = plocal_position_ned_system_global_offset.roll()
    val some_pitch = plocal_position_ned_system_global_offset.pitch()
    val some_yaw = plocal_position_ned_system_global_offset.yaw()

}

fun test_.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.copyFrom(SRC: LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) {
    LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.push(
            SRC, { src -> time_boot_ms(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }

    )
}

fun onCOMMAND_ACK(pcommand_ack: COMMAND_ACK) {

    pcommand_ack.command()?.let { item ->
        some_MAV_CMD = item.get()
    }

    pcommand_ack.result()?.let { item ->
        some_MAV_RESULT = item.get()
    }

    pcommand_ack.progress().let { item -> println("Receive Byte pack.") }

    pcommand_ack.result_param2().let { item -> println("Receive Int pack.") }

    pcommand_ack.target_system().let { item -> println("Receive Byte pack.") }

    pcommand_ack.target_component().let { item -> println("Receive Byte pack.") }

}

fun test_.COMMAND_ACK.copyFrom(SRC: COMMAND_ACK) {
    COMMAND_ACK.push(
            SRC, { src -> command(src) }, { src -> result(src) }, { src -> progress(src) }, { src -> result_param2(src) }, { src -> target_system(src) }, { src -> target_component(src) }

    )
}

fun onDATA_STREAM(pdata_stream: DATA_STREAM) {
    val some_stream_id = pdata_stream.stream_id()
    val some_message_rate = pdata_stream.message_rate()
    val some_on_off = pdata_stream.on_off()

}

fun test_.DATA_STREAM.copyFrom(SRC: DATA_STREAM) {
    DATA_STREAM.push(
            SRC, { src -> stream_id(src) }, { src -> message_rate(src) }, { src -> on_off(src) }

    )
}

fun onMISSION_REQUEST(pmission_request: MISSION_REQUEST) {
    val some_target_system = pmission_request.target_system()
    val some_target_component = pmission_request.target_component()
    val some_seq = pmission_request.seq()

    pmission_request.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_REQUEST.copyFrom(SRC: MISSION_REQUEST) {
    MISSION_REQUEST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> seq(src) }, { src -> mission_type(src) }

    )
}

fun onTERRAIN_REPORT(pterrain_report: TERRAIN_REPORT) {
    val some_lat = pterrain_report.lat()
    val some_lon = pterrain_report.lon()
    val some_spacing = pterrain_report.spacing()
    val some_terrain_height = pterrain_report.terrain_height()
    val some_current_height = pterrain_report.current_height()
    val some_pending = pterrain_report.pending()
    val some_loaded = pterrain_report.loaded()

}

fun TERRAIN_REPORT.copyFrom(SRC: TERRAIN_REPORT) {
    TERRAIN_REPORT.push(
            SRC, { src -> lat(src) }, { src -> lon(src) }, { src -> spacing(src) }, { src -> terrain_height(src) }, { src -> current_height(src) }, { src -> pending(src) }, { src -> loaded(src) }

    )
}

fun fill(pterrain_report: TERRAIN_REPORT) {
    pterrain_report.lat(Int)
    pterrain_report.lon(Int)
    pterrain_report.spacing(Short)
    pterrain_report.terrain_height(Float)
    pterrain_report.current_height(Float)
    pterrain_report.pending(Short)
    pterrain_report.loaded(Short)

}

fun TERRAIN_REPORT.copyInto(DST: TERRAIN_REPORT) {
    TERRAIN_REPORT.pull(
            DST, { -> lat() }, { -> lon() }, { -> spacing() }, { -> terrain_height() }, { -> current_height() }, { -> pending() }, { -> loaded() }

    )
}

fun onSET_HOME_POSITION(pset_home_position: SET_HOME_POSITION) {
    val some_target_system = pset_home_position.target_system()
    val some_latitude = pset_home_position.latitude()
    val some_longitude = pset_home_position.longitude()
    val some_altitude = pset_home_position.altitude()
    val some_x = pset_home_position.x()
    val some_y = pset_home_position.y()
    val some_z = pset_home_position.z()
    pset_home_position.q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_approach_x = pset_home_position.approach_x()
    val some_approach_y = pset_home_position.approach_y()
    val some_approach_z = pset_home_position.approach_z()

    pset_home_position.time_usec().let { item -> println("Receive Long pack.") }

}

fun SET_HOME_POSITION.copyFrom(SRC: SET_HOME_POSITION) {
    SET_HOME_POSITION.push(
            SRC, { src -> target_system(src) }, { src -> latitude(src) }, { src -> longitude(src) }, { src -> altitude(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src ->
        val item = q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> approach_x(src) }, { src -> approach_y(src) }, { src -> approach_z(src) }, { src -> time_usec(src) }

    )
}

fun fill(pset_home_position: SET_HOME_POSITION) {
    pset_home_position.target_system(Byte)
    pset_home_position.latitude(Int)
    pset_home_position.longitude(Int)
    pset_home_position.altitude(Int)
    pset_home_position.x(Float)
    pset_home_position.y(Float)
    pset_home_position.z(Float)
    pset_home_position.q().let { item ->
        pset_home_position.q().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pset_home_position.approach_x(Float)
    pset_home_position.approach_y(Float)
    pset_home_position.approach_z(Float)

    pset_home_position.time_usec(Long)

}

fun SET_HOME_POSITION.copyInto(DST: SET_HOME_POSITION) {
    SET_HOME_POSITION.pull(
            DST, { -> target_system() }, { -> latitude() }, { -> longitude() }, { -> altitude() }, { -> x() }, { -> y() }, { -> z() }, { dst ->
        var src = this.q()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }, { -> approach_x() }, { -> approach_y() }, { -> approach_z() }, { time_usec() != null }, { time_usec().get() }

    )
}

fun onSwitchModeCommand() {}
fun onHIL_RC_INPUTS_RAW(phil_rc_inputs_raw: HIL_RC_INPUTS_RAW) {
    val some_time_usec = phil_rc_inputs_raw.time_usec()
    val some_chan1_raw = phil_rc_inputs_raw.chan1_raw()
    val some_chan2_raw = phil_rc_inputs_raw.chan2_raw()
    val some_chan3_raw = phil_rc_inputs_raw.chan3_raw()
    val some_chan4_raw = phil_rc_inputs_raw.chan4_raw()
    val some_chan5_raw = phil_rc_inputs_raw.chan5_raw()
    val some_chan6_raw = phil_rc_inputs_raw.chan6_raw()
    val some_chan7_raw = phil_rc_inputs_raw.chan7_raw()
    val some_chan8_raw = phil_rc_inputs_raw.chan8_raw()
    val some_chan9_raw = phil_rc_inputs_raw.chan9_raw()
    val some_chan10_raw = phil_rc_inputs_raw.chan10_raw()
    val some_chan11_raw = phil_rc_inputs_raw.chan11_raw()
    val some_chan12_raw = phil_rc_inputs_raw.chan12_raw()
    val some_rssi = phil_rc_inputs_raw.rssi()

}

fun test_.HIL_RC_INPUTS_RAW.copyFrom(SRC: HIL_RC_INPUTS_RAW) {
    HIL_RC_INPUTS_RAW.push(
            SRC, { src -> time_usec(src) }, { src -> chan1_raw(src) }, { src -> chan2_raw(src) }, { src -> chan3_raw(src) }, { src -> chan4_raw(src) }, { src -> chan5_raw(src) }, { src -> chan6_raw(src) }, { src -> chan7_raw(src) }, { src -> chan8_raw(src) }, { src -> chan9_raw(src) }, { src -> chan10_raw(src) }, { src -> chan11_raw(src) }, { src -> chan12_raw(src) }, { src -> rssi(src) }

    )
}

fun onSCALED_IMU3(pscaled_imu3: SCALED_IMU3) {
    val some_time_boot_ms = pscaled_imu3.time_boot_ms()
    val some_xacc = pscaled_imu3.xacc()
    val some_yacc = pscaled_imu3.yacc()
    val some_zacc = pscaled_imu3.zacc()
    val some_xgyro = pscaled_imu3.xgyro()
    val some_ygyro = pscaled_imu3.ygyro()
    val some_zgyro = pscaled_imu3.zgyro()
    val some_xmag = pscaled_imu3.xmag()
    val some_ymag = pscaled_imu3.ymag()
    val some_zmag = pscaled_imu3.zmag()

}

fun SCALED_IMU3.copyFrom(SRC: SCALED_IMU3) {
    SCALED_IMU3.push(
            SRC, { src -> time_boot_ms(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }, { src -> xgyro(src) }, { src -> ygyro(src) }, { src -> zgyro(src) }, { src -> xmag(src) }, { src -> ymag(src) }, { src -> zmag(src) }

    )
}

fun fill(pscaled_imu3: SCALED_IMU3) {
    pscaled_imu3.time_boot_ms(Int)
    pscaled_imu3.xacc(Short)
    pscaled_imu3.yacc(Short)
    pscaled_imu3.zacc(Short)
    pscaled_imu3.xgyro(Short)
    pscaled_imu3.ygyro(Short)
    pscaled_imu3.zgyro(Short)
    pscaled_imu3.xmag(Short)
    pscaled_imu3.ymag(Short)
    pscaled_imu3.zmag(Short)

}

fun SCALED_IMU3.copyInto(DST: SCALED_IMU3) {
    SCALED_IMU3.pull(
            DST, { -> time_boot_ms() }, { -> xacc() }, { -> yacc() }, { -> zacc() }, { -> xgyro() }, { -> ygyro() }, { -> zgyro() }, { -> xmag() }, { -> ymag() }, { -> zmag() }

    )
}

fun onSET_MODE(pset_mode: SET_MODE) {
    val some_target_system = pset_mode.target_system()

    pset_mode.base_mode()?.let { item ->
        some_MAV_MODE = item.get()
    }
    val some_custom_mode = pset_mode.custom_mode()

}

fun test_.SET_MODE.copyFrom(SRC: SET_MODE) {
    SET_MODE.push(
            SRC, { src -> target_system(src) }, { src -> base_mode(src) }, { src -> custom_mode(src) }

    )
}

fun onMOUNT_CONTROL(pmount_control: MOUNT_CONTROL) {
    val some_target_system = pmount_control.target_system()
    val some_target_component = pmount_control.target_component()
    val some_input_a = pmount_control.input_a()
    val some_input_b = pmount_control.input_b()
    val some_input_c = pmount_control.input_c()
    val some_save_position = pmount_control.save_position()

}

fun MOUNT_CONTROL.copyFrom(SRC: MOUNT_CONTROL) {
    MOUNT_CONTROL.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> input_a(src) }, { src -> input_b(src) }, { src -> input_c(src) }, { src -> save_position(src) }

    )
}

fun fill(pmount_control: MOUNT_CONTROL) {
    pmount_control.target_system(Byte)
    pmount_control.target_component(Byte)
    pmount_control.input_a(Int)
    pmount_control.input_b(Int)
    pmount_control.input_c(Int)
    pmount_control.save_position(Byte)

}

fun MOUNT_CONTROL.copyInto(DST: MOUNT_CONTROL) {
    MOUNT_CONTROL.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> input_a() }, { -> input_b() }, { -> input_c() }, { -> save_position() }

    )
}

fun onPOSITION_TARGET_GLOBAL_INT(pposition_target_global_int: POSITION_TARGET_GLOBAL_INT) {
    val some_time_boot_ms = pposition_target_global_int.time_boot_ms()

    pposition_target_global_int.coordinate_frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }
    val some_type_mask = pposition_target_global_int.type_mask()
    val some_lat_int = pposition_target_global_int.lat_int()
    val some_lon_int = pposition_target_global_int.lon_int()
    val some_alt = pposition_target_global_int.alt()
    val some_vx = pposition_target_global_int.vx()
    val some_vy = pposition_target_global_int.vy()
    val some_vz = pposition_target_global_int.vz()
    val some_afx = pposition_target_global_int.afx()
    val some_afy = pposition_target_global_int.afy()
    val some_afz = pposition_target_global_int.afz()
    val some_yaw = pposition_target_global_int.yaw()
    val some_yaw_rate = pposition_target_global_int.yaw_rate()

}

fun test_.POSITION_TARGET_GLOBAL_INT.copyFrom(SRC: POSITION_TARGET_GLOBAL_INT) {
    POSITION_TARGET_GLOBAL_INT.push(
            SRC, { src -> time_boot_ms(src) }, { src -> coordinate_frame(src) }, { src -> type_mask(src) }, { src -> lat_int(src) }, { src -> lon_int(src) }, { src -> alt(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> afx(src) }, { src -> afy(src) }, { src -> afz(src) }, { src -> yaw(src) }, { src -> yaw_rate(src) }

    )
}

fun onLED_CONTROL(pled_control: LED_CONTROL) {
    val some_target_system = pled_control.target_system()
    val some_target_component = pled_control.target_component()
    val some_instance = pled_control.instance()
    val some_pattern = pled_control.pattern()
    val some_custom_len = pled_control.custom_len()
    pled_control.custom_bytes().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun LED_CONTROL.copyFrom(SRC: LED_CONTROL) {
    LED_CONTROL.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> instance(src) }, { src -> pattern(src) }, { src -> custom_len(src) }, { src ->
        val item = custom_bytes()
        for (i in 0 until 24)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pled_control: LED_CONTROL) {
    pled_control.target_system(Byte)
    pled_control.target_component(Byte)
    pled_control.instance(Byte)
    pled_control.pattern(Byte)
    pled_control.custom_len(Byte)
    pled_control.custom_bytes().let { item ->
        pled_control.custom_bytes().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun LED_CONTROL.copyInto(DST: LED_CONTROL) {
    LED_CONTROL.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> instance() }, { -> pattern() }, { -> custom_len() }, { dst ->
        var src = this.custom_bytes()
        for (i in 0 until 24)
            dst.set(src.get(i), i)
    }

    )
}

fun onSIM_STATE(psim_state: SIM_STATE) {
    val some_q1 = psim_state.q1()
    val some_q2 = psim_state.q2()
    val some_q3 = psim_state.q3()
    val some_q4 = psim_state.q4()
    val some_roll = psim_state.roll()
    val some_pitch = psim_state.pitch()
    val some_yaw = psim_state.yaw()
    val some_xacc = psim_state.xacc()
    val some_yacc = psim_state.yacc()
    val some_zacc = psim_state.zacc()
    val some_xgyro = psim_state.xgyro()
    val some_ygyro = psim_state.ygyro()
    val some_zgyro = psim_state.zgyro()
    val some_lat = psim_state.lat()
    val some_lon = psim_state.lon()
    val some_alt = psim_state.alt()
    val some_std_dev_horz = psim_state.std_dev_horz()
    val some_std_dev_vert = psim_state.std_dev_vert()
    val some_vn = psim_state.vn()
    val some_ve = psim_state.ve()
    val some_vd = psim_state.vd()

}

fun SIM_STATE.copyFrom(SRC: SIM_STATE) {
    SIM_STATE.push(
            SRC, { src -> q1(src) }, { src -> q2(src) }, { src -> q3(src) }, { src -> q4(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }, { src -> xgyro(src) }, { src -> ygyro(src) }, { src -> zgyro(src) }, { src -> lat(src) }, { src -> lon(src) }, { src -> alt(src) }, { src -> std_dev_horz(src) }, { src -> std_dev_vert(src) }, { src -> vn(src) }, { src -> ve(src) }, { src -> vd(src) }

    )
}

fun fill(psim_state: SIM_STATE) {
    psim_state.q1(Float)
    psim_state.q2(Float)
    psim_state.q3(Float)
    psim_state.q4(Float)
    psim_state.roll(Float)
    psim_state.pitch(Float)
    psim_state.yaw(Float)
    psim_state.xacc(Float)
    psim_state.yacc(Float)
    psim_state.zacc(Float)
    psim_state.xgyro(Float)
    psim_state.ygyro(Float)
    psim_state.zgyro(Float)
    psim_state.lat(Float)
    psim_state.lon(Float)
    psim_state.alt(Float)
    psim_state.std_dev_horz(Float)
    psim_state.std_dev_vert(Float)
    psim_state.vn(Float)
    psim_state.ve(Float)
    psim_state.vd(Float)

}

fun SIM_STATE.copyInto(DST: SIM_STATE) {
    SIM_STATE.pull(
            DST, { -> q1() }, { -> q2() }, { -> q3() }, { -> q4() }, { -> roll() }, { -> pitch() }, { -> yaw() }, { -> xacc() }, { -> yacc() }, { -> zacc() }, { -> xgyro() }, { -> ygyro() }, { -> zgyro() }, { -> lat() }, { -> lon() }, { -> alt() }, { -> std_dev_horz() }, { -> std_dev_vert() }, { -> vn() }, { -> ve() }, { -> vd() }

    )
}

fun onWIFI_CONFIG_AP(pwifi_config_ap: WIFI_CONFIG_AP) {

    pwifi_config_ap.ssid()?.let { item -> char = item.get() }

    pwifi_config_ap.password()?.let { item -> char = item.get() }

}

fun WIFI_CONFIG_AP.copyFrom(SRC: WIFI_CONFIG_AP) {
    WIFI_CONFIG_AP.push(
            SRC, { src ->
        val item = ssid(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src ->
        val item = password(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun fill(pwifi_config_ap: WIFI_CONFIG_AP) {

    pwifi_config_ap.ssid(some_String, null)

    pwifi_config_ap.password(some_String, null)

}

fun WIFI_CONFIG_AP.copyInto(DST: WIFI_CONFIG_AP) {
    WIFI_CONFIG_AP.pull(
            DST, { ssid()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = ssid()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { password()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = password()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }

    )
}

fun onDATA96(pdata96: DATA96) {
    val some_typE = pdata96.typE()
    val some_len = pdata96.len()
    pdata96.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun DATA96.copyFrom(SRC: DATA96) {
    DATA96.push(
            SRC, { src -> typE(src) }, { src -> len(src) }, { src ->
        val item = daTa()
        for (i in 0 until 96)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pdata96: DATA96) {
    pdata96.typE(Byte)
    pdata96.len(Byte)
    pdata96.daTa().let { item ->
        pdata96.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun DATA96.copyInto(DST: DATA96) {
    DATA96.pull(
            DST, { -> typE() }, { -> len() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 96)
            dst.set(src.get(i), i)
    }

    )
}

fun onFLIGHT_INFORMATION(pflight_information: FLIGHT_INFORMATION) {
    val some_time_boot_ms = pflight_information.time_boot_ms()
    val some_arming_time_utc = pflight_information.arming_time_utc()
    val some_takeoff_time_utc = pflight_information.takeoff_time_utc()
    val some_flight_uuid = pflight_information.flight_uuid()

}

fun FLIGHT_INFORMATION.copyFrom(SRC: FLIGHT_INFORMATION) {
    FLIGHT_INFORMATION.push(
            SRC, { src -> time_boot_ms(src) }, { src -> arming_time_utc(src) }, { src -> takeoff_time_utc(src) }, { src -> flight_uuid(src) }

    )
}

fun fill(pflight_information: FLIGHT_INFORMATION) {
    pflight_information.time_boot_ms(Int)
    pflight_information.arming_time_utc(Long)
    pflight_information.takeoff_time_utc(Long)
    pflight_information.flight_uuid(Long)

}

fun FLIGHT_INFORMATION.copyInto(DST: FLIGHT_INFORMATION) {
    FLIGHT_INFORMATION.pull(
            DST, { -> time_boot_ms() }, { -> arming_time_utc() }, { -> takeoff_time_utc() }, { -> flight_uuid() }

    )
}

fun onRC_CHANNELS_RAW(prc_channels_raw: RC_CHANNELS_RAW) {
    val some_time_boot_ms = prc_channels_raw.time_boot_ms()
    val some_port = prc_channels_raw.port()
    val some_chan1_raw = prc_channels_raw.chan1_raw()
    val some_chan2_raw = prc_channels_raw.chan2_raw()
    val some_chan3_raw = prc_channels_raw.chan3_raw()
    val some_chan4_raw = prc_channels_raw.chan4_raw()
    val some_chan5_raw = prc_channels_raw.chan5_raw()
    val some_chan6_raw = prc_channels_raw.chan6_raw()
    val some_chan7_raw = prc_channels_raw.chan7_raw()
    val some_chan8_raw = prc_channels_raw.chan8_raw()
    val some_rssi = prc_channels_raw.rssi()

}

fun test_.RC_CHANNELS_RAW.copyFrom(SRC: RC_CHANNELS_RAW) {
    RC_CHANNELS_RAW.push(
            SRC, { src -> time_boot_ms(src) }, { src -> port(src) }, { src -> chan1_raw(src) }, { src -> chan2_raw(src) }, { src -> chan3_raw(src) }, { src -> chan4_raw(src) }, { src -> chan5_raw(src) }, { src -> chan6_raw(src) }, { src -> chan7_raw(src) }, { src -> chan8_raw(src) }, { src -> rssi(src) }

    )
}

fun onSERVO_OUTPUT_RAW(pservo_output_raw: SERVO_OUTPUT_RAW) {
    val some_time_usec = pservo_output_raw.time_usec()
    val some_port = pservo_output_raw.port()
    val some_servo1_raw = pservo_output_raw.servo1_raw()
    val some_servo2_raw = pservo_output_raw.servo2_raw()
    val some_servo3_raw = pservo_output_raw.servo3_raw()
    val some_servo4_raw = pservo_output_raw.servo4_raw()
    val some_servo5_raw = pservo_output_raw.servo5_raw()
    val some_servo6_raw = pservo_output_raw.servo6_raw()
    val some_servo7_raw = pservo_output_raw.servo7_raw()
    val some_servo8_raw = pservo_output_raw.servo8_raw()

    pservo_output_raw.servo9_raw().let { item -> println("Receive Short pack.") }

    pservo_output_raw.servo10_raw().let { item -> println("Receive Short pack.") }

    pservo_output_raw.servo11_raw().let { item -> println("Receive Short pack.") }

    pservo_output_raw.servo12_raw().let { item -> println("Receive Short pack.") }

    pservo_output_raw.servo13_raw().let { item -> println("Receive Short pack.") }

    pservo_output_raw.servo14_raw().let { item -> println("Receive Short pack.") }

    pservo_output_raw.servo15_raw().let { item -> println("Receive Short pack.") }

    pservo_output_raw.servo16_raw().let { item -> println("Receive Short pack.") }

}

fun test_.SERVO_OUTPUT_RAW.copyFrom(SRC: SERVO_OUTPUT_RAW) {
    SERVO_OUTPUT_RAW.push(
            SRC, { src -> time_usec(src) }, { src -> port(src) }, { src -> servo1_raw(src) }, { src -> servo2_raw(src) }, { src -> servo3_raw(src) }, { src -> servo4_raw(src) }, { src -> servo5_raw(src) }, { src -> servo6_raw(src) }, { src -> servo7_raw(src) }, { src -> servo8_raw(src) }, { src -> servo9_raw(src) }, { src -> servo10_raw(src) }, { src -> servo11_raw(src) }, { src -> servo12_raw(src) }, { src -> servo13_raw(src) }, { src -> servo14_raw(src) }, { src -> servo15_raw(src) }, { src -> servo16_raw(src) }

    )
}

fun onMEMINFO(pmeminfo: MEMINFO) {
    val some_brkval = pmeminfo.brkval()
    val some_freemem = pmeminfo.freemem()

    pmeminfo.freemem32().let { item -> println("Receive Int pack.") }

}

fun MEMINFO.copyFrom(SRC: MEMINFO) {
    MEMINFO.push(
            SRC, { src -> brkval(src) }, { src -> freemem(src) }, { src -> freemem32(src) }

    )
}

fun fill(pmeminfo: MEMINFO) {
    pmeminfo.brkval(Short)
    pmeminfo.freemem(Short)

    pmeminfo.freemem32(Int)

}

fun MEMINFO.copyInto(DST: MEMINFO) {
    MEMINFO.pull(
            DST, { -> brkval() }, { -> freemem() }, { freemem32() != null }, { freemem32().get() }

    )
}

fun onMISSION_ITEM_REACHED(pmission_item_reached: MISSION_ITEM_REACHED) {
    val some_seq = pmission_item_reached.seq()

}

fun test_.MISSION_ITEM_REACHED.copyFrom(SRC: MISSION_ITEM_REACHED) {
    MISSION_ITEM_REACHED.push(
            SRC, { src -> seq(src) }

    )
}

fun onLOGGING_ACK(plogging_ack: LOGGING_ACK) {
    val some_target_system = plogging_ack.target_system()
    val some_target_component = plogging_ack.target_component()
    val some_sequence = plogging_ack.sequence()

}

fun LOGGING_ACK.copyFrom(SRC: LOGGING_ACK) {
    LOGGING_ACK.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> sequence(src) }

    )
}

fun fill(plogging_ack: LOGGING_ACK) {
    plogging_ack.target_system(Byte)
    plogging_ack.target_component(Byte)
    plogging_ack.sequence(Short)

}

fun LOGGING_ACK.copyInto(DST: LOGGING_ACK) {
    LOGGING_ACK.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> sequence() }

    )
}

fun onVISION_SPEED_ESTIMATE(pvision_speed_estimate: VISION_SPEED_ESTIMATE) {
    val some_usec = pvision_speed_estimate.usec()
    val some_x = pvision_speed_estimate.x()
    val some_y = pvision_speed_estimate.y()
    val some_z = pvision_speed_estimate.z()

}

fun VISION_SPEED_ESTIMATE.copyFrom(SRC: VISION_SPEED_ESTIMATE) {
    VISION_SPEED_ESTIMATE.push(
            SRC, { src -> usec(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }

    )
}

fun fill(pvision_speed_estimate: VISION_SPEED_ESTIMATE) {
    pvision_speed_estimate.usec(Long)
    pvision_speed_estimate.x(Float)
    pvision_speed_estimate.y(Float)
    pvision_speed_estimate.z(Float)

}

fun VISION_SPEED_ESTIMATE.copyInto(DST: VISION_SPEED_ESTIMATE) {
    VISION_SPEED_ESTIMATE.pull(
            DST, { -> usec() }, { -> x() }, { -> y() }, { -> z() }

    )
}

fun onDEBUG_VECT(pdebug_vect: DEBUG_VECT) {

    pdebug_vect.name()?.let { item -> char = item.get() }
    val some_time_usec = pdebug_vect.time_usec()
    val some_x = pdebug_vect.x()
    val some_y = pdebug_vect.y()
    val some_z = pdebug_vect.z()

}

fun DEBUG_VECT.copyFrom(SRC: DEBUG_VECT) {
    DEBUG_VECT.push(
            SRC, { src ->
        val item = name(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> time_usec(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }

    )
}

fun fill(pdebug_vect: DEBUG_VECT) {

    pdebug_vect.name(some_String, null)
    pdebug_vect.time_usec(Long)
    pdebug_vect.x(Float)
    pdebug_vect.y(Float)
    pdebug_vect.z(Float)

}

fun DEBUG_VECT.copyInto(DST: DEBUG_VECT) {
    DEBUG_VECT.pull(
            DST, { name()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = name()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { -> time_usec() }, { -> x() }, { -> y() }, { -> z() }

    )
}

fun onLOG_REQUEST_END(plog_request_end: LOG_REQUEST_END) {
    val some_target_system = plog_request_end.target_system()
    val some_target_component = plog_request_end.target_component()

}

fun LOG_REQUEST_END.copyFrom(SRC: LOG_REQUEST_END) {
    LOG_REQUEST_END.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }

    )
}

fun fill(plog_request_end: LOG_REQUEST_END) {
    plog_request_end.target_system(Byte)
    plog_request_end.target_component(Byte)

}

fun LOG_REQUEST_END.copyInto(DST: LOG_REQUEST_END) {
    LOG_REQUEST_END.pull(
            DST, { -> target_system() }, { -> target_component() }

    )
}

fun onMISSION_ACK(pmission_ack: MISSION_ACK) {
    val some_target_system = pmission_ack.target_system()
    val some_target_component = pmission_ack.target_component()

    pmission_ack.typE()?.let { item ->
        some_MAV_MISSION_RESULT = item.get()
    }

    pmission_ack.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_ACK.copyFrom(SRC: MISSION_ACK) {
    MISSION_ACK.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> typE(src) }, { src -> mission_type(src) }

    )
}

fun onCHANGE_OPERATOR_CONTROL_ACK(pchange_operator_control_ack: CHANGE_OPERATOR_CONTROL_ACK) {
    val some_gcs_system_id = pchange_operator_control_ack.gcs_system_id()
    val some_control_request = pchange_operator_control_ack.control_request()
    val some_ack = pchange_operator_control_ack.ack()

}

fun test_.CHANGE_OPERATOR_CONTROL_ACK.copyFrom(SRC: CHANGE_OPERATOR_CONTROL_ACK) {
    CHANGE_OPERATOR_CONTROL_ACK.push(
            SRC, { src -> gcs_system_id(src) }, { src -> control_request(src) }, { src -> ack(src) }

    )
}

fun onMISSION_CURRENT(pmission_current: MISSION_CURRENT) {
    val some_seq = pmission_current.seq()

}

fun test_.MISSION_CURRENT.copyFrom(SRC: MISSION_CURRENT) {
    MISSION_CURRENT.push(
            SRC, { src -> seq(src) }

    )
}

fun onSYSTEM_TIME(psystem_time: SYSTEM_TIME) {
    val some_time_unix_usec = psystem_time.time_unix_usec()
    val some_time_boot_ms = psystem_time.time_boot_ms()

}

fun test_.SYSTEM_TIME.copyFrom(SRC: SYSTEM_TIME) {
    SYSTEM_TIME.push(
            SRC, { src -> time_unix_usec(src) }, { src -> time_boot_ms(src) }

    )
}

fun onCAMERA_TRIGGER(pcamera_trigger: CAMERA_TRIGGER) {
    val some_time_usec = pcamera_trigger.time_usec()
    val some_seq = pcamera_trigger.seq()

}

fun CAMERA_TRIGGER.copyFrom(SRC: CAMERA_TRIGGER) {
    CAMERA_TRIGGER.push(
            SRC, { src -> time_usec(src) }, { src -> seq(src) }

    )
}

fun fill(pcamera_trigger: CAMERA_TRIGGER) {
    pcamera_trigger.time_usec(Long)
    pcamera_trigger.seq(Int)

}

fun CAMERA_TRIGGER.copyInto(DST: CAMERA_TRIGGER) {
    CAMERA_TRIGGER.pull(
            DST, { -> time_usec() }, { -> seq() }

    )
}

fun onGOPRO_SET_RESPONSE(pgopro_set_response: GOPRO_SET_RESPONSE) {

    pgopro_set_response.cmd_id()?.let { item ->
        some_GOPRO_COMMAND = item.get()
    }

    pgopro_set_response.status()?.let { item ->
        some_GOPRO_REQUEST_STATUS = item.get()
    }

}

fun GOPRO_SET_RESPONSE.copyFrom(SRC: GOPRO_SET_RESPONSE) {
    GOPRO_SET_RESPONSE.push(
            SRC, { src -> cmd_id(src) }, { src -> status(src) }

    )
}

fun fill(pgopro_set_response: GOPRO_SET_RESPONSE) {

    pgopro_set_response.cmd_id(some_GOPRO_COMMAND)

    pgopro_set_response.status(some_GOPRO_REQUEST_STATUS)

}

fun GOPRO_SET_RESPONSE.copyInto(DST: GOPRO_SET_RESPONSE) {
    GOPRO_SET_RESPONSE.pull(
            DST, { cmd_id() != null }, { cmd_id()!!.get() }, { status() != null }, { status()!!.get() }

    )
}

fun onVISION_POSITION_ESTIMATE(pvision_position_estimate: VISION_POSITION_ESTIMATE) {
    val some_usec = pvision_position_estimate.usec()
    val some_x = pvision_position_estimate.x()
    val some_y = pvision_position_estimate.y()
    val some_z = pvision_position_estimate.z()
    val some_roll = pvision_position_estimate.roll()
    val some_pitch = pvision_position_estimate.pitch()
    val some_yaw = pvision_position_estimate.yaw()

}

fun test_.VISION_POSITION_ESTIMATE.copyFrom(SRC: VISION_POSITION_ESTIMATE) {
    VISION_POSITION_ESTIMATE.push(
            SRC, { src -> usec(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }

    )
}

fun onMANUAL_CONTROL(pmanual_control: MANUAL_CONTROL) {
    val some_target = pmanual_control.target()
    val some_x = pmanual_control.x()
    val some_y = pmanual_control.y()
    val some_z = pmanual_control.z()
    val some_r = pmanual_control.r()
    val some_buttons = pmanual_control.buttons()

}

fun test_.MANUAL_CONTROL.copyFrom(SRC: MANUAL_CONTROL) {
    MANUAL_CONTROL.push(
            SRC, { src -> target(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> r(src) }, { src -> buttons(src) }

    )
}

fun onRC_CHANNELS(prc_channels: RC_CHANNELS) {
    val some_time_boot_ms = prc_channels.time_boot_ms()
    val some_chancount = prc_channels.chancount()
    val some_chan1_raw = prc_channels.chan1_raw()
    val some_chan2_raw = prc_channels.chan2_raw()
    val some_chan3_raw = prc_channels.chan3_raw()
    val some_chan4_raw = prc_channels.chan4_raw()
    val some_chan5_raw = prc_channels.chan5_raw()
    val some_chan6_raw = prc_channels.chan6_raw()
    val some_chan7_raw = prc_channels.chan7_raw()
    val some_chan8_raw = prc_channels.chan8_raw()
    val some_chan9_raw = prc_channels.chan9_raw()
    val some_chan10_raw = prc_channels.chan10_raw()
    val some_chan11_raw = prc_channels.chan11_raw()
    val some_chan12_raw = prc_channels.chan12_raw()
    val some_chan13_raw = prc_channels.chan13_raw()
    val some_chan14_raw = prc_channels.chan14_raw()
    val some_chan15_raw = prc_channels.chan15_raw()
    val some_chan16_raw = prc_channels.chan16_raw()
    val some_chan17_raw = prc_channels.chan17_raw()
    val some_chan18_raw = prc_channels.chan18_raw()
    val some_rssi = prc_channels.rssi()

}

fun test_.RC_CHANNELS.copyFrom(SRC: RC_CHANNELS) {
    RC_CHANNELS.push(
            SRC, { src -> time_boot_ms(src) }, { src -> chancount(src) }, { src -> chan1_raw(src) }, { src -> chan2_raw(src) }, { src -> chan3_raw(src) }, { src -> chan4_raw(src) }, { src -> chan5_raw(src) }, { src -> chan6_raw(src) }, { src -> chan7_raw(src) }, { src -> chan8_raw(src) }, { src -> chan9_raw(src) }, { src -> chan10_raw(src) }, { src -> chan11_raw(src) }, { src -> chan12_raw(src) }, { src -> chan13_raw(src) }, { src -> chan14_raw(src) }, { src -> chan15_raw(src) }, { src -> chan16_raw(src) }, { src -> chan17_raw(src) }, { src -> chan18_raw(src) }, { src -> rssi(src) }

    )
}

fun onPROTOCOL_VERSION(pprotocol_version: PROTOCOL_VERSION) {
    val some_version = pprotocol_version.version()
    val some_min_version = pprotocol_version.min_version()
    val some_max_version = pprotocol_version.max_version()
    pprotocol_version.spec_version_hash().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    pprotocol_version.library_version_hash().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun PROTOCOL_VERSION.copyFrom(SRC: PROTOCOL_VERSION) {
    PROTOCOL_VERSION.push(
            SRC, { src -> version(src) }, { src -> min_version(src) }, { src -> max_version(src) }, { src ->
        val item = spec_version_hash()
        for (i in 0 until 8)
            item.set(src.get(i), i)
    }, { src ->
        val item = library_version_hash()
        for (i in 0 until 8)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pprotocol_version: PROTOCOL_VERSION) {
    pprotocol_version.version(Short)
    pprotocol_version.min_version(Short)
    pprotocol_version.max_version(Short)
    pprotocol_version.spec_version_hash().let { item ->
        pprotocol_version.spec_version_hash().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    pprotocol_version.library_version_hash().let { item ->
        pprotocol_version.library_version_hash().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun PROTOCOL_VERSION.copyInto(DST: PROTOCOL_VERSION) {
    PROTOCOL_VERSION.pull(
            DST, { -> version() }, { -> min_version() }, { -> max_version() }, { dst ->
        var src = this.spec_version_hash()
        for (i in 0 until 8)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.library_version_hash()
        for (i in 0 until 8)
            dst.set(src.get(i), i)
    }

    )
}

fun onRALLY_FETCH_POINT(prally_fetch_point: RALLY_FETCH_POINT) {
    val some_target_system = prally_fetch_point.target_system()
    val some_target_component = prally_fetch_point.target_component()
    val some_idx = prally_fetch_point.idx()

}

fun RALLY_FETCH_POINT.copyFrom(SRC: RALLY_FETCH_POINT) {
    RALLY_FETCH_POINT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> idx(src) }

    )
}

fun fill(prally_fetch_point: RALLY_FETCH_POINT) {
    prally_fetch_point.target_system(Byte)
    prally_fetch_point.target_component(Byte)
    prally_fetch_point.idx(Byte)

}

fun RALLY_FETCH_POINT.copyInto(DST: RALLY_FETCH_POINT) {
    RALLY_FETCH_POINT.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> idx() }

    )
}

fun onPARAM_VALUE(pparam_value: PARAM_VALUE) {

    pparam_value.param_id()?.let { item -> char = item.get() }
    val some_param_value = pparam_value.param_value()

    pparam_value.param_type()?.let { item ->
        some_MAV_PARAM_TYPE = item.get()
    }
    val some_param_count = pparam_value.param_count()
    val some_param_index = pparam_value.param_index()

}

fun test_.PARAM_VALUE.copyFrom(SRC: PARAM_VALUE) {
    PARAM_VALUE.push(
            SRC, { src ->
        val item = param_id(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> param_value(src) }, { src -> param_type(src) }, { src -> param_count(src) }, { src -> param_index(src) }

    )
}

fun onBATTERY_STATUS(pbattery_status: BATTERY_STATUS) {
    val some_id = pbattery_status.id()

    pbattery_status.battery_function()?.let { item ->
        some_MAV_BATTERY_FUNCTION = item.get()
    }

    pbattery_status.typE()?.let { item ->
        some_MAV_BATTERY_TYPE = item.get()
    }
    val some_temperature = pbattery_status.temperature()
    pbattery_status.voltages().let { item ->
        for (index in 0 until item.len())
            Short = item.get(index)
    }
    val some_current_battery = pbattery_status.current_battery()
    val some_current_consumed = pbattery_status.current_consumed()
    val some_energy_consumed = pbattery_status.energy_consumed()
    val some_battery_remaining = pbattery_status.battery_remaining()

}

fun BATTERY_STATUS.copyFrom(SRC: BATTERY_STATUS) {
    BATTERY_STATUS.push(
            SRC, { src -> id(src) }, { src -> battery_function(src) }, { src -> typE(src) }, { src -> temperature(src) }, { src ->
        val item = voltages()
        for (i in 0 until 10)
            item.set(src.get(i), i)
    }, { src -> current_battery(src) }, { src -> current_consumed(src) }, { src -> energy_consumed(src) }, { src -> battery_remaining(src) }

    )
}

fun fill(pbattery_status: BATTERY_STATUS) {
    pbattery_status.id(Byte)

    pbattery_status.battery_function(some_MAV_BATTERY_FUNCTION)

    pbattery_status.typE(some_MAV_BATTERY_TYPE)
    pbattery_status.temperature(Short)
    pbattery_status.voltages().let { item ->
        pbattery_status.voltages().let { item ->
            for (index in 0 until item.len())
                item.set(Short, index)
        }
    }
    pbattery_status.current_battery(Short)
    pbattery_status.current_consumed(Int)
    pbattery_status.energy_consumed(Int)
    pbattery_status.battery_remaining(Byte)

}

fun BATTERY_STATUS.copyInto(DST: BATTERY_STATUS) {
    BATTERY_STATUS.pull(
            DST, { -> id() }, { battery_function() != null }, { battery_function()!!.get() }, { typE() != null }, { typE()!!.get() }, { -> temperature() }, { dst ->
        var src = this.voltages()
        for (i in 0 until 10)
            dst.set(src.get(i), i)
    }, { -> current_battery() }, { -> current_consumed() }, { -> energy_consumed() }, { -> battery_remaining() }

    )
}

fun onSERIAL_CONTROL(pserial_control: SERIAL_CONTROL) {

    pserial_control.device()?.let { item ->
        some_SERIAL_CONTROL_DEV = item.get()
    }

    pserial_control.flags()?.let { item ->
        some_SERIAL_CONTROL_FLAG = item.get()
    }
    val some_timeout = pserial_control.timeout()
    val some_baudrate = pserial_control.baudrate()
    val some_count = pserial_control.count()
    pserial_control.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun SERIAL_CONTROL.copyFrom(SRC: SERIAL_CONTROL) {
    SERIAL_CONTROL.push(
            SRC, { src -> device(src) }, { src -> flags(src) }, { src -> timeout(src) }, { src -> baudrate(src) }, { src -> count(src) }, { src ->
        val item = daTa()
        for (i in 0 until 70)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pserial_control: SERIAL_CONTROL) {

    pserial_control.device(some_SERIAL_CONTROL_DEV)

    pserial_control.flags(some_SERIAL_CONTROL_FLAG)
    pserial_control.timeout(Short)
    pserial_control.baudrate(Int)
    pserial_control.count(Byte)
    pserial_control.daTa().let { item ->
        pserial_control.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun SERIAL_CONTROL.copyInto(DST: SERIAL_CONTROL) {
    SERIAL_CONTROL.pull(
            DST, { device() != null }, { device()!!.get() }, { flags() != null }, { flags()!!.get() }, { -> timeout() }, { -> baudrate() }, { -> count() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 70)
            dst.set(src.get(i), i)
    }

    )
}

fun onSET_POSITION_TARGET_LOCAL_NED(pset_position_target_local_ned: SET_POSITION_TARGET_LOCAL_NED) {
    val some_time_boot_ms = pset_position_target_local_ned.time_boot_ms()
    val some_target_system = pset_position_target_local_ned.target_system()
    val some_target_component = pset_position_target_local_ned.target_component()

    pset_position_target_local_ned.coordinate_frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }
    val some_type_mask = pset_position_target_local_ned.type_mask()
    val some_x = pset_position_target_local_ned.x()
    val some_y = pset_position_target_local_ned.y()
    val some_z = pset_position_target_local_ned.z()
    val some_vx = pset_position_target_local_ned.vx()
    val some_vy = pset_position_target_local_ned.vy()
    val some_vz = pset_position_target_local_ned.vz()
    val some_afx = pset_position_target_local_ned.afx()
    val some_afy = pset_position_target_local_ned.afy()
    val some_afz = pset_position_target_local_ned.afz()
    val some_yaw = pset_position_target_local_ned.yaw()
    val some_yaw_rate = pset_position_target_local_ned.yaw_rate()

}

fun test_.SET_POSITION_TARGET_LOCAL_NED.copyFrom(SRC: SET_POSITION_TARGET_LOCAL_NED) {
    SET_POSITION_TARGET_LOCAL_NED.push(
            SRC, { src -> time_boot_ms(src) }, { src -> target_system(src) }, { src -> target_component(src) }, { src -> coordinate_frame(src) }, { src -> type_mask(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> afx(src) }, { src -> afy(src) }, { src -> afz(src) }, { src -> yaw(src) }, { src -> yaw_rate(src) }

    )
}

fun onMOUNT_ORIENTATION(pmount_orientation: MOUNT_ORIENTATION) {
    val some_time_boot_ms = pmount_orientation.time_boot_ms()
    val some_roll = pmount_orientation.roll()
    val some_pitch = pmount_orientation.pitch()
    val some_yaw = pmount_orientation.yaw()

}

fun MOUNT_ORIENTATION.copyFrom(SRC: MOUNT_ORIENTATION) {
    MOUNT_ORIENTATION.push(
            SRC, { src -> time_boot_ms(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }

    )
}

fun fill(pmount_orientation: MOUNT_ORIENTATION) {
    pmount_orientation.time_boot_ms(Int)
    pmount_orientation.roll(Float)
    pmount_orientation.pitch(Float)
    pmount_orientation.yaw(Float)

}

fun MOUNT_ORIENTATION.copyInto(DST: MOUNT_ORIENTATION) {
    MOUNT_ORIENTATION.pull(
            DST, { -> time_boot_ms() }, { -> roll() }, { -> pitch() }, { -> yaw() }

    )
}

fun onSET_GPS_GLOBAL_ORIGIN(pset_gps_global_origin: SET_GPS_GLOBAL_ORIGIN) {
    val some_target_system = pset_gps_global_origin.target_system()
    val some_latitude = pset_gps_global_origin.latitude()
    val some_longitude = pset_gps_global_origin.longitude()
    val some_altitude = pset_gps_global_origin.altitude()

    pset_gps_global_origin.time_usec().let { item -> println("Receive Long pack.") }

}

fun test_.SET_GPS_GLOBAL_ORIGIN.copyFrom(SRC: SET_GPS_GLOBAL_ORIGIN) {
    SET_GPS_GLOBAL_ORIGIN.push(
            SRC, { src -> target_system(src) }, { src -> latitude(src) }, { src -> longitude(src) }, { src -> altitude(src) }, { src -> time_usec(src) }

    )
}

fun onPARAM_EXT_SET(pparam_ext_set: PARAM_EXT_SET) {
    val some_target_system = pparam_ext_set.target_system()
    val some_target_component = pparam_ext_set.target_component()

    pparam_ext_set.param_id()?.let { item -> char = item.get() }

    pparam_ext_set.param_value()?.let { item -> char = item.get() }

    pparam_ext_set.param_type()?.let { item ->
        some_MAV_PARAM_EXT_TYPE = item.get()
    }

}

fun PARAM_EXT_SET.copyFrom(SRC: PARAM_EXT_SET) {
    PARAM_EXT_SET.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = param_id(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src ->
        val item = param_value(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> param_type(src) }

    )
}

fun fill(pparam_ext_set: PARAM_EXT_SET) {
    pparam_ext_set.target_system(Byte)
    pparam_ext_set.target_component(Byte)

    pparam_ext_set.param_id(some_String, null)

    pparam_ext_set.param_value(some_String, null)

    pparam_ext_set.param_type(some_MAV_PARAM_EXT_TYPE)

}

fun PARAM_EXT_SET.copyInto(DST: PARAM_EXT_SET) {
    PARAM_EXT_SET.pull(
            DST, { -> target_system() }, { -> target_component() }, { param_id()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = param_id()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { param_value()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = param_value()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { param_type() != null }, { param_type()!!.get() }

    )
}

fun onAUTOPILOT_VERSION(pautopilot_version: AUTOPILOT_VERSION) {

    pautopilot_version.capabilities()?.let { item ->
        some_MAV_PROTOCOL_CAPABILITY = item.get()
    }
    val some_flight_sw_version = pautopilot_version.flight_sw_version()
    val some_middleware_sw_version = pautopilot_version.middleware_sw_version()
    val some_os_sw_version = pautopilot_version.os_sw_version()
    val some_board_version = pautopilot_version.board_version()
    pautopilot_version.flight_custom_version().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    pautopilot_version.middleware_custom_version().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    pautopilot_version.os_custom_version().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    val some_vendor_id = pautopilot_version.vendor_id()
    val some_product_id = pautopilot_version.product_id()
    val some_uid = pautopilot_version.uid()
    pautopilot_version.uid2()?.let { fld ->
        fld.enumerate { d0 ->
            println("Receive Byte pack.")
        }
    }

}

fun AUTOPILOT_VERSION.copyFrom(SRC: AUTOPILOT_VERSION) {
    AUTOPILOT_VERSION.push(
            SRC, { src -> capabilities(src) }, { src -> flight_sw_version(src) }, { src -> middleware_sw_version(src) }, { src -> os_sw_version(src) }, { src -> board_version(src) }, { src ->
        val item = flight_custom_version()
        for (i in 0 until 8)
            item.set(src.get(i), i)
    }, { src ->
        val item = middleware_custom_version()
        for (i in 0 until 8)
            item.set(src.get(i), i)
    }, { src ->
        val item = os_custom_version()
        for (i in 0 until 8)
            item.set(src.get(i), i)
    }, { src -> vendor_id(src) }, { src -> product_id(src) }, { src -> uid(src) }, { src, d0 -> uid2(src, d0) }

    )
}

fun fill(pautopilot_version: AUTOPILOT_VERSION) {

    pautopilot_version.capabilities(some_MAV_PROTOCOL_CAPABILITY)
    pautopilot_version.flight_sw_version(Int)
    pautopilot_version.middleware_sw_version(Int)
    pautopilot_version.os_sw_version(Int)
    pautopilot_version.board_version(Int)
    pautopilot_version.flight_custom_version().let { item ->
        pautopilot_version.flight_custom_version().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    pautopilot_version.middleware_custom_version().let { item ->
        pautopilot_version.middleware_custom_version().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    pautopilot_version.os_custom_version().let { item ->
        pautopilot_version.os_custom_version().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    pautopilot_version.vendor_id(Short)
    pautopilot_version.product_id(Short)
    pautopilot_version.uid(Long)

    for (d0 in 0 until AUTOPILOT_VERSION.uid2.d0) {
        pautopilot_version.uid2(Byte, d0)
    }

}

fun AUTOPILOT_VERSION.copyInto(DST: AUTOPILOT_VERSION) {
    AUTOPILOT_VERSION.pull(
            DST, { capabilities() != null }, { capabilities()!!.get() }, { -> flight_sw_version() }, { -> middleware_sw_version() }, { -> os_sw_version() }, { -> board_version() }, { dst ->
        var src = this.flight_custom_version()
        for (i in 0 until 8)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.middleware_custom_version()
        for (i in 0 until 8)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.os_custom_version()
        for (i in 0 until 8)
            dst.set(src.get(i), i)
    }, { -> vendor_id() }, { -> product_id() }, { -> uid() }, { uid2() != null }, { d0 -> uid2()!!.get(d0) != null }, { d0 -> uid2()!!.get(d0).get() }

    )
}

fun onMISSION_REQUEST_LIST(pmission_request_list: MISSION_REQUEST_LIST) {
    val some_target_system = pmission_request_list.target_system()
    val some_target_component = pmission_request_list.target_component()

    pmission_request_list.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_REQUEST_LIST.copyFrom(SRC: MISSION_REQUEST_LIST) {
    MISSION_REQUEST_LIST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> mission_type(src) }

    )
}

fun onSIMSTATE(psimstate: SIMSTATE) {
    val some_roll = psimstate.roll()
    val some_pitch = psimstate.pitch()
    val some_yaw = psimstate.yaw()
    val some_xacc = psimstate.xacc()
    val some_yacc = psimstate.yacc()
    val some_zacc = psimstate.zacc()
    val some_xgyro = psimstate.xgyro()
    val some_ygyro = psimstate.ygyro()
    val some_zgyro = psimstate.zgyro()
    val some_lat = psimstate.lat()
    val some_lng = psimstate.lng()

}

fun SIMSTATE.copyFrom(SRC: SIMSTATE) {
    SIMSTATE.push(
            SRC, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }, { src -> xgyro(src) }, { src -> ygyro(src) }, { src -> zgyro(src) }, { src -> lat(src) }, { src -> lng(src) }

    )
}

fun fill(psimstate: SIMSTATE) {
    psimstate.roll(Float)
    psimstate.pitch(Float)
    psimstate.yaw(Float)
    psimstate.xacc(Float)
    psimstate.yacc(Float)
    psimstate.zacc(Float)
    psimstate.xgyro(Float)
    psimstate.ygyro(Float)
    psimstate.zgyro(Float)
    psimstate.lat(Int)
    psimstate.lng(Int)

}

fun SIMSTATE.copyInto(DST: SIMSTATE) {
    SIMSTATE.pull(
            DST, { -> roll() }, { -> pitch() }, { -> yaw() }, { -> xacc() }, { -> yacc() }, { -> zacc() }, { -> xgyro() }, { -> ygyro() }, { -> zgyro() }, { -> lat() }, { -> lng() }

    )
}

fun onSET_VIDEO_STREAM_SETTINGS(pset_video_stream_settings: SET_VIDEO_STREAM_SETTINGS) {
    val some_target_system = pset_video_stream_settings.target_system()
    val some_target_component = pset_video_stream_settings.target_component()
    val some_camera_id = pset_video_stream_settings.camera_id()
    val some_framerate = pset_video_stream_settings.framerate()
    val some_resolution_h = pset_video_stream_settings.resolution_h()
    val some_resolution_v = pset_video_stream_settings.resolution_v()
    val some_bitrate = pset_video_stream_settings.bitrate()
    val some_rotation = pset_video_stream_settings.rotation()

    pset_video_stream_settings.uri()?.let { item -> char = item.get() }

}

fun SET_VIDEO_STREAM_SETTINGS.copyFrom(SRC: SET_VIDEO_STREAM_SETTINGS) {
    SET_VIDEO_STREAM_SETTINGS.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> camera_id(src) }, { src -> framerate(src) }, { src -> resolution_h(src) }, { src -> resolution_v(src) }, { src -> bitrate(src) }, { src -> rotation(src) }, { src ->
        val item = uri(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun fill(pset_video_stream_settings: SET_VIDEO_STREAM_SETTINGS) {
    pset_video_stream_settings.target_system(Byte)
    pset_video_stream_settings.target_component(Byte)
    pset_video_stream_settings.camera_id(Byte)
    pset_video_stream_settings.framerate(Float)
    pset_video_stream_settings.resolution_h(Short)
    pset_video_stream_settings.resolution_v(Short)
    pset_video_stream_settings.bitrate(Int)
    pset_video_stream_settings.rotation(Short)

    pset_video_stream_settings.uri(some_String, null)

}

fun SET_VIDEO_STREAM_SETTINGS.copyInto(DST: SET_VIDEO_STREAM_SETTINGS) {
    SET_VIDEO_STREAM_SETTINGS.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> camera_id() }, { -> framerate() }, { -> resolution_h() }, { -> resolution_v() }, { -> bitrate() }, { -> rotation() }, { uri()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = uri()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }

    )
}

fun onPLAY_TUNE(pplay_tune: PLAY_TUNE) {
    val some_target_system = pplay_tune.target_system()
    val some_target_component = pplay_tune.target_component()

    pplay_tune.tune()?.let { item -> char = item.get() }

}

fun PLAY_TUNE.copyFrom(SRC: PLAY_TUNE) {
    PLAY_TUNE.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = tune(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun fill(pplay_tune: PLAY_TUNE) {
    pplay_tune.target_system(Byte)
    pplay_tune.target_component(Byte)

    pplay_tune.tune(some_String, null)

}

fun PLAY_TUNE.copyInto(DST: PLAY_TUNE) {
    PLAY_TUNE.pull(
            DST, { -> target_system() }, { -> target_component() }, { tune()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = tune()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }

    )
}

fun onDIGICAM_CONFIGURE(pdigicam_configure: DIGICAM_CONFIGURE) {
    val some_target_system = pdigicam_configure.target_system()
    val some_target_component = pdigicam_configure.target_component()
    val some_mode = pdigicam_configure.mode()
    val some_shutter_speed = pdigicam_configure.shutter_speed()
    val some_aperture = pdigicam_configure.aperture()
    val some_iso = pdigicam_configure.iso()
    val some_exposure_type = pdigicam_configure.exposure_type()
    val some_command_id = pdigicam_configure.command_id()
    val some_engine_cut_off = pdigicam_configure.engine_cut_off()
    val some_extra_param = pdigicam_configure.extra_param()
    val some_extra_value = pdigicam_configure.extra_value()

}

fun DIGICAM_CONFIGURE.copyFrom(SRC: DIGICAM_CONFIGURE) {
    DIGICAM_CONFIGURE.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> mode(src) }, { src -> shutter_speed(src) }, { src -> aperture(src) }, { src -> iso(src) }, { src -> exposure_type(src) }, { src -> command_id(src) }, { src -> engine_cut_off(src) }, { src -> extra_param(src) }, { src -> extra_value(src) }

    )
}

fun fill(pdigicam_configure: DIGICAM_CONFIGURE) {
    pdigicam_configure.target_system(Byte)
    pdigicam_configure.target_component(Byte)
    pdigicam_configure.mode(Byte)
    pdigicam_configure.shutter_speed(Short)
    pdigicam_configure.aperture(Byte)
    pdigicam_configure.iso(Byte)
    pdigicam_configure.exposure_type(Byte)
    pdigicam_configure.command_id(Byte)
    pdigicam_configure.engine_cut_off(Byte)
    pdigicam_configure.extra_param(Byte)
    pdigicam_configure.extra_value(Float)

}

fun DIGICAM_CONFIGURE.copyInto(DST: DIGICAM_CONFIGURE) {
    DIGICAM_CONFIGURE.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> mode() }, { -> shutter_speed() }, { -> aperture() }, { -> iso() }, { -> exposure_type() }, { -> command_id() }, { -> engine_cut_off() }, { -> extra_param() }, { -> extra_value() }

    )
}

fun onSCALED_PRESSURE3(pscaled_pressure3: SCALED_PRESSURE3) {
    val some_time_boot_ms = pscaled_pressure3.time_boot_ms()
    val some_press_abs = pscaled_pressure3.press_abs()
    val some_press_diff = pscaled_pressure3.press_diff()
    val some_temperature = pscaled_pressure3.temperature()

}

fun SCALED_PRESSURE3.copyFrom(SRC: SCALED_PRESSURE3) {
    SCALED_PRESSURE3.push(
            SRC, { src -> time_boot_ms(src) }, { src -> press_abs(src) }, { src -> press_diff(src) }, { src -> temperature(src) }

    )
}

fun fill(pscaled_pressure3: SCALED_PRESSURE3) {
    pscaled_pressure3.time_boot_ms(Int)
    pscaled_pressure3.press_abs(Float)
    pscaled_pressure3.press_diff(Float)
    pscaled_pressure3.temperature(Short)

}

fun SCALED_PRESSURE3.copyInto(DST: SCALED_PRESSURE3) {
    SCALED_PRESSURE3.pull(
            DST, { -> time_boot_ms() }, { -> press_abs() }, { -> press_diff() }, { -> temperature() }

    )
}

fun onMISSION_REQUEST_PARTIAL_LIST(pmission_request_partial_list: MISSION_REQUEST_PARTIAL_LIST) {
    val some_target_system = pmission_request_partial_list.target_system()
    val some_target_component = pmission_request_partial_list.target_component()
    val some_start_index = pmission_request_partial_list.start_index()
    val some_end_index = pmission_request_partial_list.end_index()

    pmission_request_partial_list.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_REQUEST_PARTIAL_LIST.copyFrom(SRC: MISSION_REQUEST_PARTIAL_LIST) {
    MISSION_REQUEST_PARTIAL_LIST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> start_index(src) }, { src -> end_index(src) }, { src -> mission_type(src) }

    )
}

fun onPARAM_EXT_ACK(pparam_ext_ack: PARAM_EXT_ACK) {

    pparam_ext_ack.param_id()?.let { item -> char = item.get() }

    pparam_ext_ack.param_value()?.let { item -> char = item.get() }

    pparam_ext_ack.param_type()?.let { item ->
        some_MAV_PARAM_EXT_TYPE = item.get()
    }

    pparam_ext_ack.param_result()?.let { item ->
        some_PARAM_ACK = item.get()
    }

}

fun PARAM_EXT_ACK.copyFrom(SRC: PARAM_EXT_ACK) {
    PARAM_EXT_ACK.push(
            SRC, { src ->
        val item = param_id(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src ->
        val item = param_value(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> param_type(src) }, { src -> param_result(src) }

    )
}

fun fill(pparam_ext_ack: PARAM_EXT_ACK) {

    pparam_ext_ack.param_id(some_String, null)

    pparam_ext_ack.param_value(some_String, null)

    pparam_ext_ack.param_type(some_MAV_PARAM_EXT_TYPE)

    pparam_ext_ack.param_result(some_PARAM_ACK)

}

fun PARAM_EXT_ACK.copyInto(DST: PARAM_EXT_ACK) {
    PARAM_EXT_ACK.pull(
            DST, { param_id()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = param_id()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { param_value()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = param_value()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { param_type() != null }, { param_type()!!.get() }, { param_result() != null }, { param_result()!!.get() }

    )
}

fun onUAVCAN_NODE_INFO(puavcan_node_info: UAVCAN_NODE_INFO) {
    val some_time_usec = puavcan_node_info.time_usec()
    val some_uptime_sec = puavcan_node_info.uptime_sec()

    puavcan_node_info.name()?.let { item -> char = item.get() }
    val some_hw_version_major = puavcan_node_info.hw_version_major()
    val some_hw_version_minor = puavcan_node_info.hw_version_minor()
    puavcan_node_info.hw_unique_id().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    val some_sw_version_major = puavcan_node_info.sw_version_major()
    val some_sw_version_minor = puavcan_node_info.sw_version_minor()
    val some_sw_vcs_commit = puavcan_node_info.sw_vcs_commit()

}

fun UAVCAN_NODE_INFO.copyFrom(SRC: UAVCAN_NODE_INFO) {
    UAVCAN_NODE_INFO.push(
            SRC, { src -> time_usec(src) }, { src -> uptime_sec(src) }, { src ->
        val item = name(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> hw_version_major(src) }, { src -> hw_version_minor(src) }, { src ->
        val item = hw_unique_id()
        for (i in 0 until 16)
            item.set(src.get(i), i)
    }, { src -> sw_version_major(src) }, { src -> sw_version_minor(src) }, { src -> sw_vcs_commit(src) }

    )
}

fun fill(puavcan_node_info: UAVCAN_NODE_INFO) {
    puavcan_node_info.time_usec(Long)
    puavcan_node_info.uptime_sec(Int)

    puavcan_node_info.name(some_String, null)
    puavcan_node_info.hw_version_major(Byte)
    puavcan_node_info.hw_version_minor(Byte)
    puavcan_node_info.hw_unique_id().let { item ->
        puavcan_node_info.hw_unique_id().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    puavcan_node_info.sw_version_major(Byte)
    puavcan_node_info.sw_version_minor(Byte)
    puavcan_node_info.sw_vcs_commit(Int)

}

fun UAVCAN_NODE_INFO.copyInto(DST: UAVCAN_NODE_INFO) {
    UAVCAN_NODE_INFO.pull(
            DST, { -> time_usec() }, { -> uptime_sec() }, { name()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = name()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { -> hw_version_major() }, { -> hw_version_minor() }, { dst ->
        var src = this.hw_unique_id()
        for (i in 0 until 16)
            dst.set(src.get(i), i)
    }, { -> sw_version_major() }, { -> sw_version_minor() }, { -> sw_vcs_commit() }

    )
}

fun onDATA16(pdata16: DATA16) {
    val some_typE = pdata16.typE()
    val some_len = pdata16.len()
    pdata16.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun DATA16.copyFrom(SRC: DATA16) {
    DATA16.push(
            SRC, { src -> typE(src) }, { src -> len(src) }, { src ->
        val item = daTa()
        for (i in 0 until 16)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pdata16: DATA16) {
    pdata16.typE(Byte)
    pdata16.len(Byte)
    pdata16.daTa().let { item ->
        pdata16.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun DATA16.copyInto(DST: DATA16) {
    DATA16.pull(
            DST, { -> typE() }, { -> len() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 16)
            dst.set(src.get(i), i)
    }

    )
}

fun onSET_MAG_OFFSETS(pset_mag_offsets: SET_MAG_OFFSETS) {
    val some_target_system = pset_mag_offsets.target_system()
    val some_target_component = pset_mag_offsets.target_component()
    val some_mag_ofs_x = pset_mag_offsets.mag_ofs_x()
    val some_mag_ofs_y = pset_mag_offsets.mag_ofs_y()
    val some_mag_ofs_z = pset_mag_offsets.mag_ofs_z()

}

fun SET_MAG_OFFSETS.copyFrom(SRC: SET_MAG_OFFSETS) {
    SET_MAG_OFFSETS.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> mag_ofs_x(src) }, { src -> mag_ofs_y(src) }, { src -> mag_ofs_z(src) }

    )
}

fun fill(pset_mag_offsets: SET_MAG_OFFSETS) {
    pset_mag_offsets.target_system(Byte)
    pset_mag_offsets.target_component(Byte)
    pset_mag_offsets.mag_ofs_x(Short)
    pset_mag_offsets.mag_ofs_y(Short)
    pset_mag_offsets.mag_ofs_z(Short)

}

fun SET_MAG_OFFSETS.copyInto(DST: SET_MAG_OFFSETS) {
    SET_MAG_OFFSETS.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> mag_ofs_x() }, { -> mag_ofs_y() }, { -> mag_ofs_z() }

    )
}

fun onAP_ADC(pap_adc: AP_ADC) {
    val some_adc1 = pap_adc.adc1()
    val some_adc2 = pap_adc.adc2()
    val some_adc3 = pap_adc.adc3()
    val some_adc4 = pap_adc.adc4()
    val some_adc5 = pap_adc.adc5()
    val some_adc6 = pap_adc.adc6()

}

fun AP_ADC.copyFrom(SRC: AP_ADC) {
    AP_ADC.push(
            SRC, { src -> adc1(src) }, { src -> adc2(src) }, { src -> adc3(src) }, { src -> adc4(src) }, { src -> adc5(src) }, { src -> adc6(src) }

    )
}

fun fill(pap_adc: AP_ADC) {
    pap_adc.adc1(Short)
    pap_adc.adc2(Short)
    pap_adc.adc3(Short)
    pap_adc.adc4(Short)
    pap_adc.adc5(Short)
    pap_adc.adc6(Short)

}

fun AP_ADC.copyInto(DST: AP_ADC) {
    AP_ADC.pull(
            DST, { -> adc1() }, { -> adc2() }, { -> adc3() }, { -> adc4() }, { -> adc5() }, { -> adc6() }

    )
}

fun onWIND(pwind: WIND) {
    val some_direction = pwind.direction()
    val some_speed = pwind.speed()
    val some_speed_z = pwind.speed_z()

}

fun WIND.copyFrom(SRC: WIND) {
    WIND.push(
            SRC, { src -> direction(src) }, { src -> speed(src) }, { src -> speed_z(src) }

    )
}

fun fill(pwind: WIND) {
    pwind.direction(Float)
    pwind.speed(Float)
    pwind.speed_z(Float)

}

fun WIND.copyInto(DST: WIND) {
    WIND.pull(
            DST, { -> direction() }, { -> speed() }, { -> speed_z() }

    )
}

fun onAUTOPILOT_VERSION_REQUEST(pautopilot_version_request: AUTOPILOT_VERSION_REQUEST) {
    val some_target_system = pautopilot_version_request.target_system()
    val some_target_component = pautopilot_version_request.target_component()

}

fun AUTOPILOT_VERSION_REQUEST.copyFrom(SRC: AUTOPILOT_VERSION_REQUEST) {
    AUTOPILOT_VERSION_REQUEST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }

    )
}

fun fill(pautopilot_version_request: AUTOPILOT_VERSION_REQUEST) {
    pautopilot_version_request.target_system(Byte)
    pautopilot_version_request.target_component(Byte)

}

fun AUTOPILOT_VERSION_REQUEST.copyInto(DST: AUTOPILOT_VERSION_REQUEST) {
    AUTOPILOT_VERSION_REQUEST.pull(
            DST, { -> target_system() }, { -> target_component() }

    )
}

fun onLOCAL_POSITION_NED(plocal_position_ned: LOCAL_POSITION_NED) {
    val some_time_boot_ms = plocal_position_ned.time_boot_ms()
    val some_x = plocal_position_ned.x()
    val some_y = plocal_position_ned.y()
    val some_z = plocal_position_ned.z()
    val some_vx = plocal_position_ned.vx()
    val some_vy = plocal_position_ned.vy()
    val some_vz = plocal_position_ned.vz()

}

fun test_.LOCAL_POSITION_NED.copyFrom(SRC: LOCAL_POSITION_NED) {
    LOCAL_POSITION_NED.push(
            SRC, { src -> time_boot_ms(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }

    )
}

fun onDATA_TRANSMISSION_HANDSHAKE(pdata_transmission_handshake: DATA_TRANSMISSION_HANDSHAKE) {
    val some_typE = pdata_transmission_handshake.typE()
    val some_size = pdata_transmission_handshake.size()
    val some_width = pdata_transmission_handshake.width()
    val some_height = pdata_transmission_handshake.height()
    val some_packets = pdata_transmission_handshake.packets()
    val some_payload = pdata_transmission_handshake.payload()
    val some_jpg_quality = pdata_transmission_handshake.jpg_quality()

}

fun DATA_TRANSMISSION_HANDSHAKE.copyFrom(SRC: DATA_TRANSMISSION_HANDSHAKE) {
    DATA_TRANSMISSION_HANDSHAKE.push(
            SRC, { src -> typE(src) }, { src -> size(src) }, { src -> width(src) }, { src -> height(src) }, { src -> packets(src) }, { src -> payload(src) }, { src -> jpg_quality(src) }

    )
}

fun fill(pdata_transmission_handshake: DATA_TRANSMISSION_HANDSHAKE) {
    pdata_transmission_handshake.typE(Byte)
    pdata_transmission_handshake.size(Int)
    pdata_transmission_handshake.width(Short)
    pdata_transmission_handshake.height(Short)
    pdata_transmission_handshake.packets(Short)
    pdata_transmission_handshake.payload(Byte)
    pdata_transmission_handshake.jpg_quality(Byte)

}

fun DATA_TRANSMISSION_HANDSHAKE.copyInto(DST: DATA_TRANSMISSION_HANDSHAKE) {
    DATA_TRANSMISSION_HANDSHAKE.pull(
            DST, { -> typE() }, { -> size() }, { -> width() }, { -> height() }, { -> packets() }, { -> payload() }, { -> jpg_quality() }

    )
}

fun onGPS_GLOBAL_ORIGIN(pgps_global_origin: GPS_GLOBAL_ORIGIN) {
    val some_latitude = pgps_global_origin.latitude()
    val some_longitude = pgps_global_origin.longitude()
    val some_altitude = pgps_global_origin.altitude()

    pgps_global_origin.time_usec().let { item -> println("Receive Long pack.") }

}

fun test_.GPS_GLOBAL_ORIGIN.copyFrom(SRC: GPS_GLOBAL_ORIGIN) {
    GPS_GLOBAL_ORIGIN.push(
            SRC, { src -> latitude(src) }, { src -> longitude(src) }, { src -> altitude(src) }, { src -> time_usec(src) }

    )
}

fun onSCALED_IMU2(pscaled_imu2: SCALED_IMU2) {
    val some_time_boot_ms = pscaled_imu2.time_boot_ms()
    val some_xacc = pscaled_imu2.xacc()
    val some_yacc = pscaled_imu2.yacc()
    val some_zacc = pscaled_imu2.zacc()
    val some_xgyro = pscaled_imu2.xgyro()
    val some_ygyro = pscaled_imu2.ygyro()
    val some_zgyro = pscaled_imu2.zgyro()
    val some_xmag = pscaled_imu2.xmag()
    val some_ymag = pscaled_imu2.ymag()
    val some_zmag = pscaled_imu2.zmag()

}

fun SCALED_IMU2.copyFrom(SRC: SCALED_IMU2) {
    SCALED_IMU2.push(
            SRC, { src -> time_boot_ms(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }, { src -> xgyro(src) }, { src -> ygyro(src) }, { src -> zgyro(src) }, { src -> xmag(src) }, { src -> ymag(src) }, { src -> zmag(src) }

    )
}

fun fill(pscaled_imu2: SCALED_IMU2) {
    pscaled_imu2.time_boot_ms(Int)
    pscaled_imu2.xacc(Short)
    pscaled_imu2.yacc(Short)
    pscaled_imu2.zacc(Short)
    pscaled_imu2.xgyro(Short)
    pscaled_imu2.ygyro(Short)
    pscaled_imu2.zgyro(Short)
    pscaled_imu2.xmag(Short)
    pscaled_imu2.ymag(Short)
    pscaled_imu2.zmag(Short)

}

fun SCALED_IMU2.copyInto(DST: SCALED_IMU2) {
    SCALED_IMU2.pull(
            DST, { -> time_boot_ms() }, { -> xacc() }, { -> yacc() }, { -> zacc() }, { -> xgyro() }, { -> ygyro() }, { -> zgyro() }, { -> xmag() }, { -> ymag() }, { -> zmag() }

    )
}

fun onATTITUDE_QUATERNION(pattitude_quaternion: ATTITUDE_QUATERNION) {
    val some_time_boot_ms = pattitude_quaternion.time_boot_ms()
    val some_q1 = pattitude_quaternion.q1()
    val some_q2 = pattitude_quaternion.q2()
    val some_q3 = pattitude_quaternion.q3()
    val some_q4 = pattitude_quaternion.q4()
    val some_rollspeed = pattitude_quaternion.rollspeed()
    val some_pitchspeed = pattitude_quaternion.pitchspeed()
    val some_yawspeed = pattitude_quaternion.yawspeed()

}

fun test_.ATTITUDE_QUATERNION.copyFrom(SRC: ATTITUDE_QUATERNION) {
    ATTITUDE_QUATERNION.push(
            SRC, { src -> time_boot_ms(src) }, { src -> q1(src) }, { src -> q2(src) }, { src -> q3(src) }, { src -> q4(src) }, { src -> rollspeed(src) }, { src -> pitchspeed(src) }, { src -> yawspeed(src) }

    )
}

fun onDATA64(pdata64: DATA64) {
    val some_typE = pdata64.typE()
    val some_len = pdata64.len()
    pdata64.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun DATA64.copyFrom(SRC: DATA64) {
    DATA64.push(
            SRC, { src -> typE(src) }, { src -> len(src) }, { src ->
        val item = daTa()
        for (i in 0 until 64)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pdata64: DATA64) {
    pdata64.typE(Byte)
    pdata64.len(Byte)
    pdata64.daTa().let { item ->
        pdata64.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun DATA64.copyInto(DST: DATA64) {
    DATA64.pull(
            DST, { -> typE() }, { -> len() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 64)
            dst.set(src.get(i), i)
    }

    )
}

fun onHIL_ACTUATOR_CONTROLS(phil_actuator_controls: HIL_ACTUATOR_CONTROLS) {
    val some_time_usec = phil_actuator_controls.time_usec()
    phil_actuator_controls.controls().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }

    phil_actuator_controls.mode()?.let { item ->
        some_MAV_MODE = item.get()
    }
    val some_flags = phil_actuator_controls.flags()

}

fun test_.HIL_ACTUATOR_CONTROLS.copyFrom(SRC: HIL_ACTUATOR_CONTROLS) {
    HIL_ACTUATOR_CONTROLS.push(
            SRC, { src -> time_usec(src) }, { src ->
        val item = controls()
        for (i in 0 until 16)
            item.set(src.get(i), i)
    }, { src -> mode(src) }, { src -> flags(src) }

    )
}

fun onPOSITION_TARGET_LOCAL_NED(pposition_target_local_ned: POSITION_TARGET_LOCAL_NED) {
    val some_time_boot_ms = pposition_target_local_ned.time_boot_ms()

    pposition_target_local_ned.coordinate_frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }
    val some_type_mask = pposition_target_local_ned.type_mask()
    val some_x = pposition_target_local_ned.x()
    val some_y = pposition_target_local_ned.y()
    val some_z = pposition_target_local_ned.z()
    val some_vx = pposition_target_local_ned.vx()
    val some_vy = pposition_target_local_ned.vy()
    val some_vz = pposition_target_local_ned.vz()
    val some_afx = pposition_target_local_ned.afx()
    val some_afy = pposition_target_local_ned.afy()
    val some_afz = pposition_target_local_ned.afz()
    val some_yaw = pposition_target_local_ned.yaw()
    val some_yaw_rate = pposition_target_local_ned.yaw_rate()

}

fun test_.POSITION_TARGET_LOCAL_NED.copyFrom(SRC: POSITION_TARGET_LOCAL_NED) {
    POSITION_TARGET_LOCAL_NED.push(
            SRC, { src -> time_boot_ms(src) }, { src -> coordinate_frame(src) }, { src -> type_mask(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> vx(src) }, { src -> vy(src) }, { src -> vz(src) }, { src -> afx(src) }, { src -> afy(src) }, { src -> afz(src) }, { src -> yaw(src) }, { src -> yaw_rate(src) }

    )
}

fun onGIMBAL_REPORT(pgimbal_report: GIMBAL_REPORT) {
    val some_target_system = pgimbal_report.target_system()
    val some_target_component = pgimbal_report.target_component()
    val some_delta_time = pgimbal_report.delta_time()
    val some_delta_angle_x = pgimbal_report.delta_angle_x()
    val some_delta_angle_y = pgimbal_report.delta_angle_y()
    val some_delta_angle_z = pgimbal_report.delta_angle_z()
    val some_delta_velocity_x = pgimbal_report.delta_velocity_x()
    val some_delta_velocity_y = pgimbal_report.delta_velocity_y()
    val some_delta_velocity_z = pgimbal_report.delta_velocity_z()
    val some_joint_roll = pgimbal_report.joint_roll()
    val some_joint_el = pgimbal_report.joint_el()
    val some_joint_az = pgimbal_report.joint_az()

}

fun GIMBAL_REPORT.copyFrom(SRC: GIMBAL_REPORT) {
    GIMBAL_REPORT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> delta_time(src) }, { src -> delta_angle_x(src) }, { src -> delta_angle_y(src) }, { src -> delta_angle_z(src) }, { src -> delta_velocity_x(src) }, { src -> delta_velocity_y(src) }, { src -> delta_velocity_z(src) }, { src -> joint_roll(src) }, { src -> joint_el(src) }, { src -> joint_az(src) }

    )
}

fun fill(pgimbal_report: GIMBAL_REPORT) {
    pgimbal_report.target_system(Byte)
    pgimbal_report.target_component(Byte)
    pgimbal_report.delta_time(Float)
    pgimbal_report.delta_angle_x(Float)
    pgimbal_report.delta_angle_y(Float)
    pgimbal_report.delta_angle_z(Float)
    pgimbal_report.delta_velocity_x(Float)
    pgimbal_report.delta_velocity_y(Float)
    pgimbal_report.delta_velocity_z(Float)
    pgimbal_report.joint_roll(Float)
    pgimbal_report.joint_el(Float)
    pgimbal_report.joint_az(Float)

}

fun GIMBAL_REPORT.copyInto(DST: GIMBAL_REPORT) {
    GIMBAL_REPORT.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> delta_time() }, { -> delta_angle_x() }, { -> delta_angle_y() }, { -> delta_angle_z() }, { -> delta_velocity_x() }, { -> delta_velocity_y() }, { -> delta_velocity_z() }, { -> joint_roll() }, { -> joint_el() }, { -> joint_az() }

    )
}

fun onDEVICE_OP_WRITE(pdevice_op_write: DEVICE_OP_WRITE) {
    val some_target_system = pdevice_op_write.target_system()
    val some_target_component = pdevice_op_write.target_component()
    val some_request_id = pdevice_op_write.request_id()

    pdevice_op_write.bustype()?.let { item ->
        some_DEVICE_OP_BUSTYPE = item.get()
    }
    val some_bus = pdevice_op_write.bus()
    val some_address = pdevice_op_write.address()

    pdevice_op_write.busname()?.let { item -> char = item.get() }
    val some_regstart = pdevice_op_write.regstart()
    val some_count = pdevice_op_write.count()
    pdevice_op_write.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun DEVICE_OP_WRITE.copyFrom(SRC: DEVICE_OP_WRITE) {
    DEVICE_OP_WRITE.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> request_id(src) }, { src -> bustype(src) }, { src -> bus(src) }, { src -> address(src) }, { src ->
        val item = busname(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> regstart(src) }, { src -> count(src) }, { src ->
        val item = daTa()
        for (i in 0 until 128)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pdevice_op_write: DEVICE_OP_WRITE) {
    pdevice_op_write.target_system(Byte)
    pdevice_op_write.target_component(Byte)
    pdevice_op_write.request_id(Int)

    pdevice_op_write.bustype(some_DEVICE_OP_BUSTYPE)
    pdevice_op_write.bus(Byte)
    pdevice_op_write.address(Byte)

    pdevice_op_write.busname(some_String, null)
    pdevice_op_write.regstart(Byte)
    pdevice_op_write.count(Byte)
    pdevice_op_write.daTa().let { item ->
        pdevice_op_write.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun DEVICE_OP_WRITE.copyInto(DST: DEVICE_OP_WRITE) {
    DEVICE_OP_WRITE.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> request_id() }, { bustype() != null }, { bustype()!!.get() }, { -> bus() }, { -> address() }, { busname()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = busname()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { -> regstart() }, { -> count() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 128)
            dst.set(src.get(i), i)
    }

    )
}

fun onDISTANCE_SENSOR(pdistance_sensor: DISTANCE_SENSOR) {
    val some_time_boot_ms = pdistance_sensor.time_boot_ms()
    val some_min_distance = pdistance_sensor.min_distance()
    val some_max_distance = pdistance_sensor.max_distance()
    val some_current_distance = pdistance_sensor.current_distance()

    pdistance_sensor.typE()?.let { item ->
        some_MAV_DISTANCE_SENSOR = item.get()
    }
    val some_id = pdistance_sensor.id()

    pdistance_sensor.orientation()?.let { item ->
        some_MAV_SENSOR_ORIENTATION = item.get()
    }
    val some_covariance = pdistance_sensor.covariance()

}

fun DISTANCE_SENSOR.copyFrom(SRC: DISTANCE_SENSOR) {
    DISTANCE_SENSOR.push(
            SRC, { src -> time_boot_ms(src) }, { src -> min_distance(src) }, { src -> max_distance(src) }, { src -> current_distance(src) }, { src -> typE(src) }, { src -> id(src) }, { src -> orientation(src) }, { src -> covariance(src) }

    )
}

fun fill(pdistance_sensor: DISTANCE_SENSOR) {
    pdistance_sensor.time_boot_ms(Int)
    pdistance_sensor.min_distance(Short)
    pdistance_sensor.max_distance(Short)
    pdistance_sensor.current_distance(Short)

    pdistance_sensor.typE(some_MAV_DISTANCE_SENSOR)
    pdistance_sensor.id(Byte)

    pdistance_sensor.orientation(some_MAV_SENSOR_ORIENTATION)
    pdistance_sensor.covariance(Byte)

}

fun DISTANCE_SENSOR.copyInto(DST: DISTANCE_SENSOR) {
    DISTANCE_SENSOR.pull(
            DST, { -> time_boot_ms() }, { -> min_distance() }, { -> max_distance() }, { -> current_distance() }, { typE() != null }, { typE()!!.get() }, { -> id() }, { orientation() != null }, { orientation()!!.get() }, { -> covariance() }

    )
}

fun onHIL_OPTICAL_FLOW(phil_optical_flow: HIL_OPTICAL_FLOW) {
    val some_time_usec = phil_optical_flow.time_usec()
    val some_sensor_id = phil_optical_flow.sensor_id()
    val some_integration_time_us = phil_optical_flow.integration_time_us()
    val some_integrated_x = phil_optical_flow.integrated_x()
    val some_integrated_y = phil_optical_flow.integrated_y()
    val some_integrated_xgyro = phil_optical_flow.integrated_xgyro()
    val some_integrated_ygyro = phil_optical_flow.integrated_ygyro()
    val some_integrated_zgyro = phil_optical_flow.integrated_zgyro()
    val some_temperature = phil_optical_flow.temperature()
    val some_quality = phil_optical_flow.quality()
    val some_time_delta_distance_us = phil_optical_flow.time_delta_distance_us()
    val some_distance = phil_optical_flow.distance()

}

fun HIL_OPTICAL_FLOW.copyFrom(SRC: HIL_OPTICAL_FLOW) {
    HIL_OPTICAL_FLOW.push(
            SRC, { src -> time_usec(src) }, { src -> sensor_id(src) }, { src -> integration_time_us(src) }, { src -> integrated_x(src) }, { src -> integrated_y(src) }, { src -> integrated_xgyro(src) }, { src -> integrated_ygyro(src) }, { src -> integrated_zgyro(src) }, { src -> temperature(src) }, { src -> quality(src) }, { src -> time_delta_distance_us(src) }, { src -> distance(src) }

    )
}

fun fill(phil_optical_flow: HIL_OPTICAL_FLOW) {
    phil_optical_flow.time_usec(Long)
    phil_optical_flow.sensor_id(Byte)
    phil_optical_flow.integration_time_us(Int)
    phil_optical_flow.integrated_x(Float)
    phil_optical_flow.integrated_y(Float)
    phil_optical_flow.integrated_xgyro(Float)
    phil_optical_flow.integrated_ygyro(Float)
    phil_optical_flow.integrated_zgyro(Float)
    phil_optical_flow.temperature(Short)
    phil_optical_flow.quality(Byte)
    phil_optical_flow.time_delta_distance_us(Int)
    phil_optical_flow.distance(Float)

}

fun HIL_OPTICAL_FLOW.copyInto(DST: HIL_OPTICAL_FLOW) {
    HIL_OPTICAL_FLOW.pull(
            DST, { -> time_usec() }, { -> sensor_id() }, { -> integration_time_us() }, { -> integrated_x() }, { -> integrated_y() }, { -> integrated_xgyro() }, { -> integrated_ygyro() }, { -> integrated_zgyro() }, { -> temperature() }, { -> quality() }, { -> time_delta_distance_us() }, { -> distance() }

    )
}

fun onSCALED_PRESSURE2(pscaled_pressure2: SCALED_PRESSURE2) {
    val some_time_boot_ms = pscaled_pressure2.time_boot_ms()
    val some_press_abs = pscaled_pressure2.press_abs()
    val some_press_diff = pscaled_pressure2.press_diff()
    val some_temperature = pscaled_pressure2.temperature()

}

fun SCALED_PRESSURE2.copyFrom(SRC: SCALED_PRESSURE2) {
    SCALED_PRESSURE2.push(
            SRC, { src -> time_boot_ms(src) }, { src -> press_abs(src) }, { src -> press_diff(src) }, { src -> temperature(src) }

    )
}

fun fill(pscaled_pressure2: SCALED_PRESSURE2) {
    pscaled_pressure2.time_boot_ms(Int)
    pscaled_pressure2.press_abs(Float)
    pscaled_pressure2.press_diff(Float)
    pscaled_pressure2.temperature(Short)

}

fun SCALED_PRESSURE2.copyInto(DST: SCALED_PRESSURE2) {
    SCALED_PRESSURE2.pull(
            DST, { -> time_boot_ms() }, { -> press_abs() }, { -> press_diff() }, { -> temperature() }

    )
}

fun onWIND_COV(pwind_cov: WIND_COV) {
    val some_time_usec = pwind_cov.time_usec()
    val some_wind_x = pwind_cov.wind_x()
    val some_wind_y = pwind_cov.wind_y()
    val some_wind_z = pwind_cov.wind_z()
    val some_var_horiz = pwind_cov.var_horiz()
    val some_var_vert = pwind_cov.var_vert()
    val some_wind_alt = pwind_cov.wind_alt()
    val some_horiz_accuracy = pwind_cov.horiz_accuracy()
    val some_vert_accuracy = pwind_cov.vert_accuracy()

}

fun WIND_COV.copyFrom(SRC: WIND_COV) {
    WIND_COV.push(
            SRC, { src -> time_usec(src) }, { src -> wind_x(src) }, { src -> wind_y(src) }, { src -> wind_z(src) }, { src -> var_horiz(src) }, { src -> var_vert(src) }, { src -> wind_alt(src) }, { src -> horiz_accuracy(src) }, { src -> vert_accuracy(src) }

    )
}

fun fill(pwind_cov: WIND_COV) {
    pwind_cov.time_usec(Long)
    pwind_cov.wind_x(Float)
    pwind_cov.wind_y(Float)
    pwind_cov.wind_z(Float)
    pwind_cov.var_horiz(Float)
    pwind_cov.var_vert(Float)
    pwind_cov.wind_alt(Float)
    pwind_cov.horiz_accuracy(Float)
    pwind_cov.vert_accuracy(Float)

}

fun WIND_COV.copyInto(DST: WIND_COV) {
    WIND_COV.pull(
            DST, { -> time_usec() }, { -> wind_x() }, { -> wind_y() }, { -> wind_z() }, { -> var_horiz() }, { -> var_vert() }, { -> wind_alt() }, { -> horiz_accuracy() }, { -> vert_accuracy() }

    )
}

fun onCHANGE_OPERATOR_CONTROL(pchange_operator_control: CHANGE_OPERATOR_CONTROL) {
    val some_target_system = pchange_operator_control.target_system()
    val some_control_request = pchange_operator_control.control_request()
    val some_version = pchange_operator_control.version()

    pchange_operator_control.passkey()?.let { item -> char = item.get() }

}

fun test_.CHANGE_OPERATOR_CONTROL.copyFrom(SRC: CHANGE_OPERATOR_CONTROL) {
    CHANGE_OPERATOR_CONTROL.push(
            SRC, { src -> target_system(src) }, { src -> control_request(src) }, { src -> version(src) }, { src ->
        val item = passkey(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }

    )
}

fun onGOPRO_SET_REQUEST(pgopro_set_request: GOPRO_SET_REQUEST) {
    val some_target_system = pgopro_set_request.target_system()
    val some_target_component = pgopro_set_request.target_component()

    pgopro_set_request.cmd_id()?.let { item ->
        some_GOPRO_COMMAND = item.get()
    }
    pgopro_set_request.value().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun GOPRO_SET_REQUEST.copyFrom(SRC: GOPRO_SET_REQUEST) {
    GOPRO_SET_REQUEST.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> cmd_id(src) }, { src ->
        val item = value()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pgopro_set_request: GOPRO_SET_REQUEST) {
    pgopro_set_request.target_system(Byte)
    pgopro_set_request.target_component(Byte)

    pgopro_set_request.cmd_id(some_GOPRO_COMMAND)
    pgopro_set_request.value().let { item ->
        pgopro_set_request.value().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun GOPRO_SET_REQUEST.copyInto(DST: GOPRO_SET_REQUEST) {
    GOPRO_SET_REQUEST.pull(
            DST, { -> target_system() }, { -> target_component() }, { cmd_id() != null }, { cmd_id()!!.get() }, { dst ->
        var src = this.value()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }

    )
}

fun onSYS_STATUS(psys_status: SYS_STATUS) {

    psys_status.onboard_control_sensors_present()?.let { item ->
        some_MAV_SYS_STATUS_SENSOR = item.get()
    }

    psys_status.onboard_control_sensors_enabled()?.let { item ->
        some_MAV_SYS_STATUS_SENSOR = item.get()
    }

    psys_status.onboard_control_sensors_health()?.let { item ->
        some_MAV_SYS_STATUS_SENSOR = item.get()
    }
    val some_load = psys_status.load()
    val some_voltage_battery = psys_status.voltage_battery()
    val some_current_battery = psys_status.current_battery()
    val some_battery_remaining = psys_status.battery_remaining()
    val some_drop_rate_comm = psys_status.drop_rate_comm()
    val some_errors_comm = psys_status.errors_comm()
    val some_errors_count1 = psys_status.errors_count1()
    val some_errors_count2 = psys_status.errors_count2()
    val some_errors_count3 = psys_status.errors_count3()
    val some_errors_count4 = psys_status.errors_count4()

}

fun test_.SYS_STATUS.copyFrom(SRC: SYS_STATUS) {
    SYS_STATUS.push(
            SRC, { src -> onboard_control_sensors_present(src) }, { src -> onboard_control_sensors_enabled(src) }, { src -> onboard_control_sensors_health(src) }, { src -> load(src) }, { src -> voltage_battery(src) }, { src -> current_battery(src) }, { src -> battery_remaining(src) }, { src -> drop_rate_comm(src) }, { src -> errors_comm(src) }, { src -> errors_count1(src) }, { src -> errors_count2(src) }, { src -> errors_count3(src) }, { src -> errors_count4(src) }

    )
}

fun onMISSION_ITEM(pmission_item: MISSION_ITEM) {
    val some_target_system = pmission_item.target_system()
    val some_target_component = pmission_item.target_component()
    val some_seq = pmission_item.seq()

    pmission_item.frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }

    pmission_item.command()?.let { item ->
        some_MAV_CMD = item.get()
    }
    val some_current = pmission_item.current()
    val some_autocontinue = pmission_item.autocontinue()
    val some_param1 = pmission_item.param1()
    val some_param2 = pmission_item.param2()
    val some_param3 = pmission_item.param3()
    val some_param4 = pmission_item.param4()
    val some_x = pmission_item.x()
    val some_y = pmission_item.y()
    val some_z = pmission_item.z()

    pmission_item.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_ITEM.copyFrom(SRC: MISSION_ITEM) {
    MISSION_ITEM.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> seq(src) }, { src -> frame(src) }, { src -> command(src) }, { src -> current(src) }, { src -> autocontinue(src) }, { src -> param1(src) }, { src -> param2(src) }, { src -> param3(src) }, { src -> param4(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> mission_type(src) }

    )
}

fun onRAW_IMU(praw_imu: RAW_IMU) {
    val some_time_usec = praw_imu.time_usec()
    val some_xacc = praw_imu.xacc()
    val some_yacc = praw_imu.yacc()
    val some_zacc = praw_imu.zacc()
    val some_xgyro = praw_imu.xgyro()
    val some_ygyro = praw_imu.ygyro()
    val some_zgyro = praw_imu.zgyro()
    val some_xmag = praw_imu.xmag()
    val some_ymag = praw_imu.ymag()
    val some_zmag = praw_imu.zmag()

}

fun test_.RAW_IMU.copyFrom(SRC: RAW_IMU) {
    RAW_IMU.push(
            SRC, { src -> time_usec(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }, { src -> xgyro(src) }, { src -> ygyro(src) }, { src -> zgyro(src) }, { src -> xmag(src) }, { src -> ymag(src) }, { src -> zmag(src) }

    )
}

fun onCOMMAND_INT(pcommand_int: COMMAND_INT) {
    val some_target_system = pcommand_int.target_system()
    val some_target_component = pcommand_int.target_component()

    pcommand_int.frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }

    pcommand_int.command()?.let { item ->
        some_MAV_CMD = item.get()
    }
    val some_current = pcommand_int.current()
    val some_autocontinue = pcommand_int.autocontinue()
    val some_param1 = pcommand_int.param1()
    val some_param2 = pcommand_int.param2()
    val some_param3 = pcommand_int.param3()
    val some_param4 = pcommand_int.param4()
    val some_x = pcommand_int.x()
    val some_y = pcommand_int.y()
    val some_z = pcommand_int.z()

}

fun test_.COMMAND_INT.copyFrom(SRC: COMMAND_INT) {
    COMMAND_INT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> frame(src) }, { src -> command(src) }, { src -> current(src) }, { src -> autocontinue(src) }, { src -> param1(src) }, { src -> param2(src) }, { src -> param3(src) }, { src -> param4(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }

    )
}

fun onOPTICAL_FLOW(poptical_flow: OPTICAL_FLOW) {
    val some_time_usec = poptical_flow.time_usec()
    val some_sensor_id = poptical_flow.sensor_id()
    val some_flow_x = poptical_flow.flow_x()
    val some_flow_y = poptical_flow.flow_y()
    val some_flow_comp_m_x = poptical_flow.flow_comp_m_x()
    val some_flow_comp_m_y = poptical_flow.flow_comp_m_y()
    val some_quality = poptical_flow.quality()
    val some_ground_distance = poptical_flow.ground_distance()

    poptical_flow.flow_rate_x().let { item -> println("Receive Float pack.") }

    poptical_flow.flow_rate_y().let { item -> println("Receive Float pack.") }

}

fun test_.OPTICAL_FLOW.copyFrom(SRC: OPTICAL_FLOW) {
    OPTICAL_FLOW.push(
            SRC, { src -> time_usec(src) }, { src -> sensor_id(src) }, { src -> flow_x(src) }, { src -> flow_y(src) }, { src -> flow_comp_m_x(src) }, { src -> flow_comp_m_y(src) }, { src -> quality(src) }, { src -> ground_distance(src) }, { src -> flow_rate_x(src) }, { src -> flow_rate_y(src) }

    )
}

fun onMISSION_ITEM_INT(pmission_item_int: MISSION_ITEM_INT) {
    val some_target_system = pmission_item_int.target_system()
    val some_target_component = pmission_item_int.target_component()
    val some_seq = pmission_item_int.seq()

    pmission_item_int.frame()?.let { item ->
        some_MAV_FRAME = item.get()
    }

    pmission_item_int.command()?.let { item ->
        some_MAV_CMD = item.get()
    }
    val some_current = pmission_item_int.current()
    val some_autocontinue = pmission_item_int.autocontinue()
    val some_param1 = pmission_item_int.param1()
    val some_param2 = pmission_item_int.param2()
    val some_param3 = pmission_item_int.param3()
    val some_param4 = pmission_item_int.param4()
    val some_x = pmission_item_int.x()
    val some_y = pmission_item_int.y()
    val some_z = pmission_item_int.z()

    pmission_item_int.mission_type()?.let { item ->
        some_MAV_MISSION_TYPE = item.get()
    }

}

fun test_.MISSION_ITEM_INT.copyFrom(SRC: MISSION_ITEM_INT) {
    MISSION_ITEM_INT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> seq(src) }, { src -> frame(src) }, { src -> command(src) }, { src -> current(src) }, { src -> autocontinue(src) }, { src -> param1(src) }, { src -> param2(src) }, { src -> param3(src) }, { src -> param4(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> mission_type(src) }

    )
}

fun onVISION_POSITION_DELTA(pvision_position_delta: VISION_POSITION_DELTA) {
    val some_time_usec = pvision_position_delta.time_usec()
    val some_time_delta_usec = pvision_position_delta.time_delta_usec()
    pvision_position_delta.angle_delta().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    pvision_position_delta.position_delta().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_confidence = pvision_position_delta.confidence()

}

fun VISION_POSITION_DELTA.copyFrom(SRC: VISION_POSITION_DELTA) {
    VISION_POSITION_DELTA.push(
            SRC, { src -> time_usec(src) }, { src -> time_delta_usec(src) }, { src ->
        val item = angle_delta()
        for (i in 0 until 3)
            item.set(src.get(i), i)
    }, { src ->
        val item = position_delta()
        for (i in 0 until 3)
            item.set(src.get(i), i)
    }, { src -> confidence(src) }

    )
}

fun fill(pvision_position_delta: VISION_POSITION_DELTA) {
    pvision_position_delta.time_usec(Long)
    pvision_position_delta.time_delta_usec(Long)
    pvision_position_delta.angle_delta().let { item ->
        pvision_position_delta.angle_delta().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pvision_position_delta.position_delta().let { item ->
        pvision_position_delta.position_delta().let { item ->
            for (index in 0 until item.len())
                item.set(Float, index)
        }
    }
    pvision_position_delta.confidence(Float)

}

fun VISION_POSITION_DELTA.copyInto(DST: VISION_POSITION_DELTA) {
    VISION_POSITION_DELTA.pull(
            DST, { -> time_usec() }, { -> time_delta_usec() }, { dst ->
        var src = this.angle_delta()
        for (i in 0 until 3)
            dst.set(src.get(i), i)
    }, { dst ->
        var src = this.position_delta()
        for (i in 0 until 3)
            dst.set(src.get(i), i)
    }, { -> confidence() }

    )
}

fun onLOGGING_DATA(plogging_data: LOGGING_DATA) {
    val some_target_system = plogging_data.target_system()
    val some_target_component = plogging_data.target_component()
    val some_sequence = plogging_data.sequence()
    val some_length = plogging_data.length()
    val some_first_message_offset = plogging_data.first_message_offset()
    plogging_data.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun LOGGING_DATA.copyFrom(SRC: LOGGING_DATA) {
    LOGGING_DATA.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> sequence(src) }, { src -> length(src) }, { src -> first_message_offset(src) }, { src ->
        val item = daTa()
        for (i in 0 until 249)
            item.set(src.get(i), i)
    }

    )
}

fun fill(plogging_data: LOGGING_DATA) {
    plogging_data.target_system(Byte)
    plogging_data.target_component(Byte)
    plogging_data.sequence(Short)
    plogging_data.length(Byte)
    plogging_data.first_message_offset(Byte)
    plogging_data.daTa().let { item ->
        plogging_data.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun LOGGING_DATA.copyInto(DST: LOGGING_DATA) {
    LOGGING_DATA.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> sequence() }, { -> length() }, { -> first_message_offset() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 249)
            dst.set(src.get(i), i)
    }

    )
}

fun onDEVICE_OP_READ(pdevice_op_read: DEVICE_OP_READ) {
    val some_target_system = pdevice_op_read.target_system()
    val some_target_component = pdevice_op_read.target_component()
    val some_request_id = pdevice_op_read.request_id()

    pdevice_op_read.bustype()?.let { item ->
        some_DEVICE_OP_BUSTYPE = item.get()
    }
    val some_bus = pdevice_op_read.bus()
    val some_address = pdevice_op_read.address()

    pdevice_op_read.busname()?.let { item -> char = item.get() }
    val some_regstart = pdevice_op_read.regstart()
    val some_count = pdevice_op_read.count()

}

fun DEVICE_OP_READ.copyFrom(SRC: DEVICE_OP_READ) {
    DEVICE_OP_READ.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> request_id(src) }, { src -> bustype(src) }, { src -> bus(src) }, { src -> address(src) }, { src ->
        val item = busname(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> regstart(src) }, { src -> count(src) }

    )
}

fun fill(pdevice_op_read: DEVICE_OP_READ) {
    pdevice_op_read.target_system(Byte)
    pdevice_op_read.target_component(Byte)
    pdevice_op_read.request_id(Int)

    pdevice_op_read.bustype(some_DEVICE_OP_BUSTYPE)
    pdevice_op_read.bus(Byte)
    pdevice_op_read.address(Byte)

    pdevice_op_read.busname(some_String, null)
    pdevice_op_read.regstart(Byte)
    pdevice_op_read.count(Byte)

}

fun DEVICE_OP_READ.copyInto(DST: DEVICE_OP_READ) {
    DEVICE_OP_READ.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> request_id() }, { bustype() != null }, { bustype()!!.get() }, { -> bus() }, { -> address() }, { busname()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = busname()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { -> regstart() }, { -> count() }

    )
}

fun onMAG_CAL_PROGRESS(pmag_cal_progress: MAG_CAL_PROGRESS) {
    val some_compass_id = pmag_cal_progress.compass_id()
    val some_cal_mask = pmag_cal_progress.cal_mask()

    pmag_cal_progress.cal_status()?.let { item ->
        some_MAG_CAL_STATUS = item.get()
    }
    val some_attempt = pmag_cal_progress.attempt()
    val some_completion_pct = pmag_cal_progress.completion_pct()
    pmag_cal_progress.completion_mask().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    val some_direction_x = pmag_cal_progress.direction_x()
    val some_direction_y = pmag_cal_progress.direction_y()
    val some_direction_z = pmag_cal_progress.direction_z()

}

fun MAG_CAL_PROGRESS.copyFrom(SRC: MAG_CAL_PROGRESS) {
    MAG_CAL_PROGRESS.push(
            SRC, { src -> compass_id(src) }, { src -> cal_mask(src) }, { src -> cal_status(src) }, { src -> attempt(src) }, { src -> completion_pct(src) }, { src ->
        val item = completion_mask()
        for (i in 0 until 10)
            item.set(src.get(i), i)
    }, { src -> direction_x(src) }, { src -> direction_y(src) }, { src -> direction_z(src) }

    )
}

fun fill(pmag_cal_progress: MAG_CAL_PROGRESS) {
    pmag_cal_progress.compass_id(Byte)
    pmag_cal_progress.cal_mask(Byte)

    pmag_cal_progress.cal_status(some_MAG_CAL_STATUS)
    pmag_cal_progress.attempt(Byte)
    pmag_cal_progress.completion_pct(Byte)
    pmag_cal_progress.completion_mask().let { item ->
        pmag_cal_progress.completion_mask().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    pmag_cal_progress.direction_x(Float)
    pmag_cal_progress.direction_y(Float)
    pmag_cal_progress.direction_z(Float)

}

fun MAG_CAL_PROGRESS.copyInto(DST: MAG_CAL_PROGRESS) {
    MAG_CAL_PROGRESS.pull(
            DST, { -> compass_id() }, { -> cal_mask() }, { cal_status() != null }, { cal_status()!!.get() }, { -> attempt() }, { -> completion_pct() }, { dst ->
        var src = this.completion_mask()
        for (i in 0 until 10)
            dst.set(src.get(i), i)
    }, { -> direction_x() }, { -> direction_y() }, { -> direction_z() }

    )
}

fun onHIGHRES_IMU(phighres_imu: HIGHRES_IMU) {
    val some_time_usec = phighres_imu.time_usec()
    val some_xacc = phighres_imu.xacc()
    val some_yacc = phighres_imu.yacc()
    val some_zacc = phighres_imu.zacc()
    val some_xgyro = phighres_imu.xgyro()
    val some_ygyro = phighres_imu.ygyro()
    val some_zgyro = phighres_imu.zgyro()
    val some_xmag = phighres_imu.xmag()
    val some_ymag = phighres_imu.ymag()
    val some_zmag = phighres_imu.zmag()
    val some_abs_pressure = phighres_imu.abs_pressure()
    val some_diff_pressure = phighres_imu.diff_pressure()
    val some_pressure_alt = phighres_imu.pressure_alt()
    val some_temperature = phighres_imu.temperature()
    val some_fields_updated = phighres_imu.fields_updated()

}

fun HIGHRES_IMU.copyFrom(SRC: HIGHRES_IMU) {
    HIGHRES_IMU.push(
            SRC, { src -> time_usec(src) }, { src -> xacc(src) }, { src -> yacc(src) }, { src -> zacc(src) }, { src -> xgyro(src) }, { src -> ygyro(src) }, { src -> zgyro(src) }, { src -> xmag(src) }, { src -> ymag(src) }, { src -> zmag(src) }, { src -> abs_pressure(src) }, { src -> diff_pressure(src) }, { src -> pressure_alt(src) }, { src -> temperature(src) }, { src -> fields_updated(src) }

    )
}

fun fill(phighres_imu: HIGHRES_IMU) {
    phighres_imu.time_usec(Long)
    phighres_imu.xacc(Float)
    phighres_imu.yacc(Float)
    phighres_imu.zacc(Float)
    phighres_imu.xgyro(Float)
    phighres_imu.ygyro(Float)
    phighres_imu.zgyro(Float)
    phighres_imu.xmag(Float)
    phighres_imu.ymag(Float)
    phighres_imu.zmag(Float)
    phighres_imu.abs_pressure(Float)
    phighres_imu.diff_pressure(Float)
    phighres_imu.pressure_alt(Float)
    phighres_imu.temperature(Float)
    phighres_imu.fields_updated(Short)

}

fun HIGHRES_IMU.copyInto(DST: HIGHRES_IMU) {
    HIGHRES_IMU.pull(
            DST, { -> time_usec() }, { -> xacc() }, { -> yacc() }, { -> zacc() }, { -> xgyro() }, { -> ygyro() }, { -> zgyro() }, { -> xmag() }, { -> ymag() }, { -> zmag() }, { -> abs_pressure() }, { -> diff_pressure() }, { -> pressure_alt() }, { -> temperature() }, { -> fields_updated() }

    )
}

fun onEXTENDED_SYS_STATE(pextended_sys_state: EXTENDED_SYS_STATE) {

    pextended_sys_state.vtol_state()?.let { item ->
        some_MAV_VTOL_STATE = item.get()
    }

    pextended_sys_state.landed_state()?.let { item ->
        some_MAV_LANDED_STATE = item.get()
    }

}

fun EXTENDED_SYS_STATE.copyFrom(SRC: EXTENDED_SYS_STATE) {
    EXTENDED_SYS_STATE.push(
            SRC, { src -> vtol_state(src) }, { src -> landed_state(src) }

    )
}

fun fill(pextended_sys_state: EXTENDED_SYS_STATE) {

    pextended_sys_state.vtol_state(some_MAV_VTOL_STATE)

    pextended_sys_state.landed_state(some_MAV_LANDED_STATE)

}

fun EXTENDED_SYS_STATE.copyInto(DST: EXTENDED_SYS_STATE) {
    EXTENDED_SYS_STATE.pull(
            DST, { vtol_state() != null }, { vtol_state()!!.get() }, { landed_state() != null }, { landed_state()!!.get() }

    )
}

fun onUAVIONIX_ADSB_OUT_DYNAMIC(puavionix_adsb_out_dynamic: UAVIONIX_ADSB_OUT_DYNAMIC) {
    val some_utcTime = puavionix_adsb_out_dynamic.utcTime()
    val some_gpsLat = puavionix_adsb_out_dynamic.gpsLat()
    val some_gpsLon = puavionix_adsb_out_dynamic.gpsLon()
    val some_gpsAlt = puavionix_adsb_out_dynamic.gpsAlt()

    puavionix_adsb_out_dynamic.gpsFix()?.let { item ->
        some_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX = item.get()
    }
    val some_numSats = puavionix_adsb_out_dynamic.numSats()
    val some_baroAltMSL = puavionix_adsb_out_dynamic.baroAltMSL()
    val some_accuracyHor = puavionix_adsb_out_dynamic.accuracyHor()
    val some_accuracyVert = puavionix_adsb_out_dynamic.accuracyVert()
    val some_accuracyVel = puavionix_adsb_out_dynamic.accuracyVel()
    val some_velVert = puavionix_adsb_out_dynamic.velVert()
    val some_velNS = puavionix_adsb_out_dynamic.velNS()
    val some_VelEW = puavionix_adsb_out_dynamic.VelEW()

    puavionix_adsb_out_dynamic.emergencyStatus()?.let { item ->
        some_UAVIONIX_ADSB_EMERGENCY_STATUS = item.get()
    }

    puavionix_adsb_out_dynamic.state()?.let { item ->
        some_UAVIONIX_ADSB_OUT_DYNAMIC_STATE = item.get()
    }
    val some_squawk = puavionix_adsb_out_dynamic.squawk()

}

fun UAVIONIX_ADSB_OUT_DYNAMIC.copyFrom(SRC: UAVIONIX_ADSB_OUT_DYNAMIC) {
    UAVIONIX_ADSB_OUT_DYNAMIC.push(
            SRC, { src -> utcTime(src) }, { src -> gpsLat(src) }, { src -> gpsLon(src) }, { src -> gpsAlt(src) }, { src -> gpsFix(src) }, { src -> numSats(src) }, { src -> baroAltMSL(src) }, { src -> accuracyHor(src) }, { src -> accuracyVert(src) }, { src -> accuracyVel(src) }, { src -> velVert(src) }, { src -> velNS(src) }, { src -> VelEW(src) }, { src -> emergencyStatus(src) }, { src -> state(src) }, { src -> squawk(src) }

    )
}

fun fill(puavionix_adsb_out_dynamic: UAVIONIX_ADSB_OUT_DYNAMIC) {
    puavionix_adsb_out_dynamic.utcTime(Int)
    puavionix_adsb_out_dynamic.gpsLat(Int)
    puavionix_adsb_out_dynamic.gpsLon(Int)
    puavionix_adsb_out_dynamic.gpsAlt(Int)

    puavionix_adsb_out_dynamic.gpsFix(some_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)
    puavionix_adsb_out_dynamic.numSats(Byte)
    puavionix_adsb_out_dynamic.baroAltMSL(Int)
    puavionix_adsb_out_dynamic.accuracyHor(Int)
    puavionix_adsb_out_dynamic.accuracyVert(Short)
    puavionix_adsb_out_dynamic.accuracyVel(Short)
    puavionix_adsb_out_dynamic.velVert(Short)
    puavionix_adsb_out_dynamic.velNS(Short)
    puavionix_adsb_out_dynamic.VelEW(Short)

    puavionix_adsb_out_dynamic.emergencyStatus(some_UAVIONIX_ADSB_EMERGENCY_STATUS)

    puavionix_adsb_out_dynamic.state(some_UAVIONIX_ADSB_OUT_DYNAMIC_STATE)
    puavionix_adsb_out_dynamic.squawk(Short)

}

fun UAVIONIX_ADSB_OUT_DYNAMIC.copyInto(DST: UAVIONIX_ADSB_OUT_DYNAMIC) {
    UAVIONIX_ADSB_OUT_DYNAMIC.pull(
            DST, { -> utcTime() }, { -> gpsLat() }, { -> gpsLon() }, { -> gpsAlt() }, { gpsFix() != null }, { gpsFix()!!.get() }, { -> numSats() }, { -> baroAltMSL() }, { -> accuracyHor() }, { -> accuracyVert() }, { -> accuracyVel() }, { -> velVert() }, { -> velNS() }, { -> VelEW() }, { emergencyStatus() != null }, { emergencyStatus()!!.get() }, { state() != null }, { state()!!.get() }, { -> squawk() }

    )
}

fun onGOPRO_GET_RESPONSE(pgopro_get_response: GOPRO_GET_RESPONSE) {

    pgopro_get_response.cmd_id()?.let { item ->
        some_GOPRO_COMMAND = item.get()
    }

    pgopro_get_response.status()?.let { item ->
        some_GOPRO_REQUEST_STATUS = item.get()
    }
    pgopro_get_response.value().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun GOPRO_GET_RESPONSE.copyFrom(SRC: GOPRO_GET_RESPONSE) {
    GOPRO_GET_RESPONSE.push(
            SRC, { src -> cmd_id(src) }, { src -> status(src) }, { src ->
        val item = value()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pgopro_get_response: GOPRO_GET_RESPONSE) {

    pgopro_get_response.cmd_id(some_GOPRO_COMMAND)

    pgopro_get_response.status(some_GOPRO_REQUEST_STATUS)
    pgopro_get_response.value().let { item ->
        pgopro_get_response.value().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun GOPRO_GET_RESPONSE.copyInto(DST: GOPRO_GET_RESPONSE) {
    GOPRO_GET_RESPONSE.pull(
            DST, { cmd_id() != null }, { cmd_id()!!.get() }, { status() != null }, { status()!!.get() }, { dst ->
        var src = this.value()
        for (i in 0 until 4)
            dst.set(src.get(i), i)
    }

    )
}

fun onGPS_INJECT_DATA(pgps_inject_data: GPS_INJECT_DATA) {
    val some_target_system = pgps_inject_data.target_system()
    val some_target_component = pgps_inject_data.target_component()
    val some_len = pgps_inject_data.len()
    pgps_inject_data.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun GPS_INJECT_DATA.copyFrom(SRC: GPS_INJECT_DATA) {
    GPS_INJECT_DATA.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> len(src) }, { src ->
        val item = daTa()
        for (i in 0 until 110)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pgps_inject_data: GPS_INJECT_DATA) {
    pgps_inject_data.target_system(Byte)
    pgps_inject_data.target_component(Byte)
    pgps_inject_data.len(Byte)
    pgps_inject_data.daTa().let { item ->
        pgps_inject_data.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun GPS_INJECT_DATA.copyInto(DST: GPS_INJECT_DATA) {
    GPS_INJECT_DATA.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> len() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 110)
            dst.set(src.get(i), i)
    }

    )
}

fun onUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(puavionix_adsb_transceiver_health_report: UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) {

    puavionix_adsb_transceiver_health_report.rfHealth()?.let { item ->
        some_UAVIONIX_ADSB_RF_HEALTH = item.get()
    }

}

fun UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.copyFrom(SRC: UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) {
    UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.push(
            SRC, { src -> rfHealth(src) }

    )
}

fun fill(puavionix_adsb_transceiver_health_report: UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) {

    puavionix_adsb_transceiver_health_report.rfHealth(some_UAVIONIX_ADSB_RF_HEALTH)

}

fun UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.copyInto(DST: UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) {
    UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.pull(
            DST, { rfHealth() != null }, { rfHealth()!!.get() }

    )
}

fun onATTITUDE_QUATERNION_COV(pattitude_quaternion_cov: ATTITUDE_QUATERNION_COV) {
    val some_time_usec = pattitude_quaternion_cov.time_usec()
    pattitude_quaternion_cov.q().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }
    val some_rollspeed = pattitude_quaternion_cov.rollspeed()
    val some_pitchspeed = pattitude_quaternion_cov.pitchspeed()
    val some_yawspeed = pattitude_quaternion_cov.yawspeed()
    pattitude_quaternion_cov.covariance().let { item ->
        for (index in 0 until item.len())
            Float = item.get(index)
    }

}

fun test_.ATTITUDE_QUATERNION_COV.copyFrom(SRC: ATTITUDE_QUATERNION_COV) {
    ATTITUDE_QUATERNION_COV.push(
            SRC, { src -> time_usec(src) }, { src ->
        val item = q()
        for (i in 0 until 4)
            item.set(src.get(i), i)
    }, { src -> rollspeed(src) }, { src -> pitchspeed(src) }, { src -> yawspeed(src) }, { src ->
        val item = covariance()
        for (i in 0 until 9)
            item.set(src.get(i), i)
    }

    )
}

fun onNAMED_VALUE_INT(pnamed_value_int: NAMED_VALUE_INT) {
    val some_time_boot_ms = pnamed_value_int.time_boot_ms()

    pnamed_value_int.name()?.let { item -> char = item.get() }
    val some_value = pnamed_value_int.value()

}

fun NAMED_VALUE_INT.copyFrom(SRC: NAMED_VALUE_INT) {
    NAMED_VALUE_INT.push(
            SRC, { src -> time_boot_ms(src) }, { src ->
        val item = name(src.len())
        for (i in 0 until item.len())
            item.set(src.get(i), i)
    }, { src -> value(src) }

    )
}

fun fill(pnamed_value_int: NAMED_VALUE_INT) {
    pnamed_value_int.time_boot_ms(Int)

    pnamed_value_int.name(some_String, null)
    pnamed_value_int.value(Int)

}

fun NAMED_VALUE_INT.copyInto(DST: NAMED_VALUE_INT) {
    NAMED_VALUE_INT.pull(
            DST, { -> time_boot_ms() }, { name()?.let { item -> item.len() } ?: 0 }, { dst ->
        val item = name()!!
        for (i in 0 until item.len())
            dst.set(item.get(i), i)
    }, { -> value() }

    )
}

fun onRPM(prpm: RPM) {
    val some_rpm1 = prpm.rpm1()
    val some_rpm2 = prpm.rpm2()

}

fun RPM.copyFrom(SRC: RPM) {
    RPM.push(
            SRC, { src -> rpm1(src) }, { src -> rpm2(src) }

    )
}

fun fill(prpm: RPM) {
    prpm.rpm1(Float)
    prpm.rpm2(Float)

}

fun RPM.copyInto(DST: RPM) {
    RPM.pull(
            DST, { -> rpm1() }, { -> rpm2() }

    )
}

fun onGPS_RTCM_DATA(pgps_rtcm_data: GPS_RTCM_DATA) {
    val some_flags = pgps_rtcm_data.flags()
    val some_len = pgps_rtcm_data.len()
    pgps_rtcm_data.daTa().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun GPS_RTCM_DATA.copyFrom(SRC: GPS_RTCM_DATA) {
    GPS_RTCM_DATA.push(
            SRC, { src -> flags(src) }, { src -> len(src) }, { src ->
        val item = daTa()
        for (i in 0 until 180)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pgps_rtcm_data: GPS_RTCM_DATA) {
    pgps_rtcm_data.flags(Byte)
    pgps_rtcm_data.len(Byte)
    pgps_rtcm_data.daTa().let { item ->
        pgps_rtcm_data.daTa().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun GPS_RTCM_DATA.copyInto(DST: GPS_RTCM_DATA) {
    GPS_RTCM_DATA.pull(
            DST, { -> flags() }, { -> len() }, { dst ->
        var src = this.daTa()
        for (i in 0 until 180)
            dst.set(src.get(i), i)
    }

    )
}

fun onGLOBAL_VISION_POSITION_ESTIMATE(pglobal_vision_position_estimate: GLOBAL_VISION_POSITION_ESTIMATE) {
    val some_usec = pglobal_vision_position_estimate.usec()
    val some_x = pglobal_vision_position_estimate.x()
    val some_y = pglobal_vision_position_estimate.y()
    val some_z = pglobal_vision_position_estimate.z()
    val some_roll = pglobal_vision_position_estimate.roll()
    val some_pitch = pglobal_vision_position_estimate.pitch()
    val some_yaw = pglobal_vision_position_estimate.yaw()

}

fun test_.GLOBAL_VISION_POSITION_ESTIMATE.copyFrom(SRC: GLOBAL_VISION_POSITION_ESTIMATE) {
    GLOBAL_VISION_POSITION_ESTIMATE.push(
            SRC, { src -> usec(src) }, { src -> x(src) }, { src -> y(src) }, { src -> z(src) }, { src -> roll(src) }, { src -> pitch(src) }, { src -> yaw(src) }

    )
}

fun onFILE_TRANSFER_PROTOCOL(pfile_transfer_protocol: FILE_TRANSFER_PROTOCOL) {
    val some_target_network = pfile_transfer_protocol.target_network()
    val some_target_system = pfile_transfer_protocol.target_system()
    val some_target_component = pfile_transfer_protocol.target_component()
    pfile_transfer_protocol.payload().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun FILE_TRANSFER_PROTOCOL.copyFrom(SRC: FILE_TRANSFER_PROTOCOL) {
    FILE_TRANSFER_PROTOCOL.push(
            SRC, { src -> target_network(src) }, { src -> target_system(src) }, { src -> target_component(src) }, { src ->
        val item = payload()
        for (i in 0 until 251)
            item.set(src.get(i), i)
    }

    )
}

fun fill(pfile_transfer_protocol: FILE_TRANSFER_PROTOCOL) {
    pfile_transfer_protocol.target_network(Byte)
    pfile_transfer_protocol.target_system(Byte)
    pfile_transfer_protocol.target_component(Byte)
    pfile_transfer_protocol.payload().let { item ->
        pfile_transfer_protocol.payload().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun FILE_TRANSFER_PROTOCOL.copyInto(DST: FILE_TRANSFER_PROTOCOL) {
    FILE_TRANSFER_PROTOCOL.pull(
            DST, { -> target_network() }, { -> target_system() }, { -> target_component() }, { dst ->
        var src = this.payload()
        for (i in 0 until 251)
            dst.set(src.get(i), i)
    }

    )
}

fun onRANGEFINDER(prangefinder: RANGEFINDER) {
    val some_distance = prangefinder.distance()
    val some_voltage = prangefinder.voltage()

}

fun RANGEFINDER.copyFrom(SRC: RANGEFINDER) {
    RANGEFINDER.push(
            SRC, { src -> distance(src) }, { src -> voltage(src) }

    )
}

fun fill(prangefinder: RANGEFINDER) {
    prangefinder.distance(Float)
    prangefinder.voltage(Float)

}

fun RANGEFINDER.copyInto(DST: RANGEFINDER) {
    RANGEFINDER.pull(
            DST, { -> distance() }, { -> voltage() }

    )
}

fun onRADIO_STATUS(pradio_status: RADIO_STATUS) {
    val some_rssi = pradio_status.rssi()
    val some_remrssi = pradio_status.remrssi()
    val some_txbuf = pradio_status.txbuf()
    val some_noise = pradio_status.noise()
    val some_remnoise = pradio_status.remnoise()
    val some_rxerrors = pradio_status.rxerrors()
    val some_fixeD = pradio_status.fixeD()

}

fun RADIO_STATUS.copyFrom(SRC: RADIO_STATUS) {
    RADIO_STATUS.push(
            SRC, { src -> rssi(src) }, { src -> remrssi(src) }, { src -> txbuf(src) }, { src -> noise(src) }, { src -> remnoise(src) }, { src -> rxerrors(src) }, { src -> fixeD(src) }

    )
}

fun fill(pradio_status: RADIO_STATUS) {
    pradio_status.rssi(Byte)
    pradio_status.remrssi(Byte)
    pradio_status.txbuf(Byte)
    pradio_status.noise(Byte)
    pradio_status.remnoise(Byte)
    pradio_status.rxerrors(Short)
    pradio_status.fixeD(Short)

}

fun RADIO_STATUS.copyInto(DST: RADIO_STATUS) {
    RADIO_STATUS.pull(
            DST, { -> rssi() }, { -> remrssi() }, { -> txbuf() }, { -> noise() }, { -> remnoise() }, { -> rxerrors() }, { -> fixeD() }

    )
}

fun onFENCE_POINT(pfence_point: FENCE_POINT) {
    val some_target_system = pfence_point.target_system()
    val some_target_component = pfence_point.target_component()
    val some_idx = pfence_point.idx()
    val some_count = pfence_point.count()
    val some_lat = pfence_point.lat()
    val some_lng = pfence_point.lng()

}

fun FENCE_POINT.copyFrom(SRC: FENCE_POINT) {
    FENCE_POINT.push(
            SRC, { src -> target_system(src) }, { src -> target_component(src) }, { src -> idx(src) }, { src -> count(src) }, { src -> lat(src) }, { src -> lng(src) }

    )
}

fun fill(pfence_point: FENCE_POINT) {
    pfence_point.target_system(Byte)
    pfence_point.target_component(Byte)
    pfence_point.idx(Byte)
    pfence_point.count(Byte)
    pfence_point.lat(Float)
    pfence_point.lng(Float)

}

fun FENCE_POINT.copyInto(DST: FENCE_POINT) {
    FENCE_POINT.pull(
            DST, { -> target_system() }, { -> target_component() }, { -> idx() }, { -> count() }, { -> lat() }, { -> lng() }

    )
}

fun onRESOURCE_REQUEST(presource_request: RESOURCE_REQUEST) {
    val some_request_id = presource_request.request_id()
    val some_uri_type = presource_request.uri_type()
    presource_request.uri().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }
    val some_transfer_type = presource_request.transfer_type()
    presource_request.storage().let { item ->
        for (index in 0 until item.len())
            Byte = item.get(index)
    }

}

fun RESOURCE_REQUEST.copyFrom(SRC: RESOURCE_REQUEST) {
    RESOURCE_REQUEST.push(
            SRC, { src -> request_id(src) }, { src -> uri_type(src) }, { src ->
        val item = uri()
        for (i in 0 until 120)
            item.set(src.get(i), i)
    }, { src -> transfer_type(src) }, { src ->
        val item = storage()
        for (i in 0 until 120)
            item.set(src.get(i), i)
    }

    )
}

fun fill(presource_request: RESOURCE_REQUEST) {
    presource_request.request_id(Byte)
    presource_request.uri_type(Byte)
    presource_request.uri().let { item ->
        presource_request.uri().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }
    presource_request.transfer_type(Byte)
    presource_request.storage().let { item ->
        presource_request.storage().let { item ->
            for (index in 0 until item.len())
                item.set(Byte, index)
        }
    }

}

fun RESOURCE_REQUEST.copyInto(DST: RESOURCE_REQUEST) {
    RESOURCE_REQUEST.pull(
            DST, { -> request_id() }, { -> uri_type() }, { dst ->
        var src = this.uri()
        for (i in 0 until 120)
            dst.set(src.get(i), i)
    }, { -> transfer_type() }, { dst ->
        var src = this.storage()
        for (i in 0 until 120)
            dst.set(src.get(i), i)
    }

    )
}

class CommunicationChannel_demo : CommunicationChannel() {
    val sendingPacks = ConcurrentLinkedQueue<AdHoc.Pack>()
    override fun pushSendingPack(pack: AdHoc.Pack): Boolean {
        return sendingPacks.add(pack)
    }

    override fun pullSendingPack(): AdHoc.Pack? {
        return sendingPacks.poll()
    }

    override fun on_RESOURCE_REQUEST(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val presource_request = RESOURCE_REQUEST(bytes)
        onRESOURCE_REQUEST.forEach { handler -> handler(presource_request) }
    }

    override fun on_ATTITUDE_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pattitude_target = ATTITUDE_TARGET(bytes)
        onATTITUDE_TARGET.forEach { handler -> handler(pattitude_target) }
    }

    override fun on_MISSION_COUNT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_count = MISSION_COUNT(cur)
        onMISSION_COUNT.forEach { handler -> handler(pmission_count) }
    }

    override fun on_ADSB_VEHICLE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val padsb_vehicle = ADSB_VEHICLE(cur)
        onADSB_VEHICLE.forEach { handler -> handler(padsb_vehicle) }
    }

    override fun on_MESSAGE_INTERVAL(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmessage_interval = MESSAGE_INTERVAL(bytes)
        onMESSAGE_INTERVAL.forEach { handler -> handler(pmessage_interval) }
    }

    override fun on_ESTIMATOR_STATUS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pestimator_status = ESTIMATOR_STATUS(cur)
        onESTIMATOR_STATUS.forEach { handler -> handler(pestimator_status) }
    }

    override fun on_TIMESYNC(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val ptimesync = TIMESYNC(bytes)
        onTIMESYNC.forEach { handler -> handler(ptimesync) }
    }

    override fun on_GLOBAL_POSITION_INT_COV(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pglobal_position_int_cov = GLOBAL_POSITION_INT_COV(cur)
        onGLOBAL_POSITION_INT_COV.forEach { handler -> handler(pglobal_position_int_cov) }
    }

    override fun on_BUTTON_CHANGE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pbutton_change = BUTTON_CHANGE(bytes)
        onBUTTON_CHANGE.forEach { handler -> handler(pbutton_change) }
    }

    override fun on_SAFETY_SET_ALLOWED_AREA(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val psafety_set_allowed_area = SAFETY_SET_ALLOWED_AREA(cur)
        onSAFETY_SET_ALLOWED_AREA.forEach { handler -> handler(psafety_set_allowed_area) }
    }

    override fun on_STORAGE_INFORMATION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pstorage_information = STORAGE_INFORMATION(bytes)
        onSTORAGE_INFORMATION.forEach { handler -> handler(pstorage_information) }
    }

    override fun on_COLLISION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcollision = COLLISION(cur)
        onCOLLISION.forEach { handler -> handler(pcollision) }
    }

    override fun on_ALTITUDE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val paltitude = ALTITUDE(bytes)
        onALTITUDE.forEach { handler -> handler(paltitude) }
    }

    override fun on_HIL_STATE_QUATERNION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_state_quaternion = HIL_STATE_QUATERNION(bytes)
        onHIL_STATE_QUATERNION.forEach { handler -> handler(phil_state_quaternion) }
    }

    override fun on_CAMERA_INFORMATION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcamera_information = CAMERA_INFORMATION(cur)
        onCAMERA_INFORMATION.forEach { handler -> handler(pcamera_information) }
    }

    override fun on_GPS_STATUS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps_status = GPS_STATUS(bytes)
        onGPS_STATUS.forEach { handler -> handler(pgps_status) }
    }

    override fun on_PARAM_SET(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pparam_set = PARAM_SET(cur)
        onPARAM_SET.forEach { handler -> handler(pparam_set) }
    }

    override fun on_TERRAIN_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pterrain_data = TERRAIN_DATA(bytes)
        onTERRAIN_DATA.forEach { handler -> handler(pterrain_data) }
    }

    override fun on_RC_CHANNELS_OVERRIDE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prc_channels_override = RC_CHANNELS_OVERRIDE(bytes)
        onRC_CHANNELS_OVERRIDE.forEach { handler -> handler(prc_channels_override) }
    }

    override fun on_SCALED_IMU(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_imu = SCALED_IMU(bytes)
        onSCALED_IMU.forEach { handler -> handler(pscaled_imu) }
    }

    override fun on_DEBUG(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pdebug = DEBUG(bytes)
        onDEBUG.forEach { handler -> handler(pdebug) }
    }

    override fun on_CAMERA_IMAGE_CAPTURED(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcamera_image_captured = CAMERA_IMAGE_CAPTURED(cur)
        onCAMERA_IMAGE_CAPTURED.forEach { handler -> handler(pcamera_image_captured) }
    }

    override fun on_LOG_ENTRY(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_entry = LOG_ENTRY(bytes)
        onLOG_ENTRY.forEach { handler -> handler(plog_entry) }
    }

    override fun on_ACTUATOR_CONTROL_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pactuator_control_target = ACTUATOR_CONTROL_TARGET(bytes)
        onACTUATOR_CONTROL_TARGET.forEach { handler -> handler(pactuator_control_target) }
    }

    override fun on_HIGH_LATENCY(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val phigh_latency = HIGH_LATENCY(cur)
        onHIGH_LATENCY.forEach { handler -> handler(phigh_latency) }
    }

    override fun on_PARAM_REQUEST_READ(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pparam_request_read = PARAM_REQUEST_READ(cur)
        onPARAM_REQUEST_READ.forEach { handler -> handler(pparam_request_read) }
    }

    override fun on_SET_ATTITUDE_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pset_attitude_target = SET_ATTITUDE_TARGET(bytes)
        onSET_ATTITUDE_TARGET.forEach { handler -> handler(pset_attitude_target) }
    }

    override fun on_FOLLOW_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pfollow_target = FOLLOW_TARGET(bytes)
        onFOLLOW_TARGET.forEach { handler -> handler(pfollow_target) }
    }

    override fun on_HIL_STATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_state = HIL_STATE(bytes)
        onHIL_STATE.forEach { handler -> handler(phil_state) }
    }

    override fun on_HOME_POSITION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val phome_position = HOME_POSITION(cur)
        onHOME_POSITION.forEach { handler -> handler(phome_position) }
    }

    override fun on_GPS2_RAW(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pgps2_raw = GPS2_RAW(cur)
        onGPS2_RAW.forEach { handler -> handler(pgps2_raw) }
    }

    override fun on_MEMORY_VECT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmemory_vect = MEMORY_VECT(bytes)
        onMEMORY_VECT.forEach { handler -> handler(pmemory_vect) }
    }

    override fun on_REQUEST_DATA_STREAM(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prequest_data_stream = REQUEST_DATA_STREAM(bytes)
        onREQUEST_DATA_STREAM.forEach { handler -> handler(prequest_data_stream) }
    }

    override fun on_HIL_CONTROLS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val phil_controls = HIL_CONTROLS(cur)
        onHIL_CONTROLS.forEach { handler -> handler(phil_controls) }
    }

    override fun on_HIL_SENSOR(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_sensor = HIL_SENSOR(bytes)
        onHIL_SENSOR.forEach { handler -> handler(phil_sensor) }
    }

    override fun on_SETUP_SIGNING(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val psetup_signing = SETUP_SIGNING(bytes)
        onSETUP_SIGNING.forEach { handler -> handler(psetup_signing) }
    }

    override fun on_GPS_RTK(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps_rtk = GPS_RTK(bytes)
        onGPS_RTK.forEach { handler -> handler(pgps_rtk) }
    }

    override fun on_PARAM_REQUEST_LIST(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pparam_request_list = PARAM_REQUEST_LIST(bytes)
        onPARAM_REQUEST_LIST.forEach { handler -> handler(pparam_request_list) }
    }

    override fun on_LANDING_TARGET(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val planding_target = LANDING_TARGET(cur)
        onLANDING_TARGET.forEach { handler -> handler(planding_target) }
    }

    override fun on_SET_ACTUATOR_CONTROL_TARGET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pset_actuator_control_target = SET_ACTUATOR_CONTROL_TARGET(bytes)
        onSET_ACTUATOR_CONTROL_TARGET.forEach { handler -> handler(pset_actuator_control_target) }
    }

    override fun on_CONTROL_SYSTEM_STATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pcontrol_system_state = CONTROL_SYSTEM_STATE(bytes)
        onCONTROL_SYSTEM_STATE.forEach { handler -> handler(pcontrol_system_state) }
    }

    override fun on_SET_POSITION_TARGET_GLOBAL_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_position_target_global_int = SET_POSITION_TARGET_GLOBAL_INT(cur)
        onSET_POSITION_TARGET_GLOBAL_INT.forEach { handler -> handler(pset_position_target_global_int) }
    }

    override fun on_VIBRATION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvibration = VIBRATION(bytes)
        onVIBRATION.forEach { handler -> handler(pvibration) }
    }

    override fun on_PING33(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pping33 = PING33(cur)
        onPING33.forEach { handler -> handler(pping33) }
    }

    override fun on_VFR_HUD(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvfr_hud = VFR_HUD(bytes)
        onVFR_HUD.forEach { handler -> handler(pvfr_hud) }
    }

    override fun on_MISSION_SET_CURRENT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmission_set_current = MISSION_SET_CURRENT(bytes)
        onMISSION_SET_CURRENT.forEach { handler -> handler(pmission_set_current) }
    }

    override fun on_HIL_GPS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_gps = HIL_GPS(bytes)
        onHIL_GPS.forEach { handler -> handler(phil_gps) }
    }

    override fun on_NAV_CONTROLLER_OUTPUT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pnav_controller_output = NAV_CONTROLLER_OUTPUT(bytes)
        onNAV_CONTROLLER_OUTPUT.forEach { handler -> handler(pnav_controller_output) }
    }

    override fun on_AUTH_KEY(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pauth_key = AUTH_KEY(cur)
        onAUTH_KEY.forEach { handler -> handler(pauth_key) }
    }

    override fun on_LOCAL_POSITION_NED_COV(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val plocal_position_ned_cov = LOCAL_POSITION_NED_COV(cur)
        onLOCAL_POSITION_NED_COV.forEach { handler -> handler(plocal_position_ned_cov) }
    }

    override fun on_ATT_POS_MOCAP(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val patt_pos_mocap = ATT_POS_MOCAP(bytes)
        onATT_POS_MOCAP.forEach { handler -> handler(patt_pos_mocap) }
    }

    override fun on_STATUSTEXT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pstatustext = STATUSTEXT(cur)
        onSTATUSTEXT.forEach { handler -> handler(pstatustext) }
    }

    override fun on_PING(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pping = PING(bytes)
        onPING.forEach { handler -> handler(pping) }
    }

    override fun on_CAMERA_CAPTURE_STATUS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pcamera_capture_status = CAMERA_CAPTURE_STATUS(bytes)
        onCAMERA_CAPTURE_STATUS.forEach { handler -> handler(pcamera_capture_status) }
    }

    override fun on_GLOBAL_POSITION_INT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pglobal_position_int = GLOBAL_POSITION_INT(bytes)
        onGLOBAL_POSITION_INT.forEach { handler -> handler(pglobal_position_int) }
    }

    override fun on_ENCAPSULATED_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pencapsulated_data = ENCAPSULATED_DATA(bytes)
        onENCAPSULATED_DATA.forEach { handler -> handler(pencapsulated_data) }
    }

    override fun on_GPS_INPUT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pgps_input = GPS_INPUT(cur)
        onGPS_INPUT.forEach { handler -> handler(pgps_input) }
    }

    override fun on_COMMAND_LONG(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcommand_long = COMMAND_LONG(cur)
        onCOMMAND_LONG.forEach { handler -> handler(pcommand_long) }
    }

    override fun on_LOG_REQUEST_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_request_data = LOG_REQUEST_DATA(bytes)
        onLOG_REQUEST_DATA.forEach { handler -> handler(plog_request_data) }
    }

    override fun on_GPS_RAW_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pgps_raw_int = GPS_RAW_INT(cur)
        onGPS_RAW_INT.forEach { handler -> handler(pgps_raw_int) }
    }

    override fun on_RC_CHANNELS_SCALED(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prc_channels_scaled = RC_CHANNELS_SCALED(bytes)
        onRC_CHANNELS_SCALED.forEach { handler -> handler(prc_channels_scaled) }
    }

    override fun on_CAMERA_SETTINGS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcamera_settings = CAMERA_SETTINGS(cur)
        onCAMERA_SETTINGS.forEach { handler -> handler(pcamera_settings) }
    }

    override fun on_RAW_PRESSURE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val praw_pressure = RAW_PRESSURE(bytes)
        onRAW_PRESSURE.forEach { handler -> handler(praw_pressure) }
    }

    override fun on_NAMED_VALUE_FLOAT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pnamed_value_float = NAMED_VALUE_FLOAT(cur)
        onNAMED_VALUE_FLOAT.forEach { handler -> handler(pnamed_value_float) }
    }

    override fun on_ATTITUDE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pattitude = ATTITUDE(bytes)
        onATTITUDE.forEach { handler -> handler(pattitude) }
    }

    override fun on_TERRAIN_REQUEST(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pterrain_request = TERRAIN_REQUEST(bytes)
        onTERRAIN_REQUEST.forEach { handler -> handler(pterrain_request) }
    }

    override fun on_MISSION_WRITE_PARTIAL_LIST(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_write_partial_list = MISSION_WRITE_PARTIAL_LIST(cur)
        onMISSION_WRITE_PARTIAL_LIST.forEach { handler -> handler(pmission_write_partial_list) }
    }

    override fun on_LOG_ERASE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_erase = LOG_ERASE(bytes)
        onLOG_ERASE.forEach { handler -> handler(plog_erase) }
    }

    override fun on_MANUAL_SETPOINT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmanual_setpoint = MANUAL_SETPOINT(bytes)
        onMANUAL_SETPOINT.forEach { handler -> handler(pmanual_setpoint) }
    }

    override fun on_SAFETY_ALLOWED_AREA(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val psafety_allowed_area = SAFETY_ALLOWED_AREA(cur)
        onSAFETY_ALLOWED_AREA.forEach { handler -> handler(psafety_allowed_area) }
    }

    override fun on_OPTICAL_FLOW_RAD(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val poptical_flow_rad = OPTICAL_FLOW_RAD(bytes)
        onOPTICAL_FLOW_RAD.forEach { handler -> handler(poptical_flow_rad) }
    }

    override fun on_LOG_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_data = LOG_DATA(bytes)
        onLOG_DATA.forEach { handler -> handler(plog_data) }
    }

    override fun on_MISSION_CLEAR_ALL(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_clear_all = MISSION_CLEAR_ALL(cur)
        onMISSION_CLEAR_ALL.forEach { handler -> handler(pmission_clear_all) }
    }

    override fun on_VICON_POSITION_ESTIMATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvicon_position_estimate = VICON_POSITION_ESTIMATE(bytes)
        onVICON_POSITION_ESTIMATE.forEach { handler -> handler(pvicon_position_estimate) }
    }

    override fun on_GPS2_RTK(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps2_rtk = GPS2_RTK(bytes)
        onGPS2_RTK.forEach { handler -> handler(pgps2_rtk) }
    }

    override fun on_LOG_REQUEST_LIST(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_request_list = LOG_REQUEST_LIST(bytes)
        onLOG_REQUEST_LIST.forEach { handler -> handler(plog_request_list) }
    }

    override fun on_SCALED_PRESSURE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_pressure = SCALED_PRESSURE(bytes)
        onSCALED_PRESSURE.forEach { handler -> handler(pscaled_pressure) }
    }

    override fun on_MISSION_REQUEST_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_request_int = MISSION_REQUEST_INT(cur)
        onMISSION_REQUEST_INT.forEach { handler -> handler(pmission_request_int) }
    }

    override fun on_V2_EXTENSION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pv2_extension = V2_EXTENSION(bytes)
        onV2_EXTENSION.forEach { handler -> handler(pv2_extension) }
    }

    override fun on_HEARTBEAT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pheartbeat = HEARTBEAT(cur)
        onHEARTBEAT.forEach { handler -> handler(pheartbeat) }
    }

    override fun on_PARAM_MAP_RC(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pparam_map_rc = PARAM_MAP_RC(cur)
        onPARAM_MAP_RC.forEach { handler -> handler(pparam_map_rc) }
    }

    override fun on_POWER_STATUS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val ppower_status = POWER_STATUS(cur)
        onPOWER_STATUS.forEach { handler -> handler(ppower_status) }
    }

    override fun on_TERRAIN_CHECK(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pterrain_check = TERRAIN_CHECK(bytes)
        onTERRAIN_CHECK.forEach { handler -> handler(pterrain_check) }
    }

    override fun on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plocal_position_ned_system_global_offset = LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(bytes)
        onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.forEach { handler -> handler(plocal_position_ned_system_global_offset) }
    }

    override fun on_COMMAND_ACK(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcommand_ack = COMMAND_ACK(cur)
        onCOMMAND_ACK.forEach { handler -> handler(pcommand_ack) }
    }

    override fun on_DATA_STREAM(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pdata_stream = DATA_STREAM(bytes)
        onDATA_STREAM.forEach { handler -> handler(pdata_stream) }
    }

    override fun on_MISSION_REQUEST(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_request = MISSION_REQUEST(cur)
        onMISSION_REQUEST.forEach { handler -> handler(pmission_request) }
    }

    override fun on_TERRAIN_REPORT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pterrain_report = TERRAIN_REPORT(bytes)
        onTERRAIN_REPORT.forEach { handler -> handler(pterrain_report) }
    }

    override fun on_SET_HOME_POSITION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_home_position = SET_HOME_POSITION(cur)
        onSET_HOME_POSITION.forEach { handler -> handler(pset_home_position) }
    }

    override fun on_SwitchModeCommand() {
        onSwitchModeCommand.forEach { handler -> handler() }
    }

    override fun on_HIL_RC_INPUTS_RAW(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_rc_inputs_raw = HIL_RC_INPUTS_RAW(bytes)
        onHIL_RC_INPUTS_RAW.forEach { handler -> handler(phil_rc_inputs_raw) }
    }

    override fun on_SCALED_IMU3(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_imu3 = SCALED_IMU3(bytes)
        onSCALED_IMU3.forEach { handler -> handler(pscaled_imu3) }
    }

    override fun on_SET_MODE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_mode = SET_MODE(cur)
        onSET_MODE.forEach { handler -> handler(pset_mode) }
    }

    override fun on_POSITION_TARGET_GLOBAL_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pposition_target_global_int = POSITION_TARGET_GLOBAL_INT(cur)
        onPOSITION_TARGET_GLOBAL_INT.forEach { handler -> handler(pposition_target_global_int) }
    }

    override fun on_FLIGHT_INFORMATION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pflight_information = FLIGHT_INFORMATION(bytes)
        onFLIGHT_INFORMATION.forEach { handler -> handler(pflight_information) }
    }

    override fun on_SIM_STATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val psim_state = SIM_STATE(bytes)
        onSIM_STATE.forEach { handler -> handler(psim_state) }
    }

    override fun on_MISSION_ITEM_REACHED(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmission_item_reached = MISSION_ITEM_REACHED(bytes)
        onMISSION_ITEM_REACHED.forEach { handler -> handler(pmission_item_reached) }
    }

    override fun on_RC_CHANNELS_RAW(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prc_channels_raw = RC_CHANNELS_RAW(bytes)
        onRC_CHANNELS_RAW.forEach { handler -> handler(prc_channels_raw) }
    }

    override fun on_SERVO_OUTPUT_RAW(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pservo_output_raw = SERVO_OUTPUT_RAW(cur)
        onSERVO_OUTPUT_RAW.forEach { handler -> handler(pservo_output_raw) }
    }

    override fun on_VISION_SPEED_ESTIMATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvision_speed_estimate = VISION_SPEED_ESTIMATE(bytes)
        onVISION_SPEED_ESTIMATE.forEach { handler -> handler(pvision_speed_estimate) }
    }

    override fun on_DEBUG_VECT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pdebug_vect = DEBUG_VECT(cur)
        onDEBUG_VECT.forEach { handler -> handler(pdebug_vect) }
    }

    override fun on_LOG_REQUEST_END(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plog_request_end = LOG_REQUEST_END(bytes)
        onLOG_REQUEST_END.forEach { handler -> handler(plog_request_end) }
    }

    override fun on_MISSION_ACK(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_ack = MISSION_ACK(cur)
        onMISSION_ACK.forEach { handler -> handler(pmission_ack) }
    }

    override fun on_CHANGE_OPERATOR_CONTROL_ACK(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pchange_operator_control_ack = CHANGE_OPERATOR_CONTROL_ACK(bytes)
        onCHANGE_OPERATOR_CONTROL_ACK.forEach { handler -> handler(pchange_operator_control_ack) }
    }

    override fun on_MISSION_CURRENT(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmission_current = MISSION_CURRENT(bytes)
        onMISSION_CURRENT.forEach { handler -> handler(pmission_current) }
    }

    override fun on_SYSTEM_TIME(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val psystem_time = SYSTEM_TIME(bytes)
        onSYSTEM_TIME.forEach { handler -> handler(psystem_time) }
    }

    override fun on_CAMERA_TRIGGER(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pcamera_trigger = CAMERA_TRIGGER(bytes)
        onCAMERA_TRIGGER.forEach { handler -> handler(pcamera_trigger) }
    }

    override fun on_VISION_POSITION_ESTIMATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pvision_position_estimate = VISION_POSITION_ESTIMATE(bytes)
        onVISION_POSITION_ESTIMATE.forEach { handler -> handler(pvision_position_estimate) }
    }

    override fun on_MANUAL_CONTROL(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pmanual_control = MANUAL_CONTROL(bytes)
        onMANUAL_CONTROL.forEach { handler -> handler(pmanual_control) }
    }

    override fun on_RC_CHANNELS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val prc_channels = RC_CHANNELS(bytes)
        onRC_CHANNELS.forEach { handler -> handler(prc_channels) }
    }

    override fun on_PARAM_VALUE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pparam_value = PARAM_VALUE(cur)
        onPARAM_VALUE.forEach { handler -> handler(pparam_value) }
    }

    override fun on_BATTERY_STATUS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pbattery_status = BATTERY_STATUS(cur)
        onBATTERY_STATUS.forEach { handler -> handler(pbattery_status) }
    }

    override fun on_SET_POSITION_TARGET_LOCAL_NED(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_position_target_local_ned = SET_POSITION_TARGET_LOCAL_NED(cur)
        onSET_POSITION_TARGET_LOCAL_NED.forEach { handler -> handler(pset_position_target_local_ned) }
    }

    override fun on_SERIAL_CONTROL(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pserial_control = SERIAL_CONTROL(cur)
        onSERIAL_CONTROL.forEach { handler -> handler(pserial_control) }
    }

    override fun on_SET_GPS_GLOBAL_ORIGIN(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pset_gps_global_origin = SET_GPS_GLOBAL_ORIGIN(cur)
        onSET_GPS_GLOBAL_ORIGIN.forEach { handler -> handler(pset_gps_global_origin) }
    }

    override fun on_AUTOPILOT_VERSION(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pautopilot_version = AUTOPILOT_VERSION(cur)
        onAUTOPILOT_VERSION.forEach { handler -> handler(pautopilot_version) }
    }

    override fun on_MISSION_REQUEST_LIST(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_request_list = MISSION_REQUEST_LIST(cur)
        onMISSION_REQUEST_LIST.forEach { handler -> handler(pmission_request_list) }
    }

    override fun on_PLAY_TUNE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pplay_tune = PLAY_TUNE(cur)
        onPLAY_TUNE.forEach { handler -> handler(pplay_tune) }
    }

    override fun on_SCALED_PRESSURE3(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_pressure3 = SCALED_PRESSURE3(bytes)
        onSCALED_PRESSURE3.forEach { handler -> handler(pscaled_pressure3) }
    }

    override fun on_MISSION_REQUEST_PARTIAL_LIST(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_request_partial_list = MISSION_REQUEST_PARTIAL_LIST(cur)
        onMISSION_REQUEST_PARTIAL_LIST.forEach { handler -> handler(pmission_request_partial_list) }
    }

    override fun on_LOCAL_POSITION_NED(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val plocal_position_ned = LOCAL_POSITION_NED(bytes)
        onLOCAL_POSITION_NED.forEach { handler -> handler(plocal_position_ned) }
    }

    override fun on_DATA_TRANSMISSION_HANDSHAKE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pdata_transmission_handshake = DATA_TRANSMISSION_HANDSHAKE(bytes)
        onDATA_TRANSMISSION_HANDSHAKE.forEach { handler -> handler(pdata_transmission_handshake) }
    }

    override fun on_GPS_GLOBAL_ORIGIN(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pgps_global_origin = GPS_GLOBAL_ORIGIN(cur)
        onGPS_GLOBAL_ORIGIN.forEach { handler -> handler(pgps_global_origin) }
    }

    override fun on_SCALED_IMU2(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_imu2 = SCALED_IMU2(bytes)
        onSCALED_IMU2.forEach { handler -> handler(pscaled_imu2) }
    }

    override fun on_ATTITUDE_QUATERNION(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pattitude_quaternion = ATTITUDE_QUATERNION(bytes)
        onATTITUDE_QUATERNION.forEach { handler -> handler(pattitude_quaternion) }
    }

    override fun on_HIL_ACTUATOR_CONTROLS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val phil_actuator_controls = HIL_ACTUATOR_CONTROLS(cur)
        onHIL_ACTUATOR_CONTROLS.forEach { handler -> handler(phil_actuator_controls) }
    }

    override fun on_POSITION_TARGET_LOCAL_NED(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pposition_target_local_ned = POSITION_TARGET_LOCAL_NED(cur)
        onPOSITION_TARGET_LOCAL_NED.forEach { handler -> handler(pposition_target_local_ned) }
    }

    override fun on_DISTANCE_SENSOR(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pdistance_sensor = DISTANCE_SENSOR(cur)
        onDISTANCE_SENSOR.forEach { handler -> handler(pdistance_sensor) }
    }

    override fun on_HIL_OPTICAL_FLOW(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phil_optical_flow = HIL_OPTICAL_FLOW(bytes)
        onHIL_OPTICAL_FLOW.forEach { handler -> handler(phil_optical_flow) }
    }

    override fun on_SCALED_PRESSURE2(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pscaled_pressure2 = SCALED_PRESSURE2(bytes)
        onSCALED_PRESSURE2.forEach { handler -> handler(pscaled_pressure2) }
    }

    override fun on_WIND_COV(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pwind_cov = WIND_COV(bytes)
        onWIND_COV.forEach { handler -> handler(pwind_cov) }
    }

    override fun on_CHANGE_OPERATOR_CONTROL(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pchange_operator_control = CHANGE_OPERATOR_CONTROL(cur)
        onCHANGE_OPERATOR_CONTROL.forEach { handler -> handler(pchange_operator_control) }
    }

    override fun on_SYS_STATUS(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val psys_status = SYS_STATUS(cur)
        onSYS_STATUS.forEach { handler -> handler(psys_status) }
    }

    override fun on_MISSION_ITEM(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_item = MISSION_ITEM(cur)
        onMISSION_ITEM.forEach { handler -> handler(pmission_item) }
    }

    override fun on_RAW_IMU(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val praw_imu = RAW_IMU(bytes)
        onRAW_IMU.forEach { handler -> handler(praw_imu) }
    }

    override fun on_COMMAND_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pcommand_int = COMMAND_INT(cur)
        onCOMMAND_INT.forEach { handler -> handler(pcommand_int) }
    }

    override fun on_OPTICAL_FLOW(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val poptical_flow = OPTICAL_FLOW(cur)
        onOPTICAL_FLOW.forEach { handler -> handler(poptical_flow) }
    }

    override fun on_MISSION_ITEM_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pmission_item_int = MISSION_ITEM_INT(cur)
        onMISSION_ITEM_INT.forEach { handler -> handler(pmission_item_int) }
    }

    override fun on_HIGHRES_IMU(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val phighres_imu = HIGHRES_IMU(bytes)
        onHIGHRES_IMU.forEach { handler -> handler(phighres_imu) }
    }

    override fun on_EXTENDED_SYS_STATE(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pextended_sys_state = EXTENDED_SYS_STATE(cur)
        onEXTENDED_SYS_STATE.forEach { handler -> handler(pextended_sys_state) }
    }

    override fun on_GPS_INJECT_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps_inject_data = GPS_INJECT_DATA(bytes)
        onGPS_INJECT_DATA.forEach { handler -> handler(pgps_inject_data) }
    }

    override fun on_ATTITUDE_QUATERNION_COV(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pattitude_quaternion_cov = ATTITUDE_QUATERNION_COV(bytes)
        onATTITUDE_QUATERNION_COV.forEach { handler -> handler(pattitude_quaternion_cov) }
    }

    override fun on_NAMED_VALUE_INT(pack: AdHoc.Pack) {
        val cur = Cursor()
        cur.wrap(pack)
        val pnamed_value_int = NAMED_VALUE_INT(cur)
        onNAMED_VALUE_INT.forEach { handler -> handler(pnamed_value_int) }
    }

    override fun on_RADIO_STATUS(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pradio_status = RADIO_STATUS(bytes)
        onRADIO_STATUS.forEach { handler -> handler(pradio_status) }
    }

    override fun on_GPS_RTCM_DATA(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pgps_rtcm_data = GPS_RTCM_DATA(bytes)
        onGPS_RTCM_DATA.forEach { handler -> handler(pgps_rtcm_data) }
    }

    override fun on_GLOBAL_VISION_POSITION_ESTIMATE(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pglobal_vision_position_estimate = GLOBAL_VISION_POSITION_ESTIMATE(bytes)
        onGLOBAL_VISION_POSITION_ESTIMATE.forEach { handler -> handler(pglobal_vision_position_estimate) }
    }

    override fun on_FILE_TRANSFER_PROTOCOL(pack: AdHoc.Pack) {
        val bytes = AdHoc.Pack.Bytes()
        bytes.wrap(pack)
        val pfile_transfer_protocol = FILE_TRANSFER_PROTOCOL(bytes)
        onFILE_TRANSFER_PROTOCOL.forEach { handler -> handler(pfile_transfer_protocol) }
    }

    val onRESOURCE_REQUEST: MutableCollection<(RESOURCE_REQUEST) -> Unit> = ArrayList()
    val onATTITUDE_TARGET: MutableCollection<(ATTITUDE_TARGET) -> Unit> = ArrayList()
    val onMISSION_COUNT: MutableCollection<(MISSION_COUNT) -> Unit> = ArrayList()
    val onADSB_VEHICLE: MutableCollection<(ADSB_VEHICLE) -> Unit> = ArrayList()
    val onMESSAGE_INTERVAL: MutableCollection<(MESSAGE_INTERVAL) -> Unit> = ArrayList()
    val onESTIMATOR_STATUS: MutableCollection<(ESTIMATOR_STATUS) -> Unit> = ArrayList()
    val onTIMESYNC: MutableCollection<(TIMESYNC) -> Unit> = ArrayList()
    val onGLOBAL_POSITION_INT_COV: MutableCollection<(GLOBAL_POSITION_INT_COV) -> Unit> = ArrayList()
    val onBUTTON_CHANGE: MutableCollection<(BUTTON_CHANGE) -> Unit> = ArrayList()
    val onSAFETY_SET_ALLOWED_AREA: MutableCollection<(SAFETY_SET_ALLOWED_AREA) -> Unit> = ArrayList()
    val onSTORAGE_INFORMATION: MutableCollection<(STORAGE_INFORMATION) -> Unit> = ArrayList()
    val onCOLLISION: MutableCollection<(COLLISION) -> Unit> = ArrayList()
    val onALTITUDE: MutableCollection<(ALTITUDE) -> Unit> = ArrayList()
    val onHIL_STATE_QUATERNION: MutableCollection<(HIL_STATE_QUATERNION) -> Unit> = ArrayList()
    val onCAMERA_INFORMATION: MutableCollection<(CAMERA_INFORMATION) -> Unit> = ArrayList()
    val onGPS_STATUS: MutableCollection<(GPS_STATUS) -> Unit> = ArrayList()
    val onPARAM_SET: MutableCollection<(PARAM_SET) -> Unit> = ArrayList()
    val onTERRAIN_DATA: MutableCollection<(TERRAIN_DATA) -> Unit> = ArrayList()
    val onRC_CHANNELS_OVERRIDE: MutableCollection<(RC_CHANNELS_OVERRIDE) -> Unit> = ArrayList()
    val onSCALED_IMU: MutableCollection<(SCALED_IMU) -> Unit> = ArrayList()
    val onDEBUG: MutableCollection<(DEBUG) -> Unit> = ArrayList()
    val onCAMERA_IMAGE_CAPTURED: MutableCollection<(CAMERA_IMAGE_CAPTURED) -> Unit> = ArrayList()
    val onLOG_ENTRY: MutableCollection<(LOG_ENTRY) -> Unit> = ArrayList()
    val onACTUATOR_CONTROL_TARGET: MutableCollection<(ACTUATOR_CONTROL_TARGET) -> Unit> = ArrayList()
    val onHIGH_LATENCY: MutableCollection<(HIGH_LATENCY) -> Unit> = ArrayList()
    val onPARAM_REQUEST_READ: MutableCollection<(PARAM_REQUEST_READ) -> Unit> = ArrayList()
    val onSET_ATTITUDE_TARGET: MutableCollection<(SET_ATTITUDE_TARGET) -> Unit> = ArrayList()
    val onFOLLOW_TARGET: MutableCollection<(FOLLOW_TARGET) -> Unit> = ArrayList()
    val onHIL_STATE: MutableCollection<(HIL_STATE) -> Unit> = ArrayList()
    val onHOME_POSITION: MutableCollection<(HOME_POSITION) -> Unit> = ArrayList()
    val onGPS2_RAW: MutableCollection<(GPS2_RAW) -> Unit> = ArrayList()
    val onMEMORY_VECT: MutableCollection<(MEMORY_VECT) -> Unit> = ArrayList()
    val onREQUEST_DATA_STREAM: MutableCollection<(REQUEST_DATA_STREAM) -> Unit> = ArrayList()
    val onHIL_CONTROLS: MutableCollection<(HIL_CONTROLS) -> Unit> = ArrayList()
    val onHIL_SENSOR: MutableCollection<(HIL_SENSOR) -> Unit> = ArrayList()
    val onSETUP_SIGNING: MutableCollection<(SETUP_SIGNING) -> Unit> = ArrayList()
    val onGPS_RTK: MutableCollection<(GPS_RTK) -> Unit> = ArrayList()
    val onPARAM_REQUEST_LIST: MutableCollection<(PARAM_REQUEST_LIST) -> Unit> = ArrayList()
    val onLANDING_TARGET: MutableCollection<(LANDING_TARGET) -> Unit> = ArrayList()
    val onSET_ACTUATOR_CONTROL_TARGET: MutableCollection<(SET_ACTUATOR_CONTROL_TARGET) -> Unit> = ArrayList()
    val onCONTROL_SYSTEM_STATE: MutableCollection<(CONTROL_SYSTEM_STATE) -> Unit> = ArrayList()
    val onSET_POSITION_TARGET_GLOBAL_INT: MutableCollection<(SET_POSITION_TARGET_GLOBAL_INT) -> Unit> = ArrayList()
    val onVIBRATION: MutableCollection<(VIBRATION) -> Unit> = ArrayList()
    val onPING33: MutableCollection<(PING33) -> Unit> = ArrayList()
    val onVFR_HUD: MutableCollection<(VFR_HUD) -> Unit> = ArrayList()
    val onMISSION_SET_CURRENT: MutableCollection<(MISSION_SET_CURRENT) -> Unit> = ArrayList()
    val onHIL_GPS: MutableCollection<(HIL_GPS) -> Unit> = ArrayList()
    val onNAV_CONTROLLER_OUTPUT: MutableCollection<(NAV_CONTROLLER_OUTPUT) -> Unit> = ArrayList()
    val onAUTH_KEY: MutableCollection<(AUTH_KEY) -> Unit> = ArrayList()
    val onLOCAL_POSITION_NED_COV: MutableCollection<(LOCAL_POSITION_NED_COV) -> Unit> = ArrayList()
    val onATT_POS_MOCAP: MutableCollection<(ATT_POS_MOCAP) -> Unit> = ArrayList()
    val onSTATUSTEXT: MutableCollection<(STATUSTEXT) -> Unit> = ArrayList()
    val onPING: MutableCollection<(PING) -> Unit> = ArrayList()
    val onCAMERA_CAPTURE_STATUS: MutableCollection<(CAMERA_CAPTURE_STATUS) -> Unit> = ArrayList()
    val onGLOBAL_POSITION_INT: MutableCollection<(GLOBAL_POSITION_INT) -> Unit> = ArrayList()
    val onENCAPSULATED_DATA: MutableCollection<(ENCAPSULATED_DATA) -> Unit> = ArrayList()
    val onGPS_INPUT: MutableCollection<(GPS_INPUT) -> Unit> = ArrayList()
    val onCOMMAND_LONG: MutableCollection<(COMMAND_LONG) -> Unit> = ArrayList()
    val onLOG_REQUEST_DATA: MutableCollection<(LOG_REQUEST_DATA) -> Unit> = ArrayList()
    val onGPS_RAW_INT: MutableCollection<(GPS_RAW_INT) -> Unit> = ArrayList()
    val onRC_CHANNELS_SCALED: MutableCollection<(RC_CHANNELS_SCALED) -> Unit> = ArrayList()
    val onCAMERA_SETTINGS: MutableCollection<(CAMERA_SETTINGS) -> Unit> = ArrayList()
    val onRAW_PRESSURE: MutableCollection<(RAW_PRESSURE) -> Unit> = ArrayList()
    val onNAMED_VALUE_FLOAT: MutableCollection<(NAMED_VALUE_FLOAT) -> Unit> = ArrayList()
    val onATTITUDE: MutableCollection<(ATTITUDE) -> Unit> = ArrayList()
    val onTERRAIN_REQUEST: MutableCollection<(TERRAIN_REQUEST) -> Unit> = ArrayList()
    val onMISSION_WRITE_PARTIAL_LIST: MutableCollection<(MISSION_WRITE_PARTIAL_LIST) -> Unit> = ArrayList()
    val onLOG_ERASE: MutableCollection<(LOG_ERASE) -> Unit> = ArrayList()
    val onMANUAL_SETPOINT: MutableCollection<(MANUAL_SETPOINT) -> Unit> = ArrayList()
    val onSAFETY_ALLOWED_AREA: MutableCollection<(SAFETY_ALLOWED_AREA) -> Unit> = ArrayList()
    val onOPTICAL_FLOW_RAD: MutableCollection<(OPTICAL_FLOW_RAD) -> Unit> = ArrayList()
    val onLOG_DATA: MutableCollection<(LOG_DATA) -> Unit> = ArrayList()
    val onMISSION_CLEAR_ALL: MutableCollection<(MISSION_CLEAR_ALL) -> Unit> = ArrayList()
    val onVICON_POSITION_ESTIMATE: MutableCollection<(VICON_POSITION_ESTIMATE) -> Unit> = ArrayList()
    val onGPS2_RTK: MutableCollection<(GPS2_RTK) -> Unit> = ArrayList()
    val onLOG_REQUEST_LIST: MutableCollection<(LOG_REQUEST_LIST) -> Unit> = ArrayList()
    val onSCALED_PRESSURE: MutableCollection<(SCALED_PRESSURE) -> Unit> = ArrayList()
    val onMISSION_REQUEST_INT: MutableCollection<(MISSION_REQUEST_INT) -> Unit> = ArrayList()
    val onV2_EXTENSION: MutableCollection<(V2_EXTENSION) -> Unit> = ArrayList()
    val onHEARTBEAT: MutableCollection<(HEARTBEAT) -> Unit> = ArrayList()
    val onPARAM_MAP_RC: MutableCollection<(PARAM_MAP_RC) -> Unit> = ArrayList()
    val onPOWER_STATUS: MutableCollection<(POWER_STATUS) -> Unit> = ArrayList()
    val onTERRAIN_CHECK: MutableCollection<(TERRAIN_CHECK) -> Unit> = ArrayList()
    val onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: MutableCollection<(LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) -> Unit> = ArrayList()
    val onCOMMAND_ACK: MutableCollection<(COMMAND_ACK) -> Unit> = ArrayList()
    val onDATA_STREAM: MutableCollection<(DATA_STREAM) -> Unit> = ArrayList()
    val onMISSION_REQUEST: MutableCollection<(MISSION_REQUEST) -> Unit> = ArrayList()
    val onTERRAIN_REPORT: MutableCollection<(TERRAIN_REPORT) -> Unit> = ArrayList()
    val onSET_HOME_POSITION: MutableCollection<(SET_HOME_POSITION) -> Unit> = ArrayList()
    val onSwitchModeCommand: MutableCollection<() -> Unit> = ArrayList()
    val onHIL_RC_INPUTS_RAW: MutableCollection<(HIL_RC_INPUTS_RAW) -> Unit> = ArrayList()
    val onSCALED_IMU3: MutableCollection<(SCALED_IMU3) -> Unit> = ArrayList()
    val onSET_MODE: MutableCollection<(SET_MODE) -> Unit> = ArrayList()
    val onPOSITION_TARGET_GLOBAL_INT: MutableCollection<(POSITION_TARGET_GLOBAL_INT) -> Unit> = ArrayList()
    val onFLIGHT_INFORMATION: MutableCollection<(FLIGHT_INFORMATION) -> Unit> = ArrayList()
    val onSIM_STATE: MutableCollection<(SIM_STATE) -> Unit> = ArrayList()
    val onMISSION_ITEM_REACHED: MutableCollection<(MISSION_ITEM_REACHED) -> Unit> = ArrayList()
    val onRC_CHANNELS_RAW: MutableCollection<(RC_CHANNELS_RAW) -> Unit> = ArrayList()
    val onSERVO_OUTPUT_RAW: MutableCollection<(SERVO_OUTPUT_RAW) -> Unit> = ArrayList()
    val onVISION_SPEED_ESTIMATE: MutableCollection<(VISION_SPEED_ESTIMATE) -> Unit> = ArrayList()
    val onDEBUG_VECT: MutableCollection<(DEBUG_VECT) -> Unit> = ArrayList()
    val onLOG_REQUEST_END: MutableCollection<(LOG_REQUEST_END) -> Unit> = ArrayList()
    val onMISSION_ACK: MutableCollection<(MISSION_ACK) -> Unit> = ArrayList()
    val onCHANGE_OPERATOR_CONTROL_ACK: MutableCollection<(CHANGE_OPERATOR_CONTROL_ACK) -> Unit> = ArrayList()
    val onMISSION_CURRENT: MutableCollection<(MISSION_CURRENT) -> Unit> = ArrayList()
    val onSYSTEM_TIME: MutableCollection<(SYSTEM_TIME) -> Unit> = ArrayList()
    val onCAMERA_TRIGGER: MutableCollection<(CAMERA_TRIGGER) -> Unit> = ArrayList()
    val onVISION_POSITION_ESTIMATE: MutableCollection<(VISION_POSITION_ESTIMATE) -> Unit> = ArrayList()
    val onMANUAL_CONTROL: MutableCollection<(MANUAL_CONTROL) -> Unit> = ArrayList()
    val onRC_CHANNELS: MutableCollection<(RC_CHANNELS) -> Unit> = ArrayList()
    val onPARAM_VALUE: MutableCollection<(PARAM_VALUE) -> Unit> = ArrayList()
    val onBATTERY_STATUS: MutableCollection<(BATTERY_STATUS) -> Unit> = ArrayList()
    val onSET_POSITION_TARGET_LOCAL_NED: MutableCollection<(SET_POSITION_TARGET_LOCAL_NED) -> Unit> = ArrayList()
    val onSERIAL_CONTROL: MutableCollection<(SERIAL_CONTROL) -> Unit> = ArrayList()
    val onSET_GPS_GLOBAL_ORIGIN: MutableCollection<(SET_GPS_GLOBAL_ORIGIN) -> Unit> = ArrayList()
    val onAUTOPILOT_VERSION: MutableCollection<(AUTOPILOT_VERSION) -> Unit> = ArrayList()
    val onMISSION_REQUEST_LIST: MutableCollection<(MISSION_REQUEST_LIST) -> Unit> = ArrayList()
    val onPLAY_TUNE: MutableCollection<(PLAY_TUNE) -> Unit> = ArrayList()
    val onSCALED_PRESSURE3: MutableCollection<(SCALED_PRESSURE3) -> Unit> = ArrayList()
    val onMISSION_REQUEST_PARTIAL_LIST: MutableCollection<(MISSION_REQUEST_PARTIAL_LIST) -> Unit> = ArrayList()
    val onLOCAL_POSITION_NED: MutableCollection<(LOCAL_POSITION_NED) -> Unit> = ArrayList()
    val onDATA_TRANSMISSION_HANDSHAKE: MutableCollection<(DATA_TRANSMISSION_HANDSHAKE) -> Unit> = ArrayList()
    val onGPS_GLOBAL_ORIGIN: MutableCollection<(GPS_GLOBAL_ORIGIN) -> Unit> = ArrayList()
    val onSCALED_IMU2: MutableCollection<(SCALED_IMU2) -> Unit> = ArrayList()
    val onATTITUDE_QUATERNION: MutableCollection<(ATTITUDE_QUATERNION) -> Unit> = ArrayList()
    val onHIL_ACTUATOR_CONTROLS: MutableCollection<(HIL_ACTUATOR_CONTROLS) -> Unit> = ArrayList()
    val onPOSITION_TARGET_LOCAL_NED: MutableCollection<(POSITION_TARGET_LOCAL_NED) -> Unit> = ArrayList()
    val onDISTANCE_SENSOR: MutableCollection<(DISTANCE_SENSOR) -> Unit> = ArrayList()
    val onHIL_OPTICAL_FLOW: MutableCollection<(HIL_OPTICAL_FLOW) -> Unit> = ArrayList()
    val onSCALED_PRESSURE2: MutableCollection<(SCALED_PRESSURE2) -> Unit> = ArrayList()
    val onWIND_COV: MutableCollection<(WIND_COV) -> Unit> = ArrayList()
    val onCHANGE_OPERATOR_CONTROL: MutableCollection<(CHANGE_OPERATOR_CONTROL) -> Unit> = ArrayList()
    val onSYS_STATUS: MutableCollection<(SYS_STATUS) -> Unit> = ArrayList()
    val onMISSION_ITEM: MutableCollection<(MISSION_ITEM) -> Unit> = ArrayList()
    val onRAW_IMU: MutableCollection<(RAW_IMU) -> Unit> = ArrayList()
    val onCOMMAND_INT: MutableCollection<(COMMAND_INT) -> Unit> = ArrayList()
    val onOPTICAL_FLOW: MutableCollection<(OPTICAL_FLOW) -> Unit> = ArrayList()
    val onMISSION_ITEM_INT: MutableCollection<(MISSION_ITEM_INT) -> Unit> = ArrayList()
    val onHIGHRES_IMU: MutableCollection<(HIGHRES_IMU) -> Unit> = ArrayList()
    val onEXTENDED_SYS_STATE: MutableCollection<(EXTENDED_SYS_STATE) -> Unit> = ArrayList()
    val onGPS_INJECT_DATA: MutableCollection<(GPS_INJECT_DATA) -> Unit> = ArrayList()
    val onATTITUDE_QUATERNION_COV: MutableCollection<(ATTITUDE_QUATERNION_COV) -> Unit> = ArrayList()
    val onNAMED_VALUE_INT: MutableCollection<(NAMED_VALUE_INT) -> Unit> = ArrayList()
    val onRADIO_STATUS: MutableCollection<(RADIO_STATUS) -> Unit> = ArrayList()
    val onGPS_RTCM_DATA: MutableCollection<(GPS_RTCM_DATA) -> Unit> = ArrayList()
    val onGLOBAL_VISION_POSITION_ESTIMATE: MutableCollection<(GLOBAL_VISION_POSITION_ESTIMATE) -> Unit> = ArrayList()
    val onFILE_TRANSFER_PROTOCOL: MutableCollection<(FILE_TRANSFER_PROTOCOL) -> Unit> = ArrayList()

}


fun main() {
    val buff = ByteArray(1024)
    var bytes_out = 0
    val cur = Cursor()

    val CommunicationChannel_instance = CommunicationChannel_demo()

    CommunicationChannel_instance.onRESOURCE_REQUEST.add { pack -> onRESOURCE_REQUEST(pack) }
    CommunicationChannel_instance.onATTITUDE_TARGET.add { pack -> onATTITUDE_TARGET(pack) }
    CommunicationChannel_instance.onMISSION_COUNT.add { pack -> onMISSION_COUNT(pack) }
    CommunicationChannel_instance.onADSB_VEHICLE.add { pack -> onADSB_VEHICLE(pack) }
    CommunicationChannel_instance.onMESSAGE_INTERVAL.add { pack -> onMESSAGE_INTERVAL(pack) }
    CommunicationChannel_instance.onESTIMATOR_STATUS.add { pack -> onESTIMATOR_STATUS(pack) }
    CommunicationChannel_instance.onTIMESYNC.add { pack -> onTIMESYNC(pack) }
    CommunicationChannel_instance.onGLOBAL_POSITION_INT_COV.add { pack -> onGLOBAL_POSITION_INT_COV(pack) }
    CommunicationChannel_instance.onBUTTON_CHANGE.add { pack -> onBUTTON_CHANGE(pack) }
    CommunicationChannel_instance.onSAFETY_SET_ALLOWED_AREA.add { pack -> onSAFETY_SET_ALLOWED_AREA(pack) }
    CommunicationChannel_instance.onSTORAGE_INFORMATION.add { pack -> onSTORAGE_INFORMATION(pack) }
    CommunicationChannel_instance.onCOLLISION.add { pack -> onCOLLISION(pack) }
    CommunicationChannel_instance.onALTITUDE.add { pack -> onALTITUDE(pack) }
    CommunicationChannel_instance.onHIL_STATE_QUATERNION.add { pack -> onHIL_STATE_QUATERNION(pack) }
    CommunicationChannel_instance.onCAMERA_INFORMATION.add { pack -> onCAMERA_INFORMATION(pack) }
    CommunicationChannel_instance.onGPS_STATUS.add { pack -> onGPS_STATUS(pack) }
    CommunicationChannel_instance.onPARAM_SET.add { pack -> onPARAM_SET(pack) }
    CommunicationChannel_instance.onTERRAIN_DATA.add { pack -> onTERRAIN_DATA(pack) }
    CommunicationChannel_instance.onRC_CHANNELS_OVERRIDE.add { pack -> onRC_CHANNELS_OVERRIDE(pack) }
    CommunicationChannel_instance.onSCALED_IMU.add { pack -> onSCALED_IMU(pack) }
    CommunicationChannel_instance.onDEBUG.add { pack -> onDEBUG(pack) }
    CommunicationChannel_instance.onCAMERA_IMAGE_CAPTURED.add { pack -> onCAMERA_IMAGE_CAPTURED(pack) }
    CommunicationChannel_instance.onLOG_ENTRY.add { pack -> onLOG_ENTRY(pack) }
    CommunicationChannel_instance.onACTUATOR_CONTROL_TARGET.add { pack -> onACTUATOR_CONTROL_TARGET(pack) }
    CommunicationChannel_instance.onHIGH_LATENCY.add { pack -> onHIGH_LATENCY(pack) }
    CommunicationChannel_instance.onPARAM_REQUEST_READ.add { pack -> onPARAM_REQUEST_READ(pack) }
    CommunicationChannel_instance.onSET_ATTITUDE_TARGET.add { pack -> onSET_ATTITUDE_TARGET(pack) }
    CommunicationChannel_instance.onFOLLOW_TARGET.add { pack -> onFOLLOW_TARGET(pack) }
    CommunicationChannel_instance.onHIL_STATE.add { pack -> onHIL_STATE(pack) }
    CommunicationChannel_instance.onHOME_POSITION.add { pack -> onHOME_POSITION(pack) }
    CommunicationChannel_instance.onGPS2_RAW.add { pack -> onGPS2_RAW(pack) }
    CommunicationChannel_instance.onMEMORY_VECT.add { pack -> onMEMORY_VECT(pack) }
    CommunicationChannel_instance.onREQUEST_DATA_STREAM.add { pack -> onREQUEST_DATA_STREAM(pack) }
    CommunicationChannel_instance.onHIL_CONTROLS.add { pack -> onHIL_CONTROLS(pack) }
    CommunicationChannel_instance.onHIL_SENSOR.add { pack -> onHIL_SENSOR(pack) }
    CommunicationChannel_instance.onSETUP_SIGNING.add { pack -> onSETUP_SIGNING(pack) }
    CommunicationChannel_instance.onGPS_RTK.add { pack -> onGPS_RTK(pack) }
    CommunicationChannel_instance.onPARAM_REQUEST_LIST.add { pack -> onPARAM_REQUEST_LIST(pack) }
    CommunicationChannel_instance.onLANDING_TARGET.add { pack -> onLANDING_TARGET(pack) }
    CommunicationChannel_instance.onSET_ACTUATOR_CONTROL_TARGET.add { pack -> onSET_ACTUATOR_CONTROL_TARGET(pack) }
    CommunicationChannel_instance.onCONTROL_SYSTEM_STATE.add { pack -> onCONTROL_SYSTEM_STATE(pack) }
    CommunicationChannel_instance.onSET_POSITION_TARGET_GLOBAL_INT.add { pack -> onSET_POSITION_TARGET_GLOBAL_INT(pack) }
    CommunicationChannel_instance.onVIBRATION.add { pack -> onVIBRATION(pack) }
    CommunicationChannel_instance.onPING33.add { pack -> onPING33(pack) }
    CommunicationChannel_instance.onVFR_HUD.add { pack -> onVFR_HUD(pack) }
    CommunicationChannel_instance.onMISSION_SET_CURRENT.add { pack -> onMISSION_SET_CURRENT(pack) }
    CommunicationChannel_instance.onHIL_GPS.add { pack -> onHIL_GPS(pack) }
    CommunicationChannel_instance.onNAV_CONTROLLER_OUTPUT.add { pack -> onNAV_CONTROLLER_OUTPUT(pack) }
    CommunicationChannel_instance.onAUTH_KEY.add { pack -> onAUTH_KEY(pack) }
    CommunicationChannel_instance.onLOCAL_POSITION_NED_COV.add { pack -> onLOCAL_POSITION_NED_COV(pack) }
    CommunicationChannel_instance.onATT_POS_MOCAP.add { pack -> onATT_POS_MOCAP(pack) }
    CommunicationChannel_instance.onSTATUSTEXT.add { pack -> onSTATUSTEXT(pack) }
    CommunicationChannel_instance.onPING.add { pack -> onPING(pack) }
    CommunicationChannel_instance.onCAMERA_CAPTURE_STATUS.add { pack -> onCAMERA_CAPTURE_STATUS(pack) }
    CommunicationChannel_instance.onGLOBAL_POSITION_INT.add { pack -> onGLOBAL_POSITION_INT(pack) }
    CommunicationChannel_instance.onENCAPSULATED_DATA.add { pack -> onENCAPSULATED_DATA(pack) }
    CommunicationChannel_instance.onGPS_INPUT.add { pack -> onGPS_INPUT(pack) }
    CommunicationChannel_instance.onCOMMAND_LONG.add { pack -> onCOMMAND_LONG(pack) }
    CommunicationChannel_instance.onLOG_REQUEST_DATA.add { pack -> onLOG_REQUEST_DATA(pack) }
    CommunicationChannel_instance.onGPS_RAW_INT.add { pack -> onGPS_RAW_INT(pack) }
    CommunicationChannel_instance.onRC_CHANNELS_SCALED.add { pack -> onRC_CHANNELS_SCALED(pack) }
    CommunicationChannel_instance.onCAMERA_SETTINGS.add { pack -> onCAMERA_SETTINGS(pack) }
    CommunicationChannel_instance.onRAW_PRESSURE.add { pack -> onRAW_PRESSURE(pack) }
    CommunicationChannel_instance.onNAMED_VALUE_FLOAT.add { pack -> onNAMED_VALUE_FLOAT(pack) }
    CommunicationChannel_instance.onATTITUDE.add { pack -> onATTITUDE(pack) }
    CommunicationChannel_instance.onTERRAIN_REQUEST.add { pack -> onTERRAIN_REQUEST(pack) }
    CommunicationChannel_instance.onMISSION_WRITE_PARTIAL_LIST.add { pack -> onMISSION_WRITE_PARTIAL_LIST(pack) }
    CommunicationChannel_instance.onLOG_ERASE.add { pack -> onLOG_ERASE(pack) }
    CommunicationChannel_instance.onMANUAL_SETPOINT.add { pack -> onMANUAL_SETPOINT(pack) }
    CommunicationChannel_instance.onSAFETY_ALLOWED_AREA.add { pack -> onSAFETY_ALLOWED_AREA(pack) }
    CommunicationChannel_instance.onOPTICAL_FLOW_RAD.add { pack -> onOPTICAL_FLOW_RAD(pack) }
    CommunicationChannel_instance.onLOG_DATA.add { pack -> onLOG_DATA(pack) }
    CommunicationChannel_instance.onMISSION_CLEAR_ALL.add { pack -> onMISSION_CLEAR_ALL(pack) }
    CommunicationChannel_instance.onVICON_POSITION_ESTIMATE.add { pack -> onVICON_POSITION_ESTIMATE(pack) }
    CommunicationChannel_instance.onGPS2_RTK.add { pack -> onGPS2_RTK(pack) }
    CommunicationChannel_instance.onLOG_REQUEST_LIST.add { pack -> onLOG_REQUEST_LIST(pack) }
    CommunicationChannel_instance.onSCALED_PRESSURE.add { pack -> onSCALED_PRESSURE(pack) }
    CommunicationChannel_instance.onMISSION_REQUEST_INT.add { pack -> onMISSION_REQUEST_INT(pack) }
    CommunicationChannel_instance.onV2_EXTENSION.add { pack -> onV2_EXTENSION(pack) }
    CommunicationChannel_instance.onHEARTBEAT.add { pack -> onHEARTBEAT(pack) }
    CommunicationChannel_instance.onPARAM_MAP_RC.add { pack -> onPARAM_MAP_RC(pack) }
    CommunicationChannel_instance.onPOWER_STATUS.add { pack -> onPOWER_STATUS(pack) }
    CommunicationChannel_instance.onTERRAIN_CHECK.add { pack -> onTERRAIN_CHECK(pack) }
    CommunicationChannel_instance.onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add { pack -> onLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(pack) }
    CommunicationChannel_instance.onCOMMAND_ACK.add { pack -> onCOMMAND_ACK(pack) }
    CommunicationChannel_instance.onDATA_STREAM.add { pack -> onDATA_STREAM(pack) }
    CommunicationChannel_instance.onMISSION_REQUEST.add { pack -> onMISSION_REQUEST(pack) }
    CommunicationChannel_instance.onTERRAIN_REPORT.add { pack -> onTERRAIN_REPORT(pack) }
    CommunicationChannel_instance.onSET_HOME_POSITION.add { pack -> onSET_HOME_POSITION(pack) }
    CommunicationChannel_instance.onSwitchModeCommand.add { onSwitchModeCommand() }
    CommunicationChannel_instance.onHIL_RC_INPUTS_RAW.add { pack -> onHIL_RC_INPUTS_RAW(pack) }
    CommunicationChannel_instance.onSCALED_IMU3.add { pack -> onSCALED_IMU3(pack) }
    CommunicationChannel_instance.onSET_MODE.add { pack -> onSET_MODE(pack) }
    CommunicationChannel_instance.onPOSITION_TARGET_GLOBAL_INT.add { pack -> onPOSITION_TARGET_GLOBAL_INT(pack) }
    CommunicationChannel_instance.onFLIGHT_INFORMATION.add { pack -> onFLIGHT_INFORMATION(pack) }
    CommunicationChannel_instance.onSIM_STATE.add { pack -> onSIM_STATE(pack) }
    CommunicationChannel_instance.onMISSION_ITEM_REACHED.add { pack -> onMISSION_ITEM_REACHED(pack) }
    CommunicationChannel_instance.onRC_CHANNELS_RAW.add { pack -> onRC_CHANNELS_RAW(pack) }
    CommunicationChannel_instance.onSERVO_OUTPUT_RAW.add { pack -> onSERVO_OUTPUT_RAW(pack) }
    CommunicationChannel_instance.onVISION_SPEED_ESTIMATE.add { pack -> onVISION_SPEED_ESTIMATE(pack) }
    CommunicationChannel_instance.onDEBUG_VECT.add { pack -> onDEBUG_VECT(pack) }
    CommunicationChannel_instance.onLOG_REQUEST_END.add { pack -> onLOG_REQUEST_END(pack) }
    CommunicationChannel_instance.onMISSION_ACK.add { pack -> onMISSION_ACK(pack) }
    CommunicationChannel_instance.onCHANGE_OPERATOR_CONTROL_ACK.add { pack -> onCHANGE_OPERATOR_CONTROL_ACK(pack) }
    CommunicationChannel_instance.onMISSION_CURRENT.add { pack -> onMISSION_CURRENT(pack) }
    CommunicationChannel_instance.onSYSTEM_TIME.add { pack -> onSYSTEM_TIME(pack) }
    CommunicationChannel_instance.onCAMERA_TRIGGER.add { pack -> onCAMERA_TRIGGER(pack) }
    CommunicationChannel_instance.onVISION_POSITION_ESTIMATE.add { pack -> onVISION_POSITION_ESTIMATE(pack) }
    CommunicationChannel_instance.onMANUAL_CONTROL.add { pack -> onMANUAL_CONTROL(pack) }
    CommunicationChannel_instance.onRC_CHANNELS.add { pack -> onRC_CHANNELS(pack) }
    CommunicationChannel_instance.onPARAM_VALUE.add { pack -> onPARAM_VALUE(pack) }
    CommunicationChannel_instance.onBATTERY_STATUS.add { pack -> onBATTERY_STATUS(pack) }
    CommunicationChannel_instance.onSET_POSITION_TARGET_LOCAL_NED.add { pack -> onSET_POSITION_TARGET_LOCAL_NED(pack) }
    CommunicationChannel_instance.onSERIAL_CONTROL.add { pack -> onSERIAL_CONTROL(pack) }
    CommunicationChannel_instance.onSET_GPS_GLOBAL_ORIGIN.add { pack -> onSET_GPS_GLOBAL_ORIGIN(pack) }
    CommunicationChannel_instance.onAUTOPILOT_VERSION.add { pack -> onAUTOPILOT_VERSION(pack) }
    CommunicationChannel_instance.onMISSION_REQUEST_LIST.add { pack -> onMISSION_REQUEST_LIST(pack) }
    CommunicationChannel_instance.onPLAY_TUNE.add { pack -> onPLAY_TUNE(pack) }
    CommunicationChannel_instance.onSCALED_PRESSURE3.add { pack -> onSCALED_PRESSURE3(pack) }
    CommunicationChannel_instance.onMISSION_REQUEST_PARTIAL_LIST.add { pack -> onMISSION_REQUEST_PARTIAL_LIST(pack) }
    CommunicationChannel_instance.onLOCAL_POSITION_NED.add { pack -> onLOCAL_POSITION_NED(pack) }
    CommunicationChannel_instance.onDATA_TRANSMISSION_HANDSHAKE.add { pack -> onDATA_TRANSMISSION_HANDSHAKE(pack) }
    CommunicationChannel_instance.onGPS_GLOBAL_ORIGIN.add { pack -> onGPS_GLOBAL_ORIGIN(pack) }
    CommunicationChannel_instance.onSCALED_IMU2.add { pack -> onSCALED_IMU2(pack) }
    CommunicationChannel_instance.onATTITUDE_QUATERNION.add { pack -> onATTITUDE_QUATERNION(pack) }
    CommunicationChannel_instance.onHIL_ACTUATOR_CONTROLS.add { pack -> onHIL_ACTUATOR_CONTROLS(pack) }
    CommunicationChannel_instance.onPOSITION_TARGET_LOCAL_NED.add { pack -> onPOSITION_TARGET_LOCAL_NED(pack) }
    CommunicationChannel_instance.onDISTANCE_SENSOR.add { pack -> onDISTANCE_SENSOR(pack) }
    CommunicationChannel_instance.onHIL_OPTICAL_FLOW.add { pack -> onHIL_OPTICAL_FLOW(pack) }
    CommunicationChannel_instance.onSCALED_PRESSURE2.add { pack -> onSCALED_PRESSURE2(pack) }
    CommunicationChannel_instance.onWIND_COV.add { pack -> onWIND_COV(pack) }
    CommunicationChannel_instance.onCHANGE_OPERATOR_CONTROL.add { pack -> onCHANGE_OPERATOR_CONTROL(pack) }
    CommunicationChannel_instance.onSYS_STATUS.add { pack -> onSYS_STATUS(pack) }
    CommunicationChannel_instance.onMISSION_ITEM.add { pack -> onMISSION_ITEM(pack) }
    CommunicationChannel_instance.onRAW_IMU.add { pack -> onRAW_IMU(pack) }
    CommunicationChannel_instance.onCOMMAND_INT.add { pack -> onCOMMAND_INT(pack) }
    CommunicationChannel_instance.onOPTICAL_FLOW.add { pack -> onOPTICAL_FLOW(pack) }
    CommunicationChannel_instance.onMISSION_ITEM_INT.add { pack -> onMISSION_ITEM_INT(pack) }
    CommunicationChannel_instance.onHIGHRES_IMU.add { pack -> onHIGHRES_IMU(pack) }
    CommunicationChannel_instance.onEXTENDED_SYS_STATE.add { pack -> onEXTENDED_SYS_STATE(pack) }
    CommunicationChannel_instance.onGPS_INJECT_DATA.add { pack -> onGPS_INJECT_DATA(pack) }
    CommunicationChannel_instance.onATTITUDE_QUATERNION_COV.add { pack -> onATTITUDE_QUATERNION_COV(pack) }
    CommunicationChannel_instance.onNAMED_VALUE_INT.add { pack -> onNAMED_VALUE_INT(pack) }
    CommunicationChannel_instance.onRADIO_STATUS.add { pack -> onRADIO_STATUS(pack) }
    CommunicationChannel_instance.onGPS_RTCM_DATA.add { pack -> onGPS_RTCM_DATA(pack) }
    CommunicationChannel_instance.onGLOBAL_VISION_POSITION_ESTIMATE.add { pack -> onGLOBAL_VISION_POSITION_ESTIMATE(pack) }
    CommunicationChannel_instance.onFILE_TRANSFER_PROTOCOL.add { pack -> onFILE_TRANSFER_PROTOCOL(pack) }

    CommunicationChannel.NEW.FOLLOW_TARGET(cur).let { pfollow_target ->
        fill(pfollow_target)
        if (!CommunicationChannel_instance.send(pfollow_target))   //push pfollow_target pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.ADSB_VEHICLE(cur).let { padsb_vehicle ->
        fill(padsb_vehicle)
        if (!CommunicationChannel_instance.send(padsb_vehicle))   //push padsb_vehicle pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MESSAGE_INTERVAL(cur).let { pmessage_interval ->
        fill(pmessage_interval)
        if (!CommunicationChannel_instance.send(pmessage_interval))   //push pmessage_interval pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.EKF_STATUS_REPORT(cur).let { pekf_status_report ->
        fill(pekf_status_report)
        if (!CommunicationChannel_instance.send(pekf_status_report))   //push pekf_status_report pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.ESTIMATOR_STATUS(cur).let { pestimator_status ->
        fill(pestimator_status)
        if (!CommunicationChannel_instance.send(pestimator_status))   //push pestimator_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.HWSTATUS(cur).let { phwstatus ->
        fill(phwstatus)
        if (!CommunicationChannel_instance.send(phwstatus))   //push phwstatus pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.TIMESYNC(cur).let { ptimesync ->
        fill(ptimesync)
        if (!CommunicationChannel_instance.send(ptimesync))   //push ptimesync pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PARAM_EXT_REQUEST_LIST(cur).let { pparam_ext_request_list ->
        fill(pparam_ext_request_list)
        if (!CommunicationChannel_instance.send(pparam_ext_request_list))   //push pparam_ext_request_list pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.BUTTON_CHANGE(cur).let { pbutton_change ->
        fill(pbutton_change)
        if (!CommunicationChannel_instance.send(pbutton_change))   //push pbutton_change pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.UAVCAN_NODE_STATUS(cur).let { puavcan_node_status ->
        fill(puavcan_node_status)
        if (!CommunicationChannel_instance.send(puavcan_node_status))   //push puavcan_node_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.COLLISION(cur).let { pcollision ->
        fill(pcollision)
        if (!CommunicationChannel_instance.send(pcollision))   //push pcollision pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GIMBAL_TORQUE_CMD_REPORT(cur).let { pgimbal_torque_cmd_report ->
        fill(pgimbal_torque_cmd_report)
        if (!CommunicationChannel_instance.send(pgimbal_torque_cmd_report))   //push pgimbal_torque_cmd_report pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.ALTITUDE(cur).let { paltitude ->
        fill(paltitude)
        if (!CommunicationChannel_instance.send(paltitude))   //push paltitude pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.HIL_STATE_QUATERNION(cur).let { phil_state_quaternion ->
        fill(phil_state_quaternion)
        if (!CommunicationChannel_instance.send(phil_state_quaternion))   //push phil_state_quaternion pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SENSOR_OFFSETS(cur).let { psensor_offsets ->
        fill(psensor_offsets)
        if (!CommunicationChannel_instance.send(psensor_offsets))   //push psensor_offsets pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.STORAGE_INFORMATION(cur).let { pstorage_information ->
        fill(pstorage_information)
        if (!CommunicationChannel_instance.send(pstorage_information))   //push pstorage_information pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.CAMERA_INFORMATION(cur).let { pcamera_information ->
        fill(pcamera_information)
        if (!CommunicationChannel_instance.send(pcamera_information))   //push pcamera_information pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DEVICE_OP_WRITE_REPLY(cur).let { pdevice_op_write_reply ->
        fill(pdevice_op_write_reply)
        if (!CommunicationChannel_instance.send(pdevice_op_write_reply))   //push pdevice_op_write_reply pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.TERRAIN_DATA(cur).let { pterrain_data ->
        fill(pterrain_data)
        if (!CommunicationChannel_instance.send(pterrain_data))   //push pterrain_data pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GIMBAL_CONTROL(cur).let { pgimbal_control ->
        fill(pgimbal_control)
        if (!CommunicationChannel_instance.send(pgimbal_control))   //push pgimbal_control pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.VIDEO_STREAM_INFORMATION(cur).let { pvideo_stream_information ->
        fill(pvideo_stream_information)
        if (!CommunicationChannel_instance.send(pvideo_stream_information))   //push pvideo_stream_information pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.AHRS(cur).let { pahrs ->
        fill(pahrs)
        if (!CommunicationChannel_instance.send(pahrs))   //push pahrs pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DEBUG(cur).let { pdebug ->
        fill(pdebug)
        if (!CommunicationChannel_instance.send(pdebug))   //push pdebug pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.CAMERA_IMAGE_CAPTURED(cur).let { pcamera_image_captured ->
        fill(pcamera_image_captured)
        if (!CommunicationChannel_instance.send(pcamera_image_captured))   //push pcamera_image_captured pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOG_ENTRY(cur).let { plog_entry ->
        fill(plog_entry)
        if (!CommunicationChannel_instance.send(plog_entry))   //push plog_entry pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.ACTUATOR_CONTROL_TARGET(cur).let { pactuator_control_target ->
        fill(pactuator_control_target)
        if (!CommunicationChannel_instance.send(pactuator_control_target))   //push pactuator_control_target pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.HIGH_LATENCY(cur).let { phigh_latency ->
        fill(phigh_latency)
        if (!CommunicationChannel_instance.send(phigh_latency))   //push phigh_latency pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.HOME_POSITION(cur).let { phome_position ->
        fill(phome_position)
        if (!CommunicationChannel_instance.send(phome_position))   //push phome_position pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.FENCE_STATUS(cur).let { pfence_status ->
        fill(pfence_status)
        if (!CommunicationChannel_instance.send(pfence_status))   //push pfence_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.REMOTE_LOG_BLOCK_STATUS(cur).let { premote_log_block_status ->
        fill(premote_log_block_status)
        if (!CommunicationChannel_instance.send(premote_log_block_status))   //push premote_log_block_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.OBSTACLE_DISTANCE(cur).let { pobstacle_distance ->
        fill(pobstacle_distance)
        if (!CommunicationChannel_instance.send(pobstacle_distance))   //push pobstacle_distance pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GPS2_RAW(cur).let { pgps2_raw ->
        fill(pgps2_raw)
        if (!CommunicationChannel_instance.send(pgps2_raw))   //push pgps2_raw pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MEMORY_VECT(cur).let { pmemory_vect ->
        fill(pmemory_vect)
        if (!CommunicationChannel_instance.send(pmemory_vect))   //push pmemory_vect pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PARAM_EXT_REQUEST_READ(cur).let { pparam_ext_request_read ->
        fill(pparam_ext_request_read)
        if (!CommunicationChannel_instance.send(pparam_ext_request_read))   //push pparam_ext_request_read pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.HIL_SENSOR(cur).let { phil_sensor ->
        fill(phil_sensor)
        if (!CommunicationChannel_instance.send(phil_sensor))   //push phil_sensor pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SETUP_SIGNING(cur).let { psetup_signing ->
        fill(psetup_signing)
        if (!CommunicationChannel_instance.send(psetup_signing))   //push psetup_signing pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GPS_RTK(cur).let { pgps_rtk ->
        fill(pgps_rtk)
        if (!CommunicationChannel_instance.send(pgps_rtk))   //push pgps_rtk pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_CFG(cur).let { puavionix_adsb_out_cfg ->
        fill(puavionix_adsb_out_cfg)
        if (!CommunicationChannel_instance.send(puavionix_adsb_out_cfg))   //push puavionix_adsb_out_cfg pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LANDING_TARGET(cur).let { planding_target ->
        fill(planding_target)
        if (!CommunicationChannel_instance.send(planding_target))   //push planding_target pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SET_ACTUATOR_CONTROL_TARGET(cur).let { pset_actuator_control_target ->
        fill(pset_actuator_control_target)
        if (!CommunicationChannel_instance.send(pset_actuator_control_target))   //push pset_actuator_control_target pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.CONTROL_SYSTEM_STATE(cur).let { pcontrol_system_state ->
        fill(pcontrol_system_state)
        if (!CommunicationChannel_instance.send(pcontrol_system_state))   //push pcontrol_system_state pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DATA32(cur).let { pdata32 ->
        fill(pdata32)
        if (!CommunicationChannel_instance.send(pdata32))   //push pdata32 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PING33(cur).let { pping33 ->
        fill(pping33)
        if (!CommunicationChannel_instance.send(pping33))   //push pping33 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.RALLY_POINT(cur).let { prally_point ->
        fill(prally_point)
        if (!CommunicationChannel_instance.send(prally_point))   //push prally_point pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.ADAP_TUNING(cur).let { padap_tuning ->
        fill(padap_tuning)
        if (!CommunicationChannel_instance.send(padap_tuning))   //push padap_tuning pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.VIBRATION(cur).let { pvibration ->
        fill(pvibration)
        if (!CommunicationChannel_instance.send(pvibration))   //push pvibration pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PARAM_EXT_VALUE(cur).let { pparam_ext_value ->
        fill(pparam_ext_value)
        if (!CommunicationChannel_instance.send(pparam_ext_value))   //push pparam_ext_value pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.BATTERY2(cur).let { pbattery2 ->
        fill(pbattery2)
        if (!CommunicationChannel_instance.send(pbattery2))   //push pbattery2 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LIMITS_STATUS(cur).let { plimits_status ->
        fill(plimits_status)
        if (!CommunicationChannel_instance.send(plimits_status))   //push plimits_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.CAMERA_FEEDBACK(cur).let { pcamera_feedback ->
        fill(pcamera_feedback)
        if (!CommunicationChannel_instance.send(pcamera_feedback))   //push pcamera_feedback pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.HIL_GPS(cur).let { phil_gps ->
        fill(phil_gps)
        if (!CommunicationChannel_instance.send(phil_gps))   //push phil_gps pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.FENCE_FETCH_POINT(cur).let { pfence_fetch_point ->
        fill(pfence_fetch_point)
        if (!CommunicationChannel_instance.send(pfence_fetch_point))   //push pfence_fetch_point pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.RADIO(cur).let { pradio ->
        fill(pradio)
        if (!CommunicationChannel_instance.send(pradio))   //push pradio pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.AIRSPEED_AUTOCAL(cur).let { pairspeed_autocal ->
        fill(pairspeed_autocal)
        if (!CommunicationChannel_instance.send(pairspeed_autocal))   //push pairspeed_autocal pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.ATT_POS_MOCAP(cur).let { patt_pos_mocap ->
        fill(patt_pos_mocap)
        if (!CommunicationChannel_instance.send(patt_pos_mocap))   //push patt_pos_mocap pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.STATUSTEXT(cur).let { pstatustext ->
        fill(pstatustext)
        if (!CommunicationChannel_instance.send(pstatustext))   //push pstatustext pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GOPRO_GET_REQUEST(cur).let { pgopro_get_request ->
        fill(pgopro_get_request)
        if (!CommunicationChannel_instance.send(pgopro_get_request))   //push pgopro_get_request pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.CAMERA_CAPTURE_STATUS(cur).let { pcamera_capture_status ->
        fill(pcamera_capture_status)
        if (!CommunicationChannel_instance.send(pcamera_capture_status))   //push pcamera_capture_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.ENCAPSULATED_DATA(cur).let { pencapsulated_data ->
        fill(pencapsulated_data)
        if (!CommunicationChannel_instance.send(pencapsulated_data))   //push pencapsulated_data pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GPS_INPUT(cur).let { pgps_input ->
        fill(pgps_input)
        if (!CommunicationChannel_instance.send(pgps_input))   //push pgps_input pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.COMPASSMOT_STATUS(cur).let { pcompassmot_status ->
        fill(pcompassmot_status)
        if (!CommunicationChannel_instance.send(pcompassmot_status))   //push pcompassmot_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOG_REQUEST_DATA(cur).let { plog_request_data ->
        fill(plog_request_data)
        if (!CommunicationChannel_instance.send(plog_request_data))   //push plog_request_data pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.CAMERA_STATUS(cur).let { pcamera_status ->
        fill(pcamera_status)
        if (!CommunicationChannel_instance.send(pcamera_status))   //push pcamera_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.CAMERA_SETTINGS(cur).let { pcamera_settings ->
        fill(pcamera_settings)
        if (!CommunicationChannel_instance.send(pcamera_settings))   //push pcamera_settings pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DEVICE_OP_READ_REPLY(cur).let { pdevice_op_read_reply ->
        fill(pdevice_op_read_reply)
        if (!CommunicationChannel_instance.send(pdevice_op_read_reply))   //push pdevice_op_read_reply pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DIGICAM_CONTROL(cur).let { pdigicam_control ->
        fill(pdigicam_control)
        if (!CommunicationChannel_instance.send(pdigicam_control))   //push pdigicam_control pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.NAMED_VALUE_FLOAT(cur).let { pnamed_value_float ->
        fill(pnamed_value_float)
        if (!CommunicationChannel_instance.send(pnamed_value_float))   //push pnamed_value_float pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GOPRO_HEARTBEAT(cur).let { pgopro_heartbeat ->
        fill(pgopro_heartbeat)
        if (!CommunicationChannel_instance.send(pgopro_heartbeat))   //push pgopro_heartbeat pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.AHRS2(cur).let { pahrs2 ->
        fill(pahrs2)
        if (!CommunicationChannel_instance.send(pahrs2))   //push pahrs2 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOG_ERASE(cur).let { plog_erase ->
        fill(plog_erase)
        if (!CommunicationChannel_instance.send(plog_erase))   //push plog_erase pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.TERRAIN_REQUEST(cur).let { pterrain_request ->
        fill(pterrain_request)
        if (!CommunicationChannel_instance.send(pterrain_request))   //push pterrain_request pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MOUNT_STATUS(cur).let { pmount_status ->
        fill(pmount_status)
        if (!CommunicationChannel_instance.send(pmount_status))   //push pmount_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PID_TUNING(cur).let { ppid_tuning ->
        fill(ppid_tuning)
        if (!CommunicationChannel_instance.send(ppid_tuning))   //push ppid_tuning pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.OPTICAL_FLOW_RAD(cur).let { poptical_flow_rad ->
        fill(poptical_flow_rad)
        if (!CommunicationChannel_instance.send(poptical_flow_rad))   //push poptical_flow_rad pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOG_DATA(cur).let { plog_data ->
        fill(plog_data)
        if (!CommunicationChannel_instance.send(plog_data))   //push plog_data pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.AHRS3(cur).let { pahrs3 ->
        fill(pahrs3)
        if (!CommunicationChannel_instance.send(pahrs3))   //push pahrs3 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.VICON_POSITION_ESTIMATE(cur).let { pvicon_position_estimate ->
        fill(pvicon_position_estimate)
        if (!CommunicationChannel_instance.send(pvicon_position_estimate))   //push pvicon_position_estimate pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GPS2_RTK(cur).let { pgps2_rtk ->
        fill(pgps2_rtk)
        if (!CommunicationChannel_instance.send(pgps2_rtk))   //push pgps2_rtk pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MAG_CAL_REPORT(cur).let { pmag_cal_report ->
        fill(pmag_cal_report)
        if (!CommunicationChannel_instance.send(pmag_cal_report))   //push pmag_cal_report pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOG_REQUEST_LIST(cur).let { plog_request_list ->
        fill(plog_request_list)
        if (!CommunicationChannel_instance.send(plog_request_list))   //push plog_request_list pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MOUNT_CONFIGURE(cur).let { pmount_configure ->
        fill(pmount_configure)
        if (!CommunicationChannel_instance.send(pmount_configure))   //push pmount_configure pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.V2_EXTENSION(cur).let { pv2_extension ->
        fill(pv2_extension)
        if (!CommunicationChannel_instance.send(pv2_extension))   //push pv2_extension pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.POWER_STATUS(cur).let { ppower_status ->
        fill(ppower_status)
        if (!CommunicationChannel_instance.send(ppower_status))   //push ppower_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.REMOTE_LOG_DATA_BLOCK(cur).let { premote_log_data_block ->
        fill(premote_log_data_block)
        if (!CommunicationChannel_instance.send(premote_log_data_block))   //push premote_log_data_block pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOGGING_DATA_ACKED(cur).let { plogging_data_acked ->
        fill(plogging_data_acked)
        if (!CommunicationChannel_instance.send(plogging_data_acked))   //push plogging_data_acked pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.TERRAIN_CHECK(cur).let { pterrain_check ->
        fill(pterrain_check)
        if (!CommunicationChannel_instance.send(pterrain_check))   //push pterrain_check pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.TERRAIN_REPORT(cur).let { pterrain_report ->
        fill(pterrain_report)
        if (!CommunicationChannel_instance.send(pterrain_report))   //push pterrain_report pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SET_HOME_POSITION(cur).let { pset_home_position ->
        fill(pset_home_position)
        if (!CommunicationChannel_instance.send(pset_home_position))   //push pset_home_position pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }
    if (!CommunicationChannel_instance.sendSwitchModeCommand())
        throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")

    CommunicationChannel.NEW.SCALED_IMU3(cur).let { pscaled_imu3 ->
        fill(pscaled_imu3)
        if (!CommunicationChannel_instance.send(pscaled_imu3))   //push pscaled_imu3 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MOUNT_CONTROL(cur).let { pmount_control ->
        fill(pmount_control)
        if (!CommunicationChannel_instance.send(pmount_control))   //push pmount_control pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LED_CONTROL(cur).let { pled_control ->
        fill(pled_control)
        if (!CommunicationChannel_instance.send(pled_control))   //push pled_control pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SIM_STATE(cur).let { psim_state ->
        fill(psim_state)
        if (!CommunicationChannel_instance.send(psim_state))   //push psim_state pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.WIFI_CONFIG_AP(cur).let { pwifi_config_ap ->
        fill(pwifi_config_ap)
        if (!CommunicationChannel_instance.send(pwifi_config_ap))   //push pwifi_config_ap pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DATA96(cur).let { pdata96 ->
        fill(pdata96)
        if (!CommunicationChannel_instance.send(pdata96))   //push pdata96 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.FLIGHT_INFORMATION(cur).let { pflight_information ->
        fill(pflight_information)
        if (!CommunicationChannel_instance.send(pflight_information))   //push pflight_information pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MEMINFO(cur).let { pmeminfo ->
        fill(pmeminfo)
        if (!CommunicationChannel_instance.send(pmeminfo))   //push pmeminfo pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOGGING_ACK(cur).let { plogging_ack ->
        fill(plogging_ack)
        if (!CommunicationChannel_instance.send(plogging_ack))   //push plogging_ack pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.VISION_SPEED_ESTIMATE(cur).let { pvision_speed_estimate ->
        fill(pvision_speed_estimate)
        if (!CommunicationChannel_instance.send(pvision_speed_estimate))   //push pvision_speed_estimate pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DEBUG_VECT(cur).let { pdebug_vect ->
        fill(pdebug_vect)
        if (!CommunicationChannel_instance.send(pdebug_vect))   //push pdebug_vect pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.CAMERA_TRIGGER(cur).let { pcamera_trigger ->
        fill(pcamera_trigger)
        if (!CommunicationChannel_instance.send(pcamera_trigger))   //push pcamera_trigger pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOG_REQUEST_END(cur).let { plog_request_end ->
        fill(plog_request_end)
        if (!CommunicationChannel_instance.send(plog_request_end))   //push plog_request_end pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GOPRO_SET_RESPONSE(cur).let { pgopro_set_response ->
        fill(pgopro_set_response)
        if (!CommunicationChannel_instance.send(pgopro_set_response))   //push pgopro_set_response pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PROTOCOL_VERSION(cur).let { pprotocol_version ->
        fill(pprotocol_version)
        if (!CommunicationChannel_instance.send(pprotocol_version))   //push pprotocol_version pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.RALLY_FETCH_POINT(cur).let { prally_fetch_point ->
        fill(prally_fetch_point)
        if (!CommunicationChannel_instance.send(prally_fetch_point))   //push prally_fetch_point pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.BATTERY_STATUS(cur).let { pbattery_status ->
        fill(pbattery_status)
        if (!CommunicationChannel_instance.send(pbattery_status))   //push pbattery_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MOUNT_ORIENTATION(cur).let { pmount_orientation ->
        fill(pmount_orientation)
        if (!CommunicationChannel_instance.send(pmount_orientation))   //push pmount_orientation pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SERIAL_CONTROL(cur).let { pserial_control ->
        fill(pserial_control)
        if (!CommunicationChannel_instance.send(pserial_control))   //push pserial_control pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PARAM_EXT_SET(cur).let { pparam_ext_set ->
        fill(pparam_ext_set)
        if (!CommunicationChannel_instance.send(pparam_ext_set))   //push pparam_ext_set pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.AUTOPILOT_VERSION(cur).let { pautopilot_version ->
        fill(pautopilot_version)
        if (!CommunicationChannel_instance.send(pautopilot_version))   //push pautopilot_version pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SIMSTATE(cur).let { psimstate ->
        fill(psimstate)
        if (!CommunicationChannel_instance.send(psimstate))   //push psimstate pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SET_VIDEO_STREAM_SETTINGS(cur).let { pset_video_stream_settings ->
        fill(pset_video_stream_settings)
        if (!CommunicationChannel_instance.send(pset_video_stream_settings))   //push pset_video_stream_settings pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PLAY_TUNE(cur).let { pplay_tune ->
        fill(pplay_tune)
        if (!CommunicationChannel_instance.send(pplay_tune))   //push pplay_tune pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DIGICAM_CONFIGURE(cur).let { pdigicam_configure ->
        fill(pdigicam_configure)
        if (!CommunicationChannel_instance.send(pdigicam_configure))   //push pdigicam_configure pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SCALED_PRESSURE3(cur).let { pscaled_pressure3 ->
        fill(pscaled_pressure3)
        if (!CommunicationChannel_instance.send(pscaled_pressure3))   //push pscaled_pressure3 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.PARAM_EXT_ACK(cur).let { pparam_ext_ack ->
        fill(pparam_ext_ack)
        if (!CommunicationChannel_instance.send(pparam_ext_ack))   //push pparam_ext_ack pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.UAVCAN_NODE_INFO(cur).let { puavcan_node_info ->
        fill(puavcan_node_info)
        if (!CommunicationChannel_instance.send(puavcan_node_info))   //push puavcan_node_info pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DATA16(cur).let { pdata16 ->
        fill(pdata16)
        if (!CommunicationChannel_instance.send(pdata16))   //push pdata16 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SET_MAG_OFFSETS(cur).let { pset_mag_offsets ->
        fill(pset_mag_offsets)
        if (!CommunicationChannel_instance.send(pset_mag_offsets))   //push pset_mag_offsets pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SCALED_IMU2(cur).let { pscaled_imu2 ->
        fill(pscaled_imu2)
        if (!CommunicationChannel_instance.send(pscaled_imu2))   //push pscaled_imu2 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.AP_ADC(cur).let { pap_adc ->
        fill(pap_adc)
        if (!CommunicationChannel_instance.send(pap_adc))   //push pap_adc pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.WIND(cur).let { pwind ->
        fill(pwind)
        if (!CommunicationChannel_instance.send(pwind))   //push pwind pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.AUTOPILOT_VERSION_REQUEST(cur).let { pautopilot_version_request ->
        fill(pautopilot_version_request)
        if (!CommunicationChannel_instance.send(pautopilot_version_request))   //push pautopilot_version_request pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DATA_TRANSMISSION_HANDSHAKE(cur).let { pdata_transmission_handshake ->
        fill(pdata_transmission_handshake)
        if (!CommunicationChannel_instance.send(pdata_transmission_handshake))   //push pdata_transmission_handshake pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DATA64(cur).let { pdata64 ->
        fill(pdata64)
        if (!CommunicationChannel_instance.send(pdata64))   //push pdata64 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GIMBAL_REPORT(cur).let { pgimbal_report ->
        fill(pgimbal_report)
        if (!CommunicationChannel_instance.send(pgimbal_report))   //push pgimbal_report pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DEVICE_OP_WRITE(cur).let { pdevice_op_write ->
        fill(pdevice_op_write)
        if (!CommunicationChannel_instance.send(pdevice_op_write))   //push pdevice_op_write pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DISTANCE_SENSOR(cur).let { pdistance_sensor ->
        fill(pdistance_sensor)
        if (!CommunicationChannel_instance.send(pdistance_sensor))   //push pdistance_sensor pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.HIL_OPTICAL_FLOW(cur).let { phil_optical_flow ->
        fill(phil_optical_flow)
        if (!CommunicationChannel_instance.send(phil_optical_flow))   //push phil_optical_flow pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.SCALED_PRESSURE2(cur).let { pscaled_pressure2 ->
        fill(pscaled_pressure2)
        if (!CommunicationChannel_instance.send(pscaled_pressure2))   //push pscaled_pressure2 pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.WIND_COV(cur).let { pwind_cov ->
        fill(pwind_cov)
        if (!CommunicationChannel_instance.send(pwind_cov))   //push pwind_cov pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GOPRO_SET_REQUEST(cur).let { pgopro_set_request ->
        fill(pgopro_set_request)
        if (!CommunicationChannel_instance.send(pgopro_set_request))   //push pgopro_set_request pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.VISION_POSITION_DELTA(cur).let { pvision_position_delta ->
        fill(pvision_position_delta)
        if (!CommunicationChannel_instance.send(pvision_position_delta))   //push pvision_position_delta pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.LOGGING_DATA(cur).let { plogging_data ->
        fill(plogging_data)
        if (!CommunicationChannel_instance.send(plogging_data))   //push plogging_data pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.DEVICE_OP_READ(cur).let { pdevice_op_read ->
        fill(pdevice_op_read)
        if (!CommunicationChannel_instance.send(pdevice_op_read))   //push pdevice_op_read pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.MAG_CAL_PROGRESS(cur).let { pmag_cal_progress ->
        fill(pmag_cal_progress)
        if (!CommunicationChannel_instance.send(pmag_cal_progress))   //push pmag_cal_progress pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.HIGHRES_IMU(cur).let { phighres_imu ->
        fill(phighres_imu)
        if (!CommunicationChannel_instance.send(phighres_imu))   //push phighres_imu pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.EXTENDED_SYS_STATE(cur).let { pextended_sys_state ->
        fill(pextended_sys_state)
        if (!CommunicationChannel_instance.send(pextended_sys_state))   //push pextended_sys_state pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.UAVIONIX_ADSB_OUT_DYNAMIC(cur).let { puavionix_adsb_out_dynamic ->
        fill(puavionix_adsb_out_dynamic)
        if (!CommunicationChannel_instance.send(puavionix_adsb_out_dynamic))   //push puavionix_adsb_out_dynamic pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GOPRO_GET_RESPONSE(cur).let { pgopro_get_response ->
        fill(pgopro_get_response)
        if (!CommunicationChannel_instance.send(pgopro_get_response))   //push pgopro_get_response pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GPS_INJECT_DATA(cur).let { pgps_inject_data ->
        fill(pgps_inject_data)
        if (!CommunicationChannel_instance.send(pgps_inject_data))   //push pgps_inject_data pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur).let { puavionix_adsb_transceiver_health_report ->
        fill(puavionix_adsb_transceiver_health_report)
        if (!CommunicationChannel_instance.send(puavionix_adsb_transceiver_health_report))   //push puavionix_adsb_transceiver_health_report pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.NAMED_VALUE_INT(cur).let { pnamed_value_int ->
        fill(pnamed_value_int)
        if (!CommunicationChannel_instance.send(pnamed_value_int))   //push pnamed_value_int pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.RPM(cur).let { prpm ->
        fill(prpm)
        if (!CommunicationChannel_instance.send(prpm))   //push prpm pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.GPS_RTCM_DATA(cur).let { pgps_rtcm_data ->
        fill(pgps_rtcm_data)
        if (!CommunicationChannel_instance.send(pgps_rtcm_data))   //push pgps_rtcm_data pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.FILE_TRANSFER_PROTOCOL(cur).let { pfile_transfer_protocol ->
        fill(pfile_transfer_protocol)
        if (!CommunicationChannel_instance.send(pfile_transfer_protocol))   //push pfile_transfer_protocol pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.RANGEFINDER(cur).let { prangefinder ->
        fill(prangefinder)
        if (!CommunicationChannel_instance.send(prangefinder))   //push prangefinder pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.RADIO_STATUS(cur).let { pradio_status ->
        fill(pradio_status)
        if (!CommunicationChannel_instance.send(pradio_status))   //push pradio_status pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.FENCE_POINT(cur).let { pfence_point ->
        fill(pfence_point)
        if (!CommunicationChannel_instance.send(pfence_point))   //push pfence_point pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    CommunicationChannel.NEW.RESOURCE_REQUEST(cur).let { presource_request ->
        fill(presource_request)
        if (!CommunicationChannel_instance.send(presource_request))   //push presource_request pack into the channel send out buffer
            throw RuntimeException("error AD_HOC_FAILURE_SENDING_QUEUE_OVERFLOW")
    }

    bytes_out = CommunicationChannel_instance.transmitter.read(buff) // sending out packs
    CommunicationChannel_instance.receiver.write(buff) // receiving packs

}
					  