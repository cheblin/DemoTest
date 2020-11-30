use ad_hoc_gen::com::company::demo as host;
use host::GroundControl as packs;
use ad_hoc_gen::sys;
use std::sync::mpsc::*;
use host::CommunicationChannel;

const some_usize: usize = 0_usize;
const some_i8: i8 = 0i8;
const some_i16s: [i16; 1] = [0i16; 1];
const some_f32s: [f32; 1] = [0f32; 1];
const some_i16: i16 = 0i16;
const some_f32: f32 = 0f32;
const some_str: &str = &"";
const some_i64: i64 = 0i64;
const some_bool: bool = true;
const some_i32: i32 = 0i32;
const some_i8s: [i8; 1] = [0i8; 1];

use host::CommunicationChannel::IReceiver as CommunicationChannel_IReceiver;

pub struct CommunicationChannel_receiver {
    pub receiver: sys::Receiver,
}

impl CommunicationChannel_receiver {
    pub fn new() -> CommunicationChannel_receiver { CommunicationChannel_receiver { receiver: <CommunicationChannel_receiver as CommunicationChannel::IReceiver>::new() } }
}

impl host::CommunicationChannel::IReceiver for CommunicationChannel_receiver {
    unsafe fn into_self<'a>(receiver: *mut sys::Receiver) -> &'a mut Self { ad_hoc_gen::sys::self_by_field_ptr!(receiver, receiver) }
    fn get_receiver(&mut self) -> &mut sys::Receiver { &mut self.receiver }

    fn on_RESOURCE_REQUEST(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::RESOURCE_REQUEST::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_RESOURCE_REQUEST(&mut data);
        println!("RESOURCE_REQUEST")
    }

    fn on_ATTITUDE_TARGET(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::ATTITUDE_TARGET::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_ATTITUDE_TARGET(&mut data);
        println!("ATTITUDE_TARGET")
    }

    fn on_MISSION_COUNT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_COUNT::Data_(curs.wrap(pack));
        read_MISSION_COUNT(&mut data);
        println!("MISSION_COUNT")
    }

    fn on_ADSB_VEHICLE(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::ADSB_VEHICLE::Data_(curs.wrap(pack));
        read_ADSB_VEHICLE(&mut data);
        println!("ADSB_VEHICLE")
    }

    fn on_MESSAGE_INTERVAL(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::MESSAGE_INTERVAL::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_MESSAGE_INTERVAL(&mut data);
        println!("MESSAGE_INTERVAL")
    }

    fn on_ESTIMATOR_STATUS(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::ESTIMATOR_STATUS::Data_(curs.wrap(pack));
        read_ESTIMATOR_STATUS(&mut data);
        println!("ESTIMATOR_STATUS")
    }

    fn on_TIMESYNC(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::TIMESYNC::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_TIMESYNC(&mut data);
        println!("TIMESYNC")
    }

    fn on_GLOBAL_POSITION_INT_COV(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::GLOBAL_POSITION_INT_COV::Data_(curs.wrap(pack));
        read_GLOBAL_POSITION_INT_COV(&mut data);
        println!("GLOBAL_POSITION_INT_COV")
    }

    fn on_BUTTON_CHANGE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::BUTTON_CHANGE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_BUTTON_CHANGE(&mut data);
        println!("BUTTON_CHANGE")
    }

    fn on_SAFETY_SET_ALLOWED_AREA(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SAFETY_SET_ALLOWED_AREA::Data_(curs.wrap(pack));
        read_SAFETY_SET_ALLOWED_AREA(&mut data);
        println!("SAFETY_SET_ALLOWED_AREA")
    }

    fn on_STORAGE_INFORMATION(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::STORAGE_INFORMATION::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_STORAGE_INFORMATION(&mut data);
        println!("STORAGE_INFORMATION")
    }

    fn on_COLLISION(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::COLLISION::Data_(curs.wrap(pack));
        read_COLLISION(&mut data);
        println!("COLLISION")
    }

    fn on_ALTITUDE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::ALTITUDE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_ALTITUDE(&mut data);
        println!("ALTITUDE")
    }

    fn on_HIL_STATE_QUATERNION(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::HIL_STATE_QUATERNION::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_HIL_STATE_QUATERNION(&mut data);
        println!("HIL_STATE_QUATERNION")
    }

    fn on_CAMERA_INFORMATION(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::CAMERA_INFORMATION::Data_(curs.wrap(pack));
        read_CAMERA_INFORMATION(&mut data);
        println!("CAMERA_INFORMATION")
    }

    fn on_GPS_STATUS(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::GPS_STATUS::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_GPS_STATUS(&mut data);
        println!("GPS_STATUS")
    }

    fn on_PARAM_SET(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::PARAM_SET::Data_(curs.wrap(pack));
        read_PARAM_SET(&mut data);
        println!("PARAM_SET")
    }

    fn on_TERRAIN_DATA(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::TERRAIN_DATA::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_TERRAIN_DATA(&mut data);
        println!("TERRAIN_DATA")
    }

    fn on_RC_CHANNELS_OVERRIDE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::RC_CHANNELS_OVERRIDE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_RC_CHANNELS_OVERRIDE(&mut data);
        println!("RC_CHANNELS_OVERRIDE")
    }

    fn on_SCALED_IMU(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SCALED_IMU::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SCALED_IMU(&mut data);
        println!("SCALED_IMU")
    }

    fn on_DEBUG(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::DEBUG::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_DEBUG(&mut data);
        println!("DEBUG")
    }

    fn on_CAMERA_IMAGE_CAPTURED(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::CAMERA_IMAGE_CAPTURED::Data_(curs.wrap(pack));
        read_CAMERA_IMAGE_CAPTURED(&mut data);
        println!("CAMERA_IMAGE_CAPTURED")
    }

    fn on_LOG_ENTRY(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::LOG_ENTRY::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_LOG_ENTRY(&mut data);
        println!("LOG_ENTRY")
    }

    fn on_ACTUATOR_CONTROL_TARGET(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::ACTUATOR_CONTROL_TARGET::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_ACTUATOR_CONTROL_TARGET(&mut data);
        println!("ACTUATOR_CONTROL_TARGET")
    }

    fn on_HIGH_LATENCY(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::HIGH_LATENCY::Data_(curs.wrap(pack));
        read_HIGH_LATENCY(&mut data);
        println!("HIGH_LATENCY")
    }

    fn on_PARAM_REQUEST_READ(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::PARAM_REQUEST_READ::Data_(curs.wrap(pack));
        read_PARAM_REQUEST_READ(&mut data);
        println!("PARAM_REQUEST_READ")
    }

    fn on_SET_ATTITUDE_TARGET(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SET_ATTITUDE_TARGET::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SET_ATTITUDE_TARGET(&mut data);
        println!("SET_ATTITUDE_TARGET")
    }

    fn on_FOLLOW_TARGET(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::FOLLOW_TARGET::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_FOLLOW_TARGET(&mut data);
        println!("FOLLOW_TARGET")
    }

    fn on_HIL_STATE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::HIL_STATE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_HIL_STATE(&mut data);
        println!("HIL_STATE")
    }

    fn on_HOME_POSITION(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::HOME_POSITION::Data_(curs.wrap(pack));
        read_HOME_POSITION(&mut data);
        println!("HOME_POSITION")
    }

    fn on_GPS2_RAW(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::GPS2_RAW::Data_(curs.wrap(pack));
        read_GPS2_RAW(&mut data);
        println!("GPS2_RAW")
    }

    fn on_MEMORY_VECT(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::MEMORY_VECT::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_MEMORY_VECT(&mut data);
        println!("MEMORY_VECT")
    }

    fn on_REQUEST_DATA_STREAM(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::REQUEST_DATA_STREAM::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_REQUEST_DATA_STREAM(&mut data);
        println!("REQUEST_DATA_STREAM")
    }

    fn on_HIL_CONTROLS(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::HIL_CONTROLS::Data_(curs.wrap(pack));
        read_HIL_CONTROLS(&mut data);
        println!("HIL_CONTROLS")
    }

    fn on_HIL_SENSOR(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::HIL_SENSOR::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_HIL_SENSOR(&mut data);
        println!("HIL_SENSOR")
    }

    fn on_SETUP_SIGNING(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SETUP_SIGNING::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SETUP_SIGNING(&mut data);
        println!("SETUP_SIGNING")
    }

    fn on_GPS_RTK(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::GPS_RTK::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_GPS_RTK(&mut data);
        println!("GPS_RTK")
    }

    fn on_PARAM_REQUEST_LIST(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::PARAM_REQUEST_LIST::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_PARAM_REQUEST_LIST(&mut data);
        println!("PARAM_REQUEST_LIST")
    }

    fn on_LANDING_TARGET(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::LANDING_TARGET::Data_(curs.wrap(pack));
        read_LANDING_TARGET(&mut data);
        println!("LANDING_TARGET")
    }

    fn on_SET_ACTUATOR_CONTROL_TARGET(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SET_ACTUATOR_CONTROL_TARGET::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SET_ACTUATOR_CONTROL_TARGET(&mut data);
        println!("SET_ACTUATOR_CONTROL_TARGET")
    }

    fn on_CONTROL_SYSTEM_STATE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::CONTROL_SYSTEM_STATE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_CONTROL_SYSTEM_STATE(&mut data);
        println!("CONTROL_SYSTEM_STATE")
    }

    fn on_SET_POSITION_TARGET_GLOBAL_INT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SET_POSITION_TARGET_GLOBAL_INT::Data_(curs.wrap(pack));
        read_SET_POSITION_TARGET_GLOBAL_INT(&mut data);
        println!("SET_POSITION_TARGET_GLOBAL_INT")
    }

    fn on_VIBRATION(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::VIBRATION::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_VIBRATION(&mut data);
        println!("VIBRATION")
    }

    fn on_PING33(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::PING33::Data_(curs.wrap(pack));
        read_PING33(&mut data);
        println!("PING33")
    }

    fn on_VFR_HUD(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::VFR_HUD::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_VFR_HUD(&mut data);
        println!("VFR_HUD")
    }

    fn on_MISSION_SET_CURRENT(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::MISSION_SET_CURRENT::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_MISSION_SET_CURRENT(&mut data);
        println!("MISSION_SET_CURRENT")
    }

    fn on_HIL_GPS(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::HIL_GPS::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_HIL_GPS(&mut data);
        println!("HIL_GPS")
    }

    fn on_NAV_CONTROLLER_OUTPUT(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::NAV_CONTROLLER_OUTPUT::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_NAV_CONTROLLER_OUTPUT(&mut data);
        println!("NAV_CONTROLLER_OUTPUT")
    }

    fn on_AUTH_KEY(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::AUTH_KEY::Data_(curs.wrap(pack));
        read_AUTH_KEY(&mut data);
        println!("AUTH_KEY")
    }

    fn on_LOCAL_POSITION_NED_COV(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::LOCAL_POSITION_NED_COV::Data_(curs.wrap(pack));
        read_LOCAL_POSITION_NED_COV(&mut data);
        println!("LOCAL_POSITION_NED_COV")
    }

    fn on_ATT_POS_MOCAP(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::ATT_POS_MOCAP::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_ATT_POS_MOCAP(&mut data);
        println!("ATT_POS_MOCAP")
    }

    fn on_STATUSTEXT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::STATUSTEXT::Data_(curs.wrap(pack));
        read_STATUSTEXT(&mut data);
        println!("STATUSTEXT")
    }

    fn on_PING(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::PING::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_PING(&mut data);
        println!("PING")
    }

    fn on_CAMERA_CAPTURE_STATUS(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::CAMERA_CAPTURE_STATUS::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_CAMERA_CAPTURE_STATUS(&mut data);
        println!("CAMERA_CAPTURE_STATUS")
    }

    fn on_GLOBAL_POSITION_INT(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::GLOBAL_POSITION_INT::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_GLOBAL_POSITION_INT(&mut data);
        println!("GLOBAL_POSITION_INT")
    }

    fn on_ENCAPSULATED_DATA(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::ENCAPSULATED_DATA::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_ENCAPSULATED_DATA(&mut data);
        println!("ENCAPSULATED_DATA")
    }

    fn on_GPS_INPUT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::GPS_INPUT::Data_(curs.wrap(pack));
        read_GPS_INPUT(&mut data);
        println!("GPS_INPUT")
    }

    fn on_COMMAND_LONG(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::COMMAND_LONG::Data_(curs.wrap(pack));
        read_COMMAND_LONG(&mut data);
        println!("COMMAND_LONG")
    }

    fn on_LOG_REQUEST_DATA(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::LOG_REQUEST_DATA::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_LOG_REQUEST_DATA(&mut data);
        println!("LOG_REQUEST_DATA")
    }

    fn on_GPS_RAW_INT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::GPS_RAW_INT::Data_(curs.wrap(pack));
        read_GPS_RAW_INT(&mut data);
        println!("GPS_RAW_INT")
    }

    fn on_RC_CHANNELS_SCALED(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::RC_CHANNELS_SCALED::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_RC_CHANNELS_SCALED(&mut data);
        println!("RC_CHANNELS_SCALED")
    }

    fn on_CAMERA_SETTINGS(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::CAMERA_SETTINGS::Data_(curs.wrap(pack));
        read_CAMERA_SETTINGS(&mut data);
        println!("CAMERA_SETTINGS")
    }

    fn on_RAW_PRESSURE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::RAW_PRESSURE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_RAW_PRESSURE(&mut data);
        println!("RAW_PRESSURE")
    }

    fn on_NAMED_VALUE_FLOAT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::NAMED_VALUE_FLOAT::Data_(curs.wrap(pack));
        read_NAMED_VALUE_FLOAT(&mut data);
        println!("NAMED_VALUE_FLOAT")
    }

    fn on_ATTITUDE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::ATTITUDE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_ATTITUDE(&mut data);
        println!("ATTITUDE")
    }

    fn on_TERRAIN_REQUEST(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::TERRAIN_REQUEST::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_TERRAIN_REQUEST(&mut data);
        println!("TERRAIN_REQUEST")
    }

    fn on_MISSION_WRITE_PARTIAL_LIST(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_WRITE_PARTIAL_LIST::Data_(curs.wrap(pack));
        read_MISSION_WRITE_PARTIAL_LIST(&mut data);
        println!("MISSION_WRITE_PARTIAL_LIST")
    }

    fn on_LOG_ERASE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::LOG_ERASE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_LOG_ERASE(&mut data);
        println!("LOG_ERASE")
    }

    fn on_MANUAL_SETPOINT(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::MANUAL_SETPOINT::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_MANUAL_SETPOINT(&mut data);
        println!("MANUAL_SETPOINT")
    }

    fn on_SAFETY_ALLOWED_AREA(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SAFETY_ALLOWED_AREA::Data_(curs.wrap(pack));
        read_SAFETY_ALLOWED_AREA(&mut data);
        println!("SAFETY_ALLOWED_AREA")
    }

    fn on_OPTICAL_FLOW_RAD(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::OPTICAL_FLOW_RAD::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_OPTICAL_FLOW_RAD(&mut data);
        println!("OPTICAL_FLOW_RAD")
    }

    fn on_LOG_DATA(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::LOG_DATA::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_LOG_DATA(&mut data);
        println!("LOG_DATA")
    }

    fn on_MISSION_CLEAR_ALL(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_CLEAR_ALL::Data_(curs.wrap(pack));
        read_MISSION_CLEAR_ALL(&mut data);
        println!("MISSION_CLEAR_ALL")
    }

    fn on_VICON_POSITION_ESTIMATE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::VICON_POSITION_ESTIMATE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_VICON_POSITION_ESTIMATE(&mut data);
        println!("VICON_POSITION_ESTIMATE")
    }

    fn on_GPS2_RTK(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::GPS2_RTK::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_GPS2_RTK(&mut data);
        println!("GPS2_RTK")
    }

    fn on_LOG_REQUEST_LIST(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::LOG_REQUEST_LIST::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_LOG_REQUEST_LIST(&mut data);
        println!("LOG_REQUEST_LIST")
    }

    fn on_SCALED_PRESSURE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SCALED_PRESSURE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SCALED_PRESSURE(&mut data);
        println!("SCALED_PRESSURE")
    }

    fn on_MISSION_REQUEST_INT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_REQUEST_INT::Data_(curs.wrap(pack));
        read_MISSION_REQUEST_INT(&mut data);
        println!("MISSION_REQUEST_INT")
    }

    fn on_V2_EXTENSION(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::V2_EXTENSION::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_V2_EXTENSION(&mut data);
        println!("V2_EXTENSION")
    }

    fn on_HEARTBEAT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::HEARTBEAT::Data_(curs.wrap(pack));
        read_HEARTBEAT(&mut data);
        println!("HEARTBEAT")
    }

    fn on_PARAM_MAP_RC(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::PARAM_MAP_RC::Data_(curs.wrap(pack));
        read_PARAM_MAP_RC(&mut data);
        println!("PARAM_MAP_RC")
    }

    fn on_POWER_STATUS(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::POWER_STATUS::Data_(curs.wrap(pack));
        read_POWER_STATUS(&mut data);
        println!("POWER_STATUS")
    }

    fn on_TERRAIN_CHECK(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::TERRAIN_CHECK::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_TERRAIN_CHECK(&mut data);
        println!("TERRAIN_CHECK")
    }

    fn on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(&mut data);
        println!("LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET")
    }

    fn on_COMMAND_ACK(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::COMMAND_ACK::Data_(curs.wrap(pack));
        read_COMMAND_ACK(&mut data);
        println!("COMMAND_ACK")
    }

    fn on_DATA_STREAM(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::DATA_STREAM::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_DATA_STREAM(&mut data);
        println!("DATA_STREAM")
    }

    fn on_MISSION_REQUEST(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_REQUEST::Data_(curs.wrap(pack));
        read_MISSION_REQUEST(&mut data);
        println!("MISSION_REQUEST")
    }

    fn on_TERRAIN_REPORT(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::TERRAIN_REPORT::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_TERRAIN_REPORT(&mut data);
        println!("TERRAIN_REPORT")
    }

    fn on_SET_HOME_POSITION(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SET_HOME_POSITION::Data_(curs.wrap(pack));
        read_SET_HOME_POSITION(&mut data);
        println!("SET_HOME_POSITION")
    }
    fn on_SwitchModeCommand(&mut self) { println!("SwitchModeCommand") }

    fn on_HIL_RC_INPUTS_RAW(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::HIL_RC_INPUTS_RAW::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_HIL_RC_INPUTS_RAW(&mut data);
        println!("HIL_RC_INPUTS_RAW")
    }

    fn on_SCALED_IMU3(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SCALED_IMU3::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SCALED_IMU3(&mut data);
        println!("SCALED_IMU3")
    }

    fn on_SET_MODE(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SET_MODE::Data_(curs.wrap(pack));
        read_SET_MODE(&mut data);
        println!("SET_MODE")
    }

    fn on_POSITION_TARGET_GLOBAL_INT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::POSITION_TARGET_GLOBAL_INT::Data_(curs.wrap(pack));
        read_POSITION_TARGET_GLOBAL_INT(&mut data);
        println!("POSITION_TARGET_GLOBAL_INT")
    }

    fn on_FLIGHT_INFORMATION(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::FLIGHT_INFORMATION::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_FLIGHT_INFORMATION(&mut data);
        println!("FLIGHT_INFORMATION")
    }

    fn on_SIM_STATE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SIM_STATE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SIM_STATE(&mut data);
        println!("SIM_STATE")
    }

    fn on_MISSION_ITEM_REACHED(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::MISSION_ITEM_REACHED::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_MISSION_ITEM_REACHED(&mut data);
        println!("MISSION_ITEM_REACHED")
    }

    fn on_RC_CHANNELS_RAW(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::RC_CHANNELS_RAW::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_RC_CHANNELS_RAW(&mut data);
        println!("RC_CHANNELS_RAW")
    }

    fn on_SERVO_OUTPUT_RAW(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SERVO_OUTPUT_RAW::Data_(curs.wrap(pack));
        read_SERVO_OUTPUT_RAW(&mut data);
        println!("SERVO_OUTPUT_RAW")
    }

    fn on_VISION_SPEED_ESTIMATE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::VISION_SPEED_ESTIMATE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_VISION_SPEED_ESTIMATE(&mut data);
        println!("VISION_SPEED_ESTIMATE")
    }

    fn on_DEBUG_VECT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::DEBUG_VECT::Data_(curs.wrap(pack));
        read_DEBUG_VECT(&mut data);
        println!("DEBUG_VECT")
    }

    fn on_LOG_REQUEST_END(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::LOG_REQUEST_END::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_LOG_REQUEST_END(&mut data);
        println!("LOG_REQUEST_END")
    }

    fn on_MISSION_ACK(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_ACK::Data_(curs.wrap(pack));
        read_MISSION_ACK(&mut data);
        println!("MISSION_ACK")
    }

    fn on_CHANGE_OPERATOR_CONTROL_ACK(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::CHANGE_OPERATOR_CONTROL_ACK::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_CHANGE_OPERATOR_CONTROL_ACK(&mut data);
        println!("CHANGE_OPERATOR_CONTROL_ACK")
    }

    fn on_MISSION_CURRENT(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::MISSION_CURRENT::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_MISSION_CURRENT(&mut data);
        println!("MISSION_CURRENT")
    }

    fn on_SYSTEM_TIME(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SYSTEM_TIME::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SYSTEM_TIME(&mut data);
        println!("SYSTEM_TIME")
    }

    fn on_CAMERA_TRIGGER(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::CAMERA_TRIGGER::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_CAMERA_TRIGGER(&mut data);
        println!("CAMERA_TRIGGER")
    }

    fn on_VISION_POSITION_ESTIMATE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::VISION_POSITION_ESTIMATE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_VISION_POSITION_ESTIMATE(&mut data);
        println!("VISION_POSITION_ESTIMATE")
    }

    fn on_MANUAL_CONTROL(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::MANUAL_CONTROL::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_MANUAL_CONTROL(&mut data);
        println!("MANUAL_CONTROL")
    }

    fn on_RC_CHANNELS(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::RC_CHANNELS::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_RC_CHANNELS(&mut data);
        println!("RC_CHANNELS")
    }

    fn on_PARAM_VALUE(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::PARAM_VALUE::Data_(curs.wrap(pack));
        read_PARAM_VALUE(&mut data);
        println!("PARAM_VALUE")
    }

    fn on_BATTERY_STATUS(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::BATTERY_STATUS::Data_(curs.wrap(pack));
        read_BATTERY_STATUS(&mut data);
        println!("BATTERY_STATUS")
    }

    fn on_SET_POSITION_TARGET_LOCAL_NED(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SET_POSITION_TARGET_LOCAL_NED::Data_(curs.wrap(pack));
        read_SET_POSITION_TARGET_LOCAL_NED(&mut data);
        println!("SET_POSITION_TARGET_LOCAL_NED")
    }

    fn on_SERIAL_CONTROL(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SERIAL_CONTROL::Data_(curs.wrap(pack));
        read_SERIAL_CONTROL(&mut data);
        println!("SERIAL_CONTROL")
    }

    fn on_SET_GPS_GLOBAL_ORIGIN(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SET_GPS_GLOBAL_ORIGIN::Data_(curs.wrap(pack));
        read_SET_GPS_GLOBAL_ORIGIN(&mut data);
        println!("SET_GPS_GLOBAL_ORIGIN")
    }

    fn on_AUTOPILOT_VERSION(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::AUTOPILOT_VERSION::Data_(curs.wrap(pack));
        read_AUTOPILOT_VERSION(&mut data);
        println!("AUTOPILOT_VERSION")
    }

    fn on_MISSION_REQUEST_LIST(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_REQUEST_LIST::Data_(curs.wrap(pack));
        read_MISSION_REQUEST_LIST(&mut data);
        println!("MISSION_REQUEST_LIST")
    }

    fn on_PLAY_TUNE(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::PLAY_TUNE::Data_(curs.wrap(pack));
        read_PLAY_TUNE(&mut data);
        println!("PLAY_TUNE")
    }

    fn on_SCALED_PRESSURE3(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SCALED_PRESSURE3::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SCALED_PRESSURE3(&mut data);
        println!("SCALED_PRESSURE3")
    }

    fn on_MISSION_REQUEST_PARTIAL_LIST(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_REQUEST_PARTIAL_LIST::Data_(curs.wrap(pack));
        read_MISSION_REQUEST_PARTIAL_LIST(&mut data);
        println!("MISSION_REQUEST_PARTIAL_LIST")
    }

    fn on_LOCAL_POSITION_NED(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::LOCAL_POSITION_NED::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_LOCAL_POSITION_NED(&mut data);
        println!("LOCAL_POSITION_NED")
    }

    fn on_DATA_TRANSMISSION_HANDSHAKE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::DATA_TRANSMISSION_HANDSHAKE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_DATA_TRANSMISSION_HANDSHAKE(&mut data);
        println!("DATA_TRANSMISSION_HANDSHAKE")
    }

    fn on_GPS_GLOBAL_ORIGIN(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::GPS_GLOBAL_ORIGIN::Data_(curs.wrap(pack));
        read_GPS_GLOBAL_ORIGIN(&mut data);
        println!("GPS_GLOBAL_ORIGIN")
    }

    fn on_SCALED_IMU2(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SCALED_IMU2::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SCALED_IMU2(&mut data);
        println!("SCALED_IMU2")
    }

    fn on_ATTITUDE_QUATERNION(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::ATTITUDE_QUATERNION::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_ATTITUDE_QUATERNION(&mut data);
        println!("ATTITUDE_QUATERNION")
    }

    fn on_HIL_ACTUATOR_CONTROLS(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::HIL_ACTUATOR_CONTROLS::Data_(curs.wrap(pack));
        read_HIL_ACTUATOR_CONTROLS(&mut data);
        println!("HIL_ACTUATOR_CONTROLS")
    }

    fn on_POSITION_TARGET_LOCAL_NED(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::POSITION_TARGET_LOCAL_NED::Data_(curs.wrap(pack));
        read_POSITION_TARGET_LOCAL_NED(&mut data);
        println!("POSITION_TARGET_LOCAL_NED")
    }

    fn on_DISTANCE_SENSOR(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::DISTANCE_SENSOR::Data_(curs.wrap(pack));
        read_DISTANCE_SENSOR(&mut data);
        println!("DISTANCE_SENSOR")
    }

    fn on_HIL_OPTICAL_FLOW(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::HIL_OPTICAL_FLOW::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_HIL_OPTICAL_FLOW(&mut data);
        println!("HIL_OPTICAL_FLOW")
    }

    fn on_SCALED_PRESSURE2(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::SCALED_PRESSURE2::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_SCALED_PRESSURE2(&mut data);
        println!("SCALED_PRESSURE2")
    }

    fn on_WIND_COV(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::WIND_COV::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_WIND_COV(&mut data);
        println!("WIND_COV")
    }

    fn on_CHANGE_OPERATOR_CONTROL(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::CHANGE_OPERATOR_CONTROL::Data_(curs.wrap(pack));
        read_CHANGE_OPERATOR_CONTROL(&mut data);
        println!("CHANGE_OPERATOR_CONTROL")
    }

    fn on_SYS_STATUS(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::SYS_STATUS::Data_(curs.wrap(pack));
        read_SYS_STATUS(&mut data);
        println!("SYS_STATUS")
    }

    fn on_MISSION_ITEM(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_ITEM::Data_(curs.wrap(pack));
        read_MISSION_ITEM(&mut data);
        println!("MISSION_ITEM")
    }

    fn on_RAW_IMU(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::RAW_IMU::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_RAW_IMU(&mut data);
        println!("RAW_IMU")
    }

    fn on_COMMAND_INT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::COMMAND_INT::Data_(curs.wrap(pack));
        read_COMMAND_INT(&mut data);
        println!("COMMAND_INT")
    }

    fn on_OPTICAL_FLOW(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::OPTICAL_FLOW::Data_(curs.wrap(pack));
        read_OPTICAL_FLOW(&mut data);
        println!("OPTICAL_FLOW")
    }

    fn on_MISSION_ITEM_INT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::MISSION_ITEM_INT::Data_(curs.wrap(pack));
        read_MISSION_ITEM_INT(&mut data);
        println!("MISSION_ITEM_INT")
    }

    fn on_HIGHRES_IMU(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::HIGHRES_IMU::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_HIGHRES_IMU(&mut data);
        println!("HIGHRES_IMU")
    }

    fn on_EXTENDED_SYS_STATE(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::EXTENDED_SYS_STATE::Data_(curs.wrap(pack));
        read_EXTENDED_SYS_STATE(&mut data);
        println!("EXTENDED_SYS_STATE")
    }

    fn on_GPS_INJECT_DATA(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::GPS_INJECT_DATA::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_GPS_INJECT_DATA(&mut data);
        println!("GPS_INJECT_DATA")
    }

    fn on_ATTITUDE_QUATERNION_COV(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::ATTITUDE_QUATERNION_COV::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_ATTITUDE_QUATERNION_COV(&mut data);
        println!("ATTITUDE_QUATERNION_COV")
    }

    fn on_NAMED_VALUE_INT(&mut self, pack: *mut sys::Pack) {
        let mut curs = sys::Cursors::new();
        let mut data = packs::NAMED_VALUE_INT::Data_(curs.wrap(pack));
        read_NAMED_VALUE_INT(&mut data);
        println!("NAMED_VALUE_INT")
    }

    fn on_RADIO_STATUS(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::RADIO_STATUS::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_RADIO_STATUS(&mut data);
        println!("RADIO_STATUS")
    }

    fn on_GPS_RTCM_DATA(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::GPS_RTCM_DATA::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_GPS_RTCM_DATA(&mut data);
        println!("GPS_RTCM_DATA")
    }

    fn on_GLOBAL_VISION_POSITION_ESTIMATE(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::GLOBAL_VISION_POSITION_ESTIMATE::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_GLOBAL_VISION_POSITION_ESTIMATE(&mut data);
        println!("GLOBAL_VISION_POSITION_ESTIMATE")
    }

    fn on_FILE_TRANSFER_PROTOCOL(&mut self, pack: *mut sys::Pack) {
        let mut data = packs::FILE_TRANSFER_PROTOCOL::Data_(unsafe { (*pack).bytes.as_mut_ptr() });
        read_FILE_TRANSFER_PROTOCOL(&mut data);
        println!("FILE_TRANSFER_PROTOCOL")
    }
}

impl ::std::io::Write for CommunicationChannel_receiver {
    #[inline(always)]
    fn write(&mut self, src: &[u8]) -> Result<usize, ::std::io::Error> { self.bytes_into_packs(src) }
    #[inline(always)]
    fn flush(&mut self) -> Result<(), ::std::io::Error> { Ok(()) }
}

use host::CommunicationChannel::ITransmitter as CommunicationChannel_ITransmitter;

pub struct CommunicationChannel_transmitter {
    pub transmitter: sys::Transmitter,
    pub sending_queue: (Sender<*mut sys::Pack>, Receiver<*mut sys::Pack>),
}

impl CommunicationChannel_transmitter {
    pub fn new() -> CommunicationChannel_transmitter { CommunicationChannel_transmitter { transmitter: <CommunicationChannel_transmitter as CommunicationChannel::ITransmitter>::new(), sending_queue: channel::<*mut sys::Pack>() } }
}

impl host::CommunicationChannel::ITransmitter for CommunicationChannel_transmitter {
    unsafe extern "C" fn dequeue(dst: *mut sys::Transmitter) -> *const sys::Pack {
        let self_: &mut Self = ad_hoc_gen::sys::self_by_field_ptr!(dst, transmitter);

        if let Ok(mut pack) = self_.sending_queue.1.try_recv() {
            return pack;
        }
        return std::ptr::null_mut();
    }
    fn queue(&mut self, pack: *mut sys::Pack) -> Result<(), *mut sys::Pack> {
        if self.sending_queue.0.send(pack).is_err() {
            return Err(pack);
        }
        Ok(())
    }
    fn get_transmitter(&mut self) -> &mut sys::Transmitter { &mut self.transmitter }
}

impl ::std::io::Read for CommunicationChannel_transmitter {
    #[inline(always)]
    fn read(&mut self, dst: &mut [u8]) -> Result<usize, ::std::io::Error> { self.packs_into_bytes(dst) }
}

fn read_ATTITUDE_TARGET(src: &mut packs::ATTITUDE_TARGET::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let type_mask = src.type_mask();
    let q = src.q();
    let body_roll_rate = src.body_roll_rate();
    let body_pitch_rate = src.body_pitch_rate();
    let body_yaw_rate = src.body_yaw_rate();
    let thrust = src.thrust();
}

fn read_MISSION_COUNT(src: &mut packs::MISSION_COUNT::Data_) {
    let count = src.count();
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(mission_type) = src.mission_type() {}
}

fn read_ADSB_VEHICLE(src: &mut packs::ADSB_VEHICLE::Data_) {
    let heading = src.heading();
    let hor_velocity = src.hor_velocity();
    let squawk = src.squawk();
    let ICAO_address = src.ICAO_address();
    let lat = src.lat();
    let lon = src.lon();
    let altitude = src.altitude();
    let ver_velocity = src.ver_velocity();
    let tslc = src.tslc();
    if let Some(altitude_type) = src.altitude_type().get() {}
    if let Some(src) = src.callsign().get() {}
    if let Some(emitter_type) = src.emitter_type().get() {}
    if let Some(flags) = src.flags().get() {}
}

fn write_ADSB_VEHICLE(dst: &mut packs::ADSB_VEHICLE::Data_) {
    dst.heading().set(some_i16);
    dst.hor_velocity().set(some_i16);
    dst.squawk().set(some_i16);
    dst.ICAO_address().set(some_i32);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.altitude().set(some_i32);
    dst.ver_velocity().set(some_i16);
    dst.tslc().set(some_i8);
    dst.altitude_type().set(packs::ADSB_ALTITUDE_TYPE);
    dst.callsign().set(some_str);
    dst.emitter_type().set(packs::ADSB_EMITTER_TYPE);
    dst.flags().set(packs::ADSB_FLAGS);
}

fn read_MESSAGE_INTERVAL(src: &mut packs::MESSAGE_INTERVAL::Data_) {
    let message_id = src.message_id();
    let interval_us = src.interval_us();
}

fn write_MESSAGE_INTERVAL(dst: &mut packs::MESSAGE_INTERVAL::Data_) {
    dst.message_id().set(some_i16);
    dst.interval_us().set(some_i32);
}

fn read_EKF_STATUS_REPORT(src: &mut packs::EKF_STATUS_REPORT::Data_) {
    let velocity_variance = src.velocity_variance();
    let pos_horiz_variance = src.pos_horiz_variance();
    let pos_vert_variance = src.pos_vert_variance();
    let compass_variance = src.compass_variance();
    let terrain_alt_variance = src.terrain_alt_variance();
    if let Some(flags) = src.flags().get() {}
}

fn write_EKF_STATUS_REPORT(dst: &mut packs::EKF_STATUS_REPORT::Data_) {
    dst.velocity_variance().set(some_f32);
    dst.pos_horiz_variance().set(some_f32);
    dst.pos_vert_variance().set(some_f32);
    dst.compass_variance().set(some_f32);
    dst.terrain_alt_variance().set(some_f32);
    dst.flags().set(packs::EKF_STATUS_FLAGS);
}

fn read_ESTIMATOR_STATUS(src: &mut packs::ESTIMATOR_STATUS::Data_) {
    let time_usec = src.time_usec();
    let vel_ratio = src.vel_ratio();
    let pos_horiz_ratio = src.pos_horiz_ratio();
    let pos_vert_ratio = src.pos_vert_ratio();
    let mag_ratio = src.mag_ratio();
    let hagl_ratio = src.hagl_ratio();
    let tas_ratio = src.tas_ratio();
    let pos_horiz_accuracy = src.pos_horiz_accuracy();
    let pos_vert_accuracy = src.pos_vert_accuracy();
    if let Some(flags) = src.flags().get() {}
}

fn write_ESTIMATOR_STATUS(dst: &mut packs::ESTIMATOR_STATUS::Data_) {
    dst.time_usec().set(some_i64);
    dst.vel_ratio().set(some_f32);
    dst.pos_horiz_ratio().set(some_f32);
    dst.pos_vert_ratio().set(some_f32);
    dst.mag_ratio().set(some_f32);
    dst.hagl_ratio().set(some_f32);
    dst.tas_ratio().set(some_f32);
    dst.pos_horiz_accuracy().set(some_f32);
    dst.pos_vert_accuracy().set(some_f32);
    dst.flags().set(packs::ESTIMATOR_STATUS_FLAGS);
}

fn read_HWSTATUS(src: &mut packs::HWSTATUS::Data_) {
    let Vcc = src.Vcc();
    let I2Cerr = src.I2Cerr();
}

fn write_HWSTATUS(dst: &mut packs::HWSTATUS::Data_) {
    dst.Vcc().set(some_i16);
    dst.I2Cerr().set(some_i8);
}

fn read_TIMESYNC(src: &mut packs::TIMESYNC::Data_) {
    let tc1 = src.tc1();
    let ts1 = src.ts1();
}

fn write_TIMESYNC(dst: &mut packs::TIMESYNC::Data_) {
    dst.tc1().set(some_i64);
    dst.ts1().set(some_i64);
}

fn read_PARAM_EXT_REQUEST_LIST(src: &mut packs::PARAM_EXT_REQUEST_LIST::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn write_PARAM_EXT_REQUEST_LIST(dst: &mut packs::PARAM_EXT_REQUEST_LIST::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
}

fn read_GLOBAL_POSITION_INT_COV(src: &mut packs::GLOBAL_POSITION_INT_COV::Data_) {
    let time_usec = src.time_usec();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let relative_alt = src.relative_alt();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let covariance = src.covariance();
    if let Some(estimator_type) = src.estimator_type() {}
}

fn read_BUTTON_CHANGE(src: &mut packs::BUTTON_CHANGE::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let last_change_ms = src.last_change_ms();
    let state = src.state();
}

fn write_BUTTON_CHANGE(dst: &mut packs::BUTTON_CHANGE::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.last_change_ms().set(some_i32);
    dst.state().set(some_i8);
}

fn read_SAFETY_SET_ALLOWED_AREA(src: &mut packs::SAFETY_SET_ALLOWED_AREA::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let p1x = src.p1x();
    let p1y = src.p1y();
    let p1z = src.p1z();
    let p2x = src.p2x();
    let p2y = src.p2y();
    let p2z = src.p2z();
    if let Some(frame) = src.frame() {}
}

fn read_UAVCAN_NODE_STATUS(src: &mut packs::UAVCAN_NODE_STATUS::Data_) {
    let vendor_specific_status_code = src.vendor_specific_status_code();
    let uptime_sec = src.uptime_sec();
    let time_usec = src.time_usec();
    let sub_mode = src.sub_mode();
    if let Some(health) = src.health().get() {}
    if let Some(mode) = src.mode().get() {}
}

fn write_UAVCAN_NODE_STATUS(dst: &mut packs::UAVCAN_NODE_STATUS::Data_) {
    dst.vendor_specific_status_code().set(some_i16);
    dst.uptime_sec().set(some_i32);
    dst.time_usec().set(some_i64);
    dst.sub_mode().set(some_i8);
    dst.health().set(packs::UAVCAN_NODE_HEALTH);
    dst.mode().set(packs::UAVCAN_NODE_MODE);
}

fn read_COLLISION(src: &mut packs::COLLISION::Data_) {
    let id = src.id();
    let time_to_minimum_delta = src.time_to_minimum_delta();
    let altitude_minimum_delta = src.altitude_minimum_delta();
    let horizontal_minimum_delta = src.horizontal_minimum_delta();
    if let Some(sRc) = src.sRc().get() {}
    if let Some(action) = src.action().get() {}
    if let Some(threat_level) = src.threat_level().get() {}
}

fn write_COLLISION(dst: &mut packs::COLLISION::Data_) {
    dst.id().set(some_i32);
    dst.time_to_minimum_delta().set(some_f32);
    dst.altitude_minimum_delta().set(some_f32);
    dst.horizontal_minimum_delta().set(some_f32);
    dst.sRc().set(packs::MAV_COLLISION_SRC);
    dst.action().set(packs::MAV_COLLISION_ACTION);
    dst.threat_level().set(packs::MAV_COLLISION_THREAT_LEVEL);
}

fn read_GIMBAL_TORQUE_CMD_REPORT(src: &mut packs::GIMBAL_TORQUE_CMD_REPORT::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let rl_torque_cmd = src.rl_torque_cmd();
    let el_torque_cmd = src.el_torque_cmd();
    let az_torque_cmd = src.az_torque_cmd();
}

fn write_GIMBAL_TORQUE_CMD_REPORT(dst: &mut packs::GIMBAL_TORQUE_CMD_REPORT::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.rl_torque_cmd().set(some_i16);
    dst.el_torque_cmd().set(some_i16);
    dst.az_torque_cmd().set(some_i16);
}

fn read_ALTITUDE(src: &mut packs::ALTITUDE::Data_) {
    let time_usec = src.time_usec();
    let altitude_monotonic = src.altitude_monotonic();
    let altitude_amsl = src.altitude_amsl();
    let altitude_local = src.altitude_local();
    let altitude_relative = src.altitude_relative();
    let altitude_terrain = src.altitude_terrain();
    let bottom_clearance = src.bottom_clearance();
}

fn write_ALTITUDE(dst: &mut packs::ALTITUDE::Data_) {
    dst.time_usec().set(some_i64);
    dst.altitude_monotonic().set(some_f32);
    dst.altitude_amsl().set(some_f32);
    dst.altitude_local().set(some_f32);
    dst.altitude_relative().set(some_f32);
    dst.altitude_terrain().set(some_f32);
    dst.bottom_clearance().set(some_f32);
}

fn read_HIL_STATE_QUATERNION(src: &mut packs::HIL_STATE_QUATERNION::Data_) {
    let ind_airspeed = src.ind_airspeed();
    let true_airspeed = src.true_airspeed();
    let time_usec = src.time_usec();
    let attitude_quaternion = src.attitude_quaternion(None);
    let rollspeed = src.rollspeed();
    let pitchspeed = src.pitchspeed();
    let yawspeed = src.yawspeed();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
}

fn write_HIL_STATE_QUATERNION(dst: &mut packs::HIL_STATE_QUATERNION::Data_) {
    dst.ind_airspeed().set(some_i16);
    dst.true_airspeed().set(some_i16);
    dst.time_usec().set(some_i64);
    dst.attitude_quaternion(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.rollspeed().set(some_f32);
    dst.pitchspeed().set(some_f32);
    dst.yawspeed().set(some_f32);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.alt().set(some_i32);
    dst.vx().set(some_i16);
    dst.vy().set(some_i16);
    dst.vz().set(some_i16);
    dst.xacc().set(some_i16);
    dst.yacc().set(some_i16);
    dst.zacc().set(some_i16);
}

fn read_SENSOR_OFFSETS(src: &mut packs::SENSOR_OFFSETS::Data_) {
    let mag_ofs_x = src.mag_ofs_x();
    let mag_ofs_y = src.mag_ofs_y();
    let mag_ofs_z = src.mag_ofs_z();
    let mag_declination = src.mag_declination();
    let raw_press = src.raw_press();
    let raw_temp = src.raw_temp();
    let gyro_cal_x = src.gyro_cal_x();
    let gyro_cal_y = src.gyro_cal_y();
    let gyro_cal_z = src.gyro_cal_z();
    let accel_cal_x = src.accel_cal_x();
    let accel_cal_y = src.accel_cal_y();
    let accel_cal_z = src.accel_cal_z();
}

fn write_SENSOR_OFFSETS(dst: &mut packs::SENSOR_OFFSETS::Data_) {
    dst.mag_ofs_x().set(some_i16);
    dst.mag_ofs_y().set(some_i16);
    dst.mag_ofs_z().set(some_i16);
    dst.mag_declination().set(some_f32);
    dst.raw_press().set(some_i32);
    dst.raw_temp().set(some_i32);
    dst.gyro_cal_x().set(some_f32);
    dst.gyro_cal_y().set(some_f32);
    dst.gyro_cal_z().set(some_f32);
    dst.accel_cal_x().set(some_f32);
    dst.accel_cal_y().set(some_f32);
    dst.accel_cal_z().set(some_f32);
}

fn read_STORAGE_INFORMATION(src: &mut packs::STORAGE_INFORMATION::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let storage_id = src.storage_id();
    let storage_count = src.storage_count();
    let status = src.status();
    let total_capacity = src.total_capacity();
    let used_capacity = src.used_capacity();
    let available_capacity = src.available_capacity();
    let read_speed = src.read_speed();
    let write_speed = src.write_speed();
}

fn write_STORAGE_INFORMATION(dst: &mut packs::STORAGE_INFORMATION::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.storage_id().set(some_i8);
    dst.storage_count().set(some_i8);
    dst.status().set(some_i8);
    dst.total_capacity().set(some_f32);
    dst.used_capacity().set(some_f32);
    dst.available_capacity().set(some_f32);
    dst.read_speed().set(some_f32);
    dst.write_speed().set(some_f32);
}

fn read_CAMERA_INFORMATION(src: &mut packs::CAMERA_INFORMATION::Data_) {
    let resolution_h = src.resolution_h();
    let resolution_v = src.resolution_v();
    let cam_definition_version = src.cam_definition_version();
    let time_boot_ms = src.time_boot_ms();
    let firmware_version = src.firmware_version();
    let vendor_name = src.vendor_name(None);
    let model_name = src.model_name(None);
    let focal_length = src.focal_length();
    let sensor_size_h = src.sensor_size_h();
    let sensor_size_v = src.sensor_size_v();
    let lens_id = src.lens_id();
    if let Some(flags) = src.flags().get() {}
    if let Some(src) = src.cam_definition_uri().get() {}
}

fn write_CAMERA_INFORMATION(dst: &mut packs::CAMERA_INFORMATION::Data_) {
    dst.resolution_h().set(some_i16);
    dst.resolution_v().set(some_i16);
    dst.cam_definition_version().set(some_i16);
    dst.time_boot_ms().set(some_i32);
    dst.firmware_version().set(some_i32);
    dst.vendor_name(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.model_name(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.focal_length().set(some_f32);
    dst.sensor_size_h().set(some_f32);
    dst.sensor_size_v().set(some_f32);
    dst.lens_id().set(some_i8);
    dst.flags().set(packs::CAMERA_CAP_FLAGS);
    dst.cam_definition_uri().set(some_str);
}

fn read_GPS_STATUS(src: &mut packs::GPS_STATUS::Data_) {
    let satellites_visible = src.satellites_visible();
    let satellite_prn = src.satellite_prn();
    let satellite_used = src.satellite_used();
    let satellite_elevation = src.satellite_elevation();
    let satellite_azimuth = src.satellite_azimuth();
    let satellite_snr = src.satellite_snr();
}

fn read_DEVICE_OP_WRITE_REPLY(src: &mut packs::DEVICE_OP_WRITE_REPLY::Data_) {
    let request_id = src.request_id();
    let result = src.result();
}

fn write_DEVICE_OP_WRITE_REPLY(dst: &mut packs::DEVICE_OP_WRITE_REPLY::Data_) {
    dst.request_id().set(some_i32);
    dst.result().set(some_i8);
}

fn read_PARAM_SET(src: &mut packs::PARAM_SET::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let param_value = src.param_value();
    if let Some(src) = src.param_id() {}
    if let Some(param_type) = src.param_type() {}
}

fn read_TERRAIN_DATA(src: &mut packs::TERRAIN_DATA::Data_) {
    let grid_spacing = src.grid_spacing();
    let lat = src.lat();
    let lon = src.lon();
    let gridbit = src.gridbit();
    let daTa = src.daTa(None);
}

fn write_TERRAIN_DATA(dst: &mut packs::TERRAIN_DATA::Data_) {
    dst.grid_spacing().set(some_i16);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.gridbit().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i16s)));
}

fn read_GIMBAL_CONTROL(src: &mut packs::GIMBAL_CONTROL::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let demanded_rate_x = src.demanded_rate_x();
    let demanded_rate_y = src.demanded_rate_y();
    let demanded_rate_z = src.demanded_rate_z();
}

fn write_GIMBAL_CONTROL(dst: &mut packs::GIMBAL_CONTROL::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.demanded_rate_x().set(some_f32);
    dst.demanded_rate_y().set(some_f32);
    dst.demanded_rate_z().set(some_f32);
}

fn read_RC_CHANNELS_OVERRIDE(src: &mut packs::RC_CHANNELS_OVERRIDE::Data_) {
    let chan1_raw = src.chan1_raw();
    let chan2_raw = src.chan2_raw();
    let chan3_raw = src.chan3_raw();
    let chan4_raw = src.chan4_raw();
    let chan5_raw = src.chan5_raw();
    let chan6_raw = src.chan6_raw();
    let chan7_raw = src.chan7_raw();
    let chan8_raw = src.chan8_raw();
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn read_SCALED_IMU(src: &mut packs::SCALED_IMU::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
    let xgyro = src.xgyro();
    let ygyro = src.ygyro();
    let zgyro = src.zgyro();
    let xmag = src.xmag();
    let ymag = src.ymag();
    let zmag = src.zmag();
}

fn read_VIDEO_STREAM_INFORMATION(src: &mut packs::VIDEO_STREAM_INFORMATION::Data_) {
    let resolution_h = src.resolution_h();
    let resolution_v = src.resolution_v();
    let rotation = src.rotation();
    let bitrate = src.bitrate();
    let camera_id = src.camera_id();
    let status = src.status();
    let framerate = src.framerate();
    if let Some(src) = src.uri().get() {}
}

fn write_VIDEO_STREAM_INFORMATION(dst: &mut packs::VIDEO_STREAM_INFORMATION::Data_) {
    dst.resolution_h().set(some_i16);
    dst.resolution_v().set(some_i16);
    dst.rotation().set(some_i16);
    dst.bitrate().set(some_i32);
    dst.camera_id().set(some_i8);
    dst.status().set(some_i8);
    dst.framerate().set(some_f32);
    dst.uri().set(some_str);
}

fn read_AHRS(src: &mut packs::AHRS::Data_) {
    let omegaIx = src.omegaIx();
    let omegaIy = src.omegaIy();
    let omegaIz = src.omegaIz();
    let accel_weight = src.accel_weight();
    let renorm_val = src.renorm_val();
    let error_rp = src.error_rp();
    let error_yaw = src.error_yaw();
}

fn write_AHRS(dst: &mut packs::AHRS::Data_) {
    dst.omegaIx().set(some_f32);
    dst.omegaIy().set(some_f32);
    dst.omegaIz().set(some_f32);
    dst.accel_weight().set(some_f32);
    dst.renorm_val().set(some_f32);
    dst.error_rp().set(some_f32);
    dst.error_yaw().set(some_f32);
}

fn read_DEBUG(src: &mut packs::DEBUG::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let ind = src.ind();
    let value = src.value();
}

fn write_DEBUG(dst: &mut packs::DEBUG::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.ind().set(some_i8);
    dst.value().set(some_f32);
}

fn read_CAMERA_IMAGE_CAPTURED(src: &mut packs::CAMERA_IMAGE_CAPTURED::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let time_utc = src.time_utc();
    let camera_id = src.camera_id();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let relative_alt = src.relative_alt();
    let q = src.q(None);
    let image_index = src.image_index();
    let capture_result = src.capture_result();
    if let Some(src) = src.file_url().get() {}
}

fn write_CAMERA_IMAGE_CAPTURED(dst: &mut packs::CAMERA_IMAGE_CAPTURED::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.time_utc().set(some_i64);
    dst.camera_id().set(some_i8);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.alt().set(some_i32);
    dst.relative_alt().set(some_i32);
    dst.q(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.image_index().set(some_i32);
    dst.capture_result().set(some_i8);
    dst.file_url().set(some_str);
}

fn read_LOG_ENTRY(src: &mut packs::LOG_ENTRY::Data_) {
    let id = src.id();
    let num_logs = src.num_logs();
    let last_log_num = src.last_log_num();
    let time_utc = src.time_utc();
    let size = src.size();
}

fn write_LOG_ENTRY(dst: &mut packs::LOG_ENTRY::Data_) {
    dst.id().set(some_i16);
    dst.num_logs().set(some_i16);
    dst.last_log_num().set(some_i16);
    dst.time_utc().set(some_i32);
    dst.size().set(some_i32);
}

fn read_ACTUATOR_CONTROL_TARGET(src: &mut packs::ACTUATOR_CONTROL_TARGET::Data_) {
    let time_usec = src.time_usec();
    let group_mlx = src.group_mlx();
    let controls = src.controls(None);
}

fn write_ACTUATOR_CONTROL_TARGET(dst: &mut packs::ACTUATOR_CONTROL_TARGET::Data_) {
    dst.time_usec().set(some_i64);
    dst.group_mlx().set(some_i8);
    dst.controls(Some(&mut sys::ByValIter::new(&some_f32s)));
}

fn read_HIGH_LATENCY(src: &mut packs::HIGH_LATENCY::Data_) {
    let heading = src.heading();
    let wp_distance = src.wp_distance();
    let custom_mode = src.custom_mode();
    let roll = src.roll();
    let pitch = src.pitch();
    let throttle = src.throttle();
    let heading_sp = src.heading_sp();
    let latitude = src.latitude();
    let longitude = src.longitude();
    let altitude_amsl = src.altitude_amsl();
    let altitude_sp = src.altitude_sp();
    let airspeed = src.airspeed();
    let airspeed_sp = src.airspeed_sp();
    let groundspeed = src.groundspeed();
    let climb_rate = src.climb_rate();
    let gps_nsat = src.gps_nsat();
    let battery_remaining = src.battery_remaining();
    let temperature = src.temperature();
    let temperature_air = src.temperature_air();
    let failsafe = src.failsafe();
    let wp_num = src.wp_num();
    if let Some(base_mode) = src.base_mode().get() {}
    if let Some(landed_state) = src.landed_state().get() {}
    if let Some(gps_fix_type) = src.gps_fix_type().get() {}
}

fn write_HIGH_LATENCY(dst: &mut packs::HIGH_LATENCY::Data_) {
    dst.heading().set(some_i16);
    dst.wp_distance().set(some_i16);
    dst.custom_mode().set(some_i32);
    dst.roll().set(some_i16);
    dst.pitch().set(some_i16);
    dst.throttle().set(some_i8);
    dst.heading_sp().set(some_i16);
    dst.latitude().set(some_i32);
    dst.longitude().set(some_i32);
    dst.altitude_amsl().set(some_i16);
    dst.altitude_sp().set(some_i16);
    dst.airspeed().set(some_i8);
    dst.airspeed_sp().set(some_i8);
    dst.groundspeed().set(some_i8);
    dst.climb_rate().set(some_i8);
    dst.gps_nsat().set(some_i8);
    dst.battery_remaining().set(some_i8);
    dst.temperature().set(some_i8);
    dst.temperature_air().set(some_i8);
    dst.failsafe().set(some_i8);
    dst.wp_num().set(some_i8);
    dst.base_mode().set(packs::MAV_MODE_FLAG);
    dst.landed_state().set(packs::MAV_LANDED_STATE);
    dst.gps_fix_type().set(packs::GPS_FIX_TYPE);
}

fn read_PARAM_REQUEST_READ(src: &mut packs::PARAM_REQUEST_READ::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let param_index = src.param_index();
    if let Some(src) = src.param_id() {}
}

fn read_SET_ATTITUDE_TARGET(src: &mut packs::SET_ATTITUDE_TARGET::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let type_mask = src.type_mask();
    let q = src.q();
    let body_roll_rate = src.body_roll_rate();
    let body_pitch_rate = src.body_pitch_rate();
    let body_yaw_rate = src.body_yaw_rate();
    let thrust = src.thrust();
}

fn read_FOLLOW_TARGET(src: &mut packs::FOLLOW_TARGET::Data_) {
    let timestamp = src.timestamp();
    let custom_state = src.custom_state();
    let est_capabilities = src.est_capabilities();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let vel = src.vel(None);
    let acc = src.acc(None);
    let attitude_q = src.attitude_q(None);
    let rates = src.rates(None);
    let position_cov = src.position_cov(None);
}

fn write_FOLLOW_TARGET(dst: &mut packs::FOLLOW_TARGET::Data_) {
    dst.timestamp().set(some_i64);
    dst.custom_state().set(some_i64);
    dst.est_capabilities().set(some_i8);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.alt().set(some_f32);
    dst.vel(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.acc(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.attitude_q(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.rates(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.position_cov(Some(&mut sys::ByValIter::new(&some_f32s)));
}

fn read_HIL_STATE(src: &mut packs::HIL_STATE::Data_) {
    let time_usec = src.time_usec();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
    let rollspeed = src.rollspeed();
    let pitchspeed = src.pitchspeed();
    let yawspeed = src.yawspeed();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
}

fn read_HOME_POSITION(src: &mut packs::HOME_POSITION::Data_) {
    let latitude = src.latitude();
    let longitude = src.longitude();
    let altitude = src.altitude();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let q = src.q(None);
    let approach_x = src.approach_x();
    let approach_y = src.approach_y();
    let approach_z = src.approach_z();
    if let Some(time_usec) = src.time_usec().get() {}
}

fn write_HOME_POSITION(dst: &mut packs::HOME_POSITION::Data_) {
    dst.latitude().set(some_i32);
    dst.longitude().set(some_i32);
    dst.altitude().set(some_i32);
    dst.x().set(some_f32);
    dst.y().set(some_f32);
    dst.z().set(some_f32);
    dst.q(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.approach_x().set(some_f32);
    dst.approach_y().set(some_f32);
    dst.approach_z().set(some_f32);
    dst.time_usec().set(some_i64);
}

fn read_FENCE_STATUS(src: &mut packs::FENCE_STATUS::Data_) {
    let breach_count = src.breach_count();
    let breach_time = src.breach_time();
    let breach_status = src.breach_status();
    if let Some(breach_type) = src.breach_type().get() {}
}

fn write_FENCE_STATUS(dst: &mut packs::FENCE_STATUS::Data_) {
    dst.breach_count().set(some_i16);
    dst.breach_time().set(some_i32);
    dst.breach_status().set(some_i8);
    dst.breach_type().set(packs::FENCE_BREACH);
}

fn read_REMOTE_LOG_BLOCK_STATUS(src: &mut packs::REMOTE_LOG_BLOCK_STATUS::Data_) {
    let seqno = src.seqno();
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(status) = src.status().get() {}
}

fn write_REMOTE_LOG_BLOCK_STATUS(dst: &mut packs::REMOTE_LOG_BLOCK_STATUS::Data_) {
    dst.seqno().set(some_i32);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.status().set(packs::MAV_REMOTE_LOG_DATA_BLOCK_STATUSES);
}

fn read_OBSTACLE_DISTANCE(src: &mut packs::OBSTACLE_DISTANCE::Data_) {
    let distances = src.distances(None);
    let min_distance = src.min_distance();
    let max_distance = src.max_distance();
    let time_usec = src.time_usec();
    let increment = src.increment();
    if let Some(sensor_type) = src.sensor_type().get() {}
}

fn write_OBSTACLE_DISTANCE(dst: &mut packs::OBSTACLE_DISTANCE::Data_) {
    dst.distances(Some(&mut sys::ByValIter::new(&some_i16s)));
    dst.min_distance().set(some_i16);
    dst.max_distance().set(some_i16);
    dst.time_usec().set(some_i64);
    dst.increment().set(some_i8);
    dst.sensor_type().set(packs::MAV_DISTANCE_SENSOR);
}

fn read_GPS2_RAW(src: &mut packs::GPS2_RAW::Data_) {
    let eph = src.eph();
    let epv = src.epv();
    let vel = src.vel();
    let cog = src.cog();
    let dgps_age = src.dgps_age();
    let time_usec = src.time_usec();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let satellites_visible = src.satellites_visible();
    let dgps_numch = src.dgps_numch();
    if let Some(fix_type) = src.fix_type().get() {}
}

fn write_GPS2_RAW(dst: &mut packs::GPS2_RAW::Data_) {
    dst.eph().set(some_i16);
    dst.epv().set(some_i16);
    dst.vel().set(some_i16);
    dst.cog().set(some_i16);
    dst.dgps_age().set(some_i32);
    dst.time_usec().set(some_i64);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.alt().set(some_i32);
    dst.satellites_visible().set(some_i8);
    dst.dgps_numch().set(some_i8);
    dst.fix_type().set(packs::GPS_FIX_TYPE);
}

fn read_REQUEST_DATA_STREAM(src: &mut packs::REQUEST_DATA_STREAM::Data_) {
    let req_message_rate = src.req_message_rate();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let req_stream_id = src.req_stream_id();
    let start_stop = src.start_stop();
}

fn read_MEMORY_VECT(src: &mut packs::MEMORY_VECT::Data_) {
    let address = src.address();
    let ver = src.ver();
    let typE = src.typE();
    let value = src.value(None);
}

fn write_MEMORY_VECT(dst: &mut packs::MEMORY_VECT::Data_) {
    dst.address().set(some_i16);
    dst.ver().set(some_i8);
    dst.typE().set(some_i8);
    dst.value(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_PARAM_EXT_REQUEST_READ(src: &mut packs::PARAM_EXT_REQUEST_READ::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let param_index = src.param_index();
    if let Some(src) = src.param_id().get() {}
}

fn write_PARAM_EXT_REQUEST_READ(dst: &mut packs::PARAM_EXT_REQUEST_READ::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.param_index().set(some_i16);
    dst.param_id().set(some_str);
}

fn read_HIL_CONTROLS(src: &mut packs::HIL_CONTROLS::Data_) {
    let time_usec = src.time_usec();
    let roll_ailerons = src.roll_ailerons();
    let pitch_elevator = src.pitch_elevator();
    let yaw_rudder = src.yaw_rudder();
    let throttle = src.throttle();
    let aux1 = src.aux1();
    let aux2 = src.aux2();
    let aux3 = src.aux3();
    let aux4 = src.aux4();
    let nav_mode = src.nav_mode();
    if let Some(mode) = src.mode() {}
}

fn read_HIL_SENSOR(src: &mut packs::HIL_SENSOR::Data_) {
    let fields_updated = src.fields_updated();
    let time_usec = src.time_usec();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
    let xgyro = src.xgyro();
    let ygyro = src.ygyro();
    let zgyro = src.zgyro();
    let xmag = src.xmag();
    let ymag = src.ymag();
    let zmag = src.zmag();
    let abs_pressure = src.abs_pressure();
    let diff_pressure = src.diff_pressure();
    let pressure_alt = src.pressure_alt();
    let temperature = src.temperature();
}

fn write_HIL_SENSOR(dst: &mut packs::HIL_SENSOR::Data_) {
    dst.fields_updated().set(some_i32);
    dst.time_usec().set(some_i64);
    dst.xacc().set(some_f32);
    dst.yacc().set(some_f32);
    dst.zacc().set(some_f32);
    dst.xgyro().set(some_f32);
    dst.ygyro().set(some_f32);
    dst.zgyro().set(some_f32);
    dst.xmag().set(some_f32);
    dst.ymag().set(some_f32);
    dst.zmag().set(some_f32);
    dst.abs_pressure().set(some_f32);
    dst.diff_pressure().set(some_f32);
    dst.pressure_alt().set(some_f32);
    dst.temperature().set(some_f32);
}

fn read_SETUP_SIGNING(src: &mut packs::SETUP_SIGNING::Data_) {
    let initial_timestamp = src.initial_timestamp();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let secret_key = src.secret_key(None);
}

fn write_SETUP_SIGNING(dst: &mut packs::SETUP_SIGNING::Data_) {
    dst.initial_timestamp().set(some_i64);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.secret_key(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_GPS_RTK(src: &mut packs::GPS_RTK::Data_) {
    let wn = src.wn();
    let time_last_baseline_ms = src.time_last_baseline_ms();
    let tow = src.tow();
    let accuracy = src.accuracy();
    let rtk_receiver_id = src.rtk_receiver_id();
    let rtk_health = src.rtk_health();
    let rtk_rate = src.rtk_rate();
    let nsats = src.nsats();
    let baseline_coords_type = src.baseline_coords_type();
    let baseline_a_mm = src.baseline_a_mm();
    let baseline_b_mm = src.baseline_b_mm();
    let baseline_c_mm = src.baseline_c_mm();
    let iar_num_hypotheses = src.iar_num_hypotheses();
}

fn write_GPS_RTK(dst: &mut packs::GPS_RTK::Data_) {
    dst.wn().set(some_i16);
    dst.time_last_baseline_ms().set(some_i32);
    dst.tow().set(some_i32);
    dst.accuracy().set(some_i32);
    dst.rtk_receiver_id().set(some_i8);
    dst.rtk_health().set(some_i8);
    dst.rtk_rate().set(some_i8);
    dst.nsats().set(some_i8);
    dst.baseline_coords_type().set(some_i8);
    dst.baseline_a_mm().set(some_i32);
    dst.baseline_b_mm().set(some_i32);
    dst.baseline_c_mm().set(some_i32);
    dst.iar_num_hypotheses().set(some_i32);
}

fn read_PARAM_REQUEST_LIST(src: &mut packs::PARAM_REQUEST_LIST::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn read_UAVIONIX_ADSB_OUT_CFG(src: &mut packs::UAVIONIX_ADSB_OUT_CFG::Data_) {
    let stallSpeed = src.stallSpeed();
    let ICAO = src.ICAO();
    if let Some(src) = src.callsign().get() {}
    if let Some(emitterType) = src.emitterType().get() {}
    if let Some(aircraftSize) = src.aircraftSize().get() {}
    if let Some(gpsOffsetLat) = src.gpsOffsetLat().get() {}
    if let Some(gpsOffsetLon) = src.gpsOffsetLon().get() {}
    if let Some(rfSelect) = src.rfSelect().get() {}
}

fn write_UAVIONIX_ADSB_OUT_CFG(dst: &mut packs::UAVIONIX_ADSB_OUT_CFG::Data_) {
    dst.stallSpeed().set(some_i16);
    dst.ICAO().set(some_i32);
    dst.callsign().set(some_str);
    dst.emitterType().set(packs::ADSB_EMITTER_TYPE);
    dst.aircraftSize().set(packs::UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE);
    dst.gpsOffsetLat().set(packs::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT);
    dst.gpsOffsetLon().set(packs::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON);
    dst.rfSelect().set(packs::UAVIONIX_ADSB_OUT_RF_SELECT);
}

fn read_LANDING_TARGET(src: &mut packs::LANDING_TARGET::Data_) {
    let time_usec = src.time_usec();
    let target_num = src.target_num();
    let angle_x = src.angle_x();
    let angle_y = src.angle_y();
    let distance = src.distance();
    let size_x = src.size_x();
    let size_y = src.size_y();
    if let Some(frame) = src.frame().get() {}
    if let Some(x) = src.x().get() {}
    if let Some(y) = src.y().get() {}
    if let Some(z) = src.z().get() {}
    if let Some(mut src_q) = src.q().items() {
        src_q.enumerate(0, |src, d0| if let Some(q) = src.get(d0) {});
    }
    if let Some(typE) = src.typE().get() {}
    if let Some(position_valid) = src.position_valid().get() {}
}

fn write_LANDING_TARGET(dst: &mut packs::LANDING_TARGET::Data_) {
    dst.time_usec().set(some_i64);
    dst.target_num().set(some_i8);
    dst.angle_x().set(some_f32);
    dst.angle_y().set(some_f32);
    dst.distance().set(some_f32);
    dst.size_x().set(some_f32);
    dst.size_y().set(some_f32);
    dst.frame().set(packs::MAV_FRAME);
    dst.x().set(some_f32);
    dst.y().set(some_f32);
    dst.z().set(some_f32);
    let mut dst_q = dst.q();

    dst_q.enumerate(0, |dst, d0| {
        dst.set(some_f32, d0);
    });
    dst.typE().set(packs::LANDING_TARGET_TYPE);
    dst.position_valid().set(some_i8);
}

fn read_SET_ACTUATOR_CONTROL_TARGET(src: &mut packs::SET_ACTUATOR_CONTROL_TARGET::Data_) {
    let time_usec = src.time_usec();
    let group_mlx = src.group_mlx();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let controls = src.controls(None);
}

fn write_SET_ACTUATOR_CONTROL_TARGET(dst: &mut packs::SET_ACTUATOR_CONTROL_TARGET::Data_) {
    dst.time_usec().set(some_i64);
    dst.group_mlx().set(some_i8);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.controls(Some(&mut sys::ByValIter::new(&some_f32s)));
}

fn read_CONTROL_SYSTEM_STATE(src: &mut packs::CONTROL_SYSTEM_STATE::Data_) {
    let time_usec = src.time_usec();
    let x_acc = src.x_acc();
    let y_acc = src.y_acc();
    let z_acc = src.z_acc();
    let x_vel = src.x_vel();
    let y_vel = src.y_vel();
    let z_vel = src.z_vel();
    let x_pos = src.x_pos();
    let y_pos = src.y_pos();
    let z_pos = src.z_pos();
    let airspeed = src.airspeed();
    let vel_variance = src.vel_variance(None);
    let pos_variance = src.pos_variance(None);
    let q = src.q(None);
    let roll_rate = src.roll_rate();
    let pitch_rate = src.pitch_rate();
    let yaw_rate = src.yaw_rate();
}

fn write_CONTROL_SYSTEM_STATE(dst: &mut packs::CONTROL_SYSTEM_STATE::Data_) {
    dst.time_usec().set(some_i64);
    dst.x_acc().set(some_f32);
    dst.y_acc().set(some_f32);
    dst.z_acc().set(some_f32);
    dst.x_vel().set(some_f32);
    dst.y_vel().set(some_f32);
    dst.z_vel().set(some_f32);
    dst.x_pos().set(some_f32);
    dst.y_pos().set(some_f32);
    dst.z_pos().set(some_f32);
    dst.airspeed().set(some_f32);
    dst.vel_variance(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.pos_variance(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.q(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.roll_rate().set(some_f32);
    dst.pitch_rate().set(some_f32);
    dst.yaw_rate().set(some_f32);
}

fn read_SET_POSITION_TARGET_GLOBAL_INT(src: &mut packs::SET_POSITION_TARGET_GLOBAL_INT::Data_) {
    let type_mask = src.type_mask();
    let time_boot_ms = src.time_boot_ms();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let lat_int = src.lat_int();
    let lon_int = src.lon_int();
    let alt = src.alt();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let afx = src.afx();
    let afy = src.afy();
    let afz = src.afz();
    let yaw = src.yaw();
    let yaw_rate = src.yaw_rate();
    if let Some(coordinate_frame) = src.coordinate_frame() {}
}

fn read_DATA32(src: &mut packs::DATA32::Data_) {
    let typE = src.typE();
    let len = src.len();
    let daTa = src.daTa(None);
}

fn write_DATA32(dst: &mut packs::DATA32::Data_) {
    dst.typE().set(some_i8);
    dst.len().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_PING33(src: &mut packs::PING33::Data_) {
    let mut src_TTTT = src.TTTT();

    src_TTTT.enumerate(0, 0, 0, |src, d0, d1, d2| {
        let TTTT = src.get(d0, d1, d2);
    });
    let field = src.field();
    let bit_field = src.bit_field();
    let mut src_field6 = src.field6();

    src_field6.enumerate(0, 0, 0, |src, d0, d1, d2| {
        let field6 = src.get(d0, d1, d2);
    });
    let testBOOL2 = src.testBOOL2();
    let testBOOL3 = src.testBOOL3();
    if let Some(testBOOL) = src.testBOOL().get() {}
    if let Some(seq) = src.seq().get() {}
    if let Ok(mut src_field1) = src.field1() {
        src_field1.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let field1 = src.get(d0, d1, d2);
        });
    }
    if let Some(mut src_field12) = src.field12().items() {
        src_field12.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let field12 = src.get(d0, d1, d2);
        });
    }
    if let Some(mut src_field13) = src.field13().items() {
        src_field13.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(field13) = src.get(d0, d1, d2) {});
    }
    if let Some(WWWWWWWW) = src.WWWWWWWW().get() {}
    if let Some(bit_field2) = src.bit_field2().get() {}
    if let Some(mut src_Field_Bits) = src.Field_Bits().items() {
        src_Field_Bits.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let Field_Bits = src.get(d0, d1, d2);
        });
    }
    if let Ok(mut src_SparseFixAllBits) = src.SparseFixAllBits() {
        src_SparseFixAllBits.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(SparseFixAllBits) = src.get(d0, d1, d2) {});
    }
    if let Ok(mut src_FixAllBits) = src.FixAllBits() {
        src_FixAllBits.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let FixAllBits = src.get(d0, d1, d2);
        });
    }
    if let Ok(mut src_VarAllBits) = src.VarAllBits() {
        src_VarAllBits.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let VarAllBits = src.get(d0, d1, d2);
        });
    }
    if let Ok(mut src_SparseVarAllBits) = src.SparseVarAllBits() {
        src_SparseVarAllBits.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(SparseVarAllBits) = src.get(d0, d1, d2) {});
    }
    if let Ok(mut src_VarEachBits) = src.VarEachBits() {
        src_VarEachBits.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let VarEachBits = src.get(d0, d1, d2);
        });
    }
    if let Ok(mut src_SparsVarEachBits) = src.SparsVarEachBits() {
        src_SparsVarEachBits.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(SparsVarEachBits) = src.get(d0, d1, d2) {});
    }
    if let Some(testBOOLX) = src.testBOOLX().get() {}
    if let Some(testBOOL2X) = src.testBOOL2X().get() {}
    if let Some(testBOOL3X) = src.testBOOL3X().get() {}
    if let Some(MMMMMM) = src.MMMMMM().get() {}
    if let Ok(mut src_field44) = src.field44() {
        src_field44.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let field44 = src.get(d0, d1, d2);
        });
    }
    if let Ok(mut src_field634) = src.field634() {
        src_field634.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let field634 = src.get(d0, d1, d2);
        });
    }
    if let Ok(mut src_field33344) = src.field33344() {
        src_field33344.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(field33344) = src.get(d0, d1, d2) {});
    }
    if let Ok(mut src_field333634) = src.field333634() {
        src_field333634.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(field333634) = src.get(d0, d1, d2) {});
    }
    if let Some(mut src_field__) = src.field__().items() {
        src_field__.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(field__) = src.get(d0, d1, d2) {});
    }
    if let Ok(mut src_field63) = src.field63() {
        src_field63.enumerate(0, 0, 0, |src, d0, d1, d2| {
            let field63 = src.get(d0, d1, d2);
        });
    }
    if let Some(mut src_uid2) = src.uid2().items() {
        src_uid2.enumerate(0, |src, d0| if let Some(uid2) = src.get(d0) {});
    }
    if let Some(mut src_field2) = src.field2().items() {
        src_field2.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(field2) = src.get(d0, d1, d2) {});
    }
    if let Some(mut src_field4) = src.field4().items() {
        src_field4.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(field4) = src.get(d0, d1, d2) {});
    }
    if let Some(src) = src.stringtest1().get() {}
    if let Ok(mut src_stringtest2) = src.stringtest2() {
        src_stringtest2.enumerate(0, 0, 0, |src, d0, d1, d2| if let Some(stringtest2) = src.get(d0, d1, d2) {});
    }
    if let Some(src) = src.stringtest3().get() {}
    if let Some(src) = src.stringtest4().get() {}
}

fn write_PING33(dst: &mut packs::PING33::Data_) {
    let mut dst_TTTT = dst.TTTT();

    dst_TTTT.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    dst.field().set(some_i64);
    dst.bit_field().set(some_i8);
    let mut dst_field6 = dst.field6();

    dst_field6.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    dst.testBOOL2().set(some_bool);
    dst.testBOOL3().set(some_bool);
    dst.testBOOL().set(some_bool);
    dst.seq().set(some_i64);

    let mut dst_field1 = match dst.field1() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_field1.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    let mut dst_field12 = dst.field12();

    dst_field12.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    let mut dst_field13 = dst.field13();

    dst_field13.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    dst.WWWWWWWW().set(some_i32);
    dst.bit_field2().set(some_i8);
    let mut dst_Field_Bits = dst.Field_Bits();

    dst_Field_Bits.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i8, d0, d1, d2);
    });

    let mut dst_SparseFixAllBits = match dst.SparseFixAllBits() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_SparseFixAllBits.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i8, d0, d1, d2);
    });

    let mut dst_FixAllBits = match dst.FixAllBits() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_FixAllBits.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i8, d0, d1, d2);
    });

    let mut dst_VarAllBits = match dst.VarAllBits() {
        Err(init) => init.set_field(List(1, 1)),
        Ok(set) => set,
    };

    dst_VarAllBits.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i8, d0, d1, d2);
    });

    let mut dst_SparseVarAllBits = match dst.SparseVarAllBits() {
        Err(init) => init.set_field(List(1, 1)),
        Ok(set) => set,
    };

    dst_SparseVarAllBits.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i8, d0, d1, d2);
    });

    let mut dst_VarEachBits = match dst.VarEachBits() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_VarEachBits.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i8, d0, d1, d2);
    });

    let mut dst_SparsVarEachBits = match dst.SparsVarEachBits() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_SparsVarEachBits.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i16, d0, d1, d2);
    });
    dst.testBOOLX().set(some_bool);
    dst.testBOOL2X().set(some_bool);
    dst.testBOOL3X().set(some_bool);
    dst.MMMMMM().set(packs::MAV_MODE);

    let mut dst_field44 = match dst.field44() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_field44.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });

    let mut dst_field634 = match dst.field634() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_field634.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });

    let mut dst_field33344 = match dst.field33344() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_field33344.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });

    let mut dst_field333634 = match dst.field333634() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_field333634.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    let mut dst_field__ = dst.field__();

    dst_field__.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });

    let mut dst_field63 = match dst.field63() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_field63.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    let mut dst_uid2 = dst.uid2();

    dst_uid2.enumerate(0, |dst, d0| {
        dst.set(some_i8, d0);
    });
    let mut dst_field2 = dst.field2();

    dst_field2.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    let mut dst_field4 = dst.field4();

    dst_field4.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_i32, d0, d1, d2);
    });
    dst.stringtest1().set(some_str);

    let mut dst_stringtest2 = match dst.stringtest2() {
        Err(init) => init.set_field(List(1)),
        Ok(set) => set,
    };

    dst_stringtest2.enumerate(0, 0, 0, |dst, d0, d1, d2| {
        dst.set(some_str, d0, d1, d2);
    });
    dst.stringtest3().set(some_str);
    dst.stringtest4().set(some_str);
}

fn read_VFR_HUD(src: &mut packs::VFR_HUD::Data_) {
    let throttle = src.throttle();
    let airspeed = src.airspeed();
    let groundspeed = src.groundspeed();
    let heading = src.heading();
    let alt = src.alt();
    let climb = src.climb();
}

fn read_RALLY_POINT(src: &mut packs::RALLY_POINT::Data_) {
    let land_dir = src.land_dir();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let idx = src.idx();
    let count = src.count();
    let lat = src.lat();
    let lng = src.lng();
    let alt = src.alt();
    let break_alt = src.break_alt();
    if let Some(flags) = src.flags().get() {}
}

fn write_RALLY_POINT(dst: &mut packs::RALLY_POINT::Data_) {
    dst.land_dir().set(some_i16);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.idx().set(some_i8);
    dst.count().set(some_i8);
    dst.lat().set(some_i32);
    dst.lng().set(some_i32);
    dst.alt().set(some_i16);
    dst.break_alt().set(some_i16);
    dst.flags().set(packs::RALLY_FLAGS);
}

fn read_MISSION_SET_CURRENT(src: &mut packs::MISSION_SET_CURRENT::Data_) {
    let seq = src.seq();
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn read_ADAP_TUNING(src: &mut packs::ADAP_TUNING::Data_) {
    let desired = src.desired();
    let achieved = src.achieved();
    let error = src.error();
    let theta = src.theta();
    let omega = src.omega();
    let sigma = src.sigma();
    let theta_dot = src.theta_dot();
    let omega_dot = src.omega_dot();
    let sigma_dot = src.sigma_dot();
    let f = src.f();
    let f_dot = src.f_dot();
    let u = src.u();
    if let Some(axis) = src.axis().get() {}
}

fn write_ADAP_TUNING(dst: &mut packs::ADAP_TUNING::Data_) {
    dst.desired().set(some_f32);
    dst.achieved().set(some_f32);
    dst.error().set(some_f32);
    dst.theta().set(some_f32);
    dst.omega().set(some_f32);
    dst.sigma().set(some_f32);
    dst.theta_dot().set(some_f32);
    dst.omega_dot().set(some_f32);
    dst.sigma_dot().set(some_f32);
    dst.f().set(some_f32);
    dst.f_dot().set(some_f32);
    dst.u().set(some_f32);
    dst.axis().set(packs::PID_TUNING_AXIS);
}

fn read_VIBRATION(src: &mut packs::VIBRATION::Data_) {
    let clipping_0 = src.clipping_0();
    let clipping_1 = src.clipping_1();
    let clipping_2 = src.clipping_2();
    let time_usec = src.time_usec();
    let vibration_x = src.vibration_x();
    let vibration_y = src.vibration_y();
    let vibration_z = src.vibration_z();
}

fn write_VIBRATION(dst: &mut packs::VIBRATION::Data_) {
    dst.clipping_0().set(some_i32);
    dst.clipping_1().set(some_i32);
    dst.clipping_2().set(some_i32);
    dst.time_usec().set(some_i64);
    dst.vibration_x().set(some_f32);
    dst.vibration_y().set(some_f32);
    dst.vibration_z().set(some_f32);
}

fn read_PARAM_EXT_VALUE(src: &mut packs::PARAM_EXT_VALUE::Data_) {
    let param_count = src.param_count();
    let param_index = src.param_index();
    if let Some(src) = src.param_id().get() {}
    if let Some(src) = src.param_value().get() {}
    if let Some(param_type) = src.param_type().get() {}
}

fn write_PARAM_EXT_VALUE(dst: &mut packs::PARAM_EXT_VALUE::Data_) {
    dst.param_count().set(some_i16);
    dst.param_index().set(some_i16);
    dst.param_id().set(some_str);
    dst.param_value().set(some_str);
    dst.param_type().set(packs::MAV_PARAM_EXT_TYPE);
}

fn read_BATTERY2(src: &mut packs::BATTERY2::Data_) {
    let voltage = src.voltage();
    let current_battery = src.current_battery();
}

fn write_BATTERY2(dst: &mut packs::BATTERY2::Data_) {
    dst.voltage().set(some_i16);
    dst.current_battery().set(some_i16);
}

fn read_LIMITS_STATUS(src: &mut packs::LIMITS_STATUS::Data_) {
    let breach_count = src.breach_count();
    let last_trigger = src.last_trigger();
    let last_action = src.last_action();
    let last_recovery = src.last_recovery();
    let last_clear = src.last_clear();
    if let Some(limits_state) = src.limits_state().get() {}
    if let Some(mods_enabled) = src.mods_enabled().get() {}
    if let Some(mods_required) = src.mods_required().get() {}
    if let Some(mods_triggered) = src.mods_triggered().get() {}
}

fn write_LIMITS_STATUS(dst: &mut packs::LIMITS_STATUS::Data_) {
    dst.breach_count().set(some_i16);
    dst.last_trigger().set(some_i32);
    dst.last_action().set(some_i32);
    dst.last_recovery().set(some_i32);
    dst.last_clear().set(some_i32);
    dst.limits_state().set(packs::LIMITS_STATE);
    dst.mods_enabled().set(packs::LIMIT_MODULE);
    dst.mods_required().set(packs::LIMIT_MODULE);
    dst.mods_triggered().set(packs::LIMIT_MODULE);
}

fn read_CAMERA_FEEDBACK(src: &mut packs::CAMERA_FEEDBACK::Data_) {
    let img_idx = src.img_idx();
    let time_usec = src.time_usec();
    let target_system = src.target_system();
    let cam_idx = src.cam_idx();
    let lat = src.lat();
    let lng = src.lng();
    let alt_msl = src.alt_msl();
    let alt_rel = src.alt_rel();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
    let foc_len = src.foc_len();
    if let Some(flags) = src.flags().get() {}
}

fn write_CAMERA_FEEDBACK(dst: &mut packs::CAMERA_FEEDBACK::Data_) {
    dst.img_idx().set(some_i16);
    dst.time_usec().set(some_i64);
    dst.target_system().set(some_i8);
    dst.cam_idx().set(some_i8);
    dst.lat().set(some_i32);
    dst.lng().set(some_i32);
    dst.alt_msl().set(some_f32);
    dst.alt_rel().set(some_f32);
    dst.roll().set(some_f32);
    dst.pitch().set(some_f32);
    dst.yaw().set(some_f32);
    dst.foc_len().set(some_f32);
    dst.flags().set(packs::CAMERA_FEEDBACK_FLAGS);
}

fn read_HIL_GPS(src: &mut packs::HIL_GPS::Data_) {
    let eph = src.eph();
    let epv = src.epv();
    let vel = src.vel();
    let cog = src.cog();
    let time_usec = src.time_usec();
    let fix_type = src.fix_type();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let vn = src.vn();
    let ve = src.ve();
    let vd = src.vd();
    let satellites_visible = src.satellites_visible();
}

fn write_HIL_GPS(dst: &mut packs::HIL_GPS::Data_) {
    dst.eph().set(some_i16);
    dst.epv().set(some_i16);
    dst.vel().set(some_i16);
    dst.cog().set(some_i16);
    dst.time_usec().set(some_i64);
    dst.fix_type().set(some_i8);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.alt().set(some_i32);
    dst.vn().set(some_i16);
    dst.ve().set(some_i16);
    dst.vd().set(some_i16);
    dst.satellites_visible().set(some_i8);
}

fn read_NAV_CONTROLLER_OUTPUT(src: &mut packs::NAV_CONTROLLER_OUTPUT::Data_) {
    let wp_dist = src.wp_dist();
    let nav_roll = src.nav_roll();
    let nav_pitch = src.nav_pitch();
    let nav_bearing = src.nav_bearing();
    let target_bearing = src.target_bearing();
    let alt_error = src.alt_error();
    let aspd_error = src.aspd_error();
    let xtrack_error = src.xtrack_error();
}

fn read_AUTH_KEY(src: &mut packs::AUTH_KEY::Data_) { if let Some(src) = src.key() {} }

fn read_FENCE_FETCH_POINT(src: &mut packs::FENCE_FETCH_POINT::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let idx = src.idx();
}

fn write_FENCE_FETCH_POINT(dst: &mut packs::FENCE_FETCH_POINT::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.idx().set(some_i8);
}

fn read_RADIO(src: &mut packs::RADIO::Data_) {
    let rxerrors = src.rxerrors();
    let fixeD = src.fixeD();
    let rssi = src.rssi();
    let remrssi = src.remrssi();
    let txbuf = src.txbuf();
    let noise = src.noise();
    let remnoise = src.remnoise();
}

fn write_RADIO(dst: &mut packs::RADIO::Data_) {
    dst.rxerrors().set(some_i16);
    dst.fixeD().set(some_i16);
    dst.rssi().set(some_i8);
    dst.remrssi().set(some_i8);
    dst.txbuf().set(some_i8);
    dst.noise().set(some_i8);
    dst.remnoise().set(some_i8);
}

fn read_LOCAL_POSITION_NED_COV(src: &mut packs::LOCAL_POSITION_NED_COV::Data_) {
    let time_usec = src.time_usec();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let ax = src.ax();
    let ay = src.ay();
    let az = src.az();
    let covariance = src.covariance();
    if let Some(estimator_type) = src.estimator_type() {}
}

fn read_AIRSPEED_AUTOCAL(src: &mut packs::AIRSPEED_AUTOCAL::Data_) {
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let diff_pressure = src.diff_pressure();
    let EAS2TAS = src.EAS2TAS();
    let ratio = src.ratio();
    let state_x = src.state_x();
    let state_y = src.state_y();
    let state_z = src.state_z();
    let Pax = src.Pax();
    let Pby = src.Pby();
    let Pcz = src.Pcz();
}

fn write_AIRSPEED_AUTOCAL(dst: &mut packs::AIRSPEED_AUTOCAL::Data_) {
    dst.vx().set(some_f32);
    dst.vy().set(some_f32);
    dst.vz().set(some_f32);
    dst.diff_pressure().set(some_f32);
    dst.EAS2TAS().set(some_f32);
    dst.ratio().set(some_f32);
    dst.state_x().set(some_f32);
    dst.state_y().set(some_f32);
    dst.state_z().set(some_f32);
    dst.Pax().set(some_f32);
    dst.Pby().set(some_f32);
    dst.Pcz().set(some_f32);
}

fn read_ATT_POS_MOCAP(src: &mut packs::ATT_POS_MOCAP::Data_) {
    let time_usec = src.time_usec();
    let q = src.q(None);
    let x = src.x();
    let y = src.y();
    let z = src.z();
}

fn write_ATT_POS_MOCAP(dst: &mut packs::ATT_POS_MOCAP::Data_) {
    dst.time_usec().set(some_i64);
    dst.q(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.x().set(some_f32);
    dst.y().set(some_f32);
    dst.z().set(some_f32);
}

fn read_STATUSTEXT(src: &mut packs::STATUSTEXT::Data_) {
    if let Some(severity) = src.severity().get() {}
    if let Some(src) = src.text().get() {}
}

fn write_STATUSTEXT(dst: &mut packs::STATUSTEXT::Data_) {
    dst.severity().set(packs::MAV_SEVERITY);
    dst.text().set(some_str);
}

fn read_PING(src: &mut packs::PING::Data_) {
    let seq = src.seq();
    let time_usec = src.time_usec();
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn read_GOPRO_GET_REQUEST(src: &mut packs::GOPRO_GET_REQUEST::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(cmd_id) = src.cmd_id().get() {}
}

fn write_GOPRO_GET_REQUEST(dst: &mut packs::GOPRO_GET_REQUEST::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.cmd_id().set(packs::GOPRO_COMMAND);
}

fn read_CAMERA_CAPTURE_STATUS(src: &mut packs::CAMERA_CAPTURE_STATUS::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let recording_time_ms = src.recording_time_ms();
    let image_status = src.image_status();
    let video_status = src.video_status();
    let image_interval = src.image_interval();
    let available_capacity = src.available_capacity();
}

fn write_CAMERA_CAPTURE_STATUS(dst: &mut packs::CAMERA_CAPTURE_STATUS::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.recording_time_ms().set(some_i32);
    dst.image_status().set(some_i8);
    dst.video_status().set(some_i8);
    dst.image_interval().set(some_f32);
    dst.available_capacity().set(some_f32);
}

fn read_GLOBAL_POSITION_INT(src: &mut packs::GLOBAL_POSITION_INT::Data_) {
    let hdg = src.hdg();
    let time_boot_ms = src.time_boot_ms();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let relative_alt = src.relative_alt();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
}

fn read_ENCAPSULATED_DATA(src: &mut packs::ENCAPSULATED_DATA::Data_) {
    let seqnr = src.seqnr();
    let daTa = src.daTa(None);
}

fn write_ENCAPSULATED_DATA(dst: &mut packs::ENCAPSULATED_DATA::Data_) {
    dst.seqnr().set(some_i16);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_GPS_INPUT(src: &mut packs::GPS_INPUT::Data_) {
    let time_week = src.time_week();
    let time_week_ms = src.time_week_ms();
    let time_usec = src.time_usec();
    let gps_id = src.gps_id();
    let fix_type = src.fix_type();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let hdop = src.hdop();
    let vdop = src.vdop();
    let vn = src.vn();
    let ve = src.ve();
    let vd = src.vd();
    let speed_accuracy = src.speed_accuracy();
    let horiz_accuracy = src.horiz_accuracy();
    let vert_accuracy = src.vert_accuracy();
    let satellites_visible = src.satellites_visible();
    if let Some(ignore_flags) = src.ignore_flags().get() {}
}

fn write_GPS_INPUT(dst: &mut packs::GPS_INPUT::Data_) {
    dst.time_week().set(some_i16);
    dst.time_week_ms().set(some_i32);
    dst.time_usec().set(some_i64);
    dst.gps_id().set(some_i8);
    dst.fix_type().set(some_i8);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.alt().set(some_f32);
    dst.hdop().set(some_f32);
    dst.vdop().set(some_f32);
    dst.vn().set(some_f32);
    dst.ve().set(some_f32);
    dst.vd().set(some_f32);
    dst.speed_accuracy().set(some_f32);
    dst.horiz_accuracy().set(some_f32);
    dst.vert_accuracy().set(some_f32);
    dst.satellites_visible().set(some_i8);
    dst.ignore_flags().set(packs::GPS_INPUT_IGNORE_FLAGS);
}

fn read_COMMAND_LONG(src: &mut packs::COMMAND_LONG::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let confirmation = src.confirmation();
    let param1 = src.param1();
    let param2 = src.param2();
    let param3 = src.param3();
    let param4 = src.param4();
    let param5 = src.param5();
    let param6 = src.param6();
    let param7 = src.param7();
    if let Some(command) = src.command() {}
}

fn read_COMPASSMOT_STATUS(src: &mut packs::COMPASSMOT_STATUS::Data_) {
    let throttle = src.throttle();
    let interference = src.interference();
    let current = src.current();
    let CompensationX = src.CompensationX();
    let CompensationY = src.CompensationY();
    let CompensationZ = src.CompensationZ();
}

fn write_COMPASSMOT_STATUS(dst: &mut packs::COMPASSMOT_STATUS::Data_) {
    dst.throttle().set(some_i16);
    dst.interference().set(some_i16);
    dst.current().set(some_f32);
    dst.CompensationX().set(some_f32);
    dst.CompensationY().set(some_f32);
    dst.CompensationZ().set(some_f32);
}

fn read_LOG_REQUEST_DATA(src: &mut packs::LOG_REQUEST_DATA::Data_) {
    let id = src.id();
    let ofs = src.ofs();
    let count = src.count();
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn write_LOG_REQUEST_DATA(dst: &mut packs::LOG_REQUEST_DATA::Data_) {
    dst.id().set(some_i16);
    dst.ofs().set(some_i32);
    dst.count().set(some_i32);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
}

fn read_GPS_RAW_INT(src: &mut packs::GPS_RAW_INT::Data_) {
    let eph = src.eph();
    let epv = src.epv();
    let vel = src.vel();
    let cog = src.cog();
    let time_usec = src.time_usec();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let satellites_visible = src.satellites_visible();
    if let Some(fix_type) = src.fix_type() {}
    if let Some(alt_ellipsoid) = src.alt_ellipsoid() {}
    if let Some(h_acc) = src.h_acc() {}
    if let Some(v_acc) = src.v_acc() {}
    if let Some(vel_acc) = src.vel_acc() {}
    if let Some(hdg_acc) = src.hdg_acc() {}
}

fn read_CAMERA_STATUS(src: &mut packs::CAMERA_STATUS::Data_) {
    let img_idx = src.img_idx();
    let time_usec = src.time_usec();
    let target_system = src.target_system();
    let cam_idx = src.cam_idx();
    let p1 = src.p1();
    let p2 = src.p2();
    let p3 = src.p3();
    let p4 = src.p4();
    if let Some(event_id) = src.event_id().get() {}
}

fn write_CAMERA_STATUS(dst: &mut packs::CAMERA_STATUS::Data_) {
    dst.img_idx().set(some_i16);
    dst.time_usec().set(some_i64);
    dst.target_system().set(some_i8);
    dst.cam_idx().set(some_i8);
    dst.p1().set(some_f32);
    dst.p2().set(some_f32);
    dst.p3().set(some_f32);
    dst.p4().set(some_f32);
    dst.event_id().set(packs::CAMERA_STATUS_TYPES);
}

fn read_RC_CHANNELS_SCALED(src: &mut packs::RC_CHANNELS_SCALED::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let port = src.port();
    let chan1_scaled = src.chan1_scaled();
    let chan2_scaled = src.chan2_scaled();
    let chan3_scaled = src.chan3_scaled();
    let chan4_scaled = src.chan4_scaled();
    let chan5_scaled = src.chan5_scaled();
    let chan6_scaled = src.chan6_scaled();
    let chan7_scaled = src.chan7_scaled();
    let chan8_scaled = src.chan8_scaled();
    let rssi = src.rssi();
}

fn read_CAMERA_SETTINGS(src: &mut packs::CAMERA_SETTINGS::Data_) {
    let time_boot_ms = src.time_boot_ms();
    if let Some(mode_id) = src.mode_id().get() {}
}

fn write_CAMERA_SETTINGS(dst: &mut packs::CAMERA_SETTINGS::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.mode_id().set(packs::CAMERA_MODE);
}

fn read_DEVICE_OP_READ_REPLY(src: &mut packs::DEVICE_OP_READ_REPLY::Data_) {
    let request_id = src.request_id();
    let result = src.result();
    let regstart = src.regstart();
    let count = src.count();
    let daTa = src.daTa(None);
}

fn write_DEVICE_OP_READ_REPLY(dst: &mut packs::DEVICE_OP_READ_REPLY::Data_) {
    dst.request_id().set(some_i32);
    dst.result().set(some_i8);
    dst.regstart().set(some_i8);
    dst.count().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_RAW_PRESSURE(src: &mut packs::RAW_PRESSURE::Data_) {
    let time_usec = src.time_usec();
    let press_abs = src.press_abs();
    let press_diff1 = src.press_diff1();
    let press_diff2 = src.press_diff2();
    let temperature = src.temperature();
}

fn read_DIGICAM_CONTROL(src: &mut packs::DIGICAM_CONTROL::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let session = src.session();
    let zoom_pos = src.zoom_pos();
    let zoom_step = src.zoom_step();
    let focus_lock = src.focus_lock();
    let shot = src.shot();
    let command_id = src.command_id();
    let extra_param = src.extra_param();
    let extra_value = src.extra_value();
}

fn write_DIGICAM_CONTROL(dst: &mut packs::DIGICAM_CONTROL::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.session().set(some_i8);
    dst.zoom_pos().set(some_i8);
    dst.zoom_step().set(some_i8);
    dst.focus_lock().set(some_i8);
    dst.shot().set(some_i8);
    dst.command_id().set(some_i8);
    dst.extra_param().set(some_i8);
    dst.extra_value().set(some_f32);
}

fn read_NAMED_VALUE_FLOAT(src: &mut packs::NAMED_VALUE_FLOAT::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let value = src.value();
    if let Some(src) = src.name().get() {}
}

fn write_NAMED_VALUE_FLOAT(dst: &mut packs::NAMED_VALUE_FLOAT::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.value().set(some_f32);
    dst.name().set(some_str);
}

fn read_GOPRO_HEARTBEAT(src: &mut packs::GOPRO_HEARTBEAT::Data_) {
    if let Some(status) = src.status().get() {}
    if let Some(capture_mode) = src.capture_mode().get() {}
    if let Some(flags) = src.flags().get() {}
}

fn write_GOPRO_HEARTBEAT(dst: &mut packs::GOPRO_HEARTBEAT::Data_) {
    dst.status().set(packs::GOPRO_HEARTBEAT_STATUS);
    dst.capture_mode().set(packs::GOPRO_CAPTURE_MODE);
    dst.flags().set(packs::GOPRO_HEARTBEAT_FLAGS);
}

fn read_ATTITUDE(src: &mut packs::ATTITUDE::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
    let rollspeed = src.rollspeed();
    let pitchspeed = src.pitchspeed();
    let yawspeed = src.yawspeed();
}

fn read_MISSION_WRITE_PARTIAL_LIST(src: &mut packs::MISSION_WRITE_PARTIAL_LIST::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let start_index = src.start_index();
    let end_index = src.end_index();
    if let Some(mission_type) = src.mission_type() {}
}

fn read_AHRS2(src: &mut packs::AHRS2::Data_) {
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
    let altitude = src.altitude();
    let lat = src.lat();
    let lng = src.lng();
}

fn write_AHRS2(dst: &mut packs::AHRS2::Data_) {
    dst.roll().set(some_f32);
    dst.pitch().set(some_f32);
    dst.yaw().set(some_f32);
    dst.altitude().set(some_f32);
    dst.lat().set(some_i32);
    dst.lng().set(some_i32);
}

fn read_LOG_ERASE(src: &mut packs::LOG_ERASE::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn write_LOG_ERASE(dst: &mut packs::LOG_ERASE::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
}

fn read_TERRAIN_REQUEST(src: &mut packs::TERRAIN_REQUEST::Data_) {
    let grid_spacing = src.grid_spacing();
    let mask = src.mask();
    let lat = src.lat();
    let lon = src.lon();
}

fn write_TERRAIN_REQUEST(dst: &mut packs::TERRAIN_REQUEST::Data_) {
    dst.grid_spacing().set(some_i16);
    dst.mask().set(some_i64);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
}

fn read_MOUNT_STATUS(src: &mut packs::MOUNT_STATUS::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let pointing_a = src.pointing_a();
    let pointing_b = src.pointing_b();
    let pointing_c = src.pointing_c();
}

fn write_MOUNT_STATUS(dst: &mut packs::MOUNT_STATUS::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.pointing_a().set(some_i32);
    dst.pointing_b().set(some_i32);
    dst.pointing_c().set(some_i32);
}

fn read_MANUAL_SETPOINT(src: &mut packs::MANUAL_SETPOINT::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
    let thrust = src.thrust();
    let mode_switch = src.mode_switch();
    let manual_override_switch = src.manual_override_switch();
}

fn read_PID_TUNING(src: &mut packs::PID_TUNING::Data_) {
    let desired = src.desired();
    let achieved = src.achieved();
    let FF = src.FF();
    let P = src.P();
    let I = src.I();
    let D = src.D();
    if let Some(axis) = src.axis().get() {}
}

fn write_PID_TUNING(dst: &mut packs::PID_TUNING::Data_) {
    dst.desired().set(some_f32);
    dst.achieved().set(some_f32);
    dst.FF().set(some_f32);
    dst.P().set(some_f32);
    dst.I().set(some_f32);
    dst.D().set(some_f32);
    dst.axis().set(packs::PID_TUNING_AXIS);
}

fn read_SAFETY_ALLOWED_AREA(src: &mut packs::SAFETY_ALLOWED_AREA::Data_) {
    let p1x = src.p1x();
    let p1y = src.p1y();
    let p1z = src.p1z();
    let p2x = src.p2x();
    let p2y = src.p2y();
    let p2z = src.p2z();
    if let Some(frame) = src.frame() {}
}

fn read_OPTICAL_FLOW_RAD(src: &mut packs::OPTICAL_FLOW_RAD::Data_) {
    let integration_time_us = src.integration_time_us();
    let time_delta_distance_us = src.time_delta_distance_us();
    let time_usec = src.time_usec();
    let sensor_id = src.sensor_id();
    let integrated_x = src.integrated_x();
    let integrated_y = src.integrated_y();
    let integrated_xgyro = src.integrated_xgyro();
    let integrated_ygyro = src.integrated_ygyro();
    let integrated_zgyro = src.integrated_zgyro();
    let temperature = src.temperature();
    let quality = src.quality();
    let distance = src.distance();
}

fn write_OPTICAL_FLOW_RAD(dst: &mut packs::OPTICAL_FLOW_RAD::Data_) {
    dst.integration_time_us().set(some_i32);
    dst.time_delta_distance_us().set(some_i32);
    dst.time_usec().set(some_i64);
    dst.sensor_id().set(some_i8);
    dst.integrated_x().set(some_f32);
    dst.integrated_y().set(some_f32);
    dst.integrated_xgyro().set(some_f32);
    dst.integrated_ygyro().set(some_f32);
    dst.integrated_zgyro().set(some_f32);
    dst.temperature().set(some_i16);
    dst.quality().set(some_i8);
    dst.distance().set(some_f32);
}

fn read_LOG_DATA(src: &mut packs::LOG_DATA::Data_) {
    let id = src.id();
    let ofs = src.ofs();
    let count = src.count();
    let daTa = src.daTa(None);
}

fn write_LOG_DATA(dst: &mut packs::LOG_DATA::Data_) {
    dst.id().set(some_i16);
    dst.ofs().set(some_i32);
    dst.count().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_MISSION_CLEAR_ALL(src: &mut packs::MISSION_CLEAR_ALL::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(mission_type) = src.mission_type() {}
}

fn read_AHRS3(src: &mut packs::AHRS3::Data_) {
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
    let altitude = src.altitude();
    let lat = src.lat();
    let lng = src.lng();
    let v1 = src.v1();
    let v2 = src.v2();
    let v3 = src.v3();
    let v4 = src.v4();
}

fn write_AHRS3(dst: &mut packs::AHRS3::Data_) {
    dst.roll().set(some_f32);
    dst.pitch().set(some_f32);
    dst.yaw().set(some_f32);
    dst.altitude().set(some_f32);
    dst.lat().set(some_i32);
    dst.lng().set(some_i32);
    dst.v1().set(some_f32);
    dst.v2().set(some_f32);
    dst.v3().set(some_f32);
    dst.v4().set(some_f32);
}

fn read_VICON_POSITION_ESTIMATE(src: &mut packs::VICON_POSITION_ESTIMATE::Data_) {
    let usec = src.usec();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
}

fn write_VICON_POSITION_ESTIMATE(dst: &mut packs::VICON_POSITION_ESTIMATE::Data_) {
    dst.usec().set(some_i64);
    dst.x().set(some_f32);
    dst.y().set(some_f32);
    dst.z().set(some_f32);
    dst.roll().set(some_f32);
    dst.pitch().set(some_f32);
    dst.yaw().set(some_f32);
}

fn read_GPS2_RTK(src: &mut packs::GPS2_RTK::Data_) {
    let wn = src.wn();
    let time_last_baseline_ms = src.time_last_baseline_ms();
    let tow = src.tow();
    let accuracy = src.accuracy();
    let rtk_receiver_id = src.rtk_receiver_id();
    let rtk_health = src.rtk_health();
    let rtk_rate = src.rtk_rate();
    let nsats = src.nsats();
    let baseline_coords_type = src.baseline_coords_type();
    let baseline_a_mm = src.baseline_a_mm();
    let baseline_b_mm = src.baseline_b_mm();
    let baseline_c_mm = src.baseline_c_mm();
    let iar_num_hypotheses = src.iar_num_hypotheses();
}

fn write_GPS2_RTK(dst: &mut packs::GPS2_RTK::Data_) {
    dst.wn().set(some_i16);
    dst.time_last_baseline_ms().set(some_i32);
    dst.tow().set(some_i32);
    dst.accuracy().set(some_i32);
    dst.rtk_receiver_id().set(some_i8);
    dst.rtk_health().set(some_i8);
    dst.rtk_rate().set(some_i8);
    dst.nsats().set(some_i8);
    dst.baseline_coords_type().set(some_i8);
    dst.baseline_a_mm().set(some_i32);
    dst.baseline_b_mm().set(some_i32);
    dst.baseline_c_mm().set(some_i32);
    dst.iar_num_hypotheses().set(some_i32);
}

fn read_MAG_CAL_REPORT(src: &mut packs::MAG_CAL_REPORT::Data_) {
    let compass_id = src.compass_id();
    let cal_mask = src.cal_mask();
    let autosaved = src.autosaved();
    let fitness = src.fitness();
    let ofs_x = src.ofs_x();
    let ofs_y = src.ofs_y();
    let ofs_z = src.ofs_z();
    let diag_x = src.diag_x();
    let diag_y = src.diag_y();
    let diag_z = src.diag_z();
    let offdiag_x = src.offdiag_x();
    let offdiag_y = src.offdiag_y();
    let offdiag_z = src.offdiag_z();
    if let Some(cal_status) = src.cal_status().get() {}
}

fn write_MAG_CAL_REPORT(dst: &mut packs::MAG_CAL_REPORT::Data_) {
    dst.compass_id().set(some_i8);
    dst.cal_mask().set(some_i8);
    dst.autosaved().set(some_i8);
    dst.fitness().set(some_f32);
    dst.ofs_x().set(some_f32);
    dst.ofs_y().set(some_f32);
    dst.ofs_z().set(some_f32);
    dst.diag_x().set(some_f32);
    dst.diag_y().set(some_f32);
    dst.diag_z().set(some_f32);
    dst.offdiag_x().set(some_f32);
    dst.offdiag_y().set(some_f32);
    dst.offdiag_z().set(some_f32);
    dst.cal_status().set(packs::MAG_CAL_STATUS);
}

fn read_LOG_REQUEST_LIST(src: &mut packs::LOG_REQUEST_LIST::Data_) {
    let start = src.start();
    let end = src.end();
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn write_LOG_REQUEST_LIST(dst: &mut packs::LOG_REQUEST_LIST::Data_) {
    dst.start().set(some_i16);
    dst.end().set(some_i16);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
}

fn read_SCALED_PRESSURE(src: &mut packs::SCALED_PRESSURE::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let press_abs = src.press_abs();
    let press_diff = src.press_diff();
    let temperature = src.temperature();
}

fn read_V2_EXTENSION(src: &mut packs::V2_EXTENSION::Data_) {
    let message_type = src.message_type();
    let target_network = src.target_network();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let payload = src.payload(None);
}

fn write_V2_EXTENSION(dst: &mut packs::V2_EXTENSION::Data_) {
    dst.message_type().set(some_i16);
    dst.target_network().set(some_i8);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.payload(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_HEARTBEAT(src: &mut packs::HEARTBEAT::Data_) {
    let custom_mode = src.custom_mode();
    let mavlink_version = src.mavlink_version();
    if let Some(typE) = src.typE() {}
    if let Some(autopilot) = src.autopilot() {}
    if let Some(base_mode) = src.base_mode() {}
    if let Some(system_status) = src.system_status() {}
}

fn read_PARAM_MAP_RC(src: &mut packs::PARAM_MAP_RC::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let param_index = src.param_index();
    let parameter_rc_channel_index = src.parameter_rc_channel_index();
    let param_value0 = src.param_value0();
    let scale = src.scale();
    let param_value_min = src.param_value_min();
    let param_value_max = src.param_value_max();
    if let Some(src) = src.param_id() {}
}

fn read_POWER_STATUS(src: &mut packs::POWER_STATUS::Data_) {
    let Vcc = src.Vcc();
    let Vservo = src.Vservo();
    if let Some(flags) = src.flags().get() {}
}

fn write_POWER_STATUS(dst: &mut packs::POWER_STATUS::Data_) {
    dst.Vcc().set(some_i16);
    dst.Vservo().set(some_i16);
    dst.flags().set(packs::MAV_POWER_STATUS);
}

fn read_REMOTE_LOG_DATA_BLOCK(src: &mut packs::REMOTE_LOG_DATA_BLOCK::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let daTa = src.daTa(None);
    if let Some(seqno) = src.seqno().get() {}
}

fn write_REMOTE_LOG_DATA_BLOCK(dst: &mut packs::REMOTE_LOG_DATA_BLOCK::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.seqno().set(packs::MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS);
}

fn read_LOGGING_DATA_ACKED(src: &mut packs::LOGGING_DATA_ACKED::Data_) {
    let sequence = src.sequence();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let length = src.length();
    let first_message_offset = src.first_message_offset();
    let daTa = src.daTa(None);
}

fn write_LOGGING_DATA_ACKED(dst: &mut packs::LOGGING_DATA_ACKED::Data_) {
    dst.sequence().set(some_i16);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.length().set(some_i8);
    dst.first_message_offset().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_TERRAIN_CHECK(src: &mut packs::TERRAIN_CHECK::Data_) {
    let lat = src.lat();
    let lon = src.lon();
}

fn write_TERRAIN_CHECK(dst: &mut packs::TERRAIN_CHECK::Data_) {
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
}

fn read_MOUNT_CONFIGURE(src: &mut packs::MOUNT_CONFIGURE::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let stab_roll = src.stab_roll();
    let stab_pitch = src.stab_pitch();
    let stab_yaw = src.stab_yaw();
    if let Some(mount_mode) = src.mount_mode().get() {}
}

fn write_MOUNT_CONFIGURE(dst: &mut packs::MOUNT_CONFIGURE::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.stab_roll().set(some_i8);
    dst.stab_pitch().set(some_i8);
    dst.stab_yaw().set(some_i8);
    dst.mount_mode().set(packs::MAV_MOUNT_MODE);
}

fn read_MISSION_REQUEST_INT(src: &mut packs::MISSION_REQUEST_INT::Data_) {
    let seq = src.seq();
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(mission_type) = src.mission_type() {}
}

fn read_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(src: &mut packs::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
}

fn read_COMMAND_ACK(src: &mut packs::COMMAND_ACK::Data_) {
    if let Some(command) = src.command() {}
    if let Some(result) = src.result() {}
    if let Some(progress) = src.progress() {}
    if let Some(result_param2) = src.result_param2() {}
    if let Some(target_system) = src.target_system() {}
    if let Some(target_component) = src.target_component() {}
}

fn read_DATA_STREAM(src: &mut packs::DATA_STREAM::Data_) {
    let message_rate = src.message_rate();
    let stream_id = src.stream_id();
    let on_off = src.on_off();
}

fn read_MISSION_REQUEST(src: &mut packs::MISSION_REQUEST::Data_) {
    let seq = src.seq();
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(mission_type) = src.mission_type() {}
}

fn read_TERRAIN_REPORT(src: &mut packs::TERRAIN_REPORT::Data_) {
    let spacing = src.spacing();
    let pending = src.pending();
    let loaded = src.loaded();
    let lat = src.lat();
    let lon = src.lon();
    let terrain_height = src.terrain_height();
    let current_height = src.current_height();
}

fn write_TERRAIN_REPORT(dst: &mut packs::TERRAIN_REPORT::Data_) {
    dst.spacing().set(some_i16);
    dst.pending().set(some_i16);
    dst.loaded().set(some_i16);
    dst.lat().set(some_i32);
    dst.lon().set(some_i32);
    dst.terrain_height().set(some_f32);
    dst.current_height().set(some_f32);
}

fn read_SET_HOME_POSITION(src: &mut packs::SET_HOME_POSITION::Data_) {
    let target_system = src.target_system();
    let latitude = src.latitude();
    let longitude = src.longitude();
    let altitude = src.altitude();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let q = src.q(None);
    let approach_x = src.approach_x();
    let approach_y = src.approach_y();
    let approach_z = src.approach_z();
    if let Some(time_usec) = src.time_usec().get() {}
}

fn write_SET_HOME_POSITION(dst: &mut packs::SET_HOME_POSITION::Data_) {
    dst.target_system().set(some_i8);
    dst.latitude().set(some_i32);
    dst.longitude().set(some_i32);
    dst.altitude().set(some_i32);
    dst.x().set(some_f32);
    dst.y().set(some_f32);
    dst.z().set(some_f32);
    dst.q(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.approach_x().set(some_f32);
    dst.approach_y().set(some_f32);
    dst.approach_z().set(some_f32);
    dst.time_usec().set(some_i64);
}

fn read_HIL_RC_INPUTS_RAW(src: &mut packs::HIL_RC_INPUTS_RAW::Data_) {
    let chan1_raw = src.chan1_raw();
    let chan2_raw = src.chan2_raw();
    let chan3_raw = src.chan3_raw();
    let chan4_raw = src.chan4_raw();
    let chan5_raw = src.chan5_raw();
    let chan6_raw = src.chan6_raw();
    let chan7_raw = src.chan7_raw();
    let chan8_raw = src.chan8_raw();
    let chan9_raw = src.chan9_raw();
    let chan10_raw = src.chan10_raw();
    let chan11_raw = src.chan11_raw();
    let chan12_raw = src.chan12_raw();
    let time_usec = src.time_usec();
    let rssi = src.rssi();
}

fn read_SCALED_IMU3(src: &mut packs::SCALED_IMU3::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
    let xgyro = src.xgyro();
    let ygyro = src.ygyro();
    let zgyro = src.zgyro();
    let xmag = src.xmag();
    let ymag = src.ymag();
    let zmag = src.zmag();
}

fn write_SCALED_IMU3(dst: &mut packs::SCALED_IMU3::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.xacc().set(some_i16);
    dst.yacc().set(some_i16);
    dst.zacc().set(some_i16);
    dst.xgyro().set(some_i16);
    dst.ygyro().set(some_i16);
    dst.zgyro().set(some_i16);
    dst.xmag().set(some_i16);
    dst.ymag().set(some_i16);
    dst.zmag().set(some_i16);
}

fn read_SET_MODE(src: &mut packs::SET_MODE::Data_) {
    let custom_mode = src.custom_mode();
    let target_system = src.target_system();
    if let Some(base_mode) = src.base_mode() {}
}

fn read_MOUNT_CONTROL(src: &mut packs::MOUNT_CONTROL::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let input_a = src.input_a();
    let input_b = src.input_b();
    let input_c = src.input_c();
    let save_position = src.save_position();
}

fn write_MOUNT_CONTROL(dst: &mut packs::MOUNT_CONTROL::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.input_a().set(some_i32);
    dst.input_b().set(some_i32);
    dst.input_c().set(some_i32);
    dst.save_position().set(some_i8);
}

fn read_POSITION_TARGET_GLOBAL_INT(src: &mut packs::POSITION_TARGET_GLOBAL_INT::Data_) {
    let type_mask = src.type_mask();
    let time_boot_ms = src.time_boot_ms();
    let lat_int = src.lat_int();
    let lon_int = src.lon_int();
    let alt = src.alt();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let afx = src.afx();
    let afy = src.afy();
    let afz = src.afz();
    let yaw = src.yaw();
    let yaw_rate = src.yaw_rate();
    if let Some(coordinate_frame) = src.coordinate_frame() {}
}

fn read_LED_CONTROL(src: &mut packs::LED_CONTROL::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let instance = src.instance();
    let pattern = src.pattern();
    let custom_len = src.custom_len();
    let custom_bytes = src.custom_bytes(None);
}

fn write_LED_CONTROL(dst: &mut packs::LED_CONTROL::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.instance().set(some_i8);
    dst.pattern().set(some_i8);
    dst.custom_len().set(some_i8);
    dst.custom_bytes(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_SIM_STATE(src: &mut packs::SIM_STATE::Data_) {
    let q1 = src.q1();
    let q2 = src.q2();
    let q3 = src.q3();
    let q4 = src.q4();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
    let xgyro = src.xgyro();
    let ygyro = src.ygyro();
    let zgyro = src.zgyro();
    let lat = src.lat();
    let lon = src.lon();
    let alt = src.alt();
    let std_dev_horz = src.std_dev_horz();
    let std_dev_vert = src.std_dev_vert();
    let vn = src.vn();
    let ve = src.ve();
    let vd = src.vd();
}

fn write_SIM_STATE(dst: &mut packs::SIM_STATE::Data_) {
    dst.q1().set(some_f32);
    dst.q2().set(some_f32);
    dst.q3().set(some_f32);
    dst.q4().set(some_f32);
    dst.roll().set(some_f32);
    dst.pitch().set(some_f32);
    dst.yaw().set(some_f32);
    dst.xacc().set(some_f32);
    dst.yacc().set(some_f32);
    dst.zacc().set(some_f32);
    dst.xgyro().set(some_f32);
    dst.ygyro().set(some_f32);
    dst.zgyro().set(some_f32);
    dst.lat().set(some_f32);
    dst.lon().set(some_f32);
    dst.alt().set(some_f32);
    dst.std_dev_horz().set(some_f32);
    dst.std_dev_vert().set(some_f32);
    dst.vn().set(some_f32);
    dst.ve().set(some_f32);
    dst.vd().set(some_f32);
}

fn read_WIFI_CONFIG_AP(src: &mut packs::WIFI_CONFIG_AP::Data_) {
    if let Some(src) = src.ssid().get() {}
    if let Some(src) = src.password().get() {}
}

fn write_WIFI_CONFIG_AP(dst: &mut packs::WIFI_CONFIG_AP::Data_) {
    dst.ssid().set(some_str);
    dst.password().set(some_str);
}

fn read_DATA96(src: &mut packs::DATA96::Data_) {
    let typE = src.typE();
    let len = src.len();
    let daTa = src.daTa(None);
}

fn write_DATA96(dst: &mut packs::DATA96::Data_) {
    dst.typE().set(some_i8);
    dst.len().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_FLIGHT_INFORMATION(src: &mut packs::FLIGHT_INFORMATION::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let arming_time_utc = src.arming_time_utc();
    let takeoff_time_utc = src.takeoff_time_utc();
    let flight_uuid = src.flight_uuid();
}

fn write_FLIGHT_INFORMATION(dst: &mut packs::FLIGHT_INFORMATION::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.arming_time_utc().set(some_i64);
    dst.takeoff_time_utc().set(some_i64);
    dst.flight_uuid().set(some_i64);
}

fn read_RC_CHANNELS_RAW(src: &mut packs::RC_CHANNELS_RAW::Data_) {
    let chan1_raw = src.chan1_raw();
    let chan2_raw = src.chan2_raw();
    let chan3_raw = src.chan3_raw();
    let chan4_raw = src.chan4_raw();
    let chan5_raw = src.chan5_raw();
    let chan6_raw = src.chan6_raw();
    let chan7_raw = src.chan7_raw();
    let chan8_raw = src.chan8_raw();
    let time_boot_ms = src.time_boot_ms();
    let port = src.port();
    let rssi = src.rssi();
}

fn read_SERVO_OUTPUT_RAW(src: &mut packs::SERVO_OUTPUT_RAW::Data_) {
    let servo1_raw = src.servo1_raw();
    let servo2_raw = src.servo2_raw();
    let servo3_raw = src.servo3_raw();
    let servo4_raw = src.servo4_raw();
    let servo5_raw = src.servo5_raw();
    let servo6_raw = src.servo6_raw();
    let servo7_raw = src.servo7_raw();
    let servo8_raw = src.servo8_raw();
    let time_usec = src.time_usec();
    let port = src.port();
    if let Some(servo9_raw) = src.servo9_raw() {}
    if let Some(servo10_raw) = src.servo10_raw() {}
    if let Some(servo11_raw) = src.servo11_raw() {}
    if let Some(servo12_raw) = src.servo12_raw() {}
    if let Some(servo13_raw) = src.servo13_raw() {}
    if let Some(servo14_raw) = src.servo14_raw() {}
    if let Some(servo15_raw) = src.servo15_raw() {}
    if let Some(servo16_raw) = src.servo16_raw() {}
}

fn read_MEMINFO(src: &mut packs::MEMINFO::Data_) {
    let brkval = src.brkval();
    let freemem = src.freemem();
    if let Some(freemem32) = src.freemem32().get() {}
}

fn write_MEMINFO(dst: &mut packs::MEMINFO::Data_) {
    dst.brkval().set(some_i16);
    dst.freemem().set(some_i16);
    dst.freemem32().set(some_i32);
}

fn read_MISSION_ITEM_REACHED(src: &mut packs::MISSION_ITEM_REACHED::Data_) { let seq = src.seq(); }

fn read_LOGGING_ACK(src: &mut packs::LOGGING_ACK::Data_) {
    let sequence = src.sequence();
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn write_LOGGING_ACK(dst: &mut packs::LOGGING_ACK::Data_) {
    dst.sequence().set(some_i16);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
}

fn read_VISION_SPEED_ESTIMATE(src: &mut packs::VISION_SPEED_ESTIMATE::Data_) {
    let usec = src.usec();
    let x = src.x();
    let y = src.y();
    let z = src.z();
}

fn write_VISION_SPEED_ESTIMATE(dst: &mut packs::VISION_SPEED_ESTIMATE::Data_) {
    dst.usec().set(some_i64);
    dst.x().set(some_f32);
    dst.y().set(some_f32);
    dst.z().set(some_f32);
}

fn read_DEBUG_VECT(src: &mut packs::DEBUG_VECT::Data_) {
    let time_usec = src.time_usec();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    if let Some(src) = src.name().get() {}
}

fn write_DEBUG_VECT(dst: &mut packs::DEBUG_VECT::Data_) {
    dst.time_usec().set(some_i64);
    dst.x().set(some_f32);
    dst.y().set(some_f32);
    dst.z().set(some_f32);
    dst.name().set(some_str);
}

fn read_LOG_REQUEST_END(src: &mut packs::LOG_REQUEST_END::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn write_LOG_REQUEST_END(dst: &mut packs::LOG_REQUEST_END::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
}

fn read_MISSION_ACK(src: &mut packs::MISSION_ACK::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(typE) = src.typE() {}
    if let Some(mission_type) = src.mission_type() {}
}

fn read_CHANGE_OPERATOR_CONTROL_ACK(src: &mut packs::CHANGE_OPERATOR_CONTROL_ACK::Data_) {
    let gcs_system_id = src.gcs_system_id();
    let control_request = src.control_request();
    let ack = src.ack();
}

fn read_MISSION_CURRENT(src: &mut packs::MISSION_CURRENT::Data_) { let seq = src.seq(); }

fn read_SYSTEM_TIME(src: &mut packs::SYSTEM_TIME::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let time_unix_usec = src.time_unix_usec();
}

fn read_CAMERA_TRIGGER(src: &mut packs::CAMERA_TRIGGER::Data_) {
    let seq = src.seq();
    let time_usec = src.time_usec();
}

fn write_CAMERA_TRIGGER(dst: &mut packs::CAMERA_TRIGGER::Data_) {
    dst.seq().set(some_i32);
    dst.time_usec().set(some_i64);
}

fn read_GOPRO_SET_RESPONSE(src: &mut packs::GOPRO_SET_RESPONSE::Data_) {
    if let Some(cmd_id) = src.cmd_id().get() {}
    if let Some(status) = src.status().get() {}
}

fn write_GOPRO_SET_RESPONSE(dst: &mut packs::GOPRO_SET_RESPONSE::Data_) {
    dst.cmd_id().set(packs::GOPRO_COMMAND);
    dst.status().set(packs::GOPRO_REQUEST_STATUS);
}

fn read_VISION_POSITION_ESTIMATE(src: &mut packs::VISION_POSITION_ESTIMATE::Data_) {
    let usec = src.usec();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
}

fn read_MANUAL_CONTROL(src: &mut packs::MANUAL_CONTROL::Data_) {
    let buttons = src.buttons();
    let target = src.target();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let r = src.r();
}

fn read_RC_CHANNELS(src: &mut packs::RC_CHANNELS::Data_) {
    let chan1_raw = src.chan1_raw();
    let chan2_raw = src.chan2_raw();
    let chan3_raw = src.chan3_raw();
    let chan4_raw = src.chan4_raw();
    let chan5_raw = src.chan5_raw();
    let chan6_raw = src.chan6_raw();
    let chan7_raw = src.chan7_raw();
    let chan8_raw = src.chan8_raw();
    let chan9_raw = src.chan9_raw();
    let chan10_raw = src.chan10_raw();
    let chan11_raw = src.chan11_raw();
    let chan12_raw = src.chan12_raw();
    let chan13_raw = src.chan13_raw();
    let chan14_raw = src.chan14_raw();
    let chan15_raw = src.chan15_raw();
    let chan16_raw = src.chan16_raw();
    let chan17_raw = src.chan17_raw();
    let chan18_raw = src.chan18_raw();
    let time_boot_ms = src.time_boot_ms();
    let chancount = src.chancount();
    let rssi = src.rssi();
}

fn read_PROTOCOL_VERSION(src: &mut packs::PROTOCOL_VERSION::Data_) {
    let version = src.version();
    let min_version = src.min_version();
    let max_version = src.max_version();
    let spec_version_hash = src.spec_version_hash(None);
    let library_version_hash = src.library_version_hash(None);
}

fn write_PROTOCOL_VERSION(dst: &mut packs::PROTOCOL_VERSION::Data_) {
    dst.version().set(some_i16);
    dst.min_version().set(some_i16);
    dst.max_version().set(some_i16);
    dst.spec_version_hash(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.library_version_hash(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_RALLY_FETCH_POINT(src: &mut packs::RALLY_FETCH_POINT::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let idx = src.idx();
}

fn write_RALLY_FETCH_POINT(dst: &mut packs::RALLY_FETCH_POINT::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.idx().set(some_i8);
}

fn read_PARAM_VALUE(src: &mut packs::PARAM_VALUE::Data_) {
    let param_count = src.param_count();
    let param_index = src.param_index();
    let param_value = src.param_value();
    if let Some(src) = src.param_id() {}
    if let Some(param_type) = src.param_type() {}
}

fn read_BATTERY_STATUS(src: &mut packs::BATTERY_STATUS::Data_) {
    let voltages = src.voltages(None);
    let id = src.id();
    let temperature = src.temperature();
    let current_battery = src.current_battery();
    let current_consumed = src.current_consumed();
    let energy_consumed = src.energy_consumed();
    let battery_remaining = src.battery_remaining();
    if let Some(battery_function) = src.battery_function().get() {}
    if let Some(typE) = src.typE().get() {}
}

fn write_BATTERY_STATUS(dst: &mut packs::BATTERY_STATUS::Data_) {
    dst.voltages(Some(&mut sys::ByValIter::new(&some_i16s)));
    dst.id().set(some_i8);
    dst.temperature().set(some_i16);
    dst.current_battery().set(some_i16);
    dst.current_consumed().set(some_i32);
    dst.energy_consumed().set(some_i32);
    dst.battery_remaining().set(some_i8);
    dst.battery_function().set(packs::MAV_BATTERY_FUNCTION);
    dst.typE().set(packs::MAV_BATTERY_TYPE);
}

fn read_SERIAL_CONTROL(src: &mut packs::SERIAL_CONTROL::Data_) {
    let timeout = src.timeout();
    let baudrate = src.baudrate();
    let count = src.count();
    let daTa = src.daTa(None);
    if let Some(device) = src.device().get() {}
    if let Some(flags) = src.flags().get() {}
}

fn write_SERIAL_CONTROL(dst: &mut packs::SERIAL_CONTROL::Data_) {
    dst.timeout().set(some_i16);
    dst.baudrate().set(some_i32);
    dst.count().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.device().set(packs::SERIAL_CONTROL_DEV);
    dst.flags().set(packs::SERIAL_CONTROL_FLAG);
}

fn read_SET_POSITION_TARGET_LOCAL_NED(src: &mut packs::SET_POSITION_TARGET_LOCAL_NED::Data_) {
    let type_mask = src.type_mask();
    let time_boot_ms = src.time_boot_ms();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let afx = src.afx();
    let afy = src.afy();
    let afz = src.afz();
    let yaw = src.yaw();
    let yaw_rate = src.yaw_rate();
    if let Some(coordinate_frame) = src.coordinate_frame() {}
}

fn read_MOUNT_ORIENTATION(src: &mut packs::MOUNT_ORIENTATION::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
}

fn write_MOUNT_ORIENTATION(dst: &mut packs::MOUNT_ORIENTATION::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.roll().set(some_f32);
    dst.pitch().set(some_f32);
    dst.yaw().set(some_f32);
}

fn read_SET_GPS_GLOBAL_ORIGIN(src: &mut packs::SET_GPS_GLOBAL_ORIGIN::Data_) {
    let target_system = src.target_system();
    let latitude = src.latitude();
    let longitude = src.longitude();
    let altitude = src.altitude();
    if let Some(time_usec) = src.time_usec() {}
}

fn read_PARAM_EXT_SET(src: &mut packs::PARAM_EXT_SET::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(src) = src.param_id().get() {}
    if let Some(src) = src.param_value().get() {}
    if let Some(param_type) = src.param_type().get() {}
}

fn write_PARAM_EXT_SET(dst: &mut packs::PARAM_EXT_SET::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.param_id().set(some_str);
    dst.param_value().set(some_str);
    dst.param_type().set(packs::MAV_PARAM_EXT_TYPE);
}

fn read_AUTOPILOT_VERSION(src: &mut packs::AUTOPILOT_VERSION::Data_) {
    let vendor_id = src.vendor_id();
    let product_id = src.product_id();
    let flight_sw_version = src.flight_sw_version();
    let middleware_sw_version = src.middleware_sw_version();
    let os_sw_version = src.os_sw_version();
    let board_version = src.board_version();
    let uid = src.uid();
    let flight_custom_version = src.flight_custom_version(None);
    let middleware_custom_version = src.middleware_custom_version(None);
    let os_custom_version = src.os_custom_version(None);
    if let Some(capabilities) = src.capabilities().get() {}
    if let Some(mut src_uid2) = src.uid2().items() {
        src_uid2.enumerate(0, |src, d0| if let Some(uid2) = src.get(d0) {});
    }
}

fn write_AUTOPILOT_VERSION(dst: &mut packs::AUTOPILOT_VERSION::Data_) {
    dst.vendor_id().set(some_i16);
    dst.product_id().set(some_i16);
    dst.flight_sw_version().set(some_i32);
    dst.middleware_sw_version().set(some_i32);
    dst.os_sw_version().set(some_i32);
    dst.board_version().set(some_i32);
    dst.uid().set(some_i64);
    dst.flight_custom_version(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.middleware_custom_version(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.os_custom_version(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.capabilities().set(packs::MAV_PROTOCOL_CAPABILITY);
    let mut dst_uid2 = dst.uid2();

    dst_uid2.enumerate(0, |dst, d0| {
        dst.set(some_i8, d0);
    });
}

fn read_MISSION_REQUEST_LIST(src: &mut packs::MISSION_REQUEST_LIST::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(mission_type) = src.mission_type() {}
}

fn read_SIMSTATE(src: &mut packs::SIMSTATE::Data_) {
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
    let xgyro = src.xgyro();
    let ygyro = src.ygyro();
    let zgyro = src.zgyro();
    let lat = src.lat();
    let lng = src.lng();
}

fn write_SIMSTATE(dst: &mut packs::SIMSTATE::Data_) {
    dst.roll().set(some_f32);
    dst.pitch().set(some_f32);
    dst.yaw().set(some_f32);
    dst.xacc().set(some_f32);
    dst.yacc().set(some_f32);
    dst.zacc().set(some_f32);
    dst.xgyro().set(some_f32);
    dst.ygyro().set(some_f32);
    dst.zgyro().set(some_f32);
    dst.lat().set(some_i32);
    dst.lng().set(some_i32);
}

fn read_SET_VIDEO_STREAM_SETTINGS(src: &mut packs::SET_VIDEO_STREAM_SETTINGS::Data_) {
    let resolution_h = src.resolution_h();
    let resolution_v = src.resolution_v();
    let rotation = src.rotation();
    let bitrate = src.bitrate();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let camera_id = src.camera_id();
    let framerate = src.framerate();
    if let Some(src) = src.uri().get() {}
}

fn write_SET_VIDEO_STREAM_SETTINGS(dst: &mut packs::SET_VIDEO_STREAM_SETTINGS::Data_) {
    dst.resolution_h().set(some_i16);
    dst.resolution_v().set(some_i16);
    dst.rotation().set(some_i16);
    dst.bitrate().set(some_i32);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.camera_id().set(some_i8);
    dst.framerate().set(some_f32);
    dst.uri().set(some_str);
}

fn read_PLAY_TUNE(src: &mut packs::PLAY_TUNE::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    if let Some(src) = src.tune().get() {}
}

fn write_PLAY_TUNE(dst: &mut packs::PLAY_TUNE::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.tune().set(some_str);
}

fn read_DIGICAM_CONFIGURE(src: &mut packs::DIGICAM_CONFIGURE::Data_) {
    let shutter_speed = src.shutter_speed();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let mode = src.mode();
    let aperture = src.aperture();
    let iso = src.iso();
    let exposure_type = src.exposure_type();
    let command_id = src.command_id();
    let engine_cut_off = src.engine_cut_off();
    let extra_param = src.extra_param();
    let extra_value = src.extra_value();
}

fn write_DIGICAM_CONFIGURE(dst: &mut packs::DIGICAM_CONFIGURE::Data_) {
    dst.shutter_speed().set(some_i16);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.mode().set(some_i8);
    dst.aperture().set(some_i8);
    dst.iso().set(some_i8);
    dst.exposure_type().set(some_i8);
    dst.command_id().set(some_i8);
    dst.engine_cut_off().set(some_i8);
    dst.extra_param().set(some_i8);
    dst.extra_value().set(some_f32);
}

fn read_SCALED_PRESSURE3(src: &mut packs::SCALED_PRESSURE3::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let press_abs = src.press_abs();
    let press_diff = src.press_diff();
    let temperature = src.temperature();
}

fn write_SCALED_PRESSURE3(dst: &mut packs::SCALED_PRESSURE3::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.press_abs().set(some_f32);
    dst.press_diff().set(some_f32);
    dst.temperature().set(some_i16);
}

fn read_MISSION_REQUEST_PARTIAL_LIST(src: &mut packs::MISSION_REQUEST_PARTIAL_LIST::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let start_index = src.start_index();
    let end_index = src.end_index();
    if let Some(mission_type) = src.mission_type() {}
}

fn read_PARAM_EXT_ACK(src: &mut packs::PARAM_EXT_ACK::Data_) {
    if let Some(src) = src.param_id().get() {}
    if let Some(src) = src.param_value().get() {}
    if let Some(param_type) = src.param_type().get() {}
    if let Some(param_result) = src.param_result().get() {}
}

fn write_PARAM_EXT_ACK(dst: &mut packs::PARAM_EXT_ACK::Data_) {
    dst.param_id().set(some_str);
    dst.param_value().set(some_str);
    dst.param_type().set(packs::MAV_PARAM_EXT_TYPE);
    dst.param_result().set(packs::PARAM_ACK);
}

fn read_UAVCAN_NODE_INFO(src: &mut packs::UAVCAN_NODE_INFO::Data_) {
    let uptime_sec = src.uptime_sec();
    let sw_vcs_commit = src.sw_vcs_commit();
    let time_usec = src.time_usec();
    let hw_version_major = src.hw_version_major();
    let hw_version_minor = src.hw_version_minor();
    let hw_unique_id = src.hw_unique_id(None);
    let sw_version_major = src.sw_version_major();
    let sw_version_minor = src.sw_version_minor();
    if let Some(src) = src.name().get() {}
}

fn write_UAVCAN_NODE_INFO(dst: &mut packs::UAVCAN_NODE_INFO::Data_) {
    dst.uptime_sec().set(some_i32);
    dst.sw_vcs_commit().set(some_i32);
    dst.time_usec().set(some_i64);
    dst.hw_version_major().set(some_i8);
    dst.hw_version_minor().set(some_i8);
    dst.hw_unique_id(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.sw_version_major().set(some_i8);
    dst.sw_version_minor().set(some_i8);
    dst.name().set(some_str);
}

fn read_DATA16(src: &mut packs::DATA16::Data_) {
    let typE = src.typE();
    let len = src.len();
    let daTa = src.daTa(None);
}

fn write_DATA16(dst: &mut packs::DATA16::Data_) {
    dst.typE().set(some_i8);
    dst.len().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_SET_MAG_OFFSETS(src: &mut packs::SET_MAG_OFFSETS::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let mag_ofs_x = src.mag_ofs_x();
    let mag_ofs_y = src.mag_ofs_y();
    let mag_ofs_z = src.mag_ofs_z();
}

fn write_SET_MAG_OFFSETS(dst: &mut packs::SET_MAG_OFFSETS::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.mag_ofs_x().set(some_i16);
    dst.mag_ofs_y().set(some_i16);
    dst.mag_ofs_z().set(some_i16);
}

fn read_AP_ADC(src: &mut packs::AP_ADC::Data_) {
    let adc1 = src.adc1();
    let adc2 = src.adc2();
    let adc3 = src.adc3();
    let adc4 = src.adc4();
    let adc5 = src.adc5();
    let adc6 = src.adc6();
}

fn write_AP_ADC(dst: &mut packs::AP_ADC::Data_) {
    dst.adc1().set(some_i16);
    dst.adc2().set(some_i16);
    dst.adc3().set(some_i16);
    dst.adc4().set(some_i16);
    dst.adc5().set(some_i16);
    dst.adc6().set(some_i16);
}

fn read_WIND(src: &mut packs::WIND::Data_) {
    let direction = src.direction();
    let speed = src.speed();
    let speed_z = src.speed_z();
}

fn write_WIND(dst: &mut packs::WIND::Data_) {
    dst.direction().set(some_f32);
    dst.speed().set(some_f32);
    dst.speed_z().set(some_f32);
}

fn read_AUTOPILOT_VERSION_REQUEST(src: &mut packs::AUTOPILOT_VERSION_REQUEST::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
}

fn write_AUTOPILOT_VERSION_REQUEST(dst: &mut packs::AUTOPILOT_VERSION_REQUEST::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
}

fn read_LOCAL_POSITION_NED(src: &mut packs::LOCAL_POSITION_NED::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
}

fn read_DATA_TRANSMISSION_HANDSHAKE(src: &mut packs::DATA_TRANSMISSION_HANDSHAKE::Data_) {
    let width = src.width();
    let height = src.height();
    let packets = src.packets();
    let size = src.size();
    let typE = src.typE();
    let payload = src.payload();
    let jpg_quality = src.jpg_quality();
}

fn write_DATA_TRANSMISSION_HANDSHAKE(dst: &mut packs::DATA_TRANSMISSION_HANDSHAKE::Data_) {
    dst.width().set(some_i16);
    dst.height().set(some_i16);
    dst.packets().set(some_i16);
    dst.size().set(some_i32);
    dst.typE().set(some_i8);
    dst.payload().set(some_i8);
    dst.jpg_quality().set(some_i8);
}

fn read_GPS_GLOBAL_ORIGIN(src: &mut packs::GPS_GLOBAL_ORIGIN::Data_) {
    let latitude = src.latitude();
    let longitude = src.longitude();
    let altitude = src.altitude();
    if let Some(time_usec) = src.time_usec() {}
}

fn read_SCALED_IMU2(src: &mut packs::SCALED_IMU2::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
    let xgyro = src.xgyro();
    let ygyro = src.ygyro();
    let zgyro = src.zgyro();
    let xmag = src.xmag();
    let ymag = src.ymag();
    let zmag = src.zmag();
}

fn write_SCALED_IMU2(dst: &mut packs::SCALED_IMU2::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.xacc().set(some_i16);
    dst.yacc().set(some_i16);
    dst.zacc().set(some_i16);
    dst.xgyro().set(some_i16);
    dst.ygyro().set(some_i16);
    dst.zgyro().set(some_i16);
    dst.xmag().set(some_i16);
    dst.ymag().set(some_i16);
    dst.zmag().set(some_i16);
}

fn read_ATTITUDE_QUATERNION(src: &mut packs::ATTITUDE_QUATERNION::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let q1 = src.q1();
    let q2 = src.q2();
    let q3 = src.q3();
    let q4 = src.q4();
    let rollspeed = src.rollspeed();
    let pitchspeed = src.pitchspeed();
    let yawspeed = src.yawspeed();
}

fn read_DATA64(src: &mut packs::DATA64::Data_) {
    let typE = src.typE();
    let len = src.len();
    let daTa = src.daTa(None);
}

fn write_DATA64(dst: &mut packs::DATA64::Data_) {
    dst.typE().set(some_i8);
    dst.len().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_HIL_ACTUATOR_CONTROLS(src: &mut packs::HIL_ACTUATOR_CONTROLS::Data_) {
    let time_usec = src.time_usec();
    let flags = src.flags();
    let controls = src.controls();
    if let Some(mode) = src.mode() {}
}

fn read_POSITION_TARGET_LOCAL_NED(src: &mut packs::POSITION_TARGET_LOCAL_NED::Data_) {
    let type_mask = src.type_mask();
    let time_boot_ms = src.time_boot_ms();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let vx = src.vx();
    let vy = src.vy();
    let vz = src.vz();
    let afx = src.afx();
    let afy = src.afy();
    let afz = src.afz();
    let yaw = src.yaw();
    let yaw_rate = src.yaw_rate();
    if let Some(coordinate_frame) = src.coordinate_frame() {}
}

fn read_GIMBAL_REPORT(src: &mut packs::GIMBAL_REPORT::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let delta_time = src.delta_time();
    let delta_angle_x = src.delta_angle_x();
    let delta_angle_y = src.delta_angle_y();
    let delta_angle_z = src.delta_angle_z();
    let delta_velocity_x = src.delta_velocity_x();
    let delta_velocity_y = src.delta_velocity_y();
    let delta_velocity_z = src.delta_velocity_z();
    let joint_roll = src.joint_roll();
    let joint_el = src.joint_el();
    let joint_az = src.joint_az();
}

fn write_GIMBAL_REPORT(dst: &mut packs::GIMBAL_REPORT::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.delta_time().set(some_f32);
    dst.delta_angle_x().set(some_f32);
    dst.delta_angle_y().set(some_f32);
    dst.delta_angle_z().set(some_f32);
    dst.delta_velocity_x().set(some_f32);
    dst.delta_velocity_y().set(some_f32);
    dst.delta_velocity_z().set(some_f32);
    dst.joint_roll().set(some_f32);
    dst.joint_el().set(some_f32);
    dst.joint_az().set(some_f32);
}

fn read_DEVICE_OP_WRITE(src: &mut packs::DEVICE_OP_WRITE::Data_) {
    let request_id = src.request_id();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let bus = src.bus();
    let address = src.address();
    let regstart = src.regstart();
    let count = src.count();
    let daTa = src.daTa(None);
    if let Some(bustype) = src.bustype().get() {}
    if let Some(src) = src.busname().get() {}
}

fn write_DEVICE_OP_WRITE(dst: &mut packs::DEVICE_OP_WRITE::Data_) {
    dst.request_id().set(some_i32);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.bus().set(some_i8);
    dst.address().set(some_i8);
    dst.regstart().set(some_i8);
    dst.count().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.bustype().set(packs::DEVICE_OP_BUSTYPE);
    dst.busname().set(some_str);
}

fn read_DISTANCE_SENSOR(src: &mut packs::DISTANCE_SENSOR::Data_) {
    let min_distance = src.min_distance();
    let max_distance = src.max_distance();
    let current_distance = src.current_distance();
    let time_boot_ms = src.time_boot_ms();
    let id = src.id();
    let covariance = src.covariance();
    if let Some(typE) = src.typE().get() {}
    if let Some(orientation) = src.orientation().get() {}
}

fn write_DISTANCE_SENSOR(dst: &mut packs::DISTANCE_SENSOR::Data_) {
    dst.min_distance().set(some_i16);
    dst.max_distance().set(some_i16);
    dst.current_distance().set(some_i16);
    dst.time_boot_ms().set(some_i32);
    dst.id().set(some_i8);
    dst.covariance().set(some_i8);
    dst.typE().set(packs::MAV_DISTANCE_SENSOR);
    dst.orientation().set(packs::MAV_SENSOR_ORIENTATION);
}

fn read_HIL_OPTICAL_FLOW(src: &mut packs::HIL_OPTICAL_FLOW::Data_) {
    let integration_time_us = src.integration_time_us();
    let time_delta_distance_us = src.time_delta_distance_us();
    let time_usec = src.time_usec();
    let sensor_id = src.sensor_id();
    let integrated_x = src.integrated_x();
    let integrated_y = src.integrated_y();
    let integrated_xgyro = src.integrated_xgyro();
    let integrated_ygyro = src.integrated_ygyro();
    let integrated_zgyro = src.integrated_zgyro();
    let temperature = src.temperature();
    let quality = src.quality();
    let distance = src.distance();
}

fn write_HIL_OPTICAL_FLOW(dst: &mut packs::HIL_OPTICAL_FLOW::Data_) {
    dst.integration_time_us().set(some_i32);
    dst.time_delta_distance_us().set(some_i32);
    dst.time_usec().set(some_i64);
    dst.sensor_id().set(some_i8);
    dst.integrated_x().set(some_f32);
    dst.integrated_y().set(some_f32);
    dst.integrated_xgyro().set(some_f32);
    dst.integrated_ygyro().set(some_f32);
    dst.integrated_zgyro().set(some_f32);
    dst.temperature().set(some_i16);
    dst.quality().set(some_i8);
    dst.distance().set(some_f32);
}

fn read_SCALED_PRESSURE2(src: &mut packs::SCALED_PRESSURE2::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let press_abs = src.press_abs();
    let press_diff = src.press_diff();
    let temperature = src.temperature();
}

fn write_SCALED_PRESSURE2(dst: &mut packs::SCALED_PRESSURE2::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.press_abs().set(some_f32);
    dst.press_diff().set(some_f32);
    dst.temperature().set(some_i16);
}

fn read_WIND_COV(src: &mut packs::WIND_COV::Data_) {
    let time_usec = src.time_usec();
    let wind_x = src.wind_x();
    let wind_y = src.wind_y();
    let wind_z = src.wind_z();
    let var_horiz = src.var_horiz();
    let var_vert = src.var_vert();
    let wind_alt = src.wind_alt();
    let horiz_accuracy = src.horiz_accuracy();
    let vert_accuracy = src.vert_accuracy();
}

fn write_WIND_COV(dst: &mut packs::WIND_COV::Data_) {
    dst.time_usec().set(some_i64);
    dst.wind_x().set(some_f32);
    dst.wind_y().set(some_f32);
    dst.wind_z().set(some_f32);
    dst.var_horiz().set(some_f32);
    dst.var_vert().set(some_f32);
    dst.wind_alt().set(some_f32);
    dst.horiz_accuracy().set(some_f32);
    dst.vert_accuracy().set(some_f32);
}

fn read_CHANGE_OPERATOR_CONTROL(src: &mut packs::CHANGE_OPERATOR_CONTROL::Data_) {
    let target_system = src.target_system();
    let control_request = src.control_request();
    let version = src.version();
    if let Some(src) = src.passkey() {}
}

fn read_GOPRO_SET_REQUEST(src: &mut packs::GOPRO_SET_REQUEST::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let value = src.value(None);
    if let Some(cmd_id) = src.cmd_id().get() {}
}

fn write_GOPRO_SET_REQUEST(dst: &mut packs::GOPRO_SET_REQUEST::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.value(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.cmd_id().set(packs::GOPRO_COMMAND);
}

fn read_SYS_STATUS(src: &mut packs::SYS_STATUS::Data_) {
    let load = src.load();
    let voltage_battery = src.voltage_battery();
    let drop_rate_comm = src.drop_rate_comm();
    let errors_comm = src.errors_comm();
    let errors_count1 = src.errors_count1();
    let errors_count2 = src.errors_count2();
    let errors_count3 = src.errors_count3();
    let errors_count4 = src.errors_count4();
    let current_battery = src.current_battery();
    let battery_remaining = src.battery_remaining();
    if let Some(onboard_control_sensors_present) = src.onboard_control_sensors_present() {}
    if let Some(onboard_control_sensors_enabled) = src.onboard_control_sensors_enabled() {}
    if let Some(onboard_control_sensors_health) = src.onboard_control_sensors_health() {}
}

fn read_MISSION_ITEM(src: &mut packs::MISSION_ITEM::Data_) {
    let seq = src.seq();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let current = src.current();
    let autocontinue = src.autocontinue();
    let param1 = src.param1();
    let param2 = src.param2();
    let param3 = src.param3();
    let param4 = src.param4();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    if let Some(frame) = src.frame() {}
    if let Some(command) = src.command() {}
    if let Some(mission_type) = src.mission_type() {}
}

fn read_RAW_IMU(src: &mut packs::RAW_IMU::Data_) {
    let time_usec = src.time_usec();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
    let xgyro = src.xgyro();
    let ygyro = src.ygyro();
    let zgyro = src.zgyro();
    let xmag = src.xmag();
    let ymag = src.ymag();
    let zmag = src.zmag();
}

fn read_COMMAND_INT(src: &mut packs::COMMAND_INT::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let current = src.current();
    let autocontinue = src.autocontinue();
    let param1 = src.param1();
    let param2 = src.param2();
    let param3 = src.param3();
    let param4 = src.param4();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    if let Some(frame) = src.frame() {}
    if let Some(command) = src.command() {}
}

fn read_OPTICAL_FLOW(src: &mut packs::OPTICAL_FLOW::Data_) {
    let time_usec = src.time_usec();
    let sensor_id = src.sensor_id();
    let flow_x = src.flow_x();
    let flow_y = src.flow_y();
    let flow_comp_m_x = src.flow_comp_m_x();
    let flow_comp_m_y = src.flow_comp_m_y();
    let quality = src.quality();
    let ground_distance = src.ground_distance();
    if let Some(flow_rate_x) = src.flow_rate_x() {}
    if let Some(flow_rate_y) = src.flow_rate_y() {}
}

fn read_MISSION_ITEM_INT(src: &mut packs::MISSION_ITEM_INT::Data_) {
    let seq = src.seq();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let current = src.current();
    let autocontinue = src.autocontinue();
    let param1 = src.param1();
    let param2 = src.param2();
    let param3 = src.param3();
    let param4 = src.param4();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    if let Some(frame) = src.frame() {}
    if let Some(command) = src.command() {}
    if let Some(mission_type) = src.mission_type() {}
}

fn read_VISION_POSITION_DELTA(src: &mut packs::VISION_POSITION_DELTA::Data_) {
    let time_usec = src.time_usec();
    let time_delta_usec = src.time_delta_usec();
    let angle_delta = src.angle_delta(None);
    let position_delta = src.position_delta(None);
    let confidence = src.confidence();
}

fn write_VISION_POSITION_DELTA(dst: &mut packs::VISION_POSITION_DELTA::Data_) {
    dst.time_usec().set(some_i64);
    dst.time_delta_usec().set(some_i64);
    dst.angle_delta(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.position_delta(Some(&mut sys::ByValIter::new(&some_f32s)));
    dst.confidence().set(some_f32);
}

fn read_LOGGING_DATA(src: &mut packs::LOGGING_DATA::Data_) {
    let sequence = src.sequence();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let length = src.length();
    let first_message_offset = src.first_message_offset();
    let daTa = src.daTa(None);
}

fn write_LOGGING_DATA(dst: &mut packs::LOGGING_DATA::Data_) {
    dst.sequence().set(some_i16);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.length().set(some_i8);
    dst.first_message_offset().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_DEVICE_OP_READ(src: &mut packs::DEVICE_OP_READ::Data_) {
    let request_id = src.request_id();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let bus = src.bus();
    let address = src.address();
    let regstart = src.regstart();
    let count = src.count();
    if let Some(bustype) = src.bustype().get() {}
    if let Some(src) = src.busname().get() {}
}

fn write_DEVICE_OP_READ(dst: &mut packs::DEVICE_OP_READ::Data_) {
    dst.request_id().set(some_i32);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.bus().set(some_i8);
    dst.address().set(some_i8);
    dst.regstart().set(some_i8);
    dst.count().set(some_i8);
    dst.bustype().set(packs::DEVICE_OP_BUSTYPE);
    dst.busname().set(some_str);
}

fn read_MAG_CAL_PROGRESS(src: &mut packs::MAG_CAL_PROGRESS::Data_) {
    let compass_id = src.compass_id();
    let cal_mask = src.cal_mask();
    let attempt = src.attempt();
    let completion_pct = src.completion_pct();
    let completion_mask = src.completion_mask(None);
    let direction_x = src.direction_x();
    let direction_y = src.direction_y();
    let direction_z = src.direction_z();
    if let Some(cal_status) = src.cal_status().get() {}
}

fn write_MAG_CAL_PROGRESS(dst: &mut packs::MAG_CAL_PROGRESS::Data_) {
    dst.compass_id().set(some_i8);
    dst.cal_mask().set(some_i8);
    dst.attempt().set(some_i8);
    dst.completion_pct().set(some_i8);
    dst.completion_mask(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.direction_x().set(some_f32);
    dst.direction_y().set(some_f32);
    dst.direction_z().set(some_f32);
    dst.cal_status().set(packs::MAG_CAL_STATUS);
}

fn read_HIGHRES_IMU(src: &mut packs::HIGHRES_IMU::Data_) {
    let fields_updated = src.fields_updated();
    let time_usec = src.time_usec();
    let xacc = src.xacc();
    let yacc = src.yacc();
    let zacc = src.zacc();
    let xgyro = src.xgyro();
    let ygyro = src.ygyro();
    let zgyro = src.zgyro();
    let xmag = src.xmag();
    let ymag = src.ymag();
    let zmag = src.zmag();
    let abs_pressure = src.abs_pressure();
    let diff_pressure = src.diff_pressure();
    let pressure_alt = src.pressure_alt();
    let temperature = src.temperature();
}

fn write_HIGHRES_IMU(dst: &mut packs::HIGHRES_IMU::Data_) {
    dst.fields_updated().set(some_i16);
    dst.time_usec().set(some_i64);
    dst.xacc().set(some_f32);
    dst.yacc().set(some_f32);
    dst.zacc().set(some_f32);
    dst.xgyro().set(some_f32);
    dst.ygyro().set(some_f32);
    dst.zgyro().set(some_f32);
    dst.xmag().set(some_f32);
    dst.ymag().set(some_f32);
    dst.zmag().set(some_f32);
    dst.abs_pressure().set(some_f32);
    dst.diff_pressure().set(some_f32);
    dst.pressure_alt().set(some_f32);
    dst.temperature().set(some_f32);
}

fn read_EXTENDED_SYS_STATE(src: &mut packs::EXTENDED_SYS_STATE::Data_) {
    if let Some(vtol_state) = src.vtol_state().get() {}
    if let Some(landed_state) = src.landed_state().get() {}
}

fn write_EXTENDED_SYS_STATE(dst: &mut packs::EXTENDED_SYS_STATE::Data_) {
    dst.vtol_state().set(packs::MAV_VTOL_STATE);
    dst.landed_state().set(packs::MAV_LANDED_STATE);
}

fn read_UAVIONIX_ADSB_OUT_DYNAMIC(src: &mut packs::UAVIONIX_ADSB_OUT_DYNAMIC::Data_) {
    let accuracyVert = src.accuracyVert();
    let accuracyVel = src.accuracyVel();
    let squawk = src.squawk();
    let utcTime = src.utcTime();
    let accuracyHor = src.accuracyHor();
    let gpsLat = src.gpsLat();
    let gpsLon = src.gpsLon();
    let gpsAlt = src.gpsAlt();
    let numSats = src.numSats();
    let baroAltMSL = src.baroAltMSL();
    let velVert = src.velVert();
    let velNS = src.velNS();
    let VelEW = src.VelEW();
    if let Some(gpsFix) = src.gpsFix().get() {}
    if let Some(emergencyStatus) = src.emergencyStatus().get() {}
    if let Some(state) = src.state().get() {}
}

fn write_UAVIONIX_ADSB_OUT_DYNAMIC(dst: &mut packs::UAVIONIX_ADSB_OUT_DYNAMIC::Data_) {
    dst.accuracyVert().set(some_i16);
    dst.accuracyVel().set(some_i16);
    dst.squawk().set(some_i16);
    dst.utcTime().set(some_i32);
    dst.accuracyHor().set(some_i32);
    dst.gpsLat().set(some_i32);
    dst.gpsLon().set(some_i32);
    dst.gpsAlt().set(some_i32);
    dst.numSats().set(some_i8);
    dst.baroAltMSL().set(some_i32);
    dst.velVert().set(some_i16);
    dst.velNS().set(some_i16);
    dst.VelEW().set(some_i16);
    dst.gpsFix().set(packs::UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX);
    dst.emergencyStatus().set(packs::UAVIONIX_ADSB_EMERGENCY_STATUS);
    dst.state().set(packs::UAVIONIX_ADSB_OUT_DYNAMIC_STATE);
}

fn read_GOPRO_GET_RESPONSE(src: &mut packs::GOPRO_GET_RESPONSE::Data_) {
    let value = src.value(None);
    if let Some(cmd_id) = src.cmd_id().get() {}
    if let Some(status) = src.status().get() {}
}

fn write_GOPRO_GET_RESPONSE(dst: &mut packs::GOPRO_GET_RESPONSE::Data_) {
    dst.value(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.cmd_id().set(packs::GOPRO_COMMAND);
    dst.status().set(packs::GOPRO_REQUEST_STATUS);
}

fn read_GPS_INJECT_DATA(src: &mut packs::GPS_INJECT_DATA::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let len = src.len();
    let daTa = src.daTa(None);
}

fn write_GPS_INJECT_DATA(dst: &mut packs::GPS_INJECT_DATA::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.len().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(src: &mut packs::UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT::Data_) { if let Some(rfHealth) = src.rfHealth().get() {} }

fn write_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(dst: &mut packs::UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT::Data_) { dst.rfHealth().set(packs::UAVIONIX_ADSB_RF_HEALTH); }

fn read_ATTITUDE_QUATERNION_COV(src: &mut packs::ATTITUDE_QUATERNION_COV::Data_) {
    let time_usec = src.time_usec();
    let q = src.q();
    let rollspeed = src.rollspeed();
    let pitchspeed = src.pitchspeed();
    let yawspeed = src.yawspeed();
    let covariance = src.covariance();
}

fn read_NAMED_VALUE_INT(src: &mut packs::NAMED_VALUE_INT::Data_) {
    let time_boot_ms = src.time_boot_ms();
    let value = src.value();
    if let Some(src) = src.name().get() {}
}

fn write_NAMED_VALUE_INT(dst: &mut packs::NAMED_VALUE_INT::Data_) {
    dst.time_boot_ms().set(some_i32);
    dst.value().set(some_i32);
    dst.name().set(some_str);
}

fn read_RPM(src: &mut packs::RPM::Data_) {
    let rpm1 = src.rpm1();
    let rpm2 = src.rpm2();
}

fn write_RPM(dst: &mut packs::RPM::Data_) {
    dst.rpm1().set(some_f32);
    dst.rpm2().set(some_f32);
}

fn read_GPS_RTCM_DATA(src: &mut packs::GPS_RTCM_DATA::Data_) {
    let flags = src.flags();
    let len = src.len();
    let daTa = src.daTa(None);
}

fn write_GPS_RTCM_DATA(dst: &mut packs::GPS_RTCM_DATA::Data_) {
    dst.flags().set(some_i8);
    dst.len().set(some_i8);
    dst.daTa(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_GLOBAL_VISION_POSITION_ESTIMATE(src: &mut packs::GLOBAL_VISION_POSITION_ESTIMATE::Data_) {
    let usec = src.usec();
    let x = src.x();
    let y = src.y();
    let z = src.z();
    let roll = src.roll();
    let pitch = src.pitch();
    let yaw = src.yaw();
}

fn read_FILE_TRANSFER_PROTOCOL(src: &mut packs::FILE_TRANSFER_PROTOCOL::Data_) {
    let target_network = src.target_network();
    let target_system = src.target_system();
    let target_component = src.target_component();
    let payload = src.payload(None);
}

fn write_FILE_TRANSFER_PROTOCOL(dst: &mut packs::FILE_TRANSFER_PROTOCOL::Data_) {
    dst.target_network().set(some_i8);
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.payload(Some(&mut sys::ByValIter::new(&some_i8s)));
}

fn read_RANGEFINDER(src: &mut packs::RANGEFINDER::Data_) {
    let distance = src.distance();
    let voltage = src.voltage();
}

fn write_RANGEFINDER(dst: &mut packs::RANGEFINDER::Data_) {
    dst.distance().set(some_f32);
    dst.voltage().set(some_f32);
}

fn read_RADIO_STATUS(src: &mut packs::RADIO_STATUS::Data_) {
    let rxerrors = src.rxerrors();
    let fixeD = src.fixeD();
    let rssi = src.rssi();
    let remrssi = src.remrssi();
    let txbuf = src.txbuf();
    let noise = src.noise();
    let remnoise = src.remnoise();
}

fn write_RADIO_STATUS(dst: &mut packs::RADIO_STATUS::Data_) {
    dst.rxerrors().set(some_i16);
    dst.fixeD().set(some_i16);
    dst.rssi().set(some_i8);
    dst.remrssi().set(some_i8);
    dst.txbuf().set(some_i8);
    dst.noise().set(some_i8);
    dst.remnoise().set(some_i8);
}

fn read_FENCE_POINT(src: &mut packs::FENCE_POINT::Data_) {
    let target_system = src.target_system();
    let target_component = src.target_component();
    let idx = src.idx();
    let count = src.count();
    let lat = src.lat();
    let lng = src.lng();
}

fn write_FENCE_POINT(dst: &mut packs::FENCE_POINT::Data_) {
    dst.target_system().set(some_i8);
    dst.target_component().set(some_i8);
    dst.idx().set(some_i8);
    dst.count().set(some_i8);
    dst.lat().set(some_f32);
    dst.lng().set(some_f32);
}

fn read_RESOURCE_REQUEST(src: &mut packs::RESOURCE_REQUEST::Data_) {
    let request_id = src.request_id();
    let uri_type = src.uri_type();
    let uri = src.uri(None);
    let transfer_type = src.transfer_type();
    let storage = src.storage(None);
}

fn write_RESOURCE_REQUEST(dst: &mut packs::RESOURCE_REQUEST::Data_) {
    dst.request_id().set(some_i8);
    dst.uri_type().set(some_i8);
    dst.uri(Some(&mut sys::ByValIter::new(&some_i8s)));
    dst.transfer_type().set(some_i8);
    dst.storage(Some(&mut sys::ByValIter::new(&some_i8s)));
}
