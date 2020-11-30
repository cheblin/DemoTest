use ad_hoc_sys as sys;

use std::str::from_utf8_unchecked_mut;
use std::slice::from_raw_parts_mut;
use std::mem::transmute;
use std::ptr::copy_nonoverlapping;
use crate::com::company::demo as host;
use host::GroundControl as packs;


#[repr(C)]
pub struct Metaattitude_target(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaattitude_target {}

pub static mut ATTITUDE_TARGET: Metaattitude_target = Metaattitude_target(106, 0, 1, 0, 37, None, 0, 0, 0, []);

///[q](GroundControl::ATTITUDE_TARGET::q).

pub struct ItemArray__ad_hoc1352 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1352 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
}

impl Iterator for ItemArray__ad_hoc1352 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metamission_count(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamission_count {}

pub static mut MISSION_COUNT: Metamission_count = Metamission_count(50, 1, 0, 0, 5, None, 0, 32, 1, [(&fld__ad_hoc16 as *const _ad_hoc16) as *const sys::Field]);

#[repr(C)]
pub struct Metaadsb_vehicle(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 4]);

unsafe impl std::marker::Sync for Metaadsb_vehicle {}

pub static mut ADSB_VEHICLE: Metaadsb_vehicle = Metaadsb_vehicle(78, 3, 1, 0, 26, None, 2, 202, 4, [(&fld__ad_hoc10 as *const _ad_hoc10) as *const sys::Field,
    (&fld__ad_hoc1964 as *const _ad_hoc1964) as *const sys::Field,
    (&fld__ad_hoc11 as *const _ad_hoc11) as *const sys::Field,
    (&fld__ad_hoc12 as *const _ad_hoc12) as *const sys::Field]);

///[heading](GroundControl::ADSB_VEHICLE::heading).

pub struct Item__ad_hoc1961<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1961<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[hor_velocity](GroundControl::ADSB_VEHICLE::hor_velocity).

pub struct Item__ad_hoc1962<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1962<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[squawk](GroundControl::ADSB_VEHICLE::squawk).

pub struct Item__ad_hoc1966<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1966<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 4);
    }
}

///[ICAO_address](GroundControl::ADSB_VEHICLE::ICAO_address).

pub struct Item__ad_hoc1957<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1957<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 6);
    }
}

///[lat](GroundControl::ADSB_VEHICLE::lat).

pub struct Item__ad_hoc1958<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1958<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 10);
    }
}

///[lon](GroundControl::ADSB_VEHICLE::lon).

pub struct Item__ad_hoc1959<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1959<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 14, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 14);
    }
}

///[altitude](GroundControl::ADSB_VEHICLE::altitude).

pub struct Item__ad_hoc1960<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1960<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 18, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 18);
    }
}

///[ver_velocity](GroundControl::ADSB_VEHICLE::ver_velocity).

pub struct Item__ad_hoc1963<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1963<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 22, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 22);
    }
}

///[tslc](GroundControl::ADSB_VEHICLE::tslc).

pub struct Item__ad_hoc1965<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1965<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 24, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 24);
    }
}

///[altitude_type](GroundControl::ADSB_VEHICLE::altitude_type).

pub struct Item__ad_hoc10<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc10<'a> {
    pub fn get(&mut self) -> Option<packs::ADSB_ALTITUDE_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 202 && !src.set_field(202, -1) { return None; }

        Some({ packs::ADSB_ALTITUDE_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::ADSB_ALTITUDE_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 202
        {
            dst.set_field(202, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

///[callsign](GroundControl::ADSB_VEHICLE::callsign).

pub struct Item__ad_hoc1964<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1964<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 203 && !src.set_field(203, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(203, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[emitter_type](GroundControl::ADSB_VEHICLE::emitter_type).

pub struct Item__ad_hoc11<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc11<'a> {
    pub fn get(&mut self) -> Option<packs::ADSB_EMITTER_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 204 && !src.set_field(204, -1) { return None; }

        Some({ packs::ADSB_EMITTER_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 5)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::ADSB_EMITTER_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 204
        {
            dst.set_field(204, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 5, dst.base.bytes, dst.BIT);
    }
}

///[flags](GroundControl::ADSB_VEHICLE::flags).

pub struct Item__ad_hoc12<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc12<'a> {
    pub fn get(&mut self) -> Option<packs::ADSB_FLAGS> {
        let src = &mut self.0;
        if src.base.field_bit != 205 && !src.set_field(205, -1) { return None; }

        Some({ packs::ADSB_FLAGS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::ADSB_FLAGS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 205
        {
            dst.set_field(205, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metamessage_interval(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamessage_interval {}

pub static mut MESSAGE_INTERVAL: Metamessage_interval = Metamessage_interval(41, 1, 0, 0, 6, None, 0, 0, 0, []);

///[message_id](GroundControl::MESSAGE_INTERVAL::message_id).

pub struct Item__ad_hoc1955(pub *mut u8);

impl Item__ad_hoc1955
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[interval_us](GroundControl::MESSAGE_INTERVAL::interval_us).

pub struct Item__ad_hoc1956(pub *mut u8);

impl Item__ad_hoc1956
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 2);
    }
}

#[repr(C)]
pub struct Metaekf_status_report(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaekf_status_report {}

pub static mut EKF_STATUS_REPORT: Metaekf_status_report = Metaekf_status_report(96, 0, 0, 0, 21, None, 0, 160, 1, [(&fld__ad_hoc710 as *const _ad_hoc710) as *const sys::Field]);

///[velocity_variance](GroundControl::EKF_STATUS_REPORT::velocity_variance).

pub struct Item__ad_hoc2368<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2368<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 0);
    }
}

///[pos_horiz_variance](GroundControl::EKF_STATUS_REPORT::pos_horiz_variance).

pub struct Item__ad_hoc2369<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2369<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 4);
    }
}

///[pos_vert_variance](GroundControl::EKF_STATUS_REPORT::pos_vert_variance).

pub struct Item__ad_hoc2370<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2370<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 8);
    }
}

///[compass_variance](GroundControl::EKF_STATUS_REPORT::compass_variance).

pub struct Item__ad_hoc2371<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2371<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[terrain_alt_variance](GroundControl::EKF_STATUS_REPORT::terrain_alt_variance).

pub struct Item__ad_hoc2372<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2372<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 16);
    }
}

///[flags](GroundControl::EKF_STATUS_REPORT::flags).

pub struct Item__ad_hoc710<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc710<'a> {
    pub fn get(&mut self) -> Option<packs::EKF_STATUS_FLAGS> {
        let src = &mut self.0;
        if src.base.field_bit != 160 && !src.set_field(160, -1) { return None; }

        Some({ packs::EKF_STATUS_FLAGS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::EKF_STATUS_FLAGS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 160
        {
            dst.set_field(160, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaestimator_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaestimator_status {}

pub static mut ESTIMATOR_STATUS: Metaestimator_status = Metaestimator_status(38, 0, 0, 1, 41, None, 0, 320, 1, [(&fld__ad_hoc9 as *const _ad_hoc9) as *const sys::Field]);

///[time_usec](GroundControl::ESTIMATOR_STATUS::time_usec).

pub struct Item__ad_hoc1866<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1866<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 0);
    }
}

///[vel_ratio](GroundControl::ESTIMATOR_STATUS::vel_ratio).

pub struct Item__ad_hoc1867<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1867<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 8);
    }
}

///[pos_horiz_ratio](GroundControl::ESTIMATOR_STATUS::pos_horiz_ratio).

pub struct Item__ad_hoc1868<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1868<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[pos_vert_ratio](GroundControl::ESTIMATOR_STATUS::pos_vert_ratio).

pub struct Item__ad_hoc1869<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1869<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 16);
    }
}

///[mag_ratio](GroundControl::ESTIMATOR_STATUS::mag_ratio).

pub struct Item__ad_hoc1870<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1870<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 20);
    }
}

///[hagl_ratio](GroundControl::ESTIMATOR_STATUS::hagl_ratio).

pub struct Item__ad_hoc1871<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1871<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 24);
    }
}

///[tas_ratio](GroundControl::ESTIMATOR_STATUS::tas_ratio).

pub struct Item__ad_hoc1872<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1872<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 28);
    }
}

///[pos_horiz_accuracy](GroundControl::ESTIMATOR_STATUS::pos_horiz_accuracy).

pub struct Item__ad_hoc1873<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1873<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 32);
    }
}

///[pos_vert_accuracy](GroundControl::ESTIMATOR_STATUS::pos_vert_accuracy).

pub struct Item__ad_hoc1874<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1874<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 36);
    }
}

///[flags](GroundControl::ESTIMATOR_STATUS::flags).

pub struct Item__ad_hoc9<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc9<'a> {
    pub fn get(&mut self) -> Option<packs::ESTIMATOR_STATUS_FLAGS> {
        let src = &mut self.0;
        if src.base.field_bit != 320 && !src.set_field(320, -1) { return None; }

        Some({ packs::ESTIMATOR_STATUS_FLAGS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::ESTIMATOR_STATUS_FLAGS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 320
        {
            dst.set_field(320, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metahwstatus(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metahwstatus {}

pub static mut HWSTATUS: Metahwstatus = Metahwstatus(155, 1, 0, 0, 3, None, 0, 0, 0, []);

///[Vcc](GroundControl::HWSTATUS::Vcc).

pub struct Item__ad_hoc2234(pub *mut u8);

impl Item__ad_hoc2234
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[I2Cerr](GroundControl::HWSTATUS::I2Cerr).

pub struct Item__ad_hoc2235(pub *mut u8);

impl Item__ad_hoc2235
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

#[repr(C)]
pub struct Metatimesync(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metatimesync {}

pub static mut TIMESYNC: Metatimesync = Metatimesync(46, 0, 0, 0, 16, None, 0, 0, 0, []);

///[tc1](GroundControl::TIMESYNC::tc1).

pub struct Item__ad_hoc1608(pub *mut u8);

impl Item__ad_hoc1608
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[ts1](GroundControl::TIMESYNC::ts1).

pub struct Item__ad_hoc1609(pub *mut u8);

impl Item__ad_hoc1609
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 8);
    }
}

#[repr(C)]
pub struct Metaparam_ext_request_list(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                      u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaparam_ext_request_list {}

pub static mut PARAM_EXT_REQUEST_LIST: Metaparam_ext_request_list = Metaparam_ext_request_list(80, 0, 0, 0, 2, None, 0, 0, 0, []);

///[target_system](GroundControl::PARAM_EXT_REQUEST_LIST::target_system).

pub struct Item__ad_hoc2108(pub *mut u8);

impl Item__ad_hoc2108
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::PARAM_EXT_REQUEST_LIST::target_component).

pub struct Item__ad_hoc2109(pub *mut u8);

impl Item__ad_hoc2109
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

#[repr(C)]
pub struct Metaglobal_position_int_cov(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                       u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaglobal_position_int_cov {}

pub static mut GLOBAL_POSITION_INT_COV: Metaglobal_position_int_cov = Metaglobal_position_int_cov(6, 0, 0, 1, 181, None, 0, 1440, 1, [(&fld__ad_hoc636 as *const _ad_hoc636) as *const sys::Field]);

///[covariance](GroundControl::GLOBAL_POSITION_INT_COV::covariance).

pub struct ItemArray__ad_hoc1234 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1234 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
}

impl Iterator for ItemArray__ad_hoc1234 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metabutton_change(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metabutton_change {}

pub static mut BUTTON_CHANGE: Metabutton_change = Metabutton_change(146, 0, 2, 0, 9, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::BUTTON_CHANGE::time_boot_ms).

pub struct Item__ad_hoc1999(pub *mut u8);

impl Item__ad_hoc1999
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[last_change_ms](GroundControl::BUTTON_CHANGE::last_change_ms).

pub struct Item__ad_hoc2000(pub *mut u8);

impl Item__ad_hoc2000
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 4);
    }
}

///[state](GroundControl::BUTTON_CHANGE::state).

pub struct Item__ad_hoc2001(pub *mut u8);

impl Item__ad_hoc2001
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

#[repr(C)]
pub struct Metasafety_set_allowed_area(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                       u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metasafety_set_allowed_area {}

pub static mut SAFETY_SET_ALLOWED_AREA: Metasafety_set_allowed_area = Metasafety_set_allowed_area(24, 0, 0, 0, 27, None, 0, 208, 1, [(&fld__ad_hoc306 as *const _ad_hoc306) as *const sys::Field]);

#[repr(C)]
pub struct Metauavcan_node_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metauavcan_node_status {}

pub static mut UAVCAN_NODE_STATUS: Metauavcan_node_status = Metauavcan_node_status(68, 1, 1, 1, 16, None, 0, 120, 2, [(&fld__ad_hoc350 as *const _ad_hoc350) as *const sys::Field,
    (&fld__ad_hoc349 as *const _ad_hoc349) as *const sys::Field]);

///[vendor_specific_status_code](GroundControl::UAVCAN_NODE_STATUS::vendor_specific_status_code).

pub struct Item__ad_hoc2094<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2094<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[uptime_sec](GroundControl::UAVCAN_NODE_STATUS::uptime_sec).

pub struct Item__ad_hoc2092<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2092<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 2);
    }
}

///[time_usec](GroundControl::UAVCAN_NODE_STATUS::time_usec).

pub struct Item__ad_hoc2091<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2091<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 6);
    }
}

///[sub_mode](GroundControl::UAVCAN_NODE_STATUS::sub_mode).

pub struct Item__ad_hoc2093<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2093<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 14, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 14);
    }
}

///[health](GroundControl::UAVCAN_NODE_STATUS::health).

pub struct Item__ad_hoc350<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc350<'a> {
    pub fn get(&mut self) -> Option<packs::UAVCAN_NODE_HEALTH> {
        let src = &mut self.0;
        if src.base.field_bit != 120 && !src.set_field(120, -1) { return None; }

        Some({ packs::UAVCAN_NODE_HEALTH::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVCAN_NODE_HEALTH) {
        let dst = &mut self.0;
        if dst.base.field_bit != 120
        {
            dst.set_field(120, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

///[mode](GroundControl::UAVCAN_NODE_STATUS::mode).

pub struct Item__ad_hoc349<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc349<'a> {
    pub fn get(&mut self) -> Option<packs::UAVCAN_NODE_MODE> {
        let src = &mut self.0;
        if src.base.field_bit != 121 && !src.set_field(121, -1) { return None; }

        Some({ packs::UAVCAN_NODE_MODE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVCAN_NODE_MODE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 121
        {
            dst.set_field(121, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metacollision(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metacollision {}

pub static mut COLLISION: Metacollision = Metacollision(45, 0, 1, 0, 17, None, 2, 130, 3, [(&fld__ad_hoc804 as *const _ad_hoc804) as *const sys::Field,
    (&fld__ad_hoc803 as *const _ad_hoc803) as *const sys::Field,
    (&fld__ad_hoc802 as *const _ad_hoc802) as *const sys::Field]);

///[id](GroundControl::COLLISION::id).

pub struct Item__ad_hoc1967<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1967<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[time_to_minimum_delta](GroundControl::COLLISION::time_to_minimum_delta).

pub struct Item__ad_hoc1968<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1968<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 4);
    }
}

///[altitude_minimum_delta](GroundControl::COLLISION::altitude_minimum_delta).

pub struct Item__ad_hoc1969<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1969<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 8);
    }
}

///[horizontal_minimum_delta](GroundControl::COLLISION::horizontal_minimum_delta).

pub struct Item__ad_hoc1970<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1970<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[sRc](GroundControl::COLLISION::sRc).

pub struct Item__ad_hoc804<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc804<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_COLLISION_SRC> {
        let src = &mut self.0;
        if src.base.field_bit != 130 && !src.set_field(130, -1) { return None; }

        Some({ packs::MAV_COLLISION_SRC::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_COLLISION_SRC) {
        let dst = &mut self.0;
        if dst.base.field_bit != 130
        {
            dst.set_field(130, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

///[action](GroundControl::COLLISION::action).

pub struct Item__ad_hoc803<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc803<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_COLLISION_ACTION> {
        let src = &mut self.0;
        if src.base.field_bit != 131 && !src.set_field(131, -1) { return None; }

        Some({ packs::MAV_COLLISION_ACTION::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_COLLISION_ACTION) {
        let dst = &mut self.0;
        if dst.base.field_bit != 131
        {
            dst.set_field(131, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[threat_level](GroundControl::COLLISION::threat_level).

pub struct Item__ad_hoc802<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc802<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_COLLISION_THREAT_LEVEL> {
        let src = &mut self.0;
        if src.base.field_bit != 132 && !src.set_field(132, -1) { return None; }

        Some({ packs::MAV_COLLISION_THREAT_LEVEL::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_COLLISION_THREAT_LEVEL) {
        let dst = &mut self.0;
        if dst.base.field_bit != 132
        {
            dst.set_field(132, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metagimbal_torque_cmd_report(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metagimbal_torque_cmd_report {}

pub static mut GIMBAL_TORQUE_CMD_REPORT: Metagimbal_torque_cmd_report = Metagimbal_torque_cmd_report(117, 0, 0, 0, 8, None, 0, 0, 0, []);

///[target_system](GroundControl::GIMBAL_TORQUE_CMD_REPORT::target_system).

pub struct Item__ad_hoc2396(pub *mut u8);

impl Item__ad_hoc2396
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::GIMBAL_TORQUE_CMD_REPORT::target_component).

pub struct Item__ad_hoc2397(pub *mut u8);

impl Item__ad_hoc2397
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[rl_torque_cmd](GroundControl::GIMBAL_TORQUE_CMD_REPORT::rl_torque_cmd).

pub struct Item__ad_hoc2398(pub *mut u8);

impl Item__ad_hoc2398
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[el_torque_cmd](GroundControl::GIMBAL_TORQUE_CMD_REPORT::el_torque_cmd).

pub struct Item__ad_hoc2399(pub *mut u8);

impl Item__ad_hoc2399
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[az_torque_cmd](GroundControl::GIMBAL_TORQUE_CMD_REPORT::az_torque_cmd).

pub struct Item__ad_hoc2400(pub *mut u8);

impl Item__ad_hoc2400
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 6);
    }
}

#[repr(C)]
pub struct Metaaltitude(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaaltitude {}

pub static mut ALTITUDE: Metaaltitude = Metaaltitude(34, 0, 0, 1, 32, None, 0, 0, 0, []);

///[time_usec](GroundControl::ALTITUDE::time_usec).

pub struct Item__ad_hoc1792(pub *mut u8);

impl Item__ad_hoc1792
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[altitude_monotonic](GroundControl::ALTITUDE::altitude_monotonic).

pub struct Item__ad_hoc1793(pub *mut u8);

impl Item__ad_hoc1793
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[altitude_amsl](GroundControl::ALTITUDE::altitude_amsl).

pub struct Item__ad_hoc1794(pub *mut u8);

impl Item__ad_hoc1794
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[altitude_local](GroundControl::ALTITUDE::altitude_local).

pub struct Item__ad_hoc1795(pub *mut u8);

impl Item__ad_hoc1795
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[altitude_relative](GroundControl::ALTITUDE::altitude_relative).

pub struct Item__ad_hoc1796(pub *mut u8);

impl Item__ad_hoc1796
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[altitude_terrain](GroundControl::ALTITUDE::altitude_terrain).

pub struct Item__ad_hoc1797(pub *mut u8);

impl Item__ad_hoc1797
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[bottom_clearance](GroundControl::ALTITUDE::bottom_clearance).

pub struct Item__ad_hoc1798(pub *mut u8);

impl Item__ad_hoc1798
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

#[repr(C)]
pub struct Metahil_state_quaternion(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                    u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metahil_state_quaternion {}

pub static mut HIL_STATE_QUATERNION: Metahil_state_quaternion = Metahil_state_quaternion(196, 2, 0, 1, 64, None, 0, 0, 0, []);

///[ind_airspeed](GroundControl::HIL_STATE_QUATERNION::ind_airspeed).

pub struct Item__ad_hoc1648(pub *mut u8);

impl Item__ad_hoc1648
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[true_airspeed](GroundControl::HIL_STATE_QUATERNION::true_airspeed).

pub struct Item__ad_hoc1649(pub *mut u8);

impl Item__ad_hoc1649
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[time_usec](GroundControl::HIL_STATE_QUATERNION::time_usec).

pub struct Item__ad_hoc1637(pub *mut u8);

impl Item__ad_hoc1637
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 4);
    }
}

///[attitude_quaternion](GroundControl::HIL_STATE_QUATERNION::attitude_quaternion).

pub struct ItemArray__ad_hoc1638 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1638 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1638 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[rollspeed](GroundControl::HIL_STATE_QUATERNION::rollspeed).

pub struct Item__ad_hoc1639(pub *mut u8);

impl Item__ad_hoc1639
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[pitchspeed](GroundControl::HIL_STATE_QUATERNION::pitchspeed).

pub struct Item__ad_hoc1640(pub *mut u8);

impl Item__ad_hoc1640
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

///[yawspeed](GroundControl::HIL_STATE_QUATERNION::yawspeed).

pub struct Item__ad_hoc1641(pub *mut u8);

impl Item__ad_hoc1641
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 36); }
}

///[lat](GroundControl::HIL_STATE_QUATERNION::lat).

pub struct Item__ad_hoc1642(pub *mut u8);

impl Item__ad_hoc1642
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 40, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 40);
    }
}

///[lon](GroundControl::HIL_STATE_QUATERNION::lon).

pub struct Item__ad_hoc1643(pub *mut u8);

impl Item__ad_hoc1643
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 44, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 44);
    }
}

///[alt](GroundControl::HIL_STATE_QUATERNION::alt).

pub struct Item__ad_hoc1644(pub *mut u8);

impl Item__ad_hoc1644
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 48, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 48);
    }
}

///[vx](GroundControl::HIL_STATE_QUATERNION::vx).

pub struct Item__ad_hoc1645(pub *mut u8);

impl Item__ad_hoc1645
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 52, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 52);
    }
}

///[vy](GroundControl::HIL_STATE_QUATERNION::vy).

pub struct Item__ad_hoc1646(pub *mut u8);

impl Item__ad_hoc1646
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 54, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 54);
    }
}

///[vz](GroundControl::HIL_STATE_QUATERNION::vz).

pub struct Item__ad_hoc1647(pub *mut u8);

impl Item__ad_hoc1647
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 56, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 56);
    }
}

///[xacc](GroundControl::HIL_STATE_QUATERNION::xacc).

pub struct Item__ad_hoc1650(pub *mut u8);

impl Item__ad_hoc1650
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 58, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 58);
    }
}

///[yacc](GroundControl::HIL_STATE_QUATERNION::yacc).

pub struct Item__ad_hoc1651(pub *mut u8);

impl Item__ad_hoc1651
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 60, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 60);
    }
}

///[zacc](GroundControl::HIL_STATE_QUATERNION::zacc).

pub struct Item__ad_hoc1652(pub *mut u8);

impl Item__ad_hoc1652
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 62, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 62);
    }
}

#[repr(C)]
pub struct Metasensor_offsets(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metasensor_offsets {}

pub static mut SENSOR_OFFSETS: Metasensor_offsets = Metasensor_offsets(128, 0, 0, 0, 42, None, 0, 0, 0, []);

///[mag_ofs_x](GroundControl::SENSOR_OFFSETS::mag_ofs_x).

pub struct Item__ad_hoc2141(pub *mut u8);

impl Item__ad_hoc2141
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[mag_ofs_y](GroundControl::SENSOR_OFFSETS::mag_ofs_y).

pub struct Item__ad_hoc2142(pub *mut u8);

impl Item__ad_hoc2142
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[mag_ofs_z](GroundControl::SENSOR_OFFSETS::mag_ofs_z).

pub struct Item__ad_hoc2143(pub *mut u8);

impl Item__ad_hoc2143
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[mag_declination](GroundControl::SENSOR_OFFSETS::mag_declination).

pub struct Item__ad_hoc2144(pub *mut u8);

impl Item__ad_hoc2144
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 6, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 6); }
}

///[raw_press](GroundControl::SENSOR_OFFSETS::raw_press).

pub struct Item__ad_hoc2145(pub *mut u8);

impl Item__ad_hoc2145
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 10);
    }
}

///[raw_temp](GroundControl::SENSOR_OFFSETS::raw_temp).

pub struct Item__ad_hoc2146(pub *mut u8);

impl Item__ad_hoc2146
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 14, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 14);
    }
}

///[gyro_cal_x](GroundControl::SENSOR_OFFSETS::gyro_cal_x).

pub struct Item__ad_hoc2147(pub *mut u8);

impl Item__ad_hoc2147
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 18, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 18); }
}

///[gyro_cal_y](GroundControl::SENSOR_OFFSETS::gyro_cal_y).

pub struct Item__ad_hoc2148(pub *mut u8);

impl Item__ad_hoc2148
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 22, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 22); }
}

///[gyro_cal_z](GroundControl::SENSOR_OFFSETS::gyro_cal_z).

pub struct Item__ad_hoc2149(pub *mut u8);

impl Item__ad_hoc2149
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 26, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 26); }
}

///[accel_cal_x](GroundControl::SENSOR_OFFSETS::accel_cal_x).

pub struct Item__ad_hoc2150(pub *mut u8);

impl Item__ad_hoc2150
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 30, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 30); }
}

///[accel_cal_y](GroundControl::SENSOR_OFFSETS::accel_cal_y).

pub struct Item__ad_hoc2151(pub *mut u8);

impl Item__ad_hoc2151
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 34, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 34); }
}

///[accel_cal_z](GroundControl::SENSOR_OFFSETS::accel_cal_z).

pub struct Item__ad_hoc2152(pub *mut u8);

impl Item__ad_hoc2152
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 38, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 38); }
}

#[repr(C)]
pub struct Metastorage_information(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                   u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metastorage_information {}

pub static mut STORAGE_INFORMATION: Metastorage_information = Metastorage_information(58, 0, 1, 0, 27, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::STORAGE_INFORMATION::time_boot_ms).

pub struct Item__ad_hoc2018(pub *mut u8);

impl Item__ad_hoc2018
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[storage_id](GroundControl::STORAGE_INFORMATION::storage_id).

pub struct Item__ad_hoc2019(pub *mut u8);

impl Item__ad_hoc2019
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[storage_count](GroundControl::STORAGE_INFORMATION::storage_count).

pub struct Item__ad_hoc2020(pub *mut u8);

impl Item__ad_hoc2020
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

///[status](GroundControl::STORAGE_INFORMATION::status).

pub struct Item__ad_hoc2021(pub *mut u8);

impl Item__ad_hoc2021
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 6);
    }
}

///[total_capacity](GroundControl::STORAGE_INFORMATION::total_capacity).

pub struct Item__ad_hoc2022(pub *mut u8);

impl Item__ad_hoc2022
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 7, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 7); }
}

///[used_capacity](GroundControl::STORAGE_INFORMATION::used_capacity).

pub struct Item__ad_hoc2023(pub *mut u8);

impl Item__ad_hoc2023
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 11, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 11); }
}

///[available_capacity](GroundControl::STORAGE_INFORMATION::available_capacity).

pub struct Item__ad_hoc2024(pub *mut u8);

impl Item__ad_hoc2024
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 15, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 15); }
}

///[read_speed](GroundControl::STORAGE_INFORMATION::read_speed).

pub struct Item__ad_hoc2025(pub *mut u8);

impl Item__ad_hoc2025
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 19, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 19); }
}

///[write_speed](GroundControl::STORAGE_INFORMATION::write_speed).

pub struct Item__ad_hoc2026(pub *mut u8);

impl Item__ad_hoc2026
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 23, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 23); }
}

#[repr(C)]
pub struct Metacamera_information(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metacamera_information {}

pub static mut CAMERA_INFORMATION: Metacamera_information = Metacamera_information(157, 3, 2, 0, 92, None, 2, 730, 2, [(&fld__ad_hoc342 as *const _ad_hoc342) as *const sys::Field,
    (&fld__ad_hoc2016 as *const _ad_hoc2016) as *const sys::Field]);

///[resolution_h](GroundControl::CAMERA_INFORMATION::resolution_h).

pub struct Item__ad_hoc2012<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2012<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[resolution_v](GroundControl::CAMERA_INFORMATION::resolution_v).

pub struct Item__ad_hoc2013<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2013<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[cam_definition_version](GroundControl::CAMERA_INFORMATION::cam_definition_version).

pub struct Item__ad_hoc2015<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2015<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 4);
    }
}

///[time_boot_ms](GroundControl::CAMERA_INFORMATION::time_boot_ms).

pub struct Item__ad_hoc2005<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2005<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 6);
    }
}

///[firmware_version](GroundControl::CAMERA_INFORMATION::firmware_version).

pub struct Item__ad_hoc2008<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2008<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 10);
    }
}

///[vendor_name](GroundControl::CAMERA_INFORMATION::vendor_name).

pub struct ItemArray__ad_hoc2006 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2006 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2006 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[model_name](GroundControl::CAMERA_INFORMATION::model_name).

pub struct ItemArray__ad_hoc2007 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2007 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2007 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[focal_length](GroundControl::CAMERA_INFORMATION::focal_length).

pub struct Item__ad_hoc2009<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2009<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 78, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 78);
    }
}

///[sensor_size_h](GroundControl::CAMERA_INFORMATION::sensor_size_h).

pub struct Item__ad_hoc2010<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2010<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 82, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 82);
    }
}

///[sensor_size_v](GroundControl::CAMERA_INFORMATION::sensor_size_v).

pub struct Item__ad_hoc2011<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2011<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 86, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 86);
    }
}

///[lens_id](GroundControl::CAMERA_INFORMATION::lens_id).

pub struct Item__ad_hoc2014<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2014<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 90, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 90);
    }
}

///[flags](GroundControl::CAMERA_INFORMATION::flags).

pub struct Item__ad_hoc342<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc342<'a> {
    pub fn get(&mut self) -> Option<packs::CAMERA_CAP_FLAGS> {
        let src = &mut self.0;
        if src.base.field_bit != 730 && !src.set_field(730, -1) { return None; }

        Some({ packs::CAMERA_CAP_FLAGS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::CAMERA_CAP_FLAGS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 730
        {
            dst.set_field(730, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[cam_definition_uri](GroundControl::CAMERA_INFORMATION::cam_definition_uri).

pub struct Item__ad_hoc2016<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2016<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 731 && !src.set_field(731, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(731, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metagps_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                          u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metagps_status {}

pub static mut GPS_STATUS: Metagps_status = Metagps_status(48, 0, 0, 0, 101, None, 0, 0, 0, []);

///[satellite_prn](GroundControl::GPS_STATUS::satellite_prn).

pub struct ItemArray__ad_hoc1035 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1035 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
}

impl Iterator for ItemArray__ad_hoc1035 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[satellite_used](GroundControl::GPS_STATUS::satellite_used).

pub struct ItemArray__ad_hoc1036 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1036 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
}

impl Iterator for ItemArray__ad_hoc1036 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[satellite_elevation](GroundControl::GPS_STATUS::satellite_elevation).

pub struct ItemArray__ad_hoc1037 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1037 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
}

impl Iterator for ItemArray__ad_hoc1037 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[satellite_azimuth](GroundControl::GPS_STATUS::satellite_azimuth).

pub struct ItemArray__ad_hoc1038 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1038 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
}

impl Iterator for ItemArray__ad_hoc1038 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[satellite_snr](GroundControl::GPS_STATUS::satellite_snr).

pub struct ItemArray__ad_hoc1039 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1039 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
}

impl Iterator for ItemArray__ad_hoc1039 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metadevice_op_write_reply(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadevice_op_write_reply {}

pub static mut DEVICE_OP_WRITE_REPLY: Metadevice_op_write_reply = Metadevice_op_write_reply(33, 0, 1, 0, 5, None, 0, 0, 0, []);

///[request_id](GroundControl::DEVICE_OP_WRITE_REPLY::request_id).

pub struct Item__ad_hoc2431(pub *mut u8);

impl Item__ad_hoc2431
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[result](GroundControl::DEVICE_OP_WRITE_REPLY::result).

pub struct Item__ad_hoc2432(pub *mut u8);

impl Item__ad_hoc2432
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

#[repr(C)]
pub struct Metaparam_set(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metaparam_set {}

pub static mut PARAM_SET: Metaparam_set = Metaparam_set(123, 0, 0, 0, 7, None, 2, 50, 2, [(&fld__ad_hoc1018 as *const _ad_hoc1018) as *const sys::Field,
    (&fld__ad_hoc1 as *const _ad_hoc1) as *const sys::Field]);

#[repr(C)]
pub struct Metaterrain_data(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaterrain_data {}

pub static mut TERRAIN_DATA: Metaterrain_data = Metaterrain_data(173, 1, 0, 0, 43, None, 0, 0, 0, []);

///[grid_spacing](GroundControl::TERRAIN_DATA::grid_spacing).

pub struct Item__ad_hoc1763(pub *mut u8);

impl Item__ad_hoc1763
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[lat](GroundControl::TERRAIN_DATA::lat).

pub struct Item__ad_hoc1761(pub *mut u8);

impl Item__ad_hoc1761
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 2);
    }
}

///[lon](GroundControl::TERRAIN_DATA::lon).

pub struct Item__ad_hoc1762(pub *mut u8);

impl Item__ad_hoc1762
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[gridbit](GroundControl::TERRAIN_DATA::gridbit).

pub struct Item__ad_hoc1764(pub *mut u8);

impl Item__ad_hoc1764
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 10);
    }
}

///[daTa](GroundControl::TERRAIN_DATA::daTa).

pub struct ItemArray__ad_hoc1765 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1765 {
    pub fn get(&mut self, index: usize) -> i16 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, index: usize, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.bytes, self.offset + index * 2);
    }
}

impl Iterator for ItemArray__ad_hoc1765 {
    type Item = i16;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metagimbal_control(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metagimbal_control {}

pub static mut GIMBAL_CONTROL: Metagimbal_control = Metagimbal_control(104, 0, 0, 0, 14, None, 0, 0, 0, []);

///[target_system](GroundControl::GIMBAL_CONTROL::target_system).

pub struct Item__ad_hoc2391(pub *mut u8);

impl Item__ad_hoc2391
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::GIMBAL_CONTROL::target_component).

pub struct Item__ad_hoc2392(pub *mut u8);

impl Item__ad_hoc2392
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[demanded_rate_x](GroundControl::GIMBAL_CONTROL::demanded_rate_x).

pub struct Item__ad_hoc2393(pub *mut u8);

impl Item__ad_hoc2393
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 2, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 2); }
}

///[demanded_rate_y](GroundControl::GIMBAL_CONTROL::demanded_rate_y).

pub struct Item__ad_hoc2394(pub *mut u8);

impl Item__ad_hoc2394
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 6, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 6); }
}

///[demanded_rate_z](GroundControl::GIMBAL_CONTROL::demanded_rate_z).

pub struct Item__ad_hoc2395(pub *mut u8);

impl Item__ad_hoc2395
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 10, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 10); }
}

#[repr(C)]
pub struct Metarc_channels_override(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                    u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metarc_channels_override {}

pub static mut RC_CHANNELS_OVERRIDE: Metarc_channels_override = Metarc_channels_override(55, 8, 0, 0, 18, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metascaled_imu(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                          u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metascaled_imu {}

pub static mut SCALED_IMU: Metascaled_imu = Metascaled_imu(195, 0, 1, 0, 22, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metavideo_stream_information(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                        u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metavideo_stream_information {}

pub static mut VIDEO_STREAM_INFORMATION: Metavideo_stream_information = Metavideo_stream_information(1, 3, 1, 0, 17, None, 2, 130, 1, [(&fld__ad_hoc2074 as *const _ad_hoc2074) as *const sys::Field]);

///[resolution_h](GroundControl::VIDEO_STREAM_INFORMATION::resolution_h).

pub struct Item__ad_hoc2070<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2070<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[resolution_v](GroundControl::VIDEO_STREAM_INFORMATION::resolution_v).

pub struct Item__ad_hoc2071<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2071<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[rotation](GroundControl::VIDEO_STREAM_INFORMATION::rotation).

pub struct Item__ad_hoc2073<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2073<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 4);
    }
}

///[bitrate](GroundControl::VIDEO_STREAM_INFORMATION::bitrate).

pub struct Item__ad_hoc2072<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2072<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 6);
    }
}

///[camera_id](GroundControl::VIDEO_STREAM_INFORMATION::camera_id).

pub struct Item__ad_hoc2067<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2067<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 10);
    }
}

///[status](GroundControl::VIDEO_STREAM_INFORMATION::status).

pub struct Item__ad_hoc2068<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2068<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 11, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 11);
    }
}

///[framerate](GroundControl::VIDEO_STREAM_INFORMATION::framerate).

pub struct Item__ad_hoc2069<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2069<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[uri](GroundControl::VIDEO_STREAM_INFORMATION::uri).

pub struct Item__ad_hoc2074<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2074<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 130 && !src.set_field(130, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(130, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metaahrs(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                    u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaahrs {}

pub static mut AHRS: Metaahrs = Metaahrs(214, 0, 0, 0, 28, None, 0, 0, 0, []);

///[omegaIx](GroundControl::AHRS::omegaIx).

pub struct Item__ad_hoc2216(pub *mut u8);

impl Item__ad_hoc2216
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[omegaIy](GroundControl::AHRS::omegaIy).

pub struct Item__ad_hoc2217(pub *mut u8);

impl Item__ad_hoc2217
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[omegaIz](GroundControl::AHRS::omegaIz).

pub struct Item__ad_hoc2218(pub *mut u8);

impl Item__ad_hoc2218
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[accel_weight](GroundControl::AHRS::accel_weight).

pub struct Item__ad_hoc2219(pub *mut u8);

impl Item__ad_hoc2219
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[renorm_val](GroundControl::AHRS::renorm_val).

pub struct Item__ad_hoc2220(pub *mut u8);

impl Item__ad_hoc2220
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[error_rp](GroundControl::AHRS::error_rp).

pub struct Item__ad_hoc2221(pub *mut u8);

impl Item__ad_hoc2221
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[error_yaw](GroundControl::AHRS::error_yaw).

pub struct Item__ad_hoc2222(pub *mut u8);

impl Item__ad_hoc2222
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

#[repr(C)]
pub struct Metadebug(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadebug {}

pub static mut DEBUG: Metadebug = Metadebug(126, 0, 1, 0, 9, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::DEBUG::time_boot_ms).

pub struct Item__ad_hoc1992(pub *mut u8);

impl Item__ad_hoc1992
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[ind](GroundControl::DEBUG::ind).

pub struct Item__ad_hoc1993(pub *mut u8);

impl Item__ad_hoc1993
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[value](GroundControl::DEBUG::value).

pub struct Item__ad_hoc1994(pub *mut u8);

impl Item__ad_hoc1994
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 5, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 5); }
}

#[repr(C)]
pub struct Metacamera_image_captured(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metacamera_image_captured {}

pub static mut CAMERA_IMAGE_CAPTURED: Metacamera_image_captured = Metacamera_image_captured(10, 0, 1, 1, 51, None, 2, 402, 1, [(&fld__ad_hoc2043 as *const _ad_hoc2043) as *const sys::Field]);

///[time_boot_ms](GroundControl::CAMERA_IMAGE_CAPTURED::time_boot_ms).

pub struct Item__ad_hoc2033<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2033<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[time_utc](GroundControl::CAMERA_IMAGE_CAPTURED::time_utc).

pub struct Item__ad_hoc2034<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2034<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 4);
    }
}

///[camera_id](GroundControl::CAMERA_IMAGE_CAPTURED::camera_id).

pub struct Item__ad_hoc2035<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2035<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 12, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 12);
    }
}

///[lat](GroundControl::CAMERA_IMAGE_CAPTURED::lat).

pub struct Item__ad_hoc2036<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2036<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 13, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 13);
    }
}

///[lon](GroundControl::CAMERA_IMAGE_CAPTURED::lon).

pub struct Item__ad_hoc2037<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2037<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 17, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 17);
    }
}

///[alt](GroundControl::CAMERA_IMAGE_CAPTURED::alt).

pub struct Item__ad_hoc2038<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2038<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 21, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 21);
    }
}

///[relative_alt](GroundControl::CAMERA_IMAGE_CAPTURED::relative_alt).

pub struct Item__ad_hoc2039<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2039<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 25, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 25);
    }
}

///[q](GroundControl::CAMERA_IMAGE_CAPTURED::q).

pub struct ItemArray__ad_hoc2040 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2040 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc2040 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[image_index](GroundControl::CAMERA_IMAGE_CAPTURED::image_index).

pub struct Item__ad_hoc2041<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2041<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 45, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 45);
    }
}

///[capture_result](GroundControl::CAMERA_IMAGE_CAPTURED::capture_result).

pub struct Item__ad_hoc2042<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2042<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 49, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 49);
    }
}

///[file_url](GroundControl::CAMERA_IMAGE_CAPTURED::file_url).

pub struct Item__ad_hoc2043<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2043<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 402 && !src.set_field(402, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(402, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metalog_entry(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalog_entry {}

pub static mut LOG_ENTRY: Metalog_entry = Metalog_entry(161, 3, 2, 0, 14, None, 0, 0, 0, []);

///[id](GroundControl::LOG_ENTRY::id).

pub struct Item__ad_hoc1667(pub *mut u8);

impl Item__ad_hoc1667
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[num_logs](GroundControl::LOG_ENTRY::num_logs).

pub struct Item__ad_hoc1668(pub *mut u8);

impl Item__ad_hoc1668
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[last_log_num](GroundControl::LOG_ENTRY::last_log_num).

pub struct Item__ad_hoc1669(pub *mut u8);

impl Item__ad_hoc1669
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[time_utc](GroundControl::LOG_ENTRY::time_utc).

pub struct Item__ad_hoc1670(pub *mut u8);

impl Item__ad_hoc1670
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[size](GroundControl::LOG_ENTRY::size).

pub struct Item__ad_hoc1671(pub *mut u8);

impl Item__ad_hoc1671
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 10);
    }
}

#[repr(C)]
pub struct Metaactuator_control_target(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                       u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaactuator_control_target {}

pub static mut ACTUATOR_CONTROL_TARGET: Metaactuator_control_target = Metaactuator_control_target(3, 0, 0, 1, 41, None, 0, 0, 0, []);

///[time_usec](GroundControl::ACTUATOR_CONTROL_TARGET::time_usec).

pub struct Item__ad_hoc1789(pub *mut u8);

impl Item__ad_hoc1789
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[group_mlx](GroundControl::ACTUATOR_CONTROL_TARGET::group_mlx).

pub struct Item__ad_hoc1790(pub *mut u8);

impl Item__ad_hoc1790
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

///[controls](GroundControl::ACTUATOR_CONTROL_TARGET::controls).

pub struct ItemArray__ad_hoc1791 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1791 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1791 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metahigh_latency(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metahigh_latency {}

pub static mut HIGH_LATENCY: Metahigh_latency = Metahigh_latency(87, 2, 1, 0, 38, None, 2, 298, 3, [(&fld__ad_hoc213 as *const _ad_hoc213) as *const sys::Field,
    (&fld__ad_hoc215 as *const _ad_hoc215) as *const sys::Field,
    (&fld__ad_hoc214 as *const _ad_hoc214) as *const sys::Field]);

///[heading](GroundControl::HIGH_LATENCY::heading).

pub struct Item__ad_hoc1907<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1907<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[wp_distance](GroundControl::HIGH_LATENCY::wp_distance).

pub struct Item__ad_hoc1924<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1924<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[custom_mode](GroundControl::HIGH_LATENCY::custom_mode).

pub struct Item__ad_hoc1904<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1904<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 4);
    }
}

///[roll](GroundControl::HIGH_LATENCY::roll).

pub struct Item__ad_hoc1905<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1905<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 8, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 8);
    }
}

///[pitch](GroundControl::HIGH_LATENCY::pitch).

pub struct Item__ad_hoc1906<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1906<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 10);
    }
}

///[throttle](GroundControl::HIGH_LATENCY::throttle).

pub struct Item__ad_hoc1908<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1908<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 12, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 12);
    }
}

///[heading_sp](GroundControl::HIGH_LATENCY::heading_sp).

pub struct Item__ad_hoc1909<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1909<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 13, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 13);
    }
}

///[latitude](GroundControl::HIGH_LATENCY::latitude).

pub struct Item__ad_hoc1910<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1910<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 15, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 15);
    }
}

///[longitude](GroundControl::HIGH_LATENCY::longitude).

pub struct Item__ad_hoc1911<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1911<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 19, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 19);
    }
}

///[altitude_amsl](GroundControl::HIGH_LATENCY::altitude_amsl).

pub struct Item__ad_hoc1912<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1912<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 23, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 23);
    }
}

///[altitude_sp](GroundControl::HIGH_LATENCY::altitude_sp).

pub struct Item__ad_hoc1913<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1913<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 25, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 25);
    }
}

///[airspeed](GroundControl::HIGH_LATENCY::airspeed).

pub struct Item__ad_hoc1914<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1914<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 27, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 27);
    }
}

///[airspeed_sp](GroundControl::HIGH_LATENCY::airspeed_sp).

pub struct Item__ad_hoc1915<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1915<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 28, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 28);
    }
}

///[groundspeed](GroundControl::HIGH_LATENCY::groundspeed).

pub struct Item__ad_hoc1916<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1916<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 29, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 29);
    }
}

///[climb_rate](GroundControl::HIGH_LATENCY::climb_rate).

pub struct Item__ad_hoc1917<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1917<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 30, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 30);
    }
}

///[gps_nsat](GroundControl::HIGH_LATENCY::gps_nsat).

pub struct Item__ad_hoc1918<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1918<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 31, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 31);
    }
}

///[battery_remaining](GroundControl::HIGH_LATENCY::battery_remaining).

pub struct Item__ad_hoc1919<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1919<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 32, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 32);
    }
}

///[temperature](GroundControl::HIGH_LATENCY::temperature).

pub struct Item__ad_hoc1920<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1920<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 33, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 33);
    }
}

///[temperature_air](GroundControl::HIGH_LATENCY::temperature_air).

pub struct Item__ad_hoc1921<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1921<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 34, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 34);
    }
}

///[failsafe](GroundControl::HIGH_LATENCY::failsafe).

pub struct Item__ad_hoc1922<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1922<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 35, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 35);
    }
}

///[wp_num](GroundControl::HIGH_LATENCY::wp_num).

pub struct Item__ad_hoc1923<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1923<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 36, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 36);
    }
}

///[base_mode](GroundControl::HIGH_LATENCY::base_mode).

pub struct Item__ad_hoc213<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc213<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_MODE_FLAG> {
        let src = &mut self.0;
        if src.base.field_bit != 298 && !src.set_field(298, -1) { return None; }

        Some({ packs::MAV_MODE_FLAG::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_MODE_FLAG) {
        let dst = &mut self.0;
        if dst.base.field_bit != 298
        {
            dst.set_field(298, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

///[landed_state](GroundControl::HIGH_LATENCY::landed_state).

pub struct Item__ad_hoc215<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc215<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_LANDED_STATE> {
        let src = &mut self.0;
        if src.base.field_bit != 299 && !src.set_field(299, -1) { return None; }

        Some({ packs::MAV_LANDED_STATE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_LANDED_STATE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 299
        {
            dst.set_field(299, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[gps_fix_type](GroundControl::HIGH_LATENCY::gps_fix_type).

pub struct Item__ad_hoc214<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc214<'a> {
    pub fn get(&mut self) -> Option<packs::GPS_FIX_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 300 && !src.set_field(300, -1) { return None; }

        Some({ packs::GPS_FIX_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GPS_FIX_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 300
        {
            dst.set_field(300, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaparam_request_read(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaparam_request_read {}

pub static mut PARAM_REQUEST_READ: Metaparam_request_read = Metaparam_request_read(166, 0, 0, 0, 5, None, 2, 34, 1, [(&fld__ad_hoc1008 as *const _ad_hoc1008) as *const sys::Field]);

#[repr(C)]
pub struct Metaset_attitude_target(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                   u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaset_attitude_target {}

pub static mut SET_ATTITUDE_TARGET: Metaset_attitude_target = Metaset_attitude_target(74, 0, 1, 0, 39, None, 0, 0, 0, []);

///[q](GroundControl::SET_ATTITUDE_TARGET::q).

pub struct ItemArray__ad_hoc1345 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1345 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
}

impl Iterator for ItemArray__ad_hoc1345 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metafollow_target(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metafollow_target {}

pub static mut FOLLOW_TARGET: Metafollow_target = Metafollow_target(39, 0, 0, 2, 93, None, 0, 0, 0, []);

///[timestamp](GroundControl::FOLLOW_TARGET::timestamp).

pub struct Item__ad_hoc1808(pub *mut u8);

impl Item__ad_hoc1808
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[custom_state](GroundControl::FOLLOW_TARGET::custom_state).

pub struct Item__ad_hoc1818(pub *mut u8);

impl Item__ad_hoc1818
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 8);
    }
}

///[est_capabilities](GroundControl::FOLLOW_TARGET::est_capabilities).

pub struct Item__ad_hoc1809(pub *mut u8);

impl Item__ad_hoc1809
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 16);
    }
}

///[lat](GroundControl::FOLLOW_TARGET::lat).

pub struct Item__ad_hoc1810(pub *mut u8);

impl Item__ad_hoc1810
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 17, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 17);
    }
}

///[lon](GroundControl::FOLLOW_TARGET::lon).

pub struct Item__ad_hoc1811(pub *mut u8);

impl Item__ad_hoc1811
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 21, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 21);
    }
}

///[alt](GroundControl::FOLLOW_TARGET::alt).

pub struct Item__ad_hoc1812(pub *mut u8);

impl Item__ad_hoc1812
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 25, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 25); }
}

///[vel](GroundControl::FOLLOW_TARGET::vel).

pub struct ItemArray__ad_hoc1813 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1813 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1813 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[acc](GroundControl::FOLLOW_TARGET::acc).

pub struct ItemArray__ad_hoc1814 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1814 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1814 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[attitude_q](GroundControl::FOLLOW_TARGET::attitude_q).

pub struct ItemArray__ad_hoc1815 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1815 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1815 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[rates](GroundControl::FOLLOW_TARGET::rates).

pub struct ItemArray__ad_hoc1816 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1816 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1816 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[position_cov](GroundControl::FOLLOW_TARGET::position_cov).

pub struct ItemArray__ad_hoc1817 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1817 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1817 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metahil_state(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metahil_state {}

pub static mut HIL_STATE: Metahil_state = Metahil_state(140, 0, 0, 1, 56, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metahome_position(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metahome_position {}

pub static mut HOME_POSITION: Metahome_position = Metahome_position(210, 0, 0, 0, 53, None, 0, 416, 1, [(&fld__ad_hoc1942 as *const _ad_hoc1942) as *const sys::Field]);

///[latitude](GroundControl::HOME_POSITION::latitude).

pub struct Item__ad_hoc1932<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1932<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[longitude](GroundControl::HOME_POSITION::longitude).

pub struct Item__ad_hoc1933<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1933<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 4);
    }
}

///[altitude](GroundControl::HOME_POSITION::altitude).

pub struct Item__ad_hoc1934<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1934<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 8, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 8);
    }
}

///[x](GroundControl::HOME_POSITION::x).

pub struct Item__ad_hoc1935<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1935<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[y](GroundControl::HOME_POSITION::y).

pub struct Item__ad_hoc1936<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1936<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 16);
    }
}

///[z](GroundControl::HOME_POSITION::z).

pub struct Item__ad_hoc1937<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1937<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 20);
    }
}

///[q](GroundControl::HOME_POSITION::q).

pub struct ItemArray__ad_hoc1938 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1938 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1938 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[approach_x](GroundControl::HOME_POSITION::approach_x).

pub struct Item__ad_hoc1939<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1939<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 40);
    }
}

///[approach_y](GroundControl::HOME_POSITION::approach_y).

pub struct Item__ad_hoc1940<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1940<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 44, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 44);
    }
}

///[approach_z](GroundControl::HOME_POSITION::approach_z).

pub struct Item__ad_hoc1941<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1941<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 48, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 48);
    }
}

///[time_usec](GroundControl::HOME_POSITION::time_usec).

pub struct Item__ad_hoc1942<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1942<'a> {
    pub fn get(&mut self) -> Option<i64> {
        let src = &mut self.0;
        if src.base.field_bit != 416 && !src.set_field(416, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 8 as usize) as i64;
            (dst) as i64
        })
    }

    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        if dst.base.field_bit != 416
        {
            dst.set_field(416, 0i32);
        }


        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, dst.BYTE);
    }
}

#[repr(C)]
pub struct Metafence_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metafence_status {}

pub static mut FENCE_STATUS: Metafence_status = Metafence_status(93, 1, 1, 0, 8, None, 0, 56, 1, [(&fld__ad_hoc85 as *const _ad_hoc85) as *const sys::Field]);

///[breach_count](GroundControl::FENCE_STATUS::breach_count).

pub struct Item__ad_hoc2214<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2214<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[breach_time](GroundControl::FENCE_STATUS::breach_time).

pub struct Item__ad_hoc2215<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2215<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 2);
    }
}

///[breach_status](GroundControl::FENCE_STATUS::breach_status).

pub struct Item__ad_hoc2213<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2213<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 6);
    }
}

///[breach_type](GroundControl::FENCE_STATUS::breach_type).

pub struct Item__ad_hoc85<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc85<'a> {
    pub fn get(&mut self) -> Option<packs::FENCE_BREACH> {
        let src = &mut self.0;
        if src.base.field_bit != 56 && !src.set_field(56, -1) { return None; }

        Some({ packs::FENCE_BREACH::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::FENCE_BREACH) {
        let dst = &mut self.0;
        if dst.base.field_bit != 56
        {
            dst.set_field(56, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaremote_log_block_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                       u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaremote_log_block_status {}

pub static mut REMOTE_LOG_BLOCK_STATUS: Metaremote_log_block_status = Metaremote_log_block_status(171, 0, 1, 0, 7, None, 0, 48, 1, [(&fld__ad_hoc372 as *const _ad_hoc372) as *const sys::Field]);

///[seqno](GroundControl::REMOTE_LOG_BLOCK_STATUS::seqno).

pub struct Item__ad_hoc2340<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2340<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[target_system](GroundControl::REMOTE_LOG_BLOCK_STATUS::target_system).

pub struct Item__ad_hoc2338<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2338<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 4);
    }
}

///[target_component](GroundControl::REMOTE_LOG_BLOCK_STATUS::target_component).

pub struct Item__ad_hoc2339<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2339<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 5);
    }
}

///[status](GroundControl::REMOTE_LOG_BLOCK_STATUS::status).

pub struct Item__ad_hoc372<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc372<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_REMOTE_LOG_DATA_BLOCK_STATUSES> {
        let src = &mut self.0;
        if src.base.field_bit != 48 && !src.set_field(48, -1) { return None; }

        Some({ packs::MAV_REMOTE_LOG_DATA_BLOCK_STATUSES::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_REMOTE_LOG_DATA_BLOCK_STATUSES) {
        let dst = &mut self.0;
        if dst.base.field_bit != 48
        {
            dst.set_field(48, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaobstacle_distance(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaobstacle_distance {}

pub static mut OBSTACLE_DISTANCE: Metaobstacle_distance = Metaobstacle_distance(66, 74, 0, 1, 158, None, 0, 1256, 1, [(&fld__ad_hoc282 as *const _ad_hoc282) as *const sys::Field]);

///[distances](GroundControl::OBSTACLE_DISTANCE::distances).

pub struct ItemArray__ad_hoc2121 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2121 {
    pub fn get(&mut self, index: usize) -> i16 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, index: usize, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.bytes, self.offset + index * 2);
    }
}

impl Iterator for ItemArray__ad_hoc2121 {
    type Item = i16;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[min_distance](GroundControl::OBSTACLE_DISTANCE::min_distance).

pub struct Item__ad_hoc2123<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2123<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 144, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 144);
    }
}

///[max_distance](GroundControl::OBSTACLE_DISTANCE::max_distance).

pub struct Item__ad_hoc2124<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2124<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 146, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 146);
    }
}

///[time_usec](GroundControl::OBSTACLE_DISTANCE::time_usec).

pub struct Item__ad_hoc2120<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2120<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 148, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 148);
    }
}

///[increment](GroundControl::OBSTACLE_DISTANCE::increment).

pub struct Item__ad_hoc2122<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2122<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 156, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 156);
    }
}

///[sensor_type](GroundControl::OBSTACLE_DISTANCE::sensor_type).

pub struct Item__ad_hoc282<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc282<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_DISTANCE_SENSOR> {
        let src = &mut self.0;
        if src.base.field_bit != 1256 && !src.set_field(1256, -1) { return None; }

        Some({ packs::MAV_DISTANCE_SENSOR::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_DISTANCE_SENSOR) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1256
        {
            dst.set_field(1256, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metagps2_raw(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metagps2_raw {}

pub static mut GPS2_RAW: Metagps2_raw = Metagps2_raw(213, 4, 1, 1, 35, None, 0, 272, 1, [(&fld__ad_hoc972 as *const _ad_hoc972) as *const sys::Field]);

///[eph](GroundControl::GPS2_RAW::eph).

pub struct Item__ad_hoc1693<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1693<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[epv](GroundControl::GPS2_RAW::epv).

pub struct Item__ad_hoc1694<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1694<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[vel](GroundControl::GPS2_RAW::vel).

pub struct Item__ad_hoc1695<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1695<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 4);
    }
}

///[cog](GroundControl::GPS2_RAW::cog).

pub struct Item__ad_hoc1696<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1696<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 6);
    }
}

///[dgps_age](GroundControl::GPS2_RAW::dgps_age).

pub struct Item__ad_hoc1699<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1699<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 8, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 8);
    }
}

///[time_usec](GroundControl::GPS2_RAW::time_usec).

pub struct Item__ad_hoc1689<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1689<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 12, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 12);
    }
}

///[lat](GroundControl::GPS2_RAW::lat).

pub struct Item__ad_hoc1690<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1690<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 20, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 20);
    }
}

///[lon](GroundControl::GPS2_RAW::lon).

pub struct Item__ad_hoc1691<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1691<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 24, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 24);
    }
}

///[alt](GroundControl::GPS2_RAW::alt).

pub struct Item__ad_hoc1692<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1692<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 28, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 28);
    }
}

///[satellites_visible](GroundControl::GPS2_RAW::satellites_visible).

pub struct Item__ad_hoc1697<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1697<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 32, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 32);
    }
}

///[dgps_numch](GroundControl::GPS2_RAW::dgps_numch).

pub struct Item__ad_hoc1698<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1698<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 33, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 33);
    }
}

///[fix_type](GroundControl::GPS2_RAW::fix_type).

pub struct Item__ad_hoc972<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc972<'a> {
    pub fn get(&mut self) -> Option<packs::GPS_FIX_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 272 && !src.set_field(272, -1) { return None; }

        Some({ packs::GPS_FIX_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GPS_FIX_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 272
        {
            dst.set_field(272, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metarequest_data_stream(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                   u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metarequest_data_stream {}

pub static mut REQUEST_DATA_STREAM: Metarequest_data_stream = Metarequest_data_stream(98, 1, 0, 0, 6, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metamemory_vect(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamemory_vect {}

pub static mut MEMORY_VECT: Metamemory_vect = Metamemory_vect(30, 1, 0, 0, 36, None, 0, 0, 0, []);

///[address](GroundControl::MEMORY_VECT::address).

pub struct Item__ad_hoc1976(pub *mut u8);

impl Item__ad_hoc1976
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[ver](GroundControl::MEMORY_VECT::ver).

pub struct Item__ad_hoc1977(pub *mut u8);

impl Item__ad_hoc1977
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[typE](GroundControl::MEMORY_VECT::typE).

pub struct Item__ad_hoc1978(pub *mut u8);

impl Item__ad_hoc1978
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

///[value](GroundControl::MEMORY_VECT::value).

pub struct ItemArray__ad_hoc1979 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1979 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1979 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaparam_ext_request_read(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                      u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaparam_ext_request_read {}

pub static mut PARAM_EXT_REQUEST_READ: Metaparam_ext_request_read = Metaparam_ext_request_read(90, 0, 0, 0, 5, None, 2, 34, 1, [(&fld__ad_hoc2106 as *const _ad_hoc2106) as *const sys::Field]);

///[target_system](GroundControl::PARAM_EXT_REQUEST_READ::target_system).

pub struct Item__ad_hoc2104<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2104<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[target_component](GroundControl::PARAM_EXT_REQUEST_READ::target_component).

pub struct Item__ad_hoc2105<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2105<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[param_index](GroundControl::PARAM_EXT_REQUEST_READ::param_index).

pub struct Item__ad_hoc2107<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2107<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[param_id](GroundControl::PARAM_EXT_REQUEST_READ::param_id).

pub struct Item__ad_hoc2106<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2106<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 34 && !src.set_field(34, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(34, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metahil_controls(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metahil_controls {}

pub static mut HIL_CONTROLS: Metahil_controls = Metahil_controls(82, 0, 0, 1, 42, None, 0, 328, 1, [(&fld__ad_hoc353 as *const _ad_hoc353) as *const sys::Field]);

#[repr(C)]
pub struct Metahil_sensor(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                          u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metahil_sensor {}

pub static mut HIL_SENSOR: Metahil_sensor = Metahil_sensor(169, 0, 1, 1, 64, None, 0, 0, 0, []);

///[fields_updated](GroundControl::HIL_SENSOR::fields_updated).

pub struct Item__ad_hoc1575(pub *mut u8);

impl Item__ad_hoc1575
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[time_usec](GroundControl::HIL_SENSOR::time_usec).

pub struct Item__ad_hoc1561(pub *mut u8);

impl Item__ad_hoc1561
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 4);
    }
}

///[xacc](GroundControl::HIL_SENSOR::xacc).

pub struct Item__ad_hoc1562(pub *mut u8);

impl Item__ad_hoc1562
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[yacc](GroundControl::HIL_SENSOR::yacc).

pub struct Item__ad_hoc1563(pub *mut u8);

impl Item__ad_hoc1563
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[zacc](GroundControl::HIL_SENSOR::zacc).

pub struct Item__ad_hoc1564(pub *mut u8);

impl Item__ad_hoc1564
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[xgyro](GroundControl::HIL_SENSOR::xgyro).

pub struct Item__ad_hoc1565(pub *mut u8);

impl Item__ad_hoc1565
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[ygyro](GroundControl::HIL_SENSOR::ygyro).

pub struct Item__ad_hoc1566(pub *mut u8);

impl Item__ad_hoc1566
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[zgyro](GroundControl::HIL_SENSOR::zgyro).

pub struct Item__ad_hoc1567(pub *mut u8);

impl Item__ad_hoc1567
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

///[xmag](GroundControl::HIL_SENSOR::xmag).

pub struct Item__ad_hoc1568(pub *mut u8);

impl Item__ad_hoc1568
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 36); }
}

///[ymag](GroundControl::HIL_SENSOR::ymag).

pub struct Item__ad_hoc1569(pub *mut u8);

impl Item__ad_hoc1569
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 40); }
}

///[zmag](GroundControl::HIL_SENSOR::zmag).

pub struct Item__ad_hoc1570(pub *mut u8);

impl Item__ad_hoc1570
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 44, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 44); }
}

///[abs_pressure](GroundControl::HIL_SENSOR::abs_pressure).

pub struct Item__ad_hoc1571(pub *mut u8);

impl Item__ad_hoc1571
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 48, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 48); }
}

///[diff_pressure](GroundControl::HIL_SENSOR::diff_pressure).

pub struct Item__ad_hoc1572(pub *mut u8);

impl Item__ad_hoc1572
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 52, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 52); }
}

///[pressure_alt](GroundControl::HIL_SENSOR::pressure_alt).

pub struct Item__ad_hoc1573(pub *mut u8);

impl Item__ad_hoc1573
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 56, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 56); }
}

///[temperature](GroundControl::HIL_SENSOR::temperature).

pub struct Item__ad_hoc1574(pub *mut u8);

impl Item__ad_hoc1574
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 60, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 60); }
}

#[repr(C)]
pub struct Metasetup_signing(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metasetup_signing {}

pub static mut SETUP_SIGNING: Metasetup_signing = Metasetup_signing(205, 0, 0, 1, 42, None, 0, 0, 0, []);

///[initial_timestamp](GroundControl::SETUP_SIGNING::initial_timestamp).

pub struct Item__ad_hoc1998(pub *mut u8);

impl Item__ad_hoc1998
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[target_system](GroundControl::SETUP_SIGNING::target_system).

pub struct Item__ad_hoc1995(pub *mut u8);

impl Item__ad_hoc1995
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

///[target_component](GroundControl::SETUP_SIGNING::target_component).

pub struct Item__ad_hoc1996(pub *mut u8);

impl Item__ad_hoc1996
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 9, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 9);
    }
}

///[secret_key](GroundControl::SETUP_SIGNING::secret_key).

pub struct ItemArray__ad_hoc1997 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1997 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1997 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metagps_rtk(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                       u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metagps_rtk {}

pub static mut GPS_RTK: Metagps_rtk = Metagps_rtk(89, 1, 3, 0, 35, None, 0, 0, 0, []);

///[wn](GroundControl::GPS_RTK::wn).

pub struct Item__ad_hoc1708(pub *mut u8);

impl Item__ad_hoc1708
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[time_last_baseline_ms](GroundControl::GPS_RTK::time_last_baseline_ms).

pub struct Item__ad_hoc1706(pub *mut u8);

impl Item__ad_hoc1706
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 2);
    }
}

///[tow](GroundControl::GPS_RTK::tow).

pub struct Item__ad_hoc1709(pub *mut u8);

impl Item__ad_hoc1709
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[accuracy](GroundControl::GPS_RTK::accuracy).

pub struct Item__ad_hoc1717(pub *mut u8);

impl Item__ad_hoc1717
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 10);
    }
}

///[rtk_receiver_id](GroundControl::GPS_RTK::rtk_receiver_id).

pub struct Item__ad_hoc1707(pub *mut u8);

impl Item__ad_hoc1707
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 14, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 14);
    }
}

///[rtk_health](GroundControl::GPS_RTK::rtk_health).

pub struct Item__ad_hoc1710(pub *mut u8);

impl Item__ad_hoc1710
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 15, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 15);
    }
}

///[rtk_rate](GroundControl::GPS_RTK::rtk_rate).

pub struct Item__ad_hoc1711(pub *mut u8);

impl Item__ad_hoc1711
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 16);
    }
}

///[nsats](GroundControl::GPS_RTK::nsats).

pub struct Item__ad_hoc1712(pub *mut u8);

impl Item__ad_hoc1712
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 17, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 17);
    }
}

///[baseline_coords_type](GroundControl::GPS_RTK::baseline_coords_type).

pub struct Item__ad_hoc1713(pub *mut u8);

impl Item__ad_hoc1713
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 18, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 18);
    }
}

///[baseline_a_mm](GroundControl::GPS_RTK::baseline_a_mm).

pub struct Item__ad_hoc1714(pub *mut u8);

impl Item__ad_hoc1714
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 19, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 19);
    }
}

///[baseline_b_mm](GroundControl::GPS_RTK::baseline_b_mm).

pub struct Item__ad_hoc1715(pub *mut u8);

impl Item__ad_hoc1715
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 23, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 23);
    }
}

///[baseline_c_mm](GroundControl::GPS_RTK::baseline_c_mm).

pub struct Item__ad_hoc1716(pub *mut u8);

impl Item__ad_hoc1716
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 27, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 27);
    }
}

///[iar_num_hypotheses](GroundControl::GPS_RTK::iar_num_hypotheses).

pub struct Item__ad_hoc1718(pub *mut u8);

impl Item__ad_hoc1718
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 31, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 31);
    }
}

#[repr(C)]
pub struct Metaparam_request_list(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaparam_request_list {}

pub static mut PARAM_REQUEST_LIST: Metaparam_request_list = Metaparam_request_list(47, 0, 0, 0, 2, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metauavionix_adsb_out_cfg(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 6]);

unsafe impl std::marker::Sync for Metauavionix_adsb_out_cfg {}

pub static mut UAVIONIX_ADSB_OUT_CFG: Metauavionix_adsb_out_cfg = Metauavionix_adsb_out_cfg(84, 1, 1, 0, 7, None, 3, 51, 6, [(&fld__ad_hoc2126 as *const _ad_hoc2126) as *const sys::Field,
    (&fld__ad_hoc209 as *const _ad_hoc209) as *const sys::Field,
    (&fld__ad_hoc212 as *const _ad_hoc212) as *const sys::Field,
    (&fld__ad_hoc210 as *const _ad_hoc210) as *const sys::Field,
    (&fld__ad_hoc208 as *const _ad_hoc208) as *const sys::Field,
    (&fld__ad_hoc211 as *const _ad_hoc211) as *const sys::Field]);

///[stallSpeed](GroundControl::UAVIONIX_ADSB_OUT_CFG::stallSpeed).

pub struct Item__ad_hoc2127<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2127<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[ICAO](GroundControl::UAVIONIX_ADSB_OUT_CFG::ICAO).

pub struct Item__ad_hoc2125<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2125<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 2);
    }
}

///[callsign](GroundControl::UAVIONIX_ADSB_OUT_CFG::callsign).

pub struct Item__ad_hoc2126<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2126<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 51 && !src.set_field(51, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(51, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[emitterType](GroundControl::UAVIONIX_ADSB_OUT_CFG::emitterType).

pub struct Item__ad_hoc209<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc209<'a> {
    pub fn get(&mut self) -> Option<packs::ADSB_EMITTER_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 52 && !src.set_field(52, -1) { return None; }

        Some({ packs::ADSB_EMITTER_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 5)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::ADSB_EMITTER_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 52
        {
            dst.set_field(52, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 5, dst.base.bytes, dst.BIT);
    }
}

///[aircraftSize](GroundControl::UAVIONIX_ADSB_OUT_CFG::aircraftSize).

pub struct Item__ad_hoc212<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc212<'a> {
    pub fn get(&mut self) -> Option<packs::UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE> {
        let src = &mut self.0;
        if src.base.field_bit != 53 && !src.set_field(53, -1) { return None; }

        Some({ packs::UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 53
        {
            dst.set_field(53, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

///[gpsOffsetLat](GroundControl::UAVIONIX_ADSB_OUT_CFG::gpsOffsetLat).

pub struct Item__ad_hoc210<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc210<'a> {
    pub fn get(&mut self) -> Option<packs::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT> {
        let src = &mut self.0;
        if src.base.field_bit != 54 && !src.set_field(54, -1) { return None; }

        Some({ packs::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT) {
        let dst = &mut self.0;
        if dst.base.field_bit != 54
        {
            dst.set_field(54, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[gpsOffsetLon](GroundControl::UAVIONIX_ADSB_OUT_CFG::gpsOffsetLon).

pub struct Item__ad_hoc208<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc208<'a> {
    pub fn get(&mut self) -> Option<packs::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON> {
        let src = &mut self.0;
        if src.base.field_bit != 55 && !src.set_field(55, -1) { return None; }

        Some({ packs::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON) {
        let dst = &mut self.0;
        if dst.base.field_bit != 55
        {
            dst.set_field(55, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

///[rfSelect](GroundControl::UAVIONIX_ADSB_OUT_CFG::rfSelect).

pub struct Item__ad_hoc211<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc211<'a> {
    pub fn get(&mut self) -> Option<packs::UAVIONIX_ADSB_OUT_RF_SELECT> {
        let src = &mut self.0;
        if src.base.field_bit != 56 && !src.set_field(56, -1) { return None; }

        Some({ packs::UAVIONIX_ADSB_OUT_RF_SELECT::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVIONIX_ADSB_OUT_RF_SELECT) {
        let dst = &mut self.0;
        if dst.base.field_bit != 56
        {
            dst.set_field(56, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metalanding_target(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 7]);

unsafe impl std::marker::Sync for Metalanding_target {}

pub static mut LANDING_TARGET: Metalanding_target = Metalanding_target(172, 0, 0, 1, 30, None, 3, 235, 7, [(&fld__ad_hoc351 as *const _ad_hoc351) as *const sys::Field,
    (&fld__ad_hoc1861 as *const _ad_hoc1861) as *const sys::Field,
    (&fld__ad_hoc1862 as *const _ad_hoc1862) as *const sys::Field,
    (&fld__ad_hoc1863 as *const _ad_hoc1863) as *const sys::Field,
    (&fld__ad_hoc1864 as *const _ad_hoc1864) as *const sys::Field,
    (&fld__ad_hoc352 as *const _ad_hoc352) as *const sys::Field,
    (&fld__ad_hoc1865 as *const _ad_hoc1865) as *const sys::Field]);

///[time_usec](GroundControl::LANDING_TARGET::time_usec).

pub struct Item__ad_hoc1854<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1854<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 0);
    }
}

///[target_num](GroundControl::LANDING_TARGET::target_num).

pub struct Item__ad_hoc1855<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1855<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 8);
    }
}

///[angle_x](GroundControl::LANDING_TARGET::angle_x).

pub struct Item__ad_hoc1856<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1856<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 9, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 9);
    }
}

///[angle_y](GroundControl::LANDING_TARGET::angle_y).

pub struct Item__ad_hoc1857<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1857<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 13, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 13);
    }
}

///[distance](GroundControl::LANDING_TARGET::distance).

pub struct Item__ad_hoc1858<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1858<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 17, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 17);
    }
}

///[size_x](GroundControl::LANDING_TARGET::size_x).

pub struct Item__ad_hoc1859<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1859<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 21, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 21);
    }
}

///[size_y](GroundControl::LANDING_TARGET::size_y).

pub struct Item__ad_hoc1860<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1860<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 25, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 25);
    }
}

///[frame](GroundControl::LANDING_TARGET::frame).

pub struct Item__ad_hoc351<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc351<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_FRAME> {
        let src = &mut self.0;
        if src.base.field_bit != 235 && !src.set_field(235, -1) { return None; }

        Some({ packs::MAV_FRAME::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_FRAME) {
        let dst = &mut self.0;
        if dst.base.field_bit != 235
        {
            dst.set_field(235, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

///[x](GroundControl::LANDING_TARGET::x).

pub struct Item__ad_hoc1861<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1861<'a> {
    pub fn get(&mut self) -> Option<f32> {
        let src = &mut self.0;
        if src.base.field_bit != 236 && !src.set_field(236, -1) { return None; }

        Some({ f32::from_bits(sys::get_bytes(src.base.bytes, src.BYTE, 4usize) as u32) })
    }

    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        if dst.base.field_bit != 236
        {
            dst.set_field(236, 0i32);
        }


        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, dst.BYTE);
    }
}

///[y](GroundControl::LANDING_TARGET::y).

pub struct Item__ad_hoc1862<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1862<'a> {
    pub fn get(&mut self) -> Option<f32> {
        let src = &mut self.0;
        if src.base.field_bit != 237 && !src.set_field(237, -1) { return None; }

        Some({ f32::from_bits(sys::get_bytes(src.base.bytes, src.BYTE, 4usize) as u32) })
    }

    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        if dst.base.field_bit != 237
        {
            dst.set_field(237, 0i32);
        }


        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, dst.BYTE);
    }
}

///[z](GroundControl::LANDING_TARGET::z).

pub struct Item__ad_hoc1863<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1863<'a> {
    pub fn get(&mut self) -> Option<f32> {
        let src = &mut self.0;
        if src.base.field_bit != 238 && !src.set_field(238, -1) { return None; }

        Some({ f32::from_bits(sys::get_bytes(src.base.bytes, src.BYTE, 4usize) as u32) })
    }

    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        if dst.base.field_bit != 238
        {
            dst.set_field(238, 0i32);
        }


        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, dst.BYTE);
    }
}

///[q](GroundControl::LANDING_TARGET::q).

pub struct Item__ad_hoc1864<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1864<'a>
{
    pub fn set(&mut self, src: f32, d0: usize) {
        if self.0.base.field_bit != 239 { self.0.set_field(239, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0, -1) { dst.set_item(d0, 0); }
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize) -> Option<f32> {
        let src = &mut self.0;
        if (src.base.field_bit != 239 && !src.set_field(239, -1)) || !src.set_item(d0, -1) { return None; }

        Some({ f32::from_bits(sys::get_bytes(src.base.bytes, src.BYTE, 4usize) as u32) })
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize)>(&mut self, from_d0: usize, mut dst: DST) {
        for d0 in from_d0..packs::LANDING_TARGET::q::d0 {
            dst(self, d0);
        }
    }
}

pub struct FieldI__ad_hoc1864<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1864<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1864<'a>> {
        return if self.0.base.field_bit != 239 && !self.0.set_field(239, -1) { None } else { Some(Item__ad_hoc1864(self.0)) };
    }

    pub fn set(&mut self, src: f32, d0: usize) {
        if self.0.base.field_bit != 239 { self.0.set_field(239, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0, -1) { dst.set_item(d0, 0); }
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, dst.BYTE);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize)>(&mut self, from_d0: usize, mut dst: DST) {
        for d0 in from_d0..packs::LANDING_TARGET::q::d0 {
            dst(self, d0);
        }
    }
}

///[typE](GroundControl::LANDING_TARGET::typE).

pub struct Item__ad_hoc352<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc352<'a> {
    pub fn get(&mut self) -> Option<packs::LANDING_TARGET_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 240 && !src.set_field(240, -1) { return None; }

        Some({ packs::LANDING_TARGET_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::LANDING_TARGET_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 240
        {
            dst.set_field(240, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

///[position_valid](GroundControl::LANDING_TARGET::position_valid).

pub struct Item__ad_hoc1865<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1865<'a> {
    pub fn get(&mut self) -> Option<i8> {
        let src = &mut self.0;
        if src.base.field_bit != 241 && !src.set_field(241, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 1 as usize) as i8;
            (dst) as i8
        })
    }

    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        if dst.base.field_bit != 241
        {
            dst.set_field(241, 0i32);
        }


        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, dst.BYTE);
    }
}

#[repr(C)]
pub struct Metaset_actuator_control_target(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaset_actuator_control_target {}

pub static mut SET_ACTUATOR_CONTROL_TARGET: Metaset_actuator_control_target = Metaset_actuator_control_target(94, 0, 0, 1, 43, None, 0, 0, 0, []);

///[time_usec](GroundControl::SET_ACTUATOR_CONTROL_TARGET::time_usec).

pub struct Item__ad_hoc1784(pub *mut u8);

impl Item__ad_hoc1784
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[group_mlx](GroundControl::SET_ACTUATOR_CONTROL_TARGET::group_mlx).

pub struct Item__ad_hoc1785(pub *mut u8);

impl Item__ad_hoc1785
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

///[target_system](GroundControl::SET_ACTUATOR_CONTROL_TARGET::target_system).

pub struct Item__ad_hoc1786(pub *mut u8);

impl Item__ad_hoc1786
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 9, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 9);
    }
}

///[target_component](GroundControl::SET_ACTUATOR_CONTROL_TARGET::target_component).

pub struct Item__ad_hoc1787(pub *mut u8);

impl Item__ad_hoc1787
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 10);
    }
}

///[controls](GroundControl::SET_ACTUATOR_CONTROL_TARGET::controls).

pub struct ItemArray__ad_hoc1788 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1788 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1788 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metacontrol_system_state(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                    u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metacontrol_system_state {}

pub static mut CONTROL_SYSTEM_STATE: Metacontrol_system_state = Metacontrol_system_state(207, 0, 0, 1, 100, None, 0, 0, 0, []);

///[time_usec](GroundControl::CONTROL_SYSTEM_STATE::time_usec).

pub struct Item__ad_hoc1819(pub *mut u8);

impl Item__ad_hoc1819
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[x_acc](GroundControl::CONTROL_SYSTEM_STATE::x_acc).

pub struct Item__ad_hoc1820(pub *mut u8);

impl Item__ad_hoc1820
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[y_acc](GroundControl::CONTROL_SYSTEM_STATE::y_acc).

pub struct Item__ad_hoc1821(pub *mut u8);

impl Item__ad_hoc1821
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[z_acc](GroundControl::CONTROL_SYSTEM_STATE::z_acc).

pub struct Item__ad_hoc1822(pub *mut u8);

impl Item__ad_hoc1822
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[x_vel](GroundControl::CONTROL_SYSTEM_STATE::x_vel).

pub struct Item__ad_hoc1823(pub *mut u8);

impl Item__ad_hoc1823
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[y_vel](GroundControl::CONTROL_SYSTEM_STATE::y_vel).

pub struct Item__ad_hoc1824(pub *mut u8);

impl Item__ad_hoc1824
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[z_vel](GroundControl::CONTROL_SYSTEM_STATE::z_vel).

pub struct Item__ad_hoc1825(pub *mut u8);

impl Item__ad_hoc1825
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[x_pos](GroundControl::CONTROL_SYSTEM_STATE::x_pos).

pub struct Item__ad_hoc1826(pub *mut u8);

impl Item__ad_hoc1826
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

///[y_pos](GroundControl::CONTROL_SYSTEM_STATE::y_pos).

pub struct Item__ad_hoc1827(pub *mut u8);

impl Item__ad_hoc1827
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 36); }
}

///[z_pos](GroundControl::CONTROL_SYSTEM_STATE::z_pos).

pub struct Item__ad_hoc1828(pub *mut u8);

impl Item__ad_hoc1828
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 40); }
}

///[airspeed](GroundControl::CONTROL_SYSTEM_STATE::airspeed).

pub struct Item__ad_hoc1829(pub *mut u8);

impl Item__ad_hoc1829
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 44, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 44); }
}

///[vel_variance](GroundControl::CONTROL_SYSTEM_STATE::vel_variance).

pub struct ItemArray__ad_hoc1830 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1830 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1830 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[pos_variance](GroundControl::CONTROL_SYSTEM_STATE::pos_variance).

pub struct ItemArray__ad_hoc1831 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1831 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1831 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[q](GroundControl::CONTROL_SYSTEM_STATE::q).

pub struct ItemArray__ad_hoc1832 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1832 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1832 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[roll_rate](GroundControl::CONTROL_SYSTEM_STATE::roll_rate).

pub struct Item__ad_hoc1833(pub *mut u8);

impl Item__ad_hoc1833
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 88, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 88); }
}

///[pitch_rate](GroundControl::CONTROL_SYSTEM_STATE::pitch_rate).

pub struct Item__ad_hoc1834(pub *mut u8);

impl Item__ad_hoc1834
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 92, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 92); }
}

///[yaw_rate](GroundControl::CONTROL_SYSTEM_STATE::yaw_rate).

pub struct Item__ad_hoc1835(pub *mut u8);

impl Item__ad_hoc1835
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 96, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 96); }
}

#[repr(C)]
pub struct Metaset_position_target_global_int(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                              u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaset_position_target_global_int {}

pub static mut SET_POSITION_TARGET_GLOBAL_INT: Metaset_position_target_global_int = Metaset_position_target_global_int(218, 1, 1, 0, 53, None, 0, 416, 1, [(&fld__ad_hoc957 as *const _ad_hoc957) as *const sys::Field]);

#[repr(C)]
pub struct Metadata32(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                      u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadata32 {}

pub static mut DATA32: Metadata32 = Metadata32(164, 0, 0, 0, 34, None, 0, 0, 0, []);

///[typE](GroundControl::DATA32::typE).

pub struct Item__ad_hoc2254(pub *mut u8);

impl Item__ad_hoc2254
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[len](GroundControl::DATA32::len).

pub struct Item__ad_hoc2255(pub *mut u8);

impl Item__ad_hoc2255
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[daTa](GroundControl::DATA32::daTa).

pub struct ItemArray__ad_hoc2256 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2256 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2256 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaping33(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                      u16, u32, u16, [*const sys::Field; 31]);

unsafe impl std::marker::Sync for Metaping33 {}

pub static mut PING33: Metaping33 = Metaping33(139, 0, 18, 0, 155, None, 8, 1234, 31, [(&fld__ad_hoc1487 as *const _ad_hoc1487) as *const sys::Field,
    (&fld__ad_hoc1488 as *const _ad_hoc1488) as *const sys::Field,
    (&fld__ad_hoc1490 as *const _ad_hoc1490) as *const sys::Field,
    (&fld__ad_hoc1491 as *const _ad_hoc1491) as *const sys::Field,
    (&fld__ad_hoc1492 as *const _ad_hoc1492) as *const sys::Field,
    (&fld__ad_hoc1494 as *const _ad_hoc1494) as *const sys::Field,
    (&fld__ad_hoc1498 as *const _ad_hoc1498) as *const sys::Field,
    (&fld__ad_hoc1499 as *const _ad_hoc1499) as *const sys::Field,
    (&fld__ad_hoc1500 as *const _ad_hoc1500) as *const sys::Field,
    (&fld__ad_hoc1501 as *const _ad_hoc1501) as *const sys::Field,
    (&fld__ad_hoc1502 as *const _ad_hoc1502) as *const sys::Field,
    (&fld__ad_hoc1503 as *const _ad_hoc1503) as *const sys::Field,
    (&fld__ad_hoc1504 as *const _ad_hoc1504) as *const sys::Field,
    (&fld__ad_hoc1505 as *const _ad_hoc1505) as *const sys::Field,
    (&fld__ad_hoc1506 as *const _ad_hoc1506) as *const sys::Field,
    (&fld__ad_hoc1507 as *const _ad_hoc1507) as *const sys::Field,
    (&fld__ad_hoc1508 as *const _ad_hoc1508) as *const sys::Field,
    (&fld__ad_hoc139 as *const _ad_hoc139) as *const sys::Field,
    (&fld__ad_hoc1509 as *const _ad_hoc1509) as *const sys::Field,
    (&fld__ad_hoc1510 as *const _ad_hoc1510) as *const sys::Field,
    (&fld__ad_hoc1511 as *const _ad_hoc1511) as *const sys::Field,
    (&fld__ad_hoc1512 as *const _ad_hoc1512) as *const sys::Field,
    (&fld__ad_hoc1513 as *const _ad_hoc1513) as *const sys::Field,
    (&fld__ad_hoc1515 as *const _ad_hoc1515) as *const sys::Field,
    (&fld__ad_hoc1516 as *const _ad_hoc1516) as *const sys::Field,
    (&fld__ad_hoc1517 as *const _ad_hoc1517) as *const sys::Field,
    (&fld__ad_hoc1518 as *const _ad_hoc1518) as *const sys::Field,
    (&fld__ad_hoc1519 as *const _ad_hoc1519) as *const sys::Field,
    (&fld__ad_hoc1520 as *const _ad_hoc1520) as *const sys::Field,
    (&fld__ad_hoc1521 as *const _ad_hoc1521) as *const sys::Field,
    (&fld__ad_hoc1522 as *const _ad_hoc1522) as *const sys::Field]);

///[TTTT](GroundControl::PING33::TTTT).

pub struct Item__ad_hoc1493<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1493<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0 + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i32 {
        let src = &mut self.0;

        let dst = sys::get_bytes(src.base.bytes, 0 + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4 as usize) as i32;
        (dst) as i32
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::TTTT::d0 {
            for d1 in from_d1..packs::PING33::TTTT::d1 {
                for d2 in from_d2..packs::PING33::TTTT::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[field](GroundControl::PING33::field).

pub struct Item__ad_hoc1489<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1489<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 72, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 72);
    }
}

///[bit_field](GroundControl::PING33::bit_field).

pub struct Item__ad_hoc1497<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1497<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 80, 1 as usize) as i8;
        (4 + dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        if src < 4 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bytes((src - 4) as u64, 1 as usize, dst.base.bytes, 80);
    }
}

///[field6](GroundControl::PING33::field6).

pub struct Item__ad_hoc1514<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1514<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 81 + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i32 {
        let src = &mut self.0;

        let dst = sys::get_bytes(src.base.bytes, 81 + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4 as usize) as i32;
        (dst) as i32
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field6::d0 {
            for d1 in from_d1..packs::PING33::field6::d1 {
                for d2 in from_d2..packs::PING33::field6::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[testBOOL2](GroundControl::PING33::testBOOL2).

pub struct Item__ad_hoc1495<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1495<'a>
{
    pub fn get(&mut self) -> bool {
        let src = &mut self.0;
        sys::get_bit(src.base.bytes, 1224 as usize)
    }
    pub fn set(&mut self, src: bool) {
        let dst = &mut self.0;
        sys::set_bit(src, 1224 as usize, dst.base.bytes);
    }
}

///[testBOOL3](GroundControl::PING33::testBOOL3).

pub struct Item__ad_hoc1496<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1496<'a>
{
    pub fn get(&mut self) -> bool {
        let src = &mut self.0;
        sys::get_bit(src.base.bytes, 1225 as usize)
    }
    pub fn set(&mut self, src: bool) {
        let dst = &mut self.0;
        sys::set_bit(src, 1225 as usize, dst.base.bytes);
    }
}

///[testBOOL](GroundControl::PING33::testBOOL).

pub struct Item__ad_hoc1487<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1487<'a> {
    pub fn get(&mut self) -> Option<bool> {
        let src = &mut self.0;
        if src.base.field_bit != 1234 && !src.set_field(1234, -1) { return None; }

        Some({ sys::get_bit(src.base.bytes, src.BIT as usize) })
    }

    pub fn set(&mut self, src: bool) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1234
        {
            dst.set_field(1234, 0i32);
        }


        sys::set_bit(src, dst.BIT as usize, dst.base.bytes);
    }
}

///[seq](GroundControl::PING33::seq).

pub struct Item__ad_hoc1488<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1488<'a> {
    pub fn get(&mut self) -> Option<i64> {
        let src = &mut self.0;
        if src.base.field_bit != 1235 && !src.set_field(1235, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as u32;
            (dst - 14) as i64
        })
    }

    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1235
        {
            dst.set_field(1235, 0i32);
        }


        sys::set_bytes((src + 14) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }
}

pub struct FieldI__ad_hoc1490<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1490<'a> {
    pub fn set_field(self, d0: usize) -> Item__ad_hoc1490<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1236, 0, d0); }
        Item__ad_hoc1490(dst)
    }
}

///[field1](GroundControl::PING33::field1).

pub struct Item__ad_hoc1490<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1490<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE + (d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 2) * 4);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i32 {
        let src = &mut self.0;

        let dst = sys::get_bytes(src.base.bytes, src.BYTE + (d0 + d1 * src.var_dims[0] as usize + d2 * src.var_dims[0] as usize * 2) * 4, 4 as usize) as i32;
        (dst) as i32
    }

    pub fn d0(&self) -> usize { self.0.D[0] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..self.d0() {
            for d1 in from_d1..packs::PING33::field1::d1 {
                for d2 in from_d2..packs::PING33::field1::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[field12](GroundControl::PING33::field12).

pub struct Item__ad_hoc1491<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1491<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1237 { self.0.set_field(1237, 0); }
        let dst = &mut self.0;


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i32 {
        let src = &mut self.0;

        let dst = sys::get_bytes(src.base.bytes, src.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4 as usize) as i32;
        (dst) as i32
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field12::d0 {
            for d1 in from_d1..packs::PING33::field12::d1 {
                for d2 in from_d2..packs::PING33::field12::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1491<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1491<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1491<'a>> {
        return if self.0.base.field_bit != 1237 && !self.0.set_field(1237, -1) { None } else { Some(Item__ad_hoc1491(self.0)) };
    }
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1237 { self.0.set_field(1237, 0); }
        let dst = &mut self.0;


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field12::d0 {
            for d1 in from_d1..packs::PING33::field12::d1 {
                for d2 in from_d2..packs::PING33::field12::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[field13](GroundControl::PING33::field13).

pub struct Item__ad_hoc1492<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1492<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1238 { self.0.set_field(1238, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i32> {
        let src = &mut self.0;
        if (src.base.field_bit != 1238 && !src.set_field(1238, -1)) || !src.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as i32;
            (dst) as i32
        })
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field13::d0 {
            for d1 in from_d1..packs::PING33::field13::d1 {
                for d2 in from_d2..packs::PING33::field13::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1492<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1492<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1492<'a>> {
        return if self.0.base.field_bit != 1238 && !self.0.set_field(1238, -1) { None } else { Some(Item__ad_hoc1492(self.0)) };
    }

    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1238 { self.0.set_field(1238, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field13::d0 {
            for d1 in from_d1..packs::PING33::field13::d1 {
                for d2 in from_d2..packs::PING33::field13::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[WWWWWWWW](GroundControl::PING33::WWWWWWWW).

pub struct Item__ad_hoc1494<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1494<'a> {
    pub fn get(&mut self) -> Option<i32> {
        let src = &mut self.0;
        if src.base.field_bit != 1239 && !src.set_field(1239, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as i32;
            (dst) as i32
        })
    }

    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1239
        {
            dst.set_field(1239, 0i32);
        }


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }
}

///[bit_field2](GroundControl::PING33::bit_field2).

pub struct Item__ad_hoc1498<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1498<'a> {
    pub fn get(&mut self) -> Option<i8> {
        let src = &mut self.0;
        if src.base.field_bit != 1240 && !src.set_field(1240, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 1 as usize) as i8;
            (45 - dst) as i8
        })
    }

    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1240
        {
            dst.set_field(1240, 0i32);
        }


        if src < 4 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bytes((45 - src) as u64, 1 as usize, dst.base.bytes, dst.BYTE);
    }
}

///[Field_Bits](GroundControl::PING33::Field_Bits).

pub struct Item__ad_hoc1499<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1499<'a>
{
    pub fn set(&mut self, src: i8, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1241 { self.0.set_field(1241, 0); }
        let dst = &mut self.0;

        if src < 4 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bits((src - 4) as u64, 6, dst.base.bytes, dst.BIT + (d0 + d1 * 3 + d2 * 3 * 3) * 6);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i8 {
        let src = &mut self.0;

        (4 + sys::get_bits(src.base.bytes, src.BIT + (d0 + d1 * 3 + d2 * 3 * 3) * 6, 6) as i8) as i8
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::Field_Bits::d0 {
            for d1 in from_d1..packs::PING33::Field_Bits::d1 {
                for d2 in from_d2..packs::PING33::Field_Bits::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1499<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1499<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1499<'a>> {
        return if self.0.base.field_bit != 1241 && !self.0.set_field(1241, -1) { None } else { Some(Item__ad_hoc1499(self.0)) };
    }
    pub fn set(&mut self, src: i8, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1241 { self.0.set_field(1241, 0); }
        let dst = &mut self.0;

        if src < 4 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bits((src - 4) as u64, 6, dst.base.bytes, dst.BIT + (d0 + d1 * 3 + d2 * 3 * 3) * 6);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::Field_Bits::d0 {
            for d1 in from_d1..packs::PING33::Field_Bits::d1 {
                for d2 in from_d2..packs::PING33::Field_Bits::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1500<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1500<'a> {
    pub fn set_field(self, d0: usize) -> Item__ad_hoc1500<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1242, 0, d0); }
        Item__ad_hoc1500(dst)
    }
}

///[SparseFixAllBits](GroundControl::PING33::SparseFixAllBits).

pub struct Item__ad_hoc1500<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1500<'a>
{
    pub fn set(&mut self, src: i8, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3, -1) { dst.set_item(d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3, 0); }
        if src < 4 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bits((src - 4) as u64, 6, dst.base.bytes, dst.BIT);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i8> {
        let src = &mut self.0;
        if !src.set_item(d0 + d1 * src.var_dims[0] as usize + d2 * src.var_dims[0] as usize * 3, -1) { return None; }

        Some({ (4 + sys::get_bits(src.base.bytes, src.BIT, 6) as i8) as i8 })
    }

    pub fn d0(&self) -> usize { self.0.D[0] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..self.d0() {
            for d1 in from_d1..packs::PING33::SparseFixAllBits::d1 {
                for d2 in from_d2..packs::PING33::SparseFixAllBits::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1501<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1501<'a> {
    pub fn set_field(self, d0: usize) -> Item__ad_hoc1501<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1243, 0, d0); }
        Item__ad_hoc1501(dst)
    }
}

///[FixAllBits](GroundControl::PING33::FixAllBits).

pub struct Item__ad_hoc1501<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1501<'a>
{
    pub fn set(&mut self, src: i8, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;

        if src < 14 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bits((src - 14) as u64, 5, dst.base.bytes, dst.BIT + (d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3) * 5);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i8 {
        let src = &mut self.0;

        (14 + sys::get_bits(src.base.bytes, src.BIT + (d0 + d1 * src.var_dims[0] as usize + d2 * src.var_dims[0] as usize * 3) * 5, 5) as i8) as i8
    }

    pub fn d0(&self) -> usize { self.0.D[0] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..self.d0() {
            for d1 in from_d1..packs::PING33::FixAllBits::d1 {
                for d2 in from_d2..packs::PING33::FixAllBits::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1502<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1502<'a> {
    pub fn set_field(self, d0: usize, d2: usize) -> Item__ad_hoc1502<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1244, 0, d0, d2); }
        Item__ad_hoc1502(dst)
    }
}

///[VarAllBits](GroundControl::PING33::VarAllBits).

pub struct Item__ad_hoc1502<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1502<'a>
{
    pub fn set(&mut self, src: i8, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;

        if src < 14 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bits((src - 14) as u64, 5, dst.base.bytes, dst.BIT + (d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3) * 5);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i8 {
        let src = &mut self.0;

        (14 + sys::get_bits(src.base.bytes, src.BIT + (d0 + d1 * src.var_dims[0] as usize + d2 * src.var_dims[0] as usize * 3) * 5, 5) as i8) as i8
    }

    pub fn d0(&self) -> usize { self.0.D[0] as usize }
    pub fn d2(&self) -> usize { self.0.D[2] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..self.d0() {
            for d1 in from_d1..packs::PING33::VarAllBits::d1 {
                for d2 in from_d2..self.d2() {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1503<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1503<'a> {
    pub fn set_field(self, d0: usize, d2: usize) -> Item__ad_hoc1503<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1245, 0, d0, d2); }
        Item__ad_hoc1503(dst)
    }
}

///[SparseVarAllBits](GroundControl::PING33::SparseVarAllBits).

pub struct Item__ad_hoc1503<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1503<'a>
{
    pub fn set(&mut self, src: i8, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3, -1) { dst.set_item(d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3, 0); }
        if src < 14 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bits((src - 14) as u64, 5, dst.base.bytes, dst.BIT);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i8> {
        let src = &mut self.0;
        if !src.set_item(d0 + d1 * src.var_dims[0] as usize + d2 * src.var_dims[0] as usize * 3, -1) { return None; }

        Some({ (14 + sys::get_bits(src.base.bytes, src.BIT, 5) as i8) as i8 })
    }

    pub fn d0(&self) -> usize { self.0.D[0] as usize }
    pub fn d2(&self) -> usize { self.0.D[2] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..self.d0() {
            for d1 in from_d1..packs::PING33::SparseVarAllBits::d1 {
                for d2 in from_d2..self.d2() {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1504<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1504<'a> {
    pub fn set_field(self, d0: usize) -> Item__ad_hoc1504<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1246, 0, d0); }
        Item__ad_hoc1504(dst)
    }
}

///[VarEachBits](GroundControl::PING33::VarEachBits).

pub struct Item__ad_hoc1504<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1504<'a>
{
    pub fn set(&mut self, src: i8, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;

        if src < -14 || 45 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bits((src + 14) as u64, 6, dst.base.bytes, dst.BIT + (d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3) * 6);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i8 {
        let src = &mut self.0;

        (sys::get_bits(src.base.bytes, src.BIT + (d0 + d1 * src.var_dims[0] as usize + d2 * src.var_dims[0] as usize * 3) * 6, 6) as i8 - 14) as i8
    }

    pub fn d0(&self) -> usize { self.0.D[0] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..self.d0() {
            for d1 in from_d1..packs::PING33::VarEachBits::d1 {
                for d2 in from_d2..packs::PING33::VarEachBits::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1505<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1505<'a> {
    pub fn set_field(self, d0: usize) -> Item__ad_hoc1505<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1247, 0, d0); }
        Item__ad_hoc1505(dst)
    }
}

///[SparsVarEachBits](GroundControl::PING33::SparsVarEachBits).

pub struct Item__ad_hoc1505<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1505<'a>
{
    pub fn set(&mut self, src: i16, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3, -1) { dst.set_item(d0 + d1 * dst.var_dims[0] as usize + d2 * dst.var_dims[0] as usize * 3, 0); }
        if src < -14 || 450 < src { panic!("ArgumentOutOfRangeException"); }
        sys::set_bits((src + 14) as u64, 9, dst.base.bytes, dst.BIT);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i16> {
        let src = &mut self.0;
        if !src.set_item(d0 + d1 * src.var_dims[0] as usize + d2 * src.var_dims[0] as usize * 3, -1) { return None; }

        Some({ (sys::get_bits(src.base.bytes, src.BIT, 9) as i16 - 14) as i16 })
    }

    pub fn d0(&self) -> usize { self.0.D[0] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..self.d0() {
            for d1 in from_d1..packs::PING33::SparsVarEachBits::d1 {
                for d2 in from_d2..packs::PING33::SparsVarEachBits::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[testBOOLX](GroundControl::PING33::testBOOLX).

pub struct Item__ad_hoc1506<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1506<'a> {
    pub fn get(&mut self) -> Option<bool> {
        let src = &mut self.0;
        if src.base.field_bit != 1248 && !src.set_field(1248, -1) { return None; }

        Some({ sys::get_bit(src.base.bytes, src.BIT as usize) })
    }

    pub fn set(&mut self, src: bool) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1248
        {
            dst.set_field(1248, 0i32);
        }


        sys::set_bit(src, dst.BIT as usize, dst.base.bytes);
    }
}

///[testBOOL2X](GroundControl::PING33::testBOOL2X).

pub struct Item__ad_hoc1507<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1507<'a> {
    pub fn get(&mut self) -> Option<bool> {
        let src = &mut self.0;
        if src.base.field_bit != 1249 && !src.set_field(1249, -1) { return None; }

        Some({ sys::get_bit(src.base.bytes, src.BIT as usize) })
    }

    pub fn set(&mut self, src: bool) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1249
        {
            dst.set_field(1249, 0i32);
        }


        sys::set_bit(src, dst.BIT as usize, dst.base.bytes);
    }
}

///[testBOOL3X](GroundControl::PING33::testBOOL3X).

pub struct Item__ad_hoc1508<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1508<'a> {
    pub fn get(&mut self) -> Option<bool> {
        let src = &mut self.0;
        if src.base.field_bit != 1250 && !src.set_field(1250, -1) { return None; }

        Some({ sys::get_bit(src.base.bytes, src.BIT as usize) })
    }

    pub fn set(&mut self, src: bool) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1250
        {
            dst.set_field(1250, 0i32);
        }


        sys::set_bit(src, dst.BIT as usize, dst.base.bytes);
    }
}

///[MMMMMM](GroundControl::PING33::MMMMMM).

pub struct Item__ad_hoc139<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc139<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_MODE> {
        let src = &mut self.0;
        if src.base.field_bit != 1251 && !src.set_field(1251, -1) { return None; }

        Some({ packs::MAV_MODE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_MODE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1251
        {
            dst.set_field(1251, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

pub struct FieldI__ad_hoc1509<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1509<'a> {
    pub fn set_field(self, d2: usize) -> Item__ad_hoc1509<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1252, 0, d2); }
        Item__ad_hoc1509(dst)
    }
}

///[field44](GroundControl::PING33::field44).

pub struct Item__ad_hoc1509<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1509<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i32 {
        let src = &mut self.0;

        let dst = sys::get_bytes(src.base.bytes, src.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4 as usize) as i32;
        (dst) as i32
    }

    pub fn d2(&self) -> usize { self.0.D[2] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field44::d0 {
            for d1 in from_d1..packs::PING33::field44::d1 {
                for d2 in from_d2..self.d2() {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1510<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1510<'a> {
    pub fn set_field(self, d2: usize) -> Item__ad_hoc1510<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1253, 0, d2); }
        Item__ad_hoc1510(dst)
    }
}

///[field634](GroundControl::PING33::field634).

pub struct Item__ad_hoc1510<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1510<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i32 {
        let src = &mut self.0;

        let dst = sys::get_bytes(src.base.bytes, src.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4 as usize) as i32;
        (dst) as i32
    }

    pub fn d2(&self) -> usize { self.0.D[2] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field634::d0 {
            for d1 in from_d1..packs::PING33::field634::d1 {
                for d2 in from_d2..self.d2() {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1511<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1511<'a> {
    pub fn set_field(self, d2: usize) -> Item__ad_hoc1511<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1254, 0, d2); }
        Item__ad_hoc1511(dst)
    }
}

///[field33344](GroundControl::PING33::field33344).

pub struct Item__ad_hoc1511<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1511<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i32> {
        let src = &mut self.0;
        if !src.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as i32;
            (dst) as i32
        })
    }

    pub fn d2(&self) -> usize { self.0.D[2] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field33344::d0 {
            for d1 in from_d1..packs::PING33::field33344::d1 {
                for d2 in from_d2..self.d2() {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1512<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1512<'a> {
    pub fn set_field(self, d2: usize) -> Item__ad_hoc1512<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1255, 0, d2); }
        Item__ad_hoc1512(dst)
    }
}

///[field333634](GroundControl::PING33::field333634).

pub struct Item__ad_hoc1512<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1512<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i32> {
        let src = &mut self.0;
        if !src.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as i32;
            (dst) as i32
        })
    }

    pub fn d2(&self) -> usize { self.0.D[2] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field333634::d0 {
            for d1 in from_d1..packs::PING33::field333634::d1 {
                for d2 in from_d2..self.d2() {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[field__](GroundControl::PING33::field__).

pub struct Item__ad_hoc1513<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1513<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1256 { self.0.set_field(1256, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i32> {
        let src = &mut self.0;
        if (src.base.field_bit != 1256 && !src.set_field(1256, -1)) || !src.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as i32;
            (dst) as i32
        })
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field__::d0 {
            for d1 in from_d1..packs::PING33::field__::d1 {
                for d2 in from_d2..packs::PING33::field__::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1513<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1513<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1513<'a>> {
        return if self.0.base.field_bit != 1256 && !self.0.set_field(1256, -1) { None } else { Some(Item__ad_hoc1513(self.0)) };
    }

    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1256 { self.0.set_field(1256, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field__::d0 {
            for d1 in from_d1..packs::PING33::field__::d1 {
                for d2 in from_d2..packs::PING33::field__::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1515<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1515<'a> {
    pub fn set_field(self, d2: usize) -> Item__ad_hoc1515<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1257, 0, d2); }
        Item__ad_hoc1515(dst)
    }
}

///[field63](GroundControl::PING33::field63).

pub struct Item__ad_hoc1515<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1515<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4);
    }

    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> i32 {
        let src = &mut self.0;

        let dst = sys::get_bytes(src.base.bytes, src.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4 as usize) as i32;
        (dst) as i32
    }

    pub fn d2(&self) -> usize { self.0.D[2] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field63::d0 {
            for d1 in from_d1..packs::PING33::field63::d1 {
                for d2 in from_d2..self.d2() {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[uid2](GroundControl::PING33::uid2).

pub struct Item__ad_hoc1516<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1516<'a>
{
    pub fn set(&mut self, src: i8, d0: usize) {
        if self.0.base.field_bit != 1258 { self.0.set_field(1258, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0, -1) { dst.set_item(d0, 0); }

        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize) -> Option<i8> {
        let src = &mut self.0;
        if (src.base.field_bit != 1258 && !src.set_field(1258, -1)) || !src.set_item(d0, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 1 as usize) as i8;
            (dst) as i8
        })
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize)>(&mut self, from_d0: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::uid2::d0 {
            dst(self, d0);
        }
    }
}

pub struct FieldI__ad_hoc1516<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1516<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1516<'a>> {
        return if self.0.base.field_bit != 1258 && !self.0.set_field(1258, -1) { None } else { Some(Item__ad_hoc1516(self.0)) };
    }

    pub fn set(&mut self, src: i8, d0: usize) {
        if self.0.base.field_bit != 1258 { self.0.set_field(1258, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0, -1) { dst.set_item(d0, 0); }

        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, dst.BYTE);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize)>(&mut self, from_d0: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::uid2::d0 {
            dst(self, d0);
        }
    }
}

///[field2](GroundControl::PING33::field2).

pub struct Item__ad_hoc1517<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1517<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1259 { self.0.set_field(1259, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i32> {
        let src = &mut self.0;
        if (src.base.field_bit != 1259 && !src.set_field(1259, -1)) || !src.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as i32;
            (dst) as i32
        })
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field2::d0 {
            for d1 in from_d1..packs::PING33::field2::d1 {
                for d2 in from_d2..packs::PING33::field2::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1517<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1517<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1517<'a>> {
        return if self.0.base.field_bit != 1259 && !self.0.set_field(1259, -1) { None } else { Some(Item__ad_hoc1517(self.0)) };
    }

    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1259 { self.0.set_field(1259, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field2::d0 {
            for d1 in from_d1..packs::PING33::field2::d1 {
                for d2 in from_d2..packs::PING33::field2::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[field4](GroundControl::PING33::field4).

pub struct Item__ad_hoc1518<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1518<'a>
{
    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1260 { self.0.set_field(1260, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<i32> {
        let src = &mut self.0;
        if (src.base.field_bit != 1260 && !src.set_field(1260, -1)) || !src.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as i32;
            (dst) as i32
        })
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field4::d0 {
            for d1 in from_d1..packs::PING33::field4::d1 {
                for d2 in from_d2..packs::PING33::field4::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

pub struct FieldI__ad_hoc1518<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1518<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1518<'a>> {
        return if self.0.base.field_bit != 1260 && !self.0.set_field(1260, -1) { None } else { Some(Item__ad_hoc1518(self.0)) };
    }

    pub fn set(&mut self, src: i32, d0: usize, d1: usize, d2: usize) {
        if self.0.base.field_bit != 1260 { self.0.set_field(1260, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0); }

        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::field4::d0 {
            for d1 in from_d1..packs::PING33::field4::d1 {
                for d2 in from_d2..packs::PING33::field4::d2 {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[stringtest1](GroundControl::PING33::stringtest1).

pub struct Item__ad_hoc1519<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1519<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 1261 && !src.set_field(1261, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(1261, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

pub struct FieldI__ad_hoc1520<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1520<'a> {
    pub fn set_field(self, d2: usize) -> Item__ad_hoc1520<'a> {
        let dst = self.0;
        unsafe { sys::set_field(dst as *mut sys::Cursor, 1262, 0, d2); }
        Item__ad_hoc1520(dst)
    }
}

///[stringtest2](GroundControl::PING33::stringtest2).

pub struct Item__ad_hoc1520<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1520<'a>
{
    pub fn set(&mut self, src: &str, d0: usize, d1: usize, d2: usize) {
        let dst = &mut self.0;

        dst.set_item(d0 + d1 * 3 + d2 * 3 * 2, src.len() as i32);

        dst.set_str(src);
    }


    pub fn get(&mut self, d0: usize, d1: usize, d2: usize) -> Option<&str> {
        let src = &mut self.0;
        if !src.set_item(d0 + d1 * 3 + d2 * 3 * 2, -1) { return None; }
        Some(src.get_str())
    }

    pub fn d2(&self) -> usize { self.0.D[2] as usize }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize, /*d1:*/usize, /*d2:*/usize)>(&mut self, from_d0: usize, from_d1: usize, from_d2: usize, mut dst: DST) {
        for d0 in from_d0..packs::PING33::stringtest2::d0 {
            for d1 in from_d1..packs::PING33::stringtest2::d1 {
                for d2 in from_d2..self.d2() {
                    dst(self, d0, d1, d2);
                }
            }
        }
    }
}

///[stringtest3](GroundControl::PING33::stringtest3).

pub struct Item__ad_hoc1521<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1521<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 1263 && !src.set_field(1263, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(1263, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[stringtest4](GroundControl::PING33::stringtest4).

pub struct Item__ad_hoc1522<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1522<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 1264 && !src.set_field(1264, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(1264, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metavfr_hud(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                       u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metavfr_hud {}

pub static mut VFR_HUD: Metavfr_hud = Metavfr_hud(15, 1, 0, 0, 20, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metarally_point(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metarally_point {}

pub static mut RALLY_POINT: Metarally_point = Metarally_point(197, 1, 0, 0, 19, None, 0, 144, 1, [(&fld__ad_hoc645 as *const _ad_hoc645) as *const sys::Field]);

///[land_dir](GroundControl::RALLY_POINT::land_dir).

pub struct Item__ad_hoc2285<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2285<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[target_system](GroundControl::RALLY_POINT::target_system).

pub struct Item__ad_hoc2277<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2277<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 2);
    }
}

///[target_component](GroundControl::RALLY_POINT::target_component).

pub struct Item__ad_hoc2278<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2278<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 3);
    }
}

///[idx](GroundControl::RALLY_POINT::idx).

pub struct Item__ad_hoc2279<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2279<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 4);
    }
}

///[count](GroundControl::RALLY_POINT::count).

pub struct Item__ad_hoc2280<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2280<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 5);
    }
}

///[lat](GroundControl::RALLY_POINT::lat).

pub struct Item__ad_hoc2281<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2281<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 6);
    }
}

///[lng](GroundControl::RALLY_POINT::lng).

pub struct Item__ad_hoc2282<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2282<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 10);
    }
}

///[alt](GroundControl::RALLY_POINT::alt).

pub struct Item__ad_hoc2283<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2283<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 14, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 14);
    }
}

///[break_alt](GroundControl::RALLY_POINT::break_alt).

pub struct Item__ad_hoc2284<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2284<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 16, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 16);
    }
}

///[flags](GroundControl::RALLY_POINT::flags).

pub struct Item__ad_hoc645<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc645<'a> {
    pub fn get(&mut self) -> Option<packs::RALLY_FLAGS> {
        let src = &mut self.0;
        if src.base.field_bit != 144 && !src.set_field(144, -1) { return None; }

        Some({ packs::RALLY_FLAGS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::RALLY_FLAGS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 144
        {
            dst.set_field(144, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metamission_set_current(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                   u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamission_set_current {}

pub static mut MISSION_SET_CURRENT: Metamission_set_current = Metamission_set_current(212, 1, 0, 0, 4, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metaadap_tuning(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaadap_tuning {}

pub static mut ADAP_TUNING: Metaadap_tuning = Metaadap_tuning(115, 0, 0, 0, 49, None, 0, 384, 1, [(&fld__ad_hoc963 as *const _ad_hoc963) as *const sys::Field]);

///[desired](GroundControl::ADAP_TUNING::desired).

pub struct Item__ad_hoc2433<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2433<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 0);
    }
}

///[achieved](GroundControl::ADAP_TUNING::achieved).

pub struct Item__ad_hoc2434<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2434<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 4);
    }
}

///[error](GroundControl::ADAP_TUNING::error).

pub struct Item__ad_hoc2435<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2435<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 8);
    }
}

///[theta](GroundControl::ADAP_TUNING::theta).

pub struct Item__ad_hoc2436<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2436<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[omega](GroundControl::ADAP_TUNING::omega).

pub struct Item__ad_hoc2437<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2437<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 16);
    }
}

///[sigma](GroundControl::ADAP_TUNING::sigma).

pub struct Item__ad_hoc2438<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2438<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 20);
    }
}

///[theta_dot](GroundControl::ADAP_TUNING::theta_dot).

pub struct Item__ad_hoc2439<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2439<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 24);
    }
}

///[omega_dot](GroundControl::ADAP_TUNING::omega_dot).

pub struct Item__ad_hoc2440<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2440<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 28);
    }
}

///[sigma_dot](GroundControl::ADAP_TUNING::sigma_dot).

pub struct Item__ad_hoc2441<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2441<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 32);
    }
}

///[f](GroundControl::ADAP_TUNING::f).

pub struct Item__ad_hoc2442<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2442<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 36);
    }
}

///[f_dot](GroundControl::ADAP_TUNING::f_dot).

pub struct Item__ad_hoc2443<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2443<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 40);
    }
}

///[u](GroundControl::ADAP_TUNING::u).

pub struct Item__ad_hoc2444<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2444<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 44, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 44);
    }
}

///[axis](GroundControl::ADAP_TUNING::axis).

pub struct Item__ad_hoc963<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc963<'a> {
    pub fn get(&mut self) -> Option<packs::PID_TUNING_AXIS> {
        let src = &mut self.0;
        if src.base.field_bit != 384 && !src.set_field(384, -1) { return None; }

        Some({ packs::PID_TUNING_AXIS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::PID_TUNING_AXIS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 384
        {
            dst.set_field(384, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metavibration(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metavibration {}

pub static mut VIBRATION: Metavibration = Metavibration(103, 0, 3, 1, 32, None, 0, 0, 0, []);

///[clipping_0](GroundControl::VIBRATION::clipping_0).

pub struct Item__ad_hoc1929(pub *mut u8);

impl Item__ad_hoc1929
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[clipping_1](GroundControl::VIBRATION::clipping_1).

pub struct Item__ad_hoc1930(pub *mut u8);

impl Item__ad_hoc1930
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 4);
    }
}

///[clipping_2](GroundControl::VIBRATION::clipping_2).

pub struct Item__ad_hoc1931(pub *mut u8);

impl Item__ad_hoc1931
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 8);
    }
}

///[time_usec](GroundControl::VIBRATION::time_usec).

pub struct Item__ad_hoc1925(pub *mut u8);

impl Item__ad_hoc1925
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 12, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 12);
    }
}

///[vibration_x](GroundControl::VIBRATION::vibration_x).

pub struct Item__ad_hoc1926(pub *mut u8);

impl Item__ad_hoc1926
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[vibration_y](GroundControl::VIBRATION::vibration_y).

pub struct Item__ad_hoc1927(pub *mut u8);

impl Item__ad_hoc1927
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[vibration_z](GroundControl::VIBRATION::vibration_z).

pub struct Item__ad_hoc1928(pub *mut u8);

impl Item__ad_hoc1928
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

#[repr(C)]
pub struct Metaparam_ext_value(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metaparam_ext_value {}

pub static mut PARAM_EXT_VALUE: Metaparam_ext_value = Metaparam_ext_value(14, 2, 0, 0, 5, None, 3, 35, 3, [(&fld__ad_hoc2110 as *const _ad_hoc2110) as *const sys::Field,
    (&fld__ad_hoc2111 as *const _ad_hoc2111) as *const sys::Field,
    (&fld__ad_hoc411 as *const _ad_hoc411) as *const sys::Field]);

///[param_count](GroundControl::PARAM_EXT_VALUE::param_count).

pub struct Item__ad_hoc2112<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2112<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[param_index](GroundControl::PARAM_EXT_VALUE::param_index).

pub struct Item__ad_hoc2113<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2113<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[param_id](GroundControl::PARAM_EXT_VALUE::param_id).

pub struct Item__ad_hoc2110<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2110<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 35 && !src.set_field(35, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(35, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[param_value](GroundControl::PARAM_EXT_VALUE::param_value).

pub struct Item__ad_hoc2111<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2111<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 36 && !src.set_field(36, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(36, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[param_type](GroundControl::PARAM_EXT_VALUE::param_type).

pub struct Item__ad_hoc411<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc411<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_PARAM_EXT_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 37 && !src.set_field(37, -1) { return None; }

        Some({ packs::MAV_PARAM_EXT_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_PARAM_EXT_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 37
        {
            dst.set_field(37, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metabattery2(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metabattery2 {}

pub static mut BATTERY2: Metabattery2 = Metabattery2(190, 1, 0, 0, 4, None, 0, 0, 0, []);

///[voltage](GroundControl::BATTERY2::voltage).

pub struct Item__ad_hoc2321(pub *mut u8);

impl Item__ad_hoc2321
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[current_battery](GroundControl::BATTERY2::current_battery).

pub struct Item__ad_hoc2322(pub *mut u8);

impl Item__ad_hoc2322
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

#[repr(C)]
pub struct Metalimits_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 4]);

unsafe impl std::marker::Sync for Metalimits_status {}

pub static mut LIMITS_STATUS: Metalimits_status = Metalimits_status(186, 1, 4, 0, 19, None, 2, 146, 4, [(&fld__ad_hoc189 as *const _ad_hoc189) as *const sys::Field,
    (&fld__ad_hoc188 as *const _ad_hoc188) as *const sys::Field,
    (&fld__ad_hoc190 as *const _ad_hoc190) as *const sys::Field,
    (&fld__ad_hoc191 as *const _ad_hoc191) as *const sys::Field]);

///[breach_count](GroundControl::LIMITS_STATUS::breach_count).

pub struct Item__ad_hoc2247<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2247<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[last_trigger](GroundControl::LIMITS_STATUS::last_trigger).

pub struct Item__ad_hoc2243<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2243<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 2);
    }
}

///[last_action](GroundControl::LIMITS_STATUS::last_action).

pub struct Item__ad_hoc2244<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2244<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 6);
    }
}

///[last_recovery](GroundControl::LIMITS_STATUS::last_recovery).

pub struct Item__ad_hoc2245<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2245<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 10);
    }
}

///[last_clear](GroundControl::LIMITS_STATUS::last_clear).

pub struct Item__ad_hoc2246<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2246<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 14, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 14);
    }
}

///[limits_state](GroundControl::LIMITS_STATUS::limits_state).

pub struct Item__ad_hoc189<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc189<'a> {
    pub fn get(&mut self) -> Option<packs::LIMITS_STATE> {
        let src = &mut self.0;
        if src.base.field_bit != 146 && !src.set_field(146, -1) { return None; }

        Some({ packs::LIMITS_STATE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::LIMITS_STATE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 146
        {
            dst.set_field(146, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[mods_enabled](GroundControl::LIMITS_STATUS::mods_enabled).

pub struct Item__ad_hoc188<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc188<'a> {
    pub fn get(&mut self) -> Option<packs::LIMIT_MODULE> {
        let src = &mut self.0;
        if src.base.field_bit != 147 && !src.set_field(147, -1) { return None; }

        Some({ packs::LIMIT_MODULE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::LIMIT_MODULE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 147
        {
            dst.set_field(147, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

///[mods_required](GroundControl::LIMITS_STATUS::mods_required).

pub struct Item__ad_hoc190<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc190<'a> {
    pub fn get(&mut self) -> Option<packs::LIMIT_MODULE> {
        let src = &mut self.0;
        if src.base.field_bit != 148 && !src.set_field(148, -1) { return None; }

        Some({ packs::LIMIT_MODULE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::LIMIT_MODULE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 148
        {
            dst.set_field(148, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

///[mods_triggered](GroundControl::LIMITS_STATUS::mods_triggered).

pub struct Item__ad_hoc191<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc191<'a> {
    pub fn get(&mut self) -> Option<packs::LIMIT_MODULE> {
        let src = &mut self.0;
        if src.base.field_bit != 149 && !src.set_field(149, -1) { return None; }

        Some({ packs::LIMIT_MODULE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::LIMIT_MODULE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 149
        {
            dst.set_field(149, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metacamera_feedback(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metacamera_feedback {}

pub static mut CAMERA_FEEDBACK: Metacamera_feedback = Metacamera_feedback(0, 1, 0, 1, 45, None, 0, 352, 1, [(&fld__ad_hoc4 as *const _ad_hoc4) as *const sys::Field]);

///[img_idx](GroundControl::CAMERA_FEEDBACK::img_idx).

pub struct Item__ad_hoc2312<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2312<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[time_usec](GroundControl::CAMERA_FEEDBACK::time_usec).

pub struct Item__ad_hoc2309<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2309<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 2);
    }
}

///[target_system](GroundControl::CAMERA_FEEDBACK::target_system).

pub struct Item__ad_hoc2310<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2310<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 10);
    }
}

///[cam_idx](GroundControl::CAMERA_FEEDBACK::cam_idx).

pub struct Item__ad_hoc2311<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2311<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 11, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 11);
    }
}

///[lat](GroundControl::CAMERA_FEEDBACK::lat).

pub struct Item__ad_hoc2313<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2313<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 12, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 12);
    }
}

///[lng](GroundControl::CAMERA_FEEDBACK::lng).

pub struct Item__ad_hoc2314<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2314<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 16, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 16);
    }
}

///[alt_msl](GroundControl::CAMERA_FEEDBACK::alt_msl).

pub struct Item__ad_hoc2315<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2315<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 20);
    }
}

///[alt_rel](GroundControl::CAMERA_FEEDBACK::alt_rel).

pub struct Item__ad_hoc2316<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2316<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 24);
    }
}

///[roll](GroundControl::CAMERA_FEEDBACK::roll).

pub struct Item__ad_hoc2317<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2317<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 28);
    }
}

///[pitch](GroundControl::CAMERA_FEEDBACK::pitch).

pub struct Item__ad_hoc2318<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2318<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 32);
    }
}

///[yaw](GroundControl::CAMERA_FEEDBACK::yaw).

pub struct Item__ad_hoc2319<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2319<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 36);
    }
}

///[foc_len](GroundControl::CAMERA_FEEDBACK::foc_len).

pub struct Item__ad_hoc2320<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2320<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 40);
    }
}

///[flags](GroundControl::CAMERA_FEEDBACK::flags).

pub struct Item__ad_hoc4<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc4<'a> {
    pub fn get(&mut self) -> Option<packs::CAMERA_FEEDBACK_FLAGS> {
        let src = &mut self.0;
        if src.base.field_bit != 352 && !src.set_field(352, -1) { return None; }

        Some({ packs::CAMERA_FEEDBACK_FLAGS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::CAMERA_FEEDBACK_FLAGS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 352
        {
            dst.set_field(352, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metahil_gps(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                       u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metahil_gps {}

pub static mut HIL_GPS: Metahil_gps = Metahil_gps(168, 4, 0, 1, 36, None, 0, 0, 0, []);

///[eph](GroundControl::HIL_GPS::eph).

pub struct Item__ad_hoc1617(pub *mut u8);

impl Item__ad_hoc1617
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[epv](GroundControl::HIL_GPS::epv).

pub struct Item__ad_hoc1618(pub *mut u8);

impl Item__ad_hoc1618
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[vel](GroundControl::HIL_GPS::vel).

pub struct Item__ad_hoc1619(pub *mut u8);

impl Item__ad_hoc1619
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[cog](GroundControl::HIL_GPS::cog).

pub struct Item__ad_hoc1623(pub *mut u8);

impl Item__ad_hoc1623
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 6);
    }
}

///[time_usec](GroundControl::HIL_GPS::time_usec).

pub struct Item__ad_hoc1612(pub *mut u8);

impl Item__ad_hoc1612
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 8);
    }
}

///[fix_type](GroundControl::HIL_GPS::fix_type).

pub struct Item__ad_hoc1613(pub *mut u8);

impl Item__ad_hoc1613
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 16);
    }
}

///[lat](GroundControl::HIL_GPS::lat).

pub struct Item__ad_hoc1614(pub *mut u8);

impl Item__ad_hoc1614
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 17, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 17);
    }
}

///[lon](GroundControl::HIL_GPS::lon).

pub struct Item__ad_hoc1615(pub *mut u8);

impl Item__ad_hoc1615
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 21, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 21);
    }
}

///[alt](GroundControl::HIL_GPS::alt).

pub struct Item__ad_hoc1616(pub *mut u8);

impl Item__ad_hoc1616
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 25, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 25);
    }
}

///[vn](GroundControl::HIL_GPS::vn).

pub struct Item__ad_hoc1620(pub *mut u8);

impl Item__ad_hoc1620
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 29, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 29);
    }
}

///[ve](GroundControl::HIL_GPS::ve).

pub struct Item__ad_hoc1621(pub *mut u8);

impl Item__ad_hoc1621
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 31, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 31);
    }
}

///[vd](GroundControl::HIL_GPS::vd).

pub struct Item__ad_hoc1622(pub *mut u8);

impl Item__ad_hoc1622
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 33, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 33);
    }
}

///[satellites_visible](GroundControl::HIL_GPS::satellites_visible).

pub struct Item__ad_hoc1624(pub *mut u8);

impl Item__ad_hoc1624
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 35, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 35);
    }
}

#[repr(C)]
pub struct Metanav_controller_output(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metanav_controller_output {}

pub static mut NAV_CONTROLLER_OUTPUT: Metanav_controller_output = Metanav_controller_output(201, 1, 0, 0, 26, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metaauth_key(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaauth_key {}

pub static mut AUTH_KEY: Metaauth_key = Metaauth_key(95, 0, 0, 0, 1, None, 2, 2, 1, [(&fld__ad_hoc1003 as *const _ad_hoc1003) as *const sys::Field]);

#[repr(C)]
pub struct Metafence_fetch_point(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metafence_fetch_point {}

pub static mut FENCE_FETCH_POINT: Metafence_fetch_point = Metafence_fetch_point(119, 0, 0, 0, 3, None, 0, 0, 0, []);

///[target_system](GroundControl::FENCE_FETCH_POINT::target_system).

pub struct Item__ad_hoc2210(pub *mut u8);

impl Item__ad_hoc2210
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::FENCE_FETCH_POINT::target_component).

pub struct Item__ad_hoc2211(pub *mut u8);

impl Item__ad_hoc2211
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[idx](GroundControl::FENCE_FETCH_POINT::idx).

pub struct Item__ad_hoc2212(pub *mut u8);

impl Item__ad_hoc2212
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

#[repr(C)]
pub struct Metaradio(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaradio {}

pub static mut RADIO: Metaradio = Metaradio(32, 2, 0, 0, 9, None, 0, 0, 0, []);

///[rxerrors](GroundControl::RADIO::rxerrors).

pub struct Item__ad_hoc2241(pub *mut u8);

impl Item__ad_hoc2241
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[fixeD](GroundControl::RADIO::fixeD).

pub struct Item__ad_hoc2242(pub *mut u8);

impl Item__ad_hoc2242
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[rssi](GroundControl::RADIO::rssi).

pub struct Item__ad_hoc2236(pub *mut u8);

impl Item__ad_hoc2236
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[remrssi](GroundControl::RADIO::remrssi).

pub struct Item__ad_hoc2237(pub *mut u8);

impl Item__ad_hoc2237
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

///[txbuf](GroundControl::RADIO::txbuf).

pub struct Item__ad_hoc2238(pub *mut u8);

impl Item__ad_hoc2238
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 6);
    }
}

///[noise](GroundControl::RADIO::noise).

pub struct Item__ad_hoc2239(pub *mut u8);

impl Item__ad_hoc2239
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 7, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 7);
    }
}

///[remnoise](GroundControl::RADIO::remnoise).

pub struct Item__ad_hoc2240(pub *mut u8);

impl Item__ad_hoc2240
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

#[repr(C)]
pub struct Metalocal_position_ned_cov(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                      u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metalocal_position_ned_cov {}

pub static mut LOCAL_POSITION_NED_COV: Metalocal_position_ned_cov = Metalocal_position_ned_cov(180, 0, 0, 1, 225, None, 0, 1792, 1, [(&fld__ad_hoc112 as *const _ad_hoc112) as *const sys::Field]);

///[covariance](GroundControl::LOCAL_POSITION_NED_COV::covariance).

pub struct ItemArray__ad_hoc1245 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1245 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
}

impl Iterator for ItemArray__ad_hoc1245 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaairspeed_autocal(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaairspeed_autocal {}

pub static mut AIRSPEED_AUTOCAL: Metaairspeed_autocal = Metaairspeed_autocal(69, 0, 0, 0, 48, None, 0, 0, 0, []);

///[vx](GroundControl::AIRSPEED_AUTOCAL::vx).

pub struct Item__ad_hoc2265(pub *mut u8);

impl Item__ad_hoc2265
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[vy](GroundControl::AIRSPEED_AUTOCAL::vy).

pub struct Item__ad_hoc2266(pub *mut u8);

impl Item__ad_hoc2266
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[vz](GroundControl::AIRSPEED_AUTOCAL::vz).

pub struct Item__ad_hoc2267(pub *mut u8);

impl Item__ad_hoc2267
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[diff_pressure](GroundControl::AIRSPEED_AUTOCAL::diff_pressure).

pub struct Item__ad_hoc2268(pub *mut u8);

impl Item__ad_hoc2268
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[EAS2TAS](GroundControl::AIRSPEED_AUTOCAL::EAS2TAS).

pub struct Item__ad_hoc2269(pub *mut u8);

impl Item__ad_hoc2269
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[ratio](GroundControl::AIRSPEED_AUTOCAL::ratio).

pub struct Item__ad_hoc2270(pub *mut u8);

impl Item__ad_hoc2270
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[state_x](GroundControl::AIRSPEED_AUTOCAL::state_x).

pub struct Item__ad_hoc2271(pub *mut u8);

impl Item__ad_hoc2271
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[state_y](GroundControl::AIRSPEED_AUTOCAL::state_y).

pub struct Item__ad_hoc2272(pub *mut u8);

impl Item__ad_hoc2272
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[state_z](GroundControl::AIRSPEED_AUTOCAL::state_z).

pub struct Item__ad_hoc2273(pub *mut u8);

impl Item__ad_hoc2273
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

///[Pax](GroundControl::AIRSPEED_AUTOCAL::Pax).

pub struct Item__ad_hoc2274(pub *mut u8);

impl Item__ad_hoc2274
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 36); }
}

///[Pby](GroundControl::AIRSPEED_AUTOCAL::Pby).

pub struct Item__ad_hoc2275(pub *mut u8);

impl Item__ad_hoc2275
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 40); }
}

///[Pcz](GroundControl::AIRSPEED_AUTOCAL::Pcz).

pub struct Item__ad_hoc2276(pub *mut u8);

impl Item__ad_hoc2276
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 44, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 44); }
}

#[repr(C)]
pub struct Metaatt_pos_mocap(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaatt_pos_mocap {}

pub static mut ATT_POS_MOCAP: Metaatt_pos_mocap = Metaatt_pos_mocap(200, 0, 0, 1, 36, None, 0, 0, 0, []);

///[time_usec](GroundControl::ATT_POS_MOCAP::time_usec).

pub struct Item__ad_hoc1779(pub *mut u8);

impl Item__ad_hoc1779
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[q](GroundControl::ATT_POS_MOCAP::q).

pub struct ItemArray__ad_hoc1780 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1780 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1780 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[x](GroundControl::ATT_POS_MOCAP::x).

pub struct Item__ad_hoc1781(pub *mut u8);

impl Item__ad_hoc1781
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[y](GroundControl::ATT_POS_MOCAP::y).

pub struct Item__ad_hoc1782(pub *mut u8);

impl Item__ad_hoc1782
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[z](GroundControl::ATT_POS_MOCAP::z).

pub struct Item__ad_hoc1783(pub *mut u8);

impl Item__ad_hoc1783
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

#[repr(C)]
pub struct Metastatustext(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                          u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metastatustext {}

pub static mut STATUSTEXT: Metastatustext = Metastatustext(124, 0, 0, 0, 1, None, 2, 2, 2, [(&fld__ad_hoc683 as *const _ad_hoc683) as *const sys::Field,
    (&fld__ad_hoc1991 as *const _ad_hoc1991) as *const sys::Field]);

///[severity](GroundControl::STATUSTEXT::severity).

pub struct Item__ad_hoc683<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc683<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_SEVERITY> {
        let src = &mut self.0;
        if src.base.field_bit != 2 && !src.set_field(2, -1) { return None; }

        Some({ packs::MAV_SEVERITY::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_SEVERITY) {
        let dst = &mut self.0;
        if dst.base.field_bit != 2
        {
            dst.set_field(2, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[text](GroundControl::STATUSTEXT::text).

pub struct Item__ad_hoc1991<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1991<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 3 && !src.set_field(3, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(3, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metaping(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                    u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaping {}

pub static mut PING: Metaping = Metaping(122, 0, 1, 1, 14, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metagopro_get_request(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metagopro_get_request {}

pub static mut GOPRO_GET_REQUEST: Metagopro_get_request = Metagopro_get_request(138, 0, 0, 0, 3, None, 0, 16, 1, [(&fld__ad_hoc107 as *const _ad_hoc107) as *const sys::Field]);

///[target_system](GroundControl::GOPRO_GET_REQUEST::target_system).

pub struct Item__ad_hoc2401<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2401<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[target_component](GroundControl::GOPRO_GET_REQUEST::target_component).

pub struct Item__ad_hoc2402<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2402<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[cmd_id](GroundControl::GOPRO_GET_REQUEST::cmd_id).

pub struct Item__ad_hoc107<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc107<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_COMMAND> {
        let src = &mut self.0;
        if src.base.field_bit != 16 && !src.set_field(16, -1) { return None; }

        Some({ packs::GOPRO_COMMAND::from_bits((sys::get_bits(src.base.bytes, src.BIT, 5)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_COMMAND) {
        let dst = &mut self.0;
        if dst.base.field_bit != 16
        {
            dst.set_field(16, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 5, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metacamera_capture_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metacamera_capture_status {}

pub static mut CAMERA_CAPTURE_STATUS: Metacamera_capture_status = Metacamera_capture_status(216, 0, 2, 0, 18, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::CAMERA_CAPTURE_STATUS::time_boot_ms).

pub struct Item__ad_hoc2027(pub *mut u8);

impl Item__ad_hoc2027
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[recording_time_ms](GroundControl::CAMERA_CAPTURE_STATUS::recording_time_ms).

pub struct Item__ad_hoc2031(pub *mut u8);

impl Item__ad_hoc2031
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 4);
    }
}

///[image_status](GroundControl::CAMERA_CAPTURE_STATUS::image_status).

pub struct Item__ad_hoc2028(pub *mut u8);

impl Item__ad_hoc2028
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

///[video_status](GroundControl::CAMERA_CAPTURE_STATUS::video_status).

pub struct Item__ad_hoc2029(pub *mut u8);

impl Item__ad_hoc2029
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 9, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 9);
    }
}

///[image_interval](GroundControl::CAMERA_CAPTURE_STATUS::image_interval).

pub struct Item__ad_hoc2030(pub *mut u8);

impl Item__ad_hoc2030
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 10, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 10); }
}

///[available_capacity](GroundControl::CAMERA_CAPTURE_STATUS::available_capacity).

pub struct Item__ad_hoc2032(pub *mut u8);

impl Item__ad_hoc2032
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 14, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 14); }
}

#[repr(C)]
pub struct Metaglobal_position_int(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                   u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaglobal_position_int {}

pub static mut GLOBAL_POSITION_INT: Metaglobal_position_int = Metaglobal_position_int(162, 1, 1, 0, 28, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metaencapsulated_data(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaencapsulated_data {}

pub static mut ENCAPSULATED_DATA: Metaencapsulated_data = Metaencapsulated_data(170, 1, 0, 0, 255, None, 0, 0, 0, []);

///[seqnr](GroundControl::ENCAPSULATED_DATA::seqnr).

pub struct Item__ad_hoc1749(pub *mut u8);

impl Item__ad_hoc1749
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[daTa](GroundControl::ENCAPSULATED_DATA::daTa).

pub struct ItemArray__ad_hoc1750 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1750 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1750 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metagps_input(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metagps_input {}

pub static mut GPS_INPUT: Metagps_input = Metagps_input(73, 1, 1, 1, 62, None, 0, 488, 1, [(&fld__ad_hoc881 as *const _ad_hoc881) as *const sys::Field]);

///[time_week](GroundControl::GPS_INPUT::time_week).

pub struct Item__ad_hoc1887<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1887<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[time_week_ms](GroundControl::GPS_INPUT::time_week_ms).

pub struct Item__ad_hoc1886<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1886<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 2);
    }
}

///[time_usec](GroundControl::GPS_INPUT::time_usec).

pub struct Item__ad_hoc1884<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1884<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 6);
    }
}

///[gps_id](GroundControl::GPS_INPUT::gps_id).

pub struct Item__ad_hoc1885<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1885<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 14, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 14);
    }
}

///[fix_type](GroundControl::GPS_INPUT::fix_type).

pub struct Item__ad_hoc1888<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1888<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 15, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 15);
    }
}

///[lat](GroundControl::GPS_INPUT::lat).

pub struct Item__ad_hoc1889<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1889<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 16, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 16);
    }
}

///[lon](GroundControl::GPS_INPUT::lon).

pub struct Item__ad_hoc1890<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1890<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 20, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 20);
    }
}

///[alt](GroundControl::GPS_INPUT::alt).

pub struct Item__ad_hoc1891<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1891<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 24);
    }
}

///[hdop](GroundControl::GPS_INPUT::hdop).

pub struct Item__ad_hoc1892<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1892<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 28);
    }
}

///[vdop](GroundControl::GPS_INPUT::vdop).

pub struct Item__ad_hoc1893<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1893<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 32);
    }
}

///[vn](GroundControl::GPS_INPUT::vn).

pub struct Item__ad_hoc1894<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1894<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 36);
    }
}

///[ve](GroundControl::GPS_INPUT::ve).

pub struct Item__ad_hoc1895<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1895<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 40);
    }
}

///[vd](GroundControl::GPS_INPUT::vd).

pub struct Item__ad_hoc1896<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1896<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 44, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 44);
    }
}

///[speed_accuracy](GroundControl::GPS_INPUT::speed_accuracy).

pub struct Item__ad_hoc1897<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1897<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 48, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 48);
    }
}

///[horiz_accuracy](GroundControl::GPS_INPUT::horiz_accuracy).

pub struct Item__ad_hoc1898<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1898<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 52, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 52);
    }
}

///[vert_accuracy](GroundControl::GPS_INPUT::vert_accuracy).

pub struct Item__ad_hoc1899<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1899<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 56, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 56);
    }
}

///[satellites_visible](GroundControl::GPS_INPUT::satellites_visible).

pub struct Item__ad_hoc1900<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1900<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 60, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 60);
    }
}

///[ignore_flags](GroundControl::GPS_INPUT::ignore_flags).

pub struct Item__ad_hoc881<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc881<'a> {
    pub fn get(&mut self) -> Option<packs::GPS_INPUT_IGNORE_FLAGS> {
        let src = &mut self.0;
        if src.base.field_bit != 488 && !src.set_field(488, -1) { return None; }

        Some({ packs::GPS_INPUT_IGNORE_FLAGS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::GPS_INPUT_IGNORE_FLAGS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 488
        {
            dst.set_field(488, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metacommand_long(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metacommand_long {}

pub static mut COMMAND_LONG: Metacommand_long = Metacommand_long(101, 0, 0, 0, 32, None, 2, 250, 1, [(&fld__ad_hoc202 as *const _ad_hoc202) as *const sys::Field]);

#[repr(C)]
pub struct Metacompassmot_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metacompassmot_status {}

pub static mut COMPASSMOT_STATUS: Metacompassmot_status = Metacompassmot_status(43, 2, 0, 0, 20, None, 0, 0, 0, []);

///[throttle](GroundControl::COMPASSMOT_STATUS::throttle).

pub struct Item__ad_hoc2289(pub *mut u8);

impl Item__ad_hoc2289
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[interference](GroundControl::COMPASSMOT_STATUS::interference).

pub struct Item__ad_hoc2291(pub *mut u8);

impl Item__ad_hoc2291
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[current](GroundControl::COMPASSMOT_STATUS::current).

pub struct Item__ad_hoc2290(pub *mut u8);

impl Item__ad_hoc2290
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[CompensationX](GroundControl::COMPASSMOT_STATUS::CompensationX).

pub struct Item__ad_hoc2292(pub *mut u8);

impl Item__ad_hoc2292
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[CompensationY](GroundControl::COMPASSMOT_STATUS::CompensationY).

pub struct Item__ad_hoc2293(pub *mut u8);

impl Item__ad_hoc2293
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[CompensationZ](GroundControl::COMPASSMOT_STATUS::CompensationZ).

pub struct Item__ad_hoc2294(pub *mut u8);

impl Item__ad_hoc2294
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

#[repr(C)]
pub struct Metalog_request_data(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalog_request_data {}

pub static mut LOG_REQUEST_DATA: Metalog_request_data = Metalog_request_data(72, 1, 2, 0, 12, None, 0, 0, 0, []);

///[id](GroundControl::LOG_REQUEST_DATA::id).

pub struct Item__ad_hoc1674(pub *mut u8);

impl Item__ad_hoc1674
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[ofs](GroundControl::LOG_REQUEST_DATA::ofs).

pub struct Item__ad_hoc1675(pub *mut u8);

impl Item__ad_hoc1675
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 2);
    }
}

///[count](GroundControl::LOG_REQUEST_DATA::count).

pub struct Item__ad_hoc1676(pub *mut u8);

impl Item__ad_hoc1676
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[target_system](GroundControl::LOG_REQUEST_DATA::target_system).

pub struct Item__ad_hoc1672(pub *mut u8);

impl Item__ad_hoc1672
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 10);
    }
}

///[target_component](GroundControl::LOG_REQUEST_DATA::target_component).

pub struct Item__ad_hoc1673(pub *mut u8);

impl Item__ad_hoc1673
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 11, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 11);
    }
}

#[repr(C)]
pub struct Metagps_raw_int(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 6]);

unsafe impl std::marker::Sync for Metagps_raw_int {}

pub static mut GPS_RAW_INT: Metagps_raw_int = Metagps_raw_int(56, 4, 0, 1, 30, None, 2, 234, 6, [(&fld__ad_hoc153 as *const _ad_hoc153) as *const sys::Field,
    (&fld__ad_hoc1029 as *const _ad_hoc1029) as *const sys::Field,
    (&fld__ad_hoc1030 as *const _ad_hoc1030) as *const sys::Field,
    (&fld__ad_hoc1031 as *const _ad_hoc1031) as *const sys::Field,
    (&fld__ad_hoc1032 as *const _ad_hoc1032) as *const sys::Field,
    (&fld__ad_hoc1033 as *const _ad_hoc1033) as *const sys::Field]);

#[repr(C)]
pub struct Metacamera_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metacamera_status {}

pub static mut CAMERA_STATUS: Metacamera_status = Metacamera_status(211, 1, 0, 1, 29, None, 0, 224, 1, [(&fld__ad_hoc684 as *const _ad_hoc684) as *const sys::Field]);

///[img_idx](GroundControl::CAMERA_STATUS::img_idx).

pub struct Item__ad_hoc2304<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2304<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[time_usec](GroundControl::CAMERA_STATUS::time_usec).

pub struct Item__ad_hoc2301<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2301<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 2);
    }
}

///[target_system](GroundControl::CAMERA_STATUS::target_system).

pub struct Item__ad_hoc2302<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2302<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 10);
    }
}

///[cam_idx](GroundControl::CAMERA_STATUS::cam_idx).

pub struct Item__ad_hoc2303<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2303<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 11, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 11);
    }
}

///[p1](GroundControl::CAMERA_STATUS::p1).

pub struct Item__ad_hoc2305<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2305<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[p2](GroundControl::CAMERA_STATUS::p2).

pub struct Item__ad_hoc2306<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2306<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 16);
    }
}

///[p3](GroundControl::CAMERA_STATUS::p3).

pub struct Item__ad_hoc2307<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2307<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 20);
    }
}

///[p4](GroundControl::CAMERA_STATUS::p4).

pub struct Item__ad_hoc2308<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2308<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 24);
    }
}

///[event_id](GroundControl::CAMERA_STATUS::event_id).

pub struct Item__ad_hoc684<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc684<'a> {
    pub fn get(&mut self) -> Option<packs::CAMERA_STATUS_TYPES> {
        let src = &mut self.0;
        if src.base.field_bit != 224 && !src.set_field(224, -1) { return None; }

        Some({ packs::CAMERA_STATUS_TYPES::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::CAMERA_STATUS_TYPES) {
        let dst = &mut self.0;
        if dst.base.field_bit != 224
        {
            dst.set_field(224, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metarc_channels_scaled(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metarc_channels_scaled {}

pub static mut RC_CHANNELS_SCALED: Metarc_channels_scaled = Metarc_channels_scaled(120, 0, 1, 0, 22, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metacamera_settings(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metacamera_settings {}

pub static mut CAMERA_SETTINGS: Metacamera_settings = Metacamera_settings(181, 0, 1, 0, 5, None, 0, 32, 1, [(&fld__ad_hoc84 as *const _ad_hoc84) as *const sys::Field]);

///[time_boot_ms](GroundControl::CAMERA_SETTINGS::time_boot_ms).

pub struct Item__ad_hoc2017<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2017<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[mode_id](GroundControl::CAMERA_SETTINGS::mode_id).

pub struct Item__ad_hoc84<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc84<'a> {
    pub fn get(&mut self) -> Option<packs::CAMERA_MODE> {
        let src = &mut self.0;
        if src.base.field_bit != 32 && !src.set_field(32, -1) { return None; }

        Some({ packs::CAMERA_MODE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::CAMERA_MODE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 32
        {
            dst.set_field(32, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metadevice_op_read_reply(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                    u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadevice_op_read_reply {}

pub static mut DEVICE_OP_READ_REPLY: Metadevice_op_read_reply = Metadevice_op_read_reply(165, 0, 1, 0, 135, None, 0, 0, 0, []);

///[request_id](GroundControl::DEVICE_OP_READ_REPLY::request_id).

pub struct Item__ad_hoc2417(pub *mut u8);

impl Item__ad_hoc2417
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[result](GroundControl::DEVICE_OP_READ_REPLY::result).

pub struct Item__ad_hoc2418(pub *mut u8);

impl Item__ad_hoc2418
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[regstart](GroundControl::DEVICE_OP_READ_REPLY::regstart).

pub struct Item__ad_hoc2419(pub *mut u8);

impl Item__ad_hoc2419
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

///[count](GroundControl::DEVICE_OP_READ_REPLY::count).

pub struct Item__ad_hoc2420(pub *mut u8);

impl Item__ad_hoc2420
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 6);
    }
}

///[daTa](GroundControl::DEVICE_OP_READ_REPLY::daTa).

pub struct ItemArray__ad_hoc2421 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2421 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2421 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaraw_pressure(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaraw_pressure {}

pub static mut RAW_PRESSURE: Metaraw_pressure = Metaraw_pressure(19, 0, 0, 1, 16, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metadigicam_control(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadigicam_control {}

pub static mut DIGICAM_CONTROL: Metadigicam_control = Metadigicam_control(49, 0, 0, 0, 13, None, 0, 0, 0, []);

///[target_system](GroundControl::DIGICAM_CONTROL::target_system).

pub struct Item__ad_hoc2178(pub *mut u8);

impl Item__ad_hoc2178
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::DIGICAM_CONTROL::target_component).

pub struct Item__ad_hoc2179(pub *mut u8);

impl Item__ad_hoc2179
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[session](GroundControl::DIGICAM_CONTROL::session).

pub struct Item__ad_hoc2180(pub *mut u8);

impl Item__ad_hoc2180
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[zoom_pos](GroundControl::DIGICAM_CONTROL::zoom_pos).

pub struct Item__ad_hoc2181(pub *mut u8);

impl Item__ad_hoc2181
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

///[zoom_step](GroundControl::DIGICAM_CONTROL::zoom_step).

pub struct Item__ad_hoc2182(pub *mut u8);

impl Item__ad_hoc2182
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[focus_lock](GroundControl::DIGICAM_CONTROL::focus_lock).

pub struct Item__ad_hoc2183(pub *mut u8);

impl Item__ad_hoc2183
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

///[shot](GroundControl::DIGICAM_CONTROL::shot).

pub struct Item__ad_hoc2184(pub *mut u8);

impl Item__ad_hoc2184
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 6);
    }
}

///[command_id](GroundControl::DIGICAM_CONTROL::command_id).

pub struct Item__ad_hoc2185(pub *mut u8);

impl Item__ad_hoc2185
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 7, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 7);
    }
}

///[extra_param](GroundControl::DIGICAM_CONTROL::extra_param).

pub struct Item__ad_hoc2186(pub *mut u8);

impl Item__ad_hoc2186
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

///[extra_value](GroundControl::DIGICAM_CONTROL::extra_value).

pub struct Item__ad_hoc2187(pub *mut u8);

impl Item__ad_hoc2187
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 9, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 9); }
}

#[repr(C)]
pub struct Metanamed_value_float(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metanamed_value_float {}

pub static mut NAMED_VALUE_FLOAT: Metanamed_value_float = Metanamed_value_float(8, 0, 1, 0, 9, None, 2, 66, 1, [(&fld__ad_hoc1986 as *const _ad_hoc1986) as *const sys::Field]);

///[time_boot_ms](GroundControl::NAMED_VALUE_FLOAT::time_boot_ms).

pub struct Item__ad_hoc1985<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1985<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[value](GroundControl::NAMED_VALUE_FLOAT::value).

pub struct Item__ad_hoc1987<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1987<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 4);
    }
}

///[name](GroundControl::NAMED_VALUE_FLOAT::name).

pub struct Item__ad_hoc1986<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1986<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 66 && !src.set_field(66, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(66, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metagopro_heartbeat(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metagopro_heartbeat {}

pub static mut GOPRO_HEARTBEAT: Metagopro_heartbeat = Metagopro_heartbeat(175, 0, 0, 0, 1, None, 2, 2, 3, [(&fld__ad_hoc629 as *const _ad_hoc629) as *const sys::Field,
    (&fld__ad_hoc630 as *const _ad_hoc630) as *const sys::Field,
    (&fld__ad_hoc628 as *const _ad_hoc628) as *const sys::Field]);

///[status](GroundControl::GOPRO_HEARTBEAT::status).

pub struct Item__ad_hoc629<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc629<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_HEARTBEAT_STATUS> {
        let src = &mut self.0;
        if src.base.field_bit != 2 && !src.set_field(2, -1) { return None; }

        Some({ packs::GOPRO_HEARTBEAT_STATUS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_HEARTBEAT_STATUS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 2
        {
            dst.set_field(2, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

///[capture_mode](GroundControl::GOPRO_HEARTBEAT::capture_mode).

pub struct Item__ad_hoc630<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc630<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_CAPTURE_MODE> {
        let src = &mut self.0;
        if src.base.field_bit != 3 && !src.set_field(3, -1) { return None; }

        Some({ packs::GOPRO_CAPTURE_MODE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_CAPTURE_MODE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 3
        {
            dst.set_field(3, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

///[flags](GroundControl::GOPRO_HEARTBEAT::flags).

pub struct Item__ad_hoc628<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc628<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_HEARTBEAT_FLAGS> {
        let src = &mut self.0;
        if src.base.field_bit != 4 && !src.set_field(4, -1) { return None; }

        Some({ packs::GOPRO_HEARTBEAT_FLAGS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_HEARTBEAT_FLAGS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 4
        {
            dst.set_field(4, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaattitude(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaattitude {}

pub static mut ATTITUDE: Metaattitude = Metaattitude(134, 0, 1, 0, 28, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metamission_write_partial_list(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                          u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamission_write_partial_list {}

pub static mut MISSION_WRITE_PARTIAL_LIST: Metamission_write_partial_list = Metamission_write_partial_list(145, 0, 0, 0, 7, None, 0, 48, 1, [(&fld__ad_hoc422 as *const _ad_hoc422) as *const sys::Field]);

#[repr(C)]
pub struct Metaahrs2(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaahrs2 {}

pub static mut AHRS2: Metaahrs2 = Metaahrs2(17, 0, 0, 0, 24, None, 0, 0, 0, []);

///[roll](GroundControl::AHRS2::roll).

pub struct Item__ad_hoc2295(pub *mut u8);

impl Item__ad_hoc2295
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[pitch](GroundControl::AHRS2::pitch).

pub struct Item__ad_hoc2296(pub *mut u8);

impl Item__ad_hoc2296
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[yaw](GroundControl::AHRS2::yaw).

pub struct Item__ad_hoc2297(pub *mut u8);

impl Item__ad_hoc2297
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[altitude](GroundControl::AHRS2::altitude).

pub struct Item__ad_hoc2298(pub *mut u8);

impl Item__ad_hoc2298
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[lat](GroundControl::AHRS2::lat).

pub struct Item__ad_hoc2299(pub *mut u8);

impl Item__ad_hoc2299
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 16);
    }
}

///[lng](GroundControl::AHRS2::lng).

pub struct Item__ad_hoc2300(pub *mut u8);

impl Item__ad_hoc2300
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 20, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 20);
    }
}

#[repr(C)]
pub struct Metalog_erase(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalog_erase {}

pub static mut LOG_ERASE: Metalog_erase = Metalog_erase(4, 0, 0, 0, 2, None, 0, 0, 0, []);

///[target_system](GroundControl::LOG_ERASE::target_system).

pub struct Item__ad_hoc1681(pub *mut u8);

impl Item__ad_hoc1681
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::LOG_ERASE::target_component).

pub struct Item__ad_hoc1682(pub *mut u8);

impl Item__ad_hoc1682
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

#[repr(C)]
pub struct Metaterrain_request(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaterrain_request {}

pub static mut TERRAIN_REQUEST: Metaterrain_request = Metaterrain_request(92, 1, 0, 1, 18, None, 0, 0, 0, []);

///[grid_spacing](GroundControl::TERRAIN_REQUEST::grid_spacing).

pub struct Item__ad_hoc1759(pub *mut u8);

impl Item__ad_hoc1759
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[mask](GroundControl::TERRAIN_REQUEST::mask).

pub struct Item__ad_hoc1760(pub *mut u8);

impl Item__ad_hoc1760
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 2);
    }
}

///[lat](GroundControl::TERRAIN_REQUEST::lat).

pub struct Item__ad_hoc1757(pub *mut u8);

impl Item__ad_hoc1757
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 10);
    }
}

///[lon](GroundControl::TERRAIN_REQUEST::lon).

pub struct Item__ad_hoc1758(pub *mut u8);

impl Item__ad_hoc1758
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 14, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 14);
    }
}

#[repr(C)]
pub struct Metamount_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamount_status {}

pub static mut MOUNT_STATUS: Metamount_status = Metamount_status(44, 0, 0, 0, 14, None, 0, 0, 0, []);

///[target_system](GroundControl::MOUNT_STATUS::target_system).

pub struct Item__ad_hoc2199(pub *mut u8);

impl Item__ad_hoc2199
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::MOUNT_STATUS::target_component).

pub struct Item__ad_hoc2200(pub *mut u8);

impl Item__ad_hoc2200
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[pointing_a](GroundControl::MOUNT_STATUS::pointing_a).

pub struct Item__ad_hoc2201(pub *mut u8);

impl Item__ad_hoc2201
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 2);
    }
}

///[pointing_b](GroundControl::MOUNT_STATUS::pointing_b).

pub struct Item__ad_hoc2202(pub *mut u8);

impl Item__ad_hoc2202
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[pointing_c](GroundControl::MOUNT_STATUS::pointing_c).

pub struct Item__ad_hoc2203(pub *mut u8);

impl Item__ad_hoc2203
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 10);
    }
}

#[repr(C)]
pub struct Metamanual_setpoint(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamanual_setpoint {}

pub static mut MANUAL_SETPOINT: Metamanual_setpoint = Metamanual_setpoint(219, 0, 1, 0, 22, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metapid_tuning(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                          u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metapid_tuning {}

pub static mut PID_TUNING: Metapid_tuning = Metapid_tuning(64, 0, 0, 0, 25, None, 0, 192, 1, [(&fld__ad_hoc104 as *const _ad_hoc104) as *const sys::Field]);

///[desired](GroundControl::PID_TUNING::desired).

pub struct Item__ad_hoc2373<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2373<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 0);
    }
}

///[achieved](GroundControl::PID_TUNING::achieved).

pub struct Item__ad_hoc2374<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2374<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 4);
    }
}

///[FF](GroundControl::PID_TUNING::FF).

pub struct Item__ad_hoc2375<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2375<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 8);
    }
}

///[P](GroundControl::PID_TUNING::P).

pub struct Item__ad_hoc2376<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2376<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[I](GroundControl::PID_TUNING::I).

pub struct Item__ad_hoc2377<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2377<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 16);
    }
}

///[D](GroundControl::PID_TUNING::D).

pub struct Item__ad_hoc2378<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2378<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 20);
    }
}

///[axis](GroundControl::PID_TUNING::axis).

pub struct Item__ad_hoc104<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc104<'a> {
    pub fn get(&mut self) -> Option<packs::PID_TUNING_AXIS> {
        let src = &mut self.0;
        if src.base.field_bit != 192 && !src.set_field(192, -1) { return None; }

        Some({ packs::PID_TUNING_AXIS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::PID_TUNING_AXIS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 192
        {
            dst.set_field(192, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metasafety_allowed_area(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                   u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metasafety_allowed_area {}

pub static mut SAFETY_ALLOWED_AREA: Metasafety_allowed_area = Metasafety_allowed_area(179, 0, 0, 0, 25, None, 0, 192, 1, [(&fld__ad_hoc880 as *const _ad_hoc880) as *const sys::Field]);

#[repr(C)]
pub struct Metaoptical_flow_rad(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaoptical_flow_rad {}

pub static mut OPTICAL_FLOW_RAD: Metaoptical_flow_rad = Metaoptical_flow_rad(202, 0, 2, 1, 44, None, 0, 0, 0, []);

///[integration_time_us](GroundControl::OPTICAL_FLOW_RAD::integration_time_us).

pub struct Item__ad_hoc1551(pub *mut u8);

impl Item__ad_hoc1551
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[time_delta_distance_us](GroundControl::OPTICAL_FLOW_RAD::time_delta_distance_us).

pub struct Item__ad_hoc1559(pub *mut u8);

impl Item__ad_hoc1559
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 4);
    }
}

///[time_usec](GroundControl::OPTICAL_FLOW_RAD::time_usec).

pub struct Item__ad_hoc1549(pub *mut u8);

impl Item__ad_hoc1549
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 8);
    }
}

///[sensor_id](GroundControl::OPTICAL_FLOW_RAD::sensor_id).

pub struct Item__ad_hoc1550(pub *mut u8);

impl Item__ad_hoc1550
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 16);
    }
}

///[integrated_x](GroundControl::OPTICAL_FLOW_RAD::integrated_x).

pub struct Item__ad_hoc1552(pub *mut u8);

impl Item__ad_hoc1552
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 17, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 17); }
}

///[integrated_y](GroundControl::OPTICAL_FLOW_RAD::integrated_y).

pub struct Item__ad_hoc1553(pub *mut u8);

impl Item__ad_hoc1553
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 21, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 21); }
}

///[integrated_xgyro](GroundControl::OPTICAL_FLOW_RAD::integrated_xgyro).

pub struct Item__ad_hoc1554(pub *mut u8);

impl Item__ad_hoc1554
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 25, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 25); }
}

///[integrated_ygyro](GroundControl::OPTICAL_FLOW_RAD::integrated_ygyro).

pub struct Item__ad_hoc1555(pub *mut u8);

impl Item__ad_hoc1555
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 29, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 29); }
}

///[integrated_zgyro](GroundControl::OPTICAL_FLOW_RAD::integrated_zgyro).

pub struct Item__ad_hoc1556(pub *mut u8);

impl Item__ad_hoc1556
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 33, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 33); }
}

///[temperature](GroundControl::OPTICAL_FLOW_RAD::temperature).

pub struct Item__ad_hoc1557(pub *mut u8);

impl Item__ad_hoc1557
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 37, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 37);
    }
}

///[quality](GroundControl::OPTICAL_FLOW_RAD::quality).

pub struct Item__ad_hoc1558(pub *mut u8);

impl Item__ad_hoc1558
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 39, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 39);
    }
}

///[distance](GroundControl::OPTICAL_FLOW_RAD::distance).

pub struct Item__ad_hoc1560(pub *mut u8);

impl Item__ad_hoc1560
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 40); }
}

#[repr(C)]
pub struct Metalog_data(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalog_data {}

pub static mut LOG_DATA: Metalog_data = Metalog_data(204, 1, 1, 0, 97, None, 0, 0, 0, []);

///[id](GroundControl::LOG_DATA::id).

pub struct Item__ad_hoc1677(pub *mut u8);

impl Item__ad_hoc1677
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[ofs](GroundControl::LOG_DATA::ofs).

pub struct Item__ad_hoc1678(pub *mut u8);

impl Item__ad_hoc1678
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 2);
    }
}

///[count](GroundControl::LOG_DATA::count).

pub struct Item__ad_hoc1679(pub *mut u8);

impl Item__ad_hoc1679
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 6);
    }
}

///[daTa](GroundControl::LOG_DATA::daTa).

pub struct ItemArray__ad_hoc1680 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1680 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1680 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metamission_clear_all(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamission_clear_all {}

pub static mut MISSION_CLEAR_ALL: Metamission_clear_all = Metamission_clear_all(193, 0, 0, 0, 3, None, 0, 16, 1, [(&fld__ad_hoc71 as *const _ad_hoc71) as *const sys::Field]);

#[repr(C)]
pub struct Metaahrs3(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaahrs3 {}

pub static mut AHRS3: Metaahrs3 = Metaahrs3(65, 0, 0, 0, 40, None, 0, 0, 0, []);

///[roll](GroundControl::AHRS3::roll).

pub struct Item__ad_hoc2323(pub *mut u8);

impl Item__ad_hoc2323
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[pitch](GroundControl::AHRS3::pitch).

pub struct Item__ad_hoc2324(pub *mut u8);

impl Item__ad_hoc2324
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[yaw](GroundControl::AHRS3::yaw).

pub struct Item__ad_hoc2325(pub *mut u8);

impl Item__ad_hoc2325
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[altitude](GroundControl::AHRS3::altitude).

pub struct Item__ad_hoc2326(pub *mut u8);

impl Item__ad_hoc2326
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[lat](GroundControl::AHRS3::lat).

pub struct Item__ad_hoc2327(pub *mut u8);

impl Item__ad_hoc2327
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 16);
    }
}

///[lng](GroundControl::AHRS3::lng).

pub struct Item__ad_hoc2328(pub *mut u8);

impl Item__ad_hoc2328
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 20, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 20);
    }
}

///[v1](GroundControl::AHRS3::v1).

pub struct Item__ad_hoc2329(pub *mut u8);

impl Item__ad_hoc2329
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[v2](GroundControl::AHRS3::v2).

pub struct Item__ad_hoc2330(pub *mut u8);

impl Item__ad_hoc2330
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[v3](GroundControl::AHRS3::v3).

pub struct Item__ad_hoc2331(pub *mut u8);

impl Item__ad_hoc2331
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

///[v4](GroundControl::AHRS3::v4).

pub struct Item__ad_hoc2332(pub *mut u8);

impl Item__ad_hoc2332
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 36); }
}

#[repr(C)]
pub struct Metavicon_position_estimate(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                       u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metavicon_position_estimate {}

pub static mut VICON_POSITION_ESTIMATE: Metavicon_position_estimate = Metavicon_position_estimate(135, 0, 0, 1, 32, None, 0, 0, 0, []);

///[usec](GroundControl::VICON_POSITION_ESTIMATE::usec).

pub struct Item__ad_hoc1527(pub *mut u8);

impl Item__ad_hoc1527
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[x](GroundControl::VICON_POSITION_ESTIMATE::x).

pub struct Item__ad_hoc1528(pub *mut u8);

impl Item__ad_hoc1528
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[y](GroundControl::VICON_POSITION_ESTIMATE::y).

pub struct Item__ad_hoc1529(pub *mut u8);

impl Item__ad_hoc1529
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[z](GroundControl::VICON_POSITION_ESTIMATE::z).

pub struct Item__ad_hoc1530(pub *mut u8);

impl Item__ad_hoc1530
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[roll](GroundControl::VICON_POSITION_ESTIMATE::roll).

pub struct Item__ad_hoc1531(pub *mut u8);

impl Item__ad_hoc1531
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[pitch](GroundControl::VICON_POSITION_ESTIMATE::pitch).

pub struct Item__ad_hoc1532(pub *mut u8);

impl Item__ad_hoc1532
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[yaw](GroundControl::VICON_POSITION_ESTIMATE::yaw).

pub struct Item__ad_hoc1533(pub *mut u8);

impl Item__ad_hoc1533
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

#[repr(C)]
pub struct Metagps2_rtk(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metagps2_rtk {}

pub static mut GPS2_RTK: Metagps2_rtk = Metagps2_rtk(91, 1, 3, 0, 35, None, 0, 0, 0, []);

///[wn](GroundControl::GPS2_RTK::wn).

pub struct Item__ad_hoc1721(pub *mut u8);

impl Item__ad_hoc1721
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[time_last_baseline_ms](GroundControl::GPS2_RTK::time_last_baseline_ms).

pub struct Item__ad_hoc1719(pub *mut u8);

impl Item__ad_hoc1719
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 2);
    }
}

///[tow](GroundControl::GPS2_RTK::tow).

pub struct Item__ad_hoc1722(pub *mut u8);

impl Item__ad_hoc1722
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[accuracy](GroundControl::GPS2_RTK::accuracy).

pub struct Item__ad_hoc1730(pub *mut u8);

impl Item__ad_hoc1730
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 10);
    }
}

///[rtk_receiver_id](GroundControl::GPS2_RTK::rtk_receiver_id).

pub struct Item__ad_hoc1720(pub *mut u8);

impl Item__ad_hoc1720
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 14, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 14);
    }
}

///[rtk_health](GroundControl::GPS2_RTK::rtk_health).

pub struct Item__ad_hoc1723(pub *mut u8);

impl Item__ad_hoc1723
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 15, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 15);
    }
}

///[rtk_rate](GroundControl::GPS2_RTK::rtk_rate).

pub struct Item__ad_hoc1724(pub *mut u8);

impl Item__ad_hoc1724
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 16);
    }
}

///[nsats](GroundControl::GPS2_RTK::nsats).

pub struct Item__ad_hoc1725(pub *mut u8);

impl Item__ad_hoc1725
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 17, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 17);
    }
}

///[baseline_coords_type](GroundControl::GPS2_RTK::baseline_coords_type).

pub struct Item__ad_hoc1726(pub *mut u8);

impl Item__ad_hoc1726
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 18, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 18);
    }
}

///[baseline_a_mm](GroundControl::GPS2_RTK::baseline_a_mm).

pub struct Item__ad_hoc1727(pub *mut u8);

impl Item__ad_hoc1727
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 19, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 19);
    }
}

///[baseline_b_mm](GroundControl::GPS2_RTK::baseline_b_mm).

pub struct Item__ad_hoc1728(pub *mut u8);

impl Item__ad_hoc1728
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 23, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 23);
    }
}

///[baseline_c_mm](GroundControl::GPS2_RTK::baseline_c_mm).

pub struct Item__ad_hoc1729(pub *mut u8);

impl Item__ad_hoc1729
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 27, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 27);
    }
}

///[iar_num_hypotheses](GroundControl::GPS2_RTK::iar_num_hypotheses).

pub struct Item__ad_hoc1731(pub *mut u8);

impl Item__ad_hoc1731
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 31, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 31);
    }
}

#[repr(C)]
pub struct Metamag_cal_report(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamag_cal_report {}

pub static mut MAG_CAL_REPORT: Metamag_cal_report = Metamag_cal_report(63, 0, 0, 0, 44, None, 0, 344, 1, [(&fld__ad_hoc83 as *const _ad_hoc83) as *const sys::Field]);

///[compass_id](GroundControl::MAG_CAL_REPORT::compass_id).

pub struct Item__ad_hoc2355<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2355<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[cal_mask](GroundControl::MAG_CAL_REPORT::cal_mask).

pub struct Item__ad_hoc2356<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2356<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[autosaved](GroundControl::MAG_CAL_REPORT::autosaved).

pub struct Item__ad_hoc2357<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2357<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 2);
    }
}

///[fitness](GroundControl::MAG_CAL_REPORT::fitness).

pub struct Item__ad_hoc2358<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2358<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 3, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 3);
    }
}

///[ofs_x](GroundControl::MAG_CAL_REPORT::ofs_x).

pub struct Item__ad_hoc2359<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2359<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 7, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 7);
    }
}

///[ofs_y](GroundControl::MAG_CAL_REPORT::ofs_y).

pub struct Item__ad_hoc2360<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2360<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 11, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 11);
    }
}

///[ofs_z](GroundControl::MAG_CAL_REPORT::ofs_z).

pub struct Item__ad_hoc2361<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2361<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 15, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 15);
    }
}

///[diag_x](GroundControl::MAG_CAL_REPORT::diag_x).

pub struct Item__ad_hoc2362<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2362<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 19, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 19);
    }
}

///[diag_y](GroundControl::MAG_CAL_REPORT::diag_y).

pub struct Item__ad_hoc2363<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2363<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 23, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 23);
    }
}

///[diag_z](GroundControl::MAG_CAL_REPORT::diag_z).

pub struct Item__ad_hoc2364<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2364<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 27, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 27);
    }
}

///[offdiag_x](GroundControl::MAG_CAL_REPORT::offdiag_x).

pub struct Item__ad_hoc2365<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2365<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 31, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 31);
    }
}

///[offdiag_y](GroundControl::MAG_CAL_REPORT::offdiag_y).

pub struct Item__ad_hoc2366<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2366<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 35, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 35);
    }
}

///[offdiag_z](GroundControl::MAG_CAL_REPORT::offdiag_z).

pub struct Item__ad_hoc2367<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2367<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 39, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 39);
    }
}

///[cal_status](GroundControl::MAG_CAL_REPORT::cal_status).

pub struct Item__ad_hoc83<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc83<'a> {
    pub fn get(&mut self) -> Option<packs::MAG_CAL_STATUS> {
        let src = &mut self.0;
        if src.base.field_bit != 344 && !src.set_field(344, -1) { return None; }

        Some({ packs::MAG_CAL_STATUS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAG_CAL_STATUS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 344
        {
            dst.set_field(344, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metalog_request_list(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalog_request_list {}

pub static mut LOG_REQUEST_LIST: Metalog_request_list = Metalog_request_list(37, 2, 0, 0, 6, None, 0, 0, 0, []);

///[start](GroundControl::LOG_REQUEST_LIST::start).

pub struct Item__ad_hoc1665(pub *mut u8);

impl Item__ad_hoc1665
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[end](GroundControl::LOG_REQUEST_LIST::end).

pub struct Item__ad_hoc1666(pub *mut u8);

impl Item__ad_hoc1666
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[target_system](GroundControl::LOG_REQUEST_LIST::target_system).

pub struct Item__ad_hoc1663(pub *mut u8);

impl Item__ad_hoc1663
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[target_component](GroundControl::LOG_REQUEST_LIST::target_component).

pub struct Item__ad_hoc1664(pub *mut u8);

impl Item__ad_hoc1664
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

#[repr(C)]
pub struct Metascaled_pressure(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metascaled_pressure {}

pub static mut SCALED_PRESSURE: Metascaled_pressure = Metascaled_pressure(18, 0, 1, 0, 14, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metav2_extension(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metav2_extension {}

pub static mut V2_EXTENSION: Metav2_extension = Metav2_extension(174, 1, 0, 0, 254, None, 0, 0, 0, []);

///[message_type](GroundControl::V2_EXTENSION::message_type).

pub struct Item__ad_hoc1974(pub *mut u8);

impl Item__ad_hoc1974
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[target_network](GroundControl::V2_EXTENSION::target_network).

pub struct Item__ad_hoc1971(pub *mut u8);

impl Item__ad_hoc1971
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[target_system](GroundControl::V2_EXTENSION::target_system).

pub struct Item__ad_hoc1972(pub *mut u8);

impl Item__ad_hoc1972
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

///[target_component](GroundControl::V2_EXTENSION::target_component).

pub struct Item__ad_hoc1973(pub *mut u8);

impl Item__ad_hoc1973
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[payload](GroundControl::V2_EXTENSION::payload).

pub struct ItemArray__ad_hoc1975 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1975 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1975 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaheartbeat(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 4]);

unsafe impl std::marker::Sync for Metaheartbeat {}

pub static mut HEARTBEAT: Metaheartbeat = Metaheartbeat(125, 0, 1, 0, 6, None, 2, 42, 4, [(&fld__ad_hoc688 as *const _ad_hoc688) as *const sys::Field,
    (&fld__ad_hoc686 as *const _ad_hoc686) as *const sys::Field,
    (&fld__ad_hoc685 as *const _ad_hoc685) as *const sys::Field,
    (&fld__ad_hoc687 as *const _ad_hoc687) as *const sys::Field]);

#[repr(C)]
pub struct Metaparam_map_rc(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaparam_map_rc {}

pub static mut PARAM_MAP_RC: Metaparam_map_rc = Metaparam_map_rc(29, 0, 0, 0, 22, None, 2, 170, 1, [(&fld__ad_hoc1188 as *const _ad_hoc1188) as *const sys::Field]);

#[repr(C)]
pub struct Metapower_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metapower_status {}

pub static mut POWER_STATUS: Metapower_status = Metapower_status(156, 2, 0, 0, 5, None, 0, 32, 1, [(&fld__ad_hoc140 as *const _ad_hoc140) as *const sys::Field]);

///[Vcc](GroundControl::POWER_STATUS::Vcc).

pub struct Item__ad_hoc1700<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1700<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[Vservo](GroundControl::POWER_STATUS::Vservo).

pub struct Item__ad_hoc1701<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1701<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[flags](GroundControl::POWER_STATUS::flags).

pub struct Item__ad_hoc140<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc140<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_POWER_STATUS> {
        let src = &mut self.0;
        if src.base.field_bit != 32 && !src.set_field(32, -1) { return None; }

        Some({ packs::MAV_POWER_STATUS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_POWER_STATUS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 32
        {
            dst.set_field(32, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaremote_log_data_block(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaremote_log_data_block {}

pub static mut REMOTE_LOG_DATA_BLOCK: Metaremote_log_data_block = Metaremote_log_data_block(144, 0, 0, 0, 203, None, 0, 1616, 1, [(&fld__ad_hoc219 as *const _ad_hoc219) as *const sys::Field]);

///[target_system](GroundControl::REMOTE_LOG_DATA_BLOCK::target_system).

pub struct Item__ad_hoc2335<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2335<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[target_component](GroundControl::REMOTE_LOG_DATA_BLOCK::target_component).

pub struct Item__ad_hoc2336<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2336<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[daTa](GroundControl::REMOTE_LOG_DATA_BLOCK::daTa).

pub struct ItemArray__ad_hoc2337 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2337 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2337 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[seqno](GroundControl::REMOTE_LOG_DATA_BLOCK::seqno).

pub struct Item__ad_hoc219<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc219<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS> {
        let src = &mut self.0;
        if src.base.field_bit != 1616 && !src.set_field(1616, -1) { return None; }

        Some({ packs::MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as u32).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1616
        {
            dst.set_field(1616, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metalogging_data_acked(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalogging_data_acked {}

pub static mut LOGGING_DATA_ACKED: Metalogging_data_acked = Metalogging_data_acked(217, 1, 0, 0, 255, None, 0, 0, 0, []);

///[sequence](GroundControl::LOGGING_DATA_ACKED::sequence).

pub struct Item__ad_hoc2060(pub *mut u8);

impl Item__ad_hoc2060
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[target_system](GroundControl::LOGGING_DATA_ACKED::target_system).

pub struct Item__ad_hoc2058(pub *mut u8);

impl Item__ad_hoc2058
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[target_component](GroundControl::LOGGING_DATA_ACKED::target_component).

pub struct Item__ad_hoc2059(pub *mut u8);

impl Item__ad_hoc2059
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

///[length](GroundControl::LOGGING_DATA_ACKED::length).

pub struct Item__ad_hoc2061(pub *mut u8);

impl Item__ad_hoc2061
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[first_message_offset](GroundControl::LOGGING_DATA_ACKED::first_message_offset).

pub struct Item__ad_hoc2062(pub *mut u8);

impl Item__ad_hoc2062
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

///[daTa](GroundControl::LOGGING_DATA_ACKED::daTa).

pub struct ItemArray__ad_hoc2063 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2063 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2063 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaterrain_check(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaterrain_check {}

pub static mut TERRAIN_CHECK: Metaterrain_check = Metaterrain_check(152, 0, 0, 0, 8, None, 0, 0, 0, []);

///[lat](GroundControl::TERRAIN_CHECK::lat).

pub struct Item__ad_hoc1766(pub *mut u8);

impl Item__ad_hoc1766
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[lon](GroundControl::TERRAIN_CHECK::lon).

pub struct Item__ad_hoc1767(pub *mut u8);

impl Item__ad_hoc1767
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 4);
    }
}

#[repr(C)]
pub struct Metamount_configure(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamount_configure {}

pub static mut MOUNT_CONFIGURE: Metamount_configure = Metamount_configure(40, 0, 0, 0, 6, None, 0, 40, 1, [(&fld__ad_hoc106 as *const _ad_hoc106) as *const sys::Field]);

///[target_system](GroundControl::MOUNT_CONFIGURE::target_system).

pub struct Item__ad_hoc2188<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2188<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[target_component](GroundControl::MOUNT_CONFIGURE::target_component).

pub struct Item__ad_hoc2189<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2189<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[stab_roll](GroundControl::MOUNT_CONFIGURE::stab_roll).

pub struct Item__ad_hoc2190<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2190<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 2);
    }
}

///[stab_pitch](GroundControl::MOUNT_CONFIGURE::stab_pitch).

pub struct Item__ad_hoc2191<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2191<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 3);
    }
}

///[stab_yaw](GroundControl::MOUNT_CONFIGURE::stab_yaw).

pub struct Item__ad_hoc2192<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2192<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 4);
    }
}

///[mount_mode](GroundControl::MOUNT_CONFIGURE::mount_mode).

pub struct Item__ad_hoc106<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc106<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_MOUNT_MODE> {
        let src = &mut self.0;
        if src.base.field_bit != 40 && !src.set_field(40, -1) { return None; }

        Some({ packs::MAV_MOUNT_MODE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_MOUNT_MODE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 40
        {
            dst.set_field(40, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metamission_request_int(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                   u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamission_request_int {}

pub static mut MISSION_REQUEST_INT: Metamission_request_int = Metamission_request_int(188, 1, 0, 0, 5, None, 0, 32, 1, [(&fld__ad_hoc105 as *const _ad_hoc105) as *const sys::Field]);

#[repr(C)]
pub struct Metalocal_position_ned_system_global_offset(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                                       u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalocal_position_ned_system_global_offset {}

pub static mut LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: Metalocal_position_ned_system_global_offset = Metalocal_position_ned_system_global_offset(158, 0, 1, 0, 28, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metacommand_ack(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 6]);

unsafe impl std::marker::Sync for Metacommand_ack {}

pub static mut COMMAND_ACK: Metacommand_ack = Metacommand_ack(121, 0, 0, 0, 1, None, 2, 2, 6, [(&fld__ad_hoc424 as *const _ad_hoc424) as *const sys::Field,
    (&fld__ad_hoc423 as *const _ad_hoc423) as *const sys::Field,
    (&fld__ad_hoc1330 as *const _ad_hoc1330) as *const sys::Field,
    (&fld__ad_hoc1331 as *const _ad_hoc1331) as *const sys::Field,
    (&fld__ad_hoc1332 as *const _ad_hoc1332) as *const sys::Field,
    (&fld__ad_hoc1333 as *const _ad_hoc1333) as *const sys::Field]);

#[repr(C)]
pub struct Metadata_stream(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadata_stream {}

pub static mut DATA_STREAM: Metadata_stream = Metadata_stream(31, 1, 0, 0, 4, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metamission_request(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamission_request {}

pub static mut MISSION_REQUEST: Metamission_request = Metamission_request(147, 1, 0, 0, 5, None, 0, 32, 1, [(&fld__ad_hoc218 as *const _ad_hoc218) as *const sys::Field]);

#[repr(C)]
pub struct Metaterrain_report(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaterrain_report {}

pub static mut TERRAIN_REPORT: Metaterrain_report = Metaterrain_report(86, 3, 0, 0, 22, None, 0, 0, 0, []);

///[spacing](GroundControl::TERRAIN_REPORT::spacing).

pub struct Item__ad_hoc1770(pub *mut u8);

impl Item__ad_hoc1770
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[pending](GroundControl::TERRAIN_REPORT::pending).

pub struct Item__ad_hoc1773(pub *mut u8);

impl Item__ad_hoc1773
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[loaded](GroundControl::TERRAIN_REPORT::loaded).

pub struct Item__ad_hoc1774(pub *mut u8);

impl Item__ad_hoc1774
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[lat](GroundControl::TERRAIN_REPORT::lat).

pub struct Item__ad_hoc1768(pub *mut u8);

impl Item__ad_hoc1768
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[lon](GroundControl::TERRAIN_REPORT::lon).

pub struct Item__ad_hoc1769(pub *mut u8);

impl Item__ad_hoc1769
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 10);
    }
}

///[terrain_height](GroundControl::TERRAIN_REPORT::terrain_height).

pub struct Item__ad_hoc1771(pub *mut u8);

impl Item__ad_hoc1771
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 14, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 14); }
}

///[current_height](GroundControl::TERRAIN_REPORT::current_height).

pub struct Item__ad_hoc1772(pub *mut u8);

impl Item__ad_hoc1772
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 18, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 18); }
}

#[repr(C)]
pub struct Metaset_home_position(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaset_home_position {}

pub static mut SET_HOME_POSITION: Metaset_home_position = Metaset_home_position(77, 0, 0, 0, 54, None, 0, 424, 1, [(&fld__ad_hoc1954 as *const _ad_hoc1954) as *const sys::Field]);

///[target_system](GroundControl::SET_HOME_POSITION::target_system).

pub struct Item__ad_hoc1943<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1943<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[latitude](GroundControl::SET_HOME_POSITION::latitude).

pub struct Item__ad_hoc1944<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1944<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 1);
    }
}

///[longitude](GroundControl::SET_HOME_POSITION::longitude).

pub struct Item__ad_hoc1945<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1945<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 5, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 5);
    }
}

///[altitude](GroundControl::SET_HOME_POSITION::altitude).

pub struct Item__ad_hoc1946<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1946<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 9, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 9);
    }
}

///[x](GroundControl::SET_HOME_POSITION::x).

pub struct Item__ad_hoc1947<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1947<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 13, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 13);
    }
}

///[y](GroundControl::SET_HOME_POSITION::y).

pub struct Item__ad_hoc1948<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1948<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 17, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 17);
    }
}

///[z](GroundControl::SET_HOME_POSITION::z).

pub struct Item__ad_hoc1949<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1949<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 21, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 21);
    }
}

///[q](GroundControl::SET_HOME_POSITION::q).

pub struct ItemArray__ad_hoc1950 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1950 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc1950 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[approach_x](GroundControl::SET_HOME_POSITION::approach_x).

pub struct Item__ad_hoc1951<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1951<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 41, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 41);
    }
}

///[approach_y](GroundControl::SET_HOME_POSITION::approach_y).

pub struct Item__ad_hoc1952<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1952<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 45, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 45);
    }
}

///[approach_z](GroundControl::SET_HOME_POSITION::approach_z).

pub struct Item__ad_hoc1953<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1953<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 49, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 49);
    }
}

///[time_usec](GroundControl::SET_HOME_POSITION::time_usec).

pub struct Item__ad_hoc1954<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1954<'a> {
    pub fn get(&mut self) -> Option<i64> {
        let src = &mut self.0;
        if src.base.field_bit != 424 && !src.set_field(424, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 8 as usize) as i64;
            (dst) as i64
        })
    }

    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        if dst.base.field_bit != 424
        {
            dst.set_field(424, 0i32);
        }


        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, dst.BYTE);
    }
}

unsafe extern "C" fn metaswitchmodecommandalloc(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack {
    transmute(&host::GroundControl::SwitchModeCommand::meta_)
}

#[repr(C)]
pub struct Metaswitchmodecommand(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaswitchmodecommand {}

pub static mut SwitchModeCommand: Metaswitchmodecommand = Metaswitchmodecommand(28, 0, 0, 0, 0, Some(metaswitchmodecommandalloc), 0, 0, 0, []);

#[repr(C)]
pub struct Metahil_rc_inputs_raw(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metahil_rc_inputs_raw {}

pub static mut HIL_RC_INPUTS_RAW: Metahil_rc_inputs_raw = Metahil_rc_inputs_raw(184, 12, 0, 1, 33, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metascaled_imu3(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metascaled_imu3 {}

pub static mut SCALED_IMU3: Metascaled_imu3 = Metascaled_imu3(2, 0, 1, 0, 22, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::SCALED_IMU3::time_boot_ms).

pub struct Item__ad_hoc1732(pub *mut u8);

impl Item__ad_hoc1732
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[xacc](GroundControl::SCALED_IMU3::xacc).

pub struct Item__ad_hoc1733(pub *mut u8);

impl Item__ad_hoc1733
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[yacc](GroundControl::SCALED_IMU3::yacc).

pub struct Item__ad_hoc1734(pub *mut u8);

impl Item__ad_hoc1734
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 6);
    }
}

///[zacc](GroundControl::SCALED_IMU3::zacc).

pub struct Item__ad_hoc1735(pub *mut u8);

impl Item__ad_hoc1735
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 8);
    }
}

///[xgyro](GroundControl::SCALED_IMU3::xgyro).

pub struct Item__ad_hoc1736(pub *mut u8);

impl Item__ad_hoc1736
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 10);
    }
}

///[ygyro](GroundControl::SCALED_IMU3::ygyro).

pub struct Item__ad_hoc1737(pub *mut u8);

impl Item__ad_hoc1737
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 12, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 12);
    }
}

///[zgyro](GroundControl::SCALED_IMU3::zgyro).

pub struct Item__ad_hoc1738(pub *mut u8);

impl Item__ad_hoc1738
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 14, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 14);
    }
}

///[xmag](GroundControl::SCALED_IMU3::xmag).

pub struct Item__ad_hoc1739(pub *mut u8);

impl Item__ad_hoc1739
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 16);
    }
}

///[ymag](GroundControl::SCALED_IMU3::ymag).

pub struct Item__ad_hoc1740(pub *mut u8);

impl Item__ad_hoc1740
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 18, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 18);
    }
}

///[zmag](GroundControl::SCALED_IMU3::zmag).

pub struct Item__ad_hoc1741(pub *mut u8);

impl Item__ad_hoc1741
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 20, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 20);
    }
}

#[repr(C)]
pub struct Metaset_mode(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaset_mode {}

pub static mut SET_MODE: Metaset_mode = Metaset_mode(7, 0, 1, 0, 6, None, 0, 40, 1, [(&fld__ad_hoc80 as *const _ad_hoc80) as *const sys::Field]);

#[repr(C)]
pub struct Metamount_control(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamount_control {}

pub static mut MOUNT_CONTROL: Metamount_control = Metamount_control(113, 0, 0, 0, 15, None, 0, 0, 0, []);

///[target_system](GroundControl::MOUNT_CONTROL::target_system).

pub struct Item__ad_hoc2193(pub *mut u8);

impl Item__ad_hoc2193
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::MOUNT_CONTROL::target_component).

pub struct Item__ad_hoc2194(pub *mut u8);

impl Item__ad_hoc2194
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[input_a](GroundControl::MOUNT_CONTROL::input_a).

pub struct Item__ad_hoc2195(pub *mut u8);

impl Item__ad_hoc2195
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 2);
    }
}

///[input_b](GroundControl::MOUNT_CONTROL::input_b).

pub struct Item__ad_hoc2196(pub *mut u8);

impl Item__ad_hoc2196
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[input_c](GroundControl::MOUNT_CONTROL::input_c).

pub struct Item__ad_hoc2197(pub *mut u8);

impl Item__ad_hoc2197
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 10);
    }
}

///[save_position](GroundControl::MOUNT_CONTROL::save_position).

pub struct Item__ad_hoc2198(pub *mut u8);

impl Item__ad_hoc2198
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 14, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 14);
    }
}

#[repr(C)]
pub struct Metaposition_target_global_int(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                          u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaposition_target_global_int {}

pub static mut POSITION_TARGET_GLOBAL_INT: Metaposition_target_global_int = Metaposition_target_global_int(133, 1, 1, 0, 51, None, 0, 400, 1, [(&fld__ad_hoc376 as *const _ad_hoc376) as *const sys::Field]);

#[repr(C)]
pub struct Metaled_control(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaled_control {}

pub static mut LED_CONTROL: Metaled_control = Metaled_control(177, 0, 0, 0, 29, None, 0, 0, 0, []);

///[target_system](GroundControl::LED_CONTROL::target_system).

pub struct Item__ad_hoc2341(pub *mut u8);

impl Item__ad_hoc2341
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::LED_CONTROL::target_component).

pub struct Item__ad_hoc2342(pub *mut u8);

impl Item__ad_hoc2342
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[instance](GroundControl::LED_CONTROL::instance).

pub struct Item__ad_hoc2343(pub *mut u8);

impl Item__ad_hoc2343
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[pattern](GroundControl::LED_CONTROL::pattern).

pub struct Item__ad_hoc2344(pub *mut u8);

impl Item__ad_hoc2344
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

///[custom_len](GroundControl::LED_CONTROL::custom_len).

pub struct Item__ad_hoc2345(pub *mut u8);

impl Item__ad_hoc2345
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[custom_bytes](GroundControl::LED_CONTROL::custom_bytes).

pub struct ItemArray__ad_hoc2346 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2346 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2346 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metasim_state(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metasim_state {}

pub static mut SIM_STATE: Metasim_state = Metasim_state(108, 0, 0, 0, 84, None, 0, 0, 0, []);

///[q1](GroundControl::SIM_STATE::q1).

pub struct Item__ad_hoc1576(pub *mut u8);

impl Item__ad_hoc1576
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[q2](GroundControl::SIM_STATE::q2).

pub struct Item__ad_hoc1577(pub *mut u8);

impl Item__ad_hoc1577
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[q3](GroundControl::SIM_STATE::q3).

pub struct Item__ad_hoc1578(pub *mut u8);

impl Item__ad_hoc1578
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[q4](GroundControl::SIM_STATE::q4).

pub struct Item__ad_hoc1579(pub *mut u8);

impl Item__ad_hoc1579
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[roll](GroundControl::SIM_STATE::roll).

pub struct Item__ad_hoc1580(pub *mut u8);

impl Item__ad_hoc1580
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[pitch](GroundControl::SIM_STATE::pitch).

pub struct Item__ad_hoc1581(pub *mut u8);

impl Item__ad_hoc1581
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[yaw](GroundControl::SIM_STATE::yaw).

pub struct Item__ad_hoc1582(pub *mut u8);

impl Item__ad_hoc1582
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[xacc](GroundControl::SIM_STATE::xacc).

pub struct Item__ad_hoc1583(pub *mut u8);

impl Item__ad_hoc1583
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[yacc](GroundControl::SIM_STATE::yacc).

pub struct Item__ad_hoc1584(pub *mut u8);

impl Item__ad_hoc1584
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

///[zacc](GroundControl::SIM_STATE::zacc).

pub struct Item__ad_hoc1585(pub *mut u8);

impl Item__ad_hoc1585
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 36); }
}

///[xgyro](GroundControl::SIM_STATE::xgyro).

pub struct Item__ad_hoc1586(pub *mut u8);

impl Item__ad_hoc1586
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 40); }
}

///[ygyro](GroundControl::SIM_STATE::ygyro).

pub struct Item__ad_hoc1587(pub *mut u8);

impl Item__ad_hoc1587
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 44, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 44); }
}

///[zgyro](GroundControl::SIM_STATE::zgyro).

pub struct Item__ad_hoc1588(pub *mut u8);

impl Item__ad_hoc1588
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 48, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 48); }
}

///[lat](GroundControl::SIM_STATE::lat).

pub struct Item__ad_hoc1589(pub *mut u8);

impl Item__ad_hoc1589
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 52, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 52); }
}

///[lon](GroundControl::SIM_STATE::lon).

pub struct Item__ad_hoc1590(pub *mut u8);

impl Item__ad_hoc1590
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 56, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 56); }
}

///[alt](GroundControl::SIM_STATE::alt).

pub struct Item__ad_hoc1591(pub *mut u8);

impl Item__ad_hoc1591
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 60, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 60); }
}

///[std_dev_horz](GroundControl::SIM_STATE::std_dev_horz).

pub struct Item__ad_hoc1592(pub *mut u8);

impl Item__ad_hoc1592
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 64, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 64); }
}

///[std_dev_vert](GroundControl::SIM_STATE::std_dev_vert).

pub struct Item__ad_hoc1593(pub *mut u8);

impl Item__ad_hoc1593
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 68, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 68); }
}

///[vn](GroundControl::SIM_STATE::vn).

pub struct Item__ad_hoc1594(pub *mut u8);

impl Item__ad_hoc1594
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 72, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 72); }
}

///[ve](GroundControl::SIM_STATE::ve).

pub struct Item__ad_hoc1595(pub *mut u8);

impl Item__ad_hoc1595
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 76, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 76); }
}

///[vd](GroundControl::SIM_STATE::vd).

pub struct Item__ad_hoc1596(pub *mut u8);

impl Item__ad_hoc1596
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 80, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 80); }
}

#[repr(C)]
pub struct Metawifi_config_ap(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metawifi_config_ap {}

pub static mut WIFI_CONFIG_AP: Metawifi_config_ap = Metawifi_config_ap(194, 0, 0, 0, 1, None, 2, 2, 2, [(&fld__ad_hoc2084 as *const _ad_hoc2084) as *const sys::Field,
    (&fld__ad_hoc2085 as *const _ad_hoc2085) as *const sys::Field]);

///[ssid](GroundControl::WIFI_CONFIG_AP::ssid).

pub struct Item__ad_hoc2084<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2084<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 2 && !src.set_field(2, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(2, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[password](GroundControl::WIFI_CONFIG_AP::password).

pub struct Item__ad_hoc2085<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2085<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 3 && !src.set_field(3, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(3, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metadata96(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                      u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadata96 {}

pub static mut DATA96: Metadata96 = Metadata96(83, 0, 0, 0, 98, None, 0, 0, 0, []);

///[typE](GroundControl::DATA96::typE).

pub struct Item__ad_hoc2260(pub *mut u8);

impl Item__ad_hoc2260
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[len](GroundControl::DATA96::len).

pub struct Item__ad_hoc2261(pub *mut u8);

impl Item__ad_hoc2261
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[daTa](GroundControl::DATA96::daTa).

pub struct ItemArray__ad_hoc2262 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2262 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2262 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaflight_information(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaflight_information {}

pub static mut FLIGHT_INFORMATION: Metaflight_information = Metaflight_information(215, 0, 1, 3, 28, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::FLIGHT_INFORMATION::time_boot_ms).

pub struct Item__ad_hoc2044(pub *mut u8);

impl Item__ad_hoc2044
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[arming_time_utc](GroundControl::FLIGHT_INFORMATION::arming_time_utc).

pub struct Item__ad_hoc2045(pub *mut u8);

impl Item__ad_hoc2045
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 4);
    }
}

///[takeoff_time_utc](GroundControl::FLIGHT_INFORMATION::takeoff_time_utc).

pub struct Item__ad_hoc2046(pub *mut u8);

impl Item__ad_hoc2046
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 12, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 12);
    }
}

///[flight_uuid](GroundControl::FLIGHT_INFORMATION::flight_uuid).

pub struct Item__ad_hoc2047(pub *mut u8);

impl Item__ad_hoc2047
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 20, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 20);
    }
}

#[repr(C)]
pub struct Metarc_channels_raw(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metarc_channels_raw {}

pub static mut RC_CHANNELS_RAW: Metarc_channels_raw = Metarc_channels_raw(148, 8, 1, 0, 22, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metaservo_output_raw(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 8]);

unsafe impl std::marker::Sync for Metaservo_output_raw {}

pub static mut SERVO_OUTPUT_RAW: Metaservo_output_raw = Metaservo_output_raw(79, 8, 1, 0, 22, None, 0, 168, 8, [(&fld__ad_hoc1132 as *const _ad_hoc1132) as *const sys::Field,
    (&fld__ad_hoc1133 as *const _ad_hoc1133) as *const sys::Field,
    (&fld__ad_hoc1134 as *const _ad_hoc1134) as *const sys::Field,
    (&fld__ad_hoc1135 as *const _ad_hoc1135) as *const sys::Field,
    (&fld__ad_hoc1136 as *const _ad_hoc1136) as *const sys::Field,
    (&fld__ad_hoc1137 as *const _ad_hoc1137) as *const sys::Field,
    (&fld__ad_hoc1138 as *const _ad_hoc1138) as *const sys::Field,
    (&fld__ad_hoc1139 as *const _ad_hoc1139) as *const sys::Field]);

#[repr(C)]
pub struct Metameminfo(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                       u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metameminfo {}

pub static mut MEMINFO: Metameminfo = Metameminfo(159, 2, 0, 0, 5, None, 0, 32, 1, [(&fld__ad_hoc2160 as *const _ad_hoc2160) as *const sys::Field]);

///[brkval](GroundControl::MEMINFO::brkval).

pub struct Item__ad_hoc2158<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2158<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[freemem](GroundControl::MEMINFO::freemem).

pub struct Item__ad_hoc2159<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2159<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[freemem32](GroundControl::MEMINFO::freemem32).

pub struct Item__ad_hoc2160<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2160<'a> {
    pub fn get(&mut self) -> Option<i32> {
        let src = &mut self.0;
        if src.base.field_bit != 32 && !src.set_field(32, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 4 as usize) as i32;
            (dst) as i32
        })
    }

    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        if dst.base.field_bit != 32
        {
            dst.set_field(32, 0i32);
        }


        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, dst.BYTE);
    }
}

#[repr(C)]
pub struct Metamission_item_reached(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                    u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamission_item_reached {}

pub static mut MISSION_ITEM_REACHED: Metamission_item_reached = Metamission_item_reached(81, 1, 0, 0, 2, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metalogging_ack(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalogging_ack {}

pub static mut LOGGING_ACK: Metalogging_ack = Metalogging_ack(220, 1, 0, 0, 4, None, 0, 0, 0, []);

///[sequence](GroundControl::LOGGING_ACK::sequence).

pub struct Item__ad_hoc2066(pub *mut u8);

impl Item__ad_hoc2066
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[target_system](GroundControl::LOGGING_ACK::target_system).

pub struct Item__ad_hoc2064(pub *mut u8);

impl Item__ad_hoc2064
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[target_component](GroundControl::LOGGING_ACK::target_component).

pub struct Item__ad_hoc2065(pub *mut u8);

impl Item__ad_hoc2065
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

#[repr(C)]
pub struct Metavision_speed_estimate(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metavision_speed_estimate {}

pub static mut VISION_SPEED_ESTIMATE: Metavision_speed_estimate = Metavision_speed_estimate(42, 0, 0, 1, 20, None, 0, 0, 0, []);

///[usec](GroundControl::VISION_SPEED_ESTIMATE::usec).

pub struct Item__ad_hoc1523(pub *mut u8);

impl Item__ad_hoc1523
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[x](GroundControl::VISION_SPEED_ESTIMATE::x).

pub struct Item__ad_hoc1524(pub *mut u8);

impl Item__ad_hoc1524
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[y](GroundControl::VISION_SPEED_ESTIMATE::y).

pub struct Item__ad_hoc1525(pub *mut u8);

impl Item__ad_hoc1525
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[z](GroundControl::VISION_SPEED_ESTIMATE::z).

pub struct Item__ad_hoc1526(pub *mut u8);

impl Item__ad_hoc1526
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

#[repr(C)]
pub struct Metadebug_vect(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                          u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metadebug_vect {}

pub static mut DEBUG_VECT: Metadebug_vect = Metadebug_vect(187, 0, 0, 1, 21, None, 2, 162, 1, [(&fld__ad_hoc1980 as *const _ad_hoc1980) as *const sys::Field]);

///[time_usec](GroundControl::DEBUG_VECT::time_usec).

pub struct Item__ad_hoc1981<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1981<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 0);
    }
}

///[x](GroundControl::DEBUG_VECT::x).

pub struct Item__ad_hoc1982<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1982<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 8);
    }
}

///[y](GroundControl::DEBUG_VECT::y).

pub struct Item__ad_hoc1983<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1983<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 12);
    }
}

///[z](GroundControl::DEBUG_VECT::z).

pub struct Item__ad_hoc1984<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1984<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 16);
    }
}

///[name](GroundControl::DEBUG_VECT::name).

pub struct Item__ad_hoc1980<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1980<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 162 && !src.set_field(162, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(162, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metalog_request_end(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalog_request_end {}

pub static mut LOG_REQUEST_END: Metalog_request_end = Metalog_request_end(102, 0, 0, 0, 2, None, 0, 0, 0, []);

///[target_system](GroundControl::LOG_REQUEST_END::target_system).

pub struct Item__ad_hoc1683(pub *mut u8);

impl Item__ad_hoc1683
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::LOG_REQUEST_END::target_component).

pub struct Item__ad_hoc1684(pub *mut u8);

impl Item__ad_hoc1684
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

#[repr(C)]
pub struct Metamission_ack(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metamission_ack {}

pub static mut MISSION_ACK: Metamission_ack = Metamission_ack(85, 0, 0, 0, 3, None, 2, 18, 2, [(&fld__ad_hoc192 as *const _ad_hoc192) as *const sys::Field,
    (&fld__ad_hoc193 as *const _ad_hoc193) as *const sys::Field]);

#[repr(C)]
pub struct Metachange_operator_control_ack(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metachange_operator_control_ack {}

pub static mut CHANGE_OPERATOR_CONTROL_ACK: Metachange_operator_control_ack = Metachange_operator_control_ack(62, 0, 0, 0, 3, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metamission_current(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamission_current {}

pub static mut MISSION_CURRENT: Metamission_current = Metamission_current(189, 1, 0, 0, 2, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metasystem_time(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metasystem_time {}

pub static mut SYSTEM_TIME: Metasystem_time = Metasystem_time(9, 0, 1, 1, 12, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metacamera_trigger(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metacamera_trigger {}

pub static mut CAMERA_TRIGGER: Metacamera_trigger = Metacamera_trigger(191, 0, 1, 1, 12, None, 0, 0, 0, []);

///[seq](GroundControl::CAMERA_TRIGGER::seq).

pub struct Item__ad_hoc1611(pub *mut u8);

impl Item__ad_hoc1611
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[time_usec](GroundControl::CAMERA_TRIGGER::time_usec).

pub struct Item__ad_hoc1610(pub *mut u8);

impl Item__ad_hoc1610
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 4);
    }
}

#[repr(C)]
pub struct Metagopro_set_response(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metagopro_set_response {}

pub static mut GOPRO_SET_RESPONSE: Metagopro_set_response = Metagopro_set_response(192, 0, 0, 0, 1, None, 0, 0, 2, [(&fld__ad_hoc882 as *const _ad_hoc882) as *const sys::Field,
    (&fld__ad_hoc883 as *const _ad_hoc883) as *const sys::Field]);

///[cmd_id](GroundControl::GOPRO_SET_RESPONSE::cmd_id).

pub struct Item__ad_hoc882<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc882<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_COMMAND> {
        let src = &mut self.0;
        if src.base.field_bit != 0 && !src.set_field(0, -1) { return None; }

        Some({ packs::GOPRO_COMMAND::from_bits((sys::get_bits(src.base.bytes, src.BIT, 5)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_COMMAND) {
        let dst = &mut self.0;
        if dst.base.field_bit != 0
        {
            dst.set_field(0, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 5, dst.base.bytes, dst.BIT);
    }
}

///[status](GroundControl::GOPRO_SET_RESPONSE::status).

pub struct Item__ad_hoc883<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc883<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_REQUEST_STATUS> {
        let src = &mut self.0;
        if src.base.field_bit != 1 && !src.set_field(1, -1) { return None; }

        Some({ packs::GOPRO_REQUEST_STATUS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_REQUEST_STATUS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1
        {
            dst.set_field(1, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metavision_position_estimate(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metavision_position_estimate {}

pub static mut VISION_POSITION_ESTIMATE: Metavision_position_estimate = Metavision_position_estimate(16, 0, 0, 1, 32, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metamanual_control(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamanual_control {}

pub static mut MANUAL_CONTROL: Metamanual_control = Metamanual_control(54, 1, 0, 0, 11, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metarc_channels(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metarc_channels {}

pub static mut RC_CHANNELS: Metarc_channels = Metarc_channels(23, 18, 1, 0, 42, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metaprotocol_version(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaprotocol_version {}

pub static mut PROTOCOL_VERSION: Metaprotocol_version = Metaprotocol_version(150, 3, 0, 0, 22, None, 0, 0, 0, []);

///[version](GroundControl::PROTOCOL_VERSION::version).

pub struct Item__ad_hoc2086(pub *mut u8);

impl Item__ad_hoc2086
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[min_version](GroundControl::PROTOCOL_VERSION::min_version).

pub struct Item__ad_hoc2087(pub *mut u8);

impl Item__ad_hoc2087
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[max_version](GroundControl::PROTOCOL_VERSION::max_version).

pub struct Item__ad_hoc2088(pub *mut u8);

impl Item__ad_hoc2088
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[spec_version_hash](GroundControl::PROTOCOL_VERSION::spec_version_hash).

pub struct ItemArray__ad_hoc2089 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2089 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2089 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[library_version_hash](GroundControl::PROTOCOL_VERSION::library_version_hash).

pub struct ItemArray__ad_hoc2090 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2090 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2090 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metarally_fetch_point(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metarally_fetch_point {}

pub static mut RALLY_FETCH_POINT: Metarally_fetch_point = Metarally_fetch_point(5, 0, 0, 0, 3, None, 0, 0, 0, []);

///[target_system](GroundControl::RALLY_FETCH_POINT::target_system).

pub struct Item__ad_hoc2286(pub *mut u8);

impl Item__ad_hoc2286
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::RALLY_FETCH_POINT::target_component).

pub struct Item__ad_hoc2287(pub *mut u8);

impl Item__ad_hoc2287
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[idx](GroundControl::RALLY_FETCH_POINT::idx).

pub struct Item__ad_hoc2288(pub *mut u8);

impl Item__ad_hoc2288
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

#[repr(C)]
pub struct Metaparam_value(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metaparam_value {}

pub static mut PARAM_VALUE: Metaparam_value = Metaparam_value(136, 2, 0, 0, 9, None, 2, 66, 2, [(&fld__ad_hoc1012 as *const _ad_hoc1012) as *const sys::Field,
    (&fld__ad_hoc966 as *const _ad_hoc966) as *const sys::Field]);

#[repr(C)]
pub struct Metabattery_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metabattery_status {}

pub static mut BATTERY_STATUS: Metabattery_status = Metabattery_status(111, 10, 0, 0, 35, None, 0, 272, 2, [(&fld__ad_hoc58 as *const _ad_hoc58) as *const sys::Field,
    (&fld__ad_hoc59 as *const _ad_hoc59) as *const sys::Field]);

///[voltages](GroundControl::BATTERY_STATUS::voltages).

pub struct ItemArray__ad_hoc1838 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1838 {
    pub fn get(&mut self, index: usize) -> i16 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, index: usize, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.bytes, self.offset + index * 2);
    }
}

impl Iterator for ItemArray__ad_hoc1838 {
    type Item = i16;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[id](GroundControl::BATTERY_STATUS::id).

pub struct Item__ad_hoc1836<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1836<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 20, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 20);
    }
}

///[temperature](GroundControl::BATTERY_STATUS::temperature).

pub struct Item__ad_hoc1837<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1837<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 21, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 21);
    }
}

///[current_battery](GroundControl::BATTERY_STATUS::current_battery).

pub struct Item__ad_hoc1839<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1839<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 23, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 23);
    }
}

///[current_consumed](GroundControl::BATTERY_STATUS::current_consumed).

pub struct Item__ad_hoc1840<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1840<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 25, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 25);
    }
}

///[energy_consumed](GroundControl::BATTERY_STATUS::energy_consumed).

pub struct Item__ad_hoc1841<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1841<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 29, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 29);
    }
}

///[battery_remaining](GroundControl::BATTERY_STATUS::battery_remaining).

pub struct Item__ad_hoc1842<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1842<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 33, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 33);
    }
}

///[battery_function](GroundControl::BATTERY_STATUS::battery_function).

pub struct Item__ad_hoc58<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc58<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_BATTERY_FUNCTION> {
        let src = &mut self.0;
        if src.base.field_bit != 272 && !src.set_field(272, -1) { return None; }

        Some({ packs::MAV_BATTERY_FUNCTION::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_BATTERY_FUNCTION) {
        let dst = &mut self.0;
        if dst.base.field_bit != 272
        {
            dst.set_field(272, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[typE](GroundControl::BATTERY_STATUS::typE).

pub struct Item__ad_hoc59<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc59<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_BATTERY_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 273 && !src.set_field(273, -1) { return None; }

        Some({ packs::MAV_BATTERY_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_BATTERY_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 273
        {
            dst.set_field(273, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaserial_control(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metaserial_control {}

pub static mut SERIAL_CONTROL: Metaserial_control = Metaserial_control(99, 1, 1, 0, 78, None, 0, 616, 2, [(&fld__ad_hoc706 as *const _ad_hoc706) as *const sys::Field,
    (&fld__ad_hoc705 as *const _ad_hoc705) as *const sys::Field]);

///[timeout](GroundControl::SERIAL_CONTROL::timeout).

pub struct Item__ad_hoc1702<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1702<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[baudrate](GroundControl::SERIAL_CONTROL::baudrate).

pub struct Item__ad_hoc1703<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1703<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 2);
    }
}

///[count](GroundControl::SERIAL_CONTROL::count).

pub struct Item__ad_hoc1704<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1704<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 6);
    }
}

///[daTa](GroundControl::SERIAL_CONTROL::daTa).

pub struct ItemArray__ad_hoc1705 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1705 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1705 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[device](GroundControl::SERIAL_CONTROL::device).

pub struct Item__ad_hoc706<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc706<'a> {
    pub fn get(&mut self) -> Option<packs::SERIAL_CONTROL_DEV> {
        let src = &mut self.0;
        if src.base.field_bit != 616 && !src.set_field(616, -1) { return None; }

        Some({ packs::SERIAL_CONTROL_DEV::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::SERIAL_CONTROL_DEV) {
        let dst = &mut self.0;
        if dst.base.field_bit != 616
        {
            dst.set_field(616, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[flags](GroundControl::SERIAL_CONTROL::flags).

pub struct Item__ad_hoc705<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc705<'a> {
    pub fn get(&mut self) -> Option<packs::SERIAL_CONTROL_FLAG> {
        let src = &mut self.0;
        if src.base.field_bit != 617 && !src.set_field(617, -1) { return None; }

        Some({ packs::SERIAL_CONTROL_FLAG::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::SERIAL_CONTROL_FLAG) {
        let dst = &mut self.0;
        if dst.base.field_bit != 617
        {
            dst.set_field(617, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaset_position_target_local_ned(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                             u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaset_position_target_local_ned {}

pub static mut SET_POSITION_TARGET_LOCAL_NED: Metaset_position_target_local_ned = Metaset_position_target_local_ned(209, 1, 1, 0, 53, None, 0, 416, 1, [(&fld__ad_hoc51 as *const _ad_hoc51) as *const sys::Field]);

#[repr(C)]
pub struct Metamount_orientation(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metamount_orientation {}

pub static mut MOUNT_ORIENTATION: Metamount_orientation = Metamount_orientation(76, 0, 1, 0, 16, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::MOUNT_ORIENTATION::time_boot_ms).

pub struct Item__ad_hoc2048(pub *mut u8);

impl Item__ad_hoc2048
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[roll](GroundControl::MOUNT_ORIENTATION::roll).

pub struct Item__ad_hoc2049(pub *mut u8);

impl Item__ad_hoc2049
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[pitch](GroundControl::MOUNT_ORIENTATION::pitch).

pub struct Item__ad_hoc2050(pub *mut u8);

impl Item__ad_hoc2050
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[yaw](GroundControl::MOUNT_ORIENTATION::yaw).

pub struct Item__ad_hoc2051(pub *mut u8);

impl Item__ad_hoc2051
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

#[repr(C)]
pub struct Metaset_gps_global_origin(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaset_gps_global_origin {}

pub static mut SET_GPS_GLOBAL_ORIGIN: Metaset_gps_global_origin = Metaset_gps_global_origin(53, 0, 0, 0, 14, None, 0, 104, 1, [(&fld__ad_hoc1181 as *const _ad_hoc1181) as *const sys::Field]);

#[repr(C)]
pub struct Metaparam_ext_set(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metaparam_ext_set {}

pub static mut PARAM_EXT_SET: Metaparam_ext_set = Metaparam_ext_set(109, 0, 0, 0, 3, None, 3, 19, 3, [(&fld__ad_hoc2116 as *const _ad_hoc2116) as *const sys::Field,
    (&fld__ad_hoc2117 as *const _ad_hoc2117) as *const sys::Field,
    (&fld__ad_hoc152 as *const _ad_hoc152) as *const sys::Field]);

///[target_system](GroundControl::PARAM_EXT_SET::target_system).

pub struct Item__ad_hoc2114<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2114<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[target_component](GroundControl::PARAM_EXT_SET::target_component).

pub struct Item__ad_hoc2115<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2115<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[param_id](GroundControl::PARAM_EXT_SET::param_id).

pub struct Item__ad_hoc2116<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2116<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 19 && !src.set_field(19, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(19, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[param_value](GroundControl::PARAM_EXT_SET::param_value).

pub struct Item__ad_hoc2117<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2117<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 20 && !src.set_field(20, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(20, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[param_type](GroundControl::PARAM_EXT_SET::param_type).

pub struct Item__ad_hoc152<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc152<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_PARAM_EXT_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 21 && !src.set_field(21, -1) { return None; }

        Some({ packs::MAV_PARAM_EXT_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_PARAM_EXT_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 21
        {
            dst.set_field(21, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaautopilot_version(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metaautopilot_version {}

pub static mut AUTOPILOT_VERSION: Metaautopilot_version = Metaautopilot_version(60, 2, 4, 1, 53, None, 3, 419, 2, [(&fld__ad_hoc281 as *const _ad_hoc281) as *const sys::Field,
    (&fld__ad_hoc1853 as *const _ad_hoc1853) as *const sys::Field]);

///[vendor_id](GroundControl::AUTOPILOT_VERSION::vendor_id).

pub struct Item__ad_hoc1850<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1850<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[product_id](GroundControl::AUTOPILOT_VERSION::product_id).

pub struct Item__ad_hoc1851<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1851<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[flight_sw_version](GroundControl::AUTOPILOT_VERSION::flight_sw_version).

pub struct Item__ad_hoc1843<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1843<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 4);
    }
}

///[middleware_sw_version](GroundControl::AUTOPILOT_VERSION::middleware_sw_version).

pub struct Item__ad_hoc1844<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1844<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 8, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 8);
    }
}

///[os_sw_version](GroundControl::AUTOPILOT_VERSION::os_sw_version).

pub struct Item__ad_hoc1845<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1845<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 12, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 12);
    }
}

///[board_version](GroundControl::AUTOPILOT_VERSION::board_version).

pub struct Item__ad_hoc1846<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1846<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 16, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 16);
    }
}

///[uid](GroundControl::AUTOPILOT_VERSION::uid).

pub struct Item__ad_hoc1852<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1852<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 20, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 20);
    }
}

///[flight_custom_version](GroundControl::AUTOPILOT_VERSION::flight_custom_version).

pub struct ItemArray__ad_hoc1847 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1847 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1847 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[middleware_custom_version](GroundControl::AUTOPILOT_VERSION::middleware_custom_version).

pub struct ItemArray__ad_hoc1848 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1848 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1848 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[os_custom_version](GroundControl::AUTOPILOT_VERSION::os_custom_version).

pub struct ItemArray__ad_hoc1849 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1849 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1849 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[capabilities](GroundControl::AUTOPILOT_VERSION::capabilities).

pub struct Item__ad_hoc281<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc281<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_PROTOCOL_CAPABILITY> {
        let src = &mut self.0;
        if src.base.field_bit != 419 && !src.set_field(419, -1) { return None; }

        Some({ packs::MAV_PROTOCOL_CAPABILITY::from_bits((sys::get_bits(src.base.bytes, src.BIT, 5)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_PROTOCOL_CAPABILITY) {
        let dst = &mut self.0;
        if dst.base.field_bit != 419
        {
            dst.set_field(419, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 5, dst.base.bytes, dst.BIT);
    }
}

///[uid2](GroundControl::AUTOPILOT_VERSION::uid2).

pub struct Item__ad_hoc1853<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1853<'a>
{
    pub fn set(&mut self, src: i8, d0: usize) {
        if self.0.base.field_bit != 420 { self.0.set_field(420, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0, -1) { dst.set_item(d0, 0); }

        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, dst.BYTE);
    }


    pub fn get(&mut self, d0: usize) -> Option<i8> {
        let src = &mut self.0;
        if (src.base.field_bit != 420 && !src.set_field(420, -1)) || !src.set_item(d0, -1) { return None; }

        Some({
            let dst = sys::get_bytes(src.base.bytes, src.BYTE, 1 as usize) as i8;
            (dst) as i8
        })
    }


    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize)>(&mut self, from_d0: usize, mut dst: DST) {
        for d0 in from_d0..packs::AUTOPILOT_VERSION::uid2::d0 {
            dst(self, d0);
        }
    }
}

pub struct FieldI__ad_hoc1853<'a> (pub &'a mut sys::Cursor);

impl<'a> FieldI__ad_hoc1853<'a> {
    pub fn items(self) -> Option<Item__ad_hoc1853<'a>> {
        return if self.0.base.field_bit != 420 && !self.0.set_field(420, -1) { None } else { Some(Item__ad_hoc1853(self.0)) };
    }

    pub fn set(&mut self, src: i8, d0: usize) {
        if self.0.base.field_bit != 420 { self.0.set_field(420, 0); }
        let dst = &mut self.0;


        if !dst.set_item(d0, -1) { dst.set_item(d0, 0); }

        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, dst.BYTE);
    }

    pub fn enumerate<DST: FnMut(&mut Self, /*d0:*/usize)>(&mut self, from_d0: usize, mut dst: DST) {
        for d0 in from_d0..packs::AUTOPILOT_VERSION::uid2::d0 {
            dst(self, d0);
        }
    }
}

#[repr(C)]
pub struct Metamission_request_list(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                    u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamission_request_list {}

pub static mut MISSION_REQUEST_LIST: Metamission_request_list = Metamission_request_list(25, 0, 0, 0, 3, None, 0, 16, 1, [(&fld__ad_hoc820 as *const _ad_hoc820) as *const sys::Field]);

#[repr(C)]
pub struct Metasimstate(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metasimstate {}

pub static mut SIMSTATE: Metasimstate = Metasimstate(112, 0, 0, 0, 44, None, 0, 0, 0, []);

///[roll](GroundControl::SIMSTATE::roll).

pub struct Item__ad_hoc2223(pub *mut u8);

impl Item__ad_hoc2223
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[pitch](GroundControl::SIMSTATE::pitch).

pub struct Item__ad_hoc2224(pub *mut u8);

impl Item__ad_hoc2224
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[yaw](GroundControl::SIMSTATE::yaw).

pub struct Item__ad_hoc2225(pub *mut u8);

impl Item__ad_hoc2225
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[xacc](GroundControl::SIMSTATE::xacc).

pub struct Item__ad_hoc2226(pub *mut u8);

impl Item__ad_hoc2226
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[yacc](GroundControl::SIMSTATE::yacc).

pub struct Item__ad_hoc2227(pub *mut u8);

impl Item__ad_hoc2227
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[zacc](GroundControl::SIMSTATE::zacc).

pub struct Item__ad_hoc2228(pub *mut u8);

impl Item__ad_hoc2228
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[xgyro](GroundControl::SIMSTATE::xgyro).

pub struct Item__ad_hoc2229(pub *mut u8);

impl Item__ad_hoc2229
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[ygyro](GroundControl::SIMSTATE::ygyro).

pub struct Item__ad_hoc2230(pub *mut u8);

impl Item__ad_hoc2230
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[zgyro](GroundControl::SIMSTATE::zgyro).

pub struct Item__ad_hoc2231(pub *mut u8);

impl Item__ad_hoc2231
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

///[lat](GroundControl::SIMSTATE::lat).

pub struct Item__ad_hoc2232(pub *mut u8);

impl Item__ad_hoc2232
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 36, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 36);
    }
}

///[lng](GroundControl::SIMSTATE::lng).

pub struct Item__ad_hoc2233(pub *mut u8);

impl Item__ad_hoc2233
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 40, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 40);
    }
}

#[repr(C)]
pub struct Metaset_video_stream_settings(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                         u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaset_video_stream_settings {}

pub static mut SET_VIDEO_STREAM_SETTINGS: Metaset_video_stream_settings = Metaset_video_stream_settings(22, 3, 1, 0, 18, None, 2, 138, 1, [(&fld__ad_hoc2083 as *const _ad_hoc2083) as *const sys::Field]);

///[resolution_h](GroundControl::SET_VIDEO_STREAM_SETTINGS::resolution_h).

pub struct Item__ad_hoc2079<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2079<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[resolution_v](GroundControl::SET_VIDEO_STREAM_SETTINGS::resolution_v).

pub struct Item__ad_hoc2080<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2080<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[rotation](GroundControl::SET_VIDEO_STREAM_SETTINGS::rotation).

pub struct Item__ad_hoc2082<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2082<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 4);
    }
}

///[bitrate](GroundControl::SET_VIDEO_STREAM_SETTINGS::bitrate).

pub struct Item__ad_hoc2081<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2081<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 6);
    }
}

///[target_system](GroundControl::SET_VIDEO_STREAM_SETTINGS::target_system).

pub struct Item__ad_hoc2075<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2075<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 10);
    }
}

///[target_component](GroundControl::SET_VIDEO_STREAM_SETTINGS::target_component).

pub struct Item__ad_hoc2076<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2076<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 11, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 11);
    }
}

///[camera_id](GroundControl::SET_VIDEO_STREAM_SETTINGS::camera_id).

pub struct Item__ad_hoc2077<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2077<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 12, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 12);
    }
}

///[framerate](GroundControl::SET_VIDEO_STREAM_SETTINGS::framerate).

pub struct Item__ad_hoc2078<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2078<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 13, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 13);
    }
}

///[uri](GroundControl::SET_VIDEO_STREAM_SETTINGS::uri).

pub struct Item__ad_hoc2083<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2083<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 138 && !src.set_field(138, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(138, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metaplay_tune(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                         u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaplay_tune {}

pub static mut PLAY_TUNE: Metaplay_tune = Metaplay_tune(203, 0, 0, 0, 3, None, 2, 18, 1, [(&fld__ad_hoc2004 as *const _ad_hoc2004) as *const sys::Field]);

///[target_system](GroundControl::PLAY_TUNE::target_system).

pub struct Item__ad_hoc2002<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2002<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[target_component](GroundControl::PLAY_TUNE::target_component).

pub struct Item__ad_hoc2003<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2003<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[tune](GroundControl::PLAY_TUNE::tune).

pub struct Item__ad_hoc2004<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2004<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 18 && !src.set_field(18, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(18, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metadigicam_configure(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadigicam_configure {}

pub static mut DIGICAM_CONFIGURE: Metadigicam_configure = Metadigicam_configure(114, 1, 0, 0, 15, None, 0, 0, 0, []);

///[shutter_speed](GroundControl::DIGICAM_CONFIGURE::shutter_speed).

pub struct Item__ad_hoc2170(pub *mut u8);

impl Item__ad_hoc2170
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[target_system](GroundControl::DIGICAM_CONFIGURE::target_system).

pub struct Item__ad_hoc2167(pub *mut u8);

impl Item__ad_hoc2167
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[target_component](GroundControl::DIGICAM_CONFIGURE::target_component).

pub struct Item__ad_hoc2168(pub *mut u8);

impl Item__ad_hoc2168
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

///[mode](GroundControl::DIGICAM_CONFIGURE::mode).

pub struct Item__ad_hoc2169(pub *mut u8);

impl Item__ad_hoc2169
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[aperture](GroundControl::DIGICAM_CONFIGURE::aperture).

pub struct Item__ad_hoc2171(pub *mut u8);

impl Item__ad_hoc2171
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

///[iso](GroundControl::DIGICAM_CONFIGURE::iso).

pub struct Item__ad_hoc2172(pub *mut u8);

impl Item__ad_hoc2172
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 6);
    }
}

///[exposure_type](GroundControl::DIGICAM_CONFIGURE::exposure_type).

pub struct Item__ad_hoc2173(pub *mut u8);

impl Item__ad_hoc2173
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 7, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 7);
    }
}

///[command_id](GroundControl::DIGICAM_CONFIGURE::command_id).

pub struct Item__ad_hoc2174(pub *mut u8);

impl Item__ad_hoc2174
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

///[engine_cut_off](GroundControl::DIGICAM_CONFIGURE::engine_cut_off).

pub struct Item__ad_hoc2175(pub *mut u8);

impl Item__ad_hoc2175
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 9, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 9);
    }
}

///[extra_param](GroundControl::DIGICAM_CONFIGURE::extra_param).

pub struct Item__ad_hoc2176(pub *mut u8);

impl Item__ad_hoc2176
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 10);
    }
}

///[extra_value](GroundControl::DIGICAM_CONFIGURE::extra_value).

pub struct Item__ad_hoc2177(pub *mut u8);

impl Item__ad_hoc2177
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 11, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 11); }
}

#[repr(C)]
pub struct Metascaled_pressure3(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metascaled_pressure3 {}

pub static mut SCALED_PRESSURE3: Metascaled_pressure3 = Metascaled_pressure3(130, 0, 1, 0, 14, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::SCALED_PRESSURE3::time_boot_ms).

pub struct Item__ad_hoc1804(pub *mut u8);

impl Item__ad_hoc1804
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[press_abs](GroundControl::SCALED_PRESSURE3::press_abs).

pub struct Item__ad_hoc1805(pub *mut u8);

impl Item__ad_hoc1805
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[press_diff](GroundControl::SCALED_PRESSURE3::press_diff).

pub struct Item__ad_hoc1806(pub *mut u8);

impl Item__ad_hoc1806
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[temperature](GroundControl::SCALED_PRESSURE3::temperature).

pub struct Item__ad_hoc1807(pub *mut u8);

impl Item__ad_hoc1807
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 12, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 12);
    }
}

#[repr(C)]
pub struct Metamission_request_partial_list(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                            u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamission_request_partial_list {}

pub static mut MISSION_REQUEST_PARTIAL_LIST: Metamission_request_partial_list = Metamission_request_partial_list(206, 0, 0, 0, 7, None, 0, 48, 1, [(&fld__ad_hoc354 as *const _ad_hoc354) as *const sys::Field]);

#[repr(C)]
pub struct Metaparam_ext_ack(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 4]);

unsafe impl std::marker::Sync for Metaparam_ext_ack {}

pub static mut PARAM_EXT_ACK: Metaparam_ext_ack = Metaparam_ext_ack(51, 0, 0, 0, 1, None, 3, 3, 4, [(&fld__ad_hoc2118 as *const _ad_hoc2118) as *const sys::Field,
    (&fld__ad_hoc2119 as *const _ad_hoc2119) as *const sys::Field,
    (&fld__ad_hoc233 as *const _ad_hoc233) as *const sys::Field,
    (&fld__ad_hoc234 as *const _ad_hoc234) as *const sys::Field]);

///[param_id](GroundControl::PARAM_EXT_ACK::param_id).

pub struct Item__ad_hoc2118<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2118<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 3 && !src.set_field(3, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(3, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[param_value](GroundControl::PARAM_EXT_ACK::param_value).

pub struct Item__ad_hoc2119<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2119<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 4 && !src.set_field(4, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(4, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

///[param_type](GroundControl::PARAM_EXT_ACK::param_type).

pub struct Item__ad_hoc233<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc233<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_PARAM_EXT_TYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 5 && !src.set_field(5, -1) { return None; }

        Some({ packs::MAV_PARAM_EXT_TYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 4)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_PARAM_EXT_TYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 5
        {
            dst.set_field(5, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 4, dst.base.bytes, dst.BIT);
    }
}

///[param_result](GroundControl::PARAM_EXT_ACK::param_result).

pub struct Item__ad_hoc234<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc234<'a> {
    pub fn get(&mut self) -> Option<packs::PARAM_ACK> {
        let src = &mut self.0;
        if src.base.field_bit != 6 && !src.set_field(6, -1) { return None; }

        Some({ packs::PARAM_ACK::from_bits((sys::get_bits(src.base.bytes, src.BIT, 2)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::PARAM_ACK) {
        let dst = &mut self.0;
        if dst.base.field_bit != 6
        {
            dst.set_field(6, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 2, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metauavcan_node_info(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metauavcan_node_info {}

pub static mut UAVCAN_NODE_INFO: Metauavcan_node_info = Metauavcan_node_info(127, 0, 2, 1, 37, None, 2, 290, 1, [(&fld__ad_hoc2097 as *const _ad_hoc2097) as *const sys::Field]);

///[uptime_sec](GroundControl::UAVCAN_NODE_INFO::uptime_sec).

pub struct Item__ad_hoc2096<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2096<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[sw_vcs_commit](GroundControl::UAVCAN_NODE_INFO::sw_vcs_commit).

pub struct Item__ad_hoc2103<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2103<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 4);
    }
}

///[time_usec](GroundControl::UAVCAN_NODE_INFO::time_usec).

pub struct Item__ad_hoc2095<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2095<'a>
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 8, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 8 as usize, dst.base.bytes, 8);
    }
}

///[hw_version_major](GroundControl::UAVCAN_NODE_INFO::hw_version_major).

pub struct Item__ad_hoc2098<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2098<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 16, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 16);
    }
}

///[hw_version_minor](GroundControl::UAVCAN_NODE_INFO::hw_version_minor).

pub struct Item__ad_hoc2099<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2099<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 17, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 17);
    }
}

///[hw_unique_id](GroundControl::UAVCAN_NODE_INFO::hw_unique_id).

pub struct ItemArray__ad_hoc2100 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2100 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2100 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[sw_version_major](GroundControl::UAVCAN_NODE_INFO::sw_version_major).

pub struct Item__ad_hoc2101<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2101<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 34, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 34);
    }
}

///[sw_version_minor](GroundControl::UAVCAN_NODE_INFO::sw_version_minor).

pub struct Item__ad_hoc2102<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2102<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 35, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 35);
    }
}

///[name](GroundControl::UAVCAN_NODE_INFO::name).

pub struct Item__ad_hoc2097<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2097<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 290 && !src.set_field(290, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(290, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metadata16(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                      u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadata16 {}

pub static mut DATA16: Metadata16 = Metadata16(36, 0, 0, 0, 18, None, 0, 0, 0, []);

///[typE](GroundControl::DATA16::typE).

pub struct Item__ad_hoc2251(pub *mut u8);

impl Item__ad_hoc2251
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[len](GroundControl::DATA16::len).

pub struct Item__ad_hoc2252(pub *mut u8);

impl Item__ad_hoc2252
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[daTa](GroundControl::DATA16::daTa).

pub struct ItemArray__ad_hoc2253 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2253 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2253 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaset_mag_offsets(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaset_mag_offsets {}

pub static mut SET_MAG_OFFSETS: Metaset_mag_offsets = Metaset_mag_offsets(100, 0, 0, 0, 8, None, 0, 0, 0, []);

///[target_system](GroundControl::SET_MAG_OFFSETS::target_system).

pub struct Item__ad_hoc2153(pub *mut u8);

impl Item__ad_hoc2153
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::SET_MAG_OFFSETS::target_component).

pub struct Item__ad_hoc2154(pub *mut u8);

impl Item__ad_hoc2154
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[mag_ofs_x](GroundControl::SET_MAG_OFFSETS::mag_ofs_x).

pub struct Item__ad_hoc2155(pub *mut u8);

impl Item__ad_hoc2155
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[mag_ofs_y](GroundControl::SET_MAG_OFFSETS::mag_ofs_y).

pub struct Item__ad_hoc2156(pub *mut u8);

impl Item__ad_hoc2156
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[mag_ofs_z](GroundControl::SET_MAG_OFFSETS::mag_ofs_z).

pub struct Item__ad_hoc2157(pub *mut u8);

impl Item__ad_hoc2157
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 6);
    }
}

#[repr(C)]
pub struct Metaap_adc(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                      u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaap_adc {}

pub static mut AP_ADC: Metaap_adc = Metaap_adc(198, 6, 0, 0, 12, None, 0, 0, 0, []);

///[adc1](GroundControl::AP_ADC::adc1).

pub struct Item__ad_hoc2161(pub *mut u8);

impl Item__ad_hoc2161
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[adc2](GroundControl::AP_ADC::adc2).

pub struct Item__ad_hoc2162(pub *mut u8);

impl Item__ad_hoc2162
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[adc3](GroundControl::AP_ADC::adc3).

pub struct Item__ad_hoc2163(pub *mut u8);

impl Item__ad_hoc2163
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[adc4](GroundControl::AP_ADC::adc4).

pub struct Item__ad_hoc2164(pub *mut u8);

impl Item__ad_hoc2164
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 6);
    }
}

///[adc5](GroundControl::AP_ADC::adc5).

pub struct Item__ad_hoc2165(pub *mut u8);

impl Item__ad_hoc2165
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 8);
    }
}

///[adc6](GroundControl::AP_ADC::adc6).

pub struct Item__ad_hoc2166(pub *mut u8);

impl Item__ad_hoc2166
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 10);
    }
}

#[repr(C)]
pub struct Metawind(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                    u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metawind {}

pub static mut WIND: Metawind = Metawind(97, 0, 0, 0, 12, None, 0, 0, 0, []);

///[direction](GroundControl::WIND::direction).

pub struct Item__ad_hoc2248(pub *mut u8);

impl Item__ad_hoc2248
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[speed](GroundControl::WIND::speed).

pub struct Item__ad_hoc2249(pub *mut u8);

impl Item__ad_hoc2249
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[speed_z](GroundControl::WIND::speed_z).

pub struct Item__ad_hoc2250(pub *mut u8);

impl Item__ad_hoc2250
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

#[repr(C)]
pub struct Metaautopilot_version_request(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                         u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaautopilot_version_request {}

pub static mut AUTOPILOT_VERSION_REQUEST: Metaautopilot_version_request = Metaautopilot_version_request(142, 0, 0, 0, 2, None, 0, 0, 0, []);

///[target_system](GroundControl::AUTOPILOT_VERSION_REQUEST::target_system).

pub struct Item__ad_hoc2333(pub *mut u8);

impl Item__ad_hoc2333
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::AUTOPILOT_VERSION_REQUEST::target_component).

pub struct Item__ad_hoc2334(pub *mut u8);

impl Item__ad_hoc2334
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

#[repr(C)]
pub struct Metalocal_position_ned(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalocal_position_ned {}

pub static mut LOCAL_POSITION_NED: Metalocal_position_ned = Metalocal_position_ned(131, 0, 1, 0, 28, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metadata_transmission_handshake(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadata_transmission_handshake {}

pub static mut DATA_TRANSMISSION_HANDSHAKE: Metadata_transmission_handshake = Metadata_transmission_handshake(149, 3, 1, 0, 13, None, 0, 0, 0, []);

///[width](GroundControl::DATA_TRANSMISSION_HANDSHAKE::width).

pub struct Item__ad_hoc1744(pub *mut u8);

impl Item__ad_hoc1744
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[height](GroundControl::DATA_TRANSMISSION_HANDSHAKE::height).

pub struct Item__ad_hoc1745(pub *mut u8);

impl Item__ad_hoc1745
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[packets](GroundControl::DATA_TRANSMISSION_HANDSHAKE::packets).

pub struct Item__ad_hoc1746(pub *mut u8);

impl Item__ad_hoc1746
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[size](GroundControl::DATA_TRANSMISSION_HANDSHAKE::size).

pub struct Item__ad_hoc1743(pub *mut u8);

impl Item__ad_hoc1743
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 6);
    }
}

///[typE](GroundControl::DATA_TRANSMISSION_HANDSHAKE::typE).

pub struct Item__ad_hoc1742(pub *mut u8);

impl Item__ad_hoc1742
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 10);
    }
}

///[payload](GroundControl::DATA_TRANSMISSION_HANDSHAKE::payload).

pub struct Item__ad_hoc1747(pub *mut u8);

impl Item__ad_hoc1747
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 11, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 11);
    }
}

///[jpg_quality](GroundControl::DATA_TRANSMISSION_HANDSHAKE::jpg_quality).

pub struct Item__ad_hoc1748(pub *mut u8);

impl Item__ad_hoc1748
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 12, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 12);
    }
}

#[repr(C)]
pub struct Metagps_global_origin(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metagps_global_origin {}

pub static mut GPS_GLOBAL_ORIGIN: Metagps_global_origin = Metagps_global_origin(185, 0, 0, 0, 13, None, 0, 96, 1, [(&fld__ad_hoc1185 as *const _ad_hoc1185) as *const sys::Field]);

#[repr(C)]
pub struct Metascaled_imu2(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metascaled_imu2 {}

pub static mut SCALED_IMU2: Metascaled_imu2 = Metascaled_imu2(178, 0, 1, 0, 22, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::SCALED_IMU2::time_boot_ms).

pub struct Item__ad_hoc1653(pub *mut u8);

impl Item__ad_hoc1653
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[xacc](GroundControl::SCALED_IMU2::xacc).

pub struct Item__ad_hoc1654(pub *mut u8);

impl Item__ad_hoc1654
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 4);
    }
}

///[yacc](GroundControl::SCALED_IMU2::yacc).

pub struct Item__ad_hoc1655(pub *mut u8);

impl Item__ad_hoc1655
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 6);
    }
}

///[zacc](GroundControl::SCALED_IMU2::zacc).

pub struct Item__ad_hoc1656(pub *mut u8);

impl Item__ad_hoc1656
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 8);
    }
}

///[xgyro](GroundControl::SCALED_IMU2::xgyro).

pub struct Item__ad_hoc1657(pub *mut u8);

impl Item__ad_hoc1657
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 10, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 10);
    }
}

///[ygyro](GroundControl::SCALED_IMU2::ygyro).

pub struct Item__ad_hoc1658(pub *mut u8);

impl Item__ad_hoc1658
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 12, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 12);
    }
}

///[zgyro](GroundControl::SCALED_IMU2::zgyro).

pub struct Item__ad_hoc1659(pub *mut u8);

impl Item__ad_hoc1659
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 14, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 14);
    }
}

///[xmag](GroundControl::SCALED_IMU2::xmag).

pub struct Item__ad_hoc1660(pub *mut u8);

impl Item__ad_hoc1660
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 16);
    }
}

///[ymag](GroundControl::SCALED_IMU2::ymag).

pub struct Item__ad_hoc1661(pub *mut u8);

impl Item__ad_hoc1661
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 18, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 18);
    }
}

///[zmag](GroundControl::SCALED_IMU2::zmag).

pub struct Item__ad_hoc1662(pub *mut u8);

impl Item__ad_hoc1662
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 20, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 20);
    }
}

#[repr(C)]
pub struct Metaattitude_quaternion(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                   u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaattitude_quaternion {}

pub static mut ATTITUDE_QUATERNION: Metaattitude_quaternion = Metaattitude_quaternion(71, 0, 1, 0, 32, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metadata64(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                      u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metadata64 {}

pub static mut DATA64: Metadata64 = Metadata64(141, 0, 0, 0, 66, None, 0, 0, 0, []);

///[typE](GroundControl::DATA64::typE).

pub struct Item__ad_hoc2257(pub *mut u8);

impl Item__ad_hoc2257
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[len](GroundControl::DATA64::len).

pub struct Item__ad_hoc2258(pub *mut u8);

impl Item__ad_hoc2258
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[daTa](GroundControl::DATA64::daTa).

pub struct ItemArray__ad_hoc2259 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2259 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2259 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metahil_actuator_controls(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metahil_actuator_controls {}

pub static mut HIL_ACTUATOR_CONTROLS: Metahil_actuator_controls = Metahil_actuator_controls(199, 0, 0, 2, 81, None, 0, 640, 1, [(&fld__ad_hoc326 as *const _ad_hoc326) as *const sys::Field]);

///[controls](GroundControl::HIL_ACTUATOR_CONTROLS::controls).

pub struct ItemArray__ad_hoc1461 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1461 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
}

impl Iterator for ItemArray__ad_hoc1461 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaposition_target_local_ned(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                         u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metaposition_target_local_ned {}

pub static mut POSITION_TARGET_LOCAL_NED: Metaposition_target_local_ned = Metaposition_target_local_ned(129, 1, 1, 0, 51, None, 0, 400, 1, [(&fld__ad_hoc86 as *const _ad_hoc86) as *const sys::Field]);

#[repr(C)]
pub struct Metagimbal_report(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metagimbal_report {}

pub static mut GIMBAL_REPORT: Metagimbal_report = Metagimbal_report(11, 0, 0, 0, 42, None, 0, 0, 0, []);

///[target_system](GroundControl::GIMBAL_REPORT::target_system).

pub struct Item__ad_hoc2379(pub *mut u8);

impl Item__ad_hoc2379
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::GIMBAL_REPORT::target_component).

pub struct Item__ad_hoc2380(pub *mut u8);

impl Item__ad_hoc2380
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[delta_time](GroundControl::GIMBAL_REPORT::delta_time).

pub struct Item__ad_hoc2381(pub *mut u8);

impl Item__ad_hoc2381
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 2, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 2); }
}

///[delta_angle_x](GroundControl::GIMBAL_REPORT::delta_angle_x).

pub struct Item__ad_hoc2382(pub *mut u8);

impl Item__ad_hoc2382
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 6, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 6); }
}

///[delta_angle_y](GroundControl::GIMBAL_REPORT::delta_angle_y).

pub struct Item__ad_hoc2383(pub *mut u8);

impl Item__ad_hoc2383
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 10, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 10); }
}

///[delta_angle_z](GroundControl::GIMBAL_REPORT::delta_angle_z).

pub struct Item__ad_hoc2384(pub *mut u8);

impl Item__ad_hoc2384
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 14, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 14); }
}

///[delta_velocity_x](GroundControl::GIMBAL_REPORT::delta_velocity_x).

pub struct Item__ad_hoc2385(pub *mut u8);

impl Item__ad_hoc2385
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 18, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 18); }
}

///[delta_velocity_y](GroundControl::GIMBAL_REPORT::delta_velocity_y).

pub struct Item__ad_hoc2386(pub *mut u8);

impl Item__ad_hoc2386
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 22, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 22); }
}

///[delta_velocity_z](GroundControl::GIMBAL_REPORT::delta_velocity_z).

pub struct Item__ad_hoc2387(pub *mut u8);

impl Item__ad_hoc2387
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 26, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 26); }
}

///[joint_roll](GroundControl::GIMBAL_REPORT::joint_roll).

pub struct Item__ad_hoc2388(pub *mut u8);

impl Item__ad_hoc2388
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 30, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 30); }
}

///[joint_el](GroundControl::GIMBAL_REPORT::joint_el).

pub struct Item__ad_hoc2389(pub *mut u8);

impl Item__ad_hoc2389
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 34, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 34); }
}

///[joint_az](GroundControl::GIMBAL_REPORT::joint_az).

pub struct Item__ad_hoc2390(pub *mut u8);

impl Item__ad_hoc2390
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 38, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 38); }
}

#[repr(C)]
pub struct Metadevice_op_write(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metadevice_op_write {}

pub static mut DEVICE_OP_WRITE: Metadevice_op_write = Metadevice_op_write(105, 0, 1, 0, 139, None, 2, 1106, 2, [(&fld__ad_hoc285 as *const _ad_hoc285) as *const sys::Field,
    (&fld__ad_hoc2427 as *const _ad_hoc2427) as *const sys::Field]);

///[request_id](GroundControl::DEVICE_OP_WRITE::request_id).

pub struct Item__ad_hoc2424<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2424<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[target_system](GroundControl::DEVICE_OP_WRITE::target_system).

pub struct Item__ad_hoc2422<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2422<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 4);
    }
}

///[target_component](GroundControl::DEVICE_OP_WRITE::target_component).

pub struct Item__ad_hoc2423<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2423<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 5);
    }
}

///[bus](GroundControl::DEVICE_OP_WRITE::bus).

pub struct Item__ad_hoc2425<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2425<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 6);
    }
}

///[address](GroundControl::DEVICE_OP_WRITE::address).

pub struct Item__ad_hoc2426<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2426<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 7, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 7);
    }
}

///[regstart](GroundControl::DEVICE_OP_WRITE::regstart).

pub struct Item__ad_hoc2428<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2428<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 8);
    }
}

///[count](GroundControl::DEVICE_OP_WRITE::count).

pub struct Item__ad_hoc2429<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2429<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 9, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 9);
    }
}

///[daTa](GroundControl::DEVICE_OP_WRITE::daTa).

pub struct ItemArray__ad_hoc2430 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2430 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2430 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[bustype](GroundControl::DEVICE_OP_WRITE::bustype).

pub struct Item__ad_hoc285<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc285<'a> {
    pub fn get(&mut self) -> Option<packs::DEVICE_OP_BUSTYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 1106 && !src.set_field(1106, -1) { return None; }

        Some({ packs::DEVICE_OP_BUSTYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::DEVICE_OP_BUSTYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1106
        {
            dst.set_field(1106, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

///[busname](GroundControl::DEVICE_OP_WRITE::busname).

pub struct Item__ad_hoc2427<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2427<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 1107 && !src.set_field(1107, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(1107, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metadistance_sensor(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metadistance_sensor {}

pub static mut DISTANCE_SENSOR: Metadistance_sensor = Metadistance_sensor(61, 3, 1, 0, 13, None, 2, 98, 2, [(&fld__ad_hoc244 as *const _ad_hoc244) as *const sys::Field,
    (&fld__ad_hoc243 as *const _ad_hoc243) as *const sys::Field]);

///[min_distance](GroundControl::DISTANCE_SENSOR::min_distance).

pub struct Item__ad_hoc1752<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1752<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[max_distance](GroundControl::DISTANCE_SENSOR::max_distance).

pub struct Item__ad_hoc1753<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1753<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[current_distance](GroundControl::DISTANCE_SENSOR::current_distance).

pub struct Item__ad_hoc1754<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1754<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 4);
    }
}

///[time_boot_ms](GroundControl::DISTANCE_SENSOR::time_boot_ms).

pub struct Item__ad_hoc1751<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1751<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 6);
    }
}

///[id](GroundControl::DISTANCE_SENSOR::id).

pub struct Item__ad_hoc1755<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1755<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 10);
    }
}

///[covariance](GroundControl::DISTANCE_SENSOR::covariance).

pub struct Item__ad_hoc1756<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1756<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 11, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 11);
    }
}

///[typE](GroundControl::DISTANCE_SENSOR::typE).

pub struct Item__ad_hoc244<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc244<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_DISTANCE_SENSOR> {
        let src = &mut self.0;
        if src.base.field_bit != 98 && !src.set_field(98, -1) { return None; }

        Some({ packs::MAV_DISTANCE_SENSOR::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_DISTANCE_SENSOR) {
        let dst = &mut self.0;
        if dst.base.field_bit != 98
        {
            dst.set_field(98, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[orientation](GroundControl::DISTANCE_SENSOR::orientation).

pub struct Item__ad_hoc243<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc243<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_SENSOR_ORIENTATION> {
        let src = &mut self.0;
        if src.base.field_bit != 99 && !src.set_field(99, -1) { return None; }

        Some({ packs::MAV_SENSOR_ORIENTATION::from_bits((sys::get_bits(src.base.bytes, src.BIT, 6)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_SENSOR_ORIENTATION) {
        let dst = &mut self.0;
        if dst.base.field_bit != 99
        {
            dst.set_field(99, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 6, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metahil_optical_flow(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metahil_optical_flow {}

pub static mut HIL_OPTICAL_FLOW: Metahil_optical_flow = Metahil_optical_flow(116, 0, 2, 1, 44, None, 0, 0, 0, []);

///[integration_time_us](GroundControl::HIL_OPTICAL_FLOW::integration_time_us).

pub struct Item__ad_hoc1627(pub *mut u8);

impl Item__ad_hoc1627
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[time_delta_distance_us](GroundControl::HIL_OPTICAL_FLOW::time_delta_distance_us).

pub struct Item__ad_hoc1635(pub *mut u8);

impl Item__ad_hoc1635
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 4);
    }
}

///[time_usec](GroundControl::HIL_OPTICAL_FLOW::time_usec).

pub struct Item__ad_hoc1625(pub *mut u8);

impl Item__ad_hoc1625
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 8);
    }
}

///[sensor_id](GroundControl::HIL_OPTICAL_FLOW::sensor_id).

pub struct Item__ad_hoc1626(pub *mut u8);

impl Item__ad_hoc1626
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 16, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 16);
    }
}

///[integrated_x](GroundControl::HIL_OPTICAL_FLOW::integrated_x).

pub struct Item__ad_hoc1628(pub *mut u8);

impl Item__ad_hoc1628
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 17, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 17); }
}

///[integrated_y](GroundControl::HIL_OPTICAL_FLOW::integrated_y).

pub struct Item__ad_hoc1629(pub *mut u8);

impl Item__ad_hoc1629
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 21, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 21); }
}

///[integrated_xgyro](GroundControl::HIL_OPTICAL_FLOW::integrated_xgyro).

pub struct Item__ad_hoc1630(pub *mut u8);

impl Item__ad_hoc1630
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 25, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 25); }
}

///[integrated_ygyro](GroundControl::HIL_OPTICAL_FLOW::integrated_ygyro).

pub struct Item__ad_hoc1631(pub *mut u8);

impl Item__ad_hoc1631
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 29, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 29); }
}

///[integrated_zgyro](GroundControl::HIL_OPTICAL_FLOW::integrated_zgyro).

pub struct Item__ad_hoc1632(pub *mut u8);

impl Item__ad_hoc1632
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 33, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 33); }
}

///[temperature](GroundControl::HIL_OPTICAL_FLOW::temperature).

pub struct Item__ad_hoc1633(pub *mut u8);

impl Item__ad_hoc1633
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 37, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 37);
    }
}

///[quality](GroundControl::HIL_OPTICAL_FLOW::quality).

pub struct Item__ad_hoc1634(pub *mut u8);

impl Item__ad_hoc1634
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 39, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 39);
    }
}

///[distance](GroundControl::HIL_OPTICAL_FLOW::distance).

pub struct Item__ad_hoc1636(pub *mut u8);

impl Item__ad_hoc1636
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 40); }
}

#[repr(C)]
pub struct Metascaled_pressure2(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metascaled_pressure2 {}

pub static mut SCALED_PRESSURE2: Metascaled_pressure2 = Metascaled_pressure2(151, 0, 1, 0, 14, None, 0, 0, 0, []);

///[time_boot_ms](GroundControl::SCALED_PRESSURE2::time_boot_ms).

pub struct Item__ad_hoc1775(pub *mut u8);

impl Item__ad_hoc1775
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        sys::set_bytes((src) as u64, 4 as usize, self.0, 0);
    }
}

///[press_abs](GroundControl::SCALED_PRESSURE2::press_abs).

pub struct Item__ad_hoc1776(pub *mut u8);

impl Item__ad_hoc1776
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[press_diff](GroundControl::SCALED_PRESSURE2::press_diff).

pub struct Item__ad_hoc1777(pub *mut u8);

impl Item__ad_hoc1777
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[temperature](GroundControl::SCALED_PRESSURE2::temperature).

pub struct Item__ad_hoc1778(pub *mut u8);

impl Item__ad_hoc1778
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 12, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 12);
    }
}

#[repr(C)]
pub struct Metawind_cov(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                        u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metawind_cov {}

pub static mut WIND_COV: Metawind_cov = Metawind_cov(160, 0, 0, 1, 40, None, 0, 0, 0, []);

///[time_usec](GroundControl::WIND_COV::time_usec).

pub struct Item__ad_hoc1875(pub *mut u8);

impl Item__ad_hoc1875
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[wind_x](GroundControl::WIND_COV::wind_x).

pub struct Item__ad_hoc1876(pub *mut u8);

impl Item__ad_hoc1876
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

///[wind_y](GroundControl::WIND_COV::wind_y).

pub struct Item__ad_hoc1877(pub *mut u8);

impl Item__ad_hoc1877
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 12, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 12); }
}

///[wind_z](GroundControl::WIND_COV::wind_z).

pub struct Item__ad_hoc1878(pub *mut u8);

impl Item__ad_hoc1878
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 16, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 16); }
}

///[var_horiz](GroundControl::WIND_COV::var_horiz).

pub struct Item__ad_hoc1879(pub *mut u8);

impl Item__ad_hoc1879
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 20, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 20); }
}

///[var_vert](GroundControl::WIND_COV::var_vert).

pub struct Item__ad_hoc1880(pub *mut u8);

impl Item__ad_hoc1880
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 24, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 24); }
}

///[wind_alt](GroundControl::WIND_COV::wind_alt).

pub struct Item__ad_hoc1881(pub *mut u8);

impl Item__ad_hoc1881
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 28, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 28); }
}

///[horiz_accuracy](GroundControl::WIND_COV::horiz_accuracy).

pub struct Item__ad_hoc1882(pub *mut u8);

impl Item__ad_hoc1882
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 32, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 32); }
}

///[vert_accuracy](GroundControl::WIND_COV::vert_accuracy).

pub struct Item__ad_hoc1883(pub *mut u8);

impl Item__ad_hoc1883
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 36, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 36); }
}

#[repr(C)]
pub struct Metachange_operator_control(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                       u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metachange_operator_control {}

pub static mut CHANGE_OPERATOR_CONTROL: Metachange_operator_control = Metachange_operator_control(154, 0, 0, 0, 4, None, 2, 26, 1, [(&fld__ad_hoc999 as *const _ad_hoc999) as *const sys::Field]);

#[repr(C)]
pub struct Metagopro_set_request(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                 u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metagopro_set_request {}

pub static mut GOPRO_SET_REQUEST: Metagopro_set_request = Metagopro_set_request(118, 0, 0, 0, 7, None, 0, 48, 1, [(&fld__ad_hoc17 as *const _ad_hoc17) as *const sys::Field]);

///[target_system](GroundControl::GOPRO_SET_REQUEST::target_system).

pub struct Item__ad_hoc2404<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2404<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[target_component](GroundControl::GOPRO_SET_REQUEST::target_component).

pub struct Item__ad_hoc2405<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2405<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[value](GroundControl::GOPRO_SET_REQUEST::value).

pub struct ItemArray__ad_hoc2406 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2406 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2406 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[cmd_id](GroundControl::GOPRO_SET_REQUEST::cmd_id).

pub struct Item__ad_hoc17<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc17<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_COMMAND> {
        let src = &mut self.0;
        if src.base.field_bit != 48 && !src.set_field(48, -1) { return None; }

        Some({ packs::GOPRO_COMMAND::from_bits((sys::get_bits(src.base.bytes, src.BIT, 5)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_COMMAND) {
        let dst = &mut self.0;
        if dst.base.field_bit != 48
        {
            dst.set_field(48, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 5, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metasys_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                          u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metasys_status {}

pub static mut SYS_STATUS: Metasys_status = Metasys_status(132, 8, 0, 0, 20, None, 2, 154, 3, [(&fld__ad_hoc828 as *const _ad_hoc828) as *const sys::Field,
    (&fld__ad_hoc826 as *const _ad_hoc826) as *const sys::Field,
    (&fld__ad_hoc827 as *const _ad_hoc827) as *const sys::Field]);

#[repr(C)]
pub struct Metamission_item(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metamission_item {}

pub static mut MISSION_ITEM: Metamission_item = Metamission_item(35, 1, 0, 0, 35, None, 2, 274, 3, [(&fld__ad_hoc149 as *const _ad_hoc149) as *const sys::Field,
    (&fld__ad_hoc150 as *const _ad_hoc150) as *const sys::Field,
    (&fld__ad_hoc151 as *const _ad_hoc151) as *const sys::Field]);

#[repr(C)]
pub struct Metaraw_imu(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                       u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaraw_imu {}

pub static mut RAW_IMU: Metaraw_imu = Metaraw_imu(163, 0, 0, 1, 26, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metacommand_int(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metacommand_int {}

pub static mut COMMAND_INT: Metacommand_int = Metacommand_int(167, 0, 0, 0, 33, None, 2, 258, 2, [(&fld__ad_hoc420 as *const _ad_hoc420) as *const sys::Field,
    (&fld__ad_hoc421 as *const _ad_hoc421) as *const sys::Field]);

#[repr(C)]
pub struct Metaoptical_flow(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metaoptical_flow {}

pub static mut OPTICAL_FLOW: Metaoptical_flow = Metaoptical_flow(13, 0, 0, 1, 27, None, 0, 208, 2, [(&fld__ad_hoc1471 as *const _ad_hoc1471) as *const sys::Field,
    (&fld__ad_hoc1472 as *const _ad_hoc1472) as *const sys::Field]);

#[repr(C)]
pub struct Metamission_item_int(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metamission_item_int {}

pub static mut MISSION_ITEM_INT: Metamission_item_int = Metamission_item_int(59, 1, 0, 0, 35, None, 2, 274, 3, [(&fld__ad_hoc195 as *const _ad_hoc195) as *const sys::Field,
    (&fld__ad_hoc196 as *const _ad_hoc196) as *const sys::Field,
    (&fld__ad_hoc194 as *const _ad_hoc194) as *const sys::Field]);

#[repr(C)]
pub struct Metavision_position_delta(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                     u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metavision_position_delta {}

pub static mut VISION_POSITION_DELTA: Metavision_position_delta = Metavision_position_delta(12, 0, 0, 2, 44, None, 0, 0, 0, []);

///[time_usec](GroundControl::VISION_POSITION_DELTA::time_usec).

pub struct Item__ad_hoc2445(pub *mut u8);

impl Item__ad_hoc2445
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 0);
    }
}

///[time_delta_usec](GroundControl::VISION_POSITION_DELTA::time_delta_usec).

pub struct Item__ad_hoc2446(pub *mut u8);

impl Item__ad_hoc2446
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 8);
    }
}

///[angle_delta](GroundControl::VISION_POSITION_DELTA::angle_delta).

pub struct ItemArray__ad_hoc2447 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2447 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc2447 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[position_delta](GroundControl::VISION_POSITION_DELTA::position_delta).

pub struct ItemArray__ad_hoc2448 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2448 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
    pub fn set(&mut self, index: usize, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.bytes, self.offset + index * 4); }
}

impl Iterator for ItemArray__ad_hoc2448 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[confidence](GroundControl::VISION_POSITION_DELTA::confidence).

pub struct Item__ad_hoc2449(pub *mut u8);

impl Item__ad_hoc2449
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 40, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 40); }
}

#[repr(C)]
pub struct Metalogging_data(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metalogging_data {}

pub static mut LOGGING_DATA: Metalogging_data = Metalogging_data(75, 1, 0, 0, 255, None, 0, 0, 0, []);

///[sequence](GroundControl::LOGGING_DATA::sequence).

pub struct Item__ad_hoc2054(pub *mut u8);

impl Item__ad_hoc2054
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[target_system](GroundControl::LOGGING_DATA::target_system).

pub struct Item__ad_hoc2052(pub *mut u8);

impl Item__ad_hoc2052
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[target_component](GroundControl::LOGGING_DATA::target_component).

pub struct Item__ad_hoc2053(pub *mut u8);

impl Item__ad_hoc2053
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

///[length](GroundControl::LOGGING_DATA::length).

pub struct Item__ad_hoc2055(pub *mut u8);

impl Item__ad_hoc2055
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[first_message_offset](GroundControl::LOGGING_DATA::first_message_offset).

pub struct Item__ad_hoc2056(pub *mut u8);

impl Item__ad_hoc2056
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

///[daTa](GroundControl::LOGGING_DATA::daTa).

pub struct ItemArray__ad_hoc2057 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2057 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2057 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metadevice_op_read(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                              u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metadevice_op_read {}

pub static mut DEVICE_OP_READ: Metadevice_op_read = Metadevice_op_read(70, 0, 1, 0, 11, None, 2, 82, 2, [(&fld__ad_hoc248 as *const _ad_hoc248) as *const sys::Field,
    (&fld__ad_hoc2414 as *const _ad_hoc2414) as *const sys::Field]);

///[request_id](GroundControl::DEVICE_OP_READ::request_id).

pub struct Item__ad_hoc2411<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2411<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[target_system](GroundControl::DEVICE_OP_READ::target_system).

pub struct Item__ad_hoc2409<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2409<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 4);
    }
}

///[target_component](GroundControl::DEVICE_OP_READ::target_component).

pub struct Item__ad_hoc2410<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2410<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 5);
    }
}

///[bus](GroundControl::DEVICE_OP_READ::bus).

pub struct Item__ad_hoc2412<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2412<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 6);
    }
}

///[address](GroundControl::DEVICE_OP_READ::address).

pub struct Item__ad_hoc2413<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2413<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 7, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 7);
    }
}

///[regstart](GroundControl::DEVICE_OP_READ::regstart).

pub struct Item__ad_hoc2415<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2415<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 8);
    }
}

///[count](GroundControl::DEVICE_OP_READ::count).

pub struct Item__ad_hoc2416<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2416<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 9, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 9);
    }
}

///[bustype](GroundControl::DEVICE_OP_READ::bustype).

pub struct Item__ad_hoc248<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc248<'a> {
    pub fn get(&mut self) -> Option<packs::DEVICE_OP_BUSTYPE> {
        let src = &mut self.0;
        if src.base.field_bit != 82 && !src.set_field(82, -1) { return None; }

        Some({ packs::DEVICE_OP_BUSTYPE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::DEVICE_OP_BUSTYPE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 82
        {
            dst.set_field(82, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

///[busname](GroundControl::DEVICE_OP_READ::busname).

pub struct Item__ad_hoc2414<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2414<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 83 && !src.set_field(83, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(83, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metamag_cal_progress(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metamag_cal_progress {}

pub static mut MAG_CAL_PROGRESS: Metamag_cal_progress = Metamag_cal_progress(57, 0, 0, 0, 27, None, 0, 208, 1, [(&fld__ad_hoc141 as *const _ad_hoc141) as *const sys::Field]);

///[compass_id](GroundControl::MAG_CAL_PROGRESS::compass_id).

pub struct Item__ad_hoc2347<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2347<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 0);
    }
}

///[cal_mask](GroundControl::MAG_CAL_PROGRESS::cal_mask).

pub struct Item__ad_hoc2348<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2348<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 1);
    }
}

///[attempt](GroundControl::MAG_CAL_PROGRESS::attempt).

pub struct Item__ad_hoc2349<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2349<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 2);
    }
}

///[completion_pct](GroundControl::MAG_CAL_PROGRESS::completion_pct).

pub struct Item__ad_hoc2350<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2350<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 3);
    }
}

///[completion_mask](GroundControl::MAG_CAL_PROGRESS::completion_mask).

pub struct ItemArray__ad_hoc2351 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2351 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2351 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[direction_x](GroundControl::MAG_CAL_PROGRESS::direction_x).

pub struct Item__ad_hoc2352<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2352<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 14, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 14);
    }
}

///[direction_y](GroundControl::MAG_CAL_PROGRESS::direction_y).

pub struct Item__ad_hoc2353<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2353<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 18, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 18);
    }
}

///[direction_z](GroundControl::MAG_CAL_PROGRESS::direction_z).

pub struct Item__ad_hoc2354<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2354<'a>
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(src.base.bytes, 22, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) {
        let dst = &mut self.0;
        sys::set_bytes(src.to_bits() as u64, 4usize, dst.base.bytes, 22);
    }
}

///[cal_status](GroundControl::MAG_CAL_PROGRESS::cal_status).

pub struct Item__ad_hoc141<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc141<'a> {
    pub fn get(&mut self) -> Option<packs::MAG_CAL_STATUS> {
        let src = &mut self.0;
        if src.base.field_bit != 208 && !src.set_field(208, -1) { return None; }

        Some({ packs::MAG_CAL_STATUS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAG_CAL_STATUS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 208
        {
            dst.set_field(208, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metahighres_imu(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metahighres_imu {}

pub static mut HIGHRES_IMU: Metahighres_imu = Metahighres_imu(137, 1, 0, 1, 62, None, 0, 0, 0, []);

///[fields_updated](GroundControl::HIGHRES_IMU::fields_updated).

pub struct Item__ad_hoc1548(pub *mut u8);

impl Item__ad_hoc1548
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[time_usec](GroundControl::HIGHRES_IMU::time_usec).

pub struct Item__ad_hoc1534(pub *mut u8);

impl Item__ad_hoc1534
{
    pub fn get(&mut self) -> i64 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 8 as usize) as i64;
        (dst) as i64
    }
    pub fn set(&mut self, src: i64) {
        sys::set_bytes((src) as u64, 8 as usize, self.0, 2);
    }
}

///[xacc](GroundControl::HIGHRES_IMU::xacc).

pub struct Item__ad_hoc1535(pub *mut u8);

impl Item__ad_hoc1535
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 10, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 10); }
}

///[yacc](GroundControl::HIGHRES_IMU::yacc).

pub struct Item__ad_hoc1536(pub *mut u8);

impl Item__ad_hoc1536
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 14, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 14); }
}

///[zacc](GroundControl::HIGHRES_IMU::zacc).

pub struct Item__ad_hoc1537(pub *mut u8);

impl Item__ad_hoc1537
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 18, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 18); }
}

///[xgyro](GroundControl::HIGHRES_IMU::xgyro).

pub struct Item__ad_hoc1538(pub *mut u8);

impl Item__ad_hoc1538
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 22, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 22); }
}

///[ygyro](GroundControl::HIGHRES_IMU::ygyro).

pub struct Item__ad_hoc1539(pub *mut u8);

impl Item__ad_hoc1539
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 26, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 26); }
}

///[zgyro](GroundControl::HIGHRES_IMU::zgyro).

pub struct Item__ad_hoc1540(pub *mut u8);

impl Item__ad_hoc1540
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 30, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 30); }
}

///[xmag](GroundControl::HIGHRES_IMU::xmag).

pub struct Item__ad_hoc1541(pub *mut u8);

impl Item__ad_hoc1541
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 34, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 34); }
}

///[ymag](GroundControl::HIGHRES_IMU::ymag).

pub struct Item__ad_hoc1542(pub *mut u8);

impl Item__ad_hoc1542
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 38, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 38); }
}

///[zmag](GroundControl::HIGHRES_IMU::zmag).

pub struct Item__ad_hoc1543(pub *mut u8);

impl Item__ad_hoc1543
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 42, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 42); }
}

///[abs_pressure](GroundControl::HIGHRES_IMU::abs_pressure).

pub struct Item__ad_hoc1544(pub *mut u8);

impl Item__ad_hoc1544
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 46, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 46); }
}

///[diff_pressure](GroundControl::HIGHRES_IMU::diff_pressure).

pub struct Item__ad_hoc1545(pub *mut u8);

impl Item__ad_hoc1545
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 50, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 50); }
}

///[pressure_alt](GroundControl::HIGHRES_IMU::pressure_alt).

pub struct Item__ad_hoc1546(pub *mut u8);

impl Item__ad_hoc1546
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 54, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 54); }
}

///[temperature](GroundControl::HIGHRES_IMU::temperature).

pub struct Item__ad_hoc1547(pub *mut u8);

impl Item__ad_hoc1547
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 58, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 58); }
}

#[repr(C)]
pub struct Metaextended_sys_state(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metaextended_sys_state {}

pub static mut EXTENDED_SYS_STATE: Metaextended_sys_state = Metaextended_sys_state(176, 0, 0, 0, 1, None, 0, 0, 2, [(&fld__ad_hoc19 as *const _ad_hoc19) as *const sys::Field,
    (&fld__ad_hoc20 as *const _ad_hoc20) as *const sys::Field]);

///[vtol_state](GroundControl::EXTENDED_SYS_STATE::vtol_state).

pub struct Item__ad_hoc19<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc19<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_VTOL_STATE> {
        let src = &mut self.0;
        if src.base.field_bit != 0 && !src.set_field(0, -1) { return None; }

        Some({ packs::MAV_VTOL_STATE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_VTOL_STATE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 0
        {
            dst.set_field(0, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[landed_state](GroundControl::EXTENDED_SYS_STATE::landed_state).

pub struct Item__ad_hoc20<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc20<'a> {
    pub fn get(&mut self) -> Option<packs::MAV_LANDED_STATE> {
        let src = &mut self.0;
        if src.base.field_bit != 1 && !src.set_field(1, -1) { return None; }

        Some({ packs::MAV_LANDED_STATE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::MAV_LANDED_STATE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 1
        {
            dst.set_field(1, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metauavionix_adsb_out_dynamic(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                         u16, u32, u16, [*const sys::Field; 3]);

unsafe impl std::marker::Sync for Metauavionix_adsb_out_dynamic {}

pub static mut UAVIONIX_ADSB_OUT_DYNAMIC: Metauavionix_adsb_out_dynamic = Metauavionix_adsb_out_dynamic(143, 3, 2, 0, 38, None, 2, 298, 3, [(&fld__ad_hoc587 as *const _ad_hoc587) as *const sys::Field,
    (&fld__ad_hoc585 as *const _ad_hoc585) as *const sys::Field,
    (&fld__ad_hoc586 as *const _ad_hoc586) as *const sys::Field]);

///[accuracyVert](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::accuracyVert).

pub struct Item__ad_hoc2135<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2135<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 0);
    }
}

///[accuracyVel](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::accuracyVel).

pub struct Item__ad_hoc2136<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2136<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 2);
    }
}

///[squawk](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::squawk).

pub struct Item__ad_hoc2140<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2140<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 4);
    }
}

///[utcTime](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::utcTime).

pub struct Item__ad_hoc2128<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2128<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 6, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 6);
    }
}

///[accuracyHor](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::accuracyHor).

pub struct Item__ad_hoc2134<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2134<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 10, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 10);
    }
}

///[gpsLat](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::gpsLat).

pub struct Item__ad_hoc2129<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2129<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 14, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 14);
    }
}

///[gpsLon](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::gpsLon).

pub struct Item__ad_hoc2130<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2130<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 18, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 18);
    }
}

///[gpsAlt](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::gpsAlt).

pub struct Item__ad_hoc2131<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2131<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 22, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 22);
    }
}

///[numSats](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::numSats).

pub struct Item__ad_hoc2132<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2132<'a>
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 26, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 1 as usize, dst.base.bytes, 26);
    }
}

///[baroAltMSL](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::baroAltMSL).

pub struct Item__ad_hoc2133<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2133<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 27, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 27);
    }
}

///[velVert](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::velVert).

pub struct Item__ad_hoc2137<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2137<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 31, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 31);
    }
}

///[velNS](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::velNS).

pub struct Item__ad_hoc2138<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2138<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 33, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 33);
    }
}

///[VelEW](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::VelEW).

pub struct Item__ad_hoc2139<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc2139<'a>
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 35, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 2 as usize, dst.base.bytes, 35);
    }
}

///[gpsFix](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::gpsFix).

pub struct Item__ad_hoc587<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc587<'a> {
    pub fn get(&mut self) -> Option<packs::UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX> {
        let src = &mut self.0;
        if src.base.field_bit != 298 && !src.set_field(298, -1) { return None; }

        Some({ packs::UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX) {
        let dst = &mut self.0;
        if dst.base.field_bit != 298
        {
            dst.set_field(298, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[emergencyStatus](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::emergencyStatus).

pub struct Item__ad_hoc585<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc585<'a> {
    pub fn get(&mut self) -> Option<packs::UAVIONIX_ADSB_EMERGENCY_STATUS> {
        let src = &mut self.0;
        if src.base.field_bit != 299 && !src.set_field(299, -1) { return None; }

        Some({ packs::UAVIONIX_ADSB_EMERGENCY_STATUS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVIONIX_ADSB_EMERGENCY_STATUS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 299
        {
            dst.set_field(299, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

///[state](GroundControl::UAVIONIX_ADSB_OUT_DYNAMIC::state).

pub struct Item__ad_hoc586<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc586<'a> {
    pub fn get(&mut self) -> Option<packs::UAVIONIX_ADSB_OUT_DYNAMIC_STATE> {
        let src = &mut self.0;
        if src.base.field_bit != 300 && !src.set_field(300, -1) { return None; }

        Some({ packs::UAVIONIX_ADSB_OUT_DYNAMIC_STATE::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVIONIX_ADSB_OUT_DYNAMIC_STATE) {
        let dst = &mut self.0;
        if dst.base.field_bit != 300
        {
            dst.set_field(300, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metagopro_get_response(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                  u16, u32, u16, [*const sys::Field; 2]);

unsafe impl std::marker::Sync for Metagopro_get_response {}

pub static mut GOPRO_GET_RESPONSE: Metagopro_get_response = Metagopro_get_response(208, 0, 0, 0, 5, None, 0, 32, 2, [(&fld__ad_hoc426 as *const _ad_hoc426) as *const sys::Field,
    (&fld__ad_hoc425 as *const _ad_hoc425) as *const sys::Field]);

///[value](GroundControl::GOPRO_GET_RESPONSE::value).

pub struct ItemArray__ad_hoc2403 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc2403 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc2403 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[cmd_id](GroundControl::GOPRO_GET_RESPONSE::cmd_id).

pub struct Item__ad_hoc426<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc426<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_COMMAND> {
        let src = &mut self.0;
        if src.base.field_bit != 32 && !src.set_field(32, -1) { return None; }

        Some({ packs::GOPRO_COMMAND::from_bits((sys::get_bits(src.base.bytes, src.BIT, 5)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_COMMAND) {
        let dst = &mut self.0;
        if dst.base.field_bit != 32
        {
            dst.set_field(32, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 5, dst.base.bytes, dst.BIT);
    }
}

///[status](GroundControl::GOPRO_GET_RESPONSE::status).

pub struct Item__ad_hoc425<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc425<'a> {
    pub fn get(&mut self) -> Option<packs::GOPRO_REQUEST_STATUS> {
        let src = &mut self.0;
        if src.base.field_bit != 33 && !src.set_field(33, -1) { return None; }

        Some({ packs::GOPRO_REQUEST_STATUS::from_bits((sys::get_bits(src.base.bytes, src.BIT, 1)) as i8).unwrap() })
    }

    pub fn set(&mut self, src: packs::GOPRO_REQUEST_STATUS) {
        let dst = &mut self.0;
        if dst.base.field_bit != 33
        {
            dst.set_field(33, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 1, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metagps_inject_data(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metagps_inject_data {}

pub static mut GPS_INJECT_DATA: Metagps_inject_data = Metagps_inject_data(52, 0, 0, 0, 113, None, 0, 0, 0, []);

///[target_system](GroundControl::GPS_INJECT_DATA::target_system).

pub struct Item__ad_hoc1685(pub *mut u8);

impl Item__ad_hoc1685
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::GPS_INJECT_DATA::target_component).

pub struct Item__ad_hoc1686(pub *mut u8);

impl Item__ad_hoc1686
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[len](GroundControl::GPS_INJECT_DATA::len).

pub struct Item__ad_hoc1687(pub *mut u8);

impl Item__ad_hoc1687
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[daTa](GroundControl::GPS_INJECT_DATA::daTa).

pub struct ItemArray__ad_hoc1688 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1688 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1688 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metauavionix_adsb_transceiver_health_report(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                                       u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metauavionix_adsb_transceiver_health_report {}

pub static mut UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT: Metauavionix_adsb_transceiver_health_report = Metauavionix_adsb_transceiver_health_report(26, 0, 0, 0, 1, None, 0, 0, 1, [(&fld__ad_hoc377 as *const _ad_hoc377) as *const sys::Field]);

///[rfHealth](GroundControl::UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT::rfHealth).

pub struct Item__ad_hoc377<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc377<'a> {
    pub fn get(&mut self) -> Option<packs::UAVIONIX_ADSB_RF_HEALTH> {
        let src = &mut self.0;
        if src.base.field_bit != 0 && !src.set_field(0, -1) { return None; }

        Some({ packs::UAVIONIX_ADSB_RF_HEALTH::from_bits((sys::get_bits(src.base.bytes, src.BIT, 3)) as i32).unwrap() })
    }

    pub fn set(&mut self, src: packs::UAVIONIX_ADSB_RF_HEALTH) {
        let dst = &mut self.0;
        if dst.base.field_bit != 0
        {
            dst.set_field(0, 0i32);
        }


        sys::set_bits((src.bits()) as u64, 3, dst.base.bytes, dst.BIT);
    }
}

#[repr(C)]
pub struct Metaattitude_quaternion_cov(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                       u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaattitude_quaternion_cov {}

pub static mut ATTITUDE_QUATERNION_COV: Metaattitude_quaternion_cov = Metaattitude_quaternion_cov(182, 0, 0, 1, 72, None, 0, 0, 0, []);

///[q](GroundControl::ATTITUDE_QUATERNION_COV::q).

pub struct ItemArray__ad_hoc1213 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1213 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
}

impl Iterator for ItemArray__ad_hoc1213 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


///[covariance](GroundControl::ATTITUDE_QUATERNION_COV::covariance).

pub struct ItemArray__ad_hoc1217 { pub bytes: *const u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1217 {
    pub fn get(&mut self, index: usize) -> f32 { f32::from_bits(sys::get_bytes(self.bytes, self.offset + index * 4, 4usize) as u32) }
}

impl Iterator for ItemArray__ad_hoc1217 {
    type Item = f32;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metanamed_value_int(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                               u16, u32, u16, [*const sys::Field; 1]);

unsafe impl std::marker::Sync for Metanamed_value_int {}

pub static mut NAMED_VALUE_INT: Metanamed_value_int = Metanamed_value_int(21, 0, 1, 0, 9, None, 2, 66, 1, [(&fld__ad_hoc1989 as *const _ad_hoc1989) as *const sys::Field]);

///[time_boot_ms](GroundControl::NAMED_VALUE_INT::time_boot_ms).

pub struct Item__ad_hoc1988<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1988<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 0, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 0);
    }
}

///[value](GroundControl::NAMED_VALUE_INT::value).

pub struct Item__ad_hoc1990<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1990<'a>
{
    pub fn get(&mut self) -> i32 {
        let src = &mut self.0;
        let dst = sys::get_bytes(src.base.bytes, 4, 4 as usize) as i32;
        (dst) as i32
    }
    pub fn set(&mut self, src: i32) {
        let dst = &mut self.0;
        sys::set_bytes((src) as u64, 4 as usize, dst.base.bytes, 4);
    }
}

///[name](GroundControl::NAMED_VALUE_INT::name).

pub struct Item__ad_hoc1989<'a> (pub &'a mut sys::Cursor);

impl<'a> Item__ad_hoc1989<'a> {
    pub fn get(&mut self) -> Option<&str> {
        let src = &mut self.0;
        if src.base.field_bit != 66 && !src.set_field(66, -1) { return None; }
        Some(src.get_str())
    }


    pub fn set(&mut self, src: &str) {
        let len = src.len();
        let dst = &mut self.0;
        dst.set_field(66, len as i32);
        dst.set_str(src);
    }

    pub fn item_len(&mut self) -> usize { self.0.item_len as usize }
}

#[repr(C)]
pub struct Metarpm(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                   u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metarpm {}

pub static mut RPM: Metarpm = Metarpm(183, 0, 0, 0, 8, None, 0, 0, 0, []);

///[rpm1](GroundControl::RPM::rpm1).

pub struct Item__ad_hoc2407(pub *mut u8);

impl Item__ad_hoc2407
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[rpm2](GroundControl::RPM::rpm2).

pub struct Item__ad_hoc2408(pub *mut u8);

impl Item__ad_hoc2408
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

#[repr(C)]
pub struct Metagps_rtcm_data(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                             u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metagps_rtcm_data {}

pub static mut GPS_RTCM_DATA: Metagps_rtcm_data = Metagps_rtcm_data(88, 0, 0, 0, 182, None, 0, 0, 0, []);

///[flags](GroundControl::GPS_RTCM_DATA::flags).

pub struct Item__ad_hoc1901(pub *mut u8);

impl Item__ad_hoc1901
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[len](GroundControl::GPS_RTCM_DATA::len).

pub struct Item__ad_hoc1902(pub *mut u8);

impl Item__ad_hoc1902
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[daTa](GroundControl::GPS_RTCM_DATA::daTa).

pub struct ItemArray__ad_hoc1903 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1903 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1903 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metaglobal_vision_position_estimate(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                               u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaglobal_vision_position_estimate {}

pub static mut GLOBAL_VISION_POSITION_ESTIMATE: Metaglobal_vision_position_estimate = Metaglobal_vision_position_estimate(110, 0, 0, 1, 32, None, 0, 0, 0, []);

#[repr(C)]
pub struct Metafile_transfer_protocol(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                      u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metafile_transfer_protocol {}

pub static mut FILE_TRANSFER_PROTOCOL: Metafile_transfer_protocol = Metafile_transfer_protocol(27, 0, 0, 0, 254, None, 0, 0, 0, []);

///[target_network](GroundControl::FILE_TRANSFER_PROTOCOL::target_network).

pub struct Item__ad_hoc1604(pub *mut u8);

impl Item__ad_hoc1604
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_system](GroundControl::FILE_TRANSFER_PROTOCOL::target_system).

pub struct Item__ad_hoc1605(pub *mut u8);

impl Item__ad_hoc1605
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[target_component](GroundControl::FILE_TRANSFER_PROTOCOL::target_component).

pub struct Item__ad_hoc1606(pub *mut u8);

impl Item__ad_hoc1606
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[payload](GroundControl::FILE_TRANSFER_PROTOCOL::payload).

pub struct ItemArray__ad_hoc1607 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1607 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1607 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
pub struct Metarangefinder(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metarangefinder {}

pub static mut RANGEFINDER: Metarangefinder = Metarangefinder(153, 0, 0, 0, 8, None, 0, 0, 0, []);

///[distance](GroundControl::RANGEFINDER::distance).

pub struct Item__ad_hoc2263(pub *mut u8);

impl Item__ad_hoc2263
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 0, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 0); }
}

///[voltage](GroundControl::RANGEFINDER::voltage).

pub struct Item__ad_hoc2264(pub *mut u8);

impl Item__ad_hoc2264
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

#[repr(C)]
pub struct Metaradio_status(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                            u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaradio_status {}

pub static mut RADIO_STATUS: Metaradio_status = Metaradio_status(107, 2, 0, 0, 9, None, 0, 0, 0, []);

///[rxerrors](GroundControl::RADIO_STATUS::rxerrors).

pub struct Item__ad_hoc1602(pub *mut u8);

impl Item__ad_hoc1602
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 0);
    }
}

///[fixeD](GroundControl::RADIO_STATUS::fixeD).

pub struct Item__ad_hoc1603(pub *mut u8);

impl Item__ad_hoc1603
{
    pub fn get(&mut self) -> i16 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 2 as usize) as i16;
        (dst) as i16
    }
    pub fn set(&mut self, src: i16) {
        sys::set_bytes((src) as u64, 2 as usize, self.0, 2);
    }
}

///[rssi](GroundControl::RADIO_STATUS::rssi).

pub struct Item__ad_hoc1597(pub *mut u8);

impl Item__ad_hoc1597
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 4, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 4);
    }
}

///[remrssi](GroundControl::RADIO_STATUS::remrssi).

pub struct Item__ad_hoc1598(pub *mut u8);

impl Item__ad_hoc1598
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 5, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 5);
    }
}

///[txbuf](GroundControl::RADIO_STATUS::txbuf).

pub struct Item__ad_hoc1599(pub *mut u8);

impl Item__ad_hoc1599
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 6, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 6);
    }
}

///[noise](GroundControl::RADIO_STATUS::noise).

pub struct Item__ad_hoc1600(pub *mut u8);

impl Item__ad_hoc1600
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 7, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 7);
    }
}

///[remnoise](GroundControl::RADIO_STATUS::remnoise).

pub struct Item__ad_hoc1601(pub *mut u8);

impl Item__ad_hoc1601
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 8, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 8);
    }
}

#[repr(C)]
pub struct Metafence_point(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                           u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metafence_point {}

pub static mut FENCE_POINT: Metafence_point = Metafence_point(20, 0, 0, 0, 12, None, 0, 0, 0, []);

///[target_system](GroundControl::FENCE_POINT::target_system).

pub struct Item__ad_hoc2204(pub *mut u8);

impl Item__ad_hoc2204
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[target_component](GroundControl::FENCE_POINT::target_component).

pub struct Item__ad_hoc2205(pub *mut u8);

impl Item__ad_hoc2205
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[idx](GroundControl::FENCE_POINT::idx).

pub struct Item__ad_hoc2206(pub *mut u8);

impl Item__ad_hoc2206
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 2, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 2);
    }
}

///[count](GroundControl::FENCE_POINT::count).

pub struct Item__ad_hoc2207(pub *mut u8);

impl Item__ad_hoc2207
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 3, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 3);
    }
}

///[lat](GroundControl::FENCE_POINT::lat).

pub struct Item__ad_hoc2208(pub *mut u8);

impl Item__ad_hoc2208
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 4, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 4); }
}

///[lng](GroundControl::FENCE_POINT::lng).

pub struct Item__ad_hoc2209(pub *mut u8);

impl Item__ad_hoc2209
{
    pub fn get(&mut self) -> f32 {
        let src = &mut self.0;
        f32::from_bits(sys::get_bytes(self.0, 8, 4usize) as u32)
    }
    pub fn set(&mut self, src: f32) { sys::set_bytes(src.to_bits() as u64, 4usize, self.0, 8); }
}

#[repr(C)]
pub struct Metaresource_request(pub u16, u32, u32, u32, pub u32, pub Option<unsafe extern "C" fn(pack: *mut sys::Pack, len: usize) -> *mut sys::Pack>,
                                u16, u32, u16, [*const sys::Field; 0]);

unsafe impl std::marker::Sync for Metaresource_request {}

pub static mut RESOURCE_REQUEST: Metaresource_request = Metaresource_request(67, 0, 0, 0, 243, None, 0, 0, 0, []);

///[request_id](GroundControl::RESOURCE_REQUEST::request_id).

pub struct Item__ad_hoc1799(pub *mut u8);

impl Item__ad_hoc1799
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 0, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 0);
    }
}

///[uri_type](GroundControl::RESOURCE_REQUEST::uri_type).

pub struct Item__ad_hoc1800(pub *mut u8);

impl Item__ad_hoc1800
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 1);
    }
}

///[uri](GroundControl::RESOURCE_REQUEST::uri).

pub struct ItemArray__ad_hoc1801 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1801 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1801 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}

///[transfer_type](GroundControl::RESOURCE_REQUEST::transfer_type).

pub struct Item__ad_hoc1802(pub *mut u8);

impl Item__ad_hoc1802
{
    pub fn get(&mut self) -> i8 {
        let src = &mut self.0;
        let dst = sys::get_bytes(self.0, 122, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.0, 122);
    }
}

///[storage](GroundControl::RESOURCE_REQUEST::storage).

pub struct ItemArray__ad_hoc1803 { pub bytes: *mut u8, pub len: usize, pub offset: usize, pub index: usize }

impl ItemArray__ad_hoc1803 {
    pub fn get(&mut self, index: usize) -> i8 {
        let dst = sys::get_bytes(self.bytes, self.offset + index * 1, 1 as usize) as i8;
        (dst) as i8
    }
    pub fn set(&mut self, index: usize, src: i8) {
        sys::set_bytes((src) as u64, 1 as usize, self.bytes, self.offset + index * 1);
    }
}

impl Iterator for ItemArray__ad_hoc1803 {
    type Item = i8;

    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        self.index = self.index.wrapping_add(1);
        if self.index < self.len { return Some(self.get(self.index)); }
        self.index = !0;
        None
    }
}


#[repr(C)]
struct _ad_hoc233(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc233 {}

static fld__ad_hoc233: _ad_hoc233 = _ad_hoc233(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc80(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc80 {}

static fld__ad_hoc80: _ad_hoc80 = _ad_hoc80(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1863(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1863 {}

static fld__ad_hoc1863: _ad_hoc1863 = _ad_hoc1863(1, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc683(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc683 {}

static fld__ad_hoc683: _ad_hoc683 = _ad_hoc683(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1490(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1490 {}

static fld__ad_hoc1490: _ad_hoc1490 = _ad_hoc1490(1, true, 1, 4, 6, 3, 0, 0 as *const sys::Meta, 3, [3]);

#[repr(C)]
struct _ad_hoc1986(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1986 {}

static fld__ad_hoc1986: _ad_hoc1986 = _ad_hoc1986(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc84(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc84 {}

static fld__ad_hoc84: _ad_hoc84 = _ad_hoc84(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc208(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc208 {}

static fld__ad_hoc208: _ad_hoc208 = _ad_hoc208(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2126(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2126 {}

static fld__ad_hoc2126: _ad_hoc2126 = _ad_hoc2126(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc354(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc354 {}

static fld__ad_hoc354: _ad_hoc354 = _ad_hoc354(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1861(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1861 {}

static fld__ad_hoc1861: _ad_hoc1861 = _ad_hoc1861(1, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1511(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1511 {}

static fld__ad_hoc1511: _ad_hoc1511 = _ad_hoc1511(2, false, 1, 4, 6, 12, 5, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc1492(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1492 {}

static fld__ad_hoc1492: _ad_hoc1492 = _ad_hoc1492(2, true, 1, 4, 18, 10, 5 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc804(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc804 {}

static fld__ad_hoc804: _ad_hoc804 = _ad_hoc804(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc214(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc214 {}

static fld__ad_hoc214: _ad_hoc214 = _ad_hoc214(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc424(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc424 {}

static fld__ad_hoc424: _ad_hoc424 = _ad_hoc424(7, false, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc71(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc71 {}

static fld__ad_hoc71: _ad_hoc71 = _ad_hoc71(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc86(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc86 {}

static fld__ad_hoc86: _ad_hoc86 = _ad_hoc86(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);
static _ad_hoc1493_D: [usize; 3] = [3, 2, 3];

#[repr(C)]
struct _ad_hoc1964(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1964 {}

static fld__ad_hoc1964: _ad_hoc1964 = _ad_hoc1964(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc963(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc963 {}

static fld__ad_hoc963: _ad_hoc963 = _ad_hoc963(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc59(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc59 {}

static fld__ad_hoc59: _ad_hoc59 = _ad_hoc59(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1510(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1510 {}

static fld__ad_hoc1510: _ad_hoc1510 = _ad_hoc1510(1, false, 1, 4, 6, 2, 0, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc1506(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1506 {}

static fld__ad_hoc1506: _ad_hoc1506 = _ad_hoc1506(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1515(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1515 {}

static fld__ad_hoc1515: _ad_hoc1515 = _ad_hoc1515(1, false, 1, 4, 6, 2, 0, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc9(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc9 {}

static fld__ad_hoc9: _ad_hoc9 = _ad_hoc9(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1989(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1989 {}

static fld__ad_hoc1989: _ad_hoc1989 = _ad_hoc1989(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1333(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1333 {}

static fld__ad_hoc1333: _ad_hoc1333 = _ad_hoc1333(1, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1471(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1471 {}

static fld__ad_hoc1471: _ad_hoc1471 = _ad_hoc1471(1, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2414(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2414 {}

static fld__ad_hoc2414: _ad_hoc2414 = _ad_hoc2414(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2106(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2106 {}

static fld__ad_hoc2106: _ad_hoc2106 = _ad_hoc2106(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1472(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1472 {}

static fld__ad_hoc1472: _ad_hoc1472 = _ad_hoc1472(1, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1980(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1980 {}

static fld__ad_hoc1980: _ad_hoc1980 = _ad_hoc1980(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc189(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc189 {}

static fld__ad_hoc189: _ad_hoc189 = _ad_hoc189(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1499(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1499 {}

static fld__ad_hoc1499: _ad_hoc1499 = _ad_hoc1499(7, false, 1, 6, 27, 0, 0 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc83(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc83 {}

static fld__ad_hoc83: _ad_hoc83 = _ad_hoc83(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2074(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2074 {}

static fld__ad_hoc2074: _ad_hoc2074 = _ad_hoc2074(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1029(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1029 {}

static fld__ad_hoc1029: _ad_hoc1029 = _ad_hoc1029(1, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc802(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc802 {}

static fld__ad_hoc802: _ad_hoc802 = _ad_hoc802(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc58(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc58 {}

static fld__ad_hoc58: _ad_hoc58 = _ad_hoc58(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1522(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1522 {}

static fld__ad_hoc1522: _ad_hoc1522 = _ad_hoc1522(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1331(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1331 {}

static fld__ad_hoc1331: _ad_hoc1331 = _ad_hoc1331(1, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc209(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc209 {}

static fld__ad_hoc209: _ad_hoc209 = _ad_hoc209(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1991(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1991 {}

static fld__ad_hoc1991: _ad_hoc1991 = _ad_hoc1991(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1030(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1030 {}

static fld__ad_hoc1030: _ad_hoc1030 = _ad_hoc1030(1, true, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc105(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc105 {}

static fld__ad_hoc105: _ad_hoc105 = _ad_hoc105(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1517(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1517 {}

static fld__ad_hoc1517: _ad_hoc1517 = _ad_hoc1517(2, false, 1, 4, 18, 10, 5 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc586(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc586 {}

static fld__ad_hoc586: _ad_hoc586 = _ad_hoc586(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc210(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc210 {}

static fld__ad_hoc210: _ad_hoc210 = _ad_hoc210(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc972(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc972 {}

static fld__ad_hoc972: _ad_hoc972 = _ad_hoc972(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc326(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc326 {}

static fld__ad_hoc326: _ad_hoc326 = _ad_hoc326(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2097(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2097 {}

static fld__ad_hoc2097: _ad_hoc2097 = _ad_hoc2097(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1508(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1508 {}

static fld__ad_hoc1508: _ad_hoc1508 = _ad_hoc1508(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc106(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc106 {}

static fld__ad_hoc106: _ad_hoc106 = _ad_hoc106(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc218(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc218 {}

static fld__ad_hoc218: _ad_hoc218 = _ad_hoc218(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1018(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1018 {}

static fld__ad_hoc1018: _ad_hoc1018 = _ad_hoc1018(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1513(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1513 {}

static fld__ad_hoc1513: _ad_hoc1513 = _ad_hoc1513(2, false, 1, 4, 18, 10, 5 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc1488(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1488 {}

static fld__ad_hoc1488: _ad_hoc1488 = _ad_hoc1488(1, true, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc17(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc17 {}

static fld__ad_hoc17: _ad_hoc17 = _ad_hoc17(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1862(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1862 {}

static fld__ad_hoc1862: _ad_hoc1862 = _ad_hoc1862(1, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2043(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2043 {}

static fld__ad_hoc2043: _ad_hoc2043 = _ad_hoc2043(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc248(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc248 {}

static fld__ad_hoc248: _ad_hoc248 = _ad_hoc248(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1139(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1139 {}

static fld__ad_hoc1139: _ad_hoc1139 = _ad_hoc1139(1, true, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc212(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc212 {}

static fld__ad_hoc212: _ad_hoc212 = _ad_hoc212(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc686(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc686 {}

static fld__ad_hoc686: _ad_hoc686 = _ad_hoc686(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc149(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc149 {}

static fld__ad_hoc149: _ad_hoc149 = _ad_hoc149(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1502(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1502 {}

static fld__ad_hoc1502: _ad_hoc1502 = _ad_hoc1502(7, false, 1, 5, 3, 4, 0, 0 as *const sys::Meta, 3, [2, 2]);

#[repr(C)]
struct _ad_hoc820(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc820 {}

static fld__ad_hoc820: _ad_hoc820 = _ad_hoc820(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1135(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1135 {}

static fld__ad_hoc1135: _ad_hoc1135 = _ad_hoc1135(1, true, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc352(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc352 {}

static fld__ad_hoc352: _ad_hoc352 = _ad_hoc352(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc630(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc630 {}

static fld__ad_hoc630: _ad_hoc630 = _ad_hoc630(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc372(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc372 {}

static fld__ad_hoc372: _ad_hoc372 = _ad_hoc372(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc10(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc10 {}

static fld__ad_hoc10: _ad_hoc10 = _ad_hoc10(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc957(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc957 {}

static fld__ad_hoc957: _ad_hoc957 = _ad_hoc957(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2016(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2016 {}

static fld__ad_hoc2016: _ad_hoc2016 = _ad_hoc2016(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc828(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc828 {}

static fld__ad_hoc828: _ad_hoc828 = _ad_hoc828(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1491(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1491 {}

static fld__ad_hoc1491: _ad_hoc1491 = _ad_hoc1491(1, true, 1, 4, 18, 0, 0 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc1501(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1501 {}

static fld__ad_hoc1501: _ad_hoc1501 = _ad_hoc1501(7, false, 1, 5, 9, 2, 0, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc190(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc190 {}

static fld__ad_hoc190: _ad_hoc190 = _ad_hoc190(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1509(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1509 {}

static fld__ad_hoc1509: _ad_hoc1509 = _ad_hoc1509(1, false, 1, 4, 6, 2, 0, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc1132(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1132 {}

static fld__ad_hoc1132: _ad_hoc1132 = _ad_hoc1132(1, true, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1003(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1003 {}

static fld__ad_hoc1003: _ad_hoc1003 = _ad_hoc1003(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc11(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc11 {}

static fld__ad_hoc11: _ad_hoc11 = _ad_hoc11(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc636(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc636 {}

static fld__ad_hoc636: _ad_hoc636 = _ad_hoc636(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc51(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc51 {}

static fld__ad_hoc51: _ad_hoc51 = _ad_hoc51(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2117(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2117 {}

static fld__ad_hoc2117: _ad_hoc2117 = _ad_hoc2117(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2110(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2110 {}

static fld__ad_hoc2110: _ad_hoc2110 = _ad_hoc2110(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1853(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 1]);

unsafe impl std::marker::Sync for _ad_hoc1853 {}

static fld__ad_hoc1853: _ad_hoc1853 = _ad_hoc1853(2, false, 1, 1, 18, 10, 5 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc881(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc881 {}

static fld__ad_hoc881: _ad_hoc881 = _ad_hoc881(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc688(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc688 {}

static fld__ad_hoc688: _ad_hoc688 = _ad_hoc688(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc585(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc585 {}

static fld__ad_hoc585: _ad_hoc585 = _ad_hoc585(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1520(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1520 {}

static fld__ad_hoc1520: _ad_hoc1520 = _ad_hoc1520(5, false, 8, 1, 6, 2, 0, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc213(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc213 {}

static fld__ad_hoc213: _ad_hoc213 = _ad_hoc213(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc139(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc139 {}

static fld__ad_hoc139: _ad_hoc139 = _ad_hoc139(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2004(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2004 {}

static fld__ad_hoc2004: _ad_hoc2004 = _ad_hoc2004(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc376(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc376 {}

static fld__ad_hoc376: _ad_hoc376 = _ad_hoc376(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1519(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1519 {}

static fld__ad_hoc1519: _ad_hoc1519 = _ad_hoc1519(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc282(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc282 {}

static fld__ad_hoc282: _ad_hoc282 = _ad_hoc282(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc803(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc803 {}

static fld__ad_hoc803: _ad_hoc803 = _ad_hoc803(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc141(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc141 {}

static fld__ad_hoc141: _ad_hoc141 = _ad_hoc141(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1521(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1521 {}

static fld__ad_hoc1521: _ad_hoc1521 = _ad_hoc1521(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc202(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc202 {}

static fld__ad_hoc202: _ad_hoc202 = _ad_hoc202(7, false, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc219(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc219 {}

static fld__ad_hoc219: _ad_hoc219 = _ad_hoc219(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1865(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1865 {}

static fld__ad_hoc1865: _ad_hoc1865 = _ad_hoc1865(1, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1516(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 1]);

unsafe impl std::marker::Sync for _ad_hoc1516 {}

static fld__ad_hoc1516: _ad_hoc1516 = _ad_hoc1516(2, false, 1, 1, 18, 10, 5 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc421(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc421 {}

static fld__ad_hoc421: _ad_hoc421 = _ad_hoc421(7, false, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2084(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2084 {}

static fld__ad_hoc2084: _ad_hoc2084 = _ad_hoc2084(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1181(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1181 {}

static fld__ad_hoc1181: _ad_hoc1181 = _ad_hoc1181(1, true, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1032(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1032 {}

static fld__ad_hoc1032: _ad_hoc1032 = _ad_hoc1032(1, true, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1012(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1012 {}

static fld__ad_hoc1012: _ad_hoc1012 = _ad_hoc1012(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc377(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc377 {}

static fld__ad_hoc377: _ad_hoc377 = _ad_hoc377(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc353(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc353 {}

static fld__ad_hoc353: _ad_hoc353 = _ad_hoc353(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1487(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1487 {}

static fld__ad_hoc1487: _ad_hoc1487 = _ad_hoc1487(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc882(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc882 {}

static fld__ad_hoc882: _ad_hoc882 = _ad_hoc882(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc193(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc193 {}

static fld__ad_hoc193: _ad_hoc193 = _ad_hoc193(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc645(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc645 {}

static fld__ad_hoc645: _ad_hoc645 = _ad_hoc645(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2111(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2111 {}

static fld__ad_hoc2111: _ad_hoc2111 = _ad_hoc2111(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc706(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc706 {}

static fld__ad_hoc706: _ad_hoc706 = _ad_hoc706(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc710(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc710 {}

static fld__ad_hoc710: _ad_hoc710 = _ad_hoc710(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc85(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc85 {}

static fld__ad_hoc85: _ad_hoc85 = _ad_hoc85(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1137(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1137 {}

static fld__ad_hoc1137: _ad_hoc1137 = _ad_hoc1137(1, true, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc19(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc19 {}

static fld__ad_hoc19: _ad_hoc19 = _ad_hoc19(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1518(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1518 {}

static fld__ad_hoc1518: _ad_hoc1518 = _ad_hoc1518(2, false, 1, 4, 18, 10, 5 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc687(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc687 {}

static fld__ad_hoc687: _ad_hoc687 = _ad_hoc687(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc629(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc629 {}

static fld__ad_hoc629: _ad_hoc629 = _ad_hoc629(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc999(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc999 {}

static fld__ad_hoc999: _ad_hoc999 = _ad_hoc999(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc420(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc420 {}

static fld__ad_hoc420: _ad_hoc420 = _ad_hoc420(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);
static _ad_hoc1514_D: [usize; 3] = [3, 2, 3];

#[repr(C)]
struct _ad_hoc4(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc4 {}

static fld__ad_hoc4: _ad_hoc4 = _ad_hoc4(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc150(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc150 {}

static fld__ad_hoc150: _ad_hoc150 = _ad_hoc150(7, false, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1503(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1503 {}

static fld__ad_hoc1503: _ad_hoc1503 = _ad_hoc1503(8, false, 1, 5, 3, 14, 5, 0 as *const sys::Meta, 3, [2, 2]);

#[repr(C)]
struct _ad_hoc351(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc351 {}

static fld__ad_hoc351: _ad_hoc351 = _ad_hoc351(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc349(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc349 {}

static fld__ad_hoc349: _ad_hoc349 = _ad_hoc349(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc191(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc191 {}

static fld__ad_hoc191: _ad_hoc191 = _ad_hoc191(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc342(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc342 {}

static fld__ad_hoc342: _ad_hoc342 = _ad_hoc342(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc425(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc425 {}

static fld__ad_hoc425: _ad_hoc425 = _ad_hoc425(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc16(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc16 {}

static fld__ad_hoc16: _ad_hoc16 = _ad_hoc16(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1954(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1954 {}

static fld__ad_hoc1954: _ad_hoc1954 = _ad_hoc1954(1, true, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc20(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc20 {}

static fld__ad_hoc20: _ad_hoc20 = _ad_hoc20(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc211(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc211 {}

static fld__ad_hoc211: _ad_hoc211 = _ad_hoc211(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc140(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc140 {}

static fld__ad_hoc140: _ad_hoc140 = _ad_hoc140(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1136(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1136 {}

static fld__ad_hoc1136: _ad_hoc1136 = _ad_hoc1136(1, true, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc192(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc192 {}

static fld__ad_hoc192: _ad_hoc192 = _ad_hoc192(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1188(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1188 {}

static fld__ad_hoc1188: _ad_hoc1188 = _ad_hoc1188(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1332(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1332 {}

static fld__ad_hoc1332: _ad_hoc1332 = _ad_hoc1332(1, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1008(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1008 {}

static fld__ad_hoc1008: _ad_hoc1008 = _ad_hoc1008(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc880(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc880 {}

static fld__ad_hoc880: _ad_hoc880 = _ad_hoc880(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc628(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc628 {}

static fld__ad_hoc628: _ad_hoc628 = _ad_hoc628(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1185(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1185 {}

static fld__ad_hoc1185: _ad_hoc1185 = _ad_hoc1185(1, true, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc285(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc285 {}

static fld__ad_hoc285: _ad_hoc285 = _ad_hoc285(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc281(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc281 {}

static fld__ad_hoc281: _ad_hoc281 = _ad_hoc281(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1864(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 1]);

unsafe impl std::marker::Sync for _ad_hoc1864 {}

static fld__ad_hoc1864: _ad_hoc1864 = _ad_hoc1864(2, false, 1, 4, 4, 6, 3 % dims_count %   ]);

#[repr(C)]
struct _ad_hoc1033(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1033 {}

static fld__ad_hoc1033: _ad_hoc1033 = _ad_hoc1033(1, true, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc112(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc112 {}

static fld__ad_hoc112: _ad_hoc112 = _ad_hoc112(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2118(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2118 {}

static fld__ad_hoc2118: _ad_hoc2118 = _ad_hoc2118(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc152(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc152 {}

static fld__ad_hoc152: _ad_hoc152 = _ad_hoc152(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2119(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2119 {}

static fld__ad_hoc2119: _ad_hoc2119 = _ad_hoc2119(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc827(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc827 {}

static fld__ad_hoc827: _ad_hoc827 = _ad_hoc827(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc883(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc883 {}

static fld__ad_hoc883: _ad_hoc883 = _ad_hoc883(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1133(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1133 {}

static fld__ad_hoc1133: _ad_hoc1133 = _ad_hoc1133(1, true, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc151(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc151 {}

static fld__ad_hoc151: _ad_hoc151 = _ad_hoc151(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc422(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc422 {}

static fld__ad_hoc422: _ad_hoc422 = _ad_hoc422(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1 {}

static fld__ad_hoc1: _ad_hoc1 = _ad_hoc1(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1498(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1498 {}

static fld__ad_hoc1498: _ad_hoc1498 = _ad_hoc1498(1, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1505(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1505 {}

static fld__ad_hoc1505: _ad_hoc1505 = _ad_hoc1505(8, false, 1, 9, 9, 12, 5, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc215(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc215 {}

static fld__ad_hoc215: _ad_hoc215 = _ad_hoc215(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc12(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc12 {}

static fld__ad_hoc12: _ad_hoc12 = _ad_hoc12(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2083(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2083 {}

static fld__ad_hoc2083: _ad_hoc2083 = _ad_hoc2083(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc705(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc705 {}

static fld__ad_hoc705: _ad_hoc705 = _ad_hoc705(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc107(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc107 {}

static fld__ad_hoc107: _ad_hoc107 = _ad_hoc107(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1507(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1507 {}

static fld__ad_hoc1507: _ad_hoc1507 = _ad_hoc1507(7, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc966(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc966 {}

static fld__ad_hoc966: _ad_hoc966 = _ad_hoc966(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc188(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc188 {}

static fld__ad_hoc188: _ad_hoc188 = _ad_hoc188(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc350(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc350 {}

static fld__ad_hoc350: _ad_hoc350 = _ad_hoc350(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc306(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc306 {}

static fld__ad_hoc306: _ad_hoc306 = _ad_hoc306(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1031(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1031 {}

static fld__ad_hoc1031: _ad_hoc1031 = _ad_hoc1031(1, true, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc194(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc194 {}

static fld__ad_hoc194: _ad_hoc194 = _ad_hoc194(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc104(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc104 {}

static fld__ad_hoc104: _ad_hoc104 = _ad_hoc104(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1500(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1500 {}

static fld__ad_hoc1500: _ad_hoc1500 = _ad_hoc1500(8, false, 1, 6, 9, 12, 5, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc1504(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1504 {}

static fld__ad_hoc1504: _ad_hoc1504 = _ad_hoc1504(7, false, 1, 6, 9, 2, 0, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc244(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc244 {}

static fld__ad_hoc244: _ad_hoc244 = _ad_hoc244(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc423(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc423 {}

static fld__ad_hoc423: _ad_hoc423 = _ad_hoc423(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2116(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2116 {}

static fld__ad_hoc2116: _ad_hoc2116 = _ad_hoc2116(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc153(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc153 {}

static fld__ad_hoc153: _ad_hoc153 = _ad_hoc153(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc196(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc196 {}

static fld__ad_hoc196: _ad_hoc196 = _ad_hoc196(7, false, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc195(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc195 {}

static fld__ad_hoc195: _ad_hoc195 = _ad_hoc195(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc685(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc685 {}

static fld__ad_hoc685: _ad_hoc685 = _ad_hoc685(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc411(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc411 {}

static fld__ad_hoc411: _ad_hoc411 = _ad_hoc411(7, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc243(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc243 {}

static fld__ad_hoc243: _ad_hoc243 = _ad_hoc243(7, false, 1, 6, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1512(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 3]);

unsafe impl std::marker::Sync for _ad_hoc1512 {}

static fld__ad_hoc1512: _ad_hoc1512 = _ad_hoc1512(2, false, 1, 4, 6, 12, 5, 0 as *const sys::Meta, 3, [2]);

#[repr(C)]
struct _ad_hoc426(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc426 {}

static fld__ad_hoc426: _ad_hoc426 = _ad_hoc426(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1138(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1138 {}

static fld__ad_hoc1138: _ad_hoc1138 = _ad_hoc1138(1, true, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1330(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1330 {}

static fld__ad_hoc1330: _ad_hoc1330 = _ad_hoc1330(1, false, 1, 1, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2085(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2085 {}

static fld__ad_hoc2085: _ad_hoc2085 = _ad_hoc2085(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc684(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc684 {}

static fld__ad_hoc684: _ad_hoc684 = _ad_hoc684(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1134(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1134 {}

static fld__ad_hoc1134: _ad_hoc1134 = _ad_hoc1134(1, true, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc234(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc234 {}

static fld__ad_hoc234: _ad_hoc234 = _ad_hoc234(7, false, 1, 2, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1942(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1942 {}

static fld__ad_hoc1942: _ad_hoc1942 = _ad_hoc1942(1, true, 1, 8, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2427(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2427 {}

static fld__ad_hoc2427: _ad_hoc2427 = _ad_hoc2427(3, false, 8, 1, 1, 8, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc2160(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc2160 {}

static fld__ad_hoc2160: _ad_hoc2160 = _ad_hoc2160(1, true, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc587(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc587 {}

static fld__ad_hoc587: _ad_hoc587 = _ad_hoc587(7, false, 1, 3, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc826(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc826 {}

static fld__ad_hoc826: _ad_hoc826 = _ad_hoc826(7, false, 1, 5, 1, 0, 0, 0 as *const sys::Meta, 0, []);

#[repr(C)]
struct _ad_hoc1494(u8, bool, i32, i8, u32, u16, u8, u16, *const sys::Meta, u8, [i32; 0]);

unsafe impl std::marker::Sync for _ad_hoc1494 {}

static fld__ad_hoc1494: _ad_hoc1494 = _ad_hoc1494(1, false, 1, 4, 1, 0, 0, 0 as *const sys::Meta, 0, []);


pub const RECEIVE_REQ_MAX_BYTES: u32 = 255u32;
pub const RECEIVE_FULL_MAX_BYTES: u32 = 6850u32;


pub const SEND_REQ_MAX_BYTES: u32 = 255u32;
pub const SEND_FULL_MAX_BYTES: u32 = 6850u32;
							
						
						