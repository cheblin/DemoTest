package com.company.demo.GroundControl

import com.company.demo.GroundControl.util_.*
import org.unirail.AdHoc.*
import org.unirail.AdHoc.Pack.Cursor
import org.unirail.AdHoc.Pack.Meta
import java.util.*
import kotlin.experimental.*
import java.util.concurrent.ArrayBlockingQueue


fun Cursor(): Cursor {

    if (GLOBAL_POSITION_INT_COV.meta.fields[0] == null) {
        val _ad_hoc2106 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc349 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1185 = Meta.Field(1, true, 1, 8, 1, 0, 0, null)
        val _ad_hoc1137 = Meta.Field(1, true, 1, 2, 1, 0, 0, null)
        val _ad_hoc645 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc2119 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1502 = Meta.Field(7, false, 1, 5, 3, 4, 0, null, 2, 2)
        val _ad_hoc1472 = Meta.Field(1, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1139 = Meta.Field(1, true, 1, 2, 1, 0, 0, null)
        val _ad_hoc1133 = Meta.Field(1, true, 1, 2, 1, 0, 0, null)
        val _ad_hoc210 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc234 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc585 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1863 = Meta.Field(1, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc2117 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1491 = Meta.Field(1, true, 1, 4, 18, 0, 0, null)
        val _ad_hoc1031 = Meta.Field(1, true, 1, 4, 1, 0, 0, null)
        val _ad_hoc191 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc1498 = Meta.Field(1, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc1964 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc636 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1991 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc354 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc211 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc1853 = Meta.Field(2, false, 1, 1, 18, 10, 5, null)
        val _ad_hoc194 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1330 = Meta.Field(1, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc189 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc2160 = Meta.Field(1, true, 1, 4, 1, 0, 0, null)
        val _ad_hoc190 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc85 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc1499 = Meta.Field(7, false, 1, 6, 27, 0, 0, null)
        val _ad_hoc9 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc139 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc306 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc2427 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1511 = Meta.Field(2, false, 1, 4, 6, 12, 5, null, 2)
        val _ad_hoc12 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc2043 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1503 = Meta.Field(8, false, 1, 5, 3, 14, 5, null, 2, 2)
        val _ad_hoc2110 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc2126 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc963 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1136 = Meta.Field(1, true, 1, 2, 1, 0, 0, null)
        val _ad_hoc140 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc688 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc1510 = Meta.Field(1, false, 1, 4, 6, 2, 0, null, 2)
        val _ad_hoc820 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc202 = Meta.Field(7, false, 1, 8, 1, 0, 0, null)
        val _ad_hoc827 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc1506 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc350 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc1490 = Meta.Field(1, true, 1, 4, 6, 3, 0, null, 3)
        val _ad_hoc1488 = Meta.Field(1, true, 1, 4, 1, 0, 0, null)
        val _ad_hoc2097 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc2116 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc196 = Meta.Field(7, false, 1, 8, 1, 0, 0, null)
        val _ad_hoc966 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc4 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc215 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1333 = Meta.Field(1, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc2111 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc2016 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1862 = Meta.Field(1, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc353 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc586 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc71 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc424 = Meta.Field(7, false, 1, 8, 1, 0, 0, null)
        val _ad_hoc214 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1500 = Meta.Field(8, false, 1, 6, 9, 12, 5, null, 2)
        val _ad_hoc193 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc687 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1003 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc686 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc1030 = Meta.Field(1, true, 1, 4, 1, 0, 0, null)
        val _ad_hoc629 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc107 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc104 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1471 = Meta.Field(1, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1029 = Meta.Field(1, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc150 = Meta.Field(7, false, 1, 8, 1, 0, 0, null)
        val _ad_hoc1509 = Meta.Field(1, false, 1, 4, 6, 2, 0, null, 2)
        val _ad_hoc243 = Meta.Field(7, false, 1, 6, 1, 0, 0, null)
        val _ad_hoc141 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc59 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc19 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1520 = Meta.Field(5, false, 8, 1, 6, 2, 0, null, 2)
        val _ad_hoc1522 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc710 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc2084 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1512 = Meta.Field(2, false, 1, 4, 6, 12, 5, null, 2)
        val _ad_hoc1519 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1518 = Meta.Field(2, false, 1, 4, 18, 10, 5, null)
        val _ad_hoc1134 = Meta.Field(1, true, 1, 2, 1, 0, 0, null)
        val _ad_hoc425 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc957 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc2083 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc351 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc326 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc281 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc1989 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc881 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc233 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc587 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc422 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc882 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc377 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc683 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc426 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc1012 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc804 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc2118 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc2004 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc83 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc244 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc106 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc219 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc1332 = Meta.Field(1, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc803 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1507 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc248 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc802 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc972 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1865 = Meta.Field(1, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc423 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc421 = Meta.Field(7, false, 1, 8, 1, 0, 0, null)
        val _ad_hoc112 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc2074 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc209 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc376 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc20 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1501 = Meta.Field(7, false, 1, 5, 9, 2, 0, null, 2)
        val _ad_hoc880 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1515 = Meta.Field(1, false, 1, 4, 6, 2, 0, null, 2)
        val _ad_hoc342 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc684 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc828 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc188 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc1188 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1513 = Meta.Field(2, false, 1, 4, 18, 10, 5, null)
        val _ad_hoc208 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc1132 = Meta.Field(1, true, 1, 2, 1, 0, 0, null)
        val _ad_hoc213 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1032 = Meta.Field(1, true, 1, 4, 1, 0, 0, null)
        val _ad_hoc1487 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc151 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc706 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc630 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1942 = Meta.Field(1, true, 1, 8, 1, 0, 0, null)
        val _ad_hoc285 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc51 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1517 = Meta.Field(2, false, 1, 4, 18, 10, 5, null)
        val _ad_hoc58 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc149 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1492 = Meta.Field(2, true, 1, 4, 18, 10, 5, null)
        val _ad_hoc628 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc16 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1008 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc352 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc153 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc685 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc192 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc195 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc105 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc282 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc86 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc212 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc84 = Meta.Field(7, false, 1, 2, 1, 0, 0, null)
        val _ad_hoc705 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1516 = Meta.Field(2, false, 1, 1, 18, 10, 5, null)
        val _ad_hoc1980 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc17 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc1033 = Meta.Field(1, true, 1, 4, 1, 0, 0, null)
        val _ad_hoc1861 = Meta.Field(1, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1138 = Meta.Field(1, true, 1, 2, 1, 0, 0, null)
        val _ad_hoc1181 = Meta.Field(1, true, 1, 8, 1, 0, 0, null)
        val _ad_hoc1864 = Meta.Field(2, false, 1, 4, 4, 6, 3, null)
        val _ad_hoc1954 = Meta.Field(1, true, 1, 8, 1, 0, 0, null)
        val _ad_hoc826 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc420 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc10 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc411 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc999 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc372 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc1504 = Meta.Field(7, false, 1, 6, 9, 2, 0, null, 2)
        val _ad_hoc1135 = Meta.Field(1, true, 1, 2, 1, 0, 0, null)
        val _ad_hoc80 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc1018 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1521 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc218 = Meta.Field(7, false, 1, 3, 1, 0, 0, null)
        val _ad_hoc1494 = Meta.Field(1, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc11 = Meta.Field(7, false, 1, 5, 1, 0, 0, null)
        val _ad_hoc883 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc1508 = Meta.Field(7, false, 1, 1, 1, 0, 0, null)
        val _ad_hoc2414 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1505 = Meta.Field(8, false, 1, 9, 9, 12, 5, null, 2)
        val _ad_hoc1331 = Meta.Field(1, false, 1, 4, 1, 0, 0, null)
        val _ad_hoc2085 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc1986 = Meta.Field(3, false, 8, 1, 1, 8, 0, null)
        val _ad_hoc152 = Meta.Field(7, false, 1, 4, 1, 0, 0, null)

        MISSION_COUNT.meta.fields[0] = _ad_hoc16
        ADSB_VEHICLE.meta.fields[0] = _ad_hoc10
        ADSB_VEHICLE.meta.fields[1] = _ad_hoc1964
        ADSB_VEHICLE.meta.fields[2] = _ad_hoc11
        ADSB_VEHICLE.meta.fields[3] = _ad_hoc12
        EKF_STATUS_REPORT.meta.fields[0] = _ad_hoc710
        ESTIMATOR_STATUS.meta.fields[0] = _ad_hoc9
        GLOBAL_POSITION_INT_COV.meta.fields[0] = _ad_hoc636
        SAFETY_SET_ALLOWED_AREA.meta.fields[0] = _ad_hoc306
        UAVCAN_NODE_STATUS.meta.fields[0] = _ad_hoc350
        UAVCAN_NODE_STATUS.meta.fields[1] = _ad_hoc349
        COLLISION.meta.fields[0] = _ad_hoc804
        COLLISION.meta.fields[1] = _ad_hoc803
        COLLISION.meta.fields[2] = _ad_hoc802
        CAMERA_INFORMATION.meta.fields[0] = _ad_hoc342
        CAMERA_INFORMATION.meta.fields[1] = _ad_hoc2016
        PARAM_SET.meta.fields[0] = _ad_hoc1018
        PARAM_SET.meta.fields[1] = _ad_hoc1
        VIDEO_STREAM_INFORMATION.meta.fields[0] = _ad_hoc2074
        CAMERA_IMAGE_CAPTURED.meta.fields[0] = _ad_hoc2043
        HIGH_LATENCY.meta.fields[0] = _ad_hoc213
        HIGH_LATENCY.meta.fields[1] = _ad_hoc215
        HIGH_LATENCY.meta.fields[2] = _ad_hoc214
        PARAM_REQUEST_READ.meta.fields[0] = _ad_hoc1008
        HOME_POSITION.meta.fields[0] = _ad_hoc1942
        FENCE_STATUS.meta.fields[0] = _ad_hoc85
        REMOTE_LOG_BLOCK_STATUS.meta.fields[0] = _ad_hoc372
        OBSTACLE_DISTANCE.meta.fields[0] = _ad_hoc282
        GPS2_RAW.meta.fields[0] = _ad_hoc972
        PARAM_EXT_REQUEST_READ.meta.fields[0] = _ad_hoc2106
        HIL_CONTROLS.meta.fields[0] = _ad_hoc353
        UAVIONIX_ADSB_OUT_CFG.meta.fields[0] = _ad_hoc2126
        UAVIONIX_ADSB_OUT_CFG.meta.fields[1] = _ad_hoc209
        UAVIONIX_ADSB_OUT_CFG.meta.fields[2] = _ad_hoc212
        UAVIONIX_ADSB_OUT_CFG.meta.fields[3] = _ad_hoc210
        UAVIONIX_ADSB_OUT_CFG.meta.fields[4] = _ad_hoc208
        UAVIONIX_ADSB_OUT_CFG.meta.fields[5] = _ad_hoc211
        LANDING_TARGET.meta.fields[0] = _ad_hoc351
        LANDING_TARGET.meta.fields[1] = _ad_hoc1861
        LANDING_TARGET.meta.fields[2] = _ad_hoc1862
        LANDING_TARGET.meta.fields[3] = _ad_hoc1863
        LANDING_TARGET.meta.fields[4] = _ad_hoc1864
        LANDING_TARGET.meta.fields[5] = _ad_hoc352
        LANDING_TARGET.meta.fields[6] = _ad_hoc1865
        SET_POSITION_TARGET_GLOBAL_INT.meta.fields[0] = _ad_hoc957
        PING33.meta.fields[0] = _ad_hoc1487
        PING33.meta.fields[1] = _ad_hoc1488
        PING33.meta.fields[2] = _ad_hoc1490
        PING33.meta.fields[3] = _ad_hoc1491
        PING33.meta.fields[4] = _ad_hoc1492
        PING33.meta.fields[5] = _ad_hoc1494
        PING33.meta.fields[6] = _ad_hoc1498
        PING33.meta.fields[7] = _ad_hoc1499
        PING33.meta.fields[8] = _ad_hoc1500
        PING33.meta.fields[9] = _ad_hoc1501
        PING33.meta.fields[10] = _ad_hoc1502
        PING33.meta.fields[11] = _ad_hoc1503
        PING33.meta.fields[12] = _ad_hoc1504
        PING33.meta.fields[13] = _ad_hoc1505
        PING33.meta.fields[14] = _ad_hoc1506
        PING33.meta.fields[15] = _ad_hoc1507
        PING33.meta.fields[16] = _ad_hoc1508
        PING33.meta.fields[17] = _ad_hoc139
        PING33.meta.fields[18] = _ad_hoc1509
        PING33.meta.fields[19] = _ad_hoc1510
        PING33.meta.fields[20] = _ad_hoc1511
        PING33.meta.fields[21] = _ad_hoc1512
        PING33.meta.fields[22] = _ad_hoc1513
        PING33.meta.fields[23] = _ad_hoc1515
        PING33.meta.fields[24] = _ad_hoc1516
        PING33.meta.fields[25] = _ad_hoc1517
        PING33.meta.fields[26] = _ad_hoc1518
        PING33.meta.fields[27] = _ad_hoc1519
        PING33.meta.fields[28] = _ad_hoc1520
        PING33.meta.fields[29] = _ad_hoc1521
        PING33.meta.fields[30] = _ad_hoc1522
        RALLY_POINT.meta.fields[0] = _ad_hoc645
        ADAP_TUNING.meta.fields[0] = _ad_hoc963
        PARAM_EXT_VALUE.meta.fields[0] = _ad_hoc2110
        PARAM_EXT_VALUE.meta.fields[1] = _ad_hoc2111
        PARAM_EXT_VALUE.meta.fields[2] = _ad_hoc411
        LIMITS_STATUS.meta.fields[0] = _ad_hoc189
        LIMITS_STATUS.meta.fields[1] = _ad_hoc188
        LIMITS_STATUS.meta.fields[2] = _ad_hoc190
        LIMITS_STATUS.meta.fields[3] = _ad_hoc191
        CAMERA_FEEDBACK.meta.fields[0] = _ad_hoc4
        AUTH_KEY.meta.fields[0] = _ad_hoc1003
        LOCAL_POSITION_NED_COV.meta.fields[0] = _ad_hoc112
        STATUSTEXT.meta.fields[0] = _ad_hoc683
        STATUSTEXT.meta.fields[1] = _ad_hoc1991
        GOPRO_GET_REQUEST.meta.fields[0] = _ad_hoc107
        GPS_INPUT.meta.fields[0] = _ad_hoc881
        COMMAND_LONG.meta.fields[0] = _ad_hoc202
        GPS_RAW_INT.meta.fields[0] = _ad_hoc153
        GPS_RAW_INT.meta.fields[1] = _ad_hoc1029
        GPS_RAW_INT.meta.fields[2] = _ad_hoc1030
        GPS_RAW_INT.meta.fields[3] = _ad_hoc1031
        GPS_RAW_INT.meta.fields[4] = _ad_hoc1032
        GPS_RAW_INT.meta.fields[5] = _ad_hoc1033
        CAMERA_STATUS.meta.fields[0] = _ad_hoc684
        CAMERA_SETTINGS.meta.fields[0] = _ad_hoc84
        NAMED_VALUE_FLOAT.meta.fields[0] = _ad_hoc1986
        GOPRO_HEARTBEAT.meta.fields[0] = _ad_hoc629
        GOPRO_HEARTBEAT.meta.fields[1] = _ad_hoc630
        GOPRO_HEARTBEAT.meta.fields[2] = _ad_hoc628
        MISSION_WRITE_PARTIAL_LIST.meta.fields[0] = _ad_hoc422
        PID_TUNING.meta.fields[0] = _ad_hoc104
        SAFETY_ALLOWED_AREA.meta.fields[0] = _ad_hoc880
        MISSION_CLEAR_ALL.meta.fields[0] = _ad_hoc71
        MAG_CAL_REPORT.meta.fields[0] = _ad_hoc83
        HEARTBEAT.meta.fields[0] = _ad_hoc688
        HEARTBEAT.meta.fields[1] = _ad_hoc686
        HEARTBEAT.meta.fields[2] = _ad_hoc685
        HEARTBEAT.meta.fields[3] = _ad_hoc687
        PARAM_MAP_RC.meta.fields[0] = _ad_hoc1188
        POWER_STATUS.meta.fields[0] = _ad_hoc140
        REMOTE_LOG_DATA_BLOCK.meta.fields[0] = _ad_hoc219
        MOUNT_CONFIGURE.meta.fields[0] = _ad_hoc106
        MISSION_REQUEST_INT.meta.fields[0] = _ad_hoc105
        COMMAND_ACK.meta.fields[0] = _ad_hoc424
        COMMAND_ACK.meta.fields[1] = _ad_hoc423
        COMMAND_ACK.meta.fields[2] = _ad_hoc1330
        COMMAND_ACK.meta.fields[3] = _ad_hoc1331
        COMMAND_ACK.meta.fields[4] = _ad_hoc1332
        COMMAND_ACK.meta.fields[5] = _ad_hoc1333
        MISSION_REQUEST.meta.fields[0] = _ad_hoc218
        SET_HOME_POSITION.meta.fields[0] = _ad_hoc1954
        SET_MODE.meta.fields[0] = _ad_hoc80
        POSITION_TARGET_GLOBAL_INT.meta.fields[0] = _ad_hoc376
        WIFI_CONFIG_AP.meta.fields[0] = _ad_hoc2084
        WIFI_CONFIG_AP.meta.fields[1] = _ad_hoc2085
        SERVO_OUTPUT_RAW.meta.fields[0] = _ad_hoc1132
        SERVO_OUTPUT_RAW.meta.fields[1] = _ad_hoc1133
        SERVO_OUTPUT_RAW.meta.fields[2] = _ad_hoc1134
        SERVO_OUTPUT_RAW.meta.fields[3] = _ad_hoc1135
        SERVO_OUTPUT_RAW.meta.fields[4] = _ad_hoc1136
        SERVO_OUTPUT_RAW.meta.fields[5] = _ad_hoc1137
        SERVO_OUTPUT_RAW.meta.fields[6] = _ad_hoc1138
        SERVO_OUTPUT_RAW.meta.fields[7] = _ad_hoc1139
        MEMINFO.meta.fields[0] = _ad_hoc2160
        DEBUG_VECT.meta.fields[0] = _ad_hoc1980
        MISSION_ACK.meta.fields[0] = _ad_hoc192
        MISSION_ACK.meta.fields[1] = _ad_hoc193
        GOPRO_SET_RESPONSE.meta.fields[0] = _ad_hoc882
        GOPRO_SET_RESPONSE.meta.fields[1] = _ad_hoc883
        PARAM_VALUE.meta.fields[0] = _ad_hoc1012
        PARAM_VALUE.meta.fields[1] = _ad_hoc966
        BATTERY_STATUS.meta.fields[0] = _ad_hoc58
        BATTERY_STATUS.meta.fields[1] = _ad_hoc59
        SERIAL_CONTROL.meta.fields[0] = _ad_hoc706
        SERIAL_CONTROL.meta.fields[1] = _ad_hoc705
        SET_POSITION_TARGET_LOCAL_NED.meta.fields[0] = _ad_hoc51
        SET_GPS_GLOBAL_ORIGIN.meta.fields[0] = _ad_hoc1181
        PARAM_EXT_SET.meta.fields[0] = _ad_hoc2116
        PARAM_EXT_SET.meta.fields[1] = _ad_hoc2117
        PARAM_EXT_SET.meta.fields[2] = _ad_hoc152
        AUTOPILOT_VERSION.meta.fields[0] = _ad_hoc281
        AUTOPILOT_VERSION.meta.fields[1] = _ad_hoc1853
        MISSION_REQUEST_LIST.meta.fields[0] = _ad_hoc820
        SET_VIDEO_STREAM_SETTINGS.meta.fields[0] = _ad_hoc2083
        PLAY_TUNE.meta.fields[0] = _ad_hoc2004
        MISSION_REQUEST_PARTIAL_LIST.meta.fields[0] = _ad_hoc354
        PARAM_EXT_ACK.meta.fields[0] = _ad_hoc2118
        PARAM_EXT_ACK.meta.fields[1] = _ad_hoc2119
        PARAM_EXT_ACK.meta.fields[2] = _ad_hoc233
        PARAM_EXT_ACK.meta.fields[3] = _ad_hoc234
        UAVCAN_NODE_INFO.meta.fields[0] = _ad_hoc2097
        GPS_GLOBAL_ORIGIN.meta.fields[0] = _ad_hoc1185
        HIL_ACTUATOR_CONTROLS.meta.fields[0] = _ad_hoc326
        POSITION_TARGET_LOCAL_NED.meta.fields[0] = _ad_hoc86
        DEVICE_OP_WRITE.meta.fields[0] = _ad_hoc285
        DEVICE_OP_WRITE.meta.fields[1] = _ad_hoc2427
        DISTANCE_SENSOR.meta.fields[0] = _ad_hoc244
        DISTANCE_SENSOR.meta.fields[1] = _ad_hoc243
        CHANGE_OPERATOR_CONTROL.meta.fields[0] = _ad_hoc999
        GOPRO_SET_REQUEST.meta.fields[0] = _ad_hoc17
        SYS_STATUS.meta.fields[0] = _ad_hoc828
        SYS_STATUS.meta.fields[1] = _ad_hoc826
        SYS_STATUS.meta.fields[2] = _ad_hoc827
        MISSION_ITEM.meta.fields[0] = _ad_hoc149
        MISSION_ITEM.meta.fields[1] = _ad_hoc150
        MISSION_ITEM.meta.fields[2] = _ad_hoc151
        COMMAND_INT.meta.fields[0] = _ad_hoc420
        COMMAND_INT.meta.fields[1] = _ad_hoc421
        OPTICAL_FLOW.meta.fields[0] = _ad_hoc1471
        OPTICAL_FLOW.meta.fields[1] = _ad_hoc1472
        MISSION_ITEM_INT.meta.fields[0] = _ad_hoc195
        MISSION_ITEM_INT.meta.fields[1] = _ad_hoc196
        MISSION_ITEM_INT.meta.fields[2] = _ad_hoc194
        DEVICE_OP_READ.meta.fields[0] = _ad_hoc248
        DEVICE_OP_READ.meta.fields[1] = _ad_hoc2414
        MAG_CAL_PROGRESS.meta.fields[0] = _ad_hoc141
        EXTENDED_SYS_STATE.meta.fields[0] = _ad_hoc19
        EXTENDED_SYS_STATE.meta.fields[1] = _ad_hoc20
        UAVIONIX_ADSB_OUT_DYNAMIC.meta.fields[0] = _ad_hoc587
        UAVIONIX_ADSB_OUT_DYNAMIC.meta.fields[1] = _ad_hoc585
        UAVIONIX_ADSB_OUT_DYNAMIC.meta.fields[2] = _ad_hoc586
        GOPRO_GET_RESPONSE.meta.fields[0] = _ad_hoc426
        GOPRO_GET_RESPONSE.meta.fields[1] = _ad_hoc425
        UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.meta.fields[0] = _ad_hoc377
        NAMED_VALUE_INT.meta.fields[0] = _ad_hoc1989

    }

    return Cursor(null, 1, 2)
}


inline class PID_TUNING_AXIS(val value: Byte) {

    operator fun not() = PID_TUNING_AXIS(value.inv())
    operator fun contains(other: PID_TUNING_AXIS) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: PID_TUNING_AXIS) = PID_TUNING_AXIS(value or (other.value))
    infix fun and(other: PID_TUNING_AXIS) = PID_TUNING_AXIS(value and (other.value))
    infix fun xor(other: PID_TUNING_AXIS) = PID_TUNING_AXIS(value xor (other.value))

    companion object {
        val PID_TUNING_ROLL = PID_TUNING_AXIS(1)
        val PID_TUNING_PITCH = PID_TUNING_AXIS(2)
        val PID_TUNING_YAW = PID_TUNING_AXIS(3)
        val PID_TUNING_ACCZ = PID_TUNING_AXIS(4)
        val PID_TUNING_STEER = PID_TUNING_AXIS(5)
        val PID_TUNING_LANDING = PID_TUNING_AXIS(6)
    }
}


/**
 *Flags in EKF_STATUS message */
inline class ESTIMATOR_STATUS_FLAGS(val value: Int) {

    operator fun not() = ESTIMATOR_STATUS_FLAGS(value.inv())
    operator fun contains(other: ESTIMATOR_STATUS_FLAGS) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: ESTIMATOR_STATUS_FLAGS) = ESTIMATOR_STATUS_FLAGS(value or (other.value))
    infix fun and(other: ESTIMATOR_STATUS_FLAGS) = ESTIMATOR_STATUS_FLAGS(value and (other.value))
    infix fun xor(other: ESTIMATOR_STATUS_FLAGS) = ESTIMATOR_STATUS_FLAGS(value xor (other.value))

    companion object {
        val ESTIMATOR_ATTITUDE = ESTIMATOR_STATUS_FLAGS(1) //True if the attitude estimate is good
        val ESTIMATOR_VELOCITY_HORIZ = ESTIMATOR_STATUS_FLAGS(2) //True if the horizontal velocity estimate is good
        val ESTIMATOR_VELOCITY_VERT = ESTIMATOR_STATUS_FLAGS(4) //True if the  vertical velocity estimate is good
        val ESTIMATOR_POS_HORIZ_REL = ESTIMATOR_STATUS_FLAGS(8) //True if the horizontal position (relative) estimate is good
        val ESTIMATOR_POS_HORIZ_ABS = ESTIMATOR_STATUS_FLAGS(16) //True if the horizontal position (absolute) estimate is good
        val ESTIMATOR_POS_VERT_ABS = ESTIMATOR_STATUS_FLAGS(32) //True if the vertical position (absolute) estimate is good
        val ESTIMATOR_POS_VERT_AGL = ESTIMATOR_STATUS_FLAGS(64) //True if the vertical position (above ground) estimate is good

        /**
         *True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical
         *			 flow */
        val ESTIMATOR_CONST_POS_MODE = ESTIMATOR_STATUS_FLAGS(128)
        val ESTIMATOR_PRED_POS_HORIZ_REL = ESTIMATOR_STATUS_FLAGS(256) //True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimat
        val ESTIMATOR_PRED_POS_HORIZ_ABS = ESTIMATOR_STATUS_FLAGS(512) //True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimat
        val ESTIMATOR_GPS_GLITCH = ESTIMATOR_STATUS_FLAGS(1024) //True if the EKF has detected a GPS glitch

        inline fun get(id: Int): ESTIMATOR_STATUS_FLAGS {
            when (id) {
                0 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE
                1 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ
                2 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT
                3 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL
                4 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS
                5 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS
                6 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL
                7 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE
                8 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL
                9 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS
                10 ->
                    return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: ESTIMATOR_STATUS_FLAGS): Int {
            when (en) {
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE ->
                    return 0
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ ->
                    return 1
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT ->
                    return 2
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL ->
                    return 3
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS ->
                    return 4
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS ->
                    return 5
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL ->
                    return 6
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE ->
                    return 7
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL ->
                    return 8
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS ->
                    return 9
                ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH ->
                    return 10

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


inline class MAV_TYPE(val value: Byte) {

    operator fun not() = MAV_TYPE(value.inv())
    operator fun contains(other: MAV_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_TYPE) = MAV_TYPE(value or (other.value))
    infix fun and(other: MAV_TYPE) = MAV_TYPE(value and (other.value))
    infix fun xor(other: MAV_TYPE) = MAV_TYPE(value xor (other.value))

    companion object {
        val GENERIC = MAV_TYPE(0) //Generic micro air vehicle.
        val FIXED_WING = MAV_TYPE(1) //Fixed wing aircraft.
        val QUADROTOR = MAV_TYPE(2) //Quadrotor
        val COAXIAL = MAV_TYPE(3) //Coaxial helicopter
        val HELICOPTER = MAV_TYPE(4) //Normal helicopter with tail rotor.
        val ANTENNA_TRACKER = MAV_TYPE(5) //Ground installation
        val GCS = MAV_TYPE(6) //Operator control unit / ground control station
        val AIRSHIP = MAV_TYPE(7) //Airship, controlled
        val FREE_BALLOON = MAV_TYPE(8) //Free balloon, uncontrolled
        val ROCKET = MAV_TYPE(9) //Rocket
        val GROUND_ROVER = MAV_TYPE(10) //Ground rover
        val SURFACE_BOAT = MAV_TYPE(11) //Surface vessel, boat, ship
        val SUBMARINE = MAV_TYPE(12) //Submarine
        val HEXAROTOR = MAV_TYPE(13) //Hexarotor
        val OCTOROTOR = MAV_TYPE(14) //Octorotor
        val TRICOPTER = MAV_TYPE(15) //Tricopter
        val FLAPPING_WING = MAV_TYPE(16) //Flapping wing
        val KITE = MAV_TYPE(17) //Kite
        val ONBOARD_CONTROLLER = MAV_TYPE(18) //Onboard companion controller
        val VTOL_DUOROTOR = MAV_TYPE(19) //Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
        val VTOL_QUADROTOR = MAV_TYPE(20) //Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
        val VTOL_TILTROTOR = MAV_TYPE(21) //Tiltrotor VTOL
        val VTOL_RESERVED2 = MAV_TYPE(22) //VTOL reserved 2
        val VTOL_RESERVED3 = MAV_TYPE(23) //VTOL reserved 3
        val VTOL_RESERVED4 = MAV_TYPE(24) //VTOL reserved 4
        val VTOL_RESERVED5 = MAV_TYPE(25) //VTOL reserved 5
        val GIMBAL = MAV_TYPE(26) //Onboard gimbal
        val ADSB = MAV_TYPE(27) //Onboard ADSB peripheral
        val PARAFOIL = MAV_TYPE(28) //Steerable, nonrigid airfoil
    }
}


inline class GOPRO_CAPTURE_MODE(val value: Int) {

    operator fun not() = GOPRO_CAPTURE_MODE(value.inv())
    operator fun contains(other: GOPRO_CAPTURE_MODE) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: GOPRO_CAPTURE_MODE) = GOPRO_CAPTURE_MODE(value or (other.value))
    infix fun and(other: GOPRO_CAPTURE_MODE) = GOPRO_CAPTURE_MODE(value and (other.value))
    infix fun xor(other: GOPRO_CAPTURE_MODE) = GOPRO_CAPTURE_MODE(value xor (other.value))

    companion object {
        val GOPRO_CAPTURE_MODE_VIDEO = GOPRO_CAPTURE_MODE(0) //Video mode
        val GOPRO_CAPTURE_MODE_PHOTO = GOPRO_CAPTURE_MODE(1) //Photo mode
        val GOPRO_CAPTURE_MODE_BURST = GOPRO_CAPTURE_MODE(2) //Burst mode, hero 3+ only
        val GOPRO_CAPTURE_MODE_TIME_LAPSE = GOPRO_CAPTURE_MODE(3) //Time lapse mode, hero 3+ only
        val GOPRO_CAPTURE_MODE_MULTI_SHOT = GOPRO_CAPTURE_MODE(4) //Multi shot mode, hero 4 only
        val GOPRO_CAPTURE_MODE_PLAYBACK = GOPRO_CAPTURE_MODE(5) //Playback mode, hero 4 only, silver only except when LCD or HDMI is connected to black
        val GOPRO_CAPTURE_MODE_SETUP = GOPRO_CAPTURE_MODE(6) //Playback mode, hero 4 only
        val GOPRO_CAPTURE_MODE_UNKNOWN = GOPRO_CAPTURE_MODE(255) //Mode not yet known

        inline fun get(id: Int): GOPRO_CAPTURE_MODE {
            when (id) {
                0 ->
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO
                1 ->
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO
                2 ->
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST
                3 ->
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE
                4 ->
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_MULTI_SHOT
                5 ->
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PLAYBACK
                6 ->
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_SETUP
                7 ->
                    return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_UNKNOWN

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: GOPRO_CAPTURE_MODE): Int {
            when (en) {
                GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO ->
                    return 0
                GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO ->
                    return 1
                GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST ->
                    return 2
                GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE ->
                    return 3
                GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_MULTI_SHOT ->
                    return 4
                GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PLAYBACK ->
                    return 5
                GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_SETUP ->
                    return 6
                GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_UNKNOWN ->
                    return 7

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Micro air vehicle / autopilot classes. This identifies the individual model. */
inline class MAV_AUTOPILOT(val value: Byte) {

    operator fun not() = MAV_AUTOPILOT(value.inv())
    operator fun contains(other: MAV_AUTOPILOT) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_AUTOPILOT) = MAV_AUTOPILOT(value or (other.value))
    infix fun and(other: MAV_AUTOPILOT) = MAV_AUTOPILOT(value and (other.value))
    infix fun xor(other: MAV_AUTOPILOT) = MAV_AUTOPILOT(value xor (other.value))

    companion object {
        val GENERIC = MAV_AUTOPILOT(0) //Generic autopilot, full support for everything
        val RESERVED = MAV_AUTOPILOT(1) //Reserved for future use.
        val SLUGS = MAV_AUTOPILOT(2) //SLUGS autopilot, http:slugsuav.soe.ucsc.edu
        val ARDUPILOTMEGA = MAV_AUTOPILOT(3) //ArduPilotMega / ArduCopter, http:diydrones.com
        val OPENPILOT = MAV_AUTOPILOT(4) //OpenPilot, http:openpilot.org
        val GENERIC_WAYPOINTS_ONLY = MAV_AUTOPILOT(5) //Generic autopilot only supporting simple waypoints
        val GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = MAV_AUTOPILOT(6) //Generic autopilot supporting waypoints and other simple navigation commands
        val GENERIC_MISSION_FULL = MAV_AUTOPILOT(7) //Generic autopilot supporting the full mission command set
        val INVALID = MAV_AUTOPILOT(8) //No valid autopilot, e.g. a GCS or other MAVLink component
        val PPZ = MAV_AUTOPILOT(9) //PPZ UAV - http:nongnu.org/paparazzi
        val UDB = MAV_AUTOPILOT(10) //UAV Dev Board
        val FP = MAV_AUTOPILOT(11) //FlexiPilot
        val PX4 = MAV_AUTOPILOT(12) //PX4 Autopilot - http:pixhawk.ethz.ch/px4/
        val SMACCMPILOT = MAV_AUTOPILOT(13) //SMACCMPilot - http:smaccmpilot.org
        val AUTOQUAD = MAV_AUTOPILOT(14) //AutoQuad -- http:autoquad.org
        val ARMAZILA = MAV_AUTOPILOT(15) //Armazila -- http:armazila.com
        val AEROB = MAV_AUTOPILOT(16) //Aerob -- http:aerob.ru
        val ASLUAV = MAV_AUTOPILOT(17) //ASLUAV autopilot -- http:www.asl.ethz.ch
        val SMARTAP = MAV_AUTOPILOT(18) //SmartAP Autopilot - http:sky-drones.com
    }
}


/**
 *Enumeration of battery functions */
inline class MAV_BATTERY_FUNCTION(val value: Byte) {

    operator fun not() = MAV_BATTERY_FUNCTION(value.inv())
    operator fun contains(other: MAV_BATTERY_FUNCTION) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_BATTERY_FUNCTION) = MAV_BATTERY_FUNCTION(value or (other.value))
    infix fun and(other: MAV_BATTERY_FUNCTION) = MAV_BATTERY_FUNCTION(value and (other.value))
    infix fun xor(other: MAV_BATTERY_FUNCTION) = MAV_BATTERY_FUNCTION(value xor (other.value))

    companion object {
        val MAV_BATTERY_FUNCTION_UNKNOWN = MAV_BATTERY_FUNCTION(0) //Battery function is unknown
        val MAV_BATTERY_FUNCTION_ALL = MAV_BATTERY_FUNCTION(1) //Battery supports all flight systems
        val MAV_BATTERY_FUNCTION_PROPULSION = MAV_BATTERY_FUNCTION(2) //Battery for the propulsion system
        val MAV_BATTERY_FUNCTION_AVIONICS = MAV_BATTERY_FUNCTION(3) //Avionics battery
        val MAV_BATTERY_TYPE_PAYLOAD = MAV_BATTERY_FUNCTION(4) //Payload battery
    }
}


/**
 *Type of landing target */
inline class LANDING_TARGET_TYPE(val value: Byte) {

    operator fun not() = LANDING_TARGET_TYPE(value.inv())
    operator fun contains(other: LANDING_TARGET_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: LANDING_TARGET_TYPE) = LANDING_TARGET_TYPE(value or (other.value))
    infix fun and(other: LANDING_TARGET_TYPE) = LANDING_TARGET_TYPE(value and (other.value))
    infix fun xor(other: LANDING_TARGET_TYPE) = LANDING_TARGET_TYPE(value xor (other.value))

    companion object {
        val LANDING_TARGET_TYPE_LIGHT_BEACON = LANDING_TARGET_TYPE(0) //Landing target signaled by light beacon (ex: IR-LOCK)
        val LANDING_TARGET_TYPE_RADIO_BEACON = LANDING_TARGET_TYPE(1) //Landing target signaled by radio beacon (ex: ILS, NDB)
        val LANDING_TARGET_TYPE_VISION_FIDUCIAL = LANDING_TARGET_TYPE(2) //Landing target represented by a fiducial marker (ex: ARTag)
        val LANDING_TARGET_TYPE_VISION_OTHER = LANDING_TARGET_TYPE(3) //Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
    }
}


inline class LIMIT_MODULE(val value: Int) {

    operator fun not() = LIMIT_MODULE(value.inv())
    operator fun contains(other: LIMIT_MODULE) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: LIMIT_MODULE) = LIMIT_MODULE(value or (other.value))
    infix fun and(other: LIMIT_MODULE) = LIMIT_MODULE(value and (other.value))
    infix fun xor(other: LIMIT_MODULE) = LIMIT_MODULE(value xor (other.value))

    companion object {
        val LIMIT_GPSLOCK = LIMIT_MODULE(1) //pre-initialization
        val LIMIT_GEOFENCE = LIMIT_MODULE(2) //disabled
        val LIMIT_ALTITUDE = LIMIT_MODULE(4) //checking limits

        inline fun get(id: Int): LIMIT_MODULE {
            when (id) {
                0 ->
                    return LIMIT_MODULE.LIMIT_GPSLOCK
                1 ->
                    return LIMIT_MODULE.LIMIT_GEOFENCE
                2 ->
                    return LIMIT_MODULE.LIMIT_ALTITUDE

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: LIMIT_MODULE): Int {
            when (en) {
                LIMIT_MODULE.LIMIT_GPSLOCK ->
                    return 0
                LIMIT_MODULE.LIMIT_GEOFENCE ->
                    return 1
                LIMIT_MODULE.LIMIT_ALTITUDE ->
                    return 2

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Enumeration of landed detector states */
inline class MAV_LANDED_STATE(val value: Byte) {

    operator fun not() = MAV_LANDED_STATE(value.inv())
    operator fun contains(other: MAV_LANDED_STATE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_LANDED_STATE) = MAV_LANDED_STATE(value or (other.value))
    infix fun and(other: MAV_LANDED_STATE) = MAV_LANDED_STATE(value and (other.value))
    infix fun xor(other: MAV_LANDED_STATE) = MAV_LANDED_STATE(value xor (other.value))

    companion object {
        val MAV_LANDED_STATE_UNDEFINED = MAV_LANDED_STATE(0) //MAV landed state is unknown
        val MAV_LANDED_STATE_ON_GROUND = MAV_LANDED_STATE(1) //MAV is landed (on ground)
        val MAV_LANDED_STATE_IN_AIR = MAV_LANDED_STATE(2) //MAV is in air
        val MAV_LANDED_STATE_TAKEOFF = MAV_LANDED_STATE(3) //MAV currently taking off
        val MAV_LANDED_STATE_LANDING = MAV_LANDED_STATE(4) //MAV currently landing
    }
}


/**
 *Specifies the datatype of a MAVLink parameter. */
inline class MAV_PARAM_TYPE(val value: Byte) {

    operator fun not() = MAV_PARAM_TYPE(value.inv())
    operator fun contains(other: MAV_PARAM_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_PARAM_TYPE) = MAV_PARAM_TYPE(value or (other.value))
    infix fun and(other: MAV_PARAM_TYPE) = MAV_PARAM_TYPE(value and (other.value))
    infix fun xor(other: MAV_PARAM_TYPE) = MAV_PARAM_TYPE(value xor (other.value))

    companion object {
        val MAV_PARAM_TYPE_UINT8 = MAV_PARAM_TYPE(1) //8-bit unsigned integer
        val MAV_PARAM_TYPE_INT8 = MAV_PARAM_TYPE(2) //8-bit signed integer
        val MAV_PARAM_TYPE_UINT16 = MAV_PARAM_TYPE(3) //16-bit unsigned integer
        val MAV_PARAM_TYPE_INT16 = MAV_PARAM_TYPE(4) //16-bit signed integer
        val MAV_PARAM_TYPE_UINT32 = MAV_PARAM_TYPE(5) //32-bit unsigned integer
        val MAV_PARAM_TYPE_INT32 = MAV_PARAM_TYPE(6) //32-bit signed integer
        val MAV_PARAM_TYPE_UINT64 = MAV_PARAM_TYPE(7) //64-bit unsigned integer
        val MAV_PARAM_TYPE_INT64 = MAV_PARAM_TYPE(8) //64-bit signed integer
        val MAV_PARAM_TYPE_REAL32 = MAV_PARAM_TYPE(9) //32-bit floating-point
        val MAV_PARAM_TYPE_REAL64 = MAV_PARAM_TYPE(10) //64-bit floating-point
    }
}


/**
 *Emergency status encoding */
inline class UAVIONIX_ADSB_EMERGENCY_STATUS(val value: Byte) {

    operator fun not() = UAVIONIX_ADSB_EMERGENCY_STATUS(value.inv())
    operator fun contains(other: UAVIONIX_ADSB_EMERGENCY_STATUS) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: UAVIONIX_ADSB_EMERGENCY_STATUS) = UAVIONIX_ADSB_EMERGENCY_STATUS(value or (other.value))
    infix fun and(other: UAVIONIX_ADSB_EMERGENCY_STATUS) = UAVIONIX_ADSB_EMERGENCY_STATUS(value and (other.value))
    infix fun xor(other: UAVIONIX_ADSB_EMERGENCY_STATUS) = UAVIONIX_ADSB_EMERGENCY_STATUS(value xor (other.value))

    companion object {
        val UAVIONIX_ADSB_OUT_NO_EMERGENCY = UAVIONIX_ADSB_EMERGENCY_STATUS(0)
        val UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY = UAVIONIX_ADSB_EMERGENCY_STATUS(1)
        val UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY = UAVIONIX_ADSB_EMERGENCY_STATUS(2)
        val UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY = UAVIONIX_ADSB_EMERGENCY_STATUS(3)
        val UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY = UAVIONIX_ADSB_EMERGENCY_STATUS(4)
        val UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY = UAVIONIX_ADSB_EMERGENCY_STATUS(5)
        val UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY = UAVIONIX_ADSB_EMERGENCY_STATUS(6)
        val UAVIONIX_ADSB_OUT_RESERVED = UAVIONIX_ADSB_EMERGENCY_STATUS(7)
    }
}


/**
 *Indicates the severity level, generally used for status messages to indicate their relative urgency. Based
 *		 on RFC-5424 using expanded definitions at: http:www.kiwisyslog.com/kb/info:-syslog-message-levels/ */
inline class MAV_SEVERITY(val value: Byte) {

    operator fun not() = MAV_SEVERITY(value.inv())
    operator fun contains(other: MAV_SEVERITY) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_SEVERITY) = MAV_SEVERITY(value or (other.value))
    infix fun and(other: MAV_SEVERITY) = MAV_SEVERITY(value and (other.value))
    infix fun xor(other: MAV_SEVERITY) = MAV_SEVERITY(value xor (other.value))

    companion object {
        val MAV_SEVERITY_EMERGENCY = MAV_SEVERITY(0) //System is unusable. This is a "panic" condition.
        val MAV_SEVERITY_ALERT = MAV_SEVERITY(1) //Action should be taken immediately. Indicates error in non-critical systems.
        val MAV_SEVERITY_CRITICAL = MAV_SEVERITY(2) //Action must be taken immediately. Indicates failure in a primary system.
        val MAV_SEVERITY_ERROR = MAV_SEVERITY(3) //Indicates an error in secondary/redundant systems.

        /**
         *Indicates about a possible future error if this is not resolved within a given timeframe. Example would
         *			 be a low battery warning */
        val MAV_SEVERITY_WARNING = MAV_SEVERITY(4)

        /**
         *An unusual event has occured, though not an error condition. This should be investigated for the root
         *			 cause */
        val MAV_SEVERITY_NOTICE = MAV_SEVERITY(5)
        val MAV_SEVERITY_INFO = MAV_SEVERITY(6) //Normal operational messages. Useful for logging. No action is required for these messages.
        val MAV_SEVERITY_DEBUG = MAV_SEVERITY(7) //Useful non-operational messages that can assist in debugging. These should not occur during normal operation
    }
}


/**
 *These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
 *		 simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. */
inline class MAV_MODE(val value: Int) {

    operator fun not() = MAV_MODE(value.inv())
    operator fun contains(other: MAV_MODE) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: MAV_MODE) = MAV_MODE(value or (other.value))
    infix fun and(other: MAV_MODE) = MAV_MODE(value and (other.value))
    infix fun xor(other: MAV_MODE) = MAV_MODE(value xor (other.value))

    companion object {
        val PREFLIGHT = MAV_MODE(0) //System is not ready to fly, booting, calibrating, etc. No flag is set.
        val MANUAL_DISARMED = MAV_MODE(64) //System is allowed to be active, under manual (RC) control, no stabilization
        val MAV_MODE_TEST_DISARMED = MAV_MODE(66) //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
        val STABILIZE_DISARMED = MAV_MODE(80) //System is allowed to be active, under assisted RC control.
        val GUIDED_DISARMED = MAV_MODE(88) //System is allowed to be active, under autonomous control, manual setpoint

        /**
         *System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
         *			 and not pre-programmed by waypoints */
        val MAV_MODE_AUTO_DISARMED = MAV_MODE(92)
        val MANUAL_ARMED = MAV_MODE(192) //System is allowed to be active, under manual (RC) control, no stabilization
        val MAV_MODE_TEST_ARMED = MAV_MODE(194) //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
        val STABILIZE_ARMED = MAV_MODE(208) //System is allowed to be active, under assisted RC control.
        val GUIDED_ARMED = MAV_MODE(216) //System is allowed to be active, under autonomous control, manual setpoint

        /**
         *System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
         *			 and not pre-programmed by waypoints */
        val MAV_MODE_AUTO_ARMED = MAV_MODE(220)

        inline fun get(id: Int): MAV_MODE {
            when (id) {
                0 ->
                    return MAV_MODE.PREFLIGHT
                1 ->
                    return MAV_MODE.MANUAL_DISARMED
                2 ->
                    return MAV_MODE.MAV_MODE_TEST_DISARMED
                3 ->
                    return MAV_MODE.STABILIZE_DISARMED
                4 ->
                    return MAV_MODE.GUIDED_DISARMED
                5 ->
                    return MAV_MODE.MAV_MODE_AUTO_DISARMED
                6 ->
                    return MAV_MODE.MANUAL_ARMED
                7 ->
                    return MAV_MODE.MAV_MODE_TEST_ARMED
                8 ->
                    return MAV_MODE.STABILIZE_ARMED
                9 ->
                    return MAV_MODE.GUIDED_ARMED
                10 ->
                    return MAV_MODE.MAV_MODE_AUTO_ARMED

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: MAV_MODE): Int {
            when (en) {
                MAV_MODE.PREFLIGHT ->
                    return 0
                MAV_MODE.MANUAL_DISARMED ->
                    return 1
                MAV_MODE.MAV_MODE_TEST_DISARMED ->
                    return 2
                MAV_MODE.STABILIZE_DISARMED ->
                    return 3
                MAV_MODE.GUIDED_DISARMED ->
                    return 4
                MAV_MODE.MAV_MODE_AUTO_DISARMED ->
                    return 5
                MAV_MODE.MANUAL_ARMED ->
                    return 6
                MAV_MODE.MAV_MODE_TEST_ARMED ->
                    return 7
                MAV_MODE.STABILIZE_ARMED ->
                    return 8
                MAV_MODE.GUIDED_ARMED ->
                    return 9
                MAV_MODE.MAV_MODE_AUTO_ARMED ->
                    return 10

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Enumeration of the ADSB altimeter types */
inline class ADSB_ALTITUDE_TYPE(val value: Byte) {

    operator fun not() = ADSB_ALTITUDE_TYPE(value.inv())
    operator fun contains(other: ADSB_ALTITUDE_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: ADSB_ALTITUDE_TYPE) = ADSB_ALTITUDE_TYPE(value or (other.value))
    infix fun and(other: ADSB_ALTITUDE_TYPE) = ADSB_ALTITUDE_TYPE(value and (other.value))
    infix fun xor(other: ADSB_ALTITUDE_TYPE) = ADSB_ALTITUDE_TYPE(value xor (other.value))

    companion object {
        val ADSB_ALTITUDE_TYPE_PRESSURE_QNH = ADSB_ALTITUDE_TYPE(0) //Altitude reported from a Baro source using QNH reference
        val ADSB_ALTITUDE_TYPE_GEOMETRIC = ADSB_ALTITUDE_TYPE(1) //Altitude reported from a GNSS source
    }
}


/**
 *Type of mission items being requested/sent in mission protocol. */
inline class MAV_MISSION_TYPE(val value: Int) {

    operator fun not() = MAV_MISSION_TYPE(value.inv())
    operator fun contains(other: MAV_MISSION_TYPE) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: MAV_MISSION_TYPE) = MAV_MISSION_TYPE(value or (other.value))
    infix fun and(other: MAV_MISSION_TYPE) = MAV_MISSION_TYPE(value and (other.value))
    infix fun xor(other: MAV_MISSION_TYPE) = MAV_MISSION_TYPE(value xor (other.value))

    companion object {
        val MAV_MISSION_TYPE_MISSION = MAV_MISSION_TYPE(0) //Items are mission commands for main mission.
        val MAV_MISSION_TYPE_FENCE = MAV_MISSION_TYPE(1) //Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.

        /**
         *Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT
         *			 rally point items */
        val MAV_MISSION_TYPE_RALLY = MAV_MISSION_TYPE(2)
        val MAV_MISSION_TYPE_ALL = MAV_MISSION_TYPE(255) //Only used in MISSION_CLEAR_ALL to clear all mission types.

        inline fun get(id: Int): MAV_MISSION_TYPE {
            when (id) {
                0 ->
                    return MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION
                1 ->
                    return MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE
                2 ->
                    return MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY
                3 ->
                    return MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: MAV_MISSION_TYPE): Int {
            when (en) {
                MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION ->
                    return 0
                MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE ->
                    return 1
                MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY ->
                    return 2
                MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL ->
                    return 3

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Enumeration of distance sensor types */
inline class MAV_DISTANCE_SENSOR(val value: Byte) {

    operator fun not() = MAV_DISTANCE_SENSOR(value.inv())
    operator fun contains(other: MAV_DISTANCE_SENSOR) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_DISTANCE_SENSOR) = MAV_DISTANCE_SENSOR(value or (other.value))
    infix fun and(other: MAV_DISTANCE_SENSOR) = MAV_DISTANCE_SENSOR(value and (other.value))
    infix fun xor(other: MAV_DISTANCE_SENSOR) = MAV_DISTANCE_SENSOR(value xor (other.value))

    companion object {
        val MAV_DISTANCE_SENSOR_LASER = MAV_DISTANCE_SENSOR(0) //Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
        val MAV_DISTANCE_SENSOR_ULTRASOUND = MAV_DISTANCE_SENSOR(1) //Ultrasound rangefinder, e.g. MaxBotix units
        val MAV_DISTANCE_SENSOR_INFRARED = MAV_DISTANCE_SENSOR(2) //Infrared rangefinder, e.g. Sharp units
        val MAV_DISTANCE_SENSOR_RADAR = MAV_DISTANCE_SENSOR(3) //Radar type, e.g. uLanding units
        val MAV_DISTANCE_SENSOR_UNKNOWN = MAV_DISTANCE_SENSOR(4) //Broken or unknown type, e.g. analog units
    }
}


/**
 *Enumeration of VTOL states */
inline class MAV_VTOL_STATE(val value: Byte) {

    operator fun not() = MAV_VTOL_STATE(value.inv())
    operator fun contains(other: MAV_VTOL_STATE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_VTOL_STATE) = MAV_VTOL_STATE(value or (other.value))
    infix fun and(other: MAV_VTOL_STATE) = MAV_VTOL_STATE(value and (other.value))
    infix fun xor(other: MAV_VTOL_STATE) = MAV_VTOL_STATE(value xor (other.value))

    companion object {
        val MAV_VTOL_STATE_UNDEFINED = MAV_VTOL_STATE(0) //MAV is not configured as VTOL
        val MAV_VTOL_STATE_TRANSITION_TO_FW = MAV_VTOL_STATE(1) //VTOL is in transition from multicopter to fixed-wing
        val MAV_VTOL_STATE_TRANSITION_TO_MC = MAV_VTOL_STATE(2) //VTOL is in transition from fixed-wing to multicopter
        val MAV_VTOL_STATE_MC = MAV_VTOL_STATE(3) //VTOL is in multicopter state
        val MAV_VTOL_STATE_FW = MAV_VTOL_STATE(4) //VTOL is in fixed-wing state
    }
}


/**
 *Type of GPS fix */
inline class GPS_FIX_TYPE(val value: Byte) {

    operator fun not() = GPS_FIX_TYPE(value.inv())
    operator fun contains(other: GPS_FIX_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: GPS_FIX_TYPE) = GPS_FIX_TYPE(value or (other.value))
    infix fun and(other: GPS_FIX_TYPE) = GPS_FIX_TYPE(value and (other.value))
    infix fun xor(other: GPS_FIX_TYPE) = GPS_FIX_TYPE(value xor (other.value))

    companion object {
        val GPS_FIX_TYPE_NO_GPS = GPS_FIX_TYPE(0) //No GPS connected
        val GPS_FIX_TYPE_NO_FIX = GPS_FIX_TYPE(1) //No position information, GPS is connected
        val GPS_FIX_TYPE_2D_FIX = GPS_FIX_TYPE(2) //2D position
        val GPS_FIX_TYPE_3D_FIX = GPS_FIX_TYPE(3) //3D position
        val GPS_FIX_TYPE_DGPS = GPS_FIX_TYPE(4) //DGPS/SBAS aided 3D position
        val GPS_FIX_TYPE_RTK_FLOAT = GPS_FIX_TYPE(5) //RTK float, 3D position
        val GPS_FIX_TYPE_RTK_FIXED = GPS_FIX_TYPE(6) //RTK Fixed, 3D position
        val GPS_FIX_TYPE_STATIC = GPS_FIX_TYPE(7) //Static fixed, typically used for base stations
        val GPS_FIX_TYPE_PPP = GPS_FIX_TYPE(8) //PPP, 3D position.
    }
}


/**
 *Specifies the datatype of a MAVLink extended parameter. */
inline class MAV_PARAM_EXT_TYPE(val value: Byte) {

    operator fun not() = MAV_PARAM_EXT_TYPE(value.inv())
    operator fun contains(other: MAV_PARAM_EXT_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_PARAM_EXT_TYPE) = MAV_PARAM_EXT_TYPE(value or (other.value))
    infix fun and(other: MAV_PARAM_EXT_TYPE) = MAV_PARAM_EXT_TYPE(value and (other.value))
    infix fun xor(other: MAV_PARAM_EXT_TYPE) = MAV_PARAM_EXT_TYPE(value xor (other.value))

    companion object {
        val MAV_PARAM_EXT_TYPE_UINT8 = MAV_PARAM_EXT_TYPE(1) //8-bit unsigned integer
        val MAV_PARAM_EXT_TYPE_INT8 = MAV_PARAM_EXT_TYPE(2) //8-bit signed integer
        val MAV_PARAM_EXT_TYPE_UINT16 = MAV_PARAM_EXT_TYPE(3) //16-bit unsigned integer
        val MAV_PARAM_EXT_TYPE_INT16 = MAV_PARAM_EXT_TYPE(4) //16-bit signed integer
        val MAV_PARAM_EXT_TYPE_UINT32 = MAV_PARAM_EXT_TYPE(5) //32-bit unsigned integer
        val MAV_PARAM_EXT_TYPE_INT32 = MAV_PARAM_EXT_TYPE(6) //32-bit signed integer
        val MAV_PARAM_EXT_TYPE_UINT64 = MAV_PARAM_EXT_TYPE(7) //64-bit unsigned integer
        val MAV_PARAM_EXT_TYPE_INT64 = MAV_PARAM_EXT_TYPE(8) //64-bit signed integer
        val MAV_PARAM_EXT_TYPE_REAL32 = MAV_PARAM_EXT_TYPE(9) //32-bit floating-point
        val MAV_PARAM_EXT_TYPE_REAL64 = MAV_PARAM_EXT_TYPE(10) //64-bit floating-point
        val MAV_PARAM_EXT_TYPE_CUSTOM = MAV_PARAM_EXT_TYPE(11) //Custom Type
    }
}


/**
 *Enumeration of estimator types */
inline class MAV_ESTIMATOR_TYPE(val value: Byte) {

    operator fun not() = MAV_ESTIMATOR_TYPE(value.inv())
    operator fun contains(other: MAV_ESTIMATOR_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_ESTIMATOR_TYPE) = MAV_ESTIMATOR_TYPE(value or (other.value))
    infix fun and(other: MAV_ESTIMATOR_TYPE) = MAV_ESTIMATOR_TYPE(value and (other.value))
    infix fun xor(other: MAV_ESTIMATOR_TYPE) = MAV_ESTIMATOR_TYPE(value xor (other.value))

    companion object {
        val MAV_ESTIMATOR_TYPE_NAIVE = MAV_ESTIMATOR_TYPE(1) //This is a naive estimator without any real covariance feedback.
        val MAV_ESTIMATOR_TYPE_VISION = MAV_ESTIMATOR_TYPE(2) //Computer vision based estimate. Might be up to scale.
        val MAV_ESTIMATOR_TYPE_VIO = MAV_ESTIMATOR_TYPE(3) //Visual-inertial estimate.
        val MAV_ESTIMATOR_TYPE_GPS = MAV_ESTIMATOR_TYPE(4) //Plain GPS estimate.
        val MAV_ESTIMATOR_TYPE_GPS_INS = MAV_ESTIMATOR_TYPE(5) //Estimator integrating GPS and inertial sensing.
    }
}


inline class CAMERA_FEEDBACK_FLAGS(val value: Byte) {

    operator fun not() = CAMERA_FEEDBACK_FLAGS(value.inv())
    operator fun contains(other: CAMERA_FEEDBACK_FLAGS) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: CAMERA_FEEDBACK_FLAGS) = CAMERA_FEEDBACK_FLAGS(value or (other.value))
    infix fun and(other: CAMERA_FEEDBACK_FLAGS) = CAMERA_FEEDBACK_FLAGS(value and (other.value))
    infix fun xor(other: CAMERA_FEEDBACK_FLAGS) = CAMERA_FEEDBACK_FLAGS(value xor (other.value))

    companion object {
        val CAMERA_FEEDBACK_PHOTO = CAMERA_FEEDBACK_FLAGS(0) //Shooting photos, not video
        val CAMERA_FEEDBACK_VIDEO = CAMERA_FEEDBACK_FLAGS(1) //Shooting video, not stills
        val CAMERA_FEEDBACK_BADEXPOSURE = CAMERA_FEEDBACK_FLAGS(2) //Unable to achieve requested exposure (e.g. shutter speed too low)
        val CAMERA_FEEDBACK_CLOSEDLOOP = CAMERA_FEEDBACK_FLAGS(3) //Closed loop feedback from camera, we know for sure it has successfully taken a picture

        /**
         *Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken
         *			 a pictur */
        val CAMERA_FEEDBACK_OPENLOOP = CAMERA_FEEDBACK_FLAGS(4)
    }
}


/**
 *Generalized UAVCAN node health */
inline class UAVCAN_NODE_HEALTH(val value: Byte) {

    operator fun not() = UAVCAN_NODE_HEALTH(value.inv())
    operator fun contains(other: UAVCAN_NODE_HEALTH) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: UAVCAN_NODE_HEALTH) = UAVCAN_NODE_HEALTH(value or (other.value))
    infix fun and(other: UAVCAN_NODE_HEALTH) = UAVCAN_NODE_HEALTH(value and (other.value))
    infix fun xor(other: UAVCAN_NODE_HEALTH) = UAVCAN_NODE_HEALTH(value xor (other.value))

    companion object {
        val UAVCAN_NODE_HEALTH_OK = UAVCAN_NODE_HEALTH(0) //The node is functioning properly.
        val UAVCAN_NODE_HEALTH_WARNING = UAVCAN_NODE_HEALTH(1) //A critical parameter went out of range or the node has encountered a minor failure.
        val UAVCAN_NODE_HEALTH_ERROR = UAVCAN_NODE_HEALTH(2) //The node has encountered a major failure.
        val UAVCAN_NODE_HEALTH_CRITICAL = UAVCAN_NODE_HEALTH(3) //The node has suffered a fatal malfunction.
    }
}


/**
 *Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script.
 *		 If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows:
 *		 Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what
 *		 ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data */
inline class MAV_CMD(val value: Int) {

    operator fun not() = MAV_CMD(value.inv())
    operator fun contains(other: MAV_CMD) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: MAV_CMD) = MAV_CMD(value or (other.value))
    infix fun and(other: MAV_CMD) = MAV_CMD(value and (other.value))
    infix fun xor(other: MAV_CMD) = MAV_CMD(value xor (other.value))

    companion object {
        /**
         *Navigate to waypoint.
         *					 1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
         *					 2	Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
         *					 3	0 to pass through the WP, if 	>	0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
         *					 4	Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
         *					 5	Latitude
         *					 6	Longitude
         *					 7	Altitude */
        val MAV_CMD_NAV_WAYPOINT = MAV_CMD(16)

        /**
         *Loiter around this waypoint an unlimited amount of time
         *			 1	Empty
         *			 2	Empty
         *			 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
         *			 4	Desired yaw angle.
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude */
        val MAV_CMD_NAV_LOITER_UNLIM = MAV_CMD(17)

        /**
         *Loiter around this waypoint for X turns
         *			 1	Turns
         *			 2	Empty
         *			 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
         *			 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude */
        val MAV_CMD_NAV_LOITER_TURNS = MAV_CMD(18)

        /**
         *Loiter around this waypoint for X seconds
         *			 1	Seconds (decimal)
         *			 2	Empty
         *			 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
         *			 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude */
        val MAV_CMD_NAV_LOITER_TIME = MAV_CMD(19)

        /**
         *Return to launch location
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_NAV_RETURN_TO_LAUNCH = MAV_CMD(20)

        /**
         *Land at location
         *			 1	Abort Alt
         *			 2	Empty
         *			 3	Empty
         *			 4	Desired yaw angle. NaN for unchanged.
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude (ground level) */
        val MAV_CMD_NAV_LAND = MAV_CMD(21)

        /**
         *Takeoff from ground / hand
         *			 1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
         *			 2	Empty
         *			 3	Empty
         *			 4	Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude */
        val MAV_CMD_NAV_TAKEOFF = MAV_CMD(22)

        /**
         *Land at local position (local frame only)
         *			 1	Landing target number (if available)
         *			 2	Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
         *			 3	Landing descend rate [ms^-1]
         *			 4	Desired yaw angle [rad]
         *			 5	Y-axis position [m]
         *			 6	X-axis position [m]
         *			 7	Z-axis / ground level position [m] */
        val MAV_CMD_NAV_LAND_LOCAL = MAV_CMD(23)

        /**
         *Takeoff from local position (local frame only)
         *			 1	Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
         *			 2	Empty
         *			 3	Takeoff ascend rate [ms^-1]
         *			 4	Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
         *			 5	Y-axis position [m]
         *			 6	X-axis position [m]
         *			 7	Z-axis position [m] */
        val MAV_CMD_NAV_TAKEOFF_LOCAL = MAV_CMD(24)

        /**
         *Vehicle following, i.e. this waypoint represents the position of a moving vehicle
         *			 1	Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
         *			 2	Ground speed of vehicle to be followed
         *			 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
         *			 4	Desired yaw angle.
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude */
        val MAV_CMD_NAV_FOLLOW = MAV_CMD(25)

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
        val MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = MAV_CMD(30)

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
        val MAV_CMD_NAV_LOITER_TO_ALT = MAV_CMD(31)

        /**
         *Being following a target
         *			 1	System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode
         *			 2	RESERVED
         *			 3	RESERVED
         *			 4	altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home
         *			 5	altitude
         *			 6	RESERVED
         *			 7	TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout */
        val MAV_CMD_DO_FOLLOW = MAV_CMD(32)

        /**
         *Reposition the MAV after a follow target command has been sent
         *			 1	Camera q1 (where 0 is on the ray from the camera to the tracking device)
         *			 2	Camera q2
         *			 3	Camera q3
         *			 4	Camera q4
         *			 5	altitude offset from target (m)
         *			 6	X offset from target (m)
         *			 7	Y offset from target (m) */
        val MAV_CMD_DO_FOLLOW_REPOSITION = MAV_CMD(33)

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
        val MAV_CMD_NAV_ROI = MAV_CMD(80)

        /**
         *Control autonomous path planning on the MAV.
         *			 1	0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
         *			 2	0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
         *			 3	Empty
         *			 4	Yaw angle at goal, in compass degrees, [0..360]
         *			 5	Latitude/X of goal
         *			 6	Longitude/Y of goal
         *			 7	Altitude/Z of goal */
        val MAV_CMD_NAV_PATHPLANNING = MAV_CMD(81)

        /**
         *Navigate to waypoint using a spline path.
         *			 1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Latitude/X of goal
         *			 6	Longitude/Y of goal
         *			 7	Altitude/Z of goal */
        val MAV_CMD_NAV_SPLINE_WAYPOINT = MAV_CMD(82)

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
        val MAV_CMD_NAV_ALTITUDE_WAIT = MAV_CMD(83)

        /**
         *Takeoff from ground using VTOL mode
         *			 1	Empty
         *			 2	Front transition heading, see VTOL_TRANSITION_HEADING enum.
         *			 3	Empty
         *			 4	Yaw angle in degrees. NaN for unchanged.
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude */
        val MAV_CMD_NAV_VTOL_TAKEOFF = MAV_CMD(84)

        /**
         *Land using VTOL mode
         *			 1	Empty
         *			 2	Empty
         *			 3	Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
         *			 4	Yaw angle in degrees. NaN for unchanged.
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude (ground level) */
        val MAV_CMD_NAV_VTOL_LAND = MAV_CMD(85)

        /**
         *hand control over to an external controller
         *			 1	On / Off (	>	0.5f on)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_NAV_GUIDED_ENABLE = MAV_CMD(92)

        /**
         *Delay the next navigation command a number of seconds or until a specified time
         *			 1	Delay in seconds (decimal, -1 to enable time-of-day fields)
         *			 2	hour (24h format, UTC, -1 to ignore)
         *			 3	minute (24h format, UTC, -1 to ignore)
         *			 4	second (24h format, UTC)
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_NAV_DELAY = MAV_CMD(93)

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
        val MAV_CMD_NAV_PAYLOAD_PLACE = MAV_CMD(94)

        /**
         *NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeratio
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_NAV_LAST = MAV_CMD(95)

        /**
         *Delay mission state machine.
         *			 1	Delay in seconds (decimal)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_CONDITION_DELAY = MAV_CMD(112)

        /**
         *Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
         *			 1	Descent / Ascend rate (m/s)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Finish Altitude */
        val MAV_CMD_CONDITION_CHANGE_ALT = MAV_CMD(113)

        /**
         *Delay mission state machine until within desired distance of next NAV point.
         *			 1	Distance (meters)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_CONDITION_DISTANCE = MAV_CMD(114)

        /**
         *Reach a certain target angle.
         *			 1	target angle: [0-360], 0 is north
         *			 2	speed during yaw change:[deg per second]
         *			 3	direction: negative: counter clockwise, positive: clockwise [-1,1]
         *			 4	relative offset or absolute angle: [ 1,0]
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_CONDITION_YAW = MAV_CMD(115)

        /**
         *NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeratio
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_CONDITION_LAST = MAV_CMD(159)

        /**
         *Set system mode.
         *			 1	Mode, as defined by ENUM MAV_MODE
         *			 2	Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
         *			 3	Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_SET_MODE = MAV_CMD(176)

        /**
         *Jump to the desired command in the mission list.  Repeat this action only the specified number of time
         *			 1	Sequence number
         *			 2	Repeat count
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_JUMP = MAV_CMD(177)

        /**
         *Change speed and/or throttle set points.
         *			 1	Speed type (0=Airspeed, 1=Ground Speed)
         *			 2	Speed  (m/s, -1 indicates no change)
         *			 3	Throttle  ( Percent, -1 indicates no change)
         *			 4	absolute or relative [0,1]
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_CHANGE_SPEED = MAV_CMD(178)

        /**
         *Changes the home location either to the current location or a specified location.
         *			 1	Use current (1=use current location, 0=use specified location)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude */
        val MAV_CMD_DO_SET_HOME = MAV_CMD(179)

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
        val MAV_CMD_DO_SET_PARAMETER = MAV_CMD(180)

        /**
         *Set a relay to a condition.
         *			 1	Relay number
         *			 2	Setting (1=on, 0=off, others possible depending on system hardware)
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_SET_RELAY = MAV_CMD(181)

        /**
         *Cycle a relay on and off for a desired number of cyles with a desired period.
         *			 1	Relay number
         *			 2	Cycle count
         *			 3	Cycle time (seconds, decimal)
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_REPEAT_RELAY = MAV_CMD(182)

        /**
         *Set a servo to a desired PWM value.
         *			 1	Servo number
         *			 2	PWM (microseconds, 1000 to 2000 typical)
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_SET_SERVO = MAV_CMD(183)

        /**
         *Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period
         *			 1	Servo number
         *			 2	PWM (microseconds, 1000 to 2000 typical)
         *			 3	Cycle count
         *			 4	Cycle time (seconds)
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_REPEAT_SERVO = MAV_CMD(184)

        /**
         *Terminate flight immediately
         *			 1	Flight termination activated if 	>	0.5
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_FLIGHTTERMINATION = MAV_CMD(185)

        /**
         *Change altitude set point.
         *			 1	Altitude in meters
         *			 2	Mav frame of new altitude (see MAV_FRAME)
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_CHANGE_ALTITUDE = MAV_CMD(186)

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
        val MAV_CMD_DO_LAND_START = MAV_CMD(189)

        /**
         *Mission command to perform a landing from a rally point.
         *			 1	Break altitude (meters)
         *			 2	Landing speed (m/s)
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_RALLY_LAND = MAV_CMD(190)

        /**
         *Mission command to safely abort an autonmous landing.
         *			 1	Altitude (meters)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_GO_AROUND = MAV_CMD(191)

        /**
         *Reposition the vehicle to a specific WGS84 global position.
         *			 1	Ground speed, less than 0 (-1) for default
         *			 2	Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.
         *			 3	Reserved
         *			 4	Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
         *			 5	Latitude (deg * 1E7)
         *			 6	Longitude (deg * 1E7)
         *			 7	Altitude (meters) */
        val MAV_CMD_DO_REPOSITION = MAV_CMD(192)

        /**
         *If in a GPS controlled position mode, hold the current position or continue.
         *			 1	0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
         *			 2	Reserved
         *			 3	Reserved
         *			 4	Reserved
         *			 5	Reserved
         *			 6	Reserved
         *			 7	Reserved */
        val MAV_CMD_DO_PAUSE_CONTINUE = MAV_CMD(193)

        /**
         *Set moving direction to forward or reverse.
         *			 1	Direction (0=Forward, 1=Reverse)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_SET_REVERSE = MAV_CMD(194)

        /**
         *Control onboard camera system.
         *			 1	Camera ID (-1 for all)
         *			 2	Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
         *			 3	Transmission mode: 0: video stream, 	>	0: single images every n seconds (decimal)
         *			 4	Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_CONTROL_VIDEO = MAV_CMD(200)

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
        val MAV_CMD_DO_SET_ROI = MAV_CMD(201)

        /**
         *Mission command to configure an on-board camera controller system.
         *			 1	Modes: P, TV, AV, M, Etc
         *			 2	Shutter speed: Divisor number for one second
         *			 3	Aperture: F stop number
         *			 4	ISO number e.g. 80, 100, 200, Etc
         *			 5	Exposure type enumerator
         *			 6	Command Identity
         *			 7	Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off) */
        val MAV_CMD_DO_DIGICAM_CONFIGURE = MAV_CMD(202)

        /**
         *Mission command to control an on-board camera controller system.
         *			 1	Session control e.g. show/hide lens
         *			 2	Zoom's absolute position
         *			 3	Zooming step value to offset zoom from the current position
         *			 4	Focus Locking, Unlocking or Re-locking
         *			 5	Shooting Command
         *			 6	Command Identity
         *			 7	Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count. */
        val MAV_CMD_DO_DIGICAM_CONTROL = MAV_CMD(203)

        /**
         *Mission command to configure a camera or antenna mount
         *			 1	Mount operation mode (see MAV_MOUNT_MODE enum)
         *			 2	stabilize roll? (1 = yes, 0 = no)
         *			 3	stabilize pitch? (1 = yes, 0 = no)
         *			 4	stabilize yaw? (1 = yes, 0 = no)
         *			 5	roll input (0 = angle, 1 = angular rate)
         *			 6	pitch input (0 = angle, 1 = angular rate)
         *			 7	yaw input (0 = angle, 1 = angular rate) */
        val MAV_CMD_DO_MOUNT_CONFIGURE = MAV_CMD(204)

        /**
         *Mission command to control a camera or antenna mount
         *			 1	pitch depending on mount mode (degrees or degrees/second depending on pitch input).
         *			 2	roll depending on mount mode (degrees or degrees/second depending on roll input).
         *			 3	yaw depending on mount mode (degrees or degrees/second depending on yaw input).
         *			 4	alt in meters depending on mount mode.
         *			 5	latitude in degrees * 1E7, set if appropriate mount mode.
         *			 6	longitude in degrees * 1E7, set if appropriate mount mode.
         *			 7	MAV_MOUNT_MODE enum value */
        val MAV_CMD_DO_MOUNT_CONTROL = MAV_CMD(205)

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
        val MAV_CMD_DO_SET_CAM_TRIGG_DIST = MAV_CMD(206)

        /**
         *Mission command to enable the geofence
         *			 1	enable? (0=disable, 1=enable, 2=disable_floor_only)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_FENCE_ENABLE = MAV_CMD(207)

        /**
         *Mission command to trigger a parachute
         *			 1	action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_PARACHUTE = MAV_CMD(208)

        /**
         *Mission command to perform motor test
         *			 1	motor sequence number (a number from 1 to max number of motors on the vehicle)
         *			 2	throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
         *			 3	throttle
         *			 4	timeout (in seconds)
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_MOTOR_TEST = MAV_CMD(209)

        /**
         *Change to/from inverted flight
         *			 1	inverted (0=normal, 1=inverted)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_INVERTED_FLIGHT = MAV_CMD(210)

        /**
         *Mission command to operate EPM gripper
         *			 1	gripper number (a number from 1 to max number of grippers on the vehicle)
         *			 2	gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum)
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_GRIPPER = MAV_CMD(211)

        /**
         *Enable/disable autotune
         *			 1	enable (1: enable, 0:disable)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_AUTOTUNE_ENABLE = MAV_CMD(212)

        /**
         *Sets a desired vehicle turn angle and speed change
         *			 1	yaw angle to adjust steering by in centidegress
         *			 2	speed - normalized to 0 .. 1
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_NAV_SET_YAW_SPEED = MAV_CMD(213)

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
        val MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = MAV_CMD(214)

        /**
         *Mission command to control a camera or antenna mount, using a quaternion as reference.
         *			 1	q1 - quaternion param #1, w (1 in null-rotation)
         *			 2	q2 - quaternion param #2, x (0 in null-rotation)
         *			 3	q3 - quaternion param #3, y (0 in null-rotation)
         *			 4	q4 - quaternion param #4, z (0 in null-rotation)
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_MOUNT_CONTROL_QUAT = MAV_CMD(220)

        /**
         *set id of master controller
         *			 1	System ID
         *			 2	Component ID
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_GUIDED_MASTER = MAV_CMD(221)

        /**
         *set limits for external control
         *			 1	timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout
         *			 2	absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit
         *			 3	absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit
         *			 4	horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_GUIDED_LIMITS = MAV_CMD(222)

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
        val MAV_CMD_DO_ENGINE_CONTROL = MAV_CMD(223)

        /**
         *NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_LAST = MAV_CMD(240)

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
        val MAV_CMD_PREFLIGHT_CALIBRATION = MAV_CMD(241)

        /**
         *Set sensor offsets. This command will be only accepted if in pre-flight mode.
         *			 1	Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
         *			 2	X axis offset (or generic dimension 1), in the sensor's raw units
         *			 3	Y axis offset (or generic dimension 2), in the sensor's raw units
         *			 4	Z axis offset (or generic dimension 3), in the sensor's raw units
         *			 5	Generic dimension 4, in the sensor's raw units
         *			 6	Generic dimension 5, in the sensor's raw units
         *			 7	Generic dimension 6, in the sensor's raw units */
        val MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = MAV_CMD(242)

        /**
         *Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
         *			 1	1: Trigger actuator ID assignment and direction mapping.
         *			 2	Reserved
         *			 3	Reserved
         *			 4	Reserved
         *			 5	Reserved
         *			 6	Reserved
         *			 7	Reserved */
        val MAV_CMD_PREFLIGHT_UAVCAN = MAV_CMD(243)

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
        val MAV_CMD_PREFLIGHT_STORAGE = MAV_CMD(245)

        /**
         *Request the reboot or shutdown of system components.
         *			 1	0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
         *			 2	0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
         *			 3	WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded
         *			 4	WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded
         *			 5	Reserved, send 0
         *			 6	Reserved, send 0
         *			 7	WIP: ID (e.g. camera ID -1 for all IDs) */
        val MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = MAV_CMD(246)

        /**
         *Hold / continue the current action
         *			 1	MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan
         *			 2	MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position
         *			 3	MAV_FRAME coordinate frame of hold point
         *			 4	Desired yaw angle in degrees
         *			 5	Latitude / X position
         *			 6	Longitude / Y position
         *			 7	Altitude / Z position */
        val MAV_CMD_OVERRIDE_GOTO = MAV_CMD(252)

        /**
         *start running a mission
         *			 1	first_item: the first mission item to run
         *			 2	last_item:  the last mission item to run (after this item is run, the mission ends) */
        val MAV_CMD_MISSION_START = MAV_CMD(300)

        /**
         *Arms / Disarms a component
         *			 1	1 to arm, 0 to disarm */
        val MAV_CMD_COMPONENT_ARM_DISARM = MAV_CMD(400)

        /**
         *Request the home position from the vehicle.
         *			 1	Reserved
         *			 2	Reserved
         *			 3	Reserved
         *			 4	Reserved
         *			 5	Reserved
         *			 6	Reserved
         *			 7	Reserved */
        val MAV_CMD_GET_HOME_POSITION = MAV_CMD(410)

        /**
         *Starts receiver pairing
         *			 1	0:Spektrum
         *			 2	0:Spektrum DSM2, 1:Spektrum DSMX */
        val MAV_CMD_START_RX_PAIR = MAV_CMD(500)

        /**
         *Request the interval between messages for a particular MAVLink message ID
         *			 1	The MAVLink message ID */
        val MAV_CMD_GET_MESSAGE_INTERVAL = MAV_CMD(510)

        /**
         *Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREA
         *			 1	The MAVLink message ID
         *			 2	The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate. */
        val MAV_CMD_SET_MESSAGE_INTERVAL = MAV_CMD(511)

        /**
         *Request MAVLink protocol version compatibility
         *			 1	1: Request supported protocol versions by all nodes on the network
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_PROTOCOL_VERSION = MAV_CMD(519)

        /**
         *Request autopilot capabilities
         *			 1	1: Request autopilot version
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = MAV_CMD(520)

        /**
         *WIP: Request camera information (CAMERA_INFORMATION).
         *			 1	0: No action 1: Request camera capabilities
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_CAMERA_INFORMATION = MAV_CMD(521)

        /**
         *WIP: Request camera settings (CAMERA_SETTINGS).
         *			 1	0: No Action 1: Request camera settings
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_CAMERA_SETTINGS = MAV_CMD(522)

        /**
         *WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a
         *			 specific component's storage
         *			 1	Storage ID (0 for all, 1 for first, 2 for second, etc.)
         *			 2	0: No Action 1: Request storage information
         *			 3	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_STORAGE_INFORMATION = MAV_CMD(525)

        /**
         *WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the
         *			 command's target_component to target a specific component's storage
         *			 1	Storage ID (1 for first, 2 for second, etc.)
         *			 2	0: No action 1: Format storage
         *			 3	Reserved (all remaining params) */
        val MAV_CMD_STORAGE_FORMAT = MAV_CMD(526)

        /**
         *WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
         *			 1	0: No Action 1: Request camera capture status
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = MAV_CMD(527)

        /**
         *WIP: Request flight information (FLIGHT_INFORMATION)
         *			 1	1: Request flight information
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_FLIGHT_INFORMATION = MAV_CMD(528)

        /**
         *WIP: Reset all camera settings to Factory Default
         *			 1	0: No Action 1: Reset all settings
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_RESET_CAMERA_SETTINGS = MAV_CMD(529)

        /**
         *Set camera running mode. Use NAN for reserved values.
         *			 1	Reserved (Set to 0)
         *			 2	Camera mode (see CAMERA_MODE enum)
         *			 3	Reserved (all remaining params) */
        val MAV_CMD_SET_CAMERA_MODE = MAV_CMD(530)

        /**
         *Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values
         *			 1	Reserved (Set to 0)
         *			 2	Duration between two consecutive pictures (in seconds)
         *			 3	Number of images to capture total - 0 for unlimited capture
         *			 4	Reserved (all remaining params) */
        val MAV_CMD_IMAGE_START_CAPTURE = MAV_CMD(2000)

        /**
         *Stop image capture sequence Use NAN for reserved values.
         *			 1	Reserved (Set to 0)
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_IMAGE_STOP_CAPTURE = MAV_CMD(2001)

        /**
         *WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values.
         *			 1	Sequence number for missing CAMERA_IMAGE_CAPTURE packet
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = MAV_CMD(2002)

        /**
         *Enable or disable on-board camera triggering system.
         *			 1	Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
         *			 2	1 to reset the trigger sequence, -1 or 0 to ignore
         *			 3	1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore */
        val MAV_CMD_DO_TRIGGER_CONTROL = MAV_CMD(2003)

        /**
         *Starts video capture (recording). Use NAN for reserved values.
         *			 1	Reserved (Set to 0)
         *			 2	Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)
         *			 3	Reserved (all remaining params) */
        val MAV_CMD_VIDEO_START_CAPTURE = MAV_CMD(2500)

        /**
         *Stop the current video capture (recording). Use NAN for reserved values.
         *			 1	Reserved (Set to 0)
         *			 2	Reserved (all remaining params) */
        val MAV_CMD_VIDEO_STOP_CAPTURE = MAV_CMD(2501)

        /**
         *WIP: Start video streaming
         *			 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
         *			 2	Reserved */
        val MAV_CMD_VIDEO_START_STREAMING = MAV_CMD(2502)

        /**
         *WIP: Stop the current video streaming
         *			 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
         *			 2	Reserved */
        val MAV_CMD_VIDEO_STOP_STREAMING = MAV_CMD(2503)

        /**
         *WIP: Request video stream information (VIDEO_STREAM_INFORMATION)
         *			 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
         *			 2	0: No Action 1: Request video stream information
         *			 3	Reserved (all remaining params) */
        val MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = MAV_CMD(2504)

        /**
         *Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
         *			 1	Format: 0: ULog
         *			 2	Reserved (set to 0)
         *			 3	Reserved (set to 0)
         *			 4	Reserved (set to 0)
         *			 5	Reserved (set to 0)
         *			 6	Reserved (set to 0)
         *			 7	Reserved (set to 0) */
        val MAV_CMD_LOGGING_START = MAV_CMD(2510)

        /**
         *Request to stop streaming log data over MAVLink
         *			 1	Reserved (set to 0)
         *			 2	Reserved (set to 0)
         *			 3	Reserved (set to 0)
         *			 4	Reserved (set to 0)
         *			 5	Reserved (set to 0)
         *			 6	Reserved (set to 0)
         *			 7	Reserved (set to 0) */
        val MAV_CMD_LOGGING_STOP = MAV_CMD(2511)

        /**
         *1	Landing gear ID (default: 0, -1 for all)
         *			 2	Landing gear position (Down: 0, Up: 1, NAN for no change)
         *			 3	Reserved, set to NAN
         *			 4	Reserved, set to NAN
         *			 5	Reserved, set to NAN
         *			 6	Reserved, set to NAN
         *			 7	Reserved, set to NAN */
        val MAV_CMD_AIRFRAME_CONFIGURATION = MAV_CMD(2520)

        /**
         *Create a panorama at the current position
         *			 1	Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)
         *			 2	Viewing angle vertical of panorama (in degrees)
         *			 3	Speed of the horizontal rotation (in degrees per second)
         *			 4	Speed of the vertical rotation (in degrees per second) */
        val MAV_CMD_PANORAMA_CREATE = MAV_CMD(2800)

        /**
         *Request VTOL transition
         *			 1	The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used. */
        val MAV_CMD_DO_VTOL_TRANSITION = MAV_CMD(3000)

        /**
         *Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
         *			 <p>
         *			 1	Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle */
        val MAV_CMD_ARM_AUTHORIZATION_REQUEST = MAV_CMD(3001)

        /**
         *This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes. */
        val MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = MAV_CMD(4000)

        /**
         *This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
         *			 <p>
         *			 1	Radius of desired circle in CIRCLE_MODE
         *			 2	User defined
         *			 3	User defined
         *			 4	User defined
         *			 5	Unscaled target latitude of center of circle in CIRCLE_MODE
         *			 6	Unscaled target longitude of center of circle in CIRCLE_MODE */
        val MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = MAV_CMD(4001)

        /**
         *WIP: Delay mission state machine until gate has been reached.
         *			 1	Geometry: 0: orthogonal to path between previous and next waypoint.
         *			 2	Altitude: 0: ignore altitude
         *			 3	Empty
         *			 4	Empty
         *			 5	Latitude
         *			 6	Longitude
         *			 7	Altitude */
        val MAV_CMD_CONDITION_GATE = MAV_CMD(4501)

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
        val MAV_CMD_NAV_FENCE_RETURN_POINT = MAV_CMD(5000)

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
        val MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = MAV_CMD(5001)

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
        val MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = MAV_CMD(5002)

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
        val MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = MAV_CMD(5003)

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
        val MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = MAV_CMD(5004)

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
        val MAV_CMD_NAV_RALLY_POINT = MAV_CMD(5100)

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
        val MAV_CMD_UAVCAN_GET_NODE_INFO = MAV_CMD(5200)

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
        val MAV_CMD_PAYLOAD_PREPARE_DEPLOY = MAV_CMD(30001)

        /**
         *Control the payload deployment.
         *			 1	Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
         *			 2	Reserved
         *			 3	Reserved
         *			 4	Reserved
         *			 5	Reserved
         *			 6	Reserved
         *			 7	Reserved */
        val MAV_CMD_PAYLOAD_CONTROL_DEPLOY = MAV_CMD(30002)

        /**
         *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
         *			 1	User defined
         *			 2	User defined
         *			 3	User defined
         *			 4	User defined
         *			 5	Latitude unscaled
         *			 6	Longitude unscaled
         *			 7	Altitude, in meters AMSL */
        val MAV_CMD_WAYPOINT_USER_1 = MAV_CMD(31000)

        /**
         *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
         *			 1	User defined
         *			 2	User defined
         *			 3	User defined
         *			 4	User defined
         *			 5	Latitude unscaled
         *			 6	Longitude unscaled
         *			 7	Altitude, in meters AMSL */
        val MAV_CMD_WAYPOINT_USER_2 = MAV_CMD(31001)

        /**
         *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
         *			 1	User defined
         *			 2	User defined
         *			 3	User defined
         *			 4	User defined
         *			 5	Latitude unscaled
         *			 6	Longitude unscaled
         *			 7	Altitude, in meters AMSL */
        val MAV_CMD_WAYPOINT_USER_3 = MAV_CMD(31002)

        /**
         *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
         *			 1	User defined
         *			 2	User defined
         *			 3	User defined
         *			 4	User defined
         *			 5	Latitude unscaled
         *			 6	Longitude unscaled
         *			 7	Altitude, in meters AMSL */
        val MAV_CMD_WAYPOINT_USER_4 = MAV_CMD(31003)

        /**
         *User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
         *			 1	User defined
         *			 2	User defined
         *			 3	User defined
         *			 4	User defined
         *			 5	Latitude unscaled
         *			 6	Longitude unscaled
         *			 7	Altitude, in meters AMSL */
        val MAV_CMD_WAYPOINT_USER_5 = MAV_CMD(31004)

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
        val MAV_CMD_SPATIAL_USER_1 = MAV_CMD(31005)

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
        val MAV_CMD_SPATIAL_USER_2 = MAV_CMD(31006)

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
        val MAV_CMD_SPATIAL_USER_3 = MAV_CMD(31007)

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
        val MAV_CMD_SPATIAL_USER_4 = MAV_CMD(31008)

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
        val MAV_CMD_SPATIAL_USER_5 = MAV_CMD(31009)

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
        val MAV_CMD_USER_1 = MAV_CMD(31010)

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
        val MAV_CMD_USER_2 = MAV_CMD(31011)

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
        val MAV_CMD_USER_3 = MAV_CMD(31012)

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
        val MAV_CMD_USER_4 = MAV_CMD(31013)

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
        val MAV_CMD_USER_5 = MAV_CMD(31014)

        /**
         *A system wide power-off event has been initiated.
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_POWER_OFF_INITIATED = MAV_CMD(42000)

        /**
         *FLY button has been clicked.
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_SOLO_BTN_FLY_CLICK = MAV_CMD(42001)

        /**
         *FLY button has been held for 1.5 seconds.
         *			 1	Takeoff altitude
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_SOLO_BTN_FLY_HOLD = MAV_CMD(42002)

        /**
         *PAUSE button has been clicked.
         *			 1	1 if Solo is in a shot mode, 0 otherwise
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_SOLO_BTN_PAUSE_CLICK = MAV_CMD(42003)

        /**
         *Initiate a magnetometer calibration
         *			 1	uint8_t bitmask of magnetometers (0 means all)
         *			 2	Automatically retry on failure (0=no retry, 1=retry).
         *			 3	Save without user input (0=require input, 1=autosave).
         *			 4	Delay (seconds)
         *			 5	Autoreboot (0=user reboot, 1=autoreboot)
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_START_MAG_CAL = MAV_CMD(42424)

        /**
         *Initiate a magnetometer calibration
         *			 1	uint8_t bitmask of magnetometers (0 means all)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_ACCEPT_MAG_CAL = MAV_CMD(42425)

        /**
         *Cancel a running magnetometer calibration
         *			 1	uint8_t bitmask of magnetometers (0 means all)
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_CANCEL_MAG_CAL = MAV_CMD(42426)

        /**
         *Command autopilot to get into factory test/diagnostic mode
         *			 1	0 means get out of test mode, 1 means get into test mode
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_SET_FACTORY_TEST_MODE = MAV_CMD(42427)

        /**
         *Reply with the version banner
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_DO_SEND_BANNER = MAV_CMD(42428)

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
        val MAV_CMD_ACCELCAL_VEHICLE_POS = MAV_CMD(42429)

        /**
         *Causes the gimbal to reset and boot as if it was just powered on
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_GIMBAL_RESET = MAV_CMD(42501)

        /**
         *Reports progress and success or failure of gimbal axis calibration procedure
         *			 1	Gimbal axis we're reporting calibration progress for
         *			 2	Current calibration progress for this axis, 0x64=100%
         *			 3	Status of the calibration
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS = MAV_CMD(42502)

        /**
         *Starts commutation calibration on the gimbal
         *			 1	Empty
         *			 2	Empty
         *			 3	Empty
         *			 4	Empty
         *			 5	Empty
         *			 6	Empty
         *			 7	Empty */
        val MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION = MAV_CMD(42503)

        /**
         *Erases gimbal application and parameters
         *			 1	Magic number
         *			 2	Magic number
         *			 3	Magic number
         *			 4	Magic number
         *			 5	Magic number
         *			 6	Magic number
         *			 7	Magic number */
        val MAV_CMD_GIMBAL_FULL_RESET = MAV_CMD(42505)

        inline fun get(id: Int): MAV_CMD {
            when (id) {
                0 ->
                    return MAV_CMD.MAV_CMD_NAV_WAYPOINT
                1 ->
                    return MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM
                2 ->
                    return MAV_CMD.MAV_CMD_NAV_LOITER_TURNS
                3 ->
                    return MAV_CMD.MAV_CMD_NAV_LOITER_TIME
                4 ->
                    return MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH
                5 ->
                    return MAV_CMD.MAV_CMD_NAV_LAND
                6 ->
                    return MAV_CMD.MAV_CMD_NAV_TAKEOFF
                7 ->
                    return MAV_CMD.MAV_CMD_NAV_LAND_LOCAL
                8 ->
                    return MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL
                9 ->
                    return MAV_CMD.MAV_CMD_NAV_FOLLOW
                10 ->
                    return MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
                11 ->
                    return MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT
                12 ->
                    return MAV_CMD.MAV_CMD_DO_FOLLOW
                13 ->
                    return MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION
                14 ->
                    return MAV_CMD.MAV_CMD_NAV_ROI
                15 ->
                    return MAV_CMD.MAV_CMD_NAV_PATHPLANNING
                16 ->
                    return MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT
                17 ->
                    return MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT
                18 ->
                    return MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF
                19 ->
                    return MAV_CMD.MAV_CMD_NAV_VTOL_LAND
                20 ->
                    return MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE
                21 ->
                    return MAV_CMD.MAV_CMD_NAV_DELAY
                22 ->
                    return MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE
                23 ->
                    return MAV_CMD.MAV_CMD_NAV_LAST
                24 ->
                    return MAV_CMD.MAV_CMD_CONDITION_DELAY
                25 ->
                    return MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT
                26 ->
                    return MAV_CMD.MAV_CMD_CONDITION_DISTANCE
                27 ->
                    return MAV_CMD.MAV_CMD_CONDITION_YAW
                28 ->
                    return MAV_CMD.MAV_CMD_CONDITION_LAST
                29 ->
                    return MAV_CMD.MAV_CMD_DO_SET_MODE
                30 ->
                    return MAV_CMD.MAV_CMD_DO_JUMP
                31 ->
                    return MAV_CMD.MAV_CMD_DO_CHANGE_SPEED
                32 ->
                    return MAV_CMD.MAV_CMD_DO_SET_HOME
                33 ->
                    return MAV_CMD.MAV_CMD_DO_SET_PARAMETER
                34 ->
                    return MAV_CMD.MAV_CMD_DO_SET_RELAY
                35 ->
                    return MAV_CMD.MAV_CMD_DO_REPEAT_RELAY
                36 ->
                    return MAV_CMD.MAV_CMD_DO_SET_SERVO
                37 ->
                    return MAV_CMD.MAV_CMD_DO_REPEAT_SERVO
                38 ->
                    return MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION
                39 ->
                    return MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE
                40 ->
                    return MAV_CMD.MAV_CMD_DO_LAND_START
                41 ->
                    return MAV_CMD.MAV_CMD_DO_RALLY_LAND
                42 ->
                    return MAV_CMD.MAV_CMD_DO_GO_AROUND
                43 ->
                    return MAV_CMD.MAV_CMD_DO_REPOSITION
                44 ->
                    return MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE
                45 ->
                    return MAV_CMD.MAV_CMD_DO_SET_REVERSE
                46 ->
                    return MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO
                47 ->
                    return MAV_CMD.MAV_CMD_DO_SET_ROI
                48 ->
                    return MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE
                49 ->
                    return MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL
                50 ->
                    return MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE
                51 ->
                    return MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL
                52 ->
                    return MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST
                53 ->
                    return MAV_CMD.MAV_CMD_DO_FENCE_ENABLE
                54 ->
                    return MAV_CMD.MAV_CMD_DO_PARACHUTE
                55 ->
                    return MAV_CMD.MAV_CMD_DO_MOTOR_TEST
                56 ->
                    return MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT
                57 ->
                    return MAV_CMD.MAV_CMD_DO_GRIPPER
                58 ->
                    return MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE
                59 ->
                    return MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED
                60 ->
                    return MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL
                61 ->
                    return MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT
                62 ->
                    return MAV_CMD.MAV_CMD_DO_GUIDED_MASTER
                63 ->
                    return MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS
                64 ->
                    return MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL
                65 ->
                    return MAV_CMD.MAV_CMD_DO_LAST
                66 ->
                    return MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION
                67 ->
                    return MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS
                68 ->
                    return MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN
                69 ->
                    return MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE
                70 ->
                    return MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                71 ->
                    return MAV_CMD.MAV_CMD_OVERRIDE_GOTO
                72 ->
                    return MAV_CMD.MAV_CMD_MISSION_START
                73 ->
                    return MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM
                74 ->
                    return MAV_CMD.MAV_CMD_GET_HOME_POSITION
                75 ->
                    return MAV_CMD.MAV_CMD_START_RX_PAIR
                76 ->
                    return MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL
                77 ->
                    return MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL
                78 ->
                    return MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION
                79 ->
                    return MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                80 ->
                    return MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION
                81 ->
                    return MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS
                82 ->
                    return MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION
                83 ->
                    return MAV_CMD.MAV_CMD_STORAGE_FORMAT
                84 ->
                    return MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS
                85 ->
                    return MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION
                86 ->
                    return MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS
                87 ->
                    return MAV_CMD.MAV_CMD_SET_CAMERA_MODE
                88 ->
                    return MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE
                89 ->
                    return MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE
                90 ->
                    return MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE
                91 ->
                    return MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL
                92 ->
                    return MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE
                93 ->
                    return MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE
                94 ->
                    return MAV_CMD.MAV_CMD_VIDEO_START_STREAMING
                95 ->
                    return MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING
                96 ->
                    return MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION
                97 ->
                    return MAV_CMD.MAV_CMD_LOGGING_START
                98 ->
                    return MAV_CMD.MAV_CMD_LOGGING_STOP
                99 ->
                    return MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION
                100 ->
                    return MAV_CMD.MAV_CMD_PANORAMA_CREATE
                101 ->
                    return MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION
                102 ->
                    return MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST
                103 ->
                    return MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD
                104 ->
                    return MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE
                105 ->
                    return MAV_CMD.MAV_CMD_CONDITION_GATE
                106 ->
                    return MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT
                107 ->
                    return MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
                108 ->
                    return MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
                109 ->
                    return MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                110 ->
                    return MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
                111 ->
                    return MAV_CMD.MAV_CMD_NAV_RALLY_POINT
                112 ->
                    return MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO
                113 ->
                    return MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY
                114 ->
                    return MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY
                115 ->
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_1
                116 ->
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_2
                117 ->
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_3
                118 ->
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_4
                119 ->
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_5
                120 ->
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_1
                121 ->
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_2
                122 ->
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_3
                123 ->
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_4
                124 ->
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_5
                125 ->
                    return MAV_CMD.MAV_CMD_USER_1
                126 ->
                    return MAV_CMD.MAV_CMD_USER_2
                127 ->
                    return MAV_CMD.MAV_CMD_USER_3
                128 ->
                    return MAV_CMD.MAV_CMD_USER_4
                129 ->
                    return MAV_CMD.MAV_CMD_USER_5
                130 ->
                    return MAV_CMD.MAV_CMD_POWER_OFF_INITIATED
                131 ->
                    return MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK
                132 ->
                    return MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD
                133 ->
                    return MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK
                134 ->
                    return MAV_CMD.MAV_CMD_DO_START_MAG_CAL
                135 ->
                    return MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL
                136 ->
                    return MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL
                137 ->
                    return MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE
                138 ->
                    return MAV_CMD.MAV_CMD_DO_SEND_BANNER
                139 ->
                    return MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS
                140 ->
                    return MAV_CMD.MAV_CMD_GIMBAL_RESET
                141 ->
                    return MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS
                142 ->
                    return MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION
                143 ->
                    return MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: MAV_CMD): Int {
            when (en) {
                MAV_CMD.MAV_CMD_NAV_WAYPOINT ->
                    return 0
                MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM ->
                    return 1
                MAV_CMD.MAV_CMD_NAV_LOITER_TURNS ->
                    return 2
                MAV_CMD.MAV_CMD_NAV_LOITER_TIME ->
                    return 3
                MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH ->
                    return 4
                MAV_CMD.MAV_CMD_NAV_LAND ->
                    return 5
                MAV_CMD.MAV_CMD_NAV_TAKEOFF ->
                    return 6
                MAV_CMD.MAV_CMD_NAV_LAND_LOCAL ->
                    return 7
                MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL ->
                    return 8
                MAV_CMD.MAV_CMD_NAV_FOLLOW ->
                    return 9
                MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT ->
                    return 10
                MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT ->
                    return 11
                MAV_CMD.MAV_CMD_DO_FOLLOW ->
                    return 12
                MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION ->
                    return 13
                MAV_CMD.MAV_CMD_NAV_ROI ->
                    return 14
                MAV_CMD.MAV_CMD_NAV_PATHPLANNING ->
                    return 15
                MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT ->
                    return 16
                MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT ->
                    return 17
                MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF ->
                    return 18
                MAV_CMD.MAV_CMD_NAV_VTOL_LAND ->
                    return 19
                MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE ->
                    return 20
                MAV_CMD.MAV_CMD_NAV_DELAY ->
                    return 21
                MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE ->
                    return 22
                MAV_CMD.MAV_CMD_NAV_LAST ->
                    return 23
                MAV_CMD.MAV_CMD_CONDITION_DELAY ->
                    return 24
                MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT ->
                    return 25
                MAV_CMD.MAV_CMD_CONDITION_DISTANCE ->
                    return 26
                MAV_CMD.MAV_CMD_CONDITION_YAW ->
                    return 27
                MAV_CMD.MAV_CMD_CONDITION_LAST ->
                    return 28
                MAV_CMD.MAV_CMD_DO_SET_MODE ->
                    return 29
                MAV_CMD.MAV_CMD_DO_JUMP ->
                    return 30
                MAV_CMD.MAV_CMD_DO_CHANGE_SPEED ->
                    return 31
                MAV_CMD.MAV_CMD_DO_SET_HOME ->
                    return 32
                MAV_CMD.MAV_CMD_DO_SET_PARAMETER ->
                    return 33
                MAV_CMD.MAV_CMD_DO_SET_RELAY ->
                    return 34
                MAV_CMD.MAV_CMD_DO_REPEAT_RELAY ->
                    return 35
                MAV_CMD.MAV_CMD_DO_SET_SERVO ->
                    return 36
                MAV_CMD.MAV_CMD_DO_REPEAT_SERVO ->
                    return 37
                MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION ->
                    return 38
                MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE ->
                    return 39
                MAV_CMD.MAV_CMD_DO_LAND_START ->
                    return 40
                MAV_CMD.MAV_CMD_DO_RALLY_LAND ->
                    return 41
                MAV_CMD.MAV_CMD_DO_GO_AROUND ->
                    return 42
                MAV_CMD.MAV_CMD_DO_REPOSITION ->
                    return 43
                MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE ->
                    return 44
                MAV_CMD.MAV_CMD_DO_SET_REVERSE ->
                    return 45
                MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO ->
                    return 46
                MAV_CMD.MAV_CMD_DO_SET_ROI ->
                    return 47
                MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE ->
                    return 48
                MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL ->
                    return 49
                MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE ->
                    return 50
                MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL ->
                    return 51
                MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST ->
                    return 52
                MAV_CMD.MAV_CMD_DO_FENCE_ENABLE ->
                    return 53
                MAV_CMD.MAV_CMD_DO_PARACHUTE ->
                    return 54
                MAV_CMD.MAV_CMD_DO_MOTOR_TEST ->
                    return 55
                MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT ->
                    return 56
                MAV_CMD.MAV_CMD_DO_GRIPPER ->
                    return 57
                MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE ->
                    return 58
                MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED ->
                    return 59
                MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL ->
                    return 60
                MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT ->
                    return 61
                MAV_CMD.MAV_CMD_DO_GUIDED_MASTER ->
                    return 62
                MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS ->
                    return 63
                MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL ->
                    return 64
                MAV_CMD.MAV_CMD_DO_LAST ->
                    return 65
                MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION ->
                    return 66
                MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS ->
                    return 67
                MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN ->
                    return 68
                MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE ->
                    return 69
                MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN ->
                    return 70
                MAV_CMD.MAV_CMD_OVERRIDE_GOTO ->
                    return 71
                MAV_CMD.MAV_CMD_MISSION_START ->
                    return 72
                MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM ->
                    return 73
                MAV_CMD.MAV_CMD_GET_HOME_POSITION ->
                    return 74
                MAV_CMD.MAV_CMD_START_RX_PAIR ->
                    return 75
                MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL ->
                    return 76
                MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL ->
                    return 77
                MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION ->
                    return 78
                MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ->
                    return 79
                MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION ->
                    return 80
                MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS ->
                    return 81
                MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION ->
                    return 82
                MAV_CMD.MAV_CMD_STORAGE_FORMAT ->
                    return 83
                MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS ->
                    return 84
                MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION ->
                    return 85
                MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS ->
                    return 86
                MAV_CMD.MAV_CMD_SET_CAMERA_MODE ->
                    return 87
                MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE ->
                    return 88
                MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE ->
                    return 89
                MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE ->
                    return 90
                MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL ->
                    return 91
                MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE ->
                    return 92
                MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE ->
                    return 93
                MAV_CMD.MAV_CMD_VIDEO_START_STREAMING ->
                    return 94
                MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING ->
                    return 95
                MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION ->
                    return 96
                MAV_CMD.MAV_CMD_LOGGING_START ->
                    return 97
                MAV_CMD.MAV_CMD_LOGGING_STOP ->
                    return 98
                MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION ->
                    return 99
                MAV_CMD.MAV_CMD_PANORAMA_CREATE ->
                    return 100
                MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION ->
                    return 101
                MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST ->
                    return 102
                MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD ->
                    return 103
                MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE ->
                    return 104
                MAV_CMD.MAV_CMD_CONDITION_GATE ->
                    return 105
                MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT ->
                    return 106
                MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ->
                    return 107
                MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION ->
                    return 108
                MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION ->
                    return 109
                MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION ->
                    return 110
                MAV_CMD.MAV_CMD_NAV_RALLY_POINT ->
                    return 111
                MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO ->
                    return 112
                MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY ->
                    return 113
                MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY ->
                    return 114
                MAV_CMD.MAV_CMD_WAYPOINT_USER_1 ->
                    return 115
                MAV_CMD.MAV_CMD_WAYPOINT_USER_2 ->
                    return 116
                MAV_CMD.MAV_CMD_WAYPOINT_USER_3 ->
                    return 117
                MAV_CMD.MAV_CMD_WAYPOINT_USER_4 ->
                    return 118
                MAV_CMD.MAV_CMD_WAYPOINT_USER_5 ->
                    return 119
                MAV_CMD.MAV_CMD_SPATIAL_USER_1 ->
                    return 120
                MAV_CMD.MAV_CMD_SPATIAL_USER_2 ->
                    return 121
                MAV_CMD.MAV_CMD_SPATIAL_USER_3 ->
                    return 122
                MAV_CMD.MAV_CMD_SPATIAL_USER_4 ->
                    return 123
                MAV_CMD.MAV_CMD_SPATIAL_USER_5 ->
                    return 124
                MAV_CMD.MAV_CMD_USER_1 ->
                    return 125
                MAV_CMD.MAV_CMD_USER_2 ->
                    return 126
                MAV_CMD.MAV_CMD_USER_3 ->
                    return 127
                MAV_CMD.MAV_CMD_USER_4 ->
                    return 128
                MAV_CMD.MAV_CMD_USER_5 ->
                    return 129
                MAV_CMD.MAV_CMD_POWER_OFF_INITIATED ->
                    return 130
                MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK ->
                    return 131
                MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD ->
                    return 132
                MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK ->
                    return 133
                MAV_CMD.MAV_CMD_DO_START_MAG_CAL ->
                    return 134
                MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL ->
                    return 135
                MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL ->
                    return 136
                MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE ->
                    return 137
                MAV_CMD.MAV_CMD_DO_SEND_BANNER ->
                    return 138
                MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS ->
                    return 139
                MAV_CMD.MAV_CMD_GIMBAL_RESET ->
                    return 140
                MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS ->
                    return 141
                MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION ->
                    return 142
                MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET ->
                    return 143

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Camera Modes. */
inline class CAMERA_MODE(val value: Byte) {

    operator fun not() = CAMERA_MODE(value.inv())
    operator fun contains(other: CAMERA_MODE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: CAMERA_MODE) = CAMERA_MODE(value or (other.value))
    infix fun and(other: CAMERA_MODE) = CAMERA_MODE(value and (other.value))
    infix fun xor(other: CAMERA_MODE) = CAMERA_MODE(value xor (other.value))

    companion object {
        val CAMERA_MODE_IMAGE = CAMERA_MODE(0) //Camera is in image/photo capture mode.
        val CAMERA_MODE_VIDEO = CAMERA_MODE(1) //Camera is in video capture mode.
        val CAMERA_MODE_IMAGE_SURVEY = CAMERA_MODE(2) //Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
    }
}


/**
 *Source of information about this collision. */
inline class MAV_COLLISION_SRC(val value: Byte) {

    operator fun not() = MAV_COLLISION_SRC(value.inv())
    operator fun contains(other: MAV_COLLISION_SRC) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_COLLISION_SRC) = MAV_COLLISION_SRC(value or (other.value))
    infix fun and(other: MAV_COLLISION_SRC) = MAV_COLLISION_SRC(value and (other.value))
    infix fun xor(other: MAV_COLLISION_SRC) = MAV_COLLISION_SRC(value xor (other.value))

    companion object {
        val MAV_COLLISION_SRC_ADSB = MAV_COLLISION_SRC(0) //ID field references ADSB_VEHICLE packets
        val MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = MAV_COLLISION_SRC(1) //ID field references MAVLink SRC ID
    }
}


/**
 *Possible remote log data block statuses */
inline class MAV_REMOTE_LOG_DATA_BLOCK_STATUSES(val value: Byte) {

    operator fun not() = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES(value.inv())
    operator fun contains(other: MAV_REMOTE_LOG_DATA_BLOCK_STATUSES) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_REMOTE_LOG_DATA_BLOCK_STATUSES) = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES(value or (other.value))
    infix fun and(other: MAV_REMOTE_LOG_DATA_BLOCK_STATUSES) = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES(value and (other.value))
    infix fun xor(other: MAV_REMOTE_LOG_DATA_BLOCK_STATUSES) = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES(value xor (other.value))

    companion object {
        val MAV_REMOTE_LOG_DATA_BLOCK_NACK = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES(0) //This block has NOT been received
        val MAV_REMOTE_LOG_DATA_BLOCK_ACK = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES(1) //This block has been received
    }
}


/**
 *Status flags for ADS-B transponder dynamic output */
inline class UAVIONIX_ADSB_RF_HEALTH(val value: Int) {

    operator fun not() = UAVIONIX_ADSB_RF_HEALTH(value.inv())
    operator fun contains(other: UAVIONIX_ADSB_RF_HEALTH) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: UAVIONIX_ADSB_RF_HEALTH) = UAVIONIX_ADSB_RF_HEALTH(value or (other.value))
    infix fun and(other: UAVIONIX_ADSB_RF_HEALTH) = UAVIONIX_ADSB_RF_HEALTH(value and (other.value))
    infix fun xor(other: UAVIONIX_ADSB_RF_HEALTH) = UAVIONIX_ADSB_RF_HEALTH(value xor (other.value))

    companion object {
        val UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = UAVIONIX_ADSB_RF_HEALTH(0)
        val UAVIONIX_ADSB_RF_HEALTH_OK = UAVIONIX_ADSB_RF_HEALTH(1)
        val UAVIONIX_ADSB_RF_HEALTH_FAIL_TX = UAVIONIX_ADSB_RF_HEALTH(2)
        val UAVIONIX_ADSB_RF_HEALTH_FAIL_RX = UAVIONIX_ADSB_RF_HEALTH(16)

        inline fun get(id: Int): UAVIONIX_ADSB_RF_HEALTH {
            when (id) {
                0 ->
                    return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING
                1 ->
                    return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK
                2 ->
                    return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX
                3 ->
                    return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: UAVIONIX_ADSB_RF_HEALTH): Int {
            when (en) {
                UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING ->
                    return 0
                UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK ->
                    return 1
                UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX ->
                    return 2
                UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX ->
                    return 3

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *result from a mavlink command */
inline class MAV_RESULT(val value: Byte) {

    operator fun not() = MAV_RESULT(value.inv())
    operator fun contains(other: MAV_RESULT) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_RESULT) = MAV_RESULT(value or (other.value))
    infix fun and(other: MAV_RESULT) = MAV_RESULT(value and (other.value))
    infix fun xor(other: MAV_RESULT) = MAV_RESULT(value xor (other.value))

    companion object {
        val MAV_RESULT_ACCEPTED = MAV_RESULT(0) //Command ACCEPTED and EXECUTED
        val MAV_RESULT_TEMPORARILY_REJECTED = MAV_RESULT(1) //Command TEMPORARY REJECTED/DENIED
        val MAV_RESULT_DENIED = MAV_RESULT(2) //Command PERMANENTLY DENIED
        val MAV_RESULT_UNSUPPORTED = MAV_RESULT(3) //Command UNKNOWN/UNSUPPORTED
        val MAV_RESULT_FAILED = MAV_RESULT(4) //Command executed, but failed
        val MAV_RESULT_IN_PROGRESS = MAV_RESULT(5) //WIP: Command being executed
    }
}


/**
 *Special ACK block numbers control activation of dataflash log streaming */
inline class MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS(val value: UInt) {

    operator fun not() = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS(value.inv())
    operator fun contains(other: MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) = (value and other.value) != 0.toUInt()
    operator fun contains(other: ULong) = (value and other.toUInt()) != 0.toUInt()
    infix fun or(other: MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS(value or (other.value))
    infix fun and(other: MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS(value and (other.value))
    infix fun xor(other: MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS(value xor (other.value))

    companion object {
        val MAV_REMOTE_LOG_DATA_BLOCK_STOP = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS(2147483645U) //UAV to stop sending DataFlash blocks
        val MAV_REMOTE_LOG_DATA_BLOCK_START = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS(2147483646U) //UAV to start sending DataFlash blocks
    }
}


/**
 *Status for ADS-B transponder dynamic input */
inline class UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(val value: Byte) {

    operator fun not() = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(value.inv())
    operator fun contains(other: UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX) = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(value or (other.value))
    infix fun and(other: UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX) = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(value and (other.value))
    infix fun xor(other: UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX) = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(value xor (other.value))

    companion object {
        val UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(0)
        val UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1 = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(1)
        val UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(2)
        val UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(3)
        val UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(4)
        val UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX(5)
    }
}


inline class GOPRO_COMMAND(val value: Byte) {

    operator fun not() = GOPRO_COMMAND(value.inv())
    operator fun contains(other: GOPRO_COMMAND) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: GOPRO_COMMAND) = GOPRO_COMMAND(value or (other.value))
    infix fun and(other: GOPRO_COMMAND) = GOPRO_COMMAND(value and (other.value))
    infix fun xor(other: GOPRO_COMMAND) = GOPRO_COMMAND(value xor (other.value))

    companion object {
        val GOPRO_COMMAND_POWER = GOPRO_COMMAND(0) //(Get/Set)
        val GOPRO_COMMAND_CAPTURE_MODE = GOPRO_COMMAND(1) //(Get/Set)
        val GOPRO_COMMAND_SHUTTER = GOPRO_COMMAND(2) //(___/Set)
        val GOPRO_COMMAND_BATTERY = GOPRO_COMMAND(3) //(Get/___)
        val GOPRO_COMMAND_MODEL = GOPRO_COMMAND(4) //(Get/___)
        val GOPRO_COMMAND_VIDEO_SETTINGS = GOPRO_COMMAND(5) //(Get/Set)
        val GOPRO_COMMAND_LOW_LIGHT = GOPRO_COMMAND(6) //(Get/Set)
        val GOPRO_COMMAND_PHOTO_RESOLUTION = GOPRO_COMMAND(7) //(Get/Set)
        val GOPRO_COMMAND_PHOTO_BURST_RATE = GOPRO_COMMAND(8) //(Get/Set)
        val GOPRO_COMMAND_PROTUNE = GOPRO_COMMAND(9) //(Get/Set)
        val GOPRO_COMMAND_PROTUNE_WHITE_BALANCE = GOPRO_COMMAND(10) //(Get/Set) Hero 3+ Only
        val GOPRO_COMMAND_PROTUNE_COLOUR = GOPRO_COMMAND(11) //(Get/Set) Hero 3+ Only
        val GOPRO_COMMAND_PROTUNE_GAIN = GOPRO_COMMAND(12) //(Get/Set) Hero 3+ Only
        val GOPRO_COMMAND_PROTUNE_SHARPNESS = GOPRO_COMMAND(13) //(Get/Set) Hero 3+ Only
        val GOPRO_COMMAND_PROTUNE_EXPOSURE = GOPRO_COMMAND(14) //(Get/Set) Hero 3+ Only
        val GOPRO_COMMAND_TIME = GOPRO_COMMAND(15) //(Get/Set)
        val GOPRO_COMMAND_CHARGING = GOPRO_COMMAND(16) //(Get/Set)
    }
}


inline class GOPRO_REQUEST_STATUS(val value: Byte) {

    operator fun not() = GOPRO_REQUEST_STATUS(value.inv())
    operator fun contains(other: GOPRO_REQUEST_STATUS) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: GOPRO_REQUEST_STATUS) = GOPRO_REQUEST_STATUS(value or (other.value))
    infix fun and(other: GOPRO_REQUEST_STATUS) = GOPRO_REQUEST_STATUS(value and (other.value))
    infix fun xor(other: GOPRO_REQUEST_STATUS) = GOPRO_REQUEST_STATUS(value xor (other.value))

    companion object {
        val GOPRO_REQUEST_SUCCESS = GOPRO_REQUEST_STATUS(0) //The write message with ID indicated succeeded
        val GOPRO_REQUEST_FAILED = GOPRO_REQUEST_STATUS(1) //The write message with ID indicated failed
    }
}


/**
 *Definitions for aircraft size */
inline class UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(val value: Byte) {

    operator fun not() = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(value.inv())
    operator fun contains(other: UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE) = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(value or (other.value))
    infix fun and(other: UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE) = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(value and (other.value))
    infix fun xor(other: UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE) = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(value xor (other.value))

    companion object {
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(0)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(1)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(2)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(3)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_33M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(4)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(5)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(6)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(7)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(8)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(9)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(10)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(11)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(12)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W80M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(13)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(14)
        val UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE(15)
    }
}


inline class MAV_STATE(val value: Byte) {

    operator fun not() = MAV_STATE(value.inv())
    operator fun contains(other: MAV_STATE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_STATE) = MAV_STATE(value or (other.value))
    infix fun and(other: MAV_STATE) = MAV_STATE(value and (other.value))
    infix fun xor(other: MAV_STATE) = MAV_STATE(value xor (other.value))

    companion object {
        val UNINIT = MAV_STATE(0) //Uninitialized system, state is unknown.
        val ACTIVE = MAV_STATE(1) //System is active and might be already airborne. Motors are engaged.
        val BOOT = MAV_STATE(2) //System is booting up.
        val CALIBRATING = MAV_STATE(3) //System is calibrating and not flight-ready.
        val CRITICAL = MAV_STATE(4) //System is in a non-normal flight mode. It can however still navigate.

        /**
         *System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
         *			 mayday and going down */
        val EMERGENCY = MAV_STATE(5)
        val FLIGHT_TERMINATION = MAV_STATE(6) //System is terminating itself.
        val POWEROFF = MAV_STATE(7) //System just initialized its power-down sequence, will shut down now.
        val STANDBY = MAV_STATE(8) //System is grounded and on standby. It can be launched any time.
    }
}


/**
 *SERIAL_CONTROL flags (bitmask) */
inline class SERIAL_CONTROL_FLAG(val value: Int) {

    operator fun not() = SERIAL_CONTROL_FLAG(value.inv())
    operator fun contains(other: SERIAL_CONTROL_FLAG) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: SERIAL_CONTROL_FLAG) = SERIAL_CONTROL_FLAG(value or (other.value))
    infix fun and(other: SERIAL_CONTROL_FLAG) = SERIAL_CONTROL_FLAG(value and (other.value))
    infix fun xor(other: SERIAL_CONTROL_FLAG) = SERIAL_CONTROL_FLAG(value xor (other.value))

    companion object {
        val SERIAL_CONTROL_FLAG_REPLY = SERIAL_CONTROL_FLAG(1) //Set if this is a reply
        val SERIAL_CONTROL_FLAG_RESPOND = SERIAL_CONTROL_FLAG(2) //Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message

        /**
         *Set if access to the serial port should be removed from whatever driver is currently using it, giving
         *			 exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without
         *			 this flag se */
        val SERIAL_CONTROL_FLAG_EXCLUSIVE = SERIAL_CONTROL_FLAG(4)
        val SERIAL_CONTROL_FLAG_BLOCKING = SERIAL_CONTROL_FLAG(8) //Block on writes to the serial port
        val SERIAL_CONTROL_FLAG_MULTI = SERIAL_CONTROL_FLAG(16) //Send multiple replies until port is drained

        inline fun get(id: Int): SERIAL_CONTROL_FLAG {
            when (id) {
                0 ->
                    return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY
                1 ->
                    return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND
                2 ->
                    return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE
                3 ->
                    return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING
                4 ->
                    return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: SERIAL_CONTROL_FLAG): Int {
            when (en) {
                SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY ->
                    return 0
                SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND ->
                    return 1
                SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE ->
                    return 2
                SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING ->
                    return 3
                SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI ->
                    return 4

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Camera capability flags (Bitmap). */
inline class CAMERA_CAP_FLAGS(val value: Int) {

    operator fun not() = CAMERA_CAP_FLAGS(value.inv())
    operator fun contains(other: CAMERA_CAP_FLAGS) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: CAMERA_CAP_FLAGS) = CAMERA_CAP_FLAGS(value or (other.value))
    infix fun and(other: CAMERA_CAP_FLAGS) = CAMERA_CAP_FLAGS(value and (other.value))
    infix fun xor(other: CAMERA_CAP_FLAGS) = CAMERA_CAP_FLAGS(value xor (other.value))

    companion object {
        val CAMERA_CAP_FLAGS_CAPTURE_VIDEO = CAMERA_CAP_FLAGS(1) //Camera is able to record video.
        val CAMERA_CAP_FLAGS_CAPTURE_IMAGE = CAMERA_CAP_FLAGS(2) //Camera is able to capture images.
        val CAMERA_CAP_FLAGS_HAS_MODES = CAMERA_CAP_FLAGS(4) //Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
        val CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = CAMERA_CAP_FLAGS(8) //Camera can capture images while in video mode
        val CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = CAMERA_CAP_FLAGS(16) //Camera can capture videos while in Photo/Image mode
        val CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = CAMERA_CAP_FLAGS(32) //Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)

        inline fun get(id: Int): CAMERA_CAP_FLAGS {
            when (id) {
                0 ->
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO
                1 ->
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE
                2 ->
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES
                3 ->
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE
                4 ->
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE
                5 ->
                    return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: CAMERA_CAP_FLAGS): Int {
            when (en) {
                CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO ->
                    return 0
                CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE ->
                    return 1
                CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES ->
                    return 2
                CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE ->
                    return 3
                CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE ->
                    return 4
                CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE ->
                    return 5

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


inline class GOPRO_HEARTBEAT_FLAGS(val value: Byte) {

    operator fun not() = GOPRO_HEARTBEAT_FLAGS(value.inv())
    operator fun contains(other: GOPRO_HEARTBEAT_FLAGS) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: GOPRO_HEARTBEAT_FLAGS) = GOPRO_HEARTBEAT_FLAGS(value or (other.value))
    infix fun and(other: GOPRO_HEARTBEAT_FLAGS) = GOPRO_HEARTBEAT_FLAGS(value and (other.value))
    infix fun xor(other: GOPRO_HEARTBEAT_FLAGS) = GOPRO_HEARTBEAT_FLAGS(value xor (other.value))

    companion object {
        val GOPRO_FLAG_RECORDING = GOPRO_HEARTBEAT_FLAGS(1) //GoPro is currently recording
    }
}


/**
 *Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability */
inline class MAV_PROTOCOL_CAPABILITY(val value: Int) {

    operator fun not() = MAV_PROTOCOL_CAPABILITY(value.inv())
    operator fun contains(other: MAV_PROTOCOL_CAPABILITY) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: MAV_PROTOCOL_CAPABILITY) = MAV_PROTOCOL_CAPABILITY(value or (other.value))
    infix fun and(other: MAV_PROTOCOL_CAPABILITY) = MAV_PROTOCOL_CAPABILITY(value and (other.value))
    infix fun xor(other: MAV_PROTOCOL_CAPABILITY) = MAV_PROTOCOL_CAPABILITY(value xor (other.value))

    companion object {
        val MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = MAV_PROTOCOL_CAPABILITY(1) //Autopilot supports MISSION float message type.
        val MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = MAV_PROTOCOL_CAPABILITY(2) //Autopilot supports the new param float message type.
        val MAV_PROTOCOL_CAPABILITY_MISSION_INT = MAV_PROTOCOL_CAPABILITY(4) //Autopilot supports MISSION_INT scaled integer message type.
        val MAV_PROTOCOL_CAPABILITY_COMMAND_INT = MAV_PROTOCOL_CAPABILITY(8) //Autopilot supports COMMAND_INT scaled integer message type.
        val MAV_PROTOCOL_CAPABILITY_PARAM_UNION = MAV_PROTOCOL_CAPABILITY(16) //Autopilot supports the new param union message type.
        val MAV_PROTOCOL_CAPABILITY_FTP = MAV_PROTOCOL_CAPABILITY(32) //Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
        val MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = MAV_PROTOCOL_CAPABILITY(64) //Autopilot supports commanding attitude offboard.
        val MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = MAV_PROTOCOL_CAPABILITY(128) //Autopilot supports commanding position and velocity targets in local NED frame.
        val MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = MAV_PROTOCOL_CAPABILITY(256) //Autopilot supports commanding position and velocity targets in global scaled integers.
        val MAV_PROTOCOL_CAPABILITY_TERRAIN = MAV_PROTOCOL_CAPABILITY(512) //Autopilot supports terrain protocol / data handling.
        val MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = MAV_PROTOCOL_CAPABILITY(1024) //Autopilot supports direct actuator control.
        val MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = MAV_PROTOCOL_CAPABILITY(2048) //Autopilot supports the flight termination command.
        val MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = MAV_PROTOCOL_CAPABILITY(4096) //Autopilot supports onboard compass calibration.
        val MAV_PROTOCOL_CAPABILITY_MAVLINK2 = MAV_PROTOCOL_CAPABILITY(8192) //Autopilot supports mavlink version 2.
        val MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = MAV_PROTOCOL_CAPABILITY(16384) //Autopilot supports mission fence protocol.
        val MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = MAV_PROTOCOL_CAPABILITY(32768) //Autopilot supports mission rally point protocol.
        val MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION = MAV_PROTOCOL_CAPABILITY(65536) //Autopilot supports the flight information protocol.

        inline fun get(id: Int): MAV_PROTOCOL_CAPABILITY {
            when (id) {
                0 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
                1 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
                2 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT
                3 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT
                4 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION
                5 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP
                6 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
                7 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED
                8 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
                9 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN
                10 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET
                11 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION
                12 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION
                13 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2
                14 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE
                15 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY
                16 ->
                    return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: MAV_PROTOCOL_CAPABILITY): Int {
            when (en) {
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT ->
                    return 0
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT ->
                    return 1
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT ->
                    return 2
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT ->
                    return 3
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION ->
                    return 4
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP ->
                    return 5
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET ->
                    return 6
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED ->
                    return 7
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT ->
                    return 8
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN ->
                    return 9
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET ->
                    return 10
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION ->
                    return 11
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION ->
                    return 12
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 ->
                    return 13
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE ->
                    return 14
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY ->
                    return 15
                MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION ->
                    return 16

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


inline class GOPRO_HEARTBEAT_STATUS(val value: Byte) {

    operator fun not() = GOPRO_HEARTBEAT_STATUS(value.inv())
    operator fun contains(other: GOPRO_HEARTBEAT_STATUS) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: GOPRO_HEARTBEAT_STATUS) = GOPRO_HEARTBEAT_STATUS(value or (other.value))
    infix fun and(other: GOPRO_HEARTBEAT_STATUS) = GOPRO_HEARTBEAT_STATUS(value and (other.value))
    infix fun xor(other: GOPRO_HEARTBEAT_STATUS) = GOPRO_HEARTBEAT_STATUS(value xor (other.value))

    companion object {
        val GOPRO_HEARTBEAT_STATUS_DISCONNECTED = GOPRO_HEARTBEAT_STATUS(0) //No GoPro connected
        val GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE = GOPRO_HEARTBEAT_STATUS(1) //The detected GoPro is not HeroBus compatible
        val GOPRO_HEARTBEAT_STATUS_CONNECTED = GOPRO_HEARTBEAT_STATUS(2) //A HeroBus compatible GoPro is connected
        val GOPRO_HEARTBEAT_STATUS_ERROR = GOPRO_HEARTBEAT_STATUS(3) //An unrecoverable error was encountered with the connected GoPro, it may require a power cycle
    }
}


inline class MAG_CAL_STATUS(val value: Byte) {

    operator fun not() = MAG_CAL_STATUS(value.inv())
    operator fun contains(other: MAG_CAL_STATUS) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAG_CAL_STATUS) = MAG_CAL_STATUS(value or (other.value))
    infix fun and(other: MAG_CAL_STATUS) = MAG_CAL_STATUS(value and (other.value))
    infix fun xor(other: MAG_CAL_STATUS) = MAG_CAL_STATUS(value xor (other.value))

    companion object {
        val MAG_CAL_NOT_STARTED = MAG_CAL_STATUS(0)
        val MAG_CAL_WAITING_TO_START = MAG_CAL_STATUS(1)
        val MAG_CAL_RUNNING_STEP_ONE = MAG_CAL_STATUS(2)
        val MAG_CAL_RUNNING_STEP_TWO = MAG_CAL_STATUS(3)
        val MAG_CAL_SUCCESS = MAG_CAL_STATUS(4)
        val MAG_CAL_FAILED = MAG_CAL_STATUS(5)
    }
}


inline class CAMERA_STATUS_TYPES(val value: Byte) {

    operator fun not() = CAMERA_STATUS_TYPES(value.inv())
    operator fun contains(other: CAMERA_STATUS_TYPES) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: CAMERA_STATUS_TYPES) = CAMERA_STATUS_TYPES(value or (other.value))
    infix fun and(other: CAMERA_STATUS_TYPES) = CAMERA_STATUS_TYPES(value and (other.value))
    infix fun xor(other: CAMERA_STATUS_TYPES) = CAMERA_STATUS_TYPES(value xor (other.value))

    companion object {
        val CAMERA_STATUS_TYPE_HEARTBEAT = CAMERA_STATUS_TYPES(0) //Camera heartbeat, announce camera component ID at 1hz
        val CAMERA_STATUS_TYPE_TRIGGER = CAMERA_STATUS_TYPES(1) //Camera image triggered
        val CAMERA_STATUS_TYPE_DISCONNECT = CAMERA_STATUS_TYPES(2) //Camera connection lost
        val CAMERA_STATUS_TYPE_ERROR = CAMERA_STATUS_TYPES(3) //Camera unknown error
        val CAMERA_STATUS_TYPE_LOWBATT = CAMERA_STATUS_TYPES(4) //Camera battery low. Parameter p1 shows reported voltage
        val CAMERA_STATUS_TYPE_LOWSTORE = CAMERA_STATUS_TYPES(5) //Camera storage low. Parameter p1 shows reported shots remaining
        val CAMERA_STATUS_TYPE_LOWSTOREV = CAMERA_STATUS_TYPES(6) //Camera storage low. Parameter p1 shows reported video minutes remaining
    }
}


/**
 *SERIAL_CONTROL device types */
inline class SERIAL_CONTROL_DEV(val value: Int) {

    operator fun not() = SERIAL_CONTROL_DEV(value.inv())
    operator fun contains(other: SERIAL_CONTROL_DEV) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: SERIAL_CONTROL_DEV) = SERIAL_CONTROL_DEV(value or (other.value))
    infix fun and(other: SERIAL_CONTROL_DEV) = SERIAL_CONTROL_DEV(value and (other.value))
    infix fun xor(other: SERIAL_CONTROL_DEV) = SERIAL_CONTROL_DEV(value xor (other.value))

    companion object {
        val SERIAL_CONTROL_DEV_TELEM1 = SERIAL_CONTROL_DEV(0) //First telemetry port
        val SERIAL_CONTROL_DEV_TELEM2 = SERIAL_CONTROL_DEV(1) //Second telemetry port
        val SERIAL_CONTROL_DEV_GPS1 = SERIAL_CONTROL_DEV(2) //First GPS port
        val SERIAL_CONTROL_DEV_GPS2 = SERIAL_CONTROL_DEV(3) //Second GPS port
        val SERIAL_CONTROL_DEV_SHELL = SERIAL_CONTROL_DEV(10) //system shell

        inline fun get(id: Int): SERIAL_CONTROL_DEV {
            when (id) {
                0 ->
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1
                1 ->
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2
                2 ->
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1
                3 ->
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2
                4 ->
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: SERIAL_CONTROL_DEV): Int {
            when (en) {
                SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1 ->
                    return 0
                SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2 ->
                    return 1
                SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1 ->
                    return 2
                SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2 ->
                    return 3
                SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL ->
                    return 4

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *result in a mavlink mission ack */
inline class MAV_MISSION_RESULT(val value: Byte) {

    operator fun not() = MAV_MISSION_RESULT(value.inv())
    operator fun contains(other: MAV_MISSION_RESULT) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_MISSION_RESULT) = MAV_MISSION_RESULT(value or (other.value))
    infix fun and(other: MAV_MISSION_RESULT) = MAV_MISSION_RESULT(value and (other.value))
    infix fun xor(other: MAV_MISSION_RESULT) = MAV_MISSION_RESULT(value xor (other.value))

    companion object {
        val MAV_MISSION_ACCEPTED = MAV_MISSION_RESULT(0) //mission accepted OK
        val MAV_MISSION_ERROR = MAV_MISSION_RESULT(1) //generic error / not accepting mission commands at all right now
        val MAV_MISSION_UNSUPPORTED_FRAME = MAV_MISSION_RESULT(2) //coordinate frame is not supported
        val MAV_MISSION_UNSUPPORTED = MAV_MISSION_RESULT(3) //command is not supported
        val MAV_MISSION_NO_SPACE = MAV_MISSION_RESULT(4) //mission item exceeds storage space
        val MAV_MISSION_INVALID = MAV_MISSION_RESULT(5) //one of the parameters has an invalid value
        val MAV_MISSION_INVALID_PARAM1 = MAV_MISSION_RESULT(6) //param1 has an invalid value
        val MAV_MISSION_INVALID_PARAM2 = MAV_MISSION_RESULT(7) //param2 has an invalid value
        val MAV_MISSION_INVALID_PARAM3 = MAV_MISSION_RESULT(8) //param3 has an invalid value
        val MAV_MISSION_INVALID_PARAM4 = MAV_MISSION_RESULT(9) //param4 has an invalid value
        val MAV_MISSION_INVALID_PARAM5_X = MAV_MISSION_RESULT(10) //x/param5 has an invalid value
        val MAV_MISSION_INVALID_PARAM6_Y = MAV_MISSION_RESULT(11) //y/param6 has an invalid value
        val MAV_MISSION_INVALID_PARAM7 = MAV_MISSION_RESULT(12) //param7 has an invalid value
        val MAV_MISSION_INVALID_SEQUENCE = MAV_MISSION_RESULT(13) //received waypoint out of sequence
        val MAV_MISSION_DENIED = MAV_MISSION_RESULT(14) //not accepting any mission commands from this communication partner
    }
}


/**
 *Power supply status flags (bitmask) */
inline class MAV_POWER_STATUS(val value: Int) {

    operator fun not() = MAV_POWER_STATUS(value.inv())
    operator fun contains(other: MAV_POWER_STATUS) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: MAV_POWER_STATUS) = MAV_POWER_STATUS(value or (other.value))
    infix fun and(other: MAV_POWER_STATUS) = MAV_POWER_STATUS(value and (other.value))
    infix fun xor(other: MAV_POWER_STATUS) = MAV_POWER_STATUS(value xor (other.value))

    companion object {
        val MAV_POWER_STATUS_BRICK_VALID = MAV_POWER_STATUS(1) //main brick power supply valid
        val MAV_POWER_STATUS_SERVO_VALID = MAV_POWER_STATUS(2) //main servo power supply valid for FMU
        val MAV_POWER_STATUS_USB_CONNECTED = MAV_POWER_STATUS(4) //USB power is connected
        val MAV_POWER_STATUS_PERIPH_OVERCURRENT = MAV_POWER_STATUS(8) //peripheral supply is in over-current state
        val MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = MAV_POWER_STATUS(16) //hi-power peripheral supply is in over-current state
        val MAV_POWER_STATUS_CHANGED = MAV_POWER_STATUS(32) //Power status has changed since boot

        inline fun get(id: Int): MAV_POWER_STATUS {
            when (id) {
                0 ->
                    return MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID
                1 ->
                    return MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID
                2 ->
                    return MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED
                3 ->
                    return MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT
                4 ->
                    return MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT
                5 ->
                    return MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: MAV_POWER_STATUS): Int {
            when (en) {
                MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID ->
                    return 0
                MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID ->
                    return 1
                MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED ->
                    return 2
                MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT ->
                    return 3
                MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT ->
                    return 4
                MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED ->
                    return 5

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Generalized UAVCAN node mode */
inline class UAVCAN_NODE_MODE(val value: Int) {

    operator fun not() = UAVCAN_NODE_MODE(value.inv())
    operator fun contains(other: UAVCAN_NODE_MODE) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: UAVCAN_NODE_MODE) = UAVCAN_NODE_MODE(value or (other.value))
    infix fun and(other: UAVCAN_NODE_MODE) = UAVCAN_NODE_MODE(value and (other.value))
    infix fun xor(other: UAVCAN_NODE_MODE) = UAVCAN_NODE_MODE(value xor (other.value))

    companion object {
        val UAVCAN_NODE_MODE_OPERATIONAL = UAVCAN_NODE_MODE(0) //The node is performing its primary functions.
        val UAVCAN_NODE_MODE_INITIALIZATION = UAVCAN_NODE_MODE(1) //The node is initializing; this mode is entered immediately after startup.
        val UAVCAN_NODE_MODE_MAINTENANCE = UAVCAN_NODE_MODE(2) //The node is under maintenance.
        val UAVCAN_NODE_MODE_SOFTWARE_UPDATE = UAVCAN_NODE_MODE(3) //The node is in the process of updating its software.
        val UAVCAN_NODE_MODE_OFFLINE = UAVCAN_NODE_MODE(7) //The node is no longer available online.

        inline fun get(id: Int): UAVCAN_NODE_MODE {
            when (id) {
                0 ->
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL
                1 ->
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION
                2 ->
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE
                3 ->
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE
                4 ->
                    return UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: UAVCAN_NODE_MODE): Int {
            when (en) {
                UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL ->
                    return 0
                UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION ->
                    return 1
                UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE ->
                    return 2
                UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE ->
                    return 3
                UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE ->
                    return 4

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *These flags indicate status such as data validity of each data source. Set = data valid */
inline class ADSB_FLAGS(val value: Int) {

    operator fun not() = ADSB_FLAGS(value.inv())
    operator fun contains(other: ADSB_FLAGS) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: ADSB_FLAGS) = ADSB_FLAGS(value or (other.value))
    infix fun and(other: ADSB_FLAGS) = ADSB_FLAGS(value and (other.value))
    infix fun xor(other: ADSB_FLAGS) = ADSB_FLAGS(value xor (other.value))

    companion object {
        val ADSB_FLAGS_VALID_COORDS = ADSB_FLAGS(1)
        val ADSB_FLAGS_VALID_ALTITUDE = ADSB_FLAGS(2)
        val ADSB_FLAGS_VALID_HEADING = ADSB_FLAGS(4)
        val ADSB_FLAGS_VALID_VELOCITY = ADSB_FLAGS(8)
        val ADSB_FLAGS_VALID_CALLSIGN = ADSB_FLAGS(16)
        val ADSB_FLAGS_VALID_SQUAWK = ADSB_FLAGS(32)
        val ADSB_FLAGS_SIMULATED = ADSB_FLAGS(64)

        inline fun get(id: Int): ADSB_FLAGS {
            when (id) {
                0 ->
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS
                1 ->
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE
                2 ->
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING
                3 ->
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY
                4 ->
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN
                5 ->
                    return ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK
                6 ->
                    return ADSB_FLAGS.ADSB_FLAGS_SIMULATED

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: ADSB_FLAGS): Int {
            when (en) {
                ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS ->
                    return 0
                ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE ->
                    return 1
                ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING ->
                    return 2
                ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY ->
                    return 3
                ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN ->
                    return 4
                ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK ->
                    return 5
                ADSB_FLAGS.ADSB_FLAGS_SIMULATED ->
                    return 6

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Result from a PARAM_EXT_SET message. */
inline class PARAM_ACK(val value: Byte) {

    operator fun not() = PARAM_ACK(value.inv())
    operator fun contains(other: PARAM_ACK) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: PARAM_ACK) = PARAM_ACK(value or (other.value))
    infix fun and(other: PARAM_ACK) = PARAM_ACK(value and (other.value))
    infix fun xor(other: PARAM_ACK) = PARAM_ACK(value xor (other.value))

    companion object {
        val PARAM_ACK_ACCEPTED = PARAM_ACK(0) //Parameter value ACCEPTED and SET
        val PARAM_ACK_VALUE_UNSUPPORTED = PARAM_ACK(1) //Parameter value UNKNOWN/UNSUPPORTED
        val PARAM_ACK_FAILED = PARAM_ACK(2) //Parameter failed to set

        /**
         *Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation
         *			 is completed with the actual result. These are for parameters that may take longer to set. Instead of
         *			 waiting for an ACK and potentially timing out, you will immediately receive this response to let you
         *			 know it was received */
        val PARAM_ACK_IN_PROGRESS = PARAM_ACK(3)
    }
}


/**
 *Enumeration of possible mount operation modes */
inline class MAV_MOUNT_MODE(val value: Byte) {

    operator fun not() = MAV_MOUNT_MODE(value.inv())
    operator fun contains(other: MAV_MOUNT_MODE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_MOUNT_MODE) = MAV_MOUNT_MODE(value or (other.value))
    infix fun and(other: MAV_MOUNT_MODE) = MAV_MOUNT_MODE(value and (other.value))
    infix fun xor(other: MAV_MOUNT_MODE) = MAV_MOUNT_MODE(value xor (other.value))

    companion object {
        val MAV_MOUNT_MODE_RETRACT = MAV_MOUNT_MODE(0) //Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
        val MAV_MOUNT_MODE_NEUTRAL = MAV_MOUNT_MODE(1) //Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
        val MAV_MOUNT_MODE_MAVLINK_TARGETING = MAV_MOUNT_MODE(2) //Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
        val MAV_MOUNT_MODE_RC_TARGETING = MAV_MOUNT_MODE(3) //Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
        val MAV_MOUNT_MODE_GPS_POINT = MAV_MOUNT_MODE(4) //Load neutral position and start to point to Lat,Lon,Alt
    }
}


/**
 *Enumeration of sensor orientation, according to its rotations */
inline class MAV_SENSOR_ORIENTATION(val value: Byte) {

    operator fun not() = MAV_SENSOR_ORIENTATION(value.inv())
    operator fun contains(other: MAV_SENSOR_ORIENTATION) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_SENSOR_ORIENTATION) = MAV_SENSOR_ORIENTATION(value or (other.value))
    infix fun and(other: MAV_SENSOR_ORIENTATION) = MAV_SENSOR_ORIENTATION(value and (other.value))
    infix fun xor(other: MAV_SENSOR_ORIENTATION) = MAV_SENSOR_ORIENTATION(value xor (other.value))

    companion object {
        val NONE = MAV_SENSOR_ORIENTATION(0) //Roll: 0, Pitch: 0, Yaw: 0
        val YAW_45 = MAV_SENSOR_ORIENTATION(1) //Roll: 0, Pitch: 0, Yaw: 45
        val YAW_90 = MAV_SENSOR_ORIENTATION(2) //Roll: 0, Pitch: 0, Yaw: 90
        val YAW_135 = MAV_SENSOR_ORIENTATION(3) //Roll: 0, Pitch: 0, Yaw: 135
        val YAW_180 = MAV_SENSOR_ORIENTATION(4) //Roll: 0, Pitch: 0, Yaw: 180
        val YAW_225 = MAV_SENSOR_ORIENTATION(5) //Roll: 0, Pitch: 0, Yaw: 225
        val YAW_270 = MAV_SENSOR_ORIENTATION(6) //Roll: 0, Pitch: 0, Yaw: 270
        val YAW_315 = MAV_SENSOR_ORIENTATION(7) //Roll: 0, Pitch: 0, Yaw: 315
        val ROLL_180 = MAV_SENSOR_ORIENTATION(8) //Roll: 180, Pitch: 0, Yaw: 0
        val ROLL_180_YAW_45 = MAV_SENSOR_ORIENTATION(9) //Roll: 180, Pitch: 0, Yaw: 45
        val ROLL_180_YAW_90 = MAV_SENSOR_ORIENTATION(10) //Roll: 180, Pitch: 0, Yaw: 90
        val ROLL_180_YAW_135 = MAV_SENSOR_ORIENTATION(11) //Roll: 180, Pitch: 0, Yaw: 135
        val PITCH_180 = MAV_SENSOR_ORIENTATION(12) //Roll: 0, Pitch: 180, Yaw: 0
        val ROLL_180_YAW_225 = MAV_SENSOR_ORIENTATION(13) //Roll: 180, Pitch: 0, Yaw: 225
        val ROLL_180_YAW_270 = MAV_SENSOR_ORIENTATION(14) //Roll: 180, Pitch: 0, Yaw: 270
        val ROLL_180_YAW_315 = MAV_SENSOR_ORIENTATION(15) //Roll: 180, Pitch: 0, Yaw: 315
        val ROLL_90 = MAV_SENSOR_ORIENTATION(16) //Roll: 90, Pitch: 0, Yaw: 0
        val ROLL_90_YAW_45 = MAV_SENSOR_ORIENTATION(17) //Roll: 90, Pitch: 0, Yaw: 45
        val ROLL_90_YAW_90 = MAV_SENSOR_ORIENTATION(18) //Roll: 90, Pitch: 0, Yaw: 90
        val ROLL_90_YAW_135 = MAV_SENSOR_ORIENTATION(19) //Roll: 90, Pitch: 0, Yaw: 135
        val ROLL_270 = MAV_SENSOR_ORIENTATION(20) //Roll: 270, Pitch: 0, Yaw: 0
        val ROLL_270_YAW_45 = MAV_SENSOR_ORIENTATION(21) //Roll: 270, Pitch: 0, Yaw: 45
        val ROLL_270_YAW_90 = MAV_SENSOR_ORIENTATION(22) //Roll: 270, Pitch: 0, Yaw: 90
        val ROLL_270_YAW_135 = MAV_SENSOR_ORIENTATION(23) //Roll: 270, Pitch: 0, Yaw: 135
        val PITCH_90 = MAV_SENSOR_ORIENTATION(24) //Roll: 0, Pitch: 90, Yaw: 0
        val PITCH_270 = MAV_SENSOR_ORIENTATION(25) //Roll: 0, Pitch: 270, Yaw: 0
        val PITCH_180_YAW_90 = MAV_SENSOR_ORIENTATION(26) //Roll: 0, Pitch: 180, Yaw: 90
        val PITCH_180_YAW_270 = MAV_SENSOR_ORIENTATION(27) //Roll: 0, Pitch: 180, Yaw: 270
        val ROLL_90_PITCH_90 = MAV_SENSOR_ORIENTATION(28) //Roll: 90, Pitch: 90, Yaw: 0
        val ROLL_180_PITCH_90 = MAV_SENSOR_ORIENTATION(29) //Roll: 180, Pitch: 90, Yaw: 0
        val ROLL_270_PITCH_90 = MAV_SENSOR_ORIENTATION(30) //Roll: 270, Pitch: 90, Yaw: 0
        val ROLL_90_PITCH_180 = MAV_SENSOR_ORIENTATION(31) //Roll: 90, Pitch: 180, Yaw: 0
        val ROLL_270_PITCH_180 = MAV_SENSOR_ORIENTATION(32) //Roll: 270, Pitch: 180, Yaw: 0
        val ROLL_90_PITCH_270 = MAV_SENSOR_ORIENTATION(33) //Roll: 90, Pitch: 270, Yaw: 0
        val ROLL_180_PITCH_270 = MAV_SENSOR_ORIENTATION(34) //Roll: 180, Pitch: 270, Yaw: 0
        val ROLL_270_PITCH_270 = MAV_SENSOR_ORIENTATION(35) //Roll: 270, Pitch: 270, Yaw: 0
        val ROLL_90_PITCH_180_YAW_90 = MAV_SENSOR_ORIENTATION(36) //Roll: 90, Pitch: 180, Yaw: 90
        val ROLL_90_YAW_270 = MAV_SENSOR_ORIENTATION(37) //Roll: 90, Pitch: 0, Yaw: 270
        val ROLL_315_PITCH_315_YAW_315 = MAV_SENSOR_ORIENTATION(38) //Roll: 315, Pitch: 315, Yaw: 315
    }
}


/**
 *State flags for ADS-B transponder dynamic report */
inline class UAVIONIX_ADSB_OUT_DYNAMIC_STATE(val value: Int) {

    operator fun not() = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(value.inv())
    operator fun contains(other: UAVIONIX_ADSB_OUT_DYNAMIC_STATE) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: UAVIONIX_ADSB_OUT_DYNAMIC_STATE) = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(value or (other.value))
    infix fun and(other: UAVIONIX_ADSB_OUT_DYNAMIC_STATE) = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(value and (other.value))
    infix fun xor(other: UAVIONIX_ADSB_OUT_DYNAMIC_STATE) = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(value xor (other.value))

    companion object {
        val UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(1)
        val UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(2)
        val UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(4)
        val UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(8)
        val UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT = UAVIONIX_ADSB_OUT_DYNAMIC_STATE(16)

        inline fun get(id: Int): UAVIONIX_ADSB_OUT_DYNAMIC_STATE {
            when (id) {
                0 ->
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE
                1 ->
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED
                2 ->
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED
                3 ->
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND
                4 ->
                    return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: UAVIONIX_ADSB_OUT_DYNAMIC_STATE): Int {
            when (en) {
                UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE ->
                    return 0
                UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED ->
                    return 1
                UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED ->
                    return 2
                UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND ->
                    return 3
                UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT ->
                    return 4

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Possible actions an aircraft can take to avoid a collision. */
inline class MAV_COLLISION_ACTION(val value: Byte) {

    operator fun not() = MAV_COLLISION_ACTION(value.inv())
    operator fun contains(other: MAV_COLLISION_ACTION) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_COLLISION_ACTION) = MAV_COLLISION_ACTION(value or (other.value))
    infix fun and(other: MAV_COLLISION_ACTION) = MAV_COLLISION_ACTION(value and (other.value))
    infix fun xor(other: MAV_COLLISION_ACTION) = MAV_COLLISION_ACTION(value xor (other.value))

    companion object {
        val MAV_COLLISION_ACTION_NONE = MAV_COLLISION_ACTION(0) //Ignore any potential collisions
        val MAV_COLLISION_ACTION_REPORT = MAV_COLLISION_ACTION(1) //Report potential collision
        val MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = MAV_COLLISION_ACTION(2) //Ascend or Descend to avoid threat
        val MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = MAV_COLLISION_ACTION(3) //Move horizontally to avoid threat
        val MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = MAV_COLLISION_ACTION(4) //Aircraft to move perpendicular to the collision's velocity vector
        val MAV_COLLISION_ACTION_RTL = MAV_COLLISION_ACTION(5) //Aircraft to fly directly back to its launch point
        val MAV_COLLISION_ACTION_HOVER = MAV_COLLISION_ACTION(6) //Aircraft to stop in place
    }
}


/**
 *Aircraft-rated danger from this threat. */
inline class MAV_COLLISION_THREAT_LEVEL(val value: Byte) {

    operator fun not() = MAV_COLLISION_THREAT_LEVEL(value.inv())
    operator fun contains(other: MAV_COLLISION_THREAT_LEVEL) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_COLLISION_THREAT_LEVEL) = MAV_COLLISION_THREAT_LEVEL(value or (other.value))
    infix fun and(other: MAV_COLLISION_THREAT_LEVEL) = MAV_COLLISION_THREAT_LEVEL(value and (other.value))
    infix fun xor(other: MAV_COLLISION_THREAT_LEVEL) = MAV_COLLISION_THREAT_LEVEL(value xor (other.value))

    companion object {
        val MAV_COLLISION_THREAT_LEVEL_NONE = MAV_COLLISION_THREAT_LEVEL(0) //Not a threat
        val MAV_COLLISION_THREAT_LEVEL_LOW = MAV_COLLISION_THREAT_LEVEL(1) //Craft is mildly concerned about this threat
        val MAV_COLLISION_THREAT_LEVEL_HIGH = MAV_COLLISION_THREAT_LEVEL(2) //Craft is panicing, and may take actions to avoid threat
    }
}


inline class LIMITS_STATE(val value: Byte) {

    operator fun not() = LIMITS_STATE(value.inv())
    operator fun contains(other: LIMITS_STATE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: LIMITS_STATE) = LIMITS_STATE(value or (other.value))
    infix fun and(other: LIMITS_STATE) = LIMITS_STATE(value and (other.value))
    infix fun xor(other: LIMITS_STATE) = LIMITS_STATE(value xor (other.value))

    companion object {
        val LIMITS_INIT = LIMITS_STATE(0) //pre-initialization
        val LIMITS_DISABLED = LIMITS_STATE(1) //disabled
        val LIMITS_ENABLED = LIMITS_STATE(2) //checking limits
        val LIMITS_TRIGGERED = LIMITS_STATE(3) //a limit has been breached
        val LIMITS_RECOVERING = LIMITS_STATE(4) //taking action eg. RTL
        val LIMITS_RECOVERED = LIMITS_STATE(5) //we're no longer in breach of a limit
    }
}


/**
 *Transceiver RF control flags for ADS-B transponder dynamic reports */
inline class UAVIONIX_ADSB_OUT_RF_SELECT(val value: Byte) {

    operator fun not() = UAVIONIX_ADSB_OUT_RF_SELECT(value.inv())
    operator fun contains(other: UAVIONIX_ADSB_OUT_RF_SELECT) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: UAVIONIX_ADSB_OUT_RF_SELECT) = UAVIONIX_ADSB_OUT_RF_SELECT(value or (other.value))
    infix fun and(other: UAVIONIX_ADSB_OUT_RF_SELECT) = UAVIONIX_ADSB_OUT_RF_SELECT(value and (other.value))
    infix fun xor(other: UAVIONIX_ADSB_OUT_RF_SELECT) = UAVIONIX_ADSB_OUT_RF_SELECT(value xor (other.value))

    companion object {
        val UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY = UAVIONIX_ADSB_OUT_RF_SELECT(0)
        val UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED = UAVIONIX_ADSB_OUT_RF_SELECT(1)
        val UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED = UAVIONIX_ADSB_OUT_RF_SELECT(2)
    }
}


/**
 *Enumeration of battery types */
inline class MAV_BATTERY_TYPE(val value: Byte) {

    operator fun not() = MAV_BATTERY_TYPE(value.inv())
    operator fun contains(other: MAV_BATTERY_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_BATTERY_TYPE) = MAV_BATTERY_TYPE(value or (other.value))
    infix fun and(other: MAV_BATTERY_TYPE) = MAV_BATTERY_TYPE(value and (other.value))
    infix fun xor(other: MAV_BATTERY_TYPE) = MAV_BATTERY_TYPE(value xor (other.value))

    companion object {
        val UNKNOWN = MAV_BATTERY_TYPE(0) //Not specified.
        val LIPO = MAV_BATTERY_TYPE(1) //Lithium polymer battery
        val LIFE = MAV_BATTERY_TYPE(2) //Lithium-iron-phosphate battery
        val LION = MAV_BATTERY_TYPE(3) //Lithium-ION battery
        val NIMH = MAV_BATTERY_TYPE(4) //Nickel metal hydride battery
    }
}


/**
 *Flags in EKF_STATUS message */
inline class EKF_STATUS_FLAGS(val value: Int) {

    operator fun not() = EKF_STATUS_FLAGS(value.inv())
    operator fun contains(other: EKF_STATUS_FLAGS) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: EKF_STATUS_FLAGS) = EKF_STATUS_FLAGS(value or (other.value))
    infix fun and(other: EKF_STATUS_FLAGS) = EKF_STATUS_FLAGS(value and (other.value))
    infix fun xor(other: EKF_STATUS_FLAGS) = EKF_STATUS_FLAGS(value xor (other.value))

    companion object {
        val EKF_ATTITUDE = EKF_STATUS_FLAGS(1) //set if EKF's attitude estimate is good
        val EKF_VELOCITY_HORIZ = EKF_STATUS_FLAGS(2) //set if EKF's horizontal velocity estimate is good
        val EKF_VELOCITY_VERT = EKF_STATUS_FLAGS(4) //set if EKF's vertical velocity estimate is good
        val EKF_POS_HORIZ_REL = EKF_STATUS_FLAGS(8) //set if EKF's horizontal position (relative) estimate is good
        val EKF_POS_HORIZ_ABS = EKF_STATUS_FLAGS(16) //set if EKF's horizontal position (absolute) estimate is good
        val EKF_POS_VERT_ABS = EKF_STATUS_FLAGS(32) //set if EKF's vertical position (absolute) estimate is good
        val EKF_POS_VERT_AGL = EKF_STATUS_FLAGS(64) //set if EKF's vertical position (above ground) estimate is good
        val EKF_CONST_POS_MODE = EKF_STATUS_FLAGS(128) //EKF is in constant position mode and does not know it's absolute or relative position
        val EKF_PRED_POS_HORIZ_REL = EKF_STATUS_FLAGS(256) //set if EKF's predicted horizontal position (relative) estimate is good
        val EKF_PRED_POS_HORIZ_ABS = EKF_STATUS_FLAGS(512) //set if EKF's predicted horizontal position (absolute) estimate is good

        inline fun get(id: Int): EKF_STATUS_FLAGS {
            when (id) {
                0 ->
                    return EKF_STATUS_FLAGS.EKF_ATTITUDE
                1 ->
                    return EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ
                2 ->
                    return EKF_STATUS_FLAGS.EKF_VELOCITY_VERT
                3 ->
                    return EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL
                4 ->
                    return EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS
                5 ->
                    return EKF_STATUS_FLAGS.EKF_POS_VERT_ABS
                6 ->
                    return EKF_STATUS_FLAGS.EKF_POS_VERT_AGL
                7 ->
                    return EKF_STATUS_FLAGS.EKF_CONST_POS_MODE
                8 ->
                    return EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL
                9 ->
                    return EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: EKF_STATUS_FLAGS): Int {
            when (en) {
                EKF_STATUS_FLAGS.EKF_ATTITUDE ->
                    return 0
                EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ ->
                    return 1
                EKF_STATUS_FLAGS.EKF_VELOCITY_VERT ->
                    return 2
                EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL ->
                    return 3
                EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS ->
                    return 4
                EKF_STATUS_FLAGS.EKF_POS_VERT_ABS ->
                    return 5
                EKF_STATUS_FLAGS.EKF_POS_VERT_AGL ->
                    return 6
                EKF_STATUS_FLAGS.EKF_CONST_POS_MODE ->
                    return 7
                EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL ->
                    return 8
                EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS ->
                    return 9

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *These encode the sensors whose status is sent as part of the SYS_STATUS message. */
inline class MAV_SYS_STATUS_SENSOR(val value: Int) {

    operator fun not() = MAV_SYS_STATUS_SENSOR(value.inv())
    operator fun contains(other: MAV_SYS_STATUS_SENSOR) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: MAV_SYS_STATUS_SENSOR) = MAV_SYS_STATUS_SENSOR(value or (other.value))
    infix fun and(other: MAV_SYS_STATUS_SENSOR) = MAV_SYS_STATUS_SENSOR(value and (other.value))
    infix fun xor(other: MAV_SYS_STATUS_SENSOR) = MAV_SYS_STATUS_SENSOR(value xor (other.value))

    companion object {
        val MAV_SYS_STATUS_SENSOR_3D_GYRO = MAV_SYS_STATUS_SENSOR(1) //0x01 3D gyro
        val MAV_SYS_STATUS_SENSOR_3D_ACCEL = MAV_SYS_STATUS_SENSOR(2) //0x02 3D accelerometer
        val MAV_SYS_STATUS_SENSOR_3D_MAG = MAV_SYS_STATUS_SENSOR(4) //0x04 3D magnetometer
        val MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = MAV_SYS_STATUS_SENSOR(8) //0x08 absolute pressure
        val MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = MAV_SYS_STATUS_SENSOR(16) //0x10 differential pressure
        val MAV_SYS_STATUS_SENSOR_GPS = MAV_SYS_STATUS_SENSOR(32) //0x20 GPS
        val MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = MAV_SYS_STATUS_SENSOR(64) //0x40 optical flow
        val MAV_SYS_STATUS_SENSOR_VISION_POSITION = MAV_SYS_STATUS_SENSOR(128) //0x80 computer vision position
        val MAV_SYS_STATUS_SENSOR_LASER_POSITION = MAV_SYS_STATUS_SENSOR(256) //0x100 laser based position
        val MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = MAV_SYS_STATUS_SENSOR(512) //0x200 external ground truth (Vicon or Leica)
        val MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = MAV_SYS_STATUS_SENSOR(1024) //0x400 3D angular rate control
        val MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = MAV_SYS_STATUS_SENSOR(2048) //0x800 attitude stabilization
        val MAV_SYS_STATUS_SENSOR_YAW_POSITION = MAV_SYS_STATUS_SENSOR(4096) //0x1000 yaw position
        val MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = MAV_SYS_STATUS_SENSOR(8192) //0x2000 z/altitude control
        val MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = MAV_SYS_STATUS_SENSOR(16384) //0x4000 x/y position control
        val MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = MAV_SYS_STATUS_SENSOR(32768) //0x8000 motor outputs / control
        val MAV_SYS_STATUS_SENSOR_RC_RECEIVER = MAV_SYS_STATUS_SENSOR(65536) //0x10000 rc receiver
        val MAV_SYS_STATUS_SENSOR_3D_GYRO2 = MAV_SYS_STATUS_SENSOR(131072) //0x20000 2nd 3D gyro
        val MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = MAV_SYS_STATUS_SENSOR(262144) //0x40000 2nd 3D accelerometer
        val MAV_SYS_STATUS_SENSOR_3D_MAG2 = MAV_SYS_STATUS_SENSOR(524288) //0x80000 2nd 3D magnetometer
        val MAV_SYS_STATUS_GEOFENCE = MAV_SYS_STATUS_SENSOR(1048576) //0x100000 geofence
        val MAV_SYS_STATUS_AHRS = MAV_SYS_STATUS_SENSOR(2097152) //0x200000 AHRS subsystem health
        val MAV_SYS_STATUS_TERRAIN = MAV_SYS_STATUS_SENSOR(4194304) //0x400000 Terrain subsystem health
        val MAV_SYS_STATUS_REVERSE_MOTOR = MAV_SYS_STATUS_SENSOR(8388608) //0x800000 Motors are reversed
        val MAV_SYS_STATUS_LOGGING = MAV_SYS_STATUS_SENSOR(16777216) //0x1000000 Logging
        val MAV_SYS_STATUS_SENSOR_BATTERY = MAV_SYS_STATUS_SENSOR(33554432) //0x2000000 Battery

        inline fun get(id: Int): MAV_SYS_STATUS_SENSOR {
            when (id) {
                0 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO
                1 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL
                2 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG
                3 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
                4 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
                5 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS
                6 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW
                7 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION
                8 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION
                9 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH
                10 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL
                11 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION
                12 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION
                13 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL
                14 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL
                15 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS
                16 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
                17 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2
                18 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2
                19 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2
                20 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE
                21 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS
                22 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN
                23 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR
                24 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING
                25 ->
                    return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: MAV_SYS_STATUS_SENSOR): Int {
            when (en) {
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO ->
                    return 0
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL ->
                    return 1
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG ->
                    return 2
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE ->
                    return 3
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE ->
                    return 4
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS ->
                    return 5
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW ->
                    return 6
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION ->
                    return 7
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION ->
                    return 8
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH ->
                    return 9
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL ->
                    return 10
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION ->
                    return 11
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION ->
                    return 12
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL ->
                    return 13
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL ->
                    return 14
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS ->
                    return 15
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER ->
                    return 16
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 ->
                    return 17
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 ->
                    return 18
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 ->
                    return 19
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE ->
                    return 20
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS ->
                    return 21
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN ->
                    return 22
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR ->
                    return 23
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING ->
                    return 24
                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY ->
                    return 25

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


inline class MAV_FRAME(val value: Byte) {

    operator fun not() = MAV_FRAME(value.inv())
    operator fun contains(other: MAV_FRAME) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: MAV_FRAME) = MAV_FRAME(value or (other.value))
    infix fun and(other: MAV_FRAME) = MAV_FRAME(value and (other.value))
    infix fun xor(other: MAV_FRAME) = MAV_FRAME(value xor (other.value))

    companion object {
        /**
         *Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude,
         *					 third value / z: positive altitude over mean sea level (MSL */
        val MAV_FRAME_GLOBAL = MAV_FRAME(0)
        val MAV_FRAME_LOCAL_NED = MAV_FRAME(1) //Local coordinate frame, Z-up (x: north, y: east, z: down).
        val MAV_FRAME_MISSION = MAV_FRAME(2) //NOT a coordinate frame, indicates a mission command.

        /**
         *Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
         *			 position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude
         *			 with 0 being at the altitude of the home location */
        val MAV_FRAME_GLOBAL_RELATIVE_ALT = MAV_FRAME(3)
        val MAV_FRAME_LOCAL_ENU = MAV_FRAME(4) //Local coordinate frame, Z-down (x: east, y: north, z: up)

        /**
         *Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second
         *			 value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL */
        val MAV_FRAME_GLOBAL_INT = MAV_FRAME(5)

        /**
         *Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
         *			 position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third
         *			 value / z: positive altitude with 0 being at the altitude of the home location */
        val MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = MAV_FRAME(6)

        /**
         *Offset to the current local frame. Anything expressed in this frame should be added to the current local
         *			 frame position */
        val MAV_FRAME_LOCAL_OFFSET_NED = MAV_FRAME(7)

        /**
         *Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to
         *			 command 2 m/s^2 acceleration to the right */
        val MAV_FRAME_BODY_NED = MAV_FRAME(8)

        /**
         *Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an
         *			 obstacle - e.g. useful to command 2 m/s^2 acceleration to the east */
        val MAV_FRAME_BODY_OFFSET_NED = MAV_FRAME(9)

        /**
         *Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
         *			 over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value
         *			 / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level
         *			 in terrain model */
        val MAV_FRAME_GLOBAL_TERRAIN_ALT = MAV_FRAME(10)

        /**
         *Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
         *			 over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second
         *			 value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground
         *			 level in terrain model */
        val MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = MAV_FRAME(11)
    }
}


/**
 *ADSB classification for the type of vehicle emitting the transponder signal */
inline class ADSB_EMITTER_TYPE(val value: Byte) {

    operator fun not() = ADSB_EMITTER_TYPE(value.inv())
    operator fun contains(other: ADSB_EMITTER_TYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: ADSB_EMITTER_TYPE) = ADSB_EMITTER_TYPE(value or (other.value))
    infix fun and(other: ADSB_EMITTER_TYPE) = ADSB_EMITTER_TYPE(value and (other.value))
    infix fun xor(other: ADSB_EMITTER_TYPE) = ADSB_EMITTER_TYPE(value xor (other.value))

    companion object {
        val ADSB_EMITTER_TYPE_NO_INFO = ADSB_EMITTER_TYPE(0)
        val ADSB_EMITTER_TYPE_LIGHT = ADSB_EMITTER_TYPE(1)
        val ADSB_EMITTER_TYPE_SMALL = ADSB_EMITTER_TYPE(2)
        val ADSB_EMITTER_TYPE_LARGE = ADSB_EMITTER_TYPE(3)
        val ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = ADSB_EMITTER_TYPE(4)
        val ADSB_EMITTER_TYPE_HEAVY = ADSB_EMITTER_TYPE(5)
        val ADSB_EMITTER_TYPE_HIGHLY_MANUV = ADSB_EMITTER_TYPE(6)
        val ADSB_EMITTER_TYPE_ROTOCRAFT = ADSB_EMITTER_TYPE(7)
        val ADSB_EMITTER_TYPE_UNASSIGNED = ADSB_EMITTER_TYPE(8)
        val ADSB_EMITTER_TYPE_GLIDER = ADSB_EMITTER_TYPE(9)
        val ADSB_EMITTER_TYPE_LIGHTER_AIR = ADSB_EMITTER_TYPE(10)
        val ADSB_EMITTER_TYPE_PARACHUTE = ADSB_EMITTER_TYPE(11)
        val ADSB_EMITTER_TYPE_ULTRA_LIGHT = ADSB_EMITTER_TYPE(12)
        val ADSB_EMITTER_TYPE_UNASSIGNED2 = ADSB_EMITTER_TYPE(13)
        val ADSB_EMITTER_TYPE_UAV = ADSB_EMITTER_TYPE(14)
        val ADSB_EMITTER_TYPE_SPACE = ADSB_EMITTER_TYPE(15)
        val ADSB_EMITTER_TYPE_UNASSGINED3 = ADSB_EMITTER_TYPE(16)
        val ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = ADSB_EMITTER_TYPE(17)
        val ADSB_EMITTER_TYPE_SERVICE_SURFACE = ADSB_EMITTER_TYPE(18)
        val ADSB_EMITTER_TYPE_POINT_OBSTACLE = ADSB_EMITTER_TYPE(19)
    }
}


/**
 *GPS lataral offset encoding */
inline class UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(val value: Byte) {

    operator fun not() = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(value.inv())
    operator fun contains(other: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT) = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(value or (other.value))
    infix fun and(other: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT) = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(value and (other.value))
    infix fun xor(other: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT) = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(value xor (other.value))

    companion object {
        val NO_DATA = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(0)
        val LEFT_2M = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(1)
        val LEFT_4M = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(2)
        val LEFT_6M = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(3)
        val RIGHT_0M = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(4)
        val RIGHT_2M = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(5)
        val RIGHT_4M = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(6)
        val RIGHT_6M = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT(7)
    }
}


/**
 *GPS longitudinal offset encoding */
inline class UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON(val value: Byte) {

    operator fun not() = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON(value.inv())
    operator fun contains(other: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON) = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON(value or (other.value))
    infix fun and(other: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON) = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON(value and (other.value))
    infix fun xor(other: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON) = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON(value xor (other.value))

    companion object {
        val UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON(0)
        val UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON(1)
    }
}


inline class GPS_INPUT_IGNORE_FLAGS(val value: Int) {

    operator fun not() = GPS_INPUT_IGNORE_FLAGS(value.inv())
    operator fun contains(other: GPS_INPUT_IGNORE_FLAGS) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: GPS_INPUT_IGNORE_FLAGS) = GPS_INPUT_IGNORE_FLAGS(value or (other.value))
    infix fun and(other: GPS_INPUT_IGNORE_FLAGS) = GPS_INPUT_IGNORE_FLAGS(value and (other.value))
    infix fun xor(other: GPS_INPUT_IGNORE_FLAGS) = GPS_INPUT_IGNORE_FLAGS(value xor (other.value))

    companion object {
        val GPS_INPUT_IGNORE_FLAG_ALT = GPS_INPUT_IGNORE_FLAGS(1) //ignore altitude field
        val GPS_INPUT_IGNORE_FLAG_HDOP = GPS_INPUT_IGNORE_FLAGS(2) //ignore hdop field
        val GPS_INPUT_IGNORE_FLAG_VDOP = GPS_INPUT_IGNORE_FLAGS(4) //ignore vdop field
        val GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = GPS_INPUT_IGNORE_FLAGS(8) //ignore horizontal velocity field (vn and ve)
        val GPS_INPUT_IGNORE_FLAG_VEL_VERT = GPS_INPUT_IGNORE_FLAGS(16) //ignore vertical velocity field (vd)
        val GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = GPS_INPUT_IGNORE_FLAGS(32) //ignore speed accuracy field
        val GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = GPS_INPUT_IGNORE_FLAGS(64) //ignore horizontal accuracy field
        val GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = GPS_INPUT_IGNORE_FLAGS(128) //ignore vertical accuracy field

        inline fun get(id: Int): GPS_INPUT_IGNORE_FLAGS {
            when (id) {
                0 ->
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT
                1 ->
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP
                2 ->
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP
                3 ->
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ
                4 ->
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT
                5 ->
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY
                6 ->
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY
                7 ->
                    return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: GPS_INPUT_IGNORE_FLAGS): Int {
            when (en) {
                GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT ->
                    return 0
                GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP ->
                    return 1
                GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP ->
                    return 2
                GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ ->
                    return 3
                GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT ->
                    return 4
                GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY ->
                    return 5
                GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY ->
                    return 6
                GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY ->
                    return 7

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Flags in RALLY_POINT message */
inline class RALLY_FLAGS(val value: Byte) {

    operator fun not() = RALLY_FLAGS(value.inv())
    operator fun contains(other: RALLY_FLAGS) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: RALLY_FLAGS) = RALLY_FLAGS(value or (other.value))
    infix fun and(other: RALLY_FLAGS) = RALLY_FLAGS(value and (other.value))
    infix fun xor(other: RALLY_FLAGS) = RALLY_FLAGS(value xor (other.value))

    companion object {
        val FAVORABLE_WIND = RALLY_FLAGS(1) //Flag set when requiring favorable winds for landing.

        /**
         *Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag
         *			 not set when plane is to loiter at Rally point until commanded to land */
        val LAND_IMMEDIATELY = RALLY_FLAGS(2)
    }
}


inline class FENCE_BREACH(val value: Byte) {

    operator fun not() = FENCE_BREACH(value.inv())
    operator fun contains(other: FENCE_BREACH) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: FENCE_BREACH) = FENCE_BREACH(value or (other.value))
    infix fun and(other: FENCE_BREACH) = FENCE_BREACH(value and (other.value))
    infix fun xor(other: FENCE_BREACH) = FENCE_BREACH(value xor (other.value))

    companion object {
        val FENCE_BREACH_NONE = FENCE_BREACH(0) //No last fence breach
        val FENCE_BREACH_MINALT = FENCE_BREACH(1) //Breached minimum altitude
        val FENCE_BREACH_MAXALT = FENCE_BREACH(2) //Breached maximum altitude
        val FENCE_BREACH_BOUNDARY = FENCE_BREACH(3) //Breached fence boundary
    }
}


/**
 *These flags encode the MAV mode. */
inline class MAV_MODE_FLAG(val value: Int) {

    operator fun not() = MAV_MODE_FLAG(value.inv())
    operator fun contains(other: MAV_MODE_FLAG) = (value and other.value) != 0.toInt()
    operator fun contains(other: ULong) = (value and other.toInt()) != 0.toInt()
    infix fun or(other: MAV_MODE_FLAG) = MAV_MODE_FLAG(value or (other.value))
    infix fun and(other: MAV_MODE_FLAG) = MAV_MODE_FLAG(value and (other.value))
    infix fun xor(other: MAV_MODE_FLAG) = MAV_MODE_FLAG(value xor (other.value))

    companion object {
        val MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = MAV_MODE_FLAG(1) //0b00000001 Reserved for future use.

        /**
         *0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
         *			 not be used for stable implementations */
        val MAV_MODE_FLAG_TEST_ENABLED = MAV_MODE_FLAG(2)

        /**
         *0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
         *			 depends on the actual implementation */
        val MAV_MODE_FLAG_AUTO_ENABLED = MAV_MODE_FLAG(4)
        val MAV_MODE_FLAG_GUIDED_ENABLED = MAV_MODE_FLAG(8) //0b00001000 guided mode enabled, system flies waypoints / mission items.

        /**
         *0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
         *			 control inputs to move around */
        val MAV_MODE_FLAG_STABILIZE_ENABLED = MAV_MODE_FLAG(16)

        /**
         *0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
         *			 is full operational */
        val MAV_MODE_FLAG_HIL_ENABLED = MAV_MODE_FLAG(32)
        val MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = MAV_MODE_FLAG(64) //0b01000000 remote control input is enabled.

        /**
         *0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional
         *					 note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM
         *					 shall be used instead. The flag can still be used to report the armed state */
        val MAV_MODE_FLAG_SAFETY_ARMED = MAV_MODE_FLAG(128)

        inline fun get(id: Int): MAV_MODE_FLAG {
            when (id) {
                0 ->
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                1 ->
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED
                2 ->
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED
                3 ->
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED
                4 ->
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED
                5 ->
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED
                6 ->
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                7 ->
                    return MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED

            }
            throw RuntimeException("Unknown enum ID " + id)
        }

        inline fun set(en: MAV_MODE_FLAG): Int {
            when (en) {
                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ->
                    return 0
                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED ->
                    return 1
                MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED ->
                    return 2
                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED ->
                    return 3
                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED ->
                    return 4
                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED ->
                    return 5
                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED ->
                    return 6
                MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED ->
                    return 7

                else -> assert(false)//("Unknown enum" + id)
            }
            throw RuntimeException("Unknown enum " + en)
        }
    }
}


/**
 *Bus types for device operations */
inline class DEVICE_OP_BUSTYPE(val value: Byte) {

    operator fun not() = DEVICE_OP_BUSTYPE(value.inv())
    operator fun contains(other: DEVICE_OP_BUSTYPE) = (value and other.value) != 0.toByte()
    operator fun contains(other: ULong) = (value and other.toByte()) != 0.toByte()
    infix fun or(other: DEVICE_OP_BUSTYPE) = DEVICE_OP_BUSTYPE(value or (other.value))
    infix fun and(other: DEVICE_OP_BUSTYPE) = DEVICE_OP_BUSTYPE(value and (other.value))
    infix fun xor(other: DEVICE_OP_BUSTYPE) = DEVICE_OP_BUSTYPE(value xor (other.value))

    companion object {
        val DEVICE_OP_BUSTYPE_I2C = DEVICE_OP_BUSTYPE(0) //I2C Device operation
        val DEVICE_OP_BUSTYPE_SPI = DEVICE_OP_BUSTYPE(1) //SPI Device operation
    }
}

inline class ATTITUDE_TARGET(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun type_mask(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun q(): ATTITUDE_TARGET_q {

        return ATTITUDE_TARGET_q(data)
    }

    object q {
        const val item_len = 4


    }

    inline fun body_roll_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 21, 4).toInt())
    }


    inline fun body_pitch_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 25, 4).toInt())
    }


    inline fun body_yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 29, 4).toInt())
    }


    inline fun thrust(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 33, 4).toInt())
    }


    companion object {

        val meta = Meta(106, 0, 1, 0, 37, 1, 296)


        inline fun push(src: ATTITUDE_TARGET, time_boot_ms: (src: Int) -> Unit, type_mask: (src: Byte) -> Unit, q: (src: ATTITUDE_TARGET_q) -> Unit, body_roll_rate: (src: Float) -> Unit, body_pitch_rate: (src: Float) -> Unit, body_yaw_rate: (src: Float) -> Unit, thrust: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            type_mask(src.type_mask())
            src.q().let { item ->
                q(item)
            }

            body_roll_rate(src.body_roll_rate())

            body_pitch_rate(src.body_pitch_rate())

            body_yaw_rate(src.body_yaw_rate())

            thrust(src.thrust())

        }

    }
}

inline class MISSION_COUNT(val data: Cursor) {

    inline fun count(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    inline fun mission_type(): MISSION_COUNT_mission_type? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return MISSION_COUNT_mission_type(data)
    }


    companion object {

        val meta = Meta(50, 1, 0, 0, 5, 1, 32, 0, 1)


        inline fun push(src: MISSION_COUNT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, count: (src: Short) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            count(src.count())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class ADSB_VEHICLE(val data: Cursor) {

    inline fun heading(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun heading_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun hor_velocity(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun hor_velocity_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun squawk(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun squawk_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun ICAO_address(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun ICAO_address_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 14, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun altitude(): Int {
        return (get_bytes(data.bytes, data.origin + 18, 4)).toInt()
    }

    inline fun altitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun ver_velocity(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }

    inline fun ver_velocity_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 22)
    }

    inline fun tslc(): Byte {
        return (data.bytes[data.origin + 24]).toByte()
    }

    inline fun tslc_(src: Byte) {
        data.bytes[data.origin + 24] = (src).toByte()
    }


    inline fun altitude_type(): ADSB_VEHICLE_altitude_type? {
        if ((data.field_bit != 202 && !data.set_field(202, -1))) return null

        return ADSB_VEHICLE_altitude_type(data)
    }


    inline fun altitude_type(src: ADSB_ALTITUDE_TYPE) {
        if (data.field_bit != 202) data.set_field(202, 0)

        set_bits((src.value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    inline fun callsign(): ADSB_VEHICLE_callsign? {
        if ((data.field_bit != 203 && !data.set_field(203, -1))) return null

        return ADSB_VEHICLE_callsign(data)
    }


    inline fun callsign_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -203 - 1, reuse)
    }


    inline fun callsign_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(203, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun callsign_(len: Int): ADSB_VEHICLE_callsign {

        data.set_field(203, minOf(len, 255))
        return ADSB_VEHICLE_callsign(data)
    }

    object callsign {
        const val item_len_max = 255

    }


    inline fun emitter_type(): ADSB_VEHICLE_emitter_type? {
        if ((data.field_bit != 204 && !data.set_field(204, -1))) return null

        return ADSB_VEHICLE_emitter_type(data)
    }


    inline fun emitter_type(src: ADSB_EMITTER_TYPE) {
        if (data.field_bit != 204) data.set_field(204, 0)

        set_bits((src.value).toULong().toLong(), 5, data.bytes, data.BIT)
    }


    inline fun flags(): ADSB_VEHICLE_flags? {
        if ((data.field_bit != 205 && !data.set_field(205, -1))) return null

        return ADSB_VEHICLE_flags(data)
    }


    inline fun flags(src: ADSB_FLAGS) {
        if (data.field_bit != 205) data.set_field(205, 0)

        set_bits(ADSB_FLAGS.set(src).toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(78, 3, 1, 0, 26, 1, 202, 2, 4)


        inline fun push(src: ADSB_VEHICLE, ICAO_address: (src: Int) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, altitude_type: (src: com.company.demo.GroundControl.ADSB_ALTITUDE_TYPE) -> Unit, altitude: (src: Int) -> Unit, heading: (src: Short) -> Unit, hor_velocity: (src: Short) -> Unit, ver_velocity: (src: Short) -> Unit, callsign: (src: ADSB_VEHICLE_callsign) -> Unit, emitter_type: (src: com.company.demo.GroundControl.ADSB_EMITTER_TYPE) -> Unit, tslc: (src: Byte) -> Unit, flags: (src: com.company.demo.GroundControl.ADSB_FLAGS) -> Unit, squawk: (src: Short) -> Unit
        ) {

            ICAO_address(src.ICAO_address())

            lat(src.lat())

            lon(src.lon())

            src.altitude_type()?.let { item ->
                altitude_type(item.get())
            }

            altitude(src.altitude())

            heading(src.heading())

            hor_velocity(src.hor_velocity())

            ver_velocity(src.ver_velocity())

            src.callsign()?.let { item -> callsign(item) }

            src.emitter_type()?.let { item ->
                emitter_type(item.get())
            }

            tslc(src.tslc())

            src.flags()?.let { item ->
                flags(item.get())
            }

            squawk(src.squawk())

        }

        inline fun pull(dst: ADSB_VEHICLE, ICAO_address: () -> Int, lat: () -> Int, lon: () -> Int, altitude_type_exist: () -> Boolean, altitude_type: () -> com.company.demo.GroundControl.ADSB_ALTITUDE_TYPE, altitude: () -> Int, heading: () -> Short, hor_velocity: () -> Short, ver_velocity: () -> Short, callsign_exist: () -> Int, callsign: (dst: ADSB_VEHICLE_callsign) -> Unit, emitter_type_exist: () -> Boolean, emitter_type: () -> com.company.demo.GroundControl.ADSB_EMITTER_TYPE, tslc: () -> Byte, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.ADSB_FLAGS, squawk: () -> Short
        ) {

            dst.ICAO_address(ICAO_address())

            dst.lat(lat())

            dst.lon(lon())

            if (altitude_type_exist()) dst.altitude_type(altitude_type())


            dst.altitude(altitude())

            dst.heading(heading())

            dst.hor_velocity(hor_velocity())

            dst.ver_velocity(ver_velocity())


            callsign_exist().let { len ->
                if (0 < len)
                    callsign(dst.callsign(len))
            }


            if (emitter_type_exist()) dst.emitter_type(emitter_type())


            dst.tslc(tslc())

            if (flags_exist()) dst.flags(flags())


            dst.squawk(squawk())

        }

    }
}

inline class MESSAGE_INTERVAL(val data: Pack.Bytes) {

    inline fun message_id(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun message_id_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun interval_us(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun interval_us_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }


    companion object {

        val meta = Meta(41, 1, 0, 0, 6, 1, 48)


        inline fun push(src: MESSAGE_INTERVAL, message_id: (src: Short) -> Unit, interval_us: (src: Int) -> Unit
        ) {

            message_id(src.message_id())

            interval_us(src.interval_us())

        }

        inline fun pull(dst: MESSAGE_INTERVAL, message_id: () -> Short, interval_us: () -> Int
        ) {

            dst.message_id(message_id())

            dst.interval_us(interval_us())

        }

    }
}

inline class EKF_STATUS_REPORT(val data: Cursor) {

    inline fun velocity_variance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun velocity_variance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun pos_horiz_variance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun pos_horiz_variance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun pos_vert_variance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun pos_vert_variance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun compass_variance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun compass_variance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun terrain_alt_variance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun terrain_alt_variance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }


    inline fun flags(): EKF_STATUS_REPORT_flags? {
        if ((data.field_bit != 160 && !data.set_field(160, -1))) return null

        return EKF_STATUS_REPORT_flags(data)
    }


    inline fun flags(src: EKF_STATUS_FLAGS) {
        if (data.field_bit != 160) data.set_field(160, 0)

        set_bits(EKF_STATUS_FLAGS.set(src).toLong(), 4, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(96, 0, 0, 0, 21, 1, 160, 0, 1)


        inline fun push(src: EKF_STATUS_REPORT, flags: (src: com.company.demo.GroundControl.EKF_STATUS_FLAGS) -> Unit, velocity_variance: (src: Float) -> Unit, pos_horiz_variance: (src: Float) -> Unit, pos_vert_variance: (src: Float) -> Unit, compass_variance: (src: Float) -> Unit, terrain_alt_variance: (src: Float) -> Unit
        ) {

            src.flags()?.let { item ->
                flags(item.get())
            }

            velocity_variance(src.velocity_variance())

            pos_horiz_variance(src.pos_horiz_variance())

            pos_vert_variance(src.pos_vert_variance())

            compass_variance(src.compass_variance())

            terrain_alt_variance(src.terrain_alt_variance())

        }

        inline fun pull(dst: EKF_STATUS_REPORT, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.EKF_STATUS_FLAGS, velocity_variance: () -> Float, pos_horiz_variance: () -> Float, pos_vert_variance: () -> Float, compass_variance: () -> Float, terrain_alt_variance: () -> Float
        ) {

            if (flags_exist()) dst.flags(flags())


            dst.velocity_variance(velocity_variance())

            dst.pos_horiz_variance(pos_horiz_variance())

            dst.pos_vert_variance(pos_vert_variance())

            dst.compass_variance(compass_variance())

            dst.terrain_alt_variance(terrain_alt_variance())

        }

    }
}

inline class ESTIMATOR_STATUS(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun vel_ratio(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun vel_ratio_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun pos_horiz_ratio(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun pos_horiz_ratio_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun pos_vert_ratio(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun pos_vert_ratio_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun mag_ratio(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun mag_ratio_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun hagl_ratio(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun hagl_ratio_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun tas_ratio(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun tas_ratio_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun pos_horiz_accuracy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun pos_horiz_accuracy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun pos_vert_accuracy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun pos_vert_accuracy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }


    inline fun flags(): ESTIMATOR_STATUS_flags? {
        if ((data.field_bit != 320 && !data.set_field(320, -1))) return null

        return ESTIMATOR_STATUS_flags(data)
    }


    inline fun flags(src: ESTIMATOR_STATUS_FLAGS) {
        if (data.field_bit != 320) data.set_field(320, 0)

        set_bits(ESTIMATOR_STATUS_FLAGS.set(src).toLong(), 4, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(38, 0, 0, 1, 41, 1, 320, 0, 1)


        inline fun push(src: ESTIMATOR_STATUS, time_usec: (src: Long) -> Unit, flags: (src: com.company.demo.GroundControl.ESTIMATOR_STATUS_FLAGS) -> Unit, vel_ratio: (src: Float) -> Unit, pos_horiz_ratio: (src: Float) -> Unit, pos_vert_ratio: (src: Float) -> Unit, mag_ratio: (src: Float) -> Unit, hagl_ratio: (src: Float) -> Unit, tas_ratio: (src: Float) -> Unit, pos_horiz_accuracy: (src: Float) -> Unit, pos_vert_accuracy: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            src.flags()?.let { item ->
                flags(item.get())
            }

            vel_ratio(src.vel_ratio())

            pos_horiz_ratio(src.pos_horiz_ratio())

            pos_vert_ratio(src.pos_vert_ratio())

            mag_ratio(src.mag_ratio())

            hagl_ratio(src.hagl_ratio())

            tas_ratio(src.tas_ratio())

            pos_horiz_accuracy(src.pos_horiz_accuracy())

            pos_vert_accuracy(src.pos_vert_accuracy())

        }

        inline fun pull(dst: ESTIMATOR_STATUS, time_usec: () -> Long, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.ESTIMATOR_STATUS_FLAGS, vel_ratio: () -> Float, pos_horiz_ratio: () -> Float, pos_vert_ratio: () -> Float, mag_ratio: () -> Float, hagl_ratio: () -> Float, tas_ratio: () -> Float, pos_horiz_accuracy: () -> Float, pos_vert_accuracy: () -> Float
        ) {

            dst.time_usec(time_usec())

            if (flags_exist()) dst.flags(flags())


            dst.vel_ratio(vel_ratio())

            dst.pos_horiz_ratio(pos_horiz_ratio())

            dst.pos_vert_ratio(pos_vert_ratio())

            dst.mag_ratio(mag_ratio())

            dst.hagl_ratio(hagl_ratio())

            dst.tas_ratio(tas_ratio())

            dst.pos_horiz_accuracy(pos_horiz_accuracy())

            dst.pos_vert_accuracy(pos_vert_accuracy())

        }

    }
}

inline class HWSTATUS(val data: Pack.Bytes) {

    inline fun Vcc(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun Vcc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun I2Cerr(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun I2Cerr_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }


    companion object {

        val meta = Meta(155, 1, 0, 0, 3, 1, 24)


        inline fun push(src: HWSTATUS, Vcc: (src: Short) -> Unit, I2Cerr: (src: Byte) -> Unit
        ) {

            Vcc(src.Vcc())

            I2Cerr(src.I2Cerr())

        }

        inline fun pull(dst: HWSTATUS, Vcc: () -> Short, I2Cerr: () -> Byte
        ) {

            dst.Vcc(Vcc())

            dst.I2Cerr(I2Cerr())

        }

    }
}

inline class TIMESYNC(val data: Pack.Bytes) {

    inline fun tc1(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun tc1_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun ts1(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun ts1_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }


    companion object {

        val meta = Meta(46, 0, 0, 0, 16, 1, 128)


        inline fun push(src: TIMESYNC, tc1: (src: Long) -> Unit, ts1: (src: Long) -> Unit
        ) {

            tc1(src.tc1())

            ts1(src.ts1())

        }

        inline fun pull(dst: TIMESYNC, tc1: () -> Long, ts1: () -> Long
        ) {

            dst.tc1(tc1())

            dst.ts1(ts1())

        }

    }
}

inline class PARAM_EXT_REQUEST_LIST(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    companion object {

        val meta = Meta(80, 0, 0, 0, 2, 1, 16)


        inline fun push(src: PARAM_EXT_REQUEST_LIST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

        }

        inline fun pull(dst: PARAM_EXT_REQUEST_LIST, target_system: () -> Byte, target_component: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

        }

    }
}

inline class GLOBAL_POSITION_INT_COV(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }


    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 12, 4)).toInt()
    }


    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }


    inline fun relative_alt(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }


    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }


    inline fun covariance(): GLOBAL_POSITION_INT_COV_covariance {

        return GLOBAL_POSITION_INT_COV_covariance(data)
    }

    object covariance {
        const val item_len = 36


    }


    inline fun estimator_type(): GLOBAL_POSITION_INT_COV_estimator_type? {
        if ((data.field_bit != 1440 && !data.set_field(1440, -1))) return null

        return GLOBAL_POSITION_INT_COV_estimator_type(data)
    }


    companion object {

        val meta = Meta(6, 0, 0, 1, 181, 1, 1440, 0, 1)


        inline fun push(src: GLOBAL_POSITION_INT_COV, time_usec: (src: Long) -> Unit, estimator_type: (src: com.company.demo.GroundControl.MAV_ESTIMATOR_TYPE) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Int) -> Unit, relative_alt: (src: Int) -> Unit, vx: (src: Float) -> Unit, vy: (src: Float) -> Unit, vz: (src: Float) -> Unit, covariance: (src: GLOBAL_POSITION_INT_COV_covariance) -> Unit
        ) {

            time_usec(src.time_usec())

            src.estimator_type()?.let { item ->
                estimator_type(item.get())
            }

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            relative_alt(src.relative_alt())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())
            src.covariance().let { item ->
                covariance(item)
            }

        }

    }
}

inline class BUTTON_CHANGE(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun last_change_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun last_change_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun state(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun state_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }


    companion object {

        val meta = Meta(146, 0, 2, 0, 9, 1, 72)


        inline fun push(src: BUTTON_CHANGE, time_boot_ms: (src: Int) -> Unit, last_change_ms: (src: Int) -> Unit, state: (src: Byte) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            last_change_ms(src.last_change_ms())

            state(src.state())

        }

        inline fun pull(dst: BUTTON_CHANGE, time_boot_ms: () -> Int, last_change_ms: () -> Int, state: () -> Byte
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.last_change_ms(last_change_ms())

            dst.state(state())

        }

    }
}

inline class SAFETY_SET_ALLOWED_AREA(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun p1x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }


    inline fun p1y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }


    inline fun p1z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }


    inline fun p2x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }


    inline fun p2y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }


    inline fun p2z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }


    inline fun frame(): SAFETY_SET_ALLOWED_AREA_frame? {
        if ((data.field_bit != 208 && !data.set_field(208, -1))) return null

        return SAFETY_SET_ALLOWED_AREA_frame(data)
    }


    companion object {

        val meta = Meta(24, 0, 0, 0, 27, 1, 208, 0, 1)


        inline fun push(src: SAFETY_SET_ALLOWED_AREA, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, p1x: (src: Float) -> Unit, p1y: (src: Float) -> Unit, p1z: (src: Float) -> Unit, p2x: (src: Float) -> Unit, p2y: (src: Float) -> Unit, p2z: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.frame()?.let { item ->
                frame(item.get())
            }

            p1x(src.p1x())

            p1y(src.p1y())

            p1z(src.p1z())

            p2x(src.p2x())

            p2y(src.p2y())

            p2z(src.p2z())

        }

    }
}

inline class UAVCAN_NODE_STATUS(val data: Cursor) {

    inline fun vendor_specific_status_code(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun vendor_specific_status_code_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun uptime_sec(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun uptime_sec_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 6, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 6)
    }

    inline fun sub_mode(): Byte {
        return (data.bytes[data.origin + 14]).toByte()
    }

    inline fun sub_mode_(src: Byte) {
        data.bytes[data.origin + 14] = (src).toByte()
    }


    inline fun health(): UAVCAN_NODE_STATUS_health? {
        if ((data.field_bit != 120 && !data.set_field(120, -1))) return null

        return UAVCAN_NODE_STATUS_health(data)
    }


    inline fun health(src: UAVCAN_NODE_HEALTH) {
        if (data.field_bit != 120) data.set_field(120, 0)

        set_bits((src.value).toULong().toLong(), 2, data.bytes, data.BIT)
    }


    inline fun mode(): UAVCAN_NODE_STATUS_mode? {
        if ((data.field_bit != 121 && !data.set_field(121, -1))) return null

        return UAVCAN_NODE_STATUS_mode(data)
    }


    inline fun mode(src: UAVCAN_NODE_MODE) {
        if (data.field_bit != 121) data.set_field(121, 0)

        set_bits(UAVCAN_NODE_MODE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(68, 1, 1, 1, 16, 1, 120, 0, 2)


        inline fun push(src: UAVCAN_NODE_STATUS, time_usec: (src: Long) -> Unit, uptime_sec: (src: Int) -> Unit, health: (src: com.company.demo.GroundControl.UAVCAN_NODE_HEALTH) -> Unit, mode: (src: com.company.demo.GroundControl.UAVCAN_NODE_MODE) -> Unit, sub_mode: (src: Byte) -> Unit, vendor_specific_status_code: (src: Short) -> Unit
        ) {

            time_usec(src.time_usec())

            uptime_sec(src.uptime_sec())

            src.health()?.let { item ->
                health(item.get())
            }

            src.mode()?.let { item ->
                mode(item.get())
            }

            sub_mode(src.sub_mode())

            vendor_specific_status_code(src.vendor_specific_status_code())

        }

        inline fun pull(dst: UAVCAN_NODE_STATUS, time_usec: () -> Long, uptime_sec: () -> Int, health_exist: () -> Boolean, health: () -> com.company.demo.GroundControl.UAVCAN_NODE_HEALTH, mode_exist: () -> Boolean, mode: () -> com.company.demo.GroundControl.UAVCAN_NODE_MODE, sub_mode: () -> Byte, vendor_specific_status_code: () -> Short
        ) {

            dst.time_usec(time_usec())

            dst.uptime_sec(uptime_sec())

            if (health_exist()) dst.health(health())


            if (mode_exist()) dst.mode(mode())


            dst.sub_mode(sub_mode())

            dst.vendor_specific_status_code(vendor_specific_status_code())

        }

    }
}

inline class COLLISION(val data: Cursor) {

    inline fun id(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun id_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun time_to_minimum_delta(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun time_to_minimum_delta_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun altitude_minimum_delta(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun altitude_minimum_delta_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun horizontal_minimum_delta(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun horizontal_minimum_delta_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }


    inline fun sRc(): COLLISION_sRc? {
        if ((data.field_bit != 130 && !data.set_field(130, -1))) return null

        return COLLISION_sRc(data)
    }


    inline fun sRc(src: MAV_COLLISION_SRC) {
        if (data.field_bit != 130) data.set_field(130, 0)

        set_bits((src.value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    inline fun action(): COLLISION_action? {
        if ((data.field_bit != 131 && !data.set_field(131, -1))) return null

        return COLLISION_action(data)
    }


    inline fun action(src: MAV_COLLISION_ACTION) {
        if (data.field_bit != 131) data.set_field(131, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun threat_level(): COLLISION_threat_level? {
        if ((data.field_bit != 132 && !data.set_field(132, -1))) return null

        return COLLISION_threat_level(data)
    }


    inline fun threat_level(src: MAV_COLLISION_THREAT_LEVEL) {
        if (data.field_bit != 132) data.set_field(132, 0)

        set_bits((src.value).toULong().toLong(), 2, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(45, 0, 1, 0, 17, 1, 130, 2, 3)


        inline fun push(src: COLLISION, sRc: (src: com.company.demo.GroundControl.MAV_COLLISION_SRC) -> Unit, id: (src: Int) -> Unit, action: (src: com.company.demo.GroundControl.MAV_COLLISION_ACTION) -> Unit, threat_level: (src: com.company.demo.GroundControl.MAV_COLLISION_THREAT_LEVEL) -> Unit, time_to_minimum_delta: (src: Float) -> Unit, altitude_minimum_delta: (src: Float) -> Unit, horizontal_minimum_delta: (src: Float) -> Unit
        ) {

            src.sRc()?.let { item ->
                sRc(item.get())
            }

            id(src.id())

            src.action()?.let { item ->
                action(item.get())
            }

            src.threat_level()?.let { item ->
                threat_level(item.get())
            }

            time_to_minimum_delta(src.time_to_minimum_delta())

            altitude_minimum_delta(src.altitude_minimum_delta())

            horizontal_minimum_delta(src.horizontal_minimum_delta())

        }

        inline fun pull(dst: COLLISION, sRc_exist: () -> Boolean, sRc: () -> com.company.demo.GroundControl.MAV_COLLISION_SRC, id: () -> Int, action_exist: () -> Boolean, action: () -> com.company.demo.GroundControl.MAV_COLLISION_ACTION, threat_level_exist: () -> Boolean, threat_level: () -> com.company.demo.GroundControl.MAV_COLLISION_THREAT_LEVEL, time_to_minimum_delta: () -> Float, altitude_minimum_delta: () -> Float, horizontal_minimum_delta: () -> Float
        ) {

            if (sRc_exist()) dst.sRc(sRc())


            dst.id(id())

            if (action_exist()) dst.action(action())


            if (threat_level_exist()) dst.threat_level(threat_level())


            dst.time_to_minimum_delta(time_to_minimum_delta())

            dst.altitude_minimum_delta(altitude_minimum_delta())

            dst.horizontal_minimum_delta(horizontal_minimum_delta())

        }

    }
}

inline class GIMBAL_TORQUE_CMD_REPORT(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun rl_torque_cmd(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun rl_torque_cmd_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun el_torque_cmd(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun el_torque_cmd_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun az_torque_cmd(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun az_torque_cmd_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }


    companion object {

        val meta = Meta(117, 0, 0, 0, 8, 1, 64)


        inline fun push(src: GIMBAL_TORQUE_CMD_REPORT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, rl_torque_cmd: (src: Short) -> Unit, el_torque_cmd: (src: Short) -> Unit, az_torque_cmd: (src: Short) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            rl_torque_cmd(src.rl_torque_cmd())

            el_torque_cmd(src.el_torque_cmd())

            az_torque_cmd(src.az_torque_cmd())

        }

        inline fun pull(dst: GIMBAL_TORQUE_CMD_REPORT, target_system: () -> Byte, target_component: () -> Byte, rl_torque_cmd: () -> Short, el_torque_cmd: () -> Short, az_torque_cmd: () -> Short
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.rl_torque_cmd(rl_torque_cmd())

            dst.el_torque_cmd(el_torque_cmd())

            dst.az_torque_cmd(az_torque_cmd())

        }

    }
}

inline class ALTITUDE(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun altitude_monotonic(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun altitude_monotonic_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun altitude_amsl(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun altitude_amsl_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun altitude_local(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun altitude_local_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun altitude_relative(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun altitude_relative_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun altitude_terrain(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun altitude_terrain_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun bottom_clearance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun bottom_clearance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }


    companion object {

        val meta = Meta(34, 0, 0, 1, 32, 1, 256)


        inline fun push(src: ALTITUDE, time_usec: (src: Long) -> Unit, altitude_monotonic: (src: Float) -> Unit, altitude_amsl: (src: Float) -> Unit, altitude_local: (src: Float) -> Unit, altitude_relative: (src: Float) -> Unit, altitude_terrain: (src: Float) -> Unit, bottom_clearance: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            altitude_monotonic(src.altitude_monotonic())

            altitude_amsl(src.altitude_amsl())

            altitude_local(src.altitude_local())

            altitude_relative(src.altitude_relative())

            altitude_terrain(src.altitude_terrain())

            bottom_clearance(src.bottom_clearance())

        }

        inline fun pull(dst: ALTITUDE, time_usec: () -> Long, altitude_monotonic: () -> Float, altitude_amsl: () -> Float, altitude_local: () -> Float, altitude_relative: () -> Float, altitude_terrain: () -> Float, bottom_clearance: () -> Float
        ) {

            dst.time_usec(time_usec())

            dst.altitude_monotonic(altitude_monotonic())

            dst.altitude_amsl(altitude_amsl())

            dst.altitude_local(altitude_local())

            dst.altitude_relative(altitude_relative())

            dst.altitude_terrain(altitude_terrain())

            dst.bottom_clearance(bottom_clearance())

        }

    }
}

inline class HIL_STATE_QUATERNION(val data: Pack.Bytes) {

    inline fun ind_airspeed(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun ind_airspeed_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun true_airspeed(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun true_airspeed_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 4)
    }

    inline fun attitude_quaternion(): HIL_STATE_QUATERNION_attitude_quaternion {

        return HIL_STATE_QUATERNION_attitude_quaternion(data)
    }

    inline fun attitude_quaternion_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 12 + index * 4)
    }

    object attitude_quaternion {
        const val item_len = 4


    }

    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun rollspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun pitchspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun yawspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 40, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 44, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }

    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 48, 4)).toInt()
    }

    inline fun alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 48)
    }

    inline fun vx(): Short {
        return (get_bytes(data.bytes, data.origin + 52, 2)).toShort()
    }

    inline fun vx_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 52)
    }

    inline fun vy(): Short {
        return (get_bytes(data.bytes, data.origin + 54, 2)).toShort()
    }

    inline fun vy_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 54)
    }

    inline fun vz(): Short {
        return (get_bytes(data.bytes, data.origin + 56, 2)).toShort()
    }

    inline fun vz_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 56)
    }

    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 58, 2)).toShort()
    }

    inline fun xacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 58)
    }

    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 60, 2)).toShort()
    }

    inline fun yacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 60)
    }

    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 62, 2)).toShort()
    }

    inline fun zacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 62)
    }


    companion object {

        val meta = Meta(196, 2, 0, 1, 64, 1, 512)


        inline fun push(src: HIL_STATE_QUATERNION, time_usec: (src: Long) -> Unit, attitude_quaternion: (src: HIL_STATE_QUATERNION_attitude_quaternion) -> Unit, rollspeed: (src: Float) -> Unit, pitchspeed: (src: Float) -> Unit, yawspeed: (src: Float) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Int) -> Unit, vx: (src: Short) -> Unit, vy: (src: Short) -> Unit, vz: (src: Short) -> Unit, ind_airspeed: (src: Short) -> Unit, true_airspeed: (src: Short) -> Unit, xacc: (src: Short) -> Unit, yacc: (src: Short) -> Unit, zacc: (src: Short) -> Unit
        ) {

            time_usec(src.time_usec())
            src.attitude_quaternion().let { item ->
                attitude_quaternion(item)
            }

            rollspeed(src.rollspeed())

            pitchspeed(src.pitchspeed())

            yawspeed(src.yawspeed())

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            ind_airspeed(src.ind_airspeed())

            true_airspeed(src.true_airspeed())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

        }

        inline fun pull(dst: HIL_STATE_QUATERNION, time_usec: () -> Long, attitude_quaternion: (dst: HIL_STATE_QUATERNION_attitude_quaternion) -> Unit, rollspeed: () -> Float, pitchspeed: () -> Float, yawspeed: () -> Float, lat: () -> Int, lon: () -> Int, alt: () -> Int, vx: () -> Short, vy: () -> Short, vz: () -> Short, ind_airspeed: () -> Short, true_airspeed: () -> Short, xacc: () -> Short, yacc: () -> Short, zacc: () -> Short
        ) {

            dst.time_usec(time_usec())
            attitude_quaternion(dst.attitude_quaternion())

            dst.rollspeed(rollspeed())

            dst.pitchspeed(pitchspeed())

            dst.yawspeed(yawspeed())

            dst.lat(lat())

            dst.lon(lon())

            dst.alt(alt())

            dst.vx(vx())

            dst.vy(vy())

            dst.vz(vz())

            dst.ind_airspeed(ind_airspeed())

            dst.true_airspeed(true_airspeed())

            dst.xacc(xacc())

            dst.yacc(yacc())

            dst.zacc(zacc())

        }

    }
}

inline class SENSOR_OFFSETS(val data: Pack.Bytes) {

    inline fun mag_ofs_x(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun mag_ofs_x_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun mag_ofs_y(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun mag_ofs_y_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun mag_ofs_z(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun mag_ofs_z_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun mag_declination(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun mag_declination_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun raw_press(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun raw_press_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun raw_temp(): Int {
        return (get_bytes(data.bytes, data.origin + 14, 4)).toInt()
    }

    inline fun raw_temp_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun gyro_cal_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun gyro_cal_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun gyro_cal_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun gyro_cal_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

    inline fun gyro_cal_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }

    inline fun gyro_cal_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 26)
    }

    inline fun accel_cal_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }

    inline fun accel_cal_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 30)
    }

    inline fun accel_cal_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 34, 4).toInt())
    }

    inline fun accel_cal_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 34)
    }

    inline fun accel_cal_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 38, 4).toInt())
    }

    inline fun accel_cal_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 38)
    }


    companion object {

        val meta = Meta(128, 0, 0, 0, 42, 1, 336)


        inline fun push(src: SENSOR_OFFSETS, mag_ofs_x: (src: Short) -> Unit, mag_ofs_y: (src: Short) -> Unit, mag_ofs_z: (src: Short) -> Unit, mag_declination: (src: Float) -> Unit, raw_press: (src: Int) -> Unit, raw_temp: (src: Int) -> Unit, gyro_cal_x: (src: Float) -> Unit, gyro_cal_y: (src: Float) -> Unit, gyro_cal_z: (src: Float) -> Unit, accel_cal_x: (src: Float) -> Unit, accel_cal_y: (src: Float) -> Unit, accel_cal_z: (src: Float) -> Unit
        ) {

            mag_ofs_x(src.mag_ofs_x())

            mag_ofs_y(src.mag_ofs_y())

            mag_ofs_z(src.mag_ofs_z())

            mag_declination(src.mag_declination())

            raw_press(src.raw_press())

            raw_temp(src.raw_temp())

            gyro_cal_x(src.gyro_cal_x())

            gyro_cal_y(src.gyro_cal_y())

            gyro_cal_z(src.gyro_cal_z())

            accel_cal_x(src.accel_cal_x())

            accel_cal_y(src.accel_cal_y())

            accel_cal_z(src.accel_cal_z())

        }

        inline fun pull(dst: SENSOR_OFFSETS, mag_ofs_x: () -> Short, mag_ofs_y: () -> Short, mag_ofs_z: () -> Short, mag_declination: () -> Float, raw_press: () -> Int, raw_temp: () -> Int, gyro_cal_x: () -> Float, gyro_cal_y: () -> Float, gyro_cal_z: () -> Float, accel_cal_x: () -> Float, accel_cal_y: () -> Float, accel_cal_z: () -> Float
        ) {

            dst.mag_ofs_x(mag_ofs_x())

            dst.mag_ofs_y(mag_ofs_y())

            dst.mag_ofs_z(mag_ofs_z())

            dst.mag_declination(mag_declination())

            dst.raw_press(raw_press())

            dst.raw_temp(raw_temp())

            dst.gyro_cal_x(gyro_cal_x())

            dst.gyro_cal_y(gyro_cal_y())

            dst.gyro_cal_z(gyro_cal_z())

            dst.accel_cal_x(accel_cal_x())

            dst.accel_cal_y(accel_cal_y())

            dst.accel_cal_z(accel_cal_z())

        }

    }
}

inline class STORAGE_INFORMATION(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun storage_id(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun storage_id_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun storage_count(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun storage_count_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun status(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun status_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun total_capacity(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 7, 4).toInt())
    }

    inline fun total_capacity_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 7)
    }

    inline fun used_capacity(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 11, 4).toInt())
    }

    inline fun used_capacity_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 11)
    }

    inline fun available_capacity(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 15, 4).toInt())
    }

    inline fun available_capacity_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 15)
    }

    inline fun read_speed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 19, 4).toInt())
    }

    inline fun read_speed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 19)
    }

    inline fun write_speed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 23, 4).toInt())
    }

    inline fun write_speed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 23)
    }


    companion object {

        val meta = Meta(58, 0, 1, 0, 27, 1, 216)


        inline fun push(src: STORAGE_INFORMATION, time_boot_ms: (src: Int) -> Unit, storage_id: (src: Byte) -> Unit, storage_count: (src: Byte) -> Unit, status: (src: Byte) -> Unit, total_capacity: (src: Float) -> Unit, used_capacity: (src: Float) -> Unit, available_capacity: (src: Float) -> Unit, read_speed: (src: Float) -> Unit, write_speed: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            storage_id(src.storage_id())

            storage_count(src.storage_count())

            status(src.status())

            total_capacity(src.total_capacity())

            used_capacity(src.used_capacity())

            available_capacity(src.available_capacity())

            read_speed(src.read_speed())

            write_speed(src.write_speed())

        }

        inline fun pull(dst: STORAGE_INFORMATION, time_boot_ms: () -> Int, storage_id: () -> Byte, storage_count: () -> Byte, status: () -> Byte, total_capacity: () -> Float, used_capacity: () -> Float, available_capacity: () -> Float, read_speed: () -> Float, write_speed: () -> Float
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.storage_id(storage_id())

            dst.storage_count(storage_count())

            dst.status(status())

            dst.total_capacity(total_capacity())

            dst.used_capacity(used_capacity())

            dst.available_capacity(available_capacity())

            dst.read_speed(read_speed())

            dst.write_speed(write_speed())

        }

    }
}

inline class CAMERA_INFORMATION(val data: Cursor) {

    inline fun resolution_h(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun resolution_h_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun resolution_v(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun resolution_v_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun cam_definition_version(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun cam_definition_version_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun firmware_version(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun firmware_version_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun vendor_name(): CAMERA_INFORMATION_vendor_name {

        return CAMERA_INFORMATION_vendor_name(data)
    }

    inline fun vendor_name_(src: ByteArray) {
        val len = minOf(src.size, 32)

        for (index in 0 until len)
            data.bytes[data.origin + 14 + index] = (src[index]).toByte()
    }

    object vendor_name {
        const val item_len = 32


    }

    inline fun model_name(): CAMERA_INFORMATION_model_name {

        return CAMERA_INFORMATION_model_name(data)
    }

    inline fun model_name_(src: ByteArray) {
        val len = minOf(src.size, 32)

        for (index in 0 until len)
            data.bytes[data.origin + 46 + index] = (src[index]).toByte()
    }

    object model_name {
        const val item_len = 32


    }

    inline fun focal_length(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 78, 4).toInt())
    }

    inline fun focal_length_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 78)
    }

    inline fun sensor_size_h(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 82, 4).toInt())
    }

    inline fun sensor_size_h_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 82)
    }

    inline fun sensor_size_v(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 86, 4).toInt())
    }

    inline fun sensor_size_v_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 86)
    }

    inline fun lens_id(): Byte {
        return (data.bytes[data.origin + 90]).toByte()
    }

    inline fun lens_id_(src: Byte) {
        data.bytes[data.origin + 90] = (src).toByte()
    }


    inline fun flags(): CAMERA_INFORMATION_flags? {
        if ((data.field_bit != 730 && !data.set_field(730, -1))) return null

        return CAMERA_INFORMATION_flags(data)
    }


    inline fun flags(src: CAMERA_CAP_FLAGS) {
        if (data.field_bit != 730) data.set_field(730, 0)

        set_bits(CAMERA_CAP_FLAGS.set(src).toLong(), 3, data.bytes, data.BIT)
    }


    inline fun cam_definition_uri(): CAMERA_INFORMATION_cam_definition_uri? {
        if ((data.field_bit != 731 && !data.set_field(731, -1))) return null

        return CAMERA_INFORMATION_cam_definition_uri(data)
    }


    inline fun cam_definition_uri_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -731 - 1, reuse)
    }


    inline fun cam_definition_uri_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(731, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun cam_definition_uri_(len: Int): CAMERA_INFORMATION_cam_definition_uri {

        data.set_field(731, minOf(len, 255))
        return CAMERA_INFORMATION_cam_definition_uri(data)
    }

    object cam_definition_uri {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(157, 3, 2, 0, 92, 1, 730, 2, 2)


        inline fun push(src: CAMERA_INFORMATION, time_boot_ms: (src: Int) -> Unit, vendor_name: (src: CAMERA_INFORMATION_vendor_name) -> Unit, model_name: (src: CAMERA_INFORMATION_model_name) -> Unit, firmware_version: (src: Int) -> Unit, focal_length: (src: Float) -> Unit, sensor_size_h: (src: Float) -> Unit, sensor_size_v: (src: Float) -> Unit, resolution_h: (src: Short) -> Unit, resolution_v: (src: Short) -> Unit, lens_id: (src: Byte) -> Unit, flags: (src: com.company.demo.GroundControl.CAMERA_CAP_FLAGS) -> Unit, cam_definition_version: (src: Short) -> Unit, cam_definition_uri: (src: CAMERA_INFORMATION_cam_definition_uri) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())
            src.vendor_name().let { item ->
                vendor_name(item)
            }
            src.model_name().let { item ->
                model_name(item)
            }

            firmware_version(src.firmware_version())

            focal_length(src.focal_length())

            sensor_size_h(src.sensor_size_h())

            sensor_size_v(src.sensor_size_v())

            resolution_h(src.resolution_h())

            resolution_v(src.resolution_v())

            lens_id(src.lens_id())

            src.flags()?.let { item ->
                flags(item.get())
            }

            cam_definition_version(src.cam_definition_version())

            src.cam_definition_uri()?.let { item -> cam_definition_uri(item) }

        }

        inline fun pull(dst: CAMERA_INFORMATION, time_boot_ms: () -> Int, vendor_name: (dst: CAMERA_INFORMATION_vendor_name) -> Unit, model_name: (dst: CAMERA_INFORMATION_model_name) -> Unit, firmware_version: () -> Int, focal_length: () -> Float, sensor_size_h: () -> Float, sensor_size_v: () -> Float, resolution_h: () -> Short, resolution_v: () -> Short, lens_id: () -> Byte, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.CAMERA_CAP_FLAGS, cam_definition_version: () -> Short, cam_definition_uri_exist: () -> Int, cam_definition_uri: (dst: CAMERA_INFORMATION_cam_definition_uri) -> Unit
        ) {

            dst.time_boot_ms(time_boot_ms())
            vendor_name(dst.vendor_name())
            model_name(dst.model_name())

            dst.firmware_version(firmware_version())

            dst.focal_length(focal_length())

            dst.sensor_size_h(sensor_size_h())

            dst.sensor_size_v(sensor_size_v())

            dst.resolution_h(resolution_h())

            dst.resolution_v(resolution_v())

            dst.lens_id(lens_id())

            if (flags_exist()) dst.flags(flags())


            dst.cam_definition_version(cam_definition_version())


            cam_definition_uri_exist().let { len ->
                if (0 < len)
                    cam_definition_uri(dst.cam_definition_uri(len))
            }


        }

    }
}

inline class GPS_STATUS(val data: Pack.Bytes) {

    inline fun satellites_visible(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun satellite_prn(): GPS_STATUS_satellite_prn {

        return GPS_STATUS_satellite_prn(data)
    }

    object satellite_prn {
        const val item_len = 20


    }

    inline fun satellite_used(): GPS_STATUS_satellite_used {

        return GPS_STATUS_satellite_used(data)
    }

    object satellite_used {
        const val item_len = 20


    }

    inline fun satellite_elevation(): GPS_STATUS_satellite_elevation {

        return GPS_STATUS_satellite_elevation(data)
    }

    object satellite_elevation {
        const val item_len = 20


    }

    inline fun satellite_azimuth(): GPS_STATUS_satellite_azimuth {

        return GPS_STATUS_satellite_azimuth(data)
    }

    object satellite_azimuth {
        const val item_len = 20


    }

    inline fun satellite_snr(): GPS_STATUS_satellite_snr {

        return GPS_STATUS_satellite_snr(data)
    }

    object satellite_snr {
        const val item_len = 20


    }


    companion object {

        val meta = Meta(48, 0, 0, 0, 101, 1, 808)


        inline fun push(src: GPS_STATUS, satellites_visible: (src: Byte) -> Unit, satellite_prn: (src: GPS_STATUS_satellite_prn) -> Unit, satellite_used: (src: GPS_STATUS_satellite_used) -> Unit, satellite_elevation: (src: GPS_STATUS_satellite_elevation) -> Unit, satellite_azimuth: (src: GPS_STATUS_satellite_azimuth) -> Unit, satellite_snr: (src: GPS_STATUS_satellite_snr) -> Unit
        ) {

            satellites_visible(src.satellites_visible())
            src.satellite_prn().let { item ->
                satellite_prn(item)
            }
            src.satellite_used().let { item ->
                satellite_used(item)
            }
            src.satellite_elevation().let { item ->
                satellite_elevation(item)
            }
            src.satellite_azimuth().let { item ->
                satellite_azimuth(item)
            }
            src.satellite_snr().let { item ->
                satellite_snr(item)
            }

        }

    }
}

inline class DEVICE_OP_WRITE_REPLY(val data: Pack.Bytes) {

    inline fun request_id(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun request_id_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun result(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun result_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }


    companion object {

        val meta = Meta(33, 0, 1, 0, 5, 1, 40)


        inline fun push(src: DEVICE_OP_WRITE_REPLY, request_id: (src: Int) -> Unit, result: (src: Byte) -> Unit
        ) {

            request_id(src.request_id())

            result(src.result())

        }

        inline fun pull(dst: DEVICE_OP_WRITE_REPLY, request_id: () -> Int, result: () -> Byte
        ) {

            dst.request_id(request_id())

            dst.result(result())

        }

    }
}

inline class PARAM_SET(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun param_value(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }


    inline fun param_id(): PARAM_SET_param_id? {
        if ((data.field_bit != 50 && !data.set_field(50, -1))) return null

        return PARAM_SET_param_id(data)
    }


    object param_id {
        const val item_len_max = 255

    }


    inline fun param_type(): PARAM_SET_param_type? {
        if ((data.field_bit != 51 && !data.set_field(51, -1))) return null

        return PARAM_SET_param_type(data)
    }


    companion object {

        val meta = Meta(123, 0, 0, 0, 7, 1, 50, 2, 2)


        inline fun push(src: PARAM_SET, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, param_id: (src: PARAM_SET_param_id) -> Unit, param_value: (src: Float) -> Unit, param_type: (src: com.company.demo.GroundControl.MAV_PARAM_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.param_id()?.let { item -> param_id(item) }

            param_value(src.param_value())

            src.param_type()?.let { item ->
                param_type(item.get())
            }

        }

    }
}

inline class TERRAIN_DATA(val data: Pack.Bytes) {

    inline fun grid_spacing(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun grid_spacing_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun gridbit(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun gridbit_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun daTa(): TERRAIN_DATA_daTa {

        return TERRAIN_DATA_daTa(data)
    }

    inline fun daTa_(src: ShortArray) {
        val len = minOf(src.size, 16)

        for (index in 0 until len)
            set_bytes((src[index]).toULong().toLong(), 2, data.bytes, data.origin + 11 + index * 2)
    }

    object daTa {
        const val item_len = 16


    }


    companion object {

        val meta = Meta(173, 1, 0, 0, 43, 1, 344)


        inline fun push(src: TERRAIN_DATA, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, grid_spacing: (src: Short) -> Unit, gridbit: (src: Byte) -> Unit, daTa: (src: TERRAIN_DATA_daTa) -> Unit
        ) {

            lat(src.lat())

            lon(src.lon())

            grid_spacing(src.grid_spacing())

            gridbit(src.gridbit())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: TERRAIN_DATA, lat: () -> Int, lon: () -> Int, grid_spacing: () -> Short, gridbit: () -> Byte, daTa: (dst: TERRAIN_DATA_daTa) -> Unit
        ) {

            dst.lat(lat())

            dst.lon(lon())

            dst.grid_spacing(grid_spacing())

            dst.gridbit(gridbit())
            daTa(dst.daTa())

        }

    }
}

inline class GIMBAL_CONTROL(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun demanded_rate_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }

    inline fun demanded_rate_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun demanded_rate_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun demanded_rate_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun demanded_rate_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }

    inline fun demanded_rate_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }


    companion object {

        val meta = Meta(104, 0, 0, 0, 14, 1, 112)


        inline fun push(src: GIMBAL_CONTROL, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, demanded_rate_x: (src: Float) -> Unit, demanded_rate_y: (src: Float) -> Unit, demanded_rate_z: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            demanded_rate_x(src.demanded_rate_x())

            demanded_rate_y(src.demanded_rate_y())

            demanded_rate_z(src.demanded_rate_z())

        }

        inline fun pull(dst: GIMBAL_CONTROL, target_system: () -> Byte, target_component: () -> Byte, demanded_rate_x: () -> Float, demanded_rate_y: () -> Float, demanded_rate_z: () -> Float
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.demanded_rate_x(demanded_rate_x())

            dst.demanded_rate_y(demanded_rate_y())

            dst.demanded_rate_z(demanded_rate_z())

        }

    }
}

inline class RC_CHANNELS_OVERRIDE(val data: Pack.Bytes) {

    inline fun chan1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun chan2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun chan3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun chan4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }


    inline fun chan5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun chan6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun chan7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun chan8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 17]).toByte()
    }


    companion object {

        val meta = Meta(55, 8, 0, 0, 18, 1, 144)


        inline fun push(src: RC_CHANNELS_OVERRIDE, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, chan1_raw: (src: Short) -> Unit, chan2_raw: (src: Short) -> Unit, chan3_raw: (src: Short) -> Unit, chan4_raw: (src: Short) -> Unit, chan5_raw: (src: Short) -> Unit, chan6_raw: (src: Short) -> Unit, chan7_raw: (src: Short) -> Unit, chan8_raw: (src: Short) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            chan1_raw(src.chan1_raw())

            chan2_raw(src.chan2_raw())

            chan3_raw(src.chan3_raw())

            chan4_raw(src.chan4_raw())

            chan5_raw(src.chan5_raw())

            chan6_raw(src.chan6_raw())

            chan7_raw(src.chan7_raw())

            chan8_raw(src.chan8_raw())

        }

    }
}

inline class SCALED_IMU(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }


    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun xgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun ygyro(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun zgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    inline fun xmag(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }


    inline fun ymag(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }


    inline fun zmag(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }


    companion object {

        val meta = Meta(195, 0, 1, 0, 22, 1, 176)


        inline fun push(src: SCALED_IMU, time_boot_ms: (src: Int) -> Unit, xacc: (src: Short) -> Unit, yacc: (src: Short) -> Unit, zacc: (src: Short) -> Unit, xgyro: (src: Short) -> Unit, ygyro: (src: Short) -> Unit, zgyro: (src: Short) -> Unit, xmag: (src: Short) -> Unit, ymag: (src: Short) -> Unit, zmag: (src: Short) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

            xgyro(src.xgyro())

            ygyro(src.ygyro())

            zgyro(src.zgyro())

            xmag(src.xmag())

            ymag(src.ymag())

            zmag(src.zmag())

        }

    }
}

inline class VIDEO_STREAM_INFORMATION(val data: Cursor) {

    inline fun resolution_h(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun resolution_h_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun resolution_v(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun resolution_v_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun rotation(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun rotation_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun bitrate(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun bitrate_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun camera_id(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun camera_id_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun status(): Byte {
        return (data.bytes[data.origin + 11]).toByte()
    }

    inline fun status_(src: Byte) {
        data.bytes[data.origin + 11] = (src).toByte()
    }

    inline fun framerate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun framerate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }


    inline fun uri(): VIDEO_STREAM_INFORMATION_uri? {
        if ((data.field_bit != 130 && !data.set_field(130, -1))) return null

        return VIDEO_STREAM_INFORMATION_uri(data)
    }


    inline fun uri_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -130 - 1, reuse)
    }


    inline fun uri_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(130, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun uri_(len: Int): VIDEO_STREAM_INFORMATION_uri {

        data.set_field(130, minOf(len, 255))
        return VIDEO_STREAM_INFORMATION_uri(data)
    }

    object uri {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(1, 3, 1, 0, 17, 1, 130, 2, 1)


        inline fun push(src: VIDEO_STREAM_INFORMATION, camera_id: (src: Byte) -> Unit, status: (src: Byte) -> Unit, framerate: (src: Float) -> Unit, resolution_h: (src: Short) -> Unit, resolution_v: (src: Short) -> Unit, bitrate: (src: Int) -> Unit, rotation: (src: Short) -> Unit, uri: (src: VIDEO_STREAM_INFORMATION_uri) -> Unit
        ) {

            camera_id(src.camera_id())

            status(src.status())

            framerate(src.framerate())

            resolution_h(src.resolution_h())

            resolution_v(src.resolution_v())

            bitrate(src.bitrate())

            rotation(src.rotation())

            src.uri()?.let { item -> uri(item) }

        }

        inline fun pull(dst: VIDEO_STREAM_INFORMATION, camera_id: () -> Byte, status: () -> Byte, framerate: () -> Float, resolution_h: () -> Short, resolution_v: () -> Short, bitrate: () -> Int, rotation: () -> Short, uri_exist: () -> Int, uri: (dst: VIDEO_STREAM_INFORMATION_uri) -> Unit
        ) {

            dst.camera_id(camera_id())

            dst.status(status())

            dst.framerate(framerate())

            dst.resolution_h(resolution_h())

            dst.resolution_v(resolution_v())

            dst.bitrate(bitrate())

            dst.rotation(rotation())


            uri_exist().let { len ->
                if (0 < len)
                    uri(dst.uri(len))
            }


        }

    }
}

inline class AHRS(val data: Pack.Bytes) {

    inline fun omegaIx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun omegaIx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun omegaIy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun omegaIy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun omegaIz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun omegaIz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun accel_weight(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun accel_weight_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun renorm_val(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun renorm_val_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun error_rp(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun error_rp_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun error_yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun error_yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }


    companion object {

        val meta = Meta(214, 0, 0, 0, 28, 1, 224)


        inline fun push(src: AHRS, omegaIx: (src: Float) -> Unit, omegaIy: (src: Float) -> Unit, omegaIz: (src: Float) -> Unit, accel_weight: (src: Float) -> Unit, renorm_val: (src: Float) -> Unit, error_rp: (src: Float) -> Unit, error_yaw: (src: Float) -> Unit
        ) {

            omegaIx(src.omegaIx())

            omegaIy(src.omegaIy())

            omegaIz(src.omegaIz())

            accel_weight(src.accel_weight())

            renorm_val(src.renorm_val())

            error_rp(src.error_rp())

            error_yaw(src.error_yaw())

        }

        inline fun pull(dst: AHRS, omegaIx: () -> Float, omegaIy: () -> Float, omegaIz: () -> Float, accel_weight: () -> Float, renorm_val: () -> Float, error_rp: () -> Float, error_yaw: () -> Float
        ) {

            dst.omegaIx(omegaIx())

            dst.omegaIy(omegaIy())

            dst.omegaIz(omegaIz())

            dst.accel_weight(accel_weight())

            dst.renorm_val(renorm_val())

            dst.error_rp(error_rp())

            dst.error_yaw(error_yaw())

        }

    }
}

inline class DEBUG(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun ind(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun ind_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun value(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 5, 4).toInt())
    }

    inline fun value_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 5)
    }


    companion object {

        val meta = Meta(126, 0, 1, 0, 9, 1, 72)


        inline fun push(src: DEBUG, time_boot_ms: (src: Int) -> Unit, ind: (src: Byte) -> Unit, value: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            ind(src.ind())

            value(src.value())

        }

        inline fun pull(dst: DEBUG, time_boot_ms: () -> Int, ind: () -> Byte, value: () -> Float
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.ind(ind())

            dst.value(value())

        }

    }
}

inline class CAMERA_IMAGE_CAPTURED(val data: Cursor) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun time_utc(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }

    inline fun time_utc_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 4)
    }

    inline fun camera_id(): Byte {
        return (data.bytes[data.origin + 12]).toByte()
    }

    inline fun camera_id_(src: Byte) {
        data.bytes[data.origin + 12] = (src).toByte()
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 13, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 13)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 17, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }

    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 21, 4)).toInt()
    }

    inline fun alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 21)
    }

    inline fun relative_alt(): Int {
        return (get_bytes(data.bytes, data.origin + 25, 4)).toInt()
    }

    inline fun relative_alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 25)
    }

    inline fun q(): CAMERA_IMAGE_CAPTURED_q {

        return CAMERA_IMAGE_CAPTURED_q(data)
    }

    inline fun q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 29 + index * 4)
    }

    object q {
        const val item_len = 4


    }

    inline fun image_index(): Int {
        return (get_bytes(data.bytes, data.origin + 45, 4)).toInt()
    }

    inline fun image_index_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 45)
    }

    inline fun capture_result(): Byte {
        return (data.bytes[data.origin + 49]).toByte()
    }

    inline fun capture_result_(src: Byte) {
        data.bytes[data.origin + 49] = (src).toByte()
    }


    inline fun file_url(): CAMERA_IMAGE_CAPTURED_file_url? {
        if ((data.field_bit != 402 && !data.set_field(402, -1))) return null

        return CAMERA_IMAGE_CAPTURED_file_url(data)
    }


    inline fun file_url_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -402 - 1, reuse)
    }


    inline fun file_url_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(402, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun file_url_(len: Int): CAMERA_IMAGE_CAPTURED_file_url {

        data.set_field(402, minOf(len, 255))
        return CAMERA_IMAGE_CAPTURED_file_url(data)
    }

    object file_url {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(10, 0, 1, 1, 51, 1, 402, 2, 1)


        inline fun push(src: CAMERA_IMAGE_CAPTURED, time_boot_ms: (src: Int) -> Unit, time_utc: (src: Long) -> Unit, camera_id: (src: Byte) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Int) -> Unit, relative_alt: (src: Int) -> Unit, q: (src: CAMERA_IMAGE_CAPTURED_q) -> Unit, image_index: (src: Int) -> Unit, capture_result: (src: Byte) -> Unit, file_url: (src: CAMERA_IMAGE_CAPTURED_file_url) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            time_utc(src.time_utc())

            camera_id(src.camera_id())

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            relative_alt(src.relative_alt())
            src.q().let { item ->
                q(item)
            }

            image_index(src.image_index())

            capture_result(src.capture_result())

            src.file_url()?.let { item -> file_url(item) }

        }

        inline fun pull(dst: CAMERA_IMAGE_CAPTURED, time_boot_ms: () -> Int, time_utc: () -> Long, camera_id: () -> Byte, lat: () -> Int, lon: () -> Int, alt: () -> Int, relative_alt: () -> Int, q: (dst: CAMERA_IMAGE_CAPTURED_q) -> Unit, image_index: () -> Int, capture_result: () -> Byte, file_url_exist: () -> Int, file_url: (dst: CAMERA_IMAGE_CAPTURED_file_url) -> Unit
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.time_utc(time_utc())

            dst.camera_id(camera_id())

            dst.lat(lat())

            dst.lon(lon())

            dst.alt(alt())

            dst.relative_alt(relative_alt())
            q(dst.q())

            dst.image_index(image_index())

            dst.capture_result(capture_result())


            file_url_exist().let { len ->
                if (0 < len)
                    file_url(dst.file_url(len))
            }


        }

    }
}

inline class LOG_ENTRY(val data: Pack.Bytes) {

    inline fun id(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun id_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun num_logs(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun num_logs_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun last_log_num(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun last_log_num_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun time_utc(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun time_utc_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun size(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun size_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }


    companion object {

        val meta = Meta(161, 3, 2, 0, 14, 1, 112)


        inline fun push(src: LOG_ENTRY, id: (src: Short) -> Unit, num_logs: (src: Short) -> Unit, last_log_num: (src: Short) -> Unit, time_utc: (src: Int) -> Unit, size: (src: Int) -> Unit
        ) {

            id(src.id())

            num_logs(src.num_logs())

            last_log_num(src.last_log_num())

            time_utc(src.time_utc())

            size(src.size())

        }

        inline fun pull(dst: LOG_ENTRY, id: () -> Short, num_logs: () -> Short, last_log_num: () -> Short, time_utc: () -> Int, size: () -> Int
        ) {

            dst.id(id())

            dst.num_logs(num_logs())

            dst.last_log_num(last_log_num())

            dst.time_utc(time_utc())

            dst.size(size())

        }

    }
}

inline class ACTUATOR_CONTROL_TARGET(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun group_mlx(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun group_mlx_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun controls(): ACTUATOR_CONTROL_TARGET_controls {

        return ACTUATOR_CONTROL_TARGET_controls(data)
    }

    inline fun controls_(src: FloatArray) {
        val len = minOf(src.size, 8)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 9 + index * 4)
    }

    object controls {
        const val item_len = 8


    }


    companion object {

        val meta = Meta(3, 0, 0, 1, 41, 1, 328)


        inline fun push(src: ACTUATOR_CONTROL_TARGET, time_usec: (src: Long) -> Unit, group_mlx: (src: Byte) -> Unit, controls: (src: ACTUATOR_CONTROL_TARGET_controls) -> Unit
        ) {

            time_usec(src.time_usec())

            group_mlx(src.group_mlx())
            src.controls().let { item ->
                controls(item)
            }

        }

        inline fun pull(dst: ACTUATOR_CONTROL_TARGET, time_usec: () -> Long, group_mlx: () -> Byte, controls: (dst: ACTUATOR_CONTROL_TARGET_controls) -> Unit
        ) {

            dst.time_usec(time_usec())

            dst.group_mlx(group_mlx())
            controls(dst.controls())

        }

    }
}

inline class HIGH_LATENCY(val data: Cursor) {

    inline fun heading(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun heading_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun wp_distance(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun wp_distance_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun custom_mode(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun custom_mode_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun roll(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun roll_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun pitch(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun pitch_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun throttle(): Byte {
        return (data.bytes[data.origin + 12]).toByte()
    }

    inline fun throttle_(src: Byte) {
        data.bytes[data.origin + 12] = (src).toByte()
    }

    inline fun heading_sp(): Short {
        return (get_bytes(data.bytes, data.origin + 13, 2)).toShort()
    }

    inline fun heading_sp_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 13)
    }

    inline fun latitude(): Int {
        return (get_bytes(data.bytes, data.origin + 15, 4)).toInt()
    }

    inline fun latitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 15)
    }

    inline fun longitude(): Int {
        return (get_bytes(data.bytes, data.origin + 19, 4)).toInt()
    }

    inline fun longitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 19)
    }

    inline fun altitude_amsl(): Short {
        return (get_bytes(data.bytes, data.origin + 23, 2)).toShort()
    }

    inline fun altitude_amsl_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 23)
    }

    inline fun altitude_sp(): Short {
        return (get_bytes(data.bytes, data.origin + 25, 2)).toShort()
    }

    inline fun altitude_sp_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 25)
    }

    inline fun airspeed(): Byte {
        return (data.bytes[data.origin + 27]).toByte()
    }

    inline fun airspeed_(src: Byte) {
        data.bytes[data.origin + 27] = (src).toByte()
    }

    inline fun airspeed_sp(): Byte {
        return (data.bytes[data.origin + 28]).toByte()
    }

    inline fun airspeed_sp_(src: Byte) {
        data.bytes[data.origin + 28] = (src).toByte()
    }

    inline fun groundspeed(): Byte {
        return (data.bytes[data.origin + 29]).toByte()
    }

    inline fun groundspeed_(src: Byte) {
        data.bytes[data.origin + 29] = (src).toByte()
    }

    inline fun climb_rate(): Byte {
        return (data.bytes[data.origin + 30]).toByte()
    }

    inline fun climb_rate_(src: Byte) {
        data.bytes[data.origin + 30] = (src).toByte()
    }

    inline fun gps_nsat(): Byte {
        return (data.bytes[data.origin + 31]).toByte()
    }

    inline fun gps_nsat_(src: Byte) {
        data.bytes[data.origin + 31] = (src).toByte()
    }

    inline fun battery_remaining(): Byte {
        return (data.bytes[data.origin + 32]).toByte()
    }

    inline fun battery_remaining_(src: Byte) {
        data.bytes[data.origin + 32] = (src).toByte()
    }

    inline fun temperature(): Byte {
        return (data.bytes[data.origin + 33]).toByte()
    }

    inline fun temperature_(src: Byte) {
        data.bytes[data.origin + 33] = (src).toByte()
    }

    inline fun temperature_air(): Byte {
        return (data.bytes[data.origin + 34]).toByte()
    }

    inline fun temperature_air_(src: Byte) {
        data.bytes[data.origin + 34] = (src).toByte()
    }

    inline fun failsafe(): Byte {
        return (data.bytes[data.origin + 35]).toByte()
    }

    inline fun failsafe_(src: Byte) {
        data.bytes[data.origin + 35] = (src).toByte()
    }

    inline fun wp_num(): Byte {
        return (data.bytes[data.origin + 36]).toByte()
    }

    inline fun wp_num_(src: Byte) {
        data.bytes[data.origin + 36] = (src).toByte()
    }


    inline fun base_mode(): HIGH_LATENCY_base_mode? {
        if ((data.field_bit != 298 && !data.set_field(298, -1))) return null

        return HIGH_LATENCY_base_mode(data)
    }


    inline fun base_mode(src: MAV_MODE_FLAG) {
        if (data.field_bit != 298) data.set_field(298, 0)

        set_bits(MAV_MODE_FLAG.set(src).toLong(), 4, data.bytes, data.BIT)
    }


    inline fun landed_state(): HIGH_LATENCY_landed_state? {
        if ((data.field_bit != 299 && !data.set_field(299, -1))) return null

        return HIGH_LATENCY_landed_state(data)
    }


    inline fun landed_state(src: MAV_LANDED_STATE) {
        if (data.field_bit != 299) data.set_field(299, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun gps_fix_type(): HIGH_LATENCY_gps_fix_type? {
        if ((data.field_bit != 300 && !data.set_field(300, -1))) return null

        return HIGH_LATENCY_gps_fix_type(data)
    }


    inline fun gps_fix_type(src: GPS_FIX_TYPE) {
        if (data.field_bit != 300) data.set_field(300, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(87, 2, 1, 0, 38, 1, 298, 2, 3)


        inline fun push(src: HIGH_LATENCY, base_mode: (src: com.company.demo.GroundControl.MAV_MODE_FLAG) -> Unit, custom_mode: (src: Int) -> Unit, landed_state: (src: com.company.demo.GroundControl.MAV_LANDED_STATE) -> Unit, roll: (src: Short) -> Unit, pitch: (src: Short) -> Unit, heading: (src: Short) -> Unit, throttle: (src: Byte) -> Unit, heading_sp: (src: Short) -> Unit, latitude: (src: Int) -> Unit, longitude: (src: Int) -> Unit, altitude_amsl: (src: Short) -> Unit, altitude_sp: (src: Short) -> Unit, airspeed: (src: Byte) -> Unit, airspeed_sp: (src: Byte) -> Unit, groundspeed: (src: Byte) -> Unit, climb_rate: (src: Byte) -> Unit, gps_nsat: (src: Byte) -> Unit, gps_fix_type: (src: com.company.demo.GroundControl.GPS_FIX_TYPE) -> Unit, battery_remaining: (src: Byte) -> Unit, temperature: (src: Byte) -> Unit, temperature_air: (src: Byte) -> Unit, failsafe: (src: Byte) -> Unit, wp_num: (src: Byte) -> Unit, wp_distance: (src: Short) -> Unit
        ) {

            src.base_mode()?.let { item ->
                base_mode(item.get())
            }

            custom_mode(src.custom_mode())

            src.landed_state()?.let { item ->
                landed_state(item.get())
            }

            roll(src.roll())

            pitch(src.pitch())

            heading(src.heading())

            throttle(src.throttle())

            heading_sp(src.heading_sp())

            latitude(src.latitude())

            longitude(src.longitude())

            altitude_amsl(src.altitude_amsl())

            altitude_sp(src.altitude_sp())

            airspeed(src.airspeed())

            airspeed_sp(src.airspeed_sp())

            groundspeed(src.groundspeed())

            climb_rate(src.climb_rate())

            gps_nsat(src.gps_nsat())

            src.gps_fix_type()?.let { item ->
                gps_fix_type(item.get())
            }

            battery_remaining(src.battery_remaining())

            temperature(src.temperature())

            temperature_air(src.temperature_air())

            failsafe(src.failsafe())

            wp_num(src.wp_num())

            wp_distance(src.wp_distance())

        }

        inline fun pull(dst: HIGH_LATENCY, base_mode_exist: () -> Boolean, base_mode: () -> com.company.demo.GroundControl.MAV_MODE_FLAG, custom_mode: () -> Int, landed_state_exist: () -> Boolean, landed_state: () -> com.company.demo.GroundControl.MAV_LANDED_STATE, roll: () -> Short, pitch: () -> Short, heading: () -> Short, throttle: () -> Byte, heading_sp: () -> Short, latitude: () -> Int, longitude: () -> Int, altitude_amsl: () -> Short, altitude_sp: () -> Short, airspeed: () -> Byte, airspeed_sp: () -> Byte, groundspeed: () -> Byte, climb_rate: () -> Byte, gps_nsat: () -> Byte, gps_fix_type_exist: () -> Boolean, gps_fix_type: () -> com.company.demo.GroundControl.GPS_FIX_TYPE, battery_remaining: () -> Byte, temperature: () -> Byte, temperature_air: () -> Byte, failsafe: () -> Byte, wp_num: () -> Byte, wp_distance: () -> Short
        ) {

            if (base_mode_exist()) dst.base_mode(base_mode())


            dst.custom_mode(custom_mode())

            if (landed_state_exist()) dst.landed_state(landed_state())


            dst.roll(roll())

            dst.pitch(pitch())

            dst.heading(heading())

            dst.throttle(throttle())

            dst.heading_sp(heading_sp())

            dst.latitude(latitude())

            dst.longitude(longitude())

            dst.altitude_amsl(altitude_amsl())

            dst.altitude_sp(altitude_sp())

            dst.airspeed(airspeed())

            dst.airspeed_sp(airspeed_sp())

            dst.groundspeed(groundspeed())

            dst.climb_rate(climb_rate())

            dst.gps_nsat(gps_nsat())

            if (gps_fix_type_exist()) dst.gps_fix_type(gps_fix_type())


            dst.battery_remaining(battery_remaining())

            dst.temperature(temperature())

            dst.temperature_air(temperature_air())

            dst.failsafe(failsafe())

            dst.wp_num(wp_num())

            dst.wp_distance(wp_distance())

        }

    }
}

inline class PARAM_REQUEST_READ(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun param_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun param_id(): PARAM_REQUEST_READ_param_id? {
        if ((data.field_bit != 34 && !data.set_field(34, -1))) return null

        return PARAM_REQUEST_READ_param_id(data)
    }


    object param_id {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(166, 0, 0, 0, 5, 1, 34, 2, 1)


        inline fun push(src: PARAM_REQUEST_READ, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, param_id: (src: PARAM_REQUEST_READ_param_id) -> Unit, param_index: (src: Short) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.param_id()?.let { item -> param_id(item) }

            param_index(src.param_index())

        }

    }
}

inline class SET_ATTITUDE_TARGET(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }


    inline fun type_mask(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }


    inline fun q(): SET_ATTITUDE_TARGET_q {

        return SET_ATTITUDE_TARGET_q(data)
    }

    object q {
        const val item_len = 4


    }

    inline fun body_roll_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 23, 4).toInt())
    }


    inline fun body_pitch_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 27, 4).toInt())
    }


    inline fun body_yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 31, 4).toInt())
    }


    inline fun thrust(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 35, 4).toInt())
    }


    companion object {

        val meta = Meta(74, 0, 1, 0, 39, 1, 312)


        inline fun push(src: SET_ATTITUDE_TARGET, time_boot_ms: (src: Int) -> Unit, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, type_mask: (src: Byte) -> Unit, q: (src: SET_ATTITUDE_TARGET_q) -> Unit, body_roll_rate: (src: Float) -> Unit, body_pitch_rate: (src: Float) -> Unit, body_yaw_rate: (src: Float) -> Unit, thrust: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            target_system(src.target_system())

            target_component(src.target_component())

            type_mask(src.type_mask())
            src.q().let { item ->
                q(item)
            }

            body_roll_rate(src.body_roll_rate())

            body_pitch_rate(src.body_pitch_rate())

            body_yaw_rate(src.body_yaw_rate())

            thrust(src.thrust())

        }

    }
}

inline class FOLLOW_TARGET(val data: Pack.Bytes) {

    inline fun timestamp(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun timestamp_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun custom_state(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun custom_state_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }

    inline fun est_capabilities(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }

    inline fun est_capabilities_(src: Byte) {
        data.bytes[data.origin + 16] = (src).toByte()
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 17, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 21, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 21)
    }

    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 25, 4).toInt())
    }

    inline fun alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 25)
    }

    inline fun vel(): FOLLOW_TARGET_vel {

        return FOLLOW_TARGET_vel(data)
    }

    inline fun vel_(src: FloatArray) {
        val len = minOf(src.size, 3)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 29 + index * 4)
    }

    object vel {
        const val item_len = 3


    }

    inline fun acc(): FOLLOW_TARGET_acc {

        return FOLLOW_TARGET_acc(data)
    }

    inline fun acc_(src: FloatArray) {
        val len = minOf(src.size, 3)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 41 + index * 4)
    }

    object acc {
        const val item_len = 3


    }

    inline fun attitude_q(): FOLLOW_TARGET_attitude_q {

        return FOLLOW_TARGET_attitude_q(data)
    }

    inline fun attitude_q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 53 + index * 4)
    }

    object attitude_q {
        const val item_len = 4


    }

    inline fun rates(): FOLLOW_TARGET_rates {

        return FOLLOW_TARGET_rates(data)
    }

    inline fun rates_(src: FloatArray) {
        val len = minOf(src.size, 3)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 69 + index * 4)
    }

    object rates {
        const val item_len = 3


    }

    inline fun position_cov(): FOLLOW_TARGET_position_cov {

        return FOLLOW_TARGET_position_cov(data)
    }

    inline fun position_cov_(src: FloatArray) {
        val len = minOf(src.size, 3)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 81 + index * 4)
    }

    object position_cov {
        const val item_len = 3


    }


    companion object {

        val meta = Meta(39, 0, 0, 2, 93, 1, 744)


        inline fun push(src: FOLLOW_TARGET, timestamp: (src: Long) -> Unit, est_capabilities: (src: Byte) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Float) -> Unit, vel: (src: FOLLOW_TARGET_vel) -> Unit, acc: (src: FOLLOW_TARGET_acc) -> Unit, attitude_q: (src: FOLLOW_TARGET_attitude_q) -> Unit, rates: (src: FOLLOW_TARGET_rates) -> Unit, position_cov: (src: FOLLOW_TARGET_position_cov) -> Unit, custom_state: (src: Long) -> Unit
        ) {

            timestamp(src.timestamp())

            est_capabilities(src.est_capabilities())

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())
            src.vel().let { item ->
                vel(item)
            }
            src.acc().let { item ->
                acc(item)
            }
            src.attitude_q().let { item ->
                attitude_q(item)
            }
            src.rates().let { item ->
                rates(item)
            }
            src.position_cov().let { item ->
                position_cov(item)
            }

            custom_state(src.custom_state())

        }

        inline fun pull(dst: FOLLOW_TARGET, timestamp: () -> Long, est_capabilities: () -> Byte, lat: () -> Int, lon: () -> Int, alt: () -> Float, vel: (dst: FOLLOW_TARGET_vel) -> Unit, acc: (dst: FOLLOW_TARGET_acc) -> Unit, attitude_q: (dst: FOLLOW_TARGET_attitude_q) -> Unit, rates: (dst: FOLLOW_TARGET_rates) -> Unit, position_cov: (dst: FOLLOW_TARGET_position_cov) -> Unit, custom_state: () -> Long
        ) {

            dst.timestamp(timestamp())

            dst.est_capabilities(est_capabilities())

            dst.lat(lat())

            dst.lon(lon())

            dst.alt(alt())
            vel(dst.vel())
            acc(dst.acc())
            attitude_q(dst.attitude_q())
            rates(dst.rates())
            position_cov(dst.position_cov())

            dst.custom_state(custom_state())

        }

    }
}

inline class HIL_STATE(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 32, 4)).toInt()
    }


    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 36, 4)).toInt()
    }


    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 40, 4)).toInt()
    }


    inline fun vx(): Short {
        return (get_bytes(data.bytes, data.origin + 44, 2)).toShort()
    }


    inline fun vy(): Short {
        return (get_bytes(data.bytes, data.origin + 46, 2)).toShort()
    }


    inline fun vz(): Short {
        return (get_bytes(data.bytes, data.origin + 48, 2)).toShort()
    }


    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 50, 2)).toShort()
    }


    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 52, 2)).toShort()
    }


    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 54, 2)).toShort()
    }


    companion object {

        val meta = Meta(140, 0, 0, 1, 56, 1, 448)


        inline fun push(src: HIL_STATE, time_usec: (src: Long) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit, rollspeed: (src: Float) -> Unit, pitchspeed: (src: Float) -> Unit, yawspeed: (src: Float) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Int) -> Unit, vx: (src: Short) -> Unit, vy: (src: Short) -> Unit, vz: (src: Short) -> Unit, xacc: (src: Short) -> Unit, yacc: (src: Short) -> Unit, zacc: (src: Short) -> Unit
        ) {

            time_usec(src.time_usec())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

            rollspeed(src.rollspeed())

            pitchspeed(src.pitchspeed())

            yawspeed(src.yawspeed())

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

        }

    }
}

inline class HOME_POSITION(val data: Cursor) {

    inline fun latitude(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun latitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun longitude(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun longitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun altitude(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }

    inline fun altitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun q(): HOME_POSITION_q {

        return HOME_POSITION_q(data)
    }

    inline fun q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 24 + index * 4)
    }

    object q {
        const val item_len = 4


    }

    inline fun approach_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun approach_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun approach_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun approach_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }

    inline fun approach_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48, 4).toInt())
    }

    inline fun approach_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 48)
    }


    inline fun time_usec(): Boolean {
        return !(data.field_bit != 416 && !data.set_field(416, -1))
    }


    inline fun time_usec_() {
        if (data.field_bit != 416) data.set_field(416, 0)
    }


    companion object {

        val meta = Meta(210, 0, 0, 0, 53, 1, 416, 0, 1)


        inline fun push(src: HOME_POSITION, latitude: (src: Int) -> Unit, longitude: (src: Int) -> Unit, altitude: (src: Int) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, q: (src: HOME_POSITION_q) -> Unit, approach_x: (src: Float) -> Unit, approach_y: (src: Float) -> Unit, approach_z: (src: Float) -> Unit, time_usec: (src: Long) -> Unit
        ) {

            latitude(src.latitude())

            longitude(src.longitude())

            altitude(src.altitude())

            x(src.x())

            y(src.y())

            z(src.z())
            src.q().let { item ->
                q(item)
            }

            approach_x(src.approach_x())

            approach_y(src.approach_y())

            approach_z(src.approach_z())

            src.time_usec().let { item ->
                time_usec(item.get())
            }

        }

        inline fun pull(dst: HOME_POSITION, latitude: () -> Int, longitude: () -> Int, altitude: () -> Int, x: () -> Float, y: () -> Float, z: () -> Float, q: (dst: HOME_POSITION_q) -> Unit, approach_x: () -> Float, approach_y: () -> Float, approach_z: () -> Float, time_usec_exist: () -> Boolean, time_usec: () -> Long
        ) {

            dst.latitude(latitude())

            dst.longitude(longitude())

            dst.altitude(altitude())

            dst.x(x())

            dst.y(y())

            dst.z(z())
            q(dst.q())

            dst.approach_x(approach_x())

            dst.approach_y(approach_y())

            dst.approach_z(approach_z())

            if (time_usec_exist()) dst.time_usec(time_usec())


        }

    }
}

inline class FENCE_STATUS(val data: Cursor) {

    inline fun breach_count(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun breach_count_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun breach_time(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun breach_time_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun breach_status(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun breach_status_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }


    inline fun breach_type(): FENCE_STATUS_breach_type? {
        if ((data.field_bit != 56 && !data.set_field(56, -1))) return null

        return FENCE_STATUS_breach_type(data)
    }


    inline fun breach_type(src: FENCE_BREACH) {
        if (data.field_bit != 56) data.set_field(56, 0)

        set_bits((src.value).toULong().toLong(), 2, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(93, 1, 1, 0, 8, 1, 56, 0, 1)


        inline fun push(src: FENCE_STATUS, breach_status: (src: Byte) -> Unit, breach_count: (src: Short) -> Unit, breach_type: (src: com.company.demo.GroundControl.FENCE_BREACH) -> Unit, breach_time: (src: Int) -> Unit
        ) {

            breach_status(src.breach_status())

            breach_count(src.breach_count())

            src.breach_type()?.let { item ->
                breach_type(item.get())
            }

            breach_time(src.breach_time())

        }

        inline fun pull(dst: FENCE_STATUS, breach_status: () -> Byte, breach_count: () -> Short, breach_type_exist: () -> Boolean, breach_type: () -> com.company.demo.GroundControl.FENCE_BREACH, breach_time: () -> Int
        ) {

            dst.breach_status(breach_status())

            dst.breach_count(breach_count())

            if (breach_type_exist()) dst.breach_type(breach_type())


            dst.breach_time(breach_time())

        }

    }
}

inline class REMOTE_LOG_BLOCK_STATUS(val data: Cursor) {

    inline fun seqno(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun seqno_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }


    inline fun status(): REMOTE_LOG_BLOCK_STATUS_status? {
        if ((data.field_bit != 48 && !data.set_field(48, -1))) return null

        return REMOTE_LOG_BLOCK_STATUS_status(data)
    }


    inline fun status(src: MAV_REMOTE_LOG_DATA_BLOCK_STATUSES) {
        if (data.field_bit != 48) data.set_field(48, 0)

        set_bits((src.value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(171, 0, 1, 0, 7, 1, 48, 0, 1)


        inline fun push(src: REMOTE_LOG_BLOCK_STATUS, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, seqno: (src: Int) -> Unit, status: (src: com.company.demo.GroundControl.MAV_REMOTE_LOG_DATA_BLOCK_STATUSES) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            seqno(src.seqno())

            src.status()?.let { item ->
                status(item.get())
            }

        }

        inline fun pull(dst: REMOTE_LOG_BLOCK_STATUS, target_system: () -> Byte, target_component: () -> Byte, seqno: () -> Int, status_exist: () -> Boolean, status: () -> com.company.demo.GroundControl.MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.seqno(seqno())

            if (status_exist()) dst.status(status())


        }

    }
}

inline class OBSTACLE_DISTANCE(val data: Cursor) {

    inline fun distances(): OBSTACLE_DISTANCE_distances {

        return OBSTACLE_DISTANCE_distances(data)
    }

    inline fun distances_(src: ShortArray) {
        val len = minOf(src.size, 72)

        for (index in 0 until len)
            set_bytes((src[index]).toULong().toLong(), 2, data.bytes, data.origin + 0 + index * 2)
    }

    object distances {
        const val item_len = 72


    }

    inline fun min_distance(): Short {
        return (get_bytes(data.bytes, data.origin + 144, 2)).toShort()
    }

    inline fun min_distance_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 144)
    }

    inline fun max_distance(): Short {
        return (get_bytes(data.bytes, data.origin + 146, 2)).toShort()
    }

    inline fun max_distance_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 146)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 148, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 148)
    }

    inline fun increment(): Byte {
        return (data.bytes[data.origin + 156]).toByte()
    }

    inline fun increment_(src: Byte) {
        data.bytes[data.origin + 156] = (src).toByte()
    }


    inline fun sensor_type(): OBSTACLE_DISTANCE_sensor_type? {
        if ((data.field_bit != 1256 && !data.set_field(1256, -1))) return null

        return OBSTACLE_DISTANCE_sensor_type(data)
    }


    inline fun sensor_type(src: MAV_DISTANCE_SENSOR) {
        if (data.field_bit != 1256) data.set_field(1256, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(66, 74, 0, 1, 158, 1, 1256, 0, 1)


        inline fun push(src: OBSTACLE_DISTANCE, time_usec: (src: Long) -> Unit, sensor_type: (src: com.company.demo.GroundControl.MAV_DISTANCE_SENSOR) -> Unit, distances: (src: OBSTACLE_DISTANCE_distances) -> Unit, increment: (src: Byte) -> Unit, min_distance: (src: Short) -> Unit, max_distance: (src: Short) -> Unit
        ) {

            time_usec(src.time_usec())

            src.sensor_type()?.let { item ->
                sensor_type(item.get())
            }
            src.distances().let { item ->
                distances(item)
            }

            increment(src.increment())

            min_distance(src.min_distance())

            max_distance(src.max_distance())

        }

        inline fun pull(dst: OBSTACLE_DISTANCE, time_usec: () -> Long, sensor_type_exist: () -> Boolean, sensor_type: () -> com.company.demo.GroundControl.MAV_DISTANCE_SENSOR, distances: (dst: OBSTACLE_DISTANCE_distances) -> Unit, increment: () -> Byte, min_distance: () -> Short, max_distance: () -> Short
        ) {

            dst.time_usec(time_usec())

            if (sensor_type_exist()) dst.sensor_type(sensor_type())

            distances(dst.distances())

            dst.increment(increment())

            dst.min_distance(min_distance())

            dst.max_distance(max_distance())

        }

    }
}

inline class GPS2_RAW(val data: Cursor) {

    inline fun eph(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun eph_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun epv(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun epv_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun vel(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun vel_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun cog(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun cog_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun dgps_age(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }

    inline fun dgps_age_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 12, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 12)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 24, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 28, 4)).toInt()
    }

    inline fun alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun satellites_visible(): Byte {
        return (data.bytes[data.origin + 32]).toByte()
    }

    inline fun satellites_visible_(src: Byte) {
        data.bytes[data.origin + 32] = (src).toByte()
    }

    inline fun dgps_numch(): Byte {
        return (data.bytes[data.origin + 33]).toByte()
    }

    inline fun dgps_numch_(src: Byte) {
        data.bytes[data.origin + 33] = (src).toByte()
    }


    inline fun fix_type(): GPS2_RAW_fix_type? {
        if ((data.field_bit != 272 && !data.set_field(272, -1))) return null

        return GPS2_RAW_fix_type(data)
    }


    inline fun fix_type(src: GPS_FIX_TYPE) {
        if (data.field_bit != 272) data.set_field(272, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(213, 4, 1, 1, 35, 1, 272, 0, 1)


        inline fun push(src: GPS2_RAW, time_usec: (src: Long) -> Unit, fix_type: (src: com.company.demo.GroundControl.GPS_FIX_TYPE) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Int) -> Unit, eph: (src: Short) -> Unit, epv: (src: Short) -> Unit, vel: (src: Short) -> Unit, cog: (src: Short) -> Unit, satellites_visible: (src: Byte) -> Unit, dgps_numch: (src: Byte) -> Unit, dgps_age: (src: Int) -> Unit
        ) {

            time_usec(src.time_usec())

            src.fix_type()?.let { item ->
                fix_type(item.get())
            }

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            eph(src.eph())

            epv(src.epv())

            vel(src.vel())

            cog(src.cog())

            satellites_visible(src.satellites_visible())

            dgps_numch(src.dgps_numch())

            dgps_age(src.dgps_age())

        }

        inline fun pull(dst: GPS2_RAW, time_usec: () -> Long, fix_type_exist: () -> Boolean, fix_type: () -> com.company.demo.GroundControl.GPS_FIX_TYPE, lat: () -> Int, lon: () -> Int, alt: () -> Int, eph: () -> Short, epv: () -> Short, vel: () -> Short, cog: () -> Short, satellites_visible: () -> Byte, dgps_numch: () -> Byte, dgps_age: () -> Int
        ) {

            dst.time_usec(time_usec())

            if (fix_type_exist()) dst.fix_type(fix_type())


            dst.lat(lat())

            dst.lon(lon())

            dst.alt(alt())

            dst.eph(eph())

            dst.epv(epv())

            dst.vel(vel())

            dst.cog(cog())

            dst.satellites_visible(satellites_visible())

            dst.dgps_numch(dgps_numch())

            dst.dgps_age(dgps_age())

        }

    }
}

inline class REQUEST_DATA_STREAM(val data: Pack.Bytes) {

    inline fun req_message_rate(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    inline fun req_stream_id(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun start_stop(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }


    companion object {

        val meta = Meta(98, 1, 0, 0, 6, 1, 48)


        inline fun push(src: REQUEST_DATA_STREAM, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, req_stream_id: (src: Byte) -> Unit, req_message_rate: (src: Short) -> Unit, start_stop: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            req_stream_id(src.req_stream_id())

            req_message_rate(src.req_message_rate())

            start_stop(src.start_stop())

        }

    }
}

inline class MEMORY_VECT(val data: Pack.Bytes) {

    inline fun address(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun address_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun ver(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun ver_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun typE(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun typE_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun value(): MEMORY_VECT_value {

        return MEMORY_VECT_value(data)
    }

    inline fun value_(src: ByteArray) {
        val len = minOf(src.size, 32)

        for (index in 0 until len)
            data.bytes[data.origin + 4 + index] = (src[index]).toByte()
    }

    object value {
        const val item_len = 32


    }


    companion object {

        val meta = Meta(30, 1, 0, 0, 36, 1, 288)


        inline fun push(src: MEMORY_VECT, address: (src: Short) -> Unit, ver: (src: Byte) -> Unit, typE: (src: Byte) -> Unit, value: (src: MEMORY_VECT_value) -> Unit
        ) {

            address(src.address())

            ver(src.ver())

            typE(src.typE())
            src.value().let { item ->
                value(item)
            }

        }

        inline fun pull(dst: MEMORY_VECT, address: () -> Short, ver: () -> Byte, typE: () -> Byte, value: (dst: MEMORY_VECT_value) -> Unit
        ) {

            dst.address(address())

            dst.ver(ver())

            dst.typE(typE())
            value(dst.value())

        }

    }
}

inline class PARAM_EXT_REQUEST_READ(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun param_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun param_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }


    inline fun param_id(): PARAM_EXT_REQUEST_READ_param_id? {
        if ((data.field_bit != 34 && !data.set_field(34, -1))) return null

        return PARAM_EXT_REQUEST_READ_param_id(data)
    }


    inline fun param_id_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -34 - 1, reuse)
    }


    inline fun param_id_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(34, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_id_(len: Int): PARAM_EXT_REQUEST_READ_param_id {

        data.set_field(34, minOf(len, 255))
        return PARAM_EXT_REQUEST_READ_param_id(data)
    }

    object param_id {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(90, 0, 0, 0, 5, 1, 34, 2, 1)


        inline fun push(src: PARAM_EXT_REQUEST_READ, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, param_id: (src: PARAM_EXT_REQUEST_READ_param_id) -> Unit, param_index: (src: Short) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.param_id()?.let { item -> param_id(item) }

            param_index(src.param_index())

        }

        inline fun pull(dst: PARAM_EXT_REQUEST_READ, target_system: () -> Byte, target_component: () -> Byte, param_id_exist: () -> Int, param_id: (dst: PARAM_EXT_REQUEST_READ_param_id) -> Unit, param_index: () -> Short
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())


            param_id_exist().let { len ->
                if (0 < len)
                    param_id(dst.param_id(len))
            }


            dst.param_index(param_index())

        }

    }
}

inline class HIL_CONTROLS(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun roll_ailerons(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun pitch_elevator(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun yaw_rudder(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun throttle(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun aux1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun aux2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    inline fun aux3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }


    inline fun aux4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }


    inline fun nav_mode(): Byte {
        return (data.bytes[data.origin + 40]).toByte()
    }


    inline fun mode(): HIL_CONTROLS_mode? {
        if ((data.field_bit != 328 && !data.set_field(328, -1))) return null

        return HIL_CONTROLS_mode(data)
    }


    companion object {

        val meta = Meta(82, 0, 0, 1, 42, 1, 328, 0, 1)


        inline fun push(src: HIL_CONTROLS, time_usec: (src: Long) -> Unit, roll_ailerons: (src: Float) -> Unit, pitch_elevator: (src: Float) -> Unit, yaw_rudder: (src: Float) -> Unit, throttle: (src: Float) -> Unit, aux1: (src: Float) -> Unit, aux2: (src: Float) -> Unit, aux3: (src: Float) -> Unit, aux4: (src: Float) -> Unit, mode: (src: com.company.demo.GroundControl.MAV_MODE) -> Unit, nav_mode: (src: Byte) -> Unit
        ) {

            time_usec(src.time_usec())

            roll_ailerons(src.roll_ailerons())

            pitch_elevator(src.pitch_elevator())

            yaw_rudder(src.yaw_rudder())

            throttle(src.throttle())

            aux1(src.aux1())

            aux2(src.aux2())

            aux3(src.aux3())

            aux4(src.aux4())

            src.mode()?.let { item ->
                mode(item.get())
            }

            nav_mode(src.nav_mode())

        }

    }
}

inline class HIL_SENSOR(val data: Pack.Bytes) {

    inline fun fields_updated(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun fields_updated_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 4)
    }

    inline fun xacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun xacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun yacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun yacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun zacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun zacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun xgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun xgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun ygyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun ygyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun zgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun zgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun xmag(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun xmag_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun ymag(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun ymag_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun zmag(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun zmag_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }

    inline fun abs_pressure(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48, 4).toInt())
    }

    inline fun abs_pressure_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 48)
    }

    inline fun diff_pressure(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 52, 4).toInt())
    }

    inline fun diff_pressure_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 52)
    }

    inline fun pressure_alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 56, 4).toInt())
    }

    inline fun pressure_alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 56)
    }

    inline fun temperature(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 60, 4).toInt())
    }

    inline fun temperature_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 60)
    }


    companion object {

        val meta = Meta(169, 0, 1, 1, 64, 1, 512)


        inline fun push(src: HIL_SENSOR, time_usec: (src: Long) -> Unit, xacc: (src: Float) -> Unit, yacc: (src: Float) -> Unit, zacc: (src: Float) -> Unit, xgyro: (src: Float) -> Unit, ygyro: (src: Float) -> Unit, zgyro: (src: Float) -> Unit, xmag: (src: Float) -> Unit, ymag: (src: Float) -> Unit, zmag: (src: Float) -> Unit, abs_pressure: (src: Float) -> Unit, diff_pressure: (src: Float) -> Unit, pressure_alt: (src: Float) -> Unit, temperature: (src: Float) -> Unit, fields_updated: (src: Int) -> Unit
        ) {

            time_usec(src.time_usec())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

            xgyro(src.xgyro())

            ygyro(src.ygyro())

            zgyro(src.zgyro())

            xmag(src.xmag())

            ymag(src.ymag())

            zmag(src.zmag())

            abs_pressure(src.abs_pressure())

            diff_pressure(src.diff_pressure())

            pressure_alt(src.pressure_alt())

            temperature(src.temperature())

            fields_updated(src.fields_updated())

        }

        inline fun pull(dst: HIL_SENSOR, time_usec: () -> Long, xacc: () -> Float, yacc: () -> Float, zacc: () -> Float, xgyro: () -> Float, ygyro: () -> Float, zgyro: () -> Float, xmag: () -> Float, ymag: () -> Float, zmag: () -> Float, abs_pressure: () -> Float, diff_pressure: () -> Float, pressure_alt: () -> Float, temperature: () -> Float, fields_updated: () -> Int
        ) {

            dst.time_usec(time_usec())

            dst.xacc(xacc())

            dst.yacc(yacc())

            dst.zacc(zacc())

            dst.xgyro(xgyro())

            dst.ygyro(ygyro())

            dst.zgyro(zgyro())

            dst.xmag(xmag())

            dst.ymag(ymag())

            dst.zmag(zmag())

            dst.abs_pressure(abs_pressure())

            dst.diff_pressure(diff_pressure())

            dst.pressure_alt(pressure_alt())

            dst.temperature(temperature())

            dst.fields_updated(fields_updated())

        }

    }
}

inline class SETUP_SIGNING(val data: Pack.Bytes) {

    inline fun initial_timestamp(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun initial_timestamp_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 9]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 9] = (src).toByte()
    }

    inline fun secret_key(): SETUP_SIGNING_secret_key {

        return SETUP_SIGNING_secret_key(data)
    }

    inline fun secret_key_(src: ByteArray) {
        val len = minOf(src.size, 32)

        for (index in 0 until len)
            data.bytes[data.origin + 10 + index] = (src[index]).toByte()
    }

    object secret_key {
        const val item_len = 32


    }


    companion object {

        val meta = Meta(205, 0, 0, 1, 42, 1, 336)


        inline fun push(src: SETUP_SIGNING, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, secret_key: (src: SETUP_SIGNING_secret_key) -> Unit, initial_timestamp: (src: Long) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())
            src.secret_key().let { item ->
                secret_key(item)
            }

            initial_timestamp(src.initial_timestamp())

        }

        inline fun pull(dst: SETUP_SIGNING, target_system: () -> Byte, target_component: () -> Byte, secret_key: (dst: SETUP_SIGNING_secret_key) -> Unit, initial_timestamp: () -> Long
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())
            secret_key(dst.secret_key())

            dst.initial_timestamp(initial_timestamp())

        }

    }
}

inline class GPS_RTK(val data: Pack.Bytes) {

    inline fun wn(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun wn_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_last_baseline_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun time_last_baseline_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun tow(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun tow_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun accuracy(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun accuracy_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun rtk_receiver_id(): Byte {
        return (data.bytes[data.origin + 14]).toByte()
    }

    inline fun rtk_receiver_id_(src: Byte) {
        data.bytes[data.origin + 14] = (src).toByte()
    }

    inline fun rtk_health(): Byte {
        return (data.bytes[data.origin + 15]).toByte()
    }

    inline fun rtk_health_(src: Byte) {
        data.bytes[data.origin + 15] = (src).toByte()
    }

    inline fun rtk_rate(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }

    inline fun rtk_rate_(src: Byte) {
        data.bytes[data.origin + 16] = (src).toByte()
    }

    inline fun nsats(): Byte {
        return (data.bytes[data.origin + 17]).toByte()
    }

    inline fun nsats_(src: Byte) {
        data.bytes[data.origin + 17] = (src).toByte()
    }

    inline fun baseline_coords_type(): Byte {
        return (data.bytes[data.origin + 18]).toByte()
    }

    inline fun baseline_coords_type_(src: Byte) {
        data.bytes[data.origin + 18] = (src).toByte()
    }

    inline fun baseline_a_mm(): Int {
        return (get_bytes(data.bytes, data.origin + 19, 4)).toInt()
    }

    inline fun baseline_a_mm_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 19)
    }

    inline fun baseline_b_mm(): Int {
        return (get_bytes(data.bytes, data.origin + 23, 4)).toInt()
    }

    inline fun baseline_b_mm_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 23)
    }

    inline fun baseline_c_mm(): Int {
        return (get_bytes(data.bytes, data.origin + 27, 4)).toInt()
    }

    inline fun baseline_c_mm_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 27)
    }

    inline fun iar_num_hypotheses(): Int {
        return (get_bytes(data.bytes, data.origin + 31, 4)).toInt()
    }

    inline fun iar_num_hypotheses_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 31)
    }


    companion object {

        val meta = Meta(89, 1, 3, 0, 35, 1, 280)


        inline fun push(src: GPS_RTK, time_last_baseline_ms: (src: Int) -> Unit, rtk_receiver_id: (src: Byte) -> Unit, wn: (src: Short) -> Unit, tow: (src: Int) -> Unit, rtk_health: (src: Byte) -> Unit, rtk_rate: (src: Byte) -> Unit, nsats: (src: Byte) -> Unit, baseline_coords_type: (src: Byte) -> Unit, baseline_a_mm: (src: Int) -> Unit, baseline_b_mm: (src: Int) -> Unit, baseline_c_mm: (src: Int) -> Unit, accuracy: (src: Int) -> Unit, iar_num_hypotheses: (src: Int) -> Unit
        ) {

            time_last_baseline_ms(src.time_last_baseline_ms())

            rtk_receiver_id(src.rtk_receiver_id())

            wn(src.wn())

            tow(src.tow())

            rtk_health(src.rtk_health())

            rtk_rate(src.rtk_rate())

            nsats(src.nsats())

            baseline_coords_type(src.baseline_coords_type())

            baseline_a_mm(src.baseline_a_mm())

            baseline_b_mm(src.baseline_b_mm())

            baseline_c_mm(src.baseline_c_mm())

            accuracy(src.accuracy())

            iar_num_hypotheses(src.iar_num_hypotheses())

        }

        inline fun pull(dst: GPS_RTK, time_last_baseline_ms: () -> Int, rtk_receiver_id: () -> Byte, wn: () -> Short, tow: () -> Int, rtk_health: () -> Byte, rtk_rate: () -> Byte, nsats: () -> Byte, baseline_coords_type: () -> Byte, baseline_a_mm: () -> Int, baseline_b_mm: () -> Int, baseline_c_mm: () -> Int, accuracy: () -> Int, iar_num_hypotheses: () -> Int
        ) {

            dst.time_last_baseline_ms(time_last_baseline_ms())

            dst.rtk_receiver_id(rtk_receiver_id())

            dst.wn(wn())

            dst.tow(tow())

            dst.rtk_health(rtk_health())

            dst.rtk_rate(rtk_rate())

            dst.nsats(nsats())

            dst.baseline_coords_type(baseline_coords_type())

            dst.baseline_a_mm(baseline_a_mm())

            dst.baseline_b_mm(baseline_b_mm())

            dst.baseline_c_mm(baseline_c_mm())

            dst.accuracy(accuracy())

            dst.iar_num_hypotheses(iar_num_hypotheses())

        }

    }
}

inline class PARAM_REQUEST_LIST(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    companion object {

        val meta = Meta(47, 0, 0, 0, 2, 1, 16)


        inline fun push(src: PARAM_REQUEST_LIST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

        }

    }
}

inline class UAVIONIX_ADSB_OUT_CFG(val data: Cursor) {

    inline fun stallSpeed(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun stallSpeed_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun ICAO(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun ICAO_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }


    inline fun callsign(): UAVIONIX_ADSB_OUT_CFG_callsign? {
        if ((data.field_bit != 51 && !data.set_field(51, -1))) return null

        return UAVIONIX_ADSB_OUT_CFG_callsign(data)
    }


    inline fun callsign_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -51 - 1, reuse)
    }


    inline fun callsign_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(51, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun callsign_(len: Int): UAVIONIX_ADSB_OUT_CFG_callsign {

        data.set_field(51, minOf(len, 255))
        return UAVIONIX_ADSB_OUT_CFG_callsign(data)
    }

    object callsign {
        const val item_len_max = 255

    }


    inline fun emitterType(): UAVIONIX_ADSB_OUT_CFG_emitterType? {
        if ((data.field_bit != 52 && !data.set_field(52, -1))) return null

        return UAVIONIX_ADSB_OUT_CFG_emitterType(data)
    }


    inline fun emitterType(src: ADSB_EMITTER_TYPE) {
        if (data.field_bit != 52) data.set_field(52, 0)

        set_bits((src.value).toULong().toLong(), 5, data.bytes, data.BIT)
    }


    inline fun aircraftSize(): UAVIONIX_ADSB_OUT_CFG_aircraftSize? {
        if ((data.field_bit != 53 && !data.set_field(53, -1))) return null

        return UAVIONIX_ADSB_OUT_CFG_aircraftSize(data)
    }


    inline fun aircraftSize(src: UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE) {
        if (data.field_bit != 53) data.set_field(53, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    inline fun gpsOffsetLat(): UAVIONIX_ADSB_OUT_CFG_gpsOffsetLat? {
        if ((data.field_bit != 54 && !data.set_field(54, -1))) return null

        return UAVIONIX_ADSB_OUT_CFG_gpsOffsetLat(data)
    }


    inline fun gpsOffsetLat(src: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT) {
        if (data.field_bit != 54) data.set_field(54, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun gpsOffsetLon(): UAVIONIX_ADSB_OUT_CFG_gpsOffsetLon? {
        if ((data.field_bit != 55 && !data.set_field(55, -1))) return null

        return UAVIONIX_ADSB_OUT_CFG_gpsOffsetLon(data)
    }


    inline fun gpsOffsetLon(src: UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON) {
        if (data.field_bit != 55) data.set_field(55, 0)

        set_bits((src.value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    inline fun rfSelect(): UAVIONIX_ADSB_OUT_CFG_rfSelect? {
        if ((data.field_bit != 56 && !data.set_field(56, -1))) return null

        return UAVIONIX_ADSB_OUT_CFG_rfSelect(data)
    }


    inline fun rfSelect(src: UAVIONIX_ADSB_OUT_RF_SELECT) {
        if (data.field_bit != 56) data.set_field(56, 0)

        set_bits((src.value).toULong().toLong(), 2, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(84, 1, 1, 0, 7, 1, 51, 3, 6)


        inline fun push(src: UAVIONIX_ADSB_OUT_CFG, ICAO: (src: Int) -> Unit, callsign: (src: UAVIONIX_ADSB_OUT_CFG_callsign) -> Unit, emitterType: (src: com.company.demo.GroundControl.ADSB_EMITTER_TYPE) -> Unit, aircraftSize: (src: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE) -> Unit, gpsOffsetLat: (src: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT) -> Unit, gpsOffsetLon: (src: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON) -> Unit, stallSpeed: (src: Short) -> Unit, rfSelect: (src: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_RF_SELECT) -> Unit
        ) {

            ICAO(src.ICAO())

            src.callsign()?.let { item -> callsign(item) }

            src.emitterType()?.let { item ->
                emitterType(item.get())
            }

            src.aircraftSize()?.let { item ->
                aircraftSize(item.get())
            }

            src.gpsOffsetLat()?.let { item ->
                gpsOffsetLat(item.get())
            }

            src.gpsOffsetLon()?.let { item ->
                gpsOffsetLon(item.get())
            }

            stallSpeed(src.stallSpeed())

            src.rfSelect()?.let { item ->
                rfSelect(item.get())
            }

        }

        inline fun pull(dst: UAVIONIX_ADSB_OUT_CFG, ICAO: () -> Int, callsign_exist: () -> Int, callsign: (dst: UAVIONIX_ADSB_OUT_CFG_callsign) -> Unit, emitterType_exist: () -> Boolean, emitterType: () -> com.company.demo.GroundControl.ADSB_EMITTER_TYPE, aircraftSize_exist: () -> Boolean, aircraftSize: () -> com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE, gpsOffsetLat_exist: () -> Boolean, gpsOffsetLat: () -> com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT, gpsOffsetLon_exist: () -> Boolean, gpsOffsetLon: () -> com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON, stallSpeed: () -> Short, rfSelect_exist: () -> Boolean, rfSelect: () -> com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_RF_SELECT
        ) {

            dst.ICAO(ICAO())


            callsign_exist().let { len ->
                if (0 < len)
                    callsign(dst.callsign(len))
            }


            if (emitterType_exist()) dst.emitterType(emitterType())


            if (aircraftSize_exist()) dst.aircraftSize(aircraftSize())


            if (gpsOffsetLat_exist()) dst.gpsOffsetLat(gpsOffsetLat())


            if (gpsOffsetLon_exist()) dst.gpsOffsetLon(gpsOffsetLon())


            dst.stallSpeed(stallSpeed())

            if (rfSelect_exist()) dst.rfSelect(rfSelect())


        }

    }
}

inline class LANDING_TARGET(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun target_num(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun target_num_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun angle_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 9, 4).toInt())
    }

    inline fun angle_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 9)
    }

    inline fun angle_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 13, 4).toInt())
    }

    inline fun angle_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 13)
    }

    inline fun distance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 17, 4).toInt())
    }

    inline fun distance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }

    inline fun size_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 21, 4).toInt())
    }

    inline fun size_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 21)
    }

    inline fun size_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 25, 4).toInt())
    }

    inline fun size_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 25)
    }


    inline fun frame(): LANDING_TARGET_frame? {
        if ((data.field_bit != 235 && !data.set_field(235, -1))) return null

        return LANDING_TARGET_frame(data)
    }


    inline fun frame(src: MAV_FRAME) {
        if (data.field_bit != 235) data.set_field(235, 0)

        set_bits((src.value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    inline fun x(): Boolean {
        return !(data.field_bit != 236 && !data.set_field(236, -1))
    }


    inline fun x_() {
        if (data.field_bit != 236) data.set_field(236, 0)
    }


    inline fun y(): Boolean {
        return !(data.field_bit != 237 && !data.set_field(237, -1))
    }


    inline fun y_() {
        if (data.field_bit != 237) data.set_field(237, 0)
    }


    inline fun z(): Boolean {
        return !(data.field_bit != 238 && !data.set_field(238, -1))
    }


    inline fun z_() {
        if (data.field_bit != 238) data.set_field(238, 0)
    }


    inline fun q(d0: Int) {
        if (data.field_bit != 239) data.set_field(239, 0)
        data.set_item(d0, 0)
    }


    inline fun q(): LANDING_TARGET_q? {
        return if (data.field_bit == 239 || data.set_field(239, -1)) LANDING_TARGET_q(data) else null
    }

    object q {
        const val d0: Int = 4

    }


    inline fun typE(): LANDING_TARGET_typE? {
        if ((data.field_bit != 240 && !data.set_field(240, -1))) return null

        return LANDING_TARGET_typE(data)
    }


    inline fun typE(src: LANDING_TARGET_TYPE) {
        if (data.field_bit != 240) data.set_field(240, 0)

        set_bits((src.value).toULong().toLong(), 2, data.bytes, data.BIT)
    }


    inline fun position_valid(): Boolean {
        return !(data.field_bit != 241 && !data.set_field(241, -1))
    }


    inline fun position_valid_() {
        if (data.field_bit != 241) data.set_field(241, 0)
    }


    companion object {

        val meta = Meta(172, 0, 0, 1, 30, 1, 235, 3, 7)


        inline fun push(src: LANDING_TARGET, time_usec: (src: Long) -> Unit, target_num: (src: Byte) -> Unit, frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, angle_x: (src: Float) -> Unit, angle_y: (src: Float) -> Unit, distance: (src: Float) -> Unit, size_x: (src: Float) -> Unit, size_y: (src: Float) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, q: (src: Float, d0: Int) -> Unit, typE: (src: com.company.demo.GroundControl.LANDING_TARGET_TYPE) -> Unit, position_valid: (src: Byte) -> Unit
        ) {

            time_usec(src.time_usec())

            target_num(src.target_num())

            src.frame()?.let { item ->
                frame(item.get())
            }

            angle_x(src.angle_x())

            angle_y(src.angle_y())

            distance(src.distance())

            size_x(src.size_x())

            size_y(src.size_y())

            src.x().let { item ->
                x(item.get())
            }

            src.y().let { item ->
                y(item.get())
            }

            src.z().let { item ->
                z(item.get())
            }

            src.q()?.let { fld ->
                fld.enumerate { d0 ->
                    q(item, d0)
                }
            }

            src.typE()?.let { item ->
                typE(item.get())
            }

            src.position_valid().let { item ->
                position_valid(item.get())
            }

        }

        inline fun pull(dst: LANDING_TARGET, time_usec: () -> Long, target_num: () -> Byte, frame_exist: () -> Boolean, frame: () -> com.company.demo.GroundControl.MAV_FRAME, angle_x: () -> Float, angle_y: () -> Float, distance: () -> Float, size_x: () -> Float, size_y: () -> Float, x_exist: () -> Boolean, x: () -> Float, y_exist: () -> Boolean, y: () -> Float, z_exist: () -> Boolean, z: () -> Float, q_exist: () -> Boolean, q_item_exist: (d0: Int) -> Boolean, q: (d0: Int) -> Float, typE_exist: () -> Boolean, typE: () -> com.company.demo.GroundControl.LANDING_TARGET_TYPE, position_valid_exist: () -> Boolean, position_valid: () -> Byte
        ) {

            dst.time_usec(time_usec())

            dst.target_num(target_num())

            if (frame_exist()) dst.frame(frame())


            dst.angle_x(angle_x())

            dst.angle_y(angle_y())

            dst.distance(distance())

            dst.size_x(size_x())

            dst.size_y(size_y())

            if (x_exist()) dst.x(x())


            if (y_exist()) dst.y(y())


            if (z_exist()) dst.z(z())


            if (q_exist())
                for (d0 in 0 until LANDING_TARGET.q.d0) {
                    if (q_item_exist(d0)) dst.q(q(d0), d0)

                }

            if (typE_exist()) dst.typE(typE())


            if (position_valid_exist()) dst.position_valid(position_valid())


        }

    }
}

inline class SET_ACTUATOR_CONTROL_TARGET(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun group_mlx(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun group_mlx_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 9]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 9] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun controls(): SET_ACTUATOR_CONTROL_TARGET_controls {

        return SET_ACTUATOR_CONTROL_TARGET_controls(data)
    }

    inline fun controls_(src: FloatArray) {
        val len = minOf(src.size, 8)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 11 + index * 4)
    }

    object controls {
        const val item_len = 8


    }


    companion object {

        val meta = Meta(94, 0, 0, 1, 43, 1, 344)


        inline fun push(src: SET_ACTUATOR_CONTROL_TARGET, time_usec: (src: Long) -> Unit, group_mlx: (src: Byte) -> Unit, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, controls: (src: SET_ACTUATOR_CONTROL_TARGET_controls) -> Unit
        ) {

            time_usec(src.time_usec())

            group_mlx(src.group_mlx())

            target_system(src.target_system())

            target_component(src.target_component())
            src.controls().let { item ->
                controls(item)
            }

        }

        inline fun pull(dst: SET_ACTUATOR_CONTROL_TARGET, time_usec: () -> Long, group_mlx: () -> Byte, target_system: () -> Byte, target_component: () -> Byte, controls: (dst: SET_ACTUATOR_CONTROL_TARGET_controls) -> Unit
        ) {

            dst.time_usec(time_usec())

            dst.group_mlx(group_mlx())

            dst.target_system(target_system())

            dst.target_component(target_component())
            controls(dst.controls())

        }

    }
}

inline class CONTROL_SYSTEM_STATE(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun x_acc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun x_acc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun y_acc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun y_acc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun z_acc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun z_acc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun x_vel(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun x_vel_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun y_vel(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun y_vel_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun z_vel(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun z_vel_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun x_pos(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun x_pos_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun y_pos(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun y_pos_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun z_pos(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun z_pos_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun airspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun airspeed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }

    inline fun vel_variance(): CONTROL_SYSTEM_STATE_vel_variance {

        return CONTROL_SYSTEM_STATE_vel_variance(data)
    }

    inline fun vel_variance_(src: FloatArray) {
        val len = minOf(src.size, 3)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 48 + index * 4)
    }

    object vel_variance {
        const val item_len = 3


    }

    inline fun pos_variance(): CONTROL_SYSTEM_STATE_pos_variance {

        return CONTROL_SYSTEM_STATE_pos_variance(data)
    }

    inline fun pos_variance_(src: FloatArray) {
        val len = minOf(src.size, 3)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 60 + index * 4)
    }

    object pos_variance {
        const val item_len = 3


    }

    inline fun q(): CONTROL_SYSTEM_STATE_q {

        return CONTROL_SYSTEM_STATE_q(data)
    }

    inline fun q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 72 + index * 4)
    }

    object q {
        const val item_len = 4


    }

    inline fun roll_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 88, 4).toInt())
    }

    inline fun roll_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 88)
    }

    inline fun pitch_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 92, 4).toInt())
    }

    inline fun pitch_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 92)
    }

    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 96, 4).toInt())
    }

    inline fun yaw_rate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 96)
    }


    companion object {

        val meta = Meta(207, 0, 0, 1, 100, 1, 800)


        inline fun push(src: CONTROL_SYSTEM_STATE, time_usec: (src: Long) -> Unit, x_acc: (src: Float) -> Unit, y_acc: (src: Float) -> Unit, z_acc: (src: Float) -> Unit, x_vel: (src: Float) -> Unit, y_vel: (src: Float) -> Unit, z_vel: (src: Float) -> Unit, x_pos: (src: Float) -> Unit, y_pos: (src: Float) -> Unit, z_pos: (src: Float) -> Unit, airspeed: (src: Float) -> Unit, vel_variance: (src: CONTROL_SYSTEM_STATE_vel_variance) -> Unit, pos_variance: (src: CONTROL_SYSTEM_STATE_pos_variance) -> Unit, q: (src: CONTROL_SYSTEM_STATE_q) -> Unit, roll_rate: (src: Float) -> Unit, pitch_rate: (src: Float) -> Unit, yaw_rate: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            x_acc(src.x_acc())

            y_acc(src.y_acc())

            z_acc(src.z_acc())

            x_vel(src.x_vel())

            y_vel(src.y_vel())

            z_vel(src.z_vel())

            x_pos(src.x_pos())

            y_pos(src.y_pos())

            z_pos(src.z_pos())

            airspeed(src.airspeed())
            src.vel_variance().let { item ->
                vel_variance(item)
            }
            src.pos_variance().let { item ->
                pos_variance(item)
            }
            src.q().let { item ->
                q(item)
            }

            roll_rate(src.roll_rate())

            pitch_rate(src.pitch_rate())

            yaw_rate(src.yaw_rate())

        }

        inline fun pull(dst: CONTROL_SYSTEM_STATE, time_usec: () -> Long, x_acc: () -> Float, y_acc: () -> Float, z_acc: () -> Float, x_vel: () -> Float, y_vel: () -> Float, z_vel: () -> Float, x_pos: () -> Float, y_pos: () -> Float, z_pos: () -> Float, airspeed: () -> Float, vel_variance: (dst: CONTROL_SYSTEM_STATE_vel_variance) -> Unit, pos_variance: (dst: CONTROL_SYSTEM_STATE_pos_variance) -> Unit, q: (dst: CONTROL_SYSTEM_STATE_q) -> Unit, roll_rate: () -> Float, pitch_rate: () -> Float, yaw_rate: () -> Float
        ) {

            dst.time_usec(time_usec())

            dst.x_acc(x_acc())

            dst.y_acc(y_acc())

            dst.z_acc(z_acc())

            dst.x_vel(x_vel())

            dst.y_vel(y_vel())

            dst.z_vel(z_vel())

            dst.x_pos(x_pos())

            dst.y_pos(y_pos())

            dst.z_pos(z_pos())

            dst.airspeed(airspeed())
            vel_variance(dst.vel_variance())
            pos_variance(dst.pos_variance())
            q(dst.q())

            dst.roll_rate(roll_rate())

            dst.pitch_rate(pitch_rate())

            dst.yaw_rate(yaw_rate())

        }

    }
}

inline class SET_POSITION_TARGET_GLOBAL_INT(val data: Cursor) {

    inline fun type_mask(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }


    inline fun lat_int(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }


    inline fun lon_int(): Int {
        return (get_bytes(data.bytes, data.origin + 12, 4)).toInt()
    }


    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    inline fun afx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }


    inline fun afy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }


    inline fun afz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }


    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48, 4).toInt())
    }


    inline fun coordinate_frame(): SET_POSITION_TARGET_GLOBAL_INT_coordinate_frame? {
        if ((data.field_bit != 416 && !data.set_field(416, -1))) return null

        return SET_POSITION_TARGET_GLOBAL_INT_coordinate_frame(data)
    }


    companion object {

        val meta = Meta(218, 1, 1, 0, 53, 1, 416, 0, 1)


        inline fun push(src: SET_POSITION_TARGET_GLOBAL_INT, time_boot_ms: (src: Int) -> Unit, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, coordinate_frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, type_mask: (src: Short) -> Unit, lat_int: (src: Int) -> Unit, lon_int: (src: Int) -> Unit, alt: (src: Float) -> Unit, vx: (src: Float) -> Unit, vy: (src: Float) -> Unit, vz: (src: Float) -> Unit, afx: (src: Float) -> Unit, afy: (src: Float) -> Unit, afz: (src: Float) -> Unit, yaw: (src: Float) -> Unit, yaw_rate: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            target_system(src.target_system())

            target_component(src.target_component())

            src.coordinate_frame()?.let { item ->
                coordinate_frame(item.get())
            }

            type_mask(src.type_mask())

            lat_int(src.lat_int())

            lon_int(src.lon_int())

            alt(src.alt())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            afx(src.afx())

            afy(src.afy())

            afz(src.afz())

            yaw(src.yaw())

            yaw_rate(src.yaw_rate())

        }

    }
}

inline class DATA32(val data: Pack.Bytes) {

    inline fun typE(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun typE_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun len(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun len_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun daTa(): DATA32_daTa {

        return DATA32_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 32)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 32


    }


    companion object {

        val meta = Meta(164, 0, 0, 0, 34, 1, 272)


        inline fun push(src: DATA32, typE: (src: Byte) -> Unit, len: (src: Byte) -> Unit, daTa: (src: DATA32_daTa) -> Unit
        ) {

            typE(src.typE())

            len(src.len())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: DATA32, typE: () -> Byte, len: () -> Byte, daTa: (dst: DATA32_daTa) -> Unit
        ) {

            dst.typE(typE())

            dst.len(len())
            daTa(dst.daTa())

        }

    }
}

inline class PING33(val data: Cursor) {

    inline fun TTTT(d0: Int, d1: Int, d2: Int): Int {
        return (get_bytes(data.bytes, data.origin + 0 + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)).toInt()
    }

    inline fun TTTT_(src: Int, d0: Int, d1: Int, d2: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0 + (d0 + d1 * 3 + d2 * 3 * 2) * 4)
    }

    object TTTT {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2: Int = 3


        inline fun enumerate(pack: PING33, dst: (item: Int, d0: Int, d1: Int, d2: Int) -> Unit) {
            for (d0 in 0 until PING33.TTTT.d0)
                for (d1 in 0 until PING33.TTTT.d1)
                    for (d2 in 0 until PING33.TTTT.d2)

                        dst((get_bytes(pack.data.bytes, pack.data.origin + 0 + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)).toInt(), d0, d1, d2)
        }

    }

    inline fun field(): Long {
        return (get_bytes(data.bytes, data.origin + 72, 8)).toLong()
    }

    inline fun field_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 72)
    }

    inline fun bit_field(): Byte {
        return (4 + data.bytes[data.origin + 80]).toByte()
    }

    inline fun bit_field_(src: Byte) {
        data.bytes[data.origin + 80] = (-4 + src).toByte()
    }

    object bit_field {

        val min_value = (4).toByte()
        val max_value = (45).toByte()


    }

    inline fun field6(d0: Int, d1: Int, d2: Int): Int {
        return (get_bytes(data.bytes, data.origin + 81 + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)).toInt()
    }

    inline fun field6_(src: Int, d0: Int, d1: Int, d2: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 81 + (d0 + d1 * 3 + d2 * 3 * 2) * 4)
    }

    object field6 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2: Int = 3


        inline fun enumerate(pack: PING33, dst: (item: Int, d0: Int, d1: Int, d2: Int) -> Unit) {
            for (d0 in 0 until PING33.field6.d0)
                for (d1 in 0 until PING33.field6.d1)
                    for (d2 in 0 until PING33.field6.d2)

                        dst((get_bytes(pack.data.bytes, pack.data.origin + 81 + (d0 + d1 * 3 + d2 * 3 * 2) * 4, 4)).toInt(), d0, d1, d2)
        }

    }

    inline fun testBOOL2(): Boolean {
        return (data.bytes[(data.origin * 8 + 1224) shr 3].toUInt() and (1u shl (data.origin * 8 + 1224 and 7))) != 0u
    }

    inline fun testBOOL2_(src: Boolean) {
        if (src) data.bytes[(data.origin * 8 + 1224) shr 3] = (data.bytes[(data.origin * 8 + 1224) shr 3].toUInt() or (1u shl (data.origin * 8 + 1224 and 7))).toByte()
        else data.bytes[(data.origin * 8 + 1224) shr 3] = (data.bytes[(data.origin * 8 + 1224) shr 3].toUInt() and (1u shl (data.origin * 8 + 1224 and 7)).inv()).toByte()
    }

    inline fun testBOOL3(): Boolean {
        return (data.bytes[(data.origin * 8 + 1225) shr 3].toUInt() and (1u shl (data.origin * 8 + 1225 and 7))) != 0u
    }

    inline fun testBOOL3_(src: Boolean) {
        if (src) data.bytes[(data.origin * 8 + 1225) shr 3] = (data.bytes[(data.origin * 8 + 1225) shr 3].toUInt() or (1u shl (data.origin * 8 + 1225 and 7))).toByte()
        else data.bytes[(data.origin * 8 + 1225) shr 3] = (data.bytes[(data.origin * 8 + 1225) shr 3].toUInt() and (1u shl (data.origin * 8 + 1225 and 7)).inv()).toByte()
    }


    inline fun testBOOL(): Boolean {
        return !(data.field_bit != 1234 && !data.set_field(1234, -1))
    }


    inline fun testBOOL_() {
        if (data.field_bit != 1234) data.set_field(1234, 0)
    }


    inline fun seq(): Boolean {
        return !(data.field_bit != 1235 && !data.set_field(1235, -1))
    }


    inline fun seq_() {
        if (data.field_bit != 1235) data.set_field(1235, 0)
    }


    inline fun field1(): PING33_field1 {
        return PING33_field1(data)
    }

    object field1 {
        const val d0_max: Int = 7
        const val d1: Int = 2
        const val d2: Int = 3

    }


    inline fun field12(src: Int, d0: Int, d1: Int, d2: Int) {
        if (data.field_bit != 1237) data.set_field(1237, 0)
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.BYTE + (d0 + d1 * 3 + d2 * 3 * 2) * 4)
    }


    inline fun field12(): PING33_field12? {
        return if (data.field_bit == 1237 || data.set_field(1237, -1)) PING33_field12(data) else null
    }

    object field12 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2: Int = 3

    }


    inline fun field13(d0: Int, d1: Int, d2: Int) {
        if (data.field_bit != 1238) data.set_field(1238, 0)
        data.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0)
    }


    inline fun field13(): PING33_field13? {
        return if (data.field_bit == 1238 || data.set_field(1238, -1)) PING33_field13(data) else null
    }

    object field13 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2: Int = 3

    }


    inline fun WWWWWWWW(): Boolean {
        return !(data.field_bit != 1239 && !data.set_field(1239, -1))
    }


    inline fun WWWWWWWW_() {
        if (data.field_bit != 1239) data.set_field(1239, 0)
    }


    inline fun bit_field2(): Boolean {
        return !(data.field_bit != 1240 && !data.set_field(1240, -1))
    }


    inline fun bit_field2_() {
        if (data.field_bit != 1240) data.set_field(1240, 0)
    }

    object bit_field2 {

        val min_value = (4).toByte()
        val max_value = (45).toByte()

    }


    inline fun Field_Bits(src: Byte, d0: Int, d1: Int, d2: Int) {
        if (data.field_bit != 1241) data.set_field(1241, 0)
        set_bits((-4 + src).toULong().toLong(), 6, data.bytes, data.BIT + (d0 + d1 * 3 + d2 * 3 * 3) * 6)
    }


    inline fun Field_Bits(): PING33_Field_Bits? {
        return if (data.field_bit == 1241 || data.set_field(1241, -1)) PING33_Field_Bits(data) else null
    }

    object Field_Bits {

        val min_value = (4).toByte()
        val max_value = (45).toByte()
        const val d0: Int = 3
        const val d1: Int = 3
        const val d2: Int = 3

    }


    inline fun SparseFixAllBits(): PING33_SparseFixAllBits {
        return PING33_SparseFixAllBits(data)
    }

    object SparseFixAllBits {

        val min_value = (4).toByte()
        val max_value = (45).toByte()
        const val d0_max: Int = 3
        const val d1: Int = 3
        const val d2: Int = 3

    }


    inline fun FixAllBits(): PING33_FixAllBits {
        return PING33_FixAllBits(data)
    }

    object FixAllBits {

        val min_value = (14).toByte()
        val max_value = (45).toByte()
        const val d0_max: Int = 3
        const val d1: Int = 3
        const val d2: Int = 3

    }


    inline fun VarAllBits(): PING33_VarAllBits {
        return PING33_VarAllBits(data)
    }

    object VarAllBits {

        val min_value = (14).toByte()
        val max_value = (45).toByte()
        const val d0_max: Int = 3
        const val d1: Int = 3
        const val d2_max: Int = 3

    }


    inline fun SparseVarAllBits(): PING33_SparseVarAllBits {
        return PING33_SparseVarAllBits(data)
    }

    object SparseVarAllBits {

        val min_value = (14).toByte()
        val max_value = (45).toByte()
        const val d0_max: Int = 3
        const val d1: Int = 3
        const val d2_max: Int = 3

    }


    inline fun VarEachBits(): PING33_VarEachBits {
        return PING33_VarEachBits(data)
    }

    object VarEachBits {

        val min_value = (-14).toByte()
        val max_value = (45).toByte()
        const val d0_max: Int = 3
        const val d1: Int = 3
        const val d2: Int = 3

    }


    inline fun SparsVarEachBits(): PING33_SparsVarEachBits {
        return PING33_SparsVarEachBits(data)
    }

    object SparsVarEachBits {

        val min_value = (-14).toShort()
        val max_value = (450).toShort()
        const val d0_max: Int = 3
        const val d1: Int = 3
        const val d2: Int = 3

    }


    inline fun testBOOLX(): Boolean {
        return !(data.field_bit != 1248 && !data.set_field(1248, -1))
    }


    inline fun testBOOLX_() {
        if (data.field_bit != 1248) data.set_field(1248, 0)
    }


    inline fun testBOOL2X(): Boolean {
        return !(data.field_bit != 1249 && !data.set_field(1249, -1))
    }


    inline fun testBOOL2X_() {
        if (data.field_bit != 1249) data.set_field(1249, 0)
    }


    inline fun testBOOL3X(): Boolean {
        return !(data.field_bit != 1250 && !data.set_field(1250, -1))
    }


    inline fun testBOOL3X_() {
        if (data.field_bit != 1250) data.set_field(1250, 0)
    }


    inline fun MMMMMM(): PING33_MMMMMM? {
        if ((data.field_bit != 1251 && !data.set_field(1251, -1))) return null

        return PING33_MMMMMM(data)
    }


    inline fun MMMMMM(src: MAV_MODE) {
        if (data.field_bit != 1251) data.set_field(1251, 0)

        set_bits(MAV_MODE.set(src).toLong(), 4, data.bytes, data.BIT)
    }


    inline fun field44(): PING33_field44 {
        return PING33_field44(data)
    }

    object field44 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2_max: Int = 3

    }


    inline fun field634(): PING33_field634 {
        return PING33_field634(data)
    }

    object field634 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2_max: Int = 3

    }


    inline fun field33344(): PING33_field33344 {
        return PING33_field33344(data)
    }

    object field33344 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2_max: Int = 3

    }


    inline fun field333634(): PING33_field333634 {
        return PING33_field333634(data)
    }

    object field333634 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2_max: Int = 3

    }


    inline fun field__(d0: Int, d1: Int, d2: Int) {
        if (data.field_bit != 1256) data.set_field(1256, 0)
        data.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0)
    }


    inline fun field__(): PING33_field__? {
        return if (data.field_bit == 1256 || data.set_field(1256, -1)) PING33_field__(data) else null
    }

    object field__ {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2: Int = 3

    }


    inline fun field63(): PING33_field63 {
        return PING33_field63(data)
    }

    object field63 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2_max: Int = 3

    }


    inline fun uid2(d0: Int) {
        if (data.field_bit != 1258) data.set_field(1258, 0)
        data.set_item(d0, 0)
    }


    inline fun uid2(): PING33_uid2? {
        return if (data.field_bit == 1258 || data.set_field(1258, -1)) PING33_uid2(data) else null
    }

    object uid2 {
        const val d0: Int = 18

    }


    inline fun field2(d0: Int, d1: Int, d2: Int) {
        if (data.field_bit != 1259) data.set_field(1259, 0)
        data.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0)
    }


    inline fun field2(): PING33_field2? {
        return if (data.field_bit == 1259 || data.set_field(1259, -1)) PING33_field2(data) else null
    }

    object field2 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2: Int = 3

    }


    inline fun field4(d0: Int, d1: Int, d2: Int) {
        if (data.field_bit != 1260) data.set_field(1260, 0)
        data.set_item(d0 + d1 * 3 + d2 * 3 * 2, 0)
    }


    inline fun field4(): PING33_field4? {
        return if (data.field_bit == 1260 || data.set_field(1260, -1)) PING33_field4(data) else null
    }

    object field4 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2: Int = 3

    }


    inline fun stringtest1(): PING33_stringtest1? {
        if ((data.field_bit != 1261 && !data.set_field(1261, -1))) return null

        return PING33_stringtest1(data)
    }


    inline fun stringtest1_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -1261 - 1, reuse)
    }


    inline fun stringtest1_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(1261, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun stringtest1_(len: Int): PING33_stringtest1 {

        data.set_field(1261, minOf(len, 255))
        return PING33_stringtest1(data)
    }

    object stringtest1 {
        const val item_len_max = 255

    }


    inline fun stringtest2(): PING33_stringtest2 {
        return PING33_stringtest2(data)
    }

    object stringtest2 {
        const val d0: Int = 3
        const val d1: Int = 2
        const val d2_max: Int = 3
        const val item_len_max = 255

    }


    inline fun stringtest3(): PING33_stringtest3? {
        if ((data.field_bit != 1263 && !data.set_field(1263, -1))) return null

        return PING33_stringtest3(data)
    }


    inline fun stringtest3_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -1263 - 1, reuse)
    }


    inline fun stringtest3_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(1263, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun stringtest3_(len: Int): PING33_stringtest3 {

        data.set_field(1263, minOf(len, 255))
        return PING33_stringtest3(data)
    }

    object stringtest3 {
        const val item_len_max = 255

    }


    inline fun stringtest4(): PING33_stringtest4? {
        if ((data.field_bit != 1264 && !data.set_field(1264, -1))) return null

        return PING33_stringtest4(data)
    }


    inline fun stringtest4_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -1264 - 1, reuse)
    }


    inline fun stringtest4_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(1264, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun stringtest4_(len: Int): PING33_stringtest4 {

        data.set_field(1264, minOf(len, 255))
        return PING33_stringtest4(data)
    }

    object stringtest4 {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(139, 0, 18, 0, 155, 1, 1234, 8, 31)
        @JvmField
        val const3: Int = (56).toInt()//56
        @JvmField
        val stati_cconst1: Int = (1).toInt()//1
        @JvmField
        val stati_cconst1D: Float = (1.456).toFloat()//(float)1.456
        @JvmField
        val const3D: Float = (56.555).toFloat()//(float)56.555


        inline fun push(src: PING33, testBOOL: (src: Boolean) -> Unit, seq: (src: Long) -> Unit, field: (src: Long) -> Unit, field1_init: (d0: Int) -> Unit, field1: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field12: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field13: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, TTTT: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, WWWWWWWW: (src: Int) -> Unit, testBOOL2: (src: Boolean) -> Unit, testBOOL3: (src: Boolean) -> Unit, bit_field: (src: Byte) -> Unit, bit_field2: (src: Byte) -> Unit, Field_Bits: (src: Byte, d0: Int, d1: Int, d2: Int) -> Unit, SparseFixAllBits_init: (d0: Int) -> Unit, SparseFixAllBits: (src: Byte, d0: Int, d1: Int, d2: Int) -> Unit, FixAllBits_init: (d0: Int) -> Unit, FixAllBits: (src: Byte, d0: Int, d1: Int, d2: Int) -> Unit, VarAllBits_init: (d0: Int, d2: Int) -> Unit, VarAllBits: (src: Byte, d0: Int, d1: Int, d2: Int) -> Unit, SparseVarAllBits_init: (d0: Int, d2: Int) -> Unit, SparseVarAllBits: (src: Byte, d0: Int, d1: Int, d2: Int) -> Unit, VarEachBits_init: (d0: Int) -> Unit, VarEachBits: (src: Byte, d0: Int, d1: Int, d2: Int) -> Unit, SparsVarEachBits_init: (d0: Int) -> Unit, SparsVarEachBits: (src: Short, d0: Int, d1: Int, d2: Int) -> Unit, testBOOLX: (src: Boolean) -> Unit, testBOOL2X: (src: Boolean) -> Unit, testBOOL3X: (src: Boolean) -> Unit, MMMMMM: (src: com.company.demo.GroundControl.MAV_MODE) -> Unit, field44_init: (d2: Int) -> Unit, field44: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field634_init: (d2: Int) -> Unit, field634: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field33344_init: (d2: Int) -> Unit, field33344: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field333634_init: (d2: Int) -> Unit, field333634: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field__: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field6: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field63_init: (d2: Int) -> Unit, field63: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, uid2: (src: Byte, d0: Int) -> Unit, field2: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, field4: (src: Int, d0: Int, d1: Int, d2: Int) -> Unit, stringtest1: (src: PING33_stringtest1) -> Unit, stringtest2_init: (d2: Int) -> Unit, stringtest2: (src: PING33_stringtest2_Item, d0: Int, d1: Int, d2: Int) -> Unit, stringtest3: (src: PING33_stringtest3) -> Unit, stringtest4: (src: PING33_stringtest4) -> Unit
        ) {

            src.testBOOL().let { item ->
                testBOOL(item.get())
            }

            src.seq().let { item ->
                seq(item.get())
            }

            field(src.field())

            src.field1().Field()?.let { fld ->
                field1_init(fld.d0())
                fld.enumerate { d0, d1, d2 ->
                    field1(item, d0, d1, d2)

                }
            }

            src.field12()?.let { fld ->
                fld.enumerate { d0, d1, d2 ->
                    field12(item, d0, d1, d2)
                }
            }

            src.field13()?.let { fld ->
                fld.enumerate { d0, d1, d2 ->
                    field13(item, d0, d1, d2)
                }
            }

            PING33.TTTT.enumerate(src)
            { item, d0, d1, d2 ->

                TTTT(src.TTTT(d0, d1, d2), d0, d1, d2)

            }

            src.WWWWWWWW().let { item ->
                WWWWWWWW(item.get())
            }

            testBOOL2(src.testBOOL2())

            testBOOL3(src.testBOOL3())

            bit_field(src.bit_field())

            src.bit_field2().let { item ->
                bit_field2(item.get())
            }

            src.Field_Bits()?.let { fld ->
                fld.enumerate { d0, d1, d2 ->
                    Field_Bits(item, d0, d1, d2)
                }
            }

            src.SparseFixAllBits().Field()?.let { fld ->
                SparseFixAllBits_init(fld.d0())
                fld.enumerate { d0, d1, d2 ->
                    SparseFixAllBits(item, d0, d1, d2)

                }
            }

            src.FixAllBits().Field()?.let { fld ->
                FixAllBits_init(fld.d0())
                fld.enumerate { d0, d1, d2 ->
                    FixAllBits(item, d0, d1, d2)

                }
            }

            src.VarAllBits().Field()?.let { fld ->
                VarAllBits_init(fld.d0(), fld.d2())
                fld.enumerate { d0, d1, d2 ->
                    VarAllBits(item, d0, d1, d2)

                }
            }

            src.SparseVarAllBits().Field()?.let { fld ->
                SparseVarAllBits_init(fld.d0(), fld.d2())
                fld.enumerate { d0, d1, d2 ->
                    SparseVarAllBits(item, d0, d1, d2)

                }
            }

            src.VarEachBits().Field()?.let { fld ->
                VarEachBits_init(fld.d0())
                fld.enumerate { d0, d1, d2 ->
                    VarEachBits(item, d0, d1, d2)

                }
            }

            src.SparsVarEachBits().Field()?.let { fld ->
                SparsVarEachBits_init(fld.d0())
                fld.enumerate { d0, d1, d2 ->
                    SparsVarEachBits(item, d0, d1, d2)

                }
            }

            src.testBOOLX().let { item ->
                testBOOLX(item.get())
            }

            src.testBOOL2X().let { item ->
                testBOOL2X(item.get())
            }

            src.testBOOL3X().let { item ->
                testBOOL3X(item.get())
            }

            src.MMMMMM()?.let { item ->
                MMMMMM(item.get())
            }

            src.field44().Field()?.let { fld ->
                field44_init(fld.d2())
                fld.enumerate { d0, d1, d2 ->
                    field44(item, d0, d1, d2)

                }
            }

            src.field634().Field()?.let { fld ->
                field634_init(fld.d2())
                fld.enumerate { d0, d1, d2 ->
                    field634(item, d0, d1, d2)

                }
            }

            src.field33344().Field()?.let { fld ->
                field33344_init(fld.d2())
                fld.enumerate { d0, d1, d2 ->
                    field33344(item, d0, d1, d2)

                }
            }

            src.field333634().Field()?.let { fld ->
                field333634_init(fld.d2())
                fld.enumerate { d0, d1, d2 ->
                    field333634(item, d0, d1, d2)

                }
            }

            src.field__()?.let { fld ->
                fld.enumerate { d0, d1, d2 ->
                    field__(item, d0, d1, d2)
                }
            }

            PING33.field6.enumerate(src)
            { item, d0, d1, d2 ->

                field6(src.field6(d0, d1, d2), d0, d1, d2)

            }

            src.field63().Field()?.let { fld ->
                field63_init(fld.d2())
                fld.enumerate { d0, d1, d2 ->
                    field63(item, d0, d1, d2)

                }
            }

            src.uid2()?.let { fld ->
                fld.enumerate { d0 ->
                    uid2(item, d0)
                }
            }

            src.field2()?.let { fld ->
                fld.enumerate { d0, d1, d2 ->
                    field2(item, d0, d1, d2)
                }
            }

            src.field4()?.let { fld ->
                fld.enumerate { d0, d1, d2 ->
                    field4(item, d0, d1, d2)
                }
            }

            src.stringtest1()?.let { item -> stringtest1(item) }

            src.stringtest2().Field()?.let { fld ->
                stringtest2_init(fld.d2())
                fld.enumerate { d0, d1, d2 ->
                    stringtest2(item, d0, d1, d2)
                }
            }

            src.stringtest3()?.let { item -> stringtest3(item) }

            src.stringtest4()?.let { item -> stringtest4(item) }

        }

        inline fun pull(dst: PING33, testBOOL_exist: () -> Boolean, testBOOL: () -> Boolean, seq_exist: () -> Boolean, seq: () -> Long, field: () -> Long, field1_init: (init_field: PING33_field1_Initializer) -> Unit, field1: (d0: Int, d1: Int, d2: Int) -> Int, field12_exist: () -> Boolean, field12: (d0: Int, d1: Int, d2: Int) -> Int, field13_exist: () -> Boolean, field13_item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, field13: (d0: Int, d1: Int, d2: Int) -> Int, TTTT: (d0: Int, d1: Int, d2: Int) -> Int, WWWWWWWW_exist: () -> Boolean, WWWWWWWW: () -> Int, testBOOL2: () -> Boolean, testBOOL3: () -> Boolean, bit_field: () -> Byte, bit_field2_exist: () -> Boolean, bit_field2: () -> Byte, Field_Bits_exist: () -> Boolean, Field_Bits: (d0: Int, d1: Int, d2: Int) -> Byte, SparseFixAllBits_init: (init_field: PING33_SparseFixAllBits_Initializer) -> Unit, SparseFixAllBits_item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, SparseFixAllBits: (d0: Int, d1: Int, d2: Int) -> Byte, FixAllBits_init: (init_field: PING33_FixAllBits_Initializer) -> Unit, FixAllBits: (d0: Int, d1: Int, d2: Int) -> Byte, VarAllBits_init: (init_field: PING33_VarAllBits_Initializer) -> Unit, VarAllBits: (d0: Int, d1: Int, d2: Int) -> Byte, SparseVarAllBits_init: (init_field: PING33_SparseVarAllBits_Initializer) -> Unit, SparseVarAllBits_item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, SparseVarAllBits: (d0: Int, d1: Int, d2: Int) -> Byte, VarEachBits_init: (init_field: PING33_VarEachBits_Initializer) -> Unit, VarEachBits: (d0: Int, d1: Int, d2: Int) -> Byte, SparsVarEachBits_init: (init_field: PING33_SparsVarEachBits_Initializer) -> Unit, SparsVarEachBits_item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, SparsVarEachBits: (d0: Int, d1: Int, d2: Int) -> Short, testBOOLX_exist: () -> Boolean, testBOOLX: () -> Boolean, testBOOL2X_exist: () -> Boolean, testBOOL2X: () -> Boolean, testBOOL3X_exist: () -> Boolean, testBOOL3X: () -> Boolean, MMMMMM_exist: () -> Boolean, MMMMMM: () -> com.company.demo.GroundControl.MAV_MODE, field44_init: (init_field: PING33_field44_Initializer) -> Unit, field44: (d0: Int, d1: Int, d2: Int) -> Int, field634_init: (init_field: PING33_field634_Initializer) -> Unit, field634: (d0: Int, d1: Int, d2: Int) -> Int, field33344_init: (init_field: PING33_field33344_Initializer) -> Unit, field33344_item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, field33344: (d0: Int, d1: Int, d2: Int) -> Int, field333634_init: (init_field: PING33_field333634_Initializer) -> Unit, field333634_item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, field333634: (d0: Int, d1: Int, d2: Int) -> Int, field___exist: () -> Boolean, field___item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, field__: (d0: Int, d1: Int, d2: Int) -> Int, field6: (d0: Int, d1: Int, d2: Int) -> Int, field63_init: (init_field: PING33_field63_Initializer) -> Unit, field63: (d0: Int, d1: Int, d2: Int) -> Int, uid2_exist: () -> Boolean, uid2_item_exist: (d0: Int) -> Boolean, uid2: (d0: Int) -> Byte, field2_exist: () -> Boolean, field2_item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, field2: (d0: Int, d1: Int, d2: Int) -> Int, field4_exist: () -> Boolean, field4_item_exist: (d0: Int, d1: Int, d2: Int) -> Boolean, field4: (d0: Int, d1: Int, d2: Int) -> Int, stringtest1_exist: () -> Int, stringtest1: (dst: PING33_stringtest1) -> Unit, stringtest2_init: (init_field: PING33_stringtest2_Initializer) -> Unit, stringtest2_item_exist: (d0: Int, d1: Int, d2: Int) -> Int, stringtest2: (dst: PING33_stringtest2_Item, d0: Int, d1: Int, d2: Int) -> Unit, stringtest3_exist: () -> Int, stringtest3: (dst: PING33_stringtest3) -> Unit, stringtest4_exist: () -> Int, stringtest4: (dst: PING33_stringtest4) -> Unit
        ) {

            if (testBOOL_exist()) dst.testBOOL(testBOOL())


            if (seq_exist()) dst.seq(seq())


            dst.field(field())

            field1_init(dst.field1().Initializer()!!)
            dst.field1().Field()?.let { fld ->
                for (d0 in 0 until fld.d0())
                    for (d1 in 0 until PING33.field1.d1)
                        for (d2 in 0 until PING33.field1.d2) {

                            fld.set(field1(d0, d1, d2), d0, d1, d2)

                        }
            }

            if (field12_exist())
                for (d0 in 0 until PING33.field12.d0)
                    for (d1 in 0 until PING33.field12.d1)
                        for (d2 in 0 until PING33.field12.d2) {

                            dst.field12(field12(d0, d1, d2), d0, d1, d2)

                        }

            if (field13_exist())
                for (d0 in 0 until PING33.field13.d0)
                    for (d1 in 0 until PING33.field13.d1)
                        for (d2 in 0 until PING33.field13.d2) {
                            if (field13_item_exist(d0, d1, d2)) dst.field13(field13(d0, d1, d2), d0, d1, d2)

                        }
            for (d0 in 0 until PING33.TTTT.d0)
                for (d1 in 0 until PING33.TTTT.d1)
                    for (d2 in 0 until PING33.TTTT.d2) {

                        dst.TTTT(TTTT(d0, d1, d2), d0, d1, d2)

                    }

            if (WWWWWWWW_exist()) dst.WWWWWWWW(WWWWWWWW())


            dst.testBOOL2(testBOOL2())

            dst.testBOOL3(testBOOL3())

            dst.bit_field(bit_field())

            if (bit_field2_exist()) dst.bit_field2(bit_field2())


            if (Field_Bits_exist())
                for (d0 in 0 until PING33.Field_Bits.d0)
                    for (d1 in 0 until PING33.Field_Bits.d1)
                        for (d2 in 0 until PING33.Field_Bits.d2) {

                            dst.Field_Bits(Field_Bits(d0, d1, d2), d0, d1, d2)

                        }

            SparseFixAllBits_init(dst.SparseFixAllBits().Initializer()!!)
            dst.SparseFixAllBits().Field()?.let { fld ->
                for (d0 in 0 until fld.d0())
                    for (d1 in 0 until PING33.SparseFixAllBits.d1)
                        for (d2 in 0 until PING33.SparseFixAllBits.d2) {
                            if (SparseFixAllBits_item_exist(d0, d1, d2)) fld.set(SparseFixAllBits(d0, d1, d2), d0, d1, d2)

                        }
            }

            FixAllBits_init(dst.FixAllBits().Initializer()!!)
            dst.FixAllBits().Field()?.let { fld ->
                for (d0 in 0 until fld.d0())
                    for (d1 in 0 until PING33.FixAllBits.d1)
                        for (d2 in 0 until PING33.FixAllBits.d2) {

                            fld.set(FixAllBits(d0, d1, d2), d0, d1, d2)

                        }
            }

            VarAllBits_init(dst.VarAllBits().Initializer()!!)
            dst.VarAllBits().Field()?.let { fld ->
                for (d0 in 0 until fld.d0())
                    for (d1 in 0 until PING33.VarAllBits.d1)
                        for (d2 in 0 until fld.d2()) {

                            fld.set(VarAllBits(d0, d1, d2), d0, d1, d2)

                        }
            }

            SparseVarAllBits_init(dst.SparseVarAllBits().Initializer()!!)
            dst.SparseVarAllBits().Field()?.let { fld ->
                for (d0 in 0 until fld.d0())
                    for (d1 in 0 until PING33.SparseVarAllBits.d1)
                        for (d2 in 0 until fld.d2()) {
                            if (SparseVarAllBits_item_exist(d0, d1, d2)) fld.set(SparseVarAllBits(d0, d1, d2), d0, d1, d2)

                        }
            }

            VarEachBits_init(dst.VarEachBits().Initializer()!!)
            dst.VarEachBits().Field()?.let { fld ->
                for (d0 in 0 until fld.d0())
                    for (d1 in 0 until PING33.VarEachBits.d1)
                        for (d2 in 0 until PING33.VarEachBits.d2) {

                            fld.set(VarEachBits(d0, d1, d2), d0, d1, d2)

                        }
            }

            SparsVarEachBits_init(dst.SparsVarEachBits().Initializer()!!)
            dst.SparsVarEachBits().Field()?.let { fld ->
                for (d0 in 0 until fld.d0())
                    for (d1 in 0 until PING33.SparsVarEachBits.d1)
                        for (d2 in 0 until PING33.SparsVarEachBits.d2) {
                            if (SparsVarEachBits_item_exist(d0, d1, d2)) fld.set(SparsVarEachBits(d0, d1, d2), d0, d1, d2)

                        }
            }

            if (testBOOLX_exist()) dst.testBOOLX(testBOOLX())


            if (testBOOL2X_exist()) dst.testBOOL2X(testBOOL2X())


            if (testBOOL3X_exist()) dst.testBOOL3X(testBOOL3X())


            if (MMMMMM_exist()) dst.MMMMMM(MMMMMM())


            field44_init(dst.field44().Initializer()!!)
            dst.field44().Field()?.let { fld ->
                for (d0 in 0 until PING33.field44.d0)
                    for (d1 in 0 until PING33.field44.d1)
                        for (d2 in 0 until fld.d2()) {

                            fld.set(field44(d0, d1, d2), d0, d1, d2)

                        }
            }

            field634_init(dst.field634().Initializer()!!)
            dst.field634().Field()?.let { fld ->
                for (d0 in 0 until PING33.field634.d0)
                    for (d1 in 0 until PING33.field634.d1)
                        for (d2 in 0 until fld.d2()) {

                            fld.set(field634(d0, d1, d2), d0, d1, d2)

                        }
            }

            field33344_init(dst.field33344().Initializer()!!)
            dst.field33344().Field()?.let { fld ->
                for (d0 in 0 until PING33.field33344.d0)
                    for (d1 in 0 until PING33.field33344.d1)
                        for (d2 in 0 until fld.d2()) {
                            if (field33344_item_exist(d0, d1, d2)) fld.set(field33344(d0, d1, d2), d0, d1, d2)

                        }
            }

            field333634_init(dst.field333634().Initializer()!!)
            dst.field333634().Field()?.let { fld ->
                for (d0 in 0 until PING33.field333634.d0)
                    for (d1 in 0 until PING33.field333634.d1)
                        for (d2 in 0 until fld.d2()) {
                            if (field333634_item_exist(d0, d1, d2)) fld.set(field333634(d0, d1, d2), d0, d1, d2)

                        }
            }

            if (field___exist())
                for (d0 in 0 until PING33.field__.d0)
                    for (d1 in 0 until PING33.field__.d1)
                        for (d2 in 0 until PING33.field__.d2) {
                            if (field___item_exist(d0, d1, d2)) dst.field__(field__(d0, d1, d2), d0, d1, d2)

                        }
            for (d0 in 0 until PING33.field6.d0)
                for (d1 in 0 until PING33.field6.d1)
                    for (d2 in 0 until PING33.field6.d2) {

                        dst.field6(field6(d0, d1, d2), d0, d1, d2)

                    }

            field63_init(dst.field63().Initializer()!!)
            dst.field63().Field()?.let { fld ->
                for (d0 in 0 until PING33.field63.d0)
                    for (d1 in 0 until PING33.field63.d1)
                        for (d2 in 0 until fld.d2()) {

                            fld.set(field63(d0, d1, d2), d0, d1, d2)

                        }
            }

            if (uid2_exist())
                for (d0 in 0 until PING33.uid2.d0) {
                    if (uid2_item_exist(d0)) dst.uid2(uid2(d0), d0)

                }

            if (field2_exist())
                for (d0 in 0 until PING33.field2.d0)
                    for (d1 in 0 until PING33.field2.d1)
                        for (d2 in 0 until PING33.field2.d2) {
                            if (field2_item_exist(d0, d1, d2)) dst.field2(field2(d0, d1, d2), d0, d1, d2)

                        }

            if (field4_exist())
                for (d0 in 0 until PING33.field4.d0)
                    for (d1 in 0 until PING33.field4.d1)
                        for (d2 in 0 until PING33.field4.d2) {
                            if (field4_item_exist(d0, d1, d2)) dst.field4(field4(d0, d1, d2), d0, d1, d2)

                        }


            stringtest1_exist().let { len ->
                if (0 < len)
                    stringtest1(dst.stringtest1(len))
            }


            stringtest2_init(dst.stringtest2().Initializer()!!)
            dst.stringtest2().Field()?.let { fld ->
                for (d0 in 0 until PING33.stringtest2.d0)
                    for (d1 in 0 until PING33.stringtest2.d1)
                        for (d2 in 0 until fld.d2()) {

                            stringtest2_item_exist(d0, d1, d2).let { len ->
                                if (0 < len)
                                    stringtest2(fld.set(len, d0, d1, d2), d0, d1, d2)
                            }

                        }
            }


            stringtest3_exist().let { len ->
                if (0 < len)
                    stringtest3(dst.stringtest3(len))
            }



            stringtest4_exist().let { len ->
                if (0 < len)
                    stringtest4(dst.stringtest4(len))
            }


        }

    }
}

inline class VFR_HUD(val data: Pack.Bytes) {

    inline fun throttle(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun airspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }


    inline fun groundspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }


    inline fun heading(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun climb(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    companion object {

        val meta = Meta(15, 1, 0, 0, 20, 1, 160)


        inline fun push(src: VFR_HUD, airspeed: (src: Float) -> Unit, groundspeed: (src: Float) -> Unit, heading: (src: Short) -> Unit, throttle: (src: Short) -> Unit, alt: (src: Float) -> Unit, climb: (src: Float) -> Unit
        ) {

            airspeed(src.airspeed())

            groundspeed(src.groundspeed())

            heading(src.heading())

            throttle(src.throttle())

            alt(src.alt())

            climb(src.climb())

        }

    }
}

inline class RALLY_POINT(val data: Cursor) {

    inline fun land_dir(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun land_dir_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun idx(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun idx_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun count(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun count_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun lng(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun lng_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun alt(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun alt_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun break_alt(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }

    inline fun break_alt_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 16)
    }


    inline fun flags(): RALLY_POINT_flags? {
        if ((data.field_bit != 144 && !data.set_field(144, -1))) return null

        return RALLY_POINT_flags(data)
    }


    inline fun flags(src: RALLY_FLAGS) {
        if (data.field_bit != 144) data.set_field(144, 0)

        set_bits((-1 src . value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(197, 1, 0, 0, 19, 1, 144, 0, 1)


        inline fun push(src: RALLY_POINT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, idx: (src: Byte) -> Unit, count: (src: Byte) -> Unit, lat: (src: Int) -> Unit, lng: (src: Int) -> Unit, alt: (src: Short) -> Unit, break_alt: (src: Short) -> Unit, land_dir: (src: Short) -> Unit, flags: (src: com.company.demo.GroundControl.RALLY_FLAGS) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            idx(src.idx())

            count(src.count())

            lat(src.lat())

            lng(src.lng())

            alt(src.alt())

            break_alt(src.break_alt())

            land_dir(src.land_dir())

            src.flags()?.let { item ->
                flags(item.get())
            }

        }

        inline fun pull(dst: RALLY_POINT, target_system: () -> Byte, target_component: () -> Byte, idx: () -> Byte, count: () -> Byte, lat: () -> Int, lng: () -> Int, alt: () -> Short, break_alt: () -> Short, land_dir: () -> Short, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.RALLY_FLAGS
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.idx(idx())

            dst.count(count())

            dst.lat(lat())

            dst.lng(lng())

            dst.alt(alt())

            dst.break_alt(break_alt())

            dst.land_dir(land_dir())

            if (flags_exist()) dst.flags(flags())


        }

    }
}

inline class MISSION_SET_CURRENT(val data: Pack.Bytes) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    companion object {

        val meta = Meta(212, 1, 0, 0, 4, 1, 32)


        inline fun push(src: MISSION_SET_CURRENT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, seq: (src: Short) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            seq(src.seq())

        }

    }
}

inline class ADAP_TUNING(val data: Cursor) {

    inline fun desired(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun desired_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun achieved(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun achieved_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun error(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun error_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun theta(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun theta_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun omega(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun omega_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun sigma(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun sigma_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun theta_dot(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun theta_dot_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun omega_dot(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun omega_dot_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun sigma_dot(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun sigma_dot_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun f(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun f_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun f_dot(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun f_dot_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun u(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun u_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }


    inline fun axis(): ADAP_TUNING_axis? {
        if ((data.field_bit != 384 && !data.set_field(384, -1))) return null

        return ADAP_TUNING_axis(data)
    }


    inline fun axis(src: PID_TUNING_AXIS) {
        if (data.field_bit != 384) data.set_field(384, 0)

        set_bits((-1 src . value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(115, 0, 0, 0, 49, 1, 384, 0, 1)


        inline fun push(src: ADAP_TUNING, axis: (src: com.company.demo.GroundControl.PID_TUNING_AXIS) -> Unit, desired: (src: Float) -> Unit, achieved: (src: Float) -> Unit, error: (src: Float) -> Unit, theta: (src: Float) -> Unit, omega: (src: Float) -> Unit, sigma: (src: Float) -> Unit, theta_dot: (src: Float) -> Unit, omega_dot: (src: Float) -> Unit, sigma_dot: (src: Float) -> Unit, f: (src: Float) -> Unit, f_dot: (src: Float) -> Unit, u: (src: Float) -> Unit
        ) {

            src.axis()?.let { item ->
                axis(item.get())
            }

            desired(src.desired())

            achieved(src.achieved())

            error(src.error())

            theta(src.theta())

            omega(src.omega())

            sigma(src.sigma())

            theta_dot(src.theta_dot())

            omega_dot(src.omega_dot())

            sigma_dot(src.sigma_dot())

            f(src.f())

            f_dot(src.f_dot())

            u(src.u())

        }

        inline fun pull(dst: ADAP_TUNING, axis_exist: () -> Boolean, axis: () -> com.company.demo.GroundControl.PID_TUNING_AXIS, desired: () -> Float, achieved: () -> Float, error: () -> Float, theta: () -> Float, omega: () -> Float, sigma: () -> Float, theta_dot: () -> Float, omega_dot: () -> Float, sigma_dot: () -> Float, f: () -> Float, f_dot: () -> Float, u: () -> Float
        ) {

            if (axis_exist()) dst.axis(axis())


            dst.desired(desired())

            dst.achieved(achieved())

            dst.error(error())

            dst.theta(theta())

            dst.omega(omega())

            dst.sigma(sigma())

            dst.theta_dot(theta_dot())

            dst.omega_dot(omega_dot())

            dst.sigma_dot(sigma_dot())

            dst.f(f())

            dst.f_dot(f_dot())

            dst.u(u())

        }

    }
}

inline class VIBRATION(val data: Pack.Bytes) {

    inline fun clipping_0(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun clipping_0_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun clipping_1(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun clipping_1_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun clipping_2(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }

    inline fun clipping_2_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 12, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 12)
    }

    inline fun vibration_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun vibration_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun vibration_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun vibration_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun vibration_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun vibration_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }


    companion object {

        val meta = Meta(103, 0, 3, 1, 32, 1, 256)


        inline fun push(src: VIBRATION, time_usec: (src: Long) -> Unit, vibration_x: (src: Float) -> Unit, vibration_y: (src: Float) -> Unit, vibration_z: (src: Float) -> Unit, clipping_0: (src: Int) -> Unit, clipping_1: (src: Int) -> Unit, clipping_2: (src: Int) -> Unit
        ) {

            time_usec(src.time_usec())

            vibration_x(src.vibration_x())

            vibration_y(src.vibration_y())

            vibration_z(src.vibration_z())

            clipping_0(src.clipping_0())

            clipping_1(src.clipping_1())

            clipping_2(src.clipping_2())

        }

        inline fun pull(dst: VIBRATION, time_usec: () -> Long, vibration_x: () -> Float, vibration_y: () -> Float, vibration_z: () -> Float, clipping_0: () -> Int, clipping_1: () -> Int, clipping_2: () -> Int
        ) {

            dst.time_usec(time_usec())

            dst.vibration_x(vibration_x())

            dst.vibration_y(vibration_y())

            dst.vibration_z(vibration_z())

            dst.clipping_0(clipping_0())

            dst.clipping_1(clipping_1())

            dst.clipping_2(clipping_2())

        }

    }
}

inline class PARAM_EXT_VALUE(val data: Cursor) {

    inline fun param_count(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun param_count_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun param_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun param_index_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }


    inline fun param_id(): PARAM_EXT_VALUE_param_id? {
        if ((data.field_bit != 35 && !data.set_field(35, -1))) return null

        return PARAM_EXT_VALUE_param_id(data)
    }


    inline fun param_id_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -35 - 1, reuse)
    }


    inline fun param_id_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(35, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_id_(len: Int): PARAM_EXT_VALUE_param_id {

        data.set_field(35, minOf(len, 255))
        return PARAM_EXT_VALUE_param_id(data)
    }

    object param_id {
        const val item_len_max = 255

    }


    inline fun param_value(): PARAM_EXT_VALUE_param_value? {
        if ((data.field_bit != 36 && !data.set_field(36, -1))) return null

        return PARAM_EXT_VALUE_param_value(data)
    }


    inline fun param_value_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -36 - 1, reuse)
    }


    inline fun param_value_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(36, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_value_(len: Int): PARAM_EXT_VALUE_param_value {

        data.set_field(36, minOf(len, 255))
        return PARAM_EXT_VALUE_param_value(data)
    }

    object param_value {
        const val item_len_max = 255

    }


    inline fun param_type(): PARAM_EXT_VALUE_param_type? {
        if ((data.field_bit != 37 && !data.set_field(37, -1))) return null

        return PARAM_EXT_VALUE_param_type(data)
    }


    inline fun param_type(src: MAV_PARAM_EXT_TYPE) {
        if (data.field_bit != 37) data.set_field(37, 0)

        set_bits((-1 src . value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(14, 2, 0, 0, 5, 1, 35, 3, 3)


        inline fun push(src: PARAM_EXT_VALUE, param_id: (src: PARAM_EXT_VALUE_param_id) -> Unit, param_value: (src: PARAM_EXT_VALUE_param_value) -> Unit, param_type: (src: com.company.demo.GroundControl.MAV_PARAM_EXT_TYPE) -> Unit, param_count: (src: Short) -> Unit, param_index: (src: Short) -> Unit
        ) {

            src.param_id()?.let { item -> param_id(item) }

            src.param_value()?.let { item -> param_value(item) }

            src.param_type()?.let { item ->
                param_type(item.get())
            }

            param_count(src.param_count())

            param_index(src.param_index())

        }

        inline fun pull(dst: PARAM_EXT_VALUE, param_id_exist: () -> Int, param_id: (dst: PARAM_EXT_VALUE_param_id) -> Unit, param_value_exist: () -> Int, param_value: (dst: PARAM_EXT_VALUE_param_value) -> Unit, param_type_exist: () -> Boolean, param_type: () -> com.company.demo.GroundControl.MAV_PARAM_EXT_TYPE, param_count: () -> Short, param_index: () -> Short
        ) {


            param_id_exist().let { len ->
                if (0 < len)
                    param_id(dst.param_id(len))
            }



            param_value_exist().let { len ->
                if (0 < len)
                    param_value(dst.param_value(len))
            }


            if (param_type_exist()) dst.param_type(param_type())


            dst.param_count(param_count())

            dst.param_index(param_index())

        }

    }
}

inline class BATTERY2(val data: Pack.Bytes) {

    inline fun voltage(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun voltage_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun current_battery(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun current_battery_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }


    companion object {

        val meta = Meta(190, 1, 0, 0, 4, 1, 32)


        inline fun push(src: BATTERY2, voltage: (src: Short) -> Unit, current_battery: (src: Short) -> Unit
        ) {

            voltage(src.voltage())

            current_battery(src.current_battery())

        }

        inline fun pull(dst: BATTERY2, voltage: () -> Short, current_battery: () -> Short
        ) {

            dst.voltage(voltage())

            dst.current_battery(current_battery())

        }

    }
}

inline class LIMITS_STATUS(val data: Cursor) {

    inline fun breach_count(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun breach_count_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun last_trigger(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun last_trigger_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun last_action(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun last_action_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun last_recovery(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun last_recovery_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun last_clear(): Int {
        return (get_bytes(data.bytes, data.origin + 14, 4)).toInt()
    }

    inline fun last_clear_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }


    inline fun limits_state(): LIMITS_STATUS_limits_state? {
        if ((data.field_bit != 146 && !data.set_field(146, -1))) return null

        return LIMITS_STATUS_limits_state(data)
    }


    inline fun limits_state(src: LIMITS_STATE) {
        if (data.field_bit != 146) data.set_field(146, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun mods_enabled(): LIMITS_STATUS_mods_enabled? {
        if ((data.field_bit != 147 && !data.set_field(147, -1))) return null

        return LIMITS_STATUS_mods_enabled(data)
    }


    inline fun mods_enabled(src: LIMIT_MODULE) {
        if (data.field_bit != 147) data.set_field(147, 0)

        set_bits(LIMIT_MODULE.set(src).toLong(), 2, data.bytes, data.BIT)
    }


    inline fun mods_required(): LIMITS_STATUS_mods_required? {
        if ((data.field_bit != 148 && !data.set_field(148, -1))) return null

        return LIMITS_STATUS_mods_required(data)
    }


    inline fun mods_required(src: LIMIT_MODULE) {
        if (data.field_bit != 148) data.set_field(148, 0)

        set_bits(LIMIT_MODULE.set(src).toLong(), 2, data.bytes, data.BIT)
    }


    inline fun mods_triggered(): LIMITS_STATUS_mods_triggered? {
        if ((data.field_bit != 149 && !data.set_field(149, -1))) return null

        return LIMITS_STATUS_mods_triggered(data)
    }


    inline fun mods_triggered(src: LIMIT_MODULE) {
        if (data.field_bit != 149) data.set_field(149, 0)

        set_bits(LIMIT_MODULE.set(src).toLong(), 2, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(186, 1, 4, 0, 19, 1, 146, 2, 4)


        inline fun push(src: LIMITS_STATUS, limits_state: (src: com.company.demo.GroundControl.LIMITS_STATE) -> Unit, last_trigger: (src: Int) -> Unit, last_action: (src: Int) -> Unit, last_recovery: (src: Int) -> Unit, last_clear: (src: Int) -> Unit, breach_count: (src: Short) -> Unit, mods_enabled: (src: com.company.demo.GroundControl.LIMIT_MODULE) -> Unit, mods_required: (src: com.company.demo.GroundControl.LIMIT_MODULE) -> Unit, mods_triggered: (src: com.company.demo.GroundControl.LIMIT_MODULE) -> Unit
        ) {

            src.limits_state()?.let { item ->
                limits_state(item.get())
            }

            last_trigger(src.last_trigger())

            last_action(src.last_action())

            last_recovery(src.last_recovery())

            last_clear(src.last_clear())

            breach_count(src.breach_count())

            src.mods_enabled()?.let { item ->
                mods_enabled(item.get())
            }

            src.mods_required()?.let { item ->
                mods_required(item.get())
            }

            src.mods_triggered()?.let { item ->
                mods_triggered(item.get())
            }

        }

        inline fun pull(dst: LIMITS_STATUS, limits_state_exist: () -> Boolean, limits_state: () -> com.company.demo.GroundControl.LIMITS_STATE, last_trigger: () -> Int, last_action: () -> Int, last_recovery: () -> Int, last_clear: () -> Int, breach_count: () -> Short, mods_enabled_exist: () -> Boolean, mods_enabled: () -> com.company.demo.GroundControl.LIMIT_MODULE, mods_required_exist: () -> Boolean, mods_required: () -> com.company.demo.GroundControl.LIMIT_MODULE, mods_triggered_exist: () -> Boolean, mods_triggered: () -> com.company.demo.GroundControl.LIMIT_MODULE
        ) {

            if (limits_state_exist()) dst.limits_state(limits_state())


            dst.last_trigger(last_trigger())

            dst.last_action(last_action())

            dst.last_recovery(last_recovery())

            dst.last_clear(last_clear())

            dst.breach_count(breach_count())

            if (mods_enabled_exist()) dst.mods_enabled(mods_enabled())


            if (mods_required_exist()) dst.mods_required(mods_required())


            if (mods_triggered_exist()) dst.mods_triggered(mods_triggered())


        }

    }
}

inline class CAMERA_FEEDBACK(val data: Cursor) {

    inline fun img_idx(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun img_idx_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 2, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 2)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun cam_idx(): Byte {
        return (data.bytes[data.origin + 11]).toByte()
    }

    inline fun cam_idx_(src: Byte) {
        data.bytes[data.origin + 11] = (src).toByte()
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 12, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun lng(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun lng_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun alt_msl(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun alt_msl_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun alt_rel(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun alt_rel_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun foc_len(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun foc_len_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }


    inline fun flags(): CAMERA_FEEDBACK_flags? {
        if ((data.field_bit != 352 && !data.set_field(352, -1))) return null

        return CAMERA_FEEDBACK_flags(data)
    }


    inline fun flags(src: CAMERA_FEEDBACK_FLAGS) {
        if (data.field_bit != 352) data.set_field(352, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(0, 1, 0, 1, 45, 1, 352, 0, 1)


        inline fun push(src: CAMERA_FEEDBACK, time_usec: (src: Long) -> Unit, target_system: (src: Byte) -> Unit, cam_idx: (src: Byte) -> Unit, img_idx: (src: Short) -> Unit, lat: (src: Int) -> Unit, lng: (src: Int) -> Unit, alt_msl: (src: Float) -> Unit, alt_rel: (src: Float) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit, foc_len: (src: Float) -> Unit, flags: (src: com.company.demo.GroundControl.CAMERA_FEEDBACK_FLAGS) -> Unit
        ) {

            time_usec(src.time_usec())

            target_system(src.target_system())

            cam_idx(src.cam_idx())

            img_idx(src.img_idx())

            lat(src.lat())

            lng(src.lng())

            alt_msl(src.alt_msl())

            alt_rel(src.alt_rel())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

            foc_len(src.foc_len())

            src.flags()?.let { item ->
                flags(item.get())
            }

        }

        inline fun pull(dst: CAMERA_FEEDBACK, time_usec: () -> Long, target_system: () -> Byte, cam_idx: () -> Byte, img_idx: () -> Short, lat: () -> Int, lng: () -> Int, alt_msl: () -> Float, alt_rel: () -> Float, roll: () -> Float, pitch: () -> Float, yaw: () -> Float, foc_len: () -> Float, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.CAMERA_FEEDBACK_FLAGS
        ) {

            dst.time_usec(time_usec())

            dst.target_system(target_system())

            dst.cam_idx(cam_idx())

            dst.img_idx(img_idx())

            dst.lat(lat())

            dst.lng(lng())

            dst.alt_msl(alt_msl())

            dst.alt_rel(alt_rel())

            dst.roll(roll())

            dst.pitch(pitch())

            dst.yaw(yaw())

            dst.foc_len(foc_len())

            if (flags_exist()) dst.flags(flags())


        }

    }
}

inline class HIL_GPS(val data: Pack.Bytes) {

    inline fun eph(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun eph_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun epv(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun epv_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun vel(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun vel_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun cog(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun cog_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }

    inline fun fix_type(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }

    inline fun fix_type_(src: Byte) {
        data.bytes[data.origin + 16] = (src).toByte()
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 17, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 21, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 21)
    }

    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 25, 4)).toInt()
    }

    inline fun alt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 25)
    }

    inline fun vn(): Short {
        return (get_bytes(data.bytes, data.origin + 29, 2)).toShort()
    }

    inline fun vn_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 29)
    }

    inline fun ve(): Short {
        return (get_bytes(data.bytes, data.origin + 31, 2)).toShort()
    }

    inline fun ve_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 31)
    }

    inline fun vd(): Short {
        return (get_bytes(data.bytes, data.origin + 33, 2)).toShort()
    }

    inline fun vd_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 33)
    }

    inline fun satellites_visible(): Byte {
        return (data.bytes[data.origin + 35]).toByte()
    }

    inline fun satellites_visible_(src: Byte) {
        data.bytes[data.origin + 35] = (src).toByte()
    }


    companion object {

        val meta = Meta(168, 4, 0, 1, 36, 1, 288)


        inline fun push(src: HIL_GPS, time_usec: (src: Long) -> Unit, fix_type: (src: Byte) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Int) -> Unit, eph: (src: Short) -> Unit, epv: (src: Short) -> Unit, vel: (src: Short) -> Unit, vn: (src: Short) -> Unit, ve: (src: Short) -> Unit, vd: (src: Short) -> Unit, cog: (src: Short) -> Unit, satellites_visible: (src: Byte) -> Unit
        ) {

            time_usec(src.time_usec())

            fix_type(src.fix_type())

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            eph(src.eph())

            epv(src.epv())

            vel(src.vel())

            vn(src.vn())

            ve(src.ve())

            vd(src.vd())

            cog(src.cog())

            satellites_visible(src.satellites_visible())

        }

        inline fun pull(dst: HIL_GPS, time_usec: () -> Long, fix_type: () -> Byte, lat: () -> Int, lon: () -> Int, alt: () -> Int, eph: () -> Short, epv: () -> Short, vel: () -> Short, vn: () -> Short, ve: () -> Short, vd: () -> Short, cog: () -> Short, satellites_visible: () -> Byte
        ) {

            dst.time_usec(time_usec())

            dst.fix_type(fix_type())

            dst.lat(lat())

            dst.lon(lon())

            dst.alt(alt())

            dst.eph(eph())

            dst.epv(epv())

            dst.vel(vel())

            dst.vn(vn())

            dst.ve(ve())

            dst.vd(vd())

            dst.cog(cog())

            dst.satellites_visible(satellites_visible())

        }

    }
}

inline class NAV_CONTROLLER_OUTPUT(val data: Pack.Bytes) {

    inline fun wp_dist(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun nav_roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }


    inline fun nav_pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }


    inline fun nav_bearing(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun target_bearing(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun alt_error(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }


    inline fun aspd_error(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }


    inline fun xtrack_error(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }


    companion object {

        val meta = Meta(201, 1, 0, 0, 26, 1, 208)


        inline fun push(src: NAV_CONTROLLER_OUTPUT, nav_roll: (src: Float) -> Unit, nav_pitch: (src: Float) -> Unit, nav_bearing: (src: Short) -> Unit, target_bearing: (src: Short) -> Unit, wp_dist: (src: Short) -> Unit, alt_error: (src: Float) -> Unit, aspd_error: (src: Float) -> Unit, xtrack_error: (src: Float) -> Unit
        ) {

            nav_roll(src.nav_roll())

            nav_pitch(src.nav_pitch())

            nav_bearing(src.nav_bearing())

            target_bearing(src.target_bearing())

            wp_dist(src.wp_dist())

            alt_error(src.alt_error())

            aspd_error(src.aspd_error())

            xtrack_error(src.xtrack_error())

        }

    }
}

inline class AUTH_KEY(val data: Cursor) {


    inline fun key(): AUTH_KEY_key? {
        if ((data.field_bit != 2 && !data.set_field(2, -1))) return null

        return AUTH_KEY_key(data)
    }


    object key {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(95, 0, 0, 0, 1, 1, 2, 2, 1)


        inline fun push(src: AUTH_KEY, key: (src: AUTH_KEY_key) -> Unit
        ) {

            src.key()?.let { item -> key(item) }

        }

    }
}

inline class FENCE_FETCH_POINT(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun idx(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun idx_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }


    companion object {

        val meta = Meta(119, 0, 0, 0, 3, 1, 24)


        inline fun push(src: FENCE_FETCH_POINT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, idx: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            idx(src.idx())

        }

        inline fun pull(dst: FENCE_FETCH_POINT, target_system: () -> Byte, target_component: () -> Byte, idx: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.idx(idx())

        }

    }
}

inline class RADIO(val data: Pack.Bytes) {

    inline fun rxerrors(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun rxerrors_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun fixeD(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun fixeD_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun rssi_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun remrssi(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun remrssi_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun txbuf(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun txbuf_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun noise(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }

    inline fun noise_(src: Byte) {
        data.bytes[data.origin + 7] = (src).toByte()
    }

    inline fun remnoise(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun remnoise_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }


    companion object {

        val meta = Meta(32, 2, 0, 0, 9, 1, 72)


        inline fun push(src: RADIO, rssi: (src: Byte) -> Unit, remrssi: (src: Byte) -> Unit, txbuf: (src: Byte) -> Unit, noise: (src: Byte) -> Unit, remnoise: (src: Byte) -> Unit, rxerrors: (src: Short) -> Unit, fixeD: (src: Short) -> Unit
        ) {

            rssi(src.rssi())

            remrssi(src.remrssi())

            txbuf(src.txbuf())

            noise(src.noise())

            remnoise(src.remnoise())

            rxerrors(src.rxerrors())

            fixeD(src.fixeD())

        }

        inline fun pull(dst: RADIO, rssi: () -> Byte, remrssi: () -> Byte, txbuf: () -> Byte, noise: () -> Byte, remnoise: () -> Byte, rxerrors: () -> Short, fixeD: () -> Short
        ) {

            dst.rssi(rssi())

            dst.remrssi(remrssi())

            dst.txbuf(txbuf())

            dst.noise(noise())

            dst.remnoise(remnoise())

            dst.rxerrors(rxerrors())

            dst.fixeD(fixeD())

        }

    }
}

inline class LOCAL_POSITION_NED_COV(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    inline fun ax(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }


    inline fun ay(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }


    inline fun az(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }


    inline fun covariance(): LOCAL_POSITION_NED_COV_covariance {

        return LOCAL_POSITION_NED_COV_covariance(data)
    }

    object covariance {
        const val item_len = 45


    }


    inline fun estimator_type(): LOCAL_POSITION_NED_COV_estimator_type? {
        if ((data.field_bit != 1792 && !data.set_field(1792, -1))) return null

        return LOCAL_POSITION_NED_COV_estimator_type(data)
    }


    companion object {

        val meta = Meta(180, 0, 0, 1, 225, 1, 1792, 0, 1)


        inline fun push(src: LOCAL_POSITION_NED_COV, time_usec: (src: Long) -> Unit, estimator_type: (src: com.company.demo.GroundControl.MAV_ESTIMATOR_TYPE) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, vx: (src: Float) -> Unit, vy: (src: Float) -> Unit, vz: (src: Float) -> Unit, ax: (src: Float) -> Unit, ay: (src: Float) -> Unit, az: (src: Float) -> Unit, covariance: (src: LOCAL_POSITION_NED_COV_covariance) -> Unit
        ) {

            time_usec(src.time_usec())

            src.estimator_type()?.let { item ->
                estimator_type(item.get())
            }

            x(src.x())

            y(src.y())

            z(src.z())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            ax(src.ax())

            ay(src.ay())

            az(src.az())
            src.covariance().let { item ->
                covariance(item)
            }

        }

    }
}

inline class AIRSPEED_AUTOCAL(val data: Pack.Bytes) {

    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun vx_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun vy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun vz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun diff_pressure(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun diff_pressure_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun EAS2TAS(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun EAS2TAS_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun ratio(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun ratio_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun state_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun state_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun state_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun state_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun state_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun state_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun Pax(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun Pax_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun Pby(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun Pby_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun Pcz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun Pcz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }


    companion object {

        val meta = Meta(69, 0, 0, 0, 48, 1, 384)


        inline fun push(src: AIRSPEED_AUTOCAL, vx: (src: Float) -> Unit, vy: (src: Float) -> Unit, vz: (src: Float) -> Unit, diff_pressure: (src: Float) -> Unit, EAS2TAS: (src: Float) -> Unit, ratio: (src: Float) -> Unit, state_x: (src: Float) -> Unit, state_y: (src: Float) -> Unit, state_z: (src: Float) -> Unit, Pax: (src: Float) -> Unit, Pby: (src: Float) -> Unit, Pcz: (src: Float) -> Unit
        ) {

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            diff_pressure(src.diff_pressure())

            EAS2TAS(src.EAS2TAS())

            ratio(src.ratio())

            state_x(src.state_x())

            state_y(src.state_y())

            state_z(src.state_z())

            Pax(src.Pax())

            Pby(src.Pby())

            Pcz(src.Pcz())

        }

        inline fun pull(dst: AIRSPEED_AUTOCAL, vx: () -> Float, vy: () -> Float, vz: () -> Float, diff_pressure: () -> Float, EAS2TAS: () -> Float, ratio: () -> Float, state_x: () -> Float, state_y: () -> Float, state_z: () -> Float, Pax: () -> Float, Pby: () -> Float, Pcz: () -> Float
        ) {

            dst.vx(vx())

            dst.vy(vy())

            dst.vz(vz())

            dst.diff_pressure(diff_pressure())

            dst.EAS2TAS(EAS2TAS())

            dst.ratio(ratio())

            dst.state_x(state_x())

            dst.state_y(state_y())

            dst.state_z(state_z())

            dst.Pax(Pax())

            dst.Pby(Pby())

            dst.Pcz(Pcz())

        }

    }
}

inline class ATT_POS_MOCAP(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun q(): ATT_POS_MOCAP_q {

        return ATT_POS_MOCAP_q(data)
    }

    inline fun q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 8 + index * 4)
    }

    object q {
        const val item_len = 4


    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }


    companion object {

        val meta = Meta(200, 0, 0, 1, 36, 1, 288)


        inline fun push(src: ATT_POS_MOCAP, time_usec: (src: Long) -> Unit, q: (src: ATT_POS_MOCAP_q) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())
            src.q().let { item ->
                q(item)
            }

            x(src.x())

            y(src.y())

            z(src.z())

        }

        inline fun pull(dst: ATT_POS_MOCAP, time_usec: () -> Long, q: (dst: ATT_POS_MOCAP_q) -> Unit, x: () -> Float, y: () -> Float, z: () -> Float
        ) {

            dst.time_usec(time_usec())
            q(dst.q())

            dst.x(x())

            dst.y(y())

            dst.z(z())

        }

    }
}

inline class STATUSTEXT(val data: Cursor) {


    inline fun severity(): STATUSTEXT_severity? {
        if ((data.field_bit != 2 && !data.set_field(2, -1))) return null

        return STATUSTEXT_severity(data)
    }


    inline fun severity(src: MAV_SEVERITY) {
        if (data.field_bit != 2) data.set_field(2, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun text(): STATUSTEXT_text? {
        if ((data.field_bit != 3 && !data.set_field(3, -1))) return null

        return STATUSTEXT_text(data)
    }


    inline fun text_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -3 - 1, reuse)
    }


    inline fun text_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(3, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun text_(len: Int): STATUSTEXT_text {

        data.set_field(3, minOf(len, 255))
        return STATUSTEXT_text(data)
    }

    object text {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(124, 0, 0, 0, 1, 1, 2, 2, 2)


        inline fun push(src: STATUSTEXT, severity: (src: com.company.demo.GroundControl.MAV_SEVERITY) -> Unit, text: (src: STATUSTEXT_text) -> Unit
        ) {

            src.severity()?.let { item ->
                severity(item.get())
            }

            src.text()?.let { item -> text(item) }

        }

        inline fun pull(dst: STATUSTEXT, severity_exist: () -> Boolean, severity: () -> com.company.demo.GroundControl.MAV_SEVERITY, text_exist: () -> Int, text: (dst: STATUSTEXT_text) -> Unit
        ) {

            if (severity_exist()) dst.severity(severity())



            text_exist().let { len ->
                if (0 < len)
                    text(dst.text(len))
            }


        }

    }
}

inline class PING(val data: Pack.Bytes) {

    inline fun seq(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 12]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 13]).toByte()
    }


    companion object {

        val meta = Meta(122, 0, 1, 1, 14, 1, 112)


        inline fun push(src: PING, time_usec: (src: Long) -> Unit, seq: (src: Int) -> Unit, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit
        ) {

            time_usec(src.time_usec())

            seq(src.seq())

            target_system(src.target_system())

            target_component(src.target_component())

        }

    }
}

inline class GOPRO_GET_REQUEST(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    inline fun cmd_id(): GOPRO_GET_REQUEST_cmd_id? {
        if ((data.field_bit != 16 && !data.set_field(16, -1))) return null

        return GOPRO_GET_REQUEST_cmd_id(data)
    }


    inline fun cmd_id(src: GOPRO_COMMAND) {
        if (data.field_bit != 16) data.set_field(16, 0)

        set_bits((src.value).toULong().toLong(), 5, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(138, 0, 0, 0, 3, 1, 16, 0, 1)


        inline fun push(src: GOPRO_GET_REQUEST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, cmd_id: (src: com.company.demo.GroundControl.GOPRO_COMMAND) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.cmd_id()?.let { item ->
                cmd_id(item.get())
            }

        }

        inline fun pull(dst: GOPRO_GET_REQUEST, target_system: () -> Byte, target_component: () -> Byte, cmd_id_exist: () -> Boolean, cmd_id: () -> com.company.demo.GroundControl.GOPRO_COMMAND
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            if (cmd_id_exist()) dst.cmd_id(cmd_id())


        }

    }
}

inline class CAMERA_CAPTURE_STATUS(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun recording_time_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun recording_time_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun image_status(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun image_status_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun video_status(): Byte {
        return (data.bytes[data.origin + 9]).toByte()
    }

    inline fun video_status_(src: Byte) {
        data.bytes[data.origin + 9] = (src).toByte()
    }

    inline fun image_interval(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }

    inline fun image_interval_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun available_capacity(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun available_capacity_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }


    companion object {

        val meta = Meta(216, 0, 2, 0, 18, 1, 144)


        inline fun push(src: CAMERA_CAPTURE_STATUS, time_boot_ms: (src: Int) -> Unit, image_status: (src: Byte) -> Unit, video_status: (src: Byte) -> Unit, image_interval: (src: Float) -> Unit, recording_time_ms: (src: Int) -> Unit, available_capacity: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            image_status(src.image_status())

            video_status(src.video_status())

            image_interval(src.image_interval())

            recording_time_ms(src.recording_time_ms())

            available_capacity(src.available_capacity())

        }

        inline fun pull(dst: CAMERA_CAPTURE_STATUS, time_boot_ms: () -> Int, image_status: () -> Byte, video_status: () -> Byte, image_interval: () -> Float, recording_time_ms: () -> Int, available_capacity: () -> Float
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.image_status(image_status())

            dst.video_status(video_status())

            dst.image_interval(image_interval())

            dst.recording_time_ms(recording_time_ms())

            dst.available_capacity(available_capacity())

        }

    }
}

inline class GLOBAL_POSITION_INT(val data: Pack.Bytes) {

    inline fun hdg(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }


    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }


    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }


    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 14, 4)).toInt()
    }


    inline fun relative_alt(): Int {
        return (get_bytes(data.bytes, data.origin + 18, 4)).toInt()
    }


    inline fun vx(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }


    inline fun vy(): Short {
        return (get_bytes(data.bytes, data.origin + 24, 2)).toShort()
    }


    inline fun vz(): Short {
        return (get_bytes(data.bytes, data.origin + 26, 2)).toShort()
    }


    companion object {

        val meta = Meta(162, 1, 1, 0, 28, 1, 224)


        inline fun push(src: GLOBAL_POSITION_INT, time_boot_ms: (src: Int) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Int) -> Unit, relative_alt: (src: Int) -> Unit, vx: (src: Short) -> Unit, vy: (src: Short) -> Unit, vz: (src: Short) -> Unit, hdg: (src: Short) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            relative_alt(src.relative_alt())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            hdg(src.hdg())

        }

    }
}

inline class ENCAPSULATED_DATA(val data: Pack.Bytes) {

    inline fun seqnr(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun seqnr_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun daTa(): ENCAPSULATED_DATA_daTa {

        return ENCAPSULATED_DATA_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 253)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 253


    }


    companion object {

        val meta = Meta(170, 1, 0, 0, 255, 1, 2040)


        inline fun push(src: ENCAPSULATED_DATA, seqnr: (src: Short) -> Unit, daTa: (src: ENCAPSULATED_DATA_daTa) -> Unit
        ) {

            seqnr(src.seqnr())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: ENCAPSULATED_DATA, seqnr: () -> Short, daTa: (dst: ENCAPSULATED_DATA_daTa) -> Unit
        ) {

            dst.seqnr(seqnr())
            daTa(dst.daTa())

        }

    }
}

inline class GPS_INPUT(val data: Cursor) {

    inline fun time_week(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun time_week_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_week_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun time_week_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 6, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 6)
    }

    inline fun gps_id(): Byte {
        return (data.bytes[data.origin + 14]).toByte()
    }

    inline fun gps_id_(src: Byte) {
        data.bytes[data.origin + 14] = (src).toByte()
    }

    inline fun fix_type(): Byte {
        return (data.bytes[data.origin + 15]).toByte()
    }

    inline fun fix_type_(src: Byte) {
        data.bytes[data.origin + 15] = (src).toByte()
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun hdop(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun hdop_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun vdop(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun vdop_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun vn(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun vn_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun ve(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun ve_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun vd(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun vd_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }

    inline fun speed_accuracy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48, 4).toInt())
    }

    inline fun speed_accuracy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 48)
    }

    inline fun horiz_accuracy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 52, 4).toInt())
    }

    inline fun horiz_accuracy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 52)
    }

    inline fun vert_accuracy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 56, 4).toInt())
    }

    inline fun vert_accuracy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 56)
    }

    inline fun satellites_visible(): Byte {
        return (data.bytes[data.origin + 60]).toByte()
    }

    inline fun satellites_visible_(src: Byte) {
        data.bytes[data.origin + 60] = (src).toByte()
    }


    inline fun ignore_flags(): GPS_INPUT_ignore_flags? {
        if ((data.field_bit != 488 && !data.set_field(488, -1))) return null

        return GPS_INPUT_ignore_flags(data)
    }


    inline fun ignore_flags(src: GPS_INPUT_IGNORE_FLAGS) {
        if (data.field_bit != 488) data.set_field(488, 0)

        set_bits(GPS_INPUT_IGNORE_FLAGS.set(src).toLong(), 4, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(73, 1, 1, 1, 62, 1, 488, 0, 1)


        inline fun push(src: GPS_INPUT, time_usec: (src: Long) -> Unit, gps_id: (src: Byte) -> Unit, ignore_flags: (src: com.company.demo.GroundControl.GPS_INPUT_IGNORE_FLAGS) -> Unit, time_week_ms: (src: Int) -> Unit, time_week: (src: Short) -> Unit, fix_type: (src: Byte) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Float) -> Unit, hdop: (src: Float) -> Unit, vdop: (src: Float) -> Unit, vn: (src: Float) -> Unit, ve: (src: Float) -> Unit, vd: (src: Float) -> Unit, speed_accuracy: (src: Float) -> Unit, horiz_accuracy: (src: Float) -> Unit, vert_accuracy: (src: Float) -> Unit, satellites_visible: (src: Byte) -> Unit
        ) {

            time_usec(src.time_usec())

            gps_id(src.gps_id())

            src.ignore_flags()?.let { item ->
                ignore_flags(item.get())
            }

            time_week_ms(src.time_week_ms())

            time_week(src.time_week())

            fix_type(src.fix_type())

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            hdop(src.hdop())

            vdop(src.vdop())

            vn(src.vn())

            ve(src.ve())

            vd(src.vd())

            speed_accuracy(src.speed_accuracy())

            horiz_accuracy(src.horiz_accuracy())

            vert_accuracy(src.vert_accuracy())

            satellites_visible(src.satellites_visible())

        }

        inline fun pull(dst: GPS_INPUT, time_usec: () -> Long, gps_id: () -> Byte, ignore_flags_exist: () -> Boolean, ignore_flags: () -> com.company.demo.GroundControl.GPS_INPUT_IGNORE_FLAGS, time_week_ms: () -> Int, time_week: () -> Short, fix_type: () -> Byte, lat: () -> Int, lon: () -> Int, alt: () -> Float, hdop: () -> Float, vdop: () -> Float, vn: () -> Float, ve: () -> Float, vd: () -> Float, speed_accuracy: () -> Float, horiz_accuracy: () -> Float, vert_accuracy: () -> Float, satellites_visible: () -> Byte
        ) {

            dst.time_usec(time_usec())

            dst.gps_id(gps_id())

            if (ignore_flags_exist()) dst.ignore_flags(ignore_flags())


            dst.time_week_ms(time_week_ms())

            dst.time_week(time_week())

            dst.fix_type(fix_type())

            dst.lat(lat())

            dst.lon(lon())

            dst.alt(alt())

            dst.hdop(hdop())

            dst.vdop(vdop())

            dst.vn(vn())

            dst.ve(ve())

            dst.vd(vd())

            dst.speed_accuracy(speed_accuracy())

            dst.horiz_accuracy(horiz_accuracy())

            dst.vert_accuracy(vert_accuracy())

            dst.satellites_visible(satellites_visible())

        }

    }
}

inline class COMMAND_LONG(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun confirmation(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun param1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 3, 4).toInt())
    }


    inline fun param2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 7, 4).toInt())
    }


    inline fun param3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 11, 4).toInt())
    }


    inline fun param4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 15, 4).toInt())
    }


    inline fun param5(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 19, 4).toInt())
    }


    inline fun param6(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 23, 4).toInt())
    }


    inline fun param7(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 27, 4).toInt())
    }


    inline fun command(): COMMAND_LONG_command? {
        if ((data.field_bit != 250 && !data.set_field(250, -1))) return null

        return COMMAND_LONG_command(data)
    }


    companion object {

        val meta = Meta(101, 0, 0, 0, 32, 1, 250, 2, 1)


        inline fun push(src: COMMAND_LONG, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, command: (src: com.company.demo.GroundControl.MAV_CMD) -> Unit, confirmation: (src: Byte) -> Unit, param1: (src: Float) -> Unit, param2: (src: Float) -> Unit, param3: (src: Float) -> Unit, param4: (src: Float) -> Unit, param5: (src: Float) -> Unit, param6: (src: Float) -> Unit, param7: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.command()?.let { item ->
                command(item.get())
            }

            confirmation(src.confirmation())

            param1(src.param1())

            param2(src.param2())

            param3(src.param3())

            param4(src.param4())

            param5(src.param5())

            param6(src.param6())

            param7(src.param7())

        }

    }
}

inline class COMPASSMOT_STATUS(val data: Pack.Bytes) {

    inline fun throttle(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun throttle_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun interference(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun interference_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun current(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun current_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun CompensationX(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun CompensationX_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun CompensationY(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun CompensationY_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun CompensationZ(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun CompensationZ_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }


    companion object {

        val meta = Meta(43, 2, 0, 0, 20, 1, 160)


        inline fun push(src: COMPASSMOT_STATUS, throttle: (src: Short) -> Unit, current: (src: Float) -> Unit, interference: (src: Short) -> Unit, CompensationX: (src: Float) -> Unit, CompensationY: (src: Float) -> Unit, CompensationZ: (src: Float) -> Unit
        ) {

            throttle(src.throttle())

            current(src.current())

            interference(src.interference())

            CompensationX(src.CompensationX())

            CompensationY(src.CompensationY())

            CompensationZ(src.CompensationZ())

        }

        inline fun pull(dst: COMPASSMOT_STATUS, throttle: () -> Short, current: () -> Float, interference: () -> Short, CompensationX: () -> Float, CompensationY: () -> Float, CompensationZ: () -> Float
        ) {

            dst.throttle(throttle())

            dst.current(current())

            dst.interference(interference())

            dst.CompensationX(CompensationX())

            dst.CompensationY(CompensationY())

            dst.CompensationZ(CompensationZ())

        }

    }
}

inline class LOG_REQUEST_DATA(val data: Pack.Bytes) {

    inline fun id(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun id_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun ofs(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun ofs_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun count(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun count_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 11]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 11] = (src).toByte()
    }


    companion object {

        val meta = Meta(72, 1, 2, 0, 12, 1, 96)


        inline fun push(src: LOG_REQUEST_DATA, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, id: (src: Short) -> Unit, ofs: (src: Int) -> Unit, count: (src: Int) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            id(src.id())

            ofs(src.ofs())

            count(src.count())

        }

        inline fun pull(dst: LOG_REQUEST_DATA, target_system: () -> Byte, target_component: () -> Byte, id: () -> Short, ofs: () -> Int, count: () -> Int
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.id(id())

            dst.ofs(ofs())

            dst.count(count())

        }

    }
}

inline class GPS_RAW_INT(val data: Cursor) {

    inline fun eph(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun epv(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun vel(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun cog(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }


    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }


    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }


    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }


    inline fun alt(): Int {
        return (get_bytes(data.bytes, data.origin + 24, 4)).toInt()
    }


    inline fun satellites_visible(): Byte {
        return (data.bytes[data.origin + 28]).toByte()
    }


    inline fun fix_type(): GPS_RAW_INT_fix_type? {
        if ((data.field_bit != 234 && !data.set_field(234, -1))) return null

        return GPS_RAW_INT_fix_type(data)
    }


    inline fun alt_ellipsoid(): Boolean {
        return !(data.field_bit != 235 && !data.set_field(235, -1))
    }


    inline fun h_acc(): Boolean {
        return !(data.field_bit != 236 && !data.set_field(236, -1))
    }


    inline fun v_acc(): Boolean {
        return !(data.field_bit != 237 && !data.set_field(237, -1))
    }


    inline fun vel_acc(): Boolean {
        return !(data.field_bit != 238 && !data.set_field(238, -1))
    }


    inline fun hdg_acc(): Boolean {
        return !(data.field_bit != 239 && !data.set_field(239, -1))
    }


    companion object {

        val meta = Meta(56, 4, 0, 1, 30, 1, 234, 2, 6)


        inline fun push(src: GPS_RAW_INT, time_usec: (src: Long) -> Unit, fix_type: (src: com.company.demo.GroundControl.GPS_FIX_TYPE) -> Unit, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, alt: (src: Int) -> Unit, eph: (src: Short) -> Unit, epv: (src: Short) -> Unit, vel: (src: Short) -> Unit, cog: (src: Short) -> Unit, satellites_visible: (src: Byte) -> Unit, alt_ellipsoid: (src: Int) -> Unit, h_acc: (src: Int) -> Unit, v_acc: (src: Int) -> Unit, vel_acc: (src: Int) -> Unit, hdg_acc: (src: Int) -> Unit
        ) {

            time_usec(src.time_usec())

            src.fix_type()?.let { item ->
                fix_type(item.get())
            }

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            eph(src.eph())

            epv(src.epv())

            vel(src.vel())

            cog(src.cog())

            satellites_visible(src.satellites_visible())

            src.alt_ellipsoid().let { item ->
                alt_ellipsoid(item.get())
            }

            src.h_acc().let { item ->
                h_acc(item.get())
            }

            src.v_acc().let { item ->
                v_acc(item.get())
            }

            src.vel_acc().let { item ->
                vel_acc(item.get())
            }

            src.hdg_acc().let { item ->
                hdg_acc(item.get())
            }

        }

    }
}

inline class CAMERA_STATUS(val data: Cursor) {

    inline fun img_idx(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun img_idx_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 2, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 2)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun cam_idx(): Byte {
        return (data.bytes[data.origin + 11]).toByte()
    }

    inline fun cam_idx_(src: Byte) {
        data.bytes[data.origin + 11] = (src).toByte()
    }

    inline fun p1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun p1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun p2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun p2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun p3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun p3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun p4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun p4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }


    inline fun event_id(): CAMERA_STATUS_event_id? {
        if ((data.field_bit != 224 && !data.set_field(224, -1))) return null

        return CAMERA_STATUS_event_id(data)
    }


    inline fun event_id(src: CAMERA_STATUS_TYPES) {
        if (data.field_bit != 224) data.set_field(224, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(211, 1, 0, 1, 29, 1, 224, 0, 1)


        inline fun push(src: CAMERA_STATUS, time_usec: (src: Long) -> Unit, target_system: (src: Byte) -> Unit, cam_idx: (src: Byte) -> Unit, img_idx: (src: Short) -> Unit, event_id: (src: com.company.demo.GroundControl.CAMERA_STATUS_TYPES) -> Unit, p1: (src: Float) -> Unit, p2: (src: Float) -> Unit, p3: (src: Float) -> Unit, p4: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            target_system(src.target_system())

            cam_idx(src.cam_idx())

            img_idx(src.img_idx())

            src.event_id()?.let { item ->
                event_id(item.get())
            }

            p1(src.p1())

            p2(src.p2())

            p3(src.p3())

            p4(src.p4())

        }

        inline fun pull(dst: CAMERA_STATUS, time_usec: () -> Long, target_system: () -> Byte, cam_idx: () -> Byte, img_idx: () -> Short, event_id_exist: () -> Boolean, event_id: () -> com.company.demo.GroundControl.CAMERA_STATUS_TYPES, p1: () -> Float, p2: () -> Float, p3: () -> Float, p4: () -> Float
        ) {

            dst.time_usec(time_usec())

            dst.target_system(target_system())

            dst.cam_idx(cam_idx())

            dst.img_idx(img_idx())

            if (event_id_exist()) dst.event_id(event_id())


            dst.p1(p1())

            dst.p2(p2())

            dst.p3(p3())

            dst.p4(p4())

        }

    }
}

inline class RC_CHANNELS_SCALED(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun port(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun chan1_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 5, 2)).toShort()
    }


    inline fun chan2_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 7, 2)).toShort()
    }


    inline fun chan3_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 9, 2)).toShort()
    }


    inline fun chan4_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 11, 2)).toShort()
    }


    inline fun chan5_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 13, 2)).toShort()
    }


    inline fun chan6_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 15, 2)).toShort()
    }


    inline fun chan7_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 17, 2)).toShort()
    }


    inline fun chan8_scaled(): Short {
        return (get_bytes(data.bytes, data.origin + 19, 2)).toShort()
    }


    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 21]).toByte()
    }


    companion object {

        val meta = Meta(120, 0, 1, 0, 22, 1, 176)


        inline fun push(src: RC_CHANNELS_SCALED, time_boot_ms: (src: Int) -> Unit, port: (src: Byte) -> Unit, chan1_scaled: (src: Short) -> Unit, chan2_scaled: (src: Short) -> Unit, chan3_scaled: (src: Short) -> Unit, chan4_scaled: (src: Short) -> Unit, chan5_scaled: (src: Short) -> Unit, chan6_scaled: (src: Short) -> Unit, chan7_scaled: (src: Short) -> Unit, chan8_scaled: (src: Short) -> Unit, rssi: (src: Byte) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            port(src.port())

            chan1_scaled(src.chan1_scaled())

            chan2_scaled(src.chan2_scaled())

            chan3_scaled(src.chan3_scaled())

            chan4_scaled(src.chan4_scaled())

            chan5_scaled(src.chan5_scaled())

            chan6_scaled(src.chan6_scaled())

            chan7_scaled(src.chan7_scaled())

            chan8_scaled(src.chan8_scaled())

            rssi(src.rssi())

        }

    }
}

inline class CAMERA_SETTINGS(val data: Cursor) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }


    inline fun mode_id(): CAMERA_SETTINGS_mode_id? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return CAMERA_SETTINGS_mode_id(data)
    }


    inline fun mode_id(src: CAMERA_MODE) {
        if (data.field_bit != 32) data.set_field(32, 0)

        set_bits((src.value).toULong().toLong(), 2, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(181, 0, 1, 0, 5, 1, 32, 0, 1)


        inline fun push(src: CAMERA_SETTINGS, time_boot_ms: (src: Int) -> Unit, mode_id: (src: com.company.demo.GroundControl.CAMERA_MODE) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            src.mode_id()?.let { item ->
                mode_id(item.get())
            }

        }

        inline fun pull(dst: CAMERA_SETTINGS, time_boot_ms: () -> Int, mode_id_exist: () -> Boolean, mode_id: () -> com.company.demo.GroundControl.CAMERA_MODE
        ) {

            dst.time_boot_ms(time_boot_ms())

            if (mode_id_exist()) dst.mode_id(mode_id())


        }

    }
}

inline class DEVICE_OP_READ_REPLY(val data: Pack.Bytes) {

    inline fun request_id(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun request_id_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun result(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun result_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun regstart(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun regstart_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun count(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun count_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun daTa(): DEVICE_OP_READ_REPLY_daTa {

        return DEVICE_OP_READ_REPLY_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 128)

        for (index in 0 until len)
            data.bytes[data.origin + 7 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 128


    }


    companion object {

        val meta = Meta(165, 0, 1, 0, 135, 1, 1080)


        inline fun push(src: DEVICE_OP_READ_REPLY, request_id: (src: Int) -> Unit, result: (src: Byte) -> Unit, regstart: (src: Byte) -> Unit, count: (src: Byte) -> Unit, daTa: (src: DEVICE_OP_READ_REPLY_daTa) -> Unit
        ) {

            request_id(src.request_id())

            result(src.result())

            regstart(src.regstart())

            count(src.count())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: DEVICE_OP_READ_REPLY, request_id: () -> Int, result: () -> Byte, regstart: () -> Byte, count: () -> Byte, daTa: (dst: DEVICE_OP_READ_REPLY_daTa) -> Unit
        ) {

            dst.request_id(request_id())

            dst.result(result())

            dst.regstart(regstart())

            dst.count(count())
            daTa(dst.daTa())

        }

    }
}

inline class RAW_PRESSURE(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun press_abs(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun press_diff1(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun press_diff2(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    companion object {

        val meta = Meta(19, 0, 0, 1, 16, 1, 128)


        inline fun push(src: RAW_PRESSURE, time_usec: (src: Long) -> Unit, press_abs: (src: Short) -> Unit, press_diff1: (src: Short) -> Unit, press_diff2: (src: Short) -> Unit, temperature: (src: Short) -> Unit
        ) {

            time_usec(src.time_usec())

            press_abs(src.press_abs())

            press_diff1(src.press_diff1())

            press_diff2(src.press_diff2())

            temperature(src.temperature())

        }

    }
}

inline class DIGICAM_CONTROL(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun session(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun session_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun zoom_pos(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun zoom_pos_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun zoom_step(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun zoom_step_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun focus_lock(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun focus_lock_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun shot(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun shot_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun command_id(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }

    inline fun command_id_(src: Byte) {
        data.bytes[data.origin + 7] = (src).toByte()
    }

    inline fun extra_param(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun extra_param_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun extra_value(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 9, 4).toInt())
    }

    inline fun extra_value_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 9)
    }


    companion object {

        val meta = Meta(49, 0, 0, 0, 13, 1, 104)


        inline fun push(src: DIGICAM_CONTROL, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, session: (src: Byte) -> Unit, zoom_pos: (src: Byte) -> Unit, zoom_step: (src: Byte) -> Unit, focus_lock: (src: Byte) -> Unit, shot: (src: Byte) -> Unit, command_id: (src: Byte) -> Unit, extra_param: (src: Byte) -> Unit, extra_value: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            session(src.session())

            zoom_pos(src.zoom_pos())

            zoom_step(src.zoom_step())

            focus_lock(src.focus_lock())

            shot(src.shot())

            command_id(src.command_id())

            extra_param(src.extra_param())

            extra_value(src.extra_value())

        }

        inline fun pull(dst: DIGICAM_CONTROL, target_system: () -> Byte, target_component: () -> Byte, session: () -> Byte, zoom_pos: () -> Byte, zoom_step: () -> Byte, focus_lock: () -> Byte, shot: () -> Byte, command_id: () -> Byte, extra_param: () -> Byte, extra_value: () -> Float
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.session(session())

            dst.zoom_pos(zoom_pos())

            dst.zoom_step(zoom_step())

            dst.focus_lock(focus_lock())

            dst.shot(shot())

            dst.command_id(command_id())

            dst.extra_param(extra_param())

            dst.extra_value(extra_value())

        }

    }
}

inline class NAMED_VALUE_FLOAT(val data: Cursor) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun value(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun value_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }


    inline fun name(): NAMED_VALUE_FLOAT_name? {
        if ((data.field_bit != 66 && !data.set_field(66, -1))) return null

        return NAMED_VALUE_FLOAT_name(data)
    }


    inline fun name_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -66 - 1, reuse)
    }


    inline fun name_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(66, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun name_(len: Int): NAMED_VALUE_FLOAT_name {

        data.set_field(66, minOf(len, 255))
        return NAMED_VALUE_FLOAT_name(data)
    }

    object name {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(8, 0, 1, 0, 9, 1, 66, 2, 1)


        inline fun push(src: NAMED_VALUE_FLOAT, time_boot_ms: (src: Int) -> Unit, name: (src: NAMED_VALUE_FLOAT_name) -> Unit, value: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            src.name()?.let { item -> name(item) }

            value(src.value())

        }

        inline fun pull(dst: NAMED_VALUE_FLOAT, time_boot_ms: () -> Int, name_exist: () -> Int, name: (dst: NAMED_VALUE_FLOAT_name) -> Unit, value: () -> Float
        ) {

            dst.time_boot_ms(time_boot_ms())


            name_exist().let { len ->
                if (0 < len)
                    name(dst.name(len))
            }


            dst.value(value())

        }

    }
}

inline class GOPRO_HEARTBEAT(val data: Cursor) {


    inline fun status(): GOPRO_HEARTBEAT_status? {
        if ((data.field_bit != 2 && !data.set_field(2, -1))) return null

        return GOPRO_HEARTBEAT_status(data)
    }


    inline fun status(src: GOPRO_HEARTBEAT_STATUS) {
        if (data.field_bit != 2) data.set_field(2, 0)

        set_bits((src.value).toULong().toLong(), 2, data.bytes, data.BIT)
    }


    inline fun capture_mode(): GOPRO_HEARTBEAT_capture_mode? {
        if ((data.field_bit != 3 && !data.set_field(3, -1))) return null

        return GOPRO_HEARTBEAT_capture_mode(data)
    }


    inline fun capture_mode(src: GOPRO_CAPTURE_MODE) {
        if (data.field_bit != 3) data.set_field(3, 0)

        set_bits(GOPRO_CAPTURE_MODE.set(src).toLong(), 4, data.bytes, data.BIT)
    }


    inline fun flags(): GOPRO_HEARTBEAT_flags? {
        if ((data.field_bit != 4 && !data.set_field(4, -1))) return null

        return GOPRO_HEARTBEAT_flags(data)
    }


    inline fun flags(src: GOPRO_HEARTBEAT_FLAGS) {
        if (data.field_bit != 4) data.set_field(4, 0)

        set_bits((-1 src . value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(175, 0, 0, 0, 1, 1, 2, 2, 3)


        inline fun push(src: GOPRO_HEARTBEAT, status: (src: com.company.demo.GroundControl.GOPRO_HEARTBEAT_STATUS) -> Unit, capture_mode: (src: com.company.demo.GroundControl.GOPRO_CAPTURE_MODE) -> Unit, flags: (src: com.company.demo.GroundControl.GOPRO_HEARTBEAT_FLAGS) -> Unit
        ) {

            src.status()?.let { item ->
                status(item.get())
            }

            src.capture_mode()?.let { item ->
                capture_mode(item.get())
            }

            src.flags()?.let { item ->
                flags(item.get())
            }

        }

        inline fun pull(dst: GOPRO_HEARTBEAT, status_exist: () -> Boolean, status: () -> com.company.demo.GroundControl.GOPRO_HEARTBEAT_STATUS, capture_mode_exist: () -> Boolean, capture_mode: () -> com.company.demo.GroundControl.GOPRO_CAPTURE_MODE, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.GOPRO_HEARTBEAT_FLAGS
        ) {

            if (status_exist()) dst.status(status())


            if (capture_mode_exist()) dst.capture_mode(capture_mode())


            if (flags_exist()) dst.flags(flags())


        }

    }
}

inline class ATTITUDE(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    companion object {

        val meta = Meta(134, 0, 1, 0, 28, 1, 224)


        inline fun push(src: ATTITUDE, time_boot_ms: (src: Int) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit, rollspeed: (src: Float) -> Unit, pitchspeed: (src: Float) -> Unit, yawspeed: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

            rollspeed(src.rollspeed())

            pitchspeed(src.pitchspeed())

            yawspeed(src.yawspeed())

        }

    }
}

inline class MISSION_WRITE_PARTIAL_LIST(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun start_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun end_index(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun mission_type(): MISSION_WRITE_PARTIAL_LIST_mission_type? {
        if ((data.field_bit != 48 && !data.set_field(48, -1))) return null

        return MISSION_WRITE_PARTIAL_LIST_mission_type(data)
    }


    companion object {

        val meta = Meta(145, 0, 0, 0, 7, 1, 48, 0, 1)


        inline fun push(src: MISSION_WRITE_PARTIAL_LIST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, start_index: (src: Short) -> Unit, end_index: (src: Short) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            start_index(src.start_index())

            end_index(src.end_index())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class AHRS2(val data: Pack.Bytes) {

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun altitude(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun altitude_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun lng(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }

    inline fun lng_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }


    companion object {

        val meta = Meta(17, 0, 0, 0, 24, 1, 192)


        inline fun push(src: AHRS2, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit, altitude: (src: Float) -> Unit, lat: (src: Int) -> Unit, lng: (src: Int) -> Unit
        ) {

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

            altitude(src.altitude())

            lat(src.lat())

            lng(src.lng())

        }

        inline fun pull(dst: AHRS2, roll: () -> Float, pitch: () -> Float, yaw: () -> Float, altitude: () -> Float, lat: () -> Int, lng: () -> Int
        ) {

            dst.roll(roll())

            dst.pitch(pitch())

            dst.yaw(yaw())

            dst.altitude(altitude())

            dst.lat(lat())

            dst.lng(lng())

        }

    }
}

inline class LOG_ERASE(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    companion object {

        val meta = Meta(4, 0, 0, 0, 2, 1, 16)


        inline fun push(src: LOG_ERASE, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

        }

        inline fun pull(dst: LOG_ERASE, target_system: () -> Byte, target_component: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

        }

    }
}

inline class TERRAIN_REQUEST(val data: Pack.Bytes) {

    inline fun grid_spacing(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun grid_spacing_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun mask(): Long {
        return (get_bytes(data.bytes, data.origin + 2, 8)).toLong()
    }

    inline fun mask_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 2)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 14, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }


    companion object {

        val meta = Meta(92, 1, 0, 1, 18, 1, 144)


        inline fun push(src: TERRAIN_REQUEST, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, grid_spacing: (src: Short) -> Unit, mask: (src: Long) -> Unit
        ) {

            lat(src.lat())

            lon(src.lon())

            grid_spacing(src.grid_spacing())

            mask(src.mask())

        }

        inline fun pull(dst: TERRAIN_REQUEST, lat: () -> Int, lon: () -> Int, grid_spacing: () -> Short, mask: () -> Long
        ) {

            dst.lat(lat())

            dst.lon(lon())

            dst.grid_spacing(grid_spacing())

            dst.mask(mask())

        }

    }
}

inline class MOUNT_STATUS(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun pointing_a(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun pointing_a_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun pointing_b(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun pointing_b_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun pointing_c(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun pointing_c_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }


    companion object {

        val meta = Meta(44, 0, 0, 0, 14, 1, 112)


        inline fun push(src: MOUNT_STATUS, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, pointing_a: (src: Int) -> Unit, pointing_b: (src: Int) -> Unit, pointing_c: (src: Int) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            pointing_a(src.pointing_a())

            pointing_b(src.pointing_b())

            pointing_c(src.pointing_c())

        }

        inline fun pull(dst: MOUNT_STATUS, target_system: () -> Byte, target_component: () -> Byte, pointing_a: () -> Int, pointing_b: () -> Int, pointing_c: () -> Int
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.pointing_a(pointing_a())

            dst.pointing_b(pointing_b())

            dst.pointing_c(pointing_c())

        }

    }
}

inline class MANUAL_SETPOINT(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun thrust(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun mode_switch(): Byte {
        return (data.bytes[data.origin + 20]).toByte()
    }


    inline fun manual_override_switch(): Byte {
        return (data.bytes[data.origin + 21]).toByte()
    }


    companion object {

        val meta = Meta(219, 0, 1, 0, 22, 1, 176)


        inline fun push(src: MANUAL_SETPOINT, time_boot_ms: (src: Int) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit, thrust: (src: Float) -> Unit, mode_switch: (src: Byte) -> Unit, manual_override_switch: (src: Byte) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

            thrust(src.thrust())

            mode_switch(src.mode_switch())

            manual_override_switch(src.manual_override_switch())

        }

    }
}

inline class PID_TUNING(val data: Cursor) {

    inline fun desired(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun desired_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun achieved(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun achieved_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun FF(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun FF_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun P(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun P_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun I(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun I_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun D(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun D_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }


    inline fun axis(): PID_TUNING_axis? {
        if ((data.field_bit != 192 && !data.set_field(192, -1))) return null

        return PID_TUNING_axis(data)
    }


    inline fun axis(src: PID_TUNING_AXIS) {
        if (data.field_bit != 192) data.set_field(192, 0)

        set_bits((-1 src . value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(64, 0, 0, 0, 25, 1, 192, 0, 1)


        inline fun push(src: PID_TUNING, axis: (src: com.company.demo.GroundControl.PID_TUNING_AXIS) -> Unit, desired: (src: Float) -> Unit, achieved: (src: Float) -> Unit, FF: (src: Float) -> Unit, P: (src: Float) -> Unit, I: (src: Float) -> Unit, D: (src: Float) -> Unit
        ) {

            src.axis()?.let { item ->
                axis(item.get())
            }

            desired(src.desired())

            achieved(src.achieved())

            FF(src.FF())

            P(src.P())

            I(src.I())

            D(src.D())

        }

        inline fun pull(dst: PID_TUNING, axis_exist: () -> Boolean, axis: () -> com.company.demo.GroundControl.PID_TUNING_AXIS, desired: () -> Float, achieved: () -> Float, FF: () -> Float, P: () -> Float, I: () -> Float, D: () -> Float
        ) {

            if (axis_exist()) dst.axis(axis())


            dst.desired(desired())

            dst.achieved(achieved())

            dst.FF(FF())

            dst.P(P())

            dst.I(I())

            dst.D(D())

        }

    }
}

inline class SAFETY_ALLOWED_AREA(val data: Cursor) {

    inline fun p1x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }


    inline fun p1y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun p1z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun p2x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun p2y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun p2z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun frame(): SAFETY_ALLOWED_AREA_frame? {
        if ((data.field_bit != 192 && !data.set_field(192, -1))) return null

        return SAFETY_ALLOWED_AREA_frame(data)
    }


    companion object {

        val meta = Meta(179, 0, 0, 0, 25, 1, 192, 0, 1)


        inline fun push(src: SAFETY_ALLOWED_AREA, frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, p1x: (src: Float) -> Unit, p1y: (src: Float) -> Unit, p1z: (src: Float) -> Unit, p2x: (src: Float) -> Unit, p2y: (src: Float) -> Unit, p2z: (src: Float) -> Unit
        ) {

            src.frame()?.let { item ->
                frame(item.get())
            }

            p1x(src.p1x())

            p1y(src.p1y())

            p1z(src.p1z())

            p2x(src.p2x())

            p2y(src.p2y())

            p2z(src.p2z())

        }

    }
}

inline class OPTICAL_FLOW_RAD(val data: Pack.Bytes) {

    inline fun integration_time_us(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun integration_time_us_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun time_delta_distance_us(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun time_delta_distance_us_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }

    inline fun sensor_id(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }

    inline fun sensor_id_(src: Byte) {
        data.bytes[data.origin + 16] = (src).toByte()
    }

    inline fun integrated_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 17, 4).toInt())
    }

    inline fun integrated_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }

    inline fun integrated_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 21, 4).toInt())
    }

    inline fun integrated_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 21)
    }

    inline fun integrated_xgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 25, 4).toInt())
    }

    inline fun integrated_xgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 25)
    }

    inline fun integrated_ygyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 29, 4).toInt())
    }

    inline fun integrated_ygyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 29)
    }

    inline fun integrated_zgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 33, 4).toInt())
    }

    inline fun integrated_zgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 33)
    }

    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 37, 2)).toShort()
    }

    inline fun temperature_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 37)
    }

    inline fun quality(): Byte {
        return (data.bytes[data.origin + 39]).toByte()
    }

    inline fun quality_(src: Byte) {
        data.bytes[data.origin + 39] = (src).toByte()
    }

    inline fun distance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun distance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }


    companion object {

        val meta = Meta(202, 0, 2, 1, 44, 1, 352)


        inline fun push(src: OPTICAL_FLOW_RAD, time_usec: (src: Long) -> Unit, sensor_id: (src: Byte) -> Unit, integration_time_us: (src: Int) -> Unit, integrated_x: (src: Float) -> Unit, integrated_y: (src: Float) -> Unit, integrated_xgyro: (src: Float) -> Unit, integrated_ygyro: (src: Float) -> Unit, integrated_zgyro: (src: Float) -> Unit, temperature: (src: Short) -> Unit, quality: (src: Byte) -> Unit, time_delta_distance_us: (src: Int) -> Unit, distance: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            sensor_id(src.sensor_id())

            integration_time_us(src.integration_time_us())

            integrated_x(src.integrated_x())

            integrated_y(src.integrated_y())

            integrated_xgyro(src.integrated_xgyro())

            integrated_ygyro(src.integrated_ygyro())

            integrated_zgyro(src.integrated_zgyro())

            temperature(src.temperature())

            quality(src.quality())

            time_delta_distance_us(src.time_delta_distance_us())

            distance(src.distance())

        }

        inline fun pull(dst: OPTICAL_FLOW_RAD, time_usec: () -> Long, sensor_id: () -> Byte, integration_time_us: () -> Int, integrated_x: () -> Float, integrated_y: () -> Float, integrated_xgyro: () -> Float, integrated_ygyro: () -> Float, integrated_zgyro: () -> Float, temperature: () -> Short, quality: () -> Byte, time_delta_distance_us: () -> Int, distance: () -> Float
        ) {

            dst.time_usec(time_usec())

            dst.sensor_id(sensor_id())

            dst.integration_time_us(integration_time_us())

            dst.integrated_x(integrated_x())

            dst.integrated_y(integrated_y())

            dst.integrated_xgyro(integrated_xgyro())

            dst.integrated_ygyro(integrated_ygyro())

            dst.integrated_zgyro(integrated_zgyro())

            dst.temperature(temperature())

            dst.quality(quality())

            dst.time_delta_distance_us(time_delta_distance_us())

            dst.distance(distance())

        }

    }
}

inline class LOG_DATA(val data: Pack.Bytes) {

    inline fun id(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun id_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun ofs(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun ofs_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun count(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun count_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun daTa(): LOG_DATA_daTa {

        return LOG_DATA_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 90)

        for (index in 0 until len)
            data.bytes[data.origin + 7 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 90


    }


    companion object {

        val meta = Meta(204, 1, 1, 0, 97, 1, 776)


        inline fun push(src: LOG_DATA, id: (src: Short) -> Unit, ofs: (src: Int) -> Unit, count: (src: Byte) -> Unit, daTa: (src: LOG_DATA_daTa) -> Unit
        ) {

            id(src.id())

            ofs(src.ofs())

            count(src.count())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: LOG_DATA, id: () -> Short, ofs: () -> Int, count: () -> Byte, daTa: (dst: LOG_DATA_daTa) -> Unit
        ) {

            dst.id(id())

            dst.ofs(ofs())

            dst.count(count())
            daTa(dst.daTa())

        }

    }
}

inline class MISSION_CLEAR_ALL(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun mission_type(): MISSION_CLEAR_ALL_mission_type? {
        if ((data.field_bit != 16 && !data.set_field(16, -1))) return null

        return MISSION_CLEAR_ALL_mission_type(data)
    }


    companion object {

        val meta = Meta(193, 0, 0, 0, 3, 1, 16, 0, 1)


        inline fun push(src: MISSION_CLEAR_ALL, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class AHRS3(val data: Pack.Bytes) {

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun altitude(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun altitude_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun lng(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }

    inline fun lng_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun v1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun v1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun v2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun v2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun v3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun v3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun v4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun v4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }


    companion object {

        val meta = Meta(65, 0, 0, 0, 40, 1, 320)


        inline fun push(src: AHRS3, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit, altitude: (src: Float) -> Unit, lat: (src: Int) -> Unit, lng: (src: Int) -> Unit, v1: (src: Float) -> Unit, v2: (src: Float) -> Unit, v3: (src: Float) -> Unit, v4: (src: Float) -> Unit
        ) {

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

            altitude(src.altitude())

            lat(src.lat())

            lng(src.lng())

            v1(src.v1())

            v2(src.v2())

            v3(src.v3())

            v4(src.v4())

        }

        inline fun pull(dst: AHRS3, roll: () -> Float, pitch: () -> Float, yaw: () -> Float, altitude: () -> Float, lat: () -> Int, lng: () -> Int, v1: () -> Float, v2: () -> Float, v3: () -> Float, v4: () -> Float
        ) {

            dst.roll(roll())

            dst.pitch(pitch())

            dst.yaw(yaw())

            dst.altitude(altitude())

            dst.lat(lat())

            dst.lng(lng())

            dst.v1(v1())

            dst.v2(v2())

            dst.v3(v3())

            dst.v4(v4())

        }

    }
}

inline class VICON_POSITION_ESTIMATE(val data: Pack.Bytes) {

    inline fun usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }


    companion object {

        val meta = Meta(135, 0, 0, 1, 32, 1, 256)


        inline fun push(src: VICON_POSITION_ESTIMATE, usec: (src: Long) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit
        ) {

            usec(src.usec())

            x(src.x())

            y(src.y())

            z(src.z())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

        }

        inline fun pull(dst: VICON_POSITION_ESTIMATE, usec: () -> Long, x: () -> Float, y: () -> Float, z: () -> Float, roll: () -> Float, pitch: () -> Float, yaw: () -> Float
        ) {

            dst.usec(usec())

            dst.x(x())

            dst.y(y())

            dst.z(z())

            dst.roll(roll())

            dst.pitch(pitch())

            dst.yaw(yaw())

        }

    }
}

inline class GPS2_RTK(val data: Pack.Bytes) {

    inline fun wn(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun wn_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_last_baseline_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun time_last_baseline_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun tow(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun tow_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun accuracy(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun accuracy_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun rtk_receiver_id(): Byte {
        return (data.bytes[data.origin + 14]).toByte()
    }

    inline fun rtk_receiver_id_(src: Byte) {
        data.bytes[data.origin + 14] = (src).toByte()
    }

    inline fun rtk_health(): Byte {
        return (data.bytes[data.origin + 15]).toByte()
    }

    inline fun rtk_health_(src: Byte) {
        data.bytes[data.origin + 15] = (src).toByte()
    }

    inline fun rtk_rate(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }

    inline fun rtk_rate_(src: Byte) {
        data.bytes[data.origin + 16] = (src).toByte()
    }

    inline fun nsats(): Byte {
        return (data.bytes[data.origin + 17]).toByte()
    }

    inline fun nsats_(src: Byte) {
        data.bytes[data.origin + 17] = (src).toByte()
    }

    inline fun baseline_coords_type(): Byte {
        return (data.bytes[data.origin + 18]).toByte()
    }

    inline fun baseline_coords_type_(src: Byte) {
        data.bytes[data.origin + 18] = (src).toByte()
    }

    inline fun baseline_a_mm(): Int {
        return (get_bytes(data.bytes, data.origin + 19, 4)).toInt()
    }

    inline fun baseline_a_mm_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 19)
    }

    inline fun baseline_b_mm(): Int {
        return (get_bytes(data.bytes, data.origin + 23, 4)).toInt()
    }

    inline fun baseline_b_mm_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 23)
    }

    inline fun baseline_c_mm(): Int {
        return (get_bytes(data.bytes, data.origin + 27, 4)).toInt()
    }

    inline fun baseline_c_mm_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 27)
    }

    inline fun iar_num_hypotheses(): Int {
        return (get_bytes(data.bytes, data.origin + 31, 4)).toInt()
    }

    inline fun iar_num_hypotheses_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 31)
    }


    companion object {

        val meta = Meta(91, 1, 3, 0, 35, 1, 280)


        inline fun push(src: GPS2_RTK, time_last_baseline_ms: (src: Int) -> Unit, rtk_receiver_id: (src: Byte) -> Unit, wn: (src: Short) -> Unit, tow: (src: Int) -> Unit, rtk_health: (src: Byte) -> Unit, rtk_rate: (src: Byte) -> Unit, nsats: (src: Byte) -> Unit, baseline_coords_type: (src: Byte) -> Unit, baseline_a_mm: (src: Int) -> Unit, baseline_b_mm: (src: Int) -> Unit, baseline_c_mm: (src: Int) -> Unit, accuracy: (src: Int) -> Unit, iar_num_hypotheses: (src: Int) -> Unit
        ) {

            time_last_baseline_ms(src.time_last_baseline_ms())

            rtk_receiver_id(src.rtk_receiver_id())

            wn(src.wn())

            tow(src.tow())

            rtk_health(src.rtk_health())

            rtk_rate(src.rtk_rate())

            nsats(src.nsats())

            baseline_coords_type(src.baseline_coords_type())

            baseline_a_mm(src.baseline_a_mm())

            baseline_b_mm(src.baseline_b_mm())

            baseline_c_mm(src.baseline_c_mm())

            accuracy(src.accuracy())

            iar_num_hypotheses(src.iar_num_hypotheses())

        }

        inline fun pull(dst: GPS2_RTK, time_last_baseline_ms: () -> Int, rtk_receiver_id: () -> Byte, wn: () -> Short, tow: () -> Int, rtk_health: () -> Byte, rtk_rate: () -> Byte, nsats: () -> Byte, baseline_coords_type: () -> Byte, baseline_a_mm: () -> Int, baseline_b_mm: () -> Int, baseline_c_mm: () -> Int, accuracy: () -> Int, iar_num_hypotheses: () -> Int
        ) {

            dst.time_last_baseline_ms(time_last_baseline_ms())

            dst.rtk_receiver_id(rtk_receiver_id())

            dst.wn(wn())

            dst.tow(tow())

            dst.rtk_health(rtk_health())

            dst.rtk_rate(rtk_rate())

            dst.nsats(nsats())

            dst.baseline_coords_type(baseline_coords_type())

            dst.baseline_a_mm(baseline_a_mm())

            dst.baseline_b_mm(baseline_b_mm())

            dst.baseline_c_mm(baseline_c_mm())

            dst.accuracy(accuracy())

            dst.iar_num_hypotheses(iar_num_hypotheses())

        }

    }
}

inline class MAG_CAL_REPORT(val data: Cursor) {

    inline fun compass_id(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun compass_id_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun cal_mask(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun cal_mask_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun autosaved(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun autosaved_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun fitness(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 3, 4).toInt())
    }

    inline fun fitness_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 3)
    }

    inline fun ofs_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 7, 4).toInt())
    }

    inline fun ofs_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 7)
    }

    inline fun ofs_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 11, 4).toInt())
    }

    inline fun ofs_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 11)
    }

    inline fun ofs_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 15, 4).toInt())
    }

    inline fun ofs_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 15)
    }

    inline fun diag_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 19, 4).toInt())
    }

    inline fun diag_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 19)
    }

    inline fun diag_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 23, 4).toInt())
    }

    inline fun diag_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 23)
    }

    inline fun diag_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 27, 4).toInt())
    }

    inline fun diag_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 27)
    }

    inline fun offdiag_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 31, 4).toInt())
    }

    inline fun offdiag_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 31)
    }

    inline fun offdiag_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 35, 4).toInt())
    }

    inline fun offdiag_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 35)
    }

    inline fun offdiag_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 39, 4).toInt())
    }

    inline fun offdiag_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 39)
    }


    inline fun cal_status(): MAG_CAL_REPORT_cal_status? {
        if ((data.field_bit != 344 && !data.set_field(344, -1))) return null

        return MAG_CAL_REPORT_cal_status(data)
    }


    inline fun cal_status(src: MAG_CAL_STATUS) {
        if (data.field_bit != 344) data.set_field(344, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(63, 0, 0, 0, 44, 1, 344, 0, 1)


        inline fun push(src: MAG_CAL_REPORT, compass_id: (src: Byte) -> Unit, cal_mask: (src: Byte) -> Unit, cal_status: (src: com.company.demo.GroundControl.MAG_CAL_STATUS) -> Unit, autosaved: (src: Byte) -> Unit, fitness: (src: Float) -> Unit, ofs_x: (src: Float) -> Unit, ofs_y: (src: Float) -> Unit, ofs_z: (src: Float) -> Unit, diag_x: (src: Float) -> Unit, diag_y: (src: Float) -> Unit, diag_z: (src: Float) -> Unit, offdiag_x: (src: Float) -> Unit, offdiag_y: (src: Float) -> Unit, offdiag_z: (src: Float) -> Unit
        ) {

            compass_id(src.compass_id())

            cal_mask(src.cal_mask())

            src.cal_status()?.let { item ->
                cal_status(item.get())
            }

            autosaved(src.autosaved())

            fitness(src.fitness())

            ofs_x(src.ofs_x())

            ofs_y(src.ofs_y())

            ofs_z(src.ofs_z())

            diag_x(src.diag_x())

            diag_y(src.diag_y())

            diag_z(src.diag_z())

            offdiag_x(src.offdiag_x())

            offdiag_y(src.offdiag_y())

            offdiag_z(src.offdiag_z())

        }

        inline fun pull(dst: MAG_CAL_REPORT, compass_id: () -> Byte, cal_mask: () -> Byte, cal_status_exist: () -> Boolean, cal_status: () -> com.company.demo.GroundControl.MAG_CAL_STATUS, autosaved: () -> Byte, fitness: () -> Float, ofs_x: () -> Float, ofs_y: () -> Float, ofs_z: () -> Float, diag_x: () -> Float, diag_y: () -> Float, diag_z: () -> Float, offdiag_x: () -> Float, offdiag_y: () -> Float, offdiag_z: () -> Float
        ) {

            dst.compass_id(compass_id())

            dst.cal_mask(cal_mask())

            if (cal_status_exist()) dst.cal_status(cal_status())


            dst.autosaved(autosaved())

            dst.fitness(fitness())

            dst.ofs_x(ofs_x())

            dst.ofs_y(ofs_y())

            dst.ofs_z(ofs_z())

            dst.diag_x(diag_x())

            dst.diag_y(diag_y())

            dst.diag_z(diag_z())

            dst.offdiag_x(offdiag_x())

            dst.offdiag_y(offdiag_y())

            dst.offdiag_z(offdiag_z())

        }

    }
}

inline class LOG_REQUEST_LIST(val data: Pack.Bytes) {

    inline fun start(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun start_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun end(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun end_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }


    companion object {

        val meta = Meta(37, 2, 0, 0, 6, 1, 48)


        inline fun push(src: LOG_REQUEST_LIST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, start: (src: Short) -> Unit, end: (src: Short) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            start(src.start())

            end(src.end())

        }

        inline fun pull(dst: LOG_REQUEST_LIST, target_system: () -> Byte, target_component: () -> Byte, start: () -> Short, end: () -> Short
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.start(start())

            dst.end(end())

        }

    }
}

inline class SCALED_PRESSURE(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun press_abs(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun press_diff(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    companion object {

        val meta = Meta(18, 0, 1, 0, 14, 1, 112)


        inline fun push(src: SCALED_PRESSURE, time_boot_ms: (src: Int) -> Unit, press_abs: (src: Float) -> Unit, press_diff: (src: Float) -> Unit, temperature: (src: Short) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            press_abs(src.press_abs())

            press_diff(src.press_diff())

            temperature(src.temperature())

        }

    }
}

inline class V2_EXTENSION(val data: Pack.Bytes) {

    inline fun message_type(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun message_type_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_network(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_network_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun payload(): V2_EXTENSION_payload {

        return V2_EXTENSION_payload(data)
    }

    inline fun payload_(src: ByteArray) {
        val len = minOf(src.size, 249)

        for (index in 0 until len)
            data.bytes[data.origin + 5 + index] = (src[index]).toByte()
    }

    object payload {
        const val item_len = 249


    }


    companion object {

        val meta = Meta(174, 1, 0, 0, 254, 1, 2032)


        inline fun push(src: V2_EXTENSION, target_network: (src: Byte) -> Unit, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, message_type: (src: Short) -> Unit, payload: (src: V2_EXTENSION_payload) -> Unit
        ) {

            target_network(src.target_network())

            target_system(src.target_system())

            target_component(src.target_component())

            message_type(src.message_type())
            src.payload().let { item ->
                payload(item)
            }

        }

        inline fun pull(dst: V2_EXTENSION, target_network: () -> Byte, target_system: () -> Byte, target_component: () -> Byte, message_type: () -> Short, payload: (dst: V2_EXTENSION_payload) -> Unit
        ) {

            dst.target_network(target_network())

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.message_type(message_type())
            payload(dst.payload())

        }

    }
}

inline class HEARTBEAT(val data: Cursor) {

    inline fun custom_mode(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun mavlink_version(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun typE(): HEARTBEAT_typE? {
        if ((data.field_bit != 42 && !data.set_field(42, -1))) return null

        return HEARTBEAT_typE(data)
    }


    inline fun autopilot(): HEARTBEAT_autopilot? {
        if ((data.field_bit != 43 && !data.set_field(43, -1))) return null

        return HEARTBEAT_autopilot(data)
    }


    inline fun base_mode(): HEARTBEAT_base_mode? {
        if ((data.field_bit != 44 && !data.set_field(44, -1))) return null

        return HEARTBEAT_base_mode(data)
    }


    inline fun system_status(): HEARTBEAT_system_status? {
        if ((data.field_bit != 45 && !data.set_field(45, -1))) return null

        return HEARTBEAT_system_status(data)
    }


    companion object {

        val meta = Meta(125, 0, 1, 0, 6, 1, 42, 2, 4)


        inline fun push(src: HEARTBEAT, typE: (src: com.company.demo.GroundControl.MAV_TYPE) -> Unit, autopilot: (src: com.company.demo.GroundControl.MAV_AUTOPILOT) -> Unit, base_mode: (src: com.company.demo.GroundControl.MAV_MODE_FLAG) -> Unit, custom_mode: (src: Int) -> Unit, system_status: (src: com.company.demo.GroundControl.MAV_STATE) -> Unit, mavlink_version: (src: Byte) -> Unit
        ) {

            src.typE()?.let { item ->
                typE(item.get())
            }

            src.autopilot()?.let { item ->
                autopilot(item.get())
            }

            src.base_mode()?.let { item ->
                base_mode(item.get())
            }

            custom_mode(src.custom_mode())

            src.system_status()?.let { item ->
                system_status(item.get())
            }

            mavlink_version(src.mavlink_version())

        }

    }
}

inline class PARAM_MAP_RC(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun param_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun parameter_rc_channel_index(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun param_value0(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 5, 4).toInt())
    }


    inline fun scale(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 9, 4).toInt())
    }


    inline fun param_value_min(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 13, 4).toInt())
    }


    inline fun param_value_max(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 17, 4).toInt())
    }


    inline fun param_id(): PARAM_MAP_RC_param_id? {
        if ((data.field_bit != 170 && !data.set_field(170, -1))) return null

        return PARAM_MAP_RC_param_id(data)
    }


    object param_id {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(29, 0, 0, 0, 22, 1, 170, 2, 1)


        inline fun push(src: PARAM_MAP_RC, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, param_id: (src: PARAM_MAP_RC_param_id) -> Unit, param_index: (src: Short) -> Unit, parameter_rc_channel_index: (src: Byte) -> Unit, param_value0: (src: Float) -> Unit, scale: (src: Float) -> Unit, param_value_min: (src: Float) -> Unit, param_value_max: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.param_id()?.let { item -> param_id(item) }

            param_index(src.param_index())

            parameter_rc_channel_index(src.parameter_rc_channel_index())

            param_value0(src.param_value0())

            scale(src.scale())

            param_value_min(src.param_value_min())

            param_value_max(src.param_value_max())

        }

    }
}

inline class POWER_STATUS(val data: Cursor) {

    inline fun Vcc(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun Vcc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun Vservo(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun Vservo_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }


    inline fun flags(): POWER_STATUS_flags? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return POWER_STATUS_flags(data)
    }


    inline fun flags(src: MAV_POWER_STATUS) {
        if (data.field_bit != 32) data.set_field(32, 0)

        set_bits(MAV_POWER_STATUS.set(src).toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(156, 2, 0, 0, 5, 1, 32, 0, 1)


        inline fun push(src: POWER_STATUS, Vcc: (src: Short) -> Unit, Vservo: (src: Short) -> Unit, flags: (src: com.company.demo.GroundControl.MAV_POWER_STATUS) -> Unit
        ) {

            Vcc(src.Vcc())

            Vservo(src.Vservo())

            src.flags()?.let { item ->
                flags(item.get())
            }

        }

        inline fun pull(dst: POWER_STATUS, Vcc: () -> Short, Vservo: () -> Short, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.MAV_POWER_STATUS
        ) {

            dst.Vcc(Vcc())

            dst.Vservo(Vservo())

            if (flags_exist()) dst.flags(flags())


        }

    }
}

inline class REMOTE_LOG_DATA_BLOCK(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun daTa(): REMOTE_LOG_DATA_BLOCK_daTa {

        return REMOTE_LOG_DATA_BLOCK_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 200)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 200


    }


    inline fun seqno(): REMOTE_LOG_DATA_BLOCK_seqno? {
        if ((data.field_bit != 1616 && !data.set_field(1616, -1))) return null

        return REMOTE_LOG_DATA_BLOCK_seqno(data)
    }


    inline fun seqno(src: MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) {
        if (data.field_bit != 1616) data.set_field(1616, 0)

        set_bits((-2147483645 src . value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(144, 0, 0, 0, 203, 1, 1616, 0, 1)


        inline fun push(src: REMOTE_LOG_DATA_BLOCK, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, seqno: (src: com.company.demo.GroundControl.MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) -> Unit, daTa: (src: REMOTE_LOG_DATA_BLOCK_daTa) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.seqno()?.let { item ->
                seqno(item.get())
            }
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: REMOTE_LOG_DATA_BLOCK, target_system: () -> Byte, target_component: () -> Byte, seqno_exist: () -> Boolean, seqno: () -> com.company.demo.GroundControl.MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS, daTa: (dst: REMOTE_LOG_DATA_BLOCK_daTa) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            if (seqno_exist()) dst.seqno(seqno())

            daTa(dst.daTa())

        }

    }
}

inline class LOGGING_DATA_ACKED(val data: Pack.Bytes) {

    inline fun sequence(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun sequence_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun length(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun length_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun first_message_offset(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun first_message_offset_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun daTa(): LOGGING_DATA_ACKED_daTa {

        return LOGGING_DATA_ACKED_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 249)

        for (index in 0 until len)
            data.bytes[data.origin + 6 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 249


    }


    companion object {

        val meta = Meta(217, 1, 0, 0, 255, 1, 2040)


        inline fun push(src: LOGGING_DATA_ACKED, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, sequence: (src: Short) -> Unit, length: (src: Byte) -> Unit, first_message_offset: (src: Byte) -> Unit, daTa: (src: LOGGING_DATA_ACKED_daTa) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            sequence(src.sequence())

            length(src.length())

            first_message_offset(src.first_message_offset())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: LOGGING_DATA_ACKED, target_system: () -> Byte, target_component: () -> Byte, sequence: () -> Short, length: () -> Byte, first_message_offset: () -> Byte, daTa: (dst: LOGGING_DATA_ACKED_daTa) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.sequence(sequence())

            dst.length(length())

            dst.first_message_offset(first_message_offset())
            daTa(dst.daTa())

        }

    }
}

inline class TERRAIN_CHECK(val data: Pack.Bytes) {

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }


    companion object {

        val meta = Meta(152, 0, 0, 0, 8, 1, 64)


        inline fun push(src: TERRAIN_CHECK, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit
        ) {

            lat(src.lat())

            lon(src.lon())

        }

        inline fun pull(dst: TERRAIN_CHECK, lat: () -> Int, lon: () -> Int
        ) {

            dst.lat(lat())

            dst.lon(lon())

        }

    }
}

inline class MOUNT_CONFIGURE(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun stab_roll(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun stab_roll_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun stab_pitch(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun stab_pitch_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun stab_yaw(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun stab_yaw_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }


    inline fun mount_mode(): MOUNT_CONFIGURE_mount_mode? {
        if ((data.field_bit != 40 && !data.set_field(40, -1))) return null

        return MOUNT_CONFIGURE_mount_mode(data)
    }


    inline fun mount_mode(src: MAV_MOUNT_MODE) {
        if (data.field_bit != 40) data.set_field(40, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(40, 0, 0, 0, 6, 1, 40, 0, 1)


        inline fun push(src: MOUNT_CONFIGURE, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, mount_mode: (src: com.company.demo.GroundControl.MAV_MOUNT_MODE) -> Unit, stab_roll: (src: Byte) -> Unit, stab_pitch: (src: Byte) -> Unit, stab_yaw: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.mount_mode()?.let { item ->
                mount_mode(item.get())
            }

            stab_roll(src.stab_roll())

            stab_pitch(src.stab_pitch())

            stab_yaw(src.stab_yaw())

        }

        inline fun pull(dst: MOUNT_CONFIGURE, target_system: () -> Byte, target_component: () -> Byte, mount_mode_exist: () -> Boolean, mount_mode: () -> com.company.demo.GroundControl.MAV_MOUNT_MODE, stab_roll: () -> Byte, stab_pitch: () -> Byte, stab_yaw: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            if (mount_mode_exist()) dst.mount_mode(mount_mode())


            dst.stab_roll(stab_roll())

            dst.stab_pitch(stab_pitch())

            dst.stab_yaw(stab_yaw())

        }

    }
}

inline class MISSION_REQUEST_INT(val data: Cursor) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    inline fun mission_type(): MISSION_REQUEST_INT_mission_type? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return MISSION_REQUEST_INT_mission_type(data)
    }


    companion object {

        val meta = Meta(188, 1, 0, 0, 5, 1, 32, 0, 1)


        inline fun push(src: MISSION_REQUEST_INT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, seq: (src: Short) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            seq(src.seq())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    companion object {

        val meta = Meta(158, 0, 1, 0, 28, 1, 224)


        inline fun push(src: LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, time_boot_ms: (src: Int) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            x(src.x())

            y(src.y())

            z(src.z())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

        }

    }
}

inline class COMMAND_ACK(val data: Cursor) {


    inline fun command(): COMMAND_ACK_command? {
        if ((data.field_bit != 2 && !data.set_field(2, -1))) return null

        return COMMAND_ACK_command(data)
    }


    inline fun result(): COMMAND_ACK_result? {
        if ((data.field_bit != 3 && !data.set_field(3, -1))) return null

        return COMMAND_ACK_result(data)
    }


    inline fun progress(): Boolean {
        return !(data.field_bit != 4 && !data.set_field(4, -1))
    }


    inline fun result_param2(): Boolean {
        return !(data.field_bit != 5 && !data.set_field(5, -1))
    }


    inline fun target_system(): Boolean {
        return !(data.field_bit != 6 && !data.set_field(6, -1))
    }


    inline fun target_component(): Boolean {
        return !(data.field_bit != 7 && !data.set_field(7, -1))
    }


    companion object {

        val meta = Meta(121, 0, 0, 0, 1, 1, 2, 2, 6)


        inline fun push(src: COMMAND_ACK, command: (src: com.company.demo.GroundControl.MAV_CMD) -> Unit, result: (src: com.company.demo.GroundControl.MAV_RESULT) -> Unit, progress: (src: Byte) -> Unit, result_param2: (src: Int) -> Unit, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit
        ) {

            src.command()?.let { item ->
                command(item.get())
            }

            src.result()?.let { item ->
                result(item.get())
            }

            src.progress().let { item ->
                progress(item.get())
            }

            src.result_param2().let { item ->
                result_param2(item.get())
            }

            src.target_system().let { item ->
                target_system(item.get())
            }

            src.target_component().let { item ->
                target_component(item.get())
            }

        }

    }
}

inline class DATA_STREAM(val data: Pack.Bytes) {

    inline fun message_rate(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun stream_id(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun on_off(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    companion object {

        val meta = Meta(31, 1, 0, 0, 4, 1, 32)


        inline fun push(src: DATA_STREAM, stream_id: (src: Byte) -> Unit, message_rate: (src: Short) -> Unit, on_off: (src: Byte) -> Unit
        ) {

            stream_id(src.stream_id())

            message_rate(src.message_rate())

            on_off(src.on_off())

        }

    }
}

inline class MISSION_REQUEST(val data: Cursor) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    inline fun mission_type(): MISSION_REQUEST_mission_type? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return MISSION_REQUEST_mission_type(data)
    }


    companion object {

        val meta = Meta(147, 1, 0, 0, 5, 1, 32, 0, 1)


        inline fun push(src: MISSION_REQUEST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, seq: (src: Short) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            seq(src.seq())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class TERRAIN_REPORT(val data: Pack.Bytes) {

    inline fun spacing(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun spacing_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun pending(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun pending_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun loaded(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun loaded_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun lon(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun lon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun terrain_height(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun terrain_height_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun current_height(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun current_height_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }


    companion object {

        val meta = Meta(86, 3, 0, 0, 22, 1, 176)


        inline fun push(src: TERRAIN_REPORT, lat: (src: Int) -> Unit, lon: (src: Int) -> Unit, spacing: (src: Short) -> Unit, terrain_height: (src: Float) -> Unit, current_height: (src: Float) -> Unit, pending: (src: Short) -> Unit, loaded: (src: Short) -> Unit
        ) {

            lat(src.lat())

            lon(src.lon())

            spacing(src.spacing())

            terrain_height(src.terrain_height())

            current_height(src.current_height())

            pending(src.pending())

            loaded(src.loaded())

        }

        inline fun pull(dst: TERRAIN_REPORT, lat: () -> Int, lon: () -> Int, spacing: () -> Short, terrain_height: () -> Float, current_height: () -> Float, pending: () -> Short, loaded: () -> Short
        ) {

            dst.lat(lat())

            dst.lon(lon())

            dst.spacing(spacing())

            dst.terrain_height(terrain_height())

            dst.current_height(current_height())

            dst.pending(pending())

            dst.loaded(loaded())

        }

    }
}

inline class SET_HOME_POSITION(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun latitude(): Int {
        return (get_bytes(data.bytes, data.origin + 1, 4)).toInt()
    }

    inline fun latitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 1)
    }

    inline fun longitude(): Int {
        return (get_bytes(data.bytes, data.origin + 5, 4)).toInt()
    }

    inline fun longitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 5)
    }

    inline fun altitude(): Int {
        return (get_bytes(data.bytes, data.origin + 9, 4)).toInt()
    }

    inline fun altitude_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 9)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 13, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 13)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 17, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 21, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 21)
    }

    inline fun q(): SET_HOME_POSITION_q {

        return SET_HOME_POSITION_q(data)
    }

    inline fun q_(src: FloatArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 25 + index * 4)
    }

    object q {
        const val item_len = 4


    }

    inline fun approach_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 41, 4).toInt())
    }

    inline fun approach_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 41)
    }

    inline fun approach_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 45, 4).toInt())
    }

    inline fun approach_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 45)
    }

    inline fun approach_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 49, 4).toInt())
    }

    inline fun approach_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 49)
    }


    inline fun time_usec(): Boolean {
        return !(data.field_bit != 424 && !data.set_field(424, -1))
    }


    inline fun time_usec_() {
        if (data.field_bit != 424) data.set_field(424, 0)
    }


    companion object {

        val meta = Meta(77, 0, 0, 0, 54, 1, 424, 0, 1)


        inline fun push(src: SET_HOME_POSITION, target_system: (src: Byte) -> Unit, latitude: (src: Int) -> Unit, longitude: (src: Int) -> Unit, altitude: (src: Int) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, q: (src: SET_HOME_POSITION_q) -> Unit, approach_x: (src: Float) -> Unit, approach_y: (src: Float) -> Unit, approach_z: (src: Float) -> Unit, time_usec: (src: Long) -> Unit
        ) {

            target_system(src.target_system())

            latitude(src.latitude())

            longitude(src.longitude())

            altitude(src.altitude())

            x(src.x())

            y(src.y())

            z(src.z())
            src.q().let { item ->
                q(item)
            }

            approach_x(src.approach_x())

            approach_y(src.approach_y())

            approach_z(src.approach_z())

            src.time_usec().let { item ->
                time_usec(item.get())
            }

        }

        inline fun pull(dst: SET_HOME_POSITION, target_system: () -> Byte, latitude: () -> Int, longitude: () -> Int, altitude: () -> Int, x: () -> Float, y: () -> Float, z: () -> Float, q: (dst: SET_HOME_POSITION_q) -> Unit, approach_x: () -> Float, approach_y: () -> Float, approach_z: () -> Float, time_usec_exist: () -> Boolean, time_usec: () -> Long
        ) {

            dst.target_system(target_system())

            dst.latitude(latitude())

            dst.longitude(longitude())

            dst.altitude(altitude())

            dst.x(x())

            dst.y(y())

            dst.z(z())
            q(dst.q())

            dst.approach_x(approach_x())

            dst.approach_y(approach_y())

            dst.approach_z(approach_z())

            if (time_usec_exist()) dst.time_usec(time_usec())


        }

    }
}

object SwitchModeCommand : Pack(Meta(28))

inline class HIL_RC_INPUTS_RAW(val data: Pack.Bytes) {

    inline fun chan1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun chan2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun chan3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun chan4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }


    inline fun chan5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun chan6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun chan7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun chan8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    inline fun chan9_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }


    inline fun chan10_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }


    inline fun chan11_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }


    inline fun chan12_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }


    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 24, 8)).toLong()
    }


    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 32]).toByte()
    }


    companion object {

        val meta = Meta(184, 12, 0, 1, 33, 1, 264)


        inline fun push(src: HIL_RC_INPUTS_RAW, time_usec: (src: Long) -> Unit, chan1_raw: (src: Short) -> Unit, chan2_raw: (src: Short) -> Unit, chan3_raw: (src: Short) -> Unit, chan4_raw: (src: Short) -> Unit, chan5_raw: (src: Short) -> Unit, chan6_raw: (src: Short) -> Unit, chan7_raw: (src: Short) -> Unit, chan8_raw: (src: Short) -> Unit, chan9_raw: (src: Short) -> Unit, chan10_raw: (src: Short) -> Unit, chan11_raw: (src: Short) -> Unit, chan12_raw: (src: Short) -> Unit, rssi: (src: Byte) -> Unit
        ) {

            time_usec(src.time_usec())

            chan1_raw(src.chan1_raw())

            chan2_raw(src.chan2_raw())

            chan3_raw(src.chan3_raw())

            chan4_raw(src.chan4_raw())

            chan5_raw(src.chan5_raw())

            chan6_raw(src.chan6_raw())

            chan7_raw(src.chan7_raw())

            chan8_raw(src.chan8_raw())

            chan9_raw(src.chan9_raw())

            chan10_raw(src.chan10_raw())

            chan11_raw(src.chan11_raw())

            chan12_raw(src.chan12_raw())

            rssi(src.rssi())

        }

    }
}

inline class SCALED_IMU3(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun xacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun yacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun zacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun xgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun xgyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun ygyro(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun ygyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun zgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun zgyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun xmag(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }

    inline fun xmag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 16)
    }

    inline fun ymag(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }

    inline fun ymag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 18)
    }

    inline fun zmag(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }

    inline fun zmag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 20)
    }


    companion object {

        val meta = Meta(2, 0, 1, 0, 22, 1, 176)


        inline fun push(src: SCALED_IMU3, time_boot_ms: (src: Int) -> Unit, xacc: (src: Short) -> Unit, yacc: (src: Short) -> Unit, zacc: (src: Short) -> Unit, xgyro: (src: Short) -> Unit, ygyro: (src: Short) -> Unit, zgyro: (src: Short) -> Unit, xmag: (src: Short) -> Unit, ymag: (src: Short) -> Unit, zmag: (src: Short) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

            xgyro(src.xgyro())

            ygyro(src.ygyro())

            zgyro(src.zgyro())

            xmag(src.xmag())

            ymag(src.ymag())

            zmag(src.zmag())

        }

        inline fun pull(dst: SCALED_IMU3, time_boot_ms: () -> Int, xacc: () -> Short, yacc: () -> Short, zacc: () -> Short, xgyro: () -> Short, ygyro: () -> Short, zgyro: () -> Short, xmag: () -> Short, ymag: () -> Short, zmag: () -> Short
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.xacc(xacc())

            dst.yacc(yacc())

            dst.zacc(zacc())

            dst.xgyro(xgyro())

            dst.ygyro(ygyro())

            dst.zgyro(zgyro())

            dst.xmag(xmag())

            dst.ymag(ymag())

            dst.zmag(zmag())

        }

    }
}

inline class SET_MODE(val data: Cursor) {

    inline fun custom_mode(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun base_mode(): SET_MODE_base_mode? {
        if ((data.field_bit != 40 && !data.set_field(40, -1))) return null

        return SET_MODE_base_mode(data)
    }


    companion object {

        val meta = Meta(7, 0, 1, 0, 6, 1, 40, 0, 1)


        inline fun push(src: SET_MODE, target_system: (src: Byte) -> Unit, base_mode: (src: com.company.demo.GroundControl.MAV_MODE) -> Unit, custom_mode: (src: Int) -> Unit
        ) {

            target_system(src.target_system())

            src.base_mode()?.let { item ->
                base_mode(item.get())
            }

            custom_mode(src.custom_mode())

        }

    }
}

inline class MOUNT_CONTROL(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun input_a(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun input_a_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun input_b(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun input_b_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun input_c(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun input_c_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun save_position(): Byte {
        return (data.bytes[data.origin + 14]).toByte()
    }

    inline fun save_position_(src: Byte) {
        data.bytes[data.origin + 14] = (src).toByte()
    }


    companion object {

        val meta = Meta(113, 0, 0, 0, 15, 1, 120)


        inline fun push(src: MOUNT_CONTROL, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, input_a: (src: Int) -> Unit, input_b: (src: Int) -> Unit, input_c: (src: Int) -> Unit, save_position: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            input_a(src.input_a())

            input_b(src.input_b())

            input_c(src.input_c())

            save_position(src.save_position())

        }

        inline fun pull(dst: MOUNT_CONTROL, target_system: () -> Byte, target_component: () -> Byte, input_a: () -> Int, input_b: () -> Int, input_c: () -> Int, save_position: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.input_a(input_a())

            dst.input_b(input_b())

            dst.input_c(input_c())

            dst.save_position(save_position())

        }

    }
}

inline class POSITION_TARGET_GLOBAL_INT(val data: Cursor) {

    inline fun type_mask(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }


    inline fun lat_int(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }


    inline fun lon_int(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }


    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }


    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }


    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }


    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }


    inline fun afx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }


    inline fun afy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 34, 4).toInt())
    }


    inline fun afz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 38, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 42, 4).toInt())
    }


    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 46, 4).toInt())
    }


    inline fun coordinate_frame(): POSITION_TARGET_GLOBAL_INT_coordinate_frame? {
        if ((data.field_bit != 400 && !data.set_field(400, -1))) return null

        return POSITION_TARGET_GLOBAL_INT_coordinate_frame(data)
    }


    companion object {

        val meta = Meta(133, 1, 1, 0, 51, 1, 400, 0, 1)


        inline fun push(src: POSITION_TARGET_GLOBAL_INT, time_boot_ms: (src: Int) -> Unit, coordinate_frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, type_mask: (src: Short) -> Unit, lat_int: (src: Int) -> Unit, lon_int: (src: Int) -> Unit, alt: (src: Float) -> Unit, vx: (src: Float) -> Unit, vy: (src: Float) -> Unit, vz: (src: Float) -> Unit, afx: (src: Float) -> Unit, afy: (src: Float) -> Unit, afz: (src: Float) -> Unit, yaw: (src: Float) -> Unit, yaw_rate: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            src.coordinate_frame()?.let { item ->
                coordinate_frame(item.get())
            }

            type_mask(src.type_mask())

            lat_int(src.lat_int())

            lon_int(src.lon_int())

            alt(src.alt())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            afx(src.afx())

            afy(src.afy())

            afz(src.afz())

            yaw(src.yaw())

            yaw_rate(src.yaw_rate())

        }

    }
}

inline class LED_CONTROL(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun instance(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun instance_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun pattern(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun pattern_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun custom_len(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun custom_len_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun custom_bytes(): LED_CONTROL_custom_bytes {

        return LED_CONTROL_custom_bytes(data)
    }

    inline fun custom_bytes_(src: ByteArray) {
        val len = minOf(src.size, 24)

        for (index in 0 until len)
            data.bytes[data.origin + 5 + index] = (src[index]).toByte()
    }

    object custom_bytes {
        const val item_len = 24


    }


    companion object {

        val meta = Meta(177, 0, 0, 0, 29, 1, 232)


        inline fun push(src: LED_CONTROL, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, instance: (src: Byte) -> Unit, pattern: (src: Byte) -> Unit, custom_len: (src: Byte) -> Unit, custom_bytes: (src: LED_CONTROL_custom_bytes) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            instance(src.instance())

            pattern(src.pattern())

            custom_len(src.custom_len())
            src.custom_bytes().let { item ->
                custom_bytes(item)
            }

        }

        inline fun pull(dst: LED_CONTROL, target_system: () -> Byte, target_component: () -> Byte, instance: () -> Byte, pattern: () -> Byte, custom_len: () -> Byte, custom_bytes: (dst: LED_CONTROL_custom_bytes) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.instance(instance())

            dst.pattern(pattern())

            dst.custom_len(custom_len())
            custom_bytes(dst.custom_bytes())

        }

    }
}

inline class SIM_STATE(val data: Pack.Bytes) {

    inline fun q1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun q1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun q2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun q2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun q3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun q3_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun q4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun q4_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun xacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun xacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun yacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun yacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun zacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun zacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun xgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun xgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }

    inline fun ygyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }

    inline fun ygyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 44)
    }

    inline fun zgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48, 4).toInt())
    }

    inline fun zgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 48)
    }

    inline fun lat(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 52, 4).toInt())
    }

    inline fun lat_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 52)
    }

    inline fun lon(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 56, 4).toInt())
    }

    inline fun lon_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 56)
    }

    inline fun alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 60, 4).toInt())
    }

    inline fun alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 60)
    }

    inline fun std_dev_horz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 64, 4).toInt())
    }

    inline fun std_dev_horz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 64)
    }

    inline fun std_dev_vert(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 68, 4).toInt())
    }

    inline fun std_dev_vert_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 68)
    }

    inline fun vn(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 72, 4).toInt())
    }

    inline fun vn_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 72)
    }

    inline fun ve(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 76, 4).toInt())
    }

    inline fun ve_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 76)
    }

    inline fun vd(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 80, 4).toInt())
    }

    inline fun vd_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 80)
    }


    companion object {

        val meta = Meta(108, 0, 0, 0, 84, 1, 672)


        inline fun push(src: SIM_STATE, q1: (src: Float) -> Unit, q2: (src: Float) -> Unit, q3: (src: Float) -> Unit, q4: (src: Float) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit, xacc: (src: Float) -> Unit, yacc: (src: Float) -> Unit, zacc: (src: Float) -> Unit, xgyro: (src: Float) -> Unit, ygyro: (src: Float) -> Unit, zgyro: (src: Float) -> Unit, lat: (src: Float) -> Unit, lon: (src: Float) -> Unit, alt: (src: Float) -> Unit, std_dev_horz: (src: Float) -> Unit, std_dev_vert: (src: Float) -> Unit, vn: (src: Float) -> Unit, ve: (src: Float) -> Unit, vd: (src: Float) -> Unit
        ) {

            q1(src.q1())

            q2(src.q2())

            q3(src.q3())

            q4(src.q4())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

            xgyro(src.xgyro())

            ygyro(src.ygyro())

            zgyro(src.zgyro())

            lat(src.lat())

            lon(src.lon())

            alt(src.alt())

            std_dev_horz(src.std_dev_horz())

            std_dev_vert(src.std_dev_vert())

            vn(src.vn())

            ve(src.ve())

            vd(src.vd())

        }

        inline fun pull(dst: SIM_STATE, q1: () -> Float, q2: () -> Float, q3: () -> Float, q4: () -> Float, roll: () -> Float, pitch: () -> Float, yaw: () -> Float, xacc: () -> Float, yacc: () -> Float, zacc: () -> Float, xgyro: () -> Float, ygyro: () -> Float, zgyro: () -> Float, lat: () -> Float, lon: () -> Float, alt: () -> Float, std_dev_horz: () -> Float, std_dev_vert: () -> Float, vn: () -> Float, ve: () -> Float, vd: () -> Float
        ) {

            dst.q1(q1())

            dst.q2(q2())

            dst.q3(q3())

            dst.q4(q4())

            dst.roll(roll())

            dst.pitch(pitch())

            dst.yaw(yaw())

            dst.xacc(xacc())

            dst.yacc(yacc())

            dst.zacc(zacc())

            dst.xgyro(xgyro())

            dst.ygyro(ygyro())

            dst.zgyro(zgyro())

            dst.lat(lat())

            dst.lon(lon())

            dst.alt(alt())

            dst.std_dev_horz(std_dev_horz())

            dst.std_dev_vert(std_dev_vert())

            dst.vn(vn())

            dst.ve(ve())

            dst.vd(vd())

        }

    }
}

inline class WIFI_CONFIG_AP(val data: Cursor) {


    inline fun ssid(): WIFI_CONFIG_AP_ssid? {
        if ((data.field_bit != 2 && !data.set_field(2, -1))) return null

        return WIFI_CONFIG_AP_ssid(data)
    }


    inline fun ssid_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -2 - 1, reuse)
    }


    inline fun ssid_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(2, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun ssid_(len: Int): WIFI_CONFIG_AP_ssid {

        data.set_field(2, minOf(len, 255))
        return WIFI_CONFIG_AP_ssid(data)
    }

    object ssid {
        const val item_len_max = 255

    }


    inline fun password(): WIFI_CONFIG_AP_password? {
        if ((data.field_bit != 3 && !data.set_field(3, -1))) return null

        return WIFI_CONFIG_AP_password(data)
    }


    inline fun password_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -3 - 1, reuse)
    }


    inline fun password_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(3, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun password_(len: Int): WIFI_CONFIG_AP_password {

        data.set_field(3, minOf(len, 255))
        return WIFI_CONFIG_AP_password(data)
    }

    object password {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(194, 0, 0, 0, 1, 1, 2, 2, 2)


        inline fun push(src: WIFI_CONFIG_AP, ssid: (src: WIFI_CONFIG_AP_ssid) -> Unit, password: (src: WIFI_CONFIG_AP_password) -> Unit
        ) {

            src.ssid()?.let { item -> ssid(item) }

            src.password()?.let { item -> password(item) }

        }

        inline fun pull(dst: WIFI_CONFIG_AP, ssid_exist: () -> Int, ssid: (dst: WIFI_CONFIG_AP_ssid) -> Unit, password_exist: () -> Int, password: (dst: WIFI_CONFIG_AP_password) -> Unit
        ) {


            ssid_exist().let { len ->
                if (0 < len)
                    ssid(dst.ssid(len))
            }



            password_exist().let { len ->
                if (0 < len)
                    password(dst.password(len))
            }


        }

    }
}

inline class DATA96(val data: Pack.Bytes) {

    inline fun typE(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun typE_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun len(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun len_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun daTa(): DATA96_daTa {

        return DATA96_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 96)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 96


    }


    companion object {

        val meta = Meta(83, 0, 0, 0, 98, 1, 784)


        inline fun push(src: DATA96, typE: (src: Byte) -> Unit, len: (src: Byte) -> Unit, daTa: (src: DATA96_daTa) -> Unit
        ) {

            typE(src.typE())

            len(src.len())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: DATA96, typE: () -> Byte, len: () -> Byte, daTa: (dst: DATA96_daTa) -> Unit
        ) {

            dst.typE(typE())

            dst.len(len())
            daTa(dst.daTa())

        }

    }
}

inline class FLIGHT_INFORMATION(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun arming_time_utc(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }

    inline fun arming_time_utc_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 4)
    }

    inline fun takeoff_time_utc(): Long {
        return (get_bytes(data.bytes, data.origin + 12, 8)).toLong()
    }

    inline fun takeoff_time_utc_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 12)
    }

    inline fun flight_uuid(): Long {
        return (get_bytes(data.bytes, data.origin + 20, 8)).toLong()
    }

    inline fun flight_uuid_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 20)
    }


    companion object {

        val meta = Meta(215, 0, 1, 3, 28, 1, 224)


        inline fun push(src: FLIGHT_INFORMATION, time_boot_ms: (src: Int) -> Unit, arming_time_utc: (src: Long) -> Unit, takeoff_time_utc: (src: Long) -> Unit, flight_uuid: (src: Long) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            arming_time_utc(src.arming_time_utc())

            takeoff_time_utc(src.takeoff_time_utc())

            flight_uuid(src.flight_uuid())

        }

        inline fun pull(dst: FLIGHT_INFORMATION, time_boot_ms: () -> Int, arming_time_utc: () -> Long, takeoff_time_utc: () -> Long, flight_uuid: () -> Long
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.arming_time_utc(arming_time_utc())

            dst.takeoff_time_utc(takeoff_time_utc())

            dst.flight_uuid(flight_uuid())

        }

    }
}

inline class RC_CHANNELS_RAW(val data: Pack.Bytes) {

    inline fun chan1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun chan2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun chan3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun chan4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }


    inline fun chan5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun chan6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun chan7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun chan8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }


    inline fun port(): Byte {
        return (data.bytes[data.origin + 20]).toByte()
    }


    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 21]).toByte()
    }


    companion object {

        val meta = Meta(148, 8, 1, 0, 22, 1, 176)


        inline fun push(src: RC_CHANNELS_RAW, time_boot_ms: (src: Int) -> Unit, port: (src: Byte) -> Unit, chan1_raw: (src: Short) -> Unit, chan2_raw: (src: Short) -> Unit, chan3_raw: (src: Short) -> Unit, chan4_raw: (src: Short) -> Unit, chan5_raw: (src: Short) -> Unit, chan6_raw: (src: Short) -> Unit, chan7_raw: (src: Short) -> Unit, chan8_raw: (src: Short) -> Unit, rssi: (src: Byte) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            port(src.port())

            chan1_raw(src.chan1_raw())

            chan2_raw(src.chan2_raw())

            chan3_raw(src.chan3_raw())

            chan4_raw(src.chan4_raw())

            chan5_raw(src.chan5_raw())

            chan6_raw(src.chan6_raw())

            chan7_raw(src.chan7_raw())

            chan8_raw(src.chan8_raw())

            rssi(src.rssi())

        }

    }
}

inline class SERVO_OUTPUT_RAW(val data: Cursor) {

    inline fun servo1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun servo2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun servo3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun servo4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }


    inline fun servo5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun servo6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun servo7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun servo8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    inline fun time_usec(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }


    inline fun port(): Byte {
        return (data.bytes[data.origin + 20]).toByte()
    }


    inline fun servo9_raw(): Boolean {
        return !(data.field_bit != 168 && !data.set_field(168, -1))
    }


    inline fun servo10_raw(): Boolean {
        return !(data.field_bit != 169 && !data.set_field(169, -1))
    }


    inline fun servo11_raw(): Boolean {
        return !(data.field_bit != 170 && !data.set_field(170, -1))
    }


    inline fun servo12_raw(): Boolean {
        return !(data.field_bit != 171 && !data.set_field(171, -1))
    }


    inline fun servo13_raw(): Boolean {
        return !(data.field_bit != 172 && !data.set_field(172, -1))
    }


    inline fun servo14_raw(): Boolean {
        return !(data.field_bit != 173 && !data.set_field(173, -1))
    }


    inline fun servo15_raw(): Boolean {
        return !(data.field_bit != 174 && !data.set_field(174, -1))
    }


    inline fun servo16_raw(): Boolean {
        return !(data.field_bit != 175 && !data.set_field(175, -1))
    }


    companion object {

        val meta = Meta(79, 8, 1, 0, 22, 1, 168, 0, 8)


        inline fun push(src: SERVO_OUTPUT_RAW, time_usec: (src: Int) -> Unit, port: (src: Byte) -> Unit, servo1_raw: (src: Short) -> Unit, servo2_raw: (src: Short) -> Unit, servo3_raw: (src: Short) -> Unit, servo4_raw: (src: Short) -> Unit, servo5_raw: (src: Short) -> Unit, servo6_raw: (src: Short) -> Unit, servo7_raw: (src: Short) -> Unit, servo8_raw: (src: Short) -> Unit, servo9_raw: (src: Short) -> Unit, servo10_raw: (src: Short) -> Unit, servo11_raw: (src: Short) -> Unit, servo12_raw: (src: Short) -> Unit, servo13_raw: (src: Short) -> Unit, servo14_raw: (src: Short) -> Unit, servo15_raw: (src: Short) -> Unit, servo16_raw: (src: Short) -> Unit
        ) {

            time_usec(src.time_usec())

            port(src.port())

            servo1_raw(src.servo1_raw())

            servo2_raw(src.servo2_raw())

            servo3_raw(src.servo3_raw())

            servo4_raw(src.servo4_raw())

            servo5_raw(src.servo5_raw())

            servo6_raw(src.servo6_raw())

            servo7_raw(src.servo7_raw())

            servo8_raw(src.servo8_raw())

            src.servo9_raw().let { item ->
                servo9_raw(item.get())
            }

            src.servo10_raw().let { item ->
                servo10_raw(item.get())
            }

            src.servo11_raw().let { item ->
                servo11_raw(item.get())
            }

            src.servo12_raw().let { item ->
                servo12_raw(item.get())
            }

            src.servo13_raw().let { item ->
                servo13_raw(item.get())
            }

            src.servo14_raw().let { item ->
                servo14_raw(item.get())
            }

            src.servo15_raw().let { item ->
                servo15_raw(item.get())
            }

            src.servo16_raw().let { item ->
                servo16_raw(item.get())
            }

        }

    }
}

inline class MEMINFO(val data: Cursor) {

    inline fun brkval(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun brkval_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun freemem(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun freemem_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }


    inline fun freemem32(): Boolean {
        return !(data.field_bit != 32 && !data.set_field(32, -1))
    }


    inline fun freemem32_() {
        if (data.field_bit != 32) data.set_field(32, 0)
    }


    companion object {

        val meta = Meta(159, 2, 0, 0, 5, 1, 32, 0, 1)


        inline fun push(src: MEMINFO, brkval: (src: Short) -> Unit, freemem: (src: Short) -> Unit, freemem32: (src: Int) -> Unit
        ) {

            brkval(src.brkval())

            freemem(src.freemem())

            src.freemem32().let { item ->
                freemem32(item.get())
            }

        }

        inline fun pull(dst: MEMINFO, brkval: () -> Short, freemem: () -> Short, freemem32_exist: () -> Boolean, freemem32: () -> Int
        ) {

            dst.brkval(brkval())

            dst.freemem(freemem())

            if (freemem32_exist()) dst.freemem32(freemem32())


        }

    }
}

inline class MISSION_ITEM_REACHED(val data: Pack.Bytes) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    companion object {

        val meta = Meta(81, 1, 0, 0, 2, 1, 16)


        inline fun push(src: MISSION_ITEM_REACHED, seq: (src: Short) -> Unit
        ) {

            seq(src.seq())

        }

    }
}

inline class LOGGING_ACK(val data: Pack.Bytes) {

    inline fun sequence(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun sequence_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }


    companion object {

        val meta = Meta(220, 1, 0, 0, 4, 1, 32)


        inline fun push(src: LOGGING_ACK, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, sequence: (src: Short) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            sequence(src.sequence())

        }

        inline fun pull(dst: LOGGING_ACK, target_system: () -> Byte, target_component: () -> Byte, sequence: () -> Short
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.sequence(sequence())

        }

    }
}

inline class VISION_SPEED_ESTIMATE(val data: Pack.Bytes) {

    inline fun usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }


    companion object {

        val meta = Meta(42, 0, 0, 1, 20, 1, 160)


        inline fun push(src: VISION_SPEED_ESTIMATE, usec: (src: Long) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit
        ) {

            usec(src.usec())

            x(src.x())

            y(src.y())

            z(src.z())

        }

        inline fun pull(dst: VISION_SPEED_ESTIMATE, usec: () -> Long, x: () -> Float, y: () -> Float, z: () -> Float
        ) {

            dst.usec(usec())

            dst.x(x())

            dst.y(y())

            dst.z(z())

        }

    }
}

inline class DEBUG_VECT(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }


    inline fun name(): DEBUG_VECT_name? {
        if ((data.field_bit != 162 && !data.set_field(162, -1))) return null

        return DEBUG_VECT_name(data)
    }


    inline fun name_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -162 - 1, reuse)
    }


    inline fun name_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(162, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun name_(len: Int): DEBUG_VECT_name {

        data.set_field(162, minOf(len, 255))
        return DEBUG_VECT_name(data)
    }

    object name {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(187, 0, 0, 1, 21, 1, 162, 2, 1)


        inline fun push(src: DEBUG_VECT, name: (src: DEBUG_VECT_name) -> Unit, time_usec: (src: Long) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit
        ) {

            src.name()?.let { item -> name(item) }

            time_usec(src.time_usec())

            x(src.x())

            y(src.y())

            z(src.z())

        }

        inline fun pull(dst: DEBUG_VECT, name_exist: () -> Int, name: (dst: DEBUG_VECT_name) -> Unit, time_usec: () -> Long, x: () -> Float, y: () -> Float, z: () -> Float
        ) {


            name_exist().let { len ->
                if (0 < len)
                    name(dst.name(len))
            }


            dst.time_usec(time_usec())

            dst.x(x())

            dst.y(y())

            dst.z(z())

        }

    }
}

inline class LOG_REQUEST_END(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    companion object {

        val meta = Meta(102, 0, 0, 0, 2, 1, 16)


        inline fun push(src: LOG_REQUEST_END, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

        }

        inline fun pull(dst: LOG_REQUEST_END, target_system: () -> Byte, target_component: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

        }

    }
}

inline class MISSION_ACK(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun typE(): MISSION_ACK_typE? {
        if ((data.field_bit != 18 && !data.set_field(18, -1))) return null

        return MISSION_ACK_typE(data)
    }


    inline fun mission_type(): MISSION_ACK_mission_type? {
        if ((data.field_bit != 19 && !data.set_field(19, -1))) return null

        return MISSION_ACK_mission_type(data)
    }


    companion object {

        val meta = Meta(85, 0, 0, 0, 3, 1, 18, 2, 2)


        inline fun push(src: MISSION_ACK, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, typE: (src: com.company.demo.GroundControl.MAV_MISSION_RESULT) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.typE()?.let { item ->
                typE(item.get())
            }

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class CHANGE_OPERATOR_CONTROL_ACK(val data: Pack.Bytes) {

    inline fun gcs_system_id(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun control_request(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun ack(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    companion object {

        val meta = Meta(62, 0, 0, 0, 3, 1, 24)


        inline fun push(src: CHANGE_OPERATOR_CONTROL_ACK, gcs_system_id: (src: Byte) -> Unit, control_request: (src: Byte) -> Unit, ack: (src: Byte) -> Unit
        ) {

            gcs_system_id(src.gcs_system_id())

            control_request(src.control_request())

            ack(src.ack())

        }

    }
}

inline class MISSION_CURRENT(val data: Pack.Bytes) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    companion object {

        val meta = Meta(189, 1, 0, 0, 2, 1, 16)


        inline fun push(src: MISSION_CURRENT, seq: (src: Short) -> Unit
        ) {

            seq(src.seq())

        }

    }
}

inline class SYSTEM_TIME(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun time_unix_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }


    companion object {

        val meta = Meta(9, 0, 1, 1, 12, 1, 96)


        inline fun push(src: SYSTEM_TIME, time_unix_usec: (src: Long) -> Unit, time_boot_ms: (src: Int) -> Unit
        ) {

            time_unix_usec(src.time_unix_usec())

            time_boot_ms(src.time_boot_ms())

        }

    }
}

inline class CAMERA_TRIGGER(val data: Pack.Bytes) {

    inline fun seq(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun seq_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 4, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 4)
    }


    companion object {

        val meta = Meta(191, 0, 1, 1, 12, 1, 96)


        inline fun push(src: CAMERA_TRIGGER, time_usec: (src: Long) -> Unit, seq: (src: Int) -> Unit
        ) {

            time_usec(src.time_usec())

            seq(src.seq())

        }

        inline fun pull(dst: CAMERA_TRIGGER, time_usec: () -> Long, seq: () -> Int
        ) {

            dst.time_usec(time_usec())

            dst.seq(seq())

        }

    }
}

inline class GOPRO_SET_RESPONSE(val data: Cursor) {


    inline fun cmd_id(): GOPRO_SET_RESPONSE_cmd_id? {
        if ((data.field_bit != 0 && !data.set_field(0, -1))) return null

        return GOPRO_SET_RESPONSE_cmd_id(data)
    }


    inline fun cmd_id(src: GOPRO_COMMAND) {
        if (data.field_bit != 0) data.set_field(0, 0)

        set_bits((src.value).toULong().toLong(), 5, data.bytes, data.BIT)
    }


    inline fun status(): GOPRO_SET_RESPONSE_status? {
        if ((data.field_bit != 1 && !data.set_field(1, -1))) return null

        return GOPRO_SET_RESPONSE_status(data)
    }


    inline fun status(src: GOPRO_REQUEST_STATUS) {
        if (data.field_bit != 1) data.set_field(1, 0)

        set_bits((src.value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(192, 0, 0, 0, 1, 1, 0, 0, 2)


        inline fun push(src: GOPRO_SET_RESPONSE, cmd_id: (src: com.company.demo.GroundControl.GOPRO_COMMAND) -> Unit, status: (src: com.company.demo.GroundControl.GOPRO_REQUEST_STATUS) -> Unit
        ) {

            src.cmd_id()?.let { item ->
                cmd_id(item.get())
            }

            src.status()?.let { item ->
                status(item.get())
            }

        }

        inline fun pull(dst: GOPRO_SET_RESPONSE, cmd_id_exist: () -> Boolean, cmd_id: () -> com.company.demo.GroundControl.GOPRO_COMMAND, status_exist: () -> Boolean, status: () -> com.company.demo.GroundControl.GOPRO_REQUEST_STATUS
        ) {

            if (cmd_id_exist()) dst.cmd_id(cmd_id())


            if (status_exist()) dst.status(status())


        }

    }
}

inline class VISION_POSITION_ESTIMATE(val data: Pack.Bytes) {

    inline fun usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    companion object {

        val meta = Meta(16, 0, 0, 1, 32, 1, 256)


        inline fun push(src: VISION_POSITION_ESTIMATE, usec: (src: Long) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit
        ) {

            usec(src.usec())

            x(src.x())

            y(src.y())

            z(src.z())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

        }

    }
}

inline class MANUAL_CONTROL(val data: Pack.Bytes) {

    inline fun buttons(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun target(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun x(): Short {
        return (get_bytes(data.bytes, data.origin + 3, 2)).toShort()
    }


    inline fun y(): Short {
        return (get_bytes(data.bytes, data.origin + 5, 2)).toShort()
    }


    inline fun z(): Short {
        return (get_bytes(data.bytes, data.origin + 7, 2)).toShort()
    }


    inline fun r(): Short {
        return (get_bytes(data.bytes, data.origin + 9, 2)).toShort()
    }


    companion object {

        val meta = Meta(54, 1, 0, 0, 11, 1, 88)


        inline fun push(src: MANUAL_CONTROL, target: (src: Byte) -> Unit, x: (src: Short) -> Unit, y: (src: Short) -> Unit, z: (src: Short) -> Unit, r: (src: Short) -> Unit, buttons: (src: Short) -> Unit
        ) {

            target(src.target())

            x(src.x())

            y(src.y())

            z(src.z())

            r(src.r())

            buttons(src.buttons())

        }

    }
}

inline class RC_CHANNELS(val data: Pack.Bytes) {

    inline fun chan1_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun chan2_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun chan3_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun chan4_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }


    inline fun chan5_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun chan6_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun chan7_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun chan8_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    inline fun chan9_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }


    inline fun chan10_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }


    inline fun chan11_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }


    inline fun chan12_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }


    inline fun chan13_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 24, 2)).toShort()
    }


    inline fun chan14_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 26, 2)).toShort()
    }


    inline fun chan15_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 28, 2)).toShort()
    }


    inline fun chan16_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 30, 2)).toShort()
    }


    inline fun chan17_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 32, 2)).toShort()
    }


    inline fun chan18_raw(): Short {
        return (get_bytes(data.bytes, data.origin + 34, 2)).toShort()
    }


    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 36, 4)).toInt()
    }


    inline fun chancount(): Byte {
        return (data.bytes[data.origin + 40]).toByte()
    }


    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 41]).toByte()
    }


    companion object {

        val meta = Meta(23, 18, 1, 0, 42, 1, 336)


        inline fun push(src: RC_CHANNELS, time_boot_ms: (src: Int) -> Unit, chancount: (src: Byte) -> Unit, chan1_raw: (src: Short) -> Unit, chan2_raw: (src: Short) -> Unit, chan3_raw: (src: Short) -> Unit, chan4_raw: (src: Short) -> Unit, chan5_raw: (src: Short) -> Unit, chan6_raw: (src: Short) -> Unit, chan7_raw: (src: Short) -> Unit, chan8_raw: (src: Short) -> Unit, chan9_raw: (src: Short) -> Unit, chan10_raw: (src: Short) -> Unit, chan11_raw: (src: Short) -> Unit, chan12_raw: (src: Short) -> Unit, chan13_raw: (src: Short) -> Unit, chan14_raw: (src: Short) -> Unit, chan15_raw: (src: Short) -> Unit, chan16_raw: (src: Short) -> Unit, chan17_raw: (src: Short) -> Unit, chan18_raw: (src: Short) -> Unit, rssi: (src: Byte) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            chancount(src.chancount())

            chan1_raw(src.chan1_raw())

            chan2_raw(src.chan2_raw())

            chan3_raw(src.chan3_raw())

            chan4_raw(src.chan4_raw())

            chan5_raw(src.chan5_raw())

            chan6_raw(src.chan6_raw())

            chan7_raw(src.chan7_raw())

            chan8_raw(src.chan8_raw())

            chan9_raw(src.chan9_raw())

            chan10_raw(src.chan10_raw())

            chan11_raw(src.chan11_raw())

            chan12_raw(src.chan12_raw())

            chan13_raw(src.chan13_raw())

            chan14_raw(src.chan14_raw())

            chan15_raw(src.chan15_raw())

            chan16_raw(src.chan16_raw())

            chan17_raw(src.chan17_raw())

            chan18_raw(src.chan18_raw())

            rssi(src.rssi())

        }

    }
}

inline class PROTOCOL_VERSION(val data: Pack.Bytes) {

    inline fun version(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun version_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun min_version(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun min_version_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun max_version(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun max_version_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun spec_version_hash(): PROTOCOL_VERSION_spec_version_hash {

        return PROTOCOL_VERSION_spec_version_hash(data)
    }

    inline fun spec_version_hash_(src: ByteArray) {
        val len = minOf(src.size, 8)

        for (index in 0 until len)
            data.bytes[data.origin + 6 + index] = (src[index]).toByte()
    }

    object spec_version_hash {
        const val item_len = 8


    }

    inline fun library_version_hash(): PROTOCOL_VERSION_library_version_hash {

        return PROTOCOL_VERSION_library_version_hash(data)
    }

    inline fun library_version_hash_(src: ByteArray) {
        val len = minOf(src.size, 8)

        for (index in 0 until len)
            data.bytes[data.origin + 14 + index] = (src[index]).toByte()
    }

    object library_version_hash {
        const val item_len = 8


    }


    companion object {

        val meta = Meta(150, 3, 0, 0, 22, 1, 176)


        inline fun push(src: PROTOCOL_VERSION, version: (src: Short) -> Unit, min_version: (src: Short) -> Unit, max_version: (src: Short) -> Unit, spec_version_hash: (src: PROTOCOL_VERSION_spec_version_hash) -> Unit, library_version_hash: (src: PROTOCOL_VERSION_library_version_hash) -> Unit
        ) {

            version(src.version())

            min_version(src.min_version())

            max_version(src.max_version())
            src.spec_version_hash().let { item ->
                spec_version_hash(item)
            }
            src.library_version_hash().let { item ->
                library_version_hash(item)
            }

        }

        inline fun pull(dst: PROTOCOL_VERSION, version: () -> Short, min_version: () -> Short, max_version: () -> Short, spec_version_hash: (dst: PROTOCOL_VERSION_spec_version_hash) -> Unit, library_version_hash: (dst: PROTOCOL_VERSION_library_version_hash) -> Unit
        ) {

            dst.version(version())

            dst.min_version(min_version())

            dst.max_version(max_version())
            spec_version_hash(dst.spec_version_hash())
            library_version_hash(dst.library_version_hash())

        }

    }
}

inline class RALLY_FETCH_POINT(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun idx(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun idx_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }


    companion object {

        val meta = Meta(5, 0, 0, 0, 3, 1, 24)


        inline fun push(src: RALLY_FETCH_POINT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, idx: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            idx(src.idx())

        }

        inline fun pull(dst: RALLY_FETCH_POINT, target_system: () -> Byte, target_component: () -> Byte, idx: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.idx(idx())

        }

    }
}

inline class PARAM_VALUE(val data: Cursor) {

    inline fun param_count(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun param_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun param_value(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun param_id(): PARAM_VALUE_param_id? {
        if ((data.field_bit != 66 && !data.set_field(66, -1))) return null

        return PARAM_VALUE_param_id(data)
    }


    object param_id {
        const val item_len_max = 255

    }


    inline fun param_type(): PARAM_VALUE_param_type? {
        if ((data.field_bit != 67 && !data.set_field(67, -1))) return null

        return PARAM_VALUE_param_type(data)
    }


    companion object {

        val meta = Meta(136, 2, 0, 0, 9, 1, 66, 2, 2)


        inline fun push(src: PARAM_VALUE, param_id: (src: PARAM_VALUE_param_id) -> Unit, param_value: (src: Float) -> Unit, param_type: (src: com.company.demo.GroundControl.MAV_PARAM_TYPE) -> Unit, param_count: (src: Short) -> Unit, param_index: (src: Short) -> Unit
        ) {

            src.param_id()?.let { item -> param_id(item) }

            param_value(src.param_value())

            src.param_type()?.let { item ->
                param_type(item.get())
            }

            param_count(src.param_count())

            param_index(src.param_index())

        }

    }
}

inline class BATTERY_STATUS(val data: Cursor) {

    inline fun voltages(): BATTERY_STATUS_voltages {

        return BATTERY_STATUS_voltages(data)
    }

    inline fun voltages_(src: ShortArray) {
        val len = minOf(src.size, 10)

        for (index in 0 until len)
            set_bytes((src[index]).toULong().toLong(), 2, data.bytes, data.origin + 0 + index * 2)
    }

    object voltages {
        const val item_len = 10


    }

    inline fun id(): Byte {
        return (data.bytes[data.origin + 20]).toByte()
    }

    inline fun id_(src: Byte) {
        data.bytes[data.origin + 20] = (src).toByte()
    }

    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 21, 2)).toShort()
    }

    inline fun temperature_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 21)
    }

    inline fun current_battery(): Short {
        return (get_bytes(data.bytes, data.origin + 23, 2)).toShort()
    }

    inline fun current_battery_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 23)
    }

    inline fun current_consumed(): Int {
        return (get_bytes(data.bytes, data.origin + 25, 4)).toInt()
    }

    inline fun current_consumed_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 25)
    }

    inline fun energy_consumed(): Int {
        return (get_bytes(data.bytes, data.origin + 29, 4)).toInt()
    }

    inline fun energy_consumed_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 29)
    }

    inline fun battery_remaining(): Byte {
        return (data.bytes[data.origin + 33]).toByte()
    }

    inline fun battery_remaining_(src: Byte) {
        data.bytes[data.origin + 33] = (src).toByte()
    }


    inline fun battery_function(): BATTERY_STATUS_battery_function? {
        if ((data.field_bit != 272 && !data.set_field(272, -1))) return null

        return BATTERY_STATUS_battery_function(data)
    }


    inline fun battery_function(src: MAV_BATTERY_FUNCTION) {
        if (data.field_bit != 272) data.set_field(272, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun typE(): BATTERY_STATUS_typE? {
        if ((data.field_bit != 273 && !data.set_field(273, -1))) return null

        return BATTERY_STATUS_typE(data)
    }


    inline fun typE(src: MAV_BATTERY_TYPE) {
        if (data.field_bit != 273) data.set_field(273, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(111, 10, 0, 0, 35, 1, 272, 0, 2)


        inline fun push(src: BATTERY_STATUS, id: (src: Byte) -> Unit, battery_function: (src: com.company.demo.GroundControl.MAV_BATTERY_FUNCTION) -> Unit, typE: (src: com.company.demo.GroundControl.MAV_BATTERY_TYPE) -> Unit, temperature: (src: Short) -> Unit, voltages: (src: BATTERY_STATUS_voltages) -> Unit, current_battery: (src: Short) -> Unit, current_consumed: (src: Int) -> Unit, energy_consumed: (src: Int) -> Unit, battery_remaining: (src: Byte) -> Unit
        ) {

            id(src.id())

            src.battery_function()?.let { item ->
                battery_function(item.get())
            }

            src.typE()?.let { item ->
                typE(item.get())
            }

            temperature(src.temperature())
            src.voltages().let { item ->
                voltages(item)
            }

            current_battery(src.current_battery())

            current_consumed(src.current_consumed())

            energy_consumed(src.energy_consumed())

            battery_remaining(src.battery_remaining())

        }

        inline fun pull(dst: BATTERY_STATUS, id: () -> Byte, battery_function_exist: () -> Boolean, battery_function: () -> com.company.demo.GroundControl.MAV_BATTERY_FUNCTION, typE_exist: () -> Boolean, typE: () -> com.company.demo.GroundControl.MAV_BATTERY_TYPE, temperature: () -> Short, voltages: (dst: BATTERY_STATUS_voltages) -> Unit, current_battery: () -> Short, current_consumed: () -> Int, energy_consumed: () -> Int, battery_remaining: () -> Byte
        ) {

            dst.id(id())

            if (battery_function_exist()) dst.battery_function(battery_function())


            if (typE_exist()) dst.typE(typE())


            dst.temperature(temperature())
            voltages(dst.voltages())

            dst.current_battery(current_battery())

            dst.current_consumed(current_consumed())

            dst.energy_consumed(energy_consumed())

            dst.battery_remaining(battery_remaining())

        }

    }
}

inline class SERIAL_CONTROL(val data: Cursor) {

    inline fun timeout(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun timeout_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun baudrate(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }

    inline fun baudrate_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun count(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun count_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun daTa(): SERIAL_CONTROL_daTa {

        return SERIAL_CONTROL_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 70)

        for (index in 0 until len)
            data.bytes[data.origin + 7 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 70


    }


    inline fun device(): SERIAL_CONTROL_device? {
        if ((data.field_bit != 616 && !data.set_field(616, -1))) return null

        return SERIAL_CONTROL_device(data)
    }


    inline fun device(src: SERIAL_CONTROL_DEV) {
        if (data.field_bit != 616) data.set_field(616, 0)

        set_bits(SERIAL_CONTROL_DEV.set(src).toLong(), 3, data.bytes, data.BIT)
    }


    inline fun flags(): SERIAL_CONTROL_flags? {
        if ((data.field_bit != 617 && !data.set_field(617, -1))) return null

        return SERIAL_CONTROL_flags(data)
    }


    inline fun flags(src: SERIAL_CONTROL_FLAG) {
        if (data.field_bit != 617) data.set_field(617, 0)

        set_bits(SERIAL_CONTROL_FLAG.set(src).toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(99, 1, 1, 0, 78, 1, 616, 0, 2)


        inline fun push(src: SERIAL_CONTROL, device: (src: com.company.demo.GroundControl.SERIAL_CONTROL_DEV) -> Unit, flags: (src: com.company.demo.GroundControl.SERIAL_CONTROL_FLAG) -> Unit, timeout: (src: Short) -> Unit, baudrate: (src: Int) -> Unit, count: (src: Byte) -> Unit, daTa: (src: SERIAL_CONTROL_daTa) -> Unit
        ) {

            src.device()?.let { item ->
                device(item.get())
            }

            src.flags()?.let { item ->
                flags(item.get())
            }

            timeout(src.timeout())

            baudrate(src.baudrate())

            count(src.count())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: SERIAL_CONTROL, device_exist: () -> Boolean, device: () -> com.company.demo.GroundControl.SERIAL_CONTROL_DEV, flags_exist: () -> Boolean, flags: () -> com.company.demo.GroundControl.SERIAL_CONTROL_FLAG, timeout: () -> Short, baudrate: () -> Int, count: () -> Byte, daTa: (dst: SERIAL_CONTROL_daTa) -> Unit
        ) {

            if (device_exist()) dst.device(device())


            if (flags_exist()) dst.flags(flags())


            dst.timeout(timeout())

            dst.baudrate(baudrate())

            dst.count(count())
            daTa(dst.daTa())

        }

    }
}

inline class SET_POSITION_TARGET_LOCAL_NED(val data: Cursor) {

    inline fun type_mask(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }


    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    inline fun afx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }


    inline fun afy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }


    inline fun afz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 44, 4).toInt())
    }


    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 48, 4).toInt())
    }


    inline fun coordinate_frame(): SET_POSITION_TARGET_LOCAL_NED_coordinate_frame? {
        if ((data.field_bit != 416 && !data.set_field(416, -1))) return null

        return SET_POSITION_TARGET_LOCAL_NED_coordinate_frame(data)
    }


    companion object {

        val meta = Meta(209, 1, 1, 0, 53, 1, 416, 0, 1)


        inline fun push(src: SET_POSITION_TARGET_LOCAL_NED, time_boot_ms: (src: Int) -> Unit, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, coordinate_frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, type_mask: (src: Short) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, vx: (src: Float) -> Unit, vy: (src: Float) -> Unit, vz: (src: Float) -> Unit, afx: (src: Float) -> Unit, afy: (src: Float) -> Unit, afz: (src: Float) -> Unit, yaw: (src: Float) -> Unit, yaw_rate: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            target_system(src.target_system())

            target_component(src.target_component())

            src.coordinate_frame()?.let { item ->
                coordinate_frame(item.get())
            }

            type_mask(src.type_mask())

            x(src.x())

            y(src.y())

            z(src.z())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            afx(src.afx())

            afy(src.afy())

            afz(src.afz())

            yaw(src.yaw())

            yaw_rate(src.yaw_rate())

        }

    }
}

inline class MOUNT_ORIENTATION(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }


    companion object {

        val meta = Meta(76, 0, 1, 0, 16, 1, 128)


        inline fun push(src: MOUNT_ORIENTATION, time_boot_ms: (src: Int) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

        }

        inline fun pull(dst: MOUNT_ORIENTATION, time_boot_ms: () -> Int, roll: () -> Float, pitch: () -> Float, yaw: () -> Float
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.roll(roll())

            dst.pitch(pitch())

            dst.yaw(yaw())

        }

    }
}

inline class SET_GPS_GLOBAL_ORIGIN(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun latitude(): Int {
        return (get_bytes(data.bytes, data.origin + 1, 4)).toInt()
    }


    inline fun longitude(): Int {
        return (get_bytes(data.bytes, data.origin + 5, 4)).toInt()
    }


    inline fun altitude(): Int {
        return (get_bytes(data.bytes, data.origin + 9, 4)).toInt()
    }


    inline fun time_usec(): Boolean {
        return !(data.field_bit != 104 && !data.set_field(104, -1))
    }


    companion object {

        val meta = Meta(53, 0, 0, 0, 14, 1, 104, 0, 1)


        inline fun push(src: SET_GPS_GLOBAL_ORIGIN, target_system: (src: Byte) -> Unit, latitude: (src: Int) -> Unit, longitude: (src: Int) -> Unit, altitude: (src: Int) -> Unit, time_usec: (src: Long) -> Unit
        ) {

            target_system(src.target_system())

            latitude(src.latitude())

            longitude(src.longitude())

            altitude(src.altitude())

            src.time_usec().let { item ->
                time_usec(item.get())
            }

        }

    }
}

inline class PARAM_EXT_SET(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    inline fun param_id(): PARAM_EXT_SET_param_id? {
        if ((data.field_bit != 19 && !data.set_field(19, -1))) return null

        return PARAM_EXT_SET_param_id(data)
    }


    inline fun param_id_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -19 - 1, reuse)
    }


    inline fun param_id_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(19, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_id_(len: Int): PARAM_EXT_SET_param_id {

        data.set_field(19, minOf(len, 255))
        return PARAM_EXT_SET_param_id(data)
    }

    object param_id {
        const val item_len_max = 255

    }


    inline fun param_value(): PARAM_EXT_SET_param_value? {
        if ((data.field_bit != 20 && !data.set_field(20, -1))) return null

        return PARAM_EXT_SET_param_value(data)
    }


    inline fun param_value_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -20 - 1, reuse)
    }


    inline fun param_value_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(20, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_value_(len: Int): PARAM_EXT_SET_param_value {

        data.set_field(20, minOf(len, 255))
        return PARAM_EXT_SET_param_value(data)
    }

    object param_value {
        const val item_len_max = 255

    }


    inline fun param_type(): PARAM_EXT_SET_param_type? {
        if ((data.field_bit != 21 && !data.set_field(21, -1))) return null

        return PARAM_EXT_SET_param_type(data)
    }


    inline fun param_type(src: MAV_PARAM_EXT_TYPE) {
        if (data.field_bit != 21) data.set_field(21, 0)

        set_bits((-1 src . value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(109, 0, 0, 0, 3, 1, 19, 3, 3)


        inline fun push(src: PARAM_EXT_SET, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, param_id: (src: PARAM_EXT_SET_param_id) -> Unit, param_value: (src: PARAM_EXT_SET_param_value) -> Unit, param_type: (src: com.company.demo.GroundControl.MAV_PARAM_EXT_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.param_id()?.let { item -> param_id(item) }

            src.param_value()?.let { item -> param_value(item) }

            src.param_type()?.let { item ->
                param_type(item.get())
            }

        }

        inline fun pull(dst: PARAM_EXT_SET, target_system: () -> Byte, target_component: () -> Byte, param_id_exist: () -> Int, param_id: (dst: PARAM_EXT_SET_param_id) -> Unit, param_value_exist: () -> Int, param_value: (dst: PARAM_EXT_SET_param_value) -> Unit, param_type_exist: () -> Boolean, param_type: () -> com.company.demo.GroundControl.MAV_PARAM_EXT_TYPE
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())


            param_id_exist().let { len ->
                if (0 < len)
                    param_id(dst.param_id(len))
            }



            param_value_exist().let { len ->
                if (0 < len)
                    param_value(dst.param_value(len))
            }


            if (param_type_exist()) dst.param_type(param_type())


        }

    }
}

inline class AUTOPILOT_VERSION(val data: Cursor) {

    inline fun vendor_id(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun vendor_id_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun product_id(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun product_id_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun flight_sw_version(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun flight_sw_version_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun middleware_sw_version(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }

    inline fun middleware_sw_version_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun os_sw_version(): Int {
        return (get_bytes(data.bytes, data.origin + 12, 4)).toInt()
    }

    inline fun os_sw_version_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun board_version(): Int {
        return (get_bytes(data.bytes, data.origin + 16, 4)).toInt()
    }

    inline fun board_version_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun uid(): Long {
        return (get_bytes(data.bytes, data.origin + 20, 8)).toLong()
    }

    inline fun uid_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 20)
    }

    inline fun flight_custom_version(): AUTOPILOT_VERSION_flight_custom_version {

        return AUTOPILOT_VERSION_flight_custom_version(data)
    }

    inline fun flight_custom_version_(src: ByteArray) {
        val len = minOf(src.size, 8)

        for (index in 0 until len)
            data.bytes[data.origin + 28 + index] = (src[index]).toByte()
    }

    object flight_custom_version {
        const val item_len = 8


    }

    inline fun middleware_custom_version(): AUTOPILOT_VERSION_middleware_custom_version {

        return AUTOPILOT_VERSION_middleware_custom_version(data)
    }

    inline fun middleware_custom_version_(src: ByteArray) {
        val len = minOf(src.size, 8)

        for (index in 0 until len)
            data.bytes[data.origin + 36 + index] = (src[index]).toByte()
    }

    object middleware_custom_version {
        const val item_len = 8


    }

    inline fun os_custom_version(): AUTOPILOT_VERSION_os_custom_version {

        return AUTOPILOT_VERSION_os_custom_version(data)
    }

    inline fun os_custom_version_(src: ByteArray) {
        val len = minOf(src.size, 8)

        for (index in 0 until len)
            data.bytes[data.origin + 44 + index] = (src[index]).toByte()
    }

    object os_custom_version {
        const val item_len = 8


    }


    inline fun capabilities(): AUTOPILOT_VERSION_capabilities? {
        if ((data.field_bit != 419 && !data.set_field(419, -1))) return null

        return AUTOPILOT_VERSION_capabilities(data)
    }


    inline fun capabilities(src: MAV_PROTOCOL_CAPABILITY) {
        if (data.field_bit != 419) data.set_field(419, 0)

        set_bits(MAV_PROTOCOL_CAPABILITY.set(src).toLong(), 5, data.bytes, data.BIT)
    }


    inline fun uid2(d0: Int) {
        if (data.field_bit != 420) data.set_field(420, 0)
        data.set_item(d0, 0)
    }


    inline fun uid2(): AUTOPILOT_VERSION_uid2? {
        return if (data.field_bit == 420 || data.set_field(420, -1)) AUTOPILOT_VERSION_uid2(data) else null
    }

    object uid2 {
        const val d0: Int = 18

    }


    companion object {

        val meta = Meta(60, 2, 4, 1, 53, 1, 419, 3, 2)


        inline fun push(src: AUTOPILOT_VERSION, capabilities: (src: com.company.demo.GroundControl.MAV_PROTOCOL_CAPABILITY) -> Unit, flight_sw_version: (src: Int) -> Unit, middleware_sw_version: (src: Int) -> Unit, os_sw_version: (src: Int) -> Unit, board_version: (src: Int) -> Unit, flight_custom_version: (src: AUTOPILOT_VERSION_flight_custom_version) -> Unit, middleware_custom_version: (src: AUTOPILOT_VERSION_middleware_custom_version) -> Unit, os_custom_version: (src: AUTOPILOT_VERSION_os_custom_version) -> Unit, vendor_id: (src: Short) -> Unit, product_id: (src: Short) -> Unit, uid: (src: Long) -> Unit, uid2: (src: Byte, d0: Int) -> Unit
        ) {

            src.capabilities()?.let { item ->
                capabilities(item.get())
            }

            flight_sw_version(src.flight_sw_version())

            middleware_sw_version(src.middleware_sw_version())

            os_sw_version(src.os_sw_version())

            board_version(src.board_version())
            src.flight_custom_version().let { item ->
                flight_custom_version(item)
            }
            src.middleware_custom_version().let { item ->
                middleware_custom_version(item)
            }
            src.os_custom_version().let { item ->
                os_custom_version(item)
            }

            vendor_id(src.vendor_id())

            product_id(src.product_id())

            uid(src.uid())

            src.uid2()?.let { fld ->
                fld.enumerate { d0 ->
                    uid2(item, d0)
                }
            }

        }

        inline fun pull(dst: AUTOPILOT_VERSION, capabilities_exist: () -> Boolean, capabilities: () -> com.company.demo.GroundControl.MAV_PROTOCOL_CAPABILITY, flight_sw_version: () -> Int, middleware_sw_version: () -> Int, os_sw_version: () -> Int, board_version: () -> Int, flight_custom_version: (dst: AUTOPILOT_VERSION_flight_custom_version) -> Unit, middleware_custom_version: (dst: AUTOPILOT_VERSION_middleware_custom_version) -> Unit, os_custom_version: (dst: AUTOPILOT_VERSION_os_custom_version) -> Unit, vendor_id: () -> Short, product_id: () -> Short, uid: () -> Long, uid2_exist: () -> Boolean, uid2_item_exist: (d0: Int) -> Boolean, uid2: (d0: Int) -> Byte
        ) {

            if (capabilities_exist()) dst.capabilities(capabilities())


            dst.flight_sw_version(flight_sw_version())

            dst.middleware_sw_version(middleware_sw_version())

            dst.os_sw_version(os_sw_version())

            dst.board_version(board_version())
            flight_custom_version(dst.flight_custom_version())
            middleware_custom_version(dst.middleware_custom_version())
            os_custom_version(dst.os_custom_version())

            dst.vendor_id(vendor_id())

            dst.product_id(product_id())

            dst.uid(uid())

            if (uid2_exist())
                for (d0 in 0 until AUTOPILOT_VERSION.uid2.d0) {
                    if (uid2_item_exist(d0)) dst.uid2(uid2(d0), d0)

                }

        }

    }
}

inline class MISSION_REQUEST_LIST(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun mission_type(): MISSION_REQUEST_LIST_mission_type? {
        if ((data.field_bit != 16 && !data.set_field(16, -1))) return null

        return MISSION_REQUEST_LIST_mission_type(data)
    }


    companion object {

        val meta = Meta(25, 0, 0, 0, 3, 1, 16, 0, 1)


        inline fun push(src: MISSION_REQUEST_LIST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class SIMSTATE(val data: Pack.Bytes) {

    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun pitch_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun yaw_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun xacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun xacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun yacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun yacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun zacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun zacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun xgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun xgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun ygyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun ygyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun zgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun zgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun lat(): Int {
        return (get_bytes(data.bytes, data.origin + 36, 4)).toInt()
    }

    inline fun lat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }

    inline fun lng(): Int {
        return (get_bytes(data.bytes, data.origin + 40, 4)).toInt()
    }

    inline fun lng_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }


    companion object {

        val meta = Meta(112, 0, 0, 0, 44, 1, 352)


        inline fun push(src: SIMSTATE, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit, xacc: (src: Float) -> Unit, yacc: (src: Float) -> Unit, zacc: (src: Float) -> Unit, xgyro: (src: Float) -> Unit, ygyro: (src: Float) -> Unit, zgyro: (src: Float) -> Unit, lat: (src: Int) -> Unit, lng: (src: Int) -> Unit
        ) {

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

            xgyro(src.xgyro())

            ygyro(src.ygyro())

            zgyro(src.zgyro())

            lat(src.lat())

            lng(src.lng())

        }

        inline fun pull(dst: SIMSTATE, roll: () -> Float, pitch: () -> Float, yaw: () -> Float, xacc: () -> Float, yacc: () -> Float, zacc: () -> Float, xgyro: () -> Float, ygyro: () -> Float, zgyro: () -> Float, lat: () -> Int, lng: () -> Int
        ) {

            dst.roll(roll())

            dst.pitch(pitch())

            dst.yaw(yaw())

            dst.xacc(xacc())

            dst.yacc(yacc())

            dst.zacc(zacc())

            dst.xgyro(xgyro())

            dst.ygyro(ygyro())

            dst.zgyro(zgyro())

            dst.lat(lat())

            dst.lng(lng())

        }

    }
}

inline class SET_VIDEO_STREAM_SETTINGS(val data: Cursor) {

    inline fun resolution_h(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun resolution_h_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun resolution_v(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun resolution_v_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun rotation(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun rotation_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun bitrate(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun bitrate_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 11]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 11] = (src).toByte()
    }

    inline fun camera_id(): Byte {
        return (data.bytes[data.origin + 12]).toByte()
    }

    inline fun camera_id_(src: Byte) {
        data.bytes[data.origin + 12] = (src).toByte()
    }

    inline fun framerate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 13, 4).toInt())
    }

    inline fun framerate_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 13)
    }


    inline fun uri(): SET_VIDEO_STREAM_SETTINGS_uri? {
        if ((data.field_bit != 138 && !data.set_field(138, -1))) return null

        return SET_VIDEO_STREAM_SETTINGS_uri(data)
    }


    inline fun uri_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -138 - 1, reuse)
    }


    inline fun uri_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(138, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun uri_(len: Int): SET_VIDEO_STREAM_SETTINGS_uri {

        data.set_field(138, minOf(len, 255))
        return SET_VIDEO_STREAM_SETTINGS_uri(data)
    }

    object uri {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(22, 3, 1, 0, 18, 1, 138, 2, 1)


        inline fun push(src: SET_VIDEO_STREAM_SETTINGS, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, camera_id: (src: Byte) -> Unit, framerate: (src: Float) -> Unit, resolution_h: (src: Short) -> Unit, resolution_v: (src: Short) -> Unit, bitrate: (src: Int) -> Unit, rotation: (src: Short) -> Unit, uri: (src: SET_VIDEO_STREAM_SETTINGS_uri) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            camera_id(src.camera_id())

            framerate(src.framerate())

            resolution_h(src.resolution_h())

            resolution_v(src.resolution_v())

            bitrate(src.bitrate())

            rotation(src.rotation())

            src.uri()?.let { item -> uri(item) }

        }

        inline fun pull(dst: SET_VIDEO_STREAM_SETTINGS, target_system: () -> Byte, target_component: () -> Byte, camera_id: () -> Byte, framerate: () -> Float, resolution_h: () -> Short, resolution_v: () -> Short, bitrate: () -> Int, rotation: () -> Short, uri_exist: () -> Int, uri: (dst: SET_VIDEO_STREAM_SETTINGS_uri) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.camera_id(camera_id())

            dst.framerate(framerate())

            dst.resolution_h(resolution_h())

            dst.resolution_v(resolution_v())

            dst.bitrate(bitrate())

            dst.rotation(rotation())


            uri_exist().let { len ->
                if (0 < len)
                    uri(dst.uri(len))
            }


        }

    }
}

inline class PLAY_TUNE(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    inline fun tune(): PLAY_TUNE_tune? {
        if ((data.field_bit != 18 && !data.set_field(18, -1))) return null

        return PLAY_TUNE_tune(data)
    }


    inline fun tune_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -18 - 1, reuse)
    }


    inline fun tune_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(18, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun tune_(len: Int): PLAY_TUNE_tune {

        data.set_field(18, minOf(len, 255))
        return PLAY_TUNE_tune(data)
    }

    object tune {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(203, 0, 0, 0, 3, 1, 18, 2, 1)


        inline fun push(src: PLAY_TUNE, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, tune: (src: PLAY_TUNE_tune) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.tune()?.let { item -> tune(item) }

        }

        inline fun pull(dst: PLAY_TUNE, target_system: () -> Byte, target_component: () -> Byte, tune_exist: () -> Int, tune: (dst: PLAY_TUNE_tune) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())


            tune_exist().let { len ->
                if (0 < len)
                    tune(dst.tune(len))
            }


        }

    }
}

inline class DIGICAM_CONFIGURE(val data: Pack.Bytes) {

    inline fun shutter_speed(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun shutter_speed_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun mode(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun mode_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun aperture(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun aperture_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun iso(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun iso_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun exposure_type(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }

    inline fun exposure_type_(src: Byte) {
        data.bytes[data.origin + 7] = (src).toByte()
    }

    inline fun command_id(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun command_id_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun engine_cut_off(): Byte {
        return (data.bytes[data.origin + 9]).toByte()
    }

    inline fun engine_cut_off_(src: Byte) {
        data.bytes[data.origin + 9] = (src).toByte()
    }

    inline fun extra_param(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun extra_param_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun extra_value(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 11, 4).toInt())
    }

    inline fun extra_value_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 11)
    }


    companion object {

        val meta = Meta(114, 1, 0, 0, 15, 1, 120)


        inline fun push(src: DIGICAM_CONFIGURE, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, mode: (src: Byte) -> Unit, shutter_speed: (src: Short) -> Unit, aperture: (src: Byte) -> Unit, iso: (src: Byte) -> Unit, exposure_type: (src: Byte) -> Unit, command_id: (src: Byte) -> Unit, engine_cut_off: (src: Byte) -> Unit, extra_param: (src: Byte) -> Unit, extra_value: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            mode(src.mode())

            shutter_speed(src.shutter_speed())

            aperture(src.aperture())

            iso(src.iso())

            exposure_type(src.exposure_type())

            command_id(src.command_id())

            engine_cut_off(src.engine_cut_off())

            extra_param(src.extra_param())

            extra_value(src.extra_value())

        }

        inline fun pull(dst: DIGICAM_CONFIGURE, target_system: () -> Byte, target_component: () -> Byte, mode: () -> Byte, shutter_speed: () -> Short, aperture: () -> Byte, iso: () -> Byte, exposure_type: () -> Byte, command_id: () -> Byte, engine_cut_off: () -> Byte, extra_param: () -> Byte, extra_value: () -> Float
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.mode(mode())

            dst.shutter_speed(shutter_speed())

            dst.aperture(aperture())

            dst.iso(iso())

            dst.exposure_type(exposure_type())

            dst.command_id(command_id())

            dst.engine_cut_off(engine_cut_off())

            dst.extra_param(extra_param())

            dst.extra_value(extra_value())

        }

    }
}

inline class SCALED_PRESSURE3(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun press_abs(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun press_abs_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun press_diff(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun press_diff_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun temperature_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }


    companion object {

        val meta = Meta(130, 0, 1, 0, 14, 1, 112)


        inline fun push(src: SCALED_PRESSURE3, time_boot_ms: (src: Int) -> Unit, press_abs: (src: Float) -> Unit, press_diff: (src: Float) -> Unit, temperature: (src: Short) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            press_abs(src.press_abs())

            press_diff(src.press_diff())

            temperature(src.temperature())

        }

        inline fun pull(dst: SCALED_PRESSURE3, time_boot_ms: () -> Int, press_abs: () -> Float, press_diff: () -> Float, temperature: () -> Short
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.press_abs(press_abs())

            dst.press_diff(press_diff())

            dst.temperature(temperature())

        }

    }
}

inline class MISSION_REQUEST_PARTIAL_LIST(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun start_index(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun end_index(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun mission_type(): MISSION_REQUEST_PARTIAL_LIST_mission_type? {
        if ((data.field_bit != 48 && !data.set_field(48, -1))) return null

        return MISSION_REQUEST_PARTIAL_LIST_mission_type(data)
    }


    companion object {

        val meta = Meta(206, 0, 0, 0, 7, 1, 48, 0, 1)


        inline fun push(src: MISSION_REQUEST_PARTIAL_LIST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, start_index: (src: Short) -> Unit, end_index: (src: Short) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            start_index(src.start_index())

            end_index(src.end_index())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class PARAM_EXT_ACK(val data: Cursor) {


    inline fun param_id(): PARAM_EXT_ACK_param_id? {
        if ((data.field_bit != 3 && !data.set_field(3, -1))) return null

        return PARAM_EXT_ACK_param_id(data)
    }


    inline fun param_id_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -3 - 1, reuse)
    }


    inline fun param_id_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(3, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_id_(len: Int): PARAM_EXT_ACK_param_id {

        data.set_field(3, minOf(len, 255))
        return PARAM_EXT_ACK_param_id(data)
    }

    object param_id {
        const val item_len_max = 255

    }


    inline fun param_value(): PARAM_EXT_ACK_param_value? {
        if ((data.field_bit != 4 && !data.set_field(4, -1))) return null

        return PARAM_EXT_ACK_param_value(data)
    }


    inline fun param_value_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -4 - 1, reuse)
    }


    inline fun param_value_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(4, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun param_value_(len: Int): PARAM_EXT_ACK_param_value {

        data.set_field(4, minOf(len, 255))
        return PARAM_EXT_ACK_param_value(data)
    }

    object param_value {
        const val item_len_max = 255

    }


    inline fun param_type(): PARAM_EXT_ACK_param_type? {
        if ((data.field_bit != 5 && !data.set_field(5, -1))) return null

        return PARAM_EXT_ACK_param_type(data)
    }


    inline fun param_type(src: MAV_PARAM_EXT_TYPE) {
        if (data.field_bit != 5) data.set_field(5, 0)

        set_bits((-1 src . value).toULong().toLong(), 4, data.bytes, data.BIT)
    }


    inline fun param_result(): PARAM_EXT_ACK_param_result? {
        if ((data.field_bit != 6 && !data.set_field(6, -1))) return null

        return PARAM_EXT_ACK_param_result(data)
    }


    inline fun param_result(src: PARAM_ACK) {
        if (data.field_bit != 6) data.set_field(6, 0)

        set_bits((src.value).toULong().toLong(), 2, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(51, 0, 0, 0, 1, 1, 3, 3, 4)


        inline fun push(src: PARAM_EXT_ACK, param_id: (src: PARAM_EXT_ACK_param_id) -> Unit, param_value: (src: PARAM_EXT_ACK_param_value) -> Unit, param_type: (src: com.company.demo.GroundControl.MAV_PARAM_EXT_TYPE) -> Unit, param_result: (src: com.company.demo.GroundControl.PARAM_ACK) -> Unit
        ) {

            src.param_id()?.let { item -> param_id(item) }

            src.param_value()?.let { item -> param_value(item) }

            src.param_type()?.let { item ->
                param_type(item.get())
            }

            src.param_result()?.let { item ->
                param_result(item.get())
            }

        }

        inline fun pull(dst: PARAM_EXT_ACK, param_id_exist: () -> Int, param_id: (dst: PARAM_EXT_ACK_param_id) -> Unit, param_value_exist: () -> Int, param_value: (dst: PARAM_EXT_ACK_param_value) -> Unit, param_type_exist: () -> Boolean, param_type: () -> com.company.demo.GroundControl.MAV_PARAM_EXT_TYPE, param_result_exist: () -> Boolean, param_result: () -> com.company.demo.GroundControl.PARAM_ACK
        ) {


            param_id_exist().let { len ->
                if (0 < len)
                    param_id(dst.param_id(len))
            }



            param_value_exist().let { len ->
                if (0 < len)
                    param_value(dst.param_value(len))
            }


            if (param_type_exist()) dst.param_type(param_type())


            if (param_result_exist()) dst.param_result(param_result())


        }

    }
}

inline class UAVCAN_NODE_INFO(val data: Cursor) {

    inline fun uptime_sec(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun uptime_sec_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun sw_vcs_commit(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun sw_vcs_commit_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }

    inline fun hw_version_major(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }

    inline fun hw_version_major_(src: Byte) {
        data.bytes[data.origin + 16] = (src).toByte()
    }

    inline fun hw_version_minor(): Byte {
        return (data.bytes[data.origin + 17]).toByte()
    }

    inline fun hw_version_minor_(src: Byte) {
        data.bytes[data.origin + 17] = (src).toByte()
    }

    inline fun hw_unique_id(): UAVCAN_NODE_INFO_hw_unique_id {

        return UAVCAN_NODE_INFO_hw_unique_id(data)
    }

    inline fun hw_unique_id_(src: ByteArray) {
        val len = minOf(src.size, 16)

        for (index in 0 until len)
            data.bytes[data.origin + 18 + index] = (src[index]).toByte()
    }

    object hw_unique_id {
        const val item_len = 16


    }

    inline fun sw_version_major(): Byte {
        return (data.bytes[data.origin + 34]).toByte()
    }

    inline fun sw_version_major_(src: Byte) {
        data.bytes[data.origin + 34] = (src).toByte()
    }

    inline fun sw_version_minor(): Byte {
        return (data.bytes[data.origin + 35]).toByte()
    }

    inline fun sw_version_minor_(src: Byte) {
        data.bytes[data.origin + 35] = (src).toByte()
    }


    inline fun name(): UAVCAN_NODE_INFO_name? {
        if ((data.field_bit != 290 && !data.set_field(290, -1))) return null

        return UAVCAN_NODE_INFO_name(data)
    }


    inline fun name_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -290 - 1, reuse)
    }


    inline fun name_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(290, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun name_(len: Int): UAVCAN_NODE_INFO_name {

        data.set_field(290, minOf(len, 255))
        return UAVCAN_NODE_INFO_name(data)
    }

    object name {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(127, 0, 2, 1, 37, 1, 290, 2, 1)


        inline fun push(src: UAVCAN_NODE_INFO, time_usec: (src: Long) -> Unit, uptime_sec: (src: Int) -> Unit, name: (src: UAVCAN_NODE_INFO_name) -> Unit, hw_version_major: (src: Byte) -> Unit, hw_version_minor: (src: Byte) -> Unit, hw_unique_id: (src: UAVCAN_NODE_INFO_hw_unique_id) -> Unit, sw_version_major: (src: Byte) -> Unit, sw_version_minor: (src: Byte) -> Unit, sw_vcs_commit: (src: Int) -> Unit
        ) {

            time_usec(src.time_usec())

            uptime_sec(src.uptime_sec())

            src.name()?.let { item -> name(item) }

            hw_version_major(src.hw_version_major())

            hw_version_minor(src.hw_version_minor())
            src.hw_unique_id().let { item ->
                hw_unique_id(item)
            }

            sw_version_major(src.sw_version_major())

            sw_version_minor(src.sw_version_minor())

            sw_vcs_commit(src.sw_vcs_commit())

        }

        inline fun pull(dst: UAVCAN_NODE_INFO, time_usec: () -> Long, uptime_sec: () -> Int, name_exist: () -> Int, name: (dst: UAVCAN_NODE_INFO_name) -> Unit, hw_version_major: () -> Byte, hw_version_minor: () -> Byte, hw_unique_id: (dst: UAVCAN_NODE_INFO_hw_unique_id) -> Unit, sw_version_major: () -> Byte, sw_version_minor: () -> Byte, sw_vcs_commit: () -> Int
        ) {

            dst.time_usec(time_usec())

            dst.uptime_sec(uptime_sec())


            name_exist().let { len ->
                if (0 < len)
                    name(dst.name(len))
            }


            dst.hw_version_major(hw_version_major())

            dst.hw_version_minor(hw_version_minor())
            hw_unique_id(dst.hw_unique_id())

            dst.sw_version_major(sw_version_major())

            dst.sw_version_minor(sw_version_minor())

            dst.sw_vcs_commit(sw_vcs_commit())

        }

    }
}

inline class DATA16(val data: Pack.Bytes) {

    inline fun typE(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun typE_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun len(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun len_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun daTa(): DATA16_daTa {

        return DATA16_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 16)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 16


    }


    companion object {

        val meta = Meta(36, 0, 0, 0, 18, 1, 144)


        inline fun push(src: DATA16, typE: (src: Byte) -> Unit, len: (src: Byte) -> Unit, daTa: (src: DATA16_daTa) -> Unit
        ) {

            typE(src.typE())

            len(src.len())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: DATA16, typE: () -> Byte, len: () -> Byte, daTa: (dst: DATA16_daTa) -> Unit
        ) {

            dst.typE(typE())

            dst.len(len())
            daTa(dst.daTa())

        }

    }
}

inline class SET_MAG_OFFSETS(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun mag_ofs_x(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun mag_ofs_x_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun mag_ofs_y(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun mag_ofs_y_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun mag_ofs_z(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun mag_ofs_z_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }


    companion object {

        val meta = Meta(100, 0, 0, 0, 8, 1, 64)


        inline fun push(src: SET_MAG_OFFSETS, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, mag_ofs_x: (src: Short) -> Unit, mag_ofs_y: (src: Short) -> Unit, mag_ofs_z: (src: Short) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            mag_ofs_x(src.mag_ofs_x())

            mag_ofs_y(src.mag_ofs_y())

            mag_ofs_z(src.mag_ofs_z())

        }

        inline fun pull(dst: SET_MAG_OFFSETS, target_system: () -> Byte, target_component: () -> Byte, mag_ofs_x: () -> Short, mag_ofs_y: () -> Short, mag_ofs_z: () -> Short
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.mag_ofs_x(mag_ofs_x())

            dst.mag_ofs_y(mag_ofs_y())

            dst.mag_ofs_z(mag_ofs_z())

        }

    }
}

inline class AP_ADC(val data: Pack.Bytes) {

    inline fun adc1(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun adc1_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun adc2(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun adc2_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun adc3(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun adc3_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun adc4(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun adc4_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun adc5(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun adc5_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun adc6(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun adc6_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }


    companion object {

        val meta = Meta(198, 6, 0, 0, 12, 1, 96)


        inline fun push(src: AP_ADC, adc1: (src: Short) -> Unit, adc2: (src: Short) -> Unit, adc3: (src: Short) -> Unit, adc4: (src: Short) -> Unit, adc5: (src: Short) -> Unit, adc6: (src: Short) -> Unit
        ) {

            adc1(src.adc1())

            adc2(src.adc2())

            adc3(src.adc3())

            adc4(src.adc4())

            adc5(src.adc5())

            adc6(src.adc6())

        }

        inline fun pull(dst: AP_ADC, adc1: () -> Short, adc2: () -> Short, adc3: () -> Short, adc4: () -> Short, adc5: () -> Short, adc6: () -> Short
        ) {

            dst.adc1(adc1())

            dst.adc2(adc2())

            dst.adc3(adc3())

            dst.adc4(adc4())

            dst.adc5(adc5())

            dst.adc6(adc6())

        }

    }
}

inline class WIND(val data: Pack.Bytes) {

    inline fun direction(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun direction_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun speed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun speed_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun speed_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun speed_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }


    companion object {

        val meta = Meta(97, 0, 0, 0, 12, 1, 96)


        inline fun push(src: WIND, direction: (src: Float) -> Unit, speed: (src: Float) -> Unit, speed_z: (src: Float) -> Unit
        ) {

            direction(src.direction())

            speed(src.speed())

            speed_z(src.speed_z())

        }

        inline fun pull(dst: WIND, direction: () -> Float, speed: () -> Float, speed_z: () -> Float
        ) {

            dst.direction(direction())

            dst.speed(speed())

            dst.speed_z(speed_z())

        }

    }
}

inline class AUTOPILOT_VERSION_REQUEST(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }


    companion object {

        val meta = Meta(142, 0, 0, 0, 2, 1, 16)


        inline fun push(src: AUTOPILOT_VERSION_REQUEST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

        }

        inline fun pull(dst: AUTOPILOT_VERSION_REQUEST, target_system: () -> Byte, target_component: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

        }

    }
}

inline class LOCAL_POSITION_NED(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    companion object {

        val meta = Meta(131, 0, 1, 0, 28, 1, 224)


        inline fun push(src: LOCAL_POSITION_NED, time_boot_ms: (src: Int) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, vx: (src: Float) -> Unit, vy: (src: Float) -> Unit, vz: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            x(src.x())

            y(src.y())

            z(src.z())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

        }

    }
}

inline class DATA_TRANSMISSION_HANDSHAKE(val data: Pack.Bytes) {

    inline fun width(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun width_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun height(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun height_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun packets(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun packets_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun size(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun size_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun typE(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun typE_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun payload(): Byte {
        return (data.bytes[data.origin + 11]).toByte()
    }

    inline fun payload_(src: Byte) {
        data.bytes[data.origin + 11] = (src).toByte()
    }

    inline fun jpg_quality(): Byte {
        return (data.bytes[data.origin + 12]).toByte()
    }

    inline fun jpg_quality_(src: Byte) {
        data.bytes[data.origin + 12] = (src).toByte()
    }


    companion object {

        val meta = Meta(149, 3, 1, 0, 13, 1, 104)


        inline fun push(src: DATA_TRANSMISSION_HANDSHAKE, typE: (src: Byte) -> Unit, size: (src: Int) -> Unit, width: (src: Short) -> Unit, height: (src: Short) -> Unit, packets: (src: Short) -> Unit, payload: (src: Byte) -> Unit, jpg_quality: (src: Byte) -> Unit
        ) {

            typE(src.typE())

            size(src.size())

            width(src.width())

            height(src.height())

            packets(src.packets())

            payload(src.payload())

            jpg_quality(src.jpg_quality())

        }

        inline fun pull(dst: DATA_TRANSMISSION_HANDSHAKE, typE: () -> Byte, size: () -> Int, width: () -> Short, height: () -> Short, packets: () -> Short, payload: () -> Byte, jpg_quality: () -> Byte
        ) {

            dst.typE(typE())

            dst.size(size())

            dst.width(width())

            dst.height(height())

            dst.packets(packets())

            dst.payload(payload())

            dst.jpg_quality(jpg_quality())

        }

    }
}

inline class GPS_GLOBAL_ORIGIN(val data: Cursor) {

    inline fun latitude(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun longitude(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }


    inline fun altitude(): Int {
        return (get_bytes(data.bytes, data.origin + 8, 4)).toInt()
    }


    inline fun time_usec(): Boolean {
        return !(data.field_bit != 96 && !data.set_field(96, -1))
    }


    companion object {

        val meta = Meta(185, 0, 0, 0, 13, 1, 96, 0, 1)


        inline fun push(src: GPS_GLOBAL_ORIGIN, latitude: (src: Int) -> Unit, longitude: (src: Int) -> Unit, altitude: (src: Int) -> Unit, time_usec: (src: Long) -> Unit
        ) {

            latitude(src.latitude())

            longitude(src.longitude())

            altitude(src.altitude())

            src.time_usec().let { item ->
                time_usec(item.get())
            }

        }

    }
}

inline class SCALED_IMU2(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun xacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }

    inline fun yacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 6)
    }

    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }

    inline fun zacc_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 8)
    }

    inline fun xgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }

    inline fun xgyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 10)
    }

    inline fun ygyro(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun ygyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }

    inline fun zgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }

    inline fun zgyro_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 14)
    }

    inline fun xmag(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }

    inline fun xmag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 16)
    }

    inline fun ymag(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }

    inline fun ymag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 18)
    }

    inline fun zmag(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }

    inline fun zmag_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 20)
    }


    companion object {

        val meta = Meta(178, 0, 1, 0, 22, 1, 176)


        inline fun push(src: SCALED_IMU2, time_boot_ms: (src: Int) -> Unit, xacc: (src: Short) -> Unit, yacc: (src: Short) -> Unit, zacc: (src: Short) -> Unit, xgyro: (src: Short) -> Unit, ygyro: (src: Short) -> Unit, zgyro: (src: Short) -> Unit, xmag: (src: Short) -> Unit, ymag: (src: Short) -> Unit, zmag: (src: Short) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

            xgyro(src.xgyro())

            ygyro(src.ygyro())

            zgyro(src.zgyro())

            xmag(src.xmag())

            ymag(src.ymag())

            zmag(src.zmag())

        }

        inline fun pull(dst: SCALED_IMU2, time_boot_ms: () -> Int, xacc: () -> Short, yacc: () -> Short, zacc: () -> Short, xgyro: () -> Short, ygyro: () -> Short, zgyro: () -> Short, xmag: () -> Short, ymag: () -> Short, zmag: () -> Short
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.xacc(xacc())

            dst.yacc(yacc())

            dst.zacc(zacc())

            dst.xgyro(xgyro())

            dst.ygyro(ygyro())

            dst.zgyro(zgyro())

            dst.xmag(xmag())

            dst.ymag(ymag())

            dst.zmag(zmag())

        }

    }
}

inline class ATTITUDE_QUATERNION(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }


    inline fun q1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun q2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun q3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun q4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    companion object {

        val meta = Meta(71, 0, 1, 0, 32, 1, 256)


        inline fun push(src: ATTITUDE_QUATERNION, time_boot_ms: (src: Int) -> Unit, q1: (src: Float) -> Unit, q2: (src: Float) -> Unit, q3: (src: Float) -> Unit, q4: (src: Float) -> Unit, rollspeed: (src: Float) -> Unit, pitchspeed: (src: Float) -> Unit, yawspeed: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            q1(src.q1())

            q2(src.q2())

            q3(src.q3())

            q4(src.q4())

            rollspeed(src.rollspeed())

            pitchspeed(src.pitchspeed())

            yawspeed(src.yawspeed())

        }

    }
}

inline class DATA64(val data: Pack.Bytes) {

    inline fun typE(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun typE_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun len(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun len_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun daTa(): DATA64_daTa {

        return DATA64_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 64)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 64


    }


    companion object {

        val meta = Meta(141, 0, 0, 0, 66, 1, 528)


        inline fun push(src: DATA64, typE: (src: Byte) -> Unit, len: (src: Byte) -> Unit, daTa: (src: DATA64_daTa) -> Unit
        ) {

            typE(src.typE())

            len(src.len())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: DATA64, typE: () -> Byte, len: () -> Byte, daTa: (dst: DATA64_daTa) -> Unit
        ) {

            dst.typE(typE())

            dst.len(len())
            daTa(dst.daTa())

        }

    }
}

inline class HIL_ACTUATOR_CONTROLS(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun flags(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }


    inline fun controls(): HIL_ACTUATOR_CONTROLS_controls {

        return HIL_ACTUATOR_CONTROLS_controls(data)
    }

    object controls {
        const val item_len = 16


    }


    inline fun mode(): HIL_ACTUATOR_CONTROLS_mode? {
        if ((data.field_bit != 640 && !data.set_field(640, -1))) return null

        return HIL_ACTUATOR_CONTROLS_mode(data)
    }


    companion object {

        val meta = Meta(199, 0, 0, 2, 81, 1, 640, 0, 1)


        inline fun push(src: HIL_ACTUATOR_CONTROLS, time_usec: (src: Long) -> Unit, controls: (src: HIL_ACTUATOR_CONTROLS_controls) -> Unit, mode: (src: com.company.demo.GroundControl.MAV_MODE) -> Unit, flags: (src: Long) -> Unit
        ) {

            time_usec(src.time_usec())
            src.controls().let { item ->
                controls(item)
            }

            src.mode()?.let { item ->
                mode(item.get())
            }

            flags(src.flags())

        }

    }
}

inline class POSITION_TARGET_LOCAL_NED(val data: Cursor) {

    inline fun type_mask(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 2, 4)).toInt()
    }


    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }


    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }


    inline fun vx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }


    inline fun vy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }


    inline fun vz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }


    inline fun afx(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }


    inline fun afy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 34, 4).toInt())
    }


    inline fun afz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 38, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 42, 4).toInt())
    }


    inline fun yaw_rate(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 46, 4).toInt())
    }


    inline fun coordinate_frame(): POSITION_TARGET_LOCAL_NED_coordinate_frame? {
        if ((data.field_bit != 400 && !data.set_field(400, -1))) return null

        return POSITION_TARGET_LOCAL_NED_coordinate_frame(data)
    }


    companion object {

        val meta = Meta(129, 1, 1, 0, 51, 1, 400, 0, 1)


        inline fun push(src: POSITION_TARGET_LOCAL_NED, time_boot_ms: (src: Int) -> Unit, coordinate_frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, type_mask: (src: Short) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, vx: (src: Float) -> Unit, vy: (src: Float) -> Unit, vz: (src: Float) -> Unit, afx: (src: Float) -> Unit, afy: (src: Float) -> Unit, afz: (src: Float) -> Unit, yaw: (src: Float) -> Unit, yaw_rate: (src: Float) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            src.coordinate_frame()?.let { item ->
                coordinate_frame(item.get())
            }

            type_mask(src.type_mask())

            x(src.x())

            y(src.y())

            z(src.z())

            vx(src.vx())

            vy(src.vy())

            vz(src.vz())

            afx(src.afx())

            afy(src.afy())

            afz(src.afz())

            yaw(src.yaw())

            yaw_rate(src.yaw_rate())

        }

    }
}

inline class GIMBAL_REPORT(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun delta_time(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 2, 4).toInt())
    }

    inline fun delta_time_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 2)
    }

    inline fun delta_angle_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }

    inline fun delta_angle_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun delta_angle_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }

    inline fun delta_angle_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun delta_angle_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun delta_angle_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun delta_velocity_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun delta_velocity_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun delta_velocity_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun delta_velocity_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

    inline fun delta_velocity_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }

    inline fun delta_velocity_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 26)
    }

    inline fun joint_roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }

    inline fun joint_roll_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 30)
    }

    inline fun joint_el(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 34, 4).toInt())
    }

    inline fun joint_el_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 34)
    }

    inline fun joint_az(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 38, 4).toInt())
    }

    inline fun joint_az_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 38)
    }


    companion object {

        val meta = Meta(11, 0, 0, 0, 42, 1, 336)


        inline fun push(src: GIMBAL_REPORT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, delta_time: (src: Float) -> Unit, delta_angle_x: (src: Float) -> Unit, delta_angle_y: (src: Float) -> Unit, delta_angle_z: (src: Float) -> Unit, delta_velocity_x: (src: Float) -> Unit, delta_velocity_y: (src: Float) -> Unit, delta_velocity_z: (src: Float) -> Unit, joint_roll: (src: Float) -> Unit, joint_el: (src: Float) -> Unit, joint_az: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            delta_time(src.delta_time())

            delta_angle_x(src.delta_angle_x())

            delta_angle_y(src.delta_angle_y())

            delta_angle_z(src.delta_angle_z())

            delta_velocity_x(src.delta_velocity_x())

            delta_velocity_y(src.delta_velocity_y())

            delta_velocity_z(src.delta_velocity_z())

            joint_roll(src.joint_roll())

            joint_el(src.joint_el())

            joint_az(src.joint_az())

        }

        inline fun pull(dst: GIMBAL_REPORT, target_system: () -> Byte, target_component: () -> Byte, delta_time: () -> Float, delta_angle_x: () -> Float, delta_angle_y: () -> Float, delta_angle_z: () -> Float, delta_velocity_x: () -> Float, delta_velocity_y: () -> Float, delta_velocity_z: () -> Float, joint_roll: () -> Float, joint_el: () -> Float, joint_az: () -> Float
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.delta_time(delta_time())

            dst.delta_angle_x(delta_angle_x())

            dst.delta_angle_y(delta_angle_y())

            dst.delta_angle_z(delta_angle_z())

            dst.delta_velocity_x(delta_velocity_x())

            dst.delta_velocity_y(delta_velocity_y())

            dst.delta_velocity_z(delta_velocity_z())

            dst.joint_roll(joint_roll())

            dst.joint_el(joint_el())

            dst.joint_az(joint_az())

        }

    }
}

inline class DEVICE_OP_WRITE(val data: Cursor) {

    inline fun request_id(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun request_id_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun bus(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun bus_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun address(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }

    inline fun address_(src: Byte) {
        data.bytes[data.origin + 7] = (src).toByte()
    }

    inline fun regstart(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun regstart_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun count(): Byte {
        return (data.bytes[data.origin + 9]).toByte()
    }

    inline fun count_(src: Byte) {
        data.bytes[data.origin + 9] = (src).toByte()
    }

    inline fun daTa(): DEVICE_OP_WRITE_daTa {

        return DEVICE_OP_WRITE_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 128)

        for (index in 0 until len)
            data.bytes[data.origin + 10 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 128


    }


    inline fun bustype(): DEVICE_OP_WRITE_bustype? {
        if ((data.field_bit != 1106 && !data.set_field(1106, -1))) return null

        return DEVICE_OP_WRITE_bustype(data)
    }


    inline fun bustype(src: DEVICE_OP_BUSTYPE) {
        if (data.field_bit != 1106) data.set_field(1106, 0)

        set_bits((src.value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    inline fun busname(): DEVICE_OP_WRITE_busname? {
        if ((data.field_bit != 1107 && !data.set_field(1107, -1))) return null

        return DEVICE_OP_WRITE_busname(data)
    }


    inline fun busname_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -1107 - 1, reuse)
    }


    inline fun busname_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(1107, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun busname_(len: Int): DEVICE_OP_WRITE_busname {

        data.set_field(1107, minOf(len, 255))
        return DEVICE_OP_WRITE_busname(data)
    }

    object busname {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(105, 0, 1, 0, 139, 1, 1106, 2, 2)


        inline fun push(src: DEVICE_OP_WRITE, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, request_id: (src: Int) -> Unit, bustype: (src: com.company.demo.GroundControl.DEVICE_OP_BUSTYPE) -> Unit, bus: (src: Byte) -> Unit, address: (src: Byte) -> Unit, busname: (src: DEVICE_OP_WRITE_busname) -> Unit, regstart: (src: Byte) -> Unit, count: (src: Byte) -> Unit, daTa: (src: DEVICE_OP_WRITE_daTa) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            request_id(src.request_id())

            src.bustype()?.let { item ->
                bustype(item.get())
            }

            bus(src.bus())

            address(src.address())

            src.busname()?.let { item -> busname(item) }

            regstart(src.regstart())

            count(src.count())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: DEVICE_OP_WRITE, target_system: () -> Byte, target_component: () -> Byte, request_id: () -> Int, bustype_exist: () -> Boolean, bustype: () -> com.company.demo.GroundControl.DEVICE_OP_BUSTYPE, bus: () -> Byte, address: () -> Byte, busname_exist: () -> Int, busname: (dst: DEVICE_OP_WRITE_busname) -> Unit, regstart: () -> Byte, count: () -> Byte, daTa: (dst: DEVICE_OP_WRITE_daTa) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.request_id(request_id())

            if (bustype_exist()) dst.bustype(bustype())


            dst.bus(bus())

            dst.address(address())


            busname_exist().let { len ->
                if (0 < len)
                    busname(dst.busname(len))
            }


            dst.regstart(regstart())

            dst.count(count())
            daTa(dst.daTa())

        }

    }
}

inline class DISTANCE_SENSOR(val data: Cursor) {

    inline fun min_distance(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun min_distance_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun max_distance(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun max_distance_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun current_distance(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun current_distance_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun id(): Byte {
        return (data.bytes[data.origin + 10]).toByte()
    }

    inline fun id_(src: Byte) {
        data.bytes[data.origin + 10] = (src).toByte()
    }

    inline fun covariance(): Byte {
        return (data.bytes[data.origin + 11]).toByte()
    }

    inline fun covariance_(src: Byte) {
        data.bytes[data.origin + 11] = (src).toByte()
    }


    inline fun typE(): DISTANCE_SENSOR_typE? {
        if ((data.field_bit != 98 && !data.set_field(98, -1))) return null

        return DISTANCE_SENSOR_typE(data)
    }


    inline fun typE(src: MAV_DISTANCE_SENSOR) {
        if (data.field_bit != 98) data.set_field(98, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun orientation(): DISTANCE_SENSOR_orientation? {
        if ((data.field_bit != 99 && !data.set_field(99, -1))) return null

        return DISTANCE_SENSOR_orientation(data)
    }


    inline fun orientation(src: MAV_SENSOR_ORIENTATION) {
        if (data.field_bit != 99) data.set_field(99, 0)

        set_bits((src.value).toULong().toLong(), 6, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(61, 3, 1, 0, 13, 1, 98, 2, 2)


        inline fun push(src: DISTANCE_SENSOR, time_boot_ms: (src: Int) -> Unit, min_distance: (src: Short) -> Unit, max_distance: (src: Short) -> Unit, current_distance: (src: Short) -> Unit, typE: (src: com.company.demo.GroundControl.MAV_DISTANCE_SENSOR) -> Unit, id: (src: Byte) -> Unit, orientation: (src: com.company.demo.GroundControl.MAV_SENSOR_ORIENTATION) -> Unit, covariance: (src: Byte) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            min_distance(src.min_distance())

            max_distance(src.max_distance())

            current_distance(src.current_distance())

            src.typE()?.let { item ->
                typE(item.get())
            }

            id(src.id())

            src.orientation()?.let { item ->
                orientation(item.get())
            }

            covariance(src.covariance())

        }

        inline fun pull(dst: DISTANCE_SENSOR, time_boot_ms: () -> Int, min_distance: () -> Short, max_distance: () -> Short, current_distance: () -> Short, typE_exist: () -> Boolean, typE: () -> com.company.demo.GroundControl.MAV_DISTANCE_SENSOR, id: () -> Byte, orientation_exist: () -> Boolean, orientation: () -> com.company.demo.GroundControl.MAV_SENSOR_ORIENTATION, covariance: () -> Byte
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.min_distance(min_distance())

            dst.max_distance(max_distance())

            dst.current_distance(current_distance())

            if (typE_exist()) dst.typE(typE())


            dst.id(id())

            if (orientation_exist()) dst.orientation(orientation())


            dst.covariance(covariance())

        }

    }
}

inline class HIL_OPTICAL_FLOW(val data: Pack.Bytes) {

    inline fun integration_time_us(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun integration_time_us_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun time_delta_distance_us(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun time_delta_distance_us_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }

    inline fun sensor_id(): Byte {
        return (data.bytes[data.origin + 16]).toByte()
    }

    inline fun sensor_id_(src: Byte) {
        data.bytes[data.origin + 16] = (src).toByte()
    }

    inline fun integrated_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 17, 4).toInt())
    }

    inline fun integrated_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 17)
    }

    inline fun integrated_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 21, 4).toInt())
    }

    inline fun integrated_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 21)
    }

    inline fun integrated_xgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 25, 4).toInt())
    }

    inline fun integrated_xgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 25)
    }

    inline fun integrated_ygyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 29, 4).toInt())
    }

    inline fun integrated_ygyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 29)
    }

    inline fun integrated_zgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 33, 4).toInt())
    }

    inline fun integrated_zgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 33)
    }

    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 37, 2)).toShort()
    }

    inline fun temperature_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 37)
    }

    inline fun quality(): Byte {
        return (data.bytes[data.origin + 39]).toByte()
    }

    inline fun quality_(src: Byte) {
        data.bytes[data.origin + 39] = (src).toByte()
    }

    inline fun distance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun distance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }


    companion object {

        val meta = Meta(116, 0, 2, 1, 44, 1, 352)


        inline fun push(src: HIL_OPTICAL_FLOW, time_usec: (src: Long) -> Unit, sensor_id: (src: Byte) -> Unit, integration_time_us: (src: Int) -> Unit, integrated_x: (src: Float) -> Unit, integrated_y: (src: Float) -> Unit, integrated_xgyro: (src: Float) -> Unit, integrated_ygyro: (src: Float) -> Unit, integrated_zgyro: (src: Float) -> Unit, temperature: (src: Short) -> Unit, quality: (src: Byte) -> Unit, time_delta_distance_us: (src: Int) -> Unit, distance: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            sensor_id(src.sensor_id())

            integration_time_us(src.integration_time_us())

            integrated_x(src.integrated_x())

            integrated_y(src.integrated_y())

            integrated_xgyro(src.integrated_xgyro())

            integrated_ygyro(src.integrated_ygyro())

            integrated_zgyro(src.integrated_zgyro())

            temperature(src.temperature())

            quality(src.quality())

            time_delta_distance_us(src.time_delta_distance_us())

            distance(src.distance())

        }

        inline fun pull(dst: HIL_OPTICAL_FLOW, time_usec: () -> Long, sensor_id: () -> Byte, integration_time_us: () -> Int, integrated_x: () -> Float, integrated_y: () -> Float, integrated_xgyro: () -> Float, integrated_ygyro: () -> Float, integrated_zgyro: () -> Float, temperature: () -> Short, quality: () -> Byte, time_delta_distance_us: () -> Int, distance: () -> Float
        ) {

            dst.time_usec(time_usec())

            dst.sensor_id(sensor_id())

            dst.integration_time_us(integration_time_us())

            dst.integrated_x(integrated_x())

            dst.integrated_y(integrated_y())

            dst.integrated_xgyro(integrated_xgyro())

            dst.integrated_ygyro(integrated_ygyro())

            dst.integrated_zgyro(integrated_zgyro())

            dst.temperature(temperature())

            dst.quality(quality())

            dst.time_delta_distance_us(time_delta_distance_us())

            dst.distance(distance())

        }

    }
}

inline class SCALED_PRESSURE2(val data: Pack.Bytes) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun press_abs(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun press_abs_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun press_diff(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun press_diff_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun temperature(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }

    inline fun temperature_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 12)
    }


    companion object {

        val meta = Meta(151, 0, 1, 0, 14, 1, 112)


        inline fun push(src: SCALED_PRESSURE2, time_boot_ms: (src: Int) -> Unit, press_abs: (src: Float) -> Unit, press_diff: (src: Float) -> Unit, temperature: (src: Short) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            press_abs(src.press_abs())

            press_diff(src.press_diff())

            temperature(src.temperature())

        }

        inline fun pull(dst: SCALED_PRESSURE2, time_boot_ms: () -> Int, press_abs: () -> Float, press_diff: () -> Float, temperature: () -> Short
        ) {

            dst.time_boot_ms(time_boot_ms())

            dst.press_abs(press_abs())

            dst.press_diff(press_diff())

            dst.temperature(temperature())

        }

    }
}

inline class WIND_COV(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun wind_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun wind_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }

    inline fun wind_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }

    inline fun wind_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 12)
    }

    inline fun wind_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }

    inline fun wind_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 16)
    }

    inline fun var_horiz(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }

    inline fun var_horiz_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 20)
    }

    inline fun var_vert(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }

    inline fun var_vert_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 24)
    }

    inline fun wind_alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }

    inline fun wind_alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 28)
    }

    inline fun horiz_accuracy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }

    inline fun horiz_accuracy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 32)
    }

    inline fun vert_accuracy(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 36, 4).toInt())
    }

    inline fun vert_accuracy_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 36)
    }


    companion object {

        val meta = Meta(160, 0, 0, 1, 40, 1, 320)


        inline fun push(src: WIND_COV, time_usec: (src: Long) -> Unit, wind_x: (src: Float) -> Unit, wind_y: (src: Float) -> Unit, wind_z: (src: Float) -> Unit, var_horiz: (src: Float) -> Unit, var_vert: (src: Float) -> Unit, wind_alt: (src: Float) -> Unit, horiz_accuracy: (src: Float) -> Unit, vert_accuracy: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            wind_x(src.wind_x())

            wind_y(src.wind_y())

            wind_z(src.wind_z())

            var_horiz(src.var_horiz())

            var_vert(src.var_vert())

            wind_alt(src.wind_alt())

            horiz_accuracy(src.horiz_accuracy())

            vert_accuracy(src.vert_accuracy())

        }

        inline fun pull(dst: WIND_COV, time_usec: () -> Long, wind_x: () -> Float, wind_y: () -> Float, wind_z: () -> Float, var_horiz: () -> Float, var_vert: () -> Float, wind_alt: () -> Float, horiz_accuracy: () -> Float, vert_accuracy: () -> Float
        ) {

            dst.time_usec(time_usec())

            dst.wind_x(wind_x())

            dst.wind_y(wind_y())

            dst.wind_z(wind_z())

            dst.var_horiz(var_horiz())

            dst.var_vert(var_vert())

            dst.wind_alt(wind_alt())

            dst.horiz_accuracy(horiz_accuracy())

            dst.vert_accuracy(vert_accuracy())

        }

    }
}

inline class CHANGE_OPERATOR_CONTROL(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun control_request(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun version(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun passkey(): CHANGE_OPERATOR_CONTROL_passkey? {
        if ((data.field_bit != 26 && !data.set_field(26, -1))) return null

        return CHANGE_OPERATOR_CONTROL_passkey(data)
    }


    object passkey {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(154, 0, 0, 0, 4, 1, 26, 2, 1)


        inline fun push(src: CHANGE_OPERATOR_CONTROL, target_system: (src: Byte) -> Unit, control_request: (src: Byte) -> Unit, version: (src: Byte) -> Unit, passkey: (src: CHANGE_OPERATOR_CONTROL_passkey) -> Unit
        ) {

            target_system(src.target_system())

            control_request(src.control_request())

            version(src.version())

            src.passkey()?.let { item -> passkey(item) }

        }

    }
}

inline class GOPRO_SET_REQUEST(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun value(): GOPRO_SET_REQUEST_value {

        return GOPRO_SET_REQUEST_value(data)
    }

    inline fun value_(src: ByteArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object value {
        const val item_len = 4


    }


    inline fun cmd_id(): GOPRO_SET_REQUEST_cmd_id? {
        if ((data.field_bit != 48 && !data.set_field(48, -1))) return null

        return GOPRO_SET_REQUEST_cmd_id(data)
    }


    inline fun cmd_id(src: GOPRO_COMMAND) {
        if (data.field_bit != 48) data.set_field(48, 0)

        set_bits((src.value).toULong().toLong(), 5, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(118, 0, 0, 0, 7, 1, 48, 0, 1)


        inline fun push(src: GOPRO_SET_REQUEST, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, cmd_id: (src: com.company.demo.GroundControl.GOPRO_COMMAND) -> Unit, value: (src: GOPRO_SET_REQUEST_value) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.cmd_id()?.let { item ->
                cmd_id(item.get())
            }
            src.value().let { item ->
                value(item)
            }

        }

        inline fun pull(dst: GOPRO_SET_REQUEST, target_system: () -> Byte, target_component: () -> Byte, cmd_id_exist: () -> Boolean, cmd_id: () -> com.company.demo.GroundControl.GOPRO_COMMAND, value: (dst: GOPRO_SET_REQUEST_value) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            if (cmd_id_exist()) dst.cmd_id(cmd_id())

            value(dst.value())

        }

    }
}

inline class SYS_STATUS(val data: Cursor) {

    inline fun load(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun voltage_battery(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }


    inline fun drop_rate_comm(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }


    inline fun errors_comm(): Short {
        return (get_bytes(data.bytes, data.origin + 6, 2)).toShort()
    }


    inline fun errors_count1(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun errors_count2(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun errors_count3(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun errors_count4(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    inline fun current_battery(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }


    inline fun battery_remaining(): Byte {
        return (data.bytes[data.origin + 18]).toByte()
    }


    inline fun onboard_control_sensors_present(): SYS_STATUS_onboard_control_sensors_present? {
        if ((data.field_bit != 154 && !data.set_field(154, -1))) return null

        return SYS_STATUS_onboard_control_sensors_present(data)
    }


    inline fun onboard_control_sensors_enabled(): SYS_STATUS_onboard_control_sensors_enabled? {
        if ((data.field_bit != 155 && !data.set_field(155, -1))) return null

        return SYS_STATUS_onboard_control_sensors_enabled(data)
    }


    inline fun onboard_control_sensors_health(): SYS_STATUS_onboard_control_sensors_health? {
        if ((data.field_bit != 156 && !data.set_field(156, -1))) return null

        return SYS_STATUS_onboard_control_sensors_health(data)
    }


    companion object {

        val meta = Meta(132, 8, 0, 0, 20, 1, 154, 2, 3)


        inline fun push(src: SYS_STATUS, onboard_control_sensors_present: (src: com.company.demo.GroundControl.MAV_SYS_STATUS_SENSOR) -> Unit, onboard_control_sensors_enabled: (src: com.company.demo.GroundControl.MAV_SYS_STATUS_SENSOR) -> Unit, onboard_control_sensors_health: (src: com.company.demo.GroundControl.MAV_SYS_STATUS_SENSOR) -> Unit, load: (src: Short) -> Unit, voltage_battery: (src: Short) -> Unit, current_battery: (src: Short) -> Unit, battery_remaining: (src: Byte) -> Unit, drop_rate_comm: (src: Short) -> Unit, errors_comm: (src: Short) -> Unit, errors_count1: (src: Short) -> Unit, errors_count2: (src: Short) -> Unit, errors_count3: (src: Short) -> Unit, errors_count4: (src: Short) -> Unit
        ) {

            src.onboard_control_sensors_present()?.let { item ->
                onboard_control_sensors_present(item.get())
            }

            src.onboard_control_sensors_enabled()?.let { item ->
                onboard_control_sensors_enabled(item.get())
            }

            src.onboard_control_sensors_health()?.let { item ->
                onboard_control_sensors_health(item.get())
            }

            load(src.load())

            voltage_battery(src.voltage_battery())

            current_battery(src.current_battery())

            battery_remaining(src.battery_remaining())

            drop_rate_comm(src.drop_rate_comm())

            errors_comm(src.errors_comm())

            errors_count1(src.errors_count1())

            errors_count2(src.errors_count2())

            errors_count3(src.errors_count3())

            errors_count4(src.errors_count4())

        }

    }
}

inline class MISSION_ITEM(val data: Cursor) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    inline fun current(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun autocontinue(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }


    inline fun param1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }


    inline fun param2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }


    inline fun param3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }


    inline fun param4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }


    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }


    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }


    inline fun frame(): MISSION_ITEM_frame? {
        if ((data.field_bit != 274 && !data.set_field(274, -1))) return null

        return MISSION_ITEM_frame(data)
    }


    inline fun command(): MISSION_ITEM_command? {
        if ((data.field_bit != 275 && !data.set_field(275, -1))) return null

        return MISSION_ITEM_command(data)
    }


    inline fun mission_type(): MISSION_ITEM_mission_type? {
        if ((data.field_bit != 276 && !data.set_field(276, -1))) return null

        return MISSION_ITEM_mission_type(data)
    }


    companion object {

        val meta = Meta(35, 1, 0, 0, 35, 1, 274, 2, 3)


        inline fun push(src: MISSION_ITEM, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, seq: (src: Short) -> Unit, frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, command: (src: com.company.demo.GroundControl.MAV_CMD) -> Unit, current: (src: Byte) -> Unit, autocontinue: (src: Byte) -> Unit, param1: (src: Float) -> Unit, param2: (src: Float) -> Unit, param3: (src: Float) -> Unit, param4: (src: Float) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            seq(src.seq())

            src.frame()?.let { item ->
                frame(item.get())
            }

            src.command()?.let { item ->
                command(item.get())
            }

            current(src.current())

            autocontinue(src.autocontinue())

            param1(src.param1())

            param2(src.param2())

            param3(src.param3())

            param4(src.param4())

            x(src.x())

            y(src.y())

            z(src.z())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class RAW_IMU(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun xacc(): Short {
        return (get_bytes(data.bytes, data.origin + 8, 2)).toShort()
    }


    inline fun yacc(): Short {
        return (get_bytes(data.bytes, data.origin + 10, 2)).toShort()
    }


    inline fun zacc(): Short {
        return (get_bytes(data.bytes, data.origin + 12, 2)).toShort()
    }


    inline fun xgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 14, 2)).toShort()
    }


    inline fun ygyro(): Short {
        return (get_bytes(data.bytes, data.origin + 16, 2)).toShort()
    }


    inline fun zgyro(): Short {
        return (get_bytes(data.bytes, data.origin + 18, 2)).toShort()
    }


    inline fun xmag(): Short {
        return (get_bytes(data.bytes, data.origin + 20, 2)).toShort()
    }


    inline fun ymag(): Short {
        return (get_bytes(data.bytes, data.origin + 22, 2)).toShort()
    }


    inline fun zmag(): Short {
        return (get_bytes(data.bytes, data.origin + 24, 2)).toShort()
    }


    companion object {

        val meta = Meta(163, 0, 0, 1, 26, 1, 208)


        inline fun push(src: RAW_IMU, time_usec: (src: Long) -> Unit, xacc: (src: Short) -> Unit, yacc: (src: Short) -> Unit, zacc: (src: Short) -> Unit, xgyro: (src: Short) -> Unit, ygyro: (src: Short) -> Unit, zgyro: (src: Short) -> Unit, xmag: (src: Short) -> Unit, ymag: (src: Short) -> Unit, zmag: (src: Short) -> Unit
        ) {

            time_usec(src.time_usec())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

            xgyro(src.xgyro())

            ygyro(src.ygyro())

            zgyro(src.zgyro())

            xmag(src.xmag())

            ymag(src.ymag())

            zmag(src.zmag())

        }

    }
}

inline class COMMAND_INT(val data: Cursor) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }


    inline fun current(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun autocontinue(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    inline fun param1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }


    inline fun param2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun param3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun param4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun x(): Int {
        return (get_bytes(data.bytes, data.origin + 20, 4)).toInt()
    }


    inline fun y(): Int {
        return (get_bytes(data.bytes, data.origin + 24, 4)).toInt()
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    inline fun frame(): COMMAND_INT_frame? {
        if ((data.field_bit != 258 && !data.set_field(258, -1))) return null

        return COMMAND_INT_frame(data)
    }


    inline fun command(): COMMAND_INT_command? {
        if ((data.field_bit != 259 && !data.set_field(259, -1))) return null

        return COMMAND_INT_command(data)
    }


    companion object {

        val meta = Meta(167, 0, 0, 0, 33, 1, 258, 2, 2)


        inline fun push(src: COMMAND_INT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, command: (src: com.company.demo.GroundControl.MAV_CMD) -> Unit, current: (src: Byte) -> Unit, autocontinue: (src: Byte) -> Unit, param1: (src: Float) -> Unit, param2: (src: Float) -> Unit, param3: (src: Float) -> Unit, param4: (src: Float) -> Unit, x: (src: Int) -> Unit, y: (src: Int) -> Unit, z: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            src.frame()?.let { item ->
                frame(item.get())
            }

            src.command()?.let { item ->
                command(item.get())
            }

            current(src.current())

            autocontinue(src.autocontinue())

            param1(src.param1())

            param2(src.param2())

            param3(src.param3())

            param4(src.param4())

            x(src.x())

            y(src.y())

            z(src.z())

        }

    }
}

inline class OPTICAL_FLOW(val data: Cursor) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun sensor_id(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }


    inline fun flow_x(): Short {
        return (get_bytes(data.bytes, data.origin + 9, 2)).toShort()
    }


    inline fun flow_y(): Short {
        return (get_bytes(data.bytes, data.origin + 11, 2)).toShort()
    }


    inline fun flow_comp_m_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 13, 4).toInt())
    }


    inline fun flow_comp_m_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 17, 4).toInt())
    }


    inline fun quality(): Byte {
        return (data.bytes[data.origin + 21]).toByte()
    }


    inline fun ground_distance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }


    inline fun flow_rate_x(): Boolean {
        return !(data.field_bit != 208 && !data.set_field(208, -1))
    }


    inline fun flow_rate_y(): Boolean {
        return !(data.field_bit != 209 && !data.set_field(209, -1))
    }


    companion object {

        val meta = Meta(13, 0, 0, 1, 27, 1, 208, 0, 2)


        inline fun push(src: OPTICAL_FLOW, time_usec: (src: Long) -> Unit, sensor_id: (src: Byte) -> Unit, flow_x: (src: Short) -> Unit, flow_y: (src: Short) -> Unit, flow_comp_m_x: (src: Float) -> Unit, flow_comp_m_y: (src: Float) -> Unit, quality: (src: Byte) -> Unit, ground_distance: (src: Float) -> Unit, flow_rate_x: (src: Float) -> Unit, flow_rate_y: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            sensor_id(src.sensor_id())

            flow_x(src.flow_x())

            flow_y(src.flow_y())

            flow_comp_m_x(src.flow_comp_m_x())

            flow_comp_m_y(src.flow_comp_m_y())

            quality(src.quality())

            ground_distance(src.ground_distance())

            src.flow_rate_x().let { item ->
                flow_rate_x(item.get())
            }

            src.flow_rate_y().let { item ->
                flow_rate_y(item.get())
            }

        }

    }
}

inline class MISSION_ITEM_INT(val data: Cursor) {

    inline fun seq(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }


    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }


    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }


    inline fun current(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }


    inline fun autocontinue(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }


    inline fun param1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 6, 4).toInt())
    }


    inline fun param2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }


    inline fun param3(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }


    inline fun param4(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }


    inline fun x(): Int {
        return (get_bytes(data.bytes, data.origin + 22, 4)).toInt()
    }


    inline fun y(): Int {
        return (get_bytes(data.bytes, data.origin + 26, 4)).toInt()
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }


    inline fun frame(): MISSION_ITEM_INT_frame? {
        if ((data.field_bit != 274 && !data.set_field(274, -1))) return null

        return MISSION_ITEM_INT_frame(data)
    }


    inline fun command(): MISSION_ITEM_INT_command? {
        if ((data.field_bit != 275 && !data.set_field(275, -1))) return null

        return MISSION_ITEM_INT_command(data)
    }


    inline fun mission_type(): MISSION_ITEM_INT_mission_type? {
        if ((data.field_bit != 276 && !data.set_field(276, -1))) return null

        return MISSION_ITEM_INT_mission_type(data)
    }


    companion object {

        val meta = Meta(59, 1, 0, 0, 35, 1, 274, 2, 3)


        inline fun push(src: MISSION_ITEM_INT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, seq: (src: Short) -> Unit, frame: (src: com.company.demo.GroundControl.MAV_FRAME) -> Unit, command: (src: com.company.demo.GroundControl.MAV_CMD) -> Unit, current: (src: Byte) -> Unit, autocontinue: (src: Byte) -> Unit, param1: (src: Float) -> Unit, param2: (src: Float) -> Unit, param3: (src: Float) -> Unit, param4: (src: Float) -> Unit, x: (src: Int) -> Unit, y: (src: Int) -> Unit, z: (src: Float) -> Unit, mission_type: (src: com.company.demo.GroundControl.MAV_MISSION_TYPE) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            seq(src.seq())

            src.frame()?.let { item ->
                frame(item.get())
            }

            src.command()?.let { item ->
                command(item.get())
            }

            current(src.current())

            autocontinue(src.autocontinue())

            param1(src.param1())

            param2(src.param2())

            param3(src.param3())

            param4(src.param4())

            x(src.x())

            y(src.y())

            z(src.z())

            src.mission_type()?.let { item ->
                mission_type(item.get())
            }

        }

    }
}

inline class VISION_POSITION_DELTA(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 0)
    }

    inline fun time_delta_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 8, 8)).toLong()
    }

    inline fun time_delta_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 8)
    }

    inline fun angle_delta(): VISION_POSITION_DELTA_angle_delta {

        return VISION_POSITION_DELTA_angle_delta(data)
    }

    inline fun angle_delta_(src: FloatArray) {
        val len = minOf(src.size, 3)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 16 + index * 4)
    }

    object angle_delta {
        const val item_len = 3


    }

    inline fun position_delta(): VISION_POSITION_DELTA_position_delta {

        return VISION_POSITION_DELTA_position_delta(data)
    }

    inline fun position_delta_(src: FloatArray) {
        val len = minOf(src.size, 3)

        for (index in 0 until len)
            set_bytes(java.lang.Float.floatToIntBits(src[index]).toULong().toLong(), 4, data.bytes, data.origin + 28 + index * 4)
    }

    object position_delta {
        const val item_len = 3


    }

    inline fun confidence(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 40, 4).toInt())
    }

    inline fun confidence_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 40)
    }


    companion object {

        val meta = Meta(12, 0, 0, 2, 44, 1, 352)


        inline fun push(src: VISION_POSITION_DELTA, time_usec: (src: Long) -> Unit, time_delta_usec: (src: Long) -> Unit, angle_delta: (src: VISION_POSITION_DELTA_angle_delta) -> Unit, position_delta: (src: VISION_POSITION_DELTA_position_delta) -> Unit, confidence: (src: Float) -> Unit
        ) {

            time_usec(src.time_usec())

            time_delta_usec(src.time_delta_usec())
            src.angle_delta().let { item ->
                angle_delta(item)
            }
            src.position_delta().let { item ->
                position_delta(item)
            }

            confidence(src.confidence())

        }

        inline fun pull(dst: VISION_POSITION_DELTA, time_usec: () -> Long, time_delta_usec: () -> Long, angle_delta: (dst: VISION_POSITION_DELTA_angle_delta) -> Unit, position_delta: (dst: VISION_POSITION_DELTA_position_delta) -> Unit, confidence: () -> Float
        ) {

            dst.time_usec(time_usec())

            dst.time_delta_usec(time_delta_usec())
            angle_delta(dst.angle_delta())
            position_delta(dst.position_delta())

            dst.confidence(confidence())

        }

    }
}

inline class LOGGING_DATA(val data: Pack.Bytes) {

    inline fun sequence(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun sequence_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun length(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun length_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun first_message_offset(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun first_message_offset_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun daTa(): LOGGING_DATA_daTa {

        return LOGGING_DATA_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 249)

        for (index in 0 until len)
            data.bytes[data.origin + 6 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 249


    }


    companion object {

        val meta = Meta(75, 1, 0, 0, 255, 1, 2040)


        inline fun push(src: LOGGING_DATA, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, sequence: (src: Short) -> Unit, length: (src: Byte) -> Unit, first_message_offset: (src: Byte) -> Unit, daTa: (src: LOGGING_DATA_daTa) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            sequence(src.sequence())

            length(src.length())

            first_message_offset(src.first_message_offset())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: LOGGING_DATA, target_system: () -> Byte, target_component: () -> Byte, sequence: () -> Short, length: () -> Byte, first_message_offset: () -> Byte, daTa: (dst: LOGGING_DATA_daTa) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.sequence(sequence())

            dst.length(length())

            dst.first_message_offset(first_message_offset())
            daTa(dst.daTa())

        }

    }
}

inline class DEVICE_OP_READ(val data: Cursor) {

    inline fun request_id(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun request_id_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun bus(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun bus_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun address(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }

    inline fun address_(src: Byte) {
        data.bytes[data.origin + 7] = (src).toByte()
    }

    inline fun regstart(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun regstart_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }

    inline fun count(): Byte {
        return (data.bytes[data.origin + 9]).toByte()
    }

    inline fun count_(src: Byte) {
        data.bytes[data.origin + 9] = (src).toByte()
    }


    inline fun bustype(): DEVICE_OP_READ_bustype? {
        if ((data.field_bit != 82 && !data.set_field(82, -1))) return null

        return DEVICE_OP_READ_bustype(data)
    }


    inline fun bustype(src: DEVICE_OP_BUSTYPE) {
        if (data.field_bit != 82) data.set_field(82, 0)

        set_bits((src.value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    inline fun busname(): DEVICE_OP_READ_busname? {
        if ((data.field_bit != 83 && !data.set_field(83, -1))) return null

        return DEVICE_OP_READ_busname(data)
    }


    inline fun busname_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -83 - 1, reuse)
    }


    inline fun busname_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(83, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun busname_(len: Int): DEVICE_OP_READ_busname {

        data.set_field(83, minOf(len, 255))
        return DEVICE_OP_READ_busname(data)
    }

    object busname {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(70, 0, 1, 0, 11, 1, 82, 2, 2)


        inline fun push(src: DEVICE_OP_READ, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, request_id: (src: Int) -> Unit, bustype: (src: com.company.demo.GroundControl.DEVICE_OP_BUSTYPE) -> Unit, bus: (src: Byte) -> Unit, address: (src: Byte) -> Unit, busname: (src: DEVICE_OP_READ_busname) -> Unit, regstart: (src: Byte) -> Unit, count: (src: Byte) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            request_id(src.request_id())

            src.bustype()?.let { item ->
                bustype(item.get())
            }

            bus(src.bus())

            address(src.address())

            src.busname()?.let { item -> busname(item) }

            regstart(src.regstart())

            count(src.count())

        }

        inline fun pull(dst: DEVICE_OP_READ, target_system: () -> Byte, target_component: () -> Byte, request_id: () -> Int, bustype_exist: () -> Boolean, bustype: () -> com.company.demo.GroundControl.DEVICE_OP_BUSTYPE, bus: () -> Byte, address: () -> Byte, busname_exist: () -> Int, busname: (dst: DEVICE_OP_READ_busname) -> Unit, regstart: () -> Byte, count: () -> Byte
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.request_id(request_id())

            if (bustype_exist()) dst.bustype(bustype())


            dst.bus(bus())

            dst.address(address())


            busname_exist().let { len ->
                if (0 < len)
                    busname(dst.busname(len))
            }


            dst.regstart(regstart())

            dst.count(count())

        }

    }
}

inline class MAG_CAL_PROGRESS(val data: Cursor) {

    inline fun compass_id(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun compass_id_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun cal_mask(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun cal_mask_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun attempt(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun attempt_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun completion_pct(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun completion_pct_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun completion_mask(): MAG_CAL_PROGRESS_completion_mask {

        return MAG_CAL_PROGRESS_completion_mask(data)
    }

    inline fun completion_mask_(src: ByteArray) {
        val len = minOf(src.size, 10)

        for (index in 0 until len)
            data.bytes[data.origin + 4 + index] = (src[index]).toByte()
    }

    object completion_mask {
        const val item_len = 10


    }

    inline fun direction_x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun direction_x_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun direction_y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun direction_y_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun direction_z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun direction_z_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }


    inline fun cal_status(): MAG_CAL_PROGRESS_cal_status? {
        if ((data.field_bit != 208 && !data.set_field(208, -1))) return null

        return MAG_CAL_PROGRESS_cal_status(data)
    }


    inline fun cal_status(src: MAG_CAL_STATUS) {
        if (data.field_bit != 208) data.set_field(208, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(57, 0, 0, 0, 27, 1, 208, 0, 1)


        inline fun push(src: MAG_CAL_PROGRESS, compass_id: (src: Byte) -> Unit, cal_mask: (src: Byte) -> Unit, cal_status: (src: com.company.demo.GroundControl.MAG_CAL_STATUS) -> Unit, attempt: (src: Byte) -> Unit, completion_pct: (src: Byte) -> Unit, completion_mask: (src: MAG_CAL_PROGRESS_completion_mask) -> Unit, direction_x: (src: Float) -> Unit, direction_y: (src: Float) -> Unit, direction_z: (src: Float) -> Unit
        ) {

            compass_id(src.compass_id())

            cal_mask(src.cal_mask())

            src.cal_status()?.let { item ->
                cal_status(item.get())
            }

            attempt(src.attempt())

            completion_pct(src.completion_pct())
            src.completion_mask().let { item ->
                completion_mask(item)
            }

            direction_x(src.direction_x())

            direction_y(src.direction_y())

            direction_z(src.direction_z())

        }

        inline fun pull(dst: MAG_CAL_PROGRESS, compass_id: () -> Byte, cal_mask: () -> Byte, cal_status_exist: () -> Boolean, cal_status: () -> com.company.demo.GroundControl.MAG_CAL_STATUS, attempt: () -> Byte, completion_pct: () -> Byte, completion_mask: (dst: MAG_CAL_PROGRESS_completion_mask) -> Unit, direction_x: () -> Float, direction_y: () -> Float, direction_z: () -> Float
        ) {

            dst.compass_id(compass_id())

            dst.cal_mask(cal_mask())

            if (cal_status_exist()) dst.cal_status(cal_status())


            dst.attempt(attempt())

            dst.completion_pct(completion_pct())
            completion_mask(dst.completion_mask())

            dst.direction_x(direction_x())

            dst.direction_y(direction_y())

            dst.direction_z(direction_z())

        }

    }
}

inline class HIGHRES_IMU(val data: Pack.Bytes) {

    inline fun fields_updated(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun fields_updated_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 2, 8)).toLong()
    }

    inline fun time_usec_(src: Long) {
        set_bytes((src).toULong().toLong(), 8, data.bytes, data.origin + 2)
    }

    inline fun xacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 10, 4).toInt())
    }

    inline fun xacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun yacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 14, 4).toInt())
    }

    inline fun yacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun zacc(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 18, 4).toInt())
    }

    inline fun zacc_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun xgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 22, 4).toInt())
    }

    inline fun xgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

    inline fun ygyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 26, 4).toInt())
    }

    inline fun ygyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 26)
    }

    inline fun zgyro(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 30, 4).toInt())
    }

    inline fun zgyro_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 30)
    }

    inline fun xmag(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 34, 4).toInt())
    }

    inline fun xmag_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 34)
    }

    inline fun ymag(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 38, 4).toInt())
    }

    inline fun ymag_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 38)
    }

    inline fun zmag(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 42, 4).toInt())
    }

    inline fun zmag_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 42)
    }

    inline fun abs_pressure(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 46, 4).toInt())
    }

    inline fun abs_pressure_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 46)
    }

    inline fun diff_pressure(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 50, 4).toInt())
    }

    inline fun diff_pressure_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 50)
    }

    inline fun pressure_alt(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 54, 4).toInt())
    }

    inline fun pressure_alt_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 54)
    }

    inline fun temperature(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 58, 4).toInt())
    }

    inline fun temperature_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 58)
    }


    companion object {

        val meta = Meta(137, 1, 0, 1, 62, 1, 496)


        inline fun push(src: HIGHRES_IMU, time_usec: (src: Long) -> Unit, xacc: (src: Float) -> Unit, yacc: (src: Float) -> Unit, zacc: (src: Float) -> Unit, xgyro: (src: Float) -> Unit, ygyro: (src: Float) -> Unit, zgyro: (src: Float) -> Unit, xmag: (src: Float) -> Unit, ymag: (src: Float) -> Unit, zmag: (src: Float) -> Unit, abs_pressure: (src: Float) -> Unit, diff_pressure: (src: Float) -> Unit, pressure_alt: (src: Float) -> Unit, temperature: (src: Float) -> Unit, fields_updated: (src: Short) -> Unit
        ) {

            time_usec(src.time_usec())

            xacc(src.xacc())

            yacc(src.yacc())

            zacc(src.zacc())

            xgyro(src.xgyro())

            ygyro(src.ygyro())

            zgyro(src.zgyro())

            xmag(src.xmag())

            ymag(src.ymag())

            zmag(src.zmag())

            abs_pressure(src.abs_pressure())

            diff_pressure(src.diff_pressure())

            pressure_alt(src.pressure_alt())

            temperature(src.temperature())

            fields_updated(src.fields_updated())

        }

        inline fun pull(dst: HIGHRES_IMU, time_usec: () -> Long, xacc: () -> Float, yacc: () -> Float, zacc: () -> Float, xgyro: () -> Float, ygyro: () -> Float, zgyro: () -> Float, xmag: () -> Float, ymag: () -> Float, zmag: () -> Float, abs_pressure: () -> Float, diff_pressure: () -> Float, pressure_alt: () -> Float, temperature: () -> Float, fields_updated: () -> Short
        ) {

            dst.time_usec(time_usec())

            dst.xacc(xacc())

            dst.yacc(yacc())

            dst.zacc(zacc())

            dst.xgyro(xgyro())

            dst.ygyro(ygyro())

            dst.zgyro(zgyro())

            dst.xmag(xmag())

            dst.ymag(ymag())

            dst.zmag(zmag())

            dst.abs_pressure(abs_pressure())

            dst.diff_pressure(diff_pressure())

            dst.pressure_alt(pressure_alt())

            dst.temperature(temperature())

            dst.fields_updated(fields_updated())

        }

    }
}

inline class EXTENDED_SYS_STATE(val data: Cursor) {


    inline fun vtol_state(): EXTENDED_SYS_STATE_vtol_state? {
        if ((data.field_bit != 0 && !data.set_field(0, -1))) return null

        return EXTENDED_SYS_STATE_vtol_state(data)
    }


    inline fun vtol_state(src: MAV_VTOL_STATE) {
        if (data.field_bit != 0) data.set_field(0, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun landed_state(): EXTENDED_SYS_STATE_landed_state? {
        if ((data.field_bit != 1 && !data.set_field(1, -1))) return null

        return EXTENDED_SYS_STATE_landed_state(data)
    }


    inline fun landed_state(src: MAV_LANDED_STATE) {
        if (data.field_bit != 1) data.set_field(1, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(176, 0, 0, 0, 1, 1, 0, 0, 2)


        inline fun push(src: EXTENDED_SYS_STATE, vtol_state: (src: com.company.demo.GroundControl.MAV_VTOL_STATE) -> Unit, landed_state: (src: com.company.demo.GroundControl.MAV_LANDED_STATE) -> Unit
        ) {

            src.vtol_state()?.let { item ->
                vtol_state(item.get())
            }

            src.landed_state()?.let { item ->
                landed_state(item.get())
            }

        }

        inline fun pull(dst: EXTENDED_SYS_STATE, vtol_state_exist: () -> Boolean, vtol_state: () -> com.company.demo.GroundControl.MAV_VTOL_STATE, landed_state_exist: () -> Boolean, landed_state: () -> com.company.demo.GroundControl.MAV_LANDED_STATE
        ) {

            if (vtol_state_exist()) dst.vtol_state(vtol_state())


            if (landed_state_exist()) dst.landed_state(landed_state())


        }

    }
}

inline class UAVIONIX_ADSB_OUT_DYNAMIC(val data: Cursor) {

    inline fun accuracyVert(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun accuracyVert_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun accuracyVel(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun accuracyVel_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun squawk(): Short {
        return (get_bytes(data.bytes, data.origin + 4, 2)).toShort()
    }

    inline fun squawk_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 4)
    }

    inline fun utcTime(): Int {
        return (get_bytes(data.bytes, data.origin + 6, 4)).toInt()
    }

    inline fun utcTime_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 6)
    }

    inline fun accuracyHor(): Int {
        return (get_bytes(data.bytes, data.origin + 10, 4)).toInt()
    }

    inline fun accuracyHor_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 10)
    }

    inline fun gpsLat(): Int {
        return (get_bytes(data.bytes, data.origin + 14, 4)).toInt()
    }

    inline fun gpsLat_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 14)
    }

    inline fun gpsLon(): Int {
        return (get_bytes(data.bytes, data.origin + 18, 4)).toInt()
    }

    inline fun gpsLon_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 18)
    }

    inline fun gpsAlt(): Int {
        return (get_bytes(data.bytes, data.origin + 22, 4)).toInt()
    }

    inline fun gpsAlt_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 22)
    }

    inline fun numSats(): Byte {
        return (data.bytes[data.origin + 26]).toByte()
    }

    inline fun numSats_(src: Byte) {
        data.bytes[data.origin + 26] = (src).toByte()
    }

    inline fun baroAltMSL(): Int {
        return (get_bytes(data.bytes, data.origin + 27, 4)).toInt()
    }

    inline fun baroAltMSL_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 27)
    }

    inline fun velVert(): Short {
        return (get_bytes(data.bytes, data.origin + 31, 2)).toShort()
    }

    inline fun velVert_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 31)
    }

    inline fun velNS(): Short {
        return (get_bytes(data.bytes, data.origin + 33, 2)).toShort()
    }

    inline fun velNS_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 33)
    }

    inline fun VelEW(): Short {
        return (get_bytes(data.bytes, data.origin + 35, 2)).toShort()
    }

    inline fun VelEW_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 35)
    }


    inline fun gpsFix(): UAVIONIX_ADSB_OUT_DYNAMIC_gpsFix? {
        if ((data.field_bit != 298 && !data.set_field(298, -1))) return null

        return UAVIONIX_ADSB_OUT_DYNAMIC_gpsFix(data)
    }


    inline fun gpsFix(src: UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX) {
        if (data.field_bit != 298) data.set_field(298, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun emergencyStatus(): UAVIONIX_ADSB_OUT_DYNAMIC_emergencyStatus? {
        if ((data.field_bit != 299 && !data.set_field(299, -1))) return null

        return UAVIONIX_ADSB_OUT_DYNAMIC_emergencyStatus(data)
    }


    inline fun emergencyStatus(src: UAVIONIX_ADSB_EMERGENCY_STATUS) {
        if (data.field_bit != 299) data.set_field(299, 0)

        set_bits((src.value).toULong().toLong(), 3, data.bytes, data.BIT)
    }


    inline fun state(): UAVIONIX_ADSB_OUT_DYNAMIC_state? {
        if ((data.field_bit != 300 && !data.set_field(300, -1))) return null

        return UAVIONIX_ADSB_OUT_DYNAMIC_state(data)
    }


    inline fun state(src: UAVIONIX_ADSB_OUT_DYNAMIC_STATE) {
        if (data.field_bit != 300) data.set_field(300, 0)

        set_bits(UAVIONIX_ADSB_OUT_DYNAMIC_STATE.set(src).toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(143, 3, 2, 0, 38, 1, 298, 2, 3)


        inline fun push(src: UAVIONIX_ADSB_OUT_DYNAMIC, utcTime: (src: Int) -> Unit, gpsLat: (src: Int) -> Unit, gpsLon: (src: Int) -> Unit, gpsAlt: (src: Int) -> Unit, gpsFix: (src: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX) -> Unit, numSats: (src: Byte) -> Unit, baroAltMSL: (src: Int) -> Unit, accuracyHor: (src: Int) -> Unit, accuracyVert: (src: Short) -> Unit, accuracyVel: (src: Short) -> Unit, velVert: (src: Short) -> Unit, velNS: (src: Short) -> Unit, VelEW: (src: Short) -> Unit, emergencyStatus: (src: com.company.demo.GroundControl.UAVIONIX_ADSB_EMERGENCY_STATUS) -> Unit, state: (src: com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC_STATE) -> Unit, squawk: (src: Short) -> Unit
        ) {

            utcTime(src.utcTime())

            gpsLat(src.gpsLat())

            gpsLon(src.gpsLon())

            gpsAlt(src.gpsAlt())

            src.gpsFix()?.let { item ->
                gpsFix(item.get())
            }

            numSats(src.numSats())

            baroAltMSL(src.baroAltMSL())

            accuracyHor(src.accuracyHor())

            accuracyVert(src.accuracyVert())

            accuracyVel(src.accuracyVel())

            velVert(src.velVert())

            velNS(src.velNS())

            VelEW(src.VelEW())

            src.emergencyStatus()?.let { item ->
                emergencyStatus(item.get())
            }

            src.state()?.let { item ->
                state(item.get())
            }

            squawk(src.squawk())

        }

        inline fun pull(dst: UAVIONIX_ADSB_OUT_DYNAMIC, utcTime: () -> Int, gpsLat: () -> Int, gpsLon: () -> Int, gpsAlt: () -> Int, gpsFix_exist: () -> Boolean, gpsFix: () -> com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX, numSats: () -> Byte, baroAltMSL: () -> Int, accuracyHor: () -> Int, accuracyVert: () -> Short, accuracyVel: () -> Short, velVert: () -> Short, velNS: () -> Short, VelEW: () -> Short, emergencyStatus_exist: () -> Boolean, emergencyStatus: () -> com.company.demo.GroundControl.UAVIONIX_ADSB_EMERGENCY_STATUS, state_exist: () -> Boolean, state: () -> com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC_STATE, squawk: () -> Short
        ) {

            dst.utcTime(utcTime())

            dst.gpsLat(gpsLat())

            dst.gpsLon(gpsLon())

            dst.gpsAlt(gpsAlt())

            if (gpsFix_exist()) dst.gpsFix(gpsFix())


            dst.numSats(numSats())

            dst.baroAltMSL(baroAltMSL())

            dst.accuracyHor(accuracyHor())

            dst.accuracyVert(accuracyVert())

            dst.accuracyVel(accuracyVel())

            dst.velVert(velVert())

            dst.velNS(velNS())

            dst.VelEW(VelEW())

            if (emergencyStatus_exist()) dst.emergencyStatus(emergencyStatus())


            if (state_exist()) dst.state(state())


            dst.squawk(squawk())

        }

    }
}

inline class GOPRO_GET_RESPONSE(val data: Cursor) {

    inline fun value(): GOPRO_GET_RESPONSE_value {

        return GOPRO_GET_RESPONSE_value(data)
    }

    inline fun value_(src: ByteArray) {
        val len = minOf(src.size, 4)

        for (index in 0 until len)
            data.bytes[data.origin + 0 + index] = (src[index]).toByte()
    }

    object value {
        const val item_len = 4


    }


    inline fun cmd_id(): GOPRO_GET_RESPONSE_cmd_id? {
        if ((data.field_bit != 32 && !data.set_field(32, -1))) return null

        return GOPRO_GET_RESPONSE_cmd_id(data)
    }


    inline fun cmd_id(src: GOPRO_COMMAND) {
        if (data.field_bit != 32) data.set_field(32, 0)

        set_bits((src.value).toULong().toLong(), 5, data.bytes, data.BIT)
    }


    inline fun status(): GOPRO_GET_RESPONSE_status? {
        if ((data.field_bit != 33 && !data.set_field(33, -1))) return null

        return GOPRO_GET_RESPONSE_status(data)
    }


    inline fun status(src: GOPRO_REQUEST_STATUS) {
        if (data.field_bit != 33) data.set_field(33, 0)

        set_bits((src.value).toULong().toLong(), 1, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(208, 0, 0, 0, 5, 1, 32, 0, 2)


        inline fun push(src: GOPRO_GET_RESPONSE, cmd_id: (src: com.company.demo.GroundControl.GOPRO_COMMAND) -> Unit, status: (src: com.company.demo.GroundControl.GOPRO_REQUEST_STATUS) -> Unit, value: (src: GOPRO_GET_RESPONSE_value) -> Unit
        ) {

            src.cmd_id()?.let { item ->
                cmd_id(item.get())
            }

            src.status()?.let { item ->
                status(item.get())
            }
            src.value().let { item ->
                value(item)
            }

        }

        inline fun pull(dst: GOPRO_GET_RESPONSE, cmd_id_exist: () -> Boolean, cmd_id: () -> com.company.demo.GroundControl.GOPRO_COMMAND, status_exist: () -> Boolean, status: () -> com.company.demo.GroundControl.GOPRO_REQUEST_STATUS, value: (dst: GOPRO_GET_RESPONSE_value) -> Unit
        ) {

            if (cmd_id_exist()) dst.cmd_id(cmd_id())


            if (status_exist()) dst.status(status())

            value(dst.value())

        }

    }
}

inline class GPS_INJECT_DATA(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun len(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun len_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun daTa(): GPS_INJECT_DATA_daTa {

        return GPS_INJECT_DATA_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 110)

        for (index in 0 until len)
            data.bytes[data.origin + 3 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 110


    }


    companion object {

        val meta = Meta(52, 0, 0, 0, 113, 1, 904)


        inline fun push(src: GPS_INJECT_DATA, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, len: (src: Byte) -> Unit, daTa: (src: GPS_INJECT_DATA_daTa) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            len(src.len())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: GPS_INJECT_DATA, target_system: () -> Byte, target_component: () -> Byte, len: () -> Byte, daTa: (dst: GPS_INJECT_DATA_daTa) -> Unit
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.len(len())
            daTa(dst.daTa())

        }

    }
}

inline class UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(val data: Cursor) {


    inline fun rfHealth(): UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_rfHealth? {
        if ((data.field_bit != 0 && !data.set_field(0, -1))) return null

        return UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_rfHealth(data)
    }


    inline fun rfHealth(src: UAVIONIX_ADSB_RF_HEALTH) {
        if (data.field_bit != 0) data.set_field(0, 0)

        set_bits(UAVIONIX_ADSB_RF_HEALTH.set(src).toLong(), 3, data.bytes, data.BIT)
    }


    companion object {

        val meta = Meta(26, 0, 0, 0, 1, 1, 0, 0, 1)


        inline fun push(src: UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT, rfHealth: (src: com.company.demo.GroundControl.UAVIONIX_ADSB_RF_HEALTH) -> Unit
        ) {

            src.rfHealth()?.let { item ->
                rfHealth(item.get())
            }

        }

        inline fun pull(dst: UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT, rfHealth_exist: () -> Boolean, rfHealth: () -> com.company.demo.GroundControl.UAVIONIX_ADSB_RF_HEALTH
        ) {

            if (rfHealth_exist()) dst.rfHealth(rfHealth())


        }

    }
}

inline class ATTITUDE_QUATERNION_COV(val data: Pack.Bytes) {

    inline fun time_usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun q(): ATTITUDE_QUATERNION_COV_q {

        return ATTITUDE_QUATERNION_COV_q(data)
    }

    object q {
        const val item_len = 4


    }

    inline fun rollspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun pitchspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    inline fun yawspeed(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 32, 4).toInt())
    }


    inline fun covariance(): ATTITUDE_QUATERNION_COV_covariance {

        return ATTITUDE_QUATERNION_COV_covariance(data)
    }

    object covariance {
        const val item_len = 9


    }


    companion object {

        val meta = Meta(182, 0, 0, 1, 72, 1, 576)


        inline fun push(src: ATTITUDE_QUATERNION_COV, time_usec: (src: Long) -> Unit, q: (src: ATTITUDE_QUATERNION_COV_q) -> Unit, rollspeed: (src: Float) -> Unit, pitchspeed: (src: Float) -> Unit, yawspeed: (src: Float) -> Unit, covariance: (src: ATTITUDE_QUATERNION_COV_covariance) -> Unit
        ) {

            time_usec(src.time_usec())
            src.q().let { item ->
                q(item)
            }

            rollspeed(src.rollspeed())

            pitchspeed(src.pitchspeed())

            yawspeed(src.yawspeed())
            src.covariance().let { item ->
                covariance(item)
            }

        }

    }
}

inline class NAMED_VALUE_INT(val data: Cursor) {

    inline fun time_boot_ms(): Int {
        return (get_bytes(data.bytes, data.origin + 0, 4)).toInt()
    }

    inline fun time_boot_ms_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun value(): Int {
        return (get_bytes(data.bytes, data.origin + 4, 4)).toInt()
    }

    inline fun value_(src: Int) {
        set_bytes((src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }


    inline fun name(): NAMED_VALUE_INT_name? {
        if ((data.field_bit != 66 && !data.set_field(66, -1))) return null

        return NAMED_VALUE_INT_name(data)
    }


    inline fun name_(src: String, reuse: CharArray?): CharArray {
        return data.encode(src, -66 - 1, reuse)
    }


    inline fun name_(src: ByteArray) {

        val len = minOf(src.size, 255)
        data.set_field(66, len)

        for (index in 0 until len)
            data.bytes[data.BYTE + index] = src[index]
    }

    inline fun name_(len: Int): NAMED_VALUE_INT_name {

        data.set_field(66, minOf(len, 255))
        return NAMED_VALUE_INT_name(data)
    }

    object name {
        const val item_len_max = 255

    }


    companion object {

        val meta = Meta(21, 0, 1, 0, 9, 1, 66, 2, 1)


        inline fun push(src: NAMED_VALUE_INT, time_boot_ms: (src: Int) -> Unit, name: (src: NAMED_VALUE_INT_name) -> Unit, value: (src: Int) -> Unit
        ) {

            time_boot_ms(src.time_boot_ms())

            src.name()?.let { item -> name(item) }

            value(src.value())

        }

        inline fun pull(dst: NAMED_VALUE_INT, time_boot_ms: () -> Int, name_exist: () -> Int, name: (dst: NAMED_VALUE_INT_name) -> Unit, value: () -> Int
        ) {

            dst.time_boot_ms(time_boot_ms())


            name_exist().let { len ->
                if (0 < len)
                    name(dst.name(len))
            }


            dst.value(value())

        }

    }
}

inline class RPM(val data: Pack.Bytes) {

    inline fun rpm1(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun rpm1_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun rpm2(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun rpm2_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }


    companion object {

        val meta = Meta(183, 0, 0, 0, 8, 1, 64)


        inline fun push(src: RPM, rpm1: (src: Float) -> Unit, rpm2: (src: Float) -> Unit
        ) {

            rpm1(src.rpm1())

            rpm2(src.rpm2())

        }

        inline fun pull(dst: RPM, rpm1: () -> Float, rpm2: () -> Float
        ) {

            dst.rpm1(rpm1())

            dst.rpm2(rpm2())

        }

    }
}

inline class GPS_RTCM_DATA(val data: Pack.Bytes) {

    inline fun flags(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun flags_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun len(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun len_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun daTa(): GPS_RTCM_DATA_daTa {

        return GPS_RTCM_DATA_daTa(data)
    }

    inline fun daTa_(src: ByteArray) {
        val len = minOf(src.size, 180)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object daTa {
        const val item_len = 180


    }


    companion object {

        val meta = Meta(88, 0, 0, 0, 182, 1, 1456)


        inline fun push(src: GPS_RTCM_DATA, flags: (src: Byte) -> Unit, len: (src: Byte) -> Unit, daTa: (src: GPS_RTCM_DATA_daTa) -> Unit
        ) {

            flags(src.flags())

            len(src.len())
            src.daTa().let { item ->
                daTa(item)
            }

        }

        inline fun pull(dst: GPS_RTCM_DATA, flags: () -> Byte, len: () -> Byte, daTa: (dst: GPS_RTCM_DATA_daTa) -> Unit
        ) {

            dst.flags(flags())

            dst.len(len())
            daTa(dst.daTa())

        }

    }
}

inline class GLOBAL_VISION_POSITION_ESTIMATE(val data: Pack.Bytes) {

    inline fun usec(): Long {
        return (get_bytes(data.bytes, data.origin + 0, 8)).toLong()
    }


    inline fun x(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }


    inline fun y(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 12, 4).toInt())
    }


    inline fun z(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 16, 4).toInt())
    }


    inline fun roll(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 20, 4).toInt())
    }


    inline fun pitch(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 24, 4).toInt())
    }


    inline fun yaw(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 28, 4).toInt())
    }


    companion object {

        val meta = Meta(110, 0, 0, 1, 32, 1, 256)


        inline fun push(src: GLOBAL_VISION_POSITION_ESTIMATE, usec: (src: Long) -> Unit, x: (src: Float) -> Unit, y: (src: Float) -> Unit, z: (src: Float) -> Unit, roll: (src: Float) -> Unit, pitch: (src: Float) -> Unit, yaw: (src: Float) -> Unit
        ) {

            usec(src.usec())

            x(src.x())

            y(src.y())

            z(src.z())

            roll(src.roll())

            pitch(src.pitch())

            yaw(src.yaw())

        }

    }
}

inline class FILE_TRANSFER_PROTOCOL(val data: Pack.Bytes) {

    inline fun target_network(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_network_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun payload(): FILE_TRANSFER_PROTOCOL_payload {

        return FILE_TRANSFER_PROTOCOL_payload(data)
    }

    inline fun payload_(src: ByteArray) {
        val len = minOf(src.size, 251)

        for (index in 0 until len)
            data.bytes[data.origin + 3 + index] = (src[index]).toByte()
    }

    object payload {
        const val item_len = 251


    }


    companion object {

        val meta = Meta(27, 0, 0, 0, 254, 1, 2032)


        inline fun push(src: FILE_TRANSFER_PROTOCOL, target_network: (src: Byte) -> Unit, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, payload: (src: FILE_TRANSFER_PROTOCOL_payload) -> Unit
        ) {

            target_network(src.target_network())

            target_system(src.target_system())

            target_component(src.target_component())
            src.payload().let { item ->
                payload(item)
            }

        }

        inline fun pull(dst: FILE_TRANSFER_PROTOCOL, target_network: () -> Byte, target_system: () -> Byte, target_component: () -> Byte, payload: (dst: FILE_TRANSFER_PROTOCOL_payload) -> Unit
        ) {

            dst.target_network(target_network())

            dst.target_system(target_system())

            dst.target_component(target_component())
            payload(dst.payload())

        }

    }
}

inline class RANGEFINDER(val data: Pack.Bytes) {

    inline fun distance(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 0, 4).toInt())
    }

    inline fun distance_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 0)
    }

    inline fun voltage(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun voltage_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }


    companion object {

        val meta = Meta(153, 0, 0, 0, 8, 1, 64)


        inline fun push(src: RANGEFINDER, distance: (src: Float) -> Unit, voltage: (src: Float) -> Unit
        ) {

            distance(src.distance())

            voltage(src.voltage())

        }

        inline fun pull(dst: RANGEFINDER, distance: () -> Float, voltage: () -> Float
        ) {

            dst.distance(distance())

            dst.voltage(voltage())

        }

    }
}

inline class RADIO_STATUS(val data: Pack.Bytes) {

    inline fun rxerrors(): Short {
        return (get_bytes(data.bytes, data.origin + 0, 2)).toShort()
    }

    inline fun rxerrors_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 0)
    }

    inline fun fixeD(): Short {
        return (get_bytes(data.bytes, data.origin + 2, 2)).toShort()
    }

    inline fun fixeD_(src: Short) {
        set_bytes((src).toULong().toLong(), 2, data.bytes, data.origin + 2)
    }

    inline fun rssi(): Byte {
        return (data.bytes[data.origin + 4]).toByte()
    }

    inline fun rssi_(src: Byte) {
        data.bytes[data.origin + 4] = (src).toByte()
    }

    inline fun remrssi(): Byte {
        return (data.bytes[data.origin + 5]).toByte()
    }

    inline fun remrssi_(src: Byte) {
        data.bytes[data.origin + 5] = (src).toByte()
    }

    inline fun txbuf(): Byte {
        return (data.bytes[data.origin + 6]).toByte()
    }

    inline fun txbuf_(src: Byte) {
        data.bytes[data.origin + 6] = (src).toByte()
    }

    inline fun noise(): Byte {
        return (data.bytes[data.origin + 7]).toByte()
    }

    inline fun noise_(src: Byte) {
        data.bytes[data.origin + 7] = (src).toByte()
    }

    inline fun remnoise(): Byte {
        return (data.bytes[data.origin + 8]).toByte()
    }

    inline fun remnoise_(src: Byte) {
        data.bytes[data.origin + 8] = (src).toByte()
    }


    companion object {

        val meta = Meta(107, 2, 0, 0, 9, 1, 72)


        inline fun push(src: RADIO_STATUS, rssi: (src: Byte) -> Unit, remrssi: (src: Byte) -> Unit, txbuf: (src: Byte) -> Unit, noise: (src: Byte) -> Unit, remnoise: (src: Byte) -> Unit, rxerrors: (src: Short) -> Unit, fixeD: (src: Short) -> Unit
        ) {

            rssi(src.rssi())

            remrssi(src.remrssi())

            txbuf(src.txbuf())

            noise(src.noise())

            remnoise(src.remnoise())

            rxerrors(src.rxerrors())

            fixeD(src.fixeD())

        }

        inline fun pull(dst: RADIO_STATUS, rssi: () -> Byte, remrssi: () -> Byte, txbuf: () -> Byte, noise: () -> Byte, remnoise: () -> Byte, rxerrors: () -> Short, fixeD: () -> Short
        ) {

            dst.rssi(rssi())

            dst.remrssi(remrssi())

            dst.txbuf(txbuf())

            dst.noise(noise())

            dst.remnoise(remnoise())

            dst.rxerrors(rxerrors())

            dst.fixeD(fixeD())

        }

    }
}

inline class FENCE_POINT(val data: Pack.Bytes) {

    inline fun target_system(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun target_system_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun target_component(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun target_component_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun idx(): Byte {
        return (data.bytes[data.origin + 2]).toByte()
    }

    inline fun idx_(src: Byte) {
        data.bytes[data.origin + 2] = (src).toByte()
    }

    inline fun count(): Byte {
        return (data.bytes[data.origin + 3]).toByte()
    }

    inline fun count_(src: Byte) {
        data.bytes[data.origin + 3] = (src).toByte()
    }

    inline fun lat(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 4, 4).toInt())
    }

    inline fun lat_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 4)
    }

    inline fun lng(): Float {
        return Float.fromBits(get_bytes(data.bytes, data.origin + 8, 4).toInt())
    }

    inline fun lng_(src: Float) {
        set_bytes(java.lang.Float.floatToIntBits(src).toULong().toLong(), 4, data.bytes, data.origin + 8)
    }


    companion object {

        val meta = Meta(20, 0, 0, 0, 12, 1, 96)


        inline fun push(src: FENCE_POINT, target_system: (src: Byte) -> Unit, target_component: (src: Byte) -> Unit, idx: (src: Byte) -> Unit, count: (src: Byte) -> Unit, lat: (src: Float) -> Unit, lng: (src: Float) -> Unit
        ) {

            target_system(src.target_system())

            target_component(src.target_component())

            idx(src.idx())

            count(src.count())

            lat(src.lat())

            lng(src.lng())

        }

        inline fun pull(dst: FENCE_POINT, target_system: () -> Byte, target_component: () -> Byte, idx: () -> Byte, count: () -> Byte, lat: () -> Float, lng: () -> Float
        ) {

            dst.target_system(target_system())

            dst.target_component(target_component())

            dst.idx(idx())

            dst.count(count())

            dst.lat(lat())

            dst.lng(lng())

        }

    }
}

inline class RESOURCE_REQUEST(val data: Pack.Bytes) {

    inline fun request_id(): Byte {
        return (data.bytes[data.origin + 0]).toByte()
    }

    inline fun request_id_(src: Byte) {
        data.bytes[data.origin + 0] = (src).toByte()
    }

    inline fun uri_type(): Byte {
        return (data.bytes[data.origin + 1]).toByte()
    }

    inline fun uri_type_(src: Byte) {
        data.bytes[data.origin + 1] = (src).toByte()
    }

    inline fun uri(): RESOURCE_REQUEST_uri {

        return RESOURCE_REQUEST_uri(data)
    }

    inline fun uri_(src: ByteArray) {
        val len = minOf(src.size, 120)

        for (index in 0 until len)
            data.bytes[data.origin + 2 + index] = (src[index]).toByte()
    }

    object uri {
        const val item_len = 120


    }

    inline fun transfer_type(): Byte {
        return (data.bytes[data.origin + 122]).toByte()
    }

    inline fun transfer_type_(src: Byte) {
        data.bytes[data.origin + 122] = (src).toByte()
    }

    inline fun storage(): RESOURCE_REQUEST_storage {

        return RESOURCE_REQUEST_storage(data)
    }

    inline fun storage_(src: ByteArray) {
        val len = minOf(src.size, 120)

        for (index in 0 until len)
            data.bytes[data.origin + 123 + index] = (src[index]).toByte()
    }

    object storage {
        const val item_len = 120


    }


    companion object {

        val meta = Meta(67, 0, 0, 0, 243, 1, 1944)


        inline fun push(src: RESOURCE_REQUEST, request_id: (src: Byte) -> Unit, uri_type: (src: Byte) -> Unit, uri: (src: RESOURCE_REQUEST_uri) -> Unit, transfer_type: (src: Byte) -> Unit, storage: (src: RESOURCE_REQUEST_storage) -> Unit
        ) {

            request_id(src.request_id())

            uri_type(src.uri_type())
            src.uri().let { item ->
                uri(item)
            }

            transfer_type(src.transfer_type())
            src.storage().let { item ->
                storage(item)
            }

        }

        inline fun pull(dst: RESOURCE_REQUEST, request_id: () -> Byte, uri_type: () -> Byte, uri: (dst: RESOURCE_REQUEST_uri) -> Unit, transfer_type: () -> Byte, storage: (dst: RESOURCE_REQUEST_storage) -> Unit
        ) {

            dst.request_id(request_id())

            dst.uri_type(uri_type())
            uri(dst.uri())

            dst.transfer_type(transfer_type())
            storage(dst.storage())

        }

    }
}


abstract class CommunicationChannel {

    abstract fun pushSendingPack(pack: Pack): Boolean
    abstract fun pullSendingPack(): Pack?

    val transmitter: java.io.InputStream = object : Channel.Transmitter(1, 1) {
        override fun pullSendingPack(): Pack? {
            return this@CommunicationChannel.pullSendingPack()
        }
    }

    fun send(src: FOLLOW_TARGET): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: ADSB_VEHICLE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MESSAGE_INTERVAL): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: EKF_STATUS_REPORT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: ESTIMATOR_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: HWSTATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: TIMESYNC): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PARAM_EXT_REQUEST_LIST): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: BUTTON_CHANGE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: UAVCAN_NODE_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: COLLISION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GIMBAL_TORQUE_CMD_REPORT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: ALTITUDE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: HIL_STATE_QUATERNION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SENSOR_OFFSETS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: STORAGE_INFORMATION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: CAMERA_INFORMATION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DEVICE_OP_WRITE_REPLY): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: TERRAIN_DATA): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GIMBAL_CONTROL): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: VIDEO_STREAM_INFORMATION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: AHRS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DEBUG): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: CAMERA_IMAGE_CAPTURED): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOG_ENTRY): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: ACTUATOR_CONTROL_TARGET): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: HIGH_LATENCY): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: HOME_POSITION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: FENCE_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: REMOTE_LOG_BLOCK_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: OBSTACLE_DISTANCE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GPS2_RAW): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MEMORY_VECT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PARAM_EXT_REQUEST_READ): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: HIL_SENSOR): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SETUP_SIGNING): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GPS_RTK): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: UAVIONIX_ADSB_OUT_CFG): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LANDING_TARGET): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SET_ACTUATOR_CONTROL_TARGET): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: CONTROL_SYSTEM_STATE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DATA32): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PING33): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: RALLY_POINT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: ADAP_TUNING): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: VIBRATION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PARAM_EXT_VALUE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: BATTERY2): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LIMITS_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: CAMERA_FEEDBACK): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: HIL_GPS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: FENCE_FETCH_POINT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: RADIO): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: AIRSPEED_AUTOCAL): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: ATT_POS_MOCAP): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: STATUSTEXT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GOPRO_GET_REQUEST): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: CAMERA_CAPTURE_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: ENCAPSULATED_DATA): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GPS_INPUT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: COMPASSMOT_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOG_REQUEST_DATA): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: CAMERA_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: CAMERA_SETTINGS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DEVICE_OP_READ_REPLY): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DIGICAM_CONTROL): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: NAMED_VALUE_FLOAT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GOPRO_HEARTBEAT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: AHRS2): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOG_ERASE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: TERRAIN_REQUEST): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MOUNT_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PID_TUNING): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: OPTICAL_FLOW_RAD): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOG_DATA): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: AHRS3): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: VICON_POSITION_ESTIMATE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GPS2_RTK): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MAG_CAL_REPORT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOG_REQUEST_LIST): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MOUNT_CONFIGURE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: V2_EXTENSION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: POWER_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: REMOTE_LOG_DATA_BLOCK): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOGGING_DATA_ACKED): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: TERRAIN_CHECK): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: TERRAIN_REPORT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SET_HOME_POSITION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun sendSwitchModeCommand(): Boolean {
        return pushSendingPack(SwitchModeCommand)
    }

    fun send(src: SCALED_IMU3): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MOUNT_CONTROL): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LED_CONTROL): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SIM_STATE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: WIFI_CONFIG_AP): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DATA96): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: FLIGHT_INFORMATION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MEMINFO): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOGGING_ACK): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: VISION_SPEED_ESTIMATE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DEBUG_VECT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: CAMERA_TRIGGER): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOG_REQUEST_END): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GOPRO_SET_RESPONSE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PROTOCOL_VERSION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: RALLY_FETCH_POINT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: BATTERY_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MOUNT_ORIENTATION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SERIAL_CONTROL): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PARAM_EXT_SET): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: AUTOPILOT_VERSION): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SIMSTATE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SET_VIDEO_STREAM_SETTINGS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PLAY_TUNE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DIGICAM_CONFIGURE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SCALED_PRESSURE3): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: PARAM_EXT_ACK): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: UAVCAN_NODE_INFO): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DATA16): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SET_MAG_OFFSETS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SCALED_IMU2): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: AP_ADC): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: WIND): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: AUTOPILOT_VERSION_REQUEST): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DATA_TRANSMISSION_HANDSHAKE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DATA64): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GIMBAL_REPORT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DEVICE_OP_WRITE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DISTANCE_SENSOR): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: HIL_OPTICAL_FLOW): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: SCALED_PRESSURE2): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: WIND_COV): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GOPRO_SET_REQUEST): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: VISION_POSITION_DELTA): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: LOGGING_DATA): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: DEVICE_OP_READ): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: MAG_CAL_PROGRESS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: HIGHRES_IMU): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: EXTENDED_SYS_STATE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: UAVIONIX_ADSB_OUT_DYNAMIC): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GOPRO_GET_RESPONSE): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GPS_INJECT_DATA): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: NAMED_VALUE_INT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: RPM): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: GPS_RTCM_DATA): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: FILE_TRANSFER_PROTOCOL): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: RANGEFINDER): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: RADIO_STATUS): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: FENCE_POINT): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }

    fun send(src: RESOURCE_REQUEST): Boolean {
        if (src.data.bytes == null) return false
        val pack = src.data.unwrap()
        if (pushSendingPack(pack)) return true
        src.data.wrap(pack)
        return false
    }


    object NEW {
        /**
         *current motion information from a designated system */
        fun FOLLOW_TARGET(bytes: Pack.Bytes): FOLLOW_TARGET {
            bytes.init(FOLLOW_TARGET.meta)
            return com.company.demo.GroundControl.FOLLOW_TARGET(bytes)
        }

        /**
         *The location and information of an ADSB vehicle */
        fun ADSB_VEHICLE(cur: Cursor): ADSB_VEHICLE {
            cur.init(ADSB_VEHICLE.meta)
            return com.company.demo.GroundControl.ADSB_VEHICLE(cur)
        }

        /**
         *This interface replaces DATA_STREAM */
        fun MESSAGE_INTERVAL(bytes: Pack.Bytes): MESSAGE_INTERVAL {
            bytes.init(MESSAGE_INTERVAL.meta)
            return com.company.demo.GroundControl.MESSAGE_INTERVAL(bytes)
        }

        /**
         *EKF Status message including flags and variances */
        fun EKF_STATUS_REPORT(cur: Cursor): EKF_STATUS_REPORT {
            cur.init(EKF_STATUS_REPORT.meta)
            return com.company.demo.GroundControl.EKF_STATUS_REPORT(cur)
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
        fun ESTIMATOR_STATUS(cur: Cursor): ESTIMATOR_STATUS {
            cur.init(ESTIMATOR_STATUS.meta)
            return com.company.demo.GroundControl.ESTIMATOR_STATUS(cur)
        }

        /**
         *Status of key hardware */
        fun HWSTATUS(bytes: Pack.Bytes): HWSTATUS {
            bytes.init(HWSTATUS.meta)
            return com.company.demo.GroundControl.HWSTATUS(bytes)
        }

        /**
         *Time synchronization message. */
        fun TIMESYNC(bytes: Pack.Bytes): TIMESYNC {
            bytes.init(TIMESYNC.meta)
            return com.company.demo.GroundControl.TIMESYNC(bytes)
        }

        /**
         *Request all parameters of this component. After this request, all parameters are emitted. */
        fun PARAM_EXT_REQUEST_LIST(bytes: Pack.Bytes): PARAM_EXT_REQUEST_LIST {
            bytes.init(PARAM_EXT_REQUEST_LIST.meta)
            return com.company.demo.GroundControl.PARAM_EXT_REQUEST_LIST(bytes)
        }

        /**
         *Report button state change */
        fun BUTTON_CHANGE(bytes: Pack.Bytes): BUTTON_CHANGE {
            bytes.init(BUTTON_CHANGE.meta)
            return com.company.demo.GroundControl.BUTTON_CHANGE(bytes)
        }

        /**
         *General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
         *				 for the background information. The UAVCAN specification is available at http:uavcan.org */
        fun UAVCAN_NODE_STATUS(cur: Cursor): UAVCAN_NODE_STATUS {
            cur.init(UAVCAN_NODE_STATUS.meta)
            return com.company.demo.GroundControl.UAVCAN_NODE_STATUS(cur)
        }

        /**
         *Information about a potential collision */
        fun COLLISION(cur: Cursor): COLLISION {
            cur.init(COLLISION.meta)
            return com.company.demo.GroundControl.COLLISION(cur)
        }

        /**
         *100 Hz gimbal torque command telemetry */
        fun GIMBAL_TORQUE_CMD_REPORT(bytes: Pack.Bytes): GIMBAL_TORQUE_CMD_REPORT {
            bytes.init(GIMBAL_TORQUE_CMD_REPORT.meta)
            return com.company.demo.GroundControl.GIMBAL_TORQUE_CMD_REPORT(bytes)
        }

        /**
         *The current system altitude. */
        fun ALTITUDE(bytes: Pack.Bytes): ALTITUDE {
            bytes.init(ALTITUDE.meta)
            return com.company.demo.GroundControl.ALTITUDE(bytes)
        }

        /**
         *Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
         *				 for high throughput applications such as hardware in the loop simulations */
        fun HIL_STATE_QUATERNION(bytes: Pack.Bytes): HIL_STATE_QUATERNION {
            bytes.init(HIL_STATE_QUATERNION.meta)
            return com.company.demo.GroundControl.HIL_STATE_QUATERNION(bytes)
        }

        /**
         *Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process */
        fun SENSOR_OFFSETS(bytes: Pack.Bytes): SENSOR_OFFSETS {
            bytes.init(SENSOR_OFFSETS.meta)
            return com.company.demo.GroundControl.SENSOR_OFFSETS(bytes)
        }

        /**
         *WIP: Information about a storage medium. */
        fun STORAGE_INFORMATION(bytes: Pack.Bytes): STORAGE_INFORMATION {
            bytes.init(STORAGE_INFORMATION.meta)
            return com.company.demo.GroundControl.STORAGE_INFORMATION(bytes)
        }

        /**
         *WIP: Information about a camera */
        fun CAMERA_INFORMATION(cur: Cursor): CAMERA_INFORMATION {
            cur.init(CAMERA_INFORMATION.meta)
            return com.company.demo.GroundControl.CAMERA_INFORMATION(cur)
        }

        /**
         *Write registers reply */
        fun DEVICE_OP_WRITE_REPLY(bytes: Pack.Bytes): DEVICE_OP_WRITE_REPLY {
            bytes.init(DEVICE_OP_WRITE_REPLY.meta)
            return com.company.demo.GroundControl.DEVICE_OP_WRITE_REPLY(bytes)
        }

        /**
         *Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES */
        fun TERRAIN_DATA(bytes: Pack.Bytes): TERRAIN_DATA {
            bytes.init(TERRAIN_DATA.meta)
            return com.company.demo.GroundControl.TERRAIN_DATA(bytes)
        }

        /**
         *Control message for rate gimbal */
        fun GIMBAL_CONTROL(bytes: Pack.Bytes): GIMBAL_CONTROL {
            bytes.init(GIMBAL_CONTROL.meta)
            return com.company.demo.GroundControl.GIMBAL_CONTROL(bytes)
        }

        /**
         *WIP: Information about video stream */
        fun VIDEO_STREAM_INFORMATION(cur: Cursor): VIDEO_STREAM_INFORMATION {
            cur.init(VIDEO_STREAM_INFORMATION.meta)
            return com.company.demo.GroundControl.VIDEO_STREAM_INFORMATION(cur)
        }

        /**
         *Status of DCM attitude estimator */
        fun AHRS(bytes: Pack.Bytes): AHRS {
            bytes.init(AHRS.meta)
            return com.company.demo.GroundControl.AHRS(bytes)
        }

        /**
         *Send a debug value. The index is used to discriminate between values. These values show up in the plot
         *				 of QGroundControl as DEBUG N */
        fun DEBUG(bytes: Pack.Bytes): DEBUG {
            bytes.init(DEBUG.meta)
            return com.company.demo.GroundControl.DEBUG(bytes)
        }

        /**
         *Information about a captured image */
        fun CAMERA_IMAGE_CAPTURED(cur: Cursor): CAMERA_IMAGE_CAPTURED {
            cur.init(CAMERA_IMAGE_CAPTURED.meta)
            return com.company.demo.GroundControl.CAMERA_IMAGE_CAPTURED(cur)
        }

        /**
         *Reply to LOG_REQUEST_LIST */
        fun LOG_ENTRY(bytes: Pack.Bytes): LOG_ENTRY {
            bytes.init(LOG_ENTRY.meta)
            return com.company.demo.GroundControl.LOG_ENTRY(bytes)
        }

        /**
         *Set the vehicle attitude and body angular rates. */
        fun ACTUATOR_CONTROL_TARGET(bytes: Pack.Bytes): ACTUATOR_CONTROL_TARGET {
            bytes.init(ACTUATOR_CONTROL_TARGET.meta)
            return com.company.demo.GroundControl.ACTUATOR_CONTROL_TARGET(bytes)
        }

        /**
         *Message appropriate for high latency connections like Iridium */
        fun HIGH_LATENCY(cur: Cursor): HIGH_LATENCY {
            cur.init(HIGH_LATENCY.meta)
            return com.company.demo.GroundControl.HIGH_LATENCY(cur)
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
        fun HOME_POSITION(cur: Cursor): HOME_POSITION {
            cur.init(HOME_POSITION.meta)
            return com.company.demo.GroundControl.HOME_POSITION(cur)
        }

        /**
         *Status of geo-fencing. Sent in extended status stream when fencing enabled */
        fun FENCE_STATUS(cur: Cursor): FENCE_STATUS {
            cur.init(FENCE_STATUS.meta)
            return com.company.demo.GroundControl.FENCE_STATUS(cur)
        }

        /**
         *Send Status of each log block that autopilot board might have sent */
        fun REMOTE_LOG_BLOCK_STATUS(cur: Cursor): REMOTE_LOG_BLOCK_STATUS {
            cur.init(REMOTE_LOG_BLOCK_STATUS.meta)
            return com.company.demo.GroundControl.REMOTE_LOG_BLOCK_STATUS(cur)
        }

        /**
         *Obstacle distances in front of the sensor, starting from the left in increment degrees to the right */
        fun OBSTACLE_DISTANCE(cur: Cursor): OBSTACLE_DISTANCE {
            cur.init(OBSTACLE_DISTANCE.meta)
            return com.company.demo.GroundControl.OBSTACLE_DISTANCE(cur)
        }

        /**
         *Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame). */
        fun GPS2_RAW(cur: Cursor): GPS2_RAW {
            cur.init(GPS2_RAW.meta)
            return com.company.demo.GroundControl.GPS2_RAW(cur)
        }

        /**
         *Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
         *				 way for testing new messages and getting experimental debug output */
        fun MEMORY_VECT(bytes: Pack.Bytes): MEMORY_VECT {
            bytes.init(MEMORY_VECT.meta)
            return com.company.demo.GroundControl.MEMORY_VECT(bytes)
        }

        /**
         *Request to read the value of a parameter with the either the param_id string id or param_index. */
        fun PARAM_EXT_REQUEST_READ(cur: Cursor): PARAM_EXT_REQUEST_READ {
            cur.init(PARAM_EXT_REQUEST_READ.meta)
            return com.company.demo.GroundControl.PARAM_EXT_REQUEST_READ(cur)
        }

        /**
         *The IMU readings in SI units in NED body frame */
        fun HIL_SENSOR(bytes: Pack.Bytes): HIL_SENSOR {
            bytes.init(HIL_SENSOR.meta)
            return com.company.demo.GroundControl.HIL_SENSOR(bytes)
        }

        /**
         *Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
         *				 signin */
        fun SETUP_SIGNING(bytes: Pack.Bytes): SETUP_SIGNING {
            bytes.init(SETUP_SIGNING.meta)
            return com.company.demo.GroundControl.SETUP_SIGNING(bytes)
        }

        /**
         *RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting */
        fun GPS_RTK(bytes: Pack.Bytes): GPS_RTK {
            bytes.init(GPS_RTK.meta)
            return com.company.demo.GroundControl.GPS_RTK(bytes)
        }

        /**
         *Static data to configure the ADS-B transponder (send within 10 sec of a POR and every 10 sec thereafter */
        fun UAVIONIX_ADSB_OUT_CFG(cur: Cursor): UAVIONIX_ADSB_OUT_CFG {
            cur.init(UAVIONIX_ADSB_OUT_CFG.meta)
            return com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_CFG(cur)
        }

        /**
         *The location of a landing area captured from a downward facing camera */
        fun LANDING_TARGET(cur: Cursor): LANDING_TARGET {
            cur.init(LANDING_TARGET.meta)
            return com.company.demo.GroundControl.LANDING_TARGET(cur)
        }

        /**
         *Set the vehicle attitude and body angular rates. */
        fun SET_ACTUATOR_CONTROL_TARGET(bytes: Pack.Bytes): SET_ACTUATOR_CONTROL_TARGET {
            bytes.init(SET_ACTUATOR_CONTROL_TARGET.meta)
            return com.company.demo.GroundControl.SET_ACTUATOR_CONTROL_TARGET(bytes)
        }

        /**
         *The smoothed, monotonic system state used to feed the control loops of the system. */
        fun CONTROL_SYSTEM_STATE(bytes: Pack.Bytes): CONTROL_SYSTEM_STATE {
            bytes.init(CONTROL_SYSTEM_STATE.meta)
            return com.company.demo.GroundControl.CONTROL_SYSTEM_STATE(bytes)
        }

        /**
         *Data packet, size 32 */
        fun DATA32(bytes: Pack.Bytes): DATA32 {
            bytes.init(DATA32.meta)
            return com.company.demo.GroundControl.DATA32(bytes)
        }

        fun PING33(cur: Cursor): PING33    //switch blink mode user command bytes
        {
            cur.init(PING33.meta)
            return com.company.demo.GroundControl.PING33(cur)
        }

        /**
         *GCS */
        fun RALLY_POINT(cur: Cursor): RALLY_POINT {
            cur.init(RALLY_POINT.meta)
            return com.company.demo.GroundControl.RALLY_POINT(cur)
        }

        /**
         *Adaptive Controller tuning information */
        fun ADAP_TUNING(cur: Cursor): ADAP_TUNING {
            cur.init(ADAP_TUNING.meta)
            return com.company.demo.GroundControl.ADAP_TUNING(cur)
        }

        /**
         *Vibration levels and accelerometer clipping */
        fun VIBRATION(bytes: Pack.Bytes): VIBRATION {
            bytes.init(VIBRATION.meta)
            return com.company.demo.GroundControl.VIBRATION(bytes)
        }

        /**
         *Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
         *				 recipient to keep track of received parameters and allows them to re-request missing parameters after
         *				 a loss or timeout */
        fun PARAM_EXT_VALUE(cur: Cursor): PARAM_EXT_VALUE {
            cur.init(PARAM_EXT_VALUE.meta)
            return com.company.demo.GroundControl.PARAM_EXT_VALUE(cur)
        }

        /**
         *2nd Battery status */
        fun BATTERY2(bytes: Pack.Bytes): BATTERY2 {
            bytes.init(BATTERY2.meta)
            return com.company.demo.GroundControl.BATTERY2(bytes)
        }

        /**
         *Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled */
        fun LIMITS_STATUS(cur: Cursor): LIMITS_STATUS {
            cur.init(LIMITS_STATUS.meta)
            return com.company.demo.GroundControl.LIMITS_STATUS(cur)
        }

        /**
         *Camera Capture Feedback */
        fun CAMERA_FEEDBACK(cur: Cursor): CAMERA_FEEDBACK {
            cur.init(CAMERA_FEEDBACK.meta)
            return com.company.demo.GroundControl.CAMERA_FEEDBACK(cur)
        }

        /**
         *The global position, as returned by the Global Positioning System (GPS). This is
         *				 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame). */
        fun HIL_GPS(bytes: Pack.Bytes): HIL_GPS {
            bytes.init(HIL_GPS.meta)
            return com.company.demo.GroundControl.HIL_GPS(bytes)
        }

        /**
         *Request a current fence point from MAV */
        fun FENCE_FETCH_POINT(bytes: Pack.Bytes): FENCE_FETCH_POINT {
            bytes.init(FENCE_FETCH_POINT.meta)
            return com.company.demo.GroundControl.FENCE_FETCH_POINT(bytes)
        }

        /**
         *Status generated by radio */
        fun RADIO(bytes: Pack.Bytes): RADIO {
            bytes.init(RADIO.meta)
            return com.company.demo.GroundControl.RADIO(bytes)
        }

        /**
         *Airspeed auto-calibration */
        fun AIRSPEED_AUTOCAL(bytes: Pack.Bytes): AIRSPEED_AUTOCAL {
            bytes.init(AIRSPEED_AUTOCAL.meta)
            return com.company.demo.GroundControl.AIRSPEED_AUTOCAL(bytes)
        }

        /**
         *Motion capture attitude and position */
        fun ATT_POS_MOCAP(bytes: Pack.Bytes): ATT_POS_MOCAP {
            bytes.init(ATT_POS_MOCAP.meta)
            return com.company.demo.GroundControl.ATT_POS_MOCAP(bytes)
        }

        /**
         *Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
         *				 They consume quite some bandwidth, so use only for important status and error messages. If implemented
         *				 wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz) */
        fun STATUSTEXT(cur: Cursor): STATUSTEXT {
            cur.init(STATUSTEXT.meta)
            return com.company.demo.GroundControl.STATUSTEXT(cur)
        }

        /**
         *Request a GOPRO_COMMAND response from the GoPro */
        fun GOPRO_GET_REQUEST(cur: Cursor): GOPRO_GET_REQUEST {
            cur.init(GOPRO_GET_REQUEST.meta)
            return com.company.demo.GroundControl.GOPRO_GET_REQUEST(cur)
        }

        /**
         *WIP: Information about the status of a capture */
        fun CAMERA_CAPTURE_STATUS(bytes: Pack.Bytes): CAMERA_CAPTURE_STATUS {
            bytes.init(CAMERA_CAPTURE_STATUS.meta)
            return com.company.demo.GroundControl.CAMERA_CAPTURE_STATUS(bytes)
        }

        fun ENCAPSULATED_DATA(bytes: Pack.Bytes): ENCAPSULATED_DATA {
            bytes.init(ENCAPSULATED_DATA.meta)
            return com.company.demo.GroundControl.ENCAPSULATED_DATA(bytes)
        }

        /**
         *GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
         *				 estimate of the sytem */
        fun GPS_INPUT(cur: Cursor): GPS_INPUT {
            cur.init(GPS_INPUT.meta)
            return com.company.demo.GroundControl.GPS_INPUT(cur)
        }

        /**
         *Status of compassmot calibration */
        fun COMPASSMOT_STATUS(bytes: Pack.Bytes): COMPASSMOT_STATUS {
            bytes.init(COMPASSMOT_STATUS.meta)
            return com.company.demo.GroundControl.COMPASSMOT_STATUS(bytes)
        }

        /**
         *Request a chunk of a log */
        fun LOG_REQUEST_DATA(bytes: Pack.Bytes): LOG_REQUEST_DATA {
            bytes.init(LOG_REQUEST_DATA.meta)
            return com.company.demo.GroundControl.LOG_REQUEST_DATA(bytes)
        }

        /**
         *Camera Event */
        fun CAMERA_STATUS(cur: Cursor): CAMERA_STATUS {
            cur.init(CAMERA_STATUS.meta)
            return com.company.demo.GroundControl.CAMERA_STATUS(cur)
        }

        /**
         *WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS. */
        fun CAMERA_SETTINGS(cur: Cursor): CAMERA_SETTINGS {
            cur.init(CAMERA_SETTINGS.meta)
            return com.company.demo.GroundControl.CAMERA_SETTINGS(cur)
        }

        /**
         *Read registers reply */
        fun DEVICE_OP_READ_REPLY(bytes: Pack.Bytes): DEVICE_OP_READ_REPLY {
            bytes.init(DEVICE_OP_READ_REPLY.meta)
            return com.company.demo.GroundControl.DEVICE_OP_READ_REPLY(bytes)
        }

        /**
         *Control on-board Camera Control System to take shots. */
        fun DIGICAM_CONTROL(bytes: Pack.Bytes): DIGICAM_CONTROL {
            bytes.init(DIGICAM_CONTROL.meta)
            return com.company.demo.GroundControl.DIGICAM_CONTROL(bytes)
        }

        /**
         *Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
         *				 efficient way for testing new messages and getting experimental debug output */
        fun NAMED_VALUE_FLOAT(cur: Cursor): NAMED_VALUE_FLOAT {
            cur.init(NAMED_VALUE_FLOAT.meta)
            return com.company.demo.GroundControl.NAMED_VALUE_FLOAT(cur)
        }

        /**
         *Heartbeat from a HeroBus attached GoPro */
        fun GOPRO_HEARTBEAT(cur: Cursor): GOPRO_HEARTBEAT {
            cur.init(GOPRO_HEARTBEAT.meta)
            return com.company.demo.GroundControl.GOPRO_HEARTBEAT(cur)
        }

        /**
         *Status of secondary AHRS filter if available */
        fun AHRS2(bytes: Pack.Bytes): AHRS2 {
            bytes.init(AHRS2.meta)
            return com.company.demo.GroundControl.AHRS2(bytes)
        }

        /**
         *Erase all logs */
        fun LOG_ERASE(bytes: Pack.Bytes): LOG_ERASE {
            bytes.init(LOG_ERASE.meta)
            return com.company.demo.GroundControl.LOG_ERASE(bytes)
        }

        /**
         *Request for terrain data and terrain status */
        fun TERRAIN_REQUEST(bytes: Pack.Bytes): TERRAIN_REQUEST {
            bytes.init(TERRAIN_REQUEST.meta)
            return com.company.demo.GroundControl.TERRAIN_REQUEST(bytes)
        }

        /**
         *Message with some status from APM to GCS about camera or antenna mount */
        fun MOUNT_STATUS(bytes: Pack.Bytes): MOUNT_STATUS {
            bytes.init(MOUNT_STATUS.meta)
            return com.company.demo.GroundControl.MOUNT_STATUS(bytes)
        }

        /**
         *PID tuning information */
        fun PID_TUNING(cur: Cursor): PID_TUNING {
            cur.init(PID_TUNING.meta)
            return com.company.demo.GroundControl.PID_TUNING(cur)
        }

        /**
         *Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor) */
        fun OPTICAL_FLOW_RAD(bytes: Pack.Bytes): OPTICAL_FLOW_RAD {
            bytes.init(OPTICAL_FLOW_RAD.meta)
            return com.company.demo.GroundControl.OPTICAL_FLOW_RAD(bytes)
        }

        /**
         *Reply to LOG_REQUEST_DATA */
        fun LOG_DATA(bytes: Pack.Bytes): LOG_DATA {
            bytes.init(LOG_DATA.meta)
            return com.company.demo.GroundControl.LOG_DATA(bytes)
        }

        /**
         *Status of third AHRS filter if available. This is for ANU research group (Ali and Sean) */
        fun AHRS3(bytes: Pack.Bytes): AHRS3 {
            bytes.init(AHRS3.meta)
            return com.company.demo.GroundControl.AHRS3(bytes)
        }

        fun VICON_POSITION_ESTIMATE(bytes: Pack.Bytes): VICON_POSITION_ESTIMATE {
            bytes.init(VICON_POSITION_ESTIMATE.meta)
            return com.company.demo.GroundControl.VICON_POSITION_ESTIMATE(bytes)
        }

        /**
         *RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting */
        fun GPS2_RTK(bytes: Pack.Bytes): GPS2_RTK {
            bytes.init(GPS2_RTK.meta)
            return com.company.demo.GroundControl.GPS2_RTK(bytes)
        }

        /**
         *Reports results of completed compass calibration. Sent until MAG_CAL_ACK received. */
        fun MAG_CAL_REPORT(cur: Cursor): MAG_CAL_REPORT {
            cur.init(MAG_CAL_REPORT.meta)
            return com.company.demo.GroundControl.MAG_CAL_REPORT(cur)
        }

        /**
         *Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
         *				 is called */
        fun LOG_REQUEST_LIST(bytes: Pack.Bytes): LOG_REQUEST_LIST {
            bytes.init(LOG_REQUEST_LIST.meta)
            return com.company.demo.GroundControl.LOG_REQUEST_LIST(bytes)
        }

        /**
         *Message to configure a camera mount, directional antenna, etc. */
        fun MOUNT_CONFIGURE(cur: Cursor): MOUNT_CONFIGURE {
            cur.init(MOUNT_CONFIGURE.meta)
            return com.company.demo.GroundControl.MOUNT_CONFIGURE(cur)
        }

        /**
         *Message implementing parts of the V2 payload specs in V1 frames for transitional support. */
        fun V2_EXTENSION(bytes: Pack.Bytes): V2_EXTENSION {
            bytes.init(V2_EXTENSION.meta)
            return com.company.demo.GroundControl.V2_EXTENSION(bytes)
        }

        /**
         *Power supply status */
        fun POWER_STATUS(cur: Cursor): POWER_STATUS {
            cur.init(POWER_STATUS.meta)
            return com.company.demo.GroundControl.POWER_STATUS(cur)
        }

        /**
         *Send a block of log data to remote location */
        fun REMOTE_LOG_DATA_BLOCK(cur: Cursor): REMOTE_LOG_DATA_BLOCK {
            cur.init(REMOTE_LOG_DATA_BLOCK.meta)
            return com.company.demo.GroundControl.REMOTE_LOG_DATA_BLOCK(cur)
        }

        /**
         *A message containing logged data which requires a LOGGING_ACK to be sent back */
        fun LOGGING_DATA_ACKED(bytes: Pack.Bytes): LOGGING_DATA_ACKED {
            bytes.init(LOGGING_DATA_ACKED.meta)
            return com.company.demo.GroundControl.LOGGING_DATA_ACKED(bytes)
        }

        /**
         *Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
         *				 has all terrain data needed for a mission */
        fun TERRAIN_CHECK(bytes: Pack.Bytes): TERRAIN_CHECK {
            bytes.init(TERRAIN_CHECK.meta)
            return com.company.demo.GroundControl.TERRAIN_CHECK(bytes)
        }

        /**
         *Response from a TERRAIN_CHECK request */
        fun TERRAIN_REPORT(bytes: Pack.Bytes): TERRAIN_REPORT {
            bytes.init(TERRAIN_REPORT.meta)
            return com.company.demo.GroundControl.TERRAIN_REPORT(bytes)
        }

        /**
         *The position the system will return to and land on. The position is set automatically by the system during
         *				 the takeoff in case it was not explicitely set by the operator before or after. The global and local
         *				 positions encode the position in the respective coordinate frames, while the q parameter encodes the
         *				 orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
         *				 can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
         *				 the system should fly in normal flight mode and then perform a landing sequence along the vector */
        fun SET_HOME_POSITION(cur: Cursor): SET_HOME_POSITION {
            cur.init(SET_HOME_POSITION.meta)
            return com.company.demo.GroundControl.SET_HOME_POSITION(cur)
        }

        /**
         *The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
         *				 unit */
        fun SCALED_IMU3(bytes: Pack.Bytes): SCALED_IMU3 {
            bytes.init(SCALED_IMU3.meta)
            return com.company.demo.GroundControl.SCALED_IMU3(bytes)
        }

        /**
         *Message to control a camera mount, directional antenna, etc. */
        fun MOUNT_CONTROL(bytes: Pack.Bytes): MOUNT_CONTROL {
            bytes.init(MOUNT_CONTROL.meta)
            return com.company.demo.GroundControl.MOUNT_CONTROL(bytes)
        }

        /**
         *Control vehicle LEDs */
        fun LED_CONTROL(bytes: Pack.Bytes): LED_CONTROL {
            bytes.init(LED_CONTROL.meta)
            return com.company.demo.GroundControl.LED_CONTROL(bytes)
        }

        /**
         *Status of simulation environment, if used */
        fun SIM_STATE(bytes: Pack.Bytes): SIM_STATE {
            bytes.init(SIM_STATE.meta)
            return com.company.demo.GroundControl.SIM_STATE(bytes)
        }

        /**
         *Configure AP SSID and Password. */
        fun WIFI_CONFIG_AP(cur: Cursor): WIFI_CONFIG_AP {
            cur.init(WIFI_CONFIG_AP.meta)
            return com.company.demo.GroundControl.WIFI_CONFIG_AP(cur)
        }

        /**
         *Data packet, size 96 */
        fun DATA96(bytes: Pack.Bytes): DATA96 {
            bytes.init(DATA96.meta)
            return com.company.demo.GroundControl.DATA96(bytes)
        }

        /**
         *WIP: Information about flight since last arming */
        fun FLIGHT_INFORMATION(bytes: Pack.Bytes): FLIGHT_INFORMATION {
            bytes.init(FLIGHT_INFORMATION.meta)
            return com.company.demo.GroundControl.FLIGHT_INFORMATION(bytes)
        }

        /**
         *state of APM memory */
        fun MEMINFO(cur: Cursor): MEMINFO {
            cur.init(MEMINFO.meta)
            return com.company.demo.GroundControl.MEMINFO(cur)
        }

        /**
         *An ack for a LOGGING_DATA_ACKED message */
        fun LOGGING_ACK(bytes: Pack.Bytes): LOGGING_ACK {
            bytes.init(LOGGING_ACK.meta)
            return com.company.demo.GroundControl.LOGGING_ACK(bytes)
        }

        fun VISION_SPEED_ESTIMATE(bytes: Pack.Bytes): VISION_SPEED_ESTIMATE {
            bytes.init(VISION_SPEED_ESTIMATE.meta)
            return com.company.demo.GroundControl.VISION_SPEED_ESTIMATE(bytes)
        }

        fun DEBUG_VECT(cur: Cursor): DEBUG_VECT {
            cur.init(DEBUG_VECT.meta)
            return com.company.demo.GroundControl.DEBUG_VECT(cur)
        }

        /**
         *Camera-IMU triggering and synchronisation message. */
        fun CAMERA_TRIGGER(bytes: Pack.Bytes): CAMERA_TRIGGER {
            bytes.init(CAMERA_TRIGGER.meta)
            return com.company.demo.GroundControl.CAMERA_TRIGGER(bytes)
        }

        /**
         *Stop log transfer and resume normal logging */
        fun LOG_REQUEST_END(bytes: Pack.Bytes): LOG_REQUEST_END {
            bytes.init(LOG_REQUEST_END.meta)
            return com.company.demo.GroundControl.LOG_REQUEST_END(bytes)
        }

        /**
         *Response from a GOPRO_COMMAND set request */
        fun GOPRO_SET_RESPONSE(cur: Cursor): GOPRO_SET_RESPONSE {
            cur.init(GOPRO_SET_RESPONSE.meta)
            return com.company.demo.GroundControl.GOPRO_SET_RESPONSE(cur)
        }

        /**
         *WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION
         *				 and is used as part of the handshaking to establish which MAVLink version should be used on the network.
         *				 Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers
         *				 should consider adding this into the default decoding state machine to allow the protocol core to respond
         *				 directly */
        fun PROTOCOL_VERSION(bytes: Pack.Bytes): PROTOCOL_VERSION {
            bytes.init(PROTOCOL_VERSION.meta)
            return com.company.demo.GroundControl.PROTOCOL_VERSION(bytes)
        }

        /**
         *Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not
         *				 respond if the request is invalid */
        fun RALLY_FETCH_POINT(bytes: Pack.Bytes): RALLY_FETCH_POINT {
            bytes.init(RALLY_FETCH_POINT.meta)
            return com.company.demo.GroundControl.RALLY_FETCH_POINT(bytes)
        }

        /**
         *Battery information */
        fun BATTERY_STATUS(cur: Cursor): BATTERY_STATUS {
            cur.init(BATTERY_STATUS.meta)
            return com.company.demo.GroundControl.BATTERY_STATUS(cur)
        }

        fun MOUNT_ORIENTATION(bytes: Pack.Bytes): MOUNT_ORIENTATION {
            bytes.init(MOUNT_ORIENTATION.meta)
            return com.company.demo.GroundControl.MOUNT_ORIENTATION(bytes)
        }

        /**
         *Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
         *				 telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
         *				 or change the devices settings. A message with zero bytes can be used to change just the baudrate */
        fun SERIAL_CONTROL(cur: Cursor): SERIAL_CONTROL {
            cur.init(SERIAL_CONTROL.meta)
            return com.company.demo.GroundControl.SERIAL_CONTROL(cur)
        }

        /**
         *Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
         *				 setting a parameter value and the new value is the same as the current value, you will immediately get
         *				 a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
         *				 a PARAM_ACK_IN_PROGRESS in response */
        fun PARAM_EXT_SET(cur: Cursor): PARAM_EXT_SET {
            cur.init(PARAM_EXT_SET.meta)
            return com.company.demo.GroundControl.PARAM_EXT_SET(cur)
        }

        /**
         *Version and capability of autopilot software */
        fun AUTOPILOT_VERSION(cur: Cursor): AUTOPILOT_VERSION {
            cur.init(AUTOPILOT_VERSION.meta)
            return com.company.demo.GroundControl.AUTOPILOT_VERSION(cur)
        }

        /**
         *Status of simulation environment, if used */
        fun SIMSTATE(bytes: Pack.Bytes): SIMSTATE {
            bytes.init(SIMSTATE.meta)
            return com.company.demo.GroundControl.SIMSTATE(bytes)
        }

        /**
         *WIP: Message that sets video stream settings */
        fun SET_VIDEO_STREAM_SETTINGS(cur: Cursor): SET_VIDEO_STREAM_SETTINGS {
            cur.init(SET_VIDEO_STREAM_SETTINGS.meta)
            return com.company.demo.GroundControl.SET_VIDEO_STREAM_SETTINGS(cur)
        }

        /**
         *Control vehicle tone generation (buzzer) */
        fun PLAY_TUNE(cur: Cursor): PLAY_TUNE {
            cur.init(PLAY_TUNE.meta)
            return com.company.demo.GroundControl.PLAY_TUNE(cur)
        }

        /**
         *Configure on-board Camera Control System. */
        fun DIGICAM_CONFIGURE(bytes: Pack.Bytes): DIGICAM_CONFIGURE {
            bytes.init(DIGICAM_CONFIGURE.meta)
            return com.company.demo.GroundControl.DIGICAM_CONFIGURE(bytes)
        }

        /**
         *Barometer readings for 3rd barometer */
        fun SCALED_PRESSURE3(bytes: Pack.Bytes): SCALED_PRESSURE3 {
            bytes.init(SCALED_PRESSURE3.meta)
            return com.company.demo.GroundControl.SCALED_PRESSURE3(bytes)
        }

        /**
         *Response from a PARAM_EXT_SET message. */
        fun PARAM_EXT_ACK(cur: Cursor): PARAM_EXT_ACK {
            cur.init(PARAM_EXT_ACK.meta)
            return com.company.demo.GroundControl.PARAM_EXT_ACK(cur)
        }

        /**
         *General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
         *				 service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
         *				 by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
         *				 emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
         *				 is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
         *				 is available at http:uavcan.org */
        fun UAVCAN_NODE_INFO(cur: Cursor): UAVCAN_NODE_INFO {
            cur.init(UAVCAN_NODE_INFO.meta)
            return com.company.demo.GroundControl.UAVCAN_NODE_INFO(cur)
        }

        /**
         *Data packet, size 16 */
        fun DATA16(bytes: Pack.Bytes): DATA16 {
            bytes.init(DATA16.meta)
            return com.company.demo.GroundControl.DATA16(bytes)
        }

        /**
         *Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets */
        fun SET_MAG_OFFSETS(bytes: Pack.Bytes): SET_MAG_OFFSETS {
            bytes.init(SET_MAG_OFFSETS.meta)
            return com.company.demo.GroundControl.SET_MAG_OFFSETS(bytes)
        }

        /**
         *The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
         *				 the described unit */
        fun SCALED_IMU2(bytes: Pack.Bytes): SCALED_IMU2 {
            bytes.init(SCALED_IMU2.meta)
            return com.company.demo.GroundControl.SCALED_IMU2(bytes)
        }

        /**
         *raw ADC output */
        fun AP_ADC(bytes: Pack.Bytes): AP_ADC {
            bytes.init(AP_ADC.meta)
            return com.company.demo.GroundControl.AP_ADC(bytes)
        }

        /**
         *Wind estimation */
        fun WIND(bytes: Pack.Bytes): WIND {
            bytes.init(WIND.meta)
            return com.company.demo.GroundControl.WIND(bytes)
        }

        /**
         *Request the autopilot version from the system/component. */
        fun AUTOPILOT_VERSION_REQUEST(bytes: Pack.Bytes): AUTOPILOT_VERSION_REQUEST {
            bytes.init(AUTOPILOT_VERSION_REQUEST.meta)
            return com.company.demo.GroundControl.AUTOPILOT_VERSION_REQUEST(bytes)
        }

        fun DATA_TRANSMISSION_HANDSHAKE(bytes: Pack.Bytes): DATA_TRANSMISSION_HANDSHAKE {
            bytes.init(DATA_TRANSMISSION_HANDSHAKE.meta)
            return com.company.demo.GroundControl.DATA_TRANSMISSION_HANDSHAKE(bytes)
        }

        /**
         *Data packet, size 64 */
        fun DATA64(bytes: Pack.Bytes): DATA64 {
            bytes.init(DATA64.meta)
            return com.company.demo.GroundControl.DATA64(bytes)
        }

        /**
         *3 axis gimbal mesuraments */
        fun GIMBAL_REPORT(bytes: Pack.Bytes): GIMBAL_REPORT {
            bytes.init(GIMBAL_REPORT.meta)
            return com.company.demo.GroundControl.GIMBAL_REPORT(bytes)
        }

        /**
         *Write registers for a device */
        fun DEVICE_OP_WRITE(cur: Cursor): DEVICE_OP_WRITE {
            cur.init(DEVICE_OP_WRITE.meta)
            return com.company.demo.GroundControl.DEVICE_OP_WRITE(cur)
        }

        fun DISTANCE_SENSOR(cur: Cursor): DISTANCE_SENSOR {
            cur.init(DISTANCE_SENSOR.meta)
            return com.company.demo.GroundControl.DISTANCE_SENSOR(cur)
        }

        /**
         *Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor) */
        fun HIL_OPTICAL_FLOW(bytes: Pack.Bytes): HIL_OPTICAL_FLOW {
            bytes.init(HIL_OPTICAL_FLOW.meta)
            return com.company.demo.GroundControl.HIL_OPTICAL_FLOW(bytes)
        }

        /**
         *Barometer readings for 2nd barometer */
        fun SCALED_PRESSURE2(bytes: Pack.Bytes): SCALED_PRESSURE2 {
            bytes.init(SCALED_PRESSURE2.meta)
            return com.company.demo.GroundControl.SCALED_PRESSURE2(bytes)
        }

        fun WIND_COV(bytes: Pack.Bytes): WIND_COV {
            bytes.init(WIND_COV.meta)
            return com.company.demo.GroundControl.WIND_COV(bytes)
        }

        /**
         *Request to set a GOPRO_COMMAND with a desired */
        fun GOPRO_SET_REQUEST(cur: Cursor): GOPRO_SET_REQUEST {
            cur.init(GOPRO_SET_REQUEST.meta)
            return com.company.demo.GroundControl.GOPRO_SET_REQUEST(cur)
        }

        /**
         *camera vision based attitude and position deltas */
        fun VISION_POSITION_DELTA(bytes: Pack.Bytes): VISION_POSITION_DELTA {
            bytes.init(VISION_POSITION_DELTA.meta)
            return com.company.demo.GroundControl.VISION_POSITION_DELTA(bytes)
        }

        /**
         *A message containing logged data (see also MAV_CMD_LOGGING_START) */
        fun LOGGING_DATA(bytes: Pack.Bytes): LOGGING_DATA {
            bytes.init(LOGGING_DATA.meta)
            return com.company.demo.GroundControl.LOGGING_DATA(bytes)
        }

        /**
         *Read registers for a device */
        fun DEVICE_OP_READ(cur: Cursor): DEVICE_OP_READ {
            cur.init(DEVICE_OP_READ.meta)
            return com.company.demo.GroundControl.DEVICE_OP_READ(cur)
        }

        /**
         *Reports progress of compass calibration. */
        fun MAG_CAL_PROGRESS(cur: Cursor): MAG_CAL_PROGRESS {
            cur.init(MAG_CAL_PROGRESS.meta)
            return com.company.demo.GroundControl.MAG_CAL_PROGRESS(cur)
        }

        /**
         *The IMU readings in SI units in NED body frame */
        fun HIGHRES_IMU(bytes: Pack.Bytes): HIGHRES_IMU {
            bytes.init(HIGHRES_IMU.meta)
            return com.company.demo.GroundControl.HIGHRES_IMU(bytes)
        }

        /**
         *Provides state for additional features */
        fun EXTENDED_SYS_STATE(cur: Cursor): EXTENDED_SYS_STATE {
            cur.init(EXTENDED_SYS_STATE.meta)
            return com.company.demo.GroundControl.EXTENDED_SYS_STATE(cur)
        }

        /**
         *Dynamic data used to generate ADS-B out transponder data (send at 5Hz) */
        fun UAVIONIX_ADSB_OUT_DYNAMIC(cur: Cursor): UAVIONIX_ADSB_OUT_DYNAMIC {
            cur.init(UAVIONIX_ADSB_OUT_DYNAMIC.meta)
            return com.company.demo.GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC(cur)
        }

        /**
         *Response from a GOPRO_COMMAND get request */
        fun GOPRO_GET_RESPONSE(cur: Cursor): GOPRO_GET_RESPONSE {
            cur.init(GOPRO_GET_RESPONSE.meta)
            return com.company.demo.GroundControl.GOPRO_GET_RESPONSE(cur)
        }

        /**
         *data for injecting into the onboard GPS (used for DGPS) */
        fun GPS_INJECT_DATA(bytes: Pack.Bytes): GPS_INJECT_DATA {
            bytes.init(GPS_INJECT_DATA.meta)
            return com.company.demo.GroundControl.GPS_INJECT_DATA(bytes)
        }

        /**
         *Transceiver heartbeat with health report (updated every 10s) */
        fun UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur: Cursor): UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT {
            cur.init(UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.meta)
            return com.company.demo.GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(cur)
        }

        /**
         *Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
         *				 efficient way for testing new messages and getting experimental debug output */
        fun NAMED_VALUE_INT(cur: Cursor): NAMED_VALUE_INT {
            cur.init(NAMED_VALUE_INT.meta)
            return com.company.demo.GroundControl.NAMED_VALUE_INT(cur)
        }

        /**
         *RPM sensor output */
        fun RPM(bytes: Pack.Bytes): RPM {
            bytes.init(RPM.meta)
            return com.company.demo.GroundControl.RPM(bytes)
        }

        /**
         *RTCM message for injecting into the onboard GPS (used for DGPS) */
        fun GPS_RTCM_DATA(bytes: Pack.Bytes): GPS_RTCM_DATA {
            bytes.init(GPS_RTCM_DATA.meta)
            return com.company.demo.GroundControl.GPS_RTCM_DATA(bytes)
        }

        /**
         *File transfer message */
        fun FILE_TRANSFER_PROTOCOL(bytes: Pack.Bytes): FILE_TRANSFER_PROTOCOL {
            bytes.init(FILE_TRANSFER_PROTOCOL.meta)
            return com.company.demo.GroundControl.FILE_TRANSFER_PROTOCOL(bytes)
        }

        /**
         *Rangefinder reporting */
        fun RANGEFINDER(bytes: Pack.Bytes): RANGEFINDER {
            bytes.init(RANGEFINDER.meta)
            return com.company.demo.GroundControl.RANGEFINDER(bytes)
        }

        /**
         *Status generated by radio and injected into MAVLink stream. */
        fun RADIO_STATUS(bytes: Pack.Bytes): RADIO_STATUS {
            bytes.init(RADIO_STATUS.meta)
            return com.company.demo.GroundControl.RADIO_STATUS(bytes)
        }

        /**
         *GCS */
        fun FENCE_POINT(bytes: Pack.Bytes): FENCE_POINT {
            bytes.init(FENCE_POINT.meta)
            return com.company.demo.GroundControl.FENCE_POINT(bytes)
        }

        /**
         *The autopilot is requesting a resource (file, binary, other type of data) */
        fun RESOURCE_REQUEST(bytes: Pack.Bytes): RESOURCE_REQUEST {
            bytes.init(RESOURCE_REQUEST.meta)
            return com.company.demo.GroundControl.RESOURCE_REQUEST(bytes)
        }
    }


    val receiver: java.io.OutputStream = object : Channel.Receiver(1, 1) {
        override fun dispatch(id: Int, pack: Pack?): Meta? {
            when (id) {
                201 -> on_NAV_CONTROLLER_OUTPUT(pack ?: return NAV_CONTROLLER_OUTPUT.meta)
                202 -> on_OPTICAL_FLOW_RAD(pack ?: return OPTICAL_FLOW_RAD.meta)
                125 -> on_HEARTBEAT(pack ?: return HEARTBEAT.meta)
                156 -> on_POWER_STATUS(pack ?: return POWER_STATUS.meta)
                196 -> on_HIL_STATE_QUATERNION(pack ?: return HIL_STATE_QUATERNION.meta)
                85 -> on_MISSION_ACK(pack ?: return MISSION_ACK.meta)
                133 -> on_POSITION_TARGET_GLOBAL_INT(pack ?: return POSITION_TARGET_GLOBAL_INT.meta)
                42 -> on_VISION_SPEED_ESTIMATE(pack ?: return VISION_SPEED_ESTIMATE.meta)
                78 -> on_ADSB_VEHICLE(pack ?: return ADSB_VEHICLE.meta)
                131 -> on_LOCAL_POSITION_NED(pack ?: return LOCAL_POSITION_NED.meta)
                107 -> on_RADIO_STATUS(pack ?: return RADIO_STATUS.meta)
                167 -> on_COMMAND_INT(pack ?: return COMMAND_INT.meta)
                174 -> on_V2_EXTENSION(pack ?: return V2_EXTENSION.meta)
                179 -> on_SAFETY_ALLOWED_AREA(pack ?: return SAFETY_ALLOWED_AREA.meta)
                108 -> on_SIM_STATE(pack ?: return SIM_STATE.meta)
                48 -> on_GPS_STATUS(pack ?: return GPS_STATUS.meta)
                151 -> on_SCALED_PRESSURE2(pack ?: return SCALED_PRESSURE2.meta)
                178 -> on_SCALED_IMU2(pack ?: return SCALED_IMU2.meta)
                73 -> on_GPS_INPUT(pack ?: return GPS_INPUT.meta)
                77 -> on_SET_HOME_POSITION(pack ?: return SET_HOME_POSITION.meta)
                163 -> on_RAW_IMU(pack ?: return RAW_IMU.meta)
                52 -> on_GPS_INJECT_DATA(pack ?: return GPS_INJECT_DATA.meta)
                120 -> on_RC_CHANNELS_SCALED(pack ?: return RC_CHANNELS_SCALED.meta)
                207 -> on_CONTROL_SYSTEM_STATE(pack ?: return CONTROL_SYSTEM_STATE.meta)
                81 -> on_MISSION_ITEM_REACHED(pack ?: return MISSION_ITEM_REACHED.meta)
                157 -> on_CAMERA_INFORMATION(pack ?: return CAMERA_INFORMATION.meta)
                146 -> on_BUTTON_CHANGE(pack ?: return BUTTON_CHANGE.meta)
                106 -> on_ATTITUDE_TARGET(pack ?: return ATTITUDE_TARGET.meta)
                89 -> on_GPS_RTK(pack ?: return GPS_RTK.meta)
                204 -> on_LOG_DATA(pack ?: return LOG_DATA.meta)
                95 -> on_AUTH_KEY(pack ?: return AUTH_KEY.meta)
                54 -> on_MANUAL_CONTROL(pack ?: return MANUAL_CONTROL.meta)
                199 -> on_HIL_ACTUATOR_CONTROLS(pack ?: return HIL_ACTUATOR_CONTROLS.meta)
                181 -> on_CAMERA_SETTINGS(pack ?: return CAMERA_SETTINGS.meta)
                67 -> on_RESOURCE_REQUEST(pack ?: return RESOURCE_REQUEST.meta)
                79 -> on_SERVO_OUTPUT_RAW(pack ?: return SERVO_OUTPUT_RAW.meta)
                206 -> on_MISSION_REQUEST_PARTIAL_LIST(pack ?: return MISSION_REQUEST_PARTIAL_LIST.meta)
                91 -> on_GPS2_RTK(pack ?: return GPS2_RTK.meta)
                137 -> on_HIGHRES_IMU(pack ?: return HIGHRES_IMU.meta)
                216 -> on_CAMERA_CAPTURE_STATUS(pack ?: return CAMERA_CAPTURE_STATUS.meta)
                123 -> on_PARAM_SET(pack ?: return PARAM_SET.meta)
                34 -> on_ALTITUDE(pack ?: return ALTITUDE.meta)
                168 -> on_HIL_GPS(pack ?: return HIL_GPS.meta)
                130 -> on_SCALED_PRESSURE3(pack ?: return SCALED_PRESSURE3.meta)
                72 -> on_LOG_REQUEST_DATA(pack ?: return LOG_REQUEST_DATA.meta)
                213 -> on_GPS2_RAW(pack ?: return GPS2_RAW.meta)
                27 -> on_FILE_TRANSFER_PROTOCOL(pack ?: return FILE_TRANSFER_PROTOCOL.meta)
                215 -> on_FLIGHT_INFORMATION(pack ?: return FLIGHT_INFORMATION.meta)
                145 -> on_MISSION_WRITE_PARTIAL_LIST(pack ?: return MISSION_WRITE_PARTIAL_LIST.meta)
                16 -> on_VISION_POSITION_ESTIMATE(pack ?: return VISION_POSITION_ESTIMATE.meta)
                147 -> on_MISSION_REQUEST(pack ?: return MISSION_REQUEST.meta)
                209 -> on_SET_POSITION_TARGET_LOCAL_NED(pack ?: return SET_POSITION_TARGET_LOCAL_NED.meta)
                154 -> on_CHANGE_OPERATOR_CONTROL(pack ?: return CHANGE_OPERATOR_CONTROL.meta)
                92 -> on_TERRAIN_REQUEST(pack ?: return TERRAIN_REQUEST.meta)
                218 -> on_SET_POSITION_TARGET_GLOBAL_INT(pack ?: return SET_POSITION_TARGET_GLOBAL_INT.meta)
                62 -> on_CHANGE_OPERATOR_CONTROL_ACK(pack ?: return CHANGE_OPERATOR_CONTROL_ACK.meta)
                8 -> on_NAMED_VALUE_FLOAT(pack ?: return NAMED_VALUE_FLOAT.meta)
                203 -> on_PLAY_TUNE(pack ?: return PLAY_TUNE.meta)
                193 -> on_MISSION_CLEAR_ALL(pack ?: return MISSION_CLEAR_ALL.meta)
                94 -> on_SET_ACTUATOR_CONTROL_TARGET(pack ?: return SET_ACTUATOR_CONTROL_TARGET.meta)
                50 -> on_MISSION_COUNT(pack ?: return MISSION_COUNT.meta)
                126 -> on_DEBUG(pack ?: return DEBUG.meta)
                149 -> on_DATA_TRANSMISSION_HANDSHAKE(pack ?: return DATA_TRANSMISSION_HANDSHAKE.meta)
                41 -> on_MESSAGE_INTERVAL(pack ?: return MESSAGE_INTERVAL.meta)
                56 -> on_GPS_RAW_INT(pack ?: return GPS_RAW_INT.meta)
                205 -> on_SETUP_SIGNING(pack ?: return SETUP_SIGNING.meta)
                61 -> on_DISTANCE_SENSOR(pack ?: return DISTANCE_SENSOR.meta)
                132 -> on_SYS_STATUS(pack ?: return SYS_STATUS.meta)
                53 -> on_SET_GPS_GLOBAL_ORIGIN(pack ?: return SET_GPS_GLOBAL_ORIGIN.meta)
                124 -> on_STATUSTEXT(pack ?: return STATUSTEXT.meta)
                24 -> on_SAFETY_SET_ALLOWED_AREA(pack ?: return SAFETY_SET_ALLOWED_AREA.meta)
                110 -> on_GLOBAL_VISION_POSITION_ESTIMATE(pack ?: return GLOBAL_VISION_POSITION_ESTIMATE.meta)
                148 -> on_RC_CHANNELS_RAW(pack ?: return RC_CHANNELS_RAW.meta)
                10 -> on_CAMERA_IMAGE_CAPTURED(pack ?: return CAMERA_IMAGE_CAPTURED.meta)
                88 -> on_GPS_RTCM_DATA(pack ?: return GPS_RTCM_DATA.meta)
                98 -> on_REQUEST_DATA_STREAM(pack ?: return REQUEST_DATA_STREAM.meta)
                99 -> on_SERIAL_CONTROL(pack ?: return SERIAL_CONTROL.meta)
                140 -> on_HIL_STATE(pack ?: return HIL_STATE.meta)
                15 -> on_VFR_HUD(pack ?: return VFR_HUD.meta)
                180 -> on_LOCAL_POSITION_NED_COV(pack ?: return LOCAL_POSITION_NED_COV.meta)
                47 -> on_PARAM_REQUEST_LIST(pack ?: return PARAM_REQUEST_LIST.meta)
                45 -> on_COLLISION(pack ?: return COLLISION.meta)
                212 -> on_MISSION_SET_CURRENT(pack ?: return MISSION_SET_CURRENT.meta)
                121 -> on_COMMAND_ACK(pack ?: return COMMAND_ACK.meta)
                219 -> on_MANUAL_SETPOINT(pack ?: return MANUAL_SETPOINT.meta)
                4 -> on_LOG_ERASE(pack ?: return LOG_ERASE.meta)
                25 -> on_MISSION_REQUEST_LIST(pack ?: return MISSION_REQUEST_LIST.meta)
                210 -> on_HOME_POSITION(pack ?: return HOME_POSITION.meta)
                39 -> on_FOLLOW_TARGET(pack ?: return FOLLOW_TARGET.meta)
                31 -> on_DATA_STREAM(pack ?: return DATA_STREAM.meta)
                19 -> on_RAW_PRESSURE(pack ?: return RAW_PRESSURE.meta)
                184 -> on_HIL_RC_INPUTS_RAW(pack ?: return HIL_RC_INPUTS_RAW.meta)
                173 -> on_TERRAIN_DATA(pack ?: return TERRAIN_DATA.meta)
                38 -> on_ESTIMATOR_STATUS(pack ?: return ESTIMATOR_STATUS.meta)
                200 -> on_ATT_POS_MOCAP(pack ?: return ATT_POS_MOCAP.meta)
                170 -> on_ENCAPSULATED_DATA(pack ?: return ENCAPSULATED_DATA.meta)
                87 -> on_HIGH_LATENCY(pack ?: return HIGH_LATENCY.meta)
                116 -> on_HIL_OPTICAL_FLOW(pack ?: return HIL_OPTICAL_FLOW.meta)
                86 -> on_TERRAIN_REPORT(pack ?: return TERRAIN_REPORT.meta)
                169 -> on_HIL_SENSOR(pack ?: return HIL_SENSOR.meta)
                176 -> on_EXTENDED_SYS_STATE(pack ?: return EXTENDED_SYS_STATE.meta)
                182 -> on_ATTITUDE_QUATERNION_COV(pack ?: return ATTITUDE_QUATERNION_COV.meta)
                37 -> on_LOG_REQUEST_LIST(pack ?: return LOG_REQUEST_LIST.meta)
                46 -> on_TIMESYNC(pack ?: return TIMESYNC.meta)
                71 -> on_ATTITUDE_QUATERNION(pack ?: return ATTITUDE_QUATERNION.meta)
                18 -> on_SCALED_PRESSURE(pack ?: return SCALED_PRESSURE.meta)
                152 -> on_TERRAIN_CHECK(pack ?: return TERRAIN_CHECK.meta)
                134 -> on_ATTITUDE(pack ?: return ATTITUDE.meta)
                188 -> on_MISSION_REQUEST_INT(pack ?: return MISSION_REQUEST_INT.meta)
                161 -> on_LOG_ENTRY(pack ?: return LOG_ENTRY.meta)
                82 -> on_HIL_CONTROLS(pack ?: return HIL_CONTROLS.meta)
                74 -> on_SET_ATTITUDE_TARGET(pack ?: return SET_ATTITUDE_TARGET.meta)
                28 -> pack?.let { on_SwitchModeCommand() } ?: return SwitchModeCommand.meta
                30 -> on_MEMORY_VECT(pack ?: return MEMORY_VECT.meta)
                58 -> on_STORAGE_INFORMATION(pack ?: return STORAGE_INFORMATION.meta)
                102 -> on_LOG_REQUEST_END(pack ?: return LOG_REQUEST_END.meta)
                2 -> on_SCALED_IMU3(pack ?: return SCALED_IMU3.meta)
                101 -> on_COMMAND_LONG(pack ?: return COMMAND_LONG.meta)
                158 -> on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(pack ?: return LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.meta)
                111 -> on_BATTERY_STATUS(pack ?: return BATTERY_STATUS.meta)
                135 -> on_VICON_POSITION_ESTIMATE(pack ?: return VICON_POSITION_ESTIMATE.meta)
                23 -> on_RC_CHANNELS(pack ?: return RC_CHANNELS.meta)
                172 -> on_LANDING_TARGET(pack ?: return LANDING_TARGET.meta)
                185 -> on_GPS_GLOBAL_ORIGIN(pack ?: return GPS_GLOBAL_ORIGIN.meta)
                136 -> on_PARAM_VALUE(pack ?: return PARAM_VALUE.meta)
                187 -> on_DEBUG_VECT(pack ?: return DEBUG_VECT.meta)
                3 -> on_ACTUATOR_CONTROL_TARGET(pack ?: return ACTUATOR_CONTROL_TARGET.meta)
                189 -> on_MISSION_CURRENT(pack ?: return MISSION_CURRENT.meta)
                6 -> on_GLOBAL_POSITION_INT_COV(pack ?: return GLOBAL_POSITION_INT_COV.meta)
                13 -> on_OPTICAL_FLOW(pack ?: return OPTICAL_FLOW.meta)
                29 -> on_PARAM_MAP_RC(pack ?: return PARAM_MAP_RC.meta)
                59 -> on_MISSION_ITEM_INT(pack ?: return MISSION_ITEM_INT.meta)
                195 -> on_SCALED_IMU(pack ?: return SCALED_IMU.meta)
                162 -> on_GLOBAL_POSITION_INT(pack ?: return GLOBAL_POSITION_INT.meta)
                129 -> on_POSITION_TARGET_LOCAL_NED(pack ?: return POSITION_TARGET_LOCAL_NED.meta)
                160 -> on_WIND_COV(pack ?: return WIND_COV.meta)
                21 -> on_NAMED_VALUE_INT(pack ?: return NAMED_VALUE_INT.meta)
                9 -> on_SYSTEM_TIME(pack ?: return SYSTEM_TIME.meta)
                103 -> on_VIBRATION(pack ?: return VIBRATION.meta)
                139 -> on_PING33(pack ?: return PING33.meta)
                7 -> on_SET_MODE(pack ?: return SET_MODE.meta)
                166 -> on_PARAM_REQUEST_READ(pack ?: return PARAM_REQUEST_READ.meta)
                60 -> on_AUTOPILOT_VERSION(pack ?: return AUTOPILOT_VERSION.meta)
                35 -> on_MISSION_ITEM(pack ?: return MISSION_ITEM.meta)
                122 -> on_PING(pack ?: return PING.meta)
                55 -> on_RC_CHANNELS_OVERRIDE(pack ?: return RC_CHANNELS_OVERRIDE.meta)
                191 -> on_CAMERA_TRIGGER(pack ?: return CAMERA_TRIGGER.meta)

                else -> return null
            }
            return null
        }
    }

    abstract fun on_RESOURCE_REQUEST(pack: Pack)
    abstract fun on_ATTITUDE_TARGET(pack: Pack)
    abstract fun on_MISSION_COUNT(pack: Pack)
    abstract fun on_ADSB_VEHICLE(pack: Pack)
    abstract fun on_MESSAGE_INTERVAL(pack: Pack)
    abstract fun on_ESTIMATOR_STATUS(pack: Pack)
    abstract fun on_TIMESYNC(pack: Pack)
    abstract fun on_GLOBAL_POSITION_INT_COV(pack: Pack)
    abstract fun on_BUTTON_CHANGE(pack: Pack)
    abstract fun on_SAFETY_SET_ALLOWED_AREA(pack: Pack)
    abstract fun on_STORAGE_INFORMATION(pack: Pack)
    abstract fun on_COLLISION(pack: Pack)
    abstract fun on_ALTITUDE(pack: Pack)
    abstract fun on_HIL_STATE_QUATERNION(pack: Pack)
    abstract fun on_CAMERA_INFORMATION(pack: Pack)
    abstract fun on_GPS_STATUS(pack: Pack)
    abstract fun on_PARAM_SET(pack: Pack)
    abstract fun on_TERRAIN_DATA(pack: Pack)
    abstract fun on_RC_CHANNELS_OVERRIDE(pack: Pack)
    abstract fun on_SCALED_IMU(pack: Pack)
    abstract fun on_DEBUG(pack: Pack)
    abstract fun on_CAMERA_IMAGE_CAPTURED(pack: Pack)
    abstract fun on_LOG_ENTRY(pack: Pack)
    abstract fun on_ACTUATOR_CONTROL_TARGET(pack: Pack)
    abstract fun on_HIGH_LATENCY(pack: Pack)
    abstract fun on_PARAM_REQUEST_READ(pack: Pack)
    abstract fun on_SET_ATTITUDE_TARGET(pack: Pack)
    abstract fun on_FOLLOW_TARGET(pack: Pack)
    abstract fun on_HIL_STATE(pack: Pack)
    abstract fun on_HOME_POSITION(pack: Pack)
    abstract fun on_GPS2_RAW(pack: Pack)
    abstract fun on_MEMORY_VECT(pack: Pack)
    abstract fun on_REQUEST_DATA_STREAM(pack: Pack)
    abstract fun on_HIL_CONTROLS(pack: Pack)
    abstract fun on_HIL_SENSOR(pack: Pack)
    abstract fun on_SETUP_SIGNING(pack: Pack)
    abstract fun on_GPS_RTK(pack: Pack)
    abstract fun on_PARAM_REQUEST_LIST(pack: Pack)
    abstract fun on_LANDING_TARGET(pack: Pack)
    abstract fun on_SET_ACTUATOR_CONTROL_TARGET(pack: Pack)
    abstract fun on_CONTROL_SYSTEM_STATE(pack: Pack)
    abstract fun on_SET_POSITION_TARGET_GLOBAL_INT(pack: Pack)
    abstract fun on_VIBRATION(pack: Pack)
    abstract fun on_PING33(pack: Pack)
    abstract fun on_VFR_HUD(pack: Pack)
    abstract fun on_MISSION_SET_CURRENT(pack: Pack)
    abstract fun on_HIL_GPS(pack: Pack)
    abstract fun on_NAV_CONTROLLER_OUTPUT(pack: Pack)
    abstract fun on_AUTH_KEY(pack: Pack)
    abstract fun on_LOCAL_POSITION_NED_COV(pack: Pack)
    abstract fun on_ATT_POS_MOCAP(pack: Pack)
    abstract fun on_STATUSTEXT(pack: Pack)
    abstract fun on_PING(pack: Pack)
    abstract fun on_CAMERA_CAPTURE_STATUS(pack: Pack)
    abstract fun on_GLOBAL_POSITION_INT(pack: Pack)
    abstract fun on_ENCAPSULATED_DATA(pack: Pack)
    abstract fun on_GPS_INPUT(pack: Pack)
    abstract fun on_COMMAND_LONG(pack: Pack)
    abstract fun on_LOG_REQUEST_DATA(pack: Pack)
    abstract fun on_GPS_RAW_INT(pack: Pack)
    abstract fun on_RC_CHANNELS_SCALED(pack: Pack)
    abstract fun on_CAMERA_SETTINGS(pack: Pack)
    abstract fun on_RAW_PRESSURE(pack: Pack)
    abstract fun on_NAMED_VALUE_FLOAT(pack: Pack)
    abstract fun on_ATTITUDE(pack: Pack)
    abstract fun on_TERRAIN_REQUEST(pack: Pack)
    abstract fun on_MISSION_WRITE_PARTIAL_LIST(pack: Pack)
    abstract fun on_LOG_ERASE(pack: Pack)
    abstract fun on_MANUAL_SETPOINT(pack: Pack)
    abstract fun on_SAFETY_ALLOWED_AREA(pack: Pack)
    abstract fun on_OPTICAL_FLOW_RAD(pack: Pack)
    abstract fun on_LOG_DATA(pack: Pack)
    abstract fun on_MISSION_CLEAR_ALL(pack: Pack)
    abstract fun on_VICON_POSITION_ESTIMATE(pack: Pack)
    abstract fun on_GPS2_RTK(pack: Pack)
    abstract fun on_LOG_REQUEST_LIST(pack: Pack)
    abstract fun on_SCALED_PRESSURE(pack: Pack)
    abstract fun on_MISSION_REQUEST_INT(pack: Pack)
    abstract fun on_V2_EXTENSION(pack: Pack)
    abstract fun on_HEARTBEAT(pack: Pack)
    abstract fun on_PARAM_MAP_RC(pack: Pack)
    abstract fun on_POWER_STATUS(pack: Pack)
    abstract fun on_TERRAIN_CHECK(pack: Pack)
    abstract fun on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(pack: Pack)
    abstract fun on_COMMAND_ACK(pack: Pack)
    abstract fun on_DATA_STREAM(pack: Pack)
    abstract fun on_MISSION_REQUEST(pack: Pack)
    abstract fun on_TERRAIN_REPORT(pack: Pack)
    abstract fun on_SET_HOME_POSITION(pack: Pack)
    abstract fun on_SwitchModeCommand()
    abstract fun on_HIL_RC_INPUTS_RAW(pack: Pack)
    abstract fun on_SCALED_IMU3(pack: Pack)
    abstract fun on_SET_MODE(pack: Pack)
    abstract fun on_POSITION_TARGET_GLOBAL_INT(pack: Pack)
    abstract fun on_FLIGHT_INFORMATION(pack: Pack)
    abstract fun on_SIM_STATE(pack: Pack)
    abstract fun on_MISSION_ITEM_REACHED(pack: Pack)
    abstract fun on_RC_CHANNELS_RAW(pack: Pack)
    abstract fun on_SERVO_OUTPUT_RAW(pack: Pack)
    abstract fun on_VISION_SPEED_ESTIMATE(pack: Pack)
    abstract fun on_DEBUG_VECT(pack: Pack)
    abstract fun on_LOG_REQUEST_END(pack: Pack)
    abstract fun on_MISSION_ACK(pack: Pack)
    abstract fun on_CHANGE_OPERATOR_CONTROL_ACK(pack: Pack)
    abstract fun on_MISSION_CURRENT(pack: Pack)
    abstract fun on_SYSTEM_TIME(pack: Pack)
    abstract fun on_CAMERA_TRIGGER(pack: Pack)
    abstract fun on_VISION_POSITION_ESTIMATE(pack: Pack)
    abstract fun on_MANUAL_CONTROL(pack: Pack)
    abstract fun on_RC_CHANNELS(pack: Pack)
    abstract fun on_PARAM_VALUE(pack: Pack)
    abstract fun on_BATTERY_STATUS(pack: Pack)
    abstract fun on_SET_POSITION_TARGET_LOCAL_NED(pack: Pack)
    abstract fun on_SERIAL_CONTROL(pack: Pack)
    abstract fun on_SET_GPS_GLOBAL_ORIGIN(pack: Pack)
    abstract fun on_AUTOPILOT_VERSION(pack: Pack)
    abstract fun on_MISSION_REQUEST_LIST(pack: Pack)
    abstract fun on_PLAY_TUNE(pack: Pack)
    abstract fun on_SCALED_PRESSURE3(pack: Pack)
    abstract fun on_MISSION_REQUEST_PARTIAL_LIST(pack: Pack)
    abstract fun on_LOCAL_POSITION_NED(pack: Pack)
    abstract fun on_DATA_TRANSMISSION_HANDSHAKE(pack: Pack)
    abstract fun on_GPS_GLOBAL_ORIGIN(pack: Pack)
    abstract fun on_SCALED_IMU2(pack: Pack)
    abstract fun on_ATTITUDE_QUATERNION(pack: Pack)
    abstract fun on_HIL_ACTUATOR_CONTROLS(pack: Pack)
    abstract fun on_POSITION_TARGET_LOCAL_NED(pack: Pack)
    abstract fun on_DISTANCE_SENSOR(pack: Pack)
    abstract fun on_HIL_OPTICAL_FLOW(pack: Pack)
    abstract fun on_SCALED_PRESSURE2(pack: Pack)
    abstract fun on_WIND_COV(pack: Pack)
    abstract fun on_CHANGE_OPERATOR_CONTROL(pack: Pack)
    abstract fun on_SYS_STATUS(pack: Pack)
    abstract fun on_MISSION_ITEM(pack: Pack)
    abstract fun on_RAW_IMU(pack: Pack)
    abstract fun on_COMMAND_INT(pack: Pack)
    abstract fun on_OPTICAL_FLOW(pack: Pack)
    abstract fun on_MISSION_ITEM_INT(pack: Pack)
    abstract fun on_HIGHRES_IMU(pack: Pack)
    abstract fun on_EXTENDED_SYS_STATE(pack: Pack)
    abstract fun on_GPS_INJECT_DATA(pack: Pack)
    abstract fun on_ATTITUDE_QUATERNION_COV(pack: Pack)
    abstract fun on_NAMED_VALUE_INT(pack: Pack)
    abstract fun on_RADIO_STATUS(pack: Pack)
    abstract fun on_GPS_RTCM_DATA(pack: Pack)
    abstract fun on_GLOBAL_VISION_POSITION_ESTIMATE(pack: Pack)
    abstract fun on_FILE_TRANSFER_PROTOCOL(pack: Pack)


}
										
						