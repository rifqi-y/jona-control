/**
 ******************************************************************************
 * @file    RobotConfig.h
 * @brief   Konfigurasi mekanik dan elektrik 6-axis arm robot.
 *
 * UBAH NILAI DI FILE INI saat ganti motor, driver, atau gear ratio.
 * Tidak perlu menyentuh file lain.
 *
 * Konvensi:
 *   - Joint chain: J0 (base) .. J5 (wrist twist)  -> index 0..5
 *   - Motor index: sesuai urutan array motors[] di MainApp.cpp
 ******************************************************************************
 */

#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

#include <cstdint>

namespace RobotConfig {

constexpr int NUM_JOINTS = 6;

// ===========================================================================
// 1) JOINT -> MOTOR MAPPING
// ===========================================================================
// motors[] di MainApp.cpp diisi urut (lihat konstruktor):
//   motors[0] = S23     (NEMA 23,   3.0  Nm)
//   motors[1] = S17_12  (NEMA 17,   0.42 Nm)  - asumsi (cek torsi)
//   motors[2] = S17_07  (NEMA 17,   0.7  Nm)
//   motors[3] = S34_85  (NEMA 34,   8.5  Nm)
//   motors[4] = S34_12  (NEMA 34,  12.0  Nm)
//   motors[5] = S34_12_1(NEMA 34,  12.0  Nm)
//
// Mapping DIUKUR di arm fisik (tes "M:<motor>:3000" satu per satu, dilihat langsung):
//   M4 -> j1 base, M5 -> j2 bahu, M3 -> j3 siku, M0 -> j4, M2 -> j6.
//   M1 tidak menggerakkan apa pun saat dites -> j5 diisi M1 dengan eliminasi (BELUM TERBUKTI).

constexpr int JOINT_TO_MOTOR[NUM_JOINTS] = {
    4,   // J0 base       -> M4 (12 Nm)
    5,   // J1 shoulder   -> M5 (12 Nm)
    3,   // J2 elbow      -> M3 (8.5 Nm)
    0,   // J3 wrist roll -> M0 (3 Nm)
    1,   // J4 wrist pitch-> M1 (0.42 Nm)  <-- motor ini diam saat dites, cek driver/kabel
    2    // J5 wrist yaw  -> M2 (0.7 Nm)
};

// ===========================================================================
// 2) PULSA PER REVOLUSI MOTOR (sudah termasuk microstepping driver)
// ===========================================================================
// NEMA 34 HBS86H + DL86H : microstep 10, native 200 -> 2000 pulse/rev
// NEMA 23 HBS57  + SL57D : microstep  8, native 200 -> 1600 pulse/rev
// NEMA 17        + TB6600: microstep  8, native 200 -> 1600 pulse/rev
//
// Index = joint index (bukan motor index).

constexpr float PULSES_PER_MOTOR_REV[NUM_JOINTS] = {
    2000.0f,   // J0 - NEMA 34 12Nm
    2000.0f,   // J1 - NEMA 34 12Nm
    2000.0f,   // J2 - NEMA 34 8.5Nm
    1600.0f,   // J3 - NEMA 23 3Nm
    1600.0f,   // J4 - NEMA 17 0.7Nm
    1600.0f    // J5 - NEMA 17 0.42Nm
};

// ===========================================================================
// 3) GEAR RATIO (motor shaft -> joint axis)        <-- *** WAJIB DI-UPDATE ***
// ===========================================================================
// Contoh: harmonic drive 50:1 + belt 2:1 -> gear_ratio = 100.0f
//         direct drive                   -> gear_ratio = 1.0f
//
// Nilai terverifikasi di arm fisik (sama dengan Jona web app lama + tes joint J0-J5).

constexpr float JOINT_GEAR_RATIO[NUM_JOINTS] = {
    50.0f,   // J0
    100.0f,  // J1
    40.0f,   // J2
    30.0f,   // J3
    15.0f,   // J4
    10.0f    // J5
};

// ===========================================================================
// 4) DIRECTION SIGN
// ===========================================================================
// +1 = arah motor sama dengan konvensi DH.
// -1 = arah dibalik (kalau J0+ ternyata muter ke arah salah).
// Cara test: command `J:0:30` lalu lihat fisiknya. Salah arah -> ganti -1.

constexpr int8_t JOINT_DIR_SIGN[NUM_JOINTS] = {
    +1, +1, +1, +1, +1, +1
};

// ===========================================================================
// 5) JOINT LIMITS (derajat) - HARUS konsisten dengan robot_set() di kinematics
// ===========================================================================

constexpr float JOINT_LIMIT_POS[NUM_JOINTS] = {
    +170.0f, + 90.0f, +107.0f, +215.0f, +105.0f, +180.0f
};

constexpr float JOINT_LIMIT_NEG[NUM_JOINTS] = {
    -170.0f, - 95.0f, - 89.0f, -180.0f, -227.0f, -180.0f
};

// ===========================================================================
// 6) KECEPATAN DEFAULT (half-period pulsa dalam microsecond)
// ===========================================================================
// Lebih kecil = lebih cepat. Minimum aman ~100 us.
// 150/125/150/150/150/150 us = kecepatan yang sudah terbukti aman di arm ini
// (firmware lama, Jona web app). 100/100/100/150/150/150 us masih dalam tahap uji.
// Jangan di bawah ~100 us.
// Kalau driver Phobot (JONA_JOINTS di packages/robot) dipakai, samakan nilainya.

constexpr uint32_t JOINT_DEFAULT_HALF_PERIOD_US[NUM_JOINTS] = {
    100, 100, 100, 150, 150, 150
};

// ===========================================================================
// 7) JOG INCREMENT (derajat per keypress untuk real-time jog w/a/s/d/q/e)
// ===========================================================================
constexpr float JOG_INCREMENT_DEG = 5.0f;

// ===========================================================================
// 8) AUTO-CALCULATED HELPERS - jangan diubah
// ===========================================================================

// Jumlah step motor yang ekuivalen 1 derajat joint
inline float stepsPerDegree(int joint_idx) {
    return (PULSES_PER_MOTOR_REV[joint_idx] * JOINT_GEAR_RATIO[joint_idx]) / 360.0f;
}

// Cek apakah sudut joint dalam batas yang diizinkan
inline bool isJointInLimit(int joint_idx, float angle_deg) {
    return angle_deg >= JOINT_LIMIT_NEG[joint_idx] &&
           angle_deg <= JOINT_LIMIT_POS[joint_idx];
}

}  // namespace RobotConfig

#endif /* ROBOT_CONFIG_H_ */