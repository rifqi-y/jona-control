/**
 ******************************************************************************
 * @file    Robot.h
 * @brief   High-level 6-axis arm controller.
 *
 *   - Konversi derajat <-> step (per joint, sesuai gear ratio).
 *   - Manajemen target Cartesian via inverse kinematics.
 *   - Sinkronisasi kecepatan multi-motor (coordinated motion).
 *   - Joint limit safety check.
 ******************************************************************************
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Motor.h"
#include "RobotConfig.h"

class Robot {
public:
    /**
     * @param motorArray  Pointer ke array StepperMotor[NUM_JOINTS]
     *                    (urutan fisik, sesuai motors[] di MainApp.cpp).
     */
    explicit Robot(StepperMotor* motorArray);

    /// Inisialisasi kinematics dan default state. Panggil saat startup.
    void init();

    // ----- Konversi koordinat -----
    long  degToSteps(int joint_idx, float deg) const;
    float stepsToDeg(int joint_idx, long  steps) const;

    /// Baca sudut joint sekarang (dari currentPosition motor).
    float getCurrentJointDeg(int joint_idx) const;

    /// Isi 6 sudut joint sekarang ke output array.
    void  getCurrentJoints(float out_joints[RobotConfig::NUM_JOINTS]) const;

    // ----- Joint-space commands (semuanya cek joint limit) -----
    /// Move joint relatif sebesar delta_deg.
    bool moveJointRelative(int joint_idx, float delta_deg);

    /// Move joint ke sudut absolut target_deg.
    bool moveJointAbsolute(int joint_idx, float target_deg);

    /// Move semua joint ke target absolut (derajat). Speed disinkronkan.
    bool moveAllJointsAbsolute(const float targets_deg[RobotConfig::NUM_JOINTS]);

    // ----- Cartesian command (pakai IK) -----
    /**
     * Move end-effector ke target pose Cartesian.
     * @param xyzuvw  {x, y, z, rx, ry, rz}  (mm, degrees)
     * @param out_joints_solution  Optional. Kalau != nullptr, isi solusi IK.
     * @return true jika IK sukses dan motor mulai bergerak.
     */
    bool moveCartesianAbsolute(const float xyzuvw[6],
                               float* out_joints_solution = nullptr);

    // ----- Kalibrasi -----
    /// Anggap posisi sekarang sebagai 0 derajat semua joint (set zero).
    void zeroHere();

    /// Move semua joint kembali ke 0 derajat (homing software, bukan limit switch).
    void homeToZero();

    // ----- Debug & status -----
    /// True jika tidak ada motor yang bergerak.
    bool isIdle() const;

    /// Hitung Cartesian pose dari sudut joint sekarang (FK).
    void getCurrentPose(float out_xyzuvw[6]) const;

    /// FK untuk sembarang sudut joint (tidak menggerakkan motor).
    void computeFK(const float joints[6], float out_xyzuvw[6]) const;

    /// IK untuk sembarang target (tidak menggerakkan motor).
    bool computeIK(const float xyzuvw[6], float out_joints[6]) const;

private:
    StepperMotor* motors;

    /**
     * Inti coordinated motion: hitung delta step tiap motor, scaling kecepatan
     * supaya semua motor selesai bersamaan, lalu set targetPosition.
     */
    void applyCoordinatedMotion(const float targets_deg[RobotConfig::NUM_JOINTS]);
};

#endif /* ROBOT_H_ */