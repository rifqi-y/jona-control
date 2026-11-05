/**
 ******************************************************************************
 * @file    Robot.cpp
 ******************************************************************************
 */

#include "Robot.h"
#include "kinematics.h"

#include <cmath>
#include <cstdint>

using namespace RobotConfig;

// ===========================================================================
// Constructor & init
// ===========================================================================
Robot::Robot(StepperMotor* motorArray) : motors(motorArray) {}

void Robot::init() {
    kin_robot_set();
    // Tool frame default: identity (TCP = flange motor terakhir).
    kin_set_tool_frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // Set kecepatan default tiap motor sesuai konfig.
    for (int j = 0; j < NUM_JOINTS; j++) {
        int m = JOINT_TO_MOTOR[j];
        motors[m].halfPeriodMicros = JOINT_DEFAULT_HALF_PERIOD_US[j];
    }
}

// ===========================================================================
// Konversi koordinat
// ===========================================================================
long Robot::degToSteps(int joint_idx, float deg) const {
    float steps = deg * stepsPerDegree(joint_idx) * (float)JOINT_DIR_SIGN[joint_idx];
    // Rounding to nearest integer (bukan truncation)
    return (long)((steps >= 0.0f) ? (steps + 0.5f) : (steps - 0.5f));
}

float Robot::stepsToDeg(int joint_idx, long steps) const {
    return ((float)steps / stepsPerDegree(joint_idx)) * (float)JOINT_DIR_SIGN[joint_idx];
}

float Robot::getCurrentJointDeg(int joint_idx) const {
    int m = JOINT_TO_MOTOR[joint_idx];
    return stepsToDeg(joint_idx, motors[m].currentPosition);
}

void Robot::getCurrentJoints(float out_joints[NUM_JOINTS]) const {
    for (int j = 0; j < NUM_JOINTS; j++) {
        out_joints[j] = getCurrentJointDeg(j);
    }
}

// ===========================================================================
// Joint-space commands
// ===========================================================================
bool Robot::moveJointRelative(int joint_idx, float delta_deg) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return false;
    int m = JOINT_TO_MOTOR[joint_idx];

    // Pengecekan limit berdasarkan target FINAL (target_sekarang + delta),
    // bukan posisi current. Penting untuk jog beruntun saat motor masih
    // bergerak ke target sebelumnya.
    float target_now   = stepsToDeg(joint_idx, motors[m].targetPosition);
    float target_final = target_now + delta_deg;

    if (!isJointInLimit(joint_idx, target_final)) return false;

    motors[m].targetPosition += degToSteps(joint_idx, delta_deg);
    return true;
}

bool Robot::moveJointAbsolute(int joint_idx, float target_deg) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return false;
    if (!isJointInLimit(joint_idx, target_deg)) return false;

    int m = JOINT_TO_MOTOR[joint_idx];
    motors[m].targetPosition = degToSteps(joint_idx, target_deg);
    return true;
}

bool Robot::moveAllJointsAbsolute(const float targets_deg[NUM_JOINTS]) {
    // Validate all joints BEFORE committing any movement
    for (int j = 0; j < NUM_JOINTS; j++) {
        if (!isJointInLimit(j, targets_deg[j])) return false;
    }
    applyCoordinatedMotion(targets_deg);
    return true;
}

// ===========================================================================
// Coordinated motion (sinkronisasi multi-motor)
// ===========================================================================
void Robot::applyCoordinatedMotion(const float targets_deg[NUM_JOINTS]) {
    long  target_steps[NUM_JOINTS];
    long  abs_delta[NUM_JOINTS];
    long  max_delta = 0;

    // 1) Hitung target_step dan |delta| tiap motor.
    for (int j = 0; j < NUM_JOINTS; j++) {
        int m = JOINT_TO_MOTOR[j];
        target_steps[j] = degToSteps(j, targets_deg[j]);
        long d = target_steps[j] - motors[m].currentPosition;
        abs_delta[j] = (d >= 0) ? d : -d;
        if (abs_delta[j] > max_delta) max_delta = abs_delta[j];
    }

    if (max_delta == 0) return;  // sudah di posisi target

    // 2) Scale halfPeriodMicros tiap motor agar semua selesai bersamaan.
    //    Joint dengan waktu tempuh terlama pakai kecepatan asli (paling cepat).
    //    Joint lain di-perlambat agar waktu tempuhnya sama.
    float max_time = 0.0f;
    for (int j = 0; j < NUM_JOINTS; j++) {
        float t = (float)abs_delta[j] * (float)JOINT_DEFAULT_HALF_PERIOD_US[j];
        if (t > max_time) max_time = t;
    }

    for (int j = 0; j < NUM_JOINTS; j++) {
        int m = JOINT_TO_MOTOR[j];
        if (abs_delta[j] > 0) {
            uint32_t period = (uint32_t)(max_time / (float)abs_delta[j]);
            // Clamp minimum agar tidak terlalu cepat
            if (period < 100) period = 100;
            motors[m].halfPeriodMicros = period;
        } else {
            motors[m].halfPeriodMicros = JOINT_DEFAULT_HALF_PERIOD_US[j];
        }
    }

    // 3) Commit target step ke semua motor (atomic-ish; tetap ada race window
    //    kecil dengan ISR USB, tapi kalau tidak ada perintah baru di tengah
    //    loop ini, aman).
    for (int j = 0; j < NUM_JOINTS; j++) {
        int m = JOINT_TO_MOTOR[j];
        motors[m].targetPosition = target_steps[j];
    }
}

// ===========================================================================
// Cartesian command (pakai IK)
// ===========================================================================
bool Robot::moveCartesianAbsolute(const float xyzuvw[6],
                                  float* out_joints_solution) {
    float current_joints[NUM_JOINTS];
    float solution_joints[NUM_JOINTS];

    getCurrentJoints(current_joints);

    int ok = kin_inverse(xyzuvw, current_joints, solution_joints);
    if (!ok) return false;

    // Cek joint limit hasil IK
    for (int j = 0; j < NUM_JOINTS; j++) {
        if (!isJointInLimit(j, solution_joints[j])) return false;
    }

    if (out_joints_solution != nullptr) {
        for (int j = 0; j < NUM_JOINTS; j++)
            out_joints_solution[j] = solution_joints[j];
    }

    applyCoordinatedMotion(solution_joints);
    return true;
}

// ===========================================================================
// Kalibrasi & query
// ===========================================================================
void Robot::zeroHere() {
    for (int j = 0; j < NUM_JOINTS; j++) {
        int m = JOINT_TO_MOTOR[j];
        motors[m].currentPosition = 0;
        motors[m].targetPosition  = 0;
    }
}

void Robot::homeToZero() {
    float zeros[NUM_JOINTS] = { 0, 0, 0, 0, 0, 0 };
    applyCoordinatedMotion(zeros);
}

bool Robot::isIdle() const {
    for (int j = 0; j < NUM_JOINTS; j++) {
        int m = JOINT_TO_MOTOR[j];
        if (motors[m].currentPosition != motors[m].targetPosition) return false;
    }
    return true;
}

void Robot::getCurrentPose(float out_xyzuvw[6]) const {
    float joints[NUM_JOINTS];
    getCurrentJoints(joints);
    kin_forward(joints, out_xyzuvw);
}

void Robot::computeFK(const float joints[6], float out_xyzuvw[6]) const {
    kin_forward(joints, out_xyzuvw);
}

bool Robot::computeIK(const float xyzuvw[6], float out_joints[6]) const {
    float current_joints[NUM_JOINTS];
    getCurrentJoints(current_joints);
    int ok = kin_inverse(xyzuvw, current_joints, out_joints);
    return ok != 0;
}