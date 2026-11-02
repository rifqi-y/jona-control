/**
 ******************************************************************************
 * @file    kinematics.h
 * @brief   Embedded interface untuk forward & inverse kinematics 6-DOF arm.
 *
 * Diadaptasi dari kinematics.cpp original (oleh teman pengguna), distrip
 * untuk STM32 (tidak ada std::vector, std::cout, atau iostream).
 ******************************************************************************
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

/**
 * @brief Reset semua data robot ke kondisi default.
 *        Wajib dipanggil sebelum kin_robot_set().
 */
void kin_robot_data_reset(void);

/**
 * @brief Set DH parameters dan joint limits sesuai 3D model arm.
 *        Panggil sekali saat startup.
 */
void kin_robot_set(void);

/**
 * @brief Forward kinematics: joint angles -> end-effector pose.
 *
 * @param joints       [in]  6 joint angles dalam derajat (J0..J5)
 * @param out_xyzuvw   [out] {x, y, z, rx, ry, rz} dalam mm dan derajat
 */
void kin_forward(const float joints[6], float out_xyzuvw[6]);

/**
 * @brief Inverse kinematics: pose target -> joint angles.
 *
 * @param target_xyzuvw    [in]  Target {x, y, z, rx, ry, rz}
 * @param joints_estimate  [in]  Estimasi joint awal (biasanya posisi sekarang)
 * @param out_joints       [out] Joint angles hasil (derajat)
 *
 * @return 1 = sukses, 0 = gagal (target unreachable atau di luar joint limit)
 */
int  kin_inverse(const float target_xyzuvw[6],
                 const float joints_estimate[6],
                 float       out_joints[6]);

/**
 * @brief Set tool frame (TCP offset dari flange motor terakhir).
 */
void kin_set_tool_frame(float x, float y, float z,
                        float rz_deg, float ry_deg, float rx_deg);

/**
 * @brief Cek apakah satu solusi joint berada di dalam joint limits.
 * @return true jika valid.
 */
bool kin_joints_valid(const float joints[6]);

#endif /* KINEMATICS_H_ */