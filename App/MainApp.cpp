/**
 ******************************************************************************
 * @file    MainApp.cpp
 * @brief   6-axis arm robot firmware entry point.
 *
 * Fitur:
 *   - Step-based motor control (lama, kompat dengan bug-test sebelumnya)
 *   - Joint-space command dalam derajat (J:, JA:, JG:)
 *   - Cartesian-space command pakai IK (L:)
 *   - Forward kinematics & IK preview (PFK, PIK:)
 *   - Coordinated multi-motor motion (semua selesai bersamaan)
 *   - Debug system 3-level (D:0/1/2)
 *   - Real-time jog dalam derajat (w/a/s/d/q/e)
 ******************************************************************************
 */

#include "MainApp.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "Motor.h"
#include "Robot.h"
#include "RobotConfig.h"

#include <string>
#include <cstdio>
#include <cstdarg>
#include <cstring>

// ==========================================
// Forward declarations
// ==========================================
void DWT_Init(void);
uint32_t micros(void);
static void serialPrint(const char* text);
static void serialPrintln(const char* text);
static void dbg_printf(int level, const char* fmt, ...);
static void processSerialCommand(const std::string& line);
static void printJointStatus(void);
static void printPoseFK(void);
static void printBanner(void);

// ==========================================
// State globals
// ==========================================
static std::string inputBuffer = "";
static bool        commandReady = false;
static uint32_t    lastLedTick = 0;
static char        pendingRealtimeAction = '\0';

enum DebugLevel { DBG_OFF = 0, DBG_INFO = 1, DBG_VERBOSE = 2 };
static int debugLevel = DBG_INFO;

// ==========================================
// Motor array (urutan fisik, sama seperti firmware original)
// ==========================================
using RobotConfig::NUM_JOINTS;

static StepperMotor motors[NUM_JOINTS] = {
    StepperMotor(S23_PUL_GPIO_Port,     S23_PUL_Pin,     S23_DIR_GPIO_Port,     S23_DIR_Pin,     250),
    StepperMotor(S17_12_PUL_GPIO_Port,  S17_12_PUL_Pin,  S17_12_DIR_GPIO_Port,  S17_12_DIR_Pin,  250),
    StepperMotor(S17_07_PUL_GPIO_Port,  S17_07_PUL_Pin,  S17_07_DIR_GPIO_Port,  S17_07_DIR_Pin,  250),
    StepperMotor(S34_85_PUL_GPIO_Port,  S34_85_PUL_Pin,  S34_85_DIR_GPIO_Port,  S34_85_DIR_Pin,  250),
    StepperMotor(S34_12_PUL_GPIO_Port,  S34_12_PUL_Pin,  S34_12_DIR_GPIO_Port,  S34_12_DIR_Pin,  250),
    StepperMotor(S34_12_PUL_1_GPIO_Port,S34_12_PUL_1_Pin,S34_12_DIR_1_GPIO_Port,S34_12_DIR_1_Pin,250)
};

// Robot controller (high-level)
static Robot robot(motors);

// ==========================================
// DWT (Cortex-M7 micros)
// ==========================================
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;  // unlock Cortex-M7 DWT
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

// ==========================================
// Serial helpers (semua via USB CDC)
// ==========================================
static void serialPrint(const char* text) {
    CDC_Transmit_FS((uint8_t*)text, std::strlen(text));
    HAL_Delay(1);
}

static void serialPrintln(const char* text) {
    char buf[256];
    int n = std::snprintf(buf, sizeof(buf), "%s\r\n", text);
    CDC_Transmit_FS((uint8_t*)buf, n);
    HAL_Delay(2);
}

// Print formatted, dengan level filter
static void dbg_printf(int level, const char* fmt, ...) {
    if (level > debugLevel) return;
    char buf[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    serialPrint(buf);
}

// ==========================================
// USB receive callback
// ==========================================
extern "C" void onUsbDataReceived(uint8_t* Buf, uint32_t* Len) {
    for (uint32_t i = 0; i < *Len; i++) {
        char c = (char)Buf[i];

        // --- Real-time jog: HURUF KECIL ---
        // Mapping standar arm:
        //   a/d : J0 base rotation
        //   w/s : J1 shoulder
        //   q/e : J2 elbow
        // Increment dari RobotConfig::JOG_INCREMENT_DEG
        switch (c) {
            case 'd': case 'a':
            case 'w': case 's':
            case 'q': case 'e':
            case ' ':
                pendingRealtimeAction = c;
                break;

            case '\n':
            case '\r':
                if (inputBuffer.length() > 0) commandReady = true;
                break;

            default:
                // Huruf besar, angka, simbol: ditampung sebagai command
                inputBuffer += c;
                break;
        }
    }
}

// ==========================================
// Eksekusi real-time jog (dipanggil dari MainApp_Loop, AMAN)
// ==========================================
static void handleRealtimeJog(char action) {
    using RobotConfig::JOG_INCREMENT_DEG;
    bool ok = true;
    const char* msg = "";

    switch (action) {
        case 'a': ok = robot.moveJointRelative(0, -JOG_INCREMENT_DEG); msg = "J0 <-"; break;
        case 'd': ok = robot.moveJointRelative(0, +JOG_INCREMENT_DEG); msg = "J0 ->"; break;
        case 's': ok = robot.moveJointRelative(1, -JOG_INCREMENT_DEG); msg = "J1 v";  break;
        case 'w': ok = robot.moveJointRelative(1, +JOG_INCREMENT_DEG); msg = "J1 ^";  break;
        case 'e': ok = robot.moveJointRelative(2, -JOG_INCREMENT_DEG); msg = "J2 v";  break;
        case 'q': ok = robot.moveJointRelative(2, +JOG_INCREMENT_DEG); msg = "J2 ^";  break;
        case ' ': {
            // Emergency stop: target = current
            for (int i = 0; i < NUM_JOINTS; i++) {
                motors[i].targetPosition = motors[i].currentPosition;
            }
            serialPrintln("!!! EMERGENCY STOP !!!");
            return;
        }
        default: return;
    }

    if (!ok) {
        dbg_printf(DBG_OFF, "ERR: joint limit reached (%s)\r\n", msg);
    } else {
        dbg_printf(DBG_INFO, "%s\r\n", msg);
    }
}

// ==========================================
// Command parsing
// ==========================================
static bool parseFloats(const char* str, float* out, int count) {
    // Parse "a:b:c:..." dengan jumlah token tepat = count
    int idx = 0;
    while (idx < count) {
        char* endptr;
        float v = std::strtof(str, &endptr);
        if (endptr == str) return false;  // tidak ada angka di sini
        out[idx++] = v;
        if (idx == count) {
            // Semua sudah terbaca; sisanya harus kosong/whitespace
            while (*endptr == ' ' || *endptr == '\t') endptr++;
            return (*endptr == '\0');
        }
        if (*endptr != ':') return false;
        str = endptr + 1;
    }
    return false;
}

// Parse "idx:value" -> int + float.
// Manual karena newlib-nano default tidak punya float scanf.
static bool parseIdxFloat(const char* str, int* out_idx, float* out_val) {
    char* endptr;
    long idx_l = std::strtol(str, &endptr, 10);
    if (endptr == str || *endptr != ':') return false;
    *out_idx = (int)idx_l;
    const char* p = endptr + 1;
    *out_val = std::strtof(p, &endptr);
    if (endptr == p) return false;
    return true;
}

static void processSerialCommand(const std::string& line) {
    const char* s = line.c_str();
    int    mIdx;
    long   val1, val2, val3;
    float  fval;
    float  fvals[6];

    // --- Cek 0-argumen commands dulu (exact match) ---
    if (line == "?") {
        printJointStatus();
        return;
    }
    if (line == "PFK") {
        printPoseFK();
        return;
    }
    if (line == "HOME") {
        robot.homeToZero();
        serialPrintln("Homing semua joint ke 0 derajat...");
        return;
    }
    if (line == "H") {
        // Legacy: home dengan set targetPosition=0 (step-based)
        for (int i = 0; i < NUM_JOINTS; i++) motors[i].targetPosition = 0;
        serialPrintln("Legacy home (step-based) -> targetPosition=0");
        return;
    }
    if (line == "Z") {
        robot.zeroHere();
        serialPrintln("Zeroed: posisi sekarang dianggap 0 derajat semua joint.");
        return;
    }

    // --- Prefix-based commands (urutan: prefix paling panjang lebih dulu) ---

    // PIK:x:y:z:rx:ry:rz  -> preview IK, tidak menggerakkan motor
    if (line.rfind("PIK:", 0) == 0) {
        if (!parseFloats(s + 4, fvals, 6)) {
            serialPrintln("Format: PIK:x:y:z:rx:ry:rz");
            return;
        }
        float joints_out[6];
        bool ok = robot.computeIK(fvals, joints_out);
        if (!ok) {
            serialPrintln("IK FAIL: target unreachable or out-of-limit");
            return;
        }
        char buf[200];
        std::snprintf(buf, sizeof(buf),
            "IK ok: J0=%.2f J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f",
            joints_out[0], joints_out[1], joints_out[2],
            joints_out[3], joints_out[4], joints_out[5]);
        serialPrintln(buf);
        return;
    }

    // JA:idx:deg  -> joint absolute
    if (line.rfind("JA:", 0) == 0) {
        if (parseIdxFloat(s + 3, &mIdx, &fval)) {
            bool ok = robot.moveJointAbsolute(mIdx, fval);
            if (!ok) {
                dbg_printf(DBG_OFF, "ERR: JA J%d ke %.2f (idx invalid atau di luar limit)\r\n", mIdx, fval);
            } else {
                dbg_printf(DBG_INFO, "JA: J%d -> %.2f deg\r\n", mIdx, fval);
            }
            return;
        }
        serialPrintln("Format: JA:idx:degrees");
        return;
    }

    // JG:j0:j1:j2:j3:j4:j5  -> group absolute (semua joint sekaligus)
    if (line.rfind("JG:", 0) == 0) {
        if (!parseFloats(s + 3, fvals, 6)) {
            serialPrintln("Format: JG:j0:j1:j2:j3:j4:j5 (derajat)");
            return;
        }
        bool ok = robot.moveAllJointsAbsolute(fvals);
        if (!ok) {
            serialPrintln("ERR: JG salah satu joint di luar limit");
            return;
        }
        dbg_printf(DBG_INFO,
            "JG: J0=%.1f J1=%.1f J2=%.1f J3=%.1f J4=%.1f J5=%.1f\r\n",
            fvals[0], fvals[1], fvals[2], fvals[3], fvals[4], fvals[5]);
        return;
    }

    // J:idx:deg  -> joint relative
    if (line.rfind("J:", 0) == 0) {
        if (parseIdxFloat(s + 2, &mIdx, &fval)) {
            bool ok = robot.moveJointRelative(mIdx, fval);
            if (!ok) {
                dbg_printf(DBG_OFF, "ERR: J%d delta %.2f (idx invalid atau limit)\r\n", mIdx, fval);
            } else {
                float now = robot.getCurrentJointDeg(mIdx);
                dbg_printf(DBG_INFO, "J: J%d += %.2f (target ~%.2f deg)\r\n",
                           mIdx, fval, now + fval);
            }
            return;
        }
        serialPrintln("Format: J:idx:degrees_delta");
        return;
    }

    // L:x:y:z:rx:ry:rz  -> Cartesian linear move (IK)
    if (line.rfind("L:", 0) == 0) {
        if (!parseFloats(s + 2, fvals, 6)) {
            serialPrintln("Format: L:x:y:z:rx:ry:rz (mm, derajat)");
            return;
        }
        float sol[6];
        bool ok = robot.moveCartesianAbsolute(fvals, sol);
        if (!ok) {
            serialPrintln("ERR: IK gagal (unreachable atau di luar limit)");
            return;
        }
        dbg_printf(DBG_INFO,
            "L: target X=%.1f Y=%.1f Z=%.1f -> J=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]\r\n",
            fvals[0], fvals[1], fvals[2],
            sol[0], sol[1], sol[2], sol[3], sol[4], sol[5]);
        return;
    }

    // V:idx:val  -> set halfPeriodMicros motor (kecepatan)
    if (std::sscanf(s, "V:%d:%ld", &mIdx, &val1) == 2) {
        if (mIdx >= 0 && mIdx < NUM_JOINTS && val1 >= 100) {
            motors[mIdx].halfPeriodMicros = (uint32_t)val1;
            dbg_printf(DBG_INFO, "Speed M%d -> %ld us\r\n", mIdx, val1);
        } else {
            serialPrintln("ERR: V salah (idx 0..5, val >= 100 us)");
        }
        return;
    }

    // M:idx:val  -> step-based relative move (legacy)
    if (std::sscanf(s, "M:%d:%ld", &mIdx, &val1) == 2) {
        if (mIdx >= 0 && mIdx < NUM_JOINTS) {
            motors[mIdx].targetPosition += val1;
            dbg_printf(DBG_INFO, "M%d target += %ld steps -> %ld\r\n",
                       mIdx, val1, motors[mIdx].targetPosition);
        } else {
            serialPrintln("ERR: M idx invalid");
        }
        return;
    }

    // G:v1:v2:v3  -> step-based group move (legacy)
    if (std::sscanf(s, "G:%ld:%ld:%ld", &val1, &val2, &val3) == 3) {
        motors[0].targetPosition += val1;
        motors[1].targetPosition += val2;
        motors[2].targetPosition += val3;
        motors[3].targetPosition += val1;
        motors[4].targetPosition += val2;
        motors[5].targetPosition += val3;
        dbg_printf(DBG_INFO, "G: M0/3+=%ld M1/4+=%ld M2/5+=%ld\r\n", val1, val2, val3);
        return;
    }

    // D:level  -> set debug level
    if (std::sscanf(s, "D:%d", &mIdx) == 1) {
        if (mIdx >= 0 && mIdx <= 2) {
            debugLevel = mIdx;
            dbg_printf(DBG_OFF, "Debug level -> %d\r\n", mIdx);
        } else {
            serialPrintln("ERR: D level 0..2");
        }
        return;
    }

    // PJ  -> print joint positions (sama dengan ?)
    if (line == "PJ") {
        printJointStatus();
        return;
    }

    // Plain number  -> legacy: gerakkan motors[0] sebanyak N steps
    if (std::sscanf(s, "%ld", &val1) == 1) {
        motors[0].targetPosition += val1;
        dbg_printf(DBG_INFO, "M0 target += %ld -> %ld\r\n",
                   val1, motors[0].targetPosition);
        return;
    }

    serialPrintln("ERR: Perintah tidak dikenali. Ketik '?' untuk status, 'PJ' untuk joint.");
}

// ==========================================
// Status printers
// ==========================================
static void printJointStatus(void) {
    char buf[160];
    for (int j = 0; j < NUM_JOINTS; j++) {
        int m = RobotConfig::JOINT_TO_MOTOR[j];
        float deg = robot.getCurrentJointDeg(j);
        std::snprintf(buf, sizeof(buf),
            "J%d (M%d): %.2f deg  step=%ld  tgt=%ld  v=%lu us",
            j, m, deg,
            motors[m].currentPosition,
            motors[m].targetPosition,
            (unsigned long)motors[m].halfPeriodMicros);
        serialPrintln(buf);
    }
}

static void printPoseFK(void) {
    float pose[6];
    robot.getCurrentPose(pose);
    char buf[160];
    std::snprintf(buf, sizeof(buf),
        "FK pose: X=%.2f Y=%.2f Z=%.2f Rx=%.2f Ry=%.2f Rz=%.2f",
        pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    serialPrintln(buf);
}

// ==========================================
// Startup banner
// ==========================================
static void printBanner(void) {
    serialPrintln("=================================================");
    serialPrintln("  6-AXIS ARM ROBOT - STM32H7");
    serialPrintln("=================================================");
    serialPrintln("Real-time jog (huruf kecil, no enter):");
    serialPrintln("  a/d = J0 base    w/s = J1 shoulder");
    serialPrintln("  q/e = J2 elbow   SPACE = emergency stop");
    serialPrintln("");
    serialPrintln("Joint commands (derajat):");
    serialPrintln("  J:idx:deg          -> joint relative");
    serialPrintln("  JA:idx:deg         -> joint absolute");
    serialPrintln("  JG:j0:j1:j2:j3:j4:j5 -> group absolute");
    serialPrintln("");
    serialPrintln("Cartesian command:");
    serialPrintln("  L:x:y:z:rx:ry:rz   -> linear move via IK");
    serialPrintln("");
    serialPrintln("Status / debug:");
    serialPrintln("  ? or PJ            -> joint status");
    serialPrintln("  PFK                -> forward kinematics pose");
    serialPrintln("  PIK:x:y:z:rx:ry:rz -> preview IK (no move)");
    serialPrintln("  HOME               -> all joints to 0 deg");
    serialPrintln("  Z                  -> zero current as 0 deg");
    serialPrintln("  D:0|1|2            -> debug level");
    serialPrintln("");
    serialPrintln("Legacy (step-based):");
    serialPrintln("  M:idx:steps  V:idx:us  G:s0:s1:s2  H");
    serialPrintln("=================================================");
}

// ==========================================
// Setup & Loop (dipanggil dari main.c)
// ==========================================
void MainApp_Setup(void) {
    DWT_Init();
    HAL_Delay(2000);  // tunggu host re-enumerate USB

    robot.init();
    printBanner();
    dbg_printf(DBG_INFO, "Setup OK. NUM_JOINTS=%d\r\n", NUM_JOINTS);
}

void MainApp_Loop(void) {
    // 1) Service semua motor (generate step pulses)
    for (int i = 0; i < NUM_JOINTS; i++) {
        motors[i].service();
    }

    // 2) LED heartbeat
    if (HAL_GetTick() - lastLedTick >= 500) {
        HAL_GPIO_TogglePin(user_led_GPIO_Port, user_led_Pin);
        lastLedTick = HAL_GetTick();
    }

    // 3) Eksekusi real-time jog yang ter-flag dari ISR
    if (pendingRealtimeAction != '\0') {
        char action = pendingRealtimeAction;
        pendingRealtimeAction = '\0';
        handleRealtimeJog(action);
    }

    // 4) Eksekusi command string
    if (commandReady) {
        std::string cmd = inputBuffer;
        inputBuffer.clear();
        commandReady = false;
        processSerialCommand(cmd);
    }
}