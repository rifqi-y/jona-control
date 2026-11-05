#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <cstdint>

uint32_t micros(void);

// Ramp akselerasi/deselerasi: bagian awal & akhir tiap gerakan dilambatkan.
// RAMP_FRACTION = porsi total step per sisi (0 = matikan ramp).
// RAMP_SLOW_MULT = periode saat step pertama/terakhir = kecepatan cruise x ini.
// Profil ramp = akselerasi konstan. Total waktu gerak ~1.05x waktu cruise;
// kalau nilai di sini diubah, samakan RAMP_TIME_FACTOR di driver Phobot (packages/robot).
constexpr float RAMP_FRACTION  = 0.05f;
constexpr float RAMP_SLOW_MULT = 3.0f;

class StepperMotor {
    public:
    GPIO_TypeDef* pulPort;
    uint16_t pulPin;
    GPIO_TypeDef* dirPort;
    uint16_t dirPin;

    long currentPosition;
    long targetPosition;
    uint32_t lastEdgeMicros;
    uint32_t halfPeriodMicros; 
    bool pulseState;
    long rampStart = 0;   // posisi saat target terakhir berubah
    long rampTarget = 0;  // target yang sedang di-ramp

    // Deklarasi Constructor
    StepperMotor(GPIO_TypeDef* pPort, uint16_t pPin, GPIO_TypeDef* dPort, uint16_t dPin, uint32_t speedDelay);

    // Deklarasi Method
    void service();
};

#endif 