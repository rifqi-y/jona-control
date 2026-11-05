#include "Motor.h"
#include <cmath>
#include <cstdlib>

// ==========================================
// Implementasi Constructor
// ==========================================
StepperMotor::StepperMotor(GPIO_TypeDef* pPort, uint16_t pPin, GPIO_TypeDef* dPort, uint16_t dPin, uint32_t speedDelay)
    : pulPort(pPort), pulPin(pPin), dirPort(dPort), dirPin(dPin), halfPeriodMicros(speedDelay) 
{
    // Inisialisasi nilai awal
    currentPosition = 0;
    targetPosition = 0;
    lastEdgeMicros = 0;
    pulseState = false;
}

// ==========================================
// Implementasi Method service()
// ==========================================
void StepperMotor::service() {
    if (currentPosition == targetPosition) return;

    // Target berubah (dari penulis mana pun) -> mulai ramp baru dari posisi sekarang.
    if (targetPosition != rampTarget) {
        rampStart = currentPosition;
        rampTarget = targetPosition;
    }

    // halfPeriodMicros = kecepatan cruise; lambatkan dekat awal & akhir gerakan.
    uint32_t period = halfPeriodMicros;
    long total = labs(rampTarget - rampStart);
    float rampLen = total * RAMP_FRACTION;
    if (rampLen >= 1.0f) {
        long done = labs(currentPosition - rampStart);
        long left = labs(targetPosition - currentPosition);
        long edge = done < left ? done : left;
        if (edge < rampLen) {
            // Akselerasi konstan: v^2 linear terhadap posisi, mulai dari cruise / RAMP_SLOW_MULT.
            const float v0 = 1.0f / (RAMP_SLOW_MULT * RAMP_SLOW_MULT);
            float v = sqrtf(v0 + (1.0f - v0) * ((float)edge / rampLen));
            period = (uint32_t)(halfPeriodMicros / v);
        }
    }

    uint32_t now = micros();
    if (now - lastEdgeMicros < period) return;

    bool dirHigh = (targetPosition > currentPosition);
    HAL_GPIO_WritePin(dirPort, dirPin, dirHigh ? GPIO_PIN_RESET : GPIO_PIN_SET);

    pulseState = !pulseState;
    HAL_GPIO_WritePin(pulPort, pulPin, pulseState ? GPIO_PIN_SET : GPIO_PIN_RESET);
    lastEdgeMicros = now;

    if (!pulseState) {
        currentPosition += dirHigh ? 1 : -1;
    }
}