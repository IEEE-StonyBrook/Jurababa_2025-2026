#include "Platform/Pico/MotorLab/MotorLabSettings.h"

#include <cstdio>

void MotorLabSettings::print() const {
    printf("MotorLab Settings:\n");
    printf("  Motor Model:\n");
    printf("    Km (velocity const) = %.2f deg/s/V\n", km);
    printf("    Tm (time const)     = %.5f s\n", tm);
    printf("  Feedforward:\n");
    printf("    bias_ff  = %.5f V\n", bias_ff);
    printf("    speed_ff = %.7f V/(deg/s)\n", speed_ff);
    printf("    acc_ff   = %.7f V/(deg/s^2)\n", acc_ff);
    printf("  Controller:\n");
    printf("    zeta = %.5f\n", zeta);
    printf("    Td   = %.5f s\n", td);
    printf("    Kp   = %.7f\n", kp);
    printf("    Kd   = %.7f\n", kd);
    printf("  Encoder:\n");
    printf("    deg_per_count = %.5f\n", deg_per_count);
    printf("  Control flags: 0x%02X\n", control_flags);
}
