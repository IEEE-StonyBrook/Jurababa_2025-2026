#include "motor_lab/settings.h"

#include <cstdio>

void MotorLabSettings::print() const
{
    printf("MotorLab Settings:\n");
    printf("  Motor Model:\n");
    printf("    Km (velocity const) = %.2f mm/s/V\n", km);
    printf("    Tm (time const)     = %.5f s\n", tm);
    printf("  Feedforward:\n");
    printf("    bias_ff  = %.5f V\n", bias_ff);
    printf("    speed_ff = %.7f V/(mm/s)\n", speed_ff);
    printf("    acc_ff   = %.7f V/(mm/s^2)\n", acc_ff);
    printf("  Controller:\n");
    printf("    zeta = %.5f\n", zeta);
    printf("    Td   = %.5f s\n", td);
    printf("    Kp   = %.7f\n", kp);
    printf("    Kd   = %.7f\n", kd);
    printf("  Control flags: 0x%02X\n", control_flags);
}
