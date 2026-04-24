#include "motor_lab/settings.h"

#include <cstdio>

void MotorLabSettings::print() const
{
    printf("MotorLab Settings:\n");
    printf("  Motor Model:\n");
    printf("    kM (velocity const) = %.2f mm/s/V\n", kM);
    printf("    Tm (time const)     = %.5f s\n", tm);
    printf("  Feedforward:\n");
    printf("    kS = %.5f V\n", kS);
    printf("    kV = %.7f V/(mm/s)\n", kV);
    printf("    kA = %.7f V/(mm/s^2)\n", kA);
    printf("  Controller:\n");
    printf("    zeta = %.5f\n", zeta);
    printf("    Td   = %.5f s\n", td);
    printf("    kP   = %.7f\n", kP);
    printf("    kD   = %.7f\n", kD);
    printf("  Control flags: 0x%02X\n", control_flags);
}
