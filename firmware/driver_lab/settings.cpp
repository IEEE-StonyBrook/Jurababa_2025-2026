#include "driver_lab/settings.h"

#include <cstdio>

void DriverLabSettings::print() const
{
    printf("\n");
    printf("=== DriverLab Settings ===\n");
    printf("\n");

    printf("--- Motor Model (Combined) ---\n");
    printf("  kM = %8.2f mm/s/V    (velocity constant)\n", kM);
    printf("  kS = %8.4f V         (static friction)\n", kS);
    printf("  Tm = %8.5f s         (time constant)\n", tm);
    printf("\n");

    printf("--- Motor Model (Per-Motor) ---\n");
    printf("  kM_L = %8.2f mm/s/V  kM_R = %8.2f mm/s/V\n", kM_L, kM_R);
    printf("  kS_L = %8.4f V       kS_R = %8.4f V\n", kS_L, kS_R);
    printf("\n");

    printf("--- Feedforward ---\n");
    printf("  kV = %.7f V/(mm/s)    (speed FF = 1/kM)\n", kV);
    printf("  kA = %.7f V/(mm/s^2)  (accel FF = Tm/kM)\n", kA);
    printf("\n");

    printf("--- Forward PD Controller ---\n");
    printf("  kP   = %.5f\n", kP);
    printf("  kD   = %.5f\n", kD);
    printf("  zeta = %.4f           (damping ratio)\n", zeta);
    printf("  Td   = %.5f s         (derivative time)\n", td);
    printf("\n");

    printf("--- Rotation PD Controller ---\n");
    printf("  turnKP = %.4f\n", turnKP);
    printf("  turnKD = %.4f\n", turnKD);
    printf("\n");

    printf("--- Control Mode ---\n");
    printf("  flags = 0x%02X", control_flags);
    if (control_flags & 0x01)
        printf(" [FF]");
    if (control_flags & 0x02)
        printf(" [PD]");
    if (control_flags == 0)
        printf(" [none]");
    printf("\n\n");
}
