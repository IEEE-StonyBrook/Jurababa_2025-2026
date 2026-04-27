#include "driver_lab/settings.h"

#include <cstdio>

void DriverLabSettings::print() const
{
    printf("\n");
    printf("=== Settings ===\n");

    // Motor model
    printf("kM   = %8.2f  mm/s/V\n", kM);
    printf("Tm   = %8.5f  s\n", tm);
    printf("kS   = %8.4f  V\n", kS);

    // Per-motor (only show if they differ from combined)
    if (kM_L != kM || kM_R != kM || kS_L != kS || kS_R != kS)
    {
        printf("  L: kM=%.1f kS=%.4f  R: kM=%.1f kS=%.4f\n", kM_L, kS_L, kM_R, kS_R);
    }

    // Feedforward
    printf("kV   = %.7f  V/(mm/s)\n", kV);
    printf("kA   = %.7f  V/(mm/s^2)\n", kA);

    // Forward PD
    printf("zeta = %8.4f\n", zeta);
    printf("Td   = %8.5f  s\n", td);
    printf("kP   = %8.5f\n", kP);
    printf("kD   = %8.5f\n", kD);

    // Rotation PD
    printf("turnKP = %.4f\n", turnKP);
    printf("turnKD = %.4f\n", turnKD);
    printf("\n");
}
