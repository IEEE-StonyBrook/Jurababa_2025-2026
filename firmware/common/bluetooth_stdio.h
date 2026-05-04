#ifndef COMMON_BLUETOOTH_STDIO_H
#define COMMON_BLUETOOTH_STDIO_H

#include <cstdint>

namespace BluetoothStdio
{
struct Diagnostics
{
    uint32_t ring_size;
    uint32_t depth;
    uint32_t max_depth;
    uint32_t dropped_bytes;
    uint32_t suppressed_csv_lines;
    uint32_t csv_interval_ms;
};

void        setCsvIntervalMs(uint32_t interval_ms);
uint32_t    csvIntervalMs();
Diagnostics diagnostics();
void        resetDiagnostics();
} // namespace BluetoothStdio

#endif
