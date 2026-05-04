# HC-05 USB ↔ UART Bridge

Tiny throwaway Pico firmware that turns the Pico into a USB-to-TTL serial
bridge for reconfiguring the HC-05 Bluetooth module's baud rate (or any
other persistent AT setting).

The main Jurababa firmware drives HC-05 at **115200 baud**, but HC-05 modules
ship at 9600. This utility lets you change that one-time setting without
buying a separate USB-TTL adapter.

## Wiring

No rewiring needed — the bridge uses the same pins as normal Jurababa
operation (`firmware/config/pins.h`):

| HC-05 Pin | Pico Pin                 |
| --------- | ------------------------ |
| VCC       | 5V (VBUS, pin 40) or 3.3V |
| GND       | GND                      |
| TXD       | GP13 (`PIN_BT_RX`)       |
| RXD       | GP12 (`PIN_BT_TX`)       |

## Build

Builds independently of the main firmware. Use the same `PICO_SDK_PATH`
you use for Jurababa.

```bash
cd tools/hc05_bridge
mkdir -p build && cd build
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk (e.g. /Users/lisulelvitigala/.pico-sdk/cmake/v3.31.5/bin/cmake)
make -j4
```

Output: `build/hc05_bridge.uf2`. Drag-and-drop onto the Pico in BOOTSEL mode,
or use `picotool load -f build/hc05_bridge.uf2`.

## Run / reprogram HC-05

1. With Pico **unplugged**, press and hold the small button on the HC-05
   module (DSD Tech boards have one near the EN pin).
2. While still holding the button, plug Pico USB into your computer.
3. Hold for ~2 more seconds, then release.
4. **Verify**: HC-05 LED should blink **slowly** (~1 Hz). That is full AT
   mode. If it blinks fast (~2 Hz), unplug and retry — timing was off.
5. Open a terminal to the Pico's USB CDC port at **115200 baud** with line
   endings set to **CR+LF** (Arduino Serial Monitor: "Both NL & CR").
6. Send each command and press Enter:

   ```
   AT
   ```
   Response: `OK` → bridge works.

   ```
   AT+UART=115200,0,0
   ```
   Response: `OK` → baud saved to EEPROM (args: baud, stop bits, parity).

   ```
   AT+UART?
   ```
   Response: `+UART:115200,0,0` then `OK` → confirmed.

7. Power down. Reflash the main Jurababa firmware (`cmake .. && make` from
   the repo root). Power up. HC-05 LED blinks fast (~2 Hz) until paired,
   slow when connected. Pair from your laptop at **115200 baud**.

## Why two different baud rates appear

- **HC-05 full AT mode is locked at 38400** in module firmware, regardless
  of what `AT+UART` is set to. This bridge runs UART0 at 38400.
- **The host terminal opens at 115200** because that is the USB CDC speed
  on the Pico side. USB CDC baud is independent of the UART baud — they
  only need to match at each physical link, not end-to-end.

## Troubleshooting

| Symptom                                       | Likely cause                                                   |
| --------------------------------------------- | -------------------------------------------------------------- |
| `AT` returns nothing                          | TX/RX swapped, missing CR+LF, or HC-05 not in full AT mode     |
| `AT` returns `OK`, `AT+UART=...` returns `ERROR` | HC-05 in *limited* AT mode — re-enter full AT mode (step 1–4) |
| Garbage characters on host terminal           | Host terminal not at 115200, or wrong line endings             |
| LED blinks fast even when button held         | Power applied before holding button — repeat from step 1       |

## Reusing for other AT commands

The bridge is a dumb pass-through, so any HC-05 AT command works. Useful ones:

```
AT+NAME=Jurababa          # change BT broadcast name
AT+PSWD=1234              # set pairing PIN
AT+ROLE=0                 # slave (default for talking to a phone/laptop)
AT+ORGL                   # factory reset
AT+VERSION?               # firmware version
```
