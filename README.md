## Spindle plugins

This plugin adds support for Huanyang VFD spindles via ModBus (RS485).

Switching between Modbus VDF spindle and PWM output is possible via a M-code, currently `M104`: `M104P0` for PWM (laser) and `M104P1` for VFD.  
Available when `\\#define DUAL_SPINDLE` is uncommented in _my_machine.h_ for drivers that has support.

__NOTE:__ `M104` does not change the `$32` setting for machine mode, the setting will still be used to select the spindle at startup.

---
2022-01-23
