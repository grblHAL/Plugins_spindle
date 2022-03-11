## Spindle plugins

Added Durapulse GS20 and Yalang YL620A support.  Hard reset is required when changing VFD models.

This plugin adds support for VFD spindles via ModBus (RS485).

Switching between Modbus VFD spindle and PWM output is possible via a M-code, currently `M104`: `M104P0` for PWM (laser) and `M104P1` for VFD.  
Available when `\\#define DUAL_SPINDLE` is uncommented in _my_machine.h_ for drivers that has support.

__NOTE:__ `M104` does not change the `$32` setting for machine mode, the setting will still be used to select the spindle at startup.

__NOTE:__ You will want to add the following two lines to settings.h in the grbl submodule around line 258:

    Setting_VFD_TYPE = 395, // Select from available VFD types
    Setting_VFD_RPM_HZ = 396, // Set RPM/Hz (not used by all VFD types)
      
---
2022-01-23
