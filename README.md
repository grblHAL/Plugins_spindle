## Spindle plugins

This plugin adds support for VFD spindles via ModBus (RS485).  Runtime switching of VFD model is currently under the Modbus settings in IOSender.  Hard reset required after saving settings.

Currentlys supported/tested VFDs:

    Huanyang model 1 and model P2A
    Yalang YL620
    Durapulse GS20

Hard reset is required when changing VFD models.

This plugin also adds the MODVFD mode where you can use different register values for the control and RPM functions.  The functionality is similar to the VFDMOD component from LinuxCNC.

Switching between Modbus VFD spindle and PWM output is possible via a M-code, currently `M104`: `M104P0` for PWM (laser) and `M104P1` for VFD.  
Available when `\\#define DUAL_SPINDLE` is uncommented in _my_machine.h_ for drivers that has support.

__NOTE:__ `M104` does not change the `$32` setting for machine mode, the setting will still be used to select the spindle at startup.

__NOTE:__ You will want to add the following lines to settings.h in the grbl submodule around line 258:

    #ifdef VFD_ENABLE
    Setting_VFD_TYPE = 460, // Select from available VFD types
    Setting_VFD_RPM_HZ = 461, // Set RPM/Hz (not used by all VFD types)
    Setting_VFD_PLUGIN_10 = 462,
    Setting_VFD_PLUGIN_11 = 463,
    Setting_VFD_PLUGIN_12 = 464,
    Setting_VFD_PLUGIN_13 = 465,
    Setting_VFD_PLUGIN_14 = 466,
    Setting_VFD_PLUGIN_15 = 467,
    Setting_VFD_PLUGIN_16 = 468,
    Setting_VFD_PLUGIN_17 = 469,
    Setting_VFD_PLUGIN_18 = 470,
    Setting_VFD_PLUGIN_19 = 471,
    Setting_VFD_PLUGIN_20 = 472,
    Setting_VFD_PLUGIN_21 = 473,         
    #endif
---


2022-04-15
