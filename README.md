## Spindle plugins

Currentlys supported/tested VFDs:

    Huanyang model 1 and model P2A
    Yalang YL620
    Durapulse GS20

H100 support is currently WIP.

Settings $395 and $396 added to select the model and change rpm/hz as required.

Hard reset is required when changing VFD models.

This plugin adds support for VFD spindles via ModBus (RS485).  Runtime switching of VFD model is currently under the Modbus settings in IOSender.  Hard reset required after saving settings.

Switching between Modbus VFD spindle and PWM output is possible via a M-code, currently `M104`: `M104P0` for PWM (laser) and `M104P1` for VFD.  
Available when `\\#define DUAL_SPINDLE` is uncommented in _my_machine.h_ for drivers that has support.

__NOTE:__ `M104` does not change the `$32` setting for machine mode, the setting will still be used to select the spindle at startup.

__NOTE:__ You will want to add the following two lines to settings.h in the grbl submodule around line 258:

    Setting_VFD_TYPE = 395, // Select from available VFD types
    Setting_VFD_RPM_HZ = 396, // Set RPM/Hz (not used by all VFD types)
      
---

And add the following in modbus.h

    typedef struct {
        uint32_t baud_rate;
        uint32_t rx_timeout;
    #ifdef VFD_ENABLE
        uint32_t vfd_type;
        uint32_t vfd_rpm_hz;
    #endif
    } modbus_settings_t;

---


2022-01-23
