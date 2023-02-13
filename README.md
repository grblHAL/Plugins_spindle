## Spindle plugins

This plugin adds support for some VFD spindles via ModBus (RS485) and a spindle selection plugin.

### VFD spindles

VFD spindle support is enabled in _my_machine.h_ by uncommenting `\\#define VFD_SPINDLE` and changing the VFD spindle number to the desired type or `-1` for all.  
A list of supported VFDs and associated spindle numbers can be found [here](./shared.h).

If all spindles or dual spindle is enabled the active spindle is configured by setting `$395`.  
Note that the setting value not the same as the VFD spindle number used in _my_machine.h_, either use a sender that supports grblHAL setting enumerations
 for configuration or output a list with the `$$=395` command to find the correct setting value.

If a single VFD spindle is added to the firmware its ModBus address is configured by setting `$360` \(default 1\).  
If more than one then the ModBus address is set by one of the following settings depending on the _spindle number_ it is assigned to:

`$476` - ModBus address of VFD bound to spindle 0, default 1. Available when spindle 0 is configured as a VFD spindle by `$395`.  
`$477` - ModBus address of VFD bound to spindle 1, default 2. Available when spindle 1 is configured as a VFD spindle by `$511`.  
`$478` - ModBus address of VFD bound to spindle 2, default 3. Available when spindle 2 is configured as a VFD spindle by `$512`.  
`$479` - ModBus address of VFD bound to spindle 4, default 4. Available when spindle 3 is configured as a VFD spindle by `$513`.

#### GS20 and YL-620

Setting `$461` can be used to set the RPM to HZ relationship. Default value is `60`.

#### MODVFD

The MODVFD spindle uses different register values for the control and RPM functions. The functionality is similar to
the [VFDMOD](https://github.com/aekhv/vfdmod)component from LinuxCNC. The following settings are available:

`$462` - Register for Run/stop, default value is `8192`.  
`$463` - Set Frequency Register, default value is `8193`.  
`$464` - Get Frequency Register, default value is `8451`.  
`$465` - Command word for CW, default value is `18`.  
`$466` - Command word for CCW, default value is `34`.  
`$467` - Command word for stop, default value is `1`.  
`$468` - RPM value multiplier for programming RPM, default value is `50`.  
`$469` - RPM value divider for programming RPM, default value is `60`.  
`$470` - RPM value multiplier for reading RPM, default value is `60`.  
`$471` - RPM value divider for reading RPM, default value is `100`.  

__NOTE:__ settings for ModBus addresses requires a hard reset after changing spindle binding settings \(see below\) before becoming available.

___

### Additional spindles

Additional spindles may be added by plugin code. If of a generic kind they might be added to this repo based on a pull request.

---

### Spindle selection

grblHAL can operate in three different spindle modes depending on compile time configuration:

* Only a single spindle is allowed.  
None of the settings or the M-code documented below is available.

* Up to 32<sup>1</sup> spindles can be added but only one of up to four<sup>1</sup> can be active at a time.
Spindle switching is by tool number \(see below\) or `M104`:  
Use `M104P0` and `M104P1` to set the active spindle as _spindle number_ 0 or 1 respectively.  
Use `M104Q<n>` where `<n>` is the _spindle number_ to set as the active spindle.

* Up to 32 spindles can be added and up to four can be active at a time.  
The spindle to control is adressed by the `$` gcode word followed by the spindle number,
available for the `S`, `M3`, `M4`, `M5`, `M51`, `G33`, `G76`, `G96` and `G97` gcode commands.  
Spindle switching by tool number or `M104` is not available in this mode.  
__NOTE:__ This mode is work in progress and functionality is not yet complete!

The spindles are dynamically assigned a _spindle id_ at registration, starting from 0. $-settings are then used to tell grblHAL which are to be enabled,
by _spindle number_.

`$395` - bind _spindle number_ 0 to the given _spindle id_. This setting is available when two or more spindles are added.  
`$511` - bind _spindle number_ 1 to the given _spindle id_. This setting is available when two or more spindles are added.  
`$512` - bind _spindle number_ 2 to the given _spindle id_. This setting is available when three or more spindles are added.  
`$513` - bind _spindle number_ 3 to the given _spindle id_. This setting is available when four or more spindles are added.  

Available spindles with _spindle id_, and _spindle number_ if bound, can be listed by the `$spindles` command. `$$=<n>` where `<n>` is a setting number above can be used for listing allowed values.

__NOTES:__ `$395` defaults to _spindle id_ 0, this is normally the driver provided PWM spindle but can be another spindle if only one spindle can be registered. Spindle 0 cannot be disabled.

If the grblHAL is configured to handle only one active spindle at a time then $-settings can be used to assign a range of tool numbers to
each spindle number. The spindle is then activated on a `M6T<n>` command where `<n>` is the tool number.

`$520` - lowest tool number for selecting spindle 0, ignored if 0 - normally leave at 0. If set > 0 tool numbers lower than the value will not cause a spindle change.  
`$521` - lowest tool number for selecting spindle 1, ignored if 0.  
`$522` - lowest tool number for selecting spindle 2, ignored if 0.  
`$523` - lowest tool number for selecting spindle 3, ignored if 0.

Tool number vs. spindle is checked from the highest to the lowest spindle number, if all are set to 0 no spindle change takes place.

__NOTE:__ settings for tool number assignments requires a hard reset after changing spindle enable settings before becoming available.  
__NOTE:__ when switching between spindles any offset between the spindles must be handled by gcode commands, typically by applying an offset and moving
the controlled point \(tooltip\) to the required position.

<sup>1</sup> These numbers are defined by compile time configuration and can be lower.

---

__NOTE:__ if laser mode is enabled by the `$32` setting it will only be honoured if the current spindle is a PWM spindle capable of laser mode.  

---
2023-02-13
