
# sawCopleyController

This SAW package contains a component (mtsCopleyController) that interfaces to a Copley Controller via a serial (RS232) interface. It has been tested with a Copley Xenus Plus single-axis controller, specifically XEC-230-15. The software has been written to support multi-axis controllers, but that feature has not been tested.

Most of the source code is in the [core](./core) subdirectory to facilitate building with ROS1 or ROS2.

The component is designed to be generic, and is configured using a JSON file, which can specify a CCX file for controller configuration. Sample JSON and CCX files are in the [core/share](./core/share) sub-directory.

Copley controllers are configured using parameters that are typically referred to by their hexadecimal addresses. Many parameters exist in both RAM and Flash. On power-up, or after reset, the controller copies the values from Flash to RAM. This component assumes that either the correct parameter values have previously been stored in Flash, or the correct values are available in a CCX file, which the component will parse during configuration and set the parameter values in Flash. The CCX file can be generated manually or by vendor tools. Documentation about the CCX file format can be found online.

The JSON file contains the following fields:

| Keyword      | Default   | Description                                     |
|:-------------|:----------|:------------------------------------------------|
| file_version | 0         | Version of JSON file format                     |
| name         |           | Descriptive name                                |
| port_name    |           | Serial port name (e.g., COM1)                   |
| baud_rate    | 9600      | Serial port baud rate                           |
| is_plus      | true      | Whether a Plus controller                       |
| ccx_file     | ""        | CCX file to configure controller                |
| axes         |           | Array of axis configuration data (see below)    |
|  - type      |           | - "PRISMATIC" or "REVOLUTE"                     |
|  - position_bits_to_SI |  | - conversion scale and offset (*)              |
|  -- scale    | 1         | -- scale factor                                 |
|  -- offset   | 0         | -- offset (currently not used)                  |
|  - home_pos  | 0         | - home position, in SI units (**)               |
|  - position_limits |     | - upper and lower joint position limits         |
|  -- lower    | -MAX      | -- lower position limit                         |
|  -- upper    | +MAX      | -- upper position limit                         |

(*) The conversion (position_bits_to_SI) is applied as follows:  value_SI = value_bits/scale.
(**) The home position is specified in the JSON file for convenience, and is used to set the home offset parameter (0xc6) on the drive; any setting of 0xc6 in the CCX file is ignored.
