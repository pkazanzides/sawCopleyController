# Sample configuration files

The mtsCopleyController component should be initialized by calling the `Configure` method, with the appropriate JSON file as a parameter. The JSON file specifies the name of the corresponding CCX file, which sets the values of parameters in controller RAM.

The JSON file format is documented in the top-level [README](/README.md).

The CCX file format documentation can be found online. The first line is the version number (14 in these examples) and the second line is the number of axes (1 in these examples). Subsequent lines are formatted as follows:

| parameter id (hex) | axis number | parameter name | value(s) |
|:-------------------|:------------|:---------------|:---------|
|                    |             |                |          |

The value field is usually a single decimal integer, but could contain multiple values (separated by `:`), or strings. Note that not all parameters in the CCX file are supported by the controller -- some of them are intended for use by software on the PC. The mtsCopleyController component  maintains a table of parameters that it will use; other parameters are ignored.
