# SU065-D4380 ROS2 Control Package
SU065-D4380 ROS2 Control Package for diff_drive_controllers.

## Parameters
- `dev` : Absolute path for the device
- `left_rotation_direction` : Left wheel rotation direction  `1/-1`.
- `right_rotation_direction` : Right wheel rotation direction `1/-1`.
- `reduction_rotation` : Mechanical reduction ratio for the wheel rotation.

#### Example

```xml
    <plugin>su065d4380_control/SU065D4380System</plugin>
    <param name="dev">${dev}</param>
```

## References.
- [ROS2 Control](https://control.ros.org/galactic/index.html)
