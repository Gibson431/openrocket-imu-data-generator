# OpenRocket IMU Data Generation

**Warning**: This uses the newer plugin API. You will need to build OpenRocket from source to use this.
See Compatibility section for more information.

An [OpenRocket](https://github.com/openrocket/openrocket) simulation extension
that logs mock IMU sensor data to CSV, for use as synthetic input when testing
inertial navigation system (INS) algorithms.

Each simulation run produces its own `<simulation name>-NNN/` directory (named after the
OpenRocket simulation that produced it) containing:

- **`data.csv`**
  ```
  time_s, pos_x_m, pos_y_m, pos_z_m,
  accel_body_x_mss, accel_body_y_mss, accel_body_z_mss,
  gyro_roll_rads, gyro_pitch_rads, gyro_yaw_rads,
  truth_quat_w, truth_quat_x, truth_quat_y, truth_quat_z
  ```
  - **Position** is absolute, in OpenRocket's launch-frame Cartesian coordinates (meters).
  - **Accelerometer** is reconstructed as specific force (what a real accelerometer reads),
    not raw kinematic acceleration.
  - **Gyro** channels are body-frame roll/pitch/yaw rates (rad/s).
  - **`truth_quat_*`** is the rocket's true orientation quaternion - ground truth for scoring
    an INS's attitude estimate, not a simulated sensor reading.

- **`events.csv`**
  ```
  time_s, event
  ```
  Simulation flight events (e.g. `LAUNCH`, `BURNOUT`, `APOGEE`, `RECOVERY_DEVICE_DEPLOYMENT`),
  excluding the continuous `ALTITUDE` event.

## Installing

Copy the built jar into OpenRocket's plugin directory for your OS:

| OS      | Plugins directory                                          |
|---------|-------------------------------------------------------------|
| Linux   | `~/.openrocket/Plugins/`                                     |
| macOS   | `~/Library/Application Support/OpenRocket/Plugins/`          |
| Windows | `%APPDATA%\OpenRocket\Plugins`                                |

Restart OpenRocket, then in a simulation's edit dialog add the extension under
**Reports → IMU Data Generator (CSV)**.

### Compatibility

This plugin requires you to build OpenRocket from unstable later than 9 September 2026 as it uses some APIs
that have not been stabilised yet.

## Building

This plugin uses `CoordinateIF` and the `TYPE_ACCELERATION_BODY*` flight data types, which
are not yet in a released version of OpenRocket core on Maven Central (as of `24.12`) - they
currently only exist on OpenRocket's `unstable` development branch. Until a release ships
with them, build against a locally-published snapshot of that branch:

```
# From an OpenRocket checkout on the unstable branch:
./gradlew :core:publishToMavenLocal

# Then, from this repo:
./gradlew jar
```

Once a release containing those APIs ships, update the `core` version in `build.gradle` to
that release and drop the `mavenLocal()` repository.

The plugin jar is compiled against OpenRocket core as a `compileOnly` dependency - it is
**not bundled** into the jar, since OpenRocket supplies it at runtime. Note that `core`
declares its own dependencies (like slf4j) as `implementation`, so they don't transitively
reach a consumer's compile classpath - this project declares `slf4j-api` directly for that
reason.
