# Lincoln MKZ Acceleration Interface

Code used to switch the Lincoln MKZ Autoware bridge from velocity commands to acceleration commands, and to test the effect on braking smoothness using recorded drives.

## Dataspeed interface

The bridge sends commands through Dataspeed's ULC interface (`ds_dbw_msgs/msg/UlcCmd.msg`).

- Original Dataspeed interface: <https://bitbucket.org/DataspeedInc/dbw_ros/src/ros2/>
- `UlcCmd.msg`: <https://bitbucket.org/DataspeedInc/dbw_ros/src/ros2/ds_dbw_msgs/msg/UlcCmd.msg>
- Installed version: on the vehicle.

## Vehicle-side code

**`bridge_braking_changes.patch`** - changes to the `universe_dbw2_bridge` package:

- `Lincoln_MKZ_Bridge.cpp`
  - Adds the `longitudinal_cmd_mode` parameter:
    - `velocity` (original): the Dataspeed ULC receives only a target speed.
    - `accel`: the ULC receives Autoware's computed acceleration (`CMD_ACCEL`), so Autoware's smooth-stop taper reaches the vehicle.
  - Replaces the hardcoded ULC limits (3 / 3 / 0 / 0) with parameters: `ulc_limit_accel`, `ulc_limit_decel`, `ulc_limit_jerk_throttle`, `ulc_limit_jerk_brake` (defaults 1.5 / 2.0 / 1.0 / 1.0).
  - Adds the `ulc_coast_decel` parameter.
  - All parameters can be changed at runtime with `ros2 param set`. Invalid values are rejected, including limits ≤ 0.
- `vehicle_params.yaml` - default values, with `longitudinal_cmd_mode: "accel"`.

**`apply_accel_mode.sh`** - applies the change on the vehicle computer:

1. Finds the Autoware workspace and the bridge package.
2. Skips the edits if they are already applied.
3. Backs up the files it modifies.
4. Applies the code and config changes.
5. Rebuilds the package with `colcon`.

```bash
./apply_accel_mode.sh
```

Verify accel mode (expect `cmd_type: 2`):

```bash
ros2 param get /Lincoln_MKZ_Bridge longitudinal_cmd_mode
ros2 topic echo /vehicle/ulc/cmd --field cmd_type
```

Revert without rebuilding:

```bash
ros2 param set /Lincoln_MKZ_Bridge longitudinal_cmd_mode velocity
```

## Analysis scripts

| Script | Purpose |
|---|---|
| `dump_bag.py` | Converts a ROS 2 bag (`.db3`) into one CSV per topic. |
| `analyze.py` | First-pass analysis of one run: ULC limits sent, brake mode, deceleration events, jerk statistics and a time trace of a stop. |
| `quant_check.py` | Checks the quantization of the ULC's `accel_meas`. One step equals about 2.5 m/s³ of apparent jerk, so jerk is derived from velocity instead. |
| `compare2.py` | Compares baseline and Stage 1 runs stop by stop: peak deceleration, jerk, release jerk, brake torque and brake chatter. |
| `smoothstop_check.py` | Compares Autoware's commanded acceleration with the vehicle's actual acceleration before each stop. |
| `make_figures.py` | Generates the six report figures. |

## Usage

Requires ROS 2 Humble for `dump_bag.py`, and Python 3 with `numpy` and `matplotlib` for the other scripts.

```bash
python3 analysis_scripts/dump_bag.py <bag_dir> extracted_csv/csv_<run>
python3 analysis_scripts/analyze.py extracted_csv/csv_<run>
python3 analysis_scripts/quant_check.py extracted_csv/csv_<run> [...]
python3 analysis_scripts/compare2.py extracted_csv/csv_<baseline> extracted_csv/csv_<stage1>
python3 analysis_scripts/smoothstop_check.py extracted_csv/csv_<run>
python3 analysis_scripts/make_figures.py
```

`make_figures.py` reads the CSVs from fixed paths under `extracted_csv/` and writes the images to `figures/`.
