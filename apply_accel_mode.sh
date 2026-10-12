#!/usr/bin/env bash
set -euo pipefail

PKG_REL="src/vehicle/external/universe_dbw2_bridge"

say()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
fail() { printf '\n\033[31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

WS="$PWD"
while [ "$WS" != "/" ] && [ ! -d "$WS/$PKG_REL" ]; do WS="$(dirname "$WS")"; done
[ -d "$WS/$PKG_REL" ] || fail "could not find $PKG_REL above $PWD -- cd into the Autoware workspace and rerun"
PKG="$WS/$PKG_REL"
say "workspace: $WS"

CPP="$PKG/src/Lincoln_MKZ_Bridge.cpp"
YAML="$PKG/config/vehicle_params.yaml"
[ -f "$CPP" ]  || fail "missing $CPP"
[ -f "$YAML" ] || fail "missing $YAML"

if grep -q 'longitudinal_cmd_mode' "$CPP"; then
  say "already patched -- source already has longitudinal_cmd_mode; skipping to build"
  SKIP_PATCH=1
else
  SKIP_PATCH=0
fi

if [ "$SKIP_PATCH" = "0" ]; then
  STAMP="$(date +%Y%m%d_%H%M%S)"
  cp "$CPP"  "$CPP.bak_$STAMP"
  cp "$YAML" "$YAML.bak_$STAMP"
  say "backed up to *.bak_$STAMP"
fi

if [ "$SKIP_PATCH" = "0" ]; then
  say "applying patch"

  python3 - "$CPP" <<'PYEOF'
import re, sys
p = sys.argv[1]
s = open(p).read()
orig = s

def sub1(pat, rep, what, count=1):
    global s
    s2, n = re.subn(pat, rep, s, count=count)
    if n != count:
        sys.exit("PATCH FAILED (%s): expected %d match, found %d.\n"
                 "The source has drifted from the expected baseline -- apply "
                 "bridge_braking_changes.patch by hand." % (what, count, n))
    s = s2

sub1(r'#include "rclcpp/rclcpp\.hpp"',
     '#include <cmath>\n#include <string>\n#include <vector>\n\n'
     '#include "rclcpp/rclcpp.hpp"\n'
     '#include "rcl_interfaces/msg/set_parameters_result.hpp"',
     "includes")

sub1(r'(steering_gain = get_parameter\("Lincoln_Steering"\)\.as_double\(\);\n)',
     r'''\1
    declare_parameter<double>("ulc_limit_accel", 1.5);
    declare_parameter<double>("ulc_limit_decel", 2.0);
    declare_parameter<double>("ulc_limit_jerk_throttle", 1.0);
    declare_parameter<double>("ulc_limit_jerk_brake", 1.0);
    declare_parameter<bool>("ulc_coast_decel", false);

    declare_parameter<std::string>("longitudinal_cmd_mode", "velocity");

    ulc_limit_accel_ = get_parameter("ulc_limit_accel").as_double();
    ulc_limit_decel_ = get_parameter("ulc_limit_decel").as_double();
    ulc_limit_jerk_throttle_ = get_parameter("ulc_limit_jerk_throttle").as_double();
    ulc_limit_jerk_brake_ = get_parameter("ulc_limit_jerk_brake").as_double();
    ulc_coast_decel_ = get_parameter("ulc_coast_decel").as_bool();
    longitudinal_cmd_accel_ =
        (get_parameter("longitudinal_cmd_mode").as_string() == "accel");

    logUlcLimits();

    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&Lincoln_MKZ_Bridge::onParameterChange, this, _1));
''',
     "parameter declarations")

sub1(r'(  double curr_steer\{0\.0\};\n)',
     r'''\1
  double ulc_limit_accel_{1.5};
  double ulc_limit_decel_{2.0};
  double ulc_limit_jerk_throttle_{1.0};
  double ulc_limit_jerk_brake_{1.0};
  bool ulc_coast_decel_{false};
  bool longitudinal_cmd_accel_{false};

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  void logUlcLimits()
  {
    RCLCPP_INFO(get_logger(),
                "ULC limits: accel=%.2f m/ss, decel=%.2f m/ss, "
                "jerk_throttle=%.2f m/sss, jerk_brake=%.2f m/sss, coast_decel=%s, "
                "longitudinal_cmd_mode=%s",
                ulc_limit_accel_, ulc_limit_decel_,
                ulc_limit_jerk_throttle_, ulc_limit_jerk_brake_,
                ulc_coast_decel_ ? "true" : "false",
                longitudinal_cmd_accel_ ? "ACCEL" : "velocity");
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
      const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "ulc_coast_decel") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = name + " must be a bool";
          return result;
        }
        continue;
      }
      if (name == "longitudinal_cmd_mode") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = name + " must be a string";
          return result;
        }
        const std::string m = p.as_string();
        if (m != "velocity" && m != "accel") {
          result.successful = false;
          result.reason = name + " must be \"velocity\" or \"accel\"";
          return result;
        }
        continue;
      }
      if (name != "ulc_limit_accel" && name != "ulc_limit_decel" &&
          name != "ulc_limit_jerk_throttle" && name != "ulc_limit_jerk_brake") {
        continue;
      }
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = name + " must be a double";
        return result;
      }
      const double v = p.as_double();
      if (!(v > 0.0) || !std::isfinite(v)) {
        result.successful = false;
        result.reason = name + " must be finite and greater than 0";
        return result;
      }
    }

    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "ulc_limit_accel") {
        ulc_limit_accel_ = p.as_double();
      } else if (name == "ulc_limit_decel") {
        ulc_limit_decel_ = p.as_double();
      } else if (name == "ulc_limit_jerk_throttle") {
        ulc_limit_jerk_throttle_ = p.as_double();
      } else if (name == "ulc_limit_jerk_brake") {
        ulc_limit_jerk_brake_ = p.as_double();
      } else if (name == "ulc_coast_decel") {
        ulc_coast_decel_ = p.as_bool();
      } else if (name == "longitudinal_cmd_mode") {
        longitudinal_cmd_accel_ = (p.as_string() == "accel");
      }
    }

    logUlcLimits();
    return result;
  }
''',
     "member variables and callback")

sub1(r'    ulc_cmd\.cmd_type = ds_dbw_msgs::msg::UlcCmd::CMD_VELOCITY;\n'
     r'    ulc_cmd\.cmd = msg->longitudinal\.velocity;\n',
     '''    if (longitudinal_cmd_accel_) {
      ulc_cmd.cmd_type = ds_dbw_msgs::msg::UlcCmd::CMD_ACCEL;
      ulc_cmd.cmd = msg->longitudinal.acceleration;
    } else {
      ulc_cmd.cmd_type = ds_dbw_msgs::msg::UlcCmd::CMD_VELOCITY;
      ulc_cmd.cmd = msg->longitudinal.velocity;
    }
''',
     "CMD_VELOCITY -> CMD_ACCEL")

sub1(r'    ulc_cmd\.limit_accel = 3;\n'
     r'    ulc_cmd\.limit_decel = 3;\n'
     r'    ulc_cmd\.limit_jerk_throttle = 0;\n'
     r'    ulc_cmd\.limit_jerk_brake = 0;\n',
     '    ulc_cmd.limit_accel = ulc_limit_accel_;\n'
     '    ulc_cmd.limit_decel = ulc_limit_decel_;\n'
     '    ulc_cmd.limit_jerk_throttle = ulc_limit_jerk_throttle_;\n'
     '    ulc_cmd.limit_jerk_brake = ulc_limit_jerk_brake_;\n',
     "ULC limits")

sub1(r'    ulc_cmd\.coast_decel = false;\n',
     '    ulc_cmd.coast_decel = ulc_coast_decel_;\n',
     "coast_decel")

open(p, 'w').write(s)
print("  patched %s" % p)
PYEOF
  cat >> "$YAML" <<'YAMLEOF'

ulc_limit_accel: 1.5
ulc_limit_decel: 2.0
ulc_limit_jerk_throttle: 1.0
ulc_limit_jerk_brake: 1.0

ulc_coast_decel: false

longitudinal_cmd_mode: "accel"
YAMLEOF

  echo "  patched $YAML"
fi

say "building universe_dbw2_bridge"
cd "$WS"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
colcon build --packages-select universe_dbw2_bridge \
             --cmake-args -DCMAKE_BUILD_TYPE=Release

cat <<'DONE'

===========================================================================
 DONE -- accel mode is now the default in vehicle_params.yaml.
 Restart the bridge for it to take effect.

 VERIFY (the only check that actually proves it -- expect cmd_type: 2)
   ros2 param get /Lincoln_MKZ_Bridge longitudinal_cmd_mode
   ros2 topic echo /vehicle/ulc/cmd --field cmd_type          (1=velocity, 2=accel)

 REVERT, no rebuild needed
   ros2 param set /Lincoln_MKZ_Bridge longitudinal_cmd_mode velocity

 OPEN EXPERIMENT -- jerk_brake 1.0 was sized for Stage 1 and now clamps
 Autoware's own command (measured: accel_ref topped out at -2.00 while
 Autoware asked for -2.32). Worth one run at:
   ros2 param set /Lincoln_MKZ_Bridge ulc_limit_jerk_brake 2.0

 To undo the source change entirely, restore the .bak_* files and rebuild.
===========================================================================
DONE
