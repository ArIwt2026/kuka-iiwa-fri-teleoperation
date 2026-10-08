# FRI Cartesian impedance and joint walls

This package contains an opt-in FRI 1.15 torque client. Monitoring remains the default.
The controller is experimental operating software, not a safety-rated joint-limit system.

## Control law

The PC sends an overlay torque composed of:

```
tau_overlay = J(q)^T (K pose_error - D J(q) dq) + tau_wall(q, dq)
```

The Cartesian reference is captured from the measured TCP pose whenever FRI enters active
commanding. Joint walls are added directly, rather than projected into the Jacobian nullspace.
When a Cartesian torque conflicts with an active wall, `outward_task_scale` controls how much of
that conflicting task torque remains. The default is zero, giving the wall priority. A newly
active wall may cut a previously rate-limited outward command directly to zero; inward wall
torque then follows the configured per-joint slew rate.

The default configuration intentionally sets all six Cartesian stiffness and damping values to
zero. It therefore behaves like the Panda zero-impedance controller: model compensation from the
robot controller plus seven independently configured joint-wall springs. The supplied wall
stiffness is 500 Nm/rad for A1--A4 and 50 Nm/rad for A5--A7. Non-zero Cartesian gains are
supported by the PC controller but are not enabled by the supplied configuration.

The Sunrise joint-impedance controller is expected to provide robot-model compensation. This
client therefore does not add estimated gravity or Coriolis torque. That behavior must be
confirmed on the installed cabinet before commissioning.

## Components

- `cartesian_wall_controller.hpp`: hardware-independent controller and validation.
- `fri_cartesian_wall_client.cpp`: sole FRI connection owner in control mode.
- `config/cartesian_wall.yaml`: iiwa7 configuration and zero Cartesian gains by default.
- `sunrise/FRICartesianWall.java`: cabinet-side FRI session and overlay-motion owner.
- `test/test_cartesian_wall_controller.cpp`: offline controller tests.

The client preserves `/iiwa7/joint_states` and `/fri/mediaflange_output1/toggle`. The first valid
request asks Sunrise to start and the next asks it to stop. STOP immediately disables PC torque
authorization and is never subject to the Sunrise start lockout. The service acknowledges the
requested IO change; the FRI state transition remains the control-mode confirmation.

Before activation, both sides independently reject a start when any joint is within the 10-degree
wall plus the configured 1-degree startup clearance. Unexpected commanding-state loss is fault
latched, stale feedback is no longer published as fresh, and another explicit START is required.

## Launch selection

The integrated KUKA workcell now uses the ROS 2 hardware-control launch, which starts the
`admittance_controller` by default:

```
ros2 launch lbr_bringup hardware_iiwa7_workcell.launch.py ctrl:=admittance_controller
```

The former `fri_monitor_rviz.launch.py` monitor-only launch has been removed. This workcell
launch does not start the separate experimental Cartesian Wall torque client described here.
Do not run that client or its Sunrise application alongside another arm-motion owner.

## Required cabinet verification

Before any hardware run, verify all of the following in Sunrise Workbench and on the controller:

1. FRI and client SDK are both compatible with 1.15.
2. `FRICartesianWall.java` compiles without API substitutions.
3. The actual tool/load and `lbr_link_ee` TCP agree with the URDF used by the PC.
4. Torque overlay uses `ClientCommandMode.TORQUE` over zero-stiffness, zero-damping joint
   impedance. This is required by FRI even when the PC control law is Cartesian.
5. KUKA gravity, Coriolis and friction compensation remain active with a zero overlay.
6. Output1 is registered for FRI and its initial level starts no motion.
7. Cabinet safety configuration and collision protections remain unchanged.
8. The PC URDF chain `lbr_link_0` to `lbr_link_ee` matches the commissioned TCP.

Commission in T1 with approved supervision: monitoring first, state transitions second, zero
overlay third, walls at artificial boundaries fourth, and only then reviewed Cartesian gains.
