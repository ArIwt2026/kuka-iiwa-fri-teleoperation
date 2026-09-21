# Sunrise application for the FRI Cartesian-wall controller

`FRICartesianWall.java` is the cabinet-side motion/session owner. It is intentionally not
compiled by the ROS workspace.

Before commissioning:

1. Copy the Java file into the Sunrise project's `application` package.
2. Set `REMOTE_HOST` to the control computer's FRI address and confirm FRI 1.15 compatibility.
3. Confirm that the generated `MediaFlangeIOGroup` contains `Output1` and that
   `registerIOHardware(mediaFlange)` is available in the installed Sunrise FRI API.
4. Configure the actual tool/load and validate it on the cabinet.
5. Compile in Sunrise Workbench. Do not remove compile errors by guessing API substitutions.

The application connects in monitoring mode and starts no arm motion until a new Output1 edge.
Before each start it requires 25 stationary samples and every joint to remain more than 11 degrees
inside both robot-reported model limits. A stop edge is never blocked by the start lockout.
It starts one zero-stiffness joint-impedance position hold with a TORQUE FRI overlay. The PC-side
client computes Cartesian impedance and joint-wall overlay torques. A subsequent edge cancels the
motion and waits for confirmed completion while leaving the FRI session available for monitoring.

This is experimental control software, not a safety-rated joint-limit mechanism.
