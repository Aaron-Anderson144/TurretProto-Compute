# fcs - fire control

Empty on purpose. This is the missing middle: the thing that turns a target
track into a gimbal setpoint.

Half the problem is now solved on the ballistics side. `ballistics_core::aim`
takes a target's position and velocity in east/north/up and returns gun bearing,
elevation, time of flight and the slew rates for feed forward. So what is left
here is the plumbing either side of it.

Planned contents:

- the tracker interface (IMM-EKF target state in a stabilized frame), which has
  to produce `aim::Target`
- the `AimPoint` struct the controller consumes
- the loop: tracker -> `aim_hinted()` (warm started off last frame) -> AimPoint
- arming and inhibit logic, with its own tests

Define `AimPoint` first. Once that struct exists the tracker side and the
controller side can be built independently.

```
AimPoint {
    az, el,               // rad, gimbal setpoint    <- Solution.bearing / .elevation
    az_rate, el_rate,     // rad/s, feed forward     <- Rates.bearing / .elevation
    tof,                  // s                       <- Solution.tof
    valid,                // firing solution good    <- Solution.status.is_converged()
}
```

The mapping is one for one, which is the point. `CascadeController.step` already
takes a rate feed forward argument and it has been fed from a hand written
scenario script; `az_rate` and `el_rate` are what belong there.
