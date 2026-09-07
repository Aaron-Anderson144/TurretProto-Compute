# sim - closed loop harness

Empty on purpose.

Right now the plant runs in MATLAB and the ballistics runs in Rust and nothing
runs both. This closes the loop:

```
synthetic target track -> tracker -> aim() -> gimbal plant
   -> pointing error at trigger pull -> predicted miss distance at the target
```

Miss distance is the number that matters. Pointing RMS in degrees is a proxy.
Nothing in the project produces miss distance today, and it is what makes the
PID vs LADRC question decidable.

Also the home for the cross validation run: same scenario file through the
MATLAB controller and the Rust port, diff the traces.
