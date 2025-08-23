# Custom Swerve Auto Path Planner

## Pre-Defined
- Robot pose (`x`, `y`, `heading`) is provided by Pinpoint.
- Full swerve drive control (angle + velocity per module) is available.
- No reliance on RoadRunner or other libraries.

---

## Data Structures

### Path Representation
- [ ] `Waypoint`: Position (`x`, `y`), optional heading, velocity target.
- [ ] `Path`: List of waypoints + interpolation method (e.g. linear, spline, bezier).
- [ ] `Trajectory`: Interpolated path with timestamps, headings, and speeds.

---

## Path Following Logic

### Path Following Controller
- [ ] Accepts current pose from Pinpoint.
- [ ] Computes target position and heading on the path based on time or distance.
- [ ] Outputs desired robot velocity vector and angular velocity.

### Error Calculation
- [ ] Position error (`x`, `y`)
- [ ] Heading error
- [ ] Lateral + longitudinal error if using a path-relative frame

### Control Output
- [ ] Feed path follower output into drivetrain's `swerveDirectionalInput(x, y, rX)` or a new method
- [ ] Optionally apply PID to reduce cross-track error or heading error

---

## Tuning + Safety

- [ ] Max velocity, acceleration, and jerk limits
- [ ] Stop or hold pose at end of path
- [ ] Dynamic replan support (future)

---

## Future Features
- [ ] Action markers or temporal callbacks
- [ ] Path blending / chaining
- [ ] Reversing / mirrored paths

# Example


