# Safety and Estop

## Where safety is enforced

Safety monitoring is implemented in the low command handling layer and continuously checks:

- Position bounds
- Velocity bounds
- Torque bounds

On violation, estop is called and command modes are dropped.

## Operator recommendations

- Start with conservative goals and small workspace changes.
- Keep a physical stop strategy active during first tests.
- Do not run full-speed hand/arm motions before state/topic validation.
- Verify no stale command sources are publishing in parallel.

## Typical safe sequence

1. Bring up TF and joint state only.
2. Validate left/right EE pose streams.
3. Send one named config move.
4. Send one small Cartesian move.
5. Enable hand commands.

## Recovery after estop

1. Stop all command clients.
2. Inspect logs for violating joint and value.
3. Reinitialize node graph/launch.
4. Reissue conservative command.
