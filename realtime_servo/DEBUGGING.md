# WX200 Velocity-Control Debug Handbook

A top-to-bottom procedure for the symptoms we've been chasing: **oscillation**,
**failure to beat gravity while moving (x)**, **wrist curling in (y)**,
**tap-moves-but-hold-doesn't**, and **path drift**. Work the layers in order —
each one rules out everything above it, so by the end the fault is localized.

> TL;DR root cause so far: the gravity-loaded joints (shoulder, elbow,
> wrist_angle) can't track their commanded velocity *while moving*. Every other
> symptom is that one deficit surfacing through different parts of the pipeline.
> Confirm it with **Layer 5**, fix it at **Layer 6**.

---

## 0. Signal chain (know what feeds what)

```
keyboard_teleop  ──/velocity_pub/vel_command (RelativeMove)──►  cbf_filter ──► /wx200/cmd_vel
   (200 Hz, byte-driven)                                     effort_motor_watchdog ──► /wx200/cmd_vel
                                                                                          │
                                                                            (RelativeMove)│
                                                                                          ▼
                                                              jacobian_velctrl_node  (30 Hz control loop)
                                                                  │  output_mode = position
                                                                  ▼
                                              /wx200/arm_controller/joint_trajectory  (JTC)
                                                                  │
                                                                  ▼
                                              Dynamixel motors  (Position PID + FF, inner Vel PI)

joint feedback:  /wx200/joint_states ──► joint_state_sanitizer ──► /wx200/joint_states_clean
```

⚠️ **Two publishers on `/wx200/cmd_vel`** — both `cbf_filter` (launch:393) and
`effort_motor_watchdog` (launch:416) target it. Verify only one is actually
driving at a time (`ros2 topic info /wx200/cmd_vel -v`); two live publishers
interleave and look like random command dropouts. (See Layer 3.)

Key fact: messages are **`realtime_servo/msg/RelativeMove`** (dx,dy,dz,dtheta),
*not* `geometry_msgs/Twist`, even on the `cmd_vel`-named topics.

---

## 1. Parameter reference (current values + where they live)

| Param | Value | Location | Notes |
|---|---|---|---|
| Position_P_Gain (kp_pos) | 2200 | runtime service | XM430 default 800; high → oscillation |
| Position_I_Gain (ki_pos) | 0 | runtime service | no steady-state correction |
| Position_D_Gain (kd_pos) | 100 | runtime service | D/P ≈ 0.045 → underdamped |
| Feedforward_1st (k1) | 400 | runtime service | velocity FF; high → overshoot |
| Feedforward_2nd (k2) | 0 | runtime service | accel FF; unused |
| Velocity_P/I (kp_vel/ki_vel) | 100 / 1920 | runtime service | Dynamixel defaults (untouched) |
| `command_timeout_sec` | 0.4 | launch:511 | dead-man stop; **< OS repeat delay** |
| `alpha` (LPF) | 0.8 | launch:504 | command smoothing; ramps up only |
| `position_lead_clamp` | 0.05 rad | jacobian_velctrl.cpp:75 | setpoint may lead measured by ≤ this |
| `output_mode` | position | control_mode default (launch:71) | lead clamp is ACTIVE in this mode |
| `control_rate_hz` | 30 | launch:65 | loop dt ≈ 33 ms |
| `reseed_divergence_threshold` | 0.3 | jacobian_velctrl.cpp:81 | integrator reseed trigger |
| `singularity_threshold` | 0.05 | jacobian_velctrl.cpp:95 | damped pinv kicks in below |
| `joint_cbf_alpha` | 2.0 | launch:517 | joint-limit braking aggressiveness |
| OS key auto-repeat | 500 ms delay, 33 Hz | `xset q` | host keyboard setting |
| teleop `scale_by` | 0.1 | keyboard_teleop.cpp ctor | m per command |

Teleop keymap: `w:+z s:-z a:+x d:-x z:-y x:+y c:+dθ v:-dθ e:zero q:quit`.

---

## 2. Symptom → most likely layer

| Symptom | Start at |
|---|---|
| Oscillation / buzzing on a joint | Layer 6 (gains: D too low / k1 too high) |
| Sags or curls **only while moving** | Layer 5 → 6 (gravity tracking) |
| Sags **while holding still** | Layer 6 (needs ki_pos or more authority) |
| Tap moves, **hold doesn't** | Layer 4 (lead-clamp deadlock) + Layer 6 |
| Hold stutters then runs | Layer 4 (timeout < repeat delay) |
| Runs away after a single tap | Layer 4 (`command_timeout_sec` = 0 or too long) |
| Motor clicks off / overload LED | Layer 7 (current/effort limit) |
| "X command also drops Z", wrong joint moves | Layer 5 (KDL chain / joint order) |
| Random command dropouts | Layer 0 (two cmd_vel publishers) |

---

## 3. Layer 1 — Is a clean command being produced?

```bash
# Raw teleop output. Tap a key, then hold a key, watch the difference.
ros2 topic echo /velocity_pub/vel_command
ros2 topic hz   /velocity_pub/vel_command     # while HOLDING a key
```
- **Tap** → exactly one message, then silence (teleop publishes only on a key
  byte; there is no zero-on-release — keyboard_teleop.cpp:85 zero line is
  commented out).
- **Hold** → one message, a **~500 ms gap** (OS repeat delay), then a ~33 Hz
  stream. If you see the gap, that's the stutter source: the controller's
  `command_timeout_sec` (0.4 s) expires *inside* that gap.
- **No stream at all when holding** → auto-repeat isn't reaching the node (SSH /
  tmux / terminal). Then hold == single tap. Fix: `xset r rate 300 33` or
  redesign teleop to republish (Layer 8).

## 4. Layer 2 — Does the command survive the filter chain?

```bash
ros2 topic info /wx200/cmd_vel -v        # expect ONE publisher (see ⚠️ Layer 0)
ros2 topic echo /wx200/cmd_vel           # compare against /velocity_pub/vel_command
```
- cbf_filter and effort_watchdog both target `/wx200/cmd_vel`. If two publishers
  show, kill/disable one — interleaved publishing looks like erratic motion.
- If `cmd_vel` is zeroed unexpectedly, the **effort watchdog** may be tripping
  (Layer 7) or `enable_z_hold` is eating dz.

## 5. Layer 3 — Timeout / tap-vs-hold behavior

This layer is pure parameter interaction; no new measurement needed.
- `command_timeout_sec = 0.4` < OS repeat delay `0.5` → **holds stutter** at
  start, taps stop hard after 0.4 s.
- Fixes (pick one):
  - `command_timeout_sec: 0.6` (launch:511) → seamless holds, taps coast 0.6 s.
  - `xset r rate 300 33` → repeat starts before the 0.4 s timeout.
  - Best: republish in teleop (Layer 8) and decouple from OS repeat entirely.
- The stop is a **hard zero** (jacobian_velctrl.cpp:409-412), not a ramp-down —
  expect a sharper stop than start regardless.

---

## 6. Layer 4 — The lead-clamp deadlock (why hold can fully stall)

In `output_mode: position` the node integrates velocity into a position
setpoint, clamped to lead **measured** position by ≤ `position_lead_clamp`
(0.05 rad) — jacobian_velctrl.cpp:622-634.

- **Hold:** `qdot` stays > deadband; setpoint pinned at `q_meas + 0.05`. If a
  loaded joint can't beat gravity while moving, `q_meas` doesn't advance, the
  ceiling doesn't rise, the setpoint freezes ~0.05 rad ahead of a stuck motor.
  0.05 rad of error isn't enough torque to accelerate against gravity → stall.
- **Tap:** times out → `qdot ≈ 0` → neither clamp side active
  (`kQdotClampDeadband`, :568) → setpoint **freezes where it got to** instead of
  being pulled back. During the static pause the PID drives the motor up to the
  setpoint (gravity is fine when *stopped*), so the next tap has fresh lead room.
  Net: **tapping ratchets forward in ~0.05 rad bites; holding deadlocks.**

**Confirm the deadlock:**
```bash
# While HOLDING, watch a loaded joint. If commanded leads measured by ~0.05 rad
# and neither advances, that's the deadlock.
ros2 topic echo /wx200/joint_states                       # measured
ros2 topic echo /wx200/arm_controller/joint_trajectory    # commanded setpoint
```
**Diagnostic band-aid (NOT a fix):** temporarily raise `position_lead_clamp`
0.05 → 0.15. More lead = more position error = more torque. If hold then moves,
the deadlock is confirmed — but you're masking the PID deficit and risking
overshoot/runaway. Revert after confirming.

## 7. Layer 5 — Is the velocity solver healthy? (read the node log)

The node logs (throttled ~0.5 s, `RCLCPP_INFO_THROTTLE` :516):
```
cmd dx=.. dy=.. dz=.. | qdot {waist=.. shoulder=.. ...} | recovered=[..] | frozen={..} | cbf_scale=..
```
Healthy = `recovered ≈ cmd`, `cbf_scale=1.00`, `frozen={none}`. If so, the
solver is NOT the problem (this matched your logs).

What to read off it:
- **Which joints carry the motion.** A `+y` command at typical poses is realized
  mostly by **waist** (~0.5 rad/s, unloaded) plus small corrections from
  shoulder/elbow/wrist_angle (loaded). Straight-line motion needs that *ratio*
  held. Loaded joints lagging → ratio breaks → **wrist curls / path drifts**
  while the waist still sweeps. That's the gravity deficit as path distortion.
- `frozen={...}` non-empty → a joint hit a position limit (column removed).
  Check `wx200_joint_limits_full.yaml` and the KDL chain print at startup.
- `cbf_scale < 1` → joint-limit CBF braking near a bound (`joint_cbf_alpha`).
- "X command also moves a joint you didn't expect" → KDL chain picked up the
  wrong joint/order; check the `KDL chain ... has N non-fixed joints: [...]`
  startup log (jacobian_velctrl.cpp:355) against the JTC joint order.

**THE decisive measurement — commanded vs actual joint velocity:**
```bash
# Send a steady y command (hold the +y key or republish), then:
ros2 topic echo /wx200/joint_states --field velocity
```
Compare actual joint velocity against the `qdot {...}` the log requested. If
`waist` hits ~0.5 rad/s but `wrist_angle/shoulder/elbow` come in far below their
commanded `-0.09 / -0.17 / ...`, you've **measured** the tracking deficit. Go to
Layer 6.

---

## 8. Layer 6 — Motor gains (the actual root cause)

Reach the loop with:
```bash
ros2 service call /wx200/set_motor_pid_gains interbotix_xs_msgs/srv/MotorGains \
  "{cmd_type: 'single', name: '<joint>', kp_pos: <>, ki_pos: <>, kd_pos: <>, \
    k1: <>, k2: <>, kp_vel: 100, ki_vel: 1920}"
```

**Shadow note:** the shoulder is two motors (ID 2 + shadow ID 3,
`Secondary_ID: 2`). Writes to ID 2 propagate to the shadow automatically via the
secondary-ID mechanism, so a `single` call to `shoulder` tunes both. You **cannot
read** the shadow back through the ROS service (reads target the primary ID only;
a single returned value is expected) — use Dynamixel Wizard on ID 3 if you must
verify it.

**Tuning order (one joint, one gain at a time):**
1. **Kill oscillation with damping first.** Raise `kd_pos` 100 → 300 → 600 → 900
   on shoulder/elbow before touching Kp. Your D/P ratio (~0.045) is far too low.
2. **Back off velocity FF.** `k1` 400 → 200 to cut overshoot feeding the ring.
3. **Then tracking lag (the gravity-while-moving deficit).** If joints still lag
   after damping settles, add **`k2`** (accel FF) in small steps (50 → 150) —
   this targets the accel/decel phases where the curl is worst. Prefer k2 over
   pushing k1 back up.
4. **Per-joint, not group.** One value can't serve a dual-motor shoulder and a
   tiny XL430 wrist_rotate.

**Suggested starting points:**

| Joint | kp_pos | ki_pos | kd_pos | k1 | k2 |
|---|---|---|---|---|---|
| shoulder | 2200 | 0 | 600→900 | 200 | 0 |
| elbow | 2000 | 0 | 600 | 200 | 0 |
| wrist_angle | 1500 | 0 | 400 | 100 | 0 |
| wrist_rotate | 800 | 0 | 0 | 0 | 0 |
| waist | 1000 | 0 | 100 | 0 | 0 |

**Highest-leverage non-gain fix:** since the sag is motion-only, **slow the
trajectory**. Lower commanded speed / `Profile_Velocity` / `Profile_Acceleration`
= less tracking lag = less droop, at zero oscillation cost. Try −20–30% first.

After each change, re-run Layer 5's commanded-vs-actual check. Done when the
loaded joints track their `qdot` and the lead clamp ceiling rises smoothly (hold
moves continuously, no curl).

## 9. Layer 7 — Motor faults / limits

```bash
ros2 service call /wx200/get_motor_registers interbotix_xs_msgs/srv/RegisterValues \
  "{cmd_type: 'single', name: 'shoulder', reg: 'Hardware_Error_Status'}"
ros2 service call /wx200/get_motor_registers interbotix_xs_msgs/srv/RegisterValues \
  "{cmd_type: 'single', name: 'shoulder', reg: 'Present_Current'}"
```
- Overload LED / motor clicks off → hitting current limit while fighting gravity.
  Check `Present_Current` vs `Current_Limit`; check the **effort watchdog**
  thresholds (it can zero `/wx200/cmd_vel`).
- Clearing a latched error requires re-torque / reboot of the servo.

## 10. Layer 8 — Teleop redesign (optional, removes Layer 1/3 fragility)

Current teleop publishes only on a key byte and leans on `command_timeout_sec`
as the dead-man. To decouple from OS auto-repeat:
- Latch the last key and **republish every timer tick** (already 200 Hz).
- Track time since last keypress; after an idle window (e.g. 100 ms) publish an
  explicit **zero** instead of nothing.
Then: hold = true continuous stream, tap = clean short move, independent of
`xset`. Re-enable the commented zero-publish in `loop()` as the idle path.

---

## Quick triage flow

1. `ros2 topic hz /velocity_pub/vel_command` while holding → stream? (L1)
2. `ros2 topic info /wx200/cmd_vel -v` → one publisher? (L0/L2)
3. Node log: `recovered≈cmd`, `cbf_scale=1`, `frozen=none`? → solver OK (L5)
4. `joint_states` velocity vs log `qdot` on loaded joints → deficit? (L5)
5. If deficit → damping-first gain tuning + slow profile (L6)
6. Still hold-stalls → confirm lead-clamp deadlock, fix underlying gains (L4/L6)
```
