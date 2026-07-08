# Bounded Motion Test Procedure — Freight100

> Companion to the [main system overview](README.md). This document records **how the robot
> was safely commanded to move** using bounded, closed-loop velocity tests, and provides a
> reusable script + procedure to repeat them.

These tests drive the base a short, fixed distance at a low speed, using odometry feedback to
stop exactly at the target — never open-loop timing alone. All commands go through the
Robust.AI motion mux, exactly as the autonomy stack does.

---

## 1. Safety Preconditions (must all be true)

Before **any** motion command is sent, verify:

```bash
export ROS_MASTER_URI=http://localhost:11311   # or http://<robot-ip>:11311 from a remote host
source /opt/ros/noetic/setup.bash
rostopic echo -n1 /robot_state | grep -E 'ready|faulted|runstopped'
```

Required readings:

| Field | Required value | Meaning |
|---|---|---|
| `runstopped` | **false** | Hardware runstop released (twist the red button) |
| `ready` | **true** | Driver will accept motion |
| `faulted` | **false** | No active fault |

Plus, physically:

- **Runstop is hardware-only** — software cannot release it. A person at the robot must twist
  the red runstop button until it pops out.
- **Area clear** — no people/obstacles in the direction of travel; operator in line-of-sight.
- **Off the dock** — check `rostopic echo -n1 /battery_state | grep is_charging`. Drive off the
  charger before running a sequence of tests.
- The **physical runstop remains the abort** at all times — pressing it opens the motor
  breakers instantly, regardless of what software is commanding.

---

## 2. Control Path Used

The test publishes a `geometry_msgs/Twist` to the mux input `/robust/nav_cmd_vel` — the same
channel the autonomous navigation stack uses:

```
test script ──► /robust/nav_cmd_vel ──► movement_mux_node ──► /robust/cmd_vel ──► robot_driver ──► wheels
```

Feedback for the closed loop comes from `/odom` (`nav_msgs/Odometry`).

Key design points of the procedure:

- **Closed-loop stop:** the script integrates Euclidean distance from the start pose and stops
  the instant `distance >= target` — not on a timer.
- **Hard time cap:** a `CAP_S` backstop stops the robot even if odom stalls, so it can never run
  away.
- **Continuous publish at 10 Hz:** the driver expects a live command stream; the loop republishes
  every 100 ms and the mux/driver watchdog stops the base if the stream drops.
- **Explicit zero on exit:** on completion the script publishes zero-velocity 15× to guarantee a
  full stop and hold.

---

## 3. Reusable Test Script

Save as `move_test.py`. Edit `TARGET` (meters) and `SPEED` (m/s; **negative = backward**).

```python
#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TARGET = 0.50       # meters of travel
SPEED  = 0.05       # m/s  (negative = backward)
CAP_S  = 20.0       # hard time cap (safety backstop)

state = {"x": None, "y": None, "x0": None, "y0": None}

def cb(msg):
    p = msg.pose.pose.position
    if state["x0"] is None:
        state["x0"], state["y0"] = p.x, p.y
    state["x"], state["y"] = p.x, p.y

rospy.init_node("bounded_move_test", anonymous=True)
pub = rospy.Publisher("/robust/nav_cmd_vel", Twist, queue_size=1)
rospy.Subscriber("/odom", Odometry, cb)

t_wait = rospy.Time.now()
while state["x0"] is None and (rospy.Time.now() - t_wait).to_sec() < 5:
    rospy.sleep(0.05)
if state["x0"] is None:
    print("ERROR: no odom received; aborting, no command sent."); exit(1)

print(f"START odom: x={state['x0']:.4f} y={state['y0']:.4f}")
print(f"Target travel: {TARGET} m at {SPEED} m/s")

cmd = Twist(); cmd.linear.x = SPEED
rate = rospy.Rate(10)
t0 = rospy.Time.now()
reason = "time cap"
while not rospy.is_shutdown():
    dist = math.hypot(state["x"] - state["x0"], state["y"] - state["y0"])
    el = (rospy.Time.now() - t0).to_sec()
    if dist >= TARGET:
        reason = "target reached"; break
    if el >= CAP_S:
        reason = "TIME CAP hit"; break
    pub.publish(cmd)
    rate.sleep()

stop = Twist()
for _ in range(15):
    pub.publish(stop); rospy.sleep(0.02)

rospy.sleep(0.3)
final = math.hypot(state["x"] - state["x0"], state["y"] - state["y0"])
print(f"STOPPED ({reason}) after {(rospy.Time.now()-t0).to_sec():.1f}s")
print(f"END odom:   x={state['x']:.4f} y={state['y']:.4f}")
print(f"DISTANCE TRAVELED: {final:.3f} m")
```

Run:

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
python3 move_test.py
```

---

## 4. Test Runs Performed & Results

Four bounded runs were executed on this unit. Every run stopped on its **odometry target**
(not the time cap), and each landed within ~7 mm of target with only 1–3 cm of lateral drift —
demonstrating a healthy, accurate, repeatable drive path.

| # | Direction | Speed (m/s) | Target (m) | **Traveled (m)** | Duration | Start x | End x | Stop reason |
|---|---|---|---|---|---|---|---|---|
| 1 | Forward | 0.01 | 0.30 | **0.301** | 37.9 s | −0.008 | 0.293 | Target reached |
| 2 | Backward | −0.03 | 0.50 | **0.505** | 18.8 s | 0.293 | −0.211 | Target reached |
| 3 | Forward | 0.05 | 0.50 | **0.507** | 11.3 s | −0.205 | 0.301 | Target reached |
| 4 | Backward | −0.05 | 0.50 | **0.505** | 11.4 s | 0.301 | −0.202 | Target reached |

**Observations**

- Accuracy: max overshoot 7 mm over 0.50 m (~1.4%); closed-loop stop worked every time.
- Tracking: lateral (y) drift stayed within ~3 cm — the base tracks nearly straight.
- Speeds from 0.01 m/s (very slow crawl) to 0.05 m/s (brisk) all behaved consistently.
- Net displacement after all four runs: odom x ≈ −0.20 m (about 0.20 m behind the dock start).

---

## 5. After Testing

- Publish zero velocity (the script does this automatically on exit).
- If finished, **drive the robot forward back onto the dock** to resume charging
  (`is_charging` returns to `true` once seated).
- Re-engage the hardware runstop when leaving the robot unattended.

---

*Generated from live operation of the running system. See [README.md](README.md) for the full
platform overview.*
