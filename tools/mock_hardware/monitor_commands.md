# Monitor Commands

Run the scenario in one terminal, then use one or more of these in other terminals.

## Recommended Minimal Set

```bash
ros2 topic echo /debug/state
```

```bash
ros2 topic echo /cmd_vel
```

```bash
ros2 topic echo /odometry/filtered
```

## IR Scan Checks

```bash
ros2 topic echo /ir_front/scan --once
```

```bash
ros2 topic echo /ir_front_left/scan --once
```

```bash
ros2 topic echo /ir_front_right/scan --once
```

```bash
ros2 topic echo /ir_left/scan --once
```

```bash
ros2 topic echo /ir_right/scan --once
```

## Combined Helper

```bash
python3 tools/mock_hardware/mock_validation_monitor.py
```