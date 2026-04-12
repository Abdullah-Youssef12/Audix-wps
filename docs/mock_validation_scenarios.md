# Mock Validation Scenarios

This document maps the named mock IR scenarios to the obstacle situations they are intended to create for [src/audix_pkg/scripts/arena_roamer.py](src/audix_pkg/scripts/arena_roamer.py).

The purpose is to validate obstacle and reroute behavior without modifying `arena_roamer.py`.

## How To Use This Document

For each scenario below:
- launch the mock stack with the named `ir_scenario`
- monitor `/debug/state`, `/cmd_vel`, `/odometry/filtered`, and the relevant IR scan topics
- compare what you see against the expected high-level reaction
- treat the warning signs as indicators that the mock inputs, bridge layer, or routing behavior may need investigation

## Scenarios

### `all_clear`

Inputs:
- all digital IR topics remain clear

Expected use:
- verify basic route following
- verify that no obstacle-triggered behavior occurs

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/odometry/filtered`

Warning signs:
- repeated obstacle-like state changes while all scans are clear
- zero or erratic `/cmd_vel` despite clear inputs and valid odometry

### `front_blocked`

Inputs:
- only `/ir_front_digital` is blocked

Expected use:
- verify direct front obstacle detection
- verify stop, caution, or obstacle-response behavior for a single frontal block

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front/scan`

Warning signs:
- front scan remains clear while the digital scenario is active
- no visible reaction in state or velocity output during sustained front blockage

### `front_left_blocked`

Inputs:
- only `/ir_front_left_digital` is blocked

Expected use:
- verify front-left detection path
- verify side-biased obstacle interpretation

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front_left/scan`

Warning signs:
- front-left scan does not show blocked samples
- motion or state output is indistinguishable from `all_clear`

### `front_right_blocked`

Inputs:
- only `/ir_front_right_digital` is blocked

Expected use:
- verify front-right detection path
- verify mirrored side-biased obstacle interpretation

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front_right/scan`

Warning signs:
- front-right scan does not show blocked samples
- mirrored behavior is inconsistent relative to the left-side case

### `left_blocked`

Inputs:
- only `/ir_left_digital` is blocked

Expected use:
- verify lateral obstacle handling on the left side

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_left/scan`

Warning signs:
- left scan does not reflect blockage
- no side-sensitive change in motion or state output

### `right_blocked`

Inputs:
- only `/ir_right_digital` is blocked

Expected use:
- verify lateral obstacle handling on the right side

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_right/scan`

Warning signs:
- right scan does not reflect blockage
- mirrored behavior is inconsistent relative to the left-side case

### `transient_front_blocked`

Inputs:
- start clear
- block only the front sensor briefly
- return to clear

Expected use:
- verify that a short-lived obstacle causes response and then recovery

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front/scan`

Warning signs:
- blocked state persists in scans after the scenario returns clear
- velocity or state never recovers after the transient phase ends

### `persistent_front_blocked`

Inputs:
- start clear
- then hold the front sensor blocked for an extended period

Expected use:
- verify sustained front-obstacle handling
- verify reroute or persistent obstruction behavior as implemented today

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front/scan`

Warning signs:
- front scan shows blockage but state remains in nominal motion
- no sustained change in output during the persistent blocked phase

### `reroute_left_case`

Inputs:
- start clear
- then block `front`, `front_right`, and `right`

Expected use:
- create a case where the left side is more favorable than the right side
- observe reroute or bypass behavior with a left-favoring clearance picture

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front/scan`
- `/ir_right/scan`
- `/ir_front_right/scan`

Warning signs:
- right-side scans do not show the intended blocked pattern
- state output never shows a meaningful obstacle-response phase
- motion output suggests symmetric or wrong-side clearance interpretation

### `reroute_right_case`

Inputs:
- start clear
- then block `front`, `front_left`, and `left`

Expected use:
- create a case where the right side is more favorable than the left side
- observe reroute or bypass behavior with a right-favoring clearance picture

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front/scan`
- `/ir_left/scan`
- `/ir_front_left/scan`

Warning signs:
- left-side scans do not show the intended blocked pattern
- state output never shows a meaningful obstacle-response phase
- motion output suggests symmetric or wrong-side clearance interpretation

### `obstacle_appears_after_motion`

Inputs:
- start clear long enough for the route to begin normally
- then block only the front sensor
- then return to clear

Expected use:
- verify reaction when an obstacle appears after an initial motion window
- verify return toward nominal behavior after the obstacle clears

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/odometry/filtered`
- `/ir_front/scan`

Warning signs:
- no change in state or motion when the front scan flips to blocked
- no recovery after the blocked phase ends

### `obstacle_reappears_during_rejoin`

Inputs:
- begin clear
- create a reroute-like blockage
- briefly clear the path
- then reintroduce a front blockage

Expected use:
- verify behavior when the route appears to reopen and then becomes obstructed again
- check for stable repeated obstacle-response transitions

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front/scan`
- `/ir_right/scan`

Warning signs:
- scans do not match the scripted phases
- state becomes stuck after the second obstacle event
- command output becomes erratic across the clear-to-blocked transition

### `side_clearance_delayed_left`

Inputs:
- front and left-side sensors begin blocked together
- left-side blockage clears only in stages
- front finally clears later

Expected use:
- verify handling when left-side clearance improves slowly
- observe whether the state and motion outputs reflect staged clearance changes

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front_left/scan`
- `/ir_left/scan`
- `/ir_front/scan`

Warning signs:
- left-side scan topics do not change in stages
- state output ignores staged clearance improvement entirely

### `side_clearance_delayed_right`

Inputs:
- front and right-side sensors begin blocked together
- right-side blockage clears only in stages
- front finally clears later

Expected use:
- verify handling when right-side clearance improves slowly
- observe whether the state and motion outputs reflect staged clearance changes

Useful topics:
- `/debug/state`
- `/cmd_vel`
- `/ir_front_right/scan`
- `/ir_right/scan`
- `/ir_front/scan`

Warning signs:
- right-side scan topics do not change in stages
- state output ignores staged clearance improvement entirely

## Recommended Observation Topics

- `/debug/state`
- `/cmd_vel`
- `/odometry/filtered`
- `/ir_front/scan`
- `/ir_front_left/scan`
- `/ir_front_right/scan`
- `/ir_left/scan`
- `/ir_right/scan`
- `/ir_back/scan`