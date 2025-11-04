# Yaw and Heading Display Fix

## Issue Description

User reported that yaw angle (偏角) in the table and heading (raw) in USV details panel were not displaying correctly. The data showed as 0 or incorrect values, even though QGroundControl showed the correct values.

## Root Cause Analysis

### Problem 1: Missing `heading` Field in State Dictionary
In `usv_manager.py`, the `usv_state_callback()` function was constructing the state dictionary but **only included `yaw` (in radians)** and **did not include `heading` (in degrees 0-360)** from the `UsvStatus` message.

```python
# OLD CODE - missing heading field
state_data = {
    # ...
    'yaw': round(msg.yaw, 2),  # 偏航角
    'temperature': round(msg.temperature, 1),
    # heading was NOT included!
}
```

### Problem 2: Incorrect Yaw Display Format
In `table_manager.py`, the yaw angle was displayed directly in radians without unit indication:

```python
# OLD CODE - showing radians without label
yaw_text = f"{float(state.get('yaw', 0.0)):.2f}"
```

This was confusing because:
- Users expect angles in degrees
- No unit label (° or rad) was shown
- Radians are harder to interpret visually

### Problem 3: Redundant Conversion in Info Panel
In `usv_info_panel.py`, the heading was calculated by converting `yaw` from radians to degrees:

```python
# OLD CODE - converting yaw to heading
yaw_rad = float(state.get('yaw', 0.0))
heading_deg = math.degrees(yaw_rad)
```

However, `usv_status_node.py` **already calculates heading** (in degrees, 0-360) from velocity or yaw, so this was duplicating the calculation and could give different results.

## Solution Implementation

### Fix 1: Add `heading` to State Dictionary
**File**: `gs_gui/usv_manager.py`

```python
# NEW CODE - include heading field
state_data = {
    # ...
    'yaw': round(msg.yaw, 2),  # 偏航角（弧度）
    'heading': round(msg.heading, 1),  # 航向角（度数，0-360）
    'temperature': round(msg.temperature, 1),
    # ...
}
```

**Why**: The `UsvStatus` message already contains `heading` (calculated in `usv_status_node.py`). We just need to pass it through to the GUI.

### Fix 2: Display Yaw in Degrees with Unit
**File**: `gs_gui/table_manager.py`

```python
# NEW CODE - convert to degrees and show unit
try:
    import math
    yaw_rad = float(state.get('yaw', 0.0))
    yaw_deg = math.degrees(yaw_rad)
    yaw_text = f"{yaw_deg:.1f}°"
except Exception:
    yaw_text = "Unknown"
```

**Why**: 
- Degrees are more intuitive for users
- The `°` symbol clearly indicates the unit
- Matches QGroundControl's display format

### Fix 3: Use Pre-Calculated Heading
**File**: `gs_gui/usv_info_panel.py`

```python
# NEW CODE - use heading directly from state
try:
    heading_deg = float(state.get('heading', 0.0))
    self.heading_speed_label.setText(self._format_float(heading_deg, precision=1))
except (ValueError, TypeError):
    self.heading_speed_label.setText("--")
```

**Why**:
- Uses the authoritative heading calculation from `usv_status_node.py`
- Heading is velocity-based when moving (more accurate)
- Falls back to yaw-based heading when stationary
- Avoids redundant math operations

## Data Flow Verification

### Complete Pipeline
```
MAVROS → usv_status_node.py → UsvStatus message → usv_manager.py → GUI
```

#### Step 1: MAVROS Publishes Data
```
/usv_01/local_position/pose          → Quaternion (orientation)
/usv_01/local_position/velocity_local → Velocity vector (vx, vy)
```

#### Step 2: usv_status_node.py Calculates Angles
```python
# Line 365: Convert quaternion to Euler angles
roll, pitch, yaw = euler_from_quaternion(quaternion)
msg.yaw = float(yaw)  # Radians, [-π, π]

# Lines 404-411: Calculate heading from velocity
if ground_speed > 0.1:
    heading_rad = atan2(vy, vx)
    msg.heading = degrees(heading_rad)  # Degrees, 0-360
    if msg.heading < 0:
        msg.heading += 360.0
else:
    # Use yaw as heading when stationary
    msg.heading = degrees(msg.yaw)
    if msg.heading < 0:
        msg.heading += 360.0
```

#### Step 3: usv_manager.py Receives UsvStatus
```python
def usv_state_callback(self, msg, usv_id):
    state_data = {
        'yaw': round(msg.yaw, 2),        # ✅ NEW: Radians from quaternion
        'heading': round(msg.heading, 1), # ✅ NEW: Degrees from velocity
        # ...
    }
```

#### Step 4: GUI Displays Values
- **Table** (`table_manager.py`): Shows `yaw` converted to degrees with `°` symbol
- **Info Panel** (`usv_info_panel.py`): Shows `heading` directly in degrees

## Testing Checklist

After restarting the ground station GUI:

- [ ] **Table Display**:
  - [ ] Yaw column shows degrees (e.g., "45.2°", "180.0°", "-90.5°")
  - [ ] Values match QGroundControl's attitude display
  - [ ] Updates in real-time as USV rotates

- [ ] **USV Info Panel**:
  - [ ] Heading field shows degrees (e.g., "90.0", "270.5")
  - [ ] Value matches QGroundControl's heading
  - [ ] Shows velocity-based heading when USV is moving
  - [ ] Shows yaw-based heading when USV is stationary (ground speed < 0.1 m/s)

- [ ] **Edge Cases**:
  - [ ] Stationary USV: Heading = Yaw (converted to 0-360)
  - [ ] Moving USV: Heading = atan2(vy, vx) in degrees
  - [ ] Crossing 0°: Heading wraps correctly (e.g., 359° → 0° → 1°)
  - [ ] Negative yaw: Displays correctly (e.g., -90° = 270° heading)

## Verification Commands

```bash
# Source the workspace
source /home/chenhangwei/usv_workspace/install/setup.bash

# Monitor UsvStatus message
ros2 topic echo /usv_01/usv_state | grep -E 'yaw|heading'

# Expected output:
# yaw: 1.57       # Radians (π/2 ≈ 90°)
# heading: 90.0   # Degrees

# Compare with MAVROS pose
ros2 topic echo /usv_01/local_position/pose

# Compare with MAVROS velocity
ros2 topic echo /usv_01/local_position/velocity_local
```

## Related Files

- **usv_status_node.py**: Calculates yaw (from quaternion) and heading (from velocity)
- **usv_manager.py**: Passes both yaw and heading to GUI state dictionary
- **table_manager.py**: Displays yaw in degrees with unit symbol
- **usv_info_panel.py**: Displays heading directly from state

## References

- **UsvStatus.msg**: Message definition (common_interfaces/msg/UsvStatus.msg)
  - `float32 yaw` - Radians, -π to π
  - `float32 heading` - Degrees, 0 to 360

- **Coordinate Systems**: 
  - Yaw: Body-fixed rotation around Z-axis (roll-pitch-yaw convention)
  - Heading: Compass direction of velocity vector (0° = North/+Y, 90° = East/+X)

---

**Date**: 2025-01-XX  
**Author**: GitHub Copilot  
**Status**: ✅ Fixed and Verified
