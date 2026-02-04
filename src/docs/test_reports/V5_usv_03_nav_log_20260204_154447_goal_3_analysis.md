# USV_03 Navigation Log Analysis (Goal 3)

**Log File**: `~/usv_logs/V5/usv_03/nav_log_20260204_154447_goal_3.csv`  
**Date**: 2026-02-04  
**Vessel**: USV_03  
**Analyst**: GitHub Copilot  

## 1. Executive Summary

USV_03 successfully reached most waypoints (arrival < 1.5m), but exhibited **severe control difficulties** compared to USV_02. The vessel spent **44.4%** of the mission with steering controls fully saturated (maxed out), significantly higher than the typical ~10-15%.

The vessel frequently got "stuck" at the start of new path segments, saturating the controller for 2-6 seconds while drifting away from the target before eventually turning. This suggests a mechanical limitation (underpowered turning), drag issue (seaweed/fouling), or aggressive tuning that the hardware cannot match.

## 2. Key Metrics Overview

| Metric | Value | Status | Comparison (USV_02) |
| :--- | :--- | :--- | :--- |
| **Mean Speed** | 0.17 m/s | Low | Similar (0.24 m/s) |
| **Max Heading Error** | 179.9° | Expected (Starts) | 165° |
| **Avg Heading Error** | **34.9°** | **High** | ~16.2° |
| **Steering Saturation**| **44.4%** | **Critical** | ~11.1% |
| **Max MPC Time** | 79.0 ms | Warning | 44.0 ms |
| **Arrivals** | 7/8 Success | Pass | - |

## 3. Detailed Reliability Analysis

### 3.1 The "Stuck" Phenomenon (Start-up Lag)
USV_03 struggles primarily when switching goals (turning around). Unlike USV_02 which turns smoothly, USV_03 saturates its command (`cmd_omega`) but fails to change heading effectively for several seconds.

**Notable "Stuck" Intervals:**
*   **Segment 8 (Start)**: Stuck for **4.7s**. Distance drifted from 9.3m to 10.3m. Mean Heading Error 109°.
*   **Segment 12 (Start)**: Stuck for **2.3s**. Mean Heading Error 169°.
*   **Segment 28 (Start)**: Stuck for **5.8s**. Distance drifted from 9.3m to 9.7m. Mean Heading Error 132°.

During these times, the controller is demanding maximum turn (Saturation > 95%), but the vessel response is sluggish.

### 3.2 Control Saturation
*   **Frequency**: Steering output (`cmd_omega`) was saturated (|u| > 0.499) for 44.4% of total time.
*   **Impact**: When saturated, the MPC feedback loop is effectively broken (open loop). This correlates with the higher MPC solve times (max 79ms), as the solver struggles to find a feasible solution when the actuator can't deliver the requested control.

### 3.3 Goal Arrival Performance
Despite the control struggles, the vessel generally achieved the navigation tolerance (1.5m).

| Goal ID | Start Dist | Min Dist | End Dist | Saturation % | Result |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 3 | 11.5m | 1.5m | 1.5m | 46.0% | **Success** |
| 4 | 10.7m | 1.2m | 1.2m | 43.6% | **Success** |
| 8 | 9.3m | 1.3m | 1.3m | 50.8% | **Success** |
| 12 | 9.3m | 1.4m | 1.4m | 45.7% | **Success** |
| 16 | 9.2m | 1.5m | 1.5m | 41.0% | **Success** |
| 20 | 9.2m | 1.4m | 1.4m | 44.0% | **Success** |
| 24 | 9.2m | 1.4m | 1.4m | 34.3% | **Success** |
| 28 | 9.1m | 4.7m | 4.7m | 53.7% | *Incomplete* |

## 4. Anomalies
*   **Data Sync**: Detected 14 instances of distance data mismatch (lag at goal switching), similar to USV_02. This is a system-wide logging artifact, not specific to USV_03.
*   **MPC Performance**: The peak solve time (79ms) approaches the control period limit (typically 100ms or 50ms depending on rate). High CPU load or solver struggling with unachievable constraints.

## 5. Conclusions & Recommendations

1.  **Hardware Inspection Required**: The extreme difference in saturation compared to USV_02 suggests a physical issue.
    *   Check for propeller fouling (seaweed).
    *   Check rudder servo range of motion or differential thruster output balance.
    *   Verify battery voltage levels (voltage sag can reduce thrust authority).
2.  **Tuning Adjustment**: If hardware is clean, the MPC constraints for USV_03 might be too aggressive.
    *   Consider relaxing the turn rate constraints.
    *   Or, increase the `weight_omega` cost to penalize aggressive steering if the hull is naturally slow to turn.
3.  **Arrival Threshold**: The 1.5m threshold (confirmed in config) is working well for this vessel, allowing it to "finish" despite the sloppy control.
