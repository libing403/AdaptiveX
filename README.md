# ADX (AdaptiveX) — Open-Source Highlights and Key Capabilities

## Project Positioning
ADX is an **adaptive multi-axis motion control framework** for industrial automation, implemented in C/C++.  
It focuses on motion control that is scalable, engineering-ready, and hardware-integrable.  
In addition to single-axis control, ADX supports multi-axis coordination, trajectory smoothing/interpolation, online parameter tuning, and hardware abstraction integration.

---

## Project Highlights

### 1) Adaptive Control Capability (Adaptive)
- Supports runtime adjustment of key parameters such as velocity, acceleration, jerk, and override ratio.
- Dynamically adapts control behavior to load and process-condition changes.
- Provides clear interface boundaries for future self-learning/intelligent optimization algorithms.

### 2) Multi-Axis Coordination and Layered Architecture (X = Extensible)
- Provides both single-axis and axis-group capabilities: positioning, velocity control, linear interpolation, and state management.
- Core architecture is layered:
	- **flexCore**: Core interfaces and orchestration.
	- **mcCore**: Axis/axis-group state machines and motion-control logic.
	- **mcHal**: Hardware abstraction layer (motors, I/O, encoders, etc.).
- The application layer can focus on process logic while hardware can be replaced independently.

### 3) Engineering-Oriented State Machine Design
- Axis states cover typical industrial workflows: DISABLED / STANDSTILL / MOVING / STOPPING / ERRORSTOP.
- A unified command/parameter channel simplifies process control, fault recovery, and coordinated motion.

### 4) Trajectory Planning and Smooth Motion
- Integrates trajectory capabilities such as Ruckig and supports jerk-constrained motion.
- Supports buffering, blending, and look-ahead smoothing to improve trajectory continuity.
- Helps reduce mechanical shock, suppress vibration, and stabilize cycle consistency.

### 5) Industrial Interface Friendliness
- Provides a clear C API (`flexCore_if.h`) for initialization, enable, motion, stop, parameter R/W, and status reads.
- Compatible with Modbus-style communication for easy host/PLC integration.
- Includes complete logging paths for debugging and issue diagnosis.

### 6) High Verifiability
- `app/` includes multiple test programs covering typical action chains:
	- Single-axis absolute/relative/velocity motion
	- Axis-group linear motion and stop
	- Override adjustment and multi-block buffering
	- Soft limits and status reads
- Useful as learning samples and as a regression-test baseline.

---

## Motion Controller Design Philosophy

### 1) Stable Interfaces, Evolvable Core
- Keep a unified external command model (enable, move, stop, parameter R/W, status read) to minimize upper-layer process-code rework.
- Use layered architecture (`flexCore` / `mcCore` / `mcHal`) to isolate hardware variance from control strategies and enable continuous upgrades.

### 2) State-Driven over Process-Coupled
- Use axis/axis-group state machines as behavioral constraints to avoid race conditions and inconsistencies.
- Unify normal flow, deceleration stop, fault handling, and reset/recovery under consistent state semantics.

### 3) Balance Real-Time Responsiveness and Smoothness
- Combine jerk constraints, look-ahead, and blending to reduce mechanical impact while preserving cycle performance.
- Allow runtime override and parameter tuning to adapt to load changes and process switching.

### 4) Engineering Deployment First
- Provide practical C APIs, logging, and test samples to close the loop from algorithm capability to field debugging.
- Expose extension points early (kinematics, drive adaptation, group-control strategy) to support diverse machine configurations.

---

## PLCopen-Inspired Design

ADX follows PLCopen Motion Control engineering practices in interface organization and state semantics, while extending them for native C/C++ controller implementation.

### 1) Function-Block Semantics Mapped to C APIs
- Single-axis motion semantics align with common PLCopen concepts:
	- Absolute/relative positioning (similar to `MC_MoveAbsolute` / `MC_MoveRelative`)
	- Velocity motion (similar to `MC_MoveVelocity`)
	- Stop/halt (similar to `MC_Stop` / `MC_Halt`)
- Retains a consistent "command + parameters + status feedback" pattern, easing migration from PLC/upper-level control logic.

### 2) State-Machine Semantics Alignment
- Uses standard industrial state expressions: DISABLED / STANDSTILL / MOVING / STOPPING / ERRORSTOP.
- Matches PLCopen’s command-trigger, state-confirmation, and fault-reset control loop philosophy.

### 3) BufferMode and Continuous Trajectory Concept
- Supports buffered execution, motion blending, and look-ahead smoothing, consistent with PLCopen BufferMode concepts.
- Extends this with jerk constraints and finer trajectory control for high-speed/high-precision scenarios.

### 4) Axis-Group and Coordinated Control Concept
- Axis-group creation, enable, status read, and linear interpolation align with PLCopen group-control thinking.
- Further extends with project-specific capabilities such as gantry synchronization, electronic gearing, and kinematics registration.

---

## Key Function List

### Single-Axis Control
- `adx_MoveAbsolute`: Absolute positioning
- `adx_MoveRelative`: Relative positioning
- `adx_MoveVelocity`: Velocity mode motion
- `adx_Stop` / `adx_Halt`: Stop and emergency halt
- `adx_SetOverride`: Runtime override adjustment

### Parameters and Status
- `adx_ReadParameter` / `adx_WriteParameter`
- `adx_ReadTargetPos` / `adx_ReadActualPos`
- `adx_ReadStatus` / `adx_ReadMultiStatus`
- Supports reading logical position, actual position, velocity, acceleration, jerk, etc.

### Axis Groups and Interpolation
- `adx_CreateAxisGroup` / `adx_GroupEnable`
- `adx_MoveLinearAbsolute` (and related group motion APIs)
- `adx_GroupReadLogicalPos` / `adx_GroupReadStatus`
- Look-ahead smoothing, buffered execution, and motion blending

### Mechanical Coordination
- Gantry synchronization: `adx_SetGantryConfig` / `adx_SetGantryEnable`
- Electronic gearing: `adx_GearIn`

### Extensible Integration
- Supports registering low-level functions such as motor enable and position readback
- Supports registering forward/inverse kinematics interfaces (`adx_SetKinematics`)

---

## Typical Application Scenarios
- Multi-axis trajectory-following processes such as dispensing, welding, and spraying
- Path smoothing and cycle-time control in transfer/loading applications
- Gantry structures and dual-axis coordinated equipment
- Platform projects requiring rapid integration of different drives/controller boards

---

## Open-Source Value
- **Readability**: Clear module boundaries and centralized interfaces for secondary development.
- **Reusability**: Complete single-axis, group, interpolation, and state-machine capabilities.
- **Portability**: HAL design enables cross-platform and multi-drive migration.
- **Evolvability**: Supports evolution from rule-based control to adaptive/intelligent control.

---

## Quick Start (Windows + CMake)
```bash
cd build
cmake ..
cmake --build . --config Debug --target install
```

After generating the project, build the Debug configuration and use test programs under `app/` to validate core motion interfaces.

---

## Conclusion
ADX targets real industrial control requirements with a focus on **stable usability** and **continuous extensibility**.  
For teams building a general-purpose motion-control foundation and gradually adding adaptive capabilities, this project offers solid open-source and engineering value.