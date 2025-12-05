# UR5e Robot Support for LIBERO

This directory contains utilities and modifications to support the UR5e robot in the LIBERO environment, addressing the shape mismatch error when loading Panda robot demonstrations with UR5e robots.

## Problem

When trying to load demonstration data collected with Panda robots into environments configured for UR5e robots, a shape mismatch error occurs:

```
ValueError: could not broadcast input array from shape (19,) into shape (25,)
```

This error happens because:
- **Panda robot**: 22 velocity DOF, 47 total state elements
- **UR5e robot**: 25 velocity DOF, 53 total state elements

The difference comes from the gripper configuration:
- Panda uses PandaGripper (2 joints)
- UR5e uses Robotiq85Gripper (6 joints)

## Solution

The `StateConverter` class provides automatic conversion between Panda and UR5e state formats by:

1. **Parsing the state structure**: `[time, qpos, qvel]`
2. **Mapping arm joints**: First 6 Panda joints → UR5e joints (ignoring Panda's 7th joint)
3. **Converting gripper state**: Panda's 2 gripper joints → UR5e's 6 gripper joints with kinematic coupling
4. **Preserving object states**: Environment objects remain unchanged

## Files

- `state_converter.py` - Core state conversion functionality
- `render.py` - Updated render script using state conversion
- `ur5e_state_utils.py` - Command-line utility for batch conversions

## Usage

### Basic Rendering with State Conversion

```python
from state_converter import StateConverter

# Initialize converter
converter = StateConverter(task_bddl_file)

# Convert single state
ur5e_state = converter.convert_panda_to_ur5e_state(panda_state)

# Use in environment
env.set_init_state(ur5e_state)
```

### Converting Demonstration Files

```bash
# Convert full demonstration file
python ur5e_state_utils.py convert_demo input_demo.hdf5 output_demo.hdf5 task.bddl

# Convert single state
python ur5e_state_utils.py convert_state state.npy task.bddl
```

### Running the Updated Render Script

```bash
conda activate libero
python render.py
```

## State Structure Details

### Panda Robot State (47 elements)
- Time: 1 element
- qpos: 24 elements (7 arm + 2 gripper + 15 objects)  
- qvel: 22 elements (7 arm + 2 gripper + 13 objects)

### UR5e Robot State (53 elements)
- Time: 1 element
- qpos: 27 elements (6 arm + 6 gripper + 15 objects)
- qvel: 25 elements (6 arm + 6 gripper + 13 objects)

## Joint Mappings

### Arm Joints
- Panda: `[joint1, joint2, joint3, joint4, joint5, joint6, joint7]` 
- UR5e: `[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`
- Mapping: First 6 Panda joints → UR5e joints (joint7 ignored)

### Gripper Joints  
- Panda: `[finger_joint1, finger_joint2]`
- UR5e: `[finger_joint, left_inner_finger, left_inner_knuckle, right_outer_knuckle, right_inner_finger, right_inner_knuckle]`
- Mapping: Average Panda gripper → UR5e main joint, with coupled secondary joints

## Notes

- The conversion provides a reasonable approximation for visualization and basic testing
- For precise control applications, consider collecting new demonstrations with UR5e
- Object joint mappings assume the same environment setup (same objects, same DOF)
- The converter handles the time component correctly in MuJoCo state format