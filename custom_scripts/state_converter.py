import numpy as np
import h5py
from libero.libero.envs import OffScreenRenderEnv


class StateConverter:
    """
    Converts states between different robot configurations.
    Specifically handles conversion from Panda robot states to UR5e robot states.
    """
    
    def __init__(self, task_bddl_file):
        self.task_bddl_file = task_bddl_file
        self._analyze_environments()
    
    def _analyze_environments(self):
        """Analyze the state structure of both robot configurations."""
        
        # Create Panda environment to get reference structure
        env_panda = OffScreenRenderEnv(**{
            'bddl_file_name': self.task_bddl_file,
            'robots': ['Panda'],
        })
        env_panda.reset()
        
        # Create UR5e environment to get target structure  
        env_ur5e = OffScreenRenderEnv(**{
            'bddl_file_name': self.task_bddl_file,
            'robots': ['UR5e'],
        })
        env_ur5e.reset()
        
        # Store environment info
        self.panda_nq = env_panda.env.sim.model.nq
        self.panda_nv = env_panda.env.sim.model.nv
        self.ur5e_nq = env_ur5e.env.sim.model.nq
        self.ur5e_nv = env_ur5e.env.sim.model.nv
        
        # Joint mappings
        self.panda_joint_names = [env_panda.env.sim.model.joint_id2name(i) 
                                  for i in range(env_panda.env.sim.model.njnt)]
        self.ur5e_joint_names = [env_ur5e.env.sim.model.joint_id2name(i) 
                                for i in range(env_ur5e.env.sim.model.njnt)]
        
        # Get default positions/velocities for UR5e
        self.ur5e_default_qpos = env_ur5e.env.sim.data.qpos.copy()
        self.ur5e_default_qvel = env_ur5e.env.sim.data.qvel.copy()
        
        env_panda.close()
        env_ur5e.close()
        
        print(f"Panda joints: {self.panda_joint_names}")
        print(f"UR5e joints: {self.ur5e_joint_names}")
        
    def convert_panda_to_ur5e_state(self, panda_state):
        """
        Convert a Panda robot state to UR5e robot state format.
        
        Args:
            panda_state: Flattened state from Panda environment (shape: 47)
                        Format: [time, qpos, qvel] = [1, 24, 22]
            
        Returns:
            ur5e_state: Flattened state for UR5e environment (shape: 53)
                       Format: [time, qpos, qvel] = [1, 27, 25]
        """
        
        # Parse Panda state: [time, qpos, qvel] = [1, 24, 22]
        panda_time = panda_state[0]  # Time (1 element)
        panda_qpos = panda_state[1:1+self.panda_nq]  # Positions (24 elements)
        panda_qvel = panda_state[1+self.panda_nq:]   # Velocities (22 elements)
        
        # Initialize UR5e state with defaults
        ur5e_qpos = self.ur5e_default_qpos.copy()
        ur5e_qvel = self.ur5e_default_qvel.copy()
        
        # Map arm joints (Panda has 7, UR5e has 6)
        # We'll map the first 6 Panda arm joints to UR5e arm joints
        # and ignore Panda's 7th joint
        panda_arm_qpos = panda_qpos[:6]  # Take first 6 arm joints
        panda_arm_qvel = panda_qvel[:6]  # Take first 6 arm velocities
        
        ur5e_qpos[:6] = panda_arm_qpos  # UR5e arm joints
        ur5e_qvel[:6] = panda_arm_qvel  # UR5e arm velocities
        
        # Map gripper: Panda has 2 gripper joints, UR5e has 6
        # We'll map Panda's gripper state to the main UR5e gripper joint
        # and set other gripper joints to corresponding values
        panda_gripper_qpos = panda_qpos[7:9]  # Panda gripper joints
        panda_gripper_qvel = panda_qvel[7:9]   # Panda gripper velocities
        
        # Map to UR5e gripper (simplified mapping)
        # The main gripper joint controls the others in Robotiq85
        main_gripper_pos = np.mean(panda_gripper_qpos)
        main_gripper_vel = np.mean(panda_gripper_qvel)
        
        ur5e_qpos[6] = main_gripper_pos  # Main gripper joint
        ur5e_qvel[6] = main_gripper_vel
        
        # The other gripper joints are coupled, set them to reasonable values
        # Based on Robotiq85 kinematics
        ur5e_qpos[7] = main_gripper_pos * 0.8   # left_inner_finger
        ur5e_qpos[8] = main_gripper_pos         # left_inner_knuckle  
        ur5e_qpos[9] = main_gripper_pos         # right_outer_knuckle
        ur5e_qpos[10] = main_gripper_pos * 0.8  # right_inner_finger
        ur5e_qpos[11] = main_gripper_pos        # right_inner_knuckle
        
        ur5e_qvel[7:12] = main_gripper_vel * 0.8  # Scale velocities
        
        # Map object joints (should be the same for both environments)
        # Objects: chefmate_8_frypan_1, moka_pot_1, flat_stove_1_button
        panda_obj_qpos = panda_qpos[9:]   # Object positions (remaining elements)
        panda_obj_qvel = panda_qvel[9:]   # Object velocities (remaining elements)
        
        ur5e_qpos[12:] = panda_obj_qpos  # Object positions in UR5e
        ur5e_qvel[12:] = panda_obj_qvel  # Object velocities in UR5e
        
        # Combine into flattened state: [time, qpos, qvel]
        ur5e_state = np.concatenate([[panda_time], ur5e_qpos, ur5e_qvel])
        
        return ur5e_state


def convert_demonstration_file(input_file, output_file, task_bddl_file):
    """
    Convert a demonstration file from Panda robot to UR5e robot format.
    
    Args:
        input_file: Path to input HDF5 file with Panda demonstrations
        output_file: Path to output HDF5 file for UR5e demonstrations  
        task_bddl_file: Path to task BDDL file
    """
    
    converter = StateConverter(task_bddl_file)
    
    with h5py.File(input_file, 'r') as f_in, h5py.File(output_file, 'w') as f_out:
        
        # Copy attributes
        for key in f_in.attrs:
            f_out.attrs[key] = f_in.attrs[key]
        
        # Update robot type in attributes if present
        if 'env_args' in f_in.attrs:
            env_args = f_in.attrs['env_args']
            # Note: this is stored as bytes, would need proper parsing to update robot type
            
        # Process data
        data_in = f_in['data']
        data_out = f_out.create_group('data')
        
        # Copy data attributes
        for key in data_in.attrs:
            data_out.attrs[key] = data_in.attrs[key]
            
        # Process each demonstration
        for demo_key in data_in.keys():
            demo_in = data_in[demo_key]
            demo_out = data_out.create_group(demo_key)
            
            # Copy non-state data directly
            for key in demo_in.keys():
                if key != 'states':
                    demo_out.create_dataset(key, data=demo_in[key][:])
            
            # Convert states
            panda_states = demo_in['states'][:]
            ur5e_states = []
            
            for panda_state in panda_states:
                ur5e_state = converter.convert_panda_to_ur5e_state(panda_state)
                ur5e_states.append(ur5e_state)
            
            demo_out.create_dataset('states', data=np.array(ur5e_states))
            
            print(f"Converted {demo_key}: {panda_states.shape} -> {np.array(ur5e_states).shape}")


if __name__ == "__main__":
    # Test the converter
    from libero.libero import benchmark, get_libero_path
    import os
    
    # Get task info
    benchmark_dict = benchmark.get_benchmark_dict()
    task_suite = benchmark_dict["libero_10"]()
    task = task_suite.get_task(2)
    task_bddl_file = os.path.join(
        get_libero_path("bddl_files"), task.problem_folder, task.bddl_file
    )
    
    # Test single state conversion
    demo_file = "../libero_10/KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it_demo.hdf5"
    
    with h5py.File(demo_file, "r") as f:
        panda_state = f["data/demo_0/states"][0]
        
    converter = StateConverter(task_bddl_file)
    ur5e_state = converter.convert_panda_to_ur5e_state(panda_state)
    
    print(f"Original Panda state shape: {panda_state.shape}")
    print(f"Converted UR5e state shape: {ur5e_state.shape}")
    print(f"Conversion successful: {ur5e_state.shape[0] == 53}")