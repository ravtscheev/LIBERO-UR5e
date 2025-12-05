#!/usr/bin/env python3
"""
UR5e State Conversion Utility

This script provides utilities for converting demonstration states from Panda robot
to UR5e robot format, solving the shape mismatch error when loading Panda demonstrations
into UR5e environments.

Usage:
    python ur5e_state_utils.py convert_demo <input_file> <output_file> <task_bddl_file>
    python ur5e_state_utils.py convert_state <state_file> <task_bddl_file>
"""

import sys
import argparse
import h5py
import numpy as np
from state_converter import StateConverter, convert_demonstration_file


def convert_demo_command(args):
    """Convert a full demonstration file from Panda to UR5e format."""
    print(f"Converting demonstration file: {args.input_file} -> {args.output_file}")
    convert_demonstration_file(args.input_file, args.output_file, args.task_bddl_file)
    print("Conversion complete!")


def convert_state_command(args):
    """Convert a single state from Panda to UR5e format."""
    converter = StateConverter(args.task_bddl_file)
    
    # Load state from file (assuming numpy format)
    panda_state = np.load(args.state_file)
    ur5e_state = converter.convert_panda_to_ur5e_state(panda_state)
    
    print(f"Converted state: {panda_state.shape} -> {ur5e_state.shape}")
    
    # Save converted state
    output_file = args.state_file.replace('.npy', '_ur5e.npy')
    np.save(output_file, ur5e_state)
    print(f"Saved converted state to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='UR5e State Conversion Utility')
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Convert demonstration file command
    demo_parser = subparsers.add_parser('convert_demo', help='Convert demonstration file')
    demo_parser.add_argument('input_file', help='Input HDF5 demonstration file')
    demo_parser.add_argument('output_file', help='Output HDF5 demonstration file')
    demo_parser.add_argument('task_bddl_file', help='Task BDDL file path')
    demo_parser.set_defaults(func=convert_demo_command)
    
    # Convert single state command  
    state_parser = subparsers.add_parser('convert_state', help='Convert single state')
    state_parser.add_argument('state_file', help='Input state file (.npy)')
    state_parser.add_argument('task_bddl_file', help='Task BDDL file path')
    state_parser.set_defaults(func=convert_state_command)
    
    args = parser.parse_args()
    
    if hasattr(args, 'func'):
        args.func(args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()