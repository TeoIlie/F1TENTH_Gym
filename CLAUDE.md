# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## About F1TENTH

This is the F1TENTH Gym environment - a high-fidelity simulation platform for 1/10th scale autonomous racing cars. F1TENTH is an international autonomous racing competition and research platform that provides a standardized testbed for autonomous vehicle algorithms at reduced scale and cost.

### Autonomous Racing Context
The simulator models realistic vehicle dynamics, sensor data (LiDAR), and racing scenarios to enable development and testing of:
- **Path Planning**: Algorithms like Pure Pursuit (implemented in examples)
- **Control Systems**: Speed and steering control for high-speed racing
- **Perception**: LiDAR-based environment understanding
- **SLAM**: Simultaneous Localization and Mapping
- **Reinforcement Learning**: Training autonomous racing agents

### Research Applications
- **Multi-agent Racing**: Support for multiple competing vehicles
- **Safety Systems**: Collision detection and avoidance
- **Vehicle Dynamics**: Realistic bicycle model with proper physics
- **Sensor Simulation**: Accurate LiDAR ray-casting for perception research

## Development Commands

### Installation
```bash
pip install -e .
```

### Testing
```bash
pytest
```
The CI runs pytest for Python versions 3.9-3.12.

### Running Examples
```bash
cd examples
python3 waypoint_follow.py
```

### Docker Support
```bash
docker build -t f1tenth_gym_container -f Dockerfile .
docker run --gpus all -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix f1tenth_gym_container
```

## Architecture Overview

### Core Autonomous Vehicle Components
- **F110Env** (`gym/f110_gym/envs/f110_env.py`): Main gymnasium environment implementing F1/10th vehicle simulation with realistic dynamics
- **Dynamic Models** (`gym/f110_gym/envs/dynamic_models.py`): Vehicle dynamics including bicycle model, tire forces, and multi-body physics
- **Laser Models** (`gym/f110_gym/envs/laser_models.py`): LiDAR simulation with ray-casting for autonomous perception
- **Collision Models** (`gym/f110_gym/envs/collision_models.py`): Safety-critical collision detection systems
- **Base Classes** (`gym/f110_gym/envs/base_classes.py`): Core simulation infrastructure (Simulator, Integrator)

### Control and Planning Systems
The examples demonstrate key autonomous vehicle algorithms:
- **Pure Pursuit Controller** (`examples/waypoint_follow.py`): Classic path-following algorithm with lookahead-based steering
- **Waypoint Navigation**: Trajectory following with speed and steering control
- **Real-time Planning**: Efficient numba-optimized trajectory calculations

### Simulation Environment
- **Multi-agent Support**: Enables competitive racing and multi-vehicle scenarios
- **Real-time Physics**: High-frequency simulation (100Hz+) suitable for control system testing  
- **Sensor Models**: Realistic LiDAR with configurable parameters matching real F1TENTH hardware
- **Track System**: Real-world inspired racing circuits with proper scaling

### Package Structure
- **gym/f110_gym/**: Main package following gymnasium RL environment interface
- **examples/**: Reference implementations of autonomous racing algorithms
- **gym/f110_gym/envs/maps/**: Racing track definitions (real-world inspired circuits)
- **gym/f110_gym/unittest/**: Validation tests for physics and sensor models

### Key Dependencies for Autonomous Systems
- **gymnasium/gym**: RL environment interface for agent training
- **numba**: JIT compilation for real-time performance in control loops
- **pyglet**: Real-time visualization for algorithm development and debugging  
- **numpy/scipy**: Mathematical foundations for control and estimation algorithms

### Vehicle Configuration
F1TENTH vehicles are configured with realistic parameters:
- **Physical**: Mass, wheelbase, center of gravity (matching 1/10th scale)
- **Control**: Speed/steering inputs with proper actuator limits
- **Sensors**: LiDAR specifications matching Hokuyo UST-10LX
- **Dynamics**: Tire models, friction coefficients, aerodynamics

### Racing Environment Setup
```python
env = gym.make('f1tenth_gym:f1tenth-v0', config={
    'map': 'Spielberg_blank',  # Racing circuit
    'num_agents': 2,           # Multi-vehicle racing
    'timestep': 0.01,          # 100Hz for real-time control
    'integrator': 'rk4',       # Accurate physics integration
    'control_input': ['speed', 'steering_angle'],
    'model': 'mb',             # Multi-body vehicle dynamics
    'params': F110Env.fullscale_vehicle_params()  # Realistic vehicle specs
})
```

## Development Notes

### Real-time Performance
Extensive use of numba @njit decorators enables real-time simulation suitable for hardware-in-the-loop testing and control development.

### Hardware Compatibility  
Simulation parameters and sensor models are designed to match real F1TENTH hardware, enabling seamless sim-to-real transfer.

### Racing Circuit Fidelity
Track maps are based on real racing circuits with proper scaling, elevation changes, and challenging features for algorithm validation.

### Safety and Validation
Robust collision detection and physics validation ensure algorithms developed in simulation behave safely on real hardware.

### Known Platform Issues
- Windows: Requires Python 3.8 due to compilation dependencies
- macOS Big Sur+: May need pyglet==1.5.11 for OpenGL framework compatibility