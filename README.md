# Pit 

## Overview

Pit is a project that implements a network traffic control algorithm by reinforcement learning and simulation environment using ns-3 (Network Simulator 3). It aims to optimize network performance by dynamically adjusting traffic control parameters.

## Components

The project consists of two main components:

1. **Network Simulator (ns_sim.py)**: Utilizes ns-3 to simulate network behavior and traffic patterns.
2. **Traffic Control Environment (tc.py)**: Implements a gym-compatible environment for reinforcement learning algorithms to interact with the network simulation.

## Prerequisites

- Docker
- Docker Compose

## Installation and Setup

1. Clone the repository:
   ```
   git clone https://github.com/Knight-X/pit.git
   ```

2. Navigate to the project directory:
   ```
   cd pit
   ```

3. Build the Docker image:
   ```
   sudo docker compose build
   ```

4. Start the containers:
   ```
   sudo docker compose up
   ```

## Usage

1. In a new terminal, access the web container:
   ```
   sudo docker compose exec web bash
   ```

2. Navigate to the app directory:
   ```
   cd /app
   ```

3. Run the training script:
   ```
   python train.py
   ```

4. After training, get ppo_stand.zip parameters


5. On the raspberry pi 4 model b+:
  ```
  python infer.py
  ```


## Future Work

- Implement more sophisticated reward functions
- Explore different network topologies and traffic patterns
- Integrate with popular RL frameworks (e.g., Stable Baselines, RLlib)
- Implement real-time visualization of network performance and RL agent behavior
- Extend the simulation to support multi-agent reinforcement learning scenarios

## Contributing

Contributions to Pit are welcome! Please feel free to submit pull requests, create issues, or suggest improvements.



## References

This project utilizes ns-3 (Network Simulator 3) for network simulation. For more information about ns-3, please visit the official repository:

- ns-3 Repository: [https://gitlab.com/nsnam/ns-3-dev/](https://gitlab.com/nsnam/ns-3-dev/)

