# joy2uf850

**joy2uf850** is a ROS 2 package designed to remap the control of the UF850 robotic arm in a more intuitive way. This package uses Python and leverages ROS 2 Humble for communication and control.

---

## Features

- Remaps joystick inputs for easier control of the UF850 robotic arm.
- Compatible with ROS 2 Humble.
- Modular and extensible design for future enhancements.

---

## Requirements

- ROS 2 Humble
- Python 3.10 or later

---

## Installation

### Clone the Repository
First, clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/joy2uf850.git
```

### Build the Package
Navigate to your workspace root and build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select joy2uf850
```

### Source the Workspace
Source your workspace to make the package available:
```bash
source ~/ros2_ws/install/setup.bash
```

---

## Usage

To run the main node:
```bash
ros2 run joy2uf850 aura_joy
```

---

## File Structure

The key files in this package are:
```bash
joy2uf850/
├── joy2uf850/
│ ├── init.py # Marks the directory as a Python package.
│ └── aura_joy.py # Main node logic.
├── package.xml # ROS 2 package metadata.
├── setup.py # Python package configuration.
├── setup.cfg # Additional Python setup configuration.
└── README.md # This file.
```

---

## Contributing

Contributions are welcome! If you'd like to contribute, please fork the repository, make changes, and submit a pull request.

1. Fork the repository on GitHub.
2. Clone your fork locally:
```bash
git clone https://github.com/your-username/joy2uf850.git
```

3. Create a new branch for your feature or fix:
```bash
git checkout -b feature-name
```

4. Commit your changes:
```bash
git add .
git commit -m "Description of changes"
```

5. Push to your fork and submit a pull request.

---

## Contact

**Maintainer**: David Ho  
**Email**: homandat2002@gmail.com  

Feel free to reach out with any questions or feedback!

---

## License

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).