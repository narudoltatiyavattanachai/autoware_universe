# Autoware Universe ROS 2 API Documentation

This directory contains comprehensive documentation of the ROS 2 interfaces used in Autoware Universe.

## Structure

- [msg/](./msg/) - Documentation of custom message definitions
- [srv/](./srv/) - Documentation of service interfaces
- [action/](./action/) - Documentation of action definitions
- [topics/](./topics/) - Documentation of topic communication patterns

## Usage

For a complete overview of the ROS 2 API, see the [index.md](./index.md) file.

## Generating Documentation

The documentation in this directory is generated using the `extract_autoware_api.py` script located in the `tools/` directory. To update the documentation, run:

```bash
cd /path/to/autoware_universe
python3 tools/extract_autoware_api.py --source ./src --output ./docs/ros_api
```

## Contributing

When adding new ROS 2 interfaces (messages, services, actions) to Autoware Universe, please update the API documentation by running the above script.