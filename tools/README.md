# Autoware Universe Tools

This directory contains various tools for working with the Autoware Universe repository.

## API Documentation Generator

### `extract_autoware_api.py`

This script scans the Autoware Universe repository for ROS 2 interfaces and generates Markdown documentation.

#### Usage

```bash
python3 extract_autoware_api.py [--source SOURCE_DIR] [--output OUTPUT_DIR] [--split-by-package]
```

#### Options

- `--source`: Source directory containing the Autoware Universe repository (default: current directory)
- `--output`: Output directory for the generated documentation (default: `./docs/ros_api`)
- `--split-by-package`: Split documentation files by package

#### Example

```bash
# Generate API documentation
python3 extract_autoware_api.py --source ./src --output ./docs/ros_api
```

#### Generated Documentation Structure

The script generates documentation in the following structure:

```
docs/ros_api/
├── msg/
│   ├── perception_msgs.md
│   ├── planning_msgs.md
│   └── ...
├── srv/
│   ├── control_services.md
│   ├── localization_services.md
│   └── ...
├── action/
│   ├── behavior_actions.md
│   ├── planning_actions.md
│   └── ...
├── topics/
│   ├── full_topic_map.md
│   ├── perception_topics.md
│   ├── planning_topics.md
│   └── ...
└── index.md  # Overview index linking to all API artifacts
```

#### Integration with CI/CD

To keep the API documentation up-to-date, you can add the following to your GitHub Actions workflow:

```yaml
- name: Generate API Documentation
  run: |
    python3 tools/extract_autoware_api.py
```