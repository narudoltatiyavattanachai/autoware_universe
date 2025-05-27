#!/bin/bash

# Generate Autoware Universe Codebase Structure
# This script analyzes the Autoware Universe codebase and generates a tree-view
# of its structure, including ROS 2 nodes, services, topics, and key functions.

REPO_ROOT="$(pwd)"
OUTPUT_DIR="${REPO_ROOT}/docs/codebase_structure"
OUTPUT_FILE="${OUTPUT_DIR}/autoware_api_structure.md"

# Create output directory if it doesn't exist
mkdir -p "${OUTPUT_DIR}"

# Copy the Python script
cp /tmp/autoware_codebase_scanner.py ${OUTPUT_DIR}/autoware_codebase_scanner.py
chmod +x ${OUTPUT_DIR}/autoware_codebase_scanner.py

# Generate the structure
echo "Generating Autoware Universe codebase structure..."
python3 ${OUTPUT_DIR}/autoware_codebase_scanner.py "${REPO_ROOT}" > /tmp/raw_structure.txt

# Format the output
cat > "${OUTPUT_FILE}" << 'EOL'
# Autoware Universe Codebase Structure

This document provides a comprehensive tree-view of the Autoware Universe codebase structure, showing:
- Main source files
- ROS 2 nodes, services, and topic publishers/subscribers
- Key public functions and class interfaces

## Repository Structure

The Autoware Universe repository is organized into several key directories:

- `common/`: Shared utilities and interfaces
- `control/`: Vehicle control-related packages
- `localization/`: Localization algorithms and nodes
- `perception/`: Perception modules (object detection, tracking, etc.)
- `planning/`: Path planning and behavioral planning modules
- `sensing/`: Sensor drivers and interfaces
- `system/`: System monitoring and management
- `launch/`: Launch files and configurations

## Detailed API Structure

```
EOL

cat /tmp/raw_structure.txt >> "${OUTPUT_FILE}"

cat >> "${OUTPUT_FILE}" << 'EOL'
```

## How to Use This Document

This document provides a high-level overview of the Autoware Universe codebase structure. It can be used to:
1. Understand the overall organization of the codebase
2. Identify key ROS 2 nodes and their relationships
3. Find important functions and interfaces for each module

For more detailed information, please refer to the individual package documentation and source code.
EOL

echo "Codebase structure generated at ${OUTPUT_FILE}"