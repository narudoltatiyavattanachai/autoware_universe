# Autoware Universe Codebase Structure Tools

This directory contains tools for analyzing and documenting the structure of the Autoware Universe codebase.

## Available Tools

### `autoware_codebase_scanner.py`

A Python script that scans the Autoware Universe repository and generates a tree view of its structure, including:
- Main source files
- ROS 2 nodes, services, and topic publishers/subscribers
- Key public functions and class interfaces

#### Usage

```bash
# Run from the repository root
python3 docs/codebase_structure/autoware_codebase_scanner.py /path/to/autoware_universe > output.txt
```

### Generated Documentation

- `autoware_api_structure.md`: A comprehensive tree view of the codebase structure.

## Regenerating Documentation

To regenerate the codebase structure documentation:

```bash
cd /path/to/autoware_universe
bash docs/codebase_structure/generate_codebase_structure.sh
```

This will update the `autoware_api_structure.md` file with the current structure of the codebase.

## Purpose

These tools are designed to help developers understand the codebase structure, refactor dependencies, or generate documentation for new developers.