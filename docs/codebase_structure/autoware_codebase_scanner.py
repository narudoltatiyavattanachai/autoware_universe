#!/usr/bin/env python3

import os
import re
import sys
from collections import defaultdict

def read_file(file_path):
    """Read file content safely with different encodings"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read()
    except UnicodeDecodeError:
        try:
            with open(file_path, 'r', encoding='latin-1') as f:
                return f.read()
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            return ""

def extract_node_info(content):
    """Extract node class information from file content"""
    # Find node class definition
    node_pattern = re.compile(r'class\s+(\w+)(?:\s*final)?\s*:\s*public\s+rclcpp::Node')
    node_match = node_pattern.search(content)
    
    if not node_match:
        return None
    
    node_name = node_match.group(1)
    
    # Find publishers
    pub_pattern = re.compile(r'rclcpp::Publisher<([^>]+)>::SharedPtr\s+(\w+)_;')
    publishers = pub_pattern.findall(content)
    
    # Find subscribers
    sub_pattern = re.compile(r'rclcpp::Subscription<([^>]+)>::SharedPtr\s+(\w+)_;')
    subscribers = sub_pattern.findall(content)
    
    # Find topic names
    topic_pattern = re.compile(r'(?:create_publisher|create_subscription)<[^>]+>\(\s*"([^"]+)"')
    topics = topic_pattern.findall(content)
    
    # Find public functions
    func_pattern = re.compile(r'public:\s+.*?(?:virtual\s+)?(\w+)\s+(\w+)\s*\(([^)]*)\)', re.DOTALL)
    functions = func_pattern.findall(content)
    
    return {
        'name': node_name,
        'publishers': publishers,
        'subscribers': subscribers,
        'topics': topics,
        'functions': functions
    }

def extract_key_functions(content):
    """Extract key public functions"""
    # Find class name
    class_pattern = re.compile(r'class\s+(\w+)')
    class_match = class_pattern.search(content)
    class_name = class_match.group(1) if class_match else ""
    
    # Find public functions
    function_pattern = re.compile(r'public:\s+.*?(?:virtual\s+)?(\w+)\s+(\w+)\s*\(([^)]*)\)', re.DOTALL)
    functions = function_pattern.findall(content)
    
    # Filter out constructors, destructors, and operators
    key_functions = []
    for return_type, func_name, params in functions:
        if func_name != class_name and not func_name.startswith('~') and not func_name.startswith('operator'):
            key_functions.append((return_type, func_name, params))
    
    return key_functions

def analyze_package(pkg_path):
    """Analyze a ROS 2 package to extract key information"""
    package_info = {
        'nodes': [],
        'source_files': [],
        'launch_files': []
    }
    
    # Process include files
    include_dir = os.path.join(pkg_path, 'include')
    if os.path.isdir(include_dir):
        for root, dirs, files in os.walk(include_dir):
            for file in files:
                if file.endswith(('.hpp', '.h')):
                    file_path = os.path.join(root, file)
                    content = read_file(file_path)
                    
                    node_info = extract_node_info(content)
                    if node_info:
                        package_info['nodes'].append({
                            'file': os.path.relpath(file_path, pkg_path),
                            'info': node_info
                        })
                    
                    # Add key functions if no node found
                    if not node_info:
                        key_functions = extract_key_functions(content)
                        if key_functions:
                            package_info['source_files'].append({
                                'file': os.path.relpath(file_path, pkg_path),
                                'functions': key_functions
                            })
    
    # Process source files
    src_dir = os.path.join(pkg_path, 'src')
    if os.path.isdir(src_dir):
        for root, dirs, files in os.walk(src_dir):
            for file in files:
                if file.endswith(('.cpp', '.cc')):
                    file_path = os.path.join(root, file)
                    content = read_file(file_path)
                    
                    node_info = extract_node_info(content)
                    if node_info:
                        package_info['nodes'].append({
                            'file': os.path.relpath(file_path, pkg_path),
                            'info': node_info
                        })
                    
                    # Add key functions if no node found
                    if not node_info:
                        key_functions = extract_key_functions(content)
                        if key_functions:
                            package_info['source_files'].append({
                                'file': os.path.relpath(file_path, pkg_path),
                                'functions': key_functions
                            })
    
    # Process launch files
    launch_dir = os.path.join(pkg_path, 'launch')
    if os.path.isdir(launch_dir):
        for root, dirs, files in os.walk(launch_dir):
            for file in files:
                if file.endswith(('.launch.xml', '.launch.py')):
                    file_path = os.path.join(root, file)
                    package_info['launch_files'].append(os.path.relpath(file_path, pkg_path))
    
    return package_info

def generate_tree_view(repo_path):
    """Generate a tree view of the Autoware Universe codebase"""
    output = []
    
    # Get main directories
    main_dirs = sorted([d for d in os.listdir(repo_path) 
                      if os.path.isdir(os.path.join(repo_path, d)) 
                      and not d.startswith('.')
                      and d not in ['docs', '.git', '.github']])
    
    # Prioritize key directories
    key_dirs = ['perception', 'planning', 'control', 'localization', 'sensing', 'system']
    main_dirs = sorted([d for d in main_dirs if d in key_dirs]) + \
                sorted([d for d in main_dirs if d not in key_dirs])
    
    # Process each main directory
    for main_dir in main_dirs[:8]:  # Limit to 8 main directories
        main_dir_path = os.path.join(repo_path, main_dir)
        output.append(f"{main_dir}/")
        
        # Get packages (subdirectories)
        packages = sorted([d for d in os.listdir(main_dir_path) 
                        if os.path.isdir(os.path.join(main_dir_path, d)) 
                        and not d.startswith('.')])
        
        for pkg in packages[:5]:  # Limit to 5 packages per directory
            pkg_path = os.path.join(main_dir_path, pkg)
            output.append(f"├── {pkg}/")
            
            # Analyze the package
            pkg_info = analyze_package(pkg_path)
            
            # Add node information first
            for node_entry in pkg_info['nodes'][:3]:  # Limit to 3 nodes
                file_path = node_entry['file']
                node_info = node_entry['info']
                
                description = f" → defines `{node_info['name']}`"
                
                # Add subscriber information
                if node_info['topics']:
                    for topic in node_info['topics'][:1]:  # Just show the first topic
                        if any(sub[1].lower() in topic.lower() for sub in node_info['subscribers']):
                            description += f", subscribes to `{topic}`"
                            break
                
                # Add publisher information
                if node_info['topics']:
                    for topic in node_info['topics'][1:2]:  # Show another topic
                        if any(pub[1].lower() in topic.lower() for pub in node_info['publishers']):
                            description += f", publishes to `{topic}`"
                            break
                
                output.append(f"│   ├── {file_path}{description}")
            
            # Add source files with key functions
            for src_entry in pkg_info['source_files'][:2]:  # Limit to 2 source files
                file_path = src_entry['file']
                functions = src_entry['functions']
                
                if functions:
                    func_name = functions[0][1] if functions[0][1] else ""
                    if func_name:
                        output.append(f"│   ├── {file_path} → `{func_name}()` function")
                    else:
                        output.append(f"│   ├── {file_path}")
                else:
                    output.append(f"│   ├── {file_path}")
            
            # Add launch files
            if pkg_info['launch_files']:
                launch_file = pkg_info['launch_files'][0]
                output.append(f"│   └── launch/ → `{os.path.basename(launch_file)}`")
    
    return "\n".join(output)

def main():
    if len(sys.argv) < 2:
        print("Usage: python autoware_codebase_scanner.py <autoware_repo_path>")
        sys.exit(1)
    
    repo_path = sys.argv[1]
    if not os.path.isdir(repo_path):
        print(f"Error: {repo_path} is not a valid directory")
        sys.exit(1)
    
    tree_view = generate_tree_view(repo_path)
    print(tree_view)

if __name__ == "__main__":
    main()