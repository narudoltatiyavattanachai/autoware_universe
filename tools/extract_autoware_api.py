#!/usr/bin/env python3

"""
Extract Autoware ROS 2 API interfaces and generate documentation.

This script scans the Autoware Universe repository for ROS 2 interface definitions
(messages, services, actions) and extracts information about publishers and subscribers
to generate comprehensive API documentation in Markdown format.

Usage:
    python3 extract_autoware_api.py [--source SOURCE_DIR] [--output OUTPUT_DIR] [--split-by-package]

Example:
    # Generate API documentation
    python3 extract_autoware_api.py --source /path/to/autoware_universe --output /path/to/autoware_universe/docs/ros_api
    
    # Or use from the Makefile
    make extract-autoware-api

Output:
    The script generates documentation in the following structure:
    docs/ros_api/
    ├── msg/
    │   ├── perception_msgs.md
    │   ├── planning_msgs.md
    │   └── ...
    ├── srv/
    │   ├── control_services.md
    │   └── ...
    ├── action/
    │   ├── behavior_actions.md
    │   └── ...
    ├── topics/
    │   ├── full_topic_map.md
    │   ├── perception_topics.md
    │   └── ...
    └── index.md
"""

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple


@dataclass
class MessageField:
    """Information about a message field."""
    field_type: str
    field_name: str
    comment: Optional[str] = None


@dataclass
class MessageInfo:
    """Information about a ROS message."""
    name: str
    path: Path
    fields: List[MessageField]
    description: Optional[str] = None
    package: Optional[str] = None


@dataclass
class ServiceInfo:
    """Information about a ROS service."""
    name: str
    path: Path
    request_fields: List[MessageField]
    response_fields: List[MessageField]
    description: Optional[str] = None
    package: Optional[str] = None


@dataclass
class ActionInfo:
    """Information about a ROS action."""
    name: str
    path: Path
    goal_fields: List[MessageField]
    result_fields: List[MessageField]
    feedback_fields: List[MessageField]
    description: Optional[str] = None
    package: Optional[str] = None


@dataclass
class TopicInfo:
    """Information about a ROS topic."""
    topic_name: str
    message_type: str
    publishers: List[str]
    subscribers: List[str]
    package: Optional[str] = None


class AutowareAPIExtractor:
    """Extract Autoware ROS 2 API interfaces."""

    def __init__(self, source_dir: Path, output_dir: Path):
        """Initialize the extractor with source and output directories."""
        self.source_dir = source_dir
        self.output_dir = output_dir
        
        # Collected information
        self.messages: List[MessageInfo] = []
        self.services: List[ServiceInfo] = []
        self.actions: List[ActionInfo] = []
        self.topics: List[TopicInfo] = []
        
        # Package categorization
        self.package_categories = {
            'perception': [],
            'planning': [],
            'control': [],
            'localization': [],
            'system': [],
            'common': [],
        }

    def extract_all(self):
        """Extract all ROS 2 API interfaces."""
        print(f"Scanning {self.source_dir} for ROS 2 interfaces...")
        
        self.extract_messages()
        self.extract_services()
        self.extract_actions()
        self.extract_topics()
        
        self.categorize_by_package()
        
        self.generate_documentation()
        
        print(f"Documentation generated in {self.output_dir}")

    def extract_messages(self):
        """Extract all message definitions."""
        print("Extracting message definitions...")
        msg_files = list(self.source_dir.glob('**/msg/*.msg'))
        
        for msg_file in msg_files:
            message = self._parse_message_file(msg_file)
            if message:
                self.messages.append(message)
        
        print(f"Found {len(self.messages)} message definitions")

    def _parse_message_file(self, file_path: Path) -> Optional[MessageInfo]:
        """Parse a message file and extract information."""
        try:
            with open(file_path, 'r') as f:
                content = f.read()
            
            # Extract package and message name from path
            parts = file_path.parts
            msg_name = file_path.stem
            
            # Find the package name (typically the parent directory of 'msg')
            package = None
            for i, part in enumerate(parts):
                if part == 'msg' and i > 0:
                    package = parts[i-1]
                    break
            
            # Parse message fields
            fields = []
            description = None
            
            # Look for description in comments at the top of the file
            description_match = re.search(r'^#\s*(.*?)$', content, re.MULTILINE)
            if description_match:
                description = description_match.group(1).strip()
            
            # Parse fields
            lines = content.split('\n')
            for line in lines:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                
                # Try to parse field definition
                field_match = re.match(r'([a-zA-Z0-9_/]+(?:\[[^\]]*\])?)(?:\s+)([a-zA-Z0-9_]+)(?:\s*#\s*(.*))?', line)
                if field_match:
                    field_type = field_match.group(1)
                    field_name = field_match.group(2)
                    comment = field_match.group(3) if field_match.group(3) else None
                    fields.append(MessageField(field_type, field_name, comment))
            
            return MessageInfo(
                name=msg_name,
                path=file_path,
                fields=fields,
                description=description,
                package=package
            )
        
        except Exception as e:
            print(f"Error parsing message file {file_path}: {e}")
            return None

    def extract_services(self):
        """Extract all service definitions."""
        print("Extracting service definitions...")
        srv_files = list(self.source_dir.glob('**/srv/*.srv'))
        
        for srv_file in srv_files:
            service = self._parse_service_file(srv_file)
            if service:
                self.services.append(service)
        
        print(f"Found {len(self.services)} service definitions")

    def _parse_service_file(self, file_path: Path) -> Optional[ServiceInfo]:
        """Parse a service file and extract information."""
        try:
            with open(file_path, 'r') as f:
                content = f.read()
            
            # Split request and response parts
            parts = content.split('---')
            if len(parts) != 2:
                print(f"Warning: Service file {file_path} does not have a request/response separator")
                return None
            
            request_part = parts[0].strip()
            response_part = parts[1].strip()
            
            # Extract package and service name from path
            srv_name = file_path.stem
            
            # Find the package name (typically the parent directory of 'srv')
            package = None
            for i, part in enumerate(file_path.parts):
                if part == 'srv' and i > 0:
                    package = file_path.parts[i-1]
                    break
            
            # Parse request fields
            request_fields = []
            for line in request_part.split('\n'):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                
                field_match = re.match(r'([a-zA-Z0-9_/]+(?:\[[^\]]*\])?)(?:\s+)([a-zA-Z0-9_]+)(?:\s*#\s*(.*))?', line)
                if field_match:
                    field_type = field_match.group(1)
                    field_name = field_match.group(2)
                    comment = field_match.group(3) if field_match.group(3) else None
                    request_fields.append(MessageField(field_type, field_name, comment))
            
            # Parse response fields
            response_fields = []
            for line in response_part.split('\n'):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                
                field_match = re.match(r'([a-zA-Z0-9_/]+(?:\[[^\]]*\])?)(?:\s+)([a-zA-Z0-9_]+)(?:\s*#\s*(.*))?', line)
                if field_match:
                    field_type = field_match.group(1)
                    field_name = field_match.group(2)
                    comment = field_match.group(3) if field_match.group(3) else None
                    response_fields.append(MessageField(field_type, field_name, comment))
            
            # Extract description from comments
            description = None
            description_match = re.search(r'^#\s*(.*?)$', content, re.MULTILINE)
            if description_match:
                description = description_match.group(1).strip()
            
            return ServiceInfo(
                name=srv_name,
                path=file_path,
                request_fields=request_fields,
                response_fields=response_fields,
                description=description,
                package=package
            )
        
        except Exception as e:
            print(f"Error parsing service file {file_path}: {e}")
            return None

    def extract_actions(self):
        """Extract all action definitions."""
        print("Extracting action definitions...")
        action_files = list(self.source_dir.glob('**/action/*.action'))
        
        for action_file in action_files:
            action = self._parse_action_file(action_file)
            if action:
                self.actions.append(action)
        
        print(f"Found {len(self.actions)} action definitions")

    def _parse_action_file(self, file_path: Path) -> Optional[ActionInfo]:
        """Parse an action file and extract information."""
        try:
            with open(file_path, 'r') as f:
                content = f.read()
            
            # Split goal, result, and feedback parts
            parts = content.split('---')
            if len(parts) != 3:
                print(f"Warning: Action file {file_path} does not have goal/result/feedback separators")
                return None
            
            goal_part = parts[0].strip()
            result_part = parts[1].strip()
            feedback_part = parts[2].strip()
            
            # Extract package and action name from path
            action_name = file_path.stem
            
            # Find the package name (typically the parent directory of 'action')
            package = None
            for i, part in enumerate(file_path.parts):
                if part == 'action' and i > 0:
                    package = file_path.parts[i-1]
                    break
            
            # Parse fields for each part
            parse_fields = lambda text: [
                MessageField(
                    match.group(1),
                    match.group(2),
                    match.group(3) if match.group(3) else None
                )
                for line in text.split('\n')
                if (line.strip() and not line.strip().startswith('#'))
                and (match := re.match(r'([a-zA-Z0-9_/]+(?:\[[^\]]*\])?)(?:\s+)([a-zA-Z0-9_]+)(?:\s*#\s*(.*))?', line.strip()))
            ]
            
            goal_fields = parse_fields(goal_part)
            result_fields = parse_fields(result_part)
            feedback_fields = parse_fields(feedback_part)
            
            # Extract description from comments
            description = None
            description_match = re.search(r'^#\s*(.*?)$', content, re.MULTILINE)
            if description_match:
                description = description_match.group(1).strip()
            
            return ActionInfo(
                name=action_name,
                path=file_path,
                goal_fields=goal_fields,
                result_fields=result_fields,
                feedback_fields=feedback_fields,
                description=description,
                package=package
            )
        
        except Exception as e:
            print(f"Error parsing action file {file_path}: {e}")
            return None

    def extract_topics(self):
        """Extract topic information from publishers and subscribers."""
        print("Extracting topic information...")
        # Find all C++ files
        cpp_files = list(self.source_dir.glob('**/*.cpp'))
        hpp_files = list(self.source_dir.glob('**/*.hpp'))
        
        # Dictionary to store topic info by topic name
        topic_info_dict = {}
        
        # Extract publishers and subscribers
        for file_path in cpp_files + hpp_files:
            try:
                with open(file_path, 'r', errors='ignore') as f:
                    content = f.read()
                
                # Extract package from file path by looking at the directory structure
                package = None
                path_str = str(file_path)
                path_parts = path_str.split('/')

                # Try to determine the package more precisely based on path components
                for category in ['perception', 'planning', 'control', 'localization', 'system']:
                    if category in path_parts:
                        package = category
                        break
            
                # If we couldn't determine from the path, use a fallback based on component names
                if not package:
                    if any(comp in path_str for comp in ['perception', 'detect', 'track', 'classify', 'segment']):
                        package = 'perception'
                    elif any(comp in path_str for comp in ['plan', 'route', 'mission', 'behavior', 'path']):
                        package = 'planning'
                    elif any(comp in path_str for comp in ['control', 'vehicle_cmd', 'accel', 'brake', 'steer']):
                        package = 'control'
                    elif any(comp in path_str for comp in ['localization', 'pose', 'position', 'location']):
                        package = 'localization'
                    elif any(comp in path_str for comp in ['system', 'monitor', 'launcher', 'tool']):
                        package = 'system'
                    else:
                        package = 'common'
                
                # Find publisher patterns
                # Examples:
                # create_publisher<MessageType>("topic_name", 10)
                # node->create_publisher<MessageType>("topic_name", 10)
                pub_patterns = [
                    r'create_publisher\s*<\s*([a-zA-Z0-9_:]+)\s*>\s*\(\s*"([^"]+)"',
                    r'create_publisher\s*<\s*([a-zA-Z0-9_:]+)\s*>\s*\(\s*this\s*,\s*"([^"]+)"',
                    r'->create_publisher\s*<\s*([a-zA-Z0-9_:]+)\s*>\s*\(\s*"([^"]+)"',
                ]
                
                for pattern in pub_patterns:
                    for match in re.finditer(pattern, content):
                        msg_type = match.group(1)
                        topic_name = match.group(2)
                        
                        if topic_name not in topic_info_dict:
                            topic_info_dict[topic_name] = TopicInfo(
                                topic_name=topic_name,
                                message_type=msg_type,
                                publishers=[str(file_path)],
                                subscribers=[],
                                package=package
                            )
                        else:
                            # Update existing info
                            topic_info = topic_info_dict[topic_name]
                            if str(file_path) not in topic_info.publishers:
                                topic_info.publishers.append(str(file_path))
                
                # Find subscriber patterns
                # Examples:
                # create_subscription<MessageType>("topic_name", 10, callback)
                # node->create_subscription<MessageType>("topic_name", 10, callback)
                sub_patterns = [
                    r'create_subscription\s*<\s*([a-zA-Z0-9_:]+)\s*>\s*\(\s*"([^"]+)"',
                    r'create_subscription\s*<\s*([a-zA-Z0-9_:]+)\s*>\s*\(\s*this\s*,\s*"([^"]+)"',
                    r'->create_subscription\s*<\s*([a-zA-Z0-9_:]+)\s*>\s*\(\s*"([^"]+)"',
                ]
                
                for pattern in sub_patterns:
                    for match in re.finditer(pattern, content):
                        msg_type = match.group(1)
                        topic_name = match.group(2)
                        
                        if topic_name not in topic_info_dict:
                            topic_info_dict[topic_name] = TopicInfo(
                                topic_name=topic_name,
                                message_type=msg_type,
                                publishers=[],
                                subscribers=[str(file_path)],
                                package=package
                            )
                        else:
                            # Update existing info
                            topic_info = topic_info_dict[topic_name]
                            if msg_type and not topic_info.message_type:
                                topic_info.message_type = msg_type
                            if str(file_path) not in topic_info.subscribers:
                                topic_info.subscribers.append(str(file_path))
            
            except Exception as e:
                print(f"Error processing file {file_path}: {e}")
        
        # Convert to list
        self.topics = list(topic_info_dict.values())
        print(f"Found {len(self.topics)} topics")

    def categorize_by_package(self):
        """Categorize interfaces by package."""
        print("Categorizing interfaces by package...")
        
        # Helper function to determine category from package name
        def get_category(package_name):
            if not package_name:
                return 'common'
            
            package_name = package_name.lower()
            for category in self.package_categories.keys():
                if category in package_name:
                    return category
            return 'common'
        
        # Categorize messages
        for message in self.messages:
            category = get_category(message.package)
            self.package_categories[category].append(('message', message))
        
        # Categorize services
        for service in self.services:
            category = get_category(service.package)
            self.package_categories[category].append(('service', service))
        
        # Categorize actions
        for action in self.actions:
            category = get_category(action.package)
            self.package_categories[category].append(('action', action))
        
        # Categorize topics
        for topic in self.topics:
            category = get_category(topic.package)
            self.package_categories[category].append(('topic', topic))

    def generate_documentation(self):
        """Generate documentation files."""
        print("Generating documentation files...")
        
        # Generate message documentation
        self._generate_message_docs()
        
        # Generate service documentation
        self._generate_service_docs()
        
        # Generate action documentation
        self._generate_action_docs()
        
        # Generate topic documentation
        self._generate_topic_docs()
        
        print("Documentation generation complete.")

    def _generate_message_docs(self):
        """Generate message documentation files."""
        # Group messages by category
        for category, items in self.package_categories.items():
            messages = [item[1] for item in items if item[0] == 'message']
            
            if not messages:
                # Create empty file
                with open(self.output_dir / 'msg' / f"{category}_msgs.md", 'w') as f:
                    f.write(f"# {category.capitalize()} Messages\n\n")
                    f.write("No message definitions found for this category.\n")
                continue
            
            # Create message docs
            with open(self.output_dir / 'msg' / f"{category}_msgs.md", 'w') as f:
                f.write(f"# {category.capitalize()} Messages\n\n")
                f.write(f"This document lists all message definitions used in the {category} category.\n\n")
                
                # Table of contents
                f.write("## Message Types\n\n")
                for message in sorted(messages, key=lambda m: m.name):
                    f.write(f"- [{message.name}](#{message.name.lower()})\n")
                f.write("\n")
                
                # Message details
                f.write("## Message Details\n\n")
                for message in sorted(messages, key=lambda m: m.name):
                    f.write(f"### {message.name}\n\n")
                    
                    if message.description:
                        f.write(f"{message.description}\n\n")
                    
                    f.write(f"**File:** `{message.path.relative_to(self.source_dir)}`\n\n")
                    
                    f.write("**Fields:**\n\n")
                    f.write("| Type | Name | Description |\n")
                    f.write("| ---- | ---- | ----------- |\n")
                    
                    for field in message.fields:
                        description = field.comment if field.comment else ""
                        f.write(f"| `{field.field_type}` | `{field.field_name}` | {description} |\n")
                    
                    f.write("\n")

    def _generate_service_docs(self):
        """Generate service documentation files."""
        # Group services by category
        for category, items in self.package_categories.items():
            services = [item[1] for item in items if item[0] == 'service']
            
            if not services:
                # Create empty file
                with open(self.output_dir / 'srv' / f"{category}_services.md", 'w') as f:
                    f.write(f"# {category.capitalize()} Services\n\n")
                    f.write("No service definitions found for this category.\n")
                continue
            
            # Create service docs
            with open(self.output_dir / 'srv' / f"{category}_services.md", 'w') as f:
                f.write(f"# {category.capitalize()} Services\n\n")
                f.write(f"This document lists all service definitions used in the {category} category.\n\n")
                
                # Table of contents
                f.write("## Service Types\n\n")
                for service in sorted(services, key=lambda s: s.name):
                    f.write(f"- [{service.name}](#{service.name.lower()})\n")
                f.write("\n")
                
                # Service details
                f.write("## Service Details\n\n")
                for service in sorted(services, key=lambda s: s.name):
                    f.write(f"### {service.name}\n\n")
                    
                    if service.description:
                        f.write(f"{service.description}\n\n")
                    
                    f.write(f"**File:** `{service.path.relative_to(self.source_dir)}`\n\n")
                    
                    f.write("**Request:**\n\n")
                    f.write("| Type | Name | Description |\n")
                    f.write("| ---- | ---- | ----------- |\n")
                    
                    for field in service.request_fields:
                        description = field.comment if field.comment else ""
                        f.write(f"| `{field.field_type}` | `{field.field_name}` | {description} |\n")
                    
                    f.write("\n**Response:**\n\n")
                    f.write("| Type | Name | Description |\n")
                    f.write("| ---- | ---- | ----------- |\n")
                    
                    for field in service.response_fields:
                        description = field.comment if field.comment else ""
                        f.write(f"| `{field.field_type}` | `{field.field_name}` | {description} |\n")
                    
                    f.write("\n")

    def _generate_action_docs(self):
        """Generate action documentation files."""
        # Group actions by category
        for category, items in self.package_categories.items():
            actions = [item[1] for item in items if item[0] == 'action']
            
            if not actions:
                # Create empty file
                with open(self.output_dir / 'action' / f"{category}_actions.md", 'w') as f:
                    f.write(f"# {category.capitalize()} Actions\n\n")
                    f.write("No action definitions found for this category.\n")
                continue
            
            # Create action docs
            with open(self.output_dir / 'action' / f"{category}_actions.md", 'w') as f:
                f.write(f"# {category.capitalize()} Actions\n\n")
                f.write(f"This document lists all action definitions used in the {category} category.\n\n")
                
                # Table of contents
                f.write("## Action Types\n\n")
                for action in sorted(actions, key=lambda a: a.name):
                    f.write(f"- [{action.name}](#{action.name.lower()})\n")
                f.write("\n")
                
                # Action details
                f.write("## Action Details\n\n")
                for action in sorted(actions, key=lambda a: a.name):
                    f.write(f"### {action.name}\n\n")
                    
                    if action.description:
                        f.write(f"{action.description}\n\n")
                    
                    f.write(f"**File:** `{action.path.relative_to(self.source_dir)}`\n\n")
                    
                    f.write("**Goal:**\n\n")
                    f.write("| Type | Name | Description |\n")
                    f.write("| ---- | ---- | ----------- |\n")
                    
                    for field in action.goal_fields:
                        description = field.comment if field.comment else ""
                        f.write(f"| `{field.field_type}` | `{field.field_name}` | {description} |\n")
                    
                    f.write("\n**Result:**\n\n")
                    f.write("| Type | Name | Description |\n")
                    f.write("| ---- | ---- | ----------- |\n")
                    
                    for field in action.result_fields:
                        description = field.comment if field.comment else ""
                        f.write(f"| `{field.field_type}` | `{field.field_name}` | {description} |\n")
                    
                    f.write("\n**Feedback:**\n\n")
                    f.write("| Type | Name | Description |\n")
                    f.write("| ---- | ---- | ----------- |\n")
                    
                    for field in action.feedback_fields:
                        description = field.comment if field.comment else ""
                        f.write(f"| `{field.field_type}` | `{field.field_name}` | {description} |\n")
                    
                    f.write("\n")

    def _generate_topic_docs(self):
        """Generate topic documentation files."""
        # Create full topic map
        with open(self.output_dir / 'topics' / 'full_topic_map.md', 'w') as f:
            f.write("# Full Topic Map\n\n")
            f.write("This document provides a complete overview of all topics used in Autoware Universe.\n\n")
            
            f.write("| Topic | Message Type | Publishers | Subscribers |\n")
            f.write("| ----- | ------------ | ---------- | ----------- |\n")
            
            for topic in sorted(self.topics, key=lambda t: t.topic_name):
                publishers = ", ".join([os.path.basename(p) for p in topic.publishers[:2]])
                if len(topic.publishers) > 2:
                    publishers += f" and {len(topic.publishers) - 2} more"
                
                subscribers = ", ".join([os.path.basename(s) for s in topic.subscribers[:2]])
                if len(topic.subscribers) > 2:
                    subscribers += f" and {len(topic.subscribers) - 2} more"
                
                f.write(f"| `{topic.topic_name}` | `{topic.message_type}` | {publishers} | {subscribers} |\n")
        
        # Group topics by category
        for category, items in self.package_categories.items():
            topics = [item[1] for item in items if item[0] == 'topic']
            
            if not topics:
                # Create empty file
                with open(self.output_dir / 'topics' / f"{category}_topics.md", 'w') as f:
                    f.write(f"# {category.capitalize()} Topics\n\n")
                    f.write("No topics found for this category.\n")
                continue
            
            # Create topic docs
            with open(self.output_dir / 'topics' / f"{category}_topics.md", 'w') as f:
                f.write(f"# {category.capitalize()} Topics\n\n")
                f.write(f"This document lists all topics used in the {category} category.\n\n")
                
                f.write("| Topic | Message Type | Publishers | Subscribers |\n")
                f.write("| ----- | ------------ | ---------- | ----------- |\n")
                
                for topic in sorted(topics, key=lambda t: t.topic_name):
                    publishers = ", ".join([os.path.basename(p) for p in topic.publishers[:2]])
                    if len(topic.publishers) > 2:
                        publishers += f" and {len(topic.publishers) - 2} more"
                    
                    subscribers = ", ".join([os.path.basename(s) for s in topic.subscribers[:2]])
                    if len(topic.subscribers) > 2:
                        subscribers += f" and {len(topic.subscribers) - 2} more"
                    
                    f.write(f"| `{topic.topic_name}` | `{topic.message_type}` | {publishers} | {subscribers} |\n")
    
    def export_to_json(self):
        """Export API information to JSON format."""
        print("Exporting API information to JSON...")
        
        # Create output directory
        json_dir = self.output_dir / 'json'
        json_dir.mkdir(exist_ok=True)
        
        # Helper function to convert dataclass to dict
        def dataclass_to_dict(obj):
            if hasattr(obj, '__dataclass_fields__'):
                result = {}
                for field in obj.__dataclass_fields__:
                    value = getattr(obj, field)
                    if field == 'path' and isinstance(value, Path):
                        result[field] = str(value.relative_to(self.source_dir))
                    elif isinstance(value, list):
                        result[field] = [dataclass_to_dict(item) for item in value]
                    else:
                        result[field] = value
                return result
            return obj
        
        # Export messages
        messages_json = [dataclass_to_dict(message) for message in self.messages]
        with open(json_dir / 'messages.json', 'w') as f:
            json.dump(messages_json, f, indent=2)
        
        # Export services
        services_json = [dataclass_to_dict(service) for service in self.services]
        with open(json_dir / 'services.json', 'w') as f:
            json.dump(services_json, f, indent=2)
        
        # Export actions
        actions_json = [dataclass_to_dict(action) for action in self.actions]
        with open(json_dir / 'actions.json', 'w') as f:
            json.dump(actions_json, f, indent=2)
        
        # Export topics
        topics_json = []
        for topic in self.topics:
            topic_dict = {
                'topic_name': topic.topic_name,
                'message_type': topic.message_type,
                'publishers': topic.publishers,
                'subscribers': topic.subscribers,
                'package': topic.package,
            }
            topics_json.append(topic_dict)
        
        with open(json_dir / 'topics.json', 'w') as f:
            json.dump(topics_json, f, indent=2)
        
        # Export package categories
        package_categories_json = {}
        for category, items in self.package_categories.items():
            package_categories_json[category] = []
            for item_type, item in items:
                if hasattr(item, 'name'):
                    package_categories_json[category].append({
                        'type': item_type,
                        'name': item.name,
                    })
        
        with open(json_dir / 'package_categories.json', 'w') as f:
            json.dump(package_categories_json, f, indent=2)
        
        print(f"JSON export completed to {json_dir}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Extract Autoware ROS 2 API interfaces and generate documentation."
    )
    parser.add_argument(
        "--source", 
        type=str, 
        default="/home/runner/work/autoware_universe/autoware_universe",
        help="Source directory containing the Autoware Universe repository."
    )
    parser.add_argument(
        "--output", 
        type=str, 
        default="/home/runner/work/autoware_universe/autoware_universe/docs/ros_api",
        help="Output directory for the generated documentation."
    )
    parser.add_argument(
        "--split-by-package",
        action="store_true",
        help="Split documentation files by package."
    )
    parser.add_argument(
        "--format",
        type=str,
        choices=["markdown", "json", "both"],
        default="markdown",
        help="Output format for the API documentation."
    )
    
    args = parser.parse_args()
    
    source_dir = Path(args.source)
    output_dir = Path(args.output)
    
    if not source_dir.exists():
        print(f"Error: Source directory {source_dir} does not exist.")
        sys.exit(1)
    
    # Create output directory if it doesn't exist
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Create subdirectories
    (output_dir / 'msg').mkdir(exist_ok=True)
    (output_dir / 'srv').mkdir(exist_ok=True)
    (output_dir / 'action').mkdir(exist_ok=True)
    (output_dir / 'topics').mkdir(exist_ok=True)
    
    # Create JSON output directory if needed
    if args.format in ["json", "both"]:
        (output_dir / 'json').mkdir(exist_ok=True)
    
    # Extract API interfaces
    extractor = AutowareAPIExtractor(source_dir, output_dir)
    extractor.extract_all()
    
    # Export to JSON if requested
    if args.format in ["json", "both"]:
        extractor.export_to_json()


if __name__ == "__main__":
    main()