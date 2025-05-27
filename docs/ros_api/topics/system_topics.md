# System Topics

This document lists all topics used in the system category.

| Topic | Message Type | Publishers | Subscribers |
| ----- | ------------ | ---------- | ----------- |
| `/api/system/diagnostics/status` | `autoware_adapi_v1_msgs::msg::DiagGraphStatus` | diagnostics.cpp |  |
| `/api/system/diagnostics/struct` | `autoware_adapi_v1_msgs::msg::DiagGraphStruct` | diagnostics.cpp |  |
| `/autoware/state` | `AutowareState` | autoware_state.cpp |  |
| `/diagnostics_array` | `DiagnosticArray` | converter.cpp |  |
| `/diagnostics_graph/status` | `DiagGraphStatus` | aggregator.cpp | subscription.cpp |
| `/diagnostics_graph/struct` | `DiagGraphStruct` | aggregator.cpp | subscription.cpp |
| `/diagnostics_graph/unknowns` | `DiagnosticArray` | aggregator.cpp |  |
| `/external/` | `OperatorHeartbeat` | manual_control.cpp |  |
| `/service_log` | `ServiceLog` | interface.hpp | service_log_checker.cpp |
| `/system/operation_mode/availability` | `Availability` | availability.cpp |  |
| `input/ground_truth_pose` | `PoseStamped` |  | reaction_analyzer_node.cpp |
| `input/kinematics` | `Odometry` |  | reaction_analyzer_node.cpp |
| `input/localization_initialization_state` | `LocalizationInitializationState` |  | reaction_analyzer_node.cpp |
| `input/operation_mode_state` | `OperationModeState` |  | reaction_analyzer_node.cpp |
| `input/routing_state` | `RouteState` |  | reaction_analyzer_node.cpp |
| `output/goal` | `geometry_msgs::msg::PoseStamped` | reaction_analyzer_node.cpp |  |
| `output/initialpose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | reaction_analyzer_node.cpp |  |
| `output/pointcloud` | `PointCloud2` | topic_publisher.cpp |  |
| `~/component_system_usage` | `ResourceUsageReport` | component_monitor_node.cpp |  |
| `~/cpu_usage` | `tier4_external_api_msgs::msg::CpuUsage` | cpu_monitor_base.cpp |  |
| `~/debug` | `visualization_msgs::msg::Marker` | reaction_analyzer_node.cpp |  |
| `~/debug/error_graph_text` | `autoware_internal_debug_msgs::msg::StringStamped` | logging.cpp |  |
| `~/debug/processing_time_tree` | `autoware::universe_utils::ProcessingTimeDetail` | test_time_keeper.cpp |  |
| `~/hazard_status` | `HazardStatusStamped` | converter.cpp |  |
| `~/initialpose` | `PoseWithCovarianceStamped` |  | initial_pose_adaptor.cpp |
| `~/input/control/control_cmd` | `Control` |  | mrm_emergency_stop_operator_core.cpp |
| `~/input/fixed_goal` | `PoseStamped` |  | routing_adaptor.cpp |
| `~/input/operation_mode_availability` | `tier4_system_msgs::msg::OperationModeAvailability` |  | mrm_handler_core.cpp |
| `~/input/reroute` | `PoseStamped` |  | routing_adaptor.cpp |
| `~/input/rough_goal` | `PoseStamped` |  | routing_adaptor.cpp |
| `~/input/waypoint` | `PoseStamped` |  | routing_adaptor.cpp |
| `~/output/emergency_holding` | `tier4_system_msgs::msg::EmergencyHoldingState` | mrm_handler_core.cpp |  |
| `~/output/gear` | `autoware_vehicle_msgs::msg::GearCommand` | mrm_handler_core.cpp |  |
| `~/output/hazard` | `autoware_vehicle_msgs::msg::HazardLightsCommand` | mrm_handler_core.cpp |  |
| `~/output/mrm/comfortable_stop/status` | `tier4_system_msgs::msg::MrmBehaviorStatus` | mrm_comfortable_stop_operator_core.cpp |  |
| `~/output/mrm/emergency_stop/control_cmd` | `Control` | mrm_emergency_stop_operator_core.cpp |  |
| `~/output/mrm/emergency_stop/status` | `MrmBehaviorStatus` | mrm_emergency_stop_operator_core.cpp |  |
| `~/output/mrm/state` | `autoware_adapi_v1_msgs::msg::MrmState` | mrm_handler_core.cpp |  |
| `~/output/state_array` | `VirtualTrafficLightStateArray` | dummy_infrastructure_node.cpp |  |
| `~/output/turn_indicators` | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` | mrm_handler_core.cpp |  |
| `~/output/velocity_limit/clear` | `autoware_internal_planning_msgs::msg::VelocityLimitClearCommand` | mrm_comfortable_stop_operator_core.cpp |  |
