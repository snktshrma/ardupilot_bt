# Behavior Tree Architecture

This document details the behavior tree (BT) system implemented for Ardupilot. The design follows the Navigation2 (Nav2) approach and is split across several packages that work together to execute decision logic through behavior tree nodes.

## Packages Overview

1. **ardupilot_bt** : Contains the core `BehaviorTreeEngine` which wraps the BehaviorTree.CPP library. It registers plugins, creates trees from XML, and ticks the tree.
2. **ardupilot_bt_nodes** : Holds the node plugin library. It include actions, conditions, controls and decorators, for now mainly :  `FailsafeCondition` and `UnloadCmgAction`.
3. **ardupilot_bt_navigator** : ROS2 component that holds a `BehaviorTreeEngine` instance. It loads the plugin library at startup and runs specific trees to control the APs subsystems.
4. **ardupilot_utils** : Collection of helper functions shared across the stack (parameter utilities, plugin loading helpers and more).
5. **ardupilot_core** : Houses abstract plugin interfaces. Subsystems implement these base classes so that the navigator can load plugins in a generic manner.
6. **ardupilot_lifecycle_manager** : Brings up and shuts down nodes using the ROS 2 managed lifecycle. It ensures plugins and other nodes are configured and activated before a mission starts.

```
BtNavigator ---> BehaviorTreeEngine ---> Behavior Tree ---> <Plugins | Actions | Conditions | Decorators>
```

## BehaviorTreeEngine

The engine is started with a list of plugin libraries. In construction it registers all available node types with the BehaviorTree.CPP factory. Trees are created from XML strings using the factory and executed by repeatedly ticking the root node until returns success or failure.

Key API functiosn:
- `BehaviorTreeEngine(const std::vector<std::string>& plugin_libraries)` : loads plugins.
- `BT::Tree createTreeFromText(const std::string & xml, BT::Blackboard::Ptr bb)` : builds a tree from the BT XML.
- `Status run(BT::Tree * tree)` : ticks the tree at 10hz until it finishes.

## BtNavigator

`BtNavigator` is a lifecycle node. During configuration it creates a `BehaviorTreeEngine` and declares parameters such as `failsafe`. When activated it spawns a thread to run the behavior tree by calling `execute()`.

This can be launched on its own or with other nodes. When the tree is running, individual plugins interact with other subsystems through topics, services or mainly actions.

## Node Plugins

`ardupilot_bt_nodes` exposes behaviors used in trees. Each plugin registers itself with the factory so the XML parser can start it. Some examples:

- **ExampleAction** : A minimal synchronous action specifically for demonstration and starting point for other SSOS subsystems and developers to build upon.
- **ExampleCondition** : returns success/failure based on defined logic.
- **ExampleDecorator / ExampleControl** : Basic dev how decorators and control nodes are written.
- **FailsafeCondition** : It reads a `failsafe` boolean from its input port. When the value is `true` the condition fails, causing the tree to fall back to recovery behavior.
- **UnloadCmgAction** : publishes a message on `gnc/unload_cmg` to trigger CMG unloading.

Registration occurs in each source file using `BT_REGISTER_NODES`, so the factory can create the node when parsing XML.

## Supporting Packages

Several additional packages mirror the Navigation2 architecture to simplify development and deployment:

- **ardupilot_utils** provides helper functions such as parameter declaration utilities and plugin loaders used by many nodes.
  The `PluginLoader` wrapper simplifies creating instances from pluginlib while
  logging meaningful errors if loading fails.
- **ardupilot_core** defines abstract base classes for plugins. Subsystems implement these interfaces so the navigator and other components can load them generically.
- **ardupilot_lifecycle_manager** implements a node that brings up managed nodes through the ROS&nbsp;2 lifecycle. It configures and activates a list of nodes on startup and can cleanly shut them down.

## Example Workflow

The tree in `ardupilot_bt_navigator/failsafe_tree.xml` describes how a sequence can fallback to a recovery action. The flow is:

1. `BtNavigator` loads the example XML at startup.
2. `FailsafeCondition` checks the `failsafe` boolean from the blackboard. When `false`, `ExampleAction` runs and the tree succeeds.
3. If `FailsafeCondition` returns failure, control moves to the fallback branch where `UnloadCmgAction` publishes a command.
4. The GNC `torque_controller` node subscribes to `gnc/unload_cmg`. On receiving the message it starts the CMG unloading logic.

## Extending the System

Additional behaviors can be implemented by adding new plugins to `ardupilot_bt_nodes`. After building the workspace, they become available for use in any XML tree. The navigator can be configured to load different trees or parameterize them through the blackboard.

For further reference, see the code in:
- `ardupilot_bt/` : core engine implementation
- `ardupilot_bt_nodes/` : plugin examples
- `ardupilot_bt_navigator/` : navigator component and example tree
- `ardupilot_utils/` : miscellaneous helper routines used by multiple nodes
- `ardupilot_core/` : plugin interface definitions
- `ardupilot_lifecycle_manager/` : lifecycle management utility node
