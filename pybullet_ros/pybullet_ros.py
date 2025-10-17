#!/usr/bin/env python3

import importlib
import os
from threading import Event, Thread
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from pybullet_ros.function_exec_manager import FuncExecManager


class PyBulletRosNode(Node):
    """ROS 2 wrapper class for the pybullet simulator."""

    def __init__(self) -> None:
        super().__init__('pybullet_ros')
        self.pb = importlib.import_module('pybullet')

        self.declare_parameter('config_file', '')
        self.config_data = self._load_configuration()

        self.loop_rate = self._declare_with_config('loop_rate', 80.0)
        self.plugin_import_prefix = self._declare_with_config('plugin_import_prefix', 'pybullet_ros.plugins')
        self.environment_module = self._declare_with_config('environment', 'environment')
        self.pybullet_gui = self._declare_with_config('pybullet_gui', True)
        self.pause_simulation = self._declare_with_config('pause_simulation', False)
        self.parallel_plugin_execution = self._declare_with_config('parallel_plugin_execution', True)
        self.robot_pose_x = self._declare_with_config('robot_pose_x', 0.0)
        self.robot_pose_y = self._declare_with_config('robot_pose_y', 0.0)
        self.robot_pose_z = self._declare_with_config('robot_pose_z', 1.0)
        self.robot_pose_yaw = self._declare_with_config('robot_pose_yaw', 0.0)
        self.fixed_base = self._declare_with_config('fixed_base', False)
        self.robot_urdf_path = self._declare_with_config('robot_urdf_path', '')
        self.use_deformable_world = self._declare_with_config('use_deformable_world', False)
        self.gui_options = self._declare_with_config('gui_options', '')

        self.plugins: List[Any] = []
        self.connected_to_physics_server: bool = False
        self.physics_client = None
        self._temp_urdf_files: List[str] = []

        self._stop_event = Event()
        self._executor_thread: Thread | None = None
        self._timer = None

        self.create_service(Empty, 'reset_simulation', self.handle_reset_simulation)
        self.create_service(Empty, 'pause_physics', self.handle_pause_physics)
        self.create_service(Empty, 'unpause_physics', self.handle_unpause_physics)

        self._initialize_world()
        self._load_plugins()

    def _declare_with_config(self, name: str, default: Any) -> Any:
        value = self.config_data.get(name, default)
        param = self.declare_parameter(name, value)
        return param.value

    def _load_configuration(self) -> Dict[str, Any]:
        config_file = self.get_parameter('config_file').value
        if not config_file:
            return {}
        config_path = os.path.expanduser(os.path.expandvars(config_file))
        if not os.path.isfile(config_path):
            self.get_logger().warning(f'Configuration file {config_file} does not exist')
            return {}
        try:
            import yaml  # type: ignore
        except ImportError as exc:  # pragma: no cover - dependency error is surfaced to the user
            self.get_logger().error(
                'Unable to import yaml to parse configuration file. '
                'Please install python3-yaml or remove the config_file parameter.'
            )
            raise exc
        with open(config_path, 'r', encoding='utf-8') as config_stream:
            data = yaml.safe_load(config_stream) or {}
        if not isinstance(data, dict):
            self.get_logger().warning('Configuration file did not produce a dictionary, ignoring contents')
            return {}
        return data

    def _initialize_world(self) -> None:
        self.get_logger().info('Starting pybullet...')
        gui = bool(self.pybullet_gui)
        self.physics_client = self._start_gui(gui=gui)
        if self.physics_client < 0:
            self.get_logger().error('Failed to connect to pybullet physics server')
            return
        self.connected_to_physics_server = True
        self.pb.setAdditionalSearchPath(importlib.import_module('pybullet_data').getDataPath())
        self.environment = self._load_environment_plugin()
        self.robot = self._init_pybullet_robot()
        if not self.robot:
            self.connected_to_physics_server = False
            return
        self.rev_joint_index_name_dic, self.prismatic_joint_index_name_dic, \
            self.fixed_joint_index_name_dic, self.link_names_to_ids_dic = self._get_properties()

    def _load_environment_plugin(self):
        plugin_module = f'{self.plugin_import_prefix}.{self.environment_module}'
        environment_cls = getattr(importlib.import_module(plugin_module), 'Environment')
        return environment_cls(self, self.pb, use_deformable_world=self.use_deformable_world,
                               config=self.config_data)

    def _load_plugins(self) -> None:
        plugin_entries = self.config_data.get('plugins', [])
        if not plugin_entries:
            self.get_logger().warn('No plugins configured; simulation will run without ROS interfaces')
            return
        for plugin in plugin_entries:
            module_name = plugin.get('module')
            class_name = plugin.get('class')
            if not module_name or not class_name:
                self.get_logger().warn('Plugin entry missing module or class; skipping')
                continue
            try:
                module = importlib.import_module(module_name)
                plugin_cls = getattr(module, class_name)
            except (ImportError, AttributeError) as exc:
                self.get_logger().error(f'Failed to import plugin {class_name} from {module_name}: {exc}')
                continue
            plugin_config = {key: value for key, value in plugin.items() if key not in {'module', 'class'}}
            namespace_name = plugin_config.get('name')
            namespace_config = self.config_data.get(namespace_name, {}) if namespace_name else {}
            self.get_logger().info(f'Loading plugin: {class_name} from {module_name}')
            try:
                plugin_obj = plugin_cls(
                    self,
                    self.pb,
                    self.robot,
                    rev_joints=self.rev_joint_index_name_dic,
                    prism_joints=self.prismatic_joint_index_name_dic,
                    fixed_joints=self.fixed_joint_index_name_dic,
                    link_ids=self.link_names_to_ids_dic,
                    plugin_config=plugin_config,
                    parameters=namespace_config,
                    global_parameters=self.config_data,
                )
                self.plugins.append(plugin_obj)
            except Exception as exc:  # pragma: no cover - plugin specific failures are logged
                self.get_logger().error(f'Failed to initialise plugin {class_name}: {exc}')

    def start(self) -> None:
        if not self.connected_to_physics_server:
            self.get_logger().error('Cannot start simulation; not connected to physics server')
            return
        if self.parallel_plugin_execution:
            self._executor_thread = Thread(
                target=self._run_parallel_execution,
                args=(self.loop_rate,),
                daemon=True,
            )
            self._executor_thread.start()
        else:
            self._timer = self.create_timer(1.0 / float(self.loop_rate), self._sequential_step)

    def stop(self) -> None:
        self._stop_event.set()
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if self._executor_thread is not None:
            self._executor_thread.join()
            self._executor_thread = None
        if self.connected_to_physics_server:
            self.pb.disconnect()
            self.connected_to_physics_server = False
        for temp_file in self._temp_urdf_files:
            try:
                os.remove(temp_file)
            except OSError:
                pass
        self._temp_urdf_files.clear()

    def _start_gui(self, gui: bool = True):
        if gui:
            self.get_logger().info('Running pybullet with GUI enabled')
            options = self.gui_options if isinstance(self.gui_options, str) else ''
            return self.pb.connect(self.pb.GUI, options=options)
        self.get_logger().info('Running pybullet without GUI')
        return self.pb.connect(self.pb.DIRECT)

    def _init_pybullet_robot(self):
        urdf_path = self.robot_urdf_path
        if not urdf_path:
            raise RuntimeError('Mandatory parameter robot_urdf_path not set')
        urdf_path = os.path.expanduser(os.path.expandvars(urdf_path))
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f'URDF file does not exist: {urdf_path}')
        if urdf_path.endswith('.xacro'):
            try:
                import xacro  # type: ignore
            except ImportError as exc:
                raise RuntimeError('xacro is required to process robot xacro files') from exc
            doc = xacro.process_file(urdf_path)
            urdf_content = doc.toxml()
            from tempfile import NamedTemporaryFile
            with NamedTemporaryFile('w', delete=False, suffix='.urdf') as temp_file:
                temp_file.write(urdf_content)
                urdf_path = temp_file.name
                self._temp_urdf_files.append(temp_file.name)
        robot_spawn_orientation = self.pb.getQuaternionFromEuler([0.0, 0.0, float(self.robot_pose_yaw)])
        fixed_base = bool(self.fixed_base)
        if self.config_data.get('use_intertia_from_file', False):
            urdf_flags = self.pb.URDF_USE_INERTIA_FROM_FILE | self.pb.URDF_USE_SELF_COLLISION
        else:
            urdf_flags = self.pb.URDF_USE_SELF_COLLISION
        self.environment.load_environment()
        self.pb.setRealTimeSimulation(0)
        self.get_logger().info(f'Loading URDF model: {urdf_path}')
        return self.pb.loadURDF(
            urdf_path,
            basePosition=[float(self.robot_pose_x), float(self.robot_pose_y), float(self.robot_pose_z)],
            baseOrientation=robot_spawn_orientation,
            useFixedBase=fixed_base,
            flags=urdf_flags,
        )

    def _get_properties(self):
        rev_joint_index_name_dic: Dict[int, str] = {}
        fixed_joint_index_name_dic: Dict[int, str] = {}
        prismatic_joint_index_name_dic: Dict[int, str] = {}
        link_names_to_ids_dic: Dict[str, int] = {}
        for joint_index in range(0, self.pb.getNumJoints(self.robot)):
            info = self.pb.getJointInfo(self.robot, joint_index)
            link_names_to_ids_dic[info[12].decode('utf-8')] = joint_index
            if info[2] == self.pb.JOINT_REVOLUTE:
                rev_joint_index_name_dic[joint_index] = info[1].decode('utf-8')
            elif info[2] == self.pb.JOINT_FIXED:
                fixed_joint_index_name_dic[joint_index] = info[1].decode('utf-8')
            elif info[2] == self.pb.JOINT_PRISMATIC:
                prismatic_joint_index_name_dic[joint_index] = info[1].decode('utf-8')
        return rev_joint_index_name_dic, prismatic_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic

    def handle_reset_simulation(self, request, response):
        self.get_logger().info('Resetting simulation')
        self.pause_simulation = True
        self.pb.resetSimulation()
        self._init_pybullet_robot()
        self.pause_simulation = False
        return response

    def handle_pause_physics(self, request, response):
        self.get_logger().info('Pausing simulation')
        self.pause_simulation = True
        return response

    def handle_unpause_physics(self, request, response):
        self.get_logger().info('Unpausing simulation')
        self.pause_simulation = False
        return response

    def _sequential_step(self) -> None:
        if self.pause_simulation:
            return
        for plugin in self.plugins:
            execute = getattr(plugin, 'execute', None)
            if callable(execute):
                execute()
        self.pb.stepSimulation()

    def _run_parallel_execution(self, loop_rate: float) -> None:
        exec_manager_obj = FuncExecManager(
            self.plugins,
            self._stop_event.is_set,
            self.pb.stepSimulation,
            lambda: self.pause_simulation,
            log_info=self.get_logger().info,
            log_warn=self.get_logger().warning,
            log_debug=self.get_logger().debug,
            function_name='plugin',
        )
        exec_manager_obj.start_synchronous_execution(loop_rate=float(loop_rate))

    def destroy_node(self) -> bool:
        self.stop()
        return super().destroy_node()


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PyBulletRosNode()
    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
