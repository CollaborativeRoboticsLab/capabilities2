import pathlib
import sys
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from capabilities2_launch_proxy.launch_process_matcher import LaunchProcessMatcher


def test_matcher_signature_is_stable_across_include_runtime_clones():
    share_dir = get_package_share_directory('capabilities2_launch_proxy')
    child_launch = f'{share_dir}/launch/package_resolved_test.launch.py'
    include = IncludeLaunchDescription(PythonLaunchDescriptionSource(child_launch))

    source_launch_description = include.get_sub_entities()[0]
    source_action = next(
        entity for entity in source_launch_description.entities
        if isinstance(entity, ExecuteProcess)
    )

    executed_entities = include.execute(LaunchContext())
    runtime_launch_description = executed_entities[-1]
    runtime_action = next(
        entity for entity in runtime_launch_description.entities
        if isinstance(entity, ExecuteProcess)
    )

    source_signature = LaunchProcessMatcher.make_process_action_signature(source_action)
    runtime_signature = LaunchProcessMatcher.make_process_action_signature(runtime_action)

    assert source_signature == runtime_signature


def test_matcher_matches_runtime_clone_from_source_execute_action():
    share_dir = get_package_share_directory('capabilities2_launch_proxy')
    child_launch = f'{share_dir}/launch/package_resolved_test.launch.py'
    include = IncludeLaunchDescription(PythonLaunchDescriptionSource(child_launch))

    source_launch_description = include.get_sub_entities()[0]
    source_action = next(
        entity for entity in source_launch_description.entities
        if isinstance(entity, ExecuteProcess)
    )

    executed_entities = include.execute(LaunchContext())
    runtime_launch_description = executed_entities[-1]
    runtime_action = next(
        entity for entity in runtime_launch_description.entities
        if isinstance(entity, ExecuteProcess)
    )

    matcher = LaunchProcessMatcher.from_execute_actions([source_action])
    event = SimpleNamespace(action=runtime_action, process_name='python3-1')

    assert matcher.matches(event) is True


def test_matcher_falls_back_to_process_name_prefix_for_nodes():
    node_action = Node(package='demo_nodes_cpp', executable='talker', name='demo_talker')
    matcher = LaunchProcessMatcher.from_execute_actions([node_action])
    unrelated_action = ExecuteProcess(cmd=['python3', '-c', 'print("other")'])
    event = SimpleNamespace(action=unrelated_action, process_name='talker-7')

    assert matcher.matches(event) is True


def test_matcher_extracts_process_name_prefix_from_execute_process_command():
    execute_action = ExecuteProcess(cmd=['python3', '/tmp/demo_script.py'])

    assert LaunchProcessMatcher.get_process_name_prefix(execute_action) == 'python3'


def test_matcher_freezes_text_substitutions_to_stable_values():
    execute_action = ExecuteProcess(cmd=['python3', '/tmp/demo_script.py'])
    signature = LaunchProcessMatcher.make_process_action_signature(execute_action)

    assert signature[1] == (
        (('TextSubstitution', ('text', 'python3')),),
        (('TextSubstitution', ('text', '/tmp/demo_script.py')),),
    )