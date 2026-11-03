import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
import yaml


def test_combined_launch(monkeypatch):
    package_dir = Path(__file__).resolve().parents[1]
    launch_file = package_dir / 'launch' / 'capabilities2_server.launch.py'
    spec = importlib.util.spec_from_file_location('capabilities2_server_launch', launch_file)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    monkeypatch.setattr(launch_module, 'get_package_share_directory', lambda _: str(package_dir))

    argument, server, proxy = launch_module.generate_launch_description().entities
    assert argument.name == 'enable_launch_proxy'
    assert argument.default_value[0].text == 'true'

    with (package_dir / 'config' / 'capabilities.yaml').open(encoding='utf-8') as config:
        expected = yaml.safe_load(config)['/**']['ros__parameters']['launch_proxy']
    assert evaluate_parameters(LaunchContext(), proxy._Node__parameters)[0] == expected

    context = LaunchContext()
    context.launch_configurations['enable_launch_proxy'] = 'true'
    assert proxy.condition.evaluate(context)
    context.launch_configurations['enable_launch_proxy'] = 'false'
    assert not proxy.condition.evaluate(context)