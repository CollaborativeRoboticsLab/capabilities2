import os
from typing import Any, List

from launch.actions import ExecuteProcess


class LaunchProcessMatcher:
    def __init__(self, process_action_signatures: set, process_name_prefixes: set):
        self.process_action_signatures = process_action_signatures
        self.process_name_prefixes = process_name_prefixes

    @classmethod
    def from_execute_actions(cls, execute_actions: List[ExecuteProcess]) -> 'LaunchProcessMatcher':
        return cls(
            process_action_signatures={
                cls.make_process_action_signature(execute_action)
                for execute_action in execute_actions
            },
            process_name_prefixes={
                prefix
                for prefix in (cls.get_process_name_prefix(execute_action) for execute_action in execute_actions)
                if prefix
            },
        )

    def matches(self, event) -> bool:
        if self.make_process_action_signature(event.action) in self.process_action_signatures:
            return True

        process_name = getattr(event, 'process_name', '')
        return any(
            process_name == prefix or process_name.startswith(f'{prefix}-')
            for prefix in self.process_name_prefixes
        )

    @classmethod
    def make_process_action_signature(cls, execute_action: ExecuteProcess) -> Any:
        node_executable = getattr(execute_action, 'node_executable', None) \
            if hasattr(execute_action, 'node_executable') else None
        command_signature = None
        if not isinstance(node_executable, str) or not node_executable:
            command_signature = cls.freeze_launch_value(getattr(execute_action, 'cmd', None))

        return (
            type(execute_action).__name__,
            command_signature,
            cls.freeze_launch_value(getattr(execute_action, 'name', None)),
            cls.freeze_launch_value(node_executable),
        )

    @classmethod
    def get_process_name_prefix(cls, execute_action: ExecuteProcess) -> str:
        node_executable = getattr(execute_action, 'node_executable', None) \
            if hasattr(execute_action, 'node_executable') else None
        if isinstance(node_executable, str) and node_executable:
            return node_executable

        cmd = getattr(execute_action, 'cmd', None)
        if not cmd:
            return ''

        first_part = cmd[0] if isinstance(cmd, (list, tuple)) and cmd else None
        command_text = cls.extract_launch_text(first_part)
        if not command_text:
            return ''

        return os.path.basename(command_text.strip())

    @classmethod
    def extract_launch_text(cls, value) -> str:
        if isinstance(value, str):
            return value

        if isinstance(value, (list, tuple)):
            parts = [cls.extract_launch_text(item) for item in value]
            return ''.join(part for part in parts if part)

        text_value = getattr(value, 'text', None)
        if isinstance(text_value, str):
            return text_value

        return ''

    @classmethod
    def freeze_launch_value(cls, value) -> Any:
        if isinstance(value, (str, int, float, bool, type(None))):
            return value

        if isinstance(value, (list, tuple)):
            return tuple(cls.freeze_launch_value(item) for item in value)

        if isinstance(value, dict):
            return tuple(sorted(
                (str(key), cls.freeze_launch_value(item))
                for key, item in value.items()
            ))

        text_value = getattr(value, 'text', None)
        if isinstance(text_value, str):
            return (type(value).__name__, ('text', text_value))

        value_dict = getattr(value, '__dict__', None)
        if isinstance(value_dict, dict):
            filtered_items = []
            for key, item in sorted(value_dict.items()):
                if key.startswith('_ExecuteLocal__') or key.startswith('_subprocess_'):
                    continue
                if key.startswith('_Action__condition'):
                    continue
                filtered_items.append((key, cls.freeze_launch_value(item)))
            return (type(value).__name__, tuple(filtered_items))

        return (type(value).__name__, str(value))