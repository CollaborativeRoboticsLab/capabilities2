'''
capabilities launch proxy server

implements a launch server responding to messages to launch and shutdown launch files
use in conjunction with capabilities2_server to launch capabilities

acts like ros2 launch command but from an action interface
'''

import os
import threading
from typing import Set
from typing import Text
from typing import Optional
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle
from launch import LaunchService
from launch import LaunchContext
from launch.event import Event
from launch.events.process import ShutdownProcess
from launch.event_handler import EventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import ExecuteProcess
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.some_entities_type import SomeEntitiesType
from capabilities2_msgs.action import Launch
from capabilities2_msgs.msg import CapabilityEvent


class CancelLaunchGoalEvent(Event):
    """
    Launch goal cancel event

    used when a launch goal is canceled
    the result should be that the launch description associated with a goal
    is shutdown in the running LaunchService
    other goals should not be affected
    """

    name = 'launch.event.CancelLaunchGoal'

    def __init__(self, *, goal_handle: ServerGoalHandle) -> None:
        """
        Create a LaunchGoalCancelEvent
        """
        self.goal_handle = goal_handle


class CancelEventHandler(EventHandler):
    """
    Cancel event handler

    handle cancel launch goal events
    cancels a launch goal if it matches the goal handle passed in
    """

    def __init__(self, goal_handle: ServerGoalHandle, pids: Set[int]):
        super().__init__(matcher=lambda event: isinstance(event, CancelLaunchGoalEvent))
        self.goal_handle = goal_handle
        self.pids = pids

    def handle(self, event: Event, context: LaunchContext) -> Optional[SomeEntitiesType]:
        """
        handle event
        """

        super().handle(event, context)

        # check if event is a cancel launch goal event
        if not isinstance(event, CancelLaunchGoalEvent):
            return None

        # if the goals match then cancel
        if event.goal_handle == self.goal_handle:
            return EmitEvent(event=ShutdownProcess(
                process_matcher=lambda action: action.process_details['pid'] in self.pids
            ))

        return None


class CapabilitiesLaunchProxy(Node):
    """
    Capabilities Launch proxy server
    """

    def __init__(self, node_name='capabilities_launch_proxy'):
        # super class init
        super().__init__(node_name)

        # active files set
        self.active_launch_files: Set[Text] = set()
        self.launch_set_lock = threading.Lock()

        # create launch action server
        self.launch_sub = ActionServer(
            self,
            Launch,
            '~/launch',
            handle_accepted_callback=self.handle_accepted_cb,
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb
        )

        # cap event pub
        self.event_pub = self.create_publisher(
            CapabilityEvent,
            '~/events',
            10
        )

        # create launch service
        self.launch_service = LaunchService()

        # log start
        self.get_logger().info('capabilities launch proxy started')

    # service interface
    def get_active_launch_files_cb(self):
        """
        get active launch files
        """
        return self.active_launch_files

    # action interface
    def handle_accepted_cb(self, goal_handle: ServerGoalHandle) -> None:
        """
        handle accepted callback
        """
        # start execution and detach
        threading.Thread(target=goal_handle.execute).start()

    # execute callback is main launch feature
    def execute_cb(self, goal_handle: ServerGoalHandle) -> Launch.Result:
        """
        launch execute callback
        """

        # check if file exists
        if not os.path.isfile(goal_handle.request.launch_file_path):
            # file does not exist
            self.get_logger().error("file does not exist: '{}'".format(
                goal_handle.request.launch_file_path))

            # abort goal
            goal_handle.abort()
            return Launch.Result()

        with self.launch_set_lock:
            # launch context already exists guard
            if goal_handle.request.launch_file_path in self.active_launch_files:
                self.get_logger().error("launch already exists for source '{}'".format(
                    goal_handle.request.launch_file_path))

                goal_handle.abort()
                return Launch.Result()

            # add context to active files
            self.active_launch_files.add(goal_handle.request.launch_file_path)

        # get launch description from file
        description = AnyLaunchDescriptionSource(
            goal_handle.request.launch_file_path).get_launch_description(self.launch_service.context)

        # set up process id get event handler
        description_process_ids: Set[int] = set()

        # go through all actions and collect all processes (mostly nodes) in the description
        for e in description.entities:
            # check the type of entity is an execute process action
            if isinstance(e, ExecuteProcess):
                # register event handler for process start event
                # on process start store the pid
                description.add_action(
                    RegisterEventHandler(OnProcessStart(
                        target_action=e,
                        on_start=lambda event, context: description_process_ids.add(
                            event.pid)
                    ))
                )

        # add shutdown event to launch description
        # add shutdown event handler to launch description
        # this uses the cancel goal event handler and cancel goal event
        description.add_action(RegisterEventHandler(CancelEventHandler(
            goal_handle,
            description_process_ids
        )))

        # add to launch service
        self.launch_service.include_launch_description(description)

        # bind request to this thread
        self.get_logger().info('launching: {}'.format(
            goal_handle.request.launch_file_path
        ))

        # sleep rate for holding this action open
        rate = self.create_rate(1.0)

        # loop while not canceled
        while rclpy.ok():
            # if cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("launch canceled: '{}'".format(
                    goal_handle.request.launch_file_path
                ))

                # shutdown context
                self.launch_service.emit_event(
                    CancelLaunchGoalEvent(goal_handle=goal_handle))

                # delete context from active files
                with self.launch_set_lock:
                    self.active_launch_files.remove(
                        goal_handle.request.launch_file_path)

                goal_handle.canceled()
                return Launch.Result()

            # send event feedback
            feedback = Launch.Feedback()
            feedback.event.type = 'launched'
            goal_handle.publish_feedback(feedback)

            # spin indefinitely (sleep loop this thread)
            rate.sleep()

        goal_handle.succeed()
        return Launch.Result()

    def cancel_cb(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """
        accept all cancellations
        """
        return CancelResponse.ACCEPT


# main function
def main(args=None):
    """
    main function
    """

    try:
        # init node
        with rclpy.init(args=args):

            # instantiate the capabilities launch server
            capabilities_launch_proxy = CapabilitiesLaunchProxy()

            # spin node in new thread
            executor = MultiThreadedExecutor()
            executor.add_node(capabilities_launch_proxy)
            executor_thread = threading.Thread(target=executor.spin)
            executor_thread.start()

            # run the launch service
            capabilities_launch_proxy.get_logger().info('running LaunchService')
            capabilities_launch_proxy.launch_service.run(
                shutdown_when_idle=False)

            # cancel the launch services
            capabilities_launch_proxy.launch_service.shutdown()

            executor.shutdown()
            executor_thread.join()
            capabilities_launch_proxy.destroy_node()

    except ExternalShutdownException:
        pass


# main
if __name__ == '__main__':
    # run the proxy
    main()
