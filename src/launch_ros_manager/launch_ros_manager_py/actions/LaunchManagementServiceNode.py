import functools
from typing import List
from typing import Optional
from typing import Callable
from typing import Text

import launch
from launch import SomeSubstitutionsType
from launch.action import Action
import launch.logging

from launch_ros_manager.srv import ShutdownNamespace

from launch_ros.actions.node import Node
from launch.events.process import ShutdownProcess

from launch_ros.ros_adapters import get_ros_node

class LaunchManagementServiceNode(Node):
    """ This action creates a node for management of the ROS launch system. """

    def __init__(
        self,
        *,
        name: SomeSubstitutionsType = "launch_manager",
        package: SomeSubstitutionsType = "launch_ros_manager",
        executable: SomeSubstitutionsType = "manager_node",
        namespace: SomeSubstitutionsType = "",
        **kwargs
    ) -> None:
        """
        Construct a LaunchManagementServiceNode action.

        This action emits some event(s) in certain circumstances:

        - :class:`launch.events.process.ShutdownProcess`:

            - this event is emitted when the "/<name>/shutdown_namespace"
              ROS service is called.

        :param name: The name of the management node.
        """
        super().__init__(name=name, namespace=namespace, package=package, executable=executable, **kwargs)
        self.__logger = launch.logging.get_logger(__name__)
        self.__service_shutdown_ns = None

    def _onRequestShutdownNamespace(self, context, request, response):
        ''' Called when a client requests namespace shutdown. '''
        response.success = False

        self.__logger.info(f"Shutting down all nodes in namespace \"{request.target_namespace}\"")

        try:
            event = ShutdownProcess(
                process_matcher=matches_ros_namespace(request.target_namespace)
            )
            context.asyncio_loop.call_soon_threadsafe(lambda: context.emit_event_sync(event))
            response.success = True
        
        except Exception as exc:
            self.__logger.error(
                "Exception in handling of 'launch_ros_manager.srv.ShutdownNamespace': {}".format(exc))

        return response 

    def execute(self, context: launch.LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        self._perform_substitutions(context)  # ensure self.node_name is expanded
        if Node.UNSPECIFIED_NODE_NAME in self.node_name:
            raise RuntimeError('node_name unexpectedly incomplete for management node')
        
        node = get_ros_node(context)

        # Create a ROS service to wait for commands.
        self.__service_shutdown_ns = node.create_service(
            ShutdownNamespace,
            '{}/shutdown_namespace'.format(self.node_name),
            functools.partial(self._onRequestShutdownNamespace, context),
        )

        # Delegate execution to Node and ExecuteProcess.
        return super().execute(context)

def matches_ros_namespace(namespace: Text) -> Callable[['Node'], bool]:
    """ Return a matcher which matches based on the name of the ROS namespace."""

    # Expand with a leading slash.
    if not namespace.startswith('/'):
        namespace = f'/{namespace}'

    # This test function is used by the launch context to determine which nodes to match.
    def test(action):
        if not isinstance(action, Node):
            return False
        return action.expanded_node_namespace == namespace
    
    return test
