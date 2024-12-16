#!/usr/bin/env python3

# =============================================================================
# TF ROS2 Topic Re-Publisher
# Created by: Shaun Altmann
# =============================================================================
'''
TF ROS2 Topic Re-Publisher
-
Creates a ROS node that re-publishes /tf and /tf_static topic data in the
required namespace.
'''
# =============================================================================

# =============================================================================
# Imports
# =============================================================================

# used for logging
import logging

# used for ros
import rclpy # type: ignore
from rclpy.node import Node # type: ignore

# used for topic messages
from tf2_msgs.msg import TFMessage # type: ignore


# =============================================================================
# TF Re-Publisher Node Definition
# =============================================================================
class TF_RePublisher(Node):
    '''
    TF Re-Publisher Node
    -
    Contains the publishers and subscribers to re-publish tf data.

    Fields
    -
    - _pub_tf : `rclpy.Publisher`
    - _pub_tf_static : `rclpy.Publisher`
    - source : `str` << readonly >>
    - target : `str` << readonly >>

    Methods
    -
    - __init__() : `None`
    - _callback_tf(msg) : `None`
    - _callback_tf_static(msg) : `None`
    '''

    # ====================
    # Method - Constructor
    def __init__(self) -> None:
        '''
        TF Re-Publisher Node Constructor
        -
        Creates a new `TF_RePublisher` node.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # initialize node
        super().__init__('tf_republisher')

        # declare parameters
        self.declare_parameter('source', '')
        self.declare_parameter('target', 'r1')

        # create publishers
        self._pub_tf = self.create_publisher(
            TFMessage,
            f'{self.target}/tf'.strip('/'),
            10
        )
        ''' Publisher for the `"/tf"` topic. '''
        self._pub_tf_static = self.create_publisher(
            TFMessage,
            f'{self.target}/tf_static'.strip('/'),
            10
        )
        ''' Publisher for the `"/tf_static"` topic. '''

        # subscribe to topics
        self.create_subscription(
            TFMessage,
            f'{self.source}/tf'.strip('/'),
            self._callback_tf,
            10
        )
        self.create_subscription(
            TFMessage,
            f'{self.source}/tf_static'.strip('/'),
            self._callback_tf_static,
            10
        )

        # log node construction
        self.get_logger().info(
            f'Created TF_RePublisher(source = {self.source!r}, target = '
            + f'{self.target!r})'
        )

    # ===========================
    # Property - Source Namespace
    @property
    def source(self) -> str:
        ''' Namespace to get the `/tf` and `/tf_static` data from. '''
        return self.get_parameter('source').get_parameter_value().string_value

    # ===========================
    # Property - Target Namespace
    @property
    def target(self) -> str:
        ''' Namespace to set the `/tf` and `/tf_static` data to. '''
        return self.get_parameter('target').get_parameter_value().string_value


    # =======================================
    # Method - Subscriber Callback - TF Topic
    def _callback_tf(self, msg: TFMessage) -> None:
        '''
        Subscriber Callback - TF Topic
        -
        Callback method for the Slamware Lidar `"/tf"` topic.

        Parameters
        -
        - msg : `TFMessage`
            - Message data from the ros topic.

        Returns
        -
        None
        '''

        self.get_logger().debug('Relaying TF Data')
        self._pub_tf.publish(msg)

    # ==============================================
    # Method - Subscriber Callback - Static TF Topic
    def _callback_tf_static(self, msg: TFMessage) -> None:
        '''
        Subscriber Callback - Static TF Topic
        -
        Callback method for the Slamware Lidar `"/tf_static"` topic.

        Parameters
        -
        - msg : `TFMessage`
            - Message data from the ros topic.

        Returns
        -
        None
        '''

        self.get_logger().debug('Relaying Static TF Data')
        self._pub_tf_static.publish(msg)


# =============================================================================
# Main Loop
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = TF_RePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

# =============================================================================
# End of File
# =============================================================================
