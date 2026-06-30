# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import unittest

import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util


def generate_test_description():
    gtest_node = launch_ros.actions.Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration('test_binary_dir'),
                launch.substitutions.LaunchConfiguration('binary'),
            ]
        ),
        name=launch.substitutions.LaunchConfiguration('name'),
        output='screen',
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name='test_binary_dir',
                description='Binary directory of package containing test executables',
            ),
            launch.actions.DeclareLaunchArgument(
                name='binary',
                description='The binary to run',
            ),
            launch.actions.DeclareLaunchArgument(
                name='name',
                description='The name of the node',
            ),
            gtest_node,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        'gtest_node': gtest_node,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, gtest_node):
        proc_info.assertWaitForShutdown(gtest_node, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, gtest_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest_node)
