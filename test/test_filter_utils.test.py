# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

from pathlib import Path
import unittest

import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util


def generate_test_description():
    filter_gtest = launch_ros.actions.Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration('test_binary_dir'),
                'test_filter_utils',
            ]
        ),
        parameters=[Path(__file__).parent / 'test_robot_body_filter.yaml'],
        name='test_chain_config',
        output='screen',
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name='test_binary_dir',
                description='Binary directory of package containing test executables',
            ),
            filter_gtest,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        'filter_gtest': filter_gtest,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, filter_gtest):
        proc_info.assertWaitForShutdown(filter_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, filter_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=filter_gtest)
