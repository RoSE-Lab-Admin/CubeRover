"""
Runs N autonomous exploration trials end-to-end, replacing the manual
4-terminal workflow (repo README, "Each exploration run" section):
  Window 1 (nav2.launch.py)        -> included here, launched once
  Window 2 (gp_explorer_gpu.py)    -> called per-trial by autonomous_trials.py
  Window 3 (waypoint.launch.py)    -> launched per-trial by autonomous_trials.py
  Window 4 (ros2 bag record)       -> started/stopped per-trial by autonomous_trials.py

Usage:
  ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir
  ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir n_trajectories:=10

bag_dir must exist. If it has no `initial_bag` yet, one is collected
automatically by driving to the origin (0,0) first -- see autonomous_trials.py.

Optional online dynamics-model retraining (default off) -- retrains whatever
model is currently deployed (per nav2_param2.yaml's dynamics_mode) on the bags
collected in bag_dir after every trial, and pushes the result into the live
controller via a controller_server lifecycle cycle. See dynamics_retrain.py.
  ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir \\
      retrain_dynamics:=true warm_start:=true retrain_subset:=false

Optional train-from-scratch mode (default off, takes over from retrain_dynamics
if both are set): runs the first from_scratch_n_bootstrap trials under pure
kinematics regardless of what's deployed, then trains a fresh MLP from a blank
init on just that data and keeps updating it every trial after. Weights are
stored per-iteration under bag_dir/from_scratch_weights/ -- the shared deployed
model is never read from or written to. See dynamics_retrain.py / autonomous_trials.py.
  ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir \\
      train_from_scratch:=true from_scratch_n_bootstrap:=5
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    if not bag_dir:
        raise ValueError(
            "bag_dir is required, e.g.: "
            "ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir")

    n_trajectories = LaunchConfiguration('n_trajectories')
    retrain_dynamics = LaunchConfiguration('retrain_dynamics')
    warm_start = LaunchConfiguration('warm_start')
    retrain_subset = LaunchConfiguration('retrain_subset')
    retrain_subset_fraction = LaunchConfiguration('retrain_subset_fraction')
    train_from_scratch = LaunchConfiguration('train_from_scratch')
    from_scratch_n_bootstrap = LaunchConfiguration('from_scratch_n_bootstrap')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_stack'),
                'launch',
                'nav2.launch.py'
            ])
        ),
    )

    orchestrator_node = Node(
        package='nav2_stack',
        executable='autonomous_trials',
        output='screen',
        arguments=[
            '--bag-dir', bag_dir,
            '--n-trajectories', n_trajectories,
            '--retrain-dynamics', retrain_dynamics,
            '--warm-start', warm_start,
            '--retrain-subset', retrain_subset,
            '--retrain-subset-fraction', retrain_subset_fraction,
            '--train-from-scratch', train_from_scratch,
            '--from-scratch-n-bootstrap', from_scratch_n_bootstrap,
        ],
    )

    return [nav2_launch, orchestrator_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n_trajectories', default_value='25',
                              description='Number of exploration trials to run'),
        DeclareLaunchArgument('bag_dir', default_value='',
                              description='Directory bags are stored in and loaded from '
                                          '(required)'),
        DeclareLaunchArgument('retrain_dynamics', default_value='false',
                              description='Retrain the deployed dynamics model after every '
                                          'trial and redeploy it live'),
        DeclareLaunchArgument('warm_start', default_value='true',
                              description='Warm-start retraining from currently deployed '
                                          'weights (MLP only; no-op for linear)'),
        DeclareLaunchArgument('retrain_subset', default_value='false',
                              description='Train on a random subset of prior bags + the new '
                                          'trajectory instead of every bag so far'),
        DeclareLaunchArgument('retrain_subset_fraction', default_value='0.3',
                              description='Fraction of prior bags to sample when '
                                          'retrain_subset is true'),
        DeclareLaunchArgument('train_from_scratch', default_value='false',
                              description='Bootstrap under kinematics then train a fresh MLP '
                                          'from scratch, isolated from the shared deployed '
                                          'model -- takes over from retrain_dynamics if both '
                                          'are set'),
        DeclareLaunchArgument('from_scratch_n_bootstrap', default_value='5',
                              description='Number of initial kinematics-only trials before '
                                          'the first from-scratch fit'),
        OpaqueFunction(function=launch_setup),
    ])
