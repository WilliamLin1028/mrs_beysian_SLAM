from setuptools import find_packages, setup

package_name = 'mric_slamtool'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch_mric_bn_slam_inside.py']),
        ('share/' + package_name, ['launch/launch_mric_bn_slam_outside.py']),
        ('share/' + package_name + '/config' , ['config/robot_config.yaml']),
        ('share/' + package_name + '/config' , ['config/basic_cpdTable.yaml']),
        ('share/' + package_name + '/config' , ['config/my_cpdTable.yaml']),
        ('share/' + package_name + '/config' , ['config/host_priority.yaml']),
        ('share/' + package_name + '/database/', ['database/__init__.py']),
        ('share/' + package_name + '/cmd_folder/', ['cmd_folder/__init__.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='william',
    maintainer_email='william@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'inside_allocation_agent = mric_slamtool.inside_allocation_agent:main',
        'inside_executor = mric_slamtool.inside_executor:main',
        'inside_pub_merge_map = mric_slamtool.inside_pub_merge_map:main',
        'inside_save_robot_map = mric_slamtool.inside_save_robot_map:main',
        'inside_mark_task = mric_slamtool.inside_mark_task:main',
        'inside_successrate_agent = mric_slamtool.inside_successrate_agent:main',

        'bilateral_topic_i2o = mric_slamtool.bilateral_topic_i2o:main',
        'bilateral_topic_o2i = mric_slamtool.bilateral_topic_o2i:main',

        'outside_auctioneer_agent = mric_slamtool.outside_auctioneer_agent:main',
        'outside_check_auctioneer_agent = mric_slamtool.outside_check_auctioneer_agent:main',
        'outside_allocation_agent = mric_slamtool.outside_allocation_agent:main',
        'outside_pub_local_map = mric_slamtool.outside_pub_local_map:main',
        'outside_pub_workstatus = mric_slamtool.outside_pub_workstatus:main',
        'outside_task_agent = mric_slamtool.outside_task_agent:main',
        'outside_sr_rq_reciver = mric_slamtool.outside_sr_rq_reciver:main',
        'outside_auc_lock_reciver = mric_slamtool.outside_auc_lock_reciver:main',
        'outside_newtask_reciver = mric_slamtool.outside_newtask_reciver:main',
        'outside_save_merge_map = mric_slamtool.outside_save_merge_map:main',
        'outside_merge_map_agent = mric_slamtool.outside_merge_map_agent:main',
        'outside_trigger_experiment = mric_slamtool.outside_trigger_experiment:main',

        'tool_initial_database = mric_slamtool.tool_initial_database:main',
        'tool_test_task_agent = mric_slamtool.tool_test_task_agent:main',
        'tool_simple_float_pub = mric_slamtool.tool_simple_float_pub:main',
        'tool_simple_string_pub = mric_slamtool.tool_simple_string_pub:main',
        'experiment_commander = mric_slamtool.experiment_commander:main',
        ],
    },
)
