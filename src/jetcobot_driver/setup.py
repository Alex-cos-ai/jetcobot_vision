from setuptools import find_packages, setup

package_name = 'jetcobot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/visual_servo.launch.py',
             'launch/wrist_camera.launch.py',
             'launch/moveit_servo.launch.py',
             'launch/visual_servo_demo.launch.py',
             'launch/hardware_visual_servo.launch.py']),
        ('share/' + package_name + '/visual_servo_6dof/config',
            ['visual_servo_6dof/config/visual_servo.yaml',
             'visual_servo_6dof/config/moveit_servo.yaml',
             'visual_servo_6dof/config/sim_initial_positions.yaml']),
        ('share/' + package_name + '/visual_servo_6dof',
            ['visual_servo_6dof/README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sync_plan = jetcobot_driver.sync_plan:main',
            'sync_plan_nx = jetcobot_driver.sync_plan_nx:main',
            'visual_servo_6dof = '
            'visual_servo_6dof.visual_servo_6dof:main',
            'servo_serial_bridge = '
            'visual_servo_6dof.servo_serial_bridge:main',
            'wrist_camera = visual_servo_6dof.wrist_camera:main',
            'check_camera_sync = visual_servo_6dof.check_camera_sync:main',
        ],
    },
)
