from glob import glob

from setuptools import Extension, find_packages, setup

package_name = 'jetcobot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    ext_modules=[Extension('visual_servo_6dof._fcl_guard',
                           sources=['visual_servo_6dof/fcl_guard.cpp'],
                           include_dirs=['/usr/include/eigen3'], libraries=['fcl'],
                           language='c++', extra_compile_args=['-std=c++14', '-O2'])],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/visual_servo.launch.py',
             'launch/wrist_camera.launch.py',
             'launch/moveit_servo.launch.py',
             'launch/visual_servo_demo.launch.py',
             'launch/hardware_visual_servo.launch.py',
             'launch/tracking_planner.launch.py']),
        ('share/' + package_name + '/visual_servo_6dof/config',
            ['visual_servo_6dof/config/visual_servo.yaml',
             'visual_servo_6dof/config/moveit_servo.yaml',
             'visual_servo_6dof/config/sim_initial_positions.yaml']),
        ('share/' + package_name + '/visual_servo_6dof',
            ['visual_servo_6dof/README.md',
             'visual_servo_6dof/PID_TRACKING_PROMPT.md',
             'visual_servo_6dof/PID_TRACKING_FEASIBILITY.md']),
        ('share/' + package_name + '/visual_servo_6dof/validation',
            glob('visual_servo_6dof/validation/*.*')),
        *[('share/' + package_name + '/visual_servo_6dof/validation/' + scenario,
           glob('visual_servo_6dof/validation/' + scenario + '/*'))
          for scenario in ['sine', 'step', 'boundary', 'singular', 'hardware_tracking']],
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
            'tracking_main = jetcobot_driver.tracking_main:main',
            'tracking_display = visual_servo_6dof.tracking_display:main',
            'tracking_mock = visual_servo_6dof.tracking_mock:main',
            'tracking_scene = visual_servo_6dof.tracking_scene:main',
            'simulate_tracking = visual_servo_6dof.simulate_tracking:main',
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
