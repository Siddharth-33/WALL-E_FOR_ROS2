from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'walle_control'

# ── What is setup.py? ─────────────────────────────────────────────────────────
# For Python ROS2 packages (ament_python build type), setup.py replaces
# CMakeLists.txt. It tells the Python build system (pip/setuptools) how to
# install this package.
#
# The CRITICAL part is `entry_points` → `console_scripts`.
# This is what makes `ros2 run walle_control monitor_node` work!
#
# Without entry_points, ROS2 can't find your Python scripts.
# With entry_points, colcon installs tiny wrapper scripts in
# ~/sid_ws/install/walle_control/lib/walle_control/ that point to
# your Python functions.
# ─────────────────────────────────────────────────────────────────────────────

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register the package with ament (ROS2 package index)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml so other packages can find this one
        ('share/' + package_name, ['package.xml']),
        # Install config files so they're accessible at runtime
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Host-side ROS2 monitor and tuner nodes for Wall-E robot',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        # console_scripts registers executable commands.
        # Format: 'command_name = package.module:function'
        #
        # 'monitor_node' becomes the command you type:
        #   ros2 run walle_control monitor_node
        # ROS2 calls: walle_control.monitor_node.main()
        #
        # 'tuner_node' becomes:
        #   ros2 run walle_control tuner_node
        # ROS2 calls: walle_control.tuner_node.main()
        'console_scripts': [
            'monitor_node        = walle_control.monitor_node:main',
            'tuner_node          = walle_control.tuner_node:main',
            'imu_tf_broadcaster  = walle_control.imu_tf_broadcaster:main',
            'lsa_visualizer_node = walle_control.lsa_visualizer_node:main',
        ],
    },
)
