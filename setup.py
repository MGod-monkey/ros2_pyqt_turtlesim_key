from setuptools import setup

package_name = 'ros2_pyqt_turtlesim_key'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lanhanba',
    maintainer_email='lanhanba@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_pyqt_turtlesim_key = ros2_pyqt_turtlesim_key.ros2_pyqt_turtlesim_key:main'
        ],
    },
)
