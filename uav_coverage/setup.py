from setuptools import find_packages, setup

package_name = 'uav_coverage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='girik',
    maintainer_email='girikai777@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'task1_server = uav_coverage.task1_server:main',
        'task1_client = uav_coverage.task1_client:main',
        'task2_manager = uav_coverage.task2_manager:main',
        'task2_optimizer = uav_coverage.task2_optimizer:main',
        'task2_visualizer = uav_coverage.task2_visualizer:main'
        ],
    },
)
