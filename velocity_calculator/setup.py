from setuptools import setup

package_name = 'velocity_calculator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Calculate robot velocities from encoder data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_calculator_node = velocity_calculator.velocity_calculator_node:main',
            'feedback_odom_encoder_node = velocity_calculator.feedback_odom_encoder_node:main',
            'feedback_odom = velocity_calculator.feedback_odom:main',
            'odom_delay = velocity_calculator.odom_delay:main',
        ],
    },
)
