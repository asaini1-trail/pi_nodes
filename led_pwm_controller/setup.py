from setuptools import setup

package_name = 'led_pwm_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pigpio'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@example.com',
    description='LED PWM controller node for Raspberry Pi using pigpio',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'led_pwm_node = led_pwm_controller.led_pwm_controller:main'
        ],
    },
)