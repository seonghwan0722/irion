from setuptools import setup

package_name = 'system2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='korea',
    maintainer_email='korea@todo.todo',
    description='System-2 High-Level Planner',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = system2.system2_node:main'
        ],
    },
)
