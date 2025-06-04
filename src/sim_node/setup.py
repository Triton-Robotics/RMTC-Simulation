from setuptools import find_packages, setup

package_name = 'sim_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'models/infantry/*',
            'models/infantry/meshes/*',
            'models/hero/*',
            'models/hero/meshes/*',
            'models/sentry/*',
            'models/sentry/meshes/*',
            'models/arena/*',
            'models/arena/meshes/*',
            'models/bullet/*',
            'models/bullet/assets/*'
        ]
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kane-li',
    maintainer_email='kal036@ucsd.edu',
    description='Pybullet simulation of our robots. Should listen to server and movement requests in order to simulate how the robot would move in real-time',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = sim_node.sim_node:main'
        ],
    },
)
