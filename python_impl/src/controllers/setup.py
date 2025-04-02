from setuptools import find_packages, setup

package_name = 'controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        # Include all `.npy` files in the "data" subdirectory
        "controllers": ["data/*.npy"],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton',
    maintainer_email='a.bredenbeck@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_traj = controllers.example_traj:main',
            'draw_traj = controllers.custom_traj:draw_traj',
            'jacobian_traj = controllers.custom_traj:jacobian_traj',
            'grab_ab_traj = controllers.custom_traj:grab_ab_traj',
            'grab_ba_traj = controllers.custom_traj:grab_ba_traj',
            'berry_traj = controllers.custom_traj:berry_traj',
            'wipe_traj = controllers.custom_traj:wipe_traj',
            'demo_traj = controllers.custom_traj:demo_traj',
            'empty_traj = controllers.custom_traj:empty_traj',
        ],
    },
)
