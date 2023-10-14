from setuptools import find_packages, setup

package_name = 'ball_tracker_package'

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
    maintainer='harish',
    maintainer_email='harish.rajkumar2022@vitstudent.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_node = ball_tracker_package.camera_node:main",
            "tracker_node = ball_tracker_package.tracker_node:main"
        ],
    },
)
