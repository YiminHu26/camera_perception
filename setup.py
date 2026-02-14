from setuptools import find_packages, setup

package_name = 'camera_perception'

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
    maintainer='Yimin Hu',
    maintainer_email='yimin.hu@student.kit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "frame_saver = camera_perception.frame_saver:main",
            "depth_viewer = camera_perception.test_depth_view:main",
        ],
    },
)
