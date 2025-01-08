from setuptools import find_packages, setup

package_name = 'record'

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
    maintainer='karin-22',
    maintainer_email='taguchan.karin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_video = record.record_video:main',
            'camera_preview_elp = record.camera_preview_elp:main',
            'fer_preview_elp = record.fer_preview_elp:main',
            'rgb_preview_multi_camera = record.rgb_preview_multi_camera:main',
            'fer2 = record.fer2:main',
            'camera_preview_realD435i = record.camera_preview_realD435i:main',
            'camera_preview_realD515 = record.camera_preview_realD515:main',
        ],
    },
)
