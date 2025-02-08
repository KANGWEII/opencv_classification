from setuptools import find_packages, setup

package_name = 'opencv_classification'

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
    maintainer='Kang Wei',
    maintainer_email='pkangwei@outlook.com',
    description='Colour and shape classification sample demo using opencv with ROS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "visual_processing = opencv_classification.visual_processing:main"
        ],
    },
)
