from setuptools import setup

package_name = 'mock_camera_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jacob Perron',
    maintainer_email='jacobmperron@gmail.com',
    description='Scripts to help mock cameras',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'mock_camera_info = mock_camera_tools.mock_camera_info:main',
        ],
    },
)
