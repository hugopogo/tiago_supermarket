from setuptools import find_packages, setup

package_name = 'tiago_supermarket'

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
    maintainer='rosdev',
    maintainer_email='rosdev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco = tiago_supermarket.aruco:main',
            'object_detection = tiago_supermarket.object_detection:main',
            'navigation = tiago_supermarket.navigation:main'
        ],
    },
)
