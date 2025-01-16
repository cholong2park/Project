from setuptools import find_packages, setup

package_name = 'control_pc'

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
    maintainer='seongjae',
    maintainer_email='tjdwocl85@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'control_node = control_pc.control_gui_node:main',
        'test_node = control_pc.test_node:main',
        'conveyor = control_pc.conveyor:main',
        ],
    },
)
