from setuptools import find_packages, setup

package_name = 'kitchen_table'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5'],
    zip_safe=True,
    maintainer='seongjae',
    maintainer_email='tjdwocl85@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kitchen_display = kitchen_table.kitchen_display:main',
            'table_monitor = kitchen_table.table_monitor:main',
        ],
    },
)
