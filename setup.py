from setuptools import setup

package_name = 'shipsim_mmg_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taiga MITSUYUKI',
    maintainer_email='mitsuyuki-taiga-my@ynu.ac.jp',
    description='MMG model module for shipsim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
