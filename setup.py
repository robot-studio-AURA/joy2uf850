from setuptools import find_packages, setup

package_name = 'joy2uf850'

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
    maintainer='David Ho',
    maintainer_email='homandat2002@gmail.com',
    description='Remap controls for UF850 robotic arm intuitively.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aura_joy = joy2uf850.aura_joy:main'
        ],
    },
)
