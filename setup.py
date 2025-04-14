from setuptools import find_packages, setup

package_name = 'hw1'

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
    maintainer='AnthonyContreras',
    maintainer_email='contrant@oregonstate.edu',
    description='hw1 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'oscope = hw1.oscope:main',
        'limiter = hw1.limiter:main',
        ],
    },
)
