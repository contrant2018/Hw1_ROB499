from setuptools import find_packages, setup

package_name = 'hw2'

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
    description='hw2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        # Publisher entry points
        'oscope_osc = hw2.oscope:main_osc',
        'oscope_slow = hw2.oscope:main_slow',
        'oscope_fast = hw2.oscope:main_fast',
        # Limiter entry points
        'limiter_osc = hw2.limiter:main_osc',
        'limiter_slow = hw2.limiter:main_slow',
        'limiter_fast = hw2.limiter:main_fast',
        ],
    },
)
