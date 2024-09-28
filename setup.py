from setuptools import find_packages, setup

package_name = 'tb_control'

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
    maintainer='rahul1',
    maintainer_email='rahuliitdh789@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb_openLoop_Scenario1 = tb_control.tb_openLoop_Scenario1:main',
            'tb_openLoop_Scenario2 = tb_control.tb_openLoop_Scenario2:main'
        ],
    },
)
