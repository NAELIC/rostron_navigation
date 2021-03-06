from setuptools import setup

package_name = 'rostron_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name +
              '/primitive', package_name+'/path_planning'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Etienne Schmitz',
    maintainer_email='contact@etienne-schmitz.com',
    description='Navigation System for SSL Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation = rostron_navigation.main:main',
            ],
    },
)
