from setuptools import setup

package_name = 'turtle_random'

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
    maintainer='luis',
    maintainer_email='luismarioribeiro01@gmail.com',
    description='RANDOM',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'random = turtle_random.random:main'
        ],
    },
)
