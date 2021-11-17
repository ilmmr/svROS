from setuptools import setup

package_name = 'turtle_multiplexer'

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
    description='MULTIPLEXER',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multiplexer = turtle_multiplexer.multiplexer:main'
        ],
    },
)
