from setuptools import setup

package_name = 'multiplexer-controller'

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
    maintainer='mariolas',
    maintainer_email='luismarioribeiro01@gmail.com',
    description='Multiplexer package to control the instructions to the turtle node.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'multiplexer = multiplexer-controller.multiplexer:main'
        ],
    },
)
