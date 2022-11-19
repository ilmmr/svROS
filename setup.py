### Hopefully will create a python package out of this. ###

###             --- Basic Description ---               ###

# The Security Verification in ROS (svROS) 
# tool concerns a technique, based on the software verification 
# perspective, to automatically verify system-wide properties 
# related to the security configuration of ROS2-based applications.

# To that purpose it will model the ROS architecture, 
# as well as the network communication behaviour, 
# in Alloy, a formal specification language 
# and analysis tool supported by a model-finder, 
# with which system-wide properties will subsequently model-checked.

###             --- Basic Description ---               ###

import os, re
from setuptools import setup, find_packages
from setuptools.command.install import install

SOURCE = os.path.relpath(os.path.join(os.path.dirname(__file__), 'svROS'))
DATA   = os.path.relpath(os.path.join(os.path.dirname(__file__), 'svROS/__init__.py'))
UTILS  = os.path.relpath(os.path.join(os.path.dirname(__file__), 'svROS/utils'))
requirements = [r for r in open(f'requirements.txt').readlines() if r]

# Worth-Mention https://stackoverflow.com/a/38933723
class PostInstallCommand(install):
    """Post-installation for installation mode."""
    def run(self):
        install.run(self)
        os.system("cat ./INFO")

def info(keyword : str) -> str:
    re_ = fr"^__{keyword}__\s*=\s*(u|f|r)?['\"]([^'\"]*)['\"]"
    match = re.search(re_, open(DATA, "rt").read(), re.M)
    if match:
        # match(1) matches to a possible defined string option --> (u|f|r)?.
        str_ = match.group(2)
    else:
        raise RuntimeError(f"Unable to find {keyword} variable-string in %s." % (DATA,))
    return str(str_)

# Worth-Mention of https://stackoverflow.com/a/36693250
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(".", path, filename))
    return paths

# SETUP variables 
__version__ = info('version')
__author__  = info('author')
__email__   = info('email')
__license__ = info('license')
__README__  = os.path.join(os.path.dirname(__file__), 'README.md')
# SETUP variables 

extra = package_files(f"{UTILS}")

setup(
    name             = "svROS",
    version          = __version__,
    author           = __author__,
    author_email     = __email__,
    description      = "Security Verification in ROS",
    license          = __license__,
    # README
    long_description = open(__README__, "rt").read(),
    long_description_content_type = "text/markdown",
    # README
    keywords         = "python sros2 ros2 property-specification model-checking alloy haros",
    url              = "https://github.com/luis1ribeiro/svROS",
    packages         = find_packages(SOURCE),
    package_dir      = {'': SOURCE},
    entry_points     = {"console_scripts": ["svROS = svROS.svROS:main"]},
    package_data     = {"svROS": extra},
    install_requires = requirements,
    extras_require   = {},
    cmdclass={
        'install': PostInstallCommand,
    },
    zip_safe         = True
)

###             --- HAROS ---               ###

# It should be noticed the importance of having
# HAROS, a framework for code quality assurance of ROS,
# as guidance and data provider through 
# out the development of this verification tool.

###             --- HAROS ---               ###