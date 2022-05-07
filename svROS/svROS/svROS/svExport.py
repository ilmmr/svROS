import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict

# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svROS_Exception as excp, svROS_Info as info
from tools.Loader import Loader

# Needed for cpp nodes...
from haros.cmake_parser import RosCMakeParser
from distutils.spawn import find_executable

# Launcher
global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

from svLauncherXML import LauncherParserXML, NodeTag
from svLauncherPY import LauncherParserPY, NodeCall

""" 
    This file contains the necessary classes and methods to export information from the launch file specified within the config file.

    Although haros already provides a well-riched launcher parser, however it is not compatible with ROS2...

    SCHEMA that ros2 provides is deprecated also... So we'll try to run ros2 launch -p instead:
        => ros2 launch $file -p :: 

    However, many of the current work explored here is thanks to haros and André's work!
    The ideia here was to take what previously worked by haros, to export the necessary data to this tool, while considering the notable changes on ROS2 launch schema...
"""
"Launcher parser in order to retrieve information about possible executables..."
@dataclass
class LauncherParser:
    file      : str
    extension : str  = ''

    def __post_init__(self):
        if not (os.path.exists(self.file) and os.path.isfile(self.file)):
            return False
        base, ext = os.path.splitext(file)
        if ext.lower() not in ['.xml', '.py']:
            # Launch File given is depecrated. Supported formats: .py and .xml!
            return False
        self.extension = ext.lower().split('.')[1]

    """ === Predefined functions === """
    "Launch Parser"
    def parse(self):
        # Use ros2 launch structures.
        from launch.launch_description_sources import get_launch_description_from_any_launch_file
        using_ros2_structures = False
        launch_description    = None
        # still deprecated...
        try:
            launch_description = get_launch_description_from_any_launch_file(self.file)
            using_ros2_structures = False # Once will be true.
        except:
            pass
        if using_ros2_structures == True:
           return self._default_parse(ros2_entities=launch_description.entities)

        if self.extension == '':
            return False
        if self.extension == 'xml':
            return LauncherParserXML(file=self.file).parse()
        if self.extension == 'py':
            return LauncherParserPY(file=self.file).parse()

    "Predefined method to extract entities from ros2! This uses the default ros2 launch structure to parse each entitie."
    def _default_parse(self, ros2_entities):
        # Needed for tag checking.
        import launch, launch_ros
        VALID_TAGS: {
            launch_ros.actions.node.Node,
            launch.actions.set_launch_configuration.SetLaunchConfiguration,
            launch.actions.declare_launch_argument.DeclareLaunchArgument,
            launch.conditions.if_condition.IfCondition,
            launch.conditions.unless_condition.UnlessCondition,
            launch.substitutions.text_substitution.TextSubstitution
        }
        return False
    """ === Predefined functions === """

""" 
    This file contains the necessary classes and methods to export information about the ros2 running environment that the user may want to analyze.

    Notably, since svROS it is expected to be full compatible with haros, later the user might want to call the harosExport class defined bellow.
    However, as default, svROSexport would take place when it comes to export parsing analysis.

    NAIVE EXTRACTOR... C++ parser is deprecated...
"""
"Default Colcon package finder, in order to retrieve information about possible executables"
@dataclass
class PackageFinder:
    ros_workspace : str  = ''
    ros_distro    : str  = '' 
    packages      : list = field(default_factory=list)

    def __post_init__(self):
        # Find and set packages list.
        if packages is []:
            self.find_packages()

    """ === Predefined functions === """
    def find_packages(self, paths=[self.ros_workspace, self.ros_distro]):
        # Find ros package installer => colcon 
        colcon = find_executable('colcon')
        if colcon is None:
            return False
        command = [colcon, 'list', '--base-paths']
        if paths is None: return False
        command.extend(paths)

        # Package processing.
        pkglist = subprocess.check_output(cmd).decode().split('\n')[:-1]
        pkgs={}
        for i in list(map(lambda info: info.split('\t'), pkglist)):
            if i[0] not in pkgs and len(i) >= 2:
                pkgs[i[0]] = i[1]
        t = self.set_packages(self, pkgs)
        return t

    def set_packages(self, packages=[]):
        if packages is not []:
            self.packages = packages
            return True
        return False
    """ === Predefined functions === """

"Default svROS exporter class..."      
@dataclass
class svrosExport:

    launch        : str
    project_dir   : str = ''
    ros_distro    : str
    ros_workspace : str
    log        : logging.getLogger() = None

    # After class __init__
    def __post_init__(self):
        if self.log is None:
            self.log = format_logger()
        # ros_distro full path
        if os.path.exists(f'/opt/ros2/{self.ros_distro}/'):
            self.ros_distro = f'/opt/ros2/{self.ros_distro}/'
        else:
            return False

    """ === Predefined functions === """
    # Main exporter...
    def _export(self):
        # get all packages
        package_finder = PackageFinder(ros_workspace=self.ros_workspace, distro=self.ros_distro)
        all_packages = package_finder.packages

        # launcher information
        launcher = LauncherParser(file=self.launch)
        # try to parse...
        if not launcher.parse():
            return False
        nodes, remaps = launcher.nodes, launcher.remaps
        pass

    # Python exporter...
    def python_export(self):
        pass
    
    # Cpp exporter... Deprecated...
    def cpp_export(self):
        pass

    def store_in_dir(self, directory=self.project_dir):
        pass
    """ === Predefined functions === """

@dataclass
class harosExport:

    workspace: str

    """ === Predefined functions === """
    pass
    """ === Predefined functions === """


### TESTING ###
if __name__ == "__main__":
    file = '/home/luis/Desktop/example.xml'
    l = LauncherParser(file=file, extension='.xml')
    print([NodeTag.NODES[n] for n in NodeTag.NODES])
    print(NodeTag.NODES)
    
    file = '/home/luis/Desktop/ros2launch.py'
    l = LauncherParserPY(file=file).parse()
    print([NodeCall.NODES[n] for n in NodeCall.NODES])
    print(NodeCall.NODES)