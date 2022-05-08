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
from haros.extractor    import RoscppExtractor, RospyExtractor
from distutils.spawn import find_executable

# Launcher
global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

from svLauncherXML import LauncherParserXML, NodeTag
from svLauncherPY import LauncherParserPY, NodeCall

"ROS2-based Topic already parse for node handling."
class Topic(object):
    TOPICS = {}
    """
        Topic
            \__ Name
            \__ Type
    """
    def __init__(self, _id, name, topic_type):
        self.id   = _id
        self.name = name
        self.type = topic_type
        Topic.TOPICS[index] = self

    @classmethod
    def init_topic(cls, **kwargs):
        return cls(_id=kwargs['_id'], name=kwargs['name'], type=kwargs['topic_type'])

"ROS2-based Node already parse, with remaps and topic handling."
class Node(object):
    PRE_PROCESSED_NODES = {}
    NODES               = {}
    """
        Node
            \__ INFO FROM NODETAG OR NODECALL
            \__ Topic subscribing and publishing
    """
    def __init__(self, name, package, executable, remaps, enclave=None, topic_sub=[], topic_pub=[]):
        self.name       = name
        self.package    = package
        self.executable = executable
        self.remaps     = remaps
        self.enclave    = enclave
        self.subscribes = topic_sub
        self.publishes  = topic_pub
        # Add to NODES class variable.
        index = self.executable
        Node.NODES[index] = self

    @classmethod
    def nodes_to_string(cls):
        for node in cls.NODES:
            return False
    
    @classmethod
    def init_node(cls, **kwargs):
        return cls(name=kwargs['name'], package=kwargs['package'], executable=kwargs['executable'], remaps=kwargs['remaps'], enclave=kwargs.get('enclave'), topic_sub=kwargs['subscribes'], topic_pub=kwargs['publishes'])

    @property
    def name(self):
        name = self.package + '/' + self._name
        return name
    
    @name.setter
    def name(self, value):
        self._name = value

    @property
    def executable(self):
        exec_ = self.package + '/' + self._executable
        return exec_
    
    @executable.setter
    def executable(self, value):
        self._executable = value

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
        ### LAUNCH STRUCTURES FROM ROS2 => Still deprecated!
        from launch.launch_description_sources import get_launch_description_from_any_launch_file
        using_ros2_structures = False
        launch_description    = None
        try:
            launch_description = get_launch_description_from_any_launch_file(self.file)
            using_ros2_structures = False # Once will be true.
        except:
            pass
        if using_ros2_structures == True:
           return self._default_parse(ros2_entities=launch_description.entities)
        # PARSING EXTENSION
        if self.extension == '':
            return False
        if self.extension == 'xml':
            if LauncherParserXML(file=self.file).parse():
                return NodeTag.NODES, NodeTag.PACKAGES_NODES 
        if self.extension == 'py':
            if LauncherParserPY(file=self.file).parse():
                return NodeCall.NODES, NodeCall.PACKAGES_NODES 

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
        self.find_packages(paths=[self.ros_workspace, self.ros_distro])

    """ === Predefined functions === """
    def find_packages(self, paths):
        # Find ros package installer => colcon 
        colcon = find_executable('colcon')
        if colcon is None:
            return False
        command = [colcon, 'list', '--base-paths']
        if paths is None: return False
        command.extend(paths)
        # Package processing.
        try:
            pkglist = subprocess.check_output(command).decode().split('\n')[:-1]
            svrosExport.remove_log_dir()
        except Exception:
            raise
        pkgs={}
        for i in list(map(lambda info: info.split('\t'), pkglist)):
            if i[0] not in pkgs and len(i) >= 2:
                pkgs[i[0]] = i[1]
        t = self.set_packages(packages=pkgs)
        return t
    
    def set_packages(self, packages):
        try:
            self.packages = packages
        except:
            return False
        return True
    """ === Predefined functions === """

"Default svROS exporter class..."      
@dataclass
class svrosExport:
    launch        : str
    ros_distro    : str
    ros_workspace : str
    project_dir   : str = ''
    log           : str = None

    def __post_init__(self):
        if self.log is None:
            print('No logger provided.')
        if os.path.exists(f'/opt/ros2/{self.ros_distro}/'):
            self.ros_distro = f'/opt/ros2/{self.ros_distro}/'
        else:
            return False

    """ === Predefined functions === """
    # Main exporter
    def _export(self):
        # Get all packages found.
        package_finder = PackageFinder(ros_workspace=self.ros_workspace, ros_distro=self.ros_distro)
        all_packages   = package_finder.packages
        launcher       = LauncherParser(file=self.launch)
        parser         = launcher.parse()
        if isinstance(parser, bool):
            return False
        nodes, packages          = parser[0], parser[1]
        Node.PRE_PROCESSED_NODES = nodes
        __VALID_PACKAGES__ = {package for package in packages}
        VALID_PACKAGES     = dict(filter(lambda package: package[0] in __VALID_PACKAGES__, all_packages.items()))
        if not self.get_valid_nodes(VALID_PACKAGES=VALID_PACKAGES, NODES_PACKAGES=packages):
            return False

    def get_valid_nodes(self, VALID_PACKAGES, NODES_PACKAGES):
        for package in VALID_PACKAGES:
            PACKAGE_PATH = VALID_PACKAGES[package]
            srcdir       = PACKAGE_PATH[len(self.ros_workspace):]
            # Process package source directory.
            srcdir     = os.path.join(self.ros_workspace, srcdir.split(os.sep, 1)[0])
            bindir     = os.path.join(self.ros_workspace, "build")
            cmake_path = os.path.join(PACKAGE_PATH, "CMakeLists.txt")

            executables_from_package, iscpp = svrosExport.executables_from_package(cmake_path=cmake_path, srcdir=srcdir, bindir=bindir, package_path=PACKAGE_PATH)
            nodes_from_package              = dict(map(lambda _node: (_node, executables_from_package.get(_node)), map(lambda node: node.executable, NODES_PACKAGES[package])))
            print(nodes_from_package, '=======')
            #if not svrosExport.process_nodes(NODES=nodes_from_package):
            #    return False

    @staticmethod
    def process_nodes(NODES):
        for node in NODES:
            info            = NODES[node]
            multi_part_node = isinstance(info, list)
            if multi_part_node:
                svrosExport.process_multi_part_node(NODE=info)
            else:
                return False

    @staticmethod
    def process_multi_part_node(NODE):
        for node_part in NODE:
            return False

    @staticmethod
    def executables_from_package(cmake_path, srcdir, bindir, package_path):
        """ LANG:
            \_ if CPP => True
            \_ if PY  => False
        """
        # CPP PACKAGES.
        if os.path.isfile(cmake_path):
            LANG = True
            "Courtesy to André's work in HAROS."
            parser = RosCMakeParser(srcdir, bindir)
            parser.parse(cmake_path)
            # Extracted executables information.
            executables = parser.executables
            installs = parser.include_dirs
            target_dict = dict()
            for target in executables.values():
                target_dict[target.name] = target.files
        # PYTHON PACKAGES.
        else:
            pattern = re.compile(r'^(\t|\s)*\'.*?\s*=\s*(.*?):main\'') # Pattern to catch def main.
            setup_path = f'{package_path}/setup.py' 
            if os.path.exists(setup_path) and os.path.isfile(setup_path):
                with open(setup_path) as setup:
                    match = re.findall(r'\'\s*([^\n\s]*?)\s+\=\s+([^\n\s]*?:main)\'', setup.read())
                    executables = match
                    if match is None:
                        raise Exception
                target_dict = dict()
                for target in executables:
                    path = f'{package_path}/{target[1].split(":")[0].replace(".", "/")}.py'
                    if not os.path.isfile(path):
                        raise Exception
                    target_dict[target[0]] = path
            else:
                raise Exception
            LANG = False
        return target_dict, LANG
    
    # Python exporter...
    def python_export(self):
        pass
    
    # Cpp exporter... Deprecated...
    def cpp_export(self):
        pass

    def store_in_dir(self, directory):
        pass

    @staticmethod
    def remove_log_dir(LOG=f'{WORKDIR}/log'):
        return shutil.rmtree(LOG)
    """ === Predefined functions === """


### TESTING ###
if __name__ == "__main__":
    file = '/home/luis/Desktop/example.xml'
    l = svrosExport(launch=file, ros_distro='galactic', ros_workspace='/home/luis/workspaces/ros2-galactic/')._export()
    # l = LauncherParser(file=file, extension='.xml').parse()
    print([NodeTag.NODES[n] for n in NodeTag.NODES])
    print(NodeTag.NODES)
    
    # file = '/home/luis/Desktop/ros2launch.py'
    # l = LauncherParser(file=file, extension='.xml').parse()
    # print([NodeCall.NODES[n] for n in NodeCall.NODES])
    # print(NodeCall.NODES)