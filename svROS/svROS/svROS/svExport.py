import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
from typing import ClassVar

# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svROS_Exception as excp, svROS_Info as info
from tools.Loader import Loader

# Needed for cpp nodes...
from haros.cmake_parser import RosCMakeParser
from haros.extractor    import RoscppExtractor, RospyExtractor
from bonsai.model import (
    CodeGlobalScope, CodeReference, CodeFunctionCall, pretty_str
)
from bonsai.analysis import (
    CodeQuery, resolve_reference, resolve_expression, get_control_depth,
    get_conditions, get_condition_paths, is_under_loop
)
from bonsai.py.py_parser import PyAstParser
from distutils.spawn import find_executable

# Launcher
global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

from svLauncherXML import LauncherParserXML, NodeTag
from svLauncherPY import LauncherParserPY, NodeCall

"Exporter CPP. => Using Regular Expressions only..."
@dataclass
class ExporterCPP:
    content: str

    def extract_publishers(self):
        publishers  = re.findall(r'(create_publisher|advertise)\<([^\>]*?)\>\s*\(\s*\"([^\"]*?)\"', self.content)
        pubs        = []
        for pub in publishers:
            topic = Topic(_id=len(Topic.TOPICS), name=pub[2], topic_type=pub[1])
            pubs.append(topic)
        return pubs
    
    def extract_subscribers(self):
        subscribers = re.findall(r'(create_subscription|subscribe)\<([^\>]*?)\>\s*\(\s*\"([^\"]*?)\"', self.content)
        subs        = []
        for sub in subscribers:
            topic = Topic(_id=len(Topic.TOPICS), name=sub[2], topic_type=sub[1])
            subs.append(topic)
        return subs

@dataclass
class ExporterPY:
    global_scope: CodeGlobalScope
    imports     : list
    from_imports: dict

    @staticmethod
    def process_topic_type(topic_type):
        if resolve_reference(topic_type) is None:
            topic_type = str(topic_type)[1:]
            if topic_type in self.imports:
                pass
            elif topic_type in self.from_imports:
                topic_type = self.from_imports[topic_type].replace('.', '/')
            else: raise Exception
        else: topic_type = resolve_reference(topic_type)
        return topic_type

    def extract_publishers(self):
        publishers  = (CodeQuery(self.global_scope).all_calls
                        .where_name(('Publisher', 'rospy.Publisher', 'create_publisher'))
                        .get())
        pubs        = []
        for pub in publishers:
            if pub.name == 'create_publisher':
                name, topic_type = pub.arguments[1], pub.arguments[0]
                if isinstance(topic_type, CodeReference):
                    topic_type = ExporterPY.process_topic_reference(topic_type=topic_type)
            else:
                name       = ExporterPY.extract_topic_name(call=pub)
                topic_type = ExporterPY.extract_topic_type(call=pub)
            topic = Topic(_id=len(Topic.TOPICS), name=name, topic_type=topic_type)
            pubs.append(topic)
        return pubs
    
    def extract_subscribers(self):
        subscribers = (CodeQuery(self.global_scope).all_calls
                         .where_name(('Subscriber', 'rospy.Subscriber', 'create_subscription'))
                         .get())
        subs        = []
        for sub in subscribers:
            if sub.name == 'create_subscription':
                name, topic_type = sub.arguments[1], sub.arguments[0]
                if isinstance(topic_type, CodeReference):
                    topic_type = ExporterPY.process_topic_reference(topic_type=topic_type)
            else:
                name       = ExporterPY.extract_topic_name(call=sub)
                topic_type = ExporterPY.extract_topic_type(call=sub)
            topic = Topic(_id=len(Topic.TOPICS), name=name, topic_type=topic_type)
            subs.append(topic)
        return subs

    @staticmethod
    def extract_topic_name(call):
        return resolve_expression(ExporterPY.get_arg(call, 0, 'name'))

    @staticmethod
    def extract_topic_type(call):
        topic_type = ExporterPY.get_arg(call, 1, 'data_class')
        return ExporterPY.process_topic_type(topic_type=topic_type)

    @staticmethod
    def get_arg(call, pos, name):
        try:
            return next(
                keyword.value
                for keyword in call.named_args
                if keyword.name == name)
        except StopIteration:
            try:
                return call.arguments[pos]
            except IndexError:
                return None

"Package class for haros integration."
@dataclass
class Package:
    name    : str
    path    : str
    nodes   : dict
    PACKAGES: ClassVar[dict] = field(default_factory=dict)

    def __post_init__(self):
        Package.PACKAGES[self.name] = self

@dataclass
class SourceFile:
    path: str

@dataclass
class NodeSource:
    name: str
    source_files: list

    def __post_init__(self):
        source_list = []
        for sf_path in self.source_files:
            sf = SourceFile(path=sf_path)
            source_list.append(sf)
        self.source_files = source_list

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
    last_workspace: ClassVar[str] = ''
    project_dir   : str = ''
    log           : str = None

    def __post_init__(self):
        if self.log is None:
            print('No logger provided.')
        if os.path.exists(f'/opt/ros2/{self.ros_distro}/'):
            self.ros_distro = f'/opt/ros2/{self.ros_distro}/'
        else:
            return False
        svrosExport.last_workspace = self.ros_workspace

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

            executables_from_package, iscpp, package = svrosExport.executables_from_package(cmake_path=cmake_path, srcdir=srcdir, bindir=bindir, package_path=PACKAGE_PATH, package=package)
            iscpp                                    = isinstance(iscpp, RoscppExtractor)
        
            nodes_from_package   = dict(map(lambda _node: (_node, executables_from_package.get(_node)), map(lambda node: node.executable, NODES_PACKAGES[package])))
            if not svrosExport.process_source_files(package=package, nodes_from_package=nodes_from_package, iscpp=iscpp):
                raise Exception

            #if not svrosExport.process_nodes(NODES=nodes_from_package):
            #    return False

    @staticmethod
    def process_source_files(package, nodes_from_package, iscpp):
        node_sources = []
        for n_source in nodes_from_package:
            source_files = nodes_from_package[n]
            node_source  = NodeSource(name=n_source, source_files=source_files)
            node_sources.append(node_source)
        package.nodes = nodes_sources
        # ...
        pass

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
    def executables_from_package(cmake_path, srcdir, bindir, package_path, package):
        """ LANG:
            \_ if CPP => True
            \_ if PY  => False
        """
        # CPP PACKAGES.
        if os.path.isfile(cmake_path):
            "Courtesy to André's work in HAROS."
            parser = RosCMakeParser(srcdir, bindir)
            parser.parse(cmake_path)
            # Extracted executables information.
            executables = parser.executables
            installs = parser.include_dirs
            target_dict = dict()
            for target in executables.values():
                target_dict[target.name] = target.files
            package   = Package(name=package, path=package_path, nodes=target_dict)
            extractor = RoscppExtractor(package=package, workspace=svrosExport.last_workspace)
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
                    target_dict[target[0]] = [path]
            else:
                raise Exception
            package   = Package(path=package_path, nodes=target_dict)
            extractor = RospyExtractor(package=package, workspace=svrosExport.last_workspace)
        return target_dict, extractor, package
    
    # Python exporter...
    @staticmethod
    def python_export(source_file, node: Node):
        parser = PyAstParser(workspace=svrosExport.last_workspace)
        parser.parse(f'{source_file}')
        __gs__ = parser.global_scope
        with open(source_file, 'r') as source_content:
            source_content = source_content.read()
        from_imports = re.findall(r'\n(\t|\s)*from\s*(.*?)\s*import\s*(.*?)\s*\n', source_content)
        from_imports = dict(map(lambda pair: (pair[1], f'{pair[0]}.{pair[1]}'), list(map(lambda fi: (fi[1], fi[2]), from_imports))))
        imports      = re.findall(r'\n(\t|\s)*import\s*(.*?)\s*\n', source_content)
        imports      = list(map(lambda fi: fi[1], imports))
        # Exporter PY
        py   = ExporterPY(global_scope=__gs__ , imports=imports, from_imports=from_imports)
        # Publisher calls
        pubs = py.extract_publishers()
        map(lambda call: node.publishes.append(call), pubs)
        # Subscriber calls
        subs = py.extract_subscribers()
        map(lambda call: node.subscribes.append(call), subs)
        return True

    # Cpp exporter... Deprecated...
    @staticmethod
    def cpp_export(source_file, node: Node):
        with open(source_file, 'r') as source_content:
            source_content = source_content.read()
        # Exporter CPP
        cpp  = ExporterCPP(content=source_content)
        # Publisher calls
        pubs = cpp.extract_publishers()
        map(lambda call: node.publishes.append(call), pubs)
        # Subscriber calls
        subs = cpp.extract_subscribers()
        map(lambda call: node.subscribes.append(call), subs)
        return True

    def store_in_dir(self, directory):
        pass

    @staticmethod
    def remove_log_dir(LOG=f'{WORKDIR}/log'):
        return shutil.rmtree(LOG)
    """ === Predefined functions === """

### TESTING ###
if __name__ == "__main__":
    file = '/home/luis/Desktop/ros2launch.py'
    l = svrosExport(launch=file, ros_distro='galactic', ros_workspace='/home/luis/workspaces/ros2-galactic/')._export()
    # # l = LauncherParser(file=file, extension='.xml').parse()
    # print([NodeTag.NODES[n] for n in NodeTag.NODES])
    # print(NodeTag.NODES)
    
    # file = '/home/luis/Desktop/ros2launch.py'
    # l = LauncherParser(file=file, extension='.xml').parse()
    # print([NodeCall.NODES[n] for n in NodeCall.NODES])
    # print(NodeCall.NODES)