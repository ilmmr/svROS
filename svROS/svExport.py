import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, xmlschema, json, pickle
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from .svInfo import color, svException, svWarning
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
# Parsers
import xml.etree.ElementTree as ET
from lark import Lark, tree
# Launcher
from .svLauncherXML import LauncherParserXML, NodeTag
from .svLauncherPY import LauncherParserPY, NodeCall
# Data
from .svData import Node, Topic, Package

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, 'schemas')

"YAML default dumper"
# Worth-Mention https://stackoverflow.com/a/39681672
class DefaultDumper(Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(DefaultDumper, self).increase_indent(flow, False)

"Exporter CPP. => Using Regular Expressions only..."
@dataclass
class ExporterCPP:
    content: str

    @staticmethod
    def call_grammar(topic_call: str):
        # Grammar to parse arguments.
        grammar = """
            sentence: CALL "<" TOPIC_TYPE ">" "(" LIXO? ASP TOPIC_NAME ASP

            CALL:"create_publisher" | "advertise" | "create_subscription" | "subscribe"
            TOPIC_TYPE:/(?!\>)[a-zA-Z0-9_\/\-.\:]+/
            TOPIC_NAME:/(?!\,)[a-zA-Z0-9_\/\-.\:]+/
            LIXO:/(?!\")[^\"]+/
            ASP:/\"/

            %import common.WS
            %ignore WS
        """
        parser = Lark(grammar, start='sentence', ambiguity='explicit')
        if not parser.parse(topic_call):
            return False
        return True

    def extract_publishers(self):
        publishers  = re.findall(r'((create_publisher|advertise)\<([^\>]*?)\>\s*\([^\"]*?\"([^\"]*?)\")', self.content)
        pubs        = []
        for pub in publishers:
            call, name, topic_type = pub[0], pub[3], pub[2]
            if not ExporterCPP.call_grammar(topic_call=call):
                continue
            topic_type = topic_type.replace('::', '/')
            topic      = Topic(name=name, topic_type=topic_type)
            pubs.append(topic)
        return pubs
    
    def extract_subscribers(self):
        subscribers = re.findall(r'((create_subscription|subscribe)\<([^\>]*?)\>\s*\(\s*\"([^\"]*?)\")', self.content)
        subs        = []
        for sub in subscribers:
            call, name, topic_type = sub[0], sub[3], sub[2]
            if not ExporterCPP.call_grammar(topic_call=call):
                continue
            topic_type = topic_type.replace('::', '/')
            topic      = Topic(name=name, topic_type=topic_type)
            subs.append(topic)
        return subs

@dataclass
class ExporterPY:
    global_scope: CodeGlobalScope
    imports     : list
    from_imports: dict

    @staticmethod
    def process_topic_reference(topic_type, imports, from_imports):
        if resolve_reference(topic_type) is None:
            topic_type = str(topic_type)[1:]
            if topic_type in from_imports:
                topic_type = from_imports[topic_type].replace('.', '/')
            elif topic_type in imports:
                pass
            else: raise svException(message=f'Topic type {str(topic_type)} failed to be processed.')
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
                if isinstance(name, CodeReference):
                    continue
                if isinstance(topic_type, CodeReference):
                    topic_type = ExporterPY.process_topic_reference(topic_type=topic_type, imports=self.imports, from_imports=self.from_imports)
            else:
                name       = ExporterPY.extract_topic_name(call=pub)
                topic_type = ExporterPY.extract_topic_type(call=pub, imports=self.imports, from_imports=self.from_imports)
            topic = Topic(name=name, topic_type=topic_type)
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
                if isinstance(name, CodeReference):
                    continue
                if isinstance(topic_type, CodeReference):
                    topic_type = ExporterPY.process_topic_reference(topic_type=topic_type, imports=self.imports, from_imports=self.from_imports)
            else:
                name       = ExporterPY.extract_topic_name(call=sub)
                topic_type = ExporterPY.extract_topic_type(call=sub, imports=self.imports, from_imports=self.from_imports)
            topic = Topic(name=name, topic_type=topic_type)
            subs.append(topic)
        return subs

    @staticmethod
    def extract_topic_name(call):
        return resolve_expression(ExporterPY.get_arg(call, 0, 'name'))

    # Method developed by HAROS.
    @staticmethod
    def extract_topic_type(call, imports, from_imports):
        topic_type = ExporterPY.get_arg(call, 1, 'data_class')
        return ExporterPY.process_topic_reference(topic_type=topic_type, imports=imports, from_imports=from_imports)

    # Method developed by HAROS.
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

@dataclass
class SourceFile:
    path : str
    iscpp: bool
    publishes   : list = field(default_factory=list)
    subscribes  : list = field(default_factory=list)

    def __post_init__(self):
        try:    
            if self.iscpp: self.publishes, self.subscribes = svrosExport.cpp_export(self.path)
            else: self.publishes, self.subscribes = svrosExport.python_export(self.path)
        except Exception:
            raise svException(message=f'Failed to export/parse source file {self.path}.')

@dataclass
class NodeSource:
    name        : str
    source_files: list
    iscpp       : bool
    publishes   : list = field(default_factory=list)
    subscribes  : list = field(default_factory=list)

    def __post_init__(self):
        source_list = []
        for sf_path in self.source_files:
            sf = SourceFile(path=sf_path, iscpp=self.iscpp)
            source_list.append(sf)
        self.source_files = source_list

    def process_calls(self):
        for sf in self.source_files:
            self.publishes  += sf.publishes
            self.subscribes += sf.subscribes
        return True

"Launcher parser in order to retrieve information about possible executables..."
@dataclass
class LauncherParser:
    file      : str
    extension : str  = ''

    def __post_init__(self):
        if not (os.path.exists(self.file) and os.path.isfile(self.file)):
            return False
        base, ext = os.path.splitext(self.file)
        if ext.lower() not in ['.xml', '.py']:
            # Launch File given is depecrated. Supported formats: .py and .xml!
            return False
        self.extension = ext.lower().split('.')[1]

    """ === Predefined functions === """
    # Launch Parser
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
            raise svException(message=f'Failed to load ROS2 packages.')
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
    launch        : list
    ros_distro    : str
    ros_workspace : str
    project       : str
    project_dir   : str
    last_workspace: ClassVar[str]

    def __post_init__(self):
        if os.path.exists(f'/opt/ros2/{self.ros_distro}/'):
            self.ros_distro = f'/opt/ros2/{self.ros_distro}/'
        else:
            return False
        svrosExport.last_workspace = self.ros_workspace

    """ === Predefined functions === """
    # Main exporter
    def launch_export(self):
        # Get all packages found.
        package_finder = PackageFinder(ros_workspace=self.ros_workspace, ros_distro=self.ros_distro)
        all_packages   = package_finder.packages
        for lf in self.launch:
            print(f'[svROS] {color.color("BOLD", color.color("BLUE", "EXPORTING FILE"))} {color.color("BOLD", color.color("UNDERLINE", lf))}')
            if not self._export(LAUNCH_FILE=lf, ALL_PACKAGES=all_packages):
                print(f'[svROS] {color.color("BOLD", color.color("RED", "EXPORTING ERROR"))} {color.color("BOLD", color.color("UNDERLINE", lf))}')
                break
            print(f'[svROS] {color.color("BOLD", color.color("GREEN", "FINISHED"))} {color.color("BOLD", color.color("UNDERLINE", lf))}')
        # Retrieve information back to the PROJECT FOLDER!
        if not self.generate_artifacts():
            return False
        return True

    def _export(self, LAUNCH_FILE, ALL_PACKAGES):
        launcher       = LauncherParser(file=LAUNCH_FILE)
        parser         = launcher.parse()
        if isinstance(parser, bool):
            return False
        # Process package.
        packages           = parser[1]
        __VALID_PACKAGES__ = {package for package in packages}
        VALID_PACKAGES     = dict(filter(lambda package: package[0] in __VALID_PACKAGES__, ALL_PACKAGES.items()))
        if not self.get_valid_nodes(VALID_PACKAGES=VALID_PACKAGES, NODES_PACKAGES=packages):
            return False
        return True
    
    def get_valid_nodes(self, VALID_PACKAGES, NODES_PACKAGES):
        for package in VALID_PACKAGES:
            PACKAGE_PATH = VALID_PACKAGES[package]
            srcdir       = PACKAGE_PATH[len(self.ros_workspace):]
            # Process package source directory.
            srcdir     = os.path.join(self.ros_workspace, srcdir.split(os.sep, 1)[0])
            bindir     = os.path.join(self.ros_workspace, "build")
            cmake_path = os.path.join(PACKAGE_PATH, "CMakeLists.txt")

            executables_from_package, iscpp, cls_package = svrosExport.executables_from_package(cmake_path=cmake_path, srcdir=srcdir, bindir=bindir, package_path=PACKAGE_PATH, package=package)
            iscpp                                        = isinstance(iscpp, RoscppExtractor)
        
            nodes_from_package   = dict(map(lambda _node: (_node, executables_from_package.get(_node)), map(lambda node: node.executable, NODES_PACKAGES[package])))
            nodes = list(map(lambda node: Node.init_node(**(node.__dict__)), NODES_PACKAGES[package]))
            if not svrosExport.process_source_files(package=cls_package, nodes_from_package=nodes_from_package, nodes=nodes, iscpp=iscpp):
                raise svException(message=f'Failed to export source files.')
        return True

    @staticmethod
    def process_source_files(package, nodes_from_package, nodes, iscpp):
        node_sources = []
        # print('===', package.name, nodes_from_package, '=> nodes_from_package ===')
        for n_source in nodes_from_package:
            source_files = nodes_from_package[n_source]
            node_source  = NodeSource(name=n_source, source_files=source_files, iscpp=iscpp)
            # Process Subscribe and Publish calls.
            node_source.process_calls()
            # Node processing.
            nodes_ = list(filter(lambda n: n.executable == n_source, nodes))
            for node in nodes_: node.store_node_source(source=node_source)
            node_sources.append(node_source)
        package.nodes = node_sources
        return True

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
            # Pattern to catch def main.
            pattern = re.compile(r'^(\t|\s)*\'.*?\s*=\s*(.*?):main\'')
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
            package   = Package(name=package, path=package_path, nodes=target_dict)
            extractor = RospyExtractor(package=package, workspace=svrosExport.last_workspace)
        return target_dict, extractor, package
    
    # Python exporter
    @staticmethod
    def python_export(source_file):
        parser = PyAstParser(workspace=svrosExport.last_workspace)
        parser.parse(f'{source_file}')
        __gs__ = parser.global_scope
        with open(source_file, 'r') as source_content:
            source_content = source_content.read()
        from_imports = re.findall(r'(\t|\s)*from\s*(.*?)\s*import\s*(.*?)\s*\n', source_content)
        from_imports = dict(map(lambda pair: (pair[1], f'{pair[0]}.{pair[1]}'), list(map(lambda fi: (fi[1], fi[2]), from_imports))))
        imports      = re.findall(r'(\t|\s)*import\s*(.*?)\s*\n', source_content)
        imports      = list(map(lambda fi: fi[1], imports))
        # Exporter PY
        py   = ExporterPY(global_scope=__gs__ , imports=imports, from_imports=from_imports)
        # Publisher calls
        pubs = py.extract_publishers()
        map(lambda call: node.publishes.append(call), pubs)
        # Subscriber calls
        subs = py.extract_subscribers()
        map(lambda call: node.subscribes.append(call), subs)
        return pubs, subs

    # Cpp exporter: Deprecated...
    @staticmethod
    def cpp_export(source_file):
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
        return pubs, subs

    # Function that will origin the needed files to run the analysis.
    def generate_artifacts(self):
        DIRECTORY = self.project_dir
        # YAML-file
        data_yml  = self.generate_config_file()
        with open(f'{self.project_dir}/config.yml', 'w+') as config:
            dump(data_yml, config, Dumper=DefaultDumper, sort_keys=False, default_flow_style=False, explicit_start=True, version=(1,1), indent=4)
        # SROS-file
        sros_root = self.generate_security_file()
        with open(f'{self.project_dir}/policies.xml', 'w+') as sros:
            ET.indent(sros_root)
            ET.register_namespace('xi', 'http://www.w3.org/2001/XInclude')
            __xml__ = ET.tostring(sros_root, encoding='unicode')
            sros.write(__xml__)
        # JSON-file
        data_json = self.generate_data_file(DATADIR=f'{self.project_dir}/data/')
        with open(f'{self.project_dir}/data/source.json', 'w+') as data:
            json.dump(data_json, data, sort_keys=False, indent=4)
        return True

    # Retrieve to a YAML-based file
    def generate_config_file(self):
        default_configuration = {'project': self.project, 'launch': self.launch, 'model': {'steps': 20, 'inbox': 4, 'behaviour': ['']}}
        tuple = Node.process_config_file()
        return {'configurations': default_configuration, 'packages': list(set(map(lambda package: package.name.lower(), Package.PACKAGES))), 'nodes': tuple[0], 'topics': tuple[1], 'types': Topic.list_of_types(), 'states': [None] }

    # Retrieve to a JSON-based file
    def generate_data_file(self, DATADIR):
        OBJDIR = f'{DATADIR}objects/'
        if not os.path.exists(OBJDIR):
            os.makedirs(OBJDIR)
        # SAVE using PICKLE.
        package_file, topic_file, node_file = open(f'{OBJDIR}Packages.obj', 'wb+'), open(f'{OBJDIR}Topics.obj', 'wb+'), open(f'{OBJDIR}Nodes.obj', 'wb+')
        pickle.dump(Package.PACKAGES, package_file, pickle.HIGHEST_PROTOCOL)
        pickle.dump(Topic.TOPICS, topic_file, pickle.HIGHEST_PROTOCOL)
        pickle.dump(Node.NODES, node_file, pickle.HIGHEST_PROTOCOL)
        # Returning object.
        return {'packages': list(set(map(lambda package: package.name.lower(), Package.PACKAGES))), 'nodes': dict(map(lambda node: (node.replace('::', '/'), Node.to_json(node)) , Node.NODES))}
    
    def generate_security_file(self):
        sch      = f'{SCHEMAS}/sros/sros.xsd'
        schema   = xmlschema.XMLSchema(sch) 
        tmp      = f'{SCHEMAS}/sros/template.xml'
        template = ET.parse(tmp).getroot()
        template = Node.process_sros_file(template=template)
        try:
            validate = schema.validate(template)
        except Exception: svException(message=f'Failed to validate SROS schema.')
        # default  = Node.retrieve_sros_default_tag(template=template)
        return template

    # Retrieve associated enclave file.
    @property
    def enclave_file(self):
        return self.project_dir + f'/{self.project}-sros.xml'
        
    @staticmethod
    def remove_log_dir(LOG=f'{WORKDIR}/log'):
        return shutil.rmtree(LOG)
    """ === Predefined functions === """

### TESTING ###
if __name__ == "__main__":
    project_dir = os.path.expanduser("~") + '/.svROS/projects/Project'
    project     = 'project'
    file = '/home/luis/Desktop/ros2launch.py'
    fil2 = '/home/luis/Desktop/example.xml'
    export = svrosExport(launch=[fil2, file], ros_distro='galactic', ros_workspace='/home/luis/workspaces/ros2-galactic/', project=project, project_dir=project_dir)
    if not export.launch_export():
        print('ERRO::TESTE')
### TESTING ###