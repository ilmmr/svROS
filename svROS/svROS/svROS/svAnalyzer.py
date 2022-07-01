import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, xmlschema
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException, svWarning
# Node parser
from svData import svROSNode, svROSProfile, svROSEnclave, svROSObject, svState, Node, Package, Topic
import xml.etree.ElementTree as ET

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

""" 
    This file contains the necessary classes and methods to translate from Python Structures into Alloy configuration model.
    Additionally some methods were implemented to approprietally retrieve text-based structures into Alloy.
"""
@dataclass
class svHPLParser(object):
    node      : svROSNode
    properties: list = field(default_factory=list)

"Main class that implements the analyzing process while accounting the configuration."
@dataclass
class svAnalyzer(object):
    EXTRACTOR     : object
    MODELS_DIR    : str

    def __post_init__(self):
        # GET FROM EXTRACTOR
        project, PROJECT_DIR, scopes = self.EXTRACTOR.project, self.EXTRACTOR.PROJECT_DIR, self.EXTRACTOR.scopes
        module_name, sros_module_name = "module " + str(project) + "\n/* === PROJECT " + str(project).upper() + " ===*/", "module sros-" + str(project) + "\n/* === PROJECT " + str(project).upper() + " ===*/"
        self.meta_model, self.sros_model = self.load_configuration(MODELS_DIR=self.MODELS_DIR, PROJECT_DIR=PROJECT_DIR, name=project.lower())
        # self.configuration = Configuration(c_name, self.nodes, self.topics, scopes, properties=self.properties)
        # self.specification = (self.module_name + meta_model + self.configuration.specification())
	
    @staticmethod
    def run_dir():
        return os.getcwd() 
	
    @staticmethod
    def load_configuration(MODELS_DIR, PROJECT_DIR, name):
        # ROS META-MODEL
        with open(f'{MODELS_DIR}/ros_base.als') as f:
            ros_meta_model = f.read()
        # SROS META-MODEL
        with open(f'{MODELS_DIR}/sros_base.als') as f:
            sros_meta_model = f.read()
        return ros_meta_model, sros_meta_model

    # ALLOY => Runs Model Checking in ROS_MODEL
    def ros_verification(self):
        NODES, TOPICS = svROSNode.NODES, Topic.TOPICS
        ROS_FILE = self.generate_ros_model(NODES=NODES, TOPICS=TOPICS)
        if not os.path.isfile(path=ROS_FILE): return False
        else: return True

    def generate_ros_model(self, NODES, TOPICS):
        model, file_path = self.meta_model, f'{self.EXTRACTOR.PROJECT_DIR}models/ros-concrete.als'
        if not os.path.exists(path=file_path): open(file_path, 'w+').close()
        if not os.path.isfile(path=file_path): raise svException('Unexpected error happend while creating ROS file.')
        # NODES.
        model += '/* === NODES === */\n'
        model += ''.join(list(map(lambda node: str(NODES[node]), NODES)))
        # TOPICS.
        model += '/* === NODES === */\n\n/* === TOPICS === */\n'
        model += Topic.topic_declaration()
        model +=  '/* === TOPICS === */\n\n/* === NODE BEHAVIOUR === */\n'
        #model += svROSNode.node_property_behaviour()
        model += '/* === NODE BEHAVIOUR === */\n\n/* === OBSERVABLE DETERMINISM === */\n'
        model += svROSNode.observable_determinism()
        model += '\n/* === OBSERVABLE DETERMINISM === */'
        with open(file_path, 'w+') as ros: ros.write(model)
        return file_path
    
    # ALLOY => Runs Structure Checking in SROS_MODEL
    def security_verification(self):
        # NODES = dict(filter(lambda node: node[1].profile is not None, svROSNode.NODES.items())) ==> NOT NEEDED. 
        ENCLAVES, OBJECTS, PROFILES = svROSEnclave.ENCLAVES, svROSObject.OBJECTS, svROSProfile.PROFILES
        SROS_FILE = self.generate_sros_model(PROFILES=PROFILES, ENCLAVES=ENCLAVES, OBJECTS=OBJECTS)
        if not os.path.isfile(path=SROS_FILE): return False
        else: return True
        # RUN ALLOY.
    
    def generate_sros_model(self, PROFILES, ENCLAVES, OBJECTS):
        model, file_path = self.sros_model, f'{self.EXTRACTOR.PROJECT_DIR}models/sros-concrete.als'
        if not os.path.exists(path=file_path): open(file_path, 'w+').close()
        if not os.path.isfile(path=file_path): raise svException('Unexpected error happend while creating SROS file.')
        # ENCLAVES.
        model += '/* === ENCLAVES === */\n'
        model += ''.join(list(map(lambda enclave: str(ENCLAVES[enclave]), ENCLAVES)))
        # NODES.
        model += '/* === ENCLAVES === */\n\n/* === PROFILES === */\n'
        model += ''.join(list(map(lambda profile: str(PROFILES[profile]), PROFILES)))
        # OBJECTS.
        model += '/* === PROFILES === */\n\n/* === OBJECTS === */\n'
        model += ''.join(list(map(lambda obj: str(OBJECTS[obj]), OBJECTS)))
        model += '/* === OBJECTS === */'
        with open(file_path, 'w+') as sros: sros.write(model)
        return file_path

"Main exporter parser from current project's directory files: SROS and configuration file"
@dataclass
class svProjectExtractor:
    project       : str
    PROJECT_DIR   : str
    IMPORTED_DATA : dict

    # Extract from SROS file
    def extract_sros(self, sros_file=''):
        if not sros_file:
            sros_file = f'{self.PROJECT_DIR}policies.xml'
        if not self.validate_sros(sros=sros_file):
            raise svException('Failed to validate SROS file schema.')
        tree = ET.parse(sros_file)
        root = tree.getroot()
        if not root.tag == "policy":
            raise svException('Failed to validate SROS file schema.')
        # Load SROS DATA.
        self.sros, enclaves = root, root.findall('.//enclave')
        for enclave in enclaves:
            path, profiles = enclave.get('path'), enclave.findall('.//profile')
            enclave        = svROSEnclave(path=path, profiles=profiles)
        return True

    def validate_sros(self, sros):
        sch      = f'{SCHEMAS}sros/sros.xsd'
        schema   = xmlschema.XMLSchema(sch)
        template = ET.parse(sros).getroot()
        try:
            validate = schema.validate(template)
        except Exception: return False
        return svProjectExtractor.retrieve_sros_default_tag(_file_=f'{self.PROJECT_DIR}data/policies.xml', template=template)

    @staticmethod
    def retrieve_sros_default_tag(_file_, template):
        default_tag = '<xi:include href="path/to/common/node.xml" xpointer="xpointer(/profile/*)"/>'
        default_    = ET.Element('{http://www.w3.org/2001/XInclude}include')
        default_.set('href', f'{SCHEMAS}sros/common/node.xml')
        default_.set('xpointer', 'xpointer(/profile/*)')
        profiles = template.findall(f'.//profile')
        for profile in profiles:
            profile.append(default_)
        with open(_file_, 'w+') as sros:
            ET.indent(template)
            ET.register_namespace('xi', 'http://www.w3.org/2001/XInclude')
            __xml__ = ET.tostring(template, encoding='unicode')
            sros.write(__xml__)
        return True
        
    # Extract from config file
    def extract_config(self, config_file=''):
        if not config_file:
            config_file = f'{self.PROJECT_DIR}config.yml'
        config = safe_load(stream=open(config_file, 'r'))
        self.config, packages, nodes, observable_determinism = config, list(set(config.get('packages'))), config.get('nodes'), config.get('configurations', {}).get('analysis', {}).get('observable determinism')
        # LOAD PICKLE.
        if not self.IMPORTED_DATA == {}: Node.NODES = self.IMPORTED_DATA['nodes']
        for package in packages: 
            Package.init_package_name(name=package, index=packages.index(package))
        if not (nodes and observable_determinism):
            raise svException(f'Failed to import config file of project.')
        if not self.load_nodes_profiles(nodes=nodes, observable_determinism=observable_determinism):
            raise svException(f'Failed to import config file of project.')
        return True

    def load_nodes_profiles(self, nodes, observable_determinism):
        if svROSEnclave.ENCLAVES is {}:
            raise svException("No enclaves found, security in ROS is yet to be defined.")
        # Processing nodes.
        for node in nodes:
            name, node = node, nodes[node]
            unsecured, enclave = False, None
            if node.get('enclave') == '' or node.get('enclave') == 'None': unsecured = True
            elif node.get('enclave') not in svROSEnclave.ENCLAVES:
                raise svException(f"{node.get('rosname')} enclave is not defined.")
            else:
                enclave = svROSEnclave.ENCLAVES[node.get('enclave')]
            if not unsecured:
                rosname = node.get('rosname')
                profile = enclave.profiles.get(rosname)
                if not profile: raise svException(f"{node.get('rosname')} profile is not defined.")
                node         = svROSNode(full_name=name, profile=profile, **node)
                # Update NODE and PROFILE.
                profile.node                   = node
                node.advertise, node.subscribe = node.constrain_topics()
            else: 
                node['enclave'] = None
                node            = svROSNode(full_name=name, profile=None, **node)
        for od in observable_determinism:
            # PARSING OBSERVABLE DETERMINISM.
            if not bool(re.match(pattern=r'(.*?)=>(.*?)', string=od)): raise svException(f"Failed to parse Observable Determinism rule.")
            else: 
                pattern = re.match(pattern=r'(.*?)=>(.*?)$', string=od)
                un_node, od_output = pattern.groups()[0].strip(), pattern.groups()[1].strip()
            # OD involving nodes.
            un_node   = un_node[1:] if un_node.startswith('/') else un_node
            if un_node not in svROSNode.NODES: 
                raise svException(f"Unsecured node {un_node} is not defined.")
            if not od_output.startswith('$'):
                od_output = od_output[1:] if od_output.startswith('/') else od_output 
                if od_output not in svROSNode.NODES:
                    raise svException(f"Observable node {od_output} is not defined.")
                od_output = svROSNode.NODES[od_output]
                # Is it observable ?
                if od_output.secure: raise svException(f"Node {od_output.rosname} is not observable, as it is secured.")
            else:
                if od_output[1:] not in svState.STATES:
                    raise svException(f"Observable state {od_output} is not defined.")
                od_output = svState.STATES[od_output[1:]]

            node = svROSNode.NODES[un_node]
            # Connections and NODE => OBSERVABLE
            if node.secure: print(svWarning(f'Node is SROS secured, but its identified as an outsider to the determinism of the program.'))
            if node not in svROSNode.OBSDT: 
                svROSNode.OBSDT[node] = set()
            svROSNode.OBSDT[node].add(od_output)
        # HANDLE class methods.
        svROSNode.handle_connections()  # Set connections up.
        svROSNode.observalDeterminism(scopes=self.scopes) # Observable determinism in Unsecured Nodes.
        return True

    @property
    def scopes(self):
        scopes = self.config.get('configurations', {}).get('analysis', {}).get('scope')
        messages, steps = scopes.get('Message'), scopes.get('Steps') 
        if not (messages and steps):
            raise svException('Failed to retrieve scopes. Either Message or Steps scopes are not defined.')
        return scopes