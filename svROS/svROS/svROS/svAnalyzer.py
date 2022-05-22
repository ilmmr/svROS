import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, xmlschema
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException, svInfo
# Node parser
from svData import svROSNode, svROSProfile, svROSEnclave, Package
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

@dataclass
class svSROSAlloy(object):
    """
        - GET ENCLAVES
        - GET NODES that have associated profiles
    """
    PROJECT_DIR : str
    SROS_MODEL  : str
    NODES       : dict
    ENCLAVES    : dict

    def generate(self):
        model, file_path = self.SROS_MODEL, f'{self.PROJECT_DIR}/models/sros-concrete.als'
        if not os.path.isfile(path=file_path): svException('Unexpected error happend while creating SROS file.')
        model += ' '.join(list(map(lambda enclave: str(self.ENCLAVES[enclave]), self.ENCLAVES)))
        model += ' '.join(list(map(lambda node: str(self.NODES[node].profile), self.NODES)))
        with open(file_path, 'w+') as sros: sros.write(model)
        return file_path

"Main class that implements the analyzing process while accounting the configuration."
@dataclass
class svAnalyzer(object):
    EXTRACTOR     : svProjectExtractor
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
    def model_check(self):
    	pass
    
    # ALLOY => Runs Structure Checking in SROS_MODEL
    def security_verification(self):
        NODES, ENCLAVES = dict(filter(lambda node: node[1].profile is not None, svROSNode.NODES.items())), svROSEnclave.ENCLAVES
        parser    = svSROSAlloy(PROJECT_DIR=self.EXTRACTOR.PROJECT_DIR, SROS_MODEL=self.sros_model, NODES=NODES, ENCLAVES=ENCLAVES)
        SROS_FILE = parser.generate()

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
        self.config, packages, nodes, unsecured_enclaves = config, list(set(config.get('packages'))), config.get('nodes'), config.get('configurations', {}).get('analysis', {}).get('unsecured_enclaves')
        # LOAD PICKLE.
        if not self.IMPORTED_DATA == {}: svROSNode.LOADED_NODES = self.IMPORTED_DATA['nodes']
        for package in packages: 
            Package.init_package_name(name=package, index=packages.index(package))
        if not (nodes and unsecured_enclaves):
            raise svException(f'Failed to import config file of project.')
        if not self.load_nodes_profiles(nodes=nodes, unsecured_enclaves=unsecured_enclaves):
            raise svException(f'Failed to import config file of project.')
        return True

    def load_nodes_profiles(self, nodes, unsecured_enclaves):
        if svROSEnclave.ENCLAVES is {}:
            raise svException("No enclaves found, security in ROS is yet to be defined.")
        for node in nodes:
            name, node = node, nodes[node]
            unsecured, enclave = False, None
            if node.get('enclave') == '' or node.get('enclave') in unsecured_enclaves: unsecured = True
            elif node.get('enclave') not in svROSEnclave.ENCLAVES:
                raise svException(f"{node.get('rosname')} enclave is not defined.")
            else:
                enclave = svROSEnclave.ENCLAVES[node.get('enclave')]
            if not unsecured:
                rosname = node.get('rosname')
                profile = enclave.profiles.get(rosname)
                if not profile: raise svException(f"{node.get('rosname')} profile is not defined.")
                node         = svROSNode(full_name=name, profile=profile, **node)
                profile.node = node
            else: node       = svROSNode(full_name=name, profile=None, **node)
        return True

    @property
    def scopes(self):
        scopes = self.config.get('configurations', {}).get('analysis', {}).get('scopes')
        if not scopes:
            raise svException('Failed to retrieve scopes.')
        return scopes