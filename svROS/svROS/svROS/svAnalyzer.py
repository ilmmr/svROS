import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, xmlschema
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException, svInfo
# Node parser
from svData import svROSNode, svROSProfile, svROSEnclave
import xml.etree.ElementTree as ET

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

""" 
    This file contains the necessary classes and methods to translate from Python Structures into Alloy configuration model.
"""
"Main class that implements the analyzing process while accounting the configuration."
@dataclass
class svAnalyzer(object):
    project       : str
    MODELS_DIR    : str
    PROJECT_DIR   : str
    nodes         : dict = field(default_factory=dict)
    properties    : dict = None
    configuration : str  = None
    specification : str           = ''

    def __post_init__(self):
        module_name = "module " + str(project) + "\n/* === PROJECT " + str(project).upper() + " ===*/"
        meta_model, sros_model, scopes = self.load_configuration(MODELS_DIR=self.MODELS_DIR, PROJECT_DIR=self.PROJECT_DIR, name=self.project.lower())
        # self.configuration = Configuration(c_name, self.nodes, self.topics, scopes, properties=self.properties)
        # self.specification = (self.module_name + meta_model + self.configuration.specification())
	
    @staticmethod
    def run_dir():
        return os.getcwd() 
	
    @staticmethod
    def load_configuration(MODELS_DIR, PROJECT_DIR, name):
        CONFIG_FILE = f'{PROJECT_DIR}/{name}.yaml'
        if not os.path.isfile(CONFIG_FILE):
                raise Exception
        with open(CONFIG_FILE) as f:
            data   = f.read()
            data   = yaml.load(data)
            # LOAD from configuration file.
            scopes     = data['configurations']['scope']
            self.nodes = data['nodes']
        # ROS META-MODEL
        with open(f'{MODELS_DIR}/ros_base.als') as f:
            ros_meta_model = f.read()
        # SROS META-MODEL
        with open(f'{MODELS_DIR}/sros_base.als') as f:
            sros_meta_model = f.read()
        return ros_meta_model, sros_meta_model, scopes

    # ALLOY => Runs Model Checking in ROS_MODEL
    def model_check(self):
    	pass
    
    # ALLOY => Runs Structure Checking in SROS_MODEL
    def security_verification(self):
        pass

"Main exporter parser from current project's directory files: SROS and configuration file"
@dataclass
class svProjectExtractor:
    project       : str
    MODELS_DIR    : str
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
        enclaves = root.findall('.//enclave')
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
        nodes, unsecured_enclaves = config.get('nodes'), config.get('configurations', {}).get('analysis', {}).get('unsecured_enclaves')
        if not (nodes and unsecured_enclaves):
            raise svException(f'Failed to import config file of project.')
        if not self.load_nodes_profiles(nodes=nodes, unsecured_enclaves=unsecured_enclaves):
            raise svException(f'Failed to import config file of project.')

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
                node         = svROSNode(profile=profile, **node)
                profile.node = node
            else: node       = svROSNode(**node, profile=None)
        return True