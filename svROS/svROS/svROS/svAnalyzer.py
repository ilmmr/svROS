import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
from typing import ClassVar

global WORKDIR
WORKDIR = os.path.dirname(__file__)

""" 
    This file contains the necessary classes and methods to translate from Python Structures into Alloy configuration model.
"""
"Main class that implements the analyzing process while accounting the configuration."
@dataclass
class Analyzer(object):
    project       : str
    MODELS_DIR    : str
    PROJECT_DIR   : str
    nodes         : dict = field(default_factory=dict)
    properties    : dict = None
    configuration : Configuration = None
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

    # ALLOY => Runs Model Checking in ROS_MODEL.
    def model_check(self):
    	pass
    
    # ALLOY => Runs Structure Checking in SROS_MODEL.
    def security_verification(self):
        pass