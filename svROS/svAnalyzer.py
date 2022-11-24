import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, xmlschema, json
from simple_term_menu import TerminalMenu
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from .svInfo import color, svException, svWarning, svInfo
from lark import Lark, tree
# Node parser
from .svData import svNode, svProfile, svEnclave, svTopic, svState, Node, Package, MessageType, svExecution
from .svLanguage import svPredicate
import xml.etree.ElementTree as ET
# Visualizer
from .svVisualizer import svVisualizer
from .svInitGrammar import GrammarParser

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, 'schemas')

""" 
    This file contains the necessary classes and methods to translate from Python Structures into Alloy configuration model.
    Additionally some methods were implemented to approprietally retrieve text-based structures into Alloy.
"""
"Main class that implements the analyzing process while accounting the configuration."
@dataclass
class svAnalyzer(object):
    EXTRACTOR     : object
    MODELS_DIR    : str
    MODE          : int = 0
    
    def __post_init__(self):
        # GET FROM EXTRACTOR
        project, PROJECT_DIR = self.EXTRACTOR.project, self.EXTRACTOR.PROJECT_DIR
        if self.MODE == 0:
            scopes = self.EXTRACTOR.scopes
            self.meta_model, self.sros_model = self.load_configuration(MODELS_DIR=self.MODELS_DIR, PROJECT_DIR=PROJECT_DIR, name=project.lower())
        
    @staticmethod
    def run_dir():
        return os.getcwd() 
	
    @staticmethod
    def load_configuration(MODELS_DIR, PROJECT_DIR, name):
        module_name, sros_module_name = "module " + str(name) + " /* === PROJECT " + str(name).upper() + " ===*/\n\n", "module sros-" + str(name) + " /* === PROJECT " + str(name).upper() + " ===*/\n\n"
        # ROS META-MODEL
        with open(f'{MODELS_DIR}/ros_base.als') as f:
            ros_meta_model  = module_name
            ros_meta_model += f.read()
        # SROS META-MODEL
        with open(f'{MODELS_DIR}/sros_base.als') as f:
            sros_meta_model  = sros_module_name
            sros_meta_model += f.read()
        return ros_meta_model, sros_meta_model

    # ALLOY => Runs Model Checking in ROS_MODEL
    def ros_verification(self):
        NODES, TOPICS = svNode.NODES, svTopic.TOPICS
        ROS_FILE = self.generate_ros_model(NODES=NODES, TOPICS=TOPICS)
        if not os.path.isfile(path=ROS_FILE): return False
        else: return True

    def alloy_ros(self):
        counter, file_path = list(), f'{self.EXTRACTOR.PROJECT_DIR}models/ros-concrete.als'
        if not os.path.isfile(path=file_path): return False
        # CHECK PROPERTIES if it holds counter-examples
        properties = re.findall(r'check\s+(.*?)\s+\{', open(file_path, 'r').read())
        properties = list(map(lambda check: check.strip(), properties))
        # EXECUTE JAVA
        counter    = svAnalyzer.execute_java(properties=properties, file=file_path, type="ros")
        if counter == []:
            print(svInfo(f'{color.color("GREEN", color.color("BOLD", "VERIFICATION MODEL"))} Every observation seem to hold for the given configuration → It is advisable to run with increased configuration scopes...'))
            return True
        print(svInfo(f'{color.color("RED", color.color("BOLD", "VERIFICATION MODEL"))}: Not every observation seem to hold for the given configuration...'))
        map_dict = {}
        for prop in properties:
            if prop in list(map(lambda st: st.split('.xml')[0], counter)):
                print(f'\t‣‣ OBSERVATION IN TOPIC {color.color("UNDERLINE", prop.split("topic_")[1].replace("_","/").upper())} IS NOT PUBLICLY DETERMINISTIC!')
                map_dict[prop.split("topic_")[1].replace("_","/")] = f'{prop}.xml'
        # RUN VISUALIZER
        while True:
            # open visualizer
            options = list(map(lambda option: option, map_dict.keys())) + ['Exit']
            choice = TerminalMenu(options).show()
            if options[choice] == 'Exit':
                break
            else:
                viz_directory, file = f'{self.EXTRACTOR.PROJECT_DIR}data/viz', f'/tmp/generated_models/ros/{map_dict[options[choice]]}'
                viz = svVisualizer(project=self.EXTRACTOR, directory=viz_directory)
                viz.run_file(type='OD', file=file)
                print(svInfo(f'Counterexample is being displayed on your browser'), end='')
                cont = input('...').strip()
        return True

    def generate_ros_model(self, NODES, TOPICS):
        model, file_path = self.meta_model, f'{self.EXTRACTOR.PROJECT_DIR}models/ros-concrete.als'
        if not os.path.exists(path=file_path): open(file_path, 'w+').close()
        if not os.path.isfile(path=file_path): raise svException('Unexpected error happend while creating ROS file.')
        # NODES.
        model += '\n/* === NODES === */\n'
        model += ''.join(list(map(lambda node: str(NODES[node]), NODES)))
        # TOPICS.
        model += '/* === NODES === */\n\n/* === TOPICS === */\n'
        model += svTopic.ros_declaration()
        model +=  '/* === TOPICS === */\n\n'
        # SELF-COMPOSITION.
        model += svExecution.create_executions()
        model += '\n/* === NODE BEHAVIOUR === */\n'
        model += svPredicate.node_behaviour()
        model += '\n/* === NODE BEHAVIOUR === */\n\n/* === OBSERVATIONAL DETERMINISM === */\n'
        model += svNode.observable_determinism(assumptions=self.EXTRACTOR.assumptions)
        model += '\n/* === OBSERVATIONAL DETERMINISM === */'
        with open(file_path, 'w+') as ros: ros.write(model)
        return file_path
    
    # ALLOY => Runs Structure Checking in SROS_MODEL
    def security_verification(self):
        ENCLAVES, OBJECTS, PROFILES = svEnclave.ENCLAVES, svTopic.TOPICS, svProfile.PROFILES
        SROS_FILE = self.generate_sros_model(PROFILES=PROFILES, ENCLAVES=ENCLAVES, OBJECTS=OBJECTS)
        if not os.path.isfile(path=SROS_FILE): return False
        else: return True
    
    def alloy_sros(self):
        counter, file_path = list(), f'{self.EXTRACTOR.PROJECT_DIR}models/sros-concrete.als'
        if not os.path.isfile(path=file_path): return False
        properties = ['valid_configuration']
        # EXECUTE JAVA
        counter    = svAnalyzer.execute_java(properties=properties, file=file_path, type="sros")
        if counter == []:
            print(svInfo(f'{color.color("BOLD", "Alloy-SROS")} → Every property seem to hold for the given configuration:\n\t‣‣ No profile has different privileges of access (ALLOW, DENY) to the same object {color.color("GREEN", "✅")}'))
        else:
            print(svInfo(f'{color.color("BOLD", "Alloy-SROS")} → Failed to verify SROS configuration.'))
            # RUN VISUALIZER
            options = ['View SROS Counterexample', 'Exit']
            choice = TerminalMenu(options).show()
            if choice == 1:
                return True
            else:
                viz_directory, file = f'{self.EXTRACTOR.PROJECT_DIR}data/viz', f'/tmp/generated_models/sros/valid_configuration.xml'
                viz = svVisualizer(project=self.EXTRACTOR, directory=viz_directory)
                return viz.run_file(type='SROS', file=file)    
        return True

    @staticmethod
    def execute_java(file, properties, type):
        models_path = f'/tmp/generated_models/{type}' 
        # clear directory
        files = glob.glob(f'{models_path}/*')
        for f in files:
            os.remove(f)  
        # execute 
        for prop in properties:
            javacmd = "java -jar ~/.svROS/.bin/generator.jar " + file + " " + type + " " + prop
            os.system(javacmd)
        return os.listdir(models_path)
        
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
        model += ''.join(list(map(lambda obj: OBJECTS[obj].sros_declaration(), OBJECTS)))
        model += '/* === OBJECTS === */'
        with open(file_path, 'w+') as sros: sros.write(model)
        return file_path

"Main exporter parser from current project's directory files: SROS and configuration file"
@dataclass
class svProjectExtractor:
    project       : str
    PROJECT_DIR   : str

    # Draw Architecture
    def draw_architecture(self):
        viz_directory = f'{self.PROJECT_DIR}data/viz'
        viz = svVisualizer(project=self, directory=viz_directory)
        return viz.run_file(type='ARCHITECTURE')

    # Before Analyzing...
    def update_imported_data(self):
        DATADIR = f'{self.PROJECT_DIR}/data'
        # json files.
        data_json = {'packages': list(set(map(lambda package: package.name.lower(), Package.PACKAGES))), 'nodes': list(map(lambda node: svNode.to_json(node) , svNode.NODES)), 'connections': svNode.connections_to_json(), 'states': list(set(map(lambda state: state.name.lower(), svState.STATES.values()))), 'predicates': dict(map(lambda predicate: (predicate.signature.lower(), predicate.node.rosname), svPredicate.NODE_BEHAVIOURS.values()))}
        with open(f'{DATADIR}/configurations.json', 'w+') as data:
            json.dump(data_json, data, sort_keys=False, indent=4)
        enclaves_json = list(map(lambda enclave: enclave.to_json(), svEnclave.ENCLAVES.values()))
        json_object = json.dumps(enclaves_json, indent=4)
        with open(f'{DATADIR}/enclaves.json', 'w+') as data:
            data.write(json_object)
        return True

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
            enclave        = svEnclave(path=path, profiles=profiles)
        return True

    def validate_sros(self, sros):
        if not os.path.isfile(path=sros):
            raise svException(f'Policies file not found! Please define a {color.color("BOLD", "policies.xml")} file under project {self.project.capitalize()}\'s directory.')
        sch      = f'{SCHEMAS}/sros/sros.xsd'
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
        default_.set('href', f'{SCHEMAS}/sros/common/node.xml')
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
        self.config, packages, nodes = config, list(set(config.get('packages'))), config.get('nodes')
        # ANALYSIS.
        states = config.get('variables', {})
        for package in packages: 
            Package.init_package_name(name=package, index=packages.index(package))
        if not nodes:
            raise svException(f'Failed to import config file of project.')
        if not self.load_nodes_profiles(nodes=nodes, states=states):
            return False
        return True

    def load_nodes_profiles(self, nodes, states):
        if svEnclave.ENCLAVES is {}:
            raise svException("No enclaves found, security in ROS is yet to be defined.")
        # Variables
        if states:
            for state in states: svState.init_state(name=state)
        # Processing nodes.
        for node in nodes:
            name, node = node, nodes[node]            
            unsecured, enclave = False, None
            if node.get('enclave') not in svEnclave.ENCLAVES:
                raise svException(f"{node.get('rosname')} enclave is not defined. Enclaves available: {list(map(lambda enclave: enclave.name, svEnclave.ENCLAVES.values()))}.")
            else:
                enclave = svEnclave.ENCLAVES[node.get('enclave')]
            rosname = node.get('rosname')
            profile = enclave.profiles.get(rosname)
            if not profile: raise svException(f"{node.get('rosname')} profile is not defined in enclave {enclave.name}.")
            # PROPERTIES...
            behaviour = list(filter(lambda key: key.startswith('behaviour'), node.keys()))[0]
            if behaviour is None: raise svException(f'Must defined some node behaviour to {name}!')
            properties = node.get(behaviour)
            # Update NODE and PROFILE.
            node         = svNode(full_name=name, profile=profile, **node)
            profile.node                   = node
            # PROPERTIES...
            if not self.node_behaviour(node=node, behaviour=behaviour, properties=properties):
                raise svException(f'Failed to properties from {node.rosname}.')
        svNode.handle_connections()  # Set connections up.
        steps, inbox = self.scopes
        svPredicate.parse_into_alloy()
        #if type.lower() == "od" or type.lower() == "observable determinism":
        # Observable determinism in Unsecured Nodes.
        if not svNode.observalDeterminism(steps=steps, inbox=inbox): 
            return False
        return True

    def node_behaviour(self, node, behaviour, properties): 
        # LOAD PROPERTIES ==> PARSING...
        grammar = """
        sentence: one | two
        one: "behaviour" "as" NAME
        two: "behaviour"
        NAME:/(?!\s)[a-zA-Z0-9_\/\-.\:]+/
        %import common.WS
        %ignore WS
        """
        parser, key = Lark(grammar, start='sentence', ambiguity='explicit'), behaviour
        if not parser.parse(behaviour): raise svException(f'Failed to parse node behaviour {node.rosname}.')
        t      = parser.parse(behaviour)
        if t.children[0].data == "one":
            signature = behaviour.split('as')[1].strip()
        else:
            signature = node.rosname[1:]
        # ...
        predicate       = svPredicate.init_predicate(signature=signature, node=node, properties=properties)
        # Node predicate association.
        if node.predicate is not None:
            raise svException(f"Node {node.rosname} has two different predicates mentioned. Please remove 1!")
        node.predicate  = predicate
        return True

    @property
    def scopes(self):
        steps = self.config.get('configurations').get('model', {}).get('steps', {})
        inbox = self.config.get('configurations').get('model', {}).get('inbox', {})
        if not inbox:
            raise svException('Failed to retrieve scopes on inbox: Define type in configurations/model area.')
        if not steps:
            raise svException('Failed to retrieve scopes on steps: Define type in configurations/model area.')
        return steps, inbox

    @property
    def assumptions(self):
        behaviour   = self.config.get('configurations').get('model', {}).get('behaviour', {})
        assumptions = list(map(lambda prop: self.create_prop(text=prop), behaviour))
        if assumptions == []: return None
        return assumptions

    def create_prop(self, text):
        try: 
            if isinstance(text, str):
                property = GrammarParser.parse(text=text)
                return property
            else:
                raise svException(f'Failed to parse property {text}.')
        except Exception: raise svException(f'Failed to parse property {text}.')