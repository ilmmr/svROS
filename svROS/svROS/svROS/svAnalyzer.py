import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, xmlschema, pickle, enquiries, json
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException, svWarning, svInfo
from lark import Lark, tree
# Node parser
from svData import svROSNode, svROSProfile, svROSEnclave, svROSObject, svState, Node, Package, Topic, MessageType, svExecution
from svLanguage import svPredicate
import xml.etree.ElementTree as ET

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

""" 
    This file contains the necessary classes and methods to translate from Python Structures into Alloy configuration model.
    Additionally some methods were implemented to approprietally retrieve text-based structures into Alloy.
"""
"Main class that implements the analyzing process while accounting the configuration."
@dataclass
class svAnalyzer(object):
    EXTRACTOR     : object
    MODELS_DIR    : str
    
    def __post_init__(self):
        # GET FROM EXTRACTOR
        project, PROJECT_DIR, scopes = self.EXTRACTOR.project, self.EXTRACTOR.PROJECT_DIR, self.EXTRACTOR.scopes
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
        NODES, TOPICS = svROSNode.NODES, Topic.TOPICS
        ROS_FILE = self.generate_ros_model(NODES=NODES, TOPICS=TOPICS)
        if not os.path.isfile(path=ROS_FILE): return False
        else: return True

    def alloy_ros(self):
        counter, file_path = False, f'{self.EXTRACTOR.PROJECT_DIR}models/ros-concrete.als'
        if not os.path.isfile(path=file_path): return False
        # CHECK PROPERTIES if it holds counter-examples
        properties = re.findall(r'check\s+(.*?)\s+\{', open(file_path, 'r').read())
        properties = list(map(lambda check: check.strip(), properties))
        if counter:
            print(svInfo(f'{color.color("BOLD", "Alloy-ROS")} → Not every property seem to hold for the given configuration: It is advisable to run with increased configuration scopes.'))
        else:
            print(svInfo(f'{color.color("BOLD", "Alloy-ROS")} → Every property seem to hold for the given configuration: It is advisable to run with increased configuration scopes.'))
        for prop in properties:
            if prop in counter:
                print(f'\n\t‣‣ {prop.split("2")[0]} =Observable Determinism=> {prop.split("2")[1]} {color.color("RED", "☒")}')
            else: 
                print(f'\n\t‣‣ {prop.split("2")[0]} =Observable Determinism=> {prop.split("2")[1]} {color.color("GREEN", "✅")}')
        # RUN VISUALIZER
        visualizer = input(svWarning('Open counter-examples [Y/n]: ')).strip()
        if visualizer == r"(?i)y": 
            # open visualizer
            choice = enquiries.choose('', choices=list(map(lambda option: f'{color.color("BLUE", option)}', counter)) + ['Exit'])
            if choice.strip() == r'(?i)Exit':
                return True
            else:
                visualizer_path, file = f'{self.EXTRACTOR.PROJECT_DIR}data/visualizer/', f'{choice}.html'
                os.system(command=f'firefox {visualizer_path}{file}')
        else: pass
        return True

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
        model +=  '/* === TOPICS === */\n\n'
        # SELF-COMPOSITION.
        model += svExecution.create_executions()
        model += '\n/* === NODE BEHAVIOUR === */\n'
        model += svPredicate.node_behaviour()
        model += '\n/* === NODE BEHAVIOUR === */\n\n/* === OBSERVABLE DETERMINISM === */\n'
        model += svROSNode.observable_determinism()
        model += '\n/* === OBSERVABLE DETERMINISM === */'
        # with open(file_path, 'w+') as ros: ros.write(model)
        return file_path
    
    # ALLOY => Runs Structure Checking in SROS_MODEL
    def security_verification(self):
        # NODES = dict(filter(lambda node: node[1].profile is not None, svROSNode.NODES.items())) ==> NOT NEEDED. 
        ENCLAVES, OBJECTS, PROFILES = svROSEnclave.ENCLAVES, svROSObject.OBJECTS, svROSProfile.PROFILES
        SROS_FILE = self.generate_sros_model(PROFILES=PROFILES, ENCLAVES=ENCLAVES, OBJECTS=OBJECTS)
        if not os.path.isfile(path=SROS_FILE): return False
        else: return True
    
    def alloy_sros(self):
        counter, file_path = 0, f'{self.EXTRACTOR.PROJECT_DIR}models/sros-concrete.als'
        if not os.path.isfile(path=file_path): return False
        properties = ['valid_configuration_1', 'valid_configuration_2', 'valid_configuration']
        counter = svAnalyzer.execute_java(properties=properties, file=file_path)
        if counter == []:
            print(svInfo(f'{color.color("BOLD", "Alloy-SROS")} → Every property seem to hold for the given configuration:\n\t‣‣ No profile has different privileges of access (ALLOW, DENY) to the same object {color.color("GREEN", "✅")} \n\t‣‣ Every profile corresponding node object call is within its privileges {color.color("GREEN", "✅")}'))
        else:
            print(svInfo(f'{color.color("BOLD", "Alloy-SROS")} → Not every property seem to hold for the given configuration.'))
            if 'valid_configuration_1' in counter:
                print(f'\t‣‣ No profile has different privileges of access (ALLOW, DENY) to the same object {color.color("RED", "☒")}\n\t‣‣ Every profile corresponding node object call is within its privileges {color.color("GREEN", "✅")}')
            elif 'valid_configuration_2' in counter:
                print(f'\t‣‣ No profile has different privileges of access (ALLOW, DENY) to the same object {color.color("GREEN", "✅")}\n\t‣‣ Every profile corresponding node object call is within its privileges {color.color("RED", "☒")}')
        return True

    @staticmethod
    def execute_java(properties, file):
        models_path = f'/usr/generated_models'
        returning   = []
        for prop in properties:
            javacmd = "java -jar generator/out/artifacts/generator_jar/generator.jar " + file + " " + prop
            os.system(javacmd)
            if os.path.isfile(path=f'{models_path}/{prop}.xml'):
                returning.append(prop)
        return returning

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
        # with open(file_path, 'w+') as sros: sros.write(model)
        return file_path

"Main exporter parser from current project's directory files: SROS and configuration file"
@dataclass
class svProjectExtractor:
    project       : str
    PROJECT_DIR   : str
    IMPORTED_DATA : dict

    def draw_architecture(self):
        return True

    def save_imported_data(self):
        try:
            svROSNode.NODES, svState.STATES, Topic.TOPICS, svPredicate.NODE_BEHAVIOURS, svROSEnclave.ENCLAVES = self.IMPORTED_DATA.get('nodes', {}), self.IMPORTED_DATA.get('states', {}), self.IMPORTED_DATA.get('topics', {}), self.IMPORTED_DATA.get('predicates', {}), self.IMPORTED_DATA.get('enclaves', {})
        except AttributeError as e:
            return False
        return True

    # Before Analyzing...
    def update_imported_data(self):
        DATADIR = f'{self.PROJECT_DIR}/data'
        # SAVE using PICKLE.
        package_file, topic_file, node_file = open(f'{DATADIR}Packages.obj', 'wb+'), open(f'{DATADIR}Channels.obj', 'wb+'), open(f'{DATADIR}Nodes.obj', 'wb+')
        state, predicates, enclaves = open(f'{DATADIR}States.obj', 'wb+'), open(f'{DATADIR}Predicates.obj', 'wb+'), open(f'{DATADIR}Enclaves.obj', 'wb+')
        pickle.dump(Package.PACKAGES, package_file, pickle.HIGHEST_PROTOCOL)
        pickle.dump(Topic.TOPICS, topic_file, pickle.HIGHEST_PROTOCOL)
        pickle.dump(svROSNode.NODES, node_file, pickle.HIGHEST_PROTOCOL)
        pickle.dump(svState.STATES, state, pickle.HIGHEST_PROTOCOL)
        pickle.dump(svPredicate.NODE_BEHAVIOURS, predicates, pickle.HIGHEST_PROTOCOL)
        pickle.dump(svROSEnclave.ENCLAVES, enclaves, pickle.HIGHEST_PROTOCOL)
        # json files.
        data_json = {'packages': list(set(map(lambda package: package.name.lower(), Package.PACKAGES))), 'nodes': dict(map(lambda node: (node.replace('::', '/'), svROSNode.to_json(node)) , svROSNode.NODES)), 'states': list(set(map(lambda state: state.name.lower(), svState.STATES.values()))), 'predicates': dict(map(lambda predicate: (predicate.signature.lower(), predicate.node.rosname), svPredicate.NODE_BEHAVIOURS.values()))}
        with open(f'{DATADIR}/configurations.json', 'w+') as data:
            json.dump(data_json, data, sort_keys=False, indent=4)
        enclaves_json = list(map(lambda enclave: enclave.to_json(), svROSEnclave.ENCLAVES.values()))
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
        self.config, packages, nodes = config, list(set(config.get('packages'))), config.get('architecture')
        # ANALYSIS.
        observable_determinism, types, states, analysing_nodes = config.get('analysis', {}).get('observable determinism'), config.get('analysis', {}).get('information flow').get('types'), config.get('analysis', {}).get('information flow').get('states'), config.get('analysis', {}).get('information flow').get('nodes') 
        # LOAD PICKLE.
        if not self.IMPORTED_DATA == {}: Node.NODES = self.IMPORTED_DATA['nodes']
        for package in packages: 
            Package.init_package_name(name=package, index=packages.index(package))
        if not (nodes and observable_determinism):
            raise svException(f'Failed to import config file of project.')
        if not self.load_nodes_profiles(nodes=nodes, observable_determinism=observable_determinism, states=states):
            raise svException(f'Failed to import config file of project.')
        if not self.load_analysis(nodes=analysing_nodes, types=types):
            raise svException(f'Failed to import config file of project.')
        return True

    def load_nodes_profiles(self, nodes, observable_determinism, states):
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
        if states:
            for state in states: svState.init_state(name=state, values=states[state])
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
                if od_output.private: raise svException(f"State {od_output.name} is not observable, as it is private.")
            # Parsing node...
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

    def load_analysis(self, nodes, types):
        if svROSNode.NODES is {}:
            raise svException("No nodes found, loading is yet to be launched.")
        for t in types:
            tt, mtype_temp = t, types[t].split('/')
            pattern    = re.match(pattern=r'(.*?) (.*?)$', string=str(t))
            if not bool(pattern): isint = False
            else:
                if not pattern.groups()[0].strip() == 'int': raise svException(f'Failed to parse type {str(t)}.')
                t, isint = pattern.groups()[1].strip(), True
            # ...
            t = t.replace('/', '_').lower()
            if t not in MessageType.TYPES: raise svException(f"Message Type {tt} not found.")
            mtype       = MessageType.TYPES[t]
            # SET isint.
            mtype.isint = isint
            mtype.value.values = mtype_temp    
        # LOAD PROPERTIES.
        for n in nodes:
            # PARSING.
            grammar = """
            sentence: one | two
            one: NAME "as" NAME
            two: NAME
            NAME:/(?!\s)[a-zA-Z0-9_\/\-.\:]+/
            %import common.WS
            %ignore WS
            """
            parser, key = Lark(grammar, start='sentence', ambiguity='explicit'), n
            if not parser.parse(n): raise svException(f'Failed to parse node behaviour {n}.')
            t      = parser.parse(n)
            if t.children[0].data == "one":
                n, signature = n.split('as')[0].strip(), n.split('as')[1].strip()
            else:
                signature = n
            # ...
            if n not in svROSNode.NODES: raise svException(f"Node {n} failed to be found.")
            node            = svROSNode.NODES[n]
            predicate       = svPredicate.init_predicate(signature=signature, node=node, properties=nodes[key])
            # Node predicate association.
            if node.predicate is not None:
                raise svException(f"Node {node.rosname} has two different predicates mentioned. Please remove 1!")
            node.predicate  = predicate
        return True

    @property
    def scopes(self):
        scopes = self.config.get('analysis', {}).get('scope', {})
        messages, steps = scopes.get('Message'), scopes.get('Steps') 
        if not (messages and steps):
            raise svException('Failed to retrieve scopes. Either Message or Steps scopes are not defined.')
        return scopes