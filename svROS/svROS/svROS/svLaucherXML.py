import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict

# Parsers
from lxml import etree
from lark import Lark, tree

# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svROS_Exception as excp, svROS_Info as info
from tools.Loader import Loader

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

""" 
    This file contains the necessary classes and methods to export information from the launch file XML-based specified within the config file.

    SCHEMA that ros2 provides is deprecated also... So we'll try to run ros2 launch -p instead:
        => ros2 launch $file -p :: 
"""

"Launcher with some configuration dicts..."
@dataclass
class LauncherConfigXML(object):

    booleans: dict = field(default_factory=dict) 
    nodes   : list = field(default_factory=list)
    args    : dict = field(default_factory=dict)

    """ LAUNCH TREE according to xsd...
            => node (can have if and unless)
                    \-> remap tag (can have arg/let values...)
            => arg  (can have if and unless)
            => let  (can have if and unless)
    """

    def __post_init__(self):
        # define arg node let as they must defined at the top.
        self.tree = {x: dict() for x in ['node', 'arg_let']}

    # define booleans that might found within args...
    def _booleans(self, key, t, value):
        self.booleans[key] = dict()
        self.booleans[key]['type'] = t
        self.booleans[key]['value'] = value
        return True

    # process args...
    def _process_args(self):
        # use a temp instead of the main one...
        args_temp = self.args
        for index in args_temp:
            element = args_temp[index]
            try:
                self.resolve_dependencies(args_temp, index)
                element.expand_conditions(args_temp)
                element._if_conditional(args_temp)
            except:
                return False
        # set it up...
        self.args = args_temp
        return True
        
    # process nodes and remaps...
    def _process_nodes(self):
        # use a temp instead of the main one...
        nodes_temp = self.nodes
        for node in args_temp:
                # process remaps
                node.process_remaps(self.args)
        
        # set it up...
        self.nodes = nodes_temp
        return True
        
    """ === Static Methods === """
    # recursive function to resolve some arg dependencies...
    @staticmethod
    def resolve_dependencies(args, value):
        if value == None:
                value = list(args.keys())[0]
        if args[value].dependent[0] == True:
                v = args[value].dependent[1]
                args[value].value = LauncherConfigXML.resolve_dependencies(args, v)
        
        # evalute boolValue
        args[value].evaluate()
        return args[value].value

    # custom filtering function
    @staticmethod
    def _filter(args: dict, tag: str):
        return list(map(lambda ob: args[ob], list(filter(lambda obj: obj[1] == tag, list(args.keys())))))
    
    # Using lark to parse some dependencies strings...
    @staticmethod
    def _arg_grammar(sentence='') -> (bool,str):
        # parsing grammar...
        grammar = """
            sentence: /\$/ LP ARG NAME RP

            LP: "("
            RP: ")"
            ARG: "var" | "env"
            NAME: /[a-zA-Z0-9_\/\-.]+/
            
            %import common.WS
            %ignore WS
        """
        # Larker parser
        parser = Lark(grammar, start='sentence', ambiguity='explicit')
        try:
                tree   = parser.parse(f'{sentence}')
        except:
                return '', ''
        # get token...
        ARG = list(filter(lambda t: t.type == "ARG" , tree.children))[0].value
        token = list(filter(lambda t: t.type == "NAME" , tree.children))[0].value
        # if is env
        if str(ARG) == 'env':
                return 'set_env', token

        return 'arg', token

    # Using lark to parse some dependencies strings...
    @staticmethod
    def parse_args(args=''):
        # returning dictionary
        output = {}
        output = {x: list() for x in ['remaps', 'parameters', 'enclaves']}
        # no args passed?
        if args == '':
            return False
            
        # Grammar to parse arguments...
        grammar = """
            sentence: INIT complete?
            complete: REMAP /\s/ arg_remap
                    | ENCLAVE /\s/ arg_enclave
                    | PARAMETER /\s/ arg_parameter

            arg_remap: ARG_R ":=" ARG_R (/\s/ (arg_remap | complete))*
            arg_enclave: ARG_E (/\s/ complete)*
            arg_parameter: ARG_P ":=" ARG_P (/\s/ (arg_parameter | complete))*

            INIT: "--ros-args"
            REMAP:"--remap" | "-r"
            ENCLAVE: "--enclave" | "-e"
            PARAMETER: "--parameter" | "-p"

            ARG_R:/(?!\:\=)[a-zA-Z0-9_\/\-.]+/
            ARG_E:/(?!\s)[a-zA-Z0-9_\/\-.]+/
            ARG_P:/(?!\:\=)[a-zA-Z0-9_\/\-.]+/

            %import common.WS
            %ignore WS
        """
        # Larker parser
        parser = Lark(grammar, start='sentence', ambiguity='explicit')
        tree = parser.parse(f'{args}')

        # find parameters
        p_list = list(tree.find_data("arg_parameter"))
        if p_list:
            parameters = p_list[len(p_list)-1]
            if parameters:
                # get tokens
                p_tokens = list(map(lambda v: v.value , list(filter(lambda vv: vv.type == "ARG_P" ,(parameters.scan_values(lambda v: isinstance(v, Token)))))))
                zipped = list(zip(p_tokens[0::2], p_tokens[1::2]))
                for pair in zipped:
                    output['parameters'].append({'from': str(pair[0]), 'to': str(pair[1])})

        # find remaps
        r_list = list(tree.find_data("arg_remap"))
        if r_list:
            remaps = r_list[len(p_list)-1]
            if remaps:
                # get tokens
                r_tokens = list(map(lambda v: v.value , list(filter(lambda vv: vv.type == 'ARG_R', (remaps.scan_values(lambda v: isinstance(v, Token)))))))
                zipped = list(zip(r_tokens[0::2], r_tokens[1::2]))
                for pair in zipped:
                    output['remaps'].append({'from': str(pair[0]), 'to': str(pair[1])})
                
        # find enclaves
        e_list = list(tree.find_data("arg_enclave"))
        if e_list:
            enclaves = e_list[len(p_list)-1]
            # get tokens
            if enclaves:
                # get tokens
                e_tokens = list(map(lambda v: v.value, list(filter(lambda vv: vv.type == 'ARG_E', (enclaves.scan_values(lambda v: isinstance(v, Token)))))))
                for en in e_tokens:
                    output['enclaves'].append({(str(en))})

        return output
    """ === Static Methods === """


"Base Launch Tag that might inherit some other classes..."
class BaseLaunchTag(object):
    # class variables
    CHILDREN = ()
    REQUIRED = ()
    CONDITIONAL_ATTRIBUTES = {
        "if": bool,
        "unless": bool
    }

    """
        true or false values:
        dic[name] = {value, default}
    """
    def __init__(self, attributes):
        self.attributes = attributes
        self.conditions = list()
        self.isTrue = True

    def _conditionals(self):
        # conditionals
        for key_attribute in self.CONDITIONAL_ATTRIBUTES:
            att = self.attributes.get(key_attribute)
            if att:
                t,v = LauncherConfigXML._arg_grammar(att)
                if v == '' or t == '':
                        return False
                "Can either be if or unless"
                if key_attribute == "if":
                        self.conditions.append((True, (t, v)))
                if key_attribute == "unless":
                        self.conditions.append((False, (t, v)))
        return True
        
    # expand conditions
    def expand_conditions(self, args):
        # conditional recursive...
        for condition in self.conditions:
                if condition is not None:
                        value = condition[1]
                        self.conditions.append(args[condition[1]].expand_conditions(args))
                else:
                        break
        # filter...
        self.conditions = list(filter(lambda c: c is not None, self.conditions))

    # resolve if conditionals
    def _if_conditional(self, args):
        # if conditionals to set up isTrue value...
        boolean_temp = True
        for condition in self.conditions:
            if condition[1] not in args:
                return False
            else:
                # put is True
                is_conditional_true = args[condition[1]].booleanValue
                boolean_temp = bool(condition[0] == is_conditional_true) and boolean_temp
            # if already false, break
            if boolean_temp == False:
                    break
        
        self.isTrue = boolean_temp
        return True

"Predefined Remap tag class"
class RemapTag(BaseLaunchTag):
    # class variables
    CHILDREN = ()
    REQUIRED = ("from", "to")
    ATTRIBUTES = {
            "from": str,
            "to": str
    } 

    def __init__(self, attributes):
        BaseLaunchTag.__init__(self, attributes=attributes)
        self.from_ = attributes.get("from")
        self.to_ = attributes.get("to")
        
        if LauncherConfigXML._arg_grammar(self.from_)[0] == True:
            pass
        if LauncherConfigXML._arg_grammar(self.to_)[0] == True:
            pass


"Predefined Node tag class"
class NodeTag(BaseLaunchTag):
    # class variables
    CHILDREN = ("remap", "param")
    REQUIRED = ("pkg", "exec")
    ATTRIBUTES = {
        "pkg": str,
        "executable": str,
        "name": str,
        "args": str
    } 

    def __init__(self, attributes, launcher):
            BaseLaunchTag.__init__(self, attributes=attributes)
            self.package    = attributes.get("pkg")
            self.executable = attributes.get("exec")
            self.name       = attributes.get("name")
            self.node_args  = attributes.get("args")
            self.namespace  = attributes.get("ns")
            
            # after processing
            self.remaps     = []
            self.enclave    = ''
            # loaded args
            self.loaded_args = launcher.args
            # run conditionals
            if not self._conditionals():
                    return None
            
            # process node...
            self.expand_conditions(self.loaded_args)
            self._if_conditional(self.loaded_args)
            self.process_remaps()
            self.process_node_args()
    
    # process remaps
    def process_remaps(self):
            remaps = self.attributes.findall('./remap')
            # go for remaps...
            temp_dict_remaps = {}
            for r in remaps:
                    r_tag = RemapTag(attributes=r)
                    tf, vf = LauncherConfigXML._arg_grammar(r_tag.from_)
                    tt, vt = LauncherConfigXML._arg_grammar(r_tag.to_)
                    
                    # process remaps...
                    if not (tf == '' or vf == ''):
                        r_tag.from_ = self.loaded_args[(tf,vf)].value
                    if not (tt == '' or vt == ''):
                        r_tag.to_ = self.loaded_args[(tt,vt)].value
                    temp_dict_remaps[r_tag.from_] = r_tag.to_
            
            # update remaps...
            for _from in temp_dict_remaps:
                    _to   = NodeTag.namespace(temp_dict_remaps[_from])
                    _from = NodeTag.namespace(_from)
                    self.remaps.append({'from': _from, 'to': _to})
                    
            # finish
            return True
    
    # process node-args in text...
    def process_node_args(self):
            output = LauncherConfigXML.parse_args(args=self.node_args)
            # process remaps
            for remap in output['remaps']:
                    self.remaps.append(remap)
            # process enclaves
            if output['enclaves'] != []:
                    self.enclave = output['enclaves'][0]
            
            self.remaps = NodeTag.transform_remaps(self.remaps)
            # finish
            return True

    @property
    def _remaps():
            return self.remaps
        
    @_remaps.setter
    def _remaps(r: RemapTag):
            self.remaps.append(r)

    # Managing and controlling remaps
    @staticmethod
    def transform_remaps(list_remaps: list):
            temp_dict = {}
            for lr in list_remaps: temp_dict[NodeTag.namespace(lr['from'])] = NodeTag.namespace(lr['to'])
            list_remaps = []
            list_remaps = list(map(lambda value: {'from': value, 'to': temp_dict[value]}, temp_dict))
            return list_remaps
    
    # Predefined control of topic remapping...
    @staticmethod
    def namespace(tag: str):
            return tag if tag.startswith('/') else f'/{tag}'


"Predefined launch arguments tag class"
class ArgTag(BaseLaunchTag):
    # class variables
    CHILDREN = ()
    REQUIRED = ("name", r"(default|value)")
    ATTRIBUTES = {
        "name": str,
        "default": str,
        "description": str
    } 

    def __init__(self, attributes, value, booleanValue):
            BaseLaunchTag.__init__(self, attributes=attributes)
            self.name         = attributes.get("name")
            self.tag          = attributes.tag
            self.booleanValue = booleanValue
            # if let or set_env
            self.value        = value
            
            # run conditionals
            if not self._conditionals():
                    return False
            self.dependent  = (False, '')
            # can be dependent
            t,v = LauncherConfigXML._arg_grammar(self.value)
            if not (v == '' or t == ''):
                    self.dependent = (True,(t,v))
    
    # evaluate arg... if value is boolean stringenized, then booleanValue must be set
    def evaluate(self):
            if self.value.capitalize() in ['True', 'False', '0', '1']:
                    self.booleanValue = False
                    if self.value.capitalize() in ['True', '1']:
                            self.booleanValue = True


"Launcher parser in order to retrieve information about possible executables..."
@dataclass
class LauncherParserXML:

    # XML-based tags
    TAGS = {
        "base": BaseLaunchTag,
        "node": NodeTag,
        "remap": RemapTag,
        "arg/let/set_env": ArgTag,
    }
    file      : str

    """ === Predifined Functions === """
    # validate xml schema
    # ROS2 launch xsd is depecrated... 
    @staticmethod
    def validate_schema(file, schema, execute_cmd=(False, '')):

        # Due to the depecrated xml file, the user might opt to check syntax through execution commands
        if execute_cmd[0] == True:
            cmd = execute_cmd[1].split(' ')
            try:
                subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            except Exception as error:
                return False
        else:
            # Schema routines...
            schema_root = etree.parse(schema)
            xml_schema = etree.XMLSchema(schema_root)
            xml_doc = etree.parse(filename)        
            try:
                xml_schema.assertValid(xml_doc)
            except Exception as error:
                return False
        # if everything goes to plan...
        return True

    # parsing launch arguments...
    @staticmethod
    def _launch_args(args, config):
        # process args, let and envs...
        for arg in args:
            # process args elements...
            booleanValue = None
            f = 'value'
            if arg.tag == 'arg':
                    f = 'default'
            # note that every let is referenced the same way as arg, so the last read is what is counted...
            if arg.tag == 'let':
                    arg.tag = 'arg'
            if arg.get(f).capitalize() in ['True', 'False', '0', '1']:
                    booleanValue = False
                    if arg.get(f).capitalize() in ['True', '1']:
                            booleanValue = True

            # create tags and add to elements...
            a_tag = ArgTag(attributes=arg, value=arg.get(f), booleanValue=booleanValue)
            config.args[(a_tag.tag, a_tag.name)] = a_tag
        
        # if everything goes to plan...
        return True

    # parsing launch nodes...
    @staticmethod
    def _launch_nodes(nodes, config):
        # process nodes, accounting its isTrue value...
        for n in nodes:
            n_tag = NodeTag(attributes=n, launcher=config)
            if n_tag.isTrue == True:
                config.nodes.append(n_tag)
        
        # if everything goes to plan...
        return True

    # parse xml launcher
    def parse(self, filename):
        # Warn the user first...
        print(f'[svROS] {color.color("BOLD", color.color("YELLOW", "WARNING"))} XML-Launch file parser might be deprecated due to complexity analysis...')
        print(f'[svROS] {color.color("BOLD", color.color("UNDERLINE", "SUPPORTED TAGS"))} Node, Let, Arg, SetEnv, Remaps, If and Unless Conditionals.')
        time.sleep(0.5)
        
        if not LauncherParserXML.validate_schema(file=filename, schema=f'{SCHEMAS}/launch.xsd', execute_cmd=(True,f'ros2 launch {filename} -p')):
            return False

        # parsing using haros parser... => ROS2 change its launch syntax...
        # parser = LaunchParser()
        tree = etree.parse(filename)
        root = tree.getroot()
        lconf = LauncherConfigXML()

        # should start with launch...
        if not root.tag == "launch":
            return False
        
        # Note that this xml parsing is easier because we can easily get the tags that we want...
        # need to parse args, nodes->remaps
        args = tree.xpath('(let|set_env|arg)')
        nodes = root.findall('./node')

        # parse args as dict
        if not LauncherParserXML._launch_args(args=args, config=lconf):
            print(f'[svROS] Failed parsing launch file: {color.color("BOLD", color.color("RED", "Launch args failed to parse!"))}')
            return False
        # process args
        if not lconf._process_args():
            print(f'[svROS] Failed parsing launch file: {color.color("BOLD", color.color("RED", "Launch args failed to process!"))}')
            return False

        # parse nodes
        if not LauncherParserXML._launch_nodes(nodes=nodes, config=lconf):
            print(f'[svROS] Failed parsing launch file: {color.color("BOLD", color.color("RED", "Launch nodes failed to parse!"))}')
            return False

        # return config declaration
        return lconf
    
    # get positional
    @staticmethod
    def decouple(structure):
        if structure is not None:
            return structure[0]

    """ === Predefined functions === """

# Testing...
if __name__ == "__main__":
    file = sys.argv[1]

    l = LauncherParserXML(file=file)
    conf = l.parse()
    print(conf.nodes)