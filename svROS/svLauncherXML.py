import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, xmlschema
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
# Parsers
import xml.etree.ElementTree as ET
from lark import Lark, tree, Token
# InfoHandler => Prints, Exceptions and Warnings
from .svInfo import color, svException, svWarning

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, 'schemas')

""" 
    This file contains the necessary classes and methods to export information from the launch file XML-based specified within the config file.
"""
"Functions that every class inherits."
class BaseLaunchTag(object):
    CONDITIONAL_ATTRIBUTES = {
        "if": bool,
        "unless": bool
    }
    """ LAUNCH TREE according to xsd...
            => node (can have if and unless)
                    \-> remap tag (can have arg/let values...)
            => arg  (can have if and unless)
            => let  (can have if and unless)
    """
    """ === Static Methods === """
    @staticmethod
    def _filter(args: dict, tag: str):
        return list(map(lambda ob: args[ob], list(filter(lambda obj: obj[1] == tag, list(args.keys())))))

    @staticmethod
    def _arg_grammar(sentence='') -> (bool,str):
        # Parsing Grammar.
        grammar = """
            sentence: /\$/ LP ARG NAME RP

            LP: "("
            RP: ")"
            ARG: "var" | "env"
            NAME: /[a-zA-Z0-9_\/\-.]+/
            
            %import common.WS
            %ignore WS
        """
        parser = Lark(grammar, start='sentence', ambiguity='explicit')
        try:
                tree   = parser.parse(f'{sentence}')
        except:
                return '', ''
        ARG = list(filter(lambda t: t.type == "ARG" , tree.children))[0].value
        token = list(filter(lambda t: t.type == "NAME" , tree.children))[0].value
        if str(ARG) == 'env': return 'set_env', token
        return 'arg', token

    @staticmethod
    def decouple(structure):
        if structure is not None:
            return structure[0]

    @staticmethod
    def namespace(tag: str):
        return tag if tag.startswith('/') else f'/{tag}'

    @staticmethod
    def set_conditionals(element, node_mode=False):
        for conditional in BaseLaunchTag.CONDITIONAL_ATTRIBUTES:
            cond = element.get(conditional)
            if cond:
                t,v = BaseLaunchTag._arg_grammar(cond)
                if v == '' or t == '':
                    return False
                if node_mode:
                    return ArgsTag.ARGS[(v,t)].isTrue
                return ReferenceIf(name=v, tag=t, condition=bool(cond=='if'))
        return True
    """ === Static Methods === """

"Reference through call of LaunchConfiguration"
class ReferenceVar(BaseLaunchTag):
    REQUIRED = ("value")
    """
    ...
        \_ $var or $env ==> ReferenceVar
    """
    def __init__(self, name, tag):
        # initial configuration
        self.name       = name
        self.tag        = tag


class ReferenceIf(BaseLaunchTag):
    REQUIRED = ("value")
    """
    ...
        \_ $var or $env in IF or UNLESS ==> ReferenceIf
    """
    def __init__(self, name, tag, condition=True):
        # initial configuration
        self.name       = name
        self.tag        = tag
        self.condition  = condition


"Remaps that might come from the remappings tag or from node-arguments."
class RemapTag(BaseLaunchTag):
    REQUIRED = ("from", "to")
    REMAPS = set()

    def __init__(self, f, t):
        self.origin = f
        self.destin = t
        RemapTag.REMAPS.add(self)
        
    @classmethod
    def init_remap(cls, **kwargs):
        return cls(f=kwargs.get('from'), t=kwargs.get('to'))
        

"Predefined Node tag class"
class NodeTag(BaseLaunchTag):
    NODES          = {}
    PACKAGES_NODES = {}
    CHILDREN = ("remap", "param")
    REQUIRED = ("pkg", "exec")
    """
        Node
            \__ arguments
                \_ conditionals
                \_ attributes
            \__ remaps
    """
    def __init__(self, name, package, executable, remaps, namespace=None, enclave=None):
        self.name       = name
        self.namespace  = namespace
        self.package    = package
        self.executable = executable
        self.remaps     = remaps
        self.enclave    = enclave
        index = self.name
        NodeTag.NODES[index] = self
        if self.package in NodeTag.PACKAGES_NODES: NodeTag.PACKAGES_NODES[self.package].add(self)
        else: NodeTag.PACKAGES_NODES[self.package] = {self}

    @classmethod
    def init_node(cls, **kwargs):
        return cls(name=kwargs['name'], package=kwargs['pkg'], executable=kwargs['exec'], remaps=kwargs['remaps'], namespace=kwargs.get('namespace'), enclave=kwargs.get('enclave'))
    
    @staticmethod
    def process_node_argument(arg):
        if not isinstance(arg, str):
            return None
        tag, value = BaseLaunchTag._arg_grammar(arg)
        if not (tag == '' or value == ''):
            arg = ArgsTag.ARGS[(value, tag)].value
            
        return arg

    @staticmethod
    def process_node_arguments(arguments=None):
        """
        Node args to be processed:
            \_ name
            \_ package
            \_ executable
            \_ namespace (?)
            \_ remaps and arguments
        """
        SIMPLE_NODE_ARGUMENTS = {
            'name', 'pkg', 'exec', 'namespace'
        }

        valid = BaseLaunchTag.set_conditionals(arguments, node_mode=True)
        if not valid:
            #print(f'[svROS] {color.color("BOLD", color.color("YELLOW", "A node is invalid due to its conditionals!"))}')
            return None

        node_arguments = {}
        node_arguments['remaps'] = []
        for valid in SIMPLE_NODE_ARGUMENTS:
            node_arguments[valid] = NodeTag.process_node_argument(arg=arguments.get(valid))
        
        process_remaps = NodeTag.process_remaps(node=arguments)
        for remap in process_remaps:
            node_arguments['remaps'].append(remap)
            RemapTag.init_remap(**remap)

        in_line_args = arguments.get('args')
        if in_line_args is not None:
            if not isinstance(in_line_args, str):
                #print(f'[svROS] {color.color("BOLD", color.color("RED", "A node failed to parse!"))}')
                return None
            cmd_enclave, cmd_remaps = NodeTag.parse_cmd_args(args=in_line_args)
            node_arguments['enclave'] = cmd_enclave
            for remap in cmd_remaps:
                if remap in process_remaps:
                    continue
                node_arguments['remaps'].append(remap)
                RemapTag.init_remap(**remap)

        return node_arguments
            
    @staticmethod
    def process_node(node=None):
        arguments = node
        node_arguments = NodeTag.process_node_arguments(arguments=arguments)
        if node_arguments is not None:
            NodeTag.init_node(**node_arguments)
        return True

    @staticmethod
    def process_remaps(node):
            tags = node.findall('./remap')
            remaps = []
            # For each remap tag run this snippet.
            for remap_tag in tags:
                (origin, destin) = (remap_tag.get('from'), remap_tag.get('to'))
                origin = NodeTag.process_node_argument(arg=origin)
                destin = NodeTag.process_node_argument(arg=destin)
                
                object_remap = {'from': BaseLaunchTag.namespace(origin), 'to': BaseLaunchTag.namespace(destin)}
                remaps.append(object_remap)
            return remaps

    @staticmethod
    def process_cmd_arg(tree, info_data, tag, enclave=False):
        data = list(tree.find_data(info_data))
        if data == []: return {}
        output = {}
        _data_ = data[len(data)-1]
        if _data_:
            output = list(map(lambda v: v.value , list(filter(lambda vv: str(vv.type) == tag, _data_.scan_values(lambda v: isinstance(v, Token))))))
            if enclave == False:
                output = list(zip(output[0::2], output[1::2]))
        return output
    
    "Grammar to parse inline node arguments."
    @staticmethod
    def parse_cmd_args(args=''):
        output = {}
        output['remaps'] = list()
            
        # Grammar to parse arguments.
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
        parser = Lark(grammar, start='sentence', ambiguity='explicit')
        tree = parser.parse(f'{args}')
        
        remaps  = NodeTag.process_cmd_arg(tree=tree, info_data="arg_remap", tag='ARG_R')
        enclave = NodeTag.process_cmd_arg(tree=tree, info_data="arg_enclave", tag='ARG_E', enclave=True)
        enclave = next(iter(enclave or []), None)
        for pair in remaps:
            output['remaps'].append({'from': str(pair[0]), 'to': str(pair[1])})
        output['enclave'] = str(enclave) if enclave is not None else None
        return output.get('enclave'), output.get('remaps')

    @property
    def name(self):
        if self.namespace is None:
            name = self._name
        else:
            name = self.namespace + '/' + self._name
        return name
    
    @name.setter
    def name(self, value):
        self._name = value

"ROS2-based arguments that Nodes instances might use."
class ArgsTag(BaseLaunchTag):
    ARGS         = {}
    REQUIRED = ("name", r"(default|value)")
    """
        ArgTag
            \_ arg
                \_ conditionals n references
            \_ let
                \_ conditionals n references
            \_ set_env
                \_ conditionals n references
    """
    def __init__(self, name, tag, value, valid):
        self.name         = name
        self.tag          = tag
        self.value        = value
        self.valid        = [valid]
        self.isTrue       = True
        ArgsTag.ARGS[(name, tag)] = self

    @classmethod
    def init_argument(cls, name, tag, value, valid):
        return cls(name=name, tag=tag, value=value, valid=valid)

    @staticmethod
    def process_value(arg, tag=None):
        if not tag:
            raise svException(message=f'Failed to read Launch tag.')
  
        value = arg.get('value')
        if tag == 'arg':
            value = arg.get('default')
        t,v = BaseLaunchTag._arg_grammar(value)
        if not (v == '' or t == ''):
            value = ReferenceVar(name=v, tag=t)
        return value
        
    @staticmethod
    def process_argument(argument=None):
        DEFAULT_TAGS = {'name', 'tag', r'value|default', 'if', 'unless'}

        name  = argument.get('name')
        tag   = argument.tag
        value = ArgsTag.process_value(argument, tag)
        valid = ArgsTag.set_conditionals(argument)
        "elif isinstance(conditional, ReferenceIf)"
        if isinstance(valid, bool):
            if not valid:
                return None
            else:
                valid = True
        return ArgsTag.init_argument(name, tag, value, valid)

    @classmethod
    def process_valid_arguments(cls):
        for argument in cls.ARGS:
            element       = cls.ARGS[argument]
            element.value = cls.process_var_reference(element)
            cls.process_if_reference(element)
            cls.process_valid(element)
        return True

    @staticmethod
    def process_if_reference(argument):
        if isinstance(argument.valid[0], ReferenceIf):
            (name, tag, condition) = (argument.valid[0].name, argument.valid[0].tag, argument.valid[0].condition)
            reference = ArgsTag.ARGS.get((name,tag))
            if tag == 'arg':
                if reference is None:
                    tag = 'let'
                    reference = ArgsTag.ARGS.get((name,tag))
            argument.valid.append(ArgsTag.process_if_reference(reference))
        else:
            return bool(argument.valid[0] == ArgsTag.evaluate(argument.value))
        
    @staticmethod
    def process_valid(argument):
        boolean_temp = argument.isTrue
        for condition in argument.valid:
            if not isinstance(condition, bool):
                (name, tag, condition) = (condition.name, condition.tag, condition.condition)
                if tag == 'arg':
                    reference = ArgsTag.ARGS.get((name,tag))
                    if reference is None:
                        tag = 'let'
                condition = bool(ArgsTag.ARGS[(name,tag)] == condition)
            boolean_temp = condition and boolean_temp
            if boolean_temp == False:
                break
        argument.isTrue = boolean_temp
        return True

    @staticmethod
    def process_var_reference(element):
        if isinstance(element.value, ReferenceVar):
            reference = element.value

            if (reference.name, reference.tag) not in ArgsTag.ARGS:
                if reference.tag == 'arg':
                    if (reference.name, 'let') not in ArgsTag.ARGS:
                        raise svException(message=f'Not a valid Launch Arg.')
                    reference.tag = 'let'
            value = ArgsTag.process_var_reference(element=ArgsTag.ARGS[(reference.name, reference.tag)])
        else:
            value = element.value
        return value

    @staticmethod
    def evaluate(value):
        returning_boolean = None
        if value.capitalize() in ['True', 'False', '0', '1']:
            returning_boolean = False
            if value.capitalize() in ['True', '1']:
                    returning_boolean = True
        return returning_boolean

""" 
    SCHEMA that ros2 provides is deprecated also... So we'll try to run ros2 launch -p instead: => ros2 launch $file -p
"""
"Launcher parser in order to retrieve information about possible executables..."
@dataclass
class LauncherParserXML:
    TAGS = {
        "base": BaseLaunchTag,
        "node": NodeTag,
        "remap": RemapTag,
        "arg/let/set_env": ArgsTag,
    }
    file      : str
    """ === Predifined Functions === """
    @staticmethod
    def validate_schema(file, schema, execute_cmd=(False, '')):
        # Due to the depecrated xml file, the user might opt to check syntax through execution commands.
        if execute_cmd[0] == True:
            cmd = execute_cmd[1].split(' ')
            try:
                subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            except Exception as error:
                return False
        else:
            # Schema Routines.
            xml_schema = xmlschema.XMLSchema(schema)        
            try:
                xml_schema.validate(file)
            except Exception as error:
                return False
        return True

    "Main Launch-Parser."
    def parse(self):
        # Warn the user first...
        print(f'[svROS] {color.color("BOLD", color.color("YELLOW", "WARNING"))} XML-Launch file parser might be deprecated due to complexity analysis.', f'{color.color("BOLD", color.color("UNDERLINE", "SUPPORTED TAGS"))} Node, Let, Arg, SetEnv, Remaps, If and Unless Conditionals.')
        filename = self.file

        if not LauncherParserXML.validate_schema(file=filename, schema=f'{SCHEMAS}/launch.xsd', execute_cmd=(True,f'ros2 launch {filename} -p')):
            return False
        tree = ET.parse(filename)
        root = tree.getroot()
        if not root.tag == "launch":
            return False
        arguments = root.findall('./let') + root.findall('./set_env') + root.findall('./arg')
        nodes     = root.findall('./node')

        # Processing...
        for arg  in arguments: ArgsTag.process_argument(argument=arg)
        if not ArgsTag.process_valid_arguments():
            return False
        for node in nodes    : NodeTag.process_node(node=node)
        return True 
    
    @staticmethod
    def decouple(structure):
        if structure is not None:
            return structure[0]
    """ === Predefined functions === """

### TESTING ###
if __name__ == "__main__":
    file = '/home/luis/Desktop/example.xml'
    l = LauncherParserXML(file=file).parse()
    print('==> NODES:', [NodeTag.NODES[n] for n in NodeTag.NODES])
    print('==> NODES names:', [NodeTag.NODES[n].name for n in NodeTag.NODES])
    print('==> NODES remaps:', [NodeTag.NODES[n].remaps for n in NodeTag.NODES])