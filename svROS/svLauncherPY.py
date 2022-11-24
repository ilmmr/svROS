import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from collections import defaultdict
# Parsers
from lxml import etree
from lark import Lark, tree, Token
# InfoHandler => Prints, Exceptions and Warnings
from .svInfo import color, svException, svWarning
# Python parser helper
from bonsai.model import (
    CodeGlobalScope, CodeReference, CodeFunctionCall, pretty_str
)
from bonsai.analysis import (
    CodeQuery, resolve_reference, resolve_expression, get_control_depth,
    get_conditions, get_condition_paths, is_under_loop
)
from bonsai.py.py_parser import PyAstParser

"Functions that every class inherits."
class BaseCall(object):
    """ === Static Methods === """
    @staticmethod
    def get_value(call):
        return call.named_args[0].value

    @staticmethod
    def process_code_reference(call):
        reference = resolve_reference(call)
        if reference == None:
            raise svException(message=f'Failed to resolve reference from Launch call.')
        return reference
    """ === Static Methods === """

"Reference through call of LaunchConfiguration."
class ReferenceCall(BaseCall):
    REQUIRED = ("value")
    """
    ...
        \_ LaunchConfiguration ==> ReferenceCall
    """
    def __init__(self, name):
        self.name = name
        self.referenced = []

    def _add_reference(self, reference):
        self.referenced.append(reference)
        return True

"ROS2-based arguments that Nodes instances might use."
class ArgsCall(BaseCall):
    CALL_REFERENCES = {}
    ARGS            = {}
    REQUIRED = ("name", r"(default_value|value)")
    """
        DeclareLaunchArgument/SetEnvironmentVariable
            \__ name
            \__ named_args
            \__ arguments => TextSubstitution
                                \_ named_args
                                \_ LaunchConfiguration ==> ReferenceCall
    """
    def __init__(self, name, value):
        self.name   = name
        self.value  = value
        ArgsCall.ARGS[self.name] = self

    @staticmethod
    def process_references(name, value):
        if name in ArgsCall.CALL_REFERENCES:
            reference = ArgsCall.CALL_REFERENCES[name]
            for ref in reference.referenced:
                ref.value = value
        return True

    @staticmethod
    def process_argument(call=None):
        name  = call.arguments[0]
        value = ArgsCall.get_value(call=call)

        if isinstance(value, str):
            ArgsCall.process_references(name, value)
            return ArgsCall.init_argument(name=name, value=value)
        else:
            process_argument = ArgsCall.process_argument_value(name=name, value=value, tag=value.name)
            if process_argument == '':
                return None
            value = process_argument

        if isinstance(value, ReferenceCall):
            if value.name in ArgsCall.ARGS:
                arg_reference_value = ArgsCall.ARGS[value.name].value
                return ArgsCall.init_argument(name=name, value=arg_reference_value)    
            reference = ArgsCall.CALL_REFERENCES[value.name]
            arg       = ArgsCall.init_argument(name=name, value=value)
            reference._add_reference(arg)
            return arg
        else:
            # TextSubstitution
            ArgsCall.process_references(name, value)
            return ArgsCall.init_argument(name=name, value=value)

    @staticmethod
    def process_argument_value(name, value, tag):
        VALID_VALUE_TAGS = {'LaunchConfiguration', 'TextSubstitution'}
        
        if tag not in VALID_VALUE_TAGS:
            return ''
        # Text substitution is clear.
        if tag == 'TextSubstitution':
            value = ArgsCall.get_value(call=value)
        # LaunchConfiguration might be different.
        else:
            value = value.arguments[0]
            if not value in ArgsCall.CALL_REFERENCES:
                # Arguments might be a reference => Reference Call.
                value = ReferenceCall(name=value)
                ArgsCall.CALL_REFERENCES[value.name] = value
        
        if isinstance(value, CodeReference):
            value = ArgsCall.process_code_reference(value)
        return value

    @classmethod
    def init_argument(cls, name, value):
        return cls(name=name, value=value)

"Remaps that might come from the remappings tag or from node-arguments."
class RemapCall(BaseCall):
    REQUIRED = ("from", "to")
    REMAPS = set()

    def __init__(self, f, t):
        self.origin = f
        self.destin = t
        RemapCall.REMAPS.add(self)
        
    @classmethod
    def init_remap(cls, **kwargs):
        return cls(f=kwargs.get('from'), t=kwargs.get('to'))

"ROS2-based Node parsed arguments from launch file."
class NodeCall(BaseCall):
    NODES          = {}
    PACKAGES_NODES = {}
    CHILDREN = ("remap", "param")
    REQUIRED = ("package", "executable", "name")
    """
        Node
            \__ named_args
                    \_ Text
                    \_ TextSubstitution
                    \_ LaunchConfiguration ==> ReferenceCall
    """
    def __init__(self, name, package, executable, remaps, namespace=None, enclave=None):
        self.name       = name
        self.namespace  = namespace
        self.package    = package
        self.executable = executable
        self.remaps     = remaps
        self.enclave    = enclave
        index = self.name
        NodeCall.NODES[index] = self
        if self.package in NodeCall.PACKAGES_NODES: NodeCall.PACKAGES_NODES[self.package].add(self)
        else: NodeCall.PACKAGES_NODES[self.package] = {self}

    @classmethod
    def init_node(cls, **kwargs):
        return cls(name=kwargs['name'], package=kwargs['package'], executable=kwargs['executable'], remaps=kwargs['remaps'], namespace=kwargs.get('namespace'), enclave=kwargs.get('enclave'))

    @staticmethod
    def process_cmd_arg(tree, info_data, tag, enclave=False):
        data = list(tree.find_data(info_data))
        if data == []: return {}
        output = []
        _data_ = data[len(data)-1]
        if _data_:
            output = list(map(lambda v: v.value , list(filter(lambda vv: vv.type == tag, (_data_.scan_values(lambda v: isinstance(v, Token)))))))
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

        remaps  = NodeCall.process_cmd_arg(tree=tree, info_data="arg_remap", tag='ARG_R')
        enclave = NodeCall.process_cmd_arg(tree=tree, info_data="arg_enclave", tag='ARG_E', enclave=True)
        enclave = next(iter(enclave or []), None)
        for pair in remaps:
            output['remaps'].append({'from': str(pair[0]), 'to': str(pair[1])})
        output['enclave'] = str(enclave) if enclave is not None else None
        return output

    @staticmethod
    def process_argument_value(value, tag):
        VALID_VALUE_TAGS = {'LaunchConfiguration', 'TextSubstitution'}
        if tag not in VALID_VALUE_TAGS:
            return ''
        if tag == 'TextSubstitution':
            value = NodeCall.get_value(call=value)
        elif tag == 'LaunchConfiguration':
            value = value.arguments[0]
            # Reference to argument already processed.
            if value not in ArgsCall.ARGS:
                raise svException(message=f'Not a valid Launch Arg.')
            value = ArgsCall.ARGS[value].value
        else:
            raise svException(message=f'Not a valid Launch Arg.')

        if isinstance(value, CodeReference):
            value = NodeCall.process_code_reference(value)
        return value

    @staticmethod
    def process_node_argument(value, tag=None):
        if isinstance(value, str):
            return value
        else:
            # Parsing-Error processing...
            process_argument = NodeCall.process_argument_value(value=value, tag=value.name)
            if process_argument == '':
                return None
            value = process_argument
        return value

    @staticmethod
    def process_cmd_args(values):
        ASSIGNMENT_KEYS = {':=', '/', ':', '='}
        arguments_cmd = "--ros-args "
        for v in values:
            if isinstance(v, str):
                arg_cmd = v
            else:
                arg_cmd = NodeCall.process_node_argument(value=v, tag=v.name)
            arguments_cmd += f'{arg_cmd} '
        output = NodeCall.parse_cmd_args(args=arguments_cmd)
        return output.get('enclave'), output.get('remaps')

    @staticmethod
    def process_remaps(values):
        remaps = []
        for v in values:
            if isinstance(v.value[0], str):
                _from = v.value[0]
            else:
                _from = NodeCall.process_node_argument(value=v.value[0], tag=v.value[0].name)
            if isinstance(v.value[1], str):
                _to = v.value[1]
            else:
                _to = NodeCall.process_node_argument(value=v.value[1], tag=v.value[1].name)
            object_remap = {'from': _from, 'to': _to}
            remaps.append(object_remap)
        return remaps

    @staticmethod
    def process_node_arguments(arguments):
        """
        Node args to be processed:
            \_ name
            \_ package
            \_ executable
            \_ namespace (?)
            \_ remaps and arguments
        """
        VALID_NODE_ARGUMENTS = {
            'name', 'package', 'executable', 'namespace'
        }
        REMAPPINGS  = {
            'remappings'
        }
        ARGUMENTS   = {
            'arguments'
        }
        node_arguments = {}
        node_arguments['remaps'] = []
        base_arguments = list(filter(lambda arg: arg.name in VALID_NODE_ARGUMENTS, arguments))
        # Node processing through loop iteration.
        for arg in arguments:
            if arg.name in VALID_NODE_ARGUMENTS:
                node_arguments[arg.name] = NodeCall.process_node_argument(value=arg.value, tag=arg.name)
            elif arg.name in ARGUMENTS:
                # Inline cmd node arguments.
                cmd_enclave, cmd_remaps = NodeCall.process_cmd_args(values=arg.value.value)
                node_arguments['enclave'] = cmd_enclave
                for remap in cmd_remaps:
                    node_arguments['remaps'].append(remap)
                    RemapCall.init_remap(**remap)
            elif arg.name in REMAPPINGS:
                remaps = NodeCall.process_remaps(values=arg.value.value)
                for remap in remaps:
                    node_arguments['remaps'].append(remap)
                    RemapCall.init_remap(**remap)
        return node_arguments

    @staticmethod
    def process_node(call=None):
        # in-line arguments
        inline_arguments = call.named_args
        arguments        = call.arguments
        if arguments:
            raise svException(message=f'Not a valid Launch Node call.')
            return None 
        node_arguments = NodeCall.process_node_arguments(arguments=inline_arguments)
        NodeCall.init_node(**node_arguments)

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

""" 
    This file contains the necessary classes and methods to export information from the launch file Python-based specified within the config file.

    SCHEMA that ros2 provides is deprecated also... So we'll try to run ros2 launch -p instead:
        => ros2 launch $file -p

    ROS2 launch is based on python, which I, Luís Ribeiro, test the tool in order to try to retrive some useful structures to ease the parsing process, however, they do not furnish a direct way of acessing those structures. Therefore, this parsing technique might have some attached issues.
"""
"Launcher parser in order to retrieve information about possible executables..."
@dataclass
class LauncherParserPY:
    """ TAGS:
        . Node tag                  -> Reference to a node
        . DeclareLaunchArgument tag -> Arguments that can be used inside a node
        . LaunchConfiguration   tag -> Yet more arguments...
    """
    # Call-based tags
    TAGS = {
        "Base": BaseCall,
        "Node": NodeCall, # underlying remap call
        "Remap": RemapCall,
        "DeclareLaunchArgument" : {ArgsCall, ReferenceCall},
        "SetEnvironmentVariable": {ArgsCall, ReferenceCall}

    }
    file      : str

    """ === Predefined functions === """
    @staticmethod
    def validate_schema(file, execute_cmd=(False,'')):
        # Due to the depecrated xml file, the user might opt to check syntax through execution commands.
        if execute_cmd[0] == True:
            cmd = execute_cmd[1].split(' ')
            try:
                subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            except Exception as error:
                return False
        return True
    
    @staticmethod
    def validate_py_schema(file, workspace):
        # PYTHON tags that should be evaluated.
        """ TAGS:
                . Node tag                  -> Reference to a node
                . DeclareLaunchArgument tag -> Arguments that can be used inside a node
                . LaunchConfiguration   tag -> Yet more arguments...
        """
        VALID_TAGS = {
            'Node',
            'DeclareLaunchArgument',
            'SetEnvironmentVariable'
        }
        if workspace == '':
            return {}, None
        try:
            parser = PyAstParser(workspace=workspace)
            parser.parse(file)
        except:
            return {}, None

        # Parser => Global Scope
        __gs__ = parser.global_scope
        # Python calls to get processed.
        VALID_TAGS = {tag: CodeQuery(__gs__).all_calls.where_name(tag).get() for tag in VALID_TAGS}
        CALLABLE_TAGS = {}

        ### GET VALID CALLABLE FUNCTIONS ###
        callable_functions = (CodeQuery(__gs__).all_calls.where_name('LaunchDescription').get())[0].arguments
        if not re.findall(r'\#(.*?)[\,\}]\s*',str(callable_functions)):
            callable_functions = callable_functions[0].value
            for call in callable_functions:
                CALLABLE_TAGS[len(CALLABLE_TAGS)] = call
            return CALLABLE_TAGS, __gs__
        # Launch Description might have variables associated to tags.
        for _callable in re.findall(r'\#(.*?)[\,\}]\s*',str(callable_functions)): 
            scope = CodeQuery(__gs__).all_definitions.where_name(_callable).get()[0].scope
            call, name = re.findall(r'\=.*?\]\s*(.*?)\((.*?)\)\s*$',str(scope))[0]
            call = str(call).strip()
            name = str(name).strip()
            # TAG VALIDATION #
            if call in VALID_TAGS:
                if name == '':
                        name = str(_callable).strip()
                        CALLABLE_TAGS[str(_callable).strip()] = call  
                call = list(filter(lambda n_call: str(_callable).strip() == str(n_call.parent.arguments[0].name.strip()), VALID_TAGS[call]))[0]
                CALLABLE_TAGS[name] = call
        return CALLABLE_TAGS, __gs__

    @staticmethod
    def launch_py(calls={}, __gs__=None):
        ARGS_TAGS={
            'DeclareLaunchArgument',
            'SetEnvironmentVariable'
        }
        if calls == {} or parser is None:
            return False

        arguments = list(filter(lambda call: calls[call].name == 'DeclareLaunchArgument', calls))
        envs      = list(filter(lambda call: calls[call].name == 'SetEnvironmentVariable', calls))
        nodes     = list(filter(lambda call: calls[call].name == 'Node', calls))

        # Processing...
        for arg in arguments: ArgsCall.process_argument(call=calls[arg])
        for env in envs     : ArgsCall.process_argument(call=calls[env])
        for node in nodes   : NodeCall.process_node(call=calls[node])
        return True

    "Main Launch-Parser."
    def parse(self):
        # Warner the user first...
        print(f'[svROS] {color.color("BOLD", color.color("YELLOW", "WARNING:"))} Python Launch file parser might be deprecated due to complexity analysis.', f'{color.color("BOLD", color.color("UNDERLINE", "SUPPORTED TAGS"))} Node, LaunchConfiguration, SetEnvironmentVariable, DeclareLaunchArgument and LaunchDescription.')
        filename = self.file
        
        if not LauncherParserPY.validate_schema(file=filename, execute_cmd=(True,f'ros2 launch {filename} -p')):
            return False
        # Validate schema first... Note that workspace for ros launch packages must be given...
        CALLABLE_TAGS = LauncherParserPY.validate_py_schema(file=filename, workspace=LauncherParserPY.get_launch_python_workspace())
        if CALLABLE_TAGS[0] == {}:
            return False
        global_scope    = CALLABLE_TAGS[1]
        CALLABLE_TAGS   = CALLABLE_TAGS[0]
        # Here the idea is to capture all the possible tags that python launch might have.
        if not LauncherParserPY.launch_py(calls=CALLABLE_TAGS, __gs__=global_scope):
            return False
        return True
    
    # Python workspace getter so that can parse launch file with python extension.
    @staticmethod
    def get_launch_python_workspace():
        ros_distro = os.getenv('ROS_DISTRO')
        locate = f'/opt/ros2/{ros_distro}/lib'
        workspace_path = os.getenv('PYTHONPATH').split(':')[::-1][0]
        if not workspace_path.startswith(f'{locate}'):
            return ''
        if not workspace_path.endswith('/'):
            workspace_path += workspace_path + '/'
        return workspace_path

    @staticmethod
    def decouple(structure):
        if structure is not None:
            return structure[0]
    """ === Predefined functions === """

### TESTING ###
if __name__ == "__main__":
    file  = '/home/luis/Desktop/ros2launch.py'
    file2 = '/home/luis/uni-area/Tutorials-Extra-main/ROS-Applications/Tutorials/launch-files/launch-file-turtle.py'
    l = LauncherParserPY(file=file2).parse()
    print('==> NODES:', [NodeCall.NODES[n] for n in NodeCall.NODES])
    print('==> NODES names:', [NodeCall.NODES[n].name for n in NodeCall.NODES])
    print('==> NODES remaps:', [NodeCall.NODES[n].remaps for n in NodeCall.NODES])