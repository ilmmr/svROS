import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, json
from simple_term_menu import TerminalMenu
from yaml import *
from dataclasses import dataclass, field
from datetime import datetime
from cerberus import Validator
from logging import FileHandler
from pick import pick
# InfoHandler => Prints, Exceptions and Warnings
from .svInfo import color, svException, svWarning, svInfo
# Exporters
from .svExport import svrosExport
from .svAnalyzer import svProjectExtractor, svAnalyzer
from .svData import svEnclave, svNode, Topic, svExecution

global WORKDIR, INIT_SCHEMA, _INIT_
WORKDIR  = os.path.dirname(__file__)
_INIT_       = os.path.join(os.path.dirname(__file__),"__init__.py")
_INIT_SCHEMA =  """
{
    '__name__': {
        'required': True,
        'type': 'string'
    },
    '__version__': {
        'required': True,
        'type': 'string'
    },
    '__ros_version__': {
        'required': True,
        'type': 'string'
    },
    '__ros_distro__': {
        'required': True,
        'type': 'string'
    },
    '__ros_workspace__': {
        'required': True,
        'type': 'string'
    },
    '__creation_time__': {
        'required': True,
        'type': 'string'
    }
} 
"""
_PROJECT_SCHEMA = """
{
    '__name__': {
        'required': True,
        'type': 'string'
    },
    '__original_file_path__': {
        'required': True,
        'type': 'string'
    },
    '__last_modified__': {
        'required': True,
        'type': 'string'
    }
} 
"""

"Initial Configuration class: It will initially provide information about the running ROS2 environment."
@dataclass
class Configuration:
    setup       : bool = False
    distro      : str = ''
    workspace   : str = ''
    domain_id   : str = ''
    ros_version : str = ''

    def __post_init__(self):
        if self.setup == True:
            self.export_ros_info()

    """ === Configuration functions === """
    # Export ROS info
    def export_ros_info(self):
        try:
            self.distro = os.getenv('ROS_DISTRO')
        except OSError as error:
            print("ERROR: No ROS2 distro specified --> %s" % (error.strerror))
            raise svException(message='No ROS2 distro specified.')
        # Set environment variables.
        try:
            self.domain_id   = os.getenv('ROS_DOMAIN_ID')
            self.ros_version = os.getenv('ROS_VERSION')
        except OSError as error:
            print("ERROR: Could not get any ROS environment variable! --> %s" % (error.strerror))
            raise svException(message='Could not get any ROS environment variable.')
        # Set workspace.
        try:
            self.workspace   = os.getenv('ROS_WORKSPACE')
        except OSError as error:
            print("ERROR: Could not sourced any workspace! --> %s" % (error.strerror))
            raise svException(message='Could not sourced any workspace.')
    
    # Return ROS info
    def get_ros_info(self):
        return self.distro, self.workspace, self.domain_id, self.ros_version
    """ === Configuration functions === """

"Project Parser class that through method-application helps the parsing of the user-provided files."
@dataclass
class ProjectParser:
    ros_distro      : str
    ros_workspace   : str  
    content         : dict    = field(default_factory=dict)   
    log             : logging.getLogger() = None 
    SCHEMA          : str     = """
{   
    'project': {
        'required': True,
        'type': 'string'
    },
    'launch': {
        'required': True,
        'type': 'list',
    }
}"""
    def __post_init__(self):
        if self.log is None:
            self.log = format_logger()
        if not validate(file=self.content, schema=f'{self.SCHEMA}', file_is_a_dict=True):
            raise svException(message='File given as input is not valid.')

    """ === Predefined functions === """
    # Export using svExport meta classes
    def export(self, default=True):
        if default:
            export = svrosExport(launch=self.content['launch'], project_dir=self.project_path, project=self.project, ros_distro=self.ros_distro, ros_workspace=self.ros_workspace)
            if not export.launch_export():
                raise svException(message='Failed to parse input file and its launch files.')
        return True

    @property
    def project(self):
        return self.content['project']

    @property
    def project_path(self):
        return self._project_path
    
    @project_path.setter
    def project_path(self, value):
        self._project_path = value
    """ === Predefined functions === """

"=> if $ svROS init"
@dataclass
class svINIT:
    args : dict     # Args is actually not necessary for now (for this particular class).
    _DIR : str      = os.path.join(os.path.expanduser("~"), ".svROS")
    _BIN : str      = ''
    _INIT: str      = ''
    ros  : str      = ''
    log  : logging.getLogger() = None

    def __post_init__(self):
        if self.log is None:
            self.log = format_logger()

    """ === Predefined functions === """
    # Predefined get to return info about ROS2 env variables
    def _get_ros_info(self):
        ros = self.ros.split(r'=\t=')
        return ros[0], ros[1], ros[2]

    # Creation of svROS directory
    def _create(self, log_not_reseted=False):
        created = os.path.exists(f"{self._DIR}")
        dir     =   f"{self._DIR}"
        if log_not_reseted == False and not os.path.exists(dir):
            os.mkdir(f'{dir}', mode=0o777)
        # Generate dir using a dir structure.
        try:
            os.mkdir(f'{dir}/projects', mode=0o777)
            os.mkdir(f'{dir}/.bin', mode=0o777)
            self._init_file(file=f'{dir}/.init', mode=True)
            self._log_file(file=f'{dir}/.log')
        except OSError as error:
            print("[svROS] Failed to set up svROS default directory!")
            return False
        # Copy and generate files.
        try:
            files = glob.iglob(os.path.join(f'{WORKDIR}/bin', "*.als"))
            for xfile in (list(filter(lambda x : os.path.isfile(x), files))):
                shutil.copy(xfile, f'{dir}/.bin/')
            # Create environment dir with content. => ALLOY and GENERATOR jar
            shutil.copy(f'{WORKDIR}/bin/org.alloytools.alloy.dist.jar', f'{dir}/.bin/')
            shutil.copy(f'{WORKDIR}/bin/generator.jar', f'{dir}/.bin/')
        except Exception as error:
            print("[svROS] Failed to set up svROS default directory!")
            return False
        self.log.info('Default directory setup was completed.')
        print("[svROS] svROS default directory setup was completed with success.")
        return True

    # Check the existence of the .init file in the defined svROS directory => Additionally, its syntax must also be ensured
    def _init_file(self, file, mode=False):
        f,v = validate(file=file, schema=f'{_INIT_SCHEMA}')
        if not mode:
            try:
                exists = os.path.exists(f'{self._DIR}/.init') and os.path.isfile(f'{self._DIR}/.init')
                assert(exists)
            except AssertionError as error:
                return False
            try:
                assert(f)
            except AssertionError as error:
                return False
        else:
            name = 'svROS'
            version = re.search(r"^__version__\s*=\s*(u|f|r)?['\"]([^'\"]*)['\"]", open(_INIT_, "rt").read(), re.M).group(2)
            creation_time = datetime.now().strftime("%B %d, %Y => %H:%M:%S")
            ros_version, ros_distro, ros_workspace = self._get_ros_info()
            # Validating.
            if v == '':
                v = Validator(eval(f'{_INIT_SCHEMA}'))
            dic = v.schema
            new_dict = dict()
            for key in dic:
                match = re.match(r'__(.*?)__', key).group(1)
                try:
                    new_dict[key] = locals()[match]
                except Exception:
                    raise svException(message='Init configuration failed.')
            with open(f'{file}', 'w+') as f:
                dump(new_dict, f)
        return True

    # Create log file
    def _log_file(self, file):
        # Only if not exists.
        if not os.path.exists(f'{file}'):
            ros_version, ros_distro, ros_workspace = self._get_ros_info()
            self.log.info(f"ROS version: {ros_version}; ROS distro: {ros_distro}; ROS workspace: {ros_workspace}.")

    # Ensure svROS directory existance
    def _ensure_dir(self):
        exists  = os.path.exists(f"{self._DIR}")
        dir     =   f"{self._DIR}"
        if not exists:
            print(f"[svROS] Cannot ensure a directory because {dir} does not exist.\nRUNNING $ svROS init...")
            self._create()
        if exists and not os.path.isdir(dir):
            return False
        elif exists and os.path.isdir(dir):
            init_file = os.path.exists(f'{dir}/.init') and os.path.isfile(f'{dir}/.init')
            if init_file:
                try:
                    assert(self._init_file(file=f'{dir}/.init'))
                except AssertionError as error:
                    self.log.info(f'Failed to validate svROS .init file. Might be corrupted...')
                    print(f'[svROS] Failed to verify svROS directory: {color.color("BOLD", color.color("RED", f"{dir}/.init file is corrupted"))}!')
                    return False
            else:
                return False
        # Return to the parser.
        self.log.info("svROS directory is valid.")
        print("[svROS] svROS default directory is validated.")
        return True

    # Restart svROS directory
    def _restart_dir(self):
        path = os.path.abspath(f"{self._DIR}")
        if os.path.exists(path):
            try:
                for dirpath, dirnames, filenames in os.walk(path):
                    for filename in filenames:
                        if not filename == '.log':
                            os.unlink(os.path.join(dirpath, filename))
                    for d in dirnames:
                        shutil.rmtree(f'{path}/{d}')
            except OSError as error:
                return False
        # Create directory after reset.
        if not self._create(log_not_reseted=True):
            return False
        self.log.info("============")
        self.log.info("svROS directory was successfully reseted.")
        return True
        """ === Predefined functions === """

"=> if $ svROS extract"
@dataclass
class svEXPORT:
    file       : str      = ''
    FILE_PATH  : str      = ''
    _DIR       : str      = os.path.join(os.path.expanduser("~"), ".svROS")
    _BIN       : str      = ''
    _PROJECTS  : str      = ''
    can_export : bool     = False
    reset      : bool     = False
    ros        : str      = ''
    log        : logging.getLogger() = None

    def __post_init__(self):
        if self.log is None:
            self.log = format_logger()
        if not self.can_export:
            raise svException("Could not initiate exporting of file.")
        # FILE_PATH could either be a full extended path or a relative path.
        self.FILE_PATH = self.FILE_PATH if (os.path.expanduser("~") or r'\~') in self.FILE_PATH else os.path.join(f'{WORKDIR}', f'{self.file}')

    """ === Predefined functions === """
    # Predefined get to return info about ROS2 env variables
    def _get_ros_info(self):
        ros = self.ros.split(r'=\t=')
        return ros[0], ros[1], ros[2]

    # Check the existence of the .config file in the defined project directory => Additionally, its syntax must also be ensured
    def _config_file(self, project_name, config_file, mode=False):
        f,v = validate(file=config_file, schema=f'{_PROJECT_SCHEMA}')
        if not mode:
            exists = os.path.exists(f'{config_file}') and os.path.isfile(f'{config_file}')
            if exists:
                # Validate.
                try:
                    assert(f)
                except AssertionError as error:
                    print(f'[svROS] {color.color("BOLD", color.color("RED", f"Config file from {project_name} is corrupted!"))}')
                    return False
        # Create file.
        else:
            name                = f'{project_name}'
            original_file_path  = self.FILE_PATH
            last_modified       = '__.svROS__ORIGINAL'
            if v == '':
                v = Validator(eval(f'{_PROJECT_SCHEMA}'))
            dic = v.schema
            new_dict = dict()
            for key in dic:
                match = re.match(r'__(.*?)__', key).group(1)
                try:
                    new_dict[key] = locals()[match]
                except Exception as error:
                    print(f'[svROS] {color.color("BOLD", color.color("RED", f"Failed to create {project_name} config file!"))}')
                    return False
            with open(f'{config_file}', 'w+') as f:
                dump(new_dict, f)
        return True

    # Function to retrieve project dir existance alongside with its config file
    def _exists_project_dir(self, project_name):
        project_path = os.path.join(f'{self._PROJECTS}', f'{project_name}')
        config = self._config_file(project_name=project_name, config_file=f'{project_path}/.config', mode=False)
        exists = os.path.exists(project_path)
        return exists, config

    # Setting the project directory structure up
    def _create_project_dir(self, project_name, project_path):
        try:
            os.mkdir(f'{project_path}', mode=0o777)
            os.mkdir(f'{project_path}/data', mode=0o777)
            os.mkdir(f'{project_path}/models', mode=0o777)
            # Create .config file.
            if not self._config_file(project_name, f'{project_path}/.config', mode=True):
                return False
        except OSError as error:
            return False
        return True

    # Create Project directory, considering the --reset option
    def _project_dir(self, project, reset=False):
        project_cap    = project.project.capitalize()
        path        = os.path.join(f'{self._PROJECTS}', f'{project_cap}')
        # If reset option is set, then directory must be reseted!
        if reset:
            if os.path.exists(path):
                try:
                    shutil.rmtree(path, onerror = lambda f, p, e : print(e))
                except OSError as error:
                    self.log.info(f'Failed to reset {project_cap} project directory...')
                    print(f'[svROS] Failed to reset {project_cap} project directory...')
                    return ''
            else:
                self.log.info(f'Failed to reset {project_cap} project directory... Directory does not exist.')
                print(f'[svROS] Failed to reset {project_cap} project directory... {color.color("BOLD", color.color("RED", "Directory do not exist!"))}')
                return ''
        # Create project directory.
        if not self._create_project_dir(project_name=project_cap, project_path=path):
            self.log.info(f'Failed to create {project_cap} project directory...')
            print(f'[svROS] Failed to create {project_cap} project directory...')
            return ''
        return path

    # Call parser function and some verification techniques
    def _call_parser(self, default=True):
        content = _load(self.FILE_PATH)
        if content is None:
            self.log.info(f'Failed to load {os.path.relpath(self.FILE_PATH)}.')
            print(f'[svROS] Failed to load {color.color("BOLD", f"{os.path.relpath(self.FILE_PATH)}")} => {color.color("BOLD", color.color("RED", "Not a yaml-based file!"))}')
            return False
        # Call another class instance => Project Parser <=
        ros_version, ros_distro, ros_workspace = self._get_ros_info()
        project_parser = ProjectParser(content=content, log=self.log, ros_distro=ros_distro, ros_workspace=ros_workspace)
        project_name   = project_parser.project.capitalize()
        # Project directory validater.
        exists, valid_config = self._exists_project_dir(project_name)
        if exists:
            print(f'[svROS] Project directory {color.color("ORANGE", project_name)} already exists...', end=' ')
            self.log.info(f'Project directory {project_name} already exists.')
            if not valid_config:
                # Valid config :: if not valid => RESET
                self.log.info(f'Config file deprecated... Reseting project {project_name} directory.')
                print(f'{color.color("BOLD", color.color("RED", "Config file is deprecated!"))} Reseting project {project_name} directory...')
                self.reset = True
                pass
            else:   
                if self.reset:
                    self.log.info(f'Reseting project {project_name} directory.')
                    print(f'{color.color("BOLD", "Using --reset option...")}')
                    pass
                else:
                    print(f'Make sure you reset the project directory {color.color("RED", project_name)}!')
                    return False, project_name
        path = self._project_dir(project_parser, reset=self.reset)
        if path == '': return False, project_name
        project_parser.project_path = path
        return project_parser, project_name

    # svROS import
    def _default_export(self):
        project_parser, project_name = self._call_parser(default=True)
        if isinstance(project_parser, bool):
            return False
        project_name   = project_parser.project.capitalize()
        self.log.info(f'Exporting project {project_name}...')
        # Exporter.
        project_parser.export(default=True)
        loading(txt=f'{color.color("RED", project_name)} directory in {color.color("UNDERLINE", f"{self._PROJECTS}{project_name}")}.')
        return True
    """ === Predefined functions === """

"=> if $ svROS launch"
"=> if $ svROS analyze"
@dataclass
class svRUN:
    project   : str
    _BIN      : str
    _PROJECTS : str
    _DIR      : str      = os.path.join(os.path.expanduser("~"), ".svROS")
    can_run   : bool     = False
    log       : logging.getLogger() = None

    def __post_init__(self):
        if not self.can_run:
            raise svException("Could not initiate running of project.")
        if self.log is None:
            self.log = format_logger()

    """ === Predefined functions === """
    def _run(self):
        # File existance handling
        if not os.path.exists(path=f'{self.project_path}config.yml'):
            raise svException(f'Could not initiate running of project: Config file does not exist. Please create a file named {color.color("RED", "config.yml")} in the {self.project.capitalize()} directory or export a project.')
        if not os.path.exists(path=f'{self.project_path}policies.xml'):
            raise svException(f'Could not initiate running of project: SROS file does not exist. Please create a file named {color.color("RED", "policies.xml")} in the {self.project.capitalize()} directory or export a project.')
        # Loading pre-existing data:
        project_extractor = svProjectExtractor(project=self.project, PROJECT_DIR=self.project_path)
        print(svInfo(f'Enclaves paths can be defined as the user intends. However, path "/public" is restricted to nodes considered as non-trusted!'))
        time.sleep(1)
        if project_extractor.extract_sros() and project_extractor.extract_config():
            project_analyzer  = svAnalyzer(EXTRACTOR=project_extractor, MODELS_DIR=self._BIN)
            if not (project_analyzer.security_verification() and project_analyzer.ros_verification()):
                raise svException('Could not initiate running of project => LAUNCHER FAILED.')
            # UPDATE IMPORTED DATA - JSON files
            if not project_extractor.update_imported_data():
                raise svException('Failed to update data on launching.')
            # Feedback reporting and information
            print(svInfo(f'Project {self.project.capitalize()} successfully generated {color.color("BOLD", "Alloy")} files (SROS and ROS), ready to be analyzed. Make sure to run: {color.  color("UNDERLINE", f"svROS analyze -p {self.project}")}!'))
        print(svInfo(f'Application architecture was succefully created.'))
        time.sleep(1)
        options = ["View Application Architecture", "Exit"]
        choice = TerminalMenu(options).show()
        if choice == 1:
            exit
        else:
            project_extractor.draw_architecture()
            print(svInfo(f'Application architecture is being displayed on your browser.'))
            exit

    def _analyze(self):
        project_extractor = svProjectExtractor(project=self.project, PROJECT_DIR=self.project_path)
        project_analyzer  = svAnalyzer(EXTRACTOR=project_extractor, MODELS_DIR=self._BIN, MODE=1)
        # VERIFYING SROS
        # _continue_ = input(svWarning(f'Perform verification of SROS model... [N/y] ')).strip()
        # if _continue_ in ['y', 'Y']:
        #     if not project_analyzer.alloy_sros():
        #        raise svException('Could not initiate running of project => ANALYZER FAILED.')
        # VERIFYING ROS
        _continue_ = input(svWarning(f'MODEL-CHECKING VERIFICATION MODEL... [Y/n] ')).strip()
        if not _continue_ in ['y',"", 'Y']: return
        if not project_analyzer.alloy_ros():
            raise svException('Could not initiate running of project => ANALYZER FAILED.')
        exit
    
    @property
    def project_path(self):
        project_path = self._PROJECTS + self.project.capitalize() + '/'
        if not os.path.exists(project_path):
            raise svException("Could not initiate running of project.")
        return project_path
    """ === Predefined functions === """

#############################################
#              svROS Launcher               #
#############################################
""" 
    This works as an application-launcher parser, that will eventually call up one of these classes:
        => svINIT,   if $ svROS init
        => svEXPORT, if $ svROS extract
        => svRUN,    if $ svROS launch
"""
@dataclass
class Launcher:
    """ 
    - This class contains the necessary methods to launch svROS.
    - Responsible for creating directories and executing the launcher according to the args provided.

    OPTIONS:
        => svROS init [ , --reset]
        => svROS export -f $file [ , optional]
            '-> optional:
                --force-init => Force creation of svROS dir           
                --reset      => Reset project directory 
        => svROS launch  -p $project
        => svROS analyze -p $project
    """
    # ROS2 environment variables.
    distro      : str
    workspace   : str
    domain_id   : str
    ros_version : str
    # Remaining variables.
    if not os.path.expanduser("~"): 
        raise Exception
    _DIR : str      = os.path.join(os.path.expanduser("~"), ".svROS")
    _BIN : str      = ''
    _LOG : str      = ''
    _PROJECTS : str = ''
    log : logging.getLogger() = None

    def __post_init__(self):
        if not os.path.exists(self._DIR):
            os.makedirs(self._DIR)
        self._LOG      = os.path.join(self._DIR, ".log")
        if not os.path.exists(self._LOG):
            with open(self._LOG, 'w+') as fp:
                pass
        self.log       = set_logger(log_path=self._LOG, new=(False, ''))
        self._BIN      = os.path.join(self._DIR, ".bin/")
        self._PROJECTS = os.path.join(self._DIR, "projects/")

    """ === Predefined functions === """
    # Predefined get to return info about ROS2 env variables
    def _get_ros_info(self):
        return self.ros_version, self.distro, self.workspace

    # Check the existence of the .init file in the defined svROS directory => Additionally, its syntax must also be ensured
    def _check_file(self, file, mode=False):
        f,v = validate(file=file, schema=f'{_INIT_SCHEMA}')
        # Check file structure.
        if not mode:
            try:
                exists = os.path.exists(f'{self._DIR}/.init') and os.path.isfile(f'{self._DIR}/.init')
                assert(exists)
            except AssertionError as error:
                return False, True
            try:
                assert(f)
            except AssertionError as error:
                return True, False
        return True, True
    """ === Predefined functions === """

    """ === Launcher functions === """
    # Launcher launch
    def launch(self, argv=None):
        args = self.parse(arguments=argv)
        # Get function from parsing options.
        func   = getattr(args, "func",  None)
        bin    = getattr(args, "bin",   None)
        home   = getattr(args, "home",  None)
        reset  = getattr(args, "reset", None)
        log_cl = getattr(args, "clear_log", None)
        # Retrieving function.
        if func is None:
            if not (bin or home or reset or log_cl or scopes):
                return True
            if reset:
                created = os.path.exists(f"{self._DIR}")
                if created:
                    return self.setup(args, reset=True)
                else:
                    print(f'[svROS] Failed to reset directory {self._DIR}... {color.color("BOLD", "Directory does not exist")}')
                    return False
            if log_cl:
                created = os.path.exists(f"{self._DIR}") and os.path.exists(f"{self._LOG}")
                if created:
                    if not clear_logger(logger=self.log):
                        return False
                    else:
                        return True
                else:
                    print(f'[svROS] Failed to clear directory {self._DIR}... {color.color("BOLD", "Directory does not exist")}')
                    return False
        # Output for bin and home help commands!
        if bin:
            bin_dir  = f'$HOME/{self._BIN[len(os.path.expanduser("~"))+1:]}'
            bin_tree = str(subprocess.check_output(f'tree -a {self._BIN}', shell=True).decode())[len(self._BIN):]
            print(f"\n=> --bin output <=\n{bin_dir}{bin_tree}", end='')
            return True
        if home:
            home_dir = f'$HOME/{self._DIR[len(os.path.expanduser("~"))+1:]}'
            home_tree = str(subprocess.check_output(f'tree -a {self._DIR}', shell=True).decode())[len(self._DIR):]
            print(f"\n=> --home output <=\n{home_dir}{home_tree}", end='')
            return True
        # Since it was defined a function to run withing each subparser -,
        # The idea is to parse using that defined function       <--^___/
        return func(args)

    # Default parser
    def parse(self, arguments):
        interpreter = argparse.ArgumentParser(prog = "svROS", description='=> Security Verification in ROS <=', epilog='Stay tuned for more! (ง ͡❛ ͜ʖ ͡❛)ง')
        # HELP COMMANDS
        interpreter.add_argument("--home", help=f"svROS local directory -> default: $HOME/{self._DIR[len(os.path.expanduser('~'))+1:]}", action='store_true')
        interpreter.add_argument("--reset", help=f"Reset the directory :: Including log {self._LOG} file!", action='store_true')
        interpreter.add_argument("--clear-log", help=f"Clear {self._LOG} file!", action='store_true')
        bin = ''
        if os.path.exists(f'{self._BIN}'):
            try:
                bin = str(subprocess.check_output(f'tree {self._BIN}', shell=True).decode())
            except OSError as error:
                raise svException(message='Argument parsing failed.')
        interpreter.add_argument("--bin", help=f"svROS bin directory -> default: $HOME/{self._BIN[len(os.path.expanduser('~'))+1:]}", action='store_true')
        # SUBSPARSERS: init, extract and run
        options = interpreter.add_subparsers(help='sub-command')
        init    = options.add_parser('init')
        export  = options.add_parser('extract')
        run     = options.add_parser('launch')
        analyze = options.add_parser('analyze')
        # Handling functions
        self._init(parser=init)
        self._export(parser=export)
        self._run(parser=run)
        self._analyze(parser=analyze)
        return interpreter.parse_args(arguments)

    # Handler svROS init
    def setup(self, args, reset=False):
        created = os.path.exists(f"{self._DIR}/.bin/")
        dir     =   f"{self._DIR}"
        init    = svINIT(args, _DIR=self._DIR, _BIN=self._BIN, _INIT=os.path.join(self._DIR, ".init"), ros=rf'{self.ros_version}=\t={self.distro}=\t={self.workspace}', log=self.log)

        if not created:
            print("[svROS] Running directory setup operation...")
            self.log.info("Running directory setup operation...")
            loading(txt="svROS directory setted.")
            return init._create()
        else:
            if reset == True or args.reset:
                print(f"[svROS] Reseting svROS directory...")
                self.log.info("Reseting svROS directory...")
                loading(txt="svROS directory resetted.")
                if init._restart_dir():
                    return True
                else:
                    print(f'[svROS] {color.color("RED", f"Failed to reset {color.bold(dir)}!")}')
                    self.log.info(f"Failed to reset {dir}.")
                    return False
            else:
                print(f"[svROS] svROS directory already setted! Verification is running...")
                self.log.info(f"svROS directory already setted! Verification is running...")
                loading(txt="svROS directory verified.")
                if init._ensure_dir():
                    return True
                else:
                    print(f'[svROS] {color.color("RED", f"Default directory {color.bold(dir)} exists, but its verification failed!")}')
                    self.log.info(f"Failed to verify {dir}.")
                    return False

    # => svROS init
    def _init(self, parser):
        parser.add_argument("--reset",  help = "Reset the svROS directory and all the directory subdirectories.", action="store_true")
        parser.set_defaults(func = self.setup)

    # Handler svROS export
    def command_export(self, args):
        # Check if init file exists.
        exists, init = self._check_file(f'{self._DIR}/.init', mode=False)
        if not os.path.isfile(args.file):
            print(f'[svROS] Failed to export... {color.color("RED", f"{color.bold(args.file)} is not a file")}')
            self.log.info(f"Failed to export... {args.file} is not a file.")
            return False
        # Directory may not be setted...
        if not exists:
            print("[svROS] Directory is not set!", end=' ')
            if args.force_init:
                print('\033[1m', "Using --force-init option...", '\033[0m')
                self.setup(args)
            else:
                while True:
                    opt = input('Do you want to set the environment directory up? [y/N] ').strip()
                    if opt == "y" or opt == "Y":
                        # run init associated function
                        self.setup(args)
                        break
                    elif opt == "n" or opt == "N" or opt == "":
                        print(f'[svROS] Failed to set directory up: {color.color("BOLD", "run $ svROS init!")}')
                        return False
                    else:
                        continue
        if not init:
            print(f'[svROS] Failed to export... svROS directory is corrupted: {color.color("BOLD", "run $ svROS init --reset!")}')
            self.log.info(f'Failed to export file {args.file}.')
            return False

        export = svEXPORT(file=args.file, FILE_PATH=os.path.abspath(args.file), _DIR=self._DIR, _BIN=self._BIN, _PROJECTS=self._PROJECTS, can_export=init, reset=args.reset, log=self.log, ros=rf'{self.ros_version}=\t={self.distro}=\t={self.workspace}')
        print(f'[svROS] EXPORTING file {color.color("BOLD", color.color("ORANGE", args.file))} into a project: Setup operation.')
        self.log.info(f"Exporting file {args.file} into a project: Setup operation.")
        return export._default_export()
        
    # => svROS export -f (--file) $file [, --force-init, --reset] (optional)
    def _export(self, parser):
        parser.add_argument("-f", "--file",  help = "Provide yaml-based file.", required=True)
        parser.add_argument("--force-init",  help = "Force creation of svROS directory, if not created.", action="store_true")
        parser.add_argument("--reset",  help = "Reset the project directory, if it already exists.", action="store_true")
        parser.set_defaults(func = self.command_export)

    # Handler svROS launch
    def command_run(self, args):
        # Check if init file exists.
        exists, init = self._check_file(f'{self._DIR}/.init', mode=False)
        # Directory may not be setted...
        if not exists:
            print(f'[svROS] Failed to set directory up: {color.color("BOLD", "run $ svROS init!")}')
            return False
        if not init:
            print(f'[svROS] Failed to run... svROS directory is corrupted: {color.color("BOLD", "run $ svROS init --reset!")}')
            self.log.info(f'Failed to run {args.project}...')
            return False
        
        run = svRUN(project=args.project.capitalize(), _DIR=self._DIR, _BIN=self._BIN, _PROJECTS=self._PROJECTS, can_run=init, log=self.log)
        project_name = args.project.capitalize()
        self.log.info(f'Running svROS Project => {project_name}.')
        print(f'[svROS] RUNNING svROS :: Project {color.color("BOLD", color.color("ORANGE", project_name))}')
        return run._run()

    # => svROS launch -p (--project) $project
    def _run(self, parser):
        parser.add_argument("-p", "--project", help = "Provide a project to be analyzed.", required=True)
        parser.set_defaults(func = self.command_run)

    # Handler svROS analyze
    def command_analyze(self, args):
        # Check if init file exists.
        exists, init = self._check_file(f'{self._DIR}/.init', mode=False)
        # Directory may not be setted...
        if not exists:
            print(f'[svROS] Failed to set directory up: {color.color("BOLD", "run $ svROS init!")}')
            return False
        if not init:
            print(f'[svROS] Failed to run... svROS directory is corrupted: {color.color("BOLD", "run $ svROS init --reset!")}')
            self.log.info(f'Failed to run {args.project}...')
            return False
        
        run = svRUN(project=args.project.capitalize(), _DIR=self._DIR, _BIN=self._BIN, _PROJECTS=self._PROJECTS, can_run=init, log=self.log)
        project_name = args.project.capitalize()
        self.log.info(f'Analyzing svROS Project => {project_name}.')
        print(f'[svROS] ANALYZING svROS :: Project {color.color("BOLD", color.color("ORANGE", project_name))}')
        return run._analyze()

    # => svROS analyze -p (--project) $project
    def _analyze(self, parser):
        parser.add_argument("-p", "--project", help = "Provide a project to be analyzed.", required=True)
        parser.set_defaults(func = self.command_analyze)
    """ === Launcher functions === """

###             --- additional ---               ###

# Useful for logging purposes
# Worth-Mention https://stackoverflow.com/a/56689445
#               https://stackoverflow.com/a/13733863
def set_logger(log_path='', new=(False, "")):
    # SET LOGGER CONFIG - CURRENT = ROOT.
    current_logger = logging.getLogger()
    if new[0] == True:
        current_logger = logging.getLogger(new[1])
    current_logger.setLevel(logging.INFO)
    # FILE HANDLER.
    fileHandler = logging.FileHandler(f"{log_path}", 'a')
    fileHandler.setLevel(logging.INFO)
    formatter = logging.Formatter('[svROS LOGGER] %(asctime)s :: %(message)s', "%Y-%m-%d %H:%M:%S")
    fileHandler.setFormatter(formatter)
    # FINALLY => Add the defined handler to the returning logger.
    current_logger.addHandler(fileHandler)
    return current_logger

# Clearing .log file from directory
def clear_logger(logger=None):
    if logger is None:
        return False
    else:
        if logger == logging.getLogger().name:
            logger = logging.getLogger()
        else:
            if logger in logging.root.manager.loggerDict:
                logger = logging.getLogger()
        files_to_clear = list(map(lambda fhandler: fhandler.stream.name, list(filter(lambda handler: isinstance(handler, FileHandler), logger.handlers))))
        for f in files_to_clear:
            try:
                with open(f'{f}', 'w'):
                    pass
            except:
                print(f'[svROS] {color.color("BOLD", color.color("RED", "Failed to clear log files!"))}')
                return False
    print(f'[svROS] Log files cleared!')
    return True

# Useful for validating files according to a yaml schema
# Worth-Mention https://stackoverflow.com/a/46626418
def validate(file, schema, file_is_a_dict=False):
    if os.path.exists(f'{schema}') or os.path.isfile(f'{schema}'):
        schema = eval(open(f'{schema}', 'r').read())
    else:
        schema = eval(str(f'{schema}'))
    # YAML validator -> from Cerberus.
    v = Validator(schema)
    if not file_is_a_dict:
        if not os.path.exists(f'{file}'):
            return False, ''
        try:
            assert(v.validate(safe_load(open(f'{file}', 'r'))))
        except AssertionError as error:
            return False, v
    else:
        try:
            assert(v.validate(file))
        except AssertionError as error:
            return False, v
    return True, v

# Loading effect
# Worth-Mention https://stackoverflow.com/a/61602308
animation = ["■□□□□□□","■■□□□□□", "■■■□□□□", "■■■■□□□", "■■■■■□□", "■■■■■■□", "■■■■■■■"]
def loading(txt=''):
    for i in range(len(animation)):
        time.sleep(0.2)
        sys.stdout.write("\r[svROS] " + animation[i % len(animation)])
        sys.stdout.flush()
    print(f'\t=> {color.color("BOLD", color.color("GREEN", "FINISHED:"))} {txt}')

# Load .yml file using yaml.safe_load
def _load(FILE_PATH):
    f = os.path.abspath(FILE_PATH)
    if not _check_extension(f, extension=['.yml', '.yaml']):
        return None
    with open(FILE_PATH, 'r') as stream:
        try:
            content = safe_load(stream)
        except yaml.YAMLError:
            raise Exception
    return content

# Check extension function
def _check_extension(f, extension=None):
    # extension not none
    if extension is not None:
        base, ext = os.path.splitext(f)
        if ext.lower() not in list(map(lambda e: e.lower(), extension)):
            return False
    return True

# Iterate through dict
# Worth-Mention https://stackoverflow.com/a/62897981
def iterate_dict(dictionary: dict, key: str):
    exists = key in dictionary
    if not exists:
        for key, value in dictionary.items():
            if isinstance(value, dict):
                exists = exists or iterate_dict(value, key)
    return exists
###             --- additional ---               ###

###             --- svROS main ---               ###
def main(argv=None, src=False):
    # Set some configuration up.
    configuration = Configuration(setup=True)
    distro, workspace, id, ros_version = configuration.get_ros_info()
    # LOG => ROS distro and workspace.
    print(f"[svROS] ROS_DISTRO => {distro.capitalize()}\n[svROS] ROS_WORKSPACE => {workspace}")
    # Set svROS UP!!
    launcher = Launcher(distro=distro, workspace=workspace, domain_id=id, ros_version=ros_version)
    if launcher.launch(argv=argv):
        return 0
    return 1

if __name__ == "__main__":
    main()
###             --- svROS main ---               ###