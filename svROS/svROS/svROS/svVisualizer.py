import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, json
from jinja2 import Environment, FileSystemLoader
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException, svWarning, svInfo
global WORKDIR
WORKDIR      = os.path.dirname(__file__)

class svVisualizer(object):
    HOST_NAME = "localhost"
    PORT      = 8080
    COMMAND   = "xdg-open"
    TYPES = {'ARCHITECTURE', 'OD', 'SROS'}

    def __init__(self, project, directory):
        self.project, self.directory = project, directory
        if not self.ensure_dir():
            raise svException('Failed to set up Visualizer directory.')

    def ensure_dir(self):
        try:
            if os.path.isdir(self.directory) or os.path.exists(self.directory):
                shutil.rmtree(path=self.directory)
            shutil.copytree(src=f'{WORKDIR}/visualizer', dst=self.directory)        
        except OSError as error:
            print(error)
            return False
        return True

    def run_file(self, type):
        assert type in self.TYPES
        # GENERATE JINJA
        jinja = self.generate_jinja()
        if type == 'ARCHITECTURE': 
            file, js  = f'{self.directory}/template-network.html', f'network-script.js'
            template  = jinja.get_template(f'{js}')
            data_path = open(f'{self.project.PROJECT_DIR}data/configurations.json')
            data      = json.load(data_path)
            render    = template.render(nodes=data['nodes'], connections=data['connections'])
            with open(f'{self.directory}/js/{js}', 'w+') as data:
                data.write(render)
            data_path.close()
        if type == 'OD'          : 
            file, js = f'{self.directory}/template-obsdet.html', f'obsdet-script.js'
            template = jinja.get_template(f'{js}')
        if type == 'SROS'        : 
            file, js = f'{self.directory}/template-sros.html', 'sros-script.js'
            template = jinja.get_template(f'{js}')
        # GENERATE JINJA
        return os.system(f'{self.COMMAND} {file}')
        
    def generate_jinja(self, path='visualizer/js'):
        return Environment(
            loader=FileSystemLoader(f'{WORKDIR}/{path}'),
            line_statement_prefix=None,
            line_comment_prefix=None,
            trim_blocks=True,
            lstrip_blocks=True,
            autoescape=False
        )