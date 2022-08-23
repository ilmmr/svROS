import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
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

    def ensure_dir(self):
        try:
            os.makedirs(path=self.directory, mode=0o777, exist_ok=True)
        except OSError as error:
            raise svException('Failed to set up Visualizer directory.')
        shutil.copytree(src=f'{WORKDIR}/visualizer', dst=self.directory)

    def run_file(self, type):
        assert type in self.TYPES
        if type == 'ARCHITECTURE': file, js = f'{self.directory}/template-network.html', f'js/network-script.js'
        if type == 'OD'          : file, js = f'{self.directory}/template-obsdet.html', f'js/obsdet-script.js'
        if type == 'SROS'        : file, js = f'{self.directory}/template-sros.html', 'js/sros-script.js'
        # GENERATE JINJA
        jinja = self.generate_jinja()
        template = jinja.get_template(f'{WORKDIR}/{js}')
        f = self.generate_from_jinja(template=template, type=type)
        return os.system(f'{self.COMMAND} {file}')

    def generate_from_jinja(self, template, type):
        return
        
    def generate_jinja(self, path='/js'):
        return Environment(
            loader=FileSystemLoader(f'{WORKDIR}{js}'),
            line_statement_prefix=None,
            line_comment_prefix=None,
            trim_blocks=True,
            lstrip_blocks=True,
            autoescape=False
        )