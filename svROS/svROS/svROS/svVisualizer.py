import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, json
from jinja2 import Environment, FileSystemLoader
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException, svWarning, svInfo
import xml.etree.ElementTree as ET
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

class SecurityInstanceParser(object):

    def __init__(self, file):
        self.path = file

class ODInstanceParser(object):

    def __init__(self, file):
        self.path = file

    def parse(self):
        tree = ET.parse(file)
        root = tree.getroot()
        instances = root.findall('./instance')
        if instances.__len__() <= 0: raise svException(f'Failed to parse Instance from file {self.file}.')
        slides = instances.__len__()
        nodes, edges = self.get_nodes(instances[0])
        states       = self.get_states(instances[0])
        instances    = self.__json__(edges=edges, instances=instances)
        return instances, slides, nodes + states

    def remove_signature(self, value):
        return value.split('_', 1)[1].replace('_','/')

    def get_nodes(self, instance):
        edges, nodes, advertises, subscribes = instance.find(f'.//skolem[@label="$this/isconnected"]').findall('./tuple'), {}, instance.find(f'.//field[@label="advertises"]').findall('./tuple'), instance.find(f'.//field[@label="subscribes"]').findall('./tuple')
        for adv in advertises:
            node, adv = adv.findall('./atom')[0].get('label').split('$')[0], adv.findall('./atom')[1].get('label').split('$')[0]
            node, adv = self.remove_signature(value=node), self.remove_signature(value=adv)
            if node not in nodes:
                nodes[node] = {'name': node, 'subscribes': [], 'advertises':[], 'type': 'node'}
            nodes[node]['advertises'].append(adv)
            edges.add(adv)
        for sub in subscribes:
            node, sub = sub.findall('./atom')[0].get('label').split('$')[0], sub.findall('./atom')[1].get('label').split('$')[0]
            node, sub = self.remove_signature(value=node), self.remove_signature(value=sub)
            if node not in nodes:
                nodes[node] = {'name': node, 'subscribes': [], 'advertises':[], 'type': 'node'}
            nodes[node]['subscribes'].append(sub)
            edges.add(sub)
        # PROCESS EDGES ::
        edges_json = {}
        for edge in edges:
            name, src, dest = edge.findall('./atom')[1].get('label').split('$')[0], edge.findall('./atom')[0].get('label').split('$')[0], edge.findall('./atom')[2].get('label').split('$')[0]
            # name, src, dest = self.remove_signature(value=name), self.remove_signature(value=src), self.remove_signature(value=dest)
            edges_json[name] = {'src': src, 'dest': dest}
        return list(map(lambda node: node, nodes.values())), edges_json

    def get_states(self, instance):
        states_json, parent_id = {}, '24'
        states = instance.findall(f'.//field[@parentID="{parent_id}"][@label!="inbox"]')
        for state in states:
            name = state.get('label')
            if state not in states_json:
                states_json[name] = {'name': name, 'type': 'state'}
        return list(map(lambda state: state, states_json.values()))

    def __json__(self, edges, instances):
        for index in range(len(instances)):
            id, instance = index, instances[index]
            
            