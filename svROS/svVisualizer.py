import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, json
from jinja2 import Environment, FileSystemLoader
# InfoHandler => Prints, Exceptions and Warnings
from .svInfo import color, svException, svWarning, svInfo
import xml.etree.ElementTree as ET
global WORKDIR
WORKDIR      = os.path.dirname(__file__)
VISUALIZER   = os.path.join(WORKDIR, 'visualizer')

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
            shutil.copytree(src=f'{VISUALIZER}', dst=self.directory)        
        except OSError as error:
            print(error)
            return False
        return True

    def run_file(self, type, file=''):
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
            inst, slides = ODInstanceParser(file=file).parse()
            file, js = f'{self.directory}/template-obsdet.html', f'obsdet-script.js'
            template = jinja.get_template(f'{js}')
            render    = template.render(instances=inst, slides=slides)
            with open(f'{self.directory}/js/{js}', 'w+') as data:
                data.write(render)
        if type == 'SROS'        : 
            nodes, edges = SecurityInstanceParser(file=file).parse()
            file, js = f'{self.directory}/template-sros.html', 'sros-script.js'
            template = jinja.get_template(f'{js}')
            render    = template.render(nodes=nodes, edges=edges)
            with open(f'{self.directory}/js/{js}', 'w+') as data:
                data.write(render)
        # GENERATE JINJA
        return os.system(f'{self.COMMAND} {file}')
        
    def generate_jinja(self, path='js'):
        return Environment(
            loader=FileSystemLoader(f'{VISUALIZER}/{path}'),
            line_statement_prefix=None,
            line_comment_prefix=None,
            trim_blocks=True,
            lstrip_blocks=True,
            autoescape=False
        )

class SecurityInstanceParser(object):

    def __init__(self, file):
        self.path = file

    def parse(self):
        tree = ET.parse(file)
        root = tree.getroot()
        rule = root.find('.//skolem[@label="$this/different_privileges"]').findall('./tuple')
        if rule == []:
            return None
        profiles_states_json, edges = dict(), dict()
        for tup in rule:
            profile, role, object = self.remove_signature(value=tup.findall('./atom')[0].get('label').split('$')[0]), tup.findall('./atom')[1].get('label').split('$')[0], self.remove_signature(value=tup.findall('./atom')[2].get('label').split('$')[0].lower())
            # PROCESS TO JSON
            profiles_states_json[f'obj_{object}'] = {'id': f'obj_{object}', 'name': object,  'type': 'object'}
            profiles_states_json[f'prof_{profile}'] = {'id': f'prof_{profile}', 'name': profile, 'type': 'profile'}
            edges[f'{profile}_to_{object}_priv_all'] = {'relation': f'{profile}_to_{object}_priv_all', 'source': f'prof_{profile}', 'target': f'obj_{object}', 'role': role, 'rule': 'Allow'}
            edges[f'{profile}_to_{object}_priv_deny'] = {'relation': f'{profile}_to_{object}_priv_deny', 'source': f'prof_{profile}', 'target': f'obj_{object}', 'role': role, 'rule': 'Deny'}
        return list(profiles_states_json.values()), list(edges.values())

    def remove_signature(self, value):
        return '/' + value.split('_', 1)[1]

class ODInstanceParser(object):

    def __init__(self, file):
        self.path = file

    def parse(self):
        tree = ET.parse(self.path)
        root = tree.getroot()
        instances = root.findall('./instance')
        if instances.__len__() <= 0: raise svException(f'Failed to parse Instance from file {self.file}.')
        slides = instances.__len__()
        nodes, edges = self.get_nodes(instances[0])
        states       = self.get_states(instances[0])
        instances    = self.__json__(nodes=nodes, edges=edges, states=states, instances=instances)
        return instances, slides

    def remove_signature(self, value):
        return '/' + value.split('_', 1)[1]

    def get_nodes(self, instance):
        nodes, advertises, subscribes = {}, instance.find(f'.//field[@label="advertises"]').findall('./tuple'), instance.find(f'.//field[@label="subscribes"]').findall('./tuple')
        edges = {}
        for adv in advertises:
            # Process topic
            topic = adv.findall('./atom')[1].get('label').split('$')[0]
            if topic not in edges:
                edges[topic] = (set(),set())
            # node
            node  = adv.findall('./atom')[0].get('label').split('$')[0]
            node  = self.remove_signature(value=node)
            edges[topic][0].add(node)
            if node not in nodes:
                nodes[node] = {'id': node, 'name': node, 'type': 'node'}
        for sub in subscribes:
            # Process topic
            topic = sub.findall('./atom')[1].get('label').split('$')[0]
            if topic not in edges:
                edges[topic] = (set(),set())
            # node
            node = sub.findall('./atom')[0].get('label').split('$')[0]
            node = self.remove_signature(value=node)
            edges[topic][1].add(node)
            if node not in nodes:
                nodes[node] = {'id': node, 'name': node, 'type': 'node'}
        # PROCESS EDGES ::
        edges_json = {}
        for edge in edges:
            pairs = edges[edge]
            for src in pairs[0]:
                for dest in pairs[1]:
                    if edge not in edges_json:
                        edges_json[edge] = []
                    edges_json[edge].append({'source': src, 'target': dest})
        return nodes, edges_json

    def get_states(self, instance):
        states_json, parent_id = {}, '24'
        states = instance.findall(f'.//field[@parentID="{parent_id}"][@label!="inbox"]')
        for state in states:
            name = state.get('label')
            if state not in states_json:
                states_json[name] = {'name': name, 'type': 'state'}
        return states

    def __json__(self, nodes, edges, states, instances):
        __json__ = list()
        for index in range(len(instances)):
            id, instance, object = index, instances[index], {}
            object['id'], object['edges'], object['nodes'] = id, list(), list()
            # Treat inbox and nodes
            inboxs = instance.find('.//field[@label="inbox"]').findall('./tuple')
            index  = 0
            for i in inboxs:
                trace, name, pos, value = i.findall('./atom')[0].get('label').split('$')[0], i.findall('./atom')[1].get('label').split('$')[0], i.findall('./atom')[2].get('label').split('$')[0], i.findall('./atom')[3].get('label').split('$')[0]
                channel = edges[name]
                for ind in range(len(channel)):
                    c = channel[ind].copy()
                    # Treat nodes
                    node1, node2 = nodes[c['source']].copy(), nodes[c['target']].copy()
                    if node1 not in object['nodes']:
                        object['nodes'].append(node1)
                    if node2 not in object['nodes']:
                        object['nodes'].append(node2)
                    # Treat channel
                    c['trace'], c['relation'], c['label'], c['pos'] = trace, self.remove_signature(value=name), f'm = {value}', pos 
                    object['edges'].append(c)
                    index = index + 1

            # Process states
            parent_id = '24'
            _states_ = instance.findall(f'.//field[@parentID="{parent_id}"][@label!="inbox"]')
            for state in _states_:
                name = state.get('label')
                for tup in state.findall(f'./tuple'):
                    trace, value = tup.findall('./atom')[0].get('label').split('$')[0], tup.findall('./atom')[1].get('label').split('$')[0]
                    state_object = states[name].copy()
                    # Process STATE -> STATE_VALUE
                    state_value  = {'id': f'{name}_{value}', 'name': self.remove_signature(value=value), 'type': 'state_value'}
                    if state_value not in object['nodes']:
                        object['nodes'].append(state_value)
                    object['edges'].append({'trace': trace, 'source': name, 'target': f'{name}_{value}', 'label': '', 'pos': ''})
                    if state_object not in object['nodes']:
                        object['nodes'].append(state_object)
            __json__.append(object)
        return __json__            