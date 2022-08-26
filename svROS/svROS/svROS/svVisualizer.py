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

    def parse(self):
        tree = ET.parse(file)
        root = tree.getroot()
        rule1, rule2 = root.find('.//skolem[@label="$this/different_privileges"]').findall('./tuple') , root.find('.//skolem[@label="$this/access_in_privileges"]').findall('./tuple')
        if rule1 == [] and rule2 == []: 
            return None
        profiles_states_json, edges = dict(), dict()
        for tup in rule1:
            profile, role, object = self.remove_signature(value=tup.findall('./atom')[0].get('label').split('$')[0]), tup.findall('./atom')[1].get('label').split('$')[0], self.remove_signature(value=tup.findall('./atom')[2].get('label').split('$')[0].lower())
            # PROCESS TO JSON
            profiles_states_json[f'obj_{object}_allow'] = {'id': f'obj_{object}_allow', 'name': object,  'type': 'object', 'rule': 'Allow'}
            profiles_states_json[f'obj_{object}_deny'] = {'id': f'obj_{object}_deny', 'name': object,  'type': 'object', 'rule': 'Deny'}
            profiles_states_json[f'prof_{profile}'] = {'id': f'prof_{profile}', 'name': profile, 'type': 'profile'}
            edges[f'{profile}_to_{object}_priv_all'] = {'relation': f'{profile}_to_{object}_priv_all', 'source': f'prof_{profile}', 'target': f'obj_{object}_allow', 'role': role, 'call': 'privilege'}
            edges[f'{profile}_to_{object}_priv_deny'] = {'relation': f'{profile}_to_{object}_priv_deny', 'source': f'prof_{profile}', 'target': f'obj_{object}_deny', 'role': role, 'call': 'privilege'}
        for tup in rule2:
            profile, role, object = self.remove_signature(value=tup.findall('./atom')[0].get('label').split('$')[0]), tup.findall('./atom')[1].get('label').split('$')[0], self.remove_signature(value=tup.findall('./atom')[2].get('label').split('$')[0])
            # PROCESS TO JSON
            profiles_states_json[f'obj_{object}_allow'] = {'id': f'obj_{object}_allow', 'name': object,  'type': 'object', 'rule': 'Allow'}
            profiles_states_json[f'prof_{profile}'] = {'id': f'prof_{profile}', 'name': profile, 'type': 'profile'}
            edges[f'{profile}_to_{object}_sc'] = {'relation': f'{profile}_to_{object}_sc', 'source': f'prof_{profile}', 'target': f'obj_{object}_allow', 'role': role, 'call': 'source call'}
            edges[f'{profile}_to_{object}_nopriv'] = {'relation': f'{profile}_to_{object}_nopriv', 'source': f'prof_{profile}', 'target': f'obj_{object}_allow', 'role': role, 'call': 'no privilege'}
        return list(profiles_states_json.values()), list(edges.values())

    def remove_signature(self, value):
        return '/' + value.split('_', 1)[1].replace('_','/')

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
        instances    = self.__json__(nodes=nodes, edges=edges, states=states, instances=instances)
        return instances, slides

    def remove_signature(self, value):
        return '/' + value.split('_', 1)[1].replace('_','/')

    def get_nodes(self, instance):
        edges, nodes, advertises, subscribes = instance.find(f'.//skolem[@label="$this/isconnected"]').findall('./tuple'), {}, instance.find(f'.//field[@label="advertises"]').findall('./tuple'), instance.find(f'.//field[@label="subscribes"]').findall('./tuple')
        for adv in advertises:
            node = adv.findall('./atom')[0].get('label').split('$')[0]
            node = self.remove_signature(value=node)
            if node not in nodes:
                nodes[node] = {'name': node, 'type': 'node'}
        for sub in subscribes:
            node = sub.findall('./atom')[0].get('label').split('$')[0]
            node = self.remove_signature(value=node)
            if node not in nodes:
                nodes[node] = {'name': node, 'type': 'node'}
        # PROCESS EDGES ::
        edges_json = {}
        for edge in edges:
            name, src, dest = edge.findall('./atom')[1].get('label').split('$')[0], edge.findall('./atom')[0].get('label').split('$')[0], edge.findall('./atom')[2].get('label').split('$')[0]
            # name, src, dest = self.remove_signature(value=name), self.remove_signature(value=src), self.remove_signature(value=dest)
            edges_json[name] = {'source': self.remove_signature(value=src), 'target': self.remove_signature(value=dest)}
        # return list(map(lambda node: node, nodes.values())), edges_json
        return nodes, edges_json

    def get_states(self, instance):
        states_json, parent_id = {}, '24'
        states = instance.findall(f'.//field[@parentID="{parent_id}"][@label!="inbox"]')
        for state in states:
            name = state.get('label')
            if state not in states_json:
                states_json[name] = {'name': name, 'type': 'state'}
        return states
        # return list(map(lambda state: state, states_json.values()))

    def __json__(self, nodes, edges, states, instances):
        __json__ = list()
        for index in range(len(instances)):
            id, instance, object = index, instances[index], {}
            object['id'], object['edges'], object['nodes'] = id, list(), list()
            # Treat inbox and nodes
            inboxs = instance.find('.//field[@label="inbox"]').findall('./tuple')
            for i in inboxs:
                trace, name, pos, value = i.findall('./atom')[0].get('label').split('$')[0], i.findall('./atom')[1].get('label').split('$')[0], i.findall('./atom')[2].get('label').split('$')[0], i.findall('./atom')[3].get('label').split('$')[0]
                channel = edges[name]
                # Treat nodes
                node1, node2 = nodes[channel['source']], nodes[channel['target']]
                object['nodes'] += [node1, node2]
                # Treat channel
                channel['trace'], channel['relation'], channel['label'], channel['pos'] = trace, self.remove_signature(value=name), f'm = {value}', pos 
                object['edges'].append(channel)

            # Process states
            parent_id = '24'
            _states_ = instance.findall(f'.//field[@parentID="{parent_id}"][@label!="inbox"]')
            for state in _states_:
                name = state.get('label')
                for tup in state.findall(f'./tuple'):
                    trace, value = tup.findall('./atom')[0].get('label').split('$')[0], tup.findall('./atom')[1].get('label').split('$')[0]
                    state_object = states[name]
                    state_object['trace'], state_object['value'] = trace, value
                    object['nodes'].append(state_object)
            __json__.append(object)
        return __json__            