import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException
# XML Parser
import xml.etree.ElementTree as ET
# svROS HPL Exporter

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

""" 
    This file contains the necessary classes and methods to store and retrieve information about the ros2 running environment => NODES, TOPICS AND PACKAGES INVOLVED.
"""
"ROS2-based Package class."
class Package:
    PACKAGES = set()
    """
        Packages
            \_ Valid nodes from packages
    """
    def __init__(self, name: str, path: str, nodes: dict):
        self.name, self.path, self.nodes = name, path, nodes
        Package.PACKAGES.add(self)

    @classmethod
    def init_package_name(cls, name, index):
        # Only clear if its the first package.
        if index == 0: cls.PACKAGES.clear()
        return cls(name=name, path='', nodes=None)

"ROS2-based Topic already parse for node handling."
class Topic(object):
    TOPICS = set()
    """
        Topic
            \__ Name
            \__ Type
    """
    def __init__(self, name, topic_type):
        self.name, self.type, self.remap = name, topic_type, None
        Topic.TOPICS.add(self)

    @classmethod
    def init_topic(cls, **kwargs):
        return cls(name=kwargs['name'], type=kwargs['topic_type'])

    @staticmethod
    def namespace(tag: str):
        return tag if tag.startswith('/') else f'/{tag}'
    
    def rosname(self, node):
        if node and node.namespace:
            rosname = Topic.namespace(tag=node.namespace)
            if self.remap: rosname += Topic.namespace(tag=self.remap)
            else:          rosname += Topic.namespace(tag=self.name)
        else:
            if self.remap: rosname = Topic.namespace(tag=self.remap)
            else:          rosname = Topic.namespace(tag=self.name)
        return rosname

"ROS2-based Node already parse, with remaps and topic handling."
class Node(object):
    NODES = {}
    """
        Node
            \__ INFO FROM NODETAG OR NODECALL
            \__ Topic subscribing and publishing
    """
    def __init__(self, name, namespace, package, executable, remaps, enclave=None):
        self.name, self.namespace, self.package, self.executable, self.remaps, self.enclave  = name, namespace, package, executable, remaps, enclave
        # Associated source file.
        self.source     = None
        # Add to NODES class variable.
        index = self.index
        Node.NODES[index] = self

    # Update node with its Source and Topic Handling
    def store_node_source(self, source):
        self.source, self.subscribes, self.publishes = source.name, source.subscribes, source.publishes

    @classmethod
    def process_config_file(cls):
        # Returning object.
        ret_object = {}
        for index in cls.NODES:
            node  = cls.NODES[index]
            ret_object[index]               = {}
            ret_object[index]['rosname']    = node.rosname
            ret_object[index]['enclave']    = node.enclave if node.enclave else Node.namespace(tag=index)
            # Topic treatment.
            if node.publishes:  ret_object[index]['advertise'] = {}
            if node.subscribes: ret_object[index]['subscribe'] = {}
            if not node.source:
                continue
            for adv in node.publishes:
                topic_type = adv.type
                name       = Node.render_remap(topic=adv, remaps=node.remaps).rosname(node=node)
                ret_object[index]['advertise'][name] = topic_type
            for sub in node.subscribes:
                topic_type = sub.type
                name       = Node.render_remap(topic=sub, remaps=node.remaps).rosname(node=node)
                ret_object[index]['subscribe'][name] = topic_type
            # HPL Properties
            ret_object[index]['hpl'] = {'properties': ['']}
        return ret_object

    @classmethod
    def process_sros_file(cls, template):
        enclaves = template.find(f'.//enclaves')
        for index in cls.NODES:
            node = cls.NODES[index]
            # Process Enclave
            if node.enclave is not None:
                enclave = None
                for en in enclaves:
                    if en.get('path') == str(node.enclave):
                        enclave  = en
                        profiles = enclave[0]
                        break
                if enclave is None:
                    enclave  = ET.Element('enclave')
                    enclave.set('path', str(node.enclave))
                    profiles = ET.Element('profiles')
                    enclave.append(profiles)
                    enclaves.append(enclave)
            else:
                enclave  = ET.Element('enclave')
                enclave.set('path', Node.namespace(tag=node.index))
                profiles = ET.Element('profiles')
                enclave.append(profiles)
                enclaves.append(enclave)
            # Process Node
            profile = ET.Element('profile')
            if node.namespace: profile.set('ns', Node.namespace(tag=node.namespace) + '/')
            else: profile.set('ns', '/')
            profile.set('node', node.name)
            # Process Topic Publish
            pubs = ET.Element('topics')
            pubs.set('publish', 'ALLOW')
            pubs_names = []
            for adv in node.publishes:
                topic = Node.render_remap(topic=adv, remaps=node.remaps)
                if topic.remap: name = topic.remap 
                else:           name = topic.name
                if name in pubs_names: continue
                pubs_names.append(name)
                topic      = ET.Element('topic')
                topic.text = str(name) if not name.startswith('/') else str(name[1:])
                pubs.append(topic)
            # Process Topic Subscribe
            subs = ET.Element('topics')
            subs.set('subscribe', 'ALLOW')
            subs_names = []
            for sub in node.subscribes:
                topic = Node.render_remap(topic=sub, remaps=node.remaps)
                if topic.remap: name = topic.remap 
                else:           name = topic.name
                if name in subs_names: continue
                subs_names.append(name)
                topic      = ET.Element('topic')
                topic.text = str(name) if not name.startswith('/') else str(name[1:])
                subs.append(topic)
            # Processing Profile
            if subs.findall('./') != []: profile.append(subs)
            if pubs.findall('./') != []: profile.append(pubs)
            profiles.append(profile)
        # Retrieve XML
        return template

    # Retrieve JSON-based dict information
    @staticmethod
    def to_json(node: str):
        node   = Node.NODES[node]
        # Enclave is not needed at this point
        topics = {'subscribe': list(map(lambda subs: subs.rosname(node=node), node.subscribes)), 'advertise': list(map(lambda pubs: pubs.rosname(node=node), node.publishes)), 'remaps': node.remaps}
        return {'package': node.package, 'executable': node.executable, 'namespace': node.namespace, 'rosname': node.rosname, 'topics': topics}

    @staticmethod
    def render_remap(topic, remaps):
        name = Topic.namespace(tag=topic.name)
        for r in remaps:
            if name.strip() == r['from'].strip():
                name        =  r['to'].strip()
                topic.remap = name
                break
        return topic

    @staticmethod
    def namespace(tag: str):
        return tag if tag.startswith('/') else f'/{tag}'
    
    @classmethod
    def init_node(cls, **kwargs):
        return cls(name=kwargs['_name'], namespace=kwargs['namespace'], package=kwargs['package'], executable=kwargs['executable'], remaps=Node.process_remaps(kwargs['remaps']), enclave=kwargs.get('enclave'))

    @staticmethod
    def process_remaps(remaps: list):
        # Snub list
        remap_temp = dict()
        for remap in remaps:
            remap_temp[remap.get('from')] = remap.get('to')
        # Iter through temp dict
        for remap in remap_temp:
            value = remap_temp[remap]
            value = Node.process_remap_value(remap=remap, value=value, remaps=remap_temp)
            # Process after getting new value
            if remap == value:
                del remap_temp[remap]
                continue
            remap_temp[remap] = value
        return list(map(lambda remap: {'from': remap, 'to': remap_temp[remap]}, remap_temp))

    @staticmethod
    def process_remap_value(remap, value, remaps):
        if value in remaps:
            current_value = remaps[value]
            if list(remaps.keys()).index(value) > list(remaps.keys()).index(remap):
                value = Node.process_remap_value(value, current_value, remaps)
        return value

    @property
    def rosname(self):
        if self.namespace: rsn_ = self.namespace + '/' + self.name
        else: rsn_ = self.name
        return Node.namespace(tag=rsn_)

    @property
    def index(self):
        if self.namespace: index_ = self.package + '/' + self.namespace + '/' + self.name
        else: index_ = self.package + '/' + self.name
        return index_

""" 
    The remaining of this file contains the necessary classes and methods to retrieve and use information for analysis purposes. 
"""
"ROS2-based Node to be analyzed."
class svROSNode(object):
    NODES        = {}
    LOADED_NODES = {}
    """
        svROSNode
            \__ Already parsed node
            \__ Associated with Profile from SROS (can either be secured or unsecured)
    """
    def __init__(self, full_name, profile, **kwargs):
        self.index, self.rosname, self.namespace, self.executable, self.enclave, self.advertise, self.subscribe, self.properties, self.profile = full_name, kwargs.get('rosname'), kwargs.get('namespace'), kwargs.get('executable'), kwargs.get('enclave'), kwargs.get('advertise'), kwargs.get('subscribe'), kwargs.get('hpl', {}).get('properties'), profile
        # Process node-package
        self.package = self.index.replace(self.rosname, '') 
        if self.package not in list(map(lambda pkg: pkg.name, Package.PACKAGES)):
            raise svException(f'Package {self.package} defined in node {self.index} not defined.')
        # Constrain topic allowance.
        self.topic_allowance = self.constrain_topics() if self.secure else None
        # GET from Pickle classes.
        if svROSNode.LOADED_NODES: self.remaps = svROSNode.load_remaps(node_name=self.index)
        # LOAD properties.
        if self.properties: self.properties = svROSNode.parse_hpl_properties(properties=self.properties)
        # Store in class variable.
        svROSNode.NODES[self.index] = self

    def constrain_topics(self):
        topic_allowance = {method: set() for method in ['advertise', 'subscribe']}
        profile         = self.profile
        namespace, allow_subscribe, allow_advertise = profile.namespace, profile.subscribe, profile.advertise
        # Advertise.
        if allow_advertise:
            if self.advertise:
                for adv in self.advertise:
                    name, adv  = adv, adv.replace(namespace, '').strip()
                    if adv in allow_advertise:
                        topic_allowance['advertise'].add(name)
        else: topic_allowance['advertise'].clear()
        # Subscribe
        if allow_subscribe:
            if self.subscribe:
                for sub in self.subscribe:
                    name, sub  = sub, sub.replace(namespace, '').strip()
                    if sub in allow_subscribe:
                        topic_allowance['subscribe'].add(name)
        else: topic_allowance['subscribe'].clear()
        return topic_allowance

    @classmethod
    def load_remaps(cls, node_name):
        if node_name in cls.LOADED_NODES:
            node = cls.LOADED_NODES[node_name]
            return node.remaps
        else: return list()

    @staticmethod
    def parse_hpl_properties(properties):
        pass

    def get_enclave_profile(self):
        if self.enclave == '' or self.enclave is None:
            return None
        index = self.enclave + self.rosname
        if not index in svROSProfile.PROFILES:
            raise svException('Enclave Profile not found.')
        return svROSProfile.PROFILES[index]

    def update_node_with_profile(self):
        pass

    @staticmethod
    def topic_handler(topics):
        topic_list = []
        for topic in topics:
            topic = Topic(name=topic, topic_type=topics[topic])
            topic_list.append(topic)
        return topic_list

    # This method will allow to check what the output might be when an unsecured enclave publishes something from one of its topics
    @classmethod
    def obsDetfromEnclave (cls, enclave: str):
        if enclave not in svROSEnclave.ENCLAVES:
            raise svException("Unsecured enclave not found.")
        pass
    
    @property
    def secure(self):
        if self.profile is None: return False
        else: return True

""" 
    The remaining classes also help to check the SROS structure within Alloy. Some methods allow svROS to retrieve data into Alloy already-made model.
"""
"SROS2-based Enclave with associated profiles."
class svROSEnclave(object):
    ENCLAVES = {}
    """
        svROSEnclave
            \_ path
            \_ profiles
    """
    def __init__(self, path, profiles):
        self.name, self.profiles, self.signature = path, {}, self.abstract(tag=path)
        for profile in profiles:
            p = svROSProfile.init_profile(profile, enclave=self)
            self.profiles[p.profile] = p
        svROSEnclave.ENCLAVES[self.name] = self

    def abstract(self, tag): return tag.lower().replace('/', '_')

    def __str__(self):
        profiles = None if (self.profiles == {}) else ' + '.join(list(map(lambda profile: 'profile' + self.abstract(tag=profile), self.profiles)))
        if not profiles: profiles = "no profiles"
        else:            profiles = f"profiles = {profiles}"
        return f"""one sig enclave{self.signature} extends Enclave {{}} {{{profiles}}}\n"""

"SROS2-based Profile with associated priveleges."
class svROSProfile(object):
    PROFILES = {}
    """
        SROSProfile
            \_ Associated with priveleges
            \_ Later associated with a svROSNode
    """
    def __init__(self, name, namespace, can_advertise, can_subscribe, enclave):
        self.name, self.namespace, self.enclave = name, namespace, enclave
        self.privileges = dict()
        self.advertise, self.subscribe = can_advertise, can_subscribe
        svROSProfile.PROFILES[self.index] = self

    @classmethod
    def init_profile(cls, profile, enclave):
        namespace, name, topics = profile.get('ns'), profile.get('node'), profile.findall('./topics')
        allow_publish   = list(filter(lambda topic: topic.get('publish').strip()=='ALLOW' , list(filter(lambda t: t.get('publish'), topics))))
        allow_subscribe = list(filter(lambda topic: topic.get('subscribe').strip()=='ALLOW' , list(filter(lambda t: t.get('subscribe'), topics))))
        deny_publish    = list(filter(lambda topic: topic.get('publish').strip()=='DENY' , list(filter(lambda t: t.get('publish'), topics))))
        deny_subscribe  = list(filter(lambda topic: topic.get('subscribe').strip()=='DENY' , list(filter(lambda t: t.get('subscribe'), topics))))
        # Process ALLOWS.
        if allow_publish: advertise = list(map(lambda pub: pub.text, allow_publish[0].findall('./topic')))
        else: advertise = None
        if allow_subscribe: subscribe = list(map(lambda sub: sub.text, allow_subscribe[0].findall('./topic')))
        else: subscribe = None
        # Process DENYS.
        if deny_publish: 
            deny_publish = list(map(lambda pub: pub.text, allow_publish[0].findall('./topic')))
            for deny in deny_publish:
                if deny in advertise: raise svException(f'Failed to load profile since privilege {deny} is either defiend as ALLOW and DENY.')
        if deny_subscribe: 
            deny_subscribe = list(map(lambda sub: sub.text, allow_subscribe[0].findall('./topic')))
            for deny in deny_subscribe:
                if deny in subscribe: raise svException(f'Failed to load profile since privilege {deny} is either defiend as ALLOW and DENY.')
        # Return instance created.
        return cls(name=name, namespace=namespace, can_advertise=advertise, can_subscribe=subscribe, enclave=enclave)

    @property
    def profile(self):
        return self.namespace + self.name
    
    @property
    def index(self):
        return self.enclave.name + self.profile

    @property
    def node(self):
        return self._node

    # NEEDED FOR ALLOY DEFINITION
    @node.setter
    def node(self, node):
        if not isinstance(node, svROSNode):
            raise svException('Failed to load node into profile.')
        self.signature, self.privileges, self.access  = self.abstract(tag=node.rosname), [], []
        rosname = self.signature
        # Process topic access.
        if node.advertise:
            for adv in node.advertise: 
                privilege = svROSPrivilege.init_privilege(node=rosname, role='advertise', rosname=adv,method='access', len=len(self.access))
                if privilege not in self.access: self.access.append(privilege)
        if node.subscribe:
            for sub in node.subscribe: 
                privilege = svROSPrivilege.init_privilege(node=rosname, role='subscribe', rosname=adv,method='access', len=len(self.access))
                if privilege not in self.access: self.access.append(privilege)
        # Process topic privilege
        if node.topic_allowance.get('advertise', {}):
            for adv in node.topic_allowance.get('advertise', {}):
                privilege = svROSPrivilege.init_privilege(node=rosname, role='advertise', rosname=adv,method='privilege', len=len(self.privileges))
                if privilege not in self.privileges: self.privileges.append(privilege)
        if node.topic_allowance.get('subscribe', {}):
            for sub in node.topic_allowance.get('subscribe', {}): 
                privilege = svROSPrivilege.init_privilege(node=rosname, role='subscribe', rosname=adv,method='privilege', len=len(self.privileges))
                if privilege not in self.privileges: self.privileges.append(privilege)

    def abstract(self, tag): return tag.lower().replace('/', '_')

    def profile_declaration(self):
        privileges = None if (self.privileges == []) else ' + '.join(list(map(lambda p: p.signature, self.privileges)))
        access     = None if (self.access == [])     else ' + '.join(list(map(lambda p: p.signature, self.access)))
        if not privileges: privileges = "no privileges"
        else:              privileges = f"privileges = {privileges}"
        if not access:     access     = "no access"
        else:              access     = f"access = {access}"
        return f"""one sig profile{self.signature} extends Profile {{}} {{{privileges}\n{access}}}\n"""

    def privilege_declaration(self):
        _str_return_ = ""
        privileges   = list(set(self.privileges + self.access))
        for privilege in privileges: _str_return_ += str(privilege)
        return _str_return_

    def __str__(self):
        return self.profile_declaration() + self.privilege_declaration()

class svROSObject(object):
    OBJECTS = {}
    def __init__(self, name):
        self.name = name
        svROSObject.OBJECTS[name] = self

    @classmethod
    def init_object(cls, name):
        if name in cls.OBJECTS: return cls.OBJECTS[name]
        return cls(name=name)
    
    def __str__(self):
        return f'one sig {self.name} extends Object {{}}\n'

class svROSPrivilege(object):
    PRIVILEGES       = {'Advertise', 'Subscribe'}
    METHODS          = {'Privilege', 'Access'}
    PRIVILEGES_SET   = {}
    OBJECTS_DECLARED = set()
    def __init__(self, index, signature, role, rosname):
        self.signature = self.abstract(tag=signature)
        self.role      = role.capitalize()
        if not self.role in svROSPrivilege.PRIVILEGES: raise svException('Not identified role.')
        self.object                          = svROSObject.init_object(name=self.abstract(tag=rosname))
        svROSPrivilege.PRIVILEGES_SET[index] = self

    @classmethod
    def init_privilege(cls, node, role, rosname, method, len):
        if not role.capitalize() in svROSPrivilege.PRIVILEGES: raise svException('Not identified role.')
        if not method.capitalize() in svROSPrivilege.METHODS:  raise svException('Not identified method.')
        index = node + rosname
        if index in cls.PRIVILEGES_SET:
            return cls.PRIVILEGES_SET[index]
        else:
            return cls(index=index, signature=f'{rosname}_{method}_{len}', role=role, rosname=rosname)

    def abstract(self, tag): return tag.lower().replace('/', '_')

    def __str__(self):
        _str_ = f"""one sig {self.signature} extends Privilege {{}} {{role = {self.role}\nobject = {self.object.name}}}\n""" 
        if not self.object in svROSPrivilege.OBJECTS_DECLARED:
            _str_ += str(self.object)
            svROSPrivilege.OBJECTS_DECLARED.add(self.object)
        return _str_