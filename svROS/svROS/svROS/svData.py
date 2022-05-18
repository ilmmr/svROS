import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException
# XML Parser
import xml.etree.ElementTree as ET

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

"ROS2-based Topic already parse for node handling."
class Topic(object):
    TOPICS = set()
    """
        Topic
            \__ Name
            \__ Type
    """
    def __init__(self, name, topic_type):
        self.name, self.type = name, topic_type
        Topic.TOPICS.add(self)

    @classmethod
    def init_topic(cls, **kwargs):
        return cls(name=kwargs['name'], type=kwargs['topic_type'])

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

    @classmethod
    def process_config_file(cls):
        # Returning object.
        ret_object = {}
        for index in cls.NODES:
            node  = cls.NODES[index]
            index = index.replace('::', '/')
            ret_object[index]               = {}
            ret_object[index]['rosname']    = node.rosname
            ret_object[index]['executable'] = node.executable
            ret_object[index]['enclave']    = node.enclave if node.enclave else Node.namespace(tag=index)
            # Topic treatment.
            if node.source.publishes:  ret_object[index]['advertise'] = {}
            if node.source.subscribes: ret_object[index]['subscribe'] = {}
            if not node.source:
                continue
            for adv in node.source.publishes:
                topic_type = adv.type
                adv        = Node.render_remap(topic=adv.name, remaps=node.remaps)
                ret_object[index]['advertise'][adv] = topic_type
            for sub in node.source.subscribes:
                topic_type = sub.type
                sub        = Node.render_remap(topic=sub.name, remaps=node.remaps)
                ret_object[index]['subscribe'][sub] = topic_type
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
                enclave.set('path', Node.namespace(tag=node.index).replace('::', '/'))
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
            for adv in node.source.publishes:
                topic = ET.Element('topic')
                adv        = Node.render_remap(topic=adv.name, remaps=node.remaps)
                topic.text = str(adv)
                pubs.append(topic)
            # Process Topic Subscribe
            subs = ET.Element('topics')
            subs.set('subscribe', 'ALLOW')
            for sub in node.source.subscribes:
                topic = ET.Element('topic')
                sub        = Node.render_remap(topic=sub.name, remaps=node.remaps)
                topic.text = str(sub)
                subs.append(topic)
            # Processing Profile
            if subs.findall('./') != []: profile.append(subs)
            if pubs.findall('./') != []: profile.append(pubs)
            profiles.append(profile)
        # Retrieve XML
        return template

    @staticmethod
    def retrieve_sros_default_tag(template):
        default_tag = '<xi:include href="path/to/common/node.xml" xpointer="xpointer(/profile/*)"/>'
        default_    = ET.Element('{http://www.w3.org/2001/XInclude}include')
        default_.set('href', f'{SCHEMAS}sros/common/node.xml')
        default_.set('xpointer', 'xpointer(/profile/*)')
        profiles = template.findall(f'.//profile')
        for profile in profiles:
            profile.append(default_)
        return template

    # Retrieve JSON-based dict information
    @staticmethod
    def to_json(node: str):
        node   = Node.NODES[node]
        # Enclave is not needed at this point
        topics = {'subscribe': list(map(lambda subs: subs.name, node.source.subscribes)), 'advertise': list(map(lambda pubs: pubs.name, node.source.publishes)), 'remaps': node.remaps}
        return {'package': node.package, 'executable': node.executable, 'rosname': node.rosname, 'topics': topics}

    @staticmethod
    def render_remap(topic, remaps):
        for r in remaps:
            if topic.strip() == r['to'].strip():
                topic = r['to'].strip()
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
        return rsn_

    @property
    def index(self):
        if self.namespace: index_ = self.package + '::' + self.namespace + '/' + self.name
        else: index_ = self.package + '::' + self.name
        return index_

""" 
    The remaining of this file contains the necessary classes and methods to retrieve and use information for analysis purposes. 
"""
"ROS2-based Node to be analyzed."
class svROSNode(object):
    NODES = {}
    """
        svROSNode
            \__ Already parsed node
            \__ Associated with Profile from SROS (can either be secured or unsecured)
    """
    def __init__(self, **kwargs):
        # self            = super().init_node(**kwargs)
        self.advertises, self.subscribes, self.profile = self.get_enclave_profile()
        self.priveleges = self.update_node_with_profile()

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
    def is_secure(self):
        if self.profile is None: return False
        else: return True

"SROS2-based Enclave with associated profiles."
class svROSEnclave(object):
    ENCLAVES = set()
    """
        svROSEnclave
            \_ path
            \_ profiles
    """
    def __init__(self, path, profiles):
        self.name, self.profiles = path, []
        for profile in profiles:
            profile         = svROSProfile.init_profile(profile)
            profile.enclave = self
            self.profiles.append(profile)
        svROSEnclave.add(self)

"SROS2-based Profile with associated priveleges."
class svROSProfile(object):
    PROFILES = {}
    """
        SROSProfile
            \_ Associated with priveleges
            \_ Later associated with a svROSNode
    """
    def __init__(self, name, namespace, can_advertise, can_subscribe):
        self.name, self.namespace = name, namespace
        self.privileges = dict()
        self.privileges['advertise'], self.privileges['subscribe'] = can_advertise, can_subscribe
        svROSProfile.PROFILES[self.index] = self

    @classmethod
    def init_profile(cls, profile):
        pass

    @property
    def profile(self):
        return self.namespace + self.name
    
    @property
    def index(self):
        return self.enclave.name + self.profile
    
    @property
    def enclave(self):
        return self._enclave
    
    @enclave.setter
    def enclave(self, enclave):
        self._enclave = enclave