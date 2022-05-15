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

    def __init__(self, name: str, path: str, nodes: dict):
        self.name  = name
        self.path  = path
        self.nodes = nodes
        Package.PACKAGES.add(self)

"ROS2-based Topic already parse for node handling."
class Topic(object):
    TOPICS = {}
    """
        Topic
            \__ Name
            \__ Type
    """
    def __init__(self, _id, name, topic_type):
        self.id   = _id
        self.name = name
        self.type = topic_type
        Topic.TOPICS[self.id] = self

    @classmethod
    def init_topic(cls, **kwargs):
        return cls(_id=kwargs['_id'], name=kwargs['name'], type=kwargs['topic_type'])

"ROS2-based Node already parse, with remaps and topic handling."
class Node(object):
    NODES               = {}
    """
        Node
            \__ INFO FROM NODETAG OR NODECALL
            \__ Topic subscribing and publishing
    """
    def __init__(self, name, namespace, package, executable, remaps, enclave=None):
        self.name       = name
        self.namespace  = namespace
        self.package    = package
        self.executable = executable
        self.remaps     = remaps
        self.enclave    = enclave
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
            ret_object[index]['enclave']    = node.enclave if node.enclave else ''
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
                enclave.set('path', str(node.rosname))
                profiles = ET.Element('profiles')
                enclave.append(profiles)
                enclaves.append(enclave)
            # Process Node
            profile = ET.Element('profile')
            if node.namespace: profile.set('ns', node.namespace)
            else: profile.set('ns', '')
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

    @staticmethod
    def render_remap(topic, remaps):
        for r in remaps:
            if topic.strip() == r['to'].strip():
                topic = r['to'].strip()
                break
        return topic
    
    @classmethod
    def init_node(cls, **kwargs):
        return cls(name=kwargs['_name'], namespace=kwargs['namespace'], package=kwargs['package'], executable=kwargs['executable'], remaps=kwargs['remaps'], enclave=kwargs.get('enclave'))

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

"ROS2-based Node to be analyzed."
class svROSNode(object):
    NODES               = {}
    """
        svROSNode
            \__ ALREADY PARSED NODE
            \__ PROFILE FROM SROS
    """