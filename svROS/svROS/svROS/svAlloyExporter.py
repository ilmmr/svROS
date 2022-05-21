import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, yaml
from dataclasses import dataclass, field
from logging import FileHandler
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException
# HPL Parser
from hpl.parser import HplParser, parse_property, parse_specification, parse_predicate, HplPredicate, HplEventDisjunction, HplSimpleEvent, HplBinaryOperator
# svROS 
from svData import svROSNode, svROSEnclave

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

""" 
    This file contains the necessary classes and methods to parse HPL-based properties back to classes to then retrieve to Alloy.
"""
class svAlloyObject(object):
    def __init__(self, name):
        self.name = name
    
    def __str__(self):
        return f'one sig {self.name} extends Object {{}}\n'

class svAlloyPrivilege(object):
    PRIVILEGES = {'Advertise', 'Subscribe'}
    def __init__(self, signature, role, rosname):
        self.signature = self.abstract(tag=signature)
        self.role      = role.capitalize()
        if not self.role in svAlloyPrivilege.PRIVILEGES: raise svException('Not identified role.')
        self.object    = svAlloyObject(name=self.abstract(tag=rosname))

    def abstract(self, tag): return tag.lower().replace('/', '_')

    def __str__(self):
        return f"""one sig {self.signature} extends Privilege {{}} {{role = {self.role}, object = {self.object.name}, value = ALLOW}}\n""" + str(self.object)

@dataclass
class svAlloyProfile(object):
    NODE : svROSNode

    def __post_init__(self):
        rosname, self.signature, node = self.NODE.rosname, self.abstract(tag=self.NODE.rosname), self.NODE
        self.privileges, self.access  = [], []
        # Process topic access.
        if node.advertise:
            for adv in node.advertise: self.access.append(svAlloyPrivilege(signature=f'{rosname}_access_{len(self.access)}', role='advertise', rosname=adv))
        if node.subscribe:
            for sub in node.subscribe: self.access.append(svAlloyPrivilege(signature=f'{rosname}_access_{len(self.access)}', role='subscribe', rosname=sub))
        # Process topic privilege.
        if node.topic_allowance.get('advertise', {}):
            for adv in node.topic_allowance.get('advertise', {}): self.privileges.append(svAlloyPrivilege(signature=f'{rosname}_privilege_{len(self.privileges)}', role='advertise', rosname=adv))
        if node.topic_allowance.get('subscribe', {}):
            for sub in node.topic_allowance.get('subscribe', {}): self.privileges.append(svAlloyPrivilege(signature=f'{rosname}_privilege_{len(self.privileges)}', role='subscribe', rosname=sub))

    def abstract(self, tag): return tag.lower().replace('/', '_')

    def profile_declaration(self):
        privileges = "none" if (self.privileges == []) else ' + '.join(list(map(lambda p: p.signature, self.privileges)))
        access     = "none" if (self.access == [])     else ' + '.join(list(map(lambda p: p.signature, self.access)))
        return f"""one sig {self.signature} extends Profile {{}} {{privileges = {privileges}, access = {access}}}\n"""

    def privilege_declaration(self):
        _str_return_ = ""
        for privilege in self.privileges: _str_return_ += str(privilege)
        for access    in self.access:     _str_return_ += str(access)
        return _str_return_

    def __str__(self):
        return self.profile_declaration() + self.privilege_declaration()

@dataclass
class svAlloyEnclave(object):
    ENCLAVE : svROSEnclave

    def __post_init__(self):
        rosname, self.signature = self.ENCLAVE.name, self.abstract(tag=self.ENCLAVE.name)

    def abstract(self, tag): return tag.lower().replace('/', '_')

    def __str__(self):
        profiles = "none" if (self.ENCLAVE.profiles == {}) else ' + '.join(list(map(lambda profile: self.abstract(tag=profile), self.ENCLAVE.profiles)))
        return f"""one sig {self.signature} extends Enclave {{}} {{profiles = {profiles}}}\n"""

@dataclass
class svHPLParser(object):
    node      : svROSNode
    properties: list = field(default_factory=list)
    