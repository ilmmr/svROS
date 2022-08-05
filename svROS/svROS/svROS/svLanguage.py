import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException, svWarning
# Parsers
import xml.etree.ElementTree as ET
from lark import Lark, tree

from svData import Topic, svROSNode, svState
global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

###############################
# === ANALYSING !! YAY :))) ===
###############################
class svAlloyPredicate(object):
    """
        Each svPredicate => Alloy Predicate
    """
    def __init__(pre_condition, changable_variables, changable_channels, properties):
        pass

    @classmethod
    def parse(cls, node, properties, pre_condition, changable_channels, changable_variables):
        pre, properties = '', ''
        if properties is None:
            if pre_condition: pass 
            return pre + svAlloyPredicate.frame_conditions(nop=True)
        else:
            if pre_condition: pass
            return pre + properties + svAlloyPredicate.frame_conditions(nop=False, channels=changable_channels, variables=changable_variables)

    @staticmethod
    def frame_conditions(nop, channels=None, variables=None):
        if not isinstance(nop, bool): svException("!!")
        if nop is True: return f"""\t//Frame Conditions\n\tnop[t]"""
        else:
            _str_ = f"""\t//Frame Conditions"""
            # channels
            if channels is None: _str_ += f"""\n\tt.inbox' = t.inbox"""
            else:
                _str_ += f"""\n\tall c : Channel - {' - '.join([c.signature for c in channels])} | t.inbox'[c] = t.inbox[c]"""
            if variables is None: variables = svState.STATES.values()
            else: variables = list(filter(lambda state: state not in variables, svState.STATES.values()))
            for state in variables: _str_ += f"""\n\tt.{state.name.lower()}' = t.{state.name.lower()}"""
        return _str_

class svProperty(object):
    """
        Each Node behaviour => svProperty
    """

class svPredicate(object):
    """
        Overall Node behaviour => svPredicate
    """
    NODE_BEHAVIOURS = {}
    def __init__(self, signature, node, properties):
        if not isinstance(node, svROSNode):
            raise svException('Failed to create property parser since given node is not a Node.')
        self.signature, self.node, self.properties = signature, node
        if properties: self.properties = list(map(lambda prop: svPredicate.create_prop(text=prop), properties))
        else: self.properties = None
        self.behaviour = self.parse_predicate()
        svPredicate.NODE_BEHAVIOURS[signature] = self

    def parse_predicate(self):
        # Building non-accessable!
        node_access = list(self.node.advertise) if self.node.advertise is not None else []
        node_access += list(self.node.subscribe) if self.node.subscribe is not None else []
        non_accessable = list(filter(lambda t: t not in node_access, Topic.TOPICS.values()))
        parser         = svProperty.parse(node=self.node, properties=self.properties, non_accessable=non_accessable)
        # if no properties were conceived.
        print(svAlloyPredicate.parse(node=self.node, properties=None, pre_condition=None, changable_channels=None, changable_variables=None))

    @staticmethod
    def create_prop(text):
        try: print(text)
        except Exception: raise svException(f'Failed to parse property {text}.')

    @classmethod
    def init_predicate(cls, signature, node, properties):
        if properties.__len__() == 1 and properties.pop() == '': properties = None
        if signature in cls.NODE_BEHAVIOURS:
            svprop = cls.NODE_BEHAVIOURS[signature]
            svprop.properties = properties
            return svprop
        elif node.predicate is not None:
            raise svException(f"Node {node.rosname} has two different predicates mentioned. Please remove 1!")
        else:
            return cls(signature=signature, node=node, properties=properties)