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
import itertools

from language.grammar import GrammarParser, Read, Publish, Alter, ReadConditional, ReadConsequence
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
    def __init__(self, signature, node, properties, parent=None):
        if not isinstance(node, svROSNode):
            raise svException('Failed to create property parser since given node is not a Node.')
        self.signature, self.node, self.sub_predicates, self.parent = signature, node, set(), parent
        if properties: self.properties = list(map(lambda prop: self.create_prop(text=prop), properties))
        else: self.properties = None
        self.behaviour = self.parse_predicate()
        # Only store top-level predicates.
        if parent == None: svPredicate.NODE_BEHAVIOURS[signature] = self

    def parse_predicate(self):
        # Building non-accessable!
        node_access = list(self.node.advertise) if self.node.advertise is not None else []
        node_access += list(self.node.subscribe) if self.node.subscribe is not None else []
        non_accessable = list(filter(lambda t: t not in node_access, Topic.TOPICS.values()))
        try:
            changable_channels, changable_variables = self.check_accessable(non_accessable=non_accessable)
        except AttributeError:
            raise svException(f"Failed to parse predicate {self.signature}.")
        # parser         = svProperty.parse(node=self.node, properties=self.properties, non_accessable=non_accessable)
        # if no properties were conceived.
        # print(self.properties)
        # print(svAlloyPredicate.parse(node=self.node, properties=None, pre_condition=None, changable_channels=None, changable_variables=None))

    # Method that either checks node access capacities as it outputs the non frame conditions under such node.
    def check_accessable(self, non_accessable=None):
        changable_variables, chagable_channels = list(), list()
        for prop in self.properties:
            # READ
            if isinstance(prop, Read):
                entity = Topic.TOPICS[prop.entity]
                if entity in non_accessable: raise svException(f"Predicate under node {self.node.rosname} can not access read object {prop.entity}.")
                chagable_channels.append(entity)
                for condition in list(itertools.chain(*prop.conditions)):
                    if isinstance(condition, ReadConditional):
                        if condition.consequence is None: continue
                        else: condition = condition.consequence
                    elif isinstance(condition, ReadConsequence): pass
                    else: raise svException("ERROR")
                    # PARSE TYPE
                    if condition.type == "TOPIC":
                        entity = Topic.TOPICS[condition.entity]
                        if entity in non_accessable: raise svException(f"Predicate under node {self.node.rosname} can not access read object {condition.entity}.")
                        chagable_channels.append(entity)
                    else:
                        state = svState.STATES[condition.entity]
                        changable_variables.append(state)
            # PUBLISH        
            if isinstance(prop, Publish):
                entity = Topic.TOPICS[prop.entity]
                if entity in non_accessable: raise svException(f"Predicate under node {self.node.rosname} can not access publish object {prop.entity}.")
                chagable_channels.append(entity)
            # ALTER
            if isinstance(prop, Alter):
                state = svState.STATES[prop.entity]
                changable_variables.append(state)     
        # Return accessable channels and variables!
        return chagable_channels, changable_variables

    # Method to extract and parse text properties into class properties!
    def create_prop(self, text):
        try: 
            # Dict means that is another predicate.
            if isinstance(text, dict):
                if list(text.keys()).__len__() != 1: raise svException(f'Failed to parse property {text}.')
                signature, properties, node  = str(self.signature + '_' + list(text.keys())[0]).strip(), text[list(text.keys())[0]], self.node
                sub_predicate = svPredicate.init_predicate(signature, node, properties, parent=self)
                if sub_predicate in self.sub_predicates: raise svException(f'Sub-Predicate {signature} of predicate {self.signature} already specified.')
                self.sub_predicates.add(sub_predicate)
                return sub_predicate
            elif isinstance(text, str):
                property = GrammarParser.parse(text=text)
                return property
            else:
                raise svException(f'Failed to parse property {text}.')
        except Exception: raise svException(f'Failed to parse property {text}.')

    @classmethod
    def init_predicate(cls, signature, node, properties, parent=None):
        if properties.__len__() == 1 and properties[0] == '': 
            properties = None
        if parent is not None: return cls(signature=signature, node=node, properties=properties, parent=parent)
        # Not a subpredicate!
        if signature in cls.NODE_BEHAVIOURS:
            svprop = cls.NODE_BEHAVIOURS[signature]
            svprop.properties = properties
            return svprop
        elif node.predicate is not None:
            raise svException(f"Node {node.rosname} has two different predicates mentioned. Please remove 1!")
        return cls(signature=signature, node=node, properties=properties, parent=parent)