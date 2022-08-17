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
    def parse(cls, node, properties, changable_channels, changable_variables):
        if properties is None:
            return svAlloyPredicate.frame_conditions(nop=True)
        else:
            return '\n\t' + '\n\t'.join([re.sub(r'\s{2,}', ' ', p.__alloy__()) for p in properties]) + svAlloyPredicate.frame_conditions(nop=False, channels=changable_channels, variables=changable_variables)

    @staticmethod
    def frame_conditions(nop, channels=None, variables=None):
        if not isinstance(nop, bool): svException("Non expected error occurred.")
        if nop is True: return f"""\n\t//Frame Conditions\n\tnop[t]"""
        else:
            _str_ = f"""\n\t//Frame Conditions"""
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
        signature = svPredicate.signature(value=signature)
        self.signature, self.node, self.sub_predicates = signature, node, set()
        if properties: 
            properties = list(map(lambda prop: self.create_prop(text=prop), properties))
            self.properties = list(filter(lambda prop: not isinstance(prop, svPredicate), properties))
        else: 
            self.properties = None
        self.behaviour = self.parse_predicate()
        # Only store top-level predicates.
        svPredicate.NODE_BEHAVIOURS[signature] = self

    @staticmethod
    def signature(value):
        return str(value).lower().replace('/','_')

    def parse_predicate(self):
        # Building non-accessable!
        node_access = list(self.node.advertise) if self.node.advertise is not None else []
        node_access += list(self.node.subscribe) if self.node.subscribe is not None else []
        non_accessable = list(filter(lambda t: t not in node_access, Topic.TOPICS.values()))
        try:
            changable_channels, changable_variables = self.check_accessable(non_accessable=non_accessable)
            if changable_channels  == []: changable_channels  = None
            if changable_variables == []: changable_variables = None
        except AttributeError:
            raise svException(f"Failed to parse predicate {self.signature}.")
        # parser         = svProperty.parse(node=self.node, properties=self.properties, non_accessable=non_accessable)
        # if no properties were conceived.
        # print(self.properties)
        alloy = svAlloyPredicate.parse(node=self.node, properties=self.properties, changable_channels=changable_channels, changable_variables=changable_variables)
        print(alloy)
        return alloy

    # Method that either checks node access capacities as it outputs the non frame conditions under such node.
    def check_accessable(self, non_accessable=None):
        changable_variables, changable_channels = list(), list()
        if self.properties is None: return changable_channels, changable_variables
        for prop in self.properties:
            # READ
            if isinstance(prop, Read):
                entity = Topic.TOPICS[prop.entity]
                if entity in non_accessable: raise svException(f"Predicate under node {self.node.rosname} can not access read object {prop.entity}.")
                changable_channels.append(entity)
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
                        changable_channels.append(entity)
                    else:
                        state = svState.STATES[condition.entity]
                        changable_variables.append(state)
            # PUBLISH        
            if isinstance(prop, Publish):
                entity = Topic.TOPICS[prop.entity]
                if entity in non_accessable: raise svException(f"Predicate under node {self.node.rosname} can not access publish object {prop.entity}.")
                changable_channels.append(entity)
            # ALTER
            if isinstance(prop, Alter):
                state = svState.STATES[prop.entity]
                changable_variables.append(state)     
        # Return accessable channels and variables!
        return list(set(changable_channels)), list(set(changable_variables))

    # Method to extract and parse text properties into class properties!
    def create_prop(self, text):
        try: 
            # Dict means that is another predicate.
            if isinstance(text, dict):
                if list(text.keys()).__len__() != 1: raise svException(f'Failed to parse property {text}.')
                signature, properties, node  = str(self.signature + '_' + list(text.keys())[0]).strip(), text[list(text.keys())[0]], self.node
                sub_predicate = svPredicate.init_predicate(signature, node, properties)
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
    def init_predicate(cls, signature, node, properties):
        if properties.__len__() == 1 and properties[0] == '': 
            properties = None
        if signature in cls.NODE_BEHAVIOURS:
            raise svException(f"Predicate {signature} is already defined.")
        return cls(signature=signature, node=node, properties=properties)

    def __subpredicates__(self):
        if self.sub_predicates.__len__() == 0: return ''
        return  '\n\t-- SUB-PREDICATES\n\t' + ' or '.join([f'{s.signature}[t]' for s in self.sub_predicates])

    def __str__(self):
        return f"""pred {self.signature} [t : Execution] {{{self.__subpredicates__()}\n{self.behaviour}\n}}"""