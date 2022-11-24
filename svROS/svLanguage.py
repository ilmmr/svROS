import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess
from yaml import *
from dataclasses import dataclass, field
from logging import FileHandler
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from .svInfo import color, svException, svWarning, svInfo
# Parsers
import xml.etree.ElementTree as ET
from lark import Lark, tree
import itertools

from .svGrammar import GrammarParser, Read, Publish, Update
from .svData import Topic, svNode, svState

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
            return svAlloyPredicate.frame_conditions()
        else:
            return '\n\t' + '\n\t'.join([re.sub(r'[ ]+',' ',p.str) for p in properties]) + svAlloyPredicate.frame_conditions(channels=changable_channels, variables=changable_variables)

    @classmethod
    def parse_only_properties(cls, node, properties):
        return '\n\t' + '\n\t'.join([re.sub(r'[ ]+',' ',p.str) for p in properties])
    
    @staticmethod
    def frame_conditions(channels=None, variables=None):
        _str_ = f"""\n\t// Frame Conditions:"""
        # channels
        if channels is None: _str_ += f"""\n\tt.inbox' = t.inbox"""
        else:
            _str_ += f"""\n\tall c : Topic - {' - '.join([c.signature for c in channels])} | t.inbox'[c] = t.inbox[c]"""
        if variables is None: variables = svState.STATES.values()
        else: variables = list(filter(lambda state: state not in variables, svState.STATES.values()))
        for state in variables: 
            # _str_ += f"""\n\tsome t.{state.name.lower()}.1 implies t.{state.name.lower()}' = (t.{state.name.lower()}.1)->0 else t.{state.name.lower()}' = t.{state.name.lower()}"""
            _str_ += f"""\n\tt.{state.name.lower()}' = t.{state.name.lower()}"""
        return _str_

class svPredicate(object):
    """
        Overall Node behaviour => svPredicate
    """
    NODE_BEHAVIOURS = {}
    def __init__(self, signature, node, properties, is_sub_predicate=False):
        if not isinstance(node, svNode):
            raise svException('Failed to create property parser since given node is not a Node.')
        signature = svPredicate.signature(value=signature)
        self.signature, self.node, self.sub_predicates = signature, node, set()
        # CHANGABLE
        self.changable_channels  = []
        self.changable_variables = []
        if properties: 
            properties = list(map(lambda prop: self.create_prop(text=prop), properties))
            properties = list(filter(lambda prop: not isinstance(prop, svPredicate), properties))
            self.properties = None if properties == [] else properties 
        else: 
            self.properties = None
        # REGARD TOP-LEVEL SIG.
        self.is_sub_predicate = is_sub_predicate
        svPredicate.NODE_BEHAVIOURS[signature] = self

    @staticmethod
    def signature(value):
        return str(value).lower().replace('/','_')

    @classmethod
    def parse_into_alloy(cls):
        for predicate in cls.NODE_BEHAVIOURS.values():
            predicate.behaviour = predicate.parse_predicate()
        return True

    def parse_predicate(self):
        changable_channels  = list(set(self.changable_channels))
        changable_variables = list(set(self.changable_variables))
        # try:
        #     changable_channels, changable_variables = self.check_accessable(non_accessable=non_accessable)
        #     if changable_channels  == []: changable_channels  = None
        #     if changable_variables == []: changable_variables = None
        # except AttributeError:
        #     raise svException(f"Failed to parse predicate {self.signature}.")
        if not self.sub_predicates == set():
            return svAlloyPredicate.parse_only_properties(node=self.node, properties=self.properties)
        return svAlloyPredicate.parse(node=self.node, properties=self.properties, changable_channels=changable_channels, changable_variables=changable_variables)

    # Method to extract and parse text properties into class properties!
    def create_prop(self, text):
        try: 
            # Dict means that is another predicate.
            if isinstance(text, dict):
                if list(text.keys()).__len__() != 1: raise svException(f'Failed to parse property {text}.')
                signature, properties, node  = str(self.signature + '_' + list(text.keys())[0]).strip(), text[list(text.keys())[0]], self.node
                sub_predicate = svPredicate.init_predicate(signature, node, properties, is_sub_predicate=True)
                if sub_predicate in self.sub_predicates: raise svException(f'Sub-Predicate {signature} of predicate {self.signature} already specified.')
                self.sub_predicates.add(sub_predicate)
                return sub_predicate
            elif isinstance(text, str):
                property = GrammarParser.parse(text=text, node=self)
                return property
            else:
                print(text)
                raise svException(f'Failed to parse property {text}.')
        except Exception: raise svException(f'Failed to parse property {text}.')

    @classmethod
    def init_predicate(cls, signature, node, properties, is_sub_predicate=False):
        if properties.__len__() == 1 and properties[0] == '': 
            properties = None
        if signature in cls.NODE_BEHAVIOURS:
            raise svException(f"Predicate {signature} is already defined.")
        return cls(signature=signature, node=node, properties=properties, is_sub_predicate=is_sub_predicate)

    @classmethod
    def node_behaviour(cls):
        return '\n'.join([str(predicate) for predicate in cls.NODE_BEHAVIOURS.values()])

    def __subpredicates__(self):
        if self.sub_predicates.__len__() == 0: return ''
        return  '\n\t// Sub-Predicates:\n\t' + ' or '.join([f'{s.signature}[t]' for s in self.sub_predicates])

    def __str__(self):
        return re.sub('\n\n', '\n', f"""pred {self.signature} [t : Trace] {{{self.behaviour}\n{self.__subpredicates__()}\n}}""")