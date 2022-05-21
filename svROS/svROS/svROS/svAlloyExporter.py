import os, argparse, time, shutil, glob, warnings, logging, re, sys, subprocess, yaml
from dataclasses import dataclass, field
from logging import FileHandler
from typing import ClassVar
# InfoHandler => Prints, Exceptions and Warnings
from tools.InfoHandler import color, svException
# HPL Parser
from hpl.parser import HplParser, parse_property, parse_specification, parse_predicate, HplPredicate, HplEventDisjunction, HplSimpleEvent, HplBinaryOperator
# svROS 
from svData import svROSNode

global WORKDIR, SCHEMAS
WORKDIR = os.path.dirname(__file__)
SCHEMAS = os.path.join(WORKDIR, '../schemas/')

""" 
    This file contains the necessary classes and methods to parse HPL-based properties back to classes to then retrieve to Alloy.
"""
@dataclass
class svHPLParser(object):
    node      : svROSNode
    properties: list = field(default_factory=list)
    