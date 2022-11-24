# Copyright © 2022 Luís Ribeiro
# Grammar for Assumptions.

from .svInfo import svException, svWarning
GRAMMAR = f"""
    property : pattern

    pattern  : "requires"  formula 

    formula      : conditional
    conditional  : [ conditional AND_OPERATOR ] conjunction 
    conjunction  : [ conjunction OR_OPERATOR ] condition
    condition    : [ NO_OPERATOR ] cond

    cond         : STATE evaluate
    evaluate     : binop ( VALUE | STATE )

    binop       : EQUAL_OPERATOR | DIFF_OPERATOR | GREATER_OPERATOR | LESSER_OPERATOR | GT_EQ | LS_EQ

    AND_OPERATOR    : "and" | "&&"
    OR_OPERATOR     : "or"  | "||"
    NO_OPERATOR     : "no" | "not"
    
    EQUAL_OPERATOR   : "="
    GREATER_OPERATOR : ">"
    GT_EQ            : ">="
    LESSER_OPERATOR  : "<"
    LS_EQ            : "<="
    DIFF_OPERATOR    : "!="

    VALUE     : /(?!=>)[a-zA-Z0-9_\/\-.\:]+/
    STATE     : /(?!\s)\$[a-zA-Z0-9_\/\-.\:]+/

    %import common.WS
    %ignore WS
"""

from lark import Lark, tree, Token, Transformer
from lark.exceptions import UnexpectedCharacters, UnexpectedToken
from .svData import svState
class GrammarParser(object):
    """
        Grammar Main Parser
    """
    GRAMMAR = f'{GRAMMAR}'

    @classmethod
    def parse(cls, text=''):
        if text == '': return
        grammar, parser = cls.GRAMMAR, Lark(cls.GRAMMAR, parser="lalr", start='property', transformer=LanguageTransformer())
        # PARSE!
        try:
            conditions = parser.parse(text)
        except (UnexpectedToken, UnexpectedCharacters, SyntaxError) as e:
            raise svException(f'Failed to parse property {text}: {e}')
        try:
            return conditions
        except AttributeError: raise svException(f'Failed to parse property {text}: {e}')

ALLOY_OPERATORS = {
    "NO_OPERATOR": "no",
    "SOME_OPERATOR": "some",
    "OR_OPERATOR": "or",
    "AND_OPERATOR": "and",
    "EQUAL_OPERATOR": "=",
    "DIFF_OPERATOR": "!=",
    "GREATER_OPERATOR": "gt",
    "LESSER_OPERATOR": "lt",
    "INC_OPERATOR": "plus",
    "DEC_OPERATOR": "minus",
    "GT_EQ": "gte",
    "LS_EQ": "lte"
}
###############################
# === LANGUAGE TRANSFORMER  ===
###############################

class LanguageTransformer(Transformer):

    def property(self, children):
        return children[0]

    def pattern(self, children):
        return children[0]

    def condition(self, children):
        assumpt = GlobalAssumption(assumption=children[1])
        if children[0] is not None:
            assumpt.deny = True
        return assumpt

    def binop(self, children):
        return children[0].type

    def evaluate(self, children):
        return Evaluate(binop=children[0], value=children[1])

    def cond(self, children):
        return Variable(entity=(children[0].value)[1:], evaluate=children[1])

    def formula(self, children):
        return Disjunction(conditions=self.conditionals)

    def conditional(self, children):
        disjunction, conjunctions = None, Conjunction(conditions=self.conj)
        if children[1] is None:
            self.conditionals = [conjunctions]
        else:
            self.conditionals += [conjunctions]

    def conjunction(self, children):
        if children[1] is None: 
            self.conj  = [children[2]]
        else: 
            self.conj += [children[2]]

###############################
# === PARSER FROM TRANSFMER ===
###############################
class Disjunction(object):
    
    def __init__(self, conditions):
        self.conditions = conditions
    
    def __alloy__(self):
        return '(' + ' and '.join([f'{cond.__alloy__()}' for cond in self.conditions]) + ')'

class Conjunction(object):

    def __init__(self, conditions):
        self.conditions = conditions
    
    def __alloy__(self):
        return '(' + ' or '.join([f'{cond.__alloy__()}' for cond in self.conditions]) + ')'

class GlobalAssumption(object):
    
    def __init__(self, assumption):
        self.assumption, self.deny = assumption, False

    def __alloy__(self):
        return self.assumption.__alloy__(no_quantifier=self.deny)

class TraceAssumption(object):
    
    def __init__(self, trace, assumption):
        self.trace, self.assumption, self.deny = trace, assumption, False

    def __alloy__(self):
        return self.assumption.__alloy__(trace=trace, no_quantifier=self.deny)

class Evaluate(object):
    
    def __init__(self, binop, value):
        value = value.value if value.type == "VALUE" else f"t.{value.value[1:]}"
        self.binop, self.value = binop, value

    def __alloy__(self, entity):
        isint = True if entity.isint else False
        if isint and not self.value.lstrip("-").isdigit():
            raise svException(f"Variable value is not a number but variable is numeric!")
        entity.values.add(self.value)
        return self.operation(binop=self.binop, signature=f"T1.{entity.name.lower()}", value=self.value, isint=isint) + " and " + self.operation(binop=self.binop, signature=f"T2.{entity.name.lower()}", value=self.value, isint=isint)

    @staticmethod
    def operation(binop, signature, value, isint, prefix="", sufix="", prev=""):
        if binop in {"EQUAL_OPERATOR", "DIFF_OPERATOR"}:
            return f"{prefix} {signature} {ALLOY_OPERATORS[binop]} {value} {sufix}"
        assert isint
        if binop in {"INC_OPERATOR", "DEC_OPERATOR"}:
            if prev:
                return f"{signature} = {ALLOY_OPERATORS[binop]}[{prev}, {value}]"
        return f"{prefix} {ALLOY_OPERATORS[binop]}[{signature}, {value}] {sufix}"

class Variable(object):

    def __init__(self, entity, evaluate):
        try:
            state = svState.STATES[entity]
        except AttributeError as e : raise svException(f'{e}')
        svState.ASSUMPTIONS.add(state)
        self.entity, self.object, self.evaluate = entity, state, evaluate

    def __alloy__(self, trace=None, no_quantifier=False):
        # CONDITION
        quantifier, trace = 'no'        if no_quantifier else '', f'{trace}.' if trace         else ''
        return f"{quantifier} {self.evaluate.__alloy__(entity=self.object)}"

class State(object):

    def __init__(self, entity, evaluate):
        try:
            state = svState.STATES[entity]
        except AttributeError as e : raise svException(f'{e}')
        svState.ASSUMPTIONS.add(state)
        self.entity, self.object, self.evaluate = entity, state, evaluate

    def __alloy__(self, trace=None, no_quantifier=False):
        # CONDITION
        quantifier = 'no'        if no_quantifier else ''
        trace      = f'{trace}.' if trace         else ''
        if self.relation == 'EQUAL_OPERATOR': 
                return f"{quantifier} {trace}{self.object.name.lower()} = {self.value}"
        assert self.object.isint
        return f"{quantifier} {ALLOY_OPERATORS[self.relation]}[{trace}{self.object.name.lower()}, {self.value}]"