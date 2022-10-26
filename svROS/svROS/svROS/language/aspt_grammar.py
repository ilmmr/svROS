# Copyright © 2022 Luís Ribeiro
# Grammar for svLanguage.

from tools.InfoHandler import svException, svWarning
GRAMMAR = f"""
    property : pattern

    pattern  : "requires"  formula 

    formula      : conditional
    conditional  : [ conditional AND_OPERATOR ] conjunction 
    conjunction  : [ conjunction OR_OPERATOR ] condition
    condition    : [ NO_OPERATOR ] cond

    cond         : STATE ( EQUAL_OPERATOR | DIFF_OPERATOR | GREATER_OPERATOR | LESSER_OPERATOR ) VALUE

    AND_OPERATOR    : "and" | "&&"
    OR_OPERATOR     : "or"  | "||"
    NO_OPERATOR     : "no" | "not"
    
    EQUAL_OPERATOR   : "="
    GREATER_OPERATOR : ">"
    LESSER_OPERATOR  : "<"
    DIFF_OPERATOR    : "!="

    VALUE     : /(?!=>)[a-zA-Z0-9_\/\-.\:]+/
    STATE     : /(?!\s)\$[a-zA-Z0-9_\/\-.\:]+/

    %import common.WS
    %ignore WS
"""

from lark import Lark, tree, Token, Transformer
from lark.exceptions import UnexpectedCharacters, UnexpectedToken
from svData import Topic, svState
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
    "LESSER_OPERATOR": "le",
    "INC_OPERATOR": "plus",
    "DEC_OPERATOR": "minus"
}
        
###############################
# === LANGUAGE TRANSFORMER  ===
###############################

class LanguageTransformer(Transformer):

    def condition(self, children):
        assumpt = GlobalAssumption(assumption=children[1])
        if children[0] is not None:
            assumpt.deny = True
        return assumpt

    def cond(self, children):
        relation, value = children[1].type, children[2].value
        state = State(entity=(children[0].value)[1:], relation=relation, value=value)
        return state

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
        return self.assumption.__alloy__(trace=None, no_quantifier=self.deny)

class TraceAssumption(object):
    
    def __init__(self, trace, assumption):
        self.trace, self.assumption, self.deny = trace, assumption, False

    def __alloy__(self):
        return self.assumption.__alloy__(trace=trace, no_quantifier=self.deny)

class State(object):

    def __init__(self, entity, relation, value):
        try:
            state = svState.STATES[entity]
            if state.isint:
                if not value.lstrip("-").isdigit():
                    raise svException(f"State value is not a number but state is numeric!")
            else:
                state.values.add(value)
        except AttributeError as e : raise svException(f'{e}')
        svState.ASSUMPTIONS.add(state)
        self.entity, self.object, self.relation, self.value = entity, state, relation, value

    def __alloy__(self, trace=None, no_quantifier=False):
        # CONDITION
        quantifier = 'no'        if no_quantifier else ''
        trace      = f'{trace}.' if trace         else ''
        if self.relation == 'EQUAL_OPERATOR': 
                return f"{quantifier} {trace}{self.object.name.lower()} = {self.value}"
        assert self.object.isint
        return f"{quantifier} {ALLOY_OPERATORS[self.relation]}[{trace}{self.object.name.lower()}, {self.value}]"