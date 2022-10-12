# Copyright © 2022 Luís Ribeiro
# Grammar for svLanguage.

from tools.InfoHandler import svException, svWarning
GRAMMAR = f"""

    property : pattern [ "iff" formula ]

    pattern  : "requires"  formula 
             | event

    event    : "publishes" publishes
             | "alters"    alters
             | "reads"     reads

    formula      : conditional
    conditional  : [ conditional AND_OPERATOR ] conjunction 
    conjunction  : [ conjunction OR_OPERATOR ] condition
    condition    : [ NO_OPERATOR ] cond | event

    reads       : TOPIC [ "as" message_token "::" implication "]]"
    implication : operation ( ";" operation )*
    operation   : "if" formula "then" "{{" formula "}}" [ "else" "{{" formula "}}" ]
                | formula

    publishes   : TOPIC [ EQUAL_OPERATOR ( VALUE | STATE ) ]
    alters      : STATE ( EQUAL_OPERATOR | INC_OPERATOR | DEC_OPERATOR ) VALUE
    
    cond        : publishes
                | alters
                | PREDICATE
                | message_cond

    message_token: MESSAGE
    message_cond : MESSAGE ( EQUAL_OPERATOR | DIFF_OPERATOR | GREATER_OPERATOR | LESSER_OPERATOR ) VALUE

    param       : TOPIC | STATE | PREDICATE | MESSAGE
    binop       : EQUAL_OPERATOR | DIFF_OPERATOR | GREATER_OPERATOR | LESSER_OPERATOR | INC_OPERATOR | DEC_OPERATOR

    AND_OPERATOR    : "and" | "&&"
    OR_OPERATOR     : "or"  | "||"

    NO_OPERATOR     : "no" | "not"
    SOME_OPERATOR   : "some" | "exists"
    
    EQUAL_OPERATOR   : "="
    GREATER_OPERATOR : ">"
    LESSER_OPERATOR  : "<"
    INC_OPERATOR     : "+="
    ADD_OPERATOR     : "++"
    DEC_OPERATOR     : "-="
    DIFF_OPERATOR    : "!="

    MESSAGE   : /(?!\s)[a-z]/
    VALUE     : /(?!=>)[a-zA-Z0-9_\/\-.\:]+/
    TOPIC     : /(?!\s)\/[a-zA-Z0-9_\/\-.\:]+/
    STATE     : /(?!\s)\$[a-zA-Z0-9_\/\-.\:]+/
    PREDICATE : /(?!\s)\?[a-zA-Z0-9_\/\-.\:]+/

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
    def parse(cls, node, text=''):
        if text == '': return
        grammar, parser = cls.GRAMMAR, Lark(cls.GRAMMAR, parser="lalr", start='property', transformer=LanguageTransformer(node=node, text=text))
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

MESSAGE_TOKENS = {}
        
###############################
# === LANGUAGE TRANSFORMER  ===
###############################
class BinaryOperator(object):
    # OPERATORS
    IN_OPERATORS = {"INC_OPERATOR", "DEC_OPERATOR"}
    FT_OPERATORS = {"GREATER_OPERATOR", "LESSER_OPERATOR"}
    BT_OPERATORS = {"EQUAL_OPERATOR", "DIFF_OPERATOR"}

    def __init__(self, op, argument, value):
        assert op in self.IN_OPERATORS + self.BT_OPERATORS + self.FT_OPERATORS
        self.operator, self.argument1, self.argument2 = op, argument, value

    def __str__(self):
        if self.operator in self.IN_OPERATORS:
            return f'= {ALLOY_OPERATORS[self.operator]}[{self.argument1},{self.argument2}]'
        if self.operator in self.FT_OPERATORS:
            return f'{ALLOY_OPERATORS[self.operator]}[{self.argument1},{self.argument2}]'
        return f'{ALLOY_OPERATORS[self.operator]} {self.argument2}'

class LanguageTransformer(Transformer):

    def __init__(self, node, text):
        MESSAGE_TOKENS.clear()
        self.node, self.text = node, text

    def property(self, children):
        # CLEAR #
        object = children[0]
        if not children[1] is None:
            conditional = children[1]
            object = IffConditional(pre=object, conditional=conditional)
        return object

    def pattern(self, children):
        pattern = children[0]
        if isinstance(children[0], Event):
            pattern = children[0].event
        return pattern

    def param(self, children):
        type, value = children[0].type, children[0].value
        if type in { "TOPIC" , "MESSAGE" }:
            return type, value
        return type, value[1:]

    def event(self, children):
        return Event(event=children[0])

    def cond(self, children):
        if isinstance(children[0], Token):
            return children[0]
        else:
            return Cond(cond=children[0])

    def message_cond(self, children):
        return MessageCond(token=children[0].value, relation=children[1].type, value=children[2].value)

    def message_token(self, children):
        MESSAGE_TOKENS[children[0].value] = None
        return children[0]

    def binop(self, children):
        return children[0].type

    def condition(self, children):
        if isinstance(children[0], Event):
            return children[0].event
        # Quantifier
        if not children[0] is None: no_quantifier = True
        else: no_quantifier = False
        # Predicate
        if isinstance(children[1], Cond):
            conditional = Conditional(no_quantifier=no_quantifier, predicate=children[1].cond)
        else:
            conditional = Conditional(no_quantifier=no_quantifier, token=children[1])
        return conditional

    def reads(self, children):
        if children[1] is not None:
            token = children[1].value
            if token in MESSAGE_TOKENS.keys() and MESSAGE_TOKENS[token] != None: 
                raise svException(f"Two message tokens with the same value {token}.")
            #### #### ####
            entity = Topic.TOPICS[children[0].value]
            if entity in self.node.non_accessable:
                raise svException(f"Property '{self.text}' failed: Node {self.node.rosname} can not access read object {children[0].value}.")
            self.node.changable_channels.append(entity)
            #### #### ####
            read = Read(entity=children[0].value, token=children[1].value, conditions=children[2])
            MESSAGE_TOKENS[token] = read
        else:
            read = Read(entity=children[0].value, token=None, conditions=None)
        return read

    def implication(self, children):
        return Implication(conditions=children)

    def operation(self, children):
        if children.__len__() == 1:
            conditional, more_disjunctions, disjunctions = None, None, children[0]
        else:
            conditional, more_disjunctions, disjunctions = children[0], children[2], children[1]
        # conditional = None if children.__len__() < 2 else children[0]
        return ReadImplication(conditional=conditional, implication=disjunctions, denial=more_disjunctions, frame_conditions=disjunctions.conditions)

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

    # PUBLISH
    def publishes(self, children):
        if children.__len__() > 3: raise svException("")
        #### #### ####
        entity = Topic.TOPICS[children[0].value]
        if entity in self.node.non_accessable:
            raise svException(f"Property '{self.text}' failed: Node {self.node.rosname} can not access publish object {children[0].value}.")
        self.node.changable_channels.append(entity)
        #### #### ####
        if children[1] is not None: 
            value = children[::-1][0]
            type, value = value.type, value.value
        else: value, type = None, None
        if type == "STATE":
            value = value[1:]
        publish = Publish(entity=children[0].value, value=value, type=type)
        return publish

    # ALTERS
    def alters(self, children):
        if not children.__len__() == 3: raise svException("")
        #### #### ####
        entity = svState.STATES[(children[0].value)[1:]]
        if entity.private and not self.node.secure:
            raise svException(f"Property '{self.text}' failed: Node {self.node.rosname} is public and can not access {state.name}.")
        self.node.changable_variables.append(entity)
        #### #### ####
        relation, value = children[1].type, children[2].value
        alter = Alter(entity=(children[0].value)[1:], relation=relation, value=value)
        return alter

###############################
# === PARSER FROM TRANSFMER ===
###############################
class MultipleConditions(object):

    def __init__(self, conditions):
        self.conditions = conditions
    
    def __alloy__(self):
        return ' and '.join([f'{cond.__alloy__()}' for cond in self.conditions])

class Event(object):

    def __init__(self, event):
        self.event = event

    def __alloy__(self):
        return self.event.__alloy__()

class Cond(object):

    def __init__(self, cond):
        self.cond = cond

    def __alloy__(self):
        return self.cond.__alloy__()

class MessageCond(object):

    def __init__(self, token, relation, value):
        if token not in MESSAGE_TOKENS:
            raise svException(f"Token {token} was not initiated.")
        self.token, self.relation, self.value = token, relation, value

    def __alloy__(self, no_quantifier):
        channel, quantifier  = MESSAGE_TOKENS[self.token].object, 'no' if no_quantifier else ''
        channel.message_type.values.add(self.value)
        # if self.value not in channel.message_type.value.values: raise svException(f"Channel {channel.signature} value {self.value} does not exist!")
        value = self.value
        if self.relation == 'EQUAL_OPERATOR': 
            return f"{quantifier} {self.token} = {value}"
        if self.relation == 'DIFF_OPERATOR':
            return f"{quantifier} {self.token} != {value}"
        assert channel.message_type.isint
        return f"{quantifier} {ALLOY_OPERATORS[self.relation]}[{self.token}, {value}]"

class Conditional(object):

    def __init__(self, no_quantifier, token=None, predicate=None):
        self.no_quantifier, self.token, self.predicate = no_quantifier, token, predicate
            
    def __alloy__(self):
        if self.ispredicate:
            return self.predicate.__alloy__(no_quantifier=self.no_quantifier)
        else:
            quantifier = 'no' if self.no_quantifier else ''
            return f'{quantifier} {self.token.value[1:]}[t]'

    @property
    def ispredicate(self):
        return bool(self.predicate is not None)

class Implication(object):
    
    def __init__(self, conditions):
        self.conditions = conditions
    
    def __alloy__(self):
        return '\n\t\t'.join([f'{cond.__alloy__()}' for cond in self.conditions])

class ReadImplication(object):

    def __init__(self, conditional, implication, frame_conditions, denial=None):
        self.conditional, self.implication, self.denial, self.frame_conditions = conditional, implication, denial, frame_conditions
    
    def __alloy__(self):
        if self.conditional is None:
            return f'{self.implication.__alloy__()}'
        if self.denial is None:
            return f'{self.conditional.__alloy__()} implies {{ {self.implication.__alloy__()} }}'
        return f'{self.conditional.__alloy__()} implies {{ {self.implication.__alloy__()} }} else {{ {self.denial.__alloy__()} {self.resolve_frame_conditions() }}}'

    def resolve_frame_conditions(self):
        denial = self.denial.conditions
        frame  = list(map(lambda c : c.object, list(map(lambda cond : cond.conditions, self.frame_conditions.conditions))))
        denial = list(map(lambda c : c.object, list(map(lambda cond : cond.conditions, denial.conditions)))) 
        _str_ = ""
        for f in frame:
            if f not in denial:
                if isinstance(f, Topic):
                    _str_ += f" and t.inbox'[{f.signature}] = t.inbox[{f.signature}]"
                if isinstance(f, svState):
                    _str_ += f" and t.{f.name.lower()}' = t.{f.name.lower()}"
        return _str_

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

class Read(object):

    def __init__(self, entity, token, conditions):
        try:
            channel = Topic.TOPICS[entity]
        except AttributeError as e:
            raise svException(f"Channel {entity} does not exist!")
        self.entity, self.object, self.conditions, self.token = entity, channel, conditions, token

    def __alloy__(self):
        channel = self.object
        if self.token:
            return f"let m = first[t.inbox[{channel.signature}]] {{\n\t\t" + self.conditions.__alloy__() + f"\n\t}}" + f"\n\tt.inbox'[{channel.signature}] = rest[t.inbox[{channel.signature}]] "
        return f"t.inbox'[{channel.signature}] = rest[t.inbox[{channel.signature}]]"

class Publish(object):

    def __init__(self, entity, value=None, type=None):
        try:
            channel = Topic.TOPICS[entity]
            # if type == "VALUE":
            #    if value not in ( channel.message_type.value.values + list(MESSAGE_TOKENS.keys())): raise svException(f"Channel value {value} does not exist!")
        except AttributeError as e : raise svException(f'{e}')
        self.entity, self.object, self.value, self.type = entity, channel, value, type

    def __alloy__(self, no_quantifier=None):
        # CONDITION
        if isinstance(no_quantifier, bool):
            quantifier = 'no' if no_quantifier else 'some'
            if self.value is None:
                return f'{quantifier} t.inbox[{self.object.signature}]'
            elif self.value not in list(MESSAGE_TOKENS.keys()):
                self.object.message_type.values.add(self.value)
            isstate = 't.' if self.type == "STATE" else ''
            if not no_quantifier: return f"{isstate}{self.value} in t.inbox[{self.object.signature}].elems"
            else: return f"{isstate}{self.value} not in t.inbox[{self.object.signature}].elems"
        # EVENT
        else:
            channel = self.object
            if self.value is None:
                return f"some m : Message | t.inbox'[{channel.signature}] = add[t.inbox[{channel.signature}], m]"
            if self.type == "VALUE":
                if self.value in MESSAGE_TOKENS.keys():
                    read = MESSAGE_TOKENS[self.value].object
                    if not bool(channel.message_type == read.message_type):
                        raise svException(f"Channel {channel.signature} can not use value of {read.signature} through {self.value}, as they have different message types.")
                else:
                    channel.message_type.values.add(self.value)
                return f"t.inbox'[{channel.signature}] = add[t.inbox[{channel.signature}], {self.value}]"
            return f"t.inbox'[{channel.signature}] = add[t.inbox[{channel.signature}], t.{self.value}']"

class Alter(object):

    def __init__(self, entity, relation, value):
        try:
            state = svState.STATES[entity]
            if value not in ( state.values + list(MESSAGE_TOKENS.keys()) ): raise svException(f"State value {value} does not exist!")
        except AttributeError as e : raise svException(f'{e}')
        self.entity, self.object, self.relation, self.value = entity, state, relation, value

    def __alloy__(self, no_quantifier=None):
        # CONDITION
        if isinstance(no_quantifier, bool):
            quantifier = 'no' if no_quantifier else ''
            if self.relation == 'EQUAL_OPERATOR': 
                    return f"{quantifier} t.{self.object.name.lower()} = {self.value}"
            assert self.object.isint
            return f"{quantifier} {ALLOY_OPERATORS[self.relation]}[t.{self.object.name.lower()}, {self.value}]"
        else:
            value, state = self.value, self.object
            if self.relation == 'EQUAL_OPERATOR': 
                return f"t.{state.name.lower()}' = {value}"
            assert state.isint
            return f"t.{state.name.lower()}' = {ALLOY_OPERATORS[self.relation]}[t.{state.name.lower()}, {value}]"

class IfConditional(object):

    def __init__(self, pre, conditional):
        self.pre, self.conditional = pre, conditional

    def __alloy__(self):
        return f'({self.conditional.__alloy__()}) implies ({self.pre.__alloy__()})'

class IffConditional(object):

    def __init__(self, pre, conditional):
        self.pre, self.conditional = pre, conditional

    def __alloy__(self):
        return f'({self.pre.__alloy__()}) iff ({self.conditional.__alloy__()})'