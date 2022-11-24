# Copyright © 2022 Luís Ribeiro
# Grammar for Nodes.

from .svInfo import svException, svWarning
GRAMMAR = f"""

    property : pattern [ "iff" formula ]

    pattern  : "requires"  formula 
             | event

    event    : publishes
             | updates
             | reads

    reads       : "reads"     TOPIC [ topics ]
    publishes   : "publishes" TOPIC [ topics ]
    updates     : "updates"   STATE evaluate
    topics      : "as" message_token "where" implication
                | evaluate

    implication : operation ( ";" operation )*
    operation   : "if" formula "then" "{{" formula "}}" [ "else" "{{" operation "}}" ]
                | formula

    formula      : conditional
    conditional  : [ conditional AND_OPERATOR ] conjunction 
    conjunction  : [ conjunction OR_OPERATOR ] condition
    condition    : [ NO_OPERATOR ] cond | event

    cond        : TOPIC [ evaluate ]
                | STATE [ evaluate ]
                | PREDICATE
                | MESSAGE evaluate

    evaluate     : binop ( VALUE | STATE )
    message_cond : MESSAGE ( EQUAL_OPERATOR | DIFF_OPERATOR | GREATER_OPERATOR | LESSER_OPERATOR ) VALUE

    message_token: MESSAGE
    param       : TOPIC | STATE | PREDICATE | MESSAGE
    binop       : EQUAL_OPERATOR | DIFF_OPERATOR | GREATER_OPERATOR | LESSER_OPERATOR | INC_OPERATOR | DEC_OPERATOR | GT_EQ | LS_EQ
    AS_TOKEN    : "as"

    AND_OPERATOR    : "and" | "&&"
    OR_OPERATOR     : "or"  | "||"

    NO_OPERATOR     : "no" | "not"
    SOME_OPERATOR   : "some" | "exists"
    
    EQUAL_OPERATOR   : "="
    GREATER_OPERATOR : ">"
    GT_EQ            : ">="
    LESSER_OPERATOR  : "<"
    LS_EQ            : "<="
    INC_OPERATOR     : "+="
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
from .svData import svTopic, svState, NonNumeric

import random
import string
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
    "LESSER_OPERATOR": "lt",
    "INC_OPERATOR": "plus",
    "DEC_OPERATOR": "minus",
    "GT_EQ": "gte",
    "LS_EQ": "lte"
}

MESSAGE_TOKENS = {}
        
###############################
# === LANGUAGE TRANSFORMER  ===
###############################
class LanguageTransformer(Transformer):

    def __init__(self, node, text):
        MESSAGE_TOKENS.clear()
        self.predicate, self.node, self.text = node, node.node, text

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
        return Cond(entity=children[0], cond=children[1])


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
        return Conditional(no_quantifier=no_quantifier, predicate=children[1])

    def evaluate(self, children):
        return Evaluate(binop=children[0], value=children[1])

    def topics(self, children):
        if children.__len__() > 1:
            token = children[0].value
            declaration = Declaration(token=token, conditions=children[1])
            if token in MESSAGE_TOKENS.keys() and MESSAGE_TOKENS[token] != None: 
                raise svException(f"Two message tokens with the same value {token}.")
            MESSAGE_TOKENS[token] = declaration.parent
            return declaration
        else:
            # EVALUATE
            return children[0]

    # READ
    def reads(self, children):
        topic = children[0].value
        entity = svTopic.TOPICS[topic]
        #### #### ####
        if entity in self.node.non_accessable:
            raise svException(f"Property '{self.text}' failed: Node {self.node.rosname} can not access read object {topic}.")
        self.predicate.changable_channels.append(entity)
        #### #### ####
        return Read(entity=topic, read=children[1])

    # PUBLISH
    def publishes(self, children):
        topic = children[0].value
        entity = svTopic.TOPICS[topic]
        #### #### ####
        if entity in self.node.non_accessable:
            raise svException(f"Property '{self.text}' failed: Node {self.node.rosname} can not access publish object {topic}.")
        self.predicate.changable_channels.append(entity)
        #### #### ####
        return Publish(entity=topic, publish=children[1])

    # UPDATE
    def updates(self, children):
        state = children[0].value
        #### #### ####
        entity = svState.STATES[state[1:]]
        if entity.private and not self.node.secure:
            raise svException(f"Property '{self.text}' failed: Node {self.node.rosname} is public and can not access {entity.name}.")
        self.predicate.changable_variables.append(entity)
        #### #### ####
        return Update(entity=state[1:], update=children[1])

    def implication(self, children):
        return Implication(conditions=children)

    def operation(self, children):
        if children.__len__() == 1:
            conditional, more_disjunctions, disjunctions = None, None, children[0]
        else:
            conditional, more_disjunctions, disjunctions = children[0], children[2], children[1]
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

    def __init__(self, entity, cond):
        self.entity, self.cond = entity, cond
        if not self.ismessage: return 

    def __alloy__(self, no_quantifier):
        quantifier = 'no' if no_quantifier else ''
        if self.cond:
            return f'{quantifier} {self.cond.__alloy__(entity=self.entity)}'
        else:
            if self.entity.type == "PREDICATE":
                return f'{quantifier} {self.entity.value[1:]}[t]'
            if self.entity.type == "TOPIC":
                quantifier = 'no' if no_quantifier else 'some'
                entity = svTopic.TOPICS[self.entity.value]
                return f'{quantifier} t.inbox[{entity.signature}]'
            return None

    @property
    def ismessage(self):
        if self.entity.type == "MESSAGE":
            token = self.entity.value
            if token not in MESSAGE_TOKENS:
                raise svException(f"Token {token} was not initiated.")
        return True

class Evaluate(object):

    def __init__(self, binop, value):
        self.binop = binop
        # ADD NON_NUMERIC
        if value.type == "VALUE":
            value = value.value
            if value not in list(MESSAGE_TOKENS.keys()):
                if not value.lstrip("-").isdigit():
                    NonNumeric(name=value)
        else:
            value = f"t.{value.value[1:]}'"
        self.value = value

    
    def __alloy__(self, entity, action="evaluate"):
        if action == "evaluate":
            return self.evaluate(entity)
        elif action == "publish":
            assert isinstance(entity, svTopic)
            return self.publish(entity)
        elif action == "state":
            assert isinstance(entity, svState)
            return self.state(entity)
        else:
            return None 
        
    def evaluate(self, entity):
        if entity.type == "MESSAGE":
            token = entity.value
            # isint = True if entity.message_type.isint else False
            # entity.message_type.values.add(self.value)
            return self.operation(binop=self.binop, signature=token, value=self.value, isint=True)
        elif entity.type == "TOPIC":
            # isint = True if entity.message_type.isint else False
            # if self.value not in list(MESSAGE_TOKENS.keys()):
            #    entity.message_type.values.add(self.value)
            return self.operation(binop=self.binop, signature=f"first[t.inbox[{entity.signature}]]", value=self.value, isint=True)
        elif entity.type == "STATE":
            isint = True if entity.isint else False
            if self.value not in list(MESSAGE_TOKENS.keys()):
                if isint and not self.value.lstrip("-").isdigit():
                    raise svException(f"Variable value is not a number but variable is numeric!")
                entity.values.add(self.value)
            return self.operation(binop=self.binop, signature=f"t.{entity.name.lower()}", value=self.value, isint=isint)
        else:
            return None

    def publish(self, entity):
        binop = self.binop
        if binop in {"EQUAL_OPERATOR", "DIFF_OPERATOR", "LESSER_OPERATOR", "GREATER_OPERATOR"}:
            _not = "" if binop != "DIFF_OPERATOR" else "not"
            return f"( some msg : Message & {_not} {self.value} | t.inbox'[{entity.signature}] = add[t.inbox[{entity.signature}], msg] )"
        if binop == "LS_EQ":
            return f"( some msg : Message & {self.value}.*prev | t.inbox'[{entity.signature}] = add[t.inbox[{entity.signature}], msg] )"
        if binop == "GT_EQ":
            return f"( some msg : Message & {self.value}.*next | t.inbox'[{entity.signature}] = add[t.inbox[{entity.signature}], msg] )"
        if binop == "GREATER_OPERATOR":
            return f"( some msg : Message & {self.value}.prevs | t.inbox'[{entity.signature}] = add[t.inbox[{entity.signature}], msg] )"
        if binop == "LESSER_OPERATOR":
            return f"( some msg : Message & {self.value}.nexts | t.inbox'[{entity.signature}] = add[t.inbox[{entity.signature}], msg] )"
        return ''    

    def state(self, entity):
        isint = True if entity.isint else False
        if self.value not in list(MESSAGE_TOKENS.keys()):
            if isint and not self.value.lstrip("-").isdigit():
                raise svException(f"Variable value is not a number but variable is numeric!")
            entity.values.add(self.value)
        return self.operation(binop=self.binop, signature=f"t.{entity.name.lower()}'", prev=f"t.{entity.name.lower()}", value=self.value, isint=isint)

    @staticmethod
    def operation(binop, signature, value, isint, prefix="", sufix="", prev=""):
        if binop in {"EQUAL_OPERATOR", "DIFF_OPERATOR"}:
            return f"{prefix} {signature} {ALLOY_OPERATORS[binop]} {value} {sufix}"
        assert isint
        if binop in {"INC_OPERATOR", "DEC_OPERATOR"}:
            if prev:
                return f"{signature} = {ALLOY_OPERATORS[binop]}[{prev}, {value}]"
        return f"{prefix} {ALLOY_OPERATORS[binop]}[{signature}, {value}] {sufix}"

class Declaration(object):

    def __init__(self, token, conditions):
        self.token, self.conditions, self.parent = token, conditions, None

class Conditional(object):

    def __init__(self, no_quantifier, predicate=None):
        self.no_quantifier, self.predicate = no_quantifier, predicate
            
    def __alloy__(self):
        return self.predicate.__alloy__(no_quantifier=self.no_quantifier)

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
            return f'{self.conditional.__alloy__()} implies {{ {self.implication.__alloy__()} }} else {{ {self.resolve_frame_conditions()} }}'
        return f'{self.conditional.__alloy__()} implies {{ {self.implication.__alloy__()} }} else {{ {self.denial.__alloy__()} {self.resolve_frame_conditions()} }}'

    def resolve_frame_conditions(self):
        if isinstance(self.denial, ReadImplication): return ''
        denial = self.denial.conditions if self.denial else []
        frame  = list(map(lambda c : c.object, sum(list(map(lambda cond : cond.conditions, self.frame_conditions)), [])))
        if denial:
            denial = list(map(lambda c : c.object, list(map(lambda cond : cond.conditions, denial.conditions)))) 
        _str_ = []
        for f in frame:
            if f not in denial:
                if isinstance(f, svTopic):
                    _str_.append(f"t.inbox'[{f.signature}] = t.inbox[{f.signature}]")
                if isinstance(f, svState):
                    _str_.append(f" and t.{f.name.lower()}' = t.{f.name.lower()}")
        return ' and '.join(_str_)

class Disjunction(object):
    
    def __init__(self, conditions):
        self.conditions = conditions
        self.str = self.__alloy__()
    
    def __alloy__(self):
        return '(' + ' and '.join([f'{cond.__alloy__()}' for cond in self.conditions]) + ')'

class Conjunction(object):

    def __init__(self, conditions):
        self.conditions = conditions
    
    def __alloy__(self):
        return '(' + ' or '.join([f'{cond.__alloy__()}' for cond in self.conditions]) + ')'

class Read(object):

    def __init__(self, entity, read):
        try:
            topic = svTopic.TOPICS[entity]
        except AttributeError as e:
            raise svException(f"Topic {entity} does not exist!")
        self.entity, self.object = entity, topic
        if isinstance(read, Declaration):
            read.parent = self 
        self.read = read
        self.str = self.__alloy__()

    def __alloy__(self):
        if isinstance(self.read, Declaration):
            return f"let {self.read.token} = first[t.inbox[{self.object.signature}]] {{\n\t\t{self.read.conditions.__alloy__()}\n\t}}\n\tt.inbox'[{self.object.signature}] = rest[t.inbox[{self.object.signature}]]"
        elif isinstance(self.read, Evaluate):
            return f"{self.read.__alloy__()}\n\tt.inbox'[{self.object.signature}] = rest[t.inbox[{self.object.signature}]]"
        else:
            return f"t.inbox'[{self.object.signature}] = rest[t.inbox[{self.object.signature}]]"

class Publish(object):
    
    def __init__(self, entity, publish):
        try:
            topic = svTopic.TOPICS[entity]
        except AttributeError as e:
            raise svException(f"Topic {entity} does not exist!")
        self.entity, self.object = entity, topic
        if isinstance(publish, Declaration):
            publish.parent = self 
        self.publish = publish
        self.str = self.__alloy__()

    def __alloy__(self):
        if isinstance(self.publish, Declaration):
            return f"some {self.publish.token} : Message {{\n\t\t{self.publish.conditions.__alloy__()}\n\t\tt.inbox'[{self.object.signature}] = add[t.inbox[{self.object.signature}], {self.publish.token}]\n\t}}"
        elif isinstance(self.publish, Evaluate):
            return f"{self.publish.__alloy__(entity=self.object, action='publish')}"
        else:
            return f"( some message : Message | t.inbox'[{self.object.signature}] = add[t.inbox[{self.object.signature}], message] )"

class Update(object):

    def __init__(self, entity, update):
        try:
            state = svState.STATES[entity]
        except AttributeError as e : 
            raise svException(f'Variable {entity} does not exist!')
        self.entity, self.object, self.update = entity, state, update
        self.str = self.__alloy__()

    def __alloy__(self):
        return f"{self.update.__alloy__(entity=self.object, action='state')}"

class IffConditional(object):

    def __init__(self, pre, conditional):
        self.pre, self.conditional = pre, conditional

    def __alloy__(self):
        return f'({self.pre.__alloy__()}) iff ({self.conditional.__alloy__()})'