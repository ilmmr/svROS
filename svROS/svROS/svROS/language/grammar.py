# Copyright © 2022 Luís Ribeiro
# Grammar for svLanguage.

from tools.InfoHandler import svException, svWarning
GRAMMAR = f"""

    axiom : REQUIRES_TOKEN condition 
          | READS_TOKEN reads 
          | PUBLISHES_TOKEN publishes 
          | ALTERS_TOKEN alters

    condition  : [condition AND_OPERATOR] cond
    cond       : ( NO_OPERATOR | SOME_OPERATOR ) ( TOPIC | PREDICATE | evaluation )
    evaluation : ( TOPIC | STATE ) EQUAL_OPERATOR VALUE

    reads       : reads_only | reads_perf
    reads_only  : TOPIC
    reads_perf  : TOPIC "then" "{{" disjunction "}}"

    disjunction : [disjunction OR_OPERATOR] conjunction
    conjunction : [conjunction AND_OPERATOR] read_condition

    read_condition   : ["m =" VALUE CONSEQUENCE_OPERATOR] read_consequence
    read_consequence : ( TOPIC | STATE ) ( EQUAL_OPERATOR | INC_OPERATOR | DEC_OPERATOR ) VALUE
    
    publishes : TOPIC ("{{" "m =" VALUE "}}")

    alters : [alters AND_OPERATOR] alters_condition
    alters_condition : STATE ( EQUAL_OPERATOR | INC_OPERATOR | DEC_OPERATOR ) VALUE

    REQUIRES_TOKEN  : "requires"
    READS_TOKEN     : "reads"
    PUBLISHES_TOKEN : "publishes"
    ALTERS_TOKEN    : "alters"

    NO_OPERATOR   : "no" | "not"
    SOME_OPERATOR : "some" | "exists"

    OR_OPERATOR : "or"  | "+"
    AND_OPERATOR  : "and" | "&"
    CONSEQUENCE_OPERATOR : "implies" | "=>"
    
    EQUAL_OPERATOR : "eql" | "="
    INC_OPERATOR   : "add" | "+="
    DEC_OPERATOR   : "rmv" | "-="

    VALUE     : /(?!=>)[a-zA-Z0-9_\/\-.\:]+/
    TOPIC     : /(?!\s)[a-zA-Z0-9_\/\-.\:]+/
    STATE     : /(?!\s)\$[a-zA-Z0-9_\/\-.\:]+/
    PREDICATE : /(?!\s)\?[a-zA-Z0-9_\/\-.\:]+/

    %import common.WS
    %ignore WS
"""

from lark import Lark, tree, Token, Transformer
from lark.exceptions import UnexpectedCharacters, UnexpectedToken
class GrammarParser(object):
    """
        Grammar Main Parser
    """
    GRAMMAR = f'{GRAMMAR}'

    @classmethod
    def parse(cls, text=''):
        if text == '': return
        grammar, parser = cls.GRAMMAR, Lark(cls.GRAMMAR, parser="lalr", start='axiom', transformer=LanguageTransformer())
        # PARSE!
        try:
            text = parser.parse(text)
        except (UnexpectedToken, UnexpectedCharacters, SyntaxError) as e:
            raise svException(f'Failed to parse property {text}: {e}')

###############################
# === LANGUAGE TRANSFORMER  ===
###############################
class LanguageTransformer(Transformer):

    def axiom(self, children):
        return children

    # CONDITIONAL
    def evaluation_cond(self, children):
        if not children.__len__() == 3: raise svException("")
        predicate, entity, value = children[0].type, children[0].value, children[2].value
        return predicate, entity, value

    def cond(self, children):
        if not children.__len__() == 2: raise svException("")
        quantifier, predicate = children[0].type, children[1]
        if isinstance(predicate, Token):
            predicate, entity, value = children[1].type, children[1].value, None
        else:
            predicate, entity, value = LanguageTransformer.evaluation_cond(self, children[1].children)
        return Conditional(quantifier=quantifier, type=predicate, entity=entity, value=value)

    # READS
    def reads_only(self, children):
        return Read(entity=children[0].value, conditions=None, readonly=True)

    # PUBLISH
    def publishes(self, children):
        if children.__len__() > 3: raise svException("")
        entity = children[0].value
        if children.__len__() > 1: value = children[::-1][0]
        else: value = None
        return Publish(entity=entity, value=value)

    # ALTERS
    def alters_condition(self, children):
        if not children.__len__() == 3: raise svException("")
        entity, relation, value = (children[0].value)[1:], children[1].type, children[2].value
        return Alter(entity=entity, relation=relation, value=value)

class Conditional(object):

    def __init__(self, quantifier, type, entity, value=None):
        self.quantifier, self.type, self.value = quantifier, type, value
        if self.type in {'PREDICATE', 'STATE'}: self.entity = entity[1:]
        else: self.entity = entity

    def __alloy__(self):
        return ''

class Read(object):

    def __init__(self, entity, conditions, readonly=False):
        self.entity, self.conditions = entity, conditions

    def __alloy__(self):
        return ''

class Publish(object):

    def __init__(self, entity, value=None):
        self.entity, self.value = entity, value

    def __alloy__(self):
        return ''

class Alter(object):

    def __init__(self, entity, relation, value):
        self.entity, self.relation, self.value = entity, relation, value

    def __alloy__(self):
        return ''