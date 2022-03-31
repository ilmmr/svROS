
class Configuration:

  def __init__(self):
    self.nodes = set()
    self.topics = set()
    self.fields = set()
    self.values = set()
    self.assumptions = set()

  def addNode(self, node):
    self.nodes.add(node)

  def addTopic(self, topic):
    self.topics.add(topic)

  def fields(self, field):
    self.fields.add(field)

  def values(self, value):
    self.values.add(value)
  
  def __str__(self):
    # print