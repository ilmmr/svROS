import hashlib

class Node:

  def __init__(self, namespace, rosname):
    self.id = int(hashlib.sha256(namespace.encode('utf-8')).hexdigest(), 16) % 10**8
    self.rosname = rosname

    self.subscribes = set()
    self.advertises = set()

    self.properties = set()
    # Both inbox will be created later on

  def setAdvertise(self, idTopic):
    self.advertises.add(idTopic)

  def setSubscribe(self, idTopic):
    self.subscribes.add(idTopic)

  def __str__(self):
    # print
