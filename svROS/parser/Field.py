import hashlib

class Field:

  def __init__(self, namespace):
    self.id = int(hashlib.sha256(namespace.encode('utf-8')).hexdigest(), 16) % 10**8

  def __str__(self):
    # print

#from dataclasses import dataclass
#@dataclass
#class Point:
#x: float
#y: float
#z: float = 0.0