import hashlib

class Value:

  def __init__(self, namespace):
    self.id = int(hashlib.sha256(namespace.encode('utf-8')).hexdigest(), 16) % 10**8

  def __str__(self):
    # print

