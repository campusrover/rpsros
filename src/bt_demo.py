import py_trees
import random
import time


class Random(py_trees.Behaviour):
  def __init__(self, name, fraction):
    self.fraction = fraction
    super(Random, self).__init__(name)
  
  def update(self):
    r = random.random()
    succeed = r < self.fraction
    print("Tick %s succeded? %s" % (self.name, succeed))
    if succeed:
      return py_trees.Status.RUNNING
    else:
      return py_trees.Status.FAILURE

def create_tree():
  recover = py_trees.composites.Selector("Recover")
  cons_reset = Random("Conservative Reset", 0.2)
  rotate = Random("Clearing Rotation", 0.2)
  aggressive = Random("Aggressive Reset", 0.2)
  recover.add_child(cons_reset)
  recover.add_child(rotate)
  recover.add_child(aggressive)
  recover.add_child(rotate)
  root = py_trees.composites.Sequence("move_base")
  navigate = Random("navigate",0.6)
  root.add_child(navigate)
  root.add_child(recover)
  return root

if __name__ == "__main__":
  tr = create_tree()
  py_trees.display.render_dot_tree(tr)
  try:
    for _ in range(0,100):
      res = tr.tick_once()
      print(res)
      time.sleep(0.5)
  except KeyboardInterrupt:
    pass

