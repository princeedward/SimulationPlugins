class Module:
  def __init__(self, modelname,jointangle):
    self.ModelName = modelname
    try:
      self.JointAngle = jointangle
    except ValueError:
      pass
    self.Group = 1
    self.Speeds = [0,0,0]