class GaitEntry:
	def __init__(self,module_id,jointangles,timer,dependency = "",condition = "",special = False, flags = [0,0,0]):
		self.ModuleName = module_id
		self.Joints = jointangles
		self.condition_id = condition
		self.dependency_id = dependency
		self.Timer = timer
		self.AngleFlags = flags
		self.SpecialEntry = special