class GaitEntry:
	def __init__(self,module_id,jointangles,gait_increment,timer,flags = [0,0,0]):
		self.ModuleName = module_id
		self.Joints = jointangles
		self.GroupIncr = gait_increment
		self.Timer = timer
		self.AngleFlags = flags