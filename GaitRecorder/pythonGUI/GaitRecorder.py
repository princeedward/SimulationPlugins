from Tkinter import *
import tkFileDialog
import ttk
import xml.etree.ElementTree as ET
import numpy as np
# from PIL import Image, ImageTk  # need to install: sudo apt-get install python-imaging-tk
from Module import *
from GaitEntry import *
# from Connection import *
# from config_message_pb2 import *
# import eventlet  # need to install: $:sudo pip install eventlet
# from pygazebo import *  #need to install: $: sudo pip install pygazebo
# from gztopic import *
# from SimpleKL import CloseEnough, Connectable
import pdb

window_width = 800
window_height = 520
Border_width = 20
Border_hieht = 40

class App(Frame):
  
  def __init__(self, parent):
    Frame.__init__(self, parent)   
     
    self.parent = parent
    self.modelname = StringVar()
    self.frontmode = IntVar()
    self.wheelmode = IntVar()
    self.front_angle = DoubleVar()
    self.left_angle = DoubleVar()
    self.right_angle = DoubleVar()
    self.priority = IntVar()
    self.group = IntVar()
    self.elapstime = DoubleVar()
    self.savepath = StringVar()
    self.framename = StringVar()
    self.commandlist = StringVar()
    self.commandhis = StringVar()
    self.selectedcommand = StringVar()
    self.FrameList = []
    self.CurrentFrameRec = []
    self.CurrentFrameHis = []
    self.ModuleList = []
    self.currentgroup = 1
    #-------------- File Definition ---------------------------
    # define options for opening or saving a file
    self.file_opt = options = {}
    # options['defaultextension'] = '.txt'
    options['filetypes'] = [('all files', '*'), ('text files', '.txt')]
    options['initialdir'] = '/home/edward/'
    # options['initialfile'] = 'myfile.txt'
    options['parent'] = parent
    options['title'] = 'Open Configuration File'

    self.initUI()
      
  def initUI(self):
    self.parent.title("Gait Table Recorder")
      
    self.pack(fill=BOTH, expand=1)
    # okButton = Button(self, text="OK")
    # okButton.pack(side=RIGHT)
    n = ttk.Notebook(self)
    f1 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED,); # first page, which would get widgets gridded into it
    f2 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED,); # second page
    n.add(f1, text='Record New Gaits')
    n.add(f2, text='Manage Gait Table')
    n.pack()

    # --------------- Close Button ------------------------------
    closeButton = Button(f1, text="Close")
    closeButton["command"] = self.CloseWindow
    closeButton.place(x = window_width-Border_width-5, y = window_height-Border_hieht-5, anchor = SE)
    closeButton2 = Button(f2, text="Close")
    closeButton2["command"] = self.CloseWindow
    closeButton2.place(x = window_width-Border_width-5, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Model Name ---------------------------------
    label = Label(f1, text='Select Model: ')
    label.place(x = 10, y = 5)
    self.name = ttk.Combobox(f1, textvariable=self.modelname) #, command = self.checkConnectivity
    self.name['values'] = ()
    self.name.bind('<<ComboboxSelected>>',self.UpdateJoint)
    self.name.place(x = 100, y = 5)

    #--------------- Joint Angle Modification -------------------
    JointModSec = ttk.Labelframe(f1, text='Joint Angle Update ', width = 450, height = 250)
    JointModSec.place(x = 10, y = 40)

    label2 = Label(JointModSec, text='Bending Joint ')
    label2.place(x = 30, y = 10)
    self.Joint3 = Scale(JointModSec, from_=-90, to=90, orient=HORIZONTAL,length = 150, resolution = 1) #, command = self.ChangeJointAngle)
    # self.Joint_3.bind('<ButtonRelease-1>',self.FindConnectable)
    self.Joint3.place(x = 80, y = 50, anchor = CENTER)
    label3 = Label(JointModSec, text='Front Wheel ')
    label3.place(x = 280, y = 10)
    self.frontModA = Radiobutton(JointModSec, text='Angle (deg)', variable=self.frontmode, value=0, command = self.EnableFrontAngle)
    self.frontModS = Radiobutton(JointModSec, text='Speed (RPM)', variable=self.frontmode, value=1, command = self.EnableFrontSpeed)
    self.frontModA.select()
    self.frontModA.place(x= 220, y = 40,anchor = CENTER)
    self.frontModS.place(x= 220, y = 80,anchor = CENTER)
    self.frontangle = Entry(JointModSec, textvariable=self.front_angle, width = 18)
    self.frontangle.place(x = 280, y = 30)
    self.frontspeed = Scale(JointModSec, from_=-15, to=15, orient=HORIZONTAL,length = 150, resolution = 1, state = DISABLED)
    self.frontspeed.place(x = 280, y = 50)
    label4 = Label(JointModSec, text='Wheels: ')
    label4.place(x = 10, y = 110)
    label5 = Label(JointModSec, text='Left ')
    label5.place(x = 150, y = 110)
    label6 = Label(JointModSec, text='Right ')
    label6.place(x = 300, y = 110)
    self.WheelModA = Radiobutton(JointModSec, text='Angle (deg)', variable=self.wheelmode, value=0, command = self.EnableWheelAngle)
    self.WheelModS = Radiobutton(JointModSec, text='Speed (RPM)', variable=self.wheelmode, value=1, command = self.EnableWheelSpeed)
    self.WheelModA.select()
    self.WheelModA.place(x= 10, y = 140)
    self.WheelModS.place(x= 10, y = 180)
    self.leftangle = Entry(JointModSec, textvariable=self.left_angle, width = 15)
    self.leftangle.place(x = 120, y = 140)
    self.leftspeed = Scale(JointModSec, from_=-15, to=15, orient=HORIZONTAL,length = 120, resolution = 1, state = DISABLED)
    self.leftspeed.place(x = 120, y = 160)
    self.rightangle = Entry(JointModSec, textvariable=self.right_angle, width = 15)
    self.rightangle.place(x = 280, y = 140)
    self.rightspeed = Scale(JointModSec, from_=-15, to=15, orient=HORIZONTAL,length = 120, resolution = 1, state = DISABLED)
    self.rightspeed.place(x = 280, y = 160)

    #---------------- Extra Information -------------------------
    ExtraInfoSec = ttk.Labelframe(f1, text='Extra Information ', width = 450, height = 90)
    ExtraInfoSec.place(x = 10, y = 300)
    label7 = Label(ExtraInfoSec, text='Priority ')
    label7.place(x = 10, y = 10)
    Priority = Entry(ExtraInfoSec, textvariable=self.priority, width = 10)
    Priority.place(x = 10, y = 35)
    label8 = Label(ExtraInfoSec, text='Group ')
    label8.place(x = 150, y = 10)
    Group = Entry(ExtraInfoSec, textvariable=self.group, width = 10)
    Group.place(x = 130, y = 35)
    IncGroup = Button(ExtraInfoSec, text="+", command = self.GroupIncrease)
    IncGroup.place(x = 220, y = 30)
    label9 = Label(ExtraInfoSec, text='Elapsed time ')
    label9.place(x = 300, y = 10)
    ElapsTime = Entry(ExtraInfoSec, textvariable=self.elapstime, width = 10)
    ElapsTime.place(x = 300, y = 35)
    label10 = Label(ExtraInfoSec, text='sec ')
    label10.place(x = 390, y = 35)

    #--------------- Command Records ---------------------------
    label11 = Label(f1, text='Command History in Current Frame ')
    label11.place(x = 475, y = 20)
    CommandRec = Listbox(f1, width=35, height=22,listvariable = self.commandlist) #listvariable = self.commandlist
    # self.commandlist.set() 
    Commandscroller = Scrollbar(f1, command=CommandRec.yview)
    CommandRec.config(yscrollcommand=Commandscroller.set)
    Commandscroller.place(x = 750, y = 40, height = 355)
    CommandRec.place(x = 475, y = 40)

    #---------------- Command Record Update -------------------
    RecModify = Button(f1, text='Modify')
    RecModify.place(x = 475, y = 405)
    RecSave = Button(f1, text='Save')
    RecSave.place(x = 545, y = 405)
    RecDelete = Button(f1, text='Delete')
    RecDelete.place(x = 605, y = 405)

    #---------------- Save Path -------------------------------
    SavePathSec = ttk.Labelframe(f1, text='Save Path ', width = 450, height = 50)
    SavePathSec.place(x = 10, y = 390)
    Savepath = Entry(SavePathSec, textvariable=self.savepath, width = 50)
    Savepath.place(x = 20, y = 3)

    #---------------- Model Name ---------------------------------
    label12 = Label(f2, text='Select Frame: ')
    label12.place(x = 10, y = 5)
    self.Framename = ttk.Combobox(f2, textvariable=self.framename) #, command = self.checkConnectivity
    self.Framename['values'] = ()
    self.Framename.bind('<<ComboboxSelected>>',self.UpdateFrameWindows)
    self.Framename.place(x = 100, y = 5)

    #----------------- Command History --------------------------
    self.CommandHis = Listbox(f2, width=44, height=24,listvariable = self.commandhis)
    self.CommandHis.bind('<<ListboxSelect>>', self.ModifyHistory)
    CommandHisScroller = Scrollbar(f2, command=self.CommandHis.yview)
    self.CommandHis.config(yscrollcommand=CommandHisScroller.set)
    CommandHisScroller.place(x = 361, y = 40, height = 390)
    self.CommandHis.place(x = 10, y = 40)

    #----------------- Update Command --------------------------
    UpdateCommandSec = ttk.Labelframe(f2, text='Update Command ', width = 370, height = 90)
    UpdateCommandSec.place(x = 390, y = 40)
    CommandEntry = Entry(UpdateCommandSec, textvariable=self.selectedcommand, width = 42)
    CommandEntry.place(x = 10, y = 5)
    CommandUpdateBtn = Button(UpdateCommandSec, text = "Update", command = self.UpdateSingleGaitEntry)
    CommandUpdateBtn.place(x = 10, y = 35)
    CommandDeleteBtn = Button(UpdateCommandSec, text = "Delete", command = self.DeleteSingleGait)
    CommandDeleteBtn.place(x = 290, y = 35)

    #---------------- Frame Based Operation --------------------
    FrameOpSec = ttk.Labelframe(f2, text='Frame Based Commands ', width = 370, height = 65)
    FrameOpSec.place(x = 390, y = 140)
    FramePlay = Button(FrameOpSec, text = "Play Current Frame")
    FramePlay.place(x = 5, y = 10)
    FrameDelete = Button(FrameOpSec, text = "Delete All After Current Frame")
    FrameDelete.place(x = 152, y = 10)
    # WarningLabel = Label(FrameOpSec, text = "Warning: Delete current frame will delte all the frames after current frame") # , font={"family":"Times", "size":8, "weight":"BOLD"}
    # WarningLabel.place(x = 10, y = 40)

    #--------------- Play All Button --------------------------
    PlayallButton = Button(f2, text = "Play all the frames")
    PlayallButton.place(x = window_width-Border_width-130, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Open File Button ---------------------------
    Openfile = Button(f1, text = "Open Configuration", command = self.AskOpenFile)
    Openfile.place(x = window_width-Border_width-130, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Save Button --------------------------------
    self.saveButton = Button(f1, text="Save")
    self.saveButton.place(x = window_width-Border_width-65, y = window_height-Border_hieht-5, anchor = SE)
    self.saveButton2 = Button(f2, text="Save")
    self.saveButton2.place(x = window_width-Border_width-65, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Play Frame --------------------------------
    Playframe = Button(f1, text = "Play Frame")
    Playframe.place(x = 125, y = window_height-Border_hieht-5, anchor = SW)

    #---------------- Reset Frame -------------------------------
    Resetframe = Button(f1,text = "Reset")
    Resetframe.place(x = 220, y = window_height-Border_hieht-5, anchor = SW)

    #---------------- Add Frame --------------------------------
    Addframe = Button(f1,text = "Save Frame", command = self.SaveFrame)
    Addframe.place(x = 285, y = window_height-Border_hieht-5, anchor = SW)

    #----------------- Add Current Command ---------------------
    Addcommand = Button(f1,text = "Add Command", command = self.AddGaitTable)
    Addcommand.place(x = 5, y = window_height-Border_hieht-5, anchor = SW)

  def CloseWindow(self):
    # self.communicator.stop()
    self.quit()

  def AskOpenFile(self):
    filename = tkFileDialog.askopenfilename(**self.file_opt)

    # open file on your own
    if filename:
      print "Filename is : ",filename
      # configFile = open(filename, 'r')
      self.ReadInConfiguration(filename)

  def ReadInConfiguration(self, filename):
    self.tree = ET.parse(filename)
    root = self.tree.getroot()
    print "Root is: ",root.tag
    modules = root.find("modules")
    modulenames = []
    for eachmodule in modules.findall('module') :
      modelname = eachmodule.find('name').text
      print "Module name: ",modelname
      jointanglestr = eachmodule.find('joints').text
      print "Joint angle: ",self.StringToTuple(jointanglestr)
      newmodule = Module(modelname, self.StringToTuple(jointanglestr))
      self.ModuleList.append(newmodule)
      modulenames.append(modelname)
    self.name['values'] = tuple(modulenames)

  def StringToTuple(self, anglestring):
    jointangles = []
    while True:
      idx = anglestring.find(" ")
      if idx >=0 :
        jointangles.append(float(anglestring[0:idx]))
        anglestring = anglestring[idx+1:]
      else:
        jointangles.append(float(anglestring))
        break
    return tuple(jointangles)

  def UpdateJoint(self,*args):
    modelname = self.modelname.get()
    moduleObj = self.GetModuleByName(modelname)
    self.Joint3.set(moduleObj.JointAngle[3]/np.pi*180)
    self.front_angle.set(moduleObj.JointAngle[0]/np.pi*180)
    self.left_angle.set(moduleObj.JointAngle[1]/np.pi*180)
    self.right_angle.set(moduleObj.JointAngle[2]/np.pi*180)
    self.group.set(moduleObj.Group)
    self.elapstime.set(0.0)
    self.currentgroup = moduleObj.Group

  def GetModuleByName(self,modelname):
    for eachmodule in self.ModuleList:
      if eachmodule.ModelName == modelname:
        return eachmodule

  def GroupIncrease(self):
    self.group.set(self.group.get()+1)

  def RefreshGaitRecorder(self):
    self.GaitTableList =[]
    for eachgait in self.CurrentFrameRec:
      self.GaitTableList.append(self.GaitObjToStr(eachgait))
    self.commandlist.set(tuple(self.GaitTableList))

  def AddGaitTable(self):
    module_id = self.modelname.get()
    moduleObj = self.GetModuleByName(module_id)
    joints = []
    jointsflags = []
    if self.frontmode.get() == 0:
      joints.append(self.front_angle.get()/180.0*np.pi)
      # moduleObj.JointAngle[0] = self.front_angle.get()/180.0*np.pi
      moduleObj.Speeds[0] = 0
      jointsflags.append(0)
    else:
      joints.append(self.frontspeed.get())
      moduleObj.Speeds[0] = 1
      jointsflags.append(1)
    if self.wheelmode.get() == 0 :
      joints.append(self.left_angle.get()/180.0*np.pi)
      joints.append(self.right_angle.get()/180.0*np.pi)
      # moduleObj.JointAngle[1] = self.left_angle.get()/180.0*np.pi
      # moduleObj.JointAngle[2] = self.right_angle.get()/180.0*np.pi
      moduleObj.Speeds[1] = 0
      moduleObj.Speeds[2] = 0
      jointsflags.append(0)
      jointsflags.append(0)
    else:
      joints.append(self.leftspeed.get())
      joints.append(self.rightspeed.get())
      moduleObj.Speeds[1] = 1
      moduleObj.Speeds[2] = 1
      jointsflags.append(1)
      jointsflags.append(1)
    joints.append(self.Joint3.get()/180.0*np.pi)
    # moduleObj.JointAngle[3] = self.Joint3.get()/180.0*np.pi
    print "Joint angles", joints
    moduleObj.JointAngle = tuple(joints)
    currenttimer = int(self.elapstime.get()*1000)
    groupinc = int(self.group.get()-self.currentgroup)
    moduleObj.Group += groupinc
    self.currentgroup = self.group.get()
    newgaits = GaitEntry(module_id,joints,groupinc,currenttimer,jointsflags)
    self.CurrentFrameRec.append(newgaits)
    self.RefreshGaitRecorder()

  def GaitObjToStr(self,gaittableobj):
    gaitstr = ""
    gaitstr += gaittableobj.ModuleName+" "
    for i in xrange(4):
      gaitstr+= str(gaittableobj.Joints[i])+" "
    gaitstr += str(gaittableobj.GroupIncr)+" "
    gaitstr += str(gaittableobj.Timer)
    return gaitstr

  def SaveFrame(self):
    self.FrameList.append(self.CurrentFrameRec)
    self.CurrentFrameRec = []
    self.RefreshGaitRecorder()
    self.UpdateFrameBox()

  def UpdateFrameBox(self):
    frameliststr = []
    for idx in xrange(len(self.FrameList)):
      frameliststr.append("Frame "+str(idx))
    self.Framename['values'] = tuple(frameliststr)

  def UpdateFrameWindows(self,*args):
    idx = int(self.framename.get()[6:])
    self.CurrentFrameHis = self.FrameList[idx]
    self.GaiTableHisList = []
    for eachgait in self.CurrentFrameHis:
      self.GaiTableHisList.append(self.GaitObjToStr(eachgait))
    self.commandhis.set(tuple(self.GaiTableHisList))

  def EnableFrontAngle(self):
    self.frontangle["state"] = NORMAL
    self.frontspeed["state"] = DISABLED

  def EnableFrontSpeed(self):
    self.frontangle["state"] = DISABLED
    self.frontspeed["state"] = NORMAL

  def EnableWheelAngle(self):
    self.leftangle["state"] = NORMAL
    self.leftspeed["state"] = DISABLED
    self.rightangle["state"] = NORMAL
    self.rightspeed["state"] = DISABLED

  def EnableWheelSpeed(self):
    self.leftangle["state"] = DISABLED
    self.leftspeed["state"] = NORMAL
    self.rightangle["state"] = DISABLED
    self.rightspeed["state"] = NORMAL

  def ModifyHistory(self,*args):
    self.historyidx = int(self.CommandHis.curselection()[0])
    print "Select item: ",self.historyidx
    self.selectedcommand.set(self.GaiTableHisList[self.historyidx])

  def UpdateSingleGaitEntry(self):
    newsinglegait = self.selectedcommand.get()
    self.GaiTableHisList[self.historyidx] = newsinglegait
    updatedgait = self.InterpretGaitString(newsinglegait)
    self.CurrentFrameHis[self.historyidx].ModuleName = updatedgait.ModuleName
    self.CurrentFrameHis[self.historyidx].Joints = updatedgait.Joints
    self.CurrentFrameHis[self.historyidx].GroupIncr = updatedgait.GroupIncr
    self.CurrentFrameHis[self.historyidx].Timer = updatedgait.Timer
    self.UpdateFrameWindows()

  def InterpretGaitString(self,gaitstring):
    idx = gaitstring.find(" ")
    modelname = gaitstring[0:idx]
    gaitstring = gaitstring[idx+1:]
    joints = []
    for i in xrange(4):
      idx = gaitstring.find(" ")
      joints.append(float(gaitstring[0:idx]))
      gaitstring = gaitstring[idx+1:]
    idx = gaitstring.find(" ")
    groupinc = int(gaitstring[0:idx])
    gaitstring = gaitstring[idx+1:]
    timer = int(gaitstring)
    return GaitEntry(modelname,joints,groupinc,timer)

  def DeleteSingleGait(self):
    del self.GaiTableHisList[self.historyidx]
    del self.CurrentFrameHis[self.historyidx]
    if len(self.CurrentFrameHis) == 0:
      idx = int(self.framename.get()[6:])
      del self.FrameList[idx]
      self.framename.set("")
      self.commandhis.set(())
      self.selectedcommand.set("")
    self.UpdateFrameBox()


def main():
  
  root = Tk()
  root.geometry(str(window_width)+"x"+str(window_height))
  app = App(root)
  root.mainloop()  


if __name__ == '__main__':
  main()  