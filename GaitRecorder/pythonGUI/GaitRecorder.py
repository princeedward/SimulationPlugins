from Tkinter import *
import tkFileDialog
import ttk
import xml.etree.ElementTree as ET
# from PIL import Image, ImageTk  # need to install: sudo apt-get install python-imaging-tk
# from Module import *
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
    self.priority = IntVar()
    self.group = IntVar()
    self.elapstime = DoubleVar()
    self.savepath = StringVar()
    self.framename = StringVar()
    self.commandlist = StringVar()
    self.commandhis = StringVar()
    self.selectedcommand = StringVar()
    self.FrameList = []
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
    # self.connectmodel.bind('<<ComboboxSelected>>',self.checkConnectivity)
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
    self.frontModA = Radiobutton(JointModSec, text='Angle (deg)', variable=self.frontmode, value=0)
    self.frontModS = Radiobutton(JointModSec, text='Speed (RPM)', variable=self.frontmode, value=1)
    self.frontModA.select()
    self.frontModA.place(x= 220, y = 40,anchor = CENTER)
    self.frontModS.place(x= 220, y = 80,anchor = CENTER)
    frontangle = Entry(JointModSec, textvariable=self.front_angle, width = 18)
    frontangle.place(x = 280, y = 30)
    frontspeed = Scale(JointModSec, from_=-15, to=15, orient=HORIZONTAL,length = 150, resolution = 1)
    frontspeed.place(x = 280, y = 50)
    label4 = Label(JointModSec, text='Wheels: ')
    label4.place(x = 10, y = 110)
    label5 = Label(JointModSec, text='Left ')
    label5.place(x = 150, y = 110)
    label6 = Label(JointModSec, text='Right ')
    label6.place(x = 300, y = 110)
    self.WheelModA = Radiobutton(JointModSec, text='Angle (deg)', variable=self.wheelmode, value=0)
    self.WheelModS = Radiobutton(JointModSec, text='Speed (RPM)', variable=self.wheelmode, value=1)
    self.WheelModA.select()
    self.WheelModA.place(x= 10, y = 140)
    self.WheelModS.place(x= 10, y = 180)
    leftangle = Entry(JointModSec, textvariable=self.left_angle, width = 15)
    leftangle.place(x = 120, y = 140)
    leftspeed = Scale(JointModSec, from_=-15, to=15, orient=HORIZONTAL,length = 120, resolution = 1)
    leftspeed.place(x = 120, y = 160)
    rightangle = Entry(JointModSec, textvariable=self.left_angle, width = 15)
    rightangle.place(x = 280, y = 140)
    rightspeed = Scale(JointModSec, from_=-15, to=15, orient=HORIZONTAL,length = 120, resolution = 1)
    rightspeed.place(x = 280, y = 160)

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
    IncGroup = Button(ExtraInfoSec, text="+")
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
    CommandRec = Listbox(f1, width=35, height=24,listvariable = self.commandlist) #listvariable = self.commandlist
    # self.commandlist.set(("one", "two", "three", "four")) 
    Commandscroller = Scrollbar(f1, command=CommandRec.yview)
    CommandRec.config(yscrollcommand=Commandscroller.set)
    Commandscroller.place(x = 750, y = 40, height = 390)
    CommandRec.place(x = 475, y = 40)

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
    # self.Framename.bind('<<ComboboxSelected>>',self.checkConnectivity)
    self.Framename.place(x = 100, y = 5)

    #----------------- Command History --------------------------
    CommandHis = Listbox(f2, width=44, height=24,listvariable = self.commandhis)
    CommandHisScroller = Scrollbar(f2, command=CommandHis.yview)
    CommandHis.config(yscrollcommand=CommandHisScroller.set)
    CommandHisScroller.place(x = 361, y = 40, height = 390)
    CommandHis.place(x = 10, y = 40)

    #----------------- Update Command --------------------------
    UpdateCommandSec = ttk.Labelframe(f2, text='Update Command ', width = 370, height = 90)
    UpdateCommandSec.place(x = 390, y = 40)
    CommandEntry = Entry(UpdateCommandSec, textvariable=self.selectedcommand, width = 42)
    CommandEntry.place(x = 10, y = 5)
    CommandUpdateBtn = Button(UpdateCommandSec, text = "Update")
    CommandUpdateBtn.place(x = 10, y = 35)
    CommandDeleteBtn = Button(UpdateCommandSec, text = "Delete")
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
    Addframe = Button(f1,text = "Save Frame")
    Addframe.place(x = 285, y = window_height-Border_hieht-5, anchor = SW)

    #----------------- Add Current Command ---------------------
    Addcommand = Button(f1,text = "Add Command")
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
    tree = ET.parse(filename)
    root = tree.getroot()
    print "Root is: ",root.tag

def main():
  
  root = Tk()
  root.geometry(str(window_width)+"x"+str(window_height))
  app = App(root)
  root.mainloop()  


if __name__ == '__main__':
  main()  