# from ttk import *
from Tkinter import *
import ttk
from PIL import Image, ImageTk  # need to install: sudo apt-get install python-imaging-tk
from Module import *
from Connection import *
from config_message_pb2 import *
import eventlet  # need to install: $:sudo pip install eventlet
from pygazebo import *  #need to install: $: sudo pip install pygazebo
from gztopic import *
import pdb

window_width = 800
window_height = 520
Border_width = 20
Border_hieht = 40
PI = 3.1415926

class Example(Frame):
  
    def __init__(self, parent):
        Frame.__init__(self, parent)   
         
        self.parent = parent
        self.ModuleList = []
        self.ConnectionList = []
        self.modelname = StringVar()
        self.modelname.set('Module_0')
        self.InsertMethod = StringVar()
        self.InsertMethod.set('Connection')
        self.connectedmodelvar = StringVar()
        self.Node1 = IntVar()
        self.Node1.set(3)
        self.Node2 = IntVar()
        self.Node2.set(3)
        self.X_coor = DoubleVar()
        self.Y_coor = DoubleVar()
        self.Z_coor = DoubleVar()
        self.C_dis = DoubleVar()
        self.C_dis.set(0)
        self.a_dis = DoubleVar()
        self.a_dis.set(0)
        self.modellist = StringVar()
        self.adjacentnode = StringVar()
        self.communicator = GzCommunicator()
        self.communicator.StartCommunicator()
        # self.newconnection = gztopic.CoonectionEstablish()
        # pdb.set_trace()
        # self.manager = Manager()
        self.ServerConnected = 1
        # try:
        # pdb.set_trace()
        # self.publisher = self.manager.advertise('/','config_message.msgs.ConfigMessage')  # 'config_message.msgs.ConfigMessage' /gazebo/Configuration/configSubscriber
        # self.publisher.wait_for_listener()
        # except ValueError:
        #   self.ServerConnected = 0
        self.initUI()
        
    def initUI(self):
      
        self.parent.title("Configuration Generator")
        # self.style = Style()
        # self.style.theme_use("default")
        
        # frame1 = Frame(self, relief=RAISED, borderwidth=1)
        # frame1.pack(fill=BOTH, expand=1)
        
        self.pack(fill=BOTH, expand=1)
        # okButton = Button(self, text="OK")
        # okButton.pack(side=RIGHT)
        n = ttk.Notebook(self)
        f1 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED,); # first page, which would get widgets gridded into it
        f2 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED,); # second page
        n.add(f1, text='New Module')
        n.add(f2, text='Modify Old Module')
        n.pack()

        # --------------- Close Button ------------------------------
        closeButton = Button(f1, text="Close")
        closeButton["command"] = self.CloseWindow
        closeButton.place(x = window_width-Border_width-5, y = window_height-Border_hieht-5, anchor = SE)
        saveButton = Button(f1, text="Save")
        saveButton.place(x = window_width-Border_width-65, y = window_height-Border_hieht-5, anchor = SE)
        closeButton2 = Button(f2, text="Close")
        closeButton2["command"] = self.CloseWindow
        closeButton2.place(x = window_width-Border_width-5, y = window_height-Border_hieht-5, anchor = SE)
        saveButton2 = Button(f2, text="Save")
        saveButton2.place(x = window_width-Border_width-65, y = window_height-Border_hieht-5, anchor = SE)

        #---------------- Model Name ---------------------------------
        label = Label(f1, text='ModelName: ')
        label.place(x = 10, y = 5)
        name = Entry(f1, textvariable=self.modelname)
        name.place(x = 90, y = 5)

        #---------------- Radio selection -------------------------
        self.Rel_pos = Radiobutton(f1, text='Connection', variable=self.InsertMethod, value='Connection', command = self.DisableXYZInput)
        self.Abs_pos = Radiobutton(f1, text='Position', variable=self.InsertMethod, value='Position', command = self.EnableXYZInput)
        self.Abs_pos.select()  # this line doesn't work, haven't figure out why
        self.Rel_pos.place(x = 10, y = 30)
        self.Abs_pos.place(relx = 0.6, y = 30)

        #--------------- Connect to -------------------------------
        label2 = Label(f1, text='Connect to:')
        label2.place(x = 10, y = 55)
        self.connectmodel = ttk.Combobox(f1, textvariable=self.connectedmodelvar) #, command = self.checkConnectivity
        self.connectmodel['values'] = ()
        self.connectmodel.bind('<<ComboboxSelected>>',self.checkConnectivity)
        self.connectmodel.place(x = 100, y = 55)

        #--------------- Connected face of inserting module -------
        label2 = Label(f1, text='Face of New Module')
        label2.place(x = 60, y = 85)
        bard = Image.open("SmallSmores.png")
        bardejov = ImageTk.PhotoImage(bard)
        label3 = Label(f1, image=bardejov)
        label3.image = bardejov
        label3.place(x=90, y=140)
        Back_face = Radiobutton(f1, text='Back Face', variable=self.Node1, value=0)
        left_face = Radiobutton(f1, text='Left Face', variable=self.Node1, value=1)
        right_face = Radiobutton(f1, text='Right Face', variable=self.Node1, value=2)
        front_face = Radiobutton(f1, text='Front Face', variable=self.Node1, value=3)
        front_face.select()
        Back_face.place(x= 120, y = 120,anchor = CENTER)
        front_face.place(x= 120, y = 250,anchor = CENTER)
        right_face.place(x= 40, y = 200,anchor = CENTER)
        left_face.place(x= 230, y = 200,anchor = CENTER)

        #--------------- Connected face of existing module -------
        label4 = Label(f1, text='Face of Existing Module')
        label4.place(x = 360, y = 85)
        label5 = Label(f1, image=bardejov)
        label5.image = bardejov
        label5.place(x=380, y=140)
        self.Back_face2 = Radiobutton(f1, text='Back Face', variable=self.Node2, value=0)
        self.left_face2 = Radiobutton(f1, text='Left Face', variable=self.Node2, value=1)
        self.right_face2 = Radiobutton(f1, text='Right Face', variable=self.Node2, value=2)
        self.front_face2 = Radiobutton(f1, text='Front Face', variable=self.Node2, value=3)
        self.front_face2.select()
        self.Back_face2.place(x= 410, y = 120,anchor = CENTER)
        self.front_face2.place(x= 410, y = 250,anchor = CENTER)
        self.right_face2.place(x= 320, y = 200,anchor = CENTER)
        self.left_face2.place(x= 520, y = 200,anchor = CENTER)

        #---------------- Position Entries -------------------------
        label6 = Label(f1, text='X: ')
        label6.place(x = 610, y = 50)
        self.x_coor = Entry(f1, textvariable=self.X_coor,width=10,state=NORMAL)
        self.x_coor.place(x = 610, y = 75)

        label7 = Label(f1, text='Y: ')
        label7.place(x = 610, y = 100)
        self.y_coor = Entry(f1, textvariable=self.Y_coor,width=10,state=NORMAL)
        self.y_coor.place(x = 610, y = 125)

        label8 = Label(f1, text='Z: ')
        label8.place(x = 610, y = 150)
        self.z_coor = Entry(f1, textvariable=self.Z_coor,width=10,state=NORMAL)
        self.z_coor.place(x = 610, y = 175)

        #--------------- Joint Angle Setting -----------------------
        label9 = Label(f1, text='Joint Angle Bending ')
        label9.place(x = 40, y = 280)
        self.Joint3 = Scale(f1, from_=-90, to=90, orient=HORIZONTAL,length = 150, resolution = 90)
        self.Joint3.place(x = 100, y = 330, anchor = CENTER)
        label10 = Label(f1, text='Joint Angle Left Wheel ')
        label10.place(x = 210, y = 280)
        self.Joint1 = Scale(f1, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 90)
        self.Joint1.place(x = 280, y = 330, anchor = CENTER)

        label11 = Label(f1, text='Joint Angle Right Wheel ')
        label11.place(x = 30, y = 360)
        self.Joint2 = Scale(f1, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 90)
        self.Joint2.place(x = 100, y = 410, anchor = CENTER)
        label12 = Label(f1, text='Joint Angle Front Wheel ')
        label12.place(x = 210, y = 360)
        self.Joint0 = Scale(f1, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 90)
        self.Joint0.place(x = 280, y = 410, anchor = CENTER)

        #--------------- Face Displacement ------------------------
        label13 = Label(f1, text='Distance offset ')
        label13.place(x = 380, y = 280)
        doffste = Entry(f1, textvariable=self.C_dis,width=10)
        doffste.place(x = 380, y = 310)

        label14 = Label(f1, text='Angle offset ')
        label14.place(x = 380, y = 360)
        aoffste = Entry(f1, textvariable=self.a_dis,width=10)
        aoffste.place(x = 380, y = 390)

        #--------------- Model Orientation ------------------------
        label15 = Label(f1, text='Model Orientation ')
        label15.place(x = 610, y = 210)

        label16 = Label(f1, text='Row ')
        label16.place(x = 610, y = 240)
        self.row = Scale(f1, from_=0, to=360, orient=HORIZONTAL,length = 100, resolution = 90)
        self.row.place(x = 660, y = 280, anchor = CENTER)
        label17 = Label(f1, text='Pitch ')
        label17.place(x = 610, y = 310)
        self.pitch = Scale(f1, from_=0, to=360, orient=HORIZONTAL,length = 100, resolution = 90)
        self.pitch.place(x = 660, y = 350, anchor = CENTER)
        label18 = Label(f1, text='Yaw ')
        label18.place(x = 610, y = 380)
        self.yaw = Scale(f1, from_=0, to=360, orient=HORIZONTAL,length = 100, resolution = 90)
        self.yaw.place(x = 660, y = 420, anchor = CENTER)

        #-------------- Select Model -----------------------------
        label19 = Label(f2, text='Select Model ')
        label19.place(x = 10, y = 10)
        self.modelselect = ttk.Combobox(f2, textvariable=self.modellist)
        self.modelselect['values'] = ()
        self.modelselect.place(x = 95, y = 10)

        #---------------- Joint Angle Setting -------------------
        lf = ttk.Labelframe(f2, text='Joint Angle Modification ', width = 350, height = 180)
        lf.place(x = 10, y = 50)
        # label24 = Label(f2, text='Joint Angle Modification ')
        # label24.place(x = 10, y = 40)
        label20 = Label(lf, text='Joint Angle Bending ')
        label20.place(x = 20, y = 10)
        Joint_0 = Scale(lf, from_=-90, to=90, orient=HORIZONTAL,length = 150)
        Joint_0.place(x = 80, y = 50, anchor = CENTER)
        label21 = Label(lf, text='Joint Angle Left Wheel ')
        label21.place(x = 190, y = 10)
        Joint_1 = Scale(lf, from_=0, to=360, orient=HORIZONTAL,length = 150)
        Joint_1.place(x = 260, y = 50, anchor = CENTER)

        label22 = Label(lf, text='Joint Angle Right Wheel ')
        label22.place(x = 20, y = 80)
        Joint_2 = Scale(lf, from_=0, to=360, orient=HORIZONTAL,length = 150)
        Joint_2.place(x = 80, y = 120, anchor = CENTER)
        label23 = Label(lf, text='Joint Angle Front Wheel ')
        label23.place(x = 190, y = 80)
        Joint_3 = Scale(lf, from_=0, to=360, orient=HORIZONTAL,length = 150)
        Joint_3.place(x = 260, y = 120, anchor = CENTER)

        #---------------- Delete Model ----------------------------
        DeleteButton = Button(f2, text="Delete")
        # InsertButton["command"] = self.quit
        DeleteButton["fg"]   = "red"
        DeleteButton.place(x = 5, y = window_height-Border_hieht-5, anchor = SW)

        #---------------- Add More Connections --------------------
        more_conn = ttk.Labelframe(f2, text='Add More Connection ', width = 220, height = 100)
        more_conn.place(x = 400, y = 50)

        label24 = Label(more_conn, text='Select Possible Connection ')
        label24.place(x = 10, y = 10)
        nodeselect = ttk.Combobox(more_conn, textvariable=self.adjacentnode)
        nodeselect['values'] = ('USA', 'Canada', 'Australia')
        nodeselect.place(x = 10, y = 40)

        #---------------- Insert Model -------------------------------
        InsertButton = Button(f1, text="Insert")
        InsertButton["command"] = self.InsertModel
        InsertButton.place(x = 5, y = window_height-Border_hieht-5, anchor = SW)

        #---------------- Here is a test -----------------------------
        # newmessage = ConfigMessage()
        # newmessage.ModelName = "hello"
        # for i in xrange(6):
        #   newmessage.ModelPosition.append(0)
        # for i in xrange(4):
        #   newmessage.JointAngles.append(0)
        # print "The Messgae is: ",newmessage.ModelName
        # self.publisher.publish(newmessage)

    def InsertModel(self,*args):
      print "frob called with {} arguments".format(len(args))
      if self.InsertMethod.get() == "Position" :
        print "Method is position"
        module_position = (self.X_coor.get(),self.Y_coor.get(),self.Z_coor.get(),degree2rad(self.row.get()),degree2rad(self.pitch.get()),degree2rad(self.yaw.get()))
        print "Position tuple is ",module_position
        module_jointangle = (degree2rad(self.Joint0.get()),degree2rad(self.Joint1.get()),degree2rad(self.Joint2.get()),degree2rad(self.Joint3.get()))
        print "Joint angle tuple is ",module_jointangle
        new_module = Module(self.modelname.get(),module_position,module_jointangle)
        self.ModuleList.append(new_module)
        if self.ServerConnected == 1:
          self.PublishMessage(self.ModuleList[-1])
      else:
        print "Method is Connection"
        module_position = self.CalculatePosition()
        module_jointangle = (degree2rad(self.Joint0.get()),degree2rad(self.Joint1.get()),degree2rad(self.Joint2.get()),degree2rad(self.Joint3.get()))
        new_module = Module(self.modelname.get(),module_position,module_jointangle)
        self.ModuleList.append(new_module)
        print "Connected module name",self.connectedmodelvar.get()
        new_connection = Connection(self.modelname.get(),self.connectedmodelvar.get(),self.Node1.get(),self.Node2.get(),self.C_dis.get(),self.a_dis.get())
        self.ConnectionList.append(new_connection)
        self.ModuleList[-1].connection(self.Node1.get(),self.ConnectionList[-1])
        theOtherModule = self.findModule(self.connectedmodelvar.get())
        theOtherModule.connection(self.Node2.get(),self.ConnectionList[-1])
        if self.ServerConnected == 1:
          self.PublishMessage(self.ModuleList[-1])

      self.updateModuleList()
      self.nameIncrement()
      self.Rel_pos.select()
      self.DisableXYZInput()
      print "Combox value", self.connectmodel.get()
      if self.connectmodel.get() != '':
        self.checkConnectivity()

    def EnableXYZInput(self,*args):
      self.x_coor["state"] = NORMAL
      self.y_coor["state"] = NORMAL
      self.z_coor["state"] = NORMAL

    def DisableXYZInput(self,*args):
      self.x_coor["state"] = DISABLED
      self.y_coor["state"] = DISABLED
      self.z_coor["state"] = DISABLED

    def updateModuleList(self):
      a_model_list = []
      for eachmodel in self.ModuleList:
        a_model_list.append(eachmodel.ModelName)
      self.modelselect['values'] = tuple(a_model_list)
      self.connectmodel['values'] = tuple(a_model_list)

    def  nameIncrement(self):
      self.modelname.set('Module_'+str(len(self.ModuleList)))

    # Put the position calculation code here
    def CalculatePosition(self):
      return (0.0,0.0,0.0,0.0,0.0,0.0)

    def findModule(self,modelname):
      for eachmodule in self.ModuleList:
        if eachmodule.ModelName == modelname:
          return eachmodule

    def checkConnectivity(self,*args):
      # print "Check the connectivity"
      themodule = self.findModule(self.connectedmodelvar.get())
      if len(themodule.nodes[0]) != 0:
        self.Back_face2["state"] = DISABLED
      else:
        self.Back_face2["state"] = NORMAL
        self.Back_face2.select()
      if len(themodule.nodes[1]) != 0:
        self.left_face2["state"] = DISABLED
      else:
        self.left_face2["state"] = NORMAL
        self.left_face2.select()
      if len(themodule.nodes[2]) != 0:
        self.right_face2["state"] = DISABLED
      else:
        self.right_face2["state"] = NORMAL
        self.right_face2.select()
      if len(themodule.nodes[3]) != 0:
        self.front_face2["state"] = DISABLED
      else:
        self.front_face2["state"] = NORMAL
        self.front_face2.select()

    def PublishMessage(self,amodule):
      newmessage = ConfigMessage()
      newmessage.ModelName = amodule.ModelName
      for i in xrange(6):
        newmessage.ModelPosition.append(amodule.Position[i])
      print "Coor info",newmessage.ModelPosition
      for i in xrange(4):
        newmessage.JointAngles.append(amodule.JointAngle[i])
      # print "Listeners are ",self.publisher.showlisteners()
      # self.newconnection.sendData(newmessage)
      # self.publisher.publish(newmessage)
      self.communicator.publish(newmessage)
      # eventlet.sleep(1.0)
      print "Information published"

    def CloseWindow(self):
      self.communicator.stop()
      self.quit()

def degree2rad(angle):
  return angle/180.0*PI
                     
def main():
  
    root = Tk()
    root.geometry(str(window_width)+"x"+str(window_height))
    app = Example(root)
    root.mainloop()  


if __name__ == '__main__':
    main()  